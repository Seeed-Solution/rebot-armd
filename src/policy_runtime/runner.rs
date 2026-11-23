//! Async policy runner, prepared policy pipeline, and supervisor `PolicyRun` state.

use std::path::Path;
use std::sync::Arc;
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::api::sandbox::validate_pack_id;
use crate::model::{ArmConfig, JointLimit, PolicyObservation};
use crate::motion::MotionError;
use crate::safety::SafetyState;
use crate::supervisor::{ArmError, ArmSupervisor, SupervisorInner};

use super::manifest::{
    validate_chunking_horizon, PolicySnapshot, StrictPolicyContract, VideoEmbeddingSpec,
    VisionStalePolicy,
};
use super::release_gate::VerifiedReleaseGate;
use super::units::{
    denormalize_action_into, normalize_state_into, policy_units_to_rad, rad_to_policy_units,
    JointPermutation,
};
use super::vision::{
    embedding_hub_for_supervisor, evaluate_vision_frame, extract_top1_target,
    DetectorTargetIdentity, EmbeddingSampleDecision, HubCommitGuard, PolicyVisionSource,
    VideoEmbeddingHub, VisionFeatureCollector, VisionStepDecision,
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

/// Largest age (ms) of the source observation behind a cached action-chunk entry:
/// `max_policy_input_age_ms(fps) + replan_every * period_ms`.
pub fn max_cached_chunk_source_age_ms(fps: f64, replan_every: usize) -> u64 {
    let base = max_policy_input_age_ms(fps);
    if !fps.is_finite() || fps <= 0.0 {
        return base;
    }
    let horizon_ms = ((replan_every as f64) * (1000.0 / fps)).round().max(0.0) as u64;
    base.saturating_add(horizon_ms)
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

/// Fixed-size provenance metadata for the observation that produced an inferred action chunk.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChunkSource {
    /// Monotonic instant when the joint observation was sampled.
    pub joint_captured_at: Instant,
    /// Driver-reported joint feedback age (ms) at `joint_captured_at`, when available.
    pub joint_base_age_ms: Option<u64>,
    /// Monotonic instant when the detector vision frame was captured, if any.
    pub vision_captured_at: Option<Instant>,
    /// Detector vision model generation locked at inference time, if any.
    pub vision_generation: Option<u64>,
    /// Fixed-size identity of the detector target this chunk was computed for, if any.
    pub vision_target: Option<DetectorTargetIdentity>,
    /// Video-embedding inference ticket captured for this chunk, if any.
    pub embedding_ticket: Option<InferenceTicket>,
    /// Monotonic sequence id assigned when this chunk was pushed into `ChunkRing`.
    pub infer_seq: u64,
}

impl ChunkSource {
    pub fn from_joint(joint_captured_at: Instant, joint_base_age_ms: Option<u64>) -> Self {
        Self {
            joint_captured_at,
            joint_base_age_ms,
            vision_captured_at: None,
            vision_generation: None,
            vision_target: None,
            embedding_ticket: None,
            infer_seq: 0,
        }
    }

    pub fn joint_age_ms(&self, now: Instant) -> u64 {
        let elapsed = now
            .saturating_duration_since(self.joint_captured_at)
            .as_millis()
            .min(u64::MAX as u128) as u64;
        self.joint_base_age_ms.unwrap_or(0).saturating_add(elapsed)
    }

    pub fn vision_age_ms(&self, now: Instant) -> Option<u64> {
        self.vision_captured_at.map(|t| {
            now.saturating_duration_since(t)
                .as_millis()
                .min(u64::MAX as u128) as u64
        })
    }
}

/// Execution-state snapshot used to clear the action-chunk ring on stop/E-stop,
/// fault, lease change/revocation, epoch change, or shadow↔live mode switch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChunkRunContext {
    pub mode: PolicyMode,
    pub epoch_id: u64,
    pub lease_generation: Option<u64>,
    pub active_ok: bool,
}

/// Freshness and ticket-validity bounds enforced against cached chunk provenance before
/// executing a cached action.
#[derive(Debug, Clone)]
pub struct ChunkFreshnessLimits {
    pub now: Instant,
    pub max_source_joint_age_ms: u64,
    pub max_source_vision_age_ms: Option<u64>,
    pub horizon_ms: u64,
    pub vision_generation: Option<u64>,
    pub vision_target: Option<DetectorTargetIdentity>,
    pub live_ticket_ctx: Option<TicketVerificationContext>,
}

/// Parameters for the single [`PreparedPolicy::commit_gate`] check immediately before servo submission.
#[derive(Debug, Clone)]
pub struct CommitGateParams {
    pub max_fresh_joint_age_ms: u64,
    pub max_cached_joint_age_ms: u64,
    pub max_fresh_vision_age_ms: Option<u64>,
    pub max_cached_vision_age_ms: Option<u64>,
    pub horizon_ms: u64,
    pub vision_generation: Option<u64>,
    pub vision_target: Option<DetectorTargetIdentity>,
    pub live_ticket_ctx: Option<TicketVerificationContext>,
}

/// Decision returned by [`PreparedPolicy::commit_gate`].
#[derive(Debug, Clone, PartialEq)]
pub enum CommitGateDecision {
    Submit {
        target: Vec<f32>,
        commit_period: u64,
    },
    Skip {
        commit_period: u64,
    },
}

/// Preallocated circular buffer of future action-chunk slots and fixed-storage provenance.
///
/// Allocates `chunk_size` slots × up to `chunk_size` overlapping predictions × `action_dim`
/// floats (plus fixed-size provenance tables) once in [`ChunkRing::new`]
/// (`PreparedPolicy::from_backend`). No heap allocations occur while pushing chunks,
/// pruning stale/invalidated provenance, discarding skipped periods, or popping actions.
struct ChunkRing {
    chunk_size: usize,
    max_preds: usize,
    action_dim: usize,
    head: usize,
    len: usize,
    /// Control period index corresponding to logical offset 0 when `len > 0`.
    front_period: Option<u64>,
    /// Monotonic sequence number assigned to each pushed chunk.
    next_infer_seq: u64,
    /// Number of valid predictions stored in each physical slot (`len == chunk_size`, or 0 when `chunk_size <= 1`).
    counts: Vec<usize>,
    /// Flat storage laid out as `[physical_slot][pred_idx][dim]`.
    data: Vec<f32>,
    /// Fixed-capacity ring of source metadata for at most `max_preds` concurrently active chunks.
    sources: Vec<Option<ChunkSource>>,
    next_source_slot: usize,
    /// Parallel index into `sources` laid out as `[physical_slot][pred_idx]`.
    source_ids: Vec<usize>,
}

impl ChunkRing {
    fn new(chunk_size: usize, action_dim: usize, temporal_ensemble: bool) -> Self {
        if chunk_size <= 1 {
            return Self {
                chunk_size,
                max_preds: 0,
                action_dim,
                head: 0,
                len: 0,
                front_period: None,
                next_infer_seq: 1,
                counts: Vec::new(),
                data: Vec::new(),
                sources: Vec::new(),
                next_source_slot: 0,
                source_ids: Vec::new(),
            };
        }
        let max_preds = if temporal_ensemble { chunk_size } else { 1 };
        Self {
            chunk_size,
            max_preds,
            action_dim,
            head: 0,
            len: 0,
            front_period: None,
            next_infer_seq: 1,
            counts: vec![0; chunk_size],
            data: vec![0.0; chunk_size * max_preds * action_dim],
            sources: vec![None; max_preds],
            next_source_slot: 0,
            source_ids: vec![0; chunk_size * max_preds],
        }
    }

    #[inline]
    fn is_empty(&self) -> bool {
        self.len == 0
    }

    #[inline]
    fn len(&self) -> usize {
        self.len
    }

    #[inline]
    fn front_period(&self) -> Option<u64> {
        (self.len > 0).then_some(self.front_period).flatten()
    }

    #[inline]
    fn phys_slot(&self, logical_offset: usize) -> usize {
        debug_assert!(logical_offset < self.chunk_size);
        (self.head + logical_offset) % self.chunk_size
    }

    #[inline]
    fn pred_slot(&self, phys: usize, pred_idx: usize) -> usize {
        phys * self.max_preds + pred_idx
    }

    #[inline]
    fn pred_offset(&self, phys: usize, pred_idx: usize) -> usize {
        self.pred_slot(phys, pred_idx) * self.action_dim
    }

    fn clear(&mut self) {
        for offset in 0..self.len {
            let phys = self.phys_slot(offset);
            self.counts[phys] = 0;
        }
        self.sources.fill(None);
        self.head = 0;
        self.len = 0;
        self.front_period = None;
        self.next_source_slot = 0;
    }

    fn discard_front(&mut self, count: usize) {
        let drop_n = count.min(self.len);
        for _ in 0..drop_n {
            self.counts[self.head] = 0;
            self.head = (self.head + 1) % self.chunk_size;
            self.len -= 1;
        }
        if self.len == 0 {
            self.front_period = None;
        } else if let Some(fp) = self.front_period {
            self.front_period = Some(fp.saturating_add(drop_n as u64));
        }
    }

    fn advance_to_period(&mut self, target_period: u64) {
        if let Some(fp) = self.front_period() {
            if target_period > fp {
                let drop_n = (target_period - fp).min(usize::MAX as u64) as usize;
                self.discard_front(drop_n);
            }
        }
    }

    /// Enforce source provenance invalidation and freshness limits across cached predictions.
    ///
    /// - Any hard invalidation (revoked/mismatched embedding ticket identity or invalidation
    ///   generation, detector model-generation mismatch, or detector target lost/replaced)
    ///   clears the entire ring immediately.
    /// - Any prediction whose source observation exceeded `max_source_joint_age_ms`,
    ///   `max_source_vision_age_ms`, or its embedding window horizon is dropped in place;
    ///   if the front slot loses all predictions, the whole ring is cleared to force a replan.
    fn prune_invalid_or_stale(&mut self, limits: &ChunkFreshnessLimits) -> bool {
        if self.len == 0 {
            return false;
        }
        // Pass 1: hard invalidation clears the entire ring.
        for offset in 0..self.len {
            let phys = self.phys_slot(offset);
            let count = self.counts[phys];
            for pred_idx in 0..count {
                let src_id = self.source_ids[self.pred_slot(phys, pred_idx)];
                let Some(src) = self.sources[src_id] else {
                    self.clear();
                    return true;
                };
                if limits.vision_generation.is_some()
                    && src.vision_generation != limits.vision_generation
                {
                    self.clear();
                    return true;
                }
                if let Some(ref cur_target) = limits.vision_target {
                    if !src.vision_target.is_some_and(|s| s.matches(cur_target)) {
                        self.clear();
                        return true;
                    }
                }
                if let (Some(ticket), Some(live_ctx)) =
                    (src.embedding_ticket, limits.live_ticket_ctx.as_ref())
                {
                    if let Err(v) = ticket.verify_identity_and_horizon(live_ctx, limits.horizon_ms)
                    {
                        if !matches!(
                            v,
                            TicketViolation::WindowNewestExpired { .. }
                                | TicketViolation::WindowOldestExpired { .. }
                        ) {
                            self.clear();
                            return true;
                        }
                    }
                }
            }
        }

        // Pass 2: drop predictions whose source observation age exceeded the freshness limit.
        let dim = self.action_dim;
        let mut dropped_any = false;
        for offset in 0..self.len {
            let phys = self.phys_slot(offset);
            let count = self.counts[phys];
            let mut write_idx = 0usize;
            for read_idx in 0..count {
                let src_id = self.source_ids[self.pred_slot(phys, read_idx)];
                let fresh = self.sources[src_id].is_some_and(|src| {
                    if src.joint_age_ms(limits.now) > limits.max_source_joint_age_ms {
                        return false;
                    }
                    if let Some(max_v) = limits.max_source_vision_age_ms {
                        if src.vision_age_ms(limits.now).is_some_and(|age| age > max_v) {
                            return false;
                        }
                    }
                    if let (Some(ticket), Some(live_ctx)) =
                        (src.embedding_ticket, limits.live_ticket_ctx.as_ref())
                    {
                        if ticket
                            .verify_identity_and_horizon(live_ctx, limits.horizon_ms)
                            .is_err()
                        {
                            return false;
                        }
                    }
                    true
                });
                if fresh {
                    if write_idx != read_idx {
                        let src_off = self.pred_offset(phys, read_idx);
                        let dst_off = self.pred_offset(phys, write_idx);
                        self.data.copy_within(src_off..src_off + dim, dst_off);
                        let dst_slot = self.pred_slot(phys, write_idx);
                        self.source_ids[dst_slot] = src_id;
                    }
                    write_idx += 1;
                } else {
                    dropped_any = true;
                }
            }
            self.counts[phys] = write_idx;
        }

        if self.counts[self.head] == 0 {
            self.clear();
            return true;
        }
        while self.len > 0 && self.counts[self.phys_slot(self.len - 1)] == 0 {
            self.len -= 1;
        }
        dropped_any
    }

    fn push_chunk(
        &mut self,
        flat_chunk: &[f32],
        chunk_start_period: Option<u64>,
        mut source: ChunkSource,
        clear_existing: bool,
    ) -> u64 {
        debug_assert_eq!(flat_chunk.len(), self.chunk_size * self.action_dim);
        let seq = self.next_infer_seq;
        self.next_infer_seq = self.next_infer_seq.wrapping_add(1).max(1);
        source.infer_seq = seq;
        if clear_existing || self.len == 0 {
            self.clear();
            self.front_period = chunk_start_period;
        } else if let Some(target_p) = chunk_start_period {
            self.advance_to_period(target_p);
            if self.len > 0 && self.front_period != Some(target_p) {
                self.clear();
            }
            if self.len == 0 {
                self.front_period = Some(target_p);
            }
        }
        let src_slot = self.next_source_slot;
        self.sources[src_slot] = Some(source);
        self.next_source_slot = (self.next_source_slot + 1) % self.max_preds;
        for offset in 0..self.chunk_size {
            let phys = self.phys_slot(offset);
            if offset >= self.len {
                self.counts[phys] = 0;
            }
            let pred_idx = self.counts[phys];
            debug_assert!(pred_idx < self.max_preds);
            if pred_idx < self.max_preds {
                let dst_off = self.pred_offset(phys, pred_idx);
                let src_off = offset * self.action_dim;
                self.data[dst_off..dst_off + self.action_dim]
                    .copy_from_slice(&flat_chunk[src_off..src_off + self.action_dim]);
                let p_slot = self.pred_slot(phys, pred_idx);
                self.source_ids[p_slot] = src_slot;
                self.counts[phys] = pred_idx + 1;
            }
        }
        self.len = self.chunk_size;
        seq
    }

    fn collect_front_sources(&self, temporal_ensemble_k: Option<f32>, out: &mut Vec<ChunkSource>) {
        out.clear();
        if self.len == 0 {
            return;
        }
        let phys = self.head;
        let count = self.counts[phys];
        if count == 0 {
            return;
        }
        if temporal_ensemble_k.is_some() && count > 1 {
            for age_idx in 0..count {
                let src_id = self.source_ids[self.pred_slot(phys, age_idx)];
                if let Some(src) = self.sources[src_id] {
                    out.push(src);
                }
            }
        } else {
            let src_id = self.source_ids[self.pred_slot(phys, count - 1)];
            if let Some(src) = self.sources[src_id] {
                out.push(src);
            }
        }
    }

    fn peek_front_into(
        &self,
        temporal_ensemble_k: Option<f32>,
        out: &mut [f32],
        used_sources: &mut Vec<ChunkSource>,
    ) -> Result<(), PolicyError> {
        used_sources.clear();
        if self.len == 0 {
            return Err(PolicyError::InferenceFailed(
                "action chunk buffer unexpectedly empty".into(),
            ));
        }
        let phys = self.head;
        let count = self.counts[phys];
        if count == 0 {
            return Err(PolicyError::InferenceFailed(
                "action chunk slot unexpectedly empty".into(),
            ));
        }
        let dim = self.action_dim;
        debug_assert_eq!(out.len(), dim);
        if let Some(k) = temporal_ensemble_k.filter(|_| count > 1) {
            out.fill(0.0);
            let mut w_sum = 0.0f32;
            for age_idx in 0..count {
                let w = (-k * (age_idx as f32)).exp();
                w_sum += w;
                let off = self.pred_offset(phys, age_idx);
                let pred = &self.data[off..off + dim];
                for (dst, &src) in out.iter_mut().zip(pred.iter()) {
                    *dst += w * src;
                }
                let src_id = self.source_ids[self.pred_slot(phys, age_idx)];
                if let Some(src) = self.sources[src_id] {
                    used_sources.push(src);
                }
            }
            if w_sum > 0.0 {
                for v in out.iter_mut() {
                    *v /= w_sum;
                }
            }
        } else {
            let last_idx = count - 1;
            let off = self.pred_offset(phys, last_idx);
            out.copy_from_slice(&self.data[off..off + dim]);
            let src_id = self.source_ids[self.pred_slot(phys, last_idx)];
            if let Some(src) = self.sources[src_id] {
                used_sources.push(src);
            }
        }
        Ok(())
    }

    fn consume_front(&mut self) {
        self.discard_front(1);
    }

    fn pop_front_into(
        &mut self,
        temporal_ensemble_k: Option<f32>,
        out: &mut [f32],
        used_sources: &mut Vec<ChunkSource>,
    ) -> Result<(), PolicyError> {
        self.peek_front_into(temporal_ensemble_k, out, used_sources)?;
        self.consume_front();
        Ok(())
    }

    #[cfg(test)]
    fn storage_snapshot(&self) -> (usize, usize, usize, usize, usize, usize, usize, usize) {
        (
            self.counts.capacity(),
            self.counts.as_ptr() as usize,
            self.data.capacity(),
            self.data.as_ptr() as usize,
            self.sources.capacity(),
            self.sources.as_ptr() as usize,
            self.source_ids.capacity(),
            self.source_ids.as_ptr() as usize,
        )
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
    step_out_norm: Vec<f32>,
    policy_act_units: Vec<f32>,
    arm_target_rad: Vec<f32>,
    periods_since_replan: usize,
    last_replan_period: Option<u64>,
    staged_period: Option<u64>,
    staged_infer_seq: Option<u64>,
    staged_ready: bool,
    chunk_ring: ChunkRing,
    last_step_sources: Vec<ChunkSource>,
    last_run_ctx: Option<ChunkRunContext>,
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
        let expected_out = contract.expected_output_dim();
        if backend.output_dim() != expected_out {
            return Err(PolicyError::DimensionMismatch {
                expected: expected_out,
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
        let chunk_ring = ChunkRing::new(
            contract.chunking.chunk_size,
            dim,
            contract.chunking.temporal_ensemble_k.is_some(),
        );
        let last_step_sources = Vec::with_capacity(chunk_ring.max_preds);
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
            infer_out: vec![0.0; expected_out],
            step_out_norm: vec![0.0; dim],
            policy_act_units: vec![0.0; dim],
            arm_target_rad: vec![0.0; dim],
            periods_since_replan: 0,
            last_replan_period: None,
            staged_period: None,
            staged_infer_seq: None,
            staged_ready: false,
            chunk_ring,
            last_step_sources,
            last_run_ctx: None,
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

    /// Drop all cached action-chunk predictions and source metadata, forcing a fresh
    /// inference on the next step.
    pub fn clear_chunk_buffer(&mut self) {
        self.chunk_ring.clear();
        self.periods_since_replan = 0;
        self.last_replan_period = None;
        self.staged_period = None;
        self.staged_infer_seq = None;
        self.staged_ready = false;
        self.last_step_sources.clear();
    }

    /// Number of future control periods currently buffered in the chunk ring.
    pub fn chunk_buffer_len(&self) -> usize {
        self.chunk_ring.len()
    }

    /// Provenance records of the chunk(s) that contributed to the most recently emitted action.
    pub fn last_step_sources(&self) -> &[ChunkSource] {
        &self.last_step_sources
    }

    /// Clear the whole chunk ring on stop/E-stop, fault, lease change or revocation,
    /// epoch change, or shadow↔live mode switch. Returns `true` if cleared.
    pub fn sync_run_context(&mut self, ctx: ChunkRunContext) -> bool {
        let must_clear = !ctx.active_ok
            || ctx.lease_generation.is_none()
            || self.last_run_ctx.is_some_and(|prev| {
                prev.mode != ctx.mode
                    || prev.epoch_id != ctx.epoch_id
                    || prev.lease_generation != ctx.lease_generation
            });
        self.last_run_ctx = Some(ctx);
        if must_clear {
            self.clear_chunk_buffer();
            return true;
        }
        false
    }

    /// Prune expired or invalidated cached chunks against `limits`. Returns `true` if any
    /// cached prediction or the whole ring was dropped.
    pub fn prune_chunk_buffer(&mut self, limits: &ChunkFreshnessLimits) -> bool {
        let dropped = self.chunk_ring.prune_invalid_or_stale(limits);
        if self.chunk_ring.is_empty() {
            self.periods_since_replan = 0;
            self.last_replan_period = None;
            self.staged_period = None;
            self.staged_infer_seq = None;
            self.staged_ready = false;
        }
        dropped
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
        self.step_infer_with_vision_periods(measured_arm_rad, None, 1)
    }

    /// Run one inference step advancing the action-chunk timeline by `periods_elapsed` control
    /// periods (clamped to at least 1).
    pub fn step_infer_periods(
        &mut self,
        measured_arm_rad: &[f32],
        periods_elapsed: u32,
    ) -> Result<Vec<f32>, PolicyError> {
        self.step_infer_with_vision_periods(measured_arm_rad, None, periods_elapsed)
    }

    /// Run one full inference step, optionally appending pre-extracted vision features after the
    /// normalized proprioception vector.
    pub fn step_infer_with_vision(
        &mut self,
        measured_arm_rad: &[f32],
        vision_features: Option<&[f32]>,
    ) -> Result<Vec<f32>, PolicyError> {
        self.step_infer_with_vision_periods(measured_arm_rad, vision_features, 1)
    }

    /// Run one full inference step, advancing the action-chunk timeline by `periods_elapsed`
    /// control periods since the previous step (clamped to at least 1).
    pub fn step_infer_with_vision_periods(
        &mut self,
        measured_arm_rad: &[f32],
        vision_features: Option<&[f32]>,
        periods_elapsed: u32,
    ) -> Result<Vec<f32>, PolicyError> {
        let source = ChunkSource::from_joint(Instant::now(), None);
        self.step_infer_with_source(measured_arm_rad, vision_features, periods_elapsed, source, None)
    }

    fn prepare_infer_input(
        &mut self,
        measured_arm_rad: &[f32],
        vision_features: Option<&[f32]>,
    ) -> Result<(), PolicyError> {
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
        Ok(())
    }

    /// Run one full inference step with explicit source provenance and optional cached-chunk
    /// freshness/invalidation bounds.
    pub fn step_infer_with_source(
        &mut self,
        measured_arm_rad: &[f32],
        vision_features: Option<&[f32]>,
        periods_elapsed: u32,
        source: ChunkSource,
        freshness: Option<&ChunkFreshnessLimits>,
    ) -> Result<Vec<f32>, PolicyError> {
        let dim = self.contract.action_dim;
        self.prepare_infer_input(measured_arm_rad, vision_features)?;

        let chunking = self.contract.chunking;
        if chunking.chunk_size <= 1 {
            self.last_step_sources.clear();
            self.backend.infer(&self.infer_in, &mut self.infer_out)?;
            for (idx, &v) in self.infer_out.iter().enumerate() {
                if !v.is_finite() {
                    return Err(PolicyError::NonFinite(format!(
                        "model output[{idx}] is non-finite: {v}"
                    )));
                }
            }
            self.step_out_norm.copy_from_slice(&self.infer_out[..dim]);
        } else {
            if let Some(limits) = freshness {
                self.prune_chunk_buffer(limits);
            }
            let elapsed = periods_elapsed.max(1) as usize;
            let skipped = if self.chunk_ring.is_empty() {
                0
            } else {
                elapsed - 1
            };
            let remaining_after_skip = self.chunk_ring.len().saturating_sub(skipped);
            let next_periods_since_replan = self.periods_since_replan.saturating_add(elapsed);
            let need_replan =
                remaining_after_skip == 0 || next_periods_since_replan >= chunking.replan_every;

            if need_replan {
                self.backend.infer(&self.infer_in, &mut self.infer_out)?;
                for (idx, &v) in self.infer_out.iter().enumerate() {
                    if !v.is_finite() {
                        return Err(PolicyError::NonFinite(format!(
                            "model output[{idx}] is non-finite: {v}"
                        )));
                    }
                }
            }

            // Discard entries for skipped control periods before taking or ensembling the current step.
            self.chunk_ring.discard_front(skipped);

            if need_replan {
                self.chunk_ring.push_chunk(
                    &self.infer_out,
                    None,
                    source,
                    chunking.temporal_ensemble_k.is_none(),
                );
                self.periods_since_replan = 0;
            } else {
                self.periods_since_replan = next_periods_since_replan;
            }

            self.chunk_ring.pop_front_into(
                chunking.temporal_ensemble_k,
                &mut self.step_out_norm,
                &mut self.last_step_sources,
            )?;
        }

        self.finalize_step_target()
    }

    /// Off-lock planning stage for chunked policies (`chunk_size > 1`): prunes stale entries,
    /// runs backend inference when due for `sample_period`, and stages the converted candidate
    /// target for `sample_period` without consuming the ring slot. Final slot selection,
    /// freshness, and provenance checks run in [`Self::commit_gate`].
    pub fn plan_chunk_step(
        &mut self,
        measured_arm_rad: &[f32],
        vision_features: Option<&[f32]>,
        sample_period: u64,
        source: ChunkSource,
        freshness: Option<&ChunkFreshnessLimits>,
    ) -> Result<Option<u64>, PolicyError> {
        let chunking = self.contract.chunking;
        debug_assert!(chunking.chunk_size > 1);
        self.prepare_infer_input(measured_arm_rad, vision_features)?;

        if let Some(limits) = freshness {
            self.prune_chunk_buffer(limits);
        }
        self.chunk_ring.advance_to_period(sample_period);

        let need_replan = self.chunk_ring.is_empty()
            || self.chunk_ring.front_period() != Some(sample_period)
            || self.last_replan_period.is_none_or(|rp| {
                sample_period.saturating_sub(rp) >= chunking.replan_every as u64
            });

        let fresh_seq = if need_replan {
            self.backend.infer(&self.infer_in, &mut self.infer_out)?;
            for (idx, &v) in self.infer_out.iter().enumerate() {
                if !v.is_finite() {
                    return Err(PolicyError::NonFinite(format!(
                        "model output[{idx}] is non-finite: {v}"
                    )));
                }
            }
            let seq = self.chunk_ring.push_chunk(
                &self.infer_out,
                Some(sample_period),
                source,
                chunking.temporal_ensemble_k.is_none(),
            );
            self.last_replan_period = Some(sample_period);
            self.periods_since_replan = 0;
            Some(seq)
        } else {
            None
        };

        self.chunk_ring.peek_front_into(
            chunking.temporal_ensemble_k,
            &mut self.step_out_norm,
            &mut self.last_step_sources,
        )?;
        let _ = self.finalize_step_target()?;
        self.staged_period = Some(sample_period);
        self.staged_infer_seq = fresh_seq;
        self.staged_ready = true;
        Ok(fresh_seq)
    }

    fn finalize_step_target(&mut self) -> Result<Vec<f32>, PolicyError> {
        if let (Some(mean), Some(std)) = (
            &self.contract.normalization.action_mean,
            &self.contract.normalization.action_std,
        ) {
            denormalize_action_into(&self.step_out_norm, mean, std, &mut self.policy_act_units)?;
        } else {
            self.policy_act_units.copy_from_slice(&self.step_out_norm);
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

    /// Single final commit gate right before servo submission: reads the monotonic clock itself
    /// and runs [`Self::commit_gate`] with that single reading.
    pub fn commit_gate_now(
        &mut self,
        interval_origin: Instant,
        period_ns: u128,
        last_committed_period: Option<u64>,
        params: &CommitGateParams,
    ) -> Result<(CommitGateDecision, Instant), PolicyError> {
        let now = Instant::now();
        let decision = self.commit_gate(now, interval_origin, period_ns, last_committed_period, params)?;
        Ok((decision, now))
    }

    /// Single final commit gate for chunked policies (`chunk_size > 1`), executed immediately
    /// before submitting to `policy_step_with_vision`:
    ///
    /// (a) Computes `commit_period` from `now` and selects that period's slot (discarding any
    ///     passed slots). If the slot has already been committed/passed or is missing, submits
    ///     nothing.
    /// (b) Checks every source age of the selected slot against `now` (`max_fresh_*` for a
    ///     fresh inference from this tick, `max_cached_*` for cached entries). On expiry, drops
    ///     the whole chunk to force a replan on the next period and submits nothing (without
    ///     failing the run).
    /// (c) Checks provenance (detector model generation, detector target identity, embedding
    ///     ticket identity/invalidation/deadlines) and clears the ring on mismatch.
    pub fn commit_gate(
        &mut self,
        now: Instant,
        interval_origin: Instant,
        period_ns: u128,
        last_committed_period: Option<u64>,
        params: &CommitGateParams,
    ) -> Result<CommitGateDecision, PolicyError> {
        let commit_period = (now
            .saturating_duration_since(interval_origin)
            .as_nanos()
            / period_ns.max(1))
        .min(u64::MAX as u128) as u64;

        // (a) Select the slot for `commit_period`.
        if last_committed_period.is_some_and(|p| commit_period <= p) {
            self.staged_ready = false;
            return Ok(CommitGateDecision::Skip { commit_period });
        }
        let Some(front_p) = self.chunk_ring.front_period() else {
            self.staged_ready = false;
            return Ok(CommitGateDecision::Skip { commit_period });
        };
        if commit_period < front_p {
            self.staged_ready = false;
            return Ok(CommitGateDecision::Skip { commit_period });
        }
        let offset = (commit_period - front_p) as usize;
        if offset >= self.chunk_ring.len() {
            self.clear_chunk_buffer();
            return Ok(CommitGateDecision::Skip { commit_period });
        }
        if offset > 0 {
            self.chunk_ring.discard_front(offset);
            self.staged_ready = false;
        }

        // (b) & (c) Inspect every contributing source of the selected slot against `now`.
        let te_k = self.contract.chunking.temporal_ensemble_k;
        self.chunk_ring
            .collect_front_sources(te_k, &mut self.last_step_sources);
        if self.last_step_sources.is_empty() {
            self.clear_chunk_buffer();
            return Ok(CommitGateDecision::Skip { commit_period });
        }
        let fresh_seq = self.staged_infer_seq;
        for &src in &self.last_step_sources {
            if params.vision_generation.is_some()
                && src.vision_generation != params.vision_generation
            {
                self.clear_chunk_buffer();
                return Ok(CommitGateDecision::Skip { commit_period });
            }
            if let Some(ref cur_target) = params.vision_target {
                if !src.vision_target.is_some_and(|s| s.matches(cur_target)) {
                    self.clear_chunk_buffer();
                    return Ok(CommitGateDecision::Skip { commit_period });
                }
            }
            let is_fresh = fresh_seq.is_some_and(|seq| src.infer_seq == seq);
            let joint_limit = if is_fresh {
                params.max_fresh_joint_age_ms
            } else {
                params.max_cached_joint_age_ms
            };
            if src.joint_age_ms(now) > joint_limit {
                self.clear_chunk_buffer();
                return Ok(CommitGateDecision::Skip { commit_period });
            }
            let vision_limit = if is_fresh {
                params.max_fresh_vision_age_ms
            } else {
                params.max_cached_vision_age_ms
            };
            if let Some(max_v) = vision_limit {
                if src.vision_age_ms(now).is_some_and(|age| age > max_v) {
                    self.clear_chunk_buffer();
                    return Ok(CommitGateDecision::Skip { commit_period });
                }
            }
            if let (Some(ticket), Some(live_ctx)) =
                (src.embedding_ticket, params.live_ticket_ctx.as_ref())
            {
                let ticket_horizon = if is_fresh { 0 } else { params.horizon_ms };
                if ticket
                    .verify_identity_and_horizon(live_ctx, ticket_horizon)
                    .is_err()
                {
                    self.clear_chunk_buffer();
                    return Ok(CommitGateDecision::Skip { commit_period });
                }
            }
        }

        let target = if self.staged_ready && self.staged_period == Some(commit_period) {
            self.chunk_ring.consume_front();
            self.arm_target_rad.clone()
        } else {
            self.chunk_ring.pop_front_into(
                te_k,
                &mut self.step_out_norm,
                &mut self.last_step_sources,
            )?;
            self.finalize_step_target()?
        };
        self.staged_ready = false;
        self.staged_period = None;
        self.staged_infer_seq = None;
        Ok(CommitGateDecision::Submit {
            target,
            commit_period,
        })
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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
        self.verify_identity_and_horizon(live, 0)
    }

    /// Verify all identity, lease, epoch, generation, invalidation, and clock-handshake fields
    /// strictly, allowing the observation window deadlines to cover up to `horizon_ms` of
    /// open-loop chunk playback.
    pub fn verify_identity_and_horizon(
        &self,
        live: &TicketVerificationContext,
        horizon_ms: u64,
    ) -> Result<(), TicketViolation> {
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
        if live.clock_handshake_generation != self.clock_handshake_generation {
            return Err(TicketViolation::ClockHandshakeGenerationMismatch {
                expected: self.clock_handshake_generation,
                actual: live.clock_handshake_generation,
            });
        }
        if live.now_armd_ms > self.clock_handshake_expires_at_ms {
            return Err(TicketViolation::ClockHandshakeExpired {
                now_ms: live.now_armd_ms,
                expires_at_ms: self.clock_handshake_expires_at_ms,
            });
        }
        let newest_limit = self.window_newest_deadline_ms.saturating_add(horizon_ms);
        if live.now_armd_ms > newest_limit {
            return Err(TicketViolation::WindowNewestExpired {
                now_ms: live.now_armd_ms,
                deadline_ms: newest_limit,
            });
        }
        let oldest_limit = self.window_oldest_deadline_ms.saturating_add(horizon_ms);
        if live.now_armd_ms > oldest_limit {
            return Err(TicketViolation::WindowOldestExpired {
                now_ms: live.now_armd_ms,
                deadline_ms: oldest_limit,
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
    if let Err(err) = validate_chunking_horizon(prepared.contract().chunking, clamped_fps) {
        return tokio::spawn(async move {
            let mut inner = supervisor.inner().write().await;
            inner.fail_policy_run(action_id, "INVALID_POLICY", err.to_string());
        });
    }
    let period = Duration::from_secs_f64(1.0 / clamped_fps);
    let is_chunked = prepared.contract().chunking.chunk_size > 1;
    let replan_every = prepared.contract().chunking.replan_every;
    let horizon_ms = ((replan_every as f64) * (1000.0 / clamped_fps))
        .round()
        .max(0.0) as u64;
    let max_fresh_joint_age_ms = max_policy_input_age_ms(clamped_fps);
    let max_source_joint_age_ms = max_cached_chunk_source_age_ms(clamped_fps, replan_every);
    let max_fresh_vision_age_ms = vision_spec.as_ref().map(|s| s.max_staleness_ms);
    let max_source_vision_age_ms = vision_spec
        .as_ref()
        .map(|s| s.max_staleness_ms.saturating_add(horizon_ms));
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

        let interval_origin = tokio::time::Instant::now();
        let interval_origin_std = interval_origin.into_std();
        let mut interval = tokio::time::interval_at(interval_origin, period);
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        let period_ns = period.as_nanos().max(1);
        let mut last_infer_period: Option<u64> = None;
        let mut last_committed_period: Option<u64> = None;

        loop {
            if let Some(dl) = tokio_dl {
                tokio::select! {
                    biased;
                    _ = tokio::time::sleep_until(dl) => {
                        prepared.clear_chunk_buffer();
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
            let cur_period = (step_started_at
                .saturating_duration_since(interval_origin_std)
                .as_nanos()
                / period_ns)
                .min(u64::MAX as u128) as u64;
            if is_chunked && last_committed_period.is_some_and(|p| cur_period <= p) {
                continue;
            }
            if let Some(dl) = deadline {
                if Instant::now() >= dl {
                    prepared.clear_chunk_buffer();
                    let mut inner = supervisor.inner().write().await;
                    inner.finish_policy_run(action_id, "MAX_SECONDS");
                    break;
                }
            }

            let (measured, prop_age_range_ms, captured_at, snap_lease_gen, run_ctx) = {
                let inner = supervisor.inner().read().await;
                let Some(run) = inner.policy_run.as_ref().filter(|r| r.action_id == action_id) else {
                    prepared.clear_chunk_buffer();
                    break;
                };
                let lease_gen = active_run_lease_generation(&inner, action_id);
                let active_ok = !inner.estop_pending()
                    && !matches!(
                        inner.safety.state(),
                        SafetyState::EStop | SafetyState::Fault(_)
                    )
                    && inner.safety.epoch_id() == run.epoch_id;
                let ctx = ChunkRunContext {
                    mode: run.mode,
                    epoch_id: run.epoch_id,
                    lease_generation: lease_gen,
                    active_ok,
                };
                (
                    inner.measured_positions.clone(),
                    inner.policy_input_age_range_ms(),
                    Instant::now(),
                    lease_gen,
                    ctx,
                )
            };
            prepared.sync_run_context(run_ctx);
            let base_input_age_ms = prop_age_range_ms.map(|(_, max_prop)| max_prop);

            let (
                vision_features,
                vision_captured_at,
                vision_frame_id,
                vision_target,
                emb_ticket,
                emb_mapped_ms,
            ) = if let (Some(emb_spec), Some(hub)) = (&video_embedding_spec, &embedding_hub) {
                let Some(lease_gen) = snap_lease_gen else {
                    prepared.clear_chunk_buffer();
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
                            None,
                            Some(ticket),
                            Some(win.newest_mapped_at_ms),
                        )
                    }
                    EmbeddingSampleDecision::Hold => {
                        prepared.clear_chunk_buffer();
                        supervisor.inner().write().await.hold_policy_run(action_id);
                        continue;
                    }
                    EmbeddingSampleDecision::Stop { reason, message } => {
                        prepared.clear_chunk_buffer();
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
                        target_identity,
                    } => (
                        Some(features.to_vec()),
                        frame.as_ref().and_then(|f| f.captured_at),
                        Some(frame_id),
                        Some(target_identity),
                        None,
                        None,
                    ),
                    VisionStepDecision::Hold => {
                        prepared.clear_chunk_buffer();
                        supervisor.inner().write().await.hold_policy_run(action_id);
                        continue;
                    }
                    VisionStepDecision::Stop { reason, message } => {
                        prepared.clear_chunk_buffer();
                        let mut inner = supervisor.inner().write().await;
                        inner.fail_policy_run(action_id, reason, message);
                        break;
                    }
                }
            } else {
                (None, None, None, None, None, None)
            };

            let live_ticket_ctx = embedding_hub.as_ref().map(|hub| {
                let guard = hub.lock_for_commit();
                let live_hub = guard.live_ticket_state();
                TicketVerificationContext {
                    policy_instance_id: live_hub.policy_instance_id,
                    lease_generation: snap_lease_gen,
                    producer_epoch: live_hub.producer_epoch,
                    model_generation: live_hub.model_generation,
                    invalidation_generation: live_hub.invalidation_generation,
                    clock_handshake_generation: live_hub.clock_handshake_generation,
                    now_armd_ms: live_hub.now_armd_ms,
                }
            });
            let chunk_source = ChunkSource {
                joint_captured_at: captured_at,
                joint_base_age_ms: base_input_age_ms,
                vision_captured_at,
                vision_generation: locked_generation,
                vision_target,
                embedding_ticket: emb_ticket,
                infer_seq: 0,
            };
            let freshness_limits = ChunkFreshnessLimits {
                now: Instant::now(),
                max_source_joint_age_ms,
                max_source_vision_age_ms,
                horizon_ms,
                vision_generation: locked_generation,
                vision_target,
                live_ticket_ctx,
            };

            let periods_elapsed = match last_infer_period {
                None => 1u32,
                Some(prev) => cur_period.saturating_sub(prev).clamp(1, u32::MAX as u64) as u32,
            };
            last_infer_period = Some(cur_period);

            let infer_handle = tokio::task::spawn_blocking(move || {
                let t0 = Instant::now();
                let res: Result<Option<Vec<f32>>, PolicyError> = if is_chunked {
                    prepared
                        .plan_chunk_step(
                            &measured,
                            vision_features.as_deref(),
                            cur_period,
                            chunk_source,
                            Some(&freshness_limits),
                        )
                        .map(|_| None)
                } else {
                    prepared
                        .step_infer_with_source(
                            &measured,
                            vision_features.as_deref(),
                            periods_elapsed,
                            chunk_source,
                            Some(&freshness_limits),
                        )
                        .map(Some)
                };
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

            let (single_step_target, infer_us) = match infer_result {
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
                                prepared.clear_chunk_buffer();
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
                                prepared.clear_chunk_buffer();
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
            // `inner`, re-verify current-tick `InferenceTicket`, then run the single
            // `commit_gate_now` immediately before `policy_step_with_vision`.
            let _hub_guard = embedding_hub.as_ref().map(|hub| hub.lock_for_commit());
            let emb_commit_now_ms = if let (Some(ref ticket), Some(ref guard)) = (&emb_ticket, &_hub_guard) {
                match verify_ticket_in_critical_section(&inner, action_id, ticket, guard) {
                    Ok(now_ms) => Some(now_ms),
                    Err(violation) => {
                        prepared.clear_chunk_buffer();
                        inner.fail_policy_run(action_id, violation.code(), violation.message());
                        break;
                    }
                }
            } else {
                None
            };
            let (target, committed_period, input_age_ms, post_vision_age_ms) = if is_chunked {
                let commit_ticket_ctx = _hub_guard.as_ref().map(|guard| {
                    let live_hub = guard.live_ticket_state();
                    TicketVerificationContext {
                        policy_instance_id: live_hub.policy_instance_id,
                        lease_generation: active_run_lease_generation(&inner, action_id),
                        producer_epoch: live_hub.producer_epoch,
                        model_generation: live_hub.model_generation,
                        invalidation_generation: live_hub.invalidation_generation,
                        clock_handshake_generation: live_hub.clock_handshake_generation,
                        now_armd_ms: live_hub.now_armd_ms,
                    }
                });
                let commit_vision_target = if let (Some(spec), Some(col)) = (&vision_spec, &collector) {
                    col.latest()
                        .map(|latest| {
                            extract_top1_target(&latest, spec)
                                .map_or(DetectorTargetIdentity::Absent, |(_, id)| id)
                        })
                        .or(vision_target)
                } else {
                    vision_target
                };
                let gate_params = CommitGateParams {
                    max_fresh_joint_age_ms,
                    max_cached_joint_age_ms: max_source_joint_age_ms,
                    max_fresh_vision_age_ms,
                    max_cached_vision_age_ms: max_source_vision_age_ms,
                    horizon_ms,
                    vision_generation: locked_generation,
                    vision_target: commit_vision_target,
                    live_ticket_ctx: commit_ticket_ctx,
                };
                match prepared.commit_gate_now(
                    interval_origin_std,
                    period_ns,
                    last_committed_period,
                    &gate_params,
                ) {
                    Ok((CommitGateDecision::Submit { target, commit_period }, gate_now)) => {
                        let input_age = base_input_age_ms.map(|base| {
                            base.saturating_add(
                                gate_now
                                    .saturating_duration_since(captured_at)
                                    .as_millis()
                                    .min(u64::MAX as u128) as u64,
                            )
                        });
                        let vis_age = if let (Some(now_ms), Some(mapped_ms)) =
                            (emb_commit_now_ms, emb_mapped_ms)
                        {
                            Some(now_ms.saturating_sub(mapped_ms))
                        } else {
                            vision_captured_at.map(|cap| {
                                gate_now
                                    .saturating_duration_since(cap)
                                    .as_millis()
                                    .min(u64::MAX as u128) as u64
                            })
                        };
                        (target, Some(commit_period), input_age, vis_age)
                    }
                    Ok((CommitGateDecision::Skip { .. }, _)) => {
                        continue;
                    }
                    Err(e) => {
                        prepared.clear_chunk_buffer();
                        let reason = match e {
                            PolicyError::NonFinite(_) => "NON_FINITE",
                            _ => "INFERENCE_ERROR",
                        };
                        inner.fail_policy_run(action_id, reason, e.to_string());
                        break;
                    }
                }
            } else {
                let target = single_step_target.expect("single-step target");
                let post_vision_age_ms = if let (Some(now_ms), Some(mapped_ms)) =
                    (emb_commit_now_ms, emb_mapped_ms)
                {
                    Some(now_ms.saturating_sub(mapped_ms))
                } else {
                    detector_post_vision_age_ms
                };
                let input_age_ms = base_input_age_ms.map(|base| {
                    base.saturating_add(captured_at.elapsed().as_millis().min(u64::MAX as u128) as u64)
                });
                (target, None, input_age_ms, post_vision_age_ms)
            };
            if let Err(e) = inner.policy_step_with_vision(
                action_id,
                target,
                input_age_ms,
                post_vision_age_ms,
                vision_frame_id,
            ) {
                prepared.clear_chunk_buffer();
                tracing::info!("Policy runner {action_id} exited on step: {e}");
                break;
            }
            if let Some(cp) = committed_period {
                last_committed_period = Some(cp);
                last_infer_period = Some(cp);
            }
        }
    })
}

#[cfg(test)]
mod chunking_tests {
    use super::*;
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicUsize, Ordering};

    struct CountingChunkBackend {
        in_dim: usize,
        out_dim: usize,
        calls: Arc<AtomicUsize>,
        chunks: Vec<Vec<f32>>,
    }

    impl CountingChunkBackend {
        fn new(in_dim: usize, out_dim: usize, chunks: Vec<Vec<f32>>) -> (Self, Arc<AtomicUsize>) {
            let calls = Arc::new(AtomicUsize::new(0));
            (
                Self {
                    in_dim,
                    out_dim,
                    calls: Arc::clone(&calls),
                    chunks,
                },
                calls,
            )
        }
    }

    impl PolicyBackend for CountingChunkBackend {
        fn backend_name(&self) -> &'static str {
            "counting_chunk_mock"
        }
        fn input_dim(&self) -> usize {
            self.in_dim
        }
        fn output_dim(&self) -> usize {
            self.out_dim
        }
        fn infer(&mut self, _input: &[f32], output: &mut [f32]) -> Result<(), PolicyError> {
            let idx = self.calls.fetch_add(1, Ordering::Relaxed);
            let src = &self.chunks[idx.min(self.chunks.len() - 1)];
            output.copy_from_slice(src);
            Ok(())
        }
    }

    fn make_chunk_snapshot_and_contract(
        chunk_size: usize,
        replan_every: usize,
        te_k: Option<f32>,
    ) -> (PolicySnapshot, StrictPolicyContract, ArmConfig) {
        let cfg = ArmConfig::default();
        let dim = cfg.dof;
        let names = cfg.joint_names.clone();
        let mut pj = serde_json::json!({
            "format": "rebot-onnx-policy/v1",
            "name": "chunk_unit_test",
            "unit_mode": "rad",
            "action_dim": dim,
            "action_space": "joint_positions",
            "joint_names": names,
            "model_file": "model.rmlp",
            "outputs": {
                "action": {
                    "name": "action_chunk",
                    "shape": [1, chunk_size * dim]
                }
            },
            "chunking": {
                "chunk_size": chunk_size,
                "replan_every": replan_every,
                "temporal_ensemble_k": te_k
            }
        });
        if te_k.is_none() {
            pj["chunking"].as_object_mut().unwrap().remove("temporal_ensemble_k");
        }
        let pj_bytes = serde_json::to_vec(&pj).unwrap();
        let contract = StrictPolicyContract::parse(&pj_bytes).unwrap();
        let snap = PolicySnapshot {
            dir_path: PathBuf::from("."),
            policy_json_bytes: pj_bytes,
            model_rel_posix: "model.rmlp".into(),
            model_bytes: Vec::new(),
            release_digest: "0".repeat(64),
        };
        (snap, contract, cfg)
    }

    #[test]
    fn chunking_replan_and_temporal_ensemble_exact_values() {
        // Test 1: Open-loop chunking without temporal ensemble (chunk_size=4, replan_every=2)
        let (snap, contract, cfg) = make_chunk_snapshot_and_contract(4, 2, None);
        let dim = cfg.dof;
        // Chunk 0 fills step 0..3 with [0.1, 0.2, 0.3, 0.4]; Chunk 1 fills step 2..5 with [0.5, 0.6, 0.7, 0.8]
        let mut c0 = Vec::with_capacity(4 * dim);
        let mut c1 = Vec::with_capacity(4 * dim);
        for k in 0..4 {
            c0.extend(std::iter::repeat_n((k + 1) as f32 * 0.1, dim));
            c1.extend(std::iter::repeat_n((k + 5) as f32 * 0.1, dim));
        }
        let (backend, calls) = CountingChunkBackend::new(dim, 4 * dim, vec![c0.clone(), c1.clone()]);
        let mut prep = PreparedPolicy::from_backend(snap, contract, Box::new(backend), &cfg).unwrap();
        let state = vec![0.0f32; dim];
        let a0 = prep.step_infer(&state).unwrap(); // replans chunk 0 -> 0.1
        let a1 = prep.step_infer(&state).unwrap(); // consumes chunk 0[1] -> 0.2
        let a2 = prep.step_infer(&state).unwrap(); // replans chunk 1 -> 0.5
        let a3 = prep.step_infer(&state).unwrap(); // consumes chunk 1[1] -> 0.6
        assert_eq!(calls.load(Ordering::Relaxed), 2);
        assert!((a0[0] - 0.1).abs() < 1e-6);
        assert!((a1[0] - 0.2).abs() < 1e-6);
        assert!((a2[0] - 0.5).abs() < 1e-6);
        assert!((a3[0] - 0.6).abs() < 1e-6);

        // Test 2: Temporal ensembling (chunk_size=4, replan_every=2, k=0.01)
        let (snap2, contract2, cfg2) = make_chunk_snapshot_and_contract(4, 2, Some(0.01));
        let (backend2, calls2) = CountingChunkBackend::new(dim, 4 * dim, vec![c0, c1]);
        let mut prep2 = PreparedPolicy::from_backend(snap2, contract2, Box::new(backend2), &cfg2).unwrap();
        let e0 = prep2.step_infer(&state).unwrap(); // only c0[0] = 0.1
        let e1 = prep2.step_infer(&state).unwrap(); // only c0[1] = 0.2
        let e2 = prep2.step_infer(&state).unwrap(); // overlaps c0[2]=0.3 (age 0) and c1[0]=0.5 (age 1)
        assert_eq!(calls2.load(Ordering::Relaxed), 2);
        assert!((e0[0] - 0.1).abs() < 1e-6);
        assert!((e1[0] - 0.2).abs() < 1e-6);
        let w0 = 1.0f32;
        let w1 = (-0.01f32).exp();
        let expected_e2 = (w0 * 0.3 + w1 * 0.5) / (w0 + w1);
        assert!((e2[0] - expected_e2).abs() < 1e-5, "got {}, expected {}", e2[0], expected_e2);
    }

    fn make_constant_chunk(dim: usize, vals: &[f32]) -> Vec<f32> {
        let mut out = Vec::with_capacity(vals.len() * dim);
        for &v in vals {
            out.extend(std::iter::repeat_n(v, dim));
        }
        out
    }

    #[test]
    fn chunking_skips_elapsed_periods_without_timeline_drift() {
        let state_dim = ArmConfig::default().dof;
        let state = vec![0.0f32; state_dim];
        let c0 = make_constant_chunk(state_dim, &[0.1, 0.2, 0.3, 0.4]);
        let c1 = make_constant_chunk(state_dim, &[0.5, 0.6, 0.7, 0.8]);

        // Case 1: Skip within chunk (chunk_size=4, replan_every=4):
        // step 0 takes c0[0], next step reports 2 elapsed periods -> takes c0[2], not c0[1], without replanning.
        let (snap, contract, cfg) = make_chunk_snapshot_and_contract(4, 4, None);
        let (backend, calls) = CountingChunkBackend::new(state_dim, 4 * state_dim, vec![c0.clone(), c1.clone()]);
        let mut prep = PreparedPolicy::from_backend(snap, contract, Box::new(backend), &cfg).unwrap();
        let a0 = prep.step_infer_periods(&state, 1).unwrap();
        let a2 = prep.step_infer_periods(&state, 2).unwrap(); // period 1 skipped -> c0[2]
        let a3 = prep.step_infer_periods(&state, 1).unwrap(); // period 3 -> c0[3]
        assert_eq!(calls.load(Ordering::Relaxed), 1);
        assert!((a0[0] - 0.1).abs() < 1e-6);
        assert!((a2[0] - 0.3).abs() < 1e-6, "expected chunk[2]=0.3, got {}", a2[0]);
        assert!((a3[0] - 0.4).abs() < 1e-6);
        // Next period (t=4) hits replan_every=4 and empty buffer -> replans c1.
        let a4 = prep.step_infer_periods(&state, 1).unwrap();
        assert_eq!(calls.load(Ordering::Relaxed), 2);
        assert!((a4[0] - 0.5).abs() < 1e-6);

        // Case 2: chunk_size=4, replan_every=2 with 2 elapsed periods:
        // elapsed periods since last replan reaches 2 >= replan_every -> triggers replan immediately.
        let (snap2, contract2, cfg2) = make_chunk_snapshot_and_contract(4, 2, None);
        let (backend2, calls2) = CountingChunkBackend::new(state_dim, 4 * state_dim, vec![c0, c1]);
        let mut prep2 = PreparedPolicy::from_backend(snap2, contract2, Box::new(backend2), &cfg2).unwrap();
        let b0 = prep2.step_infer_periods(&state, 1).unwrap();
        let b2 = prep2.step_infer_periods(&state, 2).unwrap();
        assert_eq!(calls2.load(Ordering::Relaxed), 2);
        assert!((b0[0] - 0.1).abs() < 1e-6);
        assert!((b2[0] - 0.5).abs() < 1e-6);
    }

    #[test]
    fn chunking_skip_larger_than_buffer_triggers_immediate_replan() {
        let state_dim = ArmConfig::default().dof;
        let state = vec![0.0f32; state_dim];
        let c0 = make_constant_chunk(state_dim, &[0.1, 0.2, 0.3, 0.4]);
        let c1 = make_constant_chunk(state_dim, &[0.5, 0.6, 0.7, 0.8]);

        // Open-loop: skip 5 periods when only 3 remain -> discard all and replan immediately.
        let (snap, contract, cfg) = make_chunk_snapshot_and_contract(4, 4, None);
        let (backend, calls) = CountingChunkBackend::new(state_dim, 4 * state_dim, vec![c0.clone(), c1.clone()]);
        let mut prep = PreparedPolicy::from_backend(snap, contract, Box::new(backend), &cfg).unwrap();
        let a0 = prep.step_infer_periods(&state, 1).unwrap();
        let a5 = prep.step_infer_periods(&state, 5).unwrap();
        let a6 = prep.step_infer_periods(&state, 1).unwrap();
        assert_eq!(calls.load(Ordering::Relaxed), 2);
        assert!((a0[0] - 0.1).abs() < 1e-6);
        assert!((a5[0] - 0.5).abs() < 1e-6);
        assert!((a6[0] - 0.6).abs() < 1e-6);

        // Temporal ensembling: skip larger than buffer discards all stale entries so c1[0] is unblended.
        let (snap2, contract2, cfg2) = make_chunk_snapshot_and_contract(4, 2, Some(0.01));
        let (backend2, calls2) = CountingChunkBackend::new(state_dim, 4 * state_dim, vec![c0, c1]);
        let mut prep2 = PreparedPolicy::from_backend(snap2, contract2, Box::new(backend2), &cfg2).unwrap();
        let e0 = prep2.step_infer_periods(&state, 1).unwrap();
        let e5 = prep2.step_infer_periods(&state, 5).unwrap();
        assert_eq!(calls2.load(Ordering::Relaxed), 2);
        assert!((e0[0] - 0.1).abs() < 1e-6);
        assert!((e5[0] - 0.5).abs() < 1e-6, "stale entries should be discarded, got {}", e5[0]);
    }

    #[test]
    fn chunking_temporal_ensemble_discards_skipped_periods() {
        let state_dim = ArmConfig::default().dof;
        let state = vec![0.0f32; state_dim];
        let c0 = make_constant_chunk(state_dim, &[0.1, 0.2, 0.3, 0.4]);
        let c1 = make_constant_chunk(state_dim, &[0.5, 0.6, 0.7, 0.8]);
        let c2 = make_constant_chunk(state_dim, &[0.9, 1.0, 1.1, 1.2]);

        let (snap, contract, cfg) = make_chunk_snapshot_and_contract(4, 2, Some(0.01));
        let (backend, calls) = CountingChunkBackend::new(state_dim, 4 * state_dim, vec![c0, c1, c2]);
        let mut prep = PreparedPolicy::from_backend(snap, contract, Box::new(backend), &cfg).unwrap();

        let e0 = prep.step_infer_periods(&state, 1).unwrap(); // t=0: c0[0]=0.1
        assert!((e0[0] - 0.1).abs() < 1e-6);
        assert_eq!(calls.load(Ordering::Relaxed), 1);

        // t=2 (periods_elapsed=2): skips c0[1]=0.2, triggers replan c1, ensembles c0[2]=0.3 with c1[0]=0.5.
        let e2 = prep.step_infer_periods(&state, 2).unwrap();
        assert_eq!(calls.load(Ordering::Relaxed), 2);
        let w0 = 1.0f32;
        let w1 = (-0.01f32).exp();
        let expected_e2 = (w0 * 0.3 + w1 * 0.5) / (w0 + w1);
        assert!((e2[0] - expected_e2).abs() < 1e-5, "got {}, expected {}", e2[0], expected_e2);

        // t=3 (periods_elapsed=1): no replan, ensembles c0[3]=0.4 with c1[1]=0.6.
        let e3 = prep.step_infer_periods(&state, 1).unwrap();
        assert_eq!(calls.load(Ordering::Relaxed), 2);
        let expected_e3 = (w0 * 0.4 + w1 * 0.6) / (w0 + w1);
        assert!((e3[0] - expected_e3).abs() < 1e-5, "got {}, expected {}", e3[0], expected_e3);

        // t=5 (periods_elapsed=2): skips c1[2]=0.7, triggers replan c2, ensembles c1[3]=0.8 with c2[0]=0.9.
        let e5 = prep.step_infer_periods(&state, 2).unwrap();
        assert_eq!(calls.load(Ordering::Relaxed), 3);
        let expected_e5 = (w0 * 0.8 + w1 * 0.9) / (w0 + w1);
        assert!((e5[0] - expected_e5).abs() < 1e-5, "got {}, expected {}", e5[0], expected_e5);
    }

    #[test]
    fn chunking_size_one_is_unchanged_with_elapsed_periods() {
        let state_dim = ArmConfig::default().dof;
        let state = vec![0.0f32; state_dim];
        let chunks = vec![
            make_constant_chunk(state_dim, &[0.1]),
            make_constant_chunk(state_dim, &[0.25]),
            make_constant_chunk(state_dim, &[-0.3]),
            make_constant_chunk(state_dim, &[0.42]),
        ];

        let (snap_a, contract_a, cfg_a) = make_chunk_snapshot_and_contract(1, 1, None);
        let (backend_a, calls_a) = CountingChunkBackend::new(state_dim, state_dim, chunks.clone());
        let mut prep_a = PreparedPolicy::from_backend(snap_a, contract_a, Box::new(backend_a), &cfg_a).unwrap();

        let (snap_b, contract_b, cfg_b) = make_chunk_snapshot_and_contract(1, 1, None);
        let (backend_b, calls_b) = CountingChunkBackend::new(state_dim, state_dim, chunks);
        let mut prep_b = PreparedPolicy::from_backend(snap_b, contract_b, Box::new(backend_b), &cfg_b).unwrap();

        for elapsed in [1u32, 3, 10, 0] {
            let ref_out = prep_a.step_infer(&state).unwrap();
            let per_out = prep_b.step_infer_periods(&state, elapsed).unwrap();
            assert_eq!(
                per_out.iter().map(|v| v.to_bits()).collect::<Vec<_>>(),
                ref_out.iter().map(|v| v.to_bits()).collect::<Vec<_>>(),
                "chunk_size=1 must stay byte-identical for periods_elapsed={elapsed}"
            );
        }
        assert_eq!(calls_a.load(Ordering::Relaxed), 4);
        assert_eq!(calls_b.load(Ordering::Relaxed), 4);
    }

    #[test]
    fn chunking_ring_capacity_does_not_grow_over_50_steps() {
        let state_dim = ArmConfig::default().dof;
        let state = vec![0.0f32; state_dim];
        let chunk_size = 8usize;
        let chunks: Vec<Vec<f32>> = (0..60)
            .map(|c| {
                let vals: Vec<f32> = (0..chunk_size)
                    .map(|k| ((c * chunk_size + k) % 20) as f32 * 0.02 - 0.2)
                    .collect();
                make_constant_chunk(state_dim, &vals)
            })
            .collect();

        // Case 1: Open-loop chunking (replan_every=2) over 50 steps including occasional skips.
        let (snap_ol, contract_ol, cfg_ol) = make_chunk_snapshot_and_contract(chunk_size, 2, None);
        let (backend_ol, _) =
            CountingChunkBackend::new(state_dim, chunk_size * state_dim, chunks.clone());
        let mut prep_ol =
            PreparedPolicy::from_backend(snap_ol, contract_ol, Box::new(backend_ol), &cfg_ol)
                .unwrap();
        let initial_ol = prep_ol.chunk_ring.storage_snapshot();
        assert_eq!(initial_ol.0, chunk_size);
        assert_eq!(initial_ol.2, chunk_size * state_dim);
        for step in 0..50usize {
            let elapsed = if step > 0 && step.is_multiple_of(11) { 3 } else { 1 };
            let act = prep_ol.step_infer_periods(&state, elapsed).unwrap();
            assert_eq!(act.len(), state_dim);
            assert!(act.iter().all(|v| v.is_finite()));
            assert_eq!(
                prep_ol.chunk_ring.storage_snapshot(),
                initial_ol,
                "open-loop ChunkRing reallocated on step {step}"
            );
        }

        // Case 2: Worst-case temporal ensembling (replan_every=1, full chunk_size overlap) over 50 steps.
        let (snap_te, contract_te, cfg_te) =
            make_chunk_snapshot_and_contract(chunk_size, 1, Some(0.05));
        let (backend_te, calls_te) =
            CountingChunkBackend::new(state_dim, chunk_size * state_dim, chunks);
        let mut prep_te =
            PreparedPolicy::from_backend(snap_te, contract_te, Box::new(backend_te), &cfg_te)
                .unwrap();
        let initial_te = prep_te.chunk_ring.storage_snapshot();
        assert_eq!(initial_te.0, chunk_size);
        assert_eq!(initial_te.2, chunk_size * chunk_size * state_dim);
        for step in 0..50usize {
            let elapsed = if step > 0 && step.is_multiple_of(13) { 2 } else { 1 };
            let act = prep_te.step_infer_periods(&state, elapsed).unwrap();
            assert_eq!(act.len(), state_dim);
            assert!(act.iter().all(|v| v.is_finite()));
            assert_eq!(
                prep_te.chunk_ring.storage_snapshot(),
                initial_te,
                "temporal-ensemble ChunkRing reallocated on step {step}"
            );
        }
        assert_eq!(calls_te.load(Ordering::Relaxed), 50);
    }

    #[test]
    fn chunking_stale_source_drops_chunk_and_forces_replan() {
        let state_dim = ArmConfig::default().dof;
        let state = vec![0.0f32; state_dim];
        let c0 = make_constant_chunk(state_dim, &[0.1, 0.2, 0.3, 0.4]);
        let c1 = make_constant_chunk(state_dim, &[0.5, 0.6, 0.7, 0.8]);

        // 20 Hz, replan_every=4 -> STALE_INPUT=100ms, horizon=200ms, max_source_age=300ms.
        let max_age_ms = max_cached_chunk_source_age_ms(20.0, 4);
        assert_eq!(max_age_ms, 300);

        let (snap, contract, cfg) = make_chunk_snapshot_and_contract(4, 4, None);
        let (backend, calls) =
            CountingChunkBackend::new(state_dim, 4 * state_dim, vec![c0.clone(), c1.clone()]);
        let mut prep =
            PreparedPolicy::from_backend(snap, contract, Box::new(backend), &cfg).unwrap();

        let t0 = Instant::now();
        let src0 = ChunkSource::from_joint(t0, Some(10));
        let fresh_at_t0 = ChunkFreshnessLimits {
            now: t0,
            max_source_joint_age_ms: max_age_ms,
            max_source_vision_age_ms: None,
            horizon_ms: 200,
            vision_generation: None,
            vision_target: None,
            live_ticket_ctx: None,
        };
        let a0 = prep
            .step_infer_with_source(&state, None, 1, src0, Some(&fresh_at_t0))
            .unwrap();
        assert!((a0[0] - 0.1).abs() < 1e-6);
        assert_eq!(calls.load(Ordering::Relaxed), 1);

        // 50 ms later: source age = 60 ms <= 300 ms -> uses cached c0[1].
        let t1 = t0 + Duration::from_millis(50);
        let fresh_at_t1 = ChunkFreshnessLimits { now: t1, ..fresh_at_t0.clone() };
        let a1 = prep
            .step_infer_with_source(&state, None, 1, ChunkSource::from_joint(t1, Some(10)), Some(&fresh_at_t1))
            .unwrap();
        assert!((a1[0] - 0.2).abs() < 1e-6);
        assert_eq!(calls.load(Ordering::Relaxed), 1);

        // 350 ms after t0: source age = 360 ms > 300 ms -> chunk dropped and replans c1 immediately.
        let t2 = t0 + Duration::from_millis(350);
        let stale_at_t2 = ChunkFreshnessLimits { now: t2, ..fresh_at_t0 };
        let a2 = prep
            .step_infer_with_source(&state, None, 1, ChunkSource::from_joint(t2, Some(10)), Some(&stale_at_t2))
            .unwrap();
        assert_eq!(calls.load(Ordering::Relaxed), 2);
        assert!((a2[0] - 0.5).abs() < 1e-6, "stale chunk must be dropped and replanned, got {}", a2[0]);
    }

    #[test]
    fn chunking_invalidated_ticket_and_run_context_clear_ring() {
        let state_dim = ArmConfig::default().dof;
        let state = vec![0.0f32; state_dim];
        let c0 = make_constant_chunk(state_dim, &[0.1, 0.2, 0.3, 0.4]);
        let c1 = make_constant_chunk(state_dim, &[0.5, 0.6, 0.7, 0.8]);

        let (snap, contract, cfg) = make_chunk_snapshot_and_contract(4, 4, None);
        let (backend, calls) = CountingChunkBackend::new(state_dim, 4 * state_dim, vec![c0, c1]);
        let mut prep =
            PreparedPolicy::from_backend(snap, contract, Box::new(backend), &cfg).unwrap();

        let t0 = Instant::now();
        let ticket = InferenceTicket::new(1, 7, 1, Some(3), 10, 1, 50_000, 1_000, 1_000);
        let target_a = DetectorTargetIdentity::from_detection(None, "cube", [0.1, 0.2, 0.3, 0.4]);
        let mut src0 = ChunkSource::from_joint(t0, Some(5));
        src0.embedding_ticket = Some(ticket);
        src0.vision_target = Some(target_a);

        let valid_ctx = TicketVerificationContext {
            policy_instance_id: Some(1),
            lease_generation: Some(7),
            producer_epoch: Some(1),
            model_generation: Some(3),
            invalidation_generation: 10,
            clock_handshake_generation: 1,
            now_armd_ms: 500,
        };
        let valid_limits = ChunkFreshnessLimits {
            now: t0,
            max_source_joint_age_ms: 300,
            max_source_vision_age_ms: None,
            horizon_ms: 200,
            vision_generation: None,
            vision_target: Some(target_a),
            live_ticket_ctx: Some(valid_ctx.clone()),
        };
        let a0 = prep
            .step_infer_with_source(&state, None, 1, src0, Some(&valid_limits))
            .unwrap();
        assert!((a0[0] - 0.1).abs() < 1e-6);
        assert_eq!(prep.chunk_buffer_len(), 3);

        // Invalidate the source embedding ticket (invalidation_generation bumped 10 -> 11).
        let invalidated_limits = ChunkFreshnessLimits {
            live_ticket_ctx: Some(TicketVerificationContext {
                invalidation_generation: 11,
                ..valid_ctx.clone()
            }),
            ..valid_limits.clone()
        };
        assert!(prep.prune_chunk_buffer(&invalidated_limits));
        assert_eq!(prep.chunk_buffer_len(), 0);

        // Repopulate buffer, then verify every ChunkRunContext trigger clears the whole ring:
        // stop/E-stop/fault (active_ok=false), lease change, lease revocation, mode switch.
        let base_ctx = ChunkRunContext {
            mode: PolicyMode::Shadow,
            epoch_id: 1,
            lease_generation: Some(7),
            active_ok: true,
        };
        prep.sync_run_context(base_ctx);
        let _ = prep
            .step_infer_with_source(&state, None, 1, src0, Some(&valid_limits))
            .unwrap();
        assert_eq!(calls.load(Ordering::Relaxed), 2);
        assert_eq!(prep.chunk_buffer_len(), 3);

        // Stop / E-stop / Fault
        assert!(prep.sync_run_context(ChunkRunContext { active_ok: false, ..base_ctx }));
        assert_eq!(prep.chunk_buffer_len(), 0);

        // Repopulate and test lease revocation
        prep.sync_run_context(base_ctx);
        let _ = prep.step_infer_with_source(&state, None, 1, src0, Some(&valid_limits)).unwrap();
        assert_eq!(prep.chunk_buffer_len(), 3);
        assert!(prep.sync_run_context(ChunkRunContext { lease_generation: None, ..base_ctx }));
        assert_eq!(prep.chunk_buffer_len(), 0);

        // Repopulate and test lease change
        prep.sync_run_context(base_ctx);
        let _ = prep.step_infer_with_source(&state, None, 1, src0, Some(&valid_limits)).unwrap();
        assert_eq!(prep.chunk_buffer_len(), 3);
        assert!(prep.sync_run_context(ChunkRunContext { lease_generation: Some(8), ..base_ctx }));
        assert_eq!(prep.chunk_buffer_len(), 0);

        // Repopulate and test shadow <-> live mode switch
        let live_ctx = ChunkRunContext {
            mode: PolicyMode::Live,
            lease_generation: Some(8),
            ..base_ctx
        };
        let _ = prep.step_infer_with_source(&state, None, 1, src0, Some(&valid_limits)).unwrap();
        assert_eq!(prep.chunk_buffer_len(), 3);
        assert!(prep.sync_run_context(live_ctx));
        assert_eq!(prep.chunk_buffer_len(), 0);

        // Repopulate and test detector target lost (target_a -> Absent) clears ring
        let _ = prep.step_infer_with_source(&state, None, 1, src0, Some(&valid_limits)).unwrap();
        assert_eq!(prep.chunk_buffer_len(), 3);
        let target_lost_limits = ChunkFreshnessLimits {
            vision_target: Some(DetectorTargetIdentity::Absent),
            ..valid_limits
        };
        assert!(prep.prune_chunk_buffer(&target_lost_limits));
        assert_eq!(prep.chunk_buffer_len(), 0);
    }

    #[test]
    fn chunking_commit_gate_target_identity_keeps_stationary_and_clears_on_move_or_label() {
        use crate::grasp::Detection;
        use super::super::manifest::{VisionFeatureSpec, VisionLayout, VisionSourceKind, VisionTargetLostPolicy};

        let state_dim = ArmConfig::default().dof;
        let state = vec![0.0f32; state_dim];
        let origin = Instant::now();
        let period_ns = Duration::from_millis(50).as_nanos();

        let make_det = |label: &str, box_norm: [f64; 4]| Detection {
            label: label.to_string(),
            score: 0.95,
            raw_score_valid: true,
            box_norm,
            angle_deg: None,
            angle_conf: None,
            grasp_width_px: None,
            mask_focus: false,
            disp_fg: None,
            disp_bg: None,
            depth_conf: None,
            length_px: None,
            jaw_clear: None,
            depth_angle_deg: None,
            depth_angle_conf: None,
            depth_width_px: None,
        };
        let vision_spec = VisionFeatureSpec {
            source: VisionSourceKind::Detector,
            layout: VisionLayout::Top1BoxNormV1,
            target_label: None,
            min_score: 0.3,
            max_staleness_ms: 500,
            max_sync_skew_ms: 500,
            on_target_lost: VisionTargetLostPolicy::Hold,
            on_stale: VisionStalePolicy::Stop,
            require_model_generation: false,
        };

        // Part 1: Stationary target across 10 new frames (frame_id 1..=10, small box jitter with IoU >= 0.5)
        // keeps the chunk without dropping or replanning.
        let chunk_size = 12usize;
        let vals: Vec<f32> = (0..chunk_size).map(|i| 0.05 + i as f32 * 0.01).collect();
        let c0 = make_constant_chunk(state_dim, &vals);
        let (snap, contract, cfg) = make_chunk_snapshot_and_contract(chunk_size, chunk_size, None);
        let (backend, calls) =
            CountingChunkBackend::new(state_dim, chunk_size * state_dim, vec![c0]);
        let mut prep =
            PreparedPolicy::from_backend(snap, contract, Box::new(backend), &cfg).unwrap();

        let frame0 = super::super::vision::PolicyVisionFrame {
            frame_id: 1,
            captured_at: Some(origin),
            detections: vec![make_det("cube", [0.20, 0.20, 0.40, 0.40])],
            model_generation: None,
        };
        let (_, id0) = extract_top1_target(&frame0, &vision_spec).unwrap();
        let mut src0 = ChunkSource::from_joint(origin, Some(5));
        src0.vision_target = Some(id0);
        prep.plan_chunk_step(&state, None, 0, src0, None).unwrap();
        assert_eq!(calls.load(Ordering::Relaxed), 1);

        let mut last_committed = None;
        for step in 0u64..10 {
            // New frame_id every step with tiny 0.01 jitter (IoU ~0.82 >= 0.5).
            let jitter = if step % 2 == 0 { 0.0 } else { 0.01 };
            let frame_n = super::super::vision::PolicyVisionFrame {
                frame_id: 1 + step,
                captured_at: Some(origin + Duration::from_millis(step * 50)),
                detections: vec![make_det(
                    "cube",
                    [0.20 + jitter, 0.20, 0.40 + jitter, 0.40],
                )],
                model_generation: None,
            };
            let (_, id_n) = extract_top1_target(&frame_n, &vision_spec).unwrap();
            let gate = CommitGateParams {
                max_fresh_joint_age_ms: 100,
                max_cached_joint_age_ms: 1000,
                max_fresh_vision_age_ms: None,
                max_cached_vision_age_ms: None,
                horizon_ms: 600,
                vision_generation: None,
                vision_target: Some(id_n),
                live_ticket_ctx: None,
            };
            let d = prep
                .commit_gate(
                    origin + Duration::from_millis(step * 50 + 5),
                    origin,
                    period_ns,
                    last_committed,
                    &gate,
                )
                .unwrap();
            match d {
                CommitGateDecision::Submit { target, commit_period } => {
                    assert_eq!(commit_period, step);
                    assert!((target[0] - vals[step as usize]).abs() < 1e-6);
                }
                other => panic!("stationary target at frame {} must keep chunk, got {other:?}", 1 + step),
            }
            last_committed = Some(step);
        }
        assert_eq!(calls.load(Ordering::Relaxed), 1, "chunk must stay alive across 10 stationary frames");
        assert_eq!(prep.chunk_buffer_len(), 2);

        // Part 2: Target moved to IoU < 0.5 (same label) or changed label (same box) clears the ring.
        let target_cube = DetectorTargetIdentity::from_detection(None, "cube", [0.20, 0.20, 0.40, 0.40]);
        let target_moved_low_iou =
            DetectorTargetIdentity::from_detection(None, "cube", [0.35, 0.20, 0.55, 0.40]); // IoU = (0.05*0.2)/(0.08-0.01) = 0.14 < 0.5
        let target_other_label =
            DetectorTargetIdentity::from_detection(None, "cup", [0.20, 0.20, 0.40, 0.40]);
        let c4 = make_constant_chunk(state_dim, &[0.1, 0.2, 0.3, 0.4]);

        for replacement in [target_moved_low_iou, target_other_label] {
            let (snap_r, contract_r, cfg_r) = make_chunk_snapshot_and_contract(4, 4, None);
            let (backend_r, _) =
                CountingChunkBackend::new(state_dim, 4 * state_dim, vec![c4.clone()]);
            let mut prep_r =
                PreparedPolicy::from_backend(snap_r, contract_r, Box::new(backend_r), &cfg_r)
                    .unwrap();
            let mut src_r = ChunkSource::from_joint(origin, Some(5));
            src_r.vision_target = Some(target_cube);
            let gate_same = CommitGateParams {
                max_fresh_joint_age_ms: 100,
                max_cached_joint_age_ms: 300,
                max_fresh_vision_age_ms: None,
                max_cached_vision_age_ms: None,
                horizon_ms: 200,
                vision_generation: None,
                vision_target: Some(target_cube),
                live_ticket_ctx: None,
            };
            prep_r.plan_chunk_step(&state, None, 0, src_r, None).unwrap();
            assert!(matches!(
                prep_r.commit_gate(origin + Duration::from_millis(5), origin, period_ns, None, &gate_same).unwrap(),
                CommitGateDecision::Submit { commit_period: 0, .. }
            ));
            assert_eq!(prep_r.chunk_buffer_len(), 3);

            let gate_replaced = CommitGateParams {
                vision_target: Some(replacement),
                ..gate_same
            };
            let d1 = prep_r
                .commit_gate(
                    origin + Duration::from_millis(55),
                    origin,
                    period_ns,
                    Some(0),
                    &gate_replaced,
                )
                .unwrap();
            assert_eq!(d1, CommitGateDecision::Skip { commit_period: 1 });
            assert_eq!(prep_r.chunk_buffer_len(), 0);
        }

        // Part 3: TrackId matching ignores box movement; changed TrackId clears ring.
        let tid1 = DetectorTargetIdentity::from_detection(Some(42), "cube", [0.1, 0.2, 0.3, 0.4]);
        let tid2 = DetectorTargetIdentity::from_detection(Some(43), "cube", [0.1, 0.2, 0.3, 0.4]);
        let (snap_t, contract_t, cfg_t) = make_chunk_snapshot_and_contract(4, 4, None);
        let (backend_t, _) = CountingChunkBackend::new(state_dim, 4 * state_dim, vec![c4]);
        let mut prep_t =
            PreparedPolicy::from_backend(snap_t, contract_t, Box::new(backend_t), &cfg_t).unwrap();
        let mut src_t = ChunkSource::from_joint(origin, Some(5));
        src_t.vision_target = Some(tid1);
        let gate_t1 = CommitGateParams {
            max_fresh_joint_age_ms: 100,
            max_cached_joint_age_ms: 300,
            max_fresh_vision_age_ms: None,
            max_cached_vision_age_ms: None,
            horizon_ms: 200,
            vision_generation: None,
            vision_target: Some(tid1),
            live_ticket_ctx: None,
        };
        prep_t.plan_chunk_step(&state, None, 0, src_t, None).unwrap();
        assert!(matches!(
            prep_t.commit_gate(origin + Duration::from_millis(5), origin, period_ns, None, &gate_t1).unwrap(),
            CommitGateDecision::Submit { commit_period: 0, .. }
        ));
        assert!(matches!(
            prep_t.commit_gate(origin + Duration::from_millis(55), origin, period_ns, Some(0), &gate_t1).unwrap(),
            CommitGateDecision::Submit { commit_period: 1, .. }
        ));
        let gate_t2 = CommitGateParams { vision_target: Some(tid2), ..gate_t1 };
        let d2 = prep_t
            .commit_gate(origin + Duration::from_millis(105), origin, period_ns, Some(1), &gate_t2)
            .unwrap();
        assert_eq!(d2, CommitGateDecision::Skip { commit_period: 2 });
        assert_eq!(prep_t.chunk_buffer_len(), 0);
    }

    #[test]
    fn chunking_horizon_over_1000ms_is_refused_in_manifest_and_at_start() {
        let base = serde_json::json!({
            "format": "rebot-onnx-policy/v1",
            "name": "horizon_test",
            "unit_mode": "rad",
            "action_dim": 7,
            "action_space": "joint_positions",
            "joint_names": ["j1", "j2", "j3", "j4", "j5", "j6", "g"],
            "model_file": "model.rmlp"
        });

        // 21 * (1000 / 20) = 1050 ms > 1000 ms -> refused at manifest parse.
        let mut over = base.clone();
        over["fps"] = serde_json::json!(20.0);
        over["chunking"] = serde_json::json!({"chunk_size": 24, "replan_every": 21});
        let err = StrictPolicyContract::parse(&serde_json::to_vec(&over).unwrap()).unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("replan_every") && msg.contains("fps"), "unexpected error: {msg}");

        // Exact boundary: 20 * (1000 / 20) = 1000 ms -> accepted.
        let mut exact = base.clone();
        exact["fps"] = serde_json::json!(20.0);
        exact["chunking"] = serde_json::json!({"chunk_size": 24, "replan_every": 20});
        assert!(StrictPolicyContract::parse(&serde_json::to_vec(&exact).unwrap()).is_ok());

        // Manifest omits fps -> accepted at parse, refused at start when effective fps yields > 1000 ms.
        let mut no_fps = base;
        no_fps["chunking"] = serde_json::json!({"chunk_size": 24, "replan_every": 15});
        let contract = StrictPolicyContract::parse(&serde_json::to_vec(&no_fps).unwrap()).unwrap();
        let start_err = validate_chunking_horizon(contract.chunking, 10.0).unwrap_err();
        let start_msg = start_err.to_string();
        assert!(
            start_msg.contains("replan_every") && start_msg.contains("fps"),
            "unexpected start error: {start_msg}"
        );
        assert!(validate_chunking_horizon(contract.chunking, 20.0).is_ok());
    }

    #[test]
    fn chunking_commit_gate_selects_current_period_or_skips_when_exhausted() {
        let state_dim = ArmConfig::default().dof;
        let state = vec![0.0f32; state_dim];
        let c0 = make_constant_chunk(state_dim, &[0.05, 0.15, 0.25, 0.35]);
        let origin = Instant::now();
        let period_ns = Duration::from_millis(50).as_nanos();
        let gate_params = CommitGateParams {
            max_fresh_joint_age_ms: 100,
            max_cached_joint_age_ms: 200,
            max_fresh_vision_age_ms: None,
            max_cached_vision_age_ms: None,
            horizon_ms: 100,
            vision_generation: None,
            vision_target: None,
            live_ticket_ctx: None,
        };

        // Case 1: planned in period 0, commit_gate runs at 60 ms (period 1) -> submits chunk[1]=0.15.
        let (snap, contract, cfg) = make_chunk_snapshot_and_contract(4, 2, None);
        let (backend, _) = CountingChunkBackend::new(state_dim, 4 * state_dim, vec![c0]);
        let mut prep =
            PreparedPolicy::from_backend(snap, contract, Box::new(backend), &cfg).unwrap();
        prep.plan_chunk_step(&state, None, 0, ChunkSource::from_joint(origin, Some(5)), None)
            .unwrap();
        let decision = prep
            .commit_gate(origin + Duration::from_millis(60), origin, period_ns, None, &gate_params)
            .unwrap();
        match decision {
            CommitGateDecision::Submit { target, commit_period } => {
                assert_eq!(commit_period, 1);
                assert!((target[0] - 0.15).abs() < 1e-6);
            }
            other => panic!("expected Submit for period 1, got {other:?}"),
        }
        assert_eq!(prep.chunk_buffer_len(), 2);

        // Case 2: overrun larger than chunk buffer -> Skip and ring cleared.
        let (snap2, contract2, cfg2) = make_chunk_snapshot_and_contract(2, 2, None);
        let c_short = make_constant_chunk(state_dim, &[0.05, 0.15]);
        let (backend2, _) = CountingChunkBackend::new(state_dim, 2 * state_dim, vec![c_short]);
        let mut prep2 =
            PreparedPolicy::from_backend(snap2, contract2, Box::new(backend2), &cfg2).unwrap();
        prep2
            .plan_chunk_step(&state, None, 0, ChunkSource::from_joint(origin, Some(5)), None)
            .unwrap();
        let decision2 = prep2
            .commit_gate(origin + Duration::from_millis(115), origin, period_ns, None, &gate_params)
            .unwrap();
        assert_eq!(decision2, CommitGateDecision::Skip { commit_period: 2 });
        assert_eq!(prep2.chunk_buffer_len(), 0);
    }

    #[test]
    fn chunking_commit_gate_drops_source_expiring_before_commit_in_same_period_and_replans() {
        let state_dim = ArmConfig::default().dof;
        let state = vec![0.0f32; state_dim];
        let c0 = make_constant_chunk(state_dim, &[0.1, 0.2, 0.3, 0.4]);
        let c1 = make_constant_chunk(state_dim, &[0.5, 0.6, 0.7, 0.8]);
        let origin = Instant::now();
        let period_ns = Duration::from_millis(50).as_nanos(); // 20 Hz

        // 20 Hz, replan_every=4 -> fresh limit=100ms, cached limit=300ms.
        let (snap, contract, cfg) = make_chunk_snapshot_and_contract(4, 4, None);
        let (backend, calls) = CountingChunkBackend::new(state_dim, 4 * state_dim, vec![c0, c1]);
        let mut prep =
            PreparedPolicy::from_backend(snap, contract, Box::new(backend), &cfg).unwrap();
        let gate_params = CommitGateParams {
            max_fresh_joint_age_ms: 100,
            max_cached_joint_age_ms: 300,
            max_fresh_vision_age_ms: None,
            max_cached_vision_age_ms: None,
            horizon_ms: 200,
            vision_generation: None,
            vision_target: None,
            live_ticket_ctx: None,
        };

        // Period 0: fresh inference with base age 90 ms; at t=5ms age=95ms <= 100ms -> submits c0[0].
        let src0 = ChunkSource::from_joint(origin, Some(90));
        prep.plan_chunk_step(&state, None, 0, src0, None).unwrap();
        let d0 = prep
            .commit_gate(origin + Duration::from_millis(5), origin, period_ns, None, &gate_params)
            .unwrap();
        assert!(matches!(d0, CommitGateDecision::Submit { commit_period: 0, .. }));
        assert_eq!(calls.load(Ordering::Relaxed), 1);

        // Part 1: fresh inference inside the limit at plan_chunk_step (age 94 ms <= 100 ms)
        // that expires before commit within the SAME period (period 0, age 107 ms > 100 ms)
        // -> not submitted, chunk dropped, forced replan on period 1.
        let (snap_b, contract_b, cfg_b) = make_chunk_snapshot_and_contract(4, 4, None);
        let (backend_b, calls_b) = CountingChunkBackend::new(
            state_dim,
            4 * state_dim,
            vec![
                make_constant_chunk(state_dim, &[0.1, 0.2, 0.3, 0.4]),
                make_constant_chunk(state_dim, &[0.5, 0.6, 0.7, 0.8]),
            ],
        );
        let mut prep_b =
            PreparedPolicy::from_backend(snap_b, contract_b, Box::new(backend_b), &cfg_b).unwrap();
        let src_fresh = ChunkSource::from_joint(origin, Some(92));
        let fresh_plan_limits = ChunkFreshnessLimits {
            now: origin + Duration::from_millis(2),
            max_source_joint_age_ms: 300,
            max_source_vision_age_ms: None,
            horizon_ms: 200,
            vision_generation: None,
            vision_target: None,
            live_ticket_ctx: None,
        };
        prep_b
            .plan_chunk_step(&state, None, 0, src_fresh, Some(&fresh_plan_limits))
            .unwrap();
        assert_eq!(calls_b.load(Ordering::Relaxed), 1);
        let d_exp = prep_b
            .commit_gate(origin + Duration::from_millis(15), origin, period_ns, None, &gate_params)
            .unwrap();
        assert_eq!(d_exp, CommitGateDecision::Skip { commit_period: 0 });
        assert_eq!(prep_b.chunk_buffer_len(), 0, "expired chunk must be dropped");

        // Next period (period 1, t=52ms): fresh source replans immediately -> calls_b == 2, submits c1[0]=0.5.
        let src_next = ChunkSource::from_joint(origin + Duration::from_millis(50), Some(5));
        prep_b
            .plan_chunk_step(&state, None, 1, src_next, None)
            .unwrap();
        assert_eq!(calls_b.load(Ordering::Relaxed), 2);
        let d_next = prep_b
            .commit_gate(origin + Duration::from_millis(52), origin, period_ns, None, &gate_params)
            .unwrap();
        match d_next {
            CommitGateDecision::Submit { target, commit_period } => {
                assert_eq!(commit_period, 1);
                assert!((target[0] - 0.5).abs() < 1e-6);
            }
            other => panic!("expected replanned Submit on period 1, got {other:?}"),
        }

        // Part 2: cached entry inside cached limit (200 ms) during plan_chunk_step in period 2
        // (t=105ms, age=195ms <= 200ms) that expires before commit within the SAME period 2
        // (t=115ms, age=205ms > 200ms) -> not submitted, chunk dropped, forced replan on period 3.
        let (snap_c, contract_c, cfg_c) = make_chunk_snapshot_and_contract(4, 4, None);
        let (backend_c, calls_c) = CountingChunkBackend::new(
            state_dim,
            4 * state_dim,
            vec![
                make_constant_chunk(state_dim, &[0.1, 0.2, 0.3, 0.4]),
                make_constant_chunk(state_dim, &[0.5, 0.6, 0.7, 0.8]),
            ],
        );
        let mut prep_c =
            PreparedPolicy::from_backend(snap_c, contract_c, Box::new(backend_c), &cfg_c).unwrap();
        let cached_gate = CommitGateParams {
            max_fresh_joint_age_ms: 100,
            max_cached_joint_age_ms: 200,
            max_fresh_vision_age_ms: None,
            max_cached_vision_age_ms: None,
            horizon_ms: 100,
            vision_generation: None,
            vision_target: None,
            live_ticket_ctx: None,
        };
        prep_c
            .plan_chunk_step(&state, None, 0, ChunkSource::from_joint(origin, Some(90)), None)
            .unwrap();
        assert!(matches!(
            prep_c.commit_gate(origin + Duration::from_millis(2), origin, period_ns, None, &cached_gate).unwrap(),
            CommitGateDecision::Submit { commit_period: 0, .. }
        ));
        // Period 2 ([100ms, 150ms)): at t=105ms age=195ms <= 200ms (passes pruning),
        // then at commit t=115ms (STILL period 2!) age=205ms > 200ms -> Skip & drop chunk.
        let plan_limits_p2 = ChunkFreshnessLimits {
            now: origin + Duration::from_millis(105),
            max_source_joint_age_ms: 200,
            max_source_vision_age_ms: None,
            horizon_ms: 100,
            vision_generation: None,
            vision_target: None,
            live_ticket_ctx: None,
        };
        prep_c
            .plan_chunk_step(
                &state,
                None,
                2,
                ChunkSource::from_joint(origin + Duration::from_millis(105), Some(5)),
                Some(&plan_limits_p2),
            )
            .unwrap();
        assert_eq!(calls_c.load(Ordering::Relaxed), 1, "still inside limit during plan_chunk_step");
        let d_cached_exp = prep_c
            .commit_gate(
                origin + Duration::from_millis(115),
                origin,
                period_ns,
                Some(0),
                &cached_gate,
            )
            .unwrap();
        assert_eq!(d_cached_exp, CommitGateDecision::Skip { commit_period: 2 });
        assert_eq!(prep_c.chunk_buffer_len(), 0);
        // Period 3 (t=152ms): replans c1!
        prep_c
            .plan_chunk_step(
                &state,
                None,
                3,
                ChunkSource::from_joint(origin + Duration::from_millis(150), Some(5)),
                None,
            )
            .unwrap();
        assert_eq!(calls_c.load(Ordering::Relaxed), 2);
        let d_replan = prep_c
            .commit_gate(
                origin + Duration::from_millis(152),
                origin,
                period_ns,
                Some(0),
                &cached_gate,
            )
            .unwrap();
        match d_replan {
            CommitGateDecision::Submit { target, commit_period } => {
                assert_eq!(commit_period, 3);
                assert!((target[0] - 0.5).abs() < 1e-6);
            }
            other => panic!("expected replanned Submit on period 3, got {other:?}"),
        }
    }

    #[test]
    fn chunking_commit_gate_rejects_old_target_when_period_boundary_passes_before_submit() {
        let state_dim = ArmConfig::default().dof;
        let state = vec![0.0f32; state_dim];
        let c0 = make_constant_chunk(state_dim, &[0.05, 0.15, 0.25, 0.35]);
        let origin = Instant::now();
        let period_ns = Duration::from_millis(50).as_nanos();
        let gate_params = CommitGateParams {
            max_fresh_joint_age_ms: 100,
            max_cached_joint_age_ms: 300,
            max_fresh_vision_age_ms: None,
            max_cached_vision_age_ms: None,
            horizon_ms: 200,
            vision_generation: None,
            vision_target: None,
            live_ticket_ctx: None,
        };

        let (snap, contract, cfg) = make_chunk_snapshot_and_contract(4, 4, None);
        let (backend, _) = CountingChunkBackend::new(state_dim, 4 * state_dim, vec![c0]);
        let mut prep =
            PreparedPolicy::from_backend(snap, contract, Box::new(backend), &cfg).unwrap();

        // Planned and pruned at t=48ms (period 0 -> staged target is c0[0]=0.05),
        // then 50ms boundary passes before submit (t=52ms -> period 1):
        // old period-0 target (0.05) must NOT be submitted; period-1 slot (0.15) is selected.
        prep.plan_chunk_step(
            &state,
            None,
            0,
            ChunkSource::from_joint(origin + Duration::from_millis(45), Some(2)),
            None,
        )
        .unwrap();
        let d1 = prep
            .commit_gate(origin + Duration::from_millis(52), origin, period_ns, None, &gate_params)
            .unwrap();
        match d1 {
            CommitGateDecision::Submit { target, commit_period } => {
                assert_eq!(commit_period, 1);
                assert!(
                    (target[0] - 0.15).abs() < 1e-6,
                    "old period-0 target (0.05) must not be submitted after boundary; got {}",
                    target[0]
                );
            }
            other => panic!("expected Submit for period 1, got {other:?}"),
        }

        // Now plan at t=198ms (period 3, the LAST slot c0[3]=0.35), then 200ms boundary passes
        // before commit (t=202ms -> period 4, missing from buffer) -> Skip, old 0.35 NOT submitted.
        prep.plan_chunk_step(
            &state,
            None,
            3,
            ChunkSource::from_joint(origin + Duration::from_millis(195), Some(2)),
            None,
        )
        .unwrap();
        let d4 = prep
            .commit_gate(
                origin + Duration::from_millis(202),
                origin,
                period_ns,
                Some(1),
                &gate_params,
            )
            .unwrap();
        assert_eq!(d4, CommitGateDecision::Skip { commit_period: 4 });
        assert_eq!(prep.chunk_buffer_len(), 0);
    }

    #[tokio::test]
    async fn chunking_overrunning_replan_commits_current_period_or_nothing() {
        // Part A: 60 ms replan at 20 Hz (50 ms period) starting in period 0 with chunk_size=4
        // lands in period 1 -> action submitted must be chunk[1] (0.15), not chunk[0] (0.05).
        let (snap, contract, cfg) = make_chunk_snapshot_and_contract(4, 2, None);
        let dim = cfg.dof;
        let c0 = make_constant_chunk(dim, &[0.05, 0.15, 0.25, 0.35]);
        let (backend, _) = CountingChunkBackend::new(dim, 4 * dim, vec![c0]);
        let prep = PreparedPolicy::from_backend(snap, contract, Box::new(backend), &cfg)
            .unwrap()
            .with_infer_delay(Duration::from_millis(60));

        let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
        let lease_gen = sup.acquire_lease("chunk-overrun-a").await.unwrap();
        let action_id = {
            let mut inner = sup.inner().write().await;
            let epoch_id = inner.safety.epoch_id();
            inner
                .begin_policy_run(PolicyRunStart {
                    epoch_id,
                    connection_id: "chunk-overrun-a".into(),
                    lease_generation: lease_gen,
                    policy_id: "chunk_overrun_a".into(),
                    artifact_digest: prep.artifact_digest().to_string(),
                    backend: prep.backend_name().to_string(),
                    mode: PolicyMode::Shadow,
                    sim_only: true,
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
                    i_am_at_the_arm: false,
                    verified_release_gate: None,
                })
                .unwrap()
        };
        let handle = spawn_policy_runner(
            sup.clone(),
            action_id,
            prep,
            20.0,
            compute_policy_deadline(Some(0.095)).unwrap(),
            None,
            None,
        );
        handle.await.unwrap();
        let obs = sup.get_observation().await;
        let pol = obs.policy.expect("policy observation");
        assert_eq!(pol.steps, 1, "expected exactly 1 committed step before 95 ms deadline");
        let first_target = pol.last_shadow_target.expect("shadow target");
        assert!(
            (first_target[0] - 0.15).abs() < 1e-6,
            "60 ms replan at 20 Hz must commit period 1 (0.15), not period 0 (0.05); got {}",
            first_target[0]
        );

        // Part B: 115 ms replan at 20 Hz with chunk_size=2 lands in period >= 2, which the
        // 2-slot chunk [0, 1] cannot cover -> nothing is submitted on that tick.
        let (snap2, contract2, cfg2) = make_chunk_snapshot_and_contract(2, 2, None);
        let c_short = make_constant_chunk(dim, &[0.05, 0.15]);
        let (backend2, _) = CountingChunkBackend::new(dim, 2 * dim, vec![c_short]);
        let prep2 = PreparedPolicy::from_backend(snap2, contract2, Box::new(backend2), &cfg2)
            .unwrap()
            .with_infer_delay(Duration::from_millis(115));

        let sup2 = Arc::new(ArmSupervisor::new(cfg2));
        let lease_gen2 = sup2.acquire_lease("chunk-overrun-b").await.unwrap();
        let action_id2 = {
            let mut inner = sup2.inner().write().await;
            let epoch_id = inner.safety.epoch_id();
            inner
                .begin_policy_run(PolicyRunStart {
                    epoch_id,
                    connection_id: "chunk-overrun-b".into(),
                    lease_generation: lease_gen2,
                    policy_id: "chunk_overrun_b".into(),
                    artifact_digest: prep2.artifact_digest().to_string(),
                    backend: prep2.backend_name().to_string(),
                    mode: PolicyMode::Shadow,
                    sim_only: true,
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
                    i_am_at_the_arm: false,
                    verified_release_gate: None,
                })
                .unwrap()
        };
        let handle2 = spawn_policy_runner(
            sup2.clone(),
            action_id2,
            prep2,
            20.0,
            compute_policy_deadline(Some(0.14)).unwrap(),
            None,
            None,
        );
        handle2.await.unwrap();
        let obs2 = sup2.get_observation().await;
        let pol2 = obs2.policy.expect("policy observation");
        assert_eq!(pol2.steps, 0, "overrun past chunk buffer must submit nothing");
        assert!(pol2.last_shadow_target.is_none());
    }

    #[test]
    fn chunking_manifest_rejects_invalid_specs() {
        let base = serde_json::json!({
            "format": "rebot-onnx-policy/v1",
            "name": "bad_chunk",
            "unit_mode": "rad",
            "action_dim": 7,
            "action_space": "joint_positions",
            "joint_names": ["j1", "j2", "j3", "j4", "j5", "j6", "g"],
            "model_file": "model.rmlp"
        });
        for bad_chunk in [
            serde_json::json!({"chunk_size": 0}),
            serde_json::json!({"chunk_size": 4, "replan_every": 5}),
            serde_json::json!({"chunk_size": 4, "replan_every": 0}),
            serde_json::json!({"chunk_size": 4, "temporal_ensemble_k": -0.1}),
            serde_json::json!({"chunk_size": 4, "temporal_ensemble_k": 100.001}),
            serde_json::json!({"chunk_size": 4, "temporal_ensemble_k": 1.0e300}),
            serde_json::json!({"chunk_size": 4, "temporal_ensemble_k": "0.1"}),
            serde_json::json!({"chunk_size": 200}), // > MAX_CHUNK_SIZE
        ] {
            let mut pj = base.clone();
            pj["chunking"] = bad_chunk;
            assert!(
                StrictPolicyContract::parse(&serde_json::to_vec(&pj).unwrap()).is_err(),
                "expected invalid chunking manifest to fail: {pj}"
            );
        }
        // Boundary values 0.0 and 100.0 are accepted for temporal_ensemble_k.
        for ok_k in [0.0f64, 0.01, 100.0] {
            let mut pj = base.clone();
            pj["chunking"] = serde_json::json!({"chunk_size": 4, "replan_every": 2, "temporal_ensemble_k": ok_k});
            let c = StrictPolicyContract::parse(&serde_json::to_vec(&pj).unwrap()).unwrap();
            assert_eq!(c.chunking.temporal_ensemble_k, Some(ok_k as f32));
        }
        // Mismatched outputs.action.shape when chunk_size=4 (expected 28, given 7)
        let mut pj = base.clone();
        pj["chunking"] = serde_json::json!({"chunk_size": 4, "replan_every": 2});
        pj["outputs"] = serde_json::json!({"action": {"name": "a", "shape": [1, 7]}});
        assert!(StrictPolicyContract::parse(&serde_json::to_vec(&pj).unwrap()).is_err());

        // Unflattened [K, D] and [1, K, D] output shapes are rejected with instructions to export [1, K*D].
        for unflat in [serde_json::json!([4, 7]), serde_json::json!([1, 4, 7])] {
            let mut pj = base.clone();
            pj["chunking"] = serde_json::json!({"chunk_size": 4, "replan_every": 2});
            pj["outputs"] = serde_json::json!({"action": {"name": "a", "shape": unflat}});
            let err = StrictPolicyContract::parse(&serde_json::to_vec(&pj).unwrap()).unwrap_err();
            let msg = err.to_string();
            assert!(
                msg.contains("[1, 28]"),
                "expected error to recommend flattened [1, 28], got: {msg}"
            );
        }
        // Flattened [1, K*D] and [K*D] output shapes are accepted.
        for flat in [serde_json::json!([1, 28]), serde_json::json!([28])] {
            let mut pj = base.clone();
            pj["chunking"] = serde_json::json!({"chunk_size": 4, "replan_every": 2});
            pj["outputs"] = serde_json::json!({"action": {"name": "a", "shape": flat}});
            assert!(StrictPolicyContract::parse(&serde_json::to_vec(&pj).unwrap()).is_ok());
        }
    }

    #[test]
    #[ignore = "requires local exported bundle path via REBOT_TEST_RECAMERA_BUNDLE"]
    fn loads_and_steps_exported_microduck_recamera_bundle_when_present() {
        let Some(bundle_env) = std::env::var_os("REBOT_TEST_RECAMERA_BUNDLE") else {
            eprintln!("skipped: REBOT_TEST_RECAMERA_BUNDLE is not set");
            return;
        };
        let bundle = PathBuf::from(bundle_env);
        if !bundle.join("policy.json").is_file() {
            eprintln!(
                "skipped: policy.json not found in REBOT_TEST_RECAMERA_BUNDLE ({})",
                bundle.display()
            );
            return;
        }
        let snap = PolicySnapshot::load_from_dir(&bundle).expect("load_from_dir");
        let contract = StrictPolicyContract::parse(&snap.policy_json_bytes).expect("parse contract");
        assert_eq!(contract.chunking.chunk_size, 24);
        assert_eq!(contract.chunking.replan_every, 2);
        assert_eq!(contract.expected_input_dim(), 135);
        assert_eq!(contract.expected_output_dim(), 168);

        let backend = super::super::load_backend(&snap, &contract).expect("load RMLP backend");
        assert_eq!(backend.backend_name(), "mlp");
        let mut cfg = ArmConfig::default();
        cfg.dof = 7;
        cfg.joint_names = vec![
            "joint1".into(),
            "joint2".into(),
            "joint3".into(),
            "joint4".into(),
            "joint5".into(),
            "joint6".into(),
            "gripper".into(),
        ];
        while cfg.limits.len() < 7 {
            cfg.limits.push(cfg.limits[0].clone());
        }
        for (i, name) in cfg.joint_names.iter().enumerate() {
            cfg.limits[i].name = name.clone();
        }
        let mut prep = PreparedPolicy::from_backend(snap, contract, backend, &cfg).expect("from_backend");
        let warm = prep.warmup(5, Duration::from_secs(5)).expect("warmup");
        let state = vec![0.0f32; cfg.dof];
        let emb = vec![0.0f32; 128];
        for _ in 0..6 {
            let act = prep.step_infer_with_vision(&state, Some(&emb)).expect("step_infer_with_vision");
            assert_eq!(act.len(), cfg.dof);
            assert!(act.iter().all(|v| v.is_finite()));
        }
        eprintln!("microduck recamera_bundle RMLP warmup p50={}us max={}us", warm.p50, warm.max);
    }
}
