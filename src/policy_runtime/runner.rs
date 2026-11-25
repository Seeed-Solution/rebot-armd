//! Async policy runner, prepared policy pipeline, and supervisor `PolicyRun` state.

use std::path::Path;
use std::sync::atomic::{AtomicU64, Ordering};
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
    validate_chunking_horizon, validate_chunking_spec, PolicySnapshot, StrictPolicyContract,
    VideoEmbeddingSpec, VisionStalePolicy, VisionTargetLostPolicy,
};
use super::release_gate::VerifiedReleaseGate;
use super::units::{
    denormalize_action_into, normalize_state_into, policy_units_to_rad, rad_to_policy_units,
    JointPermutation,
};
use super::vision::{
    embedding_hub_for_supervisor, evaluate_vision_frame, extract_top1_target, monotonic_ms_at,
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

/// Copyable RT release-gate binding carried by executor-owned [`PolicyRun`] instances so
/// `armd-control` enforces real-arm artifact, calibration, and velocity-scale invariants with
/// zero heap ownership and zero frees on termination.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RtGateBinding {
    pub policy_sha256: crate::model::HolderLabel,
    pub calibration_digest: crate::model::HolderLabel,
    pub max_speed_scale: f64,
    pub production_release: bool,
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
    /// Copyable RT binding used on `OWNER_EXECUTOR` when `verified_release_gate` is `None`.
    pub(crate) rt_gate_binding: Option<RtGateBinding>,
    /// Authorization generation accepted with the currently installed live `servo_target`.
    pub(crate) live_target_auth_gen: Option<u64>,
    /// True for executor-owned runs recycled unconditionally into `SupervisorInner::policy_run_spare`.
    pub(crate) rt_owned: bool,
    /// Client-chosen monotonic attempt token owning this run (for cancellation isolation & runner ownership).
    pub(crate) load_token: u64,
    /// Preallocated slot index in `PolicyCancelSlots` for `load_token`.
    pub(crate) cancel_slot: u8,
    /// Optional absolute `max_seconds` run deadline enforced directly on `armd-control`.
    pub(crate) deadline: Option<Instant>,
    pub stop_reason: Option<String>,
    /// [`max_policy_input_age_ms`] for the run's rate (set by the runner; until then the
    /// bound for [`MIN_LIVE_POLICY_FPS`]).
    pub max_input_age_ms: u64,
    /// Per-step timing behind `ActionStatus::policy_stats`.
    pub step_stats: PolicyStepStats,
    /// Output authorization of targets waiting in the latest-target mailbox (see
    /// [`PolicyAuthorization`]); a hold or a vision invalidation revokes it.
    pub authorization: PolicyAuthorization,
    /// Largest camera-sample age a target may have when the RT consumes it (None: no vision),
    /// and whether an older one holds the run (`on_stale: hold`) instead of ending it.
    pub max_vision_age_ms: Option<u64>,
    pub vision_stale_hold: bool,
}

/// Output authorization generation of one policy run, shared between the run's state in the
/// core, the runner and the vision hub. A target is stamped with [`Self::current`] when its
/// inputs are sampled ([`crate::rt_core::mailbox::TargetFrame::policy_generation`]); the RT
/// applies it only while the generation is unchanged. [`Self::revoke`] (a vision hold, a
/// vision invalidation, a model or clock-handshake change) refuses every target stamped
/// before it. Never 0 (0 marks a frame that is not a policy frame). Lock-free: the RT only
/// loads it.
#[derive(Debug, Clone)]
pub struct PolicyAuthorization(Arc<AtomicU64>);

impl Default for PolicyAuthorization {
    fn default() -> Self { Self(Arc::new(AtomicU64::new(1))) }
}

impl PolicyAuthorization {
    const GEN_MASK: u64 = 0xFFFF_FFFF;

    #[inline]
    fn action_tag(action_id: u64) -> u64 {
        let t = action_id & Self::GEN_MASK;
        if t == 0 { 1 } else { t }
    }

    #[inline]
    fn next_gen(cur: u64) -> u64 {
        let g = (cur & Self::GEN_MASK).wrapping_add(1) & Self::GEN_MASK;
        if g == 0 { 1 } else { g }
    }

    pub fn current(&self) -> u64 { self.0.load(Ordering::Acquire) }

    /// Binds this shared RT authorization to `action_id` and advances the generation (0 allocs).
    pub(crate) fn bind_action(&self, action_id: u64) -> u64 {
        let tag = Self::action_tag(action_id);
        let mut cur = self.0.load(Ordering::Acquire);
        loop {
            let next = (tag << 32) | Self::next_gen(cur);
            match self.0.compare_exchange_weak(cur, next, Ordering::AcqRel, Ordering::Acquire) {
                Ok(_) => return next,
                Err(actual) => cur = actual,
            }
        }
    }

    /// Unbinds `action_id` and advances the generation (CAS-scoped so a replacement run is untouched).
    pub(crate) fn unbind_action(&self, action_id: u64) {
        let tag = Self::action_tag(action_id);
        let mut cur = self.0.load(Ordering::Acquire);
        while (cur >> 32) == tag {
            let next = Self::next_gen(cur);
            match self.0.compare_exchange_weak(cur, next, Ordering::AcqRel, Ordering::Acquire) {
                Ok(_) => return,
                Err(actual) => cur = actual,
            }
        }
    }

    /// Revokes targets ONLY while `action_id` still owns this authorization (returns `false`
    /// without mutating a replacement run's generation if the owner already changed).
    pub(crate) fn revoke_for_action(&self, action_id: u64) -> bool {
        let tag = Self::action_tag(action_id);
        let mut cur = self.0.load(Ordering::Acquire);
        while (cur >> 32) == tag {
            let next = (tag << 32) | Self::next_gen(cur);
            match self.0.compare_exchange_weak(cur, next, Ordering::AcqRel, Ordering::Acquire) {
                Ok(_) => return true,
                Err(actual) => cur = actual,
            }
        }
        false
    }

    /// Revokes every target stamped so far (preserving any active action tag); returns the new generation.
    pub fn revoke(&self) -> u64 {
        let mut cur = self.0.load(Ordering::Acquire);
        loop {
            let next = (cur & !Self::GEN_MASK) | Self::next_gen(cur);
            match self.0.compare_exchange_weak(cur, next, Ordering::AcqRel, Ordering::Acquire) {
                Ok(_) => return next,
                Err(actual) => cur = actual,
            }
        }
    }
}

impl PolicyRun {
    /// Preallocated empty `PolicyRun` recycled in place by `OWNER_EXECUTOR` (0 RT alloc/free).
    pub(crate) fn preallocated_rt_spare(authorization: PolicyAuthorization) -> Self {
        Self {
            action_id: 0,
            epoch_id: 0,
            connection_id: String::with_capacity(crate::model::HolderLabel::CAPACITY),
            lease_generation: 0,
            policy_id: String::with_capacity(crate::model::HolderLabel::CAPACITY),
            artifact_digest: String::with_capacity(crate::model::HolderLabel::CAPACITY),
            backend: String::with_capacity(64),
            mode: PolicyMode::Shadow,
            steps: 0,
            last_input_age_ms: None,
            last_vision_age_ms: None,
            vision_frame_id: None,
            last_shadow_target: None,
            last_live_target: None,
            max_first_step_rad: DEFAULT_MAX_FIRST_STEP_RAD,
            max_step_rad: None,
            i_am_at_the_arm: false,
            verified_release_gate: None,
            rt_gate_binding: None,
            live_target_auth_gen: None,
            rt_owned: true,
            load_token: 0,
            cancel_slot: 0,
            deadline: None,
            stop_reason: None,
            max_input_age_ms: max_policy_input_age_ms(MIN_LIVE_POLICY_FPS),
            step_stats: PolicyStepStats::default(),
            authorization,
            max_vision_age_ms: None,
            vision_stale_hold: false,
        }
    }

    pub(crate) fn release_gate_max_speed_scale(&self) -> Option<f64> {
        self.verified_release_gate
            .as_ref()
            .map(VerifiedReleaseGate::max_speed_scale)
            .or_else(|| self.rt_gate_binding.as_ref().map(|g| g.max_speed_scale))
    }

    pub(crate) fn gate_calibration_digest(&self) -> Option<&str> {
        self.verified_release_gate
            .as_ref()
            .map(VerifiedReleaseGate::calibration_digest)
            .or_else(|| self.rt_gate_binding.as_ref().map(|g| g.calibration_digest.as_str()))
    }

    pub(crate) fn gate_policy_sha256(&self) -> Option<&str> {
        self.verified_release_gate
            .as_ref()
            .map(VerifiedReleaseGate::policy_sha256)
            .or_else(|| self.rt_gate_binding.as_ref().map(|g| g.policy_sha256.as_str()))
    }

    /// [`Self::to_observation`] into `out`, reusing `vec_spare` for `last_shadow_target`
    /// transitions so `None <-> Some(Vec<f32>)` never allocates or frees on `armd-control`.
    pub fn to_observation_into_with_vec_spare(
        &self,
        active: bool,
        out: &mut PolicyObservation,
        vec_spare: &mut Vec<f32>,
    ) {
        use crate::rt_core::scratch::{copy_into, set_str};
        out.active = active;
        out.action_id = self.action_id;
        set_str(&mut out.policy_id, &self.policy_id);
        set_str(&mut out.artifact_digest, &self.artifact_digest);
        set_str(&mut out.backend, &self.backend);
        set_str(&mut out.mode, self.mode.as_str());
        out.steps = self.steps;
        out.last_input_age_ms = self.last_input_age_ms;
        out.last_vision_age_ms = self.last_vision_age_ms;
        out.vision_frame_id = self.vision_frame_id;
        match self.last_shadow_target.as_deref() {
            Some(src) => match out.last_shadow_target.as_mut() {
                Some(dst) => copy_into(dst, src),
                None => {
                    let mut v = std::mem::take(vec_spare);
                    copy_into(&mut v, src);
                    out.last_shadow_target = Some(v);
                }
            },
            None => {
                if let Some(old) = out.last_shadow_target.take() {
                    if old.capacity() >= vec_spare.capacity() {
                        *vec_spare = old;
                    }
                }
            }
        }
        out.stop_reason = self.stop_reason.as_deref().map(crate::model::HolderLabel::new);
    }

    /// [`Self::to_observation`] into `out`, reusing its memory.
    pub fn to_observation_into(&self, active: bool, out: &mut PolicyObservation) {
        use crate::rt_core::scratch::{set_opt, set_str};
        out.active = active;
        out.action_id = self.action_id;
        set_str(&mut out.policy_id, &self.policy_id);
        set_str(&mut out.artifact_digest, &self.artifact_digest);
        set_str(&mut out.backend, &self.backend);
        set_str(&mut out.mode, self.mode.as_str());
        out.steps = self.steps;
        out.last_input_age_ms = self.last_input_age_ms;
        out.last_vision_age_ms = self.last_vision_age_ms;
        out.vision_frame_id = self.vision_frame_id;
        set_opt(&mut out.last_shadow_target, self.last_shadow_target.as_ref());
        out.stop_reason = self.stop_reason.as_deref().map(crate::model::HolderLabel::new);
    }

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
            stop_reason: self.stop_reason.as_deref().map(crate::model::HolderLabel::new),
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

    /// Record one target skipped due to mailbox backpressure (`Submitted::Busy`).
    pub fn record_skipped(&mut self) {
        self.skipped = self.skipped.saturating_add(1);
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

    /// Exact monotonic deadline at which this source's joint observation exceeds `limit_ms`
    /// (`now > deadline` is stale). Preserves sub-millisecond precision without floor loss.
    pub fn joint_deadline(&self, limit_ms: u64) -> Option<Instant> {
        let base = self.joint_base_age_ms.unwrap_or(0);
        if base > limit_ms {
            return None;
        }
        self.joint_captured_at
            .checked_add(Duration::from_millis(limit_ms - base))
    }

    pub fn vision_age_ms(&self, now: Instant) -> Option<u64> {
        self.vision_captured_at.map(|t| {
            now.saturating_duration_since(t)
                .as_millis()
                .min(u64::MAX as u128) as u64
        })
    }

    /// Exact monotonic deadline at which this source's detector frame exceeds `limit_ms`
    /// (`now > deadline` is stale). Preserves sub-millisecond precision without floor loss.
    pub fn vision_deadline(&self, limit_ms: u64) -> Option<Instant> {
        self.vision_captured_at
            .and_then(|t| t.checked_add(Duration::from_millis(limit_ms)))
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
    /// Optional synthetic `armd_ms` clock override for tests (`None` evaluates embedding
    /// deadlines directly against `monotonic_ms_at(now)` from the gate's own clock read).
    pub embedding_clock_override_ms: Option<u64>,
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
                    if src
                        .joint_deadline(limits.max_source_joint_age_ms)
                        .is_none_or(|dl| limits.now > dl)
                    {
                        return false;
                    }
                    if let Some(max_v) = limits.max_source_vision_age_ms {
                        if src.vision_captured_at.is_some()
                            && src
                                .vision_deadline(max_v)
                                .is_none_or(|dl| limits.now > dl)
                        {
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
    single_step_source: Option<ChunkSource>,
    single_step_seq: u64,
    chunk_ring: ChunkRing,
    last_step_sources: Vec<ChunkSource>,
    last_run_ctx: Option<ChunkRunContext>,
    last_commit_slot_expires_at: Option<Instant>,
    last_commit_period_end: Option<Instant>,
    last_commit_handshake_expires_at: Option<Instant>,
}

impl PreparedPolicy {
    /// Construct a validated inference pipeline from an explicit backend and arm config.
    pub fn from_backend(
        snapshot: PolicySnapshot,
        contract: StrictPolicyContract,
        backend: Box<dyn PolicyBackend>,
        config: &ArmConfig,
    ) -> Result<Self, PolicyError> {
        validate_chunking_spec(contract.chunking, contract.action_dim)?;
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
            single_step_source: None,
            single_step_seq: 1,
            chunk_ring,
            last_step_sources,
            last_run_ctx: None,
            last_commit_slot_expires_at: None,
            last_commit_period_end: None,
            last_commit_handshake_expires_at: None,
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
        self.single_step_source = None;
        self.last_step_sources.clear();
        self.last_commit_slot_expires_at = None;
        self.last_commit_period_end = None;
        self.last_commit_handshake_expires_at = None;
    }

    /// Strictest contributing source expiry (inclusive: `now > exp`) from the most recent
    /// `commit_gate` `Submit` (`None` for single-step policies or after `Skip` / clear).
    pub fn last_commit_slot_expires_at(&self) -> Option<Instant> {
        self.last_commit_slot_expires_at
    }

    /// Exclusive end instant of the selected control period (`now >= end` is past the slot)
    /// from the most recent chunked `commit_gate` `Submit`.
    pub fn last_commit_period_end(&self) -> Option<Instant> {
        self.last_commit_period_end
    }

    /// Strictest embedding clock-handshake expiry across all contributing chunk sources from
    /// the most recent `commit_gate` `Submit` (`None` when no embedding ticket contributed).
    pub fn last_commit_handshake_expires_at(&self) -> Option<Instant> {
        self.last_commit_handshake_expires_at
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

    /// Off-lock planning stage for both single-step (`chunk_size == 1`) and chunked
    /// (`chunk_size > 1`) policies: prepares inputs, prunes stale chunk entries, runs backend
    /// inference when due for `sample_period`, and stages the converted candidate target
    /// without consuming the ring slot. Final slot selection, freshness, and provenance checks
    /// run in [`Self::commit_gate`].
    pub fn plan_chunk_step(
        &mut self,
        measured_arm_rad: &[f32],
        vision_features: Option<&[f32]>,
        sample_period: u64,
        mut source: ChunkSource,
        freshness: Option<&ChunkFreshnessLimits>,
    ) -> Result<Option<u64>, PolicyError> {
        let dim = self.contract.action_dim;
        let chunking = self.contract.chunking;
        self.prepare_infer_input(measured_arm_rad, vision_features)?;

        if chunking.chunk_size <= 1 {
            self.backend.infer(&self.infer_in, &mut self.infer_out)?;
            for (idx, &v) in self.infer_out.iter().enumerate() {
                if !v.is_finite() {
                    return Err(PolicyError::NonFinite(format!(
                        "model output[{idx}] is non-finite: {v}"
                    )));
                }
            }
            self.step_out_norm.copy_from_slice(&self.infer_out[..dim]);
            let _ = self.finalize_step_target()?;
            let seq = self.single_step_seq;
            self.single_step_seq = self.single_step_seq.wrapping_add(1).max(1);
            source.infer_seq = seq;
            self.single_step_source = Some(source);
            self.staged_period = Some(sample_period);
            self.staged_infer_seq = Some(seq);
            self.staged_ready = true;
            return Ok(Some(seq));
        }

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

    /// Single final commit gate for both single-step (`chunk_size == 1`) and chunked
    /// (`chunk_size > 1`) policies, executed immediately before submitting to
    /// `policy_step_with_vision`:
    ///
    /// (a) Computes `commit_period` from `now`. For `chunk_size > 1`, selects that period's
    ///     slot (discarding passed slots) and skips if already committed/passed or missing;
    ///     for `chunk_size == 1`, consumes the single freshly inferred staged target.
    /// (b) Checks every contributing source age against `now` (`max_fresh_*` for a fresh
    ///     inference from this tick, `max_cached_*` for cached chunk entries). For chunked
    ///     policies, expiry drops the whole chunk and skips submission without failing the run;
    ///     for single-step policies, joint freshness continues to be enforced by the supervisor's
    ///     `STALE_INPUT` fence (keeping single-step live/shadow behaviour unchanged).
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
        self.last_commit_slot_expires_at = None;
        self.last_commit_period_end = None;
        self.last_commit_handshake_expires_at = None;
        let commit_period = (now
            .saturating_duration_since(interval_origin)
            .as_nanos()
            / period_ns.max(1))
        .min(u64::MAX as u128) as u64;

        let eval_ticket_ctx = params.live_ticket_ctx.as_ref().map(|ctx| {
            let now_armd_ms = params
                .embedding_clock_override_ms
                .unwrap_or_else(|| monotonic_ms_at(now));
            TicketVerificationContext {
                now_armd_ms,
                ..ctx.clone()
            }
        });

        if self.contract.chunking.chunk_size <= 1 {
            let Some(src) = self.single_step_source.filter(|_| self.staged_ready) else {
                self.staged_ready = false;
                return Ok(CommitGateDecision::Skip { commit_period });
            };
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
            if let Some(max_v) = params.max_fresh_vision_age_ms {
                if src.vision_captured_at.is_some()
                    && src.vision_deadline(max_v).is_none_or(|dl| now > dl)
                {
                    self.clear_chunk_buffer();
                    return Ok(CommitGateDecision::Skip { commit_period });
                }
            }
            if let (Some(ticket), Some(live_ctx)) =
                (src.embedding_ticket, eval_ticket_ctx.as_ref())
            {
                if ticket.verify_identity_and_horizon(live_ctx, 0).is_err() {
                    self.clear_chunk_buffer();
                    return Ok(CommitGateDecision::Skip { commit_period });
                }
                let hs_rem_ms = ticket
                    .clock_handshake_expires_at_ms()
                    .saturating_sub(live_ctx.now_armd_ms);
                self.last_commit_handshake_expires_at =
                    now.checked_add(Duration::from_millis(hs_rem_ms));
            }
            self.staged_ready = false;
            self.staged_period = None;
            self.staged_infer_seq = None;
            self.single_step_source = None;
            return Ok(CommitGateDecision::Submit {
                target: self.arm_target_rad.clone(),
                commit_period,
            });
        }

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
        let mut slot_exp: Option<Instant> = None;
        let mut hs_exp: Option<Instant> = None;
        let tighten = |dst: &mut Option<Instant>, cand: Option<Instant>| {
            if let Some(c) = cand {
                *dst = Some(dst.map_or(c, |cur| cur.min(c)));
            }
        };
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
            let Some(joint_dl) = src.joint_deadline(joint_limit) else {
                self.clear_chunk_buffer();
                return Ok(CommitGateDecision::Skip { commit_period });
            };
            if now > joint_dl {
                self.clear_chunk_buffer();
                return Ok(CommitGateDecision::Skip { commit_period });
            }
            tighten(&mut slot_exp, Some(joint_dl));
            let vision_limit = if is_fresh {
                params.max_fresh_vision_age_ms
            } else {
                params.max_cached_vision_age_ms
            };
            if let Some(max_v) = vision_limit {
                if src.vision_captured_at.is_some() {
                    let Some(vision_dl) = src.vision_deadline(max_v) else {
                        self.clear_chunk_buffer();
                        return Ok(CommitGateDecision::Skip { commit_period });
                    };
                    if now > vision_dl {
                        self.clear_chunk_buffer();
                        return Ok(CommitGateDecision::Skip { commit_period });
                    }
                    tighten(&mut slot_exp, Some(vision_dl));
                }
            }
            if let (Some(ticket), Some(live_ctx)) =
                (src.embedding_ticket, eval_ticket_ctx.as_ref())
            {
                let ticket_horizon = if is_fresh { 0 } else { params.horizon_ms };
                if ticket
                    .verify_identity_and_horizon(live_ctx, ticket_horizon)
                    .is_err()
                {
                    self.clear_chunk_buffer();
                    return Ok(CommitGateDecision::Skip { commit_period });
                }
                let win_dl = ticket
                    .window_oldest_deadline_ms()
                    .min(ticket.window_newest_deadline_ms())
                    .saturating_add(ticket_horizon);
                let win_rem_ms = win_dl.saturating_sub(live_ctx.now_armd_ms);
                tighten(
                    &mut slot_exp,
                    now.checked_add(Duration::from_millis(win_rem_ms)),
                );
                let hs_rem_ms = ticket
                    .clock_handshake_expires_at_ms()
                    .saturating_sub(live_ctx.now_armd_ms);
                tighten(
                    &mut hs_exp,
                    now.checked_add(Duration::from_millis(hs_rem_ms)),
                );
            }
        }
        let period_end_ns = (commit_period as u128)
            .saturating_add(1)
            .saturating_mul(period_ns.max(1));
        let period_end = u64::try_from(period_end_ns)
            .ok()
            .and_then(|ns_u64| interval_origin.checked_add(Duration::from_nanos(ns_u64)));

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
        self.last_commit_slot_expires_at = slot_exp;
        self.last_commit_period_end = period_end;
        self.last_commit_handshake_expires_at = hs_exp;
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

enum FinalCommitGateOutcome {
    Submit {
        target: Vec<f32>,
        commit_period: u64,
        gate_now: Instant,
        eval_now_ms: u64,
        input_age_ms: Option<u64>,
        post_vision_age_ms: Option<u64>,
    },
    Skip,
    Hold,
    Finish(&'static str),
    Fail {
        reason: &'static str,
        message: String,
    },
}

/// Single synchronous final commit gate shared by BOTH legacy (`spawn_policy_runner`) and
/// RT-executor (`executor_commit_gate`) owners.
///
/// Acquires the latest detector snapshot (`collector.latest()`) BEFORE reading `gate_now` ONCE,
/// then enforces deadline, coherent latest-frame detector timestamp / staleness / generation /
/// target-lost disposition (`Hold` / `Stop` / `ZeroFeatures`), pre-inference detector staleness,
/// current-tick embedding `InferenceTicket` (with `is_unbroken_active_renewal_at(..., eval_now_ms)`),
/// and `prepared.commit_gate(gate_now, ...)` with zero intervening clock reads or extra snapshots.
#[allow(clippy::too_many_arguments)]
fn evaluate_final_commit_gate(
    prepared: &mut PreparedPolicy,
    deadline: Option<Instant>,
    interval_origin_std: Instant,
    period_ns: u128,
    last_committed_period: Option<u64>,
    max_fresh_joint_age_ms: u64,
    max_source_joint_age_ms: u64,
    max_fresh_vision_age_ms: Option<u64>,
    max_source_vision_age_ms: Option<u64>,
    horizon_ms: u64,
    video_embedding_spec: Option<&VideoEmbeddingSpec>,
    vision_spec: Option<&super::manifest::VisionFeatureSpec>,
    hub_guard: Option<&HubCommitGuard<'_>>,
    live_lease: Option<u64>,
    collector: Option<&VisionFeatureCollector>,
    locked_generation: Option<u64>,
    base_input_age_ms: Option<u64>,
    captured_at: Instant,
    vision_captured_at: Option<Instant>,
    vision_target: Option<DetectorTargetIdentity>,
    emb_ticket: Option<&InferenceTicket>,
    emb_mapped_and_unc_ms: Option<(u64, u64)>,
) -> FinalCommitGateOutcome {
    // Acquire the detector frame snapshot BEFORE sampling `gate_now` so lock acquisition and
    // frame cloning cannot move the sampled instant behind a period boundary.
    let latest_detector_frame = if video_embedding_spec.is_none() {
        collector.and_then(VisionFeatureCollector::latest)
    } else {
        None
    };

    let gate_now = Instant::now();
    if deadline.is_some_and(|dl| gate_now >= dl) {
        prepared.clear_chunk_buffer();
        return FinalCommitGateOutcome::Finish("MAX_SECONDS");
    }

    // 1. Single coherent detector snapshot check: timestamp presence, latest-frame and
    //    pre-inference staleness (exact Instant deadlines), model_generation, and target-lost disposition.
    let commit_vision_target = if video_embedding_spec.is_none() {
        if let (Some(spec), Some(vis_cap)) = (vision_spec, vision_captured_at) {
            if vis_cap
                .checked_add(Duration::from_millis(spec.max_staleness_ms))
                .is_none_or(|dl| gate_now > dl)
            {
                let age_ms = gate_now
                    .saturating_duration_since(vis_cap)
                    .as_millis()
                    .min(u64::MAX as u128) as u64;
                prepared.clear_chunk_buffer();
                return match spec.on_stale {
                    VisionStalePolicy::Stop => FinalCommitGateOutcome::Fail {
                        reason: "VISION_STALE",
                        message: format!(
                            "Vision frame became stale before step commit ({age_ms} ms > {} ms)",
                            spec.max_staleness_ms
                        ),
                    },
                    VisionStalePolicy::Hold => FinalCommitGateOutcome::Hold,
                };
            }
        }
        if let (Some(spec), Some(ref frame)) = (vision_spec, &latest_detector_frame) {
            let Some(frame_cap) = frame.captured_at else {
                prepared.clear_chunk_buffer();
                return match spec.on_stale {
                    VisionStalePolicy::Stop => FinalCommitGateOutcome::Fail {
                        reason: "VISION_TIMESTAMP_MISSING",
                        message: format!(
                            "Vision frame {} has no capture timestamp before step commit",
                            frame.frame_id
                        ),
                    },
                    VisionStalePolicy::Hold => FinalCommitGateOutcome::Hold,
                };
            };
            if frame_cap
                .checked_add(Duration::from_millis(spec.max_staleness_ms))
                .is_none_or(|dl| gate_now > dl)
            {
                let latest_age_ms = gate_now
                    .saturating_duration_since(frame_cap)
                    .as_millis()
                    .min(u64::MAX as u128) as u64;
                prepared.clear_chunk_buffer();
                return match spec.on_stale {
                    VisionStalePolicy::Stop => FinalCommitGateOutcome::Fail {
                        reason: "VISION_STALE",
                        message: format!(
                            "Latest vision frame {} is stale before step commit ({latest_age_ms} ms > {} ms)",
                            frame.frame_id, spec.max_staleness_ms
                        ),
                    },
                    VisionStalePolicy::Hold => FinalCommitGateOutcome::Hold,
                };
            }
            if spec.require_model_generation && frame.model_generation != locked_generation {
                prepared.clear_chunk_buffer();
                let (reason, message) = match frame.model_generation {
                    None => (
                        "VISION_GENERATION_MISSING",
                        format!(
                            "Vision frame {} missing required model_generation before step commit",
                            frame.frame_id
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
                return FinalCommitGateOutcome::Fail { reason, message };
            }
            match extract_top1_target(frame, spec) {
                Some((_, id)) => Some(id),
                None => match spec.on_target_lost {
                    VisionTargetLostPolicy::ZeroFeatures => Some(DetectorTargetIdentity::Absent),
                    VisionTargetLostPolicy::Hold => {
                        prepared.clear_chunk_buffer();
                        return FinalCommitGateOutcome::Hold;
                    }
                    VisionTargetLostPolicy::Stop => {
                        prepared.clear_chunk_buffer();
                        return FinalCommitGateOutcome::Fail {
                            reason: "TARGET_LOST",
                            message: match spec.target_label {
                                Some(ref label) => format!(
                                    "Target '{label}' lost in vision frame {} before step commit",
                                    frame.frame_id
                                ),
                                None => format!(
                                    "No valid detection above min_score {} in frame {} before step commit",
                                    spec.min_score, frame.frame_id
                                ),
                            },
                        };
                    }
                },
            }
        } else {
            vision_target
        }
    } else {
        vision_target
    };

    // 2. Current-tick embedding `InferenceTicket` verification at `gate_now` (preserving
    //    `is_unbroken_active_renewal_at`, `on_stale=hold`, `ClockHandshakeExpired`, and `Stop`).
    let embedding_clock_override_ms = hub_guard.and_then(HubCommitGuard::test_now_ms);
    let eval_now_ms = embedding_clock_override_ms.unwrap_or_else(|| monotonic_ms_at(gate_now));
    let commit_ticket_ctx = hub_guard.map(|guard| {
        let live_hub = guard.live_ticket_state_at(eval_now_ms);
        TicketVerificationContext {
            policy_instance_id: live_hub.policy_instance_id,
            lease_generation: live_lease,
            producer_epoch: live_hub.producer_epoch,
            model_generation: live_hub.model_generation,
            invalidation_generation: live_hub.invalidation_generation,
            clock_handshake_generation: live_hub.clock_handshake_generation,
            now_armd_ms: eval_now_ms,
        }
    });
    let hold_on_stale =
        video_embedding_spec.is_some_and(|s| s.on_stale == VisionStalePolicy::Hold);
    if let (Some(ticket), Some(guard), Some(ref ctx)) =
        (emb_ticket, hub_guard, &commit_ticket_ctx)
    {
        match ticket.verify(ctx) {
            Ok(()) => {}
            Err(TicketViolation::ClockHandshakeGenerationMismatch { expected, actual }) => {
                prepared.clear_chunk_buffer();
                if actual != 0 && guard.is_unbroken_active_renewal_at(expected, eval_now_ms) {
                    return FinalCommitGateOutcome::Hold;
                }
                if eval_now_ms > ticket.clock_handshake_expires_at_ms() {
                    let exp = TicketViolation::ClockHandshakeExpired {
                        now_ms: eval_now_ms,
                        expires_at_ms: ticket.clock_handshake_expires_at_ms(),
                    };
                    return FinalCommitGateOutcome::Fail {
                        reason: exp.code(),
                        message: exp.message(),
                    };
                }
                let v = TicketViolation::ClockHandshakeGenerationMismatch { expected, actual };
                return FinalCommitGateOutcome::Fail {
                    reason: v.code(),
                    message: v.message(),
                };
            }
            Err(
                TicketViolation::WindowNewestExpired { .. }
                | TicketViolation::WindowOldestExpired { .. },
            ) if hold_on_stale => {
                prepared.clear_chunk_buffer();
                return FinalCommitGateOutcome::Hold;
            }
            Err(violation) => {
                prepared.clear_chunk_buffer();
                return FinalCommitGateOutcome::Fail {
                    reason: violation.code(),
                    message: violation.message(),
                };
            }
        }
    }

    // 3. Final `commit_gate(gate_now, ...)` slot selection, provenance, and source-age bounds.
    let gate_params = CommitGateParams {
        max_fresh_joint_age_ms,
        max_cached_joint_age_ms: max_source_joint_age_ms,
        max_fresh_vision_age_ms,
        max_cached_vision_age_ms: max_source_vision_age_ms,
        horizon_ms,
        vision_generation: locked_generation,
        vision_target: commit_vision_target,
        live_ticket_ctx: commit_ticket_ctx,
        embedding_clock_override_ms,
    };
    match prepared.commit_gate(
        gate_now,
        interval_origin_std,
        period_ns,
        last_committed_period,
        &gate_params,
    ) {
        Ok(CommitGateDecision::Submit {
            target,
            commit_period,
        }) => {
            let input_age_ms = base_input_age_ms.map(|base| {
                base.saturating_add(
                    gate_now
                        .saturating_duration_since(captured_at)
                        .as_millis()
                        .min(u64::MAX as u128) as u64,
                )
            });
            let post_vision_age_ms = if let Some((mapped_ms, unc_ms)) = emb_mapped_and_unc_ms {
                Some(eval_now_ms.saturating_sub(mapped_ms).saturating_add(unc_ms))
            } else {
                vision_captured_at.map(|cap| {
                    gate_now
                        .saturating_duration_since(cap)
                        .as_millis()
                        .min(u64::MAX as u128) as u64
                })
            };
            FinalCommitGateOutcome::Submit {
                target,
                commit_period,
                gate_now,
                eval_now_ms,
                input_age_ms,
                post_vision_age_ms,
            }
        }
        Ok(CommitGateDecision::Skip { .. }) => FinalCommitGateOutcome::Skip,
        Err(e) => {
            prepared.clear_chunk_buffer();
            let reason = match e {
                PolicyError::NonFinite(_) => "NON_FINITE",
                _ => "INFERENCE_ERROR",
            };
            FinalCommitGateOutcome::Fail {
                reason,
                message: e.to_string(),
            }
        }
    }
}

pub(crate) fn verify_embedding_release_gate(
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
            let (max_vision_age_ms, hold_on_stale) = match (&video_embedding_spec, &vision_spec) {
                (Some(emb), _) => (Some(emb.effective_staleness_ms_for_fps(clamped_fps)), emb.on_stale == VisionStalePolicy::Hold),
                (None, Some(spec)) => (Some(spec.max_staleness_ms), spec.on_stale == VisionStalePolicy::Hold),
                (None, None) => (None, false),
            };
            inner.set_policy_max_vision_age(action_id, max_vision_age_ms, hold_on_stale);
            let authorization = inner.policy_run.as_ref().map(|r| r.authorization.clone());

            embedding_hub
                .as_ref()
                .zip(video_embedding_spec.as_ref())
                .map(|(hub, spec)| {
                    hub.bind_policy(action_id, spec.clone(), clamped_fps);
                    if let Some(a) = authorization { hub.bind_target_authorization(action_id, a); }
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
        let mut locked_continuity_epoch: Option<u64> = embedding_hub
            .as_ref()
            .and_then(|h| h.active_continuity_epoch());

        let interval_origin = tokio::time::Instant::now();
        let interval_origin_std = interval_origin.into_std();
        let mut interval = tokio::time::interval_at(interval_origin, period);
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        let period_ns = period.as_nanos().max(1);
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

            let (measured, prop_age_range_ms, oldest_input_at, captured_at, snap_lease_gen, run_ctx) = {
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
                let now_sample = Instant::now();
                let (age_range, oldest) = match inner.policy_input_age_and_oldest_at(now_sample) {
                    Some((range, oldest)) => (Some(range), oldest),
                    None => (None, now_sample),
                };
                (
                    inner.measured_positions.clone(),
                    age_range,
                    oldest,
                    now_sample,
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
                emb_mapped_and_unc_ms,
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
                match hub.sample_window_with_continuity(
                    emb_spec,
                    emb_run_started_ms,
                    locked_continuity_epoch,
                ) {
                    EmbeddingSampleDecision::Ready(win) => {
                        locked_continuity_epoch.get_or_insert(win.handshake_continuity_epoch);
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
                            Some((win.newest_oldest_mapped_at_ms, win.effective_uncertainty_ms)),
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
                joint_captured_at: oldest_input_at,
                joint_base_age_ms: None,
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

            let infer_handle = tokio::task::spawn_blocking(move || {
                let t0 = Instant::now();
                let res = prepared.plan_chunk_step(
                    &measured,
                    vision_features.as_deref(),
                    cur_period,
                    chunk_source,
                    Some(&freshness_limits),
                );
                let infer_us = t0.elapsed().as_micros().min(u64::MAX as u128) as u64;
                (prepared, res.map(|_| infer_us))
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

            let infer_us = match infer_result {
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

            if let Some(gate) = active_gate {
                if let Some(ref emb) = video_embedding_spec {
                    if let Err(msg) = verify_embedding_release_gate(&gate, emb) {
                        prepared.clear_chunk_buffer();
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
                    prepared.clear_chunk_buffer();
                    inner.fail_policy_run(action_id, "DEPLOYMENT_CHANGED", e.to_string());
                    break;
                }
            }

            // Final critical-section check (A-R2-1): hold `HubCommitGuard` together with
            // `inner`, read `gate_now` and `collector.latest()` ONCE, enforce current-tick
            // detector/ticket fences + `commit_gate`, and immediately submit to `policy_step_with_vision`.
            let _hub_guard = embedding_hub.as_ref().map(|hub| hub.lock_for_commit());
            let live_lease = active_run_lease_generation(&inner, action_id);
            let (target, committed_period, input_age_ms, post_vision_age_ms) =
                match evaluate_final_commit_gate(
                    &mut prepared,
                    deadline,
                    interval_origin_std,
                    period_ns,
                    last_committed_period,
                    max_fresh_joint_age_ms,
                    max_source_joint_age_ms,
                    max_fresh_vision_age_ms,
                    max_source_vision_age_ms,
                    horizon_ms,
                    video_embedding_spec.as_ref(),
                    vision_spec.as_ref(),
                    _hub_guard.as_ref(),
                    live_lease,
                    collector.as_ref(),
                    locked_generation,
                    base_input_age_ms,
                    captured_at,
                    vision_captured_at,
                    vision_target,
                    emb_ticket.as_ref(),
                    emb_mapped_and_unc_ms,
                ) {
                    FinalCommitGateOutcome::Submit {
                        target,
                        commit_period,
                        input_age_ms,
                        post_vision_age_ms,
                        ..
                    } => (target, commit_period, input_age_ms, post_vision_age_ms),
                    FinalCommitGateOutcome::Skip => continue,
                    FinalCommitGateOutcome::Hold => {
                        inner.hold_policy_run(action_id);
                        continue;
                    }
                    FinalCommitGateOutcome::Finish(reason) => {
                        inner.finish_policy_run(action_id, reason);
                        break;
                    }
                    FinalCommitGateOutcome::Fail { reason, message } => {
                        inner.fail_policy_run(action_id, reason, message);
                        break;
                    }
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
            if is_chunked {
                last_committed_period = Some(committed_period);
            }
        }
    })
}

/// Parameters for the RT-executor off-thread policy runner (`OWNER_EXECUTOR`).
#[cfg(not(target_arch = "wasm32"))]
pub struct ExecutorPolicyRunParams {
    pub supervisor: Arc<ArmSupervisor>,
    pub handle: crate::rt_core::executor::RtExecutorHandle,
    pub client: crate::rt_core::broker::ClientId,
    pub action_id: u64,
    pub lease_generation: u64,
    pub epoch_id: u64,
    pub mode: PolicyMode,
    pub prepared: PreparedPolicy,
    pub fps: f64,
    pub deadline: Option<Instant>,
    pub vision_source: Option<Arc<dyn PolicyVisionSource>>,
    pub sandbox: Option<Arc<crate::api::sandbox::SandboxState>>,
    pub verified_release_gate: Option<VerifiedReleaseGate>,
    pub policy_live_allowed: bool,
    pub has_driver: bool,
    pub token_guard: crate::rt_core::requests::policy::PolicyTokenGuard,
    pub pending_qual_guard: Option<crate::rt_core::requests::policy::PendingQualGuard>,
}

#[cfg(not(target_arch = "wasm32"))]
enum ExecutorCommitOutcome {
    Submitted { commit_period: u64 },
    Skip,
    Hold,
    RtEnded,
    Finish(&'static str),
    Fail { reason: &'static str, message: String },
}

#[cfg(not(target_arch = "wasm32"))]
#[allow(clippy::too_many_arguments)]
async fn send_executor_end_policy(
    handle: &crate::rt_core::executor::RtExecutorHandle,
    supervisor: &ArmSupervisor,
    client: crate::rt_core::broker::ClientId,
    client_seq: &mut u64,
    lease_generation: u64,
    epoch_id: u64,
    token_guard: &crate::rt_core::requests::policy::PolicyTokenGuard,
    spec: crate::rt_core::requests::policy::EndPolicySpec,
) -> bool {
    use crate::rt_core::broker::{AdmitError, Authority, Reply};
    use crate::rt_core::executor::BrokerDispatchError;
    use crate::rt_core::requests::policy::PolicyCommand;

    for attempt in 0..12u32 {
        *client_seq = client_seq.wrapping_add(1);
        let authority = Authority {
            lease_generation,
            epoch: epoch_id,
            stop_generation: supervisor.current_stop_gen(),
            config_revision: 0,
            parent_generation: 0,
        };
        let deadline = Instant::now() + Duration::from_secs(2);
        match handle
            .submit_policy_command_async(
                client,
                *client_seq,
                authority,
                deadline,
                PolicyCommand::EndPolicy(spec),
                |req_id| token_guard.record_admitted_req(req_id.0),
            )
            .await
        {
            Ok(Reply::Ack { .. }) => return true,
            Ok(Reply::Refused {
                code: "POLICY_NOT_ACTIVE" | "LEASE_LOST" | "EPOCH_MISMATCH" | "STOPPED",
                ..
            }) => return true,
            Ok(Reply::Refused { .. } | Reply::Lease { .. } | Reply::Terminal { .. }) => {
                return false;
            }
            Err(BrokerDispatchError::Admit(
                AdmitError::CommandsFull | AdmitError::RepliesFull,
            )) if attempt + 1 < 12 => {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
            Err(
                BrokerDispatchError::Admit(AdmitError::Disconnected)
                | BrokerDispatchError::Stopped,
            ) => return true,
            Err(_) => return false,
        }
    }
    false
}

/// Single post-inference commit gate for the RT-executor policy runner: checks deadline,
/// detector staleness & generation, deployment/release-gate binding, `InferenceTicket` under
/// `HubCommitGuard`, runs the single final `commit_gate_now` (slot selection, source-age
/// bounds, target/ticket provenance), and then either updates off-RT Shadow diagnostics
/// (0 mailbox submissions) or submits one live frame synchronously via
/// `TargetPort::submit_policy_with_validity`.
#[cfg(not(target_arch = "wasm32"))]
#[allow(clippy::too_many_arguments)]
async fn executor_commit_gate(
    supervisor: &ArmSupervisor,
    handle: &crate::rt_core::executor::RtExecutorHandle,
    prepared: &mut PreparedPolicy,
    action_id: u64,
    lease_generation: u64,
    mode: PolicyMode,
    deadline: Option<Instant>,
    max_input_age: Duration,
    interval_origin_std: Instant,
    period_ns: u128,
    last_committed_period: Option<u64>,
    max_fresh_joint_age_ms: u64,
    max_source_joint_age_ms: u64,
    max_fresh_vision_age_ms: Option<u64>,
    max_source_vision_age_ms: Option<u64>,
    horizon_ms: u64,
    video_embedding_spec: Option<&VideoEmbeddingSpec>,
    vision_spec: Option<&super::manifest::VisionFeatureSpec>,
    embedding_hub: Option<&Arc<VideoEmbeddingHub>>,
    collector: Option<&VisionFeatureCollector>,
    locked_generation: Option<u64>,
    verified_release_gate: Option<&VerifiedReleaseGate>,
    sandbox: Option<&Arc<crate::api::sandbox::SandboxState>>,
    snap_authority: crate::rt_core::broker::Authority,
    authorization: &PolicyAuthorization,
    snap_auth_gen: u64,
    last_seen_refusal_gen: &mut u64,
    oldest_input_at: Instant,
    base_input_age_ms: Option<u64>,
    captured_at: Instant,
    vision_captured_at: Option<Instant>,
    vision_frame_id: Option<u64>,
    vision_target: Option<DetectorTargetIdentity>,
    emb_ticket: Option<&InferenceTicket>,
    emb_mapped_and_unc_ms: Option<(u64, u64)>,
    infer_us: u64,
    step_us: u64,
    shadow_steps: &mut u64,
    step_stats: &mut PolicyStepStats,
) -> ExecutorCommitOutcome {
    use crate::rt_core::mailbox::Submitted;
    use crate::rt_core::publish::PolicyDiagUpdate;
    use crate::supervisor::targets::{PolicyStamp, PolicyTarget};

    if let Some(dl) = deadline {
        if Instant::now() >= dl {
            prepared.clear_chunk_buffer();
            return ExecutorCommitOutcome::Finish("MAX_SECONDS");
        }
    }

    if let Some(gate) = verified_release_gate {
        if let Some(emb) = video_embedding_spec {
            if let Err(msg) = verify_embedding_release_gate(gate, emb) {
                prepared.clear_chunk_buffer();
                return ExecutorCommitOutcome::Fail {
                    reason: "RELEASE_GATE_INVALID",
                    message: msg,
                };
            }
        }
        let (scene_id, tool_id) = match sandbox {
            Some(s) => s.active_scene_and_tool().await,
            None => (gate.scene_id().to_string(), gate.tool_id().to_string()),
        };
        let calibration_digest = supervisor
            .published_config_hub()
            .latest()
            .map(|c| c.active_calibration_digest())
            .unwrap_or_default();
        let cur_ctx = super::release_gate::ActiveDeploymentContext {
            scene_id,
            tool_id,
            calibration_digest,
        };
        if let Err(e) = gate.verify_deployment(&cur_ctx) {
            prepared.clear_chunk_buffer();
            return ExecutorCommitOutcome::Fail {
                reason: "DEPLOYMENT_CHANGED",
                message: e.to_string(),
            };
        }
    }

    // Synchronous critical section: hold `HubCommitGuard` across `evaluate_final_commit_gate`
    // and immediate publication (for Live mode, inside `handle.submit_policy_gated` so no
    // dispatcher mutex stall can happen between `gate_now` slot selection and triple-buffer
    // publish; NO `.await` while held!).
    let hub_guard = embedding_hub.map(|hub| hub.lock_for_commit());

    if mode == PolicyMode::Shadow {
        let mirror_lease = supervisor.lease_generation();
        let fenced_lease = supervisor.atomic_fenced_lease_gen();
        let hub_active = supervisor.hub().latest().is_some_and(|s| {
            s.obs
                .policy
                .as_ref()
                .is_some_and(|p| p.active && p.action_id == action_id)
        });
        let live_lease = (hub_active
            && mirror_lease == lease_generation
            && fenced_lease < lease_generation)
            .then_some(lease_generation);
        let outcome = evaluate_final_commit_gate(
            prepared,
            deadline,
            interval_origin_std,
            period_ns,
            last_committed_period,
            max_fresh_joint_age_ms,
            max_source_joint_age_ms,
            max_fresh_vision_age_ms,
            max_source_vision_age_ms,
            horizon_ms,
            video_embedding_spec,
            vision_spec,
            hub_guard.as_ref(),
            live_lease,
            collector,
            locked_generation,
            base_input_age_ms,
            captured_at,
            vision_captured_at,
            vision_target,
            emb_ticket,
            emb_mapped_and_unc_ms,
        );
        drop(hub_guard);
        return match outcome {
            FinalCommitGateOutcome::Submit {
                target,
                commit_period,
                input_age_ms,
                post_vision_age_ms,
                ..
            } => {
                if !hub_active || handle.is_policy_closed(action_id) {
                    prepared.clear_chunk_buffer();
                    return ExecutorCommitOutcome::RtEnded;
                }
                *shadow_steps = shadow_steps.saturating_add(1);
                step_stats.record(infer_us, step_us);
                supervisor.hub().update_policy_diag(PolicyDiagUpdate {
                    action_id,
                    is_shadow: true,
                    steps: *shadow_steps,
                    stats: step_stats.report(),
                    last_input_age_ms: input_age_ms,
                    last_vision_age_ms: post_vision_age_ms,
                    vision_frame_id,
                    last_shadow_target: Some(target),
                });
                ExecutorCommitOutcome::Submitted { commit_period }
            }
            FinalCommitGateOutcome::Skip => {
                step_stats.record(infer_us, step_us);
                ExecutorCommitOutcome::Skip
            }
            FinalCommitGateOutcome::Hold => ExecutorCommitOutcome::Hold,
            FinalCommitGateOutcome::Finish(reason) => ExecutorCommitOutcome::Finish(reason),
            FinalCommitGateOutcome::Fail { reason, message } => {
                ExecutorCommitOutcome::Fail { reason, message }
            }
        };
    }

    // Live mode: run `evaluate_final_commit_gate` and `submit` atomically under BOTH `hub_guard`
    // and the target-dispatcher mutex (`submit_policy_gated`).
    let outcome = handle.submit_policy_gated(action_id, |is_closed, has_pending_refusal, submit| {
        let mirror_lease = supervisor.lease_generation();
        let fenced_lease = supervisor.atomic_fenced_lease_gen();
        let hub_active = supervisor.hub().latest().is_some_and(|s| {
            s.obs
                .policy
                .as_ref()
                .is_some_and(|p| p.active && p.action_id == action_id)
        });
        let live_lease = (hub_active
            && mirror_lease == lease_generation
            && fenced_lease < lease_generation)
            .then_some(lease_generation);
        if is_closed || live_lease.is_none() {
            prepared.clear_chunk_buffer();
            return ExecutorCommitOutcome::RtEnded;
        }
        let cur_refusal_gen = handle.policy_refusal_gen();
        if has_pending_refusal
            || cur_refusal_gen != *last_seen_refusal_gen
            || authorization.current() != snap_auth_gen
        {
            *last_seen_refusal_gen = cur_refusal_gen;
            prepared.clear_chunk_buffer();
            step_stats.record(infer_us, step_us);
            return ExecutorCommitOutcome::Skip;
        }
        match evaluate_final_commit_gate(
            prepared,
            deadline,
            interval_origin_std,
            period_ns,
            last_committed_period,
            max_fresh_joint_age_ms,
            max_source_joint_age_ms,
            max_fresh_vision_age_ms,
            max_source_vision_age_ms,
            horizon_ms,
            video_embedding_spec,
            vision_spec,
            hub_guard.as_ref(),
            live_lease,
            collector,
            locked_generation,
            base_input_age_ms,
            captured_at,
            vision_captured_at,
            vision_target,
            emb_ticket,
            emb_mapped_and_unc_ms,
        ) {
            FinalCommitGateOutcome::Submit {
                target,
                commit_period,
                gate_now,
                eval_now_ms,
                post_vision_age_ms,
                ..
            } => {
                // Re-check lock-free RT refusal/revocation counters right before swapping into
                // the triple buffer; if RT refused a prior slot while `evaluate_final_commit_gate`
                // ran, clear the chunk ring now and do not publish.
                let post_refusal_gen = handle.policy_refusal_gen();
                if post_refusal_gen != cur_refusal_gen || authorization.current() != snap_auth_gen {
                    *last_seen_refusal_gen = post_refusal_gen;
                    prepared.clear_chunk_buffer();
                    step_stats.record(infer_us, step_us);
                    return ExecutorCommitOutcome::Skip;
                }
                let vision_at = if let Some(age_ms) =
                    post_vision_age_ms.filter(|_| video_embedding_spec.is_some())
                {
                    gate_now.checked_sub(Duration::from_millis(age_ms))
                } else {
                    vision_captured_at
                };
                let (vision_window_expires_at, mut handshake_expires_at) =
                    if let (Some(ticket), Some(_)) = (emb_ticket, &hub_guard) {
                        let win_dl_ms = ticket
                            .window_oldest_deadline_ms()
                            .min(ticket.window_newest_deadline_ms());
                        let win_rem = win_dl_ms.saturating_sub(eval_now_ms);
                        let hs_rem = ticket
                            .clock_handshake_expires_at_ms()
                            .saturating_sub(eval_now_ms);
                        (
                            gate_now.checked_add(Duration::from_millis(win_rem)),
                            gate_now.checked_add(Duration::from_millis(hs_rem)),
                        )
                    } else {
                        (None, None)
                    };
                if let Some(src_hs_exp) = prepared.last_commit_handshake_expires_at() {
                    handshake_expires_at = Some(
                        handshake_expires_at.map_or(src_hs_exp, |cur| cur.min(src_hs_exp)),
                    );
                }
                let validity = crate::rt_core::mailbox::PolicyValidityBounds {
                    vision_window_expires_at,
                    handshake_expires_at,
                    chunk_slot_expires_at: prepared.last_commit_slot_expires_at(),
                    chunk_period_end: prepared.last_commit_period_end(),
                    expected_refusal_gen: (prepared.contract().chunking.chunk_size > 1)
                        .then_some(cur_refusal_gen),
                };
                let policy_target = PolicyTarget {
                    action_id,
                    stamp: PolicyStamp {
                        authority: snap_authority,
                        authorization: snap_auth_gen,
                        max_input_age,
                    },
                    targets: &target,
                    input_at: oldest_input_at,
                    vision_at,
                    vision_frame_id,
                };
                let sub = submit(&policy_target, validity);
                match sub {
                    Submitted::Accepted | Submitted::Coalesced { .. } => {
                        step_stats.record(infer_us, step_us);
                        supervisor.hub().update_policy_diag(PolicyDiagUpdate {
                            action_id,
                            is_shadow: false,
                            steps: 0,
                            stats: step_stats.report(),
                            last_input_age_ms: None,
                            last_vision_age_ms: None,
                            vision_frame_id: None,
                            last_shadow_target: None,
                        });
                        ExecutorCommitOutcome::Submitted { commit_period }
                    }
                    Submitted::Busy => {
                        prepared.clear_chunk_buffer();
                        step_stats.record_skipped();
                        supervisor.hub().update_policy_diag(PolicyDiagUpdate {
                            action_id,
                            is_shadow: false,
                            steps: 0,
                            stats: step_stats.report(),
                            last_input_age_ms: None,
                            last_vision_age_ms: None,
                            vision_frame_id: None,
                            last_shadow_target: None,
                        });
                        ExecutorCommitOutcome::Skip
                    }
                    Submitted::OutOfOrder => {
                        prepared.clear_chunk_buffer();
                        if is_closed {
                            ExecutorCommitOutcome::RtEnded
                        } else {
                            ExecutorCommitOutcome::Fail {
                                reason: "POLICY_FAILED",
                                message: "Policy target stream closed or out of order".into(),
                            }
                        }
                    }
                    Submitted::StreamsFull => {
                        prepared.clear_chunk_buffer();
                        ExecutorCommitOutcome::Fail {
                            reason: "STREAMS_FULL",
                            message: "Target mailbox has too many open streams".into(),
                        }
                    }
                }
            }
            FinalCommitGateOutcome::Skip => {
                step_stats.record(infer_us, step_us);
                ExecutorCommitOutcome::Skip
            }
            FinalCommitGateOutcome::Hold => ExecutorCommitOutcome::Hold,
            FinalCommitGateOutcome::Finish(reason) => ExecutorCommitOutcome::Finish(reason),
            FinalCommitGateOutcome::Fail { reason, message } => {
                ExecutorCommitOutcome::Fail { reason, message }
            }
        }
    });
    drop(hub_guard);
    outcome
}

/// Spawn the off-thread policy runner for `OWNER_EXECUTOR`: zero `SupervisorLock` reads,
/// off-thread inference, `StateHub` input capture, synchronous `TargetPort::submit_policy`
/// under `HubCommitGuard` for Live runs, off-RT `StateHub` diagnostics for Shadow runs (0
/// mailbox submissions), and drop-safe `PolicyTokenGuard` ownership.
#[cfg(not(target_arch = "wasm32"))]
pub fn spawn_executor_policy_runner(params: ExecutorPolicyRunParams) -> tokio::task::JoinHandle<()> {
    use crate::model::HolderLabel;
    use crate::rt_core::requests::policy::EndPolicySpec;

    let ExecutorPolicyRunParams {
        supervisor,
        handle,
        client,
        action_id,
        lease_generation,
        epoch_id,
        mode,
        mut prepared,
        fps,
        deadline,
        vision_source,
        sandbox,
        verified_release_gate,
        policy_live_allowed,
        has_driver,
        mut token_guard,
        mut pending_qual_guard,
    } = params;

    let clamped_fps = if fps.is_finite() && fps > 0.0 {
        fps.clamp(MIN_POLICY_FPS, MAX_POLICY_FPS)
    } else {
        DEFAULT_POLICY_FPS
    };
    let period = Duration::from_secs_f64(1.0 / clamped_fps);
    let is_chunked = prepared.contract().chunking.chunk_size > 1;
    let replan_every = prepared.contract().chunking.replan_every;
    let horizon_ms = ((replan_every as f64) * (1000.0 / clamped_fps))
        .round()
        .max(0.0) as u64;
    let max_fresh_joint_age_ms = max_policy_input_age_ms(clamped_fps);
    let max_source_joint_age_ms = max_cached_chunk_source_age_ms(clamped_fps, replan_every);
    let max_input_age = Duration::from_millis(max_fresh_joint_age_ms);
    let tokio_dl = deadline.map(tokio::time::Instant::from_std);
    let load_token = token_guard.token();

    tokio::spawn(async move {
        struct ClosePolicyGuard {
            handle: crate::rt_core::executor::RtExecutorHandle,
            action_id: u64,
        }
        impl Drop for ClosePolicyGuard {
            fn drop(&mut self) {
                self.handle.close_policy(self.action_id);
            }
        }
        struct UnbindGuard {
            hub: Arc<VideoEmbeddingHub>,
            action_id: u64,
        }
        impl Drop for UnbindGuard {
            fn drop(&mut self) {
                self.hub.unbind_policy(self.action_id);
            }
        }

        let _close_guard = ClosePolicyGuard {
            handle: handle.clone(),
            action_id,
        };
        let mut client_seq: u64 = 1u64 << 60;
        let mut step_stats = PolicyStepStats::default();
        step_stats.set_period_us(period_us_for_fps(clamped_fps));
        let mut shadow_steps: u64 = 0;

        let video_embedding_spec = prepared.contract().video_embedding.clone();
        let vision_spec = prepared.contract().vision.clone();
        let max_fresh_vision_age_ms = vision_spec.as_ref().map(|s| s.max_staleness_ms);
        let max_source_vision_age_ms = vision_spec
            .as_ref()
            .map(|s| s.max_staleness_ms.saturating_add(horizon_ms));
        let authorization = supervisor.rt_policy_authorization();

        if let Err(err) = validate_chunking_horizon(prepared.contract().chunking, clamped_fps) {
            if send_executor_end_policy(
                &handle,
                &supervisor,
                client,
                &mut client_seq,
                lease_generation,
                epoch_id,
                &token_guard,
                EndPolicySpec::Fail {
                    action_id,
                    load_token,
                    lease_generation,
                    reason: HolderLabel::new("INVALID_POLICY"),
                    message: HolderLabel::new(&err.to_string()),
                    final_steps: shadow_steps,
                    final_stats: step_stats.report(),
                },
            )
            .await
            {
                token_guard.disarm();
            }
            return;
        }

        if video_embedding_spec.is_none() && vision_spec.is_some() && vision_source.is_none() {
            if send_executor_end_policy(
                &handle,
                &supervisor,
                client,
                &mut client_seq,
                lease_generation,
                epoch_id,
                &token_guard,
                EndPolicySpec::Fail {
                    action_id,
                    load_token,
                    lease_generation,
                    reason: HolderLabel::new("VISION_UNAVAILABLE"),
                    message: HolderLabel::new(
                        "Policy requires vision input, but no vision source is available",
                    ),
                    final_steps: shadow_steps,
                    final_stats: step_stats.report(),
                },
            )
            .await
            {
                token_guard.disarm();
            }
            return;
        }

        if let (Some(ref emb), Some(ref gate)) = (&video_embedding_spec, &verified_release_gate) {
            if let Err(msg) = verify_embedding_release_gate(gate, emb) {
                if send_executor_end_policy(
                    &handle,
                    &supervisor,
                    client,
                    &mut client_seq,
                    lease_generation,
                    epoch_id,
                    &token_guard,
                    EndPolicySpec::Fail {
                        action_id,
                        load_token,
                        lease_generation,
                        reason: HolderLabel::new("RELEASE_GATE_INVALID"),
                        message: HolderLabel::new(&msg),
                        final_steps: shadow_steps,
                        final_stats: step_stats.report(),
                    },
                )
                .await
                {
                    token_guard.disarm();
                }
                return;
            }
        }

        let embedding_hub = video_embedding_spec
            .as_ref()
            .map(|_| embedding_hub_for_supervisor(&supervisor));
        let mut _unbind_guard: Option<UnbindGuard> = None;
        if let (Some(hub), Some(spec)) = (embedding_hub.as_ref(), video_embedding_spec.as_ref()) {
            // Fence hub binding by monotonic action_id and live stream status so a delayed old
            // runner A can never overwrite or unbind a newer active runner B.
            if handle.is_policy_closed(action_id)
                || !hub.bind_policy_fenced(
                    action_id,
                    spec.clone(),
                    clamped_fps,
                    Some(authorization.clone()),
                )
            {
                token_guard.disarm();
                return;
            }
            _unbind_guard = Some(UnbindGuard {
                hub: hub.clone(),
                action_id,
            });
        }
        let emb_run_started_ms = embedding_hub.as_ref().map_or(0, |h| h.now_armd_ms());

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
        let mut locked_continuity_epoch: Option<u64> = embedding_hub
            .as_ref()
            .and_then(|h| h.active_continuity_epoch());

        let interval_origin = tokio::time::Instant::now();
        let interval_origin_std = interval_origin.into_std();
        let mut interval = tokio::time::interval_at(interval_origin, period);
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        let period_ns = period.as_nanos().max(1);
        let mut last_committed_period: Option<u64> = None;
        let mut last_auth_gen: Option<u64> = None;
        let mut last_seen_refusal_gen = handle.policy_refusal_gen();
        let stop_requests = handle.stop_requests().clone();
        let initial_stop_gen = supervisor.current_stop_gen();

        loop {
            if let Some(dl) = tokio_dl {
                tokio::select! {
                    biased;
                    _ = tokio::time::sleep_until(dl) => {
                        prepared.clear_chunk_buffer();
                        if send_executor_end_policy(
                            &handle, &supervisor, client, &mut client_seq,
                            lease_generation, epoch_id, &token_guard,
                            EndPolicySpec::Finish {
                                action_id,
                                load_token,
                                lease_generation,
                                reason: HolderLabel::new("MAX_SECONDS"),
                                final_steps: shadow_steps,
                                final_stats: step_stats.report(),
                            },
                        ).await {
                            token_guard.disarm();
                        }
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
                    if send_executor_end_policy(
                        &handle,
                        &supervisor,
                        client,
                        &mut client_seq,
                        lease_generation,
                        epoch_id,
                        &token_guard,
                        EndPolicySpec::Finish {
                            action_id,
                            load_token,
                            lease_generation,
                            reason: HolderLabel::new("MAX_SECONDS"),
                            final_steps: shadow_steps,
                            final_stats: step_stats.report(),
                        },
                    )
                    .await
                    {
                        token_guard.disarm();
                    }
                    break;
                }
            }

            let outcomes = handle.drain_policy_outcomes(action_id);
            let cur_refusal_gen = handle.policy_refusal_gen();
            if outcomes.iter().any(|o| o.result.is_err()) || cur_refusal_gen != last_seen_refusal_gen {
                last_seen_refusal_gen = cur_refusal_gen;
                prepared.clear_chunk_buffer();
            }
            if handle.is_policy_closed(action_id) {
                prepared.clear_chunk_buffer();
                token_guard.disarm();
                break;
            }
            let Some(snap) = supervisor.hub().latest() else {
                continue;
            };
            let active_matches = snap
                .obs
                .policy
                .as_ref()
                .is_some_and(|p| p.active && p.action_id == action_id);
            if !active_matches {
                prepared.clear_chunk_buffer();
                token_guard.disarm();
                break;
            }

            let captured_at = Instant::now();
            let measured = snap.rt.positions.clone();
            let snap_authority = snap.rt.owner.authority;
            let snap_auth_gen = authorization.current();
            let dof = prepared.contract().action_dim;

            let (oldest_input_at, prop_age_range_ms) = if has_driver {
                let all_present = snap.rt.feedback_instants.len() == dof
                    && snap.rt.feedback_instants.iter().all(Option::is_some);
                if all_present {
                    let mut oldest = captured_at;
                    let mut newest: Option<Instant> = None;
                    for t in snap.rt.feedback_instants.iter().flatten().copied() {
                        if t < oldest {
                            oldest = t;
                        }
                        newest = Some(match newest {
                            Some(n) if n >= t => n,
                            _ => t,
                        });
                    }
                    let min_ms = newest.map_or(0, |n| {
                        captured_at
                            .saturating_duration_since(n)
                            .as_millis()
                            .min(u64::MAX as u128) as u64
                    });
                    let max_ms = captured_at
                        .saturating_duration_since(oldest)
                        .as_millis()
                        .min(u64::MAX as u128) as u64;
                    (oldest, Some((min_ms, max_ms)))
                } else if mode == PolicyMode::Live && !policy_live_allowed {
                    prepared.clear_chunk_buffer();
                    if send_executor_end_policy(
                        &handle,
                        &supervisor,
                        client,
                        &mut client_seq,
                        lease_generation,
                        epoch_id,
                        &token_guard,
                        EndPolicySpec::Fail {
                            action_id,
                            load_token,
                            lease_generation,
                            reason: HolderLabel::new("STALE_INPUT"),
                            message: HolderLabel::new(
                                "Real-arm joint feedback timestamp unavailable",
                            ),
                            final_steps: shadow_steps,
                            final_stats: step_stats.report(),
                        },
                    )
                    .await
                    {
                        token_guard.disarm();
                    }
                    break;
                } else {
                    (captured_at, None)
                }
            } else {
                (captured_at, Some((0, 0)))
            };
            let base_input_age_ms = prop_age_range_ms.map(|(_, hi)| hi);
            let snap_lease_gen = (snap_authority.lease_generation == lease_generation
                && supervisor.lease_generation() == lease_generation
                && supervisor.atomic_fenced_lease_gen() < lease_generation)
                .then_some(lease_generation);
            let snap_mode = snap.obs.policy.as_ref().map_or(mode, |p| {
                if p.mode == "live" {
                    PolicyMode::Live
                } else {
                    PolicyMode::Shadow
                }
            });
            let cur_epoch = snap.obs.epoch_id.max(supervisor.estop_epoch());
            let active_ok = active_matches
                && !stop_requests.pending()
                && supervisor.current_stop_gen() == initial_stop_gen
                && !matches!(
                    snap.obs.safety_state,
                    SafetyState::EStop | SafetyState::Fault(_)
                )
                && cur_epoch == epoch_id
                && snap_authority.epoch == epoch_id;
            let run_ctx = ChunkRunContext {
                mode: snap_mode,
                epoch_id: cur_epoch,
                lease_generation: snap_lease_gen,
                active_ok,
            };
            prepared.sync_run_context(run_ctx);
            if last_auth_gen.is_some_and(|prev| prev != snap_auth_gen) {
                prepared.clear_chunk_buffer();
            }
            last_auth_gen = Some(snap_auth_gen);

            let (
                vision_features,
                vision_captured_at,
                vision_frame_id,
                vision_target,
                emb_ticket,
                emb_mapped_and_unc_ms,
            ) = if let (Some(emb_spec), Some(hub)) = (&video_embedding_spec, &embedding_hub) {
                let Some(lease_gen) = snap_lease_gen else {
                    prepared.clear_chunk_buffer();
                    if send_executor_end_policy(
                        &handle,
                        &supervisor,
                        client,
                        &mut client_seq,
                        lease_generation,
                        epoch_id,
                        &token_guard,
                        EndPolicySpec::Fail {
                            action_id,
                            load_token,
                            lease_generation,
                            reason: HolderLabel::new("LEASE_LOST"),
                            message: HolderLabel::new(
                                "Control lease lost or expired before embedding step",
                            ),
                            final_steps: shadow_steps,
                            final_stats: step_stats.report(),
                        },
                    )
                    .await
                    {
                        token_guard.disarm();
                    }
                    break;
                };
                match hub.sample_window_with_continuity(
                    emb_spec,
                    emb_run_started_ms,
                    locked_continuity_epoch,
                ) {
                    EmbeddingSampleDecision::Ready(win) => {
                        locked_continuity_epoch.get_or_insert(win.handshake_continuity_epoch);
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
                            Some((win.newest_oldest_mapped_at_ms, win.effective_uncertainty_ms)),
                        )
                    }
                    EmbeddingSampleDecision::Hold => {
                        prepared.clear_chunk_buffer();
                        let _ = authorization.revoke_for_action(action_id);
                        let _ = send_executor_end_policy(
                            &handle,
                            &supervisor,
                            client,
                            &mut client_seq,
                            lease_generation,
                            epoch_id,
                            &token_guard,
                            EndPolicySpec::Hold {
                                action_id,
                                load_token,
                                lease_generation,
                            },
                        )
                        .await;
                        continue;
                    }
                    EmbeddingSampleDecision::Stop { reason, message } => {
                        prepared.clear_chunk_buffer();
                        if send_executor_end_policy(
                            &handle,
                            &supervisor,
                            client,
                            &mut client_seq,
                            lease_generation,
                            epoch_id,
                            &token_guard,
                            EndPolicySpec::Fail {
                                action_id,
                                load_token,
                                lease_generation,
                                reason: HolderLabel::new(reason),
                                message: HolderLabel::new(&message),
                                final_steps: shadow_steps,
                                final_stats: step_stats.report(),
                            },
                        )
                        .await
                        {
                            token_guard.disarm();
                        }
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
                        let _ = authorization.revoke_for_action(action_id);
                        let _ = send_executor_end_policy(
                            &handle,
                            &supervisor,
                            client,
                            &mut client_seq,
                            lease_generation,
                            epoch_id,
                            &token_guard,
                            EndPolicySpec::Hold {
                                action_id,
                                load_token,
                                lease_generation,
                            },
                        )
                        .await;
                        continue;
                    }
                    VisionStepDecision::Stop { reason, message } => {
                        prepared.clear_chunk_buffer();
                        if send_executor_end_policy(
                            &handle,
                            &supervisor,
                            client,
                            &mut client_seq,
                            lease_generation,
                            epoch_id,
                            &token_guard,
                            EndPolicySpec::Fail {
                                action_id,
                                load_token,
                                lease_generation,
                                reason: HolderLabel::new(reason),
                                message: HolderLabel::new(&message),
                                final_steps: shadow_steps,
                                final_stats: step_stats.report(),
                            },
                        )
                        .await
                        {
                            token_guard.disarm();
                        }
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
                joint_captured_at: oldest_input_at,
                joint_base_age_ms: None,
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

            let infer_permit = {
                let perm_fut = supervisor.policy_backend_permits().clone().acquire_owned();
                if let Some(dl) = tokio_dl {
                    tokio::select! {
                        biased;
                        _ = tokio::time::sleep_until(dl) => {
                            prepared.clear_chunk_buffer();
                            if send_executor_end_policy(
                                &handle, &supervisor, client, &mut client_seq,
                                lease_generation, epoch_id, &token_guard,
                                EndPolicySpec::Finish {
                                    action_id,
                                    load_token,
                                    lease_generation,
                                    reason: HolderLabel::new("MAX_SECONDS"),
                                    final_steps: shadow_steps,
                                    final_stats: step_stats.report(),
                                },
                            ).await {
                                token_guard.disarm();
                            }
                            break;
                        }
                        res = perm_fut => match res {
                            Ok(p) => p,
                            Err(_) => break,
                        },
                    }
                } else {
                    match perm_fut.await {
                        Ok(p) => p,
                        Err(_) => break,
                    }
                }
            };
            let infer_handle = tokio::task::spawn_blocking(move || {
                let _infer_permit = infer_permit;
                let t0 = Instant::now();
                let res = prepared.plan_chunk_step(
                    &measured,
                    vision_features.as_deref(),
                    cur_period,
                    chunk_source,
                    Some(&freshness_limits),
                );
                let infer_us = t0.elapsed().as_micros().min(u64::MAX as u128) as u64;
                (prepared, res.map(|_| infer_us))
            });
            let infer_result = if let Some(dl) = tokio_dl {
                tokio::select! {
                    biased;
                    _ = tokio::time::sleep_until(dl) => {
                        if send_executor_end_policy(
                            &handle, &supervisor, client, &mut client_seq,
                            lease_generation, epoch_id, &token_guard,
                            EndPolicySpec::Finish {
                                action_id,
                                load_token,
                                lease_generation,
                                reason: HolderLabel::new("MAX_SECONDS"),
                                final_steps: shadow_steps,
                                final_stats: step_stats.report(),
                            },
                        ).await {
                            token_guard.disarm();
                        }
                        break;
                    }
                    res = infer_handle => res,
                }
            } else {
                infer_handle.await
            };

            let infer_us = match infer_result {
                Err(e) => {
                    tracing::warn!("Policy inference task failed for action {action_id}: {e}");
                    if send_executor_end_policy(
                        &handle,
                        &supervisor,
                        client,
                        &mut client_seq,
                        lease_generation,
                        epoch_id,
                        &token_guard,
                        EndPolicySpec::Fail {
                            action_id,
                            load_token,
                            lease_generation,
                            reason: HolderLabel::new("INFERENCE_ERROR"),
                            message: HolderLabel::new(&format!("inference task failed: {e}")),
                            final_steps: shadow_steps,
                            final_stats: step_stats.report(),
                        },
                    )
                    .await
                    {
                        token_guard.disarm();
                    }
                    break;
                }
                Ok((p, res)) => {
                    prepared = p;
                    match res {
                        Ok(t) => t,
                        Err(e) => {
                            prepared.clear_chunk_buffer();
                            tracing::warn!("Policy inference failed for action {action_id}: {e}");
                            let reason = match e {
                                PolicyError::NonFinite(_) => "NON_FINITE",
                                _ => "INFERENCE_ERROR",
                            };
                            if send_executor_end_policy(
                                &handle,
                                &supervisor,
                                client,
                                &mut client_seq,
                                lease_generation,
                                epoch_id,
                                &token_guard,
                                EndPolicySpec::Fail {
                                    action_id,
                                    load_token,
                                    lease_generation,
                                    reason: HolderLabel::new(reason),
                                    message: HolderLabel::new(&e.to_string()),
                                    final_steps: shadow_steps,
                                    final_stats: step_stats.report(),
                                },
                            )
                            .await
                            {
                                token_guard.disarm();
                            }
                            break;
                        }
                    }
                }
            };

            let step_us = step_started_at
                .elapsed()
                .as_micros()
                .min(u64::MAX as u128) as u64;
            match executor_commit_gate(
                &supervisor,
                &handle,
                &mut prepared,
                action_id,
                lease_generation,
                mode,
                deadline,
                max_input_age,
                interval_origin_std,
                period_ns,
                last_committed_period,
                max_fresh_joint_age_ms,
                max_source_joint_age_ms,
                max_fresh_vision_age_ms,
                max_source_vision_age_ms,
                horizon_ms,
                video_embedding_spec.as_ref(),
                vision_spec.as_ref(),
                embedding_hub.as_ref(),
                collector.as_ref(),
                locked_generation,
                verified_release_gate.as_ref(),
                sandbox.as_ref(),
                snap_authority,
                &authorization,
                snap_auth_gen,
                &mut last_seen_refusal_gen,
                oldest_input_at,
                base_input_age_ms,
                captured_at,
                vision_captured_at,
                vision_frame_id,
                vision_target,
                emb_ticket.as_ref(),
                emb_mapped_and_unc_ms,
                infer_us,
                step_us,
                &mut shadow_steps,
                &mut step_stats,
            )
            .await
            {
                ExecutorCommitOutcome::Submitted { commit_period } => {
                    if is_chunked {
                        last_committed_period = Some(commit_period);
                    }
                }
                ExecutorCommitOutcome::Skip => {}
                ExecutorCommitOutcome::Hold => {
                    prepared.clear_chunk_buffer();
                    let _ = authorization.revoke_for_action(action_id);
                    let _ = send_executor_end_policy(
                        &handle,
                        &supervisor,
                        client,
                        &mut client_seq,
                        lease_generation,
                        epoch_id,
                        &token_guard,
                        EndPolicySpec::Hold {
                            action_id,
                            load_token,
                            lease_generation,
                        },
                    )
                    .await;
                }
                ExecutorCommitOutcome::RtEnded => {
                    prepared.clear_chunk_buffer();
                    token_guard.disarm();
                    break;
                }
                ExecutorCommitOutcome::Finish(reason) => {
                    prepared.clear_chunk_buffer();
                    if send_executor_end_policy(
                        &handle,
                        &supervisor,
                        client,
                        &mut client_seq,
                        lease_generation,
                        epoch_id,
                        &token_guard,
                        EndPolicySpec::Finish {
                            action_id,
                            load_token,
                            lease_generation,
                            reason: HolderLabel::new(reason),
                            final_steps: shadow_steps,
                            final_stats: step_stats.report(),
                        },
                    )
                    .await
                    {
                        token_guard.disarm();
                    }
                    break;
                }
                ExecutorCommitOutcome::Fail { reason, message } => {
                    prepared.clear_chunk_buffer();
                    if send_executor_end_policy(
                        &handle,
                        &supervisor,
                        client,
                        &mut client_seq,
                        lease_generation,
                        epoch_id,
                        &token_guard,
                        EndPolicySpec::Fail {
                            action_id,
                            load_token,
                            lease_generation,
                            reason: HolderLabel::new(reason),
                            message: HolderLabel::new(&message),
                            final_steps: shadow_steps,
                            final_stats: step_stats.report(),
                        },
                    )
                    .await
                    {
                        token_guard.disarm();
                    }
                    break;
                }
            }
        }

        // Off-thread device qualification ledger append for supervised first runs on real arms.
        // Uses ONLY the immutable action-keyed owner terminal outcome (`PolicyTerminalOutcome`):
        // never falls back to optimistic runner intent (`MAX_SECONDS/done`) when the owner's
        // terminal record is missing or displaced. Releases `pending_qual_guard` atomically under
        // `ledger_io` ONLY AFTER fsync completes so concurrent `production_release` starts cannot
        // approve using stale ledger state.
        if mode == PolicyMode::Live && !policy_live_allowed {
            if let Some(ref gate) = verified_release_gate {
                if gate.authorization_tier() == super::release_gate::AuthorizationTier::SupervisedFirstRun
                {
                    let qual_coord = supervisor.policy_qualification().clone();
                    let mut owner_outcome = qual_coord.take_terminal_outcome(action_id);
                    if owner_outcome.is_none() {
                        let poll_until = Instant::now() + Duration::from_millis(200);
                        while Instant::now() < poll_until {
                            tokio::time::sleep(Duration::from_millis(2)).await;
                            owner_outcome = qual_coord.take_terminal_outcome(action_id);
                            if owner_outcome.is_some() {
                                break;
                            }
                        }
                    }
                    if let Some(ledger_path) = supervisor
                        .published_config_hub()
                        .latest()
                        .and_then(|c| c.policy_ledger_path.clone())
                    {
                        let (final_steps, final_reason, passed) = match owner_outcome {
                            Some(out) => {
                                let reason = out.stop_reason.as_str().to_string();
                                let ok = reason == "MAX_SECONDS"
                                    && out.action_state.as_str() == "done";
                                (out.steps, reason, ok)
                            }
                            None => (
                                step_stats.report().steps,
                                "INTERRUPTED".to_string(),
                                false,
                            ),
                        };
                        let record = super::qualification::QualificationRecord::for_run(
                            gate,
                            action_id,
                            final_steps,
                            &final_reason,
                            passed,
                        )
                        .with_policy_stats(step_stats.report());
                        let guard_owned = pending_qual_guard.take();
                        let _ = tokio::task::spawn_blocking(move || {
                            let mut guard_owned = guard_owned;
                            if let Err(e) = qual_coord.append_and_release_pending(
                                &ledger_path,
                                &record,
                                guard_owned.as_mut(),
                            ) {
                                tracing::warn!(
                                    "Failed to append policy qualification record to {}: {e}",
                                    ledger_path.display()
                                );
                            }
                        })
                        .await;
                    } else if let Some(mut g) = pending_qual_guard.take() {
                        g.release_now();
                    }
                }
            }
        }
        drop(pending_qual_guard);
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
    fn chunking_single_step_routes_through_commit_gate() {
        let state_dim = ArmConfig::default().dof;
        let state = vec![0.0f32; state_dim];
        let origin = Instant::now();
        let period_ns = Duration::from_millis(50).as_nanos();
        let chunks = vec![
            make_constant_chunk(state_dim, &[0.1]),
            make_constant_chunk(state_dim, &[0.2]),
            make_constant_chunk(state_dim, &[0.3]),
        ];

        let (snap, contract, cfg) = make_chunk_snapshot_and_contract(1, 1, None);
        let (backend, calls) = CountingChunkBackend::new(state_dim, state_dim, chunks);
        let mut prep =
            PreparedPolicy::from_backend(snap, contract, Box::new(backend), &cfg).unwrap();

        let gate_ok = CommitGateParams {
            max_fresh_joint_age_ms: 100,
            max_cached_joint_age_ms: 100,
            max_fresh_vision_age_ms: None,
            max_cached_vision_age_ms: None,
            horizon_ms: 50,
            vision_generation: Some(7),
            vision_target: None,
            live_ticket_ctx: None,
            embedding_clock_override_ms: None,
        };

        // Tick 0: fresh inference staged and committed through commit_gate -> 0.1.
        let mut src0 = ChunkSource::from_joint(origin, Some(5));
        src0.vision_generation = Some(7);
        prep.plan_chunk_step(&state, None, 0, src0, None).unwrap();
        assert_eq!(calls.load(Ordering::Relaxed), 1);
        match prep
            .commit_gate(origin + Duration::from_millis(5), origin, period_ns, None, &gate_ok)
            .unwrap()
        {
            CommitGateDecision::Submit { target, commit_period } => {
                assert_eq!(commit_period, 0);
                assert!((target[0] - 0.1).abs() < 1e-6);
            }
            other => panic!("expected Submit on single-step tick 0, got {other:?}"),
        }

        // Calling commit_gate again without a new plan_chunk_step skips (no stale reuse).
        assert_eq!(
            prep.commit_gate(origin + Duration::from_millis(10), origin, period_ns, Some(0), &gate_ok)
                .unwrap(),
            CommitGateDecision::Skip { commit_period: 0 }
        );

        // Tick 1: generation mismatch at commit_gate refuses submission.
        let mut src1 = ChunkSource::from_joint(origin + Duration::from_millis(50), Some(5));
        src1.vision_generation = Some(7);
        prep.plan_chunk_step(&state, None, 1, src1, None).unwrap();
        assert_eq!(calls.load(Ordering::Relaxed), 2);
        let gate_bad_gen = CommitGateParams {
            vision_generation: Some(8),
            ..gate_ok.clone()
        };
        assert_eq!(
            prep.commit_gate(
                origin + Duration::from_millis(55),
                origin,
                period_ns,
                Some(0),
                &gate_bad_gen,
            )
            .unwrap(),
            CommitGateDecision::Skip { commit_period: 1 }
        );

        // Tick 2: fresh inference commits 0.3.
        let mut src2 = ChunkSource::from_joint(origin + Duration::from_millis(100), Some(5));
        src2.vision_generation = Some(7);
        prep.plan_chunk_step(&state, None, 2, src2, None).unwrap();
        assert_eq!(calls.load(Ordering::Relaxed), 3);
        match prep
            .commit_gate(origin + Duration::from_millis(105), origin, period_ns, Some(0), &gate_ok)
            .unwrap()
        {
            CommitGateDecision::Submit { target, commit_period } => {
                assert_eq!(commit_period, 2);
                assert!((target[0] - 0.3).abs() < 1e-6);
            }
            other => panic!("expected Submit on single-step tick 2, got {other:?}"),
        }
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
            contour_norm: None,
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
                embedding_clock_override_ms: None,
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
                embedding_clock_override_ms: None,
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
            embedding_clock_override_ms: None,
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
            embedding_clock_override_ms: None,
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
            embedding_clock_override_ms: None,
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
            embedding_clock_override_ms: None,
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
            embedding_clock_override_ms: None,
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

    #[test]
    fn chunking_commit_gate_evaluates_embedding_deadline_against_gate_now() {
        let state_dim = ArmConfig::default().dof;
        let state = vec![0.0f32; state_dim];
        let origin = Instant::now();
        let origin_ms = monotonic_ms_at(origin);
        let period_ns = Duration::from_millis(50).as_nanos();

        // Ticket deadline is origin_ms + 30 ms; live_ticket_ctx carries an earlier sample (origin_ms + 5 ms).
        let ticket = InferenceTicket::new(
            1,
            7,
            1,
            Some(3),
            10,
            1,
            origin_ms + 60_000,
            origin_ms + 30,
            origin_ms + 1_000,
        );
        let earlier_ctx = TicketVerificationContext {
            policy_instance_id: Some(1),
            lease_generation: Some(7),
            producer_epoch: Some(1),
            model_generation: Some(3),
            invalidation_generation: 10,
            clock_handshake_generation: 1,
            now_armd_ms: origin_ms + 5, // earlier sample: would falsely pass if not recomputed from gate `now`
        };

        // Case 1: chunk_size=4 fresh inference — deadline passes at t=20ms, expires at t=40ms (still period 0).
        let c0 = make_constant_chunk(state_dim, &[0.1, 0.2, 0.3, 0.4]);
        let (snap, contract, cfg) = make_chunk_snapshot_and_contract(4, 2, None);
        let (backend, _) = CountingChunkBackend::new(state_dim, 4 * state_dim, vec![c0]);
        let mut prep =
            PreparedPolicy::from_backend(snap, contract, Box::new(backend), &cfg).unwrap();
        let mut src = ChunkSource::from_joint(origin, Some(5));
        src.embedding_ticket = Some(ticket);
        let gate_params = CommitGateParams {
            max_fresh_joint_age_ms: 100,
            max_cached_joint_age_ms: 200,
            max_fresh_vision_age_ms: None,
            max_cached_vision_age_ms: None,
            horizon_ms: 100,
            vision_generation: None,
            vision_target: None,
            live_ticket_ctx: Some(earlier_ctx.clone()),
            embedding_clock_override_ms: None,
        };
        prep.plan_chunk_step(&state, None, 0, src, None).unwrap();
        // Gate `now` = origin + 40 ms (> deadline origin_ms + 30 ms) -> must Skip and clear ring,
        // ignoring earlier_ctx.now_armd_ms (origin_ms + 5 ms).
        let d_exp = prep
            .commit_gate(
                origin + Duration::from_millis(40),
                origin,
                period_ns,
                None,
                &gate_params,
            )
            .unwrap();
        assert_eq!(d_exp, CommitGateDecision::Skip { commit_period: 0 });
        assert_eq!(prep.chunk_buffer_len(), 0);

        // Re-plan and check at gate `now` = origin + 20 ms (<= deadline) -> Submit succeeds.
        prep.plan_chunk_step(&state, None, 0, src, None).unwrap();
        assert!(matches!(
            prep.commit_gate(
                origin + Duration::from_millis(20),
                origin,
                period_ns,
                None,
                &gate_params,
            )
            .unwrap(),
            CommitGateDecision::Submit { commit_period: 0, .. }
        ));

        // Case 2: chunk_size=1 also evaluates the embedding deadline against gate `now`.
        let c1 = make_constant_chunk(state_dim, &[0.25]);
        let (snap1, contract1, cfg1) = make_chunk_snapshot_and_contract(1, 1, None);
        let (backend1, _) = CountingChunkBackend::new(state_dim, state_dim, vec![c1]);
        let mut prep1 =
            PreparedPolicy::from_backend(snap1, contract1, Box::new(backend1), &cfg1).unwrap();
        prep1.plan_chunk_step(&state, None, 0, src, None).unwrap();
        assert_eq!(
            prep1
                .commit_gate(
                    origin + Duration::from_millis(40),
                    origin,
                    period_ns,
                    None,
                    &gate_params,
                )
                .unwrap(),
            CommitGateDecision::Skip { commit_period: 0 }
        );
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
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).unwrap();
            SupervisorInner::begin_policy_run(admitted, PolicyRunStart {
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
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).unwrap();
            SupervisorInner::begin_policy_run(admitted, PolicyRunStart {
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

    #[cfg(not(target_arch = "wasm32"))]
    async fn start_executor_shadow_run(
        sup: &Arc<ArmSupervisor>,
        handle: &crate::rt_core::executor::RtExecutorHandle,
        client_id: u64,
        conn_name: &str,
        lease_gen: u64,
        epoch_id: u64,
        prep: &PreparedPolicy,
        fps: f64,
        deadline: Option<Instant>,
    ) -> (u64, crate::rt_core::requests::policy::PolicyTokenGuard) {
        use crate::model::HolderLabel;
        use crate::rt_core::broker::{ActionId, Authority, ClientId, Reply};
        use crate::rt_core::requests::lease::ConnectionHandle;
        use crate::rt_core::requests::policy::{
            BeginPolicySpec, PolicyCommand, ReservePolicyLoadSpec,
        };

        let token_guard = sup.policy_cancel_slots().claim_slot().unwrap();
        let conn = ConnectionHandle::try_new(client_id, conn_name).unwrap();
        let _ = handle.open_connection_async(client_id).await.unwrap();
        let authority = Authority {
            lease_generation: lease_gen,
            epoch: epoch_id,
            stop_generation: sup.current_stop_gen(),
            config_revision: 0,
            parent_generation: 0,
        };
        let boot = handle.boot();
        let res_r = handle
            .submit_policy_command_async(
                ClientId(client_id),
                1,
                authority,
                Instant::now() + Duration::from_secs(2),
                PolicyCommand::ReserveLoad(ReservePolicyLoadSpec {
                    load_token: token_guard.token(),
                    cancel_slot: token_guard.slot(),
                    admission_request_id: 1,
                    prepared_boot: boot,
                    connection: conn,
                    mode: PolicyMode::Shadow,
                    sim_only: true,
                    i_am_at_the_arm: false,
                    operator: crate::auth::OperatorGrant::unenforced(),
                    reservation_expires_at: Instant::now() + Duration::from_secs(10),
                }),
                |req_id| token_guard.record_admitted_req(req_id.0),
            )
            .await
            .unwrap();
        assert!(matches!(res_r, Reply::Ack { .. }), "ReserveLoad failed: {res_r:?}");

        let begin_r = handle
            .submit_policy_command_async(
                ClientId(client_id),
                2,
                authority,
                Instant::now() + Duration::from_secs(2),
                PolicyCommand::BeginPolicy(BeginPolicySpec {
                    load_token: token_guard.token(),
                    cancel_slot: token_guard.slot(),
                    admission_request_id: 1,
                    prepared_boot: boot,
                    connection: conn,
                    operator: crate::auth::OperatorGrant::unenforced(),
                    calibration_revision: 0,
                    policy_id: HolderLabel::new("exec_chunk"),
                    artifact_digest: HolderLabel::new(prep.artifact_digest()),
                    backend: HolderLabel::new(prep.backend_name()),
                    mode: PolicyMode::Shadow,
                    sim_only: true,
                    i_am_at_the_arm: false,
                    max_first_step_rad: DEFAULT_MAX_FIRST_STEP_RAD,
                    max_step_rad: None,
                    max_input_age_ms: max_policy_input_age_ms(fps),
                    max_vision_age_ms: prep.contract().vision.as_ref().map(|v| v.max_staleness_ms),
                    vision_stale_hold: prep
                        .contract()
                        .vision
                        .as_ref()
                        .is_some_and(|v| v.on_stale == VisionStalePolicy::Hold),
                    period_us: period_us_for_fps(fps),
                    deadline,
                    has_verified_release_gate: false,
                    gate_policy_sha256: HolderLabel::new(""),
                    gate_calibration_digest: HolderLabel::new(""),
                    gate_max_speed_scale: 1.0,
                    gate_is_production_release: false,
                    production_qualified_verified: false,
                }),
                |req_id| token_guard.record_admitted_req(req_id.0),
            )
            .await
            .unwrap();
        let action_id = match begin_r {
            Reply::Ack {
                action: Some(ActionId(id)),
                ..
            } => id,
            other => panic!("BeginPolicy failed: {other:?}"),
        };
        (action_id, token_guard)
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn chunking_executor_replan_cadence_overrun_commit_gate_and_vision_hold_clear() {
        use crate::grasp::Detection;
        use crate::rt_core::broker::ClientId;
        use crate::rt_core::executor::ExecutorConfig;
        use std::sync::atomic::AtomicBool;
        use super::super::vision::PolicyVisionFrame;

        // Part A: chunk_size=4, replan_every=4 at 20 Hz on OWNER_EXECUTOR serves cached slots
        // without replanning every tick (proves ChunkRunContext does not spuriously clear ring).
        let (snap_a, contract_a, cfg_a) = make_chunk_snapshot_and_contract(4, 4, None);
        let dim = cfg_a.dof;
        let c_a = make_constant_chunk(dim, &[0.02, 0.04, 0.06, 0.08]);
        let (backend_a, calls_a) = CountingChunkBackend::new(dim, 4 * dim, vec![c_a]);
        let prep_a = PreparedPolicy::from_backend(snap_a, contract_a, Box::new(backend_a), &cfg_a)
            .unwrap();

        let sup_a = Arc::new(ArmSupervisor::new(cfg_a));
        let lease_a = sup_a.acquire_lease("exec-chunk-a").await.unwrap();
        let exec_a = sup_a
            .start_executor(ExecutorConfig::for_tests(Duration::from_millis(10)))
            .await
            .unwrap();
        let deadline_wait = Instant::now() + Duration::from_secs(2);
        while sup_a.hub().latest().is_none() && Instant::now() < deadline_wait {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        let epoch_a = sup_a.fresh_snapshot(None).await.unwrap().obs.epoch_id;
        let dl_a = compute_policy_deadline(Some(0.185)).unwrap();
        let (aid_a, guard_a) = start_executor_shadow_run(
            &sup_a, &exec_a, 101, "exec-chunk-a", lease_a, epoch_a, &prep_a, 20.0, dl_a,
        )
        .await;
        let runner_a = spawn_executor_policy_runner(ExecutorPolicyRunParams {
            supervisor: sup_a.clone(),
            handle: exec_a.clone(),
            client: ClientId(101),
            action_id: aid_a,
            lease_generation: lease_a,
            epoch_id: epoch_a,
            mode: PolicyMode::Shadow,
            prepared: prep_a,
            fps: 20.0,
            deadline: dl_a,
            vision_source: None,
            sandbox: None,
            verified_release_gate: None,
            policy_live_allowed: true,
            has_driver: false,
            token_guard: guard_a,
            pending_qual_guard: None,
        });
        runner_a.await.unwrap();
        let obs_a = sup_a.get_observation().await;
        let pol_a = obs_a.policy.expect("policy obs a");
        let n_calls_a = calls_a.load(Ordering::Relaxed);
        assert!(
            pol_a.steps >= 3,
            "expected >= 3 committed steps over 185 ms at 20 Hz, got {}",
            pol_a.steps
        );
        assert!(
            (n_calls_a as u64) < pol_a.steps,
            "cached chunk slots must execute without replanning every tick (calls={n_calls_a}, steps={})",
            pol_a.steps
        );
        exec_a.stop_and_join();

        // Part B: 60 ms replan at 20 Hz (50 ms period) on OWNER_EXECUTOR lands in period 1 ->
        // commit_gate_now selects slot 1 (0.15), not stale slot 0 (0.05).
        let (snap_b, contract_b, cfg_b) = make_chunk_snapshot_and_contract(4, 2, None);
        let c_b = make_constant_chunk(dim, &[0.05, 0.15, 0.25, 0.35]);
        let (backend_b, _) = CountingChunkBackend::new(dim, 4 * dim, vec![c_b]);
        let prep_b = PreparedPolicy::from_backend(snap_b, contract_b, Box::new(backend_b), &cfg_b)
            .unwrap()
            .with_infer_delay(Duration::from_millis(60));
        let sup_b = Arc::new(ArmSupervisor::new(cfg_b));
        let lease_b = sup_b.acquire_lease("exec-chunk-b").await.unwrap();
        let exec_b = sup_b
            .start_executor(ExecutorConfig::for_tests(Duration::from_millis(10)))
            .await
            .unwrap();
        while sup_b.hub().latest().is_none() {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        let epoch_b = sup_b.fresh_snapshot(None).await.unwrap().obs.epoch_id;
        let dl_b = compute_policy_deadline(Some(0.095)).unwrap();
        let (aid_b, guard_b) = start_executor_shadow_run(
            &sup_b, &exec_b, 102, "exec-chunk-b", lease_b, epoch_b, &prep_b, 20.0, dl_b,
        )
        .await;
        spawn_executor_policy_runner(ExecutorPolicyRunParams {
            supervisor: sup_b.clone(),
            handle: exec_b.clone(),
            client: ClientId(102),
            action_id: aid_b,
            lease_generation: lease_b,
            epoch_id: epoch_b,
            mode: PolicyMode::Shadow,
            prepared: prep_b,
            fps: 20.0,
            deadline: dl_b,
            vision_source: None,
            sandbox: None,
            verified_release_gate: None,
            policy_live_allowed: true,
            has_driver: false,
            token_guard: guard_b,
            pending_qual_guard: None,
        })
        .await
        .unwrap();
        let obs_b = sup_b.get_observation().await;
        let pol_b = obs_b.policy.expect("policy obs b");
        assert_eq!(pol_b.steps, 1, "expected 1 committed step before 95 ms deadline");
        let tgt_b = pol_b.last_shadow_target.expect("shadow target b");
        assert!(
            (tgt_b[0] - 0.15).abs() < 1e-6,
            "executor 60 ms replan at 20 Hz must commit period 1 (0.15), got {}",
            tgt_b[0]
        );
        exec_b.stop_and_join();

        // Part C: 115 ms replan with chunk_size=2 on OWNER_EXECUTOR overruns past buffer -> 0 steps.
        let (snap_c, contract_c, cfg_c) = make_chunk_snapshot_and_contract(2, 2, None);
        let c_c = make_constant_chunk(dim, &[0.05, 0.15]);
        let (backend_c, _) = CountingChunkBackend::new(dim, 2 * dim, vec![c_c]);
        let prep_c = PreparedPolicy::from_backend(snap_c, contract_c, Box::new(backend_c), &cfg_c)
            .unwrap()
            .with_infer_delay(Duration::from_millis(115));
        let sup_c = Arc::new(ArmSupervisor::new(cfg_c));
        let lease_c = sup_c.acquire_lease("exec-chunk-c").await.unwrap();
        let exec_c = sup_c
            .start_executor(ExecutorConfig::for_tests(Duration::from_millis(10)))
            .await
            .unwrap();
        while sup_c.hub().latest().is_none() {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        let epoch_c = sup_c.fresh_snapshot(None).await.unwrap().obs.epoch_id;
        let dl_c = compute_policy_deadline(Some(0.14)).unwrap();
        let (aid_c, guard_c) = start_executor_shadow_run(
            &sup_c, &exec_c, 103, "exec-chunk-c", lease_c, epoch_c, &prep_c, 20.0, dl_c,
        )
        .await;
        spawn_executor_policy_runner(ExecutorPolicyRunParams {
            supervisor: sup_c.clone(),
            handle: exec_c.clone(),
            client: ClientId(103),
            action_id: aid_c,
            lease_generation: lease_c,
            epoch_id: epoch_c,
            mode: PolicyMode::Shadow,
            prepared: prep_c,
            fps: 20.0,
            deadline: dl_c,
            vision_source: None,
            sandbox: None,
            verified_release_gate: None,
            policy_live_allowed: true,
            has_driver: false,
            token_guard: guard_c,
            pending_qual_guard: None,
        })
        .await
        .unwrap();
        let obs_c = sup_c.get_observation().await;
        let pol_c = obs_c.policy.expect("policy obs c");
        assert_eq!(pol_c.steps, 0, "executor overrun past chunk buffer must submit nothing");
        assert!(pol_c.last_shadow_target.is_none());
        exec_c.stop_and_join();

        // Part D: Vision Hold on OWNER_EXECUTOR clears the chunk ring and forces an immediate
        // replan when the target reappears (even with chunk_size=16, replan_every=16).
        struct ToggleVisionSource {
            present: Arc<AtomicBool>,
            frame_seq: AtomicU64,
        }
        impl PolicyVisionSource for ToggleVisionSource {
            fn next_frame<'a>(
                &'a self,
                _after_frame_id: u64,
                _wait_ms: u64,
            ) -> std::pin::Pin<
                Box<
                    dyn std::future::Future<Output = Result<Option<PolicyVisionFrame>, String>>
                        + Send
                        + 'a,
                >,
            > {
                let present = self.present.load(Ordering::Acquire);
                let fid = self.frame_seq.fetch_add(1, Ordering::AcqRel) + 1;
                Box::pin(async move {
                    tokio::time::sleep(Duration::from_millis(5)).await;
                    let detections = if present {
                        vec![Detection {
                            label: "cube".into(),
                            score: 0.95,
                            raw_score_valid: true,
                            box_norm: [0.4, 0.4, 0.6, 0.6],
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
                            contour_norm: None,
                        }]
                    } else {
                        vec![]
                    };
                    Ok(Some(PolicyVisionFrame {
                        frame_id: fid,
                        captured_at: Some(Instant::now()),
                        model_generation: Some(1),
                        detections,
                    }))
                })
            }
        }

        let mut cfg_d = ArmConfig::default();
        let joint_names: Vec<String> = (1..=cfg_d.dof).map(|i| format!("j{i}")).collect();
        cfg_d.joint_names = joint_names.clone();
        cfg_d.limits = joint_names
            .iter()
            .map(|n| JointLimit::new(n.clone(), -2.0, 2.0, 2.0, 10.0))
            .collect();
        let pj_d = serde_json::json!({
            "format": "rebot-onnx-policy/v1",
            "name": "exec_chunk_hold",
            "unit_mode": "rad",
            "action_dim": cfg_d.dof,
            "action_space": "joint_positions",
            "joint_names": joint_names,
            "model_file": "model.rmlp",
            "chunking": {
                "chunk_size": 16,
                "replan_every": 16
            },
            "vision": {
                "source": "detector",
                "layout": "top1_box_norm_v1",
                "target_label": "cube",
                "min_score": 0.3,
                "max_staleness_ms": 400,
                "on_target_lost": "hold",
                "on_stale": "hold",
                "require_model_generation": true
            }
        });
        let pj_bytes_d = serde_json::to_vec(&pj_d).unwrap();
        let contract_d = StrictPolicyContract::parse(&pj_bytes_d).unwrap();
        let snap_d = PolicySnapshot {
            dir_path: PathBuf::from("/tmp/exec_chunk_hold"),
            policy_json_bytes: pj_bytes_d,
            model_rel_posix: "model.rmlp".into(),
            model_bytes: vec![1, 2, 3, 4],
            release_digest: "deadbeef".into(),
        };
        let c_d0 = make_constant_chunk(cfg_d.dof, &[0.11; 16]);
        let c_d1 = make_constant_chunk(cfg_d.dof, &[0.22; 16]);
        let (backend_d, calls_d) =
            CountingChunkBackend::new(cfg_d.dof + 6, 16 * cfg_d.dof, vec![c_d0, c_d1]);
        let prep_d =
            PreparedPolicy::from_backend(snap_d, contract_d, Box::new(backend_d), &cfg_d).unwrap();

        let present = Arc::new(AtomicBool::new(true));
        let vis_src = Arc::new(ToggleVisionSource {
            present: present.clone(),
            frame_seq: AtomicU64::new(0),
        }) as Arc<dyn PolicyVisionSource>;

        let sup_d = Arc::new(ArmSupervisor::new(cfg_d));
        let lease_d = sup_d.acquire_lease("exec-chunk-d").await.unwrap();
        let exec_d = sup_d
            .start_executor(ExecutorConfig::for_tests(Duration::from_millis(10)))
            .await
            .unwrap();
        while sup_d.hub().latest().is_none() {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        let epoch_d = sup_d.fresh_snapshot(None).await.unwrap().obs.epoch_id;
        let dl_d = compute_policy_deadline(Some(0.45)).unwrap();
        let (aid_d, guard_d) = start_executor_shadow_run(
            &sup_d, &exec_d, 104, "exec-chunk-d", lease_d, epoch_d, &prep_d, 20.0, dl_d,
        )
        .await;
        let runner_d = spawn_executor_policy_runner(ExecutorPolicyRunParams {
            supervisor: sup_d.clone(),
            handle: exec_d.clone(),
            client: ClientId(104),
            action_id: aid_d,
            lease_generation: lease_d,
            epoch_id: epoch_d,
            mode: PolicyMode::Shadow,
            prepared: prep_d,
            fps: 20.0,
            deadline: dl_d,
            vision_source: Some(vis_src),
            sandbox: None,
            verified_release_gate: None,
            policy_live_allowed: true,
            has_driver: false,
            token_guard: guard_d,
            pending_qual_guard: None,
        });

        // Wait for first replan (chunk 0 -> 0.11).
        let wait_c1 = Instant::now() + Duration::from_secs(2);
        while calls_d.load(Ordering::Relaxed) < 1 && Instant::now() < wait_c1 {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        assert_eq!(calls_d.load(Ordering::Relaxed), 1, "first replan must run");
        // Drop target for ~90 ms -> triggers VisionStepDecision::Hold and clears chunk ring!
        present.store(false, Ordering::Release);
        tokio::time::sleep(Duration::from_millis(90)).await;
        // Restore target -> cleared ring forces immediate replan (calls_d -> 2, target 0.22)!
        present.store(true, Ordering::Release);
        runner_d.await.unwrap();
        assert!(
            calls_d.load(Ordering::Relaxed) >= 2,
            "vision hold on executor must clear chunk ring and force replan on recovery (calls={})",
            calls_d.load(Ordering::Relaxed)
        );
        let obs_d = sup_d.get_observation().await;
        let tgt_d = obs_d.policy.and_then(|p| p.last_shadow_target).expect("shadow target d");
        assert!(
            (tgt_d[0] - 0.22).abs() < 1e-6,
            "after vision hold cleared ring, replanned chunk 1 (0.22) must be emitted, got {}",
            tgt_d[0]
        );
        exec_d.stop_and_join();
    }

    #[test]
    fn chunking_r1_single_step_detector_identity_age_and_chunk_slot_validity_bounds() {
        use crate::rt_core::broker::{Authority, BootId};
        use crate::rt_core::ids::ArmId;
        use crate::rt_core::mailbox::{
            PolicyValidityBounds, StreamId, TargetAuthority, TargetFrame, MAX_DOF,
        };

        let state_dim = ArmConfig::default().dof;
        let state = vec![0.0f32; state_dim];
        let origin = Instant::now();
        let period_ns = Duration::from_millis(50).as_nanos();

        // 1. Blocker 1: single-step (chunk_size=1) commit_gate enforces detector target identity
        //    AND detector source age against gate `now`.
        let (snap1, contract1, cfg1) = make_chunk_snapshot_and_contract(1, 1, None);
        let c1 = make_constant_chunk(state_dim, &[0.12]);
        let (backend1, _) = CountingChunkBackend::new(state_dim, state_dim, vec![c1]);
        let mut prep1 =
            PreparedPolicy::from_backend(snap1, contract1, Box::new(backend1), &cfg1).unwrap();
        let target_a = DetectorTargetIdentity::from_detection(None, "cube", [0.1, 0.1, 0.3, 0.3]);
        let target_b = DetectorTargetIdentity::from_detection(None, "mug", [0.6, 0.6, 0.8, 0.8]);

        let mut src_a = ChunkSource::from_joint(origin, Some(5));
        src_a.vision_captured_at = Some(origin);
        src_a.vision_generation = Some(1);
        src_a.vision_target = Some(target_a);
        prep1.plan_chunk_step(&state, None, 0, src_a, None).unwrap();

        // Target replaced by B during inference -> single-step commit_gate MUST Skip.
        let gate_replaced = CommitGateParams {
            max_fresh_joint_age_ms: 100,
            max_cached_joint_age_ms: 100,
            max_fresh_vision_age_ms: Some(100),
            max_cached_vision_age_ms: Some(100),
            horizon_ms: 50,
            vision_generation: Some(1),
            vision_target: Some(target_b),
            live_ticket_ctx: None,
            embedding_clock_override_ms: None,
        };
        assert_eq!(
            prep1
                .commit_gate(origin + Duration::from_millis(10), origin, period_ns, None, &gate_replaced)
                .unwrap(),
            CommitGateDecision::Skip { commit_period: 0 }
        );

        // Detector source expired (> max_fresh_vision_age_ms=50 ms) -> single-step commit_gate MUST Skip.
        prep1.plan_chunk_step(&state, None, 0, src_a, None).unwrap();
        let gate_stale_vis = CommitGateParams {
            max_fresh_vision_age_ms: Some(50),
            vision_target: Some(target_a),
            ..gate_replaced.clone()
        };
        assert_eq!(
            prep1
                .commit_gate(origin + Duration::from_millis(60), origin, period_ns, None, &gate_stale_vis)
                .unwrap(),
            CommitGateDecision::Skip { commit_period: 1 }
        );

        // Matching target within age -> single-step Submit succeeds.
        prep1.plan_chunk_step(&state, None, 0, src_a, None).unwrap();
        assert!(matches!(
            prep1
                .commit_gate(origin + Duration::from_millis(20), origin, period_ns, None, &gate_stale_vis)
                .unwrap(),
            CommitGateDecision::Submit { commit_period: 0, .. }
        ));

        // 2. Blocker 2 + Major 4: chunked commit_gate computes strictest contributor expiry capped
        //    by period_end, and TargetFrame::check_policy_with_validity enforces it as DEADLINE_EXPIRED.
        let (snap4, contract4, cfg4) = make_chunk_snapshot_and_contract(4, 2, None);
        let c4 = make_constant_chunk(state_dim, &[0.05, 0.15, 0.25, 0.35]);
        let (backend4, _) = CountingChunkBackend::new(state_dim, 4 * state_dim, vec![c4]);
        let mut prep4 =
            PreparedPolicy::from_backend(snap4, contract4, Box::new(backend4), &cfg4).unwrap();
        let origin_ms = monotonic_ms_at(origin);
        let ticket_tight = InferenceTicket::new(
            1,
            7,
            1,
            Some(1),
            1,
            1,
            origin_ms + 40, // tight handshake expiry
            origin_ms + 25, // tight newest deadline
            origin_ms + 500,
        );
        let mut src4 = ChunkSource::from_joint(origin, Some(5));
        src4.embedding_ticket = Some(ticket_tight);
        prep4.plan_chunk_step(&state, None, 0, src4, None).unwrap();
        let live_ctx = TicketVerificationContext {
            policy_instance_id: Some(1),
            lease_generation: Some(7),
            producer_epoch: Some(1),
            model_generation: Some(1),
            invalidation_generation: 1,
            clock_handshake_generation: 1,
            now_armd_ms: origin_ms + 10,
        };
        let gate4 = CommitGateParams {
            max_fresh_joint_age_ms: 100,
            max_cached_joint_age_ms: 200,
            max_fresh_vision_age_ms: None,
            max_cached_vision_age_ms: None,
            horizon_ms: 100,
            vision_generation: None,
            vision_target: None,
            live_ticket_ctx: Some(live_ctx),
            embedding_clock_override_ms: Some(origin_ms + 10),
        };
        assert!(matches!(
            prep4
                .commit_gate(origin + Duration::from_millis(10), origin, period_ns, None, &gate4)
                .unwrap(),
            CommitGateDecision::Submit { commit_period: 0, .. }
        ));
        let slot_exp = prep4
            .last_commit_slot_expires_at()
            .expect("chunked commit must record slot expiry");
        let period_end = prep4
            .last_commit_period_end()
            .expect("chunked commit must record exclusive period_end");
        let hs_exp = prep4
            .last_commit_handshake_expires_at()
            .expect("embedding contributor must record handshake expiry");
        // Fresh slot 0 has ticket newest deadline remaining 15 ms (25 - 10); period_end is origin + 50 ms.
        assert_eq!(slot_exp, origin + Duration::from_millis(25));
        assert_eq!(period_end, origin + Duration::from_millis(50));
        assert_eq!(hs_exp, origin + Duration::from_millis(40));

        // RT mailbox check passes at slot_exp and refuses DEADLINE_EXPIRED 1 ms after slot_exp,
        // and enforces chunk_period_end as an exclusive boundary (now == period_end -> DEADLINE_EXPIRED).
        let mut q = [0.0f32; MAX_DOF];
        q[0] = 0.05;
        let auth = Authority {
            lease_generation: 7,
            epoch: 1,
            stop_generation: 0,
            config_revision: 0,
            parent_generation: 0,
        };
        let frame = TargetFrame {
            arm: ArmId(1),
            boot: BootId(1),
            stream: StreamId(1),
            seq: 1,
            authority: auth,
            policy_generation: 1,
            input_at: origin + Duration::from_millis(10),
            vision_at: None,
            vision_frame_id: None,
            deadline: origin + Duration::from_millis(200),
            dof: state_dim as u8,
            q,
            replaceable: false,
        };
        let ta = TargetAuthority {
            stream: Some(StreamId(1)),
            policy_generation: 1,
            max_input_age: Some(Duration::from_millis(100)),
            max_vision_age: None,
        };
        let validity = PolicyValidityBounds {
            vision_window_expires_at: Some(origin + Duration::from_millis(100)),
            handshake_expires_at: Some(hs_exp),
            chunk_slot_expires_at: Some(slot_exp),
            chunk_period_end: Some(period_end),
            expected_refusal_gen: None,
        };
        assert_eq!(
            frame.check_policy_with_validity(ArmId(1), BootId(1), slot_exp, &auth, &ta, validity),
            None
        );
        assert_eq!(
            frame.check_policy_with_validity(
                ArmId(1),
                BootId(1),
                slot_exp + Duration::from_millis(1),
                &auth,
                &ta,
                validity
            ),
            Some("DEADLINE_EXPIRED")
        );
        let period_only_validity = PolicyValidityBounds {
            vision_window_expires_at: None,
            handshake_expires_at: None,
            chunk_slot_expires_at: None,
            chunk_period_end: Some(period_end),
            expected_refusal_gen: None,
        };
        assert_eq!(
            frame.check_policy_with_validity(
                ArmId(1),
                BootId(1),
                period_end - Duration::from_nanos(1),
                &auth,
                &ta,
                period_only_validity
            ),
            None
        );
        assert_eq!(
            frame.check_policy_with_validity(
                ArmId(1),
                BootId(1),
                period_end,
                &auth,
                &ta,
                period_only_validity
            ),
            Some("DEADLINE_EXPIRED")
        );

        // 3. Direct Rust callers cannot bypass structural chunk validation in from_backend.
        let (snap_bad, mut contract_bad, cfg_bad) = make_chunk_snapshot_and_contract(4, 2, None);
        contract_bad.chunking.replan_every = 0;
        let (backend_bad, _) = CountingChunkBackend::new(state_dim, 4 * state_dim, vec![]);
        assert!(
            PreparedPolicy::from_backend(snap_bad, contract_bad, Box::new(backend_bad), &cfg_bad)
                .is_err(),
            "replan_every=0 must be rejected by PreparedPolicy::from_backend"
        );
        let (snap_nan, mut contract_nan, cfg_nan) = make_chunk_snapshot_and_contract(4, 2, None);
        contract_nan.chunking.temporal_ensemble_k = Some(f32::NAN);
        let (backend_nan, _) = CountingChunkBackend::new(state_dim, 4 * state_dim, vec![]);
        assert!(
            PreparedPolicy::from_backend(snap_nan, contract_nan, Box::new(backend_nan), &cfg_nan)
                .is_err(),
            "NaN temporal_ensemble_k must be rejected by PreparedPolicy::from_backend"
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn chunking_r3_fractional_ms_contributor_deadline_and_detector_frame_mutation_at_gate() {
        use super::super::manifest::{VisionFeatureSpec, VisionLayout, VisionSourceKind};
        use super::super::vision::PolicyVisionFrame;
        use crate::rt_core::broker::{Authority, BootId};
        use crate::rt_core::ids::ArmId;
        use crate::rt_core::mailbox::{
            PolicyValidityBounds, StreamId, TargetAuthority, TargetFrame, MAX_DOF,
        };
        use std::sync::Mutex;

        // 1. Fractional-ms contributor deadline: cached detector/joint contributor captured at
        //    `origin` with a 100 ms limit; commit_gate runs at `origin + 99.8 ms` with a fresh
        //    current frame. `slot_exp` MUST be exact `origin + 100.0 ms`, so RT at `origin + 100.4 ms`
        //    refuses `DEADLINE_EXPIRED` (no +0.8 ms integer-ms floor extension).
        let (snap, contract, cfg) = make_chunk_snapshot_and_contract(4, 2, None);
        let state_dim = contract.action_dim;
        let (backend, _) =
            CountingChunkBackend::new(state_dim, 4 * state_dim, vec![vec![0.05; 4 * state_dim]]);
        let mut prep =
            PreparedPolicy::from_backend(snap, contract, Box::new(backend), &cfg).unwrap();
        let origin = Instant::now();
        let period_ns = Duration::from_millis(50).as_nanos();
        let state = vec![0.0f32; state_dim];

        let mut src0 = ChunkSource::from_joint(origin, None);
        src0.vision_captured_at = Some(origin);
        prep.plan_chunk_step(&state, None, 0, src0, None).unwrap();

        let gate_p0 = CommitGateParams {
            max_fresh_joint_age_ms: 100,
            max_cached_joint_age_ms: 100,
            max_fresh_vision_age_ms: Some(100),
            max_cached_vision_age_ms: Some(100),
            horizon_ms: 100,
            vision_generation: None,
            vision_target: None,
            live_ticket_ctx: None,
            embedding_clock_override_ms: None,
        };
        assert!(matches!(
            prep.commit_gate(origin + Duration::from_millis(10), origin, period_ns, None, &gate_p0)
                .unwrap(),
            CommitGateDecision::Submit { commit_period: 0, .. }
        ));

        // Period 1 at origin + 99.8 ms: cached contributor is 99.8 ms old (<= 100.0 ms), so gate
        // submits slot 1 with exact slot_exp == origin + 100.0 ms (only 0.2 ms remaining).
        let t_gate = origin + Duration::from_micros(99_800);
        assert!(matches!(
            prep.commit_gate(t_gate, origin, period_ns, Some(0), &gate_p0)
                .unwrap(),
            CommitGateDecision::Submit { commit_period: 1, .. }
        ));
        let slot_exp = prep.last_commit_slot_expires_at().unwrap();
        assert_eq!(
            slot_exp,
            origin + Duration::from_millis(100),
            "slot_exp must equal exact contributor deadline (no integer-ms floor slip)"
        );

        let mut q = [0.0f32; MAX_DOF];
        q[0] = 0.05;
        let auth = Authority {
            lease_generation: 1,
            epoch: 1,
            stop_generation: 0,
            config_revision: 0,
            parent_generation: 0,
        };
        let frame = TargetFrame {
            arm: ArmId(1),
            boot: BootId(1),
            stream: StreamId(1),
            seq: 2,
            authority: auth,
            policy_generation: 1,
            input_at: t_gate,
            vision_at: Some(t_gate), // fresh current frame
            vision_frame_id: Some(2),
            deadline: origin + Duration::from_millis(200),
            dof: state_dim as u8,
            q,
            replaceable: false,
        };
        let ta = TargetAuthority {
            stream: Some(StreamId(1)),
            policy_generation: 1,
            max_input_age: Some(Duration::from_millis(100)),
            max_vision_age: Some(Duration::from_millis(100)),
        };
        let validity = PolicyValidityBounds {
            vision_window_expires_at: None,
            handshake_expires_at: None,
            chunk_slot_expires_at: Some(slot_exp),
            chunk_period_end: prep.last_commit_period_end(),
            expected_refusal_gen: Some(0),
        };
        // At origin + 100.4 ms, current frame is only 0.6 ms old, but contributor exceeded 100.0 ms!
        assert_eq!(
            frame.check_policy_with_validity(
                ArmId(1),
                BootId(1),
                origin + Duration::from_micros(100_400),
                &auth,
                &ta,
                validity,
            ),
            Some("DEADLINE_EXPIRED")
        );

        // 2. Detector frame mutation between sampling and `evaluate_final_commit_gate`:
        //    missing timestamp, stale frame, model_generation change, and target loss
        //    (Hold / Stop / ZeroFeatures) each apply their configured disposition and clear ring.
        struct SwappableVisionSource {
            next: Arc<Mutex<PolicyVisionFrame>>,
        }
        impl PolicyVisionSource for SwappableVisionSource {
            fn next_frame<'a>(
                &'a self,
                _after_frame_id: u64,
                _wait_ms: u64,
            ) -> std::pin::Pin<
                Box<
                    dyn std::future::Future<Output = Result<Option<PolicyVisionFrame>, String>>
                        + Send
                        + 'a,
                >,
            > {
                let frame = self.next.lock().unwrap().clone();
                Box::pin(async move {
                    tokio::time::sleep(Duration::from_millis(2)).await;
                    Ok(Some(frame))
                })
            }
        }
        let make_det = |label: &str| crate::grasp::Detection {
            label: label.to_string(),
            score: 0.95,
            raw_score_valid: true,
            box_norm: [0.4, 0.4, 0.6, 0.6],
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
            contour_norm: None,
        };
        let fresh_now = Instant::now();
        let shared_frame = Arc::new(Mutex::new(PolicyVisionFrame {
            frame_id: 1,
            detections: vec![make_det("cube")],
            captured_at: Some(fresh_now),
            model_generation: Some(7),
        }));
        let collector = VisionFeatureCollector::start(
            Arc::new(SwappableVisionSource {
                next: Arc::clone(&shared_frame),
            }),
            5,
        );
        // Wait for initial frame to land in collector.
        let wait_dl = Instant::now() + Duration::from_millis(500);
        while collector.latest().is_none() && Instant::now() < wait_dl {
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        assert!(collector.latest().is_some());

        let base_vision_spec = VisionFeatureSpec {
            source: VisionSourceKind::Detector,
            layout: VisionLayout::Top1BoxNormV1,
            target_label: Some("cube".to_string()),
            min_score: 0.5,
            max_staleness_ms: 100,
            max_sync_skew_ms: 100,
            on_stale: VisionStalePolicy::Stop,
            on_target_lost: VisionTargetLostPolicy::Stop,
            require_model_generation: true,
        };
        let target_id = DetectorTargetIdentity::from_detection(None, "cube", [0.4, 0.4, 0.6, 0.6]);
        let stage_chunk = |prep: &mut PreparedPolicy, now: Instant| {
            let mut src = ChunkSource::from_joint(now, None);
            src.vision_captured_at = Some(now);
            src.vision_generation = Some(7);
            src.vision_target = Some(target_id);
            prep.plan_chunk_step(&state, None, 0, src, None).unwrap();
            assert!(!prep.chunk_ring.is_empty());
        };
        let push_and_wait = |frame: PolicyVisionFrame| {
            let want_id = frame.frame_id;
            *shared_frame.lock().unwrap() = frame;
            let collector_ref = &collector;
            async move {
                let dl = Instant::now() + Duration::from_millis(500);
                while collector_ref.latest().map(|f| f.frame_id) != Some(want_id)
                    && Instant::now() < dl
                {
                    tokio::time::sleep(Duration::from_millis(2)).await;
                }
                assert_eq!(collector_ref.latest().map(|f| f.frame_id), Some(want_id));
            }
        };

        // (a) Missing timestamp with on_stale=Stop -> Fail("VISION_TIMESTAMP_MISSING") + clear ring.
        let now_a = Instant::now();
        stage_chunk(&mut prep, now_a);
        push_and_wait(PolicyVisionFrame {
            frame_id: 2,
            detections: vec![make_det("cube")],
            captured_at: None,
            model_generation: Some(7),
        })
        .await;
        let out_a = evaluate_final_commit_gate(
            &mut prep,
            None,
            now_a,
            period_ns,
            None,
            100,
            200,
            Some(100),
            Some(200),
            100,
            None,
            Some(&base_vision_spec),
            None,
            Some(1),
            Some(&collector),
            Some(7),
            None,
            now_a,
            Some(now_a),
            Some(target_id),
            None,
            None,
        );
        assert!(matches!(
            out_a,
            FinalCommitGateOutcome::Fail {
                reason: "VISION_TIMESTAMP_MISSING",
                ..
            }
        ));
        assert!(prep.chunk_ring.is_empty());

        // (b) Stale latest frame with on_stale=Hold -> Hold + clear ring.
        let now_b = Instant::now();
        stage_chunk(&mut prep, now_b);
        push_and_wait(PolicyVisionFrame {
            frame_id: 3,
            detections: vec![make_det("cube")],
            captured_at: now_b.checked_sub(Duration::from_millis(250)),
            model_generation: Some(7),
        })
        .await;
        let mut hold_stale_spec = base_vision_spec.clone();
        hold_stale_spec.on_stale = VisionStalePolicy::Hold;
        let out_b = evaluate_final_commit_gate(
            &mut prep,
            None,
            now_b,
            period_ns,
            None,
            100,
            200,
            Some(100),
            Some(200),
            100,
            None,
            Some(&hold_stale_spec),
            None,
            Some(1),
            Some(&collector),
            Some(7),
            None,
            now_b,
            Some(now_b),
            Some(target_id),
            None,
            None,
        );
        assert!(matches!(out_b, FinalCommitGateOutcome::Hold));
        assert!(prep.chunk_ring.is_empty());

        // (c) Model generation changed -> Fail("MODEL_CHANGED") + clear ring.
        let now_c = Instant::now();
        stage_chunk(&mut prep, now_c);
        push_and_wait(PolicyVisionFrame {
            frame_id: 4,
            detections: vec![make_det("cube")],
            captured_at: Some(now_c),
            model_generation: Some(8),
        })
        .await;
        let out_c = evaluate_final_commit_gate(
            &mut prep,
            None,
            now_c,
            period_ns,
            None,
            100,
            200,
            Some(100),
            Some(200),
            100,
            None,
            Some(&base_vision_spec),
            None,
            Some(1),
            Some(&collector),
            Some(7),
            None,
            now_c,
            Some(now_c),
            Some(target_id),
            None,
            None,
        );
        assert!(matches!(
            out_c,
            FinalCommitGateOutcome::Fail {
                reason: "VISION_GENERATION_CHANGED",
                ..
            }
        ));
        assert!(prep.chunk_ring.is_empty());

        // (d) Target lost: Stop -> Fail("TARGET_LOST"), Hold -> Hold, ZeroFeatures -> Skip + clear ring.
        let now_d = Instant::now();
        push_and_wait(PolicyVisionFrame {
            frame_id: 5,
            detections: vec![],
            captured_at: Some(now_d),
            model_generation: Some(7),
        })
        .await;
        stage_chunk(&mut prep, now_d);
        let out_d_stop = evaluate_final_commit_gate(
            &mut prep,
            None,
            now_d,
            period_ns,
            None,
            100,
            200,
            Some(100),
            Some(200),
            100,
            None,
            Some(&base_vision_spec),
            None,
            Some(1),
            Some(&collector),
            Some(7),
            None,
            now_d,
            Some(now_d),
            Some(target_id),
            None,
            None,
        );
        assert!(matches!(
            out_d_stop,
            FinalCommitGateOutcome::Fail {
                reason: "TARGET_LOST",
                ..
            }
        ));
        assert!(prep.chunk_ring.is_empty());

        let mut lost_hold_spec = base_vision_spec.clone();
        lost_hold_spec.on_target_lost = VisionTargetLostPolicy::Hold;
        stage_chunk(&mut prep, now_d);
        let out_d_hold = evaluate_final_commit_gate(
            &mut prep,
            None,
            now_d,
            period_ns,
            None,
            100,
            200,
            Some(100),
            Some(200),
            100,
            None,
            Some(&lost_hold_spec),
            None,
            Some(1),
            Some(&collector),
            Some(7),
            None,
            now_d,
            Some(now_d),
            Some(target_id),
            None,
            None,
        );
        assert!(matches!(out_d_hold, FinalCommitGateOutcome::Hold));
        assert!(prep.chunk_ring.is_empty());

        let mut lost_zero_spec = base_vision_spec;
        lost_zero_spec.on_target_lost = VisionTargetLostPolicy::ZeroFeatures;
        stage_chunk(&mut prep, now_d);
        let out_d_zero = evaluate_final_commit_gate(
            &mut prep,
            None,
            now_d,
            period_ns,
            None,
            100,
            200,
            Some(100),
            Some(200),
            100,
            None,
            Some(&lost_zero_spec),
            None,
            Some(1),
            Some(&collector),
            Some(7),
            None,
            now_d,
            Some(now_d),
            Some(target_id),
            None,
            None,
        );
        assert!(matches!(out_d_zero, FinalCommitGateOutcome::Skip));
        assert!(prep.chunk_ring.is_empty());
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn chunking_r3_rt_refusal_generation_handoff_prevents_stale_slot_survival() {
        use crate::model::HolderLabel;
        use crate::rt_core::broker::{ActionId, Authority, ClientId, Reply, TransitionOp};
        use crate::rt_core::executor::ExecutorConfig;
        use crate::rt_core::mailbox::{PolicyValidityBounds, Submitted};
        use crate::rt_core::requests::lease::{
            ConnectionHandle, LeaseAcquire, LeaseOutcome, LeaseRequest,
        };
        use crate::rt_core::requests::policy::{
            BeginPolicySpec, PolicyCommand, ReservePolicyLoadSpec,
        };
        use crate::supervisor::targets::{PolicyStamp, PolicyTarget};

        let (snap, contract, mut cfg) = make_chunk_snapshot_and_contract(4, 2, None);
        cfg.lease_timeout_ms = 10_000;
        let dim = cfg.dof;
        let (backend, _) =
            CountingChunkBackend::new(dim, 4 * dim, vec![vec![0.02; 4 * dim]]);
        let mut prep =
            PreparedPolicy::from_backend(snap, contract, Box::new(backend), &cfg).unwrap();

        let sup = Arc::new(ArmSupervisor::new(cfg));
        let handle = sup
            .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
            .await
            .unwrap();
        let dl_boot = Instant::now() + Duration::from_secs(2);
        while sup.hub().latest().is_none() && Instant::now() < dl_boot {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        let client_id = 909u64;
        let conn = ConnectionHandle::try_new(client_id, "chunk-r3-refusal").unwrap();
        let _ = handle.open_connection_async(client_id).await.unwrap();
        let epoch_id = sup.hub().latest().unwrap().obs.epoch_id;
        let lease_reply = handle
            .submit_lease_async(
                ClientId(client_id),
                1,
                Authority {
                    lease_generation: 0,
                    epoch: epoch_id,
                    stop_generation: sup.current_stop_gen(),
                    config_revision: 0,
                    parent_generation: 0,
                },
                Instant::now() + Duration::from_secs(2),
                LeaseRequest::Acquire(LeaseAcquire {
                    connection: conn,
                    label: HolderLabel::new("chunk-r3-refusal"),
                    has_label: true,
                }),
            )
            .await
            .unwrap();
        let lease_gen = match lease_reply {
            Reply::Lease {
                outcome: LeaseOutcome::Granted { generation, .. },
                ..
            } => generation,
            other => panic!("LeaseAcquire failed: {other:?}"),
        };
        assert!(lease_gen > 0);
        let epoch_id = sup.hub().latest().unwrap().obs.epoch_id;
        // Torque on the sim supervisor via executor transition so Live policy is admitted.
        let arm_ack = handle
            .submit_transition_async(
                ClientId(client_id),
                2,
                Authority {
                    lease_generation: lease_gen,
                    epoch: epoch_id,
                    stop_generation: sup.current_stop_gen(),
                    config_revision: 0,
                    parent_generation: 0,
                },
                Instant::now() + Duration::from_secs(2),
                TransitionOp::TorqueOn,
            )
            .await
            .unwrap();
        assert!(matches!(arm_ack, Reply::Terminal { .. }), "TorqueOn failed: {arm_ack:?}");

        let boot = handle.boot();
        let epoch_id = sup.hub().latest().unwrap().obs.epoch_id;
        let authority = Authority {
            lease_generation: lease_gen,
            epoch: epoch_id,
            stop_generation: sup.current_stop_gen(),
            config_revision: 0,
            parent_generation: 0,
        };
        let mut token_guard = sup.policy_cancel_slots().claim_slot().unwrap();
        let res_r = handle
            .submit_policy_command_async(
                ClientId(client_id),
                3,
                authority,
                Instant::now() + Duration::from_secs(2),
                PolicyCommand::ReserveLoad(ReservePolicyLoadSpec {
                    load_token: token_guard.token(),
                    cancel_slot: token_guard.slot(),
                    admission_request_id: 3,
                    prepared_boot: boot,
                    connection: conn,
                    mode: PolicyMode::Live,
                    sim_only: true,
                    i_am_at_the_arm: true,
                    operator: crate::auth::OperatorGrant::unenforced(),
                    reservation_expires_at: Instant::now() + Duration::from_secs(10),
                }),
                |req_id| token_guard.record_admitted_req(req_id.0),
            )
            .await
            .unwrap();
        assert!(matches!(res_r, Reply::Ack { .. }), "ReserveLoad failed: {res_r:?}");

        let begin_r = handle
            .submit_policy_command_async(
                ClientId(client_id),
                4,
                authority,
                Instant::now() + Duration::from_secs(2),
                PolicyCommand::BeginPolicy(BeginPolicySpec {
                    load_token: token_guard.token(),
                    cancel_slot: token_guard.slot(),
                    admission_request_id: 3,
                    prepared_boot: boot,
                    connection: conn,
                    operator: crate::auth::OperatorGrant::unenforced(),
                    calibration_revision: 0,
                    policy_id: HolderLabel::new("exec_chunk_r3"),
                    artifact_digest: HolderLabel::new(prep.artifact_digest()),
                    backend: HolderLabel::new(prep.backend_name()),
                    mode: PolicyMode::Live,
                    sim_only: true,
                    i_am_at_the_arm: true,
                    max_first_step_rad: DEFAULT_MAX_FIRST_STEP_RAD,
                    max_step_rad: None,
                    max_input_age_ms: 200,
                    max_vision_age_ms: None,
                    vision_stale_hold: false,
                    period_us: 50_000,
                    deadline: Some(Instant::now() + Duration::from_secs(5)),
                    has_verified_release_gate: false,
                    gate_policy_sha256: HolderLabel::new(""),
                    gate_calibration_digest: HolderLabel::new(""),
                    gate_max_speed_scale: 1.0,
                    gate_is_production_release: false,
                    production_qualified_verified: false,
                }),
                |req_id| token_guard.record_admitted_req(req_id.0),
            )
            .await
            .unwrap();
        let action_id = match begin_r {
            Reply::Ack {
                action: Some(ActionId(id)),
                ..
            } => id,
            other => panic!("BeginPolicy failed: {other:?}"),
        };
        let dl_act = Instant::now() + Duration::from_secs(2);
        while !sup
            .hub()
            .latest()
            .and_then(|s| s.obs.policy.clone())
            .is_some_and(|p| p.active && p.action_id == action_id)
            && Instant::now() < dl_act
        {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        let authorization = sup.rt_policy_authorization();

        let g0 = handle.policy_refusal_gen();
        let snap_now = sup.hub().latest().unwrap();
        let snap_authority = snap_now.rt.owner.authority;

        // 1. Submit slot 0 with already-past `chunk_period_end` -> RT refuses `DEADLINE_EXPIRED`
        //    and immediately bumps `policy_refusal_gen` (`g0 -> g0 + 1`).
        let mut q0 = vec![0.0f32; dim];
        q0[0] = 0.02;
        let sub0 = handle.submit_policy_with_validity(
            &PolicyTarget {
                action_id,
                stamp: PolicyStamp {
                    authority: snap_authority,
                    authorization: authorization.current(),
                    max_input_age: Duration::from_millis(200),
                },
                targets: &q0,
                input_at: Instant::now(),
                vision_at: None,
                vision_frame_id: None,
            },
            PolicyValidityBounds {
                vision_window_expires_at: None,
                handshake_expires_at: None,
                chunk_slot_expires_at: None,
                chunk_period_end: Instant::now().checked_sub(Duration::from_millis(1)),
                expected_refusal_gen: Some(g0),
            },
        );
        assert!(matches!(sub0, Submitted::Accepted));

        let dl_r0 = Instant::now() + Duration::from_secs(2);
        let mut outcomes0 = Vec::new();
        while outcomes0.is_empty() && Instant::now() < dl_r0 {
            outcomes0 = handle.drain_policy_outcomes(action_id);
            if outcomes0.is_empty() {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        }
        assert_eq!(
            outcomes0[0].result.unwrap_err().code,
            "DEADLINE_EXPIRED",
            "slot 0 must be refused DEADLINE_EXPIRED"
        );
        assert_eq!(
            handle.policy_refusal_gen(),
            g0 + 1,
            "primary RT refusal must bump policy_refusal_gen immediately"
        );

        // 2. Submit slot 1 carrying STALE `expected_refusal_gen = Some(g0)` (even with current
        //    authorization generation) -> RT MUST refuse it with `POLICY_REVOKED` and NOT
        //    double-bump `policy_refusal_gen`.
        let mut q1 = vec![0.0f32; dim];
        q1[0] = 0.04;
        let sub1 = handle.submit_policy_with_validity(
            &PolicyTarget {
                action_id,
                stamp: PolicyStamp {
                    authority: snap_authority,
                    authorization: authorization.current(),
                    max_input_age: Duration::from_millis(200),
                },
                targets: &q1,
                input_at: Instant::now(),
                vision_at: None,
                vision_frame_id: None,
            },
            PolicyValidityBounds {
                vision_window_expires_at: None,
                handshake_expires_at: None,
                chunk_slot_expires_at: None,
                chunk_period_end: Some(Instant::now() + Duration::from_secs(1)),
                expected_refusal_gen: Some(g0),
            },
        );
        assert!(matches!(sub1, Submitted::Accepted));

        let dl_r1 = Instant::now() + Duration::from_secs(2);
        let mut outcomes1 = Vec::new();
        while outcomes1.is_empty() && Instant::now() < dl_r1 {
            outcomes1 = handle.drain_policy_outcomes(action_id);
            if outcomes1.is_empty() {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        }
        assert_eq!(
            outcomes1[0].result.unwrap_err().code,
            "POLICY_REVOKED",
            "stale expected_refusal_gen slot must be refused POLICY_REVOKED"
        );
        assert_eq!(
            handle.policy_refusal_gen(),
            g0 + 1,
            "cascaded expected_refusal_gen refusal must NOT double-bump policy_refusal_gen"
        );

        // 3. `executor_commit_gate` with stale `last_seen_refusal_gen = g0` MUST clear the
        //    chunk ring and return `Skip` without submitting even when outcome queue is empty.
        let now_gate = Instant::now();
        prep.plan_chunk_step(
            &vec![0.0f32; dim],
            None,
            0,
            ChunkSource::from_joint(now_gate, None),
            None,
        )
        .unwrap();
        assert!(!prep.chunk_ring.is_empty());
        let mut last_seen = g0;
        let mut shadow_steps = 0u64;
        let mut step_stats = PolicyStepStats::default();
        step_stats.set_period_us(50_000);
        let submitted_before = handle.policy_submitted_count();
        let gate_out = executor_commit_gate(
            &sup,
            &handle,
            &mut prep,
            action_id,
            lease_gen,
            PolicyMode::Live,
            None,
            Duration::from_millis(200),
            now_gate,
            Duration::from_millis(50).as_nanos(),
            None,
            200,
            400,
            None,
            None,
            100,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            snap_authority,
            &authorization,
            authorization.current(),
            &mut last_seen,
            now_gate,
            None,
            now_gate,
            None,
            None,
            None,
            None,
            None,
            0,
            0,
            &mut shadow_steps,
            &mut step_stats,
        )
        .await;
        assert!(matches!(gate_out, ExecutorCommitOutcome::Skip));
        assert!(
            prep.chunk_ring.is_empty(),
            "stale last_seen_refusal_gen must clear whole chunk ring"
        );
        assert_eq!(last_seen, g0 + 1);
        assert_eq!(
            handle.policy_submitted_count(),
            submitted_before,
            "no target may be published when refusal generation advanced"
        );

        // 4. Fresh slot carrying refreshed `expected_refusal_gen = Some(g0 + 1)` is accepted.
        let mut q2 = vec![0.0f32; dim];
        q2[0] = 0.02;
        let sub2 = handle.submit_policy_with_validity(
            &PolicyTarget {
                action_id,
                stamp: PolicyStamp {
                    authority: snap_authority,
                    authorization: authorization.current(),
                    max_input_age: Duration::from_millis(200),
                },
                targets: &q2,
                input_at: Instant::now(),
                vision_at: None,
                vision_frame_id: None,
            },
            PolicyValidityBounds {
                vision_window_expires_at: None,
                handshake_expires_at: None,
                chunk_slot_expires_at: None,
                chunk_period_end: Some(Instant::now() + Duration::from_secs(1)),
                expected_refusal_gen: Some(g0 + 1),
            },
        );
        assert!(matches!(sub2, Submitted::Accepted));
        let dl_r2 = Instant::now() + Duration::from_secs(2);
        let mut outcomes2 = Vec::new();
        while outcomes2.is_empty() && Instant::now() < dl_r2 {
            outcomes2 = handle.drain_policy_outcomes(action_id);
            if outcomes2.is_empty() {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        }
        assert!(
            outcomes2[0].result.is_ok(),
            "fresh slot with matching expected_refusal_gen must be accepted: {:?}",
            outcomes2[0].result
        );

        token_guard.disarm();
        handle.stop_and_join();
    }
}
