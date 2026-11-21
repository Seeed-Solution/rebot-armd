//! Asynchronous vision feature collection, strict staleness/generation/sync-skew
//! validation, and `top1_box_norm_v1` feature extraction for `policy_runtime`.

use std::collections::{HashMap, VecDeque};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock, Weak};
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

use crate::grasp::{BoxFuture, Detection, DetectionSource};
use crate::supervisor::ArmSupervisor;

use super::manifest::{
    VideoEmbeddingSpec, VisionFeatureSpec, VisionStalePolicy, VisionTargetLostPolicy,
    MAX_POLICY_INPUT_WIDTH, MAX_VIDEO_WINDOW_SPAN_MS, MAX_VISION_STALENESS_MS,
};

/// One detection frame received from a [`PolicyVisionSource`].
#[derive(Debug, Clone)]
pub struct PolicyVisionFrame {
    pub frame_id: u64,
    /// Strictly `None` when upstream did not report a valid capture timestamp/age.
    pub captured_at: Option<Instant>,
    pub detections: Vec<Detection>,
    pub model_generation: Option<u64>,
}

/// Async source of timestamped vision detection frames for `policy_runtime`.
pub trait PolicyVisionSource: Send + Sync {
    fn next_frame<'a>(
        &'a self,
        after_frame_id: u64,
        wait_ms: u64,
    ) -> BoxFuture<'a, Result<Option<PolicyVisionFrame>, String>>;
}

/// Adapts armd's [`DetectionSource`] into [`PolicyVisionSource`], preserving
/// `frame_id`, `captured_at`, `detections`, and `model_generation`.
pub struct DetectionSourceAdapter(pub Arc<dyn DetectionSource>);

impl PolicyVisionSource for DetectionSourceAdapter {
    fn next_frame<'a>(
        &'a self,
        after_frame_id: u64,
        wait_ms: u64,
    ) -> BoxFuture<'a, Result<Option<PolicyVisionFrame>, String>> {
        let after = if after_frame_id == 0 {
            None
        } else {
            Some(after_frame_id)
        };
        Box::pin(async move {
            let frame = self.0.next_frame(after, wait_ms).await?;
            Ok(frame.map(|f| PolicyVisionFrame {
                frame_id: f.frame_id,
                captured_at: f.policy_captured_at,
                detections: f.detections,
                model_generation: f.model_generation,
            }))
        })
    }
}

/// Independent background collector that polls a [`PolicyVisionSource`] and keeps
/// only the latest received frame in a short-held `std::sync::Mutex` so the
/// policy control tick and supervisor lock never block on vision I/O.
pub struct VisionFeatureCollector {
    cancelled: Arc<AtomicBool>,
    latest: Arc<Mutex<Option<PolicyVisionFrame>>>,
    task: tokio::task::JoinHandle<()>,
}

impl VisionFeatureCollector {
    pub fn start(source: Arc<dyn PolicyVisionSource>, wait_ms: u64) -> Self {
        let cancelled = Arc::new(AtomicBool::new(false));
        let latest = Arc::new(Mutex::new(None));
        let cancelled_c = cancelled.clone();
        let latest_c = latest.clone();
        let wait_ms = wait_ms.clamp(10, 500);

        let task = tokio::spawn(async move {
            let mut after_frame_id: u64 = 0;
            while !cancelled_c.load(Ordering::SeqCst) {
                let call_started = Instant::now();
                match source.next_frame(after_frame_id, wait_ms).await {
                    Ok(Some(frame)) => {
                        let fid = frame.frame_id;
                        {
                            let mut guard =
                                latest_c.lock().unwrap_or_else(|e| e.into_inner());
                            *guard = Some(frame);
                        }
                        if fid > after_frame_id {
                            after_frame_id = fid;
                            if call_started.elapsed() < Duration::from_millis(2) {
                                tokio::time::sleep(Duration::from_millis(2)).await;
                            }
                        } else {
                            tokio::time::sleep(Duration::from_millis(5)).await;
                        }
                    }
                    Ok(None) => {
                        if call_started.elapsed() < Duration::from_millis(2) {
                            tokio::time::sleep(Duration::from_millis(5)).await;
                        }
                    }
                    Err(err) => {
                        tracing::debug!("Policy vision collector poll failed: {err}");
                        tokio::time::sleep(Duration::from_millis(10)).await;
                    }
                }
            }
        });

        Self {
            cancelled,
            latest,
            task,
        }
    }

    pub fn latest(&self) -> Option<PolicyVisionFrame> {
        self.latest
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
    }

    pub fn stop(&self) {
        self.cancelled.store(true, Ordering::SeqCst);
        self.task.abort();
    }
}

impl Drop for VisionFeatureCollector {
    fn drop(&mut self) {
        self.stop();
    }
}

/// Extract `[1.0, cx, cy, w, h, score]` from the highest-score valid detection
/// matching `spec.target_label` and `spec.min_score`.
pub fn extract_top1_box_norm_v1(
    frame: &PolicyVisionFrame,
    spec: &VisionFeatureSpec,
) -> Option<[f32; 6]> {
    let mut best: Option<&Detection> = None;
    for det in &frame.detections {
        if let Some(ref target) = spec.target_label {
            if det.label != *target {
                continue;
            }
        }
        if !det.raw_score_valid {
            continue;
        }
        if !det.score.is_finite() || !(0.0..=1.0).contains(&det.score) {
            continue;
        }
        let score_f32 = det.score as f32;
        if !score_f32.is_finite() || score_f32 < spec.min_score {
            continue;
        }
        if !det
            .box_norm
            .iter()
            .all(|v| v.is_finite() && (0.0..=1.0).contains(v))
        {
            continue;
        }
        let [x1, y1, x2, y2] = det.box_norm;
        if !(x2 > x1 && y2 > y1) {
            continue;
        }
        let replace = match best {
            None => true,
            Some(cur) => det.score > cur.score,
        };
        if replace {
            best = Some(det);
        }
    }

    let det = best?;
    let [x1, y1, x2, y2] = det.box_norm;
    let cx = ((x1 + x2) * 0.5) as f32;
    let cy = ((y1 + y2) * 0.5) as f32;
    let w = (x2 - x1).abs() as f32;
    let h = (y2 - y1).abs() as f32;
    let score = det.score as f32;
    if !cx.is_finite()
        || !cy.is_finite()
        || !w.is_finite()
        || !h.is_finite()
        || !score.is_finite()
    {
        return None;
    }
    Some([1.0, cx, cy, w, h, score])
}

#[derive(Debug, Clone, PartialEq)]
pub enum VisionStepDecision {
    Ready {
        features: [f32; 6],
        vision_age_ms: u64,
        frame_id: u64,
    },
    Hold,
    Stop {
        reason: &'static str,
        message: String,
    },
}

fn stale_decision(
    policy: VisionStalePolicy,
    reason: &'static str,
    message: String,
) -> VisionStepDecision {
    match policy {
        VisionStalePolicy::Hold => VisionStepDecision::Hold,
        VisionStalePolicy::Stop => VisionStepDecision::Stop { reason, message },
    }
}

/// Evaluate the latest cached vision frame against strict age, timestamp, model-generation,
/// sync-skew, and target-lost rules.
pub fn evaluate_vision_frame(
    frame: Option<&PolicyVisionFrame>,
    spec: &VisionFeatureSpec,
    run_started_at: Instant,
    now: Instant,
    prop_age_range_ms: Option<(u64, u64)>,
    locked_generation: &mut Option<u64>,
) -> VisionStepDecision {
    let Some(frame) = frame else {
        let elapsed_ms = now
            .saturating_duration_since(run_started_at)
            .as_millis()
            .min(u64::MAX as u128) as u64;
        if elapsed_ms <= spec.max_staleness_ms {
            return VisionStepDecision::Hold;
        }
        return stale_decision(
            spec.on_stale,
            "VISION_STALE",
            format!("No vision frame received within {} ms", spec.max_staleness_ms),
        );
    };

    let Some(captured_at) = frame.captured_at else {
        return stale_decision(
            spec.on_stale,
            "VISION_TIMESTAMP_MISSING",
            format!("Vision frame {} has no capture timestamp", frame.frame_id),
        );
    };

    let vision_age_ms = now
        .saturating_duration_since(captured_at)
        .as_millis()
        .min(u64::MAX as u128) as u64;
    if vision_age_ms > spec.max_staleness_ms {
        return stale_decision(
            spec.on_stale,
            "VISION_STALE",
            format!(
                "Vision frame {} is stale ({vision_age_ms} ms > {} ms)",
                frame.frame_id, spec.max_staleness_ms
            ),
        );
    }

    if spec.require_model_generation {
        match frame.model_generation {
            None => {
                return VisionStepDecision::Stop {
                    reason: "VISION_GENERATION_MISSING",
                    message: format!(
                        "Vision frame {} missing required model_generation",
                        frame.frame_id
                    ),
                };
            }
            Some(gen) => {
                if let Some(prev) = *locked_generation {
                    if gen != prev {
                        return VisionStepDecision::Stop {
                            reason: "VISION_GENERATION_CHANGED",
                            message: format!(
                                "Vision model generation changed from {prev} to {gen}"
                            ),
                        };
                    }
                } else {
                    *locked_generation = Some(gen);
                }
            }
        }
    }

    let Some((min_prop, max_prop)) = prop_age_range_ms else {
        return stale_decision(
            spec.on_stale,
            "VISION_SYNC_SKEW",
            "Proprioception feedback age unavailable for vision sync-skew check".into(),
        );
    };
    let max_skew = vision_age_ms
        .abs_diff(min_prop)
        .max(vision_age_ms.abs_diff(max_prop));
    if max_skew > spec.max_sync_skew_ms {
        return stale_decision(
            spec.on_stale,
            "VISION_SYNC_SKEW",
            format!(
                "Vision/proprioception sync skew {max_skew} ms exceeds {} ms",
                spec.max_sync_skew_ms
            ),
        );
    }

    match extract_top1_box_norm_v1(frame, spec) {
        Some(features) => VisionStepDecision::Ready {
            features,
            vision_age_ms,
            frame_id: frame.frame_id,
        },
        None => match spec.on_target_lost {
            VisionTargetLostPolicy::ZeroFeatures => VisionStepDecision::Ready {
                features: [0.0; 6],
                vision_age_ms,
                frame_id: frame.frame_id,
            },
            VisionTargetLostPolicy::Hold => VisionStepDecision::Hold,
            VisionTargetLostPolicy::Stop => VisionStepDecision::Stop {
                reason: "TARGET_LOST",
                message: match &spec.target_label {
                    Some(lbl) => format!(
                        "No detection matching label '{lbl}' with score >= {:.2}",
                        spec.min_score
                    ),
                    None => format!("No detection with score >= {:.2}", spec.min_score),
                },
            },
        },
    }
}

// ---------------------------------------------------------------------------
// Contract C: Video Embedding Ring Buffer & Bilateral Clock Handshake
// ---------------------------------------------------------------------------

pub const DEFAULT_HANDSHAKE_TTL_MS: u64 = 60_000;
pub const MAX_SAME_HOST_UNCERTAINTY_MS: u64 = 5;
pub const MAX_REMOTE_UNCERTAINTY_MS: u64 = 25;
pub const MAX_FUTURE_SKEW_CAP_MS: u64 = 25;
const UNBOUND_RING_CAP: usize = 8;

static MONOTONIC_EPOCH: OnceLock<Instant> = OnceLock::new();

/// Process-wide monotonic clock in milliseconds (immune to wall-clock rollbacks).
pub fn monotonic_now_ms() -> u64 {
    let epoch = MONOTONIC_EPOCH.get_or_init(Instant::now);
    epoch.elapsed().as_millis().min(u64::MAX as u128) as u64
}

fn normalize_sha256_hex(raw: &str, field: &str) -> Result<String, EmbeddingIngestError> {
    let trimmed = raw.trim();
    if trimmed.len() != 64 || !trimmed.bytes().all(|b| b.is_ascii_hexdigit()) {
        return Err(EmbeddingIngestError {
            status: 400,
            code: "INVALID_SHA256",
            message: format!("{field} must be a 64-character hex SHA-256 digest"),
        });
    }
    Ok(trimmed.to_ascii_lowercase())
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ClockHandshakeState {
    pub handshake_generation: u64,
    pub producer_epoch: u64,
    pub clock_offset_ms: i64,
    pub server_uncertainty_ms: u64,
    pub producer_uncertainty_ms: u64,
    pub handshake_expires_at_ms: u64,
}

impl ClockHandshakeState {
    pub fn effective_uncertainty_ms(&self) -> u64 {
        self.server_uncertainty_ms.max(self.producer_uncertainty_ms)
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ClockHandshakeRequest {
    #[serde(default)]
    pub producer_epoch: u64,
    #[serde(default, alias = "client_monotonic_ms")]
    pub producer_monotonic_ms: Option<i64>,
    #[serde(default)]
    pub producer_uncertainty_ms: Option<u64>,
    #[serde(default)]
    pub server_uncertainty_ms: Option<u64>,
    #[serde(default)]
    pub rtt_ms: Option<u64>,
    #[serde(default)]
    pub ttl_ms: Option<u64>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ClockHandshakeResponse {
    pub handshake_generation: u64,
    pub producer_epoch: u64,
    pub same_host: bool,
    pub clock_offset_ms: i64,
    pub server_uncertainty_ms: u64,
    pub producer_uncertainty_ms: u64,
    pub effective_uncertainty_ms: u64,
    pub server_monotonic_ms: u64,
    pub handshake_expires_at_ms: u64,
    pub ttl_ms: u64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct EmbeddingIngestRequest {
    pub handshake_generation: u64,
    pub producer_epoch: u64,
    pub frame_id: u64,
    pub captured_at_ms: i64,
    pub embedding: Vec<f64>,
    pub encoder_artifact_sha256: String,
    pub preprocess_config_sha256: String,
    #[serde(default)]
    pub model_generation: Option<u64>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EmbeddingIngestResponse {
    pub accepted: bool,
    pub frame_id: u64,
    pub producer_epoch: u64,
    pub mapped_captured_at_ms: u64,
    pub window_len: usize,
    pub history_len: usize,
    pub window_ready: bool,
    pub window_newest_deadline_ms: u64,
    pub window_oldest_deadline_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EmbeddingIngestError {
    pub status: u16,
    pub code: &'static str,
    pub message: String,
}

impl std::fmt::Display for EmbeddingIngestError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}: {}", self.code, self.message)
    }
}

impl std::error::Error for EmbeddingIngestError {}

#[derive(Debug, Clone)]
pub struct EmbeddingFrame {
    pub frame_id: u64,
    pub producer_epoch: u64,
    pub handshake_generation: u64,
    pub captured_at_ms: i64,
    pub mapped_at_ms: u64,
    pub embedding: Vec<f32>,
    pub encoder_artifact_sha256: String,
    pub preprocess_config_sha256: String,
    pub model_generation: Option<u64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct EmbeddingWindowSnapshot {
    pub features: Vec<f32>,
    pub newest_frame_id: u64,
    pub newest_mapped_at_ms: u64,
    pub producer_epoch: u64,
    pub model_generation: Option<u64>,
    pub invalidation_generation: u64,
    pub clock_handshake_generation: u64,
    pub clock_handshake_expires_at_ms: u64,
    pub window_newest_deadline_ms: u64,
    pub window_oldest_deadline_ms: u64,
    pub policy_instance_id: Option<u64>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum EmbeddingSampleDecision {
    Ready(EmbeddingWindowSnapshot),
    Hold,
    Stop {
        reason: &'static str,
        message: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LiveEmbeddingTicketState {
    pub now_armd_ms: u64,
    pub policy_instance_id: Option<u64>,
    pub producer_epoch: Option<u64>,
    pub model_generation: Option<u64>,
    pub invalidation_generation: u64,
    pub clock_handshake_generation: u64,
    pub clock_handshake_expires_at_ms: u64,
}

#[derive(Debug, Clone)]
struct BoundEmbeddingPolicy {
    policy_instance_id: u64,
    spec: VideoEmbeddingSpec,
    effective_staleness_ms: u64,
}

#[derive(Debug)]
struct EmbeddingHubInner {
    test_now_ms: Option<u64>,
    default_handshake_ttl_ms: u64,
    handshake_generation: u64,
    handshake: Option<ClockHandshakeState>,
    bound: Option<BoundEmbeddingPolicy>,
    invalidation_generation: u64,
    producer_epoch: Option<u64>,
    last_frame_id: Option<u64>,
    model_generation: Option<u64>,
    last_frame_model_generation: Option<u64>,
    has_seen_frame_in_epoch: bool,
    frames: VecDeque<EmbeddingFrame>,
    window_newest_deadline_ms: Option<u64>,
    window_oldest_deadline_ms: Option<u64>,
}

impl Default for EmbeddingHubInner {
    fn default() -> Self {
        Self {
            test_now_ms: None,
            default_handshake_ttl_ms: DEFAULT_HANDSHAKE_TTL_MS,
            handshake_generation: 0,
            handshake: None,
            bound: None,
            invalidation_generation: 0,
            producer_epoch: None,
            last_frame_id: None,
            model_generation: None,
            last_frame_model_generation: None,
            has_seen_frame_in_epoch: false,
            frames: VecDeque::new(),
            window_newest_deadline_ms: None,
            window_oldest_deadline_ms: None,
        }
    }
}

impl EmbeddingHubInner {
    fn now_ms(&self) -> u64 {
        self.test_now_ms.unwrap_or_else(monotonic_now_ms)
    }

    fn recompute_deadlines(&mut self) {
        if self.frames.is_empty() {
            self.window_newest_deadline_ms = None;
            self.window_oldest_deadline_ms = None;
            return;
        }
        let staleness = self
            .bound
            .as_ref()
            .map_or(MAX_VISION_STALENESS_MS, |b| b.effective_staleness_ms);
        let span = self
            .bound
            .as_ref()
            .map_or(MAX_VIDEO_WINDOW_SPAN_MS, |b| b.spec.max_window_span_ms);
        let oldest = self
            .frames
            .iter()
            .map(|f| f.mapped_at_ms)
            .min()
            .expect("non-empty");
        let newest = self
            .frames
            .iter()
            .map(|f| f.mapped_at_ms)
            .max()
            .expect("non-empty");
        self.window_newest_deadline_ms = Some(newest.saturating_add(staleness));
        self.window_oldest_deadline_ms = Some(oldest.saturating_add(span));
    }

    fn live_ticket_state(&self) -> LiveEmbeddingTicketState {
        let now_armd_ms = self.now_ms();
        let (clock_handshake_generation, clock_handshake_expires_at_ms) = match &self.handshake {
            Some(hs) => (hs.handshake_generation, hs.handshake_expires_at_ms),
            None => (0, 0),
        };
        LiveEmbeddingTicketState {
            now_armd_ms,
            policy_instance_id: self.bound.as_ref().map(|b| b.policy_instance_id),
            producer_epoch: self.producer_epoch,
            model_generation: self.model_generation,
            invalidation_generation: self.invalidation_generation,
            clock_handshake_generation,
            clock_handshake_expires_at_ms,
        }
    }
}

/// Held synchronously across ticket verification and `policy_step_with_vision`
/// (always acquired AFTER `SupervisorInner` lock) to eliminate commit TOCTOU.
pub struct HubCommitGuard<'a> {
    inner: MutexGuard<'a, EmbeddingHubInner>,
}

impl<'a> HubCommitGuard<'a> {
    pub fn live_ticket_state(&self) -> LiveEmbeddingTicketState {
        self.inner.live_ticket_state()
    }
}

#[derive(Debug, Default)]
pub struct VideoEmbeddingHub {
    inner: Mutex<EmbeddingHubInner>,
}

impl VideoEmbeddingHub {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn lock_for_commit(&self) -> HubCommitGuard<'_> {
        HubCommitGuard {
            inner: self.inner.lock().unwrap_or_else(|e| e.into_inner()),
        }
    }

    pub fn set_test_now_ms(&self, now_ms: Option<u64>) {
        let mut inner = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        inner.test_now_ms = now_ms;
    }

    pub fn advance_test_now_ms(&self, delta_ms: u64) -> u64 {
        let mut inner = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let next = inner.now_ms().saturating_add(delta_ms);
        inner.test_now_ms = Some(next);
        next
    }

    pub fn now_armd_ms(&self) -> u64 {
        let inner = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        inner.now_ms()
    }

    pub fn set_default_handshake_ttl_ms(&self, ttl_ms: u64) {
        let mut inner = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        inner.default_handshake_ttl_ms = ttl_ms.max(1);
    }

    pub fn handshake_state(&self) -> Option<ClockHandshakeState> {
        let inner = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        inner.handshake.clone()
    }

    pub fn window_len(&self) -> usize {
        let inner = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        inner.frames.len()
    }

    pub fn window_deadlines(&self) -> Option<(u64, u64)> {
        let inner = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        inner
            .window_newest_deadline_ms
            .zip(inner.window_oldest_deadline_ms)
    }

    pub fn frame_ids(&self) -> Vec<u64> {
        let inner = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        inner.frames.iter().map(|f| f.frame_id).collect()
    }

    pub fn invalidation_generation(&self) -> u64 {
        let inner = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        inner.invalidation_generation
    }

    pub fn bump_invalidation_generation(&self) -> u64 {
        let mut inner = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        inner.invalidation_generation = inner.invalidation_generation.saturating_add(1);
        inner.invalidation_generation
    }

    pub fn set_model_generation(&self, gen: Option<u64>) {
        let mut inner = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        inner.model_generation = gen;
    }

    pub fn set_producer_epoch(&self, epoch: u64) {
        let mut inner = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        if inner.producer_epoch != Some(epoch) {
            inner.producer_epoch = Some(epoch);
            inner.frames.clear();
            inner.last_frame_id = None;
            inner.has_seen_frame_in_epoch = false;
            inner.last_frame_model_generation = None;
            inner.recompute_deadlines();
            inner.invalidation_generation = inner.invalidation_generation.saturating_add(1);
        }
    }

    pub fn bind_policy(&self, policy_instance_id: u64, spec: VideoEmbeddingSpec, runner_fps: f64) {
        let mut inner = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let now_ms = inner.now_ms();
        let effective_staleness_ms = spec.effective_staleness_ms_for_fps(runner_fps);
        let compatible = inner.frames.iter().all(|f| {
            f.embedding.len() == spec.embedding_dim
                && f.encoder_artifact_sha256 == spec.encoder_artifact_sha256
                && f.preprocess_config_sha256 == spec.preprocess_config_sha256
        });
        if !compatible {
            inner.frames.clear();
            // Do NOT reset `last_frame_id` for the active `producer_epoch` (A9).
        } else {
            while let Some(front) = inner.frames.front() {
                if now_ms.saturating_sub(front.mapped_at_ms) > spec.max_window_span_ms {
                    inner.frames.pop_front();
                } else {
                    break;
                }
            }
            while inner.frames.len() > spec.history_len {
                inner.frames.pop_front();
            }
        }
        inner.bound = Some(BoundEmbeddingPolicy {
            policy_instance_id,
            spec,
            effective_staleness_ms,
        });
        inner.model_generation = Some(inner.model_generation.unwrap_or(0).saturating_add(1));
        inner.invalidation_generation = inner.invalidation_generation.saturating_add(1);
        inner.recompute_deadlines();
    }

    pub fn unbind_policy(&self, policy_instance_id: u64) {
        let mut inner = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        if inner
            .bound
            .as_ref()
            .is_some_and(|b| b.policy_instance_id == policy_instance_id)
        {
            inner.bound = None;
            inner.model_generation = Some(inner.model_generation.unwrap_or(0).saturating_add(1));
            inner.invalidation_generation = inner.invalidation_generation.saturating_add(1);
        }
    }

    pub fn clock_handshake(
        &self,
        req: ClockHandshakeRequest,
        socket_same_host: bool,
    ) -> Result<ClockHandshakeResponse, EmbeddingIngestError> {
        let mut inner = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let now_ms = inner.now_ms();

        let producer_monotonic_ms = req.producer_monotonic_ms.ok_or_else(|| EmbeddingIngestError {
            status: 400,
            code: "MISSING_PRODUCER_TIMESTAMP",
            message: "producer_monotonic_ms (or client_monotonic_ms) is required".into(),
        })?;
        let producer_uncertainty_ms = req.producer_uncertainty_ms.ok_or_else(|| {
            EmbeddingIngestError {
                status: 400,
                code: "MISSING_UNCERTAINTY",
                message: "producer_uncertainty_ms is required".into(),
            }
        })?;

        let rtt_unc = req.rtt_ms.map_or(0, |r| r.div_ceil(2));
        let server_uncertainty_ms = req.server_uncertainty_ms.unwrap_or(0).max(rtt_unc);
        let effective_uncertainty_ms = server_uncertainty_ms.max(producer_uncertainty_ms);
        let max_allowed = if socket_same_host {
            MAX_SAME_HOST_UNCERTAINTY_MS
        } else {
            MAX_REMOTE_UNCERTAINTY_MS
        };
        if effective_uncertainty_ms > max_allowed {
            return Err(EmbeddingIngestError {
                status: 400,
                code: "UNCERTAINTY_EXCEEDED",
                message: format!(
                    "effective clock uncertainty {effective_uncertainty_ms} ms exceeds {} limit ({max_allowed} ms)",
                    if socket_same_host { "same-host" } else { "remote live" }
                ),
            });
        }

        let ttl_ms = match req.ttl_ms {
            Some(0) => {
                return Err(EmbeddingIngestError {
                    status: 400,
                    code: "INVALID_TTL",
                    message: "handshake ttl_ms must be >= 1".into(),
                });
            }
            Some(t) => t.min(DEFAULT_HANDSHAKE_TTL_MS),
            None => inner.default_handshake_ttl_ms.min(DEFAULT_HANDSHAKE_TTL_MS),
        };

        if let Some(cur_epoch) = inner.producer_epoch {
            if req.producer_epoch < cur_epoch {
                return Err(EmbeddingIngestError {
                    status: 409,
                    code: "PRODUCER_EPOCH_STALE",
                    message: format!(
                        "producer_epoch {} is older than active epoch {cur_epoch}",
                        req.producer_epoch
                    ),
                });
            }
        }

        let clock_offset_ms = ((now_ms as i128) - (producer_monotonic_ms as i128))
            .clamp(i64::MIN as i128, i64::MAX as i128) as i64;

        inner.handshake_generation = inner.handshake_generation.saturating_add(1);
        let gen = inner.handshake_generation;
        let expires_at = now_ms.saturating_add(ttl_ms);

        // Clear any frames mapped under the previous handshake/offset (A7).
        inner.frames.clear();
        inner.recompute_deadlines();

        if inner.producer_epoch != Some(req.producer_epoch) {
            inner.producer_epoch = Some(req.producer_epoch);
            inner.last_frame_id = None;
            inner.has_seen_frame_in_epoch = false;
            inner.last_frame_model_generation = None;
        }

        inner.handshake = Some(ClockHandshakeState {
            handshake_generation: gen,
            producer_epoch: req.producer_epoch,
            clock_offset_ms,
            server_uncertainty_ms,
            producer_uncertainty_ms,
            handshake_expires_at_ms: expires_at,
        });

        Ok(ClockHandshakeResponse {
            handshake_generation: gen,
            producer_epoch: req.producer_epoch,
            same_host: socket_same_host,
            clock_offset_ms,
            server_uncertainty_ms,
            producer_uncertainty_ms,
            effective_uncertainty_ms,
            server_monotonic_ms: now_ms,
            handshake_expires_at_ms: expires_at,
            ttl_ms,
        })
    }

    pub fn ingest_embedding(
        &self,
        req: EmbeddingIngestRequest,
    ) -> Result<EmbeddingIngestResponse, EmbeddingIngestError> {
        let mut inner = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let now_ms = inner.now_ms();

        // 1. Handshake validation (without mutating state on error)
        let hs = inner.handshake.clone().ok_or_else(|| EmbeddingIngestError {
            status: 409,
            code: "HANDSHAKE_MISMATCH",
            message: "No active clock handshake; call /api/v1/vision/clock-handshake first".into(),
        })?;
        if req.handshake_generation != hs.handshake_generation {
            return Err(EmbeddingIngestError {
                status: 409,
                code: "HANDSHAKE_MISMATCH",
                message: format!(
                    "handshake_generation mismatch: got {}, active is {}",
                    req.handshake_generation, hs.handshake_generation
                ),
            });
        }
        if now_ms > hs.handshake_expires_at_ms {
            return Err(EmbeddingIngestError {
                status: 409,
                code: "HANDSHAKE_EXPIRED",
                message: format!(
                    "clock handshake {} expired at {} ms (now {} ms)",
                    hs.handshake_generation, hs.handshake_expires_at_ms, now_ms
                ),
            });
        }

        // 2. Epoch must match active handshake's bound producer_epoch (A8)
        if req.producer_epoch != hs.producer_epoch
            || inner.producer_epoch != Some(req.producer_epoch)
        {
            return Err(EmbeddingIngestError {
                status: 409,
                code: "EPOCH_MISMATCH",
                message: format!(
                    "producer_epoch {} does not match active handshake epoch {}; perform a new clock-handshake",
                    req.producer_epoch, hs.producer_epoch
                ),
            });
        }

        // 3. SHA-256 & active policy artifact check
        let enc_sha =
            normalize_sha256_hex(&req.encoder_artifact_sha256, "encoder_artifact_sha256")?;
        let prep_sha =
            normalize_sha256_hex(&req.preprocess_config_sha256, "preprocess_config_sha256")?;
        if let Some(ref bound) = inner.bound {
            if enc_sha != bound.spec.encoder_artifact_sha256 {
                return Err(EmbeddingIngestError {
                    status: 409,
                    code: "ARTIFACT_MISMATCH",
                    message: format!(
                        "encoder_artifact_sha256 {enc_sha} does not match active policy {}",
                        bound.spec.encoder_artifact_sha256
                    ),
                });
            }
            if prep_sha != bound.spec.preprocess_config_sha256 {
                return Err(EmbeddingIngestError {
                    status: 409,
                    code: "ARTIFACT_MISMATCH",
                    message: format!(
                        "preprocess_config_sha256 {prep_sha} does not match active policy {}",
                        bound.spec.preprocess_config_sha256
                    ),
                });
            }
        }

        // 4. Pre-bind width budget (A13) & bound embedding dimension + finiteness validation
        if req.embedding.is_empty() || req.embedding.len() > MAX_POLICY_INPUT_WIDTH {
            return Err(EmbeddingIngestError {
                status: 400,
                code: "WIDTH_BUDGET_EXCEEDED",
                message: format!(
                    "embedding length {} outside allowed range 1..={MAX_POLICY_INPUT_WIDTH}",
                    req.embedding.len()
                ),
            });
        }
        if let Some(ref bound) = inner.bound {
            if req.embedding.len() != bound.spec.embedding_dim {
                return Err(EmbeddingIngestError {
                    status: 400,
                    code: "EMBEDDING_DIM_MISMATCH",
                    message: format!(
                        "expected embedding length {}, got {}",
                        bound.spec.embedding_dim,
                        req.embedding.len()
                    ),
                });
            }
        } else if let Some(first) = inner.frames.front() {
            if req.embedding.len() != first.embedding.len() {
                return Err(EmbeddingIngestError {
                    status: 400,
                    code: "EMBEDDING_DIM_MISMATCH",
                    message: format!(
                        "expected embedding length {}, got {}",
                        first.embedding.len(),
                        req.embedding.len()
                    ),
                });
            }
        }
        let mut emb_f32 = Vec::with_capacity(req.embedding.len());
        for (idx, &v) in req.embedding.iter().enumerate() {
            let f = v as f32;
            if !v.is_finite() || !f.is_finite() {
                return Err(EmbeddingIngestError {
                    status: 400,
                    code: "NON_FINITE_EMBEDDING",
                    message: format!("embedding[{idx}] is non-finite"),
                });
            }
            emb_f32.push(f);
        }

        // 5. Strictly increasing frame_id check within producer_epoch
        if let Some(last_fid) = inner.last_frame_id {
            if req.frame_id <= last_fid {
                return Err(EmbeddingIngestError {
                    status: 409,
                    code: "NON_MONOTONIC_FRAME_ID",
                    message: format!(
                        "frame_id {} is not strictly greater than last frame_id {last_fid} in epoch {}",
                        req.frame_id, req.producer_epoch
                    ),
                });
            }
        }

        // 6. Bilateral timestamp check (before any window mutation!)
        let mapped_i128 = (req.captured_at_ms as i128) + (hs.clock_offset_ms as i128);
        let now_i128 = now_ms as i128;
        let future_slack = hs
            .effective_uncertainty_ms()
            .min(MAX_FUTURE_SKEW_CAP_MS) as i128;
        if mapped_i128 > now_i128 + future_slack {
            return Err(EmbeddingIngestError {
                status: 400,
                code: "FUTURE_FRAME",
                message: format!(
                    "mapped capture timestamp {mapped_i128} ms exceeds now ({now_i128} ms) + uncertainty slack ({future_slack} ms)"
                ),
            });
        }
        let staleness_budget = inner
            .bound
            .as_ref()
            .map_or(MAX_VISION_STALENESS_MS, |b| b.effective_staleness_ms);
        if mapped_i128 < 0 || (now_i128 - mapped_i128) > (staleness_budget as i128) {
            return Err(EmbeddingIngestError {
                status: 400,
                code: "STALE_FRAME",
                message: format!(
                    "frame age {} ms exceeds staleness budget {staleness_budget} ms",
                    now_i128 - mapped_i128
                ),
            });
        }
        let mapped_ms = mapped_i128 as u64;

        // Monotonic mapped_at_ms check (A2): prevent interior older timestamps from evading max_window_span_ms.
        if let Some(last_frame) = inner.frames.back() {
            if mapped_ms < last_frame.mapped_at_ms {
                return Err(EmbeddingIngestError {
                    status: 400,
                    code: "NON_MONOTONIC_TIMESTAMP",
                    message: format!(
                        "mapped capture timestamp {mapped_ms} ms is older than previous frame's {} ms",
                        last_frame.mapped_at_ms
                    ),
                });
            }
        }

        let history_len = inner.bound.as_ref().map_or(UNBOUND_RING_CAP, |b| b.spec.history_len);
        let span_limit = inner
            .bound
            .as_ref()
            .map_or(MAX_VIDEO_WINDOW_SPAN_MS, |b| b.spec.max_window_span_ms);
        if history_len == 1 && now_ms.saturating_sub(mapped_ms) > span_limit {
            return Err(EmbeddingIngestError {
                status: 400,
                code: "STALE_FRAME",
                message: format!(
                    "frame age {} ms exceeds max_window_span_ms {span_limit} ms",
                    now_ms.saturating_sub(mapped_ms)
                ),
            });
        }

        // 7. All validations passed — update ring buffer and deadlines atomically.
        // If `model_generation` changed within the epoch (A-R2-4), clear older-generation frames
        // before pushing so old and new model_generation frames can never mix in the same window.
        if !inner.has_seen_frame_in_epoch {
            inner.has_seen_frame_in_epoch = true;
            inner.last_frame_model_generation = req.model_generation;
            if inner.model_generation.is_none() {
                inner.model_generation = req.model_generation;
            }
        } else if inner.last_frame_model_generation != req.model_generation {
            inner.frames.clear();
            inner.last_frame_model_generation = req.model_generation;
            inner.model_generation = Some(inner.model_generation.unwrap_or(0).saturating_add(1));
        }

        while let Some(front) = inner.frames.front() {
            if now_ms.saturating_sub(front.mapped_at_ms) > span_limit {
                inner.frames.pop_front();
            } else {
                break;
            }
        }
        while inner.frames.len() >= history_len {
            inner.frames.pop_front();
        }
        inner.frames.push_back(EmbeddingFrame {
            frame_id: req.frame_id,
            producer_epoch: req.producer_epoch,
            handshake_generation: hs.handshake_generation,
            captured_at_ms: req.captured_at_ms,
            mapped_at_ms: mapped_ms,
            embedding: emb_f32,
            encoder_artifact_sha256: enc_sha,
            preprocess_config_sha256: prep_sha,
            model_generation: req.model_generation,
        });
        inner.last_frame_id = Some(req.frame_id);
        inner.recompute_deadlines();

        let window_len = inner.frames.len();
        let target_len = inner.bound.as_ref().map_or(1, |b| b.spec.history_len);
        let (newest_dl, oldest_dl) = inner
            .window_newest_deadline_ms
            .zip(inner.window_oldest_deadline_ms)
            .expect("non-empty window has deadlines");

        Ok(EmbeddingIngestResponse {
            accepted: true,
            frame_id: req.frame_id,
            producer_epoch: req.producer_epoch,
            mapped_captured_at_ms: mapped_ms,
            window_len,
            history_len: target_len,
            window_ready: window_len >= target_len,
            window_newest_deadline_ms: newest_dl,
            window_oldest_deadline_ms: oldest_dl,
        })
    }

    pub fn sample_window(
        &self,
        spec: &VideoEmbeddingSpec,
        run_started_ms: u64,
    ) -> EmbeddingSampleDecision {
        let inner = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let now_ms = inner.now_ms();
        let eff_staleness = inner
            .bound
            .as_ref()
            .map_or_else(|| spec.effective_staleness_ms(), |b| b.effective_staleness_ms);

        let Some(ref hs) = inner.handshake else {
            if now_ms.saturating_sub(run_started_ms) <= eff_staleness {
                return EmbeddingSampleDecision::Hold;
            }
            return EmbeddingSampleDecision::Stop {
                reason: "HANDSHAKE_REQUIRED",
                message: "No clock handshake completed for video_embedding policy".into(),
            };
        };

        if now_ms > hs.handshake_expires_at_ms {
            return EmbeddingSampleDecision::Stop {
                reason: "HANDSHAKE_EXPIRED",
                message: format!(
                    "Clock handshake {} expired at {} ms (now {} ms)",
                    hs.handshake_generation, hs.handshake_expires_at_ms, now_ms
                ),
            };
        }

        if inner.frames.len() < spec.history_len {
            if now_ms.saturating_sub(run_started_ms) <= spec.max_window_span_ms {
                return EmbeddingSampleDecision::Hold;
            }
            return match spec.on_stale {
                VisionStalePolicy::Hold => EmbeddingSampleDecision::Hold,
                VisionStalePolicy::Stop => EmbeddingSampleDecision::Stop {
                    reason: "VISION_STALE",
                    message: format!(
                        "Embedding window incomplete ({}/{}) within {} ms",
                        inner.frames.len(),
                        spec.history_len,
                        spec.max_window_span_ms
                    ),
                },
            };
        }

        // Take the newest T frames in chronological order (oldest..newest)
        let skip = inner.frames.len() - spec.history_len;
        let window_slice: Vec<&EmbeddingFrame> = inner.frames.iter().skip(skip).collect();

        // Ensure all frames in the window belong to the current handshake_generation,
        // producer_epoch, and consistent frame model_generation (A7/A8/A-R2-4).
        let expected_frame_model_gen = inner.last_frame_model_generation;
        if window_slice.iter().any(|f| {
            f.handshake_generation != hs.handshake_generation
                || f.producer_epoch != hs.producer_epoch
                || (inner.has_seen_frame_in_epoch && f.model_generation != expected_frame_model_gen)
        }) {
            return EmbeddingSampleDecision::Hold;
        }

        let oldest_mapped = window_slice
            .iter()
            .map(|f| f.mapped_at_ms)
            .min()
            .expect("history_len >= 1");
        let newest_frame = window_slice.last().expect("history_len >= 1");
        let newest_mapped = window_slice
            .iter()
            .map(|f| f.mapped_at_ms)
            .max()
            .expect("history_len >= 1");

        let newest_dl = newest_mapped.saturating_add(eff_staleness);
        let oldest_dl = oldest_mapped.saturating_add(spec.max_window_span_ms);

        if now_ms > newest_dl || now_ms > oldest_dl {
            return match spec.on_stale {
                VisionStalePolicy::Hold => EmbeddingSampleDecision::Hold,
                VisionStalePolicy::Stop => EmbeddingSampleDecision::Stop {
                    reason: "VISION_STALE",
                    message: format!(
                        "Embedding window expired (now={now_ms} ms, newest_deadline={newest_dl} ms, oldest_deadline={oldest_dl} ms)"
                    ),
                },
            };
        }

        let features = match extract_embedding_window_features(&window_slice, spec) {
            Ok(f) => f,
            Err(msg) => {
                return EmbeddingSampleDecision::Stop {
                    reason: "VISION_INVALID",
                    message: msg,
                };
            }
        };

        EmbeddingSampleDecision::Ready(EmbeddingWindowSnapshot {
            features,
            newest_frame_id: newest_frame.frame_id,
            newest_mapped_at_ms: newest_frame.mapped_at_ms,
            producer_epoch: newest_frame.producer_epoch,
            model_generation: inner.model_generation,
            invalidation_generation: inner.invalidation_generation,
            clock_handshake_generation: hs.handshake_generation,
            clock_handshake_expires_at_ms: hs.handshake_expires_at_ms,
            window_newest_deadline_ms: newest_dl,
            window_oldest_deadline_ms: oldest_dl,
            policy_instance_id: inner.bound.as_ref().map(|b| b.policy_instance_id),
        })
    }

    pub fn live_ticket_state(&self) -> LiveEmbeddingTicketState {
        let inner = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        inner.live_ticket_state()
    }
}

pub fn extract_embedding_window_features(
    frames: &[&EmbeddingFrame],
    spec: &VideoEmbeddingSpec,
) -> Result<Vec<f32>, String> {
    if frames.len() != spec.history_len {
        return Err(format!(
            "expected {} embedding frames, got {}",
            spec.history_len,
            frames.len()
        ));
    }
    let d = spec.embedding_dim;
    let mut out = Vec::with_capacity(d * spec.history_len);
    for frame in frames {
        if frame.embedding.len() != d {
            return Err(format!(
                "embedding frame {} has len {}, expected {d}",
                frame.frame_id,
                frame.embedding.len()
            ));
        }
        if frame.encoder_artifact_sha256 != spec.encoder_artifact_sha256
            || frame.preprocess_config_sha256 != spec.preprocess_config_sha256
        {
            return Err(format!(
                "embedding frame {} artifact SHA-256 mismatch",
                frame.frame_id
            ));
        }
        if let Some(ref norm) = spec.embedding_norm {
            for i in 0..d {
                let v = (frame.embedding[i] - norm.mean[i]) / norm.std[i];
                if !v.is_finite() {
                    return Err(format!(
                        "normalized embedding[{i}] on frame {} is non-finite",
                        frame.frame_id
                    ));
                }
                out.push(v);
            }
        } else {
            for (i, &v) in frame.embedding.iter().enumerate() {
                if !v.is_finite() {
                    return Err(format!(
                        "embedding[{i}] on frame {} is non-finite",
                        frame.frame_id
                    ));
                }
                out.push(v);
            }
        }
    }
    Ok(out)
}

static SUPERVISOR_EMBEDDING_HUBS: OnceLock<
    Mutex<HashMap<usize, (Weak<ArmSupervisor>, Arc<VideoEmbeddingHub>)>>,
> = OnceLock::new();

pub fn embedding_hub_for_supervisor(supervisor: &Arc<ArmSupervisor>) -> Arc<VideoEmbeddingHub> {
    let registry = SUPERVISOR_EMBEDDING_HUBS.get_or_init(|| Mutex::new(HashMap::new()));
    let mut map = registry.lock().unwrap_or_else(|e| e.into_inner());
    map.retain(|_, (weak, _)| weak.strong_count() > 0);
    let key = Arc::as_ptr(supervisor) as usize;
    let weak_target = Arc::downgrade(supervisor);
    if let Some((existing_weak, hub)) = map.get(&key) {
        if Weak::ptr_eq(existing_weak, &weak_target) {
            return hub.clone();
        }
    }
    let hub = Arc::new(VideoEmbeddingHub::new());
    map.insert(key, (weak_target, hub.clone()));
    hub
}
