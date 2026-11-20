//! Asynchronous vision feature collection, strict staleness/generation/sync-skew
//! validation, and `top1_box_norm_v1` feature extraction for `policy_runtime`.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::grasp::{BoxFuture, Detection, DetectionSource};

use super::manifest::{VisionFeatureSpec, VisionStalePolicy, VisionTargetLostPolicy};

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
