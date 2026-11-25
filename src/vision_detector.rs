//! Detection kind switch (`POST /api/v1/vision/detector`, operator only): the cockpit's simple
//! mode chooses between boxes (`detect`) and outlines (`segment`), each served by one default
//! detector model of this armd. Any other model is chosen elsewhere (grasp task `models`).
//!
//! The kind maps to the first name of [`DetectorDefaults`] (profile `vision_detectors`, else the
//! built-in names of the rebot-vision catalogue) that the provider's catalogue (`GET /api/mode`
//! `models.catalogue`) lists; armd then loads it the way a grasp task does (`POST /api/models`,
//! then `/api/mode` polled until the swap is done). Grasps, measurements and other switches use
//! the same detector, so a switch is refused while one of them runs.
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

use crate::grasp::DetectionSource;
use crate::supervisor::{ArmError, ArmSupervisor};

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
