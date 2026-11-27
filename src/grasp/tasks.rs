use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::supervisor::ArmError;

use super::{f, refusal_message, DetectionSource, ModeInfo, PlannedGrasp, ResidentModels};

/// A named grasp task (`GraspSetup.tasks`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GraspTask {
    /// Grasp setup fields applied on top of the setup (before the request's `overrides`).
    #[serde(default)]
    pub overrides: serde_json::Map<String, Value>,
    /// What the vision provider must offer; checked before the grasp begins.
    #[serde(default, skip_serializing_if = "TaskRequires::is_empty")]
    pub requires: TaskRequires,
    /// Models the provider loads for the task (`POST /api/models` before the grasp begins).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub models: Option<TaskModels>,
    /// Free text, at most 200 bytes.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
}

/// Perception requirements of a grasp task, compared with the provider's `GET /api/mode`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct TaskRequires {
    /// `model.kind` of the loaded YOLO model (`detect` | `segment`); exact match.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub model_kind: Option<ModelKind>,
    /// Depth pipeline: `any` (`available.depth`) or `metric` (also `depth_model.metric`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub depth: Option<DepthReq>,
}

/// The provider's model slots a task sets (names of the provider catalogue, `GET /api/mode`
/// `models.catalogue`). Per slot: absent = keep, `null` = unload, a name = load.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TaskModels {
    #[serde(default, deserialize_with = "some_or_null", skip_serializing_if = "Option::is_none")]
    pub detector: Option<Option<String>>,
    #[serde(default, deserialize_with = "some_or_null", skip_serializing_if = "Option::is_none")]
    pub depth: Option<Option<String>>,
}

/// A present field: `null` -> `Some(None)` (an absent one stays `None` through `default`).
fn some_or_null<'de, D: serde::Deserializer<'de>>(d: D) -> Result<Option<Option<String>>, D::Error> {
    Option::<String>::deserialize(d).map(Some)
}

impl TaskModels {
    fn slots(&self) -> [(&'static str, &Option<Option<String>>); 2] { [("detector", &self.detector), ("depth", &self.depth)] }

    pub(super) fn validate(&self) -> Result<(), String> {
        if self.detector.is_none() && self.depth.is_none() { return Err("models needs detector or depth".into()); }
        for (slot, v) in self.slots() {
            if let Some(Some(name)) = v {
                if name.is_empty() || name.len() > 64 { return Err(format!("models.{slot} must be null or a name of 1..64 bytes")); }
            }
        }
        Ok(())
    }

    /// The resident set after the switch (the provider's `resident`, with the slots this names).
    fn after(&self, resident: &ResidentModels) -> ResidentModels {
        ResidentModels {
            detector: self.detector.clone().unwrap_or_else(|| resident.detector.clone()),
            depth: self.depth.clone().unwrap_or_else(|| resident.depth.clone()),
        }
    }

    /// `POST /api/models` body when the provider's resident set differs (every slot this names;
    /// a provider without `models` in its mode always differs).
    pub fn switch_body(&self, mode: &ModeInfo) -> Option<Value> {
        if mode.models.as_ref().is_some_and(|m| self.after(&m.resident) == m.resident) { return None; }
        let mut body = serde_json::Map::new();
        for (slot, v) in self.slots() {
            if let Some(name) = v { body.insert(slot.into(), serde_json::json!(name)); }
        }
        Some(Value::Object(body))
    }
}

/// `requires.model_kind`, compared exactly with the provider's `model.kind` (rebot-vision:
/// `detect`, `segment` for an instance segmentation model, `obb` for an oriented box model).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ModelKind { Detect, Segment, Obb }

impl ModelKind {
    pub fn name(self) -> &'static str {
        match self { Self::Detect => "detect", Self::Segment => "segment", Self::Obb => "obb" }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DepthReq { Any, Metric }

impl TaskRequires {
    pub fn is_empty(&self) -> bool { self.model_kind.is_none() && self.depth.is_none() }

    /// `Err(reason)` when the provider (`mode`) cannot serve the requirements of task `task`.
    pub fn check(&self, task: &str, mode: &ModeInfo) -> Result<(), String> {
        if let Some(kind) = self.model_kind {
            if mode.reported && !mode.yolo_available {
                return Err(format!("task {task} needs model_kind {}, provider has no detector loaded", kind.name()));
            }
            match mode.model_kind.as_deref().filter(|_| mode.reported) {
                None => return Err(format!("task {task} needs model_kind {}, provider does not report model.kind", kind.name())),
                Some(have) if have != kind.name() => return Err(format!("task {task} needs model_kind {}, provider has {have}", kind.name())),
                Some(_) => {}
            }
        }
        if let Some(depth) = self.depth {
            let want = match depth { DepthReq::Any => "any", DepthReq::Metric => "metric" };
            if !mode.reported {
                return Err(format!("task {task} needs depth {want}, provider does not report depth"));
            }
            if !mode.depth_available {
                return Err(format!("task {task} needs depth {want}, provider has no depth pipeline"));
            }
            if depth == DepthReq::Metric && !mode.depth_metric {
                return Err(format!("task {task} needs depth metric, provider depth model is not metric"));
            }
        }
        Ok(())
    }
}

/// Task names: 1..=32 of `[a-z0-9_-]`.
pub(super) fn valid_task_name(name: &str) -> bool {
    (1..=32).contains(&name.len()) && name.bytes().all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'_' || b == b'-')
}


/// Code of a grasp rejected because the provider cannot serve the task's requirements.
pub const TASK_UNSUPPORTED: &str = "TASK_UNSUPPORTED";

fn task_unsupported(message: String) -> ArmError {
    ArmError::Rejected { status: 422, code: TASK_UNSUPPORTED.to_string(), message }
}

/// How long a preflight waits for the provider's swap (`/api/mode` polled).
const MODEL_SWITCH_TIMEOUT: Duration = Duration::from_secs(5);

/// Task check of a planned grasp, outside the supervisor lock and before the action begins.
/// Without requirements and models nothing is read. Otherwise one mode reading, kept in
/// `planned.mode` for the orchestrator; with `models` and a different resident set the provider
/// switches models first (serialized, [`switch_models`]) and the post-switch reading is kept. A
/// provider that cannot be read is not a task failure (the grasp then fails as without a task).
pub async fn preflight(source: &dyn DetectionSource, planned: &mut PlannedGrasp) -> Result<(), ArmError> {
    let request = &planned.request;
    let has_task_reqs = request.task.as_deref().is_some_and(|_| !request.requires.is_empty() || request.models.is_some());
    if has_task_reqs {
        let task = request.task.as_deref().unwrap();
        let _serial = if request.models.is_some() {
            Some(crate::vision_detector::begin_grasp_model_switch(source).await?)
        } else {
            crate::vision_detector::reconcile_pending_swap(source).await?;
            None
        };
        let mut mode = source.mode_info().await;
        if let (Some(models), Ok(m)) = (&request.models, &mode) {
            if let Some(body) = models.switch_body(m) {
                mode = switch_models(source, &body, task).await?;
            }
        }
        let verdict = match &mode {
            Ok(m) => request.requires.check(task, m),
            Err(e) if planned.adaptive.is_some() => {
                return Err(ArmError::Detailed {
                    status: 503,
                    code: "DETECTION_UNAVAILABLE".into(),
                    message: format!("[vision_preflight] vision provider unreachable: {e}"),
                    details: Box::new(serde_json::json!({
                        "failing_phase": "vision_preflight",
                        "outcome": "infeasible",
                    })),
                });
            }
            Err(_) => Ok(()),
        };
        planned.mode = Some(mode);
        verdict.map_err(|why| {
            if planned.adaptive.is_some() {
                ArmError::Detailed {
                    status: 422,
                    code: TASK_UNSUPPORTED.to_string(),
                    message: format!("[task] {why}"),
                    details: Box::new(serde_json::json!({
                        "failing_phase": "task",
                        "outcome": "infeasible",
                    })),
                }
            } else {
                task_unsupported(why)
            }
        })?;
    } else if planned.adaptive.is_some() && planned.mode.is_none() {
        crate::vision_detector::reconcile_pending_swap(source).await?;
        // Bind vision provider model generation before BeginMacro so a model switch between
        // planning/preflight and live revalidation is rejected, and fail closed if the provider
        // is unreachable before starting motion.
        let mode = source.mode_info().await;
        if let Err(ref e) = mode {
            return Err(ArmError::Detailed {
                status: 503,
                code: "DETECTION_UNAVAILABLE".into(),
                message: format!("[vision_preflight] vision provider unreachable: {e}"),
                details: Box::new(serde_json::json!({
                    "failing_phase": "vision_preflight",
                    "outcome": "infeasible",
                })),
            });
        }
        planned.mode = Some(mode);
    } else {
        crate::vision_detector::reconcile_pending_swap(source).await?;
    }
    if let Some(pa) = planned.adaptive.as_mut() {
        pa.expected_generation = planned
            .mode
            .as_ref()
            .and_then(|r| r.as_ref().ok())
            .and_then(|m| m.models.as_ref())
            .map(|pm| pm.generation);
    }
    Ok(())
}

/// `POST /api/models`, then `/api/mode` polled until the provider published the returned
/// generation and its swap is idle (or failed). Refusals are TASK_UNSUPPORTED; an unreachable
/// provider or a swap that does not finish in time is the returned `Err` reading.
async fn switch_models(source: &dyn DetectionSource, body: &Value, task: &str) -> Result<Result<ModeInfo, String>, ArmError> {
    let refused = |why: String| task_unsupported(format!("task {task}: {why}"));
    let mut attempt = crate::vision_detector::SwapAttemptGuard::begin();
    let (status, reply) = match source.set_models(body).await {
        Ok(r) => r,
        Err(e) => return Ok(Err(format!("model switch failed: {e}"))),
    };
    let error = reply.get("error").and_then(Value::as_str).unwrap_or("refused");
    match status {
        200..=299 => {}
        404 => {
            attempt.clear_definitive_refusal();
            return Err(refused("provider cannot switch models".into()));
        }
        409 if error == "SWAP_BUSY" => return Err(refused("model switch busy".into())),
        400..=499 => {
            attempt.clear_definitive_refusal();
            return Err(refused(format!("model switch refused: {}", refusal_message(error, f(&reply, &["need_mb"]), f(&reply, &["limit_mb"])))));
        }
        _ => return Ok(Err(format!("model switch failed: detector returned {status}"))),
    }
    let Some(generation) = reply.get("generation").and_then(Value::as_u64) else {
        return Ok(Err("model switch reply has no generation".into()));
    };
    attempt.set_generation(generation);
    let deadline = Instant::now() + MODEL_SWITCH_TIMEOUT;
    loop {
        let m = match source.mode_info().await {
            Ok(m) => m,
            Err(e) => return Ok(Err(e)),
        };
        if let Some(pm) = m.models.as_ref().filter(|pm| pm.generation >= generation && matches!(pm.swap_state.as_str(), "idle" | "failed")) {
            attempt.confirm_settled();
            if pm.swap_state == "failed" {
                return Err(refused(format!("model switch failed: {}", pm.swap_error.as_deref().unwrap_or("unknown error"))));
            }
            return Ok(Ok(m));
        }
        if Instant::now() >= deadline {
            return Ok(Err(format!("model switch did not finish within {} s", MODEL_SWITCH_TIMEOUT.as_secs())));
        }
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
}

/// The dry-run task verdict on one mode reading (no switch): a budget refusal computed from the
/// provider's catalogue, else `needs model switch` (supported) when the resident set differs,
/// else the requirement check on the current set.
pub(super) fn task_verdict(task: &str, requires: &TaskRequires, models: Option<&TaskModels>, mode: &ModeInfo) -> (bool, Option<String>) {
    if let Some(want) = models.filter(|w| w.switch_body(mode).is_some()) {
        let budget = mode.models.as_ref().and_then(|pm| pm.budget(&want.after(&pm.resident)));
        return match budget {
            Some(Err(why)) => (false, Some(format!("task {task}: model switch refused: {why}"))),
            _ => (true, Some("needs model switch".into())),
        };
    }
    match requires.check(task, mode) { Ok(()) => (true, None), Err(r) => (false, Some(r)) }
}

