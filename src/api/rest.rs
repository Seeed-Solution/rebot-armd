use std::path::PathBuf;
use std::sync::Arc;
use axum::{
    extract::State,
    http::StatusCode,
    response::{IntoResponse, Response},
    Json,
};
use serde::{Deserialize, Serialize};

use crate::model::{ArmObservation, ObservationSchema};
use crate::safety::SafetyState;
use crate::supervisor::{ArmError, ArmSupervisor};

#[derive(Debug, Serialize, Deserialize)]
pub struct TorqueRequest {
    pub enable: bool,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct TorqueResponse {
    pub is_torqued: bool,
    pub safety_state: SafetyState,
    pub epoch_id: u64,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct JogRequest {
    pub target_positions: Vec<f32>,
    pub duration_secs: Option<f64>,
    pub epoch_id: Option<u64>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct HomeRequest {
    pub duration_secs: Option<f64>,
    pub epoch_id: Option<u64>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct HeartbeatRequest {
    pub client_id: String,
    pub epoch_id: u64,
    #[serde(default)]
    pub timestamp_ms: Option<u64>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct HeartbeatResponse {
    pub status: String,
    pub epoch_id: u64,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ServoRequest {
    pub targets: Vec<f32>,
    pub epoch_id: Option<u64>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct MotionResponse {
    pub status: String,
    pub epoch_id: u64,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct SafetyResponse {
    pub safety_state: SafetyState,
    pub epoch_id: u64,
}

#[derive(Debug, Serialize, Deserialize, Default)]
pub struct StartRecordRequest {
    pub episode_id: Option<String>,
    pub output_path: Option<String>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct RecordActionResponse {
    pub status: String,
    pub path: Option<String>,
    pub frame_count: Option<usize>,
    /// Id of the episode (file stem in the episode directory).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub episode_id: Option<String>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ApiErrorResponse {
    pub error: String,
    pub message: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub safety_state: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub code: Option<String>,
    /// Extra fields of a detailed rejection (`owner`, `sequences`, `step`, `reason`).
    #[serde(flatten, skip_serializing_if = "Option::is_none")]
    pub details: Option<serde_json::Map<String, serde_json::Value>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub constraint_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub geometry_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub violation_phase: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub current_revision: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub retry_after_s: Option<u64>,
}

#[derive(Debug)]
pub struct ApiError {
    pub status: StatusCode,
    pub error: String,
    pub message: String,
    pub code: Option<String>,
    pub safety_state: Option<String>,
    /// Extra reply fields (an object) of [`ArmError::Detailed`].
    pub details: Option<Box<serde_json::Value>>,
    pub workspace_violation: Option<Box<crate::kinematics::WorkspaceViolationDetail>>,
    pub current_revision: Option<u64>,
    pub retry_after_s: Option<u64>,
}

#[allow(non_snake_case)]
pub fn ApiError(status: StatusCode, message: String, code: String) -> ApiError {
    ApiError {
        status,
        error: code.clone(),
        message,
        code: Some(code),
        safety_state: None,
        details: None,
        workspace_violation: None,
        current_revision: None,
        retry_after_s: None,
    }
}

impl ApiError {
    pub fn detailed(status: StatusCode, error: impl Into<String>, code: impl Into<String>, message: impl Into<String>, current_revision: Option<u64>) -> Self {
        Self {
            status,
            error: error.into(),
            message: message.into(),
            code: Some(code.into()),
            safety_state: None,
            workspace_violation: None,
            details: None,
            current_revision,
            retry_after_s: None,
        }
    }

    pub fn operator_required(op: &str) -> Self {
        Self::detailed(
            StatusCode::FORBIDDEN,
            "operator_required",
            "OPERATOR_REQUIRED",
            format!("Operator unlock is required for {op}"),
            None,
        )
    }

    pub fn invalid_pin() -> Self {
        Self::detailed(
            StatusCode::FORBIDDEN,
            "invalid_pin",
            "INVALID_PIN",
            "Invalid operator PIN",
            None,
        )
    }

    pub fn rate_limited(retry_after_s: u64) -> Self {
        let mut err = Self::detailed(
            StatusCode::TOO_MANY_REQUESTS,
            "rate_limited",
            "RATE_LIMITED",
            format!("Too many failed unlock attempts; retry after {retry_after_s}s"),
            None,
        );
        err.retry_after_s = Some(retry_after_s);
        err
    }

    pub fn policy_revision_mismatch(expected: u64, current: u64) -> Self {
        Self::detailed(
            StatusCode::CONFLICT,
            "policy_revision_mismatch",
            "POLICY_REVISION_MISMATCH",
            format!("Policy revision mismatch: expected {expected}, current {current}"),
            Some(current),
        )
    }

    pub fn persist_failed(message: impl Into<String>) -> Self {
        Self::detailed(
            StatusCode::INTERNAL_SERVER_ERROR,
            "persist_failed",
            "PERSIST_FAILED",
            message,
            None,
        )
    }
}

impl IntoResponse for ApiError {
    fn into_response(self) -> Response {
        let (constraint_id, geometry_id, violation_phase, reason) = match self.workspace_violation {
            Some(d) => (d.constraint_id, d.geometry_id, d.violation_phase, d.reason),
            None => (None, None, None, None),
        };
        let retry_after_s = self.retry_after_s;
        let body = Json(ApiErrorResponse {
            error: self.error,
            message: self.message,
            safety_state: self.safety_state,
            code: self.code,
            details: match self.details.map(|d| *d) { Some(serde_json::Value::Object(m)) => Some(m), _ => None },
            constraint_id,
            geometry_id,
            violation_phase,
            reason,
            current_revision: self.current_revision,
            retry_after_s,
        });
        let mut resp = (self.status, body).into_response();
        if let Some(s) = retry_after_s {
            if let Ok(hv) = axum::http::HeaderValue::from_str(&s.to_string()) {
                resp.headers_mut().insert(axum::http::header::RETRY_AFTER, hv);
            }
        }
        resp
    }
}

impl From<ArmError> for ApiError {
    fn from(err: ArmError) -> Self {
        match err {
            ArmError::Safety(safety_err) => match safety_err {
                crate::safety::SafetyError::HandoverSettling => ApiError {
                    status: StatusCode::CONFLICT,
                    error: "HANDOVER_SETTLING".to_string(),
                    message: safety_err.to_string(),
                    code: Some("HANDOVER_SETTLING".to_string()),
                    safety_state: None,
                    details: None,
                    workspace_violation: None,
                    current_revision: None,
                    retry_after_s: None,
                },
                crate::safety::SafetyError::ArmNotAtRestingPose { ref reason, .. } => ApiError {
                    status: StatusCode::CONFLICT,
                    error: "ArmNotAtRestingPose".to_string(),
                    message: safety_err.to_string(),
                    code: Some("ARM_NOT_AT_RESTING_POSE".to_string()),
                    safety_state: Some("Armed".to_string()),
                    details: reason.as_ref().map(|r| Box::new(serde_json::json!({ "reason": r }))),
                    workspace_violation: None,
                    current_revision: None,
                    retry_after_s: None,
                },
                crate::safety::SafetyError::MotionForbiddenInFaultState(msg) => ApiError {
                    status: StatusCode::CONFLICT,
                    error: "MotionForbiddenInFaultState".to_string(),
                    message: msg,
                    code: Some("MOTION_FORBIDDEN_IN_FAULT_STATE".to_string()),
                    safety_state: Some("Fault".to_string()),
                    details: None,
                    workspace_violation: None,
                    current_revision: None,
                    retry_after_s: None,
                },
                crate::safety::SafetyError::TorqueOffWouldDrop {
                    ref joint,
                    position,
                    rest,
                    ref reason,
                    ref message,
                } => ApiError {
                    status: StatusCode::CONFLICT,
                    error: "TORQUE_OFF_WOULD_DROP".to_string(),
                    message: message.clone(),
                    code: Some("TORQUE_OFF_WOULD_DROP".to_string()),
                    safety_state: None,
                    details: match (joint, position, rest) {
                        (Some(j), Some(pos), Some(r)) => Some(Box::new(serde_json::json!({
                            "reason": reason.as_deref().unwrap_or("outside_rest_pose"),
                            "joint": j,
                            "position": pos,
                            "rest": r,
                        }))),
                        _ => reason
                            .as_deref()
                            .map(|r| Box::new(serde_json::json!({ "reason": r }))),
                    },
                    workspace_violation: None,
                    current_revision: None,
                    retry_after_s: None,
                },
                crate::safety::SafetyError::ClearFaultRejected(msg) => ApiError {
                    status: StatusCode::CONFLICT,
                    error: "ClearFaultRejected".to_string(),
                    message: msg,
                    code: Some("CLEAR_FAULT_REJECTED".to_string()),
                    safety_state: Some("Fault".to_string()),
                    details: None,
                    workspace_violation: None,
                    current_revision: None,
                    retry_after_s: None,
                },
                crate::safety::SafetyError::EStopActive => ApiError {
                    status: StatusCode::CONFLICT,
                    error: "ESTOP_ACTIVE".to_string(),
                    message: safety_err.to_string(),
                    code: Some("ESTOP_ACTIVE".to_string()),
                    safety_state: Some("EStop".to_string()),
                    details: None,
                    workspace_violation: None,
                    current_revision: None,
                    retry_after_s: None,
                },
                crate::safety::SafetyError::CollisionHold(_) => {
                    ApiError(StatusCode::CONFLICT, safety_err.to_string(), "COLLISION_HOLD".to_string())
                }
                crate::safety::SafetyError::CommissioningRequired(_) => {
                    ApiError(StatusCode::CONFLICT, safety_err.to_string(), "COMMISSIONING_REQUIRED".to_string())
                }
                crate::safety::SafetyError::MotionBusy => {
                    ApiError(StatusCode::CONFLICT, safety_err.to_string(), "MOTION_BUSY".to_string())
                }
                crate::safety::SafetyError::WorkspaceViolation { ref detail, .. } => {
                    let mut api = ApiError(StatusCode::UNPROCESSABLE_ENTITY, safety_err.to_string(), "WORKSPACE_VIOLATION".to_string());
                    api.workspace_violation = Some(detail.clone());
                    api
                }
                crate::safety::SafetyError::EpochMismatch { .. } => {
                    ApiError(StatusCode::CONFLICT, safety_err.to_string(), "EPOCH_MISMATCH".to_string())
                }
                crate::safety::SafetyError::SoftLimitExceeded { .. } => {
                    ApiError(StatusCode::BAD_REQUEST, safety_err.to_string(), "SOFT_LIMIT_EXCEEDED".to_string())
                }
                crate::safety::SafetyError::FaultActive(_) => ApiError {
                    status: StatusCode::CONFLICT,
                    error: "FAULT_ACTIVE".to_string(),
                    message: safety_err.to_string(),
                    code: Some("FAULT_ACTIVE".to_string()),
                    safety_state: Some("Fault".to_string()),
                    details: None,
                    workspace_violation: None,
                    current_revision: None,
                    retry_after_s: None,
                },
                crate::safety::SafetyError::NotArmed(_) => {
                    ApiError(StatusCode::PRECONDITION_FAILED, safety_err.to_string(), "NOT_ARMED".to_string())
                }
                crate::safety::SafetyError::OwnerTransferred => {
                    ApiError(StatusCode::SERVICE_UNAVAILABLE, safety_err.to_string(), "OWNER_TRANSFERRED".to_string())
                }
                crate::safety::SafetyError::Stopped => {
                    ApiError(StatusCode::CONFLICT, safety_err.to_string(), "STOPPED".to_string())
                }
                crate::safety::SafetyError::LeaseLost => {
                    ApiError(StatusCode::CONFLICT, safety_err.to_string(), "LEASE_LOST".to_string())
                }
                crate::safety::SafetyError::TerminalsBusy => {
                    ApiError(StatusCode::SERVICE_UNAVAILABLE, safety_err.to_string(), "TERMINALS_BUSY".to_string())
                }
                crate::safety::SafetyError::StampAlreadyConsumed => {
                    ApiError(StatusCode::CONFLICT, safety_err.to_string(), "STAMP_REUSED".to_string())
                }
                _ => ApiError(StatusCode::BAD_REQUEST, safety_err.to_string(), "SAFETY_ERROR".to_string()),
            },
            ArmError::Trajectory(traj_err) => {
                ApiError(StatusCode::BAD_REQUEST, traj_err.to_string(), "TRAJECTORY_ERROR".to_string())
            }
            ArmError::Recording(rec_err) => {
                ApiError(StatusCode::BAD_REQUEST, rec_err.to_string(), "RECORDING_ERROR".to_string())
            }
            ArmError::Motion(motion_err) => {
                use crate::motion::MotionError as M;
                let status = match motion_err {
                    M::MotionBusy(_) | M::PlanStale(_) => StatusCode::CONFLICT,
                    M::InvalidCommand(_) => StatusCode::BAD_REQUEST,
                    _ => StatusCode::UNPROCESSABLE_ENTITY,
                };
                let mut api = ApiError(status, motion_err.to_string(), motion_err.code().to_string());
                if let M::WorkspaceViolation { detail, .. } = motion_err {
                    api.workspace_violation = Some(detail);
                }
                api
            }
            ArmError::Unsupported(msg) => {
                ApiError(StatusCode::UNPROCESSABLE_ENTITY, msg, "UNSUPPORTED".to_string())
            }
            ArmError::Rejected { status, code, message } => {
                ApiError(StatusCode::from_u16(status).unwrap_or(StatusCode::CONFLICT), message, code)
            }
            ArmError::Detailed { status, code, message, details } => {
                let mut e = ApiError(StatusCode::from_u16(status).unwrap_or(StatusCode::CONFLICT), message, code);
                e.details = Some(details);
                e
            }
            ArmError::WorkspaceRejected { message, detail } => {
                let mut api = ApiError(StatusCode::UNPROCESSABLE_ENTITY, message, "WORKSPACE_VIOLATION".to_string());
                api.workspace_violation = Some(detail);
                api
            }
            ArmError::PolicyRevisionMismatch { expected, current } => {
                ApiError::policy_revision_mismatch(expected, current)
            }
            ArmError::Internal(msg) => {
                ApiError(StatusCode::INTERNAL_SERVER_ERROR, msg, "INTERNAL_ERROR".to_string())
            }
        }
    }
}

/// GET /api/v1/observation
pub async fn get_observation(
    State(supervisor): State<Arc<ArmSupervisor>>,
) -> Result<Json<ArmObservation>, ApiError> {
    Ok(Json(supervisor.observation().await?))
}

/// GET /api/v1/observation/schema
/// `capabilities.vision` carries the provider's modes and current mode when it reports them.
pub async fn get_observation_schema(
    State(supervisor): State<Arc<ArmSupervisor>>,
    axum::Extension(up): axum::Extension<crate::api::proxy::DetectorUpstream>,
    axum::Extension(vision): axum::Extension<crate::api::proxy::VisionCache>,
) -> Json<ObservationSchema> {
    let mut schema = supervisor.get_schema().await;
    if let Some(v) = vision.status(&up).await {
        crate::api::proxy::apply_vision_status(&mut schema.capabilities.vision, &v);
    }
    Json(schema)
}


// Actuator authority is bound to the WebSocket that acquired the lease. REST
// cannot prove that identity using a caller-supplied client id or generation.
async fn actuator_requires_connection_lease(supervisor: &ArmSupervisor) -> ApiError {
    let code = if supervisor.is_lease_active().await { "LEASE_ACTIVE" } else { "LEASE_REQUIRED" };
    ApiError(StatusCode::CONFLICT,
        "Use actuator commands on the WebSocket connection holding the control lease".into(), code.into())
}

/// POST /api/v1/torque
pub async fn set_torque(
    State(supervisor): State<Arc<ArmSupervisor>>,
    Json(_payload): Json<TorqueRequest>,
) -> Result<Json<TorqueResponse>, ApiError> {
    Err(actuator_requires_connection_lease(&supervisor).await)
}

pub fn try_sync_system_time(timestamp_ms: Option<u64>) {
    static SYNC_FAILED_REPORTED: std::sync::atomic::AtomicBool =
        std::sync::atomic::AtomicBool::new(false);

    if let Some(ts_ms) = timestamp_ms {
        let ts_sec = ts_ms / 1000;
        // Bounded valid window: 2024-01-01 to 2035-01-01
        if (1_704_067_200..=2_051_222_400).contains(&ts_sec) {
            let now_sec = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(0, |d| d.as_secs());
            if now_sec < 1_704_067_200 {
                #[cfg(target_os = "linux")]
                unsafe {
                    let ts = libc::timespec {
                        tv_sec: ts_sec as libc::time_t,
                        tv_nsec: ((ts_ms % 1000) * 1_000_000) as libc::c_long,
                    };
                    if libc::clock_settime(libc::CLOCK_REALTIME, &ts) == 0 {
                        SYNC_FAILED_REPORTED.store(false, std::sync::atomic::Ordering::Relaxed);
                        tracing::info!(
                            "System time synchronized from client: {} (was epoch {})",
                            ts_sec,
                            now_sec
                        );
                    } else if !SYNC_FAILED_REPORTED.swap(true, std::sync::atomic::Ordering::Relaxed) {
                        tracing::warn!(
                            "Passive clock synchronization failed (errno: {})",
                            std::io::Error::last_os_error()
                        );
                    }
                }
            }
        }
    }
}

/// POST /api/v1/heartbeat
pub async fn heartbeat(
    State(supervisor): State<Arc<ArmSupervisor>>,
    Json(_payload): Json<HeartbeatRequest>,
) -> Result<Json<HeartbeatResponse>, ApiError> {
    Err(actuator_requires_connection_lease(&supervisor).await)
}

/// POST /api/v1/motion/servo
pub async fn motion_servo(
    State(supervisor): State<Arc<ArmSupervisor>>,
    Json(_payload): Json<ServoRequest>,
) -> Result<Json<MotionResponse>, ApiError> {
    Err(actuator_requires_connection_lease(&supervisor).await)
}

/// POST /api/v1/motion/jog
pub async fn motion_jog(
    State(supervisor): State<Arc<ArmSupervisor>>,
    Json(_payload): Json<JogRequest>,
) -> Result<Json<MotionResponse>, ApiError> {
    Err(actuator_requires_connection_lease(&supervisor).await)
}

/// POST /api/v1/motion/home
pub async fn motion_home(
    State(supervisor): State<Arc<ArmSupervisor>>,
    Json(_payload): Json<HomeRequest>,
) -> Result<Json<MotionResponse>, ApiError> {
    Err(actuator_requires_connection_lease(&supervisor).await)
}

/// POST /api/v1/motion/unload
pub async fn motion_unload(
    State(supervisor): State<Arc<ArmSupervisor>>,
) -> Result<Json<serde_json::Value>, ApiError> {
    Err(actuator_requires_connection_lease(&supervisor).await)
}

/// POST /api/v1/action/{move_to,move_relative,gripper}: actions run only on the
/// WebSocket that holds the lease (`docs/agent_action_api.md`).
/// Grasp dry run (`dry_run: true`) is allowed without a lease and returns 200 OK with reachability.
pub async fn action_requires_lease(
    State(supervisor): State<Arc<ArmSupervisor>>,
    axum::Extension(deps): axum::Extension<crate::grasp::GraspDeps>,
    axum::Extension(auth): axum::Extension<crate::auth::OperatorAuth>,
    headers: axum::http::HeaderMap,
    uri: axum::http::Uri,
    body: axum::body::Bytes,
) -> Result<Json<serde_json::Value>, ApiError> {
    let token = crate::auth::OperatorAuth::extract_token(&headers);
    if auth.is_enforced() && !auth.is_operator(token.as_deref()) {
        if uri.path() == "/api/v1/mode/gravity_comp" || uri.path() == crate::measure_multiview::PATH {
            return Err(ApiError::operator_required(uri.path()));
        }
        #[cfg(feature = "policy-runtime")]
        if uri.path() == "/api/v1/action/policy/start" && !supervisor.inner().read().await.policy_live_allowed() {
            return Err(ApiError::operator_required("policy/start (real arm)"));
        }
        if uri.path() == "/api/v1/action/gripper_zero_check" {
            if let Ok(val) = serde_json::from_slice::<serde_json::Value>(&body) {
                if val.get("store_offset").and_then(serde_json::Value::as_bool) == Some(true) {
                    return Err(ApiError::operator_required("gripper_zero_check (store_offset=true)"));
                }
            }
        }
    }
    if uri.path() == "/api/v1/action/grasp" {
        if let Ok(val) = serde_json::from_slice::<serde_json::Value>(&body) {
            if val.get("dry_run").and_then(serde_json::Value::as_bool) == Some(true) {
                let dry = crate::grasp::dry_run_task(&supervisor, deps.source.as_ref(), &val).await;
                return Ok(Json(serde_json::Value::Object(dry.fields())));
            }
        }
    }
    Err(actuator_requires_connection_lease(&supervisor).await)
}

/// POST /api/v1/safety/e_stop
pub async fn safety_e_stop(
    State(supervisor): State<Arc<ArmSupervisor>>,
) -> Result<Json<SafetyResponse>, ApiError> {
    let (safety_state, epoch_id) = supervisor.trigger_e_stop_checked().await?;
    Ok(Json(SafetyResponse {
        safety_state,
        epoch_id,
    }))
}

/// POST /api/v1/safety/clear_fault
pub async fn safety_clear_fault(
    State(supervisor): State<Arc<ArmSupervisor>>,
) -> Result<Json<SafetyResponse>, ApiError> {
    Err(actuator_requires_connection_lease(&supervisor).await)
}

/// POST /api/v1/dataset/record/start
pub async fn dataset_record_start(
    State(supervisor): State<Arc<ArmSupervisor>>,
    Json(payload): Json<StartRecordRequest>,
) -> Result<Json<RecordActionResponse>, ApiError> {
    let path = payload.output_path.map(PathBuf::from);
    supervisor.start_recording(payload.episode_id, path).await?;
    let (id, path) = {
        let inner = supervisor.inner().read().await;
        (inner.recorder.episode_id().to_string(), inner.recorder.output_path().to_string_lossy().to_string())
    };
    Ok(Json(RecordActionResponse {
        status: "recording_started".to_string(),
        path: Some(path),
        frame_count: Some(0),
        episode_id: Some(id),
    }))
}

/// POST /api/v1/dataset/record/save
pub async fn dataset_record_save(
    State(supervisor): State<Arc<ArmSupervisor>>,
) -> Result<Json<RecordActionResponse>, ApiError> {
    let (_, count) = supervisor.get_recording_state().await;
    let path = supervisor.save_recording().await?;
    let id = supervisor.inner().read().await.recorder.episode_id().to_string();
    Ok(Json(RecordActionResponse {
        status: "saved".to_string(),
        path: Some(path.to_string_lossy().to_string()),
        frame_count: Some(count),
        episode_id: Some(id),
    }))
}

/// POST /api/v1/dataset/record/discard
pub async fn dataset_record_discard(
    State(supervisor): State<Arc<ArmSupervisor>>,
) -> Result<Json<RecordActionResponse>, ApiError> {
    supervisor.discard_recording().await?;
    Ok(Json(RecordActionResponse {
        status: "discarded".to_string(),
        path: None,
        frame_count: Some(0),
        episode_id: None,
    }))
}

/// `GET /api/v1/dataset/episodes`: saved episodes, newest first (no lease; legacy listing of
/// the episode store).
pub async fn dataset_episodes(
    State(supervisor): State<Arc<ArmSupervisor>>,
) -> Result<Json<crate::recording::EpisodeList>, ApiError> {
    supervisor.ensure_episodes_loaded().await?;
    let (store, offsets) = {
        let inner = supervisor.inner().read().await;
        (inner.episodes.clone(), inner.joint_zero_offsets.clone())
    };
    let list = tokio::task::spawn_blocking(move || store.list_with(&offsets)).await
        .map_err(|e| ArmError::Internal(format!("episode listing failed: {e}")))?
        .map_err(ArmError::from)?;
    Ok(Json(list))
}

/// `DELETE /api/v1/dataset/episodes/{id}`: remove a saved episode. Like the other
/// `/api/v1/dataset/*` routes it needs no lease (it moves nothing); the same-origin rule
/// for browser requests applies. IN_USE when a sequence references it.
pub async fn dataset_episode_delete(
    State(supervisor): State<Arc<ArmSupervisor>>,
    axum::extract::Path(id): axum::extract::Path<String>,
) -> Result<Json<serde_json::Value>, ApiError> {
    supervisor.ensure_episodes_loaded().await?;
    let store = {
        let inner = supervisor.inner().read().await;
        inner.episodes.path_of(&id).map_err(ArmError::from)?;
        inner.episode_delete_allowed(&id)?;
        inner.episodes.take_for_delete(&id);
        inner.episodes.clone()
    };
    let id2 = id.clone();
    tokio::task::spawn_blocking(move || store.delete_files(&id2)).await
        .map_err(|e| ArmError::Internal(format!("episode delete failed: {e}")))?
        .map_err(ArmError::from)?;
    Ok(Json(serde_json::json!({"status": "deleted", "episode_id": id})))
}

#[derive(Debug, Deserialize, Default)]
pub struct EpisodesQuery {
    pub dataset: Option<String>,
    pub limit: Option<usize>,
    pub before: Option<String>,
}

/// `GET /api/v1/episodes?dataset=&limit=&before=`: `{enabled, episodes: [sidecar + stale + bytes], recording}`, newest first.
pub async fn get_episodes(
    State(supervisor): State<Arc<ArmSupervisor>>,
    axum::extract::Query(q): axum::extract::Query<EpisodesQuery>,
) -> Result<Json<serde_json::Value>, ApiError> {
    supervisor.ensure_episodes_loaded().await?;
    let inner = supervisor.inner().read().await;
    let episodes = inner.episodes.list_episodes(q.dataset.as_deref(), q.limit, q.before.as_deref(), &inner.joint_zero_offsets);
    Ok(Json(serde_json::json!({"enabled": inner.episodes.enabled(), "episodes": episodes, "recording": inner.recording_status()})))
}

/// `GET /api/v1/episodes/{id}`: the sidecar + `stale` + `bytes`.
pub async fn get_episode(
    State(supervisor): State<Arc<ArmSupervisor>>,
    axum::extract::Path(id): axum::extract::Path<String>,
) -> Result<Json<crate::recording::EpisodeView>, ApiError> {
    supervisor.ensure_episodes_loaded().await?;
    let inner = supervisor.inner().read().await;
    inner.episodes.view(&id, &inner.joint_zero_offsets).map(Json)
        .ok_or_else(|| ArmError::from(crate::recording::StoreError::NotFound(id)).into())
}

/// `GET /api/v1/episodes/{id}/data`: the episode's parquet file.
pub async fn get_episode_data(
    State(supervisor): State<Arc<ArmSupervisor>>,
    axum::extract::Path(id): axum::extract::Path<String>,
) -> Result<Response, ApiError> {
    supervisor.ensure_episodes_loaded().await?;
    let store = supervisor.episode_store().await;
    if !store.contains(&id) { return Err(ArmError::from(crate::recording::StoreError::NotFound(id)).into()); }
    let path = store.path_of(&id).map_err(ArmError::from)?;
    let bytes = tokio::task::spawn_blocking(move || std::fs::read(path)).await
        .map_err(|e| ArmError::Internal(format!("episode read failed: {e}")))?
        .map_err(|e| ArmError::from(crate::recording::StoreError::Io(e)))?;
    Ok((
        [
            (axum::http::header::CONTENT_TYPE, "application/vnd.apache.parquet".to_string()),
            (axum::http::header::CONTENT_DISPOSITION, format!("attachment; filename=\"{id}.parquet\"")),
        ],
        bytes,
    ).into_response())
}

fn import_invalid(reason: &str, message: impl Into<String>) -> ApiError {
    ArmError::detailed(422, "IMPORT_INVALID", message, serde_json::json!({"reason": reason})).into()
}

/// `POST /api/v1/episodes/import` (multipart: `parquet` = the episode file, `sidecar` = its
/// JSON): checked against this arm (`crate::recording::import`), stored as a new episode
/// (`source: import`). -> `{id, stale, episode}` | IMPORT_INVALID `{reason}`. No lease.
pub async fn episodes_import(
    State(supervisor): State<Arc<ArmSupervisor>>,
    mut multipart: axum::extract::Multipart,
) -> Result<Json<serde_json::Value>, ApiError> {
    supervisor.ensure_episodes_loaded().await?;
    let (mut parquet, mut sidecar) = (None, None);
    while let Some(field) = multipart.next_field().await.map_err(|e| import_invalid("multipart", e.to_string()))? {
        let name = field.name().unwrap_or("").to_string();
        let data = field.bytes().await.map_err(|e| import_invalid("multipart", e.to_string()))?;
        match name.as_str() {
            "parquet" => parquet = Some(data),
            "sidecar" => sidecar = Some(data),
            _ => {}
        }
    }
    let parquet = parquet.ok_or_else(|| import_invalid("multipart", "part 'parquet' is missing"))?;
    let sidecar: serde_json::Value = serde_json::from_slice(&sidecar.ok_or_else(|| import_invalid("multipart", "part 'sidecar' is missing"))?)
        .map_err(|e| import_invalid("sidecar", format!("sidecar is not JSON: {e}")))?;
    let (ctx, store) = {
        let inner = supervisor.inner().read().await;
        (crate::recording::import::ImportContext {
            joint_names: inner.joint_names(), limits: inner.config.limits.clone(), profile_id: inner.config.profile_id.clone(),
            arm_dof: inner.config.arm_dof(), gripper_joint: inner.config.gripper_index(),
            zero_offsets: inner.joint_zero_offsets.clone(), fps: f64::from(inner.config.control_frequency_hz),
        }, inner.episodes.clone())
    };
    if !store.enabled() { return Err(ArmError::from(crate::recording::StoreError::Disabled).into()); }
    let result = tokio::task::spawn_blocking(move || -> Result<(crate::recording::EpisodeSidecar, bool), ApiError> {
        let parquet_kv = crate::recording::read_parquet_kv_from_bytes(parquet.clone())
            .map_err(|e| import_invalid("schema", format!("parquet: {e}")))?;
        let frames = crate::recording::read_frames_from_bytes(parquet).map_err(|e| import_invalid("schema", format!("parquet: {e}")))?;
        let checked = crate::recording::import::check_with_provenance(&ctx, &sidecar, &frames, Some(&parquet_kv))
            .map_err(|e| import_invalid(e.reason, e.message))?;
        let mut sc = checked.sidecar;
        sc.id = crate::recording::new_ulid();
        store.check_capacity().map_err(ArmError::from)?;
        store.check_free_for_start().map_err(ArmError::from)?;
        if !store.has_dataset(&sc.dataset) { store.create_dataset(&sc.dataset, "").map_err(ArmError::from)?; }
        store.write_episode(&sc, &frames, None).map_err(ArmError::from)?;
        Ok((sc, checked.stale))
    }).await.map_err(|e| ArmError::Internal(format!("import failed: {e}")))?;
    let (sc, stale) = result?;
    tracing::info!("Episode {} imported ({} frames, dataset '{}'{})", sc.id, sc.frames, sc.dataset, if stale { ", stale" } else { "" });
    Ok(Json(serde_json::json!({"id": sc.id, "stale": stale, "episode": sc})))
}

/// `GET /api/v1/sequences`: `{sequences: [sequence + stale_steps + missing_steps + running]}`.
pub async fn get_sequences(State(supervisor): State<Arc<ArmSupervisor>>) -> Result<Json<serde_json::Value>, ApiError> {
    supervisor.ensure_episodes_loaded().await?;
    let inner = supervisor.inner().read().await;
    let list: Vec<_> = inner.sequences.file.sequences.iter().map(|s| crate::sequences::view(&inner, s)).collect();
    Ok(Json(serde_json::json!({"sequences": list, "revision": inner.sequences.revision})))
}

/// `GET /api/v1/sequences/{id}`.
pub async fn get_sequence(
    State(supervisor): State<Arc<ArmSupervisor>>,
    axum::extract::Path(id): axum::extract::Path<String>,
) -> Result<Json<serde_json::Value>, ApiError> {
    supervisor.ensure_episodes_loaded().await?;
    let inner = supervisor.inner().read().await;
    let s = inner.sequences.get(&id)
        .ok_or_else(|| ApiError(StatusCode::NOT_FOUND, format!("No sequence with id '{id}'"), "SEQUENCE_NOT_FOUND".into()))?;
    Ok(Json(crate::sequences::view(&inner, s)))
}

/// `GET /api/v1/datasets`: `{datasets: [{name, episodes, duration_s, bytes, created_at, description}]}`.
pub async fn get_datasets(State(supervisor): State<Arc<ArmSupervisor>>) -> Result<Json<serde_json::Value>, ApiError> {
    supervisor.ensure_episodes_loaded().await?;
    let store = supervisor.episode_store().await;
    Ok(Json(serde_json::json!({"enabled": store.enabled(), "datasets": store.datasets()})))
}

/// GET /api/v1/grasp/setup: the runtime grasp setup (change it with the WS command
/// `/api/v1/grasp/setup`). `camera_down` says whether a grasp on the current mount acquires
/// camera-down (`crate::grasp::camera_down_mode`; then `grip_pixel_norm` is not the centring
/// aim); null with `camera_down_error` when the setting cannot be served. `width_check` compares
/// `max_grasp_width_m` with the jaw curve (`crate::grasp::grasp_width_check_json`).
pub async fn get_grasp_setup(
    State(supervisor): State<Arc<ArmSupervisor>>,
    axum::Extension(deps): axum::Extension<crate::grasp::GraspDeps>,
) -> Response {
    let (setup, revision, camera_down, width_check) = {
        let inner = supervisor.inner().read().await;
        let setup = inner.grasp_setup();
        let camera_down = setup.as_ref().map(|s| crate::grasp::camera_down_mode(s, &inner.config.camera_mount));
        let width_check = setup.as_ref().map(|s| crate::grasp::grasp_width_check_json(&inner, s.max_grasp_width_m));
        (setup, inner.grasp_setup_revision, camera_down, width_check)
    };
    let path = deps.setup_path.as_ref().map(|p| p.display().to_string());
    match setup {
        Some(s) => {
            let mut v = serde_json::json!({"supported": true, "setup": s, "path": path, "revision": revision,
                "width_check": width_check});
            match camera_down {
                Some(Ok(b)) => { v["camera_down"] = serde_json::json!(b); }
                Some(Err(e)) => { v["camera_down"] = serde_json::Value::Null; v["camera_down_error"] = serde_json::json!(e); }
                None => {}
            }
            Json(v).into_response()
        }
        None => Json(serde_json::json!({"supported": false, "setup": null, "path": path, "revision": revision})).into_response(),
    }
}

/// `POST /api/v1/grasp/measure`: one depth measurement at the current pose, no lease and no
/// motion (`crate::depth_measure`).
pub async fn post_grasp_measure(
    State(supervisor): State<Arc<ArmSupervisor>>,
    axum::Extension(deps): axum::Extension<crate::grasp::GraspDeps>,
    body: axum::body::Bytes,
) -> Result<Json<serde_json::Value>, ApiError> {
    let body: serde_json::Value = if body.is_empty() { serde_json::Value::Null } else {
        serde_json::from_slice(&body).map_err(|e| ApiError(StatusCode::BAD_REQUEST, format!("Body is not JSON: {e}"), "INVALID_COMMAND".into()))?
    };
    Ok(Json(crate::depth_measure::measure(&supervisor, deps.source.as_ref(), &body).await?))
}

/// `GET /api/v1/calibration`: the camera calibration in effect, the live mount and grasp
/// fields, and the backups (`crate::calibration`).
pub async fn get_calibration(
    State(supervisor): State<Arc<ArmSupervisor>>,
    axum::Extension(deps): axum::Extension<crate::calibration::CalibrationDeps>,
) -> Response {
    Json(crate::calibration::status(&supervisor, &deps).await).into_response()
}

/// `GET /api/v1/workspace/keepout`: effective keep-out boxes and their source.
pub async fn get_keepout(State(supervisor): State<Arc<ArmSupervisor>>) -> Response {
    Json(supervisor.inner().read().await.keepout_state()).into_response()
}

/// `GET /api/v1/poses`: stored named poses (no lease).
pub async fn get_poses(State(supervisor): State<Arc<ArmSupervisor>>) -> Response {
    let (list, revision, default_joint_names) = {
        let inner = supervisor.inner().read().await;
        let default_joint_names = inner
            .poses
            .list
            .joint_names
            .clone()
            .unwrap_or_else(|| inner.joint_names());
        (
            inner.poses.list.clone(),
            inner.poses.revision,
            default_joint_names,
        )
    };
    let poses: Vec<serde_json::Value> = list
        .poses
        .iter()
        .map(|p| crate::poses::pose_response_value(p, &default_joint_names))
        .collect();
    let mut obj = serde_json::Map::new();
    if let Some(profile_id) = list.profile_id {
        obj.insert("profile_id".into(), serde_json::Value::String(profile_id));
    }
    if let Some(arm_dof) = list.arm_dof {
        obj.insert("arm_dof".into(), serde_json::json!(arm_dof));
    }
    if let Some(gripper_joint) = list.gripper_joint {
        obj.insert("gripper_joint".into(), serde_json::json!(gripper_joint));
    }
    obj.insert(
        "joint_names".into(),
        serde_json::json!(list.joint_names.unwrap_or(default_joint_names)),
    );
    obj.insert("poses".into(), serde_json::Value::Array(poses));
    obj.insert("revision".into(), serde_json::json!(revision));
    Json(serde_json::Value::Object(obj)).into_response()
}

/// `GET /api/v1/safety/collision`: effective collision guard config (scaled thresholds).
pub async fn get_collision(State(supervisor): State<Arc<ArmSupervisor>>) -> Response {
    Json(supervisor.inner().read().await.collision_config()).into_response()
}

/// `GET /api/v1/safety/describe`: limits, stops and faults of this armd (no lease).
pub async fn get_safety_describe(
    State(supervisor): State<Arc<ArmSupervisor>>,
    axum::Extension(auth): axum::Extension<crate::auth::OperatorAuth>,
) -> Response {
    let mut val = supervisor.inner().read().await.safety_describe();
    if let Some(hw) = val.get_mut("hardware").and_then(serde_json::Value::as_object_mut) {
        hw.insert("operator_auth_enforced".into(), serde_json::json!(auth.is_enforced()));
    }
    Json(val).into_response()
}

/// `GET /api/v1/gripper/calibration`: current gripper calibration and effective policy (no lease).
/// GET /api/v1/gripper/calibration
pub async fn get_gripper_calibration(State(supervisor): State<Arc<ArmSupervisor>>) -> Response {
    Json(supervisor.inner().read().await.gripper_calibration_response()).into_response()
}

/// GET /api/v1/tool/commissioning
pub async fn get_tool_commissioning(State(supervisor): State<Arc<ArmSupervisor>>) -> Response {
    Json(supervisor.inner().read().await.tool_commissioning_response()).into_response()
}

/// POST /api/v1/tool/commissioning/confirm
pub async fn post_tool_commissioning_confirm(
    State(supervisor): State<Arc<ArmSupervisor>>,
    axum::Extension(auth): axum::Extension<crate::auth::OperatorAuth>,
    payload: Result<Json<serde_json::Value>, axum::extract::rejection::JsonRejection>,
) -> Result<Response, ApiError> {
    let Json(body) = payload.map_err(|e| {
        ApiError(
            StatusCode::BAD_REQUEST,
            format!("Invalid commissioning confirm body: {e}"),
            "INVALID_COMMAND".into(),
        )
    })?;
    let cw_lock = supervisor.config_write_lock();
    let _cw = cw_lock.lock().await;
    let mut prepared = {
        let inner = supervisor.inner().read().await;
        crate::tool::prepare_commissioning_confirm(&inner, &body, auth.is_enforced())?
    };
    let write = prepared.write.take();
    let (ds, active_tool, revision) = if let Some(w) = write {
        let staged = tokio::task::spawn_blocking(move || crate::persist::StagedWrite::stage(w.path, w.text))
            .await
            .map_err(|e| ApiError::persist_failed(format!("commissioning stage task: {e}")))?
            .map_err(|e| ApiError::persist_failed(format!("commissioning stage failed: {e}")))?;
        let mut inner = supervisor.inner().write().await;
        if let Err(e) = crate::tool::precheck_commissioning_confirm(&inner, &prepared) {
            staged.abort();
            return Err(e.into());
        }
        // versioned change (track B): reserve at the candidate's revision, rename, apply + activate
        let reservation = match inner.reserve_config(crate::rt_core::config_iface::ConfigDomain::ToolCommissioning, prepared.base_revision, 0) {
            Ok(r) => r,
            Err(e) => {
                staged.abort();
                return Err(e.into());
            }
        };
        let ds = match staged.rename_only() {
            Ok(ds) => ds,
            Err(e) => {
                let _ = crate::rt_core::config_iface::ConfigAuthority::release(&mut *inner, reservation);
                return Err(ApiError::persist_failed(format!("commissioning rename failed: {e}")));
            }
        };
        let reservation = inner.config_persisted(reservation, crate::rt_core::config_iface::Durability::DurableUnconfirmed)?;
        let ((active_tool, _), revision) = inner.activate_config_candidate(reservation, |inner| crate::tool::commit_commissioning_confirm(inner, prepared))?;
        (ds, active_tool, revision)
    } else {
        let mut inner = supervisor.inner().write().await;
        crate::tool::precheck_commissioning_confirm(&inner, &prepared)?;
        let base = prepared.base_revision;
        let ((active_tool, _), revision) = inner.commit_config(crate::rt_core::config_iface::ConfigDomain::ToolCommissioning, base, 0, |inner| crate::tool::commit_commissioning_confirm(inner, prepared))?;
        return Ok(Json(serde_json::json!({
            "active_tool": active_tool,
            "commissioning_required": active_tool.commissioning_required,
            "revision": revision,
            "durability_confirmed": true,
        }))
        .into_response());
    };
    let outcome = tokio::task::spawn_blocking(move || ds.sync_parent_dir())
        .await
        .unwrap_or_else(|e| crate::persist::PublishOutcome::PublishedDurabilityUnconfirmed(e.to_string()));
    let mut resp = serde_json::json!({
        "active_tool": active_tool,
        "commissioning_required": active_tool.commissioning_required,
        "revision": revision,
        "durability_confirmed": matches!(outcome, crate::persist::PublishOutcome::Published),
    });
    if let crate::persist::PublishOutcome::PublishedDurabilityUnconfirmed(w) = outcome {
        resp["durability_warning"] = serde_json::json!(w);
    }
    Ok(Json(resp).into_response())
}

#[derive(Debug, Deserialize)]
pub struct UnlockRequest {
    pub pin: String,
    #[serde(default)]
    pub set_cookie: Option<bool>,
}

/// `GET /api/v1/auth/status`: current operator auth status (no lease).
pub async fn auth_status(
    axum::Extension(auth): axum::Extension<crate::auth::OperatorAuth>,
    headers: axum::http::HeaderMap,
) -> Json<crate::auth::AuthStatus> {
    let token = crate::auth::OperatorAuth::extract_token(&headers);
    Json(auth.status(token.as_deref()))
}

/// `POST /api/v1/auth/unlock`: validate operator PIN, issue session token + optional HttpOnly cookie.
pub async fn auth_unlock(
    axum::Extension(auth): axum::Extension<crate::auth::OperatorAuth>,
    headers: axum::http::HeaderMap,
    payload: Result<Json<UnlockRequest>, axum::extract::rejection::JsonRejection>,
) -> Result<Response, ApiError> {
    let Json(req) = payload.map_err(|e| ApiError(StatusCode::BAD_REQUEST, format!("Invalid unlock request: {e}"), "INVALID_COMMAND".into()))?;
    if !auth.is_enforced() {
        return Ok(Json(serde_json::json!({
            "role": "operator",
            "token": null,
            "expires_in_s": null,
        })).into_response());
    }
    let (token, expires_in_s) = auth.unlock(&req.pin).map_err(|e| match e {
        crate::auth::UnlockError::InvalidPin => ApiError::invalid_pin(),
        crate::auth::UnlockError::RateLimited { retry_after_s } => ApiError::rate_limited(retry_after_s),
        crate::auth::UnlockError::RngFailed => ApiError(
            StatusCode::INTERNAL_SERVER_ERROR,
            "Failed to generate session token".into(),
            "INTERNAL_ERROR".into(),
        ),
    })?;
    let mut resp = Json(serde_json::json!({
        "role": "operator",
        "token": token,
        "expires_in_s": expires_in_s,
    })).into_response();
    if req.set_cookie != Some(false) {
        let secure = if auth.should_set_secure_cookie(&headers) { "; Secure" } else { "" };
        let cookie = format!(
            "{}={token}; HttpOnly; SameSite=Strict; Path=/; Max-Age={expires_in_s}{secure}",
            crate::auth::SESSION_COOKIE_NAME
        );
        if let Ok(val) = axum::http::HeaderValue::from_str(&cookie) {
            resp.headers_mut().insert(axum::http::header::SET_COOKIE, val);
        }
    }
    Ok(resp)
}

/// `POST /api/v1/auth/lock`: revoke Bearer session (no cookie clearing) or Cockpit Cookie session (clears cookie).
pub async fn auth_lock(
    axum::Extension(auth): axum::Extension<crate::auth::OperatorAuth>,
    headers: axum::http::HeaderMap,
) -> Response {
    match crate::auth::OperatorAuth::extract_credential(&headers) {
        crate::auth::AuthCredential::Bearer(tok) => {
            auth.lock(tok.as_deref());
            Json(serde_json::json!({"role": "program"})).into_response()
        }
        crate::auth::AuthCredential::Cookie(tok) => {
            auth.lock(Some(&tok));
            lock_response_clearing_cookie(&auth, &headers)
        }
        crate::auth::AuthCredential::None => lock_response_clearing_cookie(&auth, &headers),
    }
}

fn lock_response_clearing_cookie(auth: &crate::auth::OperatorAuth, headers: &axum::http::HeaderMap) -> Response {
    let secure = if auth.should_set_secure_cookie(headers) { "; Secure" } else { "" };
    let clear_cookie = format!(
        "{}=; HttpOnly; SameSite=Strict; Path=/; Max-Age=0{secure}",
        crate::auth::SESSION_COOKIE_NAME
    );
    let mut resp = Json(serde_json::json!({"role": "program"})).into_response();
    if let Ok(val) = axum::http::HeaderValue::from_str(&clear_cookie) {
        resp.headers_mut().insert(axum::http::header::SET_COOKIE, val);
    }
    resp
}
