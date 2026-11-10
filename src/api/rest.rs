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
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ApiErrorResponse {
    pub error: String,
    pub message: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub safety_state: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub code: Option<String>,
}

#[derive(Debug)]
pub struct ApiError {
    pub status: StatusCode,
    pub error: String,
    pub message: String,
    pub code: Option<String>,
    pub safety_state: Option<String>,
}

#[allow(non_snake_case)]
pub fn ApiError(status: StatusCode, message: String, code: String) -> ApiError {
    ApiError {
        status,
        error: code.clone(),
        message,
        code: Some(code),
        safety_state: None,
    }
}

impl IntoResponse for ApiError {
    fn into_response(self) -> Response {
        let body = Json(ApiErrorResponse {
            error: self.error,
            message: self.message,
            safety_state: self.safety_state,
            code: self.code,
        });
        (self.status, body).into_response()
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
                },
                crate::safety::SafetyError::ArmNotAtRestingPose { .. } => ApiError {
                    status: StatusCode::CONFLICT,
                    error: "ArmNotAtRestingPose".to_string(),
                    message: safety_err.to_string(),
                    code: Some("ARM_NOT_AT_RESTING_POSE".to_string()),
                    safety_state: Some("Armed".to_string()),
                },
                crate::safety::SafetyError::MotionForbiddenInFaultState(msg) => ApiError {
                    status: StatusCode::CONFLICT,
                    error: "MotionForbiddenInFaultState".to_string(),
                    message: msg,
                    code: Some("MOTION_FORBIDDEN_IN_FAULT_STATE".to_string()),
                    safety_state: Some("Fault".to_string()),
                },
                crate::safety::SafetyError::ClearFaultRejected(msg) => ApiError {
                    status: StatusCode::CONFLICT,
                    error: "ClearFaultRejected".to_string(),
                    message: msg,
                    code: Some("CLEAR_FAULT_REJECTED".to_string()),
                    safety_state: Some("Fault".to_string()),
                },
                crate::safety::SafetyError::EStopActive => ApiError {
                    status: StatusCode::CONFLICT,
                    error: "ESTOP_ACTIVE".to_string(),
                    message: safety_err.to_string(),
                    code: Some("ESTOP_ACTIVE".to_string()),
                    safety_state: Some("EStop".to_string()),
                },
                crate::safety::SafetyError::MotionBusy => {
                    ApiError(StatusCode::CONFLICT, safety_err.to_string(), "MOTION_BUSY".to_string())
                }
                crate::safety::SafetyError::WorkspaceViolation { .. } => {
                    ApiError(StatusCode::UNPROCESSABLE_ENTITY, safety_err.to_string(), "WORKSPACE_VIOLATION".to_string())
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
                },
                crate::safety::SafetyError::NotArmed(_) => {
                    ApiError(StatusCode::PRECONDITION_FAILED, safety_err.to_string(), "NOT_ARMED".to_string())
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
                ApiError(status, motion_err.to_string(), motion_err.code().to_string())
            }
            ArmError::Unsupported(msg) => {
                ApiError(StatusCode::UNPROCESSABLE_ENTITY, msg, "UNSUPPORTED".to_string())
            }
            ArmError::Rejected { status, code, message } => {
                ApiError(StatusCode::from_u16(status).unwrap_or(StatusCode::CONFLICT), message, code)
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
) -> Json<ArmObservation> {
    Json(supervisor.get_observation().await)
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
pub async fn action_requires_lease(
    State(supervisor): State<Arc<ArmSupervisor>>,
) -> Result<Json<serde_json::Value>, ApiError> {
    Err(actuator_requires_connection_lease(&supervisor).await)
}

/// POST /api/v1/safety/e_stop
pub async fn safety_e_stop(
    State(supervisor): State<Arc<ArmSupervisor>>,
) -> Json<SafetyResponse> {
    let (safety_state, epoch_id) = supervisor.trigger_e_stop().await;
    Json(SafetyResponse {
        safety_state,
        epoch_id,
    })
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
    Ok(Json(RecordActionResponse {
        status: "recording_started".to_string(),
        path: None,
        frame_count: Some(0),
    }))
}

/// POST /api/v1/dataset/record/save
pub async fn dataset_record_save(
    State(supervisor): State<Arc<ArmSupervisor>>,
) -> Result<Json<RecordActionResponse>, ApiError> {
    let (_, count) = supervisor.get_recording_state().await;
    let path = supervisor.save_recording().await?;
    Ok(Json(RecordActionResponse {
        status: "saved".to_string(),
        path: Some(path.to_string_lossy().to_string()),
        frame_count: Some(count),
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
    }))
}

/// GET /api/v1/grasp/setup: the runtime grasp setup (change it with the WS command
/// `/api/v1/grasp/setup`).
pub async fn get_grasp_setup(
    State(supervisor): State<Arc<ArmSupervisor>>,
    axum::Extension(deps): axum::Extension<crate::grasp::GraspDeps>,
) -> Response {
    let setup = supervisor.inner().read().await.grasp_setup();
    let path = deps.setup_path.as_ref().map(|p| p.display().to_string());
    match setup {
        Some(s) => Json(serde_json::json!({"supported": true, "setup": s, "path": path})).into_response(),
        None => Json(serde_json::json!({"supported": false, "setup": null, "path": path})).into_response(),
    }
}

/// `GET /api/v1/workspace/keepout`: effective keep-out boxes and their source.
pub async fn get_keepout(State(supervisor): State<Arc<ArmSupervisor>>) -> Response {
    Json(supervisor.inner().read().await.keepout_state()).into_response()
}

/// `GET /api/v1/poses`: stored named poses (no lease).
pub async fn get_poses(State(supervisor): State<Arc<ArmSupervisor>>) -> Response {
    Json(supervisor.inner().read().await.poses.list.clone()).into_response()
}
