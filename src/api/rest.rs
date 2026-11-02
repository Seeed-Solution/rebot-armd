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

#[derive(Debug, Serialize)]
pub struct ApiErrorResponse {
    pub error: String,
    pub code: String,
}

pub struct ApiError(pub StatusCode, pub String, pub String);

impl IntoResponse for ApiError {
    fn into_response(self) -> Response {
        let body = Json(ApiErrorResponse {
            error: self.1,
            code: self.2,
        });
        (self.0, body).into_response()
    }
}

impl From<ArmError> for ApiError {
    fn from(err: ArmError) -> Self {
        match err {
            ArmError::Safety(safety_err) => match safety_err {
                crate::safety::SafetyError::EStopActive => {
                    ApiError(StatusCode::CONFLICT, safety_err.to_string(), "ESTOP_ACTIVE".to_string())
                }
                crate::safety::SafetyError::EpochMismatch { .. } => {
                    ApiError(StatusCode::CONFLICT, safety_err.to_string(), "EPOCH_MISMATCH".to_string())
                }
                crate::safety::SafetyError::SoftLimitExceeded { .. } => {
                    ApiError(StatusCode::BAD_REQUEST, safety_err.to_string(), "SOFT_LIMIT_EXCEEDED".to_string())
                }
                crate::safety::SafetyError::FaultActive(_) => {
                    ApiError(StatusCode::CONFLICT, safety_err.to_string(), "FAULT_ACTIVE".to_string())
                }
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
pub async fn get_observation_schema(
    State(supervisor): State<Arc<ArmSupervisor>>,
) -> Json<ObservationSchema> {
    Json(supervisor.get_schema().await)
}

/// POST /api/v1/torque
pub async fn set_torque(
    State(supervisor): State<Arc<ArmSupervisor>>,
    Json(payload): Json<TorqueRequest>,
) -> Result<Json<TorqueResponse>, ApiError> {
    let safety_state = supervisor.set_torque(payload.enable).await?;
    Ok(Json(TorqueResponse {
        is_torqued: payload.enable,
        safety_state,
    }))
}

/// POST /api/v1/heartbeat
pub async fn heartbeat(
    State(supervisor): State<Arc<ArmSupervisor>>,
    Json(payload): Json<HeartbeatRequest>,
) -> Result<Json<HeartbeatResponse>, ApiError> {
    supervisor
        .heartbeat(&payload.client_id, payload.epoch_id)
        .await
        .map_err(ArmError::Safety)?;
    Ok(Json(HeartbeatResponse {
        status: "ok".to_string(),
        epoch_id: payload.epoch_id,
    }))
}

/// POST /api/v1/motion/servo
pub async fn motion_servo(
    State(supervisor): State<Arc<ArmSupervisor>>,
    Json(payload): Json<ServoRequest>,
) -> Result<Json<MotionResponse>, ApiError> {
    let epoch_id = payload.epoch_id.ok_or_else(|| {
        ApiError(
            StatusCode::BAD_REQUEST,
            "epoch_id is required for servo motion commands".to_string(),
            "INVALID_COMMAND".to_string(),
        )
    })?;
    supervisor.servo(payload.targets, epoch_id).await?;
    let obs = supervisor.get_observation().await;
    Ok(Json(MotionResponse {
        status: "accepted".to_string(),
        epoch_id: obs.epoch_id,
    }))
}

/// POST /api/v1/motion/jog
pub async fn motion_jog(
    State(supervisor): State<Arc<ArmSupervisor>>,
    Json(payload): Json<JogRequest>,
) -> Result<Json<MotionResponse>, ApiError> {
    let epoch_id = supervisor
        .jog(payload.target_positions, payload.duration_secs, payload.epoch_id)
        .await?;
    Ok(Json(MotionResponse {
        status: "accepted".to_string(),
        epoch_id,
    }))
}

/// POST /api/v1/motion/home
pub async fn motion_home(
    State(supervisor): State<Arc<ArmSupervisor>>,
    Json(payload): Json<HomeRequest>,
) -> Result<Json<MotionResponse>, ApiError> {
    let epoch_id = supervisor
        .home(payload.duration_secs, payload.epoch_id)
        .await?;
    Ok(Json(MotionResponse {
        status: "accepted".to_string(),
        epoch_id,
    }))
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
    let safety_state = supervisor.clear_fault().await?;
    let obs = supervisor.get_observation().await;
    Ok(Json(SafetyResponse {
        safety_state,
        epoch_id: obs.epoch_id,
    }))
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
