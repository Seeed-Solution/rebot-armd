//! Read-only diagnostics routes (no lease): boot records and black-box records.

use axum::{
    extract::Query,
    http::header,
    response::{IntoResponse, Response},
    Extension,
};
use serde::Deserialize;

use crate::diagnostics::{blackbox_body, boots_body, DiagnosticsState};

/// Longest window `seconds` accepts (the files hold about this much while torqued).
pub const MAX_SECONDS: u64 = 3600;

fn json(body: String) -> Response {
    ([(header::CONTENT_TYPE, "application/json")], body).into_response()
}

async fn blocking(f: impl FnOnce() -> String + Send + 'static) -> Response {
    match tokio::task::spawn_blocking(f).await {
        Ok(body) => json(body),
        Err(_) => (axum::http::StatusCode::INTERNAL_SERVER_ERROR, "diagnostics read failed").into_response(),
    }
}

/// `GET /api/v1/diagnostics`: structured daemon diagnostics (shadow `TorqueController` telemetry).
pub async fn get_diagnostics(
    axum::extract::State(supervisor): axum::extract::State<std::sync::Arc<crate::supervisor::ArmSupervisor>>,
) -> Response {
    #[cfg(not(target_arch = "wasm32"))]
    {
        axum::Json(serde_json::json!({
            "torque_shadow": supervisor.torque_shadow_diagnostics(),
        }))
        .into_response()
    }
    #[cfg(target_arch = "wasm32")]
    {
        let _ = supervisor;
        axum::Json(serde_json::json!({})).into_response()
    }
}

/// `GET /api/v1/diagnostics/boots`
pub async fn get_boots(Extension(state): Extension<DiagnosticsState>) -> Response {
    blocking(move || boots_body(&state)).await
}

#[derive(Debug, Deserialize)]
pub struct BlackboxQuery {
    pub seconds: Option<u64>,
    pub prev: Option<u8>,
}

/// `GET /api/v1/diagnostics/blackbox?seconds=N[&prev=1]`
pub async fn get_blackbox(Extension(state): Extension<DiagnosticsState>, Query(q): Query<BlackboxQuery>) -> Response {
    let seconds = q.seconds.unwrap_or(60).min(MAX_SECONDS);
    let prev = q.prev.unwrap_or(0) != 0;
    blocking(move || blackbox_body(&state, seconds, prev)).await
}

/// `GET /api/v1/diagnostics/tick`: control tick timing over the last 1000 ticks plus monotonic
/// process-lifetime peaks/thresholds and optional baseline marker deltas.
pub async fn get_tick(axum::extract::State(supervisor): axum::extract::State<std::sync::Arc<crate::supervisor::ArmSupervisor>>) -> Response {
    axum::Json(supervisor.tick_stats()).into_response()
}

/// `POST /api/v1/diagnostics/tick/mark`: request a coherent control-thread baseline marker for
/// qualification runs (operator-protected; never mutates process-lifetime monotonic counters).
pub async fn post_tick_mark(
    axum::extract::State(supervisor): axum::extract::State<std::sync::Arc<crate::supervisor::ArmSupervisor>>,
) -> Response {
    match supervisor.mark_tick_baseline() {
        Ok(marker_id) => axum::Json(serde_json::json!({
            "marker_id": marker_id,
            "status": "pending",
        }))
        .into_response(),
        Err(e) => crate::api::rest::ApiError::from(e).into_response(),
    }
}

/// `GET /api/v1/diagnostics/power`: supply-current estimate, recent peaks and the budget in
/// force (`{"enabled": false}` without one).
pub async fn get_power(axum::extract::State(supervisor): axum::extract::State<std::sync::Arc<crate::supervisor::ArmSupervisor>>) -> Response {
    axum::Json(supervisor.inner().read().await.power_diagnostics()).into_response()
}
