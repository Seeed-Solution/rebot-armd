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
