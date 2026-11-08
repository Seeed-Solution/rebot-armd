pub mod proxy;
pub mod rest;
pub mod ws;

use std::sync::Arc;
use axum::{
    response::Html,
    routing::{get, post},
    Router,
};
use axum::{body::Body, http::{Request, StatusCode}, middleware::{self, Next}, response::Response};
use tower_http::trace::TraceLayer;

use crate::supervisor::ArmSupervisor;

static INDEX_HTML: &str = include_str!("index.html");

async fn serve_index() -> Html<&'static str> {
    Html(INDEX_HTML)
}

// JMuxer 2.1.0 (MIT), vendored; plays the camera's H.264-over-WebSocket stream.
static JMUXER_JS: &str = include_str!("vendor/jmuxer.min.js");

async fn serve_jmuxer() -> impl axum::response::IntoResponse {
    (
        [
            (axum::http::header::CONTENT_TYPE, "application/javascript; charset=utf-8"),
            (axum::http::header::CACHE_CONTROL, "public, max-age=31536000, immutable"),
        ],
        JMUXER_JS,
    )
}

/// Build the Axum router for the arm daemon. The detector control API upstream is
/// `ARMD_DETECTOR_URL` (default `http://127.0.0.1:8090`).
pub fn create_router(supervisor: Arc<ArmSupervisor>) -> Router {
    create_router_with_detector(supervisor, proxy::DetectorUpstream::from_env())
}

/// [`create_router`] with an explicit detector control API upstream.
pub fn create_router_with_detector(supervisor: Arc<ArmSupervisor>, detector: proxy::DetectorUpstream) -> Router {
    Router::new()
        .route("/", get(serve_index))
        .route("/assets/jmuxer.min.js", get(serve_jmuxer))
        .route("/api/v1/observation", get(rest::get_observation))
        .route("/api/v1/observation/schema", get(rest::get_observation_schema))
        .route("/api/v1/torque", post(rest::set_torque))
        .route("/api/v1/arm", post(rest::set_torque))
        .route("/api/v1/heartbeat", post(rest::heartbeat))
        .route("/api/v1/motion/jog", post(rest::motion_jog))
        .route("/api/v1/motion/home", post(rest::motion_home))
        .route("/api/v1/motion/unload", post(rest::motion_unload))
        .route("/api/v1/motion/servo", post(rest::motion_servo))
        .route("/api/v1/safety/e_stop", post(rest::safety_e_stop))
        .route("/api/v1/safety/clear_fault", post(rest::safety_clear_fault))
        .route("/api/v1/dataset/record/start", post(rest::dataset_record_start))
        .route("/api/v1/dataset/record/save", post(rest::dataset_record_save))
        .route("/api/v1/dataset/record/discard", post(rest::dataset_record_discard))
        .route("/api/v1/ws", get(ws::ws_handler))
        // Single-Origin Reverse Proxy Routes
        .route("/api/v1/detection/ws", get(proxy::proxy_detection_ws))
        .route("/results", get(proxy::proxy_detection_ws))
        .route(
            "/api/v1/detection/inference",
            get(proxy::get_detection_inference).post(proxy::post_detection_inference),
        )
        .route("/api/v1/detection/latest", get(proxy::get_detection_latest))
        .route("/api/v1/detection/trigger", post(proxy::post_detection_trigger))
        .layer(axum::Extension(detector))
        .layer(middleware::from_fn(same_origin))
        .layer(TraceLayer::new_for_http())
        .with_state(supervisor)
}

// Browser origin protection is not network authentication. Non-browser clients
// remain suitable only for a trusted network or an authenticated reverse proxy.
async fn same_origin(request: Request<Body>, next: Next) -> Response {
    let protected = request.method() != axum::http::Method::GET
        || request.headers().get("upgrade").and_then(|h| h.to_str().ok()).map_or(false, |h| h.eq_ignore_ascii_case("websocket"));
    if protected {
        if let Some(origin) = request.headers().get("origin") {
            let host = request.headers().get("host").and_then(|h| h.to_str().ok());
            let valid = origin.to_str().ok().and_then(|o| o.parse::<axum::http::Uri>().ok()).map_or(false, |uri| {
                matches!(uri.scheme_str(), Some("http" | "https"))
                    && uri.authority().map(|a| a.as_str()) == host
                    && uri.path_and_query().map_or(true, |p| p.as_str() == "/")
            });
            if !valid {
                return axum::response::IntoResponse::into_response((StatusCode::FORBIDDEN, "Cross-origin control requests are not allowed"));
            }
        }
    }
    next.run(request).await
}
