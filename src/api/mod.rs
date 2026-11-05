pub mod proxy;
pub mod rest;
pub mod ws;

use std::sync::Arc;
use axum::{
    response::Html,
    routing::{get, post},
    Router,
};
use tower_http::cors::{Any, CorsLayer};
use tower_http::trace::TraceLayer;

use crate::supervisor::ArmSupervisor;

static INDEX_HTML: &str = include_str!("index.html");

async fn serve_index() -> Html<&'static str> {
    Html(INDEX_HTML)
}

/// Build the Axum router for the arm daemon
pub fn create_router(supervisor: Arc<ArmSupervisor>) -> Router {
    let cors = CorsLayer::new()
        .allow_origin(Any)
        .allow_methods(Any)
        .allow_headers(Any);

    Router::new()
        .route("/", get(serve_index))
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
        .route("/api/ws", get(proxy::proxy_go2rtc_ws))
        .route("/api/webrtc", post(proxy::proxy_go2rtc_webrtc))
        .route("/api/stream.mp4", get(proxy::proxy_go2rtc_stream))
        .route("/api/stream.mjpeg", get(proxy::proxy_go2rtc_stream))
        .layer(cors)
        .layer(TraceLayer::new_for_http())
        .with_state(supervisor)
}
