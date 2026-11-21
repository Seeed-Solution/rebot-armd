pub mod assets;
pub mod diagnostics;
pub mod proxy;
pub mod rest;
pub mod sandbox;
pub mod ws;

use std::sync::Arc;
use axum::{
    body::Body,
    extract::DefaultBodyLimit,
    http::{Request, StatusCode},
    middleware::{self, Next},
    response::{Html, Response},
    routing::{get, post},
    Router,
};
use tower_http::trace::TraceLayer;

use crate::supervisor::ArmSupervisor;

/// Largest `POST /api/v1/episodes/import` body (a 600 s episode at the control rate is a few MB).
pub const IMPORT_BODY_LIMIT: usize = 64 * 1024 * 1024;

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

// Calibration wizard solvers (ES module, imported by index.html on demand; scripts/calib_wizard_selftest.mjs).
static CALIB_WIZARD_JS: &str = include_str!("calib_wizard.js");

async fn serve_calib_wizard() -> impl axum::response::IntoResponse {
    (
        [
            (axum::http::header::CONTENT_TYPE, "application/javascript; charset=utf-8"),
            (axum::http::header::CACHE_CONTROL, "no-cache"),
        ],
        CALIB_WIZARD_JS,
    )
}

/// Build the Axum router for the arm daemon with explicit [`crate::auth::OperatorAuth`].
/// The detector control API upstream is `ARMD_DETECTOR_URL` (default `http://127.0.0.1:8090`).
pub fn create_router(supervisor: Arc<ArmSupervisor>, auth: crate::auth::OperatorAuth) -> Router {
    create_router_with_auth(supervisor, auth)
}

/// Alias for [`create_router`] with explicit [`crate::auth::OperatorAuth`].
pub fn create_router_with_auth(supervisor: Arc<ArmSupervisor>, auth: crate::auth::OperatorAuth) -> Router {
    let detector = proxy::DetectorUpstream::from_env();
    let grasp = crate::grasp::GraspDeps::http(detector.clone(), Some(crate::grasp::setup_path_from_env()));
    let diagnostics = crate::diagnostics::DiagnosticsState::global();
    let calibration = crate::calibration::CalibrationDeps::from_env(grasp.setup_path.clone(), detector.clone());
    create_router_calibration_with_auth(supervisor, detector, grasp, diagnostics, calibration, auth)
}

/// [`create_router`] with an explicit detector control API upstream.
pub fn create_router_with_detector(
    supervisor: Arc<ArmSupervisor>,
    detector: proxy::DetectorUpstream,
    auth: crate::auth::OperatorAuth,
) -> Router {
    let grasp = crate::grasp::GraspDeps::http(detector.clone(), Some(crate::grasp::setup_path_from_env()));
    create_router_with(supervisor, detector, grasp, auth)
}

/// [`create_router_with_detector`] with explicit grasp dependencies (detection source, setup file).
pub fn create_router_with(
    supervisor: Arc<ArmSupervisor>,
    detector: proxy::DetectorUpstream,
    grasp: crate::grasp::GraspDeps,
    auth: crate::auth::OperatorAuth,
) -> Router {
    create_router_full(supervisor, detector, grasp, crate::diagnostics::DiagnosticsState::global(), auth)
}

/// [`create_router_with`] with explicit diagnostics files (`/api/v1/diagnostics/*`).
pub fn create_router_full(
    supervisor: Arc<ArmSupervisor>,
    detector: proxy::DetectorUpstream,
    grasp: crate::grasp::GraspDeps,
    diagnostics: crate::diagnostics::DiagnosticsState,
    auth: crate::auth::OperatorAuth,
) -> Router {
    let calibration = crate::calibration::CalibrationDeps::from_env(grasp.setup_path.clone(), detector.clone());
    create_router_calibration(supervisor, detector, grasp, diagnostics, calibration, auth)
}

/// [`create_router_full`] with explicit calibration files (`/api/v1/calibration*`).
pub fn create_router_calibration(
    supervisor: Arc<ArmSupervisor>,
    detector: proxy::DetectorUpstream,
    grasp: crate::grasp::GraspDeps,
    diagnostics: crate::diagnostics::DiagnosticsState,
    calibration: crate::calibration::CalibrationDeps,
    auth: crate::auth::OperatorAuth,
) -> Router {
    create_router_calibration_with_auth(
        supervisor,
        detector,
        grasp,
        diagnostics,
        calibration,
        auth,
    )
}

/// [`create_router`] with explicit [`crate::auth::OperatorAuth`] and [`sandbox::SandboxState`].
pub fn create_router_with_sandbox(
    supervisor: Arc<ArmSupervisor>,
    auth: crate::auth::OperatorAuth,
    sandbox: Arc<sandbox::SandboxState>,
) -> Router {
    let detector = proxy::DetectorUpstream::from_env();
    let grasp = crate::grasp::GraspDeps::http(detector.clone(), Some(crate::grasp::setup_path_from_env()));
    let diagnostics = crate::diagnostics::DiagnosticsState::global();
    let calibration = crate::calibration::CalibrationDeps::from_env(grasp.setup_path.clone(), detector.clone());
    create_router_full_with_sandbox(supervisor, detector, grasp, diagnostics, calibration, auth, sandbox)
}

/// [`create_router_calibration`] with explicit [`crate::auth::OperatorAuth`].
pub fn create_router_calibration_with_auth(
    supervisor: Arc<ArmSupervisor>,
    detector: proxy::DetectorUpstream,
    grasp: crate::grasp::GraspDeps,
    diagnostics: crate::diagnostics::DiagnosticsState,
    calibration: crate::calibration::CalibrationDeps,
    auth: crate::auth::OperatorAuth,
) -> Router {
    create_router_full_with_sandbox(
        supervisor,
        detector,
        grasp,
        diagnostics,
        calibration,
        auth,
        sandbox::SandboxState::new_shared(),
    )
}

pub fn create_router_full_with_sandbox(
    supervisor: Arc<ArmSupervisor>,
    detector: proxy::DetectorUpstream,
    grasp: crate::grasp::GraspDeps,
    diagnostics: crate::diagnostics::DiagnosticsState,
    calibration: crate::calibration::CalibrationDeps,
    auth: crate::auth::OperatorAuth,
    sandbox: Arc<sandbox::SandboxState>,
) -> Router {
    let router = Router::new()
        .route("/api/v1/auth/status", get(rest::auth_status))
        .route("/api/v1/auth/unlock", post(rest::auth_unlock))
        .route("/api/v1/auth/lock", post(rest::auth_lock))
        .route("/api/v1/calibration", get(rest::get_calibration))
        .route("/api/v1/calibration/hover", get(crate::calib_hover::get_jobs))
        .route("/api/v1/calibration/hover/{id}", get(crate::calib_hover::get_job))
        .route(
            "/api/v1/calibration/sessions",
            get(crate::calibration_sessions::get_sessions)
                .post(crate::calibration_sessions::post_session)
                .layer(DefaultBodyLimit::max(crate::calibration_sessions::MAX_BODY_BYTES))
                .layer(middleware::from_fn(require_operator_for_writes)),
        )
        .route(
            "/api/v1/calibration/sessions/{id}",
            get(crate::calibration_sessions::get_session)
                .delete(crate::calibration_sessions::delete_session)
                .layer(middleware::from_fn(require_operator_for_writes)),
        )
        .route("/", get(serve_index))
        .route("/assets/jmuxer.min.js", get(serve_jmuxer))
        .route("/assets/calib_wizard.js", get(serve_calib_wizard))
        .route("/assets/three-urdf.min.js", get(assets::serve_three))
        .route("/assets/model/{*path}", get(assets::serve_model))
        .route("/assets/wasm/{*path}", get(assets::serve_wasm))
        .route("/api/v1/sandbox/catalog", get(sandbox::get_catalog))
        .route(
            "/api/v1/sandbox/scenes",
            post(sandbox::upload_scene)
                .layer(DefaultBodyLimit::max(sandbox::max_upload_body_bytes()))
                .layer(middleware::from_fn(sandbox::upload_concurrency_middleware))
                .layer(middleware::from_fn(require_operator)),
        )
        .route("/api/v1/sandbox/scenes/{id}", get(sandbox::get_scene))
        .route("/api/v1/sandbox/scenes/{id}/files/{*path}", get(sandbox::serve_scene_file))
        .route(
            "/api/v1/sandbox/tools",
            post(sandbox::upload_tool)
                .layer(DefaultBodyLimit::max(sandbox::max_upload_body_bytes()))
                .layer(middleware::from_fn(sandbox::upload_concurrency_middleware))
                .layer(middleware::from_fn(require_operator)),
        )
        .route("/api/v1/sandbox/tools/{id}", get(sandbox::get_tool))
        .route("/api/v1/sandbox/tools/{id}/files/{*path}", get(sandbox::serve_tool_file))
        .route(
            "/api/v1/sandbox/policies",
            post(sandbox::upload_policy)
                .layer(DefaultBodyLimit::max(sandbox::max_upload_body_bytes()))
                .layer(middleware::from_fn(sandbox::upload_concurrency_middleware))
                .layer(middleware::from_fn(require_operator)),
        )
        .route(
            "/api/v1/sandbox/policies/{id}",
            get(sandbox::get_policy)
                .delete(sandbox::delete_policy)
                .layer(middleware::from_fn(require_operator_for_writes)),
        )
        .route("/api/v1/sandbox/policies/{id}/files/{*path}", get(sandbox::serve_policy_file))
        .route("/api/v1/sandbox/base-assets/{*path}", get(sandbox::serve_base_asset))
        .route("/api/v1/sandbox/activate", post(sandbox::activate).layer(middleware::from_fn(require_operator)))
        .route("/api/v1/sandbox/bundle", get(sandbox::get_bundle))
        .route("/api/v1/sandbox/state", get(sandbox::get_state))
        .route("/api/v1/sandbox/reset", post(sandbox::reset).layer(middleware::from_fn(require_operator)))
        .route("/api/v1/sandbox/io", post(sandbox::set_io).layer(middleware::from_fn(require_operator)))
        .route("/api/v1/sandbox/telemetry", post(sandbox::post_telemetry).layer(middleware::from_fn(require_operator)))
        .route("/api/v1/observation", get(rest::get_observation))
        .route("/api/v1/observation/schema", get(rest::get_observation_schema))
        .route("/api/v1/torque", post(rest::set_torque))
        .route("/api/v1/arm", post(rest::set_torque))
        .route("/api/v1/heartbeat", post(rest::heartbeat))
        .route("/api/v1/motion/jog", post(rest::motion_jog))
        .route("/api/v1/motion/home", post(rest::motion_home))
        .route("/api/v1/motion/unload", post(rest::motion_unload))
        .route("/api/v1/motion/servo", post(rest::motion_servo))
        .route("/api/v1/action/move_to", post(rest::action_requires_lease))
        .route("/api/v1/action/move_relative", post(rest::action_requires_lease))
        .route("/api/v1/action/gripper", post(rest::action_requires_lease))
        .route("/api/v1/action/gripper_zero_check", post(rest::action_requires_lease))
        .route("/api/v1/action/grasp", post(rest::action_requires_lease))
        .route("/api/v1/action/move_to_pose", post(rest::action_requires_lease))
        .route("/api/v1/action/place", post(rest::action_requires_lease))
        .route("/api/v1/action/replay", post(rest::action_requires_lease))
        .route(crate::measure_multiview::PATH, post(rest::action_requires_lease))
        .route("/api/v1/mode/gravity_comp", post(rest::action_requires_lease));
    #[cfg(feature = "policy-runtime")]
    let router = router
        .route("/api/v1/action/policy/start", post(rest::action_requires_lease))
        .route("/api/v1/action/policy/stop", post(rest::action_requires_lease));
    router
        .route("/api/v1/gripper/calibration", get(rest::get_gripper_calibration))
        .route("/api/v1/grasp/setup", get(rest::get_grasp_setup))
        .route(crate::depth_measure::PATH, post(rest::post_grasp_measure).layer(middleware::from_fn(require_operator)))
        .route("/api/v1/workspace/keepout", get(rest::get_keepout))
        .route("/api/v1/safety/collision", get(rest::get_collision))
        .route("/api/v1/safety/describe", get(rest::get_safety_describe))
        .route("/api/v1/poses", get(rest::get_poses))
        .route("/api/v1/safety/e_stop", post(rest::safety_e_stop))
        .route("/api/v1/safety/clear_fault", post(rest::safety_clear_fault))
        .route("/api/v1/dataset/record/start", post(rest::dataset_record_start).layer(middleware::from_fn(require_operator)))
        .route("/api/v1/dataset/record/save", post(rest::dataset_record_save).layer(middleware::from_fn(require_operator)))
        .route("/api/v1/dataset/record/discard", post(rest::dataset_record_discard).layer(middleware::from_fn(require_operator)))
        .route("/api/v1/dataset/episodes", get(rest::dataset_episodes))
        .route("/api/v1/dataset/episodes/{id}", axum::routing::delete(rest::dataset_episode_delete).layer(middleware::from_fn(require_operator)))
        .route("/api/v1/episodes", get(rest::get_episodes))
        .route(
            "/api/v1/episodes/import",
            post(rest::episodes_import)
                .layer(DefaultBodyLimit::max(IMPORT_BODY_LIMIT))
                .layer(middleware::from_fn(require_operator)),
        )
        .route("/api/v1/episodes/{id}", get(rest::get_episode))
        .route("/api/v1/episodes/{id}/data", get(rest::get_episode_data))
        .route("/api/v1/datasets", get(rest::get_datasets))
        .route("/api/v1/sequences", get(rest::get_sequences))
        .route("/api/v1/sequences/{id}", get(rest::get_sequence))
        .route("/api/v1/action/run_sequence", post(rest::action_requires_lease))
        .route("/api/v1/diagnostics/boots", get(diagnostics::get_boots))
        .route("/api/v1/diagnostics/blackbox", get(diagnostics::get_blackbox))
        .route("/api/v1/diagnostics/tick", get(diagnostics::get_tick))
        .route("/api/v1/diagnostics/power", get(diagnostics::get_power))
        .route("/api/v1/ws", get(ws::ws_handler))
        // Single-Origin Reverse Proxy Routes
        .route("/api/v1/detection/ws", get(proxy::proxy_detection_ws))
        .route("/results", get(proxy::proxy_detection_ws))
        .route(
            "/api/v1/detection/inference",
            get(proxy::get_detection_inference)
                .post(proxy::post_detection_inference)
                .layer(middleware::from_fn(require_operator_for_writes)),
        )
        .route("/api/v1/detection/latest", get(proxy::get_detection_latest))
        .route("/api/v1/camera/snapshot", get(proxy::get_camera_snapshot))
        .route("/api/v1/detection/trigger", post(proxy::post_detection_trigger).layer(middleware::from_fn(require_operator)))
        .route(
            "/api/v1/detection/mask_focus",
            get(proxy::get_detection_mask_focus)
                .post(proxy::post_detection_mask_focus)
                .layer(middleware::from_fn(require_operator_for_writes)),
        )
        .route(
            "/api/v1/vision/mode",
            get(proxy::get_vision_mode)
                .post(proxy::post_vision_mode)
                .layer(middleware::from_fn(require_operator_for_writes)),
        )
        .route("/api/v1/vision/subscribe", post(proxy::post_vision_subscribe).layer(middleware::from_fn(require_operator)))
        .route("/api/v1/vision/subscribe/{id}", axum::routing::delete(proxy::delete_vision_subscribe).layer(middleware::from_fn(require_operator)))
        .route("/api/v1/vision/subscribe/{id}/keepalive", post(proxy::post_vision_keepalive).layer(middleware::from_fn(require_operator)))
        .route("/api/v1/vision/subscriptions", get(proxy::get_vision_subscriptions))
        .route("/api/v1/vision/depth", get(proxy::get_vision_depth).layer(middleware::from_fn(require_operator)))
        .layer(axum::Extension(detector))
        .layer(axum::Extension(proxy::VisionCache::default()))
        .layer(axum::Extension(grasp))
        .layer(axum::Extension(calibration))
        .layer(axum::Extension(diagnostics))
        .layer(axum::Extension(sandbox))
        .layer(axum::Extension(auth))
        .layer(middleware::from_fn(same_origin))
        .layer(TraceLayer::new_for_http())
        .with_state(supervisor)
}

async fn require_operator(
    axum::Extension(auth): axum::Extension<crate::auth::OperatorAuth>,
    request: Request<Body>,
    next: Next,
) -> Response {
    if auth.is_enforced() {
        let token = crate::auth::OperatorAuth::extract_token(request.headers());
        if !auth.is_operator(token.as_deref()) {
            return axum::response::IntoResponse::into_response(
                rest::ApiError::operator_required(request.uri().path()),
            );
        }
    }
    next.run(request).await
}

async fn require_operator_for_writes(
    axum::Extension(auth): axum::Extension<crate::auth::OperatorAuth>,
    request: Request<Body>,
    next: Next,
) -> Response {
    let is_read = matches!(
        *request.method(),
        axum::http::Method::GET | axum::http::Method::HEAD | axum::http::Method::OPTIONS
    );
    if !is_read && auth.is_enforced() {
        let token = crate::auth::OperatorAuth::extract_token(request.headers());
        if !auth.is_operator(token.as_deref()) {
            return axum::response::IntoResponse::into_response(
                rest::ApiError::operator_required(request.uri().path()),
            );
        }
    }
    next.run(request).await
}

// Browser origin protection is not network authentication. Non-browser clients
// remain suitable only for a trusted network or an authenticated reverse proxy.
async fn same_origin(request: Request<Body>, next: Next) -> Response {
    let protected = request.method() != axum::http::Method::GET
        || request.headers().get("upgrade").and_then(|h| h.to_str().ok()).is_some_and(|h| h.eq_ignore_ascii_case("websocket"));
    if protected {
        if let Some(origin) = request.headers().get("origin") {
            let host = request.headers().get("host").and_then(|h| h.to_str().ok());
            let valid = origin.to_str().ok().and_then(|o| o.parse::<axum::http::Uri>().ok()).is_some_and(|uri| {
                matches!(uri.scheme_str(), Some("http" | "https"))
                    && uri.authority().map(|a| a.as_str()) == host
                    && uri.path_and_query().is_none_or(|p| p.as_str() == "/")
            });
            if !valid {
                return axum::response::IntoResponse::into_response((StatusCode::FORBIDDEN, "Cross-origin control requests are not allowed"));
            }
        }
    }
    next.run(request).await
}
