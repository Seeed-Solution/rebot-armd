pub mod assets;
pub mod diagnostics;
pub mod proxy;
pub mod rest;
pub mod routes;
pub mod sandbox;
pub mod ws;

use std::sync::Arc;
use axum::{
    body::Body,
    http::{Request, StatusCode},
    middleware::{self, Next},
    response::{Html, Response},
    Router,
};
use tower_http::trace::TraceLayer;

use crate::supervisor::ArmSupervisor;

/// Largest `POST /api/v1/episodes/import` body (a 600 s episode at the control rate is a few MB).
pub const IMPORT_BODY_LIMIT: usize = 64 * 1024 * 1024;

// The cockpit page assembled by build.rs from src/api/index.html and src/api/cockpit/ fragments.
static INDEX_HTML: &str = include_str!(concat!(env!("OUT_DIR"), "/cockpit/index.html"));

pub(super) async fn serve_index() -> Html<&'static str> {
    Html(INDEX_HTML)
}

pub(super) async fn serve_favicon() -> impl axum::response::IntoResponse {
    (
        StatusCode::NO_CONTENT,
        [(axum::http::header::CACHE_CONTROL, "public, max-age=86400")],
    )
}

// JMuxer 2.1.0 (MIT), vendored; plays the camera's H.264-over-WebSocket stream.
static JMUXER_JS: &str = include_str!("vendor/jmuxer.min.js");

pub(super) async fn serve_jmuxer() -> impl axum::response::IntoResponse {
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

pub(super) async fn serve_calib_wizard() -> impl axum::response::IntoResponse {
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
    ws::prewarm_dispatch_tables();
    let router = routes::build_rest_router();
    #[cfg(feature = "policy-runtime")]
    let router = router.layer(axum::Extension(
        crate::policy_runtime::vision::embedding_hub_for_supervisor(&supervisor),
    ));
    router
        .layer(axum::Extension(detector))
        .layer(axum::Extension(proxy::VisionCache::default()))
        .layer(axum::Extension(grasp))
        .layer(axum::Extension(calibration))
        .layer(axum::Extension(diagnostics))
        .layer(axum::Extension(sandbox))
        .layer(axum::Extension(auth.clone()))
        .layer(middleware::from_fn_with_state(supervisor.clone(), routes::dispatch_rest_gate))
        .layer(middleware::from_fn_with_state(auth, same_origin))
        .layer(TraceLayer::new_for_http())
        .with_state(supervisor)
}

pub(super) async fn require_operator(
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

// Browser origin protection is not network authentication. Non-browser clients
// remain suitable only for a trusted network or an authenticated reverse proxy.
async fn same_origin(
    axum::extract::State(auth): axum::extract::State<crate::auth::OperatorAuth>,
    request: Request<Body>,
    next: Next,
) -> Response {
    let is_mutating = !matches!(
        *request.method(),
        axum::http::Method::GET | axum::http::Method::HEAD | axum::http::Method::OPTIONS
    );
    let is_ws_upgrade = request
        .headers()
        .get(axum::http::header::UPGRADE)
        .and_then(|h| h.to_str().ok())
        .is_some_and(|h| h.eq_ignore_ascii_case("websocket"));
    if (is_mutating || is_ws_upgrade) && !auth.validate_origin(request.headers()) {
        return axum::response::IntoResponse::into_response((
            StatusCode::FORBIDDEN,
            "Cross-origin control requests are not allowed",
        ));
    }
    next.run(request).await
}

#[cfg(feature = "policy-runtime")]
fn is_non_loopback_forwarded_addr(raw: &str) -> bool {
    let ip_str = raw
        .strip_prefix('[')
        .and_then(|s| s.split(']').next())
        .unwrap_or_else(|| {
            if raw.matches(':').count() == 1 {
                raw.split(':').next().unwrap_or(raw)
            } else {
                raw
            }
        });
    !ip_str
        .parse::<std::net::IpAddr>()
        .is_ok_and(|ip| ip.is_loopback())
}

#[cfg(feature = "policy-runtime")]
fn header_forwarded_non_loopback(headers: &axum::http::HeaderMap) -> bool {
    if let Some(xff) = headers.get("x-forwarded-for").and_then(|v| v.to_str().ok()) {
        if xff
            .split(',')
            .map(str::trim)
            .filter(|s| !s.is_empty())
            .any(is_non_loopback_forwarded_addr)
        {
            return true;
        }
    }
    let Some(fwd) = headers
        .get(axum::http::header::FORWARDED)
        .and_then(|v| v.to_str().ok())
    else {
        return false;
    };
    for dir in fwd.split(',').flat_map(|elem| elem.split(';')) {
        let Some((k, v)) = dir.trim().split_once('=') else {
            continue;
        };
        if k.trim().eq_ignore_ascii_case("for")
            && is_non_loopback_forwarded_addr(v.trim().trim_matches('"'))
        {
            return true;
        }
    }
    false
}

#[cfg(feature = "policy-runtime")]
fn infer_same_host(
    peer_addr: Option<std::net::SocketAddr>,
    headers: &axum::http::HeaderMap,
) -> bool {
    let Some(addr) = peer_addr else {
        return false;
    };
    if !addr.ip().is_loopback() {
        return false;
    }
    !header_forwarded_non_loopback(headers)
}

#[cfg(feature = "policy-runtime")]
struct OptionalPeerAddr(Option<std::net::SocketAddr>);

#[cfg(feature = "policy-runtime")]
impl<S> axum::extract::FromRequestParts<S> for OptionalPeerAddr
where
    S: Send + Sync,
{
    type Rejection = std::convert::Infallible;

    async fn from_request_parts(
        parts: &mut axum::http::request::Parts,
        _state: &S,
    ) -> Result<Self, Self::Rejection> {
        let addr = parts
            .extensions
            .get::<axum::extract::ConnectInfo<std::net::SocketAddr>>()
            .map(|ci| ci.0);
        Ok(Self(addr))
    }
}

#[cfg(feature = "policy-runtime")]
fn embedding_err_to_api(e: crate::policy_runtime::vision::EmbeddingIngestError) -> rest::ApiError {
    let status = StatusCode::from_u16(e.status).unwrap_or(StatusCode::BAD_REQUEST);
    rest::ApiError(status, e.message, e.code.to_string())
}

#[cfg(feature = "policy-runtime")]
async fn post_vision_clock_handshake(
    axum::Extension(hub): axum::Extension<
        Arc<crate::policy_runtime::vision::VideoEmbeddingHub>,
    >,
    OptionalPeerAddr(peer_addr): OptionalPeerAddr,
    headers: axum::http::HeaderMap,
    axum::Json(req): axum::Json<crate::policy_runtime::vision::ClockHandshakeRequest>,
) -> Result<
    axum::Json<crate::policy_runtime::vision::ClockHandshakeResponse>,
    rest::ApiError,
> {
    let socket_same_host = infer_same_host(peer_addr, &headers);
    hub.clock_handshake(req, socket_same_host)
        .map(axum::Json)
        .map_err(embedding_err_to_api)
}

#[cfg(feature = "policy-runtime")]
async fn post_vision_embedding(
    axum::Extension(hub): axum::Extension<
        Arc<crate::policy_runtime::vision::VideoEmbeddingHub>,
    >,
    axum::Json(req): axum::Json<crate::policy_runtime::vision::EmbeddingIngestRequest>,
) -> Result<
    axum::Json<crate::policy_runtime::vision::EmbeddingIngestResponse>,
    rest::ApiError,
> {
    hub.ingest_embedding(req)
        .map(axum::Json)
        .map_err(embedding_err_to_api)
}

#[cfg(all(test, feature = "policy-runtime"))]
mod tests {
    use super::infer_same_host;
    use axum::http::{header, HeaderMap, HeaderValue};
    use std::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr};

    #[test]
    fn infer_same_host_requires_loopback_peer_and_rejects_forwarded_or_missing_peer() {
        let mut localhost_host_hdr = HeaderMap::new();
        localhost_host_hdr.insert(header::HOST, HeaderValue::from_static("localhost:8765"));
        assert!(!infer_same_host(None, &localhost_host_hdr));

        let lo_v4 = Some(SocketAddr::new(IpAddr::V4(Ipv4Addr::LOCALHOST), 50000));
        let lo_v6 = Some(SocketAddr::new(IpAddr::V6(Ipv6Addr::LOCALHOST), 50000));
        let empty = HeaderMap::new();
        assert!(infer_same_host(lo_v4, &empty));
        assert!(infer_same_host(lo_v6, &empty));

        let mut xff = HeaderMap::new();
        xff.insert("x-forwarded-for", HeaderValue::from_static("192.168.1.50"));
        assert!(!infer_same_host(lo_v4, &xff));

        let remote = Some(SocketAddr::new(IpAddr::V4(Ipv4Addr::new(192, 168, 1, 20)), 50000));
        assert!(!infer_same_host(remote, &empty));
        assert!(!infer_same_host(remote, &localhost_host_hdr));
    }
}

#[cfg(test)]
mod static_tests {
    use super::{serve_favicon, serve_index};
    use axum::{body::Body, http::{Request, StatusCode}, routing::get, Router};
    use tower::ServiceExt;

    #[tokio::test]
    async fn index_and_favicon_do_not_404() {
        let app = Router::new()
            .route("/", get(serve_index))
            .route("/favicon.ico", get(serve_favicon));

        let idx = app
            .clone()
            .oneshot(Request::get("/").body(Body::empty()).unwrap())
            .await
            .unwrap();
        assert_eq!(idx.status(), StatusCode::OK);
        let body = axum::body::to_bytes(idx.into_body(), usize::MAX).await.unwrap();
        let html = std::str::from_utf8(&body).unwrap();
        assert!(html.contains(r#"<link rel="icon""#), "index.html must declare an inline favicon");

        let fav = app
            .oneshot(Request::get("/favicon.ico").body(Body::empty()).unwrap())
            .await
            .unwrap();
        assert_eq!(fav.status(), StatusCode::NO_CONTENT);
    }
}
