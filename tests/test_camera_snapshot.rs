//! GET /api/v1/camera/snapshot: rebot-vision's /snapshot.jpg (video server port) from armd's origin.
use std::sync::Arc;

use armd::api::create_router_with_detector;
use armd::api::proxy::DetectorUpstream;
use armd::{ArmConfig, ArmSupervisor};

const JPEG: &[u8] = include_bytes!("fixtures/calib_t20/f03.jpg");

#[tokio::test]
async fn snapshot_is_passed_through_from_the_video_port() {
    let app = axum::Router::new().route("/snapshot.jpg", axum::routing::get(|| async { ([(axum::http::header::CONTENT_TYPE, "image/jpeg")], JPEG) }));
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    tokio::spawn(async move { axum::serve(listener, app).await.unwrap(); });
    // the only test of this binary: the env var does not race another test
    unsafe { std::env::set_var("ARMD_SNAPSHOT_PORT", port.to_string()) };
    let router = create_router_with_detector(Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm())), DetectorUpstream::parse("http://127.0.0.1:9").unwrap(), armd::OperatorAuth::disabled());
    let l2 = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = l2.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(l2, router).await.unwrap(); });
    let r = reqwest::get(format!("http://{addr}/api/v1/camera/snapshot")).await.unwrap();
    assert_eq!(r.status(), 200);
    assert_eq!(r.headers()["content-type"], "image/jpeg");
    assert_eq!(r.headers()["cache-control"], "no-store");
    assert_eq!(r.bytes().await.unwrap().as_ref(), JPEG);
    // video server down: 502 like the other detector routes
    unsafe { std::env::set_var("ARMD_SNAPSHOT_PORT", "9") };
    let r = reqwest::get(format!("http://{addr}/api/v1/camera/snapshot")).await.unwrap();
    assert_eq!(r.status(), 502);
}
