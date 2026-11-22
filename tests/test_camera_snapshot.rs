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

#[tokio::test]
async fn http_detection_source_gray_negotiation_and_strict_fetch() {
    use armd::grasp::{DetectionSource, GrayFrameError, HttpDetectionSource};
    use axum::extract::Query;
    use axum::http::{header, StatusCode};
    use axum::response::IntoResponse;
    use serde_json::json;
    use std::collections::HashMap;

    const GOLDEN_RGF1: &[u8] = include_bytes!("fixtures/gray_frame_golden.rgf1");

    let app = axum::Router::new()
        .route("/api/mode", axum::routing::get(|| async {
            axum::Json(json!({
                "api_version": 2, "mode": "yolo", "enabled": false, "per_request_pipelines": true,
                "available": {"yolo": true, "aruco": true, "depth": false, "gray": true},
                "gray_frames": {"encoding": "RGF1", "format": "gray8", "width": 640, "height": 360, "stride": 640, "ring": 4, "exposure_qualified": false}
            }))
        }))
        .route("/api/trigger", axum::routing::post(|axum::Json(body): axum::Json<serde_json::Value>| async move {
            let pipes = body["pipelines"].as_array().cloned().unwrap_or_default();
            if pipes.iter().any(|p| p == "unknown_old") {
                return (StatusCode::BAD_REQUEST, axum::Json(json!({"error": "bad pipelines"}))).into_response();
            }
            axum::Json(json!({
                "api_version": 2, "success": true, "instance_id": "rv-golden-0001", "frame_id": 7,
                "width": 640, "height": 360, "age_ms": 5,
                "pipelines": {"yolo": {"ran": true, "ms": 40.0}, "aruco": {"ran": false, "ms": 0.0}, "depth": {"ran": false, "ms": 0.0}, "gray": {"ran": true, "ms": 4.9}},
                "detections": [], "markers": []
            })).into_response()
        }))
        .route("/api/frame/gray", axum::routing::get(|Query(q): Query<HashMap<String, String>>| async move {
            let inst = q.get("instance_id").map(String::as_str).unwrap_or("");
            let fid = q.get("frame_id").map(String::as_str).unwrap_or("");
            match (inst, fid) {
                ("rv-golden-0001", "7") => (StatusCode::OK, [(header::CONTENT_TYPE, "application/octet-stream")], GOLDEN_RGF1).into_response(),
                ("rv-golden-0001", "8") => (StatusCode::OK, [(header::CONTENT_TYPE, "application/json")], GOLDEN_RGF1).into_response(),
                ("rv-golden-0001", "9") => (StatusCode::OK, [(header::CONTENT_TYPE, "application/octet-stream")], GOLDEN_RGF1).into_response(),
                ("rv-golden-0001", "404") => (StatusCode::NOT_FOUND, axum::Json(json!({"api_version": 2, "error": "FRAME_NOT_STORED"}))).into_response(),
                ("rv-golden-0001", "410") => (StatusCode::GONE, axum::Json(json!({"api_version": 2, "error": "FRAME_EXPIRED"}))).into_response(),
                ("rv-golden-0001", "403") => (StatusCode::FORBIDDEN, axum::Json(json!({"api_version": 2, "error": "FORBIDDEN"}))).into_response(),
                ("rv-golden-0001", "503") => (StatusCode::SERVICE_UNAVAILABLE, axum::Json(json!({"api_version": 2, "error": "GRAY_BUSY"}))).into_response(),
                ("rv-golden-0001", "999") => {
                    tokio::time::sleep(std::time::Duration::from_millis(2300)).await;
                    (StatusCode::OK, [(header::CONTENT_TYPE, "application/octet-stream")], GOLDEN_RGF1).into_response()
                }
                _ => (StatusCode::BAD_REQUEST, axum::Json(json!({"api_version": 2, "error": "INVALID_PARAM"}))).into_response(),
            }
        }));

    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, app).await.unwrap(); });

    let src = HttpDetectionSource {
        upstream: DetectorUpstream::parse(&format!("http://{addr}")).unwrap(),
        image_size_px: [640.0, 360.0],
    };

    let mode = src.mode_info().await.unwrap();
    assert!(mode.per_request_pipelines && mode.aruco_available && mode.gray_available && mode.upstream_loopback);
    assert_eq!(mode.gray_format.as_ref().ok().and_then(Option::as_ref).map(|g| (g.width, g.height, g.stride, g.ring)), Some((640, 360, 640, 4)));

    let trig = src.trigger_gray(&["yolo", "gray"], 1000).await.unwrap().expect("trigger result");
    assert_eq!(trig.instance_id.as_deref(), Some("rv-golden-0001"));
    assert_eq!(trig.frame.frame_id, 7);
    assert!(trig.gray_ran);

    // Valid golden RGF1 fetch
    let g = src.gray_frame("rv-golden-0001", 7).await.unwrap();
    assert_eq!((g.img.w, g.img.h, g.frame_id), (640, 360, 7));
    assert_eq!(g.img.px[0], 17);

    // Wrong Content-Type rejected before decoding
    assert!(matches!(src.gray_frame("rv-golden-0001", 8).await, Err(GrayFrameError::Protocol(m)) if m.contains("Content-Type")));
    // Identity mismatch (requested frame 9, server returned golden frame 7)
    assert!(matches!(src.gray_frame("rv-golden-0001", 9).await, Err(GrayFrameError::Protocol(m)) if m.contains("frame_id mismatch")));
    // HTTP error codes mapped cleanly
    assert!(matches!(src.gray_frame("rv-golden-0001", 404).await, Err(GrayFrameError::NotStored(m)) if m.contains("FRAME_NOT_STORED")));
    assert!(matches!(src.gray_frame("rv-golden-0001", 410).await, Err(GrayFrameError::Expired(m)) if m.contains("FRAME_EXPIRED")));
    assert!(matches!(src.gray_frame("rv-golden-0001", 403).await, Err(GrayFrameError::Forbidden(m)) if m.contains("FORBIDDEN")));
    assert!(matches!(src.gray_frame("rv-golden-0001", 503).await, Err(GrayFrameError::Busy(m)) if m.contains("GRAY_BUSY")));
    // Upstream timeout
    assert!(matches!(src.gray_frame("rv-golden-0001", 999).await, Err(GrayFrameError::Transport(m)) if m.contains("timed out")));
    // Older provider rejecting unknown pipeline
    assert!(src.trigger_gray(&["unknown_old"], 1000).await.unwrap_err().contains("400"));

    // IPv6 loopback URL ([::1]:port) connects without brackets in the resolver and sends bracketed Host
    let v6_listener = tokio::net::TcpListener::bind("[::1]:0").await.expect("bind IPv6 loopback");
    let v6_port = v6_listener.local_addr().unwrap().port();
    let v6_app = axum::Router::new()
        .route("/api/mode", axum::routing::get(move |headers: axum::http::HeaderMap| async move {
            let expected_host = format!("[::1]:{v6_port}");
            assert_eq!(headers.get(header::HOST).and_then(|v| v.to_str().ok()), Some(expected_host.as_str()));
            axum::Json(json!({
                "api_version": 2, "mode": "yolo", "enabled": false, "per_request_pipelines": true,
                "available": {"yolo": true, "aruco": true, "depth": false, "gray": true},
                "gray_frames": {"encoding": "RGF1", "format": "gray8", "width": 640, "height": 360, "stride": 640, "ring": 4, "exposure_qualified": false}
            }))
        }))
        .route("/api/frame/gray", axum::routing::get(|| async {
            (StatusCode::OK, [(header::CONTENT_TYPE, "application/octet-stream")], GOLDEN_RGF1)
        }));
    tokio::spawn(async move { axum::serve(v6_listener, v6_app).await.unwrap(); });
    let up = DetectorUpstream::parse(&format!("http://[::1]:{v6_port}")).unwrap();
    assert!(up.is_loopback());
    let v6_src = HttpDetectionSource { upstream: up, image_size_px: [640.0, 360.0] };
    let v6_mode = v6_src.mode_info().await.expect("IPv6 loopback /api/mode connect");
    assert!(v6_mode.gray_available && v6_mode.upstream_loopback);
    let v6_gray = v6_src.gray_frame("rv-golden-0001", 7).await.expect("IPv6 loopback gray fetch");
    assert_eq!((v6_gray.img.w, v6_gray.img.h), (640, 360));
    assert!(v6_gray.fetch_ms >= 0.0 && v6_gray.fetch_ms.is_finite());
}
