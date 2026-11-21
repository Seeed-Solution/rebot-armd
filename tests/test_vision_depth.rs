//! GET /api/v1/vision/depth -> rebot-vision GET /api/depth: binary pass-through, operator
//! session required, upstream errors and 429 + Retry-After passed through, 502 / 504.
use std::net::SocketAddr;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpListener;

use armd::api::create_router_with_detector;
use armd::api::proxy::DetectorUpstream;
use armd::{ArmConfig, ArmSupervisor, OperatorAuth};

/// A small "RDM1" body: magic, u32 LE JSON length, JSON padded to 8, 4 x 2 u16 LE mm.
fn depth_body() -> Vec<u8> {
    let mut js = br#"{"format":"u16le_mm","width":4,"height":2,"frame_id":7}"#.to_vec();
    while !(8 + js.len()).is_multiple_of(8) {
        js.push(b' ');
    }
    let mut out = b"RDM1".to_vec();
    out.extend_from_slice(&(js.len() as u32).to_le_bytes());
    out.extend_from_slice(&js);
    for mm in [300u16, 400, 500, 600, 0, 65535, 1, 256] {
        out.extend_from_slice(&mm.to_le_bytes());
    }
    out
}

/// One canned upstream reply per mode, chosen by `mode`.
#[derive(Clone, Copy)]
enum Mode {
    Map,
    RateLimited,
    Busy,
    Slow,
}

async fn spawn_upstream(mode: Arc<Mutex<Mode>>) -> (SocketAddr, Arc<Mutex<Vec<String>>>) {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let seen = Arc::new(Mutex::new(Vec::new()));
    let log = seen.clone();
    tokio::spawn(async move {
        loop {
            let (mut sock, _) = listener.accept().await.unwrap();
            let log = log.clone();
            let mode = *mode.lock().unwrap();
            tokio::spawn(async move {
                let mut raw = Vec::new();
                let mut buf = [0u8; 4096];
                while !raw.windows(4).any(|w| w == b"\r\n\r\n") {
                    let n = sock.read(&mut buf).await.unwrap();
                    if n == 0 {
                        return;
                    }
                    raw.extend_from_slice(&buf[..n]);
                }
                let head = String::from_utf8_lossy(&raw).to_string();
                log.lock().unwrap().push(head.lines().next().unwrap().to_string());
                let (status, ct, extra, body): (&str, &str, &str, Vec<u8>) = match mode {
                    Mode::Map => ("200 OK", "application/octet-stream", "", depth_body()),
                    Mode::RateLimited => (
                        "429 Too Many Requests",
                        "application/json",
                        "Retry-After: 1\r\n",
                        br#"{"api_version":2,"error":"depth rate limited","retry_after_ms":640}"#.to_vec(),
                    ),
                    Mode::Busy => ("409 Conflict", "application/json", "", br#"{"api_version":2,"error":"depth busy"}"#.to_vec()),
                    Mode::Slow => {
                        tokio::time::sleep(Duration::from_secs(8)).await;
                        ("200 OK", "application/octet-stream", "", depth_body())
                    }
                };
                let head = format!(
                    "HTTP/1.1 {status}\r\nContent-Type: {ct}\r\nContent-Length: {}\r\n{extra}Connection: close\r\n\r\n",
                    body.len()
                );
                let _ = sock.write_all(head.as_bytes()).await;
                let _ = sock.write_all(&body).await;
            });
        }
    });
    (addr, seen)
}

async fn spawn_armd(detector: &str, auth: OperatorAuth) -> SocketAddr {
    let mut config = ArmConfig::default();
    config.dof = 2;
    config.joint_names = vec!["joint_1".into(), "joint_2".into()];
    config.limits = vec![
        armd::JointLimit::new("joint_1", -1.0, 1.0, 2.0, 5.0),
        armd::JointLimit::new("joint_2", -2.0, 2.0, 2.0, 5.0),
    ];
    config.home_positions = vec![0.0, 0.0];
    let router = create_router_with_detector(Arc::new(ArmSupervisor::new(config)), DetectorUpstream::parse(detector).unwrap(), auth);
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap() });
    addr
}

#[tokio::test]
async fn depth_map_passes_through_binary() {
    let mode = Arc::new(Mutex::new(Mode::Map));
    let (up, seen) = spawn_upstream(mode.clone()).await;
    let addr = spawn_armd(&format!("http://{up}"), OperatorAuth::disabled()).await;
    let r = reqwest::get(format!("http://{addr}/api/v1/vision/depth")).await.unwrap();
    assert_eq!(r.status(), 200);
    assert_eq!(r.headers()["content-type"], "application/octet-stream");
    assert_eq!(r.headers()["cache-control"], "no-store");
    let body = r.bytes().await.unwrap();
    assert_eq!(body.as_ref(), depth_body().as_slice());
    assert_eq!(seen.lock().unwrap().as_slice(), ["GET /api/depth?wait_ms=4500 HTTP/1.1"]);
}

#[tokio::test]
async fn depth_map_requires_an_operator_session() {
    let mode = Arc::new(Mutex::new(Mode::Map));
    let (up, seen) = spawn_upstream(mode).await;
    let addr = spawn_armd(&format!("http://{up}"), OperatorAuth::enforced("123456", 1800)).await;
    let c = reqwest::Client::new();
    let r = c.get(format!("http://{addr}/api/v1/vision/depth")).send().await.unwrap();
    assert_eq!(r.status(), 403);
    let v: serde_json::Value = r.json().await.unwrap();
    assert_eq!(v["code"], "OPERATOR_REQUIRED", "{v}");
    assert!(seen.lock().unwrap().is_empty(), "a locked request reached the detector");

    let unlock: serde_json::Value = c
        .post(format!("http://{addr}/api/v1/auth/unlock"))
        .json(&serde_json::json!({"pin": "123456"}))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let tok = unlock["token"].as_str().unwrap();
    let r = c.get(format!("http://{addr}/api/v1/vision/depth")).bearer_auth(tok).send().await.unwrap();
    assert_eq!(r.status(), 200);
    assert_eq!(r.bytes().await.unwrap().as_ref(), depth_body().as_slice());
    assert_eq!(seen.lock().unwrap().len(), 1);
}

#[tokio::test]
async fn depth_map_upstream_errors_pass_through() {
    let mode = Arc::new(Mutex::new(Mode::RateLimited));
    let (up, _) = spawn_upstream(mode.clone()).await;
    let addr = spawn_armd(&format!("http://{up}"), OperatorAuth::disabled()).await;
    let url = format!("http://{addr}/api/v1/vision/depth");

    let r = reqwest::get(&url).await.unwrap();
    assert_eq!(r.status(), 429);
    assert_eq!(r.headers()["retry-after"], "1");
    let v: serde_json::Value = r.json().await.unwrap();
    assert_eq!(v["error"], "depth rate limited");
    assert_eq!(v["retry_after_ms"], 640);

    *mode.lock().unwrap() = Mode::Busy;
    let r = reqwest::get(&url).await.unwrap();
    assert_eq!(r.status(), 409);
    assert!(r.headers().get("retry-after").is_none());
    assert_eq!(r.json::<serde_json::Value>().await.unwrap()["error"], "depth busy");
}

#[tokio::test]
async fn depth_map_times_out_at_6_s_and_502_when_down() {
    let mode = Arc::new(Mutex::new(Mode::Slow));
    let (up, _) = spawn_upstream(mode).await;
    let addr = spawn_armd(&format!("http://{up}"), OperatorAuth::disabled()).await;
    let t0 = Instant::now();
    let r = reqwest::get(format!("http://{addr}/api/v1/vision/depth")).await.unwrap();
    let dt = t0.elapsed();
    assert_eq!(r.status(), 504);
    assert_eq!(r.json::<serde_json::Value>().await.unwrap(), serde_json::json!({"error": "detector timed out"}));
    assert!(dt >= Duration::from_millis(5900) && dt < Duration::from_millis(7000), "timeout {dt:?}");

    let port = TcpListener::bind("127.0.0.1:0").await.unwrap().local_addr().unwrap().port();
    let addr = spawn_armd(&format!("http://127.0.0.1:{port}"), OperatorAuth::disabled()).await;
    let r = reqwest::get(format!("http://{addr}/api/v1/vision/depth")).await.unwrap();
    assert_eq!(r.status(), 502);
    assert_eq!(r.json::<serde_json::Value>().await.unwrap(), serde_json::json!({"error": "detector unreachable"}));
}

#[tokio::test]
async fn depth_map_old_detector_404_passes_through() {
    // rebot-vision without the route answers 404 JSON
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let old = listener.local_addr().unwrap();
    tokio::spawn(async move {
        loop {
            let (mut sock, _) = listener.accept().await.unwrap();
            tokio::spawn(async move {
                let mut buf = [0u8; 4096];
                let _ = sock.read(&mut buf).await;
                let body = r#"{"error":"not found"}"#;
                let _ = sock
                    .write_all(format!("HTTP/1.1 404 Not Found\r\nContent-Type: application/json\r\nContent-Length: {}\r\n\r\n{body}", body.len()).as_bytes())
                    .await;
            });
        }
    });
    let addr = spawn_armd(&format!("http://{old}"), OperatorAuth::disabled()).await;
    let r = reqwest::get(format!("http://{addr}/api/v1/vision/depth")).await.unwrap();
    assert_eq!(r.status(), 404);
    assert_eq!(r.json::<serde_json::Value>().await.unwrap(), serde_json::json!({"error": "not found"}));
}
