//! Detector control API proxy: /api/v1/detection/{inference,latest} -> mock upstream.
use std::net::SocketAddr;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpListener;

use armd::api::create_router_with_detector;
use armd::api::proxy::DetectorUpstream;
use armd::{ArmConfig, ArmSupervisor};

#[derive(Clone, Debug)]
struct SeenRequest {
    method: String,
    target: String,
    body: String,
}

/// Raw-TCP mock of the detector control API. Behaviour by path:
/// - GET/POST /api/inference: JSON state; POST with a body lacking `enabled` -> 400
/// - GET /api/results/latest: sleeps `wait_ms`/10 ms (chunked reply), or 3 s with `slow=1`
/// - POST /api/trigger: sleeps `wait_ms`/10 ms (default wait 2500), or 3 s with `slow=1`;
///   `miss=1` -> `{"success":false,"timeout":true}`. Any other path (e.g. under a base path) -> 404
async fn spawn_mock_upstream() -> (SocketAddr, Arc<Mutex<Vec<SeenRequest>>>) {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let seen = Arc::new(Mutex::new(Vec::new()));
    let enabled = Arc::new(Mutex::new(false));
    let log = seen.clone();
    tokio::spawn(async move {
        loop {
            let (mut sock, _) = listener.accept().await.unwrap();
            let log = log.clone();
            let enabled = enabled.clone();
            tokio::spawn(async move {
                let mut raw = Vec::new();
                let mut buf = [0u8; 4096];
                let (head, mut body) = loop {
                    let n = sock.read(&mut buf).await.unwrap();
                    assert!(n > 0, "client closed before headers");
                    raw.extend_from_slice(&buf[..n]);
                    if let Some(i) = raw.windows(4).position(|w| w == b"\r\n\r\n") {
                        break (String::from_utf8(raw[..i].to_vec()).unwrap(), raw[i + 4..].to_vec());
                    }
                };
                let len = head
                    .lines()
                    .find_map(|l| l.to_ascii_lowercase().strip_prefix("content-length:").map(|v| v.trim().parse::<usize>().unwrap()))
                    .unwrap_or(0);
                while body.len() < len {
                    let n = sock.read(&mut buf).await.unwrap();
                    body.extend_from_slice(&buf[..n]);
                }
                let mut rl = head.lines().next().unwrap().split(' ');
                let req = SeenRequest {
                    method: rl.next().unwrap().to_string(),
                    target: rl.next().unwrap().to_string(),
                    body: String::from_utf8(body).unwrap(),
                };
                log.lock().unwrap().push(req.clone());
                let path = req.target.split('?').next().unwrap();
                let query = req.target.split_once('?').map(|(_, q)| q).unwrap_or("");
                let state = |on: bool| serde_json::json!({"enabled": on, "fps": if on {10} else {1}, "active_fps": 10, "idle_fps": 1}).to_string();
                let (status, payload, chunked) = match (req.method.as_str(), path) {
                    ("GET", "/api/inference") => ("200 OK", state(*enabled.lock().unwrap()), false),
                    ("POST", "/api/inference") => match serde_json::from_str::<serde_json::Value>(&req.body).ok().and_then(|v| v["enabled"].as_bool()) {
                        Some(on) => {
                            *enabled.lock().unwrap() = on;
                            ("200 OK", state(on), false)
                        }
                        None => ("400 Bad Request", r#"{"error":"expected {\"enabled\":bool}"}"#.to_string(), false),
                    },
                    ("GET", "/api/results/latest") => {
                        let wait = query.split('&').find_map(|kv| kv.strip_prefix("wait_ms=")).map(|v| v.parse::<u64>().unwrap()).unwrap_or(0);
                        let delay = if query.contains("slow=1") { 3000 } else { wait / 10 };
                        tokio::time::sleep(Duration::from_millis(delay)).await;
                        let body = serde_json::json!({"enabled": true, "frame_id": 7, "ts_ms": 1, "age_ms": 2, "inference_time_ms": 30,
                            "detections": [{"label": "cup", "score": 0.9, "bbox": [1, 2, 3, 4]}]});
                        ("200 OK", body.to_string(), true)
                    }
                    ("POST", "/api/trigger") => {
                        let wait = query.split('&').find_map(|kv| kv.strip_prefix("wait_ms=")).map(|v| v.parse::<u64>().unwrap()).unwrap_or(2500);
                        let delay = if query.contains("slow=1") { 3000 } else { wait / 10 };
                        tokio::time::sleep(Duration::from_millis(delay)).await;
                        let body = if query.contains("miss=1") {
                            serde_json::json!({"success": false, "timeout": true, "count": 0, "markers": []})
                        } else {
                            serde_json::json!({"success": true, "frame_id": 9, "ts_ms": 1, "age_ms": 3, "inference_time_ms": 12.5, "count": 1,
                                "markers": [{"id": 4, "corners": [[1, 2], [3, 2], [3, 4], [1, 4]]}]})
                        };
                        ("200 OK", body.to_string(), false)
                    }
                    _ => ("404 Not Found", r#"{"error":"not found"}"#.to_string(), false),
                };
                let resp = if chunked {
                    let (a, b) = payload.split_at(payload.len() / 2);
                    format!("HTTP/1.1 {status}\r\nContent-Type: application/json\r\nTransfer-Encoding: chunked\r\n\r\n{:x}\r\n{a}\r\n{:x}\r\n{b}\r\n0\r\n\r\n", a.len(), b.len())
                } else {
                    format!("HTTP/1.1 {status}\r\nContent-Type: application/json\r\nContent-Length: {}\r\n\r\n{payload}", payload.len())
                };
                let _ = sock.write_all(resp.as_bytes()).await;
            });
        }
    });
    (addr, seen)
}

async fn spawn_armd(detector: &str) -> SocketAddr {
    let mut config = ArmConfig::default();
    config.dof = 2;
    config.joint_names = vec!["joint_1".into(), "joint_2".into()];
    config.limits = vec![
        armd::JointLimit::new("joint_1", -1.0, 1.0, 2.0, 5.0),
        armd::JointLimit::new("joint_2", -2.0, 2.0, 2.0, 5.0),
    ];
    config.home_positions = vec![0.0, 0.0];
    let router = create_router_with_detector(Arc::new(ArmSupervisor::new(config)), DetectorUpstream::parse(detector).unwrap());
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap() });
    addr
}

#[tokio::test]
async fn inference_get_and_post_pass_through() {
    let (up, seen) = spawn_mock_upstream().await;
    let addr = spawn_armd(&format!("http://{up}")).await;
    let c = reqwest::Client::new();
    let url = format!("http://{addr}/api/v1/detection/inference");

    let r = c.get(&url).send().await.unwrap();
    assert_eq!(r.status(), 200);
    assert_eq!(r.json::<serde_json::Value>().await.unwrap(), serde_json::json!({"enabled": false, "fps": 1, "active_fps": 10, "idle_fps": 1}));

    let r = c.post(&url).json(&serde_json::json!({"enabled": true})).send().await.unwrap();
    assert_eq!(r.status(), 200);
    assert_eq!(r.json::<serde_json::Value>().await.unwrap()["enabled"], true);
    let r = c.get(&url).send().await.unwrap();
    assert_eq!(r.json::<serde_json::Value>().await.unwrap()["fps"], 10);

    // Upstream 400 and its JSON body are passed through unchanged.
    let r = c.post(&url).header("content-type", "application/json").body("{\"on\":1}").send().await.unwrap();
    assert_eq!(r.status(), 400);
    assert!(r.json::<serde_json::Value>().await.unwrap()["error"].is_string());

    let seen = seen.lock().unwrap().clone();
    assert_eq!(seen.len(), 4);
    assert_eq!((seen[1].method.as_str(), seen[1].target.as_str()), ("POST", "/api/inference"));
    assert_eq!(seen[1].body, r#"{"enabled":true}"#);
    assert_eq!(seen[3].body, r#"{"on":1}"#);
}

#[tokio::test]
async fn latest_passes_query_and_survives_long_poll() {
    let (up, seen) = spawn_mock_upstream().await;
    let addr = spawn_armd(&format!("http://{up}")).await;
    // Mock sleeps wait_ms/10 = 300 ms; the proxy timeout is wait_ms + 2 s.
    let t0 = Instant::now();
    let r = reqwest::get(format!("http://{addr}/api/v1/detection/latest?after=5&wait_ms=3000")).await.unwrap();
    let dt = t0.elapsed();
    assert_eq!(r.status(), 200);
    let v: serde_json::Value = r.json().await.unwrap();
    assert_eq!(v["frame_id"], 7);
    assert_eq!(v["detections"][0]["label"], "cup");
    assert!(dt >= Duration::from_millis(300), "long-poll delay not observed: {dt:?}");
    assert_eq!(seen.lock().unwrap()[0].target, "/api/results/latest?after=5&wait_ms=3000");

    let r = reqwest::get(format!("http://{addr}/api/v1/detection/latest")).await.unwrap();
    assert_eq!(r.status(), 200);
    assert_eq!(seen.lock().unwrap()[1].target, "/api/results/latest");
}

#[tokio::test]
async fn latest_times_out_after_wait_plus_margin() {
    let (up, _) = spawn_mock_upstream().await;
    let addr = spawn_armd(&format!("http://{up}")).await;
    // wait_ms=0 -> 2 s budget; the mock answers after 3 s.
    let t0 = Instant::now();
    let r = reqwest::get(format!("http://{addr}/api/v1/detection/latest?slow=1&wait_ms=0")).await.unwrap();
    let dt = t0.elapsed();
    assert_eq!(r.status(), 502);
    assert!(dt >= Duration::from_millis(1900) && dt < Duration::from_millis(2900), "timeout {dt:?}");
}

#[tokio::test]
async fn upstream_down_returns_502() {
    // Bind and drop to get a port with nothing listening.
    let port = TcpListener::bind("127.0.0.1:0").await.unwrap().local_addr().unwrap().port();
    let addr = spawn_armd(&format!("http://127.0.0.1:{port}")).await;
    let c = reqwest::Client::new();
    for req in [
        c.get(format!("http://{addr}/api/v1/detection/inference")),
        c.post(format!("http://{addr}/api/v1/detection/inference")).json(&serde_json::json!({"enabled": true})),
        c.get(format!("http://{addr}/api/v1/detection/latest?after=0&wait_ms=100")),
    ] {
        let r = req.send().await.unwrap();
        assert_eq!(r.status(), 502);
        assert_eq!(r.json::<serde_json::Value>().await.unwrap(), serde_json::json!({"error": "detector unreachable"}));
    }
}

#[tokio::test]
async fn cross_origin_post_is_rejected_like_other_control_routes() {
    let (up, seen) = spawn_mock_upstream().await;
    let addr = spawn_armd(&format!("http://{up}")).await;
    let r = reqwest::Client::new()
        .post(format!("http://{addr}/api/v1/detection/inference"))
        .header("origin", "http://evil.example")
        .json(&serde_json::json!({"enabled": true}))
        .send()
        .await
        .unwrap();
    assert_eq!(r.status(), 403);
    assert!(seen.lock().unwrap().is_empty());
}

#[tokio::test]
async fn trigger_forwards_query_and_leaves_inference_state_alone() {
    let (up, seen) = spawn_mock_upstream().await;
    let addr = spawn_armd(&format!("http://{up}")).await;
    let c = reqwest::Client::new();
    let t0 = Instant::now();
    let r = c.post(format!("http://{addr}/api/v1/detection/trigger?wait_ms=3000")).send().await.unwrap();
    let dt = t0.elapsed();
    assert_eq!(r.status(), 200);
    assert_eq!(r.headers()["cache-control"], "no-store");
    let v: serde_json::Value = r.json().await.unwrap();
    assert_eq!(v["success"], true);
    assert_eq!(v["frame_id"], 9);
    assert_eq!(v["markers"][0]["id"], 4);
    assert!(dt >= Duration::from_millis(300), "long-poll delay not observed: {dt:?}");

    let r = c.post(format!("http://{addr}/api/v1/detection/trigger?wait_ms=500&miss=1")).send().await.unwrap();
    assert_eq!(r.status(), 200);
    assert_eq!(r.json::<serde_json::Value>().await.unwrap(), serde_json::json!({"success": false, "timeout": true, "count": 0, "markers": []}));

    let r = c.post(format!("http://{addr}/api/v1/detection/trigger")).send().await.unwrap();
    assert_eq!(r.status(), 200);

    let seen = seen.lock().unwrap().clone();
    let targets: Vec<_> = seen.iter().map(|s| (s.method.as_str(), s.target.as_str())).collect();
    assert_eq!(
        targets,
        vec![("POST", "/api/trigger?wait_ms=3000"), ("POST", "/api/trigger?wait_ms=500&miss=1"), ("POST", "/api/trigger")]
    );
    // Trigger never touches the continuous-inference toggle.
    assert!(seen.iter().all(|s| s.target != "/api/inference"));
}

#[tokio::test]
async fn trigger_passes_through_404_from_old_detector() {
    let (up, seen) = spawn_mock_upstream().await;
    // Under a base path the mock knows no routes, standing in for a detector without /api/trigger.
    let addr = spawn_armd(&format!("http://{up}/old")).await;
    let r = reqwest::Client::new().post(format!("http://{addr}/api/v1/detection/trigger?wait_ms=100")).send().await.unwrap();
    assert_eq!(r.status(), 404);
    assert_eq!(r.json::<serde_json::Value>().await.unwrap(), serde_json::json!({"error": "not found"}));
    assert_eq!(seen.lock().unwrap()[0].target, "/old/api/trigger?wait_ms=100");
}

#[tokio::test]
async fn trigger_times_out_after_wait_plus_margin_and_502_when_down() {
    let (up, _) = spawn_mock_upstream().await;
    let addr = spawn_armd(&format!("http://{up}")).await;
    let c = reqwest::Client::new();
    // wait_ms=0 -> 2 s budget; the mock answers after 3 s.
    let t0 = Instant::now();
    let r = c.post(format!("http://{addr}/api/v1/detection/trigger?slow=1&wait_ms=0")).send().await.unwrap();
    let dt = t0.elapsed();
    assert_eq!(r.status(), 502);
    assert!(dt >= Duration::from_millis(1900) && dt < Duration::from_millis(2900), "timeout {dt:?}");

    let port = TcpListener::bind("127.0.0.1:0").await.unwrap().local_addr().unwrap().port();
    let addr = spawn_armd(&format!("http://127.0.0.1:{port}")).await;
    let r = c.post(format!("http://{addr}/api/v1/detection/trigger?wait_ms=100")).send().await.unwrap();
    assert_eq!(r.status(), 502);
    assert_eq!(r.json::<serde_json::Value>().await.unwrap(), serde_json::json!({"error": "detector unreachable"}));
}

#[tokio::test]
async fn cross_origin_trigger_is_rejected() {
    let (up, seen) = spawn_mock_upstream().await;
    let addr = spawn_armd(&format!("http://{up}")).await;
    let r = reqwest::Client::new()
        .post(format!("http://{addr}/api/v1/detection/trigger"))
        .header("origin", "http://evil.example")
        .send()
        .await
        .unwrap();
    assert_eq!(r.status(), 403);
    assert!(seen.lock().unwrap().is_empty());
}
