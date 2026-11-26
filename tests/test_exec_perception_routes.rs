//! Dual-owner (Legacy vs RT Executor) parity and regression tests for Adapter 1:
//! - Vision / detector / camera REST proxies (`OwnerAgnostic`)
//! - `POST /api/v1/vision/detector` (`ExecutorRequest`) + mutual exclusion with `depth_measure`
//! - Sandbox routes (`OwnerAgnostic` / `ExecutorRequest`) + `read_services` robot identity + policy-use exclusion
//! - Calibration sessions (`OwnerAgnostic`) + calibration hover reads (`ExecutorRequest`) + handoff seeding
//! - Single-view measure (`POST /api/v1/grasp/measure` REST + WS) via `read_services`
//! - Multiview REST + REST home/jog/unload & migrated action stubs (`LEASE_REQUIRED` / `LEASE_ACTIVE`)
//! - Grasp `ActionStatus` diagnostics via `rt_core::stream` patch + overflow / lossless / terminal freeze

mod common;

use std::net::SocketAddr;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use futures_util::StreamExt;
use serde_json::{json, Value};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpListener;
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::Message;

use armd::api::proxy::{DetectionWsUpstream, DetectorUpstream};
use armd::api::sandbox::SandboxState;
use armd::api::{create_router_calibration_with_auth, create_router_full_with_sandbox};
use armd::calibration::CalibrationDeps;
use armd::diagnostics::DiagnosticsState;
use armd::grasp::{
    self, BoxFuture, CamModel, DepthFrameInfo, Detection, DetectionFrame, DetectionSource,
    GraspDeps, ModeInfo,
};
use armd::kinematics::{self, WorkspaceViolationDetail};
use armd::model::ArmConfig;
use armd::motion::{ActionStatus, EstimateEntry};
use armd::rt_core::action_diag::{
    ActionDiagKind, ActionDiagPatch, ApproachKind, BorderKind, DiagStr, EstimateListKind,
    PostLiftKind, ShortLabel, ACTION_DIAG_STREAM_CAP,
};
use armd::rt_core::executor::{ExecutorConfig, RtExecutorHandle};
use armd::rt_core::publish::{state_channel, ReadRequirements};
use armd::rt_core::stream::StreamPushError;
use armd::{ArmSupervisor, OperatorAuth};

static VISION_TEST_LOCK: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum OwnerKind {
    Legacy,
    Executor,
}

struct RunningOwner {
    sup: Arc<ArmSupervisor>,
    addr: SocketAddr,
    _exec_handle: Option<RtExecutorHandle>,
    _legacy_ticker: Option<tokio::task::JoinHandle<()>>,
    _tmp: tempfile::TempDir,
}

impl Drop for RunningOwner {
    fn drop(&mut self) {
        if let Some(h) = self._exec_handle.take() {
            h.stop_and_join();
        }
        if let Some(t) = self._legacy_ticker.take() {
            t.abort();
        }
    }
}

async fn wait_for_service_ready(sup: &Arc<ArmSupervisor>) {
    let dl = Instant::now() + Duration::from_secs(3);
    while Instant::now() < dl {
        if sup
            .read_services(ReadRequirements::default())
            .await
            .is_ok()
        {
            return;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    panic!("supervisor failed to publish initial coherent ServiceSnapshot");
}

async fn spawn_owner(
    owner: OwnerKind,
    config: ArmConfig,
    detector: DetectorUpstream,
    ws_up: Option<DetectionWsUpstream>,
    source: Arc<dyn DetectionSource>,
    auth: OperatorAuth,
    sandbox: Option<Arc<SandboxState>>,
    pre_handoff: Option<Box<dyn FnOnce(&mut armd::SupervisorInner) + Send>>,
) -> RunningOwner {
    let tmp = tempfile::tempdir().unwrap();
    let sup = Arc::new(ArmSupervisor::new(config));
    if let Some(init) = pre_handoff {
        let mut inner = sup.inner().write().await;
        init(&mut inner);
    }
    let grasp = GraspDeps {
        source,
        setup_path: Some(tmp.path().join("grasp_setup.json")),
    };
    let calib = CalibrationDeps {
        path: tmp.path().join("calibration.json"),
        backup_dir: tmp.path().join("calib_backups"),
        sessions_dir: tmp.path().join("calib_sessions"),
        setup_path: grasp.setup_path.clone(),
        upstream: detector.clone(),
        vision_timeout: Duration::from_secs(2),
    };
    let diag = DiagnosticsState::global();
    let mut router = match sandbox {
        Some(sb) => create_router_full_with_sandbox(
            sup.clone(),
            detector,
            grasp,
            diag,
            calib,
            auth,
            sb,
        ),
        None => create_router_calibration_with_auth(sup.clone(), detector, grasp, diag, calib, auth),
    };
    if let Some(w) = ws_up {
        router = router.layer(axum::Extension(w));
    }
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        let _ = axum::serve(listener, router).await;
    });

    let (exec_handle, legacy_ticker) = match owner {
        OwnerKind::Executor => {
            let h = sup
                .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
                .await
                .expect("start_executor");
            wait_for_service_ready(&sup).await;
            (Some(h), None)
        }
        OwnerKind::Legacy => {
            let sup_t = sup.clone();
            let jh = tokio::spawn(async move {
                let mut tick = tokio::time::interval(Duration::from_millis(10));
                loop {
                    tick.tick().await;
                    if sup_t.executor_handle().is_some() {
                        break;
                    }
                    let mut inner = sup_t.inner().write().await;
                    let _ = inner.tick(0.01);
                }
            });
            wait_for_service_ready(&sup).await;
            (None, Some(jh))
        }
    };

    RunningOwner {
        sup,
        addr,
        _exec_handle: exec_handle,
        _legacy_ticker: legacy_ticker,
        _tmp: tmp,
    }
}

/// Minimal mock HTTP upstream echoing method + path + status for all vision/detector/camera proxy routes.
async fn spawn_mock_vision_http() -> SocketAddr {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        loop {
            let Ok((mut sock, _)) = listener.accept().await else {
                break;
            };
            tokio::spawn(async move {
                let mut raw = Vec::new();
                let mut buf = [0u8; 4096];
                while !raw.windows(4).any(|w| w == b"\r\n\r\n") {
                    let Ok(n) = sock.read(&mut buf).await else {
                        return;
                    };
                    if n == 0 {
                        return;
                    }
                    raw.extend_from_slice(&buf[..n]);
                }
                let head = String::from_utf8_lossy(&raw).to_string();
                let first = head.lines().next().unwrap_or("");
                let mut parts = first.split_whitespace();
                let method = parts.next().unwrap_or("GET");
                let path = parts.next().unwrap_or("/");
                let (ct, body): (&str, Vec<u8>) = if path.starts_with("/api/depth") {
                    ("application/octet-stream", b"RDM1test".to_vec())
                } else if path == "/snapshot.jpg" {
                    ("image/jpeg", vec![0xff, 0xd8, 0xff, 0xd9])
                } else if path == "/api/mode" {
                    (
                        "application/json",
                        serde_json::to_vec(&json!({
                            "api_version": 2,
                            "mode": "yolo",
                            "enabled": true,
                            "model_config": {"optimal": true, "missing": [], "mismatched": [], "source": "bundled"},
                            "seen_method": method
                        }))
                        .unwrap(),
                    )
                } else {
                    (
                        "application/json",
                        serde_json::to_vec(&json!({"ok": true, "method": method, "path": path}))
                            .unwrap(),
                    )
                };
                let resp = format!(
                    "HTTP/1.1 200 OK\r\nContent-Type: {ct}\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
                    body.len()
                );
                let _ = sock.write_all(resp.as_bytes()).await;
                let _ = sock.write_all(&body).await;
            });
        }
    });
    addr
}

/// Minimal mock WS upstream emitting one JSON frame and closing cleanly.
async fn spawn_mock_detection_ws() -> SocketAddr {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        loop {
            let Ok((stream, _)) = listener.accept().await else {
                break;
            };
            tokio::spawn(async move {
                if let Ok(mut ws) = tokio_tungstenite::accept_async(stream).await {
                    use futures_util::SinkExt;
                    let _ = ws
                        .send(Message::Text(r#"{"frame_id":42,"detections":[]}"#.into()))
                        .await;
                    tokio::time::sleep(Duration::from_millis(50)).await;
                }
            });
        }
    });
    addr
}

struct DummySource;
impl DetectionSource for DummySource {
    fn inference_enabled(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async { Ok(false) })
    }
    fn set_inference(&self, _enabled: bool) -> BoxFuture<'_, Result<(), String>> {
        Box::pin(async { Ok(()) })
    }
    fn trigger_with_focus(
        &self,
        pipelines: &'static [&'static str],
        wait_ms: u64,
        _focus: Option<&grasp::MaskFocus>,
    ) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        self.trigger(pipelines, wait_ms)
    }
    fn next_frame(
        &self,
        _after: Option<u64>,
        _wait_ms: u64,
    ) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        Box::pin(async { Ok(None) })
    }
}

async fn unlock_token(http: &reqwest::Client, addr: SocketAddr, pin: &str) -> String {
    let v: Value = http
        .post(format!("http://{addr}/api/v1/auth/unlock"))
        .json(&json!({"pin": pin, "set_cookie": false}))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    v["token"].as_str().unwrap().to_string()
}

// ============================================================================
// 1. Vision / detector / camera REST proxies (dual-owner + auth + fault fence)
// ============================================================================

#[tokio::test]
async fn dual_owner_vision_detector_camera_rest_proxies_and_auth_fault_parity() {
    let http_up = spawn_mock_vision_http().await;
    let ws_up_addr = spawn_mock_detection_ws().await;
    let detector = DetectorUpstream::parse(&format!("http://{http_up}")).unwrap();
    let ws_up = DetectionWsUpstream(format!("ws://{ws_up_addr}/results"));
    let http = reqwest::Client::new();

    let mut results = Vec::new();
    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let run = spawn_owner(
            owner,
            ArmConfig::rebot_dm(),
            detector.clone(),
            Some(ws_up.clone()),
            Arc::new(DummySource),
            OperatorAuth::enforced("123456", 1800),
            None,
            None,
        )
        .await;
        let base = format!("http://{}", run.addr);
        let tok = unlock_token(&http, run.addr, "123456").await;

        // Public reads
        let r_mode_get = http.get(format!("{base}/api/v1/vision/mode")).send().await.unwrap();
        let mode_get = (r_mode_get.status().as_u16(), r_mode_get.json::<Value>().await.unwrap());
        let r_inf_get = http.get(format!("{base}/api/v1/detection/inference")).send().await.unwrap();
        let inf_get = (r_inf_get.status().as_u16(), r_inf_get.json::<Value>().await.unwrap());
        let r_lat_get = http.get(format!("{base}/api/v1/detection/latest")).send().await.unwrap();
        let lat_get = (r_lat_get.status().as_u16(), r_lat_get.json::<Value>().await.unwrap());
        let r_mf_get = http.get(format!("{base}/api/v1/detection/mask_focus")).send().await.unwrap();
        let mf_get = (r_mf_get.status().as_u16(), r_mf_get.json::<Value>().await.unwrap());
        let r_subs_get = http.get(format!("{base}/api/v1/vision/subscriptions")).send().await.unwrap();
        let subs_get = (r_subs_get.status().as_u16(), r_subs_get.json::<Value>().await.unwrap());
        // Camera snapshot hits port 8001 unless overridden; unreachable -> 502 (never 503 EXECUTOR_ROUTING_PENDING).
        let r_snap = http.get(format!("{base}/api/v1/camera/snapshot")).send().await.unwrap();
        let snap_status = r_snap.status().as_u16();
        assert_ne!(snap_status, 503, "{owner:?} camera/snapshot must not be gated");

        // Operator-gated routes: rejected without token, succeed with token.
        let depth_unauth = http.get(format!("{base}/api/v1/vision/depth")).send().await.unwrap().status().as_u16();
        assert_eq!(depth_unauth, 403, "{owner:?}");
        let r_depth = http.get(format!("{base}/api/v1/vision/depth")).bearer_auth(&tok).send().await.unwrap();
        let depth_ok = (r_depth.status().as_u16(), r_depth.bytes().await.unwrap().to_vec());

        let post_mode_unauth = http.post(format!("{base}/api/v1/vision/mode")).json(&json!({"mode":"yolo"})).send().await.unwrap().status().as_u16();
        assert_eq!(post_mode_unauth, 403, "{owner:?}");
        let r_mode_post = http.post(format!("{base}/api/v1/vision/mode")).bearer_auth(&tok).json(&json!({"mode":"yolo"})).send().await.unwrap();
        let mode_post = (r_mode_post.status().as_u16(), r_mode_post.json::<Value>().await.unwrap());

        let r_inf_post = http.post(format!("{base}/api/v1/detection/inference")).bearer_auth(&tok).json(&json!({"enabled":true})).send().await.unwrap();
        let inf_post = (r_inf_post.status().as_u16(), r_inf_post.json::<Value>().await.unwrap());

        let r_trig = http.post(format!("{base}/api/v1/detection/trigger")).bearer_auth(&tok).json(&json!({})).send().await.unwrap();
        let trig_post = (r_trig.status().as_u16(), r_trig.json::<Value>().await.unwrap());

        let r_mf_post = http.post(format!("{base}/api/v1/detection/mask_focus")).bearer_auth(&tok).json(&json!({})).send().await.unwrap();
        let mf_post = (r_mf_post.status().as_u16(), r_mf_post.json::<Value>().await.unwrap());

        let r_sub = http.post(format!("{base}/api/v1/vision/subscribe")).bearer_auth(&tok).json(&json!({"client":"c"})).send().await.unwrap();
        let sub_post = (r_sub.status().as_u16(), r_sub.json::<Value>().await.unwrap());

        let r_ka = http.post(format!("{base}/api/v1/vision/subscribe/s1/keepalive")).bearer_auth(&tok).send().await.unwrap();
        let ka_post = (r_ka.status().as_u16(), r_ka.json::<Value>().await.unwrap());

        let r_del = http.delete(format!("{base}/api/v1/vision/subscribe/s1")).bearer_auth(&tok).send().await.unwrap();
        let del_sub = (r_del.status().as_u16(), r_del.json::<Value>().await.unwrap());

        // Detection WS proxies: both `/api/v1/detection/ws` and `/results` bridge upstream frames.
        let mut ws_frames = Vec::new();
        for p in ["/api/v1/detection/ws", "/results"] {
            let (mut ws, _) = connect_async(format!("ws://{}{p}", run.addr)).await.unwrap();
            let txt = tokio::time::timeout(Duration::from_secs(2), ws.next())
                .await
                .unwrap()
                .unwrap()
                .unwrap()
                .into_text()
                .unwrap()
                .to_string();
            ws_frames.push(txt);
        }

        // Fault fence: reads remain allowed, mutating proxies return 503 INTERNAL_FAULT.
        run.sup.internal_fault_lane().latch_control_safe_hold_fence();
        let fault_read = http.get(format!("{base}/api/v1/vision/mode")).send().await.unwrap().status().as_u16();
        let fault_write_resp = http.post(format!("{base}/api/v1/vision/mode")).bearer_auth(&tok).json(&json!({})).send().await.unwrap();
        let fault_write = (
            fault_write_resp.status().as_u16(),
            fault_write_resp.json::<Value>().await.unwrap()["code"]
                .as_str()
                .map(str::to_owned),
        );

        results.push(json!({
            "mode_get": mode_get,
            "inf_get": inf_get,
            "lat_get": lat_get,
            "mf_get": mf_get,
            "subs_get": subs_get,
            "depth_ok": depth_ok,
            "mode_post": mode_post,
            "inf_post": inf_post,
            "trig_post": trig_post,
            "mf_post": mf_post,
            "sub_post": sub_post,
            "ka_post": ka_post,
            "del_sub": del_sub,
            "ws_frames": ws_frames,
            "fault_read": fault_read,
            "fault_write": fault_write,
        }));
    }

    assert_eq!(results[0], results[1], "Legacy and Executor proxy responses must be identical");
    assert_eq!(results[1]["fault_read"], 200);
    assert_eq!(results[1]["fault_write"], json!([503, "INTERNAL_FAULT"]));
}

// ============================================================================
// 2. POST /api/v1/vision/detector + mutual exclusion with depth_measure
// ============================================================================

struct SwitchProvider {
    mode: Mutex<Value>,
    delay_ms: AtomicU64,
}

impl SwitchProvider {
    fn new() -> Arc<Self> {
        let det = "yolo11n_detection_cv181x_int8";
        let seg = "yolo11n_seg_384x640_cv181x_int8_fuse";
        let cat = json!([
            {"name": det, "kind": "detect", "ion_mb": 10.0},
            {"name": seg, "kind": "segment", "ion_mb": 12.0}
        ]);
        Arc::new(Self {
            mode: Mutex::new(json!({
                "api_version": 2,
                "mode": "yolo",
                "enabled": false,
                "per_request_pipelines": true,
                "available": {"yolo": true, "depth": true},
                "model": {"kind": "detect", "name": det},
                "models": {
                    "generation": 0,
                    "resident": {"detector": det, "depth": null},
                    "swap": {"state": "idle", "generation": 0},
                    "catalogue": cat
                }
            })),
            delay_ms: AtomicU64::new(0),
        })
    }
}

impl DetectionSource for SwitchProvider {
    fn inference_enabled(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async { Ok(false) })
    }
    fn set_inference(&self, _enabled: bool) -> BoxFuture<'_, Result<(), String>> {
        Box::pin(async { Ok(()) })
    }
    fn trigger_with_focus(
        &self,
        pipelines: &'static [&'static str],
        wait_ms: u64,
        _focus: Option<&grasp::MaskFocus>,
    ) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        self.trigger(pipelines, wait_ms)
    }
    fn next_frame(
        &self,
        _after: Option<u64>,
        _wait_ms: u64,
    ) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        Box::pin(async { Ok(None) })
    }
    fn per_request_pipelines(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async { Ok(true) })
    }
    fn mode_info(&self) -> BoxFuture<'_, Result<ModeInfo, String>> {
        let v = self.mode.lock().unwrap().clone();
        Box::pin(async move { Ok(ModeInfo::parse(Some(&v))) })
    }
    fn set_models(&self, body: &Value) -> BoxFuture<'_, Result<(u16, Value), String>> {
        let delay = self.delay_ms.load(Ordering::SeqCst);
        let name = body["detector"].as_str().unwrap().to_string();
        Box::pin(async move {
            if delay > 0 {
                tokio::time::sleep(Duration::from_millis(delay)).await;
            }
            let mut m = self.mode.lock().unwrap();
            let gen = m["models"]["generation"].as_u64().unwrap() + 1;
            m["models"]["generation"] = json!(gen);
            m["models"]["resident"]["detector"] = json!(name);
            Ok((202, json!({"generation": gen})))
        })
    }
    fn trigger(
        &self,
        _pipelines: &[&str],
        _wait_ms: u64,
    ) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        let delay = self.delay_ms.load(Ordering::SeqCst);
        Box::pin(async move {
            if delay > 0 {
                tokio::time::sleep(Duration::from_millis(delay)).await;
            }
            Ok(Some(DetectionFrame {
                frame_id: 1,
                captured_at: Instant::now(),
                policy_captured_at: None,
                image_size_px: [640.0, 480.0],
                detections: Vec::new(),
                depth: None,
                model_generation: Some(0),
            }))
        })
    }
}

#[tokio::test]
async fn dual_owner_vision_detector_switch_and_measure_mutual_exclusion() {
    let _vision_guard = VISION_TEST_LOCK.lock().await;
    let http = reqwest::Client::new();
    let detector = DetectorUpstream::parse("http://127.0.0.1:9").unwrap();
    let mut results = Vec::new();

    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let provider = SwitchProvider::new();
        let run = spawn_owner(
            owner,
            ArmConfig::rebot_dm(),
            detector.clone(),
            None,
            provider.clone(),
            OperatorAuth::enforced("123456", 1800),
            None,
            None,
        )
        .await;
        let base = format!("http://{}", run.addr);

        let unauth = http
            .post(format!("{base}/api/v1/vision/detector"))
            .json(&json!({"kind": "segment"}))
            .send()
            .await
            .unwrap();
        assert_eq!(unauth.status().as_u16(), 403);
        let tok = unlock_token(&http, run.addr, "123456").await;

        let r1 = http
            .post(format!("{base}/api/v1/vision/detector"))
            .bearer_auth(&tok)
            .json(&json!({"kind": "segment"}))
            .send()
            .await
            .unwrap();
        let v1 = (r1.status().as_u16(), r1.json::<Value>().await.unwrap());

        let r2 = http
            .post(format!("{base}/api/v1/vision/detector"))
            .bearer_auth(&tok)
            .json(&json!({"kind": "segment"}))
            .send()
            .await
            .unwrap();
        let v2 = (r2.status().as_u16(), r2.json::<Value>().await.unwrap());

        // Mutual exclusion direction 1: switch in flight blocks depth_measure with 409 VISION_BUSY.
        provider.delay_ms.store(150, Ordering::SeqCst);
        let http_bg = http.clone();
        let base_bg = base.clone();
        let tok_bg = tok.clone();
        let switch_task = tokio::spawn(async move {
            http_bg
                .post(format!("{base_bg}/api/v1/vision/detector"))
                .bearer_auth(&tok_bg)
                .json(&json!({"kind": "detect"}))
                .send()
                .await
                .unwrap()
                .status()
                .as_u16()
        });
        tokio::time::sleep(Duration::from_millis(30)).await;
        let meas_during_switch = http
            .post(format!("{base}/api/v1/grasp/measure"))
            .bearer_auth(&tok)
            .json(&json!({}))
            .send()
            .await
            .unwrap();
        let mds_code = (
            meas_during_switch.status().as_u16(),
            meas_during_switch.json::<Value>().await.unwrap()["code"]
                .as_str()
                .map(str::to_owned),
        );
        assert_eq!(switch_task.await.unwrap(), 200);

        // Mutual exclusion direction 2: measure in flight blocks switch with 409 VISION_BUSY.
        let http_bg2 = http.clone();
        let base_bg2 = base.clone();
        let tok_bg2 = tok.clone();
        let measure_task = tokio::spawn(async move {
            http_bg2
                .post(format!("{base_bg2}/api/v1/grasp/measure"))
                .bearer_auth(&tok_bg2)
                .json(&json!({}))
                .send()
                .await
                .unwrap()
                .status()
                .as_u16()
        });
        tokio::time::sleep(Duration::from_millis(30)).await;
        let switch_during_meas = http
            .post(format!("{base}/api/v1/vision/detector"))
            .bearer_auth(&tok)
            .json(&json!({"kind": "segment"}))
            .send()
            .await
            .unwrap();
        let sdm_code = (
            switch_during_meas.status().as_u16(),
            switch_during_meas.json::<Value>().await.unwrap()["code"]
                .as_str()
                .map(str::to_owned),
        );
        assert_eq!(measure_task.await.unwrap(), 200);
        provider.delay_ms.store(0, Ordering::SeqCst);

        results.push((v1, v2, mds_code, sdm_code));
    }

    assert_eq!(results[0], results[1]);
    assert_eq!(results[1].0 .0, 200);
    assert_eq!(results[1].0 .1["changed"], json!(true));
    assert_eq!(results[1].1 .1["changed"], json!(false));
    assert_eq!(results[1].2, (409, Some("VISION_BUSY".to_string())));
    assert_eq!(results[1].3, (409, Some("VISION_BUSY".to_string())));
}

// ============================================================================
// 3. Sandbox routes + policy-use exclusion (dual-owner + executor pin tests)
// ============================================================================

#[tokio::test]
async fn dual_owner_sandbox_routes_and_policy_use_exclusion() {
    let http = reqwest::Client::new();
    let detector = DetectorUpstream::parse("http://127.0.0.1:9").unwrap();

    let mut summaries = Vec::new();
    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let sb_dir = tempfile::tempdir().unwrap();
        let sb = Arc::new(SandboxState::new(sb_dir.path().to_path_buf()));
        let run = spawn_owner(
            owner,
            ArmConfig::rebot_dm(),
            detector.clone(),
            None,
            Arc::new(DummySource),
            OperatorAuth::enforced("123456", 1800),
            Some(sb.clone()),
            None,
        )
        .await;
        let base = format!("http://{}", run.addr);
        let tok = unlock_token(&http, run.addr, "123456").await;

        // Catalog reflects running robot identity via read_services (no SupervisorLock on executor).
        let cat: Value = http
            .get(format!("{base}/api/v1/sandbox/catalog"))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        let running_robot = cat["robot"].as_str().unwrap().to_string();

        // State, reset, io, telemetry, base-assets
        let st_status = http.get(format!("{base}/api/v1/sandbox/state")).send().await.unwrap().status().as_u16();
        let reset_status = http.post(format!("{base}/api/v1/sandbox/reset")).bearer_auth(&tok).json(&json!({})).send().await.unwrap().status().as_u16();
        let io_status = http.post(format!("{base}/api/v1/sandbox/io")).bearer_auth(&tok).json(&json!({})).send().await.unwrap().status().as_u16();
        let tel_status = http.post(format!("{base}/api/v1/sandbox/telemetry")).bearer_auth(&tok).json(&json!({})).send().await.unwrap().status().as_u16();

        // Upload a policy pack, fetch metadata & file, activate mode=none.
        let policy_json = json!({
            "format": "rebot-onnx-policy/v1",
            "id": "pol_dual_1",
            "name": "Dual Test Policy",
            "robot": "b601dm",
            "action_dim": 7,
            "inputs": {},
            "outputs": {},
            "model_file": "model.onnx"
        });
        let up_resp = http
            .post(format!("{base}/api/v1/sandbox/policies"))
            .bearer_auth(&tok)
            .json(&json!({
                "id": "pol_dual_1",
                "policy_json": policy_json,
                "model_onnx_base64": "cG9saWN5"
            }))
            .send()
            .await
            .unwrap();
        let up_status = up_resp.status().as_u16();
        let get_pol_status = http.get(format!("{base}/api/v1/sandbox/policies/pol_dual_1")).send().await.unwrap().status().as_u16();
        let get_pol_file_status = http.get(format!("{base}/api/v1/sandbox/policies/pol_dual_1/files/model.onnx")).send().await.unwrap().status().as_u16();

        let act_status = http
            .post(format!("{base}/api/v1/sandbox/activate"))
            .bearer_auth(&tok)
            .json(&json!({"mode": "none", "policy": "pol_dual_1"}))
            .send()
            .await
            .unwrap()
            .status()
            .as_u16();

        // Deleting active sandbox policy -> 409 POLICY_IN_USE.
        let del_active = http
            .delete(format!("{base}/api/v1/sandbox/policies/pol_dual_1"))
            .bearer_auth(&tok)
            .send()
            .await
            .unwrap();
        let del_active_res = (
            del_active.status().as_u16(),
            del_active.json::<Value>().await.unwrap()["code"].as_str().map(str::to_owned),
        );

        // Deactivate policy pack.
        let deact = http
            .post(format!("{base}/api/v1/sandbox/activate"))
            .bearer_auth(&tok)
            .json(&json!({"mode": "none", "policy": null}))
            .send()
            .await
            .unwrap();
        assert_eq!(deact.status().as_u16(), 200);

        // On Executor, exercise the PolicyPackUseGate load pin + atomic transition to active run pin:
        // deleting while loading OR while running MUST fail with 409 POLICY_IN_USE, and once the
        // runner drops its guard, delete succeeds.
        if owner == OwnerKind::Executor {
            let load_guard = sb.try_begin_policy_load("pol_dual_1").expect("pin load");
            let del_loading = http
                .delete(format!("{base}/api/v1/sandbox/policies/pol_dual_1"))
                .bearer_auth(&tok)
                .send()
                .await
                .unwrap();
            assert_eq!(del_loading.status().as_u16(), 409);
            assert_eq!(del_loading.json::<Value>().await.unwrap()["code"], "POLICY_IN_USE");

            // Atomic transition load -> active run closes the 1-tick StateHub publication gap.
            let run_guard = load_guard.into_active_run();
            let del_running = http
                .delete(format!("{base}/api/v1/sandbox/policies/pol_dual_1"))
                .bearer_auth(&tok)
                .send()
                .await
                .unwrap();
            assert_eq!(del_running.status().as_u16(), 409);
            assert_eq!(del_running.json::<Value>().await.unwrap()["code"], "POLICY_IN_USE");
            drop(run_guard);
        }

        // Now delete succeeds identically on both owners.
        let del_ok = http
            .delete(format!("{base}/api/v1/sandbox/policies/pol_dual_1"))
            .bearer_auth(&tok)
            .send()
            .await
            .unwrap();
        let del_ok_res = (del_ok.status().as_u16(), del_ok.json::<Value>().await.unwrap());

        // Bundle read works on both owners.
        let bundle_status = http.get(format!("{base}/api/v1/sandbox/bundle")).send().await.unwrap().status().as_u16();
        assert_ne!(bundle_status, 503, "{owner:?} bundle must not be gated");

        summaries.push((
            running_robot,
            st_status,
            reset_status,
            io_status,
            tel_status,
            up_status,
            get_pol_status,
            get_pol_file_status,
            act_status,
            del_active_res,
            del_ok_res,
            bundle_status,
        ));
    }

    assert_eq!(summaries[0], summaries[1]);
    assert_eq!(summaries[1].0, "b601dm");
    assert_eq!(summaries[1].9, (409, Some("POLICY_IN_USE".to_string())));
    assert_eq!(summaries[1].10 .0, 200);
    assert_eq!(summaries[1].10 .1["deleted"], json!(true));
}

// ============================================================================
// 4. Calibration sessions + hover reads (dual-owner + pre-handoff job seeding)
// ============================================================================

#[tokio::test]
async fn dual_owner_calibration_sessions_and_hover_reads_with_handoff() {
    let http = reqwest::Client::new();
    let detector = DetectorUpstream::parse("http://127.0.0.1:9").unwrap();
    let mut results = Vec::new();

    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let seed_job: armd::calib_hover::HoverJob = serde_json::from_value(json!({
            "id": "pre-handoff-job-1",
            "state": "done",
            "code": null,
            "message": null,
            "created_at": "2026-11-26T00:00:00.000Z",
            "updated_at": "2026-11-26T00:00:01.000Z",
            "candidate_fingerprint": "fp-test",
            "params": {
                "approach_speed_mps": 0.1,
                "descent_speed_mps": 0.05,
                "settle_s": 0.2,
                "frames": 3,
                "discard": 1
            },
            "points": [],
            "runs": 1,
            "end": null,
            "action_id": null
        }))
        .unwrap();
        let run = spawn_owner(
            owner,
            ArmConfig::rebot_dm(),
            detector.clone(),
            None,
            Arc::new(DummySource),
            OperatorAuth::enforced("123456", 1800),
            None,
            Some(Box::new(move |inner| {
                inner.hover_jobs.push(seed_job);
            })),
        )
        .await;
        let base = format!("http://{}", run.addr);
        let tok = unlock_token(&http, run.addr, "123456").await;

        // Calibration sessions CRUD: GET public, POST/DELETE operator-gated.
        let unauth_post = http
            .post(format!("{base}/api/v1/calibration/sessions"))
            .json(&json!({"kind": "camera", "views": []}))
            .send()
            .await
            .unwrap()
            .status()
            .as_u16();
        assert_eq!(unauth_post, 403);

        let created: Value = http
            .post(format!("{base}/api/v1/calibration/sessions"))
            .bearer_auth(&tok)
            .json(&json!({"id": "sess_dual_1", "kind": "camera", "views": [{"q": [0.0]}]}))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!(created["id"], "sess_dual_1");

        let list: Value = http
            .get(format!("{base}/api/v1/calibration/sessions"))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!(list["sessions"].as_array().unwrap().len(), 1);

        let fetched: Value = http
            .get(format!("{base}/api/v1/calibration/sessions/sess_dual_1"))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!(fetched["kind"], "camera");

        let deleted: Value = http
            .delete(format!("{base}/api/v1/calibration/sessions/sess_dual_1"))
            .bearer_auth(&tok)
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!(deleted, json!({"deleted": "sess_dual_1"}));

        // Calibration hover reads: pre-handoff job is visible on both Legacy and Executor,
        // and `arm` fields match exactly (including lease_held flip after acquiring a WS lease).
        let hover_no_lease: Value = http
            .get(format!("{base}/api/v1/calibration/hover"))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        let mut arm_no_lease = hover_no_lease["arm"].clone();
        arm_no_lease.as_object_mut().unwrap().remove("at");

        let (_ws, _gen) = common::controller(run.addr).await;
        tokio::time::sleep(Duration::from_millis(25)).await;
        let hover_job_lease: Value = http
            .get(format!("{base}/api/v1/calibration/hover/pre-handoff-job-1"))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        let mut arm_with_lease = hover_job_lease["arm"].clone();
        arm_with_lease.as_object_mut().unwrap().remove("at");

        results.push((
            hover_no_lease["jobs"].clone(),
            arm_no_lease,
            hover_job_lease["id"].clone(),
            arm_with_lease,
        ));
    }

    assert_eq!(results[0], results[1]);
    assert_eq!(results[1].1["lease_held"], json!(false));
    assert_eq!(results[1].1["watchdog_park"], json!(false));
    assert_eq!(results[1].2, json!("pre-handoff-job-1"));
    assert_eq!(results[1].3["lease_held"], json!(true));
}

// ============================================================================
// 5. Single-view measure (REST + WS) via read_services (dual-owner)
// ============================================================================

struct SyntheticMeasureSource {
    sup: Arc<ArmSupervisor>,
    cam: CamModel,
    trigger_delay_ms: AtomicU64,
}

impl DetectionSource for SyntheticMeasureSource {
    fn inference_enabled(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async { Ok(false) })
    }
    fn set_inference(&self, _enabled: bool) -> BoxFuture<'_, Result<(), String>> {
        Box::pin(async { Ok(()) })
    }
    fn trigger_with_focus(
        &self,
        pipelines: &'static [&'static str],
        wait_ms: u64,
        _focus: Option<&grasp::MaskFocus>,
    ) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        self.trigger(pipelines, wait_ms)
    }
    fn next_frame(
        &self,
        _after: Option<u64>,
        _wait_ms: u64,
    ) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        Box::pin(async { Ok(None) })
    }
    fn per_request_pipelines(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async { Ok(true) })
    }
    fn trigger(
        &self,
        _pipelines: &[&str],
        _wait_ms: u64,
    ) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        let delay = self.trigger_delay_ms.load(Ordering::SeqCst);
        let sup = self.sup.clone();
        let cam = self.cam.clone();
        Box::pin(async move {
            if delay > 0 {
                tokio::time::sleep(Duration::from_millis(delay)).await;
            }
            let svc = sup
                .read_services(ReadRequirements::identity())
                .await
                .map_err(|e| format!("{e}"))?;
            let env = sup.hub().prep_env().unwrap();
            let pose = env
                .config
                .arm_model
                .fk_f32(&svc.snapshot.rt.commanded_positions)
                .unwrap();
            let mut table_samples = Vec::new();
            for row in 0..4 {
                for col in 0..4 {
                    let u_n = 0.15 + 0.2 * col as f64;
                    let v_n = 0.2 + 0.2 * row as f64;
                    let px = cam.to_px([u_n, v_n]);
                    if let Some(p) = cam.plane_point(&pose, px, 0.0) {
                        let o = cam.origin(&pose);
                        let r = cam.rotation(&pose);
                        let z_cam = kinematics::dot(&kinematics::sub(&p, &o), &kinematics::column(&r, 2));
                        if z_cam > 0.05 {
                            table_samples.push([u_n, v_n, 1.0 / z_cam]);
                        }
                    }
                }
            }
            let det = Detection {
                label: "cup".into(),
                score: 0.92,
                raw_score_valid: true,
                box_norm: [0.45, 0.45, 0.55, 0.55],
                angle_deg: Some(0.0),
                angle_conf: Some(0.9),
                grasp_width_px: Some(36.0),
                mask_focus: false,
                disp_fg: Some(1.0 / 0.25),
                disp_bg: Some(1.0 / 0.30),
                depth_conf: Some(0.95),
                length_px: Some(50.0),
                jaw_clear: Some(true),
                depth_angle_deg: None,
                depth_angle_conf: None,
                depth_width_px: None,
                contour_norm: None,
            };
            Ok(Some(DetectionFrame {
                frame_id: 7,
                captured_at: Instant::now(),
                policy_captured_at: None,
                image_size_px: cam.size,
                detections: vec![det],
                depth: Some(DepthFrameInfo {
                    metric_model: true,
                    table_samples,
                }),
                model_generation: Some(1),
            }))
        })
    }
}

#[tokio::test]
async fn dual_owner_single_view_measure_rest_and_ws_via_read_services() {
    let _vision_guard = VISION_TEST_LOCK.lock().await;
    let http = reqwest::Client::new();
    let detector = DetectorUpstream::parse("http://127.0.0.1:9").unwrap();
    let mut outputs = Vec::new();

    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.camera_mount.offset_m = Some([0.0, -0.05, 0.02]);
        let setup = cfg.grasp.clone().unwrap();
        let cam = CamModel::assumed(&cfg.arm_model, &setup, &cfg.camera_mount);
        let sup_holder: Arc<Mutex<Option<Arc<ArmSupervisor>>>> = Arc::new(Mutex::new(None));
        // Build supervisor first so the synthetic camera can read it without locking inner.
        let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
        {
            let mut inner = sup.inner().write().await;
            inner.commanded_positions = common::INSPECT.to_vec();
            inner.measured_positions = common::INSPECT.to_vec();
            inner.target_positions = common::INSPECT.to_vec();
        }
        *sup_holder.lock().unwrap() = Some(sup.clone());
        let source = Arc::new(SyntheticMeasureSource {
            sup: sup.clone(),
            cam,
            trigger_delay_ms: AtomicU64::new(0),
        });
        let tmp = tempfile::tempdir().unwrap();
        let grasp_deps = GraspDeps {
            source: source.clone(),
            setup_path: Some(tmp.path().join("grasp_setup.json")),
        };
        let calib_deps = CalibrationDeps {
            path: tmp.path().join("calibration.json"),
            backup_dir: tmp.path().join("calib_backups"),
            sessions_dir: tmp.path().join("calib_sessions"),
            setup_path: grasp_deps.setup_path.clone(),
            upstream: detector.clone(),
            vision_timeout: Duration::from_secs(2),
        };
        let router = create_router_calibration_with_auth(
            sup.clone(),
            detector.clone(),
            grasp_deps,
            DiagnosticsState::global(),
            calib_deps,
            OperatorAuth::enforced("123456", 1800),
        );
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        tokio::spawn(async move {
            let _ = axum::serve(listener, router).await;
        });

        let (exec_h, leg_t) = match owner {
            OwnerKind::Executor => {
                let h = sup
                    .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
                    .await
                    .unwrap();
                wait_for_service_ready(&sup).await;
                (Some(h), None)
            }
            OwnerKind::Legacy => {
                let s = sup.clone();
                let jh = tokio::spawn(async move {
                    let mut tick = tokio::time::interval(Duration::from_millis(5));
                    loop {
                        tick.tick().await;
                        if s.executor_handle().is_some() {
                            break;
                        }
                        let mut inner = s.inner().write().await;
                        let _ = inner.tick(0.005);
                    }
                });
                wait_for_service_ready(&sup).await;
                (None, Some(jh))
            }
        };

        let base = format!("http://{addr}");
        let tok = unlock_token(&http, addr, "123456").await;

        // REST measure: 403 without token, 200 with token.
        let unauth = http.post(format!("{base}/api/v1/grasp/measure")).json(&json!({})).send().await.unwrap();
        assert_eq!(unauth.status().as_u16(), 403);
        let rest_resp = http
            .post(format!("{base}/api/v1/grasp/measure"))
            .bearer_auth(&tok)
            .json(&json!({"label": "cup"}))
            .send()
            .await
            .unwrap();
        assert_eq!(rest_resp.status().as_u16(), 200);
        let mut rest_json: Value = rest_resp.json().await.unwrap();
        rest_json.as_object_mut().unwrap().remove("age_ms");

        // WS lease-free measure: 403 without auth, 200 after auth, 400 on duplicate request_id.
        let (mut ws, _) = connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
        let rej_auth = common::ws_reply(
            &mut ws,
            json!({"type":"command", "request_id": 1, "path":"/api/v1/grasp/measure", "body":{}}),
            &["command_reject"],
        )
        .await;
        assert_eq!(rej_auth["code"], "OPERATOR_REQUIRED");

        let auth_ack = common::ws_reply(
            &mut ws,
            json!({"type":"auth", "token": tok}),
            &["auth_ack"],
        )
        .await;
        assert_eq!(auth_ack["role"], "operator");

        let mut ws_ack = common::ws_reply(
            &mut ws,
            json!({"type":"command", "request_id": 2, "path":"/api/v1/grasp/measure", "body":{"label":"cup"}}),
            &["command_ack", "command_reject"],
        )
        .await;
        assert_eq!(ws_ack["type"], "command_ack", "{ws_ack}");
        ws_ack.as_object_mut().unwrap().remove("age_ms");

        let dup_rej = common::ws_reply(
            &mut ws,
            json!({"type":"command", "request_id": 2, "path":"/api/v1/grasp/measure", "body":{}}),
            &["command_reject"],
        )
        .await;
        assert_eq!(dup_rej["code"], "INVALID_COMMAND");

        // Causally post-capture motion detection: start a jog while trigger is in flight -> 409 ARM_MOVING.
        let (mut ctrl, gen) = common::controller(addr).await;
        let ep0 = sup.get_observation().await.epoch_id;
        assert_eq!(
            common::rpc(&mut ctrl, gen, ep0, "/api/v1/torque", json!({"enable": true})).await["type"],
            "command_ack"
        );
        let ep1 = sup.get_observation().await.epoch_id;
        source.trigger_delay_ms.store(120, Ordering::SeqCst);
        let http_m = http.clone();
        let base_m = base.clone();
        let tok_m = tok.clone();
        let meas_fut = tokio::spawn(async move {
            let r = http_m
                .post(format!("{base_m}/api/v1/grasp/measure"))
                .bearer_auth(&tok_m)
                .json(&json!({}))
                .send()
                .await
                .unwrap();
            (r.status().as_u16(), r.json::<Value>().await.unwrap())
        });
        tokio::time::sleep(Duration::from_millis(30)).await;
        let mut target = common::INSPECT.to_vec();
        target[0] += 0.25;
        let jog_ack = common::rpc(
            &mut ctrl,
            gen,
            ep1,
            "/api/v1/motion/jog",
            json!({"target_positions": target, "duration_secs": 0.4}),
        )
        .await;
        assert_eq!(jog_ack["type"], "command_ack", "{jog_ack}");
        let (mv_status, mv_body) = meas_fut.await.unwrap();
        assert_eq!(mv_status, 409, "{owner:?}: {mv_body}");
        assert_eq!(mv_body["code"], "ARM_MOVING", "{owner:?}: {mv_body}");

        if let Some(h) = exec_h {
            h.stop_and_join();
        }
        if let Some(t) = leg_t {
            t.abort();
        }
        outputs.push((rest_json, ws_ack));
    }

    assert_eq!(outputs[0].0, outputs[1].0, "REST /api/v1/grasp/measure must match across Legacy and Executor");
    assert_eq!(outputs[0].1["table_fit"], outputs[1].1["table_fit"]);
    assert_eq!(outputs[0].1["detections"], outputs[1].1["detections"]);
    assert_eq!(outputs[0].1["calibration"], outputs[1].1["calibration"]);
    assert_eq!(outputs[0].1["camera"], outputs[1].1["camera"]);
}

// ============================================================================
// 6. Multiview REST + home/jog/unload & migrated action REST stubs
// ============================================================================

#[tokio::test]
async fn dual_owner_multiview_and_motion_action_rest_stubs_lease_contract() {
    let http = reqwest::Client::new();
    let detector = DetectorUpstream::parse("http://127.0.0.1:9").unwrap();
    let mut results = Vec::new();

    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let run = spawn_owner(
            owner,
            ArmConfig::rebot_dm(),
            detector.clone(),
            None,
            Arc::new(DummySource),
            OperatorAuth::enforced("123456", 1800),
            None,
            None,
        )
        .await;
        let base = format!("http://{}", run.addr);
        let tok = unlock_token(&http, run.addr, "123456").await;

        let stubs: &[(&str, Value, bool)] = &[
            ("/api/v1/motion/jog", json!({"target_positions": [0,0,0,0,0,0,0]}), false),
            ("/api/v1/motion/home", json!({}), false),
            ("/api/v1/motion/unload", json!({}), false),
            ("/api/v1/action/measure_multiview", json!({}), true),
            ("/api/v1/action/move_to", json!({}), false),
            ("/api/v1/action/move_relative", json!({}), false),
            ("/api/v1/action/gripper", json!({}), false),
            ("/api/v1/action/move_to_pose", json!({}), false),
            ("/api/v1/action/place", json!({}), false),
            ("/api/v1/action/replay", json!({}), false),
            ("/api/v1/action/run_sequence", json!({}), false),
            ("/api/v1/mode/gravity_comp", json!({}), true),
        ];

        // 1) Without active lease -> 409 LEASE_REQUIRED (or 403 OPERATOR_REQUIRED if unauth on op-gated).
        let mut without_lease = Vec::new();
        for (path, body, needs_op) in stubs {
            if *needs_op {
                let unauth = http.post(format!("{base}{path}")).json(body).send().await.unwrap();
                assert_eq!(unauth.status().as_u16(), 403, "{owner:?} {path} must require operator");
            }
            let r = http
                .post(format!("{base}{path}"))
                .bearer_auth(&tok)
                .json(body)
                .send()
                .await
                .unwrap();
            let v: Value = r.json().await.unwrap();
            without_lease.push((*path, v["code"].as_str().unwrap().to_string()));
        }

        // 2) With active WS lease -> 409 LEASE_ACTIVE.
        let (_ws, _gen) = common::controller(run.addr).await;
        tokio::time::sleep(Duration::from_millis(25)).await;
        let mut with_lease = Vec::new();
        for (path, body, _) in stubs {
            let r = http
                .post(format!("{base}{path}"))
                .bearer_auth(&tok)
                .json(body)
                .send()
                .await
                .unwrap();
            let v: Value = r.json().await.unwrap();
            with_lease.push((*path, v["code"].as_str().unwrap().to_string()));
        }

        // 3) Remaining LegacyOnly routes stay 503 EXECUTOR_ROUTING_PENDING on Executor.
        if owner == OwnerKind::Executor {
            for (method, path) in [
                ("POST", "/api/v1/action/gripper_zero_check"),
                ("POST", "/api/v1/dataset/record/start"),
                ("GET", "/api/v1/diagnostics/power"),
            ] {
                let r = if method == "POST" {
                    http.post(format!("{base}{path}")).bearer_auth(&tok).json(&json!({})).send().await.unwrap()
                } else {
                    http.get(format!("{base}{path}")).send().await.unwrap()
                };
                assert_eq!(r.status().as_u16(), 503, "{path} must stay LegacyOnly");
                assert_eq!(r.json::<Value>().await.unwrap()["code"], "EXECUTOR_ROUTING_PENDING");
            }
        }

        results.push((without_lease, with_lease));
    }

    assert_eq!(results[0], results[1]);
    for (p, code) in &results[1].0 {
        assert_eq!(code, "LEASE_REQUIRED", "{p}");
    }
    for (p, code) in &results[1].1 {
        assert_eq!(code, "LEASE_ACTIVE", "{p}");
    }
}

// ============================================================================
// 7. Grasp ActionStatus diagnostics via rt_core::stream patch (dual-owner)
// ============================================================================

struct DualGraspSim {
    sup: Arc<ArmSupervisor>,
    cam: CamModel,
    object_xy: [f64; 2],
    object_h: f64,
    frame_seq: AtomicU64,
}

impl DetectionSource for DualGraspSim {
    fn inference_enabled(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async { Ok(true) })
    }
    fn set_inference(&self, _enabled: bool) -> BoxFuture<'_, Result<(), String>> {
        Box::pin(async { Ok(()) })
    }
    fn per_request_pipelines(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async { Ok(true) })
    }
    fn mode_info(&self) -> BoxFuture<'_, Result<ModeInfo, String>> {
        let v = json!({
            "api_version": 2,
            "mode": "yolo",
            "enabled": false,
            "per_request_pipelines": true,
            "available": {"yolo": true, "depth": true},
            "model": {"kind": "segment", "name": "seg"},
            "depth_model": {"metric": true}
        });
        Box::pin(async move { Ok(ModeInfo::parse(Some(&v))) })
    }
    fn trigger_with_focus(
        &self,
        pipelines: &'static [&'static str],
        wait_ms: u64,
        _focus: Option<&grasp::MaskFocus>,
    ) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        self.trigger(pipelines, wait_ms)
    }
    fn next_frame(
        &self,
        _after: Option<u64>,
        _wait_ms: u64,
    ) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        self.trigger(&["yolo"], 200)
    }
    fn trigger(
        &self,
        pipelines: &[&str],
        _wait_ms: u64,
    ) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        let sup = self.sup.clone();
        let cam = self.cam.clone();
        let obj_xy = self.object_xy;
        let obj_h = self.object_h;
        let with_depth = pipelines.contains(&"depth");
        let fid = self.frame_seq.fetch_add(1, Ordering::SeqCst) + 1;
        Box::pin(async move {
            let obs = sup.get_observation().await;
            let env = sup.hub().prep_env().unwrap();
            let pose = env.config.arm_model.fk_f32(&obs.positions).unwrap();
            let target_top = [obj_xy[0], obj_xy[1], obj_h];
            let p_px = cam.project(&pose, &target_top).unwrap_or([cam.cx, cam.cy]);
            let half_w = 24.0;
            let half_h = 18.0;
            let x0 = ((p_px[0] - half_w) / cam.size[0]).clamp(0.05, 0.90);
            let y0 = ((p_px[1] - half_h) / cam.size[1]).clamp(0.05, 0.90);
            let x1 = ((p_px[0] + half_w) / cam.size[0]).clamp(x0 + 0.02, 0.95);
            let y1 = ((p_px[1] + half_h) / cam.size[1]).clamp(y0 + 0.02, 0.95);

            let o = cam.origin(&pose);
            let r = cam.rotation(&pose);
            let z_top = kinematics::dot(&kinematics::sub(&target_top, &o), &kinematics::column(&r, 2)).max(0.08);
            let z_bg = kinematics::dot(&kinematics::sub(&[obj_xy[0], obj_xy[1], 0.0], &o), &kinematics::column(&r, 2)).max(z_top + 0.01);

            let depth = with_depth.then(|| {
                let mut table_samples = Vec::new();
                for row in 0..4 {
                    for col in 0..4 {
                        let u_n = 0.15 + 0.2 * col as f64;
                        let v_n = 0.2 + 0.2 * row as f64;
                        let px = cam.to_px([u_n, v_n]);
                        if let Some(pt) = cam.plane_point(&pose, px, 0.0) {
                            let zc = kinematics::dot(&kinematics::sub(&pt, &o), &kinematics::column(&r, 2));
                            if zc > 0.05 {
                                table_samples.push([u_n, v_n, 1.0 / zc]);
                            }
                        }
                    }
                }
                DepthFrameInfo {
                    metric_model: true,
                    table_samples,
                }
            });

            let det = Detection {
                label: "cup".into(),
                score: 0.95,
                raw_score_valid: true,
                box_norm: [x0, y0, x1, y1],
                angle_deg: Some(12.0),
                angle_conf: Some(0.92),
                grasp_width_px: Some(34.0),
                mask_focus: false,
                disp_fg: with_depth.then_some(1.0 / z_top),
                disp_bg: with_depth.then_some(1.0 / z_bg),
                depth_conf: with_depth.then_some(0.95),
                length_px: Some(48.0),
                jaw_clear: with_depth.then_some(true),
                depth_angle_deg: None,
                depth_angle_conf: None,
                depth_width_px: None,
                contour_norm: None,
            };
            Ok(Some(DetectionFrame {
                frame_id: fid,
                captured_at: Instant::now(),
                policy_captured_at: None,
                image_size_px: cam.size,
                detections: vec![det],
                depth,
                model_generation: None,
            }))
        })
    }
}

#[tokio::test]
async fn dual_owner_grasp_action_status_diagnostics_via_stream_patch() {
    let detector = DetectorUpstream::parse("http://127.0.0.1:9").unwrap();
    let mut finals: Vec<ActionStatus> = Vec::new();

    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let mut cfg = ArmConfig {
            collision: None,
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        };
        cfg.camera_mount.offset_m = Some([0.0, -0.05, 0.02]);
        cfg.camera_mount.tilt_deg = 0.0;
        if let Some(ref mut g) = cfg.grasp {
            g.approach_speed_mps = 0.14;
            g.descend_speed_mps = 0.12;
            g.settle_delay_s = 0.01;
            g.stable_frames = 2;
            g.median_frames = 2;
            g.post_lift_settle_ms = 20;
            g.recentre_height_m = 0.0;
            g.height_sources = vec![grasp::HeightSource::Depth];
            g.width_sources = vec![grasp::WidthSource::Mask];
            g.yaw_sources = vec![grasp::YawSource::Mask, grasp::YawSource::Box];
        }
        // Mock gripper close detects holding cleanly when closing against a virtual object.
        if let Some(ref mut pol) = cfg.gripper_policy {
            pol.holding_mode = armd::model::HoldingMode::PositionGap;
            pol.holding_gap_rad = 0.0; // mock arm reaches closed_rad (gap = 0.0 >= 0.0 => held)
            pol.close_speed_rad_s = Some(2.0);
        }
        if let Some(ref mut dg) = cfg.damiao_gripper {
            dg.close_speed_rad_s = 2.0;
        }
        let setup = cfg.grasp.clone().unwrap();
        let cam = CamModel::assumed(&cfg.arm_model, &setup, &cfg.camera_mount);

        // Start from a clean look-down pose directly above a reachable table point [0.30, 0.0].
        let inspect6 = [0.0_f64, -1.167, -1.565, 1.435, 0.0, 0.0];
        let r0 = grasp::look_down(&cfg.arm_model.fk(&inspect6).r);
        let start_pose = kinematics::Pose::new(r0, [0.30, 0.0, 0.25]);
        let lo = [-3.1, -2.05, -2.6, -3.1, -2.05, -6.2];
        let hi = [3.1, 2.05, 2.6, 3.1, 2.05, 6.2];
        let mut start_q6 = inspect6;
        for _ in 0..20 {
            let sol = cfg
                .arm_model
                .solve_dls(&start_pose, &start_q6, &lo, &hi, kinematics::PAR.w);
            start_q6 = sol.q;
            if sol.ok {
                break;
            }
        }
        let start_q = [
            start_q6[0] as f32,
            start_q6[1] as f32,
            start_q6[2] as f32,
            start_q6[3] as f32,
            start_q6[4] as f32,
            start_q6[5] as f32,
            -5.65_f32,
        ];

        let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
        let obj_xy = {
            let mut inner = sup.inner().write().await;
            inner.commanded_positions = start_q.to_vec();
            inner.measured_positions = start_q.to_vec();
            inner.target_positions = start_q.to_vec();
            let pose0 = inner.config.arm_model.fk_f32(&start_q).unwrap();
            let pt = cam.plane_point(&pose0, cam.to_px(setup.grip_pixel_norm), 0.0).unwrap();
            [pt[0], pt[1]]
        };
        let sim = Arc::new(DualGraspSim {
            sup: sup.clone(),
            cam: cam.clone(),
            object_xy: obj_xy,
            object_h: 0.05,
            frame_seq: AtomicU64::new(0),
        });
        let tmp = tempfile::tempdir().unwrap();
        let grasp_deps = GraspDeps {
            source: sim.clone(),
            setup_path: Some(tmp.path().join("grasp_setup.json")),
        };
        let calib_deps = CalibrationDeps {
            path: tmp.path().join("calibration.json"),
            backup_dir: tmp.path().join("calib_backups"),
            sessions_dir: tmp.path().join("calib_sessions"),
            setup_path: grasp_deps.setup_path.clone(),
            upstream: detector.clone(),
            vision_timeout: Duration::from_secs(2),
        };
        let router = create_router_calibration_with_auth(
            sup.clone(),
            detector.clone(),
            grasp_deps,
            DiagnosticsState::global(),
            calib_deps,
            OperatorAuth::disabled(),
        );
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        tokio::spawn(async move {
            let _ = axum::serve(listener, router).await;
        });

        let (exec_h, leg_t) = match owner {
            OwnerKind::Executor => {
                let h = sup
                    .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
                    .await
                    .unwrap();
                wait_for_service_ready(&sup).await;
                (Some(h), None)
            }
            OwnerKind::Legacy => {
                let s = sup.clone();
                let jh = tokio::spawn(async move {
                    let mut tick = tokio::time::interval(Duration::from_millis(5));
                    loop {
                        tick.tick().await;
                        if s.executor_handle().is_some() {
                            break;
                        }
                        let mut inner = s.inner().write().await;
                        let _ = inner.tick(0.005);
                    }
                });
                wait_for_service_ready(&sup).await;
                (None, Some(jh))
            }
        };

        let (mut ws, gen) = common::controller(addr).await;
        let ep0 = sup.get_observation().await.epoch_id;
        assert_eq!(
            common::rpc(&mut ws, gen, ep0, "/api/v1/torque", json!({"enable": true})).await["type"],
            "command_ack"
        );
        let ep1 = sup.get_observation().await.epoch_id;

        let ack = common::rpc(
            &mut ws,
            gen,
            ep1,
            "/api/v1/action/grasp",
            json!({"label": "cup", "box_norm": [0.45, 0.45, 0.55, 0.55]}),
        )
        .await;
        assert_eq!(ack["type"], "command_ack", "{owner:?}: {ack}");
        let action_id = ack["action_id"].as_u64().unwrap();

        // Wait for terminal observation on the WS stream (also keeps WS ping/pong alive)
        // and verify GET /api/v1/observation matches it.
        let mut ws_terminal: Option<ActionStatus> = None;
        let dl = Instant::now() + Duration::from_secs(35);
        while Instant::now() < dl {
            if let Ok(Some(Ok(Message::Text(txt)))) =
                tokio::time::timeout(Duration::from_millis(50), ws.next()).await
            {
                if let Ok(v) = serde_json::from_str::<Value>(&txt) {
                    if v.get("type").and_then(Value::as_str) == Some("observation") {
                        if let Some(act_val) = v.get("action").filter(|a| !a.is_null()) {
                            if let Ok(a) = serde_json::from_value::<ActionStatus>(act_val.clone()) {
                                if a.id == action_id && !a.is_running() {
                                    ws_terminal = Some(a);
                                    break;
                                }
                            }
                        }
                    }
                }
            }
            let obs = sup.get_observation().await;
            if let Some(a) = obs.action.filter(|a| a.id == action_id && !a.is_running()) {
                ws_terminal = Some(a);
                break;
            }
        }
        let action = match ws_terminal {
            Some(a) => a,
            None => {
                let latest = sup.get_observation().await.action;
                panic!("{owner:?} grasp {action_id} did not finish; latest={latest:?}");
            }
        };
        assert_eq!(action.state.as_str(), "done", "{owner:?}: {action:?}");

        if owner == OwnerKind::Executor {
            let stats = sup.hub().action_diag_stats();
            assert!(stats.pushed > 0 && stats.dropped == 0, "executor grasp diag stats: {stats:?}");
        }

        if let Some(h) = exec_h {
            h.stop_and_join();
        }
        if let Some(t) = leg_t {
            t.abort();
        }
        finals.push(action);
    }

    let (leg, exec) = (&finals[0], &finals[1]);
    // Core diagnostic parity between Legacy and Executor!
    assert_eq!(leg.phase, exec.phase);
    assert_eq!(leg.progress, exec.progress);
    assert_eq!(leg.holding, exec.holding);
    assert_eq!(leg.height_source, exec.height_source);
    assert_eq!(leg.depth_anchor, exec.depth_anchor);
    assert_eq!(leg.jaw_clear, exec.jaw_clear);
    assert_eq!(leg.align, exec.align);
    assert_eq!(leg.pre_open, exec.pre_open);
    assert_eq!(leg.recentre, exec.recentre);
    assert_eq!(leg.post_lift, exec.post_lift);
    assert_eq!(leg.descend_contact, exec.descend_contact);
    assert!(exec.descend_contact.is_none(), "no contact must leave descend_contact None (absent)");
    assert!(exec.object_height_m.is_some() && leg.object_height_m.is_some());
    assert!(exec.close_height_m.is_some() && leg.close_height_m.is_some());
    assert!(exec.object_depth_m.is_some() && leg.object_depth_m.is_some());
    assert!(exec.grasp_width_m.is_some() && leg.grasp_width_m.is_some());
    assert!(exec.held_width_m.is_some() && leg.held_width_m.is_some());
    assert_eq!(exec.height_estimates.len(), leg.height_estimates.len());
    assert!(!exec.height_estimates.is_empty());
    assert_eq!(exec.yaw_estimates.len(), leg.yaw_estimates.len());
    assert!(!exec.yaw_estimates.is_empty());
    assert_eq!(exec.width_estimates.len(), leg.width_estimates.len());
    assert!(!exec.width_estimates.is_empty());
    assert!(!exec.phase_ms.is_empty(), "phase_ms must be populated on executor");
    assert!(!exec.motions.is_empty(), "motions must be populated on executor");
    assert_eq!(exec.motions.len(), leg.motions.len());

    // Wire JSON check: absent descend_contact must NOT appear as `"descend_contact": false`.
    let exec_json = serde_json::to_value(exec).unwrap();
    assert!(exec_json.get("descend_contact").is_none(), "{exec_json}");
}

// ============================================================================
// 8. ActionDiagPatch stream overflow, atomic estimates, lossless strings & freeze
// ============================================================================

fn blank_action_status(id: u64, state: &str) -> ActionStatus {
    serde_json::from_value(json!({
        "id": id,
        "kind": "grasp",
        "state": state,
        "progress": 0.0,
        "started_at": 0.0,
    }))
    .expect("deserialize blank ActionStatus")
}

#[test]
fn action_diag_stream_overflow_atomic_estimates_lossless_strings_and_terminal_seal() {
    let (_pub, hub) = state_channel(7);
    assert!(std::mem::size_of::<ActionDiagPatch>() <= 128);

    // 1) Lossless >200-byte strings + WorkspaceViolation + Approach + 32-motion cap + absence semantics.
    let long_cid = "c".repeat(220);
    let long_gid = "g".repeat(220);
    let long_reason = "r".repeat(240);
    hub.push_action_workspace_violation(
        10,
        WorkspaceViolationDetail {
            constraint_id: Some(long_cid.clone()),
            geometry_id: Some(long_gid.clone()),
            violation_phase: Some("descend".into()),
            reason: Some(long_reason.clone()),
        },
    );
    let long_hr = hub.intern_action_diag_str(10, &"h".repeat(210));
    assert!(hub
        .push_action_diag(ActionDiagPatch {
            action_id: 10,
            kind: ActionDiagKind::HeightReason { reason: long_hr },
        })
        .is_ok());
    // DescendContact with contact_height_m: None MUST keep descend_contact == None (absent in JSON).
    assert!(hub
        .push_action_diag(ActionDiagPatch {
            action_id: 10,
            kind: ActionDiagKind::DescendContact {
                peak_residual_nm: Some([0.4, 0.5]),
                peak_tracking_rad: Some(0.01),
                contact_height_m: None,
            },
        })
        .is_ok());
    // Push 40 MotionDiag entries -> capped at 32.
    for i in 0..40u32 {
        assert!(hub
            .push_action_diag(ActionDiagPatch {
                action_id: 10,
                kind: ActionDiagKind::MotionDiag {
                    phase: ShortLabel::new_truncated("approach"),
                    ms: i,
                    track_mm: 0.5,
                    track_deg: 0.2,
                    goal_mm: 0.1,
                    goal_deg: 0.05,
                    contact: None,
                },
            })
            .is_ok());
    }
    // Approach with >1 flipped joint name, then interrupted by contact.
    hub.push_action_approach(
        10,
        &armd::backlash::Outcome::same_side(&[1, -1, 1, 0, 0, 0], 6, vec!["joint_2".into(), "joint_3".into()]),
    );
    assert!(hub
        .push_action_diag(ActionDiagPatch {
            action_id: 10,
            kind: ActionDiagKind::Border {
                kind: BorderKind::Shifted,
                shift_px: Some([12.0, -4.0]),
            },
        })
        .is_ok());
    assert!(hub
        .push_action_diag(ActionDiagPatch {
            action_id: 10,
            kind: ActionDiagKind::PostLift {
                kind: PostLiftKind::Held,
                gripper_gap_rad: None,
            },
        })
        .is_ok());

    let mut action = blank_action_status(10, "done");
    action.phase = Some(armd::model::HolderLabel::new("done"));
    action.progress = 1.0;
    hub.seal_and_attach_action_diag_to_action(&mut action);

    assert_eq!(action.constraint_id.as_deref(), Some(long_cid.as_str()));
    assert_eq!(action.geometry_id.as_deref(), Some(long_gid.as_str()));
    assert_eq!(action.reason.as_deref(), Some(long_reason.as_str()));
    assert_eq!(action.height_reason.as_deref(), Some("h".repeat(210).as_str()));
    assert_eq!(action.motions.len(), 32);
    assert_eq!(action.descend_contact, None);
    assert_eq!(action.descend_peak_residual_nm, Some([0.4, 0.5]));
    assert_eq!(action.approach.as_deref(), Some("same_side"));
    assert_eq!(action.approach_flipped, Some(vec!["joint_2".to_string(), "joint_3".to_string()]));
    assert_eq!(action.border.as_deref(), Some("shifted"));
    assert_eq!(action.border_shift_px, Some([12.0, -4.0]));
    assert_eq!(action.post_lift.as_deref(), Some("held"));
    assert_eq!(action.phase.as_ref().map(|p| p.as_str()), Some("done"));
    assert_eq!(action.progress, 1.0);
    let serialized = serde_json::to_value(&action).unwrap();
    assert!(serialized.get("descend_contact").is_none());

    // 2) Single terminal freeze cutoff: late patches pushed AFTER seal are ignored!
    assert!(hub
        .push_action_diag(ActionDiagPatch {
            action_id: 10,
            kind: ActionDiagKind::PostLift {
                kind: PostLiftKind::Dropped,
                gripper_gap_rad: Some(0.01),
            },
        })
        .is_ok());
    let mut action_again = blank_action_status(10, "done");
    hub.seal_and_attach_action_diag_to_action(&mut action_again);
    assert_eq!(action_again.post_lift.as_deref(), Some("held"));
    assert_eq!(action_again.gripper_gap_rad, None);

    // 3) Paused consumer overflow + drop-newest loss counters + sequence gaps + atomic estimate staging.
    hub.push_action_estimates(
        20,
        EstimateListKind::Height,
        &[EstimateEntry {
            source: "depth".into(),
            value: Some(0.05),
            used: true,
            record_only: false,
            reason: None,
        }],
    );
    hub.drain_action_diag();

    hub.set_action_diag_consumer_paused_for_tests(true);
    let mut full_seen = false;
    for i in 0..(ACTION_DIAG_STREAM_CAP + 20) {
        let r = hub.push_action_diag(ActionDiagPatch {
            action_id: 20,
            kind: ActionDiagKind::Phase {
                phase: ShortLabel::new_truncated("centring"),
                progress: 0.3,
                iteration: Some(i as u32),
                error_px: Some(1.0),
                at: Instant::now(),
            },
        });
        if r == Err(StreamPushError::Full) {
            full_seen = true;
            break;
        }
    }
    assert!(full_seen, "paused consumer must trigger StreamPushError::Full");
    // Now push a 2-item estimate replacement while ring is full -> dropped, partial batch never commits.
    let partial_err = hub.push_action_diag(ActionDiagPatch {
        action_id: 20,
        kind: ActionDiagKind::EstimateBatchItem {
            list: EstimateListKind::Height,
            batch_gen: 99,
            index: 0,
            total_len: 2,
            source: ShortLabel::new_truncated("scale"),
            value: Some(0.09),
            used: true,
            record_only: false,
            reason: None,
        },
    });
    assert_eq!(partial_err, Err(StreamPushError::Full));

    let stats_mid = hub.action_diag_stats();
    assert!(stats_mid.dropped_full >= 2, "{stats_mid:?}");

    hub.set_action_diag_consumer_paused_for_tests(false);
    hub.drain_action_diag();
    // Push one post-gap frame so the consumer observes the exact sequence gap.
    assert!(hub
        .push_action_diag(ActionDiagPatch {
            action_id: 20,
            kind: ActionDiagKind::CloseHeight { close_height_m: 0.03 },
        })
        .is_ok());
    let mut a20 = blank_action_status(20, "done");
    hub.seal_and_attach_action_diag_to_action(&mut a20);
    let stats_end = hub.action_diag_stats();
    assert!(stats_end.sequence_gaps >= 1 && stats_end.lost_in_gaps >= 2, "{stats_end:?}");
    assert_eq!(a20.close_height_m, Some(0.03));
    // Previous valid estimate list remains intact (no half-replaced list!).
    assert_eq!(a20.height_estimates.len(), 1);
    assert_eq!(a20.height_estimates[0].source, "depth");

    // 4) ApproachKind::Direct roundtrip.
    assert!(hub
        .push_action_diag(ActionDiagPatch {
            action_id: 30,
            kind: ActionDiagKind::Approach {
                kind: ApproachKind::Direct,
                reason: Some(DiagStr::Inline(ShortLabel::new_truncated("request"))),
                signs: [0; 6],
                signs_len: 0,
                flipped_blob: None,
            },
        })
        .is_ok());
    let mut a30 = blank_action_status(30, "done");
    hub.seal_and_attach_action_diag_to_action(&mut a30);
    assert_eq!(a30.approach.as_deref(), Some("direct"));
    assert_eq!(a30.approach_reason.as_deref(), Some("request"));
    assert!(a30.approach_signs.is_none());
}
