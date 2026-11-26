//! `POST /api/v1/vision/detector`: detection kind switch between the default detect / segment
//! models, against a mock vision provider (`GET /api/mode` body + `POST /api/models`).
use std::sync::{Arc, Mutex};

use serde_json::{json, Value};

use armd::grasp::{BoxFuture, DetectionFrame, DetectionSource, GraspDeps, ModeInfo};
use armd::model::ArmConfig;
use armd::{ArmSupervisor, OperatorAuth};

/// The measurement counter and the model-switch lock are process-wide: one test at a time.
static SERIAL: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());

const DET: &str = "yolo11n_detection_cv181x_int8";
const SEG384: &str = "yolo11n_seg_384x640_cv181x_int8_fuse";
const SEG: &str = "yolo11n_segment_cv181x_int8";
const DEPTH: &str = "zipdepth_metric_384x224_bf16_rgb";

/// rebot-vision's mode and model state; `set_models` applies a swap at once unless `reply` is set.
struct Provider {
    mode: Mutex<Value>,
    reply: Mutex<Option<(u16, Value)>>,
    calls: Mutex<Vec<Value>>,
}

impl Provider {
    fn new(catalogue: &[&str], detector: &str) -> Arc<Self> {
        let cat: Vec<Value> = catalogue.iter().map(|n| {
            let (kind, ion_mb) = if *n == DEPTH || n.contains("depth") {
                ("depth", 20.0)
            } else if *n == SEG {
                ("segment", 19.5)
            } else if n.contains("seg") {
                ("segment", 13.1)
            } else {
                ("detect", 8.5)
            };
            json!({"name": n, "kind": kind, "ion_mb": ion_mb})
        }).collect();
        Arc::new(Self {
            mode: Mutex::new(json!({"api_version": 2, "mode": "yolo", "enabled": false, "available": {"yolo": true, "depth": false},
                "model": {"kind": if detector.contains("seg") { "segment" } else { "detect" }, "name": detector},
                "ion": {"base_mb": 21.5, "total_mb": 62.9, "reserve_mb": 8.0},
                "models": {"generation": 0, "resident": {"detector": detector, "depth": null}, "swap": {"state": "idle", "generation": 0}, "catalogue": cat}})),
            reply: Mutex::new(None),
            calls: Mutex::new(Vec::new()),
        })
    }
}

impl DetectionSource for Provider {
    fn inference_enabled(&self) -> BoxFuture<'_, Result<bool, String>> { Box::pin(async { Ok(false) }) }
    fn set_inference(&self, _enabled: bool) -> BoxFuture<'_, Result<(), String>> { Box::pin(async { Ok(()) }) }
    fn next_frame(&self, _after: Option<u64>, _wait_ms: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> { Box::pin(async { Ok(None) }) }
    fn mode_info(&self) -> BoxFuture<'_, Result<ModeInfo, String>> {
        let v = self.mode.lock().unwrap().clone();
        Box::pin(async move { Ok(ModeInfo::parse(Some(&v))) })
    }
    fn set_models(&self, body: &Value) -> BoxFuture<'_, Result<(u16, Value), String>> {
        self.calls.lock().unwrap().push(body.clone());
        if let Some(r) = self.reply.lock().unwrap().clone() { return Box::pin(async move { Ok(r) }); }
        let mut m = self.mode.lock().unwrap();
        let gen = m["models"]["generation"].as_u64().unwrap() + 1;
        m["models"]["generation"] = json!(gen);
        if let Some(det) = body.get("detector") {
            m["models"]["resident"]["detector"] = det.clone();
            if let Some(name) = det.as_str() {
                m["model"] = json!({"kind": if name.contains("seg") { "segment" } else { "detect" }, "name": name});
                m["available"]["yolo"] = json!(true);
            } else {
                m["model"] = Value::Null;
                m["available"]["yolo"] = json!(false);
            }
        }
        if let Some(dep) = body.get("depth") {
            m["models"]["resident"]["depth"] = dep.clone();
            m["available"]["depth"] = json!(dep.is_string());
        }
        Box::pin(async move { Ok((202, json!({"generation": gen}))) })
    }
}

async fn serve(provider: Arc<Provider>, auth: OperatorAuth, config: ArmConfig) -> std::net::SocketAddr {
    let sup = Arc::new(ArmSupervisor::new(config));
    let deps = GraspDeps { source: provider, setup_path: None };
    let router = armd::api::create_router_with(sup, armd::api::proxy::DetectorUpstream::parse("http://127.0.0.1:9").unwrap(), deps, auth);
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });
    addr
}

async fn post(addr: std::net::SocketAddr, body: Value, token: Option<&str>) -> (u16, Value) {
    let mut req = reqwest::Client::new().post(format!("http://{addr}/api/v1/vision/detector")).json(&body);
    if let Some(t) = token { req = req.bearer_auth(t); }
    let r = req.send().await.unwrap();
    (r.status().as_u16(), r.json().await.unwrap_or(Value::Null))
}

#[tokio::test]
async fn switches_to_the_default_model_of_a_kind() {
    let _serial = SERIAL.lock().await;
    let p = Provider::new(&[DET, SEG, SEG384], DET);
    let addr = serve(p.clone(), OperatorAuth::disabled(), ArmConfig::rebot_dm()).await;
    // segment: the first default name in the catalogue (384x640 before 640x640)
    let (s, v) = post(addr, json!({"kind": "segment"}), None).await;
    assert_eq!(s, 200, "{v}");
    assert_eq!((v["kind"].as_str(), v["model"].as_str(), v["changed"].as_bool(), v["generation"].as_u64()), (Some("segment"), Some(SEG384), Some(true), Some(1)));
    assert_eq!(*p.calls.lock().unwrap(), vec![json!({"detector": SEG384})]);
    // already loaded: nothing is sent
    let (s, v) = post(addr, json!({"kind": "segment"}), None).await;
    assert_eq!((s, v["changed"].as_bool()), (200, Some(false)));
    assert_eq!(p.calls.lock().unwrap().len(), 1);
    // back to boxes
    let (s, v) = post(addr, json!({"kind": "detect"}), None).await;
    assert_eq!((s, v["model"].as_str(), v["changed"].as_bool()), (200, Some(DET), Some(true)));
    // without the 384x640 model the next default name is used
    let p2 = Provider::new(&[DET, SEG], DET);
    let addr2 = serve(p2.clone(), OperatorAuth::disabled(), ArmConfig::rebot_dm()).await;
    let (s, v) = post(addr2, json!({"kind": "segment"}), None).await;
    assert_eq!((s, v["model"].as_str()), (200, Some(SEG)));
    // the schema announces the route
    let schema: Value = reqwest::get(format!("http://{addr}/api/v1/observation/schema")).await.unwrap().json().await.unwrap();
    assert_eq!(schema["capabilities"]["vision"]["detector_switch"], true);
}

#[tokio::test]
async fn profile_defaults_and_missing_models() {
    let _serial = SERIAL.lock().await;
    let p = Provider::new(&[DET, SEG], DET);
    let mut config = ArmConfig::rebot_dm();
    config.vision_detectors = Some(armd::vision_detector::DetectorDefaults { detect: vec![DET.into()], segment: vec!["my_seg".into()] });
    let addr = serve(p.clone(), OperatorAuth::disabled(), config).await;
    let (s, v) = post(addr, json!({"kind": "segment"}), None).await;
    assert_eq!((s, v["code"].as_str()), (422, Some("MODEL_NOT_FOUND")), "{v}");
    let msg = v["message"].as_str().unwrap_or_default().to_string();
    assert!(msg.contains("my_seg") && msg.contains("vision_detectors.segment"), "{msg}");
    assert!(p.calls.lock().unwrap().is_empty());
    // a provider without model switching
    let old = Provider::new(&[DET], DET);
    old.mode.lock().unwrap().as_object_mut().unwrap().remove("models");
    let addr = serve(old, OperatorAuth::disabled(), ArmConfig::rebot_dm()).await;
    let (s, v) = post(addr, json!({"kind": "segment"}), None).await;
    assert_eq!((s, v["code"].as_str()), (422, Some("UNSUPPORTED")), "{v}");
}

#[tokio::test]
async fn unknown_kind_and_bad_bodies_are_refused() {
    let _serial = SERIAL.lock().await;
    let p = Provider::new(&[DET, SEG384], DET);
    let addr = serve(p.clone(), OperatorAuth::disabled(), ArmConfig::rebot_dm()).await;
    for body in [json!({"kind": "obb"}), json!({}), json!({"kind": "detect", "model": "x"}), json!("detect")] {
        let (s, v) = post(addr, body.clone(), None).await;
        assert_eq!((s, v["code"].as_str()), (400, Some("INVALID_COMMAND")), "{body}: {v}");
    }
    assert!(p.calls.lock().unwrap().is_empty());
}

#[tokio::test]
async fn budget_refusal_is_passed_through() {
    let _serial = SERIAL.lock().await;
    let p = Provider::new(&[DET, SEG], DET);
    *p.reply.lock().unwrap() = Some((409, json!({"api_version": 2, "error": "ION_BUDGET", "need_mb": 69.5, "limit_mb": 62.9})));
    let addr = serve(p.clone(), OperatorAuth::disabled(), ArmConfig::rebot_dm()).await;
    let (s, v) = post(addr, json!({"kind": "segment"}), None).await;
    assert_eq!((s, v["code"].as_str()), (409, Some("ION_BUDGET")), "{v}");
    let msg = v["message"].as_str().unwrap_or_default().to_string();
    assert!(msg.contains("69.5") && msg.contains("62.9") && msg.contains(SEG), "{msg}");
    // a busy provider swap
    *p.reply.lock().unwrap() = Some((409, json!({"error": "SWAP_BUSY"})));
    let (s, v) = post(addr, json!({"kind": "segment"}), None).await;
    assert_eq!((s, v["code"].as_str()), (409, Some("VISION_BUSY")), "{v}");
}

#[tokio::test]
async fn busy_while_a_measurement_runs() {
    let _serial = SERIAL.lock().await;
    let p = Provider::new(&[DET, SEG384], DET);
    let addr = serve(p.clone(), OperatorAuth::disabled(), ArmConfig::rebot_dm()).await;
    let guard = armd::vision_detector::measure_in_flight();
    let (s, v) = post(addr, json!({"kind": "segment"}), None).await;
    assert_eq!((s, v["code"].as_str()), (409, Some("VISION_BUSY")), "{v}");
    let msg = v["message"].as_str().unwrap_or_default().to_string();
    assert!(msg.contains("measurement") && msg.contains("Wait"), "{msg}");
    assert!(p.calls.lock().unwrap().is_empty());
    drop(guard);
    let (s, _) = post(addr, json!({"kind": "segment"}), None).await;
    assert_eq!(s, 200);
}

#[tokio::test]
async fn operator_required_without_unlock() {
    let _serial = SERIAL.lock().await;
    let p = Provider::new(&[DET, SEG384], DET);
    let addr = serve(p.clone(), OperatorAuth::enforced("123456", 1800), ArmConfig::rebot_dm()).await;
    let (s, v) = post(addr, json!({"kind": "segment"}), None).await;
    assert_eq!((s, v["code"].as_str()), (403, Some("OPERATOR_REQUIRED")), "{v}");
    assert!(p.calls.lock().unwrap().is_empty());
    let r: Value = reqwest::Client::new().post(format!("http://{addr}/api/v1/auth/unlock")).json(&json!({"pin": "123456", "set_cookie": false}))
        .send().await.unwrap().json().await.unwrap();
    let (s, v) = post(addr, json!({"kind": "segment"}), r["token"].as_str()).await;
    assert_eq!((s, v["model"].as_str()), (200, Some(SEG384)), "{v}");
}

async fn post_models(addr: std::net::SocketAddr, body: Value, token: Option<&str>) -> (u16, Value) {
    let mut req = reqwest::Client::new().post(format!("http://{addr}/api/v1/vision/models")).json(&body);
    if let Some(t) = token { req = req.bearer_auth(t); }
    let r = req.send().await.unwrap();
    (r.status().as_u16(), r.json().await.unwrap_or(Value::Null))
}

#[tokio::test]
async fn vision_models_switches_variants_and_depth_with_ion_budget() {
    let _serial = SERIAL.lock().await;
    let p = Provider::new(&[DET, SEG384, SEG, DEPTH], SEG384);
    let addr = serve(p.clone(), OperatorAuth::disabled(), ArmConfig::rebot_dm()).await;

    // Schema announces model_switch
    let schema: Value = reqwest::get(format!("http://{addr}/api/v1/observation/schema")).await.unwrap().json().await.unwrap();
    assert_eq!(schema["capabilities"]["vision"]["model_switch"], true);

    // Load depth alongside seg384 (21.5 + 8.0 + 13.1 + 20.0 = 62.6 <= 62.9)
    let (s, v) = post_models(addr, json!({"depth": DEPTH}), None).await;
    assert_eq!(s, 200, "{v}");
    assert_eq!((v["detector"].as_str(), v["depth"].as_str(), v["changed"].as_bool()), (Some(SEG384), Some(DEPTH), Some(true)));

    // Same request is a no-op
    let calls_before = p.calls.lock().unwrap().len();
    let (s, v) = post_models(addr, json!({"depth": DEPTH}), None).await;
    assert_eq!((s, v["changed"].as_bool()), (200, Some(false)));
    assert_eq!(p.calls.lock().unwrap().len(), calls_before);

    // Switching to larger SEG (19.5 MB) with DEPTH (20.0 MB) exceeds 62.9 MB -> 409 ION_BUDGET (cause + fix)
    let (s, v) = post_models(addr, json!({"detector": SEG}), None).await;
    assert_eq!((s, v["code"].as_str()), (409, Some("ION_BUDGET")), "{v}");
    let msg = v["message"].as_str().unwrap_or_default();
    assert!(msg.contains("69.0") && msg.contains("62.9") && msg.to_lowercase().contains("depth"), "{msg}");

    // Atomic swap: unload depth + load SEG (21.5 + 8.0 + 19.5 = 49.0 <= 62.9) succeeds
    let (s, v) = post_models(addr, json!({"detector": SEG, "depth": null}), None).await;
    assert_eq!(s, 200, "{v}");
    assert_eq!((v["detector"].as_str(), v["depth"].is_null(), v["changed"].as_bool()), (Some(SEG), true, Some(true)));

    // Wrong slot kind -> 400 INVALID_COMMAND
    let (s, v) = post_models(addr, json!({"detector": DEPTH}), None).await;
    assert_eq!((s, v["code"].as_str()), (400, Some("INVALID_COMMAND")), "{v}");
    let (s, v) = post_models(addr, json!({"depth": DET}), None).await;
    assert_eq!((s, v["code"].as_str()), (400, Some("INVALID_COMMAND")), "{v}");

    // Unknown model -> 422 MODEL_NOT_FOUND
    let (s, v) = post_models(addr, json!({"detector": "missing_model"}), None).await;
    assert_eq!((s, v["code"].as_str()), (422, Some("MODEL_NOT_FOUND")), "{v}");

    // Bad bodies -> 400 INVALID_COMMAND
    for bad in [json!({}), json!({"foo": "bar"}), json!({"detector": 123}), json!(["seg"])] {
        let (s, v) = post_models(addr, bad.clone(), None).await;
        assert_eq!((s, v["code"].as_str()), (400, Some("INVALID_COMMAND")), "{bad}: {v}");
    }
}

#[tokio::test]
async fn vision_models_requires_operator_and_respects_busy() {
    let _serial = SERIAL.lock().await;
    let p = Provider::new(&[DET, SEG384, DEPTH], DET);
    let addr = serve(p.clone(), OperatorAuth::enforced("123456", 1800), ArmConfig::rebot_dm()).await;

    let (s, v) = post_models(addr, json!({"detector": SEG384}), None).await;
    assert_eq!((s, v["code"].as_str()), (403, Some("OPERATOR_REQUIRED")), "{v}");

    let r: Value = reqwest::Client::new().post(format!("http://{addr}/api/v1/auth/unlock")).json(&json!({"pin": "123456", "set_cookie": false}))
        .send().await.unwrap().json().await.unwrap();
    let tok = r["token"].as_str();

    let guard = armd::vision_detector::measure_in_flight();
    let (s, v) = post_models(addr, json!({"detector": SEG384}), tok).await;
    assert_eq!((s, v["code"].as_str()), (409, Some("VISION_BUSY")), "{v}");
    drop(guard);

    let (s, v) = post_models(addr, json!({"detector": SEG384}), tok).await;
    assert_eq!((s, v["detector"].as_str()), (200, Some(SEG384)), "{v}");
}
