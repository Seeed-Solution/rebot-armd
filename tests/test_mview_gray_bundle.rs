//! End-to-end test of raw gray debug bundles (PGM v2), replay parity, `debug: false`, and debug
//! write failure handling. Isolated in its own binary so `ARMD_MVIEW_DEBUG_DIR` never races
//! `tests/test_measure_multiview.rs`.
mod common;
mod render;

use std::collections::VecDeque;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::grasp::{BoxFuture, CamModel, Detection, DetectionFrame, DetectionSource, GraspDeps, GrayFrame, GrayFrameError, GrayTriggerResult};
use armd::measure_multiview as mm;
use armd::{ArmConfig, ArmSupervisor, SupervisorInner};
use render::{BoxObj, Noise, Scene};
use serde_json::{json, Value};

const START: [f32; 7] = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0, 0.0];
const HEIGHT: f64 = 0.040;

#[derive(Clone)]
struct GrayShot {
    id: u64,
    img: armd::mview_stereo::Gray,
    box_norm: Option<[f64; 4]>,
}

struct GrayOnlyVision {
    sup: Arc<ArmSupervisor>,
    cam: CamModel,
    scene: Arc<Scene>,
    frame: AtomicU64,
    ring: Mutex<VecDeque<GrayShot>>,
}

impl GrayOnlyVision {
    async fn render_shot(&self, id: u64) -> GrayShot {
        let pose = {
            let inner = self.sup.inner().read().await;
            inner.config.arm_model.fk_f32(&inner.measured_positions).unwrap()
        };
        let cam = mm::camera_at(&self.cam, &pose);
        let (w, h) = (self.cam.size[0] as usize, self.cam.size[1] as usize);
        let scene = Arc::clone(&self.scene);
        let img = tokio::task::spawn_blocking(move || {
            render::render(&scene, &cam, w, h, 2, Noise { sigma: 1.0, ..Noise::default() }, id)
        })
        .await
        .unwrap();
        let b = render::box_px(&self.scene, &self.scene.boxes[0], &cam);
        let inside = b[0] > 0.0 && b[1] > 0.0 && b[2] < w as f64 && b[3] < h as f64;
        let box_norm = inside.then(|| [b[0] / w as f64, b[1] / h as f64, b[2] / w as f64, b[3] / h as f64]);
        GrayShot { id, img, box_norm }
    }
}

impl DetectionSource for GrayOnlyVision {
    fn per_request_pipelines(&self) -> BoxFuture<'_, Result<bool, String>> { Box::pin(async { Ok(true) }) }
    fn gray_available(&self) -> BoxFuture<'_, Result<bool, String>> { Box::pin(async { Ok(true) }) }
    fn inference_enabled(&self) -> BoxFuture<'_, Result<bool, String>> { Box::pin(async { Ok(true) }) }
    fn set_inference(&self, _e: bool) -> BoxFuture<'_, Result<(), String>> { Box::pin(async { Ok(()) }) }
    fn next_frame(&self, _a: Option<u64>, _w: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        Box::pin(async { Ok(None) })
    }
    fn trigger_gray(&self, _p: &'static [&'static str], _w: u64) -> BoxFuture<'_, Result<Option<GrayTriggerResult>, String>> {
        Box::pin(async move {
            let id = self.frame.fetch_add(1, Ordering::SeqCst) + 1;
            let shot = self.render_shot(id).await;
            let detections = shot.box_norm.map(|b| vec![Detection {
                label: "box".into(), score: 0.9, raw_score_valid: true, box_norm: b,
                angle_deg: None, angle_conf: None, grasp_width_px: None, mask_focus: false,
                disp_fg: None, disp_bg: None, depth_conf: None, length_px: None, jaw_clear: None,
                depth_angle_deg: None, depth_angle_conf: None, depth_width_px: None, contour_norm: None,
            }]).unwrap_or_default();
            {
                let mut ring = self.ring.lock().unwrap();
                if ring.len() >= 4 { ring.pop_front(); }
                ring.push_back(shot);
            }
            Ok(Some(GrayTriggerResult {
                frame: DetectionFrame {
                    frame_id: id, captured_at: Instant::now(), policy_captured_at: None,
                    image_size_px: self.cam.size, detections, depth: None, model_generation: None,
                },
                markers: Vec::new(),
                instance_id: Some("rv-bundle-1".into()),
                gray_ran: true,
                gray_ms: Some(4.3),
            }))
        })
    }
    fn gray_frame(&self, instance_id: &str, frame_id: u64) -> BoxFuture<'_, Result<GrayFrame, GrayFrameError>> {
        let req_inst = instance_id.to_string();
        Box::pin(async move {
            if req_inst != "rv-bundle-1" {
                return Err(GrayFrameError::Expired("410 FRAME_EXPIRED".into()));
            }
            let shot = self.ring.lock().unwrap().iter().rev().find(|s| s.id == frame_id).cloned()
                .ok_or_else(|| GrayFrameError::NotStored("404 FRAME_NOT_STORED".into()))?;
            Ok(GrayFrame {
                instance_id: req_inst, frame_id, img: shot.img,
                frame_size: [640, 640], crop: [0, 140, 640, 360],
                ts_ms: 1763791234000 + frame_id, frame_mono_ms: 10000 + frame_id,
                wait_mono_ms: 9990 + frame_id, grab_mono_ms: 10002 + frame_id,
                stored_mono_ms: 10007 + frame_id, convert_ms: 4.3, fetch_ms: 0.8, parse_ms: 0.2,
            })
        })
    }
}

/// Runs one `measure_multiview` session.
/// When `settle_min_s` is `None`, the request omits `settle_min_s` (exercising the default 1.0 s
/// settle), and `idle_tick` sets the feedback cadence while settled (20 ms = real 50 Hz cadence).
async fn run_one(debug: bool, settle_min_s: Option<f64>, idle_tick: Duration) -> (Value, u64) {
    let config = ArmConfig::rebot_dm();
    let setup = SupervisorInner::new(config.clone()).grasp_setup().unwrap();
    let cam = CamModel::assumed(&config.arm_model, &setup, &config.camera_mount);
    let pose = config.arm_model.fk_f32(&START).unwrap();
    let look = cam.plane_point(&pose, [cam.cx, cam.cy], setup.table_z_m).unwrap();
    let sup = Arc::new(ArmSupervisor::new(config));
    let scene = Arc::new(Scene::new(0.0, vec![BoxObj { xy: [look[0], look[1]], yaw: 0.3, size: [0.05, 0.03, HEIGHT] }]));
    let vision = Arc::new(GrayOnlyVision {
        sup: sup.clone(), cam, scene, frame: AtomicU64::new(0), ring: Mutex::new(VecDeque::new()),
    });
    let deps = GraspDeps { source: vision, setup_path: None };
    let router = armd::api::create_router_with(
        sup.clone(), armd::api::proxy::DetectorUpstream::parse("http://127.0.0.1:9").unwrap(),
        deps, armd::OperatorAuth::disabled(),
    );
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let srv = tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });
    let ticker = sup.clone();
    let ticker_task = tokio::spawn(async move {
        loop {
            let moving = ticker.inner().read().await.active_trajectory.is_some();
            if moving {
                for _ in 0..4 {
                    ticker.tick(0.02).await;
                    if ticker.inner().read().await.active_trajectory.is_none() { break; }
                }
                tokio::time::sleep(Duration::from_millis(1)).await;
            } else {
                tokio::time::sleep(idle_tick).await;
                ticker.tick(0.02).await;
            }
        }
    });
    let (mut ws, generation) = common::controller(addr).await;
    let epoch = sup.get_observation().await.epoch_id;
    let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    {
        let mut inner = sup.inner().write().await;
        inner.commanded_positions = START.to_vec();
        inner.measured_positions = START.to_vec();
        inner.target_positions = START.to_vec();
    }
    let epoch = sup.get_observation().await.epoch_id;
    let mut body = json!({"label": "box", "speed_scale": 0.5, "debug": debug});
    if let Some(s) = settle_min_s {
        body["settle_min_s"] = json!(s);
    }
    let ack = common::rpc(&mut ws, generation, epoch, mm::PATH, body).await;
    let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
    let until = Instant::now() + Duration::from_secs(120);
    let mut last_hb = Instant::now();
    let res = loop {
        assert!(Instant::now() < until, "action {id} did not end");
        if last_hb.elapsed() > Duration::from_millis(200) {
            common::send(&mut ws, json!({"type":"heartbeat", "generation": generation, "epoch_id": epoch})).await;
            common::drain(&mut ws, 5).await;
            last_hb = Instant::now();
        }
        let obs = sup.get_observation().await;
        if let Some(a) = [obs.action, obs.previous_action].into_iter().flatten().find(|a| a.id == id && !a.is_running()) {
            break (serde_json::to_value(a).unwrap(), id);
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    };
    ticker_task.abort();
    srv.abort();
    res
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn gray_pgm_debug_bundle_off_on_replay_and_write_failure() {
    let dir = tempfile::tempdir().unwrap();
    // 1. debug: false writes nothing (fast settle_min_s: 0.5 case)
    unsafe { std::env::set_var("ARMD_MVIEW_DEBUG_DIR", dir.path()) };
    let (a_off, _) = run_one(false, Some(0.5), Duration::from_millis(2)).await;
    assert_eq!(a_off["state"], "done", "{a_off}");
    assert!(a_off["measurement"].get("debug").is_none());
    assert_eq!(std::fs::read_dir(dir.path()).unwrap().count(), 0);

    // 2. debug: true writes canonical P5 PGM v2 bundle and replays identically, using the
    //    DEFAULT 1.0 s settle and real 50 Hz (20 ms) feedback cadence so default settling stays covered.
    let (a_on, id) = run_one(true, None, Duration::from_millis(20)).await;
    assert_eq!(a_on["state"], "done", "{a_on}");
    let m = &a_on["measurement"];
    assert_eq!(m["frame_source"], "gray");
    assert_eq!(m["debug"]["written"], true, "{}", m["debug"]);
    for k in 1..3 {
        let settle_ms = m["views"][k]["timing"]["settle_ms"].as_f64().unwrap();
        assert!(settle_ms >= 1000.0, "view {k} must honour default 1.0 s settle at 50 Hz: {settle_ms} ms");
    }
    let bdir = dir.path().join(id.to_string());
    for k in 0..3 {
        let p = bdir.join(format!("view_{k}.pgm"));
        assert!(p.exists(), "missing {}", p.display());
        let bytes = std::fs::read(&p).unwrap();
        assert!(bytes.starts_with(b"P5\n640 360\n255\n"));
        assert_eq!(bytes.len(), "P5\n640 360\n255\n".len() + 640 * 360);
    }
    let (bundle, c) = mm::replay_bundle(&bdir).unwrap();
    assert_eq!(bundle.doc["version"], 2);
    assert_eq!(bundle.doc["frame_source"], "gray");
    assert_eq!(bundle.doc["views"][0]["encoding"], "pgm");
    assert_eq!(bundle.doc["views"][0]["instance_id"], "rv-bundle-1");
    assert_eq!(c.rec.stats.points as u64, m["stats"]["points"].as_u64().unwrap());
    let obj = c.measure.unwrap();
    assert!((obj.height_m - m["objects"][0]["height_m"].as_f64().unwrap()).abs() < 1e-4);
    assert!((obj.width_m - m["objects"][0]["width_m"].as_f64().unwrap()).abs() < 1e-4);
    assert!((obj.length_m - m["objects"][0]["length_m"].as_f64().unwrap()).abs() < 1e-4);
    assert_eq!(c.table.n_points as u64, m["table_check"]["n_points"].as_u64().unwrap());

    // 3. Unwritable ARMD_MVIEW_DEBUG_DIR (points to a regular file): run still succeeds, reports debug.written == false + error
    let blocker = dir.path().join("not_a_dir");
    std::fs::write(&blocker, b"x").unwrap();
    unsafe { std::env::set_var("ARMD_MVIEW_DEBUG_DIR", &blocker) };
    let (a_fail, _) = run_one(true, Some(0.5), Duration::from_millis(2)).await;
    assert_eq!(a_fail["state"], "done", "{a_fail}");
    assert_eq!(a_fail["measurement"]["debug"]["written"], false, "{}", a_fail["measurement"]["debug"]);
    assert!(a_fail["measurement"]["debug"]["error"].as_str().is_some_and(|s| !s.is_empty()));
}
