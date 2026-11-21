//! Action `measure_multiview` against the in-process armd (built-in B601-DM config, no driver)
//! with a fake vision provider that renders the scene (tests/render) from the camera pose of the
//! measured joints: JPEG snapshots letterboxed like rebot-vision's 640x640 `/snapshot.jpg`, and a
//! YOLO box of the object. Covers: success, target lost, a refused view pose (fallback offset)
//! and the end on lease loss.
mod common;
mod render;

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::grasp::{BoxFuture, CamModel, Detection, DetectionFrame, DetectionSource, GraspDeps};
use armd::kinematics::{self, Pose};
use armd::measure_multiview as mm;
use armd::model::KeepoutBox;
use armd::{ArmConfig, ArmSupervisor, SupervisorInner};
use render::{BoxObj, Noise, Scene};
use serde_json::{json, Value};

const START: [f32; 7] = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0, 0.0];
const HEIGHT: f64 = 0.040;

struct FakeVision {
    sup: Arc<ArmSupervisor>,
    cam: CamModel,
    scene: Scene,
    /// No detection away from the start joints.
    hide_after_start: AtomicBool,
    frame: AtomicU64,
    snapshots: AtomicU64,
    /// (joints, jpeg, box_norm) of the last render.
    cache: Mutex<Option<(Vec<f32>, Vec<u8>, Option<[f64; 4]>)>>,
}

impl FakeVision {
    /// Render (or reuse) the view of the measured joints.
    async fn view(&self) -> (Vec<f32>, Vec<u8>, Option<[f64; 4]>) {
        let (q, pose) = {
            let inner = self.sup.inner().read().await;
            let q = inner.measured_positions.clone();
            let pose = inner.config.arm_model.fk_f32(&q).unwrap();
            (q, pose)
        };
        if let Some(c) = self.cache.lock().unwrap().as_ref().filter(|c| c.0.iter().zip(&q).all(|(a, b)| (a - b).abs() < 1e-5)) {
            return c.clone();
        }
        let cam = mm::camera_at(&self.cam, &pose);
        let (w, h) = (self.cam.size[0] as usize, self.cam.size[1] as usize);
        let seed = self.frame.load(Ordering::SeqCst) + 1;
        let img = render::render(&self.scene, &cam, w, h, 2, Noise { sigma: 1.0, ..Noise::default() }, seed);
        // letterbox into a w x w square, frame in the centred rows
        let top = (w - h) / 2;
        let mut sq = vec![0u8; w * w];
        sq[top * w..(top + h) * w].copy_from_slice(&img.px);
        let mut jpeg = Vec::new();
        jpeg_encoder::Encoder::new(&mut jpeg, 95).encode(&sq, w as u16, w as u16, jpeg_encoder::ColorType::Luma).unwrap();
        let b = render::box_px(&self.scene, &self.scene.boxes[0], &cam);
        let inside = b[0] > 0.0 && b[1] > 0.0 && b[2] < w as f64 && b[3] < h as f64;
        let bn = inside.then(|| [b[0] / w as f64, b[1] / h as f64, b[2] / w as f64, b[3] / h as f64]);
        let v = (q, jpeg, bn);
        *self.cache.lock().unwrap() = Some(v.clone());
        v
    }

    async fn frame_now(&self) -> DetectionFrame {
        let (q, _, bn) = self.view().await;
        let away = START.iter().zip(&q).any(|(a, b)| (a - b).abs() > 1e-3);
        let hidden = self.hide_after_start.load(Ordering::SeqCst) && away;
        let detections = bn.filter(|_| !hidden).map(|b| vec![det("box", 0.9, b)]).unwrap_or_default();
        DetectionFrame {
            frame_id: self.frame.fetch_add(1, Ordering::SeqCst) + 1, captured_at: Instant::now(), policy_captured_at: None,
            image_size_px: self.cam.size, detections, depth: None, model_generation: None,
        }
    }
}

fn det(label: &str, score: f64, b: [f64; 4]) -> Detection {
    Detection {
        label: label.into(), score, raw_score_valid: true, box_norm: b, angle_deg: None, angle_conf: None, grasp_width_px: None,
        mask_focus: false, disp_fg: None, disp_bg: None, depth_conf: None, length_px: None, jaw_clear: None,
        depth_angle_deg: None, depth_angle_conf: None, depth_width_px: None,
    }
}

impl DetectionSource for FakeVision {
    fn per_request_pipelines(&self) -> BoxFuture<'_, Result<bool, String>> { Box::pin(async { Ok(true) }) }
    fn trigger_with_focus(&self, _p: &'static [&'static str], _w: u64, _f: Option<&armd::grasp::MaskFocus>) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        Box::pin(async move { Ok(Some(self.frame_now().await)) })
    }
    fn inference_enabled(&self) -> BoxFuture<'_, Result<bool, String>> { Box::pin(async { Ok(true) }) }
    fn set_inference(&self, _e: bool) -> BoxFuture<'_, Result<(), String>> { Box::pin(async { Ok(()) }) }
    fn next_frame(&self, _a: Option<u64>, _w: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        Box::pin(async move { Ok(Some(self.frame_now().await)) })
    }
    fn snapshot_jpeg(&self) -> BoxFuture<'_, Result<Vec<u8>, String>> {
        Box::pin(async move {
            self.snapshots.fetch_add(1, Ordering::SeqCst);
            Ok(self.view().await.1)
        })
    }
}

struct Rig {
    sup: Arc<ArmSupervisor>,
    vision: Arc<FakeVision>,
    ws: common::TestSocket,
    generation: u64,
    epoch: u64,
}

/// Camera model, start pose and where the image centre meets the table.
fn geometry(config: &ArmConfig) -> (CamModel, Pose, [f64; 3]) {
    let setup = SupervisorInner::new(config.clone()).grasp_setup().expect("grasp setup");
    let cam = CamModel::assumed(&config.arm_model, &setup, &config.camera_mount);
    let pose = config.arm_model.fk_f32(&START).unwrap();
    let look = cam.plane_point(&pose, [cam.cx, cam.cy], setup.table_z_m).expect("the start view sees the table");
    (cam, pose, look)
}

async fn rig(config: ArmConfig) -> Rig { rig_plain(config, false).await }

/// [`rig`] with a plain (uniform grey) box when `plain`.
async fn rig_plain(config: ArmConfig, plain: bool) -> Rig {
    let (cam, _, look) = geometry(&config);
    let sup = Arc::new(ArmSupervisor::new(config));
    let mut scene = Scene::new(0.0, vec![BoxObj { xy: [look[0], look[1]], yaw: 0.3, size: [0.05, 0.03, HEIGHT] }]);
    scene.plain_boxes = plain;
    let vision = Arc::new(FakeVision { sup: sup.clone(), cam, scene, hide_after_start: AtomicBool::new(false), frame: AtomicU64::new(0),
        snapshots: AtomicU64::new(0), cache: Mutex::new(None) });
    let deps = GraspDeps { source: vision.clone(), setup_path: None };
    let router = armd::api::create_router_with(sup.clone(), armd::api::proxy::DetectorUpstream::parse("http://127.0.0.1:9").unwrap(), deps, armd::OperatorAuth::disabled());
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });
    let ticker = sup.clone();
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(20));
        loop { interval.tick().await; ticker.tick(0.02).await; }
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
    Rig { sup, vision, ws, generation, epoch }
}

impl Rig {
    async fn start(&mut self, body: Value) -> Value { common::rpc(&mut self.ws, self.generation, self.epoch, mm::PATH, body).await }

    /// Heartbeats until action `id` ended; its status.
    async fn wait(&mut self, id: u64, max_s: f64) -> Value {
        let until = Instant::now() + Duration::from_secs_f64(max_s);
        loop {
            assert!(Instant::now() < until, "action {id} did not end");
            common::send(&mut self.ws, json!({"type":"heartbeat", "generation": self.generation, "epoch_id": self.epoch})).await;
            common::drain(&mut self.ws, 5).await;
            let obs = self.sup.get_observation().await;
            for a in [obs.action, obs.previous_action].into_iter().flatten() {
                if a.id == id && !a.is_running() { return serde_json::to_value(a).unwrap(); }
            }
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
    }

    async fn at_start(&self) -> bool {
        let inner = self.sup.inner().read().await;
        inner.commanded_positions.iter().zip(&START).take(6).all(|(a, b)| (a - b).abs() < 1e-3)
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn measures_the_box_height_from_three_views_and_returns_to_start() {
    let config = ArmConfig::rebot_dm();
    let (cam, pose, look) = geometry(&config);
    println!("camera {:?} -> table point {look:?}, distance {:.3} m, fx {}", cam.origin(&pose), kinematics::norm(&kinematics::sub(&look, &cam.origin(&pose))), cam.fx);
    let mut r = rig(config).await;
    let v: Value = serde_json::to_value(r.sup.inner().read().await.schema().capabilities).unwrap();
    assert_eq!(v["measure_multiview"], true, "{v}");
    let d = r.sup.inner().read().await.safety_describe();
    let l = &d["limits"]["measure_multiview"];
    assert_eq!((l["motions"].clone(), l["default_motion"].as_str()), (json!(["orbit", "translate"]), Some("orbit")), "{l}");
    assert_eq!((l["axes"]["x"].clone(), l["axes"]["xy"].clone(), l["min_frame_margin_px"].as_f64()), (json!([2, 5]), json!([3, 5]), Some(8.0)), "{l}");
    assert_eq!(l["speed_scale"], json!([0.1, 0.5]));
    let ack = r.start(json!({"label": "box", "speed_scale": 0.5})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let id = ack["action_id"].as_u64().unwrap();
    let a = r.wait(id, 120.0).await;
    let m = &a["measurement"];
    println!("{}", serde_json::to_string_pretty(&a).unwrap());
    assert_eq!((a["kind"].as_str(), a["state"].as_str()), (Some("measure_multiview"), Some("done")), "{a}");
    assert_eq!(m["method"], "motion_stereo");
    assert_eq!(m["views"].as_array().unwrap().len(), 3);
    assert!(m["why"].is_null());
    let o = &m["objects"][0];
    assert_eq!((o["label"].as_str(), o["views_used"].as_u64()), (Some("box"), Some(3)), "{o}");
    let h = o["height_m"].as_f64().unwrap();
    assert!((h - HEIGHT).abs() < 0.003, "height {h}");
    assert!((o["top_z_m"].as_f64().unwrap() - h - m["table_z_m"].as_f64().unwrap()).abs() < 1e-3);
    assert!(o["n_points"].as_u64().unwrap() >= 20 && m["table_check"]["n_points"].as_u64().unwrap() >= 10, "{m}");
    assert!((m["baseline_m"].as_f64().unwrap() - 0.08).abs() < 1e-6);
    assert!(r.at_start().await, "returned to the start joints");
    assert!(r.vision.snapshots.load(Ordering::SeqCst) >= 9, "2 discarded + 1 per view");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn target_lost_in_a_view_fails_and_returns_to_start() {
    let mut r = rig(ArmConfig::rebot_dm()).await;
    r.vision.hide_after_start.store(true, Ordering::SeqCst);
    let ack = r.start(json!({})).await;
    let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
    let a = r.wait(id, 90.0).await;
    assert_eq!((a["state"].as_str(), a["code"].as_str()), (Some("failed"), Some("TARGET_LOST")), "{a}");
    assert!(r.at_start().await, "returned to the start joints");
    // nothing at the start pose at all
    let mut r = rig(ArmConfig::rebot_dm()).await;
    let ack = r.start(json!({"label": "cup"})).await;
    let a = r.wait(ack["action_id"].as_u64().unwrap(), 30.0).await;
    assert_eq!((a["state"].as_str(), a["code"].as_str()), (Some("failed"), Some("TARGET_LOST")), "{a}");
    assert!(a["message"].as_str().unwrap().contains("cup"), "{a}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_refused_view_pose_falls_back_to_a_smaller_offset() {
    // a keep-out box that the gripper envelope enters at the +B/2 view but not at 0.75 of it
    let mut config = ArmConfig::rebot_dm();
    let (cam, pose, _) = geometry(&config);
    let c0 = mm::camera_at(&cam, &pose);
    let centre = [cam.cx, cam.cy];
    let pivot = cam.plane_point(&pose, centre, mm::LOOK_AT_ABOVE_TABLE_M).unwrap();
    let ray = kinematics::unit(&kinematics::sub(&pivot, &c0.c));
    let xc = kinematics::column(&c0.r, 0);
    let k = kinematics::dot(&ray, &xc);
    let lat = kinematics::unit(&kinematics::sub(&xc, &[ray[0] * k, ray[1] * k, ray[2] * k]));
    let ws = config.workspace.clone().unwrap();
    let side = lat[1].signum();
    let extreme = |o: f64| -> [f64; 3] {
        let p = mm::orbit_pose(&pose, &c0.c, &pivot, &lat, o);
        ws.check_points_end_m.iter().map(|c| p.transform_point(&c.map(f64::from))).max_by(|a, b| (a[1] * side).total_cmp(&(b[1] * side))).unwrap()
    };
    let (e4, e3) = (extreme(0.04), extreme(0.03));
    let edge = 0.5 * (e3[1] + e4[1]);
    let (y0, y1) = if side > 0.0 { (edge, e4[1] + 0.05) } else { (e4[1] - 0.05, edge) };
    let b = KeepoutBox { name: "fixture".into(), min_m: [(e4[0] - 0.04) as f32, y0 as f32, (e4[2] - 0.04) as f32], max_m: [(e4[0] + 0.04) as f32, y1 as f32, (e4[2] + 0.04) as f32] };
    println!("lat {lat:?}, extreme corner at +0.04 {e4:?}, at +0.03 {e3:?}, box {b:?}");
    config.workspace.as_mut().unwrap().keepout_boxes = vec![b];
    let mut r = rig(config).await;
    let ack = r.start(json!({"speed_scale": 0.5})).await;
    let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
    let a = r.wait(id, 150.0).await;
    let m = &a["measurement"];
    assert_eq!(a["state"], "done", "{a}");
    let f = m["fallbacks"].as_array().unwrap();
    println!("fallbacks {f:?}, views {}, objects {}", m["views"], m["objects"]);
    assert_eq!(f.len(), 1, "{m}");
    assert_eq!((f[0]["offset_m"][0].as_f64(), f[0]["offset_m"][1].as_f64(), f[0]["code"].as_str()), (Some(0.04), Some(0.0), Some("WORKSPACE_VIOLATION")), "{m}");
    let used = f[0]["used_offset_m"][0].as_f64().unwrap();
    assert!(used.abs() < 0.04, "{m}");
    let offs: Vec<f64> = m["views"].as_array().unwrap().iter().map(|v| v["offset_m"][0].as_f64().unwrap()).collect();
    assert!(offs.contains(&used) && offs.contains(&-0.04) && offs.contains(&0.0), "{offs:?}");
    assert!((m["objects"][0]["height_m"].as_f64().unwrap() - HEIGHT).abs() < 0.004, "{m}");
    assert!(r.at_start().await);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn lease_loss_during_a_view_motion_ends_the_action() {
    let mut r = rig(ArmConfig::rebot_dm()).await;
    let ack = r.start(json!({"speed_scale": 0.1})).await;
    let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
    // heartbeat until the first view motion runs
    let until = Instant::now() + Duration::from_secs(60);
    loop {
        assert!(Instant::now() < until, "no view motion");
        common::send(&mut r.ws, json!({"type":"heartbeat", "generation": r.generation, "epoch_id": r.epoch})).await;
        common::drain(&mut r.ws, 5).await;
        let moving = {
            let inner = r.sup.inner().read().await;
            inner.last_action.as_ref().and_then(|a| a.phase.clone()).is_some_and(|p| p.starts_with("move:")) && inner.active_trajectory.is_some()
        };
        if moving { break; }
        tokio::time::sleep(Duration::from_millis(30)).await;
    }
    let reply = common::ws_reply(&mut r.ws, json!({"type":"release_lease", "generation": r.generation}), &["lease_released"]).await;
    assert_eq!(reply["success"], true);
    let t = Instant::now();
    let a = loop {
        let obs = r.sup.get_observation().await;
        if let Some(a) = [obs.action, obs.previous_action].into_iter().flatten().find(|a| a.id == id && !a.is_running()) { break a; }
        assert!(t.elapsed() < Duration::from_secs(3), "the action did not end");
        tokio::time::sleep(Duration::from_millis(20)).await;
    };
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("cancelled", Some("LEASE_LOST")), "{a:?}");
    assert!(a.measurement.is_none());
    tokio::time::sleep(Duration::from_millis(600)).await;
    let snaps = r.vision.snapshots.load(Ordering::SeqCst);
    {
        let inner = r.sup.inner().read().await;
        assert!(inner.active_trajectory.is_none(), "the view motion stopped");
    }
    tokio::time::sleep(Duration::from_millis(1500)).await;
    assert_eq!(r.vision.snapshots.load(Ordering::SeqCst), snaps, "no capture after the end");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn refused_without_operator_session_and_bad_bodies() {
    let config = ArmConfig::rebot_dm();
    let sup = Arc::new(ArmSupervisor::new(config));
    let router = armd::api::create_router(sup.clone(), armd::OperatorAuth::enforced("123456", 1800));
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });
    let (mut ws, generation) = common::controller(addr).await;
    let epoch = sup.get_observation().await.epoch_id;
    let rej = common::rpc(&mut ws, generation, epoch, mm::PATH, json!({})).await;
    assert_eq!((rej["type"].as_str(), rej["code"].as_str()), (Some("command_reject"), Some("OPERATOR_REQUIRED")), "{rej}");
    let resp = reqwest::Client::new().post(format!("http://{addr}{}", mm::PATH)).json(&json!({})).send().await.unwrap();
    assert_eq!(resp.status().as_u16(), 403);
    // bounds (no auth)
    let mut r = rig(ArmConfig::rebot_dm()).await;
    for bad in [json!({"speed_scale": 0.8}), json!({"views": 7}), json!({"baseline_m": 0.2})] {
        let rej = r.start(bad.clone()).await;
        assert_eq!(rej["code"], "INVALID_COMMAND", "{bad}: {rej}");
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn translate_on_two_axes_measures_a_plain_box_from_its_outline() {
    let mut r = rig_plain(ArmConfig::rebot_dm(), true).await;
    let ack = r.start(json!({"label": "box", "motion": "translate", "speed_scale": 0.5})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let a = r.wait(ack["action_id"].as_u64().unwrap(), 150.0).await;
    let m = &a["measurement"];
    println!("translate xy5: objects {}, stats {}, views {}", m["objects"], m["stats"], m["views"]);
    assert_eq!(a["state"], "done", "{a}");
    assert_eq!((m["motion"].as_str(), m["axes"].as_str()), (Some("translate"), Some("xy")));
    let views = m["views"].as_array().unwrap();
    assert_eq!(views.len(), 5);
    // the start, then +x, +y, -x, -y at B/2; orientation kept, the look-at point moves
    for v in &views[1..] {
        let o = [v["offset_m"][0].as_f64().unwrap(), v["offset_m"][1].as_f64().unwrap()];
        assert!((o[0].hypot(o[1]) - 0.04).abs() < 1e-9, "{v}");
        let s = [v["predicted_shift_px"][0].as_f64().unwrap(), v["predicted_shift_px"][1].as_f64().unwrap()];
        assert!(s[0].hypot(s[1]) > 40.0 && v["frame_margin_px"].as_f64().unwrap() >= 8.0, "{v}");
        // orientation kept (within the IK tolerance of 0.5 deg)
        let q: Vec<f64> = (0..4).map(|i| v["tcp"]["quat_wxyz"][i].as_f64().unwrap()).collect();
        let q0: Vec<f64> = (0..4).map(|i| views[0]["tcp"]["quat_wxyz"][i].as_f64().unwrap()).collect();
        let dot: f64 = q.iter().zip(&q0).map(|(a, b)| a * b).sum::<f64>().abs();
        assert!(2.0 * dot.min(1.0).acos() < 0.5f64.to_radians(), "{v}");
    }
    let o = &m["objects"][0];
    assert!((o["height_m"].as_f64().unwrap() - HEIGHT).abs() < 0.003, "{o}");
    assert!(o["n_edge_points"].as_u64().unwrap() > 0, "{o}");
    for k in ["features_ms", "sweep_ms", "refine_ms", "triangulate_ms", "decode_ms", "total_ms"] { assert!(m["stats"][k].is_number(), "{k}"); }
    assert!(r.at_start().await);
}
