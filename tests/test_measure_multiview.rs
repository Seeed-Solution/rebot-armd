//! Action `measure_multiview` against the in-process armd (built-in B601-DM config, no driver)
//! with a fake vision provider that renders the scene (tests/render) from the camera pose of the
//! measured joints: JPEG snapshots letterboxed like rebot-vision's 640x640 `/snapshot.jpg`, and a
//! YOLO box of the object. Like rebot-vision with inference idle, `/snapshot.jpg` returns the
//! frame of the last trigger (a new render at the joints of that moment), not a fresh one.
//! Covers: success, target lost, a refused view pose (fallback offset), the end on lease loss, a
//! stale snapshot from another pose, a snapshot that never refreshes, and the camera rotation
//! refinement (`pose_refine`): a rotation error per view, ArUco marker corners from the provider.
mod common;
mod render;

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::grasp::{BoxFuture, CameraHoldTicket, CamModel, Detection, DetectionFrame, DetectionSource, GraspDeps, GrayFrame, GrayFrameError, GrayTriggerResult};
use armd::kinematics::{self, Pose};
use armd::measure_multiview as mm;
use armd::model::KeepoutBox;
use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointMode, JointTarget};
use armd::model::JointState;
use armd::{ArmConfig, ArmSupervisor, SupervisorInner};
use render::{BoxObj, Noise, Scene};
use serde_json::{json, Value};

const START: [f32; 7] = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0, 0.0];
const HEIGHT: f64 = 0.040;

/// A rendered camera frame: its id (the trigger's `frame_id`), the joints it was rendered at,
/// the un-letterboxed gray image, the JPEG, the object's box and the markers in view.
#[derive(Clone)]
struct Shot { id: u64, joints: Vec<f32>, img: armd::mview_stereo::Gray, jpeg: Vec<u8>, box_norm: Option<[f64; 4]>, markers: Vec<armd::mview_stereo::Marker>, contour: Vec<[f64; 2]> }

struct FakeVision {
    sup: Arc<ArmSupervisor>,
    cam: CamModel,
    scene: Arc<Scene>,
    /// No detection away from the start joints.
    hide_after_start: AtomicBool,
    frame: AtomicU64,
    snapshots: AtomicU64,
    /// Each snapshot takes this long (ms).
    snapshot_delay_ms: AtomicU64,
    /// Each trigger takes this long (ms; real rebot-vision 1.0-1.5 s).
    trigger_delay_ms: AtomicU64,
    /// Triggers leave `/snapshot.jpg` alone (a camera whose snapshot never refreshes).
    freeze_snapshot: AtomicBool,
    /// Set on the first snapshot (the arm settled): the creep driver starts shaking.
    shake_on_snapshot: Mutex<Option<Arc<Mutex<Creep>>>>,
    /// Set on the first gray trigger (since gray bypasses `/snapshot.jpg`).
    shake_on_gray_trigger: Mutex<Option<Arc<Mutex<Creep>>>>,
    /// The creep driver's clock (stopped while rendering).
    clock: Mutex<Option<Arc<Mutex<Creep>>>>,
    /// What `/snapshot.jpg` returns: the frame of the last trigger (rendered on the first
    /// snapshot when there was none).
    snapshot: Mutex<Option<Shot>>,
    /// (frame id, joints) of every snapshot returned.
    served: Mutex<Vec<(u64, Vec<f32>)>>,
    /// Away from the start joints the camera that renders is turned by this much (rad) about its
    /// own y axis, + or - with the sign of J1's change: a pose error the kinematics do not know.
    rot_err: Mutex<f64>,
    /// ArUco markers on the table (id, corners in the base frame), and whether the provider runs
    /// the aruco pipeline.
    markers: Mutex<Vec<(i64, [[f64; 3]; 4])>>,
    aruco: AtomicBool,
    /// Raw gray frames (default false so existing tests keep exercising the JPEG fallback).
    gray: AtomicBool,
    instance_id: Mutex<String>,
    gray_ring: Mutex<VecDeque<Shot>>,
    gray_fetches: AtomicU64,
    gray_fetch_delay_ms: AtomicU64,
    gray_fail_queue: Mutex<VecDeque<GrayFrameError>>,
    gray_fail_after_start: Mutex<Option<GrayFrameError>>,
    on_gray_fetch: Mutex<Option<Box<dyn Fn() + Send + Sync>>>,
    gray_trigger_timeouts: AtomicU64,
    on_gray_trigger_timeout: Mutex<Option<Box<dyn Fn() + Send + Sync>>>,
    /// Camera hold (rebot-vision subscription without pipelines): supported (default true), the
    /// hold requests (first and refreshes) and the releases, and whether a hold is active.
    hold_supported: AtomicBool,
    holds: AtomicU64,
    releases: Mutex<Vec<String>>,
    held: AtomicBool,
    /// From this many hold requests on (0: never) the hold answers as a restarted provider:
    /// another id ("s1") of another instance ("rv-restarted").
    hold_restart_after: AtomicU64,
    /// (client, instance id) of every release, and the client of every hold request.
    release_args: Mutex<Vec<(String, Option<String>)>>,
    hold_clients: Mutex<Vec<String>>,
    /// Called at the start of every trigger (JPEG and gray).
    on_trigger: Mutex<Option<Box<dyn Fn() + Send + Sync>>>,
    /// Detections carry the box's image outline as a segmentation contour.
    contours: AtomicBool,
}

/// `r` turned by `angle` about its own axis `axis` (0 x, 1 y, 2 z).
fn turned(r: &kinematics::Mat3, axis: usize, angle: f64) -> kinematics::Mat3 {
    let (c, s) = (angle.cos(), angle.sin());
    let (i, j) = ((axis + 1) % 3, (axis + 2) % 3);
    let mut e = [[0.0; 3]; 3];
    e[axis][axis] = 1.0;
    (e[i][i], e[i][j], e[j][i], e[j][j]) = (c, -s, s, c);
    let mut out = [[0.0; 3]; 3];
    for a in 0..3 { for b in 0..3 { out[a][b] = (0..3).map(|k| r[a][k] * e[k][b]).sum(); } }
    out
}

impl FakeVision {
    /// Render frame `id` (its noise seed) at joints `q`, the measured joints when None.
    async fn render(&self, id: u64, q: Option<Vec<f32>>) -> Shot {
        let (q, pose) = if self.sup.owner_mode() == armd::supervisor::OWNER_EXECUTOR {
            let obs_q = self.sup.get_observation().await.positions;
            let q = q.unwrap_or(obs_q);
            let env = self.sup.hub().prep_env().expect("prep_env published");
            let pose = env.config.arm_model.fk_f32(&q).unwrap();
            (q, pose)
        } else {
            let inner = self.sup.inner().read().await;
            let q = q.unwrap_or_else(|| inner.measured_positions.clone());
            let pose = inner.config.arm_model.fk_f32(&q).unwrap();
            (q, pose)
        };
        // rendering takes the camera no time: the creep clock stops meanwhile
        let paused = Instant::now();
        let mut cam = mm::camera_at(&self.cam, &pose);
        let err = *self.rot_err.lock().unwrap();
        let dj1 = f64::from(q[0] - START[0]);
        if err != 0.0 && START.iter().zip(&q).any(|(a, b)| (a - b).abs() > 1e-3) { cam.r = turned(&cam.r, 1, if dj1 < 0.0 { -err } else { err }); }
        let (w, h) = (self.cam.size[0] as usize, self.cam.size[1] as usize);
        let scene = Arc::clone(&self.scene);
        let img = tokio::task::spawn_blocking(move || render::render(&scene, &cam, w, h, 2, Noise { sigma: 1.0, ..Noise::default() }, id)).await.unwrap();
        // letterbox into a w x w square, frame in the centred rows
        let top = (w - h) / 2;
        let mut sq = vec![0u8; w * w];
        sq[top * w..(top + h) * w].copy_from_slice(&img.px);
        let mut jpeg = Vec::new();
        jpeg_encoder::Encoder::new(&mut jpeg, 95).encode(&sq, w as u16, w as u16, jpeg_encoder::ColorType::Luma).unwrap();
        let b = render::box_px(&self.scene, &self.scene.boxes[0], &cam);
        let inside = b[0] > 0.0 && b[1] > 0.0 && b[2] < w as f64 && b[3] < h as f64;
        let box_norm = inside.then(|| [b[0] / w as f64, b[1] / h as f64, b[2] / w as f64, b[3] / h as f64]);
        let contour: Vec<[f64; 2]> = render::box_hull_px(&self.scene, &self.scene.boxes[0], &cam).iter().map(|q| [q[0] / w as f64, q[1] / h as f64]).collect();
        let markers = self.markers.lock().unwrap().iter().filter_map(|(id, c)| {
            let mut corners = [[0.0; 2]; 4];
            for (k, x) in c.iter().enumerate() {
                let p = cam.project(x)?;
                if p[0] < 2.0 || p[1] < 2.0 || p[0] > w as f64 - 2.0 || p[1] > h as f64 - 2.0 { return None; }
                corners[k] = p;
            }
            Some(armd::mview_stereo::Marker { id: *id, corners })
        }).collect();
        if let Some(c) = self.shake_on_snapshot.lock().unwrap().as_ref()
            .or(self.shake_on_gray_trigger.lock().unwrap().as_ref())
            .or(self.clock.lock().unwrap().as_ref())
        {
            c.lock().unwrap().paused += paused.elapsed();
        }
        Shot { id, joints: q, img, jpeg, box_norm, markers, contour }
    }

    /// A trigger: a new frame at the current joints, which also becomes the snapshot.
    async fn frame_now(&self) -> DetectionFrame { self.frame_with_markers().await.0 }

    /// [`Self::frame_now`] with the markers of the frame.
    async fn frame_with_markers(&self) -> (DetectionFrame, Vec<armd::mview_stereo::Marker>) {
        self.frame_with_pipelines(&["yolo"]).await.0
    }

    async fn frame_with_pipelines(&self, pipelines: &[&str]) -> ((DetectionFrame, Vec<armd::mview_stereo::Marker>), bool) {
        if let Some(cb) = self.on_trigger.lock().unwrap().as_ref() { cb(); }
        if pipelines.contains(&"gray") {
            if let Some(c) = self.shake_on_gray_trigger.lock().unwrap().as_ref() {
                c.lock().unwrap().shake = true;
            }
        }
        let d = self.trigger_delay_ms.load(Ordering::SeqCst);
        if d > 0 { tokio::time::sleep(Duration::from_millis(d)).await; }
        // frame ids (and noise seeds) 1, 2, ... per trigger
        let shot = self.render(self.frame.fetch_add(1, Ordering::SeqCst) + 1, None).await;
        let away = START.iter().zip(&shot.joints).any(|(a, b)| (a - b).abs() > 1e-3);
        let hidden = self.hide_after_start.load(Ordering::SeqCst) && away;
        let with_contour = self.contours.load(Ordering::SeqCst);
        let detections = shot.box_norm.filter(|_| !hidden).map(|b| {
            let mut d = det("box", 0.9, b);
            if with_contour { d.contour_norm = Some(Ok(shot.contour.clone())); }
            vec![d]
        }).unwrap_or_default();
        let (frame_id, markers) = (shot.id, shot.markers.clone());
        let ran_gray = pipelines.contains(&"gray") && self.gray.load(Ordering::SeqCst);
        if ran_gray {
            let mut ring = self.gray_ring.lock().unwrap();
            if ring.len() >= 4 { ring.pop_front(); }
            ring.push_back(shot.clone());
        }
        if !self.freeze_snapshot.load(Ordering::SeqCst) { *self.snapshot.lock().unwrap() = Some(shot); }
        ((DetectionFrame {
            frame_id, captured_at: Instant::now(), policy_captured_at: None,
            image_size_px: self.cam.size, detections, depth: None, model_generation: None,
        }, markers), ran_gray)
    }
}

fn det(label: &str, score: f64, b: [f64; 4]) -> Detection {
    Detection {
        label: label.into(), score, raw_score_valid: true, box_norm: b, angle_deg: None, angle_conf: None, grasp_width_px: None,
        mask_focus: false, disp_fg: None, disp_bg: None, depth_conf: None, length_px: None, jaw_clear: None,
        depth_angle_deg: None, depth_angle_conf: None, depth_width_px: None, contour_norm: None,
    }
}

impl DetectionSource for FakeVision {
    fn per_request_pipelines(&self) -> BoxFuture<'_, Result<bool, String>> { Box::pin(async { Ok(true) }) }
    fn trigger_with_focus(&self, _p: &'static [&'static str], _w: u64, _f: Option<&armd::grasp::MaskFocus>) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        Box::pin(async move { Ok(Some(self.frame_now().await)) })
    }
    fn aruco_available(&self) -> BoxFuture<'_, Result<bool, String>> { Box::pin(async move { Ok(self.aruco.load(Ordering::SeqCst)) }) }
    fn gray_available(&self) -> BoxFuture<'_, Result<bool, String>> { Box::pin(async move { Ok(self.gray.load(Ordering::SeqCst)) }) }
    fn trigger_markers(&self, p: &'static [&'static str], _w: u64) -> BoxFuture<'_, Result<Option<(DetectionFrame, Vec<armd::mview_stereo::Marker>)>, String>> {
        Box::pin(async move {
            let ((f, m), _) = self.frame_with_pipelines(p).await;
            Ok(Some((f, if p.contains(&"aruco") { m } else { Vec::new() })))
        })
    }
    fn trigger_gray(&self, p: &'static [&'static str], _w: u64) -> BoxFuture<'_, Result<Option<GrayTriggerResult>, String>> {
        Box::pin(async move {
            if self.gray_trigger_timeouts.fetch_update(Ordering::SeqCst, Ordering::SeqCst, |n| n.checked_sub(1)).is_ok() {
                if let Some(cb) = self.on_gray_trigger_timeout.lock().unwrap().as_ref() {
                    cb();
                }
                tokio::time::sleep(Duration::from_millis(60)).await;
                return Ok(None);
            }
            let ((frame, m), gray_ran) = self.frame_with_pipelines(p).await;
            let markers = if p.contains(&"aruco") { m } else { Vec::new() };
            let instance_id = Some(self.instance_id.lock().unwrap().clone());
            Ok(Some(GrayTriggerResult { frame, markers, instance_id, gray_ran, gray_ms: gray_ran.then_some(4.5) }))
        })
    }
    fn gray_frame(&self, instance_id: &str, frame_id: u64) -> BoxFuture<'_, Result<GrayFrame, GrayFrameError>> {
        let req_inst = instance_id.to_string();
        Box::pin(async move {
            self.gray_fetches.fetch_add(1, Ordering::SeqCst);
            let t_fetch = Instant::now();
            let d = self.gray_fetch_delay_ms.load(Ordering::SeqCst);
            if d > 0 { tokio::time::sleep(Duration::from_millis(d)).await; }
            let fetch_ms = t_fetch.elapsed().as_secs_f64() * 1000.0;
            if let Some(err) = self.gray_fail_queue.lock().unwrap().pop_front() {
                return Err(err);
            }
            let away = if self.sup.owner_mode() == armd::supervisor::OWNER_EXECUTOR {
                let pos = self.sup.get_observation().await.positions;
                START.iter().zip(&pos).any(|(a, b)| (a - b).abs() > 1e-3)
            } else {
                let inner = self.sup.inner().read().await;
                START.iter().zip(&inner.measured_positions).any(|(a, b)| (a - b).abs() > 1e-3)
            };
            if away {
                if let Some(err) = self.gray_fail_after_start.lock().unwrap().clone() {
                    return Err(err);
                }
            }
            let cur_inst = self.instance_id.lock().unwrap().clone();
            if req_inst != cur_inst {
                return Err(GrayFrameError::Expired(format!("gray frame {frame_id}: 410 FRAME_EXPIRED")));
            }
            let shot = {
                let ring = self.gray_ring.lock().unwrap();
                ring.iter().rev().find(|s| s.id == frame_id).cloned()
            };
            let Some(s) = shot else {
                return Err(GrayFrameError::NotStored(format!("gray frame {frame_id}: 404 FRAME_NOT_STORED")));
            };
            if let Some(cb) = self.on_gray_fetch.lock().unwrap().as_ref() {
                cb();
            }
            Ok(GrayFrame {
                instance_id: cur_inst,
                frame_id: s.id,
                img: s.img,
                frame_size: [640, 640],
                crop: [0, 140, 640, 360],
                ts_ms: 1763791234000 + s.id,
                frame_mono_ms: 10000 + s.id,
                wait_mono_ms: 9990 + s.id,
                grab_mono_ms: 10002 + s.id,
                stored_mono_ms: 10007 + s.id,
                convert_ms: 4.5,
                fetch_ms,
                parse_ms: 0.2,
            })
        })
    }
    fn inference_enabled(&self) -> BoxFuture<'_, Result<bool, String>> { Box::pin(async { Ok(true) }) }
    fn set_inference(&self, _e: bool) -> BoxFuture<'_, Result<(), String>> { Box::pin(async { Ok(()) }) }
    fn next_frame(&self, _a: Option<u64>, _w: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        Box::pin(async move { Ok(Some(self.frame_now().await)) })
    }
    fn camera_hold(&self, client: &str, ttl_ms: u64) -> BoxFuture<'_, Result<Option<CameraHoldTicket>, String>> {
        assert!(client.starts_with(mm::CAMERA_HOLD_CLIENT_PREFIX) && ttl_ms == mm::CAMERA_HOLD_TTL_MS, "{client} {ttl_ms}");
        self.hold_clients.lock().unwrap().push(client.to_string());
        Box::pin(async move {
            if !self.hold_supported.load(Ordering::SeqCst) { return Ok(None); }
            let n = self.holds.fetch_add(1, Ordering::SeqCst) + 1;
            self.held.store(true, Ordering::SeqCst);
            let restart = self.hold_restart_after.load(Ordering::SeqCst);
            Ok(Some(if restart > 0 && n >= restart {
                CameraHoldTicket { id: "s1".into(), instance_id: Some("rv-restarted".into()) }
            } else {
                CameraHoldTicket { id: "s7".into(), instance_id: Some("rv-fake-1".into()) }
            }))
        })
    }
    fn camera_release(&self, ticket: &CameraHoldTicket, client: &str) -> BoxFuture<'_, Result<(), String>> {
        let (id, inst, client) = (ticket.id.clone(), ticket.instance_id.clone(), client.to_string());
        Box::pin(async move {
            self.held.store(false, Ordering::SeqCst);
            self.releases.lock().unwrap().push(id);
            self.release_args.lock().unwrap().push((client, inst));
            Ok(())
        })
    }
    fn snapshot_jpeg(&self) -> BoxFuture<'_, Result<Vec<u8>, String>> {
        Box::pin(async move {
            self.snapshots.fetch_add(1, Ordering::SeqCst);
            if let Some(c) = self.shake_on_snapshot.lock().unwrap().as_ref() { c.lock().unwrap().shake = true; }
            let cached = self.snapshot.lock().unwrap().clone();
            let shot = match cached {
                Some(s) => s,
                None => { let s = self.render(0, None).await; *self.snapshot.lock().unwrap() = Some(s.clone()); s }
            };
            self.served.lock().unwrap().push((shot.id, shot.joints.clone()));
            let d = self.snapshot_delay_ms.load(Ordering::SeqCst);
            if d > 0 { tokio::time::sleep(Duration::from_millis(d)).await; }
            Ok(shot.jpeg)
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

async fn rig(config: ArmConfig) -> Rig { rig_driven(config, false, None).await }

async fn rig_plain(config: ArmConfig, plain: bool) -> Rig { rig_driven(config, plain, None).await }

/// [`rig`] with a plain (uniform grey) box when `plain`, and the creep driver when given.
async fn rig_driven(config: ArmConfig, plain: bool, driver: Option<CreepArm>) -> Rig { rig_scene(config, plain, false, driver).await }

/// [`rig_driven`] with a uniform table when `plain_table`.
async fn rig_scene(config: ArmConfig, plain: bool, plain_table: bool, driver: Option<CreepArm>) -> Rig {
    let (cam, _, look) = geometry(&config);
    let sup = Arc::new(ArmSupervisor::new(config));
    if let Some(d) = driver {
        d.inner.0.lock().unwrap().q = START;
        sup.inner().write().await.driver = Some(std::sync::Mutex::new(Box::new(d)));
    }
    let mut scene = Scene::new(0.0, vec![BoxObj { xy: [look[0], look[1]], yaw: 0.3, size: [0.05, 0.03, HEIGHT] }]);
    scene.plain_boxes = plain;
    scene.plain_table = plain_table;
    let vision = Arc::new(FakeVision { sup: sup.clone(), cam, scene: Arc::new(scene), hide_after_start: AtomicBool::new(false), frame: AtomicU64::new(0),
        snapshots: AtomicU64::new(0), snapshot_delay_ms: AtomicU64::new(0), trigger_delay_ms: AtomicU64::new(0), freeze_snapshot: AtomicBool::new(false),
        shake_on_snapshot: Mutex::new(None), shake_on_gray_trigger: Mutex::new(None), clock: Mutex::new(None), snapshot: Mutex::new(None), served: Mutex::new(Vec::new()),
        rot_err: Mutex::new(0.0), markers: Mutex::new(Vec::new()), aruco: AtomicBool::new(false),
        gray: AtomicBool::new(false), instance_id: Mutex::new("rv-fake-1".into()), gray_ring: Mutex::new(VecDeque::new()),
        gray_fetches: AtomicU64::new(0), gray_fetch_delay_ms: AtomicU64::new(0), gray_fail_queue: Mutex::new(VecDeque::new()),
        gray_fail_after_start: Mutex::new(None), on_gray_fetch: Mutex::new(None),
        gray_trigger_timeouts: AtomicU64::new(0), on_gray_trigger_timeout: Mutex::new(None),
        hold_supported: AtomicBool::new(true), holds: AtomicU64::new(0), releases: Mutex::new(Vec::new()), held: AtomicBool::new(false),
        hold_restart_after: AtomicU64::new(0), release_args: Mutex::new(Vec::new()), hold_clients: Mutex::new(Vec::new()), on_trigger: Mutex::new(None),
        contours: AtomicBool::new(false) });
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
    // no model contour: a colour mask of the grey render, else the stereo points
    assert!(["classical_mask", "stereo_points"].contains(&o["footprint_source"].as_str().unwrap()), "{o}");
    assert!(o["length_confidence"].is_string() && o["width_confidence"].is_string() && o["footprint_views"].is_u64(), "{o}");
    assert!((m["baseline_m"].as_f64().unwrap() - 0.08).abs() < 1e-6);
    assert!(r.at_start().await, "returned to the start joints");
    // per view: the snapshot before the trigger, then at least one after it
    assert!(r.vision.snapshots.load(Ordering::SeqCst) >= 6, "2 per view");
    for v in m["views"].as_array().unwrap() {
        assert!(v["trigger_ms"].is_number() && v["snapshot_wait_ms"].is_number() && v["snapshot_polls"].as_u64() == Some(0), "{v}");
    }
    check_timing(m, 0.0, 0.0);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn segmentation_contours_give_the_footprint_and_keep_the_height() {
    // the 50 x 30 mm box, yaw 0.3 rad, 40 mm high; the detections carry its image outline
    let mut r = rig(ArmConfig::rebot_dm()).await;
    r.vision.contours.store(true, Ordering::SeqCst);
    let ack = r.start(json!({"label": "box", "speed_scale": 0.5, "views": 5, "motion": "translate", "axes": "xy"})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let a = r.wait(ack["action_id"].as_u64().unwrap(), 180.0).await;
    let m = &a["measurement"];
    assert_eq!(a["state"].as_str(), Some("done"), "{a}");
    let o = &m["objects"][0];
    println!("{}\n{}", o, m["footprint"]);
    assert_eq!((o["footprint_source"].as_str(), o["footprint_views"].as_u64(), o["views_used"].as_u64()), (Some("seg_mask"), Some(5), Some(5)), "{o}");
    let look = geometry(&ArmConfig::rebot_dm()).2;
    // the top rim projected to mid height: up to h/2 tan(ray angle) outside the box per side
    let sens = &m["footprint"]["result"]["height_sensitivity_mm"];
    let (sl, sw) = (sens[0].as_f64().unwrap() / 1000.0, sens[1].as_f64().unwrap() / 1000.0);
    let (l, w) = (o["length_m"].as_f64().unwrap(), o["width_m"].as_f64().unwrap());
    assert!(l > 0.048 && l - 0.05 < 0.003 + sl && w > 0.028 && w - 0.03 < 0.003 + sw, "{o} {sens}");
    let c = &o["centroid_m"];
    assert!((c[0].as_f64().unwrap() - look[0]).abs() < 0.003 && (c[1].as_f64().unwrap() - look[1]).abs() < 0.003, "{o} {look:?}");
    let dy = (o["yaw_deg"].as_f64().unwrap() - 0.3f64.to_degrees() + 90.0).rem_euclid(180.0) - 90.0;
    assert!(dy.abs() < 5.0, "{o}");
    let h = o["height_m"].as_f64().unwrap();
    assert!((h - HEIGHT).abs() < 0.003, "height {h}");
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
            inner.last_action.as_ref().and_then(|a| a.phase).is_some_and(|p| p.starts_with("move:")) && inner.active_trajectory.is_some()
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
    for bad in [json!({"speed_scale": 0.8}), json!({"views": 7}), json!({"baseline_m": 0.2}), json!({"settle_min_s": 0.2}), json!({"settle_min_s": 3.5})] {
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

/// Joint offsets the creep driver adds to an ideal position servo (rad, seconds since the start).
#[derive(Default)]
struct Creep {
    t0: Option<Instant>,
    /// J1 creep (rad/s) and the J4 limit cycle (amplitude rad, period s), always.
    creep: f32,
    cycle: (f32, f32),
    /// After `shake` is set: J4 drifts at `shake_rate` (rad/s).
    shake: bool,
    shake_rate: f32,
    shake_t0: Option<f32>,
    /// Time the clock stood still.
    paused: Duration,
    /// J4 offset by `.2` (rad) from `.0` for `.1`, then back (a move-and-return).
    bump: Option<(Instant, Duration, f32)>,
}

/// common::TestArm reporting positions with creep and limit cycles (real B601-DM, 2026-11-22).
struct CreepArm { inner: common::TestArm, creep: Arc<Mutex<Creep>> }

impl CreepArm {
    fn offsets(&self) -> ([f32; 7], [f32; 7]) {
        let mut c = self.creep.lock().unwrap();
        let t = c.t0.get_or_insert_with(Instant::now).elapsed().saturating_sub(c.paused).as_secs_f32();
        let (mut p, mut v) = ([0.0f32; 7], [0.0f32; 7]);
        let tau = std::f32::consts::TAU;
        p[0] = (c.creep * t).min(0.004);
        v[0] = if c.creep * t < 0.004 { c.creep } else { 0.0 };
        let (a, per) = c.cycle;
        if a > 0.0 {
            p[3] = a * (tau * t / per).sin();
            v[3] = a * tau / per * (tau * t / per).cos();
        }
        if let Some((t0, d, amp)) = c.bump {
            if t0.elapsed() < d { p[3] += amp; }
        }
        if c.shake {
            let ts = *c.shake_t0.get_or_insert(t);
            p[3] += c.shake_rate * (t - ts);
            v[3] += c.shake_rate;
        }
        (p, v)
    }
}

impl ArmDriver for CreepArm {
    fn capabilities(&self) -> DriverCapabilities { self.inner.capabilities() }
    fn enable_state(&self) -> EnableState { self.inner.enable_state() }
    fn is_feedback_valid(&self) -> bool { true }
    fn is_enabled(&self) -> bool { self.inner.is_enabled() }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { self.inner.feedback_ages() }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let (p, v) = self.offsets();
        let mut s = self.inner.read_joint_states()?;
        for (i, js) in s.iter_mut().enumerate().take(7) { js.position += p[i]; js.velocity += v[i]; }
        Ok(s)
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> { self.inner.write_joint_targets(targets) }
    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> { self.inner.set_torque(enabled) }
    fn supports_mit_mode_switch(&self) -> bool { false }
    fn set_joint_modes(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> { self.inner.set_joint_modes(modes, hold) }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn creep_and_a_slow_limit_cycle_do_not_stop_the_captures() {
    // J1 creeps 1.4 mrad/s (up to 4 mrad), J4 cycles +-2 mrad every 1.5 s; each snapshot takes 60 ms
    let creep = Arc::new(Mutex::new(Creep { creep: 0.0014, cycle: (0.002, 1.5), ..Default::default() }));
    let d = CreepArm { inner: common::TestArm::new(), creep: creep.clone() };
    let mut r = rig_driven(ArmConfig::rebot_dm(), false, Some(d)).await;
    r.vision.snapshot_delay_ms.store(60, Ordering::SeqCst);
    *r.vision.clock.lock().unwrap() = Some(creep.clone());
    let ack = r.start(json!({"label": "box", "speed_scale": 0.5})).await;
    let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
    let a = r.wait(id, 150.0).await;
    let m = &a["measurement"];
    let tries: Vec<Value> = m["views"].as_array().map(|v| v.iter().map(|x| x["snapshot_tries_mrad"].clone()).collect()).unwrap_or_default();
    println!("state {} code {} message {}; snapshot tries (mrad) {tries:?}; settle_s {:?}", a["state"], a["code"], a["message"],
        m["views"].as_array().map(|v| v.iter().map(|x| x["settle_s"].clone()).collect::<Vec<_>>()));
    assert_eq!(a["state"], "done", "{a}");
    assert!((m["objects"][0]["height_m"].as_f64().unwrap() - HEIGHT).abs() < 0.004, "{m}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn shaking_during_the_snapshots_retries_then_fails_moved() {
    // still while settling; from the first snapshot on J4 drifts 30 mrad/s, each trigger takes
    // 100 ms and each snapshot 60 ms: every bracket (trigger + snapshot) moves about 4.8 mrad
    let creep = Arc::new(Mutex::new(Creep { shake_rate: 0.03, ..Default::default() }));
    let d = CreepArm { inner: common::TestArm::new(), creep: creep.clone() };
    let mut r = rig_driven(ArmConfig::rebot_dm(), false, Some(d)).await;
    r.vision.snapshot_delay_ms.store(60, Ordering::SeqCst);
    r.vision.trigger_delay_ms.store(100, Ordering::SeqCst);
    *r.vision.shake_on_snapshot.lock().unwrap() = Some(creep.clone());
    let ack = r.start(json!({"pose_refine": "off"})).await;
    let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
    let a = r.wait(id, 60.0).await;
    println!("{} {}", a["code"], a["message"]);
    assert_eq!((a["state"].as_str(), a["code"].as_str()), (Some("failed"), Some("MOVED")), "{a}");
    let msg = a["message"].as_str().unwrap();
    assert!(msg.contains("3 snapshots") && msg.contains("J4") && msg.contains("limit 2.0"), "{msg}");
    // at the start view: the snapshot before the first trigger, then one per try (each differs
    // from the one before it at once)
    assert_eq!(r.vision.snapshots.load(Ordering::SeqCst), 4);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn with_pose_refine_the_bracket_limit_is_5_mrad() {
    // as above at twice the drift: every bracket moves about 9.6 mrad, above the 5 mrad limit
    // that applies when the views' rotations are refined (pose_refine auto, the default)
    let creep = Arc::new(Mutex::new(Creep { shake_rate: 0.06, ..Default::default() }));
    let d = CreepArm { inner: common::TestArm::new(), creep: creep.clone() };
    let mut r = rig_driven(ArmConfig::rebot_dm(), false, Some(d)).await;
    r.vision.snapshot_delay_ms.store(60, Ordering::SeqCst);
    r.vision.trigger_delay_ms.store(100, Ordering::SeqCst);
    *r.vision.shake_on_snapshot.lock().unwrap() = Some(creep.clone());
    let ack = r.start(json!({})).await;
    let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
    let a = r.wait(id, 60.0).await;
    assert_eq!((a["state"].as_str(), a["code"].as_str()), (Some("failed"), Some("MOVED")), "{a}");
    let msg = a["message"].as_str().unwrap();
    assert!(msg.contains("J4") && msg.contains("limit 5.0"), "{msg}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn debug_bundle_is_written_capped_and_replays_to_the_same_points() {
    let dir = tempfile::tempdir().unwrap();
    // the only test of this binary that requests debug bundles
    std::env::set_var("ARMD_MVIEW_DEBUG_DIR", dir.path());
    // six stale bundles: the newest five are kept
    for k in 0..6 {
        let d = dir.path().join(format!("old{k}"));
        std::fs::create_dir_all(&d).unwrap();
        std::fs::write(d.join("bundle.json"), "{}").unwrap();
        std::thread::sleep(Duration::from_millis(15));
    }
    std::thread::sleep(Duration::from_millis(20));
    let mut r = rig(ArmConfig::rebot_dm()).await;
    let ack = r.start(json!({"label": "box", "speed_scale": 0.5, "debug": true})).await;
    let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
    let a = r.wait(id, 150.0).await;
    let m = &a["measurement"];
    assert_eq!(a["state"], "done", "{a}");
    assert_eq!(m["debug"]["written"], true, "{}", m["debug"]);
    let b = dir.path().join(id.to_string());
    for k in 0..3 { assert!(b.join(format!("view_{k}.jpg")).exists()); }
    let mut left: Vec<String> = std::fs::read_dir(dir.path()).unwrap().flatten().map(|e| e.file_name().to_string_lossy().into_owned()).collect();
    left.sort();
    assert_eq!(left.len(), mm::MAX_DEBUG_BUNDLES, "{left:?}");
    assert_eq!(left, vec![id.to_string(), "old3".into(), "old4".into(), "old5".into(), "old2".into()].into_iter().collect::<std::collections::BTreeSet<_>>().into_iter().collect::<Vec<_>>(), "the newest 5");
    // the measurement carries the in-box height distribution and the outcome counts
    assert!(m["in_box"]["n"].as_u64().unwrap() >= m["objects"][0]["n_points"].as_u64().unwrap(), "{}", m["in_box"]);
    assert!(m["in_box"]["max_mm"].as_f64().unwrap() > 30.0, "{}", m["in_box"]);
    assert!(m["outcomes"]["corner_box"]["point"].as_u64().unwrap() > 0, "{}", m["outcomes"]);
    // replay: the same points from the stored JPEGs and poses
    let (bundle, c) = mm::replay_bundle(&b).unwrap();
    assert_eq!(bundle.views.len(), 3);
    assert_eq!(c.rec.stats.points as u64, m["stats"]["points"].as_u64().unwrap());
    let h = c.measure.unwrap().height_m;
    assert!((h - m["objects"][0]["height_m"].as_f64().unwrap()).abs() < 1e-4, "{h}");
    let doc = &bundle.doc;
    assert_eq!(doc["points"].as_array().unwrap().len(), c.rec.points.len());
    assert!(doc["views"][1]["joints_rad"].as_array().unwrap().len() >= 6 && doc["views"][0]["detections"][0]["label"] == "box", "{}", doc["views"][0]);
    // phase timing in the bundle: per view and in the stored measurement
    for k in 0..3 { assert_eq!(doc["views"][k]["timing"], m["views"][k]["timing"], "view {k}"); }
    assert_eq!(doc["measurement"]["timing"], m["timing"]);
    check_timing(m, 0.0, 0.0);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_stale_snapshot_from_another_pose_is_not_used() {
    let mut r = rig(ArmConfig::rebot_dm()).await;
    // the snapshot holds a frame taken mid-motion: J1 0.05 rad and J2 0.03 rad away from the start
    let mut q = START.to_vec();
    q[0] += 0.05;
    q[1] += 0.03;
    let stale = r.vision.render(1000, Some(q.clone())).await;
    let stale_id = stale.id;
    *r.vision.snapshot.lock().unwrap() = Some(stale);
    let ack = r.start(json!({"label": "box", "speed_scale": 0.5})).await;
    let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
    let a = r.wait(id, 150.0).await;
    let m = &a["measurement"];
    assert_eq!(a["state"], "done", "{a}");
    assert!((m["objects"][0]["height_m"].as_f64().unwrap() - HEIGHT).abs() < 0.003, "{m}");
    let served = r.vision.served.lock().unwrap().clone();
    // the stale frame was the snapshot before the first trigger, but no view used it
    assert_eq!(served[0].0, stale_id);
    for v in m["views"].as_array().unwrap() {
        let f = v["frame_id"].as_u64().unwrap();
        assert_ne!(f, stale_id, "{v}");
        let joints: Vec<f32> = v["joints_rad"].as_array().unwrap().iter().map(|x| x.as_f64().unwrap() as f32).collect();
        // the accepted snapshot is the frame the view's trigger rendered, at the view's joints
        let (_, rendered_at) = served.iter().find(|s| s.0 == f).unwrap_or_else(|| panic!("frame {f} never served as a snapshot"));
        let diff = rendered_at.iter().zip(&joints).take(6).map(|(a, b)| (a - b).abs()).fold(0.0f32, f32::max);
        assert!(diff < 2e-3, "frame {f} rendered {diff} rad from the view joints");
        // the stale frame was rendered far from every view: used as the view's image it would
        // have paired the image of one pose with the joints of another
        let off = q.iter().zip(&joints).take(6).map(|(a, b)| (a - b).abs()).fold(0.0f32, f32::max);
        assert!(off > 0.02, "stale frame {off} rad from view {v}");
    }
    // after each view motion the snapshot first still held the previous view's frame
    let views = m["views"].as_array().unwrap();
    let ids: Vec<u64> = views.iter().map(|v| v["frame_id"].as_u64().unwrap()).collect();
    let before: Vec<u64> = ids.iter().filter_map(|f| served.iter().position(|s| s.0 == *f)).filter_map(|i| i.checked_sub(1).map(|j| served[j].0)).collect();
    assert_eq!(before.len(), ids.len());
    assert_eq!(&before[1..], &ids[..ids.len() - 1], "the snapshot before each view's trigger was the previous view's frame");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_snapshot_that_never_refreshes_fails_detection_unavailable() {
    let mut r = rig(ArmConfig::rebot_dm()).await;
    r.vision.freeze_snapshot.store(true, Ordering::SeqCst);
    let ack = r.start(json!({"label": "box"})).await;
    let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
    let t = Instant::now();
    let a = r.wait(id, 30.0).await;
    println!("{} {} after {:?}", a["code"], a["message"], t.elapsed());
    assert_eq!((a["state"].as_str(), a["code"].as_str()), (Some("failed"), Some("DETECTION_UNAVAILABLE")), "{a}");
    assert!(a["message"].as_str().unwrap().contains("camera snapshot did not refresh"), "{a}");
    // the snapshot before the trigger, then polls every REFRESH_POLL for REFRESH_MAX
    let n = r.vision.snapshots.load(Ordering::SeqCst);
    let polls = (mm::REFRESH_MAX.as_millis() / mm::REFRESH_POLL.as_millis()) as u64;
    assert!((polls / 2..=polls + 20).contains(&n), "{n} snapshots, about {polls} polls expected");
    assert!(r.at_start().await);
}

/// Six 20 mm markers on the table around the box (ids 0..5, corners counter-clockwise).
fn table_markers(look: [f64; 3]) -> Vec<(i64, [[f64; 3]; 4])> {
    [[-0.07, -0.045], [-0.07, 0.045], [0.07, -0.045], [0.07, 0.045], [0.0, -0.07], [0.0, 0.07]].iter().enumerate().map(|(k, o)| {
        let (x, y) = (look[0] + o[0], look[1] + o[1]);
        (k as i64, [[x - 0.01, y - 0.01, 0.0], [x + 0.01, y - 0.01, 0.0], [x + 0.01, y + 0.01, 0.0], [x - 0.01, y + 0.01, 0.0]])
    }).collect()
}

/// One run with `body` against a rig whose views carry a 1.5 mrad rotation error (a uniform
/// table when `plain_table`); the action and whether it returned to the start.
async fn run_with_rotation_error(body: Value, markers: bool, aruco: bool, plain_table: bool) -> (Value, bool) {
    let config = ArmConfig::rebot_dm();
    let (_, _, look) = geometry(&config);
    let mut r = rig_scene(config, false, plain_table, None).await;
    *r.vision.rot_err.lock().unwrap() = 0.0015;
    if markers { *r.vision.markers.lock().unwrap() = table_markers(look); }
    r.vision.aruco.store(aruco, Ordering::SeqCst);
    let ack = r.start(body).await;
    let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
    let a = r.wait(id, 150.0).await;
    (a, r.at_start().await)
}

fn height(a: &Value) -> f64 { a["measurement"]["objects"][0]["height_m"].as_f64().unwrap_or_else(|| panic!("{a}")) }

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_rotation_error_per_view_biases_the_height_without_refinement() {
    // 1.5 mrad turns each side view's image by about 1 px along the baseline, the two sides
    // opposite ways: the box comes out about 3 mm off. On a uniform table no plane is fitted
    // (on a textured one the fitted plane takes up most of this uniform bias).
    let (a, back) = run_with_rotation_error(json!({"label": "box", "speed_scale": 0.5, "pose_refine": "off"}), false, false, true).await;
    let m = &a["measurement"];
    println!("off: height {}, pose_refine {}, table_plane {}", height(&a), m["pose_refine"], m["table_plane"]);
    assert_eq!(a["state"], "done", "{a}");
    assert!(back);
    assert_eq!(m["pose_refine"], json!({"mode": "off", "applied": false, "markers_requested": false}), "{}", m["pose_refine"]);
    assert!(m["views"][1]["pose_refine"].is_null() && m["table_plane"].is_null(), "{m}");
    assert!((height(&a) - HEIGHT).abs() > 0.002, "height {} without the refinement", height(&a));
    // on the textured table without refinement the triangulated table lies off the calibrated one
    let (a, _) = run_with_rotation_error(json!({"label": "box", "speed_scale": 0.5, "pose_refine": "off"}), false, false, false).await;
    let t = &a["measurement"]["table_check"];
    println!("off, textured table: height {}, table_check {t}, table_plane {}", height(&a), a["measurement"]["table_plane"]);
    assert!(t["median_mm"].as_f64().unwrap().abs() > 2.0, "{t}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_rotation_error_per_view_is_refined_away_from_table_texture() {
    let (a, back) = run_with_rotation_error(json!({"label": "box", "speed_scale": 0.5}), false, false, false).await;
    let m = &a["measurement"];
    println!("auto: height {}, pose_refine {}, table_check {}", height(&a), m["pose_refine"], m["table_check"]);
    assert_eq!(a["state"], "done", "{a}");
    assert!(back);
    let p = &m["pose_refine"];
    assert_eq!((p["mode"].as_str(), p["applied"].as_bool(), p["markers_requested"].as_bool(), p["n_marker_points"].as_u64()), (Some("auto"), Some(true), Some(false), Some(0)), "{p}");
    let views = m["views"].as_array().unwrap();
    assert_eq!(views[0]["pose_refine"]["status"], "reference");
    for v in &views[1..] {
        let r = &v["pose_refine"];
        assert_eq!(r["status"], "refined", "{r}");
        // a turn about the camera's y axis moves the image along the baseline: the
        // triangulation absorbs it in the depths (the reprojection rms does not change), only
        // the coplanarity of the table points finds it
        assert!((r["correction_mrad"].as_f64().unwrap() - 1.5).abs() < 0.2 && r["n_points"].as_u64().unwrap() >= 12, "{r}");
    }
    assert!((height(&a) - HEIGHT).abs() < 0.001, "height {}", height(&a));
    assert!(m["table_check"]["median_mm"].as_f64().unwrap().abs() < 1.0, "{}", m["table_check"]);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn marker_corners_refine_the_poses_in_aruco_and_auto_mode() {
    // aruco: the marker corners alone, on a uniform table (no texture, no fitted plane)
    let (a, back) = run_with_rotation_error(json!({"label": "box", "speed_scale": 0.5, "pose_refine": "aruco"}), true, true, true).await;
    let m = &a["measurement"];
    println!("aruco: height {}, pose_refine {}", height(&a), m["pose_refine"]);
    assert_eq!(a["state"], "done", "{a}");
    assert!(back);
    let p = &m["pose_refine"];
    assert_eq!((p["mode"].as_str(), p["markers_requested"].as_bool()), (Some("aruco"), Some(true)), "{p}");
    assert_eq!(p["n_points"], p["n_marker_points"], "marker corners only: {p}");
    for v in &m["views"].as_array().unwrap()[1..] {
        let r = &v["pose_refine"];
        assert_eq!(r["status"], "refined", "{r}");
        assert!(r["n_marker_corners"].as_u64().unwrap() >= 8 && (r["correction_mrad"].as_f64().unwrap() - 1.5).abs() < 0.3, "{r}");
    }
    assert!((height(&a) - HEIGHT).abs() < 0.0015 && m["table_plane"].is_null(), "height {}", height(&a));
    // auto: table texture plus the marker corners
    let (a, _) = run_with_rotation_error(json!({"label": "box", "speed_scale": 0.5}), true, true, false).await;
    let p = &a["measurement"]["pose_refine"];
    println!("auto with markers: height {}, pose_refine {p}", height(&a));
    assert_eq!(a["state"], "done", "{a}");
    assert!(p["markers_requested"] == true && p["n_marker_points"].as_u64().unwrap() >= 16 && p["n_points"].as_u64() > p["n_marker_points"].as_u64(), "{p}");
    assert!((height(&a) - HEIGHT).abs() < 0.003, "height {}", height(&a));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn aruco_mode_without_markers_fails_markers_unavailable() {
    // the provider runs ArUco but no marker is in view: after the views, back at the start
    let (a, back) = run_with_rotation_error(json!({"label": "box", "speed_scale": 0.5, "pose_refine": "aruco"}), false, true, false).await;
    println!("{} {}", a["code"], a["message"]);
    assert_eq!((a["state"].as_str(), a["code"].as_str()), (Some("failed"), Some("MARKERS_UNAVAILABLE")), "{a}");
    assert!(a["message"].as_str().unwrap().contains("marker corners"), "{a}");
    assert!(back, "returned to the start joints");
    // a provider without the aruco pipeline: refused before any motion or capture
    let mut r = rig(ArmConfig::rebot_dm()).await;
    let ack = r.start(json!({"pose_refine": "aruco"})).await;
    let a = r.wait(ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}")), 30.0).await;
    assert_eq!((a["state"].as_str(), a["code"].as_str()), (Some("failed"), Some("MARKERS_UNAVAILABLE")), "{a}");
    assert_eq!(r.vision.snapshots.load(Ordering::SeqCst), 0);
    assert!(r.at_start().await);
    let rej = r.start(json!({"pose_refine": "markers"})).await;
    assert_eq!(rej["code"], "INVALID_COMMAND", "{rej}");
}

/// `pose_refine` "off" reconstructs with the kinematic cameras, as before the refinement; on
/// exact poses "auto" changes them by almost nothing.
#[test]
fn off_uses_the_kinematic_cameras() {
    let config = ArmConfig::rebot_dm();
    let (cam, pose, look) = geometry(&config);
    let scene = Scene::new(0.0, vec![BoxObj { xy: [look[0], look[1]], yaw: 0.3, size: [0.05, 0.03, HEIGHT] }]);
    let c0 = mm::camera_at(&cam, &pose);
    let pivot = cam.plane_point(&pose, [cam.cx, cam.cy], mm::LOOK_AT_ABOVE_TABLE_M).unwrap();
    let axes = mm::plane_axes(&c0, &pivot, mm::Motion::Orbit);
    let (w, h) = (cam.size[0] as usize, cam.size[1] as usize);
    let views: Vec<armd::mview_stereo::View> = [[0.0, 0.0], [-0.04, 0.0], [0.04, 0.0]].iter().enumerate().map(|(k, o)| {
        let c = mm::camera_at(&cam, &mm::view_pose(&pose, &c0.c, &pivot, &axes, *o, mm::Motion::Orbit));
        armd::mview_stereo::View { cam: c, img: render::render(&scene, &c, w, h, 2, Noise { sigma: 1.0, ..Noise::default() }, k as u64 + 1) }
    }).collect();
    let bx = render::box_px(&scene, &scene.boxes[0], &views[0].cam);
    let off = mm::compute(&views, 0, bx, 0.0, mm::PoseRefine::Off, &[]);
    assert!(off.pose.is_none());
    let p = armd::mview_stereo::Params { exclude_px: Some(mm::gripper_region_px(cam.size)), ..armd::mview_stereo::Params::new(0.0) };
    let plain = armd::mview_stereo::reconstruct(&views, 0, off.region_px, bx, &p);
    assert_eq!(off.rec.points, plain.points);
    let auto = mm::compute(&views, 0, bx, 0.0, mm::PoseRefine::Auto, &[]);
    let pr = auto.pose.unwrap();
    println!("{:?}", pr.views);
    assert!(pr.views[1..].iter().all(|v| v.status == "refined" && v.correction_mrad < 0.3), "{:?}", pr.views);
    assert!((auto.measure.unwrap().height_m - off.measure.unwrap().height_m).abs() < 0.002);
}

/// `measurement.timing` and `views[].timing` are consistent: per-view phases add up to the
/// action sums, every trigger / fetch took at least the fake provider's delay, the overlap of the
/// reconstruction with the return is within both, and the phases fit in the total.
fn check_timing(m: &Value, trigger_min_ms: f64, fetch_min_ms: f64) {
    let t = &m["timing"];
    let f = |v: &Value, k: &str| v[k].as_f64().unwrap_or_else(|| panic!("{k} in {v}"));
    let views = m["views"].as_array().unwrap();
    for (k, v) in views.iter().enumerate() {
        let vt = &v["timing"];
        assert!(f(vt, "trigger_ms") >= trigger_min_ms && f(vt, "fetch_ms") >= fetch_min_ms, "view {k}: {vt}");
        assert!(f(vt, "capture_ms") + 0.2 >= f(vt, "trigger_ms") + f(vt, "fetch_ms"), "view {k}: {vt}");
        assert!(f(vt, "settle_ms") > 0.0, "view {k}: {vt}");
        if k == 0 { assert_eq!(f(vt, "motion_ms"), 0.0, "{vt}"); } else { assert!(f(vt, "motion_ms") > 100.0, "view {k}: {vt}"); }
    }
    for key in ["motion_ms", "settle_ms", "trigger_ms", "fetch_ms", "capture_ms"] {
        let sum: f64 = views.iter().map(|v| f(&v["timing"], key)).sum();
        assert!((sum - f(t, key)).abs() < 0.5, "{key}: {sum} vs {t}");
    }
    let (ret, comp, ov, total) = (f(t, "return_ms"), f(t, "compute_ms"), f(t, "compute_return_overlap_ms"), f(t, "total_ms"));
    assert!(ret > 100.0 && comp > 0.0 && ov >= 0.0 && ov <= ret.min(comp) + 0.2, "{t}");
    let first = &views[0]["timing"];
    assert!(f(t, "detect_ms") + 0.5 >= f(first, "settle_ms") + f(first, "capture_ms"), "{t}");
    let parts = f(t, "negotiate_ms") + f(t, "detect_ms") + f(t, "views_ms") + ret + comp - ov;
    assert!(parts <= total + 50.0 && total <= parts + 2000.0, "parts {parts} vs {t}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn gray_capture_succeeds_with_zero_snapshots_and_reports_metrics() {
    let mut r = rig(ArmConfig::rebot_dm()).await;
    r.vision.gray.store(true, Ordering::SeqCst);
    r.vision.trigger_delay_ms.store(40, Ordering::SeqCst);
    r.vision.gray_fetch_delay_ms.store(30, Ordering::SeqCst);
    let ack = r.start(json!({"label": "box", "speed_scale": 0.5})).await;
    let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
    let a = r.wait(id, 150.0).await;
    assert_eq!(a["state"], "done", "{a}");
    assert_eq!(r.vision.snapshots.load(Ordering::SeqCst), 0, "gray path must issue zero snapshot GETs");
    assert!(r.vision.gray_fetches.load(Ordering::SeqCst) >= 3);
    let m = &a["measurement"];
    assert_eq!(m["frame_source"], "gray", "{m}");
    assert!((m["objects"][0]["height_m"].as_f64().unwrap() - HEIGHT).abs() < 0.003, "{m}");
    for v in m["views"].as_array().unwrap() {
        assert_eq!(v["frame_source"], "gray", "{v}");
        assert_eq!(v["instance_id"], "rv-fake-1", "{v}");
        assert_eq!(v["decode_ms"].as_f64().unwrap(), 0.0, "{v}");
        assert!(v["frame_fetch_ms"].as_f64().unwrap() >= 0.0, "{v}");
        assert!(v["bracket_duration_ms"].as_f64().unwrap() >= 0.0, "{v}");
        assert!(v["feedback_wait_b_ms"].as_f64().unwrap() >= 0.0, "{v}");
        assert!(v["feedback_age_a_ms"].is_number() && v["feedback_age_b_ms"].is_number(), "{v}");
    }
    assert!(m["stats"]["frame_fetch_ms"].is_number() && m["stats"]["parse_ms"].is_number(), "{}", m["stats"]);
    check_timing(m, 40.0, 30.0);
    println!("timing {}", m["timing"]);
    // the start view skips the minimum settle wait (still windows only); the view poses wait 2 s
    let settle: Vec<f64> = m["views"].as_array().unwrap().iter().map(|v| v["timing"]["settle_ms"].as_f64().unwrap()).collect();
    assert!(settle[0] < 1000.0 && settle[0] >= (mm::STILL_WINDOW as f64 - 1.0) * 20.0, "{settle:?}");
    assert!(settle[1..].iter().all(|s| *s >= mm::MIN_SETTLE_S * 1000.0), "{settle:?}");
    // the camera was held for the run (refreshed: the run lasts several refresh periods) and released
    assert_eq!(m["camera_hold"], "held", "{m}");
    assert!(r.vision.holds.load(Ordering::SeqCst) >= 3, "{}", r.vision.holds.load(Ordering::SeqCst));
    assert!(!r.vision.held.load(Ordering::SeqCst) && *r.vision.releases.lock().unwrap() == vec!["s7".to_string()]);
}

/// After the action ended: the camera hold is released once and no longer refreshed.
async fn assert_released(r: &Rig) {
    let t = Instant::now();
    while r.vision.held.load(Ordering::SeqCst) {
        assert!(t.elapsed() < Duration::from_secs(3), "the camera hold was not released");
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    let n = r.vision.holds.load(Ordering::SeqCst);
    tokio::time::sleep(mm::CAMERA_HOLD_REFRESH + Duration::from_millis(300)).await;
    assert_eq!(r.vision.holds.load(Ordering::SeqCst), n, "refreshed after the release");
    assert!(!r.vision.held.load(Ordering::SeqCst));
    assert_eq!(*r.vision.releases.lock().unwrap(), vec!["s7".to_string()]);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn the_camera_hold_is_released_on_failure_and_on_lease_loss() {
    // failure after motion: TARGET_LOST in the first side view, the arm returns, the hold ends
    let mut r = rig(ArmConfig::rebot_dm()).await;
    r.vision.gray.store(true, Ordering::SeqCst);
    r.vision.hide_after_start.store(true, Ordering::SeqCst);
    let ack = r.start(json!({"speed_scale": 0.5})).await;
    let a = r.wait(ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}")), 90.0).await;
    assert_eq!((a["state"].as_str(), a["code"].as_str()), (Some("failed"), Some("TARGET_LOST")), "{a}");
    assert!(r.vision.holds.load(Ordering::SeqCst) >= 1);
    assert_released(&r).await;
    // lease loss during a view motion: the action ends cancelled, the hold ends
    let mut r = rig(ArmConfig::rebot_dm()).await;
    r.vision.gray.store(true, Ordering::SeqCst);
    let ack = r.start(json!({"speed_scale": 0.1})).await;
    let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
    let until = Instant::now() + Duration::from_secs(60);
    loop {
        assert!(Instant::now() < until, "no view motion");
        common::send(&mut r.ws, json!({"type":"heartbeat", "generation": r.generation, "epoch_id": r.epoch})).await;
        common::drain(&mut r.ws, 5).await;
        let moving = r.sup.inner().read().await.last_action.as_ref().and_then(|a| a.phase).is_some_and(|p| p.starts_with("move:"));
        if moving { break; }
        tokio::time::sleep(Duration::from_millis(30)).await;
    }
    assert!(r.vision.held.load(Ordering::SeqCst), "held during the views");
    let reply = common::ws_reply(&mut r.ws, json!({"type":"release_lease", "generation": r.generation}), &["lease_released"]).await;
    assert_eq!(reply["success"], true);
    let t = Instant::now();
    loop {
        let obs = r.sup.get_observation().await;
        if let Some(a) = [obs.action, obs.previous_action].into_iter().flatten().find(|a| a.id == id && !a.is_running()) {
            assert_eq!(a.code.as_deref(), Some("LEASE_LOST"), "{a:?}");
            break;
        }
        assert!(t.elapsed() < Duration::from_secs(3), "the action did not end");
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    assert_released(&r).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn the_camera_hold_release_uses_the_latest_ticket_after_a_provider_restart() {
    // from the second hold request on (the first refresh) the provider answers as restarted: a
    // new subscription s1 of another instance; the release names that one, not the first id s7
    let mut r = rig(ArmConfig::rebot_dm()).await;
    r.vision.gray.store(true, Ordering::SeqCst);
    r.vision.hold_restart_after.store(2, Ordering::SeqCst);
    let ack = r.start(json!({"label": "box", "speed_scale": 0.5})).await;
    let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
    let a = r.wait(id, 150.0).await;
    assert_eq!(a["state"], "done", "{a}");
    assert!(r.vision.holds.load(Ordering::SeqCst) >= 2);
    assert_eq!(*r.vision.releases.lock().unwrap(), vec!["s1".to_string()]);
    assert_eq!(*r.vision.release_args.lock().unwrap(), vec![(mm::camera_hold_client(id), Some("rv-restarted".to_string()))]);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn every_run_holds_the_camera_under_its_own_client() {
    // two runs one after the other: each holds, refreshes and releases under armd-mview-<action
    // id>, so a late release of the first cannot end the second's hold at an owner-checking provider
    let mut r = rig(ArmConfig::rebot_dm()).await;
    r.vision.gray.store(true, Ordering::SeqCst);
    let mut ids = Vec::new();
    for _ in 0..2 {
        let ack = r.start(json!({"label": "box", "speed_scale": 0.5, "views": 2})).await;
        let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
        let a = r.wait(id, 150.0).await;
        assert_eq!(a["state"], "done", "{a}");
        ids.push(id);
    }
    assert_ne!(ids[0], ids[1]);
    let clients = r.vision.hold_clients.lock().unwrap().clone();
    let (c0, c1) = (mm::camera_hold_client(ids[0]), mm::camera_hold_client(ids[1]));
    assert_eq!(c0, format!("armd-mview-{}", ids[0]));
    let split = clients.iter().position(|c| *c == c1).expect("the second run held the camera");
    assert!(split > 0 && clients[..split].iter().all(|c| *c == c0) && clients[split..].iter().all(|c| *c == c1), "{clients:?}");
    let released: Vec<String> = r.vision.release_args.lock().unwrap().iter().map(|(c, _)| c.clone()).collect();
    assert_eq!(released, vec![c0, c1]);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn without_a_camera_hold_the_run_is_unchanged_and_snapshots_never_hold() {
    // an older provider (no camera hold): the gray run measures as before
    let mut r = rig(ArmConfig::rebot_dm()).await;
    r.vision.gray.store(true, Ordering::SeqCst);
    r.vision.hold_supported.store(false, Ordering::SeqCst);
    let ack = r.start(json!({"label": "box", "speed_scale": 0.5})).await;
    let a = r.wait(ack["action_id"].as_u64().unwrap(), 150.0).await;
    assert_eq!(a["state"], "done", "{a}");
    assert_eq!(a["measurement"]["camera_hold"], "unsupported");
    assert!((a["measurement"]["objects"][0]["height_m"].as_f64().unwrap() - HEIGHT).abs() < 0.003);
    assert!(r.vision.releases.lock().unwrap().is_empty());
    // snapshot source: the hold is never requested (the snapshot would refresh with frames that
    // are not the trigger's)
    let mut r = rig(ArmConfig::rebot_dm()).await;
    let ack = r.start(json!({"label": "box", "speed_scale": 0.5})).await;
    let a = r.wait(ack["action_id"].as_u64().unwrap(), 150.0).await;
    assert_eq!((a["state"].as_str(), a["measurement"]["camera_hold"].as_str()), (Some("done"), Some("off")), "{a}");
    assert_eq!(r.vision.holds.load(Ordering::SeqCst), 0);
}


#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn settle_min_s_sets_the_wait_at_the_view_poses() {
    // 0.5 s instead of 2.0 s at the view poses (the start view has none); the still windows and
    // the bracket still decide, the measurement is the same
    let mut r = rig(ArmConfig::rebot_dm()).await;
    r.vision.gray.store(true, Ordering::SeqCst);
    let ack = r.start(json!({"label": "box", "speed_scale": 0.5, "settle_min_s": 0.5})).await;
    let a = r.wait(ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}")), 150.0).await;
    assert_eq!(a["state"], "done", "{a}");
    let m = &a["measurement"];
    assert_eq!(m["settle_min_s"], 0.5);
    let settle: Vec<f64> = m["views"].as_array().unwrap().iter().map(|v| v["timing"]["settle_ms"].as_f64().unwrap()).collect();
    assert!(settle[0] < 1000.0, "{settle:?}");
    assert!(settle[1..].iter().all(|s| (500.0..1500.0).contains(s)), "{settle:?}");
    assert!((m["objects"][0]["height_m"].as_f64().unwrap() - HEIGHT).abs() < 0.003, "{m}");
    let d = r.sup.inner().read().await.safety_describe();
    assert_eq!(d["limits"]["measure_multiview"]["settle_min_s"], json!([0.5, 3.0]));
    // the longest settle deadline a request can get: settle_min_s 3 + the 2 s of still windows
    assert_eq!(d["limits"]["measure_multiview"]["max_settle_s"], json!(5.0));
    assert_eq!((mm::max_settle_s(mm::MIN_SETTLE_S), mm::max_settle_s(0.5)), (mm::MAX_SETTLE_S, 2.5));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn an_arm_moving_at_the_start_fails_not_settled_before_any_trigger() {
    // J4 drifts 50 mrad/s from the start (above the 0.03 rad/s still velocity): without the
    // minimum wait the start view still needs a still window, so it ends NOT_SETTLED after the
    // window time (MAX_SETTLE_S - MIN_SETTLE_S) without triggering the camera or moving
    let creep = Arc::new(Mutex::new(Creep { shake: true, shake_rate: 0.05, ..Default::default() }));
    let d = CreepArm { inner: common::TestArm::new(), creep: creep.clone() };
    let mut r = rig_driven(ArmConfig::rebot_dm(), false, Some(d)).await;
    r.vision.gray.store(true, Ordering::SeqCst);
    let t0 = Instant::now();
    let ack = r.start(json!({"label": "box"})).await;
    let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
    let a = r.wait(id, 30.0).await;
    let took = t0.elapsed().as_secs_f64();
    assert_eq!((a["state"].as_str(), a["code"].as_str()), (Some("failed"), Some("NOT_SETTLED")), "{a}");
    let window = mm::MAX_SETTLE_S - mm::MIN_SETTLE_S;
    assert!(a["message"].as_str().unwrap().contains(&format!("within {window} s")), "{a}");
    // seen moving with fresh feedback: NOT_SETTLED, with the per-joint update counts
    assert!(a["message"].as_str().unwrap().contains("feedback updates per joint") && a["measurement"]["settle"]["window_samples"] == 10, "{a}");
    assert!(took >= window && took < window + 3.0, "{took}");
    assert_eq!(r.vision.frame.load(Ordering::SeqCst), 0, "no trigger before the arm is still");
    assert!(r.at_start().await);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn gray_transient_eviction_retries_with_new_bracket_and_trigger() {
    let mut r = rig(ArmConfig::rebot_dm()).await;
    r.vision.gray.store(true, Ordering::SeqCst);
    r.vision.gray_fail_queue.lock().unwrap().push_back(GrayFrameError::Expired("gray frame 1: 410 FRAME_EXPIRED".into()));
    let ack = r.start(json!({"label": "box", "speed_scale": 0.5})).await;
    let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
    let a = r.wait(id, 150.0).await;
    assert_eq!(a["state"], "done", "{a}");
    assert_eq!(r.vision.snapshots.load(Ordering::SeqCst), 0);
    let v0 = &a["measurement"]["views"][0];
    assert_eq!(v0["retry_reasons"], json!(["expired"]), "{v0}");
    assert_eq!(v0["snapshot_tries_mrad"].as_array().unwrap().len(), 1, "acquisition failures stay separate from numeric spread tries: {v0}");
    assert!(v0["frame_id"].as_u64().unwrap() >= 2, "retried with a new trigger frame: {v0}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn gray_permanent_failure_after_side_view_move_never_switches_and_returns_to_start() {
    let mut r = rig(ArmConfig::rebot_dm()).await;
    r.vision.gray.store(true, Ordering::SeqCst);
    // Start view succeeds; side view hits permanent eviction across all tries.
    *r.vision.gray_fail_after_start.lock().unwrap() = Some(GrayFrameError::Expired("gray frame: 410 FRAME_EXPIRED".into()));
    let ack = r.start(json!({"label": "box", "speed_scale": 0.5})).await;
    let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
    let a = r.wait(id, 150.0).await;
    assert_eq!((a["state"].as_str(), a["code"].as_str()), (Some("failed"), Some("DETECTION_UNAVAILABLE")), "{a}");
    assert_eq!(r.vision.snapshots.load(Ordering::SeqCst), 0, "must never switch to /snapshot.jpg mid-run");
    assert!(r.at_start().await, "must return to start joints after side-view gray failure");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn gray_motion_during_bracket_and_fetch_fails_moved() {
    // Still while settling; on first gray trigger J4 drifts 30 mrad/s across trigger (80 ms) + fetch (60 ms) -> > 4 mrad > 2 mrad limit.
    let creep = Arc::new(Mutex::new(Creep { shake_rate: 0.03, ..Default::default() }));
    let d = CreepArm { inner: common::TestArm::new(), creep: creep.clone() };
    let mut r = rig_driven(ArmConfig::rebot_dm(), false, Some(d)).await;
    r.vision.gray.store(true, Ordering::SeqCst);
    r.vision.trigger_delay_ms.store(80, Ordering::SeqCst);
    r.vision.gray_fetch_delay_ms.store(60, Ordering::SeqCst);
    *r.vision.shake_on_gray_trigger.lock().unwrap() = Some(creep.clone());
    let ack = r.start(json!({"pose_refine": "off"})).await;
    let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
    let a = r.wait(id, 60.0).await;
    assert_eq!((a["state"].as_str(), a["code"].as_str()), (Some("failed"), Some("MOVED")), "{a}");
    assert_eq!(r.vision.snapshots.load(Ordering::SeqCst), 0);
    let msg = a["message"].as_str().unwrap();
    assert!(msg.contains("J4") && msg.contains("limit 2.0"), "{msg}");
}

struct ScheduledRxFrame {
    deliver_at: Instant,
    joint: usize,
    step_rad: f32,
}

/// Queue-backed feedback fake modelling SocketCAN + Damiao decode-time stamping:
/// frames arrive in `rx_queue`, `read_joint_states()` drains at most `drain_limit` frames per
/// tick, stamps `Instant::now()` at decode time, increments `seqs[joint]` once per decoded frame,
/// and reports as queue bound the instant before the last read that found `rx_queue` empty.
struct FeedbackControl {
    /// Joint held off auto-enqueue (until `unhold_at`, or indefinitely if `None`).
    held_joint: Option<(usize, Option<Instant>)>,
    /// Frames queued to become readable at `deliver_at` (emulating queued CAN RX frames).
    pending_rx: VecDeque<ScheduledRxFrame>,
    /// RX buffer drained by `read_joint_states()`.
    rx_queue: VecDeque<(usize, f32)>,
    applied_step: [f32; 7],
    instants: [Instant; 7],
    seqs: [u64; 7],
    /// Frames decoded per `read_joint_states()` (the driver's bounded drain).
    drain_limit: usize,
    rx_empty: Instant,
    bounds: [Instant; 7],
}

impl FeedbackControl {
    fn new(now: Instant) -> Self {
        Self {
            held_joint: None,
            pending_rx: VecDeque::new(),
            rx_queue: VecDeque::new(),
            applied_step: [0.0; 7],
            instants: [now; 7],
            seqs: [1; 7],
            drain_limit: usize::MAX,
            rx_empty: now,
            bounds: [now; 7],
        }
    }
}

struct QueuedFeedbackArm {
    inner: common::TestArm,
    ctrl: Arc<Mutex<FeedbackControl>>,
}

impl ArmDriver for QueuedFeedbackArm {
    fn capabilities(&self) -> DriverCapabilities { self.inner.capabilities() }
    fn enable_state(&self) -> EnableState { self.inner.enable_state() }
    fn is_feedback_valid(&self) -> bool { self.inner.is_feedback_valid() }
    fn is_enabled(&self) -> bool { self.inner.is_enabled() }
    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        let c = self.ctrl.lock().unwrap();
        let now = Instant::now();
        c.instants.iter().map(|t| Some(now.saturating_duration_since(*t))).collect()
    }
    fn feedback_instants(&self) -> Vec<Option<Instant>> {
        let c = self.ctrl.lock().unwrap();
        c.instants.iter().copied().map(Some).collect()
    }
    fn feedback_seqs(&self) -> Vec<u64> {
        let c = self.ctrl.lock().unwrap();
        c.seqs.to_vec()
    }
    fn feedback_queue_bounds(&self) -> Vec<Option<Instant>> {
        let c = self.ctrl.lock().unwrap();
        c.bounds.iter().copied().map(Some).collect()
    }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let now = Instant::now();
        let step = {
            let mut c = self.ctrl.lock().unwrap();
            // Release held joint if its unhold deadline arrived.
            if let Some((_, Some(unhold_at))) = c.held_joint {
                if now >= unhold_at {
                    c.held_joint = None;
                }
            }
            let held = c.held_joint.map(|(j, _)| j);
            // Enqueue normal 50 Hz bus replies for unheld joints.
            for j in 0..7 {
                if held != Some(j) {
                    let s = c.applied_step[j];
                    c.rx_queue.push_back((j, s));
                }
            }
            // Move any scheduled queued CAN frames whose delivery time has arrived into rx_queue.
            let mut remaining = VecDeque::new();
            while let Some(ev) = c.pending_rx.pop_front() {
                if now >= ev.deliver_at {
                    c.rx_queue.push_back((ev.joint, ev.step_rad));
                } else {
                    remaining.push_back(ev);
                }
            }
            c.pending_rx = remaining;
            // Bounded drain, stamped at decode time (just like DamiaoArmDriver::read_joint_states).
            for _ in 0..c.drain_limit {
                let attempt = Instant::now();
                let Some((j, step_rad)) = c.rx_queue.pop_front() else {
                    c.rx_empty = attempt;
                    break;
                };
                c.applied_step[j] = step_rad;
                c.instants[j] = Instant::now();
                c.seqs[j] = c.seqs[j].wrapping_add(1);
                c.bounds[j] = c.rx_empty;
            }
            c.applied_step
        };
        let mut s = self.inner.read_joint_states()?;
        for (i, js) in s.iter_mut().enumerate().take(7) {
            js.position += step[i];
        }
        Ok(s)
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> { self.inner.write_joint_targets(targets) }
    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> { self.inner.set_torque(enabled) }
    fn supports_mit_mode_switch(&self) -> bool { false }
    fn set_joint_modes(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> { self.inner.set_joint_modes(modes, hold) }
}

async fn rig_with_driver(config: ArmConfig, driver: Box<dyn ArmDriver>) -> Rig {
    let (cam, _, look) = geometry(&config);
    let sup = Arc::new(ArmSupervisor::new(config));
    sup.inner().write().await.driver = Some(std::sync::Mutex::new(driver));
    let scene = Scene::new(0.0, vec![BoxObj { xy: [look[0], look[1]], yaw: 0.3, size: [0.05, 0.03, HEIGHT] }]);
    let vision = Arc::new(FakeVision { sup: sup.clone(), cam, scene: Arc::new(scene), hide_after_start: AtomicBool::new(false), frame: AtomicU64::new(0),
        snapshots: AtomicU64::new(0), snapshot_delay_ms: AtomicU64::new(0), trigger_delay_ms: AtomicU64::new(0), freeze_snapshot: AtomicBool::new(false),
        shake_on_snapshot: Mutex::new(None), shake_on_gray_trigger: Mutex::new(None), clock: Mutex::new(None), snapshot: Mutex::new(None), served: Mutex::new(Vec::new()),
        rot_err: Mutex::new(0.0), markers: Mutex::new(Vec::new()), aruco: AtomicBool::new(false),
        gray: AtomicBool::new(true), instance_id: Mutex::new("rv-fb-1".into()), gray_ring: Mutex::new(VecDeque::new()),
        gray_fetches: AtomicU64::new(0), gray_fetch_delay_ms: AtomicU64::new(0), gray_fail_queue: Mutex::new(VecDeque::new()),
        gray_fail_after_start: Mutex::new(None), on_gray_fetch: Mutex::new(None),
        gray_trigger_timeouts: AtomicU64::new(0), on_gray_trigger_timeout: Mutex::new(None),
        hold_supported: AtomicBool::new(true), holds: AtomicU64::new(0), releases: Mutex::new(Vec::new()), held: AtomicBool::new(false),
        hold_restart_after: AtomicU64::new(0), release_args: Mutex::new(Vec::new()), hold_clients: Mutex::new(Vec::new()), on_trigger: Mutex::new(None), contours: AtomicBool::new(false) });
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
    let epoch = sup.get_observation().await.epoch_id;
    Rig { sup, vision, ws, generation, epoch }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn gray_sample_b_waits_for_staggered_joint_update_and_rejects_cached_or_stale() {
    // 1. Pre-fence queued CAN frame (old 0-step pose) delivered 25 ms AFTER the fence (advancing
    //    seq by +1 with a post-fence decode timestamp), followed by the genuine post-fence frame
    //    (+8 mrad step on J5) at 90 ms (seq +2). A timestamp-only fence would accept the queued
    //    pre-fence frame and miss the move; requiring seq +2 catches J5 -> MOVED.
    {
        let inner_arm = common::TestArm::new();
        inner_arm.0.lock().unwrap().q = START;
        let ctrl = Arc::new(Mutex::new(FeedbackControl::new(Instant::now())));
        let drv = QueuedFeedbackArm { inner: inner_arm, ctrl: ctrl.clone() };
        let mut r = rig_with_driver(ArmConfig::rebot_dm(), Box::new(drv)).await;
        let ctrl_cb = ctrl.clone();
        *r.vision.on_gray_fetch.lock().unwrap() = Some(Box::new(move || {
            let t = Instant::now();
            let mut c = ctrl_cb.lock().unwrap();
            let prev_step = c.applied_step[4];
            c.held_joint = Some((4, Some(t + Duration::from_millis(120))));
            c.pending_rx.push_back(ScheduledRxFrame { deliver_at: t + Duration::from_millis(25), joint: 4, step_rad: prev_step });
            c.pending_rx.push_back(ScheduledRxFrame { deliver_at: t + Duration::from_millis(90), joint: 4, step_rad: prev_step + 0.008 });
        }));
        let ack = r.start(json!({"pose_refine": "off"})).await;
        let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
        let a = r.wait(id, 60.0).await;
        assert_eq!((a["state"].as_str(), a["code"].as_str()), (Some("failed"), Some("MOVED")), "{a}");
        assert!(a["message"].as_str().unwrap().contains("J5"), "{a}");
    }

    // 2. Only ONE pre-fence queued frame arrives after the fence (seq +1, fresh decode timestamp),
    //    and no second post-fence update arrives within 500 ms -> fails STALE_TELEMETRY.
    {
        let inner_arm = common::TestArm::new();
        inner_arm.0.lock().unwrap().q = START;
        let ctrl = Arc::new(Mutex::new(FeedbackControl::new(Instant::now())));
        let drv = QueuedFeedbackArm { inner: inner_arm, ctrl: ctrl.clone() };
        let mut r = rig_with_driver(ArmConfig::rebot_dm(), Box::new(drv)).await;
        let ctrl_cb = ctrl.clone();
        *r.vision.on_gray_fetch.lock().unwrap() = Some(Box::new(move || {
            let t = Instant::now();
            let mut c = ctrl_cb.lock().unwrap();
            let prev_step = c.applied_step[4];
            c.held_joint = Some((4, None));
            c.pending_rx.push_back(ScheduledRxFrame { deliver_at: t + Duration::from_millis(25), joint: 4, step_rad: prev_step });
        }));
        let ack = r.start(json!({"label": "box", "speed_scale": 0.5})).await;
        let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
        let a = r.wait(id, 60.0).await;
        assert_eq!((a["state"].as_str(), a["code"].as_str()), (Some("failed"), Some("STALE_TELEMETRY")), "{a}");
    }

    // 3. Staggered 2nd post-fence update for J5 arrives at 80 ms with 0 step -> waits and succeeds,
    //    recording feedback_wait_b_ms >= 50 ms.
    {
        let inner_arm = common::TestArm::new();
        inner_arm.0.lock().unwrap().q = START;
        let ctrl = Arc::new(Mutex::new(FeedbackControl::new(Instant::now())));
        let drv = QueuedFeedbackArm { inner: inner_arm, ctrl: ctrl.clone() };
        let mut r = rig_with_driver(ArmConfig::rebot_dm(), Box::new(drv)).await;
        let fired = Arc::new(AtomicBool::new(false));
        let ctrl_cb = ctrl.clone();
        let fired_cb = fired.clone();
        *r.vision.on_gray_fetch.lock().unwrap() = Some(Box::new(move || {
            if !fired_cb.swap(true, Ordering::SeqCst) {
                let t = Instant::now();
                let mut c = ctrl_cb.lock().unwrap();
                c.held_joint = Some((4, Some(t + Duration::from_millis(120))));
                c.pending_rx.push_back(ScheduledRxFrame { deliver_at: t + Duration::from_millis(25), joint: 4, step_rad: 0.0 });
                c.pending_rx.push_back(ScheduledRxFrame { deliver_at: t + Duration::from_millis(80), joint: 4, step_rad: 0.0 });
            }
        }));
        let ack = r.start(json!({"label": "box", "speed_scale": 0.5})).await;
        let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
        let a = r.wait(id, 150.0).await;
        assert_eq!(a["state"], "done", "{a}");
        let wait_b = a["measurement"]["views"][0]["feedback_wait_b_ms"].as_f64().unwrap();
        assert!(wait_b >= 50.0, "expected staggered wait >= 50 ms, got {wait_b}");
    }

    // 4. Frozen J3 feedback delivers 0 updates after fetch -> fails STALE_TELEMETRY.
    {
        let inner_arm = common::TestArm::new();
        inner_arm.0.lock().unwrap().q = START;
        let ctrl = Arc::new(Mutex::new(FeedbackControl::new(Instant::now())));
        let drv = QueuedFeedbackArm { inner: inner_arm, ctrl: ctrl.clone() };
        let mut r = rig_with_driver(ArmConfig::rebot_dm(), Box::new(drv)).await;
        let ctrl_cb = ctrl.clone();
        *r.vision.on_gray_fetch.lock().unwrap() = Some(Box::new(move || {
            ctrl_cb.lock().unwrap().held_joint = Some((2, None));
        }));
        let ack = r.start(json!({"label": "box", "speed_scale": 0.5})).await;
        let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
        let a = r.wait(id, 60.0).await;
        assert_eq!((a["state"].as_str(), a["code"].as_str()), (Some("failed"), Some("STALE_TELEMETRY")), "{a}");
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn gray_sample_b_rejects_two_old_replies_drained_after_the_fence() {
    // At the fence the RX queue already holds a backlog: filler replies of other joints and two
    // OLD replies of J5, which then stays silent. The bounded drain (8 frames per tick, 6 fresh
    // replies arrive per tick) decodes the backlog over several ticks after the fence, so J5's
    // decode stamps are post-fence and its counter advances by 2. Neither proves a post-fence
    // sample: with no fresh J5 reply the capture must fail STALE_TELEMETRY.
    let inner_arm = common::TestArm::new();
    inner_arm.0.lock().unwrap().q = START;
    let ctrl = Arc::new(Mutex::new(FeedbackControl::new(Instant::now())));
    let drv = QueuedFeedbackArm { inner: inner_arm, ctrl: ctrl.clone() };
    let mut r = rig_with_driver(ArmConfig::rebot_dm(), Box::new(drv)).await;
    let ctrl_cb = ctrl.clone();
    *r.vision.on_gray_fetch.lock().unwrap() = Some(Box::new(move || {
        let mut c = ctrl_cb.lock().unwrap();
        let old = c.applied_step[4];
        c.held_joint = Some((4, None));
        c.drain_limit = 8;
        for k in 0..20 {
            let j = [0, 1, 2, 3, 5, 6][k % 6];
            let s = c.applied_step[j];
            c.rx_queue.push_back((j, s));
        }
        c.rx_queue.push_back((4, old));
        c.rx_queue.push_back((4, old));
    }));
    let ack = r.start(json!({"label": "box", "speed_scale": 0.5})).await;
    let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
    let a = r.wait(id, 60.0).await;
    assert_eq!((a["state"].as_str(), a["code"].as_str()), (Some("failed"), Some("STALE_TELEMETRY")), "{a}");
    let c = ctrl.lock().unwrap();
    assert!(c.seqs[4] >= 3, "both old J5 replies were decoded after the fence: seq {}", c.seqs[4]);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn gray_trigger_timeout_resamples_sample_a() {
    let inner_arm = common::TestArm::new();
    inner_arm.0.lock().unwrap().q = START;
    let ctrl = Arc::new(Mutex::new(FeedbackControl::new(Instant::now())));
    let drv = QueuedFeedbackArm { inner: inner_arm, ctrl: ctrl.clone() };
    let mut r = rig_with_driver(ArmConfig::rebot_dm(), Box::new(drv)).await;

    // First gray trigger times out while J4 shifts +6 mrad (above the 2 mrad pose_refine=off
    // bracket limit); second trigger succeeds at the shifted pose. Keeping sample A from the
    // first attempt would fail MOVED or record the pre-timeout pose in bracket_rad[0].
    let shift = 0.006f32;
    r.vision.gray_trigger_timeouts.store(1, Ordering::SeqCst);
    let ctrl_cb = ctrl.clone();
    *r.vision.on_gray_trigger_timeout.lock().unwrap() = Some(Box::new(move || {
        ctrl_cb.lock().unwrap().applied_step[3] = shift;
    }));

    let ack = r.start(json!({"label": "box", "pose_refine": "off", "speed_scale": 0.5})).await;
    let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
    let a = r.wait(id, 120.0).await;
    assert_eq!(a["state"], "done", "{a}");
    let v0 = &a["measurement"]["views"][0];
    let a_j4 = v0["bracket_rad"][0][3].as_f64().unwrap();
    let b_j4 = v0["bracket_rad"][1][3].as_f64().unwrap();
    let expected = f64::from(START[3] + shift);
    assert!((a_j4 - expected).abs() < 1e-4, "expected second sample A ({expected}), got {a_j4}");
    assert!((b_j4 - expected).abs() < 1e-4, "expected sample B ({expected}), got {b_j4}");
    let spread_mrad = v0["snapshot_tries_mrad"][0].as_f64().unwrap();
    assert!(spread_mrad < 0.2, "expected near-zero spread on retaken bracket, got {spread_mrad}");
}

#[test]
fn malformed_gray_descriptor_does_not_select_gray() {
    let mode = |gray_frames: Option<serde_json::Value>| {
        let mut v = json!({"mode": "yolo", "per_request_pipelines": true, "available": {"yolo": true, "gray": true}});
        if let Some(g) = gray_frames { v["gray_frames"] = g; }
        armd::grasp::ModeInfo::parse(Some(&v))
    };
    let good = json!({"encoding": "RGF1", "format": "gray8", "width": 640, "height": 360, "stride": 640,
        "frame_size": [640, 640], "crop": [0, 140, 640, 360], "ring": 4});
    assert_eq!(mm::select_frame_source(&mode(Some(good.clone()))), mm::FrameSource::Gray);
    // Absent descriptor (older provider advertising available.gray only): gray with the fixed v1 contract.
    assert_eq!(mm::select_frame_source(&mode(None)), mm::FrameSource::Gray);
    let mut bad = good.clone();
    bad["crop"] = json!([0, 140, 640]);
    assert_eq!(mm::select_frame_source(&mode(Some(bad))), mm::FrameSource::Snapshot);
    let mut bad = good.clone();
    bad["width"] = json!("640");
    assert_eq!(mm::select_frame_source(&mode(Some(bad))), mm::FrameSource::Snapshot);
    assert_eq!(mm::select_frame_source(&mode(Some(json!("RGF1")))), mm::FrameSource::Snapshot);
    for wrong in [json!("false"), json!(0), json!(null)] {
        let mut bad = good.clone();
        bad["exposure_qualified"] = wrong;
        assert_eq!(mm::select_frame_source(&mode(Some(bad))), mm::FrameSource::Snapshot);
    }
    let mut qualified = good.clone();
    qualified["exposure_qualified"] = json!(true);
    assert_eq!(mm::select_frame_source(&mode(Some(qualified))), mm::FrameSource::Gray);
    let mut unsupported = good;
    unsupported["stride"] = json!(704);
    assert_eq!(mm::select_frame_source(&mode(Some(unsupported))), mm::FrameSource::Snapshot);
    let mut remote = mode(None);
    remote.upstream_loopback = false;
    assert_eq!(mm::select_frame_source(&remote), mm::FrameSource::Snapshot);
}

/// TestArm whose feedback looks fresh (age 0) but, once `frozen`, reports the same per-joint
/// sequence counters: the supervisor keeps re-reading a cached sample.
struct FrozenFeedbackArm { inner: common::TestArm, frozen: Arc<AtomicBool>, seq: u64 }

impl ArmDriver for FrozenFeedbackArm {
    fn capabilities(&self) -> DriverCapabilities { self.inner.capabilities() }
    fn enable_state(&self) -> EnableState { self.inner.enable_state() }
    fn is_feedback_valid(&self) -> bool { true }
    fn is_enabled(&self) -> bool { self.inner.is_enabled() }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { self.inner.feedback_ages() }
    fn feedback_seqs(&self) -> Vec<u64> { vec![self.seq; 7] }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        if !self.frozen.load(Ordering::SeqCst) { self.seq += 1; }
        self.inner.read_joint_states()
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> { self.inner.write_joint_targets(targets) }
    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> { self.inner.set_torque(enabled) }
    fn supports_mit_mode_switch(&self) -> bool { false }
    fn set_joint_modes(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> { self.inner.set_joint_modes(modes, hold) }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn cached_feedback_without_new_samples_never_counts_as_still() {
    // feedback within the freshness bound but no new sample (sequence counters stand still): the
    // start view's still window never fills, so FEEDBACK_SLOW (not NOT_SETTLED: nothing was seen
    // moving) without any trigger, with the per-joint update counts
    for gray in [false, true] {
        let frozen = Arc::new(AtomicBool::new(false));
        let d = FrozenFeedbackArm { inner: common::TestArm::new(), frozen: frozen.clone(), seq: 0 };
        let mut r = rig_with_driver(ArmConfig::rebot_dm(), Box::new(d)).await;
        r.vision.gray.store(gray, Ordering::SeqCst);
        frozen.store(true, Ordering::SeqCst);
        let ack = r.start(json!({"label": "box"})).await;
        let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
        let a = r.wait(id, 30.0).await;
        assert_eq!((a["state"].as_str(), a["code"].as_str()), (Some("failed"), Some("FEEDBACK_SLOW")), "gray {gray}: {a}");
        assert!(a["message"].as_str().unwrap().contains("0 of 10 samples") && a["message"].as_str().unwrap().contains("J1 0 (0"), "{a}");
        let s = &a["measurement"]["settle"];
        assert_eq!((s["code"].as_str(), s["window_samples"].as_u64(), s["joints"].as_array().map(Vec::len)), (Some("FEEDBACK_SLOW"), Some(0), Some(6)), "{a}");
        assert!(s["joints"].as_array().unwrap().iter().all(|j| j["updates"] == 0), "{a}");
        assert_eq!(r.vision.frame.load(Ordering::SeqCst), 0, "no trigger while the feedback is cached");
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_move_and_return_between_the_bracket_samples_fails_moved() {
    // J4 jumps 10 mrad for 80 ms at the start of every trigger (200 ms) and comes back before
    // sample B: the endpoints agree, the joints polled in between do not
    for gray in [false, true] {
        let creep = Arc::new(Mutex::new(Creep::default()));
        let d = CreepArm { inner: common::TestArm::new(), creep: creep.clone() };
        let mut r = rig_driven(ArmConfig::rebot_dm(), false, Some(d)).await;
        r.vision.gray.store(gray, Ordering::SeqCst);
        r.vision.trigger_delay_ms.store(200, Ordering::SeqCst);
        let c = creep.clone();
        *r.vision.on_trigger.lock().unwrap() = Some(Box::new(move || {
            c.lock().unwrap().bump = Some((Instant::now(), Duration::from_millis(80), 0.010));
        }));
        let ack = r.start(json!({"pose_refine": "off"})).await;
        let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("{ack}"));
        let a = r.wait(id, 60.0).await;
        assert_eq!((a["state"].as_str(), a["code"].as_str()), (Some("failed"), Some("MOVED")), "gray {gray}: {a}");
        let msg = a["message"].as_str().unwrap();
        assert!(msg.contains("J4") && msg.contains("limit 2.0"), "{msg}");
        assert_eq!(r.vision.frame.load(Ordering::SeqCst), mm::SNAPSHOT_TRIES as u64, "gray {gray}");
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn r2_m6_executor_and_legacy_multiview_publish_measurement_before_terminal() {
    // M6: end-to-end 3-view `measure_multiview` through `measure_multiview.rs:2637` on BOTH
    // Legacy and Executor owners: the delivered terminal observation (`TerminalBroadcaster`),
    // `sup.observation().await`, and `sup.get_observation().await` must ALL carry
    // `action.measurement` (height_m within 3 mm of HEIGHT, method == "motion_stereo")
    // BEFORE the terminal is delivered!

    // 1) Legacy owner
    {
        let mut r = rig(ArmConfig::rebot_dm()).await;
        r.sup.start_dispatcher().unwrap();
        let delivered = Arc::new(Mutex::new(Vec::<armd::ArmObservation>::new()));
        let delivered_sink = Arc::clone(&delivered);
        let _sub = r.sup.test_subscribe_action_terminals(move |obs| {
            delivered_sink.lock().unwrap().push(obs.clone());
            true
        });
        let ack = r.start(json!({"label": "box", "speed_scale": 0.5})).await;
        assert_eq!(ack["type"], "command_ack", "{ack}");
        let id = ack["action_id"].as_u64().unwrap();
        let a = r.wait(id, 120.0).await;
        assert_eq!((a["kind"].as_str(), a["state"].as_str()), (Some("measure_multiview"), Some("done")), "{a}");
        let deadline = Instant::now() + Duration::from_secs(5);
        let term_obs = loop {
            if let Some(obs) = delivered.lock().unwrap().iter().find(|o| o.action.as_ref().is_some_and(|a| a.id == id)).cloned() {
                break obs;
            }
            assert!(Instant::now() < deadline, "legacy terminal observation for {id} not delivered");
            tokio::time::sleep(Duration::from_millis(10)).await;
        };
        let term_m = term_obs
            .action
            .as_ref()
            .and_then(|a| a.measurement.as_ref())
            .expect("legacy terminal observation must carry action.measurement before delivery");
        assert_eq!(term_m["method"], "motion_stereo", "{term_m}");
        let h = term_m["objects"][0]["height_m"].as_f64().unwrap();
        assert!((h - HEIGHT).abs() < 0.003, "legacy height {h}");
        let obs = r.sup.observation().await.unwrap();
        assert_eq!(obs.action.as_ref().and_then(|a| a.measurement.as_ref()), Some(term_m));
    }

    // 2) Executor owner (live RT executor, detached shell, zero SupervisorLock!)
    {
        let mut config = ArmConfig::rebot_dm();
        config.lease_timeout_ms = 600_000;
        let (cam, _, look) = geometry(&config);
        let sup = Arc::new(ArmSupervisor::new(config));
        sup.set_lease_enabled(false).await;
        {
            let mut inner = sup.inner().write().await;
            inner.commanded_positions = START.to_vec();
            inner.measured_positions = START.to_vec();
            inner.target_positions = START.to_vec();
            let s = inner.current_admission_stamp();
            let a = inner.check_admission(s).unwrap();
            armd::SupervisorInner::enable_torque(a).unwrap();
            inner.tick(0.02);
        }
        sup.start_dispatcher().unwrap();
        let handle = sup
            .start_executor(armd::rt_core::executor::ExecutorConfig::for_tests(
                Duration::from_millis(20),
            ))
            .await
            .unwrap();
        // Attach PR L reply sink hook so begin_executor commits cleanly.
        let sink_terminals = Arc::new(Mutex::new(Vec::<(u64, armd::rt_core::broker::Outcome)>::new()));
        let sink_ref = Arc::clone(&sink_terminals);
        handle.attach_external_reply_sink(Arc::new(move |_cid, reply| {
            if let armd::rt_core::broker::Reply::Terminal { action, outcome, .. } = reply {
                sink_ref.lock().unwrap().push((action.0, outcome));
            }
            armd::rt_core::broker::DeliveryDecision::Delivered
        }));

        let delivered = Arc::new(Mutex::new(Vec::<armd::ArmObservation>::new()));
        let delivered_sink = Arc::clone(&delivered);
        let _sub = sup.test_subscribe_action_terminals(move |obs| {
            delivered_sink.lock().unwrap().push(obs.clone());
            true
        });

        let scene = Scene::new(0.0, vec![BoxObj { xy: [look[0], look[1]], yaw: 0.3, size: [0.05, 0.03, HEIGHT] }]);
        let vision = Arc::new(FakeVision {
            sup: sup.clone(), cam, scene: Arc::new(scene), hide_after_start: AtomicBool::new(false), frame: AtomicU64::new(0),
            snapshots: AtomicU64::new(0), snapshot_delay_ms: AtomicU64::new(0), trigger_delay_ms: AtomicU64::new(0), freeze_snapshot: AtomicBool::new(false),
            shake_on_snapshot: Mutex::new(None), shake_on_gray_trigger: Mutex::new(None), clock: Mutex::new(None), snapshot: Mutex::new(None), served: Mutex::new(Vec::new()),
            rot_err: Mutex::new(0.0), markers: Mutex::new(Vec::new()), aruco: AtomicBool::new(false),
            gray: AtomicBool::new(false), instance_id: Mutex::new("rv-exec-1".into()), gray_ring: Mutex::new(VecDeque::new()),
            gray_fetches: AtomicU64::new(0), gray_fetch_delay_ms: AtomicU64::new(0), gray_fail_queue: Mutex::new(VecDeque::new()),
            gray_fail_after_start: Mutex::new(None), on_gray_fetch: Mutex::new(None),
            gray_trigger_timeouts: AtomicU64::new(0), on_gray_trigger_timeout: Mutex::new(None),
            hold_supported: AtomicBool::new(true), holds: AtomicU64::new(0), releases: Mutex::new(Vec::new()), held: AtomicBool::new(false),
            hold_restart_after: AtomicU64::new(0), release_args: Mutex::new(Vec::new()), hold_clients: Mutex::new(Vec::new()), on_trigger: Mutex::new(None),
            contours: AtomicBool::new(false),
        });

        // Wait for first RT tick to publish so ActionSnapshot succeeds.
        let wait_tick = Instant::now() + Duration::from_secs(5);
        while sup.hub().published_seq() == 0 {
            assert!(Instant::now() < wait_tick, "RT executor never published");
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        let mut adm = sup.capture_admission_stamp();
        adm.connection_id = 77;
        let snap = armd::rt_core::requests::actions::ActionSnapshot::capture(
            &sup,
            adm,
            Instant::now() + armd::rt_core::requests::actions::DEFAULT_ACTION_DEADLINE,
        )
        .await
        .unwrap();
        let plan = mm::prepare_from_snapshot(
            &snap,
            &json!({"label": "box", "speed_scale": 0.5}),
            snap.owner.authority.epoch,
        )
        .unwrap();
        let (id, _) = mm::begin_executor(&sup, adm, plan, vision).await.unwrap();

        // Wait for the terminal observation delivered by `TerminalBroadcaster`!
        let deadline = Instant::now() + Duration::from_secs(120);
        let term_obs = loop {
            if let Some(obs) = delivered.lock().unwrap().iter().find(|o| o.action.as_ref().is_some_and(|a| a.id == id)).cloned() {
                break obs;
            }
            assert!(Instant::now() < deadline, "executor terminal observation for {id} not delivered");
            tokio::time::sleep(Duration::from_millis(20)).await;
        };
        let act = term_obs.action.as_ref().expect("terminal action");
        assert_eq!((act.kind.as_str(), act.state.as_str()), ("measure_multiview", "done"), "{act:?}");
        let term_m = act
            .measurement
            .as_ref()
            .expect("M6: executor terminal observation MUST carry action.measurement BEFORE delivery");
        assert_eq!(term_m["method"], "motion_stereo", "{term_m}");
        assert_eq!(term_m["views"].as_array().unwrap().len(), 3);
        let h = term_m["objects"][0]["height_m"].as_f64().unwrap();
        assert!((h - HEIGHT).abs() < 0.003, "executor height {h}");

        // `sup.observation()` and `sup.get_observation()` must ALSO publish the same measurement!
        let obs = sup.observation().await.unwrap();
        assert_eq!(obs.action.as_ref().and_then(|a| a.measurement.as_ref()), Some(term_m));
        let obs2 = sup.get_observation().await;
        assert_eq!(obs2.action.as_ref().and_then(|a| a.measurement.as_ref()), Some(term_m));

        handle.stop_and_join();
    }
}
