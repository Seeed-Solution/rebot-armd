//! Tap-to-grasp action (`/api/v1/action/grasp`) against a simulated camera.
//!
//! The simulated camera projects a box-shaped object at a fixed table position through the
//! current measured FK pose, its own camera mount and pinhole intrinsics. The controller
//! assumes different (rough) intrinsics and mount, as on the real arm.
mod common;

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::grasp::{self, BoxFuture, CamModel, DepthFrameInfo, Detection, DetectionFrame, DetectionSource, GraspDeps, GraspSetup, HeightSource, ModeInfo, VisionMode};
use armd::chain::ArmModel;
use armd::estimate::EstimateEntry;
use armd::kinematics::{self, Pose, Vec3};
use armd::motion::ActionStatus;
use armd::model::CameraMount;
use armd::{ArmConfig, ArmSupervisor};
use serde_json::{json, Value};

const DT: f64 = 0.02;
/// Motor angles of the cockpit inspect preset, J6 at its rest angle.
const INSPECT: [f64; 6] = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0];
/// The profile's arm model: motor angles -> FK / IK (B601-DM offsets).
const ARM: ArmModel = ArmModel::REBOT_DEV_ARM;
/// J7 where the jaws touch the test object (closed = 0, open = -5.7).
const OBJECT_J7: f32 = -1.5;

struct Object { centre: Vec3, half: [f64; 2], height: f64 }

#[derive(Clone, Debug)]
struct DepthSim {
    metric_model: bool,
    a0: f64,
    b0: f64,
    occluded_table: bool,
    occluded_above_z: Option<f64>,
    depth_conf: Option<f64>,
    jaw_clear: Option<bool>,
    length_px: Option<f64>,
    drift_per_frame: bool,
}

struct SimCamera {
    sup: Arc<ArmSupervisor>,
    cam: CamModel,
    object: Object,
    label: String,
    visible: AtomicBool,
    enabled: AtomicBool,
    frame: AtomicU64,
    inference_calls: Mutex<Vec<bool>>,
    /// rebot-vision mode API; None = an older detector without `/api/mode`.
    vision: Mutex<Option<VisionMode>>,
    mode_calls: Mutex<Vec<VisionMode>>,
    /// `/api/mode` rejects YOLO modes (no model loaded).
    no_yolo: AtomicBool,
    /// Perception API 2: triggers name their pipelines.
    per_request: AtomicBool,
    trigger_calls: Mutex<Vec<Vec<String>>>,
    next_frame_calls: AtomicU64,
    /// The object moved by this much (m, base x, y) since the rig was built.
    shift: Mutex<[f64; 2]>,
    /// The object is turned 90 deg about world z (long side along world y).
    turned: AtomicBool,
    /// Every frame request fails (detector error).
    failing: AtomicBool,
    /// Frames keep arriving but were captured 30 s ago (a stalled camera behind a live API).
    stale: AtomicBool,
    /// Detector latency: each result arrives 120..250 ms after its capture.
    delayed: AtomicBool,
    /// The detector reports the long side's image angle: (base-frame direction of the long
    /// side (rad), angle_conf).
    angle: Mutex<Option<(f64, f64)>>,
    /// Boxes of the object's top face only, at this height above the table (m), instead of the
    /// whole 0.05 m box.
    top: Mutex<Option<f64>>,
    /// Target grasp width reported by the detector (pixels).
    pub width_px: Mutex<Option<f64>>,
    /// Alternating noise (deg) added to `angle_deg` between frames.
    pub angle_noise: Mutex<Option<f64>>,
    pub focus_calls: Mutex<Vec<(Option<String>, [f64; 4], u64)>>,
    pub focus_error: AtomicBool,
    pub depth_available: AtomicBool,
    pub depth_sim: Mutex<Option<DepthSim>>,
    /// `/api/mode` `model.kind` and `depth_model.metric` (grasp tasks).
    pub model_kind: Mutex<Option<String>>,
    pub depth_metric: AtomicBool,
    /// The provider's mode cannot be read (connection refused).
    pub mode_down: AtomicBool,
    /// Mode readings (`mode_info`, one `GET /api/mode` on the HTTP source).
    pub mode_info_calls: AtomicU64,
    /// `models` of the mode (None: a provider that cannot switch models, POST /api/models 404).
    pub models: Mutex<Option<grasp::ProviderModels>>,
    /// `POST /api/models` bodies; the reply when set (else 202 and the switch is applied).
    pub set_models_calls: Mutex<Vec<Value>>,
    pub set_models_reply: Mutex<Option<(u16, Value)>>,
    /// Frames report this `model_generation` instead of the models' generation.
    pub frame_generation: Mutex<Option<u64>>,
}

impl SimCamera {
    /// YOLO boxes come out: inference on, and with the mode API a YOLO mode.
    fn yolo_running(&self) -> bool {
        match &*self.vision.lock().unwrap() {
            Some(v) => v.enabled && matches!(v.mode.as_str(), "yolo" | "both"),
            None => self.enabled.load(Ordering::SeqCst),
        }
    }
}

impl SimCamera {
    /// One frame through the measured pose.
    async fn frame_now(&self) -> DetectionFrame {
        tokio::time::sleep(Duration::from_millis(15)).await;
        let captured = Instant::now();
        let q = self.sup.inner().read().await.measured_positions.clone();
        let pose = ARM.fk(&kinematics::q6(&q));

        let (mut disp_fg, mut disp_bg, mut depth_conf, mut length_px, mut jaw_clear) = (None, None, None, None, None);
        let mut depth = None;
        if let Some(ref sim) = *self.depth_sim.lock().unwrap() {
            let (a_eff, b_eff) = if sim.drift_per_frame && pose.p[2] < 0.20 {
                (sim.a0 * 1.6, sim.b0 - 0.35)
            } else {
                (sim.a0, sim.b0)
            };
            let table_z = 0.0;
            let mut table_samples = Vec::new();
            if !sim.occluded_table && !sim.occluded_above_z.is_some_and(|z| pose.p[2] > z) {
                for u in [20.0, 160.0, 320.0, 480.0, 620.0] {
                    for v in [10.0, 90.0, 180.0, 270.0, 350.0] {
                        if let Some(z_table) = self.cam.table_optical_depth(&pose, [u, v], table_z) {
                            let disp = ((1.0 / z_table) - b_eff) / a_eff;
                            table_samples.push([u, v, disp]);
                        }
                    }
                }
            }
            depth = Some(DepthFrameInfo {
                metric_model: sim.metric_model,
                table_samples,
            });

            let o = &self.object;
            let sh = *self.shift.lock().unwrap();
            let top_h = self.top.lock().unwrap().unwrap_or(o.height);
            let p_top_world = [o.centre[0] + sh[0], o.centre[1] + sh[1], table_z + top_h];

            let p_top_cam = kinematics::mat_vec(&kinematics::transpose(&self.cam.rotation(&pose)), &kinematics::sub(&p_top_world, &self.cam.origin(&pose)));
            let z_fg = p_top_cam[2];
            let z_bg = self.cam.project(&pose, &p_top_world)
                .and_then(|px| self.cam.table_optical_depth(&pose, px, table_z))
                .unwrap_or(z_fg + top_h);

            if z_fg > 0.05 && z_bg > z_fg {
                let d_fg = ((1.0 / z_fg) - b_eff) / a_eff;
                let d_bg = ((1.0 / z_bg) - b_eff) / a_eff;
                disp_fg = Some(d_fg);
                disp_bg = Some(d_bg);
                depth_conf = sim.depth_conf.or(Some(0.95));
                jaw_clear = sim.jaw_clear.or(Some(true));
                length_px = sim.length_px.or(Some(60.0));
            }
        }

        let mut detections = vec![Detection { label: "person".into(), score: 0.5, raw_score_valid: true, box_norm: [0.0, 0.0, 0.08, 0.1], angle_deg: None, angle_conf: None, grasp_width_px: None, mask_focus: false, disp_fg: None, disp_bg: None, depth_conf: None, length_px: None, jaw_clear: None, depth_angle_deg: None, depth_angle_conf: None, depth_width_px: None, contour_norm: None }];
        if self.visible.load(Ordering::SeqCst) {
            if let Some(b) = self.project_box(&pose) {
                let (mut angle_deg, angle_conf) = match self.image_angle(&pose) { Some((a, c)) => (Some(a), Some(c)), None => (None, None) };
                if let (Some(a), Some(noise)) = (angle_deg, *self.angle_noise.lock().unwrap()) {
                    let frame_id = self.frame.load(Ordering::SeqCst);
                    let sign = if frame_id.is_multiple_of(2) { 1.0 } else { -1.0 };
                    angle_deg = Some(a + sign * noise);
                }
                let grasp_width_px = *self.width_px.lock().unwrap();
                detections.push(Detection { label: self.label.clone(), score: 0.9, raw_score_valid: true, box_norm: b, angle_deg, angle_conf, grasp_width_px, mask_focus: false, disp_fg, disp_bg, depth_conf, length_px, jaw_clear, depth_angle_deg: None, depth_angle_conf: None, depth_width_px: None, contour_norm: None });
            }
        }
        let frame_id = self.frame.fetch_add(1, Ordering::SeqCst) + 1;
        if self.delayed.load(Ordering::SeqCst) {
            tokio::time::sleep(Duration::from_millis(120 + (frame_id * 37) % 131)).await;
        }
        let captured_at = if self.stale.load(Ordering::SeqCst) { captured - Duration::from_secs(30) } else { captured };
        let model_generation = (*self.frame_generation.lock().unwrap()).or_else(|| self.models.lock().unwrap().as_ref().map(|m| m.generation));
        DetectionFrame { frame_id, captured_at, policy_captured_at: Some(captured_at), image_size_px: self.cam.size, detections, depth, model_generation }
    }

    /// Image angle (deg, [-90, 90)) of the object's long side when the rig reports one.
    fn image_angle(&self, pose: &Pose) -> Option<(f64, f64)> {
        let (dir, conf) = (*self.angle.lock().unwrap())?;
        let o = &self.object;
        let sh = *self.shift.lock().unwrap();
        let top = self.top.lock().unwrap().unwrap_or(o.height);
        let c = [o.centre[0] + sh[0], o.centre[1] + sh[1], o.centre[2] + top];
        let p0 = self.cam.project(pose, &c)?;
        let p1 = self.cam.project(pose, &[c[0] + 0.02 * dir.cos(), c[1] + 0.02 * dir.sin(), c[2]])?;
        let a = (p1[1] - p0[1]).atan2(p1[0] - p0[0]).to_degrees();
        Some(((a + 90.0).rem_euclid(180.0) - 90.0, conf))
    }

    /// The detector's box: [`Self::project_box_whole`] clamped to the image; None when less
    /// than 4 px of it is inside the image along either axis.
    fn project_box(&self, pose: &Pose) -> Option<[f64; 4]> {
        let [x0, y0, x1, y1] = self.project_box_whole(pose)?;
        let [w, h] = self.cam.size;
        if x1.min(w) - x0.max(0.0) < 4.0 || y1.min(h) - y0.max(0.0) < 4.0 { return None; }
        Some([(x0 / w).clamp(0.0, 1.0), (y0 / h).clamp(0.0, 1.0), (x1 / w).clamp(0.0, 1.0), (y1 / h).clamp(0.0, 1.0)])
    }

    /// The object's whole image box (px), also past the image border.
    fn project_box_whole(&self, pose: &Pose) -> Option<[f64; 4]> {
        let o = &self.object;
        let sh = *self.shift.lock().unwrap();
        let half = if self.turned.load(Ordering::SeqCst) { [o.half[1], o.half[0]] } else { o.half };
        let o = Object { centre: [o.centre[0] + sh[0], o.centre[1] + sh[1], o.centre[2]], half, height: o.height };
        let zs = match *self.top.lock().unwrap() { Some(t) => vec![o.centre[2] + t], None => vec![o.centre[2], o.centre[2] + o.height] };
        let (mut x0, mut y0, mut x1, mut y1) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        for sx in [-1.0, 1.0] { for sy in [-1.0, 1.0] { for &z in &zs {
            let p = self.cam.project(pose, &[o.centre[0] + sx * o.half[0], o.centre[1] + sy * o.half[1], z])?;
            x0 = x0.min(p[0]); y0 = y0.min(p[1]); x1 = x1.max(p[0]); y1 = y1.max(p[1]);
        }}}
        Some([x0, y0, x1, y1])
    }
}

impl DetectionSource for SimCamera {
    fn inference_enabled(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async move { Ok(self.enabled.load(Ordering::SeqCst)) })
    }
    fn set_inference(&self, enabled: bool) -> BoxFuture<'_, Result<(), String>> {
        Box::pin(async move {
            self.enabled.store(enabled, Ordering::SeqCst);
            self.inference_calls.lock().unwrap().push(enabled);
            Ok(())
        })
    }
    fn vision_mode(&self) -> BoxFuture<'_, Result<Option<VisionMode>, String>> {
        Box::pin(async move { Ok(self.vision.lock().unwrap().clone()) })
    }
    fn set_vision_mode(&self, m: &VisionMode) -> BoxFuture<'_, Result<(), String>> {
        let m = m.clone();
        Box::pin(async move {
            if self.no_yolo.load(Ordering::SeqCst) && matches!(m.mode.as_str(), "yolo" | "both") {
                return Err("detector returned 409 Conflict yolo unavailable".into());
            }
            self.mode_calls.lock().unwrap().push(m.clone());
            *self.vision.lock().unwrap() = Some(m);
            Ok(())
        })
    }
    fn next_frame(&self, _after: Option<u64>, wait_ms: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        Box::pin(async move {
            self.next_frame_calls.fetch_add(1, Ordering::SeqCst);
            if self.failing.load(Ordering::SeqCst) { return Err("detector returned 503".into()); }
            if !self.yolo_running() {
                tokio::time::sleep(Duration::from_millis(wait_ms)).await;
                return Ok(None);
            }
            Ok(Some(self.frame_now().await))
        })
    }
    fn per_request_pipelines(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async move {
            if self.mode_down.load(Ordering::SeqCst) { return Err("connection refused".into()); }
            Ok(self.per_request.load(Ordering::SeqCst))
        })
    }
    fn mode_info(&self) -> BoxFuture<'_, Result<ModeInfo, String>> {
        Box::pin(async move {
            self.mode_info_calls.fetch_add(1, Ordering::SeqCst);
            let mut m = ModeInfo::compose(self).await?;
            m.model_kind = self.model_kind.lock().unwrap().clone();
            m.yolo_available = !self.no_yolo.load(Ordering::SeqCst);
            m.depth_metric = self.depth_metric.load(Ordering::SeqCst);
            m.models = self.models.lock().unwrap().clone();
            Ok(m)
        })
    }
    fn set_models(&self, body: &Value) -> BoxFuture<'_, Result<(u16, Value), String>> {
        let body = body.clone();
        Box::pin(async move {
            self.set_models_calls.lock().unwrap().push(body.clone());
            if let Some(r) = self.set_models_reply.lock().unwrap().clone() { return Ok(r); }
            let mut models = self.models.lock().unwrap();
            let Some(m) = models.as_mut() else { return Ok((404, json!({"error": "not found"}))) };
            // applied at once (the swap's frame): the detector name is its kind
            if let Some(d) = body.get("detector") {
                m.resident.detector = d.as_str().map(str::to_string);
                *self.model_kind.lock().unwrap() = m.resident.detector.clone();
            }
            if let Some(d) = body.get("depth") {
                m.resident.depth = d.as_str().map(str::to_string);
                self.depth_available.store(m.resident.depth.is_some(), Ordering::SeqCst);
            }
            m.generation += 1;
            Ok((202, json!({"generation": m.generation})))
        })
    }
    fn depth_available(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async move { Ok(self.depth_available.load(Ordering::SeqCst)) })
    }
    fn set_mask_focus(&self, label: Option<String>, box_norm: [f64; 4], ttl_ms: u64) -> BoxFuture<'_, Result<(), String>> {
        if self.focus_error.load(Ordering::SeqCst) {
            return Box::pin(async { Err("simulated focus error".to_string()) });
        }
        self.focus_calls.lock().unwrap().push((label, box_norm, ttl_ms));
        Box::pin(async { Ok(()) })
    }
    fn trigger_with_focus(&self, pipelines: &'static [&'static str], wait_ms: u64, focus: Option<&armd::grasp::MaskFocus>) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        if let Some(f) = focus {
            if !self.focus_error.load(Ordering::SeqCst) {
                self.focus_calls.lock().unwrap().push((f.label.clone(), f.box_norm, f.ttl_ms));
            }
        }
        self.trigger(pipelines, wait_ms)
    }
    fn trigger(&self, pipelines: &'static [&'static str], _wait_ms: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        Box::pin(async move {
            self.trigger_calls.lock().unwrap().push(pipelines.iter().map(|p| p.to_string()).collect());
            if self.failing.load(Ordering::SeqCst) { return Err("detector returned 503".into()); }
            if self.no_yolo.load(Ordering::SeqCst) && pipelines.contains(&"yolo") {
                return Err("detector returned 409 Conflict yolo unavailable".into());
            }
            // runs whatever the global mode is: no continuous inference needed
            Ok(Some(self.frame_now().await))
        })
    }
}

struct Truth { tilt_deg: f64, rot_deg: f64, fx: f64, fy: f64, cx_off: f64, cy_off: f64, offset: Vec3, known_offset: bool }

impl Truth {
    fn same() -> Self { Self { tilt_deg: 0.0, rot_deg: 0.0, fx: 600.0, fy: 600.0, cx_off: 0.0, cy_off: 0.0, offset: [-0.06, 0.0, 0.04], known_offset: true } }
    fn cam(&self, setup: &GraspSetup) -> CamModel {
        CamModel {
            rec: kinematics::cam_mount(false, self.rot_deg, setup.camera_tilt_deg + self.tilt_deg),
            offset: self.offset, fx: self.fx, fy: self.fy,
            cx: setup.image_size_px[0] / 2.0 + self.cx_off, cy: setup.image_size_px[1] / 2.0 + self.cy_off,
            size: setup.image_size_px,
        }
    }
}

struct Rig { sup: Arc<ArmSupervisor>, sim: Arc<SimCamera>, object: Vec3, hold: Arc<AtomicBool> }

fn look_down_start(tilt_from_vertical_deg: f64, z: f64) -> ([f32; 7], Pose) {
    let r0 = grasp::look_down(&ARM.fk(&INSPECT).r);
    let p0 = ARM.fk(&INSPECT).p;
    let radial = kinematics::unit(&[p0[0], p0[1], 0.0]);
    let axis = [-radial[1], radial[0], 0.0];
    let r = kinematics::mat_mul(&kinematics::axis_r(&axis, tilt_from_vertical_deg.to_radians()), &r0);
    solve_start(Pose::new(r, [p0[0], p0[1], z]))
}

/// Start with the tool pitched `fraction` of the way from vertical to camera-down (mount `rec`).
fn pitched_start(rec: &kinematics::Mat3, fraction: f64, z: f64) -> ([f32; 7], Pose) {
    let r0 = grasp::look_down(&ARM.fk(&INSPECT).r);
    let w = kinematics::log_v(&kinematics::mul_t(&grasp::camera_down(&r0, rec), &r0));
    let r = kinematics::mat_mul(&kinematics::exp_r(&w.map(|v| v * fraction)), &r0);
    let p0 = ARM.fk(&INSPECT).p;
    solve_start(Pose::new(r, [p0[0], p0[1], z]))
}

fn solve_start(goal: Pose) -> ([f32; 7], Pose) {
    let lo = [-3.1, -2.05, -2.6, -3.1, -2.05, -6.2];
    let hi = [3.1, 2.05, 2.6, 3.1, 2.05, 6.2];
    let mut q = INSPECT;
    for _ in 0..20 {
        let sol = ARM.solve_dls(&goal, &q, &lo, &hi, kinematics::PAR.w);
        q = sol.q;
        if sol.ok { break; }
    }
    let pose = ARM.fk(&q);
    assert!(kinematics::norm(&kinematics::sub(&pose.p, &goal.p)) < 2e-3, "start pose reachable");
    (std::array::from_fn(|i| if i < 6 { q[i] as f32 } else { 0.0 }), pose)
}

/// Supervisor at a look-down start pose, a ticker at 10x real time, the simulated camera
/// and the grip pixel calibrated with the true model (object under the jaws at approach height).
async fn rig(setup: GraspSetup, truth: Truth, start_tilt_deg: f64, offset_xy: [f64; 2], lease: bool) -> Rig {
    rig_mounted(setup, truth, 0.0, look_down_start(start_tilt_deg, 0.24), offset_xy, lease).await
}

/// [`rig`] with the controller's `camera_mount.rot_deg` and a given start (q, pose).
async fn rig_mounted(setup: GraspSetup, truth: Truth, mount_rot_deg: f32, start: ([f32; 7], Pose), offset_xy: [f64; 2], lease: bool) -> Rig {
    rig_driven(setup, truth, mount_rot_deg, start, offset_xy, lease, None).await
}

/// [`rig_mounted`] with a driver (and the collision guard on) when `arm` is given.
async fn rig_driven(setup: GraspSetup, truth: Truth, mount_rot_deg: f32, start: ([f32; 7], Pose), offset_xy: [f64; 2], lease: bool,
                    arm: Option<common::TestArm>) -> Rig {
    let mut config = ArmConfig::rebot_dm();
    if arm.is_some() { config.collision = Some(armd::collision::CollisionPolicy { enabled: true, ..Default::default() }); }
    // the controller knows the optical centre only when the truth says so (`Truth::same`)
    let offset_m = truth.known_offset.then(|| truth.offset.map(|v| v as f32));
    config.camera_mount = CameraMount { rot_deg: mount_rot_deg, tilt_deg: setup.camera_tilt_deg as f32, offset_m, ..CameraMount::default() };
    let sup = Arc::new(ArmSupervisor::new(config));
    let (q, start) = start;
    {
        let mut inner = sup.inner().write().await;
        inner.lease_enabled = lease;
        if let Some(arm) = &arm {
            arm.0.lock().unwrap().q = q;
            inner.driver = Some(std::sync::Mutex::new(Box::new(arm.clone())));
        }
        { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| armd::SupervisorInner::enable_torque(__a)) }.unwrap();
        inner.commanded_positions = q.to_vec();
        inner.measured_positions = q.to_vec();
        inner.target_positions = q.to_vec();
        inner.set_grasp_setup(setup.clone());
    }
    let cam = truth.cam(&setup);
    // where the true camera's image centre meets the table
    let look = cam.plane_point(&start, [cam.cx, cam.cy], setup.table_z_m).expect("start view sees the table");
    let object = [look[0] + offset_xy[0], look[1] + offset_xy[1], setup.table_z_m];
    let sim = Arc::new(SimCamera {
        sup: sup.clone(), cam, object: Object { centre: object, half: [0.025, 0.015], height: 0.05 }, label: "cup".into(),
        visible: AtomicBool::new(true), enabled: AtomicBool::new(false), frame: AtomicU64::new(0), inference_calls: Mutex::new(Vec::new()),
        vision: Mutex::new(None), mode_calls: Mutex::new(Vec::new()), no_yolo: AtomicBool::new(false),
        per_request: AtomicBool::new(false), trigger_calls: Mutex::new(Vec::new()), next_frame_calls: AtomicU64::new(0),
        shift: Mutex::new([0.0; 2]), turned: AtomicBool::new(false), failing: AtomicBool::new(false), stale: AtomicBool::new(false), delayed: AtomicBool::new(false),
        angle: Mutex::new(None), top: Mutex::new(None), width_px: Mutex::new(None), angle_noise: Mutex::new(None),
        focus_calls: Mutex::new(Vec::new()), focus_error: AtomicBool::new(false),
        depth_available: AtomicBool::new(false), depth_sim: Mutex::new(None),
        model_kind: Mutex::new(None), depth_metric: AtomicBool::new(false), mode_down: AtomicBool::new(false), mode_info_calls: AtomicU64::new(0),
        models: Mutex::new(None), set_models_calls: Mutex::new(Vec::new()), set_models_reply: Mutex::new(None), frame_generation: Mutex::new(None),
    });
    let hold = Arc::new(AtomicBool::new(true));
    let (ticker, hold_t) = (sup.clone(), hold.clone());
    tokio::spawn(async move {
        loop {
            tokio::time::sleep(Duration::from_millis(2)).await;
            ticker.tick(DT).await;
            if hold_t.load(Ordering::SeqCst) {
                // the jaws stop on the object while closing
                let mut inner = ticker.inner().write().await;
                if inner.commanded_positions[6] > OBJECT_J7 && inner.measured_positions[6] >= OBJECT_J7 - 0.3 {
                    inner.measured_positions[6] = OBJECT_J7;
                    inner.measured_velocities[6] = 0.0;
                    inner.torques[6] = 0.3;
                } else { inner.torques[6] = 0.0; }
            }
        }
    });
    Rig { sup, sim, object, hold }
}

/// The grip pixel the user would record: the object's box centre with the object under the
/// jaw tips at approach height, at the approach orientation.
fn calibrate_grip(rig: &Rig, setup: &GraspSetup, start_r: &kinematics::Mat3) -> [f64; 2] {
    let r = grasp::look_down(start_r);
    let pose = Pose::new(r, [rig.object[0], rig.object[1], setup.table_z_m + setup.approach_height_m]);
    let b = rig.sim.project_box(&pose).expect("object visible under the jaws");
    grasp::box_centre(&b)
}

async fn start_pose(rig: &Rig) -> Pose { ARM.fk(&kinematics::q6(&rig.sup.inner().read().await.commanded_positions)) }

/// The box the user taps: the current simulated detection of the object.
async fn tap(rig: &Rig) -> Value {
    let pose = start_pose(rig).await;
    let b = rig.sim.project_box(&pose).expect("object visible at start");
    json!({"label": "cup", "box_norm": b, "frame_id": 1, "tag": "t"})
}

async fn wait_done(sup: &ArmSupervisor, id: u64, max_s: f64, mut each: impl FnMut(&armd::motion::ActionStatus)) -> armd::motion::ActionStatus {
    let deadline = Instant::now() + Duration::from_secs_f64(max_s);
    loop {
        assert!(Instant::now() < deadline, "grasp {id} still running: {:?}", sup.get_observation().await.action);
        let obs = sup.get_observation().await;
        let a = obs.action.clone().or(obs.previous_action.clone()).expect("action record");
        if a.id == id {
            each(&a);
            if !a.is_running() { return a; }
        } else if let Some(p) = obs.previous_action.filter(|p| p.id == id) { return p; }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

async fn wait_phase(sup: &ArmSupervisor, id: u64, phase: &str, needs_motion: Option<bool>) {
    let deadline = Instant::now() + Duration::from_secs(20);
    loop {
        assert!(Instant::now() < deadline, "phase {phase} not reached: {:?}", sup.get_observation().await.action);
        {
            let inner = sup.inner().read().await;
            let a = inner.last_action.as_ref().unwrap();
            let moving = inner.active_trajectory.as_ref().is_some_and(|t| t.action_id == Some(id));
            if a.id == id && a.phase.as_deref() == Some(phase) && needs_motion.is_none_or(|m| m == moving) { return; }
            assert!(a.is_running(), "grasp ended before {phase}: {a:?}");
        }
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
}

fn xy_err(p: &Vec3, o: &Vec3) -> f64 { (p[0] - o[0]).hypot(p[1] - o[1]) }

/// `oriented`: the detector reported an angle the grasp uses (conf >= `align_min_angle_conf`),
/// standing for an object turned off the world axes; its tests check the jaw direction themselves.
struct Outcome { status: armd::motion::ActionStatus, centred_err_m: f64, final_pose: Pose, turned: bool, oriented: bool }

async fn run_grasp(rig: &Rig) -> Outcome {
    let body = tap(rig).await;
    let (id, duration) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &body).await.expect("grasp accepted");
    assert!(duration > 5.0);
    let mut centred_err_m = f64::NAN;
    let sup = rig.sup.clone();
    let object = rig.object;
    let mut seen_align = false;
    let status = wait_done(&rig.sup, id, 40.0, |a| {
        if !seen_align && matches!(a.phase.as_deref(), Some("align" | "descend")) {
            seen_align = true;
            if let Ok(inner) = sup.inner().try_read() {
                centred_err_m = xy_err(&ARM.fk(&kinematics::q6(&inner.commanded_positions)).p, &object);
            }
        }
    }).await;
    let final_pose = start_pose(rig).await;
    if centred_err_m.is_nan() { centred_err_m = xy_err(&final_pose.p, &rig.object); }
    let min_conf = rig.sup.inner().read().await.grasp_setup().map_or(f64::INFINITY, |s| s.align_min_angle_conf);
    let oriented = rig.sim.angle.lock().unwrap().is_some_and(|(_, conf)| conf >= min_conf);
    Outcome { status, centred_err_m, final_pose, turned: rig.sim.turned.load(Ordering::SeqCst), oriented }
}

fn assert_holding_done(o: &Outcome, setup: &GraspSetup, object: &Vec3, max_err_m: f64) {
    let a = &o.status;
    assert_eq!((a.kind.as_str(), a.state.as_str()), ("grasp", "done"), "{a:?}");
    assert_eq!(a.holding, Some(true), "{a:?}");
    assert_eq!(a.phase.as_deref(), Some("done"));
    assert!(a.error_px.unwrap() <= setup.centre_tol_px as f32, "{a:?}");
    let err = xy_err(&o.final_pose.p, object);
    assert!(err <= max_err_m, "final TCP xy error {:.2} mm", err * 1000.0);
    assert!((o.final_pose.p[2] - (setup.table_z_m + setup.lift_height_m)).abs() < 2e-3, "lifted: z {}", o.final_pose.p[2]);
    assert!(kinematics::column(&o.final_pose.r, 0)[2] < -0.99, "tool axis vertical");
    if !o.oriented { assert_jaws_across_short_side(&o.final_pose, o.turned); }
}

/// The jaws (end_link y) close along the object's short side in the world: world y for the
/// object as placed (50 x 30 mm along x / y), world x when it is turned.
fn assert_jaws_across_short_side(pose: &Pose, turned: bool) {
    let jaw = kinematics::column(&pose.r, 1);
    let along = if turned { jaw[0] } else { jaw[1] };
    assert!(along.abs() > 0.99, "jaw axis {jaw:?} not across the short side (turned {turned})");
}

/// J6 alignment in world terms: with the object as placed and turned by 90 deg, the grasp
/// ends with the jaws across the short side; the align phase runs only for the orientation
/// where the start heading has them across the long side.
#[tokio::test]
async fn grasp_align_closes_the_jaws_across_the_short_side() {
    let setup = GraspSetup { camera_tilt_deg: -20.0, ..GraspSetup::default() };
    let mut aligned = Vec::new();
    for turned in [false, true] {
        let rig = rig(setup.clone(), Truth::same(), 0.0, [0.0, 0.0], false).await;
        rig.sim.turned.store(turned, Ordering::SeqCst);
        let start = start_pose(&rig).await;
        let start_jaw = kinematics::column(&start.r, 1);
        let mut s = setup.clone();
        s.grip_pixel_norm = calibrate_grip(&rig, &setup, &start.r);
        rig.sup.inner().write().await.set_grasp_setup(s.clone());
        let body = tap(&rig).await;
        let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &body).await.expect("grasp accepted");
        let mut saw_align = false;
        let status = wait_done(&rig.sup, id, 40.0, |a| { saw_align |= a.phase.as_deref() == Some("align") && a.is_running(); }).await;
        let final_pose = start_pose(&rig).await;
        let j6 = rig.sup.inner().read().await.commanded_positions[5];
        println!("turned {turned}: {} {:?}, start jaw {start_jaw:?}, final jaw {:?}, J6 motor {j6:.3}", status.state, status.code, kinematics::column(&final_pose.r, 1));
        assert_eq!((status.state.as_str(), status.holding), ("done", Some(true)), "{status:?}");
        assert_jaws_across_short_side(&final_pose, turned);
        aligned.push(saw_align);
    }
    assert_ne!(aligned[0], aligned[1], "exactly one orientation needs the J6 turn: {aligned:?}");
}

#[tokio::test]
async fn grasp_converges_from_offsets() {
    // controller model = truth; camera tilted 20 deg toward the jaws
    let setup = GraspSetup { camera_tilt_deg: -20.0, ..GraspSetup::default() };
    for offset in [[0.0, 0.0], [0.03, 0.02], [-0.025, 0.03], [0.02, -0.035]] {
        let rig = rig(setup.clone(), Truth::same(), 0.0, offset, false).await;
        let start_r = start_pose(&rig).await.r;
        let mut s = setup.clone();
        s.grip_pixel_norm = calibrate_grip(&rig, &setup, &start_r);
        rig.sup.inner().write().await.set_grasp_setup(s.clone());
        let o = run_grasp(&rig).await;
        println!("converge offset {:?}: {} {:?}, iterations {:?}, final pixel error {:.2} px, centred xy error {:.2} mm, final xy error {:.2} mm",
            offset, o.status.state, o.status.code, o.status.iteration, o.status.error_px.unwrap_or(f32::NAN), o.centred_err_m * 1000.0, xy_err(&o.final_pose.p, &rig.object) * 1000.0);
        assert_holding_done(&o, &s, &rig.object, 0.004);
        assert_eq!(*rig.sim.inference_calls.lock().unwrap(), vec![true, false], "inference enabled for the grasp, then restored");
    }
}

#[tokio::test]
async fn grasp_converges_with_wrong_intrinsics_and_mount() {
    // controller: fx = fy = 600, centred principal point, optical centre at the tip (unknown),
    // tilt -8 (toward the jaws); truth: fx 690 / fy 560, principal point off by (+14, -9) px,
    // tilt -11, rot 5 deg, camera 6 cm behind and 4 cm above the tip
    let setup = GraspSetup { camera_tilt_deg: -8.0, ..GraspSetup::default() };
    for (start_tilt, offset) in [(0.0, [0.02, 0.025]), (20.0, [-0.02, 0.0])] {
        let truth = Truth { tilt_deg: -3.0, rot_deg: 5.0, fx: 690.0, fy: 560.0, cx_off: 14.0, cy_off: -9.0, offset: [-0.06, 0.0, 0.04], known_offset: false };
        let rig = rig(setup.clone(), truth, start_tilt, offset, false).await;
        let start_r = start_pose(&rig).await.r;
        let mut s = setup.clone();
        s.grip_pixel_norm = calibrate_grip(&rig, &setup, &start_r);
        rig.sup.inner().write().await.set_grasp_setup(s.clone());
        let o = run_grasp(&rig).await;
        println!("wrong model start tilt {start_tilt} offset {:?}: {} {:?}, grip pixel {:?}, iterations {:?}, final pixel error {:.2} px, centred xy error {:.2} mm, final xy error {:.2} mm",
            offset, o.status.state, o.status.code, s.grip_pixel_norm, o.status.iteration, o.status.error_px.unwrap_or(f32::NAN), o.centred_err_m * 1000.0, xy_err(&o.final_pose.p, &rig.object) * 1000.0);
        assert_holding_done(&o, &s, &rig.object, 0.005);
        assert!(o.status.iteration.unwrap() >= 1, "closed loop corrected the coarse estimate");
    }
}

#[tokio::test]
async fn grasp_fails_target_lost_and_restores_inference() {
    let setup = GraspSetup::default();
    let rig = rig(setup, Truth::same(), 0.0, [0.01, 0.01], false).await;
    let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.unwrap();
    wait_phase(&rig.sup, id, "centring", None).await;
    rig.sim.visible.store(false, Ordering::SeqCst);
    let t = Instant::now();
    let a = wait_done(&rig.sup, id, 10.0, |_| {}).await;
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some("TARGET_LOST")), "{a:?}");
    assert!(t.elapsed() < Duration::from_secs(3), "lost within the timeout: {:?}", t.elapsed());
    assert!(!rig.sim.enabled.load(Ordering::SeqCst), "inference restored to off");
    assert_idle_after(&rig.sup, id).await;
}

#[tokio::test]
async fn grasp_fails_centring_when_iterations_run_out() {
    let setup = GraspSetup { max_iterations: 1, servo_gain: 0.2, ..GraspSetup::default() };
    let rig = rig(setup.clone(), Truth::same(), 0.0, [0.03, 0.03], false).await;
    let start_r = start_pose(&rig).await.r;
    let mut s = setup.clone();
    // grip pixel far from where the object lands after the coarse approach
    s.grip_pixel_norm = calibrate_grip(&rig, &setup, &start_r);
    s.grip_pixel_norm[0] = (s.grip_pixel_norm[0] + 0.15).min(1.0);
    rig.sup.inner().write().await.set_grasp_setup(s);
    let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.unwrap();
    let a = wait_done(&rig.sup, id, 20.0, |_| {}).await;
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some("CENTRING_FAILED")), "{a:?}");
    assert_eq!(a.iteration, Some(1));
}

#[tokio::test]
async fn grasp_not_holding_reopens_and_lifts() {
    let setup = GraspSetup::default();
    let rig = rig(setup.clone(), Truth::same(), 0.0, [0.0, 0.0], false).await;
    rig.hold.store(false, Ordering::SeqCst);
    let start_r = start_pose(&rig).await.r;
    let mut s = setup.clone();
    s.grip_pixel_norm = calibrate_grip(&rig, &setup, &start_r);
    rig.sup.inner().write().await.set_grasp_setup(s.clone());
    let o = run_grasp(&rig).await;
    let a = &o.status;
    assert_eq!((a.state.as_str(), a.code.as_deref(), a.holding), ("failed", Some("NOT_HOLDING"), Some(false)), "{a:?}");
    let inner = rig.sup.inner().read().await;
    let open = inner.config.gripper_policy.as_ref().unwrap().open_rad;
    assert!((inner.commanded_positions[6] - open).abs() < 0.3, "jaws reopened: {}", inner.commanded_positions[6]);
    assert!((o.final_pose.p[2] - (s.table_z_m + s.lift_height_m)).abs() < 2e-3, "lifted");
}

/// No grasp motion remains and the arm holds still.
async fn assert_idle_after(sup: &ArmSupervisor, id: u64) {
    tokio::time::sleep(Duration::from_millis(100)).await;
    let q0 = {
        let inner = sup.inner().read().await;
        assert!(inner.macro_run.is_none(), "grasp run cleared");
        assert!(!inner.active_trajectory.as_ref().is_some_and(|t| t.action_id == Some(id)), "no grasp motion left");
        inner.commanded_positions.clone()
    };
    tokio::time::sleep(Duration::from_millis(400)).await;
    let inner = sup.inner().read().await;
    if inner.active_trajectory.is_none() {
        let moved = q0.iter().zip(&inner.commanded_positions).map(|(a, b)| (a - b).abs()).fold(0.0f32, f32::max);
        assert!(moved < 1e-3, "orchestrator issued no motion after the end ({moved} rad)");
    }
}

async fn cancel_case(phase: &str, moving: Option<bool>, cancel: impl AsyncFnOnce(&Rig)) -> armd::motion::ActionStatus {
    let rig = rig(GraspSetup::default(), Truth::same(), 0.0, [0.02, 0.0], false).await;
    let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.unwrap();
    wait_phase(&rig.sup, id, phase, moving).await;
    let t = Instant::now();
    cancel(&rig).await;
    let a = wait_done(&rig.sup, id, 2.0, |_| {}).await;
    assert!(t.elapsed() < Duration::from_millis(500), "prompt: {:?}", t.elapsed());
    assert_idle_after(&rig.sup, id).await;
    tokio::time::sleep(Duration::from_millis(300)).await;
    assert!(!rig.sim.enabled.load(Ordering::SeqCst), "inference restored");
    a
}

#[tokio::test]
async fn grasp_stop_during_motion_and_between_motions() {
    let a = cancel_case("approach", Some(true), async |rig: &Rig| {
        let mut inner = rig.sup.inner().write().await;
        inner.stop_motion();
        inner.request_macro_stop();
    }).await;
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("cancelled", Some("STOPPED")), "{a:?}");
    let a = cancel_case("centring", Some(false), async |rig: &Rig| {
        let mut inner = rig.sup.inner().write().await;
        inner.stop_motion();
        inner.request_macro_stop();
    }).await;
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("cancelled", Some("STOPPED")), "{a:?}");
}

#[tokio::test]
async fn grasp_estop_ends_it() {
    let a = cancel_case("approach", Some(true), async |rig: &Rig| { rig.sup.trigger_e_stop().await; }).await;
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some("ESTOP_ACTIVE")), "{a:?}");
}

#[tokio::test]
async fn grasp_superseded_by_jog() {
    let a = cancel_case("centring", Some(false), async |rig: &Rig| {
        let mut inner = rig.sup.inner().write().await;
        let mut q = inner.commanded_positions.clone();
        q[0] += 0.02;
        { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::jog(__a, q, None, None)) } }.unwrap();
    }).await;
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("cancelled", Some("SUPERSEDED")), "{a:?}");
}

#[tokio::test]
async fn grasp_rejects_servo_frames_while_running() {
    let rig = rig(GraspSetup::default(), Truth::same(), 0.0, [0.0, 0.0], false).await;
    let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.unwrap();
    wait_phase(&rig.sup, id, "centring", Some(false)).await;
    let mut inner = rig.sup.inner().write().await;
    let q = inner.commanded_positions.clone();
    let epoch = inner.safety.epoch_id();
    assert_eq!({ let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).and_then(|__a| armd::SupervisorInner::servo(__a, q, epoch)) } }, Err(armd::SafetyError::MotionBusy));
}

// ---- WS: ack, lease loss, workspace rejection, setup ----

struct WsRig { rig: Rig, ws: common::TestSocket, generation: u64, epoch: u64, addr: std::net::SocketAddr }

async fn ws_rig(setup_path: Option<std::path::PathBuf>) -> WsRig {
    ws_rig_on(rig(GraspSetup::default(), Truth::same(), 0.0, [0.0, 0.0], true).await, setup_path).await
}

async fn ws_rig_on(rig: Rig, setup_path: Option<std::path::PathBuf>) -> WsRig {
    let deps = GraspDeps { source: rig.sim.clone(), setup_path };
    let router = armd::api::create_router_with(rig.sup.clone(), armd::api::proxy::DetectorUpstream::parse("http://127.0.0.1:9").unwrap(), deps, armd::OperatorAuth::disabled());
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });
    let (ws, generation) = common::controller(addr).await;
    let epoch = rig.sup.get_observation().await.epoch_id;
    WsRig { rig, ws, generation, epoch, addr }
}

async fn heartbeat_until_done(w: &mut WsRig, id: u64, max_s: f64) -> Value {
    let deadline = Instant::now() + Duration::from_secs_f64(max_s);
    loop {
        assert!(Instant::now() < deadline, "grasp not finished");
        common::send(&mut w.ws, json!({"type":"heartbeat", "generation":w.generation, "epoch_id":w.epoch})).await;
        common::drain(&mut w.ws, 5).await;
        let obs = w.rig.sup.get_observation().await;
        for a in [obs.action, obs.previous_action].into_iter().flatten() {
            if a.id == id && !a.is_running() { return serde_json::to_value(a).unwrap(); }
        }
        tokio::time::sleep(Duration::from_millis(30)).await;
    }
}

#[tokio::test]
async fn ws_grasp_acks_and_runs_to_done() {
    let mut w = ws_rig(None).await;
    let start_r = start_pose(&w.rig).await.r;
    let mut s = GraspSetup::default();
    s.grip_pixel_norm = calibrate_grip(&w.rig, &s, &start_r);
    let ack = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/grasp/setup", json!({"grip_pixel_norm": s.grip_pixel_norm})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    assert!(ack.get("persisted").is_none(), "no setup file configured");
    let body = tap(&w.rig).await;
    let ack = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", body).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let id = ack["action_id"].as_u64().unwrap();
    assert!(ack["expected_duration_s"].as_f64().unwrap() > 5.0);
    let a = heartbeat_until_done(&mut w, id, 40.0).await;
    assert_eq!((a["kind"].as_str(), a["state"].as_str(), a["holding"].as_bool(), a["tag"].as_str()), (Some("grasp"), Some("done"), Some(true), Some("t")), "{a}");
    let obs = w.rig.sup.get_observation().await;
    assert!((obs.commanded_positions[6] - OBJECT_J7).abs() < 0.3, "commanded J7 holds the grip: {}", obs.commanded_positions[6]);
}

#[tokio::test]
async fn ws_grasp_lease_loss_cancels() {
    let mut w = ws_rig(None).await;
    let ack = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", tap(&w.rig).await).await;
    let id = ack["action_id"].as_u64().expect("ack");
    wait_phase(&w.rig.sup, id, "approach", Some(true)).await;
    let t = Instant::now();
    let reply = common::ws_reply(&mut w.ws, json!({"type":"release_lease", "generation":w.generation}), &["lease_released"]).await;
    assert_eq!(reply["success"], true);
    let a = wait_done(&w.rig.sup, id, 2.0, |_| {}).await;
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("cancelled", Some("LEASE_LOST")), "{a:?}");
    assert!(t.elapsed() < Duration::from_millis(500));
    assert_idle_after(&w.rig.sup, id).await;
}

#[tokio::test]
async fn ws_grasp_stop_command_cancels() {
    let mut w = ws_rig(None).await;
    let ack = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", tap(&w.rig).await).await;
    let id = ack["action_id"].as_u64().expect("ack");
    wait_phase(&w.rig.sup, id, "centring", Some(false)).await;
    let ack = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/motion/stop", json!({})).await;
    assert_eq!(ack["type"], "command_ack");
    let a = wait_done(&w.rig.sup, id, 2.0, |_| {}).await;
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("cancelled", Some("STOPPED")), "{a:?}");
    assert_idle_after(&w.rig.sup, id).await;
}

#[tokio::test]
async fn ws_grasp_rejects_workspace_and_bad_requests() {
    let mut w = ws_rig(None).await;
    // approach height below the workspace floor
    let mut body = tap(&w.rig).await;
    body["overrides"] = json!({"table_z_m": -0.3, "approach_height_m": 0.1});
    let reply = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", body).await;
    assert_eq!((reply["type"].as_str(), reply["code"].as_str(), reply["status"].as_u64()), (Some("command_reject"), Some("WORKSPACE_VIOLATION"), Some(422)), "{reply}");
    assert!(w.rig.sup.get_observation().await.action.is_none(), "no action started");
    for bad in [json!({"label": "cup"}), json!({"label": "cup", "box_norm": [0.5, 0.5, 0.4, 0.6]}),
                json!({"label": "cup", "box_norm": [0.1, 0.1, 0.2, 0.2], "overrides": {"bogus": 1}})] {
        let reply = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", bad).await;
        assert_eq!((reply["code"].as_str(), reply["status"].as_u64()), (Some("INVALID_COMMAND"), Some(400)), "{reply}");
    }
}

#[tokio::test]
async fn grasp_setup_get_reports_camera_down_acquisition() {
    let mut w = ws_rig(None).await;
    let get = |addr| async move { reqwest::get(format!("http://{addr}/api/v1/grasp/setup")).await.unwrap().json::<Value>().await.unwrap() };
    let v = get(w.addr).await;
    assert_eq!((v["camera_down"].clone(), v.get("camera_down_error")), (json!(false), None), "auto, level mount: grip pixel {v}");
    for (patch, want) in [(json!({"camera_tilt_deg": -36.995}), true), (json!({"acquire_camera_down": "off"}), false), (json!({"acquire_camera_down": "auto", "camera_tilt_deg": 4.0}), false)] {
        let ack = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/grasp/setup", patch.clone()).await;
        assert_eq!(ack["type"], "command_ack", "{ack}");
        assert_eq!(get(w.addr).await["camera_down"], json!(want), "{patch}");
    }
    // "on" without a lens centre: grasps are refused, the GET says why
    let truth = Truth { known_offset: false, ..Truth::same() };
    let mut w = ws_rig_on(rig(GraspSetup::default(), truth, 0.0, [0.0, 0.0], true).await, None).await;
    assert_eq!(get(w.addr).await["camera_down"], json!(false), "auto without a lens centre");
    let ack = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/grasp/setup", json!({"acquire_camera_down": "on"})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let v = get(w.addr).await;
    assert!(v["camera_down"].is_null() && v["camera_down_error"].as_str().is_some_and(|e| e.contains("offset_m")), "{v}");
}

#[tokio::test]
async fn grasp_setup_persists_round_trip() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("sub").join("grasp_setup.json");
    let mut w = ws_rig(Some(path.clone())).await;
    // the capture needs closed jaws held still for 0.2 s: no simulated object between the
    // jaws (the ticker's hold opens them to OBJECT_J7), and the still window filled
    w.rig.hold.store(false, Ordering::SeqCst);
    let until = Instant::now() + Duration::from_secs(5);
    loop {
        let ready = {
            let inner = w.rig.sup.inner().read().await;
            inner.measured_positions[6].abs() < 1e-3 && inner.check_table_capture_stillness().is_ok()
        };
        if ready { break; }
        assert!(Instant::now() < until, "the arm did not settle with closed jaws");
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    let tcp_z = start_pose(&w.rig).await.p[2];
    let ack = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/grasp/setup",
        json!({"table_z_from_tcp": true, "grip_pixel_norm": [0.42, 0.71], "approach_height_m": 0.17, "camera_tilt_deg": 25.0})).await;
    assert_eq!((ack["type"].as_str(), ack["persisted"].as_bool()), (Some("command_ack"), Some(true)), "{ack}");
    assert!((ack["setup"]["table_z_m"].as_f64().unwrap() - tcp_z).abs() < 1e-3);
    let schema = w.rig.sup.get_schema().await;
    assert_eq!(schema.camera_mount.tilt_deg, 25.0, "tilt updates the camera mount");
    // REST read
    let v: Value = reqwest::get(format!("http://{}/api/v1/grasp/setup", w.addr)).await.unwrap().json().await.unwrap();
    assert_eq!(v["setup"]["grip_pixel_norm"], json!([0.42, 0.71]));
    // invalid update is rejected and leaves the file alone
    let reply = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/grasp/setup", json!({"grasp_height_m": -1.0})).await;
    assert_eq!(reply["code"], "INVALID_COMMAND", "{reply}");
    // a fresh supervisor loads the file
    let fresh = ArmSupervisor::new(ArmConfig::rebot_dm());
    grasp::load_into(&fresh, &path).await;
    let loaded = fresh.inner().read().await.grasp_setup().unwrap();
    assert_eq!(loaded.grip_pixel_norm, [0.42, 0.71]);
    assert_eq!((loaded.approach_height_m, loaded.camera_tilt_deg), (0.17, 25.0));
    assert!((loaded.table_z_m - tcp_z).abs() < 1e-3);
    // missing and invalid files keep the defaults
    let base = GraspSetup::default();
    assert_eq!(grasp::load_setup(&dir.path().join("none.json"), &base), (base.clone(), grasp::LoadOutcome::Missing));
    for text in ["{not json", "{\"grasp_height_m\": 5}", "{\"unknown\": 1}", "[1,2]"] {
        let bad = dir.path().join("bad.json");
        std::fs::write(&bad, text).unwrap();
        let (s, outcome) = grasp::load_setup(&bad, &base);
        assert_eq!(s, base);
        assert!(matches!(outcome, grasp::LoadOutcome::Invalid(_)), "{text}: {outcome:?}");
    }
    // a partial file overrides only its fields
    let part = dir.path().join("part.json");
    std::fs::write(&part, "{\"lift_height_m\": 0.2}").unwrap();
    let (s, outcome) = grasp::load_setup(&part, &base);
    assert_eq!((s.lift_height_m, s.approach_height_m, outcome), (0.2, base.approach_height_m, grasp::LoadOutcome::Loaded));
}

fn vm(mode: &str, enabled: bool) -> VisionMode { VisionMode { mode: mode.into(), enabled } }

#[test]
fn grasp_mode_adds_yolo_and_keeps_aruco() {
    assert_eq!(grasp::grasp_mode("off"), "yolo");
    assert_eq!(grasp::grasp_mode("yolo"), "yolo");
    assert_eq!(grasp::grasp_mode("aruco"), "both");
    assert_eq!(grasp::grasp_mode("both"), "both");
}

#[tokio::test]
async fn grasp_ensures_vision_mode_and_restores_it() {
    let setup = GraspSetup { camera_tilt_deg: -20.0, ..GraspSetup::default() };
    for (before, during) in [(vm("aruco", false), vm("both", true)), (vm("off", false), vm("yolo", true)), (vm("yolo", false), vm("yolo", true))] {
        let rig = rig(setup.clone(), Truth::same(), 0.0, [0.0, 0.0], false).await;
        let start_r = start_pose(&rig).await.r;
        let mut s = setup.clone();
        s.grip_pixel_norm = calibrate_grip(&rig, &setup, &start_r);
        rig.sup.inner().write().await.set_grasp_setup(s.clone());
        *rig.sim.vision.lock().unwrap() = Some(before.clone());
        let o = run_grasp(&rig).await;
        assert_holding_done(&o, &s, &rig.object, 0.004);
        assert_eq!(*rig.sim.mode_calls.lock().unwrap(), vec![during, before.clone()], "mode for the grasp, then the previous one");
        assert_eq!(rig.sim.vision.lock().unwrap().clone(), Some(before));
        assert!(rig.sim.inference_calls.lock().unwrap().is_empty(), "mode API used instead of the inference toggle");
    }
}

#[tokio::test]
async fn grasp_leaves_a_running_yolo_mode_alone_and_restores_after_failure() {
    let setup = GraspSetup::default();
    // already yolo with inference on: no mode request at all
    let rig1 = rig(setup.clone(), Truth::same(), 0.0, [0.01, 0.01], false).await;
    *rig1.sim.vision.lock().unwrap() = Some(vm("both", true));
    let (id, _) = grasp::start(&rig1.sup, rig1.sup.capture_admission_stamp(), rig1.sim.clone(), &tap(&rig1).await).await.unwrap();
    wait_phase(&rig1.sup, id, "centring", None).await;
    rig1.sim.visible.store(false, Ordering::SeqCst);
    let a = wait_done(&rig1.sup, id, 10.0, |_| {}).await;
    assert_eq!(a.code.as_deref(), Some("TARGET_LOST"));
    assert!(rig1.sim.mode_calls.lock().unwrap().is_empty());

    // aruco base mode: restored after a failure as well
    let rig2 = rig(setup.clone(), Truth::same(), 0.0, [0.01, 0.01], false).await;
    *rig2.sim.vision.lock().unwrap() = Some(vm("aruco", true));
    let (id, _) = grasp::start(&rig2.sup, rig2.sup.capture_admission_stamp(), rig2.sim.clone(), &tap(&rig2).await).await.unwrap();
    wait_phase(&rig2.sup, id, "centring", None).await;
    assert_eq!(rig2.sim.vision.lock().unwrap().clone(), Some(vm("both", true)));
    rig2.sim.visible.store(false, Ordering::SeqCst);
    let a = wait_done(&rig2.sup, id, 10.0, |_| {}).await;
    assert_eq!(a.code.as_deref(), Some("TARGET_LOST"));
    assert_eq!(rig2.sim.vision.lock().unwrap().clone(), Some(vm("aruco", true)));
    assert_idle_after(&rig2.sup, id).await;
}

#[tokio::test]
async fn grasp_without_yolo_model_fails_detection_unavailable() {
    let rig = rig(GraspSetup::default(), Truth::same(), 0.0, [0.0, 0.0], false).await;
    *rig.sim.vision.lock().unwrap() = Some(vm("aruco", false));
    rig.sim.no_yolo.store(true, Ordering::SeqCst);
    let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.unwrap();
    let a = wait_done(&rig.sup, id, 10.0, |_| {}).await;
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some("DETECTION_UNAVAILABLE")), "{a:?}");
    assert_eq!(rig.sim.vision.lock().unwrap().clone(), Some(vm("aruco", false)), "mode untouched");
    assert_idle_after(&rig.sup, id).await;
}

/// Perception API 2: a trigger with ["yolo"] per centring step; the global mode (here the
/// cockpit's ArUco, standby) is neither changed nor restored, and no continuous results are read.
#[tokio::test]
async fn grasp_triggers_yolo_per_step_without_touching_the_mode() {
    let setup = GraspSetup { camera_tilt_deg: -20.0, ..GraspSetup::default() };
    for before in [vm("aruco", true), vm("off", false), vm("yolo", false)] {
        let rig = rig(setup.clone(), Truth::same(), 0.0, [0.0, 0.0], false).await;
        let start_r = start_pose(&rig).await.r;
        let mut s = setup.clone();
        s.grip_pixel_norm = calibrate_grip(&rig, &setup, &start_r);
        rig.sup.inner().write().await.set_grasp_setup(s.clone());
        *rig.sim.vision.lock().unwrap() = Some(before.clone());
        rig.sim.per_request.store(true, Ordering::SeqCst);
        let o = run_grasp(&rig).await;
        assert_holding_done(&o, &s, &rig.object, 0.004);
        let calls = rig.sim.trigger_calls.lock().unwrap().clone();
        assert!(!calls.is_empty() && calls.iter().all(|p| p == &["yolo".to_string()]), "{calls:?}");
        assert!(calls.len() as u32 >= o.status.iteration.unwrap(), "one trigger per centring step at least");
        assert!(rig.sim.mode_calls.lock().unwrap().is_empty(), "no mode switch");
        assert!(rig.sim.inference_calls.lock().unwrap().is_empty(), "no inference toggle");
        assert_eq!(rig.sim.next_frame_calls.load(Ordering::SeqCst), 0, "no continuous results read");
        assert_eq!(rig.sim.vision.lock().unwrap().clone(), Some(before));
    }
}

#[tokio::test]
async fn grasp_triggers_fail_target_lost_and_without_model() {
    let rig1 = rig(GraspSetup::default(), Truth::same(), 0.0, [0.01, 0.01], false).await;
    rig1.sim.per_request.store(true, Ordering::SeqCst);
    *rig1.sim.vision.lock().unwrap() = Some(vm("aruco", true));
    let (id, _) = grasp::start(&rig1.sup, rig1.sup.capture_admission_stamp(), rig1.sim.clone(), &tap(&rig1).await).await.unwrap();
    wait_phase(&rig1.sup, id, "centring", None).await;
    rig1.sim.visible.store(false, Ordering::SeqCst);
    let a = wait_done(&rig1.sup, id, 10.0, |_| {}).await;
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some("TARGET_LOST")), "{a:?}");
    assert!(rig1.sim.mode_calls.lock().unwrap().is_empty());
    assert_eq!(rig1.sim.vision.lock().unwrap().clone(), Some(vm("aruco", true)));
    assert_idle_after(&rig1.sup, id).await;

    let rig2 = rig(GraspSetup::default(), Truth::same(), 0.0, [0.0, 0.0], false).await;
    rig2.sim.per_request.store(true, Ordering::SeqCst);
    rig2.sim.no_yolo.store(true, Ordering::SeqCst);
    *rig2.sim.vision.lock().unwrap() = Some(vm("aruco", false));
    let (id, _) = grasp::start(&rig2.sup, rig2.sup.capture_admission_stamp(), rig2.sim.clone(), &tap(&rig2).await).await.unwrap();
    let a = wait_done(&rig2.sup, id, 10.0, |_| {}).await;
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some("DETECTION_UNAVAILABLE")), "{a:?}");
    assert!(rig2.sim.mode_calls.lock().unwrap().is_empty());
    assert_idle_after(&rig2.sup, id).await;
}

/// Camera tilt is workspace geometry: it cannot change while a macro runs (so a grasp's own
/// plans are not made stale by a setup update), other setup fields can; idle, it can.
#[test]
fn camera_tilt_change_is_busy_while_a_macro_runs() {
    let mut inner = common::sim_at(ArmConfig::rebot_dm(), &common::INSPECT);
    inner.set_grasp_setup(GraspSetup::default());
    let input = inner.plan_input().unwrap();
    { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::begin_macro(__a, "grasp", &input, None)) } }.unwrap();
    let err = grasp::update_setup(&mut inner, &json!({"camera_tilt_deg": 25.0})).unwrap_err();
    assert_eq!(armd::api::rest::ApiError::from(err).code.as_deref(), Some("MOTION_BUSY"));
    assert_eq!(inner.config.camera_mount.tilt_deg, 0.0);
    grasp::update_setup(&mut inner, &json!({"table_z_m": 0.01})).expect("non-geometry field");
    assert_eq!(inner.workspace_revision, input.workspace_revision, "the running grasp's plans stay fresh");
    let mut inner = common::sim_at(ArmConfig::rebot_dm(), &common::INSPECT);
    inner.set_grasp_setup(GraspSetup::default());
    grasp::update_setup(&mut inner, &json!({"camera_tilt_deg": 25.0})).expect("idle");
    assert_eq!(inner.config.camera_mount.tilt_deg, 25.0);
}

// ---- camera-down acquisition (v8 holder: rot 0, tilt -38) ----

const V8_OFFSET: Vec3 = [-0.0731, 0.0005, 0.066];

/// v8 setup, truth = controller, and a start with the tool pitched half way to camera-down.
fn v8_setup() -> (GraspSetup, Truth, ([f32; 7], Pose)) {
    let setup = GraspSetup { camera_tilt_deg: -38.0, ..GraspSetup::default() };
    let truth = Truth { rot_deg: 0.0, offset: V8_OFFSET, ..Truth::same() };
    (setup, truth, pitched_start(&kinematics::cam_mount(false, 0.0, -38.0), 0.5, 0.22))
}

/// Camera-down grasps with the v8 mount: approach and centre with the optical axis down, then
/// the transition to the tool-vertical pose above the target, align, descend, close, lift.
#[tokio::test]
async fn camera_down_grasp_with_the_v8_mount() {
    let setup = GraspSetup { camera_tilt_deg: -38.0, ..GraspSetup::default() };
    // controller = truth, then truth off by 2 deg tilt, 1 deg rot and fx / fy 5 %
    let cases = [
        ("same", Truth { tilt_deg: 0.0, rot_deg: 0.0, fx: 600.0, fy: 600.0, cx_off: 0.0, cy_off: 0.0, offset: V8_OFFSET, known_offset: true }, [0.0, 0.0], 0.004),
        ("same, offset", Truth { tilt_deg: 0.0, rot_deg: 0.0, fx: 600.0, fy: 600.0, cx_off: 0.0, cy_off: 0.0, offset: V8_OFFSET, known_offset: true }, [0.03, -0.02], 0.004),
        ("perturbed", Truth { tilt_deg: -2.0, rot_deg: 1.0, fx: 630.0, fy: 570.0, cx_off: 6.0, cy_off: -4.0, offset: V8_OFFSET, known_offset: true }, [-0.02, 0.025], 0.012),
    ];
    for (name, truth, offset, max_err) in cases {
        // the operator has the tool pitched half way, so the table below is in view
        let rig = rig_mounted(setup.clone(), truth, 0.0, v8_setup().2, offset, false).await;
        let body = tap(&rig).await;
        let (id, duration) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &body).await.expect("grasp accepted");
        let mut phases: Vec<String> = Vec::new();
        let mut down_err = f64::NAN;
        let sup = rig.sup.clone();
        let cam = rig.sim.cam.clone();
        let status = wait_done(&rig.sup, id, 60.0, |a| {
            let p = a.phase.as_deref().unwrap_or_default().to_string();
            if phases.last() != Some(&p) {
                // optical axis at the end of the centring (first look at the transition phase)
                if p == "transition" {
                    if let Ok(inner) = sup.inner().try_read() {
                        let pose = ARM.fk(&kinematics::q6(&inner.commanded_positions));
                        down_err = (kinematics::column(&cam.rotation(&pose), 2)[2] + 1.0).abs();
                    }
                }
                phases.push(p);
            }
        }).await;
        let final_pose = start_pose(&rig).await;
        let err = xy_err(&final_pose.p, &rig.object);
        println!("v8 {name} offset {offset:?}: {} {:?}, iterations {:?}, error {:.2} px, final offset {:?} px, final xy error {:.2} mm, \
                  expected {duration:.1} s, phases {phases:?}",
            status.state, status.code, status.iteration, status.error_px.unwrap_or(f32::NAN), status.final_offset_px, err * 1000.0);
        let o = Outcome { status, centred_err_m: f64::NAN, final_pose, turned: false, oriented: false };
        assert_holding_done(&o, &setup, &rig.object, max_err);
        // align runs only when the jaws would close across the long side (assert_holding_done
        // checks the final jaw direction)
        let order: Vec<&str> = ["approach", "centring", "transition", "descend", "close", "lift", "done"].into_iter()
            .filter(|p| phases.iter().any(|q| q == p)).collect();
        assert_eq!(order, ["approach", "centring", "transition", "descend", "close", "lift", "done"], "{phases:?}");
        let pos = |p: &str| phases.iter().position(|q| q == p).unwrap();
        assert!(pos("centring") < pos("transition") && pos("transition") < pos("descend"), "{phases:?}");
        if name.starts_with("same") { assert!(down_err < 1e-3, "optical axis vertical while centring: {down_err}"); }
    }
}

/// At a low approach height the target under the jaws is in view after the transition: the
/// re-check records its offset from the (calibrated) grip pixel; motion is unchanged.
#[tokio::test]
async fn camera_down_recheck_records_the_final_offset() {
    let (rig, s) = v8_recheck_rig().await;
    let o = run_grasp(&rig).await;
    println!("v8 re-check at 0.06 m: {} {:?}, grip pixel {:?}, final offset {:?} px, final xy error {:.2} mm",
        o.status.state, o.status.code, s.grip_pixel_norm, o.status.final_offset_px, xy_err(&o.final_pose.p, &rig.object) * 1000.0);
    assert_holding_done(&o, &s, &rig.object, 0.004);
    let off = o.status.final_offset_px.expect("re-check measured");
    assert!(off[0].hypot(off[1]) < 15.0, "{off:?}");
    let v = serde_json::to_value(&o.status).unwrap();
    assert!(v["final_offset_px"].is_array(), "{v}");
}

/// `acquire_camera_down: "on"` without a lens centre is rejected before anything moves.
#[tokio::test]
async fn camera_down_on_without_lens_centre_is_rejected() {
    let setup = GraspSetup { camera_tilt_deg: -20.0, ..GraspSetup::default() };
    let truth = Truth { known_offset: false, ..Truth::same() };
    let rig = rig(setup, truth, 0.0, [0.0, 0.0], false).await;
    let mut body = tap(&rig).await;
    body["overrides"] = json!({"acquire_camera_down": "on"});
    let err = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &body).await.unwrap_err();
    assert_eq!(err.code.as_deref(), Some("INVALID_COMMAND"), "{err:?}");
    assert!(rig.sup.get_observation().await.action.is_none(), "no action started");
    body["overrides"] = json!({"acquire_camera_down": "sideways"});
    assert_eq!(grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &body).await.unwrap_err().code.as_deref(), Some("INVALID_COMMAND"));
}

/// Lowest base-frame z of the TCP, the workspace check points and the lens centre at `q`.
fn envelope_min_z(config: &ArmConfig, q: &[f32]) -> f64 {
    let pose = ARM.fk(&kinematics::q6(q));
    let mut pts: Vec<Vec3> = vec![[0.0; 3]];
    pts.extend(config.workspace.iter().flat_map(|w| w.check_points_end_m.iter().map(|p| p.map(f64::from))));
    pts.extend(config.camera_mount.offset_m.map(|o| o.map(f64::from)));
    pts.iter().map(|p| pose.transform_point(p)[2]).fold(f64::INFINITY, f64::min)
}

/// Raised table and a low approach: the pitched gripper envelope would reach below the table
/// at the requested approach height. The camera-down acquisition raises the TCP so every check
/// point stays above table_z_m + clearance (reported as `acquire_height_m`) through the
/// approach, centring and transition.
#[tokio::test]
async fn camera_down_keeps_the_pitched_envelope_above_a_raised_table() {
    // height probe off: it measures the 5 cm object's top at 0.050 m and fails OBJECT_TOO_TALL
    // against the 0.035 m approach (height_probe_* tests); this test checks the envelope only
    let setup = GraspSetup { camera_tilt_deg: -38.0, table_z_m: 0.10, grasp_height_m: 0.015, approach_height_m: 0.035, height_probe_m: 0.0, ..GraspSetup::default() };
    // rot 90: the tilt pitches the tool about end_link z, the axis along which the check points
    // are 92.5 mm off the tool axis (v8's rot 0 pitches about the jaw axis, end_link y, where
    // they are 34.7 mm off)
    let truth = Truth { rot_deg: 90.0, offset: V8_OFFSET, ..Truth::same() };
    let start = pitched_start(&kinematics::cam_mount(false, 90.0, -38.0), 0.5, 0.30);
    let rig = rig_mounted(setup.clone(), truth, 90.0, start, [0.0, 0.0], false).await;
    let config = rig.sup.inner().read().await.config.clone();
    let body = tap(&rig).await;
    let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &body).await.expect("grasp accepted");
    let sup = rig.sup.clone();
    let mut lowest = f64::INFINITY;
    let mut at = String::new();
    let a = wait_done(&rig.sup, id, 60.0, |a| {
        if let Ok(inner) = sup.inner().try_read() {
            let z = envelope_min_z(&config, &inner.commanded_positions);
            if z < lowest { lowest = z; at = a.phase.as_deref().unwrap_or_default().to_string(); }
        }
    }).await;
    println!("raised table: {} {:?} acquire_height_m {:?}, lowest envelope point {:.4} m (table {:.3}) in phase {at}",
        a.state, a.code, a.acquire_height_m, lowest, setup.table_z_m);
    assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{a:?}");
    let h = a.acquire_height_m.expect("acquisition height raised and reported");
    assert!(f64::from(h) > setup.approach_height_m + 0.01, "{h}");
    assert!(lowest >= setup.table_z_m + 0.004, "envelope {lowest:.4} m reached the table {:.3} m in {at}", setup.table_z_m);
}

/// v8 rig at a 0.06 m approach (the target under the jaws is in view after the transition),
/// grip pixel calibrated.
async fn v8_recheck_rig() -> (Rig, GraspSetup) {
    let (setup, truth, start) = v8_setup();
    // probe off: the 0.05 m test object is within 0.02 m of this approach height, the probe
    // would end the grasp OBJECT_TOO_TALL before the transition these tests look at
    let setup = GraspSetup { approach_height_m: 0.06, height_probe_m: 0.0, ..setup };
    let start_r = start.1.r;
    let rig = rig_mounted(setup.clone(), truth, 0.0, start, [0.01, 0.0], false).await;
    // long side along the camera's offset direction: the box centre then stays within
    // recheck_tol_px of the projected target at this 0.06 m approach (turned the other way
    // the 3D box's image centre is ~54 px from it)
    rig.sim.turned.store(true, Ordering::SeqCst);
    let mut s = setup.clone();
    s.grip_pixel_norm = calibrate_grip(&rig, &setup, &start_r);
    rig.sup.inner().write().await.set_grasp_setup(s.clone());
    (rig, s)
}

/// After the transition the model expects the target in view: losing it, finding it moved, or
/// a detector error ends the grasp with that code before the descent.
#[tokio::test]
async fn camera_down_recheck_failures_end_the_grasp_before_descent() {
    for (case, expect) in [("lost", "TARGET_LOST"), ("displaced", "TARGET_DISPLACED"), ("detector", "DETECTION_UNAVAILABLE")] {
        let (rig, s) = v8_recheck_rig().await;
        let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.unwrap();
        wait_phase(&rig.sup, id, "transition", Some(true)).await;
        match case {
            "lost" => rig.sim.visible.store(false, Ordering::SeqCst),
            "displaced" => *rig.sim.shift.lock().unwrap() = [0.0, 0.02],
            _ => rig.sim.failing.store(true, Ordering::SeqCst),
        }
        let mut phases = Vec::new();
        let a = wait_done(&rig.sup, id, 20.0, |a| {
            let p = a.phase.unwrap_or_default();
            if phases.last() != Some(&p) { phases.push(p); }
        }).await;
        let z = start_pose(&rig).await.p[2];
        println!("re-check {case}: {} {:?} {:?}, final offset {:?}, phases {phases:?}, TCP z {z:.3}", a.state, a.code, a.message, a.final_offset_px);
        assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some(expect)), "{case}: {a:?}");
        assert!(!phases.iter().any(|p| p == "descend" || p == "close"), "{case}: {phases:?}");
        assert!((z - (s.table_z_m + s.approach_height_m)).abs() < 2e-3, "{case}: still at approach height, z {z}");
        assert_idle_after(&rig.sup, id).await;
    }
}

/// Frames that keep arriving but predate the settled motion count against the observation
/// deadline: TARGET_LOST after about `lost_timeout_s`, in the centring loop and in the
/// camera-down re-check, instead of waiting for GRASP_TIMEOUT.
#[tokio::test]
async fn stale_frames_end_the_observation_as_target_lost() {
    let rig1 = rig(GraspSetup::default(), Truth::same(), 0.0, [0.01, 0.01], false).await;
    let (rig2, _) = v8_recheck_rig().await;
    for (case, rig, phase) in [("centring", &rig1, "centring"), ("re-check", &rig2, "transition")] {
        let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(rig).await).await.unwrap();
        wait_phase(&rig.sup, id, phase, Some(case == "re-check")).await;
        rig.sim.stale.store(true, Ordering::SeqCst);
        let t = Instant::now();
        let a = wait_done(&rig.sup, id, 15.0, |_| {}).await;
        println!("stale frames in {case}: {} {:?} {:?} after {:.2} s", a.state, a.code, a.message, t.elapsed().as_secs_f64());
        assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some("TARGET_LOST")), "{case}: {a:?}");
        assert!(t.elapsed() < Duration::from_secs(4), "{case}: {:?}", t.elapsed());
        assert_idle_after(&rig.sup, id).await;
    }
}

// ---- ending a camera-down grasp during the transition ----

#[tokio::test]
async fn camera_down_stop_during_the_transition() {
    let (setup, truth, start) = v8_setup();
    let rig = rig_mounted(setup, truth, 0.0, start, [0.0, 0.0], false).await;
    let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.unwrap();
    wait_phase(&rig.sup, id, "transition", Some(true)).await;
    {
        let mut inner = rig.sup.inner().write().await;
        inner.stop_motion();
        inner.request_macro_stop();
    }
    // the record ends under the same lock; generous bound for a loaded host
    let a = wait_done(&rig.sup, id, 10.0, |_| {}).await;
    assert_eq!((a.state.as_str(), a.code.as_deref(), a.phase.as_deref()), ("cancelled", Some("STOPPED"), Some("transition")), "{a:?}");
    assert_idle_after(&rig.sup, id).await;
}

#[tokio::test]
async fn camera_down_lease_loss_during_the_transition() {
    let (setup, truth, start) = v8_setup();
    let rig = rig_mounted(setup, truth, 0.0, start, [0.0, 0.0], true).await;
    let mut w = ws_rig_on(rig, None).await;
    let ack = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", tap(&w.rig).await).await;
    let id = ack["action_id"].as_u64().unwrap_or_else(|| panic!("ack {ack}"));
    // keep the lease alive until the transition move runs
    let deadline = Instant::now() + Duration::from_secs(20);
    loop {
        assert!(Instant::now() < deadline, "transition not reached: {:?}", w.rig.sup.get_observation().await.action);
        common::send(&mut w.ws, json!({"type":"heartbeat", "generation":w.generation, "epoch_id":w.epoch})).await;
        common::drain(&mut w.ws, 2).await;
        let inner = w.rig.sup.inner().read().await;
        let a = inner.last_action.as_ref().unwrap();
        assert!(a.is_running(), "{a:?}");
        if a.phase.as_deref() == Some("transition") && inner.active_trajectory.as_ref().is_some_and(|t| t.action_id == Some(id)) { break; }
        drop(inner);
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    let reply = common::ws_reply(&mut w.ws, json!({"type":"release_lease", "generation":w.generation}), &["lease_released"]).await;
    assert_eq!(reply["success"], true);
    let a = wait_done(&w.rig.sup, id, 10.0, |_| {}).await;
    assert_eq!((a.state.as_str(), a.code.as_deref(), a.phase.as_deref()), ("cancelled", Some("LEASE_LOST"), Some("transition")), "{a:?}");
    assert_idle_after(&w.rig.sup, id).await;
}

#[tokio::test]
async fn camera_down_collision_during_the_transition_holds() {
    let (setup, truth, start) = v8_setup();
    let arm = common::TestArm::new();
    let rig = rig_driven(setup, truth, 0.0, start, [0.0, 0.0], false, Some(arm.clone())).await;
    let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.unwrap();
    wait_phase(&rig.sup, id, "transition", Some(true)).await;
    arm.set_residual(1, -20.0);
    let a = wait_done(&rig.sup, id, 10.0, |_| {}).await;
    assert_eq!((a.state.as_str(), a.code.as_deref(), a.phase.as_deref()), ("failed", Some("COLLISION"), Some("transition")), "{a:?}");
    assert!(rig.sup.inner().read().await.collision.hold.is_some(), "collision hold");
    assert_idle_after(&rig.sup, id).await;
    assert!(rig.sup.inner().read().await.macro_run.is_none());
}

/// A raised table where only the camera body reaches it: lens beside the jaw tips (test mount,
/// rot 0, tilt -38, lens 30 mm off the tool axis at the tip plane) with a 90 x 90 x 40 mm body.
/// The TCP, the gripper check points and the lens clear the table on the camera-down
/// acquisition; a body corner does not at the end of the transition, so the grasp is rejected
/// up front (WORKSPACE_VIOLATION). With a 1 mm body the same grasp is accepted.
#[tokio::test]
async fn camera_down_rejects_a_camera_body_that_would_reach_the_table() {
    let setup = GraspSetup { camera_tilt_deg: -38.0, table_z_m: 0.10, grasp_height_m: 0.005, approach_height_m: 0.025, ..GraspSetup::default() };
    for (body, accepted) in [([0.001f32, 0.001, 0.001], true), ([0.09, 0.09, 0.04], false)] {
        let truth = Truth { rot_deg: 0.0, offset: [0.0, 0.0, 0.03], ..Truth::same() };
        let start = pitched_start(&kinematics::cam_mount(false, 0.0, -38.0), 0.5, 0.30);
        let rig = rig_mounted(setup.clone(), truth, 0.0, start, [0.0, 0.0], false).await;
        rig.sup.inner().write().await.config.camera_mount.body_size_m = Some(body);
        let r = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await;
        println!("camera body {body:?}: {:?}", r.as_ref().map(|_| ()).map_err(|e| (e.code.clone(), e.message.clone())));
        if accepted {
            let (id, _) = r.expect("accepted with a 1 mm body");
            rig.sup.inner().write().await.request_macro_stop();
            wait_done(&rig.sup, id, 5.0, |_| {}).await;
        } else {
            let e = r.expect_err("rejected");
            assert_eq!(e.code.as_deref(), Some("WORKSPACE_VIOLATION"), "{e:?}");
            assert!(e.message.contains("table_z_m"), "{e:?}");
        }
    }
    // profile field: optional, validated
    let mut c = ArmConfig::rebot_dm();
    assert!(c.camera_mount.body_size_m.is_none() && c.validate().is_ok());
    c.camera_mount.body_size_m = Some([0.05, 0.3, 0.04]);
    assert!(c.validate().is_err());
}

/// Look-down (v5) grasp where only the descent would take the camera to the table: tilt 0, the
/// lens `ahead_m` beyond the jaw-tip plane (along the tool axis), so with the tool vertical it is
/// that far below the TCP. At the default grasp height (15 mm) a lens 9 mm ahead stays 6 mm
/// above the table and the grasp completes; 14.5 mm ahead it would be 0.5 mm above, so the
/// grasp fails WORKSPACE_VIOLATION in the descend phase before that motion is installed (the
/// approach and centring are clear, so it is acked).
#[tokio::test]
async fn look_down_descent_that_would_bring_the_camera_to_the_table_fails() {
    // the height probe and the re-centre off: one descent from the approach height
    let setup = GraspSetup { height_probe_m: 0.0, recentre_height_m: 0.0, ..GraspSetup::default() };
    for (ahead_m, ok) in [(0.009, true), (0.0145, false)] {
        // lens 30 mm beside the tool axis along the jaws (along x_cam, the wide image axis)
        let truth = Truth { offset: [ahead_m, -0.03, 0.0], ..Truth::same() };
        let rig = rig(setup.clone(), truth, 0.0, [0.0, 0.0], false).await;
        let start_r = start_pose(&rig).await.r;
        let mut s = setup.clone();
        s.grip_pixel_norm = calibrate_grip(&rig, &setup, &start_r);
        rig.sup.inner().write().await.set_grasp_setup(s.clone());
        let config = rig.sup.inner().read().await.config.clone();
        let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.expect("acked: approach and centring are clear");
        let sup = rig.sup.clone();
        let mut lowest = f64::INFINITY;
        let a = wait_done(&rig.sup, id, 60.0, |_| {
            if let Ok(inner) = sup.inner().try_read() { lowest = lowest.min(envelope_min_z(&config, &inner.commanded_positions)); }
        }).await;
        let tcp_z = start_pose(&rig).await.p[2];
        println!("lens {:.1} mm ahead: {} {:?} in {:?}: {:?}, lowest TCP / check point / lens {lowest:.4} m, final TCP z {tcp_z:.4} m",
            ahead_m * 1000.0, a.state, a.code, a.phase, a.message);
        if ok {
            assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{a:?}");
            assert!(lowest >= s.table_z_m + grasp::TABLE_CLEARANCE_M - 1e-6, "{lowest}");
        } else {
            assert_eq!((a.state.as_str(), a.code.as_deref(), a.phase.as_deref()), ("failed", Some("WORKSPACE_VIOLATION"), Some("descend")), "{a:?}");
            assert!(a.message.as_deref().is_some_and(|m| m.contains("table_z_m")), "{a:?}");
            assert!((tcp_z - (s.table_z_m + s.approach_height_m)).abs() < 2e-3, "not descended: TCP z {tcp_z}");
            assert!(lowest >= s.table_z_m + grasp::TABLE_CLEARANCE_M, "{lowest}");
            assert_idle_after(&rig.sup, id).await;
        }
    }
}

/// Detector latency 120..250 ms per result, continuous (`/api/results/latest`) and triggered
/// (perception API 2): a stretch of stale frames (captured 30 s earlier) right after the
/// centring starts or the transition move ends is followed by fresh ones, and the grasp
/// recovers and completes, re-check included.
#[tokio::test]
async fn delayed_detector_recovers_from_stale_frames() {
    for triggered in [false, true] {
        for phase in ["centring", "transition"] {
            let (rig, s) = v8_recheck_rig().await;
            rig.sim.delayed.store(true, Ordering::SeqCst);
            rig.sim.per_request.store(triggered, Ordering::SeqCst);
            let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.unwrap();
            wait_phase(&rig.sup, id, phase, Some(phase == "transition")).await;
            if phase == "transition" {
                // stale from the end of the move: the re-check sees stale frames first
                while rig.sup.inner().read().await.active_trajectory.as_ref().is_some_and(|t| t.action_id == Some(id)) {
                    tokio::time::sleep(Duration::from_millis(2)).await;
                }
            }
            rig.sim.stale.store(true, Ordering::SeqCst);
            tokio::time::sleep(Duration::from_millis(400)).await;
            rig.sim.stale.store(false, Ordering::SeqCst);
            let a = wait_done(&rig.sup, id, 60.0, |_| {}).await;
            let mode = if triggered { "triggered" } else { "continuous" };
            println!("delayed {mode}, stale in {phase}: {} {:?} {:?}, iterations {:?}, final offset {:?}",
                a.state, a.code, a.message, a.iteration, a.final_offset_px);
            assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{mode} {phase}: {a:?}");
            assert!(a.final_offset_px.is_some(), "{mode} {phase}: re-check ran");
            let o = Outcome { status: a, centred_err_m: f64::NAN, final_pose: start_pose(&rig).await, turned: rig.sim.turned.load(Ordering::SeqCst), oriented: false };
            assert_holding_done(&o, &s, &rig.object, 0.004);
        }
    }
}

// ---- J6 alignment ----

/// The detector reports the long side at 30 deg from base x: the J6 roll turns the jaw closing
/// direction (end_link y) perpendicular to it, from the angle rather than the box aspect.
#[tokio::test]
async fn align_from_detector_angle_closes_across_the_long_side() {
    let setup = GraspSetup::default();
    let rig = rig(setup.clone(), centred_lens(), 0.0, [0.01, -0.01], false).await;
    let start_r = start_pose(&rig).await.r;
    let mut s = setup.clone();
    s.grip_pixel_norm = calibrate_grip(&rig, &setup, &start_r);
    rig.sup.inner().write().await.set_grasp_setup(s.clone());
    let long = 30f64.to_radians();
    *rig.sim.angle.lock().unwrap() = Some((long, 0.9));
    let image_angle = rig.sim.image_angle(&start_pose(&rig).await).unwrap().0;
    let o = run_grasp(&rig).await;
    let jaw = kinematics::mat_vec(&o.final_pose.r, &[0.0, 1.0, 0.0]);
    let dot = (jaw[0] * long.cos() + jaw[1] * long.sin()) / jaw[0].hypot(jaw[1]);
    println!("angle 30 deg (image {image_angle:.1} deg): {} {:?} align {:?} turn {:?} rad, jaw . long side {dot:.4}",
        o.status.state, o.status.code, o.status.align, o.status.align_turn_rad);
    assert_holding_done(&o, &s, &rig.object, 0.004);
    assert_eq!(o.status.align.as_deref(), Some("turned:angle"), "{:?}", o.status);
    assert!(dot.abs() < 3f64.to_radians().sin(), "jaws close across the long side: {dot}");
    let turn = f64::from(o.status.align_turn_rad.unwrap());
    assert!(turn.abs() <= std::f64::consts::PI, "{turn}");
    // the expected roll from the pre-align jaw direction (start heading: look-down of the start)
    let jaw0 = kinematics::mat_vec(&grasp::look_down(&start_r), &[0.0, 1.0, 0.0]);
    let t0 = grasp::roll_across_long_side(jaw0, [long.cos(), long.sin(), 0.0]);
    let d = (turn - t0).rem_euclid(std::f64::consts::PI);
    assert!(d.min(std::f64::consts::PI - d) < 2f64.to_radians(), "turn {turn} vs expected {t0} (mod pi)");
    // low confidence: the box aspect decides as before
    let rig = rig_low_conf(&s).await;
    let o = run_grasp(&rig).await;
    assert_holding_done(&o, &s, &rig.object, 0.004);
    assert!(o.status.align.as_deref().is_some_and(|a| a.ends_with(":box")), "{:?}", o.status.align);
}

/// Lens on the tool axis, 6 cm behind the jaw tips: the object under the jaws is in the middle
/// of the image (with `Truth::same` its box reaches the image border at approach height).
fn centred_lens() -> Truth { Truth { offset: [-0.06, 0.0, 0.0], ..Truth::same() } }

async fn rig_low_conf(s: &GraspSetup) -> Rig {
    let rig = rig(s.clone(), centred_lens(), 0.0, [0.01, -0.01], false).await;
    rig.sup.inner().write().await.set_grasp_setup(s.clone());
    *rig.sim.angle.lock().unwrap() = Some((30f64.to_radians(), 0.1));
    rig
}

/// Look-down with the lens 4 cm off the tool axis (`Truth::same`, no tilt): with the object
/// under the jaws at approach height its box runs past the image border. The grip pixel is, as
/// `grip_pixel_norm` is defined, where the table point under the jaws appears there. The
/// centring on the grip pixel sees a cut box; the grasp centres it uncut at an aim inside the
/// image, takes the height and the J6 alignment from those frames, and moves the TCP over the
/// box centre's point on the object's top plane (`shift`): the TCP ends over the object.
#[tokio::test]
async fn border_cut_box_is_centred_uncut_then_shifted() {
    let setup = GraspSetup::default();
    let rig = rig(setup.clone(), Truth::same(), 0.0, [0.01, 0.01], false).await;
    let start_r = start_pose(&rig).await.r;
    let under = Pose::new(grasp::look_down(&start_r), [rig.object[0], rig.object[1], setup.table_z_m + setup.approach_height_m]);
    let whole = rig.sim.project_box_whole(&under).expect("object in front of the camera");
    let [w, h] = setup.image_size_px;
    let cut = rig.sim.project_box(&under).expect("box centre inside the image");
    assert!(grasp::touches_border(&cut, setup.image_size_px), "the box under the jaws reaches the border: {whole:?}");
    let mut s = setup.clone();
    let g = rig.sim.cam.project(&under, &[rig.object[0], rig.object[1], setup.table_z_m]).expect("table point in front of the camera");
    s.grip_pixel_norm = [g[0] / w, g[1] / h];
    rig.sup.inner().write().await.set_grasp_setup(s.clone());
    let o = run_grasp(&rig).await;
    let a = &o.status;
    println!("border cut: {} {:?}, border {:?} shift {:?} px, height {:?} {:?} {:?}, align {:?}, centred xy error {:.2} mm, final {:.2} mm",
        a.state, a.code, a.border, a.border_shift_px, a.height_source, a.height_reason, a.object_height_m, a.align,
        o.centred_err_m * 1000.0, xy_err(&o.final_pose.p, &rig.object) * 1000.0);
    assert_eq!(a.border.as_deref(), Some("shifted"), "{a:?}");
    assert!(a.border_shift_px.is_some_and(|d| d[0].hypot(d[1]) > 20.0), "{a:?}");
    // the height probe and the J6 alignment ran on the uncut box
    assert_eq!(a.height_source.as_deref(), Some("scale"), "{a:?}");
    assert!(!a.align.as_deref().unwrap_or("").starts_with("skipped"), "{a:?}");
    assert!(a.phase_ms.contains_key("shift"), "{:?}", a.phase_ms);
    // centred on the cut box, without the shift, the TCP ended 1.96 mm off
    assert!(o.centred_err_m <= 0.001, "centred xy error {:.2} mm", o.centred_err_m * 1000.0);
    assert_holding_done(&o, &s, &rig.object, 0.004);
}

// ---- object height from the box scale change ----

/// Rig with the lens on the tool axis, the detector boxing the object's top face at `top` m,
/// grip pixel calibrated (`overrides` applied to the setup).
async fn height_rig(top: f64, overrides: Value) -> (Rig, GraspSetup) { height_rig_with(top, overrides, centred_lens()).await }

async fn height_rig_with(top: f64, overrides: Value, truth: Truth) -> (Rig, GraspSetup) {
    let setup = GraspSetup::default().patched(&overrides).unwrap();
    let rig = rig(setup.clone(), truth, 0.0, [0.01, 0.01], false).await;
    *rig.sim.top.lock().unwrap() = Some(top);
    let start_r = start_pose(&rig).await.r;
    let mut s = setup.clone();
    s.grip_pixel_norm = calibrate_grip(&rig, &setup, &start_r);
    rig.sup.inner().write().await.set_grasp_setup(s.clone());
    (rig, s)
}

/// TCP z (lowest seen) while the jaws close.
async fn run_grasp_close_z(rig: &Rig) -> (armd::motion::ActionStatus, f64) {
    let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(rig).await).await.expect("grasp accepted");
    let sup = rig.sup.clone();
    let mut close_z = f64::NAN;
    let a = wait_done(&rig.sup, id, 60.0, |a| {
        if a.phase.as_deref() == Some("close") {
            if let Ok(inner) = sup.inner().try_read() { close_z = kinematics::fk(&kinematics::q6(&inner.commanded_positions)).p[2]; }
        }
    }).await;
    (a, close_z)
}

#[tokio::test]
async fn height_probe_estimates_a_tall_object_and_grasps_part_way_up() {
    let (rig, s) = height_rig(0.08, json!({})).await;
    let (a, close_z) = run_grasp_close_z(&rig).await;
    println!("0.08 m object: {} {:?}, height {:?} ({:?} {:?}), close height {:?}, TCP z at close {close_z:.4}",
        a.state, a.code, a.object_height_m, a.height_source, a.height_reason, a.close_height_m);
    assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{a:?}");
    assert_eq!(a.height_source.as_deref(), Some("scale"), "{a:?}");
    assert_eq!((a.probe_mode.as_deref(), a.lateral_baseline_m), (Some("up"), None), "{a:?}");
    let h = f64::from(a.object_height_m.unwrap());
    assert!((h - 0.08).abs() <= 0.01, "estimate {h}");
    assert!((f64::from(a.close_height_m.unwrap()) - 0.04).abs() < 0.006, "{a:?}");
    assert!((close_z - (s.table_z_m + f64::from(a.close_height_m.unwrap()))).abs() < 2e-3, "TCP at the close height: {close_z}");
}

#[tokio::test]
async fn height_probe_flat_object_and_disabled_keep_the_grasp_height() {
    for (top, overrides) in [(0.0, json!({})), (0.08, json!({"height_probe_m": 0.0}))] {
        let (rig, s) = height_rig(top, overrides.clone()).await;
        let (a, close_z) = run_grasp_close_z(&rig).await;
        println!("top {top} {overrides}: {} {:?}, height {:?} ({:?} {:?}), close height {:?}, TCP z at close {close_z:.4}",
            a.state, a.code, a.object_height_m, a.height_source, a.height_reason, a.close_height_m);
        assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{a:?}");
        match a.height_source.as_deref() {
            Some("scale") => assert!(f64::from(a.object_height_m.unwrap()).abs() < 0.01, "{a:?}"),
            Some("setup") => {}
            other => panic!("{other:?}"),
        }
        if top > 0.0 { assert_eq!(a.height_reason.as_deref(), Some("disabled")); }
        assert!((f64::from(a.close_height_m.unwrap()) - s.grasp_height_m).abs() < 0.006, "{a:?}");
        assert!((close_z - (s.table_z_m + s.grasp_height_m)).abs() < 0.006, "{close_z}");
    }
}

#[tokio::test]
async fn height_probe_too_tall_object_fails() {
    // lens 12 cm behind the jaw tips: the close top face stays inside the image
    let (rig, s) = height_rig_with(0.135, json!({}), Truth { offset: [-0.12, 0.0, 0.0], ..Truth::same() }).await;
    let (a, _) = run_grasp_close_z(&rig).await;
    println!("0.135 m object: {} {:?} {:?}", a.state, a.code, a.message);
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some("OBJECT_TOO_TALL")), "{a:?}");
    let z = start_pose(&rig).await.p[2];
    assert!((z - (s.table_z_m + s.approach_height_m)).abs() < 2e-3, "back at the start pose of the probe: {z}");
    assert_idle_after(&rig.sup, a.id).await;
}

/// A 0.12 m object, taller than approach_height_m − height_probe_m (where the former downward
/// probe took the jaw tips): the probe moves up from the start pose and back, the TCP never goes
/// below the start height during it, and the estimate is valid (or OBJECT_TOO_TALL).
#[tokio::test]
async fn height_probe_moves_up_over_a_tall_object() {
    let (rig, s) = height_rig(0.12, json!({})).await;
    let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.expect("grasp accepted");
    let sup = rig.sup.clone();
    let (mut start_z, mut min_z, mut max_z) = (f64::NAN, f64::INFINITY, f64::NEG_INFINITY);
    let a = wait_done(&rig.sup, id, 60.0, |a| {
        let Ok(inner) = sup.inner().try_read() else { return };
        let z = kinematics::fk(&kinematics::q6(&inner.commanded_positions)).p[2];
        match a.phase.as_deref() {
            Some("centring") => start_z = z,
            Some("probe") => { min_z = min_z.min(z); max_z = max_z.max(z); }
            _ => {}
        }
    }).await;
    println!("0.12 m object: {} {:?}, height {:?} ({:?} {:?}), close height {:?}; probe TCP z min {min_z:.4} max {max_z:.4}, start {start_z:.4} \
              (old downward probe would reach {:.4}, object top {:.3})",
        a.state, a.code, a.object_height_m, a.height_source, a.height_reason, a.close_height_m,
        start_z - 0.04, s.table_z_m + 0.12);
    assert!(min_z.is_finite(), "probe phase observed");
    assert_eq!(a.probe_dz_m, Some(s.height_probe_m as f32), "{a:?}");
    assert!(min_z >= start_z - 1e-3, "TCP below the start height during the probe: {min_z} < {start_z}");
    assert!(max_z > start_z + s.height_probe_m - 2e-3, "moved up: {max_z}");
    match a.code.as_deref() {
        Some("OBJECT_TOO_TALL") => {}
        _ => {
            assert_eq!((a.state.as_str(), a.height_source.as_deref()), ("done", Some("scale")), "{a:?}");
            assert!((f64::from(a.object_height_m.unwrap()) - 0.12).abs() <= 0.01, "{a:?}");
        }
    }
}

/// The workspace ceiling leaves room for a 0.06 m raise but not for the default 0.10 m: the
/// probe falls back to 0.06 m (`probe_dz_m`) and still estimates the height. Without room for
/// either it falls back to the setup height with `probe_unreachable`.
#[tokio::test]
async fn height_probe_falls_back_to_a_shorter_raise() {
    // kit check points reach 0.16 m above the TCP with the tool down; the probe starts at 0.15 m
    for (ceiling, dz) in [(0.15 + 0.16 + 0.08, Some(0.06f32)), (0.15 + 0.16 + 0.03, None)] {
        let s0 = GraspSetup::default();
        let rig = rig_mounted(s0.clone(), centred_lens(), 0.0, look_down_start(0.0, 0.18), [0.01, 0.01], false).await;
        *rig.sim.top.lock().unwrap() = Some(0.08);
        let mut ws = rig.sup.inner().write().await;
        let ws_mut = ws.config.workspace.as_mut().unwrap();
        ws_mut.box_max_m[2] = ceiling as f32;
        // Synthetic ceiling tests end_effector height headroom without tripping on the physical elbow (0.444 m).
        ws_mut.link_capsules.clear();
        drop(ws);
        let start_r = start_pose(&rig).await.r;
        let mut s = s0.clone();
        s.grip_pixel_norm = calibrate_grip(&rig, &s0, &start_r);
        rig.sup.inner().write().await.set_grasp_setup(s.clone());
        let (a, _) = run_grasp_close_z(&rig).await;
        println!("ceiling {ceiling:.2} m: {} {:?}, probe_dz_m {:?}, height {:?} ({:?} {:?})",
            a.state, a.code, a.probe_dz_m, a.object_height_m, a.height_source, a.height_reason);
        assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{a:?}");
        assert_eq!(a.probe_dz_m, dz, "{a:?}");
        match dz {
            Some(_) => {
                assert_eq!(a.height_source.as_deref(), Some("scale"), "{a:?}");
                assert!((f64::from(a.object_height_m.unwrap()) - 0.08).abs() <= 0.01, "{a:?}");
            }
            None => assert_eq!(a.height_reason.as_deref(), Some("probe_unreachable"), "{a:?}"),
        }
    }
}

// ---- lateral (parallax) height probe ----

/// Lateral probe on a 0.08 m object: the tool moves ±B/2 along the image x axis at the
/// centring height, the box-centre parallax gives the top within a few mm, and the arm is back
/// at the start pose before the descent. B = 0.02 m: the 138 px wide box (lens 0.13 m above the
/// top) then straddles the principal point at both poses; with 0.04 m it lies on either side of
/// it, no box edge is on the top face at both poses, and the probe falls back (`edge`).
#[tokio::test]
async fn lateral_probe_estimates_the_top_from_parallax() {
    let (rig, s) = height_rig(0.08, json!({"height_probe_mode": "lateral", "lateral_probe_m": 0.02})).await;
    let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.expect("grasp accepted");
    let sup = rig.sup.clone();
    let (mut start, mut min_z, mut max_xy) = (None::<Vec3>, f64::INFINITY, 0.0f64);
    let mut after_probe = None::<Vec3>;
    let a = wait_done(&rig.sup, id, 60.0, |a| {
        let Ok(inner) = sup.inner().try_read() else { return };
        let p = kinematics::fk(&kinematics::q6(&inner.commanded_positions)).p;
        match a.phase.as_deref() {
            Some("centring") => start = Some(p),
            Some("probe") => if let Some(st) = start { min_z = min_z.min(p[2]); max_xy = max_xy.max(xy_err(&p, &st)); },
            Some(_) if min_z.is_finite() && after_probe.is_none() => after_probe = Some(p),
            _ => {}
        }
    }).await;
    println!("lateral 0.08 m: {} {:?}, height {:?} ({:?} {:?} {:?}), baseline {:?} m err {:?} mm; probe min z {min_z:.4}, max xy {max_xy:.4}",
        a.state, a.code, a.object_height_m, a.height_source, a.height_reason, a.probe_mode, a.lateral_baseline_m, a.lateral_baseline_err_mm);
    assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{a:?}");
    assert_eq!((a.height_source.as_deref(), a.probe_mode.as_deref(), a.height_reason.as_deref()), (Some("parallax"), Some("lateral"), None), "{a:?}");
    let h = f64::from(a.object_height_m.unwrap());
    assert!((h - 0.08).abs() <= 0.004, "estimate {h}");
    assert!((f64::from(a.lateral_baseline_m.unwrap()) - s.lateral_probe_m).abs() < 1e-3, "{a:?}");
    assert!(a.lateral_baseline_err_mm.unwrap().abs() < 1.0, "{a:?}");
    assert!(a.probe_dz_m.is_none(), "no up move: {a:?}");
    let st = start.expect("centring observed");
    assert!(min_z >= st[2] - 1e-3, "TCP below the start height during the probe: {min_z} < {}", st[2]);
    assert!((max_xy - s.lateral_probe_m / 2.0).abs() < 2e-3, "moved ±B/2 sideways: {max_xy}");
    let back = after_probe.expect("phase after the probe observed");
    assert!(kinematics::norm(&kinematics::sub(&back, &st)) < 2e-3, "back at the start pose: {back:?} vs {st:?}");
}

/// The box would reach the image border at a lateral pose (0.08 m baseline, 0.03 m object at
/// 0.06 m approach height: box 200 x 333 px, predicted shift ±228 px): no lateral move, the up probe
/// runs and the reason is recorded.
#[tokio::test]
async fn lateral_probe_falls_back_to_up_at_the_border() {
    let (rig, _s) = height_rig(0.03, json!({"height_probe_mode": "lateral", "lateral_probe_m": 0.08, "approach_height_m": 0.06})).await;
    let (a, _) = run_grasp_close_z(&rig).await;
    println!("lateral at the border: {} {:?}, height {:?} ({:?} {:?} {:?})", a.state, a.code, a.object_height_m, a.height_source, a.height_reason, a.probe_mode);
    assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{a:?}");
    assert_eq!((a.height_source.as_deref(), a.probe_mode.as_deref()), (Some("scale"), Some("up")), "{a:?}");
    assert_eq!(a.height_reason.as_deref(), Some("lateral_fallback:border"), "{a:?}");
    assert!(a.lateral_baseline_m.is_none(), "{a:?}");
    assert!((f64::from(a.object_height_m.unwrap()) - 0.03).abs() <= 0.01, "{a:?}");
}

/// With the default 0.04 m baseline the 138 px box lies right of the principal point at the
/// −B/2 pose and left of it at the +B/2 pose (the shift predicted from the table distance,
/// ±61 px, keeps it across; the seen shift is ±92 px): no edge is on the top face at both
/// poses, the arm returns to the start and the up probe runs (`lateral_fallback:edge`).
#[tokio::test]
async fn lateral_probe_falls_back_to_up_without_a_common_top_face_edge() {
    let (rig, _s) = height_rig(0.08, json!({"height_probe_mode": "lateral"})).await;
    let (a, _) = run_grasp_close_z(&rig).await;
    println!("lateral without a common edge: {} {:?}, height {:?} ({:?} {:?} {:?})", a.state, a.code, a.object_height_m, a.height_source, a.height_reason, a.probe_mode);
    assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{a:?}");
    assert_eq!((a.height_source.as_deref(), a.probe_mode.as_deref()), (Some("scale"), Some("up")), "{a:?}");
    assert_eq!(a.height_reason.as_deref(), Some("lateral_fallback:edge"), "{a:?}");
    assert!((f64::from(a.object_height_m.unwrap()) - 0.08).abs() <= 0.01, "{a:?}");
}

/// A keep-out box where the TCP would be at the +B/2 pose: the lateral probe does not move and
/// the up probe runs (`lateral_fallback:unreachable`).
#[tokio::test]
async fn lateral_probe_falls_back_to_up_when_a_pose_is_unreachable() {
    let (rig, s) = height_rig(0.08, json!({"height_probe_mode": "lateral"})).await;
    // base-frame direction of the image x axis at the (look-down) centring orientation
    let r = start_pose(&rig).await.r;
    let x = kinematics::column(&kinematics::cam_rotation(&Pose::new(r, [0.0; 3]), &rig.sim.cam.rec), 0);
    let n = x[0].hypot(x[1]);
    let dir = [x[0] / n, x[1] / n];
    // the TCP is above the object at the centring pose; the box covers 12..40 mm along +dir
    let (c, z) = (rig.object, s.table_z_m + s.approach_height_m);
    let (p0, p1) = ([c[0] + 0.012 * dir[0], c[1] + 0.012 * dir[1]], [c[0] + 0.04 * dir[0], c[1] + 0.04 * dir[1]]);
    let lo = [p0[0].min(p1[0]) - 0.008, p0[1].min(p1[1]) - 0.008, z - 0.005].map(|v| v as f32);
    let hi = [p0[0].max(p1[0]) + 0.008, p0[1].max(p1[1]) + 0.008, z + 0.005].map(|v| v as f32);
    let mut inner = rig.sup.inner().write().await;
    let ws_mut = inner.config.workspace.as_mut().unwrap();
    ws_mut.keepout_boxes = vec![armd::model::KeepoutBox { name: "probe".into(), min_m: lo, max_m: hi }];
    // Place obstacle 4 mm off the descent axis to block only the lateral TCP probe without intersecting wrist link5 during descent.
    ws_mut.link_capsules.clear();
    drop(inner);
    let (a, _) = run_grasp_close_z(&rig).await;
    println!("lateral unreachable: {} {:?} {:?}, height {:?} ({:?} {:?} {:?})", a.state, a.code, a.message, a.object_height_m, a.height_source, a.height_reason, a.probe_mode);
    assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{a:?}");
    assert_eq!((a.height_source.as_deref(), a.probe_mode.as_deref()), (Some("scale"), Some("up")), "{a:?}");
    assert_eq!(a.height_reason.as_deref(), Some("lateral_fallback:unreachable"), "{a:?}");
    assert!((f64::from(a.object_height_m.unwrap()) - 0.08).abs() <= 0.01, "{a:?}");
}

// ---- mid-height re-centre and descent contact ----

/// Look-down with the lens on the tool axis, re-centre enabled at 0.07 m (off by default): the
/// target is re-centred on the way down; losing it there skips the re-centre and the grasp completes.
#[tokio::test]
async fn recentre_at_mid_height_and_skip_when_lost() {
    for lose in [false, true] {
        let (rig, s) = height_rig(0.05, json!({"recentre_height_m": 0.07})).await;
        let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.unwrap();
        if lose {
            wait_phase(&rig.sup, id, "recentre", None).await;
            rig.sim.visible.store(false, Ordering::SeqCst);
        }
        let mut phases: Vec<String> = Vec::new();
        let a = wait_done(&rig.sup, id, 60.0, |a| {
            let p = a.phase.as_deref().unwrap_or_default().to_string();
            if phases.last() != Some(&p) { phases.push(p); }
        }).await;
        let final_pose = start_pose(&rig).await;
        println!("re-centre (lost {lose}): {} {:?} recentre {:?} iterations {:?}, final xy error {:.2} mm, phases {phases:?}",
            a.state, a.code, a.recentre, a.recentre_iterations, xy_err(&final_pose.p, &rig.object) * 1000.0);
        assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{a:?}");
        let pos = |p: &str| phases.iter().position(|q| q == p).unwrap_or_else(|| panic!("{p} in {phases:?}"));
        assert!(pos("recentre") < pos("close"), "{phases:?}");
        if !lose { assert!(pos("descend") < pos("recentre"), "{phases:?}"); }
        if lose {
            assert_eq!(a.recentre.as_deref(), Some("skipped:target_lost"), "{a:?}");
        } else {
            assert_eq!(a.recentre.as_deref(), Some("done"), "{a:?}");
            assert!(xy_err(&final_pose.p, &rig.object) < 0.004);
        }
        assert!((final_pose.p[2] - (s.table_z_m + s.lift_height_m)).abs() < 2e-3);
    }
}

/// Descent with a driver reporting residual torques: peaks are logged; with
/// `descend_contact_stop_nm` a J2 residual above it stops the descent where the arm is, and the
/// jaws close there.
#[tokio::test]
async fn descend_logs_residual_peaks_and_stops_on_contact() {
    for (stop, residual) in [(None, 1.0f32), (Some(2.0), 3.0f32)] {
        let setup = GraspSetup { height_probe_m: 0.0, recentre_height_m: 0.0, descend_contact_stop_nm: stop, ..GraspSetup::default() };
        let arm = common::TestArm::new();
        let rig = rig_driven(setup.clone(), centred_lens(), 0.0, look_down_start(0.0, 0.24), [0.0, 0.0], false, Some(arm.clone())).await;
        rig.sup.inner().write().await.config.collision = None;
        let start_r = start_pose(&rig).await.r;
        let mut s = setup.clone();
        s.grip_pixel_norm = calibrate_grip(&rig, &setup, &start_r);
        rig.sup.inner().write().await.set_grasp_setup(s.clone());
        let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.unwrap();
        wait_phase(&rig.sup, id, "descend", Some(true)).await;
        // contact below 0.08 m
        let deadline = Instant::now() + Duration::from_secs(20);
        while start_pose(&rig).await.p[2] > s.table_z_m + 0.08 {
            assert!(Instant::now() < deadline, "descent did not reach 0.08 m");
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        arm.set_residual(1, residual);
        let mut close_z = f64::NAN;
        let sup = rig.sup.clone();
        let a = wait_done(&rig.sup, id, 40.0, |a| {
            if a.phase.as_deref() == Some("close") {
                if let Ok(inner) = sup.inner().try_read() { close_z = kinematics::fk(&kinematics::q6(&inner.commanded_positions)).p[2]; }
            }
        }).await;
        println!("descent stop {stop:?}, residual {residual}: {} {:?}, peaks {:?} N·m / {:?} rad, contact {:?} at {:?} m, TCP z at close {close_z:.4}",
            a.state, a.code, a.descend_peak_residual_nm, a.descend_peak_tracking_rad, a.descend_contact, a.descend_contact_height_m);
        // the driver rig does not stop the jaws on the object (its ideal servo reports the
        // commanded J7), so the close may end NOT_HOLDING; the descent is what is checked here
        assert!(a.state == "done" || a.code.as_deref() == Some("NOT_HOLDING"), "{a:?}");
        let pk = a.descend_peak_residual_nm.expect("residual peaks logged");
        assert!((pk[0] - residual).abs() < 0.05 && pk[1] < 0.05, "{pk:?}");
        assert!(a.descend_peak_tracking_rad.is_some());
        match stop {
            None => {
                assert_eq!(a.descend_contact, None);
                assert!((close_z - (s.table_z_m + s.grasp_height_m)).abs() < 2e-3, "full descent: {close_z}");
            }
            Some(_) => {
                assert_eq!(a.descend_contact, Some(true), "{a:?}");
                let h = f64::from(a.descend_contact_height_m.unwrap());
                assert!(h > s.grasp_height_m + 0.01 && h < 0.08 + 1e-3, "{h}");
                assert!((close_z - (s.table_z_m + h)).abs() < 3e-3, "closed where it stopped: {close_z} vs {h}");
            }
        }
    }
}

// ---- holding re-check after the lift ----

/// Held through the lift: done. The object slips out after the close (the jaw closes onto its
/// hold target, the grip torque drops): the re-grip closes the jaws fully, DROPPED at lift
/// height, gripper not reopened.
#[tokio::test]
async fn post_lift_recheck_reports_a_dropped_object() {
    for drop_it in [false, true] {
        let setup = GraspSetup::default();
        let rig = rig(setup.clone(), centred_lens(), 0.0, [0.0, 0.0], false).await;
        let start_r = start_pose(&rig).await.r;
        let mut s = setup.clone();
        s.grip_pixel_norm = calibrate_grip(&rig, &setup, &start_r);
        rig.sup.inner().write().await.set_grasp_setup(s.clone());
        let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.unwrap();
        if drop_it {
            wait_phase(&rig.sup, id, "lift", Some(true)).await;
            rig.hold.store(false, Ordering::SeqCst);
            tokio::time::sleep(Duration::from_millis(30)).await;
            rig.sup.inner().write().await.torques[6] = 0.0;
        }
        let a = wait_done(&rig.sup, id, 60.0, |_| {}).await;
        let pose = start_pose(&rig).await;
        let j7 = rig.sup.inner().read().await.commanded_positions[6];
        println!("post-lift (dropped {drop_it}): {} {:?} {:?} holding {:?} gap {:?}, TCP z {:.3}, J7 command {j7:.2}",
            a.state, a.code, a.message, a.holding, a.gripper_gap_rad, pose.p[2]);
        assert!((pose.p[2] - (s.table_z_m + s.lift_height_m)).abs() < 2e-3, "at lift height");
        if drop_it {
            assert_eq!((a.state.as_str(), a.code.as_deref(), a.holding), ("failed", Some("DROPPED"), Some(false)), "{a:?}");
            assert_eq!(a.post_lift.as_deref(), Some("dropped"));
            assert!(a.gripper_gap_rad.is_some_and(|g| g < 0.3), "re-grip closed: {a:?}");
            assert!(j7 > OBJECT_J7 - 0.3, "gripper not reopened: {j7}");
            assert_idle_after(&rig.sup, id).await;
        } else {
            assert_eq!((a.state.as_str(), a.holding, a.post_lift.as_deref()), ("done", Some(true), Some("held")), "{a:?}");
        }
    }
}

/// The object is still in the jaws after the lift but the grip torque is gone (it turned in the
/// jaws; the jaw rests on its hold target): the re-grip stops on it, done with `regripped`.
#[tokio::test]
async fn post_lift_regrip_keeps_a_loose_object() {
    let setup = GraspSetup::default();
    let rig = rig(setup.clone(), centred_lens(), 0.0, [0.0, 0.0], false).await;
    let start_r = start_pose(&rig).await.r;
    let mut s = setup.clone();
    s.grip_pixel_norm = calibrate_grip(&rig, &setup, &start_r);
    rig.sup.inner().write().await.set_grasp_setup(s.clone());
    let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.unwrap();
    wait_phase(&rig.sup, id, "lift", Some(true)).await;
    // loose: the jaw reaches its hold target, no grip torque
    rig.hold.store(false, Ordering::SeqCst);
    tokio::time::sleep(Duration::from_millis(30)).await;
    rig.sup.inner().write().await.torques[6] = 0.0;
    // the object is still between the jaws: the re-grip stops on it
    wait_phase(&rig.sup, id, "regrip", None).await;
    rig.hold.store(true, Ordering::SeqCst);
    let a = wait_done(&rig.sup, id, 30.0, |_| {}).await;
    let pose = start_pose(&rig).await;
    println!("loose object: {} {:?} holding {:?} gap {:?} post_lift {:?}, TCP z {:.3}", a.state, a.code, a.holding, a.gripper_gap_rad, a.post_lift, pose.p[2]);
    assert_eq!((a.state.as_str(), a.holding, a.post_lift.as_deref()), ("done", Some(true), Some("regripped")), "{a:?}");
    assert!(a.gripper_gap_rad.is_some_and(|g| g >= 0.3), "{a:?}");
    assert!((pose.p[2] - (s.table_z_m + s.lift_height_m)).abs() < 2e-3);
}

/// A wide object stops the grasp before the descent, with a confident angle and with a low one
/// (a disc or a square: the width is valid without an angle).
#[tokio::test]
async fn too_wide_stops_before_descend() {
    for conf in [0.9, 0.05] {
        too_wide_case(conf).await;
    }
}

async fn too_wide_case(conf: f64) {
    let setup = GraspSetup {
        camera_tilt_deg: 0.0,
        max_grasp_width_m: 0.060,
        ..GraspSetup::default()
    };
    let rig = rig(setup.clone(), centred_lens(), 0.0, [0.0, 0.0], false).await;
    let start_r = start_pose(&rig).await.r;
    let mut s = setup.clone();
    s.grip_pixel_norm = calibrate_grip(&rig, &setup, &start_r);
    rig.sup.inner().write().await.set_grasp_setup(s.clone());

    *rig.sim.angle.lock().unwrap() = Some((0.0, conf));
    *rig.sim.width_px.lock().unwrap() = Some(350.0);

    let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.expect("grasp accepted");
    let sup = rig.sup.clone();
    let mut min_z = f64::MAX;
    let a = wait_done(&rig.sup, id, 30.0, |_| {
        if let Ok(inner) = sup.inner().try_read() {
            let z = kinematics::fk(&kinematics::q6(&inner.commanded_positions)).p[2];
            if z < min_z { min_z = z; }
        }
    }).await;

    println!("too wide (angle_conf {conf}): {} {:?}, phase {:?}, width {:?}, min TCP z {min_z:.4}", a.state, a.code, a.phase, a.grasp_width_m);
    assert_eq!(a.state, "failed", "{a:?}");
    assert_eq!(a.code.as_deref(), Some("TOO_WIDE"), "{a:?}");
    assert_eq!(a.phase.as_deref(), Some("align"), "{a:?}");
    let measured_w = a.grasp_width_m.expect("grasp_width_m recorded in action");
    assert!(measured_w > 0.060, "measured width {measured_w} should exceed 0.060");
    let approach_z = s.table_z_m + s.approach_height_m;
    assert!(min_z >= approach_z - 0.005, "TCP descended to {min_z:.4}, below approach height {approach_z:.4}");
}

#[tokio::test]
async fn width_within_limit_grasps() {
    let setup = GraspSetup {
        camera_tilt_deg: 0.0,
        max_grasp_width_m: 0.075,
        ..GraspSetup::default()
    };
    let rig = rig(setup.clone(), centred_lens(), 0.0, [0.0, 0.0], false).await;
    let start_r = start_pose(&rig).await.r;
    let mut s = setup.clone();
    s.grip_pixel_norm = calibrate_grip(&rig, &setup, &start_r);
    rig.sup.inner().write().await.set_grasp_setup(s.clone());

    *rig.sim.angle.lock().unwrap() = Some((0.0, 0.9));
    *rig.sim.width_px.lock().unwrap() = Some(160.0);

    let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.expect("grasp accepted");
    let a = wait_done(&rig.sup, id, 30.0, |_| {}).await;

    println!("within limit: {} {:?}, holding {:?}, width {:?}", a.state, a.code, a.holding, a.grasp_width_m);
    assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{a:?}");
    let measured_w = a.grasp_width_m.expect("grasp_width_m recorded in action");
    assert!((measured_w - 0.040).abs() < 0.008, "measured width {measured_w} vs expected ~0.040");
}

#[tokio::test]
async fn noisy_angles_skips_align_and_too_wide_fires_along_jaw_axis() {
    let setup = GraspSetup {
        camera_tilt_deg: 0.0,
        max_grasp_width_m: 0.060,
        align_max_angle_spread_deg: 15.0,
        ..GraspSetup::default()
    };
    let rig = rig(setup.clone(), centred_lens(), 0.0, [0.0, 0.0], false).await;
    let start_r = start_pose(&rig).await.r;
    let mut s = setup.clone();
    s.grip_pixel_norm = calibrate_grip(&rig, &setup, &start_r);
    rig.sup.inner().write().await.set_grasp_setup(s.clone());

    *rig.sim.angle.lock().unwrap() = Some((0.0, 0.9));
    *rig.sim.angle_noise.lock().unwrap() = Some(40.0);
    *rig.sim.width_px.lock().unwrap() = Some(350.0);

    let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.expect("grasp accepted");
    let a = wait_done(&rig.sup, id, 30.0, |_| {}).await;

    assert_eq!(a.state, "failed");
    assert_eq!(a.code.as_deref(), Some("TOO_WIDE"));
    assert_eq!(a.align.as_deref(), Some("skipped:angle_spread"));
    let spread = a.angle_spread_deg.expect("angle_spread_deg recorded");
    assert!(spread > 15.0, "spread {spread} should exceed 15 deg");
    let measured_w = a.grasp_width_m.expect("grasp_width_m recorded");
    assert!(measured_w > 0.060, "measured width along jaw axis {measured_w} should exceed 0.060");
}

#[tokio::test]
async fn dry_run_checks_reachability_without_starting_action() {
    let setup = GraspSetup { camera_tilt_deg: 0.0, ..GraspSetup::default() };
    let rig = rig(setup.clone(), centred_lens(), 0.0, [0.0, 0.0], false).await;
    let body_ok = tap(&rig).await;
    let mut body_dry = body_ok.clone();
    body_dry["dry_run"] = json!(true);

    // 1. In-reach target passes dry run
    let (reachable, code, message) = grasp::dry_run(&rig.sup, &body_dry).await;
    assert!(reachable, "in-reach target should be reachable");
    assert_eq!(code, None);
    assert_eq!(message, None);

    // Verify no action started and no macro alive
    assert!(!rig.sup.inner().read().await.macro_alive(1));
    assert!(rig.sup.inner().read().await.last_action.is_none());

    // 2. Unreachable target (out of reach / workspace violation) fails dry run
    let mut body_out = body_dry.clone();
    body_out["overrides"] = json!({"table_z_m": -0.3, "approach_height_m": 0.1});
    let (reachable, code, message) = grasp::dry_run(&rig.sup, &body_out).await;
    assert!(!reachable, "workspace violation target should be unreachable");
    assert!(code.is_some(), "code should be Some on unreachable: {code:?}");
    assert!(message.is_some());


    // 3. WS command_ack supports dry_run
    let mut w = ws_rig_on(rig, None).await;
    let res = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", body_dry).await;
    assert_eq!(res["type"], "command_ack");
    assert_eq!(res["reachable"], true);
    assert!(res["action_id"].is_null(), "dry run should not return action_id");
}

#[tokio::test]
async fn grasp_diagnostics_phase_ms_and_motions_with_tracking_error() {
    let setup = GraspSetup::default();
    let rig = rig(setup.clone(), centred_lens(), 0.0, [0.0, 0.0], false).await;
    let start_r = start_pose(&rig).await.r;
    let mut s = setup.clone();
    s.grip_pixel_norm = calibrate_grip(&rig, &setup, &start_r);
    rig.sup.inner().write().await.set_grasp_setup(s.clone());

    // Continuously force measured offset on J1 (measured differs from commanded by 0.02 rad ~ 5 mm)
    let sup_offset = rig.sup.clone();
    let offset_active = Arc::new(AtomicBool::new(true));
    let offset_active_bg = offset_active.clone();
    tokio::spawn(async move {
        while offset_active_bg.load(Ordering::SeqCst) {
            tokio::time::sleep(Duration::from_millis(1)).await;
            let mut inner = sup_offset.inner().write().await;
            if inner.commanded_positions.len() >= 6 && inner.measured_positions.len() >= 6 {
                inner.measured_positions[0] = inner.commanded_positions[0] + 0.02;
            }
        }
    });

    let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.expect("grasp accepted");
    let a = wait_done(&rig.sup, id, 40.0, |_| {}).await;
    offset_active.store(false, Ordering::SeqCst);
    assert_eq!(a.state, "done");

    // Phase timing verification
    assert!(!a.phase_ms.is_empty(), "phase_ms should not be empty");
    assert!(a.phase_ms.contains_key("approach"), "missing approach phase: {:?}", a.phase_ms);
    assert!(a.phase_ms.contains_key("descend"), "missing descend phase: {:?}", a.phase_ms);
    assert!(a.phase_ms.contains_key("lift"), "missing lift phase: {:?}", a.phase_ms);

    let sum_ms: u32 = a.phase_ms.values().sum();
    let wall_ms = ((a.finished_at.unwrap() - a.started_at) * 1000.0) as u32;
    assert!(sum_ms <= wall_ms + 100, "sum of phase_ms {sum_ms} should be <= wall time {wall_ms}");

    // Motion diagnostics verification
    assert!(!a.motions.is_empty(), "motions should not be empty");
    for m in &a.motions {
        assert!(m.ms > 0, "motion duration should be positive: {:?}", m);
    }
    assert!(a.motions.iter().any(|m| m.track_mm > 1.0), "track_mm should reflect measured offset: {:?}", a.motions);
}

#[tokio::test]
async fn orchestrator_test_pre_open_j7_before_descend() {
    // 4 cm object with pre_open "on": preopens to width (0.040) + margin (0.015) = 0.055 m.
    // Sim model: [[0, 0], [-5.7, 0.085]] -> j7_for(0.055) ≈ -3.688 rad.
    // Also tests that pre-open is allowed without an accepted angle (angle_conf low 0.1).
    let setup = GraspSetup {
        pre_open: armd::grasp::PreOpen::On,
        pre_open_margin_m: 0.015,
        ..GraspSetup::default()
    };
    let rig = rig(setup.clone(), centred_lens(), 0.0, [0.0, 0.0], false).await;
    let start_r = start_pose(&rig).await.r;
    let mut s = setup.clone();
    s.grip_pixel_norm = calibrate_grip(&rig, &setup, &start_r);
    rig.sup.inner().write().await.set_grasp_setup(s.clone());

    // Low confidence angle: no accepted angle, but pre-open still runs along the jaw axis
    *rig.sim.angle.lock().unwrap() = Some((0.0, 0.1));
    *rig.sim.width_px.lock().unwrap() = Some(160.0); // ~0.040 m width on table

    let sup = rig.sup.clone();
    let mut pre_descend_j7 = None;

    let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.expect("grasp accepted");
    let a = wait_done(&rig.sup, id, 40.0, |_| {
        if let Ok(inner) = sup.inner().try_read() {
            if let Some(act) = &inner.last_action {
                if act.phase.as_deref() == Some("descend") && pre_descend_j7.is_none() {
                    pre_descend_j7 = inner.commanded_positions.get(6).copied();
                }
            }
        }
    }).await;

    assert_eq!(a.state, "done");
    assert_eq!(a.holding, Some(true));
    assert_eq!(a.pre_open.as_deref(), Some("done"));
    let pre_w = a.pre_open_m.expect("pre_open_m recorded");
    assert!((pre_w - 0.055).abs() < 0.008, "pre_open_m {pre_w} should be near 0.055 m");

    let j7 = pre_descend_j7.expect("J7 captured during descend phase");
    let expected_j7 = -5.7 * (pre_w / 0.085);
    assert!((j7 - expected_j7).abs() < 0.2, "J7 {j7} before descend should be near {expected_j7} (far from open_rad -5.7)");

    let held_w = a.held_width_m.expect("held_width_m recorded");
    assert!(held_w > 0.02 && held_w < 0.06, "held_width_m {held_w} within object bounds");
}

#[tokio::test]
async fn pre_open_skip_conditions() {
    // 1. PreOpen::Auto without calibration skips with "skipped:uncalibrated"
    {
        let setup_auto = GraspSetup {
            pre_open: armd::grasp::PreOpen::Auto,
            ..GraspSetup::default()
        };
        let r = rig(setup_auto.clone(), centred_lens(), 0.0, [0.0, 0.0], false).await;
        let start_r = start_pose(&r).await.r;
        let mut s = setup_auto.clone();
        s.grip_pixel_norm = calibrate_grip(&r, &setup_auto, &start_r);
        r.sup.inner().write().await.set_grasp_setup(s.clone());
        *r.sim.width_px.lock().unwrap() = Some(160.0);

        let (id, _) = grasp::start(&r.sup, r.sup.capture_admission_stamp(), r.sim.clone(), &tap(&r).await).await.expect("grasp accepted");
        let a = wait_done(&r.sup, id, 30.0, |_| {}).await;
        assert_eq!(a.pre_open.as_deref(), Some("skipped:uncalibrated"));
    }

    // 2. PreOpen::Off records "off"
    {
        let setup_off = GraspSetup {
            pre_open: armd::grasp::PreOpen::Off,
            ..GraspSetup::default()
        };
        let r = rig(setup_off.clone(), centred_lens(), 0.0, [0.0, 0.0], false).await;
        let start_r = start_pose(&r).await.r;
        let mut s = setup_off.clone();
        s.grip_pixel_norm = calibrate_grip(&r, &setup_off, &start_r);
        r.sup.inner().write().await.set_grasp_setup(s.clone());
        *r.sim.width_px.lock().unwrap() = Some(160.0);

        let (id, _) = grasp::start(&r.sup, r.sup.capture_admission_stamp(), r.sim.clone(), &tap(&r).await).await.expect("grasp accepted");
        let a = wait_done(&r.sup, id, 30.0, |_| {}).await;
        assert_eq!(a.pre_open.as_deref(), Some("off"));
    }

    // 3. PreOpen::On with width reaching/exceeding open_rad skips with "skipped:open_rad"
    // At approach height 0.15 m, fx 600, 310 px -> ~0.077 m + 0.015 margin = 0.092 m >= 0.085 m (open_rad)
    {
        let setup_wide = GraspSetup {
            pre_open: armd::grasp::PreOpen::On,
            pre_open_margin_m: 0.015,
            max_grasp_width_m: 0.10, // allow wider object to pass TOO_WIDE
            ..GraspSetup::default()
        };
        let r = rig(setup_wide.clone(), centred_lens(), 0.0, [0.0, 0.0], false).await;
        let start_r = start_pose(&r).await.r;
        let mut s = setup_wide.clone();
        s.grip_pixel_norm = calibrate_grip(&r, &setup_wide, &start_r);
        r.sup.inner().write().await.set_grasp_setup(s.clone());
        *r.sim.width_px.lock().unwrap() = Some(310.0);

        let (id, _) = grasp::start(&r.sup, r.sup.capture_admission_stamp(), r.sim.clone(), &tap(&r).await).await.expect("grasp accepted");
        let a = wait_done(&r.sup, id, 30.0, |_| {}).await;
        assert_eq!(a.pre_open.as_deref(), Some("skipped:open_rad"));
    }

    // 4. No width reported -> "skipped:no_width"
    {
        let setup_nowidth = GraspSetup {
            pre_open: armd::grasp::PreOpen::On,
            ..GraspSetup::default()
        };
        let r = rig(setup_nowidth.clone(), centred_lens(), 0.0, [0.0, 0.0], false).await;
        let start_r = start_pose(&r).await.r;
        let mut s = setup_nowidth.clone();
        s.grip_pixel_norm = calibrate_grip(&r, &setup_nowidth, &start_r);
        r.sup.inner().write().await.set_grasp_setup(s.clone());
        *r.sim.width_px.lock().unwrap() = None;

        let (id, _) = grasp::start(&r.sup, r.sup.capture_admission_stamp(), r.sim.clone(), &tap(&r).await).await.expect("grasp accepted");
        let a = wait_done(&r.sup, id, 30.0, |_| {}).await;
        assert_eq!(a.pre_open.as_deref(), Some("skipped:no_width"));
    }
}

#[tokio::test]
async fn calibrated_too_wide_limit() {
    let setup = GraspSetup {
        max_grasp_width_m: 0.075,
        pre_open_margin_m: 0.015,
        ..GraspSetup::default()
    };
    let r = rig(setup.clone(), centred_lens(), 0.0, [0.0, 0.0], false).await;
    let start_r = start_pose(&r).await.r;
    let mut s = setup.clone();
    s.grip_pixel_norm = calibrate_grip(&r, &setup, &start_r);
    r.sup.inner().write().await.set_grasp_setup(s.clone());

    // Set calibrated jaw opening: open_rad -5.7 gives 0.080 m opening.
    // Calibrated limit = 0.080 - 0.015 = 0.065 m (below max_grasp_width_m 0.075).
    {
        let mut inner = r.sup.inner().write().await;
        if let Some(p) = inner.config.gripper_policy.as_mut() {
            p.jaw_opening = Some(armd::model::JawOpening {
                points: vec![[0.0, 0.0], [-5.7, 0.080]],
                calibrated: true,
            });
        }
    }

    // Width is 275 px -> ~0.069 m (between 0.065 and 0.075)
    *r.sim.angle.lock().unwrap() = Some((0.0, 0.9));
    *r.sim.width_px.lock().unwrap() = Some(275.0);

    let (id, _) = grasp::start(&r.sup, r.sup.capture_admission_stamp(), r.sim.clone(), &tap(&r).await).await.expect("grasp accepted");
    let a = wait_done(&r.sup, id, 30.0, |_| {}).await;

    assert_eq!(a.state, "failed");
    assert_eq!(a.code.as_deref(), Some("TOO_WIDE"));
    let measured_w = a.grasp_width_m.expect("width recorded");
    assert!(measured_w > 0.065 && measured_w < 0.075, "width {measured_w} between calib limit 0.065 and max 0.075");
}

#[tokio::test]
async fn grasp_records_focus_calls_and_clears_at_end() {
    let setup = GraspSetup { camera_tilt_deg: -20.0, ..GraspSetup::default() };
    let r = rig(setup.clone(), Truth::same(), 0.0, [0.0, 0.0], false).await;
    let start_r = start_pose(&r).await.r;
    let mut s = setup.clone();
    s.grip_pixel_norm = calibrate_grip(&r, &setup, &start_r);
    r.sup.inner().write().await.set_grasp_setup(s.clone());

    let body = tap(&r).await;
    let b = body["box_norm"].as_array().unwrap();
    let tapped_box = [
        b[0].as_f64().unwrap(),
        b[1].as_f64().unwrap(),
        b[2].as_f64().unwrap(),
        b[3].as_f64().unwrap(),
    ];

    let (id, _) = grasp::start(&r.sup, r.sup.capture_admission_stamp(), r.sim.clone(), &body).await.expect("grasp accepted");
    let a = wait_done(&r.sup, id, 30.0, |_| {}).await;
    assert_eq!(a.state, "done");

    let calls = r.sim.focus_calls.lock().unwrap().clone();
    assert!(!calls.is_empty(), "focus calls should be recorded");
    // First focus call is set at centring start with the tapped box and label
    assert_eq!(calls[0].0.as_deref(), Some("cup"));
    assert_eq!(calls[0].1, tapped_box);
    assert_eq!(calls[0].2, 2000);
    // Last focus call clears focus with ttl 0
    assert_eq!(calls.last().unwrap().2, 0);
}

#[tokio::test]
async fn grasp_completes_when_mask_focus_fails() {
    let setup = GraspSetup { camera_tilt_deg: -20.0, ..GraspSetup::default() };
    let r = rig(setup.clone(), Truth::same(), 0.0, [0.0, 0.0], false).await;
    let start_r = start_pose(&r).await.r;
    let mut s = setup.clone();
    s.grip_pixel_norm = calibrate_grip(&r, &setup, &start_r);
    r.sup.inner().write().await.set_grasp_setup(s.clone());

    // Simulate focus failure (e.g. older detector returning error)
    r.sim.focus_error.store(true, Ordering::SeqCst);

    let (id, _) = grasp::start(&r.sup, r.sup.capture_admission_stamp(), r.sim.clone(), &tap(&r).await).await.expect("grasp accepted");
    let a = wait_done(&r.sup, id, 30.0, |_| {}).await;
    assert_eq!(a.state, "done");
    assert_eq!(a.holding, Some(true));
}

#[tokio::test]
async fn depth_oblique_ransac_skips_probe_and_grasps() {
    let setup = GraspSetup { camera_tilt_deg: 0.0, height_sources: DEPTH_DECIDES.to_vec(), ..GraspSetup::default() };
    let rig = rig(setup.clone(), Truth::same(), 20.0, [0.0, 0.0], false).await;
    let start_r = start_pose(&rig).await.r;
    let mut s = setup.clone();
    s.grip_pixel_norm = calibrate_grip(&rig, &setup, &start_r);
    rig.sup.inner().write().await.set_grasp_setup(s.clone());

    rig.sim.per_request.store(true, Ordering::SeqCst);
    rig.sim.depth_available.store(true, Ordering::SeqCst);
    *rig.sim.width_px.lock().unwrap() = Some(40.0);
    *rig.sim.depth_sim.lock().unwrap() = Some(DepthSim {
        metric_model: false,
        a0: 2.7,
        b0: 0.45,
        occluded_table: false,
        occluded_above_z: None,
        depth_conf: Some(0.95),
        jaw_clear: Some(true),
        length_px: Some(65.0),
        drift_per_frame: false,
    });

    let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.expect("grasp accepted");
    let a = wait_done(&rig.sup, id, 30.0, |_| {}).await;
    assert_eq!(a.state, "done");
    assert_eq!(a.holding, Some(true));
    assert_eq!(a.probe_dz_m, None, "mechanical height probe should be skipped");
    assert_eq!(a.height_source.as_deref(), Some("depth"));
    assert_eq!(a.depth_anchor.as_deref(), Some("oblique_ransac"));
    assert!(
        (a.object_height_m.unwrap() as f64 - rig.sim.object.height).abs() < 0.005,
        "object height error exceeds 5 mm: {:?}",
        a.object_height_m
    );
    assert!(a.object_length_m.is_some(), "object_length_m should be present");
    assert!(a.grasp_width_m.is_some(), "grasp_width_m should be present");
    assert_eq!(a.jaw_clear, Some(true));
}

#[tokio::test]
async fn depth_metric_model_vertical_and_occluded() {
    let setup = GraspSetup { camera_tilt_deg: 0.0, height_sources: DEPTH_DECIDES.to_vec(), ..GraspSetup::default() };
    let rig1 = rig(setup.clone(), Truth::same(), 0.0, [0.0, 0.0], false).await;
    let start_r1 = start_pose(&rig1).await.r;
    let mut s1 = setup.clone();
    s1.grip_pixel_norm = calibrate_grip(&rig1, &setup, &start_r1);
    rig1.sup.inner().write().await.set_grasp_setup(s1.clone());

    rig1.sim.per_request.store(true, Ordering::SeqCst);
    rig1.sim.depth_available.store(true, Ordering::SeqCst);
    *rig1.sim.width_px.lock().unwrap() = Some(35.0);
    *rig1.sim.depth_sim.lock().unwrap() = Some(DepthSim {
        metric_model: true,
        a0: 1.0,
        b0: 0.0,
        occluded_table: false,
        occluded_above_z: None,
        depth_conf: Some(0.95),
        jaw_clear: Some(true),
        length_px: Some(60.0),
        drift_per_frame: false,
    });

    let (id, _) = grasp::start(&rig1.sup, rig1.sup.capture_admission_stamp(), rig1.sim.clone(), &tap(&rig1).await).await.expect("grasp accepted");
    let a = wait_done(&rig1.sup, id, 30.0, |_| {}).await;
    assert_eq!(a.state, "done");
    assert_eq!(a.holding, Some(true));
    assert_eq!(a.probe_dz_m, None);
    assert_eq!(a.height_source.as_deref(), Some("depth"));
    assert_eq!(a.depth_anchor.as_deref(), Some("metric_table"));
    assert!((a.object_height_m.unwrap() as f64 - rig1.sim.object.height).abs() < 0.005);

    // 2. Occluded table: no depth calibration, the box-scale probe measures the height
    let rig2 = rig(setup.clone(), Truth::same(), 0.0, [0.0, 0.0], false).await;
    let start_r2 = start_pose(&rig2).await.r;
    let mut s2 = setup.clone();
    s2.grip_pixel_norm = calibrate_grip(&rig2, &setup, &start_r2);
    rig2.sup.inner().write().await.set_grasp_setup(s2.clone());

    rig2.sim.per_request.store(true, Ordering::SeqCst);
    rig2.sim.depth_available.store(true, Ordering::SeqCst);
    *rig2.sim.width_px.lock().unwrap() = Some(35.0);
    *rig2.sim.depth_sim.lock().unwrap() = Some(DepthSim {
        metric_model: true,
        a0: 1.0,
        b0: 0.0,
        occluded_table: true,
        occluded_above_z: None,
        depth_conf: Some(0.95),
        jaw_clear: Some(true),
        length_px: Some(60.0),
        drift_per_frame: false,
    });

    let (id2, _) = grasp::start(&rig2.sup, rig2.sup.capture_admission_stamp(), rig2.sim.clone(), &tap(&rig2).await).await.expect("grasp accepted");
    let a2 = wait_done(&rig2.sup, id2, 30.0, |_| {}).await;
    assert_eq!(a2.state, "done");
    assert_eq!(a2.holding, Some(true));
    assert!(a2.probe_dz_m.is_some(), "probe_height should run when the table is occluded");
    assert_eq!(a2.height_source.as_deref(), Some("scale"));
    assert!(a2.depth_anchor.is_none());

    // 3. TOO_WIDE rejection with wide grasp_width_px
    let rig3 = rig(setup.clone(), Truth::same(), 0.0, [0.0, 0.0], false).await;
    let start_r3 = start_pose(&rig3).await.r;
    let mut s3 = setup.clone();
    s3.grip_pixel_norm = calibrate_grip(&rig3, &setup, &start_r3);
    s3.max_grasp_width_m = 0.02; // 20 mm max
    rig3.sup.inner().write().await.set_grasp_setup(s3.clone());

    rig3.sim.per_request.store(true, Ordering::SeqCst);
    rig3.sim.depth_available.store(true, Ordering::SeqCst);
    *rig3.sim.width_px.lock().unwrap() = Some(250.0);
    *rig3.sim.depth_sim.lock().unwrap() = Some(DepthSim {
        metric_model: true,
        a0: 1.0,
        b0: 0.0,
        occluded_table: false,
        occluded_above_z: None,
        depth_conf: Some(0.95),
        jaw_clear: Some(true),
        length_px: Some(60.0),
        drift_per_frame: false,
    });

    let (id3, _) = grasp::start(&rig3.sup, rig3.sup.capture_admission_stamp(), rig3.sim.clone(), &tap(&rig3).await).await.expect("grasp accepted");
    let a3 = wait_done(&rig3.sup, id3, 30.0, |_| {}).await;
    assert_eq!(a3.state, "failed");
    assert_eq!(a3.code.as_deref(), Some("TOO_WIDE"));
}

#[tokio::test]
async fn depth_unanchored_or_low_conf_falls_back_to_probe_height() {
    let setup = GraspSetup { camera_tilt_deg: 0.0, height_sources: DEPTH_DECIDES.to_vec(), height_probe_m: 0.04, ..GraspSetup::default() };

    // 1. Low confidence depth (< 0.35 threshold) falls back to probe_height
    let rig1 = rig(setup.clone(), Truth::same(), 0.0, [0.0, 0.0], false).await;
    let start_r1 = start_pose(&rig1).await.r;
    let mut s1 = setup.clone();
    s1.grip_pixel_norm = calibrate_grip(&rig1, &setup, &start_r1);
    rig1.sup.inner().write().await.set_grasp_setup(s1.clone());

    rig1.sim.per_request.store(true, Ordering::SeqCst);
    rig1.sim.depth_available.store(true, Ordering::SeqCst);
    *rig1.sim.depth_sim.lock().unwrap() = Some(DepthSim {
        metric_model: true,
        a0: 1.0,
        b0: 0.0,
        occluded_table: false,
        occluded_above_z: None,
        depth_conf: Some(0.20),
        jaw_clear: Some(true),
        length_px: Some(60.0),
        drift_per_frame: false,
    });

    let (id1, _) = grasp::start(&rig1.sup, rig1.sup.capture_admission_stamp(), rig1.sim.clone(), &tap(&rig1).await).await.expect("grasp accepted");
    let a1 = wait_done(&rig1.sup, id1, 30.0, |_| {}).await;
    assert_eq!(a1.state, "done");
    assert_eq!(a1.holding, Some(true));
    assert_eq!(a1.height_source.as_deref(), Some("scale"));
    assert!(a1.probe_dz_m.is_some(), "mechanical probe_height should have run");

    // 2. Unanchored depth (metric_model = false, vertical pose, occluded table -> calibration fails) falls back to probe_height
    let rig2 = rig(setup.clone(), Truth::same(), 0.0, [0.0, 0.0], false).await;
    let start_r2 = start_pose(&rig2).await.r;
    let mut s2 = setup.clone();
    s2.grip_pixel_norm = calibrate_grip(&rig2, &setup, &start_r2);
    rig2.sup.inner().write().await.set_grasp_setup(s2.clone());

    rig2.sim.per_request.store(true, Ordering::SeqCst);
    rig2.sim.depth_available.store(true, Ordering::SeqCst);
    *rig2.sim.depth_sim.lock().unwrap() = Some(DepthSim {
        metric_model: false,
        a0: 2.7,
        b0: 0.45,
        occluded_table: true,
        occluded_above_z: None,
        depth_conf: Some(0.95),
        jaw_clear: Some(true),
        length_px: Some(60.0),
        drift_per_frame: false,
    });

    let (id2, _) = grasp::start(&rig2.sup, rig2.sup.capture_admission_stamp(), rig2.sim.clone(), &tap(&rig2).await).await.expect("grasp accepted");
    let a2 = wait_done(&rig2.sup, id2, 30.0, |_| {}).await;
    assert_eq!(a2.state, "done");
    assert_eq!(a2.holding, Some(true));
    assert_eq!(a2.height_source.as_deref(), Some("scale"));
    assert!(a2.probe_dz_m.is_some(), "mechanical probe_height should have run");
}

#[tokio::test]
async fn depth_tall_object_fails_when_the_probe_cannot_measure() {
    let setup = GraspSetup { camera_tilt_deg: 0.0, height_sources: DEPTH_DECIDES.to_vec(), approach_height_m: 0.10, height_probe_m: 0.04, ..GraspSetup::default() };
    let rig = rig(setup.clone(), Truth::same(), 0.0, [0.0, 0.0], false).await;
    let start_r = start_pose(&rig).await.r;
    let mut s = setup.clone();
    s.grip_pixel_norm = calibrate_grip(&rig, &setup, &start_r);
    rig.sup.inner().write().await.set_grasp_setup(s.clone());

    rig.sim.per_request.store(true, Ordering::SeqCst);
    rig.sim.depth_available.store(true, Ordering::SeqCst);
    *rig.sim.top.lock().unwrap() = Some(0.09);
    *rig.sim.depth_sim.lock().unwrap() = Some(DepthSim {
        metric_model: true,
        a0: 1.0,
        b0: 0.0,
        occluded_table: false,
        occluded_above_z: None,
        depth_conf: Some(0.95),
        jaw_clear: Some(true),
        length_px: Some(60.0),
        drift_per_frame: false,
    });

    let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.expect("grasp accepted");
    let a = wait_done(&rig.sup, id, 30.0, |_| {}).await;
    assert_eq!(a.state, "failed");
    assert_eq!(a.code.as_deref(), Some("OBJECT_TOO_TALL"));
    // the probe ran and could not measure (the close object touches the border), so the depth
    // height decides
    assert_eq!(a.height_source, None);
}

#[tokio::test]
async fn depth_guards_lens_unknown_table_conflict_and_drift() {
    let setup = GraspSetup { camera_tilt_deg: 0.0, height_sources: DEPTH_DECIDES.to_vec(), height_probe_m: 0.04, ..GraspSetup::default() };

    // 1. Truth { known_offset: false, ..Truth::same() } (!lens_known) with depth_available = true skips FK depth anchoring
    let rig1 = rig(setup.clone(), Truth { known_offset: false, ..Truth::same() }, 0.0, [0.0, 0.0], false).await;
    let start_r1 = start_pose(&rig1).await.r;
    let mut s1 = setup.clone();
    s1.grip_pixel_norm = calibrate_grip(&rig1, &setup, &start_r1);
    rig1.sup.inner().write().await.set_grasp_setup(s1.clone());

    rig1.sim.per_request.store(true, Ordering::SeqCst);
    rig1.sim.depth_available.store(true, Ordering::SeqCst);
    *rig1.sim.width_px.lock().unwrap() = Some(35.0);
    *rig1.sim.depth_sim.lock().unwrap() = Some(DepthSim {
        metric_model: true,
        a0: 1.0,
        b0: 0.0,
        occluded_table: false,
        occluded_above_z: None,
        depth_conf: Some(0.95),
        jaw_clear: Some(true),
        length_px: Some(60.0),
        drift_per_frame: false,
    });

    let (id1, _) = grasp::start(&rig1.sup, rig1.sup.capture_admission_stamp(), rig1.sim.clone(), &tap(&rig1).await).await.expect("grasp accepted");
    let a1 = wait_done(&rig1.sup, id1, 30.0, |_| {}).await;
    assert_eq!(a1.state, "done");
    assert_eq!(a1.holding, Some(true));
    assert!(a1.depth_anchor.is_none(), "depth_anchor should be None when lens is unknown");

    // 2. metric_model: true, a0: 1.8, occluded_table: false (visible table contradicts FK) falls back to probe_height
    let rig2 = rig(setup.clone(), Truth::same(), 0.0, [0.0, 0.0], false).await;
    let start_r2 = start_pose(&rig2).await.r;
    let mut s2 = setup.clone();
    s2.grip_pixel_norm = calibrate_grip(&rig2, &setup, &start_r2);
    rig2.sup.inner().write().await.set_grasp_setup(s2.clone());

    rig2.sim.per_request.store(true, Ordering::SeqCst);
    rig2.sim.depth_available.store(true, Ordering::SeqCst);
    *rig2.sim.width_px.lock().unwrap() = Some(35.0);
    *rig2.sim.depth_sim.lock().unwrap() = Some(DepthSim {
        metric_model: true,
        a0: 1.8,
        b0: 0.0,
        occluded_table: false,
        occluded_above_z: None,
        depth_conf: Some(0.95),
        jaw_clear: Some(true),
        length_px: Some(60.0),
        drift_per_frame: false,
    });

    let (id2, _) = grasp::start(&rig2.sup, rig2.sup.capture_admission_stamp(), rig2.sim.clone(), &tap(&rig2).await).await.expect("grasp accepted");
    let a2 = wait_done(&rig2.sup, id2, 30.0, |_| {}).await;
    assert_eq!(a2.state, "done");
    assert_eq!(a2.holding, Some(true));
    assert_eq!(a2.height_source.as_deref(), Some("scale"));
    assert!(a2.probe_dz_m.is_some(), "probe_height should have run");

    // 3. Stationary vertical relative model (metric_model: false, a0: 2.7, b0: 0.45, drift_per_frame: false)
    // starting at look_down_start(0.0, 0.26) with approach_height_m: 0.15 calibrates via two_height_fk
    let s3_base = GraspSetup { camera_tilt_deg: 0.0, height_sources: DEPTH_DECIDES.to_vec(), approach_height_m: 0.15, height_probe_m: 0.04, ..GraspSetup::default() };
    let rig3 = rig_mounted(s3_base.clone(), Truth::same(), 0.0, look_down_start(0.0, 0.26), [0.0, 0.0], false).await;
    let start_r3 = start_pose(&rig3).await.r;
    let mut s3 = s3_base.clone();
    s3.grip_pixel_norm = calibrate_grip(&rig3, &s3_base, &start_r3);
    rig3.sup.inner().write().await.set_grasp_setup(s3.clone());

    rig3.sim.per_request.store(true, Ordering::SeqCst);
    rig3.sim.depth_available.store(true, Ordering::SeqCst);
    *rig3.sim.width_px.lock().unwrap() = Some(35.0);
    *rig3.sim.depth_sim.lock().unwrap() = Some(DepthSim {
        metric_model: false,
        a0: 2.7,
        b0: 0.45,
        occluded_table: false,
        occluded_above_z: None,
        depth_conf: Some(0.95),
        jaw_clear: Some(true),
        length_px: Some(60.0),
        drift_per_frame: false,
    });

    let (id3, _) = grasp::start(&rig3.sup, rig3.sup.capture_admission_stamp(), rig3.sim.clone(), &tap(&rig3).await).await.expect("grasp accepted");
    let a3 = wait_done(&rig3.sup, id3, 30.0, |_| {}).await;
    assert_eq!(a3.state, "done");
    assert_eq!(a3.holding, Some(true));
    assert_eq!(a3.depth_anchor.as_deref(), Some("two_height_fk"));
    assert!(
        (a3.object_height_m.unwrap() as f64 - rig3.sim.object.height).abs() < 0.005,
        "two_height_fk height error: {:?}",
        a3.object_height_m
    );

    // whereas drift_per_frame: true rejects two_height_fk and falls back to probe_height
    let rig4 = rig_mounted(s3_base.clone(), Truth::same(), 0.0, look_down_start(0.0, 0.26), [0.0, 0.0], false).await;
    let start_r4 = start_pose(&rig4).await.r;
    let mut s4 = s3_base.clone();
    s4.grip_pixel_norm = calibrate_grip(&rig4, &s3_base, &start_r4);
    rig4.sup.inner().write().await.set_grasp_setup(s4.clone());

    rig4.sim.per_request.store(true, Ordering::SeqCst);
    rig4.sim.depth_available.store(true, Ordering::SeqCst);
    *rig4.sim.width_px.lock().unwrap() = Some(35.0);
    *rig4.sim.depth_sim.lock().unwrap() = Some(DepthSim {
        metric_model: false,
        a0: 2.7,
        b0: 0.45,
        occluded_table: false,
        occluded_above_z: None,
        depth_conf: Some(0.95),
        jaw_clear: Some(true),
        length_px: Some(60.0),
        drift_per_frame: true,
    });

    let (id4, _) = grasp::start(&rig4.sup, rig4.sup.capture_admission_stamp(), rig4.sim.clone(), &tap(&rig4).await).await.expect("grasp accepted");
    let a4 = wait_done(&rig4.sup, id4, 30.0, |_| {}).await;
    assert_eq!(a4.state, "done");
    assert_eq!(a4.holding, Some(true));
    assert_eq!(a4.height_source.as_deref(), Some("scale"));
    assert!(a4.probe_dz_m.is_some(), "probe_height should have run on drift");

    // 5. metric_model: true, a0: 1.8, occluded_above_z: Some(0.20) starting at look_down_start(0.0, 0.26)
    // with approach_height_m: 0.15 (frame0 at z = 0.26 is occluded and gives no calibration,
    // overhead_frame at z = 0.15 sees conflicting table -> falls back to probe_height)
    let s5_base = GraspSetup { camera_tilt_deg: 0.0, height_sources: DEPTH_DECIDES.to_vec(), approach_height_m: 0.15, height_probe_m: 0.04, ..GraspSetup::default() };
    let rig5 = rig_mounted(s5_base.clone(), Truth::same(), 0.0, look_down_start(0.0, 0.26), [0.0, 0.0], false).await;
    let start_r5 = start_pose(&rig5).await.r;
    let mut s5 = s5_base.clone();
    s5.grip_pixel_norm = calibrate_grip(&rig5, &s5_base, &start_r5);
    rig5.sup.inner().write().await.set_grasp_setup(s5.clone());

    rig5.sim.per_request.store(true, Ordering::SeqCst);
    rig5.sim.depth_available.store(true, Ordering::SeqCst);
    *rig5.sim.width_px.lock().unwrap() = Some(35.0);
    *rig5.sim.depth_sim.lock().unwrap() = Some(DepthSim {
        metric_model: true,
        a0: 1.8,
        b0: 0.0,
        occluded_table: false,
        occluded_above_z: Some(0.20),
        depth_conf: Some(0.95),
        jaw_clear: Some(true),
        length_px: Some(60.0),
        drift_per_frame: false,
    });

    let (id5, _) = grasp::start(&rig5.sup, rig5.sup.capture_admission_stamp(), rig5.sim.clone(), &tap(&rig5).await).await.expect("grasp accepted");
    let a5 = wait_done(&rig5.sup, id5, 30.0, |_| {}).await;
    assert_eq!(a5.state, "done");
    assert_eq!(a5.holding, Some(true));
    assert!(a5.depth_anchor.is_none());
    assert_eq!(a5.height_source.as_deref(), Some("scale"));
    assert!(a5.probe_dz_m.is_some(), "probe_height should have run when table contradicted prior");
}

/// e537a3f behaviour: a depth height decides before the probe.
const DEPTH_DECIDES: [HeightSource; 2] = [HeightSource::Depth, HeightSource::Probe];

fn metric_depth() -> DepthSim {
    DepthSim { metric_model: true, a0: 1.0, b0: 0.0, occluded_table: false, occluded_above_z: None, depth_conf: Some(0.95), jaw_clear: Some(true), length_px: Some(60.0), drift_per_frame: false }
}

/// Vertical start, triggered frames with depth inference, grip pixel calibrated.
async fn depth_rig(setup: GraspSetup, depth: DepthSim, width_px: Option<f64>, top: Option<f64>) -> Rig {
    let rig = rig(setup.clone(), Truth::same(), 0.0, [0.0, 0.0], false).await;
    let start_r = start_pose(&rig).await.r;
    let mut s = setup.clone();
    s.grip_pixel_norm = calibrate_grip(&rig, &setup, &start_r);
    rig.sup.inner().write().await.set_grasp_setup(s);
    rig.sim.per_request.store(true, Ordering::SeqCst);
    rig.sim.depth_available.store(true, Ordering::SeqCst);
    *rig.sim.width_px.lock().unwrap() = width_px;
    *rig.sim.top.lock().unwrap() = top;
    *rig.sim.depth_sim.lock().unwrap() = Some(depth);
    rig
}

async fn run_to_end(rig: &Rig) -> ActionStatus {
    let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(rig).await).await.expect("grasp accepted");
    wait_done(&rig.sup, id, 30.0, |_| {}).await
}

/// (source, value, used, record_only, reason) of every height estimate.
fn estimates(a: &ActionStatus) -> Vec<(String, Option<f32>, bool, bool, Option<String>)> {
    a.height_estimates.iter().map(|e: &EstimateEntry| (e.source.clone(), e.value, e.used, e.record_only, e.reason.clone())).collect()
}

#[tokio::test]
async fn default_setup_records_depth_and_the_probe_decides() {
    let rig = depth_rig(GraspSetup { camera_tilt_deg: 0.0, ..GraspSetup::default() }, metric_depth(), Some(35.0), None).await;
    let a = run_to_end(&rig).await;
    println!("record-only depth: {} {:?}, {:?} {:?}, estimates {:?}", a.state, a.code, a.height_source, a.object_height_m, a.height_estimates);
    assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{a:?}");
    assert_eq!(a.height_source.as_deref(), Some("scale"));
    assert!(a.probe_dz_m.is_some(), "the probe decides");
    // record-only depth still reports its measurement
    assert_eq!(a.depth_anchor.as_deref(), Some("metric_table"));
    let e = estimates(&a);
    assert_eq!(e.len(), 2, "{e:?}");
    assert_eq!((e[0].0.as_str(), e[0].2, e[0].3, e[0].4.as_deref()), ("depth:metric_table", false, true, None));
    assert!((f64::from(e[0].1.unwrap()) - rig.sim.object.height).abs() < 0.005, "{e:?}");
    assert_eq!((e[1].0.as_str(), e[1].2, e[1].3), ("scale", true, false));
    assert_eq!(e[1].1, a.object_height_m);
}

#[tokio::test]
async fn height_selection_follows_the_order_of_height_sources() {
    let probe_first = GraspSetup { camera_tilt_deg: 0.0, height_sources: vec![HeightSource::Probe, HeightSource::Depth], ..GraspSetup::default() };
    let a = run_to_end(&depth_rig(probe_first, metric_depth(), Some(35.0), None).await).await;
    assert_eq!((a.state.as_str(), a.height_source.as_deref()), ("done", Some("scale")), "{a:?}");
    assert!(a.probe_dz_m.is_some());
    // depth comes after the accepted probe: not evaluated, not reported
    assert_eq!(estimates(&a).iter().map(|e| e.0.as_str()).collect::<Vec<_>>(), ["scale"]);
    assert_eq!(a.depth_anchor, None);

    let depth_first = GraspSetup { camera_tilt_deg: 0.0, height_sources: DEPTH_DECIDES.to_vec(), ..GraspSetup::default() };
    let a = run_to_end(&depth_rig(depth_first, metric_depth(), Some(35.0), None).await).await;
    assert_eq!((a.state.as_str(), a.height_source.as_deref()), ("done", Some("depth")), "{a:?}");
    assert_eq!(a.probe_dz_m, None);
    let e = estimates(&a);
    assert_eq!((e.len(), e[0].0.as_str(), e[0].2, e[0].3), (1, "depth:metric_table", true, false), "{e:?}");
}

#[tokio::test]
async fn prior_decides_when_the_probe_is_disabled() {
    let (rig, s) = height_rig(0.08, json!({"height_probe_m": 0.0, "height_sources": ["probe", "prior"], "height_priors": {"CUP": 0.08}})).await;
    let (a, close_z) = run_grasp_close_z(&rig).await;
    println!("prior: {} {:?}, {:?} {:?}, close {:?}, TCP z at close {close_z:.4}, estimates {:?}", a.state, a.code, a.height_source, a.object_height_m, a.close_height_m, a.height_estimates);
    assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{a:?}");
    assert_eq!((a.height_source.as_deref(), a.object_height_m, a.height_reason.as_deref()), (Some("prior"), Some(0.08), None));
    assert!((f64::from(a.close_height_m.unwrap()) - 0.04).abs() < 1e-6, "{a:?}");
    assert!((close_z - (s.table_z_m + 0.04)).abs() < 2e-3, "TCP at the close height: {close_z}");
    assert_eq!(estimates(&a), [
        ("depth".to_string(), None, false, true, Some("unavailable".to_string())),
        ("probe".to_string(), None, false, false, Some("disabled".to_string())),
        ("prior".to_string(), Some(0.08), true, false, None),
    ]);
}

#[tokio::test]
async fn record_only_too_tall_prior_is_ignored() {
    let (rig, _) = height_rig(0.08, json!({"height_record": ["depth", "prior"], "height_priors": {"cup": 0.3}})).await;
    let (a, _) = run_grasp_close_z(&rig).await;
    assert_eq!((a.state.as_str(), a.holding, a.height_source.as_deref()), ("done", Some(true), Some("scale")), "{a:?}");
    let e = estimates(&a);
    assert_eq!(e[1], ("prior".to_string(), Some(0.3), false, true, Some("OBJECT_TOO_TALL".to_string())), "{e:?}");
    assert_eq!((e[2].0.as_str(), e[2].2), ("scale", true));
}

/// The depth_tall_object_fails_when_the_probe_cannot_measure scene with depth record-only: the
/// too-tall depth height does not end the grasp; the probe cannot measure, the grasp goes on at
/// grasp_height_m. The width check is the same as with depth deciding.
#[tokio::test]
async fn record_only_too_tall_depth_falls_back_to_the_setup_height() {
    let setup = GraspSetup { camera_tilt_deg: 0.0, approach_height_m: 0.10, height_probe_m: 0.04, ..GraspSetup::default() };
    let a = run_to_end(&depth_rig(setup.clone(), metric_depth(), None, Some(0.09)).await).await;
    println!("record-only tall depth: {} {:?}, {:?} {:?}, estimates {:?}", a.state, a.code, a.height_source, a.height_reason, a.height_estimates);
    assert_eq!(a.state, "done", "{a:?}");
    assert_eq!(a.height_source.as_deref(), Some("setup"));
    assert_eq!(a.height_reason.as_deref(), Some("border"), "{a:?}");
    let e = estimates(&a);
    assert_eq!((e.len(), e[0].0.as_str(), e[0].2, e[0].3, e[0].4.as_deref()), (2, "depth:metric_table", false, true, Some("OBJECT_TOO_TALL")), "{e:?}");
    assert_eq!(e[1], ("probe".to_string(), None, false, false, Some("border".to_string())));
    assert_eq!(a.depth_anchor.as_deref(), Some("metric_table"));

    let narrow = GraspSetup { max_grasp_width_m: 0.02, ..setup };
    let a = run_to_end(&depth_rig(narrow, metric_depth(), Some(250.0), Some(0.09)).await).await;
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some("TOO_WIDE")), "{a:?}");
    assert!(a.grasp_width_m.is_some_and(|w| w > 0.02), "{a:?}");
}

#[tokio::test]
async fn stop_during_the_probe_ends_the_grasp_with_the_estimates_so_far() {
    let (rig, _) = height_rig(0.08, json!({})).await;
    let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.expect("grasp accepted");
    wait_phase(&rig.sup, id, "probe", Some(true)).await;
    {
        let mut inner = rig.sup.inner().write().await;
        inner.stop_motion();
        inner.request_macro_stop();
    }
    let a = wait_done(&rig.sup, id, 5.0, |_| {}).await;
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("cancelled", Some("STOPPED")), "{a:?}");
    assert_eq!(a.height_source, None);
    assert_eq!(estimates(&a), [("depth".to_string(), None, false, true, Some("unavailable".to_string()))]);
    assert_idle_after(&rig.sup, id).await;
}

/// Depth rig of `depth_oblique_ransac_skips_probe_and_grasps` with a setup patch.
async fn depth_estimates_rig(setup: GraspSetup, occluded_table: bool, metric_model: bool) -> Rig {
    let rig = rig(setup.clone(), Truth::same(), 20.0, [0.0, 0.0], false).await;
    let start_r = start_pose(&rig).await.r;
    let mut s = setup.clone();
    s.grip_pixel_norm = calibrate_grip(&rig, &setup, &start_r);
    rig.sup.inner().write().await.set_grasp_setup(s);
    rig.sim.per_request.store(true, Ordering::SeqCst);
    rig.sim.depth_available.store(true, Ordering::SeqCst);
    *rig.sim.width_px.lock().unwrap() = Some(40.0);
    *rig.sim.depth_sim.lock().unwrap() = Some(DepthSim {
        metric_model,
        a0: if metric_model { 1.0 } else { 2.7 },
        b0: if metric_model { 0.0 } else { 0.45 },
        occluded_table,
        occluded_above_z: None,
        depth_conf: Some(0.95),
        jaw_clear: Some(true),
        length_px: Some(65.0),
        drift_per_frame: false,
    });
    rig
}

fn entries(e: &[EstimateEntry]) -> Vec<(&str, bool, bool, Option<&str>)> {
    e.iter().map(|e| (e.source.as_str(), e.used, e.record_only, e.reason.as_deref())).collect()
}

#[tokio::test]
async fn yaw_and_width_estimates_follow_the_source_order() {
    // default: the mask width decides; the depth yaw is recorded (the mock reports no depth angle)
    let setup = GraspSetup { camera_tilt_deg: 0.0, height_sources: DEPTH_DECIDES.to_vec(), ..GraspSetup::default() };
    let rig = depth_estimates_rig(setup.clone(), false, false).await;
    let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.expect("grasp accepted");
    let a = wait_done(&rig.sup, id, 30.0, |_| {}).await;
    println!("default: {} align {:?} yaw {:?} width {:?} grasp_width_m {:?}", a.state, a.align, a.yaw_estimates, a.width_estimates, a.grasp_width_m);
    assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{a:?}");
    assert_eq!(entries(&a.width_estimates), [("mask", true, false, None)]);
    assert_eq!(a.width_estimates[0].value, a.grasp_width_m);
    assert_eq!(entries(&a.yaw_estimates)[0], ("depth", false, true, Some("no_angle")));
    let mask_w = f64::from(a.grasp_width_m.unwrap());

    // width_sources [depth], width_record [mask]: the depth measurement's width decides (the detector's width
    // at the measured depth); the mask width is recorded
    let setup = GraspSetup { width_sources: vec![grasp::WidthSource::Depth], width_record: vec![grasp::WidthSource::Mask], ..setup };
    let rig = depth_estimates_rig(setup, false, false).await;
    let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.expect("grasp accepted");
    let a = wait_done(&rig.sup, id, 30.0, |_| {}).await;
    println!("depth first: {} width {:?} grasp_width_m {:?}", a.state, a.width_estimates, a.grasp_width_m);
    assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{a:?}");
    assert_eq!(entries(&a.width_estimates), [("mask", false, true, None), ("depth", true, false, None)]);
    let depth_w = f64::from(a.grasp_width_m.unwrap());
    assert!((depth_w - mask_w).abs() < 0.01, "depth width {depth_w} vs mask width {mask_w}");
}

#[tokio::test]
async fn depth_entry_reports_why_there_is_no_measurement() {
    // a metric model with the table occluded: fewer than 3 table samples
    let setup = GraspSetup { camera_tilt_deg: 0.0, ..GraspSetup::default() };
    let rig = depth_estimates_rig(setup, true, true).await;
    let (id, _) = grasp::start(&rig.sup, rig.sup.capture_admission_stamp(), rig.sim.clone(), &tap(&rig).await).await.expect("grasp accepted");
    let a = wait_done(&rig.sup, id, 30.0, |_| {}).await;
    println!("occluded: {} height {:?}", a.state, a.height_estimates);
    assert_eq!(a.state, "done", "{a:?}");
    let dep = a.height_estimates.iter().find(|e| e.source.starts_with("depth")).expect("depth entry");
    assert_eq!((dep.source.as_str(), dep.reason.as_deref(), dep.record_only), ("depth", Some("too_few_samples"), true));
}

// ---- grasp tasks (GraspSetup.tasks, request `task`, TASK_UNSUPPORTED) ----

/// The setup with the documentation's example tasks: `cup` needs a segmentation model,
/// `flat` a metric depth model, `plain` needs nothing.
fn with_tasks(mut s: GraspSetup) -> GraspSetup {
    s.tasks = serde_json::from_value(json!({
        "cup": {"description": "upright cup, mask yaw", "requires": {"model_kind": "segment"},
                "overrides": {"yaw_sources": ["mask"], "height_priors": {"cup": 0.095}, "height_sources": ["probe", "prior"]}},
        "flat": {"description": "thin part, depth height", "requires": {"depth": "metric"},
                 "overrides": {"height_sources": ["depth", "probe"], "yaw_sources": ["mask", "depth", "box"]}},
        "plain": {"overrides": {"max_grasp_height_m": 0.05}}
    })).unwrap();
    s.validate().unwrap();
    s
}

/// [`ws_rig`] with the example tasks stored, the grip pixel calibrated and perception API 2.
async fn task_rig(setup_path: Option<std::path::PathBuf>) -> WsRig {
    let r = rig(GraspSetup::default(), Truth::same(), 0.0, [0.0, 0.0], true).await;
    let start_r = start_pose(&r).await.r;
    let mut s = with_tasks(GraspSetup::default());
    s.grip_pixel_norm = calibrate_grip(&r, &s, &start_r);
    r.sup.inner().write().await.set_grasp_setup(s);
    r.sim.per_request.store(true, Ordering::SeqCst);
    ws_rig_on(r, setup_path).await
}

#[tokio::test]
async fn task_requirement_mismatch_rejects_the_command_without_motion() {
    let mut w = task_rig(None).await;
    *w.rig.sim.model_kind.lock().unwrap() = Some("detect".into());
    let q0 = w.rig.sup.get_observation().await.commanded_positions;
    // model_kind: a detection model for the segment task
    let mut body = tap(&w.rig).await;
    body["task"] = json!("cup");
    let reply = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", body.clone()).await;
    assert_eq!((reply["type"].as_str(), reply["code"].as_str(), reply["status"].as_u64()), (Some("command_reject"), Some("TASK_UNSUPPORTED"), Some(422)), "{reply}");
    assert_eq!(reply["message"], "task cup needs model_kind segment, provider has detect", "{reply}");
    // depth: no depth pipeline, then a relative one, for the metric task
    body["task"] = json!("flat");
    let reply = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", body.clone()).await;
    assert_eq!(reply["code"], "TASK_UNSUPPORTED", "{reply}");
    assert!(reply["message"].as_str().unwrap().contains("no depth pipeline"), "{reply}");
    w.rig.sim.depth_available.store(true, Ordering::SeqCst);
    let reply = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", body.clone()).await;
    assert_eq!(reply["code"], "TASK_UNSUPPORTED", "{reply}");
    assert!(reply["message"].as_str().unwrap().contains("not metric"), "{reply}");
    // the in-process start rejects the same way
    let err = grasp::start(&w.rig.sup, w.rig.sup.capture_admission_stamp(), w.rig.sim.clone(), &body).await.unwrap_err();
    assert_eq!(err.code.as_deref(), Some("TASK_UNSUPPORTED"), "{err:?}");
    // one mode reading per command, no action, nothing moved
    assert_eq!(w.rig.sim.mode_info_calls.load(Ordering::SeqCst), 4);
    let obs = w.rig.sup.get_observation().await;
    assert!(obs.action.is_none(), "no action started: {:?}", obs.action);
    assert_eq!(obs.commanded_positions, q0, "no motion commanded");
    assert!(w.rig.sim.trigger_calls.lock().unwrap().is_empty(), "no detection triggered");
}

/// The detector slot empty: a provider that still reports `model.kind` `detect` (older
/// rebot-vision) does not satisfy a task that needs a model kind; depth-only tasks are not affected.
#[tokio::test]
async fn task_model_kind_without_a_loaded_detector_is_unsupported() {
    let mut w = task_rig(None).await;
    *w.rig.sim.model_kind.lock().unwrap() = Some("detect".into());
    w.rig.sim.no_yolo.store(true, Ordering::SeqCst);
    {
        let mut inner = w.rig.sup.inner().write().await;
        let mut s = inner.grasp_setup().unwrap();
        s.tasks.insert("det".into(), serde_json::from_value(json!({"requires": {"model_kind": "detect"}})).unwrap());
        inner.set_grasp_setup(s);
    }
    let mut body = tap(&w.rig).await;
    for task in ["cup", "det"] {
        body["task"] = json!(task);
        let reply = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", body.clone()).await;
        assert_eq!((reply["code"].as_str(), reply["status"].as_u64()), (Some("TASK_UNSUPPORTED"), Some(422)), "{reply}");
        let kind = if task == "cup" { "segment" } else { "detect" };
        assert_eq!(reply["message"], format!("task {task} needs model_kind {kind}, provider has no detector loaded"), "{reply}");
    }
    assert!(w.rig.sup.get_observation().await.action.is_none(), "no action started");
    assert!(w.rig.sim.trigger_calls.lock().unwrap().is_empty(), "no detection triggered");
}

#[tokio::test]
async fn task_requirement_met_runs_to_done_with_the_task_in_status() {
    let mut w = task_rig(None).await;
    *w.rig.sim.model_kind.lock().unwrap() = Some("segment".into());
    let mut body = tap(&w.rig).await;
    body["task"] = json!("cup");
    let ack = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", body).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let a = heartbeat_until_done(&mut w, ack["action_id"].as_u64().unwrap(), 40.0).await;
    assert_eq!((a["state"].as_str(), a["task"].as_str(), a["holding"].as_bool()), (Some("done"), Some("cup"), Some(true)), "{a}");
    // the task's overrides applied: yaw_sources [mask] (no box source), the probe decided the height
    let yaw: Vec<&str> = a["yaw_estimates"].as_array().unwrap().iter().map(|e| e["source"].as_str().unwrap()).collect();
    assert!(yaw.contains(&"mask") && !yaw.contains(&"box"), "{a}");
    assert_eq!(a["height_source"], "scale", "{a}");
    // the preflight's reading is reused: one per grasp
    assert_eq!(w.rig.sim.mode_info_calls.load(Ordering::SeqCst), 1);
}

#[tokio::test]
async fn grasp_without_a_task_reads_the_mode_once_and_has_no_task_field() {
    let mut w = task_rig(None).await;
    let ack = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", tap(&w.rig).await).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let a = heartbeat_until_done(&mut w, ack["action_id"].as_u64().unwrap(), 40.0).await;
    assert_eq!(a["state"], "done", "{a}");
    assert!(a.get("task").is_none(), "{a}");
    assert_eq!(w.rig.sim.mode_info_calls.load(Ordering::SeqCst), 1);
    // a task without requirements: no check before the ack, the orchestrator's one reading
    let mut body = tap(&w.rig).await;
    body["task"] = json!("plain");
    let ack = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", body).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let id = ack["action_id"].as_u64().unwrap();
    let a = w.rig.sup.get_observation().await.action.unwrap();
    assert_eq!((a.id, a.task.as_deref()), (id, Some("plain")));
    let _ = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/motion/stop", json!({})).await;
    wait_done(&w.rig.sup, id, 5.0, |_| {}).await;
    assert_eq!(w.rig.sim.mode_info_calls.load(Ordering::SeqCst), 2);
}

#[tokio::test]
async fn task_requirement_on_a_legacy_provider_and_provider_down() {
    let mut w = task_rig(None).await;
    // an older detector: no /api/mode (no per-request pipelines, no mode API)
    w.rig.sim.per_request.store(false, Ordering::SeqCst);
    let mut body = tap(&w.rig).await;
    for (task, what) in [("cup", "model.kind"), ("flat", "depth")] {
        body["task"] = json!(task);
        let reply = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", body.clone()).await;
        assert_eq!(reply["code"], "TASK_UNSUPPORTED", "{reply}");
        assert!(reply["message"].as_str().unwrap().ends_with(&format!("provider does not report {what}")), "{reply}");
    }
    // provider unreachable: not a task failure; the grasp fails as without a task
    w.rig.sim.per_request.store(true, Ordering::SeqCst);
    w.rig.sim.mode_down.store(true, Ordering::SeqCst);
    let mut results = Vec::new();
    for task in [None, Some("cup")] {
        let mut b = tap(&w.rig).await;
        if let Some(t) = task { b["task"] = json!(t); }
        let ack = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", b).await;
        assert_eq!(ack["type"], "command_ack", "{ack}");
        let a = wait_done(&w.rig.sup, ack["action_id"].as_u64().unwrap(), 5.0, |_| {}).await;
        results.push((a.state, a.code, a.message.clone()));
    }
    assert_eq!(results[0], results[1]);
    assert_eq!((results[0].0.as_str(), results[0].1.as_deref()), ("failed", Some("DETECTION_UNAVAILABLE")), "{results:?}");
    // 2 legacy checks + one reading per accepted grasp (the task's preflight reading is reused)
    assert_eq!(w.rig.sim.mode_info_calls.load(Ordering::SeqCst), 4);
}

#[tokio::test]
async fn task_request_errors_are_invalid_command() {
    let mut w = task_rig(None).await;
    let base = tap(&w.rig).await;
    let cases = [
        (json!({"task": "bogus"}), "unknown grasp task: bogus"),
        (json!({"task": 3}), "task must be a string"),
        (json!({"overrides": {"tasks": {}}}), "tasks cannot be overridden per request"),
        (json!({"task": "plain", "overrides": {"tasks": {}}}), "tasks cannot be overridden per request"),
        // the request's overrides make the task's setup invalid (plain: max_grasp_height_m 0.05)
        (json!({"task": "plain", "overrides": {"grasp_height_m": 0.055}}), "max_grasp_height_m"),
    ];
    for (extra, msg) in cases {
        let mut body = base.clone();
        for (k, v) in extra.as_object().unwrap() { body[k] = v.clone(); }
        let reply = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", body).await;
        assert_eq!((reply["code"].as_str(), reply["status"].as_u64()), (Some("INVALID_COMMAND"), Some(400)), "{reply}");
        assert!(reply["message"].as_str().unwrap().contains(msg), "{msg}: {reply}");
    }
    assert_eq!(w.rig.sim.mode_info_calls.load(Ordering::SeqCst), 0);
    assert!(w.rig.sup.get_observation().await.action.is_none());
}

#[tokio::test]
async fn dry_run_reports_the_task_requirement() {
    let mut w = task_rig(None).await;
    *w.rig.sim.model_kind.lock().unwrap() = Some("detect".into());
    let mut body = tap(&w.rig).await;
    body["dry_run"] = json!(true);
    // no task: no mode reading, no task fields
    let res = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", body.clone()).await;
    assert_eq!((res["type"].as_str(), res["reachable"].as_bool()), (Some("command_ack"), Some(true)), "{res}");
    assert!(res.get("task_supported").is_none(), "{res}");
    assert_eq!(w.rig.sim.mode_info_calls.load(Ordering::SeqCst), 0);
    // unsupported task: reachable stays the geometry's, one reading
    body["task"] = json!("cup");
    let res = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", body.clone()).await;
    assert_eq!((res["reachable"].as_bool(), res["task_supported"].as_bool()), (Some(true), Some(false)), "{res}");
    assert_eq!(res["task_reason"], "task cup needs model_kind segment, provider has detect", "{res}");
    assert_eq!(w.rig.sim.mode_info_calls.load(Ordering::SeqCst), 1);
    // REST, no lease needed
    let c = reqwest::Client::new();
    let rest: Value = c.post(format!("http://{}/api/v1/action/grasp", w.addr)).json(&body).send().await.unwrap().json().await.unwrap();
    assert_eq!((rest["reachable"].as_bool(), rest["task_supported"].as_bool()), (Some(true), Some(false)), "{rest}");
    // supported task and a task without requirements (no reading)
    *w.rig.sim.model_kind.lock().unwrap() = Some("segment".into());
    let rest: Value = c.post(format!("http://{}/api/v1/action/grasp", w.addr)).json(&body).send().await.unwrap().json().await.unwrap();
    assert_eq!((rest["task_supported"].as_bool(), rest["task_reason"].is_null()), (Some(true), true), "{rest}");
    body["task"] = json!("plain");
    let res = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", body.clone()).await;
    assert_eq!(res["task_supported"], true, "{res}");
    assert_eq!(w.rig.sim.mode_info_calls.load(Ordering::SeqCst), 3);
    // an unknown task fails the dry run itself
    body["task"] = json!("bogus");
    let res = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", body).await;
    assert_eq!((res["reachable"].as_bool(), res["code"].as_str()), (Some(false), Some("INVALID_COMMAND")), "{res}");
    assert!(res.get("task_supported").is_none(), "{res}");
    assert!(w.rig.sup.get_observation().await.action.is_none(), "dry runs start nothing");
}

#[tokio::test]
async fn grasp_tasks_through_the_setup_api_persist_and_reload() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("grasp_setup.json");
    let mut w = ws_rig(Some(path.clone())).await;
    let v: Value = reqwest::get(format!("http://{}/api/v1/grasp/setup", w.addr)).await.unwrap().json().await.unwrap();
    assert_eq!(v["setup"]["tasks"], json!({}), "an empty task map is reported");
    // first insertion
    let tasks = serde_json::to_value(&with_tasks(GraspSetup::default()).tasks).unwrap();
    let ack = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/grasp/setup", json!({"tasks": tasks})).await;
    assert_eq!((ack["type"].as_str(), ack["persisted"].as_bool()), (Some("command_ack"), Some(true)), "{ack}");
    assert_eq!(ack["setup"]["tasks"], tasks);
    // a setup change that makes a stored task invalid is rejected as a whole
    // (plain sets max_grasp_height_m 0.05, below grasp_height_m 0.055; valid without the task)
    assert!(GraspSetup::default().patched(&json!({"grasp_height_m": 0.055})).is_ok());
    let reply = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/grasp/setup", json!({"grasp_height_m": 0.055})).await;
    assert_eq!(reply["code"], "INVALID_COMMAND", "{reply}");
    assert!(reply["message"].as_str().unwrap().starts_with("grasp task plain:"), "{reply}");
    // invalid tasks: bad name, nested tasks, unknown field, unknown task key
    for bad in [json!({"Cup": {}}), json!({"a": {"overrides": {"tasks": {}}}}), json!({"a": {"overrides": {"bogus": 1}}}),
                json!({"a": {"needs": {}}}), json!({"a": {"requires": {"model_kind": "pose"}}})] {
        let reply = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/grasp/setup", json!({"tasks": bad})).await;
        assert_eq!(reply["code"], "INVALID_COMMAND", "{bad}: {reply}");
    }
    let stored = w.rig.sup.inner().read().await.grasp_setup().unwrap();
    assert_eq!(serde_json::to_value(&stored.tasks).unwrap(), tasks, "rejected updates change nothing");
    // the file reloads with the tasks
    let fresh = ArmSupervisor::new(ArmConfig::rebot_dm());
    grasp::load_into(&fresh, &path).await;
    assert_eq!(fresh.inner().read().await.grasp_setup().unwrap().tasks, stored.tasks);
    // a file whose task does not fit its setup is ignored as a whole
    let base = GraspSetup::default();
    let bad = dir.path().join("bad.json");
    std::fs::write(&bad, json!({"grasp_height_m": 0.055, "tasks": {"plain": {"overrides": {"max_grasp_height_m": 0.05}}}}).to_string()).unwrap();
    let (s, outcome) = grasp::load_setup(&bad, &base);
    assert!(matches!(outcome, grasp::LoadOutcome::Invalid(_)) && s == base, "{outcome:?}");
    // `{}` deletes all tasks
    let ack = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/grasp/setup", json!({"tasks": {}})).await;
    assert_eq!(ack["setup"]["tasks"], json!({}), "{ack}");
    let text = std::fs::read_to_string(&path).unwrap();
    assert_eq!(serde_json::from_str::<Value>(&text).unwrap()["tasks"], json!({}));
}

#[tokio::test]
async fn capabilities_advertise_grasp_tasks() {
    let w = ws_rig(None).await;
    let v: Value = reqwest::get(format!("http://{}/api/v1/observation/schema", w.addr)).await.unwrap().json().await.unwrap();
    assert_eq!((v["capabilities"]["grasp"].as_bool(), v["capabilities"]["grasp_tasks"].as_bool()), (Some(true), Some(true)), "{}", v["capabilities"]);
}

// ---- task models (POST /api/models before the grasp, generation pin) ----

/// A provider that can switch models: `detect` resident (its kind is its name), budget as
/// rebot-vision's defaults (`segment` + `depth` does not fit).
fn switching_models() -> grasp::ProviderModels {
    grasp::ProviderModels {
        generation: 0,
        resident: grasp::ResidentModels { detector: Some("detect".into()), depth: None },
        swap_state: "idle".into(),
        swap_error: None,
        catalogue: [("detect", 8.5), ("segment", 19.5), ("depth", 20.5)].into_iter().map(|(n, mb)| (n.to_string(), Some(mb))).collect(),
        ion: Some([21.5, 62.9, 8.0]),
    }
}

/// [`task_rig`] with tasks that switch models: `seg` (segment, requires it), `segdepth` (segment
/// and depth, over the budget), `nodepth` (unload depth only).
async fn models_rig() -> WsRig {
    let w = task_rig(None).await;
    let mut s = w.rig.sup.inner().read().await.grasp_setup().unwrap();
    s.tasks = serde_json::from_value(json!({
        "seg": {"requires": {"model_kind": "segment"}, "models": {"detector": "segment"}},
        "segdepth": {"models": {"detector": "segment", "depth": "depth"}},
        "nodepth": {"models": {"depth": null}}
    })).unwrap();
    s.validate().unwrap();
    w.rig.sup.inner().write().await.set_grasp_setup(s);
    *w.rig.sim.models.lock().unwrap() = Some(switching_models());
    *w.rig.sim.model_kind.lock().unwrap() = Some("detect".into());
    w
}

#[tokio::test]
async fn task_models_switch_before_the_grasp_once() {
    let mut w = models_rig().await;
    let mut body = tap(&w.rig).await;
    body["task"] = json!("seg");
    let ack = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", body.clone()).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let a = heartbeat_until_done(&mut w, ack["action_id"].as_u64().unwrap(), 40.0).await;
    assert_eq!((a["state"].as_str(), a["task"].as_str()), (Some("done"), Some("seg")), "{a}");
    // only the slot the task names; the post-switch reading checked `requires` and was reused
    assert_eq!(*w.rig.sim.set_models_calls.lock().unwrap(), vec![json!({"detector": "segment"})]);
    let m = w.rig.sim.models.lock().unwrap().clone().unwrap();
    assert_eq!((m.generation, m.resident.detector.as_deref()), (1, Some("segment")));
    assert_eq!(w.rig.sim.mode_info_calls.load(Ordering::SeqCst), 2, "one reading before, one after the switch");
    // the resident set already matches: no switch; no swap back after the grasp
    let ack = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", body).await;
    let a = heartbeat_until_done(&mut w, ack["action_id"].as_u64().unwrap(), 40.0).await;
    assert_eq!(a["state"], "done", "{a}");
    assert_eq!(w.rig.sim.set_models_calls.lock().unwrap().len(), 1);
    assert_eq!(w.rig.sim.mode_info_calls.load(Ordering::SeqCst), 3);
}

#[tokio::test]
async fn task_models_refusals_reject_without_motion() {
    let mut w = models_rig().await;
    let q0 = w.rig.sup.get_observation().await.commanded_positions;
    let mut body = tap(&w.rig).await;
    body["task"] = json!("segdepth");
    let load_failed = grasp::ProviderModels { generation: 1, swap_state: "failed".into(), swap_error: Some("LOAD_FAILED:depth".into()), ..switching_models() };
    let cases = [
        (Some((409, json!({"error": "ION_BUDGET", "need_mb": 69.5, "limit_mb": 62.9}))), Some(switching_models()),
         "task segdepth: model switch refused: ION_BUDGET (need 69.5 MB, limit 62.9 MB)"),
        (Some((409, json!({"error": "SWAP_BUSY"}))), Some(switching_models()), "task segdepth: model switch busy"),
        (Some((400, json!({"error": "no model named \"segment\""}))), Some(switching_models()),
         "task segdepth: model switch refused: no model named \"segment\""),
        (None, None, "task segdepth: provider cannot switch models"),
        // accepted, but the provider's load failed: the poll reads it
        (Some((202, json!({"generation": 1}))), Some(load_failed), "task segdepth: model switch failed: LOAD_FAILED:depth"),
    ];
    for (reply, models, msg) in cases {
        *w.rig.sim.set_models_reply.lock().unwrap() = reply;
        *w.rig.sim.models.lock().unwrap() = models;
        let reply = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", body.clone()).await;
        assert_eq!((reply["type"].as_str(), reply["code"].as_str(), reply["status"].as_u64()), (Some("command_reject"), Some("TASK_UNSUPPORTED"), Some(422)), "{reply}");
        assert_eq!(reply["message"], msg, "{reply}");
    }
    assert_eq!(w.rig.sim.set_models_calls.lock().unwrap().len(), 5);
    let obs = w.rig.sup.get_observation().await;
    assert!(obs.action.is_none(), "no action started: {:?}", obs.action);
    assert_eq!(obs.commanded_positions, q0, "no motion commanded");
    assert!(w.rig.sim.trigger_calls.lock().unwrap().is_empty(), "no detection triggered");
}

/// The control replies (`command_ack` / `command_reject`) received within `within`, keyed by
/// request id.
async fn command_replies(ws: &mut common::TestSocket, ids: &[u64], within: Duration) -> std::collections::HashMap<u64, Value> {
    use futures_util::StreamExt;
    let mut got = std::collections::HashMap::new();
    let _ = tokio::time::timeout(within, async {
        while got.len() < ids.len() {
            let Some(Ok(msg)) = ws.next().await else { return; };
            let Ok(v) = serde_json::from_str::<Value>(&msg.into_text().unwrap()) else { continue; };
            if !matches!(v["type"].as_str(), Some("command_ack" | "command_reject")) { continue; }
            if let Some(id) = v["request_id"].as_u64().filter(|id| ids.contains(id)) { got.insert(id, v); }
        }
    }).await;
    got
}

#[tokio::test]
async fn ws_stop_during_grasp_preflight_cancels_it_at_once() {
    let mut w = models_rig().await;
    let q0 = w.rig.sup.get_observation().await.commanded_positions;
    // the switch is accepted but its generation is never published: the preflight polls 5 s
    *w.rig.sim.set_models_reply.lock().unwrap() = Some((202, json!({"generation": 99})));
    let mut body = tap(&w.rig).await;
    body["task"] = json!("seg");
    let (grasp_id, queued_id, stop_id) = (1_000_000, 1_000_001, 1_000_002);
    let command = |id: u64, path: &str, body: Value| json!({"type":"command", "request_id":id, "generation":w.generation, "epoch_id":w.epoch, "path":path, "body":body});
    common::send(&mut w.ws, command(grasp_id, "/api/v1/action/grasp", body)).await;
    let deadline = Instant::now() + Duration::from_secs(3);
    while w.rig.sim.set_models_calls.lock().unwrap().is_empty() {
        assert!(Instant::now() < deadline, "preflight did not reach the model switch");
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    // a motion sent before the stop waits behind the grasp and is cancelled with it
    common::send(&mut w.ws, command(queued_id, "/api/v1/motion/home", json!({}))).await;
    let t = Instant::now();
    common::send(&mut w.ws, command(stop_id, "/api/v1/motion/stop", json!({}))).await;
    let replies = command_replies(&mut w.ws, &[grasp_id, queued_id, stop_id], Duration::from_millis(300)).await;
    let elapsed = t.elapsed();
    assert_eq!(replies.len(), 3, "all three replies within 300 ms: {replies:?}");
    assert!(elapsed < Duration::from_millis(300), "{elapsed:?}");
    assert_eq!(replies[&stop_id]["type"], "command_ack", "{}", replies[&stop_id]);
    for id in [grasp_id, queued_id] {
        let r = &replies[&id];
        assert_eq!((r["type"].as_str(), r["code"].as_str(), r["status"].as_u64()), (Some("command_reject"), Some("STOPPED"), Some(409)), "{r}");
    }
    // past the end of the preflight poll: the grasp never began and nothing moved
    let until = Instant::now() + Duration::from_millis(5600);
    while Instant::now() < until {
        common::send(&mut w.ws, json!({"type":"heartbeat", "generation":w.generation, "epoch_id":w.epoch})).await;
        // telemetry never pauses: read for a fixed 200 ms
        let late = command_replies(&mut w.ws, &[grasp_id, queued_id], Duration::from_millis(200)).await;
        assert!(late.is_empty(), "a second reply arrived: {late:?}");
    }
    let obs = w.rig.sup.get_observation().await;
    assert!(obs.action.is_none() && obs.previous_action.is_none(), "no action started: {:?}", obs.action);
    assert_eq!(obs.commanded_positions, q0, "no motion commanded");
    assert!(w.rig.sim.trigger_calls.lock().unwrap().is_empty(), "no detection triggered");
    // the connection still serves commands
    let ack = common::ws_reply(&mut w.ws, command(stop_id + 1, "/api/v1/motion/stop", json!({})), &["command_ack", "command_reject"]).await;
    assert_eq!((ack["type"].as_str(), ack["request_id"].as_u64()), (Some("command_ack"), Some(stop_id + 1)), "{ack}");
}

#[tokio::test]
async fn task_models_switch_unreachable_fails_as_a_provider_error() {
    let mut w = models_rig().await;
    *w.rig.sim.set_models_reply.lock().unwrap() = Some((503, json!({"error": "busy"})));
    w.rig.sim.models.lock().unwrap().as_mut().unwrap().resident.depth = Some("depth".into());
    let mut body = tap(&w.rig).await;
    body["task"] = json!("nodepth");
    let ack = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", body).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let a = wait_done(&w.rig.sup, ack["action_id"].as_u64().unwrap(), 5.0, |_| {}).await;
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some("DETECTION_UNAVAILABLE")), "{a:?}");
    assert!(a.message.as_deref().unwrap().contains("model switch failed: detector returned 503"), "{a:?}");
}

#[tokio::test]
async fn a_frame_from_another_model_generation_fails_the_grasp() {
    let mut w = models_rig().await;
    *w.rig.sim.frame_generation.lock().unwrap() = Some(7);
    let ack = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", tap(&w.rig).await).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let a = heartbeat_until_done(&mut w, ack["action_id"].as_u64().unwrap(), 20.0).await;
    assert_eq!((a["state"].as_str(), a["code"].as_str(), a["message"].as_str()),
               (Some("failed"), Some("DETECTION_UNAVAILABLE"), Some("model changed during grasp")), "{a}");
    // frames of the generation the grasp started with
    *w.rig.sim.frame_generation.lock().unwrap() = None;
    let ack = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", tap(&w.rig).await).await;
    let a = heartbeat_until_done(&mut w, ack["action_id"].as_u64().unwrap(), 40.0).await;
    assert_eq!(a["state"], "done", "{a}");
}

#[tokio::test]
async fn dry_run_task_models_precedence() {
    let mut w = models_rig().await;
    let mut body = tap(&w.rig).await;
    body["dry_run"] = json!(true);
    let verdict = |res: &Value| (res["task_supported"].as_bool(), res["task_reason"].as_str().map(str::to_string));
    // budget refusal from the catalogue
    body["task"] = json!("segdepth");
    let res = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", body.clone()).await;
    assert_eq!(verdict(&res), (Some(false), Some("task segdepth: model switch refused: ION_BUDGET (need 69.5 MB, limit 62.9 MB)".into())), "{res}");
    // a switch that fits: supported, the requirement is not checked on the current set
    body["task"] = json!("seg");
    let res = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", body.clone()).await;
    assert_eq!(verdict(&res), (Some(true), Some("needs model switch".into())), "{res}");
    // without the provider's ion budget nothing is computed
    w.rig.sim.models.lock().unwrap().as_mut().unwrap().ion = None;
    body["task"] = json!("segdepth");
    let res = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", body.clone()).await;
    assert_eq!(verdict(&res), (Some(true), Some("needs model switch".into())), "{res}");
    // the resident set matches: the requirement check on the current set
    w.rig.sim.models.lock().unwrap().as_mut().unwrap().resident.detector = Some("segment".into());
    body["task"] = json!("seg");
    let res = common::rpc(&mut w.ws, w.generation, w.epoch, "/api/v1/action/grasp", body).await;
    assert_eq!(verdict(&res), (Some(false), Some("task seg needs model_kind segment, provider has detect".into())), "{res}");
    assert!(w.rig.sim.set_models_calls.lock().unwrap().is_empty(), "dry runs never switch");
    assert!(w.rig.sup.get_observation().await.action.is_none());
}

// ---- depth measurement without motion (`/api/v1/grasp/measure`) ----

fn relative_depth() -> DepthSim {
    DepthSim { metric_model: false, a0: 2.7, b0: 0.45, occluded_table: false, occluded_above_z: None, depth_conf: Some(0.95),
               jaw_clear: Some(true), length_px: Some(60.0), drift_per_frame: false }
}

/// A rig 20 deg off vertical (an oblique table view: the relative fit needs inverse-depth spread),
/// per-request triggers and depth, the server, and a lease held by another connection.
async fn measure_rig(depth: DepthSim) -> WsRig {
    let setup = GraspSetup { camera_tilt_deg: 0.0, ..GraspSetup::default() };
    let r = rig(setup, Truth::same(), 20.0, [0.0, 0.0], true).await;
    r.sim.per_request.store(true, Ordering::SeqCst);
    r.sim.depth_available.store(true, Ordering::SeqCst);
    *r.sim.width_px.lock().unwrap() = Some(40.0);
    *r.sim.depth_sim.lock().unwrap() = Some(depth);
    ws_rig_on(r, None).await
}

async fn post_measure(w: &WsRig, body: Value) -> (u16, Value) {
    let r = reqwest::Client::new().post(format!("http://{}/api/v1/grasp/measure", w.addr)).json(&body).send().await.unwrap();
    (r.status().as_u16(), r.json().await.unwrap())
}

fn cup(v: &Value) -> &Value {
    v["detections"].as_array().unwrap().iter().find(|d| d["label"] == "cup").unwrap_or_else(|| panic!("no cup: {v}"))
}

#[tokio::test]
async fn measure_reports_known_heights_and_the_table_fit_without_moving() {
    let w = measure_rig(relative_depth()).await;
    let q0 = w.rig.sup.inner().read().await.commanded_positions.clone();
    // the whole 0.05 m box, then boxes of a 0.03 m top face
    for (top, height) in [(None, 0.05), (Some(0.03), 0.03)] {
        *w.rig.sim.top.lock().unwrap() = top;
        let (status, v) = post_measure(&w, json!({"label": "CUP"})).await;
        assert_eq!(status, 200, "{v}");
        let fit = &v["table_fit"];
        assert_eq!((fit["ok"].as_bool(), fit["source"].as_str(), fit["metric_model"].as_bool()), (Some(true), Some("oblique_ransac"), Some(false)), "{fit}");
        // the sim's disparity is exactly 1/z = 2.7 disp + 0.45
        assert!((fit["a"].as_f64().unwrap() - 2.7).abs() < 1e-3 && (fit["b"].as_f64().unwrap() - 0.45).abs() < 1e-3, "{fit}");
        assert!(fit["residual"]["rms_mm"].as_f64().unwrap() < 0.1, "{fit}");
        assert_eq!(fit["residual"]["inliers"], fit["n_valid"]);
        assert!(fit["n_valid"].as_u64().unwrap() >= 8 && fit["least_squares"]["r2_inv_z"].as_f64().unwrap() > 0.9999, "{fit}");
        let d = cup(&v);
        assert_eq!(v["detections"].as_array().unwrap().len(), 1, "label filter: {v}");
        let m = &d["measurement"];
        assert!((m["height_m"].as_f64().unwrap() - height).abs() < 0.003, "height {height}: {m}");
        assert_eq!((m["source"].as_str(), m["jaw_clear"].as_bool(), d["why"].is_null()), (Some("oblique_ransac"), Some(true), true), "{d}");
        assert!(m["depth_m"].as_f64().unwrap() > 0.1 && m["width_m"].as_f64().is_some() && m["length_m"].as_f64().is_some(), "{m}");
        assert!(d["disp_fg"].as_f64().unwrap() > d["disp_bg"].as_f64().unwrap());
        // camera pose and calibration identity
        assert_eq!(v["camera"]["pose_source"], "commanded");
        assert_eq!(v["camera"]["commanded_rad"].as_array().unwrap().len(), 6);
        assert_eq!(v["camera"]["lens_known"], true);
        let cal = &v["calibration"];
        assert_eq!((cal["calibrated"].as_bool(), cal["kinematics_corrected"].as_bool(), cal["distortion_modelled"].as_bool()), (Some(false), Some(false), Some(false)), "{cal}");
        assert_eq!(cal["table_z_m"].as_f64(), Some(0.0));
    }
    assert_eq!(w.rig.sim.trigger_calls.lock().unwrap().last().unwrap(), &vec!["yolo".to_string(), "depth".to_string()]);
    let inner = w.rig.sup.inner().read().await;
    assert_eq!(inner.commanded_positions, q0, "nothing moved");
    assert!(inner.active_trajectory.is_none());
}

#[tokio::test]
async fn measure_over_ws_needs_no_lease() {
    let w = measure_rig(relative_depth()).await;
    // a second connection: the lease is held by the rig's controller
    let (mut ws, _) = tokio_tungstenite::connect_async(format!("ws://{}/api/v1/ws", w.addr)).await.unwrap();
    let ack = common::ws_reply(&mut ws, json!({"type":"command", "request_id": 1, "path":"/api/v1/grasp/measure", "body":{"pipelines":["depth"]}}), &["command_ack", "command_reject"]).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    assert_eq!(ack["table_fit"]["source"], "oblique_ransac");
    assert!((cup(&ack)["measurement"]["height_m"].as_f64().unwrap() - 0.05).abs() < 0.003, "{ack}");
    assert_eq!(w.rig.sim.trigger_calls.lock().unwrap().last().unwrap(), &vec!["depth".to_string()]);
    // request ids still increase
    let rej = common::ws_reply(&mut ws, json!({"type":"command", "request_id": 1, "path":"/api/v1/grasp/measure", "body":{}}), &["command_ack", "command_reject"]).await;
    assert_eq!(rej["code"], "INVALID_COMMAND", "{rej}");
    let rej = common::ws_reply(&mut ws, json!({"type":"command", "request_id": 2, "path":"/api/v1/grasp/measure", "body":{"pipelines":["yolo"]}}), &["command_ack", "command_reject"]).await;
    assert_eq!((rej["status"].as_u64(), rej["code"].as_str()), (Some(400), Some("INVALID_COMMAND")), "{rej}");
}

#[tokio::test]
async fn measure_metric_model_and_failures_say_why() {
    let w = measure_rig(DepthSim { metric_model: true, a0: 1.0, b0: 0.0, ..relative_depth() }).await;
    let (_, v) = post_measure(&w, json!({})).await;
    assert_eq!((v["table_fit"]["source"].as_str(), v["table_fit"]["b"].as_f64()), (Some("metric_table"), Some(0.0)), "{v}");
    assert!((cup(&v)["measurement"]["height_m"].as_f64().unwrap() - 0.05).abs() < 0.003, "{v}");
    // the person box (score 0.5) is under min_score 0.6
    let (_, v) = post_measure(&w, json!({"min_score": 0.6})).await;
    assert!(v["detections"].as_array().unwrap().iter().all(|d| d["label"] == "cup"), "{v}");
    // table hidden: no fit, the detection says why
    w.rig.sim.depth_sim.lock().unwrap().as_mut().unwrap().occluded_table = true;
    let (status, v) = post_measure(&w, json!({"label": "cup"})).await;
    assert_eq!(status, 200);
    assert_eq!((v["table_fit"]["ok"].as_bool(), v["table_fit"]["why"].as_str(), v["table_fit"]["n_samples"].as_u64()), (Some(false), Some("too_few_samples"), Some(0)), "{v}");
    assert_eq!((cup(&v)["measurement"].is_null(), cup(&v)["why"].as_str()), (true, Some("too_few_samples")));
    // no depth in the frame
    *w.rig.sim.depth_sim.lock().unwrap() = None;
    let (_, v) = post_measure(&w, json!({})).await;
    assert_eq!(v["table_fit"]["why"], "no_depth_frame", "{v}");
    // a provider without per-request pipelines
    w.rig.sim.per_request.store(false, Ordering::SeqCst);
    let (status, v) = post_measure(&w, json!({})).await;
    assert_eq!((status, v["code"].as_str()), (422, Some("UNSUPPORTED")), "{v}");
}

/// The simulated camera, with the commanded positions stepped while the frame is captured.
struct MovingDuringCapture(Arc<SimCamera>);

impl DetectionSource for MovingDuringCapture {
    fn inference_enabled(&self) -> BoxFuture<'_, Result<bool, String>> { self.0.inference_enabled() }
    fn set_inference(&self, enabled: bool) -> BoxFuture<'_, Result<(), String>> { self.0.set_inference(enabled) }
    fn next_frame(&self, after: Option<u64>, wait_ms: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> { self.0.next_frame(after, wait_ms) }
    fn per_request_pipelines(&self) -> BoxFuture<'_, Result<bool, String>> { Box::pin(async { Ok(true) }) }
    fn trigger(&self, pipelines: &'static [&'static str], wait_ms: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        Box::pin(async move {
            {
                let mut inner = self.0.sup.inner().write().await;
                inner.commanded_positions[0] += 0.01;
                inner.target_positions[0] += 0.01;
            }
            self.0.trigger(pipelines, wait_ms).await
        })
    }
}

#[tokio::test]
async fn measure_refuses_a_moving_arm() {
    let w = measure_rig(relative_depth()).await;
    let moving = MovingDuringCapture(w.rig.sim.clone());
    let e = armd::api::rest::ApiError::from(armd::depth_measure::measure(&w.rig.sup, &moving, &json!({})).await.unwrap_err());
    assert_eq!((e.status.as_u16(), e.code.as_deref()), (409, Some("ARM_MOVING")), "{}", e.message);
    // a servo stream running: refused before the trigger
    let calls = w.rig.sim.trigger_calls.lock().unwrap().len();
    {
        let mut inner = w.rig.sup.inner().write().await;
        let q = inner.commanded_positions.clone();
        inner.servo_target = Some(q);
        inner.last_motion_frame = Instant::now();
    }
    let (status, v) = post_measure(&w, json!({})).await;
    assert_eq!((status, v["code"].as_str()), (409, Some("ARM_MOVING")), "{v}");
    assert_eq!(w.rig.sim.trigger_calls.lock().unwrap().len(), calls, "no trigger");
}

#[tokio::test]
async fn capabilities_advertise_grasp_measure() {
    let w = ws_rig(None).await;
    let v: Value = reqwest::get(format!("http://{}/api/v1/observation/schema", w.addr)).await.unwrap().json().await.unwrap();
    assert_eq!(v["capabilities"]["grasp_measure"].as_bool(), Some(true), "{}", v["capabilities"]);
}
