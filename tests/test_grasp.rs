//! Tap-to-grasp action (`/api/v1/action/grasp`) against a simulated camera.
//!
//! The simulated camera projects a box-shaped object at a fixed table position through the
//! current measured FK pose, its own camera mount and pinhole intrinsics. The controller
//! assumes different (rough) intrinsics and mount, as on the real arm.
mod common;

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::grasp::{self, BoxFuture, CamModel, Detection, DetectionFrame, DetectionSource, GraspDeps, GraspSetup, VisionMode};
use armd::kinematics::{self, Pose, Vec3};
use armd::model::CameraMount;
use armd::{ArmConfig, ArmSupervisor};
use serde_json::{json, Value};

const DT: f64 = 0.02;
const INSPECT: [f64; 6] = [0.0, -1.167, -1.565, 1.435, 0.0, std::f64::consts::FRAC_PI_2];
/// J7 where the jaws touch the test object (closed = 0, open = -5.7).
const OBJECT_J7: f32 = -1.5;

struct Object { centre: Vec3, half: [f64; 2], height: f64 }

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
    /// Every frame request fails (detector error).
    failing: AtomicBool,
    /// Frames keep arriving but were captured 30 s ago (a stalled camera behind a live API).
    stale: AtomicBool,
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
        let q = self.sup.inner().read().await.measured_positions.clone();
        let pose = kinematics::fk(&kinematics::q6(&q));
        let mut detections = vec![Detection { label: "person".into(), score: 0.5, box_norm: [0.0, 0.0, 0.08, 0.1] }];
        if self.visible.load(Ordering::SeqCst) {
            if let Some(b) = self.project_box(&pose) { detections.push(Detection { label: self.label.clone(), score: 0.9, box_norm: b }); }
        }
        let frame_id = self.frame.fetch_add(1, Ordering::SeqCst) + 1;
        let captured_at = if self.stale.load(Ordering::SeqCst) { Instant::now() - Duration::from_secs(30) } else { Instant::now() };
        DetectionFrame { frame_id, captured_at, detections }
    }

    fn project_box(&self, pose: &Pose) -> Option<[f64; 4]> {
        let o = &self.object;
        let sh = *self.shift.lock().unwrap();
        let o = Object { centre: [o.centre[0] + sh[0], o.centre[1] + sh[1], o.centre[2]], half: o.half, height: o.height };
        let (mut x0, mut y0, mut x1, mut y1) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        for sx in [-1.0, 1.0] { for sy in [-1.0, 1.0] { for z in [o.centre[2], o.centre[2] + o.height] {
            let p = self.cam.project(pose, &[o.centre[0] + sx * o.half[0], o.centre[1] + sy * o.half[1], z])?;
            x0 = x0.min(p[0]); y0 = y0.min(p[1]); x1 = x1.max(p[0]); y1 = y1.max(p[1]);
        }}}
        let [w, h] = self.cam.size;
        let (cx, cy) = ((x0 + x1) / 2.0, (y0 + y1) / 2.0);
        if !(cx > 0.0 && cx < w && cy > 0.0 && cy < h) { return None; }
        Some([(x0 / w).clamp(0.0, 1.0), (y0 / h).clamp(0.0, 1.0), (x1 / w).clamp(0.0, 1.0), (y1 / h).clamp(0.0, 1.0)])
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
        Box::pin(async move { Ok(self.per_request.load(Ordering::SeqCst)) })
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
    fn same() -> Self { Self { tilt_deg: 0.0, rot_deg: 0.0, fx: 600.0, fy: 600.0, cx_off: 0.0, cy_off: 0.0, offset: [-0.06, 0.04, 0.0], known_offset: true } }
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
    let r0 = grasp::look_down(&kinematics::fk(&INSPECT).r);
    let p0 = kinematics::fk(&INSPECT).p;
    let radial = kinematics::unit(&[p0[0], p0[1], 0.0]);
    let axis = [-radial[1], radial[0], 0.0];
    let r = kinematics::mat_mul(&kinematics::axis_r(&axis, tilt_from_vertical_deg.to_radians()), &r0);
    solve_start(Pose::new(r, [p0[0], p0[1], z]))
}

/// Start with the tool pitched `fraction` of the way from vertical to camera-down (mount `rec`).
fn pitched_start(rec: &kinematics::Mat3, fraction: f64, z: f64) -> ([f32; 7], Pose) {
    let r0 = grasp::look_down(&kinematics::fk(&INSPECT).r);
    let w = kinematics::log_v(&kinematics::mul_t(&grasp::camera_down(&r0, rec), &r0));
    let r = kinematics::mat_mul(&kinematics::exp_r(&w.map(|v| v * fraction)), &r0);
    let p0 = kinematics::fk(&INSPECT).p;
    solve_start(Pose::new(r, [p0[0], p0[1], z]))
}

fn solve_start(goal: Pose) -> ([f32; 7], Pose) {
    let lo = [-3.1, -2.05, -2.6, -3.1, -2.05, -6.2];
    let hi = [3.1, 2.05, 2.6, 3.1, 2.05, 6.2];
    let mut q = INSPECT;
    for _ in 0..20 {
        let sol = kinematics::solve_dls(&goal, &q, &lo, &hi, kinematics::PAR.w);
        q = sol.q;
        if sol.ok { break; }
    }
    let pose = kinematics::fk(&q);
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
                    arm: Option<TestArm>) -> Rig {
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
        inner.set_torque(true).unwrap();
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
        shift: Mutex::new([0.0; 2]), failing: AtomicBool::new(false), stale: AtomicBool::new(false),
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

async fn start_pose(rig: &Rig) -> Pose { kinematics::fk(&kinematics::q6(&rig.sup.inner().read().await.commanded_positions)) }

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

struct Outcome { status: armd::motion::ActionStatus, centred_err_m: f64, final_pose: Pose }

async fn run_grasp(rig: &Rig) -> Outcome {
    let body = tap(rig).await;
    let (id, duration) = grasp::start(&rig.sup, rig.sim.clone(), &body).await.expect("grasp accepted");
    assert!(duration > 5.0);
    let mut centred_err_m = f64::NAN;
    let sup = rig.sup.clone();
    let object = rig.object;
    let mut seen_align = false;
    let status = wait_done(&rig.sup, id, 40.0, |a| {
        if !seen_align && matches!(a.phase.as_deref(), Some("align" | "descend")) {
            seen_align = true;
            if let Ok(inner) = sup.inner().try_read() {
                centred_err_m = xy_err(&kinematics::fk(&kinematics::q6(&inner.commanded_positions)).p, &object);
            }
        }
    }).await;
    let final_pose = start_pose(rig).await;
    if centred_err_m.is_nan() { centred_err_m = xy_err(&final_pose.p, &rig.object); }
    Outcome { status, centred_err_m, final_pose }
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
        let truth = Truth { tilt_deg: -3.0, rot_deg: 5.0, fx: 690.0, fy: 560.0, cx_off: 14.0, cy_off: -9.0, offset: [-0.06, 0.04, 0.0], known_offset: false };
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
    let (id, _) = grasp::start(&rig.sup, rig.sim.clone(), &tap(&rig).await).await.unwrap();
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
    let (id, _) = grasp::start(&rig.sup, rig.sim.clone(), &tap(&rig).await).await.unwrap();
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
    let (id, _) = grasp::start(&rig.sup, rig.sim.clone(), &tap(&rig).await).await.unwrap();
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
        inner.jog(q, None, None).unwrap();
    }).await;
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("cancelled", Some("SUPERSEDED")), "{a:?}");
}

#[tokio::test]
async fn grasp_rejects_servo_frames_while_running() {
    let rig = rig(GraspSetup::default(), Truth::same(), 0.0, [0.0, 0.0], false).await;
    let (id, _) = grasp::start(&rig.sup, rig.sim.clone(), &tap(&rig).await).await.unwrap();
    wait_phase(&rig.sup, id, "centring", Some(false)).await;
    let mut inner = rig.sup.inner().write().await;
    let q = inner.commanded_positions.clone();
    let epoch = inner.safety.epoch_id();
    assert_eq!(inner.servo(q, epoch), Err(armd::SafetyError::MotionBusy));
}

// ---- WS: ack, lease loss, workspace rejection, setup ----

struct WsRig { rig: Rig, ws: common::TestSocket, generation: u64, epoch: u64, addr: std::net::SocketAddr }

async fn ws_rig(setup_path: Option<std::path::PathBuf>) -> WsRig {
    ws_rig_on(rig(GraspSetup::default(), Truth::same(), 0.0, [0.0, 0.0], true).await, setup_path).await
}

async fn ws_rig_on(rig: Rig, setup_path: Option<std::path::PathBuf>) -> WsRig {
    let deps = GraspDeps { source: rig.sim.clone(), setup_path };
    let router = armd::api::create_router_with(rig.sup.clone(), armd::api::proxy::DetectorUpstream::parse("http://127.0.0.1:9").unwrap(), deps);
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
async fn grasp_setup_persists_round_trip() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("sub").join("grasp_setup.json");
    let mut w = ws_rig(Some(path.clone())).await;
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
    let (id, _) = grasp::start(&rig1.sup, rig1.sim.clone(), &tap(&rig1).await).await.unwrap();
    wait_phase(&rig1.sup, id, "centring", None).await;
    rig1.sim.visible.store(false, Ordering::SeqCst);
    let a = wait_done(&rig1.sup, id, 10.0, |_| {}).await;
    assert_eq!(a.code.as_deref(), Some("TARGET_LOST"));
    assert!(rig1.sim.mode_calls.lock().unwrap().is_empty());

    // aruco base mode: restored after a failure as well
    let rig2 = rig(setup.clone(), Truth::same(), 0.0, [0.01, 0.01], false).await;
    *rig2.sim.vision.lock().unwrap() = Some(vm("aruco", true));
    let (id, _) = grasp::start(&rig2.sup, rig2.sim.clone(), &tap(&rig2).await).await.unwrap();
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
    let (id, _) = grasp::start(&rig.sup, rig.sim.clone(), &tap(&rig).await).await.unwrap();
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
    let (id, _) = grasp::start(&rig1.sup, rig1.sim.clone(), &tap(&rig1).await).await.unwrap();
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
    let (id, _) = grasp::start(&rig2.sup, rig2.sim.clone(), &tap(&rig2).await).await.unwrap();
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
    inner.begin_macro("grasp", &input, None).unwrap();
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

// ---- camera-down acquisition (v8 holder: rot 90, tilt -38) ----

const V8_OFFSET: Vec3 = [-0.0731, 0.0005, 0.066];

/// Camera-down grasps with the v8 mount: approach and centre with the optical axis down, then
/// the transition to the tool-vertical pose above the target, align, descend, close, lift.
#[tokio::test]
async fn camera_down_grasp_with_the_v8_mount() {
    let setup = GraspSetup { camera_tilt_deg: -38.0, ..GraspSetup::default() };
    // controller = truth, then truth off by 2 deg tilt, 1 deg rot and fx / fy 5 %
    let cases = [
        ("same", Truth { tilt_deg: 0.0, rot_deg: 90.0, fx: 600.0, fy: 600.0, cx_off: 0.0, cy_off: 0.0, offset: V8_OFFSET, known_offset: true }, [0.0, 0.0], 0.004),
        ("same, offset", Truth { tilt_deg: 0.0, rot_deg: 90.0, fx: 600.0, fy: 600.0, cx_off: 0.0, cy_off: 0.0, offset: V8_OFFSET, known_offset: true }, [0.03, -0.02], 0.004),
        ("perturbed", Truth { tilt_deg: -2.0, rot_deg: 91.0, fx: 630.0, fy: 570.0, cx_off: 6.0, cy_off: -4.0, offset: V8_OFFSET, known_offset: true }, [-0.02, 0.025], 0.012),
    ];
    for (name, truth, offset, max_err) in cases {
        // the operator has the tool pitched half way, so the table below is in view
        let start = pitched_start(&kinematics::cam_mount(false, 90.0, -38.0), 0.5, 0.22);
        let rig = rig_mounted(setup.clone(), truth, 90.0, start, offset, false).await;
        let body = tap(&rig).await;
        let (id, duration) = grasp::start(&rig.sup, rig.sim.clone(), &body).await.expect("grasp accepted");
        let mut phases: Vec<String> = Vec::new();
        let mut down_err = f64::NAN;
        let sup = rig.sup.clone();
        let cam = rig.sim.cam.clone();
        let status = wait_done(&rig.sup, id, 60.0, |a| {
            let p = a.phase.clone().unwrap_or_default();
            if phases.last() != Some(&p) {
                // optical axis at the end of the centring (first look at the transition phase)
                if p == "transition" {
                    if let Ok(inner) = sup.inner().try_read() {
                        let pose = kinematics::fk(&kinematics::q6(&inner.commanded_positions));
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
        let o = Outcome { status, centred_err_m: f64::NAN, final_pose };
        assert_holding_done(&o, &setup, &rig.object, max_err);
        let order: Vec<&str> = ["approach", "centring", "transition", "align", "descend", "close", "lift", "done"].into_iter()
            .filter(|p| phases.iter().any(|q| q == p)).collect();
        assert_eq!(order, ["approach", "centring", "transition", "align", "descend", "close", "lift", "done"], "{phases:?}");
        let pos = |p: &str| phases.iter().position(|q| q == p).unwrap();
        assert!(pos("centring") < pos("transition") && pos("transition") < pos("descend"), "{phases:?}");
        if name.starts_with("same") { assert!(down_err < 1e-3, "optical axis vertical while centring: {down_err}"); }
    }
}

/// At a low approach height the target under the jaws is in view after the transition: the
/// re-check records its offset from the (calibrated) grip pixel; motion is unchanged.
#[tokio::test]
async fn camera_down_recheck_records_the_final_offset() {
    let setup = GraspSetup { camera_tilt_deg: -38.0, approach_height_m: 0.06, ..GraspSetup::default() };
    let truth = Truth { rot_deg: 90.0, offset: V8_OFFSET, ..Truth::same() };
    let start = pitched_start(&kinematics::cam_mount(false, 90.0, -38.0), 0.5, 0.22);
    let start_r = start.1.r;
    let rig = rig_mounted(setup.clone(), truth, 90.0, start, [0.01, 0.0], false).await;
    let mut s = setup.clone();
    s.grip_pixel_norm = calibrate_grip(&rig, &setup, &start_r);
    rig.sup.inner().write().await.set_grasp_setup(s.clone());
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
    let err = grasp::start(&rig.sup, rig.sim.clone(), &body).await.unwrap_err();
    assert_eq!(err.code.as_deref(), Some("INVALID_COMMAND"), "{err:?}");
    assert!(rig.sup.get_observation().await.action.is_none(), "no action started");
    body["overrides"] = json!({"acquire_camera_down": "sideways"});
    assert_eq!(grasp::start(&rig.sup, rig.sim.clone(), &body).await.unwrap_err().code.as_deref(), Some("INVALID_COMMAND"));
}

/// Lowest base-frame z of the TCP, the workspace check points and the lens centre at `q`.
fn envelope_min_z(config: &ArmConfig, q: &[f32]) -> f64 {
    let pose = kinematics::fk(&kinematics::q6(q));
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
    let setup = GraspSetup { camera_tilt_deg: -38.0, table_z_m: 0.10, grasp_height_m: 0.015, approach_height_m: 0.035, ..GraspSetup::default() };
    // rot 0: the tilt pitches the tool about the axis along which the check points are
    // 92.5 mm off the tool axis (v8's rot 90 pitches about the 34.7 mm one)
    let truth = Truth { rot_deg: 0.0, offset: V8_OFFSET, ..Truth::same() };
    let start = pitched_start(&kinematics::cam_mount(false, 0.0, -38.0), 0.5, 0.30);
    let rig = rig_mounted(setup.clone(), truth, 0.0, start, [0.0, 0.0], false).await;
    let config = rig.sup.inner().read().await.config.clone();
    let body = tap(&rig).await;
    let (id, _) = grasp::start(&rig.sup, rig.sim.clone(), &body).await.expect("grasp accepted");
    let sup = rig.sup.clone();
    let mut lowest = f64::INFINITY;
    let mut at = String::new();
    let a = wait_done(&rig.sup, id, 60.0, |a| {
        if let Ok(inner) = sup.inner().try_read() {
            let z = envelope_min_z(&config, &inner.commanded_positions);
            if z < lowest { lowest = z; at = a.phase.clone().unwrap_or_default(); }
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
async fn v8_recheck_rig(lease: bool) -> (Rig, GraspSetup) {
    let setup = GraspSetup { camera_tilt_deg: -38.0, approach_height_m: 0.06, ..GraspSetup::default() };
    let truth = Truth { rot_deg: 90.0, offset: V8_OFFSET, ..Truth::same() };
    let start = pitched_start(&kinematics::cam_mount(false, 90.0, -38.0), 0.5, 0.22);
    let start_r = start.1.r;
    let rig = rig_mounted(setup.clone(), truth, 90.0, start, [0.01, 0.0], lease).await;
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
        let (rig, s) = v8_recheck_rig(false).await;
        let (id, _) = grasp::start(&rig.sup, rig.sim.clone(), &tap(&rig).await).await.unwrap();
        wait_phase(&rig.sup, id, "transition", Some(true)).await;
        match case {
            "lost" => rig.sim.visible.store(false, Ordering::SeqCst),
            "displaced" => *rig.sim.shift.lock().unwrap() = [0.02, 0.0],
            _ => rig.sim.failing.store(true, Ordering::SeqCst),
        }
        let mut phases = Vec::new();
        let a = wait_done(&rig.sup, id, 20.0, |a| {
            let p = a.phase.clone().unwrap_or_default();
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
    let (rig2, _) = v8_recheck_rig(false).await;
    for (case, rig, phase) in [("centring", &rig1, "centring"), ("re-check", &rig2, "transition")] {
        let (id, _) = grasp::start(&rig.sup, rig.sim.clone(), &tap(rig).await).await.unwrap();
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

/// Test driver for the collision case: an ideal position servo reporting the model gravity
/// torque plus a residual the test sets (as tests/test_collision_guard.rs).
#[derive(Default)]
struct ArmState { q: [f32; 7], torqued: bool, extra: [f32; 6] }

#[derive(Clone, Default)]
struct TestArm(Arc<Mutex<ArmState>>);

impl armd::driver::ArmDriver for TestArm {
    fn capabilities(&self) -> armd::driver::DriverCapabilities {
        armd::driver::DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true, torque_feedforward: false, enable_feedback: true }
    }
    fn enable_state(&self) -> armd::driver::EnableState {
        if self.0.lock().unwrap().torqued { armd::driver::EnableState::Enabled } else { armd::driver::EnableState::Disabled }
    }
    fn is_feedback_valid(&self) -> bool { true }
    fn is_enabled(&self) -> bool { self.0.lock().unwrap().torqued }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); 7] }
    fn read_joint_states(&mut self) -> Result<Vec<armd::model::JointState>, armd::driver::ArmDriverError> {
        let s = self.0.lock().unwrap();
        let q6: [f32; 6] = std::array::from_fn(|i| s.q[i]);
        let g = armd::compute_gravity_torques(&q6).unwrap();
        Ok((0..7).map(|i| armd::model::JointState {
            position: s.q[i], velocity: 0.0, temperature: 30.0, torque: if i < 6 { g[i] + s.extra[i] } else { 0.0 },
        }).collect())
    }
    fn write_joint_targets(&mut self, targets: &[armd::driver::JointTarget]) -> Result<(), armd::driver::ArmDriverError> {
        let mut s = self.0.lock().unwrap();
        for (i, t) in targets.iter().enumerate().take(7) { s.q[i] = t.position; }
        Ok(())
    }
    fn set_torque(&mut self, enabled: bool) -> Result<(), armd::driver::ArmDriverError> {
        self.0.lock().unwrap().torqued = enabled;
        Ok(())
    }
}

fn v8_setup() -> (GraspSetup, Truth, ([f32; 7], Pose)) {
    let setup = GraspSetup { camera_tilt_deg: -38.0, ..GraspSetup::default() };
    let truth = Truth { rot_deg: 90.0, offset: V8_OFFSET, ..Truth::same() };
    (setup, truth, pitched_start(&kinematics::cam_mount(false, 90.0, -38.0), 0.5, 0.22))
}

#[tokio::test]
async fn camera_down_stop_during_the_transition() {
    let (setup, truth, start) = v8_setup();
    let rig = rig_mounted(setup, truth, 90.0, start, [0.0, 0.0], false).await;
    let (id, _) = grasp::start(&rig.sup, rig.sim.clone(), &tap(&rig).await).await.unwrap();
    wait_phase(&rig.sup, id, "transition", Some(true)).await;
    let t = Instant::now();
    {
        let mut inner = rig.sup.inner().write().await;
        inner.stop_motion();
        inner.request_macro_stop();
    }
    let a = wait_done(&rig.sup, id, 2.0, |_| {}).await;
    assert_eq!((a.state.as_str(), a.code.as_deref(), a.phase.as_deref()), ("cancelled", Some("STOPPED"), Some("transition")), "{a:?}");
    assert!(t.elapsed() < Duration::from_millis(500), "{:?}", t.elapsed());
    assert_idle_after(&rig.sup, id).await;
}

#[tokio::test]
async fn camera_down_lease_loss_during_the_transition() {
    let (setup, truth, start) = v8_setup();
    let rig = rig_mounted(setup, truth, 90.0, start, [0.0, 0.0], true).await;
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
    let t = Instant::now();
    let reply = common::ws_reply(&mut w.ws, json!({"type":"release_lease", "generation":w.generation}), &["lease_released"]).await;
    assert_eq!(reply["success"], true);
    let a = wait_done(&w.rig.sup, id, 2.0, |_| {}).await;
    assert_eq!((a.state.as_str(), a.code.as_deref(), a.phase.as_deref()), ("cancelled", Some("LEASE_LOST"), Some("transition")), "{a:?}");
    assert!(t.elapsed() < Duration::from_millis(500));
    assert_idle_after(&w.rig.sup, id).await;
}

#[tokio::test]
async fn camera_down_collision_during_the_transition_holds() {
    let (setup, truth, start) = v8_setup();
    let arm = TestArm::default();
    let rig = rig_driven(setup, truth, 90.0, start, [0.0, 0.0], false, Some(arm.clone())).await;
    let (id, _) = grasp::start(&rig.sup, rig.sim.clone(), &tap(&rig).await).await.unwrap();
    wait_phase(&rig.sup, id, "transition", Some(true)).await;
    arm.0.lock().unwrap().extra[1] = -20.0;
    let a = wait_done(&rig.sup, id, 2.0, |_| {}).await;
    assert_eq!((a.state.as_str(), a.code.as_deref(), a.phase.as_deref()), ("failed", Some("COLLISION"), Some("transition")), "{a:?}");
    assert!(rig.sup.inner().read().await.collision.hold.is_some(), "collision hold");
    assert_idle_after(&rig.sup, id).await;
    assert!(rig.sup.inner().read().await.macro_run.is_none());
}
