//! Tap-to-grasp action (`/api/v1/action/grasp`) against a simulated camera.
//!
//! The simulated camera projects a box-shaped object at a fixed table position through the
//! current measured FK pose, its own camera mount and pinhole intrinsics. The controller
//! assumes different (rough) intrinsics and mount, as on the real arm.
mod common;

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::grasp::{self, BoxFuture, CamModel, Detection, DetectionFrame, DetectionSource, GraspDeps, GraspSetup};
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
}

impl SimCamera {
    fn project_box(&self, pose: &Pose) -> Option<[f64; 4]> {
        let o = &self.object;
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
    fn next_frame(&self, _after: Option<u64>, wait_ms: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        Box::pin(async move {
            if !self.enabled.load(Ordering::SeqCst) {
                tokio::time::sleep(Duration::from_millis(wait_ms)).await;
                return Ok(None);
            }
            tokio::time::sleep(Duration::from_millis(15)).await;
            let q = self.sup.inner().read().await.measured_positions.clone();
            let pose = kinematics::fk(&kinematics::q6(&q));
            let mut detections = vec![Detection { label: "person".into(), score: 0.5, box_norm: [0.0, 0.0, 0.08, 0.1] }];
            if self.visible.load(Ordering::SeqCst) {
                if let Some(b) = self.project_box(&pose) { detections.push(Detection { label: self.label.clone(), score: 0.9, box_norm: b }); }
            }
            let frame_id = self.frame.fetch_add(1, Ordering::SeqCst) + 1;
            Ok(Some(DetectionFrame { frame_id, captured_at: Instant::now(), detections }))
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
    let goal = Pose::new(r, [p0[0], p0[1], z]);
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
    let mut config = ArmConfig::rebot_dm();
    // the controller knows the optical centre only when the truth says so (`Truth::same`)
    let offset_m = truth.known_offset.then(|| truth.offset.map(|v| v as f32));
    config.camera_mount = CameraMount { tilt_deg: setup.camera_tilt_deg as f32, offset_m, ..CameraMount::default() };
    let sup = Arc::new(ArmSupervisor::new(config));
    let (q, start) = look_down_start(start_tilt_deg, 0.24);
    {
        let mut inner = sup.inner().write().await;
        inner.lease_enabled = lease;
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
        assert!(inner.grasp_run.is_none(), "grasp run cleared");
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
        inner.request_grasp_stop();
    }).await;
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("cancelled", Some("STOPPED")), "{a:?}");
    let a = cancel_case("centring", Some(false), async |rig: &Rig| {
        let mut inner = rig.sup.inner().write().await;
        inner.stop_motion();
        inner.request_grasp_stop();
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
    let rig = rig(GraspSetup::default(), Truth::same(), 0.0, [0.0, 0.0], true).await;
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
