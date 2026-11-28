//! GU-R4 review-finding regression suite:
//! 1. (codex B1) Scalar width must come from the chosen view along the settled closing axis:
//!    widening during camera-down transition with Depth first in `width_sources` and no contour
//!    rejects pre-transition depth (`stale_pre_transition`) and refuses `OPENING`.
//! 2. (codex M1 = Fable M1) Mid-height `recentre` horizontal corrections: unconverged-after-moves
//!    re-runs the live gate on fresh frames (refuses `OPENING`), and lost-after-move refuses
//!    `NO_WIDTH_EVIDENCE` (never reuses stale pre-correction pass).
//! 3. (codex M2) Adaptive wrist settle between `snap_rev` and `snap_ins` is caught by re-gating
//!    `grasp_goal` on `snap_ins` before insertion begins.
//! 4. (Fable M2) Depth-yaw turn with `height_source == "setup"` does NOT reuse `admission_shifted`
//!    (settled axis differs by > 5 deg from `JawAdmission.closing_axis`) and falls to Rung 3 `box`.
//! 5. (Fable m5) Post-`recentre` live gate evaluates the same `width_sources` (including `Depth`)
//!    as the main gate.
//! 6. (Fable m6) Contour miss and scalar tangential offset > half-length refuse `TARGET_DISPLACED`.
#![allow(clippy::too_many_arguments)]

mod common;

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::chain::ArmModel;
use armd::grasp::{
    self, AcquireCameraDown, BoxFuture, CamModel, DepthFrameInfo, Detection, DetectionFrame,
    DetectionSource, GraspSetup, HeightSource, MaskFocus, WidthSource, YawSource,
};
use armd::kinematics::{self, Pose, Vec3};
use armd::model::CameraMount;
use armd::motion::ActionStatus;
use armd::{ArmConfig, ArmSupervisor};
use serde_json::json;

const INSPECT: [f64; 6] = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0];
const ARM: ArmModel = ArmModel::REBOT_DEV_ARM;
const OBJECT_J7: f32 = -1.5;

fn solve_look_down(config: &ArmConfig, tcp: Vec3) -> [f64; 6] {
    let arm = &config.arm_model;
    let goal = Pose::new(grasp::look_down(&arm.fk(&INSPECT).r), tcp);
    let lo = [-3.1, -2.05, -2.6, -3.1, -2.05, -6.2];
    let hi = [3.1, 2.05, 2.6, 3.1, 2.05, 6.2];
    let mut q = INSPECT;
    for _ in 0..200 {
        let sol = arm.solve_dls(&goal, &q, &lo, &hi, kinematics::PAR.w);
        q = sol.q;
        if sol.ok {
            return q;
        }
    }
    panic!("look-down start at {tcp:?} unreachable");
}

fn solve_camera_down(config: &ArmConfig, setup: &GraspSetup, lens: Vec3) -> [f64; 6] {
    let arm = &config.arm_model;
    let cam = CamModel::assumed(arm, setup, &config.camera_mount);
    let az = lens[1].atan2(lens[0]);
    let r0 = kinematics::mat_mul(&kinematics::axis_r(&[0.0, 0.0, 1.0], az), &grasp::look_down(&arm.fk(&INSPECT).r));
    let r = grasp::camera_down(&r0, &cam.rec);
    let goal = Pose::new(r, kinematics::sub(&lens, &kinematics::mat_vec(&r, &cam.offset)));
    let lo = [-3.1, -2.05, -2.6, -3.1, -2.05, -6.2];
    let hi = [3.1, 2.05, 2.6, 3.1, 2.05, 6.2];
    let mut q = INSPECT;
    q[0] = az;
    for _ in 0..200 {
        let sol = arm.solve_dls(&goal, &q, &lo, &hi, kinematics::PAR.w);
        q = sol.q;
        if sol.ok {
            return q;
        }
    }
    panic!("camera-down start at {lens:?} unreachable");
}

fn bar(c: [f64; 2], len: f64, wid: f64, deg: f64) -> Vec<[f64; 2]> {
    let (co, si) = (deg.to_radians().cos(), deg.to_radians().sin());
    let (hl, hw) = (len / 2.0, wid / 2.0);
    [[-hl, -hw], [hl, -hw], [hl, hw], [-hl, hw]]
        .into_iter()
        .map(|[x, y]| [c[0] + x * co - y * si, c[1] + x * si + y * co])
        .collect()
}

fn banana(c: [f64; 2], chord: f64, arc_deg: f64, wid: f64, deg: f64) -> Vec<[f64; 2]> {
    let a = arc_deg.to_radians();
    let rad = chord / (2.0 * (a / 2.0).sin());
    let (co, si) = (deg.to_radians().cos(), deg.to_radians().sin());
    let rot = |[x, y]: [f64; 2]| [c[0] + x * co - y * si, c[1] + x * si + y * co];
    let mut p = Vec::new();
    for i in 0..=16 {
        let t = -a / 2.0 + a * (i as f64 / 16.0);
        p.push(rot([(rad + wid / 2.0) * t.sin(), (rad + wid / 2.0) * t.cos() - rad]));
    }
    for i in (0..=16).rev() {
        let t = -a / 2.0 + a * (i as f64 / 16.0);
        p.push(rot([(rad - wid / 2.0) * t.sin(), (rad - wid / 2.0) * t.cos() - rad]));
    }
    p
}

fn project_poly(cam: &CamModel, pose: &Pose, table_z: f64, poly: &[[f64; 2]], h: f64) -> ([f64; 4], Vec<[f64; 2]>) {
    let mut b = [1.0f64, 1.0, 0.0, 0.0];
    for p in poly {
        for z in [table_z, table_z + h] {
            let n = cam.to_norm(cam.project(pose, &[p[0], p[1], z]).expect("vertex visible"));
            b = [b[0].min(n[0]), b[1].min(n[1]), b[2].max(n[0]), b[3].max(n[1])];
        }
    }
    let contour = poly
        .iter()
        .map(|p| cam.to_norm(cam.project(pose, &[p[0], p[1], table_z + h / 2.0]).expect("contour visible")))
        .collect();
    ([b[0].clamp(0.0, 1.0), b[1].clamp(0.0, 1.0), b[2].clamp(0.0, 1.0), b[3].clamp(0.0, 1.0)], contour)
}

#[derive(Clone)]
struct R4CamCfg {
    object_xy: [f64; 2],
    half_xy: [f64; 2],
    height_m: f64,
    table_z_m: f64,
    visible: bool,
    width_px: Option<f64>,
    depth_width_px: Option<f64>,
    depth_angle: Option<(f64, f64)>,
    contour_world: Option<Vec<[f64; 2]>>,
    box_override_norm: Option<[f64; 4]>,
    box_shift_xy: [f64; 2],
}

impl R4CamCfg {
    fn plain(half_xy: [f64; 2], height_m: f64, table_z_m: f64) -> Self {
        Self {
            object_xy: [0.30, 0.0],
            half_xy,
            height_m,
            table_z_m,
            visible: true,
            width_px: None,
            depth_width_px: None,
            depth_angle: None,
            contour_world: None,
            box_override_norm: None,
            box_shift_xy: [0.0, 0.0],
        }
    }
}

type FrameHook = Arc<dyn Fn(&Pose, &mut R4CamCfg) + Send + Sync>;

struct R4Camera {
    sup: Arc<ArmSupervisor>,
    cam: CamModel,
    cfg: Mutex<R4CamCfg>,
    per_request: bool,
    depth_available: bool,
    frame: AtomicU64,
    on_frame: Option<FrameHook>,
    /// Optional J6 delta (rad) applied synchronously to supervisor joint state immediately after
    /// returning the first frame while `nudge_armed` is true (zero scheduling race).
    nudge_j6: Option<(Arc<AtomicBool>, Arc<AtomicBool>, f32)>,
}

impl R4Camera {
    fn build_frame(&self, pose: &Pose, with_depth: bool) -> DetectionFrame {
        let mut cfg_guard = self.cfg.lock().unwrap();
        if let Some(ref hook) = self.on_frame {
            hook(pose, &mut cfg_guard);
        }
        let c = cfg_guard.clone();
        drop(cfg_guard);
        let fid = self.frame.fetch_add(1, Ordering::SeqCst) + 1;
        let now = Instant::now();
        if !c.visible {
            return DetectionFrame {
                frame_id: fid,
                captured_at: now,
                policy_captured_at: Some(now),
                image_size_px: self.cam.size,
                detections: vec![],
                depth: None,
                model_generation: None,
            };
        }
        let cx = c.object_xy[0] + c.box_shift_xy[0];
        let cy = c.object_xy[1] + c.box_shift_xy[1];
        let box_norm = if let Some(b) = c.box_override_norm {
            b
        } else if let Some(ref poly) = c.contour_world {
            let shifted: Vec<[f64; 2]> =
                poly.iter().map(|p| [p[0] + c.box_shift_xy[0], p[1] + c.box_shift_xy[1]]).collect();
            project_poly(&self.cam, pose, c.table_z_m, &shifted, c.height_m).0
        } else {
            let mut b = [1.0f64, 1.0, 0.0, 0.0];
            for sx in [-1.0, 1.0] {
                for sy in [-1.0, 1.0] {
                    for z in [c.table_z_m, c.table_z_m + c.height_m] {
                        let px = self.cam.to_norm(self.cam.project(pose, &[cx + sx * c.half_xy[0], cy + sy * c.half_xy[1], z]).expect("visible"));
                        b = [b[0].min(px[0]), b[1].min(px[1]), b[2].max(px[0]), b[3].max(px[1])];
                    }
                }
            }
            [b[0].clamp(0.0, 1.0), b[1].clamp(0.0, 1.0), b[2].clamp(0.0, 1.0), b[3].clamp(0.0, 1.0)]
        };
        let contour_norm = c.contour_world.as_ref().map(|poly| {
            Ok(poly
                .iter()
                .map(|p| {
                    self.cam.to_norm(
                        self.cam
                            .project(pose, &[p[0] + c.box_shift_xy[0], p[1] + c.box_shift_xy[1], c.table_z_m + c.height_m / 2.0])
                            .expect("contour visible"),
                    )
                })
                .collect())
        });
        let (depth, disp_fg, disp_bg, depth_conf, depth_angle_deg, depth_angle_conf, depth_width_px) = if with_depth {
            let o = self.cam.origin(pose);
            let r = self.cam.rotation(pose);
            let z_top = kinematics::dot(&kinematics::sub(&[cx, cy, c.table_z_m + c.height_m], &o), &kinematics::column(&r, 2)).max(0.08);
            let z_bg = kinematics::dot(&kinematics::sub(&[cx, cy, c.table_z_m], &o), &kinematics::column(&r, 2)).max(z_top + 0.01);
            let mut table_samples = Vec::new();
            for row in 0..4 {
                for col in 0..4 {
                    let u_n = 0.15 + 0.2 * col as f64;
                    let v_n = 0.2 + 0.2 * row as f64;
                    if let Some(pt) = self.cam.plane_point(pose, self.cam.to_px([u_n, v_n]), c.table_z_m) {
                        let zc = kinematics::dot(&kinematics::sub(&pt, &o), &kinematics::column(&r, 2));
                        if zc > 0.05 {
                            table_samples.push([u_n, v_n, 1.0 / zc]);
                        }
                    }
                }
            }
            let (da, dc) = c.depth_angle.map_or((None, None), |(a, cf)| (Some(a), Some(cf)));
            (Some(DepthFrameInfo { metric_model: true, table_samples }), Some(1.0 / z_top), Some(1.0 / z_bg), Some(0.95), da, dc, c.depth_width_px)
        } else {
            (None, None, None, None, None, None, None)
        };
        let det = Detection {
            label: "cup".into(),
            score: 0.95,
            raw_score_valid: true,
            box_norm,
            angle_deg: None,
            angle_conf: None,
            grasp_width_px: c.width_px,
            mask_focus: false,
            disp_fg,
            disp_bg,
            depth_conf,
            length_px: None,
            jaw_clear: with_depth.then_some(true),
            depth_angle_deg,
            depth_angle_conf,
            depth_width_px,
            contour_norm,
        };
        DetectionFrame {
            frame_id: fid,
            captured_at: now,
            policy_captured_at: Some(now),
            image_size_px: self.cam.size,
            detections: vec![det],
            depth,
            model_generation: None,
        }
    }
}

impl DetectionSource for R4Camera {
    fn per_request_pipelines(&self) -> BoxFuture<'_, Result<bool, String>> {
        let p = self.per_request;
        Box::pin(async move { Ok(p) })
    }
    fn depth_available(&self) -> BoxFuture<'_, Result<bool, String>> {
        let d = self.depth_available;
        Box::pin(async move { Ok(d) })
    }
    fn inference_enabled(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async { Ok(true) })
    }
    fn set_inference(&self, _enabled: bool) -> BoxFuture<'_, Result<(), String>> {
        Box::pin(async { Ok(()) })
    }
    fn next_frame(&self, _after: Option<u64>, _wait_ms: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        Box::pin(async move {
            tokio::time::sleep(Duration::from_millis(4)).await;
            let q = self.sup.inner().read().await.measured_positions.clone();
            let frame = self.build_frame(&ARM.fk(&kinematics::q6(&q)), false);
            if let Some((ref armed, ref fired, dq)) = self.nudge_j6 {
                if armed.load(Ordering::SeqCst) && !fired.swap(true, Ordering::SeqCst) {
                    let mut inner = self.sup.inner().write().await;
                    inner.commanded_positions[5] += dq;
                    inner.measured_positions[5] += dq;
                    inner.target_positions[5] += dq;
                }
            }
            Ok(Some(frame))
        })
    }
    fn trigger_with_focus(
        &self,
        pipelines: &'static [&'static str],
        _wait_ms: u64,
        _focus: Option<&MaskFocus>,
    ) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        let with_depth = pipelines.contains(&"depth");
        Box::pin(async move {
            tokio::time::sleep(Duration::from_millis(4)).await;
            let q = self.sup.inner().read().await.measured_positions.clone();
            let frame = self.build_frame(&ARM.fk(&kinematics::q6(&q)), with_depth);
            if let Some((ref armed, ref fired, dq)) = self.nudge_j6 {
                if armed.load(Ordering::SeqCst) && !fired.swap(true, Ordering::SeqCst) {
                    let mut inner = self.sup.inner().write().await;
                    inner.commanded_positions[5] += dq;
                    inner.measured_positions[5] += dq;
                    inner.target_positions[5] += dq;
                }
            }
            Ok(Some(frame))
        })
    }
}

fn spawn_ticker(sup: Arc<ArmSupervisor>, stop: Arc<AtomicBool>, hold_obj: bool) -> tokio::task::JoinHandle<()> {
    tokio::spawn(async move {
        while !stop.load(Ordering::Relaxed) {
            sup.tick(0.02).await;
            let mut inner = sup.inner().write().await;
            if hold_obj && inner.commanded_positions[6] > OBJECT_J7 && inner.measured_positions[6] >= OBJECT_J7 - 0.3 {
                inner.measured_positions[6] = OBJECT_J7;
                inner.measured_velocities[6] = 0.0;
                inner.torques[6] = 0.3;
            }
            drop(inner);
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
    })
}

async fn run_r4_grasp(
    cfg: ArmConfig,
    mut setup: GraspSetup,
    q0: [f64; 6],
    cam_cfg: R4CamCfg,
    per_request: bool,
    depth_available: bool,
    calibrate_grip: bool,
    on_frame: Option<FrameHook>,
    on_phase: Option<Arc<dyn Fn(&str, &R4Camera) + Send + Sync>>,
) -> (ActionStatus, Vec<String>) {
    let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
    let cam = CamModel::assumed(&cfg.arm_model, &setup, &cfg.camera_mount);
    let start = cfg.arm_model.fk(&q0);
    let q7: Vec<f32> = (0..7).map(|i| if i < 6 { q0[i] as f32 } else { 0.0 }).collect();
    let r4cam = Arc::new(R4Camera {
        sup: sup.clone(),
        cam: cam.clone(),
        cfg: Mutex::new(cam_cfg.clone()),
        per_request,
        depth_available,
        frame: AtomicU64::new(0),
        on_frame,
        nudge_j6: None,
    });
    if calibrate_grip {
        let app = Pose::new(
            grasp::look_down(&start.r),
            [cam_cfg.object_xy[0], cam_cfg.object_xy[1], setup.table_z_m + setup.approach_height_m],
        );
        setup.grip_pixel_norm = grasp::box_centre(&r4cam.build_frame(&app, false).detections[0].box_norm);
    }
    {
        let mut inner = sup.inner().write().await;
        inner.lease_enabled = false;
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).unwrap();
        armd::SupervisorInner::enable_torque(admitted).unwrap();
        inner.commanded_positions = q7.clone();
        inner.measured_positions = q7.clone();
        inner.target_positions = q7;
        inner.set_grasp_setup(setup);
    }
    let body = json!({"label": "cup", "box_norm": r4cam.build_frame(&start, false).detections[0].box_norm, "frame_id": 1});
    let stop = Arc::new(AtomicBool::new(false));
    let ticker = spawn_ticker(sup.clone(), stop.clone(), true);
    let (id, _) = grasp::start(&sup, sup.capture_admission_stamp(), r4cam.clone(), &body).await.expect("start accepted");
    let mut phases = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(25);
    let status = loop {
        assert!(Instant::now() < deadline, "R4 sim grasp timed out");
        let obs = sup.get_observation().await;
        let a = obs.action.clone().or(obs.previous_action.clone()).expect("action");
        if a.id == id {
            if let Some(p) = a.phase.as_deref() {
                if phases.last().map(String::as_str) != Some(p) {
                    phases.push(p.to_string());
                    if let Some(ref cb) = on_phase {
                        cb(p, &r4cam);
                    }
                }
            }
            if !a.is_running() {
                break a;
            }
        }
        tokio::time::sleep(Duration::from_millis(2)).await;
    };
    stop.store(true, Ordering::Relaxed);
    let _ = ticker.await;
    (status, phases)
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn item1_depth_first_widened_during_transition_refused_both_depth_only_and_depth_mask() {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.camera_mount = CameraMount {
        rot_deg: 0.0,
        tilt_deg: -16.0,
        offset_m: Some([-0.04, 0.0, 0.04]),
        ..CameraMount::default()
    };
    let base_setup = GraspSetup {
        camera_tilt_deg: -16.0,
        acquire_camera_down: AcquireCameraDown::On,
        approach_height_m: 0.12,
        height_probe_m: 0.0,
        height_sources: vec![HeightSource::Depth],
        align_j6: false,
        max_grasp_width_m: 0.10,
        ..GraspSetup::default()
    };
    let q0 = solve_camera_down(&cfg, &base_setup, [0.30, 0.0, 0.24]);

    // (a) Depth-only width_sources, no contour: object widens during transition -> pre-transition
    //     Depth width is rejected as `stale_pre_transition`, FreshView Rung 3 box refuses OPENING.
    {
        let mut setup = base_setup.clone();
        setup.width_sources = vec![WidthSource::Depth];
        let mut cam_cfg = R4CamCfg::plain([0.036, 0.014], 0.030, setup.table_z_m);
        cam_cfg.depth_width_px = Some(80.0); // ~24 mm pre-transition depth width
        let widened = Arc::new(AtomicBool::new(false));
        let cb: Arc<dyn Fn(&str, &R4Camera) + Send + Sync> = Arc::new(move |phase, cam| {
            if phase == "transition" && !widened.swap(true, Ordering::SeqCst) {
                cam.cfg.lock().unwrap().half_xy = [0.036, 0.036]; // 72 mm post-transition
            }
        });
        let (st, phases) = run_r4_grasp(cfg.clone(), setup, q0, cam_cfg, true, true, false, None, Some(cb)).await;
        assert!(st.final_offset_px.is_some(), "FreshView recheck must have run");
        assert_eq!((st.state.as_str(), st.code.as_deref()), ("failed", Some("OPENING")), "{st:?}");
        assert!(st.message.as_deref().unwrap_or("").contains("(box)"), "{:?}", st.message);
        assert!(
            st.width_estimates.iter().any(|e| e.source == "depth" && e.reason.as_deref() == Some("stale_pre_transition")),
            "expected stale_pre_transition in width_estimates: {:?}",
            st.width_estimates
        );
        assert!(!phases.iter().any(|p| p == "descend" || p == "close"), "{phases:?}");
    }

    // (b) [Depth, Mask] width_sources, no contour: FreshView reports wide mask width -> Depth
    //     rejected `stale_pre_transition`, Mask on FreshView decides and refuses OPENING (scalar).
    {
        let mut setup = base_setup;
        setup.width_sources = vec![WidthSource::Depth, WidthSource::Mask];
        let mut cam_cfg = R4CamCfg::plain([0.036, 0.014], 0.030, setup.table_z_m);
        cam_cfg.depth_width_px = Some(80.0);
        let widened = Arc::new(AtomicBool::new(false));
        let cb: Arc<dyn Fn(&str, &R4Camera) + Send + Sync> = Arc::new(move |phase, cam| {
            if phase == "transition" && !widened.swap(true, Ordering::SeqCst) {
                let mut g = cam.cfg.lock().unwrap();
                g.half_xy = [0.036, 0.014];
                g.width_px = Some(300.0); // wide fresh mask width on FreshView
            }
        });
        let (st, phases) = run_r4_grasp(cfg, setup, q0, cam_cfg, true, true, false, None, Some(cb)).await;
        assert_eq!((st.state.as_str(), st.code.as_deref()), ("failed", Some("OPENING")), "{st:?}");
        assert!(st.message.as_deref().unwrap_or("").contains("(scalar)"), "{:?}", st.message);
        assert!(!phases.iter().any(|p| p == "descend" || p == "close"), "{phases:?}");
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn item2_recentre_unconverged_and_lost_after_move_never_reuse_stale_pass() {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.camera_mount = CameraMount { offset_m: Some([-0.06, 0.0, 0.0]), ..CameraMount::default() };
    let q0 = solve_look_down(&cfg, [0.30, 0.0, 0.24]);
    let base_setup = GraspSetup {
        align_j6: false,
        height_probe_m: 0.0,
        recentre_height_m: 0.07,
        max_grasp_width_m: 0.10,
        ..GraspSetup::default()
    };

    // (a) Unconverged after 1 horizontal move: recentre moves 5 mm, object stays ~23 mm off-TCP ->
    //     fresh frames re-run the live gate and refuse OPENING (never descends to close).
    {
        let mut setup = base_setup.clone();
        setup.recentre_max_iterations = 1;
        setup.max_step_m = 0.005;
        let cam_cfg = R4CamCfg::plain([0.035, 0.016], 0.030, setup.table_z_m);
        let shifted = Arc::new(AtomicBool::new(false));
        let cb: Arc<dyn Fn(&str, &R4Camera) + Send + Sync> = Arc::new(move |phase, cam| {
            if phase == "recentre" && !shifted.swap(true, Ordering::SeqCst) {
                cam.cfg.lock().unwrap().box_shift_xy = [0.0, 0.028];
            }
        });
        let (st, phases) = run_r4_grasp(cfg.clone(), setup, q0, cam_cfg, false, false, true, None, Some(cb)).await;
        assert_eq!(st.recentre.as_deref(), Some("unconverged"), "{st:?}");
        assert_eq!(st.recentre_iterations, Some(1), "{st:?}");
        assert_eq!((st.state.as_str(), st.code.as_deref()), ("failed", Some("OPENING")), "{st:?}");
        assert!(!phases.iter().any(|p| p == "close"), "must refuse before close: {phases:?}");
    }

    // (b) Lost after 1 horizontal move: recentre moves laterally once, then target disappears ->
    //     must refuse NO_WIDTH_EVIDENCE instead of keeping the stale pre-correction gate pass.
    {
        let mut setup = base_setup;
        setup.recentre_max_iterations = 3;
        setup.max_step_m = 0.010;
        setup.lost_frames = 2;
        setup.lost_timeout_s = 0.05;
        let cam_cfg = R4CamCfg::plain([0.035, 0.016], 0.030, setup.table_z_m);
        let in_recentre = Arc::new(AtomicBool::new(false));
        let rec_start_y = Arc::new(Mutex::new(None::<f64>));
        let in_rec_cb = in_recentre.clone();
        let cb: Arc<dyn Fn(&str, &R4Camera) + Send + Sync> = Arc::new(move |phase, cam| {
            if phase == "recentre" && !in_rec_cb.swap(true, Ordering::SeqCst) {
                cam.cfg.lock().unwrap().box_shift_xy = [0.0, 0.020];
            }
        });
        let in_rec_f = in_recentre.clone();
        let start_y_f = rec_start_y.clone();
        let hook: FrameHook = Arc::new(move |pose, cam_cfg| {
            if in_rec_f.load(Ordering::SeqCst) {
                let mut sy = start_y_f.lock().unwrap();
                let y0 = *sy.get_or_insert(pose.p[1]);
                if (pose.p[1] - y0).abs() > 0.002 {
                    cam_cfg.visible = false;
                }
            }
        });
        let (st, phases) = run_r4_grasp(cfg, setup, q0, cam_cfg, false, false, true, Some(hook), Some(cb)).await;
        assert_eq!(st.recentre.as_deref(), Some("skipped:target_lost"), "{st:?}");
        assert_eq!(st.recentre_iterations, Some(1), "{st:?}");
        assert_eq!((st.state.as_str(), st.code.as_deref()), ("failed", Some("NO_WIDTH_EVIDENCE")), "{st:?}");
        assert!(!phases.iter().any(|p| p == "close"), "must refuse before close: {phases:?}");
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn item3_adaptive_wrist_settle_between_revalidate_and_insert_is_regated() {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.camera_mount = CameraMount { offset_m: Some([-0.08, 0.0, 0.0]), ..CameraMount::default() };
    let setup = GraspSetup {
        fx_px: 320.0,
        fy_px: 320.0,
        approach_speed_mps: 0.12,
        descend_speed_mps: 0.10,
        settle_delay_s: 0.01,
        ..GraspSetup::default()
    };
    let q0 = solve_look_down(&cfg, [0.30, 0.0, 0.26]);
    let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
    let cam = CamModel::assumed(&cfg.arm_model, &setup, &cfg.camera_mount);
    let q7: Vec<f32> = (0..7).map(|i| if i < 6 { q0[i] as f32 } else { 0.0 }).collect();
    {
        let mut inner = sup.inner().write().await;
        inner.lease_enabled = false;
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).unwrap();
        armd::SupervisorInner::enable_torque(admitted).unwrap();
        inner.commanded_positions = q7.clone();
        inner.measured_positions = q7.clone();
        inner.target_positions = q7;
        inner.set_grasp_setup(setup.clone());
    }
    let start = cfg.arm_model.fk(&q0);
    let poly_init = banana([0.30, 0.0], 0.12, 30.0, 0.034, 90.0);
    let (b0, c0) = project_poly(&cam, &start, setup.table_z_m, &poly_init, 0.035);
    let req = json!({
        "label": "cup",
        "box_norm": b0,
        "frame_id": 1,
        "strategy": "adaptive",
        "evidence": {"contour": c0, "height_m": [0.035, 0.035]},
    });
    let planned = {
        let inner = sup.inner().read().await;
        let pending = grasp::prepare(&inner, &req, inner.safety.epoch_id()).expect("prepare");
        drop(inner);
        grasp::plan(pending).expect("plan adaptive chord")
    };
    let route = &planned.adaptive.as_ref().expect("chord wins for banana").route;
    let contact_xy = route.contact_centre;
    let y0 = kinematics::column(&route.plan.grasp.r, 1);
    let u0 = armd::grasp_select::geometry::unit2([y0[0], y0[1]]);
    // Compute the closing axis after J6 settles by +2.2 deg (< 2.5 deg PLAN_STALE threshold).
    let dq = 2.2f64.to_radians();
    let mut q_rot = route.plan.q_grasp;
    q_rot[5] += dq;
    let y_rot = kinematics::column(&cfg.arm_model.fk(&q_rot).r, 1);
    let u_rot = armd::grasp_select::geometry::unit2([y_rot[0], y_rot[1]]);
    let t_raw = armd::grasp_select::geometry::perp2(u0);
    let sign = if armd::grasp_select::geometry::dot2(u_rot, t_raw) >= 0.0 { 1.0 } else { -1.0 };
    let t = [t_raw[0] * sign, t_raw[1] * sign];
    // Long bar matching the banana's 120 mm principal axis (< 2 deg tilt, passes 12 deg check)
    // plus a narrow tab at x_t in [+0.8 mm, +12 mm] extending from y_u = +24 mm to +44 mm:
    // - at 0 deg (snap_rev): closing line x_t = 0 misses the tab (0.8 mm clearance) and cuts
    //   [-14, +24] mm (reserve > 6 mm -> PASS revalidate);
    // - at +2.2 deg (snap_ins): at y_u = 24 mm, x_t = 24 * tan(2.2 deg) = +0.92 mm > 0.80 mm,
    //   so the closing line enters the tab -> +44 mm (refuses OPENING at insert).
    let mut notched: Vec<[f64; 2]> = [
        [-0.060, -0.014],
        [0.060, -0.014],
        [0.060, 0.024],
        [0.012, 0.024],
        [0.012, 0.044],
        [0.0008, 0.044],
        [0.0008, 0.024],
        [-0.060, 0.024],
    ]
    .into_iter()
    .map(|[xt, yu]| [contact_xy[0] + xt * t[0] + yu * u0[0], contact_xy[1] + xt * t[1] + yu * u0[1]])
    .collect();
    armd::grasp_select::geometry::ensure_ccw(&mut notched);

    let in_revalidate = Arc::new(AtomicBool::new(false));
    let settled_j6 = Arc::new(AtomicBool::new(false));
    let mut cam_cfg = R4CamCfg::plain([0.035, 0.016], 0.035, setup.table_z_m);
    cam_cfg.contour_world = Some(poly_init.clone());
    let cam_hook = cam.clone();
    let in_rev_h = in_revalidate.clone();
    let notched_h = notched.clone();
    let poly_h = poly_init.clone();
    let table_z = setup.table_z_m;
    let swapped = Arc::new(AtomicBool::new(false));
    let hook: FrameHook = Arc::new(move |pose, cc| {
        if in_rev_h.load(Ordering::SeqCst) && !swapped.swap(true, Ordering::SeqCst) {
            let (keep_box, _) = project_poly(&cam_hook, pose, table_z, &poly_h, 0.035);
            cc.contour_world = Some(notched_h.clone());
            cc.box_override_norm = Some(keep_box);
        }
    });
    let r4cam = Arc::new(R4Camera {
        sup: sup.clone(),
        cam: cam.clone(),
        cfg: Mutex::new(cam_cfg),
        per_request: true,
        depth_available: false,
        frame: AtomicU64::new(1),
        on_frame: Some(hook),
        nudge_j6: Some((in_revalidate.clone(), settled_j6.clone(), dq as f32)),
    });
    let stop = Arc::new(AtomicBool::new(false));
    let ticker = spawn_ticker(sup.clone(), stop.clone(), false);
    let adm = sup.capture_admission_stamp();
    let (id, _) = {
        let mut inner = sup.inner().write().await;
        grasp::begin_with_stamp(&mut inner, adm, &sup, r4cam, planned).expect("begin adaptive")
    };
    let mut phases = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(20);
    let status = loop {
        assert!(Instant::now() < deadline, "item3 adaptive test timed out");
        let obs = sup.get_observation().await;
        let a = obs.action.clone().or(obs.previous_action.clone()).expect("action");
        if a.id == id {
            if let Some(p) = a.phase.as_deref() {
                if phases.last().map(String::as_str) != Some(p) {
                    phases.push(p.to_string());
                }
                if p == "revalidate" {
                    in_revalidate.store(true, Ordering::SeqCst);
                }
            }
            if !a.is_running() {
                break a;
            }
        }
        tokio::time::sleep(Duration::from_millis(1)).await;
    };
    stop.store(true, Ordering::Relaxed);
    let _ = ticker.await;
    assert!(settled_j6.load(Ordering::SeqCst), "wrist settle must have fired during revalidate");
    assert_eq!(status.phase.as_deref(), Some("insert"), "must pass revalidate and refuse at insert: {status:?}");
    assert_eq!((status.state.as_str(), status.code.as_deref()), ("failed", Some("OPENING")), "{status:?}");
    assert!(!status.motions.iter().any(|m| m.phase == "insert"), "must refuse before any insert motion: {:?}", status.motions);
    assert!(!phases.iter().any(|p| p == "close"), "{phases:?}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn item4_depth_yaw_turn_with_setup_height_falls_to_box_not_admission_shifted() {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.camera_mount = CameraMount { offset_m: Some([-0.06, 0.0, 0.0]), ..CameraMount::default() };
    let setup = GraspSetup {
        align_j6: true,
        height_probe_m: 0.0,
        height_sources: vec![], // forces height_source == "setup"
        yaw_sources: vec![YawSource::Depth, YawSource::Box],
        width_sources: vec![], // force Rung 2b vs Rung 3
        recentre_height_m: 0.0,
        max_grasp_width_m: 0.10,
        ..GraspSetup::default()
    };
    let q0 = solve_look_down(&cfg, [0.30, 0.0, 0.24]);
    let cam = CamModel::assumed(&cfg.arm_model, &setup, &cfg.camera_mount);
    let app = Pose::new(
        grasp::look_down(&cfg.arm_model.fk(&q0).r),
        [0.30, 0.0, setup.table_z_m + setup.approach_height_m],
    );
    // Report depth_angle along world +y (the initial closing axis) so J6 turns ~90 deg onto world x
    // (the 72 mm long axis), > 5 deg away from JawAdmission.closing_axis.
    let p0 = cam.project(&app, &[0.30, 0.0, setup.table_z_m + 0.030]).unwrap();
    let p1 = cam.project(&app, &[0.30, 0.02, setup.table_z_m + 0.030]).unwrap();
    let depth_deg = (p1[1] - p0[1]).atan2(p1[0] - p0[0]).to_degrees();
    let mut cam_cfg = R4CamCfg::plain([0.036, 0.015], 0.030, setup.table_z_m);
    cam_cfg.depth_angle = Some((depth_deg, 0.92));

    let (st, phases) = run_r4_grasp(cfg, setup, q0, cam_cfg, true, true, true, None, None).await;
    assert_eq!(st.height_source.as_deref(), Some("setup"), "{st:?}");
    assert_eq!(st.align.as_deref(), Some("turned:depth"), "{st:?}");
    assert_eq!((st.state.as_str(), st.code.as_deref()), ("failed", Some("OPENING")), "{st:?}");
    assert!(
        st.message.as_deref().unwrap_or("").contains("(box)"),
        "depth-yaw turn > 5 deg must fall to Rung 3 (box), not admission_shifted: {:?}",
        st.message
    );
    assert!(!phases.iter().any(|p| p == "descend" || p == "close"), "{phases:?}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn item6_post_recentre_gate_uses_depth_width_source() {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.camera_mount = CameraMount { offset_m: Some([-0.06, 0.0, 0.0]), ..CameraMount::default() };
    let setup = GraspSetup {
        align_j6: false,
        height_probe_m: 0.0,
        height_sources: vec![HeightSource::Depth],
        width_sources: vec![WidthSource::Depth],
        recentre_height_m: 0.07,
        recentre_max_iterations: 1,
        max_step_m: 0.002,
        max_grasp_width_m: 0.10,
        ..GraspSetup::default()
    };
    let q0 = solve_look_down(&cfg, [0.30, 0.0, 0.24]);
    // Box is narrow (24 mm) so Rung 3 box would STILL pass at 13 mm offset; Depth width is ~50 mm
    // (167 px at z_top = 0.18 m, fx = 600 -> 50.1 mm: passes main gate, fails OPENING after recentre).
    let mut cam_cfg = R4CamCfg::plain([0.030, 0.012], 0.030, setup.table_z_m);
    cam_cfg.depth_width_px = Some(167.0);
    let shifted = Arc::new(AtomicBool::new(false));
    let cb: Arc<dyn Fn(&str, &R4Camera) + Send + Sync> = Arc::new(move |phase, cam| {
        if phase == "recentre" && !shifted.swap(true, Ordering::SeqCst) {
            cam.cfg.lock().unwrap().box_shift_xy = [0.0, 0.016];
        }
    });
    let (st, phases) = run_r4_grasp(cfg, setup, q0, cam_cfg, true, true, true, None, Some(cb)).await;
    assert_eq!(st.recentre.as_deref(), Some("unconverged"), "{st:?}");
    assert_eq!((st.state.as_str(), st.code.as_deref()), ("failed", Some("OPENING")), "{st:?}");
    let msg = st.message.as_deref().unwrap_or("");
    assert!(msg.contains("Post-recentre") && msg.contains("(scalar)"), "expected post-recentre scalar refusal from Depth width, got: {msg:?}");
    assert!(!phases.iter().any(|p| p == "close"), "{phases:?}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn item7_tangential_displacement_and_missed_contour_refuse_target_displaced() {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.camera_mount = CameraMount { offset_m: Some([-0.06, 0.0, 0.0]), ..CameraMount::default() };
    let mut setup = GraspSetup {
        align_j6: false,
        height_probe_m: 0.0,
        recentre_height_m: 0.0,
        max_grasp_width_m: 0.10,
        ..GraspSetup::default()
    };
    let q0 = solve_look_down(&cfg, [0.30, 0.0, 0.24]);
    let cam = CamModel::assumed(&cfg.arm_model, &setup, &cfg.camera_mount);
    let app = Pose::new(
        grasp::look_down(&cfg.arm_model.fk(&q0).r),
        [0.30, 0.0, setup.table_z_m + setup.approach_height_m],
    );
    // Shift grip pixel +28 mm along tangential x (half_len = 18 mm) with 0 mm closing-axis offset.
    setup.grip_pixel_norm = cam.to_norm(cam.project(&app, &[0.328, 0.0, setup.table_z_m + 0.030]).unwrap());

    // (a) Rung 1 contour miss: closing line through TCP misses fresh contour -> TARGET_DISPLACED.
    {
        let mut cam_cfg = R4CamCfg::plain([0.018, 0.015], 0.030, setup.table_z_m);
        cam_cfg.contour_world = Some(bar([0.30, 0.0], 0.036, 0.030, 0.0));
        let (st, phases) = run_r4_grasp(cfg.clone(), setup.clone(), q0, cam_cfg, false, false, false, None, None).await;
        assert_eq!((st.state.as_str(), st.code.as_deref()), ("failed", Some("TARGET_DISPLACED")), "{st:?}");
        assert!(!phases.iter().any(|p| p == "descend" || p == "close"), "{phases:?}");
    }

    // (b) Rung 2 scalar tangential offset > half-length -> TARGET_DISPLACED.
    {
        let mut cam_cfg = R4CamCfg::plain([0.018, 0.015], 0.030, setup.table_z_m);
        cam_cfg.width_px = Some(100.0);
        let (st, phases) = run_r4_grasp(cfg, setup, q0, cam_cfg, false, false, false, None, None).await;
        assert_eq!((st.state.as_str(), st.code.as_deref()), ("failed", Some("TARGET_DISPLACED")), "{st:?}");
        assert!(!phases.iter().any(|p| p == "descend" || p == "close"), "{phases:?}");
    }
}
