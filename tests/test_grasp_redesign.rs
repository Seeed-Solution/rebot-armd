//! GU-R3 redesign verification suite:
//! 1. Plan parity sweep (>= 60 cases) covering deferred box-only, heavy payload, short
//!    `phase_timeout_s`, tiny search budget (`top_advisory`), and both owners.
//! 2. Sim dispatch parity (`strategy: "legacy"` vs `strategy: "adaptive"` forced `-top`):
//!    identical phase sequence, joint trajectories within 1e-6 rad, preopen before descend,
//!    fault count = 0.
//! 3. Offset-contour interval jaw refusal at all 3 call sites: planning admission, live legacy
//!    gate, and adaptive revalidate with rotated settled wrist.
//! 4. Rotated / widened object during camera-down transition -> refused before descent on both
//!    legacy and adaptive forced `-top`.
//! 5. One case per live-gate evidence rung (`contour`, `scalar`, `admission_shifted`, `box`,
//!    and absent evidence -> `NO_WIDTH_EVIDENCE`).
#![allow(clippy::too_many_arguments)]

mod common;

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::chain::ArmModel;
use armd::grasp::{
    self, AcquireCameraDown, BoxFuture, CamModel, Detection, DetectionFrame, DetectionSource,
    GraspSetup, PreOpen,
};
use armd::kinematics::{self, Pose, Vec3};
use armd::model::CameraMount;
use armd::motion::ActionStatus;
use armd::{ArmConfig, ArmSupervisor};
use serde_json::{json, Map, Value};

const INSPECT: [f64; 6] = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0];
const ARM: ArmModel = ArmModel::REBOT_DEV_ARM;
const OBJECT_J7: f32 = -1.5;

struct NoSource;

impl DetectionSource for NoSource {
    fn inference_enabled(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async { Ok(false) })
    }
    fn set_inference(&self, _enabled: bool) -> BoxFuture<'_, Result<(), String>> {
        Box::pin(async { Ok(()) })
    }
    fn next_frame(&self, _after: Option<u64>, _wait_ms: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        Box::pin(async { Ok(None) })
    }
}

struct Scene {
    sup: Arc<ArmSupervisor>,
    start: Pose,
    cam: CamModel,
    setup: GraspSetup,
}

fn mounted() -> (ArmConfig, GraspSetup) {
    let mut c = ArmConfig::rebot_dm();
    c.camera_mount = CameraMount {
        rot_deg: 90.0,
        tilt_deg: -38.0,
        offset_m: Some([-0.0731, 0.0005, 0.0660]),
        ..CameraMount::default()
    };
    let s = GraspSetup { fx_px: 824.0, fy_px: 824.0, camera_tilt_deg: -38.0, ..GraspSetup::default() };
    (c, s)
}

fn solve_camera_down(config: &ArmConfig, setup: &GraspSetup, lens: Vec3) -> [f64; 6] {
    let arm = &config.arm_model;
    let cam = CamModel::assumed(arm, setup, &config.camera_mount);
    let az = lens[1].atan2(lens[0]);
    let r0 = kinematics::mat_mul(&kinematics::axis_r(&[0.0, 0.0, 1.0], az), &grasp::look_down(&arm.fk(&INSPECT).r));
    let r = grasp::camera_down(&r0, &cam.rec);
    let off = kinematics::mat_vec(&r, &cam.offset);
    let goal = Pose::new(r, kinematics::sub(&lens, &off));
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

fn solve_look_down(config: &ArmConfig, tcp: Vec3) -> [f64; 6] {
    let arm = &config.arm_model;
    let r = grasp::look_down(&arm.fk(&INSPECT).r);
    let goal = Pose::new(r, tcp);
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

async fn scene_q(config: ArmConfig, setup: GraspSetup, q: [f64; 6]) -> Scene {
    let sup = Arc::new(ArmSupervisor::new(config));
    let q7: Vec<f32> = (0..7).map(|i| if i < 6 { q[i] as f32 } else { 0.0 }).collect();
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
    let inner = sup.inner().read().await;
    let s = inner.grasp_setup().unwrap();
    let cam = CamModel::assumed(&inner.config.arm_model, &s, &inner.config.camera_mount);
    let start = inner.config.arm_model.fk(&q);
    drop(inner);
    Scene { sup, start, cam, setup: s }
}

async fn mounted_scene(r: f64, az_deg: f64, cam_z: f64) -> Scene {
    let (c, s) = mounted();
    let a = az_deg.to_radians();
    let q = solve_camera_down(&c, &s, [r * a.cos(), r * a.sin(), cam_z]);
    scene_q(c, s, q).await
}

fn bar(mid: [f64; 2], len: f64, w: f64, turn_deg: f64) -> Vec<[f64; 2]> {
    let (s, c) = turn_deg.to_radians().sin_cos();
    [[-len / 2.0, -w / 2.0], [len / 2.0, -w / 2.0], [len / 2.0, w / 2.0], [-len / 2.0, w / 2.0]]
        .iter()
        .map(|p| [mid[0] + p[0] * c - p[1] * s, mid[1] + p[0] * s + p[1] * c])
        .collect()
}

fn banana(mid: [f64; 2], rc: f64, half_deg: f64, w: f64, turn_deg: f64) -> Vec<[f64; 2]> {
    let n = 12;
    let (h, t) = (half_deg.to_radians(), turn_deg.to_radians());
    let mut pts = Vec::new();
    for k in 0..=n {
        let a = -h + 2.0 * h * k as f64 / n as f64;
        pts.push([(rc + w / 2.0) * a.cos(), (rc + w / 2.0) * a.sin()]);
    }
    for k in (0..=n).rev() {
        let a = -h + 2.0 * h * k as f64 / n as f64;
        pts.push([(rc - w / 2.0) * a.cos(), (rc - w / 2.0) * a.sin()]);
    }
    pts.iter()
        .map(|p| {
            let x = p[0] - rc;
            [mid[0] + x * t.cos() - p[1] * t.sin(), mid[1] + x * t.sin() + p[1] * t.cos()]
        })
        .collect()
}

/// Asymmetric L-shaped CCW polygon centred at `mid`: axis-aligned bounding box is centred at
/// `mid` (y in [-0.038, +0.038]), while the cross-section through `mid` along +y spans
/// `[-0.005, +0.038]` (width 43 mm passes scalar check, +38 mm fails interval OPENING).
fn offset_l_poly(mid: [f64; 2]) -> Vec<[f64; 2]> {
    let pts = [
        [-0.040, -0.005],
        [0.034, -0.005],
        [0.034, -0.038],
        [0.040, -0.038],
        [0.040, 0.038],
        [-0.040, 0.038],
    ];
    pts.iter().map(|p| [mid[0] + p[0], mid[1] + p[1]]).collect()
}

fn project_poly_box_and_contour(
    cam: &CamModel,
    pose: &Pose,
    table_z: f64,
    poly: &[[f64; 2]],
    h: f64,
) -> ([f64; 4], Vec<[f64; 2]>) {
    let mut b = [1.0f64, 1.0, 0.0, 0.0];
    for z in [table_z, table_z + h] {
        for v in poly {
            let px = cam.to_norm(cam.project(pose, &[v[0], v[1], z]).expect("vertex visible"));
            b = [b[0].min(px[0]), b[1].min(px[1]), b[2].max(px[0]), b[3].max(px[1])];
        }
    }
    let contour = poly
        .iter()
        .map(|v| cam.to_norm(cam.project(pose, &[v[0], v[1], table_z + h / 2.0]).unwrap()))
        .collect();
    (b, contour)
}

fn request_for(sc: &Scene, poly: &[[f64; 2]], h: f64) -> Value {
    let (b, contour) = project_poly_box_and_contour(&sc.cam, &sc.start, sc.setup.table_z_m, poly, h);
    json!({
        "label": "cup",
        "box_norm": b,
        "frame_id": 7,
        "dry_run": true,
        "strategy": "adaptive",
        "evidence": {"contour": contour, "height_m": [h, h]},
    })
}

fn legacy_of(body: &Value) -> Value {
    let mut b = body.clone();
    b.as_object_mut().unwrap().remove("strategy");
    b
}

async fn dry(sc: &Scene, body: &Value) -> Map<String, Value> {
    grasp::dry_run_task(&sc.sup, &NoSource, body).await.fields()
}

fn assert_top_down_contract(ad: &Map<String, Value>, expect_advisory: Option<(&str, &str)>, deferred: bool) {
    assert_eq!(ad["reachable"], json!(true), "{ad:?}");
    assert_eq!(ad["pregrasp_mode"], json!("legacy"), "{ad:?}");
    let sc = &ad["selected_candidate"];
    assert!(sc["id"].as_str().unwrap().ends_with("-top"), "{sc}");
    assert_eq!(sc["approach_class"], json!("vertical"), "{sc}");
    assert_eq!(sc["tilt_deg"], json!(0.0), "{sc}");
    assert!(ad["expected_duration_s"].as_f64().is_some_and(|d| d > 0.0), "{ad:?}");
    assert!(ad["checked_phases"].as_array().is_some_and(|p| !p.is_empty()), "{ad:?}");
    assert!(ad["search"]["fully_checked"].as_u64().unwrap_or(0) >= 1, "{ad:?}");
    if deferred {
        assert_eq!(sc["jaw"]["status"], json!("deferred"), "{sc}");
        assert_eq!(sc["jaw"]["evidence"], json!("deferred"), "{sc}");
    } else {
        assert_eq!(sc["jaw"]["status"], json!("checked"), "{sc}");
        assert!(ad["margins"]["min_opening_m"].is_number(), "{ad:?}");
    }
    match expect_advisory {
        Some((code, phase)) => {
            assert_eq!(ad["top_advisory"]["code"], json!(code), "{ad:?}");
            assert_eq!(ad["top_advisory"]["phase"], json!(phase), "{ad:?}");
        }
        None => assert!(!ad.contains_key("top_advisory"), "top_advisory must be omitted when None: {ad:?}"),
    }
}

// ============================================================================
// 1. Plan parity sweep (>= 60 cases): deferred, heavy payload, short timeout,
//    tiny budget, shallow heights, wide refusals, both owners
// ============================================================================

async fn sweep_stress_cases(total: &mut usize, legacy_ok: &mut usize, top_selected: &mut usize) {
    for pos in [[0.28f64, 0.0], [0.30, 0.0]] {
        let sc = mounted_scene(pos[0], 0.0, 0.26).await;
        for angle in [35.0, 45.0, 55.0] {
            let mut body = request_for(&sc, &bar(pos, 0.09, 0.03, angle), 0.035);
            body.as_object_mut().unwrap().remove("evidence");
            let (leg, ad) = (dry(&sc, &legacy_of(&body)).await, dry(&sc, &body).await);
            *total += 1;
            assert_eq!((leg["reachable"].clone(), leg["jaw"]["status"].clone()), (json!(true), json!("deferred")), "{leg:?}");
            assert_eq!((ad["legacy"]["reachable"].clone(), ad["reachable"].clone()), (json!(true), json!(true)), "{ad:?}");
            if ad["selected_candidate"]["id"].as_str().is_some_and(|s| s.ends_with("-top")) {
                assert_top_down_contract(&ad, None, true);
                *top_selected += 1;
            }
            *legacy_ok += 1;
        }
        for h in [0.025, 0.035] {
            let mut body = request_for(&sc, &bar(pos, 0.08, 0.03, 0.0), h);
            body["evidence"]["payload_kg"] = json!(4.0);
            let (leg, ad) = (dry(&sc, &legacy_of(&body)).await, dry(&sc, &body).await);
            *total += 1;
            assert_eq!(leg["reachable"], json!(true));
            assert_top_down_contract(&ad, Some(("LIFT_TORQUE", "lift")), false);
            *legacy_ok += 1;
            *top_selected += 1;
        }
        for timeout_s in [1.0, 1.5] {
            let mut body = request_for(&sc, &bar(pos, 0.08, 0.03, 0.0), 0.035);
            body["overrides"] = json!({"phase_timeout_s": timeout_s, "timeout_s": timeout_s});
            let (leg, ad) = (dry(&sc, &legacy_of(&body)).await, dry(&sc, &body).await);
            *total += 1;
            assert_eq!((leg["reachable"].clone(), ad["reachable"].clone()), (json!(true), json!(true)), "{ad:?}");
            assert!(ad["selected_candidate"]["id"].as_str().unwrap().ends_with("-top"));
            assert_eq!(ad["top_advisory"]["code"], json!("GRASP_TIMEOUT"), "{ad:?}");
            *legacy_ok += 1;
            *top_selected += 1;
        }
        for h in [0.025, 0.035] {
            let mut body = request_for(&sc, &bar(pos, 0.08, 0.03, 0.0), h);
            body["budget_ms"] = json!(1);
            let (leg, ad) = (dry(&sc, &legacy_of(&body)).await, dry(&sc, &body).await);
            *total += 1;
            assert_eq!((leg["reachable"].clone(), ad["reachable"].clone()), (json!(true), json!(true)), "{ad:?}");
            assert!(ad["selected_candidate"]["id"].as_str().unwrap().ends_with("-top"));
            if let Some(adv) = ad.get("top_advisory") {
                assert_eq!((adv["code"].clone(), adv["phase"].clone()), (json!("BUDGET_EXHAUSTED"), json!("advisory")));
            }
            *legacy_ok += 1;
            *top_selected += 1;
        }
    }
}

async fn sweep_executor_cases(total: &mut usize, legacy_ok: &mut usize) {
    let (cfg, setup) = mounted();
    let q0 = solve_camera_down(&cfg, &setup, [0.30, 0.0, 0.26]);
    let sc_exec = scene_q(cfg.clone(), setup.clone(), q0).await;
    let handle = sc_exec
        .sup
        .start_executor(armd::rt_core::executor::ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .expect("start executor");
    for _ in 0..100 {
        if sc_exec.sup.hub().latest().is_some_and(|s| !s.rt.owner.boot.is_zero()) {
            break;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    for (w, deferred, payload, timeout) in [
        (0.028, false, None, None),
        (0.034, false, Some(4.0), None),
        (0.034, false, None, Some(1.0)),
        (0.030, true, None, None),
        (0.066, false, None, None),
        (0.025, false, None, None),
        (0.036, false, Some(4.0), None),
        (0.030, true, None, Some(1.0)),
        (0.044, false, None, None),
        (0.068, false, None, None),
    ] {
        let mut body = request_for(&sc_exec, &bar([0.30, 0.0], 0.085, w, if deferred { 45.0 } else { 0.0 }), 0.035);
        if deferred {
            body.as_object_mut().unwrap().remove("evidence");
        }
        if let Some(p) = payload {
            body["evidence"]["payload_kg"] = json!(p);
        }
        if let Some(t) = timeout {
            body["overrides"] = json!({"phase_timeout_s": t, "timeout_s": t});
        }
        let (leg, ad) = (dry(&sc_exec, &legacy_of(&body)).await, dry(&sc_exec, &body).await);
        *total += 1;
        assert_eq!(ad["legacy"]["reachable"], leg["reachable"]);
        if leg["reachable"] == json!(true) {
            *legacy_ok += 1;
            assert_eq!(ad["reachable"], json!(true), "executor parity failed: {ad:?}");
        }
    }
    handle.stop_and_join();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn plan_parity_sweep_deferred_heavy_timeout_budget_both_owners() {
    let (mut total, mut legacy_ok, mut top_selected) = (0usize, 0usize, 0usize);
    for pos in [[0.24f64, 0.0], [0.28, 0.03], [0.30, 0.0], [0.34, -0.02]] {
        let sc = mounted_scene(pos[0].hypot(pos[1]), pos[1].atan2(pos[0]).to_degrees(), 0.26).await;
        for h in [0.015, 0.030, 0.045] {
            for w in [0.022, 0.038, 0.066] {
                let body = request_for(&sc, &bar(pos, 0.075, w, 0.0), h);
                let (leg, ad) = (dry(&sc, &legacy_of(&body)).await, dry(&sc, &body).await);
                total += 1;
                assert_eq!(ad["legacy"]["reachable"], leg["reachable"], "pos={pos:?} h={h} w={w}");
                assert_eq!(ad["legacy"]["code"], leg["code"], "pos={pos:?} h={h} w={w}");
                if leg["reachable"] == json!(true) {
                    legacy_ok += 1;
                    assert_eq!(ad["reachable"], json!(true), "pos={pos:?} h={h} w={w}: {ad:?}");
                    if ad["selected_candidate"]["id"].as_str().is_some_and(|s| s.ends_with("-top")) {
                        top_selected += 1;
                    }
                }
            }
        }
    }
    sweep_stress_cases(&mut total, &mut legacy_ok, &mut top_selected).await;
    sweep_executor_cases(&mut total, &mut legacy_ok).await;
    assert!(total >= 60, "expected >= 60 sweep cases, got {total}");
    assert!(legacy_ok >= 40, "expected >= 40 passing cases, got {legacy_ok}");
    assert!(top_selected >= 10, "expected >= 10 -top selections, got {top_selected}");
}

// ============================================================================
// 2. Live mock camera & deterministic sim driver for dispatch / gate / recheck
// ============================================================================

#[derive(Clone)]
struct LiveCamCfg {
    object_xy: [f64; 2],
    half_xy: [f64; 2],
    height_m: f64,
    width_px: Option<f64>,
    angle: Option<(f64, f64)>,
    contour_world: Option<Vec<[f64; 2]>>,
    box_override_norm: Option<[f64; 4]>,
    box_shift_xy: [f64; 2],
}

struct FlexCamera {
    sup: Arc<ArmSupervisor>,
    cam: CamModel,
    cfg: Mutex<LiveCamCfg>,
    frame: AtomicU64,
}

impl FlexCamera {
    fn det_at(&self, pose: &Pose) -> Detection {
        let c = self.cfg.lock().unwrap().clone();
        let cx = c.object_xy[0] + c.box_shift_xy[0];
        let cy = c.object_xy[1] + c.box_shift_xy[1];
        let box_norm = if let Some(b) = c.box_override_norm {
            b
        } else if let Some(ref poly) = c.contour_world {
            let shifted: Vec<[f64; 2]> =
                poly.iter().map(|p| [p[0] + c.box_shift_xy[0], p[1] + c.box_shift_xy[1]]).collect();
            project_poly_box_and_contour(&self.cam, pose, 0.0, &shifted, c.height_m).0
        } else {
            let mut b = [1.0f64, 1.0, 0.0, 0.0];
            for sx in [-1.0, 1.0] {
                for sy in [-1.0, 1.0] {
                    for z in [0.0, c.height_m] {
                        let p = [cx + sx * c.half_xy[0], cy + sy * c.half_xy[1], z];
                        let px = self.cam.to_norm(self.cam.project(pose, &p).expect("visible"));
                        b = [b[0].min(px[0]), b[1].min(px[1]), b[2].max(px[0]), b[3].max(px[1])];
                    }
                }
            }
            [b[0].clamp(0.0, 1.0), b[1].clamp(0.0, 1.0), b[2].clamp(0.0, 1.0), b[3].clamp(0.0, 1.0)]
        };
        let contour_norm = c.contour_world.as_ref().map(|poly| {
            let pts: Vec<[f64; 2]> = poly
                .iter()
                .map(|p| {
                    self.cam.to_norm(
                        self.cam
                            .project(pose, &[p[0] + c.box_shift_xy[0], p[1] + c.box_shift_xy[1], c.height_m / 2.0])
                            .expect("contour visible"),
                    )
                })
                .collect();
            Ok(pts)
        });
        let (angle_deg, angle_conf) = match c.angle {
            Some((a, conf)) => (Some(a), Some(conf)),
            None => (None, None),
        };
        Detection {
            label: "cup".into(),
            score: 0.95,
            raw_score_valid: true,
            box_norm,
            angle_deg,
            angle_conf,
            grasp_width_px: c.width_px,
            mask_focus: false,
            disp_fg: None,
            disp_bg: None,
            depth_conf: None,
            length_px: None,
            jaw_clear: None,
            depth_angle_deg: None,
            depth_angle_conf: None,
            depth_width_px: None,
            contour_norm,
        }
    }
}

impl DetectionSource for FlexCamera {
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
            let pose = ARM.fk(&kinematics::q6(&q));
            let det = self.det_at(&pose);
            let fid = self.frame.fetch_add(1, Ordering::SeqCst) + 1;
            let now = Instant::now();
            Ok(Some(DetectionFrame {
                frame_id: fid,
                captured_at: now,
                policy_captured_at: Some(now),
                image_size_px: self.cam.size,
                detections: vec![det],
                depth: None,
                model_generation: None,
            }))
        })
    }
}

struct SimRun {
    status: ActionStatus,
    phases: Vec<String>,
    motion_q: Vec<[f32; 7]>,
    fault_count: usize,
}

fn spawn_sim_ticker(
    sup: Arc<ArmSupervisor>,
    stop: Arc<AtomicBool>,
    with_object_stop: bool,
) -> (
    tokio::task::JoinHandle<()>,
    Arc<Mutex<Vec<[f32; 7]>>>,
    Arc<std::sync::atomic::AtomicUsize>,
) {
    let motion_log: Arc<Mutex<Vec<[f32; 7]>>> = Arc::new(Mutex::new(Vec::new()));
    let fault_count = Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let (tlog, tfault) = (motion_log.clone(), fault_count.clone());
    let handle = tokio::spawn(async move {
        let mut last_q: Option<[f32; 7]> = None;
        while !stop.load(Ordering::Relaxed) {
            sup.tick(0.02).await;
            let mut inner = sup.inner().write().await;
            if inner.safety.fault_code().is_some() {
                tfault.fetch_add(1, Ordering::Relaxed);
            }
            if with_object_stop
                && inner.commanded_positions[6] > OBJECT_J7
                && inner.measured_positions[6] >= OBJECT_J7 - 0.3
            {
                inner.measured_positions[6] = OBJECT_J7;
                inner.measured_velocities[6] = 0.0;
                inner.torques[6] = 0.3;
            } else if with_object_stop {
                inner.torques[6] = 0.0;
            }
            let q: [f32; 7] = std::array::from_fn(|i| inner.commanded_positions[i]);
            if last_q.is_none_or(|p| (0..7).any(|i| (p[i] - q[i]).abs() > 1e-7)) {
                tlog.lock().unwrap().push(q);
                last_q = Some(q);
            }
            drop(inner);
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
    });
    (handle, motion_log, fault_count)
}

async fn run_sim_grasp(
    mut cfg: ArmConfig,
    mut setup: GraspSetup,
    q0: [f64; 6],
    cam_cfg: LiveCamCfg,
    req_patch: impl FnOnce(&mut Value, &CamModel, &Pose),
    on_phase: Option<Arc<dyn Fn(&str, &FlexCamera) + Send + Sync>>,
) -> SimRun {
    if cfg.camera_mount.offset_m.is_none() {
        cfg.camera_mount.offset_m = Some([-0.06, 0.0, 0.0]);
    }
    let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
    let cam = CamModel::assumed(&cfg.arm_model, &setup, &cfg.camera_mount);
    let start = cfg.arm_model.fk(&q0);
    let q7: Vec<f32> = (0..7).map(|i| if i < 6 { q0[i] as f32 } else { 0.0 }).collect();
    let flex = Arc::new(FlexCamera {
        sup: sup.clone(),
        cam: cam.clone(),
        cfg: Mutex::new(cam_cfg.clone()),
        frame: AtomicU64::new(0),
    });
    let app = Pose::new(
        grasp::look_down(&start.r),
        [cam_cfg.object_xy[0], cam_cfg.object_xy[1], setup.table_z_m + setup.approach_height_m],
    );
    if !grasp::camera_down_mode(&setup, &cfg.camera_mount).unwrap_or(false) {
        setup.grip_pixel_norm = grasp::box_centre(&flex.det_at(&app).box_norm);
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
    let mut body = json!({"label": "cup", "box_norm": flex.det_at(&start).box_norm, "frame_id": 1});
    req_patch(&mut body, &cam, &start);

    let stop = Arc::new(AtomicBool::new(false));
    let (ticker, motion_log, fault_count) = spawn_sim_ticker(sup.clone(), stop.clone(), true);
    let (id, _) = grasp::start(&sup, sup.capture_admission_stamp(), flex.clone(), &body)
        .await
        .expect("grasp accepted");
    let mut phases: Vec<String> = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(30);
    let status = loop {
        assert!(Instant::now() < deadline, "sim grasp timed out");
        let obs = sup.get_observation().await;
        let a = obs.action.clone().or(obs.previous_action.clone()).expect("action");
        if a.id == id {
            if let Some(p) = a.phase.as_deref() {
                if phases.last().map(String::as_str) != Some(p) {
                    phases.push(p.to_string());
                    if let Some(ref cb) = on_phase {
                        cb(p, &flex);
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
    let motion_q = motion_log.lock().unwrap().clone();
    SimRun { status, phases, motion_q, fault_count: fault_count.load(Ordering::Relaxed) }
}

// ============================================================================
// 3. Dispatch parity in sim: strategy "legacy" vs "adaptive" forced -top
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dispatch_parity_in_sim_forced_top_down_matches_legacy() {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.camera_mount = CameraMount { offset_m: Some([-0.06, 0.0, 0.0]), ..CameraMount::default() };
    let setup = GraspSetup {
        pre_open: PreOpen::On,
        pre_open_margin_m: 0.015,
        recentre_height_m: 0.0,
        height_probe_m: 0.0,
        ..GraspSetup::default()
    };
    let q0 = solve_look_down(&cfg, [0.30, 0.0, 0.24]);
    let poly = bar([0.30, 0.0], 0.06, 0.036, 0.0);
    let cam_cfg = LiveCamCfg {
        object_xy: [0.30, 0.0],
        half_xy: [0.03, 0.018],
        height_m: 0.035,
        width_px: Some(120.0),
        angle: None,
        contour_world: Some(poly.clone()),
        box_override_norm: None,
        box_shift_xy: [0.0, 0.0],
    };

    let leg = run_sim_grasp(
        cfg.clone(),
        setup.clone(),
        q0,
        cam_cfg.clone(),
        |body, cam, start| {
            let (_, contour) = project_poly_box_and_contour(cam, start, 0.0, &poly, 0.035);
            body["evidence"] = json!({"contour": contour, "height_m": [0.035, 0.035]});
        },
        None,
    )
    .await;

    // Adaptive with payload_kg = 4.0 forces SelectedRoute::TopDown -> planned_top_down -> legacy run.
    let ad = run_sim_grasp(
        cfg,
        setup,
        q0,
        cam_cfg,
        |body, cam, start| {
            let (_, contour) = project_poly_box_and_contour(cam, start, 0.0, &poly, 0.035);
            body["strategy"] = json!("adaptive");
            body["evidence"] = json!({"contour": contour, "height_m": [0.035, 0.035], "payload_kg": 4.0});
        },
        None,
    )
    .await;

    assert_eq!((leg.status.state.as_str(), leg.status.holding), ("done", Some(true)), "{:?}", leg.status);
    assert_eq!((ad.status.state.as_str(), ad.status.holding), ("done", Some(true)), "{:?}", ad.status);
    assert_eq!(leg.fault_count, 0);
    assert_eq!(ad.fault_count, 0);
    fn strip_start(v: &[String]) -> Vec<&str> {
        v.iter().map(String::as_str).filter(|p| *p != "start").collect()
    }
    assert_eq!(strip_start(&leg.phases), strip_start(&ad.phases), "phase sequences must be identical");
    let mut leg_keys: Vec<&String> = leg.status.phase_ms.keys().collect();
    let mut ad_keys: Vec<&String> = ad.status.phase_ms.keys().collect();
    leg_keys.sort();
    ad_keys.sort();
    assert_eq!(leg_keys, ad_keys, "recorded phase_ms keys must be identical");
    let pre_idx = ad.phases.iter().position(|p| p == "preopen").expect("preopen phase");
    let desc_idx = ad.phases.iter().position(|p| p == "descend").expect("descend phase");
    assert!(pre_idx < desc_idx, "preopen must precede descend: {:?}", ad.phases);
    assert_eq!(leg.status.pre_open_m, ad.status.pre_open_m);
    assert_eq!(leg.motion_q.len(), ad.motion_q.len(), "identical motion tick count");
    let max_diff = leg
        .motion_q
        .iter()
        .zip(&ad.motion_q)
        .flat_map(|(a, b)| (0..7).map(move |i| (a[i] - b[i]).abs()))
        .fold(0.0f32, f32::max);
    assert!(max_diff < 1e-6, "joint trajectory diverged by {max_diff} rad (> 1e-6 rad)");
}

// ============================================================================
// 4. Offset-contour interval check at all 3 call sites:
//    (a) planning admission, (b) live legacy gate, (c) adaptive revalidate w/ rotated wrist
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn offset_contour_refused_at_planning_admission_and_live_legacy_gate() {
    // (a) Planning admission: offset L-shape has box centre at y = 0 and cut [-0.005, +0.038]
    // (width 43 mm passes old scalar rule, +38 mm fails interval OPENING).
    let sc = mounted_scene(0.30, 0.0, 0.26).await;
    let l_poly = offset_l_poly([0.30, 0.0]);
    let mut body = request_for(&sc, &l_poly, 0.035);
    body["overrides"] = json!({"align_j6": false});
    let leg = dry(&sc, &legacy_of(&body)).await;
    assert_eq!((leg["reachable"].clone(), leg["code"].clone()), (json!(false), json!("OPENING")), "{leg:?}");
    let ad = dry(&sc, &body).await;
    assert_eq!(ad["legacy"]["code"], json!("OPENING"), "{ad:?}");

    // (b) Live legacy gate: box-only deferred admission passes planning; live fresh offset
    // contour [-0.005, +0.038] (43 mm width) refuses OPENING before descend, while centred
    // contour [-0.0215, +0.0215] (same 43 mm width) completes.
    let mut cfg = ArmConfig::rebot_dm();
    cfg.camera_mount = CameraMount { offset_m: Some([-0.06, 0.0, 0.0]), ..CameraMount::default() };
    let setup = GraspSetup { align_j6: false, height_probe_m: 0.0, recentre_height_m: 0.0, ..GraspSetup::default() };
    let q0 = solve_look_down(&cfg, [0.30, 0.0, 0.24]);
    let bad = run_sim_grasp(
        cfg.clone(),
        setup.clone(),
        q0,
        LiveCamCfg {
            object_xy: [0.30, 0.0],
            half_xy: [0.04, 0.038],
            height_m: 0.035,
            width_px: None,
            angle: None,
            contour_world: Some(l_poly),
            box_override_norm: None,
            box_shift_xy: [0.0, 0.0],
        },
        |_, _, _| {},
        None,
    )
    .await;
    assert_eq!((bad.status.state.as_str(), bad.status.code.as_deref()), ("failed", Some("OPENING")), "{:?}", bad.status);
    assert!(!bad.phases.iter().any(|p| p == "descend" || p == "close"), "{:?}", bad.phases);

    let sym = run_sim_grasp(
        cfg,
        setup,
        q0,
        LiveCamCfg {
            object_xy: [0.30, 0.0],
            half_xy: [0.04, 0.0215],
            height_m: 0.035,
            width_px: None,
            angle: None,
            contour_world: Some(bar([0.30, 0.0], 0.08, 0.043, 0.0)),
            box_override_norm: None,
            box_shift_xy: [0.0, 0.0],
        },
        |_, _, _| {},
        None,
    )
    .await;
    assert_eq!((sym.status.state.as_str(), sym.status.holding), ("done", Some(true)), "{:?}", sym.status);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn offset_contour_refused_at_adaptive_revalidate_with_rotated_wrist() {
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
    let sc = scene_q(cfg.clone(), setup.clone(), q0).await;
    // Curved banana: adaptive vertical chord beats -top AND stays fully inside the
    // look-down FOV at pregrasp (fx=fy=320) so `revalidate` runs `fresh_match_adaptive`.
    let poly_init = banana([0.30, 0.0], 0.12, 30.0, 0.034, 90.0);
    let (b0, c0) = project_poly_box_and_contour(&sc.cam, &sc.start, 0.0, &poly_init, 0.035);
    let req = json!({
        "label": "cup",
        "box_norm": b0,
        "frame_id": 1,
        "strategy": "adaptive",
        "evidence": {"contour": c0, "height_m": [0.035, 0.035]},
    });
    let planned = {
        let inner = sc.sup.inner().read().await;
        let pending = grasp::prepare(&inner, &req, inner.safety.epoch_id()).expect("prepare");
        drop(inner);
        grasp::plan(pending).expect("plan adaptive chord")
    };
    let contact_xy = planned
        .adaptive
        .as_ref()
        .expect("chord should win over -top for curved banana")
        .route
        .contact_centre;

    let swapped = Arc::new(AtomicBool::new(false));
    let flex = Arc::new(FlexCamera {
        sup: sc.sup.clone(),
        cam: sc.cam.clone(),
        cfg: Mutex::new(LiveCamCfg {
            object_xy: [0.30, 0.0],
            half_xy: [0.035, 0.016],
            height_m: 0.035,
            width_px: None,
            angle: None,
            contour_world: Some(poly_init.clone()),
            box_override_norm: None,
            box_shift_xy: [0.0, 0.0],
        }),
        frame: AtomicU64::new(1),
    });
    let stop = Arc::new(AtomicBool::new(false));
    let (ticker, _, _) = spawn_sim_ticker(sc.sup.clone(), stop.clone(), false);

    let adm = sc.sup.capture_admission_stamp();
    let (id, _) = {
        let mut inner = sc.sup.inner().write().await;
        grasp::begin_with_stamp(&mut inner, adm, &sc.sup, flex.clone(), planned).expect("start adaptive")
    };
    let mut phases = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(20);
    let status = loop {
        assert!(Instant::now() < deadline, "adaptive revalidate test timed out");
        let obs = sc.sup.get_observation().await;
        let a = obs.action.clone().or(obs.previous_action.clone()).expect("action");
        if a.id == id {
            if let Some(p) = a.phase.as_deref() {
                if phases.last().map(String::as_str) != Some(p) {
                    phases.push(p.to_string());
                }
                if p == "pre_open" && !swapped.swap(true, Ordering::SeqCst) {
                    let mut inner = sc.sup.inner().write().await;
                    let dq = 2.0f32.to_radians();
                    inner.commanded_positions[5] += dq;
                    inner.measured_positions[5] += dq;
                    inner.target_positions[5] += dq;
                    let pose_now = ARM.fk(&kinematics::q6(&inner.measured_positions));
                    drop(inner);
                    let (keep_box, _) = project_poly_box_and_contour(&sc.cam, &pose_now, 0.0, &poly_init, 0.035);
                    let mut c = flex.cfg.lock().unwrap();
                    c.contour_world = Some(offset_l_poly(contact_xy));
                    c.box_override_norm = Some(keep_box);
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
    assert_eq!(status.phase.as_deref(), Some("revalidate"), "{status:?}");
    assert_eq!((status.state.as_str(), status.code.as_deref()), ("failed", Some("OPENING")), "{status:?}");
    assert!(!phases.iter().any(|p| p == "insert" || p == "close"), "must refuse before insert: {phases:?}");
}

// ============================================================================
// 5. Rotated / widened object during camera-down transition -> refused on both
//    strategy "legacy" and strategy "adaptive" forced -top
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn rotated_and_widened_object_during_transition_refused_both_strategies() {
    // Camera-down mount where the entire 3 cm tall object sits cleanly INSIDE the post-transition
    // FOV (not touching the image border), so FreshView decides alignment and the live jaw gate.
    let mut cfg = ArmConfig::rebot_dm();
    cfg.camera_mount = CameraMount {
        rot_deg: 0.0,
        tilt_deg: -16.0,
        offset_m: Some([-0.04, 0.0, 0.04]),
        ..CameraMount::default()
    };
    let setup = GraspSetup {
        camera_tilt_deg: -16.0,
        acquire_camera_down: AcquireCameraDown::On,
        approach_height_m: 0.12,
        height_probe_m: 0.0,
        align_j6: false,
        ..GraspSetup::default()
    };
    let q0 = solve_camera_down(&cfg, &setup, [0.30, 0.0, 0.24]);
    let narrow_poly = bar([0.30, 0.0], 0.072, 0.028, 0.0);
    // Rotated 90 deg during transition: 72 mm long side now spans the y closing axis!
    let rotated_poly = bar([0.30, 0.0], 0.072, 0.028, 90.0);

    for (label, forced_adaptive) in [("legacy", false), ("adaptive-top", true)] {
        let rot = rotated_poly.clone();
        let flipped = Arc::new(AtomicBool::new(false));
        let cb: Arc<dyn Fn(&str, &FlexCamera) + Send + Sync> = Arc::new(move |phase, flex| {
            if phase == "transition" && !flipped.swap(true, Ordering::SeqCst) {
                flex.cfg.lock().unwrap().contour_world = Some(rot.clone());
            }
        });
        let run = run_sim_grasp(
            cfg.clone(),
            setup.clone(),
            q0,
            LiveCamCfg {
                object_xy: [0.30, 0.0],
                half_xy: [0.036, 0.014],
                height_m: 0.030,
                width_px: None,
                angle: None,
                contour_world: Some(narrow_poly.clone()),
                box_override_norm: None,
                box_shift_xy: [0.0, 0.0],
            },
            |body, cam, start| {
                let (_, contour) = project_poly_box_and_contour(cam, start, 0.0, &narrow_poly, 0.030);
                if forced_adaptive {
                    body["strategy"] = json!("adaptive");
                    body["evidence"] = json!({"contour": contour, "height_m": [0.030, 0.030], "payload_kg": 4.0});
                } else {
                    body["evidence"] = json!({"contour": contour, "height_m": [0.030, 0.030]});
                }
            },
            Some(cb),
        )
        .await;
        assert!(run.status.final_offset_px.is_some(), "{label}: post-transition recheck must observe target");
        assert_eq!(
            (run.status.state.as_str(), run.status.code.as_deref()),
            ("failed", Some("OPENING")),
            "{label}: rotated object during transition must refuse OPENING: {:?}",
            run.status
        );
        assert!(!run.phases.iter().any(|p| p == "descend" || p == "close"), "{label}: {:?}", run.phases);
    }
}

// ============================================================================
// 6. Evidence rungs at live gate: contour, scalar, admission_shifted, box,
//    and no evidence covering closing axis -> NO_WIDTH_EVIDENCE
// ============================================================================

async fn run_no_width_evidence_case(cfg: ArmConfig, mut setup: GraspSetup, q0: [f64; 6]) -> ActionStatus {
    // Thin along x (16 mm, uncut at v = 0.72), wide along y (76 mm -> deferred box-only):
    // grip_pixel_norm at [0.5, 0.72] leaves the fresh box off the TCP closing line so
    // cut_interval_near returns None -> NO_WIDTH_EVIDENCE before descend.
    setup.grip_pixel_norm = [0.5, 0.72];
    let sc = scene_q(cfg, setup, q0).await;
    let flex = Arc::new(FlexCamera {
        sup: sc.sup.clone(),
        cam: sc.cam.clone(),
        cfg: Mutex::new(LiveCamCfg {
            object_xy: [0.30, 0.0],
            half_xy: [0.008, 0.038],
            height_m: 0.030,
            width_px: None,
            angle: None,
            contour_world: None,
            box_override_norm: None,
            box_shift_xy: [0.0, 0.0],
        }),
        frame: AtomicU64::new(0),
    });
    let stop = Arc::new(AtomicBool::new(false));
    let (ticker, _, _) = spawn_sim_ticker(sc.sup.clone(), stop.clone(), false);
    let body = json!({"label": "cup", "box_norm": flex.det_at(&sc.start).box_norm, "frame_id": 1});
    let (id, _) = grasp::start(&sc.sup, sc.sup.capture_admission_stamp(), flex, &body)
        .await
        .expect("start accepted");
    let deadline = Instant::now() + Duration::from_secs(20);
    let status = loop {
        assert!(Instant::now() < deadline, "no-evidence test timed out");
        let obs = sc.sup.get_observation().await;
        let a = obs.action.clone().or(obs.previous_action.clone()).expect("action");
        if a.id == id && !a.is_running() {
            break a;
        }
        tokio::time::sleep(Duration::from_millis(2)).await;
    };
    stop.store(true, Ordering::Relaxed);
    let _ = ticker.await;
    status
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn live_gate_evidence_rungs_all_rungs_and_no_width_evidence() {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.camera_mount = CameraMount { offset_m: Some([-0.06, 0.0, 0.0]), ..CameraMount::default() };
    let q0 = solve_look_down(&cfg, [0.30, 0.0, 0.24]);
    let base_setup = GraspSetup {
        align_j6: false,
        height_probe_m: 0.0,
        recentre_height_m: 0.0,
        max_grasp_width_m: 0.10,
        ..GraspSetup::default()
    };
    let cam_of = |half_xy, width_px, contour_world| LiveCamCfg {
        object_xy: [0.30, 0.0],
        half_xy,
        height_m: 0.030,
        width_px,
        angle: None,
        contour_world,
        box_override_norm: None,
        box_shift_xy: [0.0, 0.0],
    };

    // Rung 1 (contour): fresh wide contour overrides narrow scalar width -> OPENING (contour).
    let r1 = run_sim_grasp(
        cfg.clone(),
        base_setup.clone(),
        q0,
        cam_of([0.025, 0.015], Some(80.0), Some(bar([0.30, 0.0], 0.05, 0.072, 0.0))),
        |body, cam, start| {
            let (b, _) = project_poly_box_and_contour(cam, start, 0.0, &bar([0.30, 0.0], 0.05, 0.03, 0.0), 0.030);
            body["box_norm"] = json!(b);
        },
        None,
    )
    .await;
    assert_eq!((r1.status.state.as_str(), r1.status.code.as_deref()), ("failed", Some("OPENING")), "{:?}", r1.status);
    assert!(r1.status.message.as_deref().unwrap_or("").contains("(contour)"), "{:?}", r1.status.message);

    // Rung 2 (scalar): no contour, wide scalar width_px (~68 mm < 85 mm cap) -> OPENING (scalar).
    let r2 = run_sim_grasp(cfg.clone(), base_setup.clone(), q0, cam_of([0.025, 0.015], Some(210.0), None), |_, _, _| {}, None).await;
    assert_eq!((r2.status.state.as_str(), r2.status.code.as_deref()), ("failed", Some("OPENING")), "{:?}", r2.status);
    assert!(r2.status.message.as_deref().unwrap_or("").contains("(scalar)"), "{:?}", r2.status.message);

    // Rung 2b (admission_shifted): non-deferred box-only with height_source == "setup".
    let r2b = run_sim_grasp(cfg.clone(), base_setup.clone(), q0, cam_of([0.025, 0.015], None, None), |_, _, _| {}, None).await;
    assert_eq!((r2b.status.state.as_str(), r2b.status.holding), ("done", Some(true)), "{:?}", r2b.status);

    // Rung 3 (box): deferred box-only (~71 mm < 85 mm cap), no contour, no scalar -> OPENING (box).
    let r3 = run_sim_grasp(cfg.clone(), base_setup.clone(), q0, cam_of([0.030, 0.030], None, None), |_, _, _| {}, None).await;
    assert_eq!((r3.status.state.as_str(), r3.status.code.as_deref()), ("failed", Some("OPENING")), "{:?}", r3.status);
    assert!(r3.status.message.as_deref().unwrap_or("").contains("(box)"), "{:?}", r3.status.message);

    // None -> NO_WIDTH_EVIDENCE.
    let r_none = run_no_width_evidence_case(cfg, base_setup, q0).await;
    assert_eq!((r_none.state.as_str(), r_none.code.as_deref()), ("failed", Some("NO_WIDTH_EVIDENCE")), "{r_none:?}");
}
