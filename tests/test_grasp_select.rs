//! Adaptive grasp pose selection (`strategy: "adaptive"` dry run) against the B601-DM model.
//!
//! The controller's camera model is the truth here (no calibration error): a banana-like
//! footprint is projected through `CamModel::assumed` at the start pose to make the request's
//! box and contour.
use std::sync::Arc;
use armd::grasp::{self, BoxFuture, CamModel, DetectionFrame, DetectionSource, GraspSetup};
use armd::grasp_select::feasibility;
use armd::kinematics::{self, Pose, Vec3};
use armd::model::{CameraMount, KeepoutBox};
use armd::{ArmConfig, ArmSupervisor};
use serde_json::{json, Map, Value};

/// A source that is never read by a dry run without a task.
struct NoSource;

impl DetectionSource for NoSource {
    fn inference_enabled(&self) -> BoxFuture<'_, Result<bool, String>> { Box::pin(async { Ok(false) }) }
    fn set_inference(&self, _enabled: bool) -> BoxFuture<'_, Result<(), String>> { Box::pin(async { Ok(()) }) }
    fn next_frame(&self, _after: Option<u64>, _wait_ms: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> { Box::pin(async { Ok(None) }) }
}

const INSPECT: [f64; 6] = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0];

struct Scene {
    sup: Arc<ArmSupervisor>,
    start: Pose,
    cam: CamModel,
    setup: GraspSetup,
}

/// Banana footprint: an arc of centreline radius `rc` about `c`, ±`half_deg`, `w` wide, the
/// arc's middle at `mid` (base xy). `turn_deg` turns the banana about its middle.
fn banana(mid: [f64; 2], rc: f64, half_deg: f64, w: f64, turn_deg: f64) -> Vec<[f64; 2]> {
    let n = 12;
    let (h, t) = (half_deg.to_radians(), turn_deg.to_radians());
    // arc about the origin with its middle at (rc, 0), then turned and moved
    let mut pts = Vec::new();
    for k in 0..=n { let a = -h + 2.0 * h * k as f64 / n as f64; pts.push([(rc + w / 2.0) * a.cos(), (rc + w / 2.0) * a.sin()]); }
    for k in (0..=n).rev() { let a = -h + 2.0 * h * k as f64 / n as f64; pts.push([(rc - w / 2.0) * a.cos(), (rc - w / 2.0) * a.sin()]); }
    pts.iter().map(|p| {
        let x = p[0] - rc;
        let (xr, yr) = (x * t.cos() - p[1] * t.sin(), x * t.sin() + p[1] * t.cos());
        [mid[0] + xr, mid[1] + yr]
    }).collect()
}

/// Request for `poly` (base xy) of height `h`: box over the table and top projections,
/// contour at mid height, normalised.
fn request(sc: &Scene, poly: &[[f64; 2]], h: f64) -> Value {
    let table = sc.setup.table_z_m;
    let norm = |p: [f64; 2]| sc.cam.to_norm(p);
    let mut b = [1.0f64, 1.0, 0.0, 0.0];
    for z in [table, table + h] {
        for v in poly {
            let px = norm(sc.cam.project(&sc.start, &[v[0], v[1], z]).expect("in front of the camera"));
            b = [b[0].min(px[0]), b[1].min(px[1]), b[2].max(px[0]), b[3].max(px[1])];
        }
    }
    for v in &b { assert!((0.0..=1.0).contains(v), "object in view: box {b:?}"); }
    let contour: Vec<[f64; 2]> = poly.iter().map(|v| norm(sc.cam.project(&sc.start, &[v[0], v[1], table + h / 2.0]).unwrap())).collect();
    json!({"label": "banana", "box_norm": b, "frame_id": 7, "dry_run": true, "strategy": "adaptive",
           "evidence": {"contour": contour, "height_m": [h * 0.9, h]}})
}

async fn dry(sc: &Scene, body: &Value) -> Map<String, Value> {
    grasp::dry_run_task(&sc.sup, &NoSource, body).await.fields()
}

fn sel(v: &Map<String, Value>) -> &Value { &v["selected_candidate"] }

fn summary(v: &Map<String, Value>) -> String {
    format!("reachable {} code {} outcome {} legacy {} sel {} {} half {} rejections {} search {}", v["reachable"], v["code"], v["outcome"], v["legacy"]["code"],
        sel(v)["approach_class"], sel(v)["tilt_deg"], sel(v)["jaw_half_turn"], v["rejection_codes"], v["search"])
}

/// The office B601-DM mount (v8 holder, 38 deg, derived values) with the derived stream intrinsics.
fn mounted() -> (ArmConfig, GraspSetup) {
    let mut c = ArmConfig::rebot_dm();
    c.camera_mount = CameraMount { rot_deg: 90.0, tilt_deg: -38.0, offset_m: Some([-0.0731, 0.0005, 0.0660]), ..CameraMount::default() };
    let s = GraspSetup { fx_px: 824.0, fy_px: 824.0, camera_tilt_deg: -38.0, ..GraspSetup::default() };
    (c, s)
}

/// End_link pose with the optical axis down and the lens at `lens` (heading toward +azimuth of `lens`).
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
        if sol.ok { return q; }
    }
    panic!("camera-down start at {lens:?} unreachable (got {:?})", arm.fk(&q).p);
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

/// Mounted camera, observing the point (r, 0) from `cam_z` above the table.
async fn mounted_scene(r: f64, az_deg: f64, cam_z: f64) -> Scene {
    let (c, s) = mounted();
    let a = az_deg.to_radians();
    let q = solve_camera_down(&c, &s, [r * a.cos(), r * a.sin(), cam_z]);
    scene_q(c, s, q).await
}

/// The selection itself (all rejections), from the same request.
async fn selection(sc: &Scene, body: &Value) -> (armd::grasp_select::Snapshot, armd::grasp_select::Selection) {
    let inner = sc.sup.inner().read().await;
    let mut legacy = body.clone();
    legacy.as_object_mut().unwrap().remove("strategy");
    let pending = grasp::prepare(&inner, &legacy, inner.safety.epoch_id()).expect("request accepted");
    let snap = armd::grasp_select::capture(&inner, &pending, body).expect("snapshot");
    drop(inner);
    let sel = armd::grasp_select::select(&snap);
    (snap, sel)
}

fn phases(v: &Value) -> Vec<&str> { v.as_array().unwrap().iter().map(|p| p.as_str().unwrap()).collect() }

const ALL_PHASES: [&str; 8] = ["acquisition", "transition", "pre_open", "insertion", "closure", "separation", "lift", "withdrawal"];

/// The reported banana at 0.40 m: the legacy dry run (vertical descent from approach height)
/// is IK_UNREACHABLE, the adaptive search checks a radial oblique completely; at 0.42 m only
/// obliques remain and one is selected.
#[tokio::test]
async fn banana_at_0_40_m_legacy_unreachable_adaptive_finds_an_oblique() {
    let sc = mounted_scene(0.40, 0.0, 0.26).await;
    let poly = banana([0.40, 0.0], 0.12, 30.0, 0.034, 90.0);
    let body = request(&sc, &poly, 0.035);
    let v = dry(&sc, &body).await;
    println!("0.40: {}", summary(&v));
    assert_eq!(v["legacy"]["reachable"], false, "{}", v["legacy"]);
    assert_eq!(v["legacy"]["code"], "IK_UNREACHABLE");
    assert_eq!(v["reachable"], true, "{}", v["message"]);
    assert_eq!(v["outcome"], "selected");
    assert_eq!(phases(&v["checked_phases"]), ALL_PHASES);
    assert_eq!(v["evidence"]["footprint_source"], "contour");
    let (_snap, sel) = selection(&sc, &body).await;
    let oblique = sel.ranked.iter().find(|c| c.hy.class.is_radial_oblique()).expect("a fully checked radial oblique");
    assert!(oblique.hy.tilt_deg >= 10.0 && oblique.checked.phases == ALL_PHASES, "{:?}", oblique.checked.phases);
    assert!(oblique.checked.margins.joint_limit > 0.0 && oblique.checked.margins.sigma_min > 0.0);

    let sc = mounted_scene(0.42, 0.0, 0.26).await;
    let poly = banana([0.42, 0.0], 0.12, 30.0, 0.034, 90.0);
    let v = dry(&sc, &request(&sc, &poly, 0.035)).await;
    println!("0.42: {}", summary(&v));
    assert_eq!(v["legacy"]["code"], "IK_UNREACHABLE");
    assert_eq!(v["reachable"], true, "{}", v["message"]);
    let c = sel_of(&v);
    assert_eq!(c["approach_class"], "radial_outward", "{c}");
    assert!(c["tilt_deg"].as_f64().unwrap() >= 10.0);
    // the plan phase B executes: insertion along a, contact below the pre-grasp
    let plan = &c["plan"];
    let a: Vec<f64> = plan["insertion_dir"].as_array().unwrap().iter().map(|x| x.as_f64().unwrap()).collect();
    let pre: Vec<f64> = plan["pregrasp"]["p"].as_array().unwrap().iter().map(|x| x.as_f64().unwrap()).collect();
    let g: Vec<f64> = plan["grasp"]["p"].as_array().unwrap().iter().map(|x| x.as_f64().unwrap()).collect();
    let d = plan["insertion_m"].as_f64().unwrap();
    for i in 0..3 { assert!((pre[i] + d * a[i] - g[i]).abs() < 1e-9); }
    assert!(a[0] > 0.1 && a[2] < -0.5, "outward and down: {a:?}");
    assert_eq!(plan["joints"]["grasp"].as_array().unwrap().len(), 6);
    assert!(plan["lift"]["p"][2].as_f64().unwrap() > g[2]);
}

fn sel_of(v: &Map<String, Value>) -> &Value { let c = sel(v); assert!(c.is_object(), "no selected candidate: {}", v["message"]); c }

#[tokio::test]
async fn easy_vertical_grasp_stays_vertical() {
    let sc = mounted_scene(0.30, 0.0, 0.26).await;
    let poly = banana([0.30, 0.0], 0.12, 30.0, 0.034, 90.0);
    let v = dry(&sc, &request(&sc, &poly, 0.035)).await;
    println!("{}", summary(&v));
    assert_eq!(v["legacy"]["reachable"], true);
    assert_eq!(v["reachable"], true);
    let c = sel_of(&v);
    assert_eq!(c["approach_class"], "vertical", "{c}");
    assert_eq!(c["tilt_deg"], 0.0);
    // the chord is across the banana (long axis radial = x): the jaws close along y
    assert!(c["chord"]["dir"][1].as_f64().unwrap().abs() > 0.95, "{}", c["chord"]);
    assert!((c["chord"]["width_m"].as_f64().unwrap() - 0.034).abs() < 0.004);
    assert!(v["alternatives"].as_array().unwrap().len() <= armd::grasp_select::MAX_ALTERNATIVES);
}

#[tokio::test]
async fn wholly_unreachable_object_fails_at_acquisition() {
    // the table 0.30 m below the base: nothing reaches the object
    let (c, mut s) = mounted();
    s.table_z_m = -0.30;
    let q = solve_camera_down(&c, &s, [0.30, 0.0, 0.26]);
    let sc = scene_q(c, s, q).await;
    let poly = banana([0.30, 0.0], 0.12, 30.0, 0.034, 90.0);
    let v = dry(&sc, &request(&sc, &poly, 0.035)).await;
    println!("{}", summary(&v));
    assert_eq!(v["reachable"], false);
    assert_eq!(v["legacy"]["reachable"], false);
    assert_eq!(v["outcome"], "infeasible");
    assert!(v["message"].as_str().unwrap().starts_with("acquisition:"), "{}", v["message"]);
    assert_eq!(v["rejections"]["acquisition"], 1);
    assert!(sel(&v).is_null());
}

#[tokio::test]
async fn reachable_contact_with_an_impossible_lift() {
    let sc = mounted_scene(0.30, 0.0, 0.26).await;
    let poly = banana([0.30, 0.0], 0.12, 30.0, 0.034, 90.0);
    let mut body = request(&sc, &poly, 0.035);
    body["evidence"]["payload_kg"] = json!(4.0);
    let v = dry(&sc, &body).await;
    println!("{}", summary(&v));
    assert_eq!(v["legacy"]["reachable"], true, "the legacy check does not look at the lift");
    assert_eq!(v["reachable"], false);
    assert_eq!(v["outcome"], "infeasible");
    assert_eq!(v["code"], "LIFT_TORQUE", "{}", v["message"]);
    assert!(v["message"].as_str().unwrap().contains("deepest failure in lift"), "{}", v["message"]);
    let lift = v["alternatives"].as_array().unwrap().iter().find(|a| a["phase"] == "lift").expect("a lift rejection");
    // numerical reason: the payload the derated torques allow, below the 4 kg bound
    assert!(lift["value"].as_f64().unwrap() < 4.0 && lift["bound"] == 4.0, "{lift}");
    // the same request with a light payload passes
    body["evidence"]["payload_kg"] = json!(0.1);
    assert_eq!(dry(&sc, &body).await["reachable"], true);
}

#[tokio::test]
async fn unknown_payload_bound_is_insufficient_evidence_when_the_lift_depends_on_it() {
    let (mut c, s) = mounted();
    let q = solve_camera_down(&c, &s, [0.30, 0.0, 0.26]);
    // derated limits just above the arm's own holding torque along the loaded phases
    let sc0 = scene_q(c.clone(), s.clone(), q).await;
    let poly = banana([0.30, 0.0], 0.12, 30.0, 0.034, 90.0);
    let mut body = request(&sc0, &poly, 0.035);
    let (_snap, sel0) = selection(&sc0, &body).await;
    let cap = sel0.ranked.iter().filter_map(|k| k.checked.margins.payload_capacity_kg).fold(0.0, f64::max);
    assert!(cap > feasibility::ASSUMED_PAYLOAD_KG, "{cap}");
    // scale the torque limits so that no candidate carries the assumed bound
    let base = c.gravity_comp.as_ref().unwrap().torque_limits_nm.clone();
    let (mut lo, mut hi) = (0.0f32, 1.0f32);
    let mut found = None;
    for _ in 0..30 {
        let mid = (lo + hi) / 2.0;
        c.gravity_comp.as_mut().unwrap().torque_limits_nm = base.iter().map(|t| t * mid).collect();
        let sc = scene_q(c.clone(), s.clone(), q).await;
        let (_snap, sel) = selection(&sc, &body).await;
        let best = sel.ranked.iter().filter_map(|k| k.checked.margins.payload_capacity_kg).fold(f64::NEG_INFINITY, f64::max);
        if sel.ranked.is_empty() || best < 0.03 { lo = mid; } else if best > 0.12 { hi = mid; } else { found = Some((mid, sel)); break; }
    }
    let (scale, sel) = found.expect("a limit scale with a small payload capacity");
    println!("torque limit scale {scale}: {:?} {:?} {:?}", sel.outcome, sel.code, sel.message);
    assert_eq!(sel.outcome, armd::grasp_select::Outcome::InsufficientEvidence);
    assert_eq!(sel.code.as_deref(), Some("INSUFFICIENT_EVIDENCE"));
    assert!(!sel.reachable() && sel.plan.is_none());
    // a stated bound below the capacity decides it
    c.gravity_comp.as_mut().unwrap().torque_limits_nm = base.iter().map(|t| t * scale).collect();
    let sc = scene_q(c.clone(), s.clone(), q).await;
    body["evidence"]["payload_kg"] = json!(0.0);
    let (_snap, sel) = selection(&sc, &body).await;
    assert!(sel.reachable(), "{:?} {:?}", sel.outcome, sel.message);
}

/// An obstacle (keep-out box, e.g. a cable) where the wrist camera passes in the middle of the
/// selected candidate's transition: that candidate fails in the transition at the camera, with
/// clear start and end, and another one is selected.
#[tokio::test]
async fn wrist_camera_collision_mid_rotation_rejects_the_candidate() {
    let (c, st) = mounted();
    let q = solve_camera_down(&c, &st, [0.30, 0.0, 0.30]);
    let sc = scene_q(c.clone(), st.clone(), q).await;
    let poly = banana([0.30, 0.0], 0.12, 20.0, 0.034, 90.0);
    let body = request(&sc, &poly, 0.035);
    let (snap, sel) = selection(&sc, &body).await;
    let best = sel.ranked.first().expect("selected").clone();
    let mut w = feasibility::Work::default();
    let acq = feasibility::acquisition(&snap, &mut w).unwrap();
    let path = feasibility::phase(&snap, "transition", acq.q_end, snap.open_rad, best.hy.pregrasp_pose(), 0.05, &mut w).unwrap();
    let off = c.camera_mount.offset_m.unwrap().map(f64::from);
    let lens = |q: &[f32; 7]| c.arm_model.fk(&std::array::from_fn::<f64, 6, _>(|i| f64::from(q[i]))).transform_point(&off);
    let n = path.q.len();
    let mid = lens(&path.q[n / 2]);
    let h = 0.008;
    let inside = |p: Vec3| (0..3).all(|i| (p[i] - mid[i]).abs() <= h);
    assert!(!inside(lens(&path.q[0])) && !inside(lens(&path.q[n - 1])), "the box is clear of the lens at both ends");
    let mut c2 = c.clone();
    c2.workspace.as_mut().unwrap().keepout_boxes = vec![KeepoutBox { name: "cable".into(),
        min_m: mid.map(|v| (v - h) as f32), max_m: mid.map(|v| (v + h) as f32) }];
    let sc2 = scene_q(c2, st, q).await;
    let (_snap2, sel2) = selection(&sc2, &body).await;
    let rej = sel2.rejected.iter().find(|r| r.hy.index == best.hy.index).expect("the former winner is rejected");
    println!("{:?} {:?} {:?} {}", rej.rejection.phase, rej.rejection.geometry, rej.rejection.s, rej.rejection.message);
    assert_eq!(rej.rejection.phase, "transition");
    assert!(rej.rejection.geometry.as_deref().is_some_and(|g| g.contains("camera")), "{:?}", rej.rejection.geometry);
    let s_fail = rej.rejection.s.unwrap();
    assert!(s_fail > 0.1 && s_fail < 0.9, "mid-rotation: s={s_fail}");
    assert!(rej.rejection.value.unwrap() > 0.0, "penetration depth reported");
    assert!(sel2.reachable());
    assert_ne!(sel2.selected().unwrap().hy.index, best.hy.index);
}

/// Same snapshot, same quotas: the same candidates, scores, rejections and work.
#[tokio::test]
async fn selection_is_deterministic() {
    let sc = mounted_scene(0.40, 0.0, 0.26).await;
    let poly = banana([0.40, 0.0], 0.12, 30.0, 0.034, 90.0);
    let body = request(&sc, &poly, 0.035);
    let strip = |mut v: Map<String, Value>| { v["search"]["elapsed_ms"] = Value::Null; v };
    let first = strip(dry(&sc, &body).await);
    for _ in 0..3 {
        assert_eq!(strip(dry(&sc, &body).await), first);
    }
    let (_s, a) = selection(&sc, &body).await;
    let (_s, b) = selection(&sc, &body).await;
    assert_eq!(a.stats, b.stats);
    assert_eq!(a.snapshot_id, b.snapshot_id);
    let ids = |s: &armd::grasp_select::Selection| s.ranked.iter().map(|c| (c.id.clone(), c.scores.total.to_bits())).collect::<Vec<_>>();
    assert_eq!(ids(&a), ids(&b));
}

/// Host timing and work counts (run with `--release --nocapture` for the numbers in the report).
#[tokio::test]
async fn timing_and_work_counts() {
    let mut rows = Vec::new();
    for r in [0.30, 0.36, 0.40, 0.42] {
        let sc = mounted_scene(r, 0.0, 0.26).await;
        let poly = banana([r, 0.0], 0.12, 30.0, 0.034, 90.0);
        let body = request(&sc, &poly, 0.035);
        let mut ms = Vec::new();
        let mut last = None;
        for _ in 0..5 {
            let (_snap, sel) = selection(&sc, &body).await;
            ms.push(sel.elapsed_ms);
            last = Some(sel);
        }
        ms.sort_by(f64::total_cmp);
        let s = last.unwrap();
        println!("timing r {r}: {:?} median {:.1} ms max {:.1} ms; ik_iterations {} endpoint_solves {} path_checks {} path_samples {} lattice_tests {} generated {} shortlisted {} fully_checked {}",
            s.outcome, ms[2], ms[4], s.stats.work.ik_iterations, s.stats.work.endpoint_solves, s.stats.work.path_checks, s.stats.work.path_samples,
            s.stats.lattice_tests, s.stats.generated, s.stats.shortlisted, s.stats.fully_checked);
        assert!(s.elapsed_ms < s.budget_ms as f64);
        rows.push(s.stats.work.ik_iterations);
    }
    assert!(rows.iter().all(|n| *n > 0));
}

#[tokio::test]
async fn budget_exhaustion_returns_no_winner() {
    let sc = mounted_scene(0.30, 0.0, 0.26).await;
    let poly = banana([0.30, 0.0], 0.12, 30.0, 0.034, 90.0);
    let mut body = request(&sc, &poly, 0.035);
    body["budget_ms"] = json!(1);
    let v = dry(&sc, &body).await;
    assert_eq!(v["outcome"], "budget_exhausted", "{}", summary(&v));
    assert_eq!(v["reachable"], false);
    assert_eq!(v["code"], "BUDGET_EXHAUSTED");
    assert!(sel(&v).is_null());
}

#[tokio::test]
async fn legacy_semantics_and_strategy_gating() {
    let sc = mounted_scene(0.30, 0.0, 0.26).await;
    let poly = banana([0.30, 0.0], 0.12, 30.0, 0.034, 90.0);
    let adaptive = request(&sc, &poly, 0.035);
    let mut legacy = adaptive.clone();
    legacy.as_object_mut().unwrap().remove("strategy");
    let v = dry(&sc, &legacy).await;
    assert_eq!(v["reachable"], true);
    assert!(!v.contains_key("strategy") && !v.contains_key("selected_candidate"), "{v:?}");
    // explicit legacy is the same
    let mut explicit = legacy.clone();
    explicit["strategy"] = json!("legacy");
    assert_eq!(dry(&sc, &explicit).await, v);
    // a bad value is refused
    let mut bad = legacy.clone();
    bad["strategy"] = json!("best");
    let v = dry(&sc, &bad).await;
    assert_eq!((v["reachable"].clone(), v["code"].clone()), (json!(false), json!("INVALID_COMMAND")));
    // an executed grasp with the adaptive strategy is refused before anything starts
    let inner = sc.sup.inner().read().await;
    let mut exec = adaptive.clone();
    exec.as_object_mut().unwrap().remove("dry_run");
    let err = grasp::prepare(&inner, &exec, inner.safety.epoch_id()).err().expect("refused");
    assert!(format!("{err:?}").contains("dry_run only"), "{err:?}");
    drop(inner);
    // capability
    assert!(armd::model::Capabilities::from_config(&ArmConfig::rebot_dm()).grasp_select);
    let so101 = armd::tool::baseline_arm_config_for_robot("so101").unwrap();
    assert!(!armd::model::Capabilities::from_config(&so101).grasp_select);
}

#[tokio::test]
async fn box_only_evidence_falls_back_with_wider_uncertainty() {
    let sc = mounted_scene(0.30, 0.0, 0.26).await;
    let poly = banana([0.30, 0.0], 0.12, 30.0, 0.034, 90.0);
    let mut body = request(&sc, &poly, 0.035);
    body["evidence"].as_object_mut().unwrap().remove("contour");
    let v = dry(&sc, &body).await;
    println!("{}", summary(&v));
    assert_eq!(v["evidence"]["footprint_source"], "box");
    assert_eq!(v["evidence"]["axis_ambiguous"], true);
    // a malformed contour is reported and the box is used
    body["evidence"]["contour"] = json!([[0.1, 0.1], [0.2, 0.2]]);
    let v = dry(&sc, &body).await;
    assert_eq!(v["evidence"]["footprint_source"], "box");
    assert!(v["evidence"]["contour_error"].is_string());
}
