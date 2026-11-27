//! Shared grasp admission (legacy and adaptive): jaw predicate, best_refusal, error margin.
//!
//! The controller's camera model is the truth here (no calibration error): a banana-like
//! footprint is projected through `CamModel::assumed` at the start pose to make the request's
//! box and contour.
mod common;

use std::sync::Arc;
use armd::grasp::{self, BoxFuture, CamModel, DetectionFrame, DetectionSource, GraspSetup};
use armd::kinematics::{self, Pose, Vec3};
use armd::model::CameraMount;
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

/// A straight bar `len` x `w` centred at `mid`, long axis at `turn_deg` from +x.
fn bar(mid: [f64; 2], len: f64, w: f64, turn_deg: f64) -> Vec<[f64; 2]> {
    let (s, c) = turn_deg.to_radians().sin_cos();
    [[-len / 2.0, -w / 2.0], [len / 2.0, -w / 2.0], [len / 2.0, w / 2.0], [-len / 2.0, w / 2.0]]
        .iter().map(|p| [mid[0] + p[0] * c - p[1] * s, mid[1] + p[0] * s + p[1] * c]).collect()
}

fn legacy_of(body: &Value) -> Value { let mut b = body.clone(); b.as_object_mut().unwrap().remove("strategy"); b }

/// Opening, effective max width and hard error of a target at (0.30, 0) seen from the mounted
/// camera (the hard error does not depend on the object's width).
async fn jaw_numbers(sc: &Scene) -> (f64, f64, f64) {
    let v = dry(sc, &exact(request(sc, &bar([0.30, 0.0], 0.08, 0.03, 0.0), 0.035), 0.035)).await;
    let ev = &v["evidence"];
    (ev["jaw_open_width_m"].as_f64().unwrap(), ev["max_grasp_width_m"].as_f64().unwrap(), ev["uncertainty"]["hard_error_m"].as_f64().unwrap())
}

/// The request with the height interval collapsed to `h`: the planner projects the contour at
/// the plane it was drawn at (table + h / 2), so the footprint width is the drawn width.
fn exact(mut body: Value, h: f64) -> Value { body["evidence"]["height_m"] = json!([h, h]); body }

fn f(v: &Value) -> f64 { v.as_f64().unwrap_or_else(|| panic!("not a number: {v}")) }

#[tokio::test]
async fn exact_threshold_passes_both_strategies_and_just_over_refuses_both() {
    let sc = mounted_scene(0.30, 0.0, 0.26).await;
    let (open, max_w, he) = jaw_numbers(&sc).await;
    // width + 2 * hard_error + 6 mm == open width
    let w = open - 2.0 * he - 0.006 - 1e-7;
    assert!(w < max_w, "threshold {w} inside the cap {max_w}");
    let body = exact(request(&sc, &bar([0.30, 0.0], 0.08, w, 0.0), 0.035), 0.035);
    let leg = dry(&sc, &legacy_of(&body)).await;
    assert_eq!(leg["reachable"], true, "{leg:?}");
    assert_eq!(leg["jaw"]["status"], "checked", "{leg:?}");
    assert!((f(&leg["jaw"]["opening_reserve_m"]) - 0.003).abs() < 1e-5, "{}", leg["jaw"]);
    let ad = dry(&sc, &body).await;
    assert_eq!(ad["reachable"], true, "{}", summary(&ad));
    assert_eq!(ad["legacy"]["reachable"], true);

    let body = exact(request(&sc, &bar([0.30, 0.0], 0.08, w + 0.001, 0.0), 0.035), 0.035);
    let leg = dry(&sc, &legacy_of(&body)).await;
    assert_eq!((leg["reachable"].clone(), leg["code"].clone()), (json!(false), json!("OPENING")), "{leg:?}");
    let br = &leg["best_refusal"];
    assert_eq!((br["phase"].as_str(), br["code"].as_str(), f(&br["bound"])), (Some("geometry"), Some("OPENING"), 0.003), "{br}");
    assert!(f(&br["value"]) < 0.003 && f(&br["value"]) > 0.002, "{br}");
    assert!(br["candidate_id"].as_str().unwrap().ends_with("-top"), "{br}");
    let ad = dry(&sc, &body).await;
    assert_eq!((ad["reachable"].clone(), ad["code"].clone()), (json!(false), json!("OPENING")), "{}", summary(&ad));
    assert_eq!(ad["best_refusal"]["code"], "OPENING", "{}", ad["best_refusal"]);
}

#[tokio::test]
async fn wide_banana_is_refused_by_both_strategies_with_the_measured_reserve() {
    // rc7.3 regression: the legacy approach path is reachable and the old legacy dry run
    // passed; the observed reserve is -3 mm per side against the +3 mm bound
    let sc = mounted_scene(0.30, 0.0, 0.26).await;
    let (open, max_w, he) = jaw_numbers(&sc).await;
    let w = open - 2.0 * he + 0.006;
    assert!(w < max_w);
    let body = request(&sc, &banana([0.30, 0.0], 0.12, 30.0, w, 90.0), 0.035);
    let leg = dry(&sc, &legacy_of(&body)).await;
    println!("legacy {leg:?}");
    assert_eq!((leg["reachable"].clone(), leg["code"].clone(), leg["failing_phase"].clone()), (json!(false), json!("OPENING"), json!("geometry")), "{leg:?}");
    let br = &leg["best_refusal"];
    assert!((f(&br["value"]) + 0.003).abs() < 0.0015 && f(&br["bound"]) == 0.003, "{br}");
    assert_eq!(leg["error_margin"]["error_margin_source"], "fallback");
    let ad = dry(&sc, &body).await;
    println!("adaptive {}", summary(&ad));
    assert_eq!((ad["reachable"].clone(), ad["code"].clone()), (json!(false), json!("OPENING")), "{}", summary(&ad));
    assert_eq!(ad["legacy"]["code"], "OPENING");
    assert_eq!(ad["best_refusal"]["bound"], 0.003);
    assert!(f(&ad["best_refusal"]["value"]) < 0.0, "{}", ad["best_refusal"]);
    assert!(ad.get("hint").is_none_or(Value::is_null), "no legacy-pass hint: {ad:?}");
}

#[tokio::test]
async fn over_cap_width_is_too_wide_for_both_strategies() {
    let sc = mounted_scene(0.30, 0.0, 0.26).await;
    let (_, max_w, _) = jaw_numbers(&sc).await;
    let body = request(&sc, &bar([0.30, 0.0], 0.09, max_w + 0.004, 0.0), 0.035);
    let leg = dry(&sc, &legacy_of(&body)).await;
    assert_eq!(leg["code"], "TOO_WIDE", "{leg:?}");
    assert!((f(&leg["best_refusal"]["bound"]) - max_w).abs() < 1e-9 && f(&leg["best_refusal"]["value"]) > max_w, "{}", leg["best_refusal"]);
    let ad = dry(&sc, &body).await;
    assert_eq!((ad["reachable"].clone(), ad["code"].clone()), (json!(false), json!("TOO_WIDE")), "{}", summary(&ad));
}

#[tokio::test]
async fn box_only_legacy_request_defers_the_jaw_check_to_the_measured_width() {
    let sc = mounted_scene(0.30, 0.0, 0.26).await;
    let mut body = legacy_of(&request(&sc, &bar([0.30, 0.0], 0.08, 0.03, 0.0), 0.035));
    body.as_object_mut().unwrap().remove("evidence");
    let leg = dry(&sc, &body).await;
    assert_eq!(leg["reachable"], true, "{leg:?}");
    assert_eq!(leg["jaw"]["status"], "deferred", "{}", leg["jaw"]);
    assert!(f(&leg["jaw"]["hard_error_m"]) > 0.0);
}

#[tokio::test]
async fn live_plan_and_dry_run_report_the_same_refusal() {
    let sc = mounted_scene(0.30, 0.0, 0.26).await;
    let (open, _, he) = jaw_numbers(&sc).await;
    let body = request(&sc, &bar([0.30, 0.0], 0.08, open - 2.0 * he, 0.0), 0.035);
    for strategy in ["legacy", "adaptive"] {
        let mut b = body.clone();
        b["strategy"] = json!(strategy);
        let d = dry(&sc, &b).await;
        b.as_object_mut().unwrap().remove("dry_run");
        let inner = sc.sup.inner().read().await;
        let pending = grasp::prepare(&inner, &b, inner.safety.epoch_id()).expect("prepared");
        drop(inner);
        match grasp::plan(pending).err().expect("live plan refuses") {
            armd::supervisor::ArmError::Detailed { code, details, .. } => {
                assert_eq!(Value::String(code), d["code"], "{strategy}");
                assert_eq!(details["best_refusal"], d["best_refusal"], "{strategy}");
                assert_eq!(details["failing_phase"], d["failing_phase"], "{strategy}");
                assert_eq!(details["error_margin"]["error_margin_source"], "fallback", "{strategy}");
            }
            other => panic!("{strategy}: expected a detailed refusal, got {other:?}"),
        }
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn best_refusal_and_margin_reach_http_and_ws_replies() {
    let sc = mounted_scene(0.30, 0.0, 0.26).await;
    let (open, _, he) = jaw_numbers(&sc).await;
    let body = request(&sc, &bar([0.30, 0.0], 0.08, open - 2.0 * he, 0.0), 0.035);
    let q: Vec<f32> = sc.sup.inner().read().await.commanded_positions.clone();
    let expected = dry(&sc, &body).await;
    let (mut ws, generation, epoch, _sup, addr) = common::torqued_rig(sc.sup.clone(), &q).await;
    let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/action/grasp", body.clone()).await;
    assert_eq!(ack["reachable"], false, "{ack}");
    for k in ["phase", "code", "value", "bound"] { assert_eq!(ack["best_refusal"][k], expected["best_refusal"][k], "{k}: {ack}"); }
    assert_eq!(ack["evidence"]["uncertainty"]["error_margin"]["error_margin_source"], "fallback", "{ack}");
    assert_eq!(ack["legacy"]["best_refusal"]["code"], "OPENING", "{ack}");
    let http: Value = reqwest::Client::new().post(format!("http://{addr}/api/v1/action/grasp")).json(&legacy_of(&body))
        .send().await.unwrap().json().await.unwrap();
    assert_eq!(http["code"], "OPENING", "{http}");
    assert_eq!(http["best_refusal"]["code"], "OPENING", "{http}");
    assert_eq!(http["error_margin"]["error_margin_source"], "fallback", "{http}");
}

#[test]
fn error_margin_source_stays_fallback_and_names_the_reason() {
    use armd::calibration::{CalibrationFile, CalibrationStale};
    use armd::grasp_select::margin::{margin, quality};
    let reason = |c: Option<&CalibrationFile>| margin(&quality(c), 0.01, 0.001, 0.25, 0.5);
    assert_eq!(reason(None).reason, "fallback: no calibration");
    let partial = CalibrationFile { rms: Some(json!({"hand_eye_m": 0.002, "hand_eye_deg": 0.3})), ..Default::default() };
    let m = reason(Some(&partial));
    assert!(m.reason.starts_with("fallback: partial: no kinematic held-out residual"), "{}", m.reason);
    assert!(m.measured_budget_m.is_none() && m.hand_eye_m.is_some());
    let full_rms = json!({"hand_eye_m": 0.002, "hand_eye_deg": 0.3, "kin_heldout_mm": [1.5, 2.5], "plane_spread_m": 0.002});
    let full = CalibrationFile { rms: Some(full_rms), ..Default::default() };
    let m = reason(Some(&full));
    // max(kinematic 2.5 mm, hand-eye 2 mm + 0.25 m tan 0.3 deg) + contour 1 mm + 2 mm * 0.5
    let he = 0.002 + 0.25 * 0.3f64.to_radians().tan();
    assert!((m.measured_budget_m.unwrap() - (0.0025f64.max(he) + 0.001 + 0.001)).abs() < 1e-12, "{m:?}");
    assert!(m.reason.starts_with("measured budget reported only"), "{}", m.reason);
    assert_eq!(m.hard_error_m, 0.01, "the fallback stays active");
    assert_eq!(m.to_json()["error_margin_source"], "fallback");
    let stale = CalibrationFile { stale: Some(CalibrationStale { reason: "zero".into(), at: "2026-11-27T00:00:00Z".into() }), ..full };
    let m = reason(Some(&stale));
    assert!(m.reason.starts_with("fallback: stale") && m.measured_budget_m.is_none(), "{m:?}");
}

#[tokio::test]
async fn nonzero_tcp_offset_envelope_points_stay_in_end_link_frame() {
    // Custom tool whose TCP is +0.030 m ahead of end_link along +x (tool axis).
    // With an end_link envelope point at +0.025 m along +x (5 mm BEHIND the TCP, not 25 mm
    // ahead of it), a vertical grasp (tool +x straight down) has that envelope point 5 mm ABOVE
    // the TCP, so z_min stays at grasp_height_m (0.015 m) rather than being pushed up by 25 mm.
    let (c, s) = mounted();
    let q = solve_camera_down(&c, &s, [0.30, 0.0, 0.26]);
    let sc = scene_q(c, s, q).await;
    let poly = bar([0.30, 0.0], 0.08, 0.030, 0.0);
    let body = exact(request(&sc, &poly, 0.040), 0.040);
    let inner = sc.sup.inner().read().await;
    let pending = grasp::prepare(&inner, &legacy_of(&body), inner.safety.epoch_id()).unwrap();
    let mut snap = armd::grasp_select::capture(&inner, &pending, &body).unwrap();
    drop(inner);
    snap.input.tcp_offset = [0.030, 0.0, 0.0];
    snap.envelope = vec![[0.0, 0.0, 0.0], [0.025, 0.0, 0.0]];
    let t = armd::grasp_select::snapshot::target(&snap).unwrap();
    let (chords, _) = armd::grasp_select::candidates::chords(&t, &snap.tool);
    let mut stats = armd::grasp_select::candidates::CheapStats::default();
    let hy = armd::grasp_select::candidates::make(
        &snap, &t, &chords[0], 0, (armd::grasp_select::candidates::DirClass::Vertical, 0.0, [1.0, 0.0]), false, false, 0, &mut stats,
    ).expect("envelope point behind TCP must not push z_min into PAD_OVERLAP");
    assert!((hy.g[2] - (snap.setup.table_z_m + snap.setup.grasp_height_m)).abs() < 1e-9, "gz={}", hy.g[2]);
    // Now place an end_link point 0.010 m BEYOND the TCP (+0.040 m vs tcp_offset +0.030 m):
    // vertical tool +x points down, so that point sits 10 mm BELOW the TCP and raises z_min by 10 mm
    // above TABLE_CLEARANCE_M + IK_POSITION_ALLOWANCE_M (= 0.0165 m > grasp_height_m 0.015 m).
    snap.envelope = vec![[0.0, 0.0, 0.0], [0.040, 0.0, 0.0]];
    let hy2 = armd::grasp_select::candidates::make(
        &snap, &t, &chords[0], 0, (armd::grasp_select::candidates::DirClass::Vertical, 0.0, [1.0, 0.0]), false, false, 1, &mut stats,
    ).expect("10 mm protrusion raises z_min to 16.5 mm, still within 40 mm object");
    let expected_z = snap.setup.table_z_m + grasp::TABLE_CLEARANCE_M + armd::grasp_select::candidates::IK_POSITION_ALLOWANCE_M + 0.010;
    assert!((hy2.g[2] - expected_z).abs() < 1e-9, "gz={} expected={expected_z}", hy2.g[2]);
    assert!((hy2.table_clearance_m - (grasp::TABLE_CLEARANCE_M + armd::grasp_select::candidates::IK_POSITION_ALLOWANCE_M)).abs() < 1e-9);
}
