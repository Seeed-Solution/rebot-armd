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
    // Direct exact-equality predicate test (zero slack below equality).
    let open_p = 0.085_f64;
    let he_p = 0.010_f64;
    let w_exact = open_p - 2.0 * he_p - 0.006;
    let fit = armd::grasp_select::admission::jaw_check(
        &armd::grasp_select::JawCut::centred(w_exact, "contour"),
        he_p,
        open_p,
        Some(0.075),
    )
    .expect("exact width + 2*hard_error + 6 mm == open_width must pass");
    assert!((fit.reserve_m - 0.003).abs() < 1e-12, "fit={fit:?}");
    assert_eq!(
        armd::grasp_select::admission::jaw_check(
            &armd::grasp_select::JawCut::centred(w_exact + 1e-6, "contour"),
            he_p,
            open_p,
            Some(0.075),
        )
        .unwrap_err()
        .code,
        "OPENING"
    );
    // Optional cap disabled (None) still enforces opening reserve.
    let fit_nocap = armd::grasp_select::admission::jaw_check(
        &armd::grasp_select::JawCut::centred(w_exact, "contour"),
        he_p,
        open_p,
        None,
    )
    .expect("exact equality passes with cap disabled");
    assert!((fit_nocap.reserve_m - 0.003).abs() < 1e-12);
    let err_nocap = armd::grasp_select::admission::jaw_check(
        &armd::grasp_select::JawCut::centred(w_exact + 1e-5, "contour"),
        he_p,
        open_p,
        None,
    )
    .expect_err("over reserve must refuse even with cap disabled");
    assert_eq!(err_nocap.code, "OPENING");

    let sc = mounted_scene(0.30, 0.0, 0.26).await;
    let (open, max_w, he) = jaw_numbers(&sc).await;
    // End-to-end camera projection: solve the exact drawn width whose projected contour width
    // equals `open - 2 * hard_error - 0.006` (zero backoff below equality).
    let w0 = open - 2.0 * he - 0.006;
    let probe = exact(request(&sc, &bar([0.30, 0.0], 0.08, w0, 0.0), 0.035), 0.035);
    let probe_ad = dry(&sc, &probe).await;
    let probe_res = probe_ad["best_refusal"]["value"].as_f64().unwrap_or(0.003);
    // Reserve error = (0.003 - probe_res) per side -> adjust drawn width by 2 * (probe_res - 0.003).
    let w = w0 + 2.0 * (probe_res - 0.003);
    assert!(w < max_w, "threshold {w} inside the cap {max_w}");
    let body = exact(request(&sc, &bar([0.30, 0.0], 0.08, w, 0.0), 0.035), 0.035);
    let leg = dry(&sc, &legacy_of(&body)).await;
    assert_eq!(leg["reachable"], true, "{leg:?}");
    assert_eq!(leg["jaw"]["status"], "checked", "{leg:?}");
    assert!((f(&leg["jaw"]["opening_reserve_m"]) - 0.003).abs() < 1e-9, "{}", leg["jaw"]);
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
    // Narrow box-only: the axis-aligned box upper bound already satisfies the jaw predicate -> checked.
    let mut narrow = legacy_of(&request(&sc, &bar([0.30, 0.0], 0.08, 0.03, 0.0), 0.035));
    narrow.as_object_mut().unwrap().remove("evidence");
    let leg = dry(&sc, &narrow).await;
    assert_eq!(leg["reachable"], true, "{leg:?}");
    assert_eq!(leg["jaw"]["status"], "checked", "{}", leg["jaw"]);
    assert!(f(&leg["jaw"]["hard_error_m"]) > 0.0);

    // Wide / rotated box-only: the box upper bound exceeds the jaw predicate -> deferred to close range.
    let mut wide = legacy_of(&request(&sc, &bar([0.30, 0.0], 0.09, 0.03, 45.0), 0.035));
    wide.as_object_mut().unwrap().remove("evidence");
    let leg_w = dry(&sc, &wide).await;
    assert_eq!(leg_w["reachable"], true, "{leg_w:?}");
    assert_eq!(leg_w["jaw"]["status"], "deferred", "{}", leg_w["jaw"]);
    assert!(f(&leg_w["jaw"]["hard_error_m"]) > 0.0);
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
    for k in ["phase", "code", "bound"] { assert_eq!(ack["best_refusal"][k], expected["best_refusal"][k], "{k}: {ack}"); }
    assert!((f(&ack["best_refusal"]["value"]) - f(&expected["best_refusal"]["value"])).abs() < 1e-12, "{ack} vs {expected:?}");
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

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn adaptive_superset_legacy_parameter_sweep_60_cases_both_owners() {
    // 5 positions x 3 heights (incl. shallow 15 mm / 25 mm where PAD_OVERLAP used to refuse -top)
    // x 4 widths (spanning pass and OPENING refusal) = 60 cases on Legacy, plus 15 on Executor.
    let positions: [[f64; 2]; 5] = [
        [0.24, 0.0],
        [0.28, 0.03],
        [0.30, 0.0],
        [0.32, -0.03],
        [0.35, 0.0],
    ];
    let heights = [0.015, 0.025, 0.045];
    let widths = [0.020, 0.035, 0.048, 0.064];
    let mut legacy_passed = 0usize;
    let mut legacy_refused = 0usize;
    let mut total_cases = 0usize;

    for pos in positions {
        let r = pos[0].hypot(pos[1]);
        let az_deg = pos[1].atan2(pos[0]).to_degrees();
        let sc = mounted_scene(r, az_deg, 0.26).await;
        for h in heights {
            for w in widths {
                total_cases += 1;
                let poly = bar(pos, 0.075, w, 0.0);
                let body = exact(request(&sc, &poly, h), h);
                let leg = dry(&sc, &legacy_of(&body)).await;
                let ad = dry(&sc, &body).await;
                assert_eq!(
                    ad["legacy"]["reachable"], leg["reachable"],
                    "pos={pos:?} h={h} w={w}: nested legacy mismatch: ad={} leg={leg:?}",
                    ad["legacy"]
                );
                assert_eq!(
                    ad["legacy"]["code"], leg["code"],
                    "pos={pos:?} h={h} w={w}: nested legacy code mismatch"
                );
                if leg["reachable"] == json!(true) {
                    legacy_passed += 1;
                    assert_eq!(
                        ad["reachable"], json!(true),
                        "adaptive must accept whenever legacy accepts (pos={pos:?} h={h} w={w}): ad={}",
                        summary(&ad)
                    );
                } else {
                    legacy_refused += 1;
                }
            }
        }
    }
    assert_eq!(total_cases, 60);
    assert!(legacy_passed >= 30, "expected >= 30 passing cases, got {legacy_passed}");
    assert!(legacy_refused >= 10, "expected >= 10 refusing cases, got {legacy_refused}");

    // Also verify adaptive >= legacy on OWNER_EXECUTOR over 12 cases (positions x heights x widths).
    let (cfg, setup) = mounted();
    let q0 = solve_camera_down(&cfg, &setup, [0.30, 0.0, 0.26]);
    let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
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
    let handle = sup
        .start_executor(armd::rt_core::executor::ExecutorConfig::for_tests(std::time::Duration::from_millis(5)))
        .await
        .expect("start RT executor");
    for _ in 0..100 {
        if sup.hub().latest().is_some_and(|s| !s.rt.owner.boot.is_zero()) {
            break;
        }
        tokio::time::sleep(std::time::Duration::from_millis(5)).await;
    }
    let cam = CamModel::assumed(&cfg.arm_model, &setup, &cfg.camera_mount);
    let start = cfg.arm_model.fk(&q0);
    let sc_exec = Scene { sup: sup.clone(), start, cam, setup: setup.clone() };
    for h in [0.015, 0.035] {
        for w in [0.022, 0.036, 0.050, 0.066] {
            let poly = bar([0.30, 0.0], 0.075, w, 0.0);
            let body = exact(request(&sc_exec, &poly, h), h);
            let leg = dry(&sc_exec, &legacy_of(&body)).await;
            let ad = dry(&sc_exec, &body).await;
            assert_eq!(ad["legacy"]["reachable"], leg["reachable"], "exec h={h} w={w}");
            if leg["reachable"] == json!(true) {
                assert_eq!(ad["reachable"], json!(true), "exec adaptive >= legacy failed h={h} w={w}: {}", summary(&ad));
            }
        }
    }
    handle.stop_and_join();
}

#[tokio::test]
async fn legacy_malformed_sibling_keeps_valid_contour_and_snapshot_id_tracks_align_j6() {
    let sc = mounted_scene(0.30, 0.0, 0.26).await;
    let (open, _, he) = jaw_numbers(&sc).await;
    let wide = bar([0.30, 0.0], 0.08, open - 2.0 * he + 0.004, 0.0);
    let mut body = legacy_of(&request(&sc, &wide, 0.035));
    // Malformed unused sibling `height_m` must NOT silently erase the valid contour into deferred!
    body["evidence"]["height_m"] = json!("not_a_number");
    let leg = dry(&sc, &body).await;
    assert_eq!((leg["reachable"].clone(), leg["code"].clone()), (json!(false), json!("OPENING")), "{leg:?}");

    // Malformed admission field (`obb_corners_norm`) must be refused as INVALID_COMMAND.
    let mut bad_obb = legacy_of(&request(&sc, &wide, 0.035));
    bad_obb["evidence"]["obb_corners_norm"] = json!([1, 2]);
    let leg_bad = dry(&sc, &bad_obb).await;
    assert_eq!((leg_bad["reachable"].clone(), leg_bad["code"].clone()), (json!(false), json!("INVALID_COMMAND")), "{leg_bad:?}");

    // Minor 1: `align_j6` must be included in `snapshot_id`.
    let body_a = request(&sc, &bar([0.30, 0.0], 0.08, 0.03, 0.0), 0.035);
    let mut body_b = body_a.clone();
    body_b["overrides"] = json!({"align_j6": false});
    let a = dry(&sc, &body_a).await;
    let b = dry(&sc, &body_b).await;
    assert_ne!(a["snapshot_id"], b["snapshot_id"], "align_j6 must change snapshot_id");
}

struct FailingModeSource;

impl DetectionSource for FailingModeSource {
    fn inference_enabled(&self) -> BoxFuture<'_, Result<bool, String>> { Box::pin(async { Ok(false) }) }
    fn set_inference(&self, _enabled: bool) -> BoxFuture<'_, Result<(), String>> { Box::pin(async { Ok(()) }) }
    fn next_frame(&self, _after: Option<u64>, _wait_ms: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> { Box::pin(async { Ok(None) }) }
    fn mode_info(&self) -> BoxFuture<'_, Result<grasp::ModeInfo, String>> { Box::pin(async { Err("provider offline".into()) }) }
}

#[tokio::test]
async fn best_refusal_ordering_past_truncation_and_task_preflight_consistency() {
    let sc = mounted_scene(0.30, 0.0, 0.26).await;
    let (open, _, he) = jaw_numbers(&sc).await;
    // Wide bar: generates > 100 rejections (well past MAX_ALTERNATIVES=6), all in `geometry`.
    // Equal-rank tie-break must pick the lowest candidate index (`-000`), NOT `-top` (99_999).
    let body = request(&sc, &bar([0.30, 0.0], 0.08, open - 2.0 * he + 0.002, 0.0), 0.035);
    let ad = dry(&sc, &body).await;
    assert_eq!(ad["reachable"], false);
    assert!(ad["rejections"]["geometry"].as_u64().unwrap() > armd::grasp_select::MAX_ALTERNATIVES as u64);
    let cid = ad["best_refusal"]["candidate_id"].as_str().expect("candidate_id");
    assert!(cid.ends_with("-000"), "lowest index must win equal-rank tie over -top, got {cid}");
    assert_eq!(ad["best_refusal"]["code"], "OPENING");

    // Task preflight failure overrides final verdict AND populates `best_refusal` consistently.
    let mut setup_task = sc.setup.clone();
    let task_spec: grasp::GraspTask = serde_json::from_value(json!({"requires": {"depth": "any"}})).unwrap();
    setup_task.tasks.insert("needs_depth".into(), task_spec);
    sc.sup.inner().write().await.set_grasp_setup(setup_task);
    let mut with_task = request(&sc, &bar([0.30, 0.0], 0.08, 0.03, 0.0), 0.035);
    with_task["task"] = json!("needs_depth");
    let res = grasp::dry_run_task(&sc.sup, &FailingModeSource, &with_task).await.fields();
    assert_eq!(res["reachable"], false, "{res:?}");
    assert_eq!(res["code"], "DETECTION_UNAVAILABLE", "{res:?}");
    assert_eq!(res["failing_phase"], "vision_preflight", "{res:?}");
    assert_eq!(res["best_refusal"]["phase"], "vision_preflight", "{res:?}");
    assert_eq!(res["best_refusal"]["code"], "DETECTION_UNAVAILABLE", "{res:?}");
    assert!(res["best_refusal"]["candidate_id"].is_null());
}

#[tokio::test]
async fn cancellation_aborts_adaptive_selection_and_elapsed_reports_total_time() {
    use std::sync::atomic::AtomicBool;
    let sc = mounted_scene(0.30, 0.0, 0.26).await;
    let body = request(&sc, &bar([0.30, 0.0], 0.08, 0.03, 0.0), 0.035);
    let inner = sc.sup.inner().read().await;
    let pending = grasp::prepare(&inner, &body, inner.safety.epoch_id()).unwrap();
    let mut pa = pending.adaptive.unwrap();
    drop(inner);
    pa.snapshot.budget.cancel = Some(Arc::new(AtomicBool::new(true)));
    let sel = armd::grasp_select::select_adaptive_route(
        &pa.snapshot, &pa.gripper_policy, &pa.gripper_limit, pa.close_speed_limit, pa.has_gravity_model,
    );
    assert_eq!(sel.selection.outcome, armd::grasp_select::Outcome::BudgetExhausted);
    assert!(sel.route.is_none());
    assert_eq!(sel.selection.stats.work.endpoint_solves, 0);

    // Uncancelled run reports total elapsed_ms > 0.
    let v = dry(&sc, &body).await;
    assert!(v["search"]["elapsed_ms"].as_f64().unwrap() > 0.0);
}

#[test]
fn error_margin_validates_residual_shapes_ranges_and_hover_acceptance_context() {
    use armd::calibration::{file_fingerprints, AcceptanceRecord, CalibrationFile};
    use armd::grasp_select::margin::{margin, quality_for, AcceptanceContext};
    use armd::tool::ActiveToolInfo;

    let mount = CameraMount {
        offset_m: Some([-0.0731, 0.0005, 0.0660]),
        q_end_cam: Some([0.0, 0.0, 0.0, 1.0]),
        ..CameraMount::default()
    };
    let offsets = vec![0.0f32; 7];
    let stock_tool = ActiveToolInfo::default();
    let ctx = AcceptanceContext { camera_mount: &mount, joint_zero_offsets: &offsets, active_tool: &stock_tool };

    // Malformed array element must NOT look complete.
    let bad_arr = CalibrationFile {
        rms: Some(json!({"hand_eye_m": 0.002, "hand_eye_deg": 0.3, "kin_heldout_mm": [1.5, "oops"], "plane_spread_m": 0.002})),
        ..Default::default()
    };
    let m = margin(&quality_for(Some(&bad_arr), Some(&ctx)), 0.01, 0.001, 0.25, 0.5);
    assert!(m.measured_budget_m.is_none() && m.reason.contains("invalid:"), "{m:?}");

    // Empty array or 3-element array is invalid.
    let empty_arr = CalibrationFile {
        rms: Some(json!({"hand_eye_m": 0.002, "hand_eye_deg": 0.3, "kin_heldout_mm": [], "plane_spread_m": 0.002})),
        ..Default::default()
    };
    let m = margin(&quality_for(Some(&empty_arr), Some(&ctx)), 0.01, 0.001, 0.25, 0.5);
    assert!(m.measured_budget_m.is_none() && m.reason.contains("invalid:"), "{m:?}");

    // Out-of-range hand_eye_deg (>= 89 deg) or array hand_eye_m is invalid.
    let bad_deg = CalibrationFile {
        rms: Some(json!({"hand_eye_m": 0.002, "hand_eye_deg": 95.0, "kin_heldout_mm": 2.0, "plane_spread_m": 0.002})),
        ..Default::default()
    };
    let m = margin(&quality_for(Some(&bad_deg), Some(&ctx)), 0.01, 0.001, 0.25, 0.5);
    assert!(m.measured_budget_m.is_none() && m.reason.contains("invalid:"), "{m:?}");

    let arr_he = CalibrationFile {
        rms: Some(json!({"hand_eye_m": [0.002], "hand_eye_deg": 0.3, "kin_heldout_mm": 2.0, "plane_spread_m": 0.002})),
        ..Default::default()
    };
    let m = margin(&quality_for(Some(&arr_he), Some(&ctx)), 0.01, 0.001, 0.25, 0.5);
    assert!(m.measured_budget_m.is_none() && m.reason.contains("invalid:"), "{m:?}");

    // Hover acceptance with mismatched fingerprint MUST be rejected and reported explicitly.
    let mut cal_hover = CalibrationFile {
        intrinsics: Some(json!({"fx": 600.0, "fy": 600.0, "cx": 320.0, "cy": 180.0, "width": 640, "height": 360})),
        camera_mount: mount.clone(),
        rms: Some(json!({"hand_eye_m": 0.002, "hand_eye_deg": 0.3, "kin_heldout_mm": 2.0})),
        ..Default::default()
    };
    let (_, valid_fp) = file_fingerprints(Some(&cal_hover), &mount, &offsets, Some(&stock_tool.config_digest));
    cal_hover.acceptance = Some(AcceptanceRecord {
        gate_version: 1,
        passed: true,
        measured_at: "2026-11-27T00:00:00Z".into(),
        board: json!({}),
        points: Vec::new(),
        job_id: None,
        fingerprint: "wrong_fingerprint".into(),
        tool_id: Some(stock_tool.id.clone()),
        tool_config_digest: Some(stock_tool.config_digest.clone()),
        summary: json!({"height_spread_m": 0.003}),
        invalidated: None,
    });
    let q_bad_fp = quality_for(Some(&cal_hover), Some(&ctx));
    assert!(q_bad_fp.hover_height_spread_m.is_none(), "{q_bad_fp:?}");
    let m_bad_fp = margin(&q_bad_fp, 0.01, 0.001, 0.25, 0.5);
    assert!(m_bad_fp.measured_budget_m.is_none() && m_bad_fp.reason.contains("mismatched:"), "{m_bad_fp:?}");

    // Matching fingerprint + wrong tool_id MUST also be rejected.
    cal_hover.acceptance.as_mut().unwrap().fingerprint = valid_fp.clone();
    cal_hover.acceptance.as_mut().unwrap().tool_id = Some("other_tool".into());
    let q_bad_tool = quality_for(Some(&cal_hover), Some(&ctx));
    assert!(q_bad_tool.hover_height_spread_m.is_none(), "{q_bad_tool:?}");
    assert!(margin(&q_bad_tool, 0.01, 0.001, 0.25, 0.5).reason.contains("mismatched:"));

    // Matching fingerprint + matching active tool IS accepted.
    cal_hover.acceptance.as_mut().unwrap().tool_id = Some(stock_tool.id.clone());
    let q_ok = quality_for(Some(&cal_hover), Some(&ctx));
    assert_eq!(q_ok.hover_height_spread_m, Some(0.003));
    assert!(margin(&q_ok, 0.01, 0.001, 0.25, 0.5).measured_budget_m.is_some());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn live_ws_refusal_carries_best_refusal_and_error_margin_both_strategies() {
    let sc = mounted_scene(0.30, 0.0, 0.26).await;
    let (open, _, he) = jaw_numbers(&sc).await;
    let body = request(&sc, &bar([0.30, 0.0], 0.08, open - 2.0 * he + 0.002, 0.0), 0.035);
    let q: Vec<f32> = sc.sup.inner().read().await.commanded_positions.clone();
    let (mut ws, generation, epoch, _sup, _addr) = common::torqued_rig(sc.sup.clone(), &q).await;
    for strategy in ["adaptive", "legacy"] {
        let mut live = body.clone();
        live["strategy"] = json!(strategy);
        let dry_res = dry(&sc, &live).await;
        live.as_object_mut().unwrap().remove("dry_run");
        let reply = common::rpc(&mut ws, generation, epoch, "/api/v1/action/grasp", live).await;
        assert_eq!(reply["type"], "command_reject", "{strategy}: {reply}");
        assert_eq!(reply["status"], 422, "{strategy}: {reply}");
        assert_eq!(reply["code"], "OPENING", "{strategy}: {reply}");
        assert_eq!(reply["failing_phase"], dry_res["failing_phase"], "{strategy}: {reply}");
        assert_eq!(reply["best_refusal"]["code"], dry_res["best_refusal"]["code"], "{strategy}: {reply}");
        assert_eq!(reply["best_refusal"]["phase"], dry_res["best_refusal"]["phase"], "{strategy}: {reply}");
        assert_eq!(reply["best_refusal"]["bound"], dry_res["best_refusal"]["bound"], "{strategy}: {reply}");
        assert_eq!(reply["error_margin"]["error_margin_source"], "fallback", "{strategy}: {reply}");
    }
}

struct LiveMockSource {
    sup: Arc<ArmSupervisor>,
    cam: CamModel,
    object_xy: [f64; 2],
    half_xy: [f64; 2],
    width_px: Option<f64>,
    angle: Option<(f64, f64)>,
    frame: std::sync::atomic::AtomicU64,
}

impl LiveMockSource {
    fn box_at(&self, pose: &Pose) -> [f64; 4] {
        let mut b = [1.0f64, 1.0, 0.0, 0.0];
        for sx in [-1.0, 1.0] {
            for sy in [-1.0, 1.0] {
                for z in [0.0, 0.04] {
                    let p = [self.object_xy[0] + sx * self.half_xy[0], self.object_xy[1] + sy * self.half_xy[1], z];
                    let px = self.cam.to_norm(self.cam.project(pose, &p).expect("visible"));
                    b = [b[0].min(px[0]), b[1].min(px[1]), b[2].max(px[0]), b[3].max(px[1])];
                }
            }
        }
        b
    }
}

impl DetectionSource for LiveMockSource {
    fn inference_enabled(&self) -> BoxFuture<'_, Result<bool, String>> { Box::pin(async { Ok(true) }) }
    fn set_inference(&self, _enabled: bool) -> BoxFuture<'_, Result<(), String>> { Box::pin(async { Ok(()) }) }
    fn next_frame(&self, _after: Option<u64>, _wait_ms: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        Box::pin(async move {
            tokio::time::sleep(std::time::Duration::from_millis(5)).await;
            let q = self.sup.inner().read().await.measured_positions.clone();
            let pose = armd::chain::ArmModel::REBOT_DEV_ARM.fk(&kinematics::q6(&q));
            let b = self.box_at(&pose);
            let (angle_deg, angle_conf) = match self.angle {
                Some((a, c)) => (Some(a), Some(c)),
                None => (None, None),
            };
            let fid = self.frame.fetch_add(1, std::sync::atomic::Ordering::SeqCst) + 1;
            Ok(Some(DetectionFrame {
                frame_id: fid,
                captured_at: std::time::Instant::now(),
                policy_captured_at: Some(std::time::Instant::now()),
                image_size_px: self.cam.size,
                detections: vec![grasp::Detection {
                    label: "cup".into(),
                    score: 0.95,
                    raw_score_valid: true,
                    box_norm: b,
                    angle_deg,
                    angle_conf,
                    grasp_width_px: self.width_px,
                    mask_focus: false,
                    disp_fg: None, disp_bg: None, depth_conf: None, length_px: None, jaw_clear: None,
                    depth_angle_deg: None, depth_angle_conf: None, depth_width_px: None, contour_norm: None,
                }],
                depth: None,
                model_generation: None,
            }))
        })
    }
}

async fn run_live_legacy_case(
    setup: GraspSetup,
    half_xy: [f64; 2],
    width_px: Option<f64>,
    angle: Option<(f64, f64)>,
) -> armd::motion::ActionStatus {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.camera_mount = CameraMount::default();
    let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
    let q0 = solve_camera_down(&cfg, &setup, [0.30, 0.0, 0.26]);
    let q7: Vec<f32> = (0..7).map(|i| if i < 6 { q0[i] as f32 } else { 0.0 }).collect();
    let cam = CamModel::assumed(&cfg.arm_model, &setup, &cfg.camera_mount);
    let start = cfg.arm_model.fk(&q0);
    let object_xy = [0.30, 0.0];
    let mut s = setup.clone();
    let approach_pose = Pose::new(grasp::look_down(&start.r), [object_xy[0], object_xy[1], s.table_z_m + s.approach_height_m]);
    let src_tmp = LiveMockSource {
        sup: sup.clone(), cam: cam.clone(), object_xy, half_xy, width_px, angle,
        frame: std::sync::atomic::AtomicU64::new(0),
    };
    s.grip_pixel_norm = grasp::box_centre(&src_tmp.box_at(&approach_pose));
    {
        let mut inner = sup.inner().write().await;
        inner.lease_enabled = false;
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).unwrap();
        armd::SupervisorInner::enable_torque(admitted).unwrap();
        inner.commanded_positions = q7.clone();
        inner.measured_positions = q7.clone();
        inner.target_positions = q7;
        inner.set_grasp_setup(s);
    }
    let src = Arc::new(src_tmp);
    let stop_ticker = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let (tsup, tstop) = (sup.clone(), stop_ticker.clone());
    let ticker = tokio::spawn(async move {
        while !tstop.load(std::sync::atomic::Ordering::Relaxed) {
            tsup.tick(0.02).await;
            {
                let mut inner = tsup.inner().write().await;
                if inner.commanded_positions[6] > -1.5 && inner.measured_positions[6] >= -1.8 {
                    inner.measured_positions[6] = -1.5;
                    inner.measured_velocities[6] = 0.0;
                    inner.torques[6] = 0.3;
                }
            }
            tokio::time::sleep(std::time::Duration::from_millis(1)).await;
        }
    });
    let tap = json!({"label": "cup", "box_norm": src.box_at(&start), "frame_id": 1});
    let (id, _) = grasp::start(&sup, sup.capture_admission_stamp(), src, &tap).await.expect("legacy start accepted");
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(20);
    let out = loop {
        assert!(std::time::Instant::now() < deadline, "live grasp timed out");
        let obs = sup.get_observation().await;
        if let Some(a) = obs.action.or(obs.previous_action).filter(|a| a.id == id && !a.is_running()) {
            break a;
        }
        tokio::time::sleep(std::time::Duration::from_millis(5)).await;
    };
    stop_ticker.store(true, std::sync::atomic::Ordering::Relaxed);
    let _ = ticker.await;
    out
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn live_legacy_insertion_enforces_77mm_opening_cap_off_absent_width_skipped_align_and_preopen() {
    // 1. Preserved 77 mm OPENING assertion (former test_grasp::pre_open_skip_conditions c3):
    // 310 px at 0.15 m, fx 600 -> ~0.077 m fails OPENING even with max_grasp_width_m = 0.10.
    let s1 = GraspSetup { pre_open: grasp::PreOpen::On, max_grasp_width_m: 0.10, ..GraspSetup::default() };
    let a1 = run_live_legacy_case(s1, [0.025, 0.015], Some(310.0), None).await;
    assert_eq!((a1.state.as_str(), a1.code.as_deref()), ("failed", Some("OPENING")), "{a1:?}");

    // 2. Cap disabled (max_grasp_width_m = 0.0) still enforces OPENING reserve!
    let s2 = GraspSetup { max_grasp_width_m: 0.0, ..GraspSetup::default() };
    let a2 = run_live_legacy_case(s2, [0.025, 0.015], Some(310.0), None).await;
    assert_eq!((a2.state.as_str(), a2.code.as_deref()), ("failed", Some("OPENING")), "{a2:?}");

    // 3. Deferred box-only (wide box > jaw reserve) with absent live width -> NO_WIDTH_EVIDENCE!
    let s3 = GraspSetup { max_grasp_width_m: 0.10, ..GraspSetup::default() };
    let a3 = run_live_legacy_case(s3, [0.036, 0.036], None, None).await;
    assert_eq!((a3.state.as_str(), a3.code.as_deref()), ("failed", Some("NO_WIDTH_EVIDENCE")), "{a3:?}");

    // 4. Skipped alignment (align_j6 = false) on elongated object (60 mm along Y, 30 mm along X):
    // detector reports short side 120 px (~27 mm) + confident angle, but jaws stay unaligned along
    // Y (~72 mm projected, under 85 mm cap but violating OPENING reserve) -> refused before descent!
    let s4 = GraspSetup { align_j6: false, max_grasp_width_m: 0.10, ..GraspSetup::default() };
    let a4 = run_live_legacy_case(s4, [0.015, 0.030], Some(120.0), Some((0.0, 0.9))).await;
    assert_eq!((a4.state.as_str(), a4.code.as_deref()), ("failed", Some("OPENING")), "{a4:?}");

    // 5. Reduced pre_open_margin_m (2 mm) must NOT narrow jaws below `w + 2*hard_error + 6 mm`:
    let s5 = GraspSetup { pre_open: grasp::PreOpen::On, pre_open_margin_m: 0.002, ..GraspSetup::default() };
    let a5 = run_live_legacy_case(s5, [0.020, 0.015], Some(140.0), None).await;
    assert_eq!(a5.state.as_str(), "done", "{a5:?}");
    assert_eq!(a5.pre_open.as_deref(), Some("done"), "{a5:?}");
    let meas_w = f64::from(a5.grasp_width_m.expect("grasp_width_m recorded"));
    let pre_w = f64::from(a5.pre_open_m.expect("pre_open_m recorded"));
    // Raw w + 0.002 would be ~0.0335 m; clamped jaw reserve floor raises it by > 20 mm (to ~0.0579 m).
    assert!(pre_w >= meas_w + 0.024, "pre_open {pre_w} narrowed below jaw reserve floor for w={meas_w}");
}
