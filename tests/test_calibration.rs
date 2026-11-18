//! Camera calibration: `camera_mount.q_end_cam`, `calibration.json` precedence, override
//! refusals, and the WS apply / restore transaction against a mock rebot-vision.
mod common;

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use armd::calibration::{self, CalibrationDeps, CalibrationFile};
use armd::chain::ArmModel;
use armd::grasp::{self, CamModel, GraspDeps, GraspSetup};
use armd::kinematics as k;
use armd::model::CameraMount;
use armd::{ArmConfig, ArmSupervisor};
use axum::extract::State;
use axum::http::StatusCode;
use axum::response::IntoResponse;
use serde_json::{json, Map, Value};

// ---- mount quaternion ----

#[test]
fn quaternion_mount_equals_the_legacy_matrix_and_gives_back_the_tilt() {
    for arm in [ArmModel::REBOT_DEV_ARM] {
        for (flip, rot, tilt) in [(false, 0.0, -38.0), (false, 0.0, 0.0), (true, 0.0, 20.0), (false, 30.0, -25.0), (true, -60.0, 12.0), (false, 5.0, -4.0)] {
            let legacy = arm.cam_mount(flip, rot, tilt);
            let q = k::r_to_quat(&legacy);
            let mount = CameraMount { flip, rot_deg: rot as f32, q_end_cam: Some(q), ..Default::default() };
            let r = arm.r_end_cam(&mount, 99.0); // the setup's tilt is ignored with q_end_cam
            for i in 0..3 { for j in 0..3 { assert!((r[i][j] - legacy[i][j]).abs() < 1e-9, "{flip} {rot} {tilt}"); } }
            let derived = arm.mount_tilt_deg(&mount);
            assert!((derived - tilt).abs() < 1e-6, "flip {flip} rot {rot}: tilt {tilt} derived {derived}");
            // without q the legacy fields decide
            let plain = CameraMount { flip, rot_deg: rot as f32, tilt_deg: tilt as f32, ..Default::default() };
            let rp = arm.r_end_cam(&plain, tilt);
            for i in 0..3 { for j in 0..3 { assert!((rp[i][j] - legacy[i][j]).abs() < 1e-9); } }
        }
    }
    // unit check within 1e-3, normalised
    assert!(armd::model::unit_quaternion(&[1.0005, 0.0, 0.0, 0.0]).is_ok_and(|q| (q[0] - 1.0).abs() < 1e-12));
    assert!(armd::model::unit_quaternion(&[1.01, 0.0, 0.0, 0.0]).is_err());
    assert!(armd::model::unit_quaternion(&[f64::NAN, 0.0, 0.0, 1.0]).is_err());
    let mut cfg = ArmConfig::rebot_dm();
    cfg.camera_mount.q_end_cam = Some([0.5, 0.5, 0.5, 0.6]);
    assert!(cfg.validate().is_err(), "a profile with a non-unit q_end_cam is invalid");
}

#[test]
fn camera_model_uses_the_principal_point() {
    let setup = GraspSetup { cx_px: Some(300.0), cy_px: Some(190.0), ..Default::default() };
    let cam = CamModel::assumed(&ArmModel::REBOT_DEV_ARM, &setup, &CameraMount::default());
    assert_eq!((cam.cx, cam.cy), (300.0, 190.0));
    let cam = CamModel::assumed(&ArmModel::REBOT_DEV_ARM, &GraspSetup::default(), &CameraMount::default());
    assert_eq!((cam.cx, cam.cy), (320.0, 180.0), "default: the image centre");
    assert!(GraspSetup { cx_px: Some(700.0), ..Default::default() }.validate().is_err());
    // the optional fields can be patched and unset
    let s = GraspSetup::default().patched(&json!({"cx_px": 321.5})).unwrap();
    assert_eq!(s.cx_px, Some(321.5));
    assert_eq!(s.patched(&json!({"cx_px": null})).unwrap().cx_px, None);
}

// ---- startup precedence and override refusals ----

fn v8_q() -> [f64; 4] { k::r_to_quat(&ArmModel::REBOT_DEV_ARM.cam_mount(false, 0.0, -38.0)) }

fn calib_file(q: [f64; 4]) -> CalibrationFile {
    CalibrationFile {
        camera_mount: CameraMount { offset_m: Some([-0.0731, 0.0005, 0.066]), q_end_cam: Some(q), ..Default::default() },
        intrinsics: Some(json!({"fx": 830.0})), table_z_m: Some(0.01), solved_at: Some("2026-11-17T00:00:00Z".into()), rms: None,
        kinematics: None,
    }
}

async fn calibrated_supervisor(dir: &Path) -> ArmSupervisor {
    let setup = dir.join("grasp_setup.json");
    std::fs::write(&setup, r#"{"camera_tilt_deg": 25.0, "fx_px": 700.0, "fy_px": 700.0, "grip_pixel_norm": [0.4, 0.6]}"#).unwrap();
    let cal = dir.join("calibration.json");
    std::fs::write(&cal, serde_json::to_string(&calib_file(v8_q())).unwrap()).unwrap();
    let sup = ArmSupervisor::new(ArmConfig::rebot_dm());
    grasp::load_into(&sup, &setup).await;
    calibration::load_into(&sup, &cal, Some(&setup)).await;
    sup
}

#[tokio::test]
async fn startup_calibration_wins_for_the_mount_and_tilt() {
    let dir = tempfile::tempdir().unwrap();
    let sup = calibrated_supervisor(dir.path()).await;
    let inner = sup.inner().read().await;
    assert!(inner.camera_calibrated());
    let m = &inner.config.camera_mount;
    assert!(m.q_end_cam.is_some() && m.offset_m == Some([-0.0731, 0.0005, 0.066]));
    assert!((m.tilt_deg + 38.0).abs() < 1e-3, "derived tilt {}", m.tilt_deg);
    let g = inner.grasp_setup().unwrap();
    assert!((g.camera_tilt_deg + 38.0).abs() < 1e-3, "calibration wins over the setup file's tilt 25");
    assert_eq!((g.fx_px, g.grip_pixel_norm), (700.0, [0.4, 0.6]), "the other setup fields come from the setup file");
    drop(inner);
    // an invalid file is set aside and the profile mount stays
    let bad = dir.path().join("bad.json");
    std::fs::write(&bad, r#"{"camera_mount": {"q_end_cam": [2, 0, 0, 0], "offset_m": [0, 0, 0]}}"#).unwrap();
    let fresh = ArmSupervisor::new(ArmConfig::rebot_dm());
    calibration::load_into(&fresh, &bad, None).await;
    assert!(!fresh.inner().read().await.camera_calibrated());
    assert!(!bad.exists(), "quarantined");
}

#[tokio::test]
async fn calibrated_fields_cannot_be_overridden() {
    let dir = tempfile::tempdir().unwrap();
    let sup = calibrated_supervisor(dir.path()).await;
    let mut inner = sup.inner().write().await;
    let msg = |r: Result<GraspSetup, armd::supervisor::ArmError>| match r { Err(e) => e.to_string(), Ok(_) => "accepted".into() };
    for f in ["camera_tilt_deg", "fx_px", "fy_px", "cx_px", "cy_px", "image_size_px"] {
        let v = if f == "image_size_px" { json!([1280.0, 720.0]) } else { json!(12.0) };
        let m = msg(grasp::update_setup(&mut inner, &json!({ f: v })));
        assert!(m.contains(f) && m.contains("calibrated"), "{f}: {m}");
    }
    // grip_pixel_norm and table_z_m stay writable; an unchanged value is no change
    assert!(grasp::update_setup(&mut inner, &json!({"grip_pixel_norm": [0.45, 0.55], "table_z_m": 0.02, "fx_px": 700})).is_ok());
    let m = msg(grasp::update_setup(&mut inner, &json!({"tasks": {"t": {"overrides": {"fy_px": 500.0}}}})));
    assert!(m.contains("fy_px"), "{m}");
    // request overrides
    let epoch = inner.safety.epoch_id();
    let req = json!({"label": "cup", "box_norm": [0.4, 0.4, 0.6, 0.6], "overrides": {"camera_tilt_deg": 0.0}});
    match grasp::prepare(&inner, &req, epoch) {
        Err(e) => assert!(e.to_string().contains("camera_tilt_deg"), "{e}"),
        Ok(_) => panic!("request override accepted"),
    }
    let req = json!({"label": "cup", "box_norm": [0.4, 0.4, 0.6, 0.6], "overrides": {"grip_pixel_norm": [0.5, 0.5]}});
    if let Err(e) = grasp::prepare(&inner, &req, epoch) { assert!(!e.to_string().contains("calibrated"), "{e}"); }
}

// ---- mock rebot-vision ----

#[derive(Default)]
struct MockVision {
    values: Map<String, Value>,
    revision: u64,
    puts: usize,
    /// The PUT with this 1-based index fails (500).
    fail_put: Option<usize>,
    unsupported: bool,
    put_delay_ms: u64,
    /// The PUT with this 1-based index is applied, then answers 500 (dir fsync after the rename).
    fail_put_after_apply: Option<usize>,
    /// The PUT with this 1-based index is applied, then its reply waits this long (ms).
    slow_reply_after_apply: Option<(usize, u64)>,
    /// GET answers 500 once this many PUTs arrived.
    fail_get_after_puts: Option<usize>,
}

type Mock = Arc<Mutex<MockVision>>;

fn mock_body(m: &MockVision) -> Value {
    let mut b = m.values.clone();
    b.insert("revision".into(), json!(m.revision));
    b.insert("api_version".into(), json!(2));
    b.insert("overridden_by_cli".into(), json!(false));
    Value::Object(b)
}

async fn mock_get(State(m): State<Mock>) -> axum::response::Response {
    let m = m.lock().unwrap();
    if m.unsupported { return (StatusCode::NOT_FOUND, axum::Json(json!({"error": "not found"}))).into_response(); }
    if m.fail_get_after_puts.is_some_and(|n| m.puts >= n) { return (StatusCode::INTERNAL_SERVER_ERROR, axum::Json(json!({"error": "busy"}))).into_response(); }
    axum::Json(mock_body(&m)).into_response()
}

async fn mock_put(State(m): State<Mock>, axum::Json(body): axum::Json<Value>) -> axum::response::Response {
    let delay = { let mut g = m.lock().unwrap(); g.puts += 1; g.put_delay_ms };
    tokio::time::sleep(Duration::from_millis(delay)).await;
    let (reply, slow) = {
        let mut m = m.lock().unwrap();
        if m.unsupported { return (StatusCode::NOT_FOUND, axum::Json(json!({"error": "not found"}))).into_response(); }
        if m.fail_put == Some(m.puts) { return (StatusCode::INTERNAL_SERVER_ERROR, axum::Json(json!({"error": "disk full"}))).into_response(); }
        if body["expected_revision"].as_u64() != Some(m.revision) {
            return (StatusCode::CONFLICT, axum::Json(json!({"error": "REVISION_MISMATCH", "revision": m.revision}))).into_response();
        }
        for k in ["fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3", "width", "height", "marker_size"] {
            if let Some(v) = body.get(k) { m.values.insert(k.into(), v.clone()); }
        }
        m.revision += 1;
        if m.fail_put_after_apply == Some(m.puts) { return (StatusCode::INTERNAL_SERVER_ERROR, axum::Json(json!({"error": "fsync failed"}))).into_response(); }
        (mock_body(&m), m.slow_reply_after_apply.filter(|(n, _)| *n == m.puts).map(|(_, ms)| ms))
    };
    if let Some(ms) = slow { tokio::time::sleep(Duration::from_millis(ms)).await; }
    axum::Json(reply).into_response()
}

async fn spawn_mock() -> (String, Mock) {
    let mut values = Map::new();
    for (k, v) in [("fx", 824.0), ("fy", 824.0), ("cx", 320.0), ("cy", 180.0), ("k1", 0.0), ("k2", 0.0), ("p1", 0.0), ("p2", 0.0), ("k3", 0.0), ("marker_size", 0.05)] {
        values.insert(k.into(), json!(v));
    }
    values.insert("width".into(), json!(640));
    values.insert("height".into(), json!(360));
    let mock: Mock = Arc::new(Mutex::new(MockVision { values, ..Default::default() }));
    let app = axum::Router::new()
        .route("/api/calibration/intrinsics", axum::routing::get(mock_get).put(mock_put))
        .with_state(mock.clone());
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let url = format!("http://{}", listener.local_addr().unwrap());
    tokio::spawn(async move { axum::serve(listener, app).await.unwrap(); });
    (url, mock)
}

struct Rig {
    sup: Arc<ArmSupervisor>,
    ws: common::TestSocket,
    generation: u64,
    epoch: u64,
    addr: std::net::SocketAddr,
    mock: Mock,
    deps: CalibrationDeps,
    _dir: tempfile::TempDir,
}

/// armd with a lease holder, a mock rebot-vision and calibration files in a temp dir;
/// `paths` may replace the calibration / grasp setup paths.
async fn rig_with(paths: impl FnOnce(&Path) -> (PathBuf, PathBuf)) -> Rig { rig_cfg(ArmConfig::rebot_dm(), paths).await }

async fn rig_cfg(config: ArmConfig, paths: impl FnOnce(&Path) -> (PathBuf, PathBuf)) -> Rig {
    let dir = tempfile::tempdir().unwrap();
    let (url, mock) = spawn_mock().await;
    let up = armd::api::proxy::DetectorUpstream::parse(&url).unwrap();
    let (cal, setup) = paths(dir.path());
    let mut deps = CalibrationDeps::new(cal, Some(setup.clone()), up.clone());
    deps.backup_dir = dir.path().join("backups");
    deps.vision_timeout = Duration::from_millis(1500);
    let sup = Arc::new(ArmSupervisor::new(config));
    let router = armd::api::create_router_calibration(sup.clone(), up.clone(), GraspDeps::http(up, Some(setup)),
        armd::diagnostics::DiagnosticsState::global(), deps.clone());
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });
    let (ws, generation) = common::controller(addr).await;
    let epoch = sup.get_observation().await.epoch_id;
    Rig { sup, ws, generation, epoch, addr, mock, deps, _dir: dir }
}

async fn rig() -> Rig { rig_with(|d| (d.join("calibration.json"), d.join("grasp_setup.json"))).await }

fn q_tilt(t: f64) -> [f64; 4] { k::r_to_quat(&ArmModel::REBOT_DEV_ARM.cam_mount(false, 0.0, t)) }

/// Apply body with the mount tilted -15 deg (grip point inside the image at approach height).
fn apply_body(dry_run: bool) -> Value { apply_body_q(dry_run, q_tilt(-15.0)) }

fn apply_body_q(dry_run: bool, q: [f64; 4]) -> Value {
    json!({"intrinsics": {"fx": 831.5, "fy": 829.0, "cx": 318.2, "cy": 183.4, "k1": 0.05, "k2": -0.1, "p1": 0.0, "p2": 0.0, "k3": 0.0,
            "width": 640, "height": 360, "marker_size": 0.0495},
        "camera_mount": {"offset_m": [-0.0728, 0.0012, 0.0655], "q_end_cam": q},
        "table_z_m": 0.012, "rms": {"intrinsics_px": 0.21, "hand_eye_mm": 1.4, "hand_eye_deg": 0.3}, "dry_run": dry_run})
}

static REQ: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1_000_000);

async fn send_cmd(r: &mut Rig, path: &str, body: Value) -> u64 {
    let id = REQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    common::send(&mut r.ws, json!({"type":"command", "request_id":id, "generation":r.generation, "epoch_id":r.epoch, "path":path, "body":body})).await;
    id
}

/// The reply to request `id` (other replies are skipped); heartbeats keep the lease.
async fn reply_for(r: &mut Rig, id: u64) -> Value {
    let deadline = tokio::time::Instant::now() + Duration::from_secs(10);
    loop {
        assert!(tokio::time::Instant::now() < deadline, "no reply to {id}");
        let v = tokio::time::timeout(Duration::from_millis(300), common::next_of(&mut r.ws, &["command_ack", "command_reject"])).await;
        match v {
            Ok(v) if v["request_id"] == json!(id) => return v,
            Ok(_) => {}
            Err(_) => common::send(&mut r.ws, json!({"type":"heartbeat", "generation":r.generation, "epoch_id":r.epoch})).await,
        }
    }
}

async fn cmd(r: &mut Rig, path: &str, body: Value) -> Value {
    let id = send_cmd(r, path, body).await;
    reply_for(r, id).await
}

fn mock_fx(r: &Rig) -> (f64, u64) { let m = r.mock.lock().unwrap(); (m.values["fx"].as_f64().unwrap(), m.revision) }

fn backups(r: &Rig) -> usize { std::fs::read_dir(&r.deps.backup_dir).map_or(0, |d| d.count()) }

#[tokio::test]
async fn apply_dry_run_reports_the_diff_and_writes_nothing() {
    let mut r = rig().await;
    let reply = cmd(&mut r, "/api/v1/calibration/apply", apply_body_q(true, v8_q())).await;
    assert_eq!(reply["type"], "command_ack", "{reply}");
    // v8: the point under the jaws is above the image at approach height; grip_pixel_norm stays
    let gp = reply["after"]["grip_pixel_model"].as_array().unwrap();
    assert!(gp[1].as_f64().unwrap() < 0.0, "{reply}");
    assert_eq!(reply["after"]["grasp"]["grip_pixel_norm"], json!([0.5, 0.5]));
    assert_eq!(reply["before"]["vision_intrinsics"]["fx"], json!(824.0));
    assert_eq!(reply["after"]["vision_intrinsics"]["fx"], json!(831.5));
    assert_eq!(reply["before"]["camera_mount"]["q_end_cam"], Value::Null);
    assert!(reply["after"]["camera_mount"]["q_end_cam"].is_array());
    assert!((reply["after"]["camera_mount"]["tilt_deg"].as_f64().unwrap() + 38.0).abs() < 1e-3);
    assert_eq!(reply["before"]["grasp"]["fx_px"], json!(600.0));
    assert_eq!(reply["after"]["grasp"]["fx_px"], json!(831.5));
    assert_eq!(reply["after"]["grasp"]["cx_px"], json!(318.2));
    assert_eq!(reply["after"]["grasp"]["table_z_m"], json!(0.012));
    assert_eq!(mock_fx(&r), (824.0, 0));
    assert!(!r.deps.path.exists() && backups(&r) == 0);
    assert!(!r.sup.inner().read().await.camera_calibrated());
}

#[tokio::test]
async fn apply_writes_vision_calibration_and_grasp_setup() {
    let mut r = rig().await;
    let reply = cmd(&mut r, "/api/v1/calibration/apply", apply_body(false)).await;
    assert_eq!(reply["type"], "command_ack", "{reply}");
    assert_eq!(mock_fx(&r), (831.5, 1));
    assert_eq!(reply["vision_intrinsics"]["fx"], json!(831.5), "read back from rebot-vision");
    assert_eq!(reply["vision_intrinsics"]["revision"], json!(1));
    // backup holds the state before
    let backup = PathBuf::from(reply["backup"].as_str().unwrap());
    let before: Value = serde_json::from_str(&std::fs::read_to_string(backup.join("vision_intrinsics.json")).unwrap()).unwrap();
    assert_eq!(before["fx"], json!(824.0));
    assert!(backup.join("manifest.json").is_file());
    // calibration.json and the live mount
    let file: CalibrationFile = serde_json::from_str(&std::fs::read_to_string(&r.deps.path).unwrap()).unwrap();
    assert_eq!(file.camera_mount.offset_m, Some([-0.0728, 0.0012, 0.0655]));
    assert_eq!(file.table_z_m, Some(0.012));
    assert_eq!(file.intrinsics.as_ref().unwrap()["fx"], json!(831.5));
    let inner = r.sup.inner().read().await;
    assert!(inner.camera_calibrated());
    let mount = inner.config.camera_mount.clone();
    let g = inner.grasp_setup().unwrap();
    drop(inner);
    assert!((g.camera_tilt_deg + 15.0).abs() < 1e-3);
    assert_eq!((g.fx_px, g.fy_px, g.cx_px, g.cy_px, g.table_z_m, g.image_size_px), (831.5, 829.0, Some(318.2), Some(183.4), 0.012, [640.0, 360.0]));
    // grip pixel: armd's camera model at the look-down approach pose, on the grasp plane
    let cam = CamModel::assumed(&ArmModel::REBOT_DEV_ARM, &g, &mount);
    let d = g.approach_height_m - g.grasp_height_m;
    let pc = k::mat_vec(&k::transpose(&cam.rec), &k::sub(&[d, 0.0, 0.0], &cam.offset));
    let want = [(831.5 * pc[0] / pc[2] + 318.2) / 640.0, (829.0 * pc[1] / pc[2] + 183.4) / 360.0];
    assert!((g.grip_pixel_norm[0] - want[0]).abs() < 1e-4 && (g.grip_pixel_norm[1] - want[1]).abs() < 1e-4, "{:?} {want:?}", g.grip_pixel_norm);
    assert!(want.iter().all(|v| (0.0..=1.0).contains(v)), "{want:?}");
    assert_eq!(reply["grasp"]["grip_pixel_norm"], json!(g.grip_pixel_norm));
    // grasp_setup.json persisted
    let saved: Value = serde_json::from_str(&std::fs::read_to_string(r.deps.setup_path.as_ref().unwrap()).unwrap()).unwrap();
    assert_eq!(saved["fx_px"], json!(831.5));
    // GET /api/v1/calibration
    let v: Value = reqwest::get(format!("http://{}/api/v1/calibration", r.addr)).await.unwrap().json().await.unwrap();
    assert_eq!((v["calibrated"].as_bool(), v["busy"].as_bool()), (Some(true), Some(false)), "{v}");
    assert_eq!(v["backups"].as_array().unwrap().len(), 1);
    assert_eq!(v["backups"][0]["reason"], "apply");
    // capability
    assert!(r.sup.get_schema().await.capabilities.calibration_apply);
    // setup writes of calibrated fields are refused over WS now
    let rej = cmd(&mut r, "/api/v1/grasp/setup", json!({"camera_tilt_deg": 10.0})).await;
    assert_eq!((rej["type"].as_str(), rej["code"].as_str()), (Some("command_reject"), Some("INVALID_COMMAND")), "{rej}");
    assert!(rej["message"].as_str().unwrap().contains("camera_tilt_deg"));
}

#[tokio::test]
async fn restore_puts_a_backup_back() {
    let mut r = rig().await;
    let reply = cmd(&mut r, "/api/v1/calibration/apply", apply_body(false)).await;
    let name = Path::new(reply["backup"].as_str().unwrap()).file_name().unwrap().to_string_lossy().into_owned();
    let back = cmd(&mut r, "/api/v1/calibration/restore", json!({"backup": name})).await;
    assert_eq!(back["type"], "command_ack", "{back}");
    assert_eq!(back["restored"], json!(name));
    assert_eq!(mock_fx(&r), (824.0, 2));
    assert!(!r.deps.path.exists(), "the backup had no calibration.json");
    let inner = r.sup.inner().read().await;
    assert!(!inner.camera_calibrated() && inner.config.camera_mount.q_end_cam.is_none());
    let g = inner.grasp_setup().unwrap();
    assert_eq!((g.fx_px, g.cx_px, g.camera_tilt_deg), (600.0, None, 0.0));
    drop(inner);
    assert_eq!(backups(&r), 2, "restore backs up first");
    let rej = cmd(&mut r, "/api/v1/calibration/restore", json!({"backup": "nope"})).await;
    assert_eq!(rej["code"], "BACKUP_NOT_FOUND", "{rej}");
    let rej = cmd(&mut r, "/api/v1/calibration/restore", json!({"backup": "../x"})).await;
    assert_eq!(rej["code"], "INVALID_COMMAND", "{rej}");
}

async fn assert_unchanged(r: &Rig) {
    let inner = r.sup.inner().read().await;
    assert!(!inner.camera_calibrated() && inner.config.camera_mount.q_end_cam.is_none());
    assert_eq!(inner.grasp_setup().unwrap().fx_px, 600.0);
    assert!(!inner.calibration_busy);
}

#[tokio::test]
async fn apply_fails_before_any_write_without_the_vision_route() {
    let mut r = rig().await;
    r.mock.lock().unwrap().unsupported = true;
    for dry in [true, false] {
        let rej = cmd(&mut r, "/api/v1/calibration/apply", apply_body(dry)).await;
        assert_eq!(rej["code"], "VISION_CALIBRATION_UNSUPPORTED", "{rej}");
    }
    assert_eq!(backups(&r), 0);
    assert!(!r.deps.path.exists());
    assert_unchanged(&r).await;
}

#[tokio::test]
async fn apply_vision_failure_changes_nothing() {
    let mut r = rig().await;
    r.mock.lock().unwrap().fail_put = Some(1);
    let rej = cmd(&mut r, "/api/v1/calibration/apply", apply_body(false)).await;
    assert_eq!((rej["code"].as_str(), rej["step"].as_str()), (Some("VISION_ERROR"), Some("vision")), "{rej}");
    assert_eq!(mock_fx(&r), (824.0, 0));
    assert!(!r.deps.path.exists());
    assert!(rej["backup"].is_string());
    assert_unchanged(&r).await;
}

#[tokio::test]
async fn apply_calibration_file_failure_reverts_vision() {
    // calibration.json is a directory: the rename fails
    let mut r = rig_with(|d| { std::fs::create_dir(d.join("cal")).unwrap(); (d.join("cal"), d.join("grasp_setup.json")) }).await;
    let rej = cmd(&mut r, "/api/v1/calibration/apply", apply_body(false)).await;
    assert_eq!((rej["code"].as_str(), rej["step"].as_str(), rej["reverted"].as_bool()), (Some("PERSIST_FAILED"), Some("calibration_file"), Some(true)), "{rej}");
    assert_eq!(mock_fx(&r), (824.0, 2), "PUT, then the revert PUT with the new revision");
    assert_unchanged(&r).await;
}

#[tokio::test]
async fn apply_grasp_setup_failure_reverts_file_live_state_and_vision() {
    let mut r = rig_with(|d| { std::fs::create_dir(d.join("gs")).unwrap(); (d.join("calibration.json"), d.join("gs")) }).await;
    let rej = cmd(&mut r, "/api/v1/calibration/apply", apply_body(false)).await;
    assert_eq!((rej["code"].as_str(), rej["step"].as_str(), rej["reverted"].as_bool()), (Some("PERSIST_FAILED"), Some("grasp_setup"), Some(true)), "{rej}");
    assert_eq!(mock_fx(&r), (824.0, 2));
    assert!(!r.deps.path.exists(), "calibration.json removed again (there was none)");
    assert_unchanged(&r).await;
}

#[tokio::test]
async fn failed_revert_reports_calibration_inconsistent() {
    let mut r = rig_with(|d| { std::fs::create_dir(d.join("cal")).unwrap(); (d.join("cal"), d.join("grasp_setup.json")) }).await;
    r.mock.lock().unwrap().fail_put = Some(2);
    let rej = cmd(&mut r, "/api/v1/calibration/apply", apply_body(false)).await;
    assert_eq!((rej["code"].as_str(), rej["step"].as_str()), (Some("CALIBRATION_INCONSISTENT"), Some("calibration_file")), "{rej}");
    assert!(rej["state"]["vision"].as_str().unwrap().starts_with("revert failed"), "{rej}");
    assert!(rej["backup"].is_string());
    assert_eq!(mock_fx(&r), (831.5, 1), "rebot-vision kept the new values");
    assert_unchanged(&r).await;
}

#[tokio::test]
async fn motion_and_config_writes_are_refused_while_an_apply_runs() {
    let mut r = rig().await;
    r.mock.lock().unwrap().put_delay_ms = 700;
    let apply = send_cmd(&mut r, "/api/v1/calibration/apply", apply_body(false)).await;
    tokio::time::sleep(Duration::from_millis(150)).await;
    assert!(r.sup.inner().read().await.calibration_busy);
    let jog = cmd(&mut r, "/api/v1/motion/home", json!({})).await;
    assert_eq!(jog["code"], "MOTION_BUSY", "{jog}");
    let mv = cmd(&mut r, "/api/v1/action/move_to", json!({"position": [0.25, 0.0, 0.2]})).await;
    assert_eq!(mv["code"], "MOTION_BUSY", "{mv}");
    let setup = cmd(&mut r, "/api/v1/grasp/setup", json!({"table_z_m": 0.0})).await;
    assert_eq!(setup["code"], "CONFIG_BUSY", "{setup}");
    let again = cmd(&mut r, "/api/v1/calibration/apply", apply_body(false)).await;
    assert_eq!(again["code"], "CONFIG_BUSY", "{again}");
    let stop = cmd(&mut r, "/api/v1/motion/stop", json!({})).await;
    assert_eq!(stop["type"], "command_ack", "stop goes through: {stop}");
    let done = reply_for(&mut r, apply).await;
    assert_eq!(done["type"], "command_ack", "{done}");
    assert!(!r.sup.inner().read().await.calibration_busy);
    // and a busy arm refuses the apply
    r.sup.inner().write().await.commanded_velocities[0] = 0.5;
    let busy = cmd(&mut r, "/api/v1/calibration/apply", apply_body(true)).await;
    assert_eq!(busy["code"], "MOTION_BUSY", "{busy}");
}

#[tokio::test]
async fn apply_rejects_bad_bodies_and_needs_the_lease() {
    let mut r = rig().await;
    for (patch, what) in [
        (json!({"camera_mount": {"offset_m": [0, 0, 0], "q_end_cam": [2, 0, 0, 0]}}), "q_end_cam"),
        (json!({"camera_mount": {"offset_m": [0, 0], "q_end_cam": [1, 0, 0, 0]}}), "offset_m"),
        (json!({"intrinsics": {"fx": 50}}), "intrinsics"),
        (json!({"table_z_m": 2.0}), "table_z_m"),
        (json!({"bogus": 1}), "bogus"),
    ] {
        let mut body = apply_body(true);
        for (k, v) in patch.as_object().unwrap() { body[k] = v.clone(); }
        let rej = cmd(&mut r, "/api/v1/calibration/apply", body).await;
        assert_eq!(rej["code"], "INVALID_COMMAND", "{what}: {rej}");
        assert!(rej["message"].as_str().unwrap().contains(what), "{what}: {rej}");
    }
    let (mut other, _) = tokio_tungstenite::connect_async(format!("ws://{}/api/v1/ws", r.addr)).await.unwrap();
    let rej = common::ws_reply(&mut other, json!({"type":"command", "request_id":1, "generation":r.generation, "epoch_id":r.epoch,
        "path":"/api/v1/calibration/apply", "body":apply_body(true)}), &["command_reject"]).await;
    assert_eq!(rej["code"], "LEASE_LOST");
}

// ---- rebot-vision applied the PUT although its reply failed (review M3) ----

/// The vision values equal the ones before the apply (fx 824), at `rev`.
fn assert_vision_old(r: &Rig, rev: u64) {
    let m = r.mock.lock().unwrap();
    assert_eq!((m.values["fx"].as_f64(), m.values["cx"].as_f64(), m.values["k1"].as_f64(), m.revision), (Some(824.0), Some(320.0), Some(0.0), rev));
}

#[tokio::test]
async fn vision_put_timeout_after_apply_is_reverted() {
    let mut r = rig().await;
    r.mock.lock().unwrap().slow_reply_after_apply = Some((1, 2500)); // > the 1.5 s vision timeout
    let rej = cmd(&mut r, "/api/v1/calibration/apply", apply_body(false)).await;
    assert_eq!((rej["code"].as_str(), rej["step"].as_str(), rej["reverted"].as_bool()), (Some("VISION_UNREACHABLE"), Some("vision"), Some(true)), "{rej}");
    assert_vision_old(&r, 2); // applied (rev 1), then reverted with the current revision
    assert!(!r.deps.path.exists());
    assert_unchanged(&r).await;
}

#[tokio::test]
async fn vision_put_500_after_rename_is_reverted() {
    let mut r = rig().await;
    r.mock.lock().unwrap().fail_put_after_apply = Some(1);
    let rej = cmd(&mut r, "/api/v1/calibration/apply", apply_body(false)).await;
    assert_eq!((rej["code"].as_str(), rej["step"].as_str(), rej["reverted"].as_bool()), (Some("VISION_ERROR"), Some("vision"), Some(true)), "{rej}");
    assert_vision_old(&r, 2);
    assert_unchanged(&r).await;
}

#[tokio::test]
async fn vision_put_and_read_back_failing_is_inconsistent() {
    let mut r = rig().await;
    { let mut m = r.mock.lock().unwrap(); m.fail_put_after_apply = Some(1); m.fail_get_after_puts = Some(1); }
    let rej = cmd(&mut r, "/api/v1/calibration/apply", apply_body(false)).await;
    assert_eq!((rej["code"].as_str(), rej["step"].as_str()), (Some("CALIBRATION_INCONSISTENT"), Some("vision")), "{rej}");
    assert_eq!(rej["state"]["vision"], "unknown", "{rej}");
    assert!(rej["backup"].as_str().is_some_and(|b| std::path::Path::new(b).join("vision_intrinsics.json").is_file()), "{rej}");
    assert_eq!(r.mock.lock().unwrap().puts, 1, "no blind revert PUT");
    assert_unchanged(&r).await;
}

#[tokio::test]
async fn restore_vision_put_500_after_rename_is_reverted() {
    let mut r = rig().await;
    let ok = cmd(&mut r, "/api/v1/calibration/apply", apply_body(false)).await;
    assert_eq!(ok["type"], "command_ack", "{ok}");
    let name = ok["backup"].as_str().unwrap().rsplit('/').next().unwrap().to_string();
    assert_eq!(mock_fx(&r), (831.5, 1));
    r.mock.lock().unwrap().fail_put_after_apply = Some(2);
    let rej = cmd(&mut r, "/api/v1/calibration/restore", json!({"backup": name})).await;
    assert_eq!((rej["code"].as_str(), rej["step"].as_str(), rej["reverted"].as_bool()), (Some("VISION_ERROR"), Some("vision"), Some(true)), "{rej}");
    assert_eq!(mock_fx(&r), (831.5, 3), "restore PUT applied (rev 2), reverted to the applied values (rev 3)");
    assert!(r.sup.inner().read().await.camera_calibrated(), "armd keeps the applied calibration");
    assert!(!r.sup.inner().read().await.calibration_busy);
}

// ---- busy flag guard (review L3) ----

#[tokio::test]
async fn busy_guard_clears_the_flag_on_panic_and_on_release() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    // panic while holding the supervisor lock: the guard clears the flag once the lock is free
    sup.inner().write().await.calibration_busy = true;
    let s2 = sup.clone();
    let task = tokio::spawn(async move {
        let _guard = calibration::BusyGuard::new(s2.clone());
        let _lock = s2.inner().write().await;
        panic!("forced panic in the apply task");
    });
    assert!(task.await.unwrap_err().is_panic());
    let deadline = tokio::time::Instant::now() + Duration::from_secs(2);
    while sup.inner().read().await.calibration_busy {
        assert!(tokio::time::Instant::now() < deadline, "busy flag still set after the panic");
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    // early return (guard dropped without release), lock free
    sup.inner().write().await.calibration_busy = true;
    async fn early(sup: Arc<ArmSupervisor>) -> Result<(), ()> { let _g = calibration::BusyGuard::new(sup); Err(()) }
    assert!(early(sup.clone()).await.is_err());
    assert!(!sup.inner().read().await.calibration_busy);
    // normal release
    sup.inner().write().await.calibration_busy = true;
    calibration::BusyGuard::new(sup.clone()).release().await;
    assert!(!sup.inner().read().await.calibration_busy);
}

// ---- calibration.json rejected at startup (review L4) ----

#[tokio::test]
async fn rejected_calibration_resets_the_grasp_camera_model() {
    let dir = tempfile::tempdir().unwrap();
    // grasp_setup.json as an apply wrote it (calibrated camera model), calibration.json corrupt
    let setup = dir.path().join("grasp_setup.json");
    std::fs::write(&setup, r#"{"camera_tilt_deg": -30.0, "fx_px": 831.5, "fy_px": 829.0, "cx_px": 318.2, "cy_px": 183.4,
        "image_size_px": [640, 360], "table_z_m": 0.012, "grip_pixel_norm": [0.41, 0.77]}"#).unwrap();
    let cal = dir.path().join("calibration.json");
    std::fs::write(&cal, r#"{"camera_mount": {"q_end_cam": [1, 0, 0"#).unwrap();
    let sup = ArmSupervisor::new(ArmConfig::rebot_dm());
    let (prof, prof_mount) = { let i = sup.inner().read().await; (i.grasp_setup().unwrap(), i.config.camera_mount.clone()) };
    grasp::load_into(&sup, &setup).await;
    assert_eq!(sup.inner().read().await.grasp_setup().unwrap().cx_px, Some(318.2));
    calibration::load_into(&sup, &cal, Some(&setup)).await;
    let inner = sup.inner().read().await;
    assert!(!inner.camera_calibrated() && inner.config.camera_mount == prof_mount, "profile mount");
    let g = inner.grasp_setup().unwrap();
    assert_eq!((g.fx_px, g.fy_px, g.cx_px, g.cy_px, g.image_size_px, g.grip_pixel_norm), (prof.fx_px, prof.fy_px, prof.cx_px, prof.cy_px, prof.image_size_px, prof.grip_pixel_norm));
    assert!((g.camera_tilt_deg - prof.camera_tilt_deg).abs() < 1e-6, "tilt {} vs profile {}", g.camera_tilt_deg, prof.camera_tilt_deg);
    assert_eq!(g.table_z_m, 0.012, "the table height is kept");
    let err = inner.calibration_load_error.clone().unwrap();
    assert!(err.contains("calibration.json") && err.contains("reset"), "{err}");
    drop(inner);
    // persisted: the next start sees the same setup; the old file is kept
    let saved: Value = serde_json::from_str(&std::fs::read_to_string(&setup).unwrap()).unwrap();
    assert_eq!(saved["fx_px"].as_f64(), Some(prof.fx_px));
    assert!(saved["cx_px"].is_null() == prof.cx_px.is_none());
    let kept: Vec<_> = std::fs::read_dir(dir.path()).unwrap().flatten().map(|e| e.file_name().to_string_lossy().into_owned()).collect();
    assert!(kept.iter().any(|n| n.starts_with("grasp_setup.json.pre-reset-")), "{kept:?}");
    assert!(kept.iter().any(|n| n.starts_with("calibration.json.bad-")), "{kept:?}");
    // reported in GET /api/v1/calibration
    let deps = CalibrationDeps::new(cal.clone(), Some(setup.clone()), armd::api::proxy::DetectorUpstream::parse("http://127.0.0.1:9").unwrap());
    let st = calibration::status(&sup, &deps).await;
    assert!(st["load_error"].as_str().is_some_and(|e| e.contains("reset")), "{st}");
    assert_eq!(st["calibrated"], false);
    // a fresh start with the saved setup and no calibration.json is consistent and has no error
    let again = ArmSupervisor::new(ArmConfig::rebot_dm());
    grasp::load_into(&again, &setup).await;
    calibration::load_into(&again, &cal, Some(&setup)).await;
    let i2 = again.inner().read().await;
    assert!(i2.calibration_load_error.is_none());
    assert_eq!(i2.grasp_setup().unwrap().fx_px, prof.fx_px);
}

#[tokio::test]
async fn successful_apply_clears_the_load_error() {
    let mut r = rig().await;
    r.sup.inner().write().await.calibration_load_error = Some("parse calibration.json: bad".into());
    let ok = cmd(&mut r, "/api/v1/calibration/apply", apply_body(false)).await;
    assert_eq!(ok["type"], "command_ack", "{ok}");
    assert!(r.sup.inner().read().await.calibration_load_error.is_none());
}

// ---- kinematic fine calibration (calibration.json `kinematics`) ----

/// A correction inside the bounds on J2..J5 (J1 / J6 zeros, as the wizard writes it).
fn kin_joints() -> Value {
    let z = json!({"offset_rad": 0.0, "scale": 0.0, "tilt_rad": [0.0, 0.0], "origin_m": [0.0, 0.0, 0.0]});
    json!([z,
        {"offset_rad": 0.021, "scale": 0.012, "tilt_rad": [0.044, -0.014], "origin_m": [0.004, -0.006, 0.002]},
        {"offset_rad": -0.035, "scale": -0.02, "tilt_rad": [0.009, 0.019], "origin_m": [-0.012, 0.003, 0.008]},
        {"offset_rad": 0.054, "scale": 0.03, "tilt_rad": [-0.026, 0.012], "origin_m": [0.009, 0.011, -0.004]},
        {"offset_rad": -0.0698, "scale": -0.05, "tilt_rad": [0.05, -0.05], "origin_m": [-0.02, 0.015, 0.025]},
        z])
}

fn kin_value() -> Value { json!({"joints": kin_joints(), "fit": {"heldout_mm": [2.0, 2.2], "train_mm": 1.7, "views": 140, "solved_at": "2026-11-18T10:00:00Z"}}) }

fn kin_correction() -> k::Correction { serde_json::from_value(kin_joints()).unwrap() }

fn with_kin(mut body: Value, kin: Value) -> Value { body["kinematics"] = kin; body }

const Q_TEST: [f64; 6] = [0.3, -0.9, 1.2, 0.4, -0.5, 1.6];

/// The live arm model is the corrected one (`Some`) or the URDF chain (`None`).
async fn assert_model(r: &Rig, want: Option<k::Correction>) {
    let inner = r.sup.inner().read().await;
    assert_eq!(inner.config.arm_model.correction().copied(), want);
    let off = k::B601DM_MODEL_OFFSETS_RAD;
    let qm: [f64; 6] = std::array::from_fn(|i| Q_TEST[i] + off[i]);
    assert_eq!(inner.config.arm_model.fk(&Q_TEST), k::fk_corrected(&qm, want.as_ref()), "FK of the live model");
    drop(inner);
    assert_eq!(r.sup.get_schema().await.kinematics_correction, want, "schema");
}

#[tokio::test]
async fn apply_with_kinematics_swaps_the_model_live_and_restore_puts_it_back() {
    let mut r = rig().await;
    assert_model(&r, None).await;
    let rev0 = r.sup.inner().read().await.workspace_revision;
    // the corrected FK differs from the URDF chain by millimetres
    let moved = k::norm(&k::sub(&ArmModel::REBOT_DEV_ARM.with_correction(Some(&kin_correction())).unwrap().fk(&Q_TEST).p, &ArmModel::REBOT_DEV_ARM.fk(&Q_TEST).p));
    assert!(moved > 0.005, "{moved}");
    // 1. apply with kinematics
    let a = cmd(&mut r, "/api/v1/calibration/apply", with_kin(apply_body(false), kin_value())).await;
    assert_eq!(a["type"], "command_ack", "{a}");
    assert_eq!(a["kinematics_correction"], kin_joints(), "{a}");
    assert_eq!(a["calibration"]["kinematics"]["fit"]["views"], json!(140));
    assert_model(&r, Some(kin_correction())).await;
    assert!(r.sup.inner().read().await.workspace_revision > rev0, "plans made with the old model go stale");
    let file: CalibrationFile = serde_json::from_str(&std::fs::read_to_string(&r.deps.path).unwrap()).unwrap();
    assert_eq!(file.kinematics.as_ref().unwrap().joints, kin_correction());
    let v: Value = reqwest::get(format!("http://{}/api/v1/calibration", r.addr)).await.unwrap().json().await.unwrap();
    assert_eq!(v["kinematics_correction"], kin_joints(), "{v}");
    assert_eq!(v["calibration"]["kinematics"]["joints"], kin_joints());
    let schema: Value = reqwest::get(format!("http://{}/api/v1/observation/schema", r.addr)).await.unwrap().json().await.unwrap();
    assert_eq!(schema["kinematics_correction"], kin_joints(), "{schema}");
    assert_eq!(schema["capabilities"]["calibration_kinematics"], json!(true));
    let backup0 = a["backup"].as_str().unwrap().rsplit('/').next().unwrap().to_string();
    // 2. an apply without kinematics keeps the stored correction
    let b = cmd(&mut r, "/api/v1/calibration/apply", apply_body(false)).await;
    assert_eq!(b["type"], "command_ack", "{b}");
    assert_model(&r, Some(kin_correction())).await;
    let file: CalibrationFile = serde_json::from_str(&std::fs::read_to_string(&r.deps.path).unwrap()).unwrap();
    assert!(file.kinematics.is_some(), "kept in calibration.json");
    let backup1 = b["backup"].as_str().unwrap().rsplit('/').next().unwrap().to_string();
    // 3. kinematics null removes it
    let c = cmd(&mut r, "/api/v1/calibration/apply", with_kin(apply_body(false), Value::Null)).await;
    assert_eq!(c["type"], "command_ack", "{c}");
    assert_model(&r, None).await;
    assert_eq!(c["kinematics_correction"], Value::Null);
    let file: CalibrationFile = serde_json::from_str(&std::fs::read_to_string(&r.deps.path).unwrap()).unwrap();
    assert!(file.kinematics.is_none());
    assert!(!std::fs::read_to_string(&r.deps.path).unwrap().contains("kinematics"));
    // 4. backup1 holds the state before apply 2 (after apply 1): the correction comes back
    let back = cmd(&mut r, "/api/v1/calibration/restore", json!({"backup": backup1})).await;
    assert_eq!(back["type"], "command_ack", "{back}");
    assert_model(&r, Some(kin_correction())).await;
    // 5. restore the state before any calibration: no correction
    let back = cmd(&mut r, "/api/v1/calibration/restore", json!({"backup": backup0})).await;
    assert_eq!(back["type"], "command_ack", "{back}");
    assert_model(&r, None).await;
    assert!(!r.deps.path.exists());
}

#[tokio::test]
async fn apply_kinematics_dry_run_reports_before_and_after() {
    let mut r = rig().await;
    r.sup.inner().write().await.is_torqued = true; // a dry run writes nothing: allowed with torque on
    let d = cmd(&mut r, "/api/v1/calibration/apply", with_kin(apply_body(true), kin_value())).await;
    assert_eq!(d["type"], "command_ack", "{d}");
    assert_eq!(d["before"]["kinematics"], Value::Null, "{d}");
    assert_eq!(d["after"]["kinematics"]["joints"], kin_joints(), "{d}");
    assert_eq!(d["after"]["kinematics"]["fit"]["heldout_mm"], json!([2.0, 2.2]));
    assert_model(&r, None).await;
    assert!(!r.deps.path.exists() && backups(&r) == 0);
}

#[tokio::test]
async fn kinematics_apply_and_restore_are_refused_while_torqued_or_acting() {
    let mut r = rig().await;
    r.sup.inner().write().await.is_torqued = true;
    let rej = cmd(&mut r, "/api/v1/calibration/apply", with_kin(apply_body(false), kin_value())).await;
    assert_eq!((rej["type"].as_str(), rej["code"].as_str()), (Some("command_reject"), Some("KINEMATICS_ARMED")), "{rej}");
    let rej = cmd(&mut r, "/api/v1/calibration/apply", with_kin(apply_body(false), Value::Null)).await;
    assert_eq!(rej["code"], "KINEMATICS_ARMED", "null removes a correction: also a change: {rej}");
    assert_eq!(backups(&r), 0);
    assert_unchanged(&r).await;
    assert_model(&r, None).await;
    // without kinematics the camera calibration still applies with torque on
    let ok = cmd(&mut r, "/api/v1/calibration/apply", apply_body(false)).await;
    assert_eq!(ok["type"], "command_ack", "{ok}");
    let backup = ok["backup"].as_str().unwrap().rsplit('/').next().unwrap().to_string();
    // a running action
    r.sup.inner().write().await.is_torqued = false;
    let id = r.sup.inner().write().await.begin_action("joint_move", None);
    let rej = cmd(&mut r, "/api/v1/calibration/apply", with_kin(apply_body(false), kin_value())).await;
    assert_eq!(rej["code"], "KINEMATICS_ARMED", "{rej}");
    assert!(rej["message"].as_str().unwrap().contains("action"), "{rej}");
    r.sup.inner().write().await.finish_action(id, "done", None, None, None, None, None);
    let ok = cmd(&mut r, "/api/v1/calibration/apply", with_kin(apply_body(false), kin_value())).await;
    assert_eq!(ok["type"], "command_ack", "{ok}");
    assert_model(&r, Some(kin_correction())).await;
    // restoring a backup with another correction is a change too
    r.sup.inner().write().await.is_torqued = true;
    let rej = cmd(&mut r, "/api/v1/calibration/restore", json!({"backup": backup})).await;
    assert_eq!(rej["code"], "KINEMATICS_ARMED", "{rej}");
    assert_model(&r, Some(kin_correction())).await;
    r.sup.inner().write().await.is_torqued = false;
    let back = cmd(&mut r, "/api/v1/calibration/restore", json!({"backup": backup})).await;
    assert_eq!(back["type"], "command_ack", "{back}");
    assert_model(&r, None).await;
}

#[tokio::test]
async fn kinematics_turned_on_during_the_transaction_is_reverted() {
    // torque comes on while rebot-vision is written: the calibration_file step refuses and reverts
    let mut r = rig().await;
    r.mock.lock().unwrap().put_delay_ms = 500;
    let id = send_cmd(&mut r, "/api/v1/calibration/apply", with_kin(apply_body(false), kin_value())).await;
    tokio::time::sleep(Duration::from_millis(150)).await;
    r.sup.inner().write().await.is_torqued = true;
    let rej = reply_for(&mut r, id).await;
    assert_eq!((rej["code"].as_str(), rej["step"].as_str(), rej["reverted"].as_bool()), (Some("KINEMATICS_ARMED"), Some("calibration_file"), Some(true)), "{rej}");
    assert_eq!(mock_fx(&r), (824.0, 2), "vision PUT reverted");
    assert!(!r.deps.path.exists());
    assert_unchanged(&r).await;
    assert_model(&r, None).await;
}

#[tokio::test]
async fn kinematics_bounds_and_shape_are_checked() {
    let mut r = rig().await;
    let joints = |j: usize, patch: Value| { let mut a = kin_joints(); for (k, v) in patch.as_object().unwrap() { a[j][k] = v.clone(); } json!({"joints": a}) };
    for (kin, what) in [
        (joints(4, json!({"offset_rad": 0.1746})), "offset_rad"),   // 10.004 deg
        (joints(2, json!({"scale": 0.0801})), "scale"),
        (joints(3, json!({"tilt_rad": [0.0, -0.0524]})), "tilt_rad"), // 3.002 deg
        (joints(1, json!({"origin_m": [0.0, 0.031, 0.0]})), "origin_m"),
        (joints(1, json!({"origin_m": [0.0, 0.0]})), "kinematics"),
        (joints(1, json!({"offset": 0.01})), "kinematics"),
        (json!({"joints": [{}, {}, {}, {}, {}]}), "kinematics"),
        (json!({"joints": kin_joints(), "fit": 3}), "fit"),
        (json!({"joints": kin_joints(), "extra": 1}), "kinematics"),
        (json!([1, 2]), "kinematics"),
    ] {
        let rej = cmd(&mut r, "/api/v1/calibration/apply", with_kin(apply_body(true), kin.clone())).await;
        assert_eq!(rej["code"], "INVALID_COMMAND", "{kin}: {rej}");
        assert!(rej["message"].as_str().unwrap().contains(what), "{what}: {rej}");
    }
    // the bounds themselves pass
    let edge = joints(4, json!({"offset_rad": 10f64.to_radians(), "scale": -0.08, "tilt_rad": [3f64.to_radians(), -3f64.to_radians()], "origin_m": [0.03, -0.03, 0.03]}));
    let ok = cmd(&mut r, "/api/v1/calibration/apply", with_kin(apply_body(true), edge)).await;
    assert_eq!(ok["type"], "command_ack", "{ok}");
}

#[tokio::test]
async fn startup_loads_the_correction_and_rejects_a_bad_one() {
    let dir = tempfile::tempdir().unwrap();
    let cal = dir.path().join("calibration.json");
    let mut file = calib_file(v8_q());
    file.kinematics = Some(serde_json::from_value(kin_value()).unwrap());
    std::fs::write(&cal, serde_json::to_string(&file).unwrap()).unwrap();
    let sup = ArmSupervisor::new(ArmConfig::rebot_dm());
    calibration::load_into(&sup, &cal, None).await;
    {
        let inner = sup.inner().read().await;
        assert_eq!(inner.config.arm_model.correction().copied(), Some(kin_correction()));
        assert!(inner.calibration_load_error.is_none() && inner.camera_calibrated());
    }
    assert_eq!(sup.get_schema().await.kinematics_correction, Some(kin_correction()));
    // outside the bounds: calibration load_error (L4 path), the uncorrected model is used
    let mut bad: Value = serde_json::to_value(&file).unwrap();
    bad["kinematics"]["joints"][4]["offset_rad"] = json!(0.5);
    std::fs::write(&cal, bad.to_string()).unwrap();
    let sup = ArmSupervisor::new(ArmConfig::rebot_dm());
    calibration::load_into(&sup, &cal, None).await;
    let inner = sup.inner().read().await;
    assert!(inner.config.arm_model.correction().is_none());
    let err = inner.calibration_load_error.clone().unwrap();
    assert!(err.contains("kinematics") && err.contains("offset_rad"), "{err}");
    assert!(!cal.exists(), "quarantined");
    drop(inner);
    // malformed (5 joints)
    let mut bad: Value = serde_json::to_value(&file).unwrap();
    bad["kinematics"]["joints"].as_array_mut().unwrap().pop();
    std::fs::write(&cal, bad.to_string()).unwrap();
    let sup = ArmSupervisor::new(ArmConfig::rebot_dm());
    calibration::load_into(&sup, &cal, None).await;
    let inner = sup.inner().read().await;
    assert!(inner.config.arm_model.correction().is_none() && inner.calibration_load_error.is_some());
    // the default schema publishes null
    drop(inner);
    let v = serde_json::to_value(sup.get_schema().await).unwrap();
    assert_eq!(v.get("kinematics_correction"), Some(&Value::Null));
}

fn so101() -> ArmConfig {
    let mut cfg: ArmConfig = serde_json::from_str(include_str!("../profiles/so_arm101.json")).unwrap();
    cfg.init_arm_model_in_memory(Some(include_str!("../assets/so101_new_calib.urdf"))).unwrap();
    cfg
}

#[tokio::test]
async fn urdf_chain_profiles_refuse_kinematics() {
    let mut r = rig_cfg(so101(), |d| (d.join("calibration.json"), d.join("grasp_setup.json"))).await;
    assert!(!r.sup.get_schema().await.capabilities.calibration_kinematics);
    for dry in [true, false] {
        let rej = cmd(&mut r, "/api/v1/calibration/apply", with_kin(apply_body(dry), kin_value())).await;
        assert_eq!(rej["code"], "UNSUPPORTED", "{rej}");
        assert!(rej["message"].as_str().unwrap().contains("URDF"), "{rej}");
    }
    assert!(r.sup.inner().read().await.config.arm_model.correction().is_none());
    // startup: the camera calibration loads, the correction is ignored with a load_error
    let dir = tempfile::tempdir().unwrap();
    let cal = dir.path().join("calibration.json");
    let mut file = calib_file(k::r_to_quat(&so101().arm_model.cam_mount(false, 0.0, -20.0)));
    file.kinematics = Some(serde_json::from_value(kin_value()).unwrap());
    std::fs::write(&cal, serde_json::to_string(&file).unwrap()).unwrap();
    let sup = ArmSupervisor::new(so101());
    calibration::load_into(&sup, &cal, None).await;
    let inner = sup.inner().read().await;
    assert!(inner.camera_calibrated() && inner.config.arm_model.correction().is_none());
    assert!(inner.calibration.as_ref().unwrap().kinematics.is_none());
    assert!(inner.calibration_load_error.as_ref().is_some_and(|e| e.contains("URDF")), "{:?}", inner.calibration_load_error);
}
