//! Motor zero calibration (`/api/v1/calibration/motor_zero`, src/motor_zero.rs): checks,
//! dry run, the run on the mock and sim followers, the log and the invalidation of the
//! kinematic correction and the camera calibration.
mod common;

use std::net::UdpSocket;
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use armd::calibration::{self, CalibrationDeps, CalibrationFile, KinematicsCalibration};
use armd::chain::ArmModel;
use armd::kinematics as k;
use armd::model::CameraMount;
use armd::{ArmConfig, ArmSupervisor};
use serde_json::{json, Value};

const PATH: &str = "/api/v1/calibration/motor_zero";

struct Rig {
    sup: Arc<ArmSupervisor>,
    ws: common::TestSocket,
    generation: u64,
    epoch: u64,
    deps: CalibrationDeps,
    dir: tempfile::TempDir,
    stop: Arc<AtomicBool>,
}

impl Drop for Rig {
    fn drop(&mut self) { self.stop.store(true, Ordering::SeqCst); }
}

/// armd with a lease holder and a control tick; `driver` None = mock follower.
async fn rig(driver: Option<Box<dyn armd::driver::ArmDriver>>, setup: impl FnOnce(&Path)) -> Rig {
    let dir = tempfile::tempdir().unwrap();
    setup(dir.path());
    let up = armd::api::proxy::DetectorUpstream::parse("http://127.0.0.1:9").unwrap();
    let mut deps = CalibrationDeps::new(dir.path().join("calibration.json"), Some(dir.path().join("grasp_setup.json")), up.clone());
    deps.backup_dir = dir.path().join("backups");
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    if let Some(d) = driver { sup.set_driver(d).await; }
    calibration::load_into(&sup, &deps.path, None).await.unwrap();
    let router = armd::api::create_router_calibration(sup.clone(), up.clone(), armd::grasp::GraspDeps::http(up, None),
        armd::diagnostics::DiagnosticsState::global(), deps.clone(), armd::auth::OperatorAuth::disabled());
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });
    let stop = Arc::new(AtomicBool::new(false));
    let (tsup, tstop) = (sup.clone(), stop.clone());
    tokio::spawn(async move {
        while !tstop.load(Ordering::SeqCst) {
            tsup.tick(0.02).await;
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    });
    let (ws, generation) = common::controller(addr).await;
    let epoch = sup.get_observation().await.epoch_id;
    Rig { sup, ws, generation, epoch, deps, dir, stop }
}

static REQ: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(2_000_000);

async fn cmd(r: &mut Rig, path: &str, body: Value) -> Value {
    let id = REQ.fetch_add(1, Ordering::Relaxed);
    common::send(&mut r.ws, json!({"type":"command", "request_id":id, "generation":r.generation, "epoch_id":r.epoch, "path":path, "body":body})).await;
    let deadline = tokio::time::Instant::now() + Duration::from_secs(10);
    loop {
        assert!(tokio::time::Instant::now() < deadline, "no reply to {id}");
        match tokio::time::timeout(Duration::from_millis(300), common::next_of(&mut r.ws, &["command_ack", "command_reject"])).await {
            Ok(v) if v["request_id"] == json!(id) => return v,
            Ok(_) => {}
            Err(_) => common::send(&mut r.ws, json!({"type":"heartbeat", "generation":r.generation, "epoch_id":r.epoch})).await,
        }
    }
}

async fn set_q(r: &Rig, q: [f32; 7]) {
    let mut inner = r.sup.inner().write().await;
    inner.measured_positions = q.to_vec();
    inner.commanded_positions = q.to_vec();
    inner.target_positions = q.to_vec();
}

const Q: [f32; 7] = [0.05, -0.12, 0.3, 0.0, -0.02, 1.5707964, 0.01];

#[tokio::test]
async fn dry_run_reports_readings_and_changes_nothing() {
    let mut r = rig(None, |_| {}).await;
    set_q(&r, Q).await;
    let reply = cmd(&mut r, PATH, json!({"joints": [2, 3, 6], "dry_run": true})).await;
    assert_eq!(reply["type"], "command_ack", "{reply}");
    assert_eq!(reply["dry_run"], json!(true));
    let rows = reply["joints"].as_array().unwrap();
    assert_eq!(rows.len(), 3);
    assert_eq!(rows[0]["joint"], json!(2));
    assert!((rows[0]["change_rad"].as_f64().unwrap() - 0.12).abs() < 1e-6, "{reply}");
    // J6 at +90 deg is far from 0: reported, the dry run is not refused
    assert_eq!(rows[2]["far"], json!(true));
    assert_eq!(reply["needs_force"], json!(true));
    assert_eq!(reply["far_joints"], json!([6]));
    assert_eq!(reply["persist"], json!("process"));
    assert_eq!(r.sup.inner().read().await.measured_positions, Q.to_vec(), "nothing changed");
    assert!(!r.deps.backup_dir.join("motor_zero.jsonl").exists());
}

#[tokio::test]
async fn refused_while_torqued_in_fault_or_with_bad_joints() {
    let mut r = rig(None, |_| {}).await;
    set_q(&r, Q).await;
    let rej = cmd(&mut r, PATH, json!({"joints": [8]})).await;
    assert_eq!(rej["code"], "JOINT_UNSUPPORTED", "{rej}");
    for body in [json!({"joints": []}), json!({"joints": [1, 1]}), json!({"joints": [0]}), json!({"joints": [1], "extra": 1}), json!({"joints": [1], "force": "yes"})] {
        assert_eq!(cmd(&mut r, PATH, body.clone()).await["code"], "INVALID_COMMAND", "{body}");
    }
    r.sup.inner().write().await.is_torqued = true;
    for dry in [true, false] {
        let rej = cmd(&mut r, PATH, json!({"joints": [2], "dry_run": dry})).await;
        assert_eq!(rej["code"], "MOTOR_ZERO_ARMED", "{rej}");
    }
    r.sup.inner().write().await.is_torqued = false;
    // e-stop latched: MOTOR_ZERO_FAULT
    r.sup.trigger_e_stop().await;
    r.epoch = r.sup.get_observation().await.epoch_id;
    let rej = cmd(&mut r, PATH, json!({"joints": [2]})).await;
    assert_eq!(rej["code"], "MOTOR_ZERO_FAULT", "{rej}");
    assert_eq!(r.sup.inner().read().await.measured_positions[1], Q[1]);
}

#[tokio::test]
async fn far_from_zero_needs_force() {
    let mut r = rig(None, |_| {}).await;
    set_q(&r, Q).await;
    let rej = cmd(&mut r, PATH, json!({"joints": [5, 6]})).await;
    assert_eq!(rej["code"], "MOTOR_ZERO_FAR", "{rej}");
    assert!(rej["message"].as_str().unwrap().contains("J6"), "{rej}");
    assert_eq!(r.sup.inner().read().await.measured_positions, Q.to_vec());
    let ok = cmd(&mut r, PATH, json!({"joints": [5, 6], "force": true})).await;
    assert_eq!(ok["type"], "command_ack", "{ok}");
    let q = r.sup.inner().read().await.measured_positions.clone();
    assert_eq!((q[4], q[5]), (0.0, 0.0));
    assert_eq!(q[2], Q[2], "other joints unchanged");
    let log = std::fs::read_to_string(r.deps.backup_dir.join("motor_zero.jsonl")).unwrap();
    let entry: Value = serde_json::from_str(log.lines().last().unwrap()).unwrap();
    assert_eq!(entry["joints"], json!([5, 6]));
    assert_eq!(entry["force"], json!(true));
    assert_eq!(entry["result"], json!("verified"));
    assert!((entry["readings_before"][5].as_f64().unwrap() - f64::from(Q[5])).abs() < 1e-6);
}

fn kin() -> KinematicsCalibration {
    let mut joints = [k::JointCorrection::default(); 6];
    joints[1].offset_rad = 0.01;
    joints[4].tilt_rad = [0.005, -0.004];
    KinematicsCalibration { joints, fit: Some(json!({"heldout_mm": [2.0, 2.1]})) }
}

fn calibrated(dir: &Path) {
    let q = k::r_to_quat(&ArmModel::REBOT_DEV_ARM.cam_mount(false, 0.0, -38.0));
    let file = CalibrationFile {
        camera_mount: CameraMount { offset_m: Some([-0.0731, 0.0005, 0.066]), q_end_cam: Some(q), ..Default::default() },
        intrinsics: Some(json!({"fx": 830.0})), table_z_m: Some(0.01), solved_at: Some("2026-11-17T00:00:00Z".into()), rms: None,
        kinematics: Some(kin()), stale: None, joint_zero: None, ..Default::default()
    };
    std::fs::write(dir.join("calibration.json"), serde_json::to_string(&file).unwrap()).unwrap();
}

#[tokio::test]
async fn zero_removes_the_kinematic_correction_and_marks_the_camera_stale() {
    let mut r = rig(None, calibrated).await;
    {
        let inner = r.sup.inner().read().await;
        assert!(inner.config.arm_model.correction().is_some() && inner.camera_calibrated());
    }
    set_q(&r, Q).await;
    let dry = cmd(&mut r, PATH, json!({"joints": [2], "dry_run": true})).await;
    assert_eq!(dry["invalidates"], json!({"kinematics": true, "camera": true, "poses": [], "gripper_calibration": false}), "{dry}");
    let ok = cmd(&mut r, PATH, json!({"joints": [2, 3]})).await;
    assert_eq!(ok["type"], "command_ack", "{ok}");
    assert_eq!(ok["invalidated"]["kinematics_removed"], json!(true), "{ok}");
    assert_eq!(ok["invalidated"]["camera_stale"], json!(true), "{ok}");
    assert_eq!(ok["invalidated"]["removed_kinematics"]["fit"]["heldout_mm"], json!([2.0, 2.1]), "the removed correction is in the reply and the log");
    assert!(ok["joints"].as_array().unwrap().iter().all(|j| j["verified"] == json!(true)), "{ok}");
    // live: uncorrected model, camera calibration kept (grasp works) but stale
    let inner = r.sup.inner().read().await;
    assert!(inner.config.arm_model.correction().is_none());
    assert!(inner.camera_calibrated());
    assert!(!inner.is_torqued, "torque stays off");
    assert!(!inner.calibration_busy && !inner.motor_zero_busy);
    drop(inner);
    // file: no kinematics, stale set
    let file: CalibrationFile = serde_json::from_str(&std::fs::read_to_string(&r.deps.path).unwrap()).unwrap();
    assert!(file.kinematics.is_none());
    assert!(file.stale.as_ref().unwrap().reason.contains("motor zero"));
    // GET /api/v1/calibration shows stale; the schema has no correction
    let status = calibration::status(&r.sup, &r.deps).await;
    assert!(status["stale"]["at"].is_string(), "{status}");
    assert!(status["kinematics_correction"].is_null());
    // a restart keeps it stale
    let fresh = ArmSupervisor::new(ArmConfig::rebot_dm());
    calibration::load_into(&fresh, &r.deps.path, None).await.unwrap();
    assert!(fresh.inner().read().await.calibration.as_ref().unwrap().stale.is_some());
    let log = std::fs::read_to_string(r.deps.backup_dir.join("motor_zero.jsonl")).unwrap();
    assert!(log.contains("\"kinematics_removed\":true"), "{log}");
    let _ = &r.dir;
}

#[tokio::test]
async fn motion_and_torque_are_refused_while_a_zero_runs() {
    let mut r = rig(None, |_| {}).await;
    set_q(&r, Q).await;
    // a run takes >= 100 ms per extra joint: send it and a torque command right after
    let id = REQ.fetch_add(1, Ordering::Relaxed);
    common::send(&mut r.ws, json!({"type":"command", "request_id":id, "generation":r.generation, "epoch_id":r.epoch, "path":PATH,
        "body":{"joints":[1, 2, 3, 4, 5]}})).await;
    tokio::time::sleep(Duration::from_millis(30)).await;
    assert!(r.sup.inner().read().await.motor_zero_busy);
    for (path, body) in [("/api/v1/torque", json!({"enable": true})), ("/api/v1/motion/home", json!({})), (PATH, json!({"joints": [6], "force": true}))] {
        let rej = cmd(&mut r, path, body).await;
        assert_eq!(rej["type"], "command_reject", "{path}: {rej}");
        assert!(matches!(rej["code"].as_str(), Some("MOTION_BUSY" | "CONFIG_BUSY")), "{path}: {rej}");
    }
    let deadline = tokio::time::Instant::now() + Duration::from_secs(5);
    while r.sup.inner().read().await.motor_zero_busy {
        assert!(tokio::time::Instant::now() < deadline);
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    assert_eq!(&r.sup.inner().read().await.measured_positions[..5], &[0.0; 5]);
}

// ---- sim follower ----

/// Fake physics simulator: reports fixed raw joint angles and acknowledges enable commands.
fn spawn_simulator(q: Vec<f32>, stop: Arc<AtomicBool>) -> String {
    let socket = UdpSocket::bind("127.0.0.1:0").unwrap();
    socket.set_read_timeout(Some(Duration::from_millis(5))).unwrap();
    let addr = format!("udp://{}", socket.local_addr().unwrap());
    std::thread::spawn(move || {
        let (mut peer, mut ack, mut enabled, mut seq) = (None, 0u64, false, 0u64);
        let mut buf = [0u8; 8192];
        while !stop.load(Ordering::SeqCst) {
            if let Ok((n, from)) = socket.recv_from(&mut buf) {
                peer = Some(from);
                if let Ok(c) = serde_json::from_slice::<Value>(&buf[..n]) {
                    if c["type"] == "cmd" { ack = c["seq"].as_u64().unwrap_or(ack); enabled = c["enable"].as_bool().unwrap_or(false); }
                }
            }
            if let Some(p) = peer {
                seq += 1;
                let n = q.len();
                let s = json!({"v":1, "type":"state", "session":1, "seq":seq, "ack":ack, "enabled":enabled, "q":q, "dq":vec![0.0; n],
                    "tau":vec![0.0; n], "temp":vec![30.0; n], "t":0.0});
                let _ = socket.send_to(s.to_string().as_bytes(), p);
            }
        }
    });
    addr
}

#[tokio::test]
async fn sim_follower_sets_the_zero_and_reads_it_back() {
    let stop = Arc::new(AtomicBool::new(false));
    let raw = vec![0.2f32, -0.3, 0.4, 0.05, -0.1, 0.25, 0.02];
    let addr = spawn_simulator(raw.clone(), stop.clone());
    let drv = armd::sim_driver::SimArmDriver::connect(&addr, 7).unwrap();
    let mut r = rig(Some(Box::new(drv)), |_| {}).await;
    // feedback flowing
    let deadline = tokio::time::Instant::now() + Duration::from_secs(3);
    while (r.sup.inner().read().await.measured_positions[0] - 0.2).abs() > 1e-6 {
        assert!(tokio::time::Instant::now() < deadline, "no sim feedback");
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    let dry = cmd(&mut r, PATH, json!({"joints": [2, 3], "dry_run": true})).await;
    assert_eq!(dry["persist"], json!("simulator"), "{dry}");
    assert!((dry["joints"][0]["reading_rad"].as_f64().unwrap() + 0.3).abs() < 1e-6);
    let ok = cmd(&mut r, PATH, json!({"joints": [2, 3]})).await;
    assert_eq!(ok["type"], "command_ack", "{ok}");
    for row in ok["joints"].as_array().unwrap() {
        assert_eq!(row["verified"], json!(true), "{ok}");
        assert_eq!(row["fresh"], json!(true), "{ok}");
        assert!(row["after_rad"].as_f64().unwrap().abs() < 1e-6, "{ok}");
    }
    tokio::time::sleep(Duration::from_millis(100)).await;
    let q = r.sup.inner().read().await.measured_positions.clone();
    assert!(q[1].abs() < 1e-6 && q[2].abs() < 1e-6, "{q:?}");
    assert!((q[0] - 0.2).abs() < 1e-6 && (q[3] - 0.05).abs() < 1e-6, "others keep their reading: {q:?}");
    assert!(!r.sup.inner().read().await.is_torqued);
    stop.store(true, Ordering::SeqCst);
}

#[tokio::test]
async fn unsupported_driver_is_refused_with_the_reason() {
    let rs = armd::driver::RobStrideArmDriver::new(armd::transport::MockCanTransport::new(), (1..=7).collect());
    let mut r = rig(Some(Box::new(rs)), |_| {}).await;
    let rej = cmd(&mut r, PATH, json!({"joints": [1], "dry_run": true, "method": "motor"})).await;
    assert_eq!(rej["code"], "UNSUPPORTED", "{rej}");
    assert!(rej["message"].as_str().unwrap().contains("RobStride"), "{rej}");
    // without a method the software zero is used (the profile has a zero_reference); the mock CAN
    // bus gives no feedback
    let rej = cmd(&mut r, PATH, json!({"joints": [1], "dry_run": true})).await;
    assert_eq!(rej["code"], "MOTOR_ZERO_NO_FEEDBACK", "{rej}");
}
