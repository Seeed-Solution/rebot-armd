//! Joint zero (src/joint_zero.rs): the driver boundary (offsets 0 pass values through unchanged,
//! offsets shift readings, commands and limits consistently), the software zero capture and its
//! restore, the legacy J6 detection, and what a zero marks stale (poses, gripper calibration).
mod common;

use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use armd::calibration::{self, CalibrationDeps};
use armd::driver::{ArmDriver, ArmDriverError, JointMode, JointTarget};
use armd::joint_zero::ZeroOffsetDriver;
use armd::kinematics as k;
use armd::model::JointState;
use armd::{ArmConfig, ArmSupervisor};
use serde_json::{json, Value};

const PATH: &str = "/api/v1/calibration/motor_zero";
const RESTORE: &str = "/api/v1/calibration/joint_zero/restore";
const H: f32 = std::f32::consts::FRAC_PI_2;
/// The office unit before its zero (legacy J6), in the rest pose.
const LEGACY_REST: [f32; 7] = [0.049, 0.011, 0.027, 0.024, 0.001, 1.574, 0.052];

/// Records what reaches the driver.
#[derive(Default)]
struct Rec { reads: Vec<f32>, written: Vec<Vec<f32>>, held: Vec<Vec<f32>> }

struct RecDriver(Arc<Mutex<Rec>>);

impl ArmDriver for RecDriver {
    fn position_limits(&self, j: usize) -> Option<(f32, f32)> { (j == 5).then_some((-2.0, 2.0)) }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        Ok(self.0.lock().unwrap().reads.iter().map(|p| JointState { position: *p, velocity: 0.25, torque: -0.5, temperature: 30.0 }).collect())
    }
    fn write_joint_targets(&mut self, t: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.0.lock().unwrap().written.push(t.iter().map(|x| x.position).collect());
        Ok(())
    }
    fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
    fn set_joint_modes(&mut self, _: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.0.lock().unwrap().held.push(hold.iter().map(|x| x.position).collect());
        Ok(())
    }
}

#[test]
fn zero_offsets_pass_every_value_through_bit_for_bit() {
    let rec = Arc::new(Mutex::new(Rec { reads: vec![-0.0, 0.1, f32::MIN_POSITIVE, -3.0, 1e-30, H, 0.02], ..Default::default() }));
    let mut d = ZeroOffsetDriver::new(Box::new(RecDriver(rec.clone())), &[0.0; 7]);
    let got: Vec<u32> = d.read_joint_states().unwrap().iter().map(|s| s.position.to_bits()).collect();
    assert_eq!(got, rec.lock().unwrap().reads.iter().map(|p| p.to_bits()).collect::<Vec<_>>());
    let t: Vec<JointTarget> = [-0.0f32, 0.3, -1.2, 0.0, 2.0, H, -5.0].iter().map(|p| JointTarget::position(*p)).collect();
    d.write_joint_targets(&t).unwrap();
    d.set_joint_modes(&[JointMode::Mit; 7], &t).unwrap();
    let r = rec.lock().unwrap();
    for sent in [&r.written[0], &r.held[0]] {
        assert_eq!(sent.iter().map(|p| p.to_bits()).collect::<Vec<_>>(), t.iter().map(|x| x.position.to_bits()).collect::<Vec<_>>());
    }
    assert_eq!(d.position_limits(5), Some((-2.0, 2.0)));
}

#[test]
fn offsets_shift_readings_commands_and_limits() {
    let rec = Arc::new(Mutex::new(Rec { reads: vec![0.0, 0.0, 0.0, 0.0, 0.0, H, 0.0], ..Default::default() }));
    let mut d = ZeroOffsetDriver::new(Box::new(RecDriver(rec.clone())), &[0.0, 0.0, 0.0, 0.0, 0.0, H, 0.0]);
    let s = d.read_joint_states().unwrap();
    assert_eq!(s[5].position, 0.0, "motor pi/2 = joint 0");
    assert_eq!((s[5].velocity, s[5].torque), (0.25, -0.5), "velocity and torque unchanged");
    d.write_joint_targets(&[0.0f32, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0].map(JointTarget::position)).unwrap();
    assert_eq!(rec.lock().unwrap().written[0][5], 0.1 + H);
    assert_eq!(d.position_limits(5), Some((-2.0 - H, 2.0 - H)), "driver limits in joint space");
    d.set_joint_zero_offsets(&[0.0; 7]);
    assert_eq!(d.read_joint_states().unwrap()[5].position, H);
}

/// A legacy unit (J6 reads pi/2 in the rest pose) with joint_zero J6 = pi/2: the supervisor sees
/// the rest pose as joint 0 (FK = URDF zero), a joint target reaches the motor with the offset and
/// a park to home (all 0) keeps the motor at pi/2 (no wrist roll).
#[tokio::test]
async fn nonzero_offsets_shift_park_limits_and_fk_consistently() {
    let arm = common::TestArm::new();
    arm.0.lock().unwrap().q = [0.0, 0.0, 0.0, 0.0, 0.0, H, 0.0];
    let sup = ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(arm.clone()));
    {
        let mut inner = sup.inner().write().await;
        inner.set_joint_zero_offsets(&[0.0, 0.0, 0.0, 0.0, 0.0, H, 0.0]).unwrap();
    }
    sup.tick(0.02).await;
    let obs = sup.get_observation().await;
    assert_eq!(obs.positions[5], 0.0, "{:?}", obs.positions);
    let q6: [f64; 6] = std::array::from_fn(|i| f64::from(obs.positions[i]));
    let fk = ArmConfig::rebot_dm().arm_model.fk(&q6);
    assert_eq!(fk.p, k::fk(&[0.0; 6]).p, "FK of the calibrated reading = the URDF rest pose");
    // park target = home (joint 0) = motor pi/2 on J6
    assert_eq!(sup.get_config().await.home_positions, vec![0.0; 7]);
    sup.set_torque(true).await.unwrap();
    let epoch = sup.inner().read().await.safety.epoch_id();
    let mut target = vec![0.0f32; 7];
    target[0] = 0.05;
    for _ in 0..60 {
        let _ = sup.servo(target.clone(), epoch).await;
        sup.tick(0.02).await;
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
    let q = arm.0.lock().unwrap().q;
    assert!((q[0] - 0.05).abs() < 1e-3, "J1 reached: {q:?}");
    assert!((q[5] - H).abs() < 1e-5, "J6 motor stays at pi/2 for joint 0: {q:?}");
}

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

async fn rig(setup: impl FnOnce(&Path)) -> Rig {
    let dir = tempfile::tempdir().unwrap();
    setup(dir.path());
    let up = armd::api::proxy::DetectorUpstream::parse("http://127.0.0.1:9").unwrap();
    let mut deps = CalibrationDeps::new(dir.path().join("calibration.json"), Some(dir.path().join("grasp_setup.json")), up.clone());
    deps.backup_dir = dir.path().join("backups");
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    {
        let mut inner = sup.inner().write().await;
        inner.poses.path = Some(dir.path().join("poses.json"));
        inner.gripper_calib_path = Some(dir.path().join("gripper_calib.json"));
    }
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

static REQ: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(3_000_000);

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

/// Without a driver every tick mirrors commanded_positions into measured_positions, so a test
/// moves the simulated arm through all three (writing only measured_positions is undone by the
/// next 20 ms tick and races the code under test).
fn put_joint(inner: &mut armd::SupervisorInner, j: usize, v: f32) {
    inner.measured_positions[j] = v;
    inner.commanded_positions[j] = v;
    inner.target_positions[j] = v;
}

async fn set_q(r: &Rig, q: [f32; 7]) {
    let mut inner = r.sup.inner().write().await;
    inner.measured_positions = q.to_vec();
    inner.commanded_positions = q.to_vec();
    inner.target_positions = q.to_vec();
    inner.zero_check_pending = true;
}

async fn status(r: &Rig) -> Value { calibration::status(&r.sup, &r.deps).await }

async fn wait_status(r: &Rig, want: &str) -> Value {
    for _ in 0..50 {
        let s = status(r).await;
        if s["zero_convention"]["status"] == json!(want) { return s; }
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    panic!("zero_convention never became {want}: {}", status(r).await);
}

#[tokio::test]
async fn legacy_j6_is_detected_at_rest_and_never_moves_the_arm() {
    let r = rig(|_| {}).await;
    set_q(&r, LEGACY_REST).await;
    let s = wait_status(&r, "legacy_j6").await;
    assert!(s["zero_convention"]["message"].as_str().unwrap().contains("J6 reads 90.2 deg"), "{s}");
    let schema = serde_json::to_value(r.sup.get_schema().await).unwrap();
    assert_eq!(schema["zero_convention"]["status"], json!("legacy_j6"));
    assert_eq!(schema["zero_reference"]["positions"], json!(vec![0.0; 7]));
    let inner = r.sup.inner().read().await;
    assert_eq!(inner.measured_positions, LEGACY_REST.to_vec(), "never moved");
    assert!(!inner.is_torqued);
    drop(inner);
    // sticky: J6 reading 0 later (e.g. after a park that rolled the wrist) does not clear it
    set_q(&r, [0.0; 7]).await;
    tokio::time::sleep(Duration::from_millis(100)).await;
    assert_eq!(status(&r).await["zero_convention"]["status"], json!("legacy_j6"));
}

#[tokio::test]
async fn software_zero_captures_marks_poses_stale_and_restores() {
    let mut r = rig(|_| {}).await;
    set_q(&r, LEGACY_REST).await;
    wait_status(&r, "legacy_j6").await;
    let saved = cmd(&mut r, "/api/v1/poses/save", json!({"name": "bin_a"})).await;
    assert_eq!(saved["type"], "command_ack", "{saved}");
    // dry run: J6 is 90 deg from the reference
    let dry = cmd(&mut r, PATH, json!({"joints": [6], "method": "software", "dry_run": true})).await;
    assert_eq!(dry["needs_force"], json!(true), "{dry}");
    assert_eq!(dry["zero_method"], json!("software"));
    assert_eq!(dry["invalidates"]["poses"], json!(["bin_a"]), "{dry}");
    let ok = cmd(&mut r, PATH, json!({"joints": [6], "method": "software", "force": true})).await;
    assert_eq!(ok["type"], "command_ack", "{ok}");
    assert!((ok["offsets_after"][5].as_f64().unwrap() - 1.574).abs() < 1e-5, "{ok}");
    assert_eq!(ok["joints"][0]["verified"], json!(true), "{ok}");
    let q = r.sup.inner().read().await.measured_positions.clone();
    assert!(q[5].abs() < 1e-6 && (q[0] - LEGACY_REST[0]).abs() < 1e-7, "J6 reads the reference, others unchanged: {q:?}");
    // calibration.json holds the joint zero; the backup and the stale pose
    let file: Value = serde_json::from_str(&std::fs::read_to_string(&r.deps.path).unwrap()).unwrap();
    assert_eq!(file["joint_zero"]["method"], json!("software"));
    assert!((file["joint_zero"]["joints"][5]["offset_rad"].as_f64().unwrap() - 1.574).abs() < 1e-5);
    let s = wait_status(&r, "ok").await;
    assert_eq!(s["stale_list"][0]["item"], json!("poses"), "{s}");
    assert_eq!(s["stale_list"][0]["names"], json!(["bin_a"]));
    let poses: Value = serde_json::from_str(&std::fs::read_to_string(r.dir.path().join("poses.json")).unwrap()).unwrap();
    assert!(poses["poses"][0]["stale"]["reason"].as_str().unwrap().contains("software zero"), "{poses}");
    assert!(s["calibration"].get("camera_mount").is_some());
    // a restart loads the offsets
    let sup2 = ArmSupervisor::new(ArmConfig::rebot_dm());
    calibration::load_into(&sup2, &r.deps.path, None).await.unwrap();
    assert!((sup2.inner().read().await.joint_zero_offsets[5] - 1.574).abs() < 1e-5);
    // restore the backup: offsets 0 again, J6 reads its motor value
    let backup = ok["restore"]["backup"].as_str().unwrap().to_string();
    let back = cmd(&mut r, RESTORE, json!({"backup": backup})).await;
    assert_eq!(back["type"], "command_ack", "{back}");
    assert_eq!(back["offsets_after"], json!(vec![0.0; 7]));
    let q = r.sup.inner().read().await.measured_positions.clone();
    assert!((q[5] - 1.574).abs() < 1e-5, "{q:?}");
    assert!(!r.deps.path.exists(), "nothing left in calibration.json: removed");
    // re-saving a pose clears its stale mark
    let saved = cmd(&mut r, "/api/v1/poses/save", json!({"name": "bin_a"})).await;
    assert!(saved["pose"]["stale"].is_null(), "{saved}");
}

#[tokio::test]
async fn software_zero_refuses_a_moving_arm() {
    let mut r = rig(|_| {}).await;
    set_q(&r, [0.01; 7]).await;
    let sup = r.sup.clone();
    let mover = tokio::spawn(async move {
        for i in 0..40 {
            put_joint(&mut *sup.inner().write().await, 1, 0.01 + 0.002 * i as f32);
            tokio::time::sleep(Duration::from_millis(15)).await;
        }
    });
    let rej = cmd(&mut r, PATH, json!({"joints": [1, 2], "method": "software"})).await;
    mover.abort();
    assert_eq!(rej["code"], json!("ZERO_MOVING"), "{rej}");
    assert!(!r.deps.path.exists());
}

#[tokio::test]
async fn motor_zero_marks_poses_and_gripper_calibration_stale() {
    let mut r = rig(|_| {}).await;
    set_q(&r, LEGACY_REST).await;
    wait_status(&r, "legacy_j6").await;
    assert_eq!(cmd(&mut r, "/api/v1/poses/save", json!({"name": "scan"})).await["type"], "command_ack");
    {
        let mut inner = r.sup.inner().write().await;
        inner.gripper_calib_offset = Some(0.02);
    }
    let ok = cmd(&mut r, PATH, json!({"joints": [6, 7], "method": "motor", "force": true})).await;
    assert_eq!(ok["type"], "command_ack", "{ok}");
    assert_eq!(ok["invalidated"]["poses_stale"], json!(["scan"]), "{ok}");
    assert_eq!(ok["invalidated"]["gripper_calibration_stale"], json!(true), "{ok}");
    let s = status(&r).await;
    let items: Vec<&str> = s["stale_list"].as_array().unwrap().iter().map(|v| v["item"].as_str().unwrap()).collect();
    assert_eq!(items, ["poses", "gripper_calibration"], "{s}");
    let g: Value = serde_json::from_str(&std::fs::read_to_string(r.dir.path().join("gripper_calib.json")).unwrap()).unwrap();
    assert!(g["stale"]["reason"].as_str().unwrap().contains("motor zero"), "{g}");
    assert!(r.sup.gripper_calibration_response().await.stale.is_some());
    // offsets 0 (the motor reads 0 at the reference): no joint_zero is written
    assert!(!r.deps.path.exists());
    // the motor zero of J6 ended the legacy verdict; the arm reads the rest pose now
    wait_status(&r, "ok").await;
    // a zero of a joint already at its reference (< 0.5 deg) marks nothing more stale
    {
        let mut inner = r.sup.inner().write().await;
        for p in inner.poses.list.poses.iter_mut() { p.stale = None; }
        put_joint(&mut inner, 0, 0.004); // 0.23 deg
    }
    let ok = cmd(&mut r, PATH, json!({"joints": [1], "method": "motor"})).await;
    assert_eq!(ok["type"], "command_ack", "{ok}");
    assert_eq!(ok["invalidated"]["poses_stale"], json!([]), "{ok}");
}
