//! Gripper zero check (`/api/v1/action/gripper_zero_check`) and runtime calibration (`/api/v1/gripper/calibration`).

mod common;

use std::sync::Arc;
use std::time::{Duration, Instant};
use serde_json::json;

use armd::model::ArmConfig;
use armd::supervisor::HeldObject as SupHeldObject;
use armd::{ArmError, ArmSupervisor, SupervisorInner};
use common::INSPECT;

const DT: f64 = 0.02;

fn sim_at(q: &[f32; 7]) -> SupervisorInner {
    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
    inner.lease_enabled = false;
    inner.set_torque(true).unwrap();
    inner.commanded_positions = q.to_vec();
    inner.measured_positions = q.to_vec();
    inner.target_positions = q.to_vec();
    inner
}

fn run_ticks(inner: &mut SupervisorInner, id: u64, max_s: f64) -> armd::motion::ActionStatus {
    for _ in 0..(max_s / DT) as usize {
        inner.tick(DT);
        if let Some(a) = inner.last_action.clone() {
            if a.id == id && !a.is_running() {
                return a;
            }
        }
    }
    panic!("action {id} still running after {max_s} s: {:?}", inner.last_action);
}

#[tokio::test]
async fn test_passive_boot_check_states() {
    // 1. Normal boot: J7 = 0.0 -> state "ok"
    let supervisor = ArmSupervisor::new(ArmConfig::rebot_dm());
    let obs = supervisor.get_observation().await;
    let gz = obs.gripper_zero.expect("gripper_zero present on 7-DOF");
    assert_eq!(gz.state, "ok");
    assert!((gz.j7 - 0.0).abs() < 1e-4);

    // 2. Boot with J7 = -6.5 rad (outside [-6.0, 0.3]) -> state "out_of_range"
    {
        let mut inner = supervisor.inner().write().await;
        inner.measured_positions[6] = -6.5;
    }
    let obs = supervisor.get_observation().await;
    let gz = obs.gripper_zero.unwrap();
    assert_eq!(gz.state, "out_of_range");
    assert!((gz.j7 - (-6.5)).abs() < 1e-4);

    // 3. Corrupt file -> state "calib_invalid"
    let dir = tempfile::tempdir().unwrap();
    let corrupt_path = dir.path().join("gripper_calib.json");
    std::fs::write(&corrupt_path, "invalid json {{{").unwrap();
    armd::gripper_calib::load_into(&supervisor, &corrupt_path).await;
    let obs = supervisor.get_observation().await;
    let gz = obs.gripper_zero.unwrap();
    assert_eq!(gz.state, "calib_invalid");

    // 4. Arm without gripper (6-DOF) -> gripper_zero is None
    let sup6 = ArmSupervisor::new(ArmConfig::default());
    assert!(sup6.get_observation().await.gripper_zero.is_none());
}

#[test]
fn test_zero_check_mock_stall_pass_and_fail() {
    let mut inner = sim_at(&INSPECT);
    inner.measured_positions[6] = -0.15; // start close to closed for fast test execution
    inner.commanded_positions[6] = -0.15;
    inner.target_positions[6] = -0.15;

    // 1. Mock stall at +0.05 -> offset 0.05, pass: true (threshold 0.10)
    inner.set_mock_gripper_stall_offset(Some(0.05));
    let epoch = inner.safety.epoch_id();
    let (id, _duration) = inner.start_gripper_zero_check(&json!({"threshold_rad": 0.10, "store_offset": false}), epoch).unwrap();
    let a = run_ticks(&mut inner, id, 5.0);
    assert_eq!(a.state, "done");
    assert!((a.stop_rad.unwrap() - 0.05).abs() < 1e-3, "stop_rad was {:?}", a.stop_rad);
    assert!((a.offset_rad.unwrap() - 0.05).abs() < 1e-3, "offset_rad was {:?}", a.offset_rad);
    assert_eq!(a.pass, Some(true));

    let gz = inner.gripper_zero_observation().unwrap();
    assert_eq!(gz.state, "checked");
    assert!((gz.j7 - 0.05).abs() < 1e-3);
    assert!((gz.offset_rad.unwrap() - 0.05).abs() < 1e-3);

    // 2. Mock stall at -0.30 -> offset -0.30, pass: false (threshold 0.10)
    inner.measured_positions[6] = -0.50;
    inner.commanded_positions[6] = -0.50;
    inner.target_positions[6] = -0.50;
    inner.gripper_zero_checked = false;
    inner.set_mock_gripper_stall_offset(Some(-0.30));
    let (id2, _) = inner.start_gripper_zero_check(&json!({"threshold_rad": 0.10, "store_offset": false}), epoch).unwrap();
    let a2 = run_ticks(&mut inner, id2, 5.0);
    assert_eq!(a2.state, "done");
    assert!((a2.stop_rad.unwrap() - (-0.30)).abs() < 1e-3);
    assert!((a2.offset_rad.unwrap() - (-0.30)).abs() < 1e-3);
    assert_eq!(a2.pass, Some(false));
}

#[test]
fn test_sim_follower_stops_at_closed_rad() {
    let mut inner = sim_at(&INSPECT);
    inner.measured_positions[6] = -0.15;
    inner.commanded_positions[6] = -0.15;
    inner.target_positions[6] = -0.15;
    inner.set_mock_gripper_stall_offset(None); // Sim follower default: stops at closed_rad (0.0)

    let epoch = inner.safety.epoch_id();
    let (id, _) = inner.start_gripper_zero_check(&json!({}), epoch).unwrap();
    let a = run_ticks(&mut inner, id, 5.0);
    assert_eq!(a.state, "done");
    assert!((a.stop_rad.unwrap() - 0.0).abs() < 1e-3);
    assert!((a.offset_rad.unwrap() - 0.0).abs() < 1e-3);
    assert_eq!(a.pass, Some(true));
}

#[test]
fn test_zero_check_refuses_when_held_or_not_torqued() {
    let mut inner = sim_at(&INSPECT);
    let epoch = inner.safety.epoch_id();

    // 1. Refuse while torque is off
    inner.is_torqued = false;
    let err = inner.start_gripper_zero_check(&json!({}), epoch).unwrap_err();
    assert!(matches!(err, ArmError::Safety(_)));

    inner.is_torqued = true;

    // 2. Refuse while an object is held
    inner.held = Some(SupHeldObject { label: "cube".into(), spill_safe: false });
    let err = inner.start_gripper_zero_check(&json!({}), epoch).unwrap_err();
    match err {
        ArmError::Rejected { status, code, message } => {
            assert_eq!(status, 409);
            assert_eq!(code, "OBJECT_HELD");
            assert!(message.contains("holds an object"));
        }
        other => panic!("expected Rejected with OBJECT_HELD, got {other:?}"),
    }
}

#[tokio::test]
async fn test_zero_check_store_and_reload_shifts_policy() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("gripper_calib.json");

    let supervisor = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    {
        let mut inner = supervisor.inner().write().await;
        inner.lease_enabled = false;
        inner.set_torque(true).unwrap();
        inner.measured_positions = INSPECT.to_vec();
        inner.commanded_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        inner.measured_positions[6] = -0.15;
        inner.commanded_positions[6] = -0.15;
        inner.target_positions[6] = -0.15;
        inner.gripper_calib_path = Some(path.clone());
        inner.set_mock_gripper_stall_offset(Some(0.05));
    }

    let epoch = supervisor.inner().read().await.safety.epoch_id();
    let (id, _) = supervisor.inner().write().await.start_gripper_zero_check(
        &json!({"threshold_rad": 0.10, "store_offset": true}),
        epoch,
    ).unwrap();

    // Run ticks until completion
    let a = {
        let mut inner = supervisor.inner().write().await;
        run_ticks(&mut inner, id, 5.0)
    };
    assert_eq!(a.state, "done");
    assert!((a.offset_rad.unwrap() - 0.05).abs() < 1e-3);
    assert_eq!(a.pass, Some(true));

    // Wait slightly for file write
    tokio::time::sleep(Duration::from_millis(50)).await;
    assert!(path.exists(), "gripper_calib.json must be written");

    let content: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    assert!((content["offset_rad"].as_f64().unwrap() - 0.05).abs() < 1e-3);

    // Effective policy of current supervisor is shifted
    {
        let inner = supervisor.inner().read().await;
        let p = inner.config.gripper_policy.as_ref().unwrap();
        assert!((p.closed_rad - 0.05).abs() < 1e-3);
        assert!((p.open_rad - (-5.65)).abs() < 1e-3);
        let jo = p.jaw_opening();
        assert!((jo.points[0][0] - 0.05).abs() < 1e-3);
        assert!((jo.points[1][0] - (-5.65)).abs() < 1e-3);
    }

    // Now reload into a fresh supervisor
    let fresh = ArmSupervisor::new(ArmConfig::rebot_dm());
    // Before load:
    {
        let inner = fresh.inner().read().await;
        let p = inner.config.gripper_policy.as_ref().unwrap();
        assert!((p.closed_rad - 0.0).abs() < 1e-4);
        assert!((p.open_rad - (-5.7)).abs() < 1e-4);
    }

    armd::gripper_calib::load_into(&fresh, &path).await;

    // After load:
    {
        let inner = fresh.inner().read().await;
        let p = inner.config.gripper_policy.as_ref().unwrap();
        assert!((p.closed_rad - 0.05).abs() < 1e-3);
        assert!((p.open_rad - (-5.65)).abs() < 1e-3);
        let jo = p.jaw_opening();
        assert!((jo.points[0][0] - 0.05).abs() < 1e-3);
        assert!((jo.points[1][0] - (-5.65)).abs() < 1e-3);
    }
}

#[tokio::test]
async fn test_zero_check_large_offset_refused_store() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("gripper_calib.json");

    let mut inner = sim_at(&INSPECT);
    inner.measured_positions[6] = -0.80;
    inner.commanded_positions[6] = -0.80;
    inner.target_positions[6] = -0.80;
    inner.gripper_calib_path = Some(path.clone());
    inner.set_mock_gripper_stall_offset(Some(-0.60)); // |-0.60| > 0.5 rad

    let epoch = inner.safety.epoch_id();
    let (id, _) = inner.start_gripper_zero_check(&json!({"threshold_rad": 0.10, "store_offset": true}), epoch).unwrap();
    let a = run_ticks(&mut inner, id, 5.0);
    assert_eq!(a.state, "done");
    assert!((a.offset_rad.unwrap() - (-0.60)).abs() < 1e-3);
    assert_eq!(a.pass, Some(false));

    tokio::time::sleep(Duration::from_millis(50)).await;
    assert!(!path.exists(), "file must not be written when |offset| > 0.5");
    // Policy remains unshifted
    let p = inner.config.gripper_policy.as_ref().unwrap();
    assert!((p.closed_rad - 0.0).abs() < 1e-4);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn test_ws_calibration_write_and_rest_get() {
    let dir = tempfile::tempdir().unwrap();
    let calib_path = dir.path().join("gripper_calib.json");

    let supervisor = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    supervisor.inner().write().await.gripper_calib_path = Some(calib_path.clone());

    let (mut ws, generation, epoch, _sup, addr) = common::torqued_rig(supervisor.clone(), &INSPECT).await;

    // 1. Without lease: calibration write is rejected
    {
        let unauth_reply = common::rpc(&mut ws, 99999, epoch, "/api/v1/gripper/calibration", json!({
            "jaw_opening": { "points": [[0.0, 0.0], [-5.0, 0.08]], "calibrated": true }
        })).await;
        assert_eq!(unauth_reply["type"], "command_reject");
        assert_eq!(unauth_reply["error"], "LEASE_LOST");
    }

    // 2. Invalid jaw_opening validations
    // Fewer than 2 points
    let r1 = common::rpc(&mut ws, generation, epoch, "/api/v1/gripper/calibration", json!({
        "jaw_opening": { "points": [[0.0, 0.0]], "calibrated": true }
    })).await;
    assert_eq!(r1["type"], "command_reject");
    assert_eq!(r1["code"], "INVALID_COMMAND");

    // Non-monotonic
    let r2 = common::rpc(&mut ws, generation, epoch, "/api/v1/gripper/calibration", json!({
        "jaw_opening": { "points": [[0.0, 0.0], [-2.0, 0.05], [-1.0, 0.08]], "calibrated": true }
    })).await;
    assert_eq!(r2["type"], "command_reject");
    assert_eq!(r2["code"], "INVALID_COMMAND");

    // Out of J7 limits
    let r3 = common::rpc(&mut ws, generation, epoch, "/api/v1/gripper/calibration", json!({
        "jaw_opening": { "points": [[0.0, 0.0], [-10.0, 0.08]], "calibrated": true }
    })).await;
    assert_eq!(r3["type"], "command_reject");
    assert_eq!(r3["code"], "INVALID_COMMAND");

    // Opening out of [0, 0.2]
    let r4 = common::rpc(&mut ws, generation, epoch, "/api/v1/gripper/calibration", json!({
        "jaw_opening": { "points": [[0.0, 0.0], [-5.0, 0.35]], "calibrated": true }
    })).await;
    assert_eq!(r4["type"], "command_reject");
    assert_eq!(r4["code"], "INVALID_COMMAND");

    // |offset_rad| > 0.5
    let r5 = common::rpc(&mut ws, generation, epoch, "/api/v1/gripper/calibration", json!({
        "jaw_opening": { "points": [[0.0, 0.0], [-5.0, 0.08]], "calibrated": true },
        "offset_rad": 0.65
    })).await;
    assert_eq!(r5["type"], "command_reject");
    assert_eq!(r5["code"], "INVALID_COMMAND");

    // 3. Valid calibration write
    let valid_write_body = json!({
        "jaw_opening": {
            "points": [[0.0, 0.0], [-5.0, 0.08]],
            "calibrated": true
        },
        "offset_rad": 0.02
    });
    let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/gripper/calibration", valid_write_body.clone()).await;
    println!("CALIBRATION WRITE REQUEST: {}", serde_json::to_string_pretty(&valid_write_body).unwrap());
    println!("CALIBRATION WRITE RESPONSE ACK: {}", serde_json::to_string_pretty(&ack).unwrap());

    assert_eq!(ack["type"], "command_ack");
    assert_eq!(ack["status"], "accepted");
    assert_eq!(ack["persisted"], true);
    assert!((ack["closed_rad"].as_f64().unwrap() - 0.02).abs() < 1e-3);
    assert!((ack["open_rad"].as_f64().unwrap() - (-5.68)).abs() < 1e-3);
    let jaw = &ack["jaw_opening"];
    assert_eq!(jaw["calibrated"], true);
    assert!((jaw["points"][0][0].as_f64().unwrap() - 0.02).abs() < 1e-3);
    assert!((jaw["points"][1][0].as_f64().unwrap() - (-4.98)).abs() < 1e-3);

    // 4. REST GET /api/v1/gripper/calibration (no lease)
    let client = reqwest::Client::new();
    let res = client.get(format!("http://{addr}/api/v1/gripper/calibration")).send().await.unwrap();
    assert_eq!(res.status(), 200);
    let body: serde_json::Value = res.json().await.unwrap();
    assert_eq!(body["calibrated"], true);
    assert!((body["offset_rad"].as_f64().unwrap() - 0.02).abs() < 1e-3);
    assert!((body["policy"]["closed_rad"].as_f64().unwrap() - 0.02).abs() < 1e-3);
    assert!((body["policy"]["open_rad"].as_f64().unwrap() - (-5.68)).abs() < 1e-3);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn test_ws_zero_check_e2e() {
    let supervisor = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let (mut ws, generation, epoch, sup, _addr) = common::torqued_rig(supervisor.clone(), &INSPECT).await;

    // Set mock stall at 0.04
    sup.set_mock_gripper_stall_offset(Some(0.04)).await;

    // Start near closed for fast test
    {
        let mut inner = sup.inner().write().await;
        inner.measured_positions[6] = -0.15;
        inner.commanded_positions[6] = -0.15;
        inner.target_positions[6] = -0.15;
    }

    let req_body = json!({
        "threshold_rad": 0.10,
        "store_offset": false
    });
    let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/action/gripper_zero_check", req_body.clone()).await;
    println!("ZERO CHECK REQUEST: {}", serde_json::to_string_pretty(&req_body).unwrap());
    println!("ZERO CHECK RESPONSE ACK: {}", serde_json::to_string_pretty(&ack).unwrap());

    assert_eq!(ack["type"], "command_ack");
    assert_eq!(ack["status"], "accepted");
    let action_id = ack["action_id"].as_u64().unwrap();

    // Wait for action completion via observation telemetry
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut finished = None;
    let mut last_hb = Instant::now();
    while Instant::now() < deadline {
        if last_hb.elapsed() > Duration::from_millis(500) {
            common::send(&mut ws, json!({"type": "heartbeat", "generation": generation, "epoch_id": epoch})).await;
            last_hb = Instant::now();
        }
        let obs = sup.get_observation().await;
        if let Some(a) = obs.action.filter(|a| a.id == action_id && !a.is_running()) {
            finished = Some(a);
            break;
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
    }

    let a = finished.expect("action did not complete in time");
    println!("COMPLETED ZERO CHECK ACTION STATUS: {}", serde_json::to_string_pretty(&a).unwrap());
    assert_eq!(a.state, "done");
    assert!((a.stop_rad.unwrap() - 0.04).abs() < 1e-3);
    assert!((a.offset_rad.unwrap() - 0.04).abs() < 1e-3);
    assert_eq!(a.pass, Some(true));
}

#[test]
fn test_zero_check_from_fully_open_finishes_before_its_deadline() {
    // 0.3 rad/s from the open limit to the stall takes about 19 s: the deadline follows the travel
    let mut inner = sim_at(&INSPECT);
    inner.measured_positions[6] = -5.7;
    inner.commanded_positions[6] = -5.7;
    inner.target_positions[6] = -5.7;
    inner.set_mock_gripper_stall_offset(Some(0.0));
    let epoch = inner.safety.epoch_id();
    let (id, _) = inner.start_gripper_zero_check(&json!({}), epoch).unwrap();
    let a = run_ticks(&mut inner, id, 40.0);
    assert_eq!(a.state, "done", "{:?} {:?}", a.code, a.message);
    assert!(a.offset_rad.unwrap().abs() < 1e-2, "offset_rad was {:?}", a.offset_rad);
}

#[test]
fn test_interrupted_release_still_counts_as_holding() {
    use armd::motion::{GripperCommand, GripperRequest};
    let open = || GripperRequest { command: GripperCommand::Open, fraction: None, grasp: false, speed: 1.0, timeout_s: Some(10.0), tag: None };
    let mut inner = sim_at(&INSPECT);
    let epoch = inner.safety.epoch_id();
    inner.held = Some(SupHeldObject { label: "cube".into(), spill_safe: false });

    // an Open stopped halfway keeps the object held: the zero check is refused
    let (_id, _) = inner.start_gripper(open(), epoch).unwrap();
    for _ in 0..10 { inner.tick(DT); }
    inner.stop_motion();
    inner.commanded_velocities.fill(0.0);
    assert!(inner.held.is_some(), "held cleared by an unfinished release");
    let err = inner.start_gripper_zero_check(&json!({}), epoch).unwrap_err();
    assert!(matches!(err, ArmError::Rejected { ref code, .. } if code == "OBJECT_HELD"), "{err:?}");

    // a completed Open clears it
    let (id, _) = inner.start_gripper(open(), epoch).unwrap();
    let a = run_ticks(&mut inner, id, 12.0);
    assert_eq!(a.state, "done");
    assert!(inner.held.is_none(), "held kept after a completed release");
}
