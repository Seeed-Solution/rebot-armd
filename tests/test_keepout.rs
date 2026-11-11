//! Keep-out boxes: profile list, runtime list over WS, persistence.

mod common;

use std::sync::Arc;
use serde_json::json;

use armd::kinematics;
use armd::model::{ArmConfig, KeepoutBox};
use armd::motion::{plan_cartesian, resolve_goal, GoalSpec, MotionError, Speed};
use armd::{ArmError, ArmSupervisor, SupervisorInner};
use common::{sim_at, INSPECT};

fn tcp(q: &[f32]) -> [f64; 3] { kinematics::fk(&kinematics::q6(q)).p }

/// 2 cm cube centred at `c` (base frame).
fn cube(name: &str, c: [f64; 3]) -> KeepoutBox {
    KeepoutBox { name: name.into(), min_m: c.map(|v| (v - 0.01) as f32), max_m: c.map(|v| (v + 0.01) as f32) }
}

fn with_boxes(boxes: Vec<KeepoutBox>) -> ArmConfig {
    let mut config = ArmConfig::rebot_dm();
    config.workspace.as_mut().unwrap().keepout_boxes = boxes;
    config
}

#[test]
fn profile_keepout_rejects_a_move_to_through_the_box() {
    let p0 = tcp(&INSPECT);
    let goal = [p0[0] + 0.08, p0[1], p0[2]];
    let mid = [p0[0] + 0.04, p0[1], p0[2]];
    let plan = |config: ArmConfig| {
        let inner = sim_at(config, &INSPECT);
        let input = inner.plan_input().unwrap();
        let g = resolve_goal(&input, &GoalSpec::Absolute { p: goal, r: None });
        plan_cartesian(&input, g, Speed::default())
    };
    assert!(plan(ArmConfig::rebot_dm()).is_ok());
    match plan(with_boxes(vec![cube("fixture", mid)])) {
        Err(MotionError::WorkspaceViolation { limit, s, .. }) => {
            assert_eq!(limit, "keepout 'fixture'");
            assert!(s > 0.2 && s < 0.8, "violation in the middle of the path: s={s}");
        }
        other => panic!("expected WORKSPACE_VIOLATION, got {other:?}"),
    }
    // A box beside the path does not interfere.
    assert!(plan(with_boxes(vec![cube("aside", [mid[0], mid[1] + 0.3, mid[2]])])).is_ok());
}

#[test]
fn keepout_applies_to_joint_moves_and_reports_the_box_name() {
    let mut inner = sim_at(with_boxes(vec![cube("bin_a", tcp(&INSPECT))]), &[0.0; 7]);
    let err = inner.jog_action(INSPECT.to_vec(), None, None, "joint_move").unwrap_err();
    let ArmError::Safety(e) = err else { panic!("{err:?}") };
    let api = armd::api::rest::ApiError::from(ArmError::Safety(e.clone()));
    assert_eq!(api.code.as_deref(), Some("WORKSPACE_VIOLATION"));
    assert!(e.to_string().contains("keepout 'bin_a'"), "{e}");
}

#[test]
fn keepout_lists_are_validated() {
    let ok = cube("a", [0.3, 0.0, 0.2]);
    let mut flat = ok.clone();
    flat.max_m[2] = flat.min_m[2];
    assert!(KeepoutBox::validate_list(&[flat]).is_err());
    assert!(KeepoutBox::validate_list(&[KeepoutBox { name: String::new(), ..ok.clone() }]).is_err());
    assert!(KeepoutBox::validate_list(&vec![ok.clone(); 17]).is_err());
    assert!(KeepoutBox::validate_list(&vec![ok.clone(); 16]).is_ok());
    let mut config = with_boxes(vec![ok]);
    config.validate().unwrap();
    config.workspace.as_mut().unwrap().keepout_boxes[0].min_m[0] = f32::NAN;
    assert!(config.validate().is_err());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn runtime_keepout_adds_to_the_profile_list_and_is_persisted() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("keepout.json");
    let supervisor = Arc::new(ArmSupervisor::new(with_boxes(vec![cube("profile_box", [0.3, 0.3, 0.3])])));
    armd::keepout::load_into(&supervisor, &path).await;
    let (mut ws, generation, epoch, _supervisor, addr) = common::torqued_rig(supervisor, &INSPECT).await;
    let get = || async { reqwest::get(format!("http://{addr}/api/v1/workspace/keepout")).await.unwrap().json::<serde_json::Value>().await.unwrap() };
    let state = get().await;
    assert_eq!(state["source"], "profile");
    assert_eq!(state["boxes"][0]["name"], "profile_box");
    assert_eq!(state["boxes"][0]["source"], "profile");

    let p0 = tcp(&INSPECT);
    let mid = [p0[0] + 0.04, p0[1], p0[2]];
    let b = cube("tray", mid);
    let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/workspace/keepout", json!({"boxes": [b]})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    assert_eq!(ack["source"], "runtime");
    assert_eq!(ack["persisted"], true);
    let state = get().await;
    assert_eq!(state["source"], "runtime");
    assert_eq!(state["boxes"].as_array().unwrap().len(), 2, "{state}");
    assert_eq!((&state["boxes"][0]["name"], &state["boxes"][0]["source"]), (&json!("profile_box"), &json!("profile")));
    assert_eq!((&state["boxes"][1]["name"], &state["boxes"][1]["source"]), (&json!("tray"), &json!("runtime")));
    let saved: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    assert_eq!(saved["boxes"].as_array().unwrap().len(), 1, "only the runtime list is persisted: {saved}");
    assert_eq!(saved["boxes"][0]["name"], "tray");

    let reply = common::rpc(&mut ws, generation, epoch, "/api/v1/action/move_to",
        json!({"position": [p0[0] + 0.08, p0[1], p0[2]], "orientation": null})).await;
    assert_eq!(reply["type"], "command_reject", "{reply}");
    assert_eq!(reply["code"], "WORKSPACE_VIOLATION");
    assert!(reply["message"].as_str().unwrap().contains("keepout 'tray'"), "{reply}");

    // Invalid list: refused, previous list kept.
    let reply = common::rpc(&mut ws, generation, epoch, "/api/v1/workspace/keepout", json!({"boxes": vec![b.clone(); 17]})).await;
    assert_eq!(reply["code"], "INVALID_COMMAND", "{reply}");
    assert_eq!(get().await["boxes"][1]["name"], "tray");
    // An empty runtime list removes no profile box.
    let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/workspace/keepout", json!({"boxes": []})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let state = get().await;
    assert_eq!(state["boxes"].as_array().unwrap().len(), 1, "{state}");
    assert_eq!(state["boxes"][0]["name"], "profile_box");
    let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/workspace/keepout", json!({"boxes": [b]})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");

    // A new start loads the runtime list.
    let again = ArmSupervisor::new(ArmConfig::rebot_dm());
    armd::keepout::load_into(&again, &path).await;
    let state = again.inner().read().await.keepout_state();
    assert_eq!((state.source, state.boxes[0].name.as_str()), ("runtime", "tray"));
}

#[tokio::test]
async fn corrupt_keepout_file_is_ignored_and_moved_aside() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("keepout.json");
    std::fs::write(&path, "{\"boxes\": [ broken").unwrap();
    let supervisor = ArmSupervisor::new(with_boxes(vec![cube("p", [0.3, 0.3, 0.3])]));
    armd::keepout::load_into(&supervisor, &path).await;
    let state = supervisor.inner().read().await.keepout_state();
    assert_eq!((state.source, state.boxes.len()), ("profile", 1));
    assert!(!path.exists());
    let bad: Vec<_> = std::fs::read_dir(dir.path()).unwrap().map(|e| e.unwrap().path()).collect();
    assert_eq!(bad.len(), 1, "{bad:?}");
    assert!(bad[0].file_name().unwrap().to_string_lossy().starts_with("keepout.json.bad-"), "{bad:?}");
    assert_eq!(std::fs::read_to_string(&bad[0]).unwrap(), "{\"boxes\": [ broken");
}

/// A keep-out box on the park path from INSPECT (at the midpoint of the joint-space move home).
fn park_blocking_box(inner: &SupervisorInner) -> KeepoutBox {
    let mid: Vec<f32> = INSPECT.iter().zip(&inner.config.home_positions).map(|(a, b)| (a + b) / 2.0).collect();
    let mut b = cube("on_park_path", tcp(&mid));
    b.min_m = b.min_m.map(|v| v - 0.03);
    b.max_m = b.max_m.map(|v| v + 0.03);
    b
}

#[test]
fn explicit_park_through_a_keepout_box_is_workspace_violation() {
    let probe = sim_at(ArmConfig::rebot_dm(), &INSPECT);
    let mut inner = sim_at(with_boxes(vec![park_blocking_box(&probe)]), &INSPECT);
    let err = inner.unload_action().unwrap_err();
    let api = armd::api::rest::ApiError::from(err);
    assert_eq!(api.code.as_deref(), Some("WORKSPACE_VIOLATION"), "{api:?}");
    assert!(api.message.contains("keepout 'on_park_path'"), "{}", api.message);
    assert!(!inner.pending_normal_unload && inner.active_trajectory.is_none(), "park must not start");
    assert!(inner.is_torqued, "torque held");
    // Without the box the same park starts.
    let mut inner = sim_at(ArmConfig::rebot_dm(), &INSPECT);
    assert!(inner.unload_action().unwrap().is_some());
}

#[test]
fn watchdog_park_through_a_keepout_box_holds_and_latches_watchdog_timeout() {
    let probe = sim_at(ArmConfig::rebot_dm(), &INSPECT);
    let mut inner = sim_at(with_boxes(vec![park_blocking_box(&probe)]), &INSPECT);
    inner.lease_enabled = true;
    inner.lease_timeout = std::time::Duration::from_millis(10);
    inner.last_heartbeat = std::time::Instant::now() - std::time::Duration::from_secs(1);
    inner.tick(0.02);
    assert!(matches!(inner.safety.state(), armd::SafetyState::Fault(armd::safety::FaultReason::WatchdogTimeout)), "{:?}", inner.safety.state());
    assert!(!inner.pending_normal_unload && inner.active_trajectory.is_none() && inner.is_torqued);
    let moved = INSPECT.iter().zip(&inner.commanded_positions).map(|(a, b)| (a - b).abs()).fold(0.0f32, f32::max);
    assert!(moved < 1e-3, "held in place ({moved} rad)");
}

/// A servo target outside every box whose stepped path crosses one: each tick's step is
/// checked, the stream stops in the recoverable hold before the box.
#[test]
fn servo_stream_cannot_sweep_through_a_keepout_box() {
    let (mut target, mut mid) = (INSPECT, INSPECT);
    // one frame (servo targets are at most 0.35 rad from the commanded pose)
    target[0] += 0.3;
    mid[0] += 0.15;
    let c = tcp(&mid);
    let b = KeepoutBox { name: "between".into(), min_m: c.map(|v| (v - 0.02) as f32), max_m: c.map(|v| (v + 0.02) as f32) };
    let mut inner = sim_at(with_boxes(vec![b]), &INSPECT);
    let epoch = inner.safety.epoch_id();
    inner.servo(target.to_vec(), epoch).expect("the target itself is outside the box");
    let ws = inner.config.workspace.clone().unwrap();
    for _ in 0..300 {
        inner.tick(0.02);
        let v = inner.config.arm_model.workspace_violation(&inner.commanded_positions, &ws, &inner.config.camera_mount);
        assert!(v.total <= 1e-9, "commanded pose entered the box: {:?}", inner.commanded_positions);
    }
    assert!(inner.servo_blocked_hold && inner.servo_target.is_none(), "stream must end in the hold: q={:?} target={:?} limit={:?} state={:?}", inner.commanded_positions, inner.servo_target, inner.motion_limit, inner.safety.state());
    assert!((inner.commanded_positions[0] - INSPECT[0]).abs() > 0.03, "moved towards the box first");
    let limit = inner.motion_limit.as_ref().expect("motion_limit");
    assert_eq!(limit.reason, armd::model::MotionLimitReason::Workspace);
}

/// The runtime list adds to the profile list: a profile box stays enforced whatever the
/// runtime list is, and the combined list is capped.
#[test]
fn runtime_keepout_is_additive_with_the_profile() {
    let p0 = tcp(&INSPECT);
    let mid = [p0[0] + 0.04, p0[1], p0[2]];
    let mut inner = sim_at(with_boxes(vec![cube("fixture", mid)]), &INSPECT);
    inner.set_keepout_boxes(vec![cube("aside", [mid[0], mid[1] + 0.3, mid[2]])]).unwrap();
    let names: Vec<_> = inner.config.workspace.as_ref().unwrap().keepout_boxes.iter().map(|b| b.name.clone()).collect();
    assert_eq!(names, ["fixture", "aside"]);
    inner.set_keepout_boxes(vec![]).unwrap();
    let input = inner.plan_input().unwrap();
    let g = resolve_goal(&input, &GoalSpec::Absolute { p: [p0[0] + 0.08, p0[1], p0[2]], r: None });
    match plan_cartesian(&input, g, Speed::default()) {
        Err(MotionError::WorkspaceViolation { limit, .. }) => assert_eq!(limit, "keepout 'fixture'"),
        other => panic!("profile box must stay in force, got {other:?}"),
    }
    let sixteen: Vec<_> = (0..16).map(|i| cube(&format!("r{i}"), [0.3, 0.3, 0.05 * f64::from(i)])).collect();
    assert!(matches!(inner.set_keepout_boxes(sixteen), Err(ArmError::Motion(MotionError::InvalidCommand(_)))), "17 in force");
    assert_eq!(inner.keepout_state().boxes.len(), 1);
}

/// Accepted motion was checked against the boxes in force: they cannot change under it, and a
/// plan made before a change is stale at install.
#[test]
fn keepout_change_is_busy_during_motion_and_makes_plans_stale() {
    let p0 = tcp(&INSPECT);
    let mut inner = sim_at(ArmConfig::rebot_dm(), &INSPECT);
    let mut q = INSPECT.to_vec();
    q[0] += 0.2;
    inner.jog(q, None, None).unwrap();
    let err = inner.set_keepout_boxes(vec![cube("late", [0.3, 0.3, 0.3])]).unwrap_err();
    assert_eq!(armd::api::rest::ApiError::from(err).code.as_deref(), Some("MOTION_BUSY"));
    assert!(inner.keepout_state().boxes.is_empty());

    let mut inner = sim_at(ArmConfig::rebot_dm(), &INSPECT);
    let input = inner.plan_input().unwrap();
    let g = resolve_goal(&input, &GoalSpec::Absolute { p: [p0[0] + 0.08, p0[1], p0[2]], r: None });
    let path = plan_cartesian(&input, g, Speed::default()).unwrap();
    // the box lands on the planned path after planning
    inner.set_keepout_boxes(vec![cube("late", [p0[0] + 0.04, p0[1], p0[2]])]).unwrap();
    let err = inner.install_path(&input, path, "move_to", None).unwrap_err();
    assert_eq!(armd::api::rest::ApiError::from(err).code.as_deref(), Some("PLAN_STALE"));
    assert!(inner.active_trajectory.is_none());
}

/// A small box between two of the 21 planner samples of a long joint move is still found.
#[test]
fn joint_move_keepout_sampling_is_dense_enough_for_small_boxes() {
    let mut target = INSPECT;
    target[0] += 1.2;
    let mut probe = sim_at(ArmConfig::rebot_dm(), &INSPECT);
    probe.jog(target.to_vec(), None, None).unwrap();
    let traj = probe.active_trajectory.as_ref().unwrap().as_quintic().unwrap().clone();
    let between = traj.sample(traj.duration * 10.5 / 20.0).positions;
    let c = tcp(&between);
    let small = KeepoutBox { name: "pin".into(), min_m: c.map(|v| (v - 0.006) as f32), max_m: c.map(|v| (v + 0.006) as f32) };
    // the 21 samples miss it
    let samples = SupervisorInner::trajectory_samples(&traj);
    let ws = { let mut w = probe.config.workspace.clone().unwrap(); w.keepout_boxes = vec![small.clone()]; w };
    assert!(samples.iter().all(|q| probe.config.arm_model.workspace_violation(q, &ws, &probe.config.camera_mount).total == 0.0), "box must sit between samples");
    let mut inner = sim_at(with_boxes(vec![small]), &INSPECT);
    let err = inner.jog(target.to_vec(), None, None).unwrap_err();
    assert!(err.to_string().contains("keepout 'pin'"), "{err}");
    assert!(inner.active_trajectory.is_none());

    // a path that would need more than MAX_KEEPOUT_SAMPLES is refused with a clear message
    let tiny = KeepoutBox { name: "speck".into(), min_m: [0.9, 0.9, 0.9], max_m: [0.9001, 0.9001, 0.9001] };
    let mut inner = sim_at(with_boxes(vec![tiny]), &INSPECT);
    let err = inner.jog(target.to_vec(), None, None).unwrap_err();
    let api = armd::api::rest::ApiError::from(err);
    assert_eq!(api.code.as_deref(), Some("WORKSPACE_VIOLATION"));
    assert!(api.message.contains("needs more than 400 samples"), "{}", api.message);
}
