//! Keep-out boxes: profile list, runtime list over WS, persistence.

mod common;

use std::sync::Arc;
use serde_json::json;

use armd::kinematics;
use armd::model::{ArmConfig, KeepoutBox};
use armd::motion::{plan_cartesian, resolve_goal, GoalSpec, MotionError, Speed};
use armd::{ArmError, ArmSupervisor, SupervisorInner};

const INSPECT: [f32; 7] = [0.0, -1.167, -1.565, 1.435, 0.0, std::f32::consts::FRAC_PI_2, 0.0];

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

fn sim_at(config: ArmConfig, q: &[f32; 7]) -> SupervisorInner {
    let mut inner = SupervisorInner::new(config);
    inner.lease_enabled = false;
    inner.set_torque(true).unwrap();
    inner.commanded_positions = q.to_vec();
    inner.measured_positions = q.to_vec();
    inner.target_positions = q.to_vec();
    inner
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
async fn runtime_keepout_replaces_the_profile_list_and_is_persisted() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("keepout.json");
    let supervisor = Arc::new(ArmSupervisor::new(with_boxes(vec![cube("profile_box", [0.3, 0.3, 0.3])])));
    armd::keepout::load_into(&supervisor, &path).await;
    let (mut ws, generation, epoch, _supervisor, addr) = common::torqued_rig(supervisor, &INSPECT).await;
    let get = || async { reqwest::get(format!("http://{addr}/api/v1/workspace/keepout")).await.unwrap().json::<serde_json::Value>().await.unwrap() };
    let state = get().await;
    assert_eq!(state["source"], "profile");
    assert_eq!(state["boxes"][0]["name"], "profile_box");

    let p0 = tcp(&INSPECT);
    let mid = [p0[0] + 0.04, p0[1], p0[2]];
    let b = cube("tray", mid);
    let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/workspace/keepout", json!({"boxes": [b]})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    assert_eq!(ack["source"], "runtime");
    assert_eq!(ack["persisted"], true);
    let state = get().await;
    assert_eq!(state["source"], "runtime");
    assert_eq!(state["boxes"].as_array().unwrap().len(), 1);
    assert_eq!(state["boxes"][0]["name"], "tray");
    let saved: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    assert_eq!(saved["boxes"][0]["name"], "tray");

    let reply = common::rpc(&mut ws, generation, epoch, "/api/v1/action/move_to",
        json!({"position": [p0[0] + 0.08, p0[1], p0[2]], "orientation": null})).await;
    assert_eq!(reply["type"], "command_reject", "{reply}");
    assert_eq!(reply["code"], "WORKSPACE_VIOLATION");
    assert!(reply["message"].as_str().unwrap().contains("keepout 'tray'"), "{reply}");

    // Invalid list: refused, previous list kept.
    let reply = common::rpc(&mut ws, generation, epoch, "/api/v1/workspace/keepout", json!({"boxes": vec![b.clone(); 17]})).await;
    assert_eq!(reply["code"], "INVALID_COMMAND", "{reply}");
    assert_eq!(get().await["boxes"][0]["name"], "tray");

    // A new start loads the runtime list.
    let again = ArmSupervisor::new(ArmConfig::rebot_dm());
    armd::keepout::load_into(&again, &path).await;
    let state = again.inner().read().await.keepout_state();
    assert_eq!((state.source, state.boxes[0].name.as_str()), ("runtime", "tray"));
}

#[tokio::test]
async fn corrupt_keepout_file_is_ignored_and_left_untouched() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("keepout.json");
    std::fs::write(&path, "{\"boxes\": [ broken").unwrap();
    let supervisor = ArmSupervisor::new(with_boxes(vec![cube("p", [0.3, 0.3, 0.3])]));
    armd::keepout::load_into(&supervisor, &path).await;
    let state = supervisor.inner().read().await.keepout_state();
    assert_eq!((state.source, state.boxes.len()), ("profile", 1));
    assert_eq!(std::fs::read_to_string(&path).unwrap(), "{\"boxes\": [ broken");
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
