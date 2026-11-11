//! Collision guard (contract section 11): detection from residual torques, protective stop,
//! auto-resume, escalation, runtime setting.
//!
//! Torques come from `TestArm`, a test `ArmDriver` (the public driver trait, as in
//! tests/test_residual_torques.rs): an ideal position servo that reports the model gravity
//! torque plus a residual the test sets per joint. Nothing in armd's API can inject torque;
//! the injection lives only in this file.
mod common;

use std::sync::{Arc, Mutex};
use std::time::Duration;
use armd::api::rest::ApiError;
use armd::collision::{CollisionPolicy, Sensitivity};
use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointMode, JointTarget};
use armd::kinematics;
use armd::model::{ArmConfig, JointState};
use armd::motion::{plan_cartesian, resolve_goal, GoalSpec, Speed};
use armd::{ArmError, ArmSupervisor, FaultCode, SafetyState, SupervisorInner};
use serde_json::json;
use common::INSPECT;

const DT: f64 = 0.02;

#[derive(Default)]
struct ArmState { q: [f32; 7], torqued: bool, extra: [f32; 6], mit: bool }

#[derive(Clone)]
struct TestArm(Arc<Mutex<ArmState>>);

impl TestArm {
    fn new() -> Self { Self(Arc::new(Mutex::new(ArmState { q: INSPECT, ..Default::default() }))) }
    fn set_residual(&self, joint: usize, nm: f32) { self.0.lock().unwrap().extra[joint] = nm; }
    fn clear(&self) { self.0.lock().unwrap().extra = [0.0; 6]; }
}

impl ArmDriver for TestArm {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true, torque_feedforward: false, enable_feedback: true }
    }
    fn enable_state(&self) -> EnableState {
        if self.0.lock().unwrap().torqued { EnableState::Enabled } else { EnableState::Disabled }
    }
    fn is_feedback_valid(&self) -> bool { true }
    fn is_enabled(&self) -> bool { self.0.lock().unwrap().torqued }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); 7] }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let s = self.0.lock().unwrap();
        let mut q6 = [0.0; 6];
        q6.copy_from_slice(&s.q[..6]);
        let g = armd::compute_gravity_torques(&q6).unwrap();
        Ok((0..7).map(|i| JointState {
            position: s.q[i], velocity: 0.0, temperature: 30.0,
            torque: if i < 6 { g[i] + s.extra[i] } else { 0.0 },
        }).collect())
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        let mut s = self.0.lock().unwrap();
        // Ideal position servo; in MIT mode (gravity compensation) the arm stays put.
        if !s.mit { for (i, t) in targets.iter().enumerate().take(7) { s.q[i] = t.position; } }
        Ok(())
    }
    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        let mut s = self.0.lock().unwrap();
        s.torqued = enabled;
        s.mit = false;
        Ok(())
    }
    fn supports_mit_mode_switch(&self) -> bool { true }
    fn set_joint_modes(&mut self, modes: &[JointMode], _hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.0.lock().unwrap().mit = modes.iter().take(6).any(|m| *m == JointMode::Mit);
        Ok(())
    }
}

fn policy() -> CollisionPolicy {
    CollisionPolicy { enabled: true, resume_after_s: 0.3, ..CollisionPolicy::default() }
}

fn profile(policy: Option<CollisionPolicy>) -> ArmConfig {
    ArmConfig { collision: policy, lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() }
}

/// Torqued, armed SupervisorInner on a TestArm at INSPECT.
fn rig(policy: Option<CollisionPolicy>) -> (SupervisorInner, TestArm) {
    let arm = TestArm::new();
    let mut inner = SupervisorInner::new(profile(policy));
    inner.lease_enabled = false;
    inner.driver = Some(Mutex::new(Box::new(arm.clone())));
    inner.commanded_positions = INSPECT.to_vec();
    inner.measured_positions = INSPECT.to_vec();
    inner.target_positions = INSPECT.to_vec();
    inner.set_torque(true).unwrap();
    ticks(&mut inner, 5);
    assert!(inner.safety.state().is_operational(), "{:?}", inner.safety.state());
    (inner, arm)
}

fn ticks(inner: &mut SupervisorInner, n: usize) -> armd::ArmObservation {
    let mut obs = None;
    for _ in 0..n { obs = Some(inner.tick(DT)); }
    obs.unwrap()
}

fn start_move(inner: &mut SupervisorInner) -> u64 {
    let input = inner.plan_input().unwrap();
    let p0 = kinematics::fk(&kinematics::q6(&INSPECT)).p;
    let spec = GoalSpec::Absolute { p: [p0[0] + 0.06, p0[1], p0[2]], r: None };
    let goal = resolve_goal(&input, &spec);
    let path = plan_cartesian(&input, goal, Speed { linear: 0.05, angular: 0.5 }).unwrap();
    inner.install_path(&input, path, "move_to", None).unwrap().0
}

fn code_of(e: ArmError) -> (u16, String) {
    let api = ApiError::from(e);
    (api.status.as_u16(), api.code.unwrap_or_default())
}

#[test]
fn disabled_by_default_and_null_in_the_observation() {
    let (mut inner, arm) = rig(None);
    arm.set_residual(1, 50.0);
    let obs = ticks(&mut inner, 20);
    assert!(obs.residual_torques.is_some(), "residuals are measured");
    assert!(obs.collision.is_none(), "guard off: field is null");
    assert!(serde_json::to_value(&obs).unwrap()["collision"].is_null());
    assert!(inner.collision.hold.is_none() && inner.safety.state().is_operational());
    assert!(!CollisionPolicy::default().enabled);
}

#[test]
fn no_trip_below_threshold_or_shorter_than_persist_ticks() {
    let (mut inner, arm) = rig(Some(policy()));
    arm.set_residual(1, 5.9); // threshold J2 6.0
    let obs = ticks(&mut inner, 50);
    let c = obs.collision.expect("enabled guard is published");
    assert!(!c.active && c.trips_in_window == 0, "{c:?}");
    // above the threshold for persist_ticks - 1 ticks, then back down: still no trip
    arm.set_residual(1, 9.0);
    ticks(&mut inner, 3);
    arm.clear();
    let obs = ticks(&mut inner, 10);
    assert!(!obs.collision.unwrap().active);
}

#[test]
fn trip_cancels_a_running_move_to_holds_rejects_motion_and_auto_resumes() {
    let (mut inner, arm) = rig(Some(policy()));
    let id = start_move(&mut inner);
    ticks(&mut inner, 10);
    assert_eq!(inner.last_action.as_ref().unwrap().state, "running");
    arm.set_residual(2, -8.0);
    ticks(&mut inner, 3);
    assert!(inner.collision.hold.is_none(), "3 ticks < persist_ticks 4");
    let obs = ticks(&mut inner, 1);
    let a = obs.action.as_ref().unwrap();
    assert_eq!((a.id, a.state.as_str(), a.code.as_deref()), (id, "failed", Some("COLLISION")), "{a:?}");
    assert!(a.message.as_deref().unwrap().contains("joint 3"), "{a:?}");
    let c = obs.collision.unwrap();
    assert!(c.active && c.joint == Some(3) && c.trips_in_window == 1, "{c:?}");
    assert!((c.residual_nm.unwrap() + 8.0).abs() < 1e-3 && (c.threshold_nm.unwrap() - 6.0).abs() < 1e-6);
    println!("collision during trip: {}", serde_json::to_string(&obs.collision).unwrap());
    // not a SafetyState change; torque stays on, holding the measured pose
    assert!(obs.safety_state.is_operational() && obs.is_torqued && obs.fault.is_none());
    assert!(inner.active_trajectory.is_none());
    let held = inner.commanded_positions.clone();
    ticks(&mut inner, 5);
    assert_eq!(inner.commanded_positions, held, "hold: commanded stays at the measured pose");

    // motion is refused with COLLISION_HOLD (409)
    assert_eq!(code_of(inner.plan_input().unwrap_err()), (409, "COLLISION_HOLD".into()));
    assert_eq!(code_of(inner.jog_action(INSPECT.to_vec(), None, None, "joint_move").unwrap_err()), (409, "COLLISION_HOLD".into()));
    let epoch = inner.safety.epoch_id();
    assert_eq!(code_of(inner.servo(INSPECT.to_vec(), epoch).unwrap_err().into()), (409, "COLLISION_HOLD".into()));
    assert_eq!(code_of(inner.enter_gravity_comp(Default::default()).unwrap_err()), (409, "COLLISION_HOLD".into()));

    // residual still high: the hold stays
    let obs = ticks(&mut inner, 30);
    assert!(obs.collision.unwrap().active);
    // quiet (below clear_ratio x threshold) for resume_after_s: the hold ends by itself
    arm.set_residual(2, 2.9); // < 0.5 x 6.0
    let obs = ticks(&mut inner, 5);
    let c = obs.collision.unwrap();
    assert!(c.active && c.resume_in_s.unwrap() < 0.3 && c.resume_in_s.unwrap() > 0.1, "{c:?}");
    let obs = ticks(&mut inner, 12);
    let c = obs.collision.unwrap();
    assert!(!c.active && c.joint.is_none() && c.trips_in_window == 1, "{c:?}");
    println!("collision after auto-resume: {}", serde_json::to_string(&obs.collision).unwrap());
    start_move(&mut inner);
}

#[test]
fn no_detection_in_gravity_comp_mode() {
    let (mut inner, arm) = rig(Some(policy()));
    if inner.gravity_comp_support().is_err() {
        eprintln!("skipped: gravity model URDF not found ({:?})", inner.gravity_comp_support());
        return;
    }
    inner.enter_gravity_comp(Default::default()).unwrap();
    ticks(&mut inner, 5);
    assert!(inner.gravity_comp.is_some());
    arm.set_residual(1, 12.0);
    ticks(&mut inner, 40);
    assert!(inner.collision.hold.is_none(), "hand guiding produces residuals by design");
    assert_eq!(inner.collision_observation().unwrap().trips_in_window, 0);
}

#[test]
fn escalates_to_fault_collision_after_max_auto_resumes() {
    let p = CollisionPolicy { max_auto_resumes: 2, ..policy() };
    let (mut inner, arm) = rig(Some(p));
    for trip in 1..=2u32 {
        arm.set_residual(0, 7.0);
        let obs = ticks(&mut inner, 4);
        assert_eq!(obs.collision.unwrap().trips_in_window, trip);
        assert!(obs.collision.unwrap().active && obs.safety_state.is_operational());
        arm.clear();
        let obs = ticks(&mut inner, 20);
        assert!(!obs.collision.unwrap().active, "trip {trip} auto-resumed");
    }
    let id = start_move(&mut inner);
    ticks(&mut inner, 3);
    arm.set_residual(0, 7.0);
    let obs = ticks(&mut inner, 4);
    let a = obs.action.as_ref().unwrap();
    assert_eq!((a.id, a.code.as_deref()), (id, Some("COLLISION")));
    let fault = obs.fault.expect("third trip within the window latches Fault");
    assert_eq!(fault.code, FaultCode::Collision);
    assert!(!fault.recoverable);
    assert!(matches!(obs.safety_state, SafetyState::Fault(_)) && obs.is_torqued);
    assert_eq!(serde_json::to_value(&fault).unwrap()["code"], "COLLISION");
    let c = obs.collision.unwrap();
    assert!(!c.active && c.trips_in_window == 3, "{c:?}");
    // stays latched: no auto-resume from a Fault
    arm.clear();
    let obs = ticks(&mut inner, 50);
    assert!(matches!(obs.safety_state, SafetyState::Fault(_)));
    inner.clear_fault().unwrap();
}

#[test]
fn sensitivity_scales_the_thresholds() {
    let (mut inner, arm) = rig(Some(policy()));
    inner.collision.sensitivity = Sensitivity::Low; // x1.5 -> J4 3.75
    assert_eq!(inner.collision_thresholds(), [9.0, 9.0, 9.0, 3.75, 3.75, 3.75]);
    arm.set_residual(3, 3.5);
    assert!(!ticks(&mut inner, 10).collision.unwrap().active);
    inner.collision.sensitivity = Sensitivity::High; // x0.7 -> J4 1.75
    let t = inner.collision_thresholds();
    assert!((t[3] - 1.75).abs() < 1e-6 && (t[0] - 4.2).abs() < 1e-5, "{t:?}");
    let obs = ticks(&mut inner, 4);
    let c = obs.collision.unwrap();
    assert!(c.active && c.joint == Some(4) && c.sensitivity == Sensitivity::High, "{c:?}");
    assert_eq!(serde_json::to_value(c).unwrap()["sensitivity"], "high");
}

#[test]
fn e_stop_works_during_a_hold_and_ends_it() {
    let (mut inner, arm) = rig(Some(policy()));
    arm.set_residual(1, 9.0);
    assert!(ticks(&mut inner, 4).collision.unwrap().active);
    inner.apply_stop_policy(armd::StopPolicy::EStopLatch);
    let obs = ticks(&mut inner, 2);
    assert_eq!(obs.safety_state, SafetyState::EStop);
    assert_eq!(obs.fault.unwrap().code, FaultCode::Estop);
    assert!(!obs.collision.unwrap().active, "the stop supersedes the hold");
    // stop (motion/stop) also works during a hold
    let (mut inner, arm) = rig(Some(policy()));
    arm.set_residual(1, 9.0);
    ticks(&mut inner, 4);
    inner.stop_motion();
    assert!(inner.collision.hold.is_some());
}

#[test]
fn profile_section_is_validated() {
    let mut c = profile(Some(CollisionPolicy { clear_ratio: 1.0, ..policy() }));
    assert!(c.validate().unwrap_err().contains("clear_ratio"));
    let mut c = profile(Some(CollisionPolicy { max_auto_resumes: 16, ..policy() }));
    assert!(c.validate().is_err());
    let v: ArmConfig = serde_json::from_value({
        let mut v = serde_json::to_value(ArmConfig::rebot_dm()).unwrap();
        v["collision"] = json!({"enabled": true, "persist_ticks": 6});
        v
    }).unwrap();
    let p = v.collision.unwrap();
    assert!(p.enabled && p.persist_ticks == 6 && p.thresholds_nm == [6.0, 6.0, 6.0, 2.5, 2.5, 2.5] && p.max_auto_resumes == 3);
}

#[tokio::test]
async fn capability_needs_gravity_model_and_torque_feedback() {
    let with = ArmSupervisor::new(profile(None)).with_driver(Box::new(TestArm::new()));
    assert!(with.get_schema().await.capabilities.collision_guard);
    // mock follower: no torque feedback
    assert!(!ArmSupervisor::new(profile(None)).get_schema().await.capabilities.collision_guard);
    let mut no_model = profile(None);
    no_model.gravity_model = None;
    no_model.gravity_comp = None;
    assert!(!ArmSupervisor::new(no_model).with_driver(Box::new(TestArm::new())).get_schema().await.capabilities.collision_guard);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn runtime_setting_over_ws_is_persisted_and_holds_reject_motion() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("collision.json");
    let arm = TestArm::new();
    let supervisor = Arc::new(ArmSupervisor::new(profile(Some(CollisionPolicy { enabled: false, ..policy() }))).with_driver(Box::new(arm.clone())));
    armd::collision::load_into(&supervisor, &path).await;
    let (mut ws, generation, epoch, supervisor, addr) = common::torqued_rig(supervisor, &INSPECT).await;
    let get = || async { reqwest::get(format!("http://{addr}/api/v1/safety/collision")).await.unwrap().json::<serde_json::Value>().await.unwrap() };
    let state = get().await;
    assert_eq!((state["enabled"].as_bool(), state["source"].as_str(), state["available"].as_bool()), (Some(false), Some("profile"), Some(true)), "{state}");
    assert!(supervisor.get_observation().await.collision.is_none());

    let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/safety/collision", json!({"enabled": true, "sensitivity": "high"})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    assert_eq!(ack["persisted"], true);
    assert_eq!(ack["collision"]["sensitivity"], "high");
    let state = get().await;
    assert_eq!((state["enabled"].as_bool(), state["source"].as_str()), (Some(true), Some("runtime")));
    let t = state["thresholds_nm"].as_array().unwrap();
    assert!((t[0].as_f64().unwrap() - 4.2).abs() < 1e-5 && (t[3].as_f64().unwrap() - 1.75).abs() < 1e-5, "{state}");
    assert_eq!(state["profile_thresholds_nm"][0], 6.0);
    let saved: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    assert_eq!(saved, json!({"enabled": true, "sensitivity": "high"}));

    for bad in [json!({}), json!({"sensitivity": "max"}), json!({"enabled": "yes"})] {
        let r = common::rpc(&mut ws, generation, epoch, "/api/v1/safety/collision", bad.clone()).await;
        assert_eq!(r["code"], "INVALID_COMMAND", "{bad} -> {r}");
    }

    // trip (ticker at 50 Hz), then motion over WS is refused with COLLISION_HOLD
    arm.set_residual(1, 9.0);
    let deadline = std::time::Instant::now() + Duration::from_secs(3);
    while !supervisor.get_observation().await.collision.is_some_and(|c| c.active) {
        assert!(std::time::Instant::now() < deadline, "no trip");
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    let p0 = kinematics::fk(&kinematics::q6(&INSPECT)).p;
    let r = common::rpc(&mut ws, generation, epoch, "/api/v1/action/move_to", json!({"position": [p0[0] + 0.02, p0[1], p0[2]]})).await;
    assert_eq!((r["code"].as_str(), r["status"].as_u64()), (Some("COLLISION_HOLD"), Some(409)), "{r}");
    let r = common::preset(&mut ws, generation, epoch, "home", None).await;
    assert_eq!(r["code"], "COLLISION_HOLD", "{r}");
    let r = common::preset(&mut ws, generation, epoch, "park", None).await;
    assert_eq!(r["code"], "COLLISION_HOLD", "{r}");
    // stop still works
    let r = common::rpc(&mut ws, generation, epoch, "/api/v1/motion/stop", json!({})).await;
    assert_eq!(r["type"], "command_ack", "{r}");

    // switching the guard off ends the hold
    let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/safety/collision", json!({"enabled": false})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    assert!(supervisor.inner().read().await.collision.hold.is_none());
    assert!(supervisor.get_observation().await.collision.is_none());

    // a new start loads the runtime setting
    let again = ArmSupervisor::new(profile(None));
    armd::collision::load_into(&again, &path).await;
    let inner = again.inner().read().await;
    assert!(!inner.collision.enabled && inner.collision.sensitivity == Sensitivity::High && inner.collision.runtime);
}

#[tokio::test]
async fn enabling_without_torque_feedback_is_refused_and_lease_is_required() {
    let (addr, supervisor) = common::spawn_server(profile(None)).await;
    let (mut ws, generation) = common::controller(addr).await;
    let epoch = supervisor.get_observation().await.epoch_id;
    let r = common::rpc(&mut ws, generation, epoch, "/api/v1/safety/collision", json!({"enabled": true})).await;
    assert_eq!(r["code"], "UNSUPPORTED", "{r}");
    let (mut other, _) = tokio_tungstenite::connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
    let r = common::ws_reply(&mut other, json!({"type":"command", "request_id": 1, "generation": 0, "epoch_id": epoch,
        "path": "/api/v1/safety/collision", "body": {"enabled": false}}), &["command_ack", "command_reject"]).await;
    assert_eq!(r["code"], "LEASE_LOST", "{r}");
    let state = reqwest::get(format!("http://{addr}/api/v1/safety/collision")).await.unwrap().json::<serde_json::Value>().await.unwrap();
    assert_eq!((state["available"].as_bool(), state["enabled"].as_bool()), (Some(false), Some(false)));
}

/// A trip during a macro sub-motion ends the macro with COLLISION; its orchestrator can
/// install nothing more (here: a grasp's next gripper close).
#[test]
fn trip_during_a_grasp_sub_motion_ends_the_macro_with_collision() {
    let (mut inner, arm) = rig(Some(policy()));
    let input = inner.plan_input().unwrap();
    let p0 = kinematics::fk(&kinematics::q6(&INSPECT)).p;
    let goal = resolve_goal(&input, &GoalSpec::Absolute { p: [p0[0] + 0.06, p0[1], p0[2]], r: None });
    let path = plan_cartesian(&input, goal, Speed { linear: 0.05, angular: 0.5 }).unwrap();
    let id = inner.begin_macro("grasp", &input, None).unwrap();
    inner.install_macro_path(id, &input, path).unwrap();
    ticks(&mut inner, 10);
    arm.set_residual(1, 9.0);
    let obs = ticks(&mut inner, 4);
    let a = obs.action.as_ref().unwrap();
    assert_eq!((a.id, a.kind.as_str(), a.state.as_str(), a.code.as_deref()), (id, "grasp", "failed", Some("COLLISION")), "{a:?}");
    assert!(inner.macro_run.is_none() && inner.active_trajectory.is_none());
    let close = armd::motion::GripperRequest { command: armd::motion::GripperCommand::Close, fraction: None, grasp: true, speed: 1.0, timeout_s: 8.0, tag: None };
    assert!(inner.start_macro_gripper(id, close).is_err());
}

/// Full place orchestrator on the 50 Hz ticker: a trip during the approach ends it with
/// COLLISION and no later phase runs.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn trip_during_place_ends_it_and_no_phase_follows() {
    let arm = TestArm::new();
    let sup = Arc::new(ArmSupervisor::new(profile(Some(policy()))).with_driver(Box::new(arm.clone())));
    {
        let mut inner = sup.inner().write().await;
        inner.lease_enabled = false;
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        inner.set_torque(true).unwrap();
    }
    let ticker = sup.clone();
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(20));
        loop { interval.tick().await; ticker.tick(DT).await; }
    });
    tokio::time::sleep(Duration::from_millis(200)).await;
    let p0 = kinematics::fk(&kinematics::q6(&INSPECT)).p;
    let (id, _) = armd::place::start(&sup, &json!({"position": [p0[0] + 0.05, p0[1], p0[2] - 0.06]})).await.unwrap();
    let deadline = std::time::Instant::now() + Duration::from_secs(20);
    loop {
        assert!(std::time::Instant::now() < deadline, "approach not reached");
        let inner = sup.inner().read().await;
        let moving = inner.active_trajectory.as_ref().is_some_and(|t| t.action_id == Some(id));
        if inner.last_action.as_ref().unwrap().phase.as_deref() == Some("approach") && moving { break; }
        drop(inner);
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    arm.set_residual(0, 9.0);
    let deadline = std::time::Instant::now() + Duration::from_secs(3);
    let a = loop {
        assert!(std::time::Instant::now() < deadline, "place not ended by the trip");
        let obs = sup.get_observation().await;
        let a = obs.action.into_iter().chain(obs.previous_action).find(|a| a.id == id).unwrap();
        if !a.is_running() { break a; }
        tokio::time::sleep(Duration::from_millis(5)).await;
    };
    assert_eq!((a.state.as_str(), a.code.as_deref(), a.phase.as_deref()), ("failed", Some("COLLISION"), Some("approach")), "{a:?}");
    arm.clear();
    let q0 = sup.inner().read().await.commanded_positions.clone();
    tokio::time::sleep(Duration::from_millis(800)).await;
    let inner = sup.inner().read().await;
    assert!(inner.macro_run.is_none() && inner.active_trajectory.is_none(), "no phase after the trip");
    let moved = q0.iter().zip(&inner.commanded_positions).map(|(a, b)| (a - b).abs()).fold(0.0f32, f32::max);
    assert!(moved < 1e-3, "orchestrator issued no motion after the trip ({moved} rad)");
}

/// Real B601-DM at the folded home pose: J3 rests on its support with ~7.5 N·m residual
/// (J2 ~1.2). Supported joints (`resting_pose.support_torque_limits`) are not watched while the
/// arm is within the resting-pose tolerances; other joints, and the same joint off home, are.
#[test]
fn supported_joints_at_the_resting_pose_do_not_trip() {
    let (mut inner, arm) = rig(Some(policy()));
    let home: Vec<f32> = inner.config.home_positions.clone();
    arm.0.lock().unwrap().q.copy_from_slice(&home);
    inner.commanded_positions = home.clone();
    inner.target_positions = home.clone();
    ticks(&mut inner, 3);
    assert_eq!(inner.collision_supported_joints(), [false, true, true, false, false, false]);
    arm.set_residual(2, 7.5);
    arm.set_residual(1, 1.2);
    ticks(&mut inner, 20);
    assert!(inner.collision.hold.is_none(), "J3 on its support must not trip at home");
    arm.set_residual(0, 7.0);
    ticks(&mut inner, 5);
    assert_eq!(inner.collision.hold.map(|h| h.joint), Some(0), "J1 is not supported: trips");
    // off home the support exemption ends
    let (mut inner, arm) = rig(Some(policy()));
    arm.set_residual(2, 7.5);
    ticks(&mut inner, 5);
    assert_eq!(inner.collision.hold.map(|h| h.joint), Some(2), "J3 away from home trips");
}

/// The support exemption needs a tolerance for every arm joint: a profile with a short
/// tolerance vector is refused, and an unvalidated one never exempts (J6 far off home).
#[test]
fn short_resting_tolerances_are_refused_and_never_exempt() {
    let mut config = profile(Some(policy()));
    config.resting_pose.as_mut().unwrap().position_tolerances = vec![0.15, 0.08, 0.08];
    let err = config.clone().validate().unwrap_err();
    assert!(err.contains("position_tolerances needs a value for each of the 6 arm joints"), "{err}");
    let (mut inner, arm) = rig(Some(policy()));
    inner.config.resting_pose.as_mut().unwrap().position_tolerances = vec![0.15, 0.08, 0.08];
    let mut q: Vec<f32> = inner.config.home_positions.clone();
    q[5] += 1.0; // J6 far from home, J1..J5 at home
    arm.0.lock().unwrap().q.copy_from_slice(&q);
    inner.commanded_positions = q.clone();
    inner.target_positions = q.clone();
    ticks(&mut inner, 3);
    assert_eq!(inner.collision_supported_joints(), [false; 6]);
}
