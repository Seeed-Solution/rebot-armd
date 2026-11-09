//! Action executor and status (spec 2.2 / 2.3, tests 4.4).
mod common;

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
use armd::kinematics::{self, Pose};
use armd::model::{ArmConfig, JointState};
use armd::motion::{self, plan_cartesian, resolve_goal, GoalSpec, MotionError, RelFrame, Speed};
use armd::{ArmError, StopPolicy, SupervisorInner};

const DT: f64 = 0.02;
const INSPECT: [f32; 7] = [0.0, -1.167, -1.565, 1.435, 0.0, std::f32::consts::FRAC_PI_2, 0.0];

fn sim_at(q: &[f32; 7]) -> SupervisorInner {
    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
    inner.lease_enabled = false;
    inner.set_torque(true).unwrap();
    inner.commanded_positions = q.to_vec();
    inner.measured_positions = q.to_vec();
    inner.target_positions = q.to_vec();
    inner
}

fn tcp(q: &[f32]) -> Pose { kinematics::fk(&kinematics::q6(q)) }

fn start_move(inner: &mut SupervisorInner, spec: &GoalSpec, speed: Speed) -> Result<(u64, f64, Pose), ArmError> {
    let input = inner.plan_input()?;
    let goal = resolve_goal(&input, spec);
    let path = plan_cartesian(&input, goal, speed)?;
    let (id, duration) = inner.install_path(&input, path, "move_to", Some("t".into()))?;
    Ok((id, duration, goal))
}

fn run(inner: &mut SupervisorInner, id: u64, max_s: f64, mut each: impl FnMut(&SupervisorInner)) -> motion::ActionStatus {
    for _ in 0..(max_s / DT) as usize {
        inner.tick(DT);
        each(inner);
        let a = inner.last_action.clone().unwrap();
        if a.id == id && !a.is_running() { return a; }
    }
    panic!("action still running: {:?}", inner.last_action);
}

fn dist_to_segment(p: &[f64; 3], a: &[f64; 3], b: &[f64; 3]) -> f64 {
    let ab = kinematics::sub(b, a);
    let t = (kinematics::dot(&kinematics::sub(p, a), &ab) / kinematics::dot(&ab, &ab)).clamp(0.0, 1.0);
    let c = [a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t];
    kinematics::norm(&kinematics::sub(p, &c))
}

#[test]
fn move_to_runs_to_done_on_a_straight_line() {
    let mut inner = sim_at(&INSPECT);
    let p0 = tcp(&INSPECT).p;
    let goal_p = [p0[0] + 0.06, p0[1] + 0.05, p0[2] - 0.04];
    let (id, duration, goal) = start_move(&mut inner, &GoalSpec::Absolute { p: goal_p, r: None }, Speed::default()).unwrap();
    let a = inner.last_action.clone().unwrap();
    assert_eq!((a.id, a.kind.as_str(), a.state.as_str()), (id, "move_to", "running"));
    let length = kinematics::norm(&kinematics::sub(&goal_p, &p0));
    assert!(duration >= 1.875 * length / 0.05, "speed bound: {duration}");
    let mut worst = 0.0f64;
    let mut peak_v = 0.0f32;
    let a = run(&mut inner, id, duration + 4.0, |i| {
        worst = worst.max(dist_to_segment(&tcp(&i.commanded_positions).p, &p0, &goal_p));
        peak_v = i.commanded_velocities[..6].iter().fold(peak_v, |m, v| m.max(v.abs()));
        let obs = i.current_observation();
        assert_eq!(obs.action.as_ref().unwrap().id, id);
    });
    assert_eq!(a.state, "done", "{a:?}");
    assert_eq!(a.settled, Some(true));
    assert_eq!(a.progress, 1.0);
    let end = tcp(&inner.commanded_positions);
    let err_mm = kinematics::norm(&kinematics::sub(&end.p, &goal.p)) * 1000.0;
    println!("move_to: duration {duration:.2} s, end error {err_mm:.3} mm, max line deviation {:.3} mm, peak joint v {peak_v:.3}", worst * 1000.0);
    assert!(err_mm <= 1.0);
    assert!(worst <= 1e-3, "{worst}");
    assert!(peak_v <= 0.7 + 0.05, "{peak_v}");
    assert!(kinematics::angle_between(&end.r, &tcp(&INSPECT).r) < 0.5f64.to_radians(), "orientation kept");
    assert!(inner.active_trajectory.is_none() && !inner.safety.state().is_stopped());
}

#[test]
fn unreachable_goal_fails_planning_without_motion() {
    let inner = sim_at(&INSPECT);
    let input = inner.plan_input().unwrap();
    let p0 = tcp(&INSPECT).p;
    let err = plan_cartesian(&input, resolve_goal(&input, &GoalSpec::Absolute { p: [p0[0] + 0.5, p0[1], p0[2]], r: None }), Speed::default()).unwrap_err();
    assert_eq!(err.code(), "IK_UNREACHABLE", "{err}");
    assert!(err.to_string().contains("s="));
    assert!(inner.active_trajectory.is_none() && inner.last_action.is_none());
    let err = plan_cartesian(&input, resolve_goal(&input, &GoalSpec::Absolute { p: [5.0, 0.0, 0.0], r: None }), Speed::default()).unwrap_err();
    assert_eq!(err.code(), "INVALID_COMMAND", "path too long: {err}");
}

#[test]
fn path_into_the_floor_is_rejected_by_the_workspace_rule() {
    let inner = sim_at(&INSPECT);
    let input = inner.plan_input().unwrap();
    let down = |dz: f64| GoalSpec::Relative { frame: RelFrame::Base, d: [0.0, 0.0, dz], dyaw: 0.0, dpitch: 0.0, droll: 0.0, level: false };
    plan_cartesian(&input, resolve_goal(&input, &down(-0.24)), Speed::default()).expect("0.24 m down stays above the floor");
    let err = plan_cartesian(&input, resolve_goal(&input, &down(-0.29)), Speed::default()).unwrap_err();
    assert_eq!(err.code(), "WORKSPACE_VIOLATION", "{err}");
    assert!(err.to_string().contains("floor"), "{err}");
}

#[test]
fn camera_frame_forward_moves_along_the_optical_axis() {
    let inner = sim_at(&INSPECT);
    let input = inner.plan_input().unwrap();
    let start = tcp(&INSPECT);
    let rec = kinematics::cam_mount(false, 0.0, 0.0);
    let z_cam = kinematics::column(&kinematics::cam_rotation(&start, &rec), 2);
    let spec = GoalSpec::Relative { frame: RelFrame::Camera, d: [0.0, 0.0, 0.05], dyaw: 0.0, dpitch: 0.0, droll: 0.0, level: false };
    let goal = resolve_goal(&input, &spec);
    let d = kinematics::sub(&goal.p, &start.p);
    assert!((kinematics::dot(&d, &z_cam) / kinematics::norm(&d) - 1.0).abs() < 1e-9 && (kinematics::norm(&d) - 0.05).abs() < 1e-9);
    // look down: optical axis drops, position kept
    let spec = GoalSpec::Relative { frame: RelFrame::Camera, d: [0.0; 3], dyaw: 0.0, dpitch: 0.2, droll: 0.0, level: true };
    let goal = resolve_goal(&input, &spec);
    let z1 = kinematics::column(&kinematics::cam_rotation(&goal, &rec), 2);
    assert!(z1[2] < z_cam[2] && kinematics::norm(&kinematics::sub(&goal.p, &start.p)) < 1e-12);
    let x1 = kinematics::column(&kinematics::cam_rotation(&goal, &rec), 0);
    assert!(x1[2].abs() < 1e-9, "auto-level keeps x_cam horizontal");
    // look right: heading turns clockwise seen from above
    let spec = GoalSpec::Relative { frame: RelFrame::Camera, d: [0.0; 3], dyaw: 0.2, dpitch: 0.0, droll: 0.0, level: true };
    let z2 = kinematics::column(&kinematics::cam_rotation(&resolve_goal(&input, &spec), &rec), 2);
    assert!(z2[1].atan2(z2[0]) < z_cam[1].atan2(z_cam[0]));
    // base frame pitch: positive tips the tool down
    let spec = GoalSpec::Relative { frame: RelFrame::Base, d: [0.0; 3], dyaw: 0.0, dpitch: 0.1, droll: 0.0, level: false };
    let x_tool = kinematics::column(&resolve_goal(&input, &spec).r, 0);
    assert!(x_tool[2] < kinematics::column(&start.r, 0)[2]);
    // tool frame translation along tool x
    let spec = GoalSpec::Relative { frame: RelFrame::Tool, d: [0.02, 0.0, 0.0], dyaw: 0.0, dpitch: 0.0, droll: 0.0, level: false };
    let d = kinematics::sub(&resolve_goal(&input, &spec).p, &start.p);
    assert!((kinematics::dot(&d, &kinematics::column(&start.r, 0)) - 0.02).abs() < 1e-12);
}

#[test]
fn estop_stop_and_supersede_end_the_action() {
    let p = |q: &[f32; 7]| { let p0 = tcp(q).p; [p0[0] + 0.05, p0[1], p0[2]] };
    // E-stop mid-path
    let mut inner = sim_at(&INSPECT);
    let (id, _, _) = start_move(&mut inner, &GoalSpec::Absolute { p: p(&INSPECT), r: None }, Speed::default()).unwrap();
    for _ in 0..20 { inner.tick(DT); }
    inner.apply_stop_policy(StopPolicy::EStopLatch);
    inner.tick(DT);
    let a = inner.last_action.clone().unwrap();
    assert_eq!((a.id, a.state.as_str(), a.code.as_deref()), (id, "failed", Some("ESTOP_ACTIVE")));
    // stop
    let mut inner = sim_at(&INSPECT);
    let (id, _, _) = start_move(&mut inner, &GoalSpec::Absolute { p: p(&INSPECT), r: None }, Speed::default()).unwrap();
    for _ in 0..20 { inner.tick(DT); }
    inner.stop_motion();
    inner.tick(DT);
    let a = inner.last_action.clone().unwrap();
    assert_eq!((a.id, a.state.as_str(), a.code.as_deref()), (id, "cancelled", Some("STOPPED")));
    // a jog replaces the action
    let mut inner = sim_at(&INSPECT);
    let (id, _, _) = start_move(&mut inner, &GoalSpec::Absolute { p: p(&INSPECT), r: None }, Speed::default()).unwrap();
    for _ in 0..20 { inner.tick(DT); }
    inner.jog(INSPECT.to_vec(), None, None).expect("jog over an action is allowed");
    let obs = inner.tick(DT);
    let prev = obs.previous_action.expect("replaced action stays visible");
    assert_eq!((prev.id, prev.state.as_str(), prev.code.as_deref()), (id, "cancelled", Some("SUPERSEDED")));
    assert_eq!(obs.action.unwrap().kind, "joint_move");
    // lease released
    let mut inner = sim_at(&INSPECT);
    let generation = inner.acquire_lease("conn").unwrap();
    let (id, _, _) = start_move(&mut inner, &GoalSpec::Absolute { p: p(&INSPECT), r: None }, Speed::default()).unwrap();
    for _ in 0..10 { inner.tick(DT); }
    assert!(inner.release_lease("conn", generation));
    inner.stop_motion();
    inner.tick(DT);
    let a = inner.last_action.clone().unwrap();
    assert_eq!((a.id, a.state.as_str(), a.code.as_deref()), (id, "cancelled", Some("LEASE_LOST")));
}

#[test]
fn busy_and_stale_preconditions() {
    let mut inner = sim_at(&INSPECT);
    let epoch = inner.safety.epoch_id();
    let mut lead = INSPECT.to_vec();
    lead[0] += 0.1;
    inner.servo(lead, epoch).unwrap();
    let err = inner.plan_input().unwrap_err();
    assert!(matches!(err, ArmError::Motion(MotionError::MotionBusy(_))), "{err:?}");
    // a jog between plan and install makes the plan stale
    let mut inner = sim_at(&INSPECT);
    let input = inner.plan_input().unwrap();
    let p0 = tcp(&INSPECT).p;
    let path = plan_cartesian(&input, resolve_goal(&input, &GoalSpec::Absolute { p: [p0[0] + 0.03, p0[1], p0[2]], r: None }), Speed::default()).unwrap();
    let mut jog_target = INSPECT.to_vec();
    jog_target[0] = 0.2;
    inner.jog(jog_target, None, None).unwrap();
    for _ in 0..400 { inner.tick(DT); }
    let err = inner.install_path(&input, path.clone(), "move_to", None).unwrap_err();
    assert!(matches!(err, ArmError::Motion(MotionError::PlanStale(_))), "{err:?}");
    // epoch change
    let mut inner = sim_at(&INSPECT);
    let input = inner.plan_input().unwrap();
    inner.apply_stop_policy(StopPolicy::EStopLatch);
    inner.clear_fault().unwrap();
    inner.set_torque(true).unwrap();
    assert!(matches!(inner.install_path(&input, path, "move_to", None), Err(ArmError::Motion(MotionError::PlanStale(_)))));
    // no kinematics
    let mut config = ArmConfig::rebot_dm();
    config.kinematics = None;
    let mut inner = SupervisorInner::new(config);
    inner.set_torque(true).unwrap();
    assert!(matches!(inner.plan_input(), Err(ArmError::Motion(MotionError::Unsupported(_)))));
}

#[test]
fn joint_move_home_and_park_create_action_records() {
    let mut inner = sim_at(&INSPECT);
    let mut target = INSPECT.to_vec();
    target[0] = 0.3;
    let (_, id, duration) = inner.jog_action(target, None, None, "joint_move").unwrap();
    let a = run(&mut inner, id, duration + 3.0, |_| {});
    assert_eq!((a.kind.as_str(), a.state.as_str(), a.settled), ("joint_move", "done", Some(true)));
    let home = inner.config.home_positions.clone();
    let (_, id, duration) = inner.jog_action(home, None, None, "home").unwrap();
    let a = run(&mut inner, id, duration + 3.0, |_| {});
    assert_eq!((a.kind.as_str(), a.state.as_str()), ("home", "done"));
    inner.commanded_positions = INSPECT.to_vec();
    inner.measured_positions = INSPECT.to_vec();
    let id = inner.unload_action().expect("park starts");
    let a = run(&mut inner, id, 15.0, |_| {});
    assert_eq!((a.kind.as_str(), a.state.as_str()), ("park", "done"), "{a:?}");
    assert!(!inner.is_torqued);
}

/// Arm whose joints do not move at all (blocked), with fresh feedback.
struct Blocked { q: Vec<f32>, enabled: EnableState }
struct BlockedDriver(Arc<Mutex<Blocked>>);
impl ArmDriver for BlockedDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true, torque_feedforward: false, enable_feedback: true }
    }
    fn enable_state(&self) -> EnableState { self.0.lock().unwrap().enabled }
    fn is_feedback_valid(&self) -> bool { true }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); 7] }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        Ok(self.0.lock().unwrap().q.iter().map(|&position| JointState { position, ..JointState::default() }).collect())
    }
    fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
    fn set_torque(&mut self, on: bool) -> Result<(), ArmDriverError> {
        self.0.lock().unwrap().enabled = if on { EnableState::Enabled } else { EnableState::Disabled };
        Ok(())
    }
}

#[test]
fn blocked_arm_fails_tracking_stall_without_a_fault() {
    let hw = Arc::new(Mutex::new(Blocked { q: INSPECT.to_vec(), enabled: EnableState::Disabled }));
    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(BlockedDriver(hw)));
    inner.lease_enabled = false;
    inner.tick(DT);
    inner.set_torque(true).unwrap();
    let p0 = tcp(&INSPECT).p;
    let spec = GoalSpec::Absolute { p: [p0[0], p0[1] + 0.2, p0[2]], r: None }; // J1 turns ~0.7 rad
    let (id, duration, _) = start_move(&mut inner, &spec, Speed { linear: 0.15, angular: 1.0 }).unwrap();
    let started = Instant::now();
    let mut worst_lag = 0.0f32;
    let a = run(&mut inner, id, duration + 5.0, |i| {
        // real time must cover the 500 ms tracking-fault persistence for this test to mean anything
        std::thread::sleep(Duration::from_millis(2));
        worst_lag = (0..6).map(|k| (i.commanded_positions[k] - i.measured_positions[k]).abs()).fold(worst_lag, f32::max);
    });
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some("TRACKING_STALL")), "{a:?}");
    assert!(!inner.safety.state().is_stopped(), "no Fault latched: {:?}", inner.safety.state());
    assert!(worst_lag < 0.5, "lag-aware scaling keeps the error under the fault bound: {worst_lag}");
    assert!(started.elapsed() > Duration::from_millis(500));
    assert_eq!(inner.commanded_positions, inner.measured_positions, "commanded reset to measured");
}

/// Planning time on this machine. On the device:
/// cargo test --release --test test_actions bench_plan -- --ignored --nocapture
#[test]
#[ignore]
fn bench_plan() {
    let inner = sim_at(&INSPECT);
    let input = inner.plan_input().unwrap();
    let p0 = tcp(&INSPECT).p;
    for (name, spec) in [
        ("0.05 m", GoalSpec::Absolute { p: [p0[0] + 0.05, p0[1], p0[2]], r: None }),
        ("0.20 m", GoalSpec::Absolute { p: [p0[0] + 0.08, p0[1] + 0.15, p0[2] - 0.08], r: None }),
        ("45 deg look", GoalSpec::Relative { frame: RelFrame::Camera, d: [0.0; 3], dyaw: 0.785, dpitch: 0.0, droll: 0.0, level: true }),
    ] {
        let goal = resolve_goal(&input, &spec);
        let runs = 10;
        let t = Instant::now();
        let mut n = 0;
        for _ in 0..runs { n = plan_cartesian(&input, goal, Speed::default()).unwrap().q.len(); }
        println!("bench_plan {name}: {n} samples, {:.2} ms per plan", t.elapsed().as_secs_f64() * 1000.0 / runs as f64);
    }
}
