//! Gripper action executor (spec 2.3 GripperMotion, tests 4.5).
use std::sync::{Arc, Mutex};
use std::time::Duration;
use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
use armd::model::{ArmConfig, GripperPolicy, JointState};
use armd::motion::{GripperCommand, GripperRequest};
use armd::SupervisorInner;

const DT: f64 = 0.02;
const INSPECT: [f32; 7] = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0, -5.0];

/// First-order lag arm; J7 is blocked by an object (or the closed stop) at `block`,
/// producing torque `stiffness * (cmd - q)` while pressed.
/// `stiffness`: N·m per rad of command past the block; `tau_sign`: sign the driver reports J7 torque with.
struct Rig { q: [f32; 7], v: [f32; 7], tau: [f32; 7], block: Option<f32>, frozen_j7: bool, open_stop: Option<f32>, enabled: EnableState, stiffness: f32, tau_sign: f32 }
struct RigDriver(Arc<Mutex<Rig>>);
const STIFFNESS: f32 = 3.0;
impl ArmDriver for RigDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true, torque_feedforward: false, enable_feedback: true, load_feedback: false }
    }
    fn enable_state(&self) -> EnableState { self.0.lock().unwrap().enabled }
    fn is_feedback_valid(&self) -> bool { true }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); 7] }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let r = self.0.lock().unwrap();
        Ok((0..7).map(|i| JointState { position: r.q[i], velocity: r.v[i], torque: if i == 6 { r.tau_sign * r.tau[i] } else { r.tau[i] }, temperature: 30.0 }).collect())
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        let mut r = self.0.lock().unwrap();
        for (i, t) in targets.iter().enumerate().take(7) {
            if i == 6 && r.frozen_j7 { r.v[6] = 0.0; r.tau[6] = 0.0; continue; }
            let v = ((t.position - r.q[i]) * 2.2).clamp(-2.0, 2.0);
            let mut q = r.q[i] + v * DT as f32;
            let mut vv = v;
            r.tau[i] = 0.0;
            if i == 6 {
                if let Some(o) = r.open_stop { if q <= o { q = o; vv = 0.0; } }
                if let Some(b) = r.block {
                    if q >= b { q = b; vv = 0.0; r.tau[6] = (r.stiffness * (t.position - b)).max(0.0); }
                }
            }
            r.q[i] = q;
            r.v[i] = vv;
        }
        Ok(())
    }
    fn set_torque(&mut self, on: bool) -> Result<(), ArmDriverError> {
        self.0.lock().unwrap().enabled = if on { EnableState::Enabled } else { EnableState::Disabled };
        Ok(())
    }
}

fn rig(j7: f32, block: Option<f32>, frozen_j7: bool) -> (SupervisorInner, Arc<Mutex<Rig>>) {
    let mut q = INSPECT;
    q[6] = j7;
    let rig = Arc::new(Mutex::new(Rig { q, v: [0.0; 7], tau: [0.0; 7], block, frozen_j7, open_stop: None, enabled: EnableState::Disabled, stiffness: STIFFNESS, tau_sign: 1.0 }));
    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(RigDriver(rig.clone())));
    inner.lease_enabled = false;
    inner.tick(DT);
    inner.set_torque(true).unwrap();
    (inner, rig)
}

fn request(command: GripperCommand, fraction: Option<f64>, grasp: bool, timeout_s: f64) -> GripperRequest {
    GripperRequest { command, fraction, grasp, speed: 1.0, timeout_s: Some(timeout_s), tag: Some("t".into()) }
}

fn run(inner: &mut SupervisorInner, id: u64, max_s: f64) -> armd::motion::ActionStatus {
    for _ in 0..(max_s / DT) as usize {
        inner.tick(DT);
        let a = inner.last_action.clone().unwrap();
        assert_eq!(a.id, id);
        if !a.is_running() { return a; }
    }
    panic!("action still running: {:?}", inner.last_action);
}

#[test]
fn close_on_an_object_holds_with_squeeze() {
    let (mut inner, _rig) = rig(-5.0, Some(-2.0), false);
    let epoch = inner.safety.epoch_id();
    let (id, expected) = inner.start_gripper(request(GripperCommand::Close, None, true, 8.0), epoch).unwrap();
    assert!(expected > 3.0 && expected < 7.0, "{expected}");
    let a = run(&mut inner, id, 8.0);
    assert_eq!((a.state.as_str(), a.kind.as_str(), a.tag.as_deref()), ("done", "gripper", Some("t")));
    assert_eq!(a.holding, Some(true), "{a:?}");
    assert!((a.gripper_gap_rad.unwrap() - 2.0).abs() < 0.02, "{a:?}");
    let squeeze = GripperPolicy::rebot_dm().squeeze_rad;
    assert!((inner.commanded_positions[6] - (-2.0 + squeeze)).abs() < 1e-3, "J7 hold {}", inner.commanded_positions[6]);
    assert!(inner.active_trajectory.is_none());
    // the hold target persists after the action
    for _ in 0..20 { inner.tick(DT); }
    assert!((inner.commanded_positions[6] - (-2.0 + squeeze)).abs() < 1e-3);
    assert!(!inner.safety.state().is_stopped());
}

#[test]
fn close_without_an_object_is_not_holding() {
    let (mut inner, _rig) = rig(-3.0, Some(-0.05), false);
    let epoch = inner.safety.epoch_id();
    let (id, _) = inner.start_gripper(request(GripperCommand::Close, None, true, 8.0), epoch).unwrap();
    let a = run(&mut inner, id, 8.0);
    assert_eq!(a.state, "done");
    assert_eq!(a.holding, Some(false), "{a:?}");
    assert!(a.gripper_gap_rad.unwrap() < 0.1);
    assert!(inner.commanded_positions[6] <= 0.0, "hold never passes closed_rad");
}

#[test]
fn jammed_jaw_times_out_and_holds_at_measured() {
    let (mut inner, _rig) = rig(0.0, None, true);
    let epoch = inner.safety.epoch_id();
    let (id, _) = inner.start_gripper(request(GripperCommand::Open, None, false, 1.0), epoch).unwrap();
    let a = run(&mut inner, id, 3.0);
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some("GRIPPER_TIMEOUT")), "{a:?}");
    assert!((inner.commanded_positions[6] - inner.measured_positions[6]).abs() < 1e-6);
    assert!(inner.active_trajectory.is_none() && !inner.safety.state().is_stopped());
}

#[test]
fn open_after_a_stall_clears_the_stall_and_reaches_open() {
    let (mut inner, _rig) = rig(-2.0, None, false);
    inner.gripper_stalled = true;
    inner.gripper_stall_sign = 1.0; // stalled while closing
    let epoch = inner.safety.epoch_id();
    let (id, _) = inner.start_gripper(request(GripperCommand::Open, None, false, 8.0), epoch).unwrap();
    inner.tick(DT);
    assert!(!inner.gripper_stalled, "reverse direction releases the stall");
    let a = run(&mut inner, id, 8.0);
    assert_eq!(a.state, "done", "{a:?}");
    assert!((inner.measured_positions[6] - GripperPolicy::rebot_dm().open_rad).abs() < 0.05);
}

#[test]
fn move_to_fraction_and_arm_joints_hold() {
    let (mut inner, _rig) = rig(-5.0, None, false);
    let before: Vec<f32> = inner.commanded_positions[..6].to_vec();
    let epoch = inner.safety.epoch_id();
    let (id, _) = inner.start_gripper(request(GripperCommand::Move, Some(0.5), false, 8.0), epoch).unwrap();
    // servo frames are rejected while the action runs
    assert!(matches!(inner.servo(inner.commanded_positions.clone(), epoch), Err(armd::SafetyError::MotionBusy)));
    let a = run(&mut inner, id, 8.0);
    assert_eq!(a.state, "done");
    assert!((inner.measured_positions[6] - (-2.85)).abs() < 0.05, "{}", inner.measured_positions[6]);
    assert_eq!(&inner.commanded_positions[..6], before.as_slice());
}

#[test]
fn gripper_action_needs_a_policy_and_an_idle_arm() {
    let mut config = ArmConfig::rebot_dm();
    config.gripper_policy = None;
    let mut inner = SupervisorInner::new(config);
    inner.lease_enabled = false;
    inner.set_torque(true).unwrap();
    let epoch = inner.safety.epoch_id();
    let err = inner.start_gripper(request(GripperCommand::Open, None, false, 6.0), epoch).unwrap_err();
    assert!(matches!(err, armd::ArmError::Motion(armd::motion::MotionError::Unsupported(_))), "{err:?}");
    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
    let err = inner.start_gripper(request(GripperCommand::Open, None, false, 6.0), inner.safety.epoch_id()).unwrap_err();
    assert!(matches!(err, armd::ArmError::Safety(armd::SafetyError::NotArmed(_))), "torque off: {err:?}");
}

#[test]
fn open_that_stops_short_of_the_target_is_done() {
    // real arm 2026-11-09: commanded -5.70, the jaw stopped at -5.611
    let (mut inner, rig) = rig(-1.0, None, false);
    rig.lock().unwrap().open_stop = Some(-5.611);
    let epoch = inner.safety.epoch_id();
    let (id, _) = inner.start_gripper(request(GripperCommand::Open, None, false, 10.0), epoch).unwrap();
    let a = run(&mut inner, id, 12.0);
    assert_eq!(a.state, "done", "{a:?}");
    assert!((inner.measured_positions[6] + 5.611).abs() < 1e-3);
}

#[test]
fn open_blocked_far_from_the_target_still_times_out() {
    let (mut inner, rig) = rig(-1.0, None, false);
    rig.lock().unwrap().open_stop = Some(-4.0);
    let epoch = inner.safety.epoch_id();
    let (id, _) = inner.start_gripper(request(GripperCommand::Open, None, false, 8.0), epoch).unwrap();
    let a = run(&mut inner, id, 10.0);
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some("GRIPPER_TIMEOUT")), "{a:?}");
}

/// Grasp with the stall latch tripped by the squeeze, carry, then open (sim 2026-11-11: the open
/// reported done after 0.14 s with J7 still in the grasp). Both torque signs; a soft object where
/// the latch is only set before the open, and a stiff one where the push during the unwind from
/// the squeeze hold sets it again.
#[test]
fn open_after_a_latched_grasp_travels_to_open() {
    let open_rad = GripperPolicy::rebot_dm().open_rad;
    for (stiffness, tau_sign) in [(STIFFNESS, 1.0), (STIFFNESS, -1.0), (10.0, 1.0), (10.0, -1.0)] {
        let case = format!("stiffness {stiffness}, torque sign {tau_sign}");
        let (mut inner, rig) = rig(-5.0, Some(-2.0), false);
        { let mut r = rig.lock().unwrap(); r.stiffness = stiffness; r.tau_sign = tau_sign; }
        let epoch = inner.safety.epoch_id();
        let (id, _) = inner.start_gripper(request(GripperCommand::Close, None, true, 8.0), epoch).unwrap();
        let a = run(&mut inner, id, 8.0);
        assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{case}: {a:?}");
        // the arm holds the object for a while; the squeeze keeps the latch set
        for _ in 0..50 { inner.tick(DT); }
        assert!(inner.gripper_stalled, "{case}: squeeze sets the stall latch");
        assert_eq!(inner.gripper_stall_sign, 1.0, "{case}: latch direction is closing");
        // the object stays between the jaws; the block only stops travel toward closed
        let (id, _) = inner.start_gripper(request(GripperCommand::Open, None, false, 10.0), epoch).unwrap();
        let mut ticks = 0;
        let a = loop {
            inner.tick(DT);
            ticks += 1;
            let a = inner.last_action.clone().unwrap();
            assert_eq!(a.id, id);
            if !a.is_running() { break a; }
            assert!(ticks < 600, "{case}: open still running");
        };
        assert_eq!(a.state, "done", "{case}: {a:?}");
        assert!(ticks as f64 * DT > 3.0, "{case}: done after {:.2} s", ticks as f64 * DT);
        assert!((inner.measured_positions[6] - open_rad).abs() < 0.05, "{case}: J7 {}", inner.measured_positions[6]);
        assert!((a.gripper_gap_rad.unwrap() + open_rad).abs() < 0.05, "{case}: {a:?}");
        assert!(!inner.gripper_stalled, "{case}");
    }
}

/// A stall in the travel direction still holds J7 (motor protection), whatever the torque sign.
#[test]
fn close_into_an_obstacle_latches_and_holds_at_measured() {
    for tau_sign in [1.0, -1.0] {
        let (mut inner, rig) = rig(-5.0, Some(-2.0), false);
        { let mut r = rig.lock().unwrap(); r.stiffness = 10.0; r.tau_sign = tau_sign; }
        let epoch = inner.safety.epoch_id();
        let (id, _) = inner.start_gripper(request(GripperCommand::Close, None, false, 8.0), epoch).unwrap();
        let a = run(&mut inner, id, 8.0);
        assert_eq!(a.state, "done", "torque sign {tau_sign}: {a:?}");
        assert!(inner.gripper_stalled && inner.gripper_stall_sign == 1.0, "torque sign {tau_sign}");
        assert!((inner.commanded_positions[6] - inner.measured_positions[6]).abs() < 1e-3,
            "torque sign {tau_sign}: J7 command {} held at measured {}", inner.commanded_positions[6], inner.measured_positions[6]);
        assert!((inner.measured_positions[6] + 2.0).abs() < 1e-3);
    }
}

#[test]
fn default_timeout_covers_a_full_stroke() {
    let req = armd::motion::parse_gripper(&serde_json::json!({"command": "open"})).unwrap();
    assert_eq!((req.speed, req.timeout_s), (1.0, None));
    let (mut inner, _rig) = rig(0.0, None, false);
    let epoch = inner.safety.epoch_id();
    let (id, expected) = inner.start_gripper(req, epoch).unwrap();
    assert!(expected > 6.0, "{expected}");
    let a = run(&mut inner, id, 12.0);
    assert_eq!(a.state, "done", "{a:?}");
    assert!((inner.measured_positions[6] - GripperPolicy::rebot_dm().open_rad).abs() < 0.05);
    // an explicit timeout_s still applies as given
    let req = armd::motion::parse_gripper(&serde_json::json!({"command": "close", "grasp": false, "timeout_s": 3.0})).unwrap();
    let (id, _) = inner.start_gripper(req, epoch).unwrap();
    let a = run(&mut inner, id, 5.0);
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some("GRIPPER_TIMEOUT")), "{a:?}");
}


/// The deadline covers the squeeze: a grasp whose squeeze would end after timeout_s fails
/// GRIPPER_TIMEOUT at the deadline, keeping the squeeze hold (the grip stays).
#[test]
fn deadline_covers_the_squeeze() {
    // contact at -2.0 after about 3.4 s from -5.0 at 1 rad/s: find when the squeeze starts
    let (mut inner, _rig) = rig(-5.0, Some(-2.0), false);
    let epoch = inner.safety.epoch_id();
    let (id, _) = inner.start_gripper(request(GripperCommand::Close, None, true, 8.0), epoch).unwrap();
    let mut done_at = 0.0;
    for k in 0..400 {
        inner.tick(DT);
        if !inner.last_action.as_ref().unwrap().is_running() { done_at = (k + 1) as f64 * DT; break; }
    }
    assert!(done_at > 1.0, "{done_at}");
    // the same grasp with a deadline 0.1 s before it would have finished (inside the squeeze)
    let (mut inner, _rig) = rig(-5.0, Some(-2.0), false);
    let epoch = inner.safety.epoch_id();
    let timeout = done_at - 0.1;
    let (id2, _) = inner.start_gripper(request(GripperCommand::Close, None, true, timeout), epoch).unwrap();
    let mut t = 0.0;
    let a = loop {
        inner.tick(DT);
        t += DT;
        let a = inner.last_action.clone().unwrap();
        if !a.is_running() { break a; }
        assert!(t < done_at + 1.0);
    };
    eprintln!("full grasp done after {done_at:.2} s; with timeout {timeout:.2} s: {} {:?} after {t:.2} s, J7 command {:.3}", a.state, a.code, inner.commanded_positions[6]);
    assert_eq!((a.id, a.state.as_str(), a.code.as_deref()), (id2, "failed", Some("GRIPPER_TIMEOUT")), "{a:?}");
    assert!(t <= timeout + 2.0 * DT + 1e-9, "{t} > {timeout}");
    assert!(inner.commanded_positions[6] > -2.0, "squeeze hold kept: {}", inner.commanded_positions[6]);
    let _ = id;
}
