//! Gripper action executor (spec 2.3 GripperMotion, tests 4.5).
use std::sync::{Arc, Mutex};
use std::time::Duration;
use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
use armd::model::{ArmConfig, GripperPolicy, JointState};
use armd::motion::{GripperCommand, GripperRequest};
use armd::SupervisorInner;

const DT: f64 = 0.02;
const INSPECT: [f32; 7] = [0.0, -1.167, -1.565, 1.435, 0.0, std::f32::consts::FRAC_PI_2, -5.0];

/// First-order lag arm; J7 is blocked by an object (or the closed stop) at `block`,
/// producing torque `stiffness * (cmd - q)` while pressed.
struct Rig { q: [f32; 7], v: [f32; 7], tau: [f32; 7], block: Option<f32>, frozen_j7: bool, enabled: EnableState }
struct RigDriver(Arc<Mutex<Rig>>);
const STIFFNESS: f32 = 3.0;
impl ArmDriver for RigDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true, torque_feedforward: false, enable_feedback: true }
    }
    fn enable_state(&self) -> EnableState { self.0.lock().unwrap().enabled }
    fn is_feedback_valid(&self) -> bool { true }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); 7] }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let r = self.0.lock().unwrap();
        Ok((0..7).map(|i| JointState { position: r.q[i], velocity: r.v[i], torque: r.tau[i], temperature: 30.0 }).collect())
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
                if let Some(b) = r.block {
                    if q >= b { q = b; vv = 0.0; r.tau[6] = (STIFFNESS * (t.position - b)).max(0.0); }
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
    let rig = Arc::new(Mutex::new(Rig { q, v: [0.0; 7], tau: [0.0; 7], block, frozen_j7, enabled: EnableState::Disabled }));
    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(RigDriver(rig.clone())));
    inner.lease_enabled = false;
    inner.tick(DT);
    inner.set_torque(true).unwrap();
    (inner, rig)
}

fn request(command: GripperCommand, fraction: Option<f64>, grasp: bool, timeout_s: f64) -> GripperRequest {
    GripperRequest { command, fraction, grasp, speed: 1.0, timeout_s, tag: Some("t".into()) }
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
