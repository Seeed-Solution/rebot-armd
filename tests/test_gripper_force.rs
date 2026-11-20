//! Gripper force protection (T23). Office B601-DM 2026-11-20 (rc10): a grasp of a toy banana
//! drove J7 on a ramp to closed; from contact at about -1.75 rad the J7 torque rose 0.36 ->
//! 0.85 -> 1.8 -> 3.1 -> 5.0 N·m while the jaw kept moving at ~1 rad/s, then a finger broke
//! (torque fell to 0.26 N·m) and the jaw ran to closed. The rig below replays that profile.
use std::sync::{Arc, Mutex};
use std::time::Duration;
use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
use armd::model::{ArmConfig, GripperPolicy, JointState, MotionLimitReason};
use armd::motion::{GripperCommand, GripperRequest};
use armd::SupervisorInner;

const DT: f64 = 0.02;
const INSPECT: [f32; 7] = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0, -5.7];
/// Incident: first contact sample.
const CONTACT_RAD: f32 = -1.75;
const CONTACT_NM: f32 = 0.85;
/// Incident: 5.0 N·m at -0.50 rad, then the finger broke.
const BREAK_NM: f32 = 5.0;
const RISE_NM_PER_RAD: f32 = (BREAK_NM - CONTACT_NM) / (-0.50 - CONTACT_RAD);
/// Incident: free close at ~1 rad/s reads 0.30-0.45 N·m.
const FREE_NM: f32 = 0.40;

/// What J7 meets while closing.
#[derive(Clone, Copy, PartialEq)]
enum Load {
    /// Nothing: the jaw runs to closed with free-motion torque.
    Free,
    /// The incident: a compliant object (or a flexing finger) past CONTACT_RAD; the jaw keeps
    /// its speed and the torque follows the position, elastic (it stays when the jaw stops).
    Incident,
    /// A rigid object at `.0`: the jaw stops there; torque `stiffness x (command - block)`.
    Rigid(f32, f32),
}

struct Rig { q: [f32; 7], v: [f32; 7], tau: [f32; 7], enabled: EnableState, load: Load, broke: bool, peak_tau: f32, max_lead: f32, max_cmd: f32 }
struct RigDriver(Arc<Mutex<Rig>>);

impl ArmDriver for RigDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true, torque_feedforward: false, enable_feedback: true, load_feedback: false }
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
            // POS_VEL-like follower: ~0.23 rad lag at 1 rad/s, capped at 1.05 rad/s (incident log)
            let mut v = ((t.position - r.q[i]) * 4.5).clamp(-1.05, 1.05);
            let mut q = r.q[i] + v * DT as f32;
            if i != 6 { r.q[i] = q; r.v[i] = v; continue; }
            if q > 0.0 { q = 0.0; v = 0.0; } // closed stop
            r.max_lead = r.max_lead.max((t.position - r.q[6]).abs());
            r.max_cmd = r.max_cmd.max(t.position);
            let tau = match r.load {
                Load::Free => if v.abs() > 0.05 { FREE_NM * v.signum() } else { 0.0 },
                Load::Incident if q >= CONTACT_RAD => CONTACT_NM + (q - CONTACT_RAD) * RISE_NM_PER_RAD,
                Load::Incident => if v.abs() > 0.05 { FREE_NM * v.signum() } else { 0.0 },
                Load::Rigid(block, stiffness) => {
                    if q >= block { q = block; v = 0.0; (stiffness * (t.position - block)).max(0.0) }
                    else if v.abs() > 0.05 { FREE_NM * v.signum() } else { 0.0 }
                }
            };
            r.q[6] = q;
            r.v[6] = v;
            r.tau[6] = tau;
            r.peak_tau = r.peak_tau.max(tau);
            if r.load == Load::Incident && tau >= BREAK_NM {
                // the finger breaks: the torque falls, the jaw runs on
                r.broke = true;
                r.load = Load::Free;
            }
        }
        Ok(())
    }
    fn set_torque(&mut self, on: bool) -> Result<(), ArmDriverError> {
        self.0.lock().unwrap().enabled = if on { EnableState::Enabled } else { EnableState::Disabled };
        Ok(())
    }
}

fn rig_with(config: ArmConfig, load: Load) -> (SupervisorInner, Arc<Mutex<Rig>>) {
    let rig = Arc::new(Mutex::new(Rig { q: INSPECT, v: [0.0; 7], tau: [0.0; 7], enabled: EnableState::Disabled, load, broke: false, peak_tau: 0.0, max_lead: 0.0, max_cmd: f32::NEG_INFINITY }));
    let mut inner = SupervisorInner::new(config).with_driver(Box::new(RigDriver(rig.clone())));
    inner.lease_enabled = false;
    inner.tick(DT);
    inner.set_torque(true).unwrap();
    (inner, rig)
}

fn rig(load: Load) -> (SupervisorInner, Arc<Mutex<Rig>>) { rig_with(ArmConfig::rebot_dm(), load) }

fn reset_stats(rig: &Arc<Mutex<Rig>>) {
    let mut r = rig.lock().unwrap();
    r.peak_tau = 0.0;
    r.max_lead = 0.0;
    r.max_cmd = f32::NEG_INFINITY;
}

fn close(grasp: bool) -> GripperRequest {
    GripperRequest { command: GripperCommand::Close, fraction: None, grasp, speed: 1.0, timeout_s: Some(10.0), tag: None }
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

fn cap() -> f32 { GripperPolicy::rebot_dm().max_close_torque_nm.unwrap() }

/// Margin above the cap: the latch needs GRIPPER_FORCE_CAP_TICKS ticks and the rig's torque
/// rises 0.07 N·m per tick at 1.05 rad/s, plus one tick of feedback delay.
const CAP_MARGIN_NM: f32 = 0.25;

/// Without the cap, the incident profile breaks the finger (the rig reproduces the incident).
#[test]
fn incident_profile_breaks_the_finger_without_the_cap() {
    let mut config = ArmConfig::rebot_dm();
    if let Some(p) = config.gripper_policy.as_mut() { p.max_close_torque_nm = None; }
    let (mut inner, rig) = rig_with(config, Load::Incident);
    let epoch = inner.safety.epoch_id();
    let (id, _) = inner.start_gripper(close(true), epoch).unwrap();
    let a = run(&mut inner, id, 12.0);
    let r = rig.lock().unwrap();
    eprintln!("no cap: peak {:.3} N·m, broke {}, holding {:?}", r.peak_tau, r.broke, a.holding);
    assert!(r.broke, "peak {:.2} N·m", r.peak_tau);
    assert_eq!(a.holding, Some(false), "{a:?}");
}

#[test]
fn incident_grasp_latches_at_the_cap_and_holds() {
    let (mut inner, rig) = rig(Load::Incident);
    let epoch = inner.safety.epoch_id();
    let (id, _) = inner.start_gripper(close(true), epoch).unwrap();
    let a = run(&mut inner, id, 12.0);
    let (peak, broke) = { let r = rig.lock().unwrap(); (r.peak_tau, r.broke) };
    eprintln!("incident grasp: peak {peak:.3} N·m, J7 {:.3} rad, holding {:?}", inner.measured_positions[6], a.holding);
    assert!(!broke, "finger broke");
    assert!(peak < cap() + CAP_MARGIN_NM, "peak {peak:.3} N·m");
    assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{a:?}");
    assert!(inner.gripper_stalled, "latched");
    assert!(inner.gripper_force_trips >= 1);
    // it holds: the jaw does not creep on over the next seconds
    let m = inner.measured_positions[6];
    for _ in 0..150 { inner.tick(DT); }
    assert!((inner.measured_positions[6] - m).abs() < 0.02, "J7 {} -> {}", m, inner.measured_positions[6]);
    assert!(rig.lock().unwrap().peak_tau < cap() + CAP_MARGIN_NM);
    assert!(!rig.lock().unwrap().broke);
}

/// A plain close (grasp false) through the incident profile ends at the cap.
#[test]
fn incident_close_without_grasp_detection_stops_at_the_cap() {
    let (mut inner, rig) = rig(Load::Incident);
    let epoch = inner.safety.epoch_id();
    let (id, _) = inner.start_gripper(close(false), epoch).unwrap();
    let a = run(&mut inner, id, 12.0);
    assert_eq!(a.state, "done", "{a:?}");
    { let r = rig.lock().unwrap(); assert!(!r.broke && r.peak_tau < cap() + CAP_MARGIN_NM, "peak {:.3} N·m", r.peak_tau); }
    let obs = inner.current_observation();
    let g = obs.gripper.unwrap();
    assert!(g.stalled && g.force_limited && g.force_limit_trips >= 1, "{g:?}");
    assert_eq!(obs.motion_limit.map(|l| (l.reason, l.joint)), Some((MotionLimitReason::GripperForce, Some(6))));
}

/// Teleop / policy stream: servo targets at closed through the incident profile.
#[test]
fn incident_servo_stream_is_capped() {
    let (mut inner, rig) = rig(Load::Incident);
    let epoch = inner.safety.epoch_id();
    let mut target = INSPECT.to_vec();
    // leader arm ramp toward closed, 1 rad/s, then held at closed
    let mut g = INSPECT[6];
    for _ in 0..(8.0 / DT) as usize {
        g = (g + DT as f32).min(0.0);
        target[6] = g;
        inner.servo(target.clone(), epoch).unwrap();
        inner.tick(DT);
    }
    let r = rig.lock().unwrap();
    eprintln!("incident servo: peak {:.3} N·m, J7 {:.3} rad, trips {}", r.peak_tau, r.q[6], inner.gripper_force_trips);
    assert!(!r.broke, "finger broke");
    assert!(r.peak_tau < cap() + CAP_MARGIN_NM, "peak {:.3} N·m", r.peak_tau);
    assert!(inner.gripper_force_trips >= 1);
    assert!(r.q[6] < CONTACT_RAD + 0.25, "J7 {}", r.q[6]);
}

/// A gripper target inside a whole-arm joint move (presets, sequences, replay use the same
/// reference path): the latch holds J7 there too.
#[test]
fn incident_joint_move_is_capped() {
    let (mut inner, rig) = rig(Load::Incident);
    let epoch = inner.safety.epoch_id();
    let mut target = INSPECT.to_vec();
    target[6] = 0.0;
    armd::poses::save(&mut inner, &serde_json::json!({"name": "closed", "positions": target})).unwrap();
    inner.move_to_pose("closed", true, 1.0, None, epoch).unwrap();
    for _ in 0..(10.0 / DT) as usize { inner.tick(DT); }
    let r = rig.lock().unwrap();
    assert!(!r.broke, "finger broke");
    assert!(r.peak_tau < cap() + CAP_MARGIN_NM, "peak {:.3} N·m", r.peak_tau);
}

/// A stiff object stopped at low speed still gives holding = true (the cap backs the squeeze
/// off instead of failing the grasp).
#[test]
fn stiff_object_still_holding_under_the_cap() {
    for stiffness in [3.0_f32, 10.0, 30.0] {
        let (mut inner, rig) = rig(Load::Rigid(-2.0, stiffness));
        let epoch = inner.safety.epoch_id();
        let (id, _) = inner.start_gripper(close(true), epoch).unwrap();
        let a = run(&mut inner, id, 12.0);
        assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "stiffness {stiffness}: {a:?}");
        reset_stats(&rig);
        for _ in 0..50 { inner.tick(DT); }
        let peak = rig.lock().unwrap().peak_tau;
        assert!(peak <= cap() + 1e-3, "stiffness {stiffness}: held at {peak:.3} N·m");
    }
}

/// No object: the close still reaches closed (holding false, as before).
#[test]
fn free_close_reaches_closed() {
    let (mut inner, rig) = rig(Load::Free);
    let epoch = inner.safety.epoch_id();
    let (id, _) = inner.start_gripper(close(true), epoch).unwrap();
    let a = run(&mut inner, id, 12.0);
    assert_eq!((a.state.as_str(), a.holding), ("done", Some(false)), "{a:?}");
    assert!(inner.measured_positions[6] > -0.1, "J7 {}", inner.measured_positions[6]);
    assert_eq!(inner.gripper_force_trips, 0);
    assert!(rig.lock().unwrap().peak_tau < cap());
}

/// Opening is not capped: an open against the full free torque runs to open.
#[test]
fn open_is_unaffected() {
    let (mut inner, rig) = rig(Load::Free);
    { let mut r = rig.lock().unwrap(); r.q[6] = 0.0; }
    inner.tick(DT);
    let epoch = inner.safety.epoch_id();
    let open = GripperRequest { command: GripperCommand::Open, fraction: None, grasp: false, speed: 1.0, timeout_s: Some(10.0), tag: None };
    let (id, _) = inner.start_gripper(open, epoch).unwrap();
    let a = run(&mut inner, id, 12.0);
    assert_eq!(a.state, "done", "{a:?}");
    assert!((inner.measured_positions[6] - GripperPolicy::rebot_dm().open_rad).abs() < 0.05);
    assert_eq!(inner.gripper_force_trips, 0);
}
