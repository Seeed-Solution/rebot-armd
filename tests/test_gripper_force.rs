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
    /// An object that gives way from `.0` to `.1` (a box closed on across its corners turns
    /// square in the jaws): up to `YIELD_NM` while the jaw pushes it on (scaled by the jaw speed
    /// up to YIELD_SPEED), nothing once the jaw stops; rigid at `.1` with stiffness `.2`.
    Yielding(f32, f32, f32),
}

/// Yielding: torque while the jaw turns the object (sim block, 2026-11-22: 0.90 N·m from
/// contact at -2.37 rad until the jaw stopped).
const YIELD_NM: f32 = 0.9;
/// ... at the grasp contact speed (0.3 rad/s); it fell to 0.28 N·m as the jaw slowed to a stop.
const YIELD_SPEED: f32 = 0.3;

struct Rig { q: [f32; 7], v: [f32; 7], tau: [f32; 7], enabled: EnableState, load: Load, broke: bool, peak_tau: f32, max_lead: f32, max_cmd: f32,
    /// J7 per write: (command, position, torque)
    log: Vec<(f32, f32, f32)> }
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
                Load::Yielding(from, to, stiffness) => {
                    if q >= to { q = to; v = 0.0; (stiffness * (t.position - to)).max(0.0) }
                    else if q >= from { YIELD_NM * (v / YIELD_SPEED).clamp(0.0, 1.0) }
                    else if v.abs() > 0.05 { FREE_NM * v.signum() } else { 0.0 }
                }
            };
            r.q[6] = q;
            r.v[6] = v;
            r.tau[6] = tau;
            r.peak_tau = r.peak_tau.max(tau);
            r.log.push((t.position, q, tau));
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
    let rig = Arc::new(Mutex::new(Rig { q: INSPECT, v: [0.0; 7], tau: [0.0; 7], enabled: EnableState::Disabled, load, broke: false, peak_tau: 0.0, max_lead: 0.0, max_cmd: f32::NEG_INFINITY, log: Vec::new() }));
    let mut inner = SupervisorInner::new(config).with_driver(Box::new(RigDriver(rig.clone())));
    inner.lease_enabled = false;
    inner.tick(DT);
    { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| inner.enable_torque(__a)) }.unwrap();
    (inner, rig)
}

fn rig(load: Load) -> (SupervisorInner, Arc<Mutex<Rig>>) { rig_with(ArmConfig::rebot_dm(), load) }

/// A calibrated baseline of the rig's free torque (FREE_NM in both directions and speed bands),
/// as `gripper_calibrate` would learn it: contact at FREE_NM + 0.15 N·m (0.55, where the former
/// absolute rule used contact_torque_nm 0.6). Without it the closing threshold is the
/// uncalibrated 0.85 N·m.
fn calibrated(inner: &mut SupervisorInner) {
    use armd::gripper_baseline::{Direction, FreeTorqueBaseline, MoveRecord, MoveSamples};
    inner.follower = Some("rig".into());
    let p = inner.config.gripper_policy.clone().unwrap();
    let tool = inner.config.active_tool.clone();
    let mut b = FreeTorqueBaseline::new(p.closed_rad, p.open_rad, Some(tool.id), Some(tool.config_digest), Some("rig".into()));
    for (dir, from, to) in [(Direction::Close, p.open_rad, p.closed_rad), (Direction::Open, p.closed_rad, p.open_rad)] {
        for speed in [1.0f32, 0.3] {
            let mut samples = MoveSamples::default();
            for i in 0..=300 { samples.push(from + (to - from) * i as f32 / 300.0, FREE_NM, speed); }
            b.learn(&MoveRecord { direction: dir, start_rad: from, end_rad: to, samples, calibration: true }, 1.0);
        }
    }
    inner.gripper_baseline = Some(b);
}

/// Start with the jaw at closed (rig and supervisor command).
fn jaw_closed(inner: &mut SupervisorInner, rig: &Arc<Mutex<Rig>>) {
    rig.lock().unwrap().q[6] = 0.0;
    inner.commanded_positions[6] = 0.0;
    inner.target_positions[6] = 0.0;
    inner.tick(DT);
}

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

/// rebot_dm with the stream contact regulator off (stream_contact_speed_rad_s None).
fn without_regulator() -> ArmConfig {
    let mut config = ArmConfig::rebot_dm();
    config.gripper_policy.as_mut().unwrap().stream_contact_speed_rad_s = None;
    config
}

/// Margin above the cap: the first sample over the cap latches (GRIPPER_FORCE_CAP_TICKS 1), so
/// at most one tick of the incident's rise passes: 3.32 N·m/rad x 1.05 rad/s x 20 ms = 0.07 N·m.
const CAP_MARGIN_NM: f32 = RISE_NM_PER_RAD * 1.05 * DT as f32 + 1e-3;

/// Without the cap and the contact detection (rc10 behaviour), the incident profile breaks the
/// finger: the rig reproduces the incident.
#[test]
fn incident_profile_breaks_the_finger_without_the_cap() {
    let mut config = ArmConfig::rebot_dm();
    if let Some(p) = config.gripper_policy.as_mut() { p.max_close_torque_nm = None; p.contact_torque_nm = None; }
    let (mut inner, rig) = rig_with(config, Load::Incident);
    let epoch = inner.safety.epoch_id();
    let (id, _) = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.start_gripper(__a, close(true), epoch)) } }.unwrap();
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
    let (id, _) = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.start_gripper(__a, close(true), epoch)) } }.unwrap();
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

/// A plain close (grasp false) through the incident profile stops at the cap and fails short of closed.
#[test]
fn incident_close_without_grasp_detection_stops_at_the_cap() {
    let (mut inner, rig) = rig(Load::Incident);
    let epoch = inner.safety.epoch_id();
    let (id, _) = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.start_gripper(__a, close(false), epoch)) } }.unwrap();
    let a = run(&mut inner, id, 12.0);
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some("GRIPPER_TIMEOUT")), "{a:?}");
    { let r = rig.lock().unwrap(); assert!(!r.broke && r.peak_tau < cap() + CAP_MARGIN_NM, "peak {:.3} N·m", r.peak_tau); }
    let obs = inner.current_observation();
    let g = obs.gripper.unwrap();
    assert!(g.stalled && g.force_limited && g.force_limit_trips >= 1, "{g:?}");
    assert_eq!(obs.motion_limit.map(|l| (l.reason, l.joint)), Some((MotionLimitReason::GripperForce, Some(6))));
}

/// Teleop / policy stream: servo targets at closed through the incident profile. With the
/// stream contact regulator the grip stays under the cap (no trip); without it the cap latches.
#[test]
fn incident_servo_stream_is_capped() {
    for regulated in [true, false] {
    let (mut inner, rig) = rig_with(if regulated { ArmConfig::rebot_dm() } else { without_regulator() }, Load::Incident);
    calibrated(&mut inner);
    let epoch = inner.safety.epoch_id();
    let mut target = INSPECT.to_vec();
    // leader arm ramp toward closed, 1 rad/s, then held at closed
    let mut g = INSPECT[6];
    for _ in 0..(8.0 / DT) as usize {
        g = (g + DT as f32).min(0.0);
        target[6] = g;
        { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).and_then(|__a| inner.servo(__a, target.clone(), epoch)) } }.unwrap();
        inner.tick(DT);
    }
    let r = rig.lock().unwrap();
    eprintln!("incident servo (regulated {regulated}): peak {:.3} N·m, J7 {:.3} rad, trips {}", r.peak_tau, r.q[6], inner.gripper_force_trips);
    assert!(!r.broke, "finger broke");
    assert!(r.peak_tau < cap() + CAP_MARGIN_NM, "peak {:.3} N·m", r.peak_tau);
    if regulated {
        assert!(r.peak_tau < cap(), "peak {:.3} N·m", r.peak_tau);
        assert_eq!(inner.gripper_force_trips, 0);
    } else {
        assert!(inner.gripper_force_trips >= 1);
    }
    assert!(r.q[6] < CONTACT_RAD + 0.25, "J7 {}", r.q[6]);
    }
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
    { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.move_to_pose(__a, "closed", true, 1.0, None, epoch)) } }.unwrap();
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
        let (id, _) = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.start_gripper(__a, close(true), epoch)) } }.unwrap();
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
    let (id, _) = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.start_gripper(__a, close(true), epoch)) } }.unwrap();
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
    jaw_closed(&mut inner, &rig);
    let epoch = inner.safety.epoch_id();
    let open = GripperRequest { command: GripperCommand::Open, fraction: None, grasp: false, speed: 1.0, timeout_s: Some(10.0), tag: None };
    let (id, _) = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.start_gripper(__a, open, epoch)) } }.unwrap();
    let a = run(&mut inner, id, 12.0);
    assert_eq!(a.state, "done", "{a:?}");
    assert!((inner.measured_positions[6] - GripperPolicy::rebot_dm().open_rad).abs() < 0.05);
    assert_eq!(inner.gripper_force_trips, 0);
}

fn gripper_motion(inner: &SupervisorInner) -> Option<&armd::motion::GripperMotion> {
    match inner.active_trajectory.as_ref().map(|a| &a.reference) {
        Some(armd::motion::Reference::Gripper(g)) => Some(g.as_ref()),
        _ => None,
    }
}

/// Runs a grasp close; returns (action, contact_rad, contact stop, max command written after the
/// contact was detected).
fn grasp_tracking_contact(inner: &mut SupervisorInner, rig: &Arc<Mutex<Rig>>) -> (armd::motion::ActionStatus, Option<f32>, Option<f32>, f32) {
    let epoch = inner.safety.epoch_id();
    let (id, _) = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.start_gripper(__a, close(true), epoch)) } }.unwrap();
    let (mut contact, mut stop) = (None, None);
    for _ in 0..(12.0 / DT) as usize {
        inner.tick(DT);
        if contact.is_none() {
            if let Some(g) = gripper_motion(inner).filter(|g| g.contact_rad.is_some()) {
                contact = g.contact_rad;
                stop = g.contact_stop();
                reset_stats(rig);
            }
        }
        let a = inner.last_action.clone().unwrap();
        assert_eq!(a.id, id);
        if !a.is_running() {
            let max_cmd = rig.lock().unwrap().max_cmd;
            return (a, contact, stop, max_cmd);
        }
    }
    panic!("grasp still running");
}

/// Item 2: the grasp detects the contact from the torque rise and never commands past
/// contact + squeeze_rad afterwards.
#[test]
fn incident_grasp_stops_at_contact_plus_squeeze() {
    let (mut inner, rig) = rig(Load::Incident);
    calibrated(&mut inner);
    let (a, contact, stop, max_cmd) = grasp_tracking_contact(&mut inner, &rig);
    eprintln!("incident contact {contact:?}, stop {stop:?}, max command after contact {max_cmd:.3}, J7 {:.3}, holding {:?}",
        inner.measured_positions[6], a.holding);
    assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{a:?}");
    match contact {
        Some(contact) => {
            assert!((contact - CONTACT_RAD).abs() < 0.08, "contact at {contact}");
            let squeeze = GripperPolicy::rebot_dm().squeeze_rad;
            assert!((stop.unwrap() - (contact + squeeze)).abs() < 1e-5);
            assert!(max_cmd <= stop.unwrap() + 1e-4, "command {max_cmd} past the stop {stop:?}");
            assert!(inner.commanded_positions[6] <= stop.unwrap() + 1e-4);
        }
        // the baseline contact needs 3 ticks; the incident rises 0.07 N·m per tick from 0.85,
        // so the cap may latch first: the jaw then holds at the cap trip
        None => assert!(inner.gripper_force_trips >= 1 && inner.measured_positions[6] < CONTACT_RAD + 0.25, "no contact and no cap trip"),
    }
    let r = rig.lock().unwrap();
    assert!(!r.broke && r.peak_tau < cap() + CAP_MARGIN_NM, "peak {:.3}", r.peak_tau);
}

/// An object that gives way after the contact (turns square in the jaws): the jaw reaches the
/// contact stop with no torque. The contact is dropped there and the close goes on to the
/// object's rigid face, where the grasp holds with torque (it stopped at the first stop with
/// about 0 N·m before and the object fell out on the lift: sim axis_block_detect_depth_record).
#[test]
fn yielding_object_closes_on_to_a_firm_grip() {
    let (from, to) = (-2.37_f32, -2.0_f32);
    let (mut inner, rig) = rig(Load::Yielding(from, to, 10.0));
    let (a, contact, stop, _) = grasp_tracking_contact(&mut inner, &rig);
    let contact = contact.expect("contact detected");
    eprintln!("yielding: first contact {contact:.3}, first stop {stop:?}, J7 {:.3}, command {:.3}, holding {:?}",
        inner.measured_positions[6], inner.commanded_positions[6], a.holding);
    assert!((contact - from).abs() < 0.08, "contact at {contact}");
    assert!(stop.unwrap() < to - 0.1, "first stop {stop:?} short of the rigid face");
    assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{a:?}");
    assert!((inner.measured_positions[6] - to).abs() < 1e-3, "J7 {} not on the rigid face", inner.measured_positions[6]);
    // the hold presses on the face: torque above holding_torque_nm, under the cap
    reset_stats(&rig);
    for _ in 0..50 { inner.tick(DT); }
    let tau = rig.lock().unwrap().tau[6];
    assert!(tau >= GripperPolicy::rebot_dm().holding_torque_nm && tau <= cap() + 1e-3, "hold torque {tau:.3} N·m");
    // never past the squeeze from the last contact
    assert!(inner.commanded_positions[6] <= to + GripperPolicy::rebot_dm().squeeze_rad + 1e-4, "command {}", inner.commanded_positions[6]);
}

/// Without torque contact detection and without the cap, the following error (command ahead,
/// jaw stopped) finds the contact on a rigid object (soft enough that the stall rule, 0.444 N·m
/// for 6 ticks, comes later).
#[test]
fn rigid_contact_from_following_error() {
    let mut config = ArmConfig::rebot_dm();
    if let Some(p) = config.gripper_policy.as_mut() { p.contact_torque_nm = None; p.max_close_torque_nm = None; }
    let (mut inner, rig) = rig_with(config, Load::Rigid(-2.0, 1.0));
    let (a, contact, stop, max_cmd) = grasp_tracking_contact(&mut inner, &rig);
    let contact = contact.expect("contact detected");
    assert!((contact + 2.0).abs() < 1e-3, "contact at {contact}");
    assert!(max_cmd <= stop.unwrap() + 1e-4, "command {max_cmd} past the stop {stop:?}");
    assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{a:?}");
}

/// Free close: no false contact, the jaw reaches closed.
#[test]
fn free_close_detects_no_contact() {
    let (mut inner, rig) = rig(Load::Free);
    let (a, contact, _, _) = grasp_tracking_contact(&mut inner, &rig);
    assert_eq!(contact, None);
    assert_eq!(a.holding, Some(false), "{a:?}");
    assert!(inner.measured_positions[6] > -0.1);
}

fn max_lead() -> f32 { GripperPolicy::rebot_dm().max_lead_rad.unwrap() }

/// Item 3: a servo stream to closed against a rigid block: the command stays within
/// max_lead_rad of the measured jaw (it ran to the end of the stroke before).
#[test]
fn blocked_servo_stream_lead_is_capped() {
    let mut config = ArmConfig::rebot_dm();
    // without the cap and the stall latch tripping first the lead cap alone bounds the error
    if let Some(p) = config.gripper_policy.as_mut() { p.max_close_torque_nm = None; }
    let (mut inner, rig) = rig_with(config, Load::Rigid(-2.0, 0.2));
    let epoch = inner.safety.epoch_id();
    let mut target = INSPECT.to_vec();
    target[6] = 0.0;
    reset_stats(&rig);
    for _ in 0..(6.0 / DT) as usize {
        { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).and_then(|__a| inner.servo(__a, target.clone(), epoch)) } }.unwrap();
        inner.tick(DT);
    }
    let r = rig.lock().unwrap();
    eprintln!("blocked servo: max lead {:.3} rad, J7 {:.3}", r.max_lead, r.q[6]);
    assert!(r.max_lead <= max_lead() + 1e-4, "lead {}", r.max_lead);
    assert!((r.q[6] + 2.0).abs() < 1e-3);
}

/// The lead cap does not stop a free open / close at 1 rad/s (lag 0.23 rad on the rig).
#[test]
fn lead_cap_leaves_free_motion_alone() {
    let (mut inner, rig) = rig(Load::Free);
    let epoch = inner.safety.epoch_id();
    let (id, _) = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.start_gripper(__a, close(false), epoch)) } }.unwrap();
    let a = run(&mut inner, id, 12.0);
    assert_eq!(a.state, "done", "{a:?}");
    assert!(inner.measured_positions[6] > -0.06);
    assert!(rig.lock().unwrap().max_lead < max_lead());
}

/// The contact stop only takes a grasp command back: an open ramps its command (no jump to open).
#[test]
fn open_command_ramps() {
    let (mut inner, rig) = rig(Load::Free);
    jaw_closed(&mut inner, &rig);
    let epoch = inner.safety.epoch_id();
    let open = GripperRequest { command: GripperCommand::Open, fraction: None, grasp: false, speed: 1.0, timeout_s: Some(10.0), tag: None };
    { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.start_gripper(__a, open, epoch)) } }.unwrap();
    for _ in 0..5 { inner.tick(DT); }
    assert!(inner.commanded_positions[6] > -0.2, "J7 command {}", inner.commanded_positions[6]);
}

/// Review item 7: a live policy run streams through the same gripper force cap and lead cap
/// as the servo stream (policy_step -> servo target -> tick). The follower is "sim" so the
/// run needs no release gate; the rig is still a driver with torque feedback.
#[cfg(feature = "policy-runtime")]
mod policy_stream {
    use super::*;
    use armd::policy_runtime::{PolicyMode, PolicyRunStart};

    fn begin(inner: &mut SupervisorInner) -> u64 {
        inner.follower = Some("sim".into());
        inner.lease_enabled = true;
        let gen = inner.acquire_lease("policy-rig").unwrap();
        let epoch = inner.safety.epoch_id();
        { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.begin_policy_run(__a, PolicyRunStart {
            epoch_id: epoch, connection_id: "policy-rig".into(), lease_generation: gen, policy_id: "rig".into(),
            artifact_digest: "0".repeat(64), backend: "mlp".into(), mode: PolicyMode::Live, sim_only: false,
            max_first_step_rad: None, max_step_rad: None, tag: None, i_am_at_the_arm: false, verified_release_gate: None,
        })) } }.unwrap()
    }

    /// The incident profile through policy targets (stream contact regulator off): the cap
    /// latches, the jaw command stays at the latch position while the policy keeps asking for
    /// closed, the finger survives. With the regulator: tests/test_gripper_force.rs
    /// rigid_policy_stream_is_regulated.
    #[test]
    fn incident_policy_stream_is_capped() {
        let (mut inner, rig) = rig_with(without_regulator(), Load::Incident);
        let id = begin(&mut inner);
        let mut target = INSPECT.to_vec();
        let mut g = INSPECT[6];
        let mut latched_cmd: Option<f32> = None;
        for _ in 0..(8.0 / DT) as usize {
            g = (g + DT as f32).min(0.0);
            target[6] = g;
            inner.policy_step(id, target.clone(), Some(0)).unwrap();
            inner.tick(DT);
            if inner.gripper_force_trips >= 1 {
                let cmd = inner.commanded_positions[6];
                // after the trip the jaw command never closes further than where it latched
                match latched_cmd {
                    None => latched_cmd = Some(cmd),
                    Some(l) => assert!(cmd <= l + 1e-3, "jaw command moved on after the cap: {cmd} > {l}"),
                }
            }
        }
        let r = rig.lock().unwrap();
        eprintln!("incident policy: peak {:.3} N·m, J7 {:.3} rad, cmd {:?}, trips {}", r.peak_tau, r.q[6], latched_cmd, inner.gripper_force_trips);
        assert!(!r.broke, "finger broke");
        assert!(r.peak_tau < cap() + CAP_MARGIN_NM, "peak {:.3} N·m", r.peak_tau);
        assert!(inner.gripper_force_trips >= 1);
        assert!(r.q[6] < CONTACT_RAD + 0.25, "J7 {}", r.q[6]);
        assert!(inner.commanded_positions[6] < CONTACT_RAD + 0.25, "jaw command {}", inner.commanded_positions[6]);
        assert!(inner.is_policy_run_active(id), "the cap holds the jaw; it does not end the run");
    }

    /// Policy targets at closed against a rigid block (cap off): the lead cap bounds the
    /// command to max_lead_rad ahead of the measured jaw.
    #[test]
    fn blocked_policy_stream_lead_is_capped() {
        let mut config = ArmConfig::rebot_dm();
        if let Some(p) = config.gripper_policy.as_mut() { p.max_close_torque_nm = None; }
        let (mut inner, rig) = rig_with(config, Load::Rigid(-2.0, 0.2));
        let id = begin(&mut inner);
        let mut target = INSPECT.to_vec();
        reset_stats(&rig);
        let mut g = INSPECT[6];
        for _ in 0..(6.0 / DT) as usize {
            g = (g + 0.2).min(0.0);
            target[6] = g;
            inner.policy_step(id, target.clone(), Some(0)).unwrap();
            inner.tick(DT);
        }
        let r = rig.lock().unwrap();
        eprintln!("blocked policy: max lead {:.3} rad, J7 {:.3}", r.max_lead, r.q[6]);
        assert!(r.max_lead <= max_lead() + 1e-4, "lead {}", r.max_lead);
        assert!((r.q[6] + 2.0).abs() < 1e-3);
    }
}

/// What a closing stream did to J7 on the rig.
#[derive(Debug, Default)]
struct StreamRun {
    /// First torque sample with the jaw at the block (the impact).
    impact_nm: f32,
    /// Max torque after the impact sample.
    after_impact_nm: f32,
    /// Max torque over the last 3 s of the hold.
    held_nm: f32,
    /// Re-squeeze cycles after the impact: torque below 0.2 N·m, then above 0.5 N·m again.
    cycles: usize,
    trips: u64,
}

/// Leader-like stream at 30 fps on the 50 Hz loop: the J7 target ramps toward closed at
/// `speed` rad/s, then stays at closed until `secs`.
fn close_stream(inner: &mut SupervisorInner, rig: &Arc<Mutex<Rig>>, speed: f32, secs: f64) -> StreamRun {
    let epoch = inner.safety.epoch_id();
    let mut target = INSPECT.to_vec();
    let (mut t, mut next_frame) = (0.0f64, 0.0f64);
    rig.lock().unwrap().log.clear();
    for _ in 0..(secs / DT) as usize {
        if t >= next_frame - 1e-9 {
            next_frame += 1.0 / 30.0;
            target[6] = (INSPECT[6] + speed * t as f32).min(0.0);
            { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).and_then(|__a| inner.servo(__a, target.clone(), epoch)) } }.unwrap();
        }
        inner.tick(DT);
        t += DT;
    }
    let r = rig.lock().unwrap();
    let block = match r.load { Load::Rigid(b, _) => b, _ => 0.0 };
    let log = &r.log;
    let contact = log.iter().position(|&(_, q, _)| q >= block - 1e-6);
    let mut run = StreamRun { trips: inner.gripper_force_trips, ..Default::default() };
    if let Some(c) = contact {
        run.impact_nm = log[c].2;
        run.after_impact_nm = log[c + 1..].iter().map(|e| e.2).fold(0.0, f32::max);
        let mut low = false;
        for &(_, _, tau) in &log[c + 1..] {
            if tau < 0.2 { low = true; } else if low && tau > 0.5 { run.cycles += 1; low = false; }
        }
    }
    let n = log.len();
    run.held_nm = log[n.saturating_sub((3.0 / DT) as usize)..].iter().map(|e| e.2).fold(0.0, f32::max);
    run
}

/// One tick of the post-latch grip ramp on a contact of `stiffness` (N·m), its largest step.
fn stream_ramp_tick_nm(stiffness: f32) -> f32 {
    stiffness * armd::supervisor::GRIPPER_STREAM_CAP_RAMP_RAD_S * DT as f32 + 1e-3
}

/// Seconds of a stream from open to closed at `speed`, plus a 5 s hold.
fn stream_secs(speed: f32) -> f64 { f64::from(-INSPECT[6] / speed) + 5.0 }

/// Item 1: a torque-cap latch under a stream that keeps asking for closed holds (no
/// re-squeeze cycle); regulator off, so the latch is what bounds the force. After the latch the
/// grip ramps back in once (GRIPPER_STREAM_CAP_RAMP_*) to about 0.6 x cap and holds there.
#[test]
fn stream_latch_does_not_release_while_closing() {
    for stiffness in [3.0f32, 10.0, 30.0] {
        let (mut inner, rig) = rig_with(without_regulator(), Load::Rigid(-2.0, stiffness));
        let run = close_stream(&mut inner, &rig, 1.0, stream_secs(1.0));
        eprintln!("latch only, k {stiffness}: {run:?}");
        if run.trips > 0 {
            assert_eq!(run.trips, 1, "k {stiffness}: one trip, no re-squeeze cycle: {run:?}");
            assert!(run.cycles <= 1, "k {stiffness}: one ramp back in after the latch: {run:?}");
            assert!(inner.gripper_stalled && inner.gripper_force_capped);
            assert!((0.55..=0.65).contains(&run.held_nm), "k {stiffness}: held at {:.3} N·m", run.held_nm);
            assert!(run.after_impact_nm <= cap() + stream_ramp_tick_nm(stiffness), "k {stiffness}: {run:?}");
        } else {
            // soft enough that the impact stays under the cap: the ordinary stall rule holds it
            assert!(run.after_impact_nm <= cap() + 1e-3, "k {stiffness}: {run:?}");
        }
    }
}

/// Item 2 + 3: rigid block at 3 / 10 / 30 N·m/rad under a 30 fps closing stream at 1 rad/s
/// (incident leader ramp) and 0.3 rad/s. After the impact sample the torque stays at or
/// below 1.2 x cap and settles near contact_torque_nm (or, when the impact sample is over the
/// cap, the cap latch holds the jaw at the block); no re-squeeze cycles over the 5 s hold.
/// The impact sample itself is stiffness x the POS_VEL lag at contact (no feedback precedes it);
/// it stays under stiffness x max_lead_rad.
#[test]
fn rigid_stream_contact_is_regulated() {
    let p = GripperPolicy::rebot_dm();
    let contact = p.contact_torque_nm.unwrap();
    for speed in [1.0f32, 0.3] {
        for stiffness in [3.0f32, 10.0, 30.0] {
            let (mut inner, rig) = rig(Load::Rigid(-2.0, stiffness));
            let run = close_stream(&mut inner, &rig, speed, stream_secs(speed));
            eprintln!("regulated, {speed} rad/s, k {stiffness}: impact {:.3} N·m, after impact {:.3}, held {:.3}, cycles {}, trips {}",
                run.impact_nm, run.after_impact_nm, run.held_nm, run.cycles, run.trips);
            assert!(run.impact_nm <= stiffness * max_lead() + 1e-3, "{run:?}");
            assert!(run.after_impact_nm <= 1.2 * cap(), "k {stiffness} {speed} rad/s: {run:?}");
            if stiffness * max_lead().min(speed / 4.5) > cap() {
                // the impact sample is over the cap: the first over-cap sample latches
                // (GRIPPER_FORCE_CAP_TICKS 1) at the measured jaw; while the stream still closes
                // the grip ramps back in once to about 0.6 x cap and holds there
                assert_eq!(run.trips, 1, "k {stiffness} {speed} rad/s: {run:?}");
                assert!(run.cycles <= 1, "k {stiffness} {speed} rad/s: {run:?}");
                assert!(run.after_impact_nm <= cap() + stream_ramp_tick_nm(stiffness), "k {stiffness} {speed} rad/s: {run:?}");
                assert!((0.55..=0.65).contains(&run.held_nm), "k {stiffness} {speed} rad/s: held {:.3}", run.held_nm);
            } else {
                assert_eq!(run.cycles, 0, "k {stiffness} {speed} rad/s: {run:?}");
                assert!(run.held_nm <= contact + 0.2 && run.held_nm >= 0.5 * contact, "k {stiffness} {speed} rad/s: held {:.3}", run.held_nm);
            }
            let r = rig.lock().unwrap();
            assert!((r.q[6] + 2.0).abs() < 1e-3, "jaw at the block: {}", r.q[6]);
        }
    }
}

/// Opening after the hold works at once: the first open frame moves the command open.
#[test]
fn open_after_stream_hold_is_immediate() {
    for config in [ArmConfig::rebot_dm(), without_regulator()] {
        let (mut inner, rig) = rig_with(config, Load::Rigid(-2.0, 10.0));
        close_stream(&mut inner, &rig, 1.0, stream_secs(1.0));
        let epoch = inner.safety.epoch_id();
        let hold = inner.commanded_positions[6];
        let mut target = INSPECT.to_vec();
        target[6] = -3.0;
        { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).and_then(|__a| inner.servo(__a, target.clone(), epoch)) } }.unwrap();
        assert!(!inner.gripper_stalled, "an open frame releases the latch");
        // the jerk-limited stepper reverses within a few ticks (it may still creep closed at
        // 0.02 rad/s under the contact regulator): 3 ticks = 60 ms
        for _ in 0..3 { inner.tick(DT); }
        assert!(inner.commanded_positions[6] < hold - 1e-4, "command {} hold {hold}", inner.commanded_positions[6]);
        for _ in 0..7 { { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).and_then(|__a| inner.servo(__a, target.clone(), epoch)) } }.unwrap(); inner.tick(DT); }
        assert!(inner.commanded_positions[6] < hold - 0.05, "after 0.2 s (jerk-limited start): command {} hold {hold}", inner.commanded_positions[6]);
        for _ in 0..100 { { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).and_then(|__a| inner.servo(__a, target.clone(), epoch)) } }.unwrap(); inner.tick(DT); }
        assert!((rig.lock().unwrap().q[6] + 3.0).abs() < 0.05, "J7 {}", rig.lock().unwrap().q[6]);
    }
}

/// An open by less than the margin keeps a torque-cap latch; the jaw stays.
#[test]
fn small_open_keeps_the_stream_latch() {
    let (mut inner, rig) = rig_with(without_regulator(), Load::Rigid(-2.0, 10.0));
    let run = close_stream(&mut inner, &rig, 1.0, stream_secs(1.0));
    assert_eq!(run.trips, 1, "{run:?}");
    let epoch = inner.safety.epoch_id();
    let hold = inner.commanded_positions[6];
    let mut target = INSPECT.to_vec();
    target[6] = hold - 0.5 * armd::supervisor::GRIPPER_FORCE_RELEASE_MARGIN_RAD;
    for _ in 0..20 { { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).and_then(|__a| inner.servo(__a, target.clone(), epoch)) } }.unwrap(); inner.tick(DT); }
    assert!(inner.gripper_stalled && inner.gripper_force_capped);
    assert!((inner.commanded_positions[6] - hold).abs() < 1e-5);
}

/// Free close (no object): the regulator never engages; the stream reaches closed in the same
/// ticks and with the same commands as without it, at 1 rad/s and at a slow 0.3 rad/s.
#[test]
fn free_stream_is_not_slowed() {
    for speed in [1.0f32, 0.3] {
        let mut logs = Vec::new();
        for config in [ArmConfig::rebot_dm(), without_regulator()] {
            let (mut inner, rig) = rig_with(config, Load::Free);
            close_stream(&mut inner, &rig, speed, stream_secs(speed));
            let r = rig.lock().unwrap();
            let closed = r.log.iter().position(|&(_, q, _)| q > -0.05);
            assert!(r.q[6] > -0.05, "J7 {}", r.q[6]);
            let lag = r.log.iter().map(|&(c, q, _)| c - q).fold(0.0, f32::max);
            eprintln!("free {speed} rad/s: closed after {closed:?} ticks, max lead {lag:.3} rad");
            logs.push(r.log.clone());
        }
        assert_eq!(logs[0], logs[1], "{speed} rad/s: the regulator changed a free close");
    }
}

/// Live policy targets at closed onto a rigid block: same regulation as the servo stream.
#[cfg(feature = "policy-runtime")]
#[test]
fn rigid_policy_stream_is_regulated() {
    use armd::policy_runtime::{PolicyMode, PolicyRunStart};
    for stiffness in [3.0f32, 10.0, 30.0] {
        let (mut inner, rig) = rig(Load::Rigid(-2.0, stiffness));
        inner.follower = Some("sim".into());
        inner.lease_enabled = true;
        let gen = inner.acquire_lease("policy-rig").unwrap();
        let epoch = inner.safety.epoch_id();
        let id = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.begin_policy_run(__a, PolicyRunStart {
            epoch_id: epoch, connection_id: "policy-rig".into(), lease_generation: gen, policy_id: "rig".into(),
            artifact_digest: "0".repeat(64), backend: "mlp".into(), mode: PolicyMode::Live, sim_only: false,
            max_first_step_rad: None, max_step_rad: None, tag: None, i_am_at_the_arm: false, verified_release_gate: None,
        })) } }.unwrap();
        let mut target = INSPECT.to_vec();
        let mut g = INSPECT[6];
        for _ in 0..(stream_secs(1.0) / DT) as usize {
            g = (g + DT as f32).min(0.0);
            target[6] = g;
            inner.policy_step(id, target.clone(), Some(0)).unwrap();
            inner.tick(DT);
        }
        let r = rig.lock().unwrap();
        let c = r.log.iter().position(|&(_, q, _)| q >= -2.0 - 1e-6).unwrap();
        let after = r.log[c + 1..].iter().map(|e| e.2).fold(0.0, f32::max);
        eprintln!("policy k {stiffness}: impact {:.3} N·m, after impact {after:.3}", r.log[c].2);
        assert!(after <= 1.2 * cap(), "k {stiffness}: {after}");
        assert!(inner.is_policy_run_active(id));
    }
}

/// Stop and E-stop end the stream's gripper state: after a torque-cap latch under a closing
/// stream, the J7 command does not advance again on the following low-torque samples (before,
/// the stream target outlived the stop until the 200 ms frame timeout).
#[test]
fn stop_and_estop_end_the_stream_gripper_state() {
    for estop in [false, true] {
        let (mut inner, rig) = rig_with(without_regulator(), Load::Rigid(-2.0, 10.0));
        let epoch = inner.safety.epoch_id();
        let mut target = INSPECT.to_vec();
        let mut g = INSPECT[6];
        let mut tripped_for = 0;
        for _ in 0..(10.0 / DT) as usize {
            g = (g + DT as f32).min(0.0);
            target[6] = g;
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).unwrap();
            inner.servo(admitted, target.clone(), epoch).unwrap();
            inner.tick(DT);
            if inner.gripper_force_trips > 0 { tripped_for += 1; }
            if tripped_for >= 5 { break; }
        }
        assert!(inner.gripper_force_trips > 0, "no trip");
        if estop {
            inner.requested_estop_count.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        } else {
            inner.stop_motion();
        }
        inner.tick(DT);
        let at_stop = inner.commanded_positions[6];
        assert!(inner.gripper_stream_target.is_none() && inner.gripper_stream_contact.is_none(), "estop {estop}");
        // the block reads about 0 N·m at the held jaw: low-torque samples
        let mut max_cmd = at_stop;
        for _ in 0..100 {
            inner.tick(DT);
            max_cmd = max_cmd.max(inner.commanded_positions[6]);
        }
        let r = rig.lock().unwrap();
        eprintln!("estop {estop}: J7 command {at_stop:.4} -> max {max_cmd:.4}, measured {:.4}, tau {:.3}", r.q[6], r.tau[6]);
        assert!(max_cmd <= at_stop + 1e-6, "estop {estop}: J7 command advanced {at_stop} -> {max_cmd}");
        assert!((at_stop - r.q[6]).abs() < 0.02, "estop {estop}: holds the measured jaw: cmd {at_stop}, jaw {}", r.q[6]);
    }
}

/// Post-impact samples of a run: every torque after the first sample at the block.
fn post_impact(rig: &Arc<Mutex<Rig>>) -> (f32, f32, usize) {
    let r = rig.lock().unwrap();
    let block = match r.load { Load::Rigid(b, _) => b, _ => 0.0 };
    let c = r.log.iter().position(|&(_, q, _)| q >= block - 1e-6).expect("reached the block");
    let after = r.log[c + 1..].iter().map(|e| e.2).fold(0.0, f32::max);
    (r.log[c].2, after, r.log.len() - c)
}

/// Very stiff contacts (250 and 1000 N·m/rad) under a closing stream: the impact sample is
/// stiffness x the command's lead at contact (not bounded by feedback); after it every torque
/// sample stays at or below the cap: the latch ramp sizes its steps from the stiffness seen
/// at the trip.
#[test]
fn very_stiff_stream_contact_stays_under_the_cap_after_the_impact() {
    for regulated in [true, false] {
        for speed in [1.0f32, 0.3] {
            for stiffness in [250.0f32, 1000.0] {
                let (mut inner, rig) = rig_with(if regulated { ArmConfig::rebot_dm() } else { without_regulator() }, Load::Rigid(-2.0, stiffness));
                let run = close_stream(&mut inner, &rig, speed, stream_secs(speed));
                let (impact, after, n) = post_impact(&rig);
                eprintln!("stream k {stiffness} {speed} rad/s regulated {regulated}: impact {impact:.2} N·m, after impact max {after:.3} N·m over {n} samples, held {:.3}, trips {}",
                    run.held_nm, run.trips);
                assert!(after <= cap() + 1e-3, "k {stiffness} {speed} rad/s: {after}");
                assert!(run.held_nm <= cap() && run.held_nm >= 0.5, "k {stiffness} {speed} rad/s: held {}", run.held_nm);
                assert!(run.trips <= 2, "k {stiffness}: trips {}", run.trips);
            }
        }
    }
}

/// The same contacts closed by a grasp: after the impact sample every torque sample stays at
/// or below the cap and the grasp holds.
#[test]
fn very_stiff_grasp_stays_under_the_cap_after_the_impact() {
    for stiffness in [250.0f32, 1000.0] {
        let (mut inner, rig) = rig(Load::Rigid(-2.0, stiffness));
        rig.lock().unwrap().log.clear();
        let epoch = inner.safety.epoch_id();
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).unwrap();
        let (id, _) = inner.start_gripper(admitted, close(true), epoch).unwrap();
        let a = run(&mut inner, id, 12.0);
        for _ in 0..100 { inner.tick(DT); }
        let (impact, after, n) = post_impact(&rig);
        eprintln!("grasp k {stiffness}: impact {impact:.2} N·m, after impact max {after:.3} N·m over {n} samples, holding {:?}, trips {}", a.holding, inner.gripper_force_trips);
        assert!(after <= cap() + 1e-3, "k {stiffness}: {after}");
        assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "k {stiffness}: {a:?}");
    }
}

/// CapRamp: steps keep the predicted torque at or below the hold torque for the estimated
/// stiffness, the estimate only grows with observed steps, and the third trip on a contact
/// ends the ramping.
#[test]
fn cap_ramp_steps_are_sized_by_stiffness() {
    use armd::motion::{CapRamp, CAP_RAMP_MARGIN_NM, CAP_RAMP_MAX_TRIPS};
    for k in [3.0f32, 30.0, 250.0, 1000.0] {
        // trip with the command 0.2 rad ahead of the jaw on a linear contact
        let mut r = CapRamp::from_trip(k * 0.2, 0.2);
        let (mut q, mut peak) = (0.0f32, 0.0f32);
        for _ in 0..2000 {
            let tau = k * q;
            peak = peak.max(tau);
            q += r.step(tau, 0.6, 0.005);
        }
        assert!(peak <= 0.6 + 1e-4, "k {k}: peak {peak}");
        assert!(k * q >= 0.6 - CAP_RAMP_MARGIN_NM - 1e-3, "k {k}: settled at {}", k * q);
    }
    // a soft first estimate on a stiff contact: the first step's response raises it
    let mut r = CapRamp::from_trip(0.6, 0.2); // k_est 3
    let k = 300.0f32;
    let mut q = 0.0f32;
    let mut taus = Vec::new();
    for _ in 0..50 { let tau = k * q; taus.push(tau); q += r.step(tau, 0.6, 0.0005); }
    assert!(r.k_est >= 0.9 * k, "k_est {}", r.k_est);
    assert!(taus.iter().all(|t| *t <= 0.6 + k * 0.0005 + 1e-4), "{taus:?}");
    // trips
    let mut r = CapRamp::from_trip(1.2, 0.01);
    for _ in 1..CAP_RAMP_MAX_TRIPS { assert!(!r.exhausted()); r.trip(1.2, 0.01); }
    assert!(r.exhausted());
    assert_eq!(r.step(0.0, 0.6, 0.005), 0.0);
}

/// The third torque-cap trip on one contact under a stream stops the stream, and the latch
/// no longer ramps on that contact (frames that keep closing do not press again).
#[test]
fn third_cap_trip_stops_the_stream() {
    let (mut inner, rig) = rig_with(without_regulator(), Load::Rigid(-2.0, 10.0));
    let epoch = inner.safety.epoch_id();
    let mut target = INSPECT.to_vec();
    let mut g = INSPECT[6];
    let mut send = |inner: &mut SupervisorInner, g: f32| {
        let mut t = target.clone();
        t[6] = g;
        target = t.clone();
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).unwrap();
        inner.servo(admitted, t, epoch).unwrap();
        inner.tick(DT);
    };
    while inner.gripper_force_trips == 0 { g = (g + DT as f32).min(0.0); send(&mut inner, g); }
    for _ in 0..20 { send(&mut inner, 0.0); }
    // two more trips on this contact: the second by a contact ten times stiffer than estimated
    inner.gripper_cap_ramp.as_mut().unwrap().trip(1.2, 0.12);
    { let mut r = rig.lock().unwrap(); r.load = Load::Rigid(-2.0, 100.0); }
    let trips = inner.gripper_force_trips;
    for _ in 0..50 {
        if inner.gripper_force_trips > trips { break; }
        send(&mut inner, 0.0);
    }
    assert!(inner.gripper_force_trips > trips, "no third trip");
    assert!(inner.gripper_cap_ramp.unwrap().exhausted());
    assert!(inner.servo_target.is_none() && inner.gripper_stream_target.is_none(), "stream not stopped");
    // a stream that keeps asking for closed: no press on this contact any more
    rig.lock().unwrap().peak_tau = 0.0;
    let cmd = inner.commanded_positions[6];
    for _ in 0..100 { send(&mut inner, 0.0); }
    let peak = rig.lock().unwrap().peak_tau;
    eprintln!("after the third trip: J7 command {cmd:.4} -> {:.4}, peak {peak:.3} N·m", inner.commanded_positions[6]);
    assert!(inner.commanded_positions[6] <= cmd + 1e-6 && peak <= 0.05, "pressed again: peak {peak}");
}

/// Stop or E-stop during a plain gripper close onto a soft object, with the command ahead of
/// the jaw: within one tick the J7 command is the measured jaw and the object is not compressed
/// further.
#[test]
fn stop_during_a_gripper_action_holds_the_measured_jaw() {
    for estop in [false, true] {
        let (mut inner, rig) = rig(Load::Rigid(-2.0, 1.5));
        let epoch = inner.safety.epoch_id();
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).unwrap();
        let (_id, _) = inner.start_gripper(admitted, close(false), epoch).unwrap();
        let mut lead = 0.0;
        for _ in 0..(10.0 / DT) as usize {
            inner.tick(DT);
            let q = rig.lock().unwrap().q[6];
            lead = inner.commanded_positions[6] - q;
            if q >= -2.0 - 1e-6 && lead >= 0.15 { break; }
        }
        let tau_at_stop = rig.lock().unwrap().tau[6];
        assert!(lead >= 0.15 && tau_at_stop < cap(), "lead {lead}, tau {tau_at_stop}");
        if estop {
            inner.requested_estop_count.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        } else {
            inner.stop_motion();
        }
        inner.tick(DT);
        let (cmd, meas) = (inner.commanded_positions[6], inner.measured_positions[6]);
        eprintln!("estop {estop}: lead {lead:.3} rad, tau {tau_at_stop:.3} N·m at the stop; one tick later command {cmd:.4}, jaw {meas:.4}");
        assert!((cmd - meas).abs() < 1e-3, "estop {estop}: command {cmd} vs jaw {meas}");
        rig.lock().unwrap().peak_tau = 0.0;
        for _ in 0..50 {
            inner.tick(DT);
            assert!((inner.commanded_positions[6] - cmd).abs() < 1e-6, "estop {estop}: command moved");
        }
        let peak = rig.lock().unwrap().peak_tau;
        assert!(peak <= 0.01, "estop {estop}: still compressing, {peak} N·m");
    }
}
