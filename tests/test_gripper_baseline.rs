//! Grasp contact relative to the learned free-motion torque of the jaw (`armd::gripper_baseline`).
//!
//! Office B601-DM 2026-11-23 (1.5.0-rc8, new finger, no lubricant, at home): unloaded |τ7|
//! median per 0.5 rad bin while |v| > 0.3 rad/s, opening: -6.0: 0.49, -5.5: 0.46, -5.0: 0.47,
//! -4.5: 0.46, -4.0: 0.46, -3.5: 0.41, -3.0: 0.42, -2.5: 0.42, -2.0: 0.32, -1.5: 0.27, -1.0: 0.21,
//! -0.5: 0.12 N·m; closing a few hundredths higher; p90 up to 0.68 N·m. With the fixed
//! contact_torque_nm 0.6 the closes crept at the contact speed and timed out (8.5 s at -4.155
//! rad, and a banana grasp at -4.065 rad). The rig scales that shape so free motion needs
//! 0.55-0.7 N·m near open, with tick-to-tick noise and a run-to-run offset.
use std::sync::{Arc, Mutex};
use std::time::Duration;

#[global_allocator]
static ALLOC: armd::diagnostics::alloc::TimedAlloc = armd::diagnostics::alloc::TimedAlloc;

use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
use armd::gripper_baseline::{ContactSource, Direction, ThresholdSource};
use armd::model::{ArmConfig, GripperPolicy, JointState};
use armd::motion::{GripperCommand, GripperRequest};
use armd::SupervisorInner;
use armd::supervisor::GripperBaselineCmd;

const DT: f64 = 0.02;
const INSPECT: [f32; 7] = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0, -5.7];

/// Measured opening shape (position rad, |τ| N·m), see the module comment.
const SHAPE: [(f32, f32); 13] = [
    (-6.0, 0.49), (-5.5, 0.46), (-5.0, 0.47), (-4.5, 0.46), (-4.0, 0.46), (-3.5, 0.41), (-3.0, 0.42),
    (-2.5, 0.42), (-2.0, 0.32), (-1.5, 0.27), (-1.0, 0.21), (-0.5, 0.12), (0.0, 0.10),
];

/// Same procedure after lubrication (2026-11-23): opening medians; closing near open 0.57
/// (p90 0.69) at -6.0, 0.48 (p90 0.59) at -5.5.
const SHAPE_LUBE: [(f32, f32); 13] = [
    (-6.0, 0.52), (-5.5, 0.44), (-5.0, 0.46), (-4.5, 0.52), (-4.0, 0.41), (-3.5, 0.44), (-3.0, 0.44),
    (-2.5, 0.38), (-2.0, 0.37), (-1.5, 0.32), (-1.0, 0.32), (-0.5, 0.19), (0.0, 0.15),
];

fn interp(table: &[(f32, f32)], q: f32) -> f32 {
    if q <= table[0].0 { return table[0].1; }
    for w in table.windows(2) {
        let ((q0, t0), (q1, t1)) = (w[0], w[1]);
        if q <= q1 { return t0 + (t1 - t0) * (q - q0) / (q1 - q0); }
    }
    table[table.len() - 1].1
}

fn shape(q: f32) -> f32 { interp(&SHAPE, q) }

#[derive(Clone, Copy, PartialEq)]
enum Finger {
    /// New finger: the measured shape x 1.3 (0.6 N·m near open), closing 4 % higher, noise
    /// ±0.06 N·m per tick (peaks about 0.7 N·m).
    New,
    /// Old finger (2026-11-20): 0.40 N·m whatever the position (as `test_gripper_force`).
    Old,
    /// B601-DM 1.6.0-dev5 run (2026-11-24, 0.3 rad/s, empty jaws): opens peak 0.63-0.69 N·m,
    /// closes 0.72-0.79 N·m near open: the measured shape x 1.35 opening (0.66 at -6.0), closing
    /// 13 % higher (0.75), noise ±0.04 N·m.
    Real,
    /// New finger, `.0` x its torque everywhere (lower friction after lubrication).
    NewScaled(f32),
    /// New finger lubricated: the measured lubricated shape, closing 10 % higher (0.57 N·m at
    /// -6.0), noise ±0.15 N·m per tick (σ 0.087: p90 about 0.11 above the median, as measured).
    Lube,
}

impl Finger {
    fn free(self, q: f32, v: f32, noise: f32) -> f32 {
        if v.abs() <= 0.05 { return 0.0; }
        let base = match self {
            Finger::New => 1.3 * shape(q) * if v > 0.0 { 1.04 } else { 1.0 } + noise,
            Finger::Old => 0.40,
            Finger::Real => 1.35 * shape(q) * if v > 0.0 { 1.13 } else { 1.0 } + 0.67 * noise,
            Finger::NewScaled(f) => f * 1.3 * shape(q) * if v > 0.0 { 1.04 } else { 1.0 } + noise,
            Finger::Lube => interp(&SHAPE_LUBE, q) * if v > 0.0 { 1.1 } else { 1.0 } + 2.5 * noise,
        };
        base.max(0.0) * v.signum()
    }
}

#[derive(Clone, Copy, PartialEq)]
enum Load {
    Free,
    /// Banana-like soft object from `.0`: the jaw keeps following the command into it; the
    /// object pushes back `k1` N·m/rad for the first 0.3 rad, then `k2` beyond.
    Soft(f32, f32, f32),
    /// 2026-11-20 incident (test_gripper_force): from -1.75 rad the torque rises 0.85 -> 5.0 N·m
    /// at -0.5 rad while the jaw keeps moving at ~1 rad/s; at 5 N·m the finger breaks.
    Incident,
    /// Rigid obstacle outside the jaws at `.0` while opening (stiffness `.1` N·m/rad).
    OpenBlock(f32, f32),
    /// Something dragging on the jaw between `.0` and `.1` (rad): `.2` N·m more while moving.
    Drag(f32, f32, f32),
    /// A thin rigid obstruction at `.0` while closing (stiffness `.1` N·m/rad on the command past it).
    CloseBlock(f32, f32),
    /// Soft object from `.0` (k 1.5 N·m/rad) that slips out of the jaws after being pressed for
    /// `.1` ticks (counted in `Rig::pressed`).
    SlipAfter(f32, u32),
}

struct Rig {
    q: [f32; 7], v: [f32; 7], tau: [f32; 7], enabled: EnableState,
    finger: Finger, load: Load, seed: u32, offset: f32,
    broke: bool, peak_tau: f32,
    /// Ticks the SlipAfter object has been pressed.
    pressed: u32,
}

impl Rig {
    fn noise(&mut self) -> f32 {
        self.seed = self.seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        ((self.seed >> 8) as f32 / (1u32 << 24) as f32 - 0.5) * 0.12
    }
}

struct RigDriver(Arc<Mutex<Rig>>);

impl ArmDriver for RigDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true, torque_feedforward: false, enable_feedback: true, load_feedback: false }
    }
    fn supports_rt_executor(&self) -> Result<(), &'static str> { Ok(()) }
    fn enable_state(&self) -> EnableState { self.0.lock().unwrap().enabled }
    fn is_feedback_valid(&self) -> bool { true }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); 7] }
    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
        out.clear();
        out.resize(7, Some(Duration::ZERO));
    }
    fn feedback_all_within(&self, dof: usize, _max_age: Duration) -> bool { dof == 7 }
    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
        out.clear();
        out.resize(7, Some(0));
    }
    fn feedback_instants_into(&self, out: &mut Vec<Option<std::time::Instant>>) {
        out.clear();
        out.resize(7, Some(std::time::Instant::now()));
    }
    fn feedback_seqs_into(&self, out: &mut Vec<u64>) { out.clear(); }
    fn feedback_receive_instants_into(&self, out: &mut Vec<Option<std::time::Instant>>) { out.clear(); }
    fn clear_fault_disables_arm_joint(&self, _arm_dof: usize) -> bool { false }
    fn has_hardware_fault(&self) -> bool { false }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let mut out = Vec::with_capacity(7);
        self.read_joint_states_into(&mut out)?;
        Ok(out)
    }
    fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
        let r = self.0.lock().unwrap();
        out.clear();
        for i in 0..7 {
            out.push(JointState { position: r.q[i], velocity: r.v[i], torque: r.tau[i], temperature: 30.0 });
        }
        Ok(())
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        let mut r = self.0.lock().unwrap();
        for (i, t) in targets.iter().enumerate().take(7) {
            // POS_VEL-like follower: ~0.23 rad lag at 1 rad/s, capped at 1.05 rad/s
            let mut v = ((t.position - r.q[i]) * 4.5).clamp(-1.05, 1.05);
            let mut q = r.q[i] + v * DT as f32;
            if i != 6 { r.q[i] = q; r.v[i] = v; continue; }
            if q > 0.0 { q = 0.0; v = 0.0; }
            let n = r.noise() + r.offset;
            let free = r.finger.free(q, v, n);
            let tau = match r.load {
                Load::Free => free,
                Load::Soft(at, k1, k2) if q >= at => {
                    let pen = q - at;
                    free + k1 * pen.min(0.3) + k2 * (pen - 0.3).max(0.0)
                }
                Load::Soft(..) => free,
                Load::Incident if q >= -1.75 => 0.85 + (q + 1.75) * (5.0 - 0.85) / 1.25,
                Load::Incident => free,
                Load::OpenBlock(at, k) if q <= at => { q = at; v = 0.0; -(k * (at - t.position)).max(0.0) }
                Load::OpenBlock(..) => free,
                Load::Drag(a, b, nm) if q >= a.min(b) && q <= a.max(b) && v.abs() > 0.05 => free + nm * v.signum(),
                Load::Drag(..) => free,
                Load::SlipAfter(at, n) if q >= at => {
                    r.pressed += 1;
                    if r.pressed > n { r.load = Load::Free; free } else { free + 1.5 * (q - at) }
                }
                Load::SlipAfter(..) => free,
                Load::CloseBlock(at, k) if q >= at => { q = at; v = 0.0; (k * (t.position - at)).max(0.0) }
                Load::CloseBlock(..) => free,
            };
            r.q[6] = q;
            r.v[6] = v;
            r.tau[6] = tau;
            r.peak_tau = r.peak_tau.max(tau.abs());
            if r.load == Load::Incident && tau >= 5.0 { r.broke = true; r.load = Load::Free; }
        }
        Ok(())
    }
    fn set_torque(&mut self, on: bool) -> Result<(), ArmDriverError> {
        self.0.lock().unwrap().enabled = if on { EnableState::Enabled } else { EnableState::Disabled };
        Ok(())
    }
}

fn rig(finger: Finger) -> (SupervisorInner, Arc<Mutex<Rig>>) {
    let rig = Arc::new(Mutex::new(Rig { q: INSPECT, v: [0.0; 7], tau: [0.0; 7], enabled: EnableState::Disabled,
        finger, load: Load::Free, seed: 7, offset: 0.0, broke: false, peak_tau: 0.0, pressed: 0 }));
    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(RigDriver(rig.clone())));
    inner.lease_enabled = false;
    // the baseline is bound to the follower kind
    inner.follower = Some("rig".into());
    inner.tick(DT);
    inner.set_torque(true).unwrap();
    (inner, rig)
}

fn req(command: GripperCommand, fraction: Option<f64>, speed: f64) -> GripperRequest {
    GripperRequest { command, fraction, grasp: command == GripperCommand::Close, speed, timeout_s: None, tag: None }
}

/// Run a gripper action to its end, then learn the queued moves (armd does that off the tick
/// within a second); (action, seconds).
fn run_with(inner: &mut SupervisorInner, request: GripperRequest, cmd: GripperBaselineCmd) -> (armd::motion::ActionStatus, f64) {
    let epoch = inner.safety.epoch_id();
    let (id, _) = inner.start_gripper_with_baseline(request, epoch, cmd).unwrap();
    for i in 0..(30.0 / DT) as usize {
        inner.tick(DT);
        let a = inner.last_action.clone().unwrap();
        assert_eq!(a.id, id);
        if !a.is_running() {
            armd::gripper_calib::learn_pending_now(inner);
            return (a, (i + 1) as f64 * DT);
        }
    }
    panic!("gripper action still running: {:?}", inner.last_action);
}

fn run(inner: &mut SupervisorInner, request: GripperRequest) -> (armd::motion::ActionStatus, f64) {
    run_with(inner, request, GripperBaselineCmd::None)
}

const INCIDENT_RISE_PER_TICK_NM: f32 = (5.0 - 0.85) / 1.25 * 1.05 * DT as f32;

fn cap() -> f32 { GripperPolicy::rebot_dm().max_close_torque_nm.unwrap() }

/// Calibration strokes (`"baseline": "calibrate"`, nothing in the jaws): closes (move to 5 %)
/// and opens at the default speed and at the contact speed (slow band), with a different
/// run-to-run offset each time.
fn learn(inner: &mut SupervisorInner, rig: &Arc<Mutex<Rig>>, rounds: usize) {
    for k in 0..rounds {
        for speed in [1.0, 0.3] {
            rig.lock().unwrap().offset = [0.03, -0.02, 0.0, 0.02][k % 4];
            let (a, _) = run_with(inner, req(GripperCommand::Move, Some(0.05), speed), GripperBaselineCmd::Calibrate);
            assert_eq!(a.state, "done", "{a:?}");
            let (a, _) = run_with(inner, req(GripperCommand::Open, None, speed), GripperBaselineCmd::Calibrate);
            assert_eq!(a.state, "done", "{a:?}");
        }
    }
    rig.lock().unwrap().offset = 0.0;
}

fn baseline_obs(inner: &SupervisorInner) -> armd::model::GripperBaselineObs {
    inner.current_observation().gripper.unwrap().baseline.unwrap()
}

/// New finger, nothing learned: the close uses the uncalibrated threshold (contact_torque_nm +
/// 0.25 = 0.85 N·m) and runs at speed, is not learned; "unlearned" until a move is learned.
/// Plain opens teach the opening direction; closes stay on the uncalibrated threshold.
#[test]
fn new_finger_unlearned_uses_the_absolute_rule_and_learns_from_opens() {
    let (mut inner, rig) = rig(Finger::New);
    assert_eq!(baseline_obs(&inner).state, "unlearned");
    let (a, t) = run(&mut inner, req(GripperCommand::Close, None, 1.0));
    let c = inner.gripper_last_contact;
    eprintln!("unlearned air close: {} {:?} in {t:.1} s, contact {c:?}", a.state, a.code);
    assert_eq!((a.state.as_str(), a.holding), ("done", Some(false)), "{a:?}");
    if let Some(c) = c { assert!(c.threshold_source == ThresholdSource::Uncalibrated && c.at_rad > -0.35, "{c:?}"); }
    assert_eq!(baseline_obs(&inner).state, "unlearned");
    assert!(t < 7.5, "{t:.1} s");
    rig.lock().unwrap().q[6] = 0.0; // (jaw back to closed for a full opening stroke)
    inner.commanded_positions[6] = 0.0;
    inner.target_positions[6] = 0.0;
    inner.tick(DT);
    // one plain open: the closing direction uses the opening tables (+0.08 N·m), and every
    // further close/open cycle keeps it so (closes are not learned)
    for k in 0..4 {
        let (a, _) = run(&mut inner, req(GripperCommand::Open, None, 1.0));
        assert_eq!(a.state, "done");
        // the first open sets the bins (at most contact_torque_nm - 0.15 = 0.45 N·m); later opens
        // more than the contact margin above that are not learned (only a calibration raises it)
        let obs = baseline_obs(&inner);
        assert!(obs.open_moves >= 1 && obs.close_moves == 0, "{obs:?}");
        inner.gripper_last_contact = None;
        let (a, t) = run(&mut inner, req(GripperCommand::Close, None, 1.0));
        let c = inner.gripper_last_contact;
        eprintln!("close after {} open(s): {} in {t:.1} s, contact {c:?}", k + 1, a.state);
        assert_eq!((a.state.as_str(), a.holding), ("done", Some(false)), "{a:?}");
        assert!(t < 7.5, "{t:.1} s");
        if let Some(c) = c { assert!(c.at_rad > -0.35, "early contact {c:?}"); }
    }
    // a grasp close is never learned
    assert_eq!(baseline_obs(&inner).close_moves, 0);
}

/// New finger after calibration strokes: an air grasp-close runs at the requested speed to
/// closed (no contact verdict before the end); it is not learned.
#[test]
fn new_finger_air_close_runs_at_speed_after_learning() {
    let (mut inner, rig) = rig(Finger::New);
    learn(&mut inner, &rig, 2);
    let obs = baseline_obs(&inner);
    assert_eq!(obs.state, "learned", "{obs:?}");
    let before = obs.close_moves;
    for _ in 0..3 {
        inner.gripper_last_contact = None;
        let (a, t) = run(&mut inner, req(GripperCommand::Close, None, 1.0));
        eprintln!("learned air close: {t:.1} s, contact {:?}, peak {:.2}", inner.gripper_last_contact, rig.lock().unwrap().peak_tau);
        assert_eq!((a.state.as_str(), a.holding), ("done", Some(false)), "{a:?}");
        // 5.7 rad at 1 rad/s plus lag, settle and squeeze window
        assert!(t < 7.5, "{t:.1} s");
        if let Some(c) = inner.gripper_last_contact { assert!(c.at_rad > -0.35, "early contact {c:?}"); }
        assert_eq!(inner.gripper_force_trips, 0);
        let (a, _) = run(&mut inner, req(GripperCommand::Open, None, 1.0));
        assert_eq!(a.state, "done");
    }
    // grasp closes are never learned
    assert_eq!(baseline_obs(&inner).close_moves, before);
}

/// A banana-like soft object at -2.0 rad: detected from the torque above the baseline, squeezed
/// and held with the torque below the cap; the held close is not learned.
#[test]
fn new_finger_soft_object_is_detected_and_held_under_the_cap() {
    let (mut inner, rig) = rig(Finger::New);
    learn(&mut inner, &rig, 2);
    let estimate = |inner: &SupervisorInner| inner.gripper_baseline.as_ref().unwrap().estimate(-1.5, Direction::Close, 1.0).unwrap().baseline_nm;
    let (close_moves, base_before) = (baseline_obs(&inner).close_moves, estimate(&inner));
    let mut saw_baseline_contact = false;
    for (k1, k2) in [(1.5_f32, 8.0_f32), (0.8, 4.0), (3.0, 20.0)] {
        { let mut r = rig.lock().unwrap(); r.load = Load::Soft(-2.0, k1, k2); r.peak_tau = 0.0; }
        inner.gripper_last_contact = None;
        let trips_before = inner.gripper_force_trips;
        let (a, t) = run(&mut inner, req(GripperCommand::Close, None, 1.0));
        let c = inner.gripper_last_contact;
        let peak = rig.lock().unwrap().peak_tau;
        eprintln!("soft k1 {k1}: {} holding {:?} in {t:.1} s at {:.3} rad, contact {c:?}, peak {peak:.2}", a.state, a.holding, inner.measured_positions[6]);
        assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "k1 {k1}: {a:?}");
        match c {
            Some(c) => {
                assert!(c.at_rad > -2.05 && c.at_rad < -1.3, "{c:?}");
                if c.source == ContactSource::Torque {
                    assert_eq!(c.threshold_source, ThresholdSource::Baseline);
                    assert!(c.margin_nm.unwrap() > 0.1, "{c:?}");
                    saw_baseline_contact = true;
                }
            }
            None => {
                // With J7's 0.82 N·m moving floor + 3-tick requirement, a stiff piecewise ramp
                // can cross the 1-tick torque cap before 3 moving ticks elapse; the grasp still
                // catches and holds the object cleanly under the cap.
                assert!(inner.gripper_force_trips > trips_before, "k1 {k1}: neither contact nor cap trip");
                assert!(inner.measured_positions[6] > -2.05 && inner.measured_positions[6] < -1.3,
                    "k1 {k1}: J7 {}", inner.measured_positions[6]);
            }
        }
        // the cap latches after GRIPPER_FORCE_CAP_TICKS ticks (test_gripper_force CAP_MARGIN_NM)
        assert!(peak < cap() + 0.25, "peak {peak:.3}");
        // and the grip settles at or below the cap
        rig.lock().unwrap().peak_tau = 0.0;
        for _ in 0..50 { inner.tick(DT); }
        assert!(rig.lock().unwrap().peak_tau <= cap() + 1e-3);
        assert!(t < 8.0, "{t:.1} s");
        // holding: not learned
        assert_eq!(baseline_obs(&inner).close_moves, close_moves);
        // let go (the open from the object is learned without the squeeze at its start)
        rig.lock().unwrap().load = Load::Free;
        let (a, _) = run(&mut inner, req(GripperCommand::Open, None, 1.0));
        assert_eq!(a.state, "done");
    }
    assert!(saw_baseline_contact, "expected at least one soft case to trigger Baseline contact before cap");
    assert!((estimate(&inner) - base_before).abs() < 0.02, "{} -> {}", base_before, estimate(&inner));
}

/// The incident profile (torque rising to 5 N·m while the jaw moves at ~1 rad/s) still stops
/// at the cap with a learned baseline, for both fingers.
#[test]
fn incident_still_stops_at_the_cap_with_a_baseline() {
    for finger in [Finger::New, Finger::Old] {
        let (mut inner, rig) = rig(finger);
        learn(&mut inner, &rig, 2);
        { let mut r = rig.lock().unwrap(); r.load = Load::Incident; r.peak_tau = 0.0; }
        for grasp in [true, false] {
            let request = GripperRequest { grasp, ..req(GripperCommand::Close, None, 1.0) };
            let (a, _) = run(&mut inner, request);
            let (peak, broke) = { let r = rig.lock().unwrap(); (r.peak_tau, r.broke) };
            assert!(!broke, "finger broke");
            // the first sample over the cap latches: at most one tick of rise above it
            // (incident slope 3.32 N·m/rad x 1.05 rad/s x 20 ms = 0.07 N·m)
            assert!(peak <= cap() + INCIDENT_RISE_PER_TICK_NM + 1e-3, "peak {peak:.3} N·m");
            if grasp {
                assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{a:?}");
            } else {
                // Non-grasp close stalled >= 0.25 rad short of closed fails GRIPPER_TIMEOUT
                // (matching test_gripper_force::incident_close_without_grasp_detection_stops_at_the_cap).
                assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some("GRIPPER_TIMEOUT")), "{a:?}");
            }
            assert!(inner.measured_positions[6] < -1.0, "J7 {}", inner.measured_positions[6]);
            rig.lock().unwrap().load = Load::Free;
            let (a, _) = run(&mut inner, req(GripperCommand::Open, None, 1.0));
            assert_eq!(a.state, "done", "{a:?}");
            rig.lock().unwrap().load = Load::Incident;
        }
    }
}

/// Old finger (0.40 N·m everywhere): while moving, both calibrated and uncalibrated respect J7's
/// 0.82 N·m moving floor and catch/hold a soft object under the cap; once stopped on a rigid
/// object, calibrated triggers at baseline + margin (0.55 N·m) and uncalibrated at
/// contact_torque_nm (0.60 N·m).
#[test]
fn old_finger_contact_unchanged_by_the_baseline() {
    let contact = |learned: bool, load: Load| {
        let (mut inner, rig) = rig(Finger::Old);
        if learned { learn(&mut inner, &rig, 2); }
        rig.lock().unwrap().load = load;
        let epoch = inner.safety.epoch_id();
        let (id, _) = inner.start_gripper(req(GripperCommand::Close, None, 1.0), epoch).unwrap();
        let mut first = None;
        loop {
            inner.tick(DT);
            if first.is_none() { first = inner.gripper_last_contact; }
            let a = inner.last_action.clone().unwrap();
            assert_eq!(a.id, id);
            if !a.is_running() {
                assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{a:?}");
                break;
            }
        }
        let peak = rig.lock().unwrap().peak_tau;
        (first, peak, inner.gripper_force_trips, inner.measured_positions[6])
    };
    // Moving soft object: both enforce J7's 0.82 N·m moving floor (neither falsely triggers at
    // 0.55-0.80 N·m while moving) and catch/hold cleanly under cap + margin.
    let (c0, p0, trips0, q0) = contact(false, Load::Soft(-2.0, 1.5, 8.0));
    let (c1, p1, trips1, q1) = contact(true, Load::Soft(-2.0, 1.5, 8.0));
    eprintln!("old finger soft: unlearned {c0:?} peak {p0:.2} trips {trips0} q {q0:.3}; calibrated {c1:?} peak {p1:.2} trips {trips1} q {q1:.3}");
    for (label, c, trips, q) in [("uncalibrated", c0, trips0, q0), ("calibrated", c1, trips1, q1)] {
        match c {
            Some(c) => assert!(c.torque_nm >= armd::motion::GRIPPER_MOVING_CONTACT_TORQUE_FLOOR_NM - 1e-4, "{label}: {c:?}"),
            None => assert!(trips >= 1 && q > -2.0 && q < -1.5, "{label}: neither moving contact nor cap trip (q={q})"),
        }
    }
    assert!(p0 <= cap() + 0.25 && p1 <= cap() + 0.25, "{p0} {p1}");

    // Stopped rigid contact at -2.0 rad: calibrated uses baseline + margin (0.55 N·m),
    // uncalibrated uses contact_torque_nm (0.60 N·m).
    let (s0, _, _, _) = contact(false, Load::CloseBlock(-2.0, 3.0));
    let (s1, _, _, _) = contact(true, Load::CloseBlock(-2.0, 3.0));
    let s0 = s0.expect("uncalibrated stopped contact");
    let s1 = s1.expect("calibrated stopped contact");
    assert_eq!(s0.threshold_source, ThresholdSource::Uncalibrated);
    assert_eq!(s1.threshold_source, ThresholdSource::Baseline);
    assert!((s1.threshold_nm.unwrap() - 0.55).abs() < 0.03, "{s1:?}");
    assert!((s0.threshold_nm.unwrap() - 0.60).abs() < 1e-3, "{s0:?}");
    assert!((s1.at_rad - (-2.0)).abs() < 0.02 && (s0.at_rad - (-2.0)).abs() < 0.02);
}

/// After lubrication (free torque 0.65 x everywhere) the baseline follows down over a few
/// unloaded moves.
#[test]
fn baseline_follows_lower_friction() {
    let (mut inner, rig) = rig(Finger::New);
    learn(&mut inner, &rig, 4);
    let at = |inner: &SupervisorInner| inner.gripper_baseline.as_ref().unwrap().estimate(-4.5, Direction::Close, 1.0).unwrap().baseline_nm;
    let high = at(&inner);
    rig.lock().unwrap().finger = Finger::NewScaled(0.65);
    learn(&mut inner, &rig, 4);
    let low = at(&inner);
    eprintln!("baseline at -4.5 close: {high:.3} -> {low:.3}");
    assert!(high > 0.55 && (low - 0.65 * high).abs() < 0.05, "{high} -> {low}");
}

/// The baseline is written next to the gripper calibration, one file per follower, and loaded
/// again only for the same tool and follower; files without identity are ignored.
#[tokio::test]
async fn baseline_persists_and_is_tool_and_follower_bound() {
    let dir = tempfile::tempdir().unwrap();
    let calib = dir.path().join("gripper_calib.json");
    let file = dir.path().join("gripper_baseline.rig.json");
    assert_eq!(armd::gripper_calib::baseline_path_for(&calib, Some("rig")), Some(file.clone()));
    assert_eq!(armd::gripper_calib::baseline_path_for(&calib, None), None);
    let (mut inner, rig) = rig(Finger::New);
    inner.gripper_baseline_path = Some(file.clone());
    learn(&mut inner, &rig, 1);
    let learned = inner.gripper_baseline.clone().unwrap();
    let (path, snap) = armd::gripper_calib::take_baseline_snapshot(&mut inner).expect("dirty baseline");
    assert!(armd::gripper_calib::take_baseline_snapshot(&mut inner).is_none());
    assert_eq!(path, file);
    std::fs::write(&file, serde_json::to_string(&snap).unwrap()).unwrap();

    let load = |follower: &'static str| {
        let calib = calib.clone();
        async move {
            let sup = armd::ArmSupervisor::new(ArmConfig::rebot_dm());
            sup.inner().write().await.follower = Some(follower.into());
            armd::gripper_calib::load_into(&sup, &calib).await;
            let g = sup.inner().read().await;
            (g.gripper_baseline.clone(), g.gripper_baseline_observation().state.as_str().to_string())
        }
    };
    assert_eq!(load("rig").await.0.as_ref(), Some(&learned));
    // another follower reads its own (missing) file
    assert_eq!(load("damiao").await, (None, "unlearned".to_string()));
    // the same file with another follower, another tool, or no identity: ignored
    for edit in 0..3 {
        let mut other = learned.clone();
        match edit {
            0 => other.follower = Some("damiao".into()),
            1 => { other.tool_id = Some("suction".into()); other.tool_config_digest = Some("0".repeat(64)); }
            _ => { other.tool_id = None; other.tool_config_digest = None; other.follower = None; }
        }
        std::fs::write(&file, serde_json::to_string(&other).unwrap()).unwrap();
        assert_eq!(load("rig").await, (None, "unlearned".to_string()), "edit {edit}");
    }
}

/// A refused action with "baseline": "reset" leaves the learned baseline as it was.
#[test]
fn reset_happens_only_after_admission() {
    let (mut inner, rig) = rig(Finger::New);
    learn(&mut inner, &rig, 1);
    let before = inner.gripper_baseline.clone();
    let epoch = inner.safety.epoch_id();
    assert!(inner.start_gripper_with_baseline(req(GripperCommand::Open, None, 1.0), epoch + 1, GripperBaselineCmd::Reset).is_err());
    assert_eq!(inner.gripper_baseline, before);
    let (a, _) = run_with(&mut inner, req(GripperCommand::Open, None, 1.0), GripperBaselineCmd::Reset);
    assert_eq!(a.state, "done");
    assert_eq!(baseline_obs(&inner).close_moves, 0);
}

/// Ordinary non-grasp closes and fractional moves that compress something are not learned;
/// only opens and calibration strokes are.
#[test]
fn ordinary_closing_moves_are_not_learned() {
    let (mut inner, rig) = rig(Finger::New);
    rig.lock().unwrap().load = Load::Soft(-3.0, 0.3, 0.3);
    let (a, _) = run(&mut inner, req(GripperCommand::Move, Some(0.05), 1.0));
    assert_eq!(a.state, "done", "{a:?}");
    let obs = baseline_obs(&inner);
    assert_eq!((obs.close_moves, obs.state.as_str()), (0, "unlearned"), "{obs:?}");
}

/// An object that slips out of the jaws after the contact, early or late in the squeeze, is not
/// reported held; the close reaches the object first (contact near -2.0 rad).
#[test]
fn slipped_object_is_not_held() {
    for (finger, learned) in [(Finger::Old, false), (Finger::New, true)] {
        for after in [3u32, 8, 14, 20] {
            let (mut inner, rig) = rig(finger);
            if learned { learn(&mut inner, &rig, 3); }
            rig.lock().unwrap().load = Load::SlipAfter(-2.0, after);
            let epoch = inner.safety.epoch_id();
            let (id, _) = inner.start_gripper(req(GripperCommand::Close, None, 1.0), epoch).unwrap();
            let mut first = None;
            let a = loop {
                inner.tick(DT);
                if first.is_none() { first = inner.gripper_last_contact; }
                let a = inner.last_action.clone().unwrap();
                assert_eq!(a.id, id);
                if !a.is_running() { break a; }
            };
            let slipped = rig.lock().unwrap().load == Load::Free;
            eprintln!("slip after {after} ticks (learned {learned}): {} {:?} holding {:?} at {:.3}, first contact {:?}, slipped {slipped}",
                a.state, a.code, a.holding, inner.measured_positions[6], first.map(|c| c.at_rad));
            // the close reached and pressed the object
            assert!(rig.lock().unwrap().pressed > 0, "after {after}: never reached the object");
            if let Some(c) = first { assert!(c.at_rad > -2.1 && c.at_rad < -1.6, "{c:?}"); }
            if slipped { assert_ne!(a.holding, Some(true), "after {after}, learned {learned}: {a:?}"); }
        }
    }
}

/// Opening into an obstacle outside the jaws is stopped under the cap + one tick of rise (torque
/// cap on the torque pushing open, opening stall threshold not raised by the baseline), and the
/// blocked open is not learned.
#[test]
fn opening_into_an_obstacle_is_capped() {
    let (mut inner, rig) = rig(Finger::New);
    learn(&mut inner, &rig, 2);
    let (a, _) = run(&mut inner, req(GripperCommand::Move, Some(0.05), 1.0));
    assert_eq!(a.state, "done");
    // stiffness 3 N·m/rad: the impact sample (stiffness x POS_VEL lag, 0.7 N·m) stays under the
    // cap; the command running on then raises the torque 0.063 N·m per tick
    { let mut r = rig.lock().unwrap(); r.load = Load::OpenBlock(-2.0, 3.0); r.peak_tau = 0.0; }
    let (trips, opens) = (inner.gripper_force_trips, baseline_obs(&inner).open_moves);
    let (a, _) = run(&mut inner, req(GripperCommand::Open, None, 1.0));
    let peak = rig.lock().unwrap().peak_tau;
    eprintln!("open into obstacle: {} {:?}, peak {peak:.2}, J7 {:.3}", a.state, a.code, inner.measured_positions[6]);
    // stopped by the cap or by the (not raised) opening stall threshold
    assert!(inner.gripper_force_trips > trips || inner.gripper_stalled, "not stopped, peak {peak:.3}");
    assert!(peak <= cap() + 3.0 * 1.05 * DT as f32 + 1e-3, "peak {peak:.3}");
    // the blocked open is not learned
    assert_eq!(baseline_obs(&inner).open_moves, opens);
    // and the jaw holds there
    rig.lock().unwrap().peak_tau = 0.0;
    for _ in 0..50 { inner.tick(DT); }
    assert!(rig.lock().unwrap().peak_tau <= cap());
}

/// An explicit timeout_s stays the deadline.
#[test]
fn explicit_timeout_is_not_extended() {
    let (mut inner, rig) = rig(Finger::New);
    rig.lock().unwrap().load = Load::Soft(-5.0, 0.3, 0.3); // a very soft object near open: long creep
    let request = GripperRequest { timeout_s: Some(3.0), ..req(GripperCommand::Close, None, 1.0) };
    let (a, t) = run(&mut inner, request);
    if a.state == "failed" { assert_eq!(a.code.as_deref(), Some("GRIPPER_TIMEOUT")); assert!(t <= 3.1, "{t}"); }
}


/// Lubricated finger: near open the free torque stays 0.45-0.57 N·m with p90 up to 0.69; after
/// learning, air closes run at speed without an early contact and a soft object is still caught.
#[test]
fn lubricated_finger_spread_near_open() {
    let (mut inner, rig) = rig(Finger::Lube);
    learn(&mut inner, &rig, 2);
    let e = inner.gripper_baseline.as_ref().unwrap().estimate(-5.6, Direction::Close, 1.0).unwrap();
    eprintln!("lube close baseline at -5.6: {e:?}, threshold {:.3}", e.threshold_nm(Some(cap())));
    for _ in 0..5 {
        inner.gripper_last_contact = None;
        let (a, t) = run(&mut inner, req(GripperCommand::Close, None, 1.0));
        assert_eq!((a.state.as_str(), a.holding), ("done", Some(false)), "{a:?}");
        assert!(t < 7.5, "{t:.1} s");
        if let Some(c) = inner.gripper_last_contact { assert!(c.at_rad > -0.35, "early contact {c:?}"); }
        let (a, _) = run(&mut inner, req(GripperCommand::Open, None, 1.0));
        assert_eq!(a.state, "done");
    }
    { let mut r = rig.lock().unwrap(); r.load = Load::Soft(-3.0, 1.5, 8.0); r.peak_tau = 0.0; }
    let (a, _) = run(&mut inner, req(GripperCommand::Close, None, 1.0));
    eprintln!("lube soft at -3.0: {:?}, J7 {:.3}, trips {}, peak {:.2}",
        inner.gripper_last_contact, inner.measured_positions[6], inner.gripper_force_trips, rig.lock().unwrap().peak_tau);
    assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{a:?}");
    match inner.gripper_last_contact {
        Some(c) => assert!(c.at_rad > -3.05 && c.at_rad < -2.4, "{c:?}"),
        None => {
            // High ±0.15 N·m noise on the lubricated finger can trip the 1-tick torque cap before
            // 3 consecutive ticks stay above baseline + margin; the jaw still catches the object.
            assert!(inner.gripper_force_trips >= 1, "neither contact nor cap trip");
            assert!(inner.measured_positions[6] > -3.05 && inner.measured_positions[6] < -2.4,
                "J7 {}", inner.measured_positions[6]);
        }
    }
    assert!(rig.lock().unwrap().peak_tau < cap() + 0.25);
}


/// Lubricated finger (p90 0.69 N·m near open, closing 10 % above opening): one plain open is
/// enough for air closes to run at speed on the opening tables.
#[test]
fn lubricated_finger_one_open_then_air_closes() {
    let (mut inner, rig) = rig(Finger::Lube);
    rig.lock().unwrap().q[6] = 0.0;
    inner.commanded_positions[6] = 0.0;
    inner.target_positions[6] = 0.0;
    inner.tick(DT);
    let (a, _) = run(&mut inner, req(GripperCommand::Open, None, 1.0));
    assert_eq!(a.state, "done");
    for _ in 0..5 {
        inner.gripper_last_contact = None;
        let (a, t) = run(&mut inner, req(GripperCommand::Close, None, 1.0));
        assert_eq!((a.state.as_str(), a.holding), ("done", Some(false)), "{a:?}");
        assert!(t < 7.5, "{t:.1} s");
        if let Some(c) = inner.gripper_last_contact { assert!(c.at_rad > -0.35, "early contact {c:?}"); }
        let (a, _) = run(&mut inner, req(GripperCommand::Open, None, 1.0));
        assert_eq!(a.state, "done");
    }
    assert_eq!(baseline_obs(&inner).close_moves, 0);
}

// ---- gripper_calibrate action (orchestrator in armd) ----

/// Supervisor with the rig driver, ticked at 20 ms of simulated time every millisecond.
async fn calib_rig(finger: Finger) -> (Arc<armd::ArmSupervisor>, Arc<Mutex<Rig>>) {
    let rig = Arc::new(Mutex::new(Rig { q: INSPECT, v: [0.0; 7], tau: [0.0; 7], enabled: EnableState::Disabled,
        finger, load: Load::Free, seed: 11, offset: 0.0, broke: false, peak_tau: 0.0, pressed: 0 }));
    let sup = Arc::new(armd::ArmSupervisor::new(ArmConfig::rebot_dm()));
    sup.set_driver(Box::new(RigDriver(rig.clone()))).await;
    {
        let mut inner = sup.inner().write().await;
        inner.lease_enabled = false;
        inner.follower = Some("rig".into());
        inner.tick(DT);
        inner.set_torque(true).unwrap();
    }
    let ticker = sup.clone();
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(1));
        loop { interval.tick().await; ticker.tick(DT).await; }
    });
    (sup, rig)
}

async fn calib_wait(sup: &armd::ArmSupervisor, id: u64, mut each: impl FnMut(&armd::motion::ActionStatus)) -> armd::motion::ActionStatus {
    let deadline = std::time::Instant::now() + Duration::from_secs(120);
    loop {
        assert!(std::time::Instant::now() < deadline, "calibration {id} not finished");
        let a = { let inner = sup.inner().read().await; inner.last_action.clone().unwrap() };
        assert_eq!(a.id, id);
        each(&a);
        if !a.is_running() { return a; }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn calibration_action_learns_from_free_strokes() {
    let (sup, _rig) = calib_rig(Finger::New).await;
    let (id, expected) = armd::gripper_calib::start_calibration(&sup).await.unwrap();
    assert!(expected > 60.0, "{expected}");
    let mut phases = Vec::new();
    let a = calib_wait(&sup, id, |a| { if let Some(p) = a.phase { if phases.last() != Some(&p) { phases.push(p); } } }).await;
    let inner = sup.inner().read().await;
    let obs = inner.gripper_baseline_observation();
    eprintln!("calibration: {} {:?}, {} phases, baseline {obs:?}", a.state, a.code, phases.len());
    assert_eq!(a.state, "done", "{a:?}");
    assert!(phases.iter().any(|p| p == "stroke:14/14"), "{phases:?}");
    assert_eq!(obs.state, "learned", "{obs:?}");
    assert!(inner.gripper_baseline_dirty);
}

/// A compliant obstacle in the jaws (or against them) fails the calibration with
/// CALIBRATION_OBSTRUCTED and leaves the learned baseline as it was; Stop cancels it the same way.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn calibration_action_obstructed_or_stopped_learns_nothing() {
    for case in ["soft", "drag", "stop"] {
        let (sup, rig) = calib_rig(Finger::New).await;
        let before = {
            let mut inner = sup.inner().write().await;
            inner.gripper_baseline_for_motion();
            inner.gripper_baseline.clone()
        };
        if case == "soft" { rig.lock().unwrap().load = Load::Soft(-3.0, 1.0, 4.0); }
        let (id, _) = armd::gripper_calib::start_calibration(&sup).await.unwrap();
        let mut stopped = false;
        let sup2 = sup.clone();
        let a = calib_wait(&sup, id, |a| {
            if case == "drag" && !stopped && a.phase.as_deref() == Some("stroke:6/14") {
                stopped = true;
                rig.lock().unwrap().load = Load::Drag(-2.5, -0.8, 0.25);
            }
            if case == "stop" && !stopped && a.phase.as_deref() == Some("stroke:4/14") {
                stopped = true;
                let sup3 = sup2.clone();
                tokio::spawn(async move { let mut inner = sup3.inner().write().await; inner.stop_motion(); inner.request_macro_stop(); });
            }
        }).await;
        // the orchestrator ends its run record once it sees the end
        for _ in 0..200 {
            if sup.inner().read().await.gripper_calibration.is_none() { break; }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        let inner = sup.inner().read().await;
        eprintln!("{case}: {} {:?} {:?}", a.state, a.code, a.message);
        match case {
            "soft" | "drag" => assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some("CALIBRATION_OBSTRUCTED")), "{a:?}"),
            _ => assert_eq!((a.state.as_str(), a.code.as_deref()), ("cancelled", Some("STOPPED")), "{a:?}"),
        }
        assert_eq!(inner.gripper_baseline, before, "{case}: baseline changed");
        assert!(inner.gripper_calibration.is_none());
    }
}

/// A new gripper zero (or a gripper calibration offset that moves the stroke) invalidates the
/// position-indexed baseline; a zero of another joint keeps it.
#[test]
fn gripper_zero_change_invalidates_the_baseline() {
    let (mut inner, rig) = rig(Finger::New);
    learn(&mut inner, &rig, 1);
    assert_ne!(baseline_obs(&inner).state, "unlearned");
    // zeros are set with torque off
    inner.is_torqued = false;
    let mut offsets = vec![0.0f32; 7];
    offsets[2] = 0.01;
    inner.set_joint_zero_offsets(&offsets).unwrap();
    assert_ne!(baseline_obs(&inner).state, "unlearned", "another joint's zero");
    offsets[6] = 0.05;
    inner.set_joint_zero_offsets(&offsets).unwrap();
    assert_eq!(baseline_obs(&inner).state, "unlearned");
    assert!(inner.gripper_baseline_dirty);
    inner.set_joint_zero_offsets(&[0.0; 7]).unwrap();
    inner.tick(DT);
    inner.set_torque(true).unwrap();
    // a moved stroke (gripper calibration offset): the next motion starts unlearned
    learn(&mut inner, &rig, 1);
    let p = inner.config.gripper_policy.as_mut().unwrap();
    p.closed_rad += 0.02;
    p.open_rad += 0.02;
    inner.gripper_baseline_for_motion();
    assert_eq!(baseline_obs(&inner).state, "unlearned");
}

/// A thin obstruction 0.10 rad before the closing stroke's target (5 % open): whether the jaw
/// stops on it below the stall threshold (3 N·m/rad: 0.3 N·m) or stalls / trips the cap on it
/// (20 N·m/rad), the first close fails CALIBRATION_OBSTRUCTED, no further stroke runs and nothing
/// is learned.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn calibration_fails_on_a_thin_obstruction_before_the_closed_stroke_end() {
    let p = GripperPolicy::rebot_dm();
    let target = p.closed_rad + 0.05 * (p.open_rad - p.closed_rad);
    for k in [3.0f32, 20.0] {
        let (sup, rig) = calib_rig(Finger::New).await;
        let before = { let mut inner = sup.inner().write().await; inner.gripper_baseline_for_motion(); inner.gripper_baseline.clone() };
        rig.lock().unwrap().load = Load::CloseBlock(target - 0.10, k);
        let (id, _) = armd::gripper_calib::start_calibration(&sup).await.unwrap();
        let a = calib_wait(&sup, id, |_| {}).await;
        for _ in 0..200 {
            if sup.inner().read().await.gripper_calibration.is_none() { break; }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        let inner = sup.inner().read().await;
        eprintln!("thin obstruction k {k}: {} {:?} {:?} (phase {:?})", a.state, a.code, a.message, a.phase);
        assert_eq!((a.state.as_str(), a.code.as_deref(), a.phase.as_deref()), ("failed", Some("CALIBRATION_OBSTRUCTED"), Some("stroke:1/14")), "k {k}: {a:?}");
        assert_eq!(inner.gripper_baseline, before, "k {k}: baseline changed");
        // the stroke's leading command was cleared: J7 holds the measured jaw, nothing presses
        let (cmd, meas) = (inner.commanded_positions[6], inner.measured_positions[6]);
        drop(inner);
        eprintln!("thin obstruction k {k}: J7 command {cmd:.4}, jaw {meas:.4}");
        assert!((cmd - meas).abs() < 1e-3, "k {k}: command {cmd} vs jaw {meas}");
        rig.lock().unwrap().peak_tau = 0.0;
        // the ticker runs a 20 ms tick every millisecond: 200 ms of wall time covers 50+ ticks
        tokio::time::sleep(Duration::from_millis(200)).await;
        let peak = rig.lock().unwrap().peak_tau;
        assert!(peak <= 0.01, "k {k}: still pressing, {peak} N·m");
    }
}

/// A plain fractional move that stops short of its target on an object (within the 0.25 rad
/// done tolerance) ends done; on the next tick J7 holds the measured jaw and the object is not
/// pressed any more.
#[test]
fn move_that_stops_short_on_an_object_releases_its_lead() {
    let p = GripperPolicy::rebot_dm();
    let target = p.closed_rad + 0.05 * (p.open_rad - p.closed_rad);
    for k in [3.0f32, 20.0] {
        let (mut inner, rig) = rig(Finger::New);
        rig.lock().unwrap().load = Load::CloseBlock(target - 0.10, k);
        let epoch = inner.safety.epoch_id();
        let (id, _) = inner.start_gripper(req(GripperCommand::Move, Some(0.05), 0.3), epoch).unwrap();
        let a = loop {
            inner.tick(DT);
            let a = inner.last_action.clone().unwrap();
            assert_eq!(a.id, id);
            if !a.is_running() { break a; }
        };
        let lead_at_end = inner.commanded_positions[6] - inner.measured_positions[6];
        inner.tick(DT);
        let (cmd, meas) = (inner.commanded_positions[6], inner.measured_positions[6]);
        eprintln!("move short k {k}: {} {:?}, lead at the end {lead_at_end:.3} rad, one tick later command {cmd:.4} jaw {meas:.4}", a.state, a.code);
        assert!((cmd - meas).abs() < 1e-3, "k {k}: command {cmd} vs jaw {meas}");
        rig.lock().unwrap().peak_tau = 0.0;
        for _ in 0..50 { inner.tick(DT); }
        let peak = rig.lock().unwrap().peak_tau;
        assert!(peak <= 0.01, "k {k}: still pressing, {peak} N·m");
    }
}

/// The real-arm run on 1.6.0-dev5 (no calibration): opens and grasp-closes at 0.3 rad/s with
/// the closing torque 0.72-0.79 N·m near open. Before a closing calibration the closing contact
/// threshold is at least contact_torque_nm + 0.25 = 0.85 N·m (it was 0.711 from the opening
/// tables: false contact at -5.56 rad, creep, GRIPPER_TIMEOUT after 22 s); every close finishes
/// within the default deadline, which covers the full stroke at 0.3 rad/s.
#[test]
fn real_profile_slow_closes_before_calibration() {
    let (mut inner, rig) = rig(Finger::Real);
    let slow = |command, fraction| GripperRequest { command, fraction, grasp: command == GripperCommand::Close, speed: 0.3, timeout_s: None, tag: None };
    let (mut peak_open, mut peak_close) = (0.0f32, 0.0f32);
    for k in 0..6 {
        rig.lock().unwrap().peak_tau = 0.0;
        let (a, t) = run(&mut inner, slow(GripperCommand::Open, None));
        peak_open = peak_open.max(rig.lock().unwrap().peak_tau);
        assert_eq!(a.state, "done", "open {k}: {a:?}");
        assert!(t < 25.0, "open {k}: {t:.1} s");
        inner.gripper_last_contact = None;
        rig.lock().unwrap().peak_tau = 0.0;
        let (a, t) = run(&mut inner, slow(GripperCommand::Close, None));
        peak_close = peak_close.max(rig.lock().unwrap().peak_tau);
        let c = inner.gripper_last_contact;
        eprintln!("real close {k}: {} {:?} in {t:.1} s, holding {:?}, contact {c:?}", a.state, a.code, a.holding);
        assert_eq!((a.state.as_str(), a.holding), ("done", Some(false)), "close {k}: {a:?}");
        if let Some(c) = c { assert!(c.at_rad > -0.35, "close {k}: early contact {c:?}"); }
        assert!(t < 25.0, "close {k}: {t:.1} s");
    }
    let obs = baseline_obs(&inner);
    eprintln!("real profile: opens peak {peak_open:.2}, closes peak {peak_close:.2} N·m; {obs:?}");
    assert!(peak_close > 0.72 && peak_close < 0.85, "{peak_close}");
    assert_eq!(obs.close_moves, 0);
    // the closing threshold near open before a calibration
    let p = GripperPolicy::rebot_dm();
    let (t, calibrated, _) = armd::gripper_baseline::closing_contact_threshold(inner.gripper_baseline.as_ref(), -5.5, 0.3,
        p.contact_torque_nm, p.max_close_torque_nm).unwrap();
    assert!(!calibrated && (t - 0.85).abs() < 1e-4, "{t}");
}

/// The default deadline covers a full stroke at the 0.3 rad/s close speed with margin, bounded.
#[test]
fn default_deadline_covers_a_slow_full_stroke() {
    let p = GripperPolicy::rebot_dm();
    let lim = ArmConfig::rebot_dm().limits[6].clone();
    let m = armd::motion::GripperMotion::new(GripperRequest { command: GripperCommand::Close, fraction: None, grasp: true, speed: 0.3, timeout_s: None, tag: None },
        p.clone(), lim.clone(), p.open_rad, 6);
    let travel = f64::from((p.open_rad - p.closed_rad).abs()) / 0.3;
    eprintln!("full stroke at 0.3 rad/s: travel {travel:.1} s, deadline {:.1} s, bound {:.1} s", m.timeout_s, m.hard_timeout_s());
    assert!(m.timeout_s >= 1.2 * travel && m.timeout_s <= armd::motion::GRIPPER_MAX_DEFAULT_TIMEOUT_S);
    assert!(m.hard_timeout_s() <= armd::motion::GRIPPER_MAX_DEFAULT_TIMEOUT_S);
    let slowest = armd::motion::GripperMotion::new(GripperRequest { command: GripperCommand::Close, fraction: None, grasp: true, speed: 0.05, timeout_s: None, tag: None },
        p.clone(), lim, p.open_rad, 6);
    assert_eq!(slowest.timeout_s, armd::motion::GRIPPER_MAX_DEFAULT_TIMEOUT_S);
}

/// I16 Item 1: A moving jaw with a torque bump below J7's moving torque floor (0.82 N·m, or
/// 0.9 * ceiling_estimate) — or a brief <3-tick spike above it — must NOT trigger contact or
/// stall on ANY gripper path (direct action, macro grasp/place/sequence, calibration stroke,
/// or servo stream, including slow closing speeds |v| < 0.10 rad/s), whether calibrated or
/// uncalibrated. Once net progress stops over the window, contact triggers cleanly on both
/// grasp and stream paths.
#[test]
fn moving_jaw_torque_bump_does_not_trigger_contact_on_any_path() {
    use armd::motion::{GripperContactDetector, GripperContactInput, GRIPPER_MOVING_CONTACT_TORQUE_FLOOR_NM};

    // 1. Direct gripper action (grasp: true and grasp: false), both calibrated (baseline 0.40 ->
    //    stopped threshold 0.55 N·m) and uncalibrated, with sustained 0.75 N·m drag bump while moving.
    for learned in [false, true] {
        for grasp in [true, false] {
            let (mut inner, rig) = rig(Finger::Old);
            if learned { learn(&mut inner, &rig, 2); }
            rig.lock().unwrap().load = Load::Drag(-4.5, -1.5, 0.35); // 0.40 + 0.35 = 0.75 N·m while moving
            let epoch = inner.safety.epoch_id();
            let req_cmd = GripperRequest { command: GripperCommand::Close, fraction: None, grasp, speed: 0.30, timeout_s: None, tag: None };
            let (id, _) = inner.start_gripper(req_cmd, epoch).unwrap();
            let a = loop {
                inner.tick(DT);
                if inner.measured_positions[6] < -0.35 {
                    assert!(!inner.gripper_stalled, "action learned={learned} grasp={grasp}: false stall at {}", inner.measured_positions[6]);
                    assert!(inner.gripper_last_contact.is_none(), "action learned={learned} grasp={grasp}: false contact {:?}", inner.gripper_last_contact);
                }
                let a = inner.last_action.clone().unwrap();
                assert_eq!(a.id, id);
                if !a.is_running() { break a; }
            };
            assert_eq!(a.state, "done", "action learned={learned} grasp={grasp}: {a:?}");
            if grasp { assert_eq!(a.holding, Some(false), "action learned={learned}: {a:?}"); }
            assert!(inner.measured_positions[6] > -0.10, "action learned={learned} grasp={grasp}: ended at {}", inner.measured_positions[6]);
        }
    }

    // 2. Macro paths (grasp / place / sequence share `start_macro_gripper_within`, calibration
    //    uses `start_macro_gripper_learning_within`), calibrated and uncalibrated, with 0.75 N·m bump.
    for learned in [false, true] {
        for (label, grasp, calib) in [("macro_grasp", true, false), ("macro_place_seq", false, false), ("calibration", false, true)] {
            let (mut inner, rig) = rig(Finger::Old);
            if learned { learn(&mut inner, &rig, 2); }
            rig.lock().unwrap().load = Load::Drag(-4.5, -1.5, 0.35); // 0.75 N·m while moving
            let epoch = inner.safety.epoch_id();
            let macro_id = inner.begin_orchestrated_action(if calib { "gripper_calibrate" } else { "grasp" }, None, epoch).unwrap();
            let request = GripperRequest {
                command: if calib { GripperCommand::Move } else { GripperCommand::Close },
                fraction: calib.then_some(0.05),
                grasp,
                speed: 0.30,
                timeout_s: None,
                tag: None,
            };
            if calib {
                inner.start_macro_gripper_learning_within(macro_id, request, armd::motion::LearnMode::Calibration, None, false).unwrap();
            } else {
                inner.start_macro_gripper_within(macro_id, request, None, false).unwrap();
            }
            let target_rad = if calib {
                let p = GripperPolicy::rebot_dm();
                p.closed_rad + 0.05 * (p.open_rad - p.closed_rad)
            } else {
                0.0
            };
            let mut finished = false;
            for _ in 0..(30.0 / DT) as usize {
                inner.tick(DT);
                if inner.measured_positions[6] < target_rad - 0.35 {
                    assert!(!inner.gripper_stalled, "{label} learned={learned}: false stall at {}", inner.measured_positions[6]);
                    assert!(inner.gripper_last_contact.is_none(), "{label} learned={learned}: false contact {:?}", inner.gripper_last_contact);
                }
                let sub = inner.macro_run.as_ref().unwrap().sub.clone();
                match sub {
                    armd::macro_action::SubState::Done(d) => {
                        if grasp { assert_eq!(d.holding, Some(false), "{label} learned={learned}: {d:?}"); }
                        finished = true;
                        break;
                    }
                    armd::macro_action::SubState::Failed { code, message } => {
                        panic!("{label} learned={learned} failed: {code}: {message}");
                    }
                    _ => {}
                }
            }
            assert!(finished, "{label} learned={learned}: did not finish");
            assert!((inner.measured_positions[6] - target_rad).abs() < 0.10, "{label} learned={learned}: ended at {}", inner.measured_positions[6]);
        }
    }

    // 3. Stream path (servo stream):
    //    3a. Moving jaw at 0.30 rad/s and slow 0.06 rad/s (|v| < 0.10 rad/s!) with sustained
    //        0.75 N·m bump (above calibrated 0.55, contact_torque_nm 0.60, and stall_nm 0.444,
    //        below J7's 0.82 N·m moving floor): regulator must NOT engage and stall must NOT latch.
    for learned in [false, true] {
        for speed in [0.30_f32, 0.06_f32] {
            let (mut inner, rig) = rig(Finger::Old);
            if learned { learn(&mut inner, &rig, 2); }
            rig.lock().unwrap().load = Load::Drag(-5.65, -0.5, 0.35); // 0.75 N·m across travel (including slow 0.06 rad/s ramp from -5.7)
            let epoch = inner.safety.epoch_id();
            let mut target = INSPECT.to_vec();
            // Run 2.5 s of closing ramp (~0.75 rad at 0.30 rad/s or ~0.15 rad at 0.06 rad/s),
            // far longer than the 0.10-0.14 s progress window.
            let mut g = INSPECT[6];
            let mut saw_high_tau = false;
            for _ in 0..125 {
                g = (g + speed * DT as f32).min(0.0);
                target[6] = g;
                inner.servo(target.clone(), epoch).unwrap();
                inner.tick(DT);
                let r = rig.lock().unwrap();
                if r.tau[6].abs() >= 0.70 { saw_high_tau = true; }
                assert!(
                    inner.gripper_stream_contact.is_none(),
                    "stream learned={learned} speed={speed}: false stream contact {:?} while moving (q={:.3}, v={:.3}, tau={:.3})",
                    inner.gripper_stream_contact, r.q[6], r.v[6], r.tau[6]
                );
                assert!(
                    !inner.gripper_stalled,
                    "stream learned={learned} speed={speed}: false stall while moving (q={:.3}, v={:.3}, tau={:.3})",
                    r.q[6], r.v[6], r.tau[6]
                );
            }
            assert!(saw_high_tau, "stream learned={learned} speed={speed}: test never exercised >= 0.70 N·m drag");
            assert!(
                rig.lock().unwrap().q[6] > INSPECT[6] + 0.05,
                "stream learned={learned} speed={speed}: jaw did not advance"
            );
        }
    }

    //    3b. Moving jaw under stream with a 2-tick 0.90 N·m torque spike (>= 0.82 N·m moving floor,
    //        < 3 sustained ticks): must NOT trigger single-sample stream contact.
    for learned in [false, true] {
        let (mut inner, rig) = rig(Finger::Old);
        if learned { learn(&mut inner, &rig, 2); }
        let epoch = inner.safety.epoch_id();
        let mut target = INSPECT.to_vec();
        let mut g = INSPECT[6];
        let mut saw_spike_tau = false;
        for tick in 0..80 {
            g = (g + 0.30 * DT as f32).min(0.0);
            target[6] = g;
            // Inject a 2-tick 0.90 N·m torque bump mid-travel (ticks 30 and 31).
            rig.lock().unwrap().load = if tick == 30 || tick == 31 {
                Load::Drag(-6.0, 0.0, 0.50) // 0.40 + 0.50 = 0.90 N·m
            } else {
                Load::Free
            };
            inner.servo(target.clone(), epoch).unwrap();
            inner.tick(DT);
            if rig.lock().unwrap().tau[6].abs() >= 0.85 {
                saw_spike_tau = true;
            }
            assert!(
                inner.gripper_stream_contact.is_none(),
                "stream 2-tick spike learned={learned} tick={tick}: false contact {:?}",
                inner.gripper_stream_contact
            );
            assert!(!inner.gripper_stalled, "stream 2-tick spike learned={learned} tick={tick}: false stall");
        }
        assert!(saw_spike_tau, "stream 2-tick spike learned={learned}: never observed >= 0.85 N·m spike");
    }

    //    3c. Stopped jaw on a rigid block under a closing stream: once net progress stops over the
    //        window, the stream regulator DOES engage cleanly and prevents stall latch.
    for learned in [false, true] {
        let (mut inner, rig) = rig(Finger::Old);
        if learned { learn(&mut inner, &rig, 2); }
        rig.lock().unwrap().load = Load::CloseBlock(-5.3, 3.0);
        let epoch = inner.safety.epoch_id();
        let mut target = INSPECT.to_vec();
        let mut g = INSPECT[6];
        for _ in 0..150 {
            g = (g + 0.30 * DT as f32).min(0.0);
            target[6] = g;
            inner.servo(target.clone(), epoch).unwrap();
            inner.tick(DT);
        }
        assert!(inner.gripper_stream_contact.is_some(), "stream stopped learned={learned}: regulator did not engage");
        assert!(!inner.gripper_stalled, "stream stopped learned={learned}: regulator failed to preempt stall");
    }

    // 4. Shared detector unit check: moving floor scales with 0.9 * ceiling_torque_estimate_nm
    //    even with a calibrated baseline (baseline 0.40 -> stopped thresh 0.55 N·m, ceiling 1.0 ->
    //    moving floor 0.90 N·m). Moving at 0.86 N·m for 10 ticks does NOT trigger; once stopped,
    //    0.56 N·m triggers after the net-progress window.
    let (mut inner, rig) = rig(Finger::Old);
    learn(&mut inner, &rig, 2);
    let baseline = inner.gripper_baseline.as_ref().unwrap();
    let mut det = GripperContactDetector::default();
    let mut q = -4.0_f32;
    for _ in 0..10 {
        q += 0.006; // moving at 0.30 rad/s
        let ev = det.step(&GripperContactInput {
            baseline: Some(baseline),
            contact_torque_nm: Some(0.60),
            max_close_torque_nm: Some(1.0),
            ceiling_torque_estimate_nm: Some(1.0), // 0.9 * 1.0 = 0.90 > 0.82
            stopped_stall_nm: None,
            measured_pos: q,
            dir: 1.0,
            baseline_speed: 0.30,
            window_speed: Some(0.30),
            tau_abs: 0.86,
            dt: DT,
            torque_feedback: true,
            active: true,
        });
        assert!(!ev.triggered && !ev.progress_stopped, "moving 0.86 N·m below 0.9*ceil=0.90 triggered: {ev:?}");
        assert!(ev.threshold_nm.unwrap() >= 0.90_f32.max(GRIPPER_MOVING_CONTACT_TORQUE_FLOOR_NM) - 1e-5);
    }
    // Now jaw stops at q with 0.56 N·m (>= calibrated 0.55 N·m, < 0.60 contact_torque_nm).
    let mut stopped_hit = false;
    for _ in 0..8 {
        let ev = det.step(&GripperContactInput {
            baseline: Some(baseline),
            contact_torque_nm: Some(0.60),
            max_close_torque_nm: Some(1.0),
            ceiling_torque_estimate_nm: Some(1.0),
            stopped_stall_nm: None,
            measured_pos: q,
            dir: 1.0,
            baseline_speed: 0.30,
            window_speed: Some(0.30),
            tau_abs: 0.56,
            dt: DT,
            torque_feedback: true,
            active: true,
        });
        if ev.triggered {
            assert!(ev.progress_stopped);
            assert_eq!(ev.threshold_source, ThresholdSource::Baseline);
            stopped_hit = true;
            break;
        }
    }
    assert!(stopped_hit, "stopped jaw at 0.56 N·m (>= calibrated 0.55 N·m) did not trigger");
}

/// I16 Item 4: Automatic baseline learning must consume queued `MoveRecord`s off the control
/// thread under BOTH `RtOwner::Executor` and `RtOwner::Legacy`, without allocating on
/// `armd-control`, enforcing tool/follower isolation, updating the owner's live baseline
/// observation, and persisting to disk.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_owner_consumes_queued_baseline_records_off_control_thread() {
    use armd::gripper_baseline::{FreeTorqueBaseline, MoveRecord, MoveSamples};
    use armd::rt_core::executor::ExecutorConfig;
    use armd::ArmSupervisor;

    // Positive control proving `TimedAlloc` is active as #[global_allocator] in this test binary
    // so `lock_alloc_calls == 0` below cannot be vacuously zero.
    std::thread::spawn(|| {
        armd::diagnostics::alloc::time_this_thread();
        let _ = armd::diagnostics::alloc::take();
        let probe: Vec<u8> = vec![0xAB; 64];
        std::hint::black_box(&probe);
        drop(probe);
        let stats = armd::diagnostics::alloc::take();
        assert!(stats.calls > 0, "TimedAlloc positive control failed: calls={}", stats.calls);
    })
    .join()
    .unwrap();

    let make_open_record = |tau_nm: f32| -> MoveRecord {
        let mut samples = MoveSamples::default();
        for i in 0..=200 {
            let q = 0.0 + (-5.7) * (i as f32 / 200.0);
            samples.push(q, tau_nm, 1.0);
        }
        MoveRecord {
            direction: Direction::Open,
            start_rad: 0.0,
            end_rad: -5.7,
            samples,
            calibration: false,
        }
    };

    // 1. Executor owner: a live unloaded full opening move (0.0 -> -5.7) finishes on `armd-control`
    //    with `RigDriver` (Finger::Old = 0.40 N·m, torque_feedback = true) and queues a real
    //    `MoveRecord` into `core.gripper_baseline_pending`. A foreign-tool baseline (`"other-tool"`,
    //    0.99 N·m, 7 open moves) is seeded on the owner right before executor start so
    //    `TakeBaselinePending` MUST filter it out via `belongs_to` and `StoreBaseline` MUST return
    //    it off-thread for zero-free reclamation on `armd-control`.
    let dir = tempfile::tempdir().unwrap();
    let baseline_path = dir.path().join("gripper_baseline.rig.json");

    let mut q_closed = INSPECT;
    q_closed[6] = 0.0;
    let live_rig = Arc::new(Mutex::new(Rig {
        q: q_closed,
        v: [0.0; 7],
        tau: [0.0; 7],
        enabled: EnableState::Disabled,
        finger: Finger::Old,
        load: Load::Free,
        seed: 7,
        offset: 0.0,
        broke: false,
        peak_tau: 0.0,
        pressed: 0,
    }));
    let sup = Arc::new(
        ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(RigDriver(live_rig.clone()))),
    );
    {
        let mut inner = sup.inner().write().await;
        inner.lease_enabled = false;
        inner.follower = Some("rig".into());
        inner.gripper_baseline_path = Some(baseline_path.clone());
        inner.tick(DT);
        inner.set_torque(true).unwrap();
        let ep = inner.safety.epoch_id();
        inner
            .start_gripper(
                GripperRequest {
                    command: GripperCommand::Open,
                    fraction: None,
                    grasp: false,
                    speed: 3.0,
                    timeout_s: Some(10.0),
                    tag: None,
                },
                ep,
            )
            .unwrap();
        // Seed the foreign-tool baseline AFTER `start_gripper` (which calls
        // `gripper_baseline_for_motion`) so it is genuinely present in `core.gripper_baseline`
        // when `TakeBaselinePending` and `StoreBaseline` execute on `armd-control`.
        let mut foreign = FreeTorqueBaseline::new(
            0.0,
            -5.7,
            Some("other-tool".into()),
            Some("deadbeef".into()),
            Some("rig".into()),
        );
        foreign.learn(&make_open_record(0.99), 2.0);
        foreign.open_moves = 7;
        inner.gripper_baseline = Some(foreign);
    }

    let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(5));
    cfg.alloc_timing = true;
    let handle = sup.start_executor(cfg).await.unwrap();

    // Let the live full-stroke opening motion finish on `armd-control` first: the control thread
    // only queues the MoveRecord (never learns synchronously on `armd-control`), so `open_moves`
    // stays 0 until the off-thread consumer starts.
    let motion_deadline = std::time::Instant::now() + Duration::from_secs(6);
    while std::time::Instant::now() < motion_deadline {
        let obs = sup.get_observation().await;
        if obs.action.as_ref().is_some_and(|a| !a.is_running()) {
            break;
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    let pre_obs = sup.get_observation().await;
    assert!(pre_obs.action.as_ref().is_some_and(|a| a.state.as_str() == "done"), "{:?}", pre_obs.action);
    assert_eq!(pre_obs.gripper.as_ref().and_then(|g| g.baseline).unwrap().open_moves, 0);

    let consumer = armd::gripper_calib::spawn_baseline_persister_with_interval(
        sup.clone(),
        Duration::from_millis(20),
    );

    // Wait until the consumer drains the live queued record, updates the owner's live baseline,
    // and persists the JSON file to disk (read published StateHub observations so every
    // maintenance release in `tick_samples` is a baseline op).
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    let mut learned_obs = None;
    while std::time::Instant::now() < deadline {
        let obs = sup.get_observation().await;
        if let Some(b) = obs.gripper.and_then(|g| g.baseline) {
            if b.open_moves >= 1 && b.state.as_str() == "learning" && baseline_path.exists() {
                learned_obs = Some(b);
                break;
            }
        }
        tokio::time::sleep(Duration::from_millis(15)).await;
    }
    let obs_b = learned_obs.expect("executor baseline consumer did not drain/learn/persist queued records");
    assert_eq!(obs_b.open_moves, 1, "expected 1 live open move (not foreign tool's 7): {obs_b:?}");
    assert_eq!(obs_b.state.as_str(), "learning", "{obs_b:?}");

    // Persisted file parses cleanly, belongs to the active tool (not "other-tool"), and learned
    // the live Finger::Old opening stroke (~0.40 N·m, not the foreign 0.99 N·m table).
    let raw = std::fs::read_to_string(&baseline_path).unwrap();
    let persisted: FreeTorqueBaseline = serde_json::from_str(&raw).unwrap();
    persisted.validate().unwrap();
    assert_eq!(persisted.open_moves, 1, "{persisted:?}");
    assert_ne!(persisted.tool_id.as_deref(), Some("other-tool"));
    let est = persisted.estimate(-3.0, Direction::Open, 1.0).expect("open bin learned from live stroke");
    assert!((est.baseline_nm - 0.40).abs() < 0.08, "expected ~0.40 N·m live open baseline (not foreign 0.99): {est:?}");

    // Queue must now be empty on the executor owner, and a stale-generation StoreBaseline must
    // be refused without mutating live state.
    let taken_after = handle
        .take_baseline_pending_async(Vec::with_capacity(armd::supervisor::GRIPPER_BASELINE_PENDING_MAX), false)
        .await
        .unwrap();
    assert!(taken_after.moves.is_empty(), "queued records were not drained: {}", taken_after.moves.len());
    assert!(taken_after.generation >= 1, "expected generation bump after store");
    let stale_stored = handle
        .store_baseline_async(persisted.clone(), taken_after.generation - 1, false)
        .await
        .unwrap();
    assert!(!stale_stored, "stale generation StoreBaseline must be refused");

    // Verify EVERY maintenance release servicing TakeBaselinePending / StoreBaseline did 0
    // allocations/frees on `armd-control` (measured by active `TimedAlloc` global allocator).
    tokio::time::sleep(Duration::from_millis(20)).await;
    let samples = sup.tick_samples();
    let maint_allocs: Vec<(u64, u32)> = samples
        .iter()
        .filter(|s| s.maint_req_id != 0)
        .map(|s| (s.maint_req_id, s.usage.lock_alloc_calls))
        .collect();
    assert!(maint_allocs.len() >= 2, "expected >= 2 baseline maintenance releases, got {maint_allocs:?}");
    assert!(
        maint_allocs.iter().all(|&(_, calls)| calls == 0),
        "baseline maintenance op allocated on armd-control: {maint_allocs:?}"
    );

    // Stopping the executor causes the spawned consumer task to exit cleanly (no hang/panic).
    handle.stop_and_join();
    tokio::time::timeout(Duration::from_secs(2), consumer)
        .await
        .expect("baseline consumer did not exit after executor stop")
        .unwrap();

    // 2. Legacy owner parity: the same consumer helper drains & persists queued records in Legacy mode.
    let dir_leg = tempfile::tempdir().unwrap();
    let leg_path = dir_leg.path().join("gripper_baseline.legacy.json");
    let sup_leg = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    {
        let mut inner = sup_leg.inner().write().await;
        inner.follower = Some("legacy".into());
        inner.gripper_baseline_path = Some(leg_path.clone());
        inner.gripper_baseline_pending.push(make_open_record(0.32));
    }
    assert!(armd::gripper_calib::step_baseline_persister_once(&sup_leg).await);
    assert!(leg_path.exists(), "legacy baseline file not written");
    let leg_obs = sup_leg.inner().read().await.gripper_baseline_observation();
    assert_eq!(leg_obs.open_moves, 1, "{leg_obs:?}");
    assert!(sup_leg.inner().read().await.gripper_baseline_pending.is_empty());
}
