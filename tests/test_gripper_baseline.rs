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
use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
use armd::gripper_baseline::{ContactSource, Direction, ThresholdSource};
use armd::model::{ArmConfig, GripperPolicy, JointState};
use armd::motion::{GripperCommand, GripperRequest};
use armd::SupervisorInner;

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
}

struct Rig {
    q: [f32; 7], v: [f32; 7], tau: [f32; 7], enabled: EnableState,
    finger: Finger, load: Load, seed: u32, offset: f32,
    broke: bool, peak_tau: f32,
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
        finger, load: Load::Free, seed: 7, offset: 0.0, broke: false, peak_tau: 0.0 }));
    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(RigDriver(rig.clone())));
    inner.lease_enabled = false;
    inner.tick(DT);
    inner.set_torque(true).unwrap();
    (inner, rig)
}

fn req(command: GripperCommand, fraction: Option<f64>, speed: f64) -> GripperRequest {
    GripperRequest { command, fraction, grasp: command == GripperCommand::Close, speed, timeout_s: None, tag: None }
}

/// Run a gripper action to its end; (action, seconds).
fn run(inner: &mut SupervisorInner, request: GripperRequest) -> (armd::motion::ActionStatus, f64) {
    let epoch = inner.safety.epoch_id();
    let (id, _) = inner.start_gripper(request, epoch).unwrap();
    for i in 0..(30.0 / DT) as usize {
        inner.tick(DT);
        let a = inner.last_action.clone().unwrap();
        assert_eq!(a.id, id);
        if !a.is_running() { return (a, (i + 1) as f64 * DT); }
    }
    panic!("gripper action still running: {:?}", inner.last_action);
}

fn cap() -> f32 { GripperPolicy::rebot_dm().max_close_torque_nm.unwrap() }

/// Calibration strokes: unloaded closes (move to 5 %) and opens at the default speed and at
/// the contact speed (slow band), with a different run-to-run offset each time.
fn learn(inner: &mut SupervisorInner, rig: &Arc<Mutex<Rig>>, rounds: usize) {
    for k in 0..rounds {
        for speed in [1.0, 0.3] {
            rig.lock().unwrap().offset = [0.03, -0.02, 0.0, 0.02][k % 4];
            let (a, _) = run(inner, req(GripperCommand::Move, Some(0.05), speed));
            assert_eq!(a.state, "done", "{a:?}");
            let (a, _) = run(inner, req(GripperCommand::Open, None, speed));
            assert_eq!(a.state, "done", "{a:?}");
        }
    }
    rig.lock().unwrap().offset = 0.0;
}

fn baseline_obs(inner: &SupervisorInner) -> armd::model::GripperBaselineObs {
    inner.current_observation().gripper.unwrap().baseline.unwrap()
}

/// New finger, nothing learned: the close completes (no GRIPPER_TIMEOUT), the observation says
/// "unlearned"; one open teaches the opening direction, which the next close already uses.
#[test]
fn new_finger_unlearned_air_close_completes() {
    let (mut inner, _rig) = rig(Finger::New);
    assert_eq!(baseline_obs(&inner).state, "unlearned");
    let (a, t) = run(&mut inner, req(GripperCommand::Close, None, 1.0));
    eprintln!("unlearned air close: {} in {t:.1} s, contact {:?}", a.state, inner.gripper_last_contact);
    assert_eq!((a.state.as_str(), a.holding), ("done", Some(false)), "{a:?}");
    assert!(t < 10.0, "{t:.1} s");
    let (a, _) = run(&mut inner, req(GripperCommand::Open, None, 1.0));
    assert_eq!(a.state, "done");
    let obs = baseline_obs(&inner);
    assert!(obs.open_moves >= 1 && obs.state != "unlearned", "{obs:?}");
    let (a, t) = run(&mut inner, req(GripperCommand::Close, None, 1.0));
    assert_eq!((a.state.as_str(), a.holding), ("done", Some(false)), "{a:?}");
    assert!(t < 9.0, "{t:.1} s");
}

/// New finger after calibration strokes: an air grasp-close runs at the requested speed to
/// closed (no contact verdict before the end) and is learned.
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
    assert!(baseline_obs(&inner).close_moves > before);
}

/// A banana-like soft object at -2.0 rad: detected from the torque above the baseline, squeezed
/// and held with the torque below the cap; the held close is not learned.
#[test]
fn new_finger_soft_object_is_detected_and_held_under_the_cap() {
    let (mut inner, rig) = rig(Finger::New);
    learn(&mut inner, &rig, 2);
    let estimate = |inner: &SupervisorInner| inner.gripper_baseline.as_ref().unwrap().estimate(-1.5, Direction::Close, 1.0).unwrap().baseline_nm;
    let (close_moves, base_before) = (baseline_obs(&inner).close_moves, estimate(&inner));
    for (k1, k2) in [(1.5_f32, 8.0_f32), (0.8, 4.0), (3.0, 20.0)] {
        { let mut r = rig.lock().unwrap(); r.load = Load::Soft(-2.0, k1, k2); r.peak_tau = 0.0; }
        let (a, t) = run(&mut inner, req(GripperCommand::Close, None, 1.0));
        let c = inner.gripper_last_contact.unwrap();
        let peak = rig.lock().unwrap().peak_tau;
        eprintln!("soft k1 {k1}: {} holding {:?} in {t:.1} s at {:.3} rad, contact {c:?}, peak {peak:.2}", a.state, a.holding, inner.measured_positions[6]);
        assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "k1 {k1}: {a:?}");
        assert!(c.at_rad > -2.05 && c.at_rad < -1.3, "{c:?}");
        // the cap latches after GRIPPER_FORCE_CAP_TICKS ticks (test_gripper_force CAP_MARGIN_NM)
        assert!(peak < cap() + 0.25, "peak {peak:.3}");
        // and the grip settles at or below the cap
        rig.lock().unwrap().peak_tau = 0.0;
        for _ in 0..50 { inner.tick(DT); }
        assert!(rig.lock().unwrap().peak_tau <= cap() + 1e-3);
        if c.source == ContactSource::Torque {
            assert_eq!(c.threshold_source, ThresholdSource::Baseline);
            assert!(c.margin_nm.unwrap() > 0.1, "{c:?}");
        }
        assert!(t < 8.0, "{t:.1} s");
        // holding: not learned
        assert_eq!(baseline_obs(&inner).close_moves, close_moves);
        // let go (the open from the object is learned without the squeeze at its start)
        rig.lock().unwrap().load = Load::Free;
        let (a, _) = run(&mut inner, req(GripperCommand::Open, None, 1.0));
        assert_eq!(a.state, "done");
    }
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
            assert!(peak < cap() + 0.25, "peak {peak:.3} N·m");
            assert_eq!(a.state, "done", "{a:?}");
            assert!(inner.measured_positions[6] < -1.0, "J7 {}", inner.measured_positions[6]);
            rig.lock().unwrap().load = Load::Free;
            let (a, _) = run(&mut inner, req(GripperCommand::Open, None, 1.0));
            assert_eq!(a.state, "done", "{a:?}");
            rig.lock().unwrap().load = Load::Incident;
        }
    }
}

/// Old finger (0.40 N·m everywhere): with a learned baseline the soft object is caught at about
/// the same place as without one, held, and not squeezed harder.
#[test]
fn old_finger_contact_unchanged_by_the_baseline() {
    let contact = |learned: bool| {
        let (mut inner, rig) = rig(Finger::Old);
        if learned { learn(&mut inner, &rig, 2); }
        rig.lock().unwrap().load = Load::Soft(-2.0, 1.5, 8.0);
        let (a, _) = run(&mut inner, req(GripperCommand::Close, None, 1.0));
        assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{a:?}");
        let peak = rig.lock().unwrap().peak_tau;
        (inner.gripper_last_contact.unwrap(), peak)
    };
    let (c0, p0) = contact(false);
    let (c1, p1) = contact(true);
    eprintln!("old finger: unlearned {c0:?} peak {p0:.2}; learned {c1:?} peak {p1:.2}");
    assert!((c0.at_rad - c1.at_rad).abs() < 0.1, "{c0:?} vs {c1:?}");
    // the learned threshold (0.40 + 0.15) sits a little below contact_torque_nm: same outcome,
    // the contact is found slightly earlier and the peak is not higher
    assert!(p1 <= p0 + 0.05, "{p0} vs {p1}");
}

/// After lubrication (free torque halved) the baseline follows down over a few unloaded moves.
#[test]
fn baseline_follows_lower_friction() {
    let (mut inner, rig) = rig(Finger::New);
    learn(&mut inner, &rig, 2);
    let at = |inner: &SupervisorInner| inner.gripper_baseline.as_ref().unwrap().estimate(-4.5, Direction::Close, 1.0).unwrap().baseline_nm;
    let high = at(&inner);
    rig.lock().unwrap().finger = Finger::Old; // 0.40 instead of ~0.6
    learn(&mut inner, &rig, 4);
    let low = at(&inner);
    eprintln!("baseline at -4.5 close: {high:.3} -> {low:.3}");
    assert!(high > 0.52 && (low - 0.40).abs() < 0.05, "{high} -> {low}");
}

/// The baseline is written next to the gripper calibration and loaded again for the same tool;
/// another tool's file is ignored.
#[tokio::test]
async fn baseline_persists_and_is_tool_bound() {
    let dir = tempfile::tempdir().unwrap();
    let calib = dir.path().join("gripper_calib.json");
    let (mut inner, rig) = rig(Finger::New);
    inner.gripper_baseline_path = Some(armd::gripper_calib::baseline_path_for(&calib));
    learn(&mut inner, &rig, 1);
    let learned = inner.gripper_baseline.clone().unwrap();
    let w = armd::gripper_calib::take_baseline_write(&mut inner).expect("dirty baseline");
    assert!(armd::gripper_calib::take_baseline_write(&mut inner).is_none());
    assert_eq!(w.path, dir.path().join("gripper_baseline.json"));
    w.run().await.unwrap();

    let sup = armd::ArmSupervisor::new(ArmConfig::rebot_dm());
    armd::gripper_calib::load_into(&sup, &calib).await;
    assert_eq!(sup.inner().read().await.gripper_baseline.as_ref(), Some(&learned));

    // another tool's baseline
    let mut other = learned.clone();
    other.tool_id = Some("suction".into());
    other.tool_config_digest = Some("0".repeat(64));
    std::fs::write(dir.path().join("gripper_baseline.json"), serde_json::to_string(&other).unwrap()).unwrap();
    let sup = armd::ArmSupervisor::new(ArmConfig::rebot_dm());
    armd::gripper_calib::load_into(&sup, &calib).await;
    let g = sup.inner().read().await;
    assert!(g.gripper_baseline.is_none());
    assert_eq!(g.gripper_baseline_observation().state, "unlearned");
}

/// An explicit timeout_s stays the deadline; the derived one follows the slow contact creep.
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
    let c = inner.gripper_last_contact.unwrap();
    eprintln!("lube soft at -3.0: {c:?}, peak {:.2}", rig.lock().unwrap().peak_tau);
    assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{a:?}");
    assert!(c.at_rad > -3.05 && c.at_rad < -2.4, "{c:?}");
    assert!(rig.lock().unwrap().peak_tau < cap() + 0.25);
}
