//! Torque-saturation guard (docs/control_safety_contract.md "Torque guard") and the torque
//! state after a bus loss, on a simulated arm.
//!
//! `StopArm` is an ideal position servo that reports the model gravity torque plus, for a
//! joint pushed past a simulated mechanical stop, `STIFFNESS` x the overshoot, saturated at
//! the joint's TMAX (like a motor stalled against a stop). A test can also force a joint's
//! reported torque, make the feedback stale (bus loss) and change the reported enable state
//! (motor power reset). The numbers printed here are simulation results, not hardware
//! measurements.
mod common;

use std::sync::{Arc, Mutex};
use std::time::Duration;

use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointMode, JointTarget};
use armd::model::{ArmConfig, JointState, MotionLimitReason};
use armd::recording::{write_frames_to_parquet, RecordFrame};
use armd::replay::ReplayRequest;
use armd::torque_guard::{TorqueGuardInfo, TorqueGuardPolicy, TORQUE_HOLD_RELEASE_RAD};
use armd::{ArmObservation, FaultCode, SafetyState, SupervisorInner};
use common::INSPECT;

const DT: f64 = 0.02;
/// N·m per rad of command past the stop.
const STIFFNESS: f32 = 100.0;
const TMAX: [f32; 7] = [28.0, 28.0, 28.0, 10.0, 10.0, 10.0, 10.0];
const J4: usize = 3;
/// B601-DM thresholds: 0.60 x TMAX (28 / 10 N·m).
const THR_BIG: f32 = 16.8;
const THR_WRIST: f32 = 6.0;

#[derive(Default)]
struct ArmState {
    q: [f32; 7],
    /// Last commanded position.
    cmd: [f32; 7],
    torqued: bool,
    mit: bool,
    /// Upper stop per joint (the joint cannot pass it).
    stop_hi: [Option<f32>; 7],
    /// Reported torque override per joint.
    forced: [Option<f32>; 7],
    /// Feedback stale (bus loss): ages 600 ms, invalid.
    stale: bool,
}

#[derive(Clone)]
struct StopArm(Arc<Mutex<ArmState>>);

impl StopArm {
    fn new(q: [f32; 7]) -> Self { Self(Arc::new(Mutex::new(ArmState { q, cmd: q, ..Default::default() }))) }
    fn with<R>(&self, f: impl FnOnce(&mut ArmState) -> R) -> R { f(&mut self.0.lock().unwrap()) }
}

impl ArmDriver for StopArm {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true, torque_feedforward: false, enable_feedback: true }
    }
    fn enable_state(&self) -> EnableState { if self.0.lock().unwrap().torqued { EnableState::Enabled } else { EnableState::Disabled } }
    fn is_feedback_valid(&self) -> bool { !self.0.lock().unwrap().stale }
    fn is_enabled(&self) -> bool { self.0.lock().unwrap().torqued }
    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        let age = if self.0.lock().unwrap().stale { Duration::from_millis(600) } else { Duration::ZERO };
        vec![Some(age); 7]
    }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let s = self.0.lock().unwrap();
        let mut q6 = [0.0; 6];
        q6.copy_from_slice(&s.q[..6]);
        let g = armd::compute_gravity_torques(&q6).unwrap();
        Ok((0..7).map(|i| {
            let push = if s.torqued && !s.mit { s.stop_hi[i].map_or(0.0, |hi| STIFFNESS * (s.cmd[i] - hi).max(0.0)) } else { 0.0 };
            let model = if i < 6 { g[i] } else { 0.0 } + push;
            let torque = s.forced[i].unwrap_or(model.clamp(-TMAX[i], TMAX[i]));
            JointState { position: s.q[i], velocity: 0.0, temperature: 30.0, torque }
        }).collect())
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        let mut s = self.0.lock().unwrap();
        if s.mit || s.stale || !s.torqued { return Ok(()); }
        for (i, t) in targets.iter().enumerate().take(7) {
            s.cmd[i] = t.position;
            s.q[i] = s.stop_hi[i].map_or(t.position, |hi| t.position.min(hi));
        }
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

fn profile() -> ArmConfig { ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() } }

fn rig_at(config: ArmConfig, q: [f32; 7]) -> (SupervisorInner, StopArm) {
    let arm = StopArm::new(q);
    let mut inner = SupervisorInner::new(config);
    inner.lease_enabled = false;
    inner.driver = Some(Mutex::new(Box::new(arm.clone())));
    inner.commanded_positions = q.to_vec();
    inner.measured_positions = q.to_vec();
    inner.target_positions = q.to_vec();
    inner.set_torque(true).unwrap();
    ticks(&mut inner, 5);
    assert!(inner.safety.state().is_operational(), "{:?}", inner.safety.state());
    (inner, arm)
}

fn rig() -> (SupervisorInner, StopArm) { rig_at(profile(), INSPECT) }

fn ticks(inner: &mut SupervisorInner, n: usize) -> ArmObservation {
    let mut obs = None;
    for _ in 0..n { obs = Some(inner.tick(DT)); }
    obs.unwrap()
}

fn torque_limit(obs: &ArmObservation) -> Option<usize> {
    obs.motion_limit.as_ref().filter(|l| l.reason == MotionLimitReason::Torque).and_then(|l| l.joint)
}

fn servo_j4(inner: &mut SupervisorInner, q4: f32) {
    let mut t = inner.commanded_positions.clone();
    t[J4] = q4;
    let epoch = inner.safety.epoch_id();
    inner.servo(t, epoch).unwrap();
}

/// Stream J4 upward into a stop at `stop`; returns (tick of the first |τ4| >= THR_WRIST, tick of
/// the hold), counted in ticks from the start of the stream.
fn push_j4_into_stop(inner: &mut SupervisorInner, arm: &StopArm, stop: f32) -> (usize, usize) {
    arm.with(|s| s.stop_hi[J4] = Some(stop));
    let mut first_over = None;
    let mut target = inner.commanded_positions[J4];
    for k in 0..200 {
        target = (target + 0.03).min(stop + 0.3);
        servo_j4(inner, target);
        let obs = inner.tick(DT);
        if first_over.is_none() && obs.torques[J4].abs() >= THR_WRIST { first_over = Some(k); }
        if torque_limit(&obs) == Some(J4) { return (first_over.expect("torque crossed the threshold before the hold"), k); }
    }
    panic!("no torque hold within 200 ticks");
}

#[test]
fn rebot_dm_thresholds_are_60_percent_of_tmax_on_j1_to_j6() {
    let config = ArmConfig::rebot_dm();
    let p = config.torque_guard.expect("on for the B601-DM");
    assert_eq!(p, TorqueGuardPolicy { enabled: true, ratio: 0.60, persist_ms: 100, joints: [true; 6] });
    assert_eq!(TorqueGuardPolicy::default().ratio, 0.85, "section default: the value for the 14.6 A supply");
    let info = TorqueGuardInfo::from_config(&config).unwrap();
    let t: Vec<f32> = info.thresholds_nm.iter().map(|t| t.unwrap_or(-1.0)).collect();
    for (j, want) in [THR_BIG, THR_BIG, THR_BIG, THR_WRIST, THR_WRIST, THR_WRIST, -1.0].iter().enumerate() {
        assert!((t[j] - want).abs() < 1e-4, "J{} threshold {} want {want}", j + 1, t[j]);
    }
    println!("schema.torque_guard: {}", serde_json::to_string(&info).unwrap());
    // other profiles: absent section = off; enabled without damiao_ranges is refused
    assert!(ArmConfig::default().torque_guard.is_none());
    assert!(TorqueGuardInfo::from_config(&ArmConfig::default()).is_none());
    let mut bad = ArmConfig { torque_guard: Some(TorqueGuardPolicy::default()), ..ArmConfig::default() };
    assert!(bad.validate().unwrap_err().contains("damiao_ranges"));
    for p in [TorqueGuardPolicy { ratio: 0.29, ..Default::default() }, TorqueGuardPolicy { ratio: 0.96, ..Default::default() },
              TorqueGuardPolicy { ratio: f32::NAN, ..Default::default() }, TorqueGuardPolicy { persist_ms: 0, ..Default::default() }] {
        let mut c = ArmConfig { torque_guard: Some(p), ..ArmConfig::rebot_dm() };
        assert!(c.validate().unwrap_err().contains("torque_guard"), "{p:?}");
    }
    // a profile without the section parses (serde default) and keeps the guard off
    let mut v = serde_json::to_value(ArmConfig::rebot_dm()).unwrap();
    v.as_object_mut().unwrap().remove("torque_guard");
    let c: ArmConfig = serde_json::from_value(v).unwrap();
    assert!(c.torque_guard.is_none());
    // partial section: missing fields take the defaults
    let mut v = serde_json::to_value(ArmConfig::rebot_dm()).unwrap();
    v["torque_guard"] = serde_json::json!({"ratio": 0.9});
    let c: ArmConfig = serde_json::from_value(v).unwrap();
    assert_eq!(c.torque_guard.unwrap(), TorqueGuardPolicy { ratio: 0.9, ..Default::default() });
}

#[tokio::test]
async fn capability_needs_torque_feedback() {
    let mock = armd::ArmSupervisor::new(ArmConfig::rebot_dm());
    let schema = mock.get_schema().await;
    assert!(!schema.capabilities.torque_guard, "no driver: no torque feedback");
    assert!(schema.torque_guard.is_some(), "the policy is published from the profile");
    let (inner, _) = rig();
    let schema = inner.schema();
    assert!(schema.capabilities.torque_guard && schema.capabilities.version >= 8);
    let d = inner.safety_describe();
    let stop = d["stops"].as_array().unwrap().iter().find(|s| s["id"] == "torque_limit_hold").expect("describe lists the stop");
    assert_eq!(stop["action_code"], "TORQUE_LIMIT");
    assert_eq!(stop["persist_ms"], 100);
}

/// (a) servo stream into a stop: hold within persist_ms + 1 tick of the first saturated
/// sample; torque falls below the threshold once held.
#[test]
fn servo_push_into_a_stop_holds_within_persist_plus_one_tick() {
    let (mut inner, arm) = rig();
    let stop = INSPECT[J4] + 0.15;
    let (first, held) = push_j4_into_stop(&mut inner, &arm, stop);
    println!("SIM servo into stop: |τ4| >= {THR_WRIST} N·m first at tick {first}, hold at tick {held} ({} ms later)", (held - first) as f64 * DT * 1000.0);
    assert!(held - first <= 5, "hold {} ticks after the first saturated sample (persist 5 ticks + 1)", held - first);
    assert!(held - first >= 4, "not earlier than persist_ms: {} ticks", held - first);
    assert!(inner.servo_target.is_none() && inner.active_trajectory.is_none(), "stream ended");
    let obs = ticks(&mut inner, 2);
    assert!(obs.torques[J4].abs() < THR_WRIST * 0.5, "torque after the hold {}", obs.torques[J4]);
    assert!((obs.commanded_positions[J4] - stop).abs() < 1e-4, "held at the measured position {}", obs.commanded_positions[J4]);
    assert!(obs.safety_state.is_operational() && obs.is_torqued && obs.fault.is_none(), "{:?}", obs.safety_state);
    assert_eq!(torque_limit(&obs), Some(J4));
    let json = serde_json::to_value(&obs.motion_limit).unwrap();
    assert_eq!(json["reason"], "torque");
    println!("motion_limit: {json}");
    // the hold keeps motion_limit = torque while nothing happens
    let obs = ticks(&mut inner, 50);
    assert_eq!(torque_limit(&obs), Some(J4));
}

/// (b) while held: pushing further in stays held (servo and jog); backing out is allowed
/// and ends the hold after TORQUE_HOLD_RELEASE_RAD.
#[test]
fn backing_out_is_allowed_pushing_in_stays_held() {
    let (mut inner, arm) = rig();
    let stop = INSPECT[J4] + 0.15;
    push_j4_into_stop(&mut inner, &arm, stop);
    let hold_q = inner.commanded_positions[J4];
    // push in by servo: clamped to the hold position, the other joints still move
    for _ in 0..30 {
        let mut t = inner.commanded_positions.clone();
        t[J4] = hold_q + 0.1;
        t[0] += 0.005;
        let epoch = inner.safety.epoch_id();
        inner.servo(t, epoch).unwrap();
        let obs = inner.tick(DT);
        assert!(obs.commanded_positions[J4] <= hold_q + 1e-5, "J4 pushed in: {}", obs.commanded_positions[J4]);
        assert!(obs.torques[J4].abs() < THR_WRIST, "no torque build-up {}", obs.torques[J4]);
        assert_eq!(torque_limit(&obs), Some(J4));
    }
    assert!(inner.commanded_positions[0] > INSPECT[0] + 0.05, "J1 followed the stream");
    // push in by jog (cockpit joint jog): the J4 target is clamped
    let mut t = inner.commanded_positions.clone();
    t[J4] = hold_q + 0.3;
    inner.jog_action(t, None, None, "joint_move").unwrap();
    let obs = ticks(&mut inner, 60);
    assert!(obs.commanded_positions[J4] <= hold_q + 1e-5 && torque_limit(&obs) == Some(J4));
    assert!(inner.torque_guard.hold.is_some());
    // back out by servo: allowed; the hold ends once J4 is TORQUE_HOLD_RELEASE_RAD away
    let mut q = hold_q;
    let mut released_at = None;
    for k in 0..30 {
        q -= 0.01;
        servo_j4(&mut inner, q);
        inner.tick(DT);
        if released_at.is_none() && inner.torque_guard.hold.is_none() { released_at = Some(k); }
    }
    let obs = ticks(&mut inner, 20);
    assert!(obs.commanded_positions[J4] < hold_q - 0.2, "backed out to {}", obs.commanded_positions[J4]);
    let moved = hold_q - inner.measured_positions[J4];
    println!("SIM back out: hold ended after {released_at:?} ticks; J4 now {moved:.3} rad from the hold position");
    assert!(released_at.is_some() && moved >= TORQUE_HOLD_RELEASE_RAD);
    assert!(obs.motion_limit.as_ref().is_none_or(|l| l.reason != MotionLimitReason::Torque), "{:?}", obs.motion_limit);
    // back out by jog works too (fresh hold)
    let (mut inner, arm) = rig();
    push_j4_into_stop(&mut inner, &arm, stop);
    let mut t = inner.commanded_positions.clone();
    t[J4] -= 0.3;
    inner.jog_action(t, None, None, "joint_move").unwrap();
    let obs = ticks(&mut inner, 100);
    assert!((obs.commanded_positions[J4] - (stop - 0.3)).abs() < 1e-3, "jogged out to {}", obs.commanded_positions[J4]);
    assert!(inner.torque_guard.hold.is_none());
}

/// (c) a planned motion and a replay into the stop fail with TORQUE_LIMIT.
#[test]
fn running_action_and_replay_fail_with_torque_limit() {
    let (mut inner, arm) = rig();
    arm.with(|s| s.stop_hi[J4] = Some(INSPECT[J4] + 0.1));
    let mut t = INSPECT.to_vec();
    t[J4] += 0.5;
    let (_, id, _) = inner.jog_action(t, None, None, "joint_move").unwrap();
    let mut obs = ticks(&mut inner, 1);
    for _ in 0..200 {
        if obs.action.as_ref().is_some_and(|a| a.state != "running") { break; }
        obs = ticks(&mut inner, 1);
    }
    let a = obs.action.as_ref().unwrap();
    assert_eq!((a.id, a.state.as_str(), a.code.as_deref()), (id, "failed", Some("TORQUE_LIMIT")), "{a:?}");
    assert!(a.message.as_deref().unwrap().contains("joint 4"), "{a:?}");
    println!("action: {}", a.message.as_deref().unwrap());
    assert!(inner.active_trajectory.is_none() && obs.is_torqued && obs.fault.is_none());

    // replay of a recorded J4 motion that now meets a stop
    let tmp = tempfile::tempdir().unwrap();
    let frames: Vec<RecordFrame> = (0..=150).map(|k| {
        let s = (1.0 - (std::f64::consts::PI * k as f64 / 150.0).cos()) as f32 / 2.0;
        let mut q = INSPECT.to_vec();
        q[J4] += 0.6 * s;
        RecordFrame { timestamp: 100.0 + k as f64 * DT, observation_state: q.clone(), action: q }
    }).collect();
    write_frames_to_parquet(&tmp.path().join("j4.parquet"), &frames).unwrap();
    let (mut inner, arm) = rig();
    inner.episodes = armd::recording::EpisodeStore::new(Some(tmp.path().to_path_buf()));
    let epoch = inner.safety.epoch_id();
    let (id, _) = inner.start_replay(ReplayRequest { episode: "j4".into(), speed: 1.0, gripper: true, tag: None }, epoch).unwrap();
    arm.with(|s| s.stop_hi[J4] = Some(INSPECT[J4] + 0.2));
    let mut obs = ticks(&mut inner, 1);
    for _ in 0..1000 {
        if obs.action.as_ref().is_some_and(|a| a.state != "running") { break; }
        obs = ticks(&mut inner, 1);
    }
    let a = obs.action.as_ref().unwrap();
    assert_eq!((a.id, a.kind.as_str(), a.state.as_str(), a.code.as_deref()), (id, "replay", "failed", Some("TORQUE_LIMIT")), "{a:?}");
    assert!(inner.active_trajectory.is_none() && torque_limit(&obs) == Some(J4));
}

/// (d) high but brief torque (< persist_ms), and a stretched arm holding gravity, do not trip.
#[test]
fn brief_peaks_and_gravity_hold_do_not_trip() {
    let (mut inner, arm) = rig();
    // 4 ticks (80 ms) at 9.5 N·m, 1 tick below the threshold, repeated
    for _ in 0..20 {
        arm.with(|s| s.forced[J4] = Some(9.5));
        ticks(&mut inner, 4);
        arm.with(|s| s.forced[J4] = Some(THR_WRIST - 0.1));
        ticks(&mut inner, 1);
    }
    arm.with(|s| s.forced[J4] = None);
    assert!(inner.torque_guard.hold.is_none() && inner.torque_guard.trips_total == 0);

    // Largest model gravity torque per joint over a grid of J2..J6 within the joint limits
    // (13 points per joint; J1 does not change gravity torques).
    let limits = ArmConfig::rebot_dm().limits;
    const N: usize = 13;
    let at = |j: usize, k: usize| limits[j].min_position + (limits[j].max_position - limits[j].min_position) * k as f32 / (N - 1) as f32;
    let mut max = [(0.0f32, [0.0f32; 6]); 6];
    for a in 0..N { for b in 0..N { for c in 0..N { for d in 0..N { for e in 0..N {
        let q6 = [0.0, at(1, a), at(2, b), at(3, c), at(4, d), at(5, e)];
        let g = armd::compute_gravity_torques(&q6).unwrap();
        for j in 0..6 { if g[j].abs() > max[j].0 { max[j] = (g[j].abs(), q6); } }
    }}}}}
    for (j, (g, q)) in max.iter().enumerate() {
        let thr = if j < 3 { THR_BIG } else { THR_WRIST };
        println!("SIM grid max model gravity |τ{}| {g:.2} N·m vs threshold {thr} ({:.0}%) at q {:?}", j + 1, 100.0 * g / thr,
            q.iter().map(|v| (v * 1000.0).round() / 1000.0).collect::<Vec<_>>());
        assert!(*g < thr, "J{} gravity alone reaches the threshold", j + 1);
    }
    let (g2, q6) = max[1];
    let mut q = INSPECT;
    q[..6].copy_from_slice(&q6);
    assert!(g2 > 10.0, "{g2}");
    let (mut inner, _arm) = rig_at(ArmConfig { workspace: None, ..profile() }, q);
    let obs = ticks(&mut inner, 100);
    assert!((obs.torques[1].abs() - g2).abs() < 0.5, "reported {}", obs.torques[1]);
    // and a move through high-gravity poses
    let mut t = q.to_vec();
    t[2] = INSPECT[2];
    t[1] = INSPECT[1];
    inner.jog_action(t, None, None, "joint_move").unwrap();
    let obs = ticks(&mut inner, 400);
    assert_eq!(obs.action.as_ref().unwrap().state, "done", "{:?}", obs.action);
    assert!(inner.torque_guard.trips_total == 0 && inner.torque_guard.hold.is_none());
}

/// (e) gravity compensation is not watched.
#[test]
fn gravity_comp_does_not_trip() {
    let (mut inner, arm) = rig();
    if inner.gravity_comp_support().is_err() {
        eprintln!("skipped: gravity model URDF not found ({:?})", inner.gravity_comp_support());
        return;
    }
    inner.enter_gravity_comp(Default::default()).unwrap();
    ticks(&mut inner, 5);
    assert!(inner.gravity_comp.is_some());
    arm.with(|s| s.forced[1] = Some(27.0));
    arm.with(|s| s.forced[J4] = Some(9.9));
    ticks(&mut inner, 50);
    assert!(inner.gravity_comp.is_some(), "still in gravity compensation");
    assert!(inner.torque_guard.trips_total == 0 && inner.torque_guard.hold.is_none());
}

/// (f) the gripper is excluded: a J7 stall at 9.5 N·m (95 % of its TMAX 10) does not trip.
#[test]
fn gripper_stall_does_not_trip() {
    let (mut inner, arm) = rig();
    arm.with(|s| s.forced[6] = Some(9.5));
    let obs = ticks(&mut inner, 100);
    assert!((obs.torques[6] - 9.5).abs() < 1e-6);
    assert!(inner.torque_guard.trips_total == 0 && inner.torque_guard.hold.is_none());
}

/// (g) J4 incident (real arm, 2026-11-12): torque -2 -> +9.5 N·m over 400 ms at a fixed
/// position. The guard trips within 100 ms of |τ4| reaching the 6.0 N·m threshold, before
/// 9.5 N·m has been held for 100 ms.
#[test]
fn j4_incident_ramp_trips_before_9_5_is_held_100_ms() {
    let (mut inner, arm) = rig();
    // a fixed stop at the current position, and a client still pushing (servo stream)
    arm.with(|s| s.stop_hi[J4] = Some(INSPECT[J4]));
    let (mut t_over, mut t_peak, mut t_trip) = (None, None, None);
    for k in 0..60usize {
        let t_ms = k as f64 * DT * 1000.0;
        let tau = if t_ms <= 400.0 { -2.0 + 11.5 * (t_ms / 400.0) as f32 } else { 9.5 };
        arm.with(|s| s.forced[J4] = Some(tau));
        if inner.torque_guard.hold.is_none() { servo_j4(&mut inner, INSPECT[J4] + 0.1); }
        let obs = inner.tick(DT);
        if t_over.is_none() && obs.torques[J4].abs() >= THR_WRIST { t_over = Some(t_ms); }
        if t_peak.is_none() && obs.torques[J4] >= 9.5 - 1e-4 { t_peak = Some(t_ms); }
        if t_trip.is_none() && torque_limit(&obs) == Some(J4) {
            t_trip = Some(t_ms);
            assert!(obs.safety_state.is_operational() && obs.is_torqued);
            assert!((obs.commanded_positions[J4] - INSPECT[J4]).abs() < 1e-4);
        }
    }
    let (t_over, t_peak, t_trip) = (t_over.unwrap(), t_peak.unwrap(), t_trip.expect("tripped"));
    println!("SIM J4 incident ramp: |τ4| >= {THR_WRIST} at {t_over:.0} ms, 9.5 reached at {t_peak:.0} ms, trip at {t_trip:.0} ms ({:.0} ms after the threshold, {:.0} ms relative to reaching 9.5)",
        t_trip - t_over, t_trip - t_peak);
    assert!(t_trip - t_over <= 100.0 + 1e-6 && t_trip - t_over >= 80.0 - 1e-6, "trip {} ms after the threshold", t_trip - t_over);
    assert!(t_trip - t_peak < 100.0, "9.5 N·m held {} ms before the trip", t_trip - t_peak);
}

/// Bus loss, then the motors come back Disabled (power reset): torque state off, Fault kept.
#[test]
fn bus_loss_then_motors_back_disabled_sets_torque_off_and_keeps_the_fault() {
    let (mut inner, arm) = rig();
    arm.with(|s| s.stale = true);
    let obs = ticks(&mut inner, 3);
    assert!(matches!(obs.safety_state, SafetyState::Fault(_)), "{:?}", obs.safety_state);
    assert_eq!(obs.fault.as_ref().unwrap().code, FaultCode::BusLoss);
    assert!(obs.is_torqued, "torque state unknown while feedback is stale");
    // feedback returns, every motor reports Disabled
    arm.with(|s| { s.stale = false; s.torqued = false; });
    let obs = ticks(&mut inner, 1);
    println!("SIM bus loss + power reset: is_torqued {} enable_state {:?} fault {:?}", obs.is_torqued, obs.enable_state, obs.fault.as_ref().map(|f| f.code));
    assert!(!obs.is_torqued, "motors are off");
    assert_eq!(obs.enable_state, EnableState::Disabled);
    assert_eq!(obs.fault.as_ref().unwrap().code, FaultCode::BusLoss, "fault kept until clear_fault");
    let obs = ticks(&mut inner, 20);
    assert!(!obs.is_torqued && matches!(obs.safety_state, SafetyState::Fault(_)));
    // clear_fault works and leaves torque off; re-arm is a normal torque-on
    inner.clear_fault().unwrap();
    let obs = ticks(&mut inner, 2);
    assert!(!obs.is_torqued && obs.fault.is_none(), "{:?}", obs.safety_state);
    inner.set_torque(true).unwrap();
    assert!(ticks(&mut inner, 2).is_torqued);
}

/// Bus loss, motors come back Enabled: unchanged behaviour (torque on, Fault kept).
#[test]
fn bus_loss_then_motors_back_enabled_is_unchanged() {
    let (mut inner, arm) = rig();
    arm.with(|s| s.stale = true);
    ticks(&mut inner, 3);
    arm.with(|s| s.stale = false);
    let obs = ticks(&mut inner, 10);
    assert!(obs.is_torqued);
    assert_eq!(obs.fault.as_ref().unwrap().code, FaultCode::BusLoss);
    inner.clear_fault().unwrap();
    assert!(ticks(&mut inner, 2).is_torqued);
}
