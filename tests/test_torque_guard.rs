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
use armd::torque_guard::{TorqueGuardInfo, TorqueGuardPolicy, TORQUE_HOLD_BACKOFF_RAD, TORQUE_HOLD_RELEASE_RAD};
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
    /// Elastic obstacle per joint: (contact position, N·m per rad past it). The reported
    /// torque adds stiffness x (q - contact) while q is past the contact, with the motor
    /// holding the commanded position (integral action), so a held joint keeps pressing.
    elastic: [Option<(f32, f32)>; 7],
}

#[derive(Clone)]
struct StopArm(Arc<Mutex<ArmState>>);

impl StopArm {
    fn new(q: [f32; 7]) -> Self { Self(Arc::new(Mutex::new(ArmState { q, cmd: q, ..Default::default() }))) }
    fn with<R>(&self, f: impl FnOnce(&mut ArmState) -> R) -> R { f(&mut self.0.lock().unwrap()) }
}

impl ArmDriver for StopArm {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true, torque_feedforward: false, enable_feedback: true, load_feedback: false }
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
            let contact = if s.torqued && !s.mit { s.elastic[i].map_or(0.0, |(c, k)| k * (s.q[i] - c).max(0.0)) } else { 0.0 };
            let model = if i < 6 { g[i] } else { 0.0 } + push + contact;
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
    { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| armd::SupervisorInner::enable_torque(__a)) }.unwrap();
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
    { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).and_then(|__a| armd::SupervisorInner::servo(__a, t, epoch)) } }.unwrap();
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
    assert!((obs.commanded_positions[J4] - (stop - TORQUE_HOLD_BACKOFF_RAD)).abs() < 1e-4, "held backed off from the measured position {}", obs.commanded_positions[J4]);
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
        { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).and_then(|__a| armd::SupervisorInner::servo(__a, t, epoch)) } }.unwrap();
        let obs = inner.tick(DT);
        assert!(obs.commanded_positions[J4] <= hold_q + 1e-5, "J4 pushed in: {}", obs.commanded_positions[J4]);
        assert!(obs.torques[J4].abs() < THR_WRIST, "no torque build-up {}", obs.torques[J4]);
        assert_eq!(torque_limit(&obs), Some(J4));
    }
    assert!(inner.commanded_positions[0] > INSPECT[0] + 0.05, "J1 followed the stream");
    // push in by jog (cockpit joint jog): the J4 target is clamped
    let mut t = inner.commanded_positions.clone();
    t[J4] = hold_q + 0.3;
    { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::jog_action(__a, t, None, None, "joint_move")) } }.unwrap();
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
    { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::jog_action(__a, t, None, None, "joint_move")) } }.unwrap();
    let obs = ticks(&mut inner, 100);
    assert!((obs.commanded_positions[J4] - (stop - TORQUE_HOLD_BACKOFF_RAD - 0.3)).abs() < 1e-3, "jogged out to {}", obs.commanded_positions[J4]);
    assert!(inner.torque_guard.hold.is_none());
}

/// (c) a planned motion and a replay into the stop fail with TORQUE_LIMIT.
#[test]
fn running_action_and_replay_fail_with_torque_limit() {
    let (mut inner, arm) = rig();
    arm.with(|s| s.stop_hi[J4] = Some(INSPECT[J4] + 0.1));
    let mut t = INSPECT.to_vec();
    t[J4] += 0.5;
    let (_, id, _) = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::jog_action(__a, t, None, None, "joint_move")) } }.unwrap();
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
    let (id, _) = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_replay(__a, ReplayRequest { episode: "j4".into(), speed: 1.0, gripper: true, tag: None, allow_stale: false }, epoch)) } }.unwrap();
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
    { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::jog_action(__a, t, None, None, "joint_move")) } }.unwrap();
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
    { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::enter_gravity_comp(__a, Default::default())) } }.unwrap();
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
            assert!((obs.commanded_positions[J4] - (INSPECT[J4] - TORQUE_HOLD_BACKOFF_RAD)).abs() < 1e-4);
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
    // feedback returns, every motor reports Disabled (supervisor no longer runs decode-seq sync
    // during stopped ticks; OffConfirmed motors are skipped by measured-position holds, and
    // explicit clear_fault reconciles is_torqued = false once every motor is Disabled).
    arm.with(|s| { s.stale = false; s.torqued = false; });
    let obs = ticks(&mut inner, 1);
    println!("SIM bus loss + power reset: is_torqued {} enable_state {:?} fault {:?}", obs.is_torqued, obs.enable_state, obs.fault.as_ref().map(|f| f.code));
    assert_eq!(obs.enable_state, EnableState::Disabled);
    assert_eq!(obs.fault.as_ref().unwrap().code, FaultCode::BusLoss, "fault kept until clear_fault");
    let obs = ticks(&mut inner, 20);
    assert!(matches!(obs.safety_state, SafetyState::Fault(_)));
    // clear_fault works and leaves torque off; re-arm is a normal torque-on
    inner.clear_fault().unwrap();
    let obs = ticks(&mut inner, 2);
    assert!(!obs.is_torqued && obs.fault.is_none(), "{:?}", obs.safety_state);
    { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| armd::SupervisorInner::enable_torque(__a)) }.unwrap();
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

// ---- 2026-11-21 incident: J2 blocked by an elastic obstacle, then park / watchdog park ----

const J2: usize = 1;
/// J2 stops at 0.033 rad (measured at the trip on the real arm).
const J2_BLOCK: f32 = 0.033;
/// Contact at the rest position 0, 650 N·m/rad: 21.4 N·m at the block (the real trip read
/// 20.14 N·m), 18.2 N·m at the backed-off hold, so the hold keeps pressing above 16.8 N·m.
const J2_CONTACT_K: f32 = 650.0;

/// B601-DM profile for the folded rest pose in this simulation: the model has no rest supports,
/// so J3 reports its full model gravity (-7.6 N·m) at the rest pose, above the profile's 7.0 N·m
/// support limit; the J2 support limit (2.333 N·m) stays checked.
fn rest_profile() -> ArmConfig {
    let mut c = profile();
    if let Some(r) = c.resting_pose.as_mut() { r.support_torque_limits.retain(|(j, _)| *j == J2); }
    c
}

/// Folded rest pose (all 0), J2 driven +0.12 rad into the obstacle by a joint_move (Action 12
/// of the incident). Returns the rig after the trip.
fn elastic_incident() -> (SupervisorInner, StopArm) {
    let (mut inner, arm) = rig_at(rest_profile(), [0.0; 7]);
    arm.with(|s| { s.stop_hi[J2] = Some(J2_BLOCK); s.elastic[J2] = Some((0.0, J2_CONTACT_K)); });
    let mut t = vec![0.0; 7];
    t[J2] = 0.12;
    let (_, id, _) = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::jog_action(__a, t, None, None, "joint_move")) } }.unwrap();
    for _ in 0..200 {
        let obs = inner.tick(DT);
        if obs.action.as_ref().is_some_and(|a| a.id == id && a.state != "running") {
            let a = obs.action.unwrap();
            assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some("TORQUE_LIMIT")), "{a:?}");
            assert!(a.message.as_deref().unwrap().contains("joint 2"), "{a:?}");
            println!("SIM incident trip: {}", a.message.unwrap());
            return (inner, arm);
        }
    }
    panic!("no trip on J2");
}

fn run_until_done(inner: &mut SupervisorInner, id: u64, max_ticks: usize) -> (ArmObservation, usize) {
    for k in 0..max_ticks {
        let obs = inner.tick(DT);
        if obs.action.as_ref().is_some_and(|a| a.id == id && a.state != "running") { return (obs, k + 1); }
    }
    panic!("action {id} still running after {max_ticks} ticks");
}

/// The hold backs J2 off by TORQUE_HOLD_BACKOFF_RAD; with the elastic obstacle it still presses
/// above the threshold (as on the real arm). The SDK park (a move back toward 0) then succeeds
/// and ends with torque off.
#[test]
fn elastic_incident_park_retreats_and_ends_with_torque_off() {
    let (mut inner, _arm) = elastic_incident();
    let h = inner.torque_guard.hold.expect("held");
    assert_eq!((h.joint, h.sign), (J2, 1.0));
    assert!((h.anchor_q - J2_BLOCK).abs() < 1e-4, "trip position {}", h.anchor_q);
    let obs = ticks(&mut inner, 50);
    println!("SIM hold after 1 s: J2 cmd {:.4} meas {:.4} rad, |τ2| {:.2} N·m (threshold {THR_BIG})",
        obs.commanded_positions[J2], inner.measured_positions[J2], obs.torques[J2]);
    assert!((obs.commanded_positions[J2] - (J2_BLOCK - TORQUE_HOLD_BACKOFF_RAD)).abs() < 1e-4, "{}", obs.commanded_positions[J2]);
    assert!(obs.torques[J2] > THR_BIG, "the elastic obstacle keeps the held joint pressing: {}", obs.torques[J2]);
    assert!(obs.torques[J2] < h.torque_nm, "the back-off lowers the torque: {} vs {}", obs.torques[J2], h.torque_nm);
    let trips = inner.torque_guard.trips_total;
    let id = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::unload_action(__a)) } }.unwrap().expect("park started");
    let mut below = None;
    let mut k_end = 0;
    for k in 0..800 {
        let obs = inner.tick(DT);
        if below.is_none() && obs.torques[J2] < THR_BIG { below = Some(k as f64 * DT); }
        assert!(obs.torques[J2] <= h.torque_nm + 0.1 * THR_BIG, "torque rose during the retreat: {}", obs.torques[J2]);
        if obs.action.as_ref().is_some_and(|a| a.id == id && a.state != "running") { k_end = k + 1; break; }
    }
    let obs = inner.current_observation();
    let a = obs.action.as_ref().unwrap();
    println!("SIM park from the hold: |τ2| below {THR_BIG} N·m after {:.2} s; park {} after {:.2} s; torqued {}",
        below.unwrap_or(f64::NAN), a.state, k_end as f64 * DT, obs.is_torqued);
    assert_eq!((a.id, a.state.as_str()), (id, "done"), "{a:?}");
    assert!(!obs.is_torqued && obs.fault.is_none(), "{:?}", obs.safety_state);
    assert_eq!(inner.torque_guard.trips_total, trips, "no re-trip during the park");
    assert!(below.unwrap() < 1.5, "torque relaxed within the retreat window");
}

/// Replay of a recorded J2 motion +0.1 rad from the current command (replay is not clamped).
fn replay_j2_in(inner: &mut SupervisorInner) -> (ArmObservation, f32) {
    let tmp = tempfile::tempdir().unwrap();
    let start = inner.commanded_positions.clone();
    let frames: Vec<RecordFrame> = (0..=100).map(|k| {
        let mut q = start.clone();
        q[J2] += 0.1 * k as f32 / 100.0;
        RecordFrame { timestamp: 100.0 + k as f64 * DT, observation_state: q.clone(), action: q }
    }).collect();
    write_frames_to_parquet(&tmp.path().join("j2.parquet"), &frames).unwrap();
    inner.episodes = armd::recording::EpisodeStore::new(Some(tmp.path().to_path_buf()));
    let epoch = inner.safety.epoch_id();
    let (id, _) = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_replay(__a, ReplayRequest { episode: "j2".into(), speed: 1.0, gripper: true, tag: None, allow_stale: false }, epoch)) } }.unwrap();
    let mut max_cmd = f32::MIN;
    for _ in 0..300 {
        let o = inner.tick(DT);
        max_cmd = max_cmd.max(o.commanded_positions[J2]);
        if o.action.as_ref().is_some_and(|a| a.id == id && a.state != "running") { return (o, max_cmd); }
    }
    panic!("replay still running");
}

/// A motion that pushes J2 further into the obstacle: a joint_move is clamped at the hold
/// position; a replay (not clamped) trips, by the persist rule while the obstacle still
/// loads the joint, and at once on passing the trip position when it does not.
#[test]
fn elastic_incident_push_further_in_is_refused_or_trips() {
    use armd::torque_guard::TORQUE_PUSH_IN_RAD;
    let (mut inner, arm) = elastic_incident();
    let hold_q = inner.torque_guard.hold.unwrap().hold_q;
    let mut t = inner.commanded_positions.clone();
    t[J2] = 0.2;
    { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::jog_action(__a, t, None, None, "joint_move")) } }.unwrap();
    for _ in 0..60 {
        let obs = inner.tick(DT);
        assert!(obs.commanded_positions[J2] <= hold_q + 1e-5, "J2 pushed in: {}", obs.commanded_positions[J2]);
    }
    let anchor = inner.torque_guard.hold.unwrap().anchor_q;
    let (obs, max_cmd) = replay_j2_in(&mut inner);
    let a = obs.action.as_ref().unwrap();
    println!("SIM replay into the elastic obstacle: {} {:?}: {}; J2 cmd peaked {max_cmd:.4} rad (trip position {anchor:.4})", a.state, a.code, a.message.as_deref().unwrap_or(""));
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some("TORQUE_LIMIT")), "{a:?}");
    assert!(max_cmd <= anchor + TORQUE_PUSH_IN_RAD + 1e-4, "pushed {} past the trip position", max_cmd - anchor);
    assert!(obs.is_torqued && obs.fault.is_none() && inner.torque_guard.hold.is_some_and(|h| h.joint == J2));
    // the obstacle no longer loads the held joint (a rigid stop at the trip position remains):
    // the replay trips on passing the trip position + TORQUE_PUSH_IN_RAD
    arm.with(|s| s.elastic[J2] = None);
    ticks(&mut inner, 5);
    assert!(inner.torque_guard.hold.is_some_and(|h| h.joint == J2), "hold kept");
    let (obs, max_cmd) = replay_j2_in(&mut inner);
    let a = obs.action.as_ref().unwrap();
    println!("SIM replay into a rigid stop after the trip: {} {:?}: {}; J2 cmd peaked {max_cmd:.4} rad, torque {:.2}", a.state, a.code, a.message.as_deref().unwrap_or(""), obs.torques[J2]);
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some("TORQUE_LIMIT")), "{a:?}");
    assert!(a.message.as_deref().unwrap().contains("past the position of the torque trip"), "{a:?}");
    assert!(max_cmd <= anchor + TORQUE_PUSH_IN_RAD + 0.003, "pushed {} past the trip position", max_cmd - anchor);
}

/// During the retreat park J2 is exempt, the other joints are not: a J4 saturation trips.
#[test]
fn elastic_incident_other_joint_still_trips_during_the_retreat() {
    let (mut inner, arm) = elastic_incident();
    let id = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::unload_action(__a)) } }.unwrap().unwrap();
    ticks(&mut inner, 5);
    arm.with(|s| s.forced[J4] = Some(9.5));
    let (obs, k) = run_until_done(&mut inner, id, 50);
    let a = obs.action.as_ref().unwrap();
    println!("SIM J4 saturation during the retreat park: {} {:?} after {} ticks: {}", a.state, a.code, k, a.message.as_deref().unwrap_or(""));
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some("TORQUE_LIMIT")), "{a:?}");
    assert!(a.message.as_deref().unwrap().contains("joint 4"), "{a:?}");
    assert!(k <= 6, "{k} ticks");
    assert_eq!(inner.torque_guard.hold.unwrap().joint, J4);
}

/// The retreat allowance is bounded: a torque rising beyond the trip torque + margin trips at
/// once; a torque that stays above the threshold trips once TORQUE_RETREAT_WINDOW_S has passed.
#[test]
fn elastic_incident_retreat_allowance_is_bounded() {
    use armd::torque_guard::{TORQUE_RETREAT_MARGIN_RATIO, TORQUE_RETREAT_WINDOW_S};
    // rise
    let (mut inner, arm) = elastic_incident();
    let trip = inner.torque_guard.hold.unwrap().torque_nm;
    let id = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::unload_action(__a)) } }.unwrap().unwrap();
    ticks(&mut inner, 3);
    arm.with(|s| s.forced[J2] = Some(trip + TORQUE_RETREAT_MARGIN_RATIO * THR_BIG + 0.5));
    let (obs, k) = run_until_done(&mut inner, id, 20);
    println!("SIM torque rise during the retreat: {:?} after {k} ticks", obs.action.as_ref().unwrap().code);
    assert_eq!(obs.action.as_ref().unwrap().code.as_deref(), Some("TORQUE_LIMIT"));
    assert!(k <= 5, "trips within persist_ms of the rise: {k} ticks");
    // a load that does not relax, during a 3 s joint_move back (the park trajectory is
    // 1.5 s long, after which its settle phase is not watched)
    let (mut inner, arm) = elastic_incident();
    arm.with(|s| s.forced[J2] = Some(THR_BIG + 1.0));
    let mut t = inner.commanded_positions.clone();
    t[J2] = -0.2;
    let (_, id, _) = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::jog_action(__a, t, Some(3.0), None, "joint_move")) } }.unwrap();
    let (obs, k) = run_until_done(&mut inner, id, 200);
    let t = k as f64 * DT;
    println!("SIM constant {:.1} N·m during the retreat: {:?} after {t:.2} s", THR_BIG + 1.0, obs.action.as_ref().unwrap().code);
    assert_eq!(obs.action.as_ref().unwrap().code.as_deref(), Some("TORQUE_LIMIT"));
    assert!(t > TORQUE_RETREAT_WINDOW_S - DT && t <= TORQUE_RETREAT_WINDOW_S + 2.0 * DT, "{t}");
}

/// Lease loss after the trip (incident 01:03:38): the watchdog park retreats and completes
/// with torque off, no WatchdogTimeout.
#[test]
fn elastic_incident_lease_loss_after_trip_watchdog_park_completes() {
    let (mut inner, _arm) = elastic_incident();
    ticks(&mut inner, 20);
    inner.lease_enabled = true;
    inner.lease_timeout = Duration::from_millis(10);
    inner.last_heartbeat = std::time::Instant::now() - Duration::from_secs(1);
    let obs = inner.tick(DT);
    assert!(inner.watchdog_park.is_some() && inner.pending_normal_unload, "watchdog park started");
    assert!(obs.fault.is_none());
    let mut done = None;
    for k in 0..800 {
        let obs = inner.tick(DT);
        assert!(obs.fault.is_none(), "fault {:?} at tick {k}", obs.fault);
        if !obs.is_torqued { done = Some(k); break; }
    }
    let obs = inner.current_observation();
    println!("SIM watchdog park after the trip: torque off after {:.2} s, safety {:?}, fault {:?}",
        done.map_or(f64::NAN, |k| k as f64 * DT), obs.safety_state, obs.fault);
    assert!(done.is_some() && !obs.is_torqued && obs.fault.is_none() && inner.watchdog_park.is_none());
}

/// Lease loss while the resting pose lies in the push direction of the hold: no park is
/// started (it would push into the load); torque stays on at the backed-off hold and
/// WatchdogTimeout is latched with that reason.
#[test]
fn lease_loss_with_the_rest_pose_behind_the_load_holds_and_latches() {
    let mut q = [0.0; 7];
    q[J2] = -0.2;
    let (mut inner, arm) = rig_at(rest_profile(), q);
    arm.with(|s| { s.stop_hi[J2] = Some(-0.1); s.elastic[J2] = Some((-0.133, J2_CONTACT_K)); });
    let mut t = q.to_vec();
    t[J2] = 0.0;
    let (_, id, _) = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::jog_action(__a, t, None, None, "joint_move")) } }.unwrap();
    let (obs, _) = run_until_done(&mut inner, id, 200);
    assert_eq!(obs.action.as_ref().unwrap().code.as_deref(), Some("TORQUE_LIMIT"));
    let hold_q = inner.torque_guard.hold.unwrap().hold_q;
    inner.lease_enabled = true;
    inner.lease_timeout = Duration::from_millis(10);
    inner.last_heartbeat = std::time::Instant::now() - Duration::from_secs(1);
    let obs = ticks(&mut inner, 3);
    println!("SIM lease loss, rest behind the load: safety {:?}, torqued {}, J2 cmd {:.4} (hold {hold_q:.4})", obs.safety_state, obs.is_torqued, obs.commanded_positions[J2]);
    assert!(inner.watchdog_park.is_none());
    assert_eq!(obs.fault.as_ref().map(|f| f.code), Some(FaultCode::WatchdogTimeout));
    assert!(obs.is_torqued);
    assert!((obs.commanded_positions[J2] - hold_q).abs() < 1e-5, "kept the backed-off hold: {}", obs.commanded_positions[J2]);
}

/// A macro whose child pushes into a stop: the guard ends it and its terminal record names
/// TORQUE_LIMIT with the child count.
#[test]
fn a_macro_child_into_a_stop_records_the_macro_terminal() {
    use armd::rt_core::broker::{ActionId, Outcome};
    use armd::rt_core::ops::CoreOps;
    let (mut inner, arm) = rig();
    arm.with(|s| s.stop_hi[J4] = Some(INSPECT[J4] + 0.1));
    let epoch = inner.safety.epoch_id();
    let id = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::begin_orchestrated_action(__a, "sequence", None, epoch)) } }.unwrap();
    let mut t = INSPECT.to_vec();
    t[J4] += 0.5;
    let limits = inner.planning_limits();
    { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::install_macro_quintic(__a, id, t, &limits)) } }.unwrap();
    for _ in 0..200 { inner.tick(DT); if inner.macro_run.is_none() { break; } }
    assert!(inner.macro_run.is_none(), "the guard ended the macro");
    let rec = inner.core.last_macro().expect("terminal record");
    assert_eq!((rec.action, rec.children, rec.outcome), (ActionId(id), 1, Outcome::Failed("TORQUE_LIMIT")));
    assert_eq!(CoreOps::action_outcome(&inner, ActionId(id)), Some(Outcome::Failed("TORQUE_LIMIT")));
}
