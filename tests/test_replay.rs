//! Episode replay (`/api/v1/action/replay`, `src/replay.rs`) on a simulated arm.
//!
//! `LagArm` is a first-order POS_VEL servo (time constant 50 ms, 2 rad/s) that reports the
//! model gravity torque plus a residual the test sets per joint. It integrates either a
//! fixed 20 ms per control tick (deterministic tests) or the wall time between reads (the
//! record-and-replay test, whose timestamps are wall time like on the device). The numbers
//! printed here are simulation results, not hardware measurements.
mod common;

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::api::rest::ApiError;
use armd::collision::CollisionPolicy;
use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointMode, JointTarget};
use armd::model::{ArmConfig, JointState, KeepoutBox};
use armd::recording::{write_frames_to_parquet, RecordFrame};
use armd::replay::{parse_replay, plan_frames, ReplayRequest};
use armd::{ArmError, ArmSupervisor, SafetyState, SupervisorInner};
use common::INSPECT;
use serde_json::json;

const DT: f64 = 0.02;
const TAU: f32 = 0.05;

#[derive(Default)]
struct LagState {
    q: [f32; 7],
    v: [f32; 7],
    target: [f32; 7],
    torqued: bool,
    mit: bool,
    residual: [f32; 6],
    /// Joint held fixed (an obstacle).
    blocked: Option<usize>,
    /// Wall-clock integration when set (time of the last read).
    last_read: Option<Instant>,
}

#[derive(Clone)]
struct LagArm(Arc<Mutex<LagState>>);

impl LagArm {
    fn new(q: [f32; 7], wall_clock: bool) -> Self {
        Self(Arc::new(Mutex::new(LagState { q, target: q, last_read: wall_clock.then(Instant::now), ..Default::default() })))
    }
}

impl ArmDriver for LagArm {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true, torque_feedforward: false, enable_feedback: true, load_feedback: false }
    }
    fn enable_state(&self) -> EnableState { if self.0.lock().unwrap().torqued { EnableState::Enabled } else { EnableState::Disabled } }
    fn is_feedback_valid(&self) -> bool { true }
    fn is_enabled(&self) -> bool { self.0.lock().unwrap().torqued }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); 7] }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let mut s = self.0.lock().unwrap();
        let h = match s.last_read {
            Some(t) => { let now = Instant::now(); s.last_read = Some(now); (now - t).as_secs_f32().min(0.1) }
            None => DT as f32,
        };
        let steps = ((h / 0.001).ceil() as usize).max(1);
        let sub = h / steps as f32;
        for _ in 0..steps {
            for i in 0..7 {
                if !s.torqued || s.mit || s.blocked == Some(i) { s.v[i] = 0.0; continue; }
                let v = ((s.target[i] - s.q[i]) / TAU).clamp(-2.0, 2.0);
                s.v[i] = v;
                s.q[i] += v * sub;
            }
        }
        let mut q6 = [0.0; 6];
        q6.copy_from_slice(&s.q[..6]);
        let g = armd::compute_gravity_torques(&q6).unwrap();
        Ok((0..7).map(|i| JointState {
            position: s.q[i], velocity: s.v[i], temperature: 30.0,
            torque: if i < 6 { g[i] + s.residual[i] } else { 0.0 },
        }).collect())
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        let mut s = self.0.lock().unwrap();
        for (i, t) in targets.iter().enumerate().take(7) { s.target[i] = t.position; }
        Ok(())
    }
    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        let mut s = self.0.lock().unwrap();
        s.torqued = enabled;
        s.mit = false;
        s.target = s.q;
        Ok(())
    }
    fn supports_mit_mode_switch(&self) -> bool { true }
    fn set_joint_modes(&mut self, modes: &[JointMode], _hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.0.lock().unwrap().mit = modes.iter().take(6).any(|m| *m == JointMode::Mit);
        Ok(())
    }
}

fn profile() -> ArmConfig { ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() } }

/// Torqued, armed, lease-free SupervisorInner on a fixed-step LagArm at INSPECT, episode
/// directory `dir`.
fn rig(config: ArmConfig, dir: &std::path::Path) -> (SupervisorInner, LagArm) {
    let arm = LagArm::new(INSPECT, false);
    let mut inner = SupervisorInner::new(config);
    inner.lease_enabled = false;
    inner.driver = Some(Mutex::new(Box::new(arm.clone())));
    inner.commanded_positions = INSPECT.to_vec();
    inner.measured_positions = INSPECT.to_vec();
    inner.target_positions = INSPECT.to_vec();
    inner.episodes = armd::recording::EpisodeStore::new(Some(dir.to_path_buf()));
    { let __s = inner.current_admission_stamp(); inner.enable_torque(__s) }.unwrap();
    ticks(&mut inner, 5);
    assert!(inner.safety.state().is_operational(), "{:?}", inner.safety.state());
    (inner, arm)
}

fn ticks(inner: &mut SupervisorInner, n: usize) -> armd::ArmObservation {
    let mut obs = None;
    for _ in 0..n { obs = Some(inner.tick(DT)); }
    obs.unwrap()
}

/// Smooth out-and-back motion from `start`: joint `j` gets `amp[j] * (1 - cos(2πt/T)) / 2`,
/// sampled at 50 Hz (peak speed `amp * π / T`).
fn wave(start: [f32; 7], amp: [f32; 7], period: f64) -> Vec<RecordFrame> {
    let n = (period / DT).round() as usize;
    (0..=n).map(|k| {
        let t = k as f64 * DT;
        let s = (1.0 - (2.0 * std::f64::consts::PI * t / period).cos()) as f32 / 2.0;
        let q: Vec<f32> = (0..7).map(|j| start[j] + amp[j] * s).collect();
        RecordFrame { timestamp: 100.0 + t, observation_state: q.clone(), action: q }
    }).collect()
}

fn save(dir: &std::path::Path, id: &str, frames: &[RecordFrame]) {
    write_frames_to_parquet(&dir.join(format!("{id}.parquet")), frames).unwrap();
}

fn req(episode: &str, speed: f64, gripper: bool) -> ReplayRequest {
    ReplayRequest { episode: episode.into(), speed, gripper, tag: None, allow_stale: false }
}

fn code_of(e: ArmError) -> (u16, String, String) {
    let msg = e.to_string();
    let api = ApiError::from(e);
    (api.status.as_u16(), api.code.unwrap_or_default(), msg)
}

const AMP: [f32; 7] = [0.4, -0.2, 0.15, 0.0, 0.3, 0.0, -1.0];

// ---------------------------------------------------------------- validation (no motion)

#[test]
fn validation_rejects_with_codes_and_moves_nothing() {
    let tmp = tempfile::tempdir().unwrap();
    let (mut inner, _arm) = rig(profile(), tmp.path());
    let epoch = inner.safety.epoch_id();
    let config = inner.config.clone();
    let try_frames = |inner: &SupervisorInner, frames: &[RecordFrame], speed: f64, gripper: bool| {
        let input = inner.replay_input(req("x", speed, gripper), epoch).unwrap();
        plan_frames(input, frames).map(|_| ()).map_err(code_of)
    };

    // accepted: 4 s wave, peak 0.4π/4 = 0.31 rad/s
    let ok = wave(INSPECT, AMP, 4.0);
    assert!(try_frames(&inner, &ok, 1.0, true).is_ok());

    // empty / one frame / wrong joint count / non-finite
    assert_eq!(try_frames(&inner, &[], 0.5, true).unwrap_err().1, "REPLAY_EMPTY");
    assert_eq!(try_frames(&inner, &ok[..1], 0.5, true).unwrap_err().1, "REPLAY_EMPTY");
    let six: Vec<_> = ok.iter().map(|f| RecordFrame { observation_state: f.observation_state[..6].to_vec(), ..f.clone() }).collect();
    let (status, code, msg) = try_frames(&inner, &six, 0.5, true).unwrap_err();
    assert_eq!((status, code.as_str()), (422, "REPLAY_JOINT_COUNT"), "{msg}");
    let mut nan = ok.clone();
    nan[10].observation_state[2] = f32::NAN;
    assert_eq!(try_frames(&inner, &nan, 0.5, true).unwrap_err().1, "REPLAY_INVALID_EPISODE");

    // joint soft limit, any frame
    let mut over = ok.clone();
    over[50].observation_state[0] = config.limits[0].max_position + 0.05;
    let (status, code, msg) = try_frames(&inner, &over, 0.5, true).unwrap_err();
    assert_eq!((status, code.as_str()), (400, "SOFT_LIMIT_EXCEEDED"), "{msg}");
    assert!(msg.contains("joint 1") && msg.contains("t=1.00"), "{msg}");
    // the gripper limit applies only when the gripper is replayed
    let mut grip = ok.clone();
    grip[20].observation_state[6] = config.limits[6].min_position - 0.5;
    assert_eq!(try_frames(&inner, &grip, 0.5, true).unwrap_err().1, "SOFT_LIMIT_EXCEEDED");
    // (the jump into and out of that frame also exceeds the gripper speed)
    assert!(try_frames(&inner, &grip, 0.5, false).is_ok(), "held gripper: its frames are not checked");

    // too fast: 1.0 rad on J1 in 2 s -> peak 1.57 rad/s; planned limit 0.45
    let fast = wave(INSPECT, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 2.0);
    let (status, code, msg) = try_frames(&inner, &fast, 0.5, true).unwrap_err();
    println!("too fast: {msg}");
    assert_eq!((status, code.as_str()), (422, "REPLAY_TOO_FAST"));
    assert!(msg.contains("joint 1") && msg.contains("0.450") && msg.contains("at most 0.29x"), "{msg}");
    // 0.3 rad/s peak: fine at 1x; 0.6 rad/s peak: rejected at 1x, accepted at 0.5x
    let medium = wave(INSPECT, [0.0, 0.0, 0.0, 0.0, 0.6 * 2.0 / std::f32::consts::PI, 0.0, 0.0], 2.0);
    let (_, code, msg) = try_frames(&inner, &medium, 1.0, true).unwrap_err();
    assert_eq!(code, "REPLAY_TOO_FAST");
    assert!(msg.contains("joint 5") && msg.contains("at most 0.75x"), "{msg}");
    assert!(try_frames(&inner, &medium, 0.5, true).is_ok());
    // a recording at the J5 planned limit (0.45 rad/s) whose timestamps jitter against the control tick (as
    // recorded from a jog): single samples ~0.5 % above the limit replay at 1x; 5 % above is refused
    let at_limit = |over: f32| -> Vec<RecordFrame> {
        (0..=100).map(|k| {
            let jit = if k % 7 == 3 { -0.0001 } else { 0.0 };
            let mut q = INSPECT.to_vec();
            q[4] += 0.45 * over * (k as f32 * DT as f32);
            RecordFrame { timestamp: 100.0 + k as f64 * DT + jit, observation_state: q.clone(), action: q }
        }).collect()
    };
    assert!(try_frames(&inner, &at_limit(1.0), 1.0, true).is_ok(), "jittered recording at the limit");
    assert_eq!(try_frames(&inner, &at_limit(1.05), 1.0, true).unwrap_err().1, "REPLAY_TOO_FAST");

    // keep-out box on the path (around the TCP at the wave's far point)
    let far = &ok[ok.len() / 2].observation_state;
    let p = inner.config.arm_model.fk_f32(far).unwrap().p;
    let b = KeepoutBox { name: "bin".into(), min_m: [p[0] as f32 - 0.02, p[1] as f32 - 0.02, p[2] as f32 - 0.02], max_m: [p[0] as f32 + 0.02, p[1] as f32 + 0.02, p[2] as f32 + 0.02] };
    inner.config.workspace.as_mut().unwrap().keepout_boxes = vec![b];
    inner.workspace_revision += 1;
    let (status, code, msg) = try_frames(&inner, &ok, 0.5, true).unwrap_err();
    println!("keep-out: {msg}");
    assert_eq!((status, code.as_str()), (422, "WORKSPACE_VIOLATION"));
    assert!(msg.contains("keepout 'bin'"), "{msg}");
    inner.config.workspace.as_mut().unwrap().keepout_boxes.clear();
    inner.workspace_revision += 1;

    // a missing file, a bad id
    save(tmp.path(), "short", &ok[..1]);
    assert_eq!(code_of({ let s = inner.current_admission_stamp(); inner.start_replay(s, req("nope", 0.5, true), epoch) }.unwrap_err()).1, "EPISODE_NOT_FOUND");
    assert_eq!(code_of({ let s = inner.current_admission_stamp(); inner.start_replay(s, req("short", 0.5, true), epoch) }.unwrap_err()).1, "REPLAY_EMPTY");
    std::fs::write(tmp.path().join("junk.parquet"), b"no").unwrap();
    assert_eq!(code_of({ let s = inner.current_admission_stamp(); inner.start_replay(s, req("junk", 0.5, true), epoch) }.unwrap_err()).1, "REPLAY_INVALID_EPISODE");
    assert_eq!(code_of({ let s = inner.current_admission_stamp(); inner.start_replay(s, req("../a", 0.5, true), epoch) }.unwrap_err()).1, "INVALID_EPISODE_ID");
    assert_eq!(code_of({ let s = inner.current_admission_stamp(); inner.start_replay(s, req("x", 0.5, true), epoch + 1) }.unwrap_err()).1, "EPOCH_MISMATCH");

    // nothing moved and no action was recorded
    let obs = ticks(&mut inner, 5);
    assert!(obs.action.is_none() && inner.active_trajectory.is_none());
    assert!((0..7).all(|i| (obs.commanded_positions[i] - INSPECT[i]).abs() < 1e-6));

    // storage off
    inner.episodes = armd::recording::EpisodeStore::new(None);
    assert_eq!(code_of({ let s = inner.current_admission_stamp(); inner.start_replay(s, req("x", 0.5, true), epoch) }.unwrap_err()).1, "EPISODES_DISABLED");

    // parse
    assert!(parse_replay(&json!({"episode": "a", "speed": 0.25})).is_err());
}

// ---------------------------------------------------------------- busy rules

#[test]
fn replay_refused_while_recording_gravity_comp_collision_hold_or_moving() {
    let tmp = tempfile::tempdir().unwrap();
    save(tmp.path(), "w", &wave(INSPECT, AMP, 4.0));

    // recording
    let (mut inner, _arm) = rig(profile(), tmp.path());
    let epoch = inner.safety.epoch_id();
    inner.recorder.start(Some("rec".into()), Some(tmp.path().join("rec.parquet"))).unwrap();
    let (status, code, msg) = code_of({ let s = inner.current_admission_stamp(); inner.start_replay(s, req("w", 0.5, true), epoch) }.unwrap_err());
    assert_eq!((status, code.as_str()), (409, "MOTION_BUSY"), "{msg}");
    assert!(msg.contains("recording"), "{msg}");
    inner.recorder.discard().unwrap();
    // another action
    { let s = inner.current_admission_stamp(); inner.jog_action(s, INSPECT.iter().enumerate().map(|(i, q)| if i == 0 { q + 0.2 } else { *q }).collect(), None, Some(epoch), "joint_move") }.unwrap();
    ticks(&mut inner, 2);
    assert_eq!(code_of({ let s = inner.current_admission_stamp(); inner.start_replay(s, req("w", 0.5, true), epoch) }.unwrap_err()).1, "MOTION_BUSY");
    ticks(&mut inner, 150);
    assert!({ let s = inner.current_admission_stamp(); inner.start_replay(s, req("w", 0.5, true), epoch) }.is_ok(), "idle again: accepted");

    // gravity compensation (needs a lease)
    let (mut inner, _arm) = rig(profile(), tmp.path());
    inner.lease_enabled = true;
    inner.acquire_lease("c").unwrap();
    { let s = inner.current_admission_stamp(); inner.enter_gravity_comp(s, Default::default()) }.unwrap();
    ticks(&mut inner, 3);
    let epoch = inner.safety.epoch_id();
    let (_, code, msg) = code_of({ let s = inner.current_admission_stamp(); inner.start_replay(s, req("w", 0.5, true), epoch) }.unwrap_err());
    assert_eq!(code, "MOTION_BUSY");
    assert!(msg.contains("gravity"), "{msg}");

    // collision hold
    let config = ArmConfig { collision: Some(CollisionPolicy { enabled: true, resume_after_s: 30.0, ..CollisionPolicy::default() }), ..profile() };
    let (mut inner, arm) = rig(config, tmp.path());
    arm.0.lock().unwrap().residual[1] = 20.0;
    ticks(&mut inner, 6);
    assert!(inner.collision.hold.is_some());
    let epoch = inner.safety.epoch_id();
    assert_eq!(code_of({ let s = inner.current_admission_stamp(); inner.start_replay(s, req("w", 0.5, true), epoch) }.unwrap_err()).1, "COLLISION_HOLD");

    // E-stop
    let (mut inner, _arm) = rig(profile(), tmp.path());
    inner.apply_stop_policy(armd::StopPolicy::EStopLatch);
    let epoch = inner.safety.epoch_id();
    assert_eq!(code_of({ let s = inner.current_admission_stamp(); inner.start_replay(s, req("w", 0.5, true), epoch) }.unwrap_err()).1, "ESTOP_ACTIVE");
}

// ---------------------------------------------------------------- running a replay

struct Run { phases: Vec<String>, last: armd::motion::ActionStatus }

fn run(inner: &mut SupervisorInner, id: u64, max_s: f64, mut each: impl FnMut(&SupervisorInner, &armd::motion::ActionStatus)) -> Run {
    let mut phases: Vec<String> = Vec::new();
    for _ in 0..(max_s / DT) as usize {
        let obs = inner.tick(DT);
        let a = obs.action.clone().unwrap();
        assert_eq!(a.id, id);
        if let Some(p) = a.phase.as_deref() { if phases.last().map(String::as_str) != Some(p) { phases.push(p.to_string()); } }
        each(inner, &a);
        if !a.is_running() { return Run { phases, last: a }; }
    }
    panic!("replay did not finish within {max_s} s");
}

#[test]
fn approach_then_frames_gripper_on_and_off() {
    let tmp = tempfile::tempdir().unwrap();
    // the recording starts away from where the arm is: the approach moves there first
    let mut start = INSPECT;
    start[0] += 0.3;
    start[6] = -0.5;
    let frames = wave(start, AMP, 4.0);
    save(tmp.path(), "w", &frames);
    let (samples, _) = armd::replay::resample(&frames, DT).unwrap();

    for gripper in [true, false] {
        let (mut inner, _arm) = rig(profile(), tmp.path());
        let epoch = inner.safety.epoch_id();
        let (id, expected) = { let s = inner.current_admission_stamp(); inner.start_replay(s, req("w", 1.0, gripper), epoch) }.unwrap();
        let a = inner.last_action.clone().unwrap();
        assert_eq!((a.kind.as_str(), a.episode.as_deref(), a.phase.as_deref()), ("replay", Some("w"), Some("approach")));
        let mut first_follow: Option<Vec<f32>> = None;
        let mut max_cmd_err = 0.0f32;
        let mut j7: Vec<f32> = Vec::new();
        let mut n_ticks = 0usize;
        let r = run(&mut inner, id, expected + 3.0, |i, a| {
            n_ticks += 1;
            if a.phase.as_deref() == Some("replay") {
                if first_follow.is_none() { first_follow = Some(i.commanded_positions.clone()); }
                // the command is the recording at t_s (1x, no lag slowdown at these speeds)
                let k = (f64::from(a.t_s.unwrap()) / DT).round() as usize;
                let rec = &samples[k.min(samples.len() - 1)];
                for j in 0..6 { max_cmd_err = max_cmd_err.max((i.commanded_positions[j] - rec[j]).abs()); }
                j7.push(i.commanded_positions[6]);
            }
        });
        assert_eq!(r.phases, ["approach", "replay", "settle"], "{:?}", r.phases);
        assert_eq!((r.last.state.as_str(), r.last.settled), ("done", Some(true)), "{:?}", r.last);
        assert!((f64::from(r.last.t_s.unwrap()) - 4.0).abs() < 1e-3 && r.last.progress == 1.0);
        let ff = first_follow.unwrap();
        // first replay command is one 20 ms step into the recording, from frame 0
        assert!((0..6).all(|j| (ff[j] - frames[0].observation_state[j]).abs() < 0.01), "{ff:?}");
        assert!(max_cmd_err < 1e-4, "commanded = recording at t_s: {max_cmd_err}");
        let end = inner.measured_positions.clone();
        let last = &frames.last().unwrap().observation_state;
        let final_err = (0..6).map(|j| (end[j] - last[j]).abs()).fold(0.0, f32::max);
        println!("SIM replay gripper={gripper}: {n_ticks} ticks ({:.2} s control time, expected {expected:.2} s), max |cmd - rec| {max_cmd_err:.2e}, final pose error {final_err:.4} rad",
            n_ticks as f64 * DT);
        assert!(final_err < 0.01);
        let span = j7.iter().cloned().fold(f32::MIN, f32::max) - j7.iter().cloned().fold(f32::MAX, f32::min);
        if gripper {
            assert!(span > 0.9, "gripper follows the recording: span {span}");
        } else {
            assert!(span < 1e-6 && j7.iter().all(|q| (q - INSPECT[6]).abs() < 1e-6), "gripper held: span {span}");
        }
    }
}

/// Tick until the running replay `id` is in phase "replay" at t_s >= `t`.
fn until_t(inner: &mut SupervisorInner, id: u64, t: f32) {
    for _ in 0..1000 {
        let obs = inner.tick(DT);
        let a = obs.action.unwrap();
        assert!(a.id == id && a.is_running(), "{a:?}");
        if a.phase.as_deref() == Some("replay") && a.t_s.unwrap() >= t { return; }
    }
    panic!("replay did not reach t={t}");
}

fn max_change(a: &[f32], b: &[f32]) -> f32 { a.iter().zip(b).map(|(x, y)| (x - y).abs()).fold(0.0, f32::max) }

#[test]
fn stop_mid_replay_holds_estop_and_lease_loss_cancel() {
    let tmp = tempfile::tempdir().unwrap();
    save(tmp.path(), "w", &wave(INSPECT, AMP, 4.0));

    // stop (the calls of the WS /api/v1/motion/stop command)
    let (mut inner, _arm) = rig(profile(), tmp.path());
    let epoch = inner.safety.epoch_id();
    let (id, _) = { let s = inner.current_admission_stamp(); inner.start_replay(s, req("w", 1.0, true), epoch) }.unwrap();
    until_t(&mut inner, id, 1.0);
    inner.exit_gravity_comp("STOPPED", "Stop command");
    inner.stop_motion();
    inner.request_macro_stop();
    let obs = ticks(&mut inner, 1);
    let a = obs.action.unwrap();
    assert_eq!((a.id, a.state.as_str(), a.code.as_deref()), (id, "cancelled", Some("STOPPED")), "{a:?}");
    assert!((a.t_s.unwrap() - 1.0).abs() < 0.05, "{a:?}");
    ticks(&mut inner, 25); // braking from the replay speed
    let held = inner.commanded_positions.clone();
    let obs = ticks(&mut inner, 50);
    let drift = max_change(&obs.commanded_positions, &held);
    let lag = max_change(&obs.positions[..6], &obs.commanded_positions[..6]);
    println!("SIM stop at t=1.0 s: commanded drift over 1 s after braking {drift:.2e} rad, |meas - cmd| {lag:.4} rad");
    assert!(drift < 1e-4 && lag < 0.005 && obs.is_torqued && obs.safety_state.is_operational());

    // E-stop
    let (mut inner, _arm) = rig(profile(), tmp.path());
    let (id, _) = { let s = inner.current_admission_stamp(); inner.start_replay(s, req("w", 1.0, true), inner.safety.epoch_id()) }.unwrap();
    until_t(&mut inner, id, 1.0);
    inner.e_stop_latched.store(true, std::sync::atomic::Ordering::SeqCst);
    let obs = ticks(&mut inner, 1);
    let a = obs.action.unwrap();
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some("ESTOP_ACTIVE")), "{a:?}");
    assert!(matches!(obs.safety_state, SafetyState::EStop) && inner.active_trajectory.is_none());

    // lease loss
    let (mut inner, _arm) = rig(profile(), tmp.path());
    let generation = inner.acquire_lease("c").unwrap();
    let (id, _) = { let s = inner.current_admission_stamp(); inner.start_replay(s, req("w", 1.0, true), inner.safety.epoch_id()) }.unwrap();
    until_t(&mut inner, id, 0.5);
    assert!(inner.release_lease("c", generation));
    let obs = ticks(&mut inner, 1);
    let a = obs.action.unwrap();
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("cancelled", Some("LEASE_LOST")), "{a:?}");
    assert!(inner.active_trajectory.is_none());
}

#[test]
fn blocked_joint_stalls_and_holds_collision_trip_fails() {
    let tmp = tempfile::tempdir().unwrap();
    save(tmp.path(), "w", &wave(INSPECT, AMP, 4.0));
    // J1 0.8 rad out and back in 8 s (peak 0.31 rad/s)
    save(tmp.path(), "big", &wave(INSPECT, [0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 8.0));

    // J1 blocked early in the replay: the playback slows, pauses at the lag bound, then
    // fails TRACKING_STALL holding the measured pose (no Fault)
    let (mut inner, arm) = rig(profile(), tmp.path());
    let (id, _) = { let s = inner.current_admission_stamp(); inner.start_replay(s, req("big", 1.0, true), inner.safety.epoch_id()) }.unwrap();
    until_t(&mut inner, id, 0.5);
    arm.0.lock().unwrap().blocked = Some(0);
    let mut max_lag = 0.0f32;
    let r = run(&mut inner, id, 5.0, |i, _| max_lag = max_lag.max((i.commanded_positions[0] - i.measured_positions[0]).abs()));
    println!("SIM blocked J1: {} after max lag {max_lag:.3} rad: {:?}", r.last.state, r.last.message);
    assert_eq!((r.last.state.as_str(), r.last.code.as_deref()), ("failed", Some("TRACKING_STALL")), "{:?}", r.last);
    assert!(max_lag < inner.config.following_error.max_error_rad, "paused below the following-error bound");
    let obs = ticks(&mut inner, 30);
    assert!(obs.safety_state.is_operational() && obs.fault.is_none() && obs.is_torqued, "{:?}", obs.safety_state);
    assert!(max_change(&obs.commanded_positions[..6], &obs.positions[..6]) < 1e-3, "holding the measured pose");

    // collision guard trip during the replay
    let config = ArmConfig { collision: Some(CollisionPolicy { enabled: true, resume_after_s: 30.0, ..CollisionPolicy::default() }), ..profile() };
    let (mut inner, arm) = rig(config, tmp.path());
    let (id, _) = { let s = inner.current_admission_stamp(); inner.start_replay(s, req("w", 1.0, true), inner.safety.epoch_id()) }.unwrap();
    until_t(&mut inner, id, 1.0);
    arm.0.lock().unwrap().residual[2] = -20.0;
    let r = run(&mut inner, id, 1.0, |_, _| {});
    assert_eq!((r.last.state.as_str(), r.last.code.as_deref()), ("failed", Some("COLLISION")), "{:?}", r.last);
    assert!(inner.collision.hold.is_some() && inner.active_trajectory.is_none());
}

// ---------------------------------------------------------------- record, save, replay (wall clock)

/// Real-time ticks (20 ms period, real dt), heartbeat for client `c`; `each` sees every
/// observation. Returns the last one.
async fn realtime(sup: &ArmSupervisor, n: usize, mut each: impl FnMut(&armd::ArmObservation)) -> armd::ArmObservation {
    let mut interval = tokio::time::interval(Duration::from_millis(20));
    let mut last = Instant::now();
    let mut obs = None;
    for _ in 0..n {
        interval.tick().await;
        let now = Instant::now();
        let dt = (now - last).as_secs_f64().clamp(0.001, 0.08);
        last = now;
        let o = sup.tick(dt).await;
        each(&o);
        obs = Some(o);
    }
    obs.unwrap()
}

async fn wait_done(sup: &ArmSupervisor, id: u64, max_ticks: usize, mut each: impl FnMut(&armd::ArmObservation)) -> armd::motion::ActionStatus {
    let mut done = None;
    let mut interval = tokio::time::interval(Duration::from_millis(20));
    let mut last = Instant::now();
    for _ in 0..max_ticks {
        interval.tick().await;
        let now = Instant::now();
        let dt = (now - last).as_secs_f64().clamp(0.001, 0.08);
        last = now;
        let o = sup.tick(dt).await;
        each(&o);
        let a = o.action.clone().unwrap();
        if a.id == id && !a.is_running() { done = Some(a); break; }
    }
    done.expect("action did not finish")
}

/// Smallest max-joint distance (J1-J6) from `q` to the polyline through `path`.
fn path_distance(q: &[f32], path: &[Vec<f32>]) -> f32 {
    let mut best = f32::INFINITY;
    for w in path.windows(2) {
        // sample the segment finely; segments are 20 ms of motion (< 0.01 rad)
        for k in 0..=4 {
            let f = k as f32 / 4.0;
            let d = (0..6).map(|j| (q[j] - (w[0][j] + (w[1][j] - w[0][j]) * f)).abs()).fold(0.0, f32::max);
            best = best.min(d);
        }
    }
    best
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn record_jog_save_and_replay_matches_the_recording() {
    let tmp = tempfile::tempdir().unwrap();
    let arm = LagArm::new(INSPECT, true);
    let sup = ArmSupervisor::new(profile()).with_driver(Box::new(arm.clone()));
    sup.set_episodes_dir(Some(tmp.path().to_path_buf())).await;
    {
        let mut i = sup.inner().write().await;
        i.lease_enabled = false;
        i.commanded_positions = INSPECT.to_vec();
        i.target_positions = INSPECT.to_vec();
    }
    sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
    realtime(&sup, 10, |_| {}).await;

    // record two joint moves (peak commanded speed 1.875 * 0.4 / 2.5 = 0.30 rad/s)
    sup.start_recording(Some("jog".into()), None).await.unwrap();
    realtime(&sup, 10, |_| {}).await;
    let mut out = INSPECT.to_vec();
    out[0] += 0.4; out[1] -= 0.2; out[4] += 0.3; out[6] = -1.0;
    let back = { let mut b = INSPECT.to_vec(); b[0] += 0.1; b };
    for (target, secs) in [(out, 2.5), (back, 2.0)] {
        let epoch = sup.get_observation().await.epoch_id;
        let (_, id, _) = sup.inner().write().await.jog_action(sup.capture_admission_stamp(), target, Some(secs), Some(epoch), "joint_move").unwrap();
        wait_done(&sup, id, 400, |_| {}).await;
    }
    realtime(&sup, 10, |_| {}).await;
    let saved = sup.save_recording().await.unwrap();
    assert_eq!(saved, tmp.path().join("jog.parquet"));
    let frames = armd::read_frames_from_parquet(&saved).unwrap();
    let (rec, rec_dur) = armd::replay::resample(&frames, DT).unwrap();
    let listed = sup.episode_store().await.list().unwrap();
    assert_eq!(listed.episodes.len(), 1);
    assert_eq!((listed.episodes[0].id.as_str(), listed.episodes[0].frames), ("jog", frames.len()));
    println!("SIM recorded {} frames, {rec_dur:.2} s", frames.len());

    for speed in [0.5, 1.0] {
        let epoch = sup.get_observation().await.epoch_id;
        let (id, expected) = sup.inner().write().await.start_replay(sup.capture_admission_stamp(), req("jog", speed, true), epoch).unwrap();
        let mut phases: Vec<String> = Vec::new();
        let (mut cmd_err, mut path_err, mut aligned_err) = (0.0f32, 0.0f32, 0.0f32);
        let mut approach_end: Option<Vec<f32>> = None;
        let a = wait_done(&sup, id, ((expected + 3.0) / DT) as usize, |o| {
            let a = o.action.as_ref().unwrap();
            let phase = a.phase.as_deref().unwrap_or_default().to_string();
            if phases.last() != Some(&phase) {
                if phase == "replay" { approach_end = Some(o.commanded_positions.clone()); }
                phases.push(phase.clone());
            }
            if phase == "replay" {
                let t = f64::from(a.t_s.unwrap());
                let k = ((t / DT).floor() as usize).min(rec.len() - 2);
                let f = ((t - k as f64 * DT) / DT).clamp(0.0, 1.0) as f32;
                let at: Vec<f32> = (0..7).map(|j| rec[k][j] + (rec[k + 1][j] - rec[k][j]) * f).collect();
                cmd_err = cmd_err.max(max_change(&o.commanded_positions[..6], &at[..6]));
                aligned_err = aligned_err.max(max_change(&o.positions[..6], &at[..6]));
                path_err = path_err.max(path_distance(&o.positions, &rec));
            }
        }).await;
        let end = sup.get_observation().await.positions;
        let final_err = max_change(&end[..6], &frames.last().unwrap().observation_state[..6]);
        println!("SIM replay {speed}x: phases {phases:?}, {} / {:?}, cmd vs recording {cmd_err:.2e} rad, \
                  measured vs recording at t_s {aligned_err:.4} rad, measured distance to recorded path {path_err:.4} rad, final {final_err:.4} rad",
            a.state, a.settled);
        assert_eq!(phases, ["approach", "replay", "settle"]);
        assert_eq!((a.state.as_str(), a.settled), ("done", Some(true)), "{a:?}");
        let ae = approach_end.unwrap();
        assert!(max_change(&ae[..6], &frames[0].observation_state[..6]) < 0.01, "replay starts at frame 0 after the approach");
        // commanded = recording at t_s, by construction
        assert!(cmd_err < 1e-4, "{cmd_err}");
        // measured vs recording at the same recording time: first-order servo lag v * tau * speed,
        // peak recorded speed 0.30 rad/s, tau 0.05 s -> 0.015 rad at 1x; bound 2x that
        assert!(aligned_err < 0.03, "{aligned_err}");
        // time-independent path deviation
        assert!(path_err < 0.01, "{path_err}");
        assert!(final_err < 0.005, "{final_err}");
    }
}

// ---------------------------------------------------------------- WebSocket + REST

#[tokio::test]
async fn ws_replay_list_delete_and_disconnect_cancels() {
    let tmp = tempfile::tempdir().unwrap();
    save(tmp.path(), "w", &wave(INSPECT, AMP, 2.0 * 4.0));
    save(tmp.path(), "fast", &wave(INSPECT, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 2.0));
    let supervisor = Arc::new(ArmSupervisor::new(profile()));
    supervisor.set_episodes_dir(Some(tmp.path().to_path_buf())).await;
    let (mut ws, generation, epoch, supervisor, addr) = common::torqued_rig(supervisor, &INSPECT).await;
    let client = reqwest::Client::new();

    // list
    let list: serde_json::Value = client.get(format!("http://{addr}/api/v1/dataset/episodes")).send().await.unwrap().json().await.unwrap();
    assert_eq!(list["enabled"], true);
    let ids: Vec<&str> = list["episodes"].as_array().unwrap().iter().map(|e| e["id"].as_str().unwrap()).collect();
    assert_eq!(ids.len(), 2, "{list}");
    let w = list["episodes"].as_array().unwrap().iter().find(|e| e["id"] == "w").unwrap();
    assert_eq!((w["frames"].as_u64(), w["dof"].as_u64()), (Some(401), Some(7)));
    assert!((w["duration_s"].as_f64().unwrap() - 8.0).abs() < 1e-6);

    // REST replay needs the WS lease
    let res = client.post(format!("http://{addr}/api/v1/action/replay")).json(&json!({"episode": "w"})).send().await.unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::CONFLICT);

    // rejections over WS
    for (body, want) in [(json!({"episode": "nope"}), "EPISODE_NOT_FOUND"), (json!({"episode": "w", "speed": 2}), "INVALID_COMMAND"),
                         (json!({"episode": "fast"}), "REPLAY_TOO_FAST"), (json!({"episode": "a/b"}), "INVALID_COMMAND")] {
        let r = common::rpc(&mut ws, generation, epoch, "/api/v1/action/replay", body.clone()).await;
        assert_eq!((r["type"].as_str(), r["code"].as_str()), (Some("command_reject"), Some(want)), "{body}: {r}");
    }
    let r = common::rpc(&mut ws, generation, epoch, "/api/v1/action/replay", json!({"episode": "fast"})).await;
    println!("WS reject: {}", r["message"]);

    // start, then a second start is MOTION_BUSY
    let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/action/replay", json!({"episode": "w", "speed": 1.0, "tag": "t"})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let id = ack["action_id"].as_u64().unwrap();
    assert!(ack["expected_duration_s"].as_f64().unwrap() > 8.0);
    let busy = common::rpc(&mut ws, generation, epoch, "/api/v1/action/replay", json!({"episode": "w"})).await;
    assert_eq!(busy["code"], "MOTION_BUSY", "{busy}");
    // deleting a file does not affect the running replay (it is in memory)
    let res = client.delete(format!("http://{addr}/api/v1/dataset/episodes/fast")).send().await.unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::OK);
    let res = client.delete(format!("http://{addr}/api/v1/dataset/episodes/fast")).send().await.unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::NOT_FOUND);
    let res = client.delete(format!("http://{addr}/api/v1/dataset/episodes/..%2Fx")).send().await.unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::BAD_REQUEST);
    let body: serde_json::Value = res.json().await.unwrap();
    assert_eq!(body["code"], "INVALID_EPISODE_ID");
    // cross-origin delete is refused
    let res = client.delete(format!("http://{addr}/api/v1/dataset/episodes/w")).header("origin", "http://evil.example").send().await.unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::FORBIDDEN);

    // progress in the observation
    tokio::time::sleep(Duration::from_millis(1500)).await;
    let a = supervisor.get_observation().await.action.unwrap();
    assert_eq!((a.id, a.kind.as_str(), a.tag.as_deref(), a.episode.as_deref()), (id, "replay", Some("t"), Some("w")));
    assert!(a.is_running() && a.t_s.is_some(), "{a:?}");
    let json = serde_json::to_value(&a).unwrap();
    assert!(json["t_s"].is_number() && json["phase"].is_string(), "{json}");

    // closing the lease holder's socket cancels it
    ws.close(None).await.unwrap();
    drop(ws);
    let mut a = None;
    for _ in 0..50 {
        tokio::time::sleep(Duration::from_millis(20)).await;
        let x = supervisor.get_observation().await.action.unwrap();
        if !x.is_running() { a = Some(x); break; }
    }
    let a = a.expect("cancelled after disconnect");
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("cancelled", Some("LEASE_LOST")), "{a:?}");
    let list: serde_json::Value = client.get(format!("http://{addr}/api/v1/dataset/episodes")).send().await.unwrap().json().await.unwrap();
    assert_eq!(list["episodes"].as_array().unwrap().len(), 1);
}
