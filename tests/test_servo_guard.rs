//! Servo streaming guard (docs/control_safety_contract.md "Servo streaming guard"):
//! speed cap, lead cap, blocked hold and `motion_limit`, against a fake arm that follows
//! commands with POS_VEL lag, a per-joint velocity limit, and optionally a pinned joint
//! (blocked by an obstacle, or pushed and held by hand).
use std::sync::{Arc, Mutex};
use std::time::Duration;
use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
use armd::model::{ArmConfig, JointState, MotionLimitReason, ObservationSchema};
use armd::{ArmObservation, ArmSupervisor};

const DT: f64 = 0.02;
/// Wall-clock time per simulated 20 ms tick; persistence is wall-clock, scaled by 1/4.
const WALL_PER_TICK: Duration = Duration::from_millis(5);
const SCALED_PERSISTENCE_MS: u64 = 125;
const LEAD: f32 = 0.2;
/// Tolerance on the lead invariant: one tick of stepper motion at the lead boundary.
const EPS: f32 = 0.03;
const HOME: [f32; 7] = [0.0, 0.0, 0.0, 0.0, 0.0, std::f32::consts::FRAC_PI_2, 0.0];

struct Arm {
    q: [f32; 7],
    v: [f32; 7],
    enabled: EnableState,
    /// POS_VEL response: v = clamp((cmd - q) * gain, +-vmax).
    gain: f32,
    vmax: [f32; 7],
    /// Joint held at a position by an obstacle or a hand.
    pinned: [Option<f32>; 7],
    status: Vec<u8>,
}

struct FakeDriver(Arc<Mutex<Arm>>);
impl ArmDriver for FakeDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true,
            torque_feedforward: true, enable_feedback: true }
    }
    fn enable_state(&self) -> EnableState { self.0.lock().unwrap().enabled }
    fn is_feedback_valid(&self) -> bool { true }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); 7] }
    fn max_command_velocity(&self, _: usize) -> Option<f32> { Some(2.0) }
    fn motor_status(&self) -> Option<Vec<u8>> { Some(self.0.lock().unwrap().status.clone()) }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let arm = self.0.lock().unwrap();
        Ok((0..7).map(|i| JointState { position: arm.q[i], velocity: arm.v[i], torque: 0.0, temperature: 30.0 }).collect())
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        let mut arm = self.0.lock().unwrap();
        for (i, t) in targets.iter().enumerate().take(7) {
            if let Some(p) = arm.pinned[i] { arm.q[i] = p; arm.v[i] = 0.0; continue; }
            let v = ((t.position - arm.q[i]) * arm.gain).clamp(-arm.vmax[i], arm.vmax[i]);
            arm.v[i] = v;
            arm.q[i] += v * DT as f32;
        }
        Ok(())
    }
    fn set_torque(&mut self, on: bool) -> Result<(), ArmDriverError> {
        let mut arm = self.0.lock().unwrap();
        arm.enabled = if on { EnableState::Enabled } else { EnableState::Disabled };
        if !on { arm.v = [0.0; 7]; }
        Ok(())
    }
}

/// Damiao-like lag (gain 1.5) with big joints that cannot exceed 0.3 rad/s.
fn lagging() -> Arm {
    Arm { q: HOME, v: [0.0; 7], enabled: EnableState::Disabled, gain: 1.5,
        vmax: [0.3, 0.3, 0.3, 2.0, 2.0, 2.0, 2.0], pinned: [None; 7], status: vec![1; 7] }
}

/// Stiff follower (negligible lag): the speed cap, not the lead cap, shapes the motion.
fn stiff() -> Arm { Arm { gain: 25.0, vmax: [2.0; 7], ..lagging() } }

/// B601-DM profile without the workspace box, so that whole-arm test streams from home
/// are not rejected by the Cartesian check (tested on its own in a5).
fn profile() -> ArmConfig { ArmConfig { workspace: None, ..ArmConfig::rebot_dm() } }

async fn armed(config: ArmConfig, arm: Arm) -> (ArmSupervisor, Arc<Mutex<Arm>>) {
    let mut config = config;
    config.following_error.persistence_ms = SCALED_PERSISTENCE_MS;
    let arm = Arc::new(Mutex::new(arm));
    let supervisor = ArmSupervisor::new(config).with_driver(Box::new(FakeDriver(arm.clone())));
    supervisor.set_lease_enabled(false).await;
    supervisor.tick(DT).await;
    supervisor.set_torque(true).await.unwrap();
    supervisor.tick(DT).await;
    (supervisor, arm)
}

async fn tick(supervisor: &ArmSupervisor) -> ArmObservation {
    tokio::time::sleep(WALL_PER_TICK).await;
    supervisor.tick(DT).await
}

#[derive(Default, Debug)]
struct Stats { max_lead: f32, max_lead_joint: usize, faults: usize, reasons: Vec<(MotionLimitReason, Option<usize>)>, peak_v: [f32; 7] }
impl Stats {
    fn record(&mut self, obs: &ArmObservation, cmd_v: &[f32]) {
        for i in 0..6 {
            let e = (obs.commanded_positions[i] - obs.positions[i]).abs();
            if e > self.max_lead { self.max_lead = e; self.max_lead_joint = i; }
        }
        for (p, v) in self.peak_v.iter_mut().zip(cmd_v) { *p = p.max(v.abs()); }
        if matches!(obs.safety_state, armd::SafetyState::Fault(_)) { self.faults += 1; }
        if let Some(l) = &obs.motion_limit {
            if !self.reasons.contains(&(l.reason, l.joint)) { self.reasons.push((l.reason, l.joint)); }
        }
    }
    fn has(&self, reason: MotionLimitReason) -> bool { self.reasons.iter().any(|(r, _)| *r == reason) }
}

/// Streams `target(t, commanded)` every tick for `seconds`; returns the stats.
async fn stream(supervisor: &ArmSupervisor, seconds: f64, target: impl Fn(f64, &[f32]) -> Vec<f32>) -> Stats {
    let mut stats = Stats::default();
    let mut obs = supervisor.get_observation().await;
    for k in 0..(seconds / DT) as usize {
        let t = k as f64 * DT;
        let _ = supervisor.servo(target(t, &obs.commanded_positions), obs.epoch_id).await;
        obs = tick(supervisor).await;
        let v = supervisor.inner().read().await.commanded_velocities.clone();
        stats.record(&obs, &v);
    }
    stats
}

/// Every non-gripper joint jumps 0.3 rad ahead of its command each frame (below the
/// 0.35 rad dirty-frame bound), +1 for the first half and -1 after.
fn jump_ahead(seconds: f64) -> impl Fn(f64, &[f32]) -> Vec<f32> {
    move |t, cmd| {
        let dir = if t < seconds / 2.0 { 1.0 } else { -1.0 };
        let mut out = cmd.to_vec();
        for (i, x) in out.iter_mut().enumerate().take(6) {
            let lim = ArmConfig::rebot_dm().limits[i].clone();
            *x = (*x + 0.3 * dir).clamp(lim.min_position + 0.01, lim.max_position - 0.01);
        }
        out
    }
}

#[tokio::test]
async fn a1_jump_ahead_stream_is_lead_capped_without_fault() {
    let (supervisor, _arm) = armed(profile(), lagging()).await;
    let stats = stream(&supervisor, 4.0, jump_ahead(4.0)).await;
    eprintln!("a1 jump-ahead: max |cmd-meas| {:.4} rad (joint {}), reasons {:?}", stats.max_lead, stats.max_lead_joint + 1, stats.reasons);
    assert_eq!(stats.faults, 0, "no Fault");
    assert!(stats.max_lead <= LEAD + EPS, "lead {}", stats.max_lead);
    assert!(stats.has(MotionLimitReason::Lead), "{:?}", stats.reasons);
    let obs = supervisor.get_observation().await;
    assert!(obs.safety_state.is_operational());
    // big joints moved, at the arm's own speed
    assert!((obs.positions[1] - HOME[1]).abs() > 0.05, "J2 moved {}", obs.positions[1]);
}

#[tokio::test]
async fn a2_jump_ahead_on_a_stiff_arm_is_speed_capped() {
    let (supervisor, _arm) = armed(profile(), stiff()).await;
    let stats = stream(&supervisor, 2.0, jump_ahead(4.0)).await;
    eprintln!("a2 stiff: max |cmd-meas| {:.4} rad, peak cmd v {:?}, reasons {:?}", stats.max_lead, stats.peak_v, stats.reasons);
    assert_eq!(stats.faults, 0);
    assert!(stats.max_lead <= LEAD + EPS);
    for i in 0..3 { assert!(stats.peak_v[i] <= 0.45 + 1e-3 && stats.peak_v[i] > 0.44, "J{} peak {}", i + 1, stats.peak_v[i]); }
    for i in 3..6 { assert!(stats.peak_v[i] <= 1.5 + 1e-3, "J{} peak {}", i + 1, stats.peak_v[i]); }
    assert!(stats.has(MotionLimitReason::Speed), "{:?}", stats.reasons);
    // the schema publishes the effective caps
    let schema = supervisor.get_schema().await;
    assert_eq!(schema.servo_max_velocity, vec![0.45, 0.45, 0.45, 1.5, 1.5, 1.5, 2.0]);
    assert_eq!(schema.servo_max_lead_rad, Some(LEAD));
    assert!(schema.capabilities.servo_guard);
}

#[tokio::test]
async fn a3_fast_sine_stream_never_faults() {
    let (supervisor, _arm) = armed(profile(), lagging()).await;
    // 0.8 rad at 0.6 Hz (peak 3 rad/s), each frame within 0.3 rad of the command
    let stats = stream(&supervisor, 5.0, |t, cmd| {
        let mut out = cmd.to_vec();
        let s = 0.8 * (2.0 * std::f64::consts::PI * 0.6 * t).sin() as f32;
        for i in 0..6 { out[i] = (HOME[i] + s).clamp(cmd[i] - 0.3, cmd[i] + 0.3); }
        out
    }).await;
    eprintln!("a3 sine: max |cmd-meas| {:.4} rad (joint {}), reasons {:?}", stats.max_lead, stats.max_lead_joint + 1, stats.reasons);
    assert_eq!(stats.faults, 0);
    assert!(stats.max_lead <= LEAD + EPS, "lead {}", stats.max_lead);
    assert!(stats.has(MotionLimitReason::Lead));
}

#[tokio::test]
async fn a4_targets_beyond_the_limits_report_joint_limit_without_fault() {
    let (supervisor, _arm) = armed(profile(), lagging()).await;
    let mut rejected = 0;
    let mut stats = Stats::default();
    for _ in 0..(5.0 / DT) as usize {
        let obs = supervisor.get_observation().await;
        let mut t = obs.commanded_positions.clone();
        t[1] = 2.5; // J2 limit 2.0944
        if supervisor.servo(t, obs.epoch_id).await.is_err() { rejected += 1; }
        let obs = tick(&supervisor).await;
        stats.record(&obs, &[]);
        assert_eq!(obs.motion_limit.as_ref().map(|l| (l.reason, l.joint)), Some((MotionLimitReason::JointLimit, Some(1))));
    }
    eprintln!("a4 beyond limits: {rejected} frames rejected, reasons {:?}", stats.reasons);
    assert_eq!(rejected, 250);
    assert_eq!(stats.faults, 0);
    // cleared ~300 ms after the last rejected frame
    for _ in 0..25 { tick(&supervisor).await; }
    tokio::time::sleep(Duration::from_millis(320)).await;
    assert!(tick(&supervisor).await.motion_limit.is_none());

    // a stream held at a joint limit reports joint_limit once the command arrives there
    let (supervisor, _arm) = armed(profile(), stiff()).await;
    let mut seen = false;
    for _ in 0..200 {
        let obs = supervisor.get_observation().await;
        let mut t = obs.commanded_positions.clone();
        t[3] = (t[3] + 0.3).min(profile().limits[3].max_position); // J4 limit
        supervisor.servo(t, obs.epoch_id).await.unwrap();
        let obs = tick(&supervisor).await;
        seen |= obs.motion_limit.as_ref().is_some_and(|l| l.reason == MotionLimitReason::JointLimit && l.joint == Some(3));
        assert!(obs.safety_state.is_operational());
    }
    assert!(seen, "J4 at its limit reported");
}

#[tokio::test]
async fn a5_workspace_rejection_reports_workspace() {
    let (supervisor, _arm) = armed(ArmConfig::rebot_dm(), stiff()).await;
    let obs = supervisor.get_observation().await;
    // a pose that leaves the workspace (searched, so the test does not hard-code geometry)
    let bad = {
        let inner = supervisor.inner().read().await;
        let mut found = None;
        'outer: for j2 in (-20..=20).map(|k| k as f32 * 0.1) {
            for j3 in (-26..=26).map(|k| k as f32 * 0.1) {
                let mut q = obs.commanded_positions.clone();
                q[1] = j2; q[2] = j3;
                if inner.check_workspace_path(&obs.commanded_positions, [q.as_slice()]).is_err() { found = Some(q); break 'outer; }
            }
        }
        found.expect("a pose outside the workspace")
    };
    let err = supervisor.servo(bad, obs.epoch_id).await.unwrap_err();
    assert!(format!("{err:?}").contains("Workspace"), "{err:?}");
    let obs = tick(&supervisor).await;
    let l = obs.motion_limit.expect("motion_limit");
    assert_eq!((l.reason, l.joint), (MotionLimitReason::Workspace, None));
    assert!(obs.safety_state.is_operational());
}

#[tokio::test]
async fn b_blocked_joint_holds_without_fault_and_accepts_the_next_command() {
    let (supervisor, arm) = armed(profile(), lagging()).await;
    arm.lock().unwrap().pinned[1] = Some(HOME[1]); // J2 against an obstacle
    let mut max_lead = 0.0f32;
    let mut blocked_at = None;
    for k in 0..150 {
        let obs = supervisor.get_observation().await;
        let mut t = obs.commanded_positions.clone();
        t[1] += 0.3;
        let _ = supervisor.servo(t, obs.epoch_id).await;
        let obs = tick(&supervisor).await;
        max_lead = max_lead.max((obs.commanded_positions[1] - obs.positions[1]).abs());
        assert!(obs.safety_state.is_operational(), "no Fault: {:?}", obs.safety_state);
        if obs.motion_limit.as_ref().is_some_and(|l| l.reason == MotionLimitReason::Blocked) { blocked_at = Some(k); break; }
    }
    let k = blocked_at.expect("blocked reported");
    eprintln!("b blocked: blocked after {k} ticks, max |cmd-meas| J2 {max_lead:.4} rad");
    assert!(max_lead <= LEAD + EPS);
    {
        let inner = supervisor.inner().read().await;
        assert!(inner.servo_target.is_none(), "stream stopped");
        assert!((inner.commanded_positions[1] - inner.measured_positions[1]).abs() < 1e-6, "holding the measured pose");
        assert!(inner.is_torqued);
    }
    // the hold keeps reporting blocked
    for _ in 0..40 {
        let obs = tick(&supervisor).await;
        let l = obs.motion_limit.expect("blocked persists");
        assert_eq!((l.reason, l.joint), (MotionLimitReason::Blocked, Some(1)));
        assert!(obs.safety_state.is_operational());
    }
    // obstacle gone: the next command is accepted normally
    arm.lock().unwrap().pinned[1] = None;
    let obs = supervisor.get_observation().await;
    let mut t = obs.commanded_positions.clone();
    t[1] += 0.1;
    supervisor.servo(t.clone(), obs.epoch_id).await.expect("next command accepted");
    for _ in 0..100 { tick(&supervisor).await; }
    let obs = supervisor.get_observation().await;
    assert!((obs.positions[1] - t[1]).abs() < 0.02, "J2 reached {} (target {})", obs.positions[1], t[1]);
    assert!(obs.motion_limit.as_ref().is_none_or(|l| l.reason != MotionLimitReason::Blocked));
}

#[tokio::test]
async fn b2_joint_pushed_back_while_streaming_holds_without_fault() {
    let (supervisor, arm) = armed(profile(), lagging()).await;
    let _ = stream(&supervisor, 0.6, jump_ahead(100.0)).await;
    // J2 is shoved 0.6 rad against the direction of travel and held there
    let q = arm.lock().unwrap().q[1];
    arm.lock().unwrap().pinned[1] = Some(q - 0.6);
    let stats = stream(&supervisor, 1.5, jump_ahead(100.0)).await;
    eprintln!("b2 pushed while streaming: reasons {:?}", stats.reasons);
    assert_eq!(stats.faults, 0);
    assert!(stats.has(MotionLimitReason::Blocked) || stats.has(MotionLimitReason::Lead), "{:?}", stats.reasons);
}

#[tokio::test]
async fn c_pushed_while_holding_latches_the_fault() {
    // plain hold, no stream
    let (supervisor, arm) = armed(profile(), lagging()).await;
    arm.lock().unwrap().pinned[1] = Some(HOME[1] + 0.6);
    let mut fault = None;
    for _ in 0..100 {
        let obs = tick(&supervisor).await;
        if let armd::SafetyState::Fault(r) = obs.safety_state { fault = Some(format!("{r:?}")); break; }
    }
    let fault = fault.expect("Fault latched while holding");
    assert!(fault.contains("Persistent joint tracking error: joint 2"), "{fault}");

    // after a blocked hold: pushed in the hold -> Fault as before
    let (supervisor, arm) = armed(profile(), lagging()).await;
    arm.lock().unwrap().pinned[1] = Some(HOME[1]);
    for _ in 0..150 {
        let obs = supervisor.get_observation().await;
        let mut t = obs.commanded_positions.clone();
        t[1] += 0.3;
        let _ = supervisor.servo(t, obs.epoch_id).await;
        if tick(&supervisor).await.motion_limit.is_some_and(|l| l.reason == MotionLimitReason::Blocked) { break; }
    }
    assert!(supervisor.inner().read().await.servo_blocked_hold);
    arm.lock().unwrap().pinned[1] = Some(HOME[1] + 0.6);
    let mut faulted = false;
    for _ in 0..100 {
        if matches!(tick(&supervisor).await.safety_state, armd::SafetyState::Fault(_)) { faulted = true; break; }
    }
    assert!(faulted, "pushed during the blocked hold latches the Fault");
}

#[tokio::test]
async fn e_profile_without_servo_cap_keeps_joint_limit_speed_and_the_lead_cap() {
    let mut config = profile();
    config.servo_max_velocity = None;
    let (supervisor, _arm) = armed(config.clone(), stiff()).await;
    let stats = stream(&supervisor, 1.0, jump_ahead(4.0)).await;
    eprintln!("e no servo cap (stiff): peak cmd v {:?}", stats.peak_v);
    assert!(stats.peak_v[1] > 1.0, "J2 runs at the joint limit speed: {}", stats.peak_v[1]);
    assert!(!stats.has(MotionLimitReason::Speed));
    let (supervisor, _arm) = armed(config, lagging()).await;
    let stats = stream(&supervisor, 3.0, jump_ahead(3.0)).await;
    eprintln!("e no servo cap (lagging): max |cmd-meas| {:.4} rad, reasons {:?}", stats.max_lead, stats.reasons);
    assert_eq!(stats.faults, 0);
    assert!(stats.max_lead <= LEAD + EPS);
    assert!(stats.has(MotionLimitReason::Lead));
    let schema = supervisor.get_schema().await;
    assert_eq!(schema.servo_max_velocity[1], 2.0);
}

#[tokio::test]
async fn motor_error_codes_are_published_per_joint() {
    let (supervisor, arm) = armed(profile(), lagging()).await;
    let obs = tick(&supervisor).await;
    assert_eq!(obs.motor_errors, vec![None; 7]);
    arm.lock().unwrap().status[2] = 0xA;
    let obs = tick(&supervisor).await;
    let e = obs.motor_errors[2].clone().expect("J3 error");
    assert_eq!((e.code, e.name.as_str()), (0xA, "Overcurrent"));
    assert!(obs.motor_errors.iter().enumerate().all(|(i, e)| i == 2 || e.is_none()));
    let json = serde_json::to_value(&obs).unwrap();
    assert_eq!(json["motor_errors"][2]["name"], "Overcurrent");
}

#[test]
fn config_validation_and_observation_shape() {
    for bad in [0.0, -0.1, 0.5, 0.6, f32::NAN] {
        let mut c = ArmConfig::rebot_dm();
        c.servo_max_lead_rad = bad;
        assert!(c.validate().is_err(), "lead {bad}");
    }
    let mut c = ArmConfig::rebot_dm();
    c.servo_max_velocity = Some(vec![Some(0.45); 6]);
    assert!(c.validate().is_err(), "length must match dof");
    c.servo_max_velocity = Some(vec![Some(0.0); 7]);
    assert!(c.validate().is_err());
    assert_eq!(ArmConfig::default().servo_max_velocity, None);
    assert_eq!(ArmConfig::default().servo_max_lead_rad, LEAD);
    // a profile without the new fields still parses (lead default 0.2)
    let mut legacy = serde_json::to_value(ArmConfig::rebot_dm()).unwrap();
    legacy.as_object_mut().unwrap().remove("servo_max_velocity");
    legacy.as_object_mut().unwrap().remove("servo_max_lead_rad");
    let parsed: ArmConfig = serde_json::from_value(legacy).unwrap();
    assert_eq!((parsed.servo_max_velocity, parsed.servo_max_lead_rad), (None, LEAD));
    // motion_limit: null when idle; older payloads without it parse
    let json = serde_json::to_value(armd::SupervisorInner::new(ArmConfig::rebot_dm()).current_observation()).unwrap();
    assert!(json["motion_limit"].is_null() && json.as_object().unwrap().contains_key("motion_limit"));
    let mut old = json.clone();
    old.as_object_mut().unwrap().remove("motion_limit");
    old.as_object_mut().unwrap().remove("motor_errors");
    let _: ArmObservation = serde_json::from_value(old).unwrap();
    let l = armd::model::MotionLimit { reason: MotionLimitReason::JointLimit, joint: Some(2), since_ms: 1500 };
    assert_eq!(serde_json::to_value(l).unwrap(), serde_json::json!({"reason": "joint_limit", "joint": 2, "since_ms": 1500}));
    let _ = ObservationSchema::from_config(&ArmConfig::default());
}
