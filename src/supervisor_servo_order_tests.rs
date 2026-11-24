//! Servo filter order across merge 6967c0a (feat/policy-runtime into rc/1.4.1-rc12).
//!
//! The merge split `servo()` into `validate_and_filter_servo_targets()` plus the store; the
//! only statement that moved is `clear_blocked_hold()`, from before the torque / power hold
//! filters to after them. The per-tick stepper (limit clamp, gripper stall freeze, lead cap,
//! jerk-limited speed step, workspace step check, gripper lead cap) is unchanged.
//!
//! `servo_pre_6967c0a` is the intake of the rc parent (2e1e836 src/supervisor.rs:2498-2571),
//! kept here to compare. The tests feed both intakes the same streams on twin supervisors and
//! require identical targets, commands and motion limits, and check the safety invariants of
//! the current order on every tick.
use super::*;
use crate::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
use crate::model::{ArmConfig, JointState, MotionLimitReason};
use std::sync::{Arc, Mutex};

const DT: f64 = 0.02;
const INSPECT: [f32; 7] = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0, -5.7];

impl SupervisorInner {
    /// Servo intake as of 2e1e836 (before 6967c0a), statement for statement.
    fn servo_pre_6967c0a(&mut self, mut targets: Vec<f32>, epoch_id: u64) -> Result<(), SafetyError> {
        self.gravity_comp_servo_busy()?;
        self.collision_hold_check()?;
        if self.handover_settling_since.is_some() {
            return Err(SafetyError::HandoverSettling);
        }
        if self.estop_pending() {
            return Err(SafetyError::EStopActive);
        }
        if epoch_id != self.safety.epoch_id() {
            return Err(SafetyError::EpochMismatch { expected: self.safety.epoch_id(), received: epoch_id });
        }
        if self.pending_rearm || self.pending_normal_unload {
            return Err(SafetyError::NotArmed(self.safety.state().clone()));
        }
        if self.active_trajectory.is_some() || self.macro_run.is_some() {
            return Err(SafetyError::MotionBusy);
        }
        if let Err(e) = self.safety.validate_motion_command(Some(epoch_id), &targets, &self.config.limits) {
            if let SafetyError::SoftLimitExceeded { joint, .. } = &e {
                let idx = self.config.limits.iter().position(|l| &l.name == joint);
                self.note_motion_limit(MotionLimitReason::JointLimit, idx);
            }
            return Err(e);
        }
        if let Err(e) = self.check_workspace_path(&self.commanded_positions, [targets.as_slice()]) {
            let e = e.with_workspace_phase("servo_target");
            self.workspace_violation = e.workspace_detail();
            self.note_motion_limit(MotionLimitReason::Workspace, None);
            return Err(e);
        }
        if self.servo_blocked_hold { self.clear_blocked_hold(); }
        self.torque_hold_filter(&mut targets);
        self.power_hold_filter(&mut targets);
        self.last_heartbeat = Instant::now();
        self.last_motion_frame = Instant::now();
        if let Some(ref mut l) = self.active_lease {
            l.last_valid_motion_instant = Some(Instant::now());
        }
        self.release_gripper_stall_on_reverse(&targets);
        let _ = self.safety.mark_motion_started();
        self.target_positions = targets.clone();
        self.servo_target = Some(targets);
        self.servo_settled_ticks.fill(0);
        Ok(())
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum Order { Old, New }

/// POS_VEL-like follower. Joint `pinned` stays where it is (obstacle); the gripper meets a
/// rigid block at `block` with torque `stiffness * (command - block)`.
struct Rig {
    q: [f32; 7], v: [f32; 7], tau: [f32; 7], enabled: EnableState,
    gain: f32, vmax: f32, pinned: Option<usize>, block: Option<(f32, f32)>,
    age: Duration, peak_tau: f32, tau_log: Vec<f32>,
}
struct RigDriver(Arc<Mutex<Rig>>);

impl ArmDriver for RigDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true,
            torque_feedforward: false, enable_feedback: true, load_feedback: false }
    }
    fn enable_state(&self) -> EnableState { self.0.lock().unwrap().enabled }
    fn is_feedback_valid(&self) -> bool { true }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(self.0.lock().unwrap().age); 7] }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let r = self.0.lock().unwrap();
        Ok((0..7).map(|i| JointState { position: r.q[i], velocity: r.v[i], torque: r.tau[i], temperature: 30.0 }).collect())
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        let mut r = self.0.lock().unwrap();
        for (i, t) in targets.iter().enumerate().take(7) {
            if r.pinned == Some(i) { r.v[i] = 0.0; continue; }
            let v = ((t.position - r.q[i]) * r.gain).clamp(-r.vmax, r.vmax);
            let mut q = r.q[i] + v * DT as f32;
            let mut v = v;
            if i == 6 {
                if let Some((block, stiffness)) = r.block {
                    if q >= block {
                        q = block;
                        v = 0.0;
                        r.tau[6] = (stiffness * (t.position - block)).max(0.0);
                    } else {
                        r.tau[6] = 0.0;
                    }
                    r.peak_tau = r.peak_tau.max(r.tau[6]);
                    let tau = r.tau[6];
                    r.tau_log.push(tau);
                }
            }
            r.q[i] = q;
            r.v[i] = v;
        }
        Ok(())
    }
    fn set_torque(&mut self, on: bool) -> Result<(), ArmDriverError> {
        let mut r = self.0.lock().unwrap();
        r.enabled = if on { EnableState::Enabled } else { EnableState::Disabled };
        Ok(())
    }
}

fn rig_default() -> Rig {
    Rig { q: INSPECT, v: [0.0; 7], tau: [0.0; 7], enabled: EnableState::Disabled, gain: 4.5, vmax: 1.5,
        pinned: None, block: None, age: Duration::ZERO, peak_tau: 0.0, tau_log: Vec::new() }
}

fn armed(config: ArmConfig, rig: Rig) -> (SupervisorInner, Arc<Mutex<Rig>>) {
    let rig = Arc::new(Mutex::new(rig));
    let mut inner = SupervisorInner::new(config).with_driver(Box::new(RigDriver(rig.clone())));
    inner.lease_enabled = false;
    inner.tick(DT);
    inner.set_torque(true).unwrap();
    inner.tick(DT);
    (inner, rig)
}

fn send(inner: &mut SupervisorInner, order: Order, targets: Vec<f32>) -> Result<(), String> {
    let epoch = inner.safety.epoch_id();
    let r = match order {
        Order::Old => inner.servo_pre_6967c0a(targets, epoch),
        Order::New => inner.servo(targets, epoch),
    };
    r.map_err(|e| format!("{e:?}").split(['(', ' ', '{']).next().unwrap_or_default().to_string())
}

/// Everything the filter order can influence.
#[derive(Debug, PartialEq, Clone)]
struct Snap {
    accepted: Option<Result<(), String>>,
    servo_target: Option<Vec<f32>>,
    target: Vec<f32>,
    cmd: Vec<f32>,
    vel: Vec<f32>,
    limit: Option<(MotionLimitReason, Option<usize>)>,
    blocked: bool,
    stalled: bool,
    workspace_violation: bool,
}

fn snap(inner: &SupervisorInner, accepted: Option<Result<(), String>>) -> Snap {
    Snap {
        accepted,
        servo_target: inner.servo_target.clone(),
        target: inner.target_positions.clone(),
        cmd: inner.commanded_positions.clone(),
        vel: inner.commanded_velocities.clone(),
        limit: inner.motion_limit.as_ref().map(|l| (l.reason, l.joint)),
        blocked: inner.servo_blocked_hold,
        stalled: inner.gripper_stalled,
        workspace_violation: inner.workspace_violation.is_some(),
    }
}

fn max_abs_diff(a: &[f32], b: &[f32]) -> f32 {
    a.iter().zip(b).map(|(x, y)| (x - y).abs()).fold(0.0, f32::max)
}

/// Per-tick safety invariants of the current order; returns the worst lead excess (rad)
/// beyond `servo_max_lead_rad` seen on an arm joint moving away from its measured position.
#[derive(Default, Debug)]
struct Worst { lead_excess: f32, speed_excess: f32, step_excess: f32, gripper_lead_excess: f32, ticks: usize }

fn check_tick(inner: &SupervisorInner, prev_cmd: &[f32], dt: f64, held_this_tick: bool, w: &mut Worst) {
    let servo_dt = dt.clamp(0.001, 0.1) as f32;
    let limits = inner.servo_limits();
    let g = inner.config.gripper_index();
    let fresh = inner.hardware_feedback_fresh(Duration::from_millis(100));
    w.ticks += 1;
    for i in 0..inner.config.dof {
        let raw = &inner.config.limits[i];
        let (q, v, m) = (inner.commanded_positions[i], inner.commanded_velocities[i], inner.measured_positions[i]);
        assert!(q.is_finite() && v.is_finite(), "joint {i}: q {q} v {v}");
        assert!(q >= raw.min_position - 1e-4 && q <= raw.max_position + 1e-4,
            "joint {i}: command {q} outside [{}, {}]", raw.min_position, raw.max_position);
        w.speed_excess = w.speed_excess.max(v.abs() - limits[i].max_velocity);
        if Some(i) == g {
            // closing direction only (rebot_dm closes towards +)
            if fresh { w.gripper_lead_excess = w.gripper_lead_excess.max(q - m - inner.config.gripper_policy.as_ref().and_then(|p| p.max_lead_rad).unwrap()); }
            // the gripper lead cap may pull the command back towards the jaw; closing steps are bounded
            if !held_this_tick { w.step_excess = w.step_excess.max((q - prev_cmd[i]) - limits[i].max_velocity * servo_dt); }
            continue;
        }
        if !held_this_tick { w.step_excess = w.step_excess.max((q - prev_cmd[i]).abs() - limits[i].max_velocity * servo_dt); }
        if fresh && inner.is_torqued && v.abs() > 1e-6 && m.is_finite() {
            w.lead_excess = w.lead_excess.max((q - m) * v.signum() - inner.config.servo_max_lead_rad);
        }
    }
}

/// One stream step: what the client sends (None = nothing this tick) and the tick dt.
struct Step { target: Option<Vec<f32>>, dt: f64, stale_frame: bool }

/// Runs `stream` on twin supervisors (old and new intake); asserts identical snapshots and
/// returns the max command difference (0 expected) and the invariant record of the new order.
fn differential(config: &ArmConfig, rig: impl Fn() -> Rig, stream: &[Step]) -> (f32, Worst, Arc<Mutex<Rig>>) {
    let (mut old, _rig_old) = armed(config.clone(), rig());
    let (mut new, rig_new) = armed(config.clone(), rig());
    let mut max_diff = 0.0f32;
    let mut w = Worst::default();
    for (k, s) in stream.iter().enumerate() {
        let (mut a_old, mut a_new) = (None, None);
        if let Some(t) = &s.target {
            a_old = Some(send(&mut old, Order::Old, t.clone()));
            a_new = Some(send(&mut new, Order::New, t.clone()));
        }
        let (so, sn) = (snap(&old, a_old), snap(&new, a_new));
        assert_eq!(so, sn, "step {k}: intake differs");
        if s.stale_frame {
            for x in [&mut old, &mut new] { x.last_motion_frame = Instant::now() - Duration::from_secs(1); }
        }
        let prev_cmd = new.commanded_positions.clone();
        let was_blocked = new.servo_blocked_hold;
        old.tick(s.dt);
        new.tick(s.dt);
        let (so, sn) = (snap(&old, None), snap(&new, None));
        max_diff = max_diff.max(max_abs_diff(&so.cmd, &sn.cmd)).max(max_abs_diff(&so.vel, &sn.vel));
        assert_eq!(so, sn, "step {k}: tick differs");
        let held = new.servo_blocked_hold != was_blocked || new.motion_limit.as_ref().is_some_and(|l| l.reason == MotionLimitReason::Workspace);
        check_tick(&new, &prev_cmd, s.dt, held, &mut w);
    }
    (max_diff, w, rig_new)
}

/// xorshift64*: deterministic randomness without a dev-dependency.
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f32 {
        self.0 ^= self.0 >> 12; self.0 ^= self.0 << 25; self.0 ^= self.0 >> 27;
        ((self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 40) as f32) / (1u64 << 24) as f32
    }
    fn range(&mut self, lo: f32, hi: f32) -> f32 { lo + (hi - lo) * self.next() }
}

fn no_ws() -> ArmConfig { ArmConfig { workspace: None, ..ArmConfig::rebot_dm() } }

fn report(name: &str, d: f32, w: &Worst) {
    eprintln!("{name}: ticks {}, max |old-new| {d:.3e} rad, lead excess {:.4}, speed excess {:.2e}, step excess {:.2e}, gripper lead excess {:.4}",
        w.ticks, w.lead_excess, w.speed_excess, w.step_excess, w.gripper_lead_excess);
}

/// Bound on the lead excess: one tick of travel at the servo speed cap (the stepper brakes
/// towards meas + lead and may pass it by one step), as in tests/test_servo_guard.rs EPS.
fn lead_tol(config: &ArmConfig) -> f32 {
    config.servo_limits().iter().take(6).map(|l| l.max_velocity).fold(0.0, f32::max) * 0.1
}

fn assert_invariants(config: &ArmConfig, w: &Worst) {
    assert!(w.speed_excess <= 1e-3, "speed excess {}", w.speed_excess);
    assert!(w.step_excess <= 1e-3, "step excess {}", w.step_excess);
    assert!(w.lead_excess <= lead_tol(config), "lead excess {} > {}", w.lead_excess, lead_tol(config));
    assert!(w.gripper_lead_excess <= 1e-4, "gripper lead excess {}", w.gripper_lead_excess);
}

/// The reordered pair itself: `clear_blocked_hold` vs the torque / power hold filters, on
/// random hold states, targets and motion limits.
#[test]
fn blocked_hold_clear_commutes_with_hold_filters() {
    let config = no_ws();
    let mut rng = Rng(0x6967_c0a0_2e1e_8360);
    let reasons = [None, Some(MotionLimitReason::Blocked), Some(MotionLimitReason::Speed), Some(MotionLimitReason::Lead),
        Some(MotionLimitReason::JointLimit), Some(MotionLimitReason::Power), Some(MotionLimitReason::Torque), Some(MotionLimitReason::GripperForce)];
    let (mut clamped_torque, mut clamped_power, mut was_blocked, mut conflicting) = (0, 0, 0, 0);
    for case in 0..2000 {
        let mut twins = [armed(config.clone(), rig_default()), armed(config.clone(), rig_default())];
        let hold_joint = (rng.next() * 6.0) as usize % 6;
        let torque_hold = (rng.next() < 0.6).then(|| crate::torque_guard::TorqueHold {
            joint: hold_joint, hold_q: INSPECT[hold_joint] + rng.range(-0.1, 0.1), sign: if rng.next() < 0.5 { 1.0 } else { -1.0 },
            torque_nm: 20.0, threshold_nm: 16.8, since_s: 0.0, anchor_q: INSPECT[hold_joint], push_in: false,
            retreat: Default::default(),
        });
        let power_hold = (rng.next() < 0.5).then(|| {
            let mut push = [0.0f32; crate::power::MAX_JOINTS];
            let mut hold_q = [0.0f32; crate::power::MAX_JOINTS];
            for j in 0..6 {
                push[j] = [0.0, 1.0, -1.0][(rng.next() * 3.0) as usize % 3];
                hold_q[j] = INSPECT[j] + rng.range(-0.1, 0.1);
            }
            crate::power::PowerHold { push, hold_q, bus_a: 5.0, since_s: 0.0 }
        });
        let blocked = rng.next() < 0.7;
        let reason = reasons[(rng.next() * reasons.len() as f32) as usize % reasons.len()];
        let mut target = INSPECT.to_vec();
        for t in target.iter_mut().take(6) { *t += rng.range(-0.2, 0.2); }
        let mut out = Vec::new();
        for (k, (inner, _)) in twins.iter_mut().enumerate() {
            inner.torque_guard.hold = torque_hold;
            inner.power.gov.hold = power_hold;
            inner.servo_blocked_hold = blocked;
            inner.motion_limit = reason.map(|r| crate::model::MotionLimit { reason: r, joint: Some(hold_joint), since_ms: 0 });
            inner.motion_limit_seen_s = inner.now_s();
            let a = send(inner, if k == 0 { Order::Old } else { Order::New }, target.clone());
            out.push(snap(inner, Some(a)));
        }
        assert_eq!(out[0], out[1], "case {case}: torque {torque_hold:?} power {power_hold:?} blocked {blocked} limit {reason:?}");
        if out[1].target[hold_joint] != target[hold_joint] {
            if torque_hold.is_some() { clamped_torque += 1 } else { clamped_power += 1 }
        }
        if blocked { was_blocked += 1; }
        // the hold filters bound the target in the push direction. A power hold that pushes the
        // torque-held joint the other way runs second and can lift the target past the torque
        // hold (same in both orders; counted, see the report of this branch).
        if let Some(h) = torque_hold {
            let opposed = power_hold.is_some_and(|p| p.push[h.joint] == -h.sign
                && (out[1].target[h.joint] - h.hold_q) * h.sign > 1e-6);
            if opposed { conflicting += 1; } else { assert!((out[1].target[h.joint] - h.hold_q) * h.sign <= 1e-6); }
        }
        if let Some(h) = power_hold {
            for j in 0..6 {
                if h.push[j] != 0.0 && !(torque_hold.is_some_and(|t| t.joint == j)) {
                    assert!((out[1].target[j] - h.hold_q[j]) * h.push[j] <= 1e-6, "power hold joint {j}");
                }
            }
        }
    }
    eprintln!("pair: 2000 cases, {was_blocked} in blocked hold, {clamped_torque} torque-clamped, {clamped_power} power-clamped, all identical; {conflicting} with opposed torque/power holds on one joint");
    assert!(clamped_torque > 100 && clamped_power > 50 && was_blocked > 1000);
}

/// Random and large jumps (inside and beyond the 0.35 rad dirty-frame bound), with targets
/// past the joint limits mixed in.
#[test]
fn random_jumps_and_out_of_limit_targets() {
    let config = no_ws();
    let mut rng = Rng(0x0bad_5eed);
    let lim = config.limits.clone();
    let mut stream = Vec::new();
    let mut base = INSPECT.to_vec();
    for k in 0..1500 {
        let mut t = base.clone();
        let mode = rng.next();
        for (i, x) in t.iter_mut().enumerate() {
            let span = lim[i].max_position - lim[i].min_position;
            *x = if mode < 0.15 {
                // anywhere in the limits: mostly dirty jumps
                rng.range(lim[i].min_position, lim[i].max_position)
            } else if mode < 0.25 {
                // past a limit: refused
                if rng.next() < 0.5 { lim[i].max_position + rng.range(0.01, 0.5) } else { lim[i].min_position - rng.range(0.01, 0.5) }
            } else if mode < 0.30 {
                lim[i].max_position.min(*x + span)
            } else {
                (*x + rng.range(-0.3, 0.3)).clamp(lim[i].min_position, lim[i].max_position)
            };
        }
        if mode >= 0.30 { base = t.clone(); }
        stream.push(Step { target: (k % 7 != 6).then_some(t), dt: DT, stale_frame: false });
    }
    let (d, w, _) = differential(&config, rig_default, &stream);
    report("random jumps", d, &w);
    assert_eq!(d, 0.0);
    assert_invariants(&config, &w);
}

/// A pinned joint (lead cap, then blocked hold, then resume by a new target) while a torque
/// hold is in force: the path through the reordered `clear_blocked_hold`.
#[test]
fn blocked_joint_with_torque_hold_resumes_identically() {
    let mut config = no_ws();
    config.following_error.persistence_ms = 1; // wall clock: each loop below sleeps 3 ms
    let rig = || Rig { pinned: Some(1), ..rig_default() };
    let mut stream = Vec::new();
    for k in 0..300 {
        let mut t = INSPECT.to_vec();
        t[1] += (k as f32 * 0.01).min(0.3); // J2 pushed into the obstacle
        t[0] += (k as f32 * 0.005).min(0.3);
        stream.push(Step { target: Some(t), dt: DT, stale_frame: false });
    }
    let (mut old, _) = armed(config.clone(), rig());
    let (mut new, _) = armed(config.clone(), rig());
    let hold = crate::torque_guard::TorqueHold { joint: 0, hold_q: INSPECT[0] + 0.05, sign: 1.0, torque_nm: 20.0,
        threshold_nm: 16.8, since_s: 0.0, anchor_q: INSPECT[0], push_in: false, retreat: Default::default() };
    let (mut entered, mut resumed, mut max_diff) = (0, 0, 0.0f32);
    let mut w = Worst::default();
    for (k, s) in stream.iter().enumerate() {
        std::thread::sleep(Duration::from_millis(3));
        for x in [&mut old, &mut new] { x.torque_guard.hold = Some(hold); }
        let was = new.servo_blocked_hold;
        let a_old = send(&mut old, Order::Old, s.target.clone().unwrap());
        let a_new = send(&mut new, Order::New, s.target.clone().unwrap());
        assert_eq!(snap(&old, Some(a_old)), snap(&new, Some(a_new)), "step {k}");
        if was && !new.servo_blocked_hold { resumed += 1; }
        let prev = new.commanded_positions.clone();
        let before = new.servo_blocked_hold;
        old.tick(DT);
        new.tick(DT);
        if !before && new.servo_blocked_hold { entered += 1; }
        max_diff = max_diff.max(max_abs_diff(&old.commanded_positions, &new.commanded_positions));
        assert_eq!(snap(&old, None), snap(&new, None), "tick {k}");
        assert!(new.commanded_positions[0] <= hold.hold_q + 1e-5, "torque hold passed: {}", new.commanded_positions[0]);
        check_tick(&new, &prev, DT, before != new.servo_blocked_hold, &mut w);
    }
    report("blocked + torque hold", max_diff, &w);
    eprintln!("blocked holds entered {entered}, resumed {resumed}, final limit {:?}", new.motion_limit.as_ref().map(|l| l.reason));
    assert!(entered >= 1 && resumed >= 1, "entered {entered} resumed {resumed}");
    assert_eq!(max_diff, 0.0);
    // the torque guard backs a held joint off by TORQUE_HOLD_BACKOFF_RAD in one tick
    assert!(w.speed_excess <= 1e-3 && w.step_excess <= crate::torque_guard::TORQUE_HOLD_BACKOFF_RAD + 1e-3, "{w:?}");
}

/// Gripper closed onto a rigid block by the stream: force cap latch and gripper lead cap.
#[test]
fn gripper_close_while_blocked() {
    for (stiffness, cap_on) in [(3.0f32, true), (10.0, true), (0.2, false)] {
        let mut config = no_ws();
        if !cap_on { if let Some(p) = config.gripper_policy.as_mut() { p.max_close_torque_nm = None; } }
        let rig = move || Rig { block: Some((-2.0, stiffness)), ..rig_default() };
        let mut stream = Vec::new();
        let mut g = INSPECT[6];
        for k in 0..400 {
            g = (g + 0.05).min(0.0);
            let mut t = INSPECT.to_vec();
            t[6] = if k > 300 { INSPECT[6] } else { g }; // then open again (reverse releases the stall)
            stream.push(Step { target: Some(t), dt: DT, stale_frame: false });
        }
        let (d, w, rig_new) = differential(&config, rig, &stream);
        report(&format!("gripper block k={stiffness} cap={cap_on}"), d, &w);
        assert_eq!(d, 0.0);
        assert_invariants(&config, &w);
        let r = rig_new.lock().unwrap();
        let lead = config.gripper_policy.as_ref().unwrap().max_lead_rad.unwrap();
        // transient bound: the gripper lead cap limits the squeeze to stiffness x max_lead_rad
        assert!(r.peak_tau <= stiffness * lead + 1e-3, "peak {} > {}", r.peak_tau, stiffness * lead);
        if cap_on {
            // after the impact, while the stream keeps asking for closed: the stream contact
            // regulator holds the grip near contact_torque_nm (no re-squeeze cycle). Before
            // f292b07 the torque-cap latch released after 6 low-torque ticks and the stream
            // squeezed again (k = 10: 1.34 N·m every 19 ticks).
            let cap = config.gripper_policy.as_ref().unwrap().max_close_torque_nm.unwrap();
            let n = r.tau_log.len();
            let held = r.tau_log[n.saturating_sub(250)..n.saturating_sub(100)].iter().copied().fold(0.0, f32::max);
            let low = r.tau_log[n.saturating_sub(250)..n.saturating_sub(100)].iter().copied().fold(f32::MAX, f32::min);
            eprintln!("  peak {:.3} N·m (bound {:.3}), held {low:.3}..{held:.3} N·m, cap {cap}", r.peak_tau, stiffness * lead);
            if r.peak_tau > cap {
                // the impact sample was over the cap: the first over-cap sample latches
                // (GRIPPER_FORCE_CAP_TICKS 1) at the measured jaw, then the grip ramps back in
                // to about 0.6 x cap and holds there
                // the ramp takes about 1 s (trip near tick 140): check the second half of the hold
                let late = &r.tau_log[n.saturating_sub(180)..n.saturating_sub(100)];
                let (lo, hi) = (late.iter().copied().fold(f32::MAX, f32::min), late.iter().copied().fold(0.0, f32::max));
                assert!(hi <= cap && lo >= 0.55 && hi <= 0.65, "held at {lo}..{hi} N·m");
            } else {
                assert!(held <= 1.2 * cap && low >= 0.3, "held at {low}..{held} N·m");
            }
        }
    }
}

/// Zero dt (clamped to 1 ms), long dt (clamped to 100 ms), dropped frames (motion-frame
/// timeout: brake), and stale feedback (lead gate off: speed cap only).
#[test]
fn zero_long_dt_stale_frames_and_feedback() {
    let config = no_ws();
    let mut rng = Rng(42);
    let mut stream = Vec::new();
    let mut t = INSPECT.to_vec();
    for k in 0..800 {
        for x in t.iter_mut().take(6) { *x = (*x + rng.range(-0.04, 0.05)).clamp(-1.4, 1.4); }
        let dt = match k % 10 { 0 => 0.0, 1 => 1.0, 2 => 0.0005, _ => DT };
        stream.push(Step { target: (k % 13 != 0).then(|| t.clone()), dt, stale_frame: k % 29 == 0 });
    }
    let (d, w, _) = differential(&config, rig_default, &stream);
    report("dt / stale frames", d, &w);
    assert_eq!(d, 0.0);
    assert_invariants(&config, &w);
    let stale = || Rig { age: Duration::from_millis(150), gain: 1.0, ..rig_default() };
    let (d, w, _) = differential(&config, stale, &stream);
    report("stale feedback", d, &w);
    assert_eq!(d, 0.0);
    assert!(w.speed_excess <= 1e-3 && w.step_excess <= 1e-3, "{w:?}");
}

/// Near the Cartesian workspace boundary (rebot_dm box): stream outward until refused;
/// commands that start inside stay inside.
#[test]
fn near_workspace_boundary() {
    let config = ArmConfig::rebot_dm();
    let (probe, _) = armed(config.clone(), rig_default());
    assert!(probe.check_workspace_path(&INSPECT, [INSPECT.as_slice()]).is_ok(), "start inside");
    let mut rng = Rng(7);
    let mut stream = Vec::new();
    for j in [1usize, 2, 3] {
        for dir in [1.0f32, -1.0] {
            let mut t = INSPECT.to_vec();
            for _ in 0..120 {
                t[j] += dir * 0.02;
                t[0] += rng.range(-0.01, 0.01);
                stream.push(Step { target: Some(t.clone()), dt: DT, stale_frame: false });
            }
            for _ in 0..80 { stream.push(Step { target: Some(INSPECT.to_vec()), dt: DT, stale_frame: false }); }
        }
    }
    let (mut old, _) = armed(config.clone(), rig_default());
    let (mut new, _) = armed(config.clone(), rig_default());
    let (mut refused, mut holds, mut max_diff) = (0, 0, 0.0f32);
    let mut w = Worst::default();
    for (k, s) in stream.iter().enumerate() {
        let t = s.target.clone().unwrap();
        let (a_old, a_new) = (send(&mut old, Order::Old, t.clone()), send(&mut new, Order::New, t));
        if a_new.as_ref().is_err_and(|e| e == "WorkspaceViolation") { refused += 1; }
        assert_eq!(snap(&old, Some(a_old)), snap(&new, Some(a_new)), "step {k}");
        let prev = new.commanded_positions.clone();
        let inside_before = new.check_workspace_path(&prev, [prev.as_slice()]).is_ok();
        old.tick(DT);
        new.tick(DT);
        max_diff = max_diff.max(max_abs_diff(&old.commanded_positions, &new.commanded_positions));
        assert_eq!(snap(&old, None), snap(&new, None), "tick {k}");
        let held = new.motion_limit.as_ref().is_some_and(|l| l.reason == MotionLimitReason::Workspace);
        if held { holds += 1; }
        if inside_before {
            let c = new.commanded_positions.clone();
            assert!(new.check_workspace_path(&c, [c.as_slice()]).is_ok(), "tick {k}: command left the workspace");
        }
        check_tick(&new, &prev, DT, held, &mut w);
    }
    report("workspace boundary", max_diff, &w);
    eprintln!("  refused {refused} targets, {holds} ticks with a workspace limit");
    assert!(refused > 0, "the stream never reached the boundary");
    assert_eq!(max_diff, 0.0);
    assert_invariants(&config, &w);
}

/// Policy-like stream at 30 fps against a 100 Hz control loop: the servo intake (old vs
/// new), and the live policy path (new only) checked for the same invariants.
#[test]
fn policy_stream_30fps() {
    const LOOP_DT: f64 = 0.01;
    let config = no_ws();
    let traj = |t: f64| -> Vec<f32> {
        let mut q = INSPECT.to_vec();
        for (j, x) in q.iter_mut().enumerate().take(6) {
            *x += 0.25 * ((t * (0.7 + 0.3 * j as f64)).sin() as f32) * if j == 0 { 2.0 } else { 1.0 };
        }
        q[6] = INSPECT[6] + 2.0 * (1.0 - (t * 0.5).cos() as f32);
        q
    };
    let mut stream = Vec::new();
    let (mut next_frame, mut t) = (0.0, 0.0);
    for _ in 0..1200 {
        let target = (t >= next_frame).then(|| { next_frame += 1.0 / 30.0; traj(t) });
        stream.push(Step { target, dt: LOOP_DT, stale_frame: false });
        t += LOOP_DT;
    }
    let (d, w, _) = differential(&config, rig_default, &stream);
    report("servo 30 fps", d, &w);
    assert_eq!(d, 0.0);
    assert_invariants(&config, &w);

    #[cfg(feature = "policy-runtime")]
    {
        use crate::policy_runtime::{PolicyMode, PolicyRunStart};
        let (mut inner, _) = armed(config.clone(), rig_default());
        inner.follower = Some("sim".into());
        inner.lease_enabled = true;
        let gen = inner.acquire_lease("policy-rig").unwrap();
        let epoch = inner.safety.epoch_id();
        let id = inner.begin_policy_run(PolicyRunStart {
            epoch_id: epoch, connection_id: "policy-rig".into(), lease_generation: gen, policy_id: "rig".into(),
            artifact_digest: "0".repeat(64), backend: "mlp".into(), mode: PolicyMode::Live, sim_only: false,
            max_first_step_rad: None, max_step_rad: None, tag: None, i_am_at_the_arm: false, verified_release_gate: None,
        }).unwrap();
        let mut w = Worst::default();
        for s in &stream {
            if let Some(tg) = &s.target { inner.policy_step(id, tg.clone(), Some(0)).unwrap(); }
            inner.last_heartbeat = Instant::now();
            let prev = inner.commanded_positions.clone();
            inner.tick(s.dt);
            check_tick(&inner, &prev, s.dt, false, &mut w);
        }
        report("policy live 30 fps", 0.0, &w);
        assert!(inner.is_policy_run_active(id), "{:?}", inner.last_action);
        assert_invariants(&config, &w);
    }
}

#[test]
fn gravity_comp_rejected_and_output_disabled_when_dynamics_unqualified() {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.dynamics_qualified = false;
    let (inner, _rig) = armed(cfg, rig_default());
    assert!(inner.gravity_comp_support().is_err());
    assert!(!inner.schema().capabilities.gravity_comp);
    assert!(!inner.gravity_output_enabled());
    let err = inner.gravity_comp_blocker().expect("unqualified dynamics must block gravity_comp");
    assert_eq!(err.code, "UNSUPPORTED");
}

#[test]
fn park_enters_safe_hold_when_destination_violates_workspace() {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.workspace.as_mut().unwrap().max_reach_m = Some(0.15);
    let (mut inner, _rig) = armed(cfg, rig_default());
    inner.execute_stop(crate::StopPolicy::NormalUnload);
    assert!(!inner.pending_normal_unload, "must not enter unload when resting pose is illegal");
    assert!(inner.active_trajectory.is_none(), "must not install unsafe park trajectory");
    assert!(inner.is_torqued, "must keep safe hold with torque on");
}
