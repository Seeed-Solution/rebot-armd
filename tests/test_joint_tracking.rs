//! Phase 1 joint-tracking tests (position lead, default off).
//!
//! Covers the four Phase 1 acceptance tests from the joint-tracking spec:
//! 1. `tracking_off_wire_equivalence`
//! 2. `lead_first_order_tracking`
//! 3. `lead_limits_and_withdrawal`
//! 4. `tracking_safety_references`

mod common;

use std::sync::{Arc, Mutex};
use std::time::Duration;

use armd::collision::CollisionPolicy;
use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointMode, JointTarget};
use armd::model::{ArmConfig, JointState, KeepoutBox, MotionLimitReason};
use armd::motion::{plan_cartesian, GripperCommand, GripperRequest, Speed};
use armd::recording::RecordFrame;
use armd::replay::{plan_frames, ReplayRequest};
use armd::torque_guard::TorqueGuardPolicy;
use armd::tracking::{JointTrackingConfig, JointTrackingJoint, TrackingStrategy};
use armd::{
    ArmObservation, FaultCode, SafetyState, StopPolicy, SupervisorInner, TrajectoryPlanner,
};
use common::INSPECT;

const DT: f64 = 0.02;

/// First-order lag + controllable fault/contact/staleness test driver.
#[derive(Clone)]
struct TrackingTestState {
    dof: usize,
    q: Vec<f32>,
    v: Vec<f32>,
    tau: Vec<f32>,
    enabled: EnableState,
    /// Per-joint first-order time constant tau_s (`dq/dt = (q_out - q) / tau_s`);
    /// 0.0 means instant follow (q = q_out).
    tau_s: Vec<f32>,
    /// Optional pinned position per joint (joint cannot move from this position).
    pinned: Vec<Option<f32>>,
    /// Optional forced measured torque per joint.
    forced_torque: Vec<Option<f32>>,
    /// Optional rigid gripper block `(block_rad, stiffness_nm_per_rad)` on `gripper_idx`.
    gripper_block: Option<(usize, f32, f32)>,
    /// Reported feedback age per joint.
    feedback_age: Duration,
    /// Every frame written via `write_joint_targets`.
    written: Vec<Vec<JointTarget>>,
    /// Include gravity model torque in reported torques when 6+ DOF arm.
    include_gravity_torque: bool,
}

impl TrackingTestState {
    fn new(q0: &[f32], tau_s: Vec<f32>) -> Self {
        let dof = q0.len();
        Self {
            dof,
            q: q0.to_vec(),
            v: vec![0.0; dof],
            tau: vec![0.0; dof],
            enabled: EnableState::Disabled,
            tau_s,
            pinned: vec![None; dof],
            forced_torque: vec![None; dof],
            gripper_block: None,
            feedback_age: Duration::ZERO,
            written: Vec::new(),
            include_gravity_torque: true,
        }
    }
}

#[derive(Clone)]
struct TrackingTestDriver(Arc<Mutex<TrackingTestState>>);

impl TrackingTestDriver {
    fn new(q0: &[f32], tau_s: Vec<f32>) -> Self {
        Self(Arc::new(Mutex::new(TrackingTestState::new(q0, tau_s))))
    }
    fn with<R>(&self, f: impl FnOnce(&mut TrackingTestState) -> R) -> R {
        f(&mut self.0.lock().unwrap())
    }
}

impl ArmDriver for TrackingTestDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            velocity_feedback: true,
            torque_feedback: true,
            temperature_feedback: true,
            torque_feedforward: false,
            enable_feedback: true,
            load_feedback: false,
        }
    }
    fn enable_state(&self) -> EnableState {
        self.0.lock().unwrap().enabled
    }
    fn is_feedback_valid(&self) -> bool {
        self.0.lock().unwrap().feedback_age <= Duration::from_millis(250)
    }
    fn is_enabled(&self) -> bool {
        self.0.lock().unwrap().enabled == EnableState::Enabled
    }
    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        let s = self.0.lock().unwrap();
        vec![Some(s.feedback_age); s.dof]
    }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let s = self.0.lock().unwrap();
        let g = if s.include_gravity_torque && s.dof >= 6 {
            let mut q6 = [0.0_f32; 6];
            q6.copy_from_slice(&s.q[..6]);
            armd::compute_gravity_torques(&q6).ok()
        } else {
            None
        };
        Ok((0..s.dof)
            .map(|i| {
                let model_tau = if let Some(forced) = s.forced_torque[i] {
                    forced
                } else {
                    let grav = g.as_ref().and_then(|gt| gt.get(i).copied()).unwrap_or(0.0);
                    grav + s.tau[i]
                };
                JointState {
                    position: s.q[i],
                    velocity: s.v[i],
                    torque: model_tau,
                    temperature: 30.0,
                }
            })
            .collect())
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        let mut s = self.0.lock().unwrap();
        s.written.push(targets.to_vec());
        if s.enabled != EnableState::Enabled {
            return Ok(());
        }
        for (i, t) in targets.iter().enumerate().take(s.dof) {
            if let Some(pin) = s.pinned[i] {
                s.q[i] = pin;
                s.v[i] = 0.0;
                continue;
            }
            let tau_i = s.tau_s[i];
            let (mut q_next, mut v_next) = if tau_i > 1e-6 {
                // Exact integration of dq/dt = (q_out - q) / tau_i over DT with ZOH q_out.
                let alpha = 1.0 - (-DT / f64::from(tau_i)).exp();
                let qn = f64::from(s.q[i]) + (f64::from(t.position) - f64::from(s.q[i])) * alpha;
                let vn = (f64::from(t.position) - qn) / f64::from(tau_i);
                (qn as f32, vn as f32)
            } else {
                let vn = ((f64::from(t.position) - f64::from(s.q[i])) / DT) as f32;
                (t.position, vn)
            };

            if let Some((g_idx, block, stiffness)) = s.gripper_block {
                if i == g_idx {
                    if q_next >= block {
                        q_next = block;
                        v_next = 0.0;
                        s.tau[i] = (stiffness * (t.position - block)).max(0.0);
                    } else {
                        s.tau[i] = if v_next.abs() > 0.05 { 0.40 * v_next.signum() } else { 0.0 };
                    }
                }
            }
            s.q[i] = q_next;
            s.v[i] = v_next;
        }
        Ok(())
    }
    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        let mut s = self.0.lock().unwrap();
        s.enabled = if enabled { EnableState::Enabled } else { EnableState::Disabled };
        if !enabled {
            s.v.fill(0.0);
        }
        Ok(())
    }
    fn supports_mit_mode_switch(&self) -> bool {
        true
    }
    fn set_joint_modes(&mut self, _modes: &[JointMode], _hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        Ok(())
    }
}

fn rig_at(config: ArmConfig, q0: &[f32], tau_s: Vec<f32>) -> (SupervisorInner, TrackingTestDriver) {
    let drv = TrackingTestDriver::new(q0, tau_s);
    let mut inner = SupervisorInner::new(config).with_driver(Box::new(drv.clone()));
    inner.lease_enabled = false;
    inner.commanded_positions = q0.to_vec();
    inner.measured_positions = q0.to_vec();
    inner.target_positions = q0.to_vec();
    inner.tick(DT);
    inner.set_torque(true).unwrap();
    inner.tick(DT);
    assert!(inner.safety.state().is_operational(), "{:?}", inner.safety.state());
    (inner, drv)
}

fn lead_config(enabled: bool, lag_j1: f32) -> JointTrackingConfig {
    let mut jt = JointTrackingConfig::rebot_dm();
    jt.enabled = enabled;
    jt.joints[0].strategy = TrackingStrategy::PositionLead;
    jt.joints[0].lag_s = lag_j1;
    jt
}

/// Bit-exact comparison of two `JointTarget` vectors.
fn assert_targets_bit_eq(a: &[Vec<JointTarget>], b: &[Vec<JointTarget>], label: &str) {
    assert_eq!(a.len(), b.len(), "{label}: frame count mismatch");
    for (k, (fa, fb)) in a.iter().zip(b).enumerate() {
        assert_eq!(fa.len(), fb.len(), "{label}: tick {k} dof mismatch");
        for (j, (ta, tb)) in fa.iter().zip(fb).enumerate() {
            assert_eq!(ta.position.to_bits(), tb.position.to_bits(), "{label}: tick {k} joint {j} position");
            assert_eq!(ta.velocity.map(f32::to_bits), tb.velocity.map(f32::to_bits), "{label}: tick {k} joint {j} velocity");
            assert_eq!(ta.torque.map(f32::to_bits), tb.torque.map(f32::to_bits), "{label}: tick {k} joint {j} torque");
            assert_eq!(ta.kp.map(f32::to_bits), tb.kp.map(f32::to_bits), "{label}: tick {k} joint {j} kp");
            assert_eq!(ta.kd.map(f32::to_bits), tb.kd.map(f32::to_bits), "{label}: tick {k} joint {j} kd");
        }
    }
}

/// Compare observation fields (ignoring wall-clock `timestamp` and `since_ms`).
fn assert_obs_equiv(a: &[ArmObservation], b: &[ArmObservation], label: &str) {
    assert_eq!(a.len(), b.len(), "{label}: obs count mismatch");
    let bits_vec = |v: &[f32]| -> Vec<u32> { v.iter().map(|x| x.to_bits()).collect() };
    for (k, (oa, ob)) in a.iter().zip(b).enumerate() {
        assert_eq!(bits_vec(&oa.positions), bits_vec(&ob.positions), "{label}: tick {k} positions");
        assert_eq!(bits_vec(&oa.velocities), bits_vec(&ob.velocities), "{label}: tick {k} velocities");
        assert_eq!(bits_vec(&oa.torques), bits_vec(&ob.torques), "{label}: tick {k} torques");
        assert_eq!(bits_vec(&oa.commanded_positions), bits_vec(&ob.commanded_positions), "{label}: tick {k} commanded");
        assert_eq!(oa.is_torqued, ob.is_torqued, "{label}: tick {k} is_torqued");
        assert_eq!(oa.safety_state, ob.safety_state, "{label}: tick {k} safety_state");
        assert_eq!(oa.fault, ob.fault, "{label}: tick {k} fault");
        assert_eq!(oa.gripper, ob.gripper, "{label}: tick {k} gripper");
        assert_eq!(
            oa.motion_limit.as_ref().map(|m| (m.reason, m.joint)),
            ob.motion_limit.as_ref().map(|m| (m.reason, m.joint)),
            "{label}: tick {k} motion_limit"
        );
    }
}

/// Phase 1 Test 1: With tracking missing or `enabled=false`, frames and observations
/// must be bit-identical across quintic, path, replay, stream, braking, park and jaw contact.
#[test]
fn tracking_off_wire_equivalence() {
    let run_scenario = |joint_tracking: Option<JointTrackingConfig>| -> (Vec<Vec<JointTarget>>, Vec<ArmObservation>) {
        let cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            joint_tracking,
            ..ArmConfig::rebot_dm()
        };
        let q0 = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0, -4.0];
        let (mut inner, drv) = rig_at(cfg, &q0, vec![0.25, 0.25, 0.25, 0.10, 0.05, 0.05, 0.15]);
        // m1: with tracking absent or disabled, output history must stay empty after enable/tick.
        assert!(inner.tracking.last_issued_positions.is_none(), "history must stay empty when tracking is off");
        assert!(inner.tracking.last_issued_velocities.is_none(), "velocity history must stay empty when tracking is off");
        let mut obs_log = Vec::new();
        let mut step = |inner: &mut SupervisorInner, n: usize| {
            for _ in 0..n {
                obs_log.push(inner.tick(DT));
            }
        };

        // 1. Quintic move
        let inp = inner.plan_input().unwrap();
        let mut goal = [0.0_f64; 6];
        for i in 0..6 {
            goal[i] = f64::from(q0[i]);
        }
        goal[0] += 0.12;
        inner.install_joint_goal(&inp, &goal, 0.5, "joint_move", None, None).unwrap();
        step(&mut inner, 120);

        // 2. Cartesian path move
        let inp = inner.plan_input().unwrap();
        let mut pose = inner.config.arm_model.fk_f32(&inner.commanded_positions).unwrap();
        pose.p[1] += 0.03;
        let path = plan_cartesian(&inp, pose, Speed::default()).unwrap();
        inner.install_path(&inp, path, "move_to", None).unwrap();
        step(&mut inner, 120);

        // 3. Replay move
        let start = inner.commanded_positions.clone();
        let replay_frames: Vec<RecordFrame> = (0..=40)
            .map(|k| {
                let t = k as f64 * DT;
                let s = (1.0 - (2.0 * std::f64::consts::PI * t / 0.8).cos()) as f32 / 2.0;
                let mut q = start.clone();
                q[0] += 0.05 * s;
                RecordFrame { timestamp: 10.0 + t, observation_state: q.clone(), action: q }
            })
            .collect();
        let epoch = inner.safety.epoch_id();
        let r_in = inner
            .replay_input(
                ReplayRequest { episode: "eq".into(), speed: 1.0, gripper: false, tag: None, allow_stale: false },
                epoch,
            )
            .unwrap();
        let r_plan = plan_frames(r_in, &replay_frames).unwrap();
        inner.install_replay(r_plan).unwrap();
        step(&mut inner, 120);

        // 4. Servo streaming + 5. Braking (dropping servo target mid-motion)
        let epoch = inner.safety.epoch_id();
        let mut target = inner.commanded_positions.clone();
        for _ in 0..20 {
            target[0] += 0.004;
            inner.servo(target.clone(), epoch).unwrap();
            step(&mut inner, 1);
        }
        inner.servo_target = None; // begin controlled braking
        step(&mut inner, 40);

        // 6. Jaw contact (rigid object at -1.75 rad)
        drv.with(|s| s.gripper_block = Some((6, -1.75, 25.0)));
        let epoch = inner.safety.epoch_id();
        inner
            .start_gripper(
                GripperRequest {
                    command: GripperCommand::Close,
                    fraction: None,
                    grasp: true,
                    speed: 1.0,
                    timeout_s: Some(5.0),
                    tag: None,
                },
                epoch,
            )
            .unwrap();
        step(&mut inner, 150);

        // 7. Park (NormalUnload)
        drv.with(|s| s.gripper_block = None);
        inner.apply_stop_policy(StopPolicy::NormalUnload);
        step(&mut inner, 200);

        let frames = drv.with(|s| s.written.clone());
        (frames, obs_log)
    };

    let (frames_none, obs_none) = run_scenario(None);
    let (frames_disabled, obs_disabled) = run_scenario(Some(JointTrackingConfig::rebot_dm()));

    assert!(frames_none.len() > 700, "expected full multi-producer trace, got {}", frames_none.len());
    assert_targets_bit_eq(&frames_none, &frames_disabled, "tracking_off_wire_equivalence");
    assert_obs_equiv(&obs_none, &obs_disabled, "tracking_off_wire_equivalence");
}

/// Phase 1 Test 2: Integrates `dq/dt = (q_out - q) / 0.68` at 20 ms with a smooth
/// <= 0.03 rad/s reference below the lead cap; requires moving RMS <= 3 mrad and
/// >= 60% reduction versus off.
#[test]
fn lead_first_order_tracking() {
    let run = |enabled: bool| -> (f64, f32, f32) {
        let cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            workspace: None,
            joint_tracking: Some(lead_config(enabled, 0.68)),
            ..ArmConfig::rebot_dm()
        };
        let q0 = [0.0_f32; 7];
        // J1 follows first-order lag tau = 0.68 s; other joints instant.
        let (mut inner, drv) = rig_at(cfg, &q0, vec![0.68, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        drv.with(|s| s.written.clear());

        // Smooth quintic on J1: 0.0 -> 0.08 rad in 5.5 s.
        // Peak velocity = 1.875 * 0.08 / 5.5 = 0.02727 rad/s <= 0.03 rad/s.
        // Peak lead = 0.68 * 0.02727 = 0.0185 rad < 0.030 rad (below lead cap).
        let target = vec![0.08, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let traj = TrajectoryPlanner::plan(
            &q0,
            &[0.0; 7],
            &target,
            &[0.0; 7],
            Some(5.5),
            &inner.config.limits,
        )
        .unwrap();
        let _ = inner.safety.mark_motion_started();
        inner.active_trajectory = Some(armd::motion::ActiveTrajectory::quintic(traj, None));

        let mut sq_err_sum = 0.0_f64;
        let mut moving_samples = 0usize;
        let mut peak_v = 0.0_f32;
        let mut peak_lead = 0.0_f32;

        // 5.5 s = 275 ticks + 25 settle ticks
        for _ in 0..300 {
            let obs = inner.tick(DT);
            let v_d = inner.commanded_velocities[0];
            peak_v = peak_v.max(v_d.abs());
            let q_d = obs.commanded_positions[0];
            let q_m = obs.positions[0];
            let q_out = drv.with(|s| s.written.last().unwrap()[0].position);
            peak_lead = peak_lead.max((q_out - q_d).abs());

            // Verify observation preserves nominal q_d (not q_out)
            assert_eq!(obs.commanded_positions[0].to_bits(), inner.commanded_positions[0].to_bits());
            assert!(inner.safety.state().is_operational(), "unexpected fault: {:?}", inner.safety.state());

            if v_d.abs() > 1e-3 {
                let err = f64::from(q_d - q_m);
                sq_err_sum += err * err;
                moving_samples += 1;
            }
        }
        assert!(moving_samples >= 200, "expected >= 200 moving samples, got {moving_samples}");
        let rms = (sq_err_sum / moving_samples as f64).sqrt();
        (rms, peak_v, peak_lead)
    };

    let (rms_off, peak_v_off, peak_lead_off) = run(false);
    let (rms_on, peak_v_on, peak_lead_on) = run(true);
    let reduction_pct = (1.0 - rms_on / rms_off) * 100.0;

    eprintln!(
        "lead_first_order_tracking: moving_rms_off={:.6} rad ({:.3} mrad), moving_rms_on={:.6} rad ({:.3} mrad), reduction={:.2}%, peak_v={:.4} rad/s, peak_lead_on={:.4} rad",
        rms_off,
        rms_off * 1000.0,
        rms_on,
        rms_on * 1000.0,
        reduction_pct,
        peak_v_on,
        peak_lead_on
    );

    assert!(peak_v_off <= 0.0301 && peak_v_on <= 0.0301, "reference speed exceeded 0.03 rad/s: {peak_v_on}");
    assert_eq!(peak_lead_off, 0.0, "tracking off must emit zero lead");
    assert!(peak_lead_on > 0.015 && peak_lead_on < 0.030, "expected unclipped positive lead, got {peak_lead_on}");
    assert!(rms_on <= 0.003, "moving RMS with lead on {:.6} rad > 3 mrad", rms_on);
    assert!(
        reduction_pct >= 60.0,
        "expected >= 60% RMS reduction vs off, got {reduction_pct:.2}% (off={rms_off:.6}, on={rms_on:.6})"
    );
}

/// Phase 1 Test 3: Covers reversal, clipping, coupled keep-out, stale feedback,
/// and unsafe return to nominal.
#[test]
fn lead_limits_and_withdrawal() {
    // --- (a) Reversal & (b) Clipping ---
    {
        let cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            workspace: None,
            joint_tracking: Some(lead_config(true, 0.68)),
            ..ArmConfig::rebot_dm()
        };
        let q0 = [0.0_f32; 7];
        let (mut inner, drv) = rig_at(cfg, &q0, vec![0.68, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        drv.with(|s| s.written.clear());

        // Forward quintic 0.0 -> 0.35 rad in 5.0 s: peak v_d = 0.131 rad/s (exceeds clip 0.030/0.68 = 0.0441),
        // peak a_d = 0.081 rad/s^2 (clip corner accel ~2.7 rad/s^2 < 5.0 rad/s^2).
        let fwd = TrajectoryPlanner::plan(
            &q0,
            &[0.0; 7],
            &[0.35, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            &[0.0; 7],
            Some(5.0),
            &inner.config.limits,
        )
        .unwrap();
        let _ = inner.safety.mark_motion_started();
        inner.active_trajectory = Some(armd::motion::ActiveTrajectory::quintic(fwd, None));

        let mut max_pos_lead = 0.0_f32;
        let mut clipped_ticks = 0usize;
        for _ in 0..260 {
            let obs = inner.tick(DT);
            assert!(inner.safety.state().is_operational(), "forward fault: {:?}", inner.safety.state());
            let q_out = drv.with(|s| s.written.last().unwrap()[0].position);
            let lead = q_out - obs.commanded_positions[0];
            assert!(lead.abs() <= 0.030 + 1e-5, "lead {lead} exceeded max_lead_rad 0.030");
            max_pos_lead = max_pos_lead.max(lead);
            if lead >= 0.0299 {
                clipped_ticks += 1;
            }
        }
        assert!(max_pos_lead >= 0.0299 && clipped_ticks >= 50, "expected clipping plateau, max={max_pos_lead}, ticks={clipped_ticks}");
        // At rest after forward move: zero velocity -> zero lead.
        let end_lead = drv.with(|s| s.written.last().unwrap()[0].position) - inner.commanded_positions[0];
        assert!(end_lead.abs() <= 1e-5, "expected zero lead at reversal rest, got {end_lead}");

        // Reverse quintic 0.35 -> 0.0 rad in 5.0 s.
        let rev = TrajectoryPlanner::plan(
            &inner.commanded_positions,
            &[0.0; 7],
            &[0.0; 7],
            &[0.0; 7],
            Some(5.0),
            &inner.config.limits,
        )
        .unwrap();
        let _ = inner.safety.mark_motion_started();
        inner.active_trajectory = Some(armd::motion::ActiveTrajectory::quintic(rev, None));

        let mut min_neg_lead = 0.0_f32;
        for _ in 0..260 {
            let obs = inner.tick(DT);
            assert!(inner.safety.state().is_operational(), "reverse fault: {:?}", inner.safety.state());
            let q_out = drv.with(|s| s.written.last().unwrap()[0].position);
            let lead = q_out - obs.commanded_positions[0];
            assert!(lead.abs() <= 0.030 + 1e-5, "reverse lead {lead} exceeded max_lead_rad 0.030");
            min_neg_lead = min_neg_lead.min(lead);
        }
        assert!(min_neg_lead <= -0.0299, "expected negative clipped lead during reversal, got {min_neg_lead}");
        let final_lead = drv.with(|s| s.written.last().unwrap()[0].position) - inner.commanded_positions[0];
        assert!(final_lead.abs() <= 1e-5, "expected zero lead after reverse stop, got {final_lead}");
    }

    // --- (c) Coupled keep-out (multiplier backoff without mutating servo_ws_min_depths) ---
    {
        let mut jt = JointTrackingConfig::rebot_dm();
        jt.enabled = true;
        jt.joints[0].lag_s = 0.68;
        jt.joints[1].lag_s = 0.65;

        let mut cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            joint_tracking: Some(jt),
            ..ArmConfig::rebot_dm()
        };
        let q0 = INSPECT;
        // Compute TCP at q0 and at q_full = q0 + (0.015, 0.015) on J1, J2.
        let p0 = cfg.arm_model.fk_f32(&q0).unwrap().p;
        let mut q_full = q0;
        q_full[0] += 0.020;
        q_full[1] += 0.020;
        let p_full = cfg.arm_model.fk_f32(&q_full).unwrap().p;
        // Place a small keep-out box covering p_full (and excluding p0 and midpoint p_half).
        let mut q_half = q0;
        q_half[0] += 0.010;
        q_half[1] += 0.010;
        let p_half = cfg.arm_model.fk_f32(&q_half).unwrap().p;
        let dir = [p_full[0] - p0[0], p_full[1] - p0[1], p_full[2] - p0[2]];
        let d_norm = (dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]).sqrt();
        assert!(d_norm > 0.005, "coupled motion should move TCP measurably");
        // Small keepout centered around p_full that does not touch p0 or p_half.
        let half_box = 0.002_f32;
        let kb = KeepoutBox {
            name: "coupled_lead_box".into(),
            min_m: [p_full[0] as f32 - half_box, p_full[1] as f32 - half_box, p_full[2] as f32 - half_box],
            max_m: [p_full[0] as f32 + half_box, p_full[1] as f32 + half_box, p_full[2] as f32 + half_box],
        };
        // Verify box blocks q_full but clears q0 and q_half.
        cfg.workspace.as_mut().unwrap().keepout_boxes.push(kb);
        let ws = cfg.workspace.as_ref().unwrap();
        assert!(cfg.arm_model.workspace_violation(&q0, ws, &cfg.camera_mount).total == 0.0);
        assert!(cfg.arm_model.workspace_violation(&q_half, ws, &cfg.camera_mount).total == 0.0);
        assert!(cfg.arm_model.workspace_violation(&q_full, ws, &cfg.camera_mount).total > 0.0);
        let _ = p_half;

        // Higher accel limits here so we isolate pure keepout multiplier backoff from accel limits.
        cfg.limits[0].max_acceleration = 100.0;
        cfg.limits[1].max_acceleration = 100.0;
        let (mut inner, _drv) = rig_at(cfg, &q0, vec![0.0; 7]);
        // Set a sentinel servo_ws_min_depths to verify candidate evaluation never mutates it.
        let (ws_ref, cam_ref) = inner.workspace_policy().unwrap();
        let eval = armd::chain::WorkspaceEvaluator::new(&inner.config.arm_model, ws_ref, cam_ref);
        let mut c0 = eval.alloc_buf();
        eval.eval_into(&q0, &mut c0, "test").unwrap();
        let sentinel = eval.init_min_depths(&c0);
        inner.servo_ws_min_depths = Some((inner.workspace_revision, sentinel.clone()));

        // Request velocities producing base_lead ~ (+0.020, +0.020) on J1, J2 -> mult 1.0 hits keepout!
        inner.commanded_velocities[0] = 0.020 / 0.68;
        inner.commanded_velocities[1] = 0.020 / 0.65;
        let jt_ref = inner.config.joint_tracking.as_ref().unwrap().clone();
        let out = inner.compute_position_lead_output(DT, &jt_ref).expect("should back off to smaller multiplier");
        assert!(out.multiplier < 1.0 && out.multiplier > 0.0, "expected reduced multiplier (1/2..1/8), got {}", out.multiplier);
        // Combined output must clear the keepout box.
        let ws = inner.config.workspace.as_ref().unwrap();
        assert_eq!(inner.config.arm_model.workspace_violation(&out.positions, ws, &inner.config.camera_mount).total, 0.0);
        // Live escape bookkeeping must be untouched.
        assert_eq!(inner.servo_ws_min_depths.as_ref().unwrap().1.as_slice(), sentinel.as_slice());
    }

    // --- (d) Stale feedback ---
    {
        // Subcase d1: small lead + stale feedback (> 100 ms, < 500 ms) -> withdraws to 0 lead safely.
        let cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            workspace: None,
            joint_tracking: Some(lead_config(true, 0.68)),
            ..ArmConfig::rebot_dm()
        };
        let q0 = [0.0_f32; 7];
        let (mut inner, drv) = rig_at(cfg, &q0, vec![0.0; 7]);
        // Small smooth velocity -> lead = 0.68 * 0.002 = 0.00136 rad (within 5.0 * 0.02^2 = 0.002 rad).
        let traj = TrajectoryPlanner::plan(
            &q0,
            &[0.0; 7],
            &[0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            &[0.0; 7],
            Some(10.0),
            &inner.config.limits,
        )
        .unwrap();
        let _ = inner.safety.mark_motion_started();
        inner.active_trajectory = Some(armd::motion::ActiveTrajectory::quintic(traj, None));
        for _ in 0..100 {
            inner.tick(DT);
        }
        let lead_before = drv.with(|s| s.written.last().unwrap()[0].position) - inner.commanded_positions[0];
        assert!(lead_before > 0.0005 && lead_before < 0.0018, "expected small active lead, got {lead_before}");

        // Feedback ages to 150 ms (> 100 ms freshness gate, < 500 ms bus loss).
        drv.with(|s| s.feedback_age = Duration::from_millis(150));
        let obs = inner.tick(DT);
        assert!(inner.safety.state().is_operational(), "safe stale withdrawal should stay operational: {:?}", inner.safety.state());
        let q_out = drv.with(|s| s.written.last().unwrap()[0].position);
        assert!((q_out - obs.commanded_positions[0]).abs() <= 1e-6, "stale feedback must suppress lead, got q_out={q_out} vs q_d={}", obs.commanded_positions[0]);

        // Subcase d2: large active lead + stale feedback -> snapping back to nominal exceeds accel limit -> fault!
        let cfg2 = ArmConfig {
            lease_timeout_ms: 600_000,
            workspace: None,
            joint_tracking: Some(lead_config(true, 0.68)),
            ..ArmConfig::rebot_dm()
        };
        let (mut inner2, drv2) = rig_at(cfg2, &q0, vec![0.68, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        let traj2 = TrajectoryPlanner::plan(
            &q0,
            &[0.0; 7],
            &[0.08, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            &[0.0; 7],
            Some(5.5),
            &inner2.config.limits,
        )
        .unwrap();
        let _ = inner2.safety.mark_motion_started();
        inner2.active_trajectory = Some(armd::motion::ActiveTrajectory::quintic(traj2, None));
        for _ in 0..130 {
            inner2.tick(DT);
        }
        let lead_big = drv2.with(|s| s.written.last().unwrap()[0].position) - inner2.commanded_positions[0];
        assert!(lead_big > 0.015, "expected large active lead, got {lead_big}");

        let writes_before = drv2.with(|s| s.written.len());
        drv2.with(|s| s.feedback_age = Duration::from_millis(150));
        let obs2 = inner2.tick(DT);
        assert!(matches!(obs2.safety_state, SafetyState::Fault(_)), "unsafe stale withdrawal must fault, got {:?}", obs2.safety_state);
        assert_eq!(obs2.fault.as_ref().map(|f| f.code), Some(FaultCode::JointLimit));
        assert!(inner2.active_trajectory.is_none(), "motion must be terminated");
        assert!(inner2.tracking.last_issued_positions.is_none(), "output history must be invalidated");
        // Verify the unsafe nominal target was NOT written; only stationary hold at measured pose was written.
        let last_written = drv2.with(|s| s.written[writes_before][0].position);
        assert!((last_written - obs2.positions[0]).abs() <= 1e-4, "expected hold at measured {}, got {last_written}", obs2.positions[0]);
    }

    // --- (e) Unsafe return to nominal (keep-out between last_issued and nominal q_d) ---
    {
        // Both streaming (recoverable workspace hold) and trajectory (fault).
        for streaming in [true, false] {
            let mut jt = JointTrackingConfig::rebot_dm();
            jt.enabled = true;
            jt.joints[0].lag_s = 0.68;
            let mut cfg = ArmConfig {
                lease_timeout_ms: 600_000,
                joint_tracking: Some(jt),
                ..ArmConfig::rebot_dm()
            };
            cfg.limits[0].max_acceleration = 100.0;
            let q0 = INSPECT;
            let mut q_lead = q0;
            q_lead[0] += 0.008; // 0.40 rad/s over DT <= 0.45 rad/s active cap so keep-out check governs
            let mut q_mid = q0;
            q_mid[0] += 0.004;
            let p_mid = cfg.arm_model.fk_f32(&q_mid).unwrap().p;

            let (mut inner, drv) = rig_at(cfg, &q0, vec![0.0; 7]);
            // Simulate that previous tick successfully issued q_lead, and now q_d is back at q0
            // with a keep-out box sitting squarely on the midpoint of last_issued -> q_d!
            inner.tracking.record_issued(q_lead.to_vec(), vec![0.0; 7]);
            let half_box = 0.0008_f32;
            inner.config.workspace.as_mut().unwrap().keepout_boxes.push(KeepoutBox {
                name: "return_block".into(),
                min_m: [p_mid[0] as f32 - half_box, p_mid[1] as f32 - half_box, p_mid[2] as f32 - half_box],
                max_m: [p_mid[0] as f32 + half_box, p_mid[1] as f32 + half_box, p_mid[2] as f32 + half_box],
            });
            inner.workspace_revision += 1;

            if streaming {
                inner.servo_target = Some(q0.to_vec());
            } else {
                let traj = TrajectoryPlanner::plan(&q0, &[0.0; 7], &q0, &[0.0; 7], Some(1.0), &inner.config.limits).unwrap();
                inner.active_trajectory = Some(armd::motion::ActiveTrajectory::quintic(traj, None));
            }
            let obs = inner.tick(DT);
            assert!(inner.servo_target.is_none() && inner.active_trajectory.is_none(), "motion must terminate");
            assert!(inner.tracking.last_issued_positions.is_none(), "history must invalidate");
            if streaming {
                assert_eq!(obs.motion_limit.as_ref().map(|m| m.reason), Some(MotionLimitReason::Workspace));
                assert!(obs.workspace_violation.is_some());
            } else {
                assert!(matches!(obs.safety_state, SafetyState::Fault(_)), "trajectory unsafe return must fault, got {:?}", obs.safety_state);
            }
            let _ = drv;
        }
    }
}

/// Phase 1 Test 4: Independently trips following error, path stall, collision, and
/// torque hold using measured inputs with position-lead tracking enabled, and checks
/// jaw squeeze numerically (7-DOF B601-DM and 6-DOF 5+1 follower).
#[test]
fn tracking_safety_references() {
    // --- (a) Following error trips on nominal q_d vs fresh q_m (never subtracts lag) ---
    {
        let mut cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            workspace: None,
            joint_tracking: Some(lead_config(true, 0.68)),
            ..ArmConfig::rebot_dm()
        };
        // Keep POSVEL_LAG_GAIN and servo_max_lead_rad invariants valid (0.45/1.5 = 0.30 < 0.35, 0.20 < 0.35).
        cfg.following_error.max_error_rad = 0.35;
        cfg.following_error.persistence_ms = 40;
        let q0 = [0.0_f32; 7];
        let (mut inner, drv) = rig_at(cfg, &q0, vec![0.68, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        drv.with(|s| s.pinned[0] = Some(0.0));

        let traj = TrajectoryPlanner::plan(
            &q0,
            &[0.0; 7],
            &[0.50, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            &[0.0; 7],
            Some(5.0),
            &inner.config.limits,
        )
        .unwrap();
        let _ = inner.safety.mark_motion_started();
        inner.active_trajectory = Some(armd::motion::ActiveTrajectory::quintic(traj, None));

        let mut tripped = false;
        let mut tripped_nom_err = 0.0_f32;
        for _ in 0..250 {
            std::thread::sleep(Duration::from_millis(5));
            let obs = inner.tick(DT);
            if matches!(obs.safety_state, SafetyState::Fault(_)) {
                assert_eq!(obs.fault.as_ref().map(|f| f.code), Some(FaultCode::FollowingError));
                tripped = true;
                break;
            }
            tripped_nom_err = (obs.commanded_positions[0] - obs.positions[0]).abs();
        }
        assert!(tripped, "expected FollowingError fault on pinned joint with tracking enabled (max err {tripped_nom_err})");
        assert!(tripped_nom_err >= 0.35, "expected trip only after nominal error crossed 0.35 rad, got {tripped_nom_err}");
    }

    // --- (a2) Streaming directional allowance trims added lead relative to q_m ---
    {
        let cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            workspace: None,
            joint_tracking: Some(lead_config(true, 0.68)),
            ..ArmConfig::rebot_dm()
        };
        let q0 = [0.0_f32; 7];
        let (mut inner, _drv) = rig_at(cfg, &q0, vec![0.0; 7]);
        let stream_lead = inner.config.servo_max_lead_rad;
        // Put q_d 0.005 rad short of q_m + stream_lead, with v_d wanting +0.025 rad lead.
        inner.servo_target = Some(vec![stream_lead + 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        inner.measured_positions[0] = 0.0;
        inner.commanded_positions[0] = stream_lead - 0.005;
        inner.commanded_velocities[0] = 0.025 / 0.68;
        inner.tracking.record_issued(inner.commanded_positions.clone(), inner.commanded_velocities.clone());
        let jt = inner.config.joint_tracking.as_ref().unwrap().clone();
        // Allow accel for single-shot unit check of directional trimming.
        inner.config.limits[0].max_acceleration = 100.0;
        let out = inner.compute_position_lead_output(DT, &jt).unwrap();
        assert!(
            out.positions[0] <= inner.measured_positions[0] + stream_lead + 1e-5,
            "streaming q_out {} exceeded q_m + servo_max_lead_rad ({})",
            out.positions[0],
            inner.measured_positions[0] + stream_lead
        );
        assert!(
            (out.positions[0] - (inner.measured_positions[0] + stream_lead)).abs() <= 1e-4,
            "expected lead trimmed tightly to remaining allowance, got {}",
            out.positions[0]
        );
    }

    // --- (b) Path stall trips TRACKING_STALL on nominal q_d vs q_m ---
    {
        let mut cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            joint_tracking: Some(lead_config(true, 0.68)),
            ..ArmConfig::rebot_dm()
        };
        // Raise following_error above LAG_STOP (0.45) so path stall fires independently.
        cfg.following_error.max_error_rad = 1.0;
        let q0 = INSPECT;
        let (mut inner, drv) = rig_at(cfg, &q0, vec![0.0; 7]);
        let inp = inner.plan_input().unwrap();
        let mut goal = inner.config.arm_model.fk_f32(&q0).unwrap();
        goal.p[1] += 0.12;
        let path = plan_cartesian(&inp, goal, Speed::default()).unwrap();
        let (id, _) = inner.install_path(&inp, path, "move_to", None).unwrap();

        // Pin J1 0.50 rad away (> LAG_STOP 0.45 rad) so path rate drops to 0 and stalls after 1.0 s.
        drv.with(|s| s.pinned[0] = Some(q0[0] - 0.50));
        let mut stall_code = None;
        for _ in 0..100 {
            let obs = inner.tick(DT);
            if let Some(a) = obs.action.as_ref().filter(|a| a.id == id && !a.is_running()) {
                stall_code = a.code.clone();
                break;
            }
        }
        assert_eq!(stall_code.as_deref(), Some("TRACKING_STALL"));
        assert!(inner.safety.state().is_operational(), "TRACKING_STALL should not latch spurious tracking fault");
    }

    // --- (c) Collision trips on measured residual, preserves jaw, discards tracking lead ---
    {
        let cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            workspace: None,
            collision: Some(CollisionPolicy {
                enabled: true,
                persist_ticks: 2,
                resume_after_s: 30.0,
                ..ArmConfig::rebot_dm().collision.unwrap()
            }),
            joint_tracking: Some(lead_config(true, 0.68)),
            ..ArmConfig::rebot_dm()
        };
        let q0 = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0, -2.5];
        let (mut inner, drv) = rig_at(cfg, &q0, vec![0.68, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        let traj = TrajectoryPlanner::plan(
            &q0,
            &[0.0; 7],
            &[0.10, -1.167, -1.565, 1.435, 0.0, 0.0, -2.5],
            &[0.0; 7],
            Some(6.0),
            &inner.config.limits,
        )
        .unwrap();
        let _ = inner.safety.mark_motion_started();
        inner.active_trajectory = Some(armd::motion::ActiveTrajectory::quintic(traj, None));
        for _ in 0..100 {
            inner.tick(DT);
        }
        let lead_before = drv.with(|s| s.written.last().unwrap()[0].position) - inner.commanded_positions[0];
        assert!(lead_before > 0.008, "expected active lead before collision, got {lead_before}");

        // Inject +5.0 N·m residual on J1 (threshold 2.0 N·m) for 3 ticks.
        drv.with(|s| s.tau[0] = 5.0);
        for _ in 0..3 {
            inner.tick(DT);
        }
        assert!(inner.collision.hold.is_some(), "collision guard must trip");
        assert!(inner.active_trajectory.is_none(), "collision must cancel motion");
        let last_written = drv.with(|s| s.written.last().unwrap().clone());
        // Lead discarded: q_out == nominal hold pose (measured pose) on arm joints.
        assert!(
            (last_written[0].position - inner.commanded_positions[0]).abs() <= 1e-6,
            "collision must discard lead: written {} vs hold {}",
            last_written[0].position,
            inner.commanded_positions[0]
        );
        // Jaw command preserved.
        assert!((last_written[6].position - (-2.5)).abs() <= 1e-5, "collision must preserve jaw");
    }

    // --- (d) Torque hold trips on measured torque and enforces directional boundary on q_out ---
    {
        let cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            workspace: None,
            torque_guard: Some(TorqueGuardPolicy { enabled: true, ratio: 0.60, persist_ms: 40, ..Default::default() }),
            joint_tracking: Some(lead_config(true, 0.68)),
            ..ArmConfig::rebot_dm()
        };
        let q0 = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0, -2.5];
        let (mut inner, drv) = rig_at(cfg, &q0, vec![0.0; 7]);
        // Force +18.0 N·m on J1 (threshold = 0.60 * 28 = 16.8 N·m, sign = +1).
        drv.with(|s| s.forced_torque[0] = Some(18.0));
        for _ in 0..4 {
            inner.tick(DT);
        }
        let hold = inner.torque_guard.hold.expect("torque guard must trip");
        assert_eq!(hold.joint, 0);
        assert_eq!(hold.sign, 1.0);
        assert!(inner.safety.state().is_operational(), "torque trip should hold without spurious fault");

        // Relax torque slightly below trip threshold while keeping hold active.
        drv.with(|s| s.forced_torque[0] = Some(5.0));
        // Even if commanded velocity tries to point in the +1 (push) direction, tracking lead
        // must NEVER push q_out past hold_q in the push direction.
        inner.commanded_velocities[0] = 0.03;
        let jt = inner.config.joint_tracking.as_ref().unwrap().clone();
        let out_push = inner.compute_position_lead_output(DT, &jt).unwrap();
        assert!(
            (out_push.positions[0] - hold.hold_q) * hold.sign <= 1e-6,
            "lead pushed into torque hold: q_out={} vs hold_q={}",
            out_push.positions[0],
            hold.hold_q
        );

        // Relieving direction (-1): lead is allowed strictly on the relieving side.
        inner.commanded_velocities[0] = -0.02;
        let out_retreat = inner.compute_position_lead_output(DT, &jt).unwrap();
        assert!(
            (out_retreat.positions[0] - hold.hold_q) * hold.sign <= 1e-6,
            "retreat must stay on relieving side: q_out={} vs hold_q={}",
            out_retreat.positions[0],
            hold.hold_q
        );
    }

    // --- (e) Jaw squeeze numerical check: 7-DOF B601-DM and 6-DOF (5+1) follower ---
    {
        let run_jaw = |enabled: bool| -> Vec<(f32, f32, f32)> {
            let cfg = ArmConfig {
                lease_timeout_ms: 600_000,
                workspace: None,
                joint_tracking: Some(lead_config(enabled, 0.68)),
                ..ArmConfig::rebot_dm()
            };
            let q0 = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0, -4.0];
            let (mut inner, drv) = rig_at(cfg, &q0, vec![0.68, 0.0, 0.0, 0.0, 0.0, 0.0, 0.22]);
            drv.with(|s| s.gripper_block = Some((6, -1.75, 25.0)));
            let epoch = inner.safety.epoch_id();
            inner
                .start_gripper(
                    GripperRequest {
                        command: GripperCommand::Close,
                        fraction: None,
                        grasp: true,
                        speed: 1.0,
                        timeout_s: Some(6.0),
                        tag: None,
                    },
                    epoch,
                )
                .unwrap();
            let mut trace = Vec::new();
            for _ in 0..160 {
                let obs = inner.tick(DT);
                let q_out = drv.with(|s| s.written.last().unwrap()[6].position);
                trace.push((q_out, obs.positions[6], obs.torques[6]));
            }
            trace
        };

        let jaw_off = run_jaw(false);
        let jaw_on = run_jaw(true);
        assert_eq!(jaw_off.len(), jaw_on.len());
        let mut peak_squeeze_cmd = f32::NEG_INFINITY;
        for (k, ((co, mo, to), (cn, mn, tn))) in jaw_off.iter().zip(&jaw_on).enumerate() {
            assert_eq!(co.to_bits(), cn.to_bits(), "7-DOF jaw tick {k} command mismatch");
            assert_eq!(mo.to_bits(), mn.to_bits(), "7-DOF jaw tick {k} measured mismatch");
            assert_eq!(to.to_bits(), tn.to_bits(), "7-DOF jaw tick {k} torque mismatch");
            peak_squeeze_cmd = peak_squeeze_cmd.max(*cn);
        }
        // Verify contact + squeeze actually occurred past the rigid object at -1.75 rad.
        assert!(peak_squeeze_cmd > -1.75, "expected squeeze past contact -1.75, got {peak_squeeze_cmd}");

        // 6-DOF (5 arm joints + gripper at index 5) follower bypass check.
        let so101_path = concat!(env!("CARGO_MANIFEST_DIR"), "/profiles/so_arm101.json");
        let run_6dof_jaw = |enabled: bool| -> Vec<f32> {
            let mut cfg = ArmConfig::from_profile_file(so101_path).unwrap();
            cfg.lease_timeout_ms = 600_000;
            cfg.workspace = None;
            cfg.joint_tracking = Some(JointTrackingConfig {
                enabled,
                joints: (0..cfg.arm_dof())
                    .map(|_| JointTrackingJoint {
                        strategy: TrackingStrategy::PositionLead,
                        lag_s: 0.40,
                        max_lead_rad: 0.030,
                        ..Default::default()
                    })
                    .collect(),
            });
            cfg.validate().unwrap();
            assert_eq!((cfg.dof, cfg.arm_dof(), cfg.gripper_index()), (6, 5, Some(5)));
            let mut q0 = cfg.home_positions.clone();
            let g_idx = cfg.gripper_index().unwrap();
            q0[g_idx] = cfg.gripper_policy.as_ref().unwrap().open_rad;
            let (mut inner, drv) = rig_at(cfg, &q0, vec![0.40, 0.0, 0.0, 0.0, 0.0, 0.15]);
            drv.with(|s| s.include_gravity_torque = false);
            let epoch = inner.safety.epoch_id();
            inner
                .start_gripper(
                    GripperRequest {
                        command: GripperCommand::Close,
                        fraction: None,
                        grasp: false,
                        speed: 1.0,
                        timeout_s: Some(5.0),
                        tag: None,
                    },
                    epoch,
                )
                .unwrap();
            let mut cmds = Vec::new();
            for _ in 0..80 {
                let obs = inner.tick(DT);
                let q_out = drv.with(|s| s.written.last().unwrap()[g_idx].position);
                assert_eq!(q_out.to_bits(), obs.commanded_positions[g_idx].to_bits(), "6-DOF gripper must never get lead");
                cmds.push(q_out);
            }
            cmds
        };
        let cmds_off = run_6dof_jaw(false);
        let cmds_on = run_6dof_jaw(true);
        assert_eq!(cmds_off, cmds_on);
    }
}

/// M3: Revision-tagged escape depths persist across ticks, are evaluated on copies,
/// committed only after a successful write, and reject slight reversals back toward
/// an obstacle (including ratcheting by sub-WORKSPACE_EPS steps).
#[test]
fn tracking_escape_depths_persist_across_ticks() {
    let mut jt = JointTrackingConfig::rebot_dm();
    jt.enabled = true;
    jt.joints[0].lag_s = 0.0; // isolate multi-tick escape bookkeeping cleanly
    let mut cfg = ArmConfig {
        lease_timeout_ms: 600_000,
        joint_tracking: Some(jt),
        ..ArmConfig::rebot_dm()
    };
    cfg.limits[0].max_acceleration = 100.0;
    let q0 = INSPECT;
    // Clear link capsules & extra end points so a single keep-out half-space in z isolates
    // monotonic escape along J2 cleanly.
    let ws_mut = cfg.workspace.as_mut().unwrap();
    ws_mut.check_points_end_m = vec![[0.0, 0.0, 0.0]];
    ws_mut.link_capsules.clear();
    ws_mut.self_collision_enabled = false;
    ws_mut.self_collision_pairs.clear();
    cfg.camera_mount.offset_m = None;
    let p0 = cfg.arm_model.fk_f32(&q0).unwrap().p;
    // Find which J2 direction moves TCP upward (+z).
    let mut q_probe = q0;
    q_probe[1] += 0.05;
    let dir: f32 = if cfg.arm_model.fk_f32(&q_probe).unwrap().p[2] > p0[2] { 1.0 } else { -1.0 };
    cfg.workspace.as_mut().unwrap().keepout_boxes.push(KeepoutBox {
        name: "escape_box".into(),
        min_m: [-1.0, -1.0, -1.0],
        max_m: [1.0, 1.0, p0[2] as f32 + 0.025],
    });
    let ws = cfg.workspace.as_ref().unwrap();
    let d0 = cfg.arm_model.workspace_violation(&q0, ws, &cfg.camera_mount).total;
    assert!(d0 > 0.020, "start pose must be inside keep-out box, depth={d0}");

    cfg.limits[1].max_acceleration = 100.0;
    let (mut inner, _drv) = rig_at(cfg, &q0, vec![0.0; 7]);
    // Step outward across 3 ticks on J2 so intrusion decreases monotonically.
    let jt_ref = inner.config.joint_tracking.as_ref().unwrap().clone();
    let mut prev_depth = d0;
    for step_idx in 1..=3 {
        let mut q_step = q0;
        q_step[1] += dir * 0.008 * step_idx as f32; // 0.40 rad/s <= 0.45 rad/s planned cap
        inner.commanded_positions = q_step.to_vec();
        inner.measured_positions = q_step.to_vec();
        let out = inner.compute_position_lead_output(DT, &jt_ref).expect("escaping step must pass");
        assert!(out.sweep_evals_used <= armd::supervisor::MAX_KEEPOUT_EVALS_PER_TICK);
        let ws = inner.config.workspace.as_ref().unwrap();
        let d_now = inner.config.arm_model.workspace_violation(&out.positions, ws, &inner.config.camera_mount).total;
        assert!(d_now < prev_depth, "escape depth should decrease: {d_now} < {prev_depth}");
        prev_depth = d_now;
        // Commit as if write succeeded.
        inner.tracking.record_issued_with_ws(out.positions, out.velocities, out.ws_min_depths);
    }
    let committed_before = inner.tracking.ws_min_depths.clone().expect("escape depths committed");
    assert_eq!(committed_before.0, inner.workspace_revision);

    // Now reverse slightly back toward the obstacle in sub-WORKSPACE_EPS steps:
    // without persistent escape depths, each tiny reversal would reset min_depths and ratchet inward.
    let q_base = inner.tracking.last_issued_positions.clone().unwrap();
    let mut reversed_rejected = false;
    for k in 1..=10 {
        let mut q_rev = q_base.clone();
        q_rev[1] -= dir * 0.002 * k as f32; // tiny step back toward q0 (~0.5 mm/step)
        inner.commanded_positions = q_rev.clone();
        inner.measured_positions = q_rev;
        match inner.compute_position_lead_output(DT, &jt_ref) {
            Ok(out) => {
                inner.tracking.record_issued_with_ws(out.positions, out.velocities, out.ws_min_depths);
            }
            Err(_) => {
                reversed_rejected = true;
                // On rejection, committed escape depths must not have been mutated.
                assert_eq!(
                    inner.tracking.ws_min_depths.as_ref().unwrap().1.as_slice(),
                    committed_before.1.as_slice()
                );
                break;
            }
        }
    }
    assert!(reversed_rejected, "slight multi-tick reversal toward obstacle must be rejected");
}

/// M2: Measured-lead allowance remains active through stream braking (first stop frame,
/// timeout frame, and every braking tick) and is checked for every multiplier including 0.
#[test]
fn tracking_stream_braking_enforces_measured_lead_allowance() {
    for trigger in ["stop", "timeout"] {
        let mut cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            workspace: None,
            joint_tracking: Some(lead_config(true, 0.68)),
            ..ArmConfig::rebot_dm()
        };
        cfg.servo_max_velocity = Some(vec![Some(0.25), Some(0.25), Some(0.25), Some(1.0), Some(1.0), Some(1.0), None]);
        cfg.following_error.persistence_ms = 5_000;
        let q0 = [0.0_f32; 7];
        let (mut inner, drv) = rig_at(cfg, &q0, vec![0.0; 7]);
        let stream_lead = inner.config.servo_max_lead_rad; // 0.20 rad
        // Pin J1 at 0.0 and stream toward +0.25 until near the boundary with positive velocity.
        drv.with(|s| s.pinned[0] = Some(0.0));
        let epoch = inner.safety.epoch_id();
        let mut reached_near = false;
        for _ in 0..80 {
            let mut tgt = inner.commanded_positions.clone();
            tgt[0] = (tgt[0] + 0.05).min(0.25);
            inner.servo(tgt, epoch).unwrap();
            let obs = inner.tick(DT);
            let q_out = drv.with(|s| s.written.last().unwrap()[0].position);
            assert!(
                q_out <= obs.positions[0] + stream_lead + 1e-4,
                "{trigger}: streaming q_out {q_out} exceeded boundary {}",
                obs.positions[0] + stream_lead
            );
            if inner.commanded_positions[0] >= stream_lead - 0.025 && inner.commanded_velocities[0] > 0.02 {
                reached_near = true;
                break;
            }
        }
        assert!(reached_near, "{trigger}: failed to reach near boundary with positive velocity");
        assert!(inner.servo_stream_active);

        // Trigger stream end while velocity is still positive near the measured-lead boundary.
        if trigger == "stop" {
            inner.stop_motion();
        } else {
            inner.last_motion_frame = std::time::Instant::now() - Duration::from_secs(1);
        }

        // First stop/timeout frame and every braking frame must keep servo_stream_active
        // until velocity hits zero, and never emit q_out past q_m + servo_max_lead_rad.
        let mut saw_braking_with_stream_active = false;
        for k in 0..40 {
            let obs = inner.tick(DT);
            let q_out = drv.with(|s| s.written.last().unwrap()[0].position);
            assert!(
                q_out <= obs.positions[0] + stream_lead + 1e-4,
                "{trigger}: braking tick {k} q_out {q_out} exceeded boundary {}",
                obs.positions[0] + stream_lead
            );
            if inner.commanded_velocities[0].abs() > 0.0 {
                assert!(inner.servo_stream_active, "{trigger}: stream ownership dropped while braking");
                saw_braking_with_stream_active = true;
            } else {
                assert!(!inner.servo_stream_active, "{trigger}: stream ownership should clear after braking ends");
                break;
            }
        }
        assert!(saw_braking_with_stream_active, "{trigger}: expected at least one braking tick with active velocity");
    }

    // Multiplier 0 must also be rejected if q_d itself advances past q_m + servo_max_lead_rad during braking.
    let cfg = ArmConfig {
        lease_timeout_ms: 600_000,
        workspace: None,
        joint_tracking: Some(lead_config(true, 0.68)),
        ..ArmConfig::rebot_dm()
    };
    let q0 = [0.0_f32; 7];
    let (mut inner, _drv) = rig_at(cfg, &q0, vec![0.0; 7]);
    let stream_lead = inner.config.servo_max_lead_rad;
    inner.servo_stream_active = true;
    inner.servo_target = None; // braking
    inner.measured_positions[0] = 0.0;
    inner.commanded_positions[0] = stream_lead + 0.01;
    inner.commanded_velocities[0] = 0.02; // still advancing in + direction past boundary
    inner.tracking.record_issued(vec![stream_lead, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vec![0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    inner.config.limits[0].max_acceleration = 100.0;
    let jt = inner.config.joint_tracking.as_ref().unwrap().clone();
    assert!(
        inner.compute_position_lead_output(DT, &jt).is_err(),
        "multiplier 0 must be rejected when advancing past measured-lead boundary during braking"
    );
}

/// M4: Validation enforces the active motion's output speed cap (planned/speed_cap,
/// servo_max_velocity) and the explicit measured-segment contract (swept keep-out on
/// q_m -> q_cand and following-error envelope).
#[test]
fn tracking_active_limits_and_measured_segment_contract() {
    // 1. Active planned speed_cap constrains output velocity: raw config limit is 2.0 rad/s,
    //    planned_max_velocity is 0.45, and speed_cap.joint_velocity_scale = 0.2 -> cap 0.09 rad/s.
    {
        let mut cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            workspace: None,
            speed_cap: Some(armd::model::SpeedCap {
                linear_mps: None,
                angular_rps: None,
                joint_velocity_scale: Some(0.2),
            }),
            joint_tracking: Some(lead_config(true, 0.68)),
            ..ArmConfig::rebot_dm()
        };
        cfg.limits[0].max_acceleration = 100.0;
        let q0 = [0.0_f32; 7];
        let (mut inner, _drv) = rig_at(cfg, &q0, vec![0.0; 7]);
        // At v_d = 0.015 rad/s, base_lead = 0.0102 rad.
        // Full lead (m=1) over DT=0.02s gives v_cand = 0.0102/0.02 = 0.51 rad/s (> 0.09 cap, < 2.0 raw!).
        // Only m=1/8 gives lead=0.001275 -> v_cand = 0.064 rad/s <= 0.09 cap!
        inner.commanded_velocities[0] = 0.015;
        let jt = inner.config.joint_tracking.as_ref().unwrap().clone();
        let out = inner.compute_position_lead_output(DT, &jt).expect("should scale down to fit active speed cap");
        assert!(out.multiplier < 1.0, "full lead must be rejected by active speed_cap, got m={}", out.multiplier);
        let v_out = ((out.positions[0] - q0[0]) as f64 / DT).abs();
        assert!(v_out <= 0.09 + 1e-4, "issued speed {v_out} exceeded active planned cap 0.09");
    }

    // 2. Measured-segment keep-out contract: prev_q -> q_cand is clear, but q_m -> q_cand cuts a keep-out box.
    {
        let mut cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            joint_tracking: Some(lead_config(true, 0.68)),
            ..ArmConfig::rebot_dm()
        };
        cfg.limits[0].max_acceleration = 100.0;
        let mut q_m = INSPECT;
        let mut q_d = INSPECT;
        q_d[0] += 0.08;
        let mut q_mid = INSPECT;
        q_mid[0] += 0.04;
        let p_mid = cfg.arm_model.fk_f32(&q_mid).unwrap().p;
        // Put a small keep-out box at the midpoint between q_m and q_d (nowhere near q_d itself).
        let hb = 0.003_f32;
        cfg.workspace.as_mut().unwrap().keepout_boxes.push(KeepoutBox {
            name: "meas_chord_box".into(),
            min_m: [p_mid[0] as f32 - hb, p_mid[1] as f32 - hb, p_mid[2] as f32 - hb],
            max_m: [p_mid[0] as f32 + hb, p_mid[1] as f32 + hb, p_mid[2] as f32 + hb],
        });
        let ws = cfg.workspace.as_ref().unwrap();
        assert_eq!(cfg.arm_model.workspace_violation(&q_m, ws, &cfg.camera_mount).total, 0.0);
        assert_eq!(cfg.arm_model.workspace_violation(&q_d, ws, &cfg.camera_mount).total, 0.0);

        let (mut inner, _drv) = rig_at(cfg, &q_m, vec![0.0; 7]);
        // prev_q is already at q_d (so prev_q -> q_d has zero travel and clears keepout),
        // while q_m -> q_d sweeps across meas_chord_box!
        inner.commanded_positions = q_d.to_vec();
        inner.measured_positions = q_m.to_vec();
        inner.tracking.record_issued(q_d.to_vec(), vec![0.0; 7]);
        let jt = inner.config.joint_tracking.as_ref().unwrap().clone();
        assert!(
            inner.compute_position_lead_output(DT, &jt).is_err(),
            "measured segment q_m -> q_cand cutting a keep-out box must be rejected"
        );
        q_m[0] = q_d[0];
        inner.measured_positions = q_m.to_vec();
        assert!(inner.compute_position_lead_output(DT, &jt).is_ok());
    }

    // 3. Active servo speed cap constrains streaming output velocity.
    {
        let mut cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            workspace: None,
            servo_max_velocity: Some(vec![Some(0.10), Some(0.45), Some(0.45), Some(1.5), Some(1.5), Some(1.5), None]),
            joint_tracking: Some(lead_config(true, 0.68)),
            ..ArmConfig::rebot_dm()
        };
        cfg.limits[0].max_acceleration = 100.0;
        let q0 = [0.0_f32; 7];
        let (mut inner, _drv) = rig_at(cfg, &q0, vec![0.0; 7]);
        inner.servo_target = Some(vec![0.10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        inner.servo_stream_active = true;
        inner.commanded_velocities[0] = 0.02; // base_lead = 0.0136 -> full lead v = 0.68 > 0.10 servo cap
        let jt = inner.config.joint_tracking.as_ref().unwrap().clone();
        let out = inner.compute_position_lead_output(DT, &jt).expect("should scale lead to fit servo cap");
        assert!(out.multiplier < 1.0, "full lead must be rejected by servo_max_velocity, got m={}", out.multiplier);
        let v_out = ((out.positions[0] - q0[0]) as f64 / DT).abs();
        assert!(v_out <= 0.10 + 1e-4, "streaming speed {v_out} exceeded servo_max_velocity 0.10");
    }
}

/// B1: Real DamiaoArmDriver + CanTransport fake with one joint's RX delayed past 45 ms:
/// autonomous POS_VEL recovery writes are visible to tracking, suspend compensated writes
/// so the motor never receives q_m followed immediately by q_out, and resume only after
/// confirmed fresh post-recovery feedback.
#[test]
fn tracking_damiao_recovery_suspends_compensated_writes() {
    use std::collections::VecDeque;
    use armd::driver::DamiaoArmDriver;
    use armd::protocol::damiao::{self, DamiaoLimits};
    use armd::transport::{CanFrame, CanTransport, TransportError};

    struct FakeDamiaoBusState {
        q: [f32; 7],
        enabled: [bool; 7],
        limits: [DamiaoLimits; 7],
        /// When true for joint i, suppresses auto-reply feedback for motor i+1.
        drop_rx: [bool; 7],
        rx_queue: VecDeque<CanFrame>,
        tx_log: Vec<CanFrame>,
    }

    impl FakeDamiaoBusState {
        fn push_feedback(&mut self, i: usize) {
            let lim = &self.limits[i];
            let p_u = damiao::float_to_uint(self.q[i], lim.p_min, lim.p_max, 16);
            let v_u = damiao::float_to_uint(0.0, lim.v_min, lim.v_max, 12);
            let t_u = damiao::float_to_uint(0.0, lim.t_min, lim.t_max, 12);
            let id = (i + 1) as u8;
            let state = if self.enabled[i] { 1u8 } else { 0u8 };
            let data = [
                (state << 4) | (id & 0x0f),
                (p_u >> 8) as u8,
                (p_u & 0xff) as u8,
                (v_u >> 4) as u8,
                (((v_u & 0x0f) << 4) | ((t_u >> 8) & 0x0f)) as u8,
                (t_u & 0xff) as u8,
                30,
                30,
            ];
            self.rx_queue.push_back(CanFrame::new_standard(0x10 + id as u16, &data).unwrap());
        }
    }

    #[derive(Clone)]
    struct FakeDamiaoCan(Arc<Mutex<FakeDamiaoBusState>>);

    impl CanTransport for FakeDamiaoCan {
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
            let mut s = self.0.lock().unwrap();
            s.tx_log.push(*frame);
            if frame.data == [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC] && (1..=7).contains(&frame.id) {
                let i = (frame.id - 1) as usize;
                s.enabled[i] = true;
                if !s.drop_rx[i] { s.push_feedback(i); }
            } else if frame.data == [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD] && (1..=7).contains(&frame.id) {
                let i = (frame.id - 1) as usize;
                s.enabled[i] = false;
                if !s.drop_rx[i] { s.push_feedback(i); }
            } else if (0x101..=0x107).contains(&frame.id) {
                let i = (frame.id - 0x101) as usize;
                let pos = f32::from_le_bytes([frame.data[0], frame.data[1], frame.data[2], frame.data[3]]);
                // First-order lag on J1 (tau = 0.68 s), instant on others.
                if i == 0 {
                    let alpha = 1.0 - (-DT / 0.68_f64).exp();
                    s.q[0] = (f64::from(s.q[0]) + (f64::from(pos) - f64::from(s.q[0])) * alpha) as f32;
                } else {
                    s.q[i] = pos;
                }
                if !s.drop_rx[i] { s.push_feedback(i); }
            } else if frame.id == 0x7FF && frame.data[2] == 0xCC {
                let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
                if (1..=7).contains(&id) {
                    let i = (id - 1) as usize;
                    if !s.drop_rx[i] { s.push_feedback(i); }
                }
            }
            Ok(())
        }
        fn receive(&mut self) -> Result<CanFrame, TransportError> {
            self.0.lock().unwrap().rx_queue.pop_front().ok_or(TransportError::WouldBlock)
        }
        fn flush(&mut self) -> Result<(), TransportError> { Ok(()) }
    }

    let cfg = ArmConfig {
        lease_timeout_ms: 600_000,
        workspace: None,
        joint_tracking: Some(lead_config(true, 0.68)),
        ..ArmConfig::rebot_dm()
    };
    let limits: [DamiaoLimits; 7] = std::array::from_fn(|i| cfg.damiao_ranges.as_ref().unwrap()[i].limits());
    let bus_state = Arc::new(Mutex::new(FakeDamiaoBusState {
        q: [0.0; 7],
        enabled: [false; 7],
        limits,
        drop_rx: [false; 7],
        rx_queue: VecDeque::new(),
        tx_log: Vec::new(),
    }));
    let drv = DamiaoArmDriver::new(FakeDamiaoCan(bus_state.clone()), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits.to_vec())
        .unwrap();
    let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
    inner.lease_enabled = false;
    inner.set_torque(true).unwrap();
    inner.tick(DT);

    // Start smooth J1 quintic so active lead is being transmitted on 0x101.
    let traj = TrajectoryPlanner::plan(
        &[0.0; 7],
        &[0.0; 7],
        &[0.08, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        &[0.0; 7],
        Some(5.5),
        &inner.config.limits,
    )
    .unwrap();
    let _ = inner.safety.mark_motion_started();
    inner.active_trajectory = Some(armd::motion::ActiveTrajectory::quintic(traj, None));
    for _ in 0..100 {
        inner.tick(DT);
    }
    let q_d_before = inner.commanded_positions[0];
    let q_m_before = inner.measured_positions[0];
    let last_j1_pos = bus_state
        .lock()
        .unwrap()
        .tx_log
        .iter()
        .rev()
        .find(|f| f.id == 0x101)
        .map(|f| f32::from_le_bytes([f.data[0], f.data[1], f.data[2], f.data[3]]))
        .unwrap();
    assert!(last_j1_pos - q_d_before > 0.010, "expected active lead before delay, got {}", last_j1_pos - q_d_before);

    // Delay ONLY motor 1's RX past Damiao's 45 ms recovery threshold while keeping motors 2..7 fresh!
    bus_state.lock().unwrap().drop_rx[0] = true;
    for _ in 0..6 {
        std::thread::sleep(Duration::from_millis(20));
        // Keep J2..J7 fresh by running ticks until J1 crosses 45 ms.
        bus_state.lock().unwrap().tx_log.clear();
        inner.tick(DT);
        if inner.tracking.recovery_suspended {
            break;
        }
    }
    assert!(inner.tracking.recovery_suspended, "recovery write on J1 must suspend tracking compensation");

    // Inspect EVERY transmitted frame on the recovery tick:
    // J1 (0x101) must have received ONLY the recovery POS_VEL hold at cached q_m, NEVER followed by q_out!
    let recovery_tx: Vec<CanFrame> = bus_state.lock().unwrap().tx_log.clone();
    let j1_frames: Vec<f32> = recovery_tx
        .iter()
        .filter(|f| f.id == 0x101)
        .map(|f| f32::from_le_bytes([f.data[0], f.data[1], f.data[2], f.data[3]]))
        .collect();
    assert_eq!(j1_frames.len(), 1, "expected exactly 1 recovery frame on 0x101 (no immediate q_out), got {j1_frames:?}");
    assert!(
        (j1_frames[0] - q_m_before).abs() < 0.01,
        "recovery frame must hold cached q_m (~{q_m_before:.4}), not compensated q_out ({last_j1_pos:.4}): got {}",
        j1_frames[0]
    );
    assert!(
        !recovery_tx.iter().any(|f| (0x102..=0x107).contains(&f.id)),
        "compensated supervisor writes must be suspended during recovery"
    );

    // Release J1's RX and push a fresh feedback frame for every joint -> confirmed fresh baseline.
    {
        let mut s = bus_state.lock().unwrap();
        s.drop_rx[0] = false;
        s.tx_log.clear();
        for i in 0..7 {
            s.push_feedback(i);
        }
    }
    inner.tick(DT);
    assert!(!inner.tracking.recovery_suspended, "fresh post-recovery feedback should resume from confirmed baseline");
    let resumed_tx: Vec<CanFrame> = bus_state.lock().unwrap().tx_log.clone();
    let j1_resumed = resumed_tx
        .iter()
        .find(|f| f.id == 0x101)
        .map(|f| f32::from_le_bytes([f.data[0], f.data[1], f.data[2], f.data[3]]))
        .expect("expected resumed POS_VEL frame on 0x101");
    let q_m_now = inner.measured_positions[0];
    assert!(
        (j1_resumed - q_m_now).abs() <= 1e-4,
        "first resumed frame must match confirmed fresh baseline {q_m_now:.4}, got {j1_resumed:.4}"
    );
}
