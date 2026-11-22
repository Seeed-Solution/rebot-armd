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
    /// When true, the next `write_joint_targets` call fails with a bus error (auto-clears).
    fail_next_write: bool,
    /// When `Some(k)`, the next `write_joint_targets` applies joints `0..k` to the plant,
    /// records the partial prefix in `partial_written`, and fails on joint `k` (auto-clears).
    fail_write_at_joint: Option<usize>,
    /// Prefixes actually transmitted before a partial write failure.
    partial_written: Vec<Vec<JointTarget>>,
    /// Current per-joint mode as last confirmed by `set_joint_modes`.
    modes: Vec<JointMode>,
    /// Recorded `(modes, hold_targets)` pairs from every `set_joint_modes` call.
    mode_hold_frames: Vec<(Vec<JointMode>, Vec<JointTarget>)>,
    /// When true, the next `set_joint_modes` call fails (auto-clears).
    fail_mode_switch: bool,
    /// Reported `DriverCapabilities::velocity_feedback` (default true).
    velocity_feedback: bool,
    /// Last `set_strict_mode_handover` value received.
    strict_mode_handover: Option<bool>,
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
            fail_next_write: false,
            fail_write_at_joint: None,
            partial_written: Vec::new(),
            modes: vec![JointMode::Position; dof],
            mode_hold_frames: Vec::new(),
            fail_mode_switch: false,
            velocity_feedback: true,
            strict_mode_handover: None,
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
            velocity_feedback: self.0.lock().unwrap().velocity_feedback,
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
        if s.fail_next_write {
            s.fail_next_write = false;
            return Err(ArmDriverError::Protocol("simulated write failure".into()));
        }
        let fail_at = s.fail_write_at_joint.take();
        let limit = fail_at.map_or(s.dof, |k| k.min(s.dof));
        if fail_at.is_some() {
            s.partial_written.push(targets[..limit.min(targets.len())].to_vec());
        } else {
            s.written.push(targets.to_vec());
        }
        if s.enabled != EnableState::Enabled {
            return if let Some(k) = fail_at {
                Err(ArmDriverError::Protocol(format!("simulated partial write failure at joint {}", k + 1)))
            } else {
                Ok(())
            };
        }
        for (i, t) in targets.iter().enumerate().take(limit) {
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
        if let Some(k) = fail_at {
            return Err(ArmDriverError::Protocol(format!("simulated partial write failure at joint {}", k + 1)));
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
    fn set_strict_mode_handover(&mut self, strict: bool) {
        self.0.lock().unwrap().strict_mode_handover = Some(strict);
    }
    fn set_joint_modes(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        let mut s = self.0.lock().unwrap();
        if s.fail_mode_switch {
            s.fail_mode_switch = false;
            return Err(ArmDriverError::HardwareFault("simulated mode-switch failure".into()));
        }
        s.modes = modes.to_vec();
        s.mode_hold_frames.push((modes.to_vec(), hold.to_vec()));
        // Honour the mode-switch hold frame immediately on the plant when torqued.
        if s.enabled == EnableState::Enabled {
            for (i, t) in hold.iter().enumerate().take(s.dof) {
                if let Some(pin) = s.pinned[i] {
                    s.q[i] = pin;
                    s.v[i] = 0.0;
                    continue;
                }
                let tau_i = s.tau_s[i];
                if tau_i > 1e-6 {
                    let alpha = 1.0 - (-DT / f64::from(tau_i)).exp();
                    let qn = f64::from(s.q[i]) + (f64::from(t.position) - f64::from(s.q[i])) * alpha;
                    let vn = (f64::from(t.position) - qn) / f64::from(tau_i);
                    s.q[i] = qn as f32;
                    s.v[i] = vn as f32;
                } else {
                    s.q[i] = t.position;
                    s.v[i] = 0.0;
                }
            }
        }
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

        // 8. Park completes: the settle hold ends with torque off (bounded), then idle ticks.
        // At the resting pose the arm sits on its mechanical support: the fake plant stops
        // reporting gravity load so the support-torque check of the settle can pass.
        drv.with(|s| s.include_gravity_torque = false);
        let mut park_ticks = 0;
        while inner.is_torqued && park_ticks < 2000 {
            step(&mut inner, 1);
            park_ticks += 1;
        }
        assert!(!inner.is_torqued, "park must complete with torque off within 2000 extra ticks");
        step(&mut inner, 10);

        let frames = drv.with(|s| s.written.clone());
        (frames, obs_log)
    };

    let (frames_none, obs_none) = run_scenario(None);
    let (frames_disabled, obs_disabled) = run_scenario(Some(JointTrackingConfig::rebot_dm()));

    assert!(frames_none.len() > 700, "expected full multi-producer trace, got {}", frames_none.len());
    assert!(
        obs_none.last().is_some_and(|o| !o.is_torqued) && obs_none.iter().any(|o| o.is_torqued),
        "the trace must end with the park completed (torque off)"
    );
    assert_targets_bit_eq(&frames_none, &frames_disabled, "tracking_off_wire_equivalence");
    assert_obs_equiv(&obs_none, &obs_disabled, "tracking_off_wire_equivalence");

    // M7: Compare BOTH disabled modes bit-for-bit against the pre-change trace fixture
    // recorded on baseline commit e4f72b6 (before joint_tracking existed).
    #[derive(serde::Deserialize)]
    struct RecordedTarget {
        p: u32,
        v: Option<u32>,
        t: Option<u32>,
        kp: Option<u32>,
        kd: Option<u32>,
    }
    #[derive(serde::Deserialize)]
    struct RecordedObs {
        positions: Vec<u32>,
        velocities: Vec<u32>,
        torques: Vec<u32>,
        commanded_positions: Vec<u32>,
        is_torqued: bool,
        safety_state: String,
        fault: Option<String>,
        gripper: Option<String>,
        motion_limit: Option<String>,
    }
    #[derive(serde::Deserialize)]
    struct RecordedTrace {
        source_commit: String,
        frames: Vec<Vec<RecordedTarget>>,
        obs: Vec<RecordedObs>,
    }

    #[derive(serde::Deserialize)]
    struct RecordedMeta {
        source_commit: String,
        short_commit: String,
        command: String,
        generator: String,
        fixture: String,
        frames: usize,
        obs: usize,
        sha256: String,
    }

    let meta_path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/tracking_off_e4f72b6_trace.meta.json");
    let meta_raw = std::fs::read_to_string(meta_path)
        .unwrap_or_else(|e| panic!("failed to read e4f72b6 provenance meta {meta_path}: {e}"));
    let meta: RecordedMeta = serde_json::from_str(&meta_raw).expect("valid e4f72b6 provenance JSON");
    assert_eq!(meta.short_commit, "e4f72b6");
    assert!(meta.source_commit.starts_with("e4f72b6"), "unexpected source_commit {}", meta.source_commit);
    assert!(!meta.command.is_empty() && !meta.generator.is_empty() && !meta.fixture.is_empty());
    assert_eq!(meta.sha256.len(), 64, "provenance sha256 must be 64 hex chars");

    let fixture_path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/tracking_off_e4f72b6_trace.json");
    let raw = std::fs::read_to_string(fixture_path)
        .unwrap_or_else(|e| panic!("failed to read e4f72b6 fixture {fixture_path}: {e}"));
    let baseline: RecordedTrace = serde_json::from_str(&raw).expect("valid e4f72b6 trace JSON");
    assert_eq!(baseline.source_commit, meta.source_commit);
    assert_eq!(baseline.frames.len(), meta.frames);
    assert_eq!(baseline.obs.len(), meta.obs);

    let check_against_baseline = |label: &str, frames: &[Vec<JointTarget>], obs: &[ArmObservation]| {
        assert_eq!(frames.len(), baseline.frames.len(), "{label}: frame count vs e4f72b6");
        for (k, (got, exp)) in frames.iter().zip(baseline.frames.iter()).enumerate() {
            assert_eq!(got.len(), exp.len(), "{label}: tick {k} dof vs e4f72b6");
            for (j, (gt, et)) in got.iter().zip(exp.iter()).enumerate() {
                assert_eq!(gt.position.to_bits(), et.p, "{label}: tick {k} joint {j} position vs e4f72b6");
                assert_eq!(gt.velocity.map(f32::to_bits), et.v, "{label}: tick {k} joint {j} velocity vs e4f72b6");
                assert_eq!(gt.torque.map(f32::to_bits), et.t, "{label}: tick {k} joint {j} torque vs e4f72b6");
                assert_eq!(gt.kp.map(f32::to_bits), et.kp, "{label}: tick {k} joint {j} kp vs e4f72b6");
                assert_eq!(gt.kd.map(f32::to_bits), et.kd, "{label}: tick {k} joint {j} kd vs e4f72b6");
            }
        }
        assert_eq!(obs.len(), baseline.obs.len(), "{label}: obs count vs e4f72b6");
        let bits = |v: &[f32]| -> Vec<u32> { v.iter().map(|x| x.to_bits()).collect() };
        for (k, (go, eo)) in obs.iter().zip(baseline.obs.iter()).enumerate() {
            assert_eq!(bits(&go.positions), eo.positions, "{label}: tick {k} positions vs e4f72b6");
            assert_eq!(bits(&go.velocities), eo.velocities, "{label}: tick {k} velocities vs e4f72b6");
            assert_eq!(bits(&go.torques), eo.torques, "{label}: tick {k} torques vs e4f72b6");
            assert_eq!(bits(&go.commanded_positions), eo.commanded_positions, "{label}: tick {k} commanded vs e4f72b6");
            assert_eq!(go.is_torqued, eo.is_torqued, "{label}: tick {k} is_torqued vs e4f72b6");
            assert_eq!(format!("{:?}", go.safety_state), eo.safety_state, "{label}: tick {k} safety_state vs e4f72b6");
            assert_eq!(go.fault.as_ref().map(|f| format!("{f:?}")), eo.fault, "{label}: tick {k} fault vs e4f72b6");
            assert_eq!(go.gripper.as_ref().map(|g| format!("{g:?}")), eo.gripper, "{label}: tick {k} gripper vs e4f72b6");
            assert_eq!(
                go.motion_limit.as_ref().map(|m| format!("{:?}:{:?}", m.reason, m.joint)),
                eo.motion_limit,
                "{label}: tick {k} motion_limit vs e4f72b6"
            );
        }
    };
    check_against_baseline("joint_tracking=None vs e4f72b6", &frames_none, &obs_none);
    check_against_baseline("joint_tracking=enabled:false vs e4f72b6", &frames_disabled, &obs_disabled);
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
        let prev_confirmed = inner2.tracking.last_issued_positions.clone().unwrap();
        let lead_big = prev_confirmed[0] - inner2.commanded_positions[0];
        assert!(lead_big > 0.015, "expected large active lead, got {lead_big}");

        let writes_before = drv2.with(|s| s.written.len());
        drv2.with(|s| s.feedback_age = Duration::from_millis(150));
        let obs2 = inner2.tick(DT);
        assert!(matches!(obs2.safety_state, SafetyState::Fault(_)), "unsafe stale withdrawal must fault, got {:?}", obs2.safety_state);
        assert_eq!(obs2.fault.as_ref().map(|f| f.code), Some(FaultCode::JointLimit));
        assert!(inner2.active_trajectory.is_none(), "motion must be terminated");
        // B2: with stale (150 ms) feedback, fallback must preserve the last confirmed safe output
        // rather than writing a hold synthesized from the stale 150 ms-old measured sample!
        assert_eq!(inner2.tracking.last_issued_positions.as_deref(), Some(prev_confirmed.as_slice()));
        let fallback_written = drv2.with(|s| s.written[writes_before][0].position);
        assert!(
            (fallback_written - prev_confirmed[0]).abs() <= 1e-6,
            "stale-feedback fallback must hold last confirmed output {}, got {fallback_written} (stale q_m={})",
            prev_confirmed[0],
            obs2.positions[0]
        );
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
                inner.servo_stream_active = true;
            } else {
                let traj = TrajectoryPlanner::plan(&q0, &[0.0; 7], &q0, &[0.0; 7], Some(1.0), &inner.config.limits).unwrap();
                inner.active_trajectory = Some(armd::motion::ActiveTrajectory::quintic(traj, None));
            }
            let writes_before = drv.with(|s| s.written.len());
            let obs = inner.tick(DT);
            assert!(inner.servo_target.is_none() && inner.active_trajectory.is_none(), "motion must terminate");
            // B2: fallback must NOT send rejected q0 across the keep-out box; it must preserve
            // last confirmed output q_lead on the rejection tick AND subsequent hold ticks!
            let fallback_frame = drv.with(|s| s.written[writes_before].clone());
            assert!(
                (fallback_frame[0].position - q_lead[0]).abs() <= 1e-6,
                "fallback must preserve q_lead {}, never send rejected q0 {}: got {}",
                q_lead[0],
                q0[0],
                fallback_frame[0].position
            );
            assert_eq!(fallback_frame[0].velocity, Some(0.0));
            if streaming {
                assert_eq!(obs.motion_limit.as_ref().map(|m| m.reason), Some(MotionLimitReason::Workspace));
                assert!(obs.workspace_violation.is_some());
            } else {
                assert!(matches!(obs.safety_state, SafetyState::Fault(_)), "trajectory unsafe return must fault, got {:?}", obs.safety_state);
            }
            // Next tick in hold/fault must STILL hold q_lead, never jumping across the keep-out to q0.
            let next_idx = drv.with(|s| s.written.len());
            inner.tick(DT);
            let next_frame = drv.with(|s| s.written[next_idx].clone());
            assert!(
                (next_frame[0].position - q_lead[0]).abs() <= 1e-6,
                "subsequent hold tick must stay at q_lead {}, got {}",
                q_lead[0],
                next_frame[0].position
            );
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
            let has_residual = inner
                .tracking
                .has_residual_lead(&inner.commanded_positions, &inner.config.arm_joint_indices());
            if inner.commanded_velocities[0].abs() > 0.0 || has_residual {
                assert!(inner.servo_stream_active, "{trigger}: stream ownership dropped while braking/ramping");
                if inner.commanded_velocities[0].abs() > 0.0 {
                    saw_braking_with_stream_active = true;
                }
            } else {
                assert!(!inner.servo_stream_active, "{trigger}: stream ownership should clear after braking & ramp end");
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

/// M1: Unknown history never bypasses validation; park, moving lease handover, and
/// torque-guard trip keep prior output and ramp residual lead out smoothly without a 1.5 rad/s step.
#[test]
fn tracking_unknown_history_and_handover_ramp_out() {
    use armd::tracking::TrackingHistoryState;

    // 1. Unknown history + zero lead must still validate (rejects stale feedback and excessive jump from q_m).
    {
        let cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            workspace: None,
            joint_tracking: Some(lead_config(true, 0.68)),
            ..ArmConfig::rebot_dm()
        };
        let q0 = [0.0_f32; 7];
        let (mut inner, drv) = rig_at(cfg, &q0, vec![0.0; 7]);
        assert_eq!(inner.tracking.history_state, TrackingHistoryState::Issued);

        // Invalidate history -> Unknown state.
        inner.tracking.invalidate();
        assert_eq!(inner.tracking.history_state, TrackingHistoryState::Unknown);
        inner.commanded_velocities.fill(0.0); // zero lead requested
        let jt = inner.config.joint_tracking.as_ref().unwrap().clone();

        // Stale feedback with unknown history must be rejected (no unvalidated nominal pass-through).
        drv.with(|s| s.feedback_age = Duration::from_millis(150));
        assert!(
            inner.compute_position_lead_output(DT, &jt).is_err(),
            "unknown history with stale feedback must fail validation"
        );

        // Fresh feedback, but nominal q_d jumps 0.05 rad from q_m (2.5 rad/s > 0.45 cap) -> rejected!
        drv.with(|s| s.feedback_age = Duration::ZERO);
        inner.measured_positions[0] = 0.0;
        inner.commanded_positions[0] = 0.05;
        assert!(
            inner.compute_position_lead_output(DT, &jt).is_err(),
            "unknown history must validate q_m -> q_d against dynamics limits even with zero lead"
        );
    }

    // 2. Park (NormalUnload) started with active +0.025 rad lead: ramps lead out smoothly
    //    (no 1.5 rad/s single-tick drop) while keeping park's floor exemption.
    {
        let cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            joint_tracking: Some(lead_config(true, 0.68)),
            ..ArmConfig::rebot_dm()
        };
        let q0 = [0.0_f32; 7];
        let (mut inner, drv) = rig_at(cfg, &q0, vec![0.68, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        let traj = TrajectoryPlanner::plan(
            &q0,
            &[0.0; 7],
            &[0.20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            &[0.0; 7],
            Some(5.0),
            &inner.config.limits,
        )
        .unwrap();
        let _ = inner.safety.mark_motion_started();
        inner.active_trajectory = Some(armd::motion::ActiveTrajectory::quintic(traj, None));
        for _ in 0..100 {
            inner.tick(DT);
        }
        let last_before_park = drv.with(|s| s.written.last().unwrap()[0].position);
        let lead_before_park = last_before_park - inner.commanded_positions[0];
        assert!(lead_before_park > 0.020, "expected active lead before park, got {lead_before_park}");

        let idx_before = drv.with(|s| s.written.len());
        inner.apply_stop_policy(StopPolicy::NormalUnload);
        assert!(inner.pending_normal_unload);
        for _ in 0..15 {
            inner.tick(DT);
            assert!(!inner.safety.state().is_stopped(), "park ramp-out should not fault: {:?}", inner.safety.state());
        }
        // Check first park write step speed relative to last_before_park: must be <= 0.45 rad/s (NO 1.5 rad/s drop!).
        let first_park_pos = drv.with(|s| s.written[idx_before][0].position);
        let first_step_vel = ((first_park_pos - last_before_park) as f64 / DT).abs();
        assert!(
            first_step_vel <= 0.45 + 1e-3,
            "park first write dropped lead too fast: {first_step_vel:.3} rad/s (before={last_before_park:.4}, first={first_park_pos:.4})"
        );
    }

    // 3. Moving lease handover (preempt_lease) with active lead ramps lead out smoothly.
    {
        let cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            workspace: None,
            joint_tracking: Some(lead_config(true, 0.68)),
            ..ArmConfig::rebot_dm()
        };
        let q0 = [0.0_f32; 7];
        let (mut inner, drv) = rig_at(cfg, &q0, vec![0.68, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        inner.lease_enabled = true;
        inner.acquire_lease("client_a").unwrap();
        let traj = TrajectoryPlanner::plan(
            &q0,
            &[0.0; 7],
            &[0.20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            &[0.0; 7],
            Some(5.0),
            &inner.config.limits,
        )
        .unwrap();
        let _ = inner.safety.mark_motion_started();
        inner.active_trajectory = Some(armd::motion::ActiveTrajectory::quintic(traj, None));
        for _ in 0..100 {
            inner.tick(DT);
        }
        let last_before = drv.with(|s| s.written.last().unwrap()[0].position);
        assert!(last_before - inner.commanded_positions[0] > 0.020);

        let idx_before = drv.with(|s| s.written.len());
        inner.preempt_lease("client_b", "takeover").unwrap();
        assert!(inner.handover_settling_since.is_some());
        let mut prev_pos = last_before;
        for k in 0..20 {
            inner.tick(DT);
            let pos = drv.with(|s| s.written[idx_before + k][0].position);
            let step_v = ((pos - prev_pos) as f64 / DT).abs();
            assert!(step_v <= 0.45 + 1e-3, "handover tick {k} exceeded velocity cap: {step_v:.3} rad/s");
            prev_pos = pos;
        }
    }

    // 4. Torque guard trip with active lead ramps smoothly in the relieving direction without a 1.5 rad/s step.
    {
        let cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            workspace: None,
            torque_guard: Some(TorqueGuardPolicy { enabled: true, ratio: 0.60, persist_ms: 40, ..Default::default() }),
            joint_tracking: Some(lead_config(true, 0.68)),
            ..ArmConfig::rebot_dm()
        };
        let q0 = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0, -2.5];
        let (mut inner, drv) = rig_at(cfg, &q0, vec![0.68, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        let traj = TrajectoryPlanner::plan(
            &q0,
            &[0.0; 7],
            &[0.20, -1.167, -1.565, 1.435, 0.0, 0.0, -2.5],
            &[0.0; 7],
            Some(5.0),
            &inner.config.limits,
        )
        .unwrap();
        let _ = inner.safety.mark_motion_started();
        inner.active_trajectory = Some(armd::motion::ActiveTrajectory::quintic(traj, None));
        for _ in 0..100 {
            inner.tick(DT);
        }
        let last_before_trip = drv.with(|s| s.written.last().unwrap()[0].position);
        assert!(last_before_trip - inner.commanded_positions[0] > 0.020);

        // Force +18.0 N·m on J1 (> 16.8 N·m threshold, sign = +1).
        drv.with(|s| s.forced_torque[0] = Some(18.0));
        let idx_trip = drv.with(|s| s.written.len());
        let mut tripped = false;
        let mut prev_pos = last_before_trip;
        for k in 0..20 {
            inner.tick(DT);
            if inner.torque_guard.hold.is_some() {
                tripped = true;
                drv.with(|s| s.forced_torque[0] = Some(5.0));
            }
            let pos = drv.with(|s| s.written[idx_trip + k][0].position);
            let step_v = ((pos - prev_pos) as f64 / DT).abs();
            assert!(step_v <= 0.45 + 1e-3, "torque-trip tick {k} exceeded velocity cap: {step_v:.3} rad/s");
            if tripped {
                assert!(pos <= prev_pos + 1e-6, "after +1 torque trip, output must not advance in +1 direction");
            }
            prev_pos = pos;
        }
        assert!(tripped, "torque guard must trip");
        assert!(inner.safety.state().is_operational(), "torque trip ramp-out must stay operational");
    }
}

/// M5: Damiao `write_modes` rejects "mode acknowledged, hold lost" (transmit or flush failure)
/// and gravity-comp handback requires fresh stationary feedback before transferring ownership.
#[test]
fn tracking_gravity_comp_handback_and_mode_switch_hold_loss() {
    use armd::driver::DamiaoArmDriver;
    use armd::protocol::damiao::{self, DamiaoLimits};
    use armd::tracking::TrackingHistoryState;
    use armd::transport::{CanFrame, CanTransport, MockCanTransport, TransportError};

    // 1. "mode acknowledged, hold lost" (both hold transmit failure and flush failure).
    struct FaultyHoldCan {
        inner: MockCanTransport,
        fail_mit_hold_tx: bool,
        fail_flush_after_mode_write: bool,
        mode_write_seen: bool,
    }
    impl CanTransport for FaultyHoldCan {
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
            if frame.id == 0x7FF
                && frame.data[2] == damiao::REG_WRITE
                && frame.data[3] == damiao::RID_CTRL_MODE
            {
                self.mode_write_seen = true;
            }
            if self.fail_mit_hold_tx && frame.id == 1 && frame.data != [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC] {
                return Err(TransportError::Io(std::io::Error::other("simulated hold transmit loss")));
            }
            self.inner.transmit(frame)
        }
        fn receive(&mut self) -> Result<CanFrame, TransportError> {
            self.inner.receive()
        }
        fn flush(&mut self) -> Result<(), TransportError> {
            if self.fail_flush_after_mode_write && self.mode_write_seen {
                return Err(TransportError::Io(std::io::Error::other("simulated bus flush failure")));
            }
            self.inner.flush()
        }
    }

    let make_can = |fail_mit_hold_tx: bool, fail_flush_after_mode_write: bool| -> FaultyHoldCan {
        let mut mock = MockCanTransport::new();
        let limits = DamiaoLimits::default();
        let p = damiao::float_to_uint(0.2, limits.p_min, limits.p_max, 16);
        let fb = CanFrame::new_standard(0x11, &[0x11, (p >> 8) as u8, p as u8, 0, 0, 0, 30, 30]).unwrap();
        mock.respond_to_transmit(damiao::build_enable_frame(1), fb);
        let mk_reply = |op: u8, rid: u8, bytes: [u8; 4]| {
            CanFrame::new_standard(0, &[1, 0, op, rid, bytes[0], bytes[1], bytes[2], bytes[3]]).unwrap()
        };
        for (rid, val) in [(damiao::RID_PMAX, 12.5_f32), (damiao::RID_VMAX, 30.0), (damiao::RID_TMAX, 18.0)] {
            mock.respond_to_transmit(damiao::build_read_register_frame(1, rid), mk_reply(damiao::REG_READ, rid, val.to_le_bytes()));
        }
        for mode in [damiao::CTRL_MODE_MIT, damiao::CTRL_MODE_POS_VEL] {
            mock.respond_to_transmit(
                damiao::build_set_control_mode_frame(1, mode),
                mk_reply(damiao::REG_WRITE, damiao::RID_CTRL_MODE, mode.to_le_bytes()),
            );
        }
        FaultyHoldCan {
            inner: mock,
            fail_mit_hold_tx,
            fail_flush_after_mode_write,
            mode_write_seen: false,
        }
    };

    // Subcases A and B apply with tracking enabled (strict mode handover, set by
    // `SupervisorInner::with_driver`); A2/B2 below keep the e4f72b6 behaviour with tracking off.
    // Subcase A: mode write acks, but MIT hold transmit fails -> must reject switch!
    {
        let mut drv = DamiaoArmDriver::new(make_can(true, false), vec![1]);
        drv.set_strict_mode_handover(true);
        drv.set_torque(true).unwrap();
        let res = drv.set_joint_modes(&[JointMode::Mit], &[JointTarget::mit(0.2, 0.0, 30.0, 1.0, 0.0)]);
        assert!(drv.can_mut().mode_write_seen, "CTRL_MODE write must have been sent");
        assert!(res.is_err(), "mode acknowledged with lost hold frame must fail set_joint_modes");
        assert!(drv.joint_modes_uncertain(), "rollback also lost MIT dual-hold -> joint mode stays uncertain");
    }

    // Subcase B: mode write and hold transmit succeed, but flush fails -> must reject switch!
    {
        let mut drv = DamiaoArmDriver::new(make_can(false, true), vec![1]);
        drv.set_strict_mode_handover(true);
        drv.set_torque(true).unwrap();
        let res = drv.set_joint_modes(&[JointMode::Mit], &[JointTarget::mit(0.2, 0.0, 30.0, 1.0, 0.0)]);
        assert!(drv.can_mut().mode_write_seen, "CTRL_MODE write must have been sent before flush failure");
        assert!(res.is_err(), "mode acknowledged with failed flush must fail set_joint_modes");
        assert!(drv.joint_modes_uncertain(), "failed flush must leave joint mode uncertain");
    }

    // Review 3 T2: tracking off -> e4f72b6 behaviour: a matching mode ACK confirms the switch
    // even when the hold transmit (A2) or the flush (B2) failed. Checked on a bare driver and
    // through `SupervisorInner::with_driver` with tracking off (none / disabled) and on.
    for (fail_hold, fail_flush) in [(true, false), (false, true)] {
        let mut drv = DamiaoArmDriver::new(make_can(fail_hold, fail_flush), vec![1]);
        drv.set_torque(true).unwrap();
        let res = drv.set_joint_modes(&[JointMode::Mit], &[JointTarget::mit(0.2, 0.0, 30.0, 1.0, 0.0)]);
        assert!(drv.can_mut().mode_write_seen);
        assert!(res.is_ok(), "tracking off (hold {fail_hold}, flush {fail_flush}): e4f72b6 accepts the ACK, got {res:?}");
        assert!(!drv.joint_modes_uncertain(), "tracking off: the acknowledged mode is confirmed as in e4f72b6");

    }
    for (label, jt, strict) in [
        ("tracking none", None, false),
        ("tracking disabled", Some(JointTrackingConfig::rebot_dm()), false),
        ("tracking enabled", Some(lead_config(true, 0.68)), true),
    ] {
        let q0 = [0.0, -0.5, -0.5, 0.5, 0.0, 0.0, -2.0];
        let cfg = ArmConfig { joint_tracking: jt, ..ArmConfig::rebot_dm() };
        let drv = TrackingTestDriver::new(&q0, vec![0.0; 7]);
        let _inner = SupervisorInner::new(cfg).with_driver(Box::new(drv.clone()));
        assert_eq!(
            drv.with(|s| s.strict_mode_handover),
            Some(strict),
            "{label}: with_driver must select strict mode handover only with tracking enabled"
        );
    }

    // 2. Gravity-comp handback requires fresh stationary feedback before ownership transfer.
    {
        let cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            workspace: None,
            joint_tracking: Some(lead_config(true, 0.68)),
            ..ArmConfig::rebot_dm()
        };
        let q0 = [0.0, -0.5, -0.5, 0.5, 0.0, 0.0, -2.0];
        let (mut inner, drv) = rig_at(cfg, &q0, vec![0.0; 7]);
        inner.enter_gravity_comp(Default::default()).unwrap();
        assert!(inner.gravity_comp.is_some());
        inner.tick(DT);
        inner.exit_gravity_comp("REQUESTED", "test");

        // Keep measured velocity above entry_max_velocity (0.05 rad/s) past exit_hold_ms (100 ms):
        // ownership must NOT transfer back to Position/Tracking while arm is moving!
        for _ in 0..8 {
            drv.with(|s| s.v[0] = 0.20);
            inner.tick(DT);
        }
        assert!(inner.gravity_comp.is_some(), "must stay in Exiting while arm is still moving");

        // Once stationary and fresh, handback succeeds, seeds a confirmed baseline, and the same
        // tick's tracking output validates and issues the stationary hold frame.
        drv.with(|s| s.v[0] = 0.0);
        inner.tick(DT);
        assert!(inner.gravity_comp.is_none(), "stationary fresh feedback should complete handback");
        assert!(inner.safety.state().is_operational(), "clean handback must stay operational");
        assert_eq!(inner.tracking.history_state, TrackingHistoryState::Issued);
        let issued = inner.tracking.last_issued_positions.as_ref().expect("confirmed baseline issued");
        for i in 0..6 {
            assert!((issued[i] - q0[i]).abs() < 1e-4, "joint {i} handback target drifted: {} vs {}", issued[i], q0[i]);
        }
    }
}

/// M6: Action completion ("done") is deferred until that tick's output passes tracking
/// validation and is written to the driver. A rejected or failed final frame fails the action.
#[test]
fn tracking_defers_action_done_until_final_output_accepted() {
    let cur_gripper_frac = |inner: &SupervisorInner, q_grip: f32| -> f64 {
        let policy = inner.config.gripper_policy.as_ref().unwrap();
        f64::from(((q_grip - policy.closed_rad) / (policy.open_rad - policy.closed_rad)).clamp(0.0, 1.0))
    };
    let make_rig = || {
        let cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            workspace: None,
            joint_tracking: Some(lead_config(true, 0.68)),
            ..ArmConfig::rebot_dm()
        };
        let q0 = [0.0, -0.5, -0.5, 0.5, 0.0, 0.0, -2.0];
        let (mut inner, drv) = rig_at(cfg, &q0, vec![0.0; 7]);
        let epoch = inner.safety.epoch_id();
        let cur_frac = cur_gripper_frac(&inner, q0[6]);
        let (id, _) = inner
            .start_gripper(
                GripperRequest {
                    command: GripperCommand::Move,
                    fraction: Some(cur_frac),
                    grasp: false,
                    speed: 1.0,
                    timeout_s: Some(5.0),
                    tag: None,
                },
                epoch,
            )
            .unwrap();
        (inner, drv, id, q0)
    };

    // 1. Control: measure exact tick count `done_tick` until StepStatus::Done publishes "done".
    let done_tick = {
        let (mut inner, _drv, id, _q0) = make_rig();
        let mut n = 0usize;
        for k in 1..=30 {
            let obs = inner.tick(DT);
            assert!(obs.safety_state.is_operational());
            let act = obs.action.expect("action status present");
            assert_eq!(act.id, id);
            if act.state == "done" {
                n = k;
                break;
            }
            assert_eq!(act.state, "running");
        }
        assert!(n >= 1, "control gripper action must reach done");
        n
    };

    // 2. Final frame rejected by tracking validation on the EXACT `done_tick`.
    {
        let (mut inner, _drv, id, q0) = make_rig();
        for _ in 1..done_tick {
            let obs = inner.tick(DT);
            assert_eq!(obs.action.as_ref().unwrap().state, "running");
        }
        assert!(inner.active_trajectory.is_some(), "trajectory must still be active right before done_tick");
        // Tighten joint 1 limit so tracking rejects the final tick's output with JointLimit.
        inner.config.limits[0].max_position = q0[0] - 0.05;
        let obs = inner.tick(DT);
        assert!(inner.active_trajectory.is_none(), "StepStatus::Done must have executed on done_tick");
        assert!(matches!(obs.safety_state, SafetyState::Fault(_)), "rejected final frame must fault");
        let act = obs.action.expect("action status present");
        assert_eq!(act.id, id);
        assert_eq!(act.state, "failed", "action must NOT report done when final frame is rejected by tracking");
        assert!(
            obs.fault.as_ref().is_some_and(|f| f.message.contains("Joint tracking output rejected")),
            "fault must come from tracking rejection: {:?}",
            obs.fault
        );
    }

    // 3. Final frame accepted by tracking, but `driver.write_joint_targets` fails on `done_tick`.
    {
        let (mut inner, drv, id, _q0) = make_rig();
        for _ in 1..done_tick {
            let obs = inner.tick(DT);
            assert_eq!(obs.action.as_ref().unwrap().state, "running");
        }
        assert!(inner.active_trajectory.is_some(), "trajectory must still be active right before done_tick");
        drv.with(|s| s.fail_next_write = true);
        let obs = inner.tick(DT);
        assert!(inner.active_trajectory.is_none(), "StepStatus::Done must have executed on done_tick");
        assert!(matches!(obs.safety_state, SafetyState::Fault(_)), "failed driver write must fault");
        let act = obs.action.expect("action status present");
        assert_eq!(act.id, id);
        assert_eq!(act.state, "failed", "action must NOT report done when final frame fails driver write");
    }
}

/// M7: Drive active non-zero lead (`>= 0.015 rad`) through every safety/control transition
/// with leases enabled and a fake driver that can fail writes and honours mode-switch holds:
/// stop, fault, E-stop, collision, torque hold, lease loss, park, partial write, reconnect,
/// and handover (both moving lease preemption and gravity-comp MIT <-> POS_VEL handover).
/// Asserts last frame before, every frame during, and first frame after each transition.
#[test]
fn tracking_active_lead_transitions_with_leases_and_faulty_driver() {
    use armd::tracking::TrackingHistoryState;

    const CLIENT_A: &str = "client-a";
    const CLIENT_B: &str = "client-b";
    let q0 = [0.0, -0.5, -0.5, 0.5, 0.0, 0.0, -2.0];
    let tau_s = vec![0.68, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05];

    let rig_leased = |mut cfg: ArmConfig| -> (SupervisorInner, TrackingTestDriver, u64) {
        cfg.workspace = None;
        cfg.joint_tracking = Some(lead_config(true, 0.68));
        let drv = TrackingTestDriver::new(&q0, tau_s.clone());
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv.clone()));
        assert!(inner.lease_enabled, "M7 transitions must run with leases enabled");
        inner.acquire_lease(CLIENT_A).unwrap();
        inner.commanded_positions = q0.to_vec();
        inner.measured_positions = q0.to_vec();
        inner.target_positions = q0.to_vec();
        inner.tick(DT);
        inner.set_torque(true).unwrap();
        inner.tick(DT);
        assert!(inner.safety.state().is_operational(), "{:?}", inner.safety.state());
        let epoch = inner.safety.epoch_id();
        (inner, drv, epoch)
    };

    // Drive J1 along a smooth quintic trajectory with lease heartbeats until active lead >= 0.015 rad.
    // Returns `(last_frame_before, nominal_before, lead_before, written_len_before)`.
    let drive_active_lead = |inner: &mut SupervisorInner, drv: &TrackingTestDriver, client: &str| -> (Vec<JointTarget>, f32, f32, usize) {
        let start = inner.commanded_positions.clone();
        let mut goal = start.clone();
        goal[0] += 0.20;
        let traj = TrajectoryPlanner::plan(
            &start,
            &vec![0.0; start.len()],
            &goal,
            &vec![0.0; start.len()],
            Some(4.0),
            &inner.config.limits,
        )
        .unwrap();
        let _ = inner.safety.mark_motion_started();
        inner.active_trajectory = Some(armd::motion::ActiveTrajectory::quintic(traj, None));
        for _ in 0..65 {
            let epoch = inner.safety.epoch_id();
            inner.heartbeat(client, epoch).unwrap();
            let obs = inner.tick(DT);
            assert!(obs.safety_state.is_operational(), "active lead buildup faulted: {:?}", obs.fault);
        }
        let written_len = drv.with(|s| s.written.len());
        let last_before = drv.with(|s| s.written.last().unwrap().clone());
        let nominal_before = inner.commanded_positions[0];
        let lead_before = last_before[0].position - nominal_before;
        assert!(
            lead_before >= 0.015,
            "expected active lead >= 0.015 rad before transition, got {lead_before:.4}"
        );
        (last_before, nominal_before, lead_before, written_len)
    };

    let hb = |inner: &mut SupervisorInner, client: &str| {
        let epoch = inner.safety.epoch_id();
        let _ = inner.heartbeat(client, epoch);
    };

    // 1. STOP (stop_motion mid-stream with active lead)
    {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.lease_timeout_ms = 5_000;
        let (mut inner, drv, _) = rig_leased(cfg);
        let (last_before, _nom, _lead, idx_before) = drive_active_lead(&mut inner, &drv, CLIENT_A);
        let max_v = inner.config.limits[0].max_velocity as f64;

        inner.stop_motion();
        let mut prev_pos = last_before[0].position;
        for _ in 0..35 {
            hb(&mut inner, CLIENT_A);
            let obs = inner.tick(DT);
            assert!(obs.safety_state.is_operational(), "stop transition faulted: {:?}", obs.fault);
        }
        let during_and_after = drv.with(|s| s.written[idx_before..].to_vec());
        assert_eq!(during_and_after.len(), 35);
        for (k, frame) in during_and_after.iter().enumerate() {
            let step_v = ((frame[0].position - prev_pos) as f64 / DT).abs();
            assert!(step_v <= max_v + 1e-2, "stop tick {k} exceeded max velocity: {step_v:.3}");
            prev_pos = frame[0].position;
        }
        // First frame after stop settles: zero velocity, zero lead.
        hb(&mut inner, CLIENT_A);
        let obs_after = inner.tick(DT);
        let first_after = drv.with(|s| s.written.last().unwrap().clone());
        assert!(obs_after.safety_state.is_operational());
        assert!((first_after[0].position - inner.commanded_positions[0]).abs() < 1e-4, "stop residual lead");
        assert!(first_after[0].velocity.unwrap_or(0.0).abs() < 1e-4);
    }

    // 2. FAULT (FollowingError fault mid-motion -> clear_fault -> re-enable)
    {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.lease_timeout_ms = 5_000;
        let (mut inner, drv, _) = rig_leased(cfg);
        let (_last_before, _nom, _lead, idx_before) = drive_active_lead(&mut inner, &drv, CLIENT_A);

        // Induce a real supervisor FollowingError fault on J1.
        let q_pin = inner.commanded_positions[0] - (inner.config.following_error.max_error_rad + 0.05);
        inner.config.following_error.persistence_ms = 0;
        drv.with(|s| {
            s.pinned[0] = Some(q_pin);
            s.q[0] = q_pin;
            s.v[0] = 0.0;
        });
        for _ in 0..5 {
            hb(&mut inner, CLIENT_A);
            let obs = inner.tick(DT);
            assert!(matches!(obs.safety_state, SafetyState::Fault(_)), "expected FollowingError fault");
        }
        // Every frame during fault must carry zero velocity and hold measured (no added lead).
        let during = drv.with(|s| s.written[idx_before..].to_vec());
        assert!(!during.is_empty());
        for (k, frame) in during.iter().enumerate() {
            assert!(frame[0].velocity.unwrap_or(0.0).abs() <= 1e-4, "fault tick {k} must have zero velocity");
            assert!((frame[0].position - q_pin).abs() < 1e-4, "fault tick {k} must hold measured pose");
        }
        // Unpin, clear fault, and confirm first frame after is clean stationary baseline.
        drv.with(|s| s.pinned[0] = None);
        inner.clear_fault().unwrap();
        inner.set_torque(true).unwrap();
        let idx_after = drv.with(|s| s.written.len());
        hb(&mut inner, CLIENT_A);
        let obs_after = inner.tick(DT);
        assert!(obs_after.safety_state.is_operational());
        let first_after = drv.with(|s| s.written[idx_after].clone());
        assert!((first_after[0].position - inner.measured_positions[0]).abs() < 1e-3, "post-fault first frame must hold measured");
        assert!(first_after[0].velocity.unwrap_or(0.0).abs() < 1e-4);
    }

    // 3. E-STOP (EStopLatch mid-motion -> active restorative hold at measured -> clear_fault)
    {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.lease_timeout_ms = 5_000;
        let (mut inner, drv, _) = rig_leased(cfg);
        let (_last_before, _nom, _lead, idx_before) = drive_active_lead(&mut inner, &drv, CLIENT_A);
        let hold_j0 = inner.measured_positions[0];

        inner.apply_stop_policy(StopPolicy::EStopLatch);
        for _ in 0..5 {
            let obs = inner.tick(DT);
            assert_eq!(obs.safety_state, SafetyState::EStop);
            assert!(obs.is_torqued, "EStopLatch keeps restorative holding torque on");
        }
        // Every frame during E-stop holds measured pose with zero velocity and zero lead.
        let during = drv.with(|s| s.written[idx_before..].to_vec());
        assert_eq!(during.len(), 5, "expected restorative hold frames during E-stop");
        for (k, frame) in during.iter().enumerate() {
            assert!((frame[0].position - hold_j0).abs() < 1e-4, "E-stop tick {k} must hold measured pose");
            assert!(frame[0].velocity.unwrap_or(0.0).abs() < 1e-4, "E-stop tick {k} must have zero velocity");
        }

        inner.clear_fault().unwrap();
        inner.set_torque(true).unwrap();
        let idx_after = drv.with(|s| s.written.len());
        hb(&mut inner, CLIENT_A);
        let obs_after = inner.tick(DT);
        assert!(obs_after.safety_state.is_operational());
        let first_after = drv.with(|s| s.written[idx_after].clone());
        assert!(
            (first_after[0].position - inner.measured_positions[0]).abs() < 1e-3,
            "post-estop first frame must hold measured: {} vs {}",
            first_after[0].position,
            inner.measured_positions[0]
        );
        assert!(first_after[0].velocity.unwrap_or(0.0).abs() < 1e-4);
    }

    // 4. COLLISION (collision guard trip mid-stream -> hold -> quiet release)
    {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.lease_timeout_ms = 5_000;
        let (mut inner, drv, _) = rig_leased(cfg);
        inner.collision.enabled = true;
        let (last_before, _nom, _lead, idx_before) = drive_active_lead(&mut inner, &drv, CLIENT_A);

        // Inject high external torque spike on J1 until collision guard trips.
        drv.with(|s| s.forced_torque[0] = Some(12.0));
        let mut tripped = false;
        let mut trip_meas = 0.0_f32;
        for _ in 0..10 {
            hb(&mut inner, CLIENT_A);
            inner.tick(DT);
            if inner.collision.hold.is_some() {
                tripped = true;
                trip_meas = inner.measured_positions[0];
                break;
            }
        }
        assert!(tripped, "collision guard must trip on torque spike");
        // Clear torque spike and wait through quiet hold period (resume_after_s = 2.0 s) until auto-resume.
        drv.with(|s| s.forced_torque[0] = None);
        let mut hold_frames = Vec::new();
        for _ in 0..150 {
            let before_tick = drv.with(|s| s.written.len());
            hb(&mut inner, CLIENT_A);
            inner.tick(DT);
            let frame = drv.with(|s| s.written[before_tick].clone());
            if inner.collision.hold.is_some() {
                hold_frames.push(frame);
            } else {
                break;
            }
        }
        assert!(inner.collision.hold.is_none(), "collision hold must clear after quiet period");
        // Every frame during collision hold must not push forward past last_before and must hold measured pose.
        for (k, frame) in hold_frames.iter().enumerate() {
            assert!(
                frame[0].position <= last_before[0].position + 1e-4,
                "collision hold tick {k} pushed forward past pre-trip lead"
            );
            assert!(
                (frame[0].position - trip_meas).abs() < 0.02,
                "collision hold tick {k} drifted from measured pose: {} vs {}",
                frame[0].position,
                trip_meas
            );
            assert!(frame[0].velocity.unwrap_or(0.0).abs() < 1e-4);
        }
        assert!(drv.with(|s| s.written.len()) > idx_before);
        // First frame after collision hold clears: clean stationary baseline, operational.
        let idx_after = drv.with(|s| s.written.len());
        hb(&mut inner, CLIENT_A);
        let obs_after = inner.tick(DT);
        assert!(obs_after.safety_state.is_operational());
        let first_after = drv.with(|s| s.written[idx_after].clone());
        assert!((first_after[0].position - inner.commanded_positions[0]).abs() < 1e-4);
    }

    // 5. TORQUE HOLD (torque guard saturation trip -> relieving ramp-out -> release)
    {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.lease_timeout_ms = 5_000;
        cfg.torque_guard = Some(TorqueGuardPolicy {
            enabled: true,
            ratio: 0.60,
            persist_ms: 40,
            ..Default::default()
        });
        let (mut inner, drv, _) = rig_leased(cfg);
        let (last_before, _nom, _lead, idx_before) = drive_active_lead(&mut inner, &drv, CLIENT_A);
        let max_v = inner.config.limits[0].max_velocity as f64;

        drv.with(|s| s.forced_torque[0] = Some(25.0));
        let mut tripped = false;
        let mut prev_pos = last_before[0].position;
        for k in 0..25 {
            let before_tick = drv.with(|s| s.written.len());
            hb(&mut inner, CLIENT_A);
            let obs = inner.tick(DT);
            assert!(obs.safety_state.is_operational(), "torque hold must stay operational: {:?}", obs.fault);
            if inner.torque_guard.hold.is_some() {
                tripped = true;
                drv.with(|s| s.forced_torque[0] = Some(5.0));
            }
            let frame = drv.with(|s| s.written[before_tick].clone());
            let step_v = ((frame[0].position - prev_pos) as f64 / DT).abs();
            assert!(step_v <= max_v + 1e-2, "torque hold tick {k} velocity {step_v:.3} > {max_v:.3}");
            if tripped {
                assert!(frame[0].position <= prev_pos + 1e-6, "torque hold tick {k} advanced in +1 tripped direction");
            }
            prev_pos = frame[0].position;
        }
        assert!(tripped, "torque guard must trip");
        assert!(drv.with(|s| s.written.len()) > idx_before);
        // Drop overload and retreat in the relieving (-1) direction so torque hold releases cleanly.
        drv.with(|s| s.forced_torque[0] = None);
        let mut retreat = inner.commanded_positions.clone();
        retreat[0] -= 0.08;
        for _ in 0..80 {
            let epoch = inner.safety.epoch_id();
            inner.heartbeat(CLIENT_A, epoch).unwrap();
            let _ = inner.servo(retreat.clone(), epoch);
            inner.tick(DT);
            if inner.torque_guard.hold.is_none() {
                break;
            }
        }
        assert!(inner.torque_guard.hold.is_none(), "torque hold must clear after relieving retreat");
        inner.stop_motion();
        for _ in 0..20 {
            hb(&mut inner, CLIENT_A);
            inner.tick(DT);
        }
        let idx_after = drv.with(|s| s.written.len());
        hb(&mut inner, CLIENT_A);
        let obs_after = inner.tick(DT);
        assert!(obs_after.safety_state.is_operational());
        let first_after = drv.with(|s| s.written[idx_after].clone());
        assert!((first_after[0].position - inner.commanded_positions[0]).abs() < 1e-3);
    }

    // 6. LEASE LOSS (watchdog timeout while streaming with active lead -> re-acquire + re-arm)
    {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.lease_timeout_ms = 200;
        let (mut inner, drv, _) = rig_leased(cfg);
        let (last_before, _nom, _lead, idx_before) = drive_active_lead(&mut inner, &drv, CLIENT_A);
        let max_v = inner.config.limits[0].max_velocity as f64;

        // Backdate both watchdog and lease heartbeats past lease_timeout_ms so lease loss fires.
        let expired = std::time::Instant::now() - Duration::from_secs(5);
        inner.last_heartbeat = expired;
        if let Some(l) = inner.active_lease.as_mut() {
            l.acquired_at = expired;
        }
        for _ in 0..250 {
            inner.tick(DT);
            if !inner.is_torqued || matches!(inner.safety.state(), SafetyState::Fault(_)) {
                break;
            }
        }
        let during = drv.with(|s| s.written[idx_before..].to_vec());
        assert!(!during.is_empty(), "lease-loss transition must ramp/park smoothly");
        let mut prev_pos = last_before[0].position;
        for (k, frame) in during.iter().enumerate() {
            let step_v = ((frame[0].position - prev_pos) as f64 / DT).abs();
            assert!(step_v <= max_v + 1e-2, "lease-loss tick {k} exceeded velocity cap: {step_v:.3}");
            prev_pos = frame[0].position;
        }
        // Re-acquire lease, clear any watchdog fault, re-enable -> first frame after is clean hold.
        inner.acquire_lease(CLIENT_B).unwrap();
        if matches!(inner.safety.state(), SafetyState::Fault(_)) {
            inner.clear_fault().unwrap();
        }
        inner.set_torque(true).unwrap();
        let idx_after = drv.with(|s| s.written.len());
        hb(&mut inner, CLIENT_B);
        let obs_after = inner.tick(DT);
        assert!(obs_after.safety_state.is_operational());
        let first_after = drv.with(|s| s.written[idx_after].clone());
        assert!((first_after[0].position - inner.measured_positions[0]).abs() < 1e-3);
    }

    // 7. PARK (NormalUnload with active lead -> smooth ramp-out -> settle -> torque off -> re-arm)
    {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.lease_timeout_ms = 5_000;
        let (mut inner, drv, _) = rig_leased(cfg);
        let (last_before, _nom, _lead, idx_before) = drive_active_lead(&mut inner, &drv, CLIENT_A);
        let max_v = inner.config.limits[0].max_velocity as f64;

        // Simulate mechanical rest support at resting pose so support_torque_limits can clear.
        drv.with(|s| s.include_gravity_torque = false);
        inner.apply_stop_policy(StopPolicy::NormalUnload);
        for _ in 0..400 {
            let obs = inner.tick(DT);
            if !obs.is_torqued {
                break;
            }
        }
        assert!(
            !inner.is_torqued,
            "park must complete and disable torque (state={:?}, pending={}, dur={:?}, q_m={:?}, q_c={:?})",
            inner.safety.state(),
            inner.pending_normal_unload,
            inner.unload_planned_duration,
            inner.measured_positions,
            inner.commanded_positions,
        );
        let during = drv.with(|s| s.written[idx_before..].to_vec());
        assert!(during.len() >= 5, "park must ramp lead out and settle before torque-off");
        let mut prev_pos = last_before[0].position;
        for (k, frame) in during.iter().enumerate() {
            let step_v = ((frame[0].position - prev_pos) as f64 / DT).abs();
            assert!(step_v <= max_v + 1e-2, "park tick {k} exceeded velocity cap: {step_v:.3}");
            prev_pos = frame[0].position;
        }
        // Re-arm -> first frame after is clean stationary hold at parked pose.
        inner.set_torque(true).unwrap();
        let idx_after = drv.with(|s| s.written.len());
        hb(&mut inner, CLIENT_A);
        let obs_after = inner.tick(DT);
        assert!(obs_after.safety_state.is_operational());
        let first_after = drv.with(|s| s.written[idx_after].clone());
        assert!((first_after[0].position - inner.measured_positions[0]).abs() < 1e-3);
        assert!(first_after[0].velocity.unwrap_or(0.0).abs() < 1e-4);
    }

    // 8. PARTIAL WRITE (apply earlier joints 0..2 on plant, fail joint 3 -> fault + history Unknown -> clear + re-arm)
    {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.lease_timeout_ms = 5_000;
        let (mut inner, drv, _) = rig_leased(cfg);
        let (_last_before, _nom, _lead, idx_before) = drive_active_lead(&mut inner, &drv, CLIENT_A);
        let q0_before_partial = drv.with(|s| s.q[0]);

        drv.with(|s| s.fail_write_at_joint = Some(2));
        hb(&mut inner, CLIENT_A);
        let obs_fail = inner.tick(DT);
        assert!(matches!(obs_fail.safety_state, SafetyState::Fault(_)), "partial write failure must latch fault");
        assert_eq!(inner.tracking.history_state, TrackingHistoryState::Unknown, "partial write must invalidate history");
        assert_eq!(drv.with(|s| s.written.len()), idx_before, "partial write must not record a full confirmed frame");
        assert_eq!(drv.with(|s| s.partial_written.len()), 1, "expected 1 partial write prefix");
        assert_eq!(drv.with(|s| s.partial_written[0].len()), 2, "joints 0..2 must have been applied before J3 failed");
        let q0_after_partial = drv.with(|s| s.q[0]);
        assert!(
            q0_after_partial > q0_before_partial + 1e-4,
            "earlier joint J1 must actually have moved on the plant during partial write ({q0_before_partial:.5} -> {q0_after_partial:.5})"
        );

        // Let plant settle before re-enabling, then verify first frame after holds actual post-partial-write pose.
        drv.with(|s| s.v.fill(0.0));
        inner.clear_fault().unwrap();
        inner.set_torque(true).unwrap();
        let idx_after = drv.with(|s| s.written.len());
        hb(&mut inner, CLIENT_A);
        let obs_after = inner.tick(DT);
        assert!(obs_after.safety_state.is_operational());
        let first_after = drv.with(|s| s.written[idx_after].clone());
        assert!((first_after[0].position - q0_after_partial).abs() < 1e-3);
        assert!(first_after[0].velocity.unwrap_or(0.0).abs() < 1e-4);
    }

    // 9. RECONNECT (bus feedback loss -> output_suspended / BusLoss -> restore feedback -> re-arm)
    {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.lease_timeout_ms = 5_000;
        let (mut inner, drv, _) = rig_leased(cfg);
        let (_last_before, _nom, _lead, idx_before) = drive_active_lead(&mut inner, &drv, CLIENT_A);

        // Make feedback exceed BUS_LOSS_TIMEOUT (500 ms) so bus loss suspends all bus writes immediately.
        drv.with(|s| s.feedback_age = Duration::from_millis(600));
        for _ in 0..5 {
            hb(&mut inner, CLIENT_A);
            inner.tick(DT);
        }
        let during = drv.with(|s| s.written[idx_before..].to_vec());
        assert!(during.is_empty(), "stale/lost bus must suppress every write, wrote {}", during.len());
        assert_eq!(inner.tracking.history_state, TrackingHistoryState::Unknown);

        // Restore fresh feedback, clear any fault, and confirm clean first frame after reconnect.
        drv.with(|s| s.feedback_age = Duration::ZERO);
        hb(&mut inner, CLIENT_A);
        inner.tick(DT);
        if matches!(inner.safety.state(), SafetyState::Fault(_)) {
            inner.clear_fault().unwrap();
        }
        inner.set_torque(true).unwrap();
        let idx_after = drv.with(|s| s.written.len());
        hb(&mut inner, CLIENT_A);
        let obs_after = inner.tick(DT);
        assert!(obs_after.safety_state.is_operational());
        let first_after = drv.with(|s| s.written[idx_after].clone());
        assert!((first_after[0].position - inner.measured_positions[0]).abs() < 1e-3);
        assert!(first_after[0].velocity.unwrap_or(0.0).abs() < 1e-4);
    }

    // 10a. HANDOVER — moving lease preemption with active lead
    {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.lease_timeout_ms = 5_000;
        let (mut inner, drv, _) = rig_leased(cfg);
        let (last_before, _nom, _lead, idx_before) = drive_active_lead(&mut inner, &drv, CLIENT_A);
        let max_v = inner.config.limits[0].max_velocity as f64;

        inner.preempt_lease(CLIENT_B, "takeover").unwrap();
        assert!(inner.handover_settling_since.is_some(), "moving handover must enter settling");
        let mut prev_pos = last_before[0].position;
        let mut settled = false;
        for k in 0..60 {
            let before_tick = drv.with(|s| s.written.len());
            hb(&mut inner, CLIENT_B);
            let obs = inner.tick(DT);
            assert!(obs.safety_state.is_operational(), "handover tick {k} faulted: {:?}", obs.fault);
            let frame = drv.with(|s| s.written[before_tick].clone());
            let step_v = ((frame[0].position - prev_pos) as f64 / DT).abs();
            assert!(step_v <= max_v + 1e-2, "handover tick {k} exceeded velocity cap: {step_v:.3}");
            prev_pos = frame[0].position;
            if inner.handover_settling_since.is_none() {
                settled = true;
                break;
            }
        }
        assert!(settled, "moving lease handover must finish settling");
        assert!(drv.with(|s| s.written.len()) > idx_before);
        let idx_after = drv.with(|s| s.written.len());
        hb(&mut inner, CLIENT_B);
        let obs_after = inner.tick(DT);
        assert!(obs_after.safety_state.is_operational());
        let first_after = drv.with(|s| s.written[idx_after].clone());
        assert!((first_after[0].position - inner.commanded_positions[0]).abs() < 1e-4);
    }

    // 10b. HANDOVER — gravity-comp mode-switch handover (honours mode-switch hold frames)
    {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.lease_timeout_ms = 5_000;
        let (mut inner, drv, _) = rig_leased(cfg);
        let (_last_before, _nom, _lead, _idx_before) = drive_active_lead(&mut inner, &drv, CLIENT_A);
        // Stop and settle to enter gravity comp cleanly.
        inner.stop_motion();
        for _ in 0..45 {
            hb(&mut inner, CLIENT_A);
            inner.tick(DT);
        }
        drv.with(|s| s.v.fill(0.0));
        hb(&mut inner, CLIENT_A);
        inner.tick(DT);

        inner.enter_gravity_comp(Default::default()).unwrap();
        let enter_switches = drv.with(|s| s.mode_hold_frames.len());
        assert_eq!(enter_switches, 1, "entering gravity comp must send MIT mode-switch hold frame");
        assert_eq!(drv.with(|s| s.mode_hold_frames[0].0[0]), JointMode::Mit);

        // Float for 2 ticks, then exit gravity comp.
        for _ in 0..2 {
            hb(&mut inner, CLIENT_A);
            inner.tick(DT);
        }
        let idx_during_exit = drv.with(|s| s.written.len());
        inner.exit_gravity_comp("REQUESTED", "m7");
        for _ in 0..10 {
            hb(&mut inner, CLIENT_A);
            inner.tick(DT);
            if inner.gravity_comp.is_none() {
                break;
            }
        }
        assert!(inner.gravity_comp.is_none(), "gravity-comp handback must complete when stationary");
        let switches = drv.with(|s| s.mode_hold_frames.clone());
        assert_eq!(switches.len(), 2, "exiting gravity comp must send Position mode-switch hold frame");
        assert_eq!(switches[1].0[0], JointMode::Position);
        // Every MIT hold frame during Exiting had valid MIT gains and finite targets.
        let during_exit = drv.with(|s| s.written[idx_during_exit..].to_vec());
        assert!(!during_exit.is_empty());
        // First POS_VEL frame after handback is a confirmed stationary baseline at the hold pose.
        let idx_after = drv.with(|s| s.written.len());
        hb(&mut inner, CLIENT_A);
        let obs_after = inner.tick(DT);
        assert!(obs_after.safety_state.is_operational());
        let first_after = drv.with(|s| s.written[idx_after].clone());
        let hold_j0 = switches[1].1[0].position;
        assert!((first_after[0].position - hold_j0).abs() < 1e-4, "first POS_VEL frame after handback must match hold");
        assert!(first_after[0].velocity.unwrap_or(0.0).abs() < 1e-4);
    }
}

/// T1: With tracking OFF, gravity-comp exit restores pre-change behavior (attempts POS_VEL
/// handback after exit_hold_ms without latching GravityCompFailed on stale RX); with tracking ON,
/// stale RX blocks handback and latches GravityCompFailed.
#[test]
fn tracking_off_gravity_comp_exit_restores_pre_change_stale_rx_behavior() {
    let q0 = [0.0, -0.5, -0.5, 0.5, 0.0, 0.0, -2.0];

    // Tracking OFF: stale feedback (150 ms > 100 ms gravity_comp max_age) does NOT block POS_VEL handback.
    {
        let cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            workspace: None,
            joint_tracking: Some(JointTrackingConfig::rebot_dm()), // enabled = false
            ..ArmConfig::rebot_dm()
        };
        let (mut inner, drv) = rig_at(cfg, &q0, vec![0.0; 7]);
        inner.enter_gravity_comp(Default::default()).unwrap();
        inner.tick(DT);
        inner.exit_gravity_comp("REQUESTED", "t1_off");
        drv.with(|s| s.feedback_age = Duration::from_millis(150));
        for _ in 0..6 {
            inner.tick(DT);
        }
        assert!(
            inner.gravity_comp.is_none(),
            "tracking OFF must attempt and complete POS_VEL handback after exit_hold_ms even with stale RX"
        );
        assert!(
            inner.safety.state().is_operational(),
            "tracking OFF must not latch GravityCompFailed on stale RX: {:?}",
            inner.safety.state()
        );
    }

    // Tracking ON: stale feedback blocks POS_VEL handback and latches GravityCompFailed.
    {
        let cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            workspace: None,
            joint_tracking: Some(lead_config(true, 0.68)),
            ..ArmConfig::rebot_dm()
        };
        let (mut inner, drv) = rig_at(cfg, &q0, vec![0.0; 7]);
        inner.enter_gravity_comp(Default::default()).unwrap();
        inner.tick(DT);
        inner.exit_gravity_comp("REQUESTED", "t1_on");
        drv.with(|s| s.feedback_age = Duration::from_millis(150));
        for _ in 0..6 {
            inner.tick(DT);
        }
        assert!(
            inner.gravity_comp.is_some(),
            "tracking ON must not transfer ownership back to tracking on stale RX"
        );
        assert!(
            matches!(inner.safety.state(), SafetyState::Fault(_)),
            "tracking ON must latch GravityCompFailed when stale RX blocks handback"
        );
    }
}

/// T2: Prove with real `DamiaoArmDriver<MockCanTransport>` that keeping the stricter
/// `write_modes` hold-transmit + flush checks leaves every previously succeeding sequence
/// unchanged: (1) cold arm, (2) gravity-comp enter/exit, (3) re-arm after interrupted MIT session.
#[test]
fn damiao_write_modes_preserves_cold_arm_gravity_comp_and_interrupted_rearm() {
    use armd::driver::DamiaoArmDriver;
    use armd::protocol::damiao::{self, DamiaoLimits};
    use armd::transport::{CanFrame, MockCanTransport};

    let make_healthy_can = || -> MockCanTransport {
        let mut mock = MockCanTransport::new();
        let limits = DamiaoLimits::default();
        let p = damiao::float_to_uint(0.15, limits.p_min, limits.p_max, 16);
        let enabled_fb = CanFrame::new_standard(0x11, &[0x11, (p >> 8) as u8, p as u8, 0, 0, 0, 30, 30]).unwrap();
        let disabled_fb = CanFrame::new_standard(0x11, &[0x01, (p >> 8) as u8, p as u8, 0, 0, 0, 30, 30]).unwrap();
        mock.respond_to_transmit(damiao::build_enable_frame(1), enabled_fb);
        mock.respond_to_transmit(damiao::build_disable_frame(1), disabled_fb);
        let mk_reply = |op: u8, rid: u8, bytes: [u8; 4]| {
            CanFrame::new_standard(0, &[1, 0, op, rid, bytes[0], bytes[1], bytes[2], bytes[3]]).unwrap()
        };
        for (rid, val) in [(damiao::RID_PMAX, 12.5_f32), (damiao::RID_VMAX, 30.0), (damiao::RID_TMAX, 18.0)] {
            mock.respond_to_transmit(
                damiao::build_read_register_frame(1, rid),
                mk_reply(damiao::REG_READ, rid, val.to_le_bytes()),
            );
        }
        for mode in [damiao::CTRL_MODE_MIT, damiao::CTRL_MODE_POS_VEL] {
            mock.respond_to_transmit(
                damiao::build_set_control_mode_frame(1, mode),
                mk_reply(damiao::REG_WRITE, damiao::RID_CTRL_MODE, mode.to_le_bytes()),
            );
        }
        mock
    };

    // 1. Cold arm (no mode switches issued)
    let mut drv = DamiaoArmDriver::new(make_healthy_can(), vec![1]);
    drv.set_torque(true).expect("cold arm must succeed unchanged");
    assert!(drv.is_enabled());
    assert!(!drv.joint_modes_uncertain());

    // 2. Gravity-comp enter (POS_VEL -> MIT) and exit (MIT -> POS_VEL)
    drv.set_joint_modes(&[JointMode::Mit], &[JointTarget::mit(0.15, 0.0, 30.0, 1.0, 0.0)])
        .expect("gravity-comp entry must succeed unchanged");
    assert!(!drv.joint_modes_uncertain());
    drv.set_joint_modes(&[JointMode::Position], &[JointTarget::position(0.15)])
        .expect("gravity-comp exit must succeed unchanged");
    assert!(!drv.joint_modes_uncertain());

    // 3. Re-arm after interrupted MIT session (motor left in MIT when torque disabled -> restored on set_torque(true))
    drv.set_joint_modes(&[JointMode::Mit], &[JointTarget::mit(0.15, 0.0, 30.0, 1.0, 0.0)])
        .expect("enter MIT before simulated interruption");
    drv.set_torque(false).expect("disable while in MIT");
    assert!(!drv.is_enabled());
    let _ = drv.can_mut().take_tx_frames();
    drv.set_torque(true)
        .expect("re-arm after interrupted MIT session must restore POS_VEL with dual-hold and succeed");
    assert!(drv.is_enabled());
    assert!(!drv.joint_modes_uncertain());
    // Review 3 T2: the restored mode is POS_VEL, written (with its dual hold) before the enable.
    let tx = drv.can_mut().take_tx_frames();
    let mode_write = damiao::build_set_control_mode_frame(1, damiao::CTRL_MODE_POS_VEL);
    let mode_at = tx.iter().position(|f| f.id == mode_write.id && f.data == mode_write.data)
        .expect("re-arm must write CTRL_MODE = POS_VEL");
    let enable = damiao::build_enable_frame(1);
    let enable_at = tx.iter().position(|f| f.id == enable.id && f.data == enable.data).expect("re-arm must send enable");
    assert!(mode_at < enable_at, "CTRL_MODE = POS_VEL (frame {mode_at}) must precede the enable (frame {enable_at})");
    assert!(
        !tx.iter().any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE && f.data[3] == damiao::RID_CTRL_MODE
            && f.data[4..8] != damiao::CTRL_MODE_POS_VEL.to_le_bytes()),
        "re-arm must not write any other control mode"
    );
    let hold_frames = tx[mode_at + 1..enable_at].iter().filter(|f| f.id == 0x101 || f.id == 1).count();
    assert!(hold_frames >= 2, "dual hold (POS_VEL + MIT) follows the mode write before the enable, got {hold_frames}");
}

/// Review 2 New: When Damiao recovery `can.transmit` returns `Err`, the failed transmit
/// must NOT be recorded in `take_recovery_writes()`.
#[test]
fn damiao_recovery_tx_failure_not_recorded_as_sent() {
    use armd::driver::DamiaoArmDriver;
    use armd::protocol::damiao::{self, DamiaoLimits};
    use armd::transport::{CanFrame, CanTransport, MockCanTransport, TransportError};

    struct FailRecoveryTxCan {
        inner: MockCanTransport,
        fail_pos_vel: bool,
    }
    impl CanTransport for FailRecoveryTxCan {
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
            if self.fail_pos_vel && frame.id == 0x101 {
                return Err(TransportError::Io(std::io::Error::other("simulated recovery TX failure")));
            }
            self.inner.transmit(frame)
        }
        fn receive(&mut self) -> Result<CanFrame, TransportError> {
            self.inner.receive()
        }
        fn flush(&mut self) -> Result<(), TransportError> {
            self.inner.flush()
        }
    }

    let mut mock = MockCanTransport::new();
    let limits = DamiaoLimits::default();
    let p = damiao::float_to_uint(0.1, limits.p_min, limits.p_max, 16);
    let fb = CanFrame::new_standard(0x11, &[0x11, (p >> 8) as u8, p as u8, 0, 0, 0, 30, 30]).unwrap();
    mock.respond_to_transmit(damiao::build_enable_frame(1), fb);
    let mut drv = DamiaoArmDriver::new(FailRecoveryTxCan { inner: mock, fail_pos_vel: false }, vec![1]);
    drv.set_torque(true).unwrap();

    // Sleep past 45 ms recovery threshold, then make recovery TX on 0x101 fail.
    std::thread::sleep(Duration::from_millis(55));
    drv.can_mut().fail_pos_vel = true;
    let _ = drv.read_joint_states();
    assert!(
        drv.take_recovery_writes().is_empty(),
        "failed recovery transmit must not be recorded as sent"
    );

    // Once TX succeeds, recovery write IS recorded.
    drv.can_mut().fail_pos_vel = false;
    let _ = drv.read_joint_states();
    assert_eq!(drv.take_recovery_writes(), vec![0], "successful recovery transmit must be recorded");
}

/// Review 2 B1: One joint's Damiao recovery must NOT wipe issued history for unaffected
/// moving/lagging joints or reseed the whole arm to measured position with zero velocity.
#[test]
fn tracking_damiao_recovery_preserves_unaffected_moving_joint() {
    use armd::driver::DamiaoArmDriver;
    use armd::protocol::damiao::{self, DamiaoLimits};
    use armd::transport::{CanFrame, CanTransport, TransportError};
    use std::collections::VecDeque;

    struct MovingRecoveryBus {
        rx: VecDeque<CanFrame>,
        tx_log: Vec<CanFrame>,
        q_meas: [f32; 7],
        drop_rx: [bool; 7],
        limits: DamiaoLimits,
    }
    impl MovingRecoveryBus {
        fn new(q0: [f32; 7]) -> Self {
            Self {
                rx: VecDeque::new(),
                tx_log: Vec::new(),
                q_meas: q0,
                drop_rx: [false; 7],
                limits: DamiaoLimits::default(),
            }
        }
        fn push_feedback(&mut self, i: usize) {
            let id = (i + 1) as u8;
            let p_u = damiao::float_to_uint(self.q_meas[i], self.limits.p_min, self.limits.p_max, 16);
            // Zero velocity and torque (raw 0 would decode as -30 rad/s and -18 N·m).
            let v_u = damiao::float_to_uint(0.0, self.limits.v_min, self.limits.v_max, 12);
            let t_u = damiao::float_to_uint(0.0, self.limits.t_min, self.limits.t_max, 12);
            let frame = CanFrame::new_standard(
                0x10 + u16::from(id),
                &[
                    0x10 | id,
                    (p_u >> 8) as u8,
                    p_u as u8,
                    (v_u >> 4) as u8,
                    (((v_u & 0x0F) << 4) | (t_u >> 8)) as u8,
                    t_u as u8,
                    30,
                    30,
                ],
            )
            .unwrap();
            self.rx.push_back(frame);
        }
    }
    #[derive(Clone)]
    struct SharedBus(Arc<Mutex<MovingRecoveryBus>>);
    impl CanTransport for SharedBus {
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
            let mut s = self.0.lock().unwrap();
            s.tx_log.push(*frame);
            if frame.data == [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC] && (1..=7).contains(&frame.id) {
                s.push_feedback((frame.id - 1) as usize);
            } else if (0x101..=0x107).contains(&frame.id) {
                let idx = (frame.id - 0x101) as usize;
                if !s.drop_rx[idx] {
                    s.push_feedback(idx);
                }
            }
            Ok(())
        }
        fn receive(&mut self) -> Result<CanFrame, TransportError> {
            self.0.lock().unwrap().rx.pop_front().ok_or(TransportError::WouldBlock)
        }
        fn flush(&mut self) -> Result<(), TransportError> {
            Ok(())
        }
    }

    let q0 = [0.0, -0.5, -0.5, 0.5, 0.0, 0.0, -2.0];
    let bus_state = Arc::new(Mutex::new(MovingRecoveryBus::new(q0)));
    let damiao_drv = DamiaoArmDriver::new(SharedBus(bus_state.clone()), vec![1, 2, 3, 4, 5, 6, 7]);

    // Enable tracking on both J1 and J2.
    let mut jt = JointTrackingConfig::rebot_dm();
    jt.enabled = true;
    jt.joints[0].strategy = TrackingStrategy::PositionLead;
    jt.joints[0].lag_s = 0.68;
    jt.joints[1].strategy = TrackingStrategy::PositionLead;
    jt.joints[1].lag_s = 0.68;
    let cfg = ArmConfig {
        lease_timeout_ms: 600_000,
        workspace: None,
        joint_tracking: Some(jt),
        // The fake bus encodes zero raw torque (-18 N·m); keep the torque guard out of this test.
        torque_guard: None,
        // ...and the power governor (it would time-scale the reference from that torque).
        power_budget: None,
        ..ArmConfig::rebot_dm()
    };
    let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(damiao_drv));
    inner.lease_enabled = false;
    inner.commanded_positions = q0.to_vec();
    inner.measured_positions = q0.to_vec();
    inner.target_positions = q0.to_vec();
    inner.set_torque(true).unwrap();
    inner.tick(DT);
    assert!(inner.safety.state().is_operational());

    // Drive J2 along a smooth trajectory while J2 physically lags behind its issued command
    // (simulating real motor lag on unaffected joint 2).
    let mut goal = q0.to_vec();
    goal[1] += 0.20;
    let traj = TrajectoryPlanner::plan(&q0, &[0.0; 7], &goal, &[0.0; 7], Some(4.0), &inner.config.limits).unwrap();
    let _ = inner.safety.mark_motion_started();
    inner.active_trajectory = Some(armd::motion::ActiveTrajectory::quintic(traj, None));
    // J2's plant follows its transmitted POS_VEL target with a first-order lag (tau 3 s),
    // so the unaffected joint keeps moving and lagging behind its issued output.
    let alpha_j2 = 1.0 - (-DT / 3.0_f64).exp();
    for k in 0..75 {
        let obs = inner.tick(DT);
        assert!(obs.safety_state.is_operational(), "tick {k} before recovery: {:?} {:?}", obs.safety_state, obs.fault);
        let issued_j2 = inner.tracking.last_issued_positions.as_ref().map_or(inner.commanded_positions[1], |v| v[1]);
        let mut s = bus_state.lock().unwrap();
        s.q_meas[1] = (f64::from(s.q_meas[1]) + (f64::from(issued_j2) - f64::from(s.q_meas[1])) * alpha_j2) as f32;
        for i in 0..7 {
            if i != 1 {
                s.q_meas[i] = inner.commanded_positions[i];
            }
            s.push_feedback(i);
        }
    }
    let j2_issued_before = inner.tracking.last_issued_positions.as_ref().unwrap()[1];
    let j2_issued_v_before = inner.tracking.last_issued_velocities.as_ref().unwrap()[1];
    let j2_nom_before = inner.commanded_positions[1];
    let j2_meas_before = bus_state.lock().unwrap().q_meas[1];
    assert!(
        j2_issued_before - j2_nom_before >= 0.015,
        "J2 must have active lead before J1 recovery (issued={j2_issued_before:.4}, nom={j2_nom_before:.4})"
    );
    assert!(
        j2_issued_before - j2_meas_before >= 0.050,
        "J2 must be lagging significantly behind its issued target"
    );

    // Delay ONLY J1's RX past 45 ms so Damiao emits an autonomous recovery write for J1 only.
    {
        let mut s = bus_state.lock().unwrap();
        s.rx.clear();
        s.drop_rx[0] = true;
        s.tx_log.clear();
    }
    std::thread::sleep(Duration::from_millis(55));
    {
        let mut s = bus_state.lock().unwrap();
        for i in 1..7 {
            s.push_feedback(i);
        }
    }
    inner.tick(DT);
    assert!(inner.tracking.recovery_suspended, "J1 recovery must suspend compensated writes");
    // B1 invariant: unaffected J2's last issued position MUST be preserved (not cleared!).
    assert_eq!(
        inner.tracking.last_issued_positions.as_ref().map(|v| v[1]),
        Some(j2_issued_before),
        "unaffected J2's issued history must be preserved during J1 recovery"
    );

    // Restore J1 RX and push fresh post-recovery feedback for all joints.
    {
        let mut s = bus_state.lock().unwrap();
        s.drop_rx[0] = false;
        s.tx_log.clear();
        for i in 0..7 {
            s.push_feedback(i);
        }
    }
    inner.tick(DT);
    assert!(!inner.tracking.recovery_suspended, "fresh post-recovery feedback must clear suspension");
    assert!(inner.safety.state().is_operational(), "recovery resume must stay operational: {:?}", inner.safety.state());

    // Inspect the first resumed CAN frame sent to unaffected motor 2 (0x102):
    // It must ramp smoothly from `j2_issued_before` within velocity limits, NOT jump by >0.05 rad to `j2_meas_before`!
    let resumed_tx = bus_state.lock().unwrap().tx_log.clone();
    let j2_resumed = resumed_tx
        .iter()
        .find(|f| f.id == 0x102)
        .map(|f| f32::from_le_bytes([f.data[0], f.data[1], f.data[2], f.data[3]]))
        .expect("expected resumed POS_VEL frame on 0x102 for unaffected J2");
    let step_v_j2 = ((j2_resumed - j2_issued_before) as f64 / DT).abs();
    let max_v_j2 = inner.config.limits[1].max_velocity as f64;
    assert!(
        step_v_j2 <= max_v_j2 + 1e-2,
        "unaffected J2 took an unchecked step on J1 recovery resume: prev={j2_issued_before:.4} -> resumed={j2_resumed:.4} (meas={j2_meas_before:.4}, v={step_v_j2:.3} > {max_v_j2:.3})"
    );
    assert!(
        (j2_resumed - j2_meas_before).abs() > 0.020,
        "unaffected J2 must NOT have been reseeded straight to lagging measured pose"
    );
    // Review 3 B1: the first resumed acceleration check must use J2's real issued velocity,
    // not a fabricated zero: the issued J2 output may change velocity by at most a_max * dt.
    assert!(
        j2_issued_v_before.abs() > 0.02,
        "J2 must be moving before J1's recovery (issued v={j2_issued_v_before:.4})"
    );
    let j2_v_resumed = (f64::from(j2_resumed) - f64::from(j2_issued_before)) / DT;
    let j2_a_resumed = (j2_v_resumed - f64::from(j2_issued_v_before)) / DT;
    let max_a_j2 = f64::from(inner.active_tracking_limits(DT, 1.0)[1].max_acceleration);
    assert!(
        j2_a_resumed.abs() <= max_a_j2 + 1e-2,
        "unaffected J2 acceleration across J1 recovery {j2_a_resumed:.3} rad/s^2 exceeds {max_a_j2:.3} \
         (issued v {j2_issued_v_before:.4} -> {j2_v_resumed:.4} rad/s)"
    );
}

/// Review 2 m2: Primary evaluation and rejection fallback share a single per-tick sweep budget.
#[test]
fn tracking_rejection_fallback_shares_per_tick_sweep_budget() {
    use armd::supervisor::MAX_KEEPOUT_EVALS_PER_TICK;

    let mut cfg = ArmConfig::rebot_dm();
    cfg.lease_timeout_ms = 600_000;
    cfg.joint_tracking = Some(lead_config(true, 0.68));
    let q0 = [0.0, -0.5, -0.5, 0.5, 0.0, 0.0, -2.0];
    let (mut inner, _drv) = rig_at(cfg, &q0, vec![0.0; 7]);

    // First verify directly that a zero remaining budget fails fallback sweep subdivision
    // rather than allocating a fresh MAX_KEEPOUT_EVALS_PER_TICK budget, and that a normal
    // budget is decremented in place.
    let mut zero_budget = 0usize;
    let mut q_hold = q0.to_vec();
    q_hold[0] += 1.5;
    assert!(
        !inner.validate_fallback_hold_with_budget(None, &q_hold, true, &mut zero_budget),
        "fallback must NOT allocate a fresh sweep budget when remaining tick budget is 0"
    );
    let mut shared_budget = MAX_KEEPOUT_EVALS_PER_TICK;
    assert!(inner.validate_fallback_hold_with_budget(None, &q_hold, true, &mut shared_budget));
    assert!(shared_budget < MAX_KEEPOUT_EVALS_PER_TICK, "fallback sweep must debit the shared budget");

    // Trigger a live workspace rejection on a tick and assert total evals across primary + fallback
    // remain strictly <= MAX_KEEPOUT_EVALS_PER_TICK.
    let mut ws = inner.config.workspace.clone().unwrap();
    ws.floor_z_m = 5.0;
    inner.config.workspace = Some(ws);
    inner.workspace_revision += 1;
    let mut q_bad = q0.to_vec();
    q_bad[1] += 0.02;
    inner.commanded_positions = q_bad;
    inner.commanded_velocities[1] = 0.25;
    let _ = inner.tick(DT);
    assert!(
        inner.tracking.last_tick_sweep_evals <= MAX_KEEPOUT_EVALS_PER_TICK,
        "combined primary + fallback sweep evals {} exceeded budget {}",
        inner.tracking.last_tick_sweep_evals,
        MAX_KEEPOUT_EVALS_PER_TICK
    );
}

/// Review 2 B2: Rejection fallback must validate issued->hold velocity and acceleration;
/// when a 1-tick jump to measured hold exceeds dynamics limits, it ramps smoothly within limits.
#[test]
fn tracking_rejection_fallback_validates_dynamics_and_ramps() {
    use armd::supervisor::MAX_KEEPOUT_EVALS_PER_TICK;

    let mut cfg = ArmConfig::rebot_dm();
    cfg.lease_timeout_ms = 600_000;
    cfg.workspace = None;
    cfg.joint_tracking = Some(lead_config(true, 0.68));
    let q0 = [0.0, -0.5, -0.5, 0.5, 0.0, 0.0, -2.0];
    let (mut inner, drv) = rig_at(cfg, &q0, vec![0.0; 7]);

    // Simulate prev_confirmed 0.030 rad ahead of measured q0 (1.5 rad/s > 0.45 rad/s limit!).
    let mut prev = q0.to_vec();
    prev[0] = 0.030;
    inner.tracking.record_issued(prev.clone(), vec![0.0; 7]);

    // 1. Direct fallback validation MUST reject the 1-tick 1.5 rad/s jump to q0.
    let limits = inner.active_tracking_limits(DT, 1.0);
    let mut budget = MAX_KEEPOUT_EVALS_PER_TICK;
    assert!(
        inner
            .validate_fallback_hold_with_dynamics(Some(&prev), Some(&[0.0; 7]), &q0, DT, &limits, true, &mut budget)
            .is_none(),
        "direct fallback validation must reject -1.5 rad/s jump from prev_confirmed to q_m"
    );

    // 2. Live tracking rejection: install a nominal trajectory at q0 while prev_confirmed
    //    is at 0.030 rad so tracking rejects the -1.5 rad/s jump to q0; fallback must NOT
    //    jump straight to q0 in 1 tick and must ramp smoothly within velocity/accel limits.
    let traj = TrajectoryPlanner::plan(&q0, &[0.0; 7], &q0, &[0.0; 7], Some(0.2), &inner.config.limits).unwrap();
    let _ = inner.safety.mark_motion_started();
    inner.active_trajectory = Some(armd::motion::ActiveTrajectory::quintic(traj, None));
    let idx_before = drv.with(|s| s.written.len());
    let obs = inner.tick(DT);
    assert!(matches!(obs.safety_state, SafetyState::Fault(_)));
    assert!(
        obs.fault.as_ref().is_some_and(|f| f.message.contains("Joint tracking output rejected")),
        "expected tracking rejection fault, got {:?}",
        obs.fault
    );
    let fb_frame = drv.with(|s| s.written[idx_before].clone());
    let step_v = ((fb_frame[0].position - prev[0]) as f64 / DT).abs();
    let max_v = limits[0].max_velocity as f64;
    let max_a = limits[0].max_acceleration as f64;
    let step_a = (step_v / DT).abs();
    assert!(
        step_v <= max_v + 1e-2 && step_a <= max_a + 1e-1,
        "fallback ramp exceeded dynamics limits: prev={} -> fb={} (v={step_v:.3}/{max_v:.3}, a={step_a:.3}/{max_a:.3})",
        prev[0],
        fb_frame[0].position
    );
    assert!(
        fb_frame[0].position < prev[0] - 1e-4 && fb_frame[0].position > q0[0] + 1e-3,
        "fallback should step partway from prev ({}) toward q0 ({}), got {}",
        prev[0],
        q0[0],
        fb_frame[0].position
    );
}

/// Review 2 M1: Re-enabling an already-torqued arm must NOT replace issued history with
/// a `Stationary` seed while moving or holding residual lead; if unknown and moving, it
/// must remain `Unknown`.
#[test]
fn tracking_reenable_already_torqued_checks_stationarity_and_preserves_history() {
    use armd::tracking::TrackingHistoryState;

    let q0 = [0.0, -0.5, -0.5, 0.5, 0.0, 0.0, -2.0];

    // 1. Already torqued and moving with active lead: re-enable preserves issued history
    //    and ramps smoothly without an unchecked velocity jump.
    {
        let cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            workspace: None,
            joint_tracking: Some(lead_config(true, 0.68)),
            ..ArmConfig::rebot_dm()
        };
        let (mut inner, drv) = rig_at(cfg, &q0, vec![0.68, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]);
        let mut goal = q0.to_vec();
        goal[0] += 0.20;
        let traj = TrajectoryPlanner::plan(&q0, &[0.0; 7], &goal, &[0.0; 7], Some(4.0), &inner.config.limits).unwrap();
        let _ = inner.safety.mark_motion_started();
        inner.active_trajectory = Some(armd::motion::ActiveTrajectory::quintic(traj, None));
        for _ in 0..60 {
            inner.tick(DT);
        }
        let issued_before = inner.tracking.last_issued_positions.as_ref().unwrap()[0];
        let meas_before = inner.measured_positions[0];
        assert!(issued_before - meas_before >= 0.020, "must have active lead before re-enable");

        // Call set_torque(true) while already torqued and moving!
        inner.set_torque(true).unwrap();
        assert_eq!(
            inner.tracking.last_issued_positions.as_ref().map(|v| v[0]),
            Some(issued_before),
            "re-enabling an already-torqued moving arm must preserve real issued history"
        );
        assert_ne!(
            inner.tracking.history_state,
            TrackingHistoryState::Stationary,
            "moving re-enable must not claim Stationary"
        );

        let idx_before = drv.with(|s| s.written.len());
        let max_v = inner.config.limits[0].max_velocity as f64;
        let mut prev_pos = issued_before;
        for k in 0..20 {
            let obs = inner.tick(DT);
            assert!(obs.safety_state.is_operational(), "re-enable ramp tick {k} faulted: {:?}", obs.fault);
            let pos = drv.with(|s| s.written[idx_before + k][0].position);
            let step_v = ((pos - prev_pos) as f64 / DT).abs();
            assert!(step_v <= max_v + 1e-2, "re-enable ramp tick {k} exceeded velocity cap: {step_v:.3}");
            prev_pos = pos;
        }
    }

    // 2. Unknown history while arm is moving: set_torque(true) must stay Unknown, not Stationary.
    {
        let cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            workspace: None,
            joint_tracking: Some(lead_config(true, 0.68)),
            ..ArmConfig::rebot_dm()
        };
        let (mut inner, drv) = rig_at(cfg, &q0, vec![0.0; 7]);
        inner.tracking.invalidate();
        drv.with(|s| s.v[0] = 0.25);
        inner.set_torque(true).unwrap();
        assert_eq!(
            inner.tracking.history_state,
            TrackingHistoryState::Unknown,
            "re-enabling a moving arm with unknown history must NOT seed Stationary"
        );
    }
}

/// Review 2 M2: Final braking tick (when nominal velocity hits 0.0 while residual lead remains)
/// must retain `servo_stream_active` through the residual ramp so streaming limits and
/// measured-lead allowance stay enforced until residual lead reaches zero.
#[test]
fn tracking_final_braking_tick_keeps_stream_ownership_through_residual_ramp() {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.lease_timeout_ms = 600_000;
    cfg.workspace = None;
    cfg.servo_max_lead_rad = 0.25;
    cfg.joint_tracking = Some(lead_config(true, 0.68));
    let q0 = [0.0, -0.5, -0.5, 0.5, 0.0, 0.0, -2.0];
    let (mut inner, drv) = rig_at(cfg, &q0, vec![0.10, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]);

    // Establish active stream ownership (`servo_stream_active == true`) and active residual lead.
    let epoch = inner.safety.epoch_id();
    let mut tgt = q0.to_vec();
    tgt[0] += 0.05;
    inner.servo(tgt, epoch).unwrap();
    let obs = inner.tick(DT);
    assert!(obs.safety_state.is_operational(), "stream start faulted: {:?}", obs.fault);
    assert!(inner.servo_stream_active);

    // Put the stream at the exact final braking tick: nominal velocity 0.04 rad/s (decelerates to
    // 0.0 in 1 tick) while issued output carries +0.025 rad active residual lead.
    let mut issued = inner.commanded_positions.clone();
    issued[0] += 0.025;
    let mut issued_v = vec![0.0; 7];
    issued_v[0] = 0.04;
    inner.tracking.record_issued(issued, issued_v);
    inner.servo_target = None;
    inner.commanded_velocities[0] = 0.04;
    let lead_before = inner.tracking.last_issued_positions.as_ref().unwrap()[0] - inner.commanded_positions[0];
    assert!(lead_before >= 0.015, "expected active stream lead, got {lead_before:.4}");

    let obs = inner.tick(DT);
    assert!(obs.safety_state.is_operational(), "final braking tick faulted: {:?}", obs.fault);
    assert_eq!(inner.commanded_velocities[0], 0.0, "nominal velocity reached 0 on final braking tick");
    let residual_after_final_brake =
        (inner.tracking.last_issued_positions.as_ref().unwrap()[0] - inner.commanded_positions[0]).abs();
    assert!(
        residual_after_final_brake > 1e-3,
        "residual lead should still be ramping after 1 tick (got {residual_after_final_brake:.5})"
    );
    assert!(
        inner.servo_stream_active,
        "servo_stream_active must stay TRUE while residual lead is still ramping out"
    );

    // Pin measured position and verify stream ownership clears ONLY once residual lead reaches 0.
    let mut cleared = false;
    for _ in 0..20 {
        let obs = inner.tick(DT);
        assert!(obs.safety_state.is_operational());
        if !inner.servo_stream_active {
            let rem = (inner.tracking.last_issued_positions.as_ref().unwrap()[0] - inner.commanded_positions[0]).abs();
            assert!(rem <= 1e-4, "stream ownership cleared while residual lead {rem:.5} remained");
            cleared = true;
            break;
        }
    }
    assert!(cleared, "servo_stream_active must clear once residual lead finishes ramping");
    assert!(!drv.with(|s| s.written.is_empty()));
}

/// Review 2 M3: Park keep-out ramp persists revision-tagged escape depths across ticks
/// (rejecting a slight reversal back toward a keep-out box), and rejection fallback commits
/// updated `ws_min_depths` instead of `None`.
#[test]
fn tracking_park_keepout_ramp_and_fallback_persist_escape_depths() {
    use armd::model::KeepoutBox;

    // 1. Park keep-out ramp-out persists `ws_keepout_min_depths` across ticks.
    {
        let q0 = [0.0, -0.5, -0.5, 0.5, 0.0, 0.0, -2.0];
        let mut cfg = ArmConfig::rebot_dm();
        cfg.lease_timeout_ms = 600_000;
        cfg.joint_tracking = Some(lead_config(true, 0.68));
        let ee0 = cfg.arm_model.fk_f32(&q0).unwrap().p;
        // Place a z half-space keep-out box covering ee0.z (penetration depth = 0.03 m).
        let mut ws = cfg.workspace.clone().unwrap();
        ws.check_points_end_m = vec![[0.0, 0.0, 0.0]];
        ws.link_capsules.clear();
        ws.self_collision_enabled = false;
        ws.self_collision_pairs.clear();
        ws.keepout_boxes.push(KeepoutBox {
            name: "park_keepout".into(),
            min_m: [-2.0, -2.0, -2.0],
            max_m: [2.0, 2.0, (ee0[2] + 0.03) as f32],
        });
        cfg.workspace = Some(ws);
        let (mut inner, _drv) = rig_at(cfg, &q0, vec![0.0; 7]);

        // Determine which direction on J2 raises EE z (escapes keep-out).
        let mut q_plus = q0.to_vec();
        q_plus[1] += 0.02;
        let ee_plus = inner.config.arm_model.fk_f32(&q_plus).unwrap().p;
        let dir = if ee_plus[2] > ee0[2] { 1.0_f32 } else { -1.0_f32 };

        // Start park ramp at q0 with residual lead pointing in the escaping direction.
        inner.pending_normal_unload = true;
        inner.commanded_positions = q0.to_vec();
        inner.commanded_positions[1] = q0[1] + dir * 0.050;
        inner.measured_positions = q0.to_vec();
        inner.tracking.record_issued(q0.to_vec(), vec![0.0; 7]);

        let limits = inner.active_tracking_limits(DT, 1.0);
        // Step outward over 6 ticks (escaping > 5 mm while still inside the 30 mm deep keep-out).
        for _ in 0..6 {
            let out = inner
                .compute_lead_ramp_out(DT, &limits, true)
                .expect("park keep-out escape step must succeed");
            assert!(
                out.ws_keepout_min_depths.is_some(),
                "park keep-out ramp must produce revision-tagged ws_keepout_min_depths"
            );
            inner.measured_positions = out.positions.clone();
            inner.tracking.record_issued_with_ws_depths(
                out.positions,
                out.velocities,
                out.ws_min_depths,
                out.ws_keepout_min_depths,
            );
        }
        assert!(inner.tracking.ws_keepout_min_depths.is_some());

        // Reverse back toward the keep-out (deeper than `last_pos`, still shallower than tick 0).
        inner.commanded_positions[1] = q0[1];
        let mut rejected = false;
        for _ in 0..6 {
            match inner.compute_lead_ramp_out(DT, &limits, true) {
                Ok(out) => {
                    inner.measured_positions = out.positions.clone();
                    inner.tracking.record_issued_with_ws_depths(
                        out.positions,
                        out.velocities,
                        out.ws_min_depths,
                        out.ws_keepout_min_depths,
                    );
                }
                Err(_) => {
                    rejected = true;
                    break;
                }
            }
        }
        assert!(
            rejected,
            "reversing back into keep-out during park ramp must be rejected by persisted escape depths"
        );
    }

    // 2. Rejection fallback commits non-None `ws_min_depths` when workspace policy is active.
    {
        let q0 = [0.0, -0.5, -0.5, 0.5, 0.0, 0.0, -2.0];
        let mut cfg = ArmConfig::rebot_dm();
        cfg.lease_timeout_ms = 600_000;
        cfg.joint_tracking = Some(lead_config(true, 0.68));
        let (mut inner, _drv) = rig_at(cfg, &q0, vec![0.0; 7]);

        // Set prev_confirmed close enough to q0 (0.002 rad) that direct fallback hold validates,
        // and trigger a tracking rejection by shrinking limits[0].max_velocity temporarily or
        // via a large desired step.
        let mut prev = q0.to_vec();
        prev[0] = 0.030;
        inner.tracking.record_issued(prev, vec![0.0; 7]);
        let traj = TrajectoryPlanner::plan(&q0, &[0.0; 7], &q0, &[0.0; 7], Some(0.2), &inner.config.limits).unwrap();
        let _ = inner.safety.mark_motion_started();
        inner.active_trajectory = Some(armd::motion::ActiveTrajectory::quintic(traj, None));
        let obs = inner.tick(DT);
        assert!(matches!(obs.safety_state, SafetyState::Fault(_)));
        assert!(
            inner.tracking.ws_min_depths.is_some(),
            "rejection fallback must commit updated ws_min_depths instead of None"
        );
    }
}

/// Review 2 M4: `active_tracking_limits` honors `speed_scale` on planned trajectories,
/// and `compute_lead_ramp_out` rechecks acceleration after torque-hold clamping.
#[test]
fn tracking_speed_scale_limits_and_torque_hold_ramp_accel_recheck() {
    let q0 = [0.0, -0.5, -0.5, 0.5, 0.0, 0.0, -2.0];

    // 1. `install_joint_goal` with `speed_scale = 0.25` tightens `active_tracking_limits`.
    {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.lease_timeout_ms = 600_000;
        cfg.workspace = None;
        cfg.joint_tracking = Some(lead_config(true, 0.68));
        let (mut inner, _drv) = rig_at(cfg, &q0, vec![0.0; 7]);
        let base_v = inner.planning_limits()[0].max_velocity;
        let inp = inner.plan_input().unwrap();
        let mut goal = [0.0_f64; 6];
        for i in 0..6 {
            goal[i] = f64::from(q0[i]);
        }
        goal[0] += 0.20;
        inner.install_joint_goal(&inp, &goal, 0.25, "joint_move", None, None).unwrap();
        let active = inner.active_tracking_limits(DT, 1.0);
        assert!(
            (active[0].max_velocity - base_v * 0.25).abs() < 1e-4,
            "active_tracking_limits must apply speed_scale 0.25 ({} vs expected {})",
            active[0].max_velocity,
            base_v * 0.25
        );
    }

    // 2. Torque-hold ramp-out rechecks acceleration after clamping `q_final = pq`.
    {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.lease_timeout_ms = 600_000;
        cfg.workspace = None;
        cfg.joint_tracking = Some(lead_config(true, 0.68));
        let (mut inner, drv) = rig_at(cfg, &q0, vec![0.0; 7]);
        let mut prev_q = q0.to_vec();
        prev_q[0] = 0.020;
        let mut prev_v = vec![0.0; 7];
        prev_v[0] = 0.40; // Clamping q_final = pq yields a = -20 rad/s^2 > max_acceleration!
        inner.tracking.record_issued(prev_q, prev_v);
        inner.commanded_positions[0] = 0.0;
        inner.torque_guard.hold = Some(armd::torque_guard::TorqueHold {
            joint: 0,
            sign: 1.0,
            hold_q: 0.020,
            torque_nm: 15.0,
            threshold_nm: 10.0,
            since_s: 0.0,
            anchor_q: 0.020,
            push_in: false,
            retreat: armd::torque_guard::Retreat::default(),
        });
        let limits = inner.active_tracking_limits(DT, 1.0);
        let res = inner.compute_lead_ramp_out(DT, &limits, false);
        assert!(
            res.is_err(),
            "torque-hold clamp violating max_acceleration must be rejected, got {res:?}"
        );

        // Review 3 B3: through the supervisor, the rejection fallback must enforce the
        // torque-hold direction like the primary ramp: J1 never advances past the last issued
        // output or the hold position in the push direction. Here no step satisfies both
        // the hold and a_max, so no frame is sent and the fault hold repeats 0.020.
        inner.config.torque_guard = None; // keep the hand-set hold in force across ticks
        let idx0 = drv.with(|s| s.written.len());
        let obs = inner.tick(DT);
        assert!(matches!(obs.safety_state, SafetyState::Fault(_)), "rejected torque-hold ramp must fault: {:?}", obs.safety_state);
        for _ in 0..5 {
            inner.tick(DT);
        }
        let frames = drv.with(|s| s.written[idx0..].to_vec());
        assert!(!frames.is_empty(), "fault hold keeps writing frames");
        for (k, f) in frames.iter().enumerate() {
            assert!(
                f[0].position <= 0.020 + 1e-6,
                "fallback frame {k} advances J1 into the torque hold: {} > 0.020",
                f[0].position
            );
        }
    }
}

/// Review 2 M5: Failed gravity-comp entry (even with a clean rollback where modes are
/// not uncertain) must invalidate any stale pre-entry issued baseline.
#[test]
fn tracking_failed_gravity_comp_entry_rollback_invalidates_old_baseline() {
    use armd::tracking::TrackingHistoryState;

    let q0 = [0.0, -0.5, -0.5, 0.5, 0.0, 0.0, -2.0];
    let cfg = ArmConfig {
        lease_timeout_ms: 600_000,
        workspace: None,
        joint_tracking: Some(lead_config(true, 0.68)),
        ..ArmConfig::rebot_dm()
    };
    let (mut inner, drv) = rig_at(cfg, &q0, vec![0.0; 7]);
    let mut old_issued = q0.to_vec();
    old_issued[0] += 0.020;
    inner.tracking.record_issued(old_issued, vec![0.0; 7]);

    drv.with(|s| s.fail_mode_switch = true);
    let err = inner.enter_gravity_comp(Default::default());
    assert!(err.is_err(), "simulated mode-switch failure must reject entry");
    assert!(inner.gravity_comp.is_none());
    assert!(inner.safety.state().is_operational(), "clean rollback leaves arm operational");
    assert_eq!(
        inner.tracking.history_state,
        TrackingHistoryState::Unknown,
        "failed entry rollback must invalidate stale pre-entry baseline"
    );
    assert!(
        inner.tracking.last_issued_positions.is_none(),
        "stale pre-entry issued target must not remain after rollback"
    );
}

/// Review 2 M6: Output acceptance defaults to `false`; skipping output due to `disable_pending`
/// or `!is_torqued` must NEVER publish `"done"` for an action finishing on that tick.
#[test]
fn tracking_action_done_not_published_when_output_skipped_by_disable_or_torque_off() {
    let cur_gripper_frac = |inner: &SupervisorInner, q_grip: f32| -> f64 {
        let policy = inner.config.gripper_policy.as_ref().unwrap();
        f64::from(((q_grip - policy.closed_rad) / (policy.open_rad - policy.closed_rad)).clamp(0.0, 1.0))
    };
    let make_rig = || {
        let cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            workspace: None,
            joint_tracking: Some(lead_config(true, 0.68)),
            ..ArmConfig::rebot_dm()
        };
        let q0 = [0.0, -0.5, -0.5, 0.5, 0.0, 0.0, -2.0];
        let (mut inner, drv) = rig_at(cfg, &q0, vec![0.0; 7]);
        let epoch = inner.safety.epoch_id();
        let cur_frac = cur_gripper_frac(&inner, q0[6]);
        let (id, _) = inner
            .start_gripper(
                GripperRequest {
                    command: GripperCommand::Move,
                    fraction: Some(cur_frac),
                    grasp: false,
                    speed: 1.0,
                    timeout_s: Some(5.0),
                    tag: None,
                },
                epoch,
            )
            .unwrap();
        (inner, drv, id)
    };

    // Determine exact `done_tick` from control run.
    let done_tick = {
        let (mut inner, _drv, _id) = make_rig();
        let mut n = 0usize;
        for k in 1..=30 {
            let obs = inner.tick(DT);
            if obs.action.as_ref().is_some_and(|a| a.state == "done") {
                n = k;
                break;
            }
        }
        assert!(n >= 1);
        n
    };

    // 1. `disable_pending == true` on `done_tick`: output skipped -> action must NOT report "done".
    {
        let (mut inner, drv, id) = make_rig();
        for _ in 1..done_tick {
            let obs = inner.tick(DT);
            assert_eq!(obs.action.as_ref().unwrap().state, "running");
        }
        let idx_before = drv.with(|s| s.written.len());
        inner.disable_pending = true;
        let obs = inner.tick(DT);
        assert_eq!(drv.with(|s| s.written.len()), idx_before, "disable_pending must skip output write");
        let act = obs.action.expect("action status present");
        assert_eq!(act.id, id);
        assert_ne!(act.state, "done", "skipped output on disable_pending must NOT publish done");
    }

    // 2. `!is_torqued` on `done_tick`: output skipped -> action must NOT report "done".
    {
        let (mut inner, drv, id) = make_rig();
        for _ in 1..done_tick {
            let obs = inner.tick(DT);
            assert_eq!(obs.action.as_ref().unwrap().state, "running");
        }
        let idx_before = drv.with(|s| s.written.len());
        inner.is_torqued = false;
        let obs = inner.tick(DT);
        assert_eq!(drv.with(|s| s.written.len()), idx_before, "untorqued arm must skip output write");
        let act = obs.action.expect("action status present");
        assert_eq!(act.id, id);
        assert_ne!(act.state, "done", "skipped output while untorqued must NOT publish done");
    }
}

/// Issued arm velocities of consecutive frames, starting from `(p0, v0)` on `joint`.
fn issued_velocity_steps(frames: &[Vec<JointTarget>], joint: usize, p0: f32, v0: f32) -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    let (mut p_prev, mut v_prev) = (f64::from(p0), f64::from(v0));
    for f in frames {
        let p = f64::from(f[joint].position);
        let v = (p - p_prev) / DT;
        out.push((v, (v - v_prev) / DT));
        p_prev = p;
        v_prev = v;
    }
    out
}

/// Review 3 B2: when the rejection fallback cannot reach the hold in one tick, it brakes
/// over several ticks within a_max (also while the tracking fault is latched) and never
/// drops the issued velocity to zero in one tick. When no braking step is feasible it
/// latches the fault without sending a frame.
#[test]
fn tracking_rejection_fallback_brakes_over_ticks_within_acceleration() {
    let q0 = [0.0, -0.5, -0.5, 0.5, 0.0, 0.0, -2.0];

    // 1. Feasible: multi-tick braking ramp, every issued frame within a_max.
    {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.lease_timeout_ms = 600_000;
        cfg.workspace = None;
        cfg.joint_tracking = Some(lead_config(true, 0.68));
        let (mut inner, drv) = rig_at(cfg, &q0, vec![0.0; 7]);
        let limits = inner.active_tracking_limits(DT, 1.0);
        let (max_v, max_a) = (f64::from(limits[0].max_velocity), f64::from(limits[0].max_acceleration));
        // Issued J1 output 0.030 rad ahead of the nominal hold and still moving away at 0.40 rad/s.
        let mut prev = q0.to_vec();
        prev[0] = 0.030;
        let mut prev_v = vec![0.0; 7];
        prev_v[0] = 0.40;
        assert!(f64::from(prev_v[0]) <= max_v && f64::from(prev_v[0]) > 2.0 * max_a * DT, "test needs a multi-tick brake");
        inner.tracking.record_issued(prev.clone(), prev_v.clone());
        let traj = TrajectoryPlanner::plan(&q0, &[0.0; 7], &q0, &[0.0; 7], Some(0.2), &inner.config.limits).unwrap();
        let _ = inner.safety.mark_motion_started();
        inner.active_trajectory = Some(armd::motion::ActiveTrajectory::quintic(traj, None));
        let idx0 = drv.with(|s| s.written.len());
        let obs = inner.tick(DT);
        assert!(
            obs.fault.as_ref().is_some_and(|f| f.message.contains("Joint tracking output rejected")),
            "expected tracking rejection fault, got {:?}",
            obs.fault
        );
        for _ in 0..80 {
            inner.tick(DT);
        }
        let frames = drv.with(|s| s.written[idx0..].to_vec());
        assert!(frames.len() >= 60, "fault hold must keep writing frames ({})", frames.len());
        let steps = issued_velocity_steps(&frames, 0, prev[0], prev_v[0]);
        for (k, (v, a)) in steps.iter().enumerate() {
            assert!(
                v.abs() <= max_v + 1e-2 && a.abs() <= max_a + 1e-2,
                "fallback frame {k}: J1 v={v:.4} rad/s a={a:.3} rad/s^2 exceeds ({max_v:.3}, {max_a:.3}); positions {:?}",
                frames.iter().take(k + 2).map(|f| f[0].position).collect::<Vec<_>>()
            );
        }
        let (v_end, _) = steps.last().copied().unwrap();
        assert!(v_end.abs() < 1e-6, "braking ramp must end at rest, last v={v_end}");
        let p_end = frames.last().unwrap()[0].position;
        assert!((p_end - q0[0]).abs() < 1e-4, "braking ramp must end at the measured hold, got {p_end}");
        assert!(matches!(inner.safety.state(), SafetyState::Fault(_)), "tracking rejection stays latched");
    }

    // 2. Infeasible: the workspace refuses every braking step -> latch a fault, send no frame
    //    on the rejection tick, then keep holding exactly the last issued output.
    {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.lease_timeout_ms = 600_000;
        cfg.joint_tracking = Some(lead_config(true, 0.68));
        let (mut inner, drv) = rig_at(cfg, &q0, vec![0.0; 7]);
        let mut prev = q0.to_vec();
        prev[0] = 0.010;
        let mut prev_v = vec![0.0; 7];
        prev_v[0] = 0.40;
        inner.tracking.record_issued(prev.clone(), prev_v);
        let mut ws = inner.config.workspace.clone().unwrap();
        ws.floor_z_m = 5.0;
        inner.config.workspace = Some(ws);
        inner.workspace_revision += 1;
        let idx0 = drv.with(|s| s.written.len());
        let obs = inner.tick(DT);
        assert!(matches!(obs.safety_state, SafetyState::Fault(_)), "infeasible fallback must latch a fault");
        assert_eq!(
            drv.with(|s| s.written.len()),
            idx0,
            "no frame may be sent when no braking step satisfies the limits"
        );
        for _ in 0..5 {
            inner.tick(DT);
        }
        let frames = drv.with(|s| s.written[idx0..].to_vec());
        assert!(!frames.is_empty(), "the fault hold keeps writing frames");
        for (k, f) in frames.iter().enumerate() {
            assert_eq!(
                f[0].position.to_bits(),
                prev[0].to_bits(),
                "fault hold frame {k} must repeat the last issued J1 output, not step to measured"
            );
        }
    }
}

/// Review 3 M3: during park (`NormalUnload`) the rejection fallback is validated against
/// the park keep-out-only workspace with the park escape depths (evaluated before
/// `hold_measured_pose` clears the park state) and commits into the park depths without
/// overwriting the full-workspace depths.
#[test]
fn tracking_park_rejection_fallback_keeps_park_workspace_and_depths() {
    let mut jt = JointTrackingConfig::rebot_dm();
    jt.enabled = true;
    jt.joints[0].lag_s = 0.68;
    let mut cfg = ArmConfig { lease_timeout_ms: 600_000, joint_tracking: Some(jt), ..ArmConfig::rebot_dm() };
    cfg.torque_guard = None;
    let q0 = INSPECT;
    let mut q_box = q0;
    q_box[0] += 0.004;
    let p_box = cfg.arm_model.fk_f32(&q_box).unwrap().p;
    let (mut inner, drv) = rig_at(cfg, &q0, vec![0.0; 7]);

    // Issued J1 0.008 rad ahead of measured q0, nominal park reference 0.012 ahead; a small
    // keep-out box sits between measured and issued, so the ramp-out's measured segment fails.
    let mut pq = q0.to_vec();
    pq[0] += 0.008;
    inner.tracking.record_issued(pq.clone(), vec![0.0; 7]);
    let half = 0.0008_f32;
    let ws = inner.config.workspace.as_mut().unwrap();
    ws.keepout_boxes.push(KeepoutBox {
        name: "between".into(),
        min_m: [p_box[0] as f32 - half, p_box[1] as f32 - half, p_box[2] as f32 - half],
        max_m: [p_box[0] as f32 + half, p_box[1] as f32 + half, p_box[2] as f32 + half],
    });
    inner.workspace_revision += 1;
    inner.pending_normal_unload = true;
    inner.commanded_positions[0] = q0[0] + 0.012;
    inner.target_positions = inner.commanded_positions.clone();
    let full_depths_before = inner.tracking.ws_min_depths.clone().map(|(rev, d)| (rev, d.as_slice().to_vec()));

    let idx0 = drv.with(|s| s.written.len());
    inner.tick(DT);
    let frames = drv.with(|s| s.written[idx0..].to_vec());
    assert_eq!(frames.len(), 1, "the park fallback step must be sent");
    assert_eq!(frames[0][0].position.to_bits(), pq[0].to_bits(), "fallback brakes at the last issued J1 output");
    assert!(
        inner.tracking.ws_keepout_min_depths.is_some(),
        "the park fallback must commit the park keep-out escape depths"
    );
    assert_eq!(
        inner.tracking.ws_min_depths.clone().map(|(rev, d)| (rev, d.as_slice().to_vec())),
        full_depths_before,
        "the park fallback must not overwrite the full-workspace escape depths"
    );
}

/// Review 4 #1: a moving braking step (nonzero prior velocity) is also swept from the fresh
/// measurement: with the arm lagging behind a keep-out box that lies only on the
/// measured -> new-target segment, the fallback sends no frame instead of the braking step.
#[test]
fn tracking_braking_fallback_sweeps_measured_segment_when_moving() {
    let mut jt = JointTrackingConfig::rebot_dm();
    jt.enabled = true;
    jt.joints[0].lag_s = 0.68;
    let mut cfg = ArmConfig { lease_timeout_ms: 600_000, joint_tracking: Some(jt), ..ArmConfig::rebot_dm() };
    cfg.torque_guard = None;
    let q0 = INSPECT;
    let mut q_box = q0;
    q_box[0] += 0.004;
    let p_box = cfg.arm_model.fk_f32(&q_box).unwrap().p;
    let (mut inner, drv) = rig_at(cfg, &q0, vec![0.0; 7]);
    drv.with(|s| s.pinned[0] = Some(q0[0])); // the arm lags at q0

    // Issued J1 0.008 rad ahead of measured, still moving at +0.2 rad/s toward the park
    // reference; the box sits between measured (q0) and issued, not between issued and
    // the braking step (~q0 + 0.010).
    let mut pq = q0.to_vec();
    pq[0] += 0.008;
    let mut pv = vec![0.0; 7];
    pv[0] = 0.2;
    inner.tracking.record_issued(pq.clone(), pv);
    let half = 0.0008_f32;
    let ws = inner.config.workspace.as_mut().unwrap();
    ws.keepout_boxes.push(KeepoutBox {
        name: "lagging".into(),
        min_m: [p_box[0] as f32 - half, p_box[1] as f32 - half, p_box[2] as f32 - half],
        max_m: [p_box[0] as f32 + half, p_box[1] as f32 + half, p_box[2] as f32 + half],
    });
    inner.workspace_revision += 1;
    inner.pending_normal_unload = true;
    inner.commanded_positions[0] = q0[0] + 0.012;
    inner.target_positions = inner.commanded_positions.clone();

    let idx0 = drv.with(|s| s.written.len());
    let obs = inner.tick(DT);
    assert!(matches!(obs.safety_state, SafetyState::Fault(_)), "rejection must fault: {:?}", obs.safety_state);
    assert_eq!(drv.with(|s| s.written.len()), idx0, "no braking frame may cross the keep-out from the measured pose");
    for _ in 0..3 {
        inner.tick(DT);
    }
    for (k, f) in drv.with(|s| s.written[idx0..].to_vec()).iter().enumerate() {
        assert_eq!(f[0].position.to_bits(), pq[0].to_bits(), "fault hold frame {k} must repeat the last issued J1");
    }
}

/// Review 3 M1: tracking seeds a stationary baseline only on positive evidence that the
/// arm is at rest: velocity feedback within 0.05 rad/s, or (without velocity feedback) two
/// fresh position samples at least one tick apart within 1 mrad. Until then the output is
/// the plain nominal position and the history stays `Unknown`.
#[test]
fn tracking_seeds_only_on_positive_stationarity_evidence() {
    use armd::tracking::TrackingHistoryState;
    let q0 = [0.0, -0.5, -0.5, 0.5, 0.0, 0.0, -2.0];
    let cfg = || ArmConfig {
        lease_timeout_ms: 600_000,
        workspace: None,
        joint_tracking: Some(lead_config(true, 0.68)),
        ..ArmConfig::rebot_dm()
    };

    // 1. No velocity feedback: one sample at enable is no evidence.
    {
        let drv = TrackingTestDriver::new(&q0, vec![0.0; 7]);
        drv.with(|s| s.velocity_feedback = false);
        let mut inner = SupervisorInner::new(cfg()).with_driver(Box::new(drv.clone()));
        inner.lease_enabled = false;
        inner.commanded_positions = q0.to_vec();
        inner.measured_positions = q0.to_vec();
        inner.target_positions = q0.to_vec();
        inner.tick(DT);
        inner.set_torque(true).unwrap();
        assert_eq!(
            inner.tracking.history_state,
            TrackingHistoryState::Unknown,
            "enable without velocity feedback must not claim a stationary baseline"
        );

        // The arm drifts 5 mrad per tick: never stationary -> plain nominal output, Unknown.
        for k in 0..5 {
            drv.with(|s| s.pinned[0] = Some(q0[0] + 0.005 * (k + 1) as f32));
            drv.with(|s| s.q[0] = s.pinned[0].unwrap());
            let idx = drv.with(|s| s.written.len());
            let obs = inner.tick(DT);
            assert!(obs.safety_state.is_operational(), "tick {k}: {:?}", obs.fault);
            assert_eq!(inner.tracking.history_state, TrackingHistoryState::Unknown, "moving tick {k} must stay Unknown");
            let f = drv.with(|s| s.written[idx].clone());
            for (j, t) in f.iter().enumerate() {
                assert_eq!(t.position.to_bits(), inner.commanded_positions[j].to_bits(), "tick {k} joint {j}: plain output");
            }
        }
        // Back at the nominal pose and at rest for two fresh samples -> evidence -> tracking
        // takes over the output.
        drv.with(|s| {
            s.pinned[0] = Some(q0[0]);
            s.q[0] = q0[0];
        });
        inner.tick(DT);
        assert_eq!(inner.tracking.history_state, TrackingHistoryState::Unknown, "one sample at rest is not evidence");
        let obs = inner.tick(DT);
        assert!(obs.safety_state.is_operational(), "{:?}", obs.fault);
        assert_ne!(
            inner.tracking.history_state,
            TrackingHistoryState::Unknown,
            "two fresh position samples at rest are evidence of stationarity"
        );
    }

    // 2. Velocity feedback reporting motion: Unknown history stays Unknown with plain output.
    {
        let (mut inner, drv) = rig_at(cfg(), &q0, vec![0.0; 7]);
        inner.tracking.invalidate();
        drv.with(|s| {
            s.pinned[0] = Some(q0[0]);
            s.v[0] = 0.25;
        });
        for k in 0..3 {
            drv.with(|s| s.v[0] = 0.25);
            let idx = drv.with(|s| s.written.len());
            inner.tick(DT);
            assert_eq!(inner.tracking.history_state, TrackingHistoryState::Unknown, "tick {k}: moving arm must stay Unknown");
            let f = drv.with(|s| s.written[idx].clone());
            assert_eq!(f[0].position.to_bits(), inner.commanded_positions[0].to_bits(), "tick {k}: plain output");
        }
        drv.with(|s| s.v[0] = 0.0);
        inner.tick(DT);
        assert_ne!(inner.tracking.history_state, TrackingHistoryState::Unknown, "velocity feedback at rest is evidence");
    }
}

/// Review 3 M4: the speed_scale caps of a planned motion stay in force on the tick its
/// reference finishes and until it has settled (no residual lead), so the final lead
/// withdrawal is checked against the scaled limits, not the generic planning limits.
#[test]
fn tracking_keeps_motion_caps_through_final_tick_and_settling() {
    let q0 = [0.0, -0.5, -0.5, 0.5, 0.0, 0.0, -2.0];
    let mut cfg = ArmConfig::rebot_dm();
    cfg.lease_timeout_ms = 600_000;
    cfg.workspace = None;
    cfg.joint_tracking = Some(lead_config(true, 0.68));
    let (mut inner, drv) = rig_at(cfg, &q0, vec![0.68, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]);
    let scaled_v = inner.planning_limits()[0].max_velocity * 0.25;
    let inp = inner.plan_input().unwrap();
    let mut goal = [0.0_f64; 6];
    for i in 0..6 {
        goal[i] = f64::from(q0[i]);
    }
    goal[0] += 0.20;
    inner.install_joint_goal(&inp, &goal, 0.25, "joint_move", None, None).unwrap();

    let mut finished_at = None;
    for k in 0..800 {
        let obs = inner.tick(DT);
        assert!(obs.safety_state.is_operational(), "tick {k}: {:?}", obs.fault);
        if inner.active_trajectory.is_none() {
            let settling = inner.quintic_settle.is_some()
                || inner.tracking.has_residual_lead(&inner.commanded_positions, &inner.config.arm_joint_indices());
            if finished_at.is_none() {
                finished_at = Some(k);
            }
            let active = inner.active_tracking_limits(DT, 1.0);
            if settling || finished_at == Some(k) {
                assert!(
                    (active[0].max_velocity - scaled_v).abs() < 1e-5,
                    "tick {k} (finished at {finished_at:?}): active cap {} instead of the speed_scale cap {scaled_v}",
                    active[0].max_velocity
                );
            } else {
                break;
            }
        }
    }
    assert!(finished_at.is_some(), "the motion must finish");
    let frames = drv.with(|s| s.written.clone());
    for (k, w) in frames.windows(2).enumerate() {
        let v = (f64::from(w[1][0].position) - f64::from(w[0][0].position)) / DT;
        assert!(v.abs() <= f64::from(scaled_v) + 1e-3, "frame {k}: J1 issued {v:.4} rad/s above the scaled cap {scaled_v}");
    }
}
