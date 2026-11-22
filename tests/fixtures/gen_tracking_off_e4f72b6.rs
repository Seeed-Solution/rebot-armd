use std::sync::{Arc, Mutex};
use std::time::Duration;

use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointMode, JointTarget};
use armd::model::{ArmConfig, ArmObservation, JointState};
use armd::motion::{plan_cartesian, GripperCommand, GripperRequest, Speed};
use armd::replay::{plan_frames, ReplayRequest};
use armd::safety::StopPolicy;
use armd::supervisor::SupervisorInner;
use armd::RecordFrame;
use serde::Serialize;

const DT: f64 = 0.02;

struct TrackingTestState {
    dof: usize,
    q: Vec<f32>,
    v: Vec<f32>,
    tau: Vec<f32>,
    enabled: EnableState,
    tau_s: Vec<f32>,
    pinned: Vec<Option<f32>>,
    forced_torque: Vec<Option<f32>>,
    gripper_block: Option<(usize, f32, f32)>,
    feedback_age: Duration,
    written: Vec<Vec<JointTarget>>,
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

#[derive(Serialize)]
struct RecordedTarget {
    p: u32,
    v: Option<u32>,
    t: Option<u32>,
    kp: Option<u32>,
    kd: Option<u32>,
}

#[derive(Serialize)]
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

#[derive(Serialize)]
struct RecordedTrace {
    source_commit: String,
    frames: Vec<Vec<RecordedTarget>>,
    obs: Vec<RecordedObs>,
}

fn encode_obs(o: &ArmObservation) -> RecordedObs {
    RecordedObs {
        positions: o.positions.iter().map(|x| x.to_bits()).collect(),
        velocities: o.velocities.iter().map(|x| x.to_bits()).collect(),
        torques: o.torques.iter().map(|x| x.to_bits()).collect(),
        commanded_positions: o.commanded_positions.iter().map(|x| x.to_bits()).collect(),
        is_torqued: o.is_torqued,
        safety_state: format!("{:?}", o.safety_state),
        fault: o.fault.as_ref().map(|f| format!("{f:?}")),
        gripper: o.gripper.as_ref().map(|g| format!("{g:?}")),
        motion_limit: o.motion_limit.as_ref().map(|m| format!("{:?}:{:?}", m.reason, m.joint)),
    }
}

#[test]
fn generate_e4f72b6_fixture() {
    let out_path = std::env::var("TRACKING_FIXTURE_OUT")
        .expect("TRACKING_FIXTURE_OUT env var must be set by generator script");
    let source_commit = std::env::var("TRACKING_FIXTURE_COMMIT")
        .unwrap_or_else(|_| "e4f72b6".to_string());

    let cfg = ArmConfig {
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    };
    let q0 = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0, -4.0];
    let drv = TrackingTestDriver::new(&q0, vec![0.25, 0.25, 0.25, 0.10, 0.05, 0.05, 0.15]);
    let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv.clone()));
    inner.lease_enabled = false;
    inner.commanded_positions = q0.to_vec();
    inner.measured_positions = q0.to_vec();
    inner.target_positions = q0.to_vec();
    inner.tick(DT);
    inner.set_torque(true).unwrap();
    inner.tick(DT);
    assert!(inner.safety.state().is_operational(), "{:?}", inner.safety.state());

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

    // 4. Servo streaming + 5. Braking
    let epoch = inner.safety.epoch_id();
    let mut target = inner.commanded_positions.clone();
    for _ in 0..20 {
        target[0] += 0.004;
        inner.servo(target.clone(), epoch).unwrap();
        step(&mut inner, 1);
    }
    inner.servo_target = None;
    step(&mut inner, 40);

    // 6. Jaw contact
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
    let trace = RecordedTrace {
        source_commit,
        frames: frames
            .iter()
            .map(|f| {
                f.iter()
                    .map(|t| RecordedTarget {
                        p: t.position.to_bits(),
                        v: t.velocity.map(f32::to_bits),
                        t: t.torque.map(f32::to_bits),
                        kp: t.kp.map(f32::to_bits),
                        kd: t.kd.map(f32::to_bits),
                    })
                    .collect()
            })
            .collect(),
        obs: obs_log.iter().map(encode_obs).collect(),
    };
    if let Some(parent) = std::path::Path::new(&out_path).parent() {
        std::fs::create_dir_all(parent).unwrap();
    }
    let json = serde_json::to_string(&trace).unwrap();
    std::fs::write(&out_path, json).unwrap();
    eprintln!("Wrote {} frames and {} obs to {out_path}", trace.frames.len(), trace.obs.len());
}
