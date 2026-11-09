//! Gravity compensation ("hand-guiding") mode with a simulated MIT-capable arm.
//!
//! `SimArm` integrates each arm joint as a decoupled inertia under the URDF gravity
//! torque times `1 + model_error`, so a wrong feed-forward makes the arm sag or fall.
//! POS_VEL joints are an ideal position servo. The numbers printed by these tests are
//! simulation results, not hardware measurements.
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointMode, JointTarget};
use armd::model::{ArmConfig, ControlMode, JointState};
use armd::supervisor::ArmError;
use armd::{ArmSupervisor, SafetyState};

const DT: f64 = 0.02;
const SUB: usize = 20; // 1 ms physics substeps per tick
const INERTIA: [f32; 6] = [0.15, 0.25, 0.12, 0.03, 0.01, 0.005];
const VISCOUS: f32 = 0.02;

struct Sim {
    q: [f32; 7],
    v: [f32; 7],
    modes: [JointMode; 7],
    cmd: Vec<JointTarget>,
    torqued: bool,
    model_error: f32,
    user_torque: [f32; 6],
    /// No feedback reaches the host (ages grow, mode switches are not acknowledged).
    feedback_dead_since: Option<Instant>,
    /// Commands do not reach the motors either (full bus loss).
    commands_dead: bool,
    hw_fault: bool,
    switches: Vec<Vec<JointMode>>,
    /// Substeps in which a loaded MIT joint had neither stiffness nor feed-forward.
    unsupported_substeps: u32,
    max_abs_tau_cmd: [f32; 6],
    motor_torque: [f32; 7],
}

impl Sim {
    fn new(model_error: f32) -> Self {
        let q = [0.0, 0.0, 0.0, 0.0, 0.0, std::f32::consts::FRAC_PI_2, 0.0];
        Self {
            q, v: [0.0; 7], modes: [JointMode::Position; 7], cmd: q.iter().map(|&p| JointTarget::position(p)).collect(),
            torqued: false, model_error, user_torque: [0.0; 6], feedback_dead_since: None, commands_dead: false,
            hw_fault: false, switches: Vec::new(), unsupported_substeps: 0, max_abs_tau_cmd: [0.0; 6], motor_torque: [0.0; 7],
        }
    }

    fn gravity(&self) -> [f32; 6] {
        let mut q6 = [0.0; 6];
        q6.copy_from_slice(&self.q[..6]);
        let g = armd::compute_gravity_torques(&q6).unwrap();
        g.map(|t| t * (1.0 + self.model_error))
    }

    fn step(&mut self, h: f32) {
        let g = self.gravity();
        for i in 0..7 {
            let target = self.cmd[i].clone();
            if !self.torqued {
                if i < 6 {
                    let a = (-g[i] - VISCOUS * self.v[i]) / INERTIA[i];
                    self.v[i] += a * h;
                    self.q[i] += self.v[i] * h;
                }
                continue;
            }
            match (self.modes[i], i < 6) {
                (JointMode::Mit, true) => {
                    let kp = target.kp.unwrap_or(0.0);
                    let kd = target.kd.unwrap_or(0.0);
                    let tff = target.torque.unwrap_or(0.0);
                    if kp == 0.0 && tff == 0.0 && g[i].abs() > 0.1 { self.unsupported_substeps += 1; }
                    let tau = (kp * (target.position - self.q[i]) + kd * (target.velocity.unwrap_or(0.0) - self.v[i]) + tff).clamp(-18.0, 18.0);
                    self.motor_torque[i] = tau;
                    let a = (tau - g[i] + self.user_torque[i] - VISCOUS * self.v[i]) / INERTIA[i];
                    self.v[i] += a * h;
                    self.q[i] += self.v[i] * h;
                }
                _ => {
                    // POS_VEL: ideal position servo, 2 rad/s limit; it carries the load.
                    let v = ((target.position - self.q[i]) / 0.05).clamp(-2.0, 2.0);
                    self.v[i] = v;
                    self.q[i] += v * h;
                    if i < 6 { self.motor_torque[i] = g[i]; }
                }
            }
        }
    }
}

#[derive(Clone)]
struct SimArm(Arc<Mutex<Sim>>);

impl ArmDriver for SimArm {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true, torque_feedforward: false, enable_feedback: true }
    }
    fn enable_state(&self) -> EnableState {
        if self.0.lock().unwrap().torqued { EnableState::Enabled } else { EnableState::Disabled }
    }
    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        let age = self.0.lock().unwrap().feedback_dead_since.map_or(Duration::ZERO, |t| t.elapsed());
        vec![Some(age); 7]
    }
    fn is_feedback_valid(&self) -> bool {
        self.0.lock().unwrap().feedback_dead_since.is_none_or(|t| t.elapsed() < Duration::from_millis(250))
    }
    fn is_enabled(&self) -> bool { self.0.lock().unwrap().torqued }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let mut s = self.0.lock().unwrap();
        for _ in 0..SUB { s.step((DT / SUB as f64) as f32); }
        Ok((0..7).map(|i| JointState { position: s.q[i], velocity: s.v[i], torque: s.motor_torque[i], temperature: 30.0 }).collect())
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        let mut s = self.0.lock().unwrap();
        if s.commands_dead { return Ok(()); }
        for (i, t) in targets.iter().enumerate().take(6) {
            if s.modes[i] == JointMode::Mit { s.max_abs_tau_cmd[i] = s.max_abs_tau_cmd[i].max(t.torque.unwrap_or(0.0).abs()); }
        }
        s.cmd = targets.to_vec();
        Ok(())
    }
    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        let mut s = self.0.lock().unwrap();
        s.torqued = enabled;
        if enabled { s.modes = [JointMode::Position; 7]; let q = s.q; s.cmd = q.iter().map(|&p| JointTarget::position(p)).collect(); }
        Ok(())
    }
    fn check_hardware_status(&self) -> Result<(), ArmDriverError> {
        if self.0.lock().unwrap().hw_fault { Err(ArmDriverError::HardwareFault("Joint 3 reported fault: Overload".into())) } else { Ok(()) }
    }
    fn supports_mit_mode_switch(&self) -> bool { true }
    fn set_joint_modes(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        let mut s = self.0.lock().unwrap();
        if s.feedback_dead_since.is_some() || s.commands_dead {
            return Err(ArmDriverError::HardwareFault("mode write not acknowledged".into()));
        }
        for i in 0..7 {
            if s.modes[i] != modes[i] { s.modes[i] = modes[i]; s.cmd[i] = hold[i].clone(); }
        }
        s.switches.push(modes.to_vec());
        Ok(())
    }
}

struct Rig { sup: ArmSupervisor, sim: Arc<Mutex<Sim>> }

const CLIENT: &str = "teach";

async fn rig_with(model_error: f32, edit: impl FnOnce(&mut ArmConfig)) -> Rig {
    let mut config = ArmConfig::rebot_dm();
    edit(&mut config);
    let sim = Arc::new(Mutex::new(Sim::new(model_error)));
    let sup = ArmSupervisor::new(config).with_driver(Box::new(SimArm(sim.clone())));
    sup.inner().write().await.acquire_lease(CLIENT).unwrap();
    sup.set_torque(true).await.unwrap();
    let rig = Rig { sup, sim };
    rig.ticks(10).await;
    rig
}

async fn rig(model_error: f32) -> Rig { rig_with(model_error, |_| {}).await }

impl Rig {
    async fn ticks(&self, n: usize) -> armd::ArmObservation {
        let mut obs = None;
        for _ in 0..n {
            let epoch = self.sup.get_observation().await.epoch_id;
            let _ = self.sup.heartbeat(CLIENT, epoch).await;
            obs = Some(self.sup.tick(DT).await);
        }
        obs.unwrap_or(self.sup.get_observation().await)
    }
    async fn ticks_no_heartbeat(&self, n: usize) -> armd::ArmObservation {
        let mut obs = None;
        for _ in 0..n { obs = Some(self.sup.tick(DT).await); }
        obs.unwrap()
    }
    async fn enter(&self) -> Result<(), ArmError> {
        self.sup.inner().write().await.enter_gravity_comp(Default::default())
    }
    fn q(&self) -> [f32; 7] { self.sim.lock().unwrap().q }
    fn modes(&self) -> [JointMode; 7] { self.sim.lock().unwrap().modes }
    async fn phase(&self) -> Option<String> { self.sup.get_observation().await.gravity_comp.phase }
}

fn code(err: ArmError) -> String {
    match err { ArmError::Rejected { code, .. } => code, other => panic!("expected a rejection, got {other:?}") }
}

/// Run until the mode is back in position control (or `max` ticks); returns ticks used.
async fn until_position(rig: &Rig, max: usize, heartbeat: bool) -> usize {
    for n in 1..=max {
        let obs = if heartbeat { rig.ticks(1).await } else { rig.ticks_no_heartbeat(1).await };
        if obs.control_mode == ControlMode::Position { return n; }
    }
    panic!("mode did not exit within {max} ticks");
}

fn max_dev(a: &[f32; 7], b: &[f32; 7]) -> f32 { (0..6).map(|i| (a[i] - b[i]).abs()).fold(0.0, f32::max) }

// ---------------------------------------------------------------- entry preconditions

#[tokio::test]
async fn enter_preconditions_reject_with_codes() {
    // unsupported profile
    let sup = ArmSupervisor::new(ArmConfig::default());
    let obs = sup.get_observation().await;
    assert!(!obs.gravity_comp.supported);
    assert_eq!(code(sup.inner().write().await.enter_gravity_comp(Default::default()).unwrap_err()), "UNSUPPORTED");

    // a driver without MIT switching
    struct NoSwitch(SimArm);
    impl ArmDriver for NoSwitch {
        fn capabilities(&self) -> DriverCapabilities { self.0.capabilities() }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> { self.0.read_joint_states() }
        fn write_joint_targets(&mut self, t: &[JointTarget]) -> Result<(), ArmDriverError> { self.0.write_joint_targets(t) }
        fn set_torque(&mut self, e: bool) -> Result<(), ArmDriverError> { self.0.set_torque(e) }
    }
    let sim = Arc::new(Mutex::new(Sim::new(0.0)));
    let sup = ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(NoSwitch(SimArm(sim))));
    assert_eq!(code(sup.inner().write().await.enter_gravity_comp(Default::default()).unwrap_err()), "UNSUPPORTED");

    // not armed
    let sim = Arc::new(Mutex::new(Sim::new(0.0)));
    let sup = ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(SimArm(sim)));
    sup.inner().write().await.acquire_lease(CLIENT).unwrap();
    sup.tick(DT).await;
    let obs = sup.get_observation().await;
    assert!(obs.gravity_comp.supported);
    assert_eq!(obs.gravity_comp.blocker.as_ref().unwrap().code, "NOT_ARMED");
    assert_eq!(code(sup.inner().write().await.enter_gravity_comp(Default::default()).unwrap_err()), "NOT_ARMED");

    // armed and still: allowed
    let r = rig(0.0).await;
    assert!(r.sup.get_observation().await.gravity_comp.blocker.is_none());

    // no lease
    let r = rig(0.0).await;
    { let mut i = r.sup.inner().write().await; let g = i.active_lease.as_ref().unwrap().generation; i.release_lease(CLIENT, g); }
    assert_eq!(code(r.enter().await.unwrap_err()), "LEASE_REQUIRED");

    // moving (servo stream running)
    let r = rig(0.0).await;
    let epoch = r.sup.get_observation().await.epoch_id;
    let mut target = r.sup.get_observation().await.positions;
    target[0] += 0.2;
    r.sup.servo(target, epoch).await.unwrap();
    r.ticks(2).await;
    assert_eq!(code(r.enter().await.unwrap_err()), "MOTION_BUSY");

    // E-stop and fault
    let r = rig(0.0).await;
    r.sup.trigger_e_stop().await;
    assert_eq!(code(r.enter().await.unwrap_err()), "ESTOP_ACTIVE");
    let r = rig(0.0).await;
    r.sim.lock().unwrap().hw_fault = true;
    r.ticks(1).await;
    assert_eq!(code(r.enter().await.unwrap_err()), "FAULT_ACTIVE");

    // stale feedback
    let r = rig(0.0).await;
    r.sim.lock().unwrap().feedback_dead_since = Some(Instant::now() - Duration::from_millis(150));
    assert_eq!(code(r.enter().await.unwrap_err()), "FEEDBACK_STALE");

    // not stationary (measured velocity)
    let r = rig(0.0).await;
    r.sup.inner().write().await.measured_velocities[2] = 0.2;
    assert_eq!(code(r.enter().await.unwrap_err()), "GRAVITY_COMP_NOT_STATIONARY");

    // next to a joint limit
    let r = rig(0.0).await;
    r.sup.inner().write().await.measured_positions[1] = 2.07;
    assert_eq!(code(r.enter().await.unwrap_err()), "GRAVITY_COMP_JOINT_LIMIT");

    // override outside the configured range
    let r = rig(0.0).await;
    let err = r.sup.inner().write().await.enter_gravity_comp(armd::gravity_comp::GravityCompRequest { scale: Some(1.5), damping_kd: None }).unwrap_err();
    assert_eq!(code(err), "INVALID_COMMAND");
    let err = r.sup.inner().write().await.enter_gravity_comp(armd::gravity_comp::GravityCompRequest { scale: None, damping_kd: Some(vec![3.0; 6]) }).unwrap_err();
    assert_eq!(code(err), "INVALID_COMMAND");
    assert_eq!(r.modes(), [JointMode::Position; 7], "a rejected entry switches nothing");
}

// ---------------------------------------------------------------- ramp in / out

#[tokio::test]
async fn ramp_in_fades_stiffness_out_and_model_in_then_exit_holds() {
    let r = rig(0.0).await;
    let entry = r.q();
    r.enter().await.unwrap();
    let mut expected = [JointMode::Mit; 7];
    expected[6] = JointMode::Position;
    assert_eq!(r.modes(), expected, "J1-J6 MIT, gripper stays POS_VEL");
    let obs = r.ticks(1).await;
    assert_eq!(obs.control_mode, ControlMode::GravityComp);
    assert_eq!(obs.gravity_comp.phase.as_deref(), Some("ramp_in"));
    let (kp0, tau0) = {
        let i = r.sup.inner().read().await;
        let run = i.gravity_comp_run().unwrap();
        (run.kp, run.tau_ff)
    };
    assert!(kp0[2] > 25.0, "stiff at the start of the ramp: {kp0:?}");
    let model = armd::compute_gravity_torques(&[0.0, 0.0, 0.0, 0.0, 0.0, std::f32::consts::FRAC_PI_2]).unwrap();
    assert!((tau0[2] - model[2]).abs() < 0.2, "starts from the held torque: {tau0:?} vs {model:?}");

    let obs = r.ticks(25).await; // 0.52 s
    assert!(obs.gravity_comp.ramp > 0.45 && obs.gravity_comp.ramp < 0.6, "ramp {}", obs.gravity_comp.ramp);
    let obs = r.ticks(30).await; // > 1 s
    assert_eq!(obs.gravity_comp.phase.as_deref(), Some("active"));
    assert_eq!(obs.gravity_comp.ramp, 1.0);
    {
        let i = r.sup.inner().read().await;
        let run = i.gravity_comp_run().unwrap();
        assert!(run.kp.iter().all(|k| *k == 0.0), "kp 0 when floating: {:?}", run.kp);
        assert_eq!(run.kd.to_vec(), vec![0.4, 0.6, 0.5, 0.35, 0.15, 0.1]);
    }
    let drift = max_dev(&r.q(), &entry);
    println!("SIM exact model: drift after 1.1 s ramp = {drift:.4} rad");
    assert!(drift < 0.02, "exact model must not sag: {drift}");

    // exit: stiff hold at once, POS_VEL after exit_hold_ms
    let before = r.q();
    r.sup.inner().write().await.exit_gravity_comp("REQUESTED", "test");
    {
        let i = r.sup.inner().read().await;
        let run = i.gravity_comp_run().unwrap();
        assert_eq!(run.phase.name(), "exiting");
        assert_eq!(run.kp[1], 30.0);
        assert!(run.tau_ff[2].abs() > 1.0, "feed-forward kept while exiting");
    }
    let n = until_position(&r, 20, true).await;
    assert_eq!(n, 5, "100 ms MIT hold, then POS_VEL");
    assert_eq!(r.modes(), [JointMode::Position; 7]);
    r.ticks(20).await;
    let obs = r.sup.get_observation().await;
    assert_eq!(obs.gravity_comp.last_exit.as_ref().unwrap().code, "REQUESTED");
    assert!(obs.is_torqued && obs.safety_state == SafetyState::Armed);
    assert!(max_dev(&r.q(), &before) < 0.01, "held at the exit pose");
    assert_eq!(r.sim.lock().unwrap().unsupported_substeps, 0, "never floating without support");
}

#[tokio::test]
async fn feed_forward_is_clamped_per_joint() {
    let r = rig_with(0.0, |c| c.gravity_comp.as_mut().unwrap().torque_limits_nm[2] = 3.0).await;
    r.enter().await.unwrap();
    let mut exit = None;
    for _ in 0..150 {
        let obs = r.ticks(1).await;
        assert!(obs.gravity_comp.tau_ff.get(2).is_none_or(|t| t.abs() <= 3.0 + 1e-6));
        if let Some(e) = obs.gravity_comp.last_exit { exit = Some(e.code); break; }
    }
    let max = r.sim.lock().unwrap().max_abs_tau_cmd;
    println!("SIM clamp J3 3.0 N·m (model 7.6 N·m): max |tau_ff| sent {:.3}, exit {exit:?}", max[2]);
    assert!(max[2] <= 3.0 + 1e-6);
    assert!(matches!(exit.as_deref(), Some("GRAVITY_COMP_SAG" | "GRAVITY_COMP_OVERSPEED")), "an under-supported joint sags and the mode exits: {exit:?}");
}

// ---------------------------------------------------------------- overspeed

#[tokio::test]
async fn model_error_20_percent_sags_and_exits_to_hold() {
    let r = rig(0.20).await;
    let entry = r.q();
    r.enter().await.unwrap();
    let mut first_fast = None;
    let mut exit_tick = None;
    let mut max_sag = 0.0f32;
    for k in 1..=200 {
        let obs = r.ticks(1).await;
        max_sag = max_sag.max(max_dev(&r.q(), &entry));
        let fast = obs.velocities.iter().take(6).any(|v| v.abs() > 1.5);
        if fast && first_fast.is_none() { first_fast = Some(k); }
        if obs.gravity_comp.phase.as_deref() == Some("exiting") { exit_tick = Some(k); break; }
    }
    let exit_tick = exit_tick.expect("exit");
    let at_exit = r.q();
    let obs = r.sup.get_observation().await;
    let exit_code = obs.gravity_comp.last_exit.as_ref().unwrap().code.clone();
    assert!(exit_code == "GRAVITY_COMP_SAG" || exit_code == "GRAVITY_COMP_OVERSPEED", "{exit_code}");
    until_position(&r, 20, true).await;
    r.ticks(25).await;
    let after = r.q();
    let sag_after_exit = max_dev(&after, &at_exit);
    println!("SIM 20% model error: exit {exit_code} at {} ms after entry, first tick over 1.5 rad/s at {:?} ms, sag before exit {max_sag:.3} rad, \
              movement after the hold {sag_after_exit:.3} rad",
        exit_tick * 20, first_fast.map(|k| k * 20));
    assert!(max_sag < 0.25, "caught early: {max_sag}");
    assert!(sag_after_exit < 0.15, "arm caught after the exit: {sag_after_exit}");
    assert!(obs.is_torqued);
    let obs = r.sup.get_observation().await;
    assert_eq!(obs.control_mode, ControlMode::Position);
    assert!(!obs.safety_state.is_stopped(), "overspeed is reported as the exit reason, not a fault");
}

#[tokio::test]
async fn payload_after_ramp_falls_and_exits_on_overspeed() {
    // Floating with an exact model, then a 30 % heavier load (e.g. an object in the
    // gripper): the arm accelerates downwards and the overspeed check ends the mode.
    let r = rig(0.0).await;
    r.enter().await.unwrap();
    r.ticks(60).await;
    assert_eq!(r.phase().await.as_deref(), Some("active"));
    let start = r.q();
    r.sim.lock().unwrap().model_error = 0.30;
    let (mut first_fast, mut exit_tick) = (None, None);
    for k in 1..=200 {
        let obs = r.ticks(1).await;
        if first_fast.is_none() && obs.velocities.iter().take(6).any(|v| v.abs() > 1.5) { first_fast = Some(k); }
        if obs.gravity_comp.phase.as_deref() == Some("exiting") { exit_tick = Some(k); break; }
    }
    let (first_fast, exit_tick) = (first_fast.expect("fast"), exit_tick.expect("exit"));
    let at_exit = r.q();
    let fall = max_dev(&at_exit, &start);
    let obs = r.sup.get_observation().await;
    assert_eq!(obs.gravity_comp.last_exit.as_ref().unwrap().code, "GRAVITY_COMP_OVERSPEED");
    until_position(&r, 20, true).await;
    r.ticks(25).await;
    let after = max_dev(&r.q(), &at_exit);
    println!("SIM +30 % load while floating: over 1.5 rad/s after {} ms, exit hold {} ms later, fall before exit {fall:.3} rad, \
              movement after the hold {after:.3} rad", first_fast * 20, (exit_tick - first_fast) * 20);
    assert!(exit_tick - first_fast <= 2, "3 consecutive ticks including the first");
    assert!(after < 0.15, "{after}");
    let obs = r.sup.get_observation().await;
    assert!(obs.is_torqued && !obs.safety_state.is_stopped(), "overspeed is an exit reason, not a fault");
}

// ---------------------------------------------------------------- safety exits to hold

async fn floating(model_error: f32) -> Rig {
    let r = rig(model_error).await;
    r.enter().await.unwrap();
    r.ticks(60).await;
    assert_eq!(r.phase().await.as_deref(), Some("active"));
    r
}

async fn assert_held(r: &Rig, code: &str, hold: [f32; 7]) {
    let obs = r.sup.get_observation().await;
    assert_eq!(obs.control_mode, ControlMode::Position, "{code}");
    assert_eq!(obs.gravity_comp.last_exit.as_ref().map(|e| e.code.as_str()), Some(code));
    assert!(obs.is_torqued, "{code}: torque must stay on");
    assert_eq!(r.modes()[..6], [JointMode::Position; 6], "{code}");
    let dev = max_dev(&r.q(), &hold);
    assert!(dev < 0.02, "{code}: arm moved {dev} rad after the exit");
}

#[tokio::test]
async fn lease_loss_exits_to_hold() {
    let r = floating(0.0).await;
    let hold = r.q();
    { let mut i = r.sup.inner().write().await; let g = i.active_lease.as_ref().unwrap().generation; i.release_lease(CLIENT, g); i.stop_motion(); }
    r.ticks_no_heartbeat(1).await;
    assert_eq!(r.phase().await.as_deref(), Some("exiting"));
    until_position(&r, 10, false).await;
    assert_held(&r, "LEASE_LOST", hold).await;
}

#[tokio::test]
async fn preemption_exits_to_hold() {
    let r = floating(0.0).await;
    let hold = r.q();
    r.sup.inner().write().await.preempt_lease("other", "cockpit").unwrap();
    until_position(&r, 10, false).await;
    assert_held(&r, "LEASE_LOST", hold).await;
}

#[tokio::test]
async fn estop_exits_to_hold_immediately() {
    let r = floating(0.0).await;
    let hold = r.q();
    r.sup.trigger_e_stop().await;
    {
        let i = r.sup.inner().read().await;
        assert_eq!(i.gravity_comp_run().unwrap().phase.name(), "exiting", "E-stop catches the arm before the next tick");
    }
    assert_eq!(r.sim.lock().unwrap().cmd[1].kp, Some(30.0), "stiff hold already sent");
    until_position(&r, 10, false).await;
    assert_held(&r, "ESTOP", hold).await;
    assert_eq!(r.sup.get_observation().await.safety_state, SafetyState::EStop);
}

#[tokio::test]
async fn hardware_fault_exits_to_hold() {
    let r = floating(0.0).await;
    let hold = r.q();
    r.sim.lock().unwrap().hw_fault = true;
    until_position(&r, 10, true).await;
    assert_held(&r, "FAULT", hold).await;
    assert!(matches!(r.sup.get_observation().await.safety_state, SafetyState::Fault(_)));
}

#[tokio::test]
async fn stale_feedback_exits_to_a_stiff_hold() {
    let r = floating(0.0).await;
    let hold = r.q();
    r.sim.lock().unwrap().feedback_dead_since = Some(Instant::now() - Duration::from_millis(120));
    r.ticks(1).await;
    assert_eq!(r.sup.get_observation().await.gravity_comp.last_exit.unwrap().code, "FEEDBACK_STALE");
    // mode switch cannot be acknowledged without feedback: the MIT stiff hold stays on
    r.ticks(10).await;
    assert_eq!(r.phase().await.as_deref(), Some("exit_failed"));
    assert!(matches!(r.sup.get_observation().await.safety_state, SafetyState::Fault(_)));
    assert!(max_dev(&r.q(), &hold) < 0.01, "held by the MIT stiff hold");
    let clear = r.sup.clear_fault().await;
    assert!(clear.is_err(), "fault stays latched until POS_VEL is confirmed");
    r.sim.lock().unwrap().feedback_dead_since = None;
    until_position(&r, 40, true).await;
    assert_held(&r, "FEEDBACK_STALE", hold).await;
}

#[tokio::test]
async fn bus_loss_exits_and_recovers_to_hold() {
    let r = floating(0.0).await;
    let hold = r.q();
    {
        let mut s = r.sim.lock().unwrap();
        s.feedback_dead_since = Some(Instant::now() - Duration::from_millis(600));
        s.commands_dead = true;
    }
    r.ticks(1).await;
    let obs = r.sup.get_observation().await;
    assert_eq!(obs.gravity_comp.last_exit.as_ref().unwrap().code, "BUS_LOSS");
    assert_eq!(obs.safety_state, SafetyState::Fault(armd::FaultReason::BusLoss));
    r.ticks(10).await;
    // With the bus down nothing reaches the motors: they keep the last MIT command
    // (model feed-forward and damping). The exact model keeps the arm where it was.
    let drift = max_dev(&r.q(), &hold);
    println!("SIM full bus loss, exact model: drift during 200 ms without commands {drift:.4} rad");
    {
        let mut s = r.sim.lock().unwrap();
        s.feedback_dead_since = None;
        s.commands_dead = false;
    }
    until_position(&r, 40, true).await;
    assert_held(&r, "BUS_LOSS", hold).await;
}

#[tokio::test]
async fn watchdog_exits_to_hold_before_the_park_starts() {
    let r = rig_with(0.0, |c| c.lease_timeout_ms = 150).await;
    r.enter().await.unwrap();
    r.ticks(60).await;
    let hold = r.q();
    tokio::time::sleep(Duration::from_millis(200)).await;
    r.ticks_no_heartbeat(1).await;
    {
        let i = r.sup.inner().read().await;
        assert_eq!(i.gravity_comp_run().unwrap().phase.name(), "exiting");
        assert!(!i.pending_normal_unload, "no park while still in MIT");
    }
    assert_eq!(r.sup.get_observation().await.gravity_comp.last_exit.unwrap().code, "WATCHDOG");
    let n = until_position(&r, 10, false).await;
    assert!(max_dev(&r.q(), &hold) < 0.02);
    // back in position mode: the existing watchdog logic takes over (park, or hold + fault)
    r.ticks_no_heartbeat(2).await;
    let i = r.sup.inner().read().await;
    println!("SIM watchdog: POS_VEL hold after {} ms, then park started = {}", n * 20, i.pending_normal_unload);
    assert!(i.pending_normal_unload || matches!(i.safety.state(), SafetyState::Fault(armd::FaultReason::WatchdogTimeout)));
    assert!(i.is_torqued);
}

// ---------------------------------------------------------------- hand motion, recording

#[tokio::test]
async fn hand_motion_does_not_trip_tracking_and_is_recorded() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("teach.parquet");
    let r = rig(0.0).await;
    r.sup.start_recording(Some("teach".into()), Some(path.clone())).await.unwrap();
    r.enter().await.unwrap();
    r.ticks(55).await;
    let start = r.q();
    // the "hand": 0.3 N·m on J1 and -0.3 N·m on J2 for 2 s
    { let mut s = r.sim.lock().unwrap(); s.user_torque[0] = 0.3; s.user_torque[1] = -0.3; }
    let mut max_err = 0.0f32;
    let mut floating_ts = Vec::new();
    for _ in 0..100 {
        let obs = r.ticks(1).await;
        assert!(!obs.safety_state.is_stopped(), "{:?}", obs.safety_state);
        assert_eq!(obs.control_mode, ControlMode::GravityComp);
        floating_ts.push(obs.timestamp);
        for i in 0..6 { max_err = max_err.max((obs.commanded_positions[i] - obs.positions[i]).abs()); }
    }
    { let mut s = r.sim.lock().unwrap(); s.user_torque = [0.0; 6]; }
    r.ticks(30).await;
    let moved = r.q();
    println!("SIM hand guiding: J1 moved {:.3} rad, J2 {:.3} rad, max |cmd - meas| {max_err:.4}", moved[0] - start[0], moved[1] - start[1]);
    assert!((moved[0] - start[0]).abs() > 0.6, "J1 moved beyond the 0.5 rad tracking bound");
    assert_eq!(r.phase().await.as_deref(), Some("active"));
    // gripper is commanded while floating
    let epoch = r.sup.get_observation().await.epoch_id;
    let req = armd::motion::parse_gripper(&serde_json::json!({"command":"open"})).unwrap();
    r.sup.inner().write().await.start_gripper(req, epoch).unwrap();
    r.ticks(60).await;
    assert!(r.q()[6] < -1.0, "gripper opened in POS_VEL: {}", r.q()[6]);
    assert_eq!(r.phase().await.as_deref(), Some("active"));
    r.sup.inner().write().await.exit_gravity_comp("REQUESTED", "done");
    until_position(&r, 10, true).await;
    let saved = r.sup.save_recording().await.unwrap();
    let frames = armd::read_frames_from_parquet(&saved).unwrap();
    let teach: Vec<_> = frames.iter().filter(|f| floating_ts.contains(&f.timestamp)).collect();
    assert_eq!(teach.len(), floating_ts.len(), "one frame per tick while floating");
    assert!(teach.iter().all(|f| (0..6).all(|i| (f.action[i] - f.observation_state[i]).abs() < 1e-6)),
        "while floating, action = measured state for J1-J6");
    let j1_span = frames.iter().map(|f| f.observation_state[0]).fold(f32::MIN, f32::max) - frames.iter().map(|f| f.observation_state[0]).fold(f32::MAX, f32::min);
    println!("SIM recording: {} frames, J1 span {j1_span:.3} rad", frames.len());
    assert!(j1_span > 0.6);
}

// ---------------------------------------------------------------- command gating (in-process)

#[tokio::test]
async fn motion_commands_are_busy_while_floating() {
    let r = floating(0.0).await;
    let obs = r.sup.get_observation().await;
    let epoch = obs.epoch_id;
    let mut i = r.sup.inner().write().await;
    assert_eq!(i.servo(obs.positions.clone(), epoch).unwrap_err(), armd::SafetyError::MotionBusy);
    let jog = i.jog_action(obs.positions.clone(), None, Some(epoch), "joint_move").unwrap_err();
    assert!(matches!(jog, ArmError::Motion(armd::motion::MotionError::MotionBusy(_))), "{jog:?}");
    assert!(matches!(i.plan_input().unwrap_err(), ArmError::Motion(armd::motion::MotionError::MotionBusy(_))));
    assert_eq!(i.set_torque(false).unwrap_err(), armd::SafetyError::MotionBusy);
    assert_eq!(i.gravity_comp_run().unwrap().phase.name(), "active", "rejections do not end the mode");
}

// ---------------------------------------------------------------- per-joint overspeed limits

#[tokio::test]
async fn fast_wrist_hand_motion_does_not_exit_but_fast_elbow_does() {
    // Real arm 2026-11-09: a hand flick of the wrist reached 2.5 rad/s on J4 and ended the
    // mode with the single 1.5 rad/s limit. rebot-dm now allows 3.0 rad/s on J4..J6.
    let r = rig(0.0).await;
    r.enter().await.unwrap();
    r.ticks(60).await;
    { let mut s = r.sim.lock().unwrap(); s.user_torque[3] = 0.8; }
    let mut peak = 0.0f32;
    for _ in 0..40 {
        let obs = r.ticks(1).await;
        peak = peak.max(obs.velocities[3].abs());
        assert_eq!(obs.control_mode, ControlMode::GravityComp, "wrist hand motion must not end the mode");
    }
    { let mut s = r.sim.lock().unwrap(); s.user_torque = [0.0; 6]; }
    println!("SIM wrist push: J4 peak {peak:.2} rad/s");
    assert!(peak > 1.5 && peak < 3.0, "test must exercise the band between the old and new limit: {peak}");
    r.ticks(30).await;

    // the elbow keeps the 1.5 rad/s fall detection
    { let mut s = r.sim.lock().unwrap(); s.user_torque[2] = -6.0; }
    let mut exited = false;
    for _ in 0..100 {
        let obs = r.ticks(1).await;
        if obs.gravity_comp.phase.as_deref() == Some("exiting") || obs.control_mode != ControlMode::GravityComp { exited = true; break; }
    }
    { let mut s = r.sim.lock().unwrap(); s.user_torque = [0.0; 6]; }
    assert!(exited, "fast J3 must still end the mode");
    let obs = r.sup.get_observation().await;
    let exit = obs.gravity_comp.last_exit.as_ref().unwrap();
    assert_eq!(exit.code, "GRAVITY_COMP_OVERSPEED");
    assert!(exit.message.contains("Joint 3"), "{}", exit.message);
}

#[test]
fn per_joint_speed_limits_validate() {
    let mut config = ArmConfig::rebot_dm();
    let gc = config.gravity_comp.as_ref().unwrap();
    assert_eq!((0..6).map(|i| gc.speed_limit(i)).collect::<Vec<_>>(), vec![1.5, 1.5, 1.5, 3.0, 3.0, 3.0]);
    assert!(config.validate().is_ok());
    config.gravity_comp.as_mut().unwrap().joint_max_speed_rad_s = vec![1.5; 5];
    assert!(config.validate().is_err(), "wrong length");
    config.gravity_comp.as_mut().unwrap().joint_max_speed_rad_s = vec![1.5, 1.5, 1.5, 0.0, 3.0, 3.0];
    assert!(config.validate().is_err(), "zero limit");
    config.gravity_comp.as_mut().unwrap().joint_max_speed_rad_s = vec![];
    assert!(config.validate().is_ok(), "empty falls back to max_speed_rad_s");
    assert_eq!(config.gravity_comp.as_ref().unwrap().speed_limit(4), 1.5);
}
