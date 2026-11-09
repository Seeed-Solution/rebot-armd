//! Park settle with realistic Damiao feedback quantization.
//!
//! The rebot-dm arm runs through the real DamiaoArmDriver over a mock CAN bus. A small
//! plant follows the commanded positions with a first-order lag; feedback frames carry the
//! plant position (16-bit), its velocity plus Gaussian noise (12-bit, J1-J3 at VMAX 10:
//! 1 LSB = 20 / 4095 = 0.00488 rad/s) and the torque plus noise. Zero velocity has no
//! exact code: raw 2047 / 2048 decode to -/+ 0.5 LSB, raw 2046 / 2049 to -/+ 1.5 LSB.
use std::sync::{Arc, Mutex};
use std::time::Duration;

use armd::driver::{ArmDriver, ArmDriverError, DamiaoArmDriver, DriverCapabilities, EnableState, JointTarget};
use armd::model::{ArmConfig, DamiaoRange};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::safety::FaultReason;
use armd::transport::{CanFrame, MockCanTransport};
use armd::{ArmSupervisor, SafetyState};

const DT: f64 = 0.02;
const INSPECT: [f32; 7] = [0.0, -1.167, -1.565, 1.435, 0.0, std::f32::consts::FRAC_PI_2, 0.0];

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    fn gauss(&mut self) -> f64 {
        let (u, v) = (self.next().max(1e-12), self.next());
        (-2.0 * u.ln()).sqrt() * (2.0 * std::f64::consts::PI * v).cos()
    }
}

fn limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm().damiao_ranges.unwrap().iter().map(DamiaoRange::limits).collect()
}

fn frame(id: u8, state: u8, p_raw: u16, v_raw: u16, t_raw: u16) -> CanFrame {
    CanFrame::new_standard(0x10 + id as u16, &[
        (state << 4) | id, (p_raw >> 8) as u8, p_raw as u8, (v_raw >> 4) as u8,
        (((v_raw & 0x0F) << 4) | (t_raw >> 8)) as u8, t_raw as u8, 30, 30,
    ]).unwrap()
}

struct Shared(Arc<Mutex<DamiaoArmDriver<MockCanTransport>>>);
impl ArmDriver for Shared {
    fn capabilities(&self) -> DriverCapabilities { self.0.lock().unwrap().capabilities() }
    fn enable_state(&self) -> EnableState { self.0.lock().unwrap().enable_state() }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { self.0.lock().unwrap().feedback_ages() }
    fn max_command_velocity(&self, j: usize) -> Option<f32> { self.0.lock().unwrap().max_command_velocity(j) }
    fn read_joint_states(&mut self) -> Result<Vec<armd::model::JointState>, ArmDriverError> { self.0.lock().unwrap().read_joint_states() }
    fn write_joint_targets(&mut self, t: &[JointTarget]) -> Result<(), ArmDriverError> { self.0.lock().unwrap().write_joint_targets(t) }
    fn set_torque(&mut self, e: bool) -> Result<(), ArmDriverError> { self.0.lock().unwrap().set_torque(e) }
    fn is_feedback_valid(&self) -> bool { self.0.lock().unwrap().is_feedback_valid() }
    fn check_hardware_status(&self) -> Result<(), ArmDriverError> { self.0.lock().unwrap().check_hardware_status() }
    fn is_enabled(&self) -> bool { self.0.lock().unwrap().is_enabled() }
    fn motor_status(&self) -> Option<Vec<u8>> { self.0.lock().unwrap().motor_status() }
    fn set_torque_flag(&mut self, e: bool) { self.0.lock().unwrap().set_torque_flag(e) }
    fn set_pending_rearm(&mut self, p: bool) { self.0.lock().unwrap().set_pending_rearm(p) }
}

/// Feedback noise: Gaussian sigma in LSB, clipped to +/- `clip` LSB.
#[derive(Clone, Copy)]
struct Noise { vel_sigma_lsb: f64, vel_clip_lsb: f64, torque_sigma_lsb: f64 }

struct Rig {
    sup: ArmSupervisor,
    drv: Arc<Mutex<DamiaoArmDriver<MockCanTransport>>>,
    lim: Vec<DamiaoLimits>,
    q: Vec<f64>,
    /// Plant lag time constant (s); Damiao POS_VEL trails by about v / 2.2 rad.
    tau: f64,
    /// Joint `.0` cannot move above `.1` (rad): a blocked or stuck arm.
    blocked: Option<(usize, f64)>,
    /// Static torque (N·m) added on each joint.
    load: Vec<f64>,
    /// Velocity (rad/s) added to each joint's motion: creep the controller does not remove.
    drift: Vec<f64>,
    noise: Noise,
    rng: Rng,
}

impl Rig {
    async fn new(start: &[f32], noise: Noise, seed: u64) -> Self {
        let mut config = ArmConfig::rebot_dm();
        config.lease_timeout_ms = 50;
        let lim = limits();
        let mut driver = DamiaoArmDriver::new(MockCanTransport::new(), (1..=7).collect())
            .with_joint_limits(lim.clone()).unwrap();
        for id in 1..=7u8 {
            let l = &lim[id as usize - 1];
            let p = damiao::float_to_uint(start[id as usize - 1], l.p_min, l.p_max, 16);
            let f = frame(id, 1, p, 2048, 2048);
            driver.can_mut().respond_to_transmit(damiao::build_enable_frame(id as u16), f);
            driver.can_mut().respond_to_transmit(damiao::build_disable_frame(id as u16), frame(id, 0, p, 2048, 2048));
            driver.can_mut().provide_rx_frame(f);
        }
        let drv = Arc::new(Mutex::new(driver));
        let sup = ArmSupervisor::new(config);
        sup.set_lease_enabled(false).await;
        sup.set_driver(Box::new(Shared(drv.clone()))).await;
        {
            // Start already at `start` (positions are read back on enable).
            let mut inner = sup.inner().write().await;
            inner.measured_positions = start.to_vec();
        }
        sup.set_torque(true).await.unwrap();
        Self { sup, drv, lim, q: start.iter().map(|&p| p as f64).collect(), tau: 0.15, blocked: None,
            load: vec![0.0; 7], drift: vec![0.0; 7], noise, rng: Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1) }
    }

    /// Advance the plant one tick toward the commanded positions, feed one frame per
    /// motor and tick the supervisor.
    async fn tick(&mut self) -> armd::ArmObservation {
        let cmd = self.sup.inner().read().await.commanded_positions.clone();
        let enabled = self.drv.lock().unwrap().is_enabled();
        let mut frames = Vec::new();
        for i in 0..7 {
            let prev = self.q[i];
            if enabled {
                self.q[i] += (cmd[i] as f64 - self.q[i]) * (DT / self.tau).min(1.0) + self.drift[i] * DT;
            }
            if let Some((j, stop)) = self.blocked {
                if j == i { self.q[i] = self.q[i].min(stop); }
            }
            let v = (self.q[i] - prev) / DT;
            let l = &self.lim[i];
            let v_lsb = (l.v_max - l.v_min) as f64 / 4095.0;
            let t_lsb = (l.t_max - l.t_min) as f64 / 4095.0;
            let n = (self.noise.vel_sigma_lsb * self.rng.gauss()).clamp(-self.noise.vel_clip_lsb, self.noise.vel_clip_lsb);
            let v_noise = if i < 3 { n } else { 0.0 };
            let v_raw = (2047.5 + v / v_lsb + v_noise).round().clamp(0.0, 4095.0) as u16;
            let t_noise = self.noise.torque_sigma_lsb * self.rng.gauss();
            let t_raw = (2047.5 + self.load[i] / t_lsb + t_noise).round().clamp(0.0, 4095.0) as u16;
            let p_raw = damiao::float_to_uint(self.q[i] as f32, l.p_min, l.p_max, 16);
            frames.push(frame(i as u8 + 1, u8::from(enabled), p_raw, v_raw, t_raw));
        }
        {
            let mut d = self.drv.lock().unwrap();
            d.can_mut().take_tx_frames();
            d.can_mut().provide_rx_frames(&frames);
        }
        self.sup.tick(DT).await
    }
}

/// Unload from rest at home: seconds from the end of the park trajectory until torque
/// is cut, or `None` within `max_s`.
async fn settle_time(noise: Noise, seed: u64, max_s: f64) -> Option<f64> {
    let home = ArmConfig::rebot_dm().home_positions;
    let mut rig = Rig::new(&home, noise, seed).await;
    rig.sup.execute_stop(armd::safety::StopPolicy::NormalUnload).await;
    assert!(rig.sup.inner().read().await.pending_normal_unload);
    let mut traj_end = None;
    let mut t = 0.0;
    while t < 30.0 {
        rig.tick().await;
        t += DT;
        let inner = rig.sup.inner().read().await;
        if traj_end.is_none() && inner.active_trajectory.is_none() { traj_end = Some(t); }
        if let Some(end) = traj_end {
            if !inner.pending_normal_unload {
                assert!(!inner.safety.state().is_stopped(), "{:?}", inner.safety.state());
                return Some(t - end);
            }
            if t - end > max_s { return None; }
        }
    }
    None
}

const QUIET: Noise = Noise { vel_sigma_lsb: 0.0, vel_clip_lsb: 0.0, torque_sigma_lsb: 0.0 };

fn noise(sigma: f64) -> Noise { Noise { vel_sigma_lsb: sigma, vel_clip_lsb: 3.0, torque_sigma_lsb: 2.0 } }

/// Settle time vs. velocity noise. `cargo test --test test_park_settle_noise -- --ignored --nocapture`
#[tokio::test]
#[ignore]
async fn report_settle_time_vs_noise() {
    println!("sigma_lsb  settled/seeds  median_s  max_s");
    for sigma in [0.0, 0.3, 0.5, 0.75, 1.0, 1.5, 2.0] {
        let mut times = Vec::new();
        let seeds = 20;
        for seed in 0..seeds {
            if let Some(t) = settle_time(noise(sigma), seed, 20.0).await { times.push(t); }
        }
        times.sort_by(f64::total_cmp);
        let median = times.get(times.len() / 2).copied().unwrap_or(f64::NAN);
        let max = times.last().copied().unwrap_or(f64::NAN);
        println!("{sigma:9.2}  {:7}/{seeds:<5}  {median:8.2}  {max:5.2}", times.len());
    }
}

#[tokio::test]
async fn quiet_feedback_settles_in_the_hold_time() {
    let t = settle_time(QUIET, 1, 5.0).await.expect("quiet arm settles");
    assert!((0.7..1.2).contains(&t), "settle {t:.2} s");
}

#[tokio::test]
async fn settle_time_is_bounded_under_quantization_noise() {
    for sigma in [0.5, 1.0, 1.5] {
        for seed in 0..8 {
            let t = settle_time(noise(sigma), seed, 10.0).await
                .unwrap_or_else(|| panic!("sigma {sigma} LSB seed {seed}: no settle within 10 s"));
            assert!(t < 1.5, "sigma {sigma} LSB seed {seed}: settle {t:.2} s");
        }
    }
}

/// Link loss during a jog toward inspect: watchdog park with realistic noise ends at the
/// resting pose with torque off, no fault.
#[tokio::test]
async fn link_loss_during_jog_parks_to_idle_with_noise() {
    let home = ArmConfig::rebot_dm().home_positions;
    let mut rig = Rig::new(&home, noise(1.0), 7).await;
    rig.sup.jog(INSPECT.to_vec(), None, None).await.unwrap();
    for _ in 0..100 { rig.tick().await; } // 2 s into the jog
    {
        let inner = rig.sup.inner().read().await;
        assert!(inner.active_trajectory.is_some());
        assert!(inner.commanded_velocities[2].abs() > 0.2, "mid-jog: {:?}", inner.commanded_velocities);
    }
    rig.sup.set_lease_enabled(true).await;
    tokio::time::sleep(Duration::from_millis(80)).await;
    let obs = rig.tick().await;
    assert!(!obs.safety_state.is_stopped(), "{:?}", obs.safety_state);
    assert!(rig.sup.inner().read().await.watchdog_park.is_some());
    let mut t = 0.0;
    loop {
        let obs = rig.tick().await;
        t += DT;
        assert!(!obs.safety_state.is_stopped(), "park faulted after {t:.2} s: {:?}", obs.safety_state);
        if obs.safety_state == SafetyState::Idle && !obs.is_torqued { break; }
        assert!(t < 30.0, "no park within 30 s");
    }
    let obs = rig.sup.get_observation().await;
    for i in 0..6 { assert!((obs.positions[i] - home[i]).abs() < 0.01, "J{} at {}", i + 1, obs.positions[i]); }
    assert!(rig.sup.inner().read().await.watchdog_park.is_none());
}

/// A joint that cannot reach home: the watchdog park latches WatchdogTimeout with torque on.
#[tokio::test]
async fn blocked_arm_still_faults_with_torque_held() {
    let mut start = ArmConfig::rebot_dm().home_positions;
    start[2] = -0.6;
    let mut rig = Rig::new(&start, noise(1.0), 3).await;
    rig.blocked = Some((2, -0.3));
    rig.sup.set_lease_enabled(true).await;
    tokio::time::sleep(Duration::from_millis(80)).await;
    rig.tick().await;
    assert!(rig.sup.inner().read().await.pending_normal_unload);
    let mut t = 0.0;
    let state = loop {
        let obs = rig.tick().await;
        t += DT;
        if obs.safety_state.is_stopped() { break obs.safety_state; }
        assert!(obs.is_torqued, "torque must never be cut off home: t {t:.2} {:?} {:?} enable {:?}", obs.safety_state, obs.positions, obs.enable_state);
        assert!(t < 60.0, "blocked arm never faulted");
    };
    assert!(matches!(state, SafetyState::Fault(FaultReason::WatchdogTimeout)), "{state:?}");
    let obs = rig.sup.get_observation().await;
    assert!(obs.is_torqued);
    assert!((obs.positions[2] + 0.3).abs() < 0.01);
}

/// At home but moving slowly (creep above the stationary bound): never released.
#[tokio::test]
async fn creeping_arm_is_not_released() {
    let home = ArmConfig::rebot_dm().home_positions;
    let mut rig = Rig::new(&home, noise(0.5), 5).await;
    rig.sup.execute_stop(armd::safety::StopPolicy::NormalUnload).await;
    for _ in 0..100 { rig.tick().await; } // trajectory done
    rig.tau = f64::INFINITY; // plant ignores commands from here
    rig.drift[1] = 0.01; // J2 creeps at 0.01 rad/s, inside the position tolerance throughout
    for _ in 0..250 {
        let obs = rig.tick().await;
        assert!(obs.is_torqued, "torque cut while J2 creeps at {}", obs.positions[1]);
    }
    assert!(rig.sup.inner().read().await.pending_normal_unload);
}

/// Unsupported at home (J3 support torque above its limit): never released.
#[tokio::test]
async fn unsupported_arm_is_not_released() {
    let home = ArmConfig::rebot_dm().home_positions;
    let mut rig = Rig::new(&home, noise(1.0), 9).await;
    rig.load[2] = 8.0;
    rig.sup.execute_stop(armd::safety::StopPolicy::NormalUnload).await;
    for _ in 0..300 {
        let obs = rig.tick().await;
        assert!(obs.is_torqued, "torque cut while J3 carries 8 N·m");
    }
    assert!(rig.sup.inner().read().await.pending_normal_unload);
}

/// Velocity feedback reads zero while the position creeps: the position span over the
/// hold window blocks the release.
#[tokio::test]
async fn silent_creep_is_blocked_by_position_span() {
    let home = ArmConfig::rebot_dm().home_positions;
    let mut rig = Rig::new(&home, QUIET, 11).await;
    rig.tau = f64::INFINITY;
    rig.sup.execute_stop(armd::safety::StopPolicy::NormalUnload).await;
    for _ in 0..250 {
        rig.q[1] += 0.01 * DT; // moved outside the plant step: reported velocity stays 0
        let obs = rig.tick().await;
        assert!(obs.is_torqued, "torque cut while J2 creeps at {}", obs.positions[1]);
    }
    assert!(rig.sup.inner().read().await.pending_normal_unload);
}

/// At the resting pose but never stationary (J1 velocity spikes above 4x the limit):
/// the watchdog park gets the settle grace once, then latches WatchdogTimeout with
/// torque held.
#[tokio::test]
async fn watchdog_park_at_pose_but_never_still_faults_after_grace() {
    use armd::supervisor::{WATCHDOG_PARK_SETTLE_GRACE, WATCHDOG_PARK_TIMEOUT};
    // `--nocapture` shows the rate-limited settle blocker reports.
    let _ = tracing_subscriber::fmt().with_test_writer().with_max_level(tracing::Level::WARN).try_init();
    let home = ArmConfig::rebot_dm().home_positions;
    let shaky = Noise { vel_sigma_lsb: 8.0, vel_clip_lsb: 8.0, torque_sigma_lsb: 2.0 };
    let mut rig = Rig::new(&home, shaky, 13).await;
    rig.sup.set_lease_enabled(true).await;
    tokio::time::sleep(Duration::from_millis(80)).await;
    rig.tick().await;
    let budget = rig.sup.inner().read().await.watchdog_park.as_ref().unwrap().budget_s;
    assert_eq!(budget, WATCHDOG_PARK_TIMEOUT.as_secs_f64(), "short park keeps the floor");
    let mut t = DT;
    let mut grace_seen = false;
    let state = loop {
        let obs = rig.tick().await;
        t += DT;
        grace_seen |= rig.sup.inner().read().await.watchdog_park.as_ref().is_some_and(|p| p.grace_granted);
        if obs.safety_state.is_stopped() { break obs.safety_state; }
        assert!(obs.is_torqued, "torque cut while not stationary");
        assert!(t < 30.0, "never faulted");
    };
    assert!(grace_seen);
    assert!(matches!(state, SafetyState::Fault(FaultReason::WatchdogTimeout)), "{state:?}");
    let expected = budget + WATCHDOG_PARK_SETTLE_GRACE.as_secs_f64();
    assert!((t - expected).abs() < 0.1, "faulted at {t:.2} s, expected {expected:.2} s");
    assert!(rig.sup.get_observation().await.is_torqued);
}

/// Worked numbers for the incident: park budget after 3 s of a jog toward inspect.
#[tokio::test]
async fn watchdog_budget_mid_jog_covers_deceleration_travel_and_settle() {
    use armd::supervisor::{UNLOAD_SETTLE_HOLD, WATCHDOG_PARK_MARGIN};
    let home = ArmConfig::rebot_dm().home_positions;
    let mut rig = Rig::new(&home, noise(1.0), 17).await;
    rig.sup.jog(INSPECT.to_vec(), None, None).await.unwrap();
    for _ in 0..150 { rig.tick().await; }
    rig.sup.set_lease_enabled(true).await;
    tokio::time::sleep(Duration::from_millis(80)).await;
    rig.tick().await;
    let inner = rig.sup.inner().read().await;
    let planned = inner.unload_planned_duration.unwrap();
    let decel = inner.measured_velocities.iter().zip(inner.planning_limits())
        .map(|(v, l)| f64::from(v.abs() / l.max_acceleration)).fold(0.0, f64::max);
    let budget = inner.watchdog_park.as_ref().unwrap().budget_s;
    let expected = decel + planned + (UNLOAD_SETTLE_HOLD + WATCHDOG_PARK_MARGIN).as_secs_f64();
    println!("planned {planned:.3} s, decel {decel:.3} s, budget {budget:.3} s");
    assert!((budget - expected.max(10.0)).abs() < 1e-3, "budget {budget} expected {expected}");
}
