//! Planned joint speed cap (spec 1c): jog / home / presets / park are planned with
//! `planning_limits()`, so Damiao POS_VEL lag stays below the following-error bound.
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
use armd::model::{ArmConfig, JointState, ObservationSchema};
use armd::safety::FaultReason;
use armd::supervisor::{WATCHDOG_PARK_MARGIN, WATCHDOG_PARK_TIMEOUT};
use armd::{ArmSupervisor, SafetyState};

const INSPECT: [f32; 7] = [0.0, -1.167, -1.565, 1.435, 0.0, std::f32::consts::FRAC_PI_2, 0.0];
const DT: f64 = 0.02;
/// Wall-clock time per simulated 20 ms tick. The tracking-fault persistence is
/// wall-clock, so it is scaled by the same factor (500 ms -> 125 ms).
const WALL_PER_TICK: Duration = Duration::from_millis(5);
const SCALED_PERSISTENCE_MS: u64 = 125;

#[test]
fn rebot_dm_inspect_jog_is_capped_at_planned_speed() {
    let supervisor = ArmSupervisor::new(ArmConfig::rebot_dm());
    let rt = tokio::runtime::Builder::new_current_thread().enable_all().build().unwrap();
    rt.block_on(async {
        supervisor.set_lease_enabled(false).await;
        supervisor.set_torque(true).await.unwrap();
        supervisor.jog(INSPECT.to_vec(), None, None).await.unwrap();
        let duration = supervisor.inner().read().await.active_trajectory.as_ref().unwrap().duration();
        assert!((duration - 1.875 * 1.565 / 0.45).abs() < 0.01, "duration {duration}");
        let mut peak = [0.0f32; 7];
        for _ in 0..400 {
            let obs = supervisor.tick(DT).await;
            let inner = supervisor.inner().read().await;
            for (p, v) in peak.iter_mut().zip(&inner.commanded_velocities) { *p = p.max(v.abs()); }
            assert_eq!(obs.commanded_positions, inner.commanded_positions);
        }
        for (i, p) in peak.iter().take(6).enumerate() {
            assert!(*p <= 0.45 + 1e-3, "joint {} commanded peak {p}", i + 1);
        }
        assert!(peak[2] > 0.44, "J3 sets the duration: {}", peak[2]);
        let obs = supervisor.get_observation().await;
        assert!(obs.positions.iter().zip(INSPECT).all(|(a, b)| (a - b).abs() < 1e-3));
    });
}

#[test]
fn planning_limits_skip_the_gripper_and_keep_lower_limits() {
    let inner = armd::SupervisorInner::new(ArmConfig::rebot_dm());
    let limits = inner.planning_limits();
    for l in &limits[..6] { assert_eq!(l.max_velocity, 0.45); }
    assert_eq!(limits[6].max_velocity, 2.0);
    let mut config = ArmConfig::default();
    config.planned_max_velocity = Some(0.6);
    config.limits[0].max_velocity = 0.5;
    let inner = armd::SupervisorInner::new(config);
    assert_eq!(inner.planning_limits()[0].max_velocity, 0.5, "cap never raises a limit");
    assert_eq!(inner.planning_limits()[5].max_velocity, 0.6, "6-DOF arm has no gripper exemption");
}

#[test]
fn validate_rejects_bad_speed_caps() {
    for bad in [0.0, -1.0, f32::NAN, f32::INFINITY, 1.1, 1.5] {
        let mut config = ArmConfig::rebot_dm();
        config.planned_max_velocity = Some(bad);
        assert!(config.validate().is_err(), "cap {bad} must be rejected");
    }
    let mut config = ArmConfig::rebot_dm();
    assert!(config.validate().is_ok());
    assert_eq!(config.planned_max_velocity, Some(0.45));
    assert_eq!(ArmConfig::default().planned_max_velocity, None);
}

#[test]
fn schema_and_observation_carry_commanded_positions() {
    let schema = ObservationSchema::from_config(&ArmConfig::rebot_dm());
    let field = schema.fields.iter().find(|f| f.name == "commanded_positions").expect("schema field");
    assert_eq!(field.shape, vec![7]);
    let json = serde_json::to_value(armd::SupervisorInner::new(ArmConfig::rebot_dm()).current_observation()).unwrap();
    assert_eq!(json["commanded_positions"].as_array().unwrap().len(), 7);
    // Older payloads without the field still parse.
    let mut legacy = json.clone();
    legacy.as_object_mut().unwrap().remove("commanded_positions");
    let parsed: armd::ArmObservation = serde_json::from_value(legacy).unwrap();
    assert!(parsed.commanded_positions.is_empty());
}

// ---- first-order POS_VEL lag model: v = clamp((cmd - meas) * 1.5, +-2), gain measured on the real arm, meas += v * dt ----

struct LagArm { q: [f32; 7], v: [f32; 7], enabled: EnableState }
struct LagDriver(Arc<Mutex<LagArm>>);
impl ArmDriver for LagDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true,
            torque_feedforward: true, enable_feedback: true }
    }
    fn enable_state(&self) -> EnableState { self.0.lock().unwrap().enabled }
    fn is_feedback_valid(&self) -> bool { true }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); 7] }
    fn max_command_velocity(&self, _: usize) -> Option<f32> { Some(2.0) }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let arm = self.0.lock().unwrap();
        Ok((0..7).map(|i| JointState { position: arm.q[i], velocity: arm.v[i], torque: 0.0, temperature: 30.0 }).collect())
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        let mut arm = self.0.lock().unwrap();
        for (i, t) in targets.iter().enumerate().take(7) {
            let v = ((t.position - arm.q[i]) * 1.5).clamp(-2.0, 2.0);
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

async fn lag_arm(cap: Option<f32>) -> (ArmSupervisor, Arc<Mutex<LagArm>>) {
    let mut config = ArmConfig::rebot_dm();
    config.planned_max_velocity = cap;
    config.following_error.persistence_ms = SCALED_PERSISTENCE_MS;
    let home: [f32; 7] = config.home_positions.clone().try_into().unwrap();
    let arm = Arc::new(Mutex::new(LagArm { q: home, v: [0.0; 7], enabled: EnableState::Disabled }));
    let supervisor = ArmSupervisor::new(config).with_driver(Box::new(LagDriver(arm.clone())));
    supervisor.set_lease_enabled(false).await;
    supervisor.tick(DT).await;
    supervisor.set_torque(true).await.unwrap();
    (supervisor, arm)
}

/// Ticks in scaled real time until the trajectory and settle are done or a fault latches.
async fn run(supervisor: &ArmSupervisor, sim_seconds: f64) -> (armd::ArmObservation, [f32; 6]) {
    let mut max_err = [0.0f32; 6];
    let mut obs = supervisor.get_observation().await;
    for _ in 0..(sim_seconds / DT) as usize {
        tokio::time::sleep(WALL_PER_TICK).await;
        obs = supervisor.tick(DT).await;
        for i in 0..6 { max_err[i] = max_err[i].max((obs.commanded_positions[i] - obs.positions[i]).abs()); }
        if obs.safety_state.is_stopped() { break; }
    }
    (obs, max_err)
}

#[tokio::test]
async fn lag_model_inspect_then_park_has_no_tracking_fault_with_cap() {
    let (supervisor, arm) = lag_arm(Some(0.45)).await;
    supervisor.jog(INSPECT.to_vec(), None, None).await.unwrap();
    let (obs, err_inspect) = run(&supervisor, 9.0).await;
    assert!(!obs.safety_state.is_stopped(), "inspect faulted: {:?}", obs.safety_state);
    assert!(err_inspect.iter().all(|e| *e < 0.4), "lag {err_inspect:?}");
    supervisor.apply_stop_policy(armd::StopPolicy::NormalUnload).await;
    assert!(supervisor.inner().read().await.pending_normal_unload);
    let (obs, err_park) = run(&supervisor, 14.0).await;
    assert!(!obs.safety_state.is_stopped(), "park faulted: {:?}", obs.safety_state);
    assert!(err_park.iter().all(|e| *e < 0.4), "lag {err_park:?}");
    assert_eq!(obs.safety_state, SafetyState::Idle, "park ends torque-off");
    assert_eq!(arm.lock().unwrap().enabled, EnableState::Disabled);
    println!("max |cmd-meas| inspect {err_inspect:?} park {err_park:?}");
}

#[tokio::test]
async fn lag_model_without_cap_reproduces_the_tracking_fault_and_names_the_joint() {
    let (supervisor, _arm) = lag_arm(None).await;
    supervisor.jog(INSPECT.to_vec(), None, None).await.unwrap();
    let (obs, err) = run(&supervisor, 6.0).await;
    let SafetyState::Fault(FaultReason::Other(msg)) = &obs.safety_state else {
        panic!("expected tracking fault, got {:?} (lag {err:?})", obs.safety_state);
    };
    println!("uncapped fault: {msg}; max lag {err:?}");
    assert!(msg.starts_with("Persistent joint tracking error: joint "), "{msg}");
    let joint: usize = msg["Persistent joint tracking error: joint ".len()..].split(' ').next().unwrap().parse().unwrap();
    assert!((2..=4).contains(&joint), "{msg}");
    assert!(msg.contains(" cmd ") && msg.contains(" meas ") && msg.ends_with(" rad"), "{msg}");
}

#[tokio::test]
async fn watchdog_park_from_far_j6_gets_an_extended_deadline_and_completes() {
    let mut config = ArmConfig::rebot_dm();
    config.lease_timeout_ms = 50;
    let supervisor = ArmSupervisor::new(config);
    supervisor.set_lease_enabled(false).await;
    supervisor.set_torque(true).await.unwrap();
    let mut far = ArmConfig::rebot_dm().home_positions;
    far[5] -= 4.7;
    supervisor.jog(far.clone(), None, None).await.unwrap();
    for _ in 0..800 { supervisor.tick(0.05).await; }
    assert!((supervisor.get_observation().await.positions[5] - far[5]).abs() < 1e-3);

    supervisor.set_lease_enabled(true).await;
    tokio::time::sleep(Duration::from_millis(80)).await;
    let obs = supervisor.tick(DT).await;
    assert!(!obs.safety_state.is_stopped(), "{:?}", obs.safety_state);
    {
        let inner = supervisor.inner().read().await;
        assert!(inner.pending_normal_unload);
        let planned = inner.unload_planned_duration.unwrap();
        assert!((planned - 1.875 * 4.7 / 0.45).abs() < 0.05, "planned {planned}");
        let timeout = inner.watchdog_park_timeout();
        assert_eq!(timeout, (Duration::from_secs_f64(planned) + WATCHDOG_PARK_MARGIN).max(WATCHDOG_PARK_TIMEOUT));
        assert!(timeout > Duration::from_secs(15));
        let remaining = inner.watchdog_park_deadline.unwrap() - Instant::now();
        assert!(remaining > Duration::from_secs(15), "deadline extended: {remaining:?}");
    }
    for _ in 0..600 { supervisor.tick(0.05).await; }
    let obs = supervisor.get_observation().await;
    assert_eq!(obs.safety_state, SafetyState::Idle, "watchdog park completes");
    assert!(!obs.is_torqued);
    assert!(supervisor.inner().read().await.watchdog_park_deadline.is_none());
}

#[test]
fn short_park_keeps_the_ten_second_floor() {
    let mut inner = armd::SupervisorInner::new(ArmConfig::rebot_dm());
    assert_eq!(inner.watchdog_park_timeout(), WATCHDOG_PARK_TIMEOUT);
    inner.unload_planned_duration = Some(2.0);
    assert_eq!(inner.watchdog_park_timeout(), WATCHDOG_PARK_TIMEOUT);
    inner.unload_planned_duration = Some(f64::NAN);
    assert_eq!(inner.watchdog_park_timeout(), WATCHDOG_PARK_TIMEOUT);
}
