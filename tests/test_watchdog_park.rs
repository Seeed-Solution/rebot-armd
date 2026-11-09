//! Lease watchdog expiry while torqued: park at the resting pose when the arm is
//! healthy, otherwise hold torque and latch Fault(WatchdogTimeout).
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
use armd::model::{ArmConfig, JointLimit, JointState, RestingPosePolicy};
use armd::safety::FaultReason;
use armd::{ArmSupervisor, SafetyState};

const LEASE_MS: u64 = 50;

fn policy(dof: usize) -> RestingPosePolicy {
    RestingPosePolicy { position_tolerances: vec![0.08; dof], max_total_deviation: 0.08 * dof as f32,
        max_velocity: 0.015, support_torque_limits: vec![(0, 1.0)], feedback_max_age_ms: 100 }
}

fn sim_config(resting_pose: bool) -> ArmConfig {
    let mut config = ArmConfig::default();
    config.dof = 2;
    config.limits = vec![JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0), JointLimit::new("j2", -3.0, 3.0, 2.0, 5.0)];
    config.home_positions = vec![0.0, 0.0];
    config.lease_timeout_ms = LEASE_MS;
    config.resting_pose = resting_pose.then(|| policy(2));
    config
}

/// Simulated arm, torqued and parked away from home with the lease watchdog armed.
async fn sim_torqued_away_from_home(resting_pose: bool) -> ArmSupervisor {
    let supervisor = ArmSupervisor::new(sim_config(resting_pose));
    supervisor.set_torque(true).await.unwrap();
    supervisor.jog(vec![1.0, 0.5], Some(0.3), None).await.unwrap();
    for _ in 0..80 { supervisor.tick(0.05).await; }
    let obs = supervisor.get_observation().await;
    assert!((obs.positions[0] - 1.0).abs() < 1e-3 && obs.is_torqued);
    supervisor
}

async fn expire_lease(supervisor: &ArmSupervisor) -> armd::ArmObservation {
    tokio::time::sleep(Duration::from_millis(LEASE_MS + 30)).await;
    supervisor.tick(0.02).await
}

#[tokio::test]
async fn lease_expiry_while_torqued_parks_then_cuts_torque_to_idle() {
    let supervisor = sim_torqued_away_from_home(true).await;
    let obs = expire_lease(&supervisor).await;
    assert!(!obs.safety_state.is_stopped(), "healthy arm must park, not fault: {:?}", obs.safety_state);
    {
        let inner = supervisor.inner().read().await;
        assert!(inner.pending_normal_unload && inner.watchdog_park_deadline.is_some());
        assert_eq!(inner.target_positions, vec![0.0, 0.0]);
    }
    for _ in 0..200 { supervisor.tick(0.05).await; }
    let obs = supervisor.get_observation().await;
    assert_eq!(obs.safety_state, SafetyState::Idle);
    assert!(!obs.is_torqued, "park must end with torque off");
    assert!(obs.positions.iter().all(|p| p.abs() < 1e-3), "torque cut only at home: {:?}", obs.positions);
    assert!(supervisor.inner().read().await.watchdog_park_deadline.is_none());
}

#[tokio::test]
async fn released_lease_while_torqued_parks_the_same_way() {
    let supervisor = sim_torqued_away_from_home(true).await;
    {
        let mut inner = supervisor.inner().write().await;
        let generation = inner.acquire_lease("client-a").unwrap();
        assert!(inner.release_lease("client-a", generation));
        inner.stop_motion(); // what the WS handler does on release / close
    }
    let obs = expire_lease(&supervisor).await;
    assert!(!obs.safety_state.is_stopped());
    assert!(supervisor.inner().read().await.pending_normal_unload);
    for _ in 0..200 { supervisor.tick(0.05).await; }
    let obs = supervisor.get_observation().await;
    assert_eq!(obs.safety_state, SafetyState::Idle);
    assert!(!obs.is_torqued);
}

#[tokio::test]
async fn new_lease_during_park_takes_over_and_can_move() {
    let supervisor = sim_torqued_away_from_home(true).await;
    expire_lease(&supervisor).await;
    for _ in 0..5 { supervisor.tick(0.02).await; }
    {
        let mut inner = supervisor.inner().write().await;
        assert!(inner.pending_normal_unload);
        inner.acquire_lease("client-b").expect("lease must not be refused during a park");
        assert!(!inner.pending_normal_unload && inner.active_trajectory.is_none());
        assert!(inner.watchdog_park_deadline.is_none());
    }
    for _ in 0..50 {
        supervisor.heartbeat("client-b", supervisor.get_observation().await.epoch_id).await.unwrap();
        supervisor.tick(0.02).await;
    }
    let obs = supervisor.get_observation().await;
    assert!(obs.is_torqued && !obs.safety_state.is_stopped(), "takeover holds torque: {:?}", obs.safety_state);
    supervisor.jog(vec![0.8, 0.4], Some(0.3), None).await.expect("new owner commands are accepted");
}

#[tokio::test]
async fn preempting_lease_during_park_cancels_it() {
    let supervisor = sim_torqued_away_from_home(true).await;
    expire_lease(&supervisor).await;
    supervisor.tick(0.02).await;
    let mut inner = supervisor.inner().write().await;
    assert!(inner.pending_normal_unload);
    inner.preempt_lease("conn-c", "cockpit").expect("preempt must not be refused during a park");
    assert!(!inner.pending_normal_unload && inner.watchdog_park_deadline.is_none());
    assert!(inner.is_torqued && !inner.safety.state().is_stopped());
}

#[tokio::test]
async fn no_resting_pose_keeps_watchdog_fault_with_torque_held() {
    let supervisor = sim_torqued_away_from_home(false).await;
    let obs = expire_lease(&supervisor).await;
    assert!(matches!(obs.safety_state, SafetyState::Fault(FaultReason::WatchdogTimeout)));
    assert!(obs.is_torqued);
    assert!(!supervisor.inner().read().await.pending_normal_unload);
}

#[tokio::test]
async fn already_latched_fault_is_not_parked() {
    let supervisor = sim_torqued_away_from_home(true).await;
    supervisor.inner().write().await.safety.trigger_fault(FaultReason::BusLoss);
    let obs = expire_lease(&supervisor).await;
    assert!(matches!(obs.safety_state, SafetyState::Fault(FaultReason::BusLoss)));
    assert!(obs.is_torqued, "torque stays on");
    assert!(!supervisor.inner().read().await.pending_normal_unload);
}

#[tokio::test]
async fn expiry_while_not_torqued_does_nothing() {
    let supervisor = ArmSupervisor::new(sim_config(true));
    let obs = expire_lease(&supervisor).await;
    assert_eq!(obs.safety_state, SafetyState::Idle);
    assert!(!obs.is_torqued);
    assert!(!supervisor.inner().read().await.pending_normal_unload);
}

// ---- hardware-like driver with controllable feedback age and fixed position ----

struct Hardware { position: f32, age: Option<Duration>, enabled: EnableState }
struct Driver(Arc<Mutex<Hardware>>);
impl ArmDriver for Driver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true,
            torque_feedforward: true, enable_feedback: true }
    }
    fn enable_state(&self) -> EnableState { self.0.lock().unwrap().enabled }
    fn is_feedback_valid(&self) -> bool { self.0.lock().unwrap().age.is_some_and(|a| a <= Duration::from_millis(250)) }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![self.0.lock().unwrap().age] }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        Ok(vec![JointState { position: self.0.lock().unwrap().position, ..JointState::default() }])
    }
    fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
    fn set_torque(&mut self, on: bool) -> Result<(), ArmDriverError> {
        self.0.lock().unwrap().enabled = if on { EnableState::Enabled } else { EnableState::Disabled };
        Ok(())
    }
}

async fn hw_torqued(position: f32) -> (ArmSupervisor, Arc<Mutex<Hardware>>) {
    let mut config = ArmConfig::default();
    config.dof = 1;
    config.home_positions = vec![0.0];
    config.limits = vec![JointLimit::new("axis", -2.0, 2.0, 2.0, 5.0)];
    config.lease_timeout_ms = LEASE_MS;
    config.resting_pose = Some(policy(1));
    let hw = Arc::new(Mutex::new(Hardware { position, age: Some(Duration::ZERO), enabled: EnableState::Disabled }));
    let supervisor = ArmSupervisor::new(config).with_driver(Box::new(Driver(hw.clone())));
    supervisor.tick(0.02).await;
    supervisor.set_torque(true).await.unwrap();
    (supervisor, hw)
}

#[tokio::test]
async fn stale_feedback_keeps_watchdog_fault_with_torque_held() {
    let (supervisor, hw) = hw_torqued(0.3).await;
    // Older than the resting-pose bound (100 ms), younger than bus loss (500 ms).
    hw.lock().unwrap().age = Some(Duration::from_millis(200));
    let obs = expire_lease(&supervisor).await;
    assert!(matches!(obs.safety_state, SafetyState::Fault(FaultReason::WatchdogTimeout)), "{:?}", obs.safety_state);
    assert!(obs.is_torqued);
    assert_eq!(hw.lock().unwrap().enabled, EnableState::Enabled);
}

#[tokio::test]
async fn park_that_cannot_reach_rest_falls_back_to_fault_without_cutting_torque() {
    // The joint stays at 0.3 rad (inside the following-error bound): the park never
    // satisfies the resting-pose check, so torque must never be cut.
    let (supervisor, hw) = hw_torqued(0.3).await;
    let obs = expire_lease(&supervisor).await;
    assert!(!obs.safety_state.is_stopped(), "{:?}", obs.safety_state);
    assert!(supervisor.inner().read().await.pending_normal_unload);
    for _ in 0..120 { supervisor.tick(0.02).await; }
    assert!(supervisor.get_observation().await.is_torqued, "never torque-off away from the resting pose");
    supervisor.inner().write().await.watchdog_park_deadline = Some(Instant::now());
    let obs = supervisor.tick(0.02).await;
    assert!(matches!(obs.safety_state, SafetyState::Fault(FaultReason::WatchdogTimeout)), "{:?}", obs.safety_state);
    assert!(obs.is_torqued);
    assert_eq!(hw.lock().unwrap().enabled, EnableState::Enabled);
    assert!(!supervisor.inner().read().await.pending_normal_unload);
}
