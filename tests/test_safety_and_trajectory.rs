use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::time::Duration;
use armd::{
    ArmConfig, ArmError, ArmSupervisor, FaultReason, JointLimit, JointState,
    QuinticPolynomial, SafetyError, SafetyManager, SafetyState, StopPolicy, TrajectoryError,
    TrajectoryPlanner,
};
use armd::driver::{ArmDriver, ArmDriverError, JointTarget};

#[test]
fn test_quintic_polynomial_boundary_conditions() {
    let p0 = 0.5;
    let p1 = 2.5;
    let v0 = 0.0;
    let v1 = 0.0;
    let a0 = 0.0;
    let a1 = 0.0;
    let duration = 2.0;

    let poly = QuinticPolynomial::new(p0, v0, a0, p1, v1, a1, duration).unwrap();

    // Check t = 0
    let s_start = poly.position(0.0);
    let v_start = poly.velocity(0.0);
    let a_start = poly.acceleration(0.0);
    assert!((s_start - p0).abs() < 1e-6, "s(0) = {}, expected {}", s_start, p0);
    assert!((v_start - v0).abs() < 1e-6, "v(0) = {}, expected {}", v_start, v0);
    assert!((a_start - a0).abs() < 1e-6, "a(0) = {}, expected {}", a_start, a0);

    // Check t = duration
    let s_end = poly.position(duration);
    let v_end = poly.velocity(duration);
    let a_end = poly.acceleration(duration);
    assert!((s_end - p1).abs() < 1e-6, "s(T) = {}, expected {}", s_end, p1);
    assert!((v_end - v1).abs() < 1e-6, "v(T) = {}, expected {}", v_end, v1);
    assert!((a_end - a1).abs() < 1e-6, "a(T) = {}, expected {}", a_end, a1);

    // Check smoothness / continuity across intermediate points
    let steps = 100;
    let mut prev_pos = s_start;
    for i in 1..=steps {
        let t = duration * (i as f64) / (steps as f64);
        let pos = poly.position(t);
        let vel = poly.velocity(t);
        let acc = poly.acceleration(t);

        // Position must advance monotonically for single direction motion
        assert!(pos >= prev_pos - 1e-6);
        // Velocity must remain finite and bounded
        assert!(vel >= 0.0 - 1e-6);
        assert!(acc.is_finite());

        prev_pos = pos;
    }
}

#[test]
fn test_multi_joint_trajectory_planning() {
    let limits = vec![
        JointLimit::new("j1", -3.14, 3.14, 2.0, 5.0),
        JointLimit::new("j2", -3.14, 3.14, 1.0, 3.0),
    ];

    let start_pos = vec![0.0, 0.0];
    let start_vel = vec![0.0, 0.0];
    let target_pos = vec![1.0, 0.5];
    let target_vel = vec![0.0, 0.0];

    // Automatic duration planning based on velocity/acceleration constraints
    let traj = TrajectoryPlanner::plan(
        &start_pos,
        &start_vel,
        &target_pos,
        &target_vel,
        None,
        &limits,
    )
    .unwrap();

    assert!(traj.duration >= 0.5);

    // Sample at midpoint
    let mid_sample = traj.sample(traj.duration * 0.5);
    assert!(mid_sample.positions[0] > 0.0 && mid_sample.positions[0] < 1.0);
    assert!(mid_sample.positions[1] > 0.0 && mid_sample.positions[1] < 0.5);

    // Sample at completion
    let end_sample = traj.sample(traj.duration);
    assert!((end_sample.positions[0] - target_pos[0]).abs() < 1e-5);
    assert!((end_sample.positions[1] - target_pos[1]).abs() < 1e-5);
    assert!(end_sample.velocities[0].abs() < 1e-5);
    assert!(end_sample.velocities[1].abs() < 1e-5);
}

#[test]
fn test_safety_soft_limits() {
    let limits = vec![
        JointLimit::new("j1", -1.0, 1.0, 2.0, 5.0),
        JointLimit::new("j2", -2.0, 2.0, 2.0, 5.0),
    ];

    let mut sm = SafetyManager::new();
    sm.set_torque(true).unwrap();

    // Valid target
    assert!(sm
        .validate_motion_command(None, &[0.5, -1.5], &limits)
        .is_ok());

    // Exceeding j1 max
    let err_max = sm
        .validate_motion_command(None, &[1.5, 0.0], &limits)
        .unwrap_err();
    assert!(matches!(err_max, SafetyError::SoftLimitExceeded { ref joint, .. } if joint == "j1"));

    // Exceeding j2 min
    let err_min = sm
        .validate_motion_command(None, &[0.0, -2.5], &limits)
        .unwrap_err();
    assert!(matches!(err_min, SafetyError::SoftLimitExceeded { ref joint, .. } if joint == "j2"));
}

#[test]
fn test_safety_epoch_generation_and_estop_latch() {
    let limits = vec![JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0)];
    let mut sm = SafetyManager::new();
    let initial_epoch = sm.epoch_id();
    assert_eq!(initial_epoch, 1);

    // Arming
    sm.set_torque(true).unwrap();
    assert_eq!(*sm.state(), SafetyState::Armed);

    // Command with matching epoch succeeds
    assert!(sm
        .validate_motion_command(Some(initial_epoch), &[1.0], &limits)
        .is_ok());

    // Command with old/mismatched epoch is rejected immediately
    let mismatch_err = sm
        .validate_motion_command(Some(initial_epoch + 1), &[1.0], &limits)
        .unwrap_err();
    assert!(matches!(
        mismatch_err,
        SafetyError::EpochMismatch {
            expected: 1,
            received: 2
        }
    ));

    // Trigger E-Stop -> epoch increments
    let new_epoch = sm.trigger_e_stop();
    assert_eq!(new_epoch, initial_epoch + 1);
    assert_eq!(*sm.state(), SafetyState::EStop);

    // Any motion command while in E-Stop is rejected
    let estop_err = sm
        .validate_motion_command(Some(new_epoch), &[1.0], &limits)
        .unwrap_err();
    assert!(matches!(estop_err, SafetyError::EStopActive));

    // Clearing fault restores Idle and advances epoch again to invalidate prior in-flight commands
    let cleared_state = sm.clear_fault().unwrap();
    assert_eq!(cleared_state, SafetyState::Idle);
    let post_clear_epoch = sm.epoch_id();
    assert_eq!(post_clear_epoch, new_epoch + 1);

    // Old commands with previous epochs are now dead
    let old_epoch_err = sm
        .validate_motion_command(Some(initial_epoch), &[1.0], &limits)
        .unwrap_err();
    assert!(matches!(old_epoch_err, SafetyError::NotArmed(_)));
}

#[tokio::test]
async fn test_supervisor_stop_policies() {
    let mut config = ArmConfig::default();
    config.dof = 2;
    config.joint_names = vec!["j1".into(), "j2".into()];
    config.limits = vec![
        JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0),
        JointLimit::new("j2", -3.0, 3.0, 2.0, 5.0),
    ];
    config.home_positions = vec![0.0, 0.0];

    let supervisor = ArmSupervisor::new(config);

    // 1. EStopLatch policy
    supervisor.set_torque(true).await.unwrap();
    supervisor.apply_stop_policy(StopPolicy::EStopLatch).await;
    let obs = supervisor.get_observation().await;
    assert_eq!(obs.safety_state, SafetyState::EStop);
    assert_eq!(obs.epoch_id, 2);

    // Clear fault
    supervisor.clear_fault().await.unwrap();

    // 2. ThermalTrip policy
    supervisor.set_torque(true).await.unwrap();
    supervisor.apply_stop_policy(StopPolicy::ThermalTrip).await;
    let obs = supervisor.get_observation().await;
    assert!(matches!(obs.safety_state, SafetyState::Fault(FaultReason::ThermalTrip { .. })));

    supervisor.clear_fault().await.unwrap();

    // 3. BusLossFallback policy
    supervisor.set_torque(true).await.unwrap();
    supervisor.apply_stop_policy(StopPolicy::BusLossFallback).await;
    let obs = supervisor.get_observation().await;
    assert!(matches!(obs.safety_state, SafetyState::Fault(FaultReason::BusLoss)));

    supervisor.clear_fault().await.unwrap();

    // 4. NormalUnload policy
    supervisor.set_torque(true).await.unwrap();
    // Jog somewhere
    supervisor.jog(vec![1.0, 1.0], Some(0.5), None).await.unwrap();
    supervisor.tick(0.2).await;
    supervisor.apply_stop_policy(StopPolicy::NormalUnload).await;
    // Step until trajectory finishes
    for _ in 0..100 {
        supervisor.tick(0.05).await;
    }
    let obs = supervisor.get_observation().await;
    assert_eq!(obs.safety_state, SafetyState::Armed);
    assert!(obs.is_torqued, "NormalUnload must maintain holding torque at home position");
    assert!((obs.positions[0] - 0.0).abs() < 1e-4);
    assert!((obs.positions[1] - 0.0).abs() < 1e-4);
}

#[tokio::test]
async fn test_supervisor_watchdog_lease_timeout() {
    let mut config = ArmConfig::default();
    config.dof = 1;
    config.limits = vec![JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0)];
    config.home_positions = vec![0.0];
    config.lease_timeout_ms = 50; // 50ms short timeout for test

    let supervisor = ArmSupervisor::new(config);
    supervisor.set_torque(true).await.unwrap();
    supervisor.jog(vec![1.0], Some(1.0), None).await.unwrap();

    // Sleep longer than lease_timeout
    tokio::time::sleep(Duration::from_millis(70)).await;

    // Next tick should detect lease expiration
    let obs = supervisor.tick(0.02).await;
    assert!(matches!(obs.safety_state, SafetyState::Fault(FaultReason::WatchdogTimeout)));
}

#[test]
fn test_trajectory_nan_protection_and_physical_min_duration() {
    let limits = vec![JointLimit::new("j1", -3.14, 3.14, 2.0, 5.0)];

    // 1. NaN and Inf position rejection
    let nan_start = vec![f32::NAN];
    let ok_vec = vec![0.0];
    assert!(matches!(
        TrajectoryPlanner::plan(&nan_start, &ok_vec, &ok_vec, &ok_vec, None, &limits),
        Err(TrajectoryError::InvalidValue)
    ));

    let inf_target = vec![f32::INFINITY];
    assert!(matches!(
        TrajectoryPlanner::plan(&ok_vec, &ok_vec, &inf_target, &ok_vec, None, &limits),
        Err(TrajectoryError::InvalidValue)
    ));

    // 2. Physical minimum duration and minimum duration clamp (>= 0.04s)
    let traj_tiny = TrajectoryPlanner::plan(
        &ok_vec,
        &ok_vec,
        &ok_vec, // zero displacement
        &ok_vec,
        Some(0.0001), // requested duration 0.1ms
        &limits,
    )
    .unwrap();
    // Must be clamped to at least 40ms (0.04s)
    assert!(traj_tiny.duration >= 0.04);

    // Large displacement: delta_p = 3.0, v_max = 2.0 => t_v = 1.875 * 3 / 2 = 2.8125s
    // Even if requested 0.1s, physical minimum t_min must override it
    let target_far = vec![3.0];
    let traj_clamped = TrajectoryPlanner::plan(
        &ok_vec,
        &ok_vec,
        &target_far,
        &ok_vec,
        Some(0.1),
        &limits,
    )
    .unwrap();
    assert!(traj_clamped.duration >= 2.8);
}

#[test]
fn test_safety_nan_protection() {
    let limits = vec![JointLimit::new("j1", -2.0, 2.0, 1.0, 3.0)];
    let mut sm = SafetyManager::new();
    sm.set_torque(true).unwrap();

    // 1. NaN in validate_motion_command
    let nan_target = vec![f32::NAN];
    assert_eq!(
        sm.validate_motion_command(Some(1), &nan_target, &limits),
        Err(SafetyError::InvalidValue)
    );

    // 2. NaN in check_motion_limits
    assert_eq!(
        sm.check_motion_limits(&[f32::NAN], &[0.0], &[0.0], &limits),
        Err(SafetyError::InvalidValue)
    );
    assert_eq!(
        sm.check_motion_limits(&[0.0], &[f32::INFINITY], &[0.0], &limits),
        Err(SafetyError::InvalidValue)
    );
    assert_eq!(
        sm.check_motion_limits(&[0.0], &[0.0], &[f32::NAN], &limits),
        Err(SafetyError::InvalidValue)
    );
}

#[tokio::test]
async fn test_supervisor_torque_disable_and_estop_state_clearing() {
    let mut config = ArmConfig::default();
    config.dof = 1;
    config.limits = vec![JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0)];
    config.home_positions = vec![0.0];

    let supervisor = ArmSupervisor::new(config);
    supervisor.set_torque(true).await.unwrap();

    // 1. Jog to start trajectory
    supervisor.jog(vec![2.0], Some(1.0), None).await.unwrap();
    supervisor.tick(0.1).await;

    // Disabling torque while moving / mid-air must be rejected with ArmNotAtRestingPose
    let res = supervisor.set_torque(false).await;
    assert!(matches!(
        res,
        Err(ArmError::Safety(SafetyError::ArmNotAtRestingPose { .. }))
    ));
    let obs = supervisor.get_observation().await;
    assert!(obs.is_torqued, "Torque must remain enabled when not at resting pose");

    // 2. Normal unload interrupted by E-Stop latch
    supervisor.apply_stop_policy(StopPolicy::NormalUnload).await;
    // Interrupt with E-Stop
    supervisor.execute_stop(StopPolicy::EStopLatch).await;
    let obs = supervisor.get_observation().await;
    assert_eq!(obs.safety_state, SafetyState::EStop);
    assert_eq!(obs.velocities[0], 0.0);
}

#[tokio::test]
async fn test_supervisor_servo_high_frequency_teleop() {
    let mut config = ArmConfig::default();
    config.dof = 1;
    config.limits = vec![JointLimit::new("j1", -1.0, 1.0, 2.0, 5.0)];
    config.home_positions = vec![0.0];

    let supervisor = ArmSupervisor::new(config);
    supervisor.set_torque(true).await.unwrap();

    // Rejection with mismatched epoch_id
    assert!(matches!(
        supervisor.servo(vec![0.5], 999).await,
        Err(ArmError::Safety(SafetyError::EpochMismatch { .. }))
    ));

    // High frequency teleop target to 0.10 (current is 0.0) with valid epoch_id = 1 (within 0.15 rad jump limit)
    // Acceleration is bounded at 5 rad/s², so the first 20ms step is 0.002 rad
    // at 0.1 rad/s rather than an instantaneous 2 rad/s velocity jump.
    supervisor.servo(vec![0.10], 1).await.unwrap();
    let obs = supervisor.tick(0.02).await;
    assert!(obs.safety_state.is_operational());
    assert!((obs.positions[0] - 0.002).abs() < 1e-4);
    assert!((obs.velocities[0] - 0.1).abs() < 1e-3);

    // Dirty frame with instantaneous jump > 0.15 rad must be dropped
    supervisor.servo(vec![0.5], 1).await.unwrap();
    let obs_dirty = supervisor.tick(0.02).await;
    assert!(obs_dirty.positions[0] < 0.15, "Dirty frame > 0.15 rad must be discarded");
}

#[derive(Clone)]
struct MockDriver {
    torque_enabled: Arc<AtomicBool>,
    written_targets: Arc<Mutex<Vec<JointTarget>>>,
    read_calls: Arc<AtomicUsize>,
    set_torque_calls: Arc<AtomicUsize>,
}

impl MockDriver {
    fn new() -> (Self, Arc<AtomicBool>, Arc<Mutex<Vec<JointTarget>>>) {
        let torque = Arc::new(AtomicBool::new(false));
        let targets = Arc::new(Mutex::new(Vec::new()));
        let read_calls = Arc::new(AtomicUsize::new(0));
        let set_torque_calls = Arc::new(AtomicUsize::new(0));
        (
            Self {
                torque_enabled: torque.clone(),
                written_targets: targets.clone(),
                read_calls,
                set_torque_calls,
            },
            torque,
            targets,
        )
    }

    fn new_with_counters() -> (Self, Arc<AtomicBool>, Arc<AtomicUsize>, Arc<AtomicUsize>) {
        let torque = Arc::new(AtomicBool::new(false));
        let targets = Arc::new(Mutex::new(Vec::new()));
        let read_calls = Arc::new(AtomicUsize::new(0));
        let set_torque_calls = Arc::new(AtomicUsize::new(0));
        (
            Self {
                torque_enabled: torque.clone(),
                written_targets: targets,
                read_calls: read_calls.clone(),
                set_torque_calls: set_torque_calls.clone(),
            },
            torque,
            read_calls,
            set_torque_calls,
        )
    }
}

impl ArmDriver for MockDriver {
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        self.read_calls.fetch_add(1, Ordering::SeqCst);
        Ok(vec![JointState {
            position: 0.123,
            velocity: 0.0,
            torque: 0.5,
            temperature: 32.0,
        }])
    }

    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        let mut guard = self.written_targets.lock().unwrap();
        *guard = targets.to_vec();
        Ok(())
    }

    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        self.set_torque_calls.fetch_add(1, Ordering::SeqCst);
        self.torque_enabled.store(enabled, Ordering::SeqCst);
        Ok(())
    }
}

#[tokio::test]
async fn test_supervisor_hardware_driver_mounting_and_anti_gravity_holding() {
    let mut config = ArmConfig::default();
    config.dof = 1;
    config.limits = vec![JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0)];
    config.home_positions = vec![0.0];

    let supervisor = ArmSupervisor::new(config);
    let (mock, torque_flag, written_targets) = MockDriver::new();
    supervisor.set_driver(Box::new(mock)).await;

    // Enabling torque delegates to mounted driver and reads current physical position (0.123)
    supervisor.set_torque(true).await.unwrap();
    assert!(torque_flag.load(Ordering::SeqCst));

    // Tick reads telemetry from driver (measured_positions)
    let obs = supervisor.tick(0.02).await;
    assert!((obs.positions[0] - 0.123).abs() < 1e-4);
    assert_eq!(obs.temperatures[0], 32.0);

    // Plan motion to 2.0 rad
    supervisor.jog(vec![2.0], None, Some(1)).await.unwrap();

    // Advance 1 tick (20ms) - should interpolate smoothly
    let _ = supervisor.tick(0.02).await;
    {
        let guard = written_targets.lock().unwrap();
        assert_eq!(guard.len(), 1);
        // Commanded position is smoothly interpolating towards 2.0, not jumping to 2.0
        assert!(guard[0].position > 0.123 && guard[0].position < 0.5);
    }

    // Advance until trajectory completes
    while supervisor.get_observation().await.safety_state == SafetyState::Moving {
        supervisor.tick(0.1).await;
    }
    // One more tick in holding phase
    supervisor.tick(0.02).await;

    // Now in holding phase:
    // 1. Measured telemetry reflects MockDriver's simulated sensor feedback (0.123)
    let obs_hold = supervisor.get_observation().await;
    assert!((obs_hold.positions[0] - 0.123).abs() < 1e-4);

    // 2. Hardware driver target remains locked at commanded endpoint (2.0 rad) with 0.0 velocity!
    // It is NEVER overwritten by the measured position (0.123 rad), guaranteeing anti-gravity holding torque!
    let guard = written_targets.lock().unwrap();
    assert_eq!(guard.len(), 1);
    assert!((guard[0].position - 2.0).abs() < 1e-4);
    assert_eq!(guard[0].velocity, Some(0.0));
}

#[tokio::test]
async fn test_supervisor_watchdog_not_renewed_on_invalid_command() {
    let mut config = ArmConfig::default();
    config.dof = 1;
    config.limits = vec![JointLimit::new("j1", -1.0, 1.0, 2.0, 5.0)];
    config.lease_timeout_ms = 100;
    config.home_positions = vec![0.0];

    let supervisor = ArmSupervisor::new(config);
    supervisor.set_torque(true).await.unwrap();

    // Sleep 60ms
    tokio::time::sleep(Duration::from_millis(60)).await;

    // Send invalid jog command (epoch mismatch)
    let bad_res = supervisor.jog(vec![0.5], Some(1.0), Some(999)).await;
    assert!(bad_res.is_err());

    // Sleep another 50ms (total elapsed > 100ms lease timeout)
    tokio::time::sleep(Duration::from_millis(50)).await;

    // Watchdog should expire because invalid command must NOT renew lease
    let obs = supervisor.tick(0.02).await;
    assert!(matches!(obs.safety_state, SafetyState::Fault(FaultReason::WatchdogTimeout)));
}

#[test]
fn test_trajectory_constraints_unattainable_on_soft_limits_or_impossible_dynamics() {
    let limits = vec![JointLimit::new("j1", -1.0, 1.0, 2.0, 5.0)];
    let ok_start = vec![0.0];
    let ok_vel = vec![0.0];

    // Target position outside limits [-1.0, 1.0]
    let out_of_bounds_target = vec![2.5];
    let res = TrajectoryPlanner::plan(
        &ok_start,
        &ok_vel,
        &out_of_bounds_target,
        &ok_vel,
        None,
        &limits,
    );
    assert!(matches!(res, Err(TrajectoryError::ConstraintsUnattainable)));
}

#[test]
fn test_trajectory_nonzero_initial_velocity_limit_expansion() {
    let limits = vec![JointLimit::new("j1", -3.14, 3.14, 2.0, 5.0)];
    let start_pos = vec![0.0];
    let start_vel = vec![1.8]; // near max velocity
    let target_pos = vec![0.2];
    let target_vel = vec![0.0];

    // If rest-to-rest formula alone was used, T would be tiny (1.875 * 0.2 / 2.0 = 0.1875s)
    // which would cause massive deceleration overshoot > 5.0 rad/s^2!
    let traj = TrajectoryPlanner::plan(
        &start_pos,
        &start_vel,
        &target_pos,
        &target_vel,
        None,
        &limits,
    )
    .unwrap();

    // Verify all 20 sample points strictly respect max_velocity and max_acceleration
    const NUM_SAMPLES: usize = 20;
    for s in 0..=NUM_SAMPLES {
        let t = traj.duration * (s as f64) / (NUM_SAMPLES as f64);
        let pt = traj.sample(t);
        assert!(
            pt.velocities[0].abs() <= limits[0].max_velocity * 1.01,
            "Velocity {} exceeded max {}",
            pt.velocities[0],
            limits[0].max_velocity
        );
        assert!(
            pt.accelerations[0].abs() <= limits[0].max_acceleration * 1.01,
            "Acceleration {} exceeded max {}",
            pt.accelerations[0],
            limits[0].max_acceleration
        );
    }
}

struct UnresponsiveMockDriver;

impl ArmDriver for UnresponsiveMockDriver {
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        Ok(Vec::new()) // No valid feedback received yet
    }

    fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        Ok(())
    }

    fn set_torque(&mut self, _enabled: bool) -> Result<(), ArmDriverError> {
        Ok(())
    }

    fn is_feedback_valid(&self) -> bool {
        false
    }
}

#[tokio::test]
async fn test_set_torque_fails_when_feedback_unavailable() {
    let mut config = ArmConfig::default();
    config.dof = 1;
    config.limits = vec![JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0)];
    config.home_positions = vec![0.0];

    let supervisor = ArmSupervisor::new(config);
    supervisor.set_driver(Box::new(UnresponsiveMockDriver)).await;

    // Enabling torque must fail with FeedbackUnavailable because hardware hasn't returned valid angles
    let res = supervisor.set_torque(true).await;
    assert!(matches!(
        res,
        Err(ArmError::Safety(SafetyError::FeedbackUnavailable))
    ));

    // Confirm arm remains un-torqued
    let obs = supervisor.get_observation().await;
    assert!(!obs.is_torqued);
    assert_eq!(obs.safety_state, SafetyState::Idle);
}

#[tokio::test]
async fn test_clear_fault_noop_when_not_fault() {
    let mut config = ArmConfig::default();
    config.dof = 1;
    config.limits = vec![JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0)];
    config.home_positions = vec![0.0];

    let supervisor = ArmSupervisor::new(config);
    supervisor.set_torque(true).await.unwrap();

    let obs_before = supervisor.get_observation().await;
    assert_eq!(obs_before.safety_state, SafetyState::Armed);
    assert!(obs_before.is_torqued);
    let epoch_before = obs_before.epoch_id;

    // Calling clear_fault when in Armed state must NOT reset to Idle or drop torque
    let state = supervisor.clear_fault().await.unwrap();
    assert_eq!(state, SafetyState::Armed);

    let obs_after = supervisor.get_observation().await;
    assert_eq!(obs_after.safety_state, SafetyState::Armed);
    assert!(obs_after.is_torqued);
    assert_eq!(obs_after.epoch_id, epoch_before);
}

#[tokio::test]
async fn test_set_torque_in_estop_makes_zero_driver_calls() {
    let mut config = ArmConfig::default();
    config.dof = 1;
    config.limits = vec![JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0)];
    config.home_positions = vec![0.0];

    let supervisor = ArmSupervisor::new(config);
    let (mock, torque_flag, read_calls, set_torque_calls) = MockDriver::new_with_counters();
    supervisor.set_driver(Box::new(mock)).await;

    // Trigger E-Stop first
    supervisor.trigger_e_stop().await;
    let initial_reads = read_calls.load(Ordering::SeqCst);
    let initial_torque_calls = set_torque_calls.load(Ordering::SeqCst);

    // Attempting set_torque(true) while in E-Stop must fail immediately
    let res = supervisor.set_torque(true).await;
    assert!(matches!(res, Err(ArmError::Safety(SafetyError::EStopActive))));

    // Verify driver was NEVER touched (0 read calls, 0 set_torque calls)
    assert_eq!(read_calls.load(Ordering::SeqCst), initial_reads);
    assert_eq!(set_torque_calls.load(Ordering::SeqCst), initial_torque_calls);
    assert!(!torque_flag.load(Ordering::SeqCst));

    let obs = supervisor.get_observation().await;
    assert!(!obs.is_torqued);
    assert_eq!(obs.safety_state, SafetyState::EStop);
}

struct FailingEnableDriver {
    set_torque_calls: Arc<AtomicUsize>,
    last_torque_state: Arc<AtomicBool>,
}

impl ArmDriver for FailingEnableDriver {
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        Ok(vec![JointState {
            position: 0.1,
            velocity: 0.0,
            torque: 0.0,
            temperature: 25.0,
        }])
    }

    fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        Ok(())
    }

    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        self.set_torque_calls.fetch_add(1, Ordering::SeqCst);
        self.last_torque_state.store(enabled, Ordering::SeqCst);
        if enabled {
            Err(ArmDriverError::HardwareFault("Hardware enable failed".to_string()))
        } else {
            Ok(())
        }
    }
}

#[tokio::test]
async fn test_enable_hardware_failure_rolls_back_and_latches_fault() {
    let mut config = ArmConfig::default();
    config.dof = 1;
    config.limits = vec![JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0)];
    config.home_positions = vec![0.0];

    let supervisor = ArmSupervisor::new(config);
    let calls = Arc::new(AtomicUsize::new(0));
    let last_state = Arc::new(AtomicBool::new(false));
    let driver = FailingEnableDriver {
        set_torque_calls: calls.clone(),
        last_torque_state: last_state.clone(),
    };
    supervisor.set_driver(Box::new(driver)).await;

    // Enabling torque must fail and rollback
    let res = supervisor.set_torque(true).await;
    assert!(res.is_err());

    // 1 call to set_torque(true), followed by 1 rollback call to set_torque(false)
    assert_eq!(calls.load(Ordering::SeqCst), 2);
    assert!(!last_state.load(Ordering::SeqCst));

    // Confirm state is latched Fault and un-torqued
    let obs = supervisor.get_observation().await;
    assert!(!obs.is_torqued);
    assert!(matches!(obs.safety_state, SafetyState::Fault(_)));
}

struct StaleAfterEnableDriver {
    feedback_valid: Arc<AtomicBool>,
}

impl ArmDriver for StaleAfterEnableDriver {
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        Ok(vec![JointState {
            position: 0.0,
            velocity: 0.0,
            torque: 0.0,
            temperature: 25.0,
        }])
    }

    fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        Ok(())
    }

    fn set_torque(&mut self, _enabled: bool) -> Result<(), ArmDriverError> {
        Ok(())
    }

    fn is_feedback_valid(&self) -> bool {
        self.feedback_valid.load(Ordering::SeqCst)
    }
}

#[tokio::test]
async fn test_stale_feedback_triggers_bus_loss_fallback() {
    let mut config = ArmConfig::default();
    config.dof = 1;
    config.limits = vec![JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0)];
    config.home_positions = vec![0.0];

    let supervisor = ArmSupervisor::new(config);
    let valid_flag = Arc::new(AtomicBool::new(true));
    let driver = StaleAfterEnableDriver {
        feedback_valid: valid_flag.clone(),
    };
    supervisor.set_driver(Box::new(driver)).await;

    // Successfully arm first
    supervisor.set_torque(true).await.unwrap();
    let obs = supervisor.get_observation().await;
    assert_eq!(obs.safety_state, SafetyState::Armed);

    // Communication loss (is_feedback_valid becomes false)
    valid_flag.store(false, Ordering::SeqCst);

    // Next tick must immediately trigger BusLossFallback
    let obs_tick = supervisor.tick(0.02).await;
    assert_eq!(obs_tick.safety_state, SafetyState::Fault(FaultReason::BusLoss));
    assert!(obs_tick.is_torqued, "BusLossFallback must maintain holding torque to prevent dropping");
}
