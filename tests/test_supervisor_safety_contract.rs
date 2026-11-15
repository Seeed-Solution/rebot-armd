use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
use armd::model::{ArmConfig, GravityModel, JointLimit, JointState, RestingPosePolicy};
use armd::{ArmSupervisor, SafetyState, SupervisorInner};
use armd::safety::FaultReason;

struct Hardware {
    states: Vec<JointState>,
    age: Option<Duration>,
    enabled: EnableState,
    disable_fails: bool,
    confirm_disable: bool,
    write_fails: bool,
    disable_calls: usize,
    written: Vec<Vec<JointTarget>>,
}
struct Driver { hardware: Arc<Mutex<Hardware>>, capabilities: DriverCapabilities, max_velocity: Option<f32> }
fn full_caps() -> DriverCapabilities {
    DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true,
        torque_feedforward: true, enable_feedback: true, load_feedback: false }
}
impl ArmDriver for Driver {
    fn capabilities(&self) -> DriverCapabilities { self.capabilities }
    fn enable_state(&self) -> EnableState { self.hardware.lock().unwrap().enabled }
    fn is_feedback_valid(&self) -> bool { self.hardware.lock().unwrap().age.is_some_and(|a| a <= Duration::from_millis(250)) }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { let h = self.hardware.lock().unwrap(); vec![h.age; h.states.len()] }
    fn max_command_velocity(&self, _: usize) -> Option<f32> { self.max_velocity }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> { Ok(self.hardware.lock().unwrap().states.clone()) }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        let mut h = self.hardware.lock().unwrap();
        if h.write_fails { return Err(ArmDriverError::HardwareFault("write failed".into())); }
        h.written.push(targets.to_vec()); Ok(())
    }
    fn set_torque(&mut self, on: bool) -> Result<(), ArmDriverError> {
        let mut h = self.hardware.lock().unwrap();
        if on { h.enabled = EnableState::Enabled; }
        else {
            h.disable_calls += 1;
            if h.disable_fails { return Err(ArmDriverError::HardwareFault("disable failed".into())); }
            if h.confirm_disable { h.enabled = EnableState::Disabled; }
        }
        Ok(())
    }
}
async fn setup(caps: DriverCapabilities, max_velocity: Option<f32>) -> (ArmSupervisor, Arc<Mutex<Hardware>>) {
    let mut config = ArmConfig::default();
    config.dof = 1;
    config.home_positions = vec![0.0];
    config.limits = vec![JointLimit::new("axis", -2.0, 2.0, 2.0, 5.0)];
    config.resting_pose = Some(RestingPosePolicy { positions: None, position_tolerances: vec![0.08], max_total_deviation: 0.08,
        max_velocity: 0.015, joint_max_velocities: vec![], support_torque_limits: vec![(0, 1.0)], support_load_limits: vec![], feedback_max_age_ms: 100 });
    let hardware = Arc::new(Mutex::new(Hardware { states: vec![JointState::default()], age: Some(Duration::ZERO),
        enabled: EnableState::Disabled, disable_fails: false, confirm_disable: true, write_fails: false,
        disable_calls: 0, written: vec![] }));
    let supervisor = ArmSupervisor::new(config).with_driver(Box::new(Driver {
        hardware: hardware.clone(), capabilities: caps, max_velocity,
    }));
    supervisor.set_lease_enabled(false).await;
    supervisor.set_torque(true).await.unwrap();
    (supervisor, hardware)
}

#[tokio::test]
async fn thermal_trip_overrides_estop_and_retries_failed_disable_without_epoch_churn() {
    let (supervisor, h) = setup(full_caps(), None).await;
    supervisor.trigger_e_stop().await;
    { let mut h = h.lock().unwrap(); h.states[0].temperature = 80.0; h.disable_fails = true; }
    let first = supervisor.tick(0.02).await;
    assert!(matches!(first.safety_state, SafetyState::Fault(FaultReason::ThermalTrip { .. })));
    assert!(first.is_torqued && first.disable_pending);
    assert!(!supervisor.shutdown_ready().await);
    assert!(supervisor.set_torque(true).await.is_err());
    assert_eq!(h.lock().unwrap().disable_calls, 1);
    let next = supervisor.tick(0.02).await;
    assert_eq!(next.epoch_id, first.epoch_id);
    assert_eq!(h.lock().unwrap().disable_calls, 1);
    supervisor.inner().write().await.last_disable_attempt = Some(Instant::now() - Duration::from_millis(251));
    h.lock().unwrap().disable_fails = false;
    let confirmed = supervisor.tick(0.02).await;
    assert!(!confirmed.is_torqued && !confirmed.disable_pending);
    assert_eq!(confirmed.epoch_id, first.epoch_id);
    assert_eq!(h.lock().unwrap().disable_calls, 2);
    assert!(supervisor.clear_fault().await.is_err(), "Hot feedback cannot reset thermal latch");
    h.lock().unwrap().states[0].temperature = 64.0;
    let err = supervisor.clear_fault().await.unwrap_err();
    assert!(err.to_string().contains("E-stop still latched"), "Expected E-stop still latched rejection, got: {err}");
    assert_eq!(supervisor.get_observation().await.safety_state, SafetyState::EStop);
    supervisor.clear_fault().await.unwrap();
    assert_eq!(supervisor.get_observation().await.safety_state, SafetyState::Idle);
}

#[tokio::test]
async fn successful_disable_write_without_feedback_is_not_reported_as_power_off() {
    let (supervisor, h) = setup(full_caps(), None).await;
    h.lock().unwrap().confirm_disable = false;
    assert!(supervisor.set_torque(false).await.is_err());
    let obs = supervisor.get_observation().await;
    assert!(obs.is_torqued && obs.disable_pending && obs.safety_state.is_stopped());
    assert!(!supervisor.shutdown_ready().await);
    supervisor.tick(0.02).await;
    assert!(h.lock().unwrap().written.is_empty(), "Pending shutdown must not send ordinary motion frames");
    h.lock().unwrap().enabled = EnableState::Disabled;
    let obs = supervisor.tick(0.02).await;
    assert!(!obs.is_torqued && !obs.disable_pending);
    assert!(supervisor.shutdown_ready().await);
}

#[tokio::test]
async fn missing_sensor_capabilities_and_old_samples_cannot_prove_safe_rest() {
    let (supervisor, _) = setup(DriverCapabilities::default(), None).await;
    assert!(!supervisor.is_at_safe_resting_pose().await);
    let (supervisor, h) = setup(full_caps(), None).await;
    assert!(supervisor.is_at_safe_resting_pose().await);
    h.lock().unwrap().age = Some(Duration::from_millis(150));
    supervisor.tick(0.02).await;
    assert!(!supervisor.is_at_safe_resting_pose().await, "Cache reads cannot renew a 150ms-old sample");
    supervisor.inner().write().await.config.resting_pose = None;
    h.lock().unwrap().age = Some(Duration::ZERO);
    assert!(!supervisor.is_at_safe_resting_pose().await, "Hardware needs a calibrated resting profile");
}

#[tokio::test]
async fn fault_reset_requires_fresh_known_enable_state_and_discards_old_targets() {
    let (supervisor, h) = setup(full_caps(), None).await;
    supervisor.trigger_e_stop().await;
    h.lock().unwrap().enabled = EnableState::Unknown;
    assert!(supervisor.clear_fault().await.is_err());
    { let mut h = h.lock().unwrap(); h.enabled = EnableState::Enabled; h.age = None; }
    assert!(supervisor.clear_fault().await.is_err());
    { let mut h = h.lock().unwrap(); h.age = Some(Duration::ZERO); h.states[0].position = 0.4; }
    supervisor.clear_fault().await.unwrap();
    let inner = supervisor.inner().read().await;
    assert_eq!(inner.commanded_positions, vec![0.4]);
    assert_eq!(inner.commanded_velocities, vec![0.0]);
    assert!(inner.pending_rearm);
}

#[tokio::test]
async fn sustained_following_error_latches_hold_but_transient_error_does_not() {
    let (supervisor, h) = setup(full_caps(), None).await;
    supervisor.inner().write().await.commanded_positions[0] = 0.8;
    let obs = supervisor.tick(0.02).await;
    assert!(!obs.safety_state.is_stopped());
    supervisor.inner().write().await.following_error_since = Some(Instant::now() - Duration::from_millis(501));
    let obs = supervisor.tick(0.02).await;
    assert!(obs.safety_state.is_stopped());
    let fault = obs.fault.as_ref().expect("fault field set in Fault");
    assert_eq!(fault.code, armd::FaultCode::FollowingError);
    assert!(!fault.recoverable);
    assert!(fault.message.starts_with("Persistent joint tracking error"), "{}", fault.message);
    assert_eq!(h.lock().unwrap().written.last().unwrap()[0].position, 0.0);
}

#[tokio::test]
async fn output_failure_latches_and_stale_reconnect_does_not_replay_old_target() {
    let (supervisor, h) = setup(full_caps(), None).await;
    h.lock().unwrap().write_fails = true;
    assert!(supervisor.tick(0.02).await.safety_state.is_stopped());
    let (supervisor, h) = setup(full_caps(), None).await;
    supervisor.inner().write().await.commanded_positions[0] = 0.3;
    h.lock().unwrap().age = None;
    supervisor.tick(0.02).await;
    assert!(h.lock().unwrap().written.is_empty());
    { let mut h = h.lock().unwrap(); h.age = Some(Duration::ZERO); h.states[0].position = -0.4; }
    let obs = supervisor.tick(0.02).await;
    assert!(obs.safety_state.is_stopped());
    assert_eq!(h.lock().unwrap().written.last().unwrap()[0].position, -0.4);
}

#[tokio::test]
async fn nonfinite_feedback_never_becomes_a_control_target() {
    let (supervisor, h) = setup(full_caps(), None).await;
    h.lock().unwrap().states[0].position = f32::NAN;
    let obs = supervisor.tick(0.02).await;
    assert!(obs.safety_state.is_stopped());
    assert!(!obs.feedback_valid);
    assert!(h.lock().unwrap().written.is_empty());
    assert!(supervisor.inner().read().await.commanded_positions.iter().all(|p| p.is_finite()));
    assert!(supervisor.clear_fault().await.is_err());
}

#[tokio::test]
async fn driver_speed_limit_and_explicit_gravity_model_are_enforced() {
    let (supervisor, _) = setup(full_caps(), Some(0.4)).await;
    assert_eq!(supervisor.get_config().await.limits[0].max_velocity, 0.4);
    assert!(!supervisor.is_gravity_compensated().await, "A driver is not a mechanical model");
    supervisor.inner().write().await.config.gravity_model = Some(GravityModel::RebotDevArm);
    // One axis does not satisfy the six-axis gravity model.
    supervisor.tick(0.02).await;
    let obs = supervisor.get_observation().await;
    assert!(!obs.gravity_compensated && obs.gravity_torques.is_empty());
}

#[tokio::test]
async fn control_loop_stall_latches_instead_of_advancing_old_trajectory() {
    let supervisor = ArmSupervisor::new(ArmConfig::default());
    supervisor.set_torque(true).await.unwrap();
    supervisor.jog(vec![0.3; 6], Some(1.0), None).await.unwrap();
    let obs = supervisor.tick(0.300).await;
    assert!(obs.safety_state.is_stopped());
    assert_eq!(obs.positions, vec![0.0; 6]);
    assert!(supervisor.inner().read().await.active_trajectory.is_none());
}

#[tokio::test]
async fn single_late_tick_below_stall_bound_keeps_arm_operational() {
    // One 100 ms scheduling hiccup on a loaded single-core host must not fault the arm.
    let supervisor = ArmSupervisor::new(ArmConfig::default());
    supervisor.set_torque(true).await.unwrap();
    supervisor.jog(vec![0.3; 6], Some(1.0), None).await.unwrap();
    let obs = supervisor.tick(0.100).await;
    assert!(!obs.safety_state.is_stopped(), "state: {:?}", obs.safety_state);
}

#[test]
fn soft_jerk_smooths_reversal_but_never_overrides_braking_bounds() {
    let limit = JointLimit::new("axis", -3.0, 3.0, 2.0, 5.0);
    let (mut q, mut v, mut a) = (0.0, 0.0, 0.0);
    for k in 0..16 {
        let target = if k < 8 { 2.0 } else { -2.0 };
        let (nq, nv, _) = SupervisorInner::bounded_servo_step_with_jerk(q, target, v, 0.02, &limit, a, Some(20.0)).unwrap();
        let na = (nv-v)/0.02;
        assert!((na-a).abs() <= 20.0*0.02 + 0.0001);
        assert!(na.abs() <= 5.0001);
        (q,v,a) = (nq,nv,na);
    }
    let limit = JointLimit::new("axis", -1.0, 1.0, 2.0, 5.0);
    let (q, v, _) = SupervisorInner::bounded_servo_step_with_jerk(0.93, 1.0, 0.8, 0.02, &limit, 5.0, Some(0.1)).unwrap();
    assert!(q <= 1.0);
    assert!((v as f64).powi(2)/10.0 <= (1.0-q as f64) + 1e-6, "Remaining distance must permit full braking");
    assert!(((v - 0.8)/0.02 - 5.0).abs() > 0.1 * 0.02, "Braking must override the soft jerk preference here");
    assert!((v - 0.8).abs() <= 5.0 * 0.02 + 1e-5);
}

#[tokio::test]
async fn jerk_enabled_servo_settles_after_small_target() {
    let mut config = ArmConfig::default();
    config.max_jerk = Some(50.0);
    let supervisor = ArmSupervisor::new(config);
    supervisor.set_lease_enabled(false).await;
    supervisor.set_torque(true).await.unwrap();
    supervisor.inner().write().await.motion_frame_timeout = Duration::from_secs(60);
    supervisor.servo(vec![0.1; 6], 1).await.unwrap();
    for _ in 0..300 { supervisor.tick(0.02).await; }
    let obs = supervisor.get_observation().await;
    assert!(!obs.safety_state.is_stopped());
    assert!((obs.positions[0] - 0.1).abs() < 1e-4, "position {}", obs.positions[0]);
    assert!(obs.velocities[0].abs() < 1e-4, "velocity {}", obs.velocities[0]);
}

async fn setup7() -> (ArmSupervisor, Arc<Mutex<Hardware>>) {
    let mut config = ArmConfig::default();
    config.dof = 7;
    config.home_positions = vec![0.0; 7];
    config.limits = (0..7).map(|i| JointLimit::new(format!("j{i}"), -6.0, 6.0, 2.0, 5.0)).collect();
    config.resting_pose = None;
    let hardware = Arc::new(Mutex::new(Hardware { states: vec![JointState::default(); 7], age: Some(Duration::ZERO),
        enabled: EnableState::Disabled, disable_fails: false, confirm_disable: true, write_fails: false,
        disable_calls: 0, written: vec![] }));
    let supervisor = ArmSupervisor::new(config).with_driver(Box::new(Driver {
        hardware: hardware.clone(), capabilities: full_caps(), max_velocity: None,
    }));
    supervisor.set_lease_enabled(false).await;
    supervisor.set_torque(true).await.unwrap();
    (supervisor, hardware)
}

#[tokio::test]
async fn gripper_trailing_its_command_does_not_latch_tracking_fault() {
    // Park closes the gripper at up to 2 rad/s; Damiao POS_VEL trails by ~0.9 rad.
    let (supervisor, _h) = setup7().await;
    supervisor.inner().write().await.commanded_positions[6] = -0.9;
    supervisor.tick(0.02).await;
    supervisor.inner().write().await.following_error_since = Some(Instant::now() - Duration::from_millis(501));
    let obs = supervisor.tick(0.02).await;
    assert!(!obs.safety_state.is_stopped(), "gripper lag must not fault the arm: {:?}", obs.safety_state);

    // An arm joint with the same error still latches.
    supervisor.inner().write().await.commanded_positions[0] = 0.9;
    supervisor.tick(0.02).await;
    supervisor.inner().write().await.following_error_since = Some(Instant::now() - Duration::from_millis(501));
    let obs = supervisor.tick(0.02).await;
    assert!(obs.safety_state.is_stopped());
}

#[tokio::test]
async fn short_feedback_gap_holds_but_long_gap_latches_bus_loss() {
    // ~260 ms CAN gaps were observed on the rig during 24 V sags; motors keep
    // holding through them, so only a gap beyond BUS_LOSS_TIMEOUT latches.
    let (supervisor, h) = setup(full_caps(), None).await;
    h.lock().unwrap().age = Some(Duration::from_millis(300));
    let obs = supervisor.tick(0.02).await;
    assert!(!obs.safety_state.is_stopped(), "300 ms gap must not fault: {:?}", obs.safety_state);
    h.lock().unwrap().age = Some(Duration::ZERO);
    assert!(!supervisor.tick(0.02).await.safety_state.is_stopped(), "recovery after a short gap must not fault");
    h.lock().unwrap().age = Some(Duration::from_millis(600));
    let obs = supervisor.tick(0.02).await;
    assert!(matches!(obs.safety_state, SafetyState::Fault(FaultReason::BusLoss)), "{:?}", obs.safety_state);
}

#[tokio::test]
async fn non_thermal_fault_not_overwritten_by_estop_tick_latch() {
    let (supervisor, h) = setup(full_caps(), None).await;
    // Set feedback age stale to latch BusLoss (> 500ms)
    h.lock().unwrap().age = Some(Duration::from_millis(600));
    let obs = supervisor.tick(0.02).await;
    assert!(matches!(obs.safety_state, SafetyState::Fault(FaultReason::BusLoss)), "{:?}", obs.safety_state);
    let fault_epoch = obs.epoch_id;

    // Set e_stop_latched while in BusLoss (e.g. hardware latch)
    supervisor.e_stop_latched.store(true, std::sync::atomic::Ordering::SeqCst);

    // Subsequent tick must NOT overwrite Fault(BusLoss) with EStop or change epoch
    let next_obs = supervisor.tick(0.02).await;
    assert!(matches!(next_obs.safety_state, SafetyState::Fault(FaultReason::BusLoss)), "{:?}", next_obs.safety_state);
    assert_eq!(next_obs.epoch_id, fault_epoch);
}

#[tokio::test]
async fn estop_during_thermal_trip_requires_two_stage_clear() {
    let (supervisor, h) = setup(full_caps(), None).await;
    // Overheat to trigger ThermalTrip
    {
        let mut h = h.lock().unwrap();
        h.states[0].temperature = 80.0;
    }
    let obs = supervisor.tick(0.02).await;
    assert!(matches!(obs.safety_state, SafetyState::Fault(FaultReason::ThermalTrip { .. })));

    // E-stop occurs while in ThermalTrip; consume_pending_estop preserves ThermalTrip precedence
    supervisor.trigger_e_stop().await;
    assert!(matches!(supervisor.get_observation().await.safety_state, SafetyState::Fault(FaultReason::ThermalTrip { .. })));

    // Cool down below reset threshold
    h.lock().unwrap().states[0].temperature = 60.0;

    // First clear_fault clears ThermalTrip but latches EStop because e_stop_latched was true
    let err1 = supervisor.clear_fault().await.unwrap_err();
    assert!(err1.to_string().contains("E-stop still latched"), "Expected E-stop still latched rejection, got: {err1}");
    assert_eq!(supervisor.get_observation().await.safety_state, SafetyState::EStop);

    // Second clear_fault clears EStop to Idle
    let state2 = supervisor.clear_fault().await.unwrap();
    assert_eq!(state2, SafetyState::Idle);
    assert_eq!(supervisor.get_observation().await.safety_state, SafetyState::Idle);
}

#[tokio::test]
async fn test_shifted_gripper_calibration_exceeding_joint_limits_rejected() {
    let config = ArmConfig::rebot_dm();
    let j7_idx = config.gripper_index().unwrap();
    let j7_limit = config.limits[j7_idx].clone();

    let supervisor = Arc::new(ArmSupervisor::new(config));
    let mut inner = supervisor.inner().write().await;
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("gripper_calib.json");
    inner.gripper_calib_path = Some(path.clone());

    let initial_policy = inner.config.gripper_policy.clone();
    let initial_offset = inner.gripper_calib_offset;

    // Jaw opening points within limits, but offset_rad = 0.3 will push max > limit.max_position
    let body = serde_json::json!({
        "jaw_opening": {
            "points": [[j7_limit.max_position - 0.05, 0.0], [j7_limit.min_position + 0.05, 0.08]],
            "calibrated": true
        },
        "offset_rad": 0.3
    });

    let res = armd::gripper_calib::update(&mut inner, &body);
    assert!(res.is_err(), "Shifted calibration exceeding limits must return Err");
    match res.unwrap_err() {
        armd::ArmError::Motion(armd::motion::MotionError::InvalidCommand(msg)) => {
            assert!(msg.contains("limits") || msg.contains("exceeds") || msg.contains("Invalid"));
        }
        other => panic!("Expected MotionError::InvalidCommand, got {:?}", other),
    }

    // Prior policy and offset must remain untouched
    assert_eq!(inner.config.gripper_policy, initial_policy, "gripper_policy must remain untouched");
    assert_eq!(inner.gripper_calib_offset, initial_offset, "gripper_calib_offset must remain untouched");
    assert!(!path.exists(), "No calib file must be written to disk on rejection");

    // Direct apply_gripper_calibration test
    let err = inner.apply_gripper_calibration(Some(0.4), None).unwrap_err();
    assert!(err.contains("limits") || err.contains("exceeds"));
    assert_eq!(inner.config.gripper_policy, initial_policy);
    assert_eq!(inner.gripper_calib_offset, initial_offset);
}

#[tokio::test]
async fn test_zero_check_store_offset_limit_rejection_marks_action_failed() {
    let config = ArmConfig::rebot_dm();
    let supervisor = Arc::new(ArmSupervisor::new(config));
    let mut inner = supervisor.inner().write().await;
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("gripper_calib.json");
    inner.gripper_calib_path = Some(path.clone());

    let initial_policy = inner.config.gripper_policy.clone();
    let initial_offset = inner.gripper_calib_offset;
    assert!(!inner.gripper_calib_invalid);

    // Start a dummy zero_check action
    let action_id = inner.begin_action("gripper_zero_check", None);
    // Set parameters: threshold_rad=0.5, store_offset=true, base_closed_rad=0.0
    // Measured offset is 0.4 rad, which when applied shifts jaw outside dm limits
    inner.zero_check_params = Some(armd::supervisor::ZeroCheckParams {
        action_id,
        store_offset: true,
        threshold_rad: 0.5,
        base_closed_rad: 0.0,
    });
    // Measured position 0.4 rad (offset = 0.4 rad <= threshold 0.5 rad, so mechanical check passes)
    inner.measured_positions[6] = 0.4;

    // Call finish_action for this zero check action
    inner.finish_action(action_id, "done", None, None, Some(true), None, None);

    // Verify action state was flipped to "failed" with code INVALID_CALIBRATION
    let action = inner.last_action.as_ref().expect("last_action must exist");
    assert_eq!(action.id, action_id);
    assert_eq!(action.state, "failed");
    assert_eq!(action.code.as_deref(), Some("INVALID_CALIBRATION"));
    assert!(action.message.as_ref().unwrap().contains("Gripper calibration rejected"));

    // Verify atomic safety invariants: gripper_calib_invalid stayed false, prior calibration intact, no disk file written
    assert!(!inner.gripper_calib_invalid, "Atomic failure must not invalidate existing calibration");
    assert_eq!(inner.config.gripper_policy, initial_policy);
    assert_eq!(inner.gripper_calib_offset, initial_offset);
    assert!(!path.exists(), "No calibration file must be written to disk on limit rejection");
}

#[tokio::test]
async fn test_estop_during_bus_loss_preserves_bus_loss_and_requires_two_stage_clear() {
    let (supervisor, h) = setup(full_caps(), None).await;
    // Latch BusLoss via stale feedback (> 500ms)
    h.lock().unwrap().age = Some(Duration::from_millis(600));
    let obs = supervisor.tick(0.02).await;
    assert!(matches!(obs.safety_state, SafetyState::Fault(FaultReason::BusLoss)));
    let bus_loss_epoch = obs.epoch_id;

    // Concurrently trigger E-Stop while in BusLoss
    supervisor.trigger_e_stop().await;

    // Active diagnostic state must remain Fault(BusLoss), epoch must not churn
    let obs_after_estop = supervisor.get_observation().await;
    assert!(matches!(obs_after_estop.safety_state, SafetyState::Fault(FaultReason::BusLoss)));
    assert_eq!(obs_after_estop.epoch_id, bus_loss_epoch);

    // Restore bus feedback freshness
    h.lock().unwrap().age = Some(Duration::from_millis(10));

    // First clear_fault clears BusLoss, re-latches EStop and returns ClearFaultRejected
    let err1 = supervisor.clear_fault().await.unwrap_err();
    assert!(err1.to_string().contains("E-stop still latched"), "Expected E-stop still latched error, got: {err1}");
    assert_eq!(supervisor.get_observation().await.safety_state, SafetyState::EStop);

    // Second clear_fault clears EStop to Idle
    let state2 = supervisor.clear_fault().await.unwrap();
    assert_eq!(state2, SafetyState::Idle);
    assert_eq!(supervisor.get_observation().await.safety_state, SafetyState::Idle);
}
