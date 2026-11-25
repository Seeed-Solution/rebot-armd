use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
use armd::model::{ArmConfig, GravityModel, JointLimit, JointState, RestingPosePolicy};
use armd::{ArmSupervisor, SafetyState, SupervisorInner};
use armd::safety::{FaultCode, FaultReason};

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
    fn has_driver_fresh_position_api(&self) -> bool { true }
    fn fresh_joint_positions_into(&self, joints: &[usize], max_age: Duration, out: &mut [f32]) -> bool {
        let h = self.hardware.lock().unwrap();
        if joints.is_empty() || out.len() < joints.len() || !h.age.is_some_and(|a| a <= max_age) {
            return false;
        }
        for (slot, &i) in out.iter_mut().zip(joints.iter()) {
            let Some(st) = h.states.get(i) else { return false; };
            if !st.position.is_finite() { return false; }
            *slot = st.position;
        }
        true
    }
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
    supervisor.enable_torque(supervisor.capture_admission_stamp()).await.unwrap();
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
    assert!(supervisor.enable_torque(supervisor.capture_admission_stamp()).await.is_err());
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
    let state = supervisor.clear_fault().await.unwrap();
    assert_eq!(state, SafetyState::EStop);
    assert_eq!(supervisor.get_observation().await.safety_state, SafetyState::EStop);
    let state = supervisor.clear_fault().await.unwrap();
    assert_eq!(state, SafetyState::Idle);
    assert_eq!(supervisor.get_observation().await.safety_state, SafetyState::Idle);
}

#[tokio::test]
async fn successful_disable_write_without_feedback_is_not_reported_as_power_off() {
    // Part 1: Accepted disable completes by decision (`torque_intent = Off`, `!is_torqued`,
    // `!disable_pending`, `shutdown_ready()`) even when enable feedback still reports Enabled.
    let (supervisor, h) = setup(full_caps(), None).await;
    h.lock().unwrap().confirm_disable = false;
    assert_eq!(supervisor.disable_torque().await.unwrap(), SafetyState::Idle);
    let obs = supervisor.get_observation().await;
    assert!(!obs.is_torqued && !obs.disable_pending && obs.safety_state == SafetyState::Idle);
    assert_eq!(obs.enable_state, EnableState::Enabled);
    assert!(supervisor.shutdown_ready().await);

    // Part 2: A failed disable write keeps `disable_pending == true && is_torqued == true && !shutdown_ready()`,
    // inhibits ordinary motion frames, and continues stationary measured holds until a later disable write succeeds.
    let (supervisor, h) = setup(full_caps(), None).await;
    {
        let mut h = h.lock().unwrap();
        h.confirm_disable = false;
        h.disable_fails = true;
    }
    supervisor.inner().write().await.commanded_positions[0] = 0.5;
    assert!(supervisor.disable_torque().await.is_err());
    let obs = supervisor.get_observation().await;
    assert!(obs.is_torqued && obs.disable_pending && obs.safety_state.is_stopped());
    assert!(!supervisor.shutdown_ready().await);
    supervisor.tick(0.02).await;
    {
        let written = &h.lock().unwrap().written;
        assert!(!written.is_empty(), "Pending unaccepted disable must keep holding at measured position");
        assert!(
            written.iter().all(|t| t[0].position == 0.0 && t[0].velocity == Some(0.0)),
            "Pending disable must emit stationary measured holds only, never ordinary motion targets: {written:?}"
        );
    }
    supervisor.inner().write().await.last_disable_attempt = Some(Instant::now() - Duration::from_millis(251));
    h.lock().unwrap().disable_fails = false;
    let obs = supervisor.tick(0.02).await;
    assert!(!obs.is_torqued && !obs.disable_pending);
    assert!(supervisor.shutdown_ready().await);
    h.lock().unwrap().written.clear();
    supervisor.tick(0.02).await;
    assert!(h.lock().unwrap().written.is_empty(), "Accepted disable must stop holds");
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
    // Missing fresh feedback (`age = None`) blocks clear_fault().
    { let mut h = h.lock().unwrap(); h.enabled = EnableState::Enabled; h.age = None; }
    assert!(supervisor.clear_fault().await.is_err());
    // `EnableState::Unknown` is telemetry only and does NOT block clear_fault() once feedback is fresh.
    { let mut h = h.lock().unwrap(); h.enabled = EnableState::Unknown; h.age = Some(Duration::ZERO); h.states[0].position = 0.4; }
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
    supervisor.enable_torque(supervisor.capture_admission_stamp()).await.unwrap();
    supervisor.jog(supervisor.capture_admission_stamp(), vec![0.3; 6], Some(1.0), None).await.unwrap();
    let obs = supervisor.tick(0.300).await;
    assert!(obs.safety_state.is_stopped());
    assert_eq!(obs.positions, vec![0.0; 6]);
    assert!(supervisor.inner().read().await.active_trajectory.is_none());
}

#[tokio::test]
async fn single_late_tick_below_stall_bound_keeps_arm_operational() {
    // One 100 ms scheduling hiccup on a loaded single-core host must not fault the arm.
    let supervisor = ArmSupervisor::new(ArmConfig::default());
    supervisor.enable_torque(supervisor.capture_admission_stamp()).await.unwrap();
    supervisor.jog(supervisor.capture_admission_stamp(), vec![0.3; 6], Some(1.0), None).await.unwrap();
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
    supervisor.enable_torque(supervisor.capture_admission_stamp()).await.unwrap();
    supervisor.inner().write().await.motion_frame_timeout = Duration::from_secs(60);
    supervisor.servo(supervisor.capture_admission_stamp(), vec![0.1; 6], 1).await.unwrap();
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
    supervisor.enable_torque(supervisor.capture_admission_stamp()).await.unwrap();
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

    // Subsequent tick must NOT overwrite Fault(BusLoss) with EStop, MUST advance epoch for the new E-stop latch,
    // and steady-state ticks after that must not churn epoch.
    let next_obs = supervisor.tick(0.02).await;
    assert!(matches!(next_obs.safety_state, SafetyState::Fault(FaultReason::BusLoss)), "{:?}", next_obs.safety_state);
    assert_eq!(next_obs.epoch_id, fault_epoch + 1);
    let steady_obs = supervisor.tick(0.02).await;
    assert_eq!(steady_obs.epoch_id, next_obs.epoch_id);
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

    // First clear_fault clears ThermalTrip and transitions to EStop because e_stop_latched was true
    let state1 = supervisor.clear_fault().await.unwrap();
    assert_eq!(state1, SafetyState::EStop);
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
    let persist_ctx = Some(Box::new(armd::supervisor::ZeroCheckPersistContext::from_inner(&inner, None)));
    inner.zero_check_params = Some(armd::supervisor::ZeroCheckParams {
        action_id,
        store_offset: true,
        threshold_rad: 0.5,
        base_closed_rad: 0.0,
        persist_ctx,
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

    // Active diagnostic state must remain Fault(BusLoss), and latching E-Stop MUST advance epoch.
    let obs_after_estop = supervisor.get_observation().await;
    assert!(matches!(obs_after_estop.safety_state, SafetyState::Fault(FaultReason::BusLoss)));
    assert_eq!(obs_after_estop.epoch_id, bus_loss_epoch + 1);

    // Restore bus feedback freshness
    h.lock().unwrap().age = Some(Duration::from_millis(10));

    // First clear_fault clears BusLoss, transitions to EStop
    let state1 = supervisor.clear_fault().await.unwrap();
    assert_eq!(state1, SafetyState::EStop);
    assert_eq!(supervisor.get_observation().await.safety_state, SafetyState::EStop);

    // Second clear_fault clears EStop to Idle
    let state2 = supervisor.clear_fault().await.unwrap();
    assert_eq!(state2, SafetyState::Idle);
    assert_eq!(supervisor.get_observation().await.safety_state, SafetyState::Idle);
}

struct DynamicLimitsDriver {
    limits: Arc<Mutex<Vec<Option<(f32, f32)>>>>,
}

impl ArmDriver for DynamicLimitsDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            velocity_feedback: true,
            torque_feedback: false,
            temperature_feedback: false,
            torque_feedforward: false,
            enable_feedback: true,
            load_feedback: false,
        }
    }
    fn enable_state(&self) -> EnableState {
        EnableState::Disabled
    }
    fn is_feedback_valid(&self) -> bool {
        true
    }
    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        vec![Some(Duration::ZERO)]
    }
    fn position_limits(&self, joint: usize) -> Option<(f32, f32)> {
        self.limits.lock().unwrap().get(joint).copied().flatten()
    }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        Ok(vec![JointState::default()])
    }
    fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        Ok(())
    }
    fn set_torque(&mut self, _on: bool) -> Result<(), ArmDriverError> {
        Ok(())
    }
}

#[tokio::test]
async fn test_apply_driver_limits_gripper_narrowing_invalidates_calib_and_falls_back() {
    let config = ArmConfig::rebot_dm();
    let j7_idx = config.gripper_index().unwrap();
    let base_policy = config.gripper_policy.clone().unwrap();

    let supervisor = Arc::new(ArmSupervisor::new(config));
    let mut inner = supervisor.inner().write().await;

    // Apply initial calibrated offset within DM limits [-5.75, 0.10]:
    // base closed=0.0, open=-5.7. With offset 0.05 -> closed=0.05, open=-5.65.
    inner.apply_gripper_calibration(Some(0.05), None).unwrap();
    assert_eq!(inner.gripper_calib_offset, Some(0.05));
    assert!(!inner.gripper_calib_invalid);
    assert_ne!(inner.config.gripper_policy, Some(base_policy.clone()));

    // Driver narrows gripper joint limit to [-5.75, 0.02]:
    // Calibrated policy (closed=0.05 > 0.02) fails candidate.validate().
    // Base policy (closed=0.0 <= 0.02) passes candidate.validate().
    let mut pos_limits: Vec<Option<(f32, f32)>> = vec![None; 7];
    pos_limits[j7_idx] = Some((-5.75, 0.02));
    let vel_limits = vec![None; 7];

    inner.apply_extracted_driver_limits(&vel_limits, &pos_limits);

    // Invariants:
    // Outdated calibration must be invalidated and cleared
    assert!(inner.gripper_calib_invalid, "gripper_calib_invalid must be true");
    assert_eq!(inner.gripper_calib_offset, None, "gripper_calib_offset must be None");
    assert_eq!(inner.gripper_calib_jaw_opening, None, "gripper_calib_jaw_opening must be None");
    // Falls back cleanly to base_gripper_policy
    assert_eq!(inner.config.gripper_policy, Some(base_policy), "Must fall back to base_gripper_policy");
    // driver_config_error must be None (without faulting)
    assert!(inner.driver_config_error.is_none(), "driver_config_error must be None");
    assert!(!inner.safety.state().is_stopped(), "Must not fault on clean fallback");
    assert!(inner.safety.fault().is_none(), "Must not have fault info on clean fallback");
    assert_eq!(inner.config.limits[j7_idx].max_position, 0.02, "Gripper max_position must be narrowed");
}

#[tokio::test]
async fn test_apply_driver_limits_home_positions_conflict_fails_closed_and_recovers() {
    let mut config = ArmConfig::default();
    config.dof = 1;
    config.limits = vec![JointLimit::new("axis", -2.0, 2.0, 2.0, 5.0)];
    config.home_positions = vec![0.0];

    let limits_holder = Arc::new(Mutex::new(vec![Some((0.5, 1.5))]));
    let driver = Box::new(DynamicLimitsDriver {
        limits: limits_holder.clone(),
    });

    let supervisor = Arc::new(ArmSupervisor::new(config).with_driver(driver));

    // Initially, driver limit [0.5, 1.5] conflicts with home_positions[0] = 0.0.
    {
        let inner = supervisor.inner().read().await;
        // 1. driver_config_error must be Some
        assert!(inner.driver_config_error.is_some(), "driver_config_error must be Some on conflict with home_positions");
        let err_msg = inner.driver_config_error.as_ref().unwrap();
        assert!(err_msg.contains("conflict") || err_msg.contains("violated"));

        // 2. FaultCode::JointLimit must be latched
        assert_eq!(inner.safety.fault().map(|f| f.code), Some(FaultCode::JointLimit));

        // 3. Hardware limit NOT dropped: config was NOT updated to silently discard EEPROM limits
        assert_eq!(inner.config.limits[0].min_position, -2.0, "Original limits must not be corrupted or partially applied");
        assert_eq!(inner.config.limits[0].max_position, 2.0);
    }

    // 4. clear_fault() must be rejected while driver limit conflict persists
    let clear_err = supervisor.clear_fault().await.unwrap_err();
    assert!(
        clear_err.to_string().contains("Driver limit configuration error"),
        "clear_fault must be rejected, got: {clear_err}"
    );

    // 5. Resolve the driver limits conflict to include home_positions [0.0]
    *limits_holder.lock().unwrap() = vec![Some((-1.0, 1.0))];

    // 6. Recovery via reapply_driver_limits()
    {
        let mut inner = supervisor.inner().write().await;
        let recovered = inner.reapply_driver_limits();
        assert!(recovered, "reapply_driver_limits must return true once limits resolved");
        assert!(inner.driver_config_error.is_none(), "driver_config_error must be None after recovery");
        assert_eq!(inner.config.limits[0].min_position, -1.0, "Resolved narrowed limits must now be applied");
        assert_eq!(inner.config.limits[0].max_position, 1.0);
    }

    // 7. Fault can now be cleared successfully
    let state = supervisor.clear_fault().await.unwrap();
    assert_eq!(state, SafetyState::Idle);
    let obs = supervisor.get_observation().await;
    assert_eq!(obs.safety_state, SafetyState::Idle);
    assert!(obs.fault.is_none(), "Fault must be cleared");
}

#[tokio::test]
async fn test_thermal_trip_overriding_joint_limit_not_overwritten_by_reapply_limits_on_clear_fault() {
    let config = ArmConfig::rebot_dm();
    let supervisor = Arc::new(ArmSupervisor::new(config));

    // DynamicLimitsDriver with conflicting joint limits for joint 0
    let limits = Arc::new(Mutex::new(vec![Some((10.0, 11.0))])); // conflicts with DM limits
    let driver = DynamicLimitsDriver { limits };
    supervisor.set_driver(Box::new(driver)).await;

    // Check that driver_config_error is set and safety state is Fault(JointLimit)
    let obs = supervisor.get_observation().await;
    assert!(obs.fault.is_some());
    assert_eq!(obs.fault.as_ref().unwrap().code, FaultCode::JointLimit);

    // Now trigger ThermalTrip
    {
        let mut inner = supervisor.inner().write().await;
        inner.temperatures[0] = 85.0;
        inner.safety.trigger_fault_coded(
            FaultCode::ThermalTrip,
            FaultReason::ThermalTrip { joint_idx: 0, temperature: 85.0, limit: 80.0 },
        );
    }
    let obs_thermal = supervisor.get_observation().await;
    assert_eq!(obs_thermal.fault.as_ref().unwrap().code, FaultCode::ThermalTrip);
    let thermal_epoch = obs_thermal.epoch_id;

    // Call clear_fault(): driver limits still conflict (10.0..11.0).
    // clear_fault() calls reapply_driver_limits(), but trigger_fault=false must NOT
    // trigger JointLimit fault or churn epoch_id!
    let res = supervisor.clear_fault().await;
    assert!(res.is_err());
    let err = res.unwrap_err();
    assert!(err.to_string().contains("Driver limit configuration error"));

    let obs_after = supervisor.get_observation().await;
    assert_eq!(
        obs_after.fault.as_ref().unwrap().code,
        FaultCode::ThermalTrip,
        "ThermalTrip must NOT be overwritten to JointLimit"
    );
    assert_eq!(
        obs_after.epoch_id,
        thermal_epoch,
        "epoch_id must NOT churn when reapply_driver_limits finds conflicting limits"
    );
}

#[tokio::test]
async fn test_gripper_calib_update_hardware_driver_limits_violation_rollback() {
    let config = ArmConfig::rebot_dm();
    let j7_idx = config.gripper_index().unwrap();

    let supervisor = Arc::new(ArmSupervisor::new(config));
    let mut pos_limits: Vec<Option<(f32, f32)>> = vec![None; 7];
    // Hardware driver limits narrow J7 from [-5.75, 0.10] to [-5.75, 0.02]
    pos_limits[j7_idx] = Some((-5.75, 0.02));
    let limits = Arc::new(Mutex::new(pos_limits));
    let driver = DynamicLimitsDriver { limits };
    supervisor.set_driver(Box::new(driver)).await;

    let mut inner = supervisor.inner().write().await;
    let initial_config = inner.config.clone();
    let initial_offset = inner.gripper_calib_offset;
    let initial_jaw = inner.gripper_calib_jaw_opening.clone();
    let initial_invalid = inner.gripper_calib_invalid;
    let initial_err = inner.driver_config_error.clone();

    // Body with offset_rad = 0.05 (passes profile limit 0.10, but violates hardware driver limit 0.02)
    let body = serde_json::json!({
        "jaw_opening": {
            "points": [[0.0, 0.0], [-5.0, 0.08]],
            "calibrated": true
        },
        "offset_rad": 0.05
    });

    let res = armd::gripper_calib::update(&mut inner, &body);
    assert!(res.is_err(), "gripper_calib::update must fail when hardware driver limits are violated");

    // Verify complete rollback
    assert_eq!(inner.config.gripper_policy, initial_config.gripper_policy, "gripper_policy must be fully restored on error");
    assert_eq!(inner.config.limits, initial_config.limits, "limits must be fully restored on error");
    assert_eq!(inner.gripper_calib_offset, initial_offset, "offset must be restored");
    assert_eq!(inner.gripper_calib_jaw_opening, initial_jaw, "jaw_opening must be restored");
    assert_eq!(inner.gripper_calib_invalid, initial_invalid, "invalid flag must be restored");
    assert_eq!(inner.driver_config_error, initial_err, "driver_config_error must be restored");
}
