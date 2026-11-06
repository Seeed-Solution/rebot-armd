use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::time::{Duration, Instant};

use armd::driver::{ArmDriver, ArmDriverError, JointTarget};
use armd::model::{ArmConfig, JointLimit, JointState};
use armd::safety::{FaultReason, SafetyError, SafetyState, StopPolicy};
use armd::supervisor::{ArmError, ArmSupervisor};

struct DummyDriver {
    pub states: Vec<JointState>,
    pub written_targets: Arc<Mutex<Vec<JointTarget>>>,
    pub write_count: Arc<AtomicUsize>,
    pub is_enabled: bool,
    pub feedback_valid: Arc<AtomicBool>,
    pub clear_fault_fail: Arc<AtomicBool>,
}

impl DummyDriver {
    fn new(states: Vec<JointState>) -> Self {
        Self {
            states,
            written_targets: Arc::new(Mutex::new(Vec::new())),
            write_count: Arc::new(AtomicUsize::new(0)),
            is_enabled: true,
            feedback_valid: Arc::new(AtomicBool::new(true)),
            clear_fault_fail: Arc::new(AtomicBool::new(false)),
        }
    }
}

impl ArmDriver for DummyDriver {
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        Ok(self.states.clone())
    }

    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.write_count.fetch_add(1, Ordering::SeqCst);
        let mut guard = self.written_targets.lock().unwrap();
        *guard = targets.to_vec();
        Ok(())
    }

    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        self.is_enabled = enabled;
        Ok(())
    }

    fn clear_fault(&mut self) -> Result<(), ArmDriverError> {
        if self.clear_fault_fail.load(Ordering::SeqCst) {
            Err(ArmDriverError::HardwareFault("Hardware clear fault failed: register locked".to_string()))
        } else {
            Ok(())
        }
    }

    fn is_feedback_valid(&self) -> bool {
        self.feedback_valid.load(Ordering::SeqCst)
    }

    fn is_enabled(&self) -> bool {
        self.is_enabled
    }
}

#[tokio::test]
async fn test_safety_error_enum_variants_and_traits() {
    let err1 = SafetyError::ArmNotAtRestingPose {
        current: vec![0.1, 0.2],
        expected: vec![0.0, 0.0],
    };
    let err2 = SafetyError::MotionForbiddenInFaultState("test fault".to_string());

    // Debug, Clone, PartialEq
    assert_eq!(err1.clone(), err1);
    assert_eq!(err2.clone(), err2);
    assert_ne!(err1, err2);

    // Display / thiserror formatting
    let msg1 = err1.to_string();
    assert!(msg1.contains("Arm is not at resting pose"));
    assert!(msg1.contains("0.1"));

    let msg2 = err2.to_string();
    assert!(msg2.contains("Motion forbidden in fault/estop state"));
    assert!(msg2.contains("test fault"));

    // Serde JSON roundtrip
    let json1 = serde_json::to_string(&err1).unwrap();
    let de1: SafetyError = serde_json::from_str(&json1).unwrap();
    assert_eq!(err1, de1);

    let json2 = serde_json::to_string(&err2).unwrap();
    let de2: SafetyError = serde_json::from_str(&json2).unwrap();
    assert_eq!(err2, de2);
}

#[tokio::test]
async fn test_is_at_safe_resting_pose_criteria() {
    let mut config = ArmConfig::default();
    config.dof = 3;
    config.home_positions = vec![0.0, 0.0, 0.0];
    config.limits = vec![
        JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0),
        JointLimit::new("j2", -3.0, 3.0, 2.0, 5.0),
        JointLimit::new("j3", -3.0, 3.0, 2.0, 5.0),
    ];

    let supervisor = ArmSupervisor::new(config.clone());
    supervisor.set_torque(true).await.unwrap();

    // 1. Initial pose at [0.0, 0.0, 0.0] with v=0: must be at safe resting pose
    assert!(supervisor.is_at_safe_resting_pose().await);

    // 2. Geometry check: each joint deviation <= 0.08, sum <= 0.16
    // Set positions to [0.06, 0.05, 0.04]: each <= 0.08, sum = 0.15 <= 0.16 -> OK
    {
        let mut inner = supervisor.inner().write().await;
        inner.measured_positions = vec![0.06, 0.05, 0.04];
        inner.measured_velocities = vec![0.0, 0.0, 0.0];
    }
    assert!(supervisor.is_at_safe_resting_pose().await);

    // Set one joint to 0.085 (> 0.08) -> must fail geometry check
    {
        let mut inner = supervisor.inner().write().await;
        inner.measured_positions = vec![0.085, 0.0, 0.0];
    }
    assert!(!supervisor.is_at_safe_resting_pose().await);

    // Set positions to [0.06, 0.06, 0.05]: each <= 0.08, but sum = 0.17 > 0.16 -> must fail
    {
        let mut inner = supervisor.inner().write().await;
        inner.measured_positions = vec![0.06, 0.06, 0.05];
    }
    assert!(!supervisor.is_at_safe_resting_pose().await);

    // 3. Velocity check: all velocities <= 0.015 rad/s
    {
        let mut inner = supervisor.inner().write().await;
        inner.measured_positions = vec![0.0, 0.0, 0.0];
        inner.measured_velocities = vec![0.012, -0.010, 0.005];
    }
    assert!(supervisor.is_at_safe_resting_pose().await);

    // Any joint velocity > 0.015 -> must fail
    {
        let mut inner = supervisor.inner().write().await;
        inner.measured_velocities = vec![0.018, 0.0, 0.0];
    }
    assert!(!supervisor.is_at_safe_resting_pose().await);

    // 4. Torque check with physical driver: J2 (idx 1) <= 1.5 N·m, J3 (idx 2) <= 4.5 N·m
    let dummy = DummyDriver::new(vec![
        JointState { position: 0.0, velocity: 0.0, torque: 0.2, temperature: 25.0 },
        JointState { position: 0.0, velocity: 0.0, torque: 0.8, temperature: 25.0 },
        JointState { position: 0.0, velocity: 0.0, torque: 3.2, temperature: 25.0 },
    ]);
    let supervisor_hw = ArmSupervisor::new(config.clone());
    supervisor_hw.set_driver(Box::new(dummy)).await;
    supervisor_hw.set_torque(true).await.unwrap();

    // J2 torque = 0.8 <= 1.5, J3 torque = 3.2 <= 4.5 -> OK
    assert!(supervisor_hw.is_at_safe_resting_pose().await);

    // If J2 torque exceeds 1.5 N·m (e.g. 1.8 N·m) -> fails
    {
        let mut inner = supervisor_hw.inner().write().await;
        inner.torques = vec![0.2, 1.8, 0.5];
    }
    assert!(!supervisor_hw.is_at_safe_resting_pose().await);

    // If J3 torque exceeds 4.5 N·m (e.g. -5.0 N·m) -> fails
    {
        let mut inner = supervisor_hw.inner().write().await;
        inner.torques = vec![0.2, 0.5, -5.0];
    }
    assert!(!supervisor_hw.is_at_safe_resting_pose().await);

    // Even if gravity compensation is active and model predicts large gravity torques,
    // measured torque within limits must pass without subtracting gravity torque (verifying bug fix)
    {
        let mut inner = supervisor_hw.inner().write().await;
        inner.torques = vec![0.1, 0.4, -3.2];
        inner.gravity_compensated = true;
        inner.gravity_torques = vec![0.0, -7.5, -3.2];
    }
    assert!(supervisor_hw.is_at_safe_resting_pose().await);

    // Missing torque data when hardware driver mounted must fail-safe (not pass)
    {
        let mut inner = supervisor_hw.inner().write().await;
        inner.torques = vec![];
    }
    assert!(!supervisor_hw.is_at_safe_resting_pose().await, "Missing torque data must fail safe");

    // 5. 7-DOF reBot-DM configuration & J7 gripper exemption
    let rebot_cfg = ArmConfig::rebot_dm();
    assert_eq!(rebot_cfg.home_positions, vec![0.0, 0.0, 0.0, 0.0, 0.0, 1.5707963, 0.0]);
    let supervisor_rebot = ArmSupervisor::new(rebot_cfg);
    supervisor_rebot.set_torque(true).await.unwrap();
    {
        let mut inner = supervisor_rebot.inner().write().await;
        // Exact hardware readings from docs: [0.049, 0.011, 0.027, 0.024, 0.001, 1.574, 0.052]
        inner.measured_positions = vec![0.049, 0.011, 0.027, 0.024, 0.001, 1.574, 0.052];
        inner.measured_velocities = vec![0.003; 7];
        inner.torques = vec![0.02; 7];
    }
    assert!(supervisor_rebot.is_at_safe_resting_pose().await, "Real robot resting readings must pass");

    // Gripper J7 deviation (e.g. 0.15 rad vs 0.0 home) must NOT block safe resting pose
    {
        let mut inner = supervisor_rebot.inner().write().await;
        inner.measured_positions[6] = 0.15;
    }
    assert!(supervisor_rebot.is_at_safe_resting_pose().await, "J7 gripper deviation must not block resting pose");
}

#[tokio::test]
async fn test_set_torque_false_safety_gates() {
    let mut config = ArmConfig::default();
    config.dof = 2;
    config.home_positions = vec![0.0, 0.0];
    config.limits = vec![
        JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0),
        JointLimit::new("j2", -3.0, 3.0, 2.0, 5.0),
    ];

    let supervisor = ArmSupervisor::new(config.clone());
    supervisor.set_torque(true).await.unwrap();

    // 1. In mid-air (not at safe resting pose): set_torque(false) must be rejected
    {
        let mut inner = supervisor.inner().write().await;
        inner.measured_positions = vec![0.5, -0.3];
    }
    let res_midair = supervisor.set_torque(false).await;
    assert!(matches!(
        res_midair,
        Err(ArmError::Safety(SafetyError::ArmNotAtRestingPose { ref current, ref expected }))
            if current == &vec![0.5, -0.3] && expected == &vec![0.0, 0.0]
    ));
    assert!(supervisor.get_observation().await.is_torqued, "Torque must NOT be cut in mid-air");

    // Return to resting pose
    {
        let mut inner = supervisor.inner().write().await;
        inner.measured_positions = vec![0.0, 0.0];
        inner.measured_velocities = vec![0.0, 0.0];
    }

    // 2. In E-Stop: set_torque(false) must be rejected with MotionForbiddenInFaultState
    supervisor.trigger_e_stop().await;
    let res_estop = supervisor.set_torque(false).await;
    assert!(matches!(
        res_estop,
        Err(ArmError::Safety(SafetyError::MotionForbiddenInFaultState(_)))
    ));

    // Clear fault
    supervisor.clear_fault().await.unwrap();

    // 3. In Fault (e.g. BusLoss): set_torque(false) must be rejected with MotionForbiddenInFaultState
    supervisor.set_torque(true).await.unwrap();
    supervisor.apply_stop_policy(StopPolicy::BusLossFallback).await;
    assert!(matches!(supervisor.get_observation().await.safety_state, SafetyState::Fault(_)));
    let res_fault = supervisor.set_torque(false).await;
    assert!(matches!(
        res_fault,
        Err(ArmError::Safety(SafetyError::MotionForbiddenInFaultState(_)))
    ));

    // Clear fault, arm at resting pose: now set_torque(false) succeeds
    supervisor.clear_fault().await.unwrap();
    let res_ok = supervisor.set_torque(false).await;
    assert!(res_ok.is_ok());
    assert!(!supervisor.get_observation().await.is_torqued);
}

#[tokio::test]
async fn test_normal_unload_forbidden_in_estop_or_fault() {
    let mut config = ArmConfig::default();
    config.dof = 1;
    config.home_positions = vec![0.0];
    config.limits = vec![JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0)];

    let supervisor = ArmSupervisor::new(config);
    supervisor.set_torque(true).await.unwrap();

    // 1. In EStop: apply_stop_policy(NormalUnload) must NOT plan motion or unload
    supervisor.trigger_e_stop().await;
    supervisor.apply_stop_policy(StopPolicy::NormalUnload).await;
    assert!(!supervisor.is_unloading().await, "Must not unload in EStop");
    assert!(supervisor.get_observation().await.is_torqued, "Must maintain holding torque");

    // 2. In Fault (e.g. Watchdog): must NOT unload
    supervisor.clear_fault().await.unwrap();
    supervisor.set_torque(true).await.unwrap();
    {
        let mut inner = supervisor.inner().write().await;
        inner.safety.trigger_fault(FaultReason::WatchdogTimeout);
    }
    supervisor.apply_stop_policy(StopPolicy::NormalUnload).await;
    assert!(!supervisor.is_unloading().await, "Must not unload in Fault");
    assert!(supervisor.get_observation().await.is_torqued, "Must maintain holding torque");
}

#[tokio::test]
async fn test_clear_fault_synchronizes_positions_and_zeroes_velocities() {
    let mut config = ArmConfig::default();
    config.dof = 2;
    config.home_positions = vec![0.0, 0.0];
    config.limits = vec![
        JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0),
        JointLimit::new("j2", -3.0, 3.0, 2.0, 5.0),
    ];

    let supervisor = ArmSupervisor::new(config);
    supervisor.set_torque(true).await.unwrap();

    // 1. Simulate large divergence between commanded state and physical measured state
    {
        let mut inner = supervisor.inner().write().await;
        inner.measured_positions = vec![0.75, -0.45];
        inner.commanded_positions = vec![1.20, 0.10];
        inner.target_positions = vec![1.50, 0.50];
        inner.commanded_velocities = vec![1.8, -1.2];
        inner.safety.trigger_fault(FaultReason::BusLoss);
    }

    // Call clear_fault: step must be slew-limited to clamp(-0.04, 0.04)
    supervisor.clear_fault().await.unwrap();

    let (cmd, tgt, vel) = {
        let inner = supervisor.inner().read().await;
        (
            inner.commanded_positions.clone(),
            inner.target_positions.clone(),
            inner.commanded_velocities.clone(),
        )
    };
    assert!((cmd[0] - 1.16).abs() < 1e-4, "Commanded position 0 must be clamped to -0.04 rad");
    assert!((cmd[1] - 0.06).abs() < 1e-4, "Commanded position 1 must be clamped to -0.04 rad");
    assert_eq!(tgt, cmd, "Target positions must match commanded positions");
    assert_eq!(vel, vec![0.0, 0.0], "Commanded velocities must be zeroed");

    // 2. Small divergence (within +/-0.04 rad) syncs directly to measured
    {
        let mut inner = supervisor.inner().write().await;
        inner.measured_positions = vec![1.18, 0.04];
        inner.safety.trigger_fault(FaultReason::WatchdogTimeout);
    }
    supervisor.clear_fault().await.unwrap();
    let (cmd2, meas2) = {
        let inner = supervisor.inner().read().await;
        (inner.commanded_positions.clone(), inner.measured_positions.clone())
    };
    assert!((cmd2[0] - meas2[0]).abs() < 1e-4);
    assert!((cmd2[1] - meas2[1]).abs() < 1e-4);
}

#[tokio::test]
async fn test_servo_target_dirty_jump_and_step_rate_limit() {
    let mut config = ArmConfig::default();
    config.dof = 1;
    config.home_positions = vec![0.0];
    config.limits = vec![JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0)];

    let supervisor = ArmSupervisor::new(config);
    supervisor.set_torque(true).await.unwrap();

    // 1. Dirty jump > 0.35 rad: must be dropped immediately
    supervisor.servo(vec![0.40], 1).await.unwrap();
    let obs1 = supervisor.tick(0.02).await;
    assert_eq!(obs1.positions[0], 0.0, "Dirty target with jump > 0.35 rad must be dropped");

    // 2. Normal jump <= 0.35 rad (e.g. 0.10 rad): accepted
    // Acceleration-bounded first step: 20ms * (5 rad/s² * 20ms) = 0.002 rad
    supervisor.servo(vec![0.10], 1).await.unwrap();
    let obs2 = supervisor.tick(0.02).await;
    assert!((obs2.positions[0] - 0.002).abs() < 1e-4, "Position step must be acceleration bounded");
    assert!((obs2.velocities[0] - 0.1).abs() < 1e-3, "Velocity must be acceleration bounded");
}

#[tokio::test]
async fn test_bus_loss_fallback_duration_and_silence_after_3000ms() {
    let mut config = ArmConfig::default();
    config.dof = 1;
    config.home_positions = vec![0.0];
    config.limits = vec![JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0)];

    let dummy = DummyDriver::new(vec![JointState {
        position: 0.35,
        velocity: 0.0,
        torque: 0.1,
        temperature: 25.0,
    }]);
    let write_count = dummy.write_count.clone();
    let written = dummy.written_targets.clone();
    let feedback_valid = dummy.feedback_valid.clone();

    let supervisor = ArmSupervisor::new(config);
    supervisor.set_driver(Box::new(dummy)).await;
    supervisor.set_torque(true).await.unwrap();

    // Trigger BusLossFallback
    supervisor.apply_stop_policy(StopPolicy::BusLossFallback).await;
    let obs = supervisor.get_observation().await;
    assert_eq!(obs.safety_state, SafetyState::Fault(FaultReason::BusLoss));

    // In first 250ms ~ 3000ms: continues to write joint targets holding last measured position (velocity 0.0)
    let initial_writes = write_count.load(Ordering::SeqCst);
    supervisor.tick(0.02).await;
    assert!(write_count.load(Ordering::SeqCst) > initial_writes, "Must keep writing during bus loss");
    {
        let guard = written.lock().unwrap();
        assert_eq!(guard[0].position, 0.35);
        assert_eq!(guard[0].velocity, Some(0.0));
    }

    // Simulate duration > 3000ms after feedback loss
    feedback_valid.store(false, Ordering::SeqCst);
    {
        let mut inner = supervisor.inner().write().await;
        inner.last_can_feedback_instant = Some(Instant::now() - Duration::from_millis(3500));
    }
    let writes_before = write_count.load(Ordering::SeqCst);
    supervisor.tick(0.02).await;
    let writes_after = write_count.load(Ordering::SeqCst);
    assert!(
        writes_after > writes_before,
        "Holding packets must CONTINUE to be sent after 3000ms in BusLossFallback to prevent dropping"
    );
}

#[tokio::test]
async fn test_clear_fault_hardware_failure_rejected_and_latch_retained() {
    let mut config = ArmConfig::default();
    config.dof = 1;
    config.home_positions = vec![0.0];
    config.limits = vec![JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0)];

    let dummy = DummyDriver::new(vec![JointState {
        position: 0.0,
        velocity: 0.0,
        torque: 0.1,
        temperature: 25.0,
    }]);
    let clear_fault_fail = dummy.clear_fault_fail.clone();

    let supervisor = ArmSupervisor::new(config);
    supervisor.set_driver(Box::new(dummy)).await;
    supervisor.set_torque(true).await.unwrap();

    // Trigger an underlying hardware fault (e.g. driver reported error)
    {
        let mut inner = supervisor.inner().write().await;
        inner.safety.trigger_fault(FaultReason::Other("Driver motor overheat alarm".to_string()));
    }
    assert!(matches!(
        supervisor.get_observation().await.safety_state,
        SafetyState::Fault(FaultReason::Other(_))
    ));

    // 1. Simulate hardware driver refusing to clear fault (e.g. error register stuck)
    clear_fault_fail.store(true, Ordering::SeqCst);
    let res = supervisor.clear_fault().await;
    assert!(matches!(
        res,
        Err(ArmError::Safety(SafetyError::ClearFaultRejected(_)))
    ));

    // Software state must NOT be cleared when hardware clear failed!
    let obs = supervisor.get_observation().await;
    assert!(matches!(obs.safety_state, SafetyState::Fault(_)), "Must stay in Fault when hardware clear fails");

    // 2. Hardware clear succeeds: clear_fault transitions cleanly to Idle
    clear_fault_fail.store(false, Ordering::SeqCst);
    let res_ok = supervisor.clear_fault().await;
    assert!(res_ok.is_ok());
    let obs_ok = supervisor.get_observation().await;
    assert_eq!(obs_ok.safety_state, SafetyState::Idle);
}

#[tokio::test]
async fn test_normal_unload_settle_and_power_off_closed_loop() {
    let mut config = ArmConfig::default();
    config.dof = 2;
    config.home_positions = vec![0.0, 0.0];
    config.limits = vec![
        JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0),
        JointLimit::new("j2", -3.0, 3.0, 2.0, 5.0),
    ];

    let supervisor = ArmSupervisor::new(config);
    supervisor.set_torque(true).await.unwrap();

    // 1. Arm is torqued, jog to an extended pose
    supervisor.jog(vec![0.5, 0.5], Some(0.3), None).await.unwrap();
    for _ in 0..10 {
        supervisor.tick(0.02).await;
    }
    assert!(supervisor.get_observation().await.is_torqued);

    // 2. Trigger NormalUnload stop policy
    supervisor.apply_stop_policy(StopPolicy::NormalUnload).await;
    assert!(supervisor.is_unloading().await);

    // 3. Step until trajectory finishes
    while supervisor.has_active_trajectory().await {
        supervisor.tick(0.02).await;
    }

    // Trajectory is finished; arm is at home positions and settling window (40 ticks = 800ms) has started
    assert!(supervisor.get_observation().await.is_torqued, "Torque must be maintained during settle window");
    assert!(supervisor.is_unloading().await, "Must report unloading during settle window");

    // Advance 39 ticks: still within the 40-tick settle window
    for _ in 0..39 {
        supervisor.tick(0.02).await;
        assert!(supervisor.get_observation().await.is_torqued, "Torque must still be held during settle window");
    }

    // Advance 2 more ticks: torque cut is triggered
    supervisor.tick(0.02).await;
    supervisor.tick(0.02).await;

    let obs_done = supervisor.get_observation().await;
    assert!(!obs_done.is_torqued, "Torque must be cut after 40 stable resting ticks");
    assert_eq!(obs_done.safety_state, SafetyState::Idle, "Must transition to Idle state");
    assert!(!supervisor.is_unloading().await, "Must no longer be unloading");
}
