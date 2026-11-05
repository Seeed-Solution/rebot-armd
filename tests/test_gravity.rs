use armd::gravity::{
    compute_gravity_torques, verify_urdf_sha256, GravityError, EXPECTED_URDF_SHA256,
};
use std::path::Path;

#[test]
fn test_golden_vector_zero() {
    let q_zero = [0.0f32; 6];
    let expected = [0.0, -1.135664, -7.615891, -2.151496, 0.000006, 0.000311];

    let tau = compute_gravity_torques(&q_zero).expect("Failed to compute gravity torque at zero pose");

    for i in 0..6 {
        let diff = (tau[i] - expected[i]).abs();
        assert!(
            diff <= 1e-4,
            "Joint {} error {} > 1e-4: actual={}, expected={}",
            i + 1,
            diff,
            tau[i],
            expected[i]
        );
    }
}

#[test]
fn test_golden_vector_test_pose() {
    let q_test = [0.0, -0.5, 0.5, 0.0, 0.0, 0.0];
    let expected = [0.0, -2.142779, -5.518330, -1.358455, -0.000252, 0.000168];

    let tau = compute_gravity_torques(&q_test).expect("Failed to compute gravity torque at test pose");

    for i in 0..6 {
        let diff = (tau[i] - expected[i]).abs();
        assert!(
            diff <= 1e-4,
            "Joint {} error {} > 1e-4: actual={}, expected={}",
            i + 1,
            diff,
            tau[i],
            expected[i]
        );
    }
}

#[test]
fn test_urdf_sha256_verification() {
    let urdf_path = Path::new("assets/reBot-DevArm_gripper.urdf");
    if urdf_path.exists() {
        assert!(
            verify_urdf_sha256(urdf_path).is_ok(),
            "URDF SHA256 should match {}",
            EXPECTED_URDF_SHA256
        );
    }
}

#[test]
fn test_invalid_angle_detection() {
    let q_nan = [0.0, f32::NAN, 0.0, 0.0, 0.0, 0.0];
    assert!(matches!(
        compute_gravity_torques(&q_nan),
        Err(GravityError::NonFiniteAngle(1, _))
    ));

    let q_inf = [0.0, 0.0, f32::INFINITY, 0.0, 0.0, 0.0];
    assert!(matches!(
        compute_gravity_torques(&q_inf),
        Err(GravityError::NonFiniteAngle(2, _))
    ));
}

#[tokio::test]
async fn test_supervisor_gravity_compensation_integration() {
    use armd::{ArmConfig, ArmSupervisor};

    let config = ArmConfig::default();
    let supervisor = ArmSupervisor::new(config);

    // Assets exist in repo, so gravity_compensated must be true
    assert!(supervisor.is_gravity_compensated().await, "Supervisor must initialize with gravity_compensated=true");

    // Tick at zero pose
    let obs = supervisor.tick(0.02).await;
    assert!(obs.gravity_compensated);
    assert_eq!(obs.gravity_torques.len(), 6);

    let expected_zero = [0.0, -1.135664, -7.615891, -2.151496, 0.000006, 0.000311];
    for i in 0..6 {
        assert!(
            (obs.gravity_torques[i] - expected_zero[i]).abs() <= 1e-4,
            "Joint {} gravity torque mismatch in observation: actual={}, expected={}",
            i + 1,
            obs.gravity_torques[i],
            expected_zero[i]
        );
    }
}

#[tokio::test]
async fn test_gravity_feedforward_torque_ramping_and_injection() {
    use armd::{ArmConfig, ArmSupervisor, JointTarget, ArmDriver, ArmDriverError, JointState};
    use std::sync::{Arc, Mutex};

    struct MockRecordingDriver {
        targets_log: Arc<Mutex<Vec<Vec<JointTarget>>>>,
    }
    impl ArmDriver for MockRecordingDriver {
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            Ok(vec![JointState::default(); 7])
        }
        fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
            self.targets_log.lock().unwrap().push(targets.to_vec());
            Ok(())
        }
        fn set_torque(&mut self, _enabled: bool) -> Result<(), ArmDriverError> { Ok(()) }
        fn is_feedback_valid(&self) -> bool { true }
    }

    let recorded = Arc::new(Mutex::new(Vec::new()));
    let driver = Box::new(MockRecordingDriver {
        targets_log: recorded.clone(),
    });

    let config = ArmConfig::default();
    let supervisor = ArmSupervisor::new(config).with_driver(driver);
    supervisor.set_torque(true).await.unwrap();

    // Advance 5 ticks (0.1s total) to ramp up torque
    for _ in 0..5 {
        supervisor.tick(0.02).await;
    }

    let logs = recorded.lock().unwrap();
    assert!(!logs.is_empty(), "Must have written joint targets to driver");
    let last_frame = logs.last().unwrap();
    assert_eq!(last_frame.len(), 6);

    // Verify J2 and J3 have non-zero feedforward torque from ramp
    let j2_torque = last_frame[1].torque.expect("Joint 2 torque must be injected");
    let j3_torque = last_frame[2].torque.expect("Joint 3 torque must be injected");
    assert!(j2_torque.abs() > 0.01, "J2 feedforward torque must be non-zero (got {})", j2_torque);
    assert!(j3_torque.abs() > 0.01, "J3 feedforward torque must be non-zero (got {})", j3_torque);
}
