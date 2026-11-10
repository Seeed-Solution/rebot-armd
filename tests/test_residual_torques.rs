//! `observation.residual_torques`: measured minus model gravity torque, independent of
//! gravity-compensation output.

use std::time::Duration;
use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
use armd::model::{ArmConfig, JointState};
use armd::ArmSupervisor;

const Q: [f32; 7] = [0.2, -0.8, -1.2, 1.0, 0.3, 1.0, 0.0];
const TAU: [f32; 7] = [0.1, 2.0, -1.5, 0.3, -0.2, 0.05, 0.0];

struct Driver { torque_feedback: bool }
impl ArmDriver for Driver {
    fn capabilities(&self) -> DriverCapabilities {
        // No torque feed-forward: gravity output stays off, the model still runs.
        DriverCapabilities { velocity_feedback: true, torque_feedback: self.torque_feedback, temperature_feedback: true,
            torque_feedforward: false, enable_feedback: true }
    }
    fn enable_state(&self) -> EnableState { EnableState::Disabled }
    fn is_feedback_valid(&self) -> bool { true }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); 7] }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        Ok(Q.iter().zip(TAU).map(|(&position, torque)| JointState { position, torque, temperature: 30.0, ..JointState::default() }).collect())
    }
    fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
    fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
}

#[tokio::test]
async fn residual_is_measured_minus_model_gravity_without_gravity_output() {
    let supervisor = ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(Driver { torque_feedback: true }));
    let obs = supervisor.tick(0.02).await;
    assert!(!obs.gravity_compensated && obs.gravity_torques.is_empty(), "gravity output stays off");
    let mut q6 = [0.0f32; 6];
    q6.copy_from_slice(&Q[..6]);
    let model = armd::compute_gravity_torques(&q6).unwrap();
    let residual = obs.residual_torques.expect("residual with a gravity model and torque feedback");
    for i in 0..6 {
        assert!((residual[i] - (TAU[i] - model[i])).abs() < 1e-5, "J{}: {} vs {}", i + 1, residual[i], TAU[i] - model[i]);
    }
    assert!(model.iter().any(|t| t.abs() > 0.1), "the pose loads the arm: {model:?}");
    let json = serde_json::to_value(&obs).unwrap();
    assert_eq!(json["residual_torques"].as_array().unwrap().len(), 6);
    assert!(!obs.safety_state.is_stopped(), "no fault is raised from residuals");
}

#[tokio::test]
async fn residual_is_null_without_torque_feedback_or_gravity_model() {
    let supervisor = ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(Driver { torque_feedback: false }));
    assert!(supervisor.tick(0.02).await.residual_torques.is_none());
    let mut config = ArmConfig::rebot_dm();
    config.gravity_model = None;
    config.gravity_comp = None;
    let supervisor = ArmSupervisor::new(config).with_driver(Box::new(Driver { torque_feedback: true }));
    assert!(supervisor.tick(0.02).await.residual_torques.is_none());
    // Mock follower (no driver): torques are not measured.
    let obs = ArmSupervisor::new(ArmConfig::rebot_dm()).tick(0.02).await;
    assert!(obs.residual_torques.is_none());
    assert!(serde_json::to_value(&obs).unwrap()["residual_torques"].is_null());
}
