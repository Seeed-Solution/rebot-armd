//! E-stop latched while torqued, then cleared: armd keeps the motors holding (safety Idle, torque
//! on, re-arm pending). A park from there re-arms through the verified enable path and ends with
//! torque off at the resting pose (real arm 2026-11-20: the cockpit park did nothing in this state).
mod common;

use std::sync::Arc;
use std::time::{Duration, Instant};

use armd::model::{ArmConfig, JointLimit, RestingPosePolicy};
use armd::{ArmSupervisor, SafetyState};
use common::rpc;

fn config() -> ArmConfig {
    let mut config = ArmConfig::default();
    config.dof = 2;
    config.limits = vec![JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0), JointLimit::new("j2", -3.0, 3.0, 2.0, 5.0)];
    config.home_positions = vec![0.0, 0.0];
    config.lease_timeout_ms = 60_000;
    config.resting_pose = Some(RestingPosePolicy { positions: None, position_tolerances: vec![0.08; 2], max_total_deviation: 0.16,
        max_velocity: 0.015, joint_max_velocities: vec![], support_torque_limits: vec![(0, 1.0)], support_load_limits: vec![], feedback_max_age_ms: 100 });
    config
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn park_after_estop_clear_with_torque_held_ends_torque_off_at_rest() {
    let (mut ws, generation, _, supervisor, addr) = common::torqued_rig(Arc::new(ArmSupervisor::new(config())), &[1.0, 0.5]).await;
    let stopped = reqwest::Client::new().post(format!("http://{addr}/api/v1/safety/e_stop")).send().await.unwrap();
    assert!(stopped.status().is_success());
    let epoch = supervisor.get_observation().await.epoch_id;
    let clear = rpc(&mut ws, generation, epoch, "/api/v1/safety/clear_fault", serde_json::json!({})).await;
    assert_eq!(clear["type"], "command_ack", "{clear}");
    let obs = supervisor.get_observation().await;
    assert_eq!(obs.safety_state, SafetyState::Idle);
    assert!(obs.is_torqued, "the clear keeps the motors holding the arm");
    assert!(supervisor.inner().read().await.holding_after_clear());

    // torque off away from the resting pose stays refused (the arm would drop)
    let off = rpc(&mut ws, generation, obs.epoch_id, "/api/v1/torque", serde_json::json!({"enable": false})).await;
    assert_eq!(off["type"], "command_reject", "{off}");
    assert!(supervisor.get_observation().await.is_torqued);

    // park: re-arm, move to rest, torque off
    let epoch = supervisor.get_observation().await.epoch_id;
    let park = rpc(&mut ws, generation, epoch, "/api/v1/motion/unload", serde_json::json!({})).await;
    assert_eq!(park["type"], "command_ack", "{park}");
    let end = Instant::now() + Duration::from_secs(8);
    loop {
        let obs = supervisor.get_observation().await;
        if !obs.is_torqued {
            assert!(obs.positions.iter().all(|q| q.abs() < 0.08), "torque off at rest: {:?}", obs.positions);
            assert_eq!(obs.safety_state, SafetyState::Idle);
            break;
        }
        assert!(Instant::now() < end, "park did not finish: {:?} {:?}", obs.safety_state, obs.positions);
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
    assert!(!supervisor.inner().read().await.holding_after_clear());
}
