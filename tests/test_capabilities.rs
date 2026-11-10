//! `capabilities` in the observation schema: derived from the profile and the driver,
//! and absent (all false, version 0) when an older schema is parsed.

use armd::model::{Capabilities, CAPABILITIES_VERSION};
use armd::{ArmConfig, ArmSupervisor, ObservationSchema, ARM_PROFILE_VERSION};

#[tokio::test]
async fn rebot_dm_schema_declares_its_features() {
    let supervisor = ArmSupervisor::new(ArmConfig::rebot_dm());
    let caps = supervisor.get_schema().await.capabilities;
    assert_eq!(caps.version, CAPABILITIES_VERSION);
    assert_eq!(caps.profile_version, ARM_PROFILE_VERSION);
    assert!(caps.cartesian && caps.workspace_check && caps.gripper && caps.grasp && caps.damiao_ranges);
    assert_eq!(caps.version, 6);
    assert!(caps.obs_rate && caps.fault_codes && caps.poses && caps.keepout && caps.speed_cap
        && caps.residual_torques && caps.tick_stats && caps.place && caps.safety_describe);
    assert!(caps.vision.detection_proxy);
    assert_eq!(caps.vision.mode_endpoint, "/api/v1/vision/mode");
    assert!(caps.vision.modes.is_empty() && caps.vision.mode.is_none(), "provider fields come from the API layer");
    // Same source as the observation (profile, URDF check, driver).
    assert_eq!(caps.gravity_comp, supervisor.get_observation().await.gravity_comp.supported);
}

#[tokio::test]
async fn generic_profile_declares_no_arm_features() {
    let supervisor = ArmSupervisor::new(ArmConfig::default());
    let caps = supervisor.get_schema().await.capabilities;
    assert!(!caps.cartesian && !caps.workspace_check && !caps.gripper && !caps.grasp && !caps.gravity_comp && !caps.damiao_ranges);
    assert!(!caps.keepout && !caps.residual_torques, "need the workspace check / a gravity model");
    assert!(caps.obs_rate && caps.fault_codes && caps.poses && caps.speed_cap && caps.tick_stats && caps.safety_describe);
    assert_eq!(caps.gravity_comp, supervisor.get_observation().await.gravity_comp.supported);
}

#[test]
fn grasp_needs_gripper_and_kinematics() {
    let mut config = ArmConfig::rebot_dm();
    config.gripper_policy = None;
    let caps = Capabilities::from_config(&config);
    assert!(!caps.gripper && !caps.grasp && caps.cartesian);
    let mut config = ArmConfig::rebot_dm();
    config.kinematics = None;
    let caps = Capabilities::from_config(&config);
    assert!(caps.gripper && !caps.grasp && !caps.cartesian && !caps.workspace_check);
}

#[test]
fn schema_without_capabilities_still_parses() {
    let mut value = serde_json::to_value(ObservationSchema::from_config(&ArmConfig::rebot_dm())).unwrap();
    value.as_object_mut().unwrap().remove("capabilities");
    let schema: ObservationSchema = serde_json::from_value(value).unwrap();
    assert_eq!(schema.capabilities, Capabilities::default());
    assert_eq!(schema.capabilities.version, 0);
}
