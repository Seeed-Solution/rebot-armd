//! `capabilities.calibration_tools`: which calibration tools an arm supports, built from the
//! profile and the driver.
use armd::{ArmConfig, ArmSupervisor};
use serde_json::{json, Value};

fn so101() -> ArmConfig {
    let mut cfg: ArmConfig = serde_json::from_str(include_str!("../profiles/so_arm101.json")).unwrap();
    cfg.init_arm_model_in_memory(Some(include_str!("../assets/so101_new_calib.urdf"))).unwrap();
    cfg
}

async fn tools(cfg: ArmConfig) -> Value {
    let sup = ArmSupervisor::new(cfg);
    let schema = serde_json::to_value(sup.get_schema().await).unwrap();
    assert!(schema["capabilities"]["version"].as_u64().unwrap() >= 10);
    schema["capabilities"]["calibration_tools"].clone()
}

#[tokio::test]
async fn b601_dm_reports_camera_kinematics_gripper_and_the_zero_pose() {
    let t = tools(ArmConfig::rebot_dm()).await;
    assert_eq!(t["camera"], json!({"supported": true}), "{t}");
    assert_eq!(t["kinematics"], json!({"supported": true, "joints": [2, 3, 4, 5]}), "{t}");
    assert_eq!(t["gripper"]["supported"], json!(true));
    assert_eq!(t["gripper"]["calibration"], json!(true));
    let zp = &t["motor_zero"]["zero_pose"];
    assert_eq!(zp["joints"].as_array().unwrap().len(), 6, "{t}");
    assert!(zp["gripper"].as_str().unwrap().contains("closed"), "{t}");
    assert_eq!(t["motor_zero"]["max_offset_deg"], json!(45));
}

#[tokio::test]
async fn urdf_chain_profile_reports_unsupported_tools_with_a_reason() {
    let t = tools(so101()).await;
    assert_eq!(t["kinematics"]["supported"], json!(false), "{t}");
    assert!(t["kinematics"]["reason"].as_str().unwrap().contains("URDF"), "{t}");
    assert!(t["kinematics"].get("joints").is_none(), "{t}");
    assert!(t["motor_zero"]["zero_pose"].is_null(), "{t}");
}

#[tokio::test]
async fn get_calibration_carries_the_tools() {
    let sup = ArmSupervisor::new(ArmConfig::rebot_dm());
    let dir = tempfile::tempdir().unwrap();
    let up = armd::api::proxy::DetectorUpstream::parse("http://127.0.0.1:9").unwrap();
    let deps = armd::calibration::CalibrationDeps::new(dir.path().join("calibration.json"), None, up);
    let status = armd::calibration::status(&sup, &deps).await;
    let schema = serde_json::to_value(sup.get_schema().await).unwrap();
    assert_eq!(status["calibration_tools"], schema["capabilities"]["calibration_tools"]);
}

#[tokio::test]
async fn motor_zero_support_comes_from_the_driver() {
    // mock follower (no driver): supported, readings reset in the process
    let t = tools(ArmConfig::rebot_dm()).await;
    assert_eq!(t["motor_zero"]["supported"], json!(true), "{t}");
    assert_eq!(t["motor_zero"]["joints"], json!([1, 2, 3, 4, 5, 6, 7]));
    assert_eq!(t["motor_zero"]["method"], json!("set_current"));
    assert_eq!(t["motor_zero"]["requires"], json!(["lease", "torque_off", "no_action", "no_fault"]));
    assert_eq!(t["motor_zero"]["invalidates"], json!(["kinematics", "camera"]));
    // Damiao: supported, persistence not verified on hardware
    let dm = armd::driver::DamiaoArmDriver::new(armd::transport::MockCanTransport::new(), (1..=7).collect());
    let sup = ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(dm));
    let t = serde_json::to_value(sup.get_schema().await).unwrap()["capabilities"]["calibration_tools"].clone();
    assert_eq!(t["motor_zero"]["persist"], json!("motor_flash"), "{t}");
    assert_eq!(t["motor_zero"]["persist_verified"], json!(false), "{t}");
    // RobStride: unsupported with the reason
    let rs = armd::driver::RobStrideArmDriver::new(armd::transport::MockCanTransport::new(), (1..=6).collect());
    let sup = ArmSupervisor::new(ArmConfig { dof: 6, ..ArmConfig::default() }).with_driver(Box::new(rs));
    let t = serde_json::to_value(sup.get_schema().await).unwrap()["capabilities"]["calibration_tools"].clone();
    assert_eq!(t["motor_zero"]["supported"], json!(false), "{t}");
    assert!(t["motor_zero"]["reason"].as_str().unwrap().contains("RobStride"), "{t}");
    assert!(t["motor_zero"].get("joints").is_none(), "{t}");
}
