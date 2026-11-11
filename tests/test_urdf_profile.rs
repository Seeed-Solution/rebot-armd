//! Profiles with `kinematics` / `gravity_model` = `Urdf` (src/chain.rs via the profile
//! fields `urdf`, `urdf_sha256`, `chain`, `gripper_joint`).
use armd::chain::ArmModel;
use armd::model::{GravityModel, KinematicsModel};
use armd::ArmConfig;

const DM_URDF: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/assets/reBot-DevArm_gripper.urdf");

fn dm_chain_json() -> serde_json::Value {
    serde_json::json!({
        "base_link": "base_link",
        "tip_link": "end_link",
        "joints": (1..=6).map(|i| serde_json::json!({"urdf_joint": format!("joint{i}")})).collect::<Vec<_>>(),
    })
}

/// The B601-DM built-in profile switched to the URDF chain of the same arm.
fn dm_as_urdf() -> ArmConfig {
    let mut v = serde_json::to_value(ArmConfig::rebot_dm()).unwrap();
    let o = v.as_object_mut().unwrap();
    o.insert("kinematics".into(), "Urdf".into());
    o.insert("gravity_model".into(), "Urdf".into());
    o.insert("urdf".into(), DM_URDF.into());
    o.insert("urdf_sha256".into(), armd::gravity::EXPECTED_URDF_SHA256.into());
    o.insert("chain".into(), dm_chain_json());
    let mut c: ArmConfig = serde_json::from_value(v).unwrap();
    c.load_arm_model(None).unwrap();
    c.validate().unwrap();
    c
}

#[test]
fn dm_urdf_profile_matches_builtin_profile() {
    let builtin = ArmConfig::rebot_dm();
    let urdf = dm_as_urdf();
    assert!(matches!(urdf.arm_model, ArmModel::Chain(_)));
    assert_eq!((urdf.arm_dof(), urdf.gripper_index()), (6, Some(6)));
    assert_eq!((builtin.arm_dof(), builtin.gripper_index()), (6, Some(6)));
    assert!(urdf.six_plus_gripper());
    // Same capabilities apart from the model names.
    assert_eq!(armd::model::Capabilities::from_config(&urdf), armd::model::Capabilities::from_config(&builtin));
    // Same workspace check results over a sweep (reach origin differs by 1.6e-7 m).
    let ws = builtin.workspace.clone().unwrap();
    let mut x = 0.123f64;
    for _ in 0..500 {
        let q: Vec<f32> = (0..7).map(|_| { x = (x * 9301.0 + 0.49297).fract(); (x * 4.0 - 2.0) as f32 }).collect();
        let a = builtin.arm_model.workspace_violation(&q, &ws, &builtin.camera_mount);
        let b = urdf.arm_model.workspace_violation(&q, &ws, &urdf.camera_mount);
        assert!((a.total - b.total).abs() < 1e-6, "{q:?}: {} vs {}", a.total, b.total);
        let ga = builtin.arm_model.gravity_torques(&q).unwrap();
        let gb = urdf.arm_model.gravity_torques(&q).unwrap();
        for i in 0..6 { assert!((ga[i] - gb[i]).abs() < 4e-6); }
    }
}

#[test]
fn urdf_profile_errors() {
    let mut c = ArmConfig::rebot_dm();
    c.kinematics = Some(KinematicsModel::Urdf);
    c.gravity_model = Some(GravityModel::Urdf);
    assert!(c.load_arm_model(None).unwrap_err().contains("urdf and chain"));
    // validate without a loaded chain
    assert!(c.validate().is_err());

    let mut c = dm_as_urdf();
    c.urdf_sha256 = Some("00".repeat(32));
    assert!(c.load_arm_model(None).unwrap_err().contains("sha256"));

    let mut c = dm_as_urdf();
    c.gravity_model = Some(GravityModel::RebotDevArm);
    assert!(c.validate().unwrap_err().contains("both"));

    // urdf without a Urdf model is not silently ignored
    let mut c = ArmConfig::rebot_dm();
    c.urdf = Some(DM_URDF.into());
    assert!(c.load_arm_model(None).is_err());

    let mut c = dm_as_urdf();
    c.gripper_joint = Some(3);
    assert!(c.validate().unwrap_err().contains("gripper_joint"));
}

#[test]
fn urdf_profile_without_gripper() {
    // 6 arm joints, no gripper: the workspace check and gravity apply; the Cartesian
    // actions (6 arm joints + gripper at joint 7) do not.
    let mut c = dm_as_urdf();
    c.dof = 6;
    c.joint_names.truncate(6);
    c.limits.truncate(6);
    c.home_positions.truncate(6);
    c.gripper_policy = None;
    c.grasp = None;
    c.gravity_comp = None;
    c.damiao_ranges.as_mut().unwrap().truncate(6);
    c.servo_max_velocity.as_mut().unwrap().truncate(6);
    if let Some(r) = c.resting_pose.as_mut() { r.joint_max_velocities.truncate(6); }
    c.validate().unwrap();
    assert_eq!(c.gripper_index(), None);
    let caps = armd::model::Capabilities::from_config(&c);
    assert!(caps.workspace_check && !caps.cartesian && !caps.grasp && caps.collision_guard);
    let sup = armd::SupervisorInner::new(c);
    assert!(sup.workspace_policy().is_some());
    assert!(sup.gravity_compensated);
}
