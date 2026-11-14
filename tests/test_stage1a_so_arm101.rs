//! Tests for Stage 1A of the SO-ARM101 plan:
//! 1) 5+1 schema fields (arm_dof=5, gripper_joint=Some(5), orientation_dof=2, gravity_torques.shape == [5], capabilities.cartesian/gripper/place/grasp),
//! 2) ArmObservation.tcp and ArmObservation.gripper populated for 5+1 / 6+1 and gripper = None for 6+0,
//! 3) gripper_zero_check rejected with UNSUPPORTED when zero_check_supported == false,
//! 4) 6+0 vs 5+1 pose and replay cross-load rejection (POSE_DOF_MISMATCH / REPLAY_DOF_MISMATCH).

use serde_json::json;

use armd::model::{
    ArmConfig, Capabilities, GripperPolicy, HoldingMode, JointLimit, ObservationSchema,
};
use armd::motion::MotionError;
use armd::poses::{NamedPose, PoseList};
use armd::recording::{write_frames_to_parquet, write_frames_to_parquet_with_meta, EpisodeMetadata, RecordFrame};
use armd::replay::{plan, ReplayInput, ReplayRequest};
use armd::supervisor::{ArmError, SupervisorInner};

const DM_URDF: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/assets/reBot-DevArm_gripper.urdf");

fn so_arm101_config() -> ArmConfig {
    let dm = ArmConfig::rebot_dm();
    let mut v = serde_json::to_value(&dm).unwrap();
    let o = v.as_object_mut().unwrap();
    o.insert("profile_id".into(), "so_arm101".into());
    o.insert("kinematics".into(), "Urdf".into());
    o.insert("gravity_model".into(), "Urdf".into());
    o.insert("urdf".into(), DM_URDF.into());
    o.insert("urdf_sha256".into(), armd::gravity::EXPECTED_URDF_SHA256.into());
    o.insert("chain".into(), serde_json::json!({
        "base_link": "base_link",
        "tip_link": "link5",
        "joints": (0..5).map(|i| serde_json::json!({
            "urdf_joint": format!("joint{}", i + 1),
            "offset_rad": 0.0
        })).collect::<Vec<_>>(),
    }));
    o.remove("model_offsets_rad");
    let mut c: ArmConfig = serde_json::from_value(v).unwrap();
    c.dof = 6;
    c.joint_names = vec!["j1".into(), "j2".into(), "j3".into(), "j4".into(), "j5".into(), "gripper".into()];
    c.limits = (0..6).map(|i| JointLimit {
        name: if i < 5 { format!("j{}", i + 1) } else { "gripper".into() },
        min_position: -3.14,
        max_position: 3.14,
        max_velocity: 1.0,
        max_acceleration: 2.0,
    }).collect();
    c.home_positions = vec![0.0; 6];
    c.gripper_joint = Some(5);
    c.gripper_policy = Some(GripperPolicy {
        closed_rad: 0.0,
        open_rad: 1.0,
        holding_gap_rad: 0.05,
        holding_torque_nm: 0.1,
        squeeze_rad: 0.02,
        settle_ms: 100,
        jaw_opening: None,
        zero_check_supported: false,
        holding_mode: HoldingMode::LoadAndGap,
        holding_load_fraction: Some(0.25),
    });
    c.grasp = Some(dm.grasp.unwrap());
    c.gravity_comp = None;
    c.torque_guard = None;
    c.damiao_ranges = None;
    c.servo_max_velocity = None;
    c.resting_pose = None;
    c.power_budget = None;
    c.load_arm_model(None).unwrap();
    c.validate().unwrap();
    c
}

fn config_6_plus_0() -> ArmConfig {
    let dm = ArmConfig::rebot_dm();
    let mut v = serde_json::to_value(&dm).unwrap();
    let o = v.as_object_mut().unwrap();
    o.insert("profile_id".into(), "b601rs_6dof".into());
    o.insert("kinematics".into(), "Urdf".into());
    o.insert("gravity_model".into(), "Urdf".into());
    o.insert("urdf".into(), DM_URDF.into());
    o.insert("urdf_sha256".into(), armd::gravity::EXPECTED_URDF_SHA256.into());
    o.insert("chain".into(), serde_json::json!({
        "base_link": "base_link",
        "tip_link": "link6",
        "joints": (0..6).map(|i| serde_json::json!({
            "urdf_joint": format!("joint{}", i + 1),
            "offset_rad": 0.0
        })).collect::<Vec<_>>(),
    }));
    o.remove("model_offsets_rad");
    let mut c: ArmConfig = serde_json::from_value(v).unwrap();
    c.dof = 6;
    c.joint_names.truncate(6);
    c.limits.truncate(6);
    c.home_positions.truncate(6);
    c.gripper_joint = None;
    c.gripper_policy = None;
    c.grasp = None;
    c.gravity_comp = None;
    c.torque_guard = None;
    c.damiao_ranges = None;
    c.servo_max_velocity = None;
    c.resting_pose = None;
    c.power_budget = None;
    c.load_arm_model(None).unwrap();
    c.validate().unwrap();
    c
}

#[test]
fn test_5_plus_1_schema_and_capabilities() {
    let c = so_arm101_config();
    assert_eq!(c.arm_dof(), 5);
    assert_eq!(c.gripper_index(), Some(5));
    assert_eq!(c.dof, 6);
    assert!(c.has_cartesian_with_gripper());
    assert!(!c.six_plus_gripper());

    let schema = ObservationSchema::from_config(&c);
    assert_eq!(schema.profile_id, Some("so_arm101".into()));
    assert_eq!(schema.arm_dof, 5);
    assert_eq!(schema.gripper_joint, Some(5));
    assert_eq!(schema.orientation_dof, 2);
    assert_eq!(schema.dof, 6);
    assert_eq!(schema.fields.iter().find(|f| f.name == "gravity_torques").unwrap().shape, vec![5]);
    assert_eq!(schema.gripper_policy.as_ref().unwrap().holding_mode, HoldingMode::LoadAndGap);
    assert_eq!(schema.gripper_policy.as_ref().unwrap().holding_load_fraction, Some(0.25));
    assert!(!schema.gripper_policy.as_ref().unwrap().zero_check_supported);

    let caps = Capabilities::from_config(&c);
    assert!(caps.cartesian);
    assert!(caps.gripper);
    assert!(caps.place);
    assert!(caps.grasp);
    assert!(!caps.gravity_comp);
    assert!(!caps.residual_torques);
    assert!(!caps.collision_guard);
}

#[test]
fn test_observation_tcp_and_gripper_populated() {
    // 1) 5+1 arm (SO-ARM101)
    let c51 = so_arm101_config();
    let sup51 = SupervisorInner::new(c51);
    let obs51 = sup51.current_observation();
    assert!(obs51.tcp.is_some(), "5+1 arm must populate tcp observation");
    let tcp51 = obs51.tcp.unwrap();
    assert!(tcp51.xyz_m.iter().all(|v| v.is_finite()));
    assert!(tcp51.rpy_rad.iter().all(|v| v.is_finite()));
    assert!(tcp51.quat_wxyz.iter().all(|v| v.is_finite()));
    assert!(obs51.gripper.is_some(), "5+1 arm must populate gripper observation");
    let g51 = obs51.gripper.unwrap();
    assert_eq!(g51.joint, 5);
    assert_eq!(g51.rad, 0.0);
    assert!(!g51.stalled);

    // 2) 6+1 arm (B601-DM)
    let c61 = ArmConfig::rebot_dm();
    let sup61 = SupervisorInner::new(c61);
    let obs61 = sup61.current_observation();
    assert!(obs61.tcp.is_some(), "6+1 arm must populate tcp observation");
    assert!(obs61.gripper.is_some(), "6+1 arm must populate gripper observation");
    assert_eq!(obs61.gripper.unwrap().joint, 6);

    // 3) 6+0 arm (B601-RS without gripper)
    let c60 = config_6_plus_0();
    let sup60 = SupervisorInner::new(c60);
    let obs60 = sup60.current_observation();
    assert!(obs60.tcp.is_some(), "6+0 arm must populate tcp observation");
    assert!(obs60.gripper.is_none(), "6+0 arm must NOT populate gripper observation");
}

#[test]
fn test_gripper_zero_check_rejected_when_not_supported() {
    let mut c = so_arm101_config();
    assert!(!c.gripper_policy.as_ref().unwrap().zero_check_supported);
    let mut sup = SupervisorInner::new(c.clone());
    sup.lease_enabled = false;
    sup.set_torque(true).unwrap();
    let res = sup.start_gripper_zero_check(&json!({}), sup.safety.epoch_id());
    match res {
        Err(ArmError::Motion(MotionError::Unsupported(msg))) => {
            assert!(msg.contains("not supported"), "expected message containing 'not supported', got: {msg}");
        }
        other => panic!("expected Unsupported error, got {other:?}"),
    }

    // Now enable zero_check_supported and verify it is accepted
    c.gripper_policy.as_mut().unwrap().zero_check_supported = true;
    let mut sup_ok = SupervisorInner::new(c);
    sup_ok.lease_enabled = false;
    sup_ok.set_torque(true).unwrap();
    let res_ok = sup_ok.start_gripper_zero_check(&json!({}), sup_ok.safety.epoch_id());
    assert!(res_ok.is_ok(), "expected Ok when zero_check_supported is true, got: {res_ok:?}");
}

#[test]
fn test_6_plus_0_vs_5_plus_1_cross_load_rejections() {
    let c60 = config_6_plus_0();
    assert_eq!((c60.dof, c60.arm_dof(), c60.gripper_index()), (6, 6, None));

    // 1) Pose cross-load rejection
    let pose_51 = NamedPose {
        name: "test_51".into(),
        positions: vec![0.0; 6],
        tcp: None,
        saved_at_ms: 100,
        arm_dof: Some(5),
        gripper_joint: Some(5),
        joint_names: Some(vec!["j1".into(), "j2".into(), "j3".into(), "j4".into(), "j5".into(), "gripper".into()]),
    };

    let list_51 = PoseList {
        arm_dof: Some(5),
        gripper_joint: Some(5),
        joint_names: Some(vec!["j1".into(), "j2".into(), "j3".into(), "j4".into(), "j5".into(), "gripper".into()]),
        poses: vec![pose_51.clone()],
    };

    // validate_list against 6+0 config must reject with POSE_DOF_MISMATCH
    let val_err = armd::poses::validate_list(&list_51, &c60).unwrap_err();
    assert!(val_err.contains("POSE_DOF_MISMATCH"), "expected POSE_DOF_MISMATCH, got: {val_err}");

    // move_to_pose against 6+0 supervisor must reject with POSE_DOF_MISMATCH
    let mut sup60 = SupervisorInner::new(c60.clone());
    sup60.poses.list.poses.push(pose_51);
    let move_err = sup60.move_to_pose("test_51", false, 1.0, None, 0).unwrap_err();
    match move_err {
        ArmError::Rejected { code, message, .. } => {
            assert_eq!(code, "POSE_DOF_MISMATCH");
            assert!(message.contains("arm_dof 5 does not match active config arm_dof 6"), "{message}");
        }
        other => panic!("expected POSE_DOF_MISMATCH rejection, got: {other:?}"),
    }

    // 2) Replay cross-load rejection
    let dir = tempfile::tempdir().unwrap();
    let ep_path = dir.path().join("ep51.parquet");
    let frames = vec![
        RecordFrame { timestamp: 0.0, observation_state: vec![0.0; 6], action: vec![0.0; 6] },
        RecordFrame { timestamp: 0.1, observation_state: vec![0.0; 6], action: vec![0.0; 6] },
    ];
    let meta_51 = EpisodeMetadata {
        arm_dof: Some(5),
        gripper_joint: Some(5),
        joint_names: Some(vec!["j1".into(), "j2".into(), "j3".into(), "j4".into(), "j5".into(), "gripper".into()]),
    };
    write_frames_to_parquet_with_meta(&ep_path, &frames, Some(&meta_51)).unwrap();

    let replay_input_60 = ReplayInput {
        request: ReplayRequest { episode: "ep51".into(), speed: 1.0, gripper: false, tag: None },
        path: ep_path.clone(),
        dof: 6,
        arm_dof: 6,
        gripper_idx: None,
        joint_names: c60.limits.iter().map(|l| l.name.clone()).collect(),
        limits: c60.limits.clone(),
        speed_limits: c60.limits.clone(),
        ws: None,
        arm: c60.arm_model.clone(),
        start: vec![0.0; 6],
        control_hz: 50,
        epoch: 0,
        workspace_revision: 0,
        power: None,
    };

    let replay_err = plan(replay_input_60).unwrap_err();
    match replay_err {
        ArmError::Rejected { code, message, .. } => {
            assert_eq!(code, "REPLAY_DOF_MISMATCH");
            assert!(message.contains("arm_dof 5 does not match active config arm_dof 6"), "{message}");
        }
        other => panic!("expected REPLAY_DOF_MISMATCH rejection, got: {other:?}"),
    }

    // 3) Backward compatibility: legacy episode without metadata must succeed
    let legacy_path = dir.path().join("legacy.parquet");
    write_frames_to_parquet(&legacy_path, &frames).unwrap();
    let replay_input_legacy = ReplayInput {
        request: ReplayRequest { episode: "legacy".into(), speed: 1.0, gripper: false, tag: None },
        path: legacy_path,
        dof: 6,
        arm_dof: 6,
        gripper_idx: None,
        joint_names: c60.limits.iter().map(|l| l.name.clone()).collect(),
        limits: c60.limits.clone(),
        speed_limits: c60.limits.clone(),
        ws: None,
        arm: c60.arm_model.clone(),
        start: vec![0.0; 6],
        control_hz: 50,
        epoch: 0,
        workspace_revision: 0,
        power: None,
    };
    let plan_res = plan(replay_input_legacy);
    assert!(plan_res.is_ok(), "expected legacy replay to succeed, got: {plan_res:?}");
}
