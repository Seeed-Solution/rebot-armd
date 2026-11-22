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
    // the B601-DM link capsules name link6, which this 5-joint chain does not have
    if let Some(ws) = o.get_mut("workspace").and_then(|w| w.as_object_mut()) {
        for k in ["link_capsules", "self_collision_enabled", "self_collision_pairs"] { ws.remove(k); }
    }
    let mut c: ArmConfig = serde_json::from_value(v).unwrap();
    c.dof = 6;
    c.zero_reference = None;
    c.joint_names = vec!["j1".into(), "j2".into(), "j3".into(), "j4".into(), "j5".into(), "gripper".into()];
    c.limits = (0..6).map(|i| JointLimit {
        name: if i < 5 { format!("j{}", i + 1) } else { "gripper".into() },
        min_position: -std::f32::consts::PI,
        max_position: std::f32::consts::PI,
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
        jaw_opening: Some(armd::model::JawOpening { points: vec![[0.0, 0.0], [1.0, 0.075]], calibrated: false }),
        zero_check_supported: false,
        holding_mode: HoldingMode::LoadAndGap,
        holding_load_fraction: Some(0.25),
        max_close_torque_nm: None,
        contact_torque_nm: None,
        max_lead_rad: None,
        stream_contact_speed_rad_s: None,
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
    // the B601-DM link capsules name link6, which this 5-joint chain does not have
    if let Some(ws) = o.get_mut("workspace").and_then(|w| w.as_object_mut()) {
        for k in ["link_capsules", "self_collision_enabled", "self_collision_pairs"] { ws.remove(k); }
    }
    let mut c: ArmConfig = serde_json::from_value(v).unwrap();
    c.dof = 6;
    c.zero_reference = None;
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
        id: String::new(),
        name: "test_51".into(),
        positions: vec![0.0; 6],
        tcp: None,
        saved_at_ms: 100,
        profile_id: None,
        arm_dof: Some(5),
        gripper_joint: Some(5),
        joint_names: Some(vec!["j1".into(), "j2".into(), "j3".into(), "j4".into(), "j5".into(), "gripper".into()]),
        stale: None,
        tool_id: None,
        tool_config_digest: None,
    };

    let list_51 = PoseList {
        profile_id: None,
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
        profile_id: None,
        arm_dof: Some(5),
        gripper_joint: Some(5),
        joint_names: Some(vec!["j1".into(), "j2".into(), "j3".into(), "j4".into(), "j5".into(), "gripper".into()]),
        tool_id: None,
        tool_config_digest: None,
        has_gripper_motion: None,
    };
    write_frames_to_parquet_with_meta(&ep_path, &frames, Some(&meta_51)).unwrap();

    let replay_input_60 = ReplayInput {
        request: ReplayRequest { episode: "ep51".into(), speed: 1.0, gripper: false, tag: None, allow_stale: false },
        path: ep_path.clone(),
        profile_id: c60.profile_id.clone(),
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
        store: armd::recording::EpisodeStore::new(Some(dir.path().to_path_buf())),
        zero_offsets: vec![0.0; 6],
        tcp_offset: [0.0; 3],
        active_tool: Default::default(),
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
        request: ReplayRequest { episode: "legacy".into(), speed: 1.0, gripper: false, tag: None, allow_stale: false },
        path: legacy_path,
        profile_id: c60.profile_id.clone(),
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
        store: armd::recording::EpisodeStore::new(Some(dir.path().to_path_buf())),
        zero_offsets: vec![0.0; 6],
        tcp_offset: [0.0; 3],
        active_tool: Default::default(),
    };
    let plan_res = plan(replay_input_legacy);
    assert!(plan_res.is_ok(), "expected legacy replay to succeed, got: {plan_res:?}");
}

#[test]
fn test_profile_id_pose_and_replay_rejection() {
    let c_so101 = so_arm101_config();
    assert_eq!(c_so101.profile_id.as_deref(), Some("so_arm101"));

    // 1) Pose with profile_id = "so_arm100" tested against active profile "so_arm101"
    let pose_100 = NamedPose {
        id: String::new(),
        name: "pose_100".into(),
        positions: vec![0.0; 6],
        tcp: None,
        saved_at_ms: 200,
        profile_id: Some("so_arm100".into()),
        arm_dof: Some(5),
        gripper_joint: Some(5),
        joint_names: Some(vec!["j1".into(), "j2".into(), "j3".into(), "j4".into(), "j5".into(), "gripper".into()]),
        stale: None,
        tool_id: None,
        tool_config_digest: None,
    };
    let list_100 = PoseList {
        profile_id: Some("so_arm100".into()),
        arm_dof: Some(5),
        gripper_joint: Some(5),
        joint_names: Some(vec!["j1".into(), "j2".into(), "j3".into(), "j4".into(), "j5".into(), "gripper".into()]),
        poses: vec![pose_100.clone()],
    };

    let err = armd::poses::validate_list(&list_100, &c_so101).unwrap_err();
    assert!(err.contains("POSE_DOF_MISMATCH"), "expected POSE_DOF_MISMATCH, got: {err}");
    assert!(err.contains("so_arm100"), "expected so_arm100 in error, got: {err}");

    let mut sup = SupervisorInner::new(c_so101.clone());
    sup.poses.list.poses.push(pose_100);
    let move_err = sup.move_to_pose("pose_100", false, 1.0, None, 0).unwrap_err();
    match move_err {
        ArmError::Rejected { code, message, .. } => {
            assert_eq!(code, "POSE_DOF_MISMATCH");
            assert!(message.contains("so_arm100"), "{message}");
        }
        other => panic!("expected POSE_DOF_MISMATCH rejection, got: {other:?}"),
    }

    // 2) Replay with profile_id = "so_arm100" against active profile "so_arm101"
    let dir = tempfile::tempdir().unwrap();
    let ep_path = dir.path().join("ep100.parquet");
    let frames = vec![
        RecordFrame { timestamp: 0.0, observation_state: vec![0.0; 6], action: vec![0.0; 6] },
        RecordFrame { timestamp: 0.1, observation_state: vec![0.0; 6], action: vec![0.0; 6] },
    ];
    let meta_100 = EpisodeMetadata {
        profile_id: Some("so_arm100".into()),
        arm_dof: Some(5),
        gripper_joint: Some(5),
        joint_names: Some(vec!["j1".into(), "j2".into(), "j3".into(), "j4".into(), "j5".into(), "gripper".into()]),
        tool_id: None,
        tool_config_digest: None,
        has_gripper_motion: None,
    };
    write_frames_to_parquet_with_meta(&ep_path, &frames, Some(&meta_100)).unwrap();

    let replay_input = ReplayInput {
        request: ReplayRequest { episode: "ep100".into(), speed: 1.0, gripper: false, tag: None, allow_stale: false },
        path: ep_path,
        profile_id: c_so101.profile_id.clone(),
        dof: 6,
        arm_dof: 5,
        gripper_idx: Some(5),
        joint_names: c_so101.limits.iter().map(|l| l.name.clone()).collect(),
        limits: c_so101.limits.clone(),
        speed_limits: c_so101.limits.clone(),
        ws: None,
        arm: c_so101.arm_model.clone(),
        start: vec![0.0; 6],
        control_hz: 50,
        epoch: 0,
        workspace_revision: 0,
        power: None,
        store: armd::recording::EpisodeStore::new(Some(dir.path().to_path_buf())),
        zero_offsets: vec![0.0; 6],
        tcp_offset: [0.0; 3],
        active_tool: Default::default(),
    };
    let replay_err = plan(replay_input).unwrap_err();
    match replay_err {
        ArmError::Rejected { code, message, .. } => {
            assert_eq!(code, "REPLAY_DOF_MISMATCH");
            assert!(message.contains("so_arm100"), "{message}");
        }
        other => panic!("expected REPLAY_DOF_MISMATCH, got: {other:?}"),
    }
}

#[test]
fn test_legacy_metadata_guard_5dof_vs_6dof_7dof() {
    let c_so101 = so_arm101_config(); // 5-DOF
    let c_60 = config_6_plus_0();      // 6-DOF (6+0)
    let c_7d = ArmConfig::rebot_dm();  // 7-DOF

    // Empty list on 5-DOF is valid
    let empty_list = PoseList { poses: vec![], ..Default::default() };
    assert!(armd::poses::validate_list(&empty_list, &c_so101).is_ok());

    // Non-empty pose missing arm_dof:
    let legacy_pose_6 = NamedPose {
        id: String::new(),
        name: "legacy".into(),
        positions: vec![0.0; 6],
        tcp: None,
        saved_at_ms: 10,
        profile_id: None,
        arm_dof: None,
        gripper_joint: None,
        joint_names: None,
        stale: None,
        tool_id: None,
        tool_config_digest: None,
    };
    let legacy_list_6 = PoseList {
        profile_id: None,
        arm_dof: None,
        gripper_joint: None,
        joint_names: None,
        poses: vec![legacy_pose_6.clone()],
    };

    // 1) 5-DOF rejects metadata-less pose
    let err_5d = armd::poses::validate_list(&legacy_list_6, &c_so101).unwrap_err();
    assert!(err_5d.contains("POSE_DOF_MISMATCH") && err_5d.contains("missing arm_dof"), "{err_5d}");

    let mut sup_5d = SupervisorInner::new(c_so101.clone());
    sup_5d.poses.list.poses.push(legacy_pose_6.clone());
    let move_err = sup_5d.move_to_pose("legacy", false, 1.0, None, 0).unwrap_err();
    match move_err {
        ArmError::Rejected { code, message, .. } => {
            assert_eq!(code, "POSE_DOF_MISMATCH");
            assert!(message.contains("missing arm_dof"), "{message}");
        }
        other => panic!("expected POSE_DOF_MISMATCH, got: {other:?}"),
    }

    // 2) 6-DOF (6+0) accepts metadata-less legacy pose
    assert!(armd::poses::validate_list(&legacy_list_6, &c_60).is_ok());

    // 3) 7-DOF accepts metadata-less legacy pose (dof=7)
    let legacy_pose_7 = NamedPose {
        id: String::new(),
        name: "legacy7".into(),
        positions: vec![0.0; 7],
        ..Default::default()
    };
    let legacy_list_7 = PoseList {
        poses: vec![legacy_pose_7],
        ..Default::default()
    };
    assert!(armd::poses::validate_list(&legacy_list_7, &c_7d).is_ok());

    // 4) Replay: metadata-less episode rejected on 5-DOF, accepted on 6+0 and 7-DOF
    let dir = tempfile::tempdir().unwrap();
    let legacy_ep = dir.path().join("legacy_bare.parquet");
    let frames6 = vec![
        RecordFrame { timestamp: 0.0, observation_state: vec![0.0; 6], action: vec![0.0; 6] },
        RecordFrame { timestamp: 0.1, observation_state: vec![0.0; 6], action: vec![0.0; 6] },
    ];
    write_frames_to_parquet(&legacy_ep, &frames6).unwrap();

    let replay_5d = ReplayInput {
        request: ReplayRequest { episode: "legacy_bare".into(), speed: 1.0, gripper: false, tag: None, allow_stale: false },
        path: legacy_ep.clone(),
        profile_id: c_so101.profile_id.clone(),
        dof: 6,
        arm_dof: 5,
        gripper_idx: Some(5),
        joint_names: c_so101.limits.iter().map(|l| l.name.clone()).collect(),
        limits: c_so101.limits.clone(),
        speed_limits: c_so101.limits.clone(),
        ws: None,
        arm: c_so101.arm_model.clone(),
        start: vec![0.0; 6],
        control_hz: 50,
        epoch: 0,
        workspace_revision: 0,
        power: None,
        store: armd::recording::EpisodeStore::new(Some(dir.path().to_path_buf())),
        zero_offsets: vec![0.0; 6],
        tcp_offset: [0.0; 3],
        active_tool: Default::default(),
    };
    let rep_err_5d = plan(replay_5d).unwrap_err();
    match rep_err_5d {
        ArmError::Rejected { code, message, .. } => {
            assert_eq!(code, "REPLAY_DOF_MISMATCH");
            assert!(message.contains("missing arm_dof"), "{message}");
        }
        other => panic!("expected REPLAY_DOF_MISMATCH on 5-DOF, got: {other:?}"),
    }

    // 6-DOF accepts legacy bare replay
    let replay_60 = ReplayInput {
        request: ReplayRequest { episode: "legacy_bare".into(), speed: 1.0, gripper: false, tag: None, allow_stale: false },
        path: legacy_ep,
        profile_id: c_60.profile_id.clone(),
        dof: 6,
        arm_dof: 6,
        gripper_idx: None,
        joint_names: c_60.limits.iter().map(|l| l.name.clone()).collect(),
        limits: c_60.limits.clone(),
        speed_limits: c_60.limits.clone(),
        ws: None,
        arm: c_60.arm_model.clone(),
        start: vec![0.0; 6],
        control_hz: 50,
        epoch: 0,
        workspace_revision: 0,
        power: None,
        store: armd::recording::EpisodeStore::new(Some(dir.path().to_path_buf())),
        zero_offsets: vec![0.0; 6],
        tcp_offset: [0.0; 3],
        active_tool: Default::default(),
    };
    assert!(plan(replay_60).is_ok());
}

#[test]
fn test_parquet_metadata_error_propagation() {
    let dir = tempfile::tempdir().unwrap();

    // 1) Nonexistent file -> 404 EPISODE_NOT_FOUND
    let input_nonexistent = ReplayInput {
        request: ReplayRequest { episode: "nonexistent".into(), speed: 1.0, gripper: false, tag: None, allow_stale: false },
        path: dir.path().join("nonexistent.parquet"),
        profile_id: None,
        dof: 6,
        arm_dof: 6,
        gripper_idx: None,
        joint_names: vec![],
        limits: vec![],
        speed_limits: vec![],
        ws: None,
        arm: ArmConfig::rebot_dm().arm_model,
        start: vec![0.0; 6],
        control_hz: 50,
        epoch: 0,
        workspace_revision: 0,
        power: None,
        store: armd::recording::EpisodeStore::new(Some(dir.path().to_path_buf())),
        zero_offsets: vec![0.0; 6],
        tcp_offset: [0.0; 3],
        active_tool: Default::default(),
    };
    let err = plan(input_nonexistent).unwrap_err();
    match err {
        ArmError::Rejected { code, status, .. } => {
            assert_eq!(code, "EPISODE_NOT_FOUND");
            assert_eq!(status, 404);
        }
        other => panic!("expected 404 EPISODE_NOT_FOUND, got {other:?}"),
    }

    // 2) Corrupt sidecar JSON -> 422 REPLAY_INVALID_EPISODE
    let ep_corrupt_json = dir.path().join("corrupt_sidecar.parquet");
    std::fs::write(&ep_corrupt_json, b"dummy parquet content").unwrap();
    let sidecar_json = dir.path().join("corrupt_sidecar.meta.json");
    std::fs::write(&sidecar_json, b"{not valid json").unwrap();

    let input_corrupt_json = ReplayInput {
        request: ReplayRequest { episode: "corrupt_sidecar".into(), speed: 1.0, gripper: false, tag: None, allow_stale: false },
        path: ep_corrupt_json,
        profile_id: None,
        dof: 6,
        arm_dof: 6,
        gripper_idx: None,
        joint_names: vec![],
        limits: vec![],
        speed_limits: vec![],
        ws: None,
        arm: ArmConfig::rebot_dm().arm_model,
        start: vec![0.0; 6],
        control_hz: 50,
        epoch: 0,
        workspace_revision: 0,
        power: None,
        store: armd::recording::EpisodeStore::new(Some(dir.path().to_path_buf())),
        zero_offsets: vec![0.0; 6],
        tcp_offset: [0.0; 3],
        active_tool: Default::default(),
    };
    let err = plan(input_corrupt_json).unwrap_err();
    match err {
        ArmError::Rejected { code, status, .. } => {
            assert_eq!(code, "REPLAY_INVALID_EPISODE");
            assert_eq!(status, 422);
        }
        other => panic!("expected 422 REPLAY_INVALID_EPISODE, got {other:?}"),
    }

    // 3) Corrupt parquet binary content -> 422 REPLAY_INVALID_EPISODE
    let ep_corrupt_parquet = dir.path().join("corrupt_bin.parquet");
    std::fs::write(&ep_corrupt_parquet, b"PAR1garbage").unwrap();
    let input_corrupt_bin = ReplayInput {
        request: ReplayRequest { episode: "corrupt_bin".into(), speed: 1.0, gripper: false, tag: None, allow_stale: false },
        path: ep_corrupt_parquet,
        profile_id: None,
        dof: 6,
        arm_dof: 6,
        gripper_idx: None,
        joint_names: vec![],
        limits: vec![],
        speed_limits: vec![],
        ws: None,
        arm: ArmConfig::rebot_dm().arm_model,
        start: vec![0.0; 6],
        control_hz: 50,
        epoch: 0,
        workspace_revision: 0,
        power: None,
        store: armd::recording::EpisodeStore::new(Some(dir.path().to_path_buf())),
        zero_offsets: vec![0.0; 6],
        tcp_offset: [0.0; 3],
        active_tool: Default::default(),
    };
    let err = plan(input_corrupt_bin).unwrap_err();
    match err {
        ArmError::Rejected { code, status, .. } => {
            assert_eq!(code, "REPLAY_INVALID_EPISODE");
            assert_eq!(status, 422);
        }
        other => panic!("expected 422 REPLAY_INVALID_EPISODE, got {other:?}"),
    }
}
