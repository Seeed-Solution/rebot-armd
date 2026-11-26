//! Acceptance and regression tests for Stage 3A:
//! 5-DOF SO-ARM101 Kinematics, Camera Mount, Cartesian Motion with Auto-Yaw,
//! Grasp/Place Planning, and Profile Validation.

use std::sync::{Arc, Mutex};
use std::time::Duration;
use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, FeetechArmDriver, JointTarget};
use armd::protocol::feetech::{self, calculate_checksum};
use armd::kinematics::{self, Pose};
use armd::model::{ArmConfig, JointState};
use armd::motion::{self, GoalSpec, MotionError, PlanInput, RelFrame, Speed};
use armd::place::{self, PendingPlace, PlaceRequest};
use armd::safety::{FaultCode, SafetyState};
use armd::supervisor::{ArmError, ArmSupervisor, SupervisorInner};
use armd::transport::mock::MockTransport;
use armd::transport::Transport;
use armd::grasp;

fn load_so101_config() -> ArmConfig {
    let mut config = ArmConfig::from_profile_file("profiles/so_arm101.json")
        .expect("so_arm101.json profile must load");
    config.validate().expect("so_arm101.json profile must validate");
    config
}

fn load_so100_config() -> ArmConfig {
    let mut config = ArmConfig::from_profile_file("profiles/so_arm100.json")
        .expect("so_arm100.json profile must load");
    config.validate().expect("so_arm100.json profile must validate");
    config
}


#[test]
fn test_profiles_load_and_validate() {
    let c101 = load_so101_config();
    assert_eq!(c101.profile_id.as_deref(), Some("so_arm101"));
    assert_eq!(c101.dof, 6);
    assert_eq!(c101.gripper_joint, Some(5));
    assert_eq!(c101.arm_dof(), 5);
    assert_eq!(c101.urdf_sha256.as_deref(), Some("a8c94aaff62965ec58dd33f91eb3b49e102f28870b9a11750de938e55fe1afa4"));
    assert!(c101.kinematics.is_some());
    assert!(c101.workspace.is_some());
    assert!(c101.gripper_policy.is_some());
    assert!(c101.grasp.is_some());
    assert!(c101.feetech.is_some());

    let c100 = load_so100_config();
    assert_eq!(c100.profile_id.as_deref(), Some("so_arm100"));
    assert_eq!(c100.dof, 6);
    assert_eq!(c100.gripper_joint, Some(5));
    assert_eq!(c100.arm_dof(), 5);
}

#[test]
fn test_fk_and_camera_optical_axis() {
    let config = load_so101_config();
    let model = &config.arm_model;

    // 1. Check FK at zero positions
    let q_zero = [0.0; 6];
    let fk_zero = model.fk(&q_zero);
    assert!(fk_zero.p[0].is_finite());
    assert!(fk_zero.p[1].is_finite());
    assert!(fk_zero.p[2].is_finite());

    // 2. Camera mount optical axis in canonical end_link basis:
    // With tilt_deg = -34.9454, flip = false, rot_deg = 0.0:
    // d_end = [cos(tilt), 0, sin(tilt)] = [0.819698, 0.0, -0.572796]
    let rec = model.cam_mount(
        config.camera_mount.flip,
        f64::from(config.camera_mount.rot_deg),
        f64::from(config.camera_mount.tilt_deg),
    );
    let optical_axis = kinematics::column(&rec, 2);
    let expected = [0.819698, 0.0, -0.572796];
    for i in 0..3 {
        assert!(
            (optical_axis[i] - expected[i]).abs() < 1e-4,
            "optical_axis[{i}] was {}, expected {}",
            optical_axis[i],
            expected[i]
        );
    }
    let norm = (optical_axis[0] * optical_axis[0] + optical_axis[1] * optical_axis[1] + optical_axis[2] * optical_axis[2]).sqrt();
    assert!((norm - 1.0).abs() < 1e-6);

    // 3. Home position FK is in forward reachable workspace
    let q_home = [
        f64::from(config.home_positions[0]),
        f64::from(config.home_positions[1]),
        f64::from(config.home_positions[2]),
        f64::from(config.home_positions[3]),
        f64::from(config.home_positions[4]),
        f64::from(config.home_positions[5]),
    ];
    let fk_home = model.fk(&q_home);
    assert!(fk_home.p[0] > 0.15 && fk_home.p[0] < 0.35, "home p_x: {}", fk_home.p[0]);
    assert!(fk_home.p[2] > 0.05 && fk_home.p[2] < 0.30, "home p_z: {}", fk_home.p[2]);
}

fn plan_input_at_home(config: &ArmConfig) -> PlanInput {
    let q0: [f64; 6] = std::array::from_fn(|i| f64::from(config.home_positions[i]));
    PlanInput {
        epoch: 1,
        workspace_revision: 0,
        q0,
        j7: config.home_positions.get(5).copied().unwrap_or(0.0),
        limits: config.limits.clone(),
        speed_cap: None,
        arm: config.arm_model.clone(),
        ws: config.workspace.clone(),
        cam: config.camera_mount.clone(),
        power: None,
        static_restricted: false,
        tcp_offset: config.tcp_offset,
        goal_frame: armd::motion::GoalFrame::EndLink,
    }
}

#[test]
fn test_5dof_move_to_auto_yaw_and_strict_yaw() {
    let config = load_so101_config();
    let input = plan_input_at_home(&config);
    let speed = Speed::default();
    let start = input.arm.fk(&input.q0);
    println!("START P: {:?}", start.p);

    // 1. Omitted orientation: auto_yaw kicks in on 5-DOF
    let spec_auto = GoalSpec::Absolute {
        p: [0.22, 0.05, 0.12],
        r: None,
    };
    let (goal_auto, yaw_free_auto) = motion::resolve_goal_ex(&input, &spec_auto);
    assert!(yaw_free_auto, "5-DOF omitted orientation must set yaw_free = true");
    let path_auto = motion::plan_cartesian_ex(&input, goal_auto, speed, yaw_free_auto)
        .expect("move_to with auto_yaw should succeed");
    assert_eq!(path_auto.dof, 6);
    assert_eq!(path_auto.last_slice().len(), 6);
    assert!(path_auto.q.len() >= 2);

    // 2. Explicit orientation + auto_yaw: true succeeds
    let rot_pitch = kinematics::mat_mul(&kinematics::axis_r(&[0.0, 1.0, 0.0], 0.1), &start.r);
    let spec_ex_ay = GoalSpec::AbsoluteEx {
        p: [0.25, -0.04, 0.08],
        r: Some(rot_pitch),
        auto_yaw: true,
    };
    let (goal_ex, yaw_free_ex) = motion::resolve_goal_ex(&input, &spec_ex_ay);
    assert!(yaw_free_ex);
    let path_ex = motion::plan_cartesian_ex(&input, goal_ex, speed, yaw_free_ex)
        .expect("move_to with auto_yaw: true should succeed");
    assert_eq!(path_ex.dof, 6);

    // 3. Explicit conflicting orientation with auto_yaw: false must fail with IkUnreachable
    let rot_conflict = kinematics::mat_mul(&kinematics::axis_r(&[0.0, 0.0, 1.0], std::f64::consts::FRAC_PI_2), &rot_pitch);
    let spec_conflict = GoalSpec::AbsoluteEx {
        p: [0.22, -0.04, 0.12],
        // Yaw rotated by 90 deg (pi/2) conflicts with 5-DOF radial plane
        r: Some(rot_conflict),
        auto_yaw: false,
    };
    let (goal_conflict, yaw_free_conflict) = motion::resolve_goal_ex(&input, &spec_conflict);
    assert!(!yaw_free_conflict);
    let err = motion::plan_cartesian_ex(&input, goal_conflict, speed, yaw_free_conflict)
        .expect_err("conflicting yaw with auto_yaw: false must be rejected");
    match err {
        MotionError::IkUnreachable { .. } => {}
        other => panic!("expected IkUnreachable, got {other:?}"),
    }
}

#[test]
fn test_5dof_move_relative_lateral_auto_yaw() {
    let config = load_so101_config();
    let input = plan_input_at_home(&config);
    let speed = Speed::default();

    // Lateral translation with frame: Base, d: [0.0, 0.06, 0.0], zero orientation deltas
    let spec = GoalSpec::Relative {
        frame: RelFrame::Base,
        d: [0.0, 0.06, 0.0],
        dyaw: 0.0,
        dpitch: 0.0,
        droll: 0.0,
        level: false,
        auto_yaw: None,
    };

    let (goal, yaw_free) = motion::resolve_goal_ex(&input, &spec);
    assert!(yaw_free, "5-DOF translation-only relative move must enable auto_yaw (yaw_free=true)");
    let path = motion::plan_cartesian_ex(&input, goal, speed, yaw_free)
        .expect("lateral move_relative on 5-DOF SO-101 with auto_yaw should succeed");
    assert_eq!(path.dof, 6);
    assert!(path.q.len() >= 2);
}

#[test]
fn test_5dof_move_relative_explicit_auto_yaw() {
    let config = load_so101_config();
    let input = plan_input_at_home(&config);
    let speed = Speed::default();

    // 1. Explicit auto_yaw: Some(true) on lateral move succeeds
    let spec_ay_true = GoalSpec::Relative {
        frame: RelFrame::Base,
        d: [0.0, 0.06, 0.0],
        dyaw: 0.0,
        dpitch: 0.0,
        droll: 0.0,
        level: false,
        auto_yaw: Some(true),
    };
    let (goal_true, yaw_free_true) = motion::resolve_goal_ex(&input, &spec_ay_true);
    assert!(yaw_free_true, "auto_yaw: Some(true) must yield yaw_free == true");
    let path_true = motion::plan_cartesian_ex(&input, goal_true, speed, yaw_free_true)
        .expect("move_relative with auto_yaw: true should succeed");
    assert_eq!(path_true.dof, 6);

    // 2. Explicit auto_yaw: Some(false) on lateral move forces yaw_free == false and fails IK
    let spec_ay_false = GoalSpec::Relative {
        frame: RelFrame::Base,
        d: [0.0, 0.06, 0.0],
        dyaw: 0.0,
        dpitch: 0.0,
        droll: 0.0,
        level: false,
        auto_yaw: Some(false),
    };
    let (goal_false, yaw_free_false) = motion::resolve_goal_ex(&input, &spec_ay_false);
    assert!(!yaw_free_false, "auto_yaw: Some(false) must yield yaw_free == false");
    let err = motion::plan_cartesian_ex(&input, goal_false, speed, yaw_free_false)
        .expect_err("lateral move_relative without auto_yaw must fail with IkUnreachable");
    assert!(matches!(err, MotionError::IkUnreachable { .. }));

    // 3. parse_move_relative parses "auto_yaw" boolean
    let body_true = serde_json::json!({
        "frame": "base",
        "d": [0.0, 0.02, 0.0],
        "auto_yaw": true,
    });
    let (parsed_spec_true, _, _) = motion::parse_move_relative(&body_true).unwrap();
    assert_eq!(parsed_spec_true, GoalSpec::Relative {
        frame: RelFrame::Base,
        d: [0.0, 0.02, 0.0],
        dyaw: 0.0,
        dpitch: 0.0,
        droll: 0.0,
        level: false,
        auto_yaw: Some(true),
    });

    let body_false = serde_json::json!({
        "frame": "base",
        "d": [0.0, 0.02, 0.0],
        "auto_yaw": false,
    });
    let (parsed_spec_false, _, _) = motion::parse_move_relative(&body_false).unwrap();
    assert_eq!(parsed_spec_false, GoalSpec::Relative {
        frame: RelFrame::Base,
        d: [0.0, 0.02, 0.0],
        dyaw: 0.0,
        dpitch: 0.0,
        droll: 0.0,
        level: false,
        auto_yaw: Some(false),
    });
}

#[test]
fn test_5dof_place_explicit_auto_yaw() {
    let config = load_so101_config();
    let input = plan_input_at_home(&config);
    let r_target = kinematics::rpy_to_r(0.0, 0.5, 0.0);

    // 1. Explicit orientation + auto_yaw: Some(true) aligns to target azimuth and plans cleanly
    let req_true = PlaceRequest {
        target: [0.20, 0.08, 0.10],
        approach_m: 0.05,
        pose: None,
        orientation: Some(r_target),
        auto_yaw: Some(true),
        yaw_outward: false,
        speed_scale: 0.5,
        release: true,
        retreat: true,
        tag: None,
        approach: armd::backlash::Request::Profile,
    };
    let pending_true = PendingPlace {
        input: input.clone(),
        request: req_true,
        open_rad: config.gripper_policy.as_ref().unwrap().open_rad,
        held_spill_safe: false,
        spill_max_tilt_deg: 8.0,
        table_clearance: None,
    };
    let planned_true = place::plan(pending_true).expect("place with explicit auto_yaw: true should succeed");
    assert!(planned_true.durations[1] > 0.0);

    // 2. Explicit orientation + auto_yaw: Some(false) fails IK on coupled 5-DOF
    let req_false = PlaceRequest {
        target: [0.20, 0.08, 0.10],
        approach_m: 0.05,
        pose: None,
        orientation: Some(r_target),
        auto_yaw: Some(false),
        yaw_outward: false,
        speed_scale: 0.5,
        release: true,
        retreat: true,
        tag: None,
        approach: armd::backlash::Request::Profile,
    };
    let pending_false = PendingPlace {
        input: input.clone(),
        request: req_false,
        open_rad: config.gripper_policy.as_ref().unwrap().open_rad,
        held_spill_safe: false,
        spill_max_tilt_deg: 8.0,
        table_clearance: None,
    };
    let res_false = place::plan(pending_false);
    assert!(res_false.is_err(), "place with explicit auto_yaw: false should fail");
    let err_false = res_false.unwrap_err();
    assert!(matches!(err_false, ArmError::Motion(MotionError::IkUnreachable { .. })));

    // 3. parse_place parses "auto_yaw" boolean
    let inner = SupervisorInner::new(config.clone());
    let body_true = serde_json::json!({
        "position": [0.20, 0.05, 0.05],
        "auto_yaw": true,
    });
    let parsed_req_true = place::parse_place(&inner, &body_true).unwrap();
    assert_eq!(parsed_req_true.auto_yaw, Some(true));

    let body_false = serde_json::json!({
        "position": [0.20, 0.05, 0.05],
        "auto_yaw": false,
    });
    let parsed_req_false = place::parse_place(&inner, &body_false).unwrap();
    assert_eq!(parsed_req_false.auto_yaw, Some(false));
}

#[test]
fn test_5dof_place_planning_lateral() {
    let config = load_so101_config();
    let mut input = plan_input_at_home(&config);

    // Verify GoalSpec::Absolute with explicit r resolves with yaw_free == false
    let r_target = kinematics::rpy_to_r(0.0, std::f64::consts::FRAC_PI_4, 0.0);
    let (_strict_goal, strict_yaw_free) = motion::resolve_goal_ex(&input, &GoalSpec::Absolute {
        p: [0.20, 0.08, 0.15],
        r: Some(r_target),
    });
    assert!(!strict_yaw_free, "GoalSpec::Absolute with explicit r must have yaw_free == false");

    // 1. Move to (0.20, 0.08, 0.15) with rpy=(0.0, FRAC_PI_4, 0.0) and auto_yaw: true
    let (goal_pose, yaw_free) = motion::resolve_goal_ex(&input, &GoalSpec::AbsoluteEx {
        p: [0.20, 0.08, 0.15],
        r: Some(r_target),
        auto_yaw: true,
    });
    assert!(yaw_free);
    let path1 = motion::plan_cartesian_ex(&input, goal_pose, Speed { linear: 0.05, angular: 1.0 }, yaw_free).unwrap();
    let last = path1.last();
    input.q0 = std::array::from_fn(|i| f64::from(last[i]));
    println!("after move_to: q0 = {:?}", input.q0);

    // Verify cross-azimuth move from non-zero-azimuth start with explicit r and auto_yaw: true
    let cross_target_r = kinematics::rpy_to_r(0.0, std::f64::consts::FRAC_PI_4, 0.0);
    let (cross_goal, cross_yaw_free) = motion::resolve_goal_ex(&input, &GoalSpec::AbsoluteEx {
        p: [0.20, -0.08, 0.15],
        r: Some(cross_target_r),
        auto_yaw: true,
    });
    assert!(cross_yaw_free);
    let expected_psi = (-0.08_f64).atan2(0.20);
    let actual_psi = cross_goal.r[1][0].atan2(cross_goal.r[0][0]);
    assert!(
        (actual_psi - expected_psi).abs() < 1e-6,
        "tool X azimuth should align to atan2(-0.08, 0.20): got {actual_psi}, expected {expected_psi}"
    );
    let cross_path = motion::plan_cartesian_ex(&input, cross_goal, Speed { linear: 0.05, angular: 1.0 }, cross_yaw_free)
        .expect("cross-azimuth move from non-zero-azimuth start should plan cleanly");
    assert!(cross_path.q.len() >= 2);

    // 2. Move relative dy = -0.04
    let (goal_rel, yaw_free_rel) = motion::resolve_goal_ex(&input, &GoalSpec::Relative {
        frame: RelFrame::Base,
        d: [0.0, -0.04, 0.0],
        dyaw: 0.0,
        dpitch: 0.0,
        droll: 0.0,
        level: false,
        auto_yaw: None,
    });
    let path2 = motion::plan_cartesian_ex(&input, goal_rel, Speed { linear: 0.05, angular: 1.0 }, yaw_free_rel).unwrap();
    let last2 = path2.last();
    input.q0 = std::array::from_fn(|i| f64::from(last2[i]));
    println!("after move_relative: q0 = {:?}", input.q0);

    let req = PlaceRequest {
        target: [0.20, -0.06, 0.10],
        approach_m: 0.05,
        pose: None,
        orientation: None,
        auto_yaw: None,
        yaw_outward: false,
        speed_scale: 0.5,
        release: true,
        retreat: true,
        tag: None,
        approach: armd::backlash::Request::Profile,
    };

    let pending = PendingPlace {
        input,
        request: req,
        open_rad: config.gripper_policy.as_ref().unwrap().open_rad,
        held_spill_safe: false,
        spill_max_tilt_deg: 8.0,
        table_clearance: None,
    };

    let planned = place::plan(pending).expect("lateral place planning on 5-DOF SO-101 should succeed");
    assert!(planned.durations[1] > 0.0, "approach duration > 0");
    assert!(planned.durations[2] > 0.0, "descend duration > 0");
    assert!(planned.durations[3] > 0.0, "retreat duration > 0");
}

#[test]
fn test_5dof_place_planning() {
    let config = load_so101_config();
    let input = plan_input_at_home(&config);

    let req = PlaceRequest {
        target: [0.20, 0.06, 0.03],
        approach_m: 0.06,
        pose: None,
        orientation: None,
        auto_yaw: None,
        yaw_outward: false,
        speed_scale: 1.0,
        release: true,
        retreat: true,
        tag: None,
        approach: armd::backlash::Request::Profile,
    };

    let pending = PendingPlace {
        input,
        request: req,
        open_rad: config.gripper_policy.as_ref().unwrap().open_rad,
        held_spill_safe: false,
        spill_max_tilt_deg: 8.0,
        table_clearance: None,
    };

    let planned = place::plan(pending).expect("place planning on 5-DOF SO-101 should succeed");
    assert!(planned.durations[1] > 0.0, "approach duration > 0");
    assert!(planned.durations[2] > 0.0, "descend duration > 0");
    assert!(planned.durations[3] > 0.0, "retreat duration > 0");
}

#[tokio::test]
async fn test_5dof_place_execution_lateral_target() {
    let config = load_so101_config();
    let sup = Arc::new(ArmSupervisor::new(config));
    {
        let mut inner = sup.inner().write().await;
        inner.lease_enabled = false;
        { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| armd::SupervisorInner::enable_torque(__a)) }.unwrap();
    }

    let ticker = sup.clone();
    let ticker_task = tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(1));
        loop {
            interval.tick().await;
            ticker.tick(0.02).await;
        }
    });

    // Lateral target with y != 0 requiring coupled yaw
    let body = serde_json::json!({
        "position": [0.20, 0.06, 0.03],
        "approach_height_m": 0.06,
        "release": true,
        "retreat": true,
    });
    let mut rx = sup.subscribe_telemetry().ok();
    let (id, _) = place::start(&sup, sup.capture_admission_stamp(), &body).await.expect("place::start should succeed");

    let deadline = std::time::Instant::now() + Duration::from_secs(15);
    let mut seen_phases = Vec::new();
    let a = loop {
        assert!(std::time::Instant::now() < deadline, "place execution timed out");
        let obs = match rx.as_mut() {
            Some(rx) => match tokio::time::timeout(Duration::from_millis(20), rx.recv()).await {
                Ok(Ok(obs)) => obs,
                _ => sup.get_observation().await,
            },
            None => {
                tokio::time::sleep(Duration::from_millis(1)).await;
                sup.get_observation().await
            }
        };
        if let Some(act) = obs.action.as_ref().or(obs.previous_action.as_ref()).filter(|a| a.id == id) {
            if let Some(ref p) = act.phase {
                if seen_phases.last() != Some(p) {
                    seen_phases.push(*p);
                }
            }
            if !act.is_running() {
                break act.clone();
            }
        }
    };

    ticker_task.abort();
    assert_eq!(a.state, "done", "Place should succeed without IK_UNREACHABLE error, got status: {:?}", a);
    assert!(seen_phases.iter().any(|p| p == "approach"), "approach phase seen");
    assert!(seen_phases.iter().any(|p| p == "descend"), "descend phase seen");
}

#[test]
fn test_5dof_grasp_table_violation_rejection() {
    let config = load_so101_config();
    let mut setup = config.grasp.clone().unwrap();
    setup.table_z_m = 0.0;

    let pts = vec![[0.0, 0.0, 0.0], [0.03, 0.0, -0.01]];
    // Direct table violation check on sample poses below table
    let from = Pose::new(kinematics::IDENTITY, [0.22, 0.0, 0.12]);
    let to_violating = Pose::new(kinematics::IDENTITY, [0.22, 0.0, -0.05]);
    let viol = grasp::table_violation(&from, &to_violating, &pts, setup.table_z_m);
    assert!(viol.is_some(), "Path plunging below table must trigger table violation");
}

#[test]
fn test_so_arm_default_grasp_no_workspace_violation() {
    for profile in ["profiles/so_arm101.json", "profiles/so_arm100.json"] {
        let config = ArmConfig::from_profile_file(profile).expect("profile loads");
        let setup = config.grasp.as_ref().expect("grasp setup exists");
        let cam = grasp::CamModel::assumed(&config.arm_model, setup, &config.camera_mount);
        let pts = grasp::envelope(config.workspace.as_ref(), &config.camera_mount, &cam.rec);

        // 1. Check points are all behind or at TCP (x <= 0) in tool frame
        for (i, p) in pts.iter().enumerate() {
            assert!(
                p[0] <= 1e-4,
                "{profile}: envelope point {i} ({:?}) has x > 0 ({}), protruding forward past TCP",
                p, p[0]
            );
        }

        // 2. Test acquisition_z and table_violation at downward grasp
        let r_down = grasp::look_down(&kinematics::IDENTITY);
        let acq_z = grasp::acquisition_z(&r_down, setup, &pts);
        assert!(acq_z.is_finite());
        assert!(acq_z >= setup.table_z_m + setup.approach_height_m);

        let target = [0.22, 0.0, setup.table_z_m];
        let approach = Pose::new(r_down, [target[0], target[1], setup.table_z_m + setup.approach_height_m]);
        let grasp_pose = Pose::new(r_down, [target[0], target[1], setup.table_z_m + setup.grasp_height_m]);
        let viol = grasp::table_violation(&approach, &grasp_pose, &pts, setup.table_z_m);
        assert!(
            viol.is_none(),
            "{profile}: downward grasp path from approach ({:.3}m) to grasp height ({:.3}m) must not violate table: got {:?}",
            approach.p[2], grasp_pose.p[2], viol
        );

        // 3. Test grasp::plan at nominal target
        let mut inner = SupervisorInner::new(config.clone());
        inner.lease_enabled = false;
        { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| armd::SupervisorInner::enable_torque(__a)) }.unwrap();
        let target_nom = [0.22, 0.0, setup.table_z_m + setup.grasp_height_m];
        let r_down = grasp::look_down(&kinematics::IDENTITY);
        let goal = Pose::new(r_down, [target_nom[0], target_nom[1], setup.table_z_m + setup.approach_height_m]);
        let arm = &config.arm_model;
        let lo = std::array::from_fn(|i| f64::from(config.limits[i].min_position));
        let hi = std::array::from_fn(|i| f64::from(config.limits[i].max_position));
        let q_home: [f64; 6] = std::array::from_fn(|i| f64::from(config.home_positions[i]));
        let sol = arm.solve_dls_ex(&goal, &q_home, &lo, &hi, kinematics::PAR.w, true);
        assert!(sol.ok, "{profile}: initial solve for downward approach pose must succeed");

        let q32: Vec<f32> = sol.q.iter().map(|&x| x as f32).collect();
        inner.commanded_positions = q32.clone();
        inner.measured_positions = q32;
        let body = serde_json::json!({
            "label": "object",
            "box_norm": [0.45, 0.45, 0.55, 0.55],
        });
        let pending = grasp::prepare(&inner, &body, inner.safety.epoch_id()).expect("prepare succeeds");
        let planned = grasp::plan(pending);
        assert!(
            planned.is_ok(),
            "{profile}: grasp::plan failed: {:?}",
            planned.err()
        );
        let plan = planned.unwrap();
        assert!(plan.approach_duration > 0.0, "{profile}: approach duration should be > 0");
        assert!(!plan.camera_down, "{profile}: camera_down should be off for 5-DOF SO-ARM");
    }
}

#[test]
fn test_5dof_grasp_off_axis_yaw_free_orchestrator_and_plan() {
    let config = load_so101_config();
    let setup = config.grasp.as_ref().expect("grasp setup exists");
    let _cam = grasp::CamModel::assumed(&config.arm_model, setup, &config.camera_mount);
    let target = [0.20, 0.06, setup.table_z_m + setup.grasp_height_m];

    // 1. Initial downward solve at approach height for off-axis target
    let r_down = grasp::look_down(&kinematics::IDENTITY);
    let goal = Pose::new(r_down, [target[0], target[1], setup.table_z_m + setup.approach_height_m]);
    let arm = &config.arm_model;
    let lo = std::array::from_fn(|i| f64::from(config.limits[i].min_position));
    let hi = std::array::from_fn(|i| f64::from(config.limits[i].max_position));
    let q_home: [f64; 6] = std::array::from_fn(|i| f64::from(config.home_positions[i]));
    let sol = arm.solve_dls_ex(&goal, &q_home, &lo, &hi, kinematics::PAR.w, true);
    assert!(sol.ok, "initial solve for off-axis downward approach pose must succeed");

    // 2. Test grasp::plan feasibility check on off-axis target
    let mut inner = SupervisorInner::new(config.clone());
    inner.lease_enabled = false;
    { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| armd::SupervisorInner::enable_torque(__a)) }.unwrap();
    let q32: Vec<f32> = sol.q.iter().map(|&x| x as f32).collect();
    inner.commanded_positions = q32.clone();
    inner.measured_positions = q32;

    let body = serde_json::json!({
        "label": "object",
        "box_norm": [0.45, 0.45, 0.55, 0.55],
    });
    let pending = grasp::prepare(&inner, &body, inner.safety.epoch_id()).expect("prepare succeeds");
    let planned = grasp::plan(pending).expect("grasp::plan on off-axis 5-DOF target must succeed");
    assert!(planned.approach_duration > 0.0);

    // 3. Test Orchestrator::move_arm mapping for non-vertical pitched motion (e.g. approach / transition / probe)
    // to an off-axis target from home:
    let input_start = plan_input_at_home(&config);
    let r_pitched = kinematics::rpy_to_r(0.0, 0.5, 0.0);
    let pitched_goal = [0.20, 0.08, 0.12];
    let spec = GoalSpec::Absolute { p: pitched_goal, r: Some(r_pitched) };
    let mapped_spec = if input_start.arm.arm_dof() == 5 {
        match spec {
            GoalSpec::Absolute { p, r } => GoalSpec::AbsoluteEx { p, r, auto_yaw: true },
            other => other,
        }
    } else {
        spec
    };
    let (resolved_goal, yaw_free) = motion::resolve_goal_ex(&input_start, &mapped_spec);
    assert!(yaw_free, "5-DOF move_arm mapping must set yaw_free = true");
    let path = motion::plan_cartesian_ex(&input_start, resolved_goal, Speed { linear: setup.approach_speed_mps, angular: motion::DEFAULT_ANGULAR_RPS }, yaw_free);
    assert!(path.is_ok(), "Orchestrator-equivalent move_arm must succeed without IK_UNREACHABLE on off-axis 5-DOF target: got {:?}", path.err());

    // Without auto_yaw (strict orientation on coupled 5-DOF), move to off-axis Y target fails IK
    let (strict_goal, strict_yaw_free) = motion::resolve_goal_ex(&input_start, &GoalSpec::Absolute { p: pitched_goal, r: Some(r_pitched) });
    assert!(!strict_yaw_free);
    let strict_path = motion::plan_cartesian_ex(&input_start, strict_goal, Speed { linear: setup.approach_speed_mps, angular: motion::DEFAULT_ANGULAR_RPS }, strict_yaw_free);
    assert!(strict_path.is_err(), "strict orientation without auto_yaw on off-axis 5-DOF must fail IK");
}

struct MockLoadDriver {
    caps: DriverCapabilities,
    loads: Arc<Mutex<Vec<f32>>>,
    positions: Arc<Mutex<Vec<f32>>>,
}

impl MockLoadDriver {
    fn new(loads: Vec<f32>, positions: Vec<f32>) -> Self {
        Self {
            caps: DriverCapabilities {
                velocity_feedback: true,
                torque_feedback: false,
                temperature_feedback: true,
                torque_feedforward: false,
                enable_feedback: true,
                load_feedback: true,
            },
            loads: Arc::new(Mutex::new(loads)),
            positions: Arc::new(Mutex::new(positions)),
        }
    }
}

impl ArmDriver for MockLoadDriver {
    fn capabilities(&self) -> DriverCapabilities {
        self.caps
    }
    fn enable_state(&self) -> EnableState {
        EnableState::Enabled
    }
    fn is_feedback_valid(&self) -> bool {
        true
    }
    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        let n = self.positions.lock().unwrap().len();
        vec![Some(Duration::ZERO); n]
    }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let pos = self.positions.lock().unwrap();
        Ok(pos
            .iter()
            .map(|&p| JointState {
                position: p,
                velocity: 0.0,
                torque: 0.0,
                temperature: 25.0,
            })
            .collect())
    }
    fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> {
        Ok(())
    }
    fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> {
        Ok(())
    }
    fn joint_loads(&self) -> Option<Vec<f32>> {
        Some(self.loads.lock().unwrap().clone())
    }
}

#[tokio::test]
async fn test_observation_loads_emitted_only_with_load_feedback() {
    let sup_dm = ArmSupervisor::new(ArmConfig::rebot_dm());
    let obs_dm = sup_dm.get_observation().await;
    assert!(obs_dm.loads.is_none());
    let json_dm = serde_json::to_string(&obs_dm).unwrap();
    assert!(!json_dm.contains("\"loads\""));

    let config = load_so101_config();
    let loads = vec![0.0, 0.12, -0.08, 0.03, 0.0, 0.01];
    let driver = MockLoadDriver::new(loads.clone(), config.home_positions.clone());
    let supervisor = ArmSupervisor::new(config).with_driver(Box::new(driver));
    let obs = supervisor.tick(0.02).await;
    assert_eq!(obs.loads, Some(loads));
    let json_so = serde_json::to_string(&obs).unwrap();
    assert!(json_so.contains("\"loads\""));
}

#[tokio::test]
async fn test_so101_park_uses_folded_resting_pose_and_load_limits() {
    let config = load_so101_config();
    let expected_resting = vec![0.0, -1.65, 1.55, 0.5, 0.0, 0.0];
    assert_eq!(
        config.resting_pose.as_ref().and_then(|p| p.positions.as_ref()),
        Some(&expected_resting)
    );
    assert_ne!(config.resting_positions(), &config.home_positions);
    assert_eq!(config.resting_positions(), &expected_resting[..]);

    // 1. Verify NormalUnload (park) targets resting_positions()
    let mut inner = armd::supervisor::SupervisorInner::new(config.clone());
    { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| armd::SupervisorInner::enable_torque(__a)) }.unwrap();
    inner.commanded_positions = config.home_positions.clone();
    inner.measured_positions = config.home_positions.clone();
    inner.apply_stop_policy(armd::safety::StopPolicy::NormalUnload);
    assert!(inner.pending_normal_unload);
    let traj = inner
        .active_trajectory
        .as_ref()
        .expect("trajectory planned for NormalUnload");
    let quintic = traj.as_quintic().expect("quintic trajectory");
    let target = quintic.sample(quintic.duration).positions;
    assert_eq!(target, config.resting_positions());

    // 2. Load limits check on J2 (index 1 limit 0.25) and J3 (index 2 limit 0.30)
    // resting positions for J1..J5, J6 (gripper) open at 1.2 rad
    let mut at_rest_open_gripper = expected_resting.clone();
    at_rest_open_gripper[5] = 1.2;

    let shared_loads = Arc::new(Mutex::new(vec![0.0; 6]));
    let shared_positions = Arc::new(Mutex::new(at_rest_open_gripper.clone()));
    let driver = MockLoadDriver {
        caps: DriverCapabilities {
            velocity_feedback: true,
            torque_feedback: false,
            temperature_feedback: true,
            torque_feedforward: false,
            enable_feedback: true,
            load_feedback: true,
        },
        loads: shared_loads.clone(),
        positions: shared_positions.clone(),
    };

    let supervisor = ArmSupervisor::new(config.clone()).with_driver(Box::new(driver));
    supervisor.enable_torque(supervisor.capture_admission_stamp()).await.unwrap();

    // Loads within limits -> safe resting pose accepted even with J6 open at 1.2 rad
    *shared_loads.lock().unwrap() = vec![0.0, 0.15, 0.20, 0.0, 0.0, 0.0];
    supervisor.tick(0.02).await;
    assert!(
        supervisor.is_at_safe_resting_pose().await,
        "Arm must accept unload at folded pose even with gripper open at 1.2 rad"
    );

    // J2 load exceeds limit (0.35 > 0.25) -> refuses unload
    *shared_loads.lock().unwrap() = vec![0.0, 0.35, 0.20, 0.0, 0.0, 0.0];
    supervisor.tick(0.02).await;
    assert!(
        !supervisor.is_at_safe_resting_pose().await,
        "Refuse unload when J2 load exceeds limit"
    );

    // J3 load exceeds limit (0.40 > 0.30) -> refuses unload
    *shared_loads.lock().unwrap() = vec![0.0, 0.15, 0.40, 0.0, 0.0, 0.0];
    supervisor.tick(0.02).await;
    assert!(
        !supervisor.is_at_safe_resting_pose().await,
        "Refuse unload when J3 load exceeds limit"
    );
}

#[test]
fn test_5dof_align_auto_yaw_preserves_near_vertical_orientation() {
    let r_near_vert = kinematics::rpy_to_r(0.0, 1.58, 0.0);
    assert!(
        armd::chain::is_tool_vertical(&r_near_vert),
        "rpy=[0.0, 1.58, 0.0] must be identified as vertical tool orientation"
    );
    let goal_p = [0.20, 0.08, 0.15];
    let aligned = motion::align_auto_yaw(&goal_p, r_near_vert);
    // When is_tool_vertical is true, align_auto_yaw returns target_r unchanged
    for r in 0..3 {
        for c in 0..3 {
            assert!(
                (aligned[r][c] - r_near_vert[r][c]).abs() < 1e-9,
                "align_auto_yaw must preserve near-vertical orientation without pi flip"
            );
        }
    }
}

#[test]
fn test_5dof_grasp_plan_from_tilted_scan_to_off_axis_target_no_branch_flip() {
    let mut config = load_so101_config();
    if let Some(ref mut grasp) = config.grasp {
        grasp.acquire_camera_down = armd::grasp::AcquireCameraDown::Auto;
    }
    let setup = config.grasp.as_ref().expect("grasp setup exists");
    let cam = grasp::CamModel::assumed(&config.arm_model, setup, &config.camera_mount);
    let arm = &config.arm_model;

    // Scan pose with tool pitched down ~45 deg (-x_tool[2] in [0.6, 0.85]):
    let q_scan = [0.0633, -1.2361, 0.2976, 1.6581, -1.5221, 0.0];
    let sp = arm.fk(&q_scan);
    let s_az = sp.p[1].atan2(sp.p[0]);
    let xt = kinematics::column(&sp.r, 0);
    let pitch_sin = -xt[2];
    assert!(
        pitch_sin > 0.6 && pitch_sin < 0.85,
        "tool should be pitched down ~45 deg (got pitch_sin = {:.3})",
        pitch_sin
    );

    // Target detection located off-axis:
    let box_norm = [0.80, 0.40, 0.90, 0.60];
    let centre = cam.to_px(grasp::box_centre(&box_norm));
    let tgt = cam.plane_point(&sp, centre, setup.table_z_m + setup.grasp_height_m).expect("target plane point exists");
    let t_az = tgt[1].atan2(tgt[0]);
    let delta_psi = (t_az - s_az).abs();
    assert!(
        delta_psi > 0.35,
        "test requires off-axis target with |Δψ| > 0.35 rad (got {:.3} rad)",
        delta_psi
    );

    let mut inner = SupervisorInner::new(config.clone());
    inner.lease_enabled = false;
    { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| armd::SupervisorInner::enable_torque(__a)) }.unwrap();
    let q32: Vec<f32> = q_scan.iter().map(|&x| x as f32).collect();
    inner.commanded_positions = q32.clone();
    inner.measured_positions = q32;

    let body = serde_json::json!({
        "label": "object",
        "box_norm": box_norm,
    });
    let pending = grasp::prepare(&inner, &body, inner.safety.epoch_id()).expect("prepare succeeds");
    let planned = grasp::plan(pending).expect("grasp::plan from tilted scan pose to off-axis target must succeed without IK_BRANCH_FLIP");
    assert!(planned.approach_duration > 0.0);
}

#[tokio::test]
async fn test_5dof_orchestrator_probe_lateral_returns_fallback() {
    let config = load_so101_config();
    let setup = config.grasp.as_ref().expect("grasp setup exists").clone();
    let target = [0.20, 0.06, setup.table_z_m + setup.grasp_height_m];
    let r_down = grasp::look_down(&kinematics::IDENTITY);
    let goal = Pose::new(r_down, [target[0], target[1], setup.table_z_m + setup.approach_height_m]);
    let arm = &config.arm_model;
    let lo = std::array::from_fn(|i| f64::from(config.limits[i].min_position));
    let hi = std::array::from_fn(|i| f64::from(config.limits[i].max_position));
    let q_home: [f64; 6] = std::array::from_fn(|i| f64::from(config.home_positions[i]));
    let sol = arm.solve_dls_ex(&goal, &q_home, &lo, &hi, kinematics::PAR.w, true);
    assert!(sol.ok);

    let driver = MockLoadDriver {
        caps: DriverCapabilities {
            velocity_feedback: true,
            torque_feedback: false,
            temperature_feedback: true,
            torque_feedforward: false,
            enable_feedback: true,
            load_feedback: false,
        },
        loads: std::sync::Arc::new(std::sync::Mutex::new(vec![0.0; 6])),
        positions: std::sync::Arc::new(std::sync::Mutex::new(sol.q.iter().map(|&x| x as f32).collect())),
    };

    let supervisor = std::sync::Arc::new(ArmSupervisor::new(config.clone()).with_driver(Box::new(driver)));
    supervisor.enable_torque(supervisor.capture_admission_stamp()).await.unwrap();

    struct DummyDetectionSource;
    impl grasp::DetectionSource for DummyDetectionSource {
        fn inference_enabled(&self) -> grasp::BoxFuture<'_, Result<bool, String>> { Box::pin(async { Ok(true) }) }
        fn set_inference(&self, _enabled: bool) -> grasp::BoxFuture<'_, Result<(), String>> { Box::pin(async { Ok(()) }) }
        fn next_frame(&self, _after: Option<u64>, _wait_ms: u64) -> grasp::BoxFuture<'_, Result<Option<grasp::DetectionFrame>, String>> { Box::pin(async { Ok(None) }) }
    }

    let cam = grasp::CamModel::assumed(&config.arm_model, &setup, &config.camera_mount);
    let ctx = armd::macro_action::MacroCtx {
        sup: supervisor.clone(),
        id: 1,
        stamp: supervisor.capture_admission_stamp(),
        deadline: std::time::Instant::now() + std::time::Duration::from_secs(10),
        phase_timeout_s: setup.phase_timeout_s,
        timeout_code: "GRASP_TIMEOUT",
        timeout_message: "timeout".into(),
    };
    let orch = grasp::Orchestrator {
        ctx,
        source: Arc::new(DummyDetectionSource),
        setup: setup.clone(),
        cam,
        envelope: vec![],
        triggered: false,
        lens_known: false,
        depth_available: false,
        generation: None,
        tcp_offset: [0.0; 3],
    };
    let dummy_det = grasp::Detection {
        label: "cup".into(),
        score: 0.9,
        raw_score_valid: true,
        box_norm: [0.4, 0.4, 0.6, 0.6],
        angle_deg: None,
        angle_conf: None,
        grasp_width_px: None,
        mask_focus: false,
        disp_fg: None,
        disp_bg: None,
        depth_conf: None,
        length_px: None,
        jaw_clear: None,
        depth_angle_deg: None,
        depth_angle_conf: None,
        depth_width_px: None, contour_norm: None,
    };
    let c = grasp::Centred {
        last: dummy_det.clone(),
        frames: vec![dummy_det],
        pose: goal,
        iterations: 1,
    };
    let mut after = None;
    let est = [0.20, 0.06, 0.0];
    let lat_res = orch.probe_lateral(&c, &est, "cup", &mut after).await.unwrap();
    assert!(
        matches!(lat_res, grasp::Lateral::Fallback("unreachable")),
        "5-DOF probe_lateral must immediately return Lateral::Fallback"
    );
}

#[test]
fn test_5dof_place_plan_slightly_past_vertical_pose_succeeds_cleanly() {
    let config = load_so101_config();
    let arm = &config.arm_model;

    // Start pose with orientation slightly past vertical (pitch = 1.58 rad > pi/2)
    let r_start = kinematics::rpy_to_r(0.0, 1.58, 0.0);
    assert!(
        armd::chain::is_tool_vertical(&r_start),
        "r_start must be identified as vertical tool orientation"
    );
    let p_start = [0.22, 0.0, 0.08];
    let goal_start = Pose::new(r_start, p_start);
    let lo = std::array::from_fn(|i| f64::from(config.limits[i].min_position));
    let hi = std::array::from_fn(|i| f64::from(config.limits[i].max_position));
    let q_home: [f64; 6] = std::array::from_fn(|i| f64::from(config.home_positions[i]));
    let sol = arm.solve_dls_ex(&goal_start, &q_home, &lo, &hi, kinematics::PAR.w, false);
    assert!(sol.ok);

    let mut input = plan_input_at_home(&config);
    input.q0 = sol.q;

    // Place request without explicit orientation inheriting slightly past-vertical start pose
    let req = PlaceRequest {
        target: [0.20, 0.06, 0.02],
        approach_m: 0.05,
        pose: None,
        orientation: None,
        auto_yaw: Some(true),
        yaw_outward: false,
        speed_scale: 0.5,
        release: true,
        retreat: true,
        tag: None,
        approach: armd::backlash::Request::Profile,
    };
    let pending = PendingPlace {
        input,
        request: req,
        open_rad: config.gripper_policy.as_ref().unwrap().open_rad,
        held_spill_safe: false,
        spill_max_tilt_deg: 8.0,
        table_clearance: None,
    };
    let planned = place::plan(pending).expect("place::plan must succeed cleanly without pi flip");
    assert!(planned.durations.iter().any(|&d| d > 0.0));
}

#[test]
fn test_auto_yaw_bypassed_when_tool_is_vertical() {
    let config = load_so101_config();
    let arm = config.arm_model.chain().unwrap();

    let lo: [f64; 6] = std::array::from_fn(|i| f64::from(config.limits[i].min_position));
    let hi: [f64; 6] = std::array::from_fn(|i| f64::from(config.limits[i].max_position));
    let q_home: [f64; 6] = std::array::from_fn(|i| f64::from(config.home_positions[i]));

    // Vertical tool pose (pointing straight down): pitch ~ 90 deg (FRAC_PI_2)
    let r_vertical = kinematics::rpy_to_r(0.0, std::f64::consts::FRAC_PI_2, 0.0);
    assert!(armd::chain::is_tool_vertical(&r_vertical));

    let goal_vert = Pose::new(r_vertical, [0.22, 0.0, 0.08]);

    // Step 1: Solve with yaw_free = false from q_home to reach initial downward pose
    let sol_down = arm.solve_dls_ex(&goal_vert, &q_home, &lo, &hi, kinematics::PAR.w, 100, false);
    assert!(sol_down.ok, "Must reach initial downward pose");
    let fk_down = arm.fk(&sol_down.q);
    assert!(armd::chain::is_tool_vertical(&fk_down.r));

    // Step 2: Rotate downward orientation by 0.7 rad around world Z (which is the tool X axis when vertical)
    let r_rolled = kinematics::mat_mul(&kinematics::axis_r(&[0.0, 0.0, 1.0], 0.7), &r_vertical);
    assert!(armd::chain::is_tool_vertical(&r_rolled));

    let goal_rolled = Pose::new(r_rolled, [0.22, 0.0, 0.08]);

    // Solve with yaw_free = true (auto_yaw requested):
    // Because tool is vertical, auto-yaw must be bypassed, preserving wrist roll!
    let sol = arm.solve_dls_ex(&goal_rolled, &sol_down.q, &lo, &hi, kinematics::PAR.w, 100, true);
    assert!(sol.ok, "IK must solve cleanly");

    let fk = arm.fk(&sol.q);
    assert!(armd::chain::is_tool_vertical(&fk.r));
    let rot_err = kinematics::angle_between(&fk.r, &r_rolled);
    assert!(rot_err < 0.02, "Wrist roll must be executed, not discarded: rot_err = {rot_err}");
}

#[test]
fn test_feetech_model_validation_rejects_invalid_parameters() {
    let mut config = load_so101_config();
    assert!(config.validate().is_ok());

    // 1. max_torque_raw == 0 rejected
    let mut bad_cfg = config.clone();
    bad_cfg.feetech.as_mut().unwrap().max_torque_raw = 0;
    assert!(bad_cfg.validate().is_err());

    // 2. max_torque_raw > 1000 rejected
    let mut bad_cfg = config.clone();
    bad_cfg.feetech.as_mut().unwrap().max_torque_raw = 1001;
    assert!(bad_cfg.validate().is_err());

    // 3. protection_current_raw == 0 rejected
    let mut bad_cfg = config.clone();
    bad_cfg.feetech.as_mut().unwrap().protection_current_raw = 0;
    assert!(bad_cfg.validate().is_err());

    // 4. protection_current_raw > 1000 rejected
    let mut bad_cfg = config.clone();
    bad_cfg.feetech.as_mut().unwrap().protection_current_raw = 1001;
    assert!(bad_cfg.validate().is_err());

    // 5. joint overload_torque_raw > 100 rejected
    let mut bad_cfg = config.clone();
    bad_cfg.feetech.as_mut().unwrap().joints[0].overload_torque_raw = Some(101);
    assert!(bad_cfg.validate().is_err());

    // 6. Arm joint overload_torque_raw < 50 rejected
    let mut bad_cfg = config.clone();
    bad_cfg.feetech.as_mut().unwrap().joints[0].overload_torque_raw = Some(49);
    assert!(bad_cfg.validate().is_err());

    // 7. Gripper joint (index 5) overload_torque_raw allowed < 50 (e.g. 25 on so-arm100)
    let mut gripper_cfg = config.clone();
    gripper_cfg.feetech.as_mut().unwrap().joints[5].overload_torque_raw = Some(25);
    assert!(gripper_cfg.validate().is_ok());

    // 8. Global feetech.overload_torque_raw < 50 rejected when arm joints inherit it (overload_torque_raw = None)
    let mut bad_global_cfg = config.clone();
    bad_global_cfg.feetech.as_mut().unwrap().overload_torque_raw = Some(25);
    bad_global_cfg.feetech.as_mut().unwrap().joints[0].overload_torque_raw = None;
    assert!(bad_global_cfg.validate().is_err());
}

#[test]
fn test_feetech_homing_offset_steps_bounds_check() {
    let mut config = load_so101_config();
    assert!(config.validate().is_ok());

    // Test homing_offset_steps
    for (val, should_be_ok) in [
        (i16::MIN, false),
        (-2048, false),
        (-2047, true),
        (0, true),
        (2047, true),
        (2048, false),
    ] {
        let mut cfg = config.clone();
        cfg.feetech.as_mut().unwrap().joints[0].homing_offset_steps = val;
        let res = cfg.validate();
        assert_eq!(
            res.is_ok(),
            should_be_ok,
            "homing_offset_steps = {} expected is_ok == {}, got {:?}",
            val,
            should_be_ok,
            res
        );
    }

    // Test expected_homing_offset_steps
    for (val, should_be_ok) in [
        (Some(i16::MIN), false),
        (Some(-2048), false),
        (Some(-2047), true),
        (Some(0), true),
        (Some(2047), true),
        (Some(2048), false),
    ] {
        let mut cfg = config.clone();
        cfg.feetech.as_mut().unwrap().joints[0].expected_homing_offset_steps = val;
        let res = cfg.validate();
        assert_eq!(
            res.is_ok(),
            should_be_ok,
            "expected_homing_offset_steps = {:?} expected is_ok == {}, got {:?}",
            val,
            should_be_ok,
            res
        );
    }
}

#[test]
fn test_vertical_pose_translation_preserves_roll_for_move_to_relative_and_place() {
    let config = load_so101_config();
    let arm = &config.arm_model;
    let lo: [f64; 6] = std::array::from_fn(|i| f64::from(config.limits[i].min_position));
    let hi: [f64; 6] = std::array::from_fn(|i| f64::from(config.limits[i].max_position));
    let q_home: [f64; 6] = std::array::from_fn(|i| f64::from(config.home_positions[i]));

    // 1. Initial vertical pose
    let r_vertical = kinematics::rpy_to_r(0.0, std::f64::consts::FRAC_PI_2, 0.0);
    assert!(armd::chain::is_tool_vertical(&r_vertical));
    let sol_down = arm.solve_dls_ex(&Pose::new(r_vertical, [0.22, 0.0, 0.08]), &q_home, &lo, &hi, kinematics::PAR.w, false);
    assert!(sol_down.ok);

    let mut input = plan_input_at_home(&config);
    input.q0 = sol_down.q;
    let start = input.arm.fk(&input.q0);
    assert!(armd::chain::is_tool_vertical(&start.r));

    // A. GoalSpec::Absolute (move_to without explicit orientation, auto_yaw kicks in)
    let spec_abs = GoalSpec::Absolute {
        p: [0.20, 0.05, 0.08],
        r: None,
    };
    let (goal_abs, _yaw_free_abs) = motion::resolve_goal_ex(&input, &spec_abs);
    let rot_diff_abs = kinematics::angle_between(&goal_abs.r, &start.r);
    assert!(rot_diff_abs < 1e-4, "move_to from vertical pose must keep initial orientation, got rot_diff = {rot_diff_abs}");

    // B. GoalSpec::Relative (move_relative translation-only in Base frame)
    let spec_rel = GoalSpec::Relative {
        frame: RelFrame::Base,
        d: [-0.02, 0.05, 0.0],
        dyaw: 0.0,
        dpitch: 0.0,
        droll: 0.0,
        level: false,
        auto_yaw: None,
    };
    let (goal_rel, _yaw_free_rel) = motion::resolve_goal_ex(&input, &spec_rel);
    let rot_diff_rel = kinematics::angle_between(&goal_rel.r, &start.r);
    assert!(rot_diff_rel < 1e-4, "move_relative from vertical pose must keep initial orientation, got rot_diff = {rot_diff_rel}");

    // C. place::plan starting from vertical pose without explicit orientation
    let req = PlaceRequest {
        target: [0.20, 0.05, 0.02],
        approach_m: 0.05,
        pose: None,
        orientation: None,
        auto_yaw: Some(true),
        yaw_outward: false,
        speed_scale: 0.3,
        release: true,
        retreat: true,
        tag: None,
        approach: armd::backlash::Request::Profile,
    };
    let pending = PendingPlace {
        input,
        request: req,
        open_rad: config.gripper_policy.as_ref().unwrap().open_rad,
        held_spill_safe: false,
        spill_max_tilt_deg: 8.0,
        table_clearance: None,
    };
    let planned = place::plan(pending).expect("place::plan must succeed from vertical pose");
    assert!(planned.durations.iter().any(|&d| d > 0.0));
}

#[test]
fn test_cross_threshold_cartesian_slerp_tilted_to_vertical() {
    let config = load_so101_config();
    let arm = &config.arm_model;
    let lo: [f64; 6] = std::array::from_fn(|i| f64::from(config.limits[i].min_position));
    let hi: [f64; 6] = std::array::from_fn(|i| f64::from(config.limits[i].max_position));
    let q_home: [f64; 6] = std::array::from_fn(|i| f64::from(config.home_positions[i]));

    // Start pose: tilted at pitch = 45 deg
    let r_tilted = kinematics::rpy_to_r(0.0, std::f64::consts::FRAC_PI_4, 0.0);
    let p_start = [0.22, 0.0, 0.12];
    let sol_start = arm.solve_dls_ex(&Pose::new(r_tilted, p_start), &q_home, &lo, &hi, kinematics::PAR.w, true);
    assert!(sol_start.ok, "Start pose at 45 deg pitch must solve cleanly");

    let mut input = plan_input_at_home(&config);
    input.q0 = sol_start.q;

    // End pose: vertical at pitch = 90 deg (FRAC_PI_2)
    let r_vert = kinematics::rpy_to_r(0.0, std::f64::consts::FRAC_PI_2, 0.0);
    let p_end = [0.22, 0.0, 0.08];
    let goal_vert = Pose::new(r_vert, p_end);

    let speed = Speed { linear: 0.05, angular: 1.0 };
    let path = motion::plan_cartesian_ex(&input, goal_vert, speed, true)
        .expect("Cartesian slerp from tilted to vertical must succeed without IkBranchFlip");
    assert!(path.q.len() >= 2);
}

#[test]
fn test_so_arm101_holding_load_fraction_validation() {
    let base_cfg = load_so101_config();

    // 1. NaN is rejected
    let mut cfg_nan = base_cfg.clone();
    cfg_nan.gripper_policy.as_mut().unwrap().holding_load_fraction = Some(f32::NAN);
    assert!(cfg_nan.validate().is_err(), "NaN holding_load_fraction must be rejected");

    // 2. < 0 is rejected
    let mut cfg_neg = base_cfg.clone();
    cfg_neg.gripper_policy.as_mut().unwrap().holding_load_fraction = Some(-0.01);
    assert!(cfg_neg.validate().is_err(), "< 0 holding_load_fraction must be rejected");

    // 3. > 1 is rejected
    let mut cfg_gt1 = base_cfg.clone();
    cfg_gt1.gripper_policy.as_mut().unwrap().holding_load_fraction = Some(1.05);
    assert!(cfg_gt1.validate().is_err(), "> 1 holding_load_fraction must be rejected");

    // 4. 0.0 with LoadAndGap is rejected
    let mut cfg_zero = base_cfg.clone();
    cfg_zero.gripper_policy.as_mut().unwrap().holding_mode = armd::model::HoldingMode::LoadAndGap;
    cfg_zero.gripper_policy.as_mut().unwrap().holding_load_fraction = Some(0.0);
    assert!(cfg_zero.validate().is_err(), "0.0 holding_load_fraction with LoadAndGap must be rejected");

    // 5. Valid holding_load_fraction (0.18) is accepted
    let mut cfg_valid = base_cfg.clone();
    cfg_valid.gripper_policy.as_mut().unwrap().holding_mode = armd::model::HoldingMode::LoadAndGap;
    cfg_valid.gripper_policy.as_mut().unwrap().holding_load_fraction = Some(0.18);
    assert!(cfg_valid.validate().is_ok());
}

#[tokio::test]
async fn test_so_arm101_gripper_stall_at_load_threshold() {
    struct So101LoadDriver {
        loads: Vec<f32>,
    }

    impl ArmDriver for So101LoadDriver {
        fn capabilities(&self) -> DriverCapabilities {
            DriverCapabilities {
                velocity_feedback: true,
                torque_feedback: false,
                temperature_feedback: false,
                torque_feedforward: false,
                enable_feedback: true,
                load_feedback: true,
            }
        }
        fn enable_state(&self) -> EnableState { EnableState::Enabled }
        fn is_feedback_valid(&self) -> bool { true }
        fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); 6] }
        fn joint_loads(&self) -> Option<Vec<f32>> { Some(self.loads.clone()) }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            Ok((0..6).map(|i| JointState {
                position: if i == 5 { 0.2 } else { 0.0 }, // gap: cmd (0.4) - meas (0.2) = 0.2 > 0.08
                velocity: 0.0,
                torque: 0.0,
                temperature: 28.0,
            }).collect())
        }
        fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
        fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
    }

    let config = load_so101_config();
    assert_eq!(config.dof, 6);
    assert_eq!(config.gripper_joint, Some(5));

    let mut loads = vec![0.0f32; 6];
    loads[5] = 0.23;

    let driver = So101LoadDriver { loads };
    let supervisor = ArmSupervisor::new(config).with_driver(Box::new(driver));

    {
        let mut inner = supervisor.inner().write().await;
        inner.is_torqued = true;
        inner.commanded_positions = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.4];
        inner.measured_positions = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.2];
        inner.measured_velocities = vec![0.0; 6];
        inner.loads = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.23];
        inner.last_can_feedback_instant = Some(std::time::Instant::now());
    }

    for _ in 0..10 {
        let _ = supervisor.tick(0.02).await;
    }

    assert!(supervisor.is_gripper_stalled().await, "Gripper must be stalled at |load| = 0.23");
    let obs = supervisor.get_observation().await;
    assert_eq!(obs.gripper.as_ref().map(|g| g.stalled), Some(true));
}

// -------------------------------------------------------------------------
// 26. place::plan with orientation: None preserves start pitch/roll while updating yaw
// -------------------------------------------------------------------------
#[test]
fn test_place_plan_orientation_none_preserves_start_pitch_roll_and_updates_yaw() {
    let config = load_so101_config();
    let mut input = plan_input_at_home(&config);

    // Initial non-vertical orientation with non-zero roll, pitch, yaw
    let start_roll = 0.05;
    let start_pitch = 0.85; // ~48.7 degrees down
    let start_yaw = 0.10;
    let r_start = kinematics::rpy_to_r(start_roll, start_pitch, start_yaw);
    let (start_goal, yaw_free_start) = motion::resolve_goal_ex(&input, &GoalSpec::AbsoluteEx {
        p: [0.20, 0.05, 0.18],
        r: Some(r_start),
        auto_yaw: true,
    });
    let start_path = motion::plan_cartesian_ex(&input, start_goal, Speed { linear: 0.05, angular: 1.0 }, yaw_free_start).unwrap();
    input.q0 = std::array::from_fn(|i| f64::from(start_path.last()[i]));

    let fk_before = input.arm.fk(&input.q0);
    let [roll_before, pitch_before, _] = kinematics::r_to_rpy(&fk_before.r);
    assert!((roll_before - start_roll).abs() < 1e-2);
    assert!((pitch_before - start_pitch).abs() < 1e-2);

    // Place to target at [0.20, -0.06, 0.12] with orientation: None
    let target = [0.20, -0.06, 0.12];
    let req = PlaceRequest {
        target,
        approach_m: 0.05,
        pose: None,
        orientation: None,
        auto_yaw: None,
        yaw_outward: false,
        speed_scale: 0.5,
        release: true,
        retreat: true,
        tag: None,
        approach: armd::backlash::Request::Profile,
    };
    let pending = PendingPlace {
        input,
        request: req,
        open_rad: config.gripper_policy.as_ref().unwrap().open_rad,
        held_spill_safe: false,
        spill_max_tilt_deg: 8.0,
        table_clearance: None,
    };

    let planned = place::plan(pending).expect("place::plan with orientation: None should succeed");
    let [r_plan_roll, r_plan_pitch, r_plan_yaw] = kinematics::r_to_rpy(&planned.r_place);

    // Pitch and roll MUST be preserved from start pose (no forced 45° tilt override!)
    assert!((r_plan_roll - roll_before).abs() < 1e-2, "Roll must be preserved: got {}, expected {}", r_plan_roll, roll_before);
    assert!((r_plan_pitch - pitch_before).abs() < 1e-2, "Pitch must be preserved: got {}, expected {}", r_plan_pitch, pitch_before);

    // Yaw MUST be updated towards target azimuth
    let expected_target_yaw = (-0.06_f64).atan2(0.20);
    assert!((r_plan_yaw - expected_target_yaw).abs() < 0.15, "Yaw must update towards target: got {}, expected ~{}", r_plan_yaw, expected_target_yaw);
}

#[tokio::test]
async fn test_so101_persistent_gripper_hw_overload_reverse_escape_and_re_squeeze() {
    let config = load_so101_config();
    let supervisor = Arc::new(ArmSupervisor::new(config));
    {
        let mut inner = supervisor.inner().write().await;
        inner.lease_enabled = false;
        { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| armd::SupervisorInner::enable_torque(__a)) }.unwrap();
        inner.measured_positions = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.5];
        inner.commanded_positions = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.5];
        inner.target_positions = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.5];
        // Inject motor error 0x20 on gripper (joint 5)
        inner.motor_errors = vec![
            None,
            None,
            None,
            None,
            None,
            Some(armd::model::MotorError {
                code: 0x20,
                name: "Overload".into(),
            }),
        ];
        inner.last_can_feedback_instant = Some(std::time::Instant::now());
    }

    // Tick: HW overload rises, gripper latches stall
    supervisor.tick(0.01).await;
    assert!(supervisor.is_gripper_stalled().await, "SO-ARM101 gripper must latch stall on 0x20");

    // Reverse servo command (opening jaw towards open_rad=1.5)
    let rev_targets = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.8];
    let servo_res = supervisor.servo(supervisor.capture_admission_stamp(), rev_targets, 1).await;
    assert!(servo_res.is_ok(), "Reverse servo command must be accepted");
    assert!(!supervisor.is_gripper_stalled().await, "Reverse servo command un-stalls gripper");

    // Ticks while 0x20 persists: escaping known stall must not re-latch
    for _ in 0..5 {
        {
            let mut inner = supervisor.inner().write().await;
            inner.last_can_feedback_instant = Some(std::time::Instant::now());
        }
        supervisor.tick(0.01).await;
        assert!(!supervisor.is_gripper_stalled().await, "Must not re-latch while escaping known stall");
    }

    // Squeeze command towards closed_rad=0.0
    let squeeze_targets = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.2];
    let sq_res = supervisor.servo(supervisor.capture_admission_stamp(), squeeze_targets, 1).await;
    assert!(sq_res.is_ok(), "Squeeze command accepted");

    // Tick: since 0x20 still asserted and pushing into stall, it must re-latch!
    {
        let mut inner = supervisor.inner().write().await;
        inner.last_can_feedback_instant = Some(std::time::Instant::now());
    }
    supervisor.tick(0.01).await;
    assert!(supervisor.is_gripper_stalled().await, "Must re-latch when pushing into stall with 0x20");
}

#[tokio::test]
async fn test_so101_feetech_driver_backed_arm_joint_0x20_overload_triggers_estop() {
    let config = load_so101_config();

    // Custom MockTransport wrapper for FeetechArmDriver
    struct SharedTransport(Arc<Mutex<MockTransport>>);
    impl Transport for SharedTransport {
        fn clock(&self) -> Option<Arc<dyn armd::transport::DriverClock>> {
            Transport::clock(&*self.0.lock().unwrap())
        }
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, armd::transport::TransportError> {
            self.0.lock().unwrap().read(buf)
        }
        fn write(&mut self, buf: &[u8]) -> Result<usize, armd::transport::TransportError> {
            self.0.lock().unwrap().write(buf)
        }
        fn flush(&mut self) -> Result<(), armd::transport::TransportError> {
            self.0.lock().unwrap().flush()
        }
        fn clear_input_buffer(&mut self) {
            self.0.lock().unwrap().clear_input_buffer();
        }
    }

    let mock = Arc::new(Mutex::new(MockTransport::new()));
    let servo_ids = vec![1, 2, 3, 4, 5, 6];
    let driver = FeetechArmDriver::new(SharedTransport(mock.clone()), servo_ids)
        .with_gripper_index(Some(5))
        .with_calibrated(true);

    let supervisor = ArmSupervisor::new(config).with_driver(Box::new(driver));
    supervisor.inner().write().await.lease_enabled = false;

    let build_resp = |id: u8, err: u8| -> Vec<u8> {
        let pos = 2048u16.to_le_bytes();
        let spd = 0u16.to_le_bytes();
        let load = 0u16.to_le_bytes();
        let params = [pos[0], pos[1], spd[0], spd[1], load[0], load[1], 70, 35];
        let cs = calculate_checksum(id, (params.len() + 2) as u8, err, &params);
        let mut pkt = vec![0xFF, 0xFF, id, (params.len() + 2) as u8, err];
        pkt.extend_from_slice(&params);
        pkt.push(cs);
        pkt
    };

    let sync_read_chunk0 = feetech::build_sync_read(&[1, 2, 3], feetech::REG_PRESENT_POSITION_L, 8).unwrap();
    let sync_read_chunk1 = feetech::build_sync_read(&[4, 5, 6], feetech::REG_PRESENT_POSITION_L, 8).unwrap();
    let sync_read_en = feetech::build_sync_read(&[1, 2, 3, 4, 5, 6], feetech::REG_TORQUE_ENABLE, 1).unwrap();

    let mut rx_c0_normal = Vec::new();
    for id in 1..=3 {
        rx_c0_normal.extend_from_slice(&build_resp(id, 0x00));
    }
    let mut rx_c1_normal = Vec::new();
    for id in 4..=6 {
        rx_c1_normal.extend_from_slice(&build_resp(id, 0x00));
    }
    let mut rx_tq = Vec::new();
    for id in 1..=6 {
        let cs = calculate_checksum(id, 3, 0x00, &[1]);
        rx_tq.extend_from_slice(&[0xFF, 0xFF, id, 3, 0x00, 1, cs]);
    }

    {
        let mut m = mock.lock().unwrap();
        m.stage_reply(&sync_read_chunk0, &rx_c0_normal);
        m.stage_reply(&sync_read_chunk1, &rx_c1_normal);
        m.stage_reply(&sync_read_en, &rx_tq);
        for id in 1..=6 {
            let req_en = feetech::build_read(id, feetech::REG_TORQUE_ENABLE, 1);
            let cs = calculate_checksum(id, 3, 0x00, &[1]);
            let resp_en = vec![0xFF, 0xFF, id, 3, 0x00, 1, cs];
            m.stage_reply(&req_en, &resp_en);
        }
    }

    supervisor.enable_torque(supervisor.capture_admission_stamp()).await.unwrap();
    assert!(supervisor.inner().read().await.is_torqued);

    supervisor.tick(0.02).await;
    let obs = supervisor.get_observation().await;
    assert_eq!(obs.safety_state, SafetyState::Armed);

    // Stale period: ticks pass without new packets
    mock.lock().unwrap().clear_write_responses();
    mock.lock().unwrap().advance_clock(Duration::from_millis(5));
    supervisor.tick(0.02).await;
    let obs_stale = supervisor.get_observation().await;
    assert_eq!(obs_stale.safety_state, SafetyState::Armed);
    assert!(obs_stale.fault.is_none());
    assert_eq!(obs.feedback_age_ms.len(), 6);
    assert_eq!(obs_stale.feedback_age_ms.len(), 6);
    assert!(obs_stale.feedback_age_ms.iter().enumerate().all(|(i, age)| {
        age.zip(obs.feedback_age_ms[i])
            .is_some_and(|(after, before)| after >= before + 5)
    }));

    // Rising edge: Arm joint 1 (servo ID 1) reports 0x20 Overload
    let mut rx_c0_overload = Vec::new();
    rx_c0_overload.extend_from_slice(&build_resp(1, 0x20));
    for id in 2..=3 {
        rx_c0_overload.extend_from_slice(&build_resp(id, 0x00));
    }

    {
        let mut m = mock.lock().unwrap();
        m.clear_write_responses();
        // Wait LINE_IDLE_QUIET (20 ms) after the stale-period SYNC_READ timeout poisoned the line.
        m.advance_clock(Duration::from_millis(20));
        m.stage_reply(&sync_read_chunk0, &rx_c0_overload);
        m.stage_reply(&sync_read_chunk1, &rx_c1_normal);
        m.stage_reply(&sync_read_en, &rx_tq);
    }

    // Tick: Arm joint 0x20 triggers HardwareFault
    let obs_fault = supervisor.tick(0.02).await;
    assert!(obs_fault.fault.is_some(), "Arm joint 0x20 rising edge must fault supervisor");
    let fault = obs_fault.fault.unwrap();
    assert_eq!(fault.code, FaultCode::HardwareFault);
    assert!(fault.message.contains("0x20: Overload"));
}
