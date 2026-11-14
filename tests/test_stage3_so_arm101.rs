//! Acceptance and regression tests for Stage 3A:
//! 5-DOF SO-ARM101 Kinematics, Camera Mount, Cartesian Motion with Auto-Yaw,
//! Grasp/Place Planning, and Profile Validation.

use std::sync::{Arc, Mutex};
use std::time::Duration;
use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
use armd::kinematics::{self, Pose};
use armd::model::{ArmConfig, JointState};
use armd::motion::{self, GoalSpec, MotionError, PlanInput, RelFrame, Speed};
use armd::place::{self, PendingPlace, PlaceRequest};
use armd::supervisor::{ArmError, ArmSupervisor, SupervisorInner};
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
    assert_eq!(c101.urdf_sha256.as_deref(), Some("93d70892729a2f99813787a19dcb63fd5aba03c3a4c9928e1b6f18b5cd0ecba7"));
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
        speed_scale: 0.5,
        release: true,
        retreat: true,
        tag: None,
    };
    let pending_true = PendingPlace {
        input: input.clone(),
        request: req_true,
        open_rad: config.gripper_policy.as_ref().unwrap().open_rad,
        held_spill_safe: false,
        spill_max_tilt_deg: 8.0,
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
        speed_scale: 0.5,
        release: true,
        retreat: true,
        tag: None,
    };
    let pending_false = PendingPlace {
        input: input.clone(),
        request: req_false,
        open_rad: config.gripper_policy.as_ref().unwrap().open_rad,
        held_spill_safe: false,
        spill_max_tilt_deg: 8.0,
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
    let r_target = kinematics::rpy_to_r(0.0, 0.5, 0.0);
    let (_strict_goal, strict_yaw_free) = motion::resolve_goal_ex(&input, &GoalSpec::Absolute {
        p: [0.20, 0.08, 0.15],
        r: Some(r_target),
    });
    assert!(!strict_yaw_free, "GoalSpec::Absolute with explicit r must have yaw_free == false");

    // 1. Move to (0.20, 0.08, 0.15) with rpy=(0.0, 0.5, 0.0) and auto_yaw: true
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
    let cross_target_r = kinematics::rpy_to_r(0.0, 0.5, 0.0);
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
        speed_scale: 0.5,
        release: true,
        retreat: true,
        tag: None,
    };

    let pending = PendingPlace {
        input,
        request: req,
        open_rad: config.gripper_policy.as_ref().unwrap().open_rad,
        held_spill_safe: false,
        spill_max_tilt_deg: 8.0,
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
        speed_scale: 1.0,
        release: true,
        retreat: true,
        tag: None,
    };

    let pending = PendingPlace {
        input,
        request: req,
        open_rad: config.gripper_policy.as_ref().unwrap().open_rad,
        held_spill_safe: false,
        spill_max_tilt_deg: 8.0,
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
        inner.set_torque(true).unwrap();
    }

    let ticker = sup.clone();
    let ticker_task = tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(10));
        loop {
            interval.tick().await;
            ticker.tick(0.01).await;
        }
    });

    // Lateral target with y != 0 requiring coupled yaw
    let body = serde_json::json!({
        "position": [0.20, 0.06, 0.03],
        "approach_height_m": 0.06,
        "release": true,
        "retreat": true,
    });
    let (id, _) = place::start(&sup, &body).await.expect("place::start should succeed");

    let deadline = std::time::Instant::now() + Duration::from_secs(15);
    let mut seen_phases = Vec::new();
    let a = loop {
        assert!(std::time::Instant::now() < deadline, "place execution timed out");
        let obs = sup.get_observation().await;
        if let Some(act) = obs.action.as_ref().or(obs.previous_action.as_ref()).filter(|a| a.id == id) {
            if let Some(ref p) = act.phase {
                if seen_phases.last() != Some(p) {
                    seen_phases.push(p.clone());
                }
            }
            if !act.is_running() {
                break act.clone();
            }
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    };

    ticker_task.abort();
    assert_eq!(a.state, "done", "Place should succeed without IK_UNREACHABLE error, got status: {:?}", a);
    assert!(seen_phases.contains(&"approach".to_string()), "approach phase seen");
    assert!(seen_phases.contains(&"descend".to_string()), "descend phase seen");
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
        inner.set_torque(true).unwrap();
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
    inner.set_torque(true).unwrap();
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
    inner.set_torque(true).unwrap();
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
    supervisor.set_torque(true).await.unwrap();

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
    let config = load_so101_config();
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
    inner.set_torque(true).unwrap();
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
    supervisor.set_torque(true).await.unwrap();

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
    };
    let dummy_det = grasp::Detection {
        label: "cup".into(),
        score: 0.9,
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
        speed_scale: 0.5,
        release: true,
        retreat: true,
        tag: None,
    };
    let pending = PendingPlace {
        input,
        request: req,
        open_rad: config.gripper_policy.as_ref().unwrap().open_rad,
        held_spill_safe: false,
        spill_max_tilt_deg: 8.0,
    };
    let planned = place::plan(pending).expect("place::plan must succeed cleanly without pi flip");
    assert!(planned.durations.iter().any(|&d| d > 0.0));
}
