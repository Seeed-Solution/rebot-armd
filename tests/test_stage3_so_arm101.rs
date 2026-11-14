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
use armd::supervisor::ArmSupervisor;
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
    };

    let (goal, yaw_free) = motion::resolve_goal_ex(&input, &spec);
    assert!(yaw_free, "5-DOF translation-only relative move must enable auto_yaw (yaw_free=true)");
    let path = motion::plan_cartesian_ex(&input, goal, speed, yaw_free)
        .expect("lateral move_relative on 5-DOF SO-101 with auto_yaw should succeed");
    assert_eq!(path.dof, 6);
    assert!(path.q.len() >= 2);
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
