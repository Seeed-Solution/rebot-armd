//! Server-side Cartesian safety box (spec 1b).
mod common;

use armd::chain::ArmModel;
use armd::kinematics::{self, Violation};
use armd::model::{ArmConfig, ObservationSchema, WorkspacePolicy};
use armd::safety::SafetyError;
use armd::{ArmError, SupervisorInner};

const HOME: [f32; 7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
const INSPECT: [f32; 7] = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0, 0.0];
// Both ends are above the floor; the joint-space midpoint puts a gripper corner ~27 mm below it.
// J6 = pi/2 (URDF J6 = joint J6): the jaw axis is in the plane where the 92.5 mm corners dip.
const DIP_A: [f32; 7] = [0.0, -1.871, -0.64, 1.817, 0.464, std::f32::consts::FRAC_PI_2, 0.0];
const DIP_B: [f32; 7] = [0.0, -1.856, -1.434, 0.482, -0.61, std::f32::consts::FRAC_PI_2, 0.0];

/// Workspace excess at motor angles `q` (the profile's arm model).
fn workspace_violation(q: &[f32], ws: &WorkspacePolicy, cam: &armd::model::CameraMount) -> Violation {
    ArmModel::REBOT_DEV_ARM.workspace_violation(q, ws, cam)
}

fn total(q: &[f32]) -> f64 {
    let c = ArmConfig::rebot_dm();
    workspace_violation(q, c.workspace.as_ref().unwrap(), &c.camera_mount).total
}

fn lerp(a: &[f32; 7], b: &[f32; 7], s: f32) -> Vec<f32> { a.iter().zip(b).map(|(x, y)| x + (y - x) * s).collect() }

/// Last in-workspace configuration and the next one (0.5 % steps) on the DIP_A -> DIP_B line.
fn floor_edge() -> (Vec<f32>, Vec<f32>) {
    let mut prev = DIP_A.to_vec();
    for k in 1..=200 {
        let q = lerp(&DIP_A, &DIP_B, k as f32 / 400.0);
        if total(&q) > 1e-3 { return (prev, q); }
        prev = q;
    }
    panic!("no floor crossing found");
}

fn armed(config: ArmConfig, at: &[f32]) -> SupervisorInner {
    let mut inner = SupervisorInner::new(config);
    inner.lease_enabled = false;
    { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| inner.enable_torque(__a)) }.unwrap();
    inner.commanded_positions = at.to_vec();
    inner.measured_positions = at.to_vec();
    inner.target_positions = at.to_vec();
    inner
}

#[test]
fn violation_function_reports_floor_box_and_reach() {
    let c = ArmConfig::rebot_dm();
    let (ws, cam) = (c.workspace.as_ref().unwrap(), &c.camera_mount);
    assert_eq!(total(&HOME), 0.0);
    assert_eq!(total(&INSPECT), 0.0);
    assert_eq!(total(&DIP_A), 0.0);
    assert_eq!(total(&DIP_B), 0.0);
    let mid = lerp(&DIP_A, &DIP_B, 0.5);
    let v = workspace_violation(&mid, ws, cam);
    let w = v.worst.unwrap();
    assert_eq!(w.limit, "floor");
    assert!((1..=8).contains(&w.point), "a gripper corner, not the origin: {w:?}");
    assert!(v.total > 0.02, "{v:?}");
        let mut ws_reach = ws.clone();
    ws_reach.max_reach_m = Some(0.20); // home end_link origin is ~0.25 m from the shoulder
    let v = workspace_violation(&HOME, &ws_reach, cam);
    assert_eq!(v.worst.unwrap().limit, "reach");
    // box
    let mut ws_box = WorkspacePolicy::rebot_dm();
    ws_box.box_max_m[0] = 0.10;
    assert!(workspace_violation(&HOME, &ws_box, cam).worst.unwrap().limit.starts_with("box_x_max"));
    // camera point is checked when configured
    let mut cam_low = cam.clone();
    let home_pose = ArmModel::REBOT_DEV_ARM.fk_f32(&HOME).unwrap();
    let down_in_end = kinematics::mat_vec(&kinematics::transpose(&home_pose.r), &[0.0, 0.0, -0.5]);
    cam_low.offset_m = Some(down_in_end.map(|v| v as f32)); // 0.5 m below end_link in the base frame
    let v = workspace_violation(&HOME, ws, &cam_low);
    assert_eq!(v.worst.unwrap().point, 9);
    // per-constraint rule: a pose moving further out is rejected, one moving back is allowed
    let (ok, bad) = floor_edge();
    let eval = armd::chain::WorkspaceEvaluator::new(&ArmModel::REBOT_DEV_ARM, ws, cam);
    let (mut c_ok, mut c_bad) = (eval.alloc_buf(), eval.alloc_buf());
    eval.eval_into(&ok, &mut c_ok, "plan").unwrap();
    eval.eval_into(&bad, &mut c_bad, "plan").unwrap();
    let mut min_from_ok = eval.init_min_depths(&c_ok);
    let err = eval.check_and_advance(&c_bad, &mut min_from_ok, "plan").unwrap_err();
    assert_eq!(err.detail.constraint_id.as_deref(), Some("floor_z"));
    let mut min_from_bad = eval.init_min_depths(&c_bad);
    eval.check_and_advance(&c_ok, &mut min_from_bad, "plan").expect("retreat allowed");
}

#[test]
fn servo_frame_into_the_floor_is_rejected_and_moving_up_is_accepted() {
    let (ok, bad) = floor_edge();
    let mut inner = armed(ArmConfig::rebot_dm(), &ok);
    let epoch = inner.safety.epoch_id();
    let err = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).and_then(|__a| inner.servo(__a, bad.clone(), epoch)) } }.unwrap_err();
    let SafetyError::WorkspaceViolation { limit, value, bound, point, detail } = &err else { panic!("{err:?}") };
    assert_eq!(limit, "floor");
    assert!(*value < *bound && (1..=8).contains(point), "{err}");
    assert_eq!(detail.constraint_id.as_deref(), Some("floor_z"));
    assert_eq!(detail.violation_phase.as_deref(), Some("servo_target"));
    assert!(inner.servo_target.is_none(), "rejected frame does not start motion");

    // from a violating pose, a frame that reduces the violation is accepted
    let deeper = lerp(&DIP_A, &DIP_B, 0.40); // deepest point of the dip
    let mut inner = armed(ArmConfig::rebot_dm(), &deeper);
    let up = lerp(&DIP_A, &DIP_B, 0.30);
    assert!(total(&up) < total(&deeper));
    { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).and_then(|__a| inner.servo(__a, up, epoch)) } }.expect("moving out of the violation is allowed");
    // ... and one that makes it worse is not
    let mut inner = armed(ArmConfig::rebot_dm(), &lerp(&DIP_A, &DIP_B, 0.30));
    assert!(matches!({ let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).and_then(|__a| inner.servo(__a, deeper, epoch)) } }, Err(SafetyError::WorkspaceViolation { .. })));
}

#[test]
fn jog_whose_path_dips_below_the_floor_is_rejected() {
    let mut inner = armed(ArmConfig::rebot_dm(), &DIP_A);
    let err = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.jog(__a, DIP_B.to_vec(), None, None)) } }.unwrap_err();
    assert!(matches!(err, ArmError::Safety(SafetyError::WorkspaceViolation { .. })), "{err:?}");
    assert!(inner.active_trajectory.is_none());
    // the same jog without a workspace policy is accepted
    let mut config = ArmConfig::rebot_dm();
    config.workspace = None;
    let mut inner = armed(config, &DIP_A);
    { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.jog(__a, DIP_B.to_vec(), None, None)) } }.expect("no workspace, no check");
}

#[test]
fn home_from_inspect_is_accepted() {
    let mut inner = armed(ArmConfig::rebot_dm(), &INSPECT);
    { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.jog(__a, HOME.to_vec(), None, None)) } }.expect("home from inspect");
    let mut inner = armed(ArmConfig::rebot_dm(), &HOME);
    { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.jog(__a, INSPECT.to_vec(), None, None)) } }.expect("inspect from home");
}

#[test]
fn unload_is_never_rejected_even_from_a_violating_pose() {
    let deeper = lerp(&DIP_A, &DIP_B, 0.5);
    let mut inner = armed(ArmConfig::rebot_dm(), &deeper);
    inner.execute_stop(armd::StopPolicy::NormalUnload);
    assert!(inner.pending_normal_unload && inner.active_trajectory.is_some(), "park must start");
}

#[test]
fn no_kinematics_workspace_or_wrong_dof_means_no_check() {
    let deeper = lerp(&DIP_A, &DIP_B, 0.5);
    let mut config = ArmConfig::rebot_dm();
    config.kinematics = None;
    let mut inner = armed(config, &DIP_A);
    assert!(inner.workspace_policy().is_none());
    let epoch = inner.safety.epoch_id();
    { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).and_then(|__a| inner.servo(__a, lerp(&DIP_A, &DIP_B, 0.05), epoch)) } }.unwrap();
    inner.check_workspace_path(&DIP_A, [deeper.as_slice()]).unwrap();
    // 6-DOF default profile: no workspace configured
    assert!(SupervisorInner::new(ArmConfig::default()).workspace_policy().is_none());
    // kinematics on a non-7-DOF profile is a config error
    let mut bad = ArmConfig::default();
    bad.kinematics = Some(armd::model::KinematicsModel::RebotDevArm);
    assert!(bad.validate().is_err());
}

#[test]
fn workspace_config_validation_and_schema() {
    let mut c = ArmConfig::rebot_dm();
    c.workspace.as_mut().unwrap().box_min_m[2] = 1.0;
    assert!(c.validate().is_err(), "box_min < box_max");
    let mut c = ArmConfig::rebot_dm();
    c.workspace.as_mut().unwrap().check_points_end_m.clear();
    assert!(c.validate().is_err(), "1..16 points");
    let mut c = ArmConfig::rebot_dm();
    c.workspace.as_mut().unwrap().floor_z_m = f32::NAN;
    assert!(c.validate().is_err());
    let mut c = ArmConfig::rebot_dm();
    c.camera_mount.offset_m = Some([0.0, f32::INFINITY, 0.0]);
    assert!(c.validate().is_err());
    let schema = serde_json::to_value(ObservationSchema::from_config(&ArmConfig::rebot_dm())).unwrap();
    assert_eq!(schema["workspace"]["floor_z_m"], 0.0);
    assert_eq!(schema["workspace"]["check_points_end_m"].as_array().unwrap().len(), 8);
    assert_eq!(schema["camera_mount"]["flip"], false);
    // profiles written before the workspace fields still load
    let mut legacy = serde_json::to_value(ArmConfig::default()).unwrap();
    for k in ["kinematics", "workspace", "camera_mount", "planned_max_velocity"] { legacy.as_object_mut().unwrap().remove(k); }
    let parsed: ArmConfig = serde_json::from_value(legacy).unwrap();
    assert!(parsed.workspace.is_none() && parsed.kinematics.is_none());
}

#[tokio::test]
async fn ws_servo_and_preset_rejections_carry_the_workspace_code() {
    let mut config = ArmConfig::rebot_dm();
    config.lease_timeout_ms = 5000;
    let (addr, supervisor) = common::spawn_server(config).await;
    let (mut ws, generation) = common::controller(addr).await;
    let epoch = supervisor.get_observation().await.epoch_id;
    let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/torque", serde_json::json!({"enable": true})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let (ok, bad) = floor_edge();
    {
        let mut inner = supervisor.inner().write().await;
        inner.commanded_positions = ok.clone();
        inner.measured_positions = ok.clone();
    }
    let epoch = supervisor.get_observation().await.epoch_id;
    common::servo(&mut ws, generation, epoch, &bad).await;
    let reject = common::servo_reject_within(&mut ws, 1000).await.expect("servo_reject");
    assert_eq!(reject["code"], "WORKSPACE_VIOLATION", "{reject}");
    assert_eq!(reject["status"], 422);

    {
        let mut inner = supervisor.inner().write().await;
        inner.commanded_positions = DIP_A.to_vec();
        inner.measured_positions = DIP_A.to_vec();
    }
    let reply = common::preset(&mut ws, generation, epoch, "custom", Some(DIP_B.to_vec())).await;
    assert_eq!(reply["type"], "preset_reject", "{reply}");
    assert_eq!(reply["code"], "WORKSPACE_VIOLATION");
    let reply = common::rpc(&mut ws, generation, epoch, "/api/v1/motion/jog", serde_json::json!({"target_positions": DIP_B})).await;
    assert_eq!(reply["type"], "command_reject");
    assert_eq!(reply["code"], "WORKSPACE_VIOLATION");
    let reply = common::preset(&mut ws, generation, epoch, "park", None).await;
    assert_eq!(reply["type"], "preset_reject", "park crossing floor must be rejected: {reply}");
    assert_eq!(reply["code"], "WORKSPACE_VIOLATION", "{reply}");
    assert_eq!(reply["violation_phase"], "park", "{reply}");
    // From a clear starting pose (INSPECT), park succeeds.
    {
        let mut inner = supervisor.inner().write().await;
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
    }
    let reply = common::preset(&mut ws, generation, epoch, "park", None).await;
    assert_eq!(reply["type"], "preset_ack", "park from INSPECT must succeed: {reply}");
}

#[test]
fn per_constraint_rule_rejects_new_keepout_even_when_retreating_from_floor_or_reach() {
    // 1. Retreating >= 5 cm from floor_z while entering a keep-out box by ~3 cm MUST be rejected.
    // Choose a pose at INSPECT, set floor_z 7 cm above its lowest gripper corner so it starts
    // violating floor_z by ~7 cm, then move up so floor_z intrusion shrinks by > 5 cm while
    // entering a keepout box around the target TCP by ~3 cm.
    let mut cfg = ArmConfig::rebot_dm();
    let p_start = ArmModel::REBOT_DEV_ARM.fk_f32(&INSPECT).unwrap();
    let floor_0 = p_start.p[2] - 0.02; // INSPECT TCP is ~0.07+ m above floor; find lowest corner
    let min_corner_z = cfg.workspace.as_ref().unwrap().check_points_end_m.iter()
        .map(|c| p_start.transform_point(&c.map(f64::from))[2]).fold(p_start.p[2], f64::min);
    let _ = floor_0;
    let mut q_up = INSPECT;
    q_up[1] += 0.35; // lift shoulder up > 5 cm
    let p_up = ArmModel::REBOT_DEV_ARM.fk_f32(&q_up).unwrap();
    let up_min_z = cfg.workspace.as_ref().unwrap().check_points_end_m.iter()
        .map(|c| p_up.transform_point(&c.map(f64::from))[2]).fold(p_up.p[2], f64::min);
    assert!(up_min_z - min_corner_z > 0.05, "need >5 cm lift: {} -> {}", min_corner_z, up_min_z);

    let ws = cfg.workspace.as_mut().unwrap();
    ws.floor_z_m = (min_corner_z + 0.07) as f32; // starts violating floor_z by 7 cm
    ws.keepout_boxes.push(armd::model::KeepoutBox {
        name: "wall".into(),
        // Put box face 3 cm past p_up.p so p_up enters keepout:wall by 3 cm (and start is outside)
        min_m: [p_up.p[0] as f32 - 0.03, p_up.p[1] as f32 - 0.05, p_up.p[2] as f32 - 0.05],
        max_m: [p_up.p[0] as f32 + 0.05, p_up.p[1] as f32 + 0.05, p_up.p[2] as f32 + 0.05],
    });
    // Ensure INSPECT is outside "wall"
    let v_start = ArmModel::REBOT_DEV_ARM.workspace_violation(&INSPECT, ws, &cfg.camera_mount);
    assert!(v_start.total > 0.05);
    let v_up = ArmModel::REBOT_DEV_ARM.workspace_violation(&q_up, ws, &cfg.camera_mount);
    assert!(v_up.total < v_start.total, "scalar total decreased ({} < {}), yet keepout:wall was entered!", v_up.total, v_start.total);

    let mut inner = armed(cfg, &INSPECT);
    let epoch = inner.safety.epoch_id();
    let err = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).and_then(|__a| inner.servo(__a, q_up.to_vec(), epoch)) } }.unwrap_err();
    let SafetyError::WorkspaceViolation { detail, .. } = &err else { panic!("expected WORKSPACE_VIOLATION, got {err:?}") };
    assert_eq!(detail.constraint_id.as_deref(), Some("keepout:wall"));
    assert_eq!(detail.violation_phase.as_deref(), Some("servo_target"));
}

#[test]
fn per_constraint_rule_allows_escape_and_rejects_reentry_and_no_eps_accumulation() {
    // 2. Legitimate escape out of a violating box -> ALLOWED; re-entering after exiting -> REJECTED;
    //    and WORKSPACE_EPS does NOT accumulate across multiple tiny steps.
    let mut cfg = ArmConfig::rebot_dm();
    cfg.workspace.as_mut().unwrap().check_points_end_m = vec![[0.0, 0.0, 0.0]];
    let p_in = ArmModel::REBOT_DEV_ARM.fk_f32(&INSPECT).unwrap().p;
    cfg.workspace.as_mut().unwrap().keepout_boxes.push(armd::model::KeepoutBox {
        name: "zone".into(),
        min_m: [p_in[0] as f32 - 0.03, p_in[1] as f32 - 0.03, p_in[2] as f32 - 0.03],
        max_m: [p_in[0] as f32 + 0.03, p_in[1] as f32 + 0.03, p_in[2] as f32 + 0.03],
    });
    let mut q_slide = INSPECT;
    q_slide[0] += 0.02; // still inside, sliding/retreating toward +y exit
    let mut q_out = INSPECT;
    q_out[0] += 0.35; // rotate J1 out of the box
    let p_out = ArmModel::REBOT_DEV_ARM.fk_f32(&q_out).unwrap().p;
    assert!(p_out[1] > p_in[1] + 0.04, "q_out must exit box in Y: {:?}", p_out);

    let ws = cfg.workspace.as_ref().unwrap();
    let cam = &cfg.camera_mount;
    // Direct check_path_with: INSPECT (inside) -> q_slide (retreating/sliding) -> q_out (outside) is ALLOWED
    armd::supervisor::check_path_with(&ArmModel::REBOT_DEV_ARM, ws, cam, &INSPECT, [q_slide.as_slice(), q_out.as_slice()]).unwrap();
    // Exiting then re-entering in the same path: INSPECT -> q_out -> INSPECT is REJECTED
    let err = armd::supervisor::check_path_with(&ArmModel::REBOT_DEV_ARM, ws, cam, &INSPECT, [q_out.as_slice(), INSPECT.as_slice()]).unwrap_err();
    let SafetyError::WorkspaceViolation { detail, .. } = &err else { panic!("{err:?}") };
    assert_eq!(detail.constraint_id.as_deref(), Some("keepout:zone"));

    // Boundary slide / no epsilon accumulation: 5 consecutive steps each nudging 0.6 * WORKSPACE_EPS
    // beyond floor_z must fail as soon as cumulative intrusion exceeds WORKSPACE_EPS.
    let eval = armd::chain::WorkspaceEvaluator::new(&ArmModel::REBOT_DEV_ARM, ws, cam);
    let mut c0 = eval.alloc_buf();
    eval.eval_into(&INSPECT, &mut c0, "plan").unwrap();
    let mut min_d = eval.init_min_depths(&c0);
    // Synthesize clearances on slot 0 (end_origin floor_z): -0.6e-4, -1.2e-4
    let mut c_step = c0.clone();
    c_step.as_mut_slice()[0] = -0.6 * kinematics::WORKSPACE_EPS;
    eval.check_and_advance(&c_step, &mut min_d, "plan").expect("within single WORKSPACE_EPS");
    c_step.as_mut_slice()[0] = -1.2 * kinematics::WORKSPACE_EPS;
    let err = eval.check_and_advance(&c_step, &mut min_d, "plan").unwrap_err();
    assert_eq!(err.detail.constraint_id.as_deref(), Some("floor_z"));
}

#[test]
fn cartesian_sweep_catches_thin_keepout_box_between_5mm_ik_samples() {
    use armd::motion::{plan_cartesian_ex, resolve_goal, GoalSpec, Speed};
    let mut cfg = ArmConfig::rebot_dm();
    let inner_probe = armed(cfg.clone(), &INSPECT);
    let input_probe = inner_probe.plan_input().unwrap();
    let p0 = ArmModel::REBOT_DEV_ARM.fk_f32(&INSPECT).unwrap().p;
    let goal = resolve_goal(&input_probe, &GoalSpec::Absolute { p: [p0[0] + 0.04, p0[1], p0[2]], r: None });
    let path = plan_cartesian_ex(&input_probe, goal, Speed::default(), false).unwrap();
    assert!(path.q.len() >= 3);
    // Midpoint in joint space between IK sample 1 and 2
    let mid_q: Vec<f32> = path.q[1].iter().zip(&path.q[2]).map(|(a, b)| 0.5 * (a + b)).collect();
    let p_mid = ArmModel::REBOT_DEV_ARM.fk_f32(&mid_q).unwrap().p;
    // Place a 0.6 mm thin box strictly between IK sample 1 and IK sample 2
    let p1 = ArmModel::REBOT_DEV_ARM.fk_f32(&path.q[1]).unwrap().p;
    let p2 = ArmModel::REBOT_DEV_ARM.fk_f32(&path.q[2]).unwrap().p;
    assert!((p_mid[0] - p1[0]).abs() > 0.001 && (p2[0] - p_mid[0]).abs() > 0.001);
    let thin = armd::model::KeepoutBox {
        name: "razor".into(),
        min_m: [(p_mid[0] - 0.0003) as f32, (p_mid[1] - 0.01) as f32, (p_mid[2] - 0.01) as f32],
        max_m: [(p_mid[0] + 0.0003) as f32, (p_mid[1] + 0.01) as f32, (p_mid[2] + 0.01) as f32],
    };
    cfg.workspace.as_mut().unwrap().keepout_boxes.push(thin);
    // Confirm the discrete IK samples themselves miss it
    let ws = cfg.workspace.as_ref().unwrap();
    assert_eq!(ArmModel::REBOT_DEV_ARM.workspace_violation(&path.q[1], ws, &cfg.camera_mount).total, 0.0);
    assert_eq!(ArmModel::REBOT_DEV_ARM.workspace_violation(&path.q[2], ws, &cfg.camera_mount).total, 0.0);

    let inner = armed(cfg, &INSPECT);
    let input = inner.plan_input().unwrap();
    let err = plan_cartesian_ex(&input, goal, Speed::default(), false).unwrap_err();
    match err {
        armd::motion::MotionError::WorkspaceViolation { detail, limit, .. } => {
            assert_eq!(limit, "keepout 'razor'");
            assert_eq!(detail.constraint_id.as_deref(), Some("keepout:razor"));
            assert_eq!(detail.violation_phase.as_deref(), Some("sweep"));
        }
        other => panic!("expected WorkspaceViolation from Cartesian sweep, got {other:?}"),
    }
}

#[test]
fn servo_mid_step_dip_below_floor_with_empty_keepouts_is_caught_and_held() {
    // 4. Empty keepout_boxes: servo step dipping below floor_z mid-step must be caught and held
    //    with violation_phase == "servo_step", preserving the gripper command.
    let mut cfg = ArmConfig::rebot_dm();
    assert!(cfg.workspace.as_ref().unwrap().keepout_boxes.is_empty());
    // Along DIP_A -> DIP_B, the minimum z of the gripper corners is around s = 0.40.
    // s = 0.28 -> s = 0.52 has max |dq| ~ 0.32 rad (< 0.35 dirty-jump bound), and dips at s = 0.40.
    let q_a = lerp(&DIP_A, &DIP_B, 0.28);
    let q_mid_ref = lerp(&DIP_A, &DIP_B, 0.40);
    let q_b = lerp(&DIP_A, &DIP_B, 0.52);
    let min_z = |q: &[f32]| -> f64 {
        let p = ArmModel::REBOT_DEV_ARM.fk_f32(q).unwrap();
        cfg.workspace.as_ref().unwrap().check_points_end_m.iter()
            .map(|c| p.transform_point(&c.map(f64::from))[2])
            .fold(p.p[2], f64::min)
    };
    let (za, zmid, zb) = (min_z(&q_a), min_z(&q_mid_ref), min_z(&q_b));
    assert!(zmid < za.min(zb) - 0.0005, "expected midpoint dip: za={za}, zmid={zmid}, zb={zb}");
    cfg.workspace.as_mut().unwrap().floor_z_m = (za.min(zb) - 0.0003) as f32;
    // Allow single-tick step to cross the dip
    cfg.servo_max_velocity = None;
    for l in &mut cfg.limits { l.max_velocity = 25.0; l.max_acceleration = 1500.0; }

    let mut q_start = q_a;
    q_start[6] = -1.25; // held gripper command
    let mut q_target = q_b;
    q_target[6] = -1.25;

    let mut inner = armed(cfg, &q_start);
    let epoch = inner.safety.epoch_id();
    { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).and_then(|__a| inner.servo(__a, q_target.to_vec(), epoch)) } }.expect("target itself is above floor_z");
    for _ in 0..40 {
        inner.tick(0.02);
        if inner.servo_blocked_hold { break; }
    }
    assert!(inner.servo_blocked_hold && inner.servo_target.is_none(), "must enter recoverable workspace hold");
    assert_eq!(inner.commanded_positions[6], -1.25, "gripper command preserved");
    let obs = inner.current_observation();
    assert_eq!(obs.motion_limit.as_ref().map(|l| l.reason), Some(armd::model::MotionLimitReason::Workspace));
    let detail = obs.workspace_violation.as_ref().expect("observation.workspace_violation");
    assert_eq!(detail.constraint_id.as_deref(), Some("floor_z"));
    assert_eq!(detail.violation_phase.as_deref(), Some("servo_step"));
}

#[test]
fn stream_braking_crossing_outer_boundary_is_caught_and_held() {
    // 5. Braking step crossing outer box_max_x -> caught and held with violation_phase == "braking"
    let mut cfg = ArmConfig::rebot_dm();
    let max_x_at = |q: &[f32]| -> f64 {
        let p = ArmModel::REBOT_DEV_ARM.fk_f32(q).unwrap();
        cfg.workspace.as_ref().unwrap().check_points_end_m.iter()
            .map(|c| p.transform_point(&c.map(f64::from))[0])
            .fold(p.p[0], f64::max)
    };
    let mut q_next = INSPECT;
    q_next[1] -= 0.01;
    let sign = if max_x_at(&q_next) > max_x_at(&INSPECT) { -1.0f32 } else { 1.0f32 };
    let mut q_step = INSPECT;
    q_step[1] += 0.008 * sign;
    let dx = max_x_at(&q_step) - max_x_at(&INSPECT);
    assert!(dx > 0.0005, "expected J2 step to increase max_x: dx={dx}");
    let max_x = max_x_at(&INSPECT);
    cfg.workspace.as_mut().unwrap().box_max_m[0] = (max_x + 0.0002) as f32;

    let mut start = INSPECT.to_vec();
    start[6] = -0.85;
    let mut inner = armed(cfg, &start);
    // Simulate a running servo stream that cleared target while still moving +x
    inner.servo_target = None;
    inner.commanded_velocities[1] = 0.45 * sign;
    let obs = inner.tick(0.02);
    assert!(inner.servo_blocked_hold, "braking across box_max_x must hold");
    assert_eq!(inner.commanded_positions[6], -0.85, "gripper command preserved");
    assert_eq!(obs.motion_limit.as_ref().map(|l| l.reason), Some(armd::model::MotionLimitReason::Workspace));
    let detail = obs.workspace_violation.as_ref().expect("observation.workspace_violation");
    assert_eq!(detail.constraint_id.as_deref(), Some("box_max_x"));
    assert_eq!(detail.violation_phase.as_deref(), Some("braking"));
}

#[test]
fn link_capsules_reject_upper_arm_intrusion_and_validate_frames() {
    // 6. link_capsules[]: end_effector safe while upper_arm capsule enters keepout box
    let mut cfg = ArmConfig::rebot_dm();
    let ws_mut = cfg.workspace.as_mut().unwrap();
    ws_mut.self_collision_enabled = false;
    ws_mut.self_collision_pairs.clear();
    ws_mut.link_capsules = vec![armd::model::LinkCapsule {
        id: "upper_arm".into(),
        frame: "upper_arm".into(),
        a_m: [0.0, 0.0, 0.0],
        b_m: [-0.264, 0.0, 0.0],
        radius_m: 0.035,
    }];
    cfg.validate().unwrap();

    // Unknown frame fails validation
    let mut bad_frame = cfg.clone();
    bad_frame.workspace.as_mut().unwrap().link_capsules[0].frame = "nonexistent_link".into();
    assert!(bad_frame.validate().is_err());

    // Duplicate id fails validation
    let mut dup_id = cfg.clone();
    let c0 = dup_id.workspace.as_ref().unwrap().link_capsules[0].clone();
    dup_id.workspace.as_mut().unwrap().link_capsules.push(c0);
    assert!(dup_id.validate().is_err());

    // Negative radius fails validation
    let mut neg_r = cfg.clone();
    neg_r.workspace.as_mut().unwrap().link_capsules[0].radius_m = -0.01;
    assert!(neg_r.validate().is_err());

    // Place a keepout box on the midpoint of link2 (upper_arm) at INSPECT (far from end_link!)
    let f = ArmModel::REBOT_DEV_ARM.frames_f32(&INSPECT).unwrap();
    let fr = ArmModel::REBOT_DEV_ARM.resolve_frame("upper_arm").unwrap();
    let link2_pose = fr.pose(&f);
    let mid_arm = link2_pose.transform_point(&[-0.132, 0.0, 0.0]);
    let end_p = f.end.p;
    assert!(kinematics::norm(&kinematics::sub(&mid_arm, &end_p)) > 0.15, "keepout box must be far from end_link");

    let box_on_arm = armd::model::KeepoutBox {
        name: "arm_obstacle".into(),
        min_m: mid_arm.map(|v| (v - 0.01) as f32),
        max_m: mid_arm.map(|v| (v + 0.01) as f32),
    };
    let mut cfg_box = cfg.clone();
    cfg_box.workspace.as_mut().unwrap().keepout_boxes.push(box_on_arm);
    // Move from HOME (safe) to INSPECT (end_link safe, upper_arm capsule inside arm_obstacle)
    let mut inner = armed(cfg_box, &HOME);
    let err = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.jog(__a, INSPECT.to_vec(), None, None)) } }.unwrap_err();
    let ArmError::Safety(SafetyError::WorkspaceViolation { detail, .. }) = &err else { panic!("expected WorkspaceViolation, got {err:?}") };
    assert_eq!(detail.geometry_id.as_deref(), Some("capsule:upper_arm"));
    assert_eq!(detail.constraint_id.as_deref(), Some("keepout:arm_obstacle"));

    // safety/describe reports "end_and_links"
    let d = armd::SupervisorInner::new(cfg).safety_describe();
    assert_eq!(d["limits"]["geometry_coverage"], "end_and_links");
    assert_eq!(d["limits"]["workspace"]["geometry_coverage"], "end_and_links");
}

#[test]
fn park_from_violating_outer_boundary_allowed_but_crossing_keepout_rejected() {
    // 7. park from a pose violating floor_z to a legal resting pose is allowed when it recovers
    //    monotonically; park whose resting pose violates max_reach_m or crosses a keep-out box
    //    is rejected with violation_phase == "park".
    let cfg = ArmConfig::rebot_dm();
    let deeper = lerp(&DIP_A, &DIP_B, 0.5);
    let mut inner = armed(cfg.clone(), &deeper);
    assert!({ let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.unload_action(__a)) } }.unwrap().is_some(), "park retreating from floor_z to legal HOME must succeed");

    let mut cfg_reach = cfg.clone();
    cfg_reach.workspace.as_mut().unwrap().max_reach_m = Some(0.15); // HOME also violates max_reach
    let mut inner = armed(cfg_reach, &deeper);
    let err = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.unload_action(__a)) } }.unwrap_err();
    let ArmError::Safety(SafetyError::WorkspaceViolation { detail, .. }) = &err else { panic!("expected WorkspaceViolation, got {err:?}") };
    assert_eq!(detail.constraint_id.as_deref(), Some("reach"));
    assert_eq!(detail.violation_phase.as_deref(), Some("park"));

    // Now add a keepout box on the park path from deeper -> HOME
    let mut cfg_keepout = cfg;
    let mid: Vec<f32> = deeper.iter().zip(&cfg_keepout.home_positions).map(|(a, b)| 0.5 * (a + b)).collect();
    let p_mid = ArmModel::REBOT_DEV_ARM.fk_f32(&mid).unwrap().p;
    cfg_keepout.workspace.as_mut().unwrap().keepout_boxes.push(armd::model::KeepoutBox {
        name: "park_wall".into(),
        min_m: p_mid.map(|v| (v - 0.04) as f32),
        max_m: p_mid.map(|v| (v + 0.04) as f32),
    });
    let mut inner = armed(cfg_keepout, &deeper);
    let err = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.unload_action(__a)) } }.unwrap_err();
    let ArmError::Safety(SafetyError::WorkspaceViolation { detail, .. }) = &err else { panic!("expected WorkspaceViolation, got {err:?}") };
    assert_eq!(detail.constraint_id.as_deref(), Some("keepout:park_wall"));
    assert_eq!(detail.violation_phase.as_deref(), Some("park"));
}

#[tokio::test]
async fn wire_contract_attaches_structured_fields_to_ws_rejects_and_action_status() {
    // Verify WS servo_reject, command_reject, preset_reject, and ActionStatus JSON attach
    // constraint_id, geometry_id, violation_phase, and reason on WORKSPACE_VIOLATION.
    let mut cfg = ArmConfig::rebot_dm();
    cfg.lease_timeout_ms = 5000;
    let p_inspect = ArmModel::REBOT_DEV_ARM.fk_f32(&INSPECT).unwrap().p;
    // Keepout box blocking a move_to target AND a park path
    let mid_park: Vec<f32> = INSPECT.iter().zip(&cfg.home_positions).map(|(a, b)| 0.5 * (a + b)).collect();
    let p_park = ArmModel::REBOT_DEV_ARM.fk_f32(&mid_park).unwrap().p;
    cfg.workspace.as_mut().unwrap().keepout_boxes.push(armd::model::KeepoutBox {
        name: "bin_wall".into(),
        min_m: [p_inspect[0] as f32 + 0.03, p_inspect[1] as f32 - 0.03, p_inspect[2] as f32 - 0.03],
        max_m: [p_inspect[0] as f32 + 0.09, p_inspect[1] as f32 + 0.03, p_inspect[2] as f32 + 0.03],
    });
    cfg.workspace.as_mut().unwrap().keepout_boxes.push(armd::model::KeepoutBox {
        name: "park_box".into(),
        min_m: p_park.map(|v| (v - 0.03) as f32),
        max_m: p_park.map(|v| (v + 0.03) as f32),
    });

    let (addr, supervisor) = common::spawn_server(cfg).await;
    let (mut ws, generation) = common::controller(addr).await;
    let epoch = supervisor.get_observation().await.epoch_id;
    let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/torque", serde_json::json!({"enable": true})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");

    {
        let mut inner = supervisor.inner().write().await;
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
    }
    let epoch = supervisor.get_observation().await.epoch_id;

    // 1. WS command_reject from move_to into keepout:bin_wall
    let reply = common::rpc(&mut ws, generation, epoch, "/api/v1/action/move_to",
        serde_json::json!({"position": [p_inspect[0] + 0.05, p_inspect[1], p_inspect[2]], "orientation": null})).await;
    assert_eq!(reply["type"], "command_reject", "{reply}");
    assert_eq!(reply["code"], "WORKSPACE_VIOLATION", "{reply}");
    assert_eq!(reply["constraint_id"], "keepout:bin_wall", "{reply}");
    assert!(reply["geometry_id"].as_str().is_some(), "{reply}");
    assert!(matches!(reply["violation_phase"].as_str(), Some("plan" | "sweep")), "{reply}");
    assert!(reply["reason"].as_str().is_some(), "{reply}");

    // 2. WS preset_reject from park crossing keepout:park_box
    let reply = common::preset(&mut ws, generation, epoch, "park", None).await;
    assert_eq!(reply["type"], "preset_reject", "{reply}");
    assert_eq!(reply["code"], "WORKSPACE_VIOLATION", "{reply}");
    assert_eq!(reply["constraint_id"], "keepout:park_box", "{reply}");
    assert_eq!(reply["violation_phase"], "park", "{reply}");
    assert!(reply["reason"].as_str().is_some(), "{reply}");

    // 3. WS servo_reject from floor_edge
    let (ok, bad) = floor_edge();
    {
        let mut inner = supervisor.inner().write().await;
        inner.commanded_positions = ok;
        inner.measured_positions = bad.clone(); // unrelated to check
        inner.measured_positions = inner.commanded_positions.clone();
    }
    common::servo(&mut ws, generation, epoch, &bad).await;
    let reject = common::servo_reject_within(&mut ws, 1000).await.expect("servo_reject");
    assert_eq!(reject["code"], "WORKSPACE_VIOLATION", "{reject}");
    assert_eq!(reject["constraint_id"], "floor_z", "{reject}");
    assert_eq!(reject["violation_phase"], "servo_target", "{reject}");
    assert!(reject["geometry_id"].as_str().is_some(), "{reject}");
    assert!(reject["reason"].as_str().is_some(), "{reject}");

    // 4. ActionStatus JSON omits structured fields when non-workspace, includes them on WORKSPACE_VIOLATION
    {
        let mut inner = supervisor.inner().write().await;
        let id = inner.begin_action("place", None);
        inner.set_action_workspace_violation(id, kinematics::WorkspaceViolationDetail {
            constraint_id: Some("keepout:bin_wall".into()),
            geometry_id: Some("end_origin".into()),
            violation_phase: Some("plan".into()),
            reason: Some("new intrusion into keepout:bin_wall by 0.012m".into()),
        });
        inner.finish_action(id, "failed", Some("WORKSPACE_VIOLATION"), Some("failed".into()), None, None, None);
        let action_json = serde_json::to_value(inner.last_action.as_ref().unwrap()).unwrap();
        assert_eq!(action_json["code"], "WORKSPACE_VIOLATION");
        assert_eq!(action_json["constraint_id"], "keepout:bin_wall");
        assert_eq!(action_json["geometry_id"], "end_origin");
        assert_eq!(action_json["violation_phase"], "plan");
        assert_eq!(action_json["reason"], "new intrusion into keepout:bin_wall by 0.012m");
        assert!(action_json["phase"].is_null());

        let id2 = inner.begin_action("home", None);
        inner.finish_action(id2, "done", None, None, Some(true), None, None);
        let ok_json = serde_json::to_value(inner.last_action.as_ref().unwrap()).unwrap();
        assert!(ok_json.get("constraint_id").is_none());
        assert!(ok_json.get("violation_phase").is_none());
    }
}

#[test]
fn servo_stream_persists_monotonic_escape_baseline_across_tiny_steps() {
    // BLOCKER-2: multiple consecutive tiny servo steps (< WORKSPACE_EPS each) from an
    // already-violating pose must NOT accumulate WORKSPACE_EPS across ticks.
    let mut cfg = ArmConfig::rebot_dm();
    cfg.servo_max_velocity = None;
    for l in &mut cfg.limits {
        l.max_velocity = 25.0;
        l.max_acceleration = 1500.0;
    }
    let mut q_start = INSPECT;
    q_start[6] = -0.5;
    let p_start = ArmModel::REBOT_DEV_ARM.fk_f32(&q_start).unwrap();
    let start_min_z = cfg
        .workspace
        .as_ref()
        .unwrap()
        .check_points_end_m
        .iter()
        .map(|c| p_start.transform_point(&c.map(f64::from))[2])
        .fold(p_start.p[2], f64::min);
    // Start 1 cm below floor_z (start_depth = 0.01 m)
    let floor_z = start_min_z + 0.01;
    cfg.workspace.as_mut().unwrap().floor_z_m = floor_z as f32;

    // Find direction on J2 that lowers min_z monotonically.
    let check_pts = cfg.workspace.as_ref().unwrap().check_points_end_m.clone();
    let min_z_at = |q: &[f32]| -> f64 {
        let p = ArmModel::REBOT_DEV_ARM.fk_f32(q).unwrap();
        check_pts
            .iter()
            .map(|c| p.transform_point(&c.map(f64::from))[2])
            .fold(p.p[2], f64::min)
    };
    let mut q_probe = q_start;
    q_probe[1] += 0.002;
    let sign = if min_z_at(&q_probe) < start_min_z { 1.0f32 } else { -1.0f32 };
    // Calibrate a tiny J2 increment yielding ~0.4 * WORKSPACE_EPS (0.04 mm) deeper intrusion per step.
    let dz_per_mrad = (start_min_z - {
        let mut q = q_start;
        q[1] += 0.001 * sign;
        min_z_at(&q)
    }) / 0.001;
    assert!(dz_per_mrad > 0.01, "expected J2 motion to change min_z: {dz_per_mrad}");
    let dq_step = ((0.4 * kinematics::WORKSPACE_EPS) / dz_per_mrad) as f32 * sign;

    let mut inner = armed(cfg, &q_start);
    let epoch = inner.safety.epoch_id();
    let start_depth = floor_z - min_z_at(&inner.commanded_positions);

    let mut held = false;
    for k in 1..=10 {
        let mut q_next = q_start;
        q_next[1] += (k as f32) * dq_step;
        let _ = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).and_then(|__a| inner.servo(__a, q_next.to_vec(), epoch)) } };
        inner.tick(0.02);
        let depth_now = floor_z - min_z_at(&inner.commanded_positions);
        assert!(
            depth_now <= start_depth + kinematics::WORKSPACE_EPS + 1e-6,
            "step {k}: cumulative intrusion {depth_now} exceeded start_depth ({start_depth}) + WORKSPACE_EPS"
        );
        if inner.servo_blocked_hold {
            held = true;
            break;
        }
    }
    assert!(held, "later tiny servo steps must be rejected and enter workspace hold");
    assert_eq!(
        inner.motion_limit.as_ref().map(|l| l.reason),
        Some(armd::model::MotionLimitReason::Workspace)
    );
}

#[test]
fn so101_urdf_rejects_link_capsule_on_moving_jaw_and_accepts_fixed_gripper_link() {
    // MAJOR-1: moving_jaw_so101_v1_link is behind unmodeled revolute joint "gripper".
    let mut cfg: ArmConfig = serde_json::from_str(include_str!("../profiles/so_arm101.json")).unwrap();
    cfg.init_arm_model_in_memory(Some(include_str!("../assets/so101_new_calib.urdf"))).unwrap();

    // Static link on the chain (gripper_link) is valid
    let mut good_cfg = cfg.clone();
    good_cfg.workspace.get_or_insert_with(WorkspacePolicy::rebot_dm).link_capsules.push(
        armd::model::LinkCapsule {
            id: "gripper_body".into(),
            frame: "gripper_link".into(),
            a_m: [0.0, 0.0, 0.0],
            b_m: [0.0, 0.0, 0.05],
            radius_m: 0.02,
        },
    );
    good_cfg.validate().unwrap();

    // Link across unmodeled revolute joint "gripper" must be rejected
    let mut bad_cfg = cfg;
    bad_cfg.workspace.get_or_insert_with(WorkspacePolicy::rebot_dm).link_capsules.push(
        armd::model::LinkCapsule {
            id: "moving_jaw".into(),
            frame: "moving_jaw_so101_v1_link".into(),
            a_m: [0.0, 0.0, 0.0],
            b_m: [0.0, -0.04, 0.0],
            radius_m: 0.015,
        },
    );
    let err = bad_cfg.validate().unwrap_err();
    assert!(err.contains("moving_jaw_so101_v1_link"), "unexpected error: {err}");
}

#[test]
fn tiny_displacement_below_eps_near_tight_boundary_catches_midpoint_dip() {
    // MAJOR-2: even when l_g < WORKSPACE_EPS, a step near a tight boundary whose midpoint dips
    // beyond allowed must still split and reject instead of being skipped.
    let mut cfg = ArmConfig::rebot_dm();
    cfg.workspace.as_mut().unwrap().check_points_end_m = vec![[0.0, 0.0, 0.0]];
    cfg.workspace.as_mut().unwrap().floor_z_m = 0.0;

    // Scan J4 over [0, 2*PI] for a true interior local minimum of end_link z, then tune J2 so
    // that local minimum sits just 2e-9 m below -WORKSPACE_EPS (exact floor_z_m = 0.0).
    let z_of = |q: &[f32; 7]| ArmModel::REBOT_DEV_ARM.fk_f32(q).unwrap().p[2];
    let find_min_j4 = |j2: f32| -> (f32, f64) {
        let mut q = [0.0f32, j2, -0.8, 0.0, 0.0, 0.0, 0.0];
        let mut best_j4 = 0.0f32;
        let mut best_z = f64::INFINITY;
        for k in 0..628 {
            let j4 = k as f32 * 0.01;
            q[3] = j4;
            let z = z_of(&q);
            if z < best_z {
                best_z = z;
                best_j4 = j4;
            }
        }
        // Refine around best_j4
        let mut lo = best_j4 - 0.02;
        let mut hi = best_j4 + 0.02;
        for _ in 0..40 {
            let m1 = lo + (hi - lo) / 3.0;
            let m2 = hi - (hi - lo) / 3.0;
            q[3] = m1;
            let z1 = z_of(&q);
            q[3] = m2;
            let z2 = z_of(&q);
            if z1 < z2 { hi = m2; } else { lo = m1; }
        }
        let j4 = 0.5 * (lo + hi);
        q[3] = j4;
        (j4, z_of(&q))
    };

    // Binary search J2 in [-2.5, -0.5] so min_z(J2) == -WORKSPACE_EPS - 2e-9
    let target_z = -kinematics::WORKSPACE_EPS - 2e-9;
    let mut j2_lo = -2.5f32;
    let mut j2_hi = -0.5f32;
    for _ in 0..40 {
        let j2_mid = 0.5 * (j2_lo + j2_hi);
        let (_, z) = find_min_j4(j2_mid);
        if z < target_z { j2_lo = j2_mid; } else { j2_hi = j2_mid; }
    }
    let j2 = 0.5 * (j2_lo + j2_hi);
    let (best_j4, _) = find_min_j4(j2);
    let q_base = [0.0f32, j2, -0.8, best_j4, 0.0, 0.0, 0.0];

    let eval_probe = armd::chain::WorkspaceEvaluator::new(
        &ArmModel::REBOT_DEV_ARM,
        cfg.workspace.as_ref().unwrap(),
        &cfg.camera_mount,
    );
    let reach4 = eval_probe.end_reach[3];
    let dq = ((0.40 * kinematics::WORKSPACE_EPS) / reach4) as f32;
    let mut q_a = q_base;
    let mut q_b = q_base;
    q_a[3] -= dq;
    q_b[3] += dq;
    let q_m: Vec<f32> = q_a.iter().zip(&q_b).map(|(a, b)| 0.5 * (a + b)).collect();
    let (za, zb, zm) = (z_of(&q_a), z_of(&q_b), ArmModel::REBOT_DEV_ARM.fk_f32(&q_m).unwrap().p[2]);

    let tr = [0.0, 0.0, 0.0, f64::from((q_b[3] - q_a[3]).abs()), 0.0, 0.0];
    let l_g = eval_probe.geom_bound(0, &tr, 6);
    assert!(l_g > 0.0 && l_g < kinematics::WORKSPACE_EPS, "need 0 < l_g < WORKSPACE_EPS, got {l_g}");
    assert!(zm < za.min(zb), "expected midpoint dip in z: za={za}, zm={zm}, zb={zb}");

    let mid_bound = 0.5 * (za.min(zb) + zm) + kinematics::WORKSPACE_EPS;
    cfg.workspace.as_mut().unwrap().floor_z_m = mid_bound as f32;
    let ws = cfg.workspace.as_ref().unwrap();
    let eval = armd::chain::WorkspaceEvaluator::new(&ArmModel::REBOT_DEV_ARM, ws, &cfg.camera_mount);
    let mut c_a = eval.alloc_buf();
    let mut c_b = eval.alloc_buf();
    let mut c_m = eval.alloc_buf();
    eval.eval_into(&q_a, &mut c_a, "plan").unwrap();
    eval.eval_into(&q_b, &mut c_b, "plan").unwrap();
    eval.eval_into(&q_m, &mut c_m, "plan").unwrap();
    let min_d = eval.init_min_depths(&c_a);
    assert!(eval.check_readonly(&c_a, &min_d, "plan").is_ok(), "start endpoint should pass");
    assert!(eval.check_readonly(&c_b, &min_d, "plan").is_ok(), "end endpoint should pass: za={za}, zm={zm}, zb={zb}");
    assert!(eval.check_readonly(&c_m, &min_d, "sweep").is_err(), "midpoint must exceed allowed: za={za}, zm={zm}, zb={zb}");

    let mut min_d_sweep = min_d;
    let mut budget = 64;
    let err = eval
        .sweep_linear_step(&q_a, &q_b, &c_a, &mut min_d_sweep, &mut budget, 64, "plan", "sweep")
        .unwrap_err()
        .1;
    assert_eq!(err.detail.constraint_id.as_deref(), Some("floor_z"), "{err:?}");
    assert_eq!(err.detail.violation_phase.as_deref(), Some("sweep"), "{err:?}");
}

fn pt_seg_dist(p: [f64; 3], a: [f32; 3], b: [f32; 3]) -> f64 {
    let ab = [f64::from(b[0] - a[0]), f64::from(b[1] - a[1]), f64::from(b[2] - a[2])];
    let ap = [p[0] - f64::from(a[0]), p[1] - f64::from(a[1]), p[2] - f64::from(a[2])];
    let den = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
    let t = if den == 0.0 {
        0.0
    } else {
        ((ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / den).clamp(0.0, 1.0)
    };
    let dx = ap[0] - t * ab[0];
    let dy = ap[1] - t * ab[1];
    let dz = ap[2] - t * ab[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn assert_eval_clear(arm: &ArmModel, ws: &WorkspacePolicy, cam: &armd::model::CameraMount, q: &[f32], label: &str) {
    let eval = armd::chain::WorkspaceEvaluator::new(arm, ws, cam);
    let mut buf = eval.alloc_buf();
    eval.eval_into(q, &mut buf, "test")
        .unwrap_or_else(|e| panic!("{label}: unexpected workspace/capsule error: {e:?}"));
    for (slot, &clr) in buf.as_slice().iter().enumerate() {
        assert!(
            clr.is_finite() && clr >= 0.0,
            "{label}: workspace/capsule clearance violation at slot {slot}: {clr} ({:?})",
            eval.describe_failure(slot, clr, -clr, 0.0, "test")
        );
    }
}

#[test]
fn b601dm_builtin_and_kit_profile_link_capsules_positive_and_negative() {
    let builtin = ArmConfig::rebot_dm();
    let kit: ArmConfig = serde_json::from_str(include_str!(
        "../kits/recamera-b601dm/profile/rebot-b601dm.json"
    ))
    .unwrap();
    assert_eq!(builtin.profile_id.as_deref(), Some("rebot_b601dm"));
    assert_eq!(kit.profile_id.as_deref(), Some("rebot_b601dm"));
    assert_eq!(
        builtin.workspace.as_ref().unwrap().link_capsules,
        kit.workspace.as_ref().unwrap().link_capsules
    );
    let capsules = &builtin.workspace.as_ref().unwrap().link_capsules;
    assert!(capsules.len() >= 6);

    // Enclosure verification against sim/mujoco/models/rebot_b601dm_scene.xml collision boxes
    // (link5 is fitted to its URDF mesh below: the MuJoCo box overstates the part by 8 mm)
    let dm_boxes: &[(&str, [f64; 3], [f64; 3])] = &[
        ("link2", [-0.132, 0.0, -0.0316], [0.1605, 0.0285, 0.0337]),
        ("link3", [0.1222, -0.03, -0.0311], [0.1489, 0.0525, 0.0401]),
        ("link4", [0.0449, -0.03, -0.03], [0.0729, 0.058, 0.0334]),
        ("link6", [0.0, 0.0, 0.0242], [0.0385, 0.0318, 0.029]),
    ];
    for (id, c, half) in dm_boxes {
        let cap = capsules.iter().find(|k| k.id == *id).unwrap_or_else(|| panic!("missing capsule {id}"));
        for sx in [-1.0, 1.0] {
            for sy in [-1.0, 1.0] {
                for sz in [-1.0, 1.0] {
                    let corner = [c[0] + sx * half[0], c[1] + sy * half[1], c[2] + sz * half[2]];
                    let d = pt_seg_dist(corner, cap.a_m, cap.b_m);
                    assert!(
                        d <= f64::from(cap.radius_m),
                        "B601-DM capsule '{id}' misses box corner {corner:?}: dist={d:.5}m > r={:.5}m",
                        cap.radius_m
                    );
                }
            }
        }
    }

    // Enclosure verification of base_link and link6 against kits/.../assets/meshes/{manifest.json, arm.bin}
    let mesh_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("kits/recamera-b601dm/packaging/rootfs/usr/share/rebot-teleop/assets/meshes");
    let manifest: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(mesh_dir.join("manifest.json")).unwrap()).unwrap();
    let arm_bin = std::fs::read(mesh_dir.join("arm.bin")).unwrap();
    let load_mesh_verts = |link: &str| -> Vec<[f64; 3]> {
        let mut out = Vec::new();
        for part in manifest["links"][link].as_array().unwrap() {
            let off = part["offset"].as_u64().unwrap() as usize;
            let n = part["verts"].as_u64().unwrap() as usize;
            let bb: Vec<f64> = part["bbox"].as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();
            for i in 0..n {
                let b = off + i * 6;
                let qx = f64::from(u16::from_le_bytes([arm_bin[b], arm_bin[b + 1]]));
                let qy = f64::from(u16::from_le_bytes([arm_bin[b + 2], arm_bin[b + 3]]));
                let qz = f64::from(u16::from_le_bytes([arm_bin[b + 4], arm_bin[b + 5]]));
                out.push([
                    bb[0] + (qx / 65535.0) * (bb[3] - bb[0]),
                    bb[1] + (qy / 65535.0) * (bb[4] - bb[1]),
                    bb[2] + (qz / 65535.0) * (bb[5] - bb[2]),
                ]);
            }
        }
        out
    };
    let base_caps: Vec<_> = capsules
        .iter()
        .filter(|k| k.frame == "base_link")
        .collect();
    assert_eq!(base_caps.len(), 2, "expected base_link + base_plate capsules");
    let base_verts = load_mesh_verts("base_link");
    assert_eq!(base_verts.len(), 3527);
    for v in base_verts {
        let min_signed = base_caps
            .iter()
            .map(|c| pt_seg_dist(v, c.a_m, c.b_m) - f64::from(c.radius_m))
            .fold(f64::INFINITY, f64::min);
        assert!(
            min_signed <= 0.0,
            "base_link capsules miss mesh vertex {v:?}: min_signed={min_signed:.5}m > 0"
        );
    }
    // link5: the URDF mesh with at least 5 mm to spare
    let link5_cap = capsules.iter().find(|k| k.id == "link5").expect("missing link5 capsule");
    for v in load_mesh_verts("link5") {
        let d = pt_seg_dist(v, link5_cap.a_m, link5_cap.b_m);
        assert!(
            d + 0.005 <= f64::from(link5_cap.radius_m),
            "link5 capsule misses mesh vertex {v:?} (5 mm margin): dist={d:.5}m, r={:.5}m",
            link5_cap.radius_m
        );
    }
    let link6_cap = capsules.iter().find(|k| k.id == "link6").expect("missing link6 capsule");
    for v in load_mesh_verts("link6") {
        let d = pt_seg_dist(v, link6_cap.a_m, link6_cap.b_m);
        assert!(
            d <= f64::from(link6_cap.radius_m),
            "link6 capsule misses mesh vertex {v:?}: dist={d:.5}m > r={:.5}m",
            link6_cap.radius_m
        );
    }

    // safety_describe reports end_and_links
    let desc = SupervisorInner::new(kit.clone()).safety_describe();
    assert_eq!(desc["limits"]["geometry_coverage"], "end_and_links");

    // Positive 1: full workspace/capsule check on home, park (resting_positions == home), INSPECT, and home <-> inspect motions
    let (ws, cam) = (kit.workspace.as_ref().unwrap(), &kit.camera_mount);
    assert_eval_clear(&ArmModel::REBOT_DEV_ARM, ws, cam, &kit.home_positions, "B601-DM home");
    assert_eval_clear(&ArmModel::REBOT_DEV_ARM, ws, cam, kit.resting_positions(), "B601-DM park");
    assert_eval_clear(&ArmModel::REBOT_DEV_ARM, ws, cam, &INSPECT, "B601-DM INSPECT");

    let mut inner = armed(kit.clone(), &INSPECT);
    let park_id = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.unload_action(__a)) } }.expect("park from INSPECT succeeds");
    assert!(park_id.is_some());

    let mut inner = armed(kit.clone(), &HOME);
    { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.jog(__a, INSPECT.to_vec(), None, None)) } }.expect("HOME -> INSPECT jog passes");

    // Positive 2: typical grasp approach -> descend -> lift -> retreat Cartesian moves pass without proximal capsule false trigger
    let mut inner = armed(kit.clone(), &INSPECT);
    let r_down = armd::grasp::look_down(&kinematics::IDENTITY);
    let waypoints = [
        [0.32, 0.0, 0.14], // approach
        [0.32, 0.0, 0.05], // descend to grasp height
        [0.32, 0.0, 0.16], // lift / retreat
    ];
    for wp in waypoints {
        let input = inner.plan_input().unwrap();
        let goal = armd::kinematics::Pose::new(r_down, wp);
        let path = armd::motion::plan_cartesian_ex(&input, goal, armd::motion::Speed::default(), false)
            .unwrap_or_else(|e| panic!("B601-DM grasp waypoint {wp:?} failed: {e:?}"));
        let q_end = path.last_slice().to_vec();
        { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.install_path(__a, &input, path, "move_to", None)) } }.expect("install_path succeeds");
        inner.stop_motion();
        inner.commanded_positions = q_end.clone();
        inner.measured_positions = q_end;
    }

    // Negative 1: link3 mid-segment hits a fixture keepout box while end_link remains clear
    let q_l3_target: [f32; 7] = [0.0, -0.4, -0.8, 0.8, 0.0, 0.0, 0.0];
    let f_l3_target = ArmModel::REBOT_DEV_ARM.frames_f32(&q_l3_target).unwrap();
    let l3_pose = ArmModel::REBOT_DEV_ARM.resolve_frame("link3").unwrap().pose(&f_l3_target);
    let l3_mid_target = l3_pose.transform_point(&[0.121, -0.030, -0.0311]);
    let end_dist = kinematics::norm(&kinematics::sub(&l3_mid_target, &f_l3_target.end.p));
    assert!(end_dist > 0.12, "end_link must be far from link3 mid-segment box (dist={end_dist})");
    let mut cfg_mid_keepout = kit.clone();
    cfg_mid_keepout.workspace.as_mut().unwrap().keepout_boxes.push(armd::model::KeepoutBox {
        name: "table_fixture".into(),
        min_m: l3_mid_target.map(|v| (v - 0.015) as f32),
        max_m: l3_mid_target.map(|v| (v + 0.015) as f32),
    });
    let mut q_side = q_l3_target;
    q_side[0] = -1.2;
    assert_eval_clear(
        &ArmModel::REBOT_DEV_ARM,
        cfg_mid_keepout.workspace.as_ref().unwrap(),
        &cfg_mid_keepout.camera_mount,
        &q_side,
        "B601-DM side start",
    );
    let mut inner = armed(cfg_mid_keepout, &q_side);
    let err = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.jog(__a, q_l3_target.to_vec(), None, None)) } }.unwrap_err();
    let ArmError::Safety(SafetyError::WorkspaceViolation { detail, .. }) = &err else {
        panic!("expected WorkspaceViolation on link3 keepout hit, got {err:?}");
    };
    assert_eq!(detail.constraint_id.as_deref(), Some("keepout:table_fixture"), "{detail:?}");
    assert_eq!(detail.geometry_id.as_deref(), Some("capsule:link3"), "{detail:?}");

    // Negative 2: swept path across a keepout box hits link3 capsule at midpoint while BOTH endpoints & all capsules clear
    let mut q_left = q_l3_target;
    q_left[0] = -1.2;
    let mut q_right = q_l3_target;
    q_right[0] = 1.2;
    let q_mid: Vec<f32> = q_left.iter().zip(&q_right).map(|(a, b)| 0.5 * (a + b)).collect();
    let f_mid = ArmModel::REBOT_DEV_ARM.frames_f32(&q_mid).unwrap();
    let link3_pose = ArmModel::REBOT_DEV_ARM.resolve_frame("link3").unwrap().pose(&f_mid);
    let l3_mid = link3_pose.transform_point(&[0.121, -0.030, -0.0311]);
    // Confirm end_link is far from this mid-link keepout box across endpoints and midpoint
    for q in [&q_left[..], &q_mid[..], &q_right[..]] {
        let p_end = ArmModel::REBOT_DEV_ARM.fk_f32(q).unwrap().p;
        assert!(kinematics::norm(&kinematics::sub(&l3_mid, &p_end)) > 0.10, "end_link must stay clear of mid-link box");
    }
    let mut cfg_sweep = kit;
    cfg_sweep.workspace.as_mut().unwrap().keepout_boxes.push(armd::model::KeepoutBox {
        name: "swept_link3_obstacle".into(),
        min_m: l3_mid.map(|v| (v - 0.015) as f32),
        max_m: l3_mid.map(|v| (v + 0.015) as f32),
    });
    assert_eval_clear(
        &ArmModel::REBOT_DEV_ARM,
        cfg_sweep.workspace.as_ref().unwrap(),
        &cfg_sweep.camera_mount,
        &q_left,
        "B601-DM sweep start",
    );
    assert_eval_clear(
        &ArmModel::REBOT_DEV_ARM,
        cfg_sweep.workspace.as_ref().unwrap(),
        &cfg_sweep.camera_mount,
        &q_right,
        "B601-DM sweep end",
    );
    let mut inner = armed(cfg_sweep, &q_left);
    let err = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.jog(__a, q_right.to_vec(), None, None)) } }.unwrap_err();
    let ArmError::Safety(SafetyError::WorkspaceViolation { detail, .. }) = &err else {
        panic!("expected WorkspaceViolation on link sweep, got {err:?}");
    };
    assert_eq!(detail.geometry_id.as_deref(), Some("capsule:link3"), "{detail:?}");
    assert_eq!(detail.constraint_id.as_deref(), Some("keepout:swept_link3_obstacle"), "{detail:?}");
    assert_eq!(detail.violation_phase.as_deref(), Some("sweep"), "{detail:?}");
}

#[test]
#[allow(clippy::approx_constant)] // mesh origins copied from the URDF (3.14159, 1.5708)
fn so101_profile_link_capsules_positive_and_negative() {
    let mut config = ArmConfig::from_profile_file("profiles/so_arm101.json").expect("so_arm101.json loads");
    config.validate().expect("so_arm101.json validates");
    assert_eq!(config.profile_id.as_deref(), Some("so_arm101"));
    let capsules = &config.workspace.as_ref().unwrap().link_capsules;
    assert_eq!(capsules.len(), 4);

    // Enclosure verification against actual transformed URDF collision STL meshes in assets/so101/*.stl
    let urdf_text = include_str!("../assets/so101_new_calib.urdf");
    let link_meshes: &[(&str, &[(&str, [f64; 3], [f64; 3])])] = &[
        (
            "upper_arm_link",
            &[
                ("so101/sts3215_03a_v1.stl", [-0.11257, -0.0155, 0.0187], [-3.14159, 0.0, -1.5708]),
                ("so101/upper_arm_so101_v1.stl", [-0.065085, 0.012, 0.0182], [3.14159, 0.0, 0.0]),
            ],
        ),
        (
            "lower_arm_link",
            &[
                ("so101/under_arm_so101_v1.stl", [-0.0648499, -0.032, 0.0182], [3.14159, 0.0, 0.0]),
                ("so101/motor_holder_so101_wrist_v1.stl", [-0.0648499, -0.032, 0.018], [-3.14159, 0.0, 0.0]),
                ("so101/sts3215_03a_v1.stl", [-0.1224, 0.0052, 0.0187], [-3.14159, 0.0, -3.14159]),
            ],
        ),
        (
            "wrist_link",
            &[
                ("so101/sts3215_03a_no_horn_v1.stl", [0.0, -0.0424, 0.0306], [1.5708, 1.5708, 0.0]),
                ("so101/wrist_roll_pitch_so101_v2.stl", [0.0, -0.028, 0.0181], [-1.5708, -1.5708, 0.0]),
            ],
        ),
        (
            "gripper_link",
            &[
                ("so101/sts3215_03a_v1.stl", [0.0077, 0.0001, -0.0234], [-1.5708, 0.0, 0.0]),
                ("so101/wrist_roll_follower_so101_v1.stl", [0.0, -0.000218214, 0.000949706], [-3.14159, 0.0, 0.0]),
            ],
        ),
    ];
    let assets_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("assets");
    for (link_name, meshes) in link_meshes {
        assert!(urdf_text.contains(link_name));
        let cap = capsules.iter().find(|c| c.id == *link_name).unwrap_or_else(|| panic!("missing capsule {link_name}"));
        for (rel, xyz, rpy) in *meshes {
            let bytes = std::fs::read(assets_dir.join(rel)).unwrap();
            let tri_count = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
            let r = kinematics::rpy_to_r(rpy[0], rpy[1], rpy[2]);
            let mut max_d = 0.0f64;
            for i in 0..tri_count {
                let off = 84 + i * 50;
                for v_off in [12usize, 24, 36] {
                    let vx = f64::from(f32::from_le_bytes(bytes[off + v_off..off + v_off + 4].try_into().unwrap()));
                    let vy = f64::from(f32::from_le_bytes(bytes[off + v_off + 4..off + v_off + 8].try_into().unwrap()));
                    let vz = f64::from(f32::from_le_bytes(bytes[off + v_off + 8..off + v_off + 12].try_into().unwrap()));
                    let rp = kinematics::mat_vec(&r, &[vx, vy, vz]);
                    let p = [xyz[0] + rp[0], xyz[1] + rp[1], xyz[2] + rp[2]];
                    max_d = max_d.max(pt_seg_dist(p, cap.a_m, cap.b_m));
                }
            }
            assert!(
                max_d <= f64::from(cap.radius_m),
                "SO-101 capsule '{link_name}' misses mesh {rel}: max_dist={max_d:.5}m > r={:.5}m",
                cap.radius_m
            );
        }
    }

    // Uncalibrated B601-RS profile remains end_only (explicitly untouched)
    let mut rs = ArmConfig::from_profile_file("profiles/rebot_b601_rs.json").expect("b601_rs loads");
    rs.validate().expect("b601_rs validates");
    let rs_desc = SupervisorInner::new(rs).safety_describe();
    assert_eq!(rs_desc["limits"]["geometry_coverage"], "end_only");

    // SO-ARM101 safety_describe reports end_and_links
    let desc = SupervisorInner::new(config.clone()).safety_describe();
    assert_eq!(desc["limits"]["geometry_coverage"], "end_and_links");

    // Positive 1: full workspace/capsule check on home & park, plus home -> park and park -> home transitions
    let q_home = config.home_positions.clone();
    let q_park = config.resting_positions().to_vec();
    assert_eval_clear(
        &config.arm_model,
        config.workspace.as_ref().unwrap(),
        &config.camera_mount,
        &q_home,
        "SO-101 home",
    );
    assert_eval_clear(
        &config.arm_model,
        config.workspace.as_ref().unwrap(),
        &config.camera_mount,
        &q_park,
        "SO-101 park",
    );
    let mut inner = armed(config.clone(), &q_home);
    assert!({ let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.unload_action(__a)) } }.expect("home -> park passes").is_some());
    let mut inner = armed(config.clone(), &q_park);
    { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.jog(__a, q_home.clone(), None, None)) } }.expect("park -> home passes");

    // Positive 2: start from actual home pose -> jog to approach -> descend -> lift -> retreat
    let setup = config.grasp.as_ref().unwrap().clone();
    let r_down = armd::grasp::look_down(&kinematics::IDENTITY);
    let approach = armd::kinematics::Pose::new(r_down, [0.22, 0.0, setup.table_z_m + setup.approach_height_m]);
    let lo = std::array::from_fn(|i| f64::from(config.limits[i].min_position));
    let hi = std::array::from_fn(|i| f64::from(config.limits[i].max_position));
    let q_home6: [f64; 6] = std::array::from_fn(|i| f64::from(q_home[i]));
    let sol = config.arm_model.solve_dls_ex(&approach, &q_home6, &lo, &hi, kinematics::PAR.w, true);
    assert!(sol.ok, "approach IK succeeds");
    let q_app: Vec<f32> = sol.q.iter().map(|v| *v as f32).collect();
    let mut inner = armed(config.clone(), &q_home);
    { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.jog(__a, q_app.clone(), None, None)) } }.expect("SO-101 home -> approach passes");
    inner.stop_motion();
    inner.commanded_positions = q_app.clone();
    inner.measured_positions = q_app;
    for z in [setup.table_z_m + setup.grasp_height_m, setup.table_z_m + setup.lift_height_m, setup.table_z_m + setup.approach_height_m] {
        let input = inner.plan_input().unwrap();
        let goal = armd::kinematics::Pose::new(r_down, [0.22, 0.0, z]);
        let path = armd::motion::plan_cartesian_ex(&input, goal, armd::motion::Speed::default(), true)
            .unwrap_or_else(|e| panic!("SO-101 grasp z={z} failed: {e:?}"));
        let q_end = path.last_slice().to_vec();
        { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.install_path(__a, &input, path, "move_to", None)) } }.expect("SO-101 install_path succeeds");
        inner.stop_motion();
        inner.commanded_positions = q_end.clone();
        inner.measured_positions = q_end;
    }

    // Negative 1: mid-segment link capsule hits a tabletop fixture keepout box while TCP check_points stay clear
    let mut q_low = q_home.clone();
    q_low[1] = 0.7;
    q_low[2] = -0.3;
    q_low[3] = -0.9;
    let f_low = config.arm_model.frames_f32(&q_low).unwrap();
    let lower_pose = config.arm_model.resolve_frame("lower_arm_link").unwrap().pose(&f_low);
    let lower_mid_pt = lower_pose.transform_point(&[-0.067, 0.0026, 0.018]);
    assert!(
        kinematics::norm(&kinematics::sub(&lower_mid_pt, &f_low.end.p)) > 0.08,
        "end_link must stay clear of lower_arm_link mid-segment obstacle"
    );
    let mut cfg_mid_box = config.clone();
    cfg_mid_box.workspace.as_mut().unwrap().keepout_boxes.push(armd::model::KeepoutBox {
        name: "so101_table_fixture".into(),
        min_m: [lower_mid_pt[0] as f32 - 0.012, lower_mid_pt[1] as f32 - 0.012, 0.0],
        max_m: [lower_mid_pt[0] as f32 + 0.012, lower_mid_pt[1] as f32 + 0.012, lower_mid_pt[2] as f32 + 0.012],
    });
    let mut inner = armed(cfg_mid_box, &q_home);
    let err = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.jog(__a, q_low, None, None)) } }.unwrap_err();
    let ArmError::Safety(SafetyError::WorkspaceViolation { detail, .. }) = &err else {
        panic!("expected WorkspaceViolation on SO-101 link keepout hit, got {err:?}");
    };
    assert_eq!(detail.constraint_id.as_deref(), Some("keepout:so101_table_fixture"), "{detail:?}");
    assert_eq!(detail.geometry_id.as_deref(), Some("capsule:lower_arm_link"), "{detail:?}");

    // Negative 2: swept path crosses a keepout box with lower_arm_link capsule while BOTH endpoints & all capsules clear
    let mut q_left = q_home.clone();
    q_left[0] = -0.7;
    let mut q_right = q_home.clone();
    q_right[0] = 0.7;
    let q_mid: Vec<f32> = q_left.iter().zip(&q_right).map(|(a, b)| 0.5 * (a + b)).collect();
    let f_mid = config.arm_model.frames_f32(&q_mid).unwrap();
    let lower_pose = config.arm_model.resolve_frame("lower_arm_link").unwrap().pose(&f_mid);
    let lower_mid = lower_pose.transform_point(&[-0.065, 0.0035, 0.020]);
    for q in [&q_left[..], &q_mid[..], &q_right[..]] {
        let p_end = config.arm_model.fk_f32(q).unwrap().p;
        assert!(kinematics::norm(&kinematics::sub(&lower_mid, &p_end)) > 0.08, "end_link must clear obstacle");
    }
    let mut cfg_sweep = config;
    cfg_sweep.workspace.as_mut().unwrap().keepout_boxes.push(armd::model::KeepoutBox {
        name: "so101_mid_link_wall".into(),
        min_m: lower_mid.map(|v| (v - 0.012) as f32),
        max_m: lower_mid.map(|v| (v + 0.012) as f32),
    });
    assert_eval_clear(
        &cfg_sweep.arm_model,
        cfg_sweep.workspace.as_ref().unwrap(),
        &cfg_sweep.camera_mount,
        &q_left,
        "SO-101 sweep start",
    );
    assert_eval_clear(
        &cfg_sweep.arm_model,
        cfg_sweep.workspace.as_ref().unwrap(),
        &cfg_sweep.camera_mount,
        &q_right,
        "SO-101 sweep end",
    );
    let mut inner = armed(cfg_sweep, &q_left);
    let err = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.jog(__a, q_right, None, None)) } }.unwrap_err();
    let ArmError::Safety(SafetyError::WorkspaceViolation { detail, .. }) = &err else {
        panic!("expected WorkspaceViolation on SO-101 link sweep, got {err:?}");
    };
    assert_eq!(detail.geometry_id.as_deref(), Some("capsule:lower_arm_link"), "{detail:?}");
    assert_eq!(detail.constraint_id.as_deref(), Some("keepout:so101_mid_link_wall"), "{detail:?}");
    assert_eq!(detail.violation_phase.as_deref(), Some("sweep"), "{detail:?}");
}

#[test]
fn seg_seg_dist_covers_skew_parallel_crossing_and_degenerate_segments() {
    use armd::chain::seg_seg_dist;
    // Crossing orthogonal segments at (0.5, 0.5, 0)
    assert!(seg_seg_dist(&[0.0, 0.5, 0.0], &[1.0, 0.5, 0.0], &[0.5, 0.0, 0.0], &[0.5, 1.0, 0.0]) < 1e-12);
    // Skew orthogonal segments separated along Z by 0.25 m
    let d_skew = seg_seg_dist(&[0.0, 0.0, 0.0], &[1.0, 0.0, 0.0], &[0.5, -1.0, 0.25], &[0.5, 1.0, 0.25]);
    assert!((d_skew - 0.25).abs() < 1e-12, "d_skew={d_skew}");
    // Parallel overlapping segments separated by 0.3 m
    let d_par = seg_seg_dist(&[0.0, 0.0, 0.0], &[1.0, 0.0, 0.0], &[0.4, 0.3, 0.0], &[1.4, 0.3, 0.0]);
    assert!((d_par - 0.3).abs() < 1e-12, "d_par={d_par}");
    // Near-parallel crossing segments (denom < 1e-12 * a * e): midpoint intersection -> ~0 distance
    let d_near_par_cross = seg_seg_dist(&[0.0, 0.0, 0.0], &[1.0, 0.0, 0.0], &[0.2, -1e-8, 0.0], &[0.8, 1e-8, 0.0]);
    assert!(d_near_par_cross < 2e-8, "d_near_par_cross={d_near_par_cross}");
    // Near-parallel skew segments where right end is closer than left end (s=0 would overestimate)
    let d_near_par_skew = seg_seg_dist(&[0.0, 0.0, 0.0], &[1.0, 0.0, 0.0], &[0.0, 0.20, 0.0], &[1.0, 0.05, 0.0]);
    assert!((d_near_par_skew - 0.05).abs() < 1e-3 || d_near_par_skew < 0.06, "d_near_par_skew={d_near_par_skew}");
    let d_near_par_tiny = seg_seg_dist(&[0.0, 0.0, 0.0], &[1.0, 0.0, 0.0], &[0.2, 0.10, 0.0], &[0.8, 0.10 - 1e-8, 0.0]);
    assert!((d_near_par_tiny - (0.10 - 1e-8)).abs() < 1e-11, "d_near_par_tiny={d_near_par_tiny}");
    // Parallel disjoint segments: closest between (1,0,0) and (2,0.3,0)
    let d_disjoint = seg_seg_dist(&[0.0, 0.0, 0.0], &[1.0, 0.0, 0.0], &[2.0, 0.3, 0.0], &[3.0, 0.3, 0.0]);
    assert!((d_disjoint - (1.0f64 + 0.09).sqrt()).abs() < 1e-12, "d_disjoint={d_disjoint}");
    // Degenerate point-to-segment and point-to-point
    let d_pt_seg = seg_seg_dist(&[0.5, 0.4, 0.0], &[0.5, 0.4, 0.0], &[0.0, 0.0, 0.0], &[1.0, 0.0, 0.0]);
    assert!((d_pt_seg - 0.4).abs() < 1e-12, "d_pt_seg={d_pt_seg}");
    let d_pt_pt = seg_seg_dist(&[0.0, 0.0, 0.0], &[0.0, 0.0, 0.0], &[0.0, 3.0, 4.0], &[0.0, 3.0, 4.0]);
    assert!((d_pt_pt - 5.0).abs() < 1e-12, "d_pt_pt={d_pt_pt}");
}

#[test]
fn self_collision_validation_positive_negative_sweep_and_servo() {
    // 1. Config validation (fail-closed)
    let mut bad_unknown = ArmConfig::rebot_dm();
    bad_unknown.workspace.as_mut().unwrap().self_collision_enabled = false; // even when disabled!
    bad_unknown.workspace.as_mut().unwrap().self_collision_pairs = vec![["base_link".into(), "nonexistent".into()]];
    assert!(bad_unknown.validate().unwrap_err().contains("nonexistent"));

    let mut bad_self_pair = ArmConfig::rebot_dm();
    bad_self_pair.workspace.as_mut().unwrap().self_collision_pairs = vec![["link4".into(), "link4".into()]];
    assert!(bad_self_pair.validate().is_err());

    let mut bad_dup_pair = ArmConfig::rebot_dm();
    bad_dup_pair.workspace.as_mut().unwrap().self_collision_pairs = vec![
        ["base_link".into(), "link4".into()],
        ["link4".into(), "base_link".into()],
    ];
    assert!(bad_dup_pair.validate().is_err());

    let mut bad_clr = ArmConfig::rebot_dm();
    bad_clr.workspace.as_mut().unwrap().self_collision_min_clearance_m = -0.001;
    assert!(bad_clr.validate().is_err());
    bad_clr.workspace.as_mut().unwrap().self_collision_min_clearance_m = f32::NAN;
    assert!(bad_clr.validate().is_err());

    // 2. Positive: B601-DM home, park, inspect, and grasp have >= 0 clearance on all self-collision slots
    let dm = ArmConfig::rebot_dm();
    let ws_dm = dm.workspace.as_ref().unwrap();
    assert!(ws_dm.self_collision_enabled && !ws_dm.self_collision_pairs.is_empty());
    let eval_dm = armd::chain::WorkspaceEvaluator::new(&ArmModel::REBOT_DEV_ARM, ws_dm, &dm.camera_mount);
    assert_eq!(
        eval_dm.total_slots,
        eval_dm.self_collision_slot_start + ws_dm.self_collision_pairs.len()
    );
    let check_sc_nonneg = |eval: &armd::chain::WorkspaceEvaluator, q: &[f32], label: &str| {
        let mut buf = eval.alloc_buf();
        eval.eval_into(q, &mut buf, "test").unwrap();
        for slot in eval.self_collision_slot_start..eval.total_slots {
            let clr = buf.as_slice()[slot];
            assert!(clr >= 0.0, "{label}: negative self-collision clearance {clr} at slot {slot}");
        }
    };
    check_sc_nonneg(&eval_dm, &HOME, "B601-DM HOME");
    check_sc_nonneg(&eval_dm, dm.resting_positions(), "B601-DM PARK");
    check_sc_nonneg(&eval_dm, &INSPECT, "B601-DM INSPECT");

    // 3. Negative (B601-DM): folded self-collision pose triggers WORKSPACE_VIOLATION with self_collision:*
    // (keep end-effector well above floor_z=0 so self_collision is the only violated constraint)
    let q_fold_dm: [f32; 7] = [0.0, -0.1, -0.1, 1.5, 1.5, 0.0, 0.0];
    let mut c_home = eval_dm.alloc_buf();
    let mut c_fold = eval_dm.alloc_buf();
    eval_dm.eval_into(&HOME, &mut c_home, "plan").unwrap();
    eval_dm.eval_into(&q_fold_dm, &mut c_fold, "plan").unwrap();
    let mut md = eval_dm.init_min_depths(&c_home);
    let fail = eval_dm.check_and_advance(&c_fold, &mut md, "plan").unwrap_err();
    let cid = fail.detail.constraint_id.as_deref().unwrap();
    assert!(cid.starts_with("self_collision:"), "expected self_collision:*, got {fail:?}");
    assert_eq!(fail.detail.geometry_id.as_deref(), Some("capsule:base_link"));

    let mut inner = armed(dm.clone(), &HOME);
    let err = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.jog(__a, q_fold_dm.to_vec(), None, None)) } }.unwrap_err();
    let ArmError::Safety(SafetyError::WorkspaceViolation { detail, .. }) = &err else {
        panic!("expected WorkspaceViolation on B601-DM self-fold, got {err:?}");
    };
    assert!(detail.constraint_id.as_deref().is_some_and(|s| s.starts_with("self_collision:")), "{detail:?}");

    // 4. Swept self-collision (violation_phase == "sweep"): both endpoints clear, midpoint collides
    // Keep J2=-0.3, J3=-0.2, J5=1.5, and sweep J4 from 0.0 to 3.14 (inside the 3.14159 soft limit) so midpoint J4=1.57 swings into base_link.
    let q_sw_a: [f32; 7] = [0.0, -0.3, -0.2, 0.0, 1.5, 0.0, 0.0];
    #[allow(clippy::approx_constant)] // 3.14: just inside the soft limit, not pi
    let q_sw_b: [f32; 7] = [0.0, -0.3, -0.2, 3.14, 1.5, 0.0, 0.0];
    assert_eval_clear(&ArmModel::REBOT_DEV_ARM, ws_dm, &dm.camera_mount, &q_sw_a, "DM sweep start");
    assert_eval_clear(&ArmModel::REBOT_DEV_ARM, ws_dm, &dm.camera_mount, &q_sw_b, "DM sweep end");
    let mut inner_sw = armed(dm.clone(), &q_sw_a);
    let err = { let s = inner_sw.current_admission_stamp(); { let __s = s; inner_sw.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner_sw.jog(__a, q_sw_b.to_vec(), None, None)) } }.unwrap_err();
    let ArmError::Safety(SafetyError::WorkspaceViolation { detail, .. }) = &err else {
        panic!("expected WorkspaceViolation on swept self-collision, got {err:?}");
    };
    assert!(detail.constraint_id.as_deref().is_some_and(|s| s.starts_with("self_collision:")), "{detail:?}");
    assert_eq!(detail.violation_phase.as_deref(), Some("sweep"), "{detail:?}");

    // 5. Servo target rejection & monotonic escape from self-collision
    let q_edge_ok = lerp(&HOME, &q_fold_dm, 0.25);
    let mut prev_ok = q_edge_ok.clone();
    let mut first_bad = q_fold_dm.to_vec();
    for k in 1..=200 {
        let q = lerp(&HOME, &q_fold_dm, k as f32 / 200.0);
        let mut buf = eval_dm.alloc_buf();
        eval_dm.eval_into(&q, &mut buf, "test").unwrap();
        if buf.as_slice()[eval_dm.self_collision_slot_start..].iter().any(|&c| c < -kinematics::WORKSPACE_EPS) {
            first_bad = q;
            break;
        }
        prev_ok = q;
    }
    let mut inner_sv = armed(dm.clone(), &prev_ok);
    let epoch = inner_sv.safety.epoch_id();
    let err = { let s = inner_sv.current_admission_stamp(); { let __s = s; inner_sv.check_admission(__s).and_then(|__a| inner_sv.servo(__a, first_bad, epoch)) } }.unwrap_err();
    let SafetyError::WorkspaceViolation { detail, .. } = &err else { panic!("{err:?}") };
    assert!(detail.constraint_id.as_deref().is_some_and(|s| s.starts_with("self_collision:")), "{detail:?}");
    assert_eq!(detail.violation_phase.as_deref(), Some("servo_target"), "{detail:?}");
    // Monotonic escape: lifting J3 away from base_link strictly reduces all self-collision intrusions
    let mut q_retreat = q_fold_dm;
    q_retreat[2] -= 0.15;
    let mut inner_esc = armed(dm.clone(), &q_fold_dm);
    let epoch_esc = inner_esc.safety.epoch_id();
    { let s = inner_esc.current_admission_stamp(); { let __s = s; inner_esc.check_admission(__s).and_then(|__a| inner_esc.servo(__a, q_retreat.to_vec(), epoch_esc)) } }.expect("retreating out of self-collision must be allowed");

    // 6. self_collision_min_clearance_m margin shifts clearance and triggers when gap < min_clearance_m
    let mut dm_margin = dm;
    dm_margin.workspace.as_mut().unwrap().self_collision_min_clearance_m = 0.08;
    dm_margin.validate().unwrap();
    let mut inner_margin = armed(dm_margin, &INSPECT);
    let err = { let s = inner_margin.current_admission_stamp(); { let __s = s; inner_margin.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner_margin.jog(__a, HOME.to_vec(), None, None)) } }.unwrap_err();
    let ArmError::Safety(SafetyError::WorkspaceViolation { detail, .. }) = &err else {
        panic!("expected WorkspaceViolation when min_clearance_m=0.08 > HOME clearance, got {err:?}");
    };
    assert!(detail.constraint_id.as_deref().is_some_and(|s| s.starts_with("self_collision:")), "{detail:?}");

    // 7. SO-ARM101 positive (home, park) and negative (self-fold)
    let mut so = ArmConfig::from_profile_file("profiles/so_arm101.json").unwrap();
    so.validate().unwrap();
    let ws_so = so.workspace.as_ref().unwrap();
    assert!(ws_so.self_collision_enabled && !ws_so.self_collision_pairs.is_empty());
    let eval_so = armd::chain::WorkspaceEvaluator::new(&so.arm_model, ws_so, &so.camera_mount);
    check_sc_nonneg(&eval_so, &so.home_positions, "SO-101 HOME");
    check_sc_nonneg(&eval_so, so.resting_positions(), "SO-101 PARK");

    let q_fold_so = vec![0.0f32, -1.65, 1.55, 1.60, 0.0, 0.0];
    let mut inner_so = armed(so.clone(), so.resting_positions());
    let err = { let s = inner_so.current_admission_stamp(); { let __s = s; inner_so.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner_so.jog(__a, q_fold_so, None, None)) } }.unwrap_err();
    let ArmError::Safety(SafetyError::WorkspaceViolation { detail, .. }) = &err else {
        panic!("expected WorkspaceViolation on SO-101 self-fold, got {err:?}");
    };
    assert_eq!(detail.constraint_id.as_deref(), Some("self_collision:upper_arm_link:gripper_link"), "{detail:?}");
    assert_eq!(detail.geometry_id.as_deref(), Some("capsule:upper_arm_link"), "{detail:?}");
}

#[test]
fn link_frames_of_the_geometry_checks_follow_the_kinematic_correction() {
    // the link capsules and self-collision pairs are placed with `frames_f32` of the live model:
    // with a correction installed they move with it (same end pose as the corrected FK)
    let corr: kinematics::Correction = serde_json::from_value(serde_json::json!([
        {"offset_rad": 0.0, "scale": 0.0, "tilt_rad": [0.0, 0.0], "origin_m": [0.0, 0.0, 0.0]},
        {"offset_rad": 0.021, "scale": 0.012, "tilt_rad": [0.044, -0.014], "origin_m": [0.004, -0.006, 0.002]},
        {"offset_rad": -0.035, "scale": -0.02, "tilt_rad": [0.009, 0.019], "origin_m": [-0.012, 0.003, 0.008]},
        {"offset_rad": 0.054, "scale": 0.03, "tilt_rad": [-0.026, 0.012], "origin_m": [0.009, 0.011, -0.004]},
        {"offset_rad": -0.0698, "scale": -0.05, "tilt_rad": [0.05, -0.05], "origin_m": [-0.02, 0.015, 0.025]},
        {"offset_rad": 0.0, "scale": 0.0, "tilt_rad": [0.0, 0.0], "origin_m": [0.0, 0.0, 0.0]}])).unwrap();
    let nominal = ArmModel::REBOT_DEV_ARM;
    let corrected = nominal.with_correction(Some(&corr)).unwrap();
    let q = INSPECT;
    let (fn_, fc) = (nominal.frames_f32(&q).unwrap(), corrected.frames_f32(&q).unwrap());
    assert_eq!(fn_.end, nominal.fk_f32(&q).unwrap());
    assert_eq!(fc.end, corrected.fk_f32(&q).unwrap());
    let moved = kinematics::norm(&kinematics::sub(&fc.origins[4], &fn_.origins[4]));
    assert!(moved > 0.005, "J5 origin moved by {moved} m");
    // HOME (J6 = 0 in the current convention) is clear of the B601-DM self-collision pairs
    let dm = ArmConfig::rebot_dm();
    let ws = dm.workspace.as_ref().unwrap();
    assert!(ws.self_collision_covered());
    let eval = armd::chain::WorkspaceEvaluator::new(&dm.arm_model, ws, &dm.camera_mount);
    let mut buf = eval.alloc_buf();
    eval.eval_into(&HOME, &mut buf, "plan").unwrap();
    assert!(buf.as_slice().iter().all(|&c| c >= -kinematics::WORKSPACE_EPS), "HOME clearances {:?}", buf.as_slice());
}

/// Vertices of a link's cockpit mesh (kits/.../assets/meshes, the URDF visual meshes decimated to 0.5 mm).
fn b601dm_mesh_verts(link: &str) -> Vec<[f64; 3]> {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("kits/recamera-b601dm/packaging/rootfs/usr/share/rebot-teleop/assets/meshes");
    let manifest: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(dir.join("manifest.json")).unwrap()).unwrap();
    let bin = std::fs::read(dir.join("arm.bin")).unwrap();
    let mut out = Vec::new();
    for part in manifest["links"][link].as_array().unwrap() {
        let off = part["offset"].as_u64().unwrap() as usize;
        let bb: Vec<f64> = part["bbox"].as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();
        for i in 0..part["verts"].as_u64().unwrap() as usize {
            let q = |k: usize| f64::from(u16::from_le_bytes([bin[off + i * 6 + 2 * k], bin[off + i * 6 + 2 * k + 1]])) / 65535.0;
            out.push([0, 1, 2].map(|k| bb[k] + q(k) * (bb[k + 3] - bb[k])));
        }
    }
    out
}

/// The office B601-DM kinematic correction of 2026-11-20 (L2 fit): J2 offset/scale/tilt_x, J3-J5 offsets.
fn office_l2_correction() -> kinematics::Correction {
    let z = serde_json::json!({"offset_rad": 0.0, "scale": 0.0, "tilt_rad": [0.0, 0.0], "origin_m": [0.0, 0.0, 0.0]});
    let off = |o: f64| serde_json::json!({"offset_rad": o, "scale": 0.0, "tilt_rad": [0.0, 0.0], "origin_m": [0.0, 0.0, 0.0]});
    serde_json::from_value(serde_json::json!([
        z,
        {"offset_rad": 0.0226, "scale": 0.0189, "tilt_rad": [-0.01325, 0.0], "origin_m": [0.0, 0.0, 0.0]},
        off(0.010), off(0.0247), off(-0.0458), z,
    ]))
    .unwrap()
}

#[test]
fn rest_pose_neighbourhood_clears_self_collision_nominal_and_corrected() {
    // T21: with the live correction the folded rest pose (HOME = park = resting pose, every joint 0) sat
    // 1 mm inside the base_link-link5 bound, so every park and servo near rest was refused.
    let dm = ArmConfig::rebot_dm();
    assert_eq!(dm.home_positions, HOME.to_vec(), "home is the folded rest pose");
    let ws = dm.workspace.as_ref().unwrap();
    let corr = office_l2_correction();
    let corrected = ArmModel::REBOT_DEV_ARM.with_correction(Some(&corr)).unwrap();
    let step = 2.0f32.to_radians();
    for (label, model) in [("nominal", ArmModel::REBOT_DEV_ARM), ("office L2", corrected)] {
        let eval = armd::chain::WorkspaceEvaluator::new(&model, ws, &dm.camera_mount);
        let mut buf = eval.alloc_buf();
        let s = eval.self_collision_slot_start;
        eval.eval_into(&HOME, &mut buf, "plan").unwrap();
        let rest = buf.as_slice()[s..].iter().copied().fold(f64::INFINITY, f64::min);
        assert!(rest >= 0.015, "{label}: rest pose self-collision clearance {rest:.4} m");
        // every corner of the +-2 deg box around rest on J1..J6
        let mut worst = f64::INFINITY;
        for i in 0..3usize.pow(6) {
            let mut q = HOME;
            let mut k = i;
            for qj in q.iter_mut().take(6) {
                *qj = (k % 3) as f32 * step - step;
                k /= 3;
            }
            eval.eval_into(&q, &mut buf, "plan").unwrap();
            let all = buf.as_slice().iter().copied().fold(f64::INFINITY, f64::min);
            assert!(all >= -kinematics::WORKSPACE_EPS, "{label}: {q:?} violates the workspace ({all:.4} m)");
            worst = worst.min(buf.as_slice()[s..].iter().copied().fold(f64::INFINITY, f64::min));
        }
        assert!(worst >= 0.004, "{label}: +-2 deg self-collision clearance {worst:.4} m");
    }
    // the supervisor with the correction installed parks / jogs at rest
    let mut inner = armed(dm.clone(), &HOME);
    inner.set_kinematics_correction(Some(&corr)).unwrap();
    let near: Vec<f32> = HOME.iter().enumerate().map(|(j, _)| if j < 5 { step } else { 0.0 }).collect();
    { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.jog(__a, near, None, None)) } }.expect("jog within 2 deg of rest");
}

#[test]
fn base_link_link5_contact_is_still_refused() {
    // J4 = 1.6 rad from rest pitches the wrist down onto the pedestal: the meshes touch (vertex gap < 3 mm)
    let dm = ArmConfig::rebot_dm();
    let ws = dm.workspace.as_ref().unwrap();
    let q: [f32; 7] = [0.0, 0.0, 0.0, 1.6, 0.0, 0.0, 0.0];
    let base = b601dm_mesh_verts("base_link");
    let l5 = b601dm_mesh_verts("link5");
    let pair = ws.self_collision_pairs.iter().position(|[a, b]| a == "base_link" && b == "link5").unwrap();
    let corr = office_l2_correction();
    for (label, model) in [("nominal", ArmModel::REBOT_DEV_ARM), ("office L2", ArmModel::REBOT_DEV_ARM.with_correction(Some(&corr)).unwrap())] {
        let pose = model.resolve_frame("link5").unwrap().pose(&model.frames_f32(&q).unwrap());
        let gap = l5
            .iter()
            .map(|p| pose.transform_point(p))
            .flat_map(|w| base.iter().map(move |b| kinematics::norm(&kinematics::sub(&w, b))))
            .fold(f64::INFINITY, f64::min);
        assert!(gap < 0.003, "{label}: meshes {gap:.4} m apart");
        let eval = armd::chain::WorkspaceEvaluator::new(&model, ws, &dm.camera_mount);
        let mut buf = eval.alloc_buf();
        eval.eval_into(&q, &mut buf, "plan").unwrap();
        let clr = buf.as_slice()[eval.self_collision_slot_start + pair];
        assert!(clr < -0.05, "{label}: base_link-link5 clearance {clr:.4} m");
    }
    // 10 mm mesh gap (J4 = 1.0) is refused too: the capsules keep their margin
    let mut inner = armed(dm.clone(), &HOME);
    inner.set_kinematics_correction(Some(&corr)).unwrap();
    for j4 in [1.0f32, 1.6] {
        let err = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.jog(__a, vec![0.0, 0.0, 0.0, j4, 0.0, 0.0, 0.0], None, None)) } }.unwrap_err();
        let ArmError::Safety(SafetyError::WorkspaceViolation { detail, .. }) = &err else {
            panic!("J4 {j4}: expected WorkspaceViolation, got {err:?}");
        };
        assert!(detail.constraint_id.as_deref().is_some_and(|s| s.starts_with("self_collision:base_")), "J4 {j4}: {detail:?}");
    }
}
