//! Server-side Cartesian safety box (spec 1b).
mod common;

use armd::chain::ArmModel;
use armd::kinematics::{self, Violation};
use armd::model::{ArmConfig, ObservationSchema, WorkspacePolicy};
use armd::safety::SafetyError;
use armd::{ArmError, SupervisorInner};

const HOME: [f32; 7] = [0.0, 0.0, 0.0, 0.0, 0.0, std::f32::consts::FRAC_PI_2, 0.0];
const INSPECT: [f32; 7] = [0.0, -1.167, -1.565, 1.435, 0.0, std::f32::consts::FRAC_PI_2, 0.0];
// Both ends are above the floor; the joint-space midpoint puts a gripper corner ~27 mm below it.
// J6 motor = pi (URDF J6 = pi/2): the jaw axis is in the plane where the 92.5 mm corners dip.
const DIP_A: [f32; 7] = [0.0, -1.871, -0.64, 1.817, 0.464, std::f32::consts::PI, 0.0];
const DIP_B: [f32; 7] = [0.0, -1.856, -1.434, 0.482, -0.61, std::f32::consts::PI, 0.0];

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
    inner.set_torque(true).unwrap();
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
    let err = inner.servo(bad.clone(), epoch).unwrap_err();
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
    inner.servo(up, epoch).expect("moving out of the violation is allowed");
    // ... and one that makes it worse is not
    let mut inner = armed(ArmConfig::rebot_dm(), &lerp(&DIP_A, &DIP_B, 0.30));
    assert!(matches!(inner.servo(deeper, epoch), Err(SafetyError::WorkspaceViolation { .. })));
}

#[test]
fn jog_whose_path_dips_below_the_floor_is_rejected() {
    let mut inner = armed(ArmConfig::rebot_dm(), &DIP_A);
    let err = inner.jog(DIP_B.to_vec(), None, None).unwrap_err();
    assert!(matches!(err, ArmError::Safety(SafetyError::WorkspaceViolation { .. })), "{err:?}");
    assert!(inner.active_trajectory.is_none());
    // the same jog without a workspace policy is accepted
    let mut config = ArmConfig::rebot_dm();
    config.workspace = None;
    let mut inner = armed(config, &DIP_A);
    inner.jog(DIP_B.to_vec(), None, None).expect("no workspace, no check");
}

#[test]
fn home_from_inspect_is_accepted() {
    let mut inner = armed(ArmConfig::rebot_dm(), &INSPECT);
    inner.jog(HOME.to_vec(), None, None).expect("home from inspect");
    let mut inner = armed(ArmConfig::rebot_dm(), &HOME);
    inner.jog(INSPECT.to_vec(), None, None).expect("inspect from home");
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
    inner.servo(lerp(&DIP_A, &DIP_B, 0.05), epoch).unwrap();
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
    assert_eq!(reply["type"], "preset_ack", "park is exempt: {reply}");
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
    let err = inner.servo(q_up.to_vec(), epoch).unwrap_err();
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
    inner.servo(q_target.to_vec(), epoch).expect("target itself is above floor_z");
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
    cfg.workspace.as_mut().unwrap().link_capsules.push(armd::model::LinkCapsule {
        id: "upper_arm".into(),
        frame: "upper_arm".into(),
        a_m: [0.0, 0.0, 0.0],
        b_m: [-0.264, 0.0, 0.0],
        radius_m: 0.035,
    });
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
    let err = inner.jog(INSPECT.to_vec(), None, None).unwrap_err();
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
    // 7. park from violating outer boundary (floor_z + box + reach) still allowed;
    //    park crossing keep-out box still rejected with violation_phase == "park".
    let mut cfg = ArmConfig::rebot_dm();
    cfg.workspace.as_mut().unwrap().max_reach_m = Some(0.15); // INSPECT also violates max_reach
    let deeper = lerp(&DIP_A, &DIP_B, 0.5);
    let mut inner = armed(cfg.clone(), &deeper);
    assert!(inner.unload_action().unwrap().is_some(), "park from violating outer boundary must succeed");

    // Now add a keepout box on the park path from deeper -> HOME
    let mid: Vec<f32> = deeper.iter().zip(&cfg.home_positions).map(|(a, b)| 0.5 * (a + b)).collect();
    let p_mid = ArmModel::REBOT_DEV_ARM.fk_f32(&mid).unwrap().p;
    cfg.workspace.as_mut().unwrap().keepout_boxes.push(armd::model::KeepoutBox {
        name: "park_wall".into(),
        min_m: p_mid.map(|v| (v - 0.04) as f32),
        max_m: p_mid.map(|v| (v + 0.04) as f32),
    });
    let mut inner = armed(cfg, &deeper);
    let err = inner.unload_action().unwrap_err();
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
