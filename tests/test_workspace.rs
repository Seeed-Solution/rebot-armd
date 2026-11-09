//! Server-side Cartesian safety box (spec 1b).
mod common;

use armd::kinematics::{self, workspace_violation};
use armd::model::{ArmConfig, ObservationSchema, WorkspacePolicy};
use armd::safety::SafetyError;
use armd::{ArmError, SupervisorInner};

const HOME: [f32; 7] = [0.0, 0.0, 0.0, 0.0, 0.0, std::f32::consts::FRAC_PI_2, 0.0];
const INSPECT: [f32; 7] = [0.0, -1.167, -1.565, 1.435, 0.0, std::f32::consts::FRAC_PI_2, 0.0];
// Both ends are above the floor; the joint-space midpoint puts a gripper corner ~27 mm below it.
const DIP_A: [f32; 7] = [0.0, -1.871, -0.64, 1.817, 0.464, std::f32::consts::FRAC_PI_2, 0.0];
const DIP_B: [f32; 7] = [0.0, -1.856, -1.434, 0.482, -0.61, std::f32::consts::FRAC_PI_2, 0.0];

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
    let home_pose = kinematics::fk(&kinematics::q6(&HOME));
    let down_in_end = kinematics::mat_vec(&kinematics::transpose(&home_pose.r), &[0.0, 0.0, -0.5]);
    cam_low.offset_m = Some(down_in_end.map(|v| v as f32)); // 0.5 m below end_link in the base frame
    let v = workspace_violation(&HOME, ws, &cam_low);
    assert_eq!(v.worst.unwrap().point, 9);
    // a pose moving further out is rejected, one moving back is allowed
    let (ok, bad) = floor_edge();
    let (vo, vb) = (workspace_violation(&ok, ws, cam), workspace_violation(&bad, ws, cam));
    assert!(kinematics::workspace_rejects(&vb, &vo));
    assert!(!kinematics::workspace_rejects(&vo, &vb));
}

#[test]
fn servo_frame_into_the_floor_is_rejected_and_moving_up_is_accepted() {
    let (ok, bad) = floor_edge();
    let mut inner = armed(ArmConfig::rebot_dm(), &ok);
    let epoch = inner.safety.epoch_id();
    let err = inner.servo(bad.clone(), epoch).unwrap_err();
    let SafetyError::WorkspaceViolation { limit, value, bound, point } = &err else { panic!("{err:?}") };
    assert_eq!(limit, "floor");
    assert!(*value < *bound && (1..=8).contains(point), "{err}");
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
