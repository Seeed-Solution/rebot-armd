//! Keep-out boxes: profile list, runtime list over WS, persistence.

mod common;

use std::sync::Arc;
use serde_json::json;

use armd::kinematics;
use armd::model::{ArmConfig, KeepoutBox};
use armd::motion::{plan_cartesian, resolve_goal, GoalSpec, MotionError, Speed};
use armd::{ArmError, ArmSupervisor, SupervisorInner};
use common::{sim_at, INSPECT};

fn tcp(q: &[f32]) -> [f64; 3] { kinematics::fk(&kinematics::q6(q)).p }

/// 2 cm cube centred at `c` (base frame).
fn cube(name: &str, c: [f64; 3]) -> KeepoutBox {
    KeepoutBox { name: name.into(), min_m: c.map(|v| (v - 0.01) as f32), max_m: c.map(|v| (v + 0.01) as f32) }
}

fn with_boxes(boxes: Vec<KeepoutBox>) -> ArmConfig {
    let mut config = ArmConfig::rebot_dm();
    config.workspace.as_mut().unwrap().keepout_boxes = boxes;
    config
}

#[test]
fn profile_keepout_rejects_a_move_to_through_the_box() {
    let p0 = tcp(&INSPECT);
    let goal = [p0[0] + 0.08, p0[1], p0[2]];
    let mid = [p0[0] + 0.04, p0[1], p0[2]];
    let plan = |config: ArmConfig| {
        let inner = sim_at(config, &INSPECT);
        let input = inner.plan_input().unwrap();
        let g = resolve_goal(&input, &GoalSpec::Absolute { p: goal, r: None });
        plan_cartesian(&input, g, Speed::default())
    };
    assert!(plan(ArmConfig::rebot_dm()).is_ok());
    match plan(with_boxes(vec![cube("fixture", mid)])) {
        Err(MotionError::WorkspaceViolation { limit, s, .. }) => {
            assert_eq!(limit, "keepout 'fixture'");
            assert!(s > 0.2 && s < 0.8, "violation in the middle of the path: s={s}");
        }
        other => panic!("expected WORKSPACE_VIOLATION, got {other:?}"),
    }
    // A box beside the path does not interfere.
    assert!(plan(with_boxes(vec![cube("aside", [mid[0], mid[1] + 0.3, mid[2]])])).is_ok());
}

#[test]
fn keepout_applies_to_joint_moves_and_reports_the_box_name() {
    let mut inner = sim_at(with_boxes(vec![cube("bin_a", tcp(&INSPECT))]), &[0.0; 7]);
    let err = inner.jog_action(INSPECT.to_vec(), None, None, "joint_move").unwrap_err();
    let ArmError::Safety(e) = err else { panic!("{err:?}") };
    let api = armd::api::rest::ApiError::from(ArmError::Safety(e.clone()));
    assert_eq!(api.code.as_deref(), Some("WORKSPACE_VIOLATION"));
    assert!(e.to_string().contains("keepout 'bin_a'"), "{e}");
}

#[test]
fn keepout_lists_are_validated() {
    let ok = cube("a", [0.3, 0.0, 0.2]);
    let mut flat = ok.clone();
    flat.max_m[2] = flat.min_m[2];
    assert!(KeepoutBox::validate_list(&[flat]).is_err());
    assert!(KeepoutBox::validate_list(&[KeepoutBox { name: String::new(), ..ok.clone() }]).is_err());
    assert!(KeepoutBox::validate_list(&vec![ok.clone(); 17]).is_err());
    assert!(KeepoutBox::validate_list(&vec![ok.clone(); 16]).is_ok());
    let mut config = with_boxes(vec![ok]);
    config.validate().unwrap();
    config.workspace.as_mut().unwrap().keepout_boxes[0].min_m[0] = f32::NAN;
    assert!(config.validate().is_err());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn runtime_keepout_adds_to_the_profile_list_and_is_persisted() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("keepout.json");
    let supervisor = Arc::new(ArmSupervisor::new(with_boxes(vec![cube("profile_box", [0.3, 0.3, 0.3])])));
    armd::keepout::load_into(&supervisor, &path).await;
    let (mut ws, generation, epoch, _supervisor, addr) = common::torqued_rig(supervisor, &INSPECT).await;
    let get = || async { reqwest::get(format!("http://{addr}/api/v1/workspace/keepout")).await.unwrap().json::<serde_json::Value>().await.unwrap() };
    let state = get().await;
    assert_eq!(state["source"], "profile");
    assert_eq!(state["boxes"][0]["name"], "profile_box");
    assert_eq!(state["boxes"][0]["source"], "profile");

    let p0 = tcp(&INSPECT);
    let mid = [p0[0] + 0.04, p0[1], p0[2]];
    let b = cube("tray", mid);
    let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/workspace/keepout", json!({"boxes": [b]})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    assert_eq!(ack["source"], "runtime");
    assert_eq!(ack["persisted"], true);
    let state = get().await;
    assert_eq!(state["source"], "runtime");
    assert_eq!(state["boxes"].as_array().unwrap().len(), 2, "{state}");
    assert_eq!((&state["boxes"][0]["name"], &state["boxes"][0]["source"]), (&json!("profile_box"), &json!("profile")));
    assert_eq!((&state["boxes"][1]["name"], &state["boxes"][1]["source"]), (&json!("tray"), &json!("runtime")));
    let saved: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    assert_eq!(saved["boxes"].as_array().unwrap().len(), 1, "only the runtime list is persisted: {saved}");
    assert_eq!(saved["boxes"][0]["name"], "tray");

    let reply = common::rpc(&mut ws, generation, epoch, "/api/v1/action/move_to",
        json!({"position": [p0[0] + 0.08, p0[1], p0[2]], "orientation": null})).await;
    assert_eq!(reply["type"], "command_reject", "{reply}");
    assert_eq!(reply["code"], "WORKSPACE_VIOLATION");
    assert!(reply["message"].as_str().unwrap().contains("keepout 'tray'"), "{reply}");

    // Invalid list: refused, previous list kept.
    let reply = common::rpc(&mut ws, generation, epoch, "/api/v1/workspace/keepout", json!({"boxes": vec![b.clone(); 17]})).await;
    assert_eq!(reply["code"], "INVALID_COMMAND", "{reply}");
    assert_eq!(get().await["boxes"][1]["name"], "tray");
    // An empty runtime list removes no profile box.
    let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/workspace/keepout", json!({"boxes": []})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let state = get().await;
    assert_eq!(state["boxes"].as_array().unwrap().len(), 1, "{state}");
    assert_eq!(state["boxes"][0]["name"], "profile_box");
    let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/workspace/keepout", json!({"boxes": [b]})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");

    // A new start loads the runtime list.
    let again = ArmSupervisor::new(ArmConfig::rebot_dm());
    armd::keepout::load_into(&again, &path).await;
    let state = again.inner().read().await.keepout_state();
    assert_eq!((state.source, state.boxes[0].name.as_str()), ("runtime", "tray"));
}

#[tokio::test]
async fn corrupt_keepout_file_is_ignored_and_moved_aside() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("keepout.json");
    std::fs::write(&path, "{\"boxes\": [ broken").unwrap();
    let supervisor = ArmSupervisor::new(with_boxes(vec![cube("p", [0.3, 0.3, 0.3])]));
    armd::keepout::load_into(&supervisor, &path).await;
    let state = supervisor.inner().read().await.keepout_state();
    assert_eq!((state.source, state.boxes.len()), ("profile", 1));
    assert!(!path.exists());
    let bad: Vec<_> = std::fs::read_dir(dir.path()).unwrap().map(|e| e.unwrap().path()).collect();
    assert_eq!(bad.len(), 1, "{bad:?}");
    assert!(bad[0].file_name().unwrap().to_string_lossy().starts_with("keepout.json.bad-"), "{bad:?}");
    assert_eq!(std::fs::read_to_string(&bad[0]).unwrap(), "{\"boxes\": [ broken");
}

/// A keep-out box on the park path from INSPECT (at the midpoint of the joint-space move home).
fn park_blocking_box(inner: &SupervisorInner) -> KeepoutBox {
    let mid: Vec<f32> = INSPECT.iter().zip(&inner.config.home_positions).map(|(a, b)| (a + b) / 2.0).collect();
    let mut b = cube("on_park_path", tcp(&mid));
    b.min_m = b.min_m.map(|v| v - 0.03);
    b.max_m = b.max_m.map(|v| v + 0.03);
    b
}

#[test]
fn explicit_park_through_a_keepout_box_is_workspace_violation() {
    let probe = sim_at(ArmConfig::rebot_dm(), &INSPECT);
    let mut inner = sim_at(with_boxes(vec![park_blocking_box(&probe)]), &INSPECT);
    let err = inner.unload_action().unwrap_err();
    let api = armd::api::rest::ApiError::from(err);
    assert_eq!(api.code.as_deref(), Some("WORKSPACE_VIOLATION"), "{api:?}");
    assert!(api.message.contains("keepout 'on_park_path'"), "{}", api.message);
    assert!(!inner.pending_normal_unload && inner.active_trajectory.is_none(), "park must not start");
    assert!(inner.is_torqued, "torque held");
    // Without the box the same park starts.
    let mut inner = sim_at(ArmConfig::rebot_dm(), &INSPECT);
    assert!(inner.unload_action().unwrap().is_some());
}

#[test]
fn watchdog_park_through_a_keepout_box_holds_and_latches_watchdog_timeout() {
    let probe = sim_at(ArmConfig::rebot_dm(), &INSPECT);
    let mut inner = sim_at(with_boxes(vec![park_blocking_box(&probe)]), &INSPECT);
    inner.lease_enabled = true;
    inner.lease_timeout = std::time::Duration::from_millis(10);
    inner.last_heartbeat = std::time::Instant::now() - std::time::Duration::from_secs(1);
    inner.tick(0.02);
    assert!(matches!(inner.safety.state(), armd::SafetyState::Fault(armd::safety::FaultReason::WatchdogTimeout)), "{:?}", inner.safety.state());
    assert!(!inner.pending_normal_unload && inner.active_trajectory.is_none() && inner.is_torqued);
    let moved = INSPECT.iter().zip(&inner.commanded_positions).map(|(a, b)| (a - b).abs()).fold(0.0f32, f32::max);
    assert!(moved < 1e-3, "held in place ({moved} rad)");
}

/// A servo target outside every box whose stepped path crosses one: each tick's step is
/// checked, the stream stops in the recoverable hold before the box.
#[test]
fn servo_stream_cannot_sweep_through_a_keepout_box() {
    let (mut target, mut mid) = (INSPECT, INSPECT);
    // one frame (servo targets are at most 0.35 rad from the commanded pose)
    target[0] += 0.3;
    mid[0] += 0.15;
    let c = tcp(&mid);
    let b = KeepoutBox { name: "between".into(), min_m: c.map(|v| (v - 0.02) as f32), max_m: c.map(|v| (v + 0.02) as f32) };
    let mut inner = sim_at(with_boxes(vec![b]), &INSPECT);
    let epoch = inner.safety.epoch_id();
    inner.servo(target.to_vec(), epoch).expect("the target itself is outside the box");
    let ws = inner.config.workspace.clone().unwrap();
    for _ in 0..300 {
        inner.tick(0.02);
        let v = inner.config.arm_model.workspace_violation(&inner.commanded_positions, &ws, &inner.config.camera_mount);
        assert!(v.total <= 1e-9, "commanded pose entered the box: {:?}", inner.commanded_positions);
    }
    assert!(inner.servo_blocked_hold && inner.servo_target.is_none(), "stream must end in the hold: q={:?} target={:?} limit={:?} state={:?}", inner.commanded_positions, inner.servo_target, inner.motion_limit, inner.safety.state());
    assert!((inner.commanded_positions[0] - INSPECT[0]).abs() > 0.03, "moved towards the box first");
    let limit = inner.motion_limit.as_ref().expect("motion_limit");
    assert_eq!(limit.reason, armd::model::MotionLimitReason::Workspace);
}

/// The runtime list adds to the profile list: a profile box stays enforced whatever the
/// runtime list is, and the combined list is capped.
#[test]
fn runtime_keepout_is_additive_with_the_profile() {
    let p0 = tcp(&INSPECT);
    let mid = [p0[0] + 0.04, p0[1], p0[2]];
    let mut inner = sim_at(with_boxes(vec![cube("fixture", mid)]), &INSPECT);
    inner.set_keepout_boxes(vec![cube("aside", [mid[0], mid[1] + 0.3, mid[2]])]).unwrap();
    let names: Vec<_> = inner.config.workspace.as_ref().unwrap().keepout_boxes.iter().map(|b| b.name.clone()).collect();
    assert_eq!(names, ["fixture", "aside"]);
    inner.set_keepout_boxes(vec![]).unwrap();
    let input = inner.plan_input().unwrap();
    let g = resolve_goal(&input, &GoalSpec::Absolute { p: [p0[0] + 0.08, p0[1], p0[2]], r: None });
    match plan_cartesian(&input, g, Speed::default()) {
        Err(MotionError::WorkspaceViolation { limit, .. }) => assert_eq!(limit, "keepout 'fixture'"),
        other => panic!("profile box must stay in force, got {other:?}"),
    }
    let sixteen: Vec<_> = (0..16).map(|i| cube(&format!("r{i}"), [0.3, 0.3, 0.05 * f64::from(i)])).collect();
    assert!(matches!(inner.set_keepout_boxes(sixteen), Err(ArmError::Motion(MotionError::InvalidCommand(_)))), "17 in force");
    assert_eq!(inner.keepout_state().boxes.len(), 1);
}

/// Accepted motion was checked against the boxes in force: they cannot change under it, and a
/// plan made before a change is stale at install.
#[test]
fn keepout_change_is_busy_during_motion_and_makes_plans_stale() {
    let p0 = tcp(&INSPECT);
    let mut inner = sim_at(ArmConfig::rebot_dm(), &INSPECT);
    let mut q = INSPECT.to_vec();
    q[0] += 0.2;
    inner.jog(q, None, None).unwrap();
    let err = inner.set_keepout_boxes(vec![cube("late", [0.3, 0.3, 0.3])]).unwrap_err();
    assert_eq!(armd::api::rest::ApiError::from(err).code.as_deref(), Some("MOTION_BUSY"));
    assert!(inner.keepout_state().boxes.is_empty());

    let mut inner = sim_at(ArmConfig::rebot_dm(), &INSPECT);
    let input = inner.plan_input().unwrap();
    let g = resolve_goal(&input, &GoalSpec::Absolute { p: [p0[0] + 0.08, p0[1], p0[2]], r: None });
    let path = plan_cartesian(&input, g, Speed::default()).unwrap();
    // the box lands on the planned path after planning
    inner.set_keepout_boxes(vec![cube("late", [p0[0] + 0.04, p0[1], p0[2]])]).unwrap();
    let err = inner.install_path(&input, path, "move_to", None).unwrap_err();
    assert_eq!(armd::api::rest::ApiError::from(err).code.as_deref(), Some("PLAN_STALE"));
    assert!(inner.active_trajectory.is_none());
}

/// A small box between two of the 21 planner samples of a long joint move is still found.
#[test]
fn joint_move_keepout_sampling_is_dense_enough_for_small_boxes() {
    let mut target = INSPECT;
    target[0] += 1.2;
    let mut probe = sim_at(ArmConfig::rebot_dm(), &INSPECT);
    probe.jog(target.to_vec(), None, None).unwrap();
    let traj = probe.active_trajectory.as_ref().unwrap().as_quintic().unwrap().clone();
    let between = traj.sample(traj.duration * 10.5 / 20.0).positions;
    let c = tcp(&between);
    let small = KeepoutBox { name: "pin".into(), min_m: c.map(|v| (v - 0.006) as f32), max_m: c.map(|v| (v + 0.006) as f32) };
    // the 21 samples miss it
    let samples = SupervisorInner::trajectory_samples(&traj);
    let ws = { let mut w = probe.config.workspace.clone().unwrap(); w.keepout_boxes = vec![small.clone()]; w };
    assert!(samples.iter().all(|q| probe.config.arm_model.workspace_violation(q, &ws, &probe.config.camera_mount).total == 0.0), "box must sit between samples");
    let mut inner = sim_at(with_boxes(vec![small]), &INSPECT);
    let err = inner.jog(target.to_vec(), None, None).unwrap_err();
    assert!(err.to_string().contains("keepout 'pin'"), "{err}");
    assert!(inner.active_trajectory.is_none());

    // a box far from the path costs no subdivision and is accepted
    let far = KeepoutBox { name: "far".into(), min_m: [0.9, 0.9, 0.9], max_m: [0.9001, 0.9001, 0.9001] };
    let mut inner = sim_at(with_boxes(vec![far]), &INSPECT);
    inner.jog(target.to_vec(), None, None).expect("a box out of reach does not block the move");

    // a path whose check needs more FK evaluations than the plan budget is refused with a clear message
    let near = KeepoutBox { name: "near".into(), min_m: c.map(|v| (v + 0.003) as f32), max_m: c.map(|v| (v + 0.02) as f32) };
    let inner = sim_at(with_boxes(vec![near]), &INSPECT);
    let err = inner.workspace_samples_with_budget(&traj, 4).unwrap_err();
    let api = armd::api::rest::ApiError::from(armd::ArmError::from(err));
    assert_eq!(api.code.as_deref(), Some("WORKSPACE_VIOLATION"));
    assert!(api.message.contains("more than 4 FK evaluations"), "{}", api.message);
}

/// Workspace check points of `q` in the base frame (TCP, envelope corners, camera point).
fn check_points(inner: &SupervisorInner, q: &[f32]) -> Vec<[f64; 3]> {
    let ws = inner.config.workspace.as_ref().unwrap();
    let pose = inner.config.arm_model.fk_f32(q).unwrap();
    std::iter::once([0.0f32; 3]).chain(ws.check_points_end_m.iter().copied()).chain(inner.config.camera_mount.offset_m)
        .map(|p| pose.transform_point(&p.map(f64::from))).collect()
}

/// The sample set of the former joint-move check: uniform, refined until no check point
/// moves more than half the smallest box dimension (`spacing`) between two samples.
fn spacing_samples(inner: &SupervisorInner, traj: &armd::trajectory::MultiJointQuinticTrajectory, spacing: f64) -> Vec<Vec<f32>> {
    let uniform = |n: usize| -> Vec<Vec<f32>> { (0..=n).map(|k| traj.sample(traj.duration * k as f64 / n as f64).positions).collect() };
    let mut n = 20;
    loop {
        let s = uniform(n);
        let step = s.windows(2).map(|w| check_points(inner, &w[0]).iter().zip(check_points(inner, &w[1]))
            .map(|(a, b)| kinematics::norm(&kinematics::sub(a, &b))).fold(0.0, f64::max)).fold(0.0, f64::max);
        if step <= spacing { return s; }
        n = ((n as f64 * step / spacing * 1.2).ceil() as usize).max(n + 1);
    }
}

fn clear_of(inner: &SupervisorInner, b: &KeepoutBox, samples: &[Vec<f32>]) -> bool {
    let ws = { let mut w = inner.config.workspace.clone().unwrap(); w.keepout_boxes = vec![b.clone()]; w };
    samples.iter().all(|q| inner.config.arm_model.workspace_violation(q, &ws, &inner.config.camera_mount).total == 0.0)
}

/// A 2 cm box whose corner the TCP path clips between two samples of the former
/// half-box-dimension sampling is found.
#[test]
fn joint_move_keepout_check_catches_a_clipped_box_corner() {
    let mut target = INSPECT;
    target[0] += 1.2;
    target[1] -= 0.3;
    let mut probe = sim_at(ArmConfig::rebot_dm(), &INSPECT);
    probe.jog(target.to_vec(), None, None).unwrap();
    let traj = probe.active_trajectory.as_ref().unwrap().as_quintic().unwrap().clone();
    let (dim, inside) = (0.02f64, 0.0005f64);
    let old = spacing_samples(&probe, &traj, 0.5 * dim);
    let n = old.len() - 1;
    let mut found = None;
    'search: for k in 0..n {
        let (ta, tb) = (traj.duration * k as f64 / n as f64, traj.duration * (k + 1) as f64 / n as f64);
        let (pa, pm, pb) = (tcp(&old[k]), tcp(&traj.sample(0.5 * (ta + tb)).positions), tcp(&old[k + 1]));
        let (u, w) = (kinematics::sub(&pa, &pm), kinematics::sub(&pb, &pm));
        for i in 0..3 {
            for j in (0..3).filter(|&j| j != i) {
                let (mut lo, mut hi) = (pm.map(|v| v - 0.5 * dim), pm.map(|v| v + 0.5 * dim));
                for (ax, d) in [(i, u[i]), (j, w[j])] {
                    if d > 0.0 { hi[ax] = pm[ax] + inside; lo[ax] = hi[ax] - dim; } else { lo[ax] = pm[ax] - inside; hi[ax] = lo[ax] + dim; }
                }
                let b = KeepoutBox { name: "corner".into(), min_m: lo.map(|v| v as f32), max_m: hi.map(|v| v as f32) };
                if clear_of(&probe, &b, &old) { found = Some(b); break 'search; }
            }
        }
    }
    let b = found.expect("a box corner between two samples of the former sampling");
    let mut inner = sim_at(with_boxes(vec![b]), &INSPECT);
    let err = inner.jog(target.to_vec(), None, None).unwrap_err();
    assert!(err.to_string().contains("keepout 'corner'"), "{err}");
    assert!(inner.active_trajectory.is_none());
}

/// One servo step whose TCP passes a narrow box between its start, midpoint and end is
/// refused; a step clear of the box is not.
#[test]
fn servo_step_check_catches_a_box_between_its_samples() {
    let mut next = INSPECT;
    next[0] += 0.06;
    next[1] -= 0.04;
    let lerp = |t: f32| -> Vec<f32> { INSPECT.iter().zip(&next).map(|(a, b)| a + t * (b - a)).collect() };
    let c = tcp(&lerp(0.25));
    let b = KeepoutBox { name: "narrow".into(), min_m: c.map(|v| (v - 0.0005) as f32), max_m: c.map(|v| (v + 0.0005) as f32) };
    let probe = sim_at(ArmConfig::rebot_dm(), &INSPECT);
    assert!(clear_of(&probe, &b, &[INSPECT.to_vec(), lerp(0.5), next.to_vec()]), "box must sit between start, midpoint and end");
    let inner = sim_at(with_boxes(vec![b]), &INSPECT);
    let err = inner.servo_step_keepout(&next).unwrap_err();
    assert!(err.to_string().contains("keepout 'narrow'"), "{err}");
    let mut away = INSPECT;
    away[0] -= 0.06;
    inner.servo_step_keepout(&away).expect("a step away from the box is clear");
}

/// The keep-out ack is sent after the file write and can follow later replies (here the
/// E-stop's epoch_sync): it carries the epoch current when it is sent, never an older one.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn delayed_persist_ack_carries_the_epoch_at_send_time() {
    use futures_util::StreamExt;
    let dir = tempfile::tempdir().unwrap();
    let supervisor = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    armd::keepout::load_into(&supervisor, &dir.path().join("keepout.json")).await;
    let (mut ws, generation, epoch, sup, _) = common::torqued_rig(supervisor, &INSPECT).await;
    armd::persist::TEST_WRITE_DELAY_MS.store(300, std::sync::atomic::Ordering::Relaxed);
    common::send(&mut ws, json!({"type":"command", "request_id": 777, "generation": generation, "epoch_id": epoch,
        "path": "/api/v1/workspace/keepout", "body": {"boxes": [cube("t", [0.3, 0.3, 0.3])]}})).await;
    common::send(&mut ws, json!({"type":"estop"})).await;
    let (mut ack, mut sync) = (None, None);
    while ack.is_none() || sync.is_none() {
        let text = tokio::time::timeout(std::time::Duration::from_secs(3), ws.next()).await.unwrap().unwrap().unwrap().into_text().unwrap();
        let v: serde_json::Value = serde_json::from_str(&text).unwrap();
        match v["type"].as_str() {
            Some("command_ack") | Some("command_reject") if v["request_id"] == 777 => ack = Some((v, sync.is_some())),
            Some("epoch_sync") => sync = Some(v),
            _ => {}
        }
    }
    armd::persist::TEST_WRITE_DELAY_MS.store(0, std::sync::atomic::Ordering::Relaxed);
    let (ack, after_sync) = ack.unwrap();
    let sync = sync.unwrap();
    assert_eq!(ack["persisted"], true, "{ack}");
    let now = sup.get_observation().await.epoch_id;
    assert_eq!(sync["epoch_id"].as_u64(), Some(now));
    if after_sync {
        assert_eq!(ack["epoch_id"].as_u64(), Some(now), "ack after epoch_sync must not roll the epoch back: {ack}");
    }
    assert!(after_sync, "the ack is sent after the write, after the immediate E-stop reply");
}

/// The bounds the sweep check rests on: the joint travel bound covers the sampled travel,
/// and the per-joint reach covers each check point's distance from the joint axis.
#[test]
fn keepout_sweep_bounds_cover_the_motion() {
    let mut target = INSPECT;
    target[0] += 1.2;
    target[2] += 0.5;
    let mut probe = sim_at(ArmConfig::rebot_dm(), &INSPECT);
    probe.jog(target.to_vec(), None, None).unwrap();
    let traj = probe.active_trajectory.as_ref().unwrap().as_quintic().unwrap().clone();
    for k in 0..10 {
        let (t0, t1) = (traj.duration * k as f64 / 10.0, traj.duration * (k + 1) as f64 / 10.0);
        let bound = traj.travel_bound(t0, t1);
        let mut travel = vec![0.0f64; bound.len()];
        let mut prev = traj.sample(t0).positions;
        for s in 1..=100 {
            let q = traj.sample(t0 + (t1 - t0) * s as f64 / 100.0).positions;
            for j in 0..travel.len() { travel[j] += f64::from((q[j] - prev[j]).abs()); }
            prev = q;
        }
        for j in 0..travel.len() { assert!(travel[j] <= bound[j] + 1e-6, "joint {j} [{t0}, {t1}]: {} > {}", travel[j], bound[j]); }
    }
    let ws = probe.config.workspace.clone().unwrap();
    let local_max = std::iter::once([0.0f32; 3]).chain(ws.check_points_end_m.iter().copied()).chain(probe.config.camera_mount.offset_m)
        .map(|p| kinematics::norm(&p.map(f64::from))).fold(0.0, f64::max);
    let reach = probe.config.arm_model.joint_reach(local_max);
    for q in [INSPECT, target, [0.0; 7]] {
        let f = kinematics::frames(&kinematics::q6(&q));
        for p in check_points(&probe, &q) {
            for j in 0..6 {
                let rel = kinematics::sub(&p, &f.origins[j]);
                let along = kinematics::dot(&rel, &kinematics::unit(&f.axes[j]));
                let dist = (kinematics::dot(&rel, &rel) - along * along).max(0.0).sqrt();
                assert!(dist <= reach[j] + 1e-9, "joint {j}: {dist} > {}", reach[j]);
            }
        }
    }
}
