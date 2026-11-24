//! Same-side (backlash-free) final approach (src/backlash.rs): pre-target, fallbacks, the
//! from-above rule of the grasp descent, action records, disabled = unchanged motions, and an
//! end-to-end move_to_pose on the mock follower.
mod common;

use std::sync::Arc;
use std::time::{Duration, Instant};
use serde_json::json;

use armd::backlash::{self, BacklashApproach, Pre, Request};
use armd::kinematics;
use armd::model::{ArmConfig, KeepoutBox};
use armd::motion::{self, GoalSpec, Speed};
use armd::{ArmSupervisor, SupervisorInner};
use common::INSPECT;

const DT: f64 = 0.02;
const READY: [f32; 7] = [0.3, -0.9, -1.3, 1.2, 0.1, 1.2, -1.0];

fn config(approach: Option<BacklashApproach>) -> ArmConfig {
    let mut c = ArmConfig::rebot_dm();
    c.backlash_approach = approach;
    c
}

/// The B601-DM profile with the approach on (the defaults: every arm joint, every motion).
fn on() -> ArmConfig { config(Some(BacklashApproach::default())) }

fn save_pose(inner: &mut SupervisorInner, name: &str, q: &[f32]) {
    armd::poses::save(inner, &json!({"name": name, "positions": q})).unwrap();
}

/// Ticks until action `id` ends; returns it and the commanded positions of every tick with its phase.
fn run(inner: &mut SupervisorInner, id: u64, max_s: f64) -> (motion::ActionStatus, Vec<(Option<String>, Vec<f32>)>) {
    let mut trace = Vec::new();
    for _ in 0..(max_s / DT) as usize {
        inner.tick(DT);
        let a = inner.last_action.clone().unwrap();
        trace.push((a.phase.as_deref().map(str::to_owned), inner.commanded_positions.clone()));
        if a.id == id && !a.is_running() { return (a, trace); }
    }
    panic!("action still running: {:?}", inner.last_action);
}

#[test]
fn kit_profile_enables_the_approach_and_publishes_it() {
    let c = ArmConfig::rebot_dm();
    let b = c.backlash_approach.as_ref().expect("rebot_dm has backlash_approach");
    assert!(b.enabled && (b.delta_deg - 0.5).abs() < 1e-12);
    assert_eq!(b.applies_to, ["move_to_pose", "grasp_final", "place_final", "move_to_precise"]);
    let schema = armd::model::ObservationSchema::from_config(&c);
    assert_eq!(schema.backlash_approach.as_ref(), Some(b));
    assert!(schema.capabilities.backlash_approach);
    // a bad section is refused at load
    let mut bad = c.clone();
    bad.backlash_approach = Some(BacklashApproach { joints: Some(vec!["gripper".into()]), ..Default::default() });
    assert!(bad.validate().is_err());
}

#[test]
fn move_to_pose_ends_from_the_same_side_with_an_approach_phase() {
    let mut inner = common::sim_at(on(), &INSPECT);
    // J1 near its lower soft limit: that joint approaches from above (sign -1)
    let mut target = READY;
    target[0] = on().limits[0].min_position + 0.004;
    save_pose(&mut inner, "edge", &target);
    let (id, duration) = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.move_to_pose(__a, "edge", false, 1.0, Some("t".into()), inner.safety.epoch_id())) } }.unwrap();
    let a = inner.last_action.clone().unwrap();
    assert_eq!(a.approach.as_deref(), Some("same_side"), "{a:?}");
    assert_eq!(a.approach_signs.as_deref(), Some(&[-1i8, 1, 1, 1, 1, 1][..]));
    assert_eq!(a.phase.as_deref(), Some("move"));
    let (a, trace) = run(&mut inner, id, duration + 4.0);
    assert_eq!(a.state, "done", "{a:?}");
    let q = &inner.commanded_positions;
    assert!(q[..6].iter().zip(&target[..6]).all(|(a, b)| (a - b).abs() < 1e-5), "{q:?}");
    // the short segment: phase approach, every joint monotonic in its direction
    let seg: Vec<&Vec<f32>> = trace.iter().filter(|(p, _)| p.as_deref() == Some("approach")).map(|(_, q)| q).collect();
    assert!(seg.len() >= 20, "segment of at least 0.5 s: {} ticks", seg.len());
    let delta = 0.5f32.to_radians();
    let signs = [-1.0f32, 1.0, 1.0, 1.0, 1.0, 1.0];
    for j in 0..6 {
        for w in seg.windows(2) { assert!((w[1][j] - w[0][j]) * signs[j] >= -1e-7, "joint {j} not monotonic"); }
        let travel = (target[j] - seg[0][j]) * signs[j];
        assert!(travel > 0.0 && travel <= delta + 1e-6, "joint {j} travel {travel}");
    }
    // expected duration covers both legs
    let ticks = trace.len() as f64 * DT;
    assert!(duration > 0.5 && ticks >= duration, "duration {duration}, ran {ticks}");
}

#[test]
fn direct_request_and_disabled_setting_record_and_move_as_before() {
    // disabled (None, enabled false, not listed): bit-identical commanded trajectories
    let traces: Vec<Vec<Vec<f32>>> = [
        None,
        Some(BacklashApproach { enabled: false, ..Default::default() }),
        Some(BacklashApproach { applies_to: vec!["grasp_final".into()], ..Default::default() }),
    ].into_iter().map(|b| {
        let mut inner = common::sim_at(config(b), &INSPECT);
        save_pose(&mut inner, "ready", &READY);
        let (id, d) = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.move_to_pose(__a, "ready", false, 1.0, None, inner.safety.epoch_id())) } }.unwrap();
        let (a, trace) = run(&mut inner, id, d + 4.0);
        assert_eq!((a.state.as_str(), a.approach.as_deref(), a.phase.as_deref()), ("done", None, None), "{a:?}");
        trace.into_iter().map(|(_, q)| q).collect()
    }).collect();
    assert_eq!(traces[0], traces[1]);
    assert_eq!(traces[0], traces[2]);
    // the request wins over the profile in both directions
    let mut inner = common::sim_at(on(), &INSPECT);
    save_pose(&mut inner, "ready", &READY);
    let (id, d) = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.move_to_pose_ex(__a, "ready", false, 1.0, None, inner.safety.epoch_id(), Request::Direct)) } }.unwrap();
    let (a, trace) = run(&mut inner, id, d + 4.0);
    assert_eq!((a.approach.as_deref(), a.approach_reason.as_deref()), (Some("direct"), Some("request")));
    assert_eq!(trace.into_iter().map(|(_, q)| q).collect::<Vec<_>>(), traces[0], "direct = the motion without the feature");
    let mut inner = common::sim_at(config(None), &INSPECT);
    save_pose(&mut inner, "ready", &READY);
    let (id, _) = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.move_to_pose_ex(__a, "ready", false, 1.0, None, inner.safety.epoch_id(), Request::SameSide)) } }.unwrap();
    assert_eq!(inner.last_action.as_ref().unwrap().approach.as_deref(), Some("same_side"));
    let _ = id;
}

#[test]
fn keepout_at_the_pre_target_falls_back_to_the_direct_approach() {
    // only J2 listed: its pre-target moves the TCP off the target; a box there blocks it
    let only_j2 = BacklashApproach { joints: Some(vec!["joint_2".into()]), ..Default::default() };
    let mut pre = READY;
    pre[1] -= 0.5f32.to_radians();
    let (p_t, p_pre) = (kinematics::fk(&kinematics::q6(&READY)).p, kinematics::fk(&kinematics::q6(&pre)).p);
    let d: Vec<f64> = (0..3).map(|k| (p_pre[k] - p_t[k]).abs()).collect();
    let m = d.iter().cloned().fold(0.0, f64::max);
    assert!(m > 1e-3, "J2 moves the TCP: {m}");
    // the box holds the pre-target TCP, not the target's (the axis of the largest offset)
    let h = 0.45 * m;
    let b = KeepoutBox { name: "pre".into(), min_m: p_pre.map(|v| (v - h) as f32), max_m: p_pre.map(|v| (v + h) as f32) };
    let mut c = config(Some(only_j2));
    c.workspace.as_mut().unwrap().keepout_boxes = vec![b];
    // start on the far side of the target (J2 above), so the direct move does not pass the box
    let mut start = READY;
    start[1] += 0.3;
    let mut inner = common::sim_at(c, &start);
    save_pose(&mut inner, "ready", &READY);
    let (id, d) = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.move_to_pose(__a, "ready", false, 1.0, None, inner.safety.epoch_id())) } }.unwrap();
    let a = inner.last_action.clone().unwrap();
    assert_eq!(a.approach.as_deref(), Some("direct"), "{a:?}");
    let reason = a.approach_reason.clone().unwrap();
    assert!(reason.contains("WORKSPACE_VIOLATION"), "{reason}");
    let (a, trace) = run(&mut inner, id, d + 4.0);
    assert_eq!(a.state, "done");
    assert!(trace.iter().all(|(p, _)| p.is_none()), "no approach phase");
}

/// Tool pointing down (end_link x = -z) 0.18 m above the base plane, as before a grasp descent.
fn tool_down_inner(c: ArmConfig) -> SupervisorInner {
    let mut inner = common::sim_at(c, &INSPECT);
    let input = inner.plan_input().unwrap();
    let goal = kinematics::Pose::new(kinematics::rpy_to_r(0.0, std::f64::consts::FRAC_PI_2, 0.0), [0.30, 0.02, 0.18]);
    let lo: [f64; 6] = std::array::from_fn(|i| f64::from(input.limits[i].min_position) + 0.05);
    let hi: [f64; 6] = std::array::from_fn(|i| f64::from(input.limits[i].max_position) - 0.05);
    let sol = input.arm.solve_dls(&goal, &input.q0, &lo, &hi, kinematics::PAR.w);
    assert!(sol.ok, "tool-down IK: {:?}", sol.ep);
    let mut q = INSPECT.to_vec();
    for i in 0..6 { q[i] = sol.q[i] as f32; }
    inner.commanded_positions = q.clone();
    inner.measured_positions = q.clone();
    inner.target_positions = q;
    inner
}

#[test]
fn grasp_final_never_approaches_from_below() {
    let inner = tool_down_inner(on());
    let input = inner.plan_input().unwrap();
    let p = input.arm.fk(&input.q0).p;
    let speed = Speed { linear: 0.02, angular: motion::DEFAULT_ANGULAR_RPS };
    let goal = motion::resolve_goal(&input, &GoalSpec::Absolute { p: [p[0], p[1], p[2] - 0.06], r: None });
    let direct = motion::plan_cartesian(&input, goal, speed).unwrap();
    let plan = inner.approach_plan("grasp_final", Request::Profile).unwrap();
    let tcp = [[0.0; 3]];
    let lowest = |q: &[f32]| backlash::lowest_z(&input.arm, &tcp, q).unwrap();
    // without the rule the plain pre-target (all joints -delta) puts the TCP lower
    let target = direct.last_slice().to_vec();
    let (plain, _) = backlash::pre_target(&target, &plan.joints, plan.policy.delta_rad(), &input.limits).unwrap();
    assert!(lowest(&plain) < lowest(&target) - backlash::BELOW_TOL_M, "the case needs a lowering joint");
    // the grasp rules: TCP from above, the gripper box corners keep the table clearance
    let mut corners = tcp.to_vec();
    corners.extend(input.ws.as_ref().unwrap().check_points_end_m.iter().map(|q| q.map(f64::from)));
    let table_bound = lowest(&target) - 0.03;
    let rules = backlash::DescentRules { table: Some((corners.clone(), table_bound)), ..backlash::DescentRules::tcp() };
    let cp = match backlash::plan_cartesian_pre(&input, &direct, &plan, Some(&rules), speed, false) {
        Pre::Ready(cp) => cp,
        other => panic!("pre-target refused: {other:?}"),
    };
    assert!(!cp.flipped.is_empty(), "the rule flipped at least one joint");
    let end = cp.path.last_slice().to_vec();
    let z_t = lowest(&target);
    for k in 0..=20 {
        let q: Vec<f32> = end.iter().zip(&target).map(|(a, b)| a + (b - a) * k as f32 / 20.0).collect();
        assert!(lowest(&q) >= z_t - backlash::BELOW_TOL_M, "segment point {k} below the target height");
    }
    for j in 0..6 {
        assert_eq!((target[j] - end[j]).signum() as i8, cp.signs[j], "joint {j} direction");
    }
    // a table just under the corners at the target refuses the approach (the clearance rule)
    let corner_low = backlash::lowest_z(&input.arm, &corners, &target).unwrap();
    let tight = backlash::DescentRules { table: Some((corners, corner_low + 1e-4)), ..backlash::DescentRules::tcp() };
    match backlash::plan_cartesian_pre(&input, &direct, &plan, Some(&tight), speed, false) {
        Pre::Direct(r) => assert!(r.starts_with("table"), "{r}"),
        other => panic!("expected a table refusal: {other:?}"),
    }
}

#[test]
fn macro_descent_records_the_outcome_and_runs_the_final_approach_phase() {
    // install_macro_path_same_side through begin_macro: the sub-motion runs path -> segment
    let mut inner = tool_down_inner(on());
    let input = inner.plan_input().unwrap();
    let p = input.arm.fk(&input.q0).p;
    let goal = motion::resolve_goal(&input, &GoalSpec::Absolute { p: [p[0], p[1], p[2] - 0.04], r: None });
    let speed = Speed { linear: 0.03, angular: motion::DEFAULT_ANGULAR_RPS };
    let direct = motion::plan_cartesian(&input, goal, speed).unwrap();
    let plan = inner.approach_plan("place_final", Request::Profile).unwrap();
    let pre = backlash::plan_cartesian_pre(&input, &direct, &plan, Some(&backlash::DescentRules::tcp()), speed, false);
    let target = direct.last_slice().to_vec();
    let id = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.begin_macro(__a, "place", &input, None)) } }.unwrap();
    { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.install_macro_path_same_side(__a, id, &input, direct, pre, Some(&plan))) } }.unwrap();
    let a = inner.last_action.clone().unwrap();
    assert_eq!(a.approach.as_deref(), Some("same_side"), "{a:?}");
    let mut phases = Vec::new();
    for _ in 0..600 {
        inner.tick(DT);
        let a = inner.last_action.clone().unwrap();
        if let Some(ph) = a.phase { if phases.last() != Some(&ph) { phases.push(ph); } }
        if matches!(inner.macro_sub(id), Some(armd::macro_action::SubState::Done(_))) { break; }
    }
    assert!(matches!(inner.macro_sub(id), Some(armd::macro_action::SubState::Done(_))), "sub-motion finished");
    assert_eq!(phases.last().map(|p| p.as_str()), Some("final_approach"), "{phases:?}");
    let q = &inner.commanded_positions;
    assert!(q[..6].iter().zip(&target[..6]).all(|(a, b)| (a - b).abs() < 1e-5));
    // a contact stop turns the record into direct / contact
    inner.approach_interrupted(id, "contact");
    let a = inner.last_action.clone().unwrap();
    assert_eq!((a.approach.as_deref(), a.approach_reason.as_deref()), (Some("direct"), Some("contact")));
}

#[test]
fn move_to_precise_ends_with_the_short_segment_and_plain_move_to_does_not() {
    for precise in [false, true] {
        let mut inner = common::sim_at(on(), &INSPECT);
        let input = inner.plan_input().unwrap();
        let p0 = input.arm.fk(&input.q0).p;
        let goal = motion::resolve_goal(&input, &GoalSpec::Absolute { p: [p0[0] + 0.04, p0[1] + 0.03, p0[2] - 0.03], r: None });
        let direct = motion::plan_cartesian(&input, goal, Speed::default()).unwrap();
        let target = direct.last_slice().to_vec();
        let plan = if precise { inner.approach_plan("move_to_precise", Request::Profile) } else { Err(None) };
        let pre = match &plan { Ok(p) => backlash::plan_cartesian_pre(&input, &direct, p, None, Speed::default(), false), _ => Pre::Off };
        let (id, d) = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.install_path_same_side(__a, &input, direct, pre, plan.as_ref().ok(), "move_to", None)) } }.unwrap();
        let (a, trace) = run(&mut inner, id, d + 4.0);
        assert_eq!(a.state, "done", "{a:?}");
        assert_eq!(a.approach.as_deref(), precise.then_some("same_side"));
        assert_eq!(trace.iter().any(|(p, _)| p.as_deref() == Some("approach")), precise);
        let q = &inner.commanded_positions;
        assert!(q[..6].iter().zip(&target[..6]).all(|(a, b)| (a - b).abs() < 1e-5), "precise {precise}: {q:?}");
    }
}

async fn wait_action(ws: &mut common::TestSocket, sup: &ArmSupervisor, generation: u64, epoch: u64, id: u64,
    mut each: impl FnMut(&armd::ArmObservation)) -> motion::ActionStatus {
    let deadline = Instant::now() + Duration::from_secs(20);
    let mut last_hb = Instant::now();
    loop {
        assert!(Instant::now() < deadline, "action {id} not finished");
        if last_hb.elapsed() > Duration::from_millis(200) {
            common::send(ws, json!({"type":"heartbeat", "generation":generation, "epoch_id":epoch})).await;
            last_hb = Instant::now();
        }
        common::drain(ws, 5).await;
        let obs = sup.get_observation().await;
        each(&obs);
        if let Some(a) = obs.action.filter(|a| a.id == id && !a.is_running()) { return a; }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn mock_follower_move_to_pose_ends_at_the_target_from_one_side() {
    let supervisor = Arc::new(ArmSupervisor::new(on()));
    let (mut ws, generation, epoch, supervisor, _addr) = common::torqued_rig(supervisor, &INSPECT).await;
    let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/poses/save", json!({"name": "ready", "positions": READY})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let schema: serde_json::Value = serde_json::to_value(supervisor.get_schema().await).unwrap();
    assert_eq!(schema["backlash_approach"]["delta_deg"], 0.5);
    // both sides: from INSPECT, then back from a pose past the target on every joint
    for (i, from_above) in [false, true].into_iter().enumerate() {
        if from_above {
            let past: Vec<f32> = READY.iter().enumerate().map(|(j, v)| if j < 6 { v + 0.1 } else { *v }).collect();
            let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/poses/save", json!({"name": "past", "positions": past})).await;
            assert_eq!(ack["type"], "command_ack", "{ack}");
            let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/action/move_to_pose", json!({"name": "past", "approach": "direct"})).await;
            let id = ack["action_id"].as_u64().unwrap();
            wait_action(&mut ws, &supervisor, generation, epoch, id, |_| {}).await;
        }
        let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/action/move_to_pose", json!({"name": "ready", "tag": format!("r{i}")})).await;
        assert_eq!(ack["type"], "command_ack", "{ack}");
        let id = ack["action_id"].as_u64().unwrap();
        let expected = ack["expected_duration_s"].as_f64().unwrap();
        let t0 = Instant::now();
        let mut seg: Vec<Vec<f32>> = Vec::new();
        let a = wait_action(&mut ws, &supervisor, generation, epoch, id, |o| {
            if o.action.as_ref().is_some_and(|a| a.id == id && a.phase.as_deref() == Some("approach")) { seg.push(o.positions.clone()); }
        }).await;
        assert_eq!((a.state.as_str(), a.approach.as_deref()), ("done", Some("same_side")), "{a:?}");
        assert!(t0.elapsed().as_secs_f64() >= expected * 0.8, "expected {expected} s");
        assert!(seg.len() >= 5, "approach samples {}", seg.len());
        let q = supervisor.get_observation().await.positions;
        assert!(q[..6].iter().zip(&READY[..6]).all(|(a, b)| (a - b).abs() < 1e-3), "{q:?}");
        // the last segment moves every joint in + (no joint near its lower limit)
        for j in 0..6 {
            for w in seg.windows(2) { assert!(w[1][j] - w[0][j] >= -1e-5, "run {i}: joint {j} reversed"); }
            assert!(READY[j] - seg[0][j] >= -1e-5, "run {i}: joint {j} starts above the target");
        }
    }
}
