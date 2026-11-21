//! `yaw: "outward"` for move_to / move_relative / place (yaw = atan2(y, x) of the target, the
//! path's yaw follows atan2(y, x) of each sample) and the IK failure hint (B601-DM profile).
mod common;

use std::sync::Arc;
use std::time::{Duration, Instant};
use serde_json::{json, Value};

use armd::kinematics::{self, Pose};
use armd::motion::{self, GoalSpec, MotionError, PathOpts, PlanInput, Speed};
use armd::{ArmConfig, ArmSupervisor};

const PROFILE_PATH: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/kits/recamera-b601dm/profile/rebot-b601dm.json");

fn profile() -> ArmConfig {
    let mut config: ArmConfig = serde_json::from_reader(std::fs::File::open(PROFILE_PATH).unwrap()).unwrap();
    config.validate().unwrap();
    config
}

fn so101() -> ArmConfig {
    let mut config = ArmConfig::from_profile_file("profiles/so_arm101.json").unwrap();
    config.validate().unwrap();
    config
}

/// Plan input at the profile's home pose (the folded rest pose, every joint 0 on the B601-DM).
fn input_at(config: ArmConfig, q: Option<&[f64]>) -> PlanInput {
    let home: Vec<f32> = config.home_positions.clone();
    let mut q7 = [0.0f32; 7];
    for (i, v) in home.iter().enumerate().take(7) { q7[i] = *v; }
    if let Some(q) = q { for (i, v) in q.iter().enumerate() { q7[i] = *v as f32; } }
    common::sim_at(config, &q7).plan_input().unwrap()
}

const PITCH_40: f64 = 40.0 * std::f64::consts::PI / 180.0;

fn spec(p: [f64; 3], outward: bool) -> GoalSpec {
    let base = GoalSpec::Absolute { p, r: Some(kinematics::rpy_to_r(0.0, PITCH_40, 0.0)) };
    if outward { GoalSpec::YawOutward(Box::new(base)) } else { base }
}

fn plan(input: &PlanInput, spec: &GoalSpec) -> Result<motion::JointPath, MotionError> {
    let (goal, opts) = motion::resolve_goal_opts(input, spec);
    motion::plan_cartesian_opts(input, goal, Speed::default(), opts)
}

/// `input` moved to the end of `path`.
fn at_end(input: &PlanInput, path: &motion::JointPath) -> PlanInput {
    let mut next = input.clone();
    let last = path.last();
    next.q0 = std::array::from_fn(|i| if i < input.arm.arm_dof() { f64::from(last[i]) } else { 0.0 });
    next
}

/// Largest |J1 − atan2(y, x)| over the samples of `path` (y, x of the sample's FK).
fn j1_tracking(input: &PlanInput, path: &motion::JointPath) -> f64 {
    path.q.iter().map(|q| {
        let q6: [f64; 6] = std::array::from_fn(|i| if i < input.arm.arm_dof() { f64::from(q[i]) } else { 0.0 });
        let p = input.arm.fk(&q6).p;
        (q6[0] - p[1].atan2(p[0])).abs()
    }).fold(0.0, f64::max)
}

#[test]
fn fixed_yaw_lands_twisted_and_the_lateral_path_flips_with_a_hint() {
    let home = input_at(profile(), None);
    let first = plan(&home, &spec([0.18, 0.10, 0.30], false)).expect("reachable with yaw 0");
    let q = first.last();
    println!("yaw 0 at (0.18, 0.10, 0.30): q = {:?}", &q[..6]);
    let psi = 0.10f64.atan2(0.18);
    assert!((f64::from(q[0]) - psi).abs() > 0.3, "J1 {} away from atan2 {psi}: the reproduced twisted configuration", q[0]);
    let there = at_end(&home, &first);
    for y in [-0.10, 0.0] {
        let err = plan(&there, &spec([0.18, y, 0.30], false)).unwrap_err();
        let msg = err.to_string();
        println!("yaw 0 to (0.18, {y}, 0.30): {} {msg}", err.code());
        assert_eq!(err.code(), "IK_BRANCH_FLIP", "{msg}");
        assert!(msg.contains("yaw differs from the outward direction by") && msg.ends_with("try yaw: \"outward\""), "{msg}");
    }
}

#[test]
fn outward_yaw_reaches_both_points_and_j1_tracks_atan2() {
    let home = input_at(profile(), None);
    let first = plan(&home, &spec([0.18, 0.10, 0.30], true)).expect("home to (0.18, 0.10) outward");
    let q = first.last();
    println!("outward at (0.18, 0.10, 0.30): q = {:?}", &q[..6]);
    assert!((f64::from(q[0]) - 0.10f64.atan2(0.18)).abs() < 0.03, "J1 {}", q[0]);
    let mut cur = at_end(&home, &first);
    for y in [-0.10, 0.0, 0.10] {
        let path = plan(&cur, &spec([0.18, y, 0.30], true)).unwrap_or_else(|e| panic!("to (0.18, {y}): {e}"));
        let dev = j1_tracking(&cur, &path);
        let end = path.last();
        println!("outward to (0.18, {y}, 0.30): {} samples, max |J1 - atan2| {dev:.4} rad, q = {:?}", path.q.len(), &end[..6]);
        assert!(dev < 0.03, "J1 tracks atan2 within 0.03 rad: {dev}");
        // goal orientation: rpy (0, 40°, atan2(y, x))
        let q6: [f64; 6] = std::array::from_fn(|i| f64::from(end[i]));
        let r_end = cur.arm.fk(&q6).r;
        let want = kinematics::rpy_to_r(0.0, PITCH_40, y.atan2(0.18));
        assert!(kinematics::norm(&kinematics::log_v(&kinematics::mul_t(&r_end, &want))) < 1e-2);
        cur = at_end(&cur, &path);
    }
}

#[test]
fn outward_path_near_the_base_axis_is_refused() {
    // home (0.26, 0) to (-0.18, -0.01): the line crosses x = 0 at y = -0.006
    let home = input_at(profile(), None);
    let err = plan(&home, &spec([-0.18, -0.01, 0.30], true)).unwrap_err();
    println!("{} {err}", err.code());
    assert_eq!(err.code(), "INVALID_COMMAND");
    assert!(err.to_string().contains("undefined near the base z axis"), "{err}");
    let err = plan(&home, &spec([0.0, 0.0, 0.30], true)).unwrap_err();
    assert_eq!(err.code(), "INVALID_COMMAND", "{err}");
}

#[test]
fn five_dof_auto_yaw_is_unchanged_and_outward_also_works() {
    let input = input_at(so101(), None);
    let p0 = input.arm.fk(&input.q0).p;
    let goal_p = [p0[0], p0[1] + 0.06, p0[2]];
    for spec in [
        GoalSpec::Absolute { p: goal_p, r: None },
        GoalSpec::AbsoluteEx { p: goal_p, r: None, auto_yaw: true },
        GoalSpec::Relative { frame: motion::RelFrame::Base, d: [0.0, 0.06, 0.0], dyaw: 0.0, dpitch: 0.0, droll: 0.0, level: false, auto_yaw: None },
    ] {
        let (goal_ex, yaw_free) = motion::resolve_goal_ex(&input, &spec);
        let (goal_opts, opts) = motion::resolve_goal_opts(&input, &spec);
        assert_eq!((goal_ex.p, goal_ex.r), (goal_opts.p, goal_opts.r));
        assert_eq!(opts, PathOpts { yaw_free, yaw_outward: false });
        assert!(yaw_free, "5-DOF translation keeps auto_yaw");
        let a = motion::plan_cartesian_ex(&input, goal_ex, Speed::default(), yaw_free).unwrap();
        let b = motion::plan_cartesian_opts(&input, goal_opts, Speed::default(), opts).unwrap();
        println!("SO-ARM101 auto_yaw {spec:?}: max |J1 - atan2| {:.4}", j1_tracking(&input, &a));
        assert_eq!(a, b);
    }
    let outward = GoalSpec::YawOutward(Box::new(GoalSpec::Absolute { p: goal_p, r: None }));
    let (goal, opts) = motion::resolve_goal_opts(&input, &outward);
    assert_eq!(opts, PathOpts { yaw_free: true, yaw_outward: true });
    let path = motion::plan_cartesian_opts(&input, goal, Speed::default(), opts).expect("5-DOF outward");
    println!("SO-ARM101 outward: {} samples, max |J1 - atan2| {:.4}", path.q.len(), j1_tracking(&input, &path));
    // no hint on 5-DOF arms
    let r = kinematics::rpy_to_r(0.0, 0.5, 1.0);
    assert_eq!(motion::outward_hint(5, PathOpts::default(), &[0.2, 0.0, 0.2], &r), None);
}

#[test]
fn hint_only_when_yaw_is_off_outward_by_more_than_15_degrees() {
    let p: [f64; 3] = [0.18, 0.10, 0.30];
    let psi = p[1].atan2(p[0]);
    let off = |deg: f64| kinematics::rpy_to_r(0.0, PITCH_40, psi + deg.to_radians());
    let none = PathOpts::default();
    assert_eq!(motion::outward_hint(6, none, &p, &off(10.0)), None);
    let h = motion::outward_hint(6, none, &p, &off(-29.0)).unwrap();
    assert!((h - 29.0).abs() < 1e-6, "{h}");
    assert_eq!(motion::outward_hint(6, PathOpts { yaw_free: false, yaw_outward: true }, &p, &off(-29.0)), None);
    assert_eq!(motion::outward_hint(6, none, &[0.01, 0.0, 0.3], &off(-29.0)), None, "near the base axis");
    assert_eq!(motion::outward_hint(6, none, &p, &kinematics::rpy_to_r(0.0, std::f64::consts::FRAC_PI_2, 0.0)), None, "tool vertical");
    let e = MotionError::IkUnreachable { s: 0.5, ep: 0.01, er: 0.2, yaw_off_deg: Some(29.4) };
    assert_eq!(e.to_string(), "IK unreachable at s=0.500 (position error 0.0100 m, rotation error 0.2000 rad); yaw differs from the outward direction by 29°; try yaw: \"outward\"");
    let e = MotionError::IkBranchFlip { s: 0.5, joint: 1, dq: 0.105, yaw_off_deg: None };
    assert_eq!(e.to_string(), "IK branch flip at s=0.500: joint 1 jumps 0.105 rad between samples (limit 0.10)");
}

#[test]
fn with_outward_yaw_replaces_yaw_and_keeps_roll_and_pitch() {
    let p: [f64; 3] = [0.18, -0.10, 0.2];
    let r = motion::with_outward_yaw(&p, &kinematics::rpy_to_r(0.3, PITCH_40, 1.2));
    let rpy = kinematics::r_to_rpy(&r);
    assert!((rpy[0] - 0.3).abs() < 1e-9 && (rpy[1] - PITCH_40).abs() < 1e-9 && (rpy[2] - p[1].atan2(p[0])).abs() < 1e-9, "{rpy:?}");
    let down = kinematics::rpy_to_r(0.4, std::f64::consts::FRAC_PI_2, 0.0);
    assert_eq!(motion::with_outward_yaw(&p, &down), down, "tool vertical: unchanged");
}

#[test]
fn parse_yaw_field() {
    let (s, _, _) = motion::parse_move_to(&json!({"position": [0.2, 0.1, 0.3], "orientation": {"rpy": [0, 0.7, 0]}, "yaw": "outward"})).unwrap();
    assert!(matches!(s, GoalSpec::YawOutward(ref inner) if matches!(**inner, GoalSpec::Absolute { .. })), "{s:?}");
    let (s, _, _) = motion::parse_move_to(&json!({"position": [0.2, 0.1, 0.3], "yaw": null})).unwrap();
    assert!(matches!(s, GoalSpec::Absolute { .. }));
    for bad in [json!("inward"), json!(true), json!(0.5)] {
        let e = motion::parse_move_to(&json!({"position": [0.2, 0.1, 0.3], "yaw": bad})).unwrap_err();
        assert_eq!(e.to_string(), "yaw must be \"outward\" or null");
    }
    let (s, _, _) = motion::parse_move_relative(&json!({"frame": "base", "d": [0, 0.05, 0], "yaw": "outward"})).unwrap();
    assert!(matches!(s, GoalSpec::YawOutward(ref inner) if matches!(**inner, GoalSpec::Relative { .. })));
    let e = motion::parse_move_relative(&json!({"frame": "base", "dyaw": 0.1, "yaw": "outward"})).unwrap_err();
    assert_eq!(e.code(), "INVALID_COMMAND");
}

#[tokio::test]
async fn capability_is_advertised_with_cartesian() {
    let sup = ArmSupervisor::new(profile());
    let schema = serde_json::to_value(sup.get_schema().await).unwrap();
    assert_eq!(schema["capabilities"]["yaw_outward"], true);
    assert_eq!(schema["capabilities"]["cartesian"], true);
}

// ---- WS: move_to, move_relative, place ----

struct Rig { ws: common::TestSocket, generation: u64, epoch: u64, sup: Arc<ArmSupervisor> }

impl Rig {
    async fn new() -> Self {
        let home: [f32; 7] = std::array::from_fn(|i| profile().home_positions.get(i).copied().unwrap_or(0.0));
        let (ws, generation, epoch, sup, _) = common::torqued_rig(Arc::new(ArmSupervisor::new(profile())), &home).await;
        Rig { ws, generation, epoch, sup }
    }
    async fn rpc(&mut self, path: &str, body: Value) -> Value { common::rpc(&mut self.ws, self.generation, self.epoch, path, body).await }
    async fn wait(&mut self, id: u64) -> armd::motion::ActionStatus {
        let deadline = Instant::now() + Duration::from_secs(90);
        let mut last_hb = Instant::now();
        loop {
            assert!(Instant::now() < deadline, "action {id} not finished");
            if last_hb.elapsed() > Duration::from_millis(200) {
                common::send(&mut self.ws, json!({"type":"heartbeat", "generation":self.generation, "epoch_id":self.epoch})).await;
                common::drain(&mut self.ws, 5).await;
                last_hb = Instant::now();
            }
            let obs = self.sup.get_observation().await;
            if let Some(a) = obs.action.into_iter().chain(obs.previous_action).find(|a| a.id == id) {
                if !a.is_running() { return a; }
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }
    async fn run(&mut self, path: &str, body: Value) -> Result<armd::motion::ActionStatus, Value> {
        let ack = self.rpc(path, body).await;
        if ack["type"] != "command_ack" { return Err(ack); }
        Ok(self.wait(ack["action_id"].as_u64().unwrap()).await)
    }
    async fn q(&self) -> Vec<f32> { self.sup.inner().read().await.commanded_positions.clone() }
}

fn tcp(q: &[f32]) -> Pose { kinematics::fk(&kinematics::q6(q)) }

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn ws_move_to_and_move_relative_with_yaw_outward() {
    let mut r = Rig::new().await;
    let rpy = json!({"rpy": [0.0, PITCH_40, 0.0]});
    let a = r.run("/api/v1/action/move_to", json!({"position": [0.18, 0.10, 0.30], "orientation": rpy, "yaw": "outward", "speed": {"linear": 0.15, "angular": 1.0}})).await.unwrap();
    assert_eq!(a.state, "done", "{a:?}");
    let a = r.run("/api/v1/action/move_to", json!({"position": [0.18, -0.10, 0.30], "orientation": rpy, "yaw": "outward", "speed": {"linear": 0.15, "angular": 1.0}})).await.unwrap();
    assert_eq!(a.state, "done", "{a:?}");
    let q = r.q().await;
    assert!((f64::from(q[0]) - (-0.10f64).atan2(0.18)).abs() < 0.03, "J1 {}", q[0]);
    // move_relative 0.1 m left with the yaw outward
    let a = r.run("/api/v1/action/move_relative", json!({"frame": "base", "d": [0.0, 0.10, 0.0], "yaw": "outward", "speed": {"linear": 0.15, "angular": 1.0}})).await.unwrap();
    assert_eq!(a.state, "done", "{a:?}");
    let q = r.q().await;
    let p = tcp(&q).p;
    assert!(p[1].abs() < 2e-3 && (f64::from(q[0]) - p[1].atan2(p[0])).abs() < 0.03, "p {p:?} q {q:?}");
    let rej = r.run("/api/v1/action/move_to", json!({"position": [0.18, 0.0, 0.30], "yaw": "sideways"})).await.unwrap_err();
    assert_eq!(rej["code"], "INVALID_COMMAND", "{rej}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn ws_place_with_yaw_outward() {
    let mut r = Rig::new().await;
    let target: [f64; 3] = [0.20, 0.12, 0.15];
    let a = r.run("/api/v1/action/place", json!({"position": target, "orientation": {"rpy": [0.0, PITCH_40, 0.0]}, "yaw": "outward", "retreat": false})).await.unwrap();
    assert_eq!(a.state, "done", "{a:?}");
    let q = r.q().await;
    let pose = tcp(&q);
    assert!(kinematics::norm(&kinematics::sub(&pose.p, &target)) < 2e-3, "{:?}", pose.p);
    let rpy = kinematics::r_to_rpy(&pose.r);
    assert!((rpy[2] - target[1].atan2(target[0])).abs() < 1e-2 && (rpy[1] - PITCH_40).abs() < 1e-2, "{rpy:?}");
    let rej = r.run("/api/v1/action/place", json!({"position": target, "yaw": 1})).await.unwrap_err();
    assert_eq!(rej["code"], "INVALID_COMMAND", "{rej}");
}
