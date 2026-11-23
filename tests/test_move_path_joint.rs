//! `path: "joint"` for move_to / move_relative (IK of the goal from several seeds, then the
//! move_to_pose joint quintic) and the `path: "joint"` hint of a straight-line IK_BRANCH_FLIP
//! (B601-DM profile).
//!
//! The refused moves are the ones of the real B601-DM on 2026-11-21: the camera data-collection
//! sweep (poses 6, 38, 42, 79, 80 skipped with IK_BRANCH_FLIP) and a lateral move at pitch 15°.
//! They are reproduced with that arm's kinematic correction (L2 fit installed on the device) and
//! the joints the sweep recorded at the pose before.
mod common;

use std::sync::Arc;
use std::time::{Duration, Instant};
use serde_json::{json, Value};

use armd::kinematics::{self, Correction, JointCorrection, Pose};
use armd::model::KeepoutBox;
use armd::motion::{self, GoalSpec, PathMode};
use armd::{ArmConfig, ArmSupervisor};

const PROFILE_PATH: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/kits/recamera-b601dm/profile/rebot-b601dm.json");

fn profile() -> ArmConfig {
    let mut config: ArmConfig = serde_json::from_reader(std::fs::File::open(PROFILE_PATH).unwrap()).unwrap();
    config.validate().unwrap();
    config
}

/// Kinematic correction of the office B601-DM (`/api/v1/calibration` kinematics, L2, 2026-11-19).
fn device_correction() -> Correction {
    let j = |offset_rad: f64, scale: f64, tilt_x: f64| JointCorrection { offset_rad, scale, tilt_rad: [tilt_x, 0.0], origin_m: [0.0; 3] };
    [
        j(0.0, 0.0, 0.0),
        j(0.022640218347280937, 0.018909738512459664, -0.013246711113889306),
        j(0.010016918929479549, 0.0, 0.0),
        j(0.02470332291804993, 0.0, 0.0),
        j(-0.04584914474685318, 0.0, 0.0),
        j(0.0, 0.0, 0.0),
    ]
}

/// Measured joints of the sweep (`dcol1/meta.json` positions) at sweep poses 4, 36, 40 and 77.
const SWEEP_Q4: [f32; 7] = [-0.22240067, -1.1383228, -1.8768597, 1.5777826, 0.004196167, -0.030517578, 0.0];
const SWEEP_Q36: [f32; 7] = [0.31433582, -1.2428474, -1.9142447, 1.5087357, 0.012588501, -0.019073486, -0.00038146973];
const SWEEP_Q40: [f32; 7] = [0.3170061, -1.1707487, -1.6319523, 1.560998, 0.020599365, -0.022125244, 0.0];
const SWEEP_Q77: [f32; 7] = [0.8461132, -0.7591362, -0.96208096, 1.0494394, 0.016784668, -0.01411438, 0.0];

/// (start joints, pose before the skipped one, skipped pose); pose = (x, y, z, pitch °), yaw outward.
type SweepCase = ([f32; 7], [f64; 4], [f64; 4]);
const SWEEP: [SweepCase; 5] = [
    (SWEEP_Q4, [0.217, -0.038, 0.38, 65.0], [0.217, -0.038, 0.30, 35.0]),
    (SWEEP_Q36, [0.244, 0.089, 0.38, 65.0], [0.244, 0.089, 0.30, 50.0]),
    (SWEEP_Q40, [0.207, 0.075, 0.30, 35.0], [0.207, 0.075, 0.38, 65.0]),
    (SWEEP_Q77, [0.141, 0.169, 0.30, 35.0], [0.141, 0.169, 0.38, 65.0]),
    (SWEEP_Q77, [0.141, 0.169, 0.30, 35.0], [0.167, 0.199, 0.38, 65.0]),
];

fn body(p: [f64; 4], extra: Value) -> Value {
    let mut b = json!({"position": [p[0], p[1], p[2]], "orientation": {"rpy": [0.0, p[3].to_radians(), 0.0]}, "yaw": "outward"});
    for (k, v) in extra.as_object().unwrap() { b[k] = v.clone(); }
    b
}

struct Rig { ws: common::TestSocket, generation: u64, epoch: u64, sup: Arc<ArmSupervisor> }

impl Rig {
    async fn new(q: &[f32; 7], corrected: bool) -> Self {
        let sup = Arc::new(ArmSupervisor::new(profile()));
        if corrected { sup.inner().write().await.set_kinematics_correction(Some(&device_correction())).unwrap(); }
        let (ws, generation, epoch, sup, _) = common::torqued_rig(sup, q).await;
        Rig { ws, generation, epoch, sup }
    }
    async fn rpc(&mut self, path: &str, body: Value) -> Value { common::rpc(&mut self.ws, self.generation, self.epoch, path, body).await }
    async fn wait(&mut self, id: u64) -> armd::motion::ActionStatus {
        let deadline = Instant::now() + Duration::from_secs(120);
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
    /// Ack (or rejection) and the finished action.
    async fn run(&mut self, path: &str, body: Value) -> Result<(Value, armd::motion::ActionStatus), Value> {
        let ack = self.rpc(path, body).await;
        if ack["type"] != "command_ack" { return Err(ack); }
        let a = self.wait(ack["action_id"].as_u64().unwrap()).await;
        Ok((ack, a))
    }
    async fn move_to(&mut self, body: Value) -> Result<(Value, armd::motion::ActionStatus), Value> { self.run("/api/v1/action/move_to", body).await }
    async fn q(&self) -> Vec<f32> { self.sup.inner().read().await.commanded_positions.clone() }
    /// TCP of the commanded joints with the supervisor's arm model (correction included).
    async fn tcp(&self) -> Pose {
        let inner = self.sup.inner().read().await;
        inner.config.arm_model.fk(&kinematics::q6(&inner.commanded_positions[..6]))
    }
    async fn set_keepout(&self, boxes: Vec<KeepoutBox>) { self.sup.inner().write().await.set_keepout_boxes(boxes).unwrap(); }
}

/// Position (m) and rotation (rad) error of `pose` from (x, y, z), rpy (0, pitch, atan2(y, x)).
fn goal_error(pose: &Pose, p: [f64; 4]) -> (f64, f64) {
    let want = kinematics::rpy_to_r(0.0, p[3].to_radians(), p[1].atan2(p[0]));
    (kinematics::norm(&kinematics::sub(&pose.p, &[p[0], p[1], p[2]])),
     kinematics::norm(&kinematics::log_v(&kinematics::mul_t(&pose.r, &want))))
}

async fn assert_linear_refused_joint_reaches(r: &mut Rig, goal: [f64; 4], label: &str) {
    let rej = r.move_to(body(goal, json!({"speed": {"linear": 0.15, "angular": 1.0}}))).await.unwrap_err();
    let msg = rej["message"].as_str().unwrap_or_default().to_string();
    println!("{label}: linear to {goal:?}: {} {msg}", rej["code"]);
    assert_eq!(rej["code"], "IK_BRANCH_FLIP", "{rej}");
    assert!(msg.ends_with("; a joint-space move reaches it: try path: \"joint\""), "{msg}");
    let q0 = r.q().await;
    let (ack, a) = r.move_to(body(goal, json!({"path": "joint"}))).await.unwrap_or_else(|e| panic!("{label}: joint refused: {e}"));
    assert_eq!(a.state, "done", "{label}: {a:?}");
    assert_eq!(a.kind, "move_to");
    let (ep, er) = goal_error(&r.tcp().await, goal);
    let q1 = r.q().await;
    println!("{label}: joint to {goal:?}: done in {:.2} s planned, error {:.2} mm / {:.4} rad, q {:?} -> {:?}",
        ack["expected_duration_s"].as_f64().unwrap(), ep * 1000.0, er, &q0[..6], &q1[..6]);
    assert!(ep < 1.5e-3 && er < 1e-2, "{label}: goal error {ep} m / {er} rad");
    assert_eq!(q1[6], q0[6], "gripper kept");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn real_sweep_skips_are_refused_linear_and_reached_with_a_joint_path() {
    for (i, (q, before, skipped)) in SWEEP.iter().enumerate() {
        let mut r = Rig::new(q, true).await;
        // the sweep's move to the pose before the skipped one (it succeeded on the real arm)
        let (_, a) = r.move_to(body(*before, json!({"speed": {"linear": 0.15, "angular": 1.0}}))).await.unwrap_or_else(|e| panic!("to {before:?}: {e}"));
        assert_eq!(a.state, "done", "{a:?}");
        assert_linear_refused_joint_reaches(&mut r, *skipped, &format!("sweep case {i}")).await;
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn lateral_move_at_pitch_15_is_refused_linear_and_reached_with_a_joint_path() {
    let home: [f32; 7] = std::array::from_fn(|i| profile().home_positions.get(i).copied().unwrap_or(0.0));
    let mut r = Rig::new(&home, true).await;
    let (_, a) = r.move_to(body([0.12, 0.18, 0.30, 15.0], json!({"speed": {"linear": 0.15, "angular": 1.0}}))).await.unwrap();
    assert_eq!(a.state, "done", "{a:?}");
    let rej = r.rpc("/api/v1/action/move_to", body([0.20, -0.12, 0.30, 15.0], json!({}))).await;
    assert!(rej["message"].as_str().unwrap().starts_with("IK branch flip at s=0.175:"), "{rej}");
    assert_linear_refused_joint_reaches(&mut r, [0.20, -0.12, 0.30, 15.0], "pitch 15").await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn move_relative_with_a_joint_path() {
    let mut r = Rig::new(&SWEEP_Q36, true).await;
    let (_, a) = r.move_to(body([0.244, 0.089, 0.38, 65.0], json!({}))).await.unwrap();
    assert_eq!(a.state, "done");
    // the goal of move_relative resolves as for the straight line (here 0.08 m down, 15° less pitch)
    let rel = json!({"frame": "base", "d": [0.0, 0.0, -0.08], "dpitch": (-15.0f64).to_radians(), "yaw": "outward", "path": "joint", "speed_scale": 0.5});
    let (_, a) = r.run("/api/v1/action/move_relative", rel).await.unwrap();
    assert_eq!((a.state.as_str(), a.kind.as_str()), ("done", "move_relative"), "{a:?}");
    let (ep, er) = goal_error(&r.tcp().await, [0.244, 0.089, 0.30, 50.0]);
    assert!(ep < 1.5e-3 && er < 1e-2, "{ep} {er}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn joint_path_through_a_keepout_box_is_refused() {
    let home: [f32; 7] = std::array::from_fn(|i| profile().home_positions.get(i).copied().unwrap_or(0.0));
    let goal = [0.12, 0.18, 0.30, 15.0];
    // the joint path's TCP halfway, from the IK the action will use
    let (mid, end) = {
        let inner = common::sim_at(profile(), &home);
        let input = inner.plan_input().unwrap();
        let spec = GoalSpec::YawOutward(Box::new(GoalSpec::Absolute { p: [goal[0], goal[1], goal[2]], r: Some(kinematics::rpy_to_r(0.0, goal[3].to_radians(), 0.0)) }));
        let (pose, opts) = motion::resolve_goal_opts(&input, &spec);
        let q = motion::solve_goal_joints(&input, &pose, opts, &motion::goal_seeds(&input, &pose, &inner.goal_seed_poses())).unwrap();
        let mid: [f64; 6] = std::array::from_fn(|i| (input.q0[i] + q[i]) / 2.0);
        (input.arm.fk(&mid).p, pose.p)
    };
    let start = kinematics::fk(&kinematics::q6(&home)).p;
    println!("joint path TCP: start {start:?}, halfway {mid:?}, goal {end:?}");
    let h = 0.02f32;
    let cube = KeepoutBox { name: "midway".into(), min_m: std::array::from_fn(|i| mid[i] as f32 - h), max_m: std::array::from_fn(|i| mid[i] as f32 + h) };
    let mut r = Rig::new(&home, false).await;
    r.set_keepout(vec![cube]).await;
    let rej = r.move_to(body(goal, json!({"path": "joint"}))).await.unwrap_err();
    println!("keep-out box at the halfway TCP: {} {}", rej["code"], rej["message"]);
    assert_eq!(rej["code"], "WORKSPACE_VIOLATION", "{rej}");
    assert_eq!(r.q().await, home.to_vec(), "nothing moved");
    // without the box the same move runs
    r.set_keepout(vec![]).await;
    let (_, a) = r.move_to(body(goal, json!({"path": "joint"}))).await.unwrap();
    assert_eq!(a.state, "done", "{a:?}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn joint_goal_below_the_floor_or_out_of_reach_is_refused() {
    let home: [f32; 7] = std::array::from_fn(|i| profile().home_positions.get(i).copied().unwrap_or(0.0));
    let mut r = Rig::new(&home, false).await;
    let rej = r.move_to(body([0.25, 0.0, -0.02, 90.0], json!({"path": "joint"}))).await.unwrap_err();
    println!("below the floor: {} {}", rej["code"], rej["message"]);
    assert_eq!(rej["code"], "WORKSPACE_VIOLATION", "{rej}");
    let rej = r.move_to(body([0.80, 0.0, 0.30, 0.0], json!({"path": "joint"}))).await.unwrap_err();
    println!("out of reach: {} {}", rej["code"], rej["message"]);
    assert_eq!(rej["code"], "IK_UNREACHABLE", "{rej}");
    assert!(rej["message"].as_str().unwrap().starts_with("No IK solution for the goal pose within the joint limits (best of 4 seeds"), "{rej}");
    assert_eq!(r.q().await, home.to_vec());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn speed_scale_slows_the_joint_path_and_bad_fields_are_refused() {
    let home: [f32; 7] = std::array::from_fn(|i| profile().home_positions.get(i).copied().unwrap_or(0.0));
    let goal = [0.20, -0.12, 0.30, 15.0];
    let mut durations = Vec::new();
    for scale in [1.0, 0.5] {
        let mut r = Rig::new(&home, false).await;
        let (ack, a) = r.move_to(body(goal, json!({"path": "joint", "speed_scale": scale}))).await.unwrap();
        assert_eq!(a.state, "done");
        durations.push(ack["expected_duration_s"].as_f64().unwrap());
    }
    println!("expected duration speed_scale 1.0: {:.3} s, 0.5: {:.3} s", durations[0], durations[1]);
    assert!(durations[1] > 1.6 * durations[0], "{durations:?}");
    let mut r = Rig::new(&home, false).await;
    for (extra, needle) in [
        (json!({"path": "curved"}), "path must be \"linear\", \"joint\" or null"),
        (json!({"path": "joint", "speed": {"linear": 0.05}}), "speed applies to path \"linear\""),
        (json!({"path": "joint", "speed_scale": 1.5}), "speed_scale must be in"),
    ] {
        let rej = r.move_to(body(goal, extra.clone())).await.unwrap_err();
        assert_eq!(rej["code"], "INVALID_COMMAND", "{extra}: {rej}");
        assert!(rej["message"].as_str().unwrap().contains(needle), "{extra}: {rej}");
    }
    assert_eq!(r.q().await, home.to_vec());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn default_path_is_the_unchanged_straight_line() {
    assert_eq!(motion::parse_path_mode(&json!({})).unwrap(), PathMode::Linear);
    assert_eq!(motion::parse_path_mode(&json!({"path": null})).unwrap(), PathMode::Linear);
    assert_eq!(motion::parse_path_mode(&json!({"path": "linear"})).unwrap(), PathMode::Linear);
    assert_eq!(motion::parse_path_mode(&json!({"path": "joint"})).unwrap(), PathMode::Joint);
    let home: [f32; 7] = std::array::from_fn(|i| profile().home_positions.get(i).copied().unwrap_or(0.0));
    // the same straight line with path absent and "linear": duration, end joints, the TCP on the line
    let goal = [0.20, 0.05, 0.25, 30.0];
    let mut runs = Vec::new();
    for extra in [json!({}), json!({"path": "linear"})] {
        let mut r = Rig::new(&home, false).await;
        let (ack, a) = r.move_to(body(goal, extra)).await.unwrap();
        assert_eq!(a.state, "done");
        runs.push((ack["expected_duration_s"].as_f64().unwrap(), r.q().await));
    }
    assert_eq!(runs[0], runs[1]);
    let inner = common::sim_at(profile(), &home);
    let input = inner.plan_input().unwrap();
    let spec = GoalSpec::YawOutward(Box::new(GoalSpec::Absolute { p: [goal[0], goal[1], goal[2]], r: Some(kinematics::rpy_to_r(0.0, goal[3].to_radians(), 0.0)) }));
    let (pose, opts) = motion::resolve_goal_opts(&input, &spec);
    let path = motion::plan_cartesian_opts(&input, pose, motion::Speed::default(), opts).unwrap();
    assert_eq!(runs[0].1[..6], path.last()[..6], "end joints of plan_cartesian_opts");
    // the refused straight line keeps its code; the message only gains the hint
    let mut r = Rig::new(&SWEEP_Q4, true).await;
    r.move_to(body(SWEEP[0].1, json!({}))).await.unwrap();
    let rej = r.move_to(body(SWEEP[0].2, json!({"path": "linear"}))).await.unwrap_err();
    let msg = rej["message"].as_str().unwrap();
    assert_eq!(rej["code"], "IK_BRANCH_FLIP");
    assert!(msg.starts_with("IK branch flip at s=") && msg.contains("between samples (limit 0.10)"), "{msg}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn no_hint_when_the_joint_path_is_blocked_too() {
    // sweep case 0, with a keep-out box over the joint path's halfway TCP
    let (q, before, skipped) = SWEEP[0];
    let mut r = Rig::new(&q, true).await;
    r.move_to(body(before, json!({}))).await.unwrap();
    let mid = {
        let inner = r.sup.inner().read().await;
        let input = inner.plan_input().unwrap();
        let spec = GoalSpec::YawOutward(Box::new(GoalSpec::Absolute { p: [skipped[0], skipped[1], skipped[2]], r: Some(kinematics::rpy_to_r(0.0, skipped[3].to_radians(), 0.0)) }));
        let (pose, opts) = motion::resolve_goal_opts(&input, &spec);
        let qg = motion::solve_goal_joints(&input, &pose, opts, &motion::goal_seeds(&input, &pose, &inner.goal_seed_poses())).unwrap();
        let mid: [f64; 6] = std::array::from_fn(|i| (input.q0[i] + qg[i]) / 2.0);
        input.arm.fk(&mid).p
    };
    let h = 0.015f32;
    r.set_keepout(vec![KeepoutBox { name: "midway".into(), min_m: std::array::from_fn(|i| mid[i] as f32 - h), max_m: std::array::from_fn(|i| mid[i] as f32 + h) }]).await;
    let rej = r.move_to(body(skipped, json!({}))).await.unwrap_err();
    println!("blocked joint path: {} {}", rej["code"], rej["message"]);
    assert!(!rej["message"].as_str().unwrap().contains("try path"), "{rej}");
    let rej = r.move_to(body(skipped, json!({"path": "joint"}))).await.unwrap_err();
    assert_eq!(rej["code"], "WORKSPACE_VIOLATION", "{rej}");
}

#[tokio::test]
async fn capability_is_advertised_with_cartesian() {
    let schema = serde_json::to_value(ArmSupervisor::new(profile()).get_schema().await).unwrap();
    assert_eq!(schema["capabilities"]["move_path_joint"], true);
    assert_eq!(schema["capabilities"]["cartesian"], true);
}

/// Longest supervisor write hold by `ws.rs` at `line` in the recorded ticks (µs).
fn ws_hold_us(sup: &ArmSupervisor, line: u32) -> u32 {
    sup.tick_samples().iter()
        .flat_map(|s| [s.longest_hold, s.waited_for])
        .flatten()
        .filter(|h| h.location.file().ends_with("api/ws.rs") && h.location.line() == line)
        .map(|h| h.hold_us)
        .max()
        .unwrap_or(0)
}

/// The quintic of a `path: "joint"` move is planned and checked with the IK, without the
/// supervisor lock: the install reuses it, and the control tick never waits for the sweep.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn joint_move_installs_the_quintic_planned_without_the_lock() {
    let home: [f32; 7] = std::array::from_fn(|i| profile().home_positions.get(i).copied().unwrap_or(0.0));
    let mut r = Rig::new(&home, true).await;
    // a keep-out box away from the path: the swept check evaluates it on every sample
    r.set_keepout(vec![KeepoutBox { name: "bin".into(), min_m: [0.30, -0.40, 0.0], max_m: [0.40, -0.30, 0.10] }]).await;
    let goals = [[0.12, 0.18, 0.30, 15.0], [0.20, -0.10, 0.30, 30.0], [0.22, 0.0, 0.25, 45.0]];
    for (i, g) in goals.iter().enumerate() {
        let (_, a) = r.move_to(body(*g, json!({"path": "joint", "speed_scale": 0.5}))).await.unwrap_or_else(|e| panic!("{g:?}: {e}"));
        assert_eq!(a.state, "done", "{a:?}");
        assert_eq!(r.sup.inner().read().await.prepared_quintics_used, i as u64 + 1);
    }
    println!("install hold max {} us", ws_hold_us(&r.sup, install_line()));
}

/// When the arm state the quintic was planned from changed, the install plans under the lock.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn joint_move_replans_under_the_lock_when_the_start_changed() {
    let home: [f32; 7] = std::array::from_fn(|i| profile().home_positions.get(i).copied().unwrap_or(0.0));
    let mut r = Rig::new(&home, true).await;
    r.sup.inner().write().await.before_install_hook = Some(Box::new(|inner: &mut armd::SupervisorInner| {
        inner.commanded_velocities[0] = 1e-4;
    }));
    let (_, a) = r.move_to(body([0.12, 0.18, 0.30, 15.0], json!({"path": "joint", "speed_scale": 0.5}))).await.unwrap();
    assert_eq!(a.state, "done", "{a:?}");
    assert_eq!(r.sup.inner().read().await.prepared_quintics_used, 0);
    println!("install hold (planned under the lock) {} us", ws_hold_us(&r.sup, install_line()));
}

/// Line of the `path: "joint"` install's write guard in src/api/ws.rs.
fn install_line() -> u32 {
    let src = include_str!("../src/api/ws.rs");
    let at = src.find("inner.install_joint_goal_prepared(").unwrap();
    let before = &src[..at];
    let guard = before.rfind("let mut inner = recv_supervisor.inner().write().await;").unwrap();
    // the partial last line is the guard's line
    before[..guard].lines().count() as u32
}
