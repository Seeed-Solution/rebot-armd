//! Place action (`/api/v1/action/place`) over WS against the simulated follower with the
//! B601-DM kit profile.

mod common;

use std::sync::Arc;
use std::time::{Duration, Instant};
use serde_json::{json, Value};

use armd::kinematics::{self, Vec3};
use armd::model::KeepoutBox;
use armd::motion::ActionStatus;
use armd::{ArmConfig, ArmSupervisor};

const PROFILE_PATH: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/kits/recamera-b601dm/profile/rebot-b601dm.json");
/// Start pose; J7 = -1.5 (jaws around an object, closed = 0, open = -5.7).
const START: [f32; 7] = [0.0, -1.167, -1.565, 1.435, 0.0, std::f32::consts::FRAC_PI_2, -1.5];

fn profile() -> ArmConfig {
    let mut config: ArmConfig = serde_json::from_reader(std::fs::File::open(PROFILE_PATH).unwrap()).unwrap();
    config.validate().unwrap();
    config
}

fn tcp(q: &[f32]) -> Vec3 { kinematics::fk(&kinematics::q6(q)).p }

fn dist(a: &Vec3, b: &Vec3) -> f64 { kinematics::norm(&kinematics::sub(a, b)) }

struct Rig { ws: common::TestSocket, generation: u64, epoch: u64, sup: Arc<ArmSupervisor> }

async fn rig(boxes: Vec<KeepoutBox>) -> Rig {
    let mut config = profile();
    config.workspace.as_mut().unwrap().keepout_boxes = boxes;
    let (ws, generation, epoch, sup, _) = common::torqued_rig(Arc::new(ArmSupervisor::new(config)), &START).await;
    Rig { ws, generation, epoch, sup }
}

impl Rig {
    async fn rpc(&mut self, path: &str, body: Value) -> Value { common::rpc(&mut self.ws, self.generation, self.epoch, path, body).await }

    async fn place(&mut self, body: Value) -> u64 {
        let ack = self.rpc("/api/v1/action/place", body).await;
        assert_eq!(ack["type"], "command_ack", "{ack}");
        assert!(ack["expected_duration_s"].as_f64().unwrap() > 1.0, "{ack}");
        ack["action_id"].as_u64().unwrap()
    }

    async fn heartbeat(&mut self) {
        common::send(&mut self.ws, json!({"type":"heartbeat", "generation":self.generation, "epoch_id":self.epoch})).await;
        common::drain(&mut self.ws, 5).await;
    }

    /// Wait for the end of action `id`, heartbeating; `each` sees every observed record and
    /// the commanded positions.
    async fn wait(&mut self, id: u64, mut each: impl FnMut(&ActionStatus, &[f32])) -> ActionStatus {
        let deadline = Instant::now() + Duration::from_secs(60);
        let mut last_hb = Instant::now();
        loop {
            assert!(Instant::now() < deadline, "place {id} not finished");
            if last_hb.elapsed() > Duration::from_millis(200) { self.heartbeat().await; last_hb = Instant::now(); }
            let obs = self.sup.get_observation().await;
            let q = self.sup.inner().read().await.commanded_positions.clone();
            let a = obs.action.into_iter().chain(obs.previous_action).find(|a| a.id == id).expect("action record");
            each(&a, &q);
            if !a.is_running() { return a; }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }

    /// Phases in order of appearance (without `start`), the TCP while the `open` phase (or the end) was seen,
    /// and the final record.
    async fn run(&mut self, id: u64) -> (Vec<String>, Option<Vec3>, ActionStatus) {
        let mut phases: Vec<String> = Vec::new();
        let mut at_open = None;
        let a = self.wait(id, |a, q| {
            // `start` (and a lift with nothing to do) last less than one poll
            if let Some(p) = a.phase.as_deref().filter(|p| *p != "start") {
                if phases.last().map(String::as_str) != Some(p) { phases.push(p.to_string()); }
                if p == "open" && at_open.is_none() { at_open = Some(tcp(q)); }
            }
        }).await;
        (phases, at_open, a)
    }

    async fn q(&self) -> Vec<f32> { self.sup.inner().read().await.commanded_positions.clone() }
}

fn open_rad() -> f32 { profile().gripper_policy.unwrap().open_rad }

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn place_at_a_named_pose_lifts_approaches_descends_opens_and_retreats() {
    let mut r = rig(vec![]).await;
    // the pose: the start pose turned 0.3 rad about the base
    let mut bin = START;
    bin[0] = 0.3;
    let ack = r.rpc("/api/v1/poses/save", json!({"name": "bin_a", "positions": bin})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let target = tcp(&bin);
    let id = r.place(json!({"pose": "bin_a", "tag": "p1"})).await;
    let (phases, at_open, a) = r.run(id).await;
    println!("place record: {}", serde_json::to_string(&a).unwrap());
    assert_eq!((a.kind.as_str(), a.state.as_str(), a.tag.as_deref()), ("place", "done", Some("p1")), "{a:?}");
    assert_eq!(phases, ["lift", "approach", "descend", "open", "retreat", "done"]);
    assert_eq!((a.progress, a.holding), (1.0, Some(false)));
    let at_open = at_open.expect("open phase seen");
    assert!(dist(&at_open, &target) < 2e-3, "placed at the pose TCP: {:.2} mm", dist(&at_open, &target) * 1000.0);
    let q = r.q().await;
    assert!((q[6] - open_rad()).abs() < 0.05, "jaws open: {}", q[6]);
    let end = tcp(&q);
    assert!(dist(&end, &[target[0], target[1], target[2] + 0.08]) < 2e-3, "retreated 8 cm up: {end:?}");
    // the pose's orientation (same as the joint pose up to IK branch)
    let r_end = kinematics::fk(&kinematics::q6(&q)).r;
    let r_pose = kinematics::fk(&kinematics::q6(&bin)).r;
    assert!(kinematics::norm(&kinematics::log_v(&kinematics::mul_t(&r_end, &r_pose))) < 1e-2);
    assert!(r.sup.inner().read().await.macro_run.is_none());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn place_at_a_position_without_retreat() {
    let mut r = rig(vec![]).await;
    let p0 = tcp(&START);
    let r0 = kinematics::fk(&kinematics::q6(&START)).r;
    let target = [p0[0] + 0.04, p0[1] - 0.03, p0[2] - 0.05];
    let id = r.place(json!({"position": target, "approach_height_m": 0.05, "retreat": false})).await;
    let (phases, at_open, a) = r.run(id).await;
    assert_eq!((a.kind.as_str(), a.state.as_str()), ("place", "done"), "{a:?}");
    // lift to target z + 0.05 = start z: no lift motion (the phase is reported but brief)
    let phases: Vec<&str> = phases.iter().map(String::as_str).filter(|p| *p != "lift").collect();
    assert_eq!(phases, ["approach", "descend", "open", "done"]);
    assert!(dist(&at_open.unwrap(), &target) < 2e-3);
    let q = r.q().await;
    assert!(dist(&tcp(&q), &target) < 2e-3, "stays at the target without retreat");
    let r_end = kinematics::fk(&kinematics::q6(&q)).r;
    assert!(kinematics::norm(&kinematics::log_v(&kinematics::mul_t(&r_end, &r0))) < 1e-2, "orientation kept");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn release_false_keeps_the_gripper_closed() {
    let mut r = rig(vec![]).await;
    let p0 = tcp(&START);
    let target = [p0[0] + 0.03, p0[1], p0[2] - 0.06];
    let id = r.place(json!({"position": target, "release": false})).await;
    let mut j7_moved = 0.0f32;
    let a = r.wait(id, |_, q| j7_moved = j7_moved.max((q[6] - START[6]).abs())).await;
    assert_eq!((a.state.as_str(), a.phase.as_deref(), a.holding), ("done", Some("done"), None), "{a:?}");
    assert!(j7_moved < 1e-6, "J7 never commanded: {j7_moved}");
    assert!(dist(&tcp(&r.q().await), &[target[0], target[1], target[2] + 0.08]) < 2e-3, "retreated");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn keepout_on_the_descent_is_rejected_before_starting() {
    let p0 = tcp(&START);
    let target = [p0[0] + 0.03, p0[1], p0[2] - 0.06];
    let c = [target[0], target[1], target[2] + 0.03];
    let b = KeepoutBox { name: "fixture".into(), min_m: c.map(|v| (v - 0.01) as f32), max_m: c.map(|v| (v + 0.01) as f32) };
    let mut r = rig(vec![b]).await;
    let reject = r.rpc("/api/v1/action/place", json!({"position": target})).await;
    assert_eq!((reject["type"].as_str(), reject["code"].as_str()), (Some("command_reject"), Some("WORKSPACE_VIOLATION")), "{reject}");
    assert!(reject["message"].as_str().unwrap().contains("keepout 'fixture'"), "{reject}");
    let inner = r.sup.inner().read().await;
    assert!(inner.last_action.is_none() && inner.macro_run.is_none() && inner.active_trajectory.is_none());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn motion_stop_cancels_mid_way() {
    let mut r = rig(vec![]).await;
    let p0 = tcp(&START);
    let id = r.place(json!({"position": [p0[0] + 0.05, p0[1], p0[2] - 0.06]})).await;
    // until the approach motion runs
    let deadline = Instant::now() + Duration::from_secs(20);
    loop {
        assert!(Instant::now() < deadline, "approach not reached");
        r.heartbeat().await;
        let inner = r.sup.inner().read().await;
        let moving = inner.active_trajectory.as_ref().is_some_and(|t| t.action_id == Some(id));
        if inner.last_action.as_ref().unwrap().phase.as_deref() == Some("approach") && moving { break; }
    }
    let ack = r.rpc("/api/v1/motion/stop", json!({})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let a = r.wait(id, |_, _| {}).await;
    assert_eq!((a.state.as_str(), a.code.as_deref(), a.phase.as_deref()), ("cancelled", Some("STOPPED"), Some("approach")), "{a:?}");
    tokio::time::sleep(Duration::from_millis(100)).await;
    let q0 = r.q().await;
    assert!(r.sup.inner().read().await.macro_run.is_none());
    tokio::time::sleep(Duration::from_millis(500)).await;
    let inner = r.sup.inner().read().await;
    assert!(inner.active_trajectory.is_none(), "no motion after the stop");
    let moved = q0.iter().zip(&inner.commanded_positions).map(|(a, b)| (a - b).abs()).fold(0.0f32, f32::max);
    assert!(moved < 1e-3, "orchestrator issued no motion after the stop ({moved} rad)");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn invalid_requests_are_rejected() {
    let mut r = rig(vec![]).await;
    let p0 = tcp(&START);
    for (body, code) in [
        (json!({}), "INVALID_COMMAND"),
        (json!({"pose": "a", "position": p0}), "INVALID_COMMAND"),
        (json!({"pose": "nope"}), "POSE_NOT_FOUND"),
        (json!({"position": p0, "approach_height_m": 0.5}), "INVALID_COMMAND"),
        (json!({"position": p0, "speed_scale": 0.05}), "INVALID_COMMAND"),
        (json!({"position": p0, "release": "yes"}), "INVALID_COMMAND"),
        (json!({"position": p0, "orientation": {"rpy": [0, 0]}}), "INVALID_COMMAND"),
        (json!({"position": [2.0, 0.0, 0.1]}), "IK_UNREACHABLE"),
    ] {
        let reject = r.rpc("/api/v1/action/place", body.clone()).await;
        assert_eq!(reject["code"], code, "{body}: {reject}");
    }
    assert!(r.sup.inner().read().await.last_action.is_none());
}

#[tokio::test]
async fn capability_follows_kinematics_and_gripper_policy() {
    assert!(armd::model::Capabilities::from_config(&profile()).place);
    let mut c = profile();
    c.gripper_policy = None;
    assert!(!armd::model::Capabilities::from_config(&c).place);
    let mut c = profile();
    c.kinematics = None;
    assert!(!armd::model::Capabilities::from_config(&c).place);
}

/// `/motion/stop` between two phases ends the macro under the stop's lock: the orchestrator's
/// next sub-motion (here the `open` of a place) is refused before any tick runs.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn stop_between_phases_refuses_the_next_sub_motion() {
    use armd::motion::{GripperCommand, GripperRequest};
    let r = rig(vec![]).await;
    let mut inner = r.sup.inner().write().await;
    let input = inner.plan_input().unwrap();
    let id = inner.begin_macro("place", &input, None).unwrap();
    // the stop handler of `/api/v1/motion/stop`
    inner.stop_motion();
    inner.request_macro_stop();
    let j7 = inner.commanded_positions[6];
    let open = GripperRequest { command: GripperCommand::Open, fraction: None, grasp: false, speed: 1.0, timeout_s: 8.0, tag: None };
    assert!(inner.start_macro_gripper(id, open).is_err(), "gripper sub-motion installed after stop");
    assert!(inner.active_trajectory.is_none() && inner.macro_run.is_none());
    assert_eq!(inner.commanded_positions[6], j7);
    assert!(inner.target_positions[6] == j7, "gripper target changed");
    let a = inner.last_action.clone().unwrap();
    assert_eq!((a.id, a.state.as_str(), a.code.as_deref()), (id, "cancelled", Some("STOPPED")), "{a:?}");
}
