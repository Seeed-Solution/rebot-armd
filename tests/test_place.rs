//! Place action (`/api/v1/action/place`) over WS against the simulated follower with the
//! B601-DM kit profile.

mod common;

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use serde_json::{json, Value};

use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointMode, JointTarget};
use armd::kinematics::{self, Vec3};
use armd::model::{JointState, KeepoutBox};
use armd::motion::ActionStatus;
use armd::{ArmConfig, ArmSupervisor};

const PROFILE_PATH: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/kits/recamera-b601dm/profile/rebot-b601dm.json");
/// Start pose; J7 = -1.5 (jaws around an object, closed = 0, open = -5.7).
const START: [f32; 7] = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0, -1.5];

fn profile() -> ArmConfig {
    let mut config: ArmConfig = serde_json::from_reader(std::fs::File::open(PROFILE_PATH).unwrap()).unwrap();
    config.validate().unwrap();
    config
}

fn tcp(q: &[f32]) -> Vec3 { kinematics::fk(&kinematics::q6(q)).p }

fn dist(a: &Vec3, b: &Vec3) -> f64 { kinematics::norm(&kinematics::sub(a, b)) }

struct RecordingState {
    q: [f32; 7],
    torqued: bool,
    tx_j7: Vec<f32>,
}

/// Ideal position servo that losslessly records every J7 position written by the control loop.
#[derive(Clone)]
struct RecordingArm(Arc<Mutex<RecordingState>>);

impl RecordingArm {
    fn new(q: [f32; 7]) -> Self {
        Self(Arc::new(Mutex::new(RecordingState { q, torqued: false, tx_j7: Vec::new() })))
    }
    fn clear_tx(&self) { self.0.lock().unwrap().tx_j7.clear(); }
    fn tx_j7(&self) -> Vec<f32> { self.0.lock().unwrap().tx_j7.clone() }
}

impl ArmDriver for RecordingArm {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true, torque_feedforward: false, enable_feedback: true, load_feedback: false }
    }
    fn enable_state(&self) -> EnableState {
        if self.0.lock().unwrap().torqued { EnableState::Enabled } else { EnableState::Disabled }
    }
    fn is_feedback_valid(&self) -> bool { true }
    fn is_enabled(&self) -> bool { self.0.lock().unwrap().torqued }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); 7] }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let s = self.0.lock().unwrap();
        let mut q6 = [0.0; 6];
        q6.copy_from_slice(&s.q[..6]);
        let g = armd::compute_gravity_torques(&q6).unwrap();
        Ok((0..7).map(|i| JointState {
            position: s.q[i], velocity: 0.0, temperature: 30.0,
            torque: if i < 6 { g[i] } else { 0.0 },
        }).collect())
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        let mut s = self.0.lock().unwrap();
        if let Some(j7) = targets.get(6) {
            s.tx_j7.push(j7.position);
        }
        for (i, t) in targets.iter().enumerate().take(7) { s.q[i] = t.position; }
        Ok(())
    }
    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        self.0.lock().unwrap().torqued = enabled;
        Ok(())
    }
    armd::ext_accessors!(joint_mode);
}
impl armd::driver::JointModeExt for RecordingArm {
    fn supports_mit_mode_switch(&self) -> bool { true }
    fn set_joint_modes(&mut self, _modes: &[JointMode], _hold: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
}

struct Rig {
    ws: common::TestSocket,
    generation: u64,
    epoch: u64,
    sup: Arc<ArmSupervisor>,
    tasks: Vec<tokio::task::JoinHandle<()>>,
}

impl Drop for Rig {
    fn drop(&mut self) {
        for t in self.tasks.drain(..) { t.abort(); }
    }
}

async fn rig(boxes: Vec<KeepoutBox>) -> Rig {
    rig_with_driver(boxes, None).await
}

async fn rig_with_driver(boxes: Vec<KeepoutBox>, driver: Option<Box<dyn ArmDriver>>) -> Rig {
    let mut config = profile();
    config.workspace.as_mut().unwrap().keepout_boxes = boxes;
    let sup = match driver {
        Some(d) => Arc::new(ArmSupervisor::new(config).with_driver(d)),
        None => Arc::new(ArmSupervisor::new(config)),
    };
    let router = armd::create_router(sup.clone(), armd::OperatorAuth::disabled());
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let srv = tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });
    let ticker = sup.clone();
    // Advance 50 Hz simulated trajectories at accelerated wall-clock rate while moving.
    let tick_task = tokio::spawn(async move {
        loop {
            let moving = ticker.inner().read().await.active_trajectory.is_some();
            if moving {
                ticker.tick(0.02).await;
                tokio::time::sleep(Duration::from_millis(2)).await;
            } else {
                tokio::time::sleep(Duration::from_millis(2)).await;
                ticker.tick(0.02).await;
            }
        }
    });
    let (mut ws, generation) = common::controller(addr).await;
    let epoch = sup.get_observation().await.epoch_id;
    let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    {
        let mut inner = sup.inner().write().await;
        inner.commanded_positions = START.to_vec();
        inner.measured_positions = START.to_vec();
        inner.target_positions = START.to_vec();
    }
    let epoch = sup.get_observation().await.epoch_id;
    Rig { ws, generation, epoch, sup, tasks: vec![srv, tick_task] }
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
    /// the commanded positions. Fails immediately if the action record is missing or mismatched.
    async fn wait(&mut self, id: u64, mut each: impl FnMut(&ActionStatus, &[f32])) -> ActionStatus {
        let deadline = Instant::now() + Duration::from_secs(60);
        let mut last_hb = Instant::now();
        let mut rx = self.sup.subscribe_telemetry().expect("telemetry subscriber");
        let first = self.sup.get_observation().await;
        let first_ts = first.timestamp;
        let a = first.action.into_iter().chain(first.previous_action).find(|a| a.id == id).expect("action record");
        each(&a, &first.commanded_positions);
        if !a.is_running() { return a; }
        loop {
            assert!(Instant::now() < deadline, "place {id} not finished");
            if last_hb.elapsed() > Duration::from_millis(200) { self.heartbeat().await; last_hb = Instant::now(); }
            let obs = match tokio::time::timeout(Duration::from_millis(20), rx.recv()).await {
                Ok(Ok(obs)) if obs.timestamp <= first_ts => continue,
                Ok(Ok(obs)) => obs,
                Ok(Err(tokio::sync::broadcast::error::RecvError::Lagged(_))) | Err(_) => self.sup.get_observation().await,
                Ok(Err(tokio::sync::broadcast::error::RecvError::Closed)) => panic!("telemetry channel closed while waiting for place {id}"),
            };
            let a = obs.action.into_iter().chain(obs.previous_action).find(|a| a.id == id).expect("action record");
            each(&a, &obs.commanded_positions);
            if !a.is_running() { return a; }
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
    assert_eq!(phases, ["lift", "approach", "descend", "final_approach", "open", "retreat", "done"]); // final_approach: same-side backlash approach (kit profile backlash_approach)
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
    assert_eq!(phases, ["approach", "descend", "final_approach", "open", "done"]);
    assert!(dist(&at_open.unwrap(), &target) < 2e-3);
    let q = r.q().await;
    assert!(dist(&tcp(&q), &target) < 2e-3, "stays at the target without retreat");
    let r_end = kinematics::fk(&kinematics::q6(&q)).r;
    assert!(kinematics::norm(&kinematics::log_v(&kinematics::mul_t(&r_end, &r0))) < 1e-2, "orientation kept");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn release_false_keeps_the_gripper_closed() {
    let arm = RecordingArm::new(START);
    let mut r = rig_with_driver(vec![], Some(Box::new(arm.clone()))).await;
    arm.clear_tx();
    let p0 = tcp(&START);
    let target = [p0[0] + 0.03, p0[1], p0[2] - 0.06];
    let id = r.place(json!({"position": target, "release": false})).await;
    let a = r.wait(id, |_, _| {}).await;
    assert_eq!((a.state.as_str(), a.phase.as_deref(), a.holding), ("done", Some("done"), None), "{a:?}");
    // Lossless proof from every driver TX write during the action (never relying on lossy telemetry).
    let tx_j7 = arm.tx_j7();
    assert!(!tx_j7.is_empty(), "driver must receive joint targets during place");
    let j7_moved = tx_j7.iter().map(|&v| (v - START[6]).abs()).fold(0.0f32, f32::max);
    assert!(j7_moved < 1e-6, "J7 never commanded across {} driver TX writes: max delta {j7_moved}", tx_j7.len());
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
    let mut last_hb = Instant::now();
    loop {
        assert!(Instant::now() < deadline, "approach not reached");
        if last_hb.elapsed() > Duration::from_millis(200) { r.heartbeat().await; last_hb = Instant::now(); }
        let inner = r.sup.inner().read().await;
        let moving = inner.active_trajectory.as_ref().is_some_and(|t| t.action_id == Some(id));
        if inner.last_action.as_ref().unwrap().phase.as_deref() == Some("approach") && moving { break; }
        drop(inner);
        tokio::time::sleep(Duration::from_millis(1)).await;
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
    let id = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::begin_macro(__a, "place", &input, None)) } }.unwrap();
    // the stop handler of `/api/v1/motion/stop`
    inner.stop_motion();
    inner.request_macro_stop();
    let j7 = inner.commanded_positions[6];
    let open = GripperRequest { command: GripperCommand::Open, fraction: None, grasp: false, speed: 1.0, timeout_s: Some(8.0), tag: None };
    assert!({ let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_macro_gripper(__a, id, open)) } }.is_err(), "gripper sub-motion installed after stop");
    assert!(inner.active_trajectory.is_none() && inner.macro_run.is_none());
    assert_eq!(inner.commanded_positions[6], j7);
    assert!(inner.target_positions[6] == j7, "gripper target changed");
    let a = inner.last_action.clone().unwrap();
    assert_eq!((a.id, a.state.as_str(), a.code.as_deref()), (id, "cancelled", Some("STOPPED")), "{a:?}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn place_spill_safe_cup_is_levelled_and_pen_is_kept() {
    let arm = armd::chain::ArmModel::REBOT_DEV_ARM;
    let lo = [-3.1, -2.05, -2.6, -3.1, -2.05, -6.2];
    let hi = [3.1, 2.05, 2.6, 3.1, 2.05, 6.2];
    let target = [0.25, 0.05, 0.15];
    let r_start = kinematics::fk(&kinematics::q6(&START)).r;
    let r_level = armd::grasp::look_down(&r_start);
    let r_tilted = kinematics::mat_mul(&kinematics::axis_r(&[0.0, 1.0, 0.0], 20.0f64.to_radians()), &r_level);
    let sol = arm.solve_dls(&kinematics::Pose::new(r_tilted, target), &std::array::from_fn(|i| START[i] as f64), &lo, &hi, kinematics::PAR.w);
    assert!(sol.ok, "tilted pose reachable: ok={}, ep={}, er={}", sol.ok, sol.ep, sol.er);
    let mut bin = [0.0f32; 7];
    for i in 0..6 { bin[i] = sol.q[i] as f32; }
    bin[6] = -1.5;

    let r_pose = kinematics::fk(&kinematics::q6(&bin)).r;
    let tool_z = kinematics::column(&r_pose, 0)[2];
    let tilt = (-tool_z).clamp(-1.0, 1.0).acos().to_degrees();
    assert!(tilt > 15.0, "bin pose is tilted: {tilt:.2}°");

    // 1. Holding a cup (spill_safe: true) -> levelled; 2. Holding a pen (spill_safe: false) -> kept
    let cup_case = async {
        let mut r = rig(vec![]).await;
        let ack = r.rpc("/api/v1/poses/save", json!({"name": "bin_tilted", "positions": bin})).await;
        assert_eq!(ack["type"], "command_ack", "{ack}");
        r.sup.inner().write().await.held = Some(armd::supervisor::HeldObject {
            label: "cup".into(),
            spill_safe: true,
        });
        let id = r.place(json!({"pose": "bin_tilted", "tag": "cup"})).await;
        let (_, _, a) = r.run(id).await;
        assert_eq!((a.state.as_str(), a.orientation.as_deref()), ("done", Some("levelled")), "{a:?}");
        let q = r.q().await;
        let r_cup = kinematics::fk(&kinematics::q6(&q)).r;
        let cup_tool_z = kinematics::column(&r_cup, 0)[2];
        let cup_tilt = (-cup_tool_z).clamp(-1.0, 1.0).acos().to_degrees();
        assert!(cup_tilt < 1.0, "cup was levelled: {cup_tilt:.2}°");
        assert!(r.sup.inner().read().await.held.is_none());
    };

    let pen_case = async {
        let mut r = rig(vec![]).await;
        let ack = r.rpc("/api/v1/poses/save", json!({"name": "bin_tilted", "positions": bin})).await;
        assert_eq!(ack["type"], "command_ack", "{ack}");
        r.sup.inner().write().await.held = Some(armd::supervisor::HeldObject {
            label: "pen".into(),
            spill_safe: false,
        });
        let id = r.place(json!({"pose": "bin_tilted", "tag": "pen"})).await;
        let (_, _, a) = r.run(id).await;
        assert_eq!((a.state.as_str(), a.orientation.as_deref()), ("done", None), "{a:?}");
        let q = r.q().await;
        let r_pen = kinematics::fk(&kinematics::q6(&q)).r;
        assert!(kinematics::norm(&kinematics::log_v(&kinematics::mul_t(&r_pen, &r_pose))) < 0.02, "pen kept pose orientation");
        assert!(r.sup.inner().read().await.held.is_none());
    };

    tokio::join!(cup_case, pen_case);
}







