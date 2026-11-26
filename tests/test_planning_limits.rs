//! Planning-only joint limits tests (1.6.1 B601-DM: J2 planning_max = 0.03, J3 planning_max = 0.04).
//!
//! Covers:
//! - Primary hardware regression: `move_to((0.26, -0.08, 0.17), yaw="outward", speed=0.05)` from
//!   folded home on B601-DM refused at planning time with `IK_UNREACHABLE` naming joint and limit,
//!   starting zero motion, identically under Legacy and RT Executor owners.
//! - The 4 real saved poses (`JJ`, `1`, `r2s_scan`, `r2s_left`) + `home` + `park` + sequences.
//! - Excluded paths: gravity comp at rest, direct `jog`, servo streaming, explicit joint-space
//!   `move_to_pose` / `sequences`, measured-position / idle brake / rest-pose gates / torque-off.
//! - Schema & `GET /api/v1/safety/describe` exposure + static validation.
//! - Certified continuous quintic envelope (constant hold at bound, rest-to-rest approach ending
//!   at bound, non-zero initial velocity overshoot, tampered coefficients, non-finite coeffs,
//!   and dual-owner commit-time velocity drift refusal before admission on `move_to path="joint"`).
//! - Uniform start-outside-planning-band refusal across Cartesian, swept joint, and multiview
//!   planners + recovery via `home` / `park` / `jog`.
//! - Adaptive grasp (`grasp_select`) named planning-limit propagation.
//! - Non-collapsing `ik_search_bounds` (`[0.99, 1.00]` inside `[-1, 1]`, narrow hard ranges).
//! - Gripper planning scope (`include_gripper` false vs true, no-gripper, non-trailing gripper).
//! - Driver narrowing clipping/emptying a planning bound without faulting hard limits or blocking
//!   rearm/recovery, while still faulting genuine hard-limit conflicts.
//! - `f32` Cartesian interpolation endpoint clamping (no overshoot past `0.04f32`).
#![allow(clippy::approx_constant)]
mod common;

use std::net::SocketAddr;
use std::sync::Arc;
use std::time::{Duration, Instant};

use futures_util::StreamExt;
use serde_json::{json, Value};

use armd::driver::{
    ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget,
};
use armd::kinematics;
use armd::model::{ArmConfig, JointLimit, JointState, PlanningScope};
use armd::motion::{self, GoalFrame, GoalSpec, JointPath, Speed};
use armd::rt_core::executor::{ExecutorConfig, RtExecutorHandle};
use armd::rt_core::requests::actions::{
    install_move_joint, prepare_move_to, ActionSnapshot, JointInstallPayload, OwnerCheckCtx,
    PreparedMoveAction, DEFAULT_ACTION_DEADLINE,
};
use armd::trajectory::{MultiJointQuinticTrajectory, QuinticPolynomial};
use armd::{create_router, ArmSupervisor, OperatorAuth, SupervisorInner};
use common::INSPECT;

/// Real B601-DM saved poses from `realarm-rc4-func-step3-poses-cart-20261126T074144Z/summary.json`.
const REAL_POSES: [(&str, [f32; 7]); 4] = [
    ("JJ", [-0.0046, -0.2266, -0.1308, -0.396, -0.0072, -0.0004, -0.1019]),
    ("1", [-0.0053, -0.4715, -0.668, -0.4192, -0.0107, -0.0008, -0.1022]),
    ("r2s_scan", [-0.1835, -1.2982, -2.1103, 1.5461, -0.3227, -0.3223, -0.1022]),
    ("r2s_left", [0.2483, -1.2898, -2.0821, 1.4675, 0.008, -0.0153, -0.1022]),
];

struct OwnerRig {
    ws: common::TestSocket,
    generation: u64,
    epoch: u64,
    sup: Arc<ArmSupervisor>,
    addr: SocketAddr,
    _exec: Option<RtExecutorHandle>,
    _ticker: Option<tokio::task::JoinHandle<()>>,
}

impl OwnerRig {
    async fn spawn(executor: bool, config: ArmConfig, start_q: [f32; 7]) -> Self {
        Self::spawn_with_driver(executor, config, start_q, None).await
    }

    async fn spawn_with_driver(
        executor: bool,
        config: ArmConfig,
        start_q: [f32; 7],
        driver: Option<Box<dyn ArmDriver>>,
    ) -> Self {
        let has_driver = driver.is_some();
        let sup = match driver {
            Some(drv) => Arc::new(ArmSupervisor::new(config).with_driver(drv)),
            None => Arc::new(ArmSupervisor::new(config)),
        };
        if has_driver {
            sup.tick(0.02).await;
        }
        {
            let mut inner = sup.inner().write().await;
            inner.commanded_positions = start_q.to_vec();
            inner.measured_positions = start_q.to_vec();
            inner.target_positions = start_q.to_vec();
        }
        let router = create_router(sup.clone(), OperatorAuth::disabled());
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        tokio::spawn(async move {
            axum::serve(listener, router).await.unwrap();
        });
        let (exec, ticker) = if executor {
            let h = sup
                .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
                .await
                .unwrap();
            tokio::time::sleep(Duration::from_millis(30)).await;
            (Some(h), None)
        } else {
            let t_sup = sup.clone();
            let h = tokio::spawn(async move {
                let mut interval = tokio::time::interval(Duration::from_millis(5));
                loop {
                    interval.tick().await;
                    t_sup.tick(0.02).await;
                }
            });
            (None, Some(h))
        };
        let (mut ws, generation) = common::controller(addr).await;
        let epoch = sup.get_observation().await.epoch_id;
        let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/torque", json!({"enable": true})).await;
        assert_eq!(ack["type"], "command_ack", "torque on failed: {ack}");
        let epoch = sup.get_observation().await.epoch_id;
        Self {
            ws,
            generation,
            epoch,
            sup,
            addr,
            _exec: exec,
            _ticker: ticker,
        }
    }

    async fn rpc(&mut self, path: &str, body: Value) -> Value {
        self.epoch = self.sup.get_observation().await.epoch_id;
        common::rpc(&mut self.ws, self.generation, self.epoch, path, body).await
    }

    async fn wait_action_done(&mut self, action_id: u64) -> armd::motion::ActionStatus {
        let deadline = Instant::now() + Duration::from_secs(15);
        loop {
            let obs = self.sup.get_observation().await;
            if let Some(a) = obs.action {
                if a.id == action_id && !a.is_running() {
                    // Drain any queued telemetry frames so the next rpc() never sees a backlog.
                    while matches!(
                        tokio::time::timeout(Duration::from_millis(2), self.ws.next()).await,
                        Ok(Some(Ok(_)))
                    ) {}
                    return a;
                }
            }
            assert!(Instant::now() < deadline, "timed out waiting for action {action_id}");
            // Drain telemetry while waiting instead of letting frames accumulate in the TCP buffer.
            let _ = tokio::time::timeout(Duration::from_millis(10), self.ws.next()).await;
        }
    }
}

impl Drop for OwnerRig {
    fn drop(&mut self) {
        if let Some(h) = self._ticker.take() {
            h.abort();
        }
    }
}

/// Primary hardware regression (2026-11-26): `move_to((0.26, -0.08, 0.17), yaw="outward", speed=0.05)`
/// from folded home on B601-DM MUST be refused at planning time with `IK_UNREACHABLE`, naming the
/// joint and planning limit, starting zero motion, identically under Legacy and Executor.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn b601dm_move_to_regression_refused_dual_owner() {
    for executor in [false, true] {
        let label = if executor { "executor" } else { "legacy" };
        let mut rig = OwnerRig::spawn(executor, ArmConfig::rebot_dm(), [0.0; 7]).await;

        let rej = rig
            .rpc(
                "/api/v1/action/move_to",
                json!({
                    "position": [0.26, -0.08, 0.17],
                    "yaw": "outward",
                    "speed": {"linear": 0.05, "angular": 0.5}
                }),
            )
            .await;
        assert_eq!(rej["type"], "command_reject", "[{label}] expected reject, got {rej}");
        assert_eq!(rej["code"], "IK_UNREACHABLE", "[{label}] {rej}");
        assert_eq!(rej["status"], 422, "[{label}] {rej}");
        let msg = rej["message"].as_str().unwrap_or("");
        assert!(
            (msg.contains("joint_3") && msg.contains("0.04"))
                || (msg.contains("joint_2") && msg.contains("0.03")),
            "[{label}] refusal message must name joint and planning limit, got: {msg}"
        );
        assert!(msg.contains("planning limit"), "[{label}] {msg}");

        // Zero motion started: no last_action and arm stayed at folded home.
        let obs = rig.sup.get_observation().await;
        assert!(obs.action.is_none(), "[{label}] no action should have started: {:?}", obs.action);
        assert!(
            obs.commanded_positions.iter().all(|q| q.abs() < 1e-6),
            "[{label}] arm must stay at home: {:?}",
            obs.commanded_positions
        );

        // Same regression target with path="joint" is also refused with named planning limit.
        let rej_j = rig
            .rpc(
                "/api/v1/action/move_to",
                json!({
                    "position": [0.26, -0.08, 0.17],
                    "yaw": "outward",
                    "path": "joint",
                    "speed_scale": 0.5
                }),
            )
            .await;
        assert_eq!(rej_j["type"], "command_reject", "[{label}] path=joint: {rej_j}");
        assert_eq!(rej_j["code"], "IK_UNREACHABLE", "[{label}] path=joint: {rej_j}");
        let msg_j = rej_j["message"].as_str().unwrap_or("");
        assert!(
            msg_j.contains("planning limit") && (msg_j.contains("joint_3") || msg_j.contains("joint_2")),
            "[{label}] path=joint message: {msg_j}"
        );

        // move_relative pushing straight into the same point from home is also refused.
        let rej_rel = rig
            .rpc(
                "/api/v1/action/move_relative",
                json!({
                    "frame": "base",
                    "d": [0.0, -0.08, -0.0217],
                    "yaw": "outward",
                    "speed": {"linear": 0.05, "angular": 0.5}
                }),
            )
            .await;
        assert_eq!(rej_rel["type"], "command_reject", "[{label}] move_relative: {rej_rel}");
        assert_eq!(rej_rel["code"], "IK_UNREACHABLE", "[{label}] move_relative: {rej_rel}");
        let msg_r = rej_rel["message"].as_str().unwrap_or("");
        assert!(
            msg_r.contains("planning limit") && (msg_r.contains("joint_3") || msg_r.contains("joint_2")),
            "[{label}] move_relative message: {msg_r}"
        );

        // Ensure describe and schema expose the planning limits consistently on both owners.
        let desc: Value = reqwest::get(format!("http://{}/api/v1/safety/describe", rig.addr))
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert!((desc["limits"]["joints"][1]["max"].as_f64().unwrap() - 2.0944).abs() < 1e-5);
        assert!((desc["limits"]["joints"][1]["planning_max"].as_f64().unwrap() - 0.03).abs() < 1e-6);
        assert!((desc["limits"]["joints"][2]["max"].as_f64().unwrap() - 2.61799).abs() < 1e-5);
        assert!((desc["limits"]["joints"][2]["planning_max"].as_f64().unwrap() - 0.04).abs() < 1e-6);
        assert!((desc["limits"]["joints"][0]["planning_max"].as_f64().unwrap() - 3.14159).abs() < 1e-5);

        let schema: Value = reqwest::get(format!("http://{}/api/v1/observation/schema", rig.addr))
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert!((schema["limits"][1]["planning_max_position"].as_f64().unwrap() - 0.03).abs() < 1e-6);
        assert!((schema["limits"][2]["planning_max_position"].as_f64().unwrap() - 0.04).abs() < 1e-6);
        assert!(schema["limits"][0].get("planning_max_position").is_none());
    }
}

/// The 4 real saved poses (`JJ`, `1`, `r2s_scan`, `r2s_left`), `home`, `park`, and sequences
/// work on both Legacy and Executor. Explicit joint-space `move_to_pose` and `run_sequence`
/// are excluded from Cartesian/IK planning limits and keep hard limits.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn real_saved_poses_home_park_and_sequences_dual_owner() {
    for executor in [false, true] {
        let label = if executor { "executor" } else { "legacy" };
        let mut cfg = ArmConfig::rebot_dm();
        cfg.backlash_approach = None;
        cfg.lease_timeout_ms = 60_000;
        let mut rig = OwnerRig::spawn(executor, cfg, [0.0; 7]).await;

        let mut pose_ids = Vec::new();
        for (name, pos) in REAL_POSES {
            let ack = rig
                .rpc("/api/v1/poses/save", json!({"name": name, "positions": pos}))
                .await;
            assert_eq!(ack["type"], "command_ack", "[{label}] save {name}: {ack}");
            pose_ids.push((name, ack["pose"]["id"].as_str().unwrap().to_string()));
        }

        // Execute all 4 real saved poses.
        for (name, _) in &pose_ids {
            let ack = rig
                .rpc("/api/v1/action/move_to_pose", json!({"name": name, "speed_scale": 1.0}))
                .await;
            assert_eq!(ack["type"], "command_ack", "[{label}] move_to_pose {name}: {ack}");
            let st = rig.wait_action_done(ack["action_id"].as_u64().unwrap()).await;
            assert_eq!(st.state, "done", "[{label}] pose {name} failed: {st:?}");
        }

        // Execute home.
        let ack_h = rig.rpc("/api/v1/motion/home", json!({"duration_secs": 0.2})).await;
        assert_eq!(ack_h["type"], "command_ack", "[{label}] home: {ack_h}");
        let st_h = rig.wait_action_done(ack_h["action_id"].as_u64().unwrap()).await;
        assert_eq!(st_h.state, "done", "[{label}] home failed: {st_h:?}");

        // Sequence visiting real pose JJ succeeds.
        let seq_ok = rig
            .rpc(
                "/api/v1/sequences/save",
                json!({
                    "name": "seq_ok",
                    "steps": [{"type": "pose", "pose_id": pose_ids[0].1, "speed_scale": 1.0}]
                }),
            )
            .await;
        assert_eq!(seq_ok["type"], "command_ack", "[{label}] seq_ok save: {seq_ok}");
        let seq_ok_id = seq_ok["sequence"]["id"].as_str().unwrap().to_string();
        let run_ok = rig
            .rpc("/api/v1/action/run_sequence", json!({"id": seq_ok_id}))
            .await;
        assert_eq!(run_ok["type"], "command_ack", "[{label}] seq_ok run: {run_ok}");
        let st_seq = rig.wait_action_done(run_ok["action_id"].as_u64().unwrap()).await;
        assert_eq!(st_seq.state, "done", "[{label}] seq_ok terminal: {st_seq:?}");

        // Park back to rest (0) succeeds.
        let ack_p = rig.rpc("/api/v1/motion/unload", json!({})).await;
        assert_eq!(ack_p["type"], "command_ack", "[{label}] park: {ack_p}");
        let _ = rig.wait_action_done(ack_p["action_id"].as_u64().unwrap()).await;

        // Re-torque at home and confirm explicit joint-space move_to_pose / run_sequence remain
        // excluded from Cartesian/IK planning limits (keep hard limits, e.g. J2 = +0.05 > 0.03).
        let _ = rig.rpc("/api/v1/torque", json!({"enable": true})).await;
        let hard_save = rig
            .rpc(
                "/api/v1/poses/save",
                json!({"name": "explicit_j2", "positions": [0.0, 0.05, -0.2, 0.0, 0.0, 0.0, 0.0]}),
            )
            .await;
        assert_eq!(hard_save["type"], "command_ack", "[{label}] poses/save keeps hard limits: {hard_save}");
        let hard_id = hard_save["pose"]["id"].as_str().unwrap().to_string();

        let ack_pose = rig
            .rpc("/api/v1/action/move_to_pose", json!({"name": "explicit_j2", "speed_scale": 1.0}))
            .await;
        assert_eq!(ack_pose["type"], "command_ack", "[{label}] move_to_pose excluded: {ack_pose}");
        let st_pose = rig.wait_action_done(ack_pose["action_id"].as_u64().unwrap()).await;
        assert_eq!(st_pose.state, "done", "[{label}] move_to_pose terminal: {st_pose:?}");

        let ack_h2 = rig.rpc("/api/v1/motion/home", json!({"duration_secs": 0.1})).await;
        assert_eq!(ack_h2["type"], "command_ack", "[{label}] home2: {ack_h2}");
        let st_h2 = rig.wait_action_done(ack_h2["action_id"].as_u64().unwrap()).await;
        assert_eq!(st_h2.state, "done", "[{label}] home2 terminal: {st_h2:?}");

        let hard_seq = rig
            .rpc(
                "/api/v1/sequences/save",
                json!({
                    "name": "seq_explicit",
                    "steps": [{"type": "pose", "pose_id": hard_id, "speed_scale": 1.0}]
                }),
            )
            .await;
        assert_eq!(hard_seq["type"], "command_ack", "[{label}] {hard_seq}");
        let hard_seq_id = hard_seq["sequence"]["id"].as_str().unwrap().to_string();
        let ack_seq = rig
            .rpc("/api/v1/action/run_sequence", json!({"id": hard_seq_id}))
            .await;
        assert_eq!(ack_seq["type"], "command_ack", "[{label}] run_sequence excluded: {ack_seq}");
        let st_seq2 = rig.wait_action_done(ack_seq["action_id"].as_u64().unwrap()).await;
        assert_eq!(st_seq2.state, "done", "[{label}] seq_explicit terminal: {st_seq2:?}");
    }
}

/// Excluded paths: gravity comp at rest, direct `jog`, servo streaming, measured-position /
/// idle checks, and recovery (`home` / `park`) from a hard-legal pose outside the planning band.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn excluded_paths_gravity_comp_jog_servo_and_recovery_work() {
    // 3a. Gravity comp at rest [0; 7] enters cleanly (hard limits +/-2.09/2.62, NOT 0.03/0.04).
    let arm = common::TestArm::new();
    arm.0.lock().unwrap().q = [0.0; 7];
    arm.0.lock().unwrap().torqued = true;
    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(arm.clone()));
    inner.lease_enabled = false;
    {
        let s = inner.current_admission_stamp();
        inner
            .check_admission(s)
            .and_then(SupervisorInner::enable_torque)
            .unwrap();
    }
    inner.tick(0.02);
    // Start gravity comp right at folded rest pose (J2 = J3 = 0, within 0.03/0.04 of planning max
    // but far from hard limits; joint_limit_margin_rad = 0.05 must use hard limits!).
    let s = inner.current_admission_stamp();
    inner
        .check_admission(s)
        .map_err(armd::ArmError::from)
        .and_then(|a| {
            SupervisorInner::enter_gravity_comp(
                a,
                armd::gravity_comp::GravityCompRequest { scale: None, damping_kd: None },
            )
        })
        .expect("gravity comp at folded rest pose must enter");
    assert!(inner.gravity_comp.is_some());

    // 3b. Measured position slightly past planning limit (J2 = 0.035, J3 = 0.045) must NOT trip
    // clear_fault joint-limit checks or ServoNoBrake.
    let mut sim = common::sim_at(ArmConfig::rebot_dm(), &[0.0, 0.035, 0.045, 0.0, 0.0, 0.0, 0.0]);
    sim.safety.trigger_e_stop();
    sim.clear_fault().expect("clear_fault at J2=0.035, J3=0.045 uses hard limits");
    {
        let s = sim.current_admission_stamp();
        sim.check_admission(s).and_then(SupervisorInner::enable_torque).unwrap();
    }
    let ep = sim.safety.epoch_id();
    let s = sim.current_admission_stamp();
    sim.check_admission(s)
        .and_then(|a| SupervisorInner::servo(a, vec![0.0, 0.035, 0.06, 0.0, 0.0, 0.0, 0.0], ep))
        .expect("servo streaming keeps hard limits");
    sim.tick(0.02);
    assert!(sim.safety.state().is_operational(), "no fault on servo past planning cap");

    // 3c. Direct jog past planning cap works, and autonomous planners refuse from that pose while
    // home recovers cleanly (dual-owner).
    for executor in [false, true] {
        let label = if executor { "executor" } else { "legacy" };
        let mut rig = OwnerRig::spawn(executor, ArmConfig::rebot_dm(), [0.0; 7]).await;
        // Direct jog to J2 = +0.05 (hard-legal, planning-excluded, above floor with J3 = -0.2).
        let jog_ack = rig
            .rpc(
                "/api/v1/motion/jog",
                json!({"target_positions": [0.0, 0.05, -0.2, 0.0, 0.0, 0.0, 0.0], "duration_secs": 0.1}),
            )
            .await;
        assert_eq!(jog_ack["type"], "command_ack", "[{label}] jog keeps hard limits: {jog_ack}");
        let st = rig.wait_action_done(jog_ack["action_id"].as_u64().unwrap()).await;
        assert_eq!(st.state, "done", "[{label}] jog failed: {st:?}");

        // From J2 = +0.05 (outside planning band), Cartesian move_to is refused consistently.
        let rej = rig
            .rpc(
                "/api/v1/action/move_to",
                json!({"position": [0.25, 0.0, 0.22], "speed": {"linear": 0.05, "angular": 0.5}}),
            )
            .await;
        assert_eq!(rej["type"], "command_reject", "[{label}] start outside band: {rej}");
        assert_eq!(rej["code"], "IK_UNREACHABLE", "[{label}] {rej}");
        assert!(
            rej["message"].as_str().unwrap_or("").contains("joint_2"),
            "[{label}] {rej}"
        );

        // Recovery via home succeeds and brings the arm back to 0.
        let home_ack = rig.rpc("/api/v1/motion/home", json!({"duration_secs": 0.1})).await;
        assert_eq!(home_ack["type"], "command_ack", "[{label}] recovery home: {home_ack}");
        let st_h = rig.wait_action_done(home_ack["action_id"].as_u64().unwrap()).await;
        assert_eq!(st_h.state, "done", "[{label}] recovery home terminal: {st_h:?}");
    }
}

/// Certified continuous quintic envelope: constant holds at bound, rest-to-rest approach ending
/// at bound, non-zero initial velocity overshoot, tampered power coefficients, and non-finite coeffs.
#[test]
fn quintic_certificate_handles_boundaries_overshoot_and_tampered_coeffs() {
    // Constant hold exactly on upper planning bound 0.03: must pass.
    let hold = QuinticPolynomial::new(0.03, 0.0, 0.0, 0.03, 0.0, 0.0, 1.0).unwrap();
    assert!(hold.certify_within(-2.0944, 0.03).is_ok());

    // Rest-to-rest approach from -0.5 ending exactly on upper planning bound 0.04 (f32 boundary): must pass f32 contract.
    let approach = QuinticPolynomial::new(-0.5, 0.0, 0.0, f64::from(0.04f32), 0.0, 0.0, 2.0).unwrap();
    assert!(approach.certify_f32_within(-2.61799f32, 0.04f32).is_ok());

    // Legal endpoints (p0 = 0.02, p1 = -0.5), but positive initial velocity v0 = +0.5 causes an
    // intermediate excursion above 0.04: MUST be rejected.
    let overshoot = QuinticPolynomial::new(0.02, 0.5, 0.0, -0.5, 0.0, 0.0, 1.5).unwrap();
    let peak = (0..=200).map(|k| overshoot.position(1.5 * k as f64 / 200.0)).fold(f64::NEG_INFINITY, f64::max);
    assert!(peak > 0.04, "test setup must actually overshoot 0.04, peak={peak}");
    let err = overshoot.certify_within(-2.61799, 0.04).unwrap_err();
    assert!(err > 0.04, "expected violation > 0.04, got {err}");

    // Interior extremum just inside vs just outside 0.04.
    // p(t) = 4 * peak * t * (1 - t) on T=1 has max = peak at t = 0.5.
    let mk_quad = |peak: f64| QuinticPolynomial {
        a0: 0.0,
        a1: 4.0 * peak,
        a2: -4.0 * peak,
        a3: 0.0,
        a4: 0.0,
        a5: 0.0,
        duration: 1.0,
        p0: 0.0,
        p1: 0.0,
        v0: 0.0, // deliberately inconsistent metadata!
        v1: 0.0,
        a_start: 0.0,
        a_end: 0.0,
    };
    assert!(mk_quad(0.0395).certify_within(-0.1, 0.04).is_ok());
    assert!(mk_quad(0.0405).certify_within(-0.1, 0.04).is_err());
    // Codex R3 counterexample: t - t^2 (peak 0.25) with zero boundary metadata vs [-0.1, 0.1].
    assert!(mk_quad(0.25).certify_within(-0.1, 0.1).is_err());

    // Non-finite coefficients must fail closed.
    let mut bad = hold;
    bad.a5 = f64::NAN;
    assert!(bad.certify_within(-1.0, 1.0).is_err());

    // Codex R1 Major 2 exact counterexample: 1s curve from 0 to -0.5 with tiny v0 = +1e-14
    // crosses above 0 (peak ~ +1.6e-22) and MUST be rejected (not passed by a slack shortcut).
    let tiny_overshoot = QuinticPolynomial::new(0.0, 1e-14, 0.0, -0.5, 0.0, 0.0, 1.0).unwrap();
    assert!(tiny_overshoot.position(2e-8) > 0.0);
    assert!(tiny_overshoot.certify_within(-2.0, 0.0).is_err());
    // Same excursion starting strictly below 0 at -1e-25 must also be rejected.
    let sub_eps_overshoot = QuinticPolynomial::new(-1e-25, 1e-14, 0.0, -0.5, 0.0, 0.0, 1.0).unwrap();
    assert!(sub_eps_overshoot.position(2e-8) > 0.0);
    assert!(sub_eps_overshoot.certify_within(-2.0, 0.0).is_err());

    // Valid exact boundary touches at 0.0 still pass: constant hold, rest-to-rest approach (f32
    // executed contract), and an interior tangent maximum p(t) = -(t - 0.5)^2 <= 0 (Codex R2 Major 1).
    let hold_zero = QuinticPolynomial::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0).unwrap();
    assert!(hold_zero.certify_within(-2.0, 0.0).is_ok());
    let rest_to_zero = QuinticPolynomial::new(-0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0).unwrap();
    assert!(rest_to_zero.position(1.0 - 1e-6) <= 0.0);
    assert!(rest_to_zero.certify_within(-2.0, 0.0).is_ok());
    assert!(rest_to_zero.certify_f32_within(-2.0f32, 0.0f32).is_ok());
    let rest_fast = QuinticPolynomial::new(-0.5, 0.0, 0.0, f64::from(0.04f32), 0.0, 0.0, 0.2).unwrap();
    assert!(rest_fast.position(0.1999998) <= f64::from(0.04f32));
    assert!((rest_fast.position(0.1999998) as f32) <= 0.04f32);
    assert!(rest_fast.certify_within(f64::from(-2.61799f32), f64::from(0.04f32)).is_ok());
    assert!(rest_fast.certify_f32_within(-2.61799f32, 0.04f32).is_ok());
    // Interior tangent maximum p(t) = -(t - 0.5)^2 = -0.25 + t - t^2 touches 0 at t = 0.5.
    let tangent_touch = QuinticPolynomial {
        a0: -0.25,
        a1: 1.0,
        a2: -1.0,
        a3: 0.0,
        a4: 0.0,
        a5: 0.0,
        duration: 1.0,
        p0: -0.25,
        p1: -0.25,
        v0: 1.0,
        v1: -1.0,
        a_start: -2.0,
        a_end: -2.0,
    };
    assert!(tangent_touch.certify_within(-1.0, 0.0).is_ok());
    // Tiny positive shift above 0 at the interior peak MUST be rejected.
    let mut tangent_over = tangent_touch;
    tangent_over.a0 = -0.25 + 1e-9;
    tangent_over.p0 = tangent_over.a0;
    tangent_over.p1 = tangent_over.a0;
    assert!(tangent_over.certify_within(-1.0, 0.0).is_err());
    // Codex R2 Minor 1: missing limits slice still formats indexed joint name and bounds.
    let v_rt = motion::PlanningLimitViolationRt::new(2, 0.09, -2.61799, 0.04);
    let fallback_err = armd::rt_core::requests::actions::ActionRefusal::new("IK_UNREACHABLE")
        .into_arm_error_with_detail(1, Some(v_rt), None);
    let fallback_api = armd::api::rest::ApiError::from(fallback_err);
    assert_eq!(fallback_api.code.as_deref(), Some("IK_UNREACHABLE"));
    assert!(
        fallback_api.message.contains("joint_3") && fallback_api.message.contains("0.04"),
        "missing limits fallback must still name joint_3 and 0.04: {fallback_api:?}"
    );

    // Codex R1 Major 3: one-sided planning_max_position (omitted planning_min_position) still
    // enforces the default hard lower bound (-2.61799) across the continuous curve.
    let dip_below_hard_min = MultiJointQuinticTrajectory::new(
        &[0.0, 0.0, -2.55, 0.0, 0.0, 0.0, 0.0],
        &[0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0],
        &[0.0, 0.0, -2.50, 0.0, 0.0, 0.0, 0.0],
        &[0.0; 7],
        1.0,
    )
    .unwrap();
    let cfg = ArmConfig::rebot_dm();
    let e_lo = motion::check_quintic_planning_limits(&dip_below_hard_min, &cfg.limits, PlanningScope::arm_only(&cfg))
        .unwrap_err();
    assert_eq!(e_lo.code(), "IK_UNREACHABLE");
    assert!(e_lo.to_string().contains("joint_3") && e_lo.to_string().contains("-2.6180"), "{e_lo}");

    // Multi-joint wrapper names the offending joint and planning limit.
    let traj = MultiJointQuinticTrajectory::new(
        &[0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.0],
        &[0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0],
        &[0.0, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0],
        &[0.0; 7],
        1.5,
    )
    .unwrap();
    let e = motion::check_quintic_planning_limits(&traj, &cfg.limits, PlanningScope::arm_only(&cfg))
        .unwrap_err();
    assert_eq!(e.code(), "IK_UNREACHABLE");
    assert!(e.to_string().contains("joint_3") && e.to_string().contains("0.04"), "{e}");
}

/// Dual-owner commit-time state drift on Cartesian `move_to(..., path="joint")`: preparation
/// passes with `v0 = 0` at `J3 = 0.03998`, then an allowed live state change occurs before commit —
/// (a) sub-threshold live velocity `v0 = +0.0008 < 1e-3` at the SAME starting position causing
/// quintic overshoot past `0.04`, and (b) start drift `+8e-5 < START_TOLERANCE` moving live start
/// to `0.04006 > 0.04`. BOTH Legacy and RT Executor commit paths refuse with `IK_UNREACHABLE`
/// naming `joint_3` and `0.04` and start zero motion.
#[test]
fn commit_time_velocity_drift_refused_dual_owner() {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.backlash_approach = None;
    let start_q = [0.0, -0.3, 0.03998, 0.0, 0.0, 0.0, 0.0];
    // Move J1 by +0.15 rad while keeping J2..J6 unchanged so duration is ~0.5 s and J3 sits right at 0.03998.
    let target_q = [0.15, -0.3, 0.03998, 0.0, 0.0, 0.0, 0.0];
    let end = cfg.arm_model.fk_f32(&target_q).unwrap();
    let tcp = motion::end_to_tcp(&end, &cfg.tcp_offset);
    let quat = kinematics::r_to_quat(&tcp.r);
    let move_body = json!({
        "position": [tcp.p[0], tcp.p[1], tcp.p[2]],
        "orientation": {"quat": [quat[0], quat[1], quat[2], quat[3]]},
        "path": "joint",
        "speed_scale": 1.0
    });

    for (case_name, drift_pos, drift_vel) in [
        ("sub_threshold_vel_same_start", 0.0f32, 0.0008f32),
        ("within_tolerance_start_drift", 8e-5f32, 0.0f32),
    ] {
        // 5a. Legacy owner commit path (with active torque hold + workspace_violation sentinel
        // to prove Major 7: planning refusal does not mutate motion_limit or clear workspace_violation):
        {
            let mut leg = common::sim_at(cfg.clone(), &start_q);
            leg.torque_guard.hold = Some(armd::torque_guard::TorqueHold {
                joint: 0,
                hold_q: 0.05,
                sign: 1.0,
                torque_nm: 12.0,
                threshold_nm: 10.0,
                since_s: 0.0,
                anchor_q: 0.05,
                push_in: false,
                retreat: armd::torque_guard::Retreat::Unused,
            });
            leg.workspace_violation = Some(kinematics::WorkspaceViolationDetail {
                constraint_id: Some("sentinel".into()),
                geometry_id: Some("tcp".into()),
                violation_phase: Some("sweep".into()),
                reason: Some("keep".into()),
            });
            let adm = leg.current_admission_stamp();
            let snap = ActionSnapshot::from_inner(&leg, adm, Instant::now() + DEFAULT_ACTION_DEADLINE);
            let PreparedMoveAction::Joint(prep) = prepare_move_to(&snap, &move_body).unwrap() else {
                panic!("expected PreparedMoveAction::Joint");
            };
            assert!(prep.checked_joints.is_some(), "move_to path=joint must carry planning scope");
            leg.commanded_positions[2] += drift_pos;
            leg.commanded_velocities[2] = drift_vel;
            let ctx = OwnerCheckCtx::for_legacy(&leg);
            let stamp = prep.stamp;
            let err = install_move_joint(
                &mut leg,
                &stamp,
                &ctx,
                JointInstallPayload::Legacy(Box::new(prep)),
            )
            .unwrap_err();
            let api = armd::api::rest::ApiError::from(err);
            assert_eq!(api.code.as_deref(), Some("IK_UNREACHABLE"), "[legacy/{case_name}] {api:?}");
            assert!(
                api.message.contains("joint_3") && api.message.contains("0.04"),
                "[legacy/{case_name}] {api:?}"
            );
            assert!(leg.active_trajectory.is_none(), "[legacy/{case_name}] must start zero motion");
            assert!(leg.motion_limit.is_none(), "[legacy/{case_name}] refusal must not mutate motion_limit");
            assert!(
                leg.workspace_violation.as_ref().and_then(|w| w.constraint_id.as_deref()) == Some("sentinel"),
                "[legacy/{case_name}] refusal must not clear workspace_violation"
            );
        }

        // 5b. RT Executor commit path (same hold + sentinel check):
        {
            let mut rt = common::sim_at(cfg.clone(), &start_q);
            rt.torque_guard.hold = Some(armd::torque_guard::TorqueHold {
                joint: 0,
                hold_q: 0.05,
                sign: 1.0,
                torque_nm: 12.0,
                threshold_nm: 10.0,
                since_s: 0.0,
                anchor_q: 0.05,
                push_in: false,
                retreat: armd::torque_guard::Retreat::Unused,
            });
            rt.workspace_violation = Some(kinematics::WorkspaceViolationDetail {
                constraint_id: Some("sentinel".into()),
                geometry_id: Some("tcp".into()),
                violation_phase: Some("sweep".into()),
                reason: Some("keep".into()),
            });
            let adm = rt.current_admission_stamp();
            let snap = ActionSnapshot::from_inner(&rt, adm, Instant::now() + DEFAULT_ACTION_DEADLINE);
            let PreparedMoveAction::Joint(prep) = prepare_move_to(&snap, &move_body).unwrap() else {
                panic!("expected PreparedMoveAction::Joint");
            };
            rt.commanded_positions[2] += drift_pos;
            rt.commanded_velocities[2] = drift_vel;
            let slot = prep.to_plan_slot(None).unwrap();
            let armd::rt_core::ops::Primitive::Action(armd::rt_core::requests::actions::StagedAction::Joint {
                kind,
                dof,
                start,
                target,
                duration_s,
                max_velocity,
                power_scale,
                settle,
                approach_then,
                checked_joints,
                ..
            }) = slot.primitive
            else {
                panic!("expected StagedAction::Joint");
            };
            let ctx = OwnerCheckCtx::for_legacy(&rt);
            let err = install_move_joint(
                &mut rt,
                &prep.stamp,
                &ctx,
                JointInstallPayload::Rt {
                    kind,
                    dof: usize::from(dof),
                    start,
                    target,
                    duration_s,
                    max_velocity,
                    power_scale,
                    settle,
                    approach_then,
                    checked_joints,
                },
            )
            .unwrap_err();
            let api = armd::api::rest::ApiError::from(err);
            assert_eq!(api.code.as_deref(), Some("IK_UNREACHABLE"), "[rt/{case_name}] {api:?}");
            assert!(
                api.message.contains("joint_3") && api.message.contains("0.04"),
                "[rt/{case_name}] {api:?}"
            );
            assert!(rt.active_trajectory.is_none(), "[rt/{case_name}] must start zero motion");
            assert!(rt.motion_limit.is_none(), "[rt/{case_name}] refusal must not mutate motion_limit");
            assert!(
                rt.workspace_violation.as_ref().and_then(|w| w.constraint_id.as_deref()) == Some("sentinel"),
                "[rt/{case_name}] refusal must not clear workspace_violation"
            );
        }
    }
}

/// IK search bounds edge cases, gripper scope, non-trailing/no-gripper profiles, swept-joint
/// path, multiview start check, adaptive grasp refusal propagation, and f32 interpolation clamp.
#[test]
fn ik_bounds_scopes_multiview_adaptive_grasp_and_lerp_clamp() {
    // 6a. IK search bounds: [0.99, 1.00] inside [-1.0, 1.0] must NOT collapse to [0.99, 0.99].
    let edge_lim = vec![
        JointLimit::new("j1", -1.0, 1.0, 1.0, 1.0)
            .with_planning_min(0.99)
            .with_planning_max(1.0),
        JointLimit::new("j2", -1.0, 1.0, 1.0, 1.0).with_planning_min(0.99),
        JointLimit::new("j3", -0.01, 0.01, 1.0, 1.0), // narrow hard range without custom fields
        JointLimit::new("j4", -3.0, 3.0, 1.0, 1.0),   // standard hard range without custom fields
        JointLimit::new("j5", -2.0, 2.0, 1.0, 1.0),
        JointLimit::new("j6", -2.0, 2.0, 1.0, 1.0),
    ];
    let (lo, hi) = motion::ik_search_bounds(&edge_lim, 6).unwrap();
    assert!((lo[0] - f64::from(0.99f32)).abs() < 1e-7 && (hi[0] - 1.0).abs() < 1e-7, "{lo:?} {hi:?}");
    assert!(0.995 >= lo[0] && 0.995 <= hi[0], "0.995 must be inside [0.99, 1.00]");
    assert!((lo[1] - f64::from(0.99f32)).abs() < 1e-7 && (hi[1] - 1.0).abs() < 1e-7, "{lo:?} {hi:?}");
    assert!((lo[2] - f64::from(-0.01f32)).abs() < 1e-7 && (hi[2] - f64::from(0.01f32)).abs() < 1e-7);
    assert!((lo[3] - (-3.0 + motion::IK_LIMIT_MARGIN)).abs() < 1e-9);
    assert!((hi[3] - (3.0 - motion::IK_LIMIT_MARGIN)).abs() < 1e-9);

    // 6b. Static validation rejects invalid planning bounds.
    let mut bad_cfg = ArmConfig::rebot_dm();
    bad_cfg.limits[1].planning_max_position = Some(5.0);
    assert!(bad_cfg.validate().is_err());
    let mut bad_nan = ArmConfig::rebot_dm();
    bad_nan.limits[1].planning_min_position = Some(f32::NAN);
    assert!(bad_nan.validate().is_err());
    let mut bad_ord = ArmConfig::rebot_dm();
    bad_ord.limits[1].planning_min_position = Some(0.05);
    bad_ord.limits[1].planning_max_position = Some(0.03);
    assert!(bad_ord.validate().is_err());

    // 6c. PlanningScope handles no-gripper and non-trailing gripper profiles, and custom gripper planning limit.
    let mut cfg_g = ArmConfig::rebot_dm();
    cfg_g.limits[6].planning_max_position = Some(-0.5);
    let arm_scope = PlanningScope::for_pose(&cfg_g, false);
    let full_scope = PlanningScope::for_pose(&cfg_g, true);
    let pose_with_high_g = [0.0, -0.5, -0.5, 0.0, 0.0, 0.0, 0.0];
    assert!(motion::check_planning_limits_f32(&pose_with_high_g, &cfg_g.limits, arm_scope, None).is_ok());
    let err_g = motion::check_planning_limits_f32(&pose_with_high_g, &cfg_g.limits, full_scope, None).unwrap_err();
    assert_eq!(err_g.code(), "IK_UNREACHABLE");
    assert!(err_g.to_string().contains("gripper") && err_g.to_string().contains("-0.5"));

    // Non-trailing gripper (index 2) and no-gripper profile.
    let mut non_trailing = ArmConfig::rebot_dm();
    non_trailing.gripper_joint = Some(2);
    let nt_arm = PlanningScope::arm_only(&non_trailing);
    assert!(!nt_arm.contains(2) && nt_arm.contains(0) && nt_arm.contains(6));

    // 6d. Swept joint path checks planning limits before search-margin InvalidCommand, AND
    // preserves original finite + IK_LIMIT_MARGIN guard on unconfigured sides (Codex R1 Major 4).
    let inner_ok = common::sim_at(ArmConfig::rebot_dm(), &INSPECT);
    let input = inner_ok.plan_input().unwrap();
    let e_swept = armd::grasp_select::feasibility::plan_swept_joint_path(
        &input,
        [0.0, -0.5, 0.08, 0.0, 0.0, 0.0],
        &[],
        -1.0,
    )
    .unwrap_err();
    assert_eq!(e_swept.code(), "IK_UNREACHABLE");
    assert!(e_swept.to_string().contains("joint_3") && e_swept.to_string().contains("0.04"), "{e_swept}");
    let e_margin = armd::grasp_select::feasibility::plan_swept_joint_path(
        &input,
        [3.139, -0.5, -0.5, 0.0, 0.0, 0.0],
        &[],
        -1.0,
    )
    .unwrap_err();
    assert_eq!(e_margin.code(), "INVALID_COMMAND", "unconfigured IK_LIMIT_MARGIN guard must remain: {e_margin}");
    let e_nan = armd::grasp_select::feasibility::plan_swept_joint_path(
        &input,
        [f64::NAN, -0.5, -0.5, 0.0, 0.0, 0.0],
        &[],
        -1.0,
    )
    .unwrap_err();
    assert_eq!(e_nan.code(), "IK_UNREACHABLE", "non-finite swept goal must be rejected: {e_nan}");
    // Codex R2 Major 5: narrow one-sided custom range [-1, 1] with planning_max = -0.99 still
    // enforces omitted-side IK_LIMIT_MARGIN (-1.0 + 0.02 = -0.98) on goal -0.995.
    let mut narrow_inp = input.clone();
    narrow_inp.limits[0] = JointLimit::new("joint_1", -1.0, 1.0, 2.0, 5.0).with_planning_max(-0.99);
    narrow_inp.q0[0] = -0.992;
    let e_narrow = armd::grasp_select::feasibility::plan_swept_joint_path(
        &narrow_inp,
        [-0.995, -0.5, -0.5, 0.0, 0.0, 0.0],
        &[],
        -1.0,
    )
    .unwrap_err();
    assert_eq!(
        e_narrow.code(),
        "INVALID_COMMAND",
        "narrow one-sided range must preserve omitted-side IK_LIMIT_MARGIN: {e_narrow}"
    );

    // 6e. Multiview upfront start check refuses when arm starts outside planning band.
    let mut mv_inner = common::sim_at(ArmConfig::rebot_dm(), &[0.0, 0.0, 0.08, 0.0, 0.0, 0.0, 0.0]);
    let ep = mv_inner.safety.epoch_id();
    let mv_err = armd::measure_multiview::prepare(&mv_inner, &json!({"label": "cup"}), ep).unwrap_err();
    let mv_api = armd::api::rest::ApiError::from(mv_err);
    assert_eq!(mv_api.code.as_deref(), Some("IK_UNREACHABLE"), "{mv_api:?}");
    assert!(mv_api.message.contains("joint_3") && mv_api.message.contains("0.04"), "{mv_api:?}");
    mv_inner.commanded_positions = INSPECT.to_vec();
    mv_inner.measured_positions = INSPECT.to_vec();
    assert!(armd::measure_multiview::prepare(&mv_inner, &json!({"label": "cup"}), ep).is_ok());

    // 6f. Adaptive grasp endpoint_ik surfaces named PlanningLimit error when planning cap blocks a hard-reachable pose.
    let pose_past_cap = kinematics::fk(&[0.0, 0.0, 0.12, 0.0, 0.0, 0.0]);
    let mut work = armd::grasp_select::feasibility::Work::default();
    let mut home_inp = common::sim_at(ArmConfig::rebot_dm(), &[0.0; 7]).plan_input().unwrap();
    home_inp.goal_frame = GoalFrame::Tcp;
    let ik_err = armd::grasp_select::feasibility::endpoint_ik(
        &home_inp,
        &pose_past_cap,
        &[[0.0; 6]],
        &mut work,
    )
    .unwrap_err();
    let rej = ik_err.into_rejection("contact pose");
    assert_eq!(rej.code, "IK_UNREACHABLE");
    assert!(
        rej.message.contains("planning limit") && (rej.message.contains("joint_3") || rej.message.contains("joint_2")),
        "expected named planning-limit rejection from endpoint_ik, got: {rej:?}"
    );
    assert!(work.ik_iterations > 0, "diagnostic work must be counted in work.ik_iterations");

    // 6g. f32 interpolation endpoint clamp: qa=-0.5f32, qb=0.04f32, f=1.0 must never exceed 0.04f32.
    let path = JointPath {
        dof: 7,
        arm_dof: 6,
        q: vec![[0.0, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.0]],
        s: vec![0.0, 1.0],
        duration: 1.0,
        power_scale: 1.0,
    };
    let at_end = path.position_at_time(1.0);
    assert!(at_end[2] <= 0.04f32, "interpolated sample exceeded 0.04f32: {}", at_end[2]);
}

/// Custom mock driver returning configurable hardware position limits for driver-narrowing tests.
struct NarrowLimitsDriver {
    pos_limits: Vec<(f32, f32)>,
    q: [f32; 7],
    torqued: bool,
}

impl NarrowLimitsDriver {
    fn new(pos_limits: Vec<(f32, f32)>) -> Self {
        Self { pos_limits, q: [0.0; 7], torqued: false }
    }
}

impl ArmDriver for NarrowLimitsDriver {
    fn supports_rt_executor(&self) -> Result<(), &'static str> { Ok(()) }
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            velocity_feedback: true,
            torque_feedback: true,
            temperature_feedback: true,
            torque_feedforward: false,
            enable_feedback: true,
            load_feedback: false,
        }
    }
    fn enable_state(&self) -> EnableState {
        if self.torqued { EnableState::Enabled } else { EnableState::Disabled }
    }
    fn is_feedback_valid(&self) -> bool { true }
    fn is_enabled(&self) -> bool { self.torqued }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); 7] }
    fn feedback_all_within(&self, dof: usize, _: Duration) -> bool { dof == 7 }
    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) { out.clear(); out.resize(7, Some(Duration::from_millis(1))); }
    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) { out.clear(); out.resize(7, Some(1)); }
    fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) { out.clear(); out.resize(7, Some(Instant::now())); }
    fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) { out.clear(); out.resize(7, Some(Instant::now())); }
    fn motor_status_into(&self, out: &mut Vec<u8>) -> bool { out.clear(); out.resize(7, u8::from(self.torqued)); true }
    fn fresh_joint_positions_into(
        &self,
        indices: &[usize],
        _max_age: Duration,
        out: &mut [f32],
    ) -> bool {
        if out.len() < indices.len() { return false; }
        for (slot, &idx) in out.iter_mut().zip(indices) {
            *slot = self.q.get(idx).copied().unwrap_or(0.0);
        }
        true
    }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        Ok(self.q.iter().map(|&position| JointState { position, velocity: 0.0, torque: 0.0, temperature: 30.0 }).collect())
    }
    fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
        out.clear();
        for &position in &self.q {
            out.push(JointState { position, velocity: 0.0, torque: 0.0, temperature: 30.0 });
        }
        Ok(())
    }
    fn read_joint_states_budgeted(&mut self, out: &mut Vec<JointState>, _: Instant, _: u16) -> Result<(u16, bool), ArmDriverError> {
        self.read_joint_states_into(out)?;
        Ok((7, false))
    }
    fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
        out.clear();
        for &position in &self.q {
            out.push(JointState { position, velocity: 0.0, torque: 0.0, temperature: 30.0 });
        }
        Some(())
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        for (i, t) in targets.iter().enumerate().take(7) {
            self.q[i] = t.position;
        }
        Ok(())
    }
    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        self.torqued = enabled;
        Ok(())
    }
    fn position_limits(&self, joint: usize) -> Option<(f32, f32)> {
        self.pos_limits.get(joint).copied()
    }
}

/// Driver narrowing clipping/emptying a planning bound preserves hard-limit admission/rearm/recovery
/// (no FaultKind::JointLimit, no driver_config_error) while refusing autonomous planning; a genuine
/// hard-limit conflict still sets driver_config_error and latches JointLimit.
#[test]
fn driver_narrowing_empty_planning_range_never_faults_hard_limits() {
    let mut cfg = ArmConfig::rebot_dm();
    // Set J2 planning range to [-2.0, -0.5] and rest/home at 0.0; then driver narrows J2 hard
    // range to [-0.2, 2.0] (valid hard width 2.2, contains home/rest 0.0, but empty planning intersection!).
    cfg.limits[1].planning_min_position = Some(-2.0);
    cfg.limits[1].planning_max_position = Some(-0.5);
    let drv = NarrowLimitsDriver::new(vec![
        (-3.14159, 3.14159),
        (-0.2, 2.0), // disjoint from planning [-2.0, -0.5], still contains 0.0!
        (-2.61799, 2.61799),
        (-3.14159, 3.14159),
        (-2.0944, 2.0944),
        (-6.28318, 6.28318),
        (-5.75, 0.10),
    ]);
    let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
    inner.lease_enabled = false;
    inner.tick(0.02);
    assert!(
        inner.driver_config_error.is_none(),
        "planning-only empty intersection must NOT set driver_config_error: {:?}",
        inner.driver_config_error
    );
    assert!(
        inner.config.limits[1].effective_planning_range().is_none(),
        "effective planning range must be empty"
    );
    // safety_describe publishes null for the empty planning range.
    let desc = inner.safety_describe();
    assert!(desc["limits"]["joints"][1]["planning_min"].is_null(), "{desc}");
    assert!(desc["limits"]["joints"][1]["planning_max"].is_null(), "{desc}");

    // Torque on (rearm) and direct jog still work!
    {
        let s = inner.current_admission_stamp();
        inner
            .check_admission(s)
            .and_then(SupervisorInner::enable_torque)
            .expect("rearm must succeed when only planning intersection is empty");
    }
    let input = inner.plan_input().unwrap();
    let (goal, opts) = motion::resolve_goal_opts(
        &input,
        &GoalSpec::Absolute { p: [0.26, 0.0, 0.22], r: None },
    );
    let err = motion::plan_cartesian_opts(&input, goal, Speed::default(), opts).unwrap_err();
    assert_eq!(err.code(), "IK_UNREACHABLE");
    assert!(
        err.to_string().contains("joint_2") && err.to_string().contains("empty"),
        "expected empty planning range message naming joint_2, got: {err}"
    );

    // Genuine hard-limit conflict (driver width < 0.01) STILL fails closed into driver_config_error.
    let bad_drv = NarrowLimitsDriver::new(vec![
        (-3.14159, 3.14159),
        (2.094, 2.0941), // intersected width < 0.01
        (-2.61799, 2.61799),
        (-3.14159, 3.14159),
        (-2.0944, 2.0944),
        (-6.28318, 6.28318),
        (-5.75, 0.10),
    ]);
    let bad_inner = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(bad_drv));
    assert!(
        bad_inner.driver_config_error.is_some(),
        "genuine hard-limit conflict must still set driver_config_error"
    );
}

/// Codex R1 Major 1 & Minor 1: broker-level interleaved RT refusals carry per-request
/// `PlanningLimitViolationRt` diagnostics in each `Reply::Refused` without clobbering.
#[test]
fn rt_broker_interleaved_refusals_preserve_per_request_diagnostics() {
    use armd::rt_core::broker::{Capacities, ClientId, Reply, Submit};
    use armd::rt_core::ids::ArmId;
    use armd::rt_core::mailbox::MAX_DOF;
    use armd::rt_core::ops::{CoreOps, PlanSlot, Primitive, ReleaseEngine};
    use armd::rt_core::requests::actions::{
        ActionSubmit, PreparedActionKind, StagedAction,
    };

    let mut cfg = ArmConfig::rebot_dm();
    cfg.backlash_approach = None;
    let mut core = common::sim_at(cfg.clone(), &[0.0, -0.3, -0.3, 0.0, 0.0, 0.0, 0.0]);
    let boot = core.boot_id();
    let arm_scope = PlanningScope::arm_only(&cfg).mask;
    let (mut engine, mut broker, _tw, _stops) = ReleaseEngine::new(
        boot,
        Capacities { commands: 8, plans: 4, replies: 16 },
    );
    let snap = core.planning_snapshot(&armd::rt_core::ops::CommitCtx {
        arm: ArmId::LOCAL,
        boot,
        now: Instant::now(),
        stop_generation: core.current_stop_gen(),
    });
    let auth = snap.authority;
    let dl = Instant::now() + Duration::from_secs(10);

    // Request 1: violates joint_2 (0.08 > 0.03).
    let write_j2 = |slot: &mut PlanSlot| {
        let mut s = [0.0f32; MAX_DOF];
        let mut t = [0.0f32; MAX_DOF];
        s[..7].copy_from_slice(&[0.0, -0.3, -0.3, 0.0, 0.0, 0.0, 0.0]);
        t[..7].copy_from_slice(&[0.0, 0.08, -0.3, 0.0, 0.0, 0.0, 0.0]);
        slot.set(Primitive::Action(StagedAction::Joint {
            child: None,
            kind: PreparedActionKind::MoveToJoint,
            dof: 7,
            start: s,
            target: t,
            duration_s: 0.5,
            max_velocity: [2.0; MAX_DOF],
            power_scale: None,
            settle: false,
            approach_then: None,
            checked_joints: Some(arm_scope),
        }));
    };
    // Request 2: violates joint_3 (0.09 > 0.04).
    let write_j3 = |slot: &mut PlanSlot| {
        let mut s = [0.0f32; MAX_DOF];
        let mut t = [0.0f32; MAX_DOF];
        s[..7].copy_from_slice(&[0.0, -0.3, -0.3, 0.0, 0.0, 0.0, 0.0]);
        t[..7].copy_from_slice(&[0.0, -0.3, 0.09, 0.0, 0.0, 0.0, 0.0]);
        slot.set(Primitive::Action(StagedAction::Joint {
            child: None,
            kind: PreparedActionKind::MoveToJoint,
            dof: 7,
            start: s,
            target: t,
            duration_s: 0.5,
            max_velocity: [2.0; MAX_DOF],
            power_scale: None,
            settle: false,
            approach_then: None,
            checked_joints: Some(arm_scope),
        }));
    };
    let r_j2 = broker
        .submit(
            ClientId(0),
            1,
            auth,
            dl,
            Submit::Action(ActionSubmit::InstallPrepared {
                write: &write_j2,
                kind: PreparedActionKind::MoveToJoint,
                prepared_boot: boot,
                admission_request_id: 101,
                check_workspace_revision: false,
                poses_revision: None,
                calibration_revision: None,
                gripper_calib_revision: None,
                power_budget_revision: None,
            }),
        )
        .unwrap();
    let r_j3 = broker
        .submit(
            ClientId(0),
            2,
            auth,
            dl,
            Submit::Action(ActionSubmit::InstallPrepared {
                write: &write_j3,
                kind: PreparedActionKind::MoveToJoint,
                prepared_boot: boot,
                admission_request_id: 102,
                check_workspace_revision: false,
                poses_revision: None,
                calibration_revision: None,
                gripper_calib_revision: None,
                power_budget_revision: None,
            }),
        )
        .unwrap();

    // Step two releases BEFORE polling any reply: both refusals sit in the reply ring simultaneously.
    let t1 = Instant::now();
    let _ = engine.step(&mut core, &mut || t1, 0.02, t1 + Duration::from_millis(20), 1);
    let t2 = t1 + Duration::from_millis(20);
    let _ = engine.step(&mut core, &mut || t2, 0.02, t2 + Duration::from_millis(20), 2);

    let mut replies = Vec::new();
    broker.poll(|_, r| {
        replies.push(r);
        true
    });
    assert_eq!(replies.len(), 2, "expected 2 replies: {replies:?}");
    let Reply::Refused { request: req1, code: c1, detail: Some(d1) } = replies[0] else {
        panic!("expected Refused with detail for r_j2, got {:?}", replies[0]);
    };
    let Reply::Refused { request: req2, code: c2, detail: Some(d2) } = replies[1] else {
        panic!("expected Refused with detail for r_j3, got {:?}", replies[1]);
    };
    assert_eq!((req1, c1), (r_j2, "IK_UNREACHABLE"));
    assert_eq!((req2, c2), (r_j3, "IK_UNREACHABLE"));
    let m1 = d1.into_motion_error(&cfg.limits, None).to_string();
    let m2 = d2.into_motion_error(&cfg.limits, None).to_string();
    assert!(m1.contains("joint_2") && m1.contains("0.03"), "r_j2 must report joint_2/0.03: {m1}");
    assert!(m2.contains("joint_3") && m2.contains("0.04"), "r_j3 must report joint_3/0.04: {m2}");

    // Codex R2 Major 4: generic CommandKind::Install (`Primitive::Prepared` Cartesian) also
    // enforces planning limits before admission and carries per-request detail in `Reply::Refused`.
    let write_prepared_cart = |slot: &mut PlanSlot| {
        let mut s = [0.0f32; MAX_DOF];
        s[..7].copy_from_slice(&[0.0, -0.3, -0.3, 0.0, 0.0, 0.0, 0.0]);
        let mut bad = [0.0f32; MAX_DOF];
        bad[..7].copy_from_slice(&[0.0, -0.3, 0.09, 0.0, 0.0, 0.0, 0.0]);
        slot.path.clear();
        slot.path.push(&s[..7], 0.0).unwrap();
        slot.path.push(&bad[..7], 1.0).unwrap();
        slot.primitive = Primitive::Prepared {
            child: None,
            dof: 7,
            start: s,
            kind: armd::rt_core::ops::PathKind::Cartesian,
            duration_s: 0.5,
            power_scale: None,
        };
    };
    let r_cart = broker
        .submit(ClientId(0), 3, auth, dl, Submit::Install(&write_prepared_cart))
        .unwrap();
    let t3 = t2 + Duration::from_millis(20);
    let _ = engine.step(&mut core, &mut || t3, 0.02, t3 + Duration::from_millis(20), 3);
    let mut cart_replies = Vec::new();
    broker.poll(|_, r| {
        cart_replies.push(r);
        true
    });
    assert_eq!(cart_replies.len(), 1, "{cart_replies:?}");
    let Reply::Refused { request: req_c, code: cc, detail: Some(dc) } = cart_replies[0] else {
        panic!("expected Refused with detail for Primitive::Prepared Cartesian, got {:?}", cart_replies[0]);
    };
    assert_eq!((req_c, cc), (r_cart, "IK_UNREACHABLE"));
    let mc = dc.into_motion_error(&cfg.limits, None).to_string();
    assert!(mc.contains("joint_3") && mc.contains("0.04"), "prepared Cartesian must report joint_3/0.04: {mc}");
}

/// Codex R2 Majors 2 & 3: torque hold activated after preparation on a two-leg same-side scoped
/// move (`move_to path="joint" precise=true`) clamps leg 1, rebuilds leg 2 continuously from the
/// clamped leg-1 endpoint (zero jump), and preserves `MotionLimitReason::Torque` bookkeeping
/// identically on BOTH Legacy and RT Executor owners.
#[test]
fn post_prepare_hold_preserves_leg2_continuity_and_clamp_bookkeeping_dual_owner() {
    let cfg = ArmConfig::rebot_dm();
    let start_q = [0.0, -0.4, -0.4, 0.0, 0.0, 0.0, 0.0];
    let target_q = [0.25, -0.4, -0.4, 0.0, 0.0, 0.0, 0.0];
    let end = cfg.arm_model.fk_f32(&target_q).unwrap();
    let tcp = motion::end_to_tcp(&end, &cfg.tcp_offset);
    let quat = kinematics::r_to_quat(&tcp.r);
    let move_body = json!({
        "position": [tcp.p[0], tcp.p[1], tcp.p[2]],
        "orientation": {"quat": [quat[0], quat[1], quat[2], quat[3]]},
        "path": "joint",
        "precise": true,
        "speed_scale": 1.0
    });
    let hold = armd::torque_guard::TorqueHold {
        joint: 0,
        hold_q: 0.10,
        sign: 1.0,
        torque_nm: 12.0,
        threshold_nm: 10.0,
        since_s: 0.0,
        anchor_q: 0.10,
        push_in: false,
        retreat: armd::torque_guard::Retreat::Unused,
    };

    // Legacy owner:
    let mut leg = common::sim_at(cfg.clone(), &start_q);
    let adm = leg.current_admission_stamp();
    let snap = ActionSnapshot::from_inner(&leg, adm, Instant::now() + DEFAULT_ACTION_DEADLINE);
    let PreparedMoveAction::Joint(prep_leg) = prepare_move_to(&snap, &move_body).unwrap() else {
        panic!("expected PreparedMoveAction::Joint");
    };
    assert!(prep_leg.approach_leg.is_some(), "test requires two-leg same-side plan");
    leg.torque_guard.hold = Some(hold);
    let ctx_leg = OwnerCheckCtx::for_legacy(&leg);
    let stamp_leg = prep_leg.stamp;
    install_move_joint(&mut leg, &stamp_leg, &ctx_leg, JointInstallPayload::Legacy(Box::new(prep_leg))).unwrap();
    assert_eq!(
        leg.motion_limit.as_ref().map(|m| m.reason),
        Some(armd::model::MotionLimitReason::Torque),
        "Legacy accepted move must record Torque clamp bookkeeping"
    );
    let active_leg = leg.active_trajectory.as_ref().expect("active trajectory installed");
    let leg1_end_j0 = match &active_leg.reference {
        motion::Reference::Quintic(q) => q.polynomials[0].p1,
        _ => panic!("expected quintic leg 1"),
    };
    let leg2_start_j0 = active_leg.then.as_ref().expect("leg 2 installed").polynomials[0].p0;
    assert!(
        (leg1_end_j0 - 0.10).abs() < 1e-5 && (leg2_start_j0 - leg1_end_j0).abs() < 1e-6,
        "Legacy leg 2 must start continuously at clamped leg 1 end: leg1_end={leg1_end_j0}, leg2_start={leg2_start_j0}"
    );

    // RT Executor owner:
    let mut rt = common::sim_at(cfg, &start_q);
    let adm_rt = rt.current_admission_stamp();
    let snap_rt = ActionSnapshot::from_inner(&rt, adm_rt, Instant::now() + DEFAULT_ACTION_DEADLINE);
    let PreparedMoveAction::Joint(prep_rt) = prepare_move_to(&snap_rt, &move_body).unwrap() else {
        panic!("expected PreparedMoveAction::Joint");
    };
    rt.torque_guard.hold = Some(hold);
    let slot = prep_rt.to_plan_slot(None).unwrap();
    let armd::rt_core::ops::Primitive::Action(armd::rt_core::requests::actions::StagedAction::Joint {
        kind, dof, start, target, duration_s, max_velocity, power_scale, settle, approach_then, checked_joints, ..
    }) = slot.primitive else {
        panic!("expected StagedAction::Joint");
    };
    let ctx_rt = OwnerCheckCtx::for_legacy(&rt);
    install_move_joint(
        &mut rt,
        &prep_rt.stamp,
        &ctx_rt,
        JointInstallPayload::Rt {
            kind, dof: usize::from(dof), start, target, duration_s, max_velocity, power_scale, settle, approach_then, checked_joints,
        },
    )
    .unwrap();
    assert_eq!(
        rt.motion_limit.as_ref().map(|m| m.reason),
        Some(armd::model::MotionLimitReason::Torque),
        "RT accepted move must record Torque clamp bookkeeping"
    );
    let active_rt = rt.active_trajectory.as_ref().expect("active trajectory installed");
    let rt_leg1_end_j0 = match &active_rt.reference {
        motion::Reference::Quintic(q) => q.polynomials[0].p1,
        _ => panic!("expected quintic leg 1"),
    };
    let rt_leg2_start_j0 = active_rt.then.as_ref().expect("leg 2 installed").polynomials[0].p0;
    assert!(
        (rt_leg1_end_j0 - 0.10).abs() < 1e-5 && (rt_leg2_start_j0 - rt_leg1_end_j0).abs() < 1e-6,
        "RT leg 2 must start continuously at clamped leg 1 end: leg1_end={rt_leg1_end_j0}, leg2_start={rt_leg2_start_j0}"
    );
    assert!(
        (leg2_start_j0 - rt_leg2_start_j0).abs() < 1e-6,
        "Legacy and RT leg 2 start must match identically"
    );
}

/// Codex R1 Major 5 & Minor 3: with driver narrowing emptying J2's planning intersection,
/// BOTH Legacy and RT Executor owners preserve `jog`, `home`, `park` (`unload`), and explicit
/// saved-pose `move_to_pose` with its configured `same_side` backlash approach (no fallback to
/// `direct`, no `JointLimit` fault).
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn driver_narrowing_empty_planning_recovery_and_saved_pose_backlash_dual_owner() {
    for executor in [false, true] {
        let label = if executor { "executor" } else { "legacy" };
        let mut cfg = ArmConfig::rebot_dm();
        cfg.limits[1].planning_min_position = Some(-2.0);
        cfg.limits[1].planning_max_position = Some(-0.5);
        cfg.lease_timeout_ms = 60_000;
        let drv = NarrowLimitsDriver::new(vec![
            (-3.14159, 3.14159),
            (-0.2, 2.0), // disjoint from planning [-2.0, -0.5], still contains 0.0 and -0.12!
            (-2.61799, 2.61799),
            (-3.14159, 3.14159),
            (-2.0944, 2.0944),
            (-6.28318, 6.28318),
            (-5.75, 0.10),
        ]);
        let mut rig = OwnerRig::spawn_with_driver(executor, cfg, [0.0; 7], Some(Box::new(drv))).await;
        // Verify driver narrowing actually produced an empty J2 planning range without faulting.
        let desc: Value = reqwest::get(format!("http://{}/api/v1/safety/describe", rig.addr))
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert!(desc["limits"]["joints"][1]["planning_min"].is_null(), "[{label}] {desc}");
        assert!(desc["limits"]["joints"][1]["planning_max"].is_null(), "[{label}] {desc}");

        // 1. Direct jog works cleanly with empty planning intersection.
        let jog_ack = rig
            .rpc(
                "/api/v1/motion/jog",
                json!({"target_positions": [0.0, 0.05, -0.2, 0.0, 0.0, 0.0, 0.0], "duration_secs": 0.1}),
            )
            .await;
        assert_eq!(jog_ack["type"], "command_ack", "[{label}] jog: {jog_ack}");
        let st_j = rig.wait_action_done(jog_ack["action_id"].as_u64().unwrap()).await;
        assert_eq!(st_j.state, "done", "[{label}] jog terminal: {st_j:?}");

        // 2. Recovery home succeeds cleanly.
        let home_ack = rig.rpc("/api/v1/motion/home", json!({"duration_secs": 0.1})).await;
        assert_eq!(home_ack["type"], "command_ack", "[{label}] home: {home_ack}");
        let st_h = rig.wait_action_done(home_ack["action_id"].as_u64().unwrap()).await;
        assert_eq!(st_h.state, "done", "[{label}] home terminal: {st_h:?}");

        // 3. Explicit saved pose keeps configured same_side backlash approach even though J2's
        // planning intersection is empty (Major 5: pre_target uses hard limits for excluded poses).
        let save_ack = rig
            .rpc(
                "/api/v1/poses/save",
                json!({"name": "near_zero", "positions": [0.0, -0.12, -0.2, 0.0, 0.0, 0.0, 0.0]}),
            )
            .await;
        assert_eq!(save_ack["type"], "command_ack", "[{label}] save pose: {save_ack}");
        let pose_ack = rig
            .rpc(
                "/api/v1/action/move_to_pose",
                json!({"name": "near_zero", "speed_scale": 1.0, "approach": "same_side"}),
            )
            .await;
        assert_eq!(pose_ack["type"], "command_ack", "[{label}] move_to_pose: {pose_ack}");
        let st_p = rig.wait_action_done(pose_ack["action_id"].as_u64().unwrap()).await;
        assert_eq!(st_p.state, "done", "[{label}] move_to_pose terminal: {st_p:?}");
        assert_eq!(
            st_p.phase.as_deref(),
            Some("approach"),
            "[{label}] saved pose must execute the same_side approach leg when planning range is empty: {st_p:?}"
        );
        if !executor {
            assert_eq!(
                st_p.approach.as_deref(),
                Some("same_side"),
                "[{label}] saved pose approach metadata: {st_p:?}"
            );
        }

        // 4. Park (unload) succeeds with zero JointLimit fault.
        let park_ack = rig.rpc("/api/v1/motion/unload", json!({})).await;
        assert_eq!(park_ack["type"], "command_ack", "[{label}] park: {park_ack}");
        let st_u = rig.wait_action_done(park_ack["action_id"].as_u64().unwrap()).await;
        assert_eq!(st_u.state, "done", "[{label}] park terminal: {st_u:?}");
        let obs = rig.sup.get_observation().await;
        assert!(
            !matches!(obs.safety_state, armd::safety::SafetyState::Fault(_)),
            "[{label}] unexpected fault: {:?}",
            obs.safety_state
        );
    }
}
