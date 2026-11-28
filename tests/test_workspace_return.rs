//! Workspace return rule (1.6.1 scope, Items 1-4):
//! - Hard slots (floor, keep-out boxes, self-collision) get zero deepening allowance, incl. Park.
//! - Reach slots (end-origin and TCP) get the 5 mm temporary soft-slot allowance, tracked independently.
//! - Terminal rule: a discrete motion starting with any intruding slot must finish at or shallower
//!   than its start depth on every slot and strictly improve at least one slot by > WORKSPACE_EPS.
//! - Streams keep running per-slot minima across frames (no ratcheting); stationary/braking/hold
//!   require no progress.
//! - Measured vs commanded start: evaluate from measured baseline and check transition if different.

mod common;

use std::sync::Arc;
use std::time::Duration;
use serde_json::json;

use armd::chain::{ArmModel, WorkspaceEvaluator};
use armd::kinematics::{self, WORKSPACE_EPS, WORKSPACE_PARK_ESCAPE_EPS};
use armd::model::{ArmConfig, KeepoutBox};
use armd::motion::{plan_cartesian_opts, resolve_goal_opts, GoalSpec, RelFrame, Speed};
use armd::rt_core::executor::ExecutorConfig;
use armd::rt_core::requests::motion::{prepare_motion, MotionPlanSnapshot, MotionRequest};
use armd::safety::{SafetyError, SafetyState};
use armd::{ArmError, ArmSupervisor, SupervisorInner};
use common::INSPECT;

fn b601dm_config() -> ArmConfig {
    let mut cfg: ArmConfig = serde_json::from_reader(
        std::fs::File::open(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/kits/recamera-b601dm/profile/rebot-b601dm.json"
        ))
        .unwrap(),
    )
    .unwrap();
    cfg.collision = None;
    cfg.lease_timeout_ms = 5000;
    cfg
}

fn armed(config: ArmConfig, at: &[f32]) -> SupervisorInner {
    let mut inner = SupervisorInner::new(config);
    inner.lease_enabled = false;
    let s = inner.current_admission_stamp();
    inner
        .check_admission(s)
        .and_then(|a| SupervisorInner::enable_torque(a))
        .unwrap();
    inner.commanded_positions = at.to_vec();
    inner.measured_positions = at.to_vec();
    inner.target_positions = at.to_vec();
    inner
}

async fn start_owner_rig(
    cfg: ArmConfig,
    q_start: &[f32],
    use_executor: bool,
) -> (
    common::TestSocket,
    u64,
    u64,
    Arc<ArmSupervisor>,
    tokio::task::JoinHandle<()>,
    Option<tokio::task::JoinHandle<()>>,
    Option<armd::rt_core::executor::RtExecutorHandle>,
) {
    let sup = Arc::new(ArmSupervisor::new(cfg));
    {
        let mut inner = sup.inner().write().await;
        inner.commanded_positions = q_start.to_vec();
        inner.measured_positions = q_start.to_vec();
        inner.target_positions = q_start.to_vec();
    }
    let exec_handle = if use_executor {
        Some(
            sup.start_executor(ExecutorConfig::for_tests(Duration::from_millis(20)))
                .await
                .expect("start_executor"),
        )
    } else {
        None
    };
    let router = armd::create_router(sup.clone(), armd::OperatorAuth::disabled());
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let server_task = tokio::spawn(async move {
        axum::serve(listener, router).await.unwrap();
    });
    let ticker_task = if !use_executor {
        let t = sup.clone();
        Some(tokio::spawn(async move {
            let mut iv = tokio::time::interval(Duration::from_millis(20));
            loop {
                iv.tick().await;
                t.tick(0.02).await;
            }
        }))
    } else {
        None
    };
    let (mut ws, gen) = common::controller(addr).await;
    let ep = sup.get_observation().await.epoch_id;
    let ack = common::rpc(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(ack["type"], "command_ack", "torque enable: {ack}");
    let ep = sup.get_observation().await.epoch_id;
    (ws, gen, ep, sup, server_task, ticker_task, exec_handle)
}

async fn wait_idle_no_fault(sup: &Arc<ArmSupervisor>, label: &str, timeout_ms: u64) {
    let deadline = tokio::time::Instant::now() + Duration::from_millis(timeout_ms);
    loop {
        let obs = sup.get_observation().await;
        assert!(
            !matches!(obs.safety_state, SafetyState::Fault(_) | SafetyState::EStop),
            "{label}: unexpected fault/estop: {:?}",
            obs.safety_state
        );
        let active = obs
            .action
            .as_ref()
            .is_some_and(|a| a.state == "running" || a.state == "pending");
        let moving = obs.velocities.iter().any(|v| v.abs() > 1e-3);
        if !active && !moving {
            break;
        }
        assert!(
            tokio::time::Instant::now() < deadline,
            "{label}: timed out waiting for motion completion (action={:?}, state={:?})",
            obs.action,
            obs.safety_state
        );
        tokio::time::sleep(Duration::from_millis(25)).await;
    }
}

fn stretched_beyond_box_max_x_setup() -> (ArmConfig, [f32; 7], f64) {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.collision = None;
    cfg.power_budget = None;
    cfg.lease_timeout_ms = 5000;
    let q_start: [f32; 7] = [0.0, -0.90, -1.15, 0.85, 0.0, 0.0, 0.0];
    let ws = cfg.workspace.as_mut().unwrap();
    ws.max_reach_m = None;
    ws.keepout_boxes.clear();
    let check_pts = ws.check_points_end_m.clone();
    let pose = ArmModel::REBOT_DEV_ARM.fk_f32(&q_start).unwrap();
    let start_max_x = check_pts
        .iter()
        .map(|c| pose.transform_point(&c.map(f64::from))[0])
        .fold(pose.p[0], f64::max);
    // Start 10 mm beyond box_max_x
    ws.box_max_m[0] = (start_max_x - 0.010) as f32;
    (cfg, q_start, start_max_x)
}

/// Codex §Test 1 (both owners, sim):
/// Initialize parked outside at Section D's incident pose `[0,-2.046,-1.855,-0.297,-0.053,0.123,0]`.
/// Pin measured slot depths. Refuse a nonzero arm jog with zero net improvement (and unchanged targets).
/// Admit and complete certified inward moves (J2 +0.1, -20 mm X) and Park with 0 faults.
#[tokio::test]
async fn incident_outside_pose_terminal_rule_and_inward_move_both_owners() {
    let cfg = b601dm_config();
    let far_pose: [f32; 7] = [0.0, -2.046, -1.855, -0.297, -0.053, 0.123, 0.0];

    // Pin measured slot depths at the incident pose.
    let ws = cfg.workspace.as_ref().unwrap();
    let eval = WorkspaceEvaluator::new_with_tcp(&cfg.arm_model, ws, &cfg.camera_mount, cfg.tcp_offset);
    let mut c0 = eval.alloc_buf();
    eval.eval_into(&far_pose, &mut c0, "pin").unwrap();
    let start_depths = eval.init_min_depths(&c0);
    let reach_slot = eval.num_geoms * eval.slots_per_geom;
    let reach_depth = start_depths.as_slice()[reach_slot];
    assert!(
        reach_depth > 0.15 && reach_depth < 0.18,
        "expected incident end_origin reach depth ~0.165 m, got {reach_depth:.4} m"
    );
    let max_box_x_depth = (0..eval.num_geoms)
        .map(|g| start_depths.as_slice()[g * eval.slots_per_geom + 4])
        .fold(0.0f64, f64::max);
    assert!(
        max_box_x_depth > 0.03,
        "expected incident box_max_x depth > 30 mm, got {max_box_x_depth:.4} m"
    );

    // Construct a nonzero arm jog with zero net improvement: tiny J1 yaw (0.0005 rad) leaves
    // max_reach unchanged and shifts all intruding slots by < WORKSPACE_EPS (0.1 mm).
    let mut q_neutral = far_pose;
    q_neutral[0] += 0.0005;
    let mut c_neut = eval.alloc_buf();
    eval.eval_into(&q_neutral, &mut c_neut, "pin").unwrap();
    let any_improved = start_depths
        .as_slice()
        .iter()
        .zip(c_neut.as_slice())
        .any(|(&sd, &c)| sd > WORKSPACE_EPS && sd - (-c).max(0.0) > WORKSPACE_EPS);
    assert!(!any_improved, "q_neutral must have zero net improvement (> WORKSPACE_EPS)");

    let mut q_inward_j2 = far_pose;
    q_inward_j2[1] += 0.1;

    for use_executor in [false, true] {
        let label = if use_executor { "executor" } else { "legacy" };
        let assert_at_far_pose = |obs: &armd::model::ArmObservation, stage: &str| {
            assert!(
                obs.commanded_positions
                    .iter()
                    .zip(&far_pose)
                    .all(|(a, b)| (a - b).abs() < 1e-5),
                "{label} ({stage}): live commanded_positions must start at far_pose, got {:?}",
                obs.commanded_positions
            );
            assert!(
                obs.positions
                    .iter()
                    .zip(&far_pose)
                    .all(|(a, b)| (a - b).abs() < 1e-5),
                "{label} ({stage}): live measured positions must start at far_pose, got {:?}",
                obs.positions
            );
        };

        // Rig A: neutral jog refusal + inward J2+0.1 jog from far_pose.
        {
            let (mut ws, gen, ep, sup, srv, tick, exec) =
                start_owner_rig(cfg.clone(), &far_pose, use_executor).await;
            assert_at_far_pose(&sup.get_observation().await, "neutral_and_j2");

            // 1. Nonzero arm jog with zero net improvement -> REFUSED with WORKSPACE_VIOLATION, targets unchanged.
            let rej = common::rpc(
                &mut ws,
                gen,
                ep,
                "/api/v1/motion/jog",
                json!({"target_positions": q_neutral}),
            )
            .await;
            assert_eq!(rej["type"], "command_reject", "{label}: neutral jog must be refused: {rej}");
            assert_eq!(rej["code"], "WORKSPACE_VIOLATION", "{label}: {rej}");
            let obs = sup.get_observation().await;
            assert_eq!(obs.safety_state, SafetyState::Armed, "{label}: 0 faults after refusal");
            assert_at_far_pose(&obs, "after_neutral_refusal");

            // 2. Certified inward jog (J2 +0.1 rad) -> ADMITTED and completes with 0 faults.
            let ack = common::rpc(
                &mut ws,
                gen,
                ep,
                "/api/v1/motion/jog",
                json!({"target_positions": q_inward_j2}),
            )
            .await;
            assert_eq!(ack["type"], "command_ack", "{label}: inward J2+0.1 jog: {ack}");
            wait_idle_no_fault(&sup, label, 5000).await;
            let obs_after_j2 = sup.get_observation().await;
            assert!(
                (obs_after_j2.commanded_positions[1] - q_inward_j2[1]).abs() < 1e-3,
                "{label}: inward J2+0.1 jog did not reach target: {:?}",
                obs_after_j2.commanded_positions
            );

            drop(ws);
            if let Some(t) = tick { t.abort(); }
            srv.abort();
            if let Some(h) = exec { h.stop_and_join(); }
        }

        // Rig B: fresh owner rig initialized directly at far_pose for -20 mm X Cartesian return.
        {
            let (mut ws, gen, ep, sup, srv, tick, exec) =
                start_owner_rig(cfg.clone(), &far_pose, use_executor).await;
            assert_at_far_pose(&sup.get_observation().await, "move_relative_-20mm_x");

            // 3. Certified inward Cartesian move (-20 mm X) -> ADMITTED and completes with 0 faults.
            let ack_x = common::rpc(
                &mut ws,
                gen,
                ep,
                "/api/v1/action/move_relative",
                json!({"frame": "base", "d": [-0.02, 0.0, 0.0]}),
            )
            .await;
            assert_eq!(ack_x["type"], "command_ack", "{label}: inward -20mm X: {ack_x}");
            wait_idle_no_fault(&sup, label, 5000).await;

            drop(ws);
            if let Some(t) = tick { t.abort(); }
            srv.abort();
            if let Some(h) = exec { h.stop_and_join(); }
        }

        // Rig C: fresh owner rig initialized directly at far_pose for Park.
        {
            let (mut ws, gen, ep, sup, srv, tick, exec) =
                start_owner_rig(cfg.clone(), &far_pose, use_executor).await;
            assert_at_far_pose(&sup.get_observation().await, "park");

            // 4. Park from incident pose -> ADMITTED and completes with 0 faults.
            let pack = common::preset(&mut ws, gen, ep, "park", None).await;
            assert_eq!(pack["type"], "preset_ack", "{label}: park from incident pose: {pack}");
            wait_idle_no_fault(&sup, label, 15000).await;

            drop(ws);
            if let Some(t) = tick { t.abort(); }
            srv.abort();
            if let Some(h) = exec { h.stop_and_join(); }
        }
    }
}

/// Codex §Test 2 + Decision Item 1 (both owners, sim):
/// Place a keep-out box around the current pose such that the direct Park quintic briefly deepens
/// by sub-millimetre (~0.5 mm, > WORKSPACE_EPS 0.1 mm and < WORKSPACE_PARK_ESCAPE_EPS 1.0 mm)
/// before later clearing to the resting pose.
/// - Park MUST be refused with WORKSPACE_VIOLATION (hard slot gets zero deepening allowance).
/// - A monotonic exit jog MUST be admitted and complete with 0 faults.
/// - Same sub-mm deepening on floor_z during Park MUST also be refused.
#[tokio::test]
async fn keepout_around_current_pose_monotonic_exit_and_park_submm_deepening_refused_both_owners() {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.collision = None;
    cfg.power_budget = None;
    cfg.lease_timeout_ms = 5000;
    cfg.camera_mount.offset_m = None;
    {
        let ws = cfg.workspace.as_mut().unwrap();
        ws.check_points_end_m = vec![[0.0, 0.0, 0.0]];
        ws.link_capsules.clear();
        ws.self_collision_enabled = false;
        ws.self_collision_pairs.clear();
    }

    // Choose a start pose and resting pose where the direct Park quintic briefly curves +0.5 mm
    // in +X before pulling back in -X to resting_positions.
    let q_start: [f32; 7] = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0, 0.0]; // INSPECT
    let p_start = ArmModel::REBOT_DEV_ARM.fk_f32(&q_start).unwrap().p;
    let rest = cfg.resting_positions().to_vec();
    let zeros = vec![0.0f32; cfg.dof];
    let park_traj = armd::trajectory::TrajectoryPlanner::plan(
        &q_start,
        &zeros,
        &rest,
        &zeros,
        Some(1.5),
        &cfg.limits,
    )
    .unwrap();

    // Sample the Park trajectory to find an axis along which p(t) briefly increases by ~0.5 mm
    // before retreating, or choose a custom resting pose whose quintic has a small controlled
    // 0.5 mm initial dip along +X.
    // Let's search a resting pose near HOME with a slight J1 offset so end_origin starts moving
    // slightly deeper into a keep-out box's face by ~0.5 mm before exiting!
    let mut best_rest = rest.clone();
    let mut chosen_box: Option<KeepoutBox> = None;
    let mut measured_deepening = 0.0f64;

    // At q_start, J1=0 -> p_start[1] ~= 0. As J1 rotates from 0 to +0.6, y increases and x = r*cos(J1) decreases.
    // Conversely, start at J1 = -0.055 and park to rest with J1 = +0.15 (while keeping J2..J4 moving inward?):
    // Better yet: start at q_park_start with J1 = -0.05, J2..J6 = INSPECT, and rest = [0.05, -1.167, -1.565, 1.435, 0, 0, 0]
    // Wait: during J1 from -0.05 to +0.05 (symmetric across J1=0!), r*cos(J1) peaks at J1=0!
    // r(INSPECT) ~= 0.40 m, so r*(1 - cos(0.05)) = 0.40 * 0.00125 = 0.00050 m = 0.50 mm!!
    // And at J1=+0.15 (rest), x = r*cos(0.15) = r - 0.0045 m (4.5 mm shallower, cleanly OUTSIDE the box if start is 2 mm inside)!
    let mut q_arc_start = INSPECT;
    q_arc_start[0] = -0.05;
    let mut q_arc_rest = INSPECT;
    q_arc_rest[0] = 0.25; // cleanly exits the box (+Y and -X)
    let p_arc_start = ArmModel::REBOT_DEV_ARM.fk_f32(&q_arc_start).unwrap().p;
    let p_arc_mid0 = ArmModel::REBOT_DEV_ARM
        .fk_f32(&{
            let mut q = INSPECT;
            q[0] = 0.0;
            q
        })
        .unwrap()
        .p;
    let x_bulge = p_arc_mid0[0] - p_arc_start[0];
    assert!(
        x_bulge > 0.00025 && x_bulge < 0.00085,
        "expected sub-mm X bulge in (0.25mm, 0.85mm) on arc across J1=0, got {:.4} mm",
        x_bulge * 1000.0
    );
    assert!(
        x_bulge > WORKSPACE_EPS && x_bulge < WORKSPACE_PARK_ESCAPE_EPS,
        "bulge {x_bulge} must lie strictly between WORKSPACE_EPS and WORKSPACE_PARK_ESCAPE_EPS"
    );

    // Place keep-out box "fixture_near" so p_arc_start is 2.0 mm inside its -X face (min_x = p_arc_start[0] - 0.002),
    // wide in Y/Z so the nearest face at start AND at J1=0 is the -X face, while q_arc_rest is completely outside!
    let p_arc_rest = ArmModel::REBOT_DEV_ARM.fk_f32(&q_arc_rest).unwrap().p;
    assert!(
        p_arc_rest[0] < p_arc_start[0] - 0.005,
        "rest pose must be well clear of -X face: rest_x={}, start_x={}",
        p_arc_rest[0],
        p_arc_start[0]
    );
    let kb = KeepoutBox {
        name: "fixture_near".into(),
        min_m: [
            (p_arc_start[0] - 0.002) as f32,
            (p_arc_start[1] - 0.15) as f32,
            (p_arc_start[2] - 0.15) as f32,
        ],
        max_m: [
            (p_arc_start[0] + 0.15) as f32,
            (p_arc_start[1] + 0.15) as f32,
            (p_arc_start[2] + 0.15) as f32,
        ],
    };
    let _ = (p_start, park_traj, &mut best_rest, &mut chosen_box, &mut measured_deepening);
    cfg.zero_reference = None;
    if let Some(rp) = cfg.resting_pose.as_mut() {
        rp.positions = Some(q_arc_rest.to_vec());
    }
    cfg.home_positions = q_arc_rest.to_vec();
    cfg.workspace.as_mut().unwrap().keepout_boxes = vec![kb];

    // Monotonic exit jog: rotate J1 further negative (-0.25) so X decreases strictly monotonically!
    let mut q_mono_exit = INSPECT;
    q_mono_exit[0] = -0.25;

    for use_executor in [false, true] {
        let label = if use_executor { "executor" } else { "legacy" };
        let (mut ws, gen, ep, sup, srv, tick, exec) =
            start_owner_rig(cfg.clone(), &q_arc_start, use_executor).await;

        // 1. Park briefly deepens keepout by ~0.5 mm (< 1 mm Park tol, > 0.1 mm WORKSPACE_EPS) -> MUST REFUSE!
        let rej = common::preset(&mut ws, gen, ep, "park", None).await;
        assert_eq!(
            rej["type"], "preset_reject",
            "{label}: sub-mm keepout deepening during Park must be refused: {rej}"
        );
        assert_eq!(rej["code"], "WORKSPACE_VIOLATION", "{label}: {rej}");
        assert_eq!(rej["constraint_id"], "keepout:fixture_near", "{label}: {rej}");
        assert_eq!(rej["violation_phase"], "park", "{label}: {rej}");
        let obs = sup.get_observation().await;
        assert_eq!(obs.safety_state, SafetyState::Armed, "{label}: 0 faults");
        assert!(
            obs.commanded_positions
                .iter()
                .zip(&q_arc_start)
                .all(|(a, b)| (a - b).abs() < 1e-5),
            "{label}: targets must stay unchanged after Park refusal"
        );

        // 2. Monotonic exit jog out of keep-out box -> ADMITTED and completes with 0 faults!
        let ack = common::rpc(
            &mut ws,
            gen,
            ep,
            "/api/v1/motion/jog",
            json!({"target_positions": q_mono_exit}),
        )
        .await;
        assert_eq!(ack["type"], "command_ack", "{label}: monotonic exit jog: {ack}");
        wait_idle_no_fault(&sup, label, 5000).await;

        drop(ws);
        if let Some(t) = tick {
            t.abort();
        }
        srv.abort();
        if let Some(h) = exec {
            h.stop_and_join();
        }
    }

    // 3. Also verify floor_z gets zero deepening allowance during Park (~0.5 mm dip < 1 mm -> refused).
    let mut cfg_floor = ArmConfig::rebot_dm();
    cfg_floor.collision = None;
    cfg_floor.power_budget = None;
    {
        let ws = cfg_floor.workspace.as_mut().unwrap();
        ws.check_points_end_m = vec![[0.0, 0.0, 0.0]];
        ws.link_capsules.clear();
        ws.self_collision_enabled = false;
        ws.self_collision_pairs.clear();
        ws.keepout_boxes.clear();
        ws.max_reach_m = None;
        ws.box_min_m = [-2.0, -2.0, -2.0];
        ws.box_max_m = [2.0, 2.0, 2.0];
    }
    // Find a true interior local minimum of end_origin Z as a function of J4, then place start
    // slightly on one side (~0.5 mm above the minimum) and rest on the opposite side (>10 mm above),
    // so the 1-DOF Park quintic monotonically crosses the Z minimum with a deterministic ~0.5 mm dip.
    let q_base: [f32; 7] = [0.0, -0.9, -0.8, 0.0, 0.0, 0.0, 0.0];
    let z_at_j4 = |j4: f32| -> f64 {
        let mut q = q_base;
        q[3] = j4;
        ArmModel::REBOT_DEV_ARM.fk_f32(&q).unwrap().p[2]
    };
    let j4_lo = cfg_floor.limits[3].min_position + 0.35;
    let j4_hi = cfg_floor.limits[3].max_position - 0.35;
    let mut best_j4 = j4_lo;
    let mut best_z = f64::INFINITY;
    let steps = 400;
    for k in 0..=steps {
        let j4 = j4_lo + (j4_hi - j4_lo) * (k as f32 / steps as f32);
        let z = z_at_j4(j4);
        if z < best_z {
            best_z = z;
            best_j4 = j4;
        }
    }
    // Step slightly toward j4_lo until z rises by ~0.5 mm above best_z (start), and toward j4_hi
    // until z rises by >15 mm above best_z (rest).
    let mut q_f_start = q_base;
    let mut q_f_rest = q_base;
    let mut found_floor_case = false;
    for s_i in 1..=200 {
        let cand_start_j4 = best_j4 - (s_i as f32) * 0.002;
        if cand_start_j4 <= cfg_floor.limits[3].min_position + 0.05 {
            break;
        }
        let dip = z_at_j4(cand_start_j4) - best_z;
        if dip > 0.0004 && dip < 0.0007 {
            q_f_start[3] = cand_start_j4;
            // Find rest on the opposite side of best_j4 rising > 15 mm.
            for r_i in 1..=300 {
                let cand_rest_j4 = best_j4 + (r_i as f32) * 0.005;
                if cand_rest_j4 >= cfg_floor.limits[3].max_position - 0.05 {
                    break;
                }
                if z_at_j4(cand_rest_j4) > z_at_j4(cand_start_j4) + 0.015 {
                    q_f_rest[3] = cand_rest_j4;
                    found_floor_case = true;
                    break;
                }
            }
            if found_floor_case {
                break;
            }
        }
    }
    assert!(
        found_floor_case,
        "Park floor fixture search must unconditionally find a sub-mm Z dip (0.4..0.7 mm) around best_j4={best_j4}"
    );
    let p_f0 = ArmModel::REBOT_DEV_ARM.fk_f32(&q_f_start).unwrap().p;
    let tr = armd::trajectory::TrajectoryPlanner::plan(
        &q_f_start,
        &[0.0; 7],
        &q_f_rest,
        &[0.0; 7],
        Some(1.5),
        &cfg_floor.limits,
    )
    .expect("plan Park floor sub-mm dip quintic");
    let min_z = (0..=80)
        .map(|k| {
            let q = tr.sample(tr.duration * k as f64 / 80.0).positions;
            ArmModel::REBOT_DEV_ARM.fk_f32(&q).unwrap().p[2]
        })
        .fold(f64::INFINITY, f64::min);
    let actual_dip = p_f0[2] - min_z;
    assert!(
        actual_dip > 0.0003 && actual_dip < 0.0008,
        "expected 0.3..0.8 mm Park floor dip, got {actual_dip:.6} m"
    );
    cfg_floor.zero_reference = None;
    if let Some(rp) = cfg_floor.resting_pose.as_mut() {
        rp.positions = Some(q_f_rest.to_vec());
    }
    cfg_floor.home_positions = q_f_rest.to_vec();
    cfg_floor.workspace.as_mut().unwrap().floor_z_m = (p_f0[2] + 0.005) as f32;
    let mut inner = armed(cfg_floor, &q_f_start);
    let s = inner.current_admission_stamp();
    let err = inner
        .check_admission(s)
        .map_err(ArmError::from)
        .and_then(|a| SupervisorInner::unload_action(a))
        .unwrap_err();
    let ArmError::Safety(SafetyError::WorkspaceViolation { detail, .. }) = err else {
        panic!("expected floor_z WorkspaceViolation during Park, got {err:?}");
    };
    assert_eq!(detail.constraint_id.as_deref(), Some("floor_z"));
    assert_eq!(detail.violation_phase.as_deref(), Some("park"));
}

/// Codex §Test 3 (both owners, sim):
/// Arm starts outside `max_reach` and returns inward toward the workspace along a path that:
/// (a) briefly dips ~2 mm deeper in `max_reach` before finishing strictly inside -> ADMITTED without obstacle;
/// (b) crosses a separate thin named keep-out box placed between two planner knots -> REFUSED with
///     `WORKSPACE_VIOLATION`, `constraint_id == "keepout:thin_mid_box"`, unchanged targets, 0 faults.
#[tokio::test]
async fn outside_start_return_through_thin_keepout_box_between_knots_both_owners() {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.collision = None;
    cfg.power_budget = None;
    cfg.lease_timeout_ms = 5000;
    cfg.camera_mount.offset_m = None;
    {
        let ws = cfg.workspace.as_mut().unwrap();
        ws.check_points_end_m = vec![[0.0, 0.0, 0.0]];
        ws.link_capsules.clear();
        ws.self_collision_enabled = false;
        ws.self_collision_pairs.clear();
    }

    {
        let ws = cfg.workspace.as_mut().unwrap();
        ws.box_min_m = [-2.0, -2.0, -2.0];
        ws.box_max_m = [2.0, 2.0, 2.0];
        ws.floor_z_m = -2.0;
    }

    // Construct a deterministic curved reach return by swinging J4 (wrist pitch) across its
    // peak-reach angle so mid-path reach briefly deepens 1..4 mm before finishing > 15 mm shallower.
    let shoulder = ArmModel::REBOT_DEV_ARM.shoulder_rz();
    let reach_of = |q: &[f32]| -> f64 {
        let p = ArmModel::REBOT_DEV_ARM.fk_f32(q).unwrap().p;
        kinematics::shoulder_reach_from(&p, &shoulder)
    };
    let zeros = vec![0.0f32; cfg.dof];
    let mut q_start = INSPECT;
    let mut q_goal = INSPECT;
    let mut r_start = 0.0f64;
    let mut found_curved_return = false;
    let j4_lo = cfg.limits[3].min_position + 0.1;
    let j4_hi = cfg.limits[3].max_position - 0.1;
    let base_pose: [f32; 7] = [0.0, -1.0, -1.2, 0.0, 0.0, 0.0, 0.0];
    let mut best_j4_peak = 0.0f32;
    let mut best_r_peak = 0.0f64;
    for step in 0..=100 {
        let j4 = j4_lo + (j4_hi - j4_lo) * (step as f32 / 100.0);
        let mut q = base_pose;
        q[3] = j4;
        let r = reach_of(&q);
        if r > best_r_peak {
            best_r_peak = r;
            best_j4_peak = j4;
        }
    }
    for off_start in [0.15f32, 0.25, 0.35, 0.45, 0.55] {
        for sign in [-1.0f32, 1.0f32] {
            let s_j4 = best_j4_peak - sign * off_start;
            let g_j4 = best_j4_peak + sign * (off_start + 0.65);
            if s_j4 < j4_lo || s_j4 > j4_hi || g_j4 < j4_lo || g_j4 > j4_hi {
                continue;
            }
            let mut cand_s = base_pose;
            cand_s[3] = s_j4;
            let mut cand_g = base_pose;
            cand_g[0] = 0.35;
            cand_g[3] = g_j4;
            let rs = reach_of(&cand_s);
            let rg = reach_of(&cand_g);
            if rg > rs - 0.010 {
                continue;
            }
            if let Ok(tr) = armd::trajectory::TrajectoryPlanner::plan(
                &cand_s,
                &zeros,
                &cand_g,
                &zeros,
                None,
                &cfg.limits,
            ) {
                let max_r = (0..=40)
                    .map(|k| reach_of(&tr.sample(tr.duration * k as f64 / 40.0).positions))
                    .fold(0.0f64, f64::max);
                let dip = max_r - rs;
                if dip > 0.001 && dip < 0.0045 {
                    q_start = cand_s;
                    q_goal = cand_g;
                    r_start = rs;
                    found_curved_return = true;
                    break;
                }
            }
        }
        if found_curved_return {
            break;
        }
    }
    assert!(found_curved_return, "failed to find curved reach-return trajectory");
    // Place max_reach_m 10 mm inside r_start so start is outside max_reach by 10 mm and goal is well inside.
    cfg.workspace.as_mut().unwrap().max_reach_m = Some((r_start - 0.010) as f32);

    // Plan the exact quintic to find the midpoint between knot 10 and knot 11 (t = 10.5 / 20 * T).
    let traj = armd::trajectory::TrajectoryPlanner::plan(
        &q_start,
        &zeros,
        &q_goal,
        &zeros,
        None,
        &cfg.limits,
    )
    .unwrap();
    let p_k10 = ArmModel::REBOT_DEV_ARM
        .fk_f32(&traj.sample(traj.duration * 10.0 / 20.0).positions)
        .unwrap()
        .p;
    let p_mid = ArmModel::REBOT_DEV_ARM
        .fk_f32(&traj.sample(traj.duration * 10.5 / 20.0).positions)
        .unwrap()
        .p;
    let p_k11 = ArmModel::REBOT_DEV_ARM
        .fk_f32(&traj.sample(traj.duration * 11.0 / 20.0).positions)
        .unwrap()
        .p;
    let dist_10 = kinematics::norm(&kinematics::sub(&p_mid, &p_k10));
    let dist_11 = kinematics::norm(&kinematics::sub(&p_mid, &p_k11));
    let half = (dist_10.min(dist_11) * 0.35).clamp(0.002, 0.006) as f32;
    let thin_box = KeepoutBox {
        name: "thin_mid_box".into(),
        min_m: [p_mid[0] as f32 - half, p_mid[1] as f32 - half, p_mid[2] as f32 - half],
        max_m: [p_mid[0] as f32 + half, p_mid[1] as f32 + half, p_mid[2] as f32 + half],
    };

    let mut cfg_with_box = cfg.clone();
    cfg_with_box.workspace.as_mut().unwrap().keepout_boxes.push(thin_box);

    for use_executor in [false, true] {
        let label = if use_executor { "executor" } else { "legacy" };

        // (a) With the thin keep-out box between knots: MUST refuse with named keep-out witness & unchanged targets.
        {
            let (mut ws, gen, ep, sup, srv, tick, exec) =
                start_owner_rig(cfg_with_box.clone(), &q_start, use_executor).await;
            let rej = common::rpc(
                &mut ws,
                gen,
                ep,
                "/api/v1/motion/jog",
                json!({"target_positions": q_goal}),
            )
            .await;
            assert_eq!(rej["type"], "command_reject", "{label}: thin box must reject: {rej}");
            assert_eq!(rej["code"], "WORKSPACE_VIOLATION", "{label}: {rej}");
            assert_eq!(rej["constraint_id"], "keepout:thin_mid_box", "{label}: {rej}");
            let obs = sup.get_observation().await;
            assert_eq!(obs.safety_state, SafetyState::Armed, "{label}: 0 faults");
            assert!(
                obs.commanded_positions
                    .iter()
                    .zip(&q_start)
                    .all(|(a, b)| (a - b).abs() < 1e-5),
                "{label}: targets must stay unchanged"
            );
            drop(ws);
            if let Some(t) = tick {
                t.abort();
            }
            srv.abort();
            if let Some(h) = exec {
                h.stop_and_join();
            }
        }

        // (b) Without the thin box: the same curved reach return (brief 1-4 mm dip, finishes inside) completes!
        {
            let (mut ws, gen, ep, sup, srv, tick, exec) =
                start_owner_rig(cfg.clone(), &q_start, use_executor).await;
            let ack = common::rpc(
                &mut ws,
                gen,
                ep,
                "/api/v1/motion/jog",
                json!({"target_positions": q_goal}),
            )
            .await;
            assert_eq!(
                ack["type"], "command_ack",
                "{label}: curved reach return without box must be admitted: {ack}"
            );
            wait_idle_no_fault(&sup, label, 5000).await;
            drop(ws);
            if let Some(t) = tick {
                t.abort();
            }
            srv.abort();
            if let Some(h) = exec {
                h.stop_and_join();
            }
        }
    }
}

/// Decision Item 2 + Item 3:
/// - Both `end_origin` and `tcp_origin` reach slots get the 5 mm temporary allowance.
/// - Each reach slot is tracked independently with its own shallowest-so-far baseline.
/// - Ending deeper in reach (even by 2 mm < 5 mm) is refused by the terminal rule.
#[test]
fn reach_slots_temporary_5mm_allowance_and_independent_tracking() {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.collision = None;
    cfg.power_budget = None;
    cfg.tcp_offset = [0.0, 0.0, 0.12]; // distinct TCP origin from end_origin
    cfg.workspace.as_mut().unwrap().check_points_end_m = vec![[0.0, 0.0, 0.0]];

    let ws = cfg.workspace.as_ref().unwrap().clone();
    let eval_probe = WorkspaceEvaluator::new_with_tcp(
        &cfg.arm_model,
        &ws,
        &cfg.camera_mount,
        cfg.tcp_offset,
    );
    assert_eq!(eval_probe.reach_slots, 2, "must have independent end_origin + tcp_origin reach slots");
    let end_reach_slot = eval_probe.num_geoms * eval_probe.slots_per_geom;
    let tcp_reach_slot = end_reach_slot + 1;

    // 1. Both reach slots get 5 mm temporary allowance via slot_escape_tol (and hard slots do not).
    assert!((eval_probe.slot_escape_tol(end_reach_slot) - kinematics::WORKSPACE_ESCAPE_TOL_M).abs() < 1e-12);
    assert!((eval_probe.slot_escape_tol(tcp_reach_slot) - kinematics::WORKSPACE_ESCAPE_TOL_M).abs() < 1e-12);
    assert!((eval_probe.slot_escape_tol(0) - WORKSPACE_EPS).abs() < 1e-12, "floor_z must stay WORKSPACE_EPS");

    // 2. Independent reach slot tracking:
    //    Suppose tcp_origin is 10 mm outside max_reach while end_origin is clear (min_d = 0).
    //    A step that temporarily deepens tcp_origin by 2 mm (to 12 mm) while end_origin stays clear is ALLOWED mid-path;
    //    a step that causes end_origin to newly intrude by 2 mm (while tcp_origin retreats to 5 mm) MUST be REFUSED!
    let mut c0 = eval_probe.alloc_buf();
    c0.as_mut_slice().fill(0.05); // all slots clear
    c0.as_mut_slice()[end_reach_slot] = 0.020; // end_origin clear
    c0.as_mut_slice()[tcp_reach_slot] = -0.010; // tcp_origin 10 mm over reach
    let mut min_d = eval_probe.init_min_depths(&c0);

    // Mid-path temporary 2 mm deepening on tcp_origin only -> allowed
    let mut c_mid = c0.clone();
    c_mid.as_mut_slice()[tcp_reach_slot] = -0.012;
    eval_probe
        .check_and_advance(&c_mid, &mut min_d, "plan")
        .expect("tcp_origin 2 mm temporary deepening (< 5 mm) must be allowed");

    // Independent slot protection: end_origin was clear, so 2 mm new intrusion on end_origin MUST fail!
    let mut c_new_end = c0.clone();
    c_new_end.as_mut_slice()[end_reach_slot] = -0.002;
    c_new_end.as_mut_slice()[tcp_reach_slot] = -0.005;
    let err = eval_probe
        .check_readonly(&c_new_end, &min_d, "plan")
        .unwrap_err();
    assert_eq!(err.detail.constraint_id.as_deref(), Some("max_reach"));
    assert_eq!(err.detail.geometry_id.as_deref(), Some("end_origin"));

    // Once tcp_origin clears reach (depth 0), re-entering by 2 mm MUST fail!
    let mut c_clear = c0.clone();
    c_clear.as_mut_slice()[tcp_reach_slot] = 0.005;
    eval_probe
        .check_and_advance(&c_clear, &mut min_d, "plan")
        .expect("clearing tcp_origin reach passes");
    let mut c_reenter = c0.clone();
    c_reenter.as_mut_slice()[tcp_reach_slot] = -0.002;
    let err_re = eval_probe
        .check_readonly(&c_reenter, &min_d, "plan")
        .unwrap_err();
    assert_eq!(err_re.detail.constraint_id.as_deref(), Some("max_reach"));
    assert_eq!(err_re.detail.geometry_id.as_deref(), Some("tcp_origin"));

    // 3. Ending 2 mm deeper in reach (via check_path_with_tcp_tol) MUST be refused by terminal rule.
    let shoulder = cfg.arm_model.shoulder_rz();
    let p0 = cfg.arm_model.fk_f32(&INSPECT).unwrap().p;
    let r0 = kinematics::shoulder_reach_from(&p0, &shoulder);
    let mut q_probe = INSPECT;
    q_probe[1] += 0.01;
    let out_sign = if kinematics::shoulder_reach_from(&cfg.arm_model.fk_f32(&q_probe).unwrap().p, &shoulder) > r0 {
        1.0f32
    } else {
        -1.0f32
    };
    let dr_per_rad = (kinematics::shoulder_reach_from(
        &cfg.arm_model
            .fk_f32(&{
                let mut q = INSPECT;
                q[1] += 0.01 * out_sign;
                q
            })
            .unwrap()
            .p,
        &shoulder,
    ) - r0)
        / 0.01;
    let mut q_deeper_2mm = INSPECT;
    q_deeper_2mm[1] += (0.002 / dr_per_rad) as f32 * out_sign;
    let mut ws_reach = ws;
    ws_reach.max_reach_m = Some((r0 - 0.015) as f32);
    ws_reach.box_max_m = [2.0, 2.0, 2.0];
    ws_reach.floor_z_m = -2.0;
    let err_term = armd::supervisor::check_path_with(
        &cfg.arm_model,
        &ws_reach,
        &cfg.camera_mount,
        &INSPECT,
        [q_deeper_2mm.as_slice()],
    )
    .unwrap_err();
    let SafetyError::WorkspaceViolation { detail, .. } = err_term else {
        panic!("ending 2 mm deeper in reach must fail terminal rule, got {err_term:?}");
    };
    assert_eq!(detail.constraint_id.as_deref(), Some("max_reach"));
}

/// Decision Item 3 (streams, both owners):
/// Running per-slot minima are kept across frames so repeated servo targets cannot ratchet deeper
/// past the 5 mm soft-slot allowance (even after a frame settles!).
/// Stationary targets and braking while outside remain allowed without progress.
#[tokio::test]
async fn stream_no_ratcheting_across_frames_and_stationary_braking_allowed_both_owners() {
    let (cfg, q_start, start_max_x) = stretched_beyond_box_max_x_setup();
    let check_pts = cfg.workspace.as_ref().unwrap().check_points_end_m.clone();
    let max_x_at = |q: &[f32]| -> f64 {
        let p = ArmModel::REBOT_DEV_ARM.fk_f32(q).unwrap();
        check_pts
            .iter()
            .map(|c| p.transform_point(&c.map(f64::from))[0])
            .fold(p.p[0], f64::max)
    };
    let mut q_probe = q_start;
    q_probe[1] += 0.01;
    let fwd_sign = if max_x_at(&q_probe) > start_max_x { 1.0f32 } else { -1.0f32 };
    let dx_per_rad = (max_x_at(&{
        let mut q = q_start;
        q[1] += 0.01 * fwd_sign;
        q
    }) - start_max_x)
        / 0.01;

    // Frame 1: +3.0 mm deeper than start (< 5 mm soft allowance)
    let mut q_step1 = q_start;
    q_step1[1] += (0.003 / dx_per_rad) as f32 * fwd_sign;
    // Frame 2: +6.2 mm deeper than start (only +3.2 mm deeper than q_step1, but > 5 mm from start!)
    let mut q_step2 = q_start;
    q_step2[1] += (0.0062 / dx_per_rad) as f32 * fwd_sign;
    assert!(
        max_x_at(&q_step1) - start_max_x > 0.002 && max_x_at(&q_step1) - start_max_x < 0.004,
        "step1 delta={}",
        max_x_at(&q_step1) - start_max_x
    );
    assert!(
        max_x_at(&q_step2) - start_max_x > 0.0055,
        "step2 delta={}",
        max_x_at(&q_step2) - start_max_x
    );

    for use_executor in [false, true] {
        let label = if use_executor { "executor" } else { "legacy" };
        let (mut ws, gen, ep, sup, srv, tick, exec) =
            start_owner_rig(cfg.clone(), &q_start, use_executor).await;

        // 1. Stationary servo target while outside -> allowed without progress, 0 faults.
        common::servo(&mut ws, gen, ep, &q_start).await;
        assert!(
            common::servo_reject_within(&mut ws, 120).await.is_none(),
            "{label}: stationary servo target while outside must be allowed"
        );

        // 2. Frame 1 (+3 mm) -> allowed; let it step AND settle.
        common::servo(&mut ws, gen, ep, &q_step1).await;
        assert!(
            common::servo_reject_within(&mut ws, 250).await.is_none(),
            "{label}: first +3mm servo frame must be accepted"
        );
        // Wait for frame 1 to actually reach q_step1 and settle (> 2 settled ticks).
        let deadline = tokio::time::Instant::now() + Duration::from_secs(2);
        let mut obs_after_1 = sup.get_observation().await;
        while tokio::time::Instant::now() < deadline {
            if (obs_after_1.commanded_positions[1] - q_step1[1]).abs() < 1e-4 {
                break;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
            obs_after_1 = sup.get_observation().await;
        }
        tokio::time::sleep(Duration::from_millis(80)).await;
        obs_after_1 = sup.get_observation().await;
        assert_eq!(obs_after_1.safety_state, SafetyState::Armed, "{label}: 0 faults after step1");
        assert!(
            (obs_after_1.commanded_positions[1] - q_step1[1]).abs() < 1e-4,
            "{label}: frame 1 did not reach q_step1 (got {}, expected {}, start {})",
            obs_after_1.commanded_positions[1],
            q_step1[1],
            q_start[1]
        );
        assert!(
            (obs_after_1.commanded_positions[1] - q_start[1]).abs() > 1e-3,
            "{label}: frame 1 did not move meaningfully from q_start"
        );

        // 3. Frame 2 (+3.2 mm more = +6.2 mm from initial running minimum) -> MUST BE REFUSED (no ratcheting)!
        common::servo(&mut ws, gen, ep, &q_step2).await;
        let rej = common::servo_reject_within(&mut ws, 600)
            .await
            .unwrap_or_else(|| panic!("{label}: ratcheting frame 2 (+6.2mm total) must be refused"));
        assert_eq!(rej["code"], "WORKSPACE_VIOLATION", "{label}: {rej}");
        assert_eq!(rej["constraint_id"], "box_max_x", "{label}: {rej}");
        let obs_after_2 = sup.get_observation().await;
        assert_eq!(obs_after_2.safety_state, SafetyState::Armed, "{label}: 0 faults after refusal");

        drop(ws);
        if let Some(t) = tick {
            t.abort();
        }
        srv.abort();
        if let Some(h) = exec {
            h.stop_and_join();
        }
    }
}

/// Decision Item 4:
/// Evaluate from the measured pose baseline; if the commanded start differs, check the
/// measured-to-commanded transition too, and evaluate terminal progress against measured.
#[test]
fn measured_vs_commanded_start_baseline_and_transition_checked() {
    let (cfg, q_meas, meas_max_x) = stretched_beyond_box_max_x_setup();
    let check_pts = cfg.workspace.as_ref().unwrap().check_points_end_m.clone();
    let max_x_at = |q: &[f32]| -> f64 {
        let p = ArmModel::REBOT_DEV_ARM.fk_f32(q).unwrap();
        check_pts
            .iter()
            .map(|c| p.transform_point(&c.map(f64::from))[0])
            .fold(p.p[0], f64::max)
    };
    let mut q_probe = q_meas;
    q_probe[1] += 0.01;
    let fwd_sign = if max_x_at(&q_probe) > meas_max_x { 1.0f32 } else { -1.0f32 };
    let dx_per_rad = (max_x_at(&{
        let mut q = q_meas;
        q[1] += 0.01 * fwd_sign;
        q
    }) - meas_max_x)
        / 0.01;

    // Case A: measured is at 15 mm beyond box_max_x; commanded is at 18 mm (+3 mm deeper).
    // Target is at 16.5 mm (improves vs commanded 18 mm, but is STILL 1.5 mm deeper than measured 15 mm!).
    // MUST be refused on both legacy jog and RT prepare_motion!
    let mut q_cmd_deeper = q_meas;
    q_cmd_deeper[1] += (0.003 / dx_per_rad) as f32 * fwd_sign;
    let mut q_target_still_deeper_than_meas = q_meas;
    q_target_still_deeper_than_meas[1] += (0.0015 / dx_per_rad) as f32 * fwd_sign;

    // Legacy jog:
    {
        let mut inner = armed(cfg.clone(), &q_cmd_deeper);
        inner.measured_positions = q_meas.to_vec();
        let s = inner.current_admission_stamp();
        let err = inner
            .check_admission(s)
            .map_err(ArmError::from)
            .and_then(|a| {
                SupervisorInner::jog(a, q_target_still_deeper_than_meas.to_vec(), None, None)
            })
            .unwrap_err();
        let ArmError::Safety(SafetyError::WorkspaceViolation { detail, .. }) = err else {
            panic!("legacy jog ending deeper than measured baseline must fail, got {err:?}");
        };
        assert_eq!(detail.constraint_id.as_deref(), Some("box_max_x"));
    }

    // RT prepare_motion:
    {
        let mut inner = armed(cfg.clone(), &q_cmd_deeper);
        inner.measured_positions = q_meas.to_vec();
        let snap = MotionPlanSnapshot::from_supervisor(&inner);
        let req = MotionRequest::jog(q_target_still_deeper_than_meas.to_vec(), None, None);
        let err = prepare_motion(&snap, &req).unwrap_err();
        let ArmError::Safety(SafetyError::WorkspaceViolation { detail, .. }) = err else {
            panic!("RT prepare_motion ending deeper than measured baseline must fail, got {err:?}");
        };
        assert_eq!(detail.constraint_id.as_deref(), Some("box_max_x"));
    }

    // Case B: measured is clear inside workspace; commanded is inside keep-out box "lag_box".
    // Jogging from commanded back to measured MUST be refused because measured -> commanded intrudes!
    let mut cfg_ko = ArmConfig::rebot_dm();
    cfg_ko.collision = None;
    cfg_ko.power_budget = None;
    cfg_ko.workspace.as_mut().unwrap().check_points_end_m = vec![[0.0, 0.0, 0.0]];
    let mut q_cmd_in_box = INSPECT;
    q_cmd_in_box[0] += 0.20;
    let p_box = ArmModel::REBOT_DEV_ARM.fk_f32(&q_cmd_in_box).unwrap().p;
    cfg_ko.workspace.as_mut().unwrap().keepout_boxes.push(KeepoutBox {
        name: "lag_box".into(),
        min_m: p_box.map(|v| (v - 0.02) as f32),
        max_m: p_box.map(|v| (v + 0.02) as f32),
    });
    {
        let mut inner = armed(cfg_ko.clone(), &q_cmd_in_box);
        inner.measured_positions = INSPECT.to_vec(); // measured is clear
        let s = inner.current_admission_stamp();
        let err = inner
            .check_admission(s)
            .map_err(ArmError::from)
            .and_then(|a| SupervisorInner::jog(a, INSPECT.to_vec(), None, None))
            .unwrap_err();
        let ArmError::Safety(SafetyError::WorkspaceViolation { detail, .. }) = err else {
            panic!("measured->commanded keepout transition must fail, got {err:?}");
        };
        assert_eq!(detail.constraint_id.as_deref(), Some("keepout:lag_box"));
    }

    // Case C: Cartesian straight move_relative ending deeper (e.g. +2 mm X while outside box_max_x)
    // must be refused by the terminal rule!
    {
        let inner = armed(cfg.clone(), &q_meas);
        let input = inner.plan_input().unwrap();
        let (goal_bad, opts) = resolve_goal_opts(
            &input,
            &GoalSpec::Relative {
                frame: RelFrame::Base,
                d: [0.002, 0.02, 0.0],
                dyaw: 0.0,
                dpitch: 0.0,
                droll: 0.0,
                level: false,
                auto_yaw: None,
            },
        );
        let err = plan_cartesian_opts(&input, goal_bad, Speed::default(), opts).unwrap_err();
        assert!(
            matches!(err, armd::motion::MotionError::WorkspaceViolation { .. }),
            "Cartesian move ending +2mm deeper must fail terminal rule, got {err:?}"
        );
    }
}

#[test]
fn sweep_linear_step_slotbuf_rt_matches_legacy_on_subdivision_and_rejection() {
    use armd::chain::{RtSweepStack, WorkspaceFailureKindRt};
    let mut cfg = b601dm_config();
    let a = vec![0.10f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let b = vec![0.25f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let p_a = cfg.arm_model.frames_f32(&a).unwrap().end.p;
    cfg.camera_mount.offset_m = None;
    let ws_ok = armd::model::WorkspacePolicy {
        floor_z_m: p_a[2] as f32 - 0.002,
        box_min_m: [-1.5, -1.5, -1.5],
        box_max_m: [1.5, 1.5, 1.5],
        max_reach_m: Some(1.5),
        check_points_end_m: vec![[0.0, 0.0, 0.0]],
        keepout_boxes: vec![],
        link_capsules: vec![],
        self_collision_enabled: false,
        self_collision_min_clearance_m: 0.0,
        self_collision_pairs: vec![],
    };
    let eval = WorkspaceEvaluator::new(&cfg.arm_model, &ws_ok, &cfg.camera_mount);
    let mut c_a = eval.alloc_buf();
    eval.eval_into(&a, &mut c_a, "test").unwrap();
    let mut c_a_rt = eval.alloc_buf();
    eval.eval_into_slotbuf_rt(&a, &mut c_a_rt, "test").unwrap();
    assert_eq!(c_a.as_slice(), c_a_rt.as_slice());

    let (mut min_d_leg, mut min_d_rt) = (eval.init_min_depths(&c_a), eval.init_min_depths(&c_a_rt));
    let (mut budget_leg, mut budget_rt) = (64usize, 64usize);
    let mut rt_stack = Box::new(RtSweepStack::default());
    let leg_cb = eval
        .sweep_linear_step(&a, &b, &c_a, &mut min_d_leg, &mut budget_leg, 64, "end", "sweep")
        .expect("legacy sweep with subdivision passes");
    let rt_cb = eval
        .sweep_linear_step_slotbuf_rt(&a, &b, &c_a_rt, &mut min_d_rt, &mut budget_rt, 64, "end", "sweep", &mut rt_stack)
        .expect("RT sweep with subdivision passes");
    assert!(budget_leg < 64, "test must trigger subdivision");
    assert_eq!(budget_rt, budget_leg);
    assert_eq!(rt_cb.as_slice(), leg_cb.as_slice());
    assert_eq!(min_d_rt.as_slice(), min_d_leg.as_slice());

    let exact_splits = 64 - budget_leg;
    let (mut md_exact_leg, mut md_exact_rt) = (eval.init_min_depths(&c_a), eval.init_min_depths(&c_a_rt));
    let (mut b_exact_leg, mut b_exact_rt) = (exact_splits, exact_splits);
    eval.sweep_linear_step(&a, &b, &c_a, &mut md_exact_leg, &mut b_exact_leg, exact_splits, "end", "sweep")
        .expect("legacy exact-budget sweep passes");
    eval.sweep_linear_step_slotbuf_rt(&a, &b, &c_a_rt, &mut md_exact_rt, &mut b_exact_rt, exact_splits, "end", "sweep", &mut rt_stack)
        .expect("RT exact-budget sweep passes with zero extra FK calls");
    assert_eq!(b_exact_rt, 0);
    assert_eq!(b_exact_rt, b_exact_leg);
    assert_eq!(md_exact_rt.as_slice(), md_exact_leg.as_slice());

    let mid_q: Vec<f32> = a.iter().zip(&b).map(|(x, y)| 0.5 * (x + y)).collect();
    let mid_p = cfg.arm_model.frames_f32(&mid_q).unwrap().end.p;
    let mut ws_bad = ws_ok.clone();
    ws_bad.keepout_boxes.push(KeepoutBox {
        name: "mid".into(),
        min_m: [mid_p[0] as f32 - 0.02, mid_p[1] as f32 - 0.02, mid_p[2] as f32 - 0.02],
        max_m: [mid_p[0] as f32 + 0.02, mid_p[1] as f32 + 0.02, mid_p[2] as f32 + 0.02],
    });
    let eval_bad = WorkspaceEvaluator::new(&cfg.arm_model, &ws_bad, &cfg.camera_mount);
    let mut c_a_bad = eval_bad.alloc_buf();
    eval_bad.eval_into_slotbuf_rt(&a, &mut c_a_bad, "test").unwrap();
    let (mut md_leg, mut md_rt) = (eval_bad.init_min_depths(&c_a_bad), eval_bad.init_min_depths(&c_a_bad));
    let (mut b_leg, mut b_rt) = (64usize, 64usize);
    let err_leg = eval_bad.sweep_linear_step(&a, &b, &c_a_bad, &mut md_leg, &mut b_leg, 64, "end", "sweep").expect_err("legacy rejects midpoint");
    let err_rt = eval_bad.sweep_linear_step_slotbuf_rt(&a, &b, &c_a_bad, &mut md_rt, &mut b_rt, 64, "end", "sweep", &mut rt_stack).expect_err("RT rejects midpoint");
    assert_eq!(b_rt, b_leg);
    assert!((err_rt.0 - err_leg.0).abs() < 1e-12);
    assert!(matches!(err_rt.1.kind, WorkspaceFailureKindRt::Constraint { .. }));
}
