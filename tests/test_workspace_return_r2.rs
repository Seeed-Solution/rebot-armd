//! Task F-ws R2 regression suite (Codex R1 findings 1B / 4M / 2m).

mod common;

use std::sync::{Arc, Mutex};
use std::time::Duration;

use armd::chain::{ArmModel, WorkspaceEvaluator};
use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
use armd::kinematics::WORKSPACE_EPS;
use armd::model::{ArmConfig, JointState};
use armd::rt_core::executor::ExecutorConfig;
use armd::safety::SafetyState;
use armd::tracking::{JointTrackingConfig, TrackingStrategy};
use armd::ArmSupervisor;
use common::INSPECT;

#[derive(Clone)]
struct FixedPoseDriver {
    q: Arc<Mutex<Vec<f32>>>,
    enabled: Arc<Mutex<EnableState>>,
    written: Arc<Mutex<Vec<Vec<JointTarget>>>>,
}

impl FixedPoseDriver {
    fn new(q0: &[f32]) -> Self {
        Self {
            q: Arc::new(Mutex::new(q0.to_vec())),
            enabled: Arc::new(Mutex::new(EnableState::Enabled)),
            written: Arc::new(Mutex::new(Vec::new())),
        }
    }
    fn written_frames(&self) -> Vec<Vec<JointTarget>> {
        self.written.lock().unwrap().clone()
    }
}

impl ArmDriver for FixedPoseDriver {
    fn supports_rt_executor(&self) -> Result<(), &'static str> {
        Ok(())
    }
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
        *self.enabled.lock().unwrap()
    }
    fn is_feedback_valid(&self) -> bool {
        true
    }
    fn is_enabled(&self) -> bool {
        *self.enabled.lock().unwrap() == EnableState::Enabled
    }
    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        vec![Some(Duration::ZERO); self.q.lock().unwrap().len()]
    }
    fn fresh_joint_positions_into(&self, joints: &[usize], _max_age: Duration, out: &mut [f32]) -> bool {
        let q = self.q.lock().unwrap();
        if out.len() < joints.len() {
            return false;
        }
        for (dst, &j) in out.iter_mut().zip(joints) {
            let Some(&v) = q.get(j) else { return false };
            *dst = v;
        }
        true
    }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let q = self.q.lock().unwrap();
        Ok(q.iter()
            .map(|&position| JointState { position, velocity: 0.0, torque: 0.0, temperature: 30.0 })
            .collect())
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.written.lock().unwrap().push(targets.to_vec());
        Ok(())
    }
    fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
        *self.enabled.lock().unwrap() = if enable { EnableState::Enabled } else { EnableState::Disabled };
        Ok(())
    }
}

/// R2 Item 1 (Blocker): Park tracking ramp-out (`pending_normal_unload` -> `keepout_only`)
/// must enforce `floor_z` and `self_collision` hard slots even with NO named keep-out boxes.
#[tokio::test]
async fn park_ramp_out_deepening_floor_with_no_keepout_boxes_refused_both_owners() {
    let q0 = INSPECT;
    let p0 = ArmModel::REBOT_DEV_ARM.fk_f32(&q0).unwrap().p;
    let mut q_probe = q0;
    q_probe[1] += 0.02;
    let down_sign = if ArmModel::REBOT_DEV_ARM.fk_f32(&q_probe).unwrap().p[2] < p0[2] {
        1.0f32
    } else {
        -1.0f32
    };
    let mut q_deeper = q0;
    q_deeper[1] += down_sign * 0.025;
    let p_deeper = ArmModel::REBOT_DEV_ARM.fk_f32(&q_deeper).unwrap().p;
    assert!(p_deeper[2] < p0[2] - 0.003, "setup must move end_origin >3 mm downward");

    // 1. Both owners (Legacy and Executor): Park with residual tracking lead and NO named keep-out
    //    boxes where ramping toward nominal target deepens a floor_z intrusion -> refused/held.
    for use_executor in [false, true] {
        let label = if use_executor { "executor" } else { "legacy" };
        let mut cfg = ArmConfig::rebot_dm();
        cfg.collision = None;
        cfg.power_budget = None;
        cfg.torque_guard = None;
        cfg.lease_timeout_ms = 60_000;
        let mut jt = JointTrackingConfig::rebot_dm();
        jt.enabled = true;
        for j in &mut jt.joints {
            j.strategy = TrackingStrategy::Position;
        }
        jt.joints[1].strategy = TrackingStrategy::PositionLead;
        jt.joints[1].lag_s = 0.68;
        jt.joints[1].max_lead_rad = 0.03;
        cfg.joint_tracking = Some(jt);
        let ws = cfg.workspace.as_mut().unwrap();
        ws.keepout_boxes.clear();
        ws.link_capsules.clear();
        ws.self_collision_enabled = false;
        ws.self_collision_pairs.clear();
        ws.check_points_end_m = vec![[0.0, 0.0, 0.0]];
        ws.floor_z_m = (p0[2] + 0.005) as f32;

        let drv = FixedPoseDriver::new(&q0);
        let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv.clone())));
        {
            let mut inner = sup.inner().write().await;
            inner.lease_enabled = false;
            let s = inner.current_admission_stamp();
            inner
                .check_admission(s)
                .and_then(|a| armd::SupervisorInner::enable_torque(a))
                .unwrap();
            inner.measured_positions = q0.to_vec();
            // Residual lead at q0 while nominal Park commanded_positions sits deeper below floor_z.
            inner.tracking.record_issued(q0.to_vec(), vec![0.0; 7]);
            inner.commanded_positions = q_deeper.to_vec();
            inner.target_positions = q_deeper.to_vec();
            inner.pending_normal_unload = true;
        }

        let exec = if use_executor {
            Some(
                sup.start_executor(ExecutorConfig::for_tests(Duration::from_millis(20)))
                    .await
                    .expect("start_executor"),
            )
        } else {
            None
        };
        if use_executor {
            tokio::time::sleep(Duration::from_millis(120)).await;
        } else {
            for _ in 0..5 {
                sup.tick(0.02).await;
            }
        }
        let obs = sup.get_observation().await;
        if let Some(h) = exec {
            h.stop_and_join();
        }

        assert!(
            matches!(obs.safety_state, SafetyState::Fault(_)),
            "{label}: Park ramp-out deepening floor_z with no keepout boxes must refuse/fault, got {:?}",
            obs.safety_state
        );
        for (idx, frame) in drv.written_frames().iter().enumerate() {
            let qf: Vec<f32> = frame.iter().map(|t| t.position).collect();
            let z = ArmModel::REBOT_DEV_ARM.fk_f32(&qf).unwrap().p[2];
            assert!(
                z >= p0[2] - WORKSPACE_EPS - 1e-5,
                "{label}: emitted frame {idx} deepened floor intrusion (z={z:.6} < start {:.6})",
                p0[2]
            );
        }
    }

    // 2. Unit check: `WorkspaceEvaluator::new_with_tcp_in_mode(..., keepout_only = true)`
    //    must keep both `floor_z` AND `self_collision` hard slots active even when `keepout_boxes` is empty.
    let mut cfg_sc = ArmConfig::rebot_dm();
    let ws_sc = cfg_sc.workspace.as_mut().unwrap();
    ws_sc.keepout_boxes.clear();
    ws_sc.floor_z_m = (p0[2] + 0.010) as f32;
    let mut bufs = armd::chain::EvalBufs::default();
    bufs.prepare_for(&cfg_sc.arm_model, ws_sc, &cfg_sc.camera_mount, cfg_sc.tcp_offset);
    let eval_ko = WorkspaceEvaluator::new_with_tcp_in_mode(
        &cfg_sc.arm_model,
        ws_sc,
        &cfg_sc.camera_mount,
        cfg_sc.tcp_offset,
        &mut bufs,
        true,
    );
    let mut c0 = eval_ko.alloc_buf();
    eval_ko.eval_into(&q0, &mut c0, "park").unwrap();
    assert!(
        c0.as_slice()[0] < -0.005,
        "keepout_only mode must keep floor_z active (clr={})",
        c0.as_slice()[0]
    );
    assert!(
        eval_ko.total_slots > eval_ko.self_collision_slot_start,
        "keepout_only mode must keep self_collision slots active"
    );
    eval_ko.recycle(&mut bufs);
}

/// R2 Item 2 (Major): Prepared RT Home/Jog/Park plans and both linear Cartesian installers
/// (Legacy + RT Slot) must fence the certified `measured_positions` baseline at install so
/// inward feedback drift between prepare and install returns `PLAN_STALE` instead of installing
/// a motion whose endpoint is deeper than the live measured pose.
#[test]
fn prepared_plans_fence_measured_drift_before_install() {
    use armd::rt_core::broker::plan_slots;
    use armd::rt_core::ops::{PlanSlot, Primitive};
    use armd::rt_core::requests::actions::{
        install_move_cartesian, prepare_move_relative, ActionSnapshot, ActionStamp,
        CartesianInstallPayload, OwnerCheckCtx, PreparedMoveAction, StagedAction,
    };
    use armd::rt_core::requests::motion::{
        install_home, install_jog, install_park, prepare_motion, MotionInstallError,
        MotionPlanSnapshot, MotionRequest,
    };
    use serde_json::json;

    // Start stretched 10 mm outside box_max_x at q_start; then feedback drifts inward by ~5 mm
    // while commanded_positions stays fixed at q_start.
    let mut cfg = ArmConfig::rebot_dm();
    cfg.collision = None;
    cfg.power_budget = None;
    cfg.lease_timeout_ms = 60_000;
    let q_start: [f32; 7] = [0.0, -0.90, -1.15, 0.85, 0.0, 0.0, 0.0];
    let ws = cfg.workspace.as_mut().unwrap();
    ws.max_reach_m = None;
    ws.keepout_boxes.clear();
    let check_pts = ws.check_points_end_m.clone();
    let max_x_at = |q: &[f32]| -> f64 {
        let pose = ArmModel::REBOT_DEV_ARM.fk_f32(q).unwrap();
        check_pts.iter().map(|c| pose.transform_point(&c.map(f64::from))[0]).fold(pose.p[0], f64::max)
    };
    let start_max_x = max_x_at(&q_start);
    ws.box_max_m[0] = (start_max_x - 0.010) as f32;

    let mut q_probe = q_start;
    q_probe[1] += 0.01;
    let inward_sign = if max_x_at(&q_probe) < start_max_x { 1.0f32 } else { -1.0f32 };
    let dx_per_rad = (start_max_x - {
        let mut q = q_start;
        q[1] += 0.01 * inward_sign;
        max_x_at(&q)
    }) / 0.01;
    // Target moves 2 mm inward from q_start (valid from q_start, but 3 mm DEEPER than q_meas_inward!)
    let mut q_jog_2mm = q_start;
    q_jog_2mm[1] += (0.002 / dx_per_rad) as f32 * inward_sign;
    // Feedback drifts 5 mm inward between prepare and install while commanded stays at q_start.
    let mut q_meas_inward = q_start;
    q_meas_inward[1] += (0.005 / dx_per_rad) as f32 * inward_sign;
    assert!(max_x_at(&q_meas_inward) < max_x_at(&q_jog_2mm) - 0.002);

    let make_armed = || {
        let mut inner = armd::SupervisorInner::new(cfg.clone());
        inner.lease_enabled = false;
        let s = inner.current_admission_stamp();
        inner.check_admission(s).and_then(|a| armd::SupervisorInner::enable_torque(a)).unwrap();
        inner.commanded_positions = q_start.to_vec();
        inner.measured_positions = q_start.to_vec();
        inner.target_positions = q_start.to_vec();
        inner
    };

    // 1. RT Jog: prepared at q_start (2 mm inward), feedback drifts 5 mm inward -> PLAN_STALE!
    {
        let mut inner = make_armed();
        let snap = MotionPlanSnapshot::from_supervisor(&inner);
        let plan = prepare_motion(&snap, &MotionRequest::jog(q_jog_2mm.to_vec(), Some(0.5), None))
            .expect("jog prepares cleanly before drift");
        inner.measured_positions = q_meas_inward.to_vec();
        let s = inner.current_admission_stamp();
        let err = install_jog(&mut inner, s, &plan.slot, plan.heap_traj, None, false)
            .expect_err("install_jog must reject after measured_positions drifted inward");
        assert_eq!(err, MotionInstallError::PlanStale, "expected PlanStale on jog, got {err:?}");
    }

    // 2. RT Home: prepared at q_start, feedback drifts inward -> PLAN_STALE!
    {
        let mut inner = make_armed();
        let snap = MotionPlanSnapshot::from_supervisor(&inner);
        let plan = prepare_motion(&snap, &MotionRequest::home(None))
            .expect("home prepares cleanly before drift");
        inner.measured_positions = q_meas_inward.to_vec();
        let s = inner.current_admission_stamp();
        let err = install_home(&mut inner, s, &plan.slot, plan.heap_traj, None, false)
            .expect_err("install_home must reject after measured_positions drifted inward");
        assert_eq!(err, MotionInstallError::PlanStale, "expected PlanStale on home, got {err:?}");
    }

    // 3. RT Park: prepared at q_start, feedback drifts inward -> PLAN_STALE!
    {
        let mut inner = make_armed();
        let snap = MotionPlanSnapshot::from_supervisor(&inner);
        let plan = prepare_motion(&snap, &MotionRequest::park())
            .expect("park prepares cleanly before drift");
        inner.measured_positions = q_meas_inward.to_vec();
        let s = inner.current_admission_stamp();
        let err = install_park(&mut inner, s, &plan.slot, plan.heap_traj, None, false)
            .expect_err("install_park must reject after measured_positions drifted inward");
        assert_eq!(err, MotionInstallError::PlanStale, "expected PlanStale on park, got {err:?}");
    }

    // 4 & 5. Both linear Cartesian installers (Legacy AND RT Slot): -2 mm X prepared at q_start,
    //        feedback drifts 5 mm inward before install -> PLAN_STALE!
    for use_slot in [false, true] {
        let mut inner = make_armed();
        let s = inner.current_admission_stamp();
        let snap = ActionSnapshot::from_inner(&inner, s, std::time::Instant::now() + Duration::from_secs(5));
        let body = json!({"frame": "base", "d": [-0.002, 0.0, 0.0], "path": "linear"});
        let prep = match prepare_move_relative(&snap, &body)
            .expect("linear move_relative prepares cleanly before drift")
        {
            PreparedMoveAction::Cartesian(c) => c,
            _ => panic!("expected Cartesian"),
        };
        inner.measured_positions = q_meas_inward.to_vec();
        let ctx = OwnerCheckCtx::for_legacy(&inner);
        let stamp: ActionStamp = prep.stamp;
        let (mut pw, pr) = plan_slots(1, PlanSlot::default);
        let payload = if use_slot {
            let slot = prep.to_plan_slot(None).expect("to_plan_slot");
            let Primitive::Action(StagedAction::Cartesian {
                kind, dof, start, measured_start, has_measured_start, duration_s, power_scale, approach_then, ..
            }) = slot.primitive
            else {
                panic!("expected StagedAction::Cartesian");
            };
            let h = pw.stage(|dst| *dst = slot).expect("publish plan slot");
            let slot_ref = pr.pin(h).expect("pin plan slot");
            CartesianInstallPayload::Slot {
                kind,
                dof: usize::from(dof),
                start,
                measured_start: has_measured_start.then_some(measured_start),
                slot: slot_ref,
                duration_s,
                power_scale,
                approach_then,
            }
        } else {
            CartesianInstallPayload::Legacy(Box::new(prep))
        };
        let err = install_move_cartesian(&mut inner, &stamp, &ctx, payload)
            .expect_err("Cartesian installer must reject after measured_positions drifted inward");
        assert!(
            matches!(err, armd::ArmError::Motion(armd::motion::MotionError::PlanStale(_))),
            "use_slot={use_slot}: expected PlanStale, got {err:?}"
        );
    }
}

/// R2 Item 3 (Major): Standalone gripper, zero-check, and macro gripper commits must NOT
/// reset arm stream minima (`servo_ws_min_depths`) while the arm is outside the workspace.
/// Scenario: 3 mm outward servo step -> gripper action -> 3 mm outward servo step -> 2nd step refused.
#[tokio::test]
async fn gripper_zero_check_and_macro_gripper_do_not_reset_arm_stream_minima() {
    use armd::macro_action::{ChildSpec, ChildTicket};
    use armd::rt_core::broker::{plan_slots, ActionId};
    use armd::rt_core::ops::{PlanSlot, Primitive};
    use armd::rt_core::requests::actions::{
        install_begin_macro, install_macro_child, ActionSnapshot, BeginMacroSpec,
        GripperCommandKind, MacroChildPayload, MacroKind, OwnerCheckCtx, StagedAction,
    };
    use serde_json::json;

    // Arm stretched 10 mm beyond box_max_x.
    let mut cfg = ArmConfig::rebot_dm();
    cfg.collision = None;
    cfg.power_budget = None;
    cfg.lease_timeout_ms = 60_000;
    let q_start: [f32; 7] = [0.0, -0.90, -1.15, 0.85, 0.0, 0.0, 0.0];
    let ws = cfg.workspace.as_mut().unwrap();
    ws.max_reach_m = None;
    ws.keepout_boxes.clear();
    let check_pts = ws.check_points_end_m.clone();
    let max_x_at = |q: &[f32]| -> f64 {
        let pose = ArmModel::REBOT_DEV_ARM.fk_f32(q).unwrap();
        check_pts.iter().map(|c| pose.transform_point(&c.map(f64::from))[0]).fold(pose.p[0], f64::max)
    };
    let start_max_x = max_x_at(&q_start);
    ws.box_max_m[0] = (start_max_x - 0.010) as f32;

    let mut q_probe = q_start;
    q_probe[1] += 0.01;
    let fwd_sign = if max_x_at(&q_probe) > start_max_x { 1.0f32 } else { -1.0f32 };
    let dx_per_rad = ({
        let mut q = q_start;
        q[1] += 0.01 * fwd_sign;
        max_x_at(&q)
    } - start_max_x) / 0.01;
    let mut q_step1 = q_start;
    q_step1[1] += (0.003 / dx_per_rad) as f32 * fwd_sign;
    let mut q_step2 = q_start;
    q_step2[1] += (0.0062 / dx_per_rad) as f32 * fwd_sign;

    // 1. E2E both owners: +3 mm servo -> standalone gripper action -> +3.2 mm servo -> REFUSED;
    //    AND +3 mm servo -> gripper_zero_check -> +3.2 mm servo -> REFUSED!
    for (action_path, action_body) in [
        ("/api/v1/action/gripper", json!({"command": "move", "fraction": 0.1, "speed": 2.0})),
        ("/api/v1/action/gripper_zero_check", json!({"store_offset": false, "threshold_rad": 0.2})),
    ] {
        for use_executor in [false, true] {
            let label = format!("{action_path}/{}", if use_executor { "executor" } else { "legacy" });
            let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
            {
                let mut inner = sup.inner().write().await;
                inner.commanded_positions = q_start.to_vec();
                inner.measured_positions = q_start.to_vec();
                inner.target_positions = q_start.to_vec();
            }
            let exec = if use_executor {
                Some(sup.start_executor(ExecutorConfig::for_tests(Duration::from_millis(20))).await.unwrap())
            } else {
                None
            };
            let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
            let addr = listener.local_addr().unwrap();
            let srv = tokio::spawn({
                let s = sup.clone();
                async move { axum::serve(listener, armd::create_router(s, armd::OperatorAuth::disabled())).await.unwrap(); }
            });
            let tick = (!use_executor).then(|| {
                let t = sup.clone();
                tokio::spawn(async move {
                    let mut iv = tokio::time::interval(Duration::from_millis(20));
                    loop { iv.tick().await; t.tick(0.02).await; }
                })
            });
            let (mut ws_sock, gen) = common::controller(addr).await;
            let ep = sup.get_observation().await.epoch_id;
            assert_eq!(common::rpc(&mut ws_sock, gen, ep, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
            let ep = sup.get_observation().await.epoch_id;

            // Step 1: +3 mm outward servo frame -> accepted and reaches q_step1.
            common::servo(&mut ws_sock, gen, ep, &q_step1).await;
            assert!(common::servo_reject_within(&mut ws_sock, 250).await.is_none(), "{label}: +3mm frame must pass");
            tokio::time::sleep(Duration::from_millis(250)).await;
            let obs1 = sup.get_observation().await;
            assert!(
                (obs1.commanded_positions[1] - q_step1[1]).abs() < 1e-4,
                "{label}: frame 1 must reach q_step1 (got {}, expected {})",
                obs1.commanded_positions[1], q_step1[1]
            );

            // Intervening gripper or gripper_zero_check action while arm is outside.
            let grip_ack = common::rpc(&mut ws_sock, gen, ep, action_path, action_body.clone()).await;
            assert_eq!(grip_ack["type"], "command_ack", "{label}: action ack: {grip_ack}");
            let dl = tokio::time::Instant::now() + Duration::from_secs(6);
            loop {
                let obs = sup.get_observation().await;
                if !obs.action.as_ref().is_some_and(|a| a.state == "running" || a.state == "pending") {
                    break;
                }
                assert!(tokio::time::Instant::now() < dl, "{label}: action timed out");
                tokio::time::sleep(Duration::from_millis(30)).await;
            }

            // Step 2: +3.2 mm further outward (+6.2 mm cumulative from initial baseline) -> MUST BE REFUSED!
            let mut q_step2_cur_g = q_step2;
            q_step2_cur_g[6] = sup.get_observation().await.commanded_positions[6];
            common::servo(&mut ws_sock, gen, ep, &q_step2_cur_g).await;
            let rej = common::servo_reject_within(&mut ws_sock, 600)
                .await
                .unwrap_or_else(|| panic!("{label}: +3mm after {action_path} must be refused (no ratcheting)"));
            assert_eq!(rej["code"], "WORKSPACE_VIOLATION", "{label}: {rej}");
            assert_eq!(rej["constraint_id"], "box_max_x", "{label}: {rej}");

            drop(ws_sock);
            if let Some(t) = tick { t.abort(); }
            srv.abort();
            if let Some(h) = exec { h.stop_and_join(); }
        }
    }

    // 2. Direct macro gripper child commit: `servo_ws_min_depths` seeded outside must survive unchanged.
    {
        let mut inner = armd::SupervisorInner::new(cfg.clone());
        inner.lease_enabled = false;
        let s = inner.current_admission_stamp();
        inner.check_admission(s).and_then(|a| armd::SupervisorInner::enable_torque(a)).unwrap();
        inner.commanded_positions = q_step1.to_vec();
        inner.measured_positions = q_step1.to_vec();
        inner.target_positions = q_step1.to_vec();
        let eval = WorkspaceEvaluator::new_with_tcp(&inner.config.arm_model, inner.config.workspace.as_ref().unwrap(), &inner.config.camera_mount, inner.config.tcp_offset);
        let mut c0 = eval.alloc_buf();
        eval.eval_into(&q_start, &mut c0, "seed").unwrap();
        let baseline = eval.init_min_depths(&c0);
        inner.servo_ws_min_depths = Some((inner.workspace_revision, baseline.clone()));

        let s_m = inner.current_admission_stamp();
        let ctx = OwnerCheckCtx::for_legacy(&inner);
        let spec = BeginMacroSpec::new(MacroKind::Grasp, ctx.boot);
        let snap_m = ActionSnapshot::from_inner(&inner, s_m, std::time::Instant::now() + Duration::from_secs(5));
        let parent_id = install_begin_macro(&mut inner, &snap_m.stamp, &ctx, &spec, None, None).expect("install_begin_macro");
        let mut start_arr = [0.0f32; 8];
        start_arr[..7].copy_from_slice(&q_step1);
        let child = ChildSpec {
            ticket: ChildTicket { parent: ActionId(parent_id), child_seq: 1 },
            timeout_s: 5.0,
            timeout_code: "GRIPPER_TIMEOUT",
            contact: None,
        };
        let mut slot = PlanSlot::default();
        slot.primitive = Primitive::Action(StagedAction::Gripper {
            child: Some(child),
            dof: 7,
            start: start_arr,
            command: GripperCommandKind::Open,
            fraction: None,
            grasp: false,
            speed: 1.0,
            timeout_s: None,
            max_budget_s: None,
            extend_pos_force_close_timeout: false,
            speed_scale: None,
        });
        let (mut pw, pr) = plan_slots(1, PlanSlot::default);
        let h = pw.stage(|dst| *dst = slot).unwrap();
        let slot_ref = pr.pin(h).unwrap();
        let s_c = inner.current_admission_stamp();
        let mut child_stamp = ActionSnapshot::from_inner(&inner, s_c, std::time::Instant::now() + Duration::from_secs(5)).stamp;
        child_stamp.parent_generation = parent_id;
        let _ = install_macro_child(&mut inner, &child_stamp, &ctx, true, MacroChildPayload::Slot(slot_ref))
            .expect("install macro gripper child");
        let after = inner.servo_ws_min_depths.as_ref().expect("macro gripper child must preserve outside arm stream minima");
        assert_eq!(after.1.as_slice(), baseline.as_slice());
    }
}

/// R2 Item 5 (Major): For initially intruding slots (`start_depth > WORKSPACE_EPS`), terminal
/// depth must satisfy `end_depth <= start_depth` EXACTLY (no `+ WORKSPACE_EPS` slack).
/// One slot improving by 0.2 mm while another ends 0.09 mm deeper MUST be refused.
#[test]
fn terminal_rule_rejects_sub_eps_deepening_even_when_another_slot_improves() {
    use armd::kinematics::{check_terminal_workspace_progress, TerminalWorkspaceReject};

    // Slot 0 starts 10.00 mm outside and improves by 0.20 mm -> 9.80 mm (> WORKSPACE_EPS progress).
    // Slot 1 starts 5.00 mm outside and ends 0.09 mm deeper -> 5.09 mm (< WORKSPACE_EPS deepening).
    // Slot 2 starts clear (0.00 mm) and stays within WORKSPACE_EPS (0.05 mm).
    let start_depths = [0.010_00, 0.005_00, 0.0];
    let end_clearances = [-0.009_80, -0.005_09, -0.000_05];
    let err = check_terminal_workspace_progress(&start_depths, &end_clearances)
        .expect_err("slot 1 deepening by 0.09 mm must be rejected even though slot 0 improved 0.2 mm");
    assert_eq!(
        err,
        TerminalWorkspaceReject::Deepened {
            slot: 1,
            depth: 0.005_09,
            start_depth: 0.005_00,
        }
    );

    // Exact equality on slot 1 (5.00 mm -> 5.00 mm) with 0.20 mm improvement on slot 0 -> ALLOWED.
    let end_equal = [-0.009_80, -0.005_00, -0.000_05];
    check_terminal_workspace_progress(&start_depths, &end_equal)
        .expect("exact end_depth == start_depth on slot 1 with >EPS improvement on slot 0 must pass");

    // End-to-end WorkspaceEvaluator check: verify `check_terminal_progress` and `check_terminal_progress_rt`
    // both reject the same 0.09 mm sub-EPS deepening on a real B601-DM evaluator.
    let cfg = ArmConfig::rebot_dm();
    let eval = WorkspaceEvaluator::new_with_tcp(
        &cfg.arm_model,
        cfg.workspace.as_ref().unwrap(),
        &cfg.camera_mount,
        cfg.tcp_offset,
    );
    let mut sd = vec![0.0f64; eval.total_slots];
    let mut ec = vec![1.0f64; eval.total_slots];
    sd[4] = 0.010_00; // end_origin box_max_x
    ec[4] = -0.009_80; // improved 0.20 mm
    sd[5] = 0.005_00; // end_origin box_max_y
    ec[5] = -0.005_09; // deepened 0.09 mm
    let alloc_err = eval
        .check_terminal_progress(&sd, &ec, "sweep")
        .expect_err("allocating check_terminal_progress must reject 0.09 mm deepening");
    assert_eq!(alloc_err.detail.constraint_id.as_deref(), Some("box_max_y"));
    let rt_err = eval
        .check_terminal_progress_rt(&sd, &ec, "sweep")
        .expect_err("RT check_terminal_progress_rt must reject 0.09 mm deepening");
    assert_eq!(rt_err.phase, "sweep");
}

/// R2 Item 6 (Minor): RT inline quintic refusal (`check_workspace_quintic_inline_rt` /
/// `install_move_joint`) must preserve the full structured witness (`constraint_id`,
/// `geometry_id`, `violation_phase`, `reason`) in the preallocated `workspace_violation`
/// buffer — including the exact no-net-return terminal text matching the allocating path.
#[test]
fn rt_inline_quintic_refusal_preserves_structured_witness_including_no_progress() {
    use armd::rt_core::requests::actions::{
        install_move_joint, ActionSnapshot, JointInstallPayload, OwnerCheckCtx,
        PreparedActionKind,
    };
    use armd::rt_core::requests::motion::{prepare_motion, MotionPlanSnapshot, MotionRequest};

    // Start 10 mm outside box_max_x.
    let mut cfg = ArmConfig::rebot_dm();
    cfg.collision = None;
    cfg.power_budget = None;
    cfg.lease_timeout_ms = 60_000;
    let q_start: [f32; 7] = [0.0, -0.90, -1.15, 0.85, 0.0, 0.0, 0.0];
    let ws = cfg.workspace.as_mut().unwrap();
    ws.max_reach_m = None;
    ws.keepout_boxes.clear();
    let check_pts = ws.check_points_end_m.clone();
    let max_x_at = |q: &[f32]| -> f64 {
        let pose = ArmModel::REBOT_DEV_ARM.fk_f32(q).unwrap();
        check_pts.iter().map(|c| pose.transform_point(&c.map(f64::from))[0]).fold(pose.p[0], f64::max)
    };
    let start_max_x = max_x_at(&q_start);
    ws.box_max_m[0] = (start_max_x - 0.010) as f32;

    // 1. Direct formatter parity on TerminalNoProgress (RT vs allocating check_terminal_progress).
    let eval = WorkspaceEvaluator::new_with_tcp(&cfg.arm_model, cfg.workspace.as_ref().unwrap(), &cfg.camera_mount, cfg.tcp_offset);
    let mut c0 = eval.alloc_buf();
    eval.eval_into(&q_start, &mut c0, "sweep").unwrap();
    let sd = eval.init_min_depths(&c0);
    let alloc_np = eval.check_terminal_progress(sd.as_slice(), c0.as_slice(), "sweep").unwrap_err();
    let rt_np = eval.check_terminal_progress_rt(sd.as_slice(), c0.as_slice(), "sweep").unwrap_err();
    let mut rt_detail = armd::kinematics::WorkspaceViolationDetail {
        constraint_id: Some(String::with_capacity(160)),
        geometry_id: Some(String::with_capacity(128)),
        violation_phase: Some(String::with_capacity(64)),
        reason: Some(String::with_capacity(384)),
    };
    eval.write_failure_detail_rt(&rt_np, &mut rt_detail);
    assert_eq!(rt_detail.constraint_id, alloc_np.detail.constraint_id);
    assert_eq!(rt_detail.geometry_id, alloc_np.detail.geometry_id);
    assert_eq!(rt_detail.violation_phase, alloc_np.detail.violation_phase);
    assert_eq!(rt_detail.reason, alloc_np.detail.reason);
    assert!(
        rt_detail.reason.as_deref().unwrap_or("").contains("move makes no net return toward the workspace"),
        "RT no-progress reason must identify no net return, got {:?}",
        rt_detail.reason
    );

    // 2. Live RT install (`JointInstallPayload::Rt`) where live measured pose is at q_start:
    //    (a) tiny J1 nudge deepens an off-axis end_point by 0.1 mm -> structured deepening witness;
    //    (b) arm-invariant target (`q_stay[6] += 0.05`) makes zero net return -> structured
    //        TerminalNoProgress witness ("move makes no net return toward the workspace").
    //    Both must be preserved in `inner.workspace_violation` AND the returned `ArmError`.
    let mut q_deeper = q_start;
    q_deeper[0] += 0.0005;
    let mut q_no_progress = q_start;
    q_no_progress[6] += 0.05;
    let mut inner = armd::SupervisorInner::new(cfg.clone());
    inner.lease_enabled = false;
    inner.rt_executor_owned = true;
    let s0 = inner.current_admission_stamp();
    inner.check_admission(s0).and_then(|a| armd::SupervisorInner::enable_torque(a)).unwrap();
    inner.commanded_positions = q_start.to_vec();
    inner.measured_positions = q_start.to_vec();
    inner.target_positions = q_start.to_vec();
    inner.prewarm_rt_buffers();
    let _ = (prepare_motion, MotionPlanSnapshot::from_supervisor(&inner), MotionRequest::home(None));

    let mut start_arr = [0.0f32; 8];
    let mut vel_arr = [1.0f32; 8];
    start_arr[..7].copy_from_slice(&q_start);
    for (i, l) in cfg.limits.iter().enumerate() { vel_arr[i] = l.max_velocity; }

    for (label, q_tgt, expected_substr) in [
        ("deepened", q_deeper, "this move would go"),
        ("no_progress", q_no_progress, "move makes no net return toward the workspace"),
    ] {
        let s1 = inner.current_admission_stamp();
        let snap = ActionSnapshot::from_inner(&inner, s1, std::time::Instant::now() + Duration::from_secs(5));
        let ctx = OwnerCheckCtx::for_legacy(&inner);
        let mut tgt_arr = [0.0f32; 8];
        tgt_arr[..7].copy_from_slice(&q_tgt);
        let err = install_move_joint(
            &mut inner,
            &snap.stamp,
            &ctx,
            JointInstallPayload::Rt {
                kind: PreparedActionKind::MoveToJoint,
                dof: 7,
                start: start_arr,
                v0: [0.0; 8],
                target: tgt_arr,
                duration_s: 0.4,
                max_velocity: vel_arr,
                power_scale: None,
                settle: false,
                approach_then: None,
                checked_joints: None,
            },
        )
        .unwrap_err();
        let detail = inner
            .workspace_violation
            .as_ref()
            .unwrap_or_else(|| panic!("{label}: RT inline refusal must record structured workspace_violation witness"));
        assert_eq!(detail.constraint_id.as_deref(), Some("box_max_x"), "{label}: {detail:?}");
        assert!(detail.geometry_id.as_deref().is_some_and(|g| !g.is_empty()), "{label}: {detail:?}");
        assert_eq!(detail.violation_phase.as_deref(), Some("sweep"), "{label}: {detail:?}");
        assert!(
            detail.reason.as_deref().unwrap_or("").contains(expected_substr),
            "{label}: expected '{expected_substr}' in witness, got {detail:?}"
        );
        match err {
            armd::ArmError::Safety(armd::safety::SafetyError::WorkspaceViolation { detail: err_detail, .. }) => {
                assert_eq!(err_detail.constraint_id, detail.constraint_id, "{label}");
                assert_eq!(err_detail.geometry_id, detail.geometry_id, "{label}");
                assert_eq!(err_detail.violation_phase, detail.violation_phase, "{label}");
                assert_eq!(err_detail.reason, detail.reason, "{label}");
            }
            other => panic!("{label}: expected SafetyError::WorkspaceViolation with structured detail, got {other:?}"),
        }
    }
}
