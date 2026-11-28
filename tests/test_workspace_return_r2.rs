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
