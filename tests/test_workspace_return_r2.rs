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

/// R2 Item 1 (Blocker) + R3 Item 2 (Major): Park tracking ramp-out (`pending_normal_unload` ->
/// `keepout_only`) must enforce `floor_z` and `self_collision` hard slots even with NO named
/// keep-out boxes, AND on refusal must hold at the last certified target WITHOUT latching a fault:
/// - Park action fails with `WORKSPACE_VIOLATION` carrying `(constraint_id, geometry_id, violation_phase, reason)`
/// - No fault latched (`SafetyState::Armed`), torque stays ON, torque-off off-rest still refused
/// - Jog moving away from the floor is admitted and completes with 0 faults
/// - Park afterwards succeeds and turns torque off at rest (both Legacy and RT Executor owners)
#[tokio::test]
async fn park_ramp_out_deepening_floor_with_no_keepout_boxes_refused_both_owners() {
    use serde_json::json;

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
    let mut q_away = q0;
    q_away[1] -= down_sign * 0.25;
    let p_away = ArmModel::REBOT_DEV_ARM.fk_f32(&q_away).unwrap().p;
    let floor_z = (p0[2] + 0.005) as f32;
    assert!(p_away[2] > f64::from(floor_z) + 0.02, "q_away must clear floor_z comfortably");

    // 1. Both owners (Legacy and Executor): Park ramp-out deepening floor_z with no keepout boxes
    //    -> holds at last certified target (no fault), fails Park action with WORKSPACE_VIOLATION
    //    witness, refuses off-rest torque-off, admits retreat jog, and succeeds on subsequent Park.
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
        ws.floor_z_m = floor_z;
        // Make resting pose well above floor_z and distinct from q0/q_away so off-rest torque-off
        // is refused and post-jog Park smoothly travels to rest and cuts torque.
        let mut q_rest = q_away;
        q_rest[1] -= down_sign * 0.25;
        let p_rest = ArmModel::REBOT_DEV_ARM.fk_f32(&q_rest).unwrap().p;
        assert!(p_rest[2] > f64::from(floor_z) + 0.02, "rest pose must be above floor_z");
        cfg.zero_reference = None;
        cfg.home_positions = q_rest.to_vec();
        if let Some(rp) = cfg.resting_pose.as_mut() {
            rp.positions = Some(q_rest.to_vec());
        }

        let drv = JitterFeedbackDriver::new(&q0, cfg.arm_dof(), 0.0);
        *drv.enabled.lock().unwrap() = EnableState::Enabled;
        let written: Arc<Mutex<Vec<Vec<JointTarget>>>> = Arc::new(Mutex::new(Vec::new()));
        struct RecordingFollowDriver {
            inner: JitterFeedbackDriver,
            written: Arc<Mutex<Vec<Vec<JointTarget>>>>,
        }
        impl ArmDriver for RecordingFollowDriver {
            fn supports_rt_executor(&self) -> Result<(), &'static str> { self.inner.supports_rt_executor() }
            fn capabilities(&self) -> DriverCapabilities { self.inner.capabilities() }
            fn enable_state(&self) -> EnableState { self.inner.enable_state() }
            fn is_feedback_valid(&self) -> bool { self.inner.is_feedback_valid() }
            fn is_enabled(&self) -> bool { self.inner.is_enabled() }
            fn feedback_ages(&self) -> Vec<Option<Duration>> { self.inner.feedback_ages() }
            fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) { self.inner.feedback_ages_into(out); }
            fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool { self.inner.feedback_all_within(dof, max_age) }
            fn feedback_instants_into(&self, out: &mut Vec<Option<std::time::Instant>>) { self.inner.feedback_instants_into(out); }
            fn feedback_receive_instants_into(&self, out: &mut Vec<Option<std::time::Instant>>) { self.inner.feedback_receive_instants_into(out); }
            fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) { self.inner.feedback_age_ms_into(out); }
            fn feedback_seqs_into(&self, out: &mut Vec<u64>) { self.inner.feedback_seqs_into(out); }
            fn joint_loads_into(&self, out: &mut Vec<f32>) -> bool { self.inner.joint_loads_into(out) }
            fn fresh_joint_positions_into(&self, joints: &[usize], max_age: Duration, out: &mut [f32]) -> bool {
                self.inner.fresh_joint_positions_into(joints, max_age, out)
            }
            fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> { self.inner.read_joint_states() }
            fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> { self.inner.read_joint_states_into(out) }
            fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> { self.inner.read_cached_joint_states_into(out) }
            fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
                self.written.lock().unwrap().push(targets.to_vec());
                self.inner.write_joint_targets(targets)
            }
            fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> { self.inner.set_torque(enable) }
        }
        let rec_drv = RecordingFollowDriver { inner: drv, written: written.clone() };
        let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(rec_drv)));
        {
            let mut inner = sup.inner().write().await;
            let s = inner.current_admission_stamp();
            inner
                .check_admission(s)
                .and_then(|a| armd::SupervisorInner::enable_torque(a))
                .unwrap();
            inner.measured_positions = q0.to_vec();
            // Active Park action + residual lead at q0 while nominal Park commanded sits deeper below floor_z.
            let _ = inner.begin_action("park", None);
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

        tokio::time::sleep(Duration::from_millis(140)).await;
        let obs = sup.get_observation().await;

        // 1a. Non-latching hold: NO fault latched, SafetyState::Armed, torque stays ON.
        assert_eq!(
            obs.safety_state,
            SafetyState::Armed,
            "{label}: Park ramp-out deepening floor_z must hold Armed without latching fault, got {:?} (fault={:?})",
            obs.safety_state,
            obs.fault
        );
        assert!(obs.fault.is_none(), "{label}: no fault may be latched: {:?}", obs.fault);
        assert!(obs.is_torqued, "{label}: torque must remain on off-rest");

        // 1b. Park action failed with WORKSPACE_VIOLATION carrying full structured witness.
        let act = obs.action.as_ref().expect("failed park action must be reported");
        assert_eq!(act.kind.as_str(), "park", "{label}: {act:?}");
        assert_eq!(act.state.as_str(), "failed", "{label}: {act:?}");
        assert_eq!(act.code.as_deref(), Some("WORKSPACE_VIOLATION"), "{label}: {act:?}");
        assert_eq!(act.constraint_id.as_deref(), Some("floor_z"), "{label}: {act:?}");
        assert_eq!(act.geometry_id.as_deref(), Some("end_origin"), "{label}: {act:?}");
        assert_eq!(act.violation_phase.as_deref(), Some("park"), "{label}: {act:?}");
        assert!(act.reason.as_deref().is_some_and(|r| !r.is_empty()), "{label}: {act:?}");
        let ws_v = obs.workspace_violation.as_ref().expect("obs.workspace_violation populated");
        assert_eq!(ws_v.constraint_id.as_deref(), Some("floor_z"), "{label}: {ws_v:?}");
        assert_eq!(ws_v.geometry_id.as_deref(), Some("end_origin"), "{label}: {ws_v:?}");
        assert_eq!(ws_v.violation_phase.as_deref(), Some("park"), "{label}: {ws_v:?}");

        // 1c. No emitted frame deepened the floor intrusion (held at last certified target q0).
        let frames_before_jog = written.lock().unwrap().clone();
        assert!(!frames_before_jog.is_empty(), "{label}: hold frame must be emitted");
        for (idx, frame) in frames_before_jog.iter().enumerate() {
            let qf: Vec<f32> = frame.iter().map(|t| t.position).collect();
            let z = ArmModel::REBOT_DEV_ARM.fk_f32(&qf).unwrap().p[2];
            assert!(
                z >= p0[2] - WORKSPACE_EPS - 1e-5,
                "{label}: emitted frame {idx} deepened floor intrusion (z={z:.6} < start {:.6})",
                p0[2]
            );
        }

        // 1d. Torque-off rule unchanged: off-rest torque-off is still refused.
        let (mut ws, gen) = common::controller(addr).await;
        let ep = sup.get_observation().await.epoch_id;
        let tq_off = common::rpc(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": false})).await;
        assert_eq!(tq_off["type"], "command_reject", "{label}: off-rest torque-off must stay refused: {tq_off}");
        assert_eq!(tq_off["code"], "ARM_NOT_AT_RESTING_POSE", "{label}: {tq_off}");

        // 1e. Jog moving away from floor_z is ADMITTED and completes with 0 faults.
        let ep = sup.get_observation().await.epoch_id;
        let jog_ack = common::rpc(
            &mut ws, gen, ep, "/api/v1/motion/jog",
            json!({"target_positions": q_away, "duration_secs": 0.4}),
        ).await;
        assert_eq!(jog_ack["type"], "command_ack", "{label}: retreat jog must be admitted: {jog_ack}");
        let dl_jog = tokio::time::Instant::now() + Duration::from_secs(5);
        loop {
            let o = sup.get_observation().await;
            assert!(matches!(o.safety_state, SafetyState::Armed | SafetyState::Moving), "{label}: fault during jog: {:?}", o.safety_state);
            if o.safety_state == SafetyState::Armed && o.action.as_ref().is_none_or(|a| !a.is_running()) {
                break;
            }
            assert!(tokio::time::Instant::now() < dl_jog, "{label}: timeout waiting retreat jog");
            tokio::time::sleep(Duration::from_millis(20)).await;
        }

        // 1f. Park afterwards succeeds and turns torque off at rest with 0 faults.
        let ep = sup.get_observation().await.epoch_id;
        let park_ack = common::preset(&mut ws, gen, ep, "park", None).await;
        assert_eq!(park_ack["type"], "preset_ack", "{label}: Park after retreat jog must be admitted: {park_ack}");
        let dl_park = tokio::time::Instant::now() + Duration::from_secs(12);
        let final_obs = loop {
            let o = sup.get_observation().await;
            assert!(
                !matches!(o.safety_state, SafetyState::Fault(_) | SafetyState::EStop),
                "{label}: unexpected fault during second Park: {:?}", o.safety_state
            );
            if !o.is_torqued && o.action.as_ref().is_some_and(|a| a.kind == "park" && a.state == "done") {
                break o;
            }
            assert!(tokio::time::Instant::now() < dl_park, "{label}: timeout waiting second Park; obs.action={:?} torqued={}", o.action, o.is_torqued);
            tokio::time::sleep(Duration::from_millis(25)).await;
        };
        assert!(!final_obs.is_torqued, "{label}: Park must turn torque off at rest");

        drop(ws);
        if let Some(t) = tick { t.abort(); }
        srv.abort();
        if let Some(h) = exec { h.stop_and_join(); }
    }

    // 2. Cockpit i18n & safety.js message check (en + zh): names blocking geometry when witness is
    //    present and falls back to generic guidance when witness is absent (never S.obs fallback).
    let en_js = include_str!("../src/api/cockpit/i18n/en.js");
    let zh_js = include_str!("../src/api/cockpit/i18n/zh.js");
    let safety_js = include_str!("../src/api/cockpit/panels/motion/safety.js");
    assert!(en_js.contains("jog away from {geom}, then Park again"), "en.js must include Park workspace guidance");
    assert!(
        en_js.contains("toast.ws_blocked_generic")
            && en_js.contains("The move would go further outside the workspace; jog back toward the workspace, then try again"),
        "en.js must include generic workspace fallback guidance"
    );
    assert!(zh_js.contains("toast.park_ws_blocked") && zh_js.contains("{geom}"), "zh.js must include Park workspace guidance");
    assert!(
        zh_js.contains("toast.ws_blocked_generic") && zh_js.contains("进一步超出工作空间"),
        "zh.js must include generic workspace fallback guidance"
    );
    assert!(
        safety_js.contains("formatParkWorkspaceBlocked")
            && safety_js.contains("WORKSPACE_VIOLATION")
            && safety_js.contains("toast.ws_blocked_generic")
            && !safety_js.contains("S.obs?.workspace_violation"),
        "safety.js must render geometry-specific or generic fallback without reading S.obs?.workspace_violation"
    );

    // 3. Unit check: `WorkspaceEvaluator::new_with_tcp_in_mode(..., keepout_only = true)`
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
    // Target moves 1.5 mm inward from q_start (valid from q_start, but 1.5 mm DEEPER than q_meas_inward!)
    let mut q_jog_2mm = q_start;
    q_jog_2mm[1] += (0.0015 / dx_per_rad) as f32 * inward_sign;
    // Feedback drifts 3.0 mm inward (within 20 mrad) while commanded stays at q_start, making
    // `q_jog_2mm` and `-1.5 mm X` endpoints DEEPER than live measured -> refused (WORKSPACE_VIOLATION).
    let mut q_meas_inward = q_start;
    q_meas_inward[1] += (0.0030 / dx_per_rad) as f32 * inward_sign;
    assert!(
        (q_meas_inward[1] - q_start[1]).abs() < armd::supervisor::MEASURED_START_DRIFT_TOL_RAD,
        "3 mm inward drift must stay within 20 mrad tolerance: {}",
        (q_meas_inward[1] - q_start[1]).abs()
    );
    assert!(max_x_at(&q_meas_inward) < max_x_at(&q_jog_2mm) - 0.001);
    // Feedback drifts > 20 mrad (25 mrad) -> PLAN_STALE even when endpoint is inside workspace.
    let mut q_meas_big_drift = q_start;
    q_meas_big_drift[1] += 0.025 * inward_sign;

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

    // 1. RT Jog: prepared at q_start (2 mm inward), feedback drifts 5 mm inward (<= 20 mrad,
    //    endpoint now deeper than live start) -> WORKSPACE_VIOLATION! Drift > 20 mrad -> PLAN_STALE!
    {
        let mut inner = make_armed();
        let snap = MotionPlanSnapshot::from_supervisor(&inner);
        let plan = prepare_motion(&snap, &MotionRequest::jog(q_jog_2mm.to_vec(), Some(0.5), None))
            .expect("jog prepares cleanly before drift");
        inner.measured_positions = q_meas_inward.to_vec();
        let s = inner.current_admission_stamp();
        let err = install_jog(&mut inner, s, &plan.slot, plan.heap_traj.clone(), None, false)
            .expect_err("install_jog must reject when inward drift makes endpoint deeper than live start");
        assert_eq!(err, MotionInstallError::WorkspaceViolation, "expected WorkspaceViolation on jog, got {err:?}");
        assert_eq!(inner.workspace_violation.as_ref().and_then(|d| d.constraint_id.as_deref()), Some("box_max_x"));

        inner.measured_positions = q_meas_big_drift.to_vec();
        let s2 = inner.current_admission_stamp();
        let err2 = install_jog(&mut inner, s2, &plan.slot, plan.heap_traj, None, false)
            .expect_err("install_jog must return PlanStale when drift > 20 mrad");
        assert_eq!(err2, MotionInstallError::PlanStale, "expected PlanStale on >20 mrad jog drift, got {err2:?}");
    }

    // 2. RT Home: <= 20 mrad inward drift where Home endpoint is inside -> ADMITTED; > 20 mrad -> PLAN_STALE!
    {
        let mut inner = make_armed();
        let snap = MotionPlanSnapshot::from_supervisor(&inner);
        let plan = prepare_motion(&snap, &MotionRequest::home(None))
            .expect("home prepares cleanly before drift");
        inner.measured_positions = q_meas_big_drift.to_vec();
        let s = inner.current_admission_stamp();
        let err = install_home(&mut inner, s, &plan.slot, plan.heap_traj.clone(), None, false)
            .expect_err("install_home must reject when measured_positions drifted > 20 mrad");
        assert_eq!(err, MotionInstallError::PlanStale, "expected PlanStale on >20 mrad home drift, got {err:?}");

        inner.measured_positions = q_meas_inward.to_vec();
        let s2 = inner.current_admission_stamp();
        install_home(&mut inner, s2, &plan.slot, plan.heap_traj, None, false)
            .expect("install_home must admit <= 20 mrad drift when terminal rule still holds");
    }

    // 3. RT Park: <= 20 mrad inward drift where Park rest endpoint is inside -> ADMITTED; > 20 mrad -> PLAN_STALE!
    {
        let mut inner = make_armed();
        let snap = MotionPlanSnapshot::from_supervisor(&inner);
        let plan = prepare_motion(&snap, &MotionRequest::park())
            .expect("park prepares cleanly before drift");
        inner.measured_positions = q_meas_big_drift.to_vec();
        let s = inner.current_admission_stamp();
        let err = install_park(&mut inner, s, &plan.slot, plan.heap_traj.clone(), None, false)
            .expect_err("install_park must reject when measured_positions drifted > 20 mrad");
        assert_eq!(err, MotionInstallError::PlanStale, "expected PlanStale on >20 mrad park drift, got {err:?}");

        inner.measured_positions = q_meas_inward.to_vec();
        let s2 = inner.current_admission_stamp();
        install_park(&mut inner, s2, &plan.slot, plan.heap_traj, None, false)
            .expect("install_park must admit <= 20 mrad drift when terminal rule still holds");
    }

    // 4 & 5. Both linear Cartesian installers (Legacy AND RT Slot): -2 mm X prepared at q_start:
    //        (a) 5 mm inward drift (<= 20 mrad, endpoint deeper than live start) -> WORKSPACE_VIOLATION!
    //        (b) 25 mrad drift (> 20 mrad) -> PLAN_STALE!
    for use_slot in [false, true] {
        for (big_drift, expect_stale) in [(false, false), (true, true)] {
            let mut inner = make_armed();
            let s = inner.current_admission_stamp();
            let snap = ActionSnapshot::from_inner(&inner, s, std::time::Instant::now() + Duration::from_secs(5));
            let body = json!({"frame": "base", "d": [-0.0015, 0.0, 0.0], "path": "linear"});
            let prep = match prepare_move_relative(&snap, &body)
                .expect("linear move_relative prepares cleanly before drift")
            {
                PreparedMoveAction::Cartesian(c) => c,
                _ => panic!("expected Cartesian"),
            };
            inner.measured_positions = if big_drift { q_meas_big_drift.to_vec() } else { q_meas_inward.to_vec() };
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
                .expect_err("Cartesian installer must reject invalid drift");
            if expect_stale {
                assert!(
                    matches!(err, armd::ArmError::Motion(armd::motion::MotionError::PlanStale(_))),
                    "use_slot={use_slot}: expected PlanStale on >20 mrad drift, got {err:?}"
                );
            } else {
                assert!(
                    matches!(err, armd::ArmError::Safety(armd::safety::SafetyError::WorkspaceViolation { .. })),
                    "use_slot={use_slot}: expected WorkspaceViolation on inward drift deeper than live start, got {err:?}"
                );
                assert_eq!(
                    inner.workspace_violation.as_ref().and_then(|d| d.constraint_id.as_deref()),
                    Some("box_max_x")
                );
            }
        }
    }
}

/// Driver that tracks commanded targets while superimposing deterministic ±jitter (e.g., ±1
/// encoder count or ±2 mrad) on consecutive `read_joint_states` calls so off-thread snapshot
/// measured positions and install-time live measured positions always differ by encoder jitter.
#[derive(Clone)]
struct JitterFeedbackDriver {
    q: Arc<Mutex<Vec<f32>>>,
    enabled: Arc<Mutex<EnableState>>,
    jitter_rad: f32,
    arm_dof: usize,
    tick: Arc<std::sync::atomic::AtomicUsize>,
}

impl JitterFeedbackDriver {
    fn new(q0: &[f32], arm_dof: usize, jitter_rad: f32) -> Self {
        Self {
            q: Arc::new(Mutex::new(q0.to_vec())),
            enabled: Arc::new(Mutex::new(EnableState::Disabled)),
            jitter_rad,
            arm_dof,
            tick: Arc::new(std::sync::atomic::AtomicUsize::new(0)),
        }
    }
    fn fill_jittered_states(&self, out: &mut Vec<JointState>) {
        let q = self.q.lock().unwrap();
        let n = self.tick.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let sign = if n.is_multiple_of(2) { 1.0f32 } else { -1.0f32 };
        out.clear();
        for (i, &pos) in q.iter().enumerate() {
            let jitter = if i < self.arm_dof { sign * self.jitter_rad } else { 0.0 };
            out.push(JointState { position: pos + jitter, velocity: 0.0, torque: 0.0, temperature: 30.0 });
        }
    }
}

impl ArmDriver for JitterFeedbackDriver {
    fn supports_rt_executor(&self) -> Result<(), &'static str> { Ok(()) }
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            velocity_feedback: true,
            torque_feedback: true,
            temperature_feedback: true,
            torque_feedforward: false,
            enable_feedback: true,
            load_feedback: true,
        }
    }
    fn joint_loads_into(&self, out: &mut Vec<f32>) -> bool {
        let n = self.q.lock().unwrap().len();
        out.clear();
        out.resize(n, 0.0);
        true
    }
    fn enable_state(&self) -> EnableState { *self.enabled.lock().unwrap() }
    fn is_feedback_valid(&self) -> bool { true }
    fn is_enabled(&self) -> bool { *self.enabled.lock().unwrap() == EnableState::Enabled }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); self.q.lock().unwrap().len()] }
    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
        let n = self.q.lock().unwrap().len();
        out.clear();
        for _ in 0..n { out.push(Some(Duration::ZERO)); }
    }
    fn feedback_all_within(&self, dof: usize, _max_age: Duration) -> bool {
        dof == self.q.lock().unwrap().len()
    }
    fn feedback_instants_into(&self, out: &mut Vec<Option<std::time::Instant>>) {
        let now = std::time::Instant::now();
        let n = self.q.lock().unwrap().len();
        out.clear();
        for _ in 0..n { out.push(Some(now)); }
    }
    fn feedback_receive_instants_into(&self, out: &mut Vec<Option<std::time::Instant>>) {
        let now = std::time::Instant::now();
        let n = self.q.lock().unwrap().len();
        out.clear();
        for _ in 0..n { out.push(Some(now)); }
    }
    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
        let n = self.q.lock().unwrap().len();
        out.clear();
        for _ in 0..n { out.push(Some(0)); }
    }
    fn feedback_seqs_into(&self, out: &mut Vec<u64>) {
        let seq = self.tick.load(std::sync::atomic::Ordering::Relaxed) as u64 + 1;
        let n = self.q.lock().unwrap().len();
        out.clear();
        for _ in 0..n { out.push(seq); }
    }
    fn fresh_joint_positions_into(&self, joints: &[usize], _max_age: Duration, out: &mut [f32]) -> bool {
        let q = self.q.lock().unwrap();
        if out.len() < joints.len() { return false; }
        for (dst, &j) in out.iter_mut().zip(joints) {
            let Some(&v) = q.get(j) else { return false };
            *dst = v;
        }
        true
    }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let mut out = Vec::new();
        self.fill_jittered_states(&mut out);
        Ok(out)
    }
    fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
        self.fill_jittered_states(out);
        Ok(())
    }
    fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
        self.fill_jittered_states(out);
        Some(())
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        let mut q = self.q.lock().unwrap();
        for (dst, t) in q.iter_mut().zip(targets) {
            *dst = t.position;
        }
        Ok(())
    }
    fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
        *self.enabled.lock().unwrap() = if enable { EnableState::Enabled } else { EnableState::Disabled };
        Ok(())
    }
}

/// Shared R3 Item 1 harness: verifies ±1 encoder count and ±2 mrad live jitter are admitted on
/// Jog, Home, optional linear `move_to`, and Park across both Legacy and RT Executor owners.
async fn assert_encoder_jitter_admitted_both_owners(
    arm_name: &str,
    cfg_base: ArmConfig,
    jitter_levels: [f32; 2],
    test_linear: bool,
) {
    use serde_json::json;
    let rest = cfg_base.resting_positions().to_vec();
    let arm_dof = cfg_base.arm_dof();
    for jitter in jitter_levels {
        for use_executor in [false, true] {
            let label = format!("{arm_name}/jitter={jitter:.4}/{}", if use_executor { "executor" } else { "legacy" });
            let drv = JitterFeedbackDriver::new(&rest, arm_dof, jitter);
            let sup = Arc::new(ArmSupervisor::new(cfg_base.clone()).with_driver(Box::new(drv)));
            {
                let mut inner = sup.inner().write().await;
                inner.commanded_positions = rest.clone();
                inner.measured_positions = rest.clone();
                inner.target_positions = rest.clone();
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

            let (mut ws, gen) = common::controller(addr).await;
            tokio::time::sleep(Duration::from_millis(60)).await;
            let ep = sup.get_observation().await.epoch_id;
            let en = common::rpc(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
            assert_eq!(en["type"], "command_ack", "{label}: enable: {en}");
            tokio::time::sleep(Duration::from_millis(60)).await;

            let wait_idle = |sup: Arc<ArmSupervisor>, tag: String| async move {
                let dl = tokio::time::Instant::now() + Duration::from_secs(15);
                loop {
                    let o = sup.get_observation().await;
                    assert!(
                        matches!(o.safety_state, SafetyState::Armed | SafetyState::Moving | SafetyState::Idle),
                        "{tag}: unexpected fault state {:?}", o.safety_state
                    );
                    if o.action.as_ref().is_none_or(|a| !a.is_running()) && o.safety_state != SafetyState::Moving {
                        break;
                    }
                    assert!(tokio::time::Instant::now() < dl, "{tag}: timeout waiting idle; state={:?} action={:?}", o.safety_state, o.action);
                    tokio::time::sleep(Duration::from_millis(20)).await;
                }
            };

            // 1. Jog +0.10 rad on J1 from rest under live jitter -> ADMITTED
            let mut q_jog = rest.clone();
            q_jog[0] += 0.10;
            let ep = sup.get_observation().await.epoch_id;
            let ack_j = common::rpc(&mut ws, gen, ep, "/api/v1/motion/jog", json!({"target_positions": q_jog, "duration_secs": 0.25})).await;
            assert_eq!(ack_j["type"], "command_ack", "{label}: jog under jitter must be admitted: {ack_j}");
            wait_idle(sup.clone(), format!("{label}/jog")).await;

            // 2. Home under live jitter -> ADMITTED
            let ep = sup.get_observation().await.epoch_id;
            let ack_h = common::rpc(&mut ws, gen, ep, "/api/v1/motion/home", json!({})).await;
            assert_eq!(ack_h["type"], "command_ack", "{label}: home under jitter must be admitted: {ack_h}");
            wait_idle(sup.clone(), format!("{label}/home")).await;

            // 3. Linear move_to under live jitter (B601-DM from home pose) -> ADMITTED
            if test_linear {
                let cur_q = sup.get_observation().await.commanded_positions;
                let p_cur = cfg_base.arm_model.fk_f32(&cur_q).unwrap().p;
                let ep = sup.get_observation().await.epoch_id;
                let ack_m = common::rpc(
                    &mut ws, gen, ep, "/api/v1/action/move_to",
                    json!({"position": [p_cur[0] + 0.015, p_cur[1], p_cur[2]], "orientation": null, "path": "linear"}),
                ).await;
                assert_eq!(ack_m["type"], "command_ack", "{label}: linear move_to under jitter must be admitted: {ack_m}");
                wait_idle(sup.clone(), format!("{label}/linear_move_to")).await;
            }

            // 4. Park under live jitter -> ADMITTED
            let ep = sup.get_observation().await.epoch_id;
            let ack_p = common::preset(&mut ws, gen, ep, "park", None).await;
            assert_eq!(ack_p["type"], "preset_ack", "{label}: park under jitter must be admitted: {ack_p}");
            wait_idle(sup.clone(), format!("{label}/park")).await;

            drop(ws);
            if let Some(t) = tick { t.abort(); }
            srv.abort();
            if let Some(h) = exec { h.stop_and_join(); }
        }
    }
}

/// R3 Item 1 (Blocker): B601-DM ±1 encoder count (~0.3 mrad) and ±2 mrad stationary encoder jitter
/// must NOT trigger `PLAN_STALE` on Park, Home, Jog, or linear `move_to` from rest (both owners).
#[tokio::test]
async fn encoder_jitter_admitted_b601dm_both_owners() {
    let mut cfg: ArmConfig = serde_json::from_reader(
        std::fs::File::open(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/kits/recamera-b601dm/profile/rebot-b601dm.json"
        ))
        .unwrap(),
    )
    .unwrap();
    cfg.collision = None;
    cfg.power_budget = None;
    cfg.lease_timeout_ms = 60_000;
    assert_encoder_jitter_admitted_both_owners("b601dm", cfg, [0.0003f32, 0.002f32], true).await;
}

/// R3 Item 1 (Blocker): SO-ARM101 ±1 encoder count (~1.5 mrad) and ±2 mrad stationary encoder
/// jitter must NOT trigger `PLAN_STALE` on Park, Home, or Jog from rest (both owners).
#[tokio::test]
async fn encoder_jitter_admitted_so101_both_owners() {
    let mut cfg: ArmConfig = serde_json::from_str(include_str!("../profiles/so_arm101.json")).unwrap();
    cfg.init_arm_model_in_memory(Some(include_str!("../assets/so101_new_calib.urdf"))).unwrap();
    cfg.collision = None;
    cfg.power_budget = None;
    cfg.lease_timeout_ms = 60_000;
    assert_encoder_jitter_admitted_both_owners("so101", cfg, [0.0015f32, 0.002f32], false).await;
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

/// R3 Item 3 (Minor): RT Executor command replies over WebSocket MUST carry the full structured
/// inline workspace refusal witness (`constraint_id`, `geometry_id`, `violation_phase`, `reason`,
/// `message`) when off-thread prepare succeeds and install-time RT validation refuses (`install_move_joint_rt`
/// deepened + `TerminalNoProgress`, and `install_move_cartesian_slot_rt` live-measured terminal check).
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_ws_command_reply_carries_inline_workspace_witness() {
    use serde_json::json;

    struct PauseGuard(String);
    impl Drop for PauseGuard {
        fn drop(&mut self) {
            armd::api::ws::set_test_ws_blocking_plan_paused(&self.0, false);
        }
    }

    // Start 10 mm beyond box_max_x at q_start.
    let mut cfg = ArmConfig::rebot_dm();
    cfg.collision = None;
    cfg.power_budget = None;
    cfg.torque_guard = None;
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
    // Live feedback shifts 3 mm inward (~15 mrad <= 20 mrad) between off-thread prepare and RT install,
    // making a prepared -1.5 mm X move deeper than the live measured pose at RT install time.
    let mut q_meas_deeper_refusal = q_start;
    q_meas_deeper_refusal[1] += (0.0030 / dx_per_rad) as f32 * inward_sign;
    assert!(
        (q_meas_deeper_refusal[1] - q_start[1]).abs() < armd::supervisor::MEASURED_START_DRIFT_TOL_RAD
    );

    // Compute the exact joint target of `move_relative [-0.0015, 0, 0] path="joint"` from q_start
    // so setting live measured equal to that target triggers RT inline `TerminalNoProgress`
    // ("move makes no net return toward the workspace") through WebSocket.
    let q_meas_no_progress = {
        use armd::rt_core::requests::actions::{prepare_move_relative, ActionSnapshot, PreparedMoveAction};
        let mut tmp = armd::SupervisorInner::new(cfg.clone());
        tmp.lease_enabled = false;
        let s0 = tmp.current_admission_stamp();
        tmp.check_admission(s0).and_then(|a| armd::SupervisorInner::enable_torque(a)).unwrap();
        tmp.commanded_positions = q_start.to_vec();
        tmp.measured_positions = q_start.to_vec();
        tmp.target_positions = q_start.to_vec();
        let s1 = tmp.current_admission_stamp();
        let snap = ActionSnapshot::from_inner(&tmp, s1, std::time::Instant::now() + Duration::from_secs(5));
        let PreparedMoveAction::Joint(j) =
            prepare_move_relative(&snap, &json!({"frame": "base", "d": [-0.0015, 0.0, 0.0], "path": "joint"}))
                .expect("prepare joint move_relative -1.5 mm X")
        else {
            panic!("expected Joint prepared action");
        };
        let mut q = q_start;
        q.copy_from_slice(&j.target[..7]);
        q
    };
    assert!(
        (0..6).all(|i| (q_meas_no_progress[i] - q_start[i]).abs() < armd::supervisor::MEASURED_START_DRIFT_TOL_RAD),
        "no-progress measured shift must stay within 20 mrad"
    );

    // Driver whose reported measured pose is explicitly controlled via `drv_q` (never overwritten by
    // idle `write_joint_targets`) so we can deterministically shift live feedback while paused.
    struct PinnedFeedbackDriver(JitterFeedbackDriver);
    impl ArmDriver for PinnedFeedbackDriver {
        fn supports_rt_executor(&self) -> Result<(), &'static str> { self.0.supports_rt_executor() }
        fn capabilities(&self) -> DriverCapabilities { self.0.capabilities() }
        fn enable_state(&self) -> EnableState { self.0.enable_state() }
        fn is_feedback_valid(&self) -> bool { self.0.is_feedback_valid() }
        fn is_enabled(&self) -> bool { self.0.is_enabled() }
        fn feedback_ages(&self) -> Vec<Option<Duration>> { self.0.feedback_ages() }
        fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) { self.0.feedback_ages_into(out); }
        fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool { self.0.feedback_all_within(dof, max_age) }
        fn feedback_instants_into(&self, out: &mut Vec<Option<std::time::Instant>>) { self.0.feedback_instants_into(out); }
        fn feedback_receive_instants_into(&self, out: &mut Vec<Option<std::time::Instant>>) { self.0.feedback_receive_instants_into(out); }
        fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) { self.0.feedback_age_ms_into(out); }
        fn feedback_seqs_into(&self, out: &mut Vec<u64>) { self.0.feedback_seqs_into(out); }
        fn joint_loads_into(&self, out: &mut Vec<f32>) -> bool { self.0.joint_loads_into(out) }
        fn fresh_joint_positions_into(&self, joints: &[usize], max_age: Duration, out: &mut [f32]) -> bool {
            self.0.fresh_joint_positions_into(joints, max_age, out)
        }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> { self.0.read_joint_states() }
        fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> { self.0.read_joint_states_into(out) }
        fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> { self.0.read_cached_joint_states_into(out) }
        fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
        fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> { self.0.set_torque(enable) }
    }

    let drv = JitterFeedbackDriver::new(&q_start, cfg.arm_dof(), 0.0);
    let drv_q = drv.q.clone();
    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(PinnedFeedbackDriver(drv))));
    {
        let mut inner = sup.inner().write().await;
        inner.commanded_positions = q_start.to_vec();
        inner.measured_positions = q_start.to_vec();
        inner.target_positions = q_start.to_vec();
    }
    let exec = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(20)))
        .await
        .expect("start_executor");
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let srv = tokio::spawn({
        let s = sup.clone();
        async move { axum::serve(listener, armd::create_router(s, armd::OperatorAuth::disabled())).await.unwrap(); }
    });

    let (mut ws, gen) = common::controller(addr).await;
    tokio::time::sleep(Duration::from_millis(60)).await;
    let ep = sup.get_observation().await.epoch_id;
    let en = common::rpc(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(en["type"], "command_ack", "enable: {en}");
    tokio::time::sleep(Duration::from_millis(60)).await;
    // First WS connection on a fresh ArmSupervisor always gets ConnectionHandle::from_ws_id(1) == "connection:1".
    let conn_id = "connection:1".to_string();

    for (case_label, path_mode, live_q, expected_phase, expected_reason_substr) in [
        ("joint_deepened", "joint", q_meas_deeper_refusal, "sweep", "this move would go"),
        (
            "joint_no_progress",
            "joint",
            q_meas_no_progress,
            "sweep",
            "move makes no net return toward the workspace",
        ),
        ("linear_deepened", "linear", q_meas_deeper_refusal, "plan", "this move would go"),
    ] {
        // Reset driver live measured pose to q_start so off-thread snapshot & prepare succeed.
        *drv_q.lock().unwrap() = q_start.to_vec();
        tokio::time::sleep(Duration::from_millis(60)).await;
        let ep = sup.get_observation().await.epoch_id;

        armd::api::ws::set_test_ws_blocking_plan_paused(&conn_id, true);
        let _guard = PauseGuard(conn_id.clone());
        let drv_q_bg = drv_q.clone();
        let conn_bg = conn_id.clone();
        let unpause_task = tokio::spawn(async move {
            let dl = tokio::time::Instant::now() + Duration::from_secs(3);
            while armd::api::ws::test_ws_blocking_plan_blocked_count(&conn_bg) < 1 {
                assert!(tokio::time::Instant::now() < dl, "{case_label}: blocking planner never paused for {conn_bg}");
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
            // Shift live measured pose AFTER ActionSnapshot::capture, BEFORE RT commit_action.
            *drv_q_bg.lock().unwrap() = live_q.to_vec();
            tokio::time::sleep(Duration::from_millis(60)).await;
            armd::api::ws::set_test_ws_blocking_plan_paused(&conn_bg, false);
        });

        let reply = common::rpc(
            &mut ws,
            gen,
            ep,
            "/api/v1/action/move_relative",
            json!({"frame": "base", "d": [-0.0015, 0.0, 0.0], "path": path_mode}),
        )
        .await;
        unpause_task.await.unwrap();

        assert_eq!(reply["type"], "command_reject", "{case_label}: expected reject: {reply}");
        assert_eq!(reply["code"], "WORKSPACE_VIOLATION", "{case_label}: {reply}");
        assert_eq!(
            reply["constraint_id"], "box_max_x",
            "{case_label}: Executor WS reply must carry constraint_id: {reply}"
        );
        assert!(
            reply["geometry_id"].as_str().is_some_and(|g| !g.is_empty()),
            "{case_label}: Executor WS reply must carry geometry_id: {reply}"
        );
        assert_eq!(
            reply["violation_phase"], expected_phase,
            "{case_label}: Executor WS reply must carry violation_phase: {reply}"
        );
        assert!(
            reply["reason"]
                .as_str()
                .is_some_and(|r| r.contains(expected_reason_substr)),
            "{case_label}: Executor WS reply must carry witness reason containing '{expected_reason_substr}': {reply}"
        );
        assert!(
            reply["message"]
                .as_str()
                .is_some_and(|m| m.contains(expected_reason_substr)),
            "{case_label}: Executor WS reply message must reflect witness: {reply}"
        );
    }

    drop(ws);
    srv.abort();
    exec.stop_and_join();
}

/// R4 (Codex R3 Major 1): Request-bound workspace-violation witness correctness.
///
/// 1. Same-release Step 3 refusal + Step 4 target overwrite/clear: Request 1 refused at Step 3
///    with `box_max_x` keeps its own witness even when Step 4 of the SAME release replaces or
///    clears `inner.workspace_violation`.
/// 2. Two interleaved requests (Request 1 refused with `box_max_x`, Request 2 refused with
///    `floor_z` or succeeding and clearing `obs.workspace_violation`, plus Request 3 refused with
///    a bare `WORKSPACE_VIOLATION` without writing a witness): each request gets ONLY its own
///    witness (or `None`), NEVER another request's witness.
/// 3. Live Executor WebSocket Motion (`/api/v1/motion/jog`) refusal at `src/api/ws/motion.rs`
///    carries its request-bound witness (`constraint_id`, `geometry_id`, `violation_phase`,
///    `reason`) end-to-end.
#[tokio::test]
async fn interleaved_requests_and_motion_replies_bind_workspace_witness_per_request() {
    use armd::rt_core::broker::{
        Authority, BootId, Capacities, ClientId, Reply, Submit,
    };
    use armd::rt_core::ids::ArmId;
    use armd::rt_core::mailbox::{StreamId, TargetAuthority, TargetFrame, MAX_DOF};
    use armd::rt_core::ops::{
        CommitCtx, CoreOps, PlanSlot, Primitive, ReleaseFixture, StubCore,
    };
    use armd::rt_core::requests::motion::{
        self as req_motion, MotionPlanSnapshot, MotionRequest, PlanAuthority,
    };
    use armd::SupervisorInner;
    use serde_json::json;
    use std::time::Instant;

    let q_start = INSPECT;
    let p_start = ArmModel::REBOT_DEV_ARM.fk_f32(&q_start).unwrap().p;
    let mut cfg = ArmConfig::rebot_dm();
    cfg.collision = None;
    cfg.power_budget = None;
    cfg.torque_guard = None;
    cfg.joint_tracking = None;
    cfg.lease_timeout_ms = 60_000;
    let ws_cfg = cfg.workspace.as_mut().unwrap();
    ws_cfg.keepout_boxes.clear();
    ws_cfg.link_capsules.clear();
    ws_cfg.self_collision_enabled = false;
    ws_cfg.self_collision_pairs.clear();
    ws_cfg.check_points_end_m = vec![[0.0, 0.0, 0.0]];
    ws_cfg.max_reach_m = Some(5.0);
    ws_cfg.floor_z_m = (p_start[2] - 0.50) as f32;
    ws_cfg.box_min_m = [-2.0, -2.0, -2.0];
    // Start outside box_max_x by 5 mm.
    ws_cfg.box_max_m = [(p_start[0] - 0.005) as f32, 2.0, 2.0];

    // Find a small jog moving further +X (deepens box_max_x) and a small jog moving -X (retreats).
    let find_step = |want_dx_pos: bool| -> [f32; 7] {
        for j in 0..6 {
            for sign in [-1.0f32, 1.0f32] {
                let mut q = q_start;
                q[j] += sign * 0.012;
                let p = ArmModel::REBOT_DEV_ARM.fk_f32(&q).unwrap().p;
                let dx = p[0] - p_start[0];
                if (want_dx_pos && dx > 0.002) || (!want_dx_pos && dx < -0.002) {
                    return q;
                }
            }
        }
        panic!("could not find joint step with dx sign {want_dx_pos}");
    };
    let q_deeper_x = find_step(true);
    let q_retreat_x = find_step(false);

    // Find a pose that moves downward in Z (for floor_z refusal on Request 2).
    let q_down_z = {
        let mut chosen = q_start;
        for j in 0..6 {
            for sign in [-1.0f32, 1.0f32] {
                let mut q = q_start;
                q[j] += sign * 0.015;
                let p = ArmModel::REBOT_DEV_ARM.fk_f32(&q).unwrap().p;
                if p[2] < p_start[2] - 0.002 {
                    chosen = q;
                    break;
                }
            }
        }
        chosen
    };
    let p_down_z = ArmModel::REBOT_DEV_ARM.fk_f32(&q_down_z).unwrap().p;
    assert!(p_down_z[2] < p_start[2] - 0.002, "q_down_z must move downward");

    // -------------------------------------------------------------------------
    // Part 1 & 2: Deterministic ReleaseFixture + Broker interleaving
    // -------------------------------------------------------------------------
    {
        let mut inner = SupervisorInner::new(cfg.clone());
        inner.lease_enabled = false;
        inner.rt_executor_owned = true;
        inner.register_live_connection(1).unwrap();
        inner.register_live_connection(2).unwrap();
        let s0 = inner.current_admission_stamp();
        inner.check_admission(s0).and_then(|a| SupervisorInner::enable_torque(a)).unwrap();
        inner.commanded_positions = q_start.to_vec();
        inner.measured_positions = q_start.to_vec();
        inner.target_positions = q_start.to_vec();

        let boot = BootId(77);
        let (mut fx, mut broker, mut tw, _stop) =
            ReleaseFixture::new(inner, boot, Capacities { commands: 8, plans: 8, replies: 16 });
        let auth = fx.core.authority(&fx.ctx(fx.now()));
        let deadline = fx.now().max(Instant::now()) + Duration::from_secs(600);

        // Helper to prepare a retreat jog on a permissive snapshot and then shift `target` so
        // Step 3's `check_measured_start_terminal_rt` refuses it with `WORKSPACE_VIOLATION`.
        let make_refused_slot = |fx_ref: &ReleaseFixture<SupervisorInner>, target: &[f32]| {
            let mut snap = MotionPlanSnapshot::from_supervisor(&fx_ref.core);
            let cfg_mut = Arc::make_mut(&mut snap.config);
            cfg_mut.workspace = None;
            cfg_mut.config.workspace = None;
            snap.commanded_positions = q_start.to_vec();
            snap.measured_positions = q_start.to_vec();
            snap.authority = PlanAuthority {
                boot: fx_ref.boot(),
                epoch: auth.epoch,
                lease_generation: auth.lease_generation,
                stop_generation: auth.stop_generation,
                deadline,
            };
            let mut slot = req_motion::prepare_motion(
                &snap,
                &MotionRequest::jog(target.to_vec(), Some(0.4), None),
            )
            .expect("prepare_motion")
            .slot;
            slot.checked_workspace = true;
            slot.workspace_revision = fx_ref.core.workspace_revision;
            slot
        };

        // Configure BOTH `box_max_x` (5 mm outside) AND `floor_z` (1 mm below q_start) so:
        // - Release 1 Step 3: Request 1 (`q_deeper_x`) is refused with `box_max_x`.
        // - Release 1 Step 4 (SAME release!): servo target `q_down_z` is refused with `floor_z`,
        //   immediately overwriting `fx.core.workspace_violation` BEFORE Step 6 publication!
        fx.core.config.workspace.as_mut().unwrap().floor_z_m = (p_start[2] - 0.001) as f32;

        // Submit Request 1: deepens box_max_x -> refused at Step 3 with `box_max_x`.
        let slot1 = make_refused_slot(&fx, &q_deeper_x);
        let r1 = broker
            .submit(
                ClientId(1),
                1,
                auth,
                deadline,
                Submit::Install(&move |dst: &mut PlanSlot| req_motion::stage_jog(dst, &slot1)),
            )
            .unwrap();

        // Authorize a servo stream and push a Step 4 servo target in the SAME release that hits
        // `floor_z` and OVERWRITES `fx.core.workspace_violation` right after Step 3 refused r1!
        fx.core.core.authorize_stream(TargetAuthority {
            stream: Some(StreamId(1)),
            max_input_age: Some(Duration::from_secs(10)),
            ..TargetAuthority::default()
        });
        let mut q_target = [0.0f32; MAX_DOF];
        q_target[..7].copy_from_slice(&q_down_z);
        tw.submit(TargetFrame {
            arm: ArmId::LOCAL,
            boot,
            stream: StreamId(1),
            seq: 1,
            authority: auth,
            policy_generation: 0,
            input_at: fx.now(),
            vision_at: None,
            vision_frame_id: None,
            deadline,
            dof: 7,
            q: q_target,
            replaceable: true,
        });

        let rep1 = fx.step();
        assert_eq!(rep1.command, Some((r1, Err("WORKSPACE_VIOLATION"))), "{rep1:?}");
        assert_eq!(rep1.target, Some((1, Err("WORKSPACE_VIOLATION"))), "Step 4 downward target must be refused: {rep1:?}");

        // Prove Step 4 of the SAME release cleared/overwrote `fx.core.workspace_violation` (it no
        // longer holds Request 1's `box_max_x` witness!), yet Request 1's request-bound witness
        // STILL holds `box_max_x`!
        assert_ne!(
            fx.core.workspace_violation.as_ref().and_then(|d| d.constraint_id.as_deref()),
            Some("box_max_x"),
            "Step 4 target must clear/overwrite workspace_violation in the same release"
        );

        // Release 2: restore low floor_z and submit Request 2 (valid retreat jog) that SUCCEEDS
        // and CLEARS `fx.core.workspace_violation` to None BEFORE we poll Request 1!
        fx.core.config.workspace.as_mut().unwrap().floor_z_m = (p_start[2] - 0.50) as f32;
        let slot_clear = make_refused_slot(&fx, &q_retreat_x);
        let r_clear = broker
            .submit(
                ClientId(2),
                1,
                auth,
                deadline,
                Submit::Install(&move |dst: &mut PlanSlot| req_motion::stage_jog(dst, &slot_clear)),
            )
            .unwrap();
        let rep_clear = fx.step();
        assert!(matches!(rep_clear.command, Some((req, Ok(_))) if req == r_clear), "retreat jog must succeed: {rep_clear:?}");
        assert!(
            fx.core.workspace_violation.is_none(),
            "Request 2 succeeding must clear workspace_violation to None"
        );

        // Release 3: reset to q_start, switch workspace to floor_z and submit Request 3 BEFORE polling!
        let ws_mut = fx.core.config.workspace.as_mut().unwrap();
        ws_mut.floor_z_m = (p_start[2] + 0.004) as f32;
        ws_mut.box_max_m = [2.0, 2.0, 2.0];
        fx.core.active_trajectory = None;
        fx.core.measured_positions = q_start.to_vec();
        fx.core.commanded_positions = q_start.to_vec();
        fx.core.target_positions = q_start.to_vec();
        let slot2 = make_refused_slot(&fx, &q_down_z);
        let r2 = broker
            .submit(
                ClientId(2),
                2,
                auth,
                deadline,
                Submit::Install(&move |dst: &mut PlanSlot| req_motion::stage_jog(dst, &slot2)),
            )
            .unwrap();
        let rep2 = fx.step();
        assert_eq!(rep2.command, Some((r2, Err("WORKSPACE_VIOLATION"))), "{rep2:?}");

        // Confirm `fx.core.workspace_violation` is now `floor_z` (Request 2's witness).
        assert_eq!(
            fx.core.workspace_violation.as_ref().and_then(|d| d.constraint_id.as_deref()),
            Some("floor_z"),
            "observation diagnostic must now hold Request 2's floor_z witness"
        );

        // Poll BOTH replies now and verify each request receives ONLY its own bound witness.
        let mut replies = Vec::new();
        broker.poll(|c, r| {
            replies.push((c, r));
            true
        });
        assert!(
            replies.iter().any(|(_, r)| matches!(r, Reply::Refused { request, code: "WORKSPACE_VIOLATION", .. } if *request == r1)),
            "expected Request 1 WORKSPACE_VIOLATION refusal: {replies:?}"
        );
        assert!(
            replies.iter().any(|(_, r)| matches!(r, Reply::Refused { request, code: "WORKSPACE_VIOLATION", .. } if *request == r2)),
            "expected Request 2 WORKSPACE_VIOLATION refusal: {replies:?}"
        );

        let w_r1 = broker
            .take_workspace_witness(r1)
            .expect("Request 1 must still have its own bound witness after Step 4 clear + Request 2 overwrite");
        assert_eq!(
            w_r1.constraint_id.as_str(),
            "box_max_x",
            "Request 1 must carry box_max_x, NEVER Request 2's floor_z: {w_r1:?}"
        );
        let w_r2 = broker
            .take_workspace_witness(r2)
            .expect("Request 2 must have its own bound floor_z witness");
        assert_eq!(w_r2.constraint_id.as_str(), "floor_z", "Request 2 witness: {w_r2:?}");
        // Consumed once -> second lookup returns None.
        assert!(broker.take_workspace_witness(r1).is_none());

        // Request 3: a CoreOps returning bare `WORKSPACE_VIOLATION` without writing a witness
        // must yield `None` (code-only), NEVER Request 2's leftover observation witness.
        struct BareWsRefuseCore(StubCore);
        impl CoreOps for BareWsRefuseCore {
            fn authority(&self, ctx: &CommitCtx) -> Authority { self.0.authority(ctx) }
            fn planning_snapshot(&self, ctx: &CommitCtx) -> armd::rt_core::ops::PlanningSnapshot {
                self.0.planning_snapshot(ctx)
            }
            fn commit_primitive(
                &mut self,
                _d: &armd::rt_core::broker::Descriptor,
                _primitive: &Primitive,
                _ctx: &CommitCtx,
            ) -> Result<armd::rt_core::ops::Committed, &'static str> {
                Err("WORKSPACE_VIOLATION")
            }
            fn commit_immediate(
                &mut self,
                d: &armd::rt_core::broker::Descriptor,
                op: armd::rt_core::broker::ImmediateOp,
                ctx: &CommitCtx,
            ) -> Result<armd::rt_core::ops::Committed, &'static str> {
                self.0.commit_immediate(d, op, ctx)
            }
            fn commit_transition(
                &mut self,
                d: &armd::rt_core::broker::Descriptor,
                op: armd::rt_core::broker::TransitionOp,
                ctx: &CommitCtx,
            ) -> Result<armd::rt_core::ops::Committed, &'static str> {
                self.0.commit_transition(d, op, ctx)
            }
            fn consume_target(
                &mut self,
                f: &TargetFrame,
                ctx: &CommitCtx,
            ) -> Result<(), armd::rt_core::mailbox::Refusal> {
                self.0.consume_target(f, ctx)
            }
            fn apply_stop(&mut self) { self.0.apply_stop() }
            fn poll_hardware(&mut self, budget: &armd::rt_core::ops::ServiceBudget) -> armd::rt_core::ops::HardwarePoll {
                self.0.poll_hardware(budget)
            }
            fn advance(&mut self, dt_s: f64, generation: u64) { self.0.advance(dt_s, generation) }
            fn action_outcome(&self, id: armd::rt_core::broker::ActionId) -> Option<armd::rt_core::broker::Outcome> {
                self.0.action_outcome(id)
            }
        }
        let (mut fx_bare, mut broker_bare, _tw_b, _stop_b) = ReleaseFixture::new(
            BareWsRefuseCore(StubCore::default()),
            BootId(78),
            Capacities { commands: 4, plans: 4, replies: 8 },
        );
        let r3 = broker_bare
            .submit(
                ClientId(3),
                1,
                Authority::default(),
                deadline,
                Submit::Install(&|dst: &mut PlanSlot| {
                    dst.set(Primitive::JointQuintic {
                        dof: 7,
                        start: [0.0; MAX_DOF],
                        q: [0.0; MAX_DOF],
                        duration_s: 0.4,
                    });
                }),
            )
            .unwrap();
        let rep3 = fx_bare.step();
        assert_eq!(rep3.command, Some((r3, Err("WORKSPACE_VIOLATION"))), "{rep3:?}");
        let mut bare_replies = Vec::new();
        broker_bare.poll(|c, r| {
            bare_replies.push((c, r));
            true
        });
        assert!(
            bare_replies.iter().any(|(_, r)| matches!(r, Reply::Refused { request, code: "WORKSPACE_VIOLATION", .. } if *request == r3)),
            "expected Request 3 WORKSPACE_VIOLATION refusal: {bare_replies:?}"
        );
        assert!(
            broker_bare.take_workspace_witness(r3).is_none(),
            "refusal without a witness must stay code-only (None), never borrow another request's witness"
        );
    }

    // -------------------------------------------------------------------------
    // Part 3: Live Executor WebSocket Motion (`/api/v1/motion/jog`) refusal
    //         carries its request-bound witness via `src/api/ws/motion.rs`
    // -------------------------------------------------------------------------
    {
        struct PinnedDriver(JitterFeedbackDriver);
        impl ArmDriver for PinnedDriver {
            fn supports_rt_executor(&self) -> Result<(), &'static str> { self.0.supports_rt_executor() }
            fn capabilities(&self) -> DriverCapabilities { self.0.capabilities() }
            fn enable_state(&self) -> EnableState { self.0.enable_state() }
            fn is_feedback_valid(&self) -> bool { self.0.is_feedback_valid() }
            fn is_enabled(&self) -> bool { self.0.is_enabled() }
            fn feedback_ages(&self) -> Vec<Option<Duration>> { self.0.feedback_ages() }
            fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) { self.0.feedback_ages_into(out); }
            fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool { self.0.feedback_all_within(dof, max_age) }
            fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) { self.0.feedback_instants_into(out); }
            fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) { self.0.feedback_receive_instants_into(out); }
            fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) { self.0.feedback_age_ms_into(out); }
            fn feedback_seqs_into(&self, out: &mut Vec<u64>) { self.0.feedback_seqs_into(out); }
            fn joint_loads_into(&self, out: &mut Vec<f32>) -> bool { self.0.joint_loads_into(out) }
            fn fresh_joint_positions_into(&self, joints: &[usize], max_age: Duration, out: &mut [f32]) -> bool {
                self.0.fresh_joint_positions_into(joints, max_age, out)
            }
            fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> { self.0.read_joint_states() }
            fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> { self.0.read_joint_states_into(out) }
            fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> { self.0.read_cached_joint_states_into(out) }
            fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
            fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> { self.0.set_torque(enable) }
        }

        // Shift live measured pose further inward (-X) by ~4 mm (still < 20 mrad joint drift) so
        // a jog prepared at q_start to q_retreat_x (moving -2.5 mm X) is now deeper than the live
        // measured pose at RT commit and fails with WORKSPACE_VIOLATION (`box_max_x`, `plan`).
        let mut q_live_inward = q_start;
        for j in 0..6 {
            let dq = q_retreat_x[j] - q_start[j];
            if dq.abs() > 1e-6 {
                q_live_inward[j] = q_start[j] + dq * 1.5;
            }
        }
        assert!(
            (0..6).all(|i| (q_live_inward[i] - q_start[i]).abs() < armd::supervisor::MEASURED_START_DRIFT_TOL_RAD),
            "inward drift must stay within 20 mrad so RT runs workspace re-check rather than PLAN_STALE"
        );
        // Verify off-thread prepare_motion succeeds from q_start -> q_retreat_x.
        {
            let mut tmp = SupervisorInner::new(cfg.clone());
            tmp.lease_enabled = false;
            let s0 = tmp.current_admission_stamp();
            tmp.check_admission(s0).and_then(|a| SupervisorInner::enable_torque(a)).unwrap();
            tmp.commanded_positions = q_start.to_vec();
            tmp.measured_positions = q_start.to_vec();
            tmp.target_positions = q_start.to_vec();
            let mut snap = MotionPlanSnapshot::from_supervisor(&tmp);
            snap.authority = PlanAuthority {
                boot: BootId(1),
                epoch: tmp.safety.epoch_id(),
                lease_generation: 0,
                stop_generation: 0,
                deadline: Instant::now() + Duration::from_secs(5),
            };
            req_motion::prepare_motion(&snap, &MotionRequest::jog(q_retreat_x.to_vec(), Some(0.4), None))
                .expect("off-thread prepare_motion for retreat must succeed");
        }

        let drv = JitterFeedbackDriver::new(&q_start, cfg.arm_dof(), 0.0);
        let drv_q = drv.q.clone();
        let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(PinnedDriver(drv))));
        {
            let mut inner = sup.inner().write().await;
            inner.commanded_positions = q_start.to_vec();
            inner.measured_positions = q_start.to_vec();
            inner.target_positions = q_start.to_vec();
        }
        let exec = sup
            .start_executor(ExecutorConfig::for_tests(Duration::from_millis(20)))
            .await
            .expect("start_executor");
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        let srv = tokio::spawn({
            let s = sup.clone();
            async move { axum::serve(listener, armd::create_router(s, armd::OperatorAuth::disabled())).await.unwrap(); }
        });

        let (mut ws, gen) = common::controller(addr).await;
        tokio::time::sleep(Duration::from_millis(60)).await;
        let ep = sup.get_observation().await.epoch_id;
        let en = common::rpc(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
        assert_eq!(en["type"], "command_ack", "enable: {en}");
        tokio::time::sleep(Duration::from_millis(60)).await;

        let conn_id = "connection:1".to_string();
        struct PauseGuard(String);
        impl Drop for PauseGuard {
            fn drop(&mut self) {
                armd::api::ws::set_test_ws_blocking_plan_paused(&self.0, false);
            }
        }
        let ep = sup.get_observation().await.epoch_id;
        armd::api::ws::set_test_ws_blocking_plan_paused(&conn_id, true);
        let _guard = PauseGuard(conn_id.clone());
        let drv_q_bg = drv_q.clone();
        let conn_bg = conn_id.clone();
        let unpause_task = tokio::spawn(async move {
            let dl = tokio::time::Instant::now() + Duration::from_secs(3);
            while armd::api::ws::test_ws_blocking_plan_blocked_count(&conn_bg) < 1 {
                assert!(tokio::time::Instant::now() < dl, "motion blocking planner never paused for {conn_bg}");
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
            *drv_q_bg.lock().unwrap() = q_live_inward.to_vec();
            tokio::time::sleep(Duration::from_millis(60)).await;
            armd::api::ws::set_test_ws_blocking_plan_paused(&conn_bg, false);
        });

        let reply = common::rpc(
            &mut ws,
            gen,
            ep,
            "/api/v1/motion/jog",
            json!({"target_positions": q_retreat_x, "duration_secs": 0.4}),
        )
        .await;
        unpause_task.await.unwrap();

        assert_eq!(reply["type"], "command_reject", "motion jog must be refused: {reply}");
        assert_eq!(reply["code"], "WORKSPACE_VIOLATION", "motion jog code: {reply}");
        assert_eq!(
            reply["constraint_id"], "box_max_x",
            "Executor WS motion reply must carry its own constraint_id witness: {reply}"
        );
        assert!(
            reply["geometry_id"].as_str().is_some_and(|g| !g.is_empty()),
            "Executor WS motion reply must carry geometry_id: {reply}"
        );
        assert_eq!(
            reply["violation_phase"], "plan",
            "Executor WS motion reply must carry violation_phase=plan: {reply}"
        );
        assert!(
            reply["reason"]
                .as_str()
                .is_some_and(|r| r.contains("this move would go")),
            "Executor WS motion reply must carry witness reason: {reply}"
        );

        drop(ws);
        srv.abort();
        exec.stop_and_join();
    }
}
