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
