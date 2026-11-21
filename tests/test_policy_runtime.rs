#![cfg(feature = "policy-runtime")]

use std::fs;
use std::net::SocketAddr;
use std::path::Path;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use futures_util::{SinkExt, StreamExt};
use tempfile::tempdir;
use tokio::net::TcpListener;
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::Message;

use armd::api::create_router_with_sandbox;
use armd::api::sandbox::SandboxState;
use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
use armd::model::{ArmConfig, JointLimit, JointState};
use armd::motion::MotionError;
use armd::policy_runtime::{
    compute_policy_deadline, compute_release_digest, encode_rmlp_bytes,
    parse_policy_start_request, prepare_policy, spawn_policy_runner, validate_policy_fps,
    Activation, ActiveDeploymentContext, MlpLayerSpec, PolicyBackend, PolicyError, PolicyMode,
    PolicyRunStart, PolicySnapshot, PreparedPolicy, ReleaseGate, StrictPolicyContract,
    EMPTY_CALIBRATION_DIGEST, MAX_POLICY_FPS, MAX_POLICY_SECONDS, MIN_LIVE_POLICY_FPS,
};
use armd::{ArmError, ArmSupervisor, FaultReason, OperatorAuth, SafetyError, SafetyState};

fn make_6dof_config() -> ArmConfig {
    let mut cfg = ArmConfig::default();
    cfg.dof = 6;
    cfg.joint_names = (1..=6).map(|i| format!("j{i}")).collect();
    cfg.limits = (1..=6)
        .map(|i| JointLimit::new(format!("j{i}"), -2.0, 2.0, 2.0, 10.0))
        .collect();
    cfg.home_positions = vec![0.0; 6];
    cfg.lease_timeout_ms = 5_000;
    cfg
}

fn write_rmlp_pack(
    dir: &Path,
    joint_names: &[&str],
    unit_mode: &str,
    layers: &[MlpLayerSpec],
    extra_manifest: serde_json::Value,
) -> String {
    let rmlp = encode_rmlp_bytes(layers).unwrap();
    let mut manifest = serde_json::json!({
        "format": "rebot-onnx-policy/v1",
        "name": "test_policy",
        "unit_mode": unit_mode,
        "action_dim": joint_names.len(),
        "action_space": "joint_positions",
        "joint_names": joint_names,
        "model_file": "model.rmlp"
    });
    if let (Some(m_obj), Some(e_obj)) = (manifest.as_object_mut(), extra_manifest.as_object()) {
        for (k, v) in e_obj {
            m_obj.insert(k.clone(), v.clone());
        }
    }
    let pj_bytes = serde_json::to_vec_pretty(&manifest).unwrap();
    fs::write(dir.join("policy.json"), &pj_bytes).unwrap();
    fs::write(dir.join("model.rmlp"), &rmlp).unwrap();
    compute_release_digest(&pj_bytes, &rmlp, "model.rmlp")
}

/// Minimal dummy hardware driver (not mock, not sim) for real-arm gate tests.
struct DummyRealDriver {
    q: Vec<f32>,
    torqued: bool,
}

impl DummyRealDriver {
    fn new(dof: usize) -> Self {
        Self {
            q: vec![0.0; dof],
            torqued: false,
        }
    }
}

impl ArmDriver for DummyRealDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            velocity_feedback: true,
            torque_feedback: false,
            temperature_feedback: true,
            torque_feedforward: false,
            enable_feedback: true,
            load_feedback: false,
        }
    }
    fn enable_state(&self) -> EnableState {
        if self.torqued {
            EnableState::Enabled
        } else {
            EnableState::Disabled
        }
    }
    fn is_feedback_valid(&self) -> bool {
        true
    }
    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        vec![Some(Duration::ZERO); self.q.len()]
    }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        Ok(self
            .q
            .iter()
            .map(|&p| JointState {
                position: p,
                velocity: 0.0,
                torque: 0.0,
                temperature: 30.0,
            })
            .collect())
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        for (i, t) in targets.iter().enumerate().take(self.q.len()) {
            self.q[i] = t.position;
        }
        Ok(())
    }
    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        self.torqued = enabled;
        Ok(())
    }
}

#[tokio::test]
async fn shadow_mode_runs_and_never_mutates_servo_target() {
    let dir = tempdir().unwrap();
    // Constant bias output [0.12, -0.05, 0.08, 0.0, -0.10, 0.04] with zero weights.
    let expected = [0.12f32, -0.05, 0.08, 0.0, -0.10, 0.04];
    let layers = vec![MlpLayerSpec::new(
        6,
        6,
        Activation::None,
        vec![0.0; 36],
        expected.to_vec(),
    )];
    let digest = write_rmlp_pack(
        dir.path(),
        &["j1", "j2", "j3", "j4", "j5", "j6"],
        "rad",
        &layers,
        serde_json::json!({}),
    );

    let cfg = make_6dof_config();
    let prepared = prepare_policy(dir.path(), &cfg).unwrap();
    assert_eq!(prepared.artifact_digest(), digest);
    assert_eq!(prepared.backend_name(), "mlp");

    let sup = Arc::new(ArmSupervisor::new(cfg));
    // Intentionally leave untorqued (Idle) to confirm Shadow works without torque!
    let gen = sup.acquire_lease("conn-shadow").await.unwrap();
    let epoch = sup.get_observation().await.epoch_id;

    let action_id = {
        let mut inner = sup.inner().write().await;
        inner
            .begin_policy_run(PolicyRunStart {
                epoch_id: epoch,
                connection_id: "conn-shadow".into(),
                lease_generation: gen,
                policy_id: "shadow_p".into(),
                artifact_digest: prepared.artifact_digest().to_string(),
                backend: prepared.backend_name().to_string(),
                mode: PolicyMode::Shadow,
                sim_only: false,
                max_first_step_rad: prepared.contract().max_first_step_rad,
                max_step_rad: prepared.contract().max_step_rad,
                tag: None,
                i_am_at_the_arm: false,
                verified_release_gate: None,
            })
            .unwrap()
    };

    let runner = spawn_policy_runner(
        sup.clone(),
        action_id,
        prepared,
        25.0,
        compute_policy_deadline(Some(0.25)).unwrap(),
        None,
        None,
    );
    for _ in 0..8 {
        tokio::time::sleep(Duration::from_millis(20)).await;
        sup.tick(0.02).await;
    }
    let _ = runner.await;

    let obs = sup.get_observation().await;
    let p = obs.policy.expect("policy observation must be populated");
    assert!(!p.active);
    assert_eq!(p.action_id, action_id);
    assert_eq!(p.artifact_digest, digest);
    assert_eq!(p.backend, "mlp");
    assert_eq!(p.mode, "shadow");
    assert!(p.steps > 0, "expected shadow steps > 0");
    assert_eq!(p.stop_reason.as_deref(), Some("MAX_SECONDS"));
    let shadow_tgt = p.last_shadow_target.expect("last_shadow_target present");
    for i in 0..6 {
        assert!(
            (shadow_tgt[i] - expected[i]).abs() < 1e-6,
            "shadow_tgt[{i}]={} vs {}",
            shadow_tgt[i],
            expected[i]
        );
        assert_eq!(obs.positions[i], 0.0, "shadow must not move joint {i}");
        assert_eq!(obs.commanded_positions[i], 0.0);
    }
    assert!(sup.inner().read().await.servo_target.is_none());
}

#[tokio::test]
async fn live_mode_moves_mock_arm_and_enforces_first_step_jump_limit() {
    // Part A: small target within max_first_step_rad moves mock arm.
    let dir_ok = tempdir().unwrap();
    let target_ok = [0.15f32, -0.10, 0.05, 0.0, 0.0, 0.0];
    let layers_ok = vec![MlpLayerSpec::new(
        6,
        6,
        Activation::None,
        vec![0.0; 36],
        target_ok.to_vec(),
    )];
    write_rmlp_pack(
        dir_ok.path(),
        &["j1", "j2", "j3", "j4", "j5", "j6"],
        "rad",
        &layers_ok,
        serde_json::json!({}),
    );

    let cfg = make_6dof_config();
    let prepared_ok = prepare_policy(dir_ok.path(), &cfg).unwrap();
    let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
    let gen = sup.acquire_lease("conn-live").await.unwrap();
    sup.set_torque(true).await.unwrap();
    sup.tick(0.02).await;
    let epoch = sup.get_observation().await.epoch_id;

    let action_id = {
        let mut inner = sup.inner().write().await;
        inner
            .begin_policy_run(PolicyRunStart {
                epoch_id: epoch,
                connection_id: "conn-live".into(),
                lease_generation: gen,
                policy_id: "live_ok".into(),
                artifact_digest: prepared_ok.artifact_digest().to_string(),
                backend: prepared_ok.backend_name().to_string(),
                mode: PolicyMode::Live,
                sim_only: false,
                max_first_step_rad: prepared_ok.contract().max_first_step_rad,
                max_step_rad: prepared_ok.contract().max_step_rad,
                tag: None,
                i_am_at_the_arm: false,
                verified_release_gate: None,
            })
            .unwrap()
    };

    let runner = spawn_policy_runner(
        sup.clone(),
        action_id,
        prepared_ok,
        25.0,
        compute_policy_deadline(Some(0.35)).unwrap(),
        None,
        None,
    );
    for _ in 0..20 {
        tokio::time::sleep(Duration::from_millis(20)).await;
        sup.tick(0.02).await;
    }
    let _ = runner.await;

    let obs = sup.get_observation().await;
    assert!(obs.positions[0] > 0.05, "expected j1 to move toward 0.15, got {}", obs.positions[0]);
    assert!(obs.positions[1] < -0.03, "expected j2 to move toward -0.10, got {}", obs.positions[1]);
    let p = obs.policy.unwrap();
    assert!(p.steps > 0);
    assert_eq!(p.stop_reason.as_deref(), Some("MAX_SECONDS"));

    // Part B: large first-step jump (> default 0.25 rad) rejected without mutating servo_target.
    let dir_jump = tempdir().unwrap();
    let layers_jump = vec![MlpLayerSpec::new(
        6,
        6,
        Activation::None,
        vec![0.0; 36],
        vec![0.60, 0.0, 0.0, 0.0, 0.0, 0.0],
    )];
    write_rmlp_pack(
        dir_jump.path(),
        &["j1", "j2", "j3", "j4", "j5", "j6"],
        "rad",
        &layers_jump,
        serde_json::json!({}),
    );
    let prepared_jump = prepare_policy(dir_jump.path(), &cfg).unwrap();
    let sup2 = Arc::new(ArmSupervisor::new(cfg.clone()));
    let gen2 = sup2.acquire_lease("conn-jump").await.unwrap();
    sup2.set_torque(true).await.unwrap();
    sup2.tick(0.02).await;
    let epoch2 = sup2.get_observation().await.epoch_id;

    let jump_action = {
        let mut inner = sup2.inner().write().await;
        inner
            .begin_policy_run(PolicyRunStart {
                epoch_id: epoch2,
                connection_id: "conn-jump".into(),
                lease_generation: gen2,
                policy_id: "live_jump".into(),
                artifact_digest: prepared_jump.artifact_digest().to_string(),
                backend: prepared_jump.backend_name().to_string(),
                mode: PolicyMode::Live,
                sim_only: false,
                max_first_step_rad: prepared_jump.contract().max_first_step_rad,
                max_step_rad: prepared_jump.contract().max_step_rad,
                tag: None,
                i_am_at_the_arm: false,
                verified_release_gate: None,
            })
            .unwrap()
    };

    let runner2 = spawn_policy_runner(
        sup2.clone(),
        jump_action,
        prepared_jump,
        25.0,
        compute_policy_deadline(Some(0.5)).unwrap(),
        None,
        None,
    );
    let _ = runner2.await;

    let obs2 = sup2.get_observation().await;
    assert_eq!(obs2.positions[0], 0.0, "jump-rejected policy must not move arm");
    assert!(sup2.inner().read().await.servo_target.is_none());
    let p2 = obs2.policy.unwrap();
    assert!(!p2.active);
    assert_eq!(p2.stop_reason.as_deref(), Some("JUMP_LIMIT"));

    // Part C: tighter contract.max_first_step_rad (0.05) and max_step_rad (0.04) enforced.
    let mut inner = sup2.inner().write().await;
    let id_c = inner
        .begin_policy_run(PolicyRunStart {
            epoch_id: epoch2,
            connection_id: "conn-jump".into(),
            lease_generation: gen2,
            policy_id: "step_cap".into(),
            artifact_digest: "abc".into(),
            backend: "mlp".into(),
            mode: PolicyMode::Live,
            sim_only: false,
            max_first_step_rad: Some(0.10),
            max_step_rad: Some(0.04),
            tag: None,
            i_am_at_the_arm: false,
            verified_release_gate: None,
        })
        .unwrap();
    inner.policy_step(id_c, vec![0.05, 0.0, 0.0, 0.0, 0.0, 0.0], Some(0)).unwrap();
    // Second step jumps 0.05 -> 0.12 (delta 0.07 > 0.04): must terminate and clear servo_target!
    let err = inner
        .policy_step(id_c, vec![0.12, 0.0, 0.0, 0.0, 0.0, 0.0], Some(0))
        .unwrap_err();
    assert!(
        matches!(err, ArmError::Rejected { ref code, .. } if code == "STEP_LIMIT"),
        "expected STEP_LIMIT, got {err:?}"
    );
    assert!(inner.servo_target.is_none());
    assert_eq!(
        inner.policy_observation().unwrap().stop_reason.as_deref(),
        Some("STEP_LIMIT")
    );
}

#[tokio::test]
async fn late_frame_after_stop_estop_preempt_and_blocked_hold_is_rejected() {
    let cfg = make_6dof_config();
    let sup = Arc::new(ArmSupervisor::new(cfg));
    let gen1 = sup.acquire_lease("conn-1").await.unwrap();
    sup.set_torque(true).await.unwrap();
    sup.tick(0.02).await;
    let epoch1 = sup.get_observation().await.epoch_id;

    // 1. After stop + newer jog action started: late policy_step must fail and NOT clear newer action or mutate servo_target.
    let id1 = {
        let mut inner = sup.inner().write().await;
        let id = inner
            .begin_policy_run(PolicyRunStart {
                epoch_id: epoch1,
                connection_id: "conn-1".into(),
                lease_generation: gen1,
                policy_id: "p1".into(),
                artifact_digest: "d1".into(),
                backend: "mlp".into(),
                mode: PolicyMode::Live,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: false,
                verified_release_gate: None,
            })
            .unwrap();
        assert!(inner.stop_policy_run(Some(id), "STOPPED"));
        // Start a newer jog action.
        inner
            .jog_action(vec![0.1, 0.0, 0.0, 0.0, 0.0, 0.0], Some(0.5), Some(epoch1), "joint_move")
            .unwrap();
        let active_traj_before = inner.active_trajectory.is_some();
        assert!(active_traj_before);
        let err = inner
            .policy_step(id, vec![0.05, 0.0, 0.0, 0.0, 0.0, 0.0], Some(0))
            .unwrap_err();
        assert!(matches!(err, ArmError::Rejected { ref code, .. } if code == "POLICY_NOT_ACTIVE"));
        assert!(inner.servo_target.is_none(), "late frame must not set servo_target");
        assert!(inner.active_trajectory.is_some(), "late frame must not clear newer action");
        assert!(inner.last_action.as_ref().unwrap().is_running());
        inner.stop_motion();
        id
    };
    let _ = id1;
    sup.tick(0.02).await;

    // 2. After lease preemption: late policy_step rejected, stop_reason = LEASE_LOST, servo_target untouched.
    let id2 = {
        let mut inner = sup.inner().write().await;
        inner
            .begin_policy_run(PolicyRunStart {
                epoch_id: epoch1,
                connection_id: "conn-1".into(),
                lease_generation: gen1,
                policy_id: "p2".into(),
                artifact_digest: "d2".into(),
                backend: "mlp".into(),
                mode: PolicyMode::Live,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: false,
                verified_release_gate: None,
            })
            .unwrap()
    };
    let (_gen2, _) = sup.preempt_lease("conn-2", "other").await.unwrap();
    {
        let mut inner = sup.inner().write().await;
        assert_eq!(
            inner.policy_observation().unwrap().stop_reason.as_deref(),
            Some("LEASE_LOST")
        );
        let err = inner
            .policy_step(id2, vec![0.02, 0.0, 0.0, 0.0, 0.0, 0.0], Some(0))
            .unwrap_err();
        assert!(matches!(err, ArmError::Rejected { ref code, .. } if code == "POLICY_NOT_ACTIVE"));
        assert!(inner.servo_target.is_none());
    }

    // Re-settle handover & re-arm for step 3 (blocked hold) and step 4 (e-stop).
    {
        let mut inner = sup.inner().write().await;
        inner.handover_settling_since = None;
    }
    let gen3 = sup.acquire_lease("conn-2").await.unwrap();
    let epoch3 = sup.get_observation().await.epoch_id;

    // 3. After servo_blocked_hold: policy_step MUST NOT clear servo_blocked_hold and must reject with BLOCKED_HOLD.
    let id3 = {
        let mut inner = sup.inner().write().await;
        let id = inner
            .begin_policy_run(PolicyRunStart {
                epoch_id: epoch3,
                connection_id: "conn-2".into(),
                lease_generation: gen3,
                policy_id: "p3".into(),
                artifact_digest: "d3".into(),
                backend: "mlp".into(),
                mode: PolicyMode::Live,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: false,
                verified_release_gate: None,
            })
            .unwrap();
        inner.servo_blocked_hold = true;
        let err = inner
            .policy_step(id, vec![0.02, 0.0, 0.0, 0.0, 0.0, 0.0], Some(0))
            .unwrap_err();
        assert!(matches!(err, ArmError::Rejected { ref code, .. } if code == "BLOCKED_HOLD"));
        assert!(inner.servo_blocked_hold, "policy_step must NEVER clear servo_blocked_hold");
        assert!(inner.servo_target.is_none());
        assert_eq!(
            inner.policy_observation().unwrap().stop_reason.as_deref(),
            Some("BLOCKED_HOLD")
        );
        // Second late frame after blocked_hold
        let err2 = inner
            .policy_step(id, vec![0.03, 0.0, 0.0, 0.0, 0.0, 0.0], Some(0))
            .unwrap_err();
        assert!(matches!(err2, ArmError::Rejected { ref code, .. } if code == "POLICY_NOT_ACTIVE"));
        assert!(inner.servo_blocked_hold);
        inner.servo_blocked_hold = false;
        id
    };
    let _ = id3;

    // 4. After e_stop: policy_step rejected, stop_reason = ESTOP, servo_target None.
    let id4 = {
        let mut inner = sup.inner().write().await;
        inner
            .begin_policy_run(PolicyRunStart {
                epoch_id: epoch3,
                connection_id: "conn-2".into(),
                lease_generation: gen3,
                policy_id: "p4".into(),
                artifact_digest: "d4".into(),
                backend: "mlp".into(),
                mode: PolicyMode::Live,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: false,
                verified_release_gate: None,
            })
            .unwrap()
    };
    let (state, _epoch_after) = sup.trigger_e_stop().await;
    assert_eq!(state, SafetyState::EStop);
    {
        let mut inner = sup.inner().write().await;
        assert_eq!(
            inner.policy_observation().unwrap().stop_reason.as_deref(),
            Some("ESTOP")
        );
        let err = inner
            .policy_step(id4, vec![0.01, 0.0, 0.0, 0.0, 0.0, 0.0], Some(0))
            .unwrap_err();
        assert!(matches!(err, ArmError::Rejected { ref code, .. } if code == "POLICY_NOT_ACTIVE"));
        assert!(inner.servo_target.is_none());
    }
}

#[tokio::test]
async fn policy_frames_do_not_renew_external_lease_watchdog() {
    let mut cfg = make_6dof_config();
    cfg.lease_timeout_ms = 120; // short watchdog timeout
    let sup = Arc::new(ArmSupervisor::new(cfg));
    let gen = sup.acquire_lease("conn-wd").await.unwrap();
    sup.set_torque(true).await.unwrap();
    sup.tick(0.02).await;
    let epoch = sup.get_observation().await.epoch_id;

    let action_id = {
        let mut inner = sup.inner().write().await;
        inner
            .begin_policy_run(PolicyRunStart {
                epoch_id: epoch,
                connection_id: "conn-wd".into(),
                lease_generation: gen,
                policy_id: "wd_policy".into(),
                artifact_digest: "dw".into(),
                backend: "mlp".into(),
                mode: PolicyMode::Live,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: false,
                verified_release_gate: None,
            })
            .unwrap()
    };

    // Keep feeding policy_step without calling heartbeat() past lease_timeout_ms (120 ms).
    let mut saw_step = false;
    let mut terminated = false;
    for k in 0..12 {
        tokio::time::sleep(Duration::from_millis(20)).await;
        {
            let mut inner = sup.inner().write().await;
            let res = inner.policy_step(
                action_id,
                vec![0.01 * ((k + 1).min(5) as f32), 0.0, 0.0, 0.0, 0.0, 0.0],
                Some(0),
            );
            if res.is_ok() {
                saw_step = true;
            } else {
                terminated = true;
            }
        }
        let obs = sup.tick(0.02).await;
        if matches!(obs.safety_state, SafetyState::Fault(FaultReason::WatchdogTimeout)) {
            terminated = true;
            break;
        }
    }

    assert!(saw_step, "first steps before lease timeout should succeed");
    assert!(terminated, "watchdog must fire even while policy_step runs");
    let obs = sup.get_observation().await;
    assert_eq!(
        obs.safety_state,
        SafetyState::Fault(FaultReason::WatchdogTimeout)
    );
    let p = obs.policy.unwrap();
    assert!(!p.active);
    assert!(
        matches!(p.stop_reason.as_deref(), Some("WATCHDOG_TIMEOUT" | "LEASE_LOST")),
        "unexpected stop_reason: {:?}",
        p.stop_reason
    );
    assert!(sup.inner().read().await.servo_target.is_none());
}

#[tokio::test]
async fn real_driver_rejects_live_mode_but_allows_shadow() {
    let cfg = make_6dof_config();
    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(DummyRealDriver::new(6))));
    sup.set_follower("rebot-rs").await;

    let schema = sup.get_schema().await;
    let pr = schema.capabilities.policy_runtime.expect("policy_runtime capability present");
    assert!(pr.backends.contains(&"mlp".to_string()));
    assert_eq!(
        pr.inputs,
        vec![
            "proprioception".to_string(),
            "detector_top1_box_norm_v1".to_string()
        ]
    );
    assert_eq!(
        pr.modes,
        vec!["shadow".to_string(), "live_gated".to_string()],
        "real driver must advertise shadow and live_gated"
    );

    let gen = sup.acquire_lease("conn-real").await.unwrap();
    sup.set_torque(true).await.unwrap();
    sup.tick(0.02).await;
    let epoch = sup.get_observation().await.epoch_id;

    let mut inner = sup.inner().write().await;
    let live_err = inner
        .begin_policy_run(PolicyRunStart {
            epoch_id: epoch,
            connection_id: "conn-real".into(),
            lease_generation: gen,
            policy_id: "p_real".into(),
            artifact_digest: "dr".into(),
            backend: "mlp".into(),
            mode: PolicyMode::Live,
            sim_only: false,
            max_first_step_rad: None,
            max_step_rad: None,
            tag: None,
            i_am_at_the_arm: false,
            verified_release_gate: None,
        })
        .unwrap_err();
    assert!(
        matches!(live_err, ArmError::Rejected { status: 403, ref code, .. } if code == "OPERATOR_CONFIRMATION_REQUIRED"),
        "expected OPERATOR_CONFIRMATION_REQUIRED, got {live_err:?}"
    );

    let shadow_id = inner
        .begin_policy_run(PolicyRunStart {
            epoch_id: epoch,
            connection_id: "conn-real".into(),
            lease_generation: gen,
            policy_id: "p_real".into(),
            artifact_digest: "dr".into(),
            backend: "mlp".into(),
            mode: PolicyMode::Shadow,
            sim_only: false,
            max_first_step_rad: None,
            max_step_rad: None,
            tag: None,
            i_am_at_the_arm: false,
            verified_release_gate: None,
        })
        .unwrap();
    inner
        .policy_step(shadow_id, vec![0.1, 0.0, 0.0, 0.0, 0.0, 0.0], Some(0))
        .unwrap();
    assert!(inner.servo_target.is_none());
}

#[tokio::test]
async fn mutual_exclusion_between_policy_and_other_motions() {
    let cfg = make_6dof_config();
    let sup = Arc::new(ArmSupervisor::new(cfg));
    let gen = sup.acquire_lease("conn-mx").await.unwrap();
    sup.set_torque(true).await.unwrap();
    sup.tick(0.02).await;
    let epoch = sup.get_observation().await.epoch_id;

    let mut inner = sup.inner().write().await;
    let pid = inner
        .begin_policy_run(PolicyRunStart {
            epoch_id: epoch,
            connection_id: "conn-mx".into(),
            lease_generation: gen,
            policy_id: "mx".into(),
            artifact_digest: "dmx".into(),
            backend: "mlp".into(),
            mode: PolicyMode::Shadow,
            sim_only: false,
            max_first_step_rad: None,
            max_step_rad: None,
            tag: None,
            i_am_at_the_arm: false,
            verified_release_gate: None,
        })
        .unwrap();

    // servo, jog, second policy must all fail with MotionBusy.
    assert_eq!(
        inner.servo(vec![0.0; 6], epoch).unwrap_err(),
        SafetyError::MotionBusy
    );
    let jog_err = inner
        .jog(vec![0.1, 0.0, 0.0, 0.0, 0.0, 0.0], Some(0.5), Some(epoch))
        .unwrap_err();
    assert!(matches!(jog_err, ArmError::Motion(_)), "{jog_err:?}");

    let second_err = inner
        .begin_policy_run(PolicyRunStart {
            epoch_id: epoch,
            connection_id: "conn-mx".into(),
            lease_generation: gen,
            policy_id: "mx2".into(),
            artifact_digest: "dmx2".into(),
            backend: "mlp".into(),
            mode: PolicyMode::Shadow,
            sim_only: false,
            max_first_step_rad: None,
            max_step_rad: None,
            tag: None,
            i_am_at_the_arm: false,
            verified_release_gate: None,
        })
        .unwrap_err();
    assert!(
        matches!(second_err, ArmError::Safety(SafetyError::MotionBusy)),
        "{second_err:?}"
    );

    assert!(inner.stop_policy_run(Some(pid), "STOPPED"));

    // Start a jog trajectory, then verify begin_policy_run returns MotionBusy.
    inner
        .jog(vec![0.1, 0.0, 0.0, 0.0, 0.0, 0.0], Some(0.5), Some(epoch))
        .unwrap();
    let busy_err = inner
        .begin_policy_run(PolicyRunStart {
            epoch_id: epoch,
            connection_id: "conn-mx".into(),
            lease_generation: gen,
            policy_id: "mx3".into(),
            artifact_digest: "dmx3".into(),
            backend: "mlp".into(),
            mode: PolicyMode::Shadow,
            sim_only: false,
            max_first_step_rad: None,
            max_step_rad: None,
            tag: None,
            i_am_at_the_arm: false,
            verified_release_gate: None,
        })
        .unwrap_err();
    assert!(
        matches!(busy_err, ArmError::Safety(SafetyError::MotionBusy)),
        "{busy_err:?}"
    );
}

#[tokio::test]
async fn ws_policy_start_and_stop_end_to_end() {
    let sandbox_dir = tempdir().unwrap();
    let sandbox = Arc::new(SandboxState::new(sandbox_dir.path().to_path_buf()));
    let sup = Arc::new(ArmSupervisor::new(make_6dof_config()));
    let router = create_router_with_sandbox(sup.clone(), OperatorAuth::disabled(), sandbox);

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr: SocketAddr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        axum::serve(listener, router).await.unwrap();
    });

    // Background control tick at 50 Hz.
    let tick_sup = sup.clone();
    let tick_stop = Arc::new(Mutex::new(false));
    let tick_stop_c = tick_stop.clone();
    let tick_task = tokio::spawn(async move {
        loop {
            if *tick_stop_c.lock().unwrap() {
                break;
            }
            tick_sup.tick(0.02).await;
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    });

    // Upload an RMLP policy pack over HTTP.
    let client = reqwest::Client::new();
    upload_rmlp_pack(
        &client,
        addr,
        "ws_demo_policy",
        vec![0.10, -0.05, 0.0, 0.0, 0.0, 0.0],
        serde_json::json!({}),
    )
    .await;

    // Check schema capabilities.
    let schema: serde_json::Value = client
        .get(format!("http://{addr}/api/v1/observation/schema"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(
        schema["capabilities"]["policy_runtime"]["inputs"],
        serde_json::json!(["proprioception", "detector_top1_box_norm_v1"])
    );
    assert!(schema["capabilities"]["policy_runtime"]["modes"]
        .as_array()
        .unwrap()
        .iter()
        .any(|v| v == "live_sim"));

    // Connect WS, acquire lease, enable torque, start policy, observe steps, stop policy.
    let (mut ws, _) = connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
    ws.send(Message::Text(
        serde_json::json!({"type": "acquire_lease", "client_id": "ws_test"})
            .to_string()
            .into(),
    ))
    .await
    .unwrap();
    let gen = loop {
        let msg: serde_json::Value =
            serde_json::from_str(&ws.next().await.unwrap().unwrap().into_text().unwrap()).unwrap();
        if msg["type"] == "lease_granted" {
            break msg["generation"].as_u64().unwrap();
        }
    };

    async fn send_cmd(
        ws: &mut tokio_tungstenite::WebSocketStream<
            tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
        >,
        req_id: u64,
        gen: u64,
        epoch: u64,
        path: &str,
        body: serde_json::Value,
    ) -> serde_json::Value {
        ws.send(Message::Text(
            serde_json::json!({
                "type": "command",
                "request_id": req_id,
                "generation": gen,
                "epoch_id": epoch,
                "path": path,
                "body": body,
            })
            .to_string()
            .into(),
        ))
        .await
        .unwrap();
        tokio::time::timeout(Duration::from_secs(3), async {
            loop {
                let txt = ws.next().await.unwrap().unwrap().into_text().unwrap();
                let v: serde_json::Value = serde_json::from_str(&txt).unwrap();
                if v["type"] == "command_ack" || v["type"] == "command_reject" {
                    return v;
                }
            }
        })
        .await
        .unwrap()
    }

    let ack_torque = send_cmd(
        &mut ws,
        1,
        gen,
        1,
        "/api/v1/torque",
        serde_json::json!({"enable": true}),
    )
    .await;
    assert_eq!(ack_torque["type"], "command_ack");

    let ack_start = send_cmd(
        &mut ws,
        2,
        gen,
        1,
        "/api/v1/action/policy/start",
        serde_json::json!({"policy_id": "ws_demo_policy", "mode": "live", "fps": 25.0, "max_seconds": 3.0}),
    )
    .await;
    assert_eq!(ack_start["type"], "command_ack", "{ack_start}");
    assert_eq!(ack_start["policy_id"], "ws_demo_policy");
    assert_eq!(ack_start["backend"], "mlp");
    assert_eq!(ack_start["mode"], "live");
    assert!(ack_start["action_id"].as_u64().is_some());
    assert!(!ack_start["artifact_digest"].as_str().unwrap().is_empty());

    // Wait until observation shows steps > 0 and j1 moved.
    let mut moved = false;
    for _ in 0..30 {
        tokio::time::sleep(Duration::from_millis(30)).await;
        let obs: serde_json::Value = client
            .get(format!("http://{addr}/api/v1/observation"))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        if obs["policy"]["steps"].as_u64().unwrap_or(0) > 0
            && obs["positions"][0].as_f64().unwrap_or(0.0) > 0.02
        {
            moved = true;
            break;
        }
    }
    assert!(moved, "arm did not move during live WS policy run");

    let ack_stop = send_cmd(
        &mut ws,
        3,
        gen,
        1,
        "/api/v1/action/policy/stop",
        serde_json::json!({}),
    )
    .await;
    assert_eq!(ack_stop["type"], "command_ack", "{ack_stop}");
    assert_eq!(ack_stop["stopped"], true);

    let obs_after = sup.get_observation().await;
    let p_after = obs_after.policy.unwrap();
    assert!(!p_after.active);
    assert_eq!(p_after.stop_reason.as_deref(), Some("STOPPED"));

    *tick_stop.lock().unwrap() = true;
    let _ = tick_task.await;
}

#[tokio::test]
async fn blocked_hold_during_tick_terminates_live_policy_and_is_not_cleared() {
    struct PinnedDriver {
        q: Vec<f32>,
        torqued: bool,
    }
    impl ArmDriver for PinnedDriver {
        fn capabilities(&self) -> DriverCapabilities {
            DriverCapabilities {
                velocity_feedback: true,
                torque_feedback: false,
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
        fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); 6] }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            Ok(self.q.iter().map(|&p| JointState { position: p, velocity: 0.0, torque: 0.0, temperature: 30.0 }).collect())
        }
        fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> {
            // Joint 0 stays pinned at 0.0 (does not move when commanded), triggering blocked hold.
            Ok(())
        }
        fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
            self.torqued = enabled;
            Ok(())
        }
    }

    let mut cfg = make_6dof_config();
    cfg.following_error.persistence_ms = 60;
    cfg.servo_max_lead_rad = 0.05;
    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(PinnedDriver { q: vec![0.0; 6], torqued: false })));
    sup.set_follower("sim").await;
    let gen = sup.acquire_lease("conn-bh").await.unwrap();
    sup.set_torque(true).await.unwrap();
    sup.tick(0.02).await;
    let epoch = sup.get_observation().await.epoch_id;

    let id = {
        let mut inner = sup.inner().write().await;
        inner.begin_policy_run(PolicyRunStart {
            epoch_id: epoch,
            connection_id: "conn-bh".into(),
            lease_generation: gen,
            policy_id: "bh".into(),
            artifact_digest: "dbh".into(),
            backend: "mlp".into(),
            mode: PolicyMode::Live,
            sim_only: false,
            max_first_step_rad: None,
            max_step_rad: None,
            tag: None,
            i_am_at_the_arm: false,
            verified_release_gate: None,
        }).unwrap()
    };

    let mut terminated_during_tick = false;
    for _ in 0..20 {
        {
            let mut inner = sup.inner().write().await;
            if inner.policy_step(id, vec![0.20, 0.0, 0.0, 0.0, 0.0, 0.0], Some(0)).is_err() {
                terminated_during_tick = true;
                break;
            }
        }
        tokio::time::sleep(Duration::from_millis(15)).await;
        sup.tick(0.02).await;
        let inner = sup.inner().read().await;
        if inner.servo_blocked_hold && !inner.policy_run_active() {
            terminated_during_tick = true;
            break;
        }
    }
    assert!(terminated_during_tick, "tick() must enter blocked hold and terminate live policy");
    let inner = sup.inner().read().await;
    assert!(inner.servo_blocked_hold, "servo_blocked_hold must stay true");
    assert!(inner.servo_target.is_none());
    assert_eq!(inner.policy_observation().unwrap().stop_reason.as_deref(), Some("BLOCKED_HOLD"));
}

#[tokio::test]
async fn shadow_mode_with_so101_norm_and_permuted_joints_matches_exact_math() {
    let dir = tempdir().unwrap();
    // Identity MLP on permuted joints ["j3", "j1", "j2", "j4", "j5", "j6"] with normalization + so101_norm.
    let w = (0..36).map(|i| if i % 7 == 0 { 1.0 } else { 0.0 }).collect();
    let layers = vec![MlpLayerSpec::new(6, 6, Activation::None, w, vec![0.0; 6])];
    write_rmlp_pack(
        dir.path(),
        &["j3", "j1", "j2", "j4", "j5", "j6"],
        "so101_norm",
        &layers,
        serde_json::json!({
            "normalization": {
                "state_mean": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "state_std": [100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
                "action_mean": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "action_std": [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
            }
        }),
    );
    let cfg = make_6dof_config();
    let mut prepared = prepare_policy(dir.path(), &cfg).unwrap();
    let measured = [0.5f32, -1.0, 0.25, 0.0, 1.5, -0.5];
    let out = prepared.step_infer(&measured).unwrap();
    for i in 0..6 {
        assert!((out[i] - measured[i]).abs() < 1e-4, "out[{i}]={} vs {}", out[i], measured[i]);
    }
}

fn b64_encode(bytes: &[u8]) -> String {
    const B64: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut b64 = String::new();
    for chunk in bytes.chunks(3) {
        let b0 = chunk[0] as usize;
        let b1 = if chunk.len() > 1 { chunk[1] as usize } else { 0 };
        let b2 = if chunk.len() > 2 { chunk[2] as usize } else { 0 };
        b64.push(B64[b0 >> 2] as char);
        b64.push(B64[((b0 & 3) << 4) | (b1 >> 4)] as char);
        b64.push(if chunk.len() > 1 { B64[((b1 & 0xF) << 2) | (b2 >> 6)] as char } else { '=' });
        b64.push(if chunk.len() > 2 { B64[b2 & 0x3F] as char } else { '=' });
    }
    b64
}

async fn upload_rmlp_pack(
    client: &reqwest::Client,
    addr: SocketAddr,
    policy_id: &str,
    bias: Vec<f32>,
    extra: serde_json::Value,
) {
    let layers = vec![MlpLayerSpec::new(6, 6, Activation::None, vec![0.0; 36], bias)];
    let rmlp_bytes = encode_rmlp_bytes(&layers).unwrap();
    let mut policy_json = serde_json::json!({
        "format": "rebot-onnx-policy/v1",
        "id": policy_id,
        "name": policy_id,
        "unit_mode": "rad",
        "action_dim": 6,
        "action_space": "joint_positions",
        "joint_names": ["j1", "j2", "j3", "j4", "j5", "j6"],
        "inputs": {"state": {"shape": [1, 6], "joint_names": ["j1", "j2", "j3", "j4", "j5", "j6"]}},
        "outputs": {"action": {"shape": [1, 6], "space": "joint_positions"}},
        "model_file": "model.rmlp"
    });
    if let (Some(m_obj), Some(e_obj)) = (policy_json.as_object_mut(), extra.as_object()) {
        for (k, v) in e_obj {
            m_obj.insert(k.clone(), v.clone());
        }
    }
    for attempt in 0..50 {
        let resp = client
            .post(format!("http://{addr}/api/v1/sandbox/policies"))
            .json(&serde_json::json!({
                "id": policy_id,
                "policy_json": policy_json,
                "files": {
                    "model.rmlp": format!("base64:{}", b64_encode(&rmlp_bytes)),
                }
            }))
            .send()
            .await
            .unwrap();
        if resp.status() == reqwest::StatusCode::TOO_MANY_REQUESTS && attempt < 49 {
            tokio::time::sleep(Duration::from_millis(15)).await;
            continue;
        }
        assert_eq!(resp.status(), reqwest::StatusCode::OK);
        return;
    }
}

#[tokio::test]
async fn max_seconds_validation_and_deadline_reject_invalid_values_without_panic() {
    // 1. Direct helper checks (including NaN / Inf / 1e20 / <= 0 / > MAX_POLICY_SECONDS).
    for bad in [1e20_f64, f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -1.0, 0.0, MAX_POLICY_SECONDS + 1.0] {
        let err = compute_policy_deadline(Some(bad)).unwrap_err();
        assert!(
            matches!(err, MotionError::InvalidCommand(_)),
            "expected InvalidCommand for {bad:?}, got {err:?}"
        );
    }
    assert!(compute_policy_deadline(None).unwrap().is_none());
    assert!(compute_policy_deadline(Some(MAX_POLICY_SECONDS)).unwrap().is_some());

    // 2. parse_policy_start_request rejects extreme and invalid max_seconds without panicking.
    for bad in [
        serde_json::json!(1e20),
        serde_json::json!(-1.0),
        serde_json::json!(0.0),
        serde_json::json!(MAX_POLICY_SECONDS + 0.1),
        serde_json::json!("NaN"),
    ] {
        let body = serde_json::json!({
            "policy_id": "p_bad_max",
            "mode": "shadow",
            "max_seconds": bad,
        });
        let err = parse_policy_start_request(&body).unwrap_err();
        assert!(matches!(err, MotionError::InvalidCommand(_)), "{err:?}");
    }

    // 3. WS /api/v1/action/policy/start rejects invalid max_seconds without installing a PolicyRun.
    let sandbox_dir = tempdir().unwrap();
    let sandbox = Arc::new(SandboxState::new(sandbox_dir.path().to_path_buf()));
    let sup = Arc::new(ArmSupervisor::new(make_6dof_config()));
    let router = create_router_with_sandbox(sup.clone(), OperatorAuth::disabled(), sandbox);
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr: SocketAddr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        axum::serve(listener, router).await.unwrap();
    });
    let client = reqwest::Client::new();
    upload_rmlp_pack(&client, addr, "p_max", vec![0.0; 6], serde_json::json!({})).await;

    let (mut ws, _) = connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
    ws.send(Message::Text(
        serde_json::json!({"type": "acquire_lease", "client_id": "max_sec_ws"})
            .to_string()
            .into(),
    ))
    .await
    .unwrap();
    let gen = loop {
        let msg: serde_json::Value =
            serde_json::from_str(&ws.next().await.unwrap().unwrap().into_text().unwrap()).unwrap();
        if msg["type"] == "lease_granted" {
            break msg["generation"].as_u64().unwrap();
        }
    };

    for (idx, bad_secs) in [serde_json::json!(1e20), serde_json::json!(-1.0), serde_json::json!(0.0)]
        .into_iter()
        .enumerate()
    {
        ws.send(Message::Text(
            serde_json::json!({
                "type": "command",
                "request_id": (idx + 1) as u64,
                "generation": gen,
                "epoch_id": 1,
                "path": "/api/v1/action/policy/start",
                "body": {"policy_id": "p_max", "mode": "shadow", "max_seconds": bad_secs},
            })
            .to_string()
            .into(),
        ))
        .await
        .unwrap();
        let reply = tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                let txt = ws.next().await.unwrap().unwrap().into_text().unwrap();
                let v: serde_json::Value = serde_json::from_str(&txt).unwrap();
                if v["type"] == "command_ack" || v["type"] == "command_reject" {
                    return v;
                }
            }
        })
        .await
        .unwrap();
        assert_eq!(reply["type"], "command_reject", "{reply}");
        assert_eq!(reply["code"], "INVALID_COMMAND", "{reply}");
        assert!(sup.inner().read().await.policy_run.is_none());
    }
}

#[tokio::test]
async fn live_mode_rejects_low_fps_and_shadow_accepts_1hz() {
    assert_eq!(MIN_LIVE_POLICY_FPS, 10.0);

    // Direct parser and validator assertions.
    for low in [1.0_f64, 5.0, 9.9] {
        let body = serde_json::json!({"policy_id": "p1", "mode": "live", "fps": low});
        let err = parse_policy_start_request(&body).unwrap_err();
        assert!(matches!(err, MotionError::InvalidCommand(_)), "live fps={low}: {err:?}");
        assert!(matches!(validate_policy_fps(PolicyMode::Live, low), Err(MotionError::InvalidCommand(_))));
    }
    let high_err = validate_policy_fps(PolicyMode::Live, MAX_POLICY_FPS + 1.0).unwrap_err();
    assert!(matches!(high_err, MotionError::InvalidCommand(_)));

    let live_ok = parse_policy_start_request(&serde_json::json!({"policy_id": "p1", "mode": "live", "fps": 20.0})).unwrap();
    assert_eq!(live_ok.fps, Some(20.0));
    let shadow_1hz = parse_policy_start_request(&serde_json::json!({"policy_id": "p1", "mode": "shadow", "fps": 1.0})).unwrap();
    assert_eq!(shadow_1hz.fps, Some(1.0));
    assert!(matches!(validate_policy_fps(PolicyMode::Shadow, 0.5), Err(MotionError::InvalidCommand(_))));

    // WS end-to-end: contract.fps = 5.0 is rejected in Live mode when request omits fps,
    // accepted in Shadow mode; explicit fps = 1.0/5.0 in Live is rejected, fps = 20.0 accepted.
    let sandbox_dir = tempdir().unwrap();
    let sandbox = Arc::new(SandboxState::new(sandbox_dir.path().to_path_buf()));
    let sup = Arc::new(ArmSupervisor::new(make_6dof_config()));
    let router = create_router_with_sandbox(sup.clone(), OperatorAuth::disabled(), sandbox);
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr: SocketAddr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        axum::serve(listener, router).await.unwrap();
    });
    let client = reqwest::Client::new();
    upload_rmlp_pack(&client, addr, "p_fps_5", vec![0.0; 6], serde_json::json!({"fps": 5.0})).await;

    let (mut ws, _) = connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
    ws.send(Message::Text(
        serde_json::json!({"type": "acquire_lease", "client_id": "fps_ws"})
            .to_string()
            .into(),
    ))
    .await
    .unwrap();
    let gen = loop {
        let msg: serde_json::Value =
            serde_json::from_str(&ws.next().await.unwrap().unwrap().into_text().unwrap()).unwrap();
        if msg["type"] == "lease_granted" {
            break msg["generation"].as_u64().unwrap();
        }
    };

    async fn send_ws_cmd(
        ws: &mut tokio_tungstenite::WebSocketStream<
            tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
        >,
        req_id: u64,
        gen: u64,
        path: &str,
        body: serde_json::Value,
    ) -> serde_json::Value {
        ws.send(Message::Text(
            serde_json::json!({
                "type": "command",
                "request_id": req_id,
                "generation": gen,
                "epoch_id": 1,
                "path": path,
                "body": body,
            })
            .to_string()
            .into(),
        ))
        .await
        .unwrap();
        tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                let txt = ws.next().await.unwrap().unwrap().into_text().unwrap();
                let v: serde_json::Value = serde_json::from_str(&txt).unwrap();
                if v["type"] == "command_ack" || v["type"] == "command_reject" {
                    return v;
                }
            }
        })
        .await
        .unwrap()
    }

    assert_eq!(send_ws_cmd(&mut ws, 1, gen, "/api/v1/torque", serde_json::json!({"enable": true})).await["type"], "command_ack");

    // Live with explicit fps = 1.0 -> rejected
    let r1 = send_ws_cmd(&mut ws, 2, gen, "/api/v1/action/policy/start", serde_json::json!({"policy_id": "p_fps_5", "mode": "live", "fps": 1.0})).await;
    assert_eq!(r1["type"], "command_reject", "{r1}");
    assert_eq!(r1["code"], "INVALID_COMMAND", "{r1}");
    assert!(sup.inner().read().await.policy_run.is_none());

    // Live omitting fps while contract has fps = 5.0 -> rejected
    let r2 = send_ws_cmd(&mut ws, 3, gen, "/api/v1/action/policy/start", serde_json::json!({"policy_id": "p_fps_5", "mode": "live"})).await;
    assert_eq!(r2["type"], "command_reject", "{r2}");
    assert_eq!(r2["code"], "INVALID_COMMAND", "{r2}");
    assert!(sup.inner().read().await.policy_run.is_none());

    // Shadow omitting fps while contract has fps = 5.0 -> accepted
    let r3 = send_ws_cmd(&mut ws, 4, gen, "/api/v1/action/policy/start", serde_json::json!({"policy_id": "p_fps_5", "mode": "shadow"})).await;
    assert_eq!(r3["type"], "command_ack", "{r3}");
    assert_eq!(send_ws_cmd(&mut ws, 5, gen, "/api/v1/action/policy/stop", serde_json::json!({})).await["type"], "command_ack");

    // Live with explicit fps = 20.0 overriding contract fps = 5.0 -> accepted
    let r4 = send_ws_cmd(&mut ws, 6, gen, "/api/v1/action/policy/start", serde_json::json!({"policy_id": "p_fps_5", "mode": "live", "fps": 20.0})).await;
    assert_eq!(r4["type"], "command_ack", "{r4}");
    assert_eq!(send_ws_cmd(&mut ws, 7, gen, "/api/v1/action/policy/stop", serde_json::json!({})).await["type"], "command_ack");
}

#[tokio::test]
async fn inference_error_terminates_action_as_failed_with_code_and_message() {
    // Part A: NON_FINITE from NaN measured_positions
    let dir = tempdir().unwrap();
    let layers = vec![MlpLayerSpec::new(6, 6, Activation::None, vec![0.0; 36], vec![0.0; 6])];
    write_rmlp_pack(dir.path(), &["j1", "j2", "j3", "j4", "j5", "j6"], "rad", &layers, serde_json::json!({}));
    let cfg = make_6dof_config();
    let prepared = prepare_policy(dir.path(), &cfg).unwrap();
    let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
    let gen = sup.acquire_lease("conn-nan").await.unwrap();
    let epoch = sup.get_observation().await.epoch_id;
    let action_id = {
        let mut inner = sup.inner().write().await;
        inner.measured_positions[0] = f32::NAN;
        inner
            .begin_policy_run(PolicyRunStart {
                epoch_id: epoch,
                connection_id: "conn-nan".into(),
                lease_generation: gen,
                policy_id: "p_nan".into(),
                artifact_digest: prepared.artifact_digest().to_string(),
                backend: prepared.backend_name().to_string(),
                mode: PolicyMode::Shadow,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: false,
                verified_release_gate: None,
            })
            .unwrap()
    };
    let runner = spawn_policy_runner(sup.clone(), action_id, prepared, 25.0, None, None, None);
    let _ = runner.await;

    {
        let inner = sup.inner().read().await;
        let act = inner.last_action.as_ref().expect("last_action present");
        assert_eq!(act.id, action_id);
        assert_eq!(act.state, "failed", "inference error must mark action failed, not cancelled");
        assert_eq!(act.code.as_deref(), Some("NON_FINITE"));
        assert!(act.message.as_deref().is_some_and(|m| !m.is_empty()), "expected non-empty error message");
        assert_eq!(
            inner.policy_observation().unwrap().stop_reason.as_deref(),
            Some("NON_FINITE")
        );
    }

    // Part B: INFERENCE_ERROR from backend failure
    struct FailingBackend;
    impl PolicyBackend for FailingBackend {
        fn backend_name(&self) -> &'static str { "mock-fail" }
        fn input_dim(&self) -> usize { 6 }
        fn output_dim(&self) -> usize { 6 }
        fn infer(&mut self, _input: &[f32], _output: &mut [f32]) -> Result<(), PolicyError> {
            Err(PolicyError::InferenceFailed("backend execution fault".into()))
        }
    }

    let snap = PolicySnapshot::load_from_dir(dir.path()).unwrap();
    let contract = StrictPolicyContract::parse(&snap.policy_json_bytes).unwrap();
    let prepared_fail = PreparedPolicy::from_backend(snap, contract, Box::new(FailingBackend), &cfg).unwrap();

    let sup2 = Arc::new(ArmSupervisor::new(cfg));
    let gen2 = sup2.acquire_lease("conn-fail").await.unwrap();
    let epoch2 = sup2.get_observation().await.epoch_id;
    let action_id2 = {
        let mut inner = sup2.inner().write().await;
        inner
            .begin_policy_run(PolicyRunStart {
                epoch_id: epoch2,
                connection_id: "conn-fail".into(),
                lease_generation: gen2,
                policy_id: "p_fail".into(),
                artifact_digest: prepared_fail.artifact_digest().to_string(),
                backend: prepared_fail.backend_name().to_string(),
                mode: PolicyMode::Shadow,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: false,
                verified_release_gate: None,
            })
            .unwrap()
    };
    let runner2 = spawn_policy_runner(sup2.clone(), action_id2, prepared_fail, 25.0, None, None, None);
    let _ = runner2.await;

    let inner2 = sup2.inner().read().await;
    let act2 = inner2.last_action.as_ref().expect("last_action present");
    assert_eq!(act2.id, action_id2);
    assert_eq!(act2.state, "failed", "backend error must mark action failed, not cancelled");
    assert_eq!(act2.code.as_deref(), Some("INFERENCE_ERROR"));
    assert!(
        act2.message.as_deref().is_some_and(|m| m.contains("backend execution fault")),
        "expected error message to preserve backend error, got {:?}",
        act2.message
    );
    assert_eq!(
        inner2.policy_observation().unwrap().stop_reason.as_deref(),
        Some("INFERENCE_ERROR")
    );
}

#[tokio::test]
async fn expired_deadline_before_submission_finishes_done_and_applies_no_target() {
    let dir = tempdir().unwrap();
    let layers = vec![MlpLayerSpec::new(
        6,
        6,
        Activation::None,
        vec![0.0; 36],
        vec![0.15, 0.0, 0.0, 0.0, 0.0, 0.0],
    )];
    write_rmlp_pack(dir.path(), &["j1", "j2", "j3", "j4", "j5", "j6"], "rad", &layers, serde_json::json!({}));

    let cfg = make_6dof_config();

    // Part A: Deadline already expired at runner start.
    let prepared_a = prepare_policy(dir.path(), &cfg).unwrap();
    let sup_a = Arc::new(ArmSupervisor::new(cfg.clone()));
    let gen_a = sup_a.acquire_lease("conn-dl-a").await.unwrap();
    sup_a.set_torque(true).await.unwrap();
    sup_a.tick(0.02).await;
    let epoch_a = sup_a.get_observation().await.epoch_id;
    let id_a = {
        let mut inner = sup_a.inner().write().await;
        inner
            .begin_policy_run(PolicyRunStart {
                epoch_id: epoch_a,
                connection_id: "conn-dl-a".into(),
                lease_generation: gen_a,
                policy_id: "p_dl_a".into(),
                artifact_digest: prepared_a.artifact_digest().to_string(),
                backend: prepared_a.backend_name().to_string(),
                mode: PolicyMode::Live,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: false,
                verified_release_gate: None,
            })
            .unwrap()
    };
    let expired = Some(Instant::now() - Duration::from_millis(10));
    let runner_a = spawn_policy_runner(sup_a.clone(), id_a, prepared_a, 25.0, expired, None, None);
    let _ = runner_a.await;
    {
        let inner = sup_a.inner().read().await;
        assert!(inner.servo_target.is_none());
        let act = inner.last_action.as_ref().unwrap();
        assert_eq!(act.state, "done");
        let pol = inner.policy_observation().unwrap();
        assert_eq!(pol.stop_reason.as_deref(), Some("MAX_SECONDS"));
        assert_eq!(pol.steps, 0);
    }

    // Part B: Deadline expires DURING step_infer -> post-inference check under write lock blocks policy_step.
    struct SlowBackend {
        calls: Arc<AtomicUsize>,
    }
    impl PolicyBackend for SlowBackend {
        fn backend_name(&self) -> &'static str { "mock-slow" }
        fn input_dim(&self) -> usize { 6 }
        fn output_dim(&self) -> usize { 6 }
        fn infer(&mut self, _input: &[f32], output: &mut [f32]) -> Result<(), PolicyError> {
            self.calls.fetch_add(1, Ordering::SeqCst);
            std::thread::sleep(Duration::from_millis(60));
            output.copy_from_slice(&[0.15, 0.0, 0.0, 0.0, 0.0, 0.0]);
            Ok(())
        }
    }

    let snap = PolicySnapshot::load_from_dir(dir.path()).unwrap();
    let contract = StrictPolicyContract::parse(&snap.policy_json_bytes).unwrap();
    let calls = Arc::new(AtomicUsize::new(0));
    let prepared_b = PreparedPolicy::from_backend(
        snap,
        contract,
        Box::new(SlowBackend { calls: calls.clone() }),
        &cfg,
    )
    .unwrap();
    let sup_b = Arc::new(ArmSupervisor::new(cfg));
    let gen_b = sup_b.acquire_lease("conn-dl-b").await.unwrap();
    sup_b.set_torque(true).await.unwrap();
    sup_b.tick(0.02).await;
    let epoch_b = sup_b.get_observation().await.epoch_id;
    let id_b = {
        let mut inner = sup_b.inner().write().await;
        inner
            .begin_policy_run(PolicyRunStart {
                epoch_id: epoch_b,
                connection_id: "conn-dl-b".into(),
                lease_generation: gen_b,
                policy_id: "p_dl_b".into(),
                artifact_digest: prepared_b.artifact_digest().to_string(),
                backend: prepared_b.backend_name().to_string(),
                mode: PolicyMode::Live,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: false,
                verified_release_gate: None,
            })
            .unwrap()
    };
    // Deadline is in the future at first check (~0ms), then expires while infer() sleeps 60ms.
    let tight_deadline = Some(Instant::now() + Duration::from_millis(20));
    let runner_b = spawn_policy_runner(sup_b.clone(), id_b, prepared_b, 25.0, tight_deadline, None, None);
    let _ = runner_b.await;

    assert_eq!(calls.load(Ordering::SeqCst), 1, "infer() must have run once before post-inference deadline check");
    let inner_b = sup_b.inner().read().await;
    assert!(inner_b.servo_target.is_none(), "post-inference deadline check must not set servo_target");
    assert_eq!(inner_b.last_action.as_ref().unwrap().state, "done");
    let pol_b = inner_b.policy_observation().unwrap();
    assert_eq!(pol_b.stop_reason.as_deref(), Some("MAX_SECONDS"));
    assert_eq!(pol_b.steps, 0);
}

#[tokio::test]
async fn deadline_races_tick_wait_and_slow_inference_wait() {
    let dir = tempdir().unwrap();
    let layers = vec![MlpLayerSpec::new(
        6,
        6,
        Activation::None,
        vec![0.0; 36],
        vec![0.05, 0.0, 0.0, 0.0, 0.0, 0.0],
    )];
    write_rmlp_pack(
        dir.path(),
        &["j1", "j2", "j3", "j4", "j5", "j6"],
        "rad",
        &layers,
        serde_json::json!({}),
    );

    let cfg = make_6dof_config();

    // Part A: 1 Hz shadow run with max_seconds = 0.05 finishes within ~150 ms (not 1 second!).
    let prepared_a = prepare_policy(dir.path(), &cfg).unwrap();
    let sup_a = Arc::new(ArmSupervisor::new(cfg.clone()));
    let gen_a = sup_a.acquire_lease("conn-race-tick").await.unwrap();
    let epoch_a = sup_a.get_observation().await.epoch_id;
    let id_a = {
        let mut inner = sup_a.inner().write().await;
        inner
            .begin_policy_run(PolicyRunStart {
                epoch_id: epoch_a,
                connection_id: "conn-race-tick".into(),
                lease_generation: gen_a,
                policy_id: "p_race_tick".into(),
                artifact_digest: prepared_a.artifact_digest().to_string(),
                backend: prepared_a.backend_name().to_string(),
                mode: PolicyMode::Shadow,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: false,
                verified_release_gate: None,
            })
            .unwrap()
    };
    let t0_a = Instant::now();
    let runner_a = spawn_policy_runner(
        sup_a.clone(),
        id_a,
        prepared_a,
        1.0,
        compute_policy_deadline(Some(0.05)).unwrap(),
        None,
        None,
    );
    let _ = runner_a.await;
    let elapsed_a = t0_a.elapsed();
    assert!(
        elapsed_a < Duration::from_millis(150),
        "1 Hz shadow run with max_seconds=0.05 took {:?}, expected < 150ms",
        elapsed_a
    );
    {
        let inner = sup_a.inner().read().await;
        let pol = inner.policy_observation().unwrap();
        assert!(!pol.active);
        assert_eq!(pol.stop_reason.as_deref(), Some("MAX_SECONDS"));
    }

    // Part B: slow inference (200 ms) with max_seconds = 0.04 finishes near 40 ms with MAX_SECONDS and steps == 0.
    struct VerySlowBackend;
    impl PolicyBackend for VerySlowBackend {
        fn backend_name(&self) -> &'static str { "mock-very-slow" }
        fn input_dim(&self) -> usize { 6 }
        fn output_dim(&self) -> usize { 6 }
        fn infer(&mut self, _input: &[f32], output: &mut [f32]) -> Result<(), PolicyError> {
            std::thread::sleep(Duration::from_millis(200));
            output.copy_from_slice(&[0.05, 0.0, 0.0, 0.0, 0.0, 0.0]);
            Ok(())
        }
    }

    let snap = PolicySnapshot::load_from_dir(dir.path()).unwrap();
    let contract = StrictPolicyContract::parse(&snap.policy_json_bytes).unwrap();
    let prepared_b = PreparedPolicy::from_backend(snap, contract, Box::new(VerySlowBackend), &cfg).unwrap();
    let sup_b = Arc::new(ArmSupervisor::new(cfg));
    let gen_b = sup_b.acquire_lease("conn-race-infer").await.unwrap();
    sup_b.set_torque(true).await.unwrap();
    sup_b.tick(0.02).await;
    let epoch_b = sup_b.get_observation().await.epoch_id;
    let id_b = {
        let mut inner = sup_b.inner().write().await;
        inner
            .begin_policy_run(PolicyRunStart {
                epoch_id: epoch_b,
                connection_id: "conn-race-infer".into(),
                lease_generation: gen_b,
                policy_id: "p_race_infer".into(),
                artifact_digest: prepared_b.artifact_digest().to_string(),
                backend: prepared_b.backend_name().to_string(),
                mode: PolicyMode::Live,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: false,
                verified_release_gate: None,
            })
            .unwrap()
    };
    let t0_b = Instant::now();
    let runner_b = spawn_policy_runner(
        sup_b.clone(),
        id_b,
        prepared_b,
        25.0,
        compute_policy_deadline(Some(0.04)).unwrap(),
        None,
        None,
    );
    let _ = runner_b.await;
    let elapsed_b = t0_b.elapsed();
    assert!(
        elapsed_b < Duration::from_millis(120),
        "slow 200ms inference with max_seconds=0.04 took {:?}, expected near 40ms (< 120ms)",
        elapsed_b
    );
    {
        let inner = sup_b.inner().read().await;
        assert!(inner.servo_target.is_none());
        let pol = inner.policy_observation().unwrap();
        assert!(!pol.active);
        assert_eq!(pol.stop_reason.as_deref(), Some("MAX_SECONDS"));
        assert_eq!(pol.steps, 0);
    }
}

#[tokio::test]
async fn live_policy_start_rejected_while_arm_braking_and_succeeds_when_settled() {
    let cfg = make_6dof_config();
    let sup = Arc::new(ArmSupervisor::new(cfg));
    let gen = sup.acquire_lease("conn-brake").await.unwrap();
    sup.set_torque(true).await.unwrap();
    sup.tick(0.02).await;
    let epoch = sup.get_observation().await.epoch_id;

    // Drive nonzero velocity via servo + tick, then drop servo_target so the arm enters braking.
    {
        let mut inner = sup.inner().write().await;
        inner.servo(vec![0.25, 0.0, 0.0, 0.0, 0.0, 0.0], epoch).unwrap();
    }
    for _ in 0..3 {
        sup.tick(0.02).await;
    }
    {
        let mut inner = sup.inner().write().await;
        inner.servo_target = None;
        assert!(
            inner.commanded_velocities.iter().any(|v| v.abs() >= 1e-3),
            "expected nonzero braking velocity, got {:?}",
            inner.commanded_velocities
        );
        assert!(matches!(
            inner.check_policy_start_preconditions(PolicyMode::Live, epoch),
            Err(ArmError::Safety(SafetyError::MotionBusy))
        ));
        assert!(matches!(
            inner.begin_policy_run(PolicyRunStart {
                epoch_id: epoch,
                connection_id: "conn-brake".into(),
                lease_generation: gen,
                policy_id: "p_brake".into(),
                artifact_digest: "d_brake".into(),
                backend: "mlp".into(),
                mode: PolicyMode::Live,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: false,
                verified_release_gate: None,
            }),
            Err(ArmError::Safety(SafetyError::MotionBusy))
        ));
    }

    // Tick until braking settles commanded_velocities to 0.
    for _ in 0..100 {
        sup.tick(0.02).await;
        if sup
            .inner()
            .read()
            .await
            .commanded_velocities
            .iter()
            .all(|v| v.abs() < 1e-3)
        {
            break;
        }
    }

    let mut inner = sup.inner().write().await;
    assert!(
        inner.commanded_velocities.iter().all(|v| v.abs() < 1e-3),
        "velocities did not settle: {:?}",
        inner.commanded_velocities
    );
    inner
        .check_policy_start_preconditions(PolicyMode::Live, epoch)
        .expect("live policy start preconditions must pass once settled");
    let action_id = inner
        .begin_policy_run(PolicyRunStart {
            epoch_id: epoch,
            connection_id: "conn-brake".into(),
            lease_generation: gen,
            policy_id: "p_brake".into(),
            artifact_digest: "d_brake".into(),
            backend: "mlp".into(),
            mode: PolicyMode::Live,
            sim_only: false,
            max_first_step_rad: None,
            max_step_rad: None,
            tag: None,
            i_am_at_the_arm: false,
            verified_release_gate: None,
        })
        .expect("live policy start must succeed once velocities settle to 0");
    assert!(inner.is_policy_run_active(action_id));
}

#[tokio::test]
async fn ws_policy_load_does_not_block_heartbeats_stop_or_disconnect() {
    let sandbox_dir = tempdir().unwrap();
    let sandbox = Arc::new(SandboxState::new(sandbox_dir.path().to_path_buf()));
    sandbox.set_policy_load_delay_ms(220);
    let mut cfg = make_6dof_config();
    cfg.lease_timeout_ms = 140; // Shorter than 220ms load delay: proves heartbeats keep lease alive!
    let sup = Arc::new(ArmSupervisor::new(cfg));
    let router = create_router_with_sandbox(sup.clone(), OperatorAuth::disabled(), sandbox);
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr: SocketAddr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        axum::serve(listener, router).await.unwrap();
    });

    let tick_sup = sup.clone();
    let tick_stop = Arc::new(Mutex::new(false));
    let tick_stop_c = tick_stop.clone();
    let tick_task = tokio::spawn(async move {
        loop {
            if *tick_stop_c.lock().unwrap() {
                break;
            }
            tick_sup.tick(0.02).await;
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    });

    let client = reqwest::Client::new();
    upload_rmlp_pack(&client, addr, "p_nonblock", vec![0.05, 0.0, 0.0, 0.0, 0.0, 0.0], serde_json::json!({})).await;

    let (mut ws, _) = connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
    ws.send(Message::Text(
        serde_json::json!({"type": "acquire_lease", "client_id": "nb_ws"})
            .to_string()
            .into(),
    ))
    .await
    .unwrap();
    let gen = loop {
        let msg: serde_json::Value =
            serde_json::from_str(&ws.next().await.unwrap().unwrap().into_text().unwrap()).unwrap();
        if msg["type"] == "lease_granted" {
            break msg["generation"].as_u64().unwrap();
        }
    };

    // Enable torque.
    ws.send(Message::Text(
        serde_json::json!({
            "type": "command", "request_id": 1, "generation": gen, "epoch_id": 1,
            "path": "/api/v1/torque", "body": {"enable": true}
        })
        .to_string()
        .into(),
    ))
    .await
    .unwrap();
    let _ = loop {
        let v: serde_json::Value = serde_json::from_str(&ws.next().await.unwrap().unwrap().into_text().unwrap()).unwrap();
        if v["request_id"] == 1 { break v; }
    };

    // Case 1: Start policy (220ms load), send heartbeat + /api/v1/motion/stop immediately.
    // Stop ack must return well before 220ms, and pending policy start must be rejected with STOPPED.
    let t0 = Instant::now();
    ws.send(Message::Text(
        serde_json::json!({
            "type": "command", "request_id": 2, "generation": gen, "epoch_id": 1,
            "path": "/api/v1/action/policy/start",
            "body": {"policy_id": "p_nonblock", "mode": "live", "fps": 20.0}
        })
        .to_string()
        .into(),
    ))
    .await
    .unwrap();
    ws.send(Message::Text(
        serde_json::json!({"type": "heartbeat", "generation": gen, "epoch_id": 1})
            .to_string()
            .into(),
    ))
    .await
    .unwrap();
    ws.send(Message::Text(
        serde_json::json!({
            "type": "command", "request_id": 3, "generation": gen, "epoch_id": 1,
            "path": "/api/v1/motion/stop", "body": {}
        })
        .to_string()
        .into(),
    ))
    .await
    .unwrap();

    let mut stop_ack_elapsed = None;
    let mut start_reply = None;
    let deadline = Instant::now() + Duration::from_secs(2);
    let mut next_hb = Instant::now() + Duration::from_millis(45);
    while Instant::now() < deadline && (stop_ack_elapsed.is_none() || start_reply.is_none()) {
        if Instant::now() >= next_hb {
            ws.send(Message::Text(
                serde_json::json!({"type": "heartbeat", "generation": gen, "epoch_id": 1})
                    .to_string()
                    .into(),
            ))
            .await
            .unwrap();
            next_hb = Instant::now() + Duration::from_millis(45);
        }
        let step_timeout = next_hb.saturating_duration_since(Instant::now()).max(Duration::from_millis(5));
        if let Ok(Some(Ok(msg))) = tokio::time::timeout(step_timeout, ws.next()).await {
            if let Ok(txt) = msg.into_text() {
                if let Ok(v) = serde_json::from_str::<serde_json::Value>(&txt) {
                    if v["request_id"] == 3 {
                        assert_eq!(v["type"], "command_ack", "{v}");
                        stop_ack_elapsed = Some(t0.elapsed());
                    } else if v["request_id"] == 2 {
                        start_reply = Some(v);
                    }
                }
            }
        }
    }
    let elapsed = stop_ack_elapsed.expect("stop ack received");
    assert!(
        elapsed < Duration::from_millis(120),
        "stop ack took {:?}, WS loop was blocked by policy load!",
        elapsed
    );
    let r_start = start_reply.expect("start reply received");
    assert_eq!(r_start["type"], "command_reject", "{r_start}");
    assert_eq!(r_start["code"], "STOPPED", "{r_start}");
    assert!(sup.inner().read().await.policy_run.is_none());

    // Case 2: Start policy again, send heartbeats every 40ms while 220ms load runs (> 140ms watchdog!).
    // Heartbeats keep lease alive and policy starts cleanly.
    ws.send(Message::Text(
        serde_json::json!({
            "type": "command", "request_id": 4, "generation": gen, "epoch_id": 1,
            "path": "/api/v1/action/policy/start",
            "body": {"policy_id": "p_nonblock", "mode": "shadow", "fps": 20.0, "max_seconds": 0.2}
        })
        .to_string()
        .into(),
    ))
    .await
    .unwrap();

    let mut r4 = None;
    for _ in 0..8 {
        ws.send(Message::Text(
            serde_json::json!({"type": "heartbeat", "generation": gen, "epoch_id": 1})
                .to_string()
                .into(),
        ))
        .await
        .unwrap();
        let wait_until = Instant::now() + Duration::from_millis(45);
        while Instant::now() < wait_until {
            if let Ok(Some(Ok(msg))) = tokio::time::timeout(Duration::from_millis(45), ws.next()).await {
                if let Ok(txt) = msg.into_text() {
                    if let Ok(v) = serde_json::from_str::<serde_json::Value>(&txt) {
                        if v["request_id"] == 4 {
                            r4 = Some(v);
                            break;
                        }
                    }
                }
            }
        }
        if r4.is_some() {
            break;
        }
    }
    let r4 = r4.expect("policy start reply received");
    assert_eq!(r4["type"], "command_ack", "heartbeats during load must prevent watchdog timeout: {r4}");

    // Wait for shadow max_seconds to finish.
    tokio::time::sleep(Duration::from_millis(260)).await;
    drop(ws);
    tokio::time::sleep(Duration::from_millis(50)).await;

    // Case 3: Start policy on a new WS and disconnect immediately while loading -> must NOT install PolicyRun.
    let (mut ws2, _) = connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
    ws2.send(Message::Text(
        serde_json::json!({"type": "acquire_lease", "client_id": "nb_ws_disc"})
            .to_string()
            .into(),
    ))
    .await
    .unwrap();
    let gen2 = loop {
        let msg: serde_json::Value =
            serde_json::from_str(&ws2.next().await.unwrap().unwrap().into_text().unwrap()).unwrap();
        if msg["type"] == "lease_granted" {
            break msg["generation"].as_u64().unwrap();
        }
    };
    let epoch2 = sup.get_observation().await.epoch_id;
    ws2.send(Message::Text(
        serde_json::json!({
            "type": "command", "request_id": 1, "generation": gen2, "epoch_id": epoch2,
            "path": "/api/v1/action/policy/start",
            "body": {"policy_id": "p_nonblock", "mode": "shadow", "fps": 20.0}
        })
        .to_string()
        .into(),
    ))
    .await
    .unwrap();
    tokio::time::sleep(Duration::from_millis(30)).await;
    drop(ws2);
    tokio::time::sleep(Duration::from_millis(260)).await;
    assert!(sup.inner().read().await.policy_run.is_none(), "disconnected socket must not install PolicyRun");

    *tick_stop.lock().unwrap() = true;
    let _ = tick_task.await;
}

#[tokio::test]
async fn overlapping_policy_load_is_rejected_with_motion_busy() {
    // Part A: Supervisor-level preconditions, begin_policy_run, and servo reject while policy_loading == true.
    let cfg = make_6dof_config();
    let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
    let gen = sup.acquire_lease("conn-load").await.unwrap();
    sup.set_torque(true).await.unwrap();
    sup.tick(0.02).await;
    let epoch = sup.get_observation().await.epoch_id;

    {
        let mut inner = sup.inner().write().await;
        inner.policy_loading = true;
        assert!(matches!(
            inner.check_policy_start_preconditions(PolicyMode::Shadow, epoch),
            Err(ArmError::Safety(SafetyError::MotionBusy))
        ));
        assert!(matches!(
            inner.check_policy_start_preconditions(PolicyMode::Live, epoch),
            Err(ArmError::Safety(SafetyError::MotionBusy))
        ));
        assert!(matches!(
            inner.begin_policy_run(PolicyRunStart {
                epoch_id: epoch,
                connection_id: "conn-load".into(),
                lease_generation: gen,
                policy_id: "p_overlap".into(),
                artifact_digest: "d".into(),
                backend: "mlp".into(),
                mode: PolicyMode::Shadow,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: false,
                verified_release_gate: None,
            }),
            Err(ArmError::Safety(SafetyError::MotionBusy))
        ));
        assert!(matches!(
            inner.servo(vec![0.0; 6], epoch),
            Err(SafetyError::MotionBusy)
        ));
        inner.policy_loading = false;
    }

    // Part B: Concurrent WS policy starts reject the second with MOTION_BUSY and always clear policy_loading.
    let sandbox_dir = tempdir().unwrap();
    let sandbox = Arc::new(SandboxState::new(sandbox_dir.path().to_path_buf()));
    sandbox.set_policy_load_delay_ms(220);
    let sup_ws = Arc::new(ArmSupervisor::new(cfg));
    let router = create_router_with_sandbox(sup_ws.clone(), OperatorAuth::disabled(), sandbox);
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr: SocketAddr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        axum::serve(listener, router).await.unwrap();
    });

    let client = reqwest::Client::new();
    upload_rmlp_pack(&client, addr, "p_overlap_ws", vec![0.0; 6], serde_json::json!({})).await;

    let (mut ws, _) = connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
    ws.send(Message::Text(
        serde_json::json!({"type": "acquire_lease", "client_id": "overlap_ws"})
            .to_string()
            .into(),
    ))
    .await
    .unwrap();
    let gen_ws = loop {
        let msg: serde_json::Value =
            serde_json::from_str(&ws.next().await.unwrap().unwrap().into_text().unwrap()).unwrap();
        if msg["type"] == "lease_granted" {
            break msg["generation"].as_u64().unwrap();
        }
    };

    // First start: missing policy fails after 220ms; verify policy_loading is cleared on error exit path too.
    ws.send(Message::Text(
        serde_json::json!({
            "type": "command", "request_id": 1, "generation": gen_ws, "epoch_id": 1,
            "path": "/api/v1/action/policy/start",
            "body": {"policy_id": "missing_pack", "mode": "shadow", "fps": 10.0}
        })
        .to_string()
        .into(),
    ))
    .await
    .unwrap();
    // Overlapping second start while first is loading -> immediately rejected with MOTION_BUSY.
    ws.send(Message::Text(
        serde_json::json!({
            "type": "command", "request_id": 2, "generation": gen_ws, "epoch_id": 1,
            "path": "/api/v1/action/policy/start",
            "body": {"policy_id": "p_overlap_ws", "mode": "shadow", "fps": 10.0}
        })
        .to_string()
        .into(),
    ))
    .await
    .unwrap();

    let mut r1 = None;
    let mut r2 = None;
    let deadline = Instant::now() + Duration::from_secs(2);
    while Instant::now() < deadline && (r1.is_none() || r2.is_none()) {
        if let Ok(Some(Ok(msg))) = tokio::time::timeout(Duration::from_millis(500), ws.next()).await {
            if let Ok(txt) = msg.into_text() {
                if let Ok(v) = serde_json::from_str::<serde_json::Value>(&txt) {
                    if v["request_id"] == 1 {
                        r1 = Some(v);
                    } else if v["request_id"] == 2 {
                        r2 = Some(v);
                    }
                }
            }
        }
    }
    let r2 = r2.expect("overlapping start reply");
    assert_eq!(r2["type"], "command_reject", "{r2}");
    assert_eq!(r2["code"], "MOTION_BUSY", "{r2}");
    let r1 = r1.expect("missing pack reply");
    assert_eq!(r1["type"], "command_reject", "{r1}");
    assert_eq!(r1["code"], "POLICY_NOT_FOUND", "{r1}");
    assert!(!sup_ws.inner().read().await.policy_loading, "policy_loading must be cleared after failed load");

    // Subsequent valid start succeeds and overlapping start is also rejected with MOTION_BUSY.
    ws.send(Message::Text(
        serde_json::json!({
            "type": "command", "request_id": 3, "generation": gen_ws, "epoch_id": 1,
            "path": "/api/v1/action/policy/start",
            "body": {"policy_id": "p_overlap_ws", "mode": "shadow", "fps": 10.0}
        })
        .to_string()
        .into(),
    ))
    .await
    .unwrap();
    // Wait briefly so request 3 has set policy_loading = true, then send request 4 on ws.
    tokio::time::sleep(Duration::from_millis(30)).await;
    assert!(sup_ws.inner().read().await.policy_loading);
    ws.send(Message::Text(
        serde_json::json!({
            "type": "command", "request_id": 4, "generation": gen_ws, "epoch_id": 1,
            "path": "/api/v1/action/policy/start",
            "body": {"policy_id": "p_overlap_ws", "mode": "shadow", "fps": 10.0}
        })
        .to_string()
        .into(),
    ))
    .await
    .unwrap();

    let mut r3 = None;
    let mut r4 = None;
    let deadline2 = Instant::now() + Duration::from_secs(2);
    while Instant::now() < deadline2 && (r3.is_none() || r4.is_none()) {
        if let Ok(Some(Ok(msg))) = tokio::time::timeout(Duration::from_millis(500), ws.next()).await {
            if let Ok(txt) = msg.into_text() {
                if let Ok(v) = serde_json::from_str::<serde_json::Value>(&txt) {
                    if v["request_id"] == 3 {
                        r3 = Some(v);
                    } else if v["request_id"] == 4 {
                        r4 = Some(v);
                    }
                }
            }
        }
    }
    let r4 = r4.expect("overlapping start #4 reply");
    assert_eq!(r4["type"], "command_reject", "{r4}");
    assert_eq!(r4["code"], "MOTION_BUSY", "{r4}");
    let r3 = r3.expect("valid start #3 reply");
    assert_eq!(r3["type"], "command_ack", "{r3}");
    assert!(!sup_ws.inner().read().await.policy_loading, "policy_loading must be cleared after successful start");
}

#[tokio::test]
async fn reciprocal_gravity_comp_and_policy_exclusion() {
    use armd::driver::JointMode;

    struct MitCapableDriver {
        q: [f32; 7],
        modes: [JointMode; 7],
        torqued: bool,
    }
    impl ArmDriver for MitCapableDriver {
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
        fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); 7] }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            Ok((0..7)
                .map(|i| JointState {
                    position: self.q[i],
                    velocity: 0.0,
                    torque: 0.0,
                    temperature: 30.0,
                })
                .collect())
        }
        fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> {
            Ok(())
        }
        fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
            self.torqued = enabled;
            if enabled {
                self.modes = [JointMode::Position; 7];
            }
            Ok(())
        }
        fn supports_mit_mode_switch(&self) -> bool { true }
        fn set_joint_modes(&mut self, modes: &[JointMode], _hold: &[JointTarget]) -> Result<(), ArmDriverError> {
            for (i, &m) in modes.iter().enumerate().take(7) {
                self.modes[i] = m;
            }
            Ok(())
        }
    }

    let cfg = ArmConfig::rebot_dm();
    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(MitCapableDriver {
        q: [0.0, 0.0, 0.0, 0.0, 0.0, std::f32::consts::FRAC_PI_2, 0.0],
        modes: [JointMode::Position; 7],
        torqued: false,
    })));
    sup.set_follower("sim").await;
    let gen = sup.acquire_lease("conn-gc").await.unwrap();
    sup.set_torque(true).await.unwrap();
    for _ in 0..5 {
        let ep = sup.get_observation().await.epoch_id;
        let _ = sup.heartbeat("conn-gc", ep).await;
        sup.tick(0.02).await;
    }
    let epoch = sup.get_observation().await.epoch_id;
    assert!(sup.inner().read().await.gravity_comp_blocker().is_none());

    // 1. Active Shadow policy blocks gravity comp entry.
    let action_id = {
        let mut inner = sup.inner().write().await;
        let id = inner
            .begin_policy_run(PolicyRunStart {
                epoch_id: epoch,
                connection_id: "conn-gc".into(),
                lease_generation: gen,
                policy_id: "p_gc_shadow".into(),
                artifact_digest: "d_gc".into(),
                backend: "mlp".into(),
                mode: PolicyMode::Shadow,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: false,
                verified_release_gate: None,
            })
            .unwrap();
        let blocker = inner.gravity_comp_blocker().expect("shadow policy must block gravity_comp");
        assert_eq!(blocker.code, "MOTION_BUSY");
        let err = inner.enter_gravity_comp(Default::default()).unwrap_err();
        assert!(
            matches!(err, ArmError::Rejected { ref code, .. } if code == "MOTION_BUSY"),
            "expected MOTION_BUSY, got {err:?}"
        );
        id
    };

    // Stop Shadow policy; policy_loading alone must also block gravity comp.
    {
        let mut inner = sup.inner().write().await;
        assert!(inner.stop_policy_run(Some(action_id), "STOPPED"));
        assert!(inner.gravity_comp_blocker().is_none());

        inner.policy_loading = true;
        let blocker = inner.gravity_comp_blocker().expect("policy_loading must block gravity_comp");
        assert_eq!(blocker.code, "MOTION_BUSY");
        let err = inner.enter_gravity_comp(Default::default()).unwrap_err();
        assert!(
            matches!(err, ArmError::Rejected { ref code, .. } if code == "MOTION_BUSY"),
            "expected MOTION_BUSY, got {err:?}"
        );
        inner.policy_loading = false;
    }

    // 2. Active gravity comp blocks starting either Shadow or Live policy.
    {
        let mut inner = sup.inner().write().await;
        inner.enter_gravity_comp(Default::default()).unwrap();
        assert!(matches!(
            inner.check_policy_start_preconditions(PolicyMode::Shadow, epoch),
            Err(ArmError::Safety(SafetyError::MotionBusy))
        ));
        assert!(matches!(
            inner.check_policy_start_preconditions(PolicyMode::Live, epoch),
            Err(ArmError::Safety(SafetyError::MotionBusy))
        ));
        assert!(matches!(
            inner.begin_policy_run(PolicyRunStart {
                epoch_id: epoch,
                connection_id: "conn-gc".into(),
                lease_generation: gen,
                policy_id: "p_while_gc".into(),
                artifact_digest: "d_gc2".into(),
                backend: "mlp".into(),
                mode: PolicyMode::Shadow,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: false,
                verified_release_gate: None,
            }),
            Err(ArmError::Safety(SafetyError::MotionBusy))
        ));
        inner.exit_gravity_comp("REQUESTED", "test");
    }
}

#[tokio::test(flavor = "current_thread")]
async fn slow_inference_updates_input_age_ms_and_does_not_block_current_thread_runtime() {
    use std::sync::atomic::AtomicBool;

    let dir = tempdir().unwrap();
    let layers = vec![MlpLayerSpec::new(6, 6, Activation::None, vec![0.0; 36], vec![0.0; 6])];
    write_rmlp_pack(
        dir.path(),
        &["j1", "j2", "j3", "j4", "j5", "j6"],
        "rad",
        &layers,
        serde_json::json!({}),
    );

    struct SlowBlockingBackend {
        calls: Arc<AtomicUsize>,
        infer_entered: Arc<AtomicBool>,
        timed_out: Arc<AtomicBool>,
        unblock_rx: Option<std::sync::mpsc::Receiver<()>>,
        sleep_ms: u64,
    }
    impl PolicyBackend for SlowBlockingBackend {
        fn backend_name(&self) -> &'static str { "mock-slow-blocking" }
        fn input_dim(&self) -> usize { 6 }
        fn output_dim(&self) -> usize { 6 }
        fn infer(&mut self, _input: &[f32], output: &mut [f32]) -> Result<(), PolicyError> {
            let call = self.calls.fetch_add(1, Ordering::SeqCst);
            std::thread::sleep(Duration::from_millis(self.sleep_ms));
            if call == 0 {
                self.infer_entered.store(true, Ordering::SeqCst);
                if let Some(rx) = self.unblock_rx.take() {
                    if rx.recv_timeout(Duration::from_secs(2)).is_err() {
                        self.timed_out.store(true, Ordering::SeqCst);
                    }
                }
            }
            output.copy_from_slice(&[0.05, 0.0, 0.0, 0.0, 0.0, 0.0]);
            Ok(())
        }
    }

    let cfg = make_6dof_config();
    let snap = PolicySnapshot::load_from_dir(dir.path()).unwrap();
    let contract = StrictPolicyContract::parse(&snap.policy_json_bytes).unwrap();
    let calls = Arc::new(AtomicUsize::new(0));
    let infer_entered = Arc::new(AtomicBool::new(false));
    let timed_out = Arc::new(AtomicBool::new(false));
    let (unblock_tx, unblock_rx) = std::sync::mpsc::channel();
    let prepared = PreparedPolicy::from_backend(
        snap.clone(),
        contract.clone(),
        Box::new(SlowBlockingBackend {
            calls: calls.clone(),
            infer_entered: infer_entered.clone(),
            timed_out: timed_out.clone(),
            unblock_rx: Some(unblock_rx),
            sleep_ms: 30,
        }),
        &cfg,
    )
    .unwrap();

    let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
    let gen = sup.acquire_lease("conn-slow-ct").await.unwrap();
    let epoch = sup.get_observation().await.epoch_id;
    let action_id = {
        let mut inner = sup.inner().write().await;
        inner
            .begin_policy_run(PolicyRunStart {
                epoch_id: epoch,
                connection_id: "conn-slow-ct".into(),
                lease_generation: gen,
                policy_id: "p_slow_ct".into(),
                artifact_digest: prepared.artifact_digest().to_string(),
                backend: prepared.backend_name().to_string(),
                mode: PolicyMode::Shadow,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: false,
                verified_release_gate: None,
            })
            .unwrap()
    };

    let runner = spawn_policy_runner(
        sup.clone(),
        action_id,
        prepared,
        20.0,
        compute_policy_deadline(Some(0.25)).unwrap(),
        None,
        None,
    );

    // Wait on the single-threaded Tokio executor until infer() is active, prove the event loop
    // still ticks asynchronously, and unblock infer(). If step_infer ran on the Tokio worker,
    // this loop could not run while infer() waits on unblock_rx.
    while !infer_entered.load(Ordering::SeqCst) {
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
    tokio::time::sleep(Duration::from_millis(5)).await;
    unblock_tx.send(()).unwrap();

    let _ = runner.await;
    assert!(!timed_out.load(Ordering::SeqCst), "infer blocked single-threaded Tokio runtime");
    assert!(calls.load(Ordering::SeqCst) >= 1);

    let obs = sup.get_observation().await;
    let pol = obs.policy.expect("policy observation present");
    assert!(pol.steps > 0, "expected at least one completed step");
    assert!(
        pol.last_input_age_ms.is_some_and(|age| age >= 25),
        "expected last_input_age_ms >= 25 (inference latency included), got {:?}",
        pol.last_input_age_ms
    );

    // Also verify None base input age stays None even when inference takes >= 25ms.
    struct UnknownAgeDriver;
    impl ArmDriver for UnknownAgeDriver {
        fn capabilities(&self) -> DriverCapabilities {
            DriverCapabilities {
                velocity_feedback: true,
                torque_feedback: false,
                temperature_feedback: true,
                torque_feedforward: false,
                enable_feedback: true,
                load_feedback: false,
            }
        }
        fn enable_state(&self) -> EnableState { EnableState::Disabled }
        fn is_feedback_valid(&self) -> bool { true }
        fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![None; 6] }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            Ok(vec![JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 30.0 }; 6])
        }
        fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
        fn set_torque(&mut self, _enabled: bool) -> Result<(), ArmDriverError> { Ok(()) }
    }

    let prepared_none = PreparedPolicy::from_backend(
        snap,
        contract,
        Box::new(SlowBlockingBackend {
            calls: Arc::new(AtomicUsize::new(0)),
            infer_entered: Arc::new(AtomicBool::new(false)),
            timed_out: Arc::new(AtomicBool::new(false)),
            unblock_rx: None,
            sleep_ms: 30,
        }),
        &cfg,
    )
    .unwrap();
    let sup_none = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(UnknownAgeDriver)));
    let gen_none = sup_none.acquire_lease("conn-none-age").await.unwrap();
    let epoch_none = sup_none.get_observation().await.epoch_id;
    let action_none = {
        let mut inner = sup_none.inner().write().await;
        inner
            .begin_policy_run(PolicyRunStart {
                epoch_id: epoch_none,
                connection_id: "conn-none-age".into(),
                lease_generation: gen_none,
                policy_id: "p_none_age".into(),
                artifact_digest: prepared_none.artifact_digest().to_string(),
                backend: prepared_none.backend_name().to_string(),
                mode: PolicyMode::Shadow,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: false,
                verified_release_gate: None,
            })
            .unwrap()
    };
    let runner_none = spawn_policy_runner(
        sup_none.clone(),
        action_none,
        prepared_none,
        20.0,
        compute_policy_deadline(Some(0.15)).unwrap(),
        None,
        None,
    );
    let _ = runner_none.await;
    let pol_none = sup_none.get_observation().await.policy.unwrap();
    assert!(pol_none.steps > 0);
    assert_eq!(pol_none.last_input_age_ms, None, "None base input age must stay None");
}

#[cfg(feature = "policy-tract")]
mod tract_e2e {
    use super::*;
    use armd::policy_runtime::MlpBackend;

    fn pb_varint(mut v: u64, out: &mut Vec<u8>) {
        loop {
            let b = (v & 0x7F) as u8;
            v >>= 7;
            if v == 0 {
                out.push(b);
                break;
            }
            out.push(b | 0x80);
        }
    }
    fn pb_int(field: u64, v: u64, out: &mut Vec<u8>) {
        pb_varint(field << 3, out);
        pb_varint(v, out);
    }
    fn pb_bytes(field: u64, data: &[u8], out: &mut Vec<u8>) {
        pb_varint((field << 3) | 2, out);
        pb_varint(data.len() as u64, out);
        out.extend_from_slice(data);
    }
    fn pb_str(field: u64, s: &str, out: &mut Vec<u8>) {
        pb_bytes(field, s.as_bytes(), out);
    }
    fn make_dim(d: i64) -> Vec<u8> {
        let mut out = Vec::new();
        pb_int(1, d as u64, &mut out);
        out
    }
    fn make_value_info(name: &str, dims: &[Vec<u8>]) -> Vec<u8> {
        let mut shape = Vec::new();
        for d in dims {
            pb_bytes(1, d, &mut shape);
        }
        let mut tt = Vec::new();
        pb_int(1, 1, &mut tt);
        pb_bytes(2, &shape, &mut tt);
        let mut tp = Vec::new();
        pb_bytes(1, &tt, &mut tp);
        let mut vi = Vec::new();
        pb_str(1, name, &mut vi);
        pb_bytes(2, &tp, &mut vi);
        vi
    }
    fn make_f32_tensor(name: &str, dims: &[i64], data: &[f32]) -> Vec<u8> {
        let mut t = Vec::new();
        for &d in dims {
            pb_int(1, d as u64, &mut t);
        }
        pb_int(2, 1, &mut t);
        pb_str(8, name, &mut t);
        let mut raw = Vec::with_capacity(data.len() * 4);
        for &v in data {
            raw.extend_from_slice(&v.to_le_bytes());
        }
        pb_bytes(9, &raw, &mut t);
        t
    }
    fn make_node(op: &str, inputs: &[&str], outputs: &[&str], name: &str) -> Vec<u8> {
        let mut n = Vec::new();
        for i in inputs {
            pb_str(1, i, &mut n);
        }
        for o in outputs {
            pb_str(2, o, &mut n);
        }
        pb_str(3, name, &mut n);
        pb_str(4, op, &mut n);
        n
    }
    fn transpose(w: &[f32], out_dim: usize, in_dim: usize) -> Vec<f32> {
        let mut wt = vec![0.0f32; in_dim * out_dim];
        for o in 0..out_dim {
            for i in 0..in_dim {
                wt[i * out_dim + o] = w[o * in_dim + i];
            }
        }
        wt
    }

    #[tokio::test]
    async fn tract_onnx_end_to_end() {
        let l1 = MlpLayerSpec::new(
            6,
            8,
            Activation::Relu,
            (0..48).map(|i| ((i as f32) * 0.05 - 1.0) * 0.1).collect(),
            vec![0.1, -0.05, 0.08, 0.12, -0.03, 0.07, -0.02, 0.04],
        );
        let l2 = MlpLayerSpec::new(
            8,
            6,
            Activation::Tanh,
            (0..48).map(|i| ((i as f32) * 0.07 - 1.2) * 0.05).collect(),
            vec![0.05, -0.04, 0.03, -0.02, 0.01, -0.06],
        );
        let mut ref_mlp = MlpBackend::from_layers(vec![l1.clone(), l2.clone()]).unwrap();
        let mut expected_out = [0.0f32; 6];
        ref_mlp.infer(&[0.0; 6], &mut expected_out).unwrap();

        let t_w1 = make_f32_tensor("W1", &[6, 8], &transpose(&l1.weights, 8, 6));
        let t_b1 = make_f32_tensor("B1", &[8], &l1.bias);
        let t_w2 = make_f32_tensor("W2", &[8, 6], &transpose(&l2.weights, 6, 8));
        let t_b2 = make_f32_tensor("B2", &[6], &l2.bias);
        let nodes = [
            make_node("MatMul", &["obs", "W1"], &["mm1"], "mm1"),
            make_node("Add", &["mm1", "B1"], &["a1"], "a1"),
            make_node("Relu", &["a1"], &["h1"], "r1"),
            make_node("MatMul", &["h1", "W2"], &["mm2"], "mm2"),
            make_node("Add", &["mm2", "B2"], &["a2"], "a2"),
            make_node("Tanh", &["a2"], &["act"], "t2"),
        ];
        let vi_in = make_value_info("obs", &[make_dim(1), make_dim(6)]);
        let vi_out = make_value_info("act", &[make_dim(1), make_dim(6)]);
        let mut graph = Vec::new();
        for n in nodes {
            pb_bytes(1, &n, &mut graph);
        }
        pb_str(2, "mlp", &mut graph);
        for init in [t_w1, t_b1, t_w2, t_b2] {
            pb_bytes(5, &init, &mut graph);
        }
        pb_bytes(11, &vi_in, &mut graph);
        pb_bytes(12, &vi_out, &mut graph);
        let mut opset = Vec::new();
        pb_str(1, "", &mut opset);
        pb_int(2, 13, &mut opset);
        let mut onnx = Vec::new();
        pb_int(1, 8, &mut onnx);
        pb_bytes(7, &graph, &mut onnx);
        pb_bytes(8, &opset, &mut onnx);

        let dir = tempdir().unwrap();
        let manifest = serde_json::json!({
            "format": "rebot-onnx-policy/v1",
            "name": "tract_e2e",
            "unit_mode": "rad",
            "action_dim": 6,
            "action_space": "joint_positions",
            "joint_names": ["j1", "j2", "j3", "j4", "j5", "j6"],
            "model_file": "model.onnx"
        });
        fs::write(dir.path().join("policy.json"), serde_json::to_vec(&manifest).unwrap()).unwrap();
        fs::write(dir.path().join("model.onnx"), &onnx).unwrap();

        let cfg = make_6dof_config();
        let prepared = prepare_policy(dir.path(), &cfg).unwrap();
        assert_eq!(prepared.backend_name(), "tract-onnx");

        let sup = Arc::new(ArmSupervisor::new(cfg));
        let schema = sup.get_schema().await;
        let pr = schema.capabilities.policy_runtime.unwrap();
        assert!(pr.backends.contains(&"tract".to_string()));

        let gen = sup.acquire_lease("conn-tract").await.unwrap();
        let epoch = sup.get_observation().await.epoch_id;
        let action_id = {
            let mut inner = sup.inner().write().await;
            inner
                .begin_policy_run(PolicyRunStart {
                    epoch_id: epoch,
                    connection_id: "conn-tract".into(),
                    lease_generation: gen,
                    policy_id: "tract_e2e".into(),
                    artifact_digest: prepared.artifact_digest().to_string(),
                    backend: prepared.backend_name().to_string(),
                    mode: PolicyMode::Shadow,
                    sim_only: false,
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
                    i_am_at_the_arm: false,
                    verified_release_gate: None,
                })
                .unwrap()
        };

        let runner = spawn_policy_runner(
            sup.clone(),
            action_id,
            prepared,
            25.0,
            compute_policy_deadline(Some(0.15)).unwrap(),
            None,
            None,
        );
        let _ = runner.await;

        let obs = sup.get_observation().await;
        let p = obs.policy.unwrap();
        assert_eq!(p.backend, "tract-onnx");
        assert!(p.steps > 0);
        let got = p.last_shadow_target.unwrap();
        for i in 0..6 {
            assert!(
                (got[i] - expected_out[i]).abs() < 1e-5,
                "tract e2e mismatch at {i}: got={} expected={}",
                got[i],
                expected_out[i]
            );
        }
    }

    /// ONNX MatMul/Add(/Relu) chain for `dims` = [in, h1, ..., out]; `fill` sets every weight.
    pub(super) fn mlp_onnx(dims: &[usize], fill: f32) -> Vec<u8> {
        let mut graph = Vec::new();
        let mut inits = Vec::new();
        let mut prev = "obs".to_string();
        for (k, w) in dims.windows(2).enumerate() {
            let (i, o) = (w[0], w[1]);
            let (wn, bn, mm, ad) = (format!("W{k}"), format!("B{k}"), format!("mm{k}"), format!("a{k}"));
            inits.push(make_f32_tensor(&wn, &[i as i64, o as i64], &vec![fill; i * o]));
            inits.push(make_f32_tensor(&bn, &[o as i64], &vec![0.0; o]));
            pb_bytes(1, &make_node("MatMul", &[&prev, &wn], &[&mm], &mm), &mut graph);
            let last = k + 2 == dims.len();
            let out = if last { "act".to_string() } else { format!("h{k}") };
            if last {
                pb_bytes(1, &make_node("Add", &[&mm, &bn], &[&out], &ad), &mut graph);
            } else {
                pb_bytes(1, &make_node("Add", &[&mm, &bn], &[&ad], &ad), &mut graph);
                pb_bytes(1, &make_node("Relu", &[&ad], &[&out], &format!("r{k}")), &mut graph);
            }
            prev = out;
        }
        pb_str(2, "mlp", &mut graph);
        for t in inits {
            pb_bytes(5, &t, &mut graph);
        }
        pb_bytes(11, &make_value_info("obs", &[make_dim(1), make_dim(dims[0] as i64)]), &mut graph);
        pb_bytes(12, &make_value_info("act", &[make_dim(1), make_dim(*dims.last().unwrap() as i64)]), &mut graph);
        let mut opset = Vec::new();
        pb_str(1, "", &mut opset);
        pb_int(2, 13, &mut opset);
        let mut onnx = Vec::new();
        pb_int(1, 8, &mut onnx);
        pb_bytes(7, &graph, &mut onnx);
        pb_bytes(8, &opset, &mut onnx);
        onnx
    }

    fn budget_err(bytes: &[u8]) -> String {
        match armd::policy_runtime::TractBackend::from_onnx_bytes(bytes) {
            Err(PolicyError::ResourceBudget(m)) => m,
            Err(e) => panic!("expected ResourceBudget, got {e:?}"),
            Ok(_) => panic!("expected ResourceBudget, model loaded"),
        }
    }

    /// Item 5: tract budgets are enforced before tract parses / optimizes the model.
    #[test]
    fn tract_budgets_refuse_large_models_before_loading() {
        use armd::policy_runtime::manifest::{onnx_resource_budget, MAX_ONNX_NODES};
        // within budget: loads
        let ok = mlp_onnx(&[6, 32, 6], 0.01);
        let b = onnx_resource_budget(&ok).unwrap();
        assert_eq!(b.nodes, 5);
        assert_eq!(b.param_elements, 6 * 32 + 32 + 32 * 6 + 6);
        armd::policy_runtime::TractBackend::from_onnx_bytes(&ok).unwrap();

        // node count
        let deep: Vec<usize> = std::iter::once(6).chain(std::iter::repeat_n(6, MAX_ONNX_NODES / 3 + 2)).collect();
        assert!(budget_err(&mlp_onnx(&deep, 0.0)).contains("nodes"));

        // parameter elements: a tensor that declares 2001 x 1000 (data never read)
        let mut t = Vec::new();
        pb_int(1, 2001, &mut t);
        pb_int(1, 1000, &mut t);
        pb_int(2, 1, &mut t);
        pb_str(8, "W", &mut t);
        let mut graph = Vec::new();
        pb_bytes(1, &make_node("MatMul", &["obs", "W"], &["act"], "mm"), &mut graph);
        pb_bytes(5, &t, &mut graph);
        let mut onnx = Vec::new();
        pb_int(1, 8, &mut onnx);
        pb_bytes(7, &graph, &mut onnx);
        assert!(budget_err(&onnx).contains("initializers"));

        // subgraph attribute (If/Loop body)
        let mut attr = Vec::new();
        pb_str(1, "then_branch", &mut attr);
        pb_bytes(6, &[], &mut attr);
        let mut node = make_node("If", &["c"], &["act"], "if");
        pb_bytes(5, &attr, &mut node);
        let mut graph = Vec::new();
        pb_bytes(1, &node, &mut graph);
        let mut onnx = Vec::new();
        pb_int(1, 8, &mut onnx);
        pb_bytes(7, &graph, &mut onnx);
        assert!(budget_err(&onnx).contains("subgraphs"));

        // intermediate tensors: Expand [1, 6] -> [200000, 6] (1.2 M elements) with tiny params
        let mut shape_t = Vec::new();
        pb_int(1, 2, &mut shape_t);
        pb_int(2, 7, &mut shape_t); // INT64
        pb_str(8, "shape", &mut shape_t);
        let mut raw = Vec::new();
        raw.extend_from_slice(&200_000i64.to_le_bytes());
        raw.extend_from_slice(&6i64.to_le_bytes());
        pb_bytes(9, &raw, &mut shape_t);
        let mut axes = Vec::new();
        pb_str(1, "axes", &mut axes);
        pb_int(8, 0, &mut axes);
        pb_int(20, 7, &mut axes); // INTS
        let mut reduce = make_node("ReduceMean", &["big"], &["act"], "rm");
        pb_bytes(5, &axes, &mut reduce);
        let mut graph = Vec::new();
        pb_bytes(1, &make_node("Expand", &["obs", "shape"], &["big"], "ex"), &mut graph);
        pb_bytes(1, &reduce, &mut graph);
        pb_str(2, "wide", &mut graph);
        pb_bytes(5, &shape_t, &mut graph);
        pb_bytes(11, &make_value_info("obs", &[make_dim(1), make_dim(6)]), &mut graph);
        pb_bytes(12, &make_value_info("act", &[make_dim(1), make_dim(6)]), &mut graph);
        let mut opset = Vec::new();
        pb_str(1, "", &mut opset);
        pb_int(2, 13, &mut opset);
        let mut wide = Vec::new();
        pb_int(1, 8, &mut wide);
        pb_bytes(7, &graph, &mut wide);
        pb_bytes(8, &opset, &mut wide);
        assert!(budget_err(&wide).contains("intermediate"), "{}", budget_err(&wide));
    }

    /// Largest RMLP the device accepts: 1.98 M parameters, 7.9 MB.
    pub(super) const MAX_DIMS: [usize; 5] = [6, 1024, 1024, 900, 6];

    /// Host measurement of load + per-step time for the largest allowed model of each
    /// backend (run with --ignored under /usr/bin/time -v for peak RSS).
    #[test]
    #[ignore]
    fn bench_largest_model_tract() {
        let onnx = mlp_onnx(&MAX_DIMS, 0.001);
        println!("onnx bytes {}", onnx.len());
        assert!(onnx.len() <= armd::policy_runtime::MAX_MODEL_BYTES);
        let t0 = Instant::now();
        let mut be = armd::policy_runtime::TractBackend::from_onnx_bytes(&onnx).unwrap();
        println!("tract load {:?}", t0.elapsed());
        drop(onnx);
        let (inp, mut out) = ([0.1f32; 6], [0.0f32; 6]);
        be.infer(&inp, &mut out).unwrap();
        let t0 = Instant::now();
        for _ in 0..200 { be.infer(&inp, &mut out).unwrap(); }
        println!("tract step {:?}", t0.elapsed() / 200);
    }

    #[test]
    #[ignore]
    fn bench_largest_model_rmlp() {
        let layers: Vec<MlpLayerSpec> = MAX_DIMS.windows(2).enumerate().map(|(k, w)| {
            MlpLayerSpec::new(w[0], w[1], if k + 2 == MAX_DIMS.len() { Activation::None } else { Activation::Relu }, vec![0.001; w[0] * w[1]], vec![0.0; w[1]])
        }).collect();
        let bytes = encode_rmlp_bytes(&layers).unwrap();
        drop(layers);
        println!("rmlp bytes {}", bytes.len());
        assert!(bytes.len() <= armd::policy_runtime::MAX_MODEL_BYTES);
        let t0 = Instant::now();
        let mut be = MlpBackend::from_rmlp_bytes(&bytes).unwrap();
        println!("rmlp load {:?}", t0.elapsed());
        drop(bytes);
        let (inp, mut out) = ([0.1f32; 6], [0.0f32; 6]);
        be.infer(&inp, &mut out).unwrap();
        let t0 = Instant::now();
        for _ in 0..200 { be.infer(&inp, &mut out).unwrap(); }
        println!("rmlp step {:?}", t0.elapsed() / 200);
    }

    #[test]
    #[ignore]
    fn bench_baseline_harness() {
        println!("baseline");
    }
}

#[tokio::test]
async fn live_policy_rejects_stale_joint_measurement_from_slow_inference() {
    struct SlowInferBackend;

    impl PolicyBackend for SlowInferBackend {
        fn backend_name(&self) -> &'static str {
            "slow_infer_backend"
        }
        fn input_dim(&self) -> usize {
            6
        }
        fn output_dim(&self) -> usize {
            6
        }
        fn infer(&mut self, _input: &[f32], output: &mut [f32]) -> Result<(), PolicyError> {
            std::thread::sleep(Duration::from_millis(280));
            output.copy_from_slice(&[0.05, 0.0, 0.0, 0.0, 0.0, 0.0]);
            Ok(())
        }
    }

    let dir = tempdir().unwrap();
    let layers = vec![MlpLayerSpec::new(
        6,
        6,
        Activation::None,
        vec![0.0; 36],
        vec![0.0; 6],
    )];
    write_rmlp_pack(
        dir.path(),
        &["j1", "j2", "j3", "j4", "j5", "j6"],
        "rad",
        &layers,
        serde_json::json!({}),
    );
    let snap = PolicySnapshot::load_from_dir(dir.path()).unwrap();
    let contract = StrictPolicyContract::parse(&snap.policy_json_bytes).unwrap();
    let cfg = make_6dof_config();
    let prepared = PreparedPolicy::from_backend(snap, contract, Box::new(SlowInferBackend), &cfg).unwrap();

    let sup = Arc::new(ArmSupervisor::new(cfg));
    let gen = sup.acquire_lease("conn-slow-infer").await.unwrap();
    sup.set_torque(true).await.unwrap();
    sup.tick(0.02).await;
    let epoch = sup.get_observation().await.epoch_id;

    let action_id = {
        let mut inner = sup.inner().write().await;
        inner
            .begin_policy_run(PolicyRunStart {
                epoch_id: epoch,
                connection_id: "conn-slow-infer".into(),
                lease_generation: gen,
                policy_id: "slow_infer_live".into(),
                artifact_digest: prepared.artifact_digest().to_string(),
                backend: prepared.backend_name().to_string(),
                mode: PolicyMode::Live,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: false,
                verified_release_gate: None,
            })
            .unwrap()
    };

    let runner = spawn_policy_runner(sup.clone(), action_id, prepared, 25.0, None, None, None);
    let _ = runner.await;

    let inner = sup.inner().read().await;
    assert!(
        inner.servo_target.is_none(),
        "stale joint measurement from slow inference must not set servo_target"
    );
    assert_eq!(inner.last_action.as_ref().unwrap().state, "failed");
    let pol = inner.policy_observation().unwrap();
    assert!(!pol.active);
    assert_eq!(pol.steps, 0);
    assert_eq!(pol.stop_reason.as_deref(), Some("STALE_INPUT"));
}

mod phase2_vision {
    use super::*;
    use armd::api::create_router_full_with_sandbox;
    use armd::api::proxy::DetectorUpstream;
    use armd::calibration::CalibrationDeps;
    use armd::diagnostics::DiagnosticsState;
    use armd::grasp::{BoxFuture, Detection, DetectionFrame, DetectionSource, GraspDeps};
    use armd::policy_runtime::{
        load_backend, PolicyVisionFrame, PolicyVisionSource, VisionLayout, VisionSourceKind,
        VisionStalePolicy, VisionTargetLostPolicy, MAX_VISION_STALENESS_MS,
    };

    pub(super) fn make_detection(label: &str, score: f64, box_norm: [f64; 4]) -> Detection {
        Detection {
            label: label.to_string(),
            score,
            raw_score_valid: true,
            box_norm,
            angle_deg: None,
            angle_conf: None,
            grasp_width_px: None,
            mask_focus: false,
            disp_fg: None,
            disp_bg: None,
            depth_conf: None,
            length_px: None,
            jaw_clear: None,
            depth_angle_deg: None,
            depth_angle_conf: None,
            depth_width_px: None,
        }
    }

    pub(super) struct MockVisionSource {
        handler: Arc<
            dyn Fn(u64, u64) -> BoxFuture<'static, Result<Option<PolicyVisionFrame>, String>>
                + Send
                + Sync,
        >,
    }

    impl MockVisionSource {
        fn new<F>(f: F) -> Arc<Self>
        where
            F: Fn(u64, u64) -> Result<Option<PolicyVisionFrame>, String> + Send + Sync + 'static,
        {
            Arc::new(Self {
                handler: Arc::new(move |after, wait| {
                    let res = f(after, wait);
                    Box::pin(async move { res })
                }),
            })
        }

        pub(super) fn new_async<F>(f: F) -> Arc<Self>
        where
            F: Fn(u64, u64) -> BoxFuture<'static, Result<Option<PolicyVisionFrame>, String>>
                + Send
                + Sync
                + 'static,
        {
            Arc::new(Self {
                handler: Arc::new(f),
            })
        }
    }

    impl PolicyVisionSource for MockVisionSource {
        fn next_frame<'a>(
            &'a self,
            after_frame_id: u64,
            wait_ms: u64,
        ) -> BoxFuture<'a, Result<Option<PolicyVisionFrame>, String>> {
            (self.handler)(after_frame_id, wait_ms)
        }
    }

    /// Weights matrix [6 x 12]: output[i] = proprio[i] + 0.1 * vision[i]
    pub(super) fn make_12_to_6_layers() -> Vec<MlpLayerSpec> {
        let mut weights = vec![0.0f32; 6 * 12];
        for r in 0..6 {
            weights[r * 12 + r] = 1.0;
            weights[r * 12 + (6 + r)] = 0.1;
        }
        vec![MlpLayerSpec::new(
            12,
            6,
            Activation::None,
            weights,
            vec![0.0; 6],
        )]
    }

    #[test]
    fn vision_manifest_strict_validation() {
        let base_vision = serde_json::json!({
            "format": "rebot-onnx-policy/v1",
            "name": "vis_policy",
            "unit_mode": "rad",
            "action_dim": 6,
            "action_space": "joint_positions",
            "joint_names": ["j1", "j2", "j3", "j4", "j5", "j6"],
            "model_file": "model.rmlp",
            "vision": {
                "source": "detector",
                "layout": "top1_box_norm_v1",
                "max_staleness_ms": 200
            }
        });

        let c = StrictPolicyContract::parse(&serde_json::to_vec(&base_vision).unwrap()).unwrap();
        let vis = c.vision.as_ref().expect("vision spec parsed");
        assert_eq!(vis.source, VisionSourceKind::Detector);
        assert_eq!(vis.layout, VisionLayout::Top1BoxNormV1);
        assert_eq!(vis.target_label, None);
        assert!((vis.min_score - 0.3).abs() < 1e-6);
        assert_eq!(vis.max_staleness_ms, 200);
        assert_eq!(vis.max_sync_skew_ms, 200);
        assert_eq!(vis.on_target_lost, VisionTargetLostPolicy::Hold);
        assert_eq!(vis.on_stale, VisionStalePolicy::Stop);
        assert!(vis.require_model_generation);
        assert_eq!(c.vision_feature_dim(), 6);
        assert_eq!(c.expected_input_dim(), 12);

        // Explicit null vision keeps Phase 1 compatibility.
        let mut null_v = base_vision.clone();
        null_v["vision"] = serde_json::Value::Null;
        let c_null = StrictPolicyContract::parse(&serde_json::to_vec(&null_v).unwrap()).unwrap();
        assert!(c_null.vision.is_none());
        assert_eq!(c_null.vision_feature_dim(), 0);
        assert_eq!(c_null.expected_input_dim(), 6);

        // Full explicit overrides
        let mut full = base_vision.clone();
        full["vision"] = serde_json::json!({
            "source": "detector",
            "layout": "top1_box_norm_v1",
            "target_label": "cube",
            "min_score": 0.65,
            "max_staleness_ms": 500,
            "max_sync_skew_ms": 120,
            "on_target_lost": "zero_features",
            "on_stale": "hold",
            "require_model_generation": false
        });
        let c_full = StrictPolicyContract::parse(&serde_json::to_vec(&full).unwrap()).unwrap();
        let v_full = c_full.vision.unwrap();
        assert_eq!(v_full.target_label.as_deref(), Some("cube"));
        assert!((v_full.min_score - 0.65).abs() < 1e-6);
        assert_eq!(v_full.max_staleness_ms, MAX_VISION_STALENESS_MS);
        assert_eq!(v_full.max_sync_skew_ms, 120);
        assert_eq!(v_full.on_target_lost, VisionTargetLostPolicy::ZeroFeatures);
        assert_eq!(v_full.on_stale, VisionStalePolicy::Hold);
        assert!(!v_full.require_model_generation);

        // Reject invalid variants
        let cases: Vec<serde_json::Value> = vec![
            serde_json::json!("not_an_object"),
            serde_json::json!({"source": "camera", "layout": "top1_box_norm_v1", "max_staleness_ms": 200}),
            serde_json::json!({"source": "detector", "layout": "rgb_chw_v1", "max_staleness_ms": 200}),
            serde_json::json!({"source": "detector", "layout": "top1_box_norm_v1"}),
            serde_json::json!({"source": "detector", "layout": "top1_box_norm_v1", "max_staleness_ms": 0}),
            serde_json::json!({"source": "detector", "layout": "top1_box_norm_v1", "max_staleness_ms": 501}),
            serde_json::json!({"source": "detector", "layout": "top1_box_norm_v1", "max_staleness_ms": 200, "max_sync_skew_ms": 0}),
            serde_json::json!({"source": "detector", "layout": "top1_box_norm_v1", "max_staleness_ms": 200, "max_sync_skew_ms": 501}),
            serde_json::json!({"source": "detector", "layout": "top1_box_norm_v1", "max_staleness_ms": 200, "min_score": -0.01}),
            serde_json::json!({"source": "detector", "layout": "top1_box_norm_v1", "max_staleness_ms": 200, "min_score": 1.01}),
            serde_json::json!({"source": "detector", "layout": "top1_box_norm_v1", "max_staleness_ms": 200, "target_label": "   "}),
            serde_json::json!({"source": "detector", "layout": "top1_box_norm_v1", "max_staleness_ms": 200, "on_target_lost": "ignore"}),
            serde_json::json!({"source": "detector", "layout": "top1_box_norm_v1", "max_staleness_ms": 200, "on_stale": "zero_features"}),
            serde_json::json!({"source": "detector", "layout": "top1_box_norm_v1", "max_staleness_ms": 200, "require_model_generation": "yes"}),
        ];
        for bad_vis in cases {
            let mut m = base_vision.clone();
            m["vision"] = bad_vis.clone();
            assert!(
                StrictPolicyContract::parse(&serde_json::to_vec(&m).unwrap()).is_err(),
                "expected manifest rejection for vision={bad_vis}"
            );
        }

        // Verify backend input_dim validation against expected_input_dim() (12 vs 6).
        let dir_bad = tempdir().unwrap();
        let layers_6 = vec![MlpLayerSpec::new(6, 6, Activation::None, vec![0.0; 36], vec![0.0; 6])];
        write_rmlp_pack(
            dir_bad.path(),
            &["j1", "j2", "j3", "j4", "j5", "j6"],
            "rad",
            &layers_6,
            serde_json::json!({
                "vision": {
                    "source": "detector",
                    "layout": "top1_box_norm_v1",
                    "max_staleness_ms": 200
                }
            }),
        );
        let snap_bad = PolicySnapshot::load_from_dir(dir_bad.path()).unwrap();
        let contract_bad = StrictPolicyContract::parse(&snap_bad.policy_json_bytes).unwrap();
        let err = load_backend(&snap_bad, &contract_bad).err().unwrap();
        assert!(
            matches!(err, PolicyError::DimensionMismatch { expected: 12, actual: 6 }),
            "expected DimensionMismatch(12, 6), got {err:?}"
        );
    }

    #[tokio::test]
    async fn vision_policy_e2e_concatenates_proprio_and_top1_box_features() {
        let dir = tempdir().unwrap();
        let layers = make_12_to_6_layers();
        write_rmlp_pack(
            dir.path(),
            &["j1", "j2", "j3", "j4", "j5", "j6"],
            "rad",
            &layers,
            serde_json::json!({
                "vision": {
                    "source": "detector",
                    "layout": "top1_box_norm_v1",
                    "target_label": "cube",
                    "min_score": 0.4,
                    "max_staleness_ms": 300,
                    "on_target_lost": "hold",
                    "on_stale": "stop",
                    "require_model_generation": true
                }
            }),
        );

        let mut cfg = make_6dof_config();
        cfg.home_positions = vec![0.1, -0.2, 0.3, -0.4, 0.5, -0.6];
        let mut prepared = prepare_policy(dir.path(), &cfg).unwrap();

        // Direct unit check: step_infer without vision must fail on vision policy.
        assert!(matches!(
            prepared.step_infer(&[0.0; 6]),
            Err(PolicyError::DimensionMismatch { expected: 6, actual: 0 })
        ));

        let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
        let gen = sup.acquire_lease("conn-vis-e2e").await.unwrap();
        let epoch = sup.get_observation().await.epoch_id;
        let action_id = {
            let mut inner = sup.inner().write().await;
            inner
                .begin_policy_run(PolicyRunStart {
                    epoch_id: epoch,
                    connection_id: "conn-vis-e2e".into(),
                    lease_generation: gen,
                    policy_id: "vis_e2e".into(),
                    artifact_digest: prepared.artifact_digest().to_string(),
                    backend: prepared.backend_name().to_string(),
                    mode: PolicyMode::Shadow,
                    sim_only: false,
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
                    i_am_at_the_arm: false,
                    verified_release_gate: None,
                })
                .unwrap()
        };

        let next_fid = Arc::new(AtomicUsize::new(41));
        let src = MockVisionSource::new(move |after_id, _wait_ms| {
            let cur = next_fid.load(Ordering::SeqCst) as u64;
            if after_id >= cur {
                return Ok(None);
            }
            Ok(Some(PolicyVisionFrame {
                frame_id: cur,
                captured_at: Some(Instant::now()),
                detections: vec![
                    make_detection("other", 0.99, [0.0, 0.0, 0.1, 0.1]),
                    make_detection("cube", 0.50, [0.1, 0.1, 0.2, 0.2]),
                    make_detection("cube", 0.90, [0.2, 0.3, 0.6, 0.5]),
                ],
                model_generation: Some(7),
            }))
        });

        let runner = spawn_policy_runner(
            sup.clone(),
            action_id,
            prepared,
            25.0,
            compute_policy_deadline(Some(0.20)).unwrap(),
            Some(src),
            None,
        );
        let _ = runner.await;

        let obs = sup.get_observation().await;
        let pol = obs.policy.expect("policy observation present");
        assert_eq!(pol.stop_reason.as_deref(), Some("MAX_SECONDS"));
        assert!(pol.steps > 0);
        assert!(pol.last_vision_age_ms.is_some());
        assert_eq!(pol.vision_frame_id, Some(41));

        // Expected vision features for [0.2, 0.3, 0.6, 0.5] score 0.9:
        // [present=1.0, cx=0.4, cy=0.4, w=0.4, h=0.2, score=0.9]
        // Expected output[i] = home_positions[i] + 0.1 * vision[i]
        let expected_vis = [1.0f32, 0.4, 0.4, 0.4, 0.2, 0.9];
        let got = pol.last_shadow_target.expect("last_shadow_target present");
        for i in 0..6 {
            let exp = cfg.home_positions[i] + 0.1 * expected_vis[i];
            assert!(
                (got[i] - exp).abs() < 1e-5,
                "target[{i}] got={} expected={exp}",
                got[i]
            );
        }

        // Also test missing vision_source -> immediate VISION_UNAVAILABLE
        let prepared2 = prepare_policy(dir.path(), &cfg).unwrap();
        let sup2 = Arc::new(ArmSupervisor::new(cfg.clone()));
        let gen2 = sup2.acquire_lease("conn-vis-unavail").await.unwrap();
        let epoch2 = sup2.get_observation().await.epoch_id;
        let action_id2 = {
            let mut inner = sup2.inner().write().await;
            inner
                .begin_policy_run(PolicyRunStart {
                    epoch_id: epoch2,
                    connection_id: "conn-vis-unavail".into(),
                    lease_generation: gen2,
                    policy_id: "vis_unavail".into(),
                    artifact_digest: prepared2.artifact_digest().to_string(),
                    backend: prepared2.backend_name().to_string(),
                    mode: PolicyMode::Shadow,
                    sim_only: false,
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
                    i_am_at_the_arm: false,
                    verified_release_gate: None,
                })
                .unwrap()
        };
        let runner2 = spawn_policy_runner(sup2.clone(), action_id2, prepared2, 25.0, None, None, None);
        let _ = runner2.await;
        let inner2 = sup2.inner().read().await;
        assert_eq!(inner2.last_action.as_ref().unwrap().state, "failed");
        assert_eq!(
            inner2.policy_observation().unwrap().stop_reason.as_deref(),
            Some("VISION_UNAVAILABLE")
        );

        // Also verify WS wiring via DetectionSourceAdapter & GraspDeps
        struct TestDetSource;
        impl DetectionSource for TestDetSource {
            fn inference_enabled(&self) -> BoxFuture<'_, Result<bool, String>> {
                Box::pin(async { Ok(true) })
            }
            fn set_inference(&self, _enabled: bool) -> BoxFuture<'_, Result<(), String>> {
                Box::pin(async { Ok(()) })
            }
            fn next_frame(
                &self,
                after: Option<u64>,
                _wait_ms: u64,
            ) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
                Box::pin(async move {
                    if after.is_some_and(|a| a >= 100) {
                        return Ok(None);
                    }
                    let now = Instant::now();
                    Ok(Some(DetectionFrame {
                        frame_id: 100,
                        captured_at: now,
                        policy_captured_at: Some(now),
                        image_size_px: [640.0, 360.0],
                        detections: vec![make_detection("cube", 0.9, [0.2, 0.3, 0.6, 0.5])],
                        depth: None,
                        model_generation: Some(1),
                    }))
                })
            }
        }

        let sandbox_dir = tempdir().unwrap();
        let pack_dir = sandbox_dir.path().join("policies").join("ws_vis");
        fs::create_dir_all(&pack_dir).unwrap();
        write_rmlp_pack(
            &pack_dir,
            &["j1", "j2", "j3", "j4", "j5", "j6"],
            "rad",
            &layers,
            serde_json::json!({
                "vision": {
                    "source": "detector",
                    "layout": "top1_box_norm_v1",
                    "target_label": "cube",
                    "min_score": 0.4,
                    "max_staleness_ms": 300,
                    "require_model_generation": true
                }
            }),
        );
        let sandbox = Arc::new(SandboxState::new(sandbox_dir.path().to_path_buf()));
        let sup_ws = Arc::new(ArmSupervisor::new(cfg.clone()));
        let upstream = DetectorUpstream::parse("http://127.0.0.1:1").unwrap();
        let deps = GraspDeps {
            source: Arc::new(TestDetSource),
            setup_path: None,
        };
        let app = create_router_full_with_sandbox(
            sup_ws.clone(),
            upstream.clone(),
            deps,
            DiagnosticsState::global(),
            CalibrationDeps::from_env(None, upstream),
            OperatorAuth::disabled(),
            sandbox,
        );
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr: SocketAddr = listener.local_addr().unwrap();
        tokio::spawn(async move {
            axum::serve(listener, app).await.unwrap();
        });

        let (mut ws, _) = connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
        ws.send(Message::Text(
            serde_json::json!({"type": "acquire_lease", "client_id": "ws_vis_test"})
                .to_string()
                .into(),
        ))
        .await
        .unwrap();
        let gen_ws = loop {
            let msg: serde_json::Value =
                serde_json::from_str(&ws.next().await.unwrap().unwrap().into_text().unwrap()).unwrap();
            if msg["type"] == "lease_granted" {
                break msg["generation"].as_u64().unwrap();
            }
        };
        let epoch_ws = sup_ws.get_observation().await.epoch_id;
        ws.send(Message::Text(
            serde_json::json!({
                "type": "command",
                "request_id": 1,
                "generation": gen_ws,
                "epoch_id": epoch_ws,
                "path": "/api/v1/action/policy/start",
                "body": {
                    "policy_id": "ws_vis",
                    "mode": "shadow",
                    "fps": 25.0,
                    "max_seconds": 0.2
                }
            })
            .to_string()
            .into(),
        ))
        .await
        .unwrap();
        loop {
            let msg: serde_json::Value =
                serde_json::from_str(&ws.next().await.unwrap().unwrap().into_text().unwrap()).unwrap();
            if msg["request_id"] == 1 {
                assert_eq!(msg["type"], "command_ack", "unexpected WS reply: {msg}");
                break;
            }
        }
        tokio::time::sleep(Duration::from_millis(260)).await;
        let pol_ws = sup_ws.get_observation().await.policy.unwrap();
        assert_eq!(pol_ws.vision_frame_id, Some(100));
        assert_eq!(pol_ws.stop_reason.as_deref(), Some("MAX_SECONDS"));
        let ws_got = pol_ws.last_shadow_target.unwrap();
        for i in 0..6 {
            let exp = cfg.home_positions[i] + 0.1 * expected_vis[i];
            assert!((ws_got[i] - exp).abs() < 1e-5);
        }
    }

    #[tokio::test]
    async fn vision_policy_stops_on_stale_frame_and_missing_timestamp() {
        let dir = tempdir().unwrap();
        let layers = make_12_to_6_layers();
        write_rmlp_pack(
            dir.path(),
            &["j1", "j2", "j3", "j4", "j5", "j6"],
            "rad",
            &layers,
            serde_json::json!({
                "vision": {
                    "source": "detector",
                    "layout": "top1_box_norm_v1",
                    "max_staleness_ms": 100,
                    "max_sync_skew_ms": 80,
                    "on_stale": "stop",
                    "require_model_generation": true
                }
            }),
        );
        let cfg = make_6dof_config();

        // Case A: Stale frame (captured 250 ms ago > max_staleness_ms 100)
        {
            let prepared = prepare_policy(dir.path(), &cfg).unwrap();
            let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
            let gen = sup.acquire_lease("conn-stale").await.unwrap();
            sup.set_torque(true).await.unwrap();
            sup.tick(0.02).await;
            let epoch = sup.get_observation().await.epoch_id;
            let action_id = {
                let mut inner = sup.inner().write().await;
                inner
                    .begin_policy_run(PolicyRunStart {
                        epoch_id: epoch,
                        connection_id: "conn-stale".into(),
                        lease_generation: gen,
                        policy_id: "vis_stale".into(),
                        artifact_digest: prepared.artifact_digest().to_string(),
                        backend: prepared.backend_name().to_string(),
                        mode: PolicyMode::Live,
                        sim_only: false,
                        max_first_step_rad: None,
                        max_step_rad: None,
                        tag: None,
                        i_am_at_the_arm: false,
                        verified_release_gate: None,
                    })
                    .unwrap()
            };
            let src = MockVisionSource::new(|after, _| {
                if after >= 1 {
                    return Ok(None);
                }
                Ok(Some(PolicyVisionFrame {
                    frame_id: 1,
                    captured_at: Some(Instant::now() - Duration::from_millis(250)),
                    detections: vec![make_detection("cube", 0.9, [0.2, 0.2, 0.4, 0.4])],
                    model_generation: Some(1),
                }))
            });
            let runner = spawn_policy_runner(sup.clone(), action_id, prepared, 25.0, None, Some(src), None);
            let _ = runner.await;

            let inner = sup.inner().read().await;
            assert!(inner.servo_target.is_none(), "stale frame must not apply target");
            assert_eq!(inner.last_action.as_ref().unwrap().state, "failed");
            let pol = inner.policy_observation().unwrap();
            assert_eq!(pol.stop_reason.as_deref(), Some("VISION_STALE"));
            assert_eq!(pol.steps, 0);
        }

        // Case B: Missing timestamp (captured_at = None)
        {
            let prepared = prepare_policy(dir.path(), &cfg).unwrap();
            let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
            let gen = sup.acquire_lease("conn-missing-ts").await.unwrap();
            sup.set_torque(true).await.unwrap();
            sup.tick(0.02).await;
            let epoch = sup.get_observation().await.epoch_id;
            let action_id = {
                let mut inner = sup.inner().write().await;
                inner
                    .begin_policy_run(PolicyRunStart {
                        epoch_id: epoch,
                        connection_id: "conn-missing-ts".into(),
                        lease_generation: gen,
                        policy_id: "vis_missing_ts".into(),
                        artifact_digest: prepared.artifact_digest().to_string(),
                        backend: prepared.backend_name().to_string(),
                        mode: PolicyMode::Live,
                        sim_only: false,
                        max_first_step_rad: None,
                        max_step_rad: None,
                        tag: None,
                        i_am_at_the_arm: false,
                        verified_release_gate: None,
                    })
                    .unwrap()
            };
            let src = MockVisionSource::new(|after, _| {
                if after >= 2 {
                    return Ok(None);
                }
                Ok(Some(PolicyVisionFrame {
                    frame_id: 2,
                    captured_at: None,
                    detections: vec![make_detection("cube", 0.9, [0.2, 0.2, 0.4, 0.4])],
                    model_generation: Some(1),
                }))
            });
            let runner = spawn_policy_runner(sup.clone(), action_id, prepared, 25.0, None, Some(src), None);
            let _ = runner.await;

            let inner = sup.inner().read().await;
            assert!(inner.servo_target.is_none(), "missing timestamp must not apply target");
            assert_eq!(inner.last_action.as_ref().unwrap().state, "failed");
            let pol = inner.policy_observation().unwrap();
            assert_eq!(pol.stop_reason.as_deref(), Some("VISION_TIMESTAMP_MISSING"));
            assert_eq!(pol.steps, 0);
        }

        // Case C: Sync skew (> max_sync_skew_ms 80) while within max_staleness_ms (100)
        {
            let prepared = prepare_policy(dir.path(), &cfg).unwrap();
            let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
            let gen = sup.acquire_lease("conn-skew").await.unwrap();
            let epoch = sup.get_observation().await.epoch_id;
            let action_id = {
                let mut inner = sup.inner().write().await;
                inner
                    .begin_policy_run(PolicyRunStart {
                        epoch_id: epoch,
                        connection_id: "conn-skew".into(),
                        lease_generation: gen,
                        policy_id: "vis_skew".into(),
                        artifact_digest: prepared.artifact_digest().to_string(),
                        backend: prepared.backend_name().to_string(),
                        mode: PolicyMode::Shadow,
                        sim_only: false,
                        max_first_step_rad: None,
                        max_step_rad: None,
                        tag: None,
                        i_am_at_the_arm: false,
                        verified_release_gate: None,
                    })
                    .unwrap()
            };
            // Mock driver base_input_age_ms = 0, vision age = 90 ms -> skew 90 > 80.
            let src = MockVisionSource::new(|after, _| {
                if after >= 3 {
                    return Ok(None);
                }
                Ok(Some(PolicyVisionFrame {
                    frame_id: 3,
                    captured_at: Some(Instant::now() - Duration::from_millis(90)),
                    detections: vec![make_detection("cube", 0.9, [0.2, 0.2, 0.4, 0.4])],
                    model_generation: Some(1),
                }))
            });
            let runner = spawn_policy_runner(sup.clone(), action_id, prepared, 25.0, None, Some(src), None);
            let _ = runner.await;

            let inner = sup.inner().read().await;
            assert_eq!(inner.last_action.as_ref().unwrap().state, "failed");
            let pol = inner.policy_observation().unwrap();
            assert_eq!(pol.stop_reason.as_deref(), Some("VISION_SYNC_SKEW"));
            assert_eq!(pol.steps, 0);
        }
    }

    #[tokio::test]
    async fn vision_policy_stops_on_model_generation_change() {
        let dir = tempdir().unwrap();
        let layers = make_12_to_6_layers();
        write_rmlp_pack(
            dir.path(),
            &["j1", "j2", "j3", "j4", "j5", "j6"],
            "rad",
            &layers,
            serde_json::json!({
                "vision": {
                    "source": "detector",
                    "layout": "top1_box_norm_v1",
                    "max_staleness_ms": 300,
                    "on_stale": "hold",
                    "require_model_generation": true
                }
            }),
        );
        let cfg = make_6dof_config();
        let prepared = prepare_policy(dir.path(), &cfg).unwrap();
        let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
        let gen = sup.acquire_lease("conn-gen").await.unwrap();
        let epoch = sup.get_observation().await.epoch_id;
        let action_id = {
            let mut inner = sup.inner().write().await;
            inner
                .begin_policy_run(PolicyRunStart {
                    epoch_id: epoch,
                    connection_id: "conn-gen".into(),
                    lease_generation: gen,
                    policy_id: "vis_gen".into(),
                    artifact_digest: prepared.artifact_digest().to_string(),
                    backend: prepared.backend_name().to_string(),
                    mode: PolicyMode::Shadow,
                    sim_only: false,
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
                    i_am_at_the_arm: false,
                    verified_release_gate: None,
                })
                .unwrap()
        };

        let gen_counter = Arc::new(AtomicUsize::new(1));
        let gen_c = gen_counter.clone();
        let src = MockVisionSource::new_async(move |after, _| {
            let gen_c = gen_c.clone();
            Box::pin(async move {
                let cur_gen = gen_c.load(Ordering::SeqCst) as u64;
                let fid = after + 1;
                if fid > 1 && cur_gen == 1 {
                    tokio::time::sleep(Duration::from_millis(10)).await;
                }
                Ok(Some(PolicyVisionFrame {
                    frame_id: fid,
                    captured_at: Some(Instant::now()),
                    detections: vec![make_detection("cube", 0.9, [0.2, 0.2, 0.4, 0.4])],
                    model_generation: Some(gen_c.load(Ordering::SeqCst) as u64),
                }))
            })
        });

        let runner = spawn_policy_runner(sup.clone(), action_id, prepared, 30.0, None, Some(src), None);
        for _ in 0..50 {
            if sup
                .get_observation()
                .await
                .policy
                .is_some_and(|p| p.steps >= 1)
            {
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        gen_counter.store(2, Ordering::SeqCst);
        let _ = runner.await;

        let inner = sup.inner().read().await;
        assert_eq!(inner.last_action.as_ref().unwrap().state, "failed");
        let pol = inner.policy_observation().unwrap();
        assert!(pol.steps >= 1);
        assert_eq!(pol.stop_reason.as_deref(), Some("VISION_GENERATION_CHANGED"));

        // Also test missing generation when required -> VISION_GENERATION_MISSING
        let prepared_missing = prepare_policy(dir.path(), &cfg).unwrap();
        let sup_m = Arc::new(ArmSupervisor::new(cfg));
        let gen_m = sup_m.acquire_lease("conn-gen-miss").await.unwrap();
        let epoch_m = sup_m.get_observation().await.epoch_id;
        let id_m = {
            let mut inner = sup_m.inner().write().await;
            inner
                .begin_policy_run(PolicyRunStart {
                    epoch_id: epoch_m,
                    connection_id: "conn-gen-miss".into(),
                    lease_generation: gen_m,
                    policy_id: "vis_gen_miss".into(),
                    artifact_digest: prepared_missing.artifact_digest().to_string(),
                    backend: prepared_missing.backend_name().to_string(),
                    mode: PolicyMode::Shadow,
                    sim_only: false,
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
                    i_am_at_the_arm: false,
                    verified_release_gate: None,
                })
                .unwrap()
        };
        let src_m = MockVisionSource::new(|after, _| {
            if after >= 1 {
                return Ok(None);
            }
            Ok(Some(PolicyVisionFrame {
                frame_id: 1,
                captured_at: Some(Instant::now()),
                detections: vec![make_detection("cube", 0.9, [0.2, 0.2, 0.4, 0.4])],
                model_generation: None,
            }))
        });
        let runner_m = spawn_policy_runner(sup_m.clone(), id_m, prepared_missing, 25.0, None, Some(src_m), None);
        let _ = runner_m.await;
        let inner_m = sup_m.inner().read().await;
        assert_eq!(inner_m.last_action.as_ref().unwrap().state, "failed");
        assert_eq!(
            inner_m.policy_observation().unwrap().stop_reason.as_deref(),
            Some("VISION_GENERATION_MISSING")
        );
    }

    #[tokio::test]
    async fn vision_policy_target_lost_modes_zero_hold_and_stop() {
        let mut cfg = make_6dof_config();
        cfg.home_positions = vec![0.1, -0.1, 0.0, 0.0, 0.0, 0.0];
        let layers = make_12_to_6_layers();

        let empty_src = || {
            MockVisionSource::new_async(|after, _| {
                Box::pin(async move {
                    if after > 0 {
                        tokio::time::sleep(Duration::from_millis(10)).await;
                    }
                    Ok(Some(PolicyVisionFrame {
                        frame_id: after + 1,
                        captured_at: Some(Instant::now()),
                        detections: vec![make_detection("wrong_label", 0.95, [0.2, 0.2, 0.4, 0.4])],
                        model_generation: Some(1),
                    }))
                })
            })
        };

        // 1. zero_features -> emits output with [0.0; 6] vision features
        {
            let dir = tempdir().unwrap();
            write_rmlp_pack(
                dir.path(),
                &["j1", "j2", "j3", "j4", "j5", "j6"],
                "rad",
                &layers,
                serde_json::json!({
                    "vision": {
                        "source": "detector",
                        "layout": "top1_box_norm_v1",
                        "target_label": "cube",
                        "max_staleness_ms": 300,
                        "on_target_lost": "zero_features"
                    }
                }),
            );
            let prepared = prepare_policy(dir.path(), &cfg).unwrap();
            let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
            let gen = sup.acquire_lease("conn-zero").await.unwrap();
            let epoch = sup.get_observation().await.epoch_id;
            let id = {
                let mut inner = sup.inner().write().await;
                inner
                    .begin_policy_run(PolicyRunStart {
                        epoch_id: epoch,
                        connection_id: "conn-zero".into(),
                        lease_generation: gen,
                        policy_id: "vis_zero".into(),
                        artifact_digest: prepared.artifact_digest().to_string(),
                        backend: prepared.backend_name().to_string(),
                        mode: PolicyMode::Shadow,
                        sim_only: false,
                        max_first_step_rad: None,
                        max_step_rad: None,
                        tag: None,
                        i_am_at_the_arm: false,
                        verified_release_gate: None,
                    })
                    .unwrap()
            };
            let runner = spawn_policy_runner(
                sup.clone(),
                id,
                prepared,
                25.0,
                compute_policy_deadline(Some(0.18)).unwrap(),
                Some(empty_src()),
                None,
            );
            let _ = runner.await;
            let pol = sup.get_observation().await.policy.unwrap();
            assert!(pol.steps > 0);
            assert_eq!(pol.stop_reason.as_deref(), Some("MAX_SECONDS"));
            let got = pol.last_shadow_target.unwrap();
            for i in 0..6 {
                assert!((got[i] - cfg.home_positions[i]).abs() < 1e-6);
            }
        }

        // 2. hold -> skips steps without failing or mutating servo_target
        {
            let dir = tempdir().unwrap();
            write_rmlp_pack(
                dir.path(),
                &["j1", "j2", "j3", "j4", "j5", "j6"],
                "rad",
                &layers,
                serde_json::json!({
                    "vision": {
                        "source": "detector",
                        "layout": "top1_box_norm_v1",
                        "target_label": "cube",
                        "max_staleness_ms": 300,
                        "on_target_lost": "hold"
                    }
                }),
            );
            let prepared = prepare_policy(dir.path(), &cfg).unwrap();
            let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
            let gen = sup.acquire_lease("conn-hold").await.unwrap();
            sup.set_torque(true).await.unwrap();
            sup.tick(0.02).await;
            let epoch = sup.get_observation().await.epoch_id;
            let id = {
                let mut inner = sup.inner().write().await;
                inner
                    .begin_policy_run(PolicyRunStart {
                        epoch_id: epoch,
                        connection_id: "conn-hold".into(),
                        lease_generation: gen,
                        policy_id: "vis_hold".into(),
                        artifact_digest: prepared.artifact_digest().to_string(),
                        backend: prepared.backend_name().to_string(),
                        mode: PolicyMode::Live,
                        sim_only: false,
                        max_first_step_rad: None,
                        max_step_rad: None,
                        tag: None,
                        i_am_at_the_arm: false,
                        verified_release_gate: None,
                    })
                    .unwrap()
            };
            let runner = spawn_policy_runner(
                sup.clone(),
                id,
                prepared,
                25.0,
                compute_policy_deadline(Some(0.18)).unwrap(),
                Some(empty_src()),
                None,
            );
            let _ = runner.await;
            let inner = sup.inner().read().await;
            assert!(inner.servo_target.is_none());
            assert_eq!(inner.last_action.as_ref().unwrap().state, "done");
            let pol = inner.policy_observation().unwrap();
            assert_eq!(pol.steps, 0);
            assert_eq!(pol.stop_reason.as_deref(), Some("MAX_SECONDS"));
        }

        // 3. stop -> terminates immediately with TARGET_LOST
        {
            let dir = tempdir().unwrap();
            write_rmlp_pack(
                dir.path(),
                &["j1", "j2", "j3", "j4", "j5", "j6"],
                "rad",
                &layers,
                serde_json::json!({
                    "vision": {
                        "source": "detector",
                        "layout": "top1_box_norm_v1",
                        "target_label": "cube",
                        "max_staleness_ms": 300,
                        "on_target_lost": "stop"
                    }
                }),
            );
            let prepared = prepare_policy(dir.path(), &cfg).unwrap();
            let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
            let gen = sup.acquire_lease("conn-stop").await.unwrap();
            let epoch = sup.get_observation().await.epoch_id;
            let id = {
                let mut inner = sup.inner().write().await;
                inner
                    .begin_policy_run(PolicyRunStart {
                        epoch_id: epoch,
                        connection_id: "conn-stop".into(),
                        lease_generation: gen,
                        policy_id: "vis_stop".into(),
                        artifact_digest: prepared.artifact_digest().to_string(),
                        backend: prepared.backend_name().to_string(),
                        mode: PolicyMode::Shadow,
                        sim_only: false,
                        max_first_step_rad: None,
                        max_step_rad: None,
                        tag: None,
                        i_am_at_the_arm: false,
                        verified_release_gate: None,
                    })
                    .unwrap()
            };
            let runner = spawn_policy_runner(sup.clone(), id, prepared, 25.0, None, Some(empty_src()), None);
            let _ = runner.await;
            let inner = sup.inner().read().await;
            assert_eq!(inner.last_action.as_ref().unwrap().state, "failed");
            let pol = inner.policy_observation().unwrap();
            assert_eq!(pol.steps, 0);
            assert_eq!(pol.stop_reason.as_deref(), Some("TARGET_LOST"));
        }
    }

    #[tokio::test]
    async fn vision_policy_allows_repeated_frame_within_staleness_window() {
        let dir = tempdir().unwrap();
        let layers = make_12_to_6_layers();
        write_rmlp_pack(
            dir.path(),
            &["j1", "j2", "j3", "j4", "j5", "j6"],
            "rad",
            &layers,
            serde_json::json!({
                "vision": {
                    "source": "detector",
                    "layout": "top1_box_norm_v1",
                    "max_staleness_ms": 200,
                    "on_stale": "stop",
                    "require_model_generation": true
                }
            }),
        );
        let cfg = make_6dof_config();
        let prepared = prepare_policy(dir.path(), &cfg).unwrap();
        let sup = Arc::new(ArmSupervisor::new(cfg));
        let gen = sup.acquire_lease("conn-repeat").await.unwrap();
        let epoch = sup.get_observation().await.epoch_id;
        let id = {
            let mut inner = sup.inner().write().await;
            inner
                .begin_policy_run(PolicyRunStart {
                    epoch_id: epoch,
                    connection_id: "conn-repeat".into(),
                    lease_generation: gen,
                    policy_id: "vis_repeat".into(),
                    artifact_digest: prepared.artifact_digest().to_string(),
                    backend: prepared.backend_name().to_string(),
                    mode: PolicyMode::Shadow,
                    sim_only: false,
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
                    i_am_at_the_arm: false,
                    verified_release_gate: None,
                })
                .unwrap()
        };

        // 15 Hz vision (~67 ms between frames) feeding 30 Hz policy (~33 ms) with 200 ms staleness window.
        let src = MockVisionSource::new_async(|after, _| {
            Box::pin(async move {
                if after > 0 {
                    tokio::time::sleep(Duration::from_millis(67)).await;
                }
                Ok(Some(PolicyVisionFrame {
                    frame_id: after + 1,
                    captured_at: Some(Instant::now()),
                    detections: vec![make_detection("cube", 0.85, [0.25, 0.25, 0.55, 0.55])],
                    model_generation: Some(1),
                }))
            })
        });

        let runner = spawn_policy_runner(
            sup.clone(),
            id,
            prepared,
            30.0,
            compute_policy_deadline(Some(0.35)).unwrap(),
            Some(src),
            None,
        );
        let _ = runner.await;

        let inner = sup.inner().read().await;
        assert_eq!(inner.last_action.as_ref().unwrap().state, "done");
        let pol = inner.policy_observation().unwrap();
        assert_eq!(pol.stop_reason.as_deref(), Some("MAX_SECONDS"));
        assert!(
            pol.steps >= 6,
            "expected >= 6 steps at 30 Hz over 350ms reusing 15 Hz vision frames, got {}",
            pol.steps
        );
        assert!(pol.vision_frame_id.is_some());
        assert!(pol.last_vision_age_ms.is_some_and(|a| a <= 200));
    }

    #[tokio::test]
    async fn http_detector_missing_or_negative_age_ms_stops_with_vision_timestamp_missing() {
        use armd::grasp::HttpDetectionSource;
        use armd::policy_runtime::DetectionSourceAdapter;
        use axum::{routing::get, Json, Router};

        let dir = tempdir().unwrap();
        let layers = make_12_to_6_layers();
        write_rmlp_pack(
            dir.path(),
            &["j1", "j2", "j3", "j4", "j5", "j6"],
            "rad",
            &layers,
            serde_json::json!({
                "vision": {
                    "source": "detector",
                    "layout": "top1_box_norm_v1",
                    "max_staleness_ms": 200,
                    "on_stale": "stop",
                    "require_model_generation": true
                }
            }),
        );
        let cfg = make_6dof_config();

        for (case_name, payload) in [
            (
                "missing_age_ms",
                serde_json::json!({
                    "frame_id": 10,
                    "width": 640,
                    "height": 360,
                    "model_generation": 1,
                    "detections": [
                        {"label": "cube", "score": 0.9, "box_norm": [0.2, 0.3, 0.6, 0.5]}
                    ]
                }),
            ),
            (
                "negative_age_ms",
                serde_json::json!({
                    "frame_id": 11,
                    "age_ms": -5.0,
                    "width": 640,
                    "height": 360,
                    "model_generation": 1,
                    "detections": [
                        {"label": "cube", "score": 0.9, "box_norm": [0.2, 0.3, 0.6, 0.5]}
                    ]
                }),
            ),
        ] {
            let body = payload.clone();
            let app = Router::new().route(
                "/api/results/latest",
                get(move || {
                    let body = body.clone();
                    async move { Json(body) }
                }),
            );
            let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
            let addr: SocketAddr = listener.local_addr().unwrap();
            let server = tokio::spawn(async move {
                let _ = axum::serve(listener, app).await;
            });

            let http_src = Arc::new(HttpDetectionSource {
                upstream: DetectorUpstream::parse(&format!("http://{addr}")).unwrap(),
                image_size_px: [640.0, 360.0],
            });
            let adapter: Arc<dyn PolicyVisionSource> = Arc::new(DetectionSourceAdapter(http_src));

            let direct = adapter
                .next_frame(0, 200)
                .await
                .unwrap()
                .expect("expected frame from local HTTP detector");
            assert!(
                direct.captured_at.is_none(),
                "{case_name}: expected captured_at == None"
            );

            let prepared = prepare_policy(dir.path(), &cfg).unwrap();
            let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
            let conn_id = format!("conn-{case_name}");
            let gen = sup.acquire_lease(&conn_id).await.unwrap();
            sup.set_torque(true).await.unwrap();
            sup.tick(0.02).await;
            let epoch = sup.get_observation().await.epoch_id;
            let action_id = {
                let mut inner = sup.inner().write().await;
                inner
                    .begin_policy_run(PolicyRunStart {
                        epoch_id: epoch,
                        connection_id: format!("conn-{case_name}"),
                        lease_generation: gen,
                        policy_id: format!("http_{case_name}"),
                        artifact_digest: prepared.artifact_digest().to_string(),
                        backend: prepared.backend_name().to_string(),
                        mode: PolicyMode::Live,
                        sim_only: false,
                        max_first_step_rad: None,
                        max_step_rad: None,
                        tag: None,
                        i_am_at_the_arm: false,
                        verified_release_gate: None,
                    })
                    .unwrap()
            };

            let runner = spawn_policy_runner(sup.clone(), action_id, prepared, 25.0, None, Some(adapter), None);
            let _ = runner.await;
            server.abort();

            let inner = sup.inner().read().await;
            assert!(
                inner.servo_target.is_none(),
                "{case_name}: missing/invalid age_ms must not apply target"
            );
            assert_eq!(inner.last_action.as_ref().unwrap().state, "failed");
            let pol = inner.policy_observation().unwrap();
            assert_eq!(
                pol.stop_reason.as_deref(),
                Some("VISION_TIMESTAMP_MISSING"),
                "{case_name}: unexpected stop_reason"
            );
            assert_eq!(pol.steps, 0, "{case_name}: expected 0 steps");
        }
    }

    #[tokio::test]
    async fn stale_vision_under_lock_contention_stops_without_applying_target() {
        use std::sync::mpsc;

        struct BlockingSignalBackend {
            entered_tx: Mutex<Option<mpsc::Sender<()>>>,
            proceed_rx: Mutex<Option<mpsc::Receiver<()>>>,
        }

        impl PolicyBackend for BlockingSignalBackend {
            fn backend_name(&self) -> &'static str {
                "blocking_signal"
            }
            fn input_dim(&self) -> usize {
                12
            }
            fn output_dim(&self) -> usize {
                6
            }
            fn infer(&mut self, _input: &[f32], output: &mut [f32]) -> Result<(), PolicyError> {
                if let Some(tx) = self.entered_tx.lock().unwrap().take() {
                    let _ = tx.send(());
                }
                if let Some(rx) = self.proceed_rx.lock().unwrap().take() {
                    let _ = rx.recv_timeout(Duration::from_secs(2));
                }
                output.copy_from_slice(&[0.05, 0.0, 0.0, 0.0, 0.0, 0.0]);
                Ok(())
            }
        }

        let dir = tempdir().unwrap();
        let layers = make_12_to_6_layers();
        write_rmlp_pack(
            dir.path(),
            &["j1", "j2", "j3", "j4", "j5", "j6"],
            "rad",
            &layers,
            serde_json::json!({
                "vision": {
                    "source": "detector",
                    "layout": "top1_box_norm_v1",
                    "max_staleness_ms": 120,
                    "on_stale": "stop",
                    "require_model_generation": true
                }
            }),
        );
        let snap = PolicySnapshot::load_from_dir(dir.path()).unwrap();
        let contract = StrictPolicyContract::parse(&snap.policy_json_bytes).unwrap();
        let cfg = make_6dof_config();

        let (entered_tx, entered_rx) = mpsc::channel();
        let (proceed_tx, proceed_rx) = mpsc::channel();
        let backend = Box::new(BlockingSignalBackend {
            entered_tx: Mutex::new(Some(entered_tx)),
            proceed_rx: Mutex::new(Some(proceed_rx)),
        });
        let prepared = PreparedPolicy::from_backend(snap, contract, backend, &cfg).unwrap();

        let sup = Arc::new(ArmSupervisor::new(cfg));
        let gen = sup.acquire_lease("conn-lock-stale").await.unwrap();
        sup.set_torque(true).await.unwrap();
        sup.tick(0.02).await;
        let epoch = sup.get_observation().await.epoch_id;
        let action_id = {
            let mut inner = sup.inner().write().await;
            inner
                .begin_policy_run(PolicyRunStart {
                    epoch_id: epoch,
                    connection_id: "conn-lock-stale".into(),
                    lease_generation: gen,
                    policy_id: "vis_lock_stale".into(),
                    artifact_digest: prepared.artifact_digest().to_string(),
                    backend: prepared.backend_name().to_string(),
                    mode: PolicyMode::Live,
                    sim_only: false,
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
                    i_am_at_the_arm: false,
                    verified_release_gate: None,
                })
                .unwrap()
        };

        let src = MockVisionSource::new(|after, _| {
            if after >= 1 {
                return Ok(None);
            }
            Ok(Some(PolicyVisionFrame {
                frame_id: 1,
                captured_at: Some(Instant::now()),
                detections: vec![make_detection("cube", 0.9, [0.2, 0.2, 0.4, 0.4])],
                model_generation: Some(1),
            }))
        });

        let runner = spawn_policy_runner(sup.clone(), action_id, prepared, 25.0, None, Some(src), None);

        // Wait until inference has started (after initial evaluate_vision_frame succeeded),
        // then hold the supervisor write lock past max_staleness_ms before allowing infer to return.
        tokio::task::spawn_blocking(move || {
            entered_rx
                .recv_timeout(Duration::from_secs(2))
                .expect("inference did not start in time");
        })
        .await
        .unwrap();

        let lock_guard = sup.inner().write().await;
        let _ = proceed_tx.send(());
        tokio::time::sleep(Duration::from_millis(180)).await;
        drop(lock_guard);

        let _ = runner.await;

        let inner = sup.inner().read().await;
        assert!(
            inner.servo_target.is_none(),
            "stale vision under lock contention must not apply target"
        );
        assert_eq!(inner.last_action.as_ref().unwrap().state, "failed");
        let pol = inner.policy_observation().unwrap();
        assert_eq!(pol.stop_reason.as_deref(), Some("VISION_STALE"));
        assert_eq!(pol.steps, 0);
    }

    #[test]
    fn evaluate_vision_frame_enforces_min_max_prop_age_range_and_fail_closed_none() {
        use armd::policy_runtime::{evaluate_vision_frame, VisionFeatureSpec, VisionStepDecision};

        let spec = VisionFeatureSpec {
            source: VisionSourceKind::Detector,
            layout: VisionLayout::Top1BoxNormV1,
            target_label: Some("cube".into()),
            min_score: 0.3,
            max_staleness_ms: 200,
            max_sync_skew_ms: 20,
            on_target_lost: VisionTargetLostPolicy::Stop,
            on_stale: VisionStalePolicy::Stop,
            require_model_generation: true,
        };
        let now = Instant::now();
        let run_started_at = now - Duration::from_millis(50);
        let frame = PolicyVisionFrame {
            frame_id: 7,
            captured_at: Some(now - Duration::from_millis(90)),
            detections: vec![make_detection("cube", 0.9, [0.2, 0.3, 0.6, 0.5])],
            model_generation: Some(1),
        };

        // 1. max_prop matches vision_age (90 ms), but min_prop (5 ms) skews by 85 ms > 20 ms -> Stop(VISION_SYNC_SKEW)
        let mut locked_gen = None;
        let d_range_skew = evaluate_vision_frame(
            Some(&frame),
            &spec,
            run_started_at,
            now,
            Some((5, 90)),
            &mut locked_gen,
        );
        assert!(
            matches!(
                d_range_skew,
                VisionStepDecision::Stop {
                    reason: "VISION_SYNC_SKEW",
                    ..
                }
            ),
            "expected VISION_SYNC_SKEW when min_prop skews, got {d_range_skew:?}"
        );

        // 2. Missing proprioception age (None) must fail-closed with VISION_SYNC_SKEW
        let mut locked_gen_none = None;
        let d_missing_age = evaluate_vision_frame(
            Some(&frame),
            &spec,
            run_started_at,
            now,
            None,
            &mut locked_gen_none,
        );
        assert!(
            matches!(
                d_missing_age,
                VisionStepDecision::Stop {
                    reason: "VISION_SYNC_SKEW",
                    ..
                }
            ),
            "expected fail-closed VISION_SYNC_SKEW for None prop_age_range_ms, got {d_missing_age:?}"
        );

        // 3. When both min_prop and max_prop are within 20 ms of 90 ms -> Ready
        let mut locked_gen_ok = None;
        let d_ok = evaluate_vision_frame(
            Some(&frame),
            &spec,
            run_started_at,
            now,
            Some((80, 100)),
            &mut locked_gen_ok,
        );
        assert!(
            matches!(d_ok, VisionStepDecision::Ready { frame_id: 7, .. }),
            "expected Ready when range is within skew, got {d_ok:?}"
        );
    }

    #[test]
    fn supervisor_policy_input_age_range_reports_min_max_and_none_on_missing() {
        struct SkewedAgeDriver {
            ages: Vec<Option<Duration>>,
        }
        impl ArmDriver for SkewedAgeDriver {
            fn capabilities(&self) -> DriverCapabilities {
                DriverCapabilities::default()
            }
            fn enable_state(&self) -> EnableState {
                EnableState::Enabled
            }
            fn is_feedback_valid(&self) -> bool {
                true
            }
            fn feedback_ages(&self) -> Vec<Option<Duration>> {
                self.ages.clone()
            }
            fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
                Ok(vec![JointState::default(); self.ages.len()])
            }
            fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> {
                Ok(())
            }
            fn set_torque(&mut self, _enabled: bool) -> Result<(), ArmDriverError> {
                Ok(())
            }
        }

        let cfg = make_6dof_config();
        let sup_skew = ArmSupervisor::new(cfg.clone()).with_driver(Box::new(SkewedAgeDriver {
            ages: vec![
                Some(Duration::from_millis(5)),
                Some(Duration::from_millis(20)),
                Some(Duration::from_millis(90)),
                Some(Duration::from_millis(12)),
                Some(Duration::from_millis(40)),
                Some(Duration::from_millis(15)),
            ],
        }));
        let range = sup_skew
            .inner()
            .try_read()
            .unwrap()
            .policy_input_age_range_ms();
        assert_eq!(range, Some((5, 90)));

        let sup_missing = ArmSupervisor::new(cfg).with_driver(Box::new(SkewedAgeDriver {
            ages: vec![
                Some(Duration::from_millis(5)),
                None,
                Some(Duration::from_millis(90)),
                Some(Duration::from_millis(12)),
                Some(Duration::from_millis(40)),
                Some(Duration::from_millis(15)),
            ],
        }));
        let range_missing = sup_missing
            .inner()
            .try_read()
            .unwrap()
            .policy_input_age_range_ms();
        assert_eq!(range_missing, None);
    }

    #[test]
    fn extract_top1_box_norm_v1_rejects_inverted_and_zero_area_boxes_before_ranking() {
        use armd::policy_runtime::{extract_top1_box_norm_v1, VisionFeatureSpec};

        let spec = VisionFeatureSpec {
            source: VisionSourceKind::Detector,
            layout: VisionLayout::Top1BoxNormV1,
            target_label: Some("cube".into()),
            min_score: 0.3,
            max_staleness_ms: 200,
            max_sync_skew_ms: 200,
            on_target_lost: VisionTargetLostPolicy::Stop,
            on_stale: VisionStalePolicy::Stop,
            require_model_generation: false,
        };

        let frame = PolicyVisionFrame {
            frame_id: 1,
            captured_at: Some(Instant::now()),
            detections: vec![
                make_detection("cube", 0.99, [0.6, 0.5, 0.2, 0.3]),
                make_detection("cube", 0.97, [0.2, 0.5, 0.6, 0.3]),
                make_detection("cube", 0.96, [0.6, 0.3, 0.2, 0.5]),
                make_detection("cube", 0.95, [0.5, 0.5, 0.5, 0.5]),
                make_detection("cube", 0.94, [0.2, 0.5, 0.6, 0.5]),
                make_detection("cube", 0.80, [0.2, 0.3, 0.6, 0.5]),
            ],
            model_generation: None,
        };

        let feats = extract_top1_box_norm_v1(&frame, &spec).expect("valid box should be selected");
        let expected = [1.0f32, 0.4, 0.4, 0.4, 0.2, 0.80];
        for i in 0..6 {
            assert!(
                (feats[i] - expected[i]).abs() < 1e-6,
                "feats[{i}]={} vs expected={}",
                feats[i],
                expected[i]
            );
        }

        let only_degenerate = PolicyVisionFrame {
            frame_id: 2,
            captured_at: Some(Instant::now()),
            detections: vec![
                make_detection("cube", 0.99, [0.6, 0.5, 0.2, 0.3]),
                make_detection("cube", 0.95, [0.5, 0.5, 0.5, 0.5]),
            ],
            model_generation: None,
        };
        assert_eq!(extract_top1_box_norm_v1(&only_degenerate, &spec), None);
    }

    #[tokio::test]
    async fn http_detector_delayed_response_penalizes_capture_time_and_stops_with_vision_stale() {
        use armd::grasp::HttpDetectionSource;
        use armd::policy_runtime::{
            evaluate_vision_frame, DetectionSourceAdapter, VisionFeatureSpec, VisionStepDecision,
        };
        use axum::{routing::get, Json, Router};

        let dir = tempdir().unwrap();
        let layers = make_12_to_6_layers();
        write_rmlp_pack(
            dir.path(),
            &["j1", "j2", "j3", "j4", "j5", "j6"],
            "rad",
            &layers,
            serde_json::json!({
                "vision": {
                    "source": "detector",
                    "layout": "top1_box_norm_v1",
                    "max_staleness_ms": 100,
                    "on_stale": "stop",
                    "require_model_generation": true
                }
            }),
        );
        let cfg = make_6dof_config();

        let app = Router::new().route(
            "/api/results/latest",
            get(|| async {
                tokio::time::sleep(Duration::from_millis(150)).await;
                Json(serde_json::json!({
                    "frame_id": 10,
                    "age_ms": 5.0,
                    "width": 640,
                    "height": 360,
                    "model_generation": 1,
                    "detections": [
                        {"label": "cube", "score": 0.9, "box_norm": [0.2, 0.3, 0.6, 0.5]}
                    ]
                }))
            }),
        );
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr: SocketAddr = listener.local_addr().unwrap();
        let server = tokio::spawn(async move {
            let _ = axum::serve(listener, app).await;
        });

        let http_src = Arc::new(HttpDetectionSource {
            upstream: DetectorUpstream::parse(&format!("http://{addr}")).unwrap(),
            image_size_px: [640.0, 360.0],
        });
        let adapter: Arc<dyn PolicyVisionSource> = Arc::new(DetectionSourceAdapter(http_src));

        // Direct adapter check: both initial poll (after=0) and long-poll (after=5, wait_ms=25)
        // penalize the 150 ms HTTP response latency so captured_at exceeds max_staleness_ms (100 ms).
        let spec = VisionFeatureSpec {
            source: VisionSourceKind::Detector,
            layout: VisionLayout::Top1BoxNormV1,
            target_label: None,
            min_score: 0.3,
            max_staleness_ms: 100,
            max_sync_skew_ms: 200,
            on_target_lost: VisionTargetLostPolicy::Hold,
            on_stale: VisionStalePolicy::Stop,
            require_model_generation: true,
        };
        let direct_after = adapter
            .next_frame(5, 25)
            .await
            .unwrap()
            .expect("expected delayed frame");
        let age_after_ms = direct_after
            .captured_at
            .expect("valid age_ms produces policy_captured_at")
            .elapsed()
            .as_millis() as u64;
        assert!(
            age_after_ms >= 120,
            "expected conservative age >= 120 ms for 150 ms sleep - 25 ms wait + 5 ms age, got {age_after_ms} ms"
        );
        let now = Instant::now();
        let mut locked_gen = None;
        let decision = evaluate_vision_frame(
            Some(&direct_after),
            &spec,
            now,
            now,
            Some((0, 0)),
            &mut locked_gen,
        );
        assert!(
            matches!(
                decision,
                VisionStepDecision::Stop {
                    reason: "VISION_STALE",
                    ..
                }
            ),
            "expected VISION_STALE for delayed HTTP response, got {decision:?}"
        );

        let prepared = prepare_policy(dir.path(), &cfg).unwrap();
        let sup = Arc::new(ArmSupervisor::new(cfg));
        let gen = sup.acquire_lease("conn-http-delay").await.unwrap();
        sup.set_torque(true).await.unwrap();
        sup.tick(0.02).await;
        let epoch = sup.get_observation().await.epoch_id;
        let action_id = {
            let mut inner = sup.inner().write().await;
            inner
                .begin_policy_run(PolicyRunStart {
                    epoch_id: epoch,
                    connection_id: "conn-http-delay".into(),
                    lease_generation: gen,
                    policy_id: "http_delay".into(),
                    artifact_digest: prepared.artifact_digest().to_string(),
                    backend: prepared.backend_name().to_string(),
                    mode: PolicyMode::Live,
                    sim_only: false,
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
                    i_am_at_the_arm: false,
                    verified_release_gate: None,
                })
                .unwrap()
        };

        let runner = spawn_policy_runner(sup.clone(), action_id, prepared, 12.0, None, Some(adapter), None);
        let _ = runner.await;
        server.abort();

        let inner = sup.inner().read().await;
        assert!(
            inner.servo_target.is_none(),
            "delayed HTTP response must not apply servo target"
        );
        assert_eq!(inner.last_action.as_ref().unwrap().state, "failed");
        let pol = inner.policy_observation().unwrap();
        assert_eq!(pol.stop_reason.as_deref(), Some("VISION_STALE"));
        assert_eq!(pol.steps, 0);
    }

    #[tokio::test]
    async fn vision_generation_change_during_slow_inference_aborts_without_committing_target() {
        use tokio::sync::Notify;

        struct SlowGenBackend {
            infer_entered: Arc<Notify>,
        }

        impl PolicyBackend for SlowGenBackend {
            fn backend_name(&self) -> &'static str {
                "slow_gen_backend"
            }
            fn input_dim(&self) -> usize {
                12
            }
            fn output_dim(&self) -> usize {
                6
            }
            fn infer(&mut self, _input: &[f32], output: &mut [f32]) -> Result<(), PolicyError> {
                self.infer_entered.notify_one();
                std::thread::sleep(Duration::from_millis(60));
                output.copy_from_slice(&[0.05, 0.0, 0.0, 0.0, 0.0, 0.0]);
                Ok(())
            }
        }

        let dir = tempdir().unwrap();
        let layers = make_12_to_6_layers();
        write_rmlp_pack(
            dir.path(),
            &["j1", "j2", "j3", "j4", "j5", "j6"],
            "rad",
            &layers,
            serde_json::json!({
                "vision": {
                    "source": "detector",
                    "layout": "top1_box_norm_v1",
                    "max_staleness_ms": 400,
                    "on_stale": "stop",
                    "require_model_generation": true
                }
            }),
        );
        let snap = PolicySnapshot::load_from_dir(dir.path()).unwrap();
        let contract = StrictPolicyContract::parse(&snap.policy_json_bytes).unwrap();
        let cfg = make_6dof_config();

        let infer_entered = Arc::new(Notify::new());
        let backend = Box::new(SlowGenBackend {
            infer_entered: infer_entered.clone(),
        });
        let prepared = PreparedPolicy::from_backend(snap, contract, backend, &cfg).unwrap();

        let sup = Arc::new(ArmSupervisor::new(cfg));
        let gen = sup.acquire_lease("conn-slow-gen").await.unwrap();
        sup.set_torque(true).await.unwrap();
        sup.tick(0.02).await;
        let epoch = sup.get_observation().await.epoch_id;
        let action_id = {
            let mut inner = sup.inner().write().await;
            inner
                .begin_policy_run(PolicyRunStart {
                    epoch_id: epoch,
                    connection_id: "conn-slow-gen".into(),
                    lease_generation: gen,
                    policy_id: "vis_slow_gen".into(),
                    artifact_digest: prepared.artifact_digest().to_string(),
                    backend: prepared.backend_name().to_string(),
                    mode: PolicyMode::Live,
                    sim_only: false,
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
                    i_am_at_the_arm: false,
                    verified_release_gate: None,
                })
                .unwrap()
        };

        let infer_entered_src = infer_entered.clone();
        let src = MockVisionSource::new_async(move |after, _| {
            let infer_entered_src = infer_entered_src.clone();
            Box::pin(async move {
                if after == 0 {
                    return Ok(Some(PolicyVisionFrame {
                        frame_id: 1,
                        captured_at: Some(Instant::now()),
                        detections: vec![make_detection("cube", 0.9, [0.2, 0.2, 0.4, 0.4])],
                        model_generation: Some(1),
                    }));
                }
                if after == 1 {
                    infer_entered_src.notified().await;
                    return Ok(Some(PolicyVisionFrame {
                        frame_id: 2,
                        captured_at: Some(Instant::now()),
                        detections: vec![make_detection("cube", 0.9, [0.2, 0.2, 0.4, 0.4])],
                        model_generation: Some(2),
                    }));
                }
                tokio::time::sleep(Duration::from_millis(20)).await;
                Ok(None)
            })
        });

        let runner = spawn_policy_runner(sup.clone(), action_id, prepared, 25.0, None, Some(src), None);
        let _ = runner.await;

        let inner = sup.inner().read().await;
        assert!(
            inner.servo_target.is_none(),
            "target computed on old generation must not be committed after generation switch"
        );
        assert_eq!(inner.last_action.as_ref().unwrap().state, "failed");
        let pol = inner.policy_observation().unwrap();
        assert_eq!(pol.stop_reason.as_deref(), Some("VISION_GENERATION_CHANGED"));
        assert_eq!(pol.steps, 0);
    }

    #[tokio::test]
    async fn http_detector_invalid_out_of_range_score_rejected_before_ranking() {
        use armd::grasp::HttpDetectionSource;
        use armd::policy_runtime::{
            extract_top1_box_norm_v1, DetectionSourceAdapter, VisionFeatureSpec,
        };
        use axum::{routing::get, Json, Router};

        let app = Router::new().route(
            "/api/results/latest",
            get(|| async {
                Json(serde_json::json!({
                    "frame_id": 1,
                    "age_ms": 5.0,
                    "width": 640,
                    "height": 360,
                    "model_generation": 1,
                    "detections": [
                        {"label": "cube", "score": 2.0, "box_norm": [0.0, 0.0, 0.1, 0.1]},
                        {"label": "cube", "score": 0.7, "box_norm": [0.2, 0.3, 0.6, 0.5]}
                    ]
                }))
            }),
        );
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr: SocketAddr = listener.local_addr().unwrap();
        let server = tokio::spawn(async move {
            let _ = axum::serve(listener, app).await;
        });

        let http_src = Arc::new(HttpDetectionSource {
            upstream: DetectorUpstream::parse(&format!("http://{addr}")).unwrap(),
            image_size_px: [640.0, 360.0],
        });
        let adapter = DetectionSourceAdapter(http_src);
        let frame = adapter
            .next_frame(0, 200)
            .await
            .unwrap()
            .expect("expected frame from local HTTP detector");
        server.abort();

        assert_eq!(frame.detections.len(), 2);
        // grasp.rs compatibility keeps clamped score=1.0, while raw_score_valid preserves rejection.
        assert!((frame.detections[0].score - 1.0).abs() < 1e-9);
        assert!(!frame.detections[0].raw_score_valid);
        assert!((frame.detections[1].score - 0.7).abs() < 1e-9);
        assert!(frame.detections[1].raw_score_valid);

        let spec = VisionFeatureSpec {
            source: VisionSourceKind::Detector,
            layout: VisionLayout::Top1BoxNormV1,
            target_label: Some("cube".into()),
            min_score: 0.3,
            max_staleness_ms: 200,
            max_sync_skew_ms: 200,
            on_target_lost: VisionTargetLostPolicy::Stop,
            on_stale: VisionStalePolicy::Stop,
            require_model_generation: false,
        };
        let feats = extract_top1_box_norm_v1(&frame, &spec)
            .expect("valid detection B should be selected after skipping out-of-range A");
        let expected = [1.0f32, 0.4, 0.4, 0.4, 0.2, 0.7];
        for i in 0..6 {
            assert!(
                (feats[i] - expected[i]).abs() < 1e-6,
                "feats[{i}]={} vs expected={}",
                feats[i],
                expected[i]
            );
        }
    }

    #[tokio::test]
    async fn vision_hold_clears_outstanding_live_servo_target() {
        let dir = tempdir().unwrap();
        let layers = make_12_to_6_layers();
        write_rmlp_pack(
            dir.path(),
            &["j1", "j2", "j3", "j4", "j5", "j6"],
            "rad",
            &layers,
            serde_json::json!({
                "vision": {
                    "source": "detector",
                    "layout": "top1_box_norm_v1",
                    "target_label": "cube",
                    "max_staleness_ms": 500,
                    "on_target_lost": "hold",
                    "require_model_generation": false
                }
            }),
        );
        let cfg = make_6dof_config();
        let prepared = prepare_policy(dir.path(), &cfg).unwrap();

        let sup = Arc::new(ArmSupervisor::new(cfg));
        let gen = sup.acquire_lease("conn-vis-hold-clear").await.unwrap();
        sup.set_torque(true).await.unwrap();
        sup.tick(0.02).await;
        let epoch = sup.get_observation().await.epoch_id;

        let action_id = {
            let mut inner = sup.inner().write().await;
            inner
                .begin_policy_run(PolicyRunStart {
                    epoch_id: epoch,
                    connection_id: "conn-vis-hold-clear".into(),
                    lease_generation: gen,
                    policy_id: "vis_hold_clear".into(),
                    artifact_digest: prepared.artifact_digest().to_string(),
                    backend: prepared.backend_name().to_string(),
                    mode: PolicyMode::Live,
                    sim_only: false,
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
                    i_am_at_the_arm: false,
                    verified_release_gate: None,
                })
                .unwrap()
        };

        let stage = Arc::new(AtomicUsize::new(0));
        let stage_c = stage.clone();
        let src = MockVisionSource::new_async(move |after, _| {
            let stage_c = stage_c.clone();
            Box::pin(async move {
                if stage_c.load(Ordering::SeqCst) == 0 {
                    if after >= 1 {
                        tokio::time::sleep(Duration::from_millis(10)).await;
                        return Ok(None);
                    }
                    return Ok(Some(PolicyVisionFrame {
                        frame_id: 1,
                        captured_at: Some(Instant::now()),
                        detections: vec![make_detection("cube", 0.9, [0.2, 0.3, 0.6, 0.5])],
                        model_generation: Some(1),
                    }));
                }
                tokio::time::sleep(Duration::from_millis(5)).await;
                Ok(Some(PolicyVisionFrame {
                    frame_id: after + 1,
                    captured_at: Some(Instant::now()),
                    detections: Vec::new(),
                    model_generation: Some(1),
                }))
            })
        });

        let runner = spawn_policy_runner(sup.clone(), action_id, prepared, 50.0, None, Some(src), None);

        let mut saw_live_target = false;
        for _ in 0..100 {
            {
                let inner = sup.inner().read().await;
                if inner.servo_target.is_some()
                    && inner.policy_observation().is_some_and(|p| p.active && p.steps >= 1)
                {
                    saw_live_target = true;
                    break;
                }
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        assert!(saw_live_target, "expected first frame with target to set servo_target");

        stage.store(1, Ordering::SeqCst);

        let mut cleared_while_active = false;
        for _ in 0..100 {
            {
                let inner = sup.inner().read().await;
                if inner.servo_target.is_none() {
                    assert!(
                        inner.policy_run_active(),
                        "policy_run_active() must remain true on Hold"
                    );
                    cleared_while_active = true;
                    break;
                }
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        assert!(
            cleared_while_active,
            "expected Hold on empty detections to clear servo_target while policy remains active"
        );

        {
            let mut inner = sup.inner().write().await;
            assert!(inner.stop_policy_run(Some(action_id), "STOPPED"));
        }
        let _ = runner.await;
    }

    #[tokio::test]
    async fn http_detector_missing_score_is_rejected_even_with_min_score_zero() {
        use armd::grasp::HttpDetectionSource;
        use armd::policy_runtime::{
            extract_top1_box_norm_v1, DetectionSourceAdapter, VisionFeatureSpec,
        };
        use axum::{routing::get, Json, Router};

        let app = Router::new().route(
            "/api/results/latest",
            get(|| async {
                Json(serde_json::json!({
                    "frame_id": 1,
                    "age_ms": 5.0,
                    "width": 640,
                    "height": 360,
                    "model_generation": 1,
                    "detections": [
                        {"label": "cube", "box_norm": [0.0, 0.0, 0.1, 0.1]},
                        {"label": "cube", "score": 0.0, "box_norm": [0.2, 0.3, 0.6, 0.5]}
                    ]
                }))
            }),
        );
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr: SocketAddr = listener.local_addr().unwrap();
        let server = tokio::spawn(async move {
            let _ = axum::serve(listener, app).await;
        });

        let http_src = Arc::new(HttpDetectionSource {
            upstream: DetectorUpstream::parse(&format!("http://{addr}")).unwrap(),
            image_size_px: [640.0, 360.0],
        });
        let adapter = DetectionSourceAdapter(http_src);
        let frame = adapter
            .next_frame(0, 200)
            .await
            .unwrap()
            .expect("expected frame from local HTTP detector");
        server.abort();

        assert_eq!(frame.detections.len(), 2);
        assert!(!frame.detections[0].raw_score_valid);
        assert!(frame.detections[1].raw_score_valid);
        assert!((frame.detections[1].score - 0.0).abs() < 1e-9);

        let spec = VisionFeatureSpec {
            source: VisionSourceKind::Detector,
            layout: VisionLayout::Top1BoxNormV1,
            target_label: Some("cube".into()),
            min_score: 0.0,
            max_staleness_ms: 200,
            max_sync_skew_ms: 200,
            on_target_lost: VisionTargetLostPolicy::Stop,
            on_stale: VisionStalePolicy::Stop,
            require_model_generation: false,
        };
        let feats = extract_top1_box_norm_v1(&frame, &spec)
            .expect("detection B with explicit score 0.0 should be selected after skipping missing-score A");
        let expected = [1.0f32, 0.4, 0.4, 0.4, 0.2, 0.0];
        for i in 0..6 {
            assert!(
                (feats[i] - expected[i]).abs() < 1e-6,
                "feats[{i}]={} vs expected={}",
                feats[i],
                expected[i]
            );
        }
    }
}

mod phase3_release_gate_tests {
    use super::*;
    use armd::policy_runtime::AuthorizationTier;
    use std::sync::atomic::AtomicBool;

    fn make_profiled_config(profile_id: Option<&str>) -> ArmConfig {
        let mut cfg = make_6dof_config();
        cfg.profile_id = profile_id.map(str::to_string);
        cfg
    }

    fn make_contract(unit_mode: &str) -> StrictPolicyContract {
        let pj = serde_json::json!({
            "format": "rebot-onnx-policy/v1",
            "name": "rg_policy",
            "unit_mode": unit_mode,
            "action_dim": 6,
            "action_space": "joint_positions",
            "joint_names": ["j1", "j2", "j3", "j4", "j5", "j6"],
            "model_file": "model.rmlp"
        });
        StrictPolicyContract::parse(&serde_json::to_vec(&pj).unwrap()).unwrap()
    }

    pub(super) fn make_valid_gate(
        policy_sha256: &str,
        tier: &str,
        profile_id: &str,
        fps: u64,
        max_speed_scale: f64,
        unit_mode: &str,
    ) -> serde_json::Value {
        let mut gate = serde_json::json!({
            "schema_version": "rebot-release-gate/v1",
            "authorization_tier": tier,
            "policy_sha256": policy_sha256,
            "scene_id": "pcb_suction",
            "tool_id": "suction",
            "profile_id": profile_id,
            "calibration_digest": "c".repeat(64),
            "runner_config": {
                "fps": fps,
                "max_speed_scale": max_speed_scale,
                "unit_mode": unit_mode,
                "action_space": "joint_positions"
            },
            "sim_eval": {
                "seeds_total": 10,
                "seeds_passed": 9,
                "min_success_rate": 0.8,
                "collisions": 0,
                "keepout_violations": 0
            },
            "prerequisites": {
                "calibration_verified": true,
                "power_cut_witnessed": true,
                "shadow_run_passed": true
            }
        });
        if tier == "production_release" {
            gate["supervised_real_run_record"] = serde_json::json!({
                "run_id": "sup-run-001",
                "passed": true,
                "policy_sha256": policy_sha256
            });
        }
        gate
    }

    #[test]
    fn release_gate_strict_validation_cases() {
        let sha = "a1b2c3d4".repeat(8);
        let cfg = make_profiled_config(Some("rebot-dm"));
        let contract = make_contract("rad");

        // 1. Valid supervised_first_run (including uppercase SHA + alias profile "b601dm")
        let deployment = ActiveDeploymentContext {
            scene_id: "pcb_suction".into(),
            tool_id: "suction".into(),
            calibration_digest: "c".repeat(64),
        };
        let gate_sup = make_valid_gate(
            &sha.to_ascii_uppercase(),
            "supervised_first_run",
            "b601dm",
            20,
            0.3,
            "rad",
        );
        let verified_sup = ReleaseGate::parse_and_verify(
            &gate_sup,
            &sha,
            &cfg,
            &contract,
            20.0,
            &deployment,
        )
        .unwrap();
        assert_eq!(
            verified_sup.authorization_tier(),
            AuthorizationTier::SupervisedFirstRun
        );
        assert_eq!(verified_sup.policy_sha256(), sha);
        assert_eq!(verified_sup.canonical_profile_id(), "b601dm");
        assert!((verified_sup.max_speed_scale() - 0.3).abs() < 1e-12);
        assert_eq!(verified_sup.fps(), 20);
        assert_eq!(verified_sup.scene_id(), "pcb_suction");
        assert_eq!(verified_sup.tool_id(), "suction");
        assert_eq!(verified_sup.calibration_digest(), "c".repeat(64));

        // 2. Valid production_release (allows max_speed_scale > 0.3)
        let gate_prod = make_valid_gate(
            &sha,
            "production_release",
            "rebot_b601_dm",
            25,
            0.8,
            "rad",
        );
        let verified_prod = ReleaseGate::parse_and_verify(
            &gate_prod,
            &sha,
            &cfg,
            &contract,
            25.0,
            &deployment,
        )
        .unwrap();
        assert_eq!(
            verified_prod.authorization_tier(),
            AuthorizationTier::ProductionRelease
        );

        // 3. Rejected digest mismatch
        let mut bad_digest = gate_sup.clone();
        bad_digest["policy_sha256"] = serde_json::json!("f".repeat(64));
        assert!(matches!(
            ReleaseGate::parse_and_verify(&bad_digest, &sha, &cfg, &contract, 20.0, &deployment),
            Err(PolicyError::ReleaseGate(_))
        ));

        // 4. Rejected profile mismatch & empty config profile
        let mut bad_profile = gate_sup.clone();
        bad_profile["profile_id"] = serde_json::json!("so-arm101");
        assert!(matches!(
            ReleaseGate::parse_and_verify(&bad_profile, &sha, &cfg, &contract, 20.0, &deployment),
            Err(PolicyError::ReleaseGate(_))
        ));
        assert!(matches!(
            ReleaseGate::parse_and_verify(
                &gate_sup,
                &sha,
                &make_profiled_config(None),
                &contract,
                20.0,
                &deployment,
            ),
            Err(PolicyError::ReleaseGate(_))
        ));

        // 5. Rejected sim_eval.collisions > 0 & keepout_violations > 0 & unmet success rate
        let mut bad_coll = gate_sup.clone();
        bad_coll["sim_eval"]["collisions"] = serde_json::json!(1);
        assert!(ReleaseGate::parse_and_verify(&bad_coll, &sha, &cfg, &contract, 20.0, &deployment).is_err());

        let mut bad_keepout = gate_sup.clone();
        bad_keepout["sim_eval"]["keepout_violations"] = serde_json::json!(1);
        assert!(ReleaseGate::parse_and_verify(&bad_keepout, &sha, &cfg, &contract, 20.0, &deployment).is_err());

        let mut bad_rate = gate_sup.clone();
        bad_rate["sim_eval"]["seeds_passed"] = serde_json::json!(7);
        assert!(ReleaseGate::parse_and_verify(&bad_rate, &sha, &cfg, &contract, 20.0, &deployment).is_err());

        // 6. Rejected seeds_total < 10
        let mut bad_seeds = gate_sup.clone();
        bad_seeds["sim_eval"]["seeds_total"] = serde_json::json!(9);
        bad_seeds["sim_eval"]["seeds_passed"] = serde_json::json!(9);
        assert!(ReleaseGate::parse_and_verify(&bad_seeds, &sha, &cfg, &contract, 20.0, &deployment).is_err());

        // 7. Rejected supervised_first_run max_speed_scale > 0.3
        let mut bad_speed = gate_sup.clone();
        bad_speed["runner_config"]["max_speed_scale"] = serde_json::json!(0.35);
        assert!(ReleaseGate::parse_and_verify(&bad_speed, &sha, &cfg, &contract, 20.0, &deployment).is_err());

        // 8. Rejected missing prerequisites
        for key in [
            "calibration_verified",
            "power_cut_witnessed",
            "shadow_run_passed",
        ] {
            let mut bad_pre = gate_sup.clone();
            bad_pre["prerequisites"][key] = serde_json::json!(false);
            assert!(
                ReleaseGate::parse_and_verify(&bad_pre, &sha, &cfg, &contract, 20.0, &deployment).is_err(),
                "expected failure when prerequisites.{key} == false"
            );
        }

        // 9. A caller-supplied supervised_real_run_record is ignored (missing, mismatched or
        // failed all parse the same): production is authorized by the device ledger only.
        let mut missing_rec = gate_prod.clone();
        missing_rec.as_object_mut().unwrap().remove("supervised_real_run_record");
        assert!(ReleaseGate::parse_and_verify(&missing_rec, &sha, &cfg, &contract, 25.0, &deployment).is_ok());

        let mut mismatch_rec = gate_prod.clone();
        mismatch_rec["supervised_real_run_record"]["policy_sha256"] =
            serde_json::json!("0".repeat(64));
        assert!(ReleaseGate::parse_and_verify(&mismatch_rec, &sha, &cfg, &contract, 25.0, &deployment).is_ok());

        let mut failed_rec = gate_prod.clone();
        failed_rec["supervised_real_run_record"]["passed"] = serde_json::json!(false);
        assert!(ReleaseGate::parse_and_verify(&failed_rec, &sha, &cfg, &contract, 25.0, &deployment).is_ok());

        // 10. Rejected fps mismatch, non-integer fps, unit_mode mismatch, action_space mismatch
        assert!(ReleaseGate::parse_and_verify(&gate_sup, &sha, &cfg, &contract, 25.0, &deployment).is_err());
        let mut float_fps = gate_sup.clone();
        float_fps["runner_config"]["fps"] = serde_json::json!(20.5);
        assert!(ReleaseGate::parse_and_verify(&float_fps, &sha, &cfg, &contract, 20.5, &deployment).is_err());

        let mut bad_units = gate_sup.clone();
        bad_units["runner_config"]["unit_mode"] = serde_json::json!("so101_norm");
        assert!(ReleaseGate::parse_and_verify(&bad_units, &sha, &cfg, &contract, 20.0, &deployment).is_err());

        let mut bad_space = gate_sup.clone();
        bad_space["runner_config"]["action_space"] = serde_json::json!("joint_velocities");
        assert!(ReleaseGate::parse_and_verify(&bad_space, &sha, &cfg, &contract, 20.0, &deployment).is_err());
    }

    #[tokio::test]
    async fn real_driver_live_requires_operator_confirmation_release_gate_and_confirmed_feedback() {
        struct ToggleFeedbackRealDriver {
            q: Vec<f32>,
            torqued: bool,
            feedback_valid: Arc<AtomicBool>,
        }
        impl ArmDriver for ToggleFeedbackRealDriver {
            fn capabilities(&self) -> DriverCapabilities {
                DriverCapabilities {
                    velocity_feedback: true,
                    torque_feedback: false,
                    temperature_feedback: true,
                    torque_feedforward: false,
                    enable_feedback: true,
                    load_feedback: false,
                }
            }
            fn enable_state(&self) -> EnableState {
                if self.torqued {
                    EnableState::Enabled
                } else {
                    EnableState::Disabled
                }
            }
            fn is_feedback_valid(&self) -> bool {
                self.feedback_valid.load(Ordering::SeqCst)
            }
            fn feedback_ages(&self) -> Vec<Option<Duration>> {
                vec![Some(Duration::ZERO); self.q.len()]
            }
            fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
                Ok(self
                    .q
                    .iter()
                    .map(|&p| JointState {
                        position: p,
                        velocity: 0.0,
                        torque: 0.0,
                        temperature: 30.0,
                    })
                    .collect())
            }
            fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
                for (i, t) in targets.iter().enumerate().take(self.q.len()) {
                    self.q[i] = t.position;
                }
                Ok(())
            }
            fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
                self.torqued = enabled;
                Ok(())
            }
        }

        let dir = tempdir().unwrap();
        let target = [0.12f32, -0.06, 0.04, 0.0, 0.0, 0.0];
        let layers = vec![MlpLayerSpec::new(
            6,
            6,
            Activation::None,
            vec![0.0; 36],
            target.to_vec(),
        )];
        let digest = write_rmlp_pack(
            dir.path(),
            &["j1", "j2", "j3", "j4", "j5", "j6"],
            "rad",
            &layers,
            serde_json::json!({}),
        );

        let mut cfg = make_6dof_config();
        cfg.profile_id = Some("so-arm101".into());
        let prepared = prepare_policy(dir.path(), &cfg).unwrap();
        let mut raw_gate = make_valid_gate(
            &digest,
            "supervised_first_run",
            "so101",
            25,
            0.3,
            "rad",
        );
        raw_gate["calibration_digest"] = serde_json::json!(EMPTY_CALIBRATION_DIGEST);
        let verified_gate = ReleaseGate::parse_and_verify(
            &raw_gate,
            &digest,
            &cfg,
            prepared.contract(),
            25.0,
            &ActiveDeploymentContext {
                scene_id: "pcb_suction".into(),
                tool_id: "suction".into(),
                calibration_digest: EMPTY_CALIBRATION_DIGEST.into(),
            },
        )
        .unwrap();

        let drv_fb_valid = Arc::new(AtomicBool::new(true));
        let sup = Arc::new(
            ArmSupervisor::new(cfg).with_driver(Box::new(ToggleFeedbackRealDriver {
                q: vec![0.0; 6],
                torqued: false,
                feedback_valid: drv_fb_valid.clone(),
            })),
        );
        sup.set_follower("so-arm101").await;

        let gen = sup.acquire_lease("conn-triple").await.unwrap();
        sup.set_torque(true).await.unwrap();
        sup.tick(0.02).await;
        let epoch = sup.get_observation().await.epoch_id;

        {
            let mut inner = sup.inner().write().await;

            // 1. Live without i_am_at_the_arm -> OPERATOR_CONFIRMATION_REQUIRED (403)
            let err1 = inner
                .begin_policy_run(PolicyRunStart {
                    epoch_id: epoch,
                    connection_id: "conn-triple".into(),
                    lease_generation: gen,
                    policy_id: "p_triple".into(),
                    artifact_digest: digest.clone(),
                    backend: "mlp".into(),
                    mode: PolicyMode::Live,
                    sim_only: false,
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
                    i_am_at_the_arm: false,
                    verified_release_gate: Some(verified_gate.clone()),
                })
                .unwrap_err();
            assert!(
                matches!(err1, ArmError::Rejected { status: 403, ref code, .. } if code == "OPERATOR_CONFIRMATION_REQUIRED"),
                "expected OPERATOR_CONFIRMATION_REQUIRED, got {err1:?}"
            );

            // 2. Live with i_am_at_the_arm: true but no release_gate (or mismatched digest) -> RELEASE_GATE_REQUIRED (403)
            let err2 = inner
                .begin_policy_run(PolicyRunStart {
                    epoch_id: epoch,
                    connection_id: "conn-triple".into(),
                    lease_generation: gen,
                    policy_id: "p_triple".into(),
                    artifact_digest: digest.clone(),
                    backend: "mlp".into(),
                    mode: PolicyMode::Live,
                    sim_only: false,
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
                    i_am_at_the_arm: true,
                    verified_release_gate: None,
                })
                .unwrap_err();
            assert!(
                matches!(err2, ArmError::Rejected { status: 403, ref code, .. } if code == "RELEASE_GATE_REQUIRED"),
                "expected RELEASE_GATE_REQUIRED, got {err2:?}"
            );

            let err2b = inner
                .begin_policy_run(PolicyRunStart {
                    epoch_id: epoch,
                    connection_id: "conn-triple".into(),
                    lease_generation: gen,
                    policy_id: "p_triple".into(),
                    artifact_digest: "0".repeat(64),
                    backend: "mlp".into(),
                    mode: PolicyMode::Live,
                    sim_only: false,
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
                    i_am_at_the_arm: true,
                    verified_release_gate: Some(verified_gate.clone()),
                })
                .unwrap_err();
            assert!(
                matches!(err2b, ArmError::Rejected { status: 403, ref code, .. } if code == "RELEASE_GATE_REQUIRED"),
                "expected RELEASE_GATE_REQUIRED on digest mismatch, got {err2b:?}"
            );

            // 3. Live with i_am_at_the_arm: true + valid release_gate, but unconfirmed feedback -> FEEDBACK_UNCONFIRMED (409)
            inner.feedback_numeric_valid = false;
            let err3a = inner
                .begin_policy_run(PolicyRunStart {
                    epoch_id: epoch,
                    connection_id: "conn-triple".into(),
                    lease_generation: gen,
                    policy_id: "p_triple".into(),
                    artifact_digest: digest.clone(),
                    backend: "mlp".into(),
                    mode: PolicyMode::Live,
                    sim_only: false,
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
                    i_am_at_the_arm: true,
                    verified_release_gate: Some(verified_gate.clone()),
                })
                .unwrap_err();
            assert!(
                matches!(err3a, ArmError::Rejected { status: 409, ref code, .. } if code == "FEEDBACK_UNCONFIRMED"),
                "expected FEEDBACK_UNCONFIRMED when feedback_numeric_valid=false, got {err3a:?}"
            );
            inner.feedback_numeric_valid = true;

            drv_fb_valid.store(false, Ordering::SeqCst);
            let err3b = inner
                .begin_policy_run(PolicyRunStart {
                    epoch_id: epoch,
                    connection_id: "conn-triple".into(),
                    lease_generation: gen,
                    policy_id: "p_triple".into(),
                    artifact_digest: digest.clone(),
                    backend: "mlp".into(),
                    mode: PolicyMode::Live,
                    sim_only: false,
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
                    i_am_at_the_arm: true,
                    verified_release_gate: Some(verified_gate.clone()),
                })
                .unwrap_err();
            assert!(
                matches!(err3b, ArmError::Rejected { status: 409, ref code, .. } if code == "FEEDBACK_UNCONFIRMED"),
                "expected FEEDBACK_UNCONFIRMED when driver.is_feedback_valid()=false, got {err3b:?}"
            );
            drv_fb_valid.store(true, Ordering::SeqCst);
        }

        // 4. All 3 satisfied -> SUCCEEDS and executes cleanly!
        let action_id = {
            let mut inner = sup.inner().write().await;
            inner
                .begin_policy_run(PolicyRunStart {
                    epoch_id: epoch,
                    connection_id: "conn-triple".into(),
                    lease_generation: gen,
                    policy_id: "p_triple".into(),
                    artifact_digest: digest.clone(),
                    backend: "mlp".into(),
                    mode: PolicyMode::Live,
                    sim_only: false,
                    max_first_step_rad: prepared.contract().max_first_step_rad,
                    max_step_rad: prepared.contract().max_step_rad,
                    tag: None,
                    i_am_at_the_arm: true,
                    verified_release_gate: Some(verified_gate),
                })
                .unwrap()
        };

        let runner = spawn_policy_runner(
            sup.clone(),
            action_id,
            prepared,
            25.0,
            compute_policy_deadline(Some(0.30)).unwrap(),
            None,
            None,
        );
        for _ in 0..16 {
            tokio::time::sleep(Duration::from_millis(20)).await;
            sup.tick(0.02).await;
        }
        let _ = runner.await;

        let obs = sup.get_observation().await;
        assert!(obs.positions[0] > 0.04, "expected j1 to move, got {}", obs.positions[0]);
        let p = obs.policy.unwrap();
        assert!(!p.active);
        assert!(p.steps > 0);
        assert_eq!(p.stop_reason.as_deref(), Some("MAX_SECONDS"));
    }

    #[tokio::test]
    async fn ws_real_driver_live_policy_with_inline_and_pack_release_gate() {
        let sandbox_dir = tempdir().unwrap();
        let sandbox = Arc::new(SandboxState::new(sandbox_dir.path().to_path_buf()));
        let mut cfg = make_6dof_config();
        cfg.profile_id = Some("b601dm".into());
        let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(DummyRealDriver::new(6))));
        sup.set_follower("rebot-dm").await;
        let router = create_router_with_sandbox(sup.clone(), OperatorAuth::disabled(), sandbox.clone());

        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr: SocketAddr = listener.local_addr().unwrap();
        tokio::spawn(async move {
            axum::serve(listener, router).await.unwrap();
        });

        let tick_sup = sup.clone();
        let tick_stop = Arc::new(Mutex::new(false));
        let tick_stop_c = tick_stop.clone();
        let tick_task = tokio::spawn(async move {
            loop {
                if *tick_stop_c.lock().unwrap() {
                    break;
                }
                tick_sup.tick(0.02).await;
                tokio::time::sleep(Duration::from_millis(20)).await;
            }
        });

        let client = reqwest::Client::new();

        // Check capabilities publish ["shadow", "live_gated"] on real hardware.
        let schema: serde_json::Value = client
            .get(format!("http://{addr}/api/v1/observation/schema"))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!(
            schema["capabilities"]["policy_runtime"]["modes"],
            serde_json::json!(["shadow", "live_gated"])
        );

        // Upload policy pack 1 (no release_gate.json on disk; tested with inline release_gate).
        let pack1_id = "ws_real_inline";
        upload_rmlp_pack(
            &client,
            addr,
            pack1_id,
            vec![0.10, -0.05, 0.0, 0.0, 0.0, 0.0],
            serde_json::json!({}),
        )
        .await;
        let pack1_dir = sandbox.policy_dir(pack1_id).unwrap();
        let digest1 = PolicySnapshot::load_from_dir(&pack1_dir).unwrap().release_digest;

        let (mut ws, _) = connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
        ws.send(Message::Text(
            serde_json::json!({"type": "acquire_lease", "client_id": "ws_real_test"})
                .to_string()
                .into(),
        ))
        .await
        .unwrap();
        let gen = loop {
            let msg: serde_json::Value =
                serde_json::from_str(&ws.next().await.unwrap().unwrap().into_text().unwrap()).unwrap();
            if msg["type"] == "lease_granted" {
                break msg["generation"].as_u64().unwrap();
            }
        };

        async fn send_cmd(
            ws: &mut tokio_tungstenite::WebSocketStream<
                tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
            >,
            req_id: u64,
            gen: u64,
            epoch: u64,
            path: &str,
            body: serde_json::Value,
        ) -> serde_json::Value {
            ws.send(Message::Text(
                serde_json::json!({
                    "type": "command",
                    "request_id": req_id,
                    "generation": gen,
                    "epoch_id": epoch,
                    "path": path,
                    "body": body,
                })
                .to_string()
                .into(),
            ))
            .await
            .unwrap();
            tokio::time::timeout(Duration::from_secs(3), async {
                loop {
                    let txt = ws.next().await.unwrap().unwrap().into_text().unwrap();
                    let v: serde_json::Value = serde_json::from_str(&txt).unwrap();
                    if v["type"] == "command_ack" || v["type"] == "command_reject" {
                        return v;
                    }
                }
            })
            .await
            .unwrap()
        }

        let ack_torque = send_cmd(
            &mut ws,
            1,
            gen,
            1,
            "/api/v1/torque",
            serde_json::json!({"enable": true}),
        )
        .await;
        assert_eq!(ack_torque["type"], "command_ack");

        // 1. Missing i_am_at_the_arm -> OPERATOR_CONFIRMATION_REQUIRED
        let rej_op = send_cmd(
            &mut ws,
            2,
            gen,
            1,
            "/api/v1/action/policy/start",
            serde_json::json!({
                "policy_id": pack1_id,
                "mode": "live",
                "fps": 25.0
            }),
        )
        .await;
        assert_eq!(rej_op["type"], "command_reject");
        assert_eq!(rej_op["code"], "OPERATOR_CONFIRMATION_REQUIRED");

        // 2. i_am_at_the_arm: true without release_gate -> RELEASE_GATE_REQUIRED
        let rej_missing_gate = send_cmd(
            &mut ws,
            3,
            gen,
            1,
            "/api/v1/action/policy/start",
            serde_json::json!({
                "policy_id": pack1_id,
                "mode": "live",
                "fps": 25.0,
                "i_am_at_the_arm": true
            }),
        )
        .await;
        assert_eq!(rej_missing_gate["type"], "command_reject");
        assert_eq!(rej_missing_gate["code"], "RELEASE_GATE_REQUIRED");

        // 3. Invalid inline release_gate -> RELEASE_GATE_INVALID
        let bad_inline_gate = make_valid_gate(
            &"0".repeat(64),
            "supervised_first_run",
            "b601dm",
            25,
            0.3,
            "rad",
        );
        let rej_bad_gate = send_cmd(
            &mut ws,
            4,
            gen,
            1,
            "/api/v1/action/policy/start",
            serde_json::json!({
                "policy_id": pack1_id,
                "mode": "live",
                "fps": 25.0,
                "i_am_at_the_arm": true,
                "release_gate": bad_inline_gate
            }),
        )
        .await;
        assert_eq!(rej_bad_gate["type"], "command_reject");
        assert_eq!(rej_bad_gate["code"], "RELEASE_GATE_INVALID");

        // 4. Valid inline release_gate -> succeeds and moves arm!
        let mut valid_inline_gate = make_valid_gate(
            &digest1,
            "supervised_first_run",
            "rebot_dm",
            25,
            0.3,
            "rad",
        );
        valid_inline_gate["scene_id"] = serde_json::json!("default");
        valid_inline_gate["tool_id"] = serde_json::json!("default");
        valid_inline_gate["calibration_digest"] = serde_json::json!(EMPTY_CALIBRATION_DIGEST);
        let ack_inline = send_cmd(
            &mut ws,
            5,
            gen,
            1,
            "/api/v1/action/policy/start",
            serde_json::json!({
                "policy_id": pack1_id,
                "mode": "live_gated",
                "fps": 25.0,
                "max_seconds": 3.0,
                "i_am_at_the_arm": true,
                "release_gate": valid_inline_gate
            }),
        )
        .await;
        assert_eq!(ack_inline["type"], "command_ack", "{ack_inline}");

        let mut moved_inline = false;
        for _ in 0..30 {
            tokio::time::sleep(Duration::from_millis(25)).await;
            let obs = sup.get_observation().await;
            if obs.policy.as_ref().is_some_and(|p| p.steps > 0) && obs.positions[0] > 0.02 {
                moved_inline = true;
                break;
            }
        }
        assert!(moved_inline, "arm did not move with inline release_gate");

        let ack_stop1 = send_cmd(
            &mut ws,
            6,
            gen,
            1,
            "/api/v1/action/policy/stop",
            serde_json::json!({}),
        )
        .await;
        assert_eq!(ack_stop1["type"], "command_ack");

        // Wait for servo braking velocities to settle before starting the second live policy.
        for _ in 0..50 {
            if sup
                .inner()
                .read()
                .await
                .commanded_velocities
                .iter()
                .all(|v| v.abs() < 1e-3)
            {
                break;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }

        // 5. Pack-embedded release_gate.json (no inline release_gate) -> succeeds!
        let pack2_id = "ws_real_pack_gate";
        upload_rmlp_pack(
            &client,
            addr,
            pack2_id,
            vec![0.12, -0.05, 0.0, 0.0, 0.0, 0.0],
            serde_json::json!({}),
        )
        .await;
        let pack2_dir = sandbox.policy_dir(pack2_id).unwrap();
        let digest2 = PolicySnapshot::load_from_dir(&pack2_dir).unwrap().release_digest;
        let mut pack2_gate = make_valid_gate(
            &digest2,
            "production_release",
            "b601dm",
            25,
            0.7,
            "rad",
        );
        pack2_gate["scene_id"] = serde_json::json!("default");
        pack2_gate["tool_id"] = serde_json::json!("default");
        pack2_gate["calibration_digest"] = serde_json::json!(EMPTY_CALIBRATION_DIGEST);
        fs::write(
            pack2_dir.join("release_gate.json"),
            serde_json::to_vec_pretty(&pack2_gate).unwrap(),
        )
        .unwrap();

        // Production release without device-recorded supervised passes -> refused.
        let ledger_dir = tempdir().unwrap();
        let ledger = ledger_dir.path().join("policy_qualification.jsonl");
        sup.inner().write().await.policy_ledger_path = Some(ledger.clone());
        let rej_unqualified = send_cmd(
            &mut ws,
            70,
            gen,
            1,
            "/api/v1/action/policy/start",
            serde_json::json!({
                "policy_id": pack2_id,
                "mode": "live",
                "fps": 25.0,
                "max_seconds": 3.0,
                "i_am_at_the_arm": true
            }),
        )
        .await;
        assert_eq!(rej_unqualified["code"], "QUALIFICATION_REQUIRED", "{rej_unqualified}");
        // Records as armd writes them after supervised runs of this artifact and deployment.
        for i in 0..armd::policy_runtime::qualification::MIN_SUPERVISED_PASSES {
            armd::policy_runtime::qualification::append_record(
                &ledger,
                &armd::policy_runtime::qualification::QualificationRecord {
                    schema: armd::policy_runtime::qualification::QUALIFICATION_SCHEMA.into(),
                    recorded_unix_s: 0,
                    action_id: i as u64,
                    policy_sha256: digest2.to_ascii_lowercase(),
                    scene_id: "default".into(),
                    tool_id: "default".into(),
                    calibration_digest: EMPTY_CALIBRATION_DIGEST.into(),
                    profile_id: "b601dm".into(),
                    fps: 25,
                    tier: "supervised_first_run".into(),
                    max_speed_scale: 0.3,
                    steps: 10,
                    stop_reason: "MAX_SECONDS".into(),
                    passed: true,
                    policy_stats: None,
                },
            )
            .unwrap();
        }

        let ack_pack = send_cmd(
            &mut ws,
            71,
            gen,
            1,
            "/api/v1/action/policy/start",
            serde_json::json!({
                "policy_id": pack2_id,
                "mode": "live",
                "fps": 25.0,
                "max_seconds": 3.0,
                "i_am_at_the_arm": true
            }),
        )
        .await;
        assert_eq!(ack_pack["type"], "command_ack", "{ack_pack}");

        let mut moved_pack = false;
        for _ in 0..30 {
            tokio::time::sleep(Duration::from_millis(25)).await;
            let obs = sup.get_observation().await;
            if obs.policy.as_ref().is_some_and(|p| p.policy_id == pack2_id && p.steps > 0)
                && obs.positions[0] > 0.03
            {
                moved_pack = true;
                break;
            }
        }
        assert!(moved_pack, "arm did not move with pack release_gate.json");

        let ack_stop2 = send_cmd(
            &mut ws,
            72,
            gen,
            1,
            "/api/v1/action/policy/stop",
            serde_json::json!({}),
        )
        .await;
        assert_eq!(ack_stop2["type"], "command_ack");

        *tick_stop.lock().unwrap() = true;
        let _ = tick_task.await;
    }

    #[tokio::test]
    async fn real_driver_live_enforces_release_gate_max_speed_scale_during_ticks() {
        let mut cfg = make_6dof_config();
        cfg.profile_id = Some("so-arm101".into());
        // High acceleration so the velocity cap (0.3 * 2.0 = 0.6 rad/s) saturates quickly.
        for lim in &mut cfg.limits {
            lim.max_acceleration = 100.0;
        }
        let normal_servo_max_velocity = cfg.servo_limits()[0].max_velocity;
        assert!((normal_servo_max_velocity - 2.0).abs() < 1e-6);

        let sha = "a1b2c3d4".repeat(8);
        let contract = make_contract("rad");
        let mut raw_gate = make_valid_gate(
            &sha,
            "supervised_first_run",
            "so101",
            25,
            0.3,
            "rad",
        );
        raw_gate["calibration_digest"] = serde_json::json!(EMPTY_CALIBRATION_DIGEST);
        let verified_gate = ReleaseGate::parse_and_verify(
            &raw_gate,
            &sha,
            &cfg,
            &contract,
            25.0,
            &ActiveDeploymentContext {
                scene_id: "pcb_suction".into(),
                tool_id: "suction".into(),
                calibration_digest: EMPTY_CALIBRATION_DIGEST.into(),
            },
        )
        .unwrap();

        let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(DummyRealDriver::new(6))));
        sup.set_follower("so-arm101").await;

        let gen = sup.acquire_lease("conn-speed-cap").await.unwrap();
        sup.set_torque(true).await.unwrap();
        sup.tick(0.02).await;
        let epoch = sup.get_observation().await.epoch_id;

        let action_id = {
            let mut inner = sup.inner().write().await;
            let id = inner
                .begin_policy_run(PolicyRunStart {
                    epoch_id: epoch,
                    connection_id: "conn-speed-cap".into(),
                    lease_generation: gen,
                    policy_id: "speed_cap_policy".into(),
                    artifact_digest: sha.clone(),
                    backend: "mlp".into(),
                    mode: PolicyMode::Live,
                    sim_only: false,
                    max_first_step_rad: Some(0.25),
                    max_step_rad: Some(0.25),
                    tag: None,
                    i_am_at_the_arm: true,
                    verified_release_gate: Some(verified_gate),
                })
                .unwrap();
            let capped = inner.servo_limits();
            assert!(
                (capped[0].max_velocity - 0.3 * normal_servo_max_velocity).abs() < 1e-5,
                "expected servo_limits[0].max_velocity == {}, got {}",
                0.3 * normal_servo_max_velocity,
                capped[0].max_velocity
            );
            id
        };

        let dt = 0.02;
        let max_allowed_step = 0.3 * f64::from(normal_servo_max_velocity) * dt + 1e-4;
        let mut max_observed_step = 0.0_f64;

        for _ in 0..8 {
            let prev_q = f64::from(sup.get_observation().await.commanded_positions[0]);
            {
                let mut inner = sup.inner().write().await;
                inner
                    .policy_step(action_id, vec![0.20, 0.0, 0.0, 0.0, 0.0, 0.0], Some(0))
                    .unwrap();
            }
            let obs = sup.tick(dt).await;
            let next_q = f64::from(obs.commanded_positions[0]);
            let delta = (next_q - prev_q).abs();
            assert!(
                delta <= max_allowed_step,
                "tick displacement {delta:.6} exceeded 0.3 * normal_servo_max_velocity * dt ({max_allowed_step:.6})"
            );
            max_observed_step = max_observed_step.max(delta);
        }

        assert!(
            max_observed_step > 0.25 * f64::from(normal_servo_max_velocity) * dt,
            "expected arm to move at capped speed, max_observed_step={max_observed_step:.6}"
        );
    }

    #[tokio::test]
    async fn release_gate_rejects_scene_tool_or_calibration_digest_mismatch() {
        let sha = "a1b2c3d4".repeat(8);
        let cfg = make_profiled_config(Some("b601dm"));
        let contract = make_contract("rad");
        let mut gate = make_valid_gate(&sha, "supervised_first_run", "b601dm", 20, 0.3, "rad");
        gate["scene_id"] = serde_json::json!("default");
        gate["tool_id"] = serde_json::json!("default");
        gate["calibration_digest"] = serde_json::json!(EMPTY_CALIBRATION_DIGEST);

        let matching = ActiveDeploymentContext {
            scene_id: "default".into(),
            tool_id: "default".into(),
            calibration_digest: EMPTY_CALIBRATION_DIGEST.into(),
        };
        assert!(ReleaseGate::parse_and_verify(&gate, &sha, &cfg, &contract, 20.0, &matching).is_ok());

        let bad_scene = ActiveDeploymentContext {
            scene_id: "pcb_suction".into(),
            ..matching.clone()
        };
        assert!(matches!(
            ReleaseGate::parse_and_verify(&gate, &sha, &cfg, &contract, 20.0, &bad_scene),
            Err(PolicyError::ReleaseGate(msg)) if msg.contains("scene_id")
        ));

        let bad_tool = ActiveDeploymentContext {
            tool_id: "suction".into(),
            ..matching.clone()
        };
        assert!(matches!(
            ReleaseGate::parse_and_verify(&gate, &sha, &cfg, &contract, 20.0, &bad_tool),
            Err(PolicyError::ReleaseGate(msg)) if msg.contains("tool_id")
        ));

        let bad_calib = ActiveDeploymentContext {
            calibration_digest: "d".repeat(64),
            ..matching.clone()
        };
        assert!(matches!(
            ReleaseGate::parse_and_verify(&gate, &sha, &cfg, &contract, 20.0, &bad_calib),
            Err(PolicyError::ReleaseGate(msg)) if msg.contains("calibration_digest")
        ));

        // Also test mid-load sandbox scene switch invalidating the verified gate under WS lock.
        let sandbox_dir = tempdir().unwrap();
        let sandbox = Arc::new(SandboxState::new(sandbox_dir.path().to_path_buf()));
        sandbox.set_policy_load_delay_ms(120);
        let mut cfg_ws = make_6dof_config();
        cfg_ws.profile_id = Some("b601dm".into());
        let sup = Arc::new(ArmSupervisor::new(cfg_ws).with_driver(Box::new(DummyRealDriver::new(6))));
        sup.set_follower("rebot-dm").await;
        assert_eq!(
            sup.inner().read().await.active_calibration_digest(),
            EMPTY_CALIBRATION_DIGEST
        );

        let router = create_router_with_sandbox(sup.clone(), OperatorAuth::disabled(), sandbox.clone());
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr: SocketAddr = listener.local_addr().unwrap();
        tokio::spawn(async move {
            axum::serve(listener, router).await.unwrap();
        });

        let tick_sup = sup.clone();
        let tick_stop = Arc::new(Mutex::new(false));
        let tick_stop_c = tick_stop.clone();
        let tick_task = tokio::spawn(async move {
            loop {
                if *tick_stop_c.lock().unwrap() {
                    break;
                }
                tick_sup.tick(0.02).await;
                tokio::time::sleep(Duration::from_millis(20)).await;
            }
        });

        let client = reqwest::Client::new();
        upload_rmlp_pack(&client, addr, "ctx_switch_p", vec![0.05; 6], serde_json::json!({})).await;
        let digest = PolicySnapshot::load_from_dir(&sandbox.policy_dir("ctx_switch_p").unwrap())
            .unwrap()
            .release_digest;

        let (mut ws, _) = connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
        ws.send(Message::Text(
            serde_json::json!({"type": "acquire_lease", "client_id": "ctx_test"})
                .to_string()
                .into(),
        ))
        .await
        .unwrap();
        let gen = loop {
            let msg: serde_json::Value =
                serde_json::from_str(&ws.next().await.unwrap().unwrap().into_text().unwrap()).unwrap();
            if msg["type"] == "lease_granted" {
                break msg["generation"].as_u64().unwrap();
            }
        };

        ws.send(Message::Text(
            serde_json::json!({
                "type": "command",
                "request_id": 1,
                "generation": gen,
                "epoch_id": 1,
                "path": "/api/v1/torque",
                "body": {"enable": true}
            })
            .to_string()
            .into(),
        ))
        .await
        .unwrap();
        loop {
            let msg: serde_json::Value =
                serde_json::from_str(&ws.next().await.unwrap().unwrap().into_text().unwrap()).unwrap();
            if msg["type"] == "command_ack" {
                break;
            }
        }

        let mut gate_ws = make_valid_gate(&digest, "supervised_first_run", "b601dm", 20, 0.3, "rad");
        gate_ws["scene_id"] = serde_json::json!("default");
        gate_ws["tool_id"] = serde_json::json!("default");
        gate_ws["calibration_digest"] = serde_json::json!(EMPTY_CALIBRATION_DIGEST);

        ws.send(Message::Text(
            serde_json::json!({
                "type": "command",
                "request_id": 2,
                "generation": gen,
                "epoch_id": 1,
                "path": "/api/v1/action/policy/start",
                "body": {
                    "policy_id": "ctx_switch_p",
                    "mode": "live_gated",
                    "fps": 20.0,
                    "i_am_at_the_arm": true,
                    "release_gate": gate_ws
                }
            })
            .to_string()
            .into(),
        ))
        .await
        .unwrap();

        tokio::time::sleep(Duration::from_millis(30)).await;
        sandbox.active.write().await.active_scene = "pcb_suction".into();

        let rej = tokio::time::timeout(Duration::from_secs(3), async {
            loop {
                let txt = ws.next().await.unwrap().unwrap().into_text().unwrap();
                let v: serde_json::Value = serde_json::from_str(&txt).unwrap();
                if v["type"] == "command_ack" || v["type"] == "command_reject" {
                    return v;
                }
            }
        })
        .await
        .unwrap();
        assert_eq!(rej["type"], "command_reject", "{rej}");
        assert_eq!(rej["code"], "RELEASE_GATE_INVALID", "{rej}");

        *tick_stop.lock().unwrap() = true;
        let _ = tick_task.await;
    }

    #[tokio::test]
    async fn live_sim_mode_rejected_on_real_driver_even_with_valid_gate() {
        let req_sim = parse_policy_start_request(&serde_json::json!({
            "policy_id": "p1",
            "mode": "live_sim",
            "fps": 20.0
        }))
        .unwrap();
        assert_eq!(req_sim.mode, PolicyMode::Live);
        assert!(req_sim.sim_only);

        let req_live = parse_policy_start_request(&serde_json::json!({
            "policy_id": "p1",
            "mode": "live_gated",
            "fps": 20.0
        }))
        .unwrap();
        assert_eq!(req_live.mode, PolicyMode::Live);
        assert!(!req_live.sim_only);

        let sandbox_dir = tempdir().unwrap();
        let sandbox = Arc::new(SandboxState::new(sandbox_dir.path().to_path_buf()));
        let mut cfg = make_6dof_config();
        cfg.profile_id = Some("b601dm".into());
        let sup = Arc::new(ArmSupervisor::new(cfg.clone()).with_driver(Box::new(DummyRealDriver::new(6))));
        sup.set_follower("rebot-dm").await;
        let router = create_router_with_sandbox(sup.clone(), OperatorAuth::disabled(), sandbox.clone());

        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr: SocketAddr = listener.local_addr().unwrap();
        tokio::spawn(async move {
            axum::serve(listener, router).await.unwrap();
        });

        let tick_sup = sup.clone();
        let tick_stop = Arc::new(Mutex::new(false));
        let tick_stop_c = tick_stop.clone();
        let tick_task = tokio::spawn(async move {
            loop {
                if *tick_stop_c.lock().unwrap() {
                    break;
                }
                tick_sup.tick(0.02).await;
                tokio::time::sleep(Duration::from_millis(20)).await;
            }
        });

        let client = reqwest::Client::new();
        upload_rmlp_pack(&client, addr, "live_sim_p", vec![0.05; 6], serde_json::json!({})).await;
        let pack_dir = sandbox.policy_dir("live_sim_p").unwrap();
        let digest = PolicySnapshot::load_from_dir(&pack_dir).unwrap().release_digest;

        let mut gate_json = make_valid_gate(&digest, "supervised_first_run", "b601dm", 20, 0.3, "rad");
        gate_json["scene_id"] = serde_json::json!("default");
        gate_json["tool_id"] = serde_json::json!("default");
        gate_json["calibration_digest"] = serde_json::json!(EMPTY_CALIBRATION_DIGEST);
        let verified_gate = ReleaseGate::parse_and_verify(
            &gate_json,
            &digest,
            &cfg,
            &make_contract("rad"),
            20.0,
            &ActiveDeploymentContext {
                scene_id: "default".into(),
                tool_id: "default".into(),
                calibration_digest: EMPTY_CALIBRATION_DIGEST.into(),
            },
        )
        .unwrap();

        let (mut ws, _) = connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
        ws.send(Message::Text(
            serde_json::json!({"type": "acquire_lease", "client_id": "live_sim_test"})
                .to_string()
                .into(),
        ))
        .await
        .unwrap();
        let gen = loop {
            let msg: serde_json::Value =
                serde_json::from_str(&ws.next().await.unwrap().unwrap().into_text().unwrap()).unwrap();
            if msg["type"] == "lease_granted" {
                break msg["generation"].as_u64().unwrap();
            }
        };

        ws.send(Message::Text(
            serde_json::json!({
                "type": "command",
                "request_id": 1,
                "generation": gen,
                "epoch_id": 1,
                "path": "/api/v1/torque",
                "body": {"enable": true}
            })
            .to_string()
            .into(),
        ))
        .await
        .unwrap();
        loop {
            let msg: serde_json::Value =
                serde_json::from_str(&ws.next().await.unwrap().unwrap().into_text().unwrap()).unwrap();
            if msg["type"] == "command_ack" {
                break;
            }
        }

        // Direct SupervisorInner check: sim_only rejected even with i_am_at_the_arm and valid gate.
        {
            let mut inner = sup.inner().write().await;
            let err = inner
                .begin_policy_run(PolicyRunStart {
                    epoch_id: 1,
                    connection_id: "direct".into(),
                    lease_generation: gen,
                    policy_id: "live_sim_p".into(),
                    artifact_digest: digest.clone(),
                    backend: "mlp".into(),
                    mode: PolicyMode::Live,
                    sim_only: true,
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
                    i_am_at_the_arm: true,
                    verified_release_gate: Some(verified_gate),
                })
                .unwrap_err();
            assert!(
                matches!(err, ArmError::Rejected { status: 403, ref code, .. } if code == "POLICY_SIM_ONLY"),
                "expected POLICY_SIM_ONLY, got {err:?}"
            );
        }

        // WS command check: mode = "live_sim" rejected with POLICY_SIM_ONLY even with i_am_at_the_arm + valid gate.
        ws.send(Message::Text(
            serde_json::json!({
                "type": "command",
                "request_id": 2,
                "generation": gen,
                "epoch_id": 1,
                "path": "/api/v1/action/policy/start",
                "body": {
                    "policy_id": "live_sim_p",
                    "mode": "live_sim",
                    "fps": 20.0,
                    "i_am_at_the_arm": true,
                    "release_gate": gate_json
                }
            })
            .to_string()
            .into(),
        ))
        .await
        .unwrap();
        let rej = tokio::time::timeout(Duration::from_secs(3), async {
            loop {
                let txt = ws.next().await.unwrap().unwrap().into_text().unwrap();
                let v: serde_json::Value = serde_json::from_str(&txt).unwrap();
                if v["type"] == "command_ack" || v["type"] == "command_reject" {
                    return v;
                }
            }
        })
        .await
        .unwrap();
        assert_eq!(rej["type"], "command_reject", "{rej}");
        assert_eq!(rej["status"], 403, "{rej}");
        assert_eq!(rej["code"], "POLICY_SIM_ONLY", "{rej}");

        *tick_stop.lock().unwrap() = true;
        let _ = tick_task.await;
    }

    #[test]
    fn release_gate_rejects_zero_seeds_passed_even_with_tiny_min_success_rate() {
        let sha = "a1b2c3d4".repeat(8);
        let cfg = make_profiled_config(Some("b601dm"));
        let contract = make_contract("rad");
        let deployment = ActiveDeploymentContext {
            scene_id: "pcb_suction".into(),
            tool_id: "suction".into(),
            calibration_digest: "c".repeat(64),
        };
        let mut gate = make_valid_gate(&sha, "supervised_first_run", "b601dm", 20, 0.3, "rad");
        gate["sim_eval"]["seeds_passed"] = serde_json::json!(0);
        gate["sim_eval"]["seeds_total"] = serde_json::json!(10);
        gate["sim_eval"]["min_success_rate"] = serde_json::json!(1e-13);

        let err = ReleaseGate::parse_and_verify(&gate, &sha, &cfg, &contract, 20.0, &deployment)
            .unwrap_err();
        assert!(matches!(err, PolicyError::ReleaseGate(_)), "{err:?}");

        // Also ensure no 1e-12 tolerance lets 7/10 pass min_success_rate = 0.7 + 5e-13.
        let mut borderline = make_valid_gate(&sha, "supervised_first_run", "b601dm", 20, 0.3, "rad");
        borderline["sim_eval"]["seeds_passed"] = serde_json::json!(7);
        borderline["sim_eval"]["seeds_total"] = serde_json::json!(10);
        borderline["sim_eval"]["min_success_rate"] = serde_json::json!(0.7 + 5e-13);
        let err2 =
            ReleaseGate::parse_and_verify(&borderline, &sha, &cfg, &contract, 20.0, &deployment)
                .unwrap_err();
        assert!(matches!(err2, PolicyError::ReleaseGate(_)), "{err2:?}");
    }

    #[tokio::test]
    async fn live_gated_policy_invalidated_on_mid_run_sandbox_scene_change() {
        let sandbox_dir = tempdir().unwrap();
        let sandbox = Arc::new(SandboxState::new(sandbox_dir.path().to_path_buf()));
        let mut cfg = make_6dof_config();
        cfg.profile_id = Some("b601dm".into());
        let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(DummyRealDriver::new(6))));
        sup.set_follower("rebot-dm").await;
        let router = create_router_with_sandbox(sup.clone(), OperatorAuth::disabled(), sandbox.clone());

        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr: SocketAddr = listener.local_addr().unwrap();
        tokio::spawn(async move {
            axum::serve(listener, router).await.unwrap();
        });

        let tick_sup = sup.clone();
        let tick_stop = Arc::new(Mutex::new(false));
        let tick_stop_c = tick_stop.clone();
        let tick_task = tokio::spawn(async move {
            loop {
                if *tick_stop_c.lock().unwrap() {
                    break;
                }
                tick_sup.tick(0.02).await;
                tokio::time::sleep(Duration::from_millis(20)).await;
            }
        });

        let client = reqwest::Client::new();
        upload_rmlp_pack(
            &client,
            addr,
            "mid_run_scene_p",
            vec![0.05, 0.0, 0.0, 0.0, 0.0, 0.0],
            serde_json::json!({}),
        )
        .await;
        let digest = PolicySnapshot::load_from_dir(&sandbox.policy_dir("mid_run_scene_p").unwrap())
            .unwrap()
            .release_digest;

        let (mut ws, _) = connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
        ws.send(Message::Text(
            serde_json::json!({"type": "acquire_lease", "client_id": "mid_run_scene_test"})
                .to_string()
                .into(),
        ))
        .await
        .unwrap();
        let gen = loop {
            let msg: serde_json::Value =
                serde_json::from_str(&ws.next().await.unwrap().unwrap().into_text().unwrap()).unwrap();
            if msg["type"] == "lease_granted" {
                break msg["generation"].as_u64().unwrap();
            }
        };

        ws.send(Message::Text(
            serde_json::json!({
                "type": "command",
                "request_id": 1,
                "generation": gen,
                "epoch_id": 1,
                "path": "/api/v1/torque",
                "body": {"enable": true}
            })
            .to_string()
            .into(),
        ))
        .await
        .unwrap();
        loop {
            let msg: serde_json::Value =
                serde_json::from_str(&ws.next().await.unwrap().unwrap().into_text().unwrap()).unwrap();
            if msg["type"] == "command_ack" {
                break;
            }
        }

        let mut gate_ws = make_valid_gate(&digest, "supervised_first_run", "b601dm", 20, 0.3, "rad");
        gate_ws["scene_id"] = serde_json::json!("default");
        gate_ws["tool_id"] = serde_json::json!("default");
        gate_ws["calibration_digest"] = serde_json::json!(EMPTY_CALIBRATION_DIGEST);

        ws.send(Message::Text(
            serde_json::json!({
                "type": "command",
                "request_id": 2,
                "generation": gen,
                "epoch_id": 1,
                "path": "/api/v1/action/policy/start",
                "body": {
                    "policy_id": "mid_run_scene_p",
                    "mode": "live_gated",
                    "fps": 20.0,
                    "i_am_at_the_arm": true,
                    "release_gate": gate_ws
                }
            })
            .to_string()
            .into(),
        ))
        .await
        .unwrap();

        let ack = tokio::time::timeout(Duration::from_secs(3), async {
            loop {
                let txt = ws.next().await.unwrap().unwrap().into_text().unwrap();
                let v: serde_json::Value = serde_json::from_str(&txt).unwrap();
                if v["type"] == "command_ack" || v["type"] == "command_reject" {
                    return v;
                }
            }
        })
        .await
        .unwrap();
        assert_eq!(ack["type"], "command_ack", "{ack}");

        let mut ran_step = false;
        for _ in 0..100 {
            let obs = sup.get_observation().await;
            if obs.policy.as_ref().is_some_and(|p| p.active && p.steps >= 1) {
                ran_step = true;
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        assert!(ran_step, "expected live_gated policy to complete at least 1 step");

        sandbox.active.write().await.active_scene = "pcb_suction".into();

        let mut terminated = false;
        for _ in 0..150 {
            let obs = sup.get_observation().await;
            if obs.policy.as_ref().is_some_and(|p| !p.active) {
                terminated = true;
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        assert!(terminated, "expected live_gated policy to terminate after scene change");

        {
            let inner = sup.inner().read().await;
            assert!(
                inner.servo_target.is_none(),
                "DEPLOYMENT_CHANGED must clear servo_target"
            );
            assert_eq!(inner.last_action.as_ref().unwrap().state, "failed");
            let pol = inner.policy_observation().unwrap();
            assert_eq!(pol.stop_reason.as_deref(), Some("DEPLOYMENT_CHANGED"));
            assert!(pol.steps >= 1);
        }

        *tick_stop.lock().unwrap() = true;
        let _ = tick_task.await;
    }
}



/// Review item 1: real-hardware policy start needs an operator session, and a production
/// release is authorized only by supervised runs that armd recorded itself.
mod device_qualification_tests {
    use super::phase3_release_gate_tests::make_valid_gate;
    use super::*;
    use armd::policy_runtime::qualification::{load_records, MIN_SUPERVISED_PASSES};
    use tokio_tungstenite::tungstenite::client::IntoClientRequest;

    fn start(epoch: u64, gen: u64, digest: &str, gate: &armd::policy_runtime::VerifiedReleaseGate) -> PolicyRunStart {
        PolicyRunStart {
            epoch_id: epoch,
            connection_id: "conn-q".into(),
            lease_generation: gen,
            policy_id: "p_q".into(),
            artifact_digest: digest.into(),
            backend: "mlp".into(),
            mode: PolicyMode::Live,
            sim_only: false,
            max_first_step_rad: None,
            max_step_rad: None,
            tag: None,
            i_am_at_the_arm: true,
            verified_release_gate: Some(gate.clone()),
        }
    }

    fn code(e: &ArmError) -> String {
        match e {
            ArmError::Rejected { code, .. } => code.clone(),
            other => format!("{other:?}"),
        }
    }

    #[tokio::test]
    async fn production_release_needs_device_recorded_supervised_passes() {
        let dir = tempdir().unwrap();
        let layers = vec![MlpLayerSpec::new(6, 6, Activation::None, vec![0.0; 36], vec![0.0; 6])];
        let digest = write_rmlp_pack(dir.path(), &["j1", "j2", "j3", "j4", "j5", "j6"], "rad", &layers, serde_json::json!({}));
        let mut cfg = make_6dof_config();
        cfg.profile_id = Some("so-arm101".into());
        let prepared = prepare_policy(dir.path(), &cfg).unwrap();
        let deployment = ActiveDeploymentContext {
            scene_id: "pcb_suction".into(),
            tool_id: "suction".into(),
            calibration_digest: EMPTY_CALIBRATION_DIGEST.into(),
        };
        let verify = |tier: &str, speed: f64| {
            let mut raw = make_valid_gate(&digest, tier, "so101", 25, speed, "rad");
            raw["calibration_digest"] = serde_json::json!(EMPTY_CALIBRATION_DIGEST);
            ReleaseGate::parse_and_verify(&raw, &digest, &cfg, prepared.contract(), 25.0, &deployment).unwrap()
        };
        let supervised = verify("supervised_first_run", 0.3);
        // make_valid_gate adds a caller-supplied `supervised_real_run_record` {passed: true}
        let production = verify("production_release", 0.7);

        let sup = Arc::new(ArmSupervisor::new(cfg.clone()).with_driver(Box::new(DummyRealDriver::new(6))));
        sup.set_follower("so-arm101").await;
        let gen = sup.acquire_lease("conn-q").await.unwrap();
        sup.set_torque(true).await.unwrap();
        sup.tick(0.02).await;
        let epoch = sup.get_observation().await.epoch_id;
        let ledger_dir = tempdir().unwrap();
        let ledger = ledger_dir.path().join("policy_qualification.jsonl");

        let mut inner = sup.inner().write().await;
        // no ledger configured: refused
        inner.policy_ledger_path = None;
        let e = inner.begin_policy_run(start(epoch, gen, &digest, &production)).unwrap_err();
        assert_eq!(code(&e), "QUALIFICATION_REQUIRED", "{e:?}");

        // empty ledger: the caller's own run record does not count
        inner.policy_ledger_path = Some(ledger.clone());
        let e = inner.begin_policy_run(start(epoch, gen, &digest, &production)).unwrap_err();
        assert_eq!(code(&e), "QUALIFICATION_REQUIRED", "{e:?}");

        // supervised runs that end at their deadline are recorded as passes
        for i in 0..MIN_SUPERVISED_PASSES {
            let id = inner.begin_policy_run(start(epoch, gen, &digest, &supervised)).unwrap();
            assert!(inner.finish_policy_run(id, "MAX_SECONDS"));
            if i + 1 < MIN_SUPERVISED_PASSES {
                let e = inner.begin_policy_run(start(epoch, gen, &digest, &production)).unwrap_err();
                assert_eq!(code(&e), "QUALIFICATION_REQUIRED", "after {} passes: {e:?}", i + 1);
            }
        }
        let id = inner.begin_policy_run(start(epoch, gen, &digest, &production)).expect("qualified");
        assert!(inner.stop_policy_run(Some(id), "STOPPED"));

        // a supervised run that is stopped is recorded as a failure and revokes the release
        let id = inner.begin_policy_run(start(epoch, gen, &digest, &supervised)).unwrap();
        inner.set_policy_period_us(id, 40_000);
        inner.record_policy_step_timing(id, 1_500, 60_000);
        assert!(inner.stop_policy_run(Some(id), "STOPPED"));
        let e = inner.begin_policy_run(start(epoch, gen, &digest, &production)).unwrap_err();
        assert_eq!(code(&e), "QUALIFICATION_REQUIRED", "{e:?}");

        let records = load_records(&ledger).unwrap();
        assert_eq!(records.len(), MIN_SUPERVISED_PASSES + 1, "production runs are not recorded");
        assert!(records[..MIN_SUPERVISED_PASSES].iter().all(|r| r.passed && r.stop_reason == "MAX_SECONDS"));
        let last = records.last().unwrap();
        assert!(!last.passed);
        assert_eq!(last.stop_reason, "STOPPED");
        assert_eq!(last.policy_sha256, digest.to_ascii_lowercase());
        assert_eq!(last.tier, "supervised_first_run");
        assert!((last.max_speed_scale - 0.3).abs() < 1e-12);
        assert_eq!((last.scene_id.as_str(), last.tool_id.as_str(), last.fps), ("pcb_suction", "suction", 25));
        let stats = last.policy_stats.expect("policy_stats in the ledger record");
        assert_eq!((stats.steps, stats.overruns, stats.skipped, stats.period_us), (1, 1, 1, 40_000));
        assert_eq!(stats.infer_us.max, 1_500);
    }

    async fn send_cmd(ws: &mut tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>>,
                      req_id: u64, gen: u64, epoch: u64, path: &str, body: serde_json::Value) -> serde_json::Value {
        ws.send(Message::Text(serde_json::json!({"type": "command", "request_id": req_id, "generation": gen,
            "epoch_id": epoch, "path": path, "body": body}).to_string().into())).await.unwrap();
        tokio::time::timeout(Duration::from_secs(3), async {
            loop {
                let v: serde_json::Value = serde_json::from_str(&ws.next().await.unwrap().unwrap().into_text().unwrap()).unwrap();
                if v["type"] == "command_ack" || v["type"] == "command_reject" { return v; }
            }
        }).await.unwrap()
    }

    #[tokio::test]
    async fn real_arm_policy_start_needs_operator_session_and_stop_does_not() {
        let sandbox_dir = tempdir().unwrap();
        let sandbox = Arc::new(SandboxState::new(sandbox_dir.path().to_path_buf()));
        let pack = sandbox_dir.path().join("policies").join("auth_pack");
        fs::create_dir_all(&pack).unwrap();
        let layers = vec![MlpLayerSpec::new(6, 6, Activation::None, vec![0.0; 36], vec![0.0; 6])];
        write_rmlp_pack(&pack, &["j1", "j2", "j3", "j4", "j5", "j6"], "rad", &layers, serde_json::json!({}));
        let sup = Arc::new(ArmSupervisor::new(make_6dof_config()).with_driver(Box::new(DummyRealDriver::new(6))));
        sup.set_follower("rebot-dm").await;
        let auth = OperatorAuth::enforced("123456", 1800);
        let router = create_router_with_sandbox(sup.clone(), auth.clone(), sandbox.clone());
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr: SocketAddr = listener.local_addr().unwrap();
        tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });

        async fn ws_with(addr: SocketAddr, token: Option<&str>) -> tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>> {
            let mut req = format!("ws://{addr}/api/v1/ws").into_client_request().unwrap();
            if let Some(t) = token {
                req.headers_mut().insert("authorization", format!("Bearer {t}").parse().unwrap());
            }
            connect_async(req).await.unwrap().0
        }
        async fn lease(ws: &mut tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>>) -> u64 {
            ws.send(Message::Text(serde_json::json!({"type": "acquire_lease", "client_id": "auth_q"}).to_string().into())).await.unwrap();
            loop {
                let v: serde_json::Value = serde_json::from_str(&ws.next().await.unwrap().unwrap().into_text().unwrap()).unwrap();
                if v["type"] == "lease_granted" { return v["generation"].as_u64().unwrap(); }
            }
        }
        let epoch = sup.get_observation().await.epoch_id;
        let shadow = serde_json::json!({"policy_id": "auth_pack", "mode": "shadow", "fps": 10.0});

        // program session (no token): start refused in every mode, stop allowed
        let mut ws = ws_with(addr, None).await;
        let gen = lease(&mut ws).await;
        let r = send_cmd(&mut ws, 1, gen, epoch, "/api/v1/action/policy/start", shadow.clone()).await;
        assert_eq!((r["type"].as_str(), r["code"].as_str()), (Some("command_reject"), Some("OPERATOR_REQUIRED")), "{r}");
        let r = send_cmd(&mut ws, 2, gen, epoch, "/api/v1/action/policy/start",
            serde_json::json!({"policy_id": "auth_pack", "mode": "live", "fps": 25.0, "i_am_at_the_arm": true})).await;
        assert_eq!(r["code"], "OPERATOR_REQUIRED", "{r}");
        let r = send_cmd(&mut ws, 3, gen, epoch, "/api/v1/action/policy/stop", serde_json::json!({})).await;
        assert_eq!(r["type"], "command_ack", "{r}");
        // REST stub classifies the same way
        let rest = reqwest::Client::new().post(format!("http://{addr}/api/v1/action/policy/start")).json(&shadow).send().await.unwrap();
        assert_eq!(rest.status(), 403);
        drop(ws);
        tokio::time::sleep(Duration::from_millis(100)).await;

        // operator session: shadow start accepted
        let unlock: serde_json::Value = reqwest::Client::new().post(format!("http://{addr}/api/v1/auth/unlock"))
            .json(&serde_json::json!({"pin": "123456"})).send().await.unwrap().json().await.unwrap();
        let token = unlock["token"].as_str().unwrap().to_string();
        let mut op = ws_with(addr, Some(&token)).await;
        let gen = lease(&mut op).await;
        let r = send_cmd(&mut op, 1, gen, epoch, "/api/v1/action/policy/start", shadow.clone()).await;
        assert_eq!(r["type"], "command_ack", "{r}");
        let r = send_cmd(&mut op, 2, gen, epoch, "/api/v1/action/policy/stop", serde_json::json!({})).await;
        assert_eq!(r["type"], "command_ack", "{r}");

        // session locked while the pack loads: the recheck after loading refuses the start
        sandbox.set_policy_load_delay_ms(300);
        op.send(Message::Text(serde_json::json!({"type": "command", "request_id": 3, "generation": gen, "epoch_id": epoch,
            "path": "/api/v1/action/policy/start", "body": shadow}).to_string().into())).await.unwrap();
        tokio::time::sleep(Duration::from_millis(80)).await;
        auth.lock(Some(&token));
        let r = tokio::time::timeout(Duration::from_secs(3), async {
            loop {
                let v: serde_json::Value = serde_json::from_str(&op.next().await.unwrap().unwrap().into_text().unwrap()).unwrap();
                if v["type"] == "command_ack" || v["type"] == "command_reject" { return v; }
            }
        }).await.unwrap();
        assert_eq!((r["type"].as_str(), r["code"].as_str()), (Some("command_reject"), Some("OPERATOR_REQUIRED")), "{r}");
        assert!(sup.inner().read().await.policy_run.is_none());
    }
}

/// Review fixes 2-4 and 7: stale targets, stale-vision hold, control-loop discards, gripper cap.
mod review_fix_tests {
    use super::*;
    use armd::policy_runtime::max_policy_input_age_ms;

    /// Returns `out` after sleeping `sleep_ms` on every call.
    pub(super) struct DelayBackend {
        pub sleep_ms: u64,
        pub out: Vec<f32>,
        pub calls: Arc<AtomicUsize>,
    }
    impl PolicyBackend for DelayBackend {
        fn backend_name(&self) -> &'static str { "mock-delay" }
        fn input_dim(&self) -> usize { self.out.len() }
        fn output_dim(&self) -> usize { self.out.len() }
        fn infer(&mut self, _input: &[f32], output: &mut [f32]) -> Result<(), PolicyError> {
            self.calls.fetch_add(1, Ordering::SeqCst);
            std::thread::sleep(Duration::from_millis(self.sleep_ms));
            output.copy_from_slice(&self.out);
            Ok(())
        }
    }

    pub(super) fn prepared_with(cfg: &ArmConfig, backend: Box<dyn PolicyBackend>, extra: serde_json::Value) -> PreparedPolicy {
        let dir = tempdir().unwrap();
        let n = cfg.dof;
        let names: Vec<String> = cfg.limits.iter().map(|l| l.name.clone()).collect();
        let names: Vec<&str> = names.iter().map(String::as_str).collect();
        let layers = vec![MlpLayerSpec::new(n, n, Activation::None, vec![0.0; n * n], vec![0.0; n])];
        write_rmlp_pack(dir.path(), &names, "rad", &layers, extra);
        let snap = PolicySnapshot::load_from_dir(dir.path()).unwrap();
        let contract = StrictPolicyContract::parse(&snap.policy_json_bytes).unwrap();
        PreparedPolicy::from_backend(snap, contract, backend, cfg).unwrap()
    }

    pub(super) async fn begin_live(sup: &Arc<ArmSupervisor>, conn: &str, prepared: &PreparedPolicy) -> (u64, u64) {
        let gen = sup.acquire_lease(conn).await.unwrap();
        sup.set_torque(true).await.unwrap();
        sup.tick(0.02).await;
        let epoch = sup.get_observation().await.epoch_id;
        let mut inner = sup.inner().write().await;
        let id = inner
            .begin_policy_run(PolicyRunStart {
                epoch_id: epoch,
                connection_id: conn.into(),
                lease_generation: gen,
                policy_id: "review".into(),
                artifact_digest: prepared.artifact_digest().to_string(),
                backend: prepared.backend_name().to_string(),
                mode: PolicyMode::Live,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: false,
                verified_release_gate: None,
            })
            .unwrap();
        (id, epoch)
    }

    #[test]
    fn input_age_bound_follows_fps() {
        assert_eq!(max_policy_input_age_ms(10.0), 200);
        assert_eq!(max_policy_input_age_ms(20.0), 100);
        assert_eq!(max_policy_input_age_ms(25.0), 80);
        assert_eq!(max_policy_input_age_ms(50.0), 50);
        assert_eq!(max_policy_input_age_ms(1.0), 200);
        assert_eq!(max_policy_input_age_ms(f64::NAN), 200);
    }

    /// Item 2: inference slower than the bound while feedback, ticks and heartbeats continue:
    /// the late result never becomes the servo target and the run ends STALE_INPUT.
    #[tokio::test]
    async fn slow_inference_target_is_refused_as_stale() {
        let cfg = make_6dof_config();
        let calls = Arc::new(AtomicUsize::new(0));
        let prepared = prepared_with(&cfg, Box::new(DelayBackend { sleep_ms: 250, out: vec![0.1, 0.0, 0.0, 0.0, 0.0, 0.0], calls: calls.clone() }), serde_json::json!({}));
        let sup = Arc::new(ArmSupervisor::new(cfg));
        let (id, epoch) = begin_live(&sup, "conn-stale", &prepared).await;
        let start = sup.get_observation().await.positions.clone();
        let runner = spawn_policy_runner(sup.clone(), id, prepared, 20.0, compute_policy_deadline(Some(2.0)).unwrap(), None, None);
        let mut saw_target = false;
        for _ in 0..30 {
            tokio::time::sleep(Duration::from_millis(20)).await;
            let _ = sup.heartbeat("conn-stale", epoch).await;
            sup.tick(0.02).await;
            saw_target |= sup.inner().read().await.servo_target.is_some();
        }
        let _ = runner.await;
        assert!(calls.load(Ordering::SeqCst) >= 1);
        assert!(!saw_target, "a stale target reached the servo loop");
        let obs = sup.get_observation().await;
        let p = obs.policy.unwrap();
        assert_eq!(p.stop_reason.as_deref(), Some("STALE_INPUT"));
        assert_eq!(p.steps, 0);
        assert_eq!(obs.positions, start, "the arm moved on a stale target");
        assert_eq!(obs.action.as_ref().and_then(|a| a.code.as_deref()), Some("STALE_INPUT"));
    }

    /// Item 3: when the runner holds on stale vision, the live target is dropped at once and
    /// the arm brakes; it does not keep tracking the last target.
    #[tokio::test]
    async fn stale_vision_hold_brakes_immediately() {
        use super::phase2_vision::{make_12_to_6_layers, make_detection, MockVisionSource};
        use armd::grasp::BoxFuture;
        use armd::policy_runtime::PolicyVisionFrame;
        use std::sync::atomic::AtomicBool;
        let dir = tempdir().unwrap();
        write_rmlp_pack(dir.path(), &["j1", "j2", "j3", "j4", "j5", "j6"], "rad", &make_12_to_6_layers(),
            serde_json::json!({"vision": {"source": "detector", "layout": "top1_box_norm_v1", "max_staleness_ms": 150, "on_stale": "hold"}}));
        let mut cfg = make_6dof_config();
        // slow joints: the arm cannot catch up with the target, so it is still tracking one
        // when the frames go stale
        for l in &mut cfg.limits { l.max_velocity = 0.3; }
        let prepared = prepare_policy(dir.path(), &cfg).unwrap();
        let sup = Arc::new(ArmSupervisor::new(cfg));
        let (id, epoch) = begin_live(&sup, "conn-vhold", &prepared).await;
        let stale = Arc::new(AtomicBool::new(false));
        let stale_c = stale.clone();
        let src = MockVisionSource::new_async(move |after, _| {
            let stale = stale_c.load(Ordering::SeqCst);
            let fut: BoxFuture<'static, Result<Option<PolicyVisionFrame>, String>> = Box::pin(async move {
                tokio::time::sleep(Duration::from_millis(10)).await;
                Ok(Some(PolicyVisionFrame {
                    frame_id: after + 1,
                    captured_at: Some(if stale { Instant::now() - Duration::from_secs(1) } else { Instant::now() }),
                    detections: vec![make_detection("cube", 0.9, [0.6, 0.4, 0.9, 0.6])],
                    model_generation: Some(1),
                }))
            });
            fut
        });
        let runner = spawn_policy_runner(sup.clone(), id, prepared, 20.0, compute_policy_deadline(Some(3.0)).unwrap(), Some(src), None);
        // move for a while
        for _ in 0..25 {
            tokio::time::sleep(Duration::from_millis(20)).await;
            let _ = sup.heartbeat("conn-vhold", epoch).await;
            sup.tick(0.02).await;
        }
        let moving = sup.inner().read().await.commanded_velocities[0];
        assert!(moving.abs() > 0.05, "arm should be moving before the hold, v={moving} {:?} {:?}", sup.get_observation().await.policy, sup.get_observation().await.action);
        stale.store(true, Ordering::SeqCst);
        // the runner sees the stale frame within a period or two (well before the 200 ms
        // motion-frame timeout that would otherwise drop the target)
        let t_stale = Instant::now();
        let mut held = false;
        while t_stale.elapsed() < Duration::from_millis(130) {
            tokio::time::sleep(Duration::from_millis(10)).await;
            let _ = sup.heartbeat("conn-vhold", epoch).await;
            if sup.inner().read().await.servo_target.is_none() { held = true; break; }
            sup.tick(0.02).await;
        }
        assert!(held, "stale-vision hold left the servo target in place");
        // from here on, every tick brakes: |v| never grows and reaches zero
        let mut prev = sup.inner().read().await.commanded_velocities[0].abs();
        let mut speeds = vec![prev];
        for _ in 0..30 {
            let _ = sup.heartbeat("conn-vhold", epoch).await;
            sup.tick(0.02).await;
            let inner = sup.inner().read().await;
            assert!(inner.servo_target.is_none(), "a target came back during the hold");
            let v = inner.commanded_velocities[0].abs();
            speeds.push(v);
            assert!(v <= prev + 1e-4, "speed grew during the hold: {speeds:?}");
            prev = v;
            drop(inner);
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        assert!(prev < 1e-3, "arm did not stop during the hold: {speeds:?}");
        assert!(sup.get_observation().await.policy.unwrap().active, "hold keeps the run active");
        sup.inner().write().await.stop_policy_run(Some(id), "STOPPED");
        let _ = runner.await;
    }

    /// Open-loop ramp on j1: `step_rad` more on every call, whatever the input.
    struct RampBackend { k: f32, step_rad: f32 }
    impl PolicyBackend for RampBackend {
        fn backend_name(&self) -> &'static str { "mock-ramp" }
        fn input_dim(&self) -> usize { 6 }
        fn output_dim(&self) -> usize { 6 }
        fn infer(&mut self, _input: &[f32], output: &mut [f32]) -> Result<(), PolicyError> {
            self.k += 1.0;
            output.fill(0.0);
            output[0] = self.k * self.step_rad;
            Ok(())
        }
    }

    /// Item 4: increments that are each valid but outrun a slow joint make the control loop
    /// discard the target; the policy action ends failed with that reason, not MAX_SECONDS.
    #[tokio::test]
    async fn control_loop_discard_ends_the_policy_action() {
        let mut cfg = make_6dof_config();
        for l in &mut cfg.limits { l.max_velocity = 0.3; }
        let prepared = prepared_with(&cfg, Box::new(RampBackend { k: 0.0, step_rad: 0.1 }), serde_json::json!({}));
        let sup = Arc::new(ArmSupervisor::new(cfg));
        let (id, epoch) = begin_live(&sup, "conn-discard", &prepared).await;
        let runner = spawn_policy_runner(sup.clone(), id, prepared, 20.0, compute_policy_deadline(Some(1.5)).unwrap(), None, None);
        for _ in 0..90 {
            tokio::time::sleep(Duration::from_millis(20)).await;
            let _ = sup.heartbeat("conn-discard", epoch).await;
            sup.tick(0.02).await;
            if !sup.inner().read().await.policy_run_active() { break; }
        }
        let _ = runner.await;
        let obs = sup.get_observation().await;
        let p = obs.policy.unwrap();
        assert_eq!(p.stop_reason.as_deref(), Some("SERVO_TARGET_DISCARDED"), "{p:?}");
        let a = obs.action.unwrap();
        assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some("SERVO_TARGET_DISCARDED")));
        assert!(sup.inner().read().await.servo_target.is_none());
    }

    /// Item 4: a runner that stops submitting (slower than the motion-frame timeout) ends the
    /// action with TARGET_TIMEOUT when the control loop drops the target.
    #[tokio::test]
    async fn control_loop_frame_timeout_ends_the_policy_action() {
        let cfg = make_6dof_config();
        let mut inner_cfg = cfg.clone();
        for l in &mut inner_cfg.limits { l.max_velocity = 0.3; }
        let prepared = prepared_with(&inner_cfg, Box::new(DelayBackend { sleep_ms: 0, out: vec![0.2, 0.0, 0.0, 0.0, 0.0, 0.0], calls: Arc::new(AtomicUsize::new(0)) }), serde_json::json!({}));
        let sup = Arc::new(ArmSupervisor::new(inner_cfg));
        let (id, epoch) = begin_live(&sup, "conn-timeout", &prepared).await;
        // one step by hand, then nothing: the target stays until the frame timeout
        sup.inner().write().await.policy_step(id, vec![0.2, 0.0, 0.0, 0.0, 0.0, 0.0], Some(0)).unwrap();
        for _ in 0..20 {
            tokio::time::sleep(Duration::from_millis(20)).await;
            let _ = sup.heartbeat("conn-timeout", epoch).await;
            sup.tick(0.02).await;
        }
        let obs = sup.get_observation().await;
        let a = obs.action.unwrap();
        assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some("TARGET_TIMEOUT")), "{:?}", obs.policy);
        drop(prepared);
    }

    /// Item 5: budgets checked before parsing: model file size and the JSON MLP size.
    #[test]
    fn model_and_json_size_budgets_are_checked_before_parsing() {
        use armd::policy_runtime::{MlpBackend, MAX_MLP_JSON_BYTES, MAX_MODEL_BYTES};
        // 4 MiB + 1 of whitespace is not parsed at all
        let big = vec![b' '; MAX_MLP_JSON_BYTES + 1];
        assert!(matches!(MlpBackend::from_json_bytes(&big), Err(PolicyError::ResourceBudget(_))));
        // a model file over the package limit is refused before it is read
        let dir = tempdir().unwrap();
        let layers = vec![MlpLayerSpec::new(6, 6, Activation::None, vec![0.0; 36], vec![0.0; 6])];
        write_rmlp_pack(dir.path(), &["j1", "j2", "j3", "j4", "j5", "j6"], "rad", &layers, serde_json::json!({}));
        fs::write(dir.path().join("model.rmlp"), vec![0u8; MAX_MODEL_BYTES + 1]).unwrap();
        let err = PolicySnapshot::load_from_dir(dir.path()).err().unwrap();
        assert!(matches!(err, PolicyError::ResourceBudget(_)), "{err:?}");
        let arm: ArmError = err.into();
        assert!(matches!(arm, ArmError::Rejected { status: 413, ref code, .. } if code == "POLICY_TOO_LARGE"));
    }

    /// Item 6: full shape validation of inputs.state / outputs.action.
    #[test]
    fn manifest_shapes_are_validated_on_every_axis() {
        let parse = |state: serde_json::Value, action: serde_json::Value| {
            let pj = serde_json::json!({
                "format": "rebot-onnx-policy/v1", "unit_mode": "rad", "action_dim": 6,
                "action_space": "joint_positions", "joint_names": ["j1", "j2", "j3", "j4", "j5", "j6"],
                "model_file": "model.rmlp",
                "inputs": {"state": {"shape": state}}, "outputs": {"action": {"shape": action}}
            });
            StrictPolicyContract::parse(&serde_json::to_vec(&pj).unwrap())
        };
        let j = |v: serde_json::Value| v;
        assert!(parse(j(serde_json::json!([6])), j(serde_json::json!([1, 6]))).is_ok());
        assert!(parse(j(serde_json::json!([1, 6])), j(serde_json::json!([6]))).is_ok());
        for bad in [
            serde_json::json!([]),            // empty
            serde_json::json!([10, 6]),       // history window
            serde_json::json!([1, 1, 6]),     // extra axis
            serde_json::json!([0, 6]),        // zero axis
            serde_json::json!([-1, 6]),       // negative / dynamic
            serde_json::json!(["batch", 6]),  // symbolic
            serde_json::json!([1, 7]),        // wrong width
            serde_json::json!([1.5, 6]),      // non-integer
        ] {
            assert!(parse(bad.clone(), serde_json::json!([1, 6])).is_err(), "state shape {bad} accepted");
            assert!(parse(serde_json::json!([1, 6]), bad.clone()).is_err(), "action shape {bad} accepted");
        }
    }
}


/// Start speed gate: warm-up inference time against the run's period.
mod speed_gate_tests {
    use super::review_fix_tests::{prepared_with, DelayBackend};
    use super::*;
    use armd::policy_runtime::{SpeedGateReport, WARMUP_INFERENCES, WARMUP_MAX_TOTAL};

    type Ws = tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>>;

    pub(super) async fn ws_cmd(ws: &mut Ws, req_id: u64, gen: u64, path: &str, body: serde_json::Value) -> serde_json::Value {
        ws.send(Message::Text(
            serde_json::json!({"type": "command", "request_id": req_id, "generation": gen, "epoch_id": 1, "path": path, "body": body})
                .to_string()
                .into(),
        ))
        .await
        .unwrap();
        tokio::time::timeout(Duration::from_secs(5), async {
            loop {
                let txt = ws.next().await.unwrap().unwrap().into_text().unwrap();
                let v: serde_json::Value = serde_json::from_str(&txt).unwrap();
                if (v["type"] == "command_ack" || v["type"] == "command_reject") && v["request_id"] == req_id {
                    return v;
                }
            }
        })
        .await
        .unwrap()
    }

    /// Server with a 50 Hz tick, one uploaded pack and a WS holding the lease with torque on.
    pub(super) async fn rig(policy_id: &str) -> (Arc<ArmSupervisor>, Arc<SandboxState>, SocketAddr, Ws, u64, tempfile::TempDir) {
        let sandbox_dir = tempdir().unwrap();
        let sandbox = Arc::new(SandboxState::new(sandbox_dir.path().to_path_buf()));
        let sup = Arc::new(ArmSupervisor::new(make_6dof_config()));
        let router = create_router_with_sandbox(sup.clone(), OperatorAuth::disabled(), sandbox.clone());
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr: SocketAddr = listener.local_addr().unwrap();
        tokio::spawn(async move { axum::serve(listener, router).await.unwrap() });
        let tick_sup = sup.clone();
        tokio::spawn(async move {
            loop {
                tick_sup.tick(0.02).await;
                tokio::time::sleep(Duration::from_millis(20)).await;
            }
        });
        let client = reqwest::Client::new();
        upload_rmlp_pack(&client, addr, policy_id, vec![0.0; 6], serde_json::json!({})).await;
        let (mut ws, _) = connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
        ws.send(Message::Text(serde_json::json!({"type": "acquire_lease", "client_id": "speed"}).to_string().into()))
            .await
            .unwrap();
        let gen = loop {
            let msg: serde_json::Value = serde_json::from_str(&ws.next().await.unwrap().unwrap().into_text().unwrap()).unwrap();
            if msg["type"] == "lease_granted" {
                break msg["generation"].as_u64().unwrap();
            }
        };
        let ack = ws_cmd(&mut ws, 1, gen, "/api/v1/torque", serde_json::json!({"enable": true})).await;
        assert_eq!(ack["type"], "command_ack", "{ack}");
        (sup, sandbox, addr, ws, gen, sandbox_dir)
    }

    #[test]
    fn slow_backend_is_too_slow_for_live_and_allowed_for_shadow() {
        let cfg = make_6dof_config();
        let calls = Arc::new(AtomicUsize::new(0));
        let mut prepared = prepared_with(&cfg, Box::new(DelayBackend { sleep_ms: 30, out: vec![0.0; 6], calls: calls.clone() }), serde_json::json!({}));
        let warmup = prepared.warmup(WARMUP_INFERENCES, WARMUP_MAX_TOTAL).unwrap();
        assert_eq!(calls.load(Ordering::SeqCst), WARMUP_INFERENCES);
        assert_eq!(warmup.n, (WARMUP_INFERENCES - 1) as u64, "first inference discarded");
        assert!(warmup.p50 >= 30_000 && warmup.max >= warmup.p50, "{warmup:?}");

        // 25 fps: period 40 ms, the gate is p50 > 20 ms
        let report = SpeedGateReport::new(warmup, 25.0);
        assert_eq!(report.period_us, 40_000);
        assert!(report.too_slow);
        assert_eq!(report.max_fps, 500_000 / warmup.p50);
        assert!(report.max_fps <= 16, "{report:?}");
        let err = report.check(PolicyMode::Live).unwrap_err();
        let ArmError::Rejected { status, code, message } = err else { panic!("unexpected {err:?}") };
        assert_eq!((status, code.as_str()), (422, "POLICY_TOO_SLOW"));
        for part in [
            format!("p50 {} us", warmup.p50),
            format!("max {} us", warmup.max),
            "40000 us period".to_string(),
            format!("at most {} fps", report.max_fps),
        ] {
            assert!(message.contains(&part), "{message} lacks {part}");
        }
        report.check(PolicyMode::Shadow).unwrap();

        // the same model at 5 fps (period 200 ms) passes
        let slow_rate = SpeedGateReport::new(warmup, 5.0);
        assert!(!slow_rate.too_slow);
        slow_rate.check(PolicyMode::Live).unwrap();
    }

    #[test]
    fn warmup_stops_at_its_time_bound() {
        let cfg = make_6dof_config();
        let calls = Arc::new(AtomicUsize::new(0));
        let mut prepared = prepared_with(&cfg, Box::new(DelayBackend { sleep_ms: 100, out: vec![0.0; 6], calls: calls.clone() }), serde_json::json!({}));
        let warmup = prepared.warmup(WARMUP_INFERENCES, Duration::from_millis(250)).unwrap();
        assert_eq!(calls.load(Ordering::SeqCst), 3);
        assert_eq!(warmup.n, 2);
    }

    #[tokio::test]
    async fn ws_live_start_refused_when_too_slow_and_shadow_starts_flagged() {
        let (_sup, sandbox, _addr, mut ws, gen, _dir) = rig("slow_pol").await;

        // fast model: the ACK carries the warm-up and too_slow false
        let ack = ws_cmd(&mut ws, 2, gen, "/api/v1/action/policy/start", serde_json::json!({"policy_id": "slow_pol", "mode": "live", "fps": 25.0, "max_seconds": 5.0})).await;
        assert_eq!(ack["type"], "command_ack", "{ack}");
        assert_eq!(ack["too_slow"], false, "{ack}");
        assert_eq!(ack["warmup_infer_us"]["n"], 9, "{ack}");
        assert!(ack["warmup_infer_us"]["p50"].as_u64().is_some() && ack["warmup_infer_us"]["max"].as_u64().is_some(), "{ack}");
        assert_eq!(ack["period_us"], 40_000);
        let stop = ws_cmd(&mut ws, 3, gen, "/api/v1/action/policy/stop", serde_json::json!({})).await;
        assert_eq!(stop["type"], "command_ack", "{stop}");

        // every inference now takes >= 30 ms; at 25 fps the gate is 20 ms
        sandbox.set_policy_infer_delay_ms(30);
        for (i, mode) in ["live", "live_sim"].into_iter().enumerate() {
            let rej = ws_cmd(&mut ws, 10 + i as u64, gen, "/api/v1/action/policy/start", serde_json::json!({"policy_id": "slow_pol", "mode": mode, "fps": 25.0, "max_seconds": 5.0})).await;
            assert_eq!(rej["type"], "command_reject", "{mode}: {rej}");
            assert_eq!(rej["code"], "POLICY_TOO_SLOW", "{mode}: {rej}");
            assert_eq!(rej["too_slow"], true, "{rej}");
            let p50 = rej["warmup_infer_us"]["p50"].as_u64().unwrap();
            assert!(p50 >= 30_000, "{rej}");
            let msg = rej["message"].as_str().unwrap();
            assert!(msg.contains(&format!("p50 {p50} us")) && msg.contains("40000 us period") && msg.contains(&format!("at most {} fps", rej["max_fps"])), "{msg}");
        }

        let ack = ws_cmd(&mut ws, 20, gen, "/api/v1/action/policy/start", serde_json::json!({"policy_id": "slow_pol", "mode": "shadow", "fps": 25.0, "max_seconds": 5.0})).await;
        assert_eq!(ack["type"], "command_ack", "{ack}");
        assert_eq!(ack["too_slow"], true, "{ack}");
        assert!(ack["warmup_infer_us"]["p50"].as_u64().unwrap() >= 30_000, "{ack}");
        assert_eq!(ack["warmup_infer_us"]["n"], 9, "{ack}");
        assert!(ack["max_fps"].as_u64().unwrap() <= 16, "{ack}");
        let stop = ws_cmd(&mut ws, 21, gen, "/api/v1/action/policy/stop", serde_json::json!({})).await;
        assert_eq!(stop["stopped"], true, "{stop}");
    }
}

/// `policy_stats` in the action status of a `policy` action.
mod policy_stats_tests {
    use super::review_fix_tests::{prepared_with, DelayBackend};
    use super::*;
    use armd::policy_runtime::{PolicyStepStats, POLICY_STATS_RING};

    /// Sleeps `slow_ms` on every `every`-th call, else `fast_ms`.
    struct SometimesSlow {
        calls: Arc<AtomicUsize>,
        every: usize,
        fast_ms: u64,
        slow_ms: u64,
    }
    impl PolicyBackend for SometimesSlow {
        fn backend_name(&self) -> &'static str { "mock-sometimes-slow" }
        fn input_dim(&self) -> usize { 6 }
        fn output_dim(&self) -> usize { 6 }
        fn infer(&mut self, _input: &[f32], output: &mut [f32]) -> Result<(), PolicyError> {
            let n = self.calls.fetch_add(1, Ordering::SeqCst) + 1;
            std::thread::sleep(Duration::from_millis(if n.is_multiple_of(self.every) { self.slow_ms } else { self.fast_ms }));
            output.fill(0.0);
            Ok(())
        }
    }

    async fn begin_shadow(sup: &Arc<ArmSupervisor>, prepared: &PreparedPolicy) -> u64 {
        let gen = sup.acquire_lease("conn-stats").await.unwrap();
        sup.tick(0.02).await;
        let epoch = sup.get_observation().await.epoch_id;
        let mut inner = sup.inner().write().await;
        inner
            .begin_policy_run(PolicyRunStart {
                epoch_id: epoch,
                connection_id: "conn-stats".into(),
                lease_generation: gen,
                policy_id: "stats".into(),
                artifact_digest: prepared.artifact_digest().to_string(),
                backend: prepared.backend_name().to_string(),
                mode: PolicyMode::Shadow,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: false,
                verified_release_gate: None,
            })
            .unwrap()
    }

    /// Run until the deadline, keeping the lease alive; returns the final action status.
    async fn run_to_end(sup: &Arc<ArmSupervisor>, id: u64, prepared: PreparedPolicy, fps: f64, secs: f64) -> (armd::motion::ActionStatus, Vec<armd::motion::PolicyStats>) {
        let epoch = sup.get_observation().await.epoch_id;
        let runner = spawn_policy_runner(sup.clone(), id, prepared, fps, compute_policy_deadline(Some(secs)).unwrap(), None, None);
        let mut running = Vec::new();
        while !runner.is_finished() {
            tokio::time::sleep(Duration::from_millis(50)).await;
            let _ = sup.heartbeat("conn-stats", epoch).await;
            let obs = sup.get_observation().await;
            if let Some(a) = obs.action.as_ref().filter(|a| a.id == id && a.state == "running") {
                running.extend(a.policy_stats);
            }
        }
        let _ = runner.await;
        let obs = sup.get_observation().await;
        (obs.action.clone().unwrap(), running)
    }

    #[test]
    fn step_stats_ring_and_percentiles() {
        let mut s = PolicyStepStats::default();
        s.set_period_us(10_000);
        assert_eq!(s.report().infer_us, armd::motion::PolicyInferUs::default());
        for i in 1..=100u64 {
            s.record(i * 10, 1_000);
        }
        let r = s.report();
        assert_eq!((r.steps, r.overruns, r.skipped, r.period_us), (100, 0, 0, 10_000));
        assert_eq!((r.infer_us.p50, r.infer_us.p99, r.infer_us.max), (500, 990, 1000));
        // a step of 2.5 periods: one overrun, two periods without a target
        s.record(9_000, 25_000);
        let r = s.report();
        assert_eq!((r.steps, r.overruns, r.skipped), (101, 1, 2));
        assert_eq!(r.infer_us.max, 9_000);
        // the ring keeps only the last POLICY_STATS_RING inferences
        for _ in 0..POLICY_STATS_RING {
            s.record(7, 1_000);
        }
        let r = s.report();
        assert_eq!((r.infer_us.p50, r.infer_us.p99, r.infer_us.max), (7, 7, 7));
        assert_eq!(r.steps, 101 + POLICY_STATS_RING as u64);
    }

    #[tokio::test]
    async fn fast_backend_counts_steps_without_overruns() {
        let cfg = make_6dof_config();
        let calls = Arc::new(AtomicUsize::new(0));
        let prepared = prepared_with(&cfg, Box::new(DelayBackend { sleep_ms: 1, out: vec![0.0; 6], calls: calls.clone() }), serde_json::json!({}));
        let sup = Arc::new(ArmSupervisor::new(cfg));
        let id = begin_shadow(&sup, &prepared).await;
        let (action, running) = run_to_end(&sup, id, prepared, 10.0, 1.0).await;
        assert_eq!(action.kind, "policy");
        assert_eq!(action.state, "done", "{action:?}");
        let stats = action.policy_stats.expect("final policy_stats");
        assert_eq!(stats.steps, calls.load(Ordering::SeqCst) as u64);
        assert!(stats.steps >= 5, "{stats:?}");
        assert_eq!(stats.overruns, 0, "{stats:?}");
        assert_eq!(stats.skipped, 0, "{stats:?}");
        assert_eq!(stats.period_us, 100_000);
        assert!(stats.infer_us.p50 >= 1_000 && stats.infer_us.p99 >= stats.infer_us.p50 && stats.infer_us.max >= stats.infer_us.p99, "{stats:?}");
        assert!(!running.is_empty() && running.iter().any(|s| s.steps > 0), "policy_stats while running: {running:?}");
        // serialized for the SDK
        let v = serde_json::to_value(&action).unwrap();
        assert_eq!(v["policy_stats"]["steps"], stats.steps);
        for k in ["overruns", "skipped", "period_us"] {
            assert!(v["policy_stats"][k].is_u64(), "{v}");
        }
        for k in ["p50", "p99", "max"] {
            assert!(v["policy_stats"]["infer_us"][k].is_u64(), "{v}");
        }
    }

    #[tokio::test]
    async fn sometimes_slow_backend_counts_overruns_and_skipped_periods() {
        let cfg = make_6dof_config();
        let calls = Arc::new(AtomicUsize::new(0));
        // 20 fps (50 ms); every third inference takes 130 ms
        let backend = SometimesSlow { calls: calls.clone(), every: 3, fast_ms: 1, slow_ms: 130 };
        let prepared = prepared_with(&cfg, Box::new(backend), serde_json::json!({}));
        let sup = Arc::new(ArmSupervisor::new(cfg));
        let id = begin_shadow(&sup, &prepared).await;
        let (action, _) = run_to_end(&sup, id, prepared, 20.0, 1.5).await;
        let stats = action.policy_stats.expect("final policy_stats");
        assert_eq!(stats.period_us, 50_000);
        assert!(stats.overruns > 0, "{stats:?}");
        assert!(stats.overruns < stats.steps, "{stats:?}");
        assert!(stats.skipped >= 2 * stats.overruns, "130 ms steps skip two 50 ms periods: {stats:?}");
        assert!(stats.infer_us.max >= 130_000, "{stats:?}");
    }

    #[test]
    fn non_policy_actions_carry_no_policy_stats() {
        let mut inner = armd::SupervisorInner::new(make_6dof_config());
        let id = inner.begin_action("move_to", None);
        let a = inner.last_action.clone().unwrap();
        assert_eq!(a.id, id);
        assert!(a.policy_stats.is_none());
        assert!(serde_json::to_value(&a).unwrap().get("policy_stats").is_none());
    }
}

