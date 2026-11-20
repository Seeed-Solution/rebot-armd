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
    Activation, MlpLayerSpec, PolicyBackend, PolicyError, PolicyMode, PolicyRunStart,
    PolicySnapshot, PreparedPolicy, StrictPolicyContract, MAX_POLICY_FPS, MAX_POLICY_SECONDS,
    MIN_LIVE_POLICY_FPS,
};
use armd::{ArmError, ArmSupervisor, FaultReason, OperatorAuth, SafetyError, SafetyState};

fn make_6dof_config() -> ArmConfig {
    let mut cfg = ArmConfig::default();
    cfg.dof = 6;
    cfg.joint_names = (1..=6).map(|i| format!("j{i}")).collect();
    cfg.limits = (1..=6)
        .map(|i| JointLimit::new(&format!("j{i}"), -2.0, 2.0, 2.0, 10.0))
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
                max_first_step_rad: prepared.contract().max_first_step_rad,
                max_step_rad: prepared.contract().max_step_rad,
                tag: None,
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
                max_first_step_rad: prepared_ok.contract().max_first_step_rad,
                max_step_rad: prepared_ok.contract().max_step_rad,
                tag: None,
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
                max_first_step_rad: prepared_jump.contract().max_first_step_rad,
                max_step_rad: prepared_jump.contract().max_step_rad,
                tag: None,
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
            max_first_step_rad: Some(0.10),
            max_step_rad: Some(0.04),
            tag: None,
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
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
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
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
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
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
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
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
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
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
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
    assert_eq!(pr.modes, vec!["shadow".to_string()], "real driver must only advertise shadow");

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
            max_first_step_rad: None,
            max_step_rad: None,
            tag: None,
        })
        .unwrap_err();
    assert!(
        matches!(live_err, ArmError::Rejected { ref code, .. } if code == "POLICY_SHADOW_ONLY"),
        "expected POLICY_SHADOW_ONLY, got {live_err:?}"
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
            max_first_step_rad: None,
            max_step_rad: None,
            tag: None,
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
            max_first_step_rad: None,
            max_step_rad: None,
            tag: None,
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
            max_first_step_rad: None,
            max_step_rad: None,
            tag: None,
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
            max_first_step_rad: None,
            max_step_rad: None,
            tag: None,
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
            max_first_step_rad: None,
            max_step_rad: None,
            tag: None,
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
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
            })
            .unwrap()
    };
    let runner = spawn_policy_runner(sup.clone(), action_id, prepared, 25.0, None, None);
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
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
            })
            .unwrap()
    };
    let runner2 = spawn_policy_runner(sup2.clone(), action_id2, prepared_fail, 25.0, None, None);
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
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
            })
            .unwrap()
    };
    let expired = Some(Instant::now() - Duration::from_millis(10));
    let runner_a = spawn_policy_runner(sup_a.clone(), id_a, prepared_a, 25.0, expired, None);
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
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
            })
            .unwrap()
    };
    // Deadline is in the future at first check (~0ms), then expires while infer() sleeps 60ms.
    let tight_deadline = Some(Instant::now() + Duration::from_millis(20));
    let runner_b = spawn_policy_runner(sup_b.clone(), id_b, prepared_b, 25.0, tight_deadline, None);
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
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
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
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
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
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
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
            max_first_step_rad: None,
            max_step_rad: None,
            tag: None,
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
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
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
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
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
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
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
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
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
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
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
        pb_varint((field << 3) | 0, out);
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
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
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

    fn make_detection(label: &str, score: f64, box_norm: [f64; 4]) -> Detection {
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

    struct MockVisionSource {
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

        fn new_async<F>(f: F) -> Arc<Self>
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
    fn make_12_to_6_layers() -> Vec<MlpLayerSpec> {
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
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
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
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
                })
                .unwrap()
        };
        let runner2 = spawn_policy_runner(sup2.clone(), action_id2, prepared2, 25.0, None, None);
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
                        max_first_step_rad: None,
                        max_step_rad: None,
                        tag: None,
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
            let runner = spawn_policy_runner(sup.clone(), action_id, prepared, 25.0, None, Some(src));
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
                        max_first_step_rad: None,
                        max_step_rad: None,
                        tag: None,
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
            let runner = spawn_policy_runner(sup.clone(), action_id, prepared, 25.0, None, Some(src));
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
                        max_first_step_rad: None,
                        max_step_rad: None,
                        tag: None,
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
            let runner = spawn_policy_runner(sup.clone(), action_id, prepared, 25.0, None, Some(src));
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
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
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

        let runner = spawn_policy_runner(sup.clone(), action_id, prepared, 30.0, None, Some(src));
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
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
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
        let runner_m = spawn_policy_runner(sup_m.clone(), id_m, prepared_missing, 25.0, None, Some(src_m));
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
                        max_first_step_rad: None,
                        max_step_rad: None,
                        tag: None,
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
                        max_first_step_rad: None,
                        max_step_rad: None,
                        tag: None,
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
                        max_first_step_rad: None,
                        max_step_rad: None,
                        tag: None,
                    })
                    .unwrap()
            };
            let runner = spawn_policy_runner(sup.clone(), id, prepared, 25.0, None, Some(empty_src()));
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
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
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
                        max_first_step_rad: None,
                        max_step_rad: None,
                        tag: None,
                    })
                    .unwrap()
            };

            let runner = spawn_policy_runner(sup.clone(), action_id, prepared, 25.0, None, Some(adapter));
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
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
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

        let runner = spawn_policy_runner(sup.clone(), action_id, prepared, 25.0, None, Some(src));

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
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
                })
                .unwrap()
        };

        let runner = spawn_policy_runner(sup.clone(), action_id, prepared, 12.0, None, Some(adapter));
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
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
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

        let runner = spawn_policy_runner(sup.clone(), action_id, prepared, 25.0, None, Some(src));
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
}

