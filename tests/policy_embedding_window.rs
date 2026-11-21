#![cfg(feature = "policy-runtime")]

use std::fs;
use std::net::SocketAddr;
use std::path::Path;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::time::Duration;

use tempfile::tempdir;
use tokio::net::TcpListener;
use tokio::sync::Notify;

use armd::api::create_router_with_sandbox;
use armd::api::sandbox::SandboxState;
use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
use armd::mjcf_compose::PolicyManifest;
use armd::model::{ArmConfig, JointLimit, JointState};
use armd::policy_runtime::manifest::{
    VideoEmbeddingSpec, VisionLayout, VisionSourceKind, MAX_POLICY_INPUT_WIDTH,
    MAX_VIDEO_WINDOW_SPAN_MS, MAX_VISION_STALENESS_MS,
};
use armd::policy_runtime::runner::{
    compute_policy_deadline, prepare_policy, spawn_policy_runner, InferenceTicket,
    PreparedPolicy, TicketVerificationContext, TicketViolation,
};
use armd::policy_runtime::vision::{
    embedding_hub_for_supervisor, ClockHandshakeRequest, EmbeddingIngestRequest,
    DEFAULT_HANDSHAKE_TTL_MS,
};
use armd::policy_runtime::{
    compute_release_digest, encode_rmlp_bytes, load_backend, Activation, ActiveDeploymentContext,
    MlpLayerSpec, PolicyBackend, PolicyError, PolicyMode, PolicyRunStart, PolicySnapshot,
    ReleaseGate, StrictPolicyContract, EMPTY_CALIBRATION_DIGEST,
};
use armd::{ArmSupervisor, OperatorAuth};

fn sha_a() -> String {
    "a".repeat(64)
}

fn sha_b() -> String {
    "b".repeat(64)
}

fn sha_c() -> String {
    "c".repeat(64)
}

fn make_6dof_config() -> ArmConfig {
    let mut cfg = ArmConfig::default();
    cfg.profile_id = Some("so-arm101".into());
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
    layers: &[MlpLayerSpec],
    extra_manifest: serde_json::Value,
) -> String {
    let rmlp = encode_rmlp_bytes(layers).unwrap();
    let mut manifest = serde_json::json!({
        "format": "rebot-onnx-policy/v1",
        "name": "emb_policy",
        "robot": "so-arm101",
        "unit_mode": "rad",
        "action_dim": 6,
        "action_space": "joint_positions",
        "joint_names": ["j1", "j2", "j3", "j4", "j5", "j6"],
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

/// Real-style driver that counts every `write_joint_targets` call and tracks non-home writes.
struct CountingRealDriver {
    home: Vec<f32>,
    q: Vec<f32>,
    torqued: bool,
    writes: Arc<AtomicUsize>,
    non_home_writes: Arc<AtomicUsize>,
}

impl CountingRealDriver {
    fn new(q: Vec<f32>, writes: Arc<AtomicUsize>, non_home_writes: Arc<AtomicUsize>) -> Self {
        Self {
            home: q.clone(),
            q,
            torqued: false,
            writes,
            non_home_writes,
        }
    }
}

impl ArmDriver for CountingRealDriver {
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
        self.writes.fetch_add(1, Ordering::SeqCst);
        let moved = targets
            .iter()
            .enumerate()
            .any(|(i, t)| (t.position - self.home.get(i).copied().unwrap_or(0.0)).abs() > 1e-6);
        if moved {
            self.non_home_writes.fetch_add(1, Ordering::SeqCst);
        }
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

/// Deterministic barrier backend for mid-inference mutation tests (A12).
struct BarrierBackend {
    inner: Box<dyn PolicyBackend>,
    infer_entered: Arc<Notify>,
    allow_finish: Arc<(Mutex<bool>, Condvar)>,
}

impl PolicyBackend for BarrierBackend {
    fn backend_name(&self) -> &'static str {
        self.inner.backend_name()
    }

    fn input_dim(&self) -> usize {
        self.inner.input_dim()
    }

    fn output_dim(&self) -> usize {
        self.inner.output_dim()
    }

    fn infer(&mut self, input: &[f32], output: &mut [f32]) -> Result<(), PolicyError> {
        self.infer_entered.notify_one();
        let (lock, cvar) = &*self.allow_finish;
        let mut guard = lock.lock().unwrap();
        while !*guard {
            let (g, res) = cvar.wait_timeout(guard, Duration::from_secs(5)).unwrap();
            guard = g;
            if res.timed_out() && !*guard {
                return Err(PolicyError::InferenceFailed(
                    "barrier backend timed out waiting for test release".into(),
                ));
            }
        }
        self.inner.infer(input, output)
    }
}

fn make_valid_release_gate(
    policy_sha256: &str,
    enc_sha: &str,
    prep_sha: &str,
    fps: u64,
) -> serde_json::Value {
    serde_json::json!({
        "schema_version": "rebot-release-gate/v1",
        "authorization_tier": "supervised_first_run",
        "policy_sha256": policy_sha256,
        "scene_id": "pcb_suction",
        "tool_id": "suction",
        "profile_id": "so101",
        "calibration_digest": EMPTY_CALIBRATION_DIGEST,
        "encoder_artifact_sha256": enc_sha,
        "preprocess_config_sha256": prep_sha,
        "runner_config": {
            "fps": fps,
            "max_speed_scale": 0.3,
            "unit_mode": "rad",
            "action_space": "joint_positions"
        },
        "sim_eval": {
            "seeds_total": 10,
            "seeds_passed": 10,
            "min_success_rate": 0.9,
            "collisions": 0,
            "keepout_violations": 0
        },
        "prerequisites": {
            "calibration_verified": true,
            "power_cut_witnessed": true,
            "shadow_run_passed": true
        }
    })
}

async fn spawn_http_server(sup: Arc<ArmSupervisor>, auth: OperatorAuth) -> SocketAddr {
    let sandbox_dir = tempdir().unwrap();
    let sandbox = Arc::new(SandboxState::new(sandbox_dir.keep()));
    let router = create_router_with_sandbox(sup, auth, sandbox);
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        axum::serve(
            listener,
            router.into_make_service_with_connect_info::<SocketAddr>(),
        )
        .await
        .unwrap();
    });
    addr
}

fn make_linear_layers(in_dim: usize, out_dim: usize, weights: Vec<f32>) -> Vec<MlpLayerSpec> {
    vec![MlpLayerSpec::new(
        in_dim,
        out_dim,
        Activation::None,
        weights,
        vec![0.0; out_dim],
    )]
}

#[test]
fn manifest_validates_video_embedding_and_width_budget_and_sandbox_parity() {
    let base = |vision: serde_json::Value| -> Vec<u8> {
        serde_json::to_vec(&serde_json::json!({
            "format": "rebot-onnx-policy/v1",
            "name": "vid_emb",
            "unit_mode": "rad",
            "action_dim": 6,
            "action_space": "joint_positions",
            "joint_names": ["j1", "j2", "j3", "j4", "j5", "j6"],
            "model_file": "model.rmlp",
            "vision": vision
        }))
        .unwrap()
    };

    let ok_t4 = base(serde_json::json!({
        "source": "video_embedding",
        "layout": "embedding_v1",
        "embedding_dim": 8,
        "history_len": 4,
        "max_staleness_ms": 300,
        "max_window_span_ms": 700,
        "fps_staleness_budget_ms": 200,
        "encoder_artifact_sha256": sha_a().to_ascii_uppercase(),
        "preprocess_config_sha256": sha_b(),
        "embedding_norm": {
            "mean": vec![0.5; 8],
            "std": vec![2.0; 8]
        }
    }));
    let c = StrictPolicyContract::parse(&ok_t4).unwrap();
    let emb = c.video_embedding.as_ref().expect("video_embedding parsed");
    assert_eq!(c.vision.as_ref().unwrap().source, VisionSourceKind::VideoEmbedding);
    assert_eq!(c.vision.as_ref().unwrap().layout, VisionLayout::EmbeddingV1);
    assert_eq!(emb.embedding_dim, 8);
    assert_eq!(emb.history_len, 4);
    assert_eq!(emb.effective_staleness_ms(), 200);
    assert_eq!(emb.effective_staleness_ms_for_fps(10.0), 200);
    assert_eq!(emb.encoder_artifact_sha256, sha_a());
    assert_eq!(c.vision_feature_dim(), 32);
    assert_eq!(c.expected_input_dim(), 38);

    // Sandbox PolicyManifest preserves vision field (A14)
    let pm_json = serde_json::json!({
        "id": "vid_emb",
        "name": "vid_emb",
        "format": "rebot-onnx-policy/v1",
        "action_dim": 6,
        "inputs": { "state": { "name": "state", "shape": [1, 6] } },
        "outputs": { "action": { "name": "action", "shape": [1, 6] } },
        "model_file": "model.rmlp",
        "vision": {
            "source": "video_embedding",
            "layout": "embedding_v1",
            "embedding_dim": 8,
            "history_len": 4
        }
    });
    let pm: PolicyManifest = serde_json::from_value(pm_json).unwrap();
    let pm_roundtrip = serde_json::to_value(&pm).unwrap();
    assert_eq!(pm_roundtrip["vision"]["source"], "video_embedding");
    assert_eq!(pm_roundtrip["vision"]["embedding_dim"], 8);

    // Exact boundary: 6 + 1018 * 1 = 1024 <= 1024 -> accepted
    let ok_exact = base(serde_json::json!({
        "source": "video_embedding",
        "layout": "embedding_v1",
        "embedding_dim": MAX_POLICY_INPUT_WIDTH - 6,
        "history_len": 1,
        "max_staleness_ms": 200,
        "max_window_span_ms": 200,
        "encoder_artifact_sha256": sha_a(),
        "preprocess_config_sha256": sha_b()
    }));
    let c_exact = StrictPolicyContract::parse(&ok_exact).unwrap();
    assert_eq!(c_exact.expected_input_dim(), MAX_POLICY_INPUT_WIDTH);

    // Width budget exceeded: 6 + 255 * 4 = 1026 > 1024 -> rejected
    let over_budget = base(serde_json::json!({
        "source": "video_embedding",
        "layout": "embedding_v1",
        "embedding_dim": 255,
        "history_len": 4,
        "max_staleness_ms": 300,
        "max_window_span_ms": 700,
        "encoder_artifact_sha256": sha_a(),
        "preprocess_config_sha256": sha_b()
    }));
    assert!(matches!(
        StrictPolicyContract::parse(&over_budget),
        Err(PolicyError::InvalidManifest(_))
    ));

    // Reject invalid variants
    let bad_cases = vec![
        serde_json::json!({
            "source": "video_embedding", "layout": "embedding_v1",
            "embedding_dim": 4, "history_len": 2,
            "max_staleness_ms": 300, "max_window_span_ms": 250,
            "encoder_artifact_sha256": sha_a(), "preprocess_config_sha256": sha_b()
        }),
        serde_json::json!({
            "source": "video_embedding", "layout": "embedding_v1",
            "embedding_dim": 4, "history_len": 1,
            "max_staleness_ms": MAX_VISION_STALENESS_MS + 1, "max_window_span_ms": 600,
            "encoder_artifact_sha256": sha_a(), "preprocess_config_sha256": sha_b()
        }),
        serde_json::json!({
            "source": "video_embedding", "layout": "embedding_v1",
            "embedding_dim": 4, "history_len": 2,
            "max_staleness_ms": 300, "max_window_span_ms": MAX_VIDEO_WINDOW_SPAN_MS + 1,
            "encoder_artifact_sha256": sha_a(), "preprocess_config_sha256": sha_b()
        }),
        serde_json::json!({
            "source": "video_embedding", "layout": "top1_box_norm_v1",
            "embedding_dim": 4, "history_len": 1,
            "max_staleness_ms": 200, "max_window_span_ms": 200,
            "encoder_artifact_sha256": sha_a(), "preprocess_config_sha256": sha_b()
        }),
        serde_json::json!({
            "source": "video_embedding", "layout": "embedding_v1",
            "embedding_dim": 0, "history_len": 1,
            "max_staleness_ms": 200, "max_window_span_ms": 200,
            "encoder_artifact_sha256": sha_a(), "preprocess_config_sha256": sha_b()
        }),
        serde_json::json!({
            "source": "video_embedding", "layout": "embedding_v1",
            "embedding_dim": 4, "history_len": 1,
            "max_staleness_ms": 200, "max_window_span_ms": 200,
            "encoder_artifact_sha256": "xyz", "preprocess_config_sha256": sha_b()
        }),
        serde_json::json!({
            "source": "video_embedding", "layout": "embedding_v1",
            "embedding_dim": 2, "history_len": 1,
            "max_staleness_ms": 200, "max_window_span_ms": 200,
            "encoder_artifact_sha256": sha_a(), "preprocess_config_sha256": sha_b(),
            "embedding_norm": { "mean": [0.0, 0.0], "std": [1.0, 0.0] }
        }),
    ];
    for bad in bad_cases {
        assert!(
            StrictPolicyContract::parse(&base(bad.clone())).is_err(),
            "expected manifest rejection for {bad}"
        );
    }
}

#[tokio::test]
async fn positive_m1_t4_http_ingestion_concat_norm_and_live_commit() {
    let mut weights = vec![0.0f32; 6 * 14];
    weights[0] = 1.0;
    for idx in 0..8 {
        weights[6 + idx] = 0.001 * ((idx + 1) as f32);
    }
    let layers = make_linear_layers(14, 6, weights);

    let dir = tempdir().unwrap();
    let digest = write_rmlp_pack(
        dir.path(),
        &layers,
        serde_json::json!({
            "fps": 10.0,
            "normalization": {
                "state_mean": [0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
                "state_std": [2.0, 1.0, 1.0, 1.0, 1.0, 1.0]
            },
            "vision": {
                "source": "video_embedding",
                "layout": "embedding_v1",
                "embedding_dim": 2,
                "history_len": 4,
                "max_staleness_ms": 300,
                "max_window_span_ms": 700,
                "encoder_artifact_sha256": sha_a(),
                "preprocess_config_sha256": sha_b(),
                "embedding_norm": {
                    "mean": [1.0, 2.0],
                    "std": [2.0, 4.0]
                }
            }
        }),
    );

    let mut cfg = make_6dof_config();
    cfg.home_positions = vec![0.3, 0.0, 0.0, 0.0, 0.0, 0.0];
    let prepared = prepare_policy(dir.path(), &cfg).unwrap();

    let writes = Arc::new(AtomicUsize::new(0));
    let non_home_writes = Arc::new(AtomicUsize::new(0));
    let drv = CountingRealDriver::new(cfg.home_positions.clone(), writes.clone(), non_home_writes.clone());
    let sup = Arc::new(ArmSupervisor::new(cfg.clone()).with_driver(Box::new(drv)));
    sup.set_follower("so-arm101").await;

    let hub = embedding_hub_for_supervisor(&sup);
    hub.set_test_now_ms(Some(1_000_000));

    let auth = OperatorAuth::enforced("123456", 1800);
    let (token, _) = auth.unlock("123456").unwrap();
    let addr = spawn_http_server(sup.clone(), auth).await;
    let client = reqwest::Client::new();

    // Operator-gated check: missing token -> 403 OPERATOR_REQUIRED
    let unauth_resp = client
        .post(format!("http://{addr}/api/v1/vision/clock-handshake"))
        .json(&serde_json::json!({
            "producer_monotonic_ms": 999_950,
            "producer_uncertainty_ms": 2
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(unauth_resp.status(), reqwest::StatusCode::FORBIDDEN);

    // Valid same-host clock handshake (now=1_000_000, producer_monotonic_ms=999_950 -> offset=+50 ms)
    // Also pass oversized ttl_ms=999_999 to verify server caps TTL <= 60_000 ms (A5).
    let hs_resp = client
        .post(format!("http://{addr}/api/v1/vision/clock-handshake"))
        .header("Authorization", format!("Bearer {token}"))
        .json(&serde_json::json!({
            "producer_epoch": 1,
            "producer_monotonic_ms": 999_950,
            "producer_uncertainty_ms": 3,
            "server_uncertainty_ms": 2,
            "ttl_ms": 999_999
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(hs_resp.status(), reqwest::StatusCode::OK);
    let hs_json: serde_json::Value = hs_resp.json().await.unwrap();
    let hs_gen = hs_json["handshake_generation"].as_u64().unwrap();
    assert_eq!(hs_json["clock_offset_ms"], 50);
    assert_eq!(hs_json["same_host"], true);
    assert_eq!(hs_json["ttl_ms"], DEFAULT_HANDSHAKE_TTL_MS);

    let raw_embs = [[3.0, 6.0], [5.0, 10.0], [7.0, 14.0], [9.0, 18.0]];
    for (k, emb) in raw_embs.iter().enumerate() {
        let now_ms = 1_000_000 + (k as u64) * 100;
        hub.set_test_now_ms(Some(now_ms));
        let cap_ms = (now_ms as i64) - 50; // mapped = now_ms
        let r = client
            .post(format!("http://{addr}/api/v1/vision/embedding"))
            .header("Authorization", format!("Bearer {token}"))
            .json(&serde_json::json!({
                "handshake_generation": hs_gen,
                "producer_epoch": 1,
                "frame_id": (k as u64) + 1,
                "captured_at_ms": cap_ms,
                "embedding": emb,
                "encoder_artifact_sha256": sha_a(),
                "preprocess_config_sha256": sha_b(),
                "model_generation": 9
            }))
            .send()
            .await
            .unwrap();
        assert_eq!(r.status(), reqwest::StatusCode::OK);
    }

    let raw_gate = make_valid_release_gate(&digest, &sha_a(), &sha_b(), 10);
    let gate = ReleaseGate::parse_and_verify(
        &raw_gate,
        &digest,
        &cfg,
        prepared.contract(),
        10.0,
        &ActiveDeploymentContext {
            scene_id: "pcb_suction".into(),
            tool_id: "suction".into(),
            calibration_digest: EMPTY_CALIBRATION_DIGEST.into(),
        },
    )
    .unwrap();

    let lease_gen = sup.acquire_lease("conn-m1").await.unwrap();
    sup.set_torque(true).await.unwrap();
    sup.tick(0.02).await;
    let epoch = sup.get_observation().await.epoch_id;

    let action_id = {
        let mut inner = sup.inner().write().await;
        inner
            .begin_policy_run(PolicyRunStart {
                epoch_id: epoch,
                connection_id: "conn-m1".into(),
                lease_generation: lease_gen,
                policy_id: "vid_m1".into(),
                artifact_digest: digest,
                backend: "mlp".into(),
                mode: PolicyMode::Live,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: true,
                verified_release_gate: Some(gate),
            })
            .unwrap()
    };

    let runner = spawn_policy_runner(
        sup.clone(),
        action_id,
        prepared,
        10.0,
        compute_policy_deadline(Some(0.08)).unwrap(),
        None,
        None,
    );
    for _ in 0..4 {
        tokio::time::sleep(Duration::from_millis(25)).await;
        sup.tick(0.02).await;
    }
    let _ = runner.await;

    let obs = sup.get_observation().await;
    let pol = obs.policy.expect("policy observation");
    assert_eq!(pol.stop_reason.as_deref(), Some("MAX_SECONDS"));
    assert_eq!(pol.steps, 1);
    assert_eq!(pol.vision_frame_id, Some(4));
    assert!(writes.load(Ordering::SeqCst) >= 1);
    assert!(non_home_writes.load(Ordering::SeqCst) >= 1);

    let got = pol.last_shadow_target.unwrap();
    assert!(
        (got[0] - 0.21).abs() < 1e-4,
        "unexpected target[0] = {}",
        got[0]
    );
}

#[tokio::test]
async fn positive_t1_video_model_commits_cleanly() {
    let mut weights = vec![0.0f32; 6 * 10];
    for r in 0..4 {
        weights[r * 10 + (6 + r)] = 0.1;
    }
    let layers = make_linear_layers(10, 6, weights);

    let dir = tempdir().unwrap();
    let digest = write_rmlp_pack(
        dir.path(),
        &layers,
        serde_json::json!({
            "fps": 10.0,
            "vision": {
                "source": "video_embedding",
                "layout": "embedding_v1",
                "embedding_dim": 4,
                "history_len": 1,
                "max_staleness_ms": 300,
                "max_window_span_ms": 300,
                "encoder_artifact_sha256": sha_a(),
                "preprocess_config_sha256": sha_b()
            }
        }),
    );

    let cfg = make_6dof_config();
    let prepared = prepare_policy(dir.path(), &cfg).unwrap();
    let writes = Arc::new(AtomicUsize::new(0));
    let non_home_writes = Arc::new(AtomicUsize::new(0));
    let drv = CountingRealDriver::new(vec![0.0; 6], writes.clone(), non_home_writes.clone());
    let sup = Arc::new(ArmSupervisor::new(cfg.clone()).with_driver(Box::new(drv)));
    sup.set_follower("so-arm101").await;

    let hub = embedding_hub_for_supervisor(&sup);
    hub.set_test_now_ms(Some(500_000));
    let hs = hub
        .clock_handshake(
            ClockHandshakeRequest {
                producer_epoch: 1,
                producer_monotonic_ms: Some(500_000),
                producer_uncertainty_ms: Some(1),
                ..Default::default()
            },
            true,
        )
        .unwrap();
    hub.ingest_embedding(EmbeddingIngestRequest {
        handshake_generation: hs.handshake_generation,
        producer_epoch: 1,
        frame_id: 1,
        captured_at_ms: 500_000,
        embedding: vec![1.0, -0.5, 0.25, 2.0],
        encoder_artifact_sha256: sha_a(),
        preprocess_config_sha256: sha_b(),
        model_generation: Some(3),
    })
    .unwrap();

    let raw_gate = make_valid_release_gate(&digest, &sha_a(), &sha_b(), 10);
    let gate = ReleaseGate::parse_and_verify(
        &raw_gate,
        &digest,
        &cfg,
        prepared.contract(),
        10.0,
        &ActiveDeploymentContext {
            scene_id: "pcb_suction".into(),
            tool_id: "suction".into(),
            calibration_digest: EMPTY_CALIBRATION_DIGEST.into(),
        },
    )
    .unwrap();

    let lease_gen = sup.acquire_lease("conn-t1").await.unwrap();
    sup.set_torque(true).await.unwrap();
    sup.tick(0.02).await;
    let epoch = sup.get_observation().await.epoch_id;

    let action_id = {
        let mut inner = sup.inner().write().await;
        inner
            .begin_policy_run(PolicyRunStart {
                epoch_id: epoch,
                connection_id: "conn-t1".into(),
                lease_generation: lease_gen,
                policy_id: "vid_t1".into(),
                artifact_digest: digest,
                backend: "mlp".into(),
                mode: PolicyMode::Live,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: true,
                verified_release_gate: Some(gate),
            })
            .unwrap()
    };

    let runner = spawn_policy_runner(
        sup.clone(),
        action_id,
        prepared,
        10.0,
        compute_policy_deadline(Some(0.08)).unwrap(),
        None,
        None,
    );
    for _ in 0..4 {
        tokio::time::sleep(Duration::from_millis(25)).await;
        sup.tick(0.02).await;
    }
    let _ = runner.await;

    let obs = sup.get_observation().await;
    let pol = obs.policy.expect("policy observation");
    assert_eq!(pol.stop_reason.as_deref(), Some("MAX_SECONDS"));
    assert!(pol.steps >= 1);
    assert!(non_home_writes.load(Ordering::SeqCst) >= 1);
    let got = pol.last_shadow_target.unwrap();
    assert!((got[0] - 0.1).abs() < 1e-5);
    assert!((got[1] - (-0.05)).abs() < 1e-5);
    assert!((got[2] - 0.025).abs() < 1e-5);
    assert!((got[3] - 0.2).abs() < 1e-5);
}

#[tokio::test]
async fn handshake_and_ingestion_security_invariants_a2_a4_a5_a7_a8_a9_a10_a13() {
    let sup = Arc::new(ArmSupervisor::new(make_6dof_config()));
    let hub = embedding_hub_for_supervisor(&sup);
    hub.set_test_now_ms(Some(10_000));
    let addr = spawn_http_server(sup.clone(), OperatorAuth::disabled()).await;
    let client = reqwest::Client::new();

    // A4: missing producer_monotonic_ms rejected 400
    let missing_ts = client
        .post(format!("http://{addr}/api/v1/vision/clock-handshake"))
        .json(&serde_json::json!({"producer_uncertainty_ms": 1}))
        .send()
        .await
        .unwrap();
    assert_eq!(missing_ts.status(), reqwest::StatusCode::BAD_REQUEST);
    assert_eq!(
        missing_ts.json::<serde_json::Value>().await.unwrap()["code"],
        "MISSING_PRODUCER_TIMESTAMP"
    );

    // A4: missing producer_uncertainty_ms rejected 400
    let missing_unc = client
        .post(format!("http://{addr}/api/v1/vision/clock-handshake"))
        .json(&serde_json::json!({"producer_monotonic_ms": 10_000}))
        .send()
        .await
        .unwrap();
    assert_eq!(missing_unc.status(), reqwest::StatusCode::BAD_REQUEST);
    assert_eq!(
        missing_unc.json::<serde_json::Value>().await.unwrap()["code"],
        "MISSING_UNCERTAINTY"
    );

    // C1: Loopback connection (127.0.0.1) enforces <= 5 ms even if caller passes same_host=false
    let bad_local = client
        .post(format!("http://{addr}/api/v1/vision/clock-handshake"))
        .json(&serde_json::json!({
            "producer_monotonic_ms": 10_000,
            "producer_uncertainty_ms": 6,
            "same_host": false
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(bad_local.status(), reqwest::StatusCode::BAD_REQUEST);

    // Remote IP via X-Forwarded-For enforces <= 25 ms (20 ms allowed, 26 ms rejected)
    let ok_remote = client
        .post(format!("http://{addr}/api/v1/vision/clock-handshake"))
        .header("X-Forwarded-For", "192.168.1.50")
        .json(&serde_json::json!({
            "producer_epoch": 1,
            "client_monotonic_ms": 9_900,
            "producer_uncertainty_ms": 10,
            "server_uncertainty_ms": 4
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(ok_remote.status(), reqwest::StatusCode::OK);
    let hs_json: serde_json::Value = ok_remote.json().await.unwrap();
    assert_eq!(hs_json["same_host"], false);
    assert_eq!(hs_json["clock_offset_ms"], 100);
    let hs_gen = hs_json["handshake_generation"].as_u64().unwrap();

    let bad_remote = client
        .post(format!("http://{addr}/api/v1/vision/clock-handshake"))
        .header("X-Forwarded-For", "192.168.1.50")
        .json(&serde_json::json!({
            "producer_epoch": 1,
            "producer_monotonic_ms": 9_900,
            "producer_uncertainty_ms": 26
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(bad_remote.status(), reqwest::StatusCode::BAD_REQUEST);

    // A13: Pre-bind width budget rejects > 1024 embedding length
    let over_prebind = client
        .post(format!("http://{addr}/api/v1/vision/embedding"))
        .json(&serde_json::json!({
            "handshake_generation": hs_gen,
            "producer_epoch": 1,
            "frame_id": 1,
            "captured_at_ms": 9_900,
            "embedding": vec![0.1; MAX_POLICY_INPUT_WIDTH + 1],
            "encoder_artifact_sha256": sha_a(),
            "preprocess_config_sha256": sha_b()
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(over_prebind.status(), reqwest::StatusCode::BAD_REQUEST);
    assert_eq!(
        over_prebind.json::<serde_json::Value>().await.unwrap()["code"],
        "WIDTH_BUDGET_EXCEEDED"
    );

    // A10: Bind policy at 20 Hz without fps_staleness_budget_ms -> period-derived budget = 100 ms
    let spec = VideoEmbeddingSpec {
        embedding_dim: 2,
        history_len: 3,
        max_staleness_ms: 300,
        max_window_span_ms: 500,
        fps_staleness_budget_ms: None,
        encoder_artifact_sha256: sha_a(),
        preprocess_config_sha256: sha_b(),
        embedding_norm: None,
        on_stale: armd::policy_runtime::VisionStalePolicy::Stop,
    };
    hub.bind_policy(1, spec.clone(), 20.0);

    // Frame 1 at mapped = 10_000 (captured_at = 9_900)
    let r1 = client
        .post(format!("http://{addr}/api/v1/vision/embedding"))
        .json(&serde_json::json!({
            "handshake_generation": hs_gen,
            "producer_epoch": 1,
            "frame_id": 10,
            "captured_at_ms": 9_900,
            "embedding": [1.0, 2.0],
            "encoder_artifact_sha256": sha_a(),
            "preprocess_config_sha256": sha_b()
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(r1.status(), reqwest::StatusCode::OK);
    let dead_before = hub.window_deadlines().unwrap();

    // Future-ahead frame: mapped = 9_911 + 100 = 10_011 > 10_000 + min(10, 25) -> 400 FUTURE_FRAME
    let r_fut = client
        .post(format!("http://{addr}/api/v1/vision/embedding"))
        .json(&serde_json::json!({
            "handshake_generation": hs_gen,
            "producer_epoch": 1,
            "frame_id": 11,
            "captured_at_ms": 9_911,
            "embedding": [3.0, 4.0],
            "encoder_artifact_sha256": sha_a(),
            "preprocess_config_sha256": sha_b()
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(r_fut.status(), reqwest::StatusCode::BAD_REQUEST);
    assert_eq!(r_fut.json::<serde_json::Value>().await.unwrap()["code"], "FUTURE_FRAME");
    assert_eq!(hub.frame_ids(), vec![10]);
    assert_eq!(hub.window_deadlines().unwrap(), dead_before);

    // A10: Stale via FPS-derived budget (100 ms at 20 Hz, even though max_staleness_ms=300)
    // age = 10_000 - (9_750 + 100) = 150 > 100 -> STALE_FRAME
    let r_stale = client
        .post(format!("http://{addr}/api/v1/vision/embedding"))
        .json(&serde_json::json!({
            "handshake_generation": hs_gen,
            "producer_epoch": 1,
            "frame_id": 11,
            "captured_at_ms": 9_750,
            "embedding": [3.0, 4.0],
            "encoder_artifact_sha256": sha_a(),
            "preprocess_config_sha256": sha_b()
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(r_stale.status(), reqwest::StatusCode::BAD_REQUEST);
    assert_eq!(r_stale.json::<serde_json::Value>().await.unwrap()["code"], "STALE_FRAME");

    // A2: Out-of-order mapped timestamp (< 10_000) rejected with NON_MONOTONIC_TIMESTAMP
    let r_ooo = client
        .post(format!("http://{addr}/api/v1/vision/embedding"))
        .json(&serde_json::json!({
            "handshake_generation": hs_gen,
            "producer_epoch": 1,
            "frame_id": 11,
            "captured_at_ms": 9_850, // mapped = 9_950 < 10_000
            "embedding": [3.0, 4.0],
            "encoder_artifact_sha256": sha_a(),
            "preprocess_config_sha256": sha_b()
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(r_ooo.status(), reqwest::StatusCode::BAD_REQUEST);
    assert_eq!(r_ooo.json::<serde_json::Value>().await.unwrap()["code"], "NON_MONOTONIC_TIMESTAMP");

    // A8: Mismatched producer_epoch rejected with 409 EPOCH_MISMATCH
    let r_ep = client
        .post(format!("http://{addr}/api/v1/vision/embedding"))
        .json(&serde_json::json!({
            "handshake_generation": hs_gen,
            "producer_epoch": 2,
            "frame_id": 11,
            "captured_at_ms": 9_900,
            "embedding": [3.0, 4.0],
            "encoder_artifact_sha256": sha_a(),
            "preprocess_config_sha256": sha_b()
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(r_ep.status(), reqwest::StatusCode::CONFLICT);
    assert_eq!(r_ep.json::<serde_json::Value>().await.unwrap()["code"], "EPOCH_MISMATCH");

    // A9: Policy rebind (even with different dim) in same producer_epoch preserves last_frame_id=10
    let mut spec2 = spec.clone();
    spec2.embedding_dim = 3;
    hub.bind_policy(2, spec2, 10.0);
    assert_eq!(hub.window_len(), 0); // incompatible dim cleared frames
    let r_replay = client
        .post(format!("http://{addr}/api/v1/vision/embedding"))
        .json(&serde_json::json!({
            "handshake_generation": hs_gen,
            "producer_epoch": 1,
            "frame_id": 10,
            "captured_at_ms": 9_900,
            "embedding": [1.0, 2.0, 3.0],
            "encoder_artifact_sha256": sha_a(),
            "preprocess_config_sha256": sha_b()
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(r_replay.status(), reqwest::StatusCode::CONFLICT);
    assert_eq!(
        r_replay.json::<serde_json::Value>().await.unwrap()["code"],
        "NON_MONOTONIC_FRAME_ID"
    );

    // Push frame 11, then verify A7: handshake renewal clears old frames
    hub.bind_policy(3, spec, 10.0);
    let r11 = client
        .post(format!("http://{addr}/api/v1/vision/embedding"))
        .json(&serde_json::json!({
            "handshake_generation": hs_gen,
            "producer_epoch": 1,
            "frame_id": 11,
            "captured_at_ms": 9_900,
            "embedding": [1.0, 2.0],
            "encoder_artifact_sha256": sha_a(),
            "preprocess_config_sha256": sha_b()
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(r11.status(), reqwest::StatusCode::OK);
    assert_eq!(hub.window_len(), 1);

    let renew = client
        .post(format!("http://{addr}/api/v1/vision/clock-handshake"))
        .json(&serde_json::json!({
            "producer_epoch": 1,
            "producer_monotonic_ms": 9_800,
            "producer_uncertainty_ms": 2
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(renew.status(), reqwest::StatusCode::OK);
    let renew_gen = renew.json::<serde_json::Value>().await.unwrap()["handshake_generation"]
        .as_u64()
        .unwrap();
    assert_eq!(hub.window_len(), 0);

    // A-R2-4: Model generation change clears prior-generation frames so windows never mix gens.
    for (fid, mgen) in [(12u64, 1u64), (13u64, 1u64)] {
        let r = client
            .post(format!("http://{addr}/api/v1/vision/embedding"))
            .json(&serde_json::json!({
                "handshake_generation": renew_gen,
                "producer_epoch": 1,
                "frame_id": fid,
                "captured_at_ms": 9_800,
                "embedding": [1.0, 2.0],
                "encoder_artifact_sha256": sha_a(),
                "preprocess_config_sha256": sha_b(),
                "model_generation": mgen
            }))
            .send()
            .await
            .unwrap();
        assert_eq!(r.status(), reqwest::StatusCode::OK);
    }
    assert_eq!(hub.frame_ids(), vec![12, 13]);

    let r_newgen = client
        .post(format!("http://{addr}/api/v1/vision/embedding"))
        .json(&serde_json::json!({
            "handshake_generation": renew_gen,
            "producer_epoch": 1,
            "frame_id": 14,
            "captured_at_ms": 9_800,
            "embedding": [3.0, 4.0],
            "encoder_artifact_sha256": sha_a(),
            "preprocess_config_sha256": sha_b(),
            "model_generation": 2
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(r_newgen.status(), reqwest::StatusCode::OK);
    assert_eq!(hub.frame_ids(), vec![14]);
}

#[derive(Clone, Copy)]
enum MidInferenceMutation {
    HandshakeExpired,
    LeaseChanged,
    ModelGenerationChanged,
    ProducerEpochChanged,
    InvalidationBumped,
    WindowNewestExpired,
    WindowOldestExpired,
    PolicyRebound,
}

async fn run_negative_ticket_case(mutation: MidInferenceMutation) -> (String, usize, usize, u64) {
    let mut weights = vec![0.0f32; 6 * 10];
    for r in 0..4 {
        weights[r * 10 + (6 + r)] = 0.05;
    }
    let layers = make_linear_layers(10, 6, weights);

    let dir = tempdir().unwrap();
    let digest = write_rmlp_pack(
        dir.path(),
        &layers,
        serde_json::json!({
            "fps": 10.0,
            "vision": {
                "source": "video_embedding",
                "layout": "embedding_v1",
                "embedding_dim": 2,
                "history_len": 2,
                "max_staleness_ms": 300,
                "max_window_span_ms": 400,
                "fps_staleness_budget_ms": 300,
                "encoder_artifact_sha256": sha_a(),
                "preprocess_config_sha256": sha_b()
            }
        }),
    );

    let cfg = make_6dof_config();
    let snapshot = PolicySnapshot::load_from_dir(dir.path()).unwrap();
    let contract = StrictPolicyContract::parse(&snapshot.policy_json_bytes).unwrap();
    let inner_backend = load_backend(&snapshot, &contract).unwrap();

    let infer_entered = Arc::new(Notify::new());
    let allow_finish = Arc::new((Mutex::new(false), Condvar::new()));
    let barrier_backend = Box::new(BarrierBackend {
        inner: inner_backend,
        infer_entered: infer_entered.clone(),
        allow_finish: allow_finish.clone(),
    });
    let prepared = PreparedPolicy::from_backend(snapshot, contract, barrier_backend, &cfg).unwrap();

    let writes = Arc::new(AtomicUsize::new(0));
    let non_home_writes = Arc::new(AtomicUsize::new(0));
    let drv = CountingRealDriver::new(vec![0.0; 6], writes.clone(), non_home_writes.clone());
    let sup = Arc::new(ArmSupervisor::new(cfg.clone()).with_driver(Box::new(drv)));
    sup.set_follower("so-arm101").await;

    let hub = embedding_hub_for_supervisor(&sup);
    let ttl_ms = match mutation {
        MidInferenceMutation::HandshakeExpired => 350,
        _ => 60_000,
    };
    hub.set_default_handshake_ttl_ms(ttl_ms);

    hub.set_test_now_ms(Some(1_000_000));
    let hs = hub
        .clock_handshake(
            ClockHandshakeRequest {
                producer_epoch: 1,
                producer_monotonic_ms: Some(1_000_000),
                producer_uncertainty_ms: Some(1),
                ..Default::default()
            },
            true,
        )
        .unwrap();

    hub.ingest_embedding(EmbeddingIngestRequest {
        handshake_generation: hs.handshake_generation,
        producer_epoch: 1,
        frame_id: 1,
        captured_at_ms: 1_000_000,
        embedding: vec![1.0, 1.0],
        encoder_artifact_sha256: sha_a(),
        preprocess_config_sha256: sha_b(),
        model_generation: Some(5),
    })
    .unwrap();

    let f2_ts = match mutation {
        MidInferenceMutation::WindowNewestExpired => 1_000_050,
        _ => 1_000_250,
    };
    hub.set_test_now_ms(Some(f2_ts));
    hub.ingest_embedding(EmbeddingIngestRequest {
        handshake_generation: hs.handshake_generation,
        producer_epoch: 1,
        frame_id: 2,
        captured_at_ms: f2_ts as i64,
        embedding: vec![1.0, 1.0],
        encoder_artifact_sha256: sha_a(),
        preprocess_config_sha256: sha_b(),
        model_generation: Some(5),
    })
    .unwrap();

    let raw_gate = make_valid_release_gate(&digest, &sha_a(), &sha_b(), 10);
    let gate = ReleaseGate::parse_and_verify(
        &raw_gate,
        &digest,
        &cfg,
        prepared.contract(),
        10.0,
        &ActiveDeploymentContext {
            scene_id: "pcb_suction".into(),
            tool_id: "suction".into(),
            calibration_digest: EMPTY_CALIBRATION_DIGEST.into(),
        },
    )
    .unwrap();

    let lease_gen = sup.acquire_lease("conn-neg").await.unwrap();
    sup.set_torque(true).await.unwrap();
    sup.tick(0.02).await;
    let epoch = sup.get_observation().await.epoch_id;

    let action_id = {
        let mut inner = sup.inner().write().await;
        inner
            .begin_policy_run(PolicyRunStart {
                epoch_id: epoch,
                connection_id: "conn-neg".into(),
                lease_generation: lease_gen,
                policy_id: "vid_neg".into(),
                artifact_digest: digest,
                backend: "mlp".into(),
                mode: PolicyMode::Live,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: true,
                verified_release_gate: Some(gate),
            })
            .unwrap()
    };

    // Reset total driver write counter right before spawning the policy runner (A12).
    let baseline_motion_frame = sup.inner().read().await.last_motion_frame;
    writes.store(0, Ordering::SeqCst);
    non_home_writes.store(0, Ordering::SeqCst);

    let runner = spawn_policy_runner(
        sup.clone(),
        action_id,
        prepared,
        10.0,
        compute_policy_deadline(Some(2.0)).unwrap(),
        None,
        None,
    );

    // Deterministic barrier with timeout: wait until inference has started inside `BarrierBackend::infer`.
    tokio::time::timeout(Duration::from_secs(5), infer_entered.notified())
        .await
        .expect("timed out waiting for BarrierBackend::infer to start");

    match mutation {
        MidInferenceMutation::HandshakeExpired => {
            hub.set_test_now_ms(Some(1_000_360));
        }
        MidInferenceMutation::LeaseChanged => {
            let _ = sup.preempt_lease("conn-hijack", "hijack").await.unwrap();
        }
        MidInferenceMutation::ModelGenerationChanged => {
            hub.set_model_generation(Some(999));
        }
        MidInferenceMutation::ProducerEpochChanged => {
            hub.set_producer_epoch(2);
        }
        MidInferenceMutation::InvalidationBumped => {
            hub.bump_invalidation_generation();
        }
        MidInferenceMutation::WindowNewestExpired => {
            hub.set_test_now_ms(Some(1_000_360));
        }
        MidInferenceMutation::WindowOldestExpired => {
            hub.set_test_now_ms(Some(1_000_420));
        }
        MidInferenceMutation::PolicyRebound => {
            hub.bind_policy(
                action_id + 1,
                VideoEmbeddingSpec {
                    embedding_dim: 2,
                    history_len: 2,
                    max_staleness_ms: 300,
                    max_window_span_ms: 400,
                    fps_staleness_budget_ms: Some(300),
                    encoder_artifact_sha256: sha_a(),
                    preprocess_config_sha256: sha_b(),
                    embedding_norm: None,
                    on_stale: armd::policy_runtime::VisionStalePolicy::Stop,
                },
                10.0,
            );
        }
    }

    // Release inference and wait (with timeout) for runner to exit at the commit critical section.
    {
        let (lock, cvar) = &*allow_finish;
        *lock.lock().unwrap() = true;
        cvar.notify_all();
    }
    tokio::time::timeout(Duration::from_secs(5), runner)
        .await
        .expect("timed out waiting for policy runner to exit")
        .expect("policy runner task panicked");

    let total_writes_before_tick = writes.load(Ordering::SeqCst);
    sup.tick(0.02).await;
    let total_writes_after_tick = writes.load(Ordering::SeqCst);
    // `sup.tick` emits at most 1 idle hold write when torqued, never a policy motion frame.
    assert!(
        total_writes_after_tick <= total_writes_before_tick + 1,
        "unexpected extra driver writes across tick: before={total_writes_before_tick}, after={total_writes_after_tick}"
    );
    assert_eq!(
        sup.inner().read().await.last_motion_frame,
        baseline_motion_frame,
        "last_motion_frame must not advance when ticket commit is rejected"
    );

    let obs = sup.get_observation().await;
    let pol = obs.policy.expect("policy observation");
    assert!(
        sup.inner().read().await.servo_target.is_none(),
        "servo_target must remain None on aborted commit"
    );
    assert!(
        pol.last_shadow_target.is_none(),
        "last_shadow_target must remain None on aborted commit"
    );
    (
        pol.stop_reason.unwrap_or_default(),
        total_writes_before_tick,
        non_home_writes.load(Ordering::SeqCst),
        pol.steps,
    )
}

#[tokio::test]
async fn negative_inference_ticket_commit_tests_emit_zero_targets() {
    for (mutation, expected_reason) in [
        (MidInferenceMutation::HandshakeExpired, "HANDSHAKE_EXPIRED"),
        (MidInferenceMutation::LeaseChanged, "LEASE_LOST"),
        (MidInferenceMutation::ModelGenerationChanged, "VISION_GENERATION_CHANGED"),
        (MidInferenceMutation::ProducerEpochChanged, "PRODUCER_EPOCH_CHANGED"),
        (MidInferenceMutation::InvalidationBumped, "WINDOW_INVALIDATED"),
        (MidInferenceMutation::WindowNewestExpired, "VISION_STALE"),
        (MidInferenceMutation::WindowOldestExpired, "VISION_STALE"),
        (MidInferenceMutation::PolicyRebound, "POLICY_INSTANCE_CHANGED"),
    ] {
        let (reason, total_writes, non_home_writes, steps) =
            run_negative_ticket_case(mutation).await;
        assert_eq!(reason, expected_reason);
        assert_eq!(total_writes, 0, "expected 0 total driver writes for {expected_reason}");
        assert_eq!(non_home_writes, 0);
        assert_eq!(steps, 0);
    }
}

#[test]
fn inference_ticket_verify_checks_all_nine_fields() {
    let ticket = InferenceTicket::new(10, 2, 3, Some(4), 5, 6, 1000, 1100, 1200);
    let base_ctx = TicketVerificationContext {
        policy_instance_id: Some(10),
        lease_generation: Some(2),
        producer_epoch: Some(3),
        model_generation: Some(4),
        invalidation_generation: 5,
        clock_handshake_generation: 6,
        now_armd_ms: 1000,
    };
    assert!(ticket.verify(&base_ctx).is_ok());

    let mut c = base_ctx.clone();
    c.policy_instance_id = Some(11);
    assert!(matches!(ticket.verify(&c), Err(TicketViolation::PolicyInstanceMismatch { .. })));

    let mut c = base_ctx.clone();
    c.clock_handshake_generation = 7;
    assert!(matches!(ticket.verify(&c), Err(TicketViolation::ClockHandshakeGenerationMismatch { .. })));
}

#[test]
fn release_gate_rejects_missing_or_mismatched_encoder_and_preprocess_sha256() {
    let dir = tempdir().unwrap();
    let layers = make_linear_layers(8, 6, vec![0.0; 48]);
    let digest = write_rmlp_pack(
        dir.path(),
        &layers,
        serde_json::json!({
            "fps": 10.0,
            "vision": {
                "source": "video_embedding",
                "layout": "embedding_v1",
                "embedding_dim": 2,
                "history_len": 1,
                "max_staleness_ms": 200,
                "max_window_span_ms": 200,
                "encoder_artifact_sha256": sha_a(),
                "preprocess_config_sha256": sha_b()
            }
        }),
    );

    let cfg = make_6dof_config();
    let prepared = prepare_policy(dir.path(), &cfg).unwrap();
    let deployment = ActiveDeploymentContext {
        scene_id: "pcb_suction".into(),
        tool_id: "suction".into(),
        calibration_digest: EMPTY_CALIBRATION_DIGEST.into(),
    };

    let ok_gate = make_valid_release_gate(&digest, &sha_a(), &sha_b(), 10);
    let verified = ReleaseGate::parse_and_verify(
        &ok_gate,
        &digest,
        &cfg,
        prepared.contract(),
        10.0,
        &deployment,
    )
    .unwrap();
    assert_eq!(verified.encoder_artifact_sha256(), Some(sha_a().as_str()));
    assert_eq!(verified.preprocess_config_sha256(), Some(sha_b().as_str()));

    let mut missing_enc = ok_gate.clone();
    missing_enc.as_object_mut().unwrap().remove("encoder_artifact_sha256");
    assert!(matches!(
        ReleaseGate::parse_and_verify(&missing_enc, &digest, &cfg, prepared.contract(), 10.0, &deployment),
        Err(PolicyError::ReleaseGate(_))
    ));

    let mut missing_prep = ok_gate.clone();
    missing_prep.as_object_mut().unwrap().remove("preprocess_config_sha256");
    assert!(matches!(
        ReleaseGate::parse_and_verify(&missing_prep, &digest, &cfg, prepared.contract(), 10.0, &deployment),
        Err(PolicyError::ReleaseGate(_))
    ));

    let bad_enc = make_valid_release_gate(&digest, &sha_c(), &sha_b(), 10);
    assert!(matches!(
        ReleaseGate::parse_and_verify(&bad_enc, &digest, &cfg, prepared.contract(), 10.0, &deployment),
        Err(PolicyError::ReleaseGate(_))
    ));

    let bad_prep = make_valid_release_gate(&digest, &sha_a(), &sha_c(), 10);
    assert!(matches!(
        ReleaseGate::parse_and_verify(&bad_prep, &digest, &cfg, prepared.contract(), 10.0, &deployment),
        Err(PolicyError::ReleaseGate(_))
    ));
}
