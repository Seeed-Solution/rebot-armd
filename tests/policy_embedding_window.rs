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
    embedding_hub_for_supervisor, ClockHandshakeRequest, ClockHandshakeResponse,
    EmbeddingIngestRequest, VideoEmbeddingHub, DEFAULT_HANDSHAKE_TTL_MS,
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

fn hub_two_step_handshake(
    hub: &VideoEmbeddingHub,
    producer_epoch: u64,
    producer_monotonic_ms: i64,
    producer_uncertainty_ms: u64,
    socket_same_host: bool,
) -> ClockHandshakeResponse {
    let chal = hub
        .clock_handshake(
            ClockHandshakeRequest {
                phase: Some("init".into()),
                producer_epoch,
                producer_monotonic_ms: Some(producer_monotonic_ms),
                ..Default::default()
            },
            socket_same_host,
        )
        .unwrap();
    assert_eq!(chal.phase, "challenge");
    hub.clock_handshake(
        ClockHandshakeRequest {
            phase: Some("confirm".into()),
            challenge_id: chal.challenge_id,
            producer_epoch,
            producer_monotonic_ms: Some(producer_monotonic_ms),
            producer_uncertainty_ms: Some(producer_uncertainty_ms),
            ..Default::default()
        },
        socket_same_host,
    )
    .unwrap()
}

async fn http_two_step_handshake(
    client: &reqwest::Client,
    addr: SocketAddr,
    token: Option<&str>,
    forwarded_for: Option<&str>,
    producer_epoch: u64,
    producer_monotonic_ms: i64,
    producer_uncertainty_ms: u64,
    server_uncertainty_ms: Option<u64>,
    ttl_ms: Option<u64>,
) -> reqwest::Response {
    let url = format!("http://{addr}/api/v1/vision/clock-handshake");
    let mut init_req = client.post(&url).json(&serde_json::json!({
        "phase": "init",
        "producer_epoch": producer_epoch,
        "producer_monotonic_ms": producer_monotonic_ms,
    }));
    if let Some(t) = token {
        init_req = init_req.header("Authorization", format!("Bearer {t}"));
    }
    if let Some(xff) = forwarded_for {
        init_req = init_req.header("X-Forwarded-For", xff);
    }
    let init_resp = init_req.send().await.unwrap();
    if !init_resp.status().is_success() {
        return init_resp;
    }
    let init_json: serde_json::Value = init_resp.json().await.unwrap();
    assert_eq!(init_json["phase"], "challenge");
    let challenge_id = init_json["challenge_id"].as_str().unwrap().to_string();

    let mut confirm_body = serde_json::json!({
        "phase": "confirm",
        "challenge_id": challenge_id,
        "producer_epoch": producer_epoch,
        "producer_monotonic_ms": producer_monotonic_ms,
        "producer_uncertainty_ms": producer_uncertainty_ms,
    });
    if let Some(su) = server_uncertainty_ms {
        confirm_body["server_uncertainty_ms"] = serde_json::json!(su);
    }
    if let Some(ttl) = ttl_ms {
        confirm_body["ttl_ms"] = serde_json::json!(ttl);
    }
    let mut confirm_req = client.post(&url).json(&confirm_body);
    if let Some(t) = token {
        confirm_req = confirm_req.header("Authorization", format!("Bearer {t}"));
    }
    if let Some(xff) = forwarded_for {
        confirm_req = confirm_req.header("X-Forwarded-For", xff);
    }
    confirm_req.send().await.unwrap()
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

    // Valid same-host 2-step clock handshake (now=1_000_000, producer_monotonic_ms=999_950 -> offset=+50 ms)
    // Also pass oversized ttl_ms=999_999 to verify server caps TTL <= 60_000 ms (A5).
    let hs_resp = http_two_step_handshake(
        &client,
        addr,
        Some(&token),
        None,
        1,
        999_950,
        3,
        Some(2),
        Some(999_999),
    )
    .await;
    assert_eq!(hs_resp.status(), reqwest::StatusCode::OK);
    let hs_json: serde_json::Value = hs_resp.json().await.unwrap();
    let hs_gen = hs_json["handshake_generation"].as_u64().unwrap();
    assert_eq!(hs_json["phase"], "confirmed");
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
                "oldest_captured_at_ms": cap_ms,
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
        { let s = inner.current_admission_stamp(); inner.begin_policy_run(s, PolicyRunStart {
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
            }) }
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
    let hs = hub_two_step_handshake(&hub, 1, 500_000, 1, true);
    hub.ingest_embedding(EmbeddingIngestRequest {
        handshake_generation: hs.handshake_generation,
        producer_epoch: 1,
        frame_id: 1,
        captured_at_ms: 500_000,
        oldest_captured_at_ms: Some(500_000),
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
        { let s = inner.current_admission_stamp(); inner.begin_policy_run(s, PolicyRunStart {
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
            }) }
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

    // A4: missing producer_uncertainty_ms on confirm rejected 400
    let init_for_missing_unc: serde_json::Value = client
        .post(format!("http://{addr}/api/v1/vision/clock-handshake"))
        .json(&serde_json::json!({"phase": "init", "producer_monotonic_ms": 10_000}))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let missing_unc = client
        .post(format!("http://{addr}/api/v1/vision/clock-handshake"))
        .json(&serde_json::json!({
            "phase": "confirm",
            "challenge_id": init_for_missing_unc["challenge_id"],
            "producer_monotonic_ms": 10_000
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(missing_unc.status(), reqwest::StatusCode::BAD_REQUEST);
    assert_eq!(
        missing_unc.json::<serde_json::Value>().await.unwrap()["code"],
        "MISSING_UNCERTAINTY"
    );

    // C1: Loopback connection (127.0.0.1) enforces <= 5 ms even if caller passes same_host=false
    let bad_local_init: serde_json::Value = client
        .post(format!("http://{addr}/api/v1/vision/clock-handshake"))
        .json(&serde_json::json!({
            "phase": "init",
            "producer_monotonic_ms": 10_000,
            "same_host": false
        }))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let bad_local = client
        .post(format!("http://{addr}/api/v1/vision/clock-handshake"))
        .json(&serde_json::json!({
            "phase": "confirm",
            "challenge_id": bad_local_init["challenge_id"],
            "producer_monotonic_ms": 10_000,
            "producer_uncertainty_ms": 6,
            "same_host": false
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(bad_local.status(), reqwest::StatusCode::BAD_REQUEST);

    // Remote IP via X-Forwarded-For enforces <= 25 ms (10 ms allowed, 26 ms rejected)
    let ok_remote_init: serde_json::Value = client
        .post(format!("http://{addr}/api/v1/vision/clock-handshake"))
        .header("X-Forwarded-For", "192.168.1.50")
        .json(&serde_json::json!({
            "phase": "init",
            "producer_epoch": 1,
            "client_monotonic_ms": 9_900
        }))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let ok_remote = client
        .post(format!("http://{addr}/api/v1/vision/clock-handshake"))
        .header("X-Forwarded-For", "192.168.1.50")
        .json(&serde_json::json!({
            "phase": "confirm",
            "challenge_id": ok_remote_init["challenge_id"],
            "producer_epoch": 1,
            "client_monotonic_ms": 9_900,
            "producer_uncertainty_ms": 6,
            "server_uncertainty_ms": 4
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(ok_remote.status(), reqwest::StatusCode::OK);
    let hs_json: serde_json::Value = ok_remote.json().await.unwrap();
    assert_eq!(hs_json["same_host"], false);
    assert_eq!(hs_json["clock_offset_ms"], 100);
    assert_eq!(hs_json["effective_uncertainty_ms"], 10);
    let hs_gen = hs_json["handshake_generation"].as_u64().unwrap();

    let bad_remote = http_two_step_handshake(
        &client,
        addr,
        None,
        Some("192.168.1.50"),
        1,
        9_900,
        26,
        None,
        None,
    )
    .await;
    assert_eq!(bad_remote.status(), reqwest::StatusCode::BAD_REQUEST);

    // A13: Pre-bind width budget rejects > 1024 embedding length
    let over_prebind = client
        .post(format!("http://{addr}/api/v1/vision/embedding"))
        .json(&serde_json::json!({
            "handshake_generation": hs_gen,
            "producer_epoch": 1,
            "frame_id": 1,
            "captured_at_ms": 9_900,
            "oldest_captured_at_ms": 9_900,
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
            "oldest_captured_at_ms": 9_900,
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
            "oldest_captured_at_ms": 9_911,
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
            "oldest_captured_at_ms": 9_750,
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
            "oldest_captured_at_ms": 9_850,
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
            "oldest_captured_at_ms": 9_900,
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
            "oldest_captured_at_ms": 9_900,
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
            "oldest_captured_at_ms": 9_900,
            "embedding": [1.0, 2.0],
            "encoder_artifact_sha256": sha_a(),
            "preprocess_config_sha256": sha_b()
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(r11.status(), reqwest::StatusCode::OK);
    assert_eq!(hub.window_len(), 1);

    let renew = http_two_step_handshake(&client, addr, None, None, 1, 9_800, 2, None, None).await;
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
                "oldest_captured_at_ms": 9_800,
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
            "oldest_captured_at_ms": 9_800,
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
    let hs = hub_two_step_handshake(&hub, 1, 1_000_000, 1, true);

    hub.ingest_embedding(EmbeddingIngestRequest {
        handshake_generation: hs.handshake_generation,
        producer_epoch: 1,
        frame_id: 1,
        captured_at_ms: 1_000_000,
        oldest_captured_at_ms: Some(1_000_000),
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
        oldest_captured_at_ms: Some(f2_ts as i64),
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
        { let s = inner.current_admission_stamp(); inner.begin_policy_run(s, PolicyRunStart {
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
            }) }
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
        pol.stop_reason.unwrap_or_default().to_string(),
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

#[tokio::test]
async fn b1_single_step_init_returns_challenge_without_activating_handshake() {
    let sup = Arc::new(ArmSupervisor::new(make_6dof_config()));
    let hub = embedding_hub_for_supervisor(&sup);
    hub.set_test_now_ms(Some(50_000));
    let addr = spawn_http_server(sup.clone(), OperatorAuth::disabled()).await;
    let client = reqwest::Client::new();

    let resp = client
        .post(format!("http://{addr}/api/v1/vision/clock-handshake"))
        .json(&serde_json::json!({
            "producer_epoch": 1,
            "producer_monotonic_ms": 49_950
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(resp.status(), reqwest::StatusCode::OK);
    let body: serde_json::Value = resp.json().await.unwrap();
    assert_eq!(body["phase"], "challenge");
    assert!(body["challenge_id"].as_str().is_some_and(|s| !s.is_empty()));
    assert_eq!(body["handshake_generation"], 0);
    assert_eq!(body["measured_rtt_ms"], serde_json::Value::Null);
    assert!(hub.handshake_state().is_none(), "init must not activate handshake");

    let ing = client
        .post(format!("http://{addr}/api/v1/vision/embedding"))
        .json(&serde_json::json!({
            "handshake_generation": 0,
            "producer_epoch": 1,
            "frame_id": 1,
            "captured_at_ms": 49_950,
            "oldest_captured_at_ms": 49_950,
            "embedding": [1.0, 2.0],
            "encoder_artifact_sha256": sha_a(),
            "preprocess_config_sha256": sha_b()
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(ing.status(), reqwest::StatusCode::CONFLICT);
    assert_eq!(
        ing.json::<serde_json::Value>().await.unwrap()["code"],
        "HANDSHAKE_MISMATCH"
    );
}

#[test]
fn b1_server_measured_rtt_sets_midpoint_offset_and_bounds_uncertainty() {
    let hub = VideoEmbeddingHub::new();
    hub.set_test_now_ms(Some(100_000));

    let init = hub
        .clock_handshake(
            ClockHandshakeRequest {
                phase: Some("init".into()),
                producer_epoch: 1,
                producer_monotonic_ms: Some(99_900),
                ..Default::default()
            },
            false,
        )
        .unwrap();
    assert_eq!(init.phase, "challenge");

    // Advance server clock by 10 ms between init and confirm.
    hub.set_test_now_ms(Some(100_010));
    let confirm = hub
        .clock_handshake(
            ClockHandshakeRequest {
                phase: Some("confirm".into()),
                challenge_id: init.challenge_id,
                producer_epoch: 1,
                producer_monotonic_ms: Some(99_910),
                producer_uncertainty_ms: Some(2),
                server_uncertainty_ms: None,
                ..Default::default()
            },
            false,
        )
        .unwrap();
    assert_eq!(confirm.phase, "confirmed");
    assert_eq!(confirm.measured_rtt_ms, Some(10));
    // midpoint = 100_000 + 10 / 2 = 100_005; offset = 100_005 - 99_910 = 95
    assert_eq!(confirm.clock_offset_ms, 95);
    assert_eq!(confirm.server_uncertainty_ms, 5);
    assert_eq!(confirm.effective_uncertainty_ms, 7);

    // Zero-RTT confirm still enforces server_uncertainty_ms >= 1 (never 0).
    let init2 = hub
        .clock_handshake(
            ClockHandshakeRequest {
                phase: Some("init".into()),
                producer_epoch: 1,
                producer_monotonic_ms: Some(99_910),
                ..Default::default()
            },
            true,
        )
        .unwrap();
    let confirm2 = hub
        .clock_handshake(
            ClockHandshakeRequest {
                phase: Some("confirm".into()),
                challenge_id: init2.challenge_id,
                producer_epoch: 1,
                producer_monotonic_ms: Some(99_910),
                producer_uncertainty_ms: Some(0),
                server_uncertainty_ms: Some(0),
                ..Default::default()
            },
            true,
        )
        .unwrap();
    assert_eq!(confirm2.measured_rtt_ms, Some(0));
    assert_eq!(confirm2.server_uncertainty_ms, 1);
    assert_eq!(confirm2.effective_uncertainty_ms, 1);
}

#[test]
fn b1_excessive_server_measured_rtt_rejected_without_clobbering_active_state() {
    let hub = VideoEmbeddingHub::new();
    hub.set_test_now_ms(Some(200_000));
    let hs = hub_two_step_handshake(&hub, 1, 200_000, 1, true);
    hub.ingest_embedding(EmbeddingIngestRequest {
        handshake_generation: hs.handshake_generation,
        producer_epoch: 1,
        frame_id: 1,
        captured_at_ms: 200_000,
        oldest_captured_at_ms: Some(200_000),
        embedding: vec![1.0, 2.0],
        encoder_artifact_sha256: sha_a(),
        preprocess_config_sha256: sha_b(),
        model_generation: None,
    })
    .unwrap();
    let prev_hs = hub.handshake_state().unwrap();

    // Same-host: 10 ms RTT -> server_unc=5 + producer_unc=1 = 6 > 5 -> UNCERTAINTY_EXCEEDED
    let init_local = hub
        .clock_handshake(
            ClockHandshakeRequest {
                phase: Some("init".into()),
                producer_epoch: 1,
                producer_monotonic_ms: Some(200_000),
                ..Default::default()
            },
            true,
        )
        .unwrap();
    hub.set_test_now_ms(Some(200_010));
    let err_local = hub
        .clock_handshake(
            ClockHandshakeRequest {
                phase: Some("confirm".into()),
                challenge_id: init_local.challenge_id,
                producer_epoch: 1,
                producer_monotonic_ms: Some(200_010),
                producer_uncertainty_ms: Some(1),
                ..Default::default()
            },
            true,
        )
        .unwrap_err();
    assert_eq!(err_local.code, "UNCERTAINTY_EXCEEDED");
    assert_eq!(hub.handshake_state().unwrap(), prev_hs);
    assert_eq!(hub.frame_ids(), vec![1]);

    // Remote: 50 ms RTT -> server_unc=25 + producer_unc=1 = 26 > 25 -> UNCERTAINTY_EXCEEDED
    let init_remote = hub
        .clock_handshake(
            ClockHandshakeRequest {
                phase: Some("init".into()),
                producer_epoch: 1,
                producer_monotonic_ms: Some(200_010),
                ..Default::default()
            },
            false,
        )
        .unwrap();
    hub.set_test_now_ms(Some(200_060));
    let err_remote = hub
        .clock_handshake(
            ClockHandshakeRequest {
                phase: Some("confirm".into()),
                challenge_id: init_remote.challenge_id,
                producer_epoch: 1,
                producer_monotonic_ms: Some(200_060),
                producer_uncertainty_ms: Some(1),
                ..Default::default()
            },
            false,
        )
        .unwrap_err();
    assert_eq!(err_remote.code, "UNCERTAINTY_EXCEEDED");
    assert_eq!(hub.handshake_state().unwrap(), prev_hs);
    assert_eq!(hub.frame_ids(), vec![1]);
}

#[test]
fn b1_replayed_expired_and_stale_challenge_rejected() {
    let hub = VideoEmbeddingHub::new();
    hub.set_test_now_ms(Some(300_000));

    // Replay check
    let init1 = hub
        .clock_handshake(
            ClockHandshakeRequest {
                phase: Some("init".into()),
                producer_epoch: 1,
                producer_monotonic_ms: Some(300_000),
                ..Default::default()
            },
            true,
        )
        .unwrap();
    let cid1 = init1.challenge_id.clone().unwrap();
    hub.clock_handshake(
        ClockHandshakeRequest {
            phase: Some("confirm".into()),
            challenge_id: Some(cid1.clone()),
            producer_epoch: 1,
            producer_monotonic_ms: Some(300_000),
            producer_uncertainty_ms: Some(1),
            ..Default::default()
        },
        true,
    )
    .unwrap();
    let replay = hub
        .clock_handshake(
            ClockHandshakeRequest {
                phase: Some("confirm".into()),
                challenge_id: Some(cid1),
                producer_epoch: 1,
                producer_monotonic_ms: Some(300_000),
                producer_uncertainty_ms: Some(1),
                ..Default::default()
            },
            true,
        )
        .unwrap_err();
    assert_eq!(replay.code, "INVALID_CHALLENGE");

    // Expired challenge check (> 5_000 ms)
    let init2 = hub
        .clock_handshake(
            ClockHandshakeRequest {
                phase: Some("init".into()),
                producer_epoch: 1,
                producer_monotonic_ms: Some(300_000),
                ..Default::default()
            },
            true,
        )
        .unwrap();
    hub.set_test_now_ms(Some(305_001));
    let expired = hub
        .clock_handshake(
            ClockHandshakeRequest {
                phase: Some("confirm".into()),
                challenge_id: init2.challenge_id,
                producer_epoch: 1,
                producer_monotonic_ms: Some(305_001),
                producer_uncertainty_ms: Some(1),
                ..Default::default()
            },
            true,
        )
        .unwrap_err();
    assert_eq!(expired.code, "CHALLENGE_EXPIRED");

    // Stale challenge check (superseded by a newer confirmed handshake generation)
    let init_a = hub
        .clock_handshake(
            ClockHandshakeRequest {
                phase: Some("init".into()),
                producer_epoch: 1,
                producer_monotonic_ms: Some(305_001),
                ..Default::default()
            },
            true,
        )
        .unwrap();
    let _hs_b = hub_two_step_handshake(&hub, 1, 305_001, 1, true);
    let stale = hub
        .clock_handshake(
            ClockHandshakeRequest {
                phase: Some("confirm".into()),
                challenge_id: init_a.challenge_id,
                producer_epoch: 1,
                producer_monotonic_ms: Some(305_001),
                producer_uncertainty_ms: Some(1),
                ..Default::default()
            },
            true,
        )
        .unwrap_err();
    assert_eq!(stale.code, "CHALLENGE_STALE");
}

#[tokio::test]
async fn b1_oldest_plausible_age_enforced_at_ingest_and_pre_commit() {
    let layers = make_linear_layers(8, 6, vec![0.05; 48]);
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
                "history_len": 1,
                "max_staleness_ms": 50,
                "max_window_span_ms": 200,
                "fps_staleness_budget_ms": 50,
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
    hub.set_test_now_ms(Some(1_000_000));
    let init = hub
        .clock_handshake(
            ClockHandshakeRequest {
                phase: Some("init".into()),
                producer_epoch: 1,
                producer_monotonic_ms: Some(1_000_000),
                ..Default::default()
            },
            true,
        )
        .unwrap();
    let hs = hub
        .clock_handshake(
            ClockHandshakeRequest {
                phase: Some("confirm".into()),
                challenge_id: init.challenge_id,
                producer_epoch: 1,
                producer_monotonic_ms: Some(1_000_000),
                producer_uncertainty_ms: Some(3),
                server_uncertainty_ms: Some(2),
                ..Default::default()
            },
            true,
        )
        .unwrap();
    assert_eq!(hs.effective_uncertainty_ms, 5);

    let spec = prepared.contract().video_embedding.clone().unwrap();
    hub.bind_policy(99, spec, 10.0);

    // At ingest: nominal age 46 <= 50, but 46 + 5 = 51 > 50 -> STALE_FRAME
    let err_ingest = hub
        .ingest_embedding(EmbeddingIngestRequest {
            handshake_generation: hs.handshake_generation,
            producer_epoch: 1,
            frame_id: 1,
            captured_at_ms: 1_000_000 - 46,
            oldest_captured_at_ms: Some(1_000_000 - 46),
            embedding: vec![1.0, 1.0],
            encoder_artifact_sha256: sha_a(),
            preprocess_config_sha256: sha_b(),
            model_generation: Some(1),
        })
        .unwrap_err();
    assert_eq!(err_ingest.code, "STALE_FRAME");

    // Ingest at nominal age 40 (40 + 5 = 45 <= 50 -> accepted)
    hub.ingest_embedding(EmbeddingIngestRequest {
        handshake_generation: hs.handshake_generation,
        producer_epoch: 1,
        frame_id: 2,
        captured_at_ms: 1_000_000 - 40,
        oldest_captured_at_ms: Some(1_000_000 - 40),
        embedding: vec![1.0, 1.0],
        encoder_artifact_sha256: sha_a(),
        preprocess_config_sha256: sha_b(),
        model_generation: Some(1),
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

    let lease_gen = sup.acquire_lease("conn-b1").await.unwrap();
    sup.set_torque(true).await.unwrap();
    sup.tick(0.02).await;
    let epoch = sup.get_observation().await.epoch_id;

    let action_id = {
        let mut inner = sup.inner().write().await;
        { let s = inner.current_admission_stamp(); inner.begin_policy_run(s, PolicyRunStart {
                epoch_id: epoch,
                connection_id: "conn-b1".into(),
                lease_generation: lease_gen,
                policy_id: "vid_b1".into(),
                artifact_digest: digest,
                backend: "mlp".into(),
                mode: PolicyMode::Live,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: true,
                verified_release_gate: Some(gate),
            }) }
            .unwrap()
    };

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

    tokio::time::timeout(Duration::from_secs(5), infer_entered.notified())
        .await
        .expect("timed out waiting for BarrierBackend::infer");

    // Advance by 6 ms: nominal age = 46 <= 50, but 46 + 5 = 51 > 50 -> pre-commit rejects!
    hub.advance_test_now_ms(6);

    {
        let (lock, cvar) = &*allow_finish;
        *lock.lock().unwrap() = true;
        cvar.notify_all();
    }
    tokio::time::timeout(Duration::from_secs(5), runner)
        .await
        .expect("runner timeout")
        .expect("runner panic");

    assert_eq!(writes.load(Ordering::SeqCst), 0);
    assert_eq!(non_home_writes.load(Ordering::SeqCst), 0);
    let obs = sup.get_observation().await;
    let pol = obs.policy.expect("policy obs");
    assert_eq!(pol.stop_reason.as_deref(), Some("VISION_STALE"));
    assert_eq!(pol.steps, 0);
}

#[tokio::test]
async fn b2_http_ingest_enforces_oldest_endpoint_staleness_and_bounds() {
    let sup = Arc::new(ArmSupervisor::new(make_6dof_config()));
    let hub = embedding_hub_for_supervisor(&sup);
    hub.set_test_now_ms(Some(500_000));
    let addr = spawn_http_server(sup.clone(), OperatorAuth::disabled()).await;
    let client = reqwest::Client::new();

    let hs_resp = http_two_step_handshake(&client, addr, None, None, 1, 500_000, 1, Some(1), None).await;
    assert_eq!(hs_resp.status(), reqwest::StatusCode::OK);
    let hs_json: serde_json::Value = hs_resp.json().await.unwrap();
    assert_eq!(hs_json["effective_uncertainty_ms"], 2);
    let hs_gen = hs_json["handshake_generation"].as_u64().unwrap();

    hub.bind_policy(
        1,
        VideoEmbeddingSpec {
            embedding_dim: 2,
            history_len: 1,
            max_staleness_ms: 50,
            max_window_span_ms: 200,
            fps_staleness_budget_ms: Some(50),
            encoder_artifact_sha256: sha_a(),
            preprocess_config_sha256: sha_b(),
            embedding_norm: None,
            on_stale: armd::policy_runtime::VisionStalePolicy::Stop,
        },
        10.0,
    );

    // Invalid bounds: oldest_captured_at_ms > captured_at_ms -> 400 INVALID_CLIP_BOUNDS
    let bad_bounds = client
        .post(format!("http://{addr}/api/v1/vision/embedding"))
        .json(&serde_json::json!({
            "handshake_generation": hs_gen,
            "producer_epoch": 1,
            "frame_id": 1,
            "captured_at_ms": 499_990,
            "oldest_captured_at_ms": 499_995,
            "embedding": [1.0, 2.0],
            "encoder_artifact_sha256": sha_a(),
            "preprocess_config_sha256": sha_b()
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(bad_bounds.status(), reqwest::StatusCode::BAD_REQUEST);
    assert_eq!(
        bad_bounds.json::<serde_json::Value>().await.unwrap()["code"],
        "INVALID_CLIP_BOUNDS"
    );

    // Newest endpoint is fresh (age 5 ms), but oldest endpoint age 80 + unc 2 = 82 > eff_staleness 50 -> 400 STALE_FRAME
    let stale_oldest = client
        .post(format!("http://{addr}/api/v1/vision/embedding"))
        .json(&serde_json::json!({
            "handshake_generation": hs_gen,
            "producer_epoch": 1,
            "frame_id": 2,
            "captured_at_ms": 499_995,
            "oldest_captured_at_ms": 499_920,
            "embedding": [1.0, 2.0],
            "encoder_artifact_sha256": sha_a(),
            "preprocess_config_sha256": sha_b()
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(stale_oldest.status(), reqwest::StatusCode::BAD_REQUEST);
    assert_eq!(
        stale_oldest.json::<serde_json::Value>().await.unwrap()["code"],
        "STALE_FRAME"
    );

    // Accepted when oldest endpoint age 48 + unc 2 = 50 <= 50
    let ok_clip = client
        .post(format!("http://{addr}/api/v1/vision/embedding"))
        .json(&serde_json::json!({
            "handshake_generation": hs_gen,
            "producer_epoch": 1,
            "frame_id": 3,
            "captured_at_ms": 499_995,
            "oldest_captured_at_ms": 499_952,
            "embedding": [1.0, 2.0],
            "encoder_artifact_sha256": sha_a(),
            "preprocess_config_sha256": sha_b()
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(ok_clip.status(), reqwest::StatusCode::OK);
    let ok_json: serde_json::Value = ok_clip.json().await.unwrap();
    assert_eq!(ok_json["mapped_oldest_captured_at_ms"], 499_952);
}

#[tokio::test]
async fn b2_mid_inference_temporal_clip_oldest_endpoint_expires_blocks_commit() {
    let layers = make_linear_layers(8, 6, vec![0.05; 48]);
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
                "history_len": 1,
                "max_staleness_ms": 100,
                "max_window_span_ms": 300,
                "fps_staleness_budget_ms": 100,
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
    // Handshake with unc = 2 (server=1, producer=1)
    hub.set_test_now_ms(Some(1_000_000));
    let hs = hub_two_step_handshake(&hub, 1, 1_000_000, 1, true);
    assert_eq!(hs.effective_uncertainty_ms, 2);

    // Clip [oldest=999_910, newest=1_000_000].
    // eff_staleness = 100, unc = 2 -> exact deadline = 999_910 + 100 - 2 = 1_000_008.
    // Newest endpoint deadline would be 1_000_098 (plenty of slack).
    hub.ingest_embedding(EmbeddingIngestRequest {
        handshake_generation: hs.handshake_generation,
        producer_epoch: 1,
        frame_id: 1,
        captured_at_ms: 1_000_000,
        oldest_captured_at_ms: Some(999_910),
        embedding: vec![1.0, 1.0],
        encoder_artifact_sha256: sha_a(),
        preprocess_config_sha256: sha_b(),
        model_generation: Some(1),
    })
    .unwrap();

    // Advance to the exact oldest-endpoint deadline so sampling succeeds.
    hub.set_test_now_ms(Some(1_000_008));

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

    let lease_gen = sup.acquire_lease("conn-b2").await.unwrap();
    sup.set_torque(true).await.unwrap();
    sup.tick(0.02).await;
    let epoch = sup.get_observation().await.epoch_id;

    let action_id = {
        let mut inner = sup.inner().write().await;
        { let s = inner.current_admission_stamp(); inner.begin_policy_run(s, PolicyRunStart {
                epoch_id: epoch,
                connection_id: "conn-b2".into(),
                lease_generation: lease_gen,
                policy_id: "vid_b2".into(),
                artifact_digest: digest,
                backend: "mlp".into(),
                mode: PolicyMode::Live,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: true,
                verified_release_gate: Some(gate),
            }) }
            .unwrap()
    };

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

    tokio::time::timeout(Duration::from_secs(5), infer_entered.notified())
        .await
        .expect("timed out waiting for BarrierBackend::infer");

    // Advance by 1 ms: oldest endpoint expires (1_000_009 > 1_000_008) while newest (age 9+2=11 <= 100) is fresh!
    hub.set_test_now_ms(Some(1_000_009));

    {
        let (lock, cvar) = &*allow_finish;
        *lock.lock().unwrap() = true;
        cvar.notify_all();
    }
    tokio::time::timeout(Duration::from_secs(5), runner)
        .await
        .expect("runner timeout")
        .expect("runner panic");

    assert_eq!(writes.load(Ordering::SeqCst), 0);
    assert_eq!(non_home_writes.load(Ordering::SeqCst), 0);
    let obs = sup.get_observation().await;
    let pol = obs.policy.expect("policy obs");
    assert_eq!(pol.stop_reason.as_deref(), Some("VISION_STALE"));
    assert_eq!(pol.steps, 0);
}

#[tokio::test]
async fn m3_unbroken_renewal_mid_inference_discards_target_holds_and_resumes() {
    let mut weights = vec![0.0f32; 6 * 8];
    weights[6] = 0.05;
    weights[7] = 0.05;
    let layers = make_linear_layers(8, 6, weights);
    let dir = tempdir().unwrap();
    let digest = write_rmlp_pack(
        dir.path(),
        &layers,
        serde_json::json!({
            "fps": 20.0,
            "vision": {
                "source": "video_embedding",
                "layout": "embedding_v1",
                "embedding_dim": 2,
                "history_len": 1,
                "max_staleness_ms": 300,
                "max_window_span_ms": 300,
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
    hub.set_test_now_ms(Some(1_000_000));
    let hs1 = hub_two_step_handshake(&hub, 1, 1_000_000, 1, true);
    hub.ingest_embedding(EmbeddingIngestRequest {
        handshake_generation: hs1.handshake_generation,
        producer_epoch: 1,
        frame_id: 1,
        captured_at_ms: 1_000_000,
        oldest_captured_at_ms: Some(1_000_000),
        embedding: vec![1.0, 1.0],
        encoder_artifact_sha256: sha_a(),
        preprocess_config_sha256: sha_b(),
        model_generation: Some(1),
    })
    .unwrap();

    let raw_gate = make_valid_release_gate(&digest, &sha_a(), &sha_b(), 20);
    let gate = ReleaseGate::parse_and_verify(
        &raw_gate,
        &digest,
        &cfg,
        prepared.contract(),
        20.0,
        &ActiveDeploymentContext {
            scene_id: "pcb_suction".into(),
            tool_id: "suction".into(),
            calibration_digest: EMPTY_CALIBRATION_DIGEST.into(),
        },
    )
    .unwrap();

    let lease_gen = sup.acquire_lease("conn-m3").await.unwrap();
    sup.set_torque(true).await.unwrap();
    sup.tick(0.02).await;
    let epoch = sup.get_observation().await.epoch_id;

    let action_id = {
        let mut inner = sup.inner().write().await;
        { let s = inner.current_admission_stamp(); inner.begin_policy_run(s, PolicyRunStart {
                epoch_id: epoch,
                connection_id: "conn-m3".into(),
                lease_generation: lease_gen,
                policy_id: "vid_m3".into(),
                artifact_digest: digest,
                backend: "mlp".into(),
                mode: PolicyMode::Live,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: true,
                verified_release_gate: Some(gate),
            }) }
            .unwrap()
    };

    writes.store(0, Ordering::SeqCst);
    non_home_writes.store(0, Ordering::SeqCst);

    let runner = spawn_policy_runner(
        sup.clone(),
        action_id,
        prepared,
        20.0,
        compute_policy_deadline(Some(0.35)).unwrap(),
        None,
        None,
    );

    tokio::time::timeout(Duration::from_secs(5), infer_entered.notified())
        .await
        .expect("timed out waiting for BarrierBackend::infer");

    // Unbroken same-epoch renewal while prior handshake is still valid (1_000_020 <= 1_060_000)
    hub.set_test_now_ms(Some(1_000_020));
    let hs2 = hub_two_step_handshake(&hub, 1, 1_000_020, 1, true);
    assert!(hs2.handshake_generation > hs1.handshake_generation);

    // Allow step 1 (and subsequent steps) to proceed
    {
        let (lock, cvar) = &*allow_finish;
        *lock.lock().unwrap() = true;
        cvar.notify_all();
    }

    // Step 1's target must be discarded and policy held while waiting for frames under hs2
    tokio::time::sleep(Duration::from_millis(80)).await;
    sup.tick(0.02).await;
    let mid_obs = sup.get_observation().await;
    let mid_pol = mid_obs.policy.expect("policy still running in hold");
    assert!(mid_pol.active, "policy must remain active during unbroken renewal hold");
    assert_eq!(mid_pol.steps, 0, "in-flight target from old handshake must be discarded");
    assert_eq!(non_home_writes.load(Ordering::SeqCst), 0);

    // Refill under hs2 and let the runner commit
    hub.ingest_embedding(EmbeddingIngestRequest {
        handshake_generation: hs2.handshake_generation,
        producer_epoch: 1,
        frame_id: 2,
        captured_at_ms: 1_000_020,
        oldest_captured_at_ms: Some(1_000_020),
        embedding: vec![1.0, 1.0],
        encoder_artifact_sha256: sha_a(),
        preprocess_config_sha256: sha_b(),
        model_generation: Some(1),
    })
    .unwrap();

    for _ in 0..8 {
        tokio::time::sleep(Duration::from_millis(45)).await;
        sup.tick(0.02).await;
    }
    tokio::time::timeout(Duration::from_secs(5), runner)
        .await
        .expect("runner timeout")
        .expect("runner panic");

    let end_obs = sup.get_observation().await;
    let end_pol = end_obs.policy.expect("policy obs");
    assert_eq!(end_pol.stop_reason.as_deref(), Some("MAX_SECONDS"));
    assert!(end_pol.steps >= 1);
    assert_eq!(end_pol.vision_frame_id, Some(2));
    assert!(non_home_writes.load(Ordering::SeqCst) >= 1);
}

#[test]
fn m3_repeated_renewal_without_frames_times_out_after_max_window_span() {
    let hub = VideoEmbeddingHub::new();
    let spec = VideoEmbeddingSpec {
        embedding_dim: 2,
        history_len: 1,
        max_staleness_ms: 200,
        max_window_span_ms: 250,
        fps_staleness_budget_ms: Some(200),
        encoder_artifact_sha256: sha_a(),
        preprocess_config_sha256: sha_b(),
        embedding_norm: None,
        on_stale: armd::policy_runtime::VisionStalePolicy::Stop,
    };
    hub.set_test_now_ms(Some(1_000_000));
    hub.bind_policy(1, spec.clone(), 20.0);
    let hs1 = hub_two_step_handshake(&hub, 1, 1_000_000, 1, true);
    hub.ingest_embedding(EmbeddingIngestRequest {
        handshake_generation: hs1.handshake_generation,
        producer_epoch: 1,
        frame_id: 1,
        captured_at_ms: 1_000_000,
        oldest_captured_at_ms: Some(1_000_000),
        embedding: vec![1.0, 2.0],
        encoder_artifact_sha256: sha_a(),
        preprocess_config_sha256: sha_b(),
        model_generation: None,
    })
    .unwrap();
    assert!(matches!(
        hub.sample_window(&spec, 1_000_000),
        armd::policy_runtime::vision::EmbeddingSampleDecision::Ready(_)
    ));

    // Unbroken renewal at 1_000_050 starts refill timer at 1_000_050
    hub.set_test_now_ms(Some(1_000_050));
    let _hs2 = hub_two_step_handshake(&hub, 1, 1_000_050, 1, true);
    assert!(matches!(
        hub.sample_window(&spec, 1_000_000),
        armd::policy_runtime::vision::EmbeddingSampleDecision::Hold
    ));

    // Repeated renewal at 1_000_200 without frames MUST NOT reset refill_started_at_ms
    hub.set_test_now_ms(Some(1_000_200));
    let _hs3 = hub_two_step_handshake(&hub, 1, 1_000_200, 1, true);
    assert!(matches!(
        hub.sample_window(&spec, 1_000_000),
        armd::policy_runtime::vision::EmbeddingSampleDecision::Hold
    ));

    // At 1_000_300 (elapsed 250 <= 250): still Hold
    hub.set_test_now_ms(Some(1_000_300));
    assert!(matches!(
        hub.sample_window(&spec, 1_000_000),
        armd::policy_runtime::vision::EmbeddingSampleDecision::Hold
    ));

    // At 1_000_301 (elapsed 251 > 250): bounded refill hold expires -> VISION_STALE
    hub.set_test_now_ms(Some(1_000_301));
    assert!(matches!(
        hub.sample_window(&spec, 1_000_000),
        armd::policy_runtime::vision::EmbeddingSampleDecision::Stop { reason: "VISION_STALE", .. }
    ));
}

async fn run_m3_fail_closed_case(case: &str) -> (String, usize, usize, u64) {
    let layers = make_linear_layers(8, 6, vec![0.05; 48]);
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
                "history_len": 1,
                "max_staleness_ms": 300,
                "max_window_span_ms": 300,
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
    if case == "late_after_expiry" {
        hub.set_default_handshake_ttl_ms(50);
    }
    hub.set_test_now_ms(Some(1_000_000));
    let hs1 = hub_two_step_handshake(&hub, 1, 1_000_000, 1, true);
    hub.ingest_embedding(EmbeddingIngestRequest {
        handshake_generation: hs1.handshake_generation,
        producer_epoch: 1,
        frame_id: 1,
        captured_at_ms: 1_000_000,
        oldest_captured_at_ms: Some(1_000_000),
        embedding: vec![1.0, 1.0],
        encoder_artifact_sha256: sha_a(),
        preprocess_config_sha256: sha_b(),
        model_generation: Some(1),
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

    let lease_gen = sup.acquire_lease("conn-m3-fc").await.unwrap();
    sup.set_torque(true).await.unwrap();
    sup.tick(0.02).await;
    let epoch = sup.get_observation().await.epoch_id;

    let action_id = {
        let mut inner = sup.inner().write().await;
        { let s = inner.current_admission_stamp(); inner.begin_policy_run(s, PolicyRunStart {
                epoch_id: epoch,
                connection_id: "conn-m3-fc".into(),
                lease_generation: lease_gen,
                policy_id: "vid_m3_fc".into(),
                artifact_digest: digest,
                backend: "mlp".into(),
                mode: PolicyMode::Live,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: true,
                verified_release_gate: Some(gate),
            }) }
            .unwrap()
    };

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

    tokio::time::timeout(Duration::from_secs(5), infer_entered.notified())
        .await
        .expect("timed out waiting for BarrierBackend::infer");

    match case {
        "late_after_expiry" => {
            hub.set_test_now_ms(Some(1_000_051));
            let _ = hub_two_step_handshake(&hub, 1, 1_000_051, 1, true);
        }
        "with_lease_loss" => {
            let _ = hub_two_step_handshake(&hub, 1, 1_000_000, 1, true);
            let _ = sup.preempt_lease("conn-hijack", "hijack").await.unwrap();
        }
        "with_model_change" => {
            let _ = hub_two_step_handshake(&hub, 1, 1_000_000, 1, true);
            hub.set_model_generation(Some(999));
        }
        _ => unreachable!(),
    }

    {
        let (lock, cvar) = &*allow_finish;
        *lock.lock().unwrap() = true;
        cvar.notify_all();
    }
    tokio::time::timeout(Duration::from_secs(5), runner)
        .await
        .expect("runner timeout")
        .expect("runner panic");

    let obs = sup.get_observation().await;
    let pol = obs.policy.expect("policy obs");
    (
        pol.stop_reason.unwrap_or_default().to_string(),
        writes.load(Ordering::SeqCst),
        non_home_writes.load(Ordering::SeqCst),
        pol.steps,
    )
}

#[tokio::test]
async fn m3_late_renewal_and_coincident_lease_or_model_change_fail_closed() {
    let (r1, w1, nh1, s1) = run_m3_fail_closed_case("late_after_expiry").await;
    assert_eq!(r1, "HANDSHAKE_EXPIRED");
    assert_eq!((w1, nh1, s1), (0, 0, 0));

    let (r2, w2, nh2, s2) = run_m3_fail_closed_case("with_lease_loss").await;
    assert_eq!(r2, "LEASE_LOST");
    assert_eq!((w2, nh2, s2), (0, 0, 0));

    let (r3, w3, nh3, s3) = run_m3_fail_closed_case("with_model_change").await;
    assert_eq!(r3, "VISION_GENERATION_CHANGED");
    assert_eq!((w3, nh3, s3), (0, 0, 0));
}

async fn run_m4_counter_case(
    initial_counters: Option<(u64, u64)>,
    initial_producer_model_gen: Option<u64>,
    mid_infer_action: impl FnOnce(&VideoEmbeddingHub, u64),
) -> (String, usize, usize, u64) {
    let layers = make_linear_layers(8, 6, vec![0.05; 48]);
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
                "history_len": 1,
                "max_staleness_ms": 300,
                "max_window_span_ms": 300,
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
    hub.set_test_now_ms(Some(1_000_000));
    let hs = hub_two_step_handshake(&hub, 1, 1_000_000, 1, true);
    hub.ingest_embedding(EmbeddingIngestRequest {
        handshake_generation: hs.handshake_generation,
        producer_epoch: 1,
        frame_id: 1,
        captured_at_ms: 1_000_000,
        oldest_captured_at_ms: Some(1_000_000),
        embedding: vec![1.0, 1.0],
        encoder_artifact_sha256: sha_a(),
        preprocess_config_sha256: sha_b(),
        model_generation: initial_producer_model_gen,
    })
    .unwrap();

    if let Some((sme, inv)) = initial_counters {
        hub.set_counters_for_test(sme, inv);
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

    let lease_gen = sup.acquire_lease("conn-m4").await.unwrap();
    sup.set_torque(true).await.unwrap();
    sup.tick(0.02).await;
    let epoch = sup.get_observation().await.epoch_id;

    let action_id = {
        let mut inner = sup.inner().write().await;
        { let s = inner.current_admission_stamp(); inner.begin_policy_run(s, PolicyRunStart {
                epoch_id: epoch,
                connection_id: "conn-m4".into(),
                lease_generation: lease_gen,
                policy_id: "vid_m4".into(),
                artifact_digest: digest,
                backend: "mlp".into(),
                mode: PolicyMode::Live,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: true,
                verified_release_gate: Some(gate),
            }) }
            .unwrap()
    };

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

    tokio::time::timeout(Duration::from_secs(5), infer_entered.notified())
        .await
        .expect("timed out waiting for BarrierBackend::infer");

    mid_infer_action(&hub, hs.handshake_generation);

    {
        let (lock, cvar) = &*allow_finish;
        *lock.lock().unwrap() = true;
        cvar.notify_all();
    }
    tokio::time::timeout(Duration::from_secs(5), runner)
        .await
        .expect("runner timeout")
        .expect("runner panic");

    let obs = sup.get_observation().await;
    let pol = obs.policy.expect("policy obs");
    (
        pol.stop_reason.unwrap_or_default().to_string(),
        writes.load(Ordering::SeqCst),
        non_home_writes.load(Ordering::SeqCst),
        pol.steps,
    )
}

#[tokio::test]
async fn m4_producer_model_generation_u64_max_to_1_invalidates_ticket() {
    let (reason, writes, non_home, steps) = run_m4_counter_case(
        None,
        Some(u64::MAX),
        |hub, hs_gen| {
            hub.ingest_embedding(EmbeddingIngestRequest {
                handshake_generation: hs_gen,
                producer_epoch: 1,
                frame_id: 2,
                captured_at_ms: 1_000_000,
                oldest_captured_at_ms: Some(1_000_000),
                embedding: vec![1.0, 1.0],
                encoder_artifact_sha256: sha_a(),
                preprocess_config_sha256: sha_b(),
                model_generation: Some(1),
            })
            .unwrap();
        },
    )
    .await;
    assert_eq!(reason, "VISION_GENERATION_CHANGED");
    assert_eq!((writes, non_home, steps), (0, 0, 0));
}

#[tokio::test]
async fn m4_server_counters_wrap_from_u64_max_to_0_and_still_invalidate_tickets() {
    // Setting u64::MAX - 1 before spawn_policy_runner means bind_policy advances both to u64::MAX.
    let (r_model, w1, nh1, s1) = run_m4_counter_case(
        Some((u64::MAX - 1, u64::MAX - 1)),
        Some(1),
        |hub, _hs_gen| {
            assert_eq!(hub.live_ticket_state().model_generation, Some(u64::MAX));
            hub.set_model_generation(Some(2));
            assert_eq!(hub.live_ticket_state().model_generation, Some(0));
        },
    )
    .await;
    assert_eq!(r_model, "VISION_GENERATION_CHANGED");
    assert_eq!((w1, nh1, s1), (0, 0, 0));

    let (r_inv, w2, nh2, s2) = run_m4_counter_case(
        Some((u64::MAX - 1, u64::MAX - 1)),
        Some(1),
        |hub, _hs_gen| {
            assert_eq!(hub.invalidation_generation(), u64::MAX);
            let next = hub.bump_invalidation_generation();
            assert_eq!(next, 0);
        },
    )
    .await;
    assert_eq!(r_inv, "WINDOW_INVALIDATED");
    assert_eq!((w2, nh2, s2), (0, 0, 0));
}

#[tokio::test]
async fn m1_on_stale_hold_discards_expired_target_and_resumes_whereas_lease_loss_terminates() {
    let mut weights = vec![0.0f32; 6 * 8];
    weights[6] = 0.05;
    weights[7] = 0.05;
    let layers = make_linear_layers(8, 6, weights);
    let dir = tempdir().unwrap();
    let digest = write_rmlp_pack(
        dir.path(),
        &layers,
        serde_json::json!({
            "fps": 20.0,
            "vision": {
                "source": "video_embedding",
                "layout": "embedding_v1",
                "embedding_dim": 2,
                "history_len": 1,
                "max_staleness_ms": 50,
                "max_window_span_ms": 300,
                "fps_staleness_budget_ms": 50,
                "on_stale": "hold",
                "encoder_artifact_sha256": sha_a(),
                "preprocess_config_sha256": sha_b()
            }
        }),
    );

    let cfg = make_6dof_config();

    // Part 1: on_stale = "hold" discards stale target mid-inference, holds, and resumes on fresh frame
    {
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
        let prepared =
            PreparedPolicy::from_backend(snapshot, contract, barrier_backend, &cfg).unwrap();

        let writes = Arc::new(AtomicUsize::new(0));
        let non_home_writes = Arc::new(AtomicUsize::new(0));
        let drv = CountingRealDriver::new(vec![0.0; 6], writes.clone(), non_home_writes.clone());
        let sup = Arc::new(ArmSupervisor::new(cfg.clone()).with_driver(Box::new(drv)));
        sup.set_follower("so-arm101").await;

        let hub = embedding_hub_for_supervisor(&sup);
        hub.set_test_now_ms(Some(1_000_000));
        let hs = hub_two_step_handshake(&hub, 1, 1_000_000, 1, true);
        hub.ingest_embedding(EmbeddingIngestRequest {
            handshake_generation: hs.handshake_generation,
            producer_epoch: 1,
            frame_id: 1,
            captured_at_ms: 1_000_000,
            oldest_captured_at_ms: Some(1_000_000),
            embedding: vec![1.0, 1.0],
            encoder_artifact_sha256: sha_a(),
            preprocess_config_sha256: sha_b(),
            model_generation: Some(1),
        })
        .unwrap();

        let raw_gate = make_valid_release_gate(&digest, &sha_a(), &sha_b(), 20);
        let gate = ReleaseGate::parse_and_verify(
            &raw_gate,
            &digest,
            &cfg,
            prepared.contract(),
            20.0,
            &ActiveDeploymentContext {
                scene_id: "pcb_suction".into(),
                tool_id: "suction".into(),
                calibration_digest: EMPTY_CALIBRATION_DIGEST.into(),
            },
        )
        .unwrap();

        let lease_gen = sup.acquire_lease("conn-m1-hold").await.unwrap();
        sup.set_torque(true).await.unwrap();
        sup.tick(0.02).await;
        let epoch = sup.get_observation().await.epoch_id;

        let action_id = {
            let mut inner = sup.inner().write().await;
            { let s = inner.current_admission_stamp(); inner.begin_policy_run(s, PolicyRunStart {
                    epoch_id: epoch,
                    connection_id: "conn-m1-hold".into(),
                    lease_generation: lease_gen,
                    policy_id: "vid_m1_hold".into(),
                    artifact_digest: digest.clone(),
                    backend: "mlp".into(),
                    mode: PolicyMode::Live,
                    sim_only: false,
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
                    i_am_at_the_arm: true,
                    verified_release_gate: Some(gate),
                }) }
                .unwrap()
        };

        writes.store(0, Ordering::SeqCst);
        non_home_writes.store(0, Ordering::SeqCst);

        let runner = spawn_policy_runner(
            sup.clone(),
            action_id,
            prepared,
            20.0,
            compute_policy_deadline(Some(0.35)).unwrap(),
            None,
            None,
        );

        tokio::time::timeout(Duration::from_secs(5), infer_entered.notified())
            .await
            .expect("timed out waiting for BarrierBackend::infer");

        // Advance past frame 1 deadline (1_000_048 -> 1_000_049) while infer is running
        hub.set_test_now_ms(Some(1_000_049));

        {
            let (lock, cvar) = &*allow_finish;
            *lock.lock().unwrap() = true;
            cvar.notify_all();
        }

        tokio::time::sleep(Duration::from_millis(80)).await;
        sup.tick(0.02).await;
        let mid_obs = sup.get_observation().await;
        let mid_pol = mid_obs.policy.expect("policy still active in hold");
        assert!(mid_pol.active, "on_stale=hold must keep policy active on expiry");
        assert_eq!(mid_pol.steps, 0, "expired step target must be discarded");
        assert_eq!(non_home_writes.load(Ordering::SeqCst), 0);

        // Ingest fresh frame 2 -> policy resumes and commits targets
        hub.ingest_embedding(EmbeddingIngestRequest {
            handshake_generation: hs.handshake_generation,
            producer_epoch: 1,
            frame_id: 2,
            captured_at_ms: 1_000_049,
            oldest_captured_at_ms: Some(1_000_049),
            embedding: vec![1.0, 1.0],
            encoder_artifact_sha256: sha_a(),
            preprocess_config_sha256: sha_b(),
            model_generation: Some(1),
        })
        .unwrap();

        for _ in 0..8 {
            tokio::time::sleep(Duration::from_millis(45)).await;
            sup.tick(0.02).await;
        }
        tokio::time::timeout(Duration::from_secs(5), runner)
            .await
            .expect("runner timeout")
            .expect("runner panic");

        let end_obs = sup.get_observation().await;
        let end_pol = end_obs.policy.expect("policy obs");
        assert_eq!(end_pol.stop_reason.as_deref(), Some("MAX_SECONDS"));
        assert!(end_pol.steps >= 1);
        assert_eq!(end_pol.vision_frame_id, Some(2));
        assert!(non_home_writes.load(Ordering::SeqCst) >= 1);
    }

    // Part 2: lease loss with on_stale = "hold" STILL terminates immediately with LEASE_LOST
    {
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
        let prepared =
            PreparedPolicy::from_backend(snapshot, contract, barrier_backend, &cfg).unwrap();

        let writes = Arc::new(AtomicUsize::new(0));
        let non_home_writes = Arc::new(AtomicUsize::new(0));
        let drv = CountingRealDriver::new(vec![0.0; 6], writes.clone(), non_home_writes.clone());
        let sup = Arc::new(ArmSupervisor::new(cfg.clone()).with_driver(Box::new(drv)));
        sup.set_follower("so-arm101").await;

        let hub = embedding_hub_for_supervisor(&sup);
        hub.set_test_now_ms(Some(1_000_000));
        let hs = hub_two_step_handshake(&hub, 1, 1_000_000, 1, true);
        hub.ingest_embedding(EmbeddingIngestRequest {
            handshake_generation: hs.handshake_generation,
            producer_epoch: 1,
            frame_id: 1,
            captured_at_ms: 1_000_000,
            oldest_captured_at_ms: Some(1_000_000),
            embedding: vec![1.0, 1.0],
            encoder_artifact_sha256: sha_a(),
            preprocess_config_sha256: sha_b(),
            model_generation: Some(1),
        })
        .unwrap();

        let raw_gate = make_valid_release_gate(&digest, &sha_a(), &sha_b(), 20);
        let gate = ReleaseGate::parse_and_verify(
            &raw_gate,
            &digest,
            &cfg,
            prepared.contract(),
            20.0,
            &ActiveDeploymentContext {
                scene_id: "pcb_suction".into(),
                tool_id: "suction".into(),
                calibration_digest: EMPTY_CALIBRATION_DIGEST.into(),
            },
        )
        .unwrap();

        let lease_gen = sup.acquire_lease("conn-m1-lease").await.unwrap();
        sup.set_torque(true).await.unwrap();
        sup.tick(0.02).await;
        let epoch = sup.get_observation().await.epoch_id;

        let action_id = {
            let mut inner = sup.inner().write().await;
            { let s = inner.current_admission_stamp(); inner.begin_policy_run(s, PolicyRunStart {
                    epoch_id: epoch,
                    connection_id: "conn-m1-lease".into(),
                    lease_generation: lease_gen,
                    policy_id: "vid_m1_lease".into(),
                    artifact_digest: digest,
                    backend: "mlp".into(),
                    mode: PolicyMode::Live,
                    sim_only: false,
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
                    i_am_at_the_arm: true,
                    verified_release_gate: Some(gate),
                }) }
                .unwrap()
        };

        writes.store(0, Ordering::SeqCst);
        non_home_writes.store(0, Ordering::SeqCst);

        let runner = spawn_policy_runner(
            sup.clone(),
            action_id,
            prepared,
            20.0,
            compute_policy_deadline(Some(2.0)).unwrap(),
            None,
            None,
        );

        tokio::time::timeout(Duration::from_secs(5), infer_entered.notified())
            .await
            .expect("timed out waiting for BarrierBackend::infer");

        let _ = sup.preempt_lease("conn-hijack", "hijack").await.unwrap();

        {
            let (lock, cvar) = &*allow_finish;
            *lock.lock().unwrap() = true;
            cvar.notify_all();
        }
        tokio::time::timeout(Duration::from_secs(5), runner)
            .await
            .expect("runner timeout")
            .expect("runner panic");

        let obs = sup.get_observation().await;
        let pol = obs.policy.expect("policy obs");
        assert_eq!(pol.stop_reason.as_deref(), Some("LEASE_LOST"));
        assert_eq!(pol.steps, 0);
        assert_eq!(writes.load(Ordering::SeqCst), 0);
        assert_eq!(non_home_writes.load(Ordering::SeqCst), 0);
    }
}

#[tokio::test]
async fn b02_missing_oldest_captured_at_ms_rejected_400_without_mutation_or_target() {
    let mut weights = vec![0.0f32; 6 * 8];
    weights[6] = 0.1;
    weights[7] = 0.1;
    let layers = make_linear_layers(8, 6, weights);
    let dir = tempdir().unwrap();
    let digest = write_rmlp_pack(
        dir.path(),
        &layers,
        serde_json::json!({
            "fps": 20.0,
            "vision": {
                "source": "video_embedding",
                "layout": "embedding_v1",
                "embedding_dim": 2,
                "history_len": 1,
                "max_staleness_ms": 200,
                "max_window_span_ms": 300,
                "fps_staleness_budget_ms": 200,
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
    let addr = spawn_http_server(sup.clone(), OperatorAuth::disabled()).await;
    let client = reqwest::Client::new();

    let hs_resp = http_two_step_handshake(&client, addr, None, None, 1, 500_000, 1, None, None).await;
    assert_eq!(hs_resp.status(), reqwest::StatusCode::OK);
    let hs_gen = hs_resp.json::<serde_json::Value>().await.unwrap()["handshake_generation"]
        .as_u64()
        .unwrap();

    // 1. Omitting oldest_captured_at_ms on an empty ring returns 400 MISSING_OLDEST_TIMESTAMP
    // and never fills the ring or allows a policy runner to apply a target.
    let missing_empty = client
        .post(format!("http://{addr}/api/v1/vision/embedding"))
        .json(&serde_json::json!({
            "handshake_generation": hs_gen,
            "producer_epoch": 1,
            "frame_id": 1,
            "captured_at_ms": 500_000,
            "embedding": [1.0, 2.0],
            "encoder_artifact_sha256": sha_a(),
            "preprocess_config_sha256": sha_b()
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(missing_empty.status(), reqwest::StatusCode::BAD_REQUEST);
    let missing_json: serde_json::Value = missing_empty.json().await.unwrap();
    assert_eq!(missing_json["code"], "MISSING_OLDEST_TIMESTAMP");
    assert_eq!(hub.window_len(), 0);
    assert!(hub.frame_ids().is_empty());
    assert!(hub.window_deadlines().is_none());

    let raw_gate = make_valid_release_gate(&digest, &sha_a(), &sha_b(), 20);
    let gate = ReleaseGate::parse_and_verify(
        &raw_gate,
        &digest,
        &cfg,
        prepared.contract(),
        20.0,
        &ActiveDeploymentContext {
            scene_id: "pcb_suction".into(),
            tool_id: "suction".into(),
            calibration_digest: EMPTY_CALIBRATION_DIGEST.into(),
        },
    )
    .unwrap();
    let lease_gen = sup.acquire_lease("conn-b02").await.unwrap();
    sup.set_torque(true).await.unwrap();
    sup.tick(0.02).await;
    let epoch = sup.get_observation().await.epoch_id;
    let action_id = {
        let mut inner = sup.inner().write().await;
        { let s = inner.current_admission_stamp(); inner.begin_policy_run(s, PolicyRunStart {
                epoch_id: epoch,
                connection_id: "conn-b02".into(),
                lease_generation: lease_gen,
                policy_id: "vid_b02".into(),
                artifact_digest: digest,
                backend: "mlp".into(),
                mode: PolicyMode::Live,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: true,
                verified_release_gate: Some(gate),
            }) }
            .unwrap()
    };

    let baseline_motion_frame = sup.inner().read().await.last_motion_frame;
    writes.store(0, Ordering::SeqCst);
    non_home_writes.store(0, Ordering::SeqCst);
    let runner = spawn_policy_runner(
        sup.clone(),
        action_id,
        prepared,
        20.0,
        compute_policy_deadline(Some(0.08)).unwrap(),
        None,
        None,
    );
    tokio::time::timeout(Duration::from_secs(5), runner)
        .await
        .expect("runner timeout")
        .expect("runner panic");
    let obs = sup.get_observation().await;
    let pol = obs.policy.expect("policy obs");
    assert_eq!(pol.steps, 0);
    assert!(pol.last_shadow_target.is_none());
    assert_eq!(sup.inner().read().await.last_motion_frame, baseline_motion_frame);
    assert_eq!(writes.load(Ordering::SeqCst), 0);
    assert_eq!(non_home_writes.load(Ordering::SeqCst), 0);

    // 2. With an existing frame in the ring, a subsequent request omitting oldest_captured_at_ms
    // is rejected with 400 MISSING_OLDEST_TIMESTAMP and leaves ring contents and deadlines unchanged.
    hub.bind_policy(
        10,
        VideoEmbeddingSpec {
            embedding_dim: 2,
            history_len: 2,
            max_staleness_ms: 200,
            max_window_span_ms: 300,
            fps_staleness_budget_ms: Some(200),
            encoder_artifact_sha256: sha_a(),
            preprocess_config_sha256: sha_b(),
            embedding_norm: None,
            on_stale: armd::policy_runtime::VisionStalePolicy::Stop,
        },
        20.0,
    );
    let valid_first = client
        .post(format!("http://{addr}/api/v1/vision/embedding"))
        .json(&serde_json::json!({
            "handshake_generation": hs_gen,
            "producer_epoch": 1,
            "frame_id": 2,
            "captured_at_ms": 500_000,
            "oldest_captured_at_ms": 499_980,
            "embedding": [1.0, 2.0],
            "encoder_artifact_sha256": sha_a(),
            "preprocess_config_sha256": sha_b()
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(valid_first.status(), reqwest::StatusCode::OK);
    let frames_before = hub.frame_ids();
    let deadlines_before = hub.window_deadlines().unwrap();

    hub.set_test_now_ms(Some(500_020));
    let err_second = hub
        .ingest_embedding(EmbeddingIngestRequest {
            handshake_generation: hs_gen,
            producer_epoch: 1,
            frame_id: 3,
            captured_at_ms: 500_020,
            oldest_captured_at_ms: None,
            embedding: vec![3.0, 4.0],
            encoder_artifact_sha256: sha_a(),
            preprocess_config_sha256: sha_b(),
            model_generation: None,
        })
        .unwrap_err();
    assert_eq!(err_second.status, 400);
    assert_eq!(err_second.code, "MISSING_OLDEST_TIMESTAMP");
    assert_eq!(hub.frame_ids(), frames_before);
    assert_eq!(hub.window_deadlines().unwrap(), deadlines_before);
}

#[tokio::test]
async fn m02_unbroken_renewal_commit_past_old_ticket_expiry_holds_and_resumes() {
    let mut weights = vec![0.0f32; 6 * 8];
    weights[6] = 0.05;
    weights[7] = 0.05;
    let layers = make_linear_layers(8, 6, weights);
    let dir = tempdir().unwrap();
    let digest = write_rmlp_pack(
        dir.path(),
        &layers,
        serde_json::json!({
            "fps": 20.0,
            "vision": {
                "source": "video_embedding",
                "layout": "embedding_v1",
                "embedding_dim": 2,
                "history_len": 1,
                "max_staleness_ms": 500,
                "max_window_span_ms": 500,
                "fps_staleness_budget_ms": 500,
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
    hub.set_default_handshake_ttl_ms(100);
    hub.set_test_now_ms(Some(1_000_000));
    let hs1 = hub_two_step_handshake(&hub, 1, 1_000_000, 1, true);
    assert_eq!(hs1.handshake_expires_at_ms, 1_000_100);

    hub.ingest_embedding(EmbeddingIngestRequest {
        handshake_generation: hs1.handshake_generation,
        producer_epoch: 1,
        frame_id: 1,
        captured_at_ms: 1_000_000,
        oldest_captured_at_ms: Some(1_000_000),
        embedding: vec![1.0, 1.0],
        encoder_artifact_sha256: sha_a(),
        preprocess_config_sha256: sha_b(),
        model_generation: Some(1),
    })
    .unwrap();

    let raw_gate = make_valid_release_gate(&digest, &sha_a(), &sha_b(), 20);
    let gate = ReleaseGate::parse_and_verify(
        &raw_gate,
        &digest,
        &cfg,
        prepared.contract(),
        20.0,
        &ActiveDeploymentContext {
            scene_id: "pcb_suction".into(),
            tool_id: "suction".into(),
            calibration_digest: EMPTY_CALIBRATION_DIGEST.into(),
        },
    )
    .unwrap();

    let lease_gen = sup.acquire_lease("conn-m02").await.unwrap();
    sup.set_torque(true).await.unwrap();
    sup.tick(0.02).await;
    let epoch = sup.get_observation().await.epoch_id;
    let action_id = {
        let mut inner = sup.inner().write().await;
        { let s = inner.current_admission_stamp(); inner.begin_policy_run(s, PolicyRunStart {
                epoch_id: epoch,
                connection_id: "conn-m02".into(),
                lease_generation: lease_gen,
                policy_id: "vid_m02".into(),
                artifact_digest: digest,
                backend: "mlp".into(),
                mode: PolicyMode::Live,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: true,
                verified_release_gate: Some(gate),
            }) }
            .unwrap()
    };

    writes.store(0, Ordering::SeqCst);
    non_home_writes.store(0, Ordering::SeqCst);
    let runner = spawn_policy_runner(
        sup.clone(),
        action_id,
        prepared,
        20.0,
        compute_policy_deadline(Some(0.45)).unwrap(),
        None,
        None,
    );

    tokio::time::timeout(Duration::from_secs(5), infer_entered.notified())
        .await
        .expect("timed out waiting for BarrierBackend::infer");

    // On-time renewal at t = 1_000_090 (before hs1 expiry 1_000_100), extending to 1_000_290.
    hub.set_default_handshake_ttl_ms(200);
    hub.set_test_now_ms(Some(1_000_090));
    let hs2 = hub_two_step_handshake(&hub, 1, 1_000_090, 1, true);
    assert!(hs2.handshake_generation > hs1.handshake_generation);
    assert_eq!(hs2.handshake_expires_at_ms, 1_000_290);

    // Advance commit clock past old ticket expiry (1_000_120 > 1_000_100) while new hs2 is valid (<= 1_000_290).
    hub.set_test_now_ms(Some(1_000_120));

    {
        let (lock, cvar) = &*allow_finish;
        *lock.lock().unwrap() = true;
        cvar.notify_all();
    }

    tokio::time::sleep(Duration::from_millis(80)).await;
    sup.tick(0.02).await;
    let mid_obs = sup.get_observation().await;
    let mid_pol = mid_obs.policy.expect("policy still running in hold");
    assert!(
        mid_pol.active,
        "policy must hold cleanly across unbroken renewal even past old ticket expiry"
    );
    assert_eq!(mid_pol.steps, 0, "in-flight target from old ticket must be discarded");
    assert_eq!(non_home_writes.load(Ordering::SeqCst), 0);

    // Refill under hs2 and let the runner commit
    hub.ingest_embedding(EmbeddingIngestRequest {
        handshake_generation: hs2.handshake_generation,
        producer_epoch: 1,
        frame_id: 2,
        captured_at_ms: 1_000_120,
        oldest_captured_at_ms: Some(1_000_120),
        embedding: vec![1.0, 1.0],
        encoder_artifact_sha256: sha_a(),
        preprocess_config_sha256: sha_b(),
        model_generation: Some(1),
    })
    .unwrap();

    for _ in 0..10 {
        tokio::time::sleep(Duration::from_millis(45)).await;
        sup.tick(0.02).await;
    }
    tokio::time::timeout(Duration::from_secs(5), runner)
        .await
        .expect("runner timeout")
        .expect("runner panic");

    let end_obs = sup.get_observation().await;
    let end_pol = end_obs.policy.expect("policy obs");
    assert_eq!(end_pol.stop_reason.as_deref(), Some("MAX_SECONDS"));
    assert!(end_pol.steps >= 1);
    assert_eq!(end_pol.vision_frame_id, Some(2));
    assert!(non_home_writes.load(Ordering::SeqCst) >= 1);
}

#[tokio::test]
async fn r2_late_handshake_renewal_between_ticks_stops_with_handshake_expired() {
    let mut weights = vec![0.0f32; 6 * 8];
    weights[6] = 0.05;
    weights[7] = 0.05;
    let layers = make_linear_layers(8, 6, weights);
    let dir = tempdir().unwrap();
    let digest = write_rmlp_pack(
        dir.path(),
        &layers,
        serde_json::json!({
            "fps": 5.0,
            "vision": {
                "source": "video_embedding",
                "layout": "embedding_v1",
                "embedding_dim": 2,
                "history_len": 1,
                "max_staleness_ms": 500,
                "max_window_span_ms": 500,
                "fps_staleness_budget_ms": 500,
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
    hub.set_default_handshake_ttl_ms(100);
    hub.set_test_now_ms(Some(1_000_000));
    let hs1 = hub_two_step_handshake(&hub, 1, 1_000_000, 1, true);
    let initial_continuity_epoch = hub.live_ticket_state().handshake_continuity_epoch;
    assert_eq!(initial_continuity_epoch, 1);

    hub.ingest_embedding(EmbeddingIngestRequest {
        handshake_generation: hs1.handshake_generation,
        producer_epoch: 1,
        frame_id: 1,
        captured_at_ms: 1_000_000,
        oldest_captured_at_ms: Some(1_000_000),
        embedding: vec![1.0, 1.0],
        encoder_artifact_sha256: sha_a(),
        preprocess_config_sha256: sha_b(),
        model_generation: Some(1),
    })
    .unwrap();

    let raw_gate = make_valid_release_gate(&digest, &sha_a(), &sha_b(), 5);
    let gate = ReleaseGate::parse_and_verify(
        &raw_gate,
        &digest,
        &cfg,
        prepared.contract(),
        5.0,
        &ActiveDeploymentContext {
            scene_id: "pcb_suction".into(),
            tool_id: "suction".into(),
            calibration_digest: EMPTY_CALIBRATION_DIGEST.into(),
        },
    )
    .unwrap();

    let lease_gen = sup.acquire_lease("conn-r2-8").await.unwrap();
    sup.set_torque(true).await.unwrap();
    sup.tick(0.02).await;
    let epoch = sup.get_observation().await.epoch_id;
    let action_id = {
        let mut inner = sup.inner().write().await;
        { let s = inner.current_admission_stamp(); inner.begin_policy_run(s, PolicyRunStart {
                epoch_id: epoch,
                connection_id: "conn-r2-8".into(),
                lease_generation: lease_gen,
                policy_id: "vid_r2_8".into(),
                artifact_digest: digest,
                backend: "mlp".into(),
                mode: PolicyMode::Live,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: true,
                verified_release_gate: Some(gate),
            }) }
            .unwrap()
    };

    writes.store(0, Ordering::SeqCst);
    non_home_writes.store(0, Ordering::SeqCst);
    let runner = spawn_policy_runner(
        sup.clone(),
        action_id,
        prepared,
        5.0,
        compute_policy_deadline(Some(1.0)).unwrap(),
        None,
        None,
    );

    // Wait for step 1 to commit cleanly.
    let deadline = tokio::time::Instant::now() + Duration::from_secs(2);
    loop {
        sup.tick(0.02).await;
        if let Some(pol) = sup.get_observation().await.policy {
            if pol.steps >= 1 {
                break;
            }
        }
        assert!(
            tokio::time::Instant::now() < deadline,
            "timed out waiting for policy step 1 to commit"
        );
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    let writes_after_step1 = non_home_writes.load(Ordering::SeqCst);
    assert_eq!(writes_after_step1, 1);

    // Between tick 1 and tick 2: hs1 expires (1_000_120 > 1_000_100), producer completes
    // a late handshake (hs2) and refills the ring buffer BEFORE tick 2 runs.
    hub.set_test_now_ms(Some(1_000_120));
    let hs2 = hub_two_step_handshake(&hub, 1, 1_000_120, 1, true);
    assert!(hs2.handshake_generation > hs1.handshake_generation);
    assert_eq!(
        hub.live_ticket_state().handshake_continuity_epoch,
        initial_continuity_epoch + 1,
        "late handshake after expiry must advance handshake_continuity_epoch"
    );
    hub.ingest_embedding(EmbeddingIngestRequest {
        handshake_generation: hs2.handshake_generation,
        producer_epoch: 1,
        frame_id: 2,
        captured_at_ms: 1_000_120,
        oldest_captured_at_ms: Some(1_000_120),
        embedding: vec![2.0, 2.0],
        encoder_artifact_sha256: sha_a(),
        preprocess_config_sha256: sha_b(),
        model_generation: Some(1),
    })
    .unwrap();

    // Tick 2 must immediately stop fail-closed with HANDSHAKE_EXPIRED and emit 0 additional targets.
    tokio::time::timeout(Duration::from_secs(5), runner)
        .await
        .expect("runner timeout")
        .expect("runner panic");

    let end_obs = sup.get_observation().await;
    let end_pol = end_obs.policy.expect("policy obs");
    assert!(!end_pol.active);
    assert_eq!(end_pol.stop_reason.as_deref(), Some("HANDSHAKE_EXPIRED"));
    assert_eq!(end_pol.steps, 1);
    assert_eq!(non_home_writes.load(Ordering::SeqCst), writes_after_step1);
}
