//! Deterministic seeded state and local mock detector server for WS replay sessions.

use std::collections::HashMap;
use std::net::SocketAddr;
use std::path::PathBuf;

use axum::{routing::get, Json, Router};
use serde_json::json;
use tempfile::TempDir;

use super::comparator::SessionSetup;

/// Fixed seeded ULID for deterministic episode replay sessions.
pub const SEEDED_EPISODE_ID: &str = "01J00000000000000000000001";
/// Fixed seeded ULID for deterministic pose/sequence sessions.
pub const SEEDED_POSE_ID: &str = "01J00000000000000000000002";
/// Fixed seeded policy pack ID for shadow policy sessions.
pub const SEEDED_POLICY_ID: &str = "shadow_demo";

/// Ports that replay tests must never bind (explicitly excluding 18765/18001/18990).
pub const FORBIDDEN_PORTS: &[u16] = &[18765, 18001, 18990];

/// Bind an OS-assigned ephemeral loopback listener (`127.0.0.1:0`) without bind-drop-rebind races.
async fn bind_ephemeral_listener() -> tokio::net::TcpListener {
    let mut held = Vec::new();
    for _ in 0..16 {
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0")
            .await
            .expect("bind ephemeral loopback port");
        let port = listener.local_addr().unwrap().port();
        if !FORBIDDEN_PORTS.contains(&port) {
            return listener;
        }
        held.push(listener);
    }
    panic!("Unable to bind non-forbidden ephemeral loopback port");
}

pub struct MaterializedSeed {
    pub _dir: TempDir,
    pub profile_path: PathBuf,
    pub episodes_dir: PathBuf,
    pub env: HashMap<String, String>,
    pub detector_handle: Option<tokio::task::JoinHandle<()>>,
}

impl Drop for MaterializedSeed {
    fn drop(&mut self) {
        if let Some(h) = self.detector_handle.take() {
            h.abort();
        }
    }
}

async fn spawn_mock_detector() -> (SocketAddr, tokio::task::JoinHandle<()>) {
    let listener = bind_ephemeral_listener().await;
    let addr = listener.local_addr().unwrap();
    let app = Router::new()
        .route(
            "/api/mode",
            get(|| async {
                Json(json!({
                    "mode": "detection",
                    "models": ["yolo", "grasp"],
                    "capabilities": ["bbox", "mask", "depth"],
                    "active_model": "yolo"
                }))
            }),
        )
        .route(
            "/api/detection/latest",
            get(|| async {
                Json(json!({
                    "timestamp": 1700000000.0,
                    "frame_id": 1,
                    "detections": [
                        {
                            "label": "cube",
                            "confidence": 0.95,
                            "bbox": [300, 220, 340, 260],
                            "center_px": [320.0, 240.0]
                        }
                    ]
                }))
            }),
        )
        .route(
            "/api/depth",
            get(|| async {
                Json(json!({
                    "available": false,
                    "error": "mock_depth_unavailable"
                }))
            }),
        );
    let handle = tokio::spawn(async move {
        let _ = axum::serve(listener, app).await;
    });
    (addr, handle)
}

fn build_profile(name: &str) -> armd::ArmConfig {
    match name {
        "rebot_dm_short_lease" => armd::ArmConfig {
            lease_timeout_ms: 400,
            ..armd::ArmConfig::rebot_dm()
        },
        "default_6dof" => armd::ArmConfig {
            lease_timeout_ms: 15_000,
            ..armd::ArmConfig::default()
        },
        _ => armd::ArmConfig {
            lease_timeout_ms: 15_000,
            ..armd::ArmConfig::rebot_dm()
        },
    }
}

/// Create a fresh isolated temp state directory with deterministic seeded files and env vars.
pub async fn materialize_seed(setup: &SessionSetup) -> MaterializedSeed {
    let dir = tempfile::tempdir().expect("tempdir for WS replay seed");
    let root = dir.path();

    let cfg = build_profile(&setup.profile);
    let profile_path = root.join("profile.json");
    std::fs::write(&profile_path, serde_json::to_vec_pretty(&cfg).unwrap()).unwrap();

    let episodes_dir = root.join("episodes");
    std::fs::create_dir_all(&episodes_dir).unwrap();

    let sandbox_dir = root.join("sandbox");
    std::fs::create_dir_all(&sandbox_dir).unwrap();

    let calib_backups = root.join("calib_backups");
    let calib_sessions = root.join("calib_sessions");
    std::fs::create_dir_all(&calib_backups).unwrap();
    std::fs::create_dir_all(&calib_sessions).unwrap();

    let keepout_path = root.join("keepout.json");
    let collision_path = root.join("collision.json");
    let gripper_calib_path = root.join("gripper_calib.json");
    let poses_path = root.join("poses.json");
    let sequences_path = root.join("sequences.json");
    let grasp_setup_path = root.join("grasp_setup.json");
    let calibration_path = root.join("calibration.json");
    let commissioning_path = root.join("commissioning.json");

    if setup.seed_poses {
        let pose_positions = vec![0.0f32, -1.167, -1.565, 1.435, 0.0, 0.0, 0.0];
        let pose_list = armd::poses::PoseList {
            profile_id: cfg.profile_id.clone(),
            arm_dof: Some(cfg.arm_dof()),
            gripper_joint: cfg.gripper_index(),
            joint_names: Some(cfg.limits.iter().map(|l| l.name.clone()).collect()),
            poses: vec![armd::poses::NamedPose {
                id: SEEDED_POSE_ID.to_string(),
                name: "ready".to_string(),
                positions: pose_positions,
                tcp: None,
                saved_at_ms: 1_700_000_000_000,
                profile_id: cfg.profile_id.clone(),
                arm_dof: Some(cfg.arm_dof()),
                gripper_joint: cfg.gripper_index(),
                joint_names: Some(cfg.limits.iter().map(|l| l.name.clone()).collect()),
                tool_id: Some(cfg.active_tool.id.clone()),
                tool_config_digest: Some(cfg.active_tool.config_digest.clone()),
                stale: None,
            }],
        };
        std::fs::write(&poses_path, serde_json::to_vec_pretty(&pose_list).unwrap()).unwrap();
    }

    if setup.seed_episode {
        let home = cfg.home_positions.clone();
        let mut frames = Vec::new();
        for i in 0..10 {
            let mut q = home.clone();
            if !q.is_empty() {
                q[0] += 0.02 * (i as f32 / 9.0);
            }
            frames.push(armd::RecordFrame {
                timestamp: i as f64 * 0.02,
                observation_state: q.clone(),
                action: q,
            });
        }
        let pq_path = episodes_dir.join(format!("{SEEDED_EPISODE_ID}.parquet"));
        armd::write_frames_to_parquet(&pq_path, &frames).unwrap();
        let jz = armd::recording::store::JointZeroStamp::of(&vec![0.0f32; cfg.dof]);
        let sidecar = json!({
            "id": SEEDED_EPISODE_ID,
            "dataset": "default",
            "task": "seeded replay",
            "source": "sdk",
            "outcome": "ok",
            "recorded_at": "2026-01-01T00:00:00Z",
            "duration_s": 0.18,
            "frames": 10,
            "joint_names": cfg.limits.iter().map(|l| l.name.clone()).collect::<Vec<_>>(),
            "units": "rad",
            "joint_zero": jz,
            "profile_id": cfg.profile_id,
            "armd_version": env!("CARGO_PKG_VERSION"),
            "fps_nominal": f64::from(cfg.control_frequency_hz),
            "arm_dof": cfg.arm_dof(),
            "gripper_joint": cfg.gripper_index(),
            "tool_id": cfg.active_tool.id,
            "tool_config_digest": cfg.active_tool.config_digest,
            "truncated": false
        });
        std::fs::write(
            episodes_dir.join(format!("{SEEDED_EPISODE_ID}.json")),
            serde_json::to_vec_pretty(&sidecar).unwrap(),
        )
        .unwrap();
    }

    #[cfg(feature = "policy-runtime")]
    if setup.seed_policy {
        let policy_dir = sandbox_dir.join("policies").join(SEEDED_POLICY_ID);
        std::fs::create_dir_all(&policy_dir).unwrap();
        let dof = cfg.dof;
        let joint_names: Vec<String> = cfg.limits.iter().map(|l| l.name.clone()).collect();
        // Single identity-like linear layer mapping dof -> dof with zero delta.
        let mut weights = vec![0.0f32; dof * dof];
        for i in 0..dof {
            weights[i * dof + i] = 1.0;
        }
        let layers = vec![armd::policy_runtime::MlpLayerSpec::new(
            dof,
            dof,
            armd::policy_runtime::Activation::None,
            weights,
            vec![0.0f32; dof],
        )];
        let rmlp = armd::policy_runtime::encode_rmlp_bytes(&layers).unwrap();
        let manifest = json!({
            "format": "rebot-onnx-policy/v1",
            "name": "shadow_demo",
            "unit_mode": "rad",
            "action_dim": dof,
            "action_space": "joint_positions",
            "joint_names": joint_names,
            "model_file": "model.rmlp",
            "fps": 20.0
        });
        std::fs::write(
            policy_dir.join("policy.json"),
            serde_json::to_vec_pretty(&manifest).unwrap(),
        )
        .unwrap();
        std::fs::write(policy_dir.join("model.rmlp"), &rmlp).unwrap();
    }

    if setup.seed_calibration {
        let calib = json!({
            "version": 1,
            "intrinsics": {
                "fx": 600.0,
                "fy": 600.0,
                "cx": 320.0,
                "cy": 240.0,
                "width": 640,
                "height": 480
            },
            "camera_mount": {
                "offset_m": [0.0, 0.0, 0.05],
                "q_end_cam": [0.0, 0.0, 0.0, 1.0]
            },
            "table_z_m": 0.0
        });
        std::fs::write(&calibration_path, serde_json::to_vec_pretty(&calib).unwrap()).unwrap();
    }

    let mut env = HashMap::new();
    env.insert("ARMD_RT_OWNER".into(), "legacy".into());
    env.insert("ARMD_KEEPOUT".into(), keepout_path.to_string_lossy().into());
    env.insert("ARMD_COLLISION".into(), collision_path.to_string_lossy().into());
    env.insert("ARMD_GRIPPER_CALIB".into(), gripper_calib_path.to_string_lossy().into());
    env.insert("ARMD_POSES".into(), poses_path.to_string_lossy().into());
    env.insert("ARMD_SEQUENCES".into(), sequences_path.to_string_lossy().into());
    env.insert("ARMD_GRASP_SETUP".into(), grasp_setup_path.to_string_lossy().into());
    env.insert("ARMD_CALIBRATION".into(), calibration_path.to_string_lossy().into());
    env.insert("ARMD_CALIBRATION_BACKUPS".into(), calib_backups.to_string_lossy().into());
    env.insert("ARMD_CALIBRATION_SESSIONS".into(), calib_sessions.to_string_lossy().into());
    env.insert("ARMD_TOOL_COMMISSIONING".into(), commissioning_path.to_string_lossy().into());
    env.insert("ARMD_SANDBOX_DIR".into(), sandbox_dir.to_string_lossy().into());

    let detector_handle = if setup.mock_detector {
        let (addr, handle) = spawn_mock_detector().await;
        env.insert("ARMD_DETECTOR_URL".into(), format!("http://{addr}"));
        Some(handle)
    } else {
        // RFC 863 discard port on loopback fails fast with ECONNREFUSED without probing/racing ports.
        env.insert("ARMD_DETECTOR_URL".into(), "http://127.0.0.1:9".into());
        None
    };

    MaterializedSeed {
        _dir: dir,
        profile_path,
        episodes_dir,
        env,
        detector_handle,
    }
}
