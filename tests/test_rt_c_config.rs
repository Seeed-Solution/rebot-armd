//! Stage 3 PR C / A8: owner-side configuration commits, published configuration readers,
//! calibration/motor-zero/gravity-comp routing, and zero-alloc + lock-proof verification
//! across BOTH `RtOwner::Legacy` and `RtOwner::Executor`.
//!
//! Installs [`TimedAlloc`] as this binary's global allocator and enables `cfg.alloc_timing = true`
//! on every executor fixture so `handle.stats().total_allocs() == AllocCounts::ZERO` proves
//! zero heap allocations and zero heap frees on `armd-control` across:
//! - `ReserveConfig`, `PersistedConfig`, `ActivateConfig`, `ReleaseConfig` (all 7 config domains
//!   + `Calibration`),
//! - revision conflicts, authority revocations, rename failures, dir-sync failures, post-rename
//!   activation failures (`Reconciling`), and dropped `ReservationGuard` cleanup,
//! - `SampleTableCapture`, `SetCalibrationBusy`, `CalibrationRevert`, `InvalidateAfterZero`,
//!   and mock `GravityComp` enter/exit.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::auth::OperatorAuth;
use armd::calibration::{
    apply_with_auth, restore_with_auth, ApplyRequest, CalibCommitAuth, CalibrationDeps,
    CalibrationFile,
};
use armd::diagnostics::alloc::{AllocCounts, TimedAlloc};
use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, JointTarget};
use armd::grasp::GraspDeps;
use armd::model::{HolderLabel, JawOpening, JointState, KeepoutBox, MotorZeroSupport};
use armd::motor_zero::{self, MotorZeroRequest};
use armd::persist::PendingWrite;
use armd::poses::{NamedPose, PoseList};
use armd::rt_core::broker::BootId;
use armd::rt_core::config_iface::{ConfigDomain, HoldState};
use armd::rt_core::executor::{ExecutorConfig, RtOwner};
use armd::rt_core::requests::config::{
    build_config_proposal, capture_config_admission, gravity_comp_command,
    prepare_eval_bufs_off_thread, run_config_commit, ConfigAuthContext, ConfigCandidate,
    ConfigCommitPlan, ConfigCommitTestHooks, ConfigReply, ConfigRequest, ConfigWriteOp,
    ReleaseConfig, ReservationGuard, ReserveConfig, ReservePrecheck,
};
use armd::sequences::{Sequence, SequenceFile, Step};
use armd::supervisor::ArmError;
use armd::{ArmConfig, ArmSupervisor};
use futures_util::{SinkExt, StreamExt};
use serde_json::{json, Value};
use tokio::net::TcpListener;
use tokio_tungstenite::tungstenite::Message;

#[global_allocator]
static ALLOC: TimedAlloc = TimedAlloc;

const INSPECT: [f32; 7] = [0.0, 0.3, -0.3, 0.0, 0.0, 0.0, 0.0];

fn make_box(name: &str, c: f32) -> KeepoutBox {
    KeepoutBox {
        name: name.into(),
        min_m: [c, c, c],
        max_m: [c + 0.05, c + 0.05, c + 0.05],
    }
}

fn disk_keepout_names(path: &Path) -> Vec<String> {
    let text = std::fs::read_to_string(path).expect("keepout file should exist");
    let v: Value = serde_json::from_str(&text).expect("valid keepout json");
    v["boxes"]
        .as_array()
        .unwrap()
        .iter()
        .map(|b| b["name"].as_str().unwrap().to_string())
        .collect()
}

fn dir_entries(dir: &Path) -> Vec<String> {
    let mut names: Vec<String> = std::fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();
    names
}

struct Rig {
    sup: Arc<ArmSupervisor>,
    _dir: tempfile::TempDir,
    keepout_path: PathBuf,
    collision_path: PathBuf,
    gripper_calib_path: PathBuf,
    poses_path: PathBuf,
    sequences_path: PathBuf,
    setup_path: PathBuf,
    commissioning_path: PathBuf,
    calib_path: PathBuf,
    lease_conn: String,
    lease_gen: u64,
    closed_conn_id: u64,
    epoch: u64,
}

impl Rig {
    async fn new(owner: RtOwner, configure: impl FnOnce(&mut ArmConfig)) -> Self {
        let dir = tempfile::tempdir().unwrap();
        let keepout_path = dir.path().join("keepout.json");
        let collision_path = dir.path().join("collision.json");
        let gripper_calib_path = dir.path().join("gripper_calib.json");
        let poses_path = dir.path().join("poses.json");
        let sequences_path = dir.path().join("sequences.json");
        let setup_path = dir.path().join("grasp_setup.json");
        let commissioning_path = dir.path().join("tool_commissioning.json");
        let calib_path = dir.path().join("calibration.json");

        let mut cfg = ArmConfig::rebot_dm();
        configure(&mut cfg);
        let sup = Arc::new(ArmSupervisor::new(cfg));

        armd::keepout::load_into(&sup, &keepout_path).await.unwrap();
        armd::collision::load_into(&sup, &collision_path).await;
        armd::gripper_calib::load_into(&sup, &gripper_calib_path).await;
        armd::poses::load_into(&sup, &poses_path).await;
        armd::sequences::load_into(&sup, &sequences_path).await;
        armd::grasp::load_into(&sup, &setup_path).await;
        armd::tool::load_commissioning_into(&sup, &commissioning_path).await;

        let lease_conn = "rt-c-owner-conn".to_string();
        let closed_conn_id = 7777;
        let (lease_gen, epoch) = {
            let mut inner = sup.inner().write().await;
            inner.measured_positions = INSPECT.to_vec();
            inner.commanded_positions = INSPECT.to_vec();
            inner.target_positions = INSPECT.to_vec();
            inner.mark_connection_closed(closed_conn_id);
            let lease_gen = inner
                .acquire_lease(&lease_conn)
                .expect("acquire initial lease");
            (lease_gen, inner.safety.epoch_id())
        };

        if owner == RtOwner::Executor {
            let mut ecfg = ExecutorConfig::for_tests(Duration::from_millis(2));
            ecfg.alloc_timing = true;
            let handle = sup.start_executor(ecfg).await.expect("start RT executor");
            // Wait until at least one tick publishes a snapshot to StateHub.
            let deadline = Instant::now() + Duration::from_secs(3);
            while sup.hub().latest().is_none() {
                assert!(Instant::now() < deadline, "timed out waiting for StateHub snapshot");
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
            assert_eq!(
                handle.stats().total_allocs(),
                AllocCounts::ZERO,
                "executor must start with zero RT allocations"
            );
        }

        Self {
            sup,
            _dir: dir,
            keepout_path,
            collision_path,
            gripper_calib_path,
            poses_path,
            sequences_path,
            setup_path,
            commissioning_path,
            calib_path,
            lease_conn,
            lease_gen,
            closed_conn_id,
            epoch,
        }
    }

    async fn is_config_inhibited(&self) -> bool {
        match self
            .sup
            .dispatch_config_request(ConfigRequest::InspectHold(ConfigDomain::Keepout))
            .await
            .expect("inspect hold")
        {
            ConfigReply::HoldInspected { config_inhibited, .. } => config_inhibited,
            _ => panic!("unexpected InspectHold reply"),
        }
    }

    async fn hold_state(&self, domain: ConfigDomain) -> Option<HoldState> {
        match self
            .sup
            .dispatch_config_request(ConfigRequest::InspectHold(domain))
            .await
            .expect("inspect hold")
        {
            ConfigReply::HoldInspected { state, .. } => state,
            _ => panic!("unexpected InspectHold reply"),
        }
    }

    fn assert_executor_zero_alloc(&self, context: &str) {
        if let Some(h) = self.sup.executor_handle() {
            assert_eq!(
                h.stats().total_allocs(),
                AllocCounts::ZERO,
                "armd-control allocated during {context}: {:?}",
                h.stats().total_allocs()
            );
            assert_eq!(
                self.sup.inner().control_thread_attempts(),
                0,
                "armd-control attempted SupervisorLock during {context}"
            );
        }
    }

    async fn shutdown(self) {
        if let Some(h) = self.sup.executor_handle() {
            self.assert_executor_zero_alloc("before shutdown");
            tokio::task::spawn_blocking(move || h.stop_and_join())
                .await
                .expect("join executor thread");
        }
    }
}

fn keepout_plan(
    rig: &Rig,
    pub_cfg: &armd::rt_core::requests::config::PublishedConfig,
    boxes: Vec<KeepoutBox>,
    expected_revision: u64,
    conn_id: u64,
    lease_gen: u64,
    _refresh_window: bool,
) -> (
    ConfigCommitPlan,
    Vec<KeepoutBox>,
    Vec<KeepoutBox>,
    Vec<KeepoutBox>,
) {
    let profile_boxes = pub_cfg.profile_keepout();
    let effective_boxes: Vec<KeepoutBox> = profile_boxes.iter().chain(&boxes).cloned().collect();
    let (ws_pol, cam) = pub_cfg.workspace_policy().expect("workspace policy");
    let mut next_ws = ws_pol.clone();
    next_ws.keepout_boxes = effective_boxes.clone();
    let ws_eval_bufs = prepare_eval_bufs_off_thread(
        &pub_cfg.config.arm_model,
        &next_ws,
        cam,
        pub_cfg.config.active_tool.tcp_offset_m,
    );
    let admission = capture_config_admission(&rig.sup, conn_id, lease_gen, Some(rig.epoch));
    let proposal = build_config_proposal(
        &rig.sup,
        ConfigDomain::Keepout,
        expected_revision,
        admission,
        lease_gen,
        pub_cfg.workspace_revision,
    );
    let write = ConfigWriteOp::from(Some(PendingWrite::new(
        rig.keepout_path.clone(),
        serde_json::to_string_pretty(&armd::keepout::KeepoutList {
            boxes: boxes.clone(),
            expected_revision: None,
        })
        .unwrap(),
    )));
    let plan = ConfigCommitPlan {
        proposal,
        auth: ConfigAuthContext {
            admission,
            lease_holder: Some((HolderLabel::new(&rig.lease_conn), lease_gen)),
        },
        precheck: ReservePrecheck::Keepout,
        candidate: ConfigCandidate::Keepout {
            profile_boxes: profile_boxes.clone(),
            runtime_boxes: boxes.clone(),
            effective_boxes: effective_boxes.clone(),
            ws_eval_bufs,
        },
        write,
        expected_workspace_revision: Some(pub_cfg.workspace_revision),
        expected_poses_revision: None,
    };
    (plan, profile_boxes, boxes, effective_boxes)
}

async fn spawn_http_router(
    rig: &Rig,
    vision_addr: Option<std::net::SocketAddr>,
    auth: OperatorAuth,
) -> (String, tokio::task::JoinHandle<()>) {
    let det_url = vision_addr
        .map(|a| format!("http://{a}"))
        .unwrap_or_else(|| "http://127.0.0.1:1".to_string());
    let det_up = armd::api::proxy::DetectorUpstream::parse(&det_url).unwrap();
    let grasp_deps = GraspDeps::http(det_up.clone(), Some(rig.setup_path.clone()));
    let cal_deps = CalibrationDeps::new(
        rig.calib_path.clone(),
        Some(rig.setup_path.clone()),
        det_up.clone(),
    );
    let router = armd::api::create_router_calibration(
        rig.sup.clone(),
        det_up,
        grasp_deps,
        armd::diagnostics::DiagnosticsState::global(),
        cal_deps,
        auth,
    );
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let handle = tokio::spawn(async move {
        let _ = axum::serve(listener, router).await;
    });
    (format!("http://{addr}"), handle)
}

async fn spawn_mock_vision() -> (
    std::net::SocketAddr,
    Arc<Mutex<(u64, Value)>>,
    tokio::task::JoinHandle<()>,
) {
    let state = Arc::new(Mutex::new((
        1u64,
        json!({
            "fx": 824.0, "fy": 824.0, "cx": 320.0, "cy": 180.0,
            "k1": 0.0, "k2": 0.0, "p1": 0.0, "p2": 0.0, "k3": 0.0,
            "width": 640, "height": 360, "marker_size": 0.05
        }),
    )));
    let s_get = state.clone();
    let s_put = state.clone();
    let app = axum::Router::new().route(
        "/api/calibration/intrinsics",
        axum::routing::get(move || {
            let s_get = s_get.clone();
            async move {
                let st = s_get.lock().unwrap();
                let mut obj = st.1.as_object().unwrap().clone();
                obj.insert("revision".into(), json!(st.0));
                obj.insert("api_version".into(), json!(2));
                axum::Json(Value::Object(obj))
            }
        })
        .put(move |axum::Json(body): axum::Json<Value>| {
            let s_put = s_put.clone();
            async move {
                use axum::response::IntoResponse;
                let mut st = s_put.lock().unwrap();
                if body.get("expected_revision").and_then(Value::as_u64) != Some(st.0) {
                    return (
                        axum::http::StatusCode::CONFLICT,
                        axum::Json(json!({"error": "REVISION_MISMATCH", "revision": st.0})),
                    )
                        .into_response();
                }
                st.0 += 1;
                let obj = st.1.as_object_mut().unwrap();
                for k in [
                    "fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3", "width", "height",
                    "marker_size",
                ] {
                    if let Some(v) = body.get(k) {
                        obj.insert(k.into(), v.clone());
                    }
                }
                let mut out = obj.clone();
                out.insert("revision".into(), json!(st.0));
                out.insert("api_version".into(), json!(2));
                axum::Json(Value::Object(out)).into_response()
            }
        }),
    );
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let jh = tokio::spawn(async move {
        let _ = axum::serve(listener, app).await;
    });
    (addr, state, jh)
}

/// Minimal zero-alloc RT-capable hardware driver for testing `EXECUTOR_CAPABILITY_UNSUPPORTED` refusals.
struct MinimalHwDriver {
    positions: Vec<f32>,
    rx_at: Vec<Option<Instant>>,
    seqs: Vec<u64>,
}

impl MinimalHwDriver {
    fn new(dof: usize) -> Self {
        let now = Instant::now();
        Self {
            positions: vec![0.0; dof],
            rx_at: vec![Some(now); dof],
            seqs: vec![1; dof],
        }
    }
}

impl ArmDriver for MinimalHwDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            torque_feedback: true,
            temperature_feedback: true,
            enable_feedback: true,
            ..DriverCapabilities::default()
        }
    }

    fn supports_rt_executor(&self) -> Result<(), &'static str> {
        Ok(())
    }

    fn motor_zero_support(&self) -> MotorZeroSupport {
        MotorZeroSupport::set_current("motor_flash", false, vec![])
    }

    fn is_feedback_valid(&self) -> bool {
        true
    }

    fn feedback_all_within(&self, dof: usize, _max_age: Duration) -> bool {
        self.rx_at.len() == dof
    }

    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
        out.clear();
        for _ in &self.rx_at {
            out.push(Some(Duration::ZERO));
        }
    }

    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
        out.clear();
        for _ in &self.rx_at {
            out.push(Some(0));
        }
    }

    fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        out.clear();
        out.extend_from_slice(&self.rx_at);
    }

    fn feedback_seqs_into(&self, out: &mut Vec<u64>) {
        out.clear();
        out.extend_from_slice(&self.seqs);
    }

    fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        out.clear();
        out.extend_from_slice(&self.rx_at);
    }

    fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
        let now = Instant::now();
        for t in &mut self.rx_at {
            *t = Some(now);
        }
        out.clear();
        for &p in &self.positions {
            out.push(JointState {
                position: p,
                velocity: 0.0,
                torque: 0.0,
                temperature: 25.0,
            });
        }
        Ok(())
    }

    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let mut out = Vec::with_capacity(self.positions.len());
        self.read_joint_states_into(&mut out)?;
        Ok(out)
    }

    fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        Ok(())
    }

    fn set_torque(&mut self, _enabled: bool) -> Result<(), ArmDriverError> {
        Ok(())
    }
}

// ============================================================================
// 1. All 7 config domains + all 10 REST readers (lock-proof & zero-alloc)
// ============================================================================

async fn run_all_domains_test(owner: RtOwner) {
    let rig = Rig::new(owner, |cfg| {
        cfg.active_tool.id = "custom_finger".into();
        cfg.active_tool.is_stock = false;
        cfg.active_tool.config_digest = "v1:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into();
        cfg.active_tool.commissioning_required = true;
    })
    .await;

    let (base, server_jh) = spawn_http_router(&rig, None, OperatorAuth::disabled()).await;
    let client = reqwest::Client::new();

    // In Executor mode, hold the detached legacy `SupervisorLock` write guard across ALL
    // 7 domain commits AND ALL 10 REST readers + REST commissioning confirm to prove 0 lock use.
    let _detached_guard = if owner == RtOwner::Executor {
        Some(rig.sup.inner().write().await)
    } else {
        None
    };

    // 1. Keepout
    let pub0 = rig.sup.published_config().await.unwrap();
    let (plan, pb, rb, eb) = keepout_plan(&rig, &pub0, vec![make_box("bench", 0.3)], 1, 10, rig.lease_gen, true);
    let out = run_config_commit(
        &rig.sup,
        plan,
        || Ok(()),
        |cfg, act| {
            if let Some(ws) = cfg.config.workspace.as_mut() {
                ws.keepout_boxes = eb;
            }
            cfg.keepout.profile = Some(pb);
            cfg.keepout.runtime_boxes = rb;
            cfg.keepout.runtime = true;
            cfg.keepout.revision = act.revision;
            cfg.workspace_revision = act.workspace_revision;
        },
    )
    .await
    .expect("keepout commit");
    assert_eq!(out.activated.revision, 2);
    assert!(out.persisted && out.durability_confirmed);
    assert_eq!(disk_keepout_names(&rig.keepout_path), ["bench"]);

    let ko: Value = client
        .get(format!("{base}/api/v1/workspace/keepout"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(ko["revision"], 2);
    assert_eq!(ko["source"], "runtime");
    assert_eq!(ko["boxes"].as_array().unwrap().len(), 1);
    assert_eq!(ko["boxes"][0]["name"], "bench");

    // 2. Collision
    let pub1 = rig.sup.published_config().await.unwrap();
    let adm = capture_config_admission(&rig.sup, 10, rig.lease_gen, Some(rig.epoch));
    let prop = build_config_proposal(
        &rig.sup,
        ConfigDomain::Collision,
        pub1.collision_revision,
        adm,
        rig.lease_gen,
        pub1.workspace_revision,
    );
    let out = run_config_commit(
        &rig.sup,
        ConfigCommitPlan {
            proposal: prop,
            auth: ConfigAuthContext {
                admission: adm,
                lease_holder: Some((HolderLabel::new(&rig.lease_conn), rig.lease_gen)),
            },
            precheck: ReservePrecheck::Collision { enabling: false },
            candidate: ConfigCandidate::Collision {
                enabled: Some(false),
                sensitivity: Some(armd::collision::Sensitivity::Low),
            },
            write: ConfigWriteOp::from(Some(PendingWrite::new(
                rig.collision_path.clone(),
                serde_json::to_string_pretty(&armd::collision::CollisionSetting {
                    enabled: false,
                    sensitivity: armd::collision::Sensitivity::Low,
                })
                .unwrap(),
            ))),
            expected_workspace_revision: None,
            expected_poses_revision: None,
        },
        || Ok(()),
        move |cfg, act| {
            cfg.collision_enabled = false;
            cfg.collision_active = false;
            cfg.collision_sensitivity = armd::collision::Sensitivity::Low;
            cfg.collision_runtime = true;
            cfg.collision_revision = act.revision;
        },
    )
    .await
    .expect("collision commit");
    assert_eq!(out.activated.revision, 2);
    let col: Value = client
        .get(format!("{base}/api/v1/safety/collision"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(col["revision"], 2);
    assert_eq!(col["sensitivity"], "low");

    // 3. GripperCalibration
    let pub2 = rig.sup.published_config().await.unwrap();
    let jaw = JawOpening {
        points: vec![[0.0, 0.0], [-5.0, 0.075]],
        calibrated: true,
    };
    let mut next_policy = pub2.config.gripper_policy.clone().unwrap();
    next_policy.jaw_opening = Some(jaw.clone());
    let adm = capture_config_admission(&rig.sup, 10, rig.lease_gen, Some(rig.epoch));
    let prop = build_config_proposal(
        &rig.sup,
        ConfigDomain::GripperCalibration,
        pub2.gripper_calib_revision,
        adm,
        rig.lease_gen,
        pub2.workspace_revision,
    );
    let pol_pub = next_policy.clone();
    let jaw_pub = jaw.clone();
    let out = run_config_commit(
        &rig.sup,
        ConfigCommitPlan {
            proposal: prop,
            auth: ConfigAuthContext {
                admission: adm,
                lease_holder: Some((HolderLabel::new(&rig.lease_conn), rig.lease_gen)),
            },
            precheck: ReservePrecheck::GripperCalibration {
                candidate_policy: Some(next_policy.clone()),
            },
            candidate: ConfigCandidate::GripperCalibration {
                offset_rad: Some(0.01),
                policy: next_policy,
                jaw_opening: Some(jaw),
                clear_stale: true,
            },
            write: ConfigWriteOp::from(Some(PendingWrite::new(
                rig.gripper_calib_path.clone(),
                serde_json::to_string_pretty(&armd::gripper_calib::GripperCalibFile {
                    offset_rad: Some(0.01),
                    jaw_opening: Some(jaw_pub.clone()),
                    ts: Some(1),
                    stale: None,
                    tool_id: Some(pub2.config.active_tool.id.clone()),
                    tool_config_digest: Some(pub2.config.active_tool.config_digest.clone()),
                })
                .unwrap(),
            ))),
            expected_workspace_revision: None,
            expected_poses_revision: None,
        },
        || Ok(()),
        move |cfg, act| {
            cfg.config.gripper_policy = Some(pol_pub);
            cfg.gripper_calib_offset = Some(0.01);
            cfg.gripper_calib_jaw_opening = Some(jaw_pub);
            cfg.gripper_calib_invalid = false;
            cfg.gripper_calib_revision = act.revision;
        },
    )
    .await
    .expect("gripper calib commit");
    assert_eq!(out.activated.revision, 2);
    let gc: Value = client
        .get(format!("{base}/api/v1/gripper/calibration"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(gc["revision"], 2);
    assert_eq!(gc["calibrated"], true);

    // 4. Poses save
    let pub3 = rig.sup.published_config().await.unwrap();
    let pose_id = armd::recording::new_ulid();
    let pose = NamedPose {
        id: pose_id.clone(),
        name: "home_pick".into(),
        positions: INSPECT.to_vec(),
        tcp: pub3.pose_tcp(&INSPECT),
        saved_at_ms: 1000,
        profile_id: pub3.config.profile_id.clone(),
        arm_dof: Some(pub3.config.arm_dof()),
        gripper_joint: pub3.config.gripper_index(),
        joint_names: Some(pub3.joint_names()),
        tool_id: Some(pub3.config.active_tool.id.clone()),
        tool_config_digest: Some(pub3.config.active_tool.config_digest.clone()),
        stale: None,
    };
    let next_list = PoseList {
        profile_id: pub3.config.profile_id.clone(),
        arm_dof: Some(pub3.config.arm_dof()),
        gripper_joint: pub3.config.gripper_index(),
        joint_names: Some(pub3.joint_names()),
        poses: vec![pose],
    };
    let adm = capture_config_admission(&rig.sup, 10, rig.lease_gen, Some(rig.epoch));
    let prop = build_config_proposal(
        &rig.sup,
        ConfigDomain::Poses,
        pub3.poses.revision,
        adm,
        rig.lease_gen,
        pub3.workspace_revision,
    );
    let nl_pub = next_list.clone();
    run_config_commit(
        &rig.sup,
        ConfigCommitPlan {
            proposal: prop,
            auth: ConfigAuthContext {
                admission: adm,
                lease_holder: Some((HolderLabel::new(&rig.lease_conn), rig.lease_gen)),
            },
            precheck: ReservePrecheck::PosesSave,
            candidate: ConfigCandidate::Poses {
                next_list: next_list.clone(),
            },
            write: ConfigWriteOp::from(Some(PendingWrite::new(
                rig.poses_path.clone(),
                serde_json::to_string_pretty(&next_list).unwrap(),
            ))),
            expected_workspace_revision: None,
            expected_poses_revision: None,
        },
        || Ok(()),
        move |cfg, act| {
            cfg.poses.list = nl_pub;
            cfg.poses.revision = act.revision;
        },
    )
    .await
    .expect("poses save commit");
    let poses_v: Value = client
        .get(format!("{base}/api/v1/poses"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(poses_v["revision"], 2);
    assert_eq!(poses_v["poses"][0]["name"], "home_pick");

    // 5. Sequences save + pose delete refused IN_USE while referenced + sequence delete + pose delete
    let pub4 = rig.sup.published_config().await.unwrap();
    let seq_id = armd::recording::new_ulid();
    let seq = Sequence {
        id: seq_id.clone(),
        name: "pick_seq".into(),
        description: String::new(),
        steps: vec![Step::Pose {
            pose_id: pose_id.clone(),
            speed_scale: 1.0,
            gripper: armd::sequences::PoseGripper::Keep,
        }],
        created_at: "2026-11-24T00:00:00.000Z".into(),
        updated_at: "2026-11-24T00:00:00.000Z".into(),
        rev: 1,
    };
    let next_seq_file = SequenceFile {
        sequences: vec![seq.clone()],
    };
    let adm = capture_config_admission(&rig.sup, 10, rig.lease_gen, Some(rig.epoch));
    let prop = build_config_proposal(
        &rig.sup,
        ConfigDomain::Sequences,
        pub4.sequences.revision,
        adm,
        rig.lease_gen,
        pub4.workspace_revision,
    );
    let nf_pub = next_seq_file.clone();
    run_config_commit(
        &rig.sup,
        ConfigCommitPlan {
            proposal: prop,
            auth: ConfigAuthContext {
                admission: adm,
                lease_holder: Some((HolderLabel::new(&rig.lease_conn), rig.lease_gen)),
            },
            precheck: ReservePrecheck::Sequences {
                steps: Some(seq.steps.clone()),
                deleting_sequence_id: None,
            },
            candidate: ConfigCandidate::Sequences {
                next_file: next_seq_file.clone(),
            },
            write: ConfigWriteOp::from(Some(PendingWrite::new(
                rig.sequences_path.clone(),
                serde_json::to_string_pretty(&next_seq_file).unwrap(),
            ))),
            expected_workspace_revision: None,
            expected_poses_revision: None,
        },
        || Ok(()),
        move |cfg, act| {
            cfg.sequences.file = nf_pub;
            cfg.sequences.revision = act.revision;
        },
    )
    .await
    .expect("sequences save commit");

    // REST GET /api/v1/sequences and /api/v1/sequences/{id}
    let seqs_v: Value = client
        .get(format!("{base}/api/v1/sequences"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(seqs_v["revision"], 1);
    assert_eq!(seqs_v["sequences"][0]["id"], seq_id);
    let one_seq: Value = client
        .get(format!("{base}/api/v1/sequences/{seq_id}"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(one_seq["name"], "pick_seq");

    // Deleting referenced pose fails IN_USE atomically on the owner!
    let pub_inuse = rig.sup.published_config().await.unwrap();
    let adm = capture_config_admission(&rig.sup, 10, rig.lease_gen, Some(rig.epoch));
    let prop = build_config_proposal(
        &rig.sup,
        ConfigDomain::Poses,
        pub_inuse.poses.revision,
        adm,
        rig.lease_gen,
        pub_inuse.workspace_revision,
    );
    let err = run_config_commit(
        &rig.sup,
        ConfigCommitPlan {
            proposal: prop,
            auth: ConfigAuthContext {
                admission: adm,
                lease_holder: Some((HolderLabel::new(&rig.lease_conn), rig.lease_gen)),
            },
            precheck: ReservePrecheck::PosesDelete {
                deleting_pose_id: HolderLabel::new(&pose_id),
            },
            candidate: ConfigCandidate::Poses {
                next_list: PoseList::default(),
            },
            write: ConfigWriteOp::Volatile,
            expected_workspace_revision: None,
            expected_poses_revision: None,
        },
        || Ok(()),
        |_, _| {},
    )
    .await
    .unwrap_err();
    assert!(
        matches!(&err, ArmError::Rejected { status: 409, code, .. } if code == "IN_USE"),
        "expected 409 IN_USE, got {err:?}"
    );

    // 6. GraspSetup + SampleTableCapture
    let sampled = rig
        .sup
        .dispatch_config_request(ConfigRequest::SampleTableCapture)
        .await
        .expect("sample table capture");
    match sampled {
        ConfigReply::TableCaptureSampled(Ok((pos, len))) => {
            assert_eq!(len, 7);
            assert!((pos[1] - INSPECT[1]).abs() < 1e-5);
        }
        _ => panic!("unexpected sample reply"),
    }
    let pub5 = rig.sup.published_config().await.unwrap();
    let mut setup = pub5.grasp_setup().unwrap();
    setup.table_z_m = 0.04;
    let adm = capture_config_admission(&rig.sup, 10, rig.lease_gen, Some(rig.epoch));
    let prop = build_config_proposal(
        &rig.sup,
        ConfigDomain::GraspSetup,
        pub5.grasp_setup_revision,
        adm,
        rig.lease_gen,
        pub5.workspace_revision,
    );
    let setup_pub = setup.clone();
    run_config_commit(
        &rig.sup,
        ConfigCommitPlan {
            proposal: prop,
            auth: ConfigAuthContext {
                admission: adm,
                lease_holder: Some((HolderLabel::new(&rig.lease_conn), rig.lease_gen)),
            },
            precheck: ReservePrecheck::GraspSetup {
                new_camera_tilt_deg: setup.camera_tilt_deg as f32,
                allow_during_calibration_busy: false,
            },
            candidate: ConfigCandidate::GraspSetup {
                setup: setup.clone(),
                ws_eval_bufs: None,
            },
            write: ConfigWriteOp::from(Some(PendingWrite::new(
                rig.setup_path.clone(),
                armd::grasp::serialize_setup_for_tool(&setup, &pub5.config.active_tool),
            ))),
            expected_workspace_revision: None,
            expected_poses_revision: None,
        },
        || Ok(()),
        move |cfg, act| {
            cfg.config.grasp = Some(setup_pub);
            cfg.grasp_setup_revision = act.revision;
        },
    )
    .await
    .expect("grasp setup commit");
    let gs_v: Value = client
        .get(format!("{base}/api/v1/grasp/setup"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(gs_v["revision"], 2);
    assert!((gs_v["setup"]["table_z_m"].as_f64().unwrap() - 0.04).abs() < 1e-6);

    // 7. ToolCommissioning via real HTTP POST /api/v1/tool/commissioning/confirm + GET readers
    let tc_before: Value = client
        .get(format!("{base}/api/v1/tool/commissioning"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(tc_before["commissioning_required"], true);
    assert_eq!(tc_before["revision"], 1);

    let confirm_resp = client
        .post(format!("{base}/api/v1/tool/commissioning/confirm"))
        .json(&json!({
            "confirm": true,
            "tool_id": "custom_finger",
            "tool_config_digest": "v1:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "expected_revision": 1
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(confirm_resp.status(), 200);
    let confirm_body: Value = confirm_resp.json().await.unwrap();
    assert_eq!(confirm_body["commissioning_required"], false);
    assert_eq!(confirm_body["revision"], 2);
    assert_eq!(confirm_body["durability_confirmed"], true);
    assert!(rig.commissioning_path.exists());

    // Remaining REST readers: GET /api/v1/tool/commissioning, /api/v1/calibration, /api/v1/safety/describe
    let tc_after: Value = client
        .get(format!("{base}/api/v1/tool/commissioning"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(tc_after["commissioning_required"], false);
    assert_eq!(tc_after["revision"], 2);

    let cal_v: Value = client
        .get(format!("{base}/api/v1/calibration"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(cal_v["grasp_revision"], 2);

    let desc_v: Value = client
        .get(format!("{base}/api/v1/safety/describe"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(desc_v["limits"]["keepout_source"], "runtime");
    assert_eq!(desc_v["limits"]["keepout_boxes"].as_array().unwrap().len(), 1);
    assert_eq!(desc_v["device"]["active_tool"]["id"], "custom_finger");

    // Verify 503 CONFIG_UNAVAILABLE when PublishedConfigHub is marked unavailable in Executor mode.
    if owner == RtOwner::Executor {
        rig.sup.published_config_hub().set_available(false);
        let unavail = client
            .get(format!("{base}/api/v1/workspace/keepout"))
            .send()
            .await
            .unwrap();
        assert_eq!(unavail.status(), 503);
        let body: Value = unavail.json().await.unwrap();
        assert_eq!(body["code"], "CONFIG_UNAVAILABLE");
        rig.sup.published_config_hub().set_available(true);
    }

    rig.assert_executor_zero_alloc("all 7 config domains + 10 REST readers");
    drop(_detached_guard);
    server_jh.abort();
    rig.shutdown().await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn all_config_domains_commit_and_publish_legacy() {
    run_all_domains_test(RtOwner::Legacy).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn all_config_domains_commit_and_publish_executor() {
    run_all_domains_test(RtOwner::Executor).await;
}

// ============================================================================
// 2. Revision conflict & all pre-rename authority revocations (both owners)
// ============================================================================

async fn run_pre_rename_revocations_test(owner: RtOwner) {
    let rig = Rig::new(owner, |_| {}).await;
    // Seed initial keepout on disk.
    let pub0 = rig.sup.published_config().await.unwrap();
    let (plan, pb, rb, eb) = keepout_plan(&rig, &pub0, vec![make_box("initial", 0.3)], 1, 10, rig.lease_gen, true);
    run_config_commit(
        &rig.sup,
        plan,
        || Ok(()),
        |cfg, act| {
            if let Some(ws) = cfg.config.workspace.as_mut() {
                ws.keepout_boxes = eb;
            }
            cfg.keepout.profile = Some(pb);
            cfg.keepout.runtime_boxes = rb;
            cfg.keepout.runtime = true;
            cfg.keepout.revision = act.revision;
        },
    )
    .await
    .unwrap();
    assert_eq!(disk_keepout_names(&rig.keepout_path), ["initial"]);

    let _detached_guard = if owner == RtOwner::Executor {
        Some(rig.sup.inner().write().await)
    } else {
        None
    };

    let pub_cur = rig.sup.published_config().await.unwrap();

    // 2a. Revision mismatch
    let (plan, ..) = keepout_plan(&rig, &pub_cur, vec![make_box("bad_rev", 0.3)], 99, 10, rig.lease_gen, true);
    let err = run_config_commit(&rig.sup, plan, || Ok(()), |_, _| {}).await.unwrap_err();
    assert!(
        matches!(err, ArmError::PolicyRevisionMismatch { expected: 99, current: 2 }),
        "got {err:?}"
    );

    // 2b. Wrong lease generation
    let (plan, ..) = keepout_plan(&rig, &pub_cur, vec![make_box("bad_lease", 0.3)], 2, 10, rig.lease_gen + 5, true);
    let err = run_config_commit(&rig.sup, plan, || Ok(()), |_, _| {}).await.unwrap_err();
    assert!(matches!(&err, ArmError::Rejected { status: 409, code, .. } if code == "LEASE_LOST"), "{err:?}");

    // 2c. Closed connection
    let (plan, ..) = keepout_plan(&rig, &pub_cur, vec![make_box("closed_conn", 0.3)], 2, rig.closed_conn_id, rig.lease_gen, true);
    let err = run_config_commit(&rig.sup, plan, || Ok(()), |_, _| {}).await.unwrap_err();
    assert!(matches!(&err, ArmError::Rejected { status: 409, code, .. } if code == "LEASE_LOST"), "{err:?}");

    // 2d. Stale boot & zero boot
    let (mut plan, ..) = keepout_plan(&rig, &pub_cur, vec![make_box("stale_boot", 0.3)], 2, 10, rig.lease_gen, false);
    plan.proposal.stamp.boot = BootId::fresh();
    let err = run_config_commit(&rig.sup, plan, || Ok(()), |_, _| {}).await.unwrap_err();
    assert!(matches!(&err, ArmError::Rejected { status: 409, code, .. } if code == "WRONG_TARGET"), "{err:?}");

    let (mut plan, ..) = keepout_plan(&rig, &pub_cur, vec![make_box("zero_boot", 0.3)], 2, 10, rig.lease_gen, false);
    plan.proposal.stamp.boot = BootId(0);
    let err = run_config_commit(&rig.sup, plan, || Ok(()), |_, _| {}).await.unwrap_err();
    assert!(matches!(&err, ArmError::Rejected { status: 409, code, .. } if code == "WRONG_TARGET"), "{err:?}");

    // 2e. Expired deadline
    let (mut plan, ..) = keepout_plan(&rig, &pub_cur, vec![make_box("expired", 0.3)], 2, 10, rig.lease_gen, false);
    plan.proposal.stamp.deadline = Instant::now() - Duration::from_millis(5);
    let err = run_config_commit(&rig.sup, plan, || Ok(()), |_, _| {}).await.unwrap_err();
    assert!(matches!(&err, ArmError::Rejected { status: 409, code, .. } if code == "DEADLINE_EXPIRED"), "{err:?}");

    // 2f. Authorized stop intervened before reserve (`refresh_window: false` / `before_reserve` hook)
    let (mut plan, ..) = keepout_plan(&rig, &pub_cur, vec![make_box("overtaken_stop", 0.3)], 2, 10, rig.lease_gen, false);
    plan.proposal.stamp.authority.stop_generation = rig.sup.current_stop_gen() + 1;
    let err = run_config_commit(&rig.sup, plan, || Ok(()), |_, _| {}).await.unwrap_err();
    assert!(matches!(&err, ArmError::Rejected { status: 409, code, .. } if code == "STOPPED"), "{err:?}");

    // 2g. Operator auth revoked BEFORE rename (after ReserveConfig granted) -> hold released `Cleared`!
    let auth_ok = Arc::new(AtomicBool::new(true));
    let auth_flip = Arc::clone(&auth_ok);
    rig.sup.set_config_commit_test_hooks(ConfigCommitTestHooks {
        after_reserve_before_rename: Some(Arc::new(move || {
            auth_flip.store(false, Ordering::SeqCst);
        })),
        ..Default::default()
    });
    let (plan, ..) = keepout_plan(&rig, &pub_cur, vec![make_box("revoked_auth", 0.3)], 2, 10, rig.lease_gen, true);
    let auth_check_flag = Arc::clone(&auth_ok);
    let err = run_config_commit(
        &rig.sup,
        plan,
        move || {
            if auth_check_flag.load(Ordering::SeqCst) {
                Ok(())
            } else {
                Err((
                    "OPERATOR_REQUIRED",
                    ArmError::Rejected {
                        status: 403,
                        code: "OPERATOR_REQUIRED".into(),
                        message: "Operator session locked".into(),
                    },
                ))
            }
        },
        |_, _| {},
    )
    .await
    .unwrap_err();
    rig.sup.set_config_commit_test_hooks(ConfigCommitTestHooks::default());
    assert!(matches!(&err, ArmError::Rejected { status: 403, code, .. } if code == "OPERATOR_REQUIRED"), "{err:?}");
    assert!(!rig.is_config_inhibited().await, "pre-rename auth revocation must release hold");

    // 2h. Stamp reuse (`STAMP_CONSUMED`): reuse the same `admission.request_id` after a granted reservation.
    let (plan1, ..) = keepout_plan(&rig, &pub_cur, vec![make_box("reuse_a", 0.3)], 2, 10, rig.lease_gen, false);
    let reused_req_id = plan1.auth.admission.request_id;
    let res1 = rig
        .sup
        .dispatch_config_request(ConfigRequest::Reserve(ReserveConfig {
            proposal: plan1.proposal,
            auth: plan1.auth,
            precheck: plan1.precheck,
            expected_workspace_revision: None,
            expected_poses_revision: None,
        }))
        .await
        .unwrap();
    let reservation = match res1 {
        ConfigReply::Reserved(Ok(r)) => r,
        _ => panic!("expected grant"),
    };
    let _ = rig
        .sup
        .dispatch_config_request(ConfigRequest::Release(ReleaseConfig { reservation }))
        .await
        .unwrap();
    let (mut plan2, ..) = keepout_plan(&rig, &pub_cur, vec![make_box("reuse_b", 0.3)], 2, 10, rig.lease_gen, false);
    plan2.auth.admission.request_id = reused_req_id;
    let err = run_config_commit(&rig.sup, plan2, || Ok(()), |_, _| {}).await.unwrap_err();
    assert!(matches!(&err, ArmError::Rejected { status: 409, code, .. } if code == "STAMP_CONSUMED"), "{err:?}");

    // 2i. Epoch mismatch (wrong epoch in proposal)
    let (mut plan, ..) = keepout_plan(&rig, &pub_cur, vec![make_box("bad_epoch", 0.3)], 2, 10, rig.lease_gen, true);
    plan.proposal.stamp.authority.epoch = rig.epoch + 9;
    let err = run_config_commit(&rig.sup, plan, || Ok(()), |_, _| {}).await.unwrap_err();
    assert!(matches!(&err, ArmError::Rejected { status: 409, code, .. } if code == "EPOCH_MISMATCH"), "{err:?}");

    // Verify disk untouched, no temp files left behind, revision still 2, not inhibited, and next valid write succeeds.
    assert_eq!(disk_keepout_names(&rig.keepout_path), ["initial"]);
    assert!(
        !dir_entries(rig.keepout_path.parent().unwrap())
            .iter()
            .any(|n| n.contains(".tmp")),
        "staged leftovers must be cleaned up"
    );
    assert!(!rig.is_config_inhibited().await);
    assert_eq!(rig.sup.published_config().await.unwrap().keepout.revision, 2);

    let (plan_ok, pb, rb, eb) = keepout_plan(&rig, &pub_cur, vec![make_box("after_refusals", 0.3)], 2, 10, rig.lease_gen, true);
    let out = run_config_commit(
        &rig.sup,
        plan_ok,
        || Ok(()),
        |cfg, act| {
            if let Some(ws) = cfg.config.workspace.as_mut() {
                ws.keepout_boxes = eb;
            }
            cfg.keepout.profile = Some(pb);
            cfg.keepout.runtime_boxes = rb;
            cfg.keepout.runtime = true;
            cfg.keepout.revision = act.revision;
        },
    )
    .await
    .expect("valid write after pre-rename refusals must succeed");
    assert_eq!(out.activated.revision, 3);
    assert_eq!(disk_keepout_names(&rig.keepout_path), ["after_refusals"]);

    rig.assert_executor_zero_alloc("all 9 pre-rename revocation paths");
    drop(_detached_guard);
    rig.shutdown().await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn revision_conflict_and_authority_revocations_before_rename_legacy() {
    run_pre_rename_revocations_test(RtOwner::Legacy).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn revision_conflict_and_authority_revocations_before_rename_executor() {
    run_pre_rename_revocations_test(RtOwner::Executor).await;
}

// ============================================================================
// 3. Rename failure (Cleared) & dir-sync failure (active + warning) (both owners)
// ============================================================================

async fn run_rename_and_dir_sync_failures_test(owner: RtOwner) {
    let rig = Rig::new(owner, |_| {}).await;
    let pub0 = rig.sup.published_config().await.unwrap();
    let (plan, pb, rb, eb) = keepout_plan(&rig, &pub0, vec![make_box("old", 0.3)], 1, 10, rig.lease_gen, true);
    run_config_commit(
        &rig.sup,
        plan,
        || Ok(()),
        |cfg, act| {
            if let Some(ws) = cfg.config.workspace.as_mut() {
                ws.keepout_boxes = eb;
            }
            cfg.keepout.profile = Some(pb);
            cfg.keepout.runtime_boxes = rb;
            cfg.keepout.runtime = true;
            cfg.keepout.revision = act.revision;
        },
    )
    .await
    .unwrap();

    let _detached_guard = if owner == RtOwner::Executor {
        Some(rig.sup.inner().write().await)
    } else {
        None
    };

    // 3a. Rename failure -> 500 PERSIST_FAILED, hold released (NOT Reconciling), disk/live unchanged.
    rig.sup.set_config_commit_test_hooks(ConfigCommitTestHooks {
        fail_rename_once: Some(Arc::new(AtomicBool::new(true))),
        ..Default::default()
    });
    let pub1 = rig.sup.published_config().await.unwrap();
    let (plan_fail, ..) = keepout_plan(&rig, &pub1, vec![make_box("rename_fail", 0.3)], 2, 10, rig.lease_gen, true);
    let err = run_config_commit(&rig.sup, plan_fail, || Ok(()), |_, _| {})
        .await
        .unwrap_err();
    assert!(matches!(&err, ArmError::Rejected { status: 500, code, .. } if code == "PERSIST_FAILED"), "{err:?}");
    assert_eq!(disk_keepout_names(&rig.keepout_path), ["old"]);
    assert_eq!(rig.sup.published_config().await.unwrap().keepout.revision, 2);
    assert!(!rig.is_config_inhibited().await, "rename failure must NOT leave motion inhibited");
    if owner == RtOwner::Legacy {
        assert_eq!(rig.hold_state(ConfigDomain::Keepout).await, None);
    }

    // 3b. Dir-sync failure -> active at rev 3, durability_confirmed=false, durability_warning=Some, NOT Reconciling.
    rig.sup.set_config_commit_test_hooks(ConfigCommitTestHooks {
        fail_dir_sync_once: Some(Arc::new(AtomicBool::new(true))),
        ..Default::default()
    });
    let (plan_ds, pb, rb, eb) = keepout_plan(&rig, &pub1, vec![make_box("ds_warn", 0.3)], 2, 10, rig.lease_gen, true);
    let out = run_config_commit(
        &rig.sup,
        plan_ds,
        || Ok(()),
        |cfg, act| {
            if let Some(ws) = cfg.config.workspace.as_mut() {
                ws.keepout_boxes = eb;
            }
            cfg.keepout.profile = Some(pb);
            cfg.keepout.runtime_boxes = rb;
            cfg.keepout.runtime = true;
            cfg.keepout.revision = act.revision;
        },
    )
    .await
    .expect("dir-sync failure still activates live state");
    rig.sup.set_config_commit_test_hooks(ConfigCommitTestHooks::default());
    assert_eq!(out.activated.revision, 3);
    assert!(out.persisted);
    assert!(!out.durability_confirmed);
    assert!(out.durability_warning.as_deref().is_some_and(|w| w.contains("fsync")));
    assert_eq!(disk_keepout_names(&rig.keepout_path), ["ds_warn"]);
    assert!(!rig.is_config_inhibited().await);

    // Next write succeeds cleanly at rev 3 -> 4.
    let pub2 = rig.sup.published_config().await.unwrap();
    let (plan_next, pb, rb, eb) = keepout_plan(&rig, &pub2, vec![make_box("after_ds", 0.3)], 3, 10, rig.lease_gen, true);
    let out2 = run_config_commit(
        &rig.sup,
        plan_next,
        || Ok(()),
        |cfg, act| {
            if let Some(ws) = cfg.config.workspace.as_mut() {
                ws.keepout_boxes = eb;
            }
            cfg.keepout.profile = Some(pb);
            cfg.keepout.runtime_boxes = rb;
            cfg.keepout.runtime = true;
            cfg.keepout.revision = act.revision;
        },
    )
    .await
    .unwrap();
    assert_eq!(out2.activated.revision, 4);
    assert!(out2.durability_confirmed);

    rig.assert_executor_zero_alloc("rename failure and dir-sync failure");
    drop(_detached_guard);
    rig.shutdown().await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn rename_failure_and_dir_sync_failure_legacy() {
    run_rename_and_dir_sync_failures_test(RtOwner::Legacy).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn rename_failure_and_dir_sync_failure_executor() {
    run_rename_and_dir_sync_failures_test(RtOwner::Executor).await;
}

// ============================================================================
// 4. Post-rename activation failure -> Reconciling & motion inhibited (both owners)
// ============================================================================

async fn run_post_rename_reconcile_test(owner: RtOwner, variant: u8) {
    let rig = Rig::new(owner, |_| {}).await;
    let pub0 = rig.sup.published_config().await.unwrap();
    let (plan, pb, rb, eb) = keepout_plan(&rig, &pub0, vec![make_box("before_reconcile", 0.3)], 1, 10, rig.lease_gen, true);
    run_config_commit(
        &rig.sup,
        plan,
        || Ok(()),
        |cfg, act| {
            if let Some(ws) = cfg.config.workspace.as_mut() {
                ws.keepout_boxes = eb;
            }
            cfg.keepout.profile = Some(pb);
            cfg.keepout.runtime_boxes = rb;
            cfg.keepout.runtime = true;
            cfg.keepout.revision = act.revision;
        },
    )
    .await
    .unwrap();

    let _detached_guard = if owner == RtOwner::Executor {
        Some(rig.sup.inner().write().await)
    } else {
        None
    };

    let pub1 = rig.sup.published_config().await.unwrap();
    let auth_ok = Arc::new(AtomicBool::new(true));

    match variant {
        0 => {
            // Revoke operator auth AFTER rename and BEFORE ActivateConfig.
            let flip = Arc::clone(&auth_ok);
            rig.sup.set_config_commit_test_hooks(ConfigCommitTestHooks {
                after_rename_before_activate: Some(Arc::new(move || {
                    flip.store(false, Ordering::SeqCst);
                })),
                ..Default::default()
            });
        }
        1 => {
            // Trigger E-stop AFTER rename and BEFORE ActivateConfig.
            let sup_es = Arc::clone(&rig.sup);
            rig.sup.set_config_commit_test_hooks(ConfigCommitTestHooks {
                after_rename_before_activate: Some(Arc::new(move || {
                    tokio::task::block_in_place(|| {
                        let _ = tokio::runtime::Handle::current().block_on(sup_es.trigger_e_stop());
                    });
                })),
                ..Default::default()
            });
        }
        _ => {
            // Simulate PersistedConfig failure after rename -> forces Reconciling.
            rig.sup.set_config_commit_test_hooks(ConfigCommitTestHooks {
                fail_persisted_once: Some(Arc::new(AtomicBool::new(true))),
                ..Default::default()
            });
        }
    }

    let (plan_rec, ..) = keepout_plan(&rig, &pub1, vec![make_box("renamed_not_active", 0.3)], 2, 10, rig.lease_gen, true);
    let auth_chk = Arc::clone(&auth_ok);
    let err = run_config_commit(
        &rig.sup,
        plan_rec,
        move || {
            if auth_chk.load(Ordering::SeqCst) {
                Ok(())
            } else {
                Err((
                    "OPERATOR_REQUIRED",
                    ArmError::Rejected {
                        status: 403,
                        code: "OPERATOR_REQUIRED".into(),
                        message: "revoked post-rename".into(),
                    },
                ))
            }
        },
        |_, _| {},
    )
    .await
    .unwrap_err();
    rig.sup.set_config_commit_test_hooks(ConfigCommitTestHooks::default());

    assert!(
        matches!(&err, ArmError::Rejected { status: 409, message, .. } if message.contains("reconciling")),
        "variant {variant}: expected Reconciling 409 error, got {err:?}"
    );
    // Disk has the renamed file, but live state & revision stay at 2 ("before_reconcile")!
    assert_eq!(disk_keepout_names(&rig.keepout_path), ["renamed_not_active"]);
    let pub_after = rig.sup.published_config().await.unwrap();
    assert_eq!(pub_after.keepout.revision, 2);
    assert_eq!(pub_after.keepout.runtime_boxes[0].name, "before_reconcile");
    // Motion stays inhibited (`Reconciling`).
    assert!(rig.is_config_inhibited().await, "variant {variant}: Reconciling must inhibit motion");
    if owner == RtOwner::Legacy {
        assert_eq!(rig.hold_state(ConfigDomain::Keepout).await, Some(HoldState::Reconciling));
    }

    // Subsequent write on Keepout is refused with CONFIG_BUSY (when not overridden by E-stop epoch).
    if variant != 1 {
        let (plan_busy, ..) = keepout_plan(&rig, &pub_after, vec![make_box("blocked", 0.3)], 2, 10, rig.lease_gen, true);
        let err_busy = run_config_commit(&rig.sup, plan_busy, || Ok(()), |_, _| {})
            .await
            .unwrap_err();
        assert!(
            matches!(&err_busy, ArmError::Rejected { status: 409, code, .. } if code == "CONFIG_BUSY"),
            "variant {variant}: subsequent write during Reconciling must return CONFIG_BUSY, got {err_busy:?}"
        );
    }

    rig.assert_executor_zero_alloc("post-rename activation failure -> Reconciling");
    drop(_detached_guard);
    rig.shutdown().await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn post_rename_activation_failures_enter_reconciling_legacy() {
    for v in 0..3 {
        run_post_rename_reconcile_test(RtOwner::Legacy, v).await;
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn post_rename_activation_failures_enter_reconciling_executor() {
    for v in 0..3 {
        run_post_rename_reconcile_test(RtOwner::Executor, v).await;
    }
}

// ============================================================================
// 5. ReservationGuard drop before rename (Cleared) vs after rename (Reconciling)
// ============================================================================

async fn run_reservation_guard_drop_test(owner: RtOwner) {
    let rig = Rig::new(owner, |_| {}).await;
    let _detached_guard = if owner == RtOwner::Executor {
        Some(rig.sup.inner().write().await)
    } else {
        None
    };

    let pub0 = rig.sup.published_config().await.unwrap();

    // 5a. Drop ReservationGuard BEFORE mark_renamed -> hold releases (`Cleared`), motion uninhibited.
    {
        let (plan, ..) = keepout_plan(&rig, &pub0, vec![make_box("g1", 0.3)], 1, 10, rig.lease_gen, false);
        let r = match rig
            .sup
            .dispatch_config_request(ConfigRequest::Reserve(ReserveConfig {
                proposal: plan.proposal,
                auth: plan.auth,
                precheck: plan.precheck,
                expected_workspace_revision: None,
                expected_poses_revision: None,
            }))
            .await
            .unwrap()
        {
            ConfigReply::Reserved(Ok(r)) => r,
            _ => panic!("expected Reserved(Ok)"),
        };
        assert!(rig.is_config_inhibited().await, "motion must be inhibited while reserved");
        let guard = ReservationGuard::new(Arc::clone(&rig.sup), r);
        drop(guard);
    }
    let deadline = Instant::now() + Duration::from_secs(2);
    while rig.is_config_inhibited().await {
        assert!(Instant::now() < deadline, "pre-rename ReservationGuard drop did not release hold");
        tokio::time::sleep(Duration::from_millis(5)).await;
    }

    // 5b. Drop ReservationGuard AFTER mark_renamed -> owner moves hold to Reconciling (motion stays inhibited).
    {
        let (plan, ..) = keepout_plan(&rig, &pub0, vec![make_box("g2", 0.3)], 1, 10, rig.lease_gen, false);
        let r = match rig
            .sup
            .dispatch_config_request(ConfigRequest::Reserve(ReserveConfig {
                proposal: plan.proposal,
                auth: plan.auth,
                precheck: plan.precheck,
                expected_workspace_revision: None,
                expected_poses_revision: None,
            }))
            .await
            .unwrap()
        {
            ConfigReply::Reserved(Ok(r)) => r,
            _ => panic!("expected Reserved(Ok)"),
        };
        let mut guard = ReservationGuard::new(Arc::clone(&rig.sup), r);
        guard.mark_renamed();
        drop(guard);
    }
    // Wait until async drop task completes and verify subsequent reserve fails CONFIG_BUSY & motion inhibited.
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        if owner == RtOwner::Legacy {
            if rig.hold_state(ConfigDomain::Keepout).await == Some(HoldState::Reconciling) {
                break;
            }
        } else {
            let (probe, ..) = keepout_plan(&rig, &pub0, vec![make_box("g3", 0.3)], 1, 10, rig.lease_gen, false);
            let res = rig
                .sup
                .dispatch_config_request(ConfigRequest::Reserve(ReserveConfig {
                    proposal: probe.proposal,
                    auth: probe.auth,
                    precheck: probe.precheck,
                    expected_workspace_revision: None,
                    expected_poses_revision: None,
                }))
                .await
                .unwrap();
            if matches!(res, ConfigReply::Reserved(Err(armd::rt_core::requests::config::ConfigError::Busy(_)))) {
                break;
            }
        }
        assert!(Instant::now() < deadline, "post-rename ReservationGuard drop did not enter Reconciling");
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    assert!(rig.is_config_inhibited().await, "post-rename ReservationGuard drop must keep motion inhibited");

    rig.assert_executor_zero_alloc("ReservationGuard drop before and after rename");
    drop(_detached_guard);
    rig.shutdown().await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn reservation_guard_drop_before_and_after_rename_legacy() {
    run_reservation_guard_drop_test(RtOwner::Legacy).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn reservation_guard_drop_before_and_after_rename_executor() {
    run_reservation_guard_drop_test(RtOwner::Executor).await;
}

// ============================================================================
// 6. Calibration apply/restore + motor_zero + MIT refusal (both owners)
// ============================================================================

async fn run_calibration_and_zero_test(owner: RtOwner) {
    let (vision_addr, _vision_state, vision_jh) = spawn_mock_vision().await;
    let rig = Rig::new(owner, |_| {}).await;
    let (base, server_jh) = spawn_http_router(&rig, Some(vision_addr), OperatorAuth::disabled()).await;
    let client = reqwest::Client::new();

    let det_up = armd::api::proxy::DetectorUpstream::parse(&format!("http://{vision_addr}")).unwrap();
    let cal_deps = CalibrationDeps::new(
        rig.calib_path.clone(),
        Some(rig.setup_path.clone()),
        det_up,
    );

    let _detached_guard = if owner == RtOwner::Executor {
        Some(rig.sup.inner().write().await)
    } else {
        None
    };

    let valid_q = armd::kinematics::r_to_quat(
        &armd::chain::ArmModel::REBOT_DEV_ARM.cam_mount(false, 0.0, -25.0),
    );
    let apply_body = json!({
        "dry_run": false,
        "camera_mount": {
            "offset_m": [0.01, -0.02, 0.05],
            "q_end_cam": valid_q
        },
        "intrinsics": {
            "fx": 831.5, "fy": 830.0, "cx": 321.0, "cy": 179.0,
            "k1": 0.0, "k2": 0.0, "p1": 0.0, "p2": 0.0, "k3": 0.0,
            "width": 640, "height": 360
        },
        "table_z_m": 0.012
    });
    let req: ApplyRequest = armd::calibration::parse_apply(&apply_body).expect("parse apply");
    let check: armd::calibration::LeaseCheck = Box::new(|_| Ok(()));
    let auth = CalibCommitAuth {
        admission: capture_config_admission(&rig.sup, 10, rig.lease_gen, Some(rig.epoch)),
        lease_holder: Some((HolderLabel::new(&rig.lease_conn), rig.lease_gen)),
        off_thread_check: Arc::new(|| Ok(())),
        job_token: None,
        deadline: Instant::now() + Duration::from_secs(30),
    };

    let applied = apply_with_auth(&rig.sup, &cal_deps, req, &check, &auth)
        .await
        .expect("calibration apply");
    let backup_path = applied["backup"].as_str().unwrap().to_string();
    let backup_name = Path::new(&backup_path)
        .file_name()
        .unwrap()
        .to_string_lossy()
        .to_string();
    assert!(rig.calib_path.exists());

    // REST GET /api/v1/calibration & /api/v1/grasp/setup reflect the applied calibration immediately.
    let cal_v: Value = client
        .get(format!("{base}/api/v1/calibration"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(cal_v["calibrated"], true);
    let gs_v: Value = client
        .get(format!("{base}/api/v1/grasp/setup"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert!((gs_v["setup"]["table_z_m"].as_f64().unwrap() - 0.012).abs() < 1e-6);

    // Calibration restore back to pre-apply backup.
    let restore_auth = CalibCommitAuth {
        admission: capture_config_admission(&rig.sup, 10, rig.lease_gen, Some(rig.epoch)),
        lease_holder: Some((HolderLabel::new(&rig.lease_conn), rig.lease_gen)),
        off_thread_check: Arc::new(|| Ok(())),
        job_token: None,
        deadline: Instant::now() + Duration::from_secs(30),
    };
    let restored = restore_with_auth(
        &rig.sup,
        &cal_deps,
        &json!({ "backup": backup_name }),
        &check,
        &restore_auth,
    )
    .await
    .expect("calibration restore");
    assert!(restored["backup"].is_string());
    let cal_after: Value = client
        .get(format!("{base}/api/v1/calibration"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(cal_after["calibrated"], false);

    // Mock motor zero via `motor_zero::run` (routes through `set_motor_zero_async` + `InvalidateAfterZero` on Executor).
    let mz_req = MotorZeroRequest {
        joints: vec![0],
        dry_run: false,
        force: false,
        method: Some("motor".into()),
    };
    let mz_res = motor_zero::run(&rig.sup, &cal_deps, mz_req, &check)
        .await
        .expect("mock motor zero");
    assert_eq!(mz_res["zero_method"], "motor");
    assert_eq!(mz_res["joints"][0]["verified"], true);

    // MIT qualification in Executor mode MUST fail closed with 503 EXECUTOR_CAPABILITY_UNSUPPORTED.
    if owner == RtOwner::Executor {
        let mit_err = armd::mit::store_qualification(
            &rig.sup,
            &cal_deps,
            CalibrationFile::default(),
            &check,
        )
        .await
        .unwrap_err();
        assert_eq!(mit_err.status, 503);
        assert_eq!(mit_err.code, "EXECUTOR_CAPABILITY_UNSUPPORTED");
    }

    rig.assert_executor_zero_alloc("calibration apply/restore + mock motor zero");
    drop(_detached_guard);
    server_jh.abort();
    vision_jh.abort();
    rig.shutdown().await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn calibration_apply_restore_and_motor_zero_legacy() {
    run_calibration_and_zero_test(RtOwner::Legacy).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn calibration_apply_restore_and_motor_zero_executor() {
    run_calibration_and_zero_test(RtOwner::Executor).await;
}

// ============================================================================
// 7. Gravity-comp (mock zero-alloc vs hardware 503 refusal) + hw motor_zero refusal
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn gravity_comp_mock_zero_alloc_and_hw_capability_refusals_executor() {
    // Part A: Mock (driverless) Executor -> gravity_comp enter/exit runs zero-alloc on armd-control.
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let lease_conn = "gc-conn";
    let (lease_gen, epoch) = {
        let mut inner = sup.inner().write().await;
        inner.measured_positions = INSPECT.to_vec();
        inner.commanded_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        let lgen = inner.acquire_lease(lease_conn).unwrap();
        let ep = inner.safety.epoch_id();
        let adm = sup.capture_admission_stamp();
        let admitted = inner.check_admission(adm).unwrap();
        armd::SupervisorInner::enable_torque(admitted).unwrap();
        (lgen, ep)
    };
    let mut ecfg = ExecutorConfig::for_tests(Duration::from_millis(2));
    ecfg.alloc_timing = true;
    let handle = sup.start_executor(ecfg).await.unwrap();
    let deadline = Instant::now() + Duration::from_secs(3);
    while sup.hub().latest().is_none() {
        assert!(Instant::now() < deadline);
        tokio::time::sleep(Duration::from_millis(5)).await;
    }

    let _detached_guard = sup.inner().write().await;

    let adm = capture_config_admission(&sup, 42, lease_gen, Some(epoch));
    let enter = gravity_comp_command(
        &sup,
        adm,
        Some((HolderLabel::new(lease_conn), lease_gen)),
        true,
        Some(0.8),
        Some(vec![0.2; 6]),
    )
    .await
    .expect("mock gravity_comp enter on executor");
    assert!(enter.active);
    assert!(enter.is_torqued);

    let adm_exit = capture_config_admission(&sup, 42, lease_gen, Some(epoch));
    let exit = gravity_comp_command(
        &sup,
        adm_exit,
        Some((HolderLabel::new(lease_conn), lease_gen)),
        false,
        None,
        None,
    )
    .await
    .expect("mock gravity_comp exit on executor");
    assert!(exit.is_torqued);

    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);
    assert_eq!(sup.inner().control_thread_attempts(), 0);
    drop(_detached_guard);
    tokio::task::spawn_blocking(move || handle.stop_and_join())
        .await
        .unwrap();

    // Part B: Hardware driver attached on Executor -> gravity_comp & hardware motor_zero fail closed 503.
    let sup_hw = Arc::new(
        ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(MinimalHwDriver::new(7))),
    );
    let mut ecfg_hw = ExecutorConfig::for_tests(Duration::from_millis(2));
    ecfg_hw.alloc_timing = true;
    let handle_hw = sup_hw.start_executor(ecfg_hw).await.unwrap();
    let deadline = Instant::now() + Duration::from_secs(3);
    while sup_hw.hub().latest().is_none() {
        assert!(Instant::now() < deadline);
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    let _detached_hw = sup_hw.inner().write().await;

    let err_enter = gravity_comp_command(
        &sup_hw,
        capture_config_admission(&sup_hw, 1, 0, Some(1)),
        None,
        true,
        None,
        None,
    )
    .await
    .unwrap_err();
    assert!(
        matches!(&err_enter, ArmError::Rejected { status: 503, code, .. } if code == "EXECUTOR_CAPABILITY_UNSUPPORTED"),
        "expected 503 EXECUTOR_CAPABILITY_UNSUPPORTED, got {err_enter:?}"
    );
    let err_exit = gravity_comp_command(
        &sup_hw,
        capture_config_admission(&sup_hw, 1, 0, Some(1)),
        None,
        false,
        None,
        None,
    )
    .await
    .unwrap_err();
    assert!(
        matches!(&err_exit, ArmError::Rejected { status: 503, code, .. } if code == "EXECUTOR_CAPABILITY_UNSUPPORTED"),
        "expected 503 EXECUTOR_CAPABILITY_UNSUPPORTED on exit, got {err_exit:?}"
    );

    let pub_hw = sup_hw.published_config().await.unwrap();
    let obs_hw = sup_hw.hub().latest().unwrap().obs.clone();
    let mz_hw_err = motor_zero::check_executor(&pub_hw, &obs_hw, &json!({ "joints": [1], "method": "motor" }))
        .unwrap_err();
    assert_eq!(mz_hw_err.status, 503);
    assert_eq!(mz_hw_err.code, "EXECUTOR_CAPABILITY_UNSUPPORTED");

    assert_eq!(handle_hw.stats().total_allocs(), AllocCounts::ZERO);
    assert_eq!(sup_hw.inner().control_thread_attempts(), 0);
    drop(_detached_hw);
    tokio::task::spawn_blocking(move || handle_hw.stop_and_join())
        .await
        .unwrap();
}

// ============================================================================
// 8. WS Executor routing for PR C paths vs unmigrated paths (zero-alloc + lock-proof)
// ============================================================================

async fn ws_send_recv(
    ws: &mut tokio_tungstenite::WebSocketStream<
        tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
    >,
    frame: Value,
) -> Value {
    let expected_req_id = frame.get("request_id").cloned();
    ws.send(Message::Text(frame.to_string().into())).await.unwrap();
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        let rem = deadline.saturating_duration_since(Instant::now());
        assert!(!rem.is_zero(), "timed out waiting for WS reply");
        let msg = tokio::time::timeout(rem, ws.next())
            .await
            .unwrap()
            .unwrap()
            .unwrap();
        if let Message::Text(t) = msg {
            let v: Value = serde_json::from_str(&t).unwrap();
            let ty = v.get("type").and_then(Value::as_str);
            if matches!(ty, Some("command_ack" | "command_reject"))
                && (expected_req_id.is_none() || v.get("request_id") == expected_req_id.as_ref())
            {
                return v;
            }
        }
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn ws_executor_routes_pr_c_paths_and_leaves_unmigrated_paths_pending() {
    let rig = Rig::new(RtOwner::Executor, |_| {}).await;
    let auth = OperatorAuth::enforced("123456", 1800);
    let (base, server_jh) = spawn_http_router(&rig, None, auth).await;
    let ws_url = format!("{}/api/v1/ws", base.replace("http://", "ws://"));

    let _detached_guard = rig.sup.inner().write().await;

    let (mut ws, _) = tokio_tungstenite::connect_async(&ws_url).await.unwrap();

    // 1. Unmigrated path (`/api/v1/torque`) MUST still return 503 EXECUTOR_ROUTING_PENDING.
    let torque_rej = ws_send_recv(
        &mut ws,
        json!({
            "type": "command",
            "request_id": 1,
            "generation": 1,
            "epoch_id": 1,
            "path": "/api/v1/torque",
            "body": { "enable": true }
        }),
    )
    .await;
    assert_eq!(torque_rej["type"], "command_reject");
    assert_eq!(torque_rej["status"], 503);
    assert_eq!(torque_rej["code"], "EXECUTOR_ROUTING_PENDING");

    // 2. PR C config path without lease on this WS connection MUST be routed by PR C (409 LEASE_LOST, NOT 503 EXECUTOR_ROUTING_PENDING).
    let cfg_rej = ws_send_recv(
        &mut ws,
        json!({
            "type": "command",
            "request_id": 2,
            "generation": 1,
            "epoch_id": 1,
            "path": "/api/v1/workspace/keepout",
            "body": { "boxes": [], "expected_revision": 1 }
        }),
    )
    .await;
    assert_eq!(cfg_rej["type"], "command_reject");
    assert_eq!(cfg_rej["status"], 409);
    assert_eq!(cfg_rej["code"], "LEASE_LOST");

    // 3. PR C gravity_comp path without lease MUST also be routed by PR C (409 LEASE_LOST).
    let gc_rej = ws_send_recv(
        &mut ws,
        json!({
            "type": "command",
            "request_id": 3,
            "generation": 1,
            "epoch_id": 1,
            "path": "/api/v1/mode/gravity_comp",
            "body": { "enable": true }
        }),
    )
    .await;
    assert_eq!(gc_rej["type"], "command_reject");
    assert_eq!(gc_rej["status"], 409);
    assert_eq!(gc_rej["code"], "LEASE_LOST");

    // 4. PR C calibration path without lease MUST also be routed by PR C (409 LEASE_LOST).
    let cal_rej = ws_send_recv(
        &mut ws,
        json!({
            "type": "command",
            "request_id": 4,
            "generation": 1,
            "epoch_id": 1,
            "path": "/api/v1/calibration/mit_timeout",
            "body": {}
        }),
    )
    .await;
    assert_eq!(cal_rej["type"], "command_reject");
    assert_eq!(cal_rej["status"], 409);
    assert_eq!(cal_rej["code"], "LEASE_LOST");

    rig.assert_executor_zero_alloc("WS executor PR C routing");
    drop(_detached_guard);
    server_jh.abort();
    rig.shutdown().await;
}

// ============================================================================
// 9. Codex review regression invariants (zero-alloc refusals, stale workspace,
//    pre-rename live authority verify, auto gravity-comp exit, exited executor 503)
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn codex_review_regression_invariants_executor() {
    // 9a. Workspace-dependent keepout commit refused PLAN_STALE when expected_workspace_revision mismatches.
    let rig = Rig::new(RtOwner::Executor, |_| {}).await;
    let _detached = rig.sup.inner().write().await;
    let pub0 = rig.sup.published_config().await.unwrap();
    let (mut stale_plan, ..) = keepout_plan(&rig, &pub0, vec![make_box("stale_ws", 0.3)], 1, 10, rig.lease_gen, false);
    stale_plan.expected_workspace_revision = Some(pub0.workspace_revision + 99);
    let err = run_config_commit(&rig.sup, stale_plan, || Ok(()), |_, _| {})
        .await
        .unwrap_err();
    assert!(
        matches!(&err, ArmError::Motion(armd::motion::MotionError::PlanStale(_))),
        "expected PlanStale for stale workspace_revision, got {err:?}"
    );
    assert!(!rig.keepout_path.exists());
    assert!(!rig.is_config_inhibited().await);

    // 9b. Pre-rename live owner authority check (`VerifyAuthority`): bump stop_gen AFTER reserve
    // and BEFORE rename -> rename MUST NOT happen, hold releases `Cleared`.
    let sup_hook = Arc::clone(&rig.sup);
    rig.sup.set_config_commit_test_hooks(ConfigCommitTestHooks {
        after_reserve_before_rename: Some(Arc::new(move || {
            let _ = sup_hook.executor_handle().unwrap().stop_requests().request();
        })),
        ..Default::default()
    });
    let (plan_stop, ..) = keepout_plan(&rig, &pub0, vec![make_box("stop_before_rename", 0.3)], 1, 10, rig.lease_gen, false);
    let err = run_config_commit(&rig.sup, plan_stop, || Ok(()), |_, _| {})
        .await
        .unwrap_err();
    rig.sup.set_config_commit_test_hooks(ConfigCommitTestHooks::default());
    assert!(
        matches!(&err, ArmError::Rejected { status: 409, code, .. } if code == "STOPPED"),
        "expected 409 STOPPED from pre-rename VerifyAuthority, got {err:?}"
    );
    assert!(!rig.keepout_path.exists(), "destination file must not be renamed after pre-rename stop");
    assert!(!rig.is_config_inhibited().await, "hold must be Cleared after pre-rename stop");

    // 9c. Ordinary motor-zero / offset refusal under E-stop allocates 0 on armd-control.
    let _ = rig.sup.trigger_e_stop().await;
    let res = rig
        .sup
        .dispatch_config_request(ConfigRequest::SetJointZeroOffsets {
            offsets: vec![0.0; 7],
            precomputed_floor: None,
            auth: None,
            expected_stop_gen: None,
            deadline: None,
            job_token: None,
        })
        .await
        .unwrap();
    assert!(matches!(res, ConfigReply::JointZeroOffsetsSet(Err(_))));
    let handle_ref = rig.sup.executor_handle().unwrap();
    let legacy_err = handle_ref.set_joint_zero_offsets_async(vec![0.0; 7]).await.unwrap();
    assert!(legacy_err.is_err());

    rig.assert_executor_zero_alloc("stale workspace + pre-rename VerifyAuthority + E-stop zero refusal");
    drop(_detached);

    // 9d. Stopped executor MUST fail closed 503 CONFIG_UNAVAILABLE on `published_config()`.
    let handle = rig.sup.executor_handle().unwrap();
    tokio::task::spawn_blocking(move || handle.stop_and_join())
        .await
        .unwrap();
    let stopped_err = rig
        .sup
        .published_config()
        .await
        .err()
        .expect("stopped executor must fail closed");
    assert!(
        matches!(&stopped_err, ArmError::Rejected { status: 503, code, .. } if code == "CONFIG_UNAVAILABLE"),
        "stopped executor must reject published_config() with 503 CONFIG_UNAVAILABLE, got {stopped_err:?}"
    );

    // 9e. Automatic gravity-comp exit on lease timeout executes with 0 allocations on armd-control.
    // Clear `resting_pose` so post-exit lease watchdog latches `WatchdogTimeout` instead of
    // invoking the unmigrated `StopPolicy::NormalUnload` trajectory planner.
    let mut cfg_gc = ArmConfig::rebot_dm();
    cfg_gc.resting_pose = None;
    let sup_gc = Arc::new(ArmSupervisor::new(cfg_gc));
    let lease_conn = "gc-auto-exit";
    let (lease_gen, epoch) = {
        let mut inner = sup_gc.inner().write().await;
        inner.lease_timeout = Duration::from_millis(60);
        inner.measured_positions = INSPECT.to_vec();
        inner.commanded_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        let lgen = inner.acquire_lease(lease_conn).unwrap();
        let ep = inner.safety.epoch_id();
        let adm = sup_gc.capture_admission_stamp();
        let admitted = inner.check_admission(adm).unwrap();
        armd::SupervisorInner::enable_torque(admitted).unwrap();
        (lgen, ep)
    };
    let mut ecfg = ExecutorConfig::for_tests(Duration::from_millis(2));
    ecfg.alloc_timing = true;
    let handle_gc = sup_gc.start_executor(ecfg).await.unwrap();
    let deadline = Instant::now() + Duration::from_secs(3);
    while sup_gc.hub().latest().is_none() {
        assert!(Instant::now() < deadline);
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    let _detached_gc = sup_gc.inner().write().await;
    let adm = capture_config_admission(&sup_gc, 77, lease_gen, Some(epoch));
    let enter = gravity_comp_command(
        &sup_gc,
        adm,
        Some((HolderLabel::new(lease_conn), lease_gen)),
        true,
        Some(0.8),
        Some(vec![0.2; 6]),
    )
    .await
    .expect("enter gravity comp");
    assert!(enter.active);
    // Wait for lease watchdog (> 60 ms) to trigger automatic gravity-comp exit on armd-control.
    let wait_exit = Instant::now() + Duration::from_secs(2);
    loop {
        if let Some(s) = sup_gc.hub().latest() {
            if s.obs.control_mode != armd::model::ControlMode::GravityComp {
                break;
            }
        }
        assert!(Instant::now() < wait_exit, "timed out waiting for automatic gravity_comp exit");
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    assert_eq!(
        handle_gc.stats().total_allocs(),
        AllocCounts::ZERO,
        "automatic gravity-comp exit allocated on armd-control: {:?}",
        handle_gc.stats().total_allocs()
    );
    assert_eq!(sup_gc.inner().control_thread_attempts(), 0);
    drop(_detached_gc);
    tokio::task::spawn_blocking(move || handle_gc.stop_and_join())
        .await
        .unwrap();

    // 9f. Hardware driver gripper calibration validates against cached_driver_pos_limits (0 mutex lock, 0 alloc).
    let sup_hw = Arc::new(
        ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(MinimalHwDriver::new(7))),
    );
    let lease_conn_hw = "hw-grip-conn";
    let (lgen_hw, ep_hw) = {
        let mut inner = sup_hw.inner().write().await;
        let l = inner.acquire_lease(lease_conn_hw).unwrap();
        (l, inner.safety.epoch_id())
    };
    let mut ecfg_hw = ExecutorConfig::for_tests(Duration::from_millis(2));
    ecfg_hw.alloc_timing = true;
    let handle_hw = sup_hw.start_executor(ecfg_hw).await.unwrap();
    let deadline = Instant::now() + Duration::from_secs(3);
    while sup_hw.hub().latest().is_none() {
        assert!(Instant::now() < deadline);
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    let _detached_hw = sup_hw.inner().write().await;
    let pub_hw = sup_hw.published_config().await.unwrap();
    let next_pol = pub_hw.config.gripper_policy.clone().unwrap();
    let adm_hw = capture_config_admission(&sup_hw, 88, lgen_hw, Some(ep_hw));
    let prop_hw = build_config_proposal(
        &sup_hw,
        ConfigDomain::GripperCalibration,
        pub_hw.gripper_calib_revision,
        adm_hw,
        lgen_hw,
        pub_hw.workspace_revision,
    );
    let out_hw = run_config_commit(
        &sup_hw,
        ConfigCommitPlan {
            proposal: prop_hw,
            auth: ConfigAuthContext {
                admission: adm_hw,
                lease_holder: Some((HolderLabel::new(lease_conn_hw), lgen_hw)),
            },
            precheck: ReservePrecheck::GripperCalibration {
                candidate_policy: Some(next_pol.clone()),
            },
            candidate: ConfigCandidate::GripperCalibration {
                offset_rad: Some(0.01),
                policy: next_pol,
                jaw_opening: None,
                clear_stale: true,
            },
            write: ConfigWriteOp::Volatile,
            expected_workspace_revision: None,
            expected_poses_revision: None,
        },
        || Ok(()),
        move |cfg, act| {
            cfg.gripper_calib_offset = Some(0.01);
            cfg.gripper_calib_invalid = false;
            cfg.gripper_calib_revision = act.revision;
        },
    )
    .await
    .expect("hw gripper calib commit via cached_driver_pos_limits");
    assert_eq!(out_hw.activated.revision, 2);
    assert_eq!(handle_hw.stats().total_allocs(), AllocCounts::ZERO);
    assert_eq!(sup_hw.inner().control_thread_attempts(), 0);
    drop(_detached_hw);
    tokio::task::spawn_blocking(move || handle_hw.stop_and_join())
        .await
        .unwrap();
}
