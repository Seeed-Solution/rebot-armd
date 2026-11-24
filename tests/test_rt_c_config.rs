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
use armd::calibration::{ApplyRequest, CalibrationDeps, CalibrationFile};
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
                reserved_pins: None,
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

    // 3b. Dir-sync failure -> MUST NOT activate: keeps OLD policy active (rev 2, "old"),
    //     enters HoldState::Reconciling, keeps motion inhibited, refuses follow-up writes CONFIG_BUSY.
    rig.sup.set_config_commit_test_hooks(ConfigCommitTestHooks {
        fail_dir_sync_once: Some(Arc::new(AtomicBool::new(true))),
        ..Default::default()
    });
    let (plan_ds, pb, rb, eb) = keepout_plan(&rig, &pub1, vec![make_box("ds_warn", 0.3)], 2, 10, rig.lease_gen, true);
    let ds_err = armd::rt_core::requests::config::run_config_commit_detailed(
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
    .expect_err("dir-sync failure must not activate and must enter Reconciling");
    rig.sup.set_config_commit_test_hooks(ConfigCommitTestHooks::default());
    assert!(ds_err.disk_published, "rename occurred before dir-sync failure");
    assert!(ds_err.reconciling, "dir-sync failure must enter Reconciling");
    assert!(
        matches!(&ds_err.error, ArmError::Rejected { status: 500, code, .. } if code == "PERSIST_FAILED"),
        "{ds_err:?}"
    );
    // File on disk was renamed, while live policy and PublishedConfig stay at the OLD revision (2, "old").
    assert_eq!(disk_keepout_names(&rig.keepout_path), ["ds_warn"]);
    let pub_after_ds = rig.sup.published_config().await.unwrap();
    assert_eq!(pub_after_ds.keepout.revision, 2, "old policy revision must stay active");
    let live_names: Vec<String> = pub_after_ds
        .keepout
        .runtime_boxes
        .iter()
        .map(|b| b.name.clone())
        .collect();
    assert_eq!(live_names, ["old"], "old keepout policy must remain active after dir-sync failure");
    assert!(rig.is_config_inhibited().await, "motion must remain inhibited until reconciliation");
    assert_eq!(
        rig.hold_state(ConfigDomain::Keepout).await,
        Some(HoldState::Reconciling),
        "dir-sync failure must transition hold to Reconciling"
    );

    // Follow-up write while Reconciling is refused CONFIG_BUSY.
    let (plan_busy, ..) = keepout_plan(&rig, &pub_after_ds, vec![make_box("while_reconciling", 0.3)], 2, 10, rig.lease_gen, true);
    let busy_err = run_config_commit(&rig.sup, plan_busy, || Ok(()), |_, _| {})
        .await
        .unwrap_err();
    assert!(
        matches!(&busy_err, ArmError::Rejected { status: 409, code, .. } if code == "CONFIG_BUSY"),
        "expected CONFIG_BUSY while Reconciling after dir-sync failure, got {busy_err:?}"
    );
    assert!(rig.is_config_inhibited().await);

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
// 6. Legacy calibration apply/restore + motor zero (pre-PR-C eb101bb parity)
//    and Executor fail-closed 409 EXECUTOR_UNSUPPORTED per-op refusal tests
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn calibration_apply_restore_and_motor_zero_legacy() {
    let (vision_addr, _vision_state, vision_jh) = spawn_mock_vision().await;
    let rig = Rig::new(RtOwner::Legacy, |_| {}).await;
    let (base, server_jh) = spawn_http_router(&rig, Some(vision_addr), OperatorAuth::disabled()).await;
    let client = reqwest::Client::new();

    let det_up = armd::api::proxy::DetectorUpstream::parse(&format!("http://{vision_addr}")).unwrap();
    let cal_deps = CalibrationDeps::new(
        rig.calib_path.clone(),
        Some(rig.setup_path.clone()),
        det_up,
    );

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

    let applied = armd::calibration::apply(&rig.sup, &cal_deps, req, &check)
        .await
        .expect("legacy calibration apply");
    let backup_path = applied["backup"].as_str().unwrap().to_string();
    let backup_name = Path::new(&backup_path)
        .file_name()
        .unwrap()
        .to_string_lossy()
        .to_string();
    assert!(rig.calib_path.exists());

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

    let restored = armd::calibration::restore(&rig.sup, &cal_deps, &json!({ "backup": backup_name }), &check)
        .await
        .expect("legacy calibration restore");
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

    let mz_req = MotorZeroRequest {
        joints: vec![0],
        dry_run: false,
        force: false,
        method: Some("motor".into()),
    };
    let mz_res = motor_zero::run(&rig.sup, &cal_deps, mz_req, &check)
        .await
        .expect("legacy mock motor zero");
    assert_eq!(mz_res["zero_method"], "motor");
    assert_eq!(mz_res["joints"][0]["verified"], true);

    server_jh.abort();
    vision_jh.abort();
    rig.shutdown().await;
}

/// Shared harness for per-op Executor capability refusal tests: seeds on-disk calibration/grasp/
/// poses/gripper files, runs one excluded operation via WS and via direct Rust API, and proves
/// `409 EXECUTOR_UNSUPPORTED` with zero changes on disk, zero changes in live state, and zero RT allocs.
async fn assert_executor_refuses_op_cleanly(
    path: &str,
    ws_body: Value,
    direct_call: impl for<'a> FnOnce(
        &'a Arc<ArmSupervisor>,
        &'a CalibrationDeps,
    ) -> std::pin::Pin<
        Box<dyn std::future::Future<Output = Option<Result<Value, armd::calibration::CalibError>>> + Send + 'a>,
    >,
) {
    let rig = Rig::new(RtOwner::Executor, |_| {}).await;
    let calib_seed = "{\n  \"version\": 1,\n  \"sentinel\": \"untouched\"\n}\n";
    let setup_seed = "{\n  \"table_z_m\": 0.05\n}\n";
    let poses_seed = "{\n  \"joint_names\": [],\n  \"poses\": []\n}\n";
    let grip_seed = "{\n  \"offset_rad\": 0.01\n}\n";
    std::fs::write(&rig.calib_path, calib_seed).unwrap();
    std::fs::write(&rig.setup_path, setup_seed).unwrap();
    std::fs::write(&rig.poses_path, poses_seed).unwrap();
    std::fs::write(&rig.gripper_calib_path, grip_seed).unwrap();

    let (base, server_jh) = spawn_http_router(&rig, None, OperatorAuth::disabled()).await;
    let ws_url = format!("{}/api/v1/ws", base.replace("http://", "ws://"));
    let det_up = armd::api::proxy::DetectorUpstream::parse("http://127.0.0.1:9").unwrap();
    let cal_deps = CalibrationDeps::new(rig.calib_path.clone(), Some(rig.setup_path.clone()), det_up);

    let _detached_guard = rig.sup.inner().write().await;
    let pub_before = rig.sup.published_config().await.unwrap();
    let obs_before = rig.sup.hub().latest().unwrap().obs.clone();

    let expected_msg =
        format!("{path} is not available while the RT executor owns the arm; use the legacy owner");

    // 1. WS call on Executor MUST fail closed with 409 EXECUTOR_UNSUPPORTED.
    let (mut ws, _) = tokio_tungstenite::connect_async(&ws_url).await.unwrap();
    let rej = ws_send_recv(
        &mut ws,
        json!({
            "type": "command",
            "request_id": 1,
            "generation": rig.lease_gen,
            "epoch_id": rig.epoch,
            "path": path,
            "body": ws_body,
        }),
    )
    .await;
    assert_eq!(rej["type"], "command_reject", "{path}: {rej}");
    assert_eq!(rej["status"], 409, "{path}: {rej}");
    assert_eq!(rej["code"], "EXECUTOR_UNSUPPORTED", "{path}: {rej}");
    assert_eq!(rej["message"], expected_msg, "{path}: {rej}");

    // 2. Direct Rust entry point on Executor MUST also fail closed with 409 EXECUTOR_UNSUPPORTED.
    if let Some(res) = direct_call(&rig.sup, &cal_deps).await {
        let err = res.expect_err("direct op on executor must fail closed");
        assert_eq!(err.status, 409, "{path}: {err:?}");
        assert_eq!(err.code, "EXECUTOR_UNSUPPORTED", "{path}: {err:?}");
        assert_eq!(err.message, expected_msg, "{path}: {err:?}");
    }

    // 3. Disk untouched (exact bytes + no backups directory created).
    assert_eq!(std::fs::read_to_string(&rig.calib_path).unwrap(), calib_seed);
    assert_eq!(std::fs::read_to_string(&rig.setup_path).unwrap(), setup_seed);
    assert_eq!(std::fs::read_to_string(&rig.poses_path).unwrap(), poses_seed);
    assert_eq!(std::fs::read_to_string(&rig.gripper_calib_path).unwrap(), grip_seed);
    assert!(
        !cal_deps.backup_dir.exists(),
        "{path}: refused executor op must not create backup directory"
    );

    // 4. Live state untouched + no config hold/reconciliation + 0 RT allocations.
    let pub_after = rig.sup.published_config().await.unwrap();
    let obs_after = rig.sup.hub().latest().unwrap().obs.clone();
    assert_eq!(pub_after.joint_zero_offsets, pub_before.joint_zero_offsets);
    assert_eq!(pub_after.calibration_revision, pub_before.calibration_revision);
    assert_eq!(pub_after.workspace_revision, pub_before.workspace_revision);
    assert_eq!(pub_after.poses.revision, pub_before.poses.revision);
    assert_eq!(pub_after.gripper_calib_revision, pub_before.gripper_calib_revision);
    assert!(!pub_after.calibration_busy);
    assert!(!rig.is_config_inhibited().await);
    assert_eq!(obs_after.positions, obs_before.positions);
    assert_eq!(obs_after.epoch_id, obs_before.epoch_id);
    rig.assert_executor_zero_alloc(&format!("executor refusal of {path}"));

    drop(_detached_guard);
    server_jh.abort();
    rig.shutdown().await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_refuses_calibration_apply_without_side_effects() {
    let valid_q = armd::kinematics::r_to_quat(
        &armd::chain::ArmModel::REBOT_DEV_ARM.cam_mount(false, 0.0, -25.0),
    );
    let body = json!({
        "dry_run": false,
        "camera_mount": { "offset_m": [0.01, -0.02, 0.05], "q_end_cam": valid_q },
        "intrinsics": {
            "fx": 831.5, "fy": 830.0, "cx": 321.0, "cy": 179.0,
            "k1": 0.0, "k2": 0.0, "p1": 0.0, "p2": 0.0, "k3": 0.0,
            "width": 640, "height": 360
        },
        "table_z_m": 0.012
    });
    let req: ApplyRequest = armd::calibration::parse_apply(&body).unwrap();
    assert_executor_refuses_op_cleanly("/api/v1/calibration/apply", body, move |sup, deps| {
        Box::pin(async move {
            let check: armd::calibration::LeaseCheck = Box::new(|_| Ok(()));
            Some(armd::calibration::apply(sup, deps, req, &check).await)
        })
    })
    .await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_refuses_calibration_restore_without_side_effects() {
    let body = json!({ "backup": "2026-01-01T00-00-00Z" });
    let body_c = body.clone();
    assert_executor_refuses_op_cleanly("/api/v1/calibration/restore", body, move |sup, deps| {
        Box::pin(async move {
            let check: armd::calibration::LeaseCheck = Box::new(|_| Ok(()));
            Some(armd::calibration::restore(sup, deps, &body_c, &check).await)
        })
    })
    .await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_refuses_software_joint_zero_without_side_effects() {
    let body = json!({ "joints": [1, 2], "method": "software", "force": true, "dry_run": false });
    let req = MotorZeroRequest {
        joints: vec![0, 1],
        dry_run: false,
        force: true,
        method: Some("software".into()),
    };
    assert_executor_refuses_op_cleanly(motor_zero::PATH, body, move |sup, deps| {
        Box::pin(async move {
            let check: armd::calibration::LeaseCheck = Box::new(|_| Ok(()));
            Some(motor_zero::run(sup, deps, req, &check).await)
        })
    })
    .await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_refuses_hardware_motor_zero_without_side_effects() {
    let body = json!({ "joints": [1], "method": "motor", "dry_run": false });
    let req = MotorZeroRequest {
        joints: vec![0],
        dry_run: false,
        force: false,
        method: Some("motor".into()),
    };
    assert_executor_refuses_op_cleanly(motor_zero::PATH, body, move |sup, deps| {
        Box::pin(async move {
            let check: armd::calibration::LeaseCheck = Box::new(|_| Ok(()));
            Some(motor_zero::run(sup, deps, req, &check).await)
        })
    })
    .await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_refuses_joint_zero_restore_without_side_effects() {
    let body = json!({ "backup": "2026-01-01T00-00-00Z" });
    assert_executor_refuses_op_cleanly(motor_zero::RESTORE_PATH, body, move |sup, deps| {
        Box::pin(async move {
            let check: armd::calibration::LeaseCheck = Box::new(|_| Ok(()));
            Some(motor_zero::restore(sup, deps, "2026-01-01T00-00-00Z".into(), &check).await)
        })
    })
    .await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_refuses_mit_timeout_without_side_effects() {
    assert_executor_refuses_op_cleanly(armd::mit::TIMEOUT_PATH, json!({}), |_, _| {
        Box::pin(async { None })
    })
    .await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_refuses_mit_qualification_without_side_effects() {
    assert_executor_refuses_op_cleanly(
        armd::mit::QUALIFICATION_PATH,
        json!({ "joints": [1] }),
        |sup, deps| {
            Box::pin(async move {
                let check: armd::calibration::LeaseCheck = Box::new(|_| Ok(()));
                Some(armd::mit::store_qualification(sup, deps, CalibrationFile::default(), &check).await)
            })
        },
    )
    .await;
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

    // 4. Out-of-scope calibration/MIT path on Executor MUST fail closed 409 EXECUTOR_UNSUPPORTED.
    let cal_rej = ws_send_recv(
        &mut ws,
        json!({
            "type": "command",
            "request_id": 4,
            "generation": 1,
            "epoch_id": 1,
            "path": armd::mit::TIMEOUT_PATH,
            "body": {}
        }),
    )
    .await;
    assert_eq!(cal_rej["type"], "command_reject");
    assert_eq!(cal_rej["status"], 409);
    assert_eq!(cal_rej["code"], "EXECUTOR_UNSUPPORTED");

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

    // 9c. Maintenance `set_joint_zero_offsets_async` refusal allocates 0 on armd-control.
    let _ = rig.sup.trigger_e_stop().await;
    let handle_ref = rig.sup.executor_handle().unwrap();
    let legacy_err = handle_ref
        .set_joint_zero_offsets_async(vec![0.0; 7])
        .await
        .unwrap()
        .expect_err("executor must refuse set_joint_zero_offsets_async");
    assert!(
        legacy_err.contains("EXECUTOR_UNSUPPORTED"),
        "expected EXECUTOR_UNSUPPORTED, got: {legacy_err}"
    );

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

    // 9e. Automatic gravity-comp exit on lease timeout is split into two standalone tests below:
    //     - `automatic_gravity_comp_exit_default_resting_pose_zero_alloc_executor` (default
    //       `resting_pose` restored, marked #[ignore] while NormalUnload park allocates)
    //     - `automatic_gravity_comp_exit_without_park_zero_alloc_executor` (no park, asserts ZERO)

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

async fn run_automatic_gravity_comp_exit_alloc_test(clear_resting_pose: bool) {
    let mut cfg_gc = ArmConfig::rebot_dm();
    if clear_resting_pose {
        cfg_gc.resting_pose = None;
    }
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
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
#[ignore = "NormalUnload watchdog park allocates on armd-control; fixed by PR M (feat/rt-m-motion)"]
async fn automatic_gravity_comp_exit_default_resting_pose_zero_alloc_executor() {
    run_automatic_gravity_comp_exit_alloc_test(false).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn automatic_gravity_comp_exit_without_park_zero_alloc_executor() {
    run_automatic_gravity_comp_exit_alloc_test(true).await;
}

// ============================================================================
// 11. M3: gripper_zero_check(store_offset=true) routed through common commit path
//         on Legacy (pre-rename VerifyAuthority + dir-sync failure -> Reconciling)
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn zero_check_store_offset_legacy_pre_rename_verify_and_dir_sync_reconcile() {
    // 11a. Pre-rename revocation (E-stop during after_reserve_before_rename on Legacy tick_shared):
    //      fresh owner VerifyAuthority aborts BEFORE rename, file untouched, hold Cleared.
    let dir1 = tempfile::tempdir().unwrap();
    let path1 = dir1.path().join("gripper_calib.json");
    let sup1 = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let id1 = {
        let mut inner = sup1.inner().write().await;
        inner.lease_enabled = false;
        inner.measured_positions = INSPECT.to_vec();
        inner.commanded_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        inner.measured_positions[6] = -0.05;
        inner.commanded_positions[6] = -0.05;
        inner.target_positions[6] = -0.05;
        inner.gripper_calib_path = Some(path1.clone());
        inner.set_mock_gripper_stall_offset(Some(0.05));
        let ep = inner.safety.epoch_id();
        let adm = sup1.capture_admission_stamp();
        let admitted = inner.check_admission(adm).unwrap();
        armd::SupervisorInner::enable_torque(admitted).unwrap();
        let adm2 = sup1.capture_admission_stamp();
        let admitted2 = inner.check_admission(adm2).unwrap();
        armd::SupervisorInner::start_gripper_zero_check(
            admitted2,
            &json!({"threshold_rad": 0.10, "store_offset": true}),
            ep,
        )
        .unwrap()
        .0
    };
    let (at_hook_tx, at_hook_rx) = std::sync::mpsc::sync_channel::<()>(1);
    let (go_tx, go_rx) = std::sync::mpsc::sync_channel::<()>(1);
    let go_rx = Arc::new(std::sync::Mutex::new(go_rx));
    sup1.set_config_commit_test_hooks(ConfigCommitTestHooks {
        after_reserve_before_rename: Some(Arc::new(move || {
            let _ = at_hook_tx.send(());
            let _ = go_rx.lock().unwrap().recv();
        })),
        ..Default::default()
    });
    // Drive legacy ticks until zero_check spawns the deferred commit task and enters the hook.
    let sup1_tick = Arc::clone(&sup1);
    let ticker = tokio::spawn(async move {
        for _ in 0..200 {
            sup1_tick.tick(0.02).await;
            if sup1_tick.inner().read().await.zero_check_persisting.is_some() {
                break;
            }
        }
    });
    tokio::task::spawn_blocking(move || at_hook_rx.recv_timeout(Duration::from_secs(3)).unwrap())
        .await
        .unwrap();
    let _ = ticker.await;
    // Trigger E-stop BEFORE pre-rename VerifyAuthority runs.
    sup1.trigger_e_stop_checked().await.unwrap();
    go_tx.send(()).unwrap();
    let dl = Instant::now() + Duration::from_secs(3);
    loop {
        let done = {
            let inner = sup1.inner().read().await;
            inner.last_action.as_ref().is_some_and(|a| a.id == id1 && !a.is_running())
        };
        if done {
            break;
        }
        assert!(Instant::now() < dl, "legacy zero_check did not terminate after pre-rename E-stop");
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    assert!(!path1.exists(), "Legacy pre-rename VerifyAuthority must abort before rename");
    {
        let inner = sup1.inner().read().await;
        assert!(!inner.core.motion_inhibited_by_config(), "pre-rename abort must leave hold Cleared");
        assert_eq!(inner.gripper_calib_offset, None, "old policy must stay unchanged");
    }

    // 11b. Dir-sync failure on Legacy tick_shared: rename occurs, OLD policy stays active,
    //      hold enters Reconciling, motion stays inhibited!
    let dir2 = tempfile::tempdir().unwrap();
    let path2 = dir2.path().join("gripper_calib.json");
    let sup2 = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let id2 = {
        let mut inner = sup2.inner().write().await;
        inner.lease_enabled = false;
        inner.measured_positions = INSPECT.to_vec();
        inner.commanded_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        inner.measured_positions[6] = -0.05;
        inner.commanded_positions[6] = -0.05;
        inner.target_positions[6] = -0.05;
        inner.gripper_calib_path = Some(path2.clone());
        inner.set_mock_gripper_stall_offset(Some(0.05));
        let ep = inner.safety.epoch_id();
        let adm = sup2.capture_admission_stamp();
        let admitted = inner.check_admission(adm).unwrap();
        armd::SupervisorInner::enable_torque(admitted).unwrap();
        let adm2 = sup2.capture_admission_stamp();
        let admitted2 = inner.check_admission(adm2).unwrap();
        armd::SupervisorInner::start_gripper_zero_check(
            admitted2,
            &json!({"threshold_rad": 0.10, "store_offset": true}),
            ep,
        )
        .unwrap()
        .0
    };
    sup2.set_config_commit_test_hooks(ConfigCommitTestHooks {
        fail_dir_sync_once: Some(Arc::new(AtomicBool::new(true))),
        ..Default::default()
    });
    for _ in 0..200 {
        sup2.tick(0.02).await;
        if sup2.inner().read().await.zero_check_persisting.is_some() {
            break;
        }
    }
    let dl2 = Instant::now() + Duration::from_secs(3);
    loop {
        let done = {
            let inner = sup2.inner().read().await;
            inner.last_action.as_ref().is_some_and(|a| a.id == id2 && !a.is_running())
        };
        if done {
            break;
        }
        assert!(Instant::now() < dl2, "legacy zero_check did not terminate after dir-sync failure");
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    assert!(path2.exists(), "rename happened before dir-sync failure");
    {
        let inner = sup2.inner().read().await;
        let a = inner.last_action.as_ref().unwrap();
        assert_eq!(a.state, "failed", "{a:?}");
        assert_eq!(a.code.as_deref(), Some("PERSIST_FAILED"), "{a:?}");
        assert_eq!(a.durability_confirmed, Some(false), "{a:?}");
        assert_eq!(inner.gripper_calib_offset, None, "old policy must stay active on dir-sync failure");
        assert!(inner.core.motion_inhibited_by_config(), "dir-sync failure must inhibit motion");
        assert_eq!(
            inner.core.config_held(ConfigDomain::GripperCalibration).map(|h| h.state),
            Some(HoldState::Reconciling)
        );
    }
}

// ============================================================================
// 12. M4: Sequences reservation pins replay-step episode IDs against concurrent
//         episode deletion (409 IN_USE while reserved AND after save; both owners)
// ============================================================================

async fn run_sequence_replay_episode_pin_test(owner: RtOwner) {
    let ep_dir = tempfile::tempdir().unwrap();
    let ep_id = "ep_m4_concurrent_pin";
    let ep_id_abort = "ep_m4_abort_unpin";

    let rig = Rig::new(owner, |_| {}).await;
    // Configure EpisodeStore and write two real episodes (catalog + files) BEFORE locking detached inner.
    let seed_episode = |store: &armd::recording::EpisodeStore, id: &str| {
        let sidecar = armd::recording::EpisodeSidecar {
            id: id.into(),
            dataset: "default".into(),
            task: "m4-test".into(),
            source: "sdk".into(),
            sequence_id: None,
            repeat_index: None,
            outcome: "ok".into(),
            recorded_at: "2026-11-24T00:00:00Z".into(),
            duration_s: 0.1,
            frames: 2,
            joint_names: vec![
                "joint_1".into(),
                "joint_2".into(),
                "joint_3".into(),
                "joint_4".into(),
                "joint_5".into(),
                "joint_6".into(),
                "gripper".into(),
            ],
            units: "rad".into(),
            joint_zero: armd::recording::JointZeroStamp::of(&[0.0; 7]),
            profile_id: Some("rebot_b601dm".into()),
            armd_version: "1.6.0".into(),
            fps_nominal: 50.0,
            truncated: false,
            stopped_by: None,
            arm_dof: Some(6),
            gripper_joint: Some(6),
            recovered: None,
            first_state: Some(INSPECT.to_vec()),
            last_state: Some(INSPECT.to_vec()),
            tool_id: None,
            tool_config_digest: None,
            has_gripper_motion: None,
            frames_dropped: None,
        };
        let frames = vec![
            armd::recording::RecordFrame {
                timestamp: 0.0,
                observation_state: INSPECT.to_vec(),
                action: INSPECT.to_vec(),
            },
            armd::recording::RecordFrame {
                timestamp: 0.02,
                observation_state: INSPECT.to_vec(),
                action: INSPECT.to_vec(),
            },
        ];
        store.write_episode(&sidecar, &frames, None).unwrap();
    };

    // Attach the episode store to both supervisor and PublishedConfigHub (shares Arc<Mutex<Catalog>>).
    let store = armd::recording::EpisodeStore::new(Some(ep_dir.path().to_path_buf()));
    seed_episode(&store, ep_id);
    seed_episode(&store, ep_id_abort);
    if owner == RtOwner::Legacy {
        rig.sup.inner().write().await.episodes = store.clone();
    }
    rig.sup.published_config_hub().update(|cfg| {
        cfg.episodes = store.clone();
    });

    let _detached = if owner == RtOwner::Executor {
        Some(rig.sup.inner().write().await)
    } else {
        None
    };

    // 12a. Start sequences/save referencing `ep_id`, pause while the reservation is held
    //      (`after_reserve_before_rename`), and attempt a concurrent episode delete -> 409 IN_USE!
    let (at_hook_tx, at_hook_rx) = std::sync::mpsc::sync_channel::<()>(1);
    let (go_tx, go_rx) = std::sync::mpsc::sync_channel::<()>(1);
    let go_rx = Arc::new(std::sync::Mutex::new(go_rx));
    rig.sup.set_config_commit_test_hooks(ConfigCommitTestHooks {
        after_reserve_before_rename: Some(Arc::new(move || {
            let _ = at_hook_tx.send(());
            let _ = go_rx.lock().unwrap().recv();
        })),
        ..Default::default()
    });

    let pub0 = rig.sup.published_config().await.unwrap();
    let seq_id = "01J00000000000000000000M41".to_string();
    let seq = armd::sequences::Sequence {
        id: seq_id.clone(),
        name: "seq_m4_replay".into(),
        description: String::new(),
        steps: vec![armd::sequences::Step::Replay {
            episode_id: ep_id.into(),
            speed: 0.5,
            gripper: true,
        }],
        created_at: "2026-11-24T00:00:00Z".into(),
        updated_at: "2026-11-24T00:00:00Z".into(),
        rev: 1,
    };
    let next_seq_file = armd::sequences::SequenceFile {
        sequences: vec![seq.clone()],
    };
    let adm = capture_config_admission(&rig.sup, 601, rig.lease_gen, Some(rig.epoch));
    let prop = build_config_proposal(
        &rig.sup,
        ConfigDomain::Sequences,
        pub0.sequences.revision,
        adm,
        rig.lease_gen,
        pub0.workspace_revision,
    );
    let sup_save = Arc::clone(&rig.sup);
    let seq_path = rig.sequences_path.clone();
    let lease_conn = rig.lease_conn.clone();
    let lease_gen = rig.lease_gen;
    let nf_pub = next_seq_file.clone();
    let save_task = tokio::spawn(async move {
        run_config_commit(
            &sup_save,
            ConfigCommitPlan {
                proposal: prop,
                auth: ConfigAuthContext {
                    admission: adm,
                    lease_holder: Some((HolderLabel::new(&lease_conn), lease_gen)),
                },
                precheck: ReservePrecheck::Sequences {
                    steps: Some(seq.steps.clone()),
                    deleting_sequence_id: None,
                    reserved_pins: Some(armd::sequences::ReservedSequencePins {
                        sequence_id: seq.id.clone(),
                        pose_ids: Vec::new(),
                        episode_ids: vec![ep_id.to_string()],
                    }),
                },
                candidate: ConfigCandidate::Sequences {
                    next_file: next_seq_file.clone(),
                },
                write: ConfigWriteOp::from(Some(PendingWrite::new(
                    seq_path,
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
    });

    // Wait until the Sequences reservation is held and replay episode is pinned.
    tokio::task::spawn_blocking(move || at_hook_rx.recv_timeout(Duration::from_secs(3)).unwrap())
        .await
        .unwrap();

    // Concurrent episode delete while reservation is held MUST fail with 409 IN_USE!
    let del_during = rig
        .sup
        .take_episode_for_delete(ep_id)
        .await
        .expect_err("concurrent episode delete during held Sequences reservation must fail IN_USE");
    assert!(
        matches!(&del_during, ArmError::Detailed { status: 409, code, .. } if code == "IN_USE"),
        "expected 409 IN_USE during held reservation, got {del_during:?}"
    );
    assert!(store.contains(ep_id), "referenced episode must still exist in catalog");

    // Concurrent dataset force-delete MUST ALSO fail with 409 IN_USE while reservation is held!
    let ds_del_during = rig
        .sup
        .take_dataset_episodes_for_delete("default")
        .await
        .expect_err("concurrent dataset force delete must fail IN_USE while reservation is held");
    assert!(
        matches!(&ds_del_during, ArmError::Detailed { status: 409, code, .. } if code == "IN_USE"),
        "expected 409 IN_USE on dataset force delete, got {ds_del_during:?}"
    );
    assert!(store.contains(ep_id) && store.contains(ep_id_abort));

    // Allow the sequence save to complete.
    go_tx.send(()).unwrap();
    let out = save_task
        .await
        .unwrap()
        .expect("sequence save should commit cleanly after refusing concurrent delete");
    rig.sup.set_config_commit_test_hooks(ConfigCommitTestHooks::default());
    assert_eq!(out.activated.revision, pub0.sequences.revision + 1);

    // 12b. After the save activates, episode delete MUST STILL be refused with 409 IN_USE!
    let del_after = rig
        .sup
        .take_episode_for_delete(ep_id)
        .await
        .expect_err("episode delete after sequence save must fail IN_USE");
    assert!(
        matches!(&del_after, ArmError::Detailed { status: 409, code, .. } if code == "IN_USE"),
        "expected 409 IN_USE after sequence save, got {del_after:?}"
    );
    assert!(store.contains(ep_id));

    // 12c. If a sequence save aborts BEFORE rename (fail_rename_once), its reservation and
    //      episode pin are released Cleared -> episode delete succeeds, and a subsequent save
    //      referencing the deleted episode fails 422 SEQUENCE_REF_MISSING (no dangling ref)!
    rig.sup.set_config_commit_test_hooks(ConfigCommitTestHooks {
        fail_rename_once: Some(Arc::new(AtomicBool::new(true))),
        ..Default::default()
    });
    let pub1 = rig.sup.published_config().await.unwrap();
    let seq_abort = armd::sequences::Sequence {
        id: "01J00000000000000000000M42".into(),
        name: "seq_m4_abort".into(),
        description: String::new(),
        steps: vec![armd::sequences::Step::Replay {
            episode_id: ep_id_abort.into(),
            speed: 0.5,
            gripper: true,
        }],
        created_at: "2026-11-24T00:00:00Z".into(),
        updated_at: "2026-11-24T00:00:00Z".into(),
        rev: 1,
    };
    let mut abort_file = pub1.sequences.file.clone();
    abort_file.sequences.push(seq_abort.clone());
    let adm2 = capture_config_admission(&rig.sup, 602, rig.lease_gen, Some(rig.epoch));
    let prop2 = build_config_proposal(
        &rig.sup,
        ConfigDomain::Sequences,
        pub1.sequences.revision,
        adm2,
        rig.lease_gen,
        pub1.workspace_revision,
    );
    let abort_err = run_config_commit(
        &rig.sup,
        ConfigCommitPlan {
            proposal: prop2,
            auth: ConfigAuthContext {
                admission: adm2,
                lease_holder: Some((HolderLabel::new(&rig.lease_conn), rig.lease_gen)),
            },
            precheck: ReservePrecheck::Sequences {
                steps: Some(seq_abort.steps.clone()),
                deleting_sequence_id: None,
                reserved_pins: None,
            },
            candidate: ConfigCandidate::Sequences {
                next_file: abort_file.clone(),
            },
            write: ConfigWriteOp::from(Some(PendingWrite::new(
                rig.sequences_path.clone(),
                serde_json::to_string_pretty(&abort_file).unwrap(),
            ))),
            expected_workspace_revision: None,
            expected_poses_revision: None,
        },
        || Ok(()),
        |_, _| {},
    )
    .await
    .unwrap_err();
    rig.sup.set_config_commit_test_hooks(ConfigCommitTestHooks::default());
    assert!(matches!(&abort_err, ArmError::Rejected { status: 500, code, .. } if code == "PERSIST_FAILED"));
    // Pin was released cleanly -> ep_id_abort can now be deleted!
    assert!(rig.sup.take_episode_for_delete(ep_id_abort).await.unwrap());
    assert!(!store.contains(ep_id_abort));

    // Trying to save a sequence referencing the deleted episode fails 422 SEQUENCE_REF_MISSING.
    let adm3 = capture_config_admission(&rig.sup, 603, rig.lease_gen, Some(rig.epoch));
    let prop3 = build_config_proposal(
        &rig.sup,
        ConfigDomain::Sequences,
        pub1.sequences.revision,
        adm3,
        rig.lease_gen,
        pub1.workspace_revision,
    );
    let missing_err = run_config_commit(
        &rig.sup,
        ConfigCommitPlan {
            proposal: prop3,
            auth: ConfigAuthContext {
                admission: adm3,
                lease_holder: Some((HolderLabel::new(&rig.lease_conn), rig.lease_gen)),
            },
            precheck: ReservePrecheck::Sequences {
                steps: Some(seq_abort.steps.clone()),
                deleting_sequence_id: None,
                reserved_pins: None,
            },
            candidate: ConfigCandidate::Sequences {
                next_file: abort_file.clone(),
            },
            write: ConfigWriteOp::from(Some(PendingWrite::new(
                rig.sequences_path.clone(),
                serde_json::to_string_pretty(&abort_file).unwrap(),
            ))),
            expected_workspace_revision: None,
            expected_poses_revision: None,
        },
        || Ok(()),
        |_, _| {},
    )
    .await
    .unwrap_err();
    assert!(
        matches!(&missing_err, ArmError::Detailed { status: 422, code, .. } if code == "SEQUENCE_REF_MISSING"),
        "expected 422 SEQUENCE_REF_MISSING for deleted episode, got {missing_err:?}"
    );
    assert!(!rig.is_config_inhibited().await);

    rig.assert_executor_zero_alloc("M4 sequence replay episode reservation and concurrent delete");
    drop(_detached);
    rig.shutdown().await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn sequence_save_reservation_pins_replay_episodes_against_concurrent_delete_legacy() {
    run_sequence_replay_episode_pin_test(RtOwner::Legacy).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn sequence_save_reservation_pins_replay_episodes_against_concurrent_delete_executor() {
    run_sequence_replay_episode_pin_test(RtOwner::Executor).await;
}

// ============================================================================
// 13. Item 4: Watchdog does NOT start a park during config hold or reconciliation
// ============================================================================

async fn setup_torqued_watchdog_rig(owner: RtOwner, lease_timeout: Duration) -> Rig {
    let dir = tempfile::tempdir().unwrap();
    let keepout_path = dir.path().join("keepout.json");
    let collision_path = dir.path().join("collision.json");
    let gripper_calib_path = dir.path().join("gripper_calib.json");
    let poses_path = dir.path().join("poses.json");
    let sequences_path = dir.path().join("sequences.json");
    let setup_path = dir.path().join("grasp_setup.json");
    let commissioning_path = dir.path().join("tool_commissioning.json");
    let calib_path = dir.path().join("calibration.json");

    let cfg = ArmConfig::rebot_dm();
    assert!(
        cfg.resting_pose.is_some(),
        "test requires default resting_pose so an unblocked watchdog would park"
    );
    let sup = Arc::new(ArmSupervisor::new(cfg));
    armd::keepout::load_into(&sup, &keepout_path).await.unwrap();
    armd::collision::load_into(&sup, &collision_path).await;
    armd::gripper_calib::load_into(&sup, &gripper_calib_path).await;
    armd::poses::load_into(&sup, &poses_path).await;
    armd::sequences::load_into(&sup, &sequences_path).await;
    armd::grasp::load_into(&sup, &setup_path).await;
    armd::tool::load_commissioning_into(&sup, &commissioning_path).await;

    let lease_conn = "rt-c-wd-conn".to_string();
    let closed_conn_id = 7777;
    let (lease_gen, epoch) = {
        let mut inner = sup.inner().write().await;
        inner.lease_timeout = lease_timeout;
        inner.measured_positions = INSPECT.to_vec();
        inner.commanded_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        inner.mark_connection_closed(closed_conn_id);
        let lgen = inner.acquire_lease(&lease_conn).expect("acquire initial lease");
        let adm = inner.current_admission_stamp();
        let admitted = inner.check_admission(adm).unwrap();
        armd::SupervisorInner::enable_torque(admitted).unwrap();
        (lgen, inner.safety.epoch_id())
    };

    if owner == RtOwner::Executor {
        let mut ecfg = ExecutorConfig::for_tests(Duration::from_millis(2));
        ecfg.alloc_timing = true;
        let handle = sup.start_executor(ecfg).await.expect("start RT executor");
        let deadline = Instant::now() + Duration::from_secs(3);
        while sup.hub().latest().is_none() {
            assert!(Instant::now() < deadline, "timed out waiting for StateHub snapshot");
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);
    }

    Rig {
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

async fn run_watchdog_no_park_during_config_reconciliation_test(owner: RtOwner) {
    // 13a. Keep-out rename + parent-dir fsync failure -> Reconciling + lease expiry on torqued arm
    //      with default `resting_pose` intact -> NO park motion, holds INSPECT, torque stays ON,
    //      observation.watchdog_park_blocked_reason explains why.
    let rig = setup_torqued_watchdog_rig(owner, Duration::from_millis(60)).await;

    let _detached = if owner == RtOwner::Executor {
        Some(rig.sup.inner().write().await)
    } else {
        None
    };

    rig.sup.set_config_commit_test_hooks(ConfigCommitTestHooks {
        fail_dir_sync_once: Some(Arc::new(AtomicBool::new(true))),
        ..Default::default()
    });
    let pub0 = rig.sup.published_config().await.unwrap();
    let boxes = vec![KeepoutBox {
        name: "reconcile-wd-box".into(),
        min_m: [0.40, 0.40, 0.40],
        max_m: [0.55, 0.55, 0.55],
    }];
    let (plan, _, _, _) = keepout_plan(&rig, &pub0, boxes, pub0.keepout.revision, 701, rig.lease_gen, false);
    let err = run_config_commit(&rig.sup, plan, || Ok(()), |_, _| {})
        .await
        .unwrap_err();
    rig.sup.set_config_commit_test_hooks(ConfigCommitTestHooks::default());
    assert!(matches!(&err, ArmError::Rejected { status: 500, code, .. } if code == "PERSIST_FAILED"));
    assert!(rig.keepout_path.exists(), "keep-out file must have been renamed before dir-sync failed");
    assert_eq!(rig.hold_state(ConfigDomain::Keepout).await, Some(HoldState::Reconciling));
    assert!(rig.is_config_inhibited().await);

    // Wait past lease_timeout (60 ms) and drive ticks (Legacy) / read hub (Executor).
    tokio::time::sleep(Duration::from_millis(80)).await;
    let obs = if owner == RtOwner::Legacy {
        let mut last = rig.sup.inner().write().await.tick(0.02);
        for _ in 0..10 {
            last = rig.sup.inner().write().await.tick(0.02);
        }
        last
    } else {
        let deadline = Instant::now() + Duration::from_secs(2);
        loop {
            if let Some(s) = rig.sup.hub().latest() {
                if matches!(
                    s.obs.safety_state,
                    armd::safety::SafetyState::Fault(armd::safety::FaultReason::WatchdogTimeout)
                ) {
                    break s.obs.clone();
                }
            }
            assert!(Instant::now() < deadline, "timed out waiting for WatchdogTimeout during reconciliation");
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    };

    assert!(
        matches!(
            obs.safety_state,
            armd::safety::SafetyState::Fault(armd::safety::FaultReason::WatchdogTimeout)
        ),
        "watchdog must latch WatchdogTimeout without parking, got {:?}",
        obs.safety_state
    );
    assert!(obs.is_torqued, "torque must remain enabled while holding during reconciliation");
    assert_eq!(
        obs.watchdog_park_blocked_reason.as_deref(),
        Some("configuration reconciliation active"),
        "observation must report why watchdog park was blocked"
    );
    assert!(
        obs.action.as_ref().is_none_or(|a| a.kind != "park"),
        "watchdog must NOT start a park action during reconciliation: {:?}",
        obs.action
    );
    for (i, (&p, &exp)) in obs.positions.iter().zip(INSPECT.iter()).enumerate() {
        assert!(
            (p - exp).abs() < 1e-4,
            "joint {i} moved away from INSPECT ({p} vs {exp}) during reconciliation"
        );
    }
    for (i, (&c, &exp)) in obs.commanded_positions.iter().zip(INSPECT.iter()).enumerate() {
        assert!(
            (c - exp).abs() < 1e-4,
            "commanded joint {i} moved away from INSPECT ({c} vs {exp}) during reconciliation"
        );
    }

    // Wait another 80 ms to confirm the arm continues holding position with torque ON.
    tokio::time::sleep(Duration::from_millis(80)).await;
    let obs2 = if owner == RtOwner::Legacy {
        let mut inner = rig.sup.inner().write().await;
        assert!(!inner.pending_normal_unload);
        assert!(inner.watchdog_park.is_none());
        inner.tick(0.02)
    } else {
        rig.sup.hub().latest().unwrap().obs.clone()
    };
    assert!(obs2.is_torqued);
    assert_eq!(
        obs2.watchdog_park_blocked_reason.as_deref(),
        Some("configuration reconciliation active")
    );
    for (i, (&p, &exp)) in obs2.positions.iter().zip(INSPECT.iter()).enumerate() {
        assert!((p - exp).abs() < 1e-4, "joint {i} drifted after hold ({p} vs {exp})");
    }

    rig.assert_executor_zero_alloc("watchdog hold during config reconciliation");
    drop(_detached);
    rig.shutdown().await;

    // 13b. Active pre-rename configuration hold + lease expiry on torqued arm -> NO park motion,
    //      observation reports "configuration hold active".
    let rig_hold = setup_torqued_watchdog_rig(owner, Duration::from_millis(60)).await;
    let _detached_hold = if owner == RtOwner::Executor {
        Some(rig_hold.sup.inner().write().await)
    } else {
        None
    };

    let (at_hook_tx, at_hook_rx) = std::sync::mpsc::sync_channel::<()>(1);
    let (go_tx, go_rx) = std::sync::mpsc::sync_channel::<()>(1);
    let go_rx = Arc::new(std::sync::Mutex::new(go_rx));
    rig_hold.sup.set_config_commit_test_hooks(ConfigCommitTestHooks {
        after_reserve_before_rename: Some(Arc::new(move || {
            let _ = at_hook_tx.send(());
            let _ = go_rx.lock().unwrap().recv();
        })),
        ..Default::default()
    });
    let pub_h = rig_hold.sup.published_config().await.unwrap();
    let boxes_h = vec![KeepoutBox {
        name: "hold-wd-box".into(),
        min_m: [0.40, 0.40, 0.40],
        max_m: [0.55, 0.55, 0.55],
    }];
    let (plan_h, _, _, _) =
        keepout_plan(&rig_hold, &pub_h, boxes_h, pub_h.keepout.revision, 702, rig_hold.lease_gen, false);
    let sup_h = Arc::clone(&rig_hold.sup);
    let commit_task = tokio::spawn(async move {
        run_config_commit(&sup_h, plan_h, || Ok(()), |_, _| {}).await
    });
    tokio::task::spawn_blocking(move || at_hook_rx.recv_timeout(Duration::from_secs(3)).unwrap())
        .await
        .unwrap();

    // Expire lease while the Keepout hold is active (before rename).
    tokio::time::sleep(Duration::from_millis(80)).await;
    let obs_h = if owner == RtOwner::Legacy {
        let mut inner = rig_hold.sup.inner().write().await;
        let o = inner.tick(0.02);
        assert!(!inner.pending_normal_unload && inner.watchdog_park.is_none());
        o
    } else {
        let deadline = Instant::now() + Duration::from_secs(2);
        loop {
            if let Some(s) = rig_hold.sup.hub().latest() {
                if matches!(
                    s.obs.safety_state,
                    armd::safety::SafetyState::Fault(armd::safety::FaultReason::WatchdogTimeout)
                ) {
                    break s.obs.clone();
                }
            }
            assert!(Instant::now() < deadline, "timed out waiting for WatchdogTimeout during config hold");
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    };
    assert!(obs_h.is_torqued);
    assert_eq!(
        obs_h.watchdog_park_blocked_reason.as_deref(),
        Some("configuration hold active")
    );
    for (i, (&p, &exp)) in obs_h.positions.iter().zip(INSPECT.iter()).enumerate() {
        assert!((p - exp).abs() < 1e-4, "joint {i} moved during config hold ({p} vs {exp})");
    }

    // Release commit hook; commit aborts cleanly before rename (epoch/lease changed on watchdog).
    let _ = go_tx.send(());
    let _ = commit_task.await.unwrap();
    rig_hold.sup.set_config_commit_test_hooks(ConfigCommitTestHooks::default());
    rig_hold.assert_executor_zero_alloc("watchdog hold during active config hold");
    drop(_detached_hold);
    rig_hold.shutdown().await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn watchdog_holds_without_parking_during_config_reconciliation_legacy() {
    run_watchdog_no_park_during_config_reconciliation_test(RtOwner::Legacy).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn watchdog_holds_without_parking_during_config_reconciliation_executor() {
    run_watchdog_no_park_during_config_reconciliation_test(RtOwner::Executor).await;
}

// ============================================================================
// 14. Round 4 M3 — Public RtExecutorHandle refuses excluded zero setters
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_public_handle_refuses_joint_zero_and_motor_zero_with_executor_unsupported() {
    let rig = Rig::new(RtOwner::Executor, |_| {}).await;
    let handle = rig
        .sup
        .executor_handle()
        .expect("RT executor handle must be present in OWNER_EXECUTOR");

    let pub_before = rig.sup.published_config().await.unwrap();
    let obs_before = rig
        .sup
        .hub()
        .latest()
        .expect("executor published observation")
        .obs
        .clone();
    let dof = pub_before.config.dof;
    assert_eq!(pub_before.joint_zero_offsets.len(), dof);

    // Arm is Idle with torque off — the exact state where the old setters mutated live state.
    let err_sync_jz = handle
        .set_joint_zero_offsets(vec![0.15; dof])
        .expect("maintenance channel healthy")
        .expect_err("public handle set_joint_zero_offsets must fail closed");
    assert!(
        err_sync_jz.contains("EXECUTOR_UNSUPPORTED"),
        "expected EXECUTOR_UNSUPPORTED, got: {err_sync_jz}"
    );

    let err_async_jz = handle
        .set_joint_zero_offsets_async(vec![-0.2; dof])
        .await
        .expect("maintenance channel healthy")
        .expect_err("public handle set_joint_zero_offsets_async must fail closed");
    assert!(
        err_async_jz.contains("EXECUTOR_UNSUPPORTED"),
        "expected EXECUTOR_UNSUPPORTED, got: {err_async_jz}"
    );

    let err_sync_mz = handle
        .set_motor_zero(0)
        .expect("maintenance channel healthy")
        .expect_err("public handle set_motor_zero must fail closed");
    assert!(
        err_sync_mz.contains("EXECUTOR_UNSUPPORTED"),
        "expected EXECUTOR_UNSUPPORTED, got: {err_sync_mz}"
    );

    let err_async_mz = handle
        .set_motor_zero_async(1)
        .await
        .expect("maintenance channel healthy")
        .expect_err("public handle set_motor_zero_async must fail closed");
    assert!(
        err_async_mz.contains("EXECUTOR_UNSUPPORTED"),
        "expected EXECUTOR_UNSUPPORTED, got: {err_async_mz}"
    );

    // Neither live positions/offsets nor PublishedConfig nor disk files changed.
    let pub_after = rig.sup.published_config().await.unwrap();
    let obs_after = rig
        .sup
        .hub()
        .latest()
        .expect("executor published observation")
        .obs
        .clone();
    assert_eq!(pub_after.joint_zero_offsets, pub_before.joint_zero_offsets);
    assert_eq!(pub_after.calibration_revision, pub_before.calibration_revision);
    assert_eq!(pub_after.workspace_revision, pub_before.workspace_revision);
    assert_eq!(obs_after.positions, obs_before.positions);
    assert_eq!(obs_after.commanded_positions, obs_before.commanded_positions);
    assert!(!rig.calib_path.exists(), "calibration.json must not be created");
    assert!(!rig.poses_path.exists(), "poses.json must not be created");

    rig.assert_executor_zero_alloc("public RtExecutorHandle zero refusals");

    // Stopped handle still returns OwnerError::Stopped.
    let stop_handle = handle.clone();
    tokio::task::spawn_blocking(move || stop_handle.stop_and_join())
        .await
        .unwrap();
    assert!(matches!(
        handle.set_joint_zero_offsets(vec![0.0; dof]),
        Err(armd::rt_core::executor::OwnerError::Stopped)
    ));
    assert!(matches!(
        handle.set_motor_zero(0),
        Err(armd::rt_core::executor::OwnerError::Stopped)
    ));
    let _ = rig;
}
