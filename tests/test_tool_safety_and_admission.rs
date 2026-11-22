//! Integration tests for tool safety & admission fixes:
//! B1, B3, B4, B5, B6, B7, B9, B11, M1, M2, M3, M16.

mod common;

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use serde_json::{json, Value};

use armd::backlash::{self, DescentRules, Pre};
use armd::calibration::{self, CalibrationFile};
use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointMode, JointTarget};
use armd::grasp::{self, GraspSetup};
use armd::kinematics::{self, Pose};
use armd::model::{ArmConfig, GripperPolicy, JointLimit, JointState};
use armd::motion::{GoalSpec, GripperCommand, GripperMotion, GripperRequest, Speed, StepCtx, StepStatus};
use armd::poses::{NamedPose, PoseList, PoseTcp};
use armd::recording::{EpisodeMetadata, EpisodeSidecar, RecordFrame};
use armd::replay::{self, ReplayInput, ReplayRequest};
use armd::tool::{self, ActiveToolInfo, ToolPackConfig};
use armd::{ArmError, ArmSupervisor, SupervisorInner};
use common::INSPECT;

const FINRAY_PACK_JSON: &str = r#"{
  "id": "soft_finray_gripper",
  "name": "Soft FinRay Parallel Gripper",
  "kind": "gripper",
  "version": "1.0.0",
  "robots": ["b601dm"],
  "attach_body": "end_link",
  "tcp_offset": [0.0288, 0.0, 0.0],
  "armd": {
    "extra_check_points_end_m": [
      [0.032,  0.082,  0.022],
      [0.032,  0.082, -0.022],
      [0.032, -0.082,  0.022],
      [0.032, -0.082, -0.022]
    ],
    "physical_contact_end_m": [0.0288, 0.0, 0.0],
    "contact_opening_m": 0.0036,
    "dynamics": { "mass_delta_kg": 0.035, "com_moment_delta_nm": 0.018 },
    "gripper_policy": {
      "jaw_opening_nominal": [[-5.7, 0.0886], [0.0, 0.0036]],
      "holding_gap_rad": 0.30,
      "holding_torque_nm": 0.0667,
      "contact_torque_nm": 0.6,
      "squeeze_rad": 0.15,
      "max_close_torque_nm": 1.0
    },
    "grasp": { "max_grasp_width_m": 0.075, "grasp_height_m": 0.015, "edge_to_low_m": null }
  }
}"#;

const OTHER_TOOL_PACK_JSON: &str = r#"{
  "id": "custom_tool_b",
  "name": "Custom Tool B",
  "kind": "gripper",
  "version": "1.0.0",
  "robots": ["b601dm"],
  "attach_body": "end_link",
  "tcp_offset": [0.045, 0.0, 0.0],
  "armd": {
    "extra_check_points_end_m": [[0.045, 0.0, 0.0]],
    "physical_contact_end_m": [0.045, 0.0, 0.0],
    "contact_opening_m": 0.002,
    "dynamics": { "mass_delta_kg": 0.025, "com_moment_delta_nm": 0.012 },
    "gripper_policy": {
      "jaw_opening_nominal": [[-5.7, 0.085], [0.0, 0.002]],
      "holding_gap_rad": 0.30,
      "holding_torque_nm": 0.0667,
      "contact_torque_nm": 0.6,
      "squeeze_rad": 0.15,
      "max_close_torque_nm": 1.0
    },
    "grasp": { "max_grasp_width_m": 0.075, "grasp_height_m": 0.015, "edge_to_low_m": 0.001 }
  }
}"#;

fn finray_cfg() -> ArmConfig {
    let base = ArmConfig::rebot_dm();
    let pack: ToolPackConfig = serde_json::from_str(FINRAY_PACK_JSON).unwrap();
    tool::apply_tool_overlay(&base, &pack).unwrap()
}

fn other_tool_cfg() -> ArmConfig {
    let base = ArmConfig::rebot_dm();
    let pack: ToolPackConfig = serde_json::from_str(OTHER_TOOL_PACK_JSON).unwrap();
    tool::apply_tool_overlay(&base, &pack).unwrap()
}

#[derive(Clone)]
struct MockFbArm {
    state: Arc<Mutex<MockFbState>>,
    torque_feedback: bool,
}

struct MockFbState {
    q: [f32; 7],
    vel: [f32; 7],
    torques: [f32; 7],
    torqued: bool,
    feedback_valid: bool,
    feedback_age: Duration,
    rx_instants: Option<Vec<Option<Instant>>>,
    fail_read: bool,
    clear_fault_calls: usize,
    clear_disables: Vec<usize>,
    hw_fault: bool,
}

impl MockFbArm {
    fn new(q: [f32; 7], torque_feedback: bool) -> Self {
        Self {
            torque_feedback,
            state: Arc::new(Mutex::new(MockFbState {
                q,
                vel: [0.0; 7],
                torques: [0.0; 7],
                torqued: true,
                feedback_valid: true,
                feedback_age: Duration::from_millis(5),
                rx_instants: None,
                fail_read: false,
                clear_fault_calls: 0,
                clear_disables: Vec::new(),
                hw_fault: false,
            })),
        }
    }
}

impl ArmDriver for MockFbArm {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            velocity_feedback: true,
            torque_feedback: self.torque_feedback,
            temperature_feedback: true,
            torque_feedforward: false,
            enable_feedback: true,
            load_feedback: false,
        }
    }
    fn enable_state(&self) -> EnableState {
        if self.state.lock().unwrap().torqued { EnableState::Enabled } else { EnableState::Disabled }
    }
    fn is_feedback_valid(&self) -> bool {
        let s = self.state.lock().unwrap();
        s.feedback_valid && s.feedback_age <= Duration::from_millis(250)
    }
    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        let s = self.state.lock().unwrap();
        vec![Some(s.feedback_age); 7]
    }
    fn feedback_receive_instants(&self) -> Vec<Option<Instant>> {
        self.state.lock().unwrap().rx_instants.clone().unwrap_or_default()
    }
    fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
        self.state.lock().unwrap().torqued = enable;
        Ok(())
    }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let s = self.state.lock().unwrap();
        if s.fail_read {
            return Err(ArmDriverError::Timeout(1));
        }
        Ok((0..7).map(|i| JointState { position: s.q[i], velocity: s.vel[i], torque: s.torques[i], temperature: 30.0 }).collect())
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        let mut s = self.state.lock().unwrap();
        for (i, t) in targets.iter().enumerate().take(7) { s.q[i] = t.position; }
        Ok(())
    }
    fn set_joint_modes(&mut self, _modes: &[JointMode], _hold: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
    fn check_hardware_status(&self) -> Result<(), ArmDriverError> {
        if self.state.lock().unwrap().hw_fault {
            Err(ArmDriverError::HardwareFault("latched hw fault".into()))
        } else {
            Ok(())
        }
    }
    fn clear_fault_disables_joints(&self) -> Vec<usize> {
        self.state.lock().unwrap().clear_disables.clone()
    }
    fn clear_fault(&mut self) -> Result<(), ArmDriverError> {
        let mut s = self.state.lock().unwrap();
        s.clear_fault_calls += 1;
        s.hw_fault = false;
        s.torqued = false;
        Ok(())
    }
}

// =========================================================================
// B1: Commissioning admission & persistence
// =========================================================================
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn b1_commissioning_admission_gates_ordinary_motions_and_persists_confirm() {
    let dir = tempfile::tempdir().unwrap();
    let comm_path = dir.path().join("tool_commissioning.json");
    let calib_path = dir.path().join("gripper_calib.json");

    let sup = Arc::new(ArmSupervisor::new(finray_cfg()));
    {
        let mut inner = sup.inner().write().await;
        inner.tool_commissioning_path = Some(comm_path.clone());
        inner.gripper_calib_path = Some(calib_path.clone());
    }
    let (mut ws, gen, epoch, _sup, addr) = common::torqued_rig(sup, &INSPECT).await;
    let base = format!("http://{addr}");
    let client = reqwest::Client::new();

    // 1. Commissioning required initially
    let st: Value = client.get(format!("{base}/api/v1/tool/commissioning")).send().await.unwrap().json().await.unwrap();
    assert_eq!(st["commissioning_required"], true);
    let expected_digest = st["active_tool"]["config_digest"].as_str().unwrap().to_string();

    // 2. Ordinary motions rejected with COMMISSIONING_REQUIRED
    for (path, body) in [
        ("/api/v1/motion/home", json!({})),
        ("/api/v1/action/move_to_pose", json!({"name": "scan"})),
        ("/api/v1/action/place", json!({"position": [0.25, 0.0, 0.15]})),
        ("/api/v1/action/grasp", json!({"box_norm": [0.4, 0.4, 0.6, 0.6]})),
        ("/api/v1/action/replay", json!({"episode": "ep1"})),
        ("/api/v1/action/run_sequence", json!({"name": "seq1"})),
        ("/api/v1/action/move_to", json!({"position": [0.25, 0.0, 0.15], "speed_scale": 0.5})),
        ("/api/v1/action/move_relative", json!({"frame": "base", "dx": 0.01, "dy": 0.0, "dz": 0.0, "speed_scale": 0.5})),
        ("/api/v1/motion/jog", json!({"target_positions": INSPECT, "speed_scale": 0.5})),
        ("/api/v1/action/gripper", json!({"command": "open", "speed": 0.8})),
    ] {
        let rej = common::rpc(&mut ws, gen, epoch, path, body).await;
        assert_eq!(rej["type"], "command_reject", "{path}: {rej}");
        assert_eq!(rej["code"], "COMMISSIONING_REQUIRED", "{path}: {rej}");
    }

    // Servo frame rejected during commissioning
    common::servo(&mut ws, gen, epoch, &INSPECT).await;
    let srej = common::servo_reject_within(&mut ws, 300).await.expect("servo_reject");
    assert_eq!(srej["code"], "COMMISSIONING_REQUIRED", "{srej}");

    // 3. Saving gripper_calib is allowed AND does NOT clear commissioning_required
    let ack_gc = common::rpc(&mut ws, gen, epoch, "/api/v1/gripper/calibration", json!({
        "jaw_opening": {"points": [[0.0, 0.0036], [-5.7, 0.0886]], "calibrated": true}
    })).await;
    assert_eq!(ack_gc["type"], "command_ack", "{ack_gc}");
    let st2: Value = client.get(format!("{base}/api/v1/tool/commissioning")).send().await.unwrap().json().await.unwrap();
    assert_eq!(st2["commissioning_required"], true);

    // 4. Low-speed debug moves (<= 0.2), controlled gripper (<= 0.4 / speed_scale <= 0.2), stop, unload are allowed
    let ack_jog = common::rpc(&mut ws, gen, epoch, "/api/v1/motion/jog", json!({"target_positions": INSPECT, "speed_scale": 0.2})).await;
    assert_eq!(ack_jog["type"], "command_ack", "{ack_jog}");
    let ack_stop0 = common::rpc(&mut ws, gen, epoch, "/api/v1/motion/stop", json!({})).await;
    assert_eq!(ack_stop0["type"], "command_ack", "{ack_stop0}");
    let ack_grip = common::rpc(&mut ws, gen, epoch, "/api/v1/action/gripper", json!({"command": "open", "speed": 0.3})).await;
    assert_eq!(ack_grip["type"], "command_ack", "{ack_grip}");
    let ack_stop = common::rpc(&mut ws, gen, epoch, "/api/v1/motion/stop", json!({})).await;
    assert_eq!(ack_stop["type"], "command_ack", "{ack_stop}");

    // 5. Explicit commissioning confirmation requires confirm: true, matching tool_id and tool_config_digest
    let resp = client
        .post(format!("{base}/api/v1/tool/commissioning/confirm"))
        .json(&json!({
            "confirm": true,
            "tool_id": "soft_finray_gripper",
            "tool_config_digest": expected_digest,
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(resp.status(), 200);
    let body: Value = resp.json().await.unwrap();
    assert_eq!(body["commissioning_required"], false);
    assert!(comm_path.exists());

    // After confirm, home is admitted
    let ack_home = common::rpc(&mut ws, gen, epoch, "/api/v1/motion/home", json!({})).await;
    assert_eq!(ack_home["type"], "command_ack", "{ack_home}");

    // 6. Fresh supervisor with exact same tool reloads commissioning_required == false
    let reloaded = ArmSupervisor::new(finray_cfg());
    tool::load_commissioning_into(&reloaded, &comm_path).await;
    {
        let inner = reloaded.inner().read().await;
        assert_eq!(inner.config.active_tool.config_digest, expected_digest);
        assert!(!inner.config.active_tool.commissioning_required);
    }

    // 7. Fresh supervisor with DIFFERENT tool stays commissioning_required == true
    let swapped = ArmSupervisor::new(other_tool_cfg());
    tool::load_commissioning_into(&swapped, &comm_path).await;
    assert!(swapped.inner().read().await.config.active_tool.commissioning_required);
}

// =========================================================================
// B3: Hard close-torque cap requires torque_feedback and fresh telemetry
// =========================================================================
#[test]
fn b3_close_torque_cap_refuses_missing_torque_feedback_and_stops_on_stale_feedback() {
    // 1. start_gripper / start_macro_gripper / zero_check refuse closing when torque_feedback == false
    let mut cfg = ArmConfig::rebot_dm();
    cfg.gripper_policy.as_mut().unwrap().max_close_torque_nm = Some(1.0);
    let drv = MockFbArm::new(INSPECT, false);
    let mut inner = SupervisorInner::new(cfg);
    inner.driver = Some(Mutex::new(Box::new(drv)));
    inner.lease_enabled = false;
    inner.set_torque(true).unwrap();
    inner.commanded_positions = vec![0.0, -1.167, -1.565, 1.435, 0.0, 0.0, -3.0];
    inner.measured_positions = inner.commanded_positions.clone();
    inner.target_positions = inner.commanded_positions.clone();
    inner.tick(0.02);
    let ep0 = inner.safety.epoch_id();

    let close_req = GripperRequest { command: GripperCommand::Close, fraction: None, grasp: true, speed: 1.0, timeout_s: Some(5.0), tag: None };
    assert!(inner.start_gripper(close_req.clone(), ep0).is_err(), "closing must reject without torque_feedback");
    let open_req = GripperRequest { command: GripperCommand::Open, fraction: None, grasp: false, speed: 1.0, timeout_s: Some(5.0), tag: None };
    assert!(inner.start_gripper(open_req, ep0).is_ok(), "opening remains allowed");

    // 2. GripperMotion::step stops immediately on !feedback_fresh when max_close_torque_nm is set
    let mut policy = GripperPolicy::rebot_dm();
    policy.max_close_torque_nm = Some(1.0);
    let lim = JointLimit::new("j7", -6.0, 0.5, 2.0, 5.0);
    let mut gm = GripperMotion::new(close_req, policy, lim, 1.0, 6);
    let meas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -2.5];
    let zeros = [0.0; 7];
    let out = gm.step(&StepCtx {
        dt: 0.02,
        time_scale: 1.0,
        measured: &meas,
        measured_velocities: &zeros,
        commanded: &meas,
        commanded_velocities: &zeros,
        torques: &zeros,
        loads: &zeros,
        feedback_fresh: false,
        torque_feedback: true,
        load_feedback: false,
        gripper_stalled: false,
        gripper_stall_sign: 1.0,
        gripper_force_capped: false,
    });
    assert!(matches!(out.status, StepStatus::Failed { code: "FEEDBACK_UNAVAILABLE", .. }), "{:?}", out.status);
    assert!((out.positions[6] - (-2.5)).abs() < 1e-6);

    // 3. Streaming servo close latches force-cap hold when feedback becomes stale
    let mut cfg2 = ArmConfig::rebot_dm();
    cfg2.gripper_policy.as_mut().unwrap().max_close_torque_nm = Some(1.0);
    let q0 = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0, -3.0];
    let drv2 = MockFbArm::new(q0, true);
    let mut inner2 = SupervisorInner::new(cfg2);
    inner2.driver = Some(Mutex::new(Box::new(drv2.clone())));
    inner2.lease_enabled = false;
    inner2.set_torque(true).unwrap();
    inner2.commanded_positions = q0.to_vec();
    inner2.measured_positions = q0.to_vec();
    inner2.target_positions = q0.to_vec();

    // Stream closing servo target (closed_rad = 0.0 > -3.0), then lose freshness
    let mut tgt = q0.to_vec();
    tgt[6] = -0.2;
    let ep = inner2.safety.epoch_id();
    inner2.servo(tgt, ep).unwrap();
    drv2.state.lock().unwrap().feedback_age = Duration::from_millis(500);
    inner2.tick(0.02);
    assert!(inner2.gripper_force_capped);
    assert!(inner2.gripper_stalled);
    assert!(inner2.commanded_positions[6] <= -2.99, "must freeze instead of closing without torque feedback: {}", inner2.commanded_positions[6]);
}

// =========================================================================
// B4: Calibration precedence across tool swaps
// =========================================================================
#[tokio::test]
async fn b4_calibration_precedence_isolates_tool_records_and_preserves_camera_kinematics() {
    let dir = tempfile::tempdir().unwrap();
    let gc_path = dir.path().join("gripper_calib.json");
    let gs_path = dir.path().join("grasp_setup.json");

    // Persist finray-stamped gripper_calib and grasp_setup
    let fin = finray_cfg();
    let fin_sup = ArmSupervisor::new(fin.clone());
    {
        let mut inner = fin_sup.inner().write().await;
        inner.gripper_calib_path = Some(gc_path.clone());
        let prep_gc = armd::gripper_calib::prepare_update(&mut inner, &json!({
            "jaw_opening": {"points": [[0.0, 0.0036], [-5.7, 0.0886]], "calibrated": true},
            "offset_rad": 0.01
        })).unwrap();
        if let Some(w) = prep_gc.write.as_ref() { std::fs::write(&w.path, &w.text).unwrap(); }
        let prep_gs = grasp::prepare_setup_update(&inner, &json!({"table_z_m": 0.042}), Some(&gs_path), false).unwrap();
        if let Some(w) = prep_gs.write.as_ref() { std::fs::write(&w.path, &w.text).unwrap(); }
    }

    // Load under a different tool -> ignored without mutating defaults
    let other = other_tool_cfg();
    let other_sup = ArmSupervisor::new(other.clone());
    armd::gripper_calib::load_into(&other_sup, &gc_path).await;
    grasp::load_into(&other_sup, &gs_path).await;
    {
        let inner = other_sup.inner().read().await;
        assert!(inner.gripper_calib_jaw_opening.is_none());
        assert!(inner.gripper_calib_offset.is_none());
        assert_eq!(inner.grasp_setup().unwrap().table_z_m, 0.0);
    }

    // CalibrationFile with camera_mount + intrinsics + tool + acceptance + hover table_z
    let cal: CalibrationFile = serde_json::from_value(json!({
        "camera_mount": {"offset_m": [-0.07, 0.0, 0.06], "q_end_cam": [1.0, 0.0, 0.0, 0.0]},
        "intrinsics": {"fx": 830.0},
        "table_z_m": 0.024,
        "table_z_source": "hover",
        "tool": {"tool_id": "soft_finray_gripper", "tool_config_digest": fin.active_tool.config_digest, "x_fingerprint": "xf", "tip_cam_m": [0.0, 0.0, 0.1], "measured_at": "2026-11-22T00:00:00Z"},
        "acceptance": {"fingerprint": "fp-1", "gate_version": 1, "passed": true, "tool_id": "soft_finray_gripper", "tool_config_digest": fin.active_tool.config_digest, "measured_at": "2026-11-22T00:00:00Z", "board": {}, "points": [], "summary": {"table_z_m": 0.024}}
    })).unwrap();
    assert!(calibration::tool_valid_for(&cal, "xf", &fin.active_tool));
    assert!(!calibration::tool_valid_for(&cal, "xf", &other.active_tool));
    assert!(!calibration::acceptance_valid_for(&cal, "fp-1", &other.active_tool));
    let (eff, src, _) = calibration::effective_floor_for(0.0, Some(&cal), "fp-1", &other.active_tool);
    assert_eq!((eff, src), (0.0, "profile"));

    let mut stripped = cal.clone();
    calibration::strip_mismatched_tool_records(&mut stripped, &other.active_tool);
    assert!(stripped.tool.is_none() && stripped.acceptance.is_none() && stripped.table_z_source.is_none());
    assert!(stripped.camera_mount.offset_m.is_some() && stripped.intrinsics.is_some(), "camera calibration stays intact across tool swap");

    // Legacy unkeyed setup applies ONLY on stock, NOT on non-stock
    std::fs::write(&gs_path, r#"{"table_z_m": 0.033}"#).unwrap();
    let (s_stock, out_stock) = grasp::load_setup_for_tool(&gs_path, &GraspSetup::default(), &ActiveToolInfo::default());
    assert_eq!((s_stock.table_z_m, matches!(out_stock, grasp::LoadOutcome::Loaded)), (0.033, true));
    let (s_non, out_non) = grasp::load_setup_for_tool(&gs_path, &GraspSetup::default(), &fin.active_tool);
    assert_eq!(s_non.table_z_m, 0.0);
    assert!(matches!(out_non, grasp::LoadOutcome::Invalid(_)));
}

// =========================================================================
// B5 & B6: Gripper-bearing replay provenance & TCP reach envelope
// =========================================================================
#[tokio::test]
async fn b5_b6_replay_checks_tool_provenance_and_tcp_reach_envelope() {
    let dir = tempfile::tempdir().unwrap();
    let fin = finray_cfg();
    let other = other_tool_cfg();

    // 1. Episode with gripper motion under finray
    let ep_grip = dir.path().join("ep_grip.parquet");
    let mut frames_g = Vec::new();
    for k in 0..20 {
        let mut q = INSPECT;
        q[6] = -2.0 + 1.5 * (k as f32 / 19.0);
        frames_g.push(RecordFrame { timestamp: k as f64 * 0.02, observation_state: q.to_vec(), action: q.to_vec() });
    }
    let meta_g = EpisodeMetadata {
        profile_id: fin.profile_id.clone(),
        arm_dof: Some(6),
        gripper_joint: Some(6),
        joint_names: Some(fin.limits.iter().map(|l| l.name.clone()).collect()),
        tool_id: Some(fin.active_tool.id.clone()),
        tool_config_digest: Some(fin.active_tool.config_digest.clone()),
        has_gripper_motion: Some(true),
    };
    armd::recording::write_frames_to_parquet_with_meta(&ep_grip, &frames_g, Some(&meta_g)).unwrap();

    // Standalone replay under other tool: gripper:true -> TOOL_MISMATCH, gripper:false -> Ok
    let make_input = |cfg: &ArmConfig, path: std::path::PathBuf, episode: &str, gripper: bool| {
        let mut inner = common::sim_at(cfg.clone(), &INSPECT);
        inner.config.active_tool.commissioning_required = false;
        ReplayInput {
            request: ReplayRequest { episode: episode.into(), speed: 1.0, gripper, tag: None, allow_stale: false },
            path,
            profile_id: cfg.profile_id.clone(),
            dof: cfg.dof,
            arm_dof: cfg.arm_dof(),
            gripper_idx: cfg.gripper_index(),
            joint_names: cfg.limits.iter().map(|l| l.name.clone()).collect(),
            limits: cfg.limits.clone(),
            speed_limits: cfg.limits.clone(),
            ws: cfg.workspace.clone().map(|w| (w, cfg.camera_mount.clone())),
            arm: cfg.arm_model.clone(),
            start: INSPECT.to_vec(),
            control_hz: 50,
            epoch: 0,
            workspace_revision: 0,
            power: None,
            store: armd::recording::EpisodeStore::new(Some(dir.path().to_path_buf())),
            zero_offsets: vec![0.0; 6],
            tcp_offset: cfg.tcp_offset,
            active_tool: cfg.active_tool.clone(),
        }
    };
    let err = replay::plan(make_input(&other, ep_grip.clone(), "ep_grip", true)).unwrap_err();
    assert!(matches!(err, ArmError::Rejected { ref code, .. } if code == "TOOL_MISMATCH"), "{err:?}");
    assert!(replay::plan(make_input(&other, ep_grip, "ep_grip", false)).is_ok());

    // Sequence replay step with mismatched gripper-bearing episode -> TOOL_MISMATCH
    let mut seq_sup = common::sim_at(other.clone(), &INSPECT);
    seq_sup.config.active_tool.commissioning_required = false;
    seq_sup.episodes = armd::recording::EpisodeStore::new(Some(dir.path().to_path_buf()));
    let sc_g = EpisodeSidecar {
        id: "ep_grip_seq".into(),
        dataset: "default".into(),
        task: "t".into(),
        source: "sdk".into(),
        sequence_id: None,
        repeat_index: None,
        outcome: "ok".into(),
        recorded_at: "2026-11-22T00:00:00Z".into(),
        duration_s: 0.4,
        frames: frames_g.len(),
        joint_names: other.limits.iter().map(|l| l.name.clone()).collect(),
        units: "rad".into(),
        joint_zero: armd::recording::JointZeroStamp::of(&seq_sup.joint_zero_offsets),
        profile_id: other.profile_id.clone(),
        armd_version: "test".into(),
        fps_nominal: 50.0,
        truncated: false,
        stopped_by: None,
        arm_dof: Some(6),
        gripper_joint: Some(6),
        tool_id: Some(fin.active_tool.id.clone()),
        tool_config_digest: Some(fin.active_tool.config_digest.clone()),
        has_gripper_motion: Some(true),
        recovered: None,
        first_state: Some(frames_g[0].observation_state.clone()),
        last_state: Some(frames_g.last().unwrap().observation_state.clone()),
    };
    seq_sup.episodes.write_episode(&sc_g, &frames_g, None).unwrap();
    let save_seq = armd::sequences::save(&mut seq_sup, &json!({
        "name": "s_mismatch",
        "steps": [{"type": "replay", "episode_id": "ep_grip_seq", "gripper": true}]
    })).unwrap();
    let seq_err = armd::sequences::prepare_run(&seq_sup, &json!({"id": save_seq.0.id}), seq_sup.safety.epoch_id()).unwrap_err();
    assert!(matches!(seq_err, ArmError::Rejected { ref code, .. } if code == "TOOL_MISMATCH"), "{seq_err:?}");

    // 2. B6: EndLink inside max_reach_m (0.53 m) while extended TCP (0.06 m) crosses max_reach_m (0.53 m)
    let mut ext_cfg = fin.clone();
    ext_cfg.tcp_offset = [0.06, 0.0, 0.0];
    ext_cfg.workspace.as_mut().unwrap().max_reach_m = Some(0.53);
    ext_cfg.active_tool.tcp_offset_m = [0.06, 0.0, 0.0];
    ext_cfg.active_tool.commissioning_required = false;

    // Solve an IK pose with EndLink at radius 0.475 m pointing radially +X
    let r_rad = kinematics::rpy_to_r(0.0, 0.0, 0.0);
    let sol = ext_cfg.arm_model.solve_dls(
        &Pose::new(r_rad, [0.475, 0.0, 0.18]),
        &[0.0, 0.2, -0.5, 0.0, 0.0, 0.0],
        &std::array::from_fn(|i| ext_cfg.limits[i].min_position as f64),
        &std::array::from_fn(|i| ext_cfg.limits[i].max_position as f64),
        kinematics::PAR.w,
    );
    assert!(sol.ok, "reachable test pose");
    let end_p = ext_cfg.arm_model.fk(&sol.q).p;
    let end_r = kinematics::norm(&end_p);
    let tcp_r = kinematics::norm(&armd::motion::end_to_tcp(&ext_cfg.arm_model.fk(&sol.q), &ext_cfg.tcp_offset).p);
    assert!(end_r < 0.53 && tcp_r > 0.53, "end_r={end_r:.3}, tcp_r={tcp_r:.3}");
    let q_far: Vec<f32> = sol.q.iter().map(|&v| v as f32).chain([0.0]).collect();

    let ep_far = dir.path().join("ep_far.parquet");
    let frames_far: Vec<RecordFrame> = (0..100).map(|k| {
        let f = k as f32 / 99.0;
        let q: Vec<f32> = INSPECT.iter().zip(&q_far).map(|(a, b)| a + (b - a) * f).collect();
        RecordFrame { timestamp: k as f64 * 0.1, observation_state: q.clone(), action: q }
    }).collect();
    let meta_far = EpisodeMetadata {
        profile_id: ext_cfg.profile_id.clone(),
        arm_dof: Some(6),
        gripper_joint: Some(6),
        joint_names: Some(ext_cfg.limits.iter().map(|l| l.name.clone()).collect()),
        tool_id: Some(ext_cfg.active_tool.id.clone()),
        tool_config_digest: Some(ext_cfg.active_tool.config_digest.clone()),
        has_gripper_motion: Some(false),
    };
    armd::recording::write_frames_to_parquet_with_meta(&ep_far, &frames_far, Some(&meta_far)).unwrap();

    let reach_err = replay::plan(make_input(&ext_cfg, ep_far.clone(), "ep_far", false)).unwrap_err();
    assert!(matches!(reach_err, ArmError::WorkspaceRejected { .. }), "{reach_err:?}");

    // Also via sequence Step::Replay executing through ArmSupervisor (uses active tcp_offset)
    let seq_sup_arc = std::sync::Arc::new(ArmSupervisor::new(ext_cfg.clone()));
    let (action_id, epoch) = {
        let mut inner = seq_sup_arc.inner().write().await;
        inner.lease_enabled = false;
        inner.set_torque(true).unwrap();
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        inner.episodes = armd::recording::EpisodeStore::new(Some(dir.path().to_path_buf()));
        let sc_far = EpisodeSidecar {
            id: "ep_far_seq".into(),
            dataset: "default".into(),
            task: "t".into(),
            source: "sdk".into(),
            sequence_id: None,
            repeat_index: None,
            outcome: "ok".into(),
            recorded_at: "2026-11-22T00:00:00Z".into(),
            duration_s: 9.9,
            frames: frames_far.len(),
            joint_names: ext_cfg.limits.iter().map(|l| l.name.clone()).collect(),
            units: "rad".into(),
            joint_zero: armd::recording::JointZeroStamp::of(&inner.joint_zero_offsets),
            profile_id: ext_cfg.profile_id.clone(),
            armd_version: "test".into(),
            fps_nominal: 10.0,
            truncated: false,
            stopped_by: None,
            arm_dof: Some(6),
            gripper_joint: Some(6),
            tool_id: Some(ext_cfg.active_tool.id.clone()),
            tool_config_digest: Some(ext_cfg.active_tool.config_digest.clone()),
            has_gripper_motion: Some(false),
            recovered: None,
            first_state: Some(frames_far[0].observation_state.clone()),
            last_state: Some(frames_far.last().unwrap().observation_state.clone()),
        };
        inner.episodes.write_episode(&sc_far, &frames_far, None).unwrap();
        let save_far = armd::sequences::save(&mut inner, &json!({
            "name": "s_far",
            "steps": [{"type": "replay", "episode_id": "ep_far_seq", "gripper": false}]
        })).unwrap();
        let epoch = inner.safety.epoch_id();
        // Direct replay_input check proves ReplayInput carries ext_cfg.tcp_offset
        let seq_input = inner.replay_input(ReplayRequest {
            episode: "ep_far_seq".into(),
            speed: 0.5,
            gripper: false,
            tag: None,
            allow_stale: false,
        }, epoch).unwrap();
        let seq_res = replay::plan(seq_input);
        assert!(matches!(seq_res, Err(ArmError::WorkspaceRejected { .. })), "got {seq_res:?}");

        let run_plan = armd::sequences::prepare_run(&inner, &json!({"id": save_far.0.id}), epoch).unwrap();
        let (id, _) = armd::sequences::begin(&mut inner, &seq_sup_arc, run_plan).unwrap();
        (id, epoch)
    };
    let mut seq_done = None;
    for _ in 0..100 {
        if let Some(a) = seq_sup_arc.inner().read().await.last_action.clone().filter(|a| a.id == action_id && !a.is_running()) {
            seq_done = Some(a);
            break;
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    let _ = epoch;
    let seq_done = seq_done.expect("sequence replay should terminate with failure");
    assert_eq!(seq_done.state, "failed", "{seq_done:?}");
    assert_eq!(seq_done.code.as_deref(), Some("WORKSPACE_VIOLATION"), "{seq_done:?}");
}

// =========================================================================
// B7: Actual grasp & place descent rejects table intrusion without unchecked fallback
// =========================================================================
#[test]
fn b7_install_macro_path_and_place_plan_reject_table_intrusion_without_unchecked_fallback() {
    let mut cfg = finray_cfg();
    cfg.active_tool.commissioning_required = false;
    let mut inner = common::sim_at(cfg, &INSPECT);
    let input = inner.plan_input().unwrap();
    let p0 = input.arm.fk(&input.q0).p;
    let goal = armd::motion::resolve_goal(&input, &GoalSpec::Absolute { p: [p0[0], p0[1], p0[2] - 0.04], r: None });
    let speed = Speed { linear: 0.03, angular: armd::motion::DEFAULT_ANGULAR_RPS };
    let direct = armd::motion::plan_cartesian(&input, goal, speed).unwrap();
    let id = inner.begin_macro("grasp", &input, None).unwrap();

    // Extended corner at [0.032, 0.082, -0.022] dips below tight table bound -> must fail WORKSPACE_VIOLATION, not install direct
    let end_pose = input.arm.fk_f32(direct.last_slice()).unwrap();
    let corner_z = end_pose.transform_point(&[0.032, 0.082, -0.022])[2];
    let rules = DescentRules {
        from_above: vec![[0.0288, 0.0, 0.0]],
        table: Some((vec![[0.0, 0.0, 0.0], [0.032, 0.082, -0.022]], corner_z + 0.002)),
    };
    let plan = inner.approach_plan("grasp_final", backlash::Request::Profile).unwrap();
    let pre = backlash::plan_cartesian_pre(&input, &direct, &plan, Some(&rules), speed, false);
    assert!(matches!(pre, Pre::Direct(_)));
    let err = inner.install_macro_path_same_side_with_rules(id, &input, direct, pre, Some(&plan), Some(&rules)).unwrap_err();
    assert!(matches!(err, ArmError::Motion(armd::motion::MotionError::WorkspaceViolation { .. })), "{err:?}");
    assert!(inner.active_trajectory.is_none(), "rejected fallback must not install trajectory");

    // place::plan also rejects when descend_path violates table_clearance
    let mut inner_p = common::sim_at(ArmConfig::rebot_dm(), &INSPECT);
    inner_p.set_grasp_setup(GraspSetup { table_z_m: 0.14, ..GraspSetup::default() });
    let pending = armd::place::prepare(&inner_p, &json!({"position": [p0[0], p0[1], 0.142]}), inner_p.safety.epoch_id()).unwrap();
    let place_err = armd::place::plan(pending).unwrap_err();
    assert!(matches!(place_err, ArmError::Motion(armd::motion::MotionError::WorkspaceViolation { .. })), "{place_err:?}");
}

// =========================================================================
// B9: Manual table capture requires fresh feedback and settled stillness
// =========================================================================
#[test]
fn b9_table_z_from_tcp_rejects_stale_or_moving_arm_without_changing_revision_or_file() {
    let dir = tempfile::tempdir().unwrap();
    let gs_path = dir.path().join("grasp_setup.json");

    let mut cfg = finray_cfg();
    cfg.active_tool.commissioning_required = false;
    // Vertical pose
    let q_seed = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0];
    let target_r = kinematics::rpy_to_r(0.0, std::f64::consts::FRAC_PI_2, 0.0);
    let p_end = cfg.arm_model.fk(&q_seed).p;
    let (lo, hi): ([f64; 6], [f64; 6]) = (
        std::array::from_fn(|i| cfg.limits[i].min_position as f64),
        std::array::from_fn(|i| cfg.limits[i].max_position as f64),
    );
    let sol = cfg.arm_model.solve_dls(&Pose::new(target_r, p_end), &q_seed, &lo, &hi, kinematics::PAR.w);
    assert!(sol.ok);
    let q_vert: [f32; 7] = std::array::from_fn(|i| if i < 6 { sol.q[i] as f32 } else { 0.0 });

    let drv = MockFbArm::new(q_vert, true);
    let mut inner = SupervisorInner::new(cfg);
    inner.driver = Some(Mutex::new(Box::new(drv.clone())));
    inner.lease_enabled = false;
    inner.set_torque(true).unwrap();
    inner.commanded_positions = q_vert.to_vec();
    inner.measured_positions = q_vert.to_vec();
    inner.target_positions = q_vert.to_vec();
    let rev0 = inner.grasp_setup_revision;

    // 1. Before settling 0.2s -> rejected, revision & disk unchanged
    let e_unsettled = grasp::prepare_setup_update(&inner, &json!({"table_z_from_tcp": true}), Some(&gs_path), false).unwrap_err();
    assert!(format!("{e_unsettled:?}").contains("still"), "{e_unsettled:?}");
    assert_eq!(inner.grasp_setup_revision, rev0);
    assert!(!gs_path.exists());

    // 2. Tick 15 times (0.30s) with advancing hardware receive timestamps -> succeeds
    let mut t_rx = Instant::now();
    for _ in 0..15 {
        t_rx += Duration::from_millis(20);
        drv.state.lock().unwrap().rx_instants = Some(vec![Some(t_rx); 7]);
        inner.tick(0.02);
    }
    assert!(grasp::prepare_setup_update(&inner, &json!({"table_z_from_tcp": true}), Some(&gs_path), false).is_ok());

    // 3. Stale feedback -> rejected
    drv.state.lock().unwrap().feedback_age = Duration::from_millis(500);
    let e_stale = grasp::prepare_setup_update(&inner, &json!({"table_z_from_tcp": true}), Some(&gs_path), false).unwrap_err();
    assert!(format!("{e_stale:?}").contains("fresh"), "{e_stale:?}");
    assert_eq!(inner.grasp_setup_revision, rev0);
    assert!(!gs_path.exists());

    // 4. Moving measured velocity -> rejected
    drv.state.lock().unwrap().feedback_age = Duration::from_millis(5);
    drv.state.lock().unwrap().vel[0] = 0.08;
    inner.tick(0.02);
    let e_mov = grasp::prepare_setup_update(&inner, &json!({"table_z_from_tcp": true}), Some(&gs_path), false).unwrap_err();
    assert!(format!("{e_mov:?}").contains("0.02"), "{e_mov:?}");
    assert_eq!(inner.grasp_setup_revision, rev0);
    assert!(!gs_path.exists());
}

// =========================================================================
// B11: Measured park endpoint violating workspace blocks torque release
// =========================================================================
#[test]
fn b11_park_retains_torque_when_measured_endpoint_violates_workspace_within_joint_tolerance() {
    let mut cfg = ArmConfig::rebot_dm();
    let shoulder = cfg.arm_model.shoulder_rz();
    let rest_p = cfg.arm_model.fk_f32(&cfg.home_positions).unwrap().p;
    let rest_reach = kinematics::shoulder_reach_from(&rest_p, &shoulder);
    // Nominal home is valid inside max_reach_m = rest_reach + 0.001 m
    cfg.workspace.as_mut().unwrap().max_reach_m = Some((rest_reach + 0.001) as f32);

    // Perturb J2/J3 within resting_pose.position_tolerances (0.08 rad) so measured endpoint exceeds max_reach_m + WORKSPACE_EPS
    let q_meas = [(-0.07f32, -0.07f32), (-0.07, 0.07), (0.07, -0.07), (0.07, 0.07)]
        .into_iter()
        .map(|(d1, d2)| {
            let mut q = cfg.home_positions.clone();
            q[1] += d1;
            q[2] += d2;
            q
        })
        .max_by(|a, b| {
            let ra = kinematics::shoulder_reach_from(&cfg.arm_model.fk_f32(a).unwrap().p, &shoulder);
            let rb = kinematics::shoulder_reach_from(&cfg.arm_model.fk_f32(b).unwrap().p, &shoulder);
            ra.total_cmp(&rb)
        })
        .unwrap();
    let meas_p = cfg.arm_model.fk_f32(&q_meas).unwrap().p;
    let meas_reach = kinematics::shoulder_reach_from(&meas_p, &shoulder);
    assert!(meas_reach > rest_reach + 0.004, "meas_reach={meas_reach:.4}, rest_reach={rest_reach:.4}");

    let mut inner = SupervisorInner::new(cfg);
    inner.lease_enabled = false;
    inner.set_torque(true).unwrap();
    inner.commanded_positions = inner.config.home_positions.clone();
    inner.target_positions = inner.config.home_positions.clone();
    inner.measured_positions = inner.config.home_positions.clone();
    inner.measured_velocities = vec![0.0; 7];
    assert!(inner.resting_pose_static_violation().is_none(), "nominal home must be valid in workspace");

    inner.measured_positions = q_meas;
    let v = inner.resting_pose_static_violation().expect("must fail on measured workspace violation");
    assert!(v.contains("max_reach"), "expected violation naming max_reach, got: {v}");
    assert!(inner.set_torque(false).is_err(), "set_torque(false) must refuse to drop torque while measured pose violates workspace");
    assert!(inner.is_torqued, "holding torque must remain enabled");
}

// =========================================================================
// M1: Hover job resume binds tool, candidate, and rechecks floor
// =========================================================================
#[test]
fn m1_calib_hover_resume_rejects_tool_mismatch_candidate_mismatch_and_floor_violation() {
    let mut cfg = finray_cfg();
    cfg.active_tool.commissioning_required = false;
    let mut inner = common::sim_at(cfg, &INSPECT);

    let body = json!({
        "candidate_fingerprint": "fp-A",
        "rpy_rad": [0.0, 1.5708, 0.0],
        "points": [
            {"id": "p1", "approach_m": [0.25, 0.0, 0.14], "hover_m": [0.25, 0.0, 0.08]},
            {"id": "p2", "approach_m": [0.25, 0.05, 0.14], "hover_m": [0.25, 0.05, 0.08]}
        ]
    });
    let mut plan = armd::calib_hover::prepare(&inner, &body, inner.safety.epoch_id()).unwrap();
    let job_id = plan.job.id.clone();
    plan.job.points[0].state = "done".into();
    plan.job.state = "failed".into();
    inner.hover_jobs.push(plan.job);

    // 1. Mismatched candidate_fingerprint -> FINGERPRINT_MISMATCH
    let err_fp = armd::calib_hover::prepare(&inner, &json!({"resume": job_id, "candidate_fingerprint": "fp-B"}), inner.safety.epoch_id()).unwrap_err();
    assert!(matches!(err_fp, ArmError::Detailed { ref code, .. } if code == "FINGERPRINT_MISMATCH"), "{err_fp:?}");

    // 2. Updated floor above pending point p2 (0.08 < floor 0.075 + 0.008) -> WORKSPACE_VIOLATION
    inner.config.workspace.as_mut().unwrap().floor_z_m = 0.075;
    let err_fl = armd::calib_hover::prepare(&inner, &json!({"resume": job_id, "candidate_fingerprint": "fp-A"}), inner.safety.epoch_id()).unwrap_err();
    assert!(matches!(err_fl, ArmError::Detailed { ref code, .. } if code == "WORKSPACE_VIOLATION"), "{err_fl:?}");
    inner.config.workspace.as_mut().unwrap().floor_z_m = 0.0;

    // 3. Tool swap -> TOOL_MISMATCH
    inner.config.active_tool = other_tool_cfg().active_tool;
    let err_tl = armd::calib_hover::prepare(&inner, &json!({"resume": job_id, "candidate_fingerprint": "fp-A"}), inner.safety.epoch_id()).unwrap_err();
    assert!(matches!(err_tl, ArmError::Detailed { ref code, .. } if code == "TOOL_MISMATCH"), "{err_tl:?}");
}

// =========================================================================
// M2: Post-edit grasp setup rejects max_grasp_width_m > reachable open width
// =========================================================================
#[test]
fn m2_grasp_setup_edit_rejects_max_grasp_width_above_jaw_reach() {
    let inner = common::sim_at(ArmConfig::rebot_dm(), &INSPECT);
    let err = grasp::prepare_setup_update(&inner, &json!({"max_grasp_width_m": 0.12}), None, false).unwrap_err();
    assert!(format!("{err:?}").contains("max_grasp_width_m"), "{err:?}");
    assert!(grasp::prepare_setup_update(&inner, &json!({"max_grasp_width_m": 0.070}), None, false).is_ok());
}

// =========================================================================
// M3: Measurement wire frames emit distinct EndLink end_pose and TCP tcp
// =========================================================================
#[tokio::test]
async fn m3_depth_measure_emits_distinct_end_pose_and_tcp() {
    let mut cfg = finray_cfg();
    cfg.active_tool.commissioning_required = false;
    let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
    {
        let mut inner = sup.inner().write().await;
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
    }
    #[derive(Clone)]
    struct DummyDet;
    impl armd::grasp::DetectionSource for DummyDet {
        fn inference_enabled(&self) -> armd::grasp::BoxFuture<'_, Result<bool, String>> {
            Box::pin(async { Ok(true) })
        }
        fn set_inference(&self, _enabled: bool) -> armd::grasp::BoxFuture<'_, Result<(), String>> {
            Box::pin(async { Ok(()) })
        }
        fn next_frame(&self, _after: Option<u64>, _w: u64) -> armd::grasp::BoxFuture<'_, Result<Option<armd::grasp::DetectionFrame>, String>> {
            Box::pin(async { Ok(None) })
        }
        fn trigger(&self, _p: &'static [&'static str], _w: u64) -> armd::grasp::BoxFuture<'_, Result<Option<armd::grasp::DetectionFrame>, String>> {
            Box::pin(async {
                Ok(Some(armd::grasp::DetectionFrame {
                    frame_id: 1,
                    captured_at: Instant::now(),
                    policy_captured_at: None,
                    image_size_px: [640.0, 360.0],
                    detections: vec![],
                    depth: None,
                    model_generation: None,
                }))
            })
        }
        fn per_request_pipelines(&self) -> armd::grasp::BoxFuture<'_, Result<bool, String>> {
            Box::pin(async { Ok(true) })
        }
    }
    let out = armd::depth_measure::measure(&sup, &DummyDet, &json!({})).await.unwrap();
    let end_xyz: [f64; 3] = serde_json::from_value(out["camera"]["end_pose"]["xyz_m"].clone()).unwrap();
    let tcp_xyz: [f64; 3] = serde_json::from_value(out["camera"]["tcp"]["xyz_m"].clone()).unwrap();
    let expected_end = cfg.arm_model.fk_f32(&INSPECT).unwrap();
    let expected_tcp = armd::motion::end_to_tcp(&expected_end, &cfg.tcp_offset);
    assert!(kinematics::norm(&kinematics::sub(&end_xyz, &expected_end.p)) < 1e-6);
    assert!(kinematics::norm(&kinematics::sub(&tcp_xyz, &expected_tcp.p)) < 1e-6);
    assert!(kinematics::norm(&kinematics::sub(&tcp_xyz, &end_xyz)) > 0.025);
}

// =========================================================================
// M16: Saved-pose display unconditionally recomputes or clears stale cache
// =========================================================================
#[tokio::test]
async fn m16_saved_pose_display_recomputes_uncached_and_clears_incompatible_cache() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("poses.json");
    let stale_cache = Some(PoseTcp { position: [0.99, 0.99, 0.99], rpy: [0.1, 0.2, 0.3] });
    let list = PoseList {
        poses: vec![
            NamedPose { id: "id1".into(), name: "valid_no_cache".into(), positions: INSPECT.to_vec(), tcp: None, saved_at_ms: 1, ..Default::default() },
        ],
        ..Default::default()
    };
    std::fs::write(&path, serde_json::to_string(&list).unwrap()).unwrap();

    let mut cfg = finray_cfg();
    cfg.active_tool.commissioning_required = false;
    let sup = ArmSupervisor::new(cfg.clone());
    armd::poses::load_into(&sup, &path).await;
    let mut inner = sup.inner().write().await;
    let expected_tcp = cfg.tcp_pose_f32(&INSPECT).unwrap();
    let got_tcp = inner.poses.list.poses[0].tcp.as_ref().expect("valid joints must recompute TCP even when tcp was None");
    assert!(kinematics::norm(&kinematics::sub(&got_tcp.position, &expected_tcp.p)) < 1e-6);

    // Also verify refresh_pose_tcps clears stale cached TCP when positions are invalid or arm_dof is incompatible
    inner.poses.list.poses.push(NamedPose { id: "id2".into(), name: "empty_stale".into(), positions: vec![], tcp: stale_cache.clone(), saved_at_ms: 1, ..Default::default() });
    inner.poses.list.poses.push(NamedPose { id: "id3".into(), name: "wrong_dof_stale".into(), positions: INSPECT.to_vec(), tcp: stale_cache, saved_at_ms: 1, arm_dof: Some(5), ..Default::default() });
    inner.refresh_pose_tcps();
    assert!(inner.poses.list.poses[1].tcp.is_none(), "empty positions must clear stale cached TCP");
    assert!(inner.poses.list.poses[2].tcp.is_none(), "incompatible arm_dof must clear stale cached TCP");
}

// =========================================================================
// R2 Negative & Regression Tests (B04/m01, B05, B06, B07, B08/M14, B09, B11, M05, M06, M12)
// =========================================================================

#[test]
fn r2_b04_m01_clear_fault_and_torque_off_enforce_measured_workspace_envelope() {
    let mut cfg = ArmConfig::rebot_dm();
    let shoulder = cfg.arm_model.shoulder_rz();
    let rest_p = cfg.arm_model.fk_f32(&cfg.home_positions).unwrap().p;
    let rest_reach = kinematics::shoulder_reach_from(&rest_p, &shoulder);
    cfg.workspace.as_mut().unwrap().max_reach_m = Some((rest_reach + 0.001) as f32);

    // Joint perturbation within resting tolerances yet violating max_reach_m
    let mut q_bad = cfg.home_positions.clone();
    q_bad[1] += 0.07;
    q_bad[2] -= 0.07;
    let q_bad_arr: [f32; 7] = q_bad.clone().try_into().unwrap();
    assert!(kinematics::shoulder_reach_from(&cfg.arm_model.fk_f32(&q_bad).unwrap().p, &shoulder) > rest_reach + 0.002);

    let drv = MockFbArm::new(q_bad_arr, true);
    {
        let mut s = drv.state.lock().unwrap();
        s.hw_fault = true;
        s.clear_disables = vec![0, 1, 2, 3, 4, 5];
    }
    let mut inner = SupervisorInner::new(cfg);
    inner.driver = Some(Mutex::new(Box::new(drv.clone())));
    inner.lease_enabled = false;
    inner.set_torque(true).unwrap();

    // m01: set_torque(false) returns ArmNotAtRestingPose carrying workspace constraint + geometry point
    let off_err = inner.set_torque(false).unwrap_err();
    let off_msg = off_err.to_string();
    assert!(off_msg.contains("max_reach") && off_msg.contains("point 0"), "got: {off_msg}");

    // B04: trigger fault, clear_fault must block BEFORE calling drv.clear_fault()
    inner.safety.trigger_fault_coded(armd::FaultCode::HardwareFault, armd::FaultReason::Other("latched".into()));
    let cf_err = inner.clear_fault().unwrap_err();
    assert!(cf_err.to_string().contains("max_reach"), "{cf_err}");
    assert_eq!(drv.state.lock().unwrap().clear_fault_calls, 0, "driver reset must never run on workspace violation");
}

#[test]
fn r2_b06_table_z_from_tcp_rejects_cached_or_gapped_hw_timestamps() {
    let mut cfg = finray_cfg();
    cfg.active_tool.commissioning_required = false;
    let q_seed = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0];
    let target_r = kinematics::rpy_to_r(0.0, std::f64::consts::FRAC_PI_2, 0.0);
    let p_end = cfg.arm_model.fk(&q_seed).p;
    let (lo, hi): ([f64; 6], [f64; 6]) = (
        std::array::from_fn(|i| cfg.limits[i].min_position as f64),
        std::array::from_fn(|i| cfg.limits[i].max_position as f64),
    );
    let sol = cfg.arm_model.solve_dls(&Pose::new(target_r, p_end), &q_seed, &lo, &hi, kinematics::PAR.w);
    let q_vert: [f32; 7] = std::array::from_fn(|i| if i < 6 { sol.q[i] as f32 } else { 0.0 });

    let drv = MockFbArm::new(q_vert, true);
    let fixed_rx = Instant::now();
    drv.state.lock().unwrap().rx_instants = Some(vec![Some(fixed_rx); 7]);

    let mut inner = SupervisorInner::new(cfg);
    inner.driver = Some(Mutex::new(Box::new(drv.clone())));
    inner.lease_enabled = false;
    inner.set_torque(true).unwrap();
    inner.commanded_positions = q_vert.to_vec();
    inner.measured_positions = q_vert.to_vec();
    inner.target_positions = q_vert.to_vec();

    // 1. Constant hardware receive timestamps across ticks -> only 1 distinct sample -> rejected
    for _ in 0..20 {
        inner.tick(0.02);
    }
    let err_dup = grasp::prepare_setup_update(&inner, &json!({"table_z_from_tcp": true}), None, false).unwrap_err();
    assert!(format!("{err_dup:?}").contains("still"), "{err_dup:?}");

    // 2. Advancing hardware receive timestamps accumulate -> succeeds, then read error clears window
    let mut t_rx = fixed_rx;
    for _ in 0..15 {
        t_rx += Duration::from_millis(20);
        drv.state.lock().unwrap().rx_instants = Some(vec![Some(t_rx); 7]);
        inner.tick(0.02);
    }
    assert!(grasp::prepare_setup_update(&inner, &json!({"table_z_from_tcp": true}), None, false).is_ok());

    drv.state.lock().unwrap().fail_read = true;
    inner.tick(0.02);
    drv.state.lock().unwrap().fail_read = false;
    t_rx += Duration::from_millis(20);
    drv.state.lock().unwrap().rx_instants = Some(vec![Some(t_rx); 7]);
    inner.tick(0.02);
    assert!(grasp::prepare_setup_update(&inner, &json!({"table_z_from_tcp": true}), None, false).is_err());

    // 3. B06 (R3): Two stationary hardware samples 20 ms apart, then frozen cache for 200 ms -> rejected
    drv.state.lock().unwrap().fail_read = true;
    inner.tick(0.02);
    drv.state.lock().unwrap().fail_read = false;
    let freeze_rx = t_rx + Duration::from_millis(20);
    drv.state.lock().unwrap().feedback_age = Duration::from_millis(5);
    drv.state.lock().unwrap().rx_instants = Some(vec![Some(freeze_rx); 7]);
    inner.tick(0.02);
    let second_rx = freeze_rx + Duration::from_millis(20);
    drv.state.lock().unwrap().rx_instants = Some(vec![Some(second_rx); 7]);
    inner.tick(0.02);
    // Freeze rx_instants at second_rx while sim_time_s advances past 200 ms and age remains <= 250 ms
    for k in 1..=10 {
        drv.state.lock().unwrap().feedback_age = Duration::from_millis(20 * k);
        inner.tick(0.02);
    }
    let err_two_then_freeze = grasp::prepare_setup_update(&inner, &json!({"table_z_from_tcp": true}), None, false).unwrap_err();
    assert!(format!("{err_two_then_freeze:?}").contains("still"), "{err_two_then_freeze:?}");

    // 4. Partial-joint freeze (one joint stuck at +20 ms while others reach +220 ms) -> rejected
    drv.state.lock().unwrap().feedback_age = Duration::from_millis(5);
    let mut partial_rx = vec![Some(freeze_rx + Duration::from_millis(220)); 7];
    partial_rx[3] = Some(second_rx);
    drv.state.lock().unwrap().rx_instants = Some(partial_rx);
    inner.tick(0.02);
    assert!(grasp::prepare_setup_update(&inner, &json!({"table_z_from_tcp": true}), None, false).is_err());

    // 5. All joints advance >= 200 ms from first_rx -> succeeds
    drv.state.lock().unwrap().rx_instants = Some(vec![Some(freeze_rx + Duration::from_millis(220)); 7]);
    inner.tick(0.02);
    assert!(grasp::prepare_setup_update(&inner, &json!({"table_z_from_tcp": true}), None, false).is_ok());
}

#[test]
fn r2_b07_commissioning_jog_scales_j7_velocity_and_requires_torque_feedback_on_close() {
    let cfg = finray_cfg();
    assert!(cfg.active_tool.commissioning_required);
    let mut q_open = INSPECT;
    q_open[6] = -3.0;
    let drv_no_tq = MockFbArm::new(q_open, false);
    let mut inner = SupervisorInner::new(cfg.clone());
    inner.driver = Some(Mutex::new(Box::new(drv_no_tq)));
    inner.lease_enabled = false;
    inner.set_torque(true).unwrap();
    inner.commanded_positions = q_open.to_vec();
    inner.measured_positions = q_open.to_vec();
    inner.target_positions = q_open.to_vec();

    let mut q_close = q_open;
    q_close[6] = -0.2;
    let ep = inner.safety.epoch_id();
    assert!(
        inner.jog_action_with_speed_scale(q_close.to_vec(), None, Some(ep), "joint_move", Some(0.2)).is_err(),
        "closing J7 jog during commissioning must require torque_feedback when max_close_torque_nm is configured"
    );

    let drv_tq = MockFbArm::new(q_open, true);
    inner.driver = Some(Mutex::new(Box::new(drv_tq)));
    let max_j7_vel = cfg.limits[6].max_velocity * 0.2;
    let (_ep, _id, dur) = inner.jog_action_with_speed_scale(q_close.to_vec(), None, Some(ep), "joint_move", Some(0.2)).unwrap();
    let dq7 = (q_close[6] - q_open[6]).abs() as f64;
    // Quintic peak velocity = 1.875 * |dq| / T <= max_j7_vel
    let peak_v7 = 1.875 * dq7 / dur;
    assert!(peak_v7 <= max_j7_vel as f64 + 1e-4, "peak_v7={peak_v7} exceeded scaled J7 limit {max_j7_vel}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn r2_b08_m14_commissioning_confirm_strict_validation_and_damaged_store_preservation() {
    let dir = tempfile::tempdir().unwrap();
    let comm_path = dir.path().join("tool_commissioning.json");
    let sup = Arc::new(ArmSupervisor::new(finray_cfg()));
    {
        let mut inner = sup.inner().write().await;
        inner.tool_commissioning_path = Some(comm_path.clone());
    }
    let (_ws, _gen, _epoch, _sup, addr) = common::torqued_rig(sup.clone(), &INSPECT).await;
    let base = format!("http://{addr}");
    let client = reqwest::Client::new();
    let st0: Value = client.get(format!("{base}/api/v1/tool/commissioning")).send().await.unwrap().json().await.unwrap();
    let rev0 = st0["revision"].as_u64().unwrap();
    let dig = st0["active_tool"]["config_digest"].as_str().unwrap().to_string();

    for bad_body in [
        json!({}),
        json!({"confirm": false, "tool_id": "soft_finray_gripper", "tool_config_digest": dig}),
        json!({"confirm": "true", "tool_id": "soft_finray_gripper", "tool_config_digest": dig}),
        json!({"confirm": true, "tool_config_digest": dig}),
        json!({"confirm": true, "tool_id": "wrong", "tool_config_digest": dig}),
        json!({"confirm": true, "tool_id": "soft_finray_gripper"}),
        json!({"confirm": true, "tool_id": "soft_finray_gripper", "tool_config_digest": "v1:wrong"}),
    ] {
        let r = client.post(format!("{base}/api/v1/tool/commissioning/confirm")).json(&bad_body).send().await.unwrap();
        assert_eq!(r.status(), 400, "body {bad_body} must be rejected");
        assert!(!comm_path.exists());
        let st: Value = client.get(format!("{base}/api/v1/tool/commissioning")).send().await.unwrap().json().await.unwrap();
        assert_eq!(st["revision"].as_u64().unwrap(), rev0);
        assert_eq!(st["commissioning_required"], true);
    }

    // M14: Damaged commissioning store file must never be overwritten or quarantined on load or confirm
    let corrupt_bytes = b"{not valid json!!";
    std::fs::write(&comm_path, corrupt_bytes).unwrap();
    let loaded_sup = ArmSupervisor::new(finray_cfg());
    tool::load_commissioning_into(&loaded_sup, &comm_path).await;
    assert!(loaded_sup.inner().read().await.config.active_tool.commissioning_required);
    assert_eq!(std::fs::read(&comm_path).unwrap(), corrupt_bytes);

    let r_dmg = client
        .post(format!("{base}/api/v1/tool/commissioning/confirm"))
        .json(&json!({"confirm": true, "tool_id": "soft_finray_gripper", "tool_config_digest": dig}))
        .send()
        .await
        .unwrap();
    assert_eq!(r_dmg.status(), 500);
    let body_dmg: Value = r_dmg.json().await.unwrap();
    assert_eq!(body_dmg["code"], "COMMISSIONING_STORE_DAMAGED");
    assert_eq!(std::fs::read(&comm_path).unwrap(), corrupt_bytes, "damaged store must be untouched");
    assert!(sup.inner().read().await.config.active_tool.commissioning_required);
    assert_eq!(sup.inner().read().await.tool_commissioning_revision, rev0);

    // M14 (R3): Valid JSON with misspelled top-level key (`recordz`) or missing `records` must
    // also fail closed without overwriting file, incrementing revision, or clearing gate.
    for bad_json_bytes in [br#"{"recordz":[]}"#.as_slice(), br#"{}"#.as_slice()] {
        std::fs::write(&comm_path, bad_json_bytes).unwrap();
        let misspelled_sup = ArmSupervisor::new(finray_cfg());
        tool::load_commissioning_into(&misspelled_sup, &comm_path).await;
        assert!(misspelled_sup.inner().read().await.config.active_tool.commissioning_required);
        assert_eq!(std::fs::read(&comm_path).unwrap(), bad_json_bytes);

        let r_bad_key = client
            .post(format!("{base}/api/v1/tool/commissioning/confirm"))
            .json(&json!({"confirm": true, "tool_id": "soft_finray_gripper", "tool_config_digest": dig}))
            .send()
            .await
            .unwrap();
        assert_eq!(r_bad_key.status(), 500);
        let body_bad_key: Value = r_bad_key.json().await.unwrap();
        assert_eq!(body_bad_key["code"], "COMMISSIONING_STORE_DAMAGED");
        assert_eq!(std::fs::read(&comm_path).unwrap(), bad_json_bytes, "misspelled store must be untouched");
        assert!(sup.inner().read().await.config.active_tool.commissioning_required);
        assert_eq!(sup.inner().read().await.tool_commissioning_revision, rev0);
    }
}

#[test]
fn r2_b09_place_approach_checks_table_clearance() {
    let mut cfg = finray_cfg();
    cfg.active_tool.commissioning_required = false;
    // Start with roll = 90 deg at tcp=[0.26, 0.0, 0.28]: lateral +/-0.082 m check points hang down to z=0.198 m.
    // Place target at tcp=[0.26, 0.0, 0.25], rpy=[0, 0, 0] (roll=0): descend corners stay >= 0.228 m.
    // With table_z_m = 0.190 (bound 0.215 m), descend clears while lift/approach violates table clearance.
    let (lo, hi): ([f64; 6], [f64; 6]) = (
        std::array::from_fn(|i| cfg.limits[i].min_position as f64),
        std::array::from_fn(|i| cfg.limits[i].max_position as f64),
    );
    let r_rolled = kinematics::rpy_to_r(std::f64::consts::FRAC_PI_2, 0.0, 0.0);
    let end_goal = cfg.tcp_to_end(&Pose::new(r_rolled, [0.26, 0.0, 0.28]));
    let sol = cfg.arm_model.solve_dls(&end_goal, &[0.0, -1.167, -1.565, 1.435, 0.0, 0.0], &lo, &hi, kinematics::PAR.w);
    assert!(sol.ok, "rolled start IK");
    let q_start: [f32; 7] = std::array::from_fn(|i| if i < 6 { sol.q[i] as f32 } else { 0.0 });

    let mut inner = common::sim_at(cfg, &q_start);
    inner.set_grasp_setup(GraspSetup { table_z_m: 0.190, ..GraspSetup::default() });
    let pending = armd::place::prepare(
        &inner,
        &json!({
            "position": [0.26, 0.0, 0.25],
            "orientation": {"rpy": [0.0, 0.0, 0.0]},
            "approach_height_m": 0.04,
        }),
        inner.safety.epoch_id(),
    )
    .unwrap();
    let err = armd::place::plan(pending).unwrap_err();
    assert!(
        matches!(err, ArmError::Motion(armd::motion::MotionError::WorkspaceViolation { .. })),
        "expected WORKSPACE_VIOLATION on approach/lift/retreat table dip, got {err:?}"
    );
}

#[test]
fn r2_b11_read_joint_states_error_clears_freshness_and_freezes_streaming_close() {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.gripper_policy.as_mut().unwrap().max_close_torque_nm = Some(1.0);
    let q0 = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0, -3.0];
    let drv = MockFbArm::new(q0, true);
    let mut inner = SupervisorInner::new(cfg);
    inner.driver = Some(Mutex::new(Box::new(drv.clone())));
    inner.lease_enabled = false;
    inner.set_torque(true).unwrap();
    inner.commanded_positions = q0.to_vec();
    inner.measured_positions = q0.to_vec();
    inner.target_positions = q0.to_vec();
    inner.tick(0.02);
    assert!(inner.last_can_feedback_instant.is_some());

    let mut tgt = q0.to_vec();
    tgt[6] = -0.2;
    let ep = inner.safety.epoch_id();
    inner.servo(tgt, ep).unwrap();
    // Single bus read failure must clear last_can_feedback_instant immediately and trip force-cap hold
    drv.state.lock().unwrap().fail_read = true;
    inner.tick(0.02);
    assert!(inner.last_can_feedback_instant.is_none());
    assert!(inner.gripper_force_capped);
    assert!(inner.gripper_stalled);
}

#[test]
fn r2_m05_m06_hover_resume_requires_fp_and_physical_jaw_curve_required() {
    // M05: resume without candidate_fingerprint -> rejected
    let mut cfg = finray_cfg();
    cfg.active_tool.commissioning_required = false;
    let mut inner = common::sim_at(cfg, &INSPECT);
    let body = json!({
        "candidate_fingerprint": "fp-A",
        "rpy_rad": [0.0, 1.5708, 0.0],
        "points": [{"id": "p1", "approach_m": [0.25, 0.0, 0.14], "hover_m": [0.25, 0.0, 0.08]}]
    });
    let mut plan = armd::calib_hover::prepare(&inner, &body, inner.safety.epoch_id()).unwrap();
    let job_id = plan.job.id.clone();
    plan.job.state = "failed".into();
    inner.hover_jobs.push(plan.job);
    let err_missing_fp = armd::calib_hover::prepare(&inner, &json!({"resume": job_id}), inner.safety.epoch_id()).unwrap_err();
    assert!(err_missing_fp.to_string().contains("candidate_fingerprint"), "{err_missing_fp}");

    // M06: physical non-sim driver with missing jaw_opening curve refuses sim_default() bypass on setup update
    let mut no_curve_cfg = ArmConfig::rebot_dm();
    no_curve_cfg.gripper_policy.as_mut().unwrap().jaw_opening = None;
    let drv = MockFbArm::new(INSPECT, true);
    let mut phys_inner = SupervisorInner::new(no_curve_cfg);
    phys_inner.driver = Some(Mutex::new(Box::new(drv)));
    phys_inner.follower = Some("rebot-dm".into());
    assert!(phys_inner.requires_real_jaw_curve());
    let err_no_curve = grasp::prepare_setup_update(&phys_inner, &json!({"max_grasp_width_m": 0.05}), None, false).unwrap_err();
    assert!(format!("{err_no_curve:?}").contains("jaw_opening"), "{err_no_curve:?}");
}

#[test]
fn r2_b05_replay_rejects_embedded_parquet_sidecar_conflict_and_m12_so101_manifest_check() {
    // B05: replay rejects episode whose parquet embedded provenance conflicts with sidecar
    let dir = tempfile::tempdir().unwrap();
    let fin = finray_cfg();
    let other = other_tool_cfg();
    let store = armd::recording::EpisodeStore::new(Some(dir.path().to_path_buf()));
    let mut frames = Vec::new();
    for k in 0..10 {
        frames.push(RecordFrame { timestamp: k as f64 * 0.02, observation_state: INSPECT.to_vec(), action: INSPECT.to_vec() });
    }
    let sc = EpisodeSidecar {
        id: "ep_conflict".into(),
        dataset: "default".into(),
        task: "t".into(),
        source: "sdk".into(),
        sequence_id: None,
        repeat_index: None,
        outcome: "ok".into(),
        recorded_at: "2026-11-22T00:00:00Z".into(),
        duration_s: 0.2,
        frames: frames.len(),
        joint_names: fin.limits.iter().map(|l| l.name.clone()).collect(),
        units: "rad".into(),
        joint_zero: armd::recording::JointZeroStamp::of(&vec![0.0; 7]),
        profile_id: fin.profile_id.clone(),
        armd_version: "test".into(),
        fps_nominal: 50.0,
        truncated: false,
        stopped_by: None,
        arm_dof: Some(6),
        gripper_joint: Some(6),
        tool_id: Some(fin.active_tool.id.clone()),
        tool_config_digest: Some(fin.active_tool.config_digest.clone()),
        has_gripper_motion: Some(false),
        recovered: None,
        first_state: Some(frames[0].observation_state.clone()),
        last_state: Some(frames[0].observation_state.clone()),
    };
    let pq_path = store.write_episode(&sc, &frames, None).unwrap();
    // Overwrite parquet with DIFFERENT embedded tool_id while keeping sidecar as finray -> conflict!
    let conflicting_meta = EpisodeMetadata {
        profile_id: fin.profile_id.clone(),
        arm_dof: Some(6),
        gripper_joint: Some(6),
        joint_names: Some(fin.limits.iter().map(|l| l.name.clone()).collect()),
        tool_id: Some(other.active_tool.id.clone()),
        tool_config_digest: Some(other.active_tool.config_digest.clone()),
        has_gripper_motion: Some(false),
    };
    armd::recording::write_frames_to_parquet_with_meta(&pq_path, &frames, Some(&conflicting_meta)).unwrap();
    let input = ReplayInput {
        request: ReplayRequest { episode: "ep_conflict".into(), speed: 1.0, gripper: false, tag: None, allow_stale: false },
        path: pq_path,
        profile_id: fin.profile_id.clone(),
        dof: fin.dof,
        arm_dof: fin.arm_dof(),
        gripper_idx: fin.gripper_index(),
        joint_names: fin.limits.iter().map(|l| l.name.clone()).collect(),
        limits: fin.limits.clone(),
        speed_limits: fin.limits.clone(),
        ws: None,
        arm: fin.arm_model.clone(),
        start: INSPECT.to_vec(),
        control_hz: 50,
        epoch: 0,
        workspace_revision: 0,
        power: None,
        store,
        zero_offsets: vec![0.0; 7],
        tcp_offset: fin.tcp_offset,
        active_tool: fin.active_tool.clone(),
    };
    let err = replay::plan(input).unwrap_err();
    assert!(format!("{err:?}").contains("conflicts"), "{err:?}");

    // M12: validate_tool_manifest rejects armd.gripper_policy violating target robot limits (so-arm101)
    let bad_so101: armd::mjcf_compose::ToolManifest = serde_json::from_value(json!({
        "id": "bad_so101_gp",
        "kind": "gripper",
        "robots": ["so-arm101"],
        "tcp_offset": [0.02, 0.0, 0.0],
        "armd": {
            "gripper_policy": {
                "closed_rad": -2.0
            }
        }
    })).unwrap();
    assert!(armd::mjcf_compose::validate_tool_manifest(&bad_so101, None).is_err());
}

#[tokio::test]
async fn m6_stream_contact_speed_invalidates_commissioning_and_m8_rejects_non_string_parquet_meta() {
    // M6: Changing base profile stream_contact_speed_rad_s invalidates prior commissioning.
    let comm_dir = tempfile::tempdir().unwrap();
    let comm_file = comm_dir.path().join("tool_commissioning.json");
    let pack: ToolPackConfig = serde_json::from_str(FINRAY_PACK_JSON).unwrap();
    let base1 = ArmConfig::rebot_dm();
    let mut base2 = base1.clone();
    base2.gripper_policy.as_mut().unwrap().stream_contact_speed_rad_s = Some(0.15);
    let cfg1 = tool::apply_tool_overlay(&base1, &pack).unwrap();
    let cfg2 = tool::apply_tool_overlay(&base2, &pack).unwrap();
    assert_ne!(cfg1.active_tool.config_digest, cfg2.active_tool.config_digest);
    let sup1 = Arc::new(ArmSupervisor::new(cfg1.clone()));
    {
        let mut inner = sup1.inner().write().await;
        inner.tool_commissioning_path = Some(comm_file.clone());
    }
    let prep = {
        let inner = sup1.inner().read().await;
        tool::prepare_commissioning_confirm(
            &inner,
            &json!({
                "confirm": true,
                "tool_id": cfg1.active_tool.id,
                "tool_config_digest": cfg1.active_tool.config_digest
            }),
            false,
        )
        .unwrap()
    };
    prep.write.unwrap().run().await.unwrap();
    let sup2 = Arc::new(ArmSupervisor::new(cfg2));
    tool::load_commissioning_into(&sup2, &comm_file).await;
    assert!(
        sup2.inner().read().await.config.active_tool.commissioning_required,
        "changing stream_contact_speed_rad_s must invalidate old commissioning"
    );

    // M8: Non-string tool_id / tool_config_digest in Parquet rebot.episode metadata must be rejected.
    let mut kv_bad_tid = std::collections::HashMap::new();
    kv_bad_tid.insert("rebot.episode".to_string(), r#"{"tool_id": 42}"#.to_string());
    assert!(armd::recording::import::extract_embedded_provenance(&kv_bad_tid)
        .unwrap_err()
        .contains("tool_id"));

    let mut kv_bad_dig = std::collections::HashMap::new();
    kv_bad_dig.insert("rebot.episode".to_string(), r#"{"tool_config_digest": []}"#.to_string());
    assert!(armd::recording::import::extract_embedded_provenance(&kv_bad_dig)
        .unwrap_err()
        .contains("tool_config_digest"));
}
