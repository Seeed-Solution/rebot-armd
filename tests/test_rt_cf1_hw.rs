//! C-F1 part 2 integration & zero-allocation test suite:
//! hardware commissioning on the RT executor (`DamiaoArmDriver<FakeBus>` + `ZeroOffsetDriver`
//! + `RtExecutor` + `TimedAlloc`).
//!
//! Covers:
//! 1. MIT TIMEOUT dry_run & RAM write (`0x55` RID 9 LE, zero `0xAA` flash writes, separate `0x33` readback).
//! 2. Causal per-request verification (B1): stale queued replies, wrong `reply_id`, ack mismatch,
//!    `ignore_reg_writes`, and `real_flush` cutoff/flush failure (`SentFlushFailed` -> `Uncertain`, no retry).
//! 3. Strict fresh rest-pose evidence for irreversible writes even with `force: true` (B4).
//! 4. Authority revocation mid-batch (Stop, Lease loss, Disconnect) with 0 subsequent TX (B2).
//! 5. MIT qualification fresh readback gate, mismatch refusal, atomic commit & revision bump (M4).
//! 6. Hardware `motor_zero` full sequence: disable-all + confirm (M1), 100 ms spacing, pre-HW MIT
//!    invalidation consistency (B3), raw post-command verify (M2), dependent invalidation, partial
//!    batch -> `Reconciling` + pending marker + startup hardware recovery guidance (B3).
//! 7. Driver-backed software zero & `joint_zero/restore` on executor (updating `ZeroOffsetDriver` + limits).
//! 8. Post-HW dir-sync failure stays `Reconciling` without config-only retry (B3).
//! 9. Damiao `0xD` command-timeout expiry on `FakeBus` + Executor latches fault, fences motion &
//!    rearm, does not auto-clear, keeps 45 ms `MIT_DEADLINE` intact (M5).
//! 10. Paired Legacy vs Executor WS parity + independent direct Rust legacy guards + real-follower gate (m1, m2).

mod mit_support;

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use futures_util::{SinkExt, StreamExt};
use serde_json::{json, Value};
use tokio::net::TcpListener;
use tokio_tungstenite::tungstenite::Message;

use armd::auth::OperatorAuth;
use armd::calibration::{CalibrationDeps, CalibrationFile};
use armd::diagnostics::alloc::{AllocCounts, TimedAlloc};
use armd::driver::{ArmDriver, DamiaoArmDriver};
use armd::grasp::GraspDeps;
use armd::joint_zero::ZeroOffsetDriver;
use armd::mit::{self, MitJointQualification, MitQualificationFile, MIT_DEADLINE};
use armd::model::ArmConfig;
use armd::motor_zero::{self, MotorZeroRequest};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::config_iface::{ConfigDomain, HoldState};
use armd::rt_core::executor::{ExecutorConfig, RtExecutorHandle, RtOwner};
use armd::rt_core::requests::config::{
    ConfigCommitTestHooks, ConfigReply, ConfigRequest, CALIBRATION_PENDING_MARKER_NAME,
};
use armd::safety::SafetyState;
use armd::supervisor::ArmSupervisor;
use armd::transport::{CanFrame, MockClock};

fn pending_commit_marker_path(calib_path: &std::path::Path) -> PathBuf {
    calib_path
        .parent()
        .unwrap_or_else(|| std::path::Path::new("."))
        .join(CALIBRATION_PENDING_MARKER_NAME)
}

use mit_support::{assert_no_flash_write, FakeBus, SW_VER};

#[global_allocator]
static ALLOC: TimedAlloc = TimedAlloc;

fn b601_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm()
        .damiao_ranges
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect()
}

type WsStream =
    tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>>;

async fn ws_send_recv(ws: &mut WsStream, msg: Value) -> Value {
    ws.send(Message::Text(msg.to_string().into())).await.unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let rem = deadline.saturating_duration_since(Instant::now());
        assert!(!rem.is_zero(), "timed out waiting for WS reply");
        let frame = tokio::time::timeout(rem, ws.next())
            .await
            .expect("WS recv timeout")
            .expect("WS closed")
            .expect("WS error");
        let Message::Text(txt) = frame else { continue };
        let v: Value = serde_json::from_str(&txt).unwrap();
        let ty = v.get("type").and_then(Value::as_str).unwrap_or("");
        if matches!(
            ty,
            "lease_granted"
                | "lease_denied"
                | "lease_released"
                | "command_ack"
                | "command_reject"
                | "error"
        ) {
            return v;
        }
    }
}

async fn ws_acquire_lease(ws: &mut WsStream, client_id: &str) -> (u64, u64) {
    let reply = ws_send_recv(
        ws,
        json!({
            "type": "acquire_lease",
            "client_id": client_id,
            "force": false,
            "seq": 1,
        }),
    )
    .await;
    assert_eq!(reply["type"], "lease_granted", "lease acquire failed: {reply}");
    (
        reply["generation"].as_u64().unwrap(),
        reply["epoch_id"].as_u64().unwrap_or(1),
    )
}

async fn ws_cmd(
    ws: &mut WsStream,
    req_id: u64,
    gen: u64,
    epoch: u64,
    path: &str,
    body: Value,
) -> Value {
    ws_send_recv(
        ws,
        json!({
            "type": "command",
            "request_id": req_id,
            "generation": gen,
            "epoch_id": epoch,
            "path": path,
            "body": body,
        }),
    )
    .await
}

struct HwRig {
    sup: Arc<ArmSupervisor>,
    bus: FakeBus,
    handle: Option<RtExecutorHandle>,
    feeder_stop: Arc<AtomicBool>,
    feeder_jh: Option<tokio::task::JoinHandle<()>>,
    server_jh: tokio::task::JoinHandle<()>,
    ws_url: String,
    _dir: tempfile::TempDir,
    calib_path: PathBuf,
    setup_path: PathBuf,
    cal_deps: CalibrationDeps,
}

impl HwRig {
    async fn new(
        owner: RtOwner,
        at_rest: bool,
        clock: Option<MockClock>,
        setup_sup: impl FnOnce(&mut armd::SupervisorInner),
    ) -> Self {
        let dir = tempfile::tempdir().unwrap();
        let calib_path = dir.path().join("calibration.json");
        let setup_path = dir.path().join("grasp_setup.json");
        let poses_path = dir.path().join("poses.json");
        let gripper_calib_path = dir.path().join("gripper_calib.json");

        let cfg = ArmConfig::rebot_dm();
        let rest_pos = cfg.resting_positions();
        let limits = b601_limits();
        let bus = match clock {
            Some(c) => FakeBus::with_clock(&limits, c),
            None => FakeBus::new(&limits),
        };
        bus.with(|s| {
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = if at_rest || i >= 6 {
                    rest_pos.get(i).copied().unwrap_or(0.0)
                } else {
                    rest_pos.get(i).copied().unwrap_or(0.0) + 0.3
                };
                m.v = 0.0;
                m.enabled = false;
            }
            s.push_all_feedback();
        });

        let mut raw_drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits)
            .unwrap();
        let _ = raw_drv.read_joint_states();
        let zero_drv = ZeroOffsetDriver::new(Box::new(raw_drv), &vec![0.0; cfg.dof]);
        let sup = Arc::new(ArmSupervisor::new(cfg.clone()).with_driver(Box::new(zero_drv)));
        {
            let mut inner = sup.inner().write().await;
            inner.poses.path = Some(poses_path.clone());
            inner.gripper_calib_path = Some(gripper_calib_path.clone());
            // Install stopped disable gate matching the profile resting pose.
            let gate = inner.stopped_disable_gate();
            if let Some(d) = inner.driver.as_mut() {
                let _ = d.with_mut(|drv| drv.set_stopped_disable_gate(gate));
            }
            setup_sup(&mut inner);
        }

        // Seed 4 fresh feedback passes so K=3 rest-pose evidence is confirmed when `at_rest`.
        for _ in 0..4 {
            bus.with(|s| s.push_all_feedback());
            let mut inner = sup.inner().write().await;
            if let Some(d) = inner.driver.as_mut() {
                let _ = d.with_mut(|drv| drv.read_joint_states());
            }
        }
        bus.clear_log();

        let feeder_stop = Arc::new(AtomicBool::new(false));
        let feeder_jh = if bus.with(|s| s.clock.is_none()) {
            let b = bus.clone();
            let stop = feeder_stop.clone();
            Some(tokio::spawn(async move {
                while !stop.load(Ordering::Acquire) {
                    b.with(|s| {
                        // Only inject periodic feedback when queue is nearly empty so we never
                        // overflow preallocated VecDeque (cap 256) or interfere with register replies.
                        if s.rx.len() < 16 && s.pending.len() < 16 {
                            s.push_all_feedback();
                        }
                    });
                    tokio::time::sleep(Duration::from_millis(8)).await;
                }
            }))
        } else {
            None
        };

        let handle = if owner == RtOwner::Executor {
            let mut ecfg = ExecutorConfig::for_tests(Duration::from_millis(8));
            ecfg.alloc_timing = true;
            let h = sup.start_executor(ecfg).await.unwrap();
            let dl = Instant::now() + Duration::from_secs(3);
            while sup.hub().latest().is_none() {
                assert!(Instant::now() < dl, "executor never published initial snapshot");
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
            // Wait a few releases so the executor refreshes feedback & K=3 rest evidence.
            tokio::time::sleep(Duration::from_millis(40)).await;
            Some(h)
        } else {
            // Legacy owner: refresh initial measured positions from driver.
            let mut inner = sup.inner().write().await;
            if let Some(d) = inner.driver.as_mut() {
                if let Some(Ok(states)) = d.with_mut(|drv| drv.read_joint_states()) {
                    for (i, st) in states.iter().enumerate().take(inner.config.dof) {
                        inner.measured_positions[i] = st.position;
                        inner.commanded_positions[i] = st.position;
                        inner.target_positions[i] = st.position;
                    }
                }
            }
            None
        };

        let det_up = armd::api::proxy::DetectorUpstream::parse("http://127.0.0.1:9").unwrap();
        let grasp_deps = GraspDeps::http(det_up.clone(), Some(setup_path.clone()));
        let cal_deps = CalibrationDeps::new(calib_path.clone(), Some(setup_path.clone()), det_up.clone());
        let router = armd::api::create_router_calibration(
            sup.clone(),
            det_up,
            grasp_deps,
            armd::diagnostics::DiagnosticsState::global(),
            cal_deps.clone(),
            OperatorAuth::disabled(),
        );
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        let server_jh = tokio::spawn(async move {
            let _ = axum::serve(listener, router).await;
        });
        let ws_url = format!("ws://{addr}/api/v1/ws");

        Self {
            sup,
            bus,
            handle,
            feeder_stop,
            feeder_jh,
            server_jh,
            ws_url,
            _dir: dir,
            calib_path,
            setup_path,
            cal_deps,
        }
    }

    async fn connect_ws(&self) -> WsStream {
        let (ws, _) = tokio_tungstenite::connect_async(&self.ws_url).await.unwrap();
        ws
    }

    async fn hold_state(&self) -> Option<HoldState> {
        match self
            .sup
            .dispatch_config_request(ConfigRequest::InspectHold(ConfigDomain::Calibration))
            .await
            .unwrap()
        {
            ConfigReply::HoldInspected { state, .. } => state,
            _ => None,
        }
    }

    fn assert_zero_alloc(&self, label: &str) {
        if let Some(ref h) = self.handle {
            assert_eq!(
                h.stats().total_allocs(),
                AllocCounts::ZERO,
                "{label}: RT executor must perform 0 heap allocs and 0 heap frees"
            );
        }
    }

    async fn shutdown(mut self) {
        self.feeder_stop.store(true, Ordering::Release);
        if let Some(jh) = self.feeder_jh.take() {
            let _ = jh.await;
        }
        self.server_jh.abort();
        if let Some(h) = self.handle.take() {
            tokio::task::spawn_blocking(move || h.stop_and_join())
                .await
                .unwrap();
        }
    }
}

fn build_valid_qualification(
    cfg: &ArmConfig,
    zero_offsets: &[f32],
    joints: &[usize],
    timeout_raw: u32,
) -> MitQualificationFile {
    let ranges = cfg.damiao_ranges.as_ref().unwrap();
    let entries = joints
        .iter()
        .map(|&j| {
            let r = &ranges[j];
            let l = r.limits();
            MitJointQualification {
                joint: j + 1,
                joint_name: cfg.joint_names[j].clone(),
                profile_id: cfg.profile_id.clone().unwrap(),
                motor_id: (j + 1) as u16,
                reply_id: 0x10 + (j + 1) as u32,
                esc_id: (j + 1) as u32,
                mst_id: 0x10 + (j + 1) as u32,
                sw_ver: SW_VER,
                pmax: l.p_max,
                vmax: l.v_max,
                tmax: l.t_max,
                timeout_raw,
                timeout_unit_us: damiao::TIMEOUT_UNIT_US_DOCUMENTED,
                timeout_unit_verified: true,
                tested_response: "disabled".into(),
                measured_expiry_ms: 100.0,
                worst_latency_ms: 5.0,
                load_envelope: "unloaded bench".into(),
                max_kp: 25.0,
                max_kd: 2.0,
                effort_ceiling_nm: l.t_max,
                zero_fingerprint: mit::zero_fingerprint(zero_offsets[j]),
                date: "2026-11-25T08:00:00Z".into(),
                trace_id: "cf1-hw-test".into(),
            }
        })
        .collect();
    MitQualificationFile {
        version: mit::QUALIFICATION_VERSION,
        joints: entries,
        invalidated: None,
    }
}

// ============================================================================
// 1. MIT TIMEOUT dry_run & RAM write exact frames + 0 RT alloc
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn mit_timeout_dry_run_and_ram_write_exact_frames_and_zero_alloc() {
    let rig = HwRig::new(RtOwner::Executor, true, None, |_| {}).await;
    let mut ws = rig.connect_ws().await;
    let (gen, epoch) = ws_acquire_lease(&mut ws, "c-timeout").await;

    // 1. dry_run: reads identity registers only, zero REG_WRITE and zero 0xAA flash writes.
    rig.bus.clear_log();
    let dry = ws_cmd(
        &mut ws,
        1,
        gen,
        epoch,
        mit::TIMEOUT_PATH,
        json!({ "joints": [1, 2], "dry_run": true }),
    )
    .await;
    assert_eq!(dry["type"], "command_ack", "{dry}");
    assert_eq!(dry["dry_run"], true);
    let rb = dry["readback"].as_array().unwrap();
    assert_eq!(rb.len(), 2);
    assert_eq!(rb[0]["joint"], 1);
    assert_eq!(rb[0]["sw_ver"], SW_VER);
    assert_eq!(rb[0]["timeout_raw"], 0);
    assert_eq!(rb[1]["joint"], 2);
    let dry_log = rig.bus.tx_log();
    assert_no_flash_write(&dry_log);
    assert!(
        !dry_log
            .iter()
            .any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE),
        "dry_run must never send REG_WRITE"
    );
    assert_eq!(rig.hold_state().await, None);

    // 2. Real RAM write: exact 0x55 RID 9 LE payload, zero 0xAA flash writes, separate 0x33 readback.
    rig.bus.clear_log();
    let wr = ws_cmd(
        &mut ws,
        2,
        gen,
        epoch,
        mit::TIMEOUT_PATH,
        json!({ "joints": [1, 2], "timeout_counts": 2000, "dry_run": false }),
    )
    .await;
    assert_eq!(wr["type"], "command_ack", "{wr}");
    assert_eq!(wr["dry_run"], false);
    assert_eq!(wr["timeout_counts"], 2000);
    let comm = wr["commissioning"].as_array().unwrap();
    assert_eq!(comm.len(), 2);
    assert_eq!(comm[0]["written_raw"], 2000);
    assert_eq!(comm[0]["ack_raw"], 2000);
    assert_eq!(comm[0]["readback"]["timeout_raw"], 2000);
    assert_eq!(comm[1]["readback"]["timeout_raw"], 2000);

    let wr_log = rig.bus.tx_log();
    assert_no_flash_write(&wr_log);
    let writes: Vec<CanFrame> = wr_log
        .iter()
        .copied()
        .filter(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE && f.data[3] == damiao::RID_TIMEOUT)
        .collect();
    assert_eq!(writes.len(), 2, "expected exactly 1 RAM REG_WRITE per joint");
    assert_eq!(u16::from_le_bytes([writes[0].data[0], writes[0].data[1]]), 1);
    assert_eq!(u32::from_le_bytes([writes[0].data[4], writes[0].data[5], writes[0].data[6], writes[0].data[7]]), 2000);
    assert_eq!(u16::from_le_bytes([writes[1].data[0], writes[1].data[1]]), 2);
    assert_eq!(u32::from_le_bytes([writes[1].data[4], writes[1].data[5], writes[1].data[6], writes[1].data[7]]), 2000);

    // Separate post-write 0x33 readback for RID_TIMEOUT must appear after each 0x55 write.
    let w1_pos = wr_log.iter().position(|f| *f == writes[0]).unwrap();
    assert!(
        wr_log[w1_pos + 1..]
            .iter()
            .any(|f| f.id == 0x7FF && f.data[0] == 1 && f.data[2] == damiao::REG_READ && f.data[3] == damiao::RID_TIMEOUT),
        "must issue separate 0x33 readback after 0x55 write"
    );

    assert!(!pending_commit_marker_path(&rig.calib_path).exists());
    assert_eq!(rig.hold_state().await, None);
    rig.assert_zero_alloc("mit_timeout dry_run + RAM write");
    rig.shutdown().await;
}

// ============================================================================
// 2. B1 causal reply verification, ignore_reg_writes, wrong reply_id, real_flush failure
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn mit_timeout_rejects_stale_and_wrong_replies_and_flush_cutoff_uncertain() {
    // Subtest A: Stale pre-TX queued reply is drained/rejected causally (B1), while a genuine
    // post-TX reply succeeds; and a post-TX bogus ack value is rejected with uncertain: [1].
    {
        let rig = HwRig::new(RtOwner::Executor, true, None, |_| {}).await;
        let mut ws = rig.connect_ws().await;
        let (gen, epoch) = ws_acquire_lease(&mut ws, "c-b1").await;

        let stale_reply = CanFrame::new_standard(
            0x11,
            &[1, 0, damiao::REG_READ, damiao::RID_TIMEOUT, 0xDE, 0xAD, 0xBE, 0xEF],
        )
        .unwrap();
        let pre_tx_ts = Instant::now().checked_sub(Duration::from_millis(50)).unwrap();
        rig.bus.with(|s| {
            s.push_rx_at(stale_reply, Some(pre_tx_ts));
        });
        let wr = ws_cmd(
            &mut ws,
            1,
            gen,
            epoch,
            mit::TIMEOUT_PATH,
            json!({ "joints": [1], "timeout_counts": 1500, "dry_run": false }),
        )
        .await;
        assert_eq!(wr["type"], "command_ack", "stale pre-TX reply must be drained/rejected causally: {wr}");
        assert_eq!(wr["commissioning"][0]["ack_raw"], 1500);
        assert_eq!(wr["commissioning"][0]["readback"]["timeout_raw"], 1500);

        // Now inject a post-TX bogus ack value: must reject with MIT_TIMEOUT_FAILED + uncertain: [1] + Reconciling.
        let bogus_ack = CanFrame::new_standard(
            0x11,
            &[1, 0, damiao::REG_WRITE, damiao::RID_TIMEOUT, 0xDE, 0xAD, 0xBE, 0xEF],
        )
        .unwrap();
        rig.bus.with(|s| s.inject_on_reg_write.push(bogus_ack));
        let bad = ws_cmd(
            &mut ws,
            2,
            gen,
            epoch,
            mit::TIMEOUT_PATH,
            json!({ "joints": [1], "timeout_counts": 1600, "dry_run": false }),
        )
        .await;
        assert_eq!(bad["type"], "command_reject", "{bad}");
        assert_eq!(bad["code"], "MIT_TIMEOUT_FAILED");
        assert_eq!(bad["uncertain"], json!([1]));
        assert_eq!(rig.hold_state().await, Some(HoldState::Reconciling));

        rig.assert_zero_alloc("stale reply rejection + bogus ack");
        rig.shutdown().await;
    }

    // Subtest B: `ignore_reg_writes` (ack matches, separate 0x33 readback still 0) ->
    // joint 1 Uncertain, joint 2 not_attempted, Reconciling + pending marker, motion & rearm fenced.
    {
        let rig = HwRig::new(RtOwner::Executor, true, None, |_| {}).await;
        let mut ws = rig.connect_ws().await;
        let (gen, epoch) = ws_acquire_lease(&mut ws, "c-ignore").await;
        rig.bus.with(|s| s.motors[0].ignore_reg_writes = true);

        let err = ws_cmd(
            &mut ws,
            1,
            gen,
            epoch,
            mit::TIMEOUT_PATH,
            json!({ "joints": [1, 2], "timeout_counts": 2000, "dry_run": false }),
        )
        .await;
        assert_eq!(err["type"], "command_reject", "{err}");
        assert_eq!(err["status"], 502);
        assert_eq!(err["code"], "MIT_TIMEOUT_FAILED");
        assert_eq!(err["applied"], json!([]));
        assert_eq!(err["uncertain"], json!([1]));
        assert_eq!(err["not_attempted"], json!([2]));
        assert_eq!(rig.hold_state().await, Some(HoldState::Reconciling));
        assert!(pending_commit_marker_path(&rig.calib_path).exists());

        // Rearm/torque-on MUST be refused while Reconciling.
        let tq = ws_cmd(
            &mut ws,
            2,
            gen,
            epoch,
            "/api/v1/torque",
            json!({ "enable": true }),
        )
        .await;
        assert_eq!(tq["type"], "command_reject", "{tq}");

        rig.assert_zero_alloc("ignore_reg_writes uncertain");
        rig.shutdown().await;
    }

    // Subtest C: `real_flush = true` + `fail_next_reg_write_flush = true` on REG_WRITE ->
    // `SentFlushFailed` -> `Uncertain`, NEVER auto-retried (exact 1 REG_WRITE frame),
    // Reconciling + pending marker.
    {
        let rig = HwRig::new(RtOwner::Executor, true, None, |_| {}).await;
        let mut ws = rig.connect_ws().await;
        let (gen, epoch) = ws_acquire_lease(&mut ws, "c-flush").await;

        rig.bus.with(|s| {
            s.real_flush = true;
            s.fail_next_reg_write_flush = true;
        });
        rig.bus.clear_log();
        let err = ws_cmd(
            &mut ws,
            1,
            gen,
            epoch,
            mit::TIMEOUT_PATH,
            json!({ "joints": [1, 2], "timeout_counts": 2000, "dry_run": false }),
        )
        .await;
        assert_eq!(err["type"], "command_reject", "{err}");
        assert_eq!(err["code"], "MIT_TIMEOUT_FAILED");
        assert_eq!(err["uncertain"], json!([1]));
        assert_eq!(err["not_attempted"], json!([2]));
        let writes: usize = rig
            .bus
            .tx_log()
            .iter()
            .filter(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE && f.data[3] == damiao::RID_TIMEOUT)
            .count();
        assert_eq!(writes, 1, "SentFlushFailed REG_WRITE must NEVER be auto-retried");
        assert_eq!(rig.hold_state().await, Some(HoldState::Reconciling));
        assert!(pending_commit_marker_path(&rig.calib_path).exists());

        rig.assert_zero_alloc("real_flush SentFlushFailed uncertain no-retry");
        rig.shutdown().await;
    }

    // Subtest D: Wrong `reply_id` (mismatch against MST_ID) refused on write without emitting REG_WRITE.
    {
        let rig = HwRig::new(RtOwner::Executor, true, None, |_| {}).await;
        let mut ws = rig.connect_ws().await;
        let (gen, epoch) = ws_acquire_lease(&mut ws, "c-replyid").await;
        rig.bus.with(|s| {
            // Motor reports MST_ID = 0x55 while replying on CAN ID 0x11 -> reply_id != mst_id.
            s.motors[0].regs.insert(damiao::RID_MST_ID, 0x55u32.to_le_bytes());
        });
        rig.bus.clear_log();
        let err = ws_cmd(
            &mut ws,
            1,
            gen,
            epoch,
            mit::TIMEOUT_PATH,
            json!({ "joints": [1], "timeout_counts": 2000, "dry_run": false }),
        )
        .await;
        assert_eq!(err["type"], "command_reject", "{err}");
        assert_eq!(err["status"], 502);
        assert_eq!(err["code"], "MIT_TIMEOUT_FAILED");
        assert_eq!(err["not_attempted"], json!([1]));
        assert!(
            !rig.bus
                .tx_log()
                .iter()
                .any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE),
            "mismatched reply_id must be refused before any REG_WRITE"
        );
        rig.assert_zero_alloc("wrong reply_id");
        rig.shutdown().await;
    }
}

// ============================================================================
// 3. B4: Strict fresh rest-pose evidence even with `force: true`
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn mit_timeout_and_motor_zero_enforce_fresh_rest_evidence_even_with_force() {
    let rig = HwRig::new(RtOwner::Executor, false, None, |_| {}).await;
    let mut ws = rig.connect_ws().await;
    let (gen, epoch) = ws_acquire_lease(&mut ws, "c-rest").await;

    rig.bus.clear_log();
    let t_err = ws_cmd(
        &mut ws,
        1,
        gen,
        epoch,
        mit::TIMEOUT_PATH,
        json!({ "joints": [1], "timeout_counts": 2000, "dry_run": false }),
    )
    .await;
    assert_eq!(t_err["type"], "command_reject", "{t_err}");
    assert_eq!(t_err["status"], 409);
    assert_eq!(t_err["code"], "ARM_NOT_AT_RESTING_POSE");
    assert!(
        !rig.bus
            .tx_log()
            .iter()
            .any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE),
        "must not emit REG_WRITE when not at rest"
    );
    assert!(!pending_commit_marker_path(&rig.calib_path).exists());
    assert_eq!(rig.hold_state().await, None);

    rig.bus.clear_log();
    let mz_err = ws_cmd(
        &mut ws,
        2,
        gen,
        epoch,
        motor_zero::PATH,
        json!({ "joints": [1], "method": "motor", "force": true, "dry_run": false }),
    )
    .await;
    assert_eq!(mz_err["type"], "command_reject", "{mz_err}");
    assert_eq!(mz_err["status"], 409);
    assert_eq!(mz_err["code"], "ARM_NOT_AT_RESTING_POSE");
    assert_eq!(
        rig.bus.with(|s| s.motors[0].set_zero_count),
        0,
        "force: true must NOT relax fresh rest-pose evidence for hardware motor zero"
    );
    assert!(!pending_commit_marker_path(&rig.calib_path).exists());
    assert_eq!(rig.hold_state().await, None);

    rig.assert_zero_alloc("rest evidence gate");
    rig.shutdown().await;
}

// ============================================================================
// 4. B2: Authority revocation mid-batch & after backpressure -> 0 subsequent TX
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn mit_timeout_authority_revocation_mid_batch_and_after_backpressure() {
    let rig = HwRig::new(RtOwner::Executor, true, None, |_| {}).await;
    let mut ws = rig.connect_ws().await;
    let (gen, epoch) = ws_acquire_lease(&mut ws, "c-b2").await;

    // Stall J2 REG_WRITE once J1 completes, then fire urgent Stop via StopRequests
    // so J2/J3 are blocked immediately at the pre_tx_gate inside send_inner Step 5 (B2).
    let bus_watch = rig.bus.clone();
    let stop_reqs = rig.handle.as_ref().unwrap().stop_requests().clone();
    let watcher = tokio::spawn(async move {
        let dl = Instant::now() + Duration::from_secs(5);
        while Instant::now() < dl {
            let j1_done = bus_watch.with(|s| s.motors[0].reg_u32(damiao::RID_TIMEOUT) == 2000);
            if j1_done {
                bus_watch.with(|s| s.stall_reg_writes = true);
                stop_reqs.request();
                tokio::time::sleep(Duration::from_millis(10)).await;
                bus_watch.with(|s| s.stall_reg_writes = false);
                break;
            }
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
    });

    rig.bus.clear_log();
    let res = ws_cmd(
        &mut ws,
        1,
        gen,
        epoch,
        mit::TIMEOUT_PATH,
        json!({ "joints": [1, 2, 3], "timeout_counts": 2000, "dry_run": false }),
    )
    .await;
    let _ = watcher.await;
    assert_eq!(res["type"], "command_reject", "{res}");
    assert_eq!(res["code"], "STOPPED", "{res}");
    // J3 must NEVER have been written after urgent stop intervened!
    assert_eq!(
        rig.bus.with(|s| s.motors[2].reg_u32(damiao::RID_TIMEOUT)),
        0,
        "J3 must not be written after urgent stop"
    );
    assert!(
        res["not_attempted"]
            .as_array()
            .is_some_and(|a| a.contains(&json!(3))),
        "J3 must be listed in 1-based not_attempted: {res}"
    );
    assert_eq!(rig.hold_state().await, Some(HoldState::Reconciling));
    assert!(pending_commit_marker_path(&rig.calib_path).exists());

    rig.assert_zero_alloc("mid-batch stop revocation");
    rig.shutdown().await;
}

// ============================================================================
// 5. M4: MIT qualification fresh readback, gate check, atomic commit & revision bump
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn mit_qualification_fresh_readback_gate_commit_and_revision_bump() {
    let rig = HwRig::new(RtOwner::Executor, true, None, |_| {}).await;
    let mut ws = rig.connect_ws().await;
    let (gen, epoch) = ws_acquire_lease(&mut ws, "c-qual").await;

    // Commission TIMEOUT=2000 on J1 & J2 first via executor.
    let wr = ws_cmd(
        &mut ws,
        1,
        gen,
        epoch,
        mit::TIMEOUT_PATH,
        json!({ "joints": [1, 2], "timeout_counts": 2000, "dry_run": false }),
    )
    .await;
    assert_eq!(wr["type"], "command_ack", "{wr}");

    let pub_before = rig.sup.published_config().await.unwrap();

    // Mismatched timeout_raw (1999 != 2000) -> 409 MIT_QUALIFICATION_MISMATCH, disk & rev untouched.
    let bad_rec = build_valid_qualification(&pub_before.config, &pub_before.joint_zero_offsets, &[0, 1], 1999);
    let rej = ws_cmd(
        &mut ws,
        2,
        gen,
        epoch,
        mit::QUALIFICATION_PATH,
        json!({ "record": bad_rec }),
    )
    .await;
    assert_eq!(rej["type"], "command_reject", "{rej}");
    assert_eq!(rej["status"], 409);
    assert_eq!(rej["code"], "MIT_QUALIFICATION_MISMATCH");
    assert!(!rig.calib_path.exists());
    assert_eq!(
        rig.sup.published_config().await.unwrap().calibration_revision,
        pub_before.calibration_revision
    );

    // Matching record -> stored on disk, live calibration + revision bumped, no lingering hold.
    let good_rec = build_valid_qualification(&pub_before.config, &pub_before.joint_zero_offsets, &[0, 1], 2000);
    let ok = ws_cmd(
        &mut ws,
        3,
        gen,
        epoch,
        mit::QUALIFICATION_PATH,
        json!({ "record": good_rec }),
    )
    .await;
    assert_eq!(ok["type"], "command_ack", "{ok}");
    assert_eq!(ok["stored"], true);
    assert_eq!(ok["joints"], json!([1, 2]));
    assert!(rig.calib_path.exists());
    let pub_after = rig.sup.published_config().await.unwrap();
    assert_eq!(pub_after.calibration_revision, pub_before.calibration_revision + 1);
    assert!(!pub_after.calibration_busy);
    assert!(pub_after
        .calibration
        .as_ref()
        .and_then(|c| c.mit_record().ok().flatten())
        .is_some());
    assert_eq!(rig.hold_state().await, None);
    assert!(!pending_commit_marker_path(&rig.calib_path).exists());

    rig.assert_zero_alloc("mit_qualification gate + commit");
    rig.shutdown().await;
}

// ============================================================================
// 6. Hardware motor_zero full sequence, invalidation, partial reconcile + startup recovery
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn hardware_motor_zero_full_sequence_invalidation_and_partial_reconcile() {
    // Part A: Full success on J1 & J2 with seeded MIT qualification + poses + backups.
    {
        let cfg = ArmConfig::rebot_dm();
        let seed_qual = build_valid_qualification(&cfg, &vec![0.0; cfg.dof], &[0, 1], 2000);
        let mut seed_cal = CalibrationFile::default();
        seed_cal.set_mit_record(&seed_qual);
        let rig = HwRig::new(RtOwner::Executor, true, None, move |inner| {
            inner.calibration = Some(seed_cal);
        })
        .await;
        // Write initial calibration.json so backup & MIT invalidation operate on real files.
        let init_cal = rig.sup.published_config().await.unwrap().calibration.clone().unwrap();
        std::fs::write(&rig.calib_path, serde_json::to_string_pretty(&init_cal).unwrap()).unwrap();

        let mut ws = rig.connect_ws().await;
        let (gen, epoch) = ws_acquire_lease(&mut ws, "c-mz").await;
        let pub_before = rig.sup.published_config().await.unwrap();

        let t0 = Instant::now();
        let ack = ws_cmd(
            &mut ws,
            1,
            gen,
            epoch,
            motor_zero::PATH,
            json!({ "joints": [1, 2], "method": "motor", "dry_run": false }),
        )
        .await;
        let elapsed = t0.elapsed();
        assert_eq!(ack["type"], "command_ack", "{ack}");
        assert_eq!(ack["zero_method"], "motor");
        assert_eq!(ack["torque"], "off");
        assert_eq!(ack["joints"][0]["verified"], true);
        assert_eq!(ack["joints"][1]["verified"], true);
        assert!(
            elapsed >= Duration::from_millis(90),
            "multi-joint CMD_SET_ZERO must respect 100 ms spacing (elapsed={elapsed:?})"
        );
        assert_eq!(rig.bus.with(|s| s.motors[0].set_zero_count), 1);
        assert_eq!(rig.bus.with(|s| s.motors[1].set_zero_count), 1);

        // Verify MIT qualification was invalidated and joint_zero offsets installed (-reference).
        let pub_after = rig.sup.published_config().await.unwrap();
        assert!(pub_after.calibration_revision > pub_before.calibration_revision);
        let rest = pub_after.config.resting_positions();
        assert!((pub_after.joint_zero_offsets[0] - (-rest[0])).abs() < 1e-4);
        assert!((pub_after.joint_zero_offsets[1] - (-rest[1])).abs() < 1e-4);
        let mit_inv = pub_after
            .calibration
            .as_ref()
            .and_then(|c| c.mit_record().ok().flatten())
            .and_then(|m| m.invalidated);
        assert!(mit_inv.is_some(), "MIT qualification must be invalidated by hardware motor zero");
        assert!(!pending_commit_marker_path(&rig.calib_path).exists());
        assert_eq!(rig.hold_state().await, None);
        assert!(rig.cal_deps.backup_dir.join(motor_zero::LOG_NAME).exists());

        rig.assert_zero_alloc("hardware motor_zero full sequence");
        rig.shutdown().await;
    }

    // Part B: Partial / unverified batch (J2 ignores CMD_SET_ZERO) ->
    // keeps Reconciling + .calibration_commit.pending + hardware recovery guidance at restart (B3).
    {
        let rig = HwRig::new(RtOwner::Executor, true, None, |_| {}).await;
        let mut ws = rig.connect_ws().await;
        let (gen, epoch) = ws_acquire_lease(&mut ws, "c-mz-part").await;
        rig.bus.with(|s| {
            s.motors[1].ignore_set_zero = true;
            // 0.03 rad is well within DEFAULT_REST_POSE_TOLERANCE_RAD (0.05 rad)
            // yet outside MOTOR_ZERO_VERIFY_MRAD (0.010 rad), so raw verification fails cleanly.
            s.motors[1].q = 0.03;
            s.push_all_feedback();
        });
        tokio::time::sleep(Duration::from_millis(25)).await;

        let err = ws_cmd(
            &mut ws,
            1,
            gen,
            epoch,
            motor_zero::PATH,
            json!({ "joints": [1, 2], "method": "motor", "dry_run": false }),
        )
        .await;
        assert_eq!(err["type"], "command_reject", "{err}");
        assert_eq!(err["code"], "MOTOR_ZERO_UNVERIFIED");
        assert_eq!(err["applied"], json!([1]));
        assert_eq!(err["uncertain"], json!([2]));
        assert_eq!(rig.hold_state().await, Some(HoldState::Reconciling));
        let marker = pending_commit_marker_path(&rig.calib_path);
        assert!(marker.exists(), "pending marker must survive partial hardware motor zero");

        // Fresh supervisor startup sees hardware-specific recovery guidance and quarantines.
        let fresh_sup = ArmSupervisor::new(ArmConfig::rebot_dm());
        armd::calibration::load_into(&fresh_sup, &rig.calib_path, Some(&rig.setup_path))
            .await
            .unwrap();
        let pub_fresh = fresh_sup.published_config().await.unwrap();
        assert!(pub_fresh.calibration_quarantined);
        let load_err = pub_fresh.calibration_load_error.as_deref().unwrap_or("");
        assert!(
            load_err.contains("hardware motor-zero") && load_err.contains("physical arm pose"),
            "expected hardware motor-zero recovery guidance, got: {load_err}"
        );
        let inner = fresh_sup.inner().read().await;
        assert_eq!(
            inner
                .core
                .config_held(ConfigDomain::Calibration)
                .map(|h| h.state),
            Some(HoldState::Reconciling)
        );
        drop(inner);

        rig.assert_zero_alloc("hardware motor_zero partial reconcile");
        rig.shutdown().await;
    }
}

// ============================================================================
// 7. Driver-backed software zero & joint_zero/restore on executor (0 RT alloc)
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn driver_backed_software_zero_and_joint_zero_restore_on_executor() {
    let rig = HwRig::new(RtOwner::Executor, true, None, |_| {}).await;
    let mut ws = rig.connect_ws().await;
    let (gen, epoch) = ws_acquire_lease(&mut ws, "c-swz").await;

    // Shift J1 slightly (within max_offset_deg) so software zero produces a non-zero offset.
    rig.bus.with(|s| {
        s.motors[0].q += 0.08;
    });
    tokio::time::sleep(Duration::from_millis(30)).await;

    let sw = ws_cmd(
        &mut ws,
        1,
        gen,
        epoch,
        motor_zero::PATH,
        json!({ "joints": [1], "method": "software", "dry_run": false }),
    )
    .await;
    assert_eq!(sw["type"], "command_ack", "{sw}");
    assert_eq!(sw["zero_method"], "software");
    assert_eq!(sw["joints"][0]["verified"], true);
    let backup_name = sw["restore"]["backup"].as_str().unwrap().to_string();
    let off_after = rig.sup.published_config().await.unwrap().joint_zero_offsets[0];
    assert!(
        (off_after - 0.08).abs() < 0.02,
        "expected ~0.08 rad software offset, got {off_after}"
    );
    // Motor itself was NOT flash-zeroed.
    assert_eq!(rig.bus.with(|s| s.motors[0].set_zero_count), 0);

    // Restore backup -> offset returns to 0.0 on disk, PublishedConfigHub, and ZeroOffsetDriver!
    let rest = ws_cmd(
        &mut ws,
        2,
        gen,
        epoch,
        motor_zero::RESTORE_PATH,
        json!({ "backup": backup_name }),
    )
    .await;
    assert_eq!(rest["type"], "command_ack", "{rest}");
    assert_eq!(
        rig.sup.published_config().await.unwrap().joint_zero_offsets[0],
        0.0
    );
    assert_eq!(rig.hold_state().await, None);

    rig.assert_zero_alloc("driver-backed software zero + restore");
    rig.shutdown().await;
}

// ============================================================================
// 8. B3: Post-HW dir-sync failure stays Reconciling without config-only retry
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn hardware_motor_zero_dir_sync_failure_stays_reconciling_without_config_retry() {
    let cfg = ArmConfig::rebot_dm();
    let seed_qual = build_valid_qualification(&cfg, &vec![0.0; cfg.dof], &[0, 1], 2000);
    let mut seed_cal = CalibrationFile::default();
    seed_cal.joint_zero = Some(armd::joint_zero::JointZero::new(
        &[0.02, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0],
        "software",
        "rest",
    ));
    seed_cal.set_mit_record(&seed_qual);
    let rig = HwRig::new(RtOwner::Executor, true, None, move |inner| {
        inner.calibration = Some(seed_cal);
        inner.joint_zero_offsets = vec![0.02, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0];
    })
    .await;
    let init_cal = rig.sup.published_config().await.unwrap().calibration.clone().unwrap();
    std::fs::write(&rig.calib_path, serde_json::to_string_pretty(&init_cal).unwrap()).unwrap();

    let fail_once = Arc::new(AtomicBool::new(true));
    rig.sup.set_config_commit_test_hooks(ConfigCommitTestHooks {
        fail_dir_sync_once: Some(fail_once),
        reconcile_retry_interval: Some(Duration::from_millis(10)),
        reconcile_max_attempts: Some(3),
        ..Default::default()
    });
    let mut ws = rig.connect_ws().await;
    let (gen, epoch) = ws_acquire_lease(&mut ws, "c-dirsync").await;

    // Zero J2: hardware motor zero emits CMD_SET_ZERO and persists calibration.json (with
    // invalidated MIT record + updated joint_zero), hitting `fail_dir_sync_once`.
    let err = ws_cmd(
        &mut ws,
        1,
        gen,
        epoch,
        motor_zero::PATH,
        json!({ "joints": [2], "method": "motor", "dry_run": false }),
    )
    .await;
    assert_eq!(err["type"], "command_reject", "{err}");
    assert_eq!(err["code"], "PERSIST_FAILED");
    assert_eq!(rig.hold_state().await, Some(HoldState::Reconciling));
    assert!(pending_commit_marker_path(&rig.calib_path).exists());

    // Wait longer than reconcile_retry_interval * max_attempts: MUST NOT auto-clear (B3)!
    tokio::time::sleep(Duration::from_millis(100)).await;
    assert_eq!(
        rig.hold_state().await,
        Some(HoldState::Reconciling),
        "post-HW dir-sync failure must NOT be auto-cleared by config-only retry"
    );
    assert!(pending_commit_marker_path(&rig.calib_path).exists());

    rig.assert_zero_alloc("hardware motor_zero dir-sync failure");
    rig.shutdown().await;
}

// ============================================================================
// 9. M5: Damiao 0xD command-timeout expiry on FakeBus + Executor latches fault & fences motion
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn damiao_timeout_0xd_expiry_latches_fault_and_fences_motion_on_executor() {
    assert_eq!(MIT_DEADLINE, Duration::from_millis(45), "45 ms MIT_DEADLINE must stay intact");

    let clock = MockClock::new();
    let rig = HwRig::new(RtOwner::Executor, true, Some(clock.clone()), |_| {}).await;

    // Feed periodic timestamps via MockClock advancing gently so executor stays fresh.
    let feed = |bus: &FakeBus, clk: &MockClock, step_ms: u64| {
        clk.advance(Duration::from_millis(step_ms));
        bus.with(|s| s.push_all_feedback());
    };
    for _ in 0..4 {
        feed(&rig.bus, &clock, 5);
        tokio::time::sleep(Duration::from_millis(8)).await;
    }

    let mut ws = rig.connect_ws().await;
    let (gen, epoch) = ws_acquire_lease(&mut ws, "c-0xd").await;

    // Commission RAM TIMEOUT = 1000 counts (50 ms documented) on J1 via executor!
    let bus_bg = rig.bus.clone();
    let clk_bg = clock.clone();
    let stop_bg = Arc::new(AtomicBool::new(false));
    let stop_bg_c = stop_bg.clone();
    let bg = tokio::spawn(async move {
        while !stop_bg_c.load(Ordering::Acquire) {
            clk_bg.advance(Duration::from_millis(1));
            bus_bg.with(|s| {
                if s.rx.len() < 16 && s.pending.len() < 16 {
                    s.push_all_feedback();
                }
            });
            tokio::time::sleep(Duration::from_millis(4)).await;
        }
    });

    let wr = ws_cmd(
        &mut ws,
        1,
        gen,
        epoch,
        mit::TIMEOUT_PATH,
        json!({ "joints": [1], "timeout_counts": 1000, "dry_run": false }),
    )
    .await;
    assert_eq!(wr["type"], "command_ack", "{wr}");
    assert_eq!(rig.bus.with(|s| s.motors[0].reg_u32(damiao::RID_TIMEOUT)), 1000);

    // Enable torque on J1..J7 via WS so the motor watchdog is armed.
    let tq_on = ws_cmd(
        &mut ws,
        2,
        gen,
        epoch,
        "/api/v1/torque",
        json!({ "enable": true }),
    )
    .await;
    let diag_fault = rig.sup.hub().latest().map(|s| (s.obs.safety_state.clone(), s.obs.fault));
    assert_eq!(tq_on["type"], "command_ack", "{tq_on} fault={diag_fault:?}");

    // Pause background feeder, advance MockClock past 1000 * 50 us = 50 ms without commands ->
    // FakeMotor watchdog expires, disables motor 1, sets state_override = Some(0xD), and emits 0xD feedback!
    stop_bg.store(true, Ordering::Release);
    let _ = bg.await;

    clock.advance(Duration::from_millis(80));
    rig.bus.with(|s| {
        s.run_watchdogs();
        assert_eq!(s.motors[0].state_override, Some(0xD), "motor 1 watchdog must trip 0xD");
        assert!(!s.motors[0].enabled);
        s.push_all_feedback();
    });

    // Wait for executor to consume 0xD feedback and latch hardware fault.
    let dl = Instant::now() + Duration::from_secs(2);
    loop {
        rig.bus.with(|s| {
            if s.rx.len() < 16 {
                s.push_all_feedback();
            }
        });
        if let Some(snap) = rig.sup.hub().latest() {
            if matches!(snap.obs.safety_state, SafetyState::Fault(_)) {
                break;
            }
        }
        assert!(Instant::now() < dl, "executor did not latch fault on Damiao 0xD expiry");
        tokio::time::sleep(Duration::from_millis(5)).await;
    }

    // Resume normal feedback without clearing 0xD: fault MUST NOT auto-clear, and rearm is fenced!
    let fault_epoch = rig.sup.hub().latest().unwrap().obs.epoch_id;
    tokio::time::sleep(Duration::from_millis(40)).await;
    assert!(
        matches!(
            rig.sup.hub().latest().unwrap().obs.safety_state,
            SafetyState::Fault(_)
        ),
        "0xD fault must NOT auto-clear"
    );
    let rearm = ws_cmd(
        &mut ws,
        3,
        gen,
        fault_epoch,
        "/api/v1/torque",
        json!({ "enable": true }),
    )
    .await;
    assert_eq!(rearm["type"], "command_reject", "{rearm}");

    rig.assert_zero_alloc("Damiao 0xD expiry fault + fence");
    rig.shutdown().await;
}

// ============================================================================
// 10. m1/m2: Legacy vs Executor WS parity + direct legacy Rust guards + real-follower gate
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn legacy_vs_executor_ws_parity_direct_guards_and_real_follower_gate() {
    // Part A: Paired Legacy vs Executor WS parity on identical FakeBus scenarios.
    async fn run_parity_script(owner: RtOwner) -> Vec<Value> {
        let rig = HwRig::new(owner, true, None, |_| {}).await;
        let mut ws = rig.connect_ws().await;
        let (gen, epoch) = ws_acquire_lease(&mut ws, "c-parity").await;
        let mut out = Vec::new();

        // 1. Invalid timeout body -> 400 INVALID_COMMAND
        out.push(
            ws_cmd(&mut ws, 1, gen, epoch, mit::TIMEOUT_PATH, json!({ "joints": [] })).await,
        );
        // 2. Timeout dry_run -> command_ack with readback
        out.push(
            ws_cmd(
                &mut ws,
                2,
                gen,
                epoch,
                mit::TIMEOUT_PATH,
                json!({ "joints": [1], "dry_run": true }),
            )
            .await,
        );
        // 3. Timeout RAM write -> command_ack
        out.push(
            ws_cmd(
                &mut ws,
                3,
                gen,
                epoch,
                mit::TIMEOUT_PATH,
                json!({ "joints": [1], "timeout_counts": 2000, "dry_run": false }),
            )
            .await,
        );
        // 4. Qualification mismatch -> 409 MIT_QUALIFICATION_MISMATCH
        let pub_cfg = rig.sup.published_config().await.unwrap();
        let bad = build_valid_qualification(&pub_cfg.config, &pub_cfg.joint_zero_offsets, &[0], 1999);
        out.push(
            ws_cmd(&mut ws, 4, gen, epoch, mit::QUALIFICATION_PATH, json!({ "record": bad })).await,
        );
        // 5. Qualification success -> command_ack {stored: true, joints: [1]}
        let good = build_valid_qualification(&pub_cfg.config, &pub_cfg.joint_zero_offsets, &[0], 2000);
        out.push(
            ws_cmd(&mut ws, 5, gen, epoch, mit::QUALIFICATION_PATH, json!({ "record": good })).await,
        );
        // 6. Motor zero dry_run -> command_ack
        out.push(
            ws_cmd(
                &mut ws,
                6,
                gen,
                epoch,
                motor_zero::PATH,
                json!({ "joints": [1], "method": "motor", "dry_run": true }),
            )
            .await,
        );

        rig.assert_zero_alloc("parity script");
        rig.shutdown().await;
        out
    }

    let legacy_res = run_parity_script(RtOwner::Legacy).await;
    let exec_res = run_parity_script(RtOwner::Executor).await;
    assert_eq!(legacy_res.len(), exec_res.len());
    for (idx, (l, e)) in legacy_res.iter().zip(exec_res.iter()).enumerate() {
        assert_eq!(l["type"], e["type"], "step {idx}: legacy={l} exec={e}");
        assert_eq!(l["status"], e["status"], "step {idx}: legacy={l} exec={e}");
        assert_eq!(l["code"], e["code"], "step {idx}: legacy={l} exec={e}");
        assert_eq!(l["stored"], e["stored"], "step {idx}: legacy={l} exec={e}");
        assert_eq!(l["dry_run"], e["dry_run"], "step {idx}: legacy={l} exec={e}");
        assert_eq!(
            l["timeout_counts"], e["timeout_counts"],
            "step {idx}: legacy={l} exec={e}"
        );
        assert_eq!(
            l["zero_method"], e["zero_method"],
            "step {idx}: legacy={l} exec={e}"
        );
    }

    // Part B: Direct Rust legacy guards ALL still fail closed with EXECUTOR_UNSUPPORTED!
    {
        let rig = HwRig::new(RtOwner::Executor, true, None, |_| {}).await;
        let check: armd::calibration::LeaseCheck = Box::new(|_| Ok(()));
        let e1 = motor_zero::run(
            &rig.sup,
            &rig.cal_deps,
            MotorZeroRequest {
                joints: vec![0],
                dry_run: false,
                force: false,
                method: Some("motor".into()),
            },
            &check,
        )
        .await
        .unwrap_err();
        assert_eq!((e1.status, e1.code.as_str()), (409, "EXECUTOR_UNSUPPORTED"));

        let e2 = motor_zero::restore(&rig.sup, &rig.cal_deps, "b1".into(), &check)
            .await
            .unwrap_err();
        assert_eq!((e2.status, e2.code.as_str()), (409, "EXECUTOR_UNSUPPORTED"));

        let e3 = mit::store_qualification(
            &rig.sup,
            &rig.cal_deps,
            CalibrationFile::default(),
            &check,
        )
        .await
        .unwrap_err();
        assert_eq!((e3.status, e3.code.as_str()), (409, "EXECUTOR_UNSUPPORTED"));

        let h = rig.handle.as_ref().unwrap();
        assert!(h
            .set_motor_zero(0)
            .unwrap()
            .unwrap_err()
            .contains("EXECUTOR_UNSUPPORTED"));
        assert!(h
            .set_motor_zero_async(0)
            .await
            .unwrap()
            .unwrap_err()
            .contains("EXECUTOR_UNSUPPORTED"));
        assert!(h
            .set_joint_zero_offsets(vec![0.0; 7])
            .unwrap()
            .unwrap_err()
            .contains("EXECUTOR_UNSUPPORTED"));
        assert!(h
            .set_joint_zero_offsets_async(vec![0.0; 7])
            .await
            .unwrap()
            .unwrap_err()
            .contains("EXECUTOR_UNSUPPORTED"));

        rig.assert_zero_alloc("direct legacy Rust guards");
        rig.shutdown().await;
    }

    // Part C: Real-follower executor startup gate remains closed (executor.rs:101-109).
    {
        armd::rt_core::executor::validate_startup_owner(RtOwner::Executor, "mock")
            .expect("mock follower must remain allowed");
        let err = armd::rt_core::executor::validate_startup_owner(RtOwner::Executor, "damiao:can0")
            .expect_err("real follower must still be refused by executor admission gate");
        assert!(
            err.contains("mock") || err.contains("damiao:can0"),
            "unexpected real-follower startup error: {err}"
        );
    }
}

// ============================================================================
// 11. Code-Review Blocker Regressions (B1 ambiguous reply, B2 direct RT gate,
//     B3 unknown post-start marker preservation, B4 mid-batch rest-pose drift)
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn blocker_regressions_b1_b2_b3_b4() {
    // B1: Duplicate/ambiguous post-TX register ACK (matching value injected twice on REG_WRITE)
    // must NEVER be promoted to Applied -> fails closed as MIT_TIMEOUT_FAILED + uncertain: [1].
    {
        let rig = HwRig::new(RtOwner::Executor, true, None, |_| {}).await;
        let mut ws = rig.connect_ws().await;
        let (gen, epoch) = ws_acquire_lease(&mut ws, "c-b1-dup").await;
        let dup_ack = CanFrame::new_standard(
            0x11,
            &[1, 0, damiao::REG_WRITE, damiao::RID_TIMEOUT, 0xD0, 0x07, 0, 0],
        )
        .unwrap();
        rig.bus.with(|s| {
            s.immediate_replies = true;
            s.inject_on_reg_write.push(dup_ack);
        });
        let bad = ws_cmd(
            &mut ws,
            1,
            gen,
            epoch,
            mit::TIMEOUT_PATH,
            json!({ "joints": [1], "timeout_counts": 2000, "dry_run": false }),
        )
        .await;
        assert_eq!(bad["type"], "command_reject", "{bad}");
        assert_eq!(bad["code"], "MIT_TIMEOUT_FAILED");
        assert_eq!(bad["uncertain"], json!([1]));
        assert_eq!(rig.hold_state().await, Some(HoldState::Reconciling));
        assert!(pending_commit_marker_path(&rig.calib_path).exists());
        rig.assert_zero_alloc("B1 ambiguous duplicate reply");
        rig.shutdown().await;
    }

    // B2: Direct `StartCommissioning(WriteTimeout | MotorZeroWriteVerify)` through the public
    // config dispatcher with missing `txn_id: 0` / `grant_id: None` / unbound marker is refused
    // fail-closed with 0 TX emitted.
    {
        let rig = HwRig::new(RtOwner::Executor, true, None, |_| {}).await;
        let mut ws = rig.connect_ws().await;
        let (gen, epoch) = ws_acquire_lease(&mut ws, "c-b2-direct").await;
        let mut joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
        joints[0] = 0;
        let mut req = armd::mit::CommissioningStartRequest {
            kind: armd::mit::CommissioningOpKind::WriteTimeout,
            joints,
            joint_len: 1,
            timeout_counts: 2000,
            references: [0.0; armd::rt_core::mailbox::MAX_DOF],
            authority: armd::mit::CommissioningAuthority {
                conn_id: 0,
                lease_holder: Some((armd::model::HolderLabel::new("c-b2-direct"), gen)),
                operator_grant: armd::auth::OperatorGrant::unenforced(),
                expected_epoch: epoch,
                expected_stop_gen: 0,
                grant_id: None,
                txn_id: 0,
            },
            txn_deadline: Instant::now() + Duration::from_secs(5),
            prior_disk_mutated: false,
        };
        rig.bus.clear_log();
        let r1 = rig
            .sup
            .dispatch_config_request_owned_reliable(ConfigRequest::StartCommissioning(req))
            .await
            .unwrap();
        assert!(
            matches!(r1, ConfigReply::CommissioningStarted(Err(_))),
            "missing txn_id & grant_id must be refused by RT write gate"
        );
        req.kind = armd::mit::CommissioningOpKind::MotorZeroWriteVerify;
        let r2 = rig
            .sup
            .dispatch_config_request_owned_reliable(ConfigRequest::StartCommissioning(req))
            .await
            .unwrap();
        assert!(
            matches!(r2, ConfigReply::CommissioningStarted(Err(_))),
            "missing txn_id & grant_id must be refused for MotorZeroWriteVerify"
        );
        assert!(
            rig.bus.tx_log().is_empty(),
            "refused direct StartCommissioning must emit 0 TX frames"
        );
        rig.assert_zero_alloc("B2 direct RT write gate");
        rig.shutdown().await;
    }

    // B3: If the RT executor disappears after accepting a TIMEOUT write (unknown post-start
    // outcome while polling), `.calibration_commit.pending` MUST survive on disk (not unlinked).
    {
        let mut rig = HwRig::new(RtOwner::Executor, true, None, |_| {}).await;
        let mut ws = rig.connect_ws().await;
        let (gen, epoch) = ws_acquire_lease(&mut ws, "c-b3-drop").await;
        rig.bus.with(|s| {
            // Stall REG_WRITE ack long enough that we can stop the executor mid-op after TX.
            s.motors[0].no_write_ack = true;
        });
        let handle = rig.handle.take().unwrap();
        let bus_watch = rig.bus.clone();
        let killer = tokio::spawn(async move {
            let dl = Instant::now() + Duration::from_secs(5);
            while Instant::now() < dl {
                let sent = bus_watch.tx_log().iter().any(|f| {
                    f.id == 0x7FF && f.data[2] == damiao::REG_WRITE && f.data[3] == damiao::RID_TIMEOUT
                });
                if sent {
                    tokio::task::spawn_blocking(move || handle.stop_and_join())
                        .await
                        .unwrap();
                    return true;
                }
                tokio::time::sleep(Duration::from_millis(2)).await;
            }
            tokio::task::spawn_blocking(move || handle.stop_and_join())
                .await
                .unwrap();
            false
        });

        let res = ws_cmd(
            &mut ws,
            1,
            gen,
            epoch,
            mit::TIMEOUT_PATH,
            json!({ "joints": [1], "timeout_counts": 2000, "dry_run": false }),
        )
        .await;
        let killed_after_tx = killer.await.unwrap();
        assert!(killed_after_tx, "executor must have sent REG_WRITE before stop");
        assert_eq!(res["type"], "command_reject", "{res}");
        assert!(
            pending_commit_marker_path(&rig.calib_path).exists(),
            "B3: unknown post-start poll failure MUST preserve .calibration_commit.pending on disk"
        );
        rig.shutdown().await;
    }

    // B4: During the 100 ms inter-joint gap of a multi-joint hardware motor_zero (J1, J2),
    // if an arm joint drifts out of rest pose after J1 is zeroed, J2 MUST be refused with
    // ARM_NOT_AT_RESTING_POSE (set_zero_count == 0) while preserving Reconciling + pending marker.
    {
        let rig = HwRig::new(RtOwner::Executor, true, None, |_| {}).await;
        let mut ws = rig.connect_ws().await;
        let (gen, epoch) = ws_acquire_lease(&mut ws, "c-b4-drift").await;
        let bus_watch = rig.bus.clone();
        let drifter = tokio::spawn(async move {
            let dl = Instant::now() + Duration::from_secs(5);
            while Instant::now() < dl {
                let j1_zeroed = bus_watch.with(|s| s.motors[0].set_zero_count >= 1);
                if j1_zeroed {
                    // Disturb J3 well outside rest_pose_tolerance_rad (0.05 rad) during the 100 ms gap.
                    bus_watch.with(|s| {
                        s.motors[2].q += 0.25;
                        s.push_all_feedback();
                    });
                    return true;
                }
                tokio::time::sleep(Duration::from_millis(2)).await;
            }
            false
        });

        let err = ws_cmd(
            &mut ws,
            1,
            gen,
            epoch,
            motor_zero::PATH,
            json!({ "joints": [1, 2], "method": "motor", "dry_run": false }),
        )
        .await;
        let drifted = drifter.await.unwrap();
        assert!(drifted, "drifter must have disturbed arm after J1 zero");
        assert_eq!(err["type"], "command_reject", "{err}");
        assert_eq!(err["code"], "ARM_NOT_AT_RESTING_POSE", "{err}");
        assert_eq!(rig.bus.with(|s| s.motors[0].set_zero_count), 1);
        assert_eq!(
            rig.bus.with(|s| s.motors[1].set_zero_count),
            0,
            "B4: J2 MUST NOT be zeroed after arm left resting pose during inter-joint gap"
        );
        assert_eq!(rig.hold_state().await, Some(HoldState::Reconciling));
        assert!(pending_commit_marker_path(&rig.calib_path).exists());
        rig.assert_zero_alloc("B4 mid-batch rest-pose drift refusal");
        rig.shutdown().await;
    }
}

/// Blocker 4 regression (`src/mit.rs` + `src/driver.rs` TIMEOUT write deadline & drain gaps):
/// 1. Authority revocation AND transaction-deadline expiry injected strictly AFTER section-6
///    bookkeeping and immediately BEFORE `can.transmit` MUST abort before any `REG_WRITE` hits
///    the CAN bus (0 `REG_WRITE` frames, marker cleanly removed).
/// 2. A late `REG_WRITE` ACK arriving at/after `COMM_REG_REPLY_TIMEOUT` (50 ms) MUST NOT be
///    accepted even when already present in the RX buffer on the poll.
/// 3. A duplicate `REG_WRITE` ACK sitting beyond a single release's 16-frame RX drain budget
///    MUST be caught by completing the RX drain before accepting, failing closed as `Uncertain`.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn cf1_blocker4_timeout_write_pre_tx_revocation_late_reply_and_duplicate_beyond_drain_budget() {
    // Part A1: Authority revocation (E-stop) injected strictly AFTER section-6 bookkeeping,
    // immediately before `can.transmit` on `REG_WRITE RID_TIMEOUT`.
    {
        let dir = tempfile::tempdir().unwrap();
        let calib_path = dir.path().join("calibration.json");
        let setup_path = dir.path().join("grasp_setup.json");
        let poses_path = dir.path().join("poses.json");
        let gripper_calib_path = dir.path().join("gripper_calib.json");

        let cfg = ArmConfig::rebot_dm();
        let rest_pos = cfg.resting_positions();
        let limits = b601_limits();
        let bus = FakeBus::new(&limits);
        bus.with(|s| {
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = rest_pos.get(i).copied().unwrap_or(0.0);
                m.v = 0.0;
                m.enabled = false;
            }
            s.immediate_replies = true;
            s.push_all_feedback();
        });

        let hook_fired = Arc::new(AtomicBool::new(false));
        let estop_cell: Arc<std::sync::Mutex<Option<Arc<armd::rt_core::urgent::EstopState>>>> =
            Arc::new(std::sync::Mutex::new(None));
        let hook_fired_c = Arc::clone(&hook_fired);
        let estop_cell_c = Arc::clone(&estop_cell);
        let mut raw_drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits)
            .unwrap()
            .with_before_final_tx_gate_hook(Arc::new(move |frame: &CanFrame| {
                if frame.id == 0x7FF
                    && frame.data[2] == damiao::REG_WRITE
                    && frame.data[3] == damiao::RID_TIMEOUT
                    && !hook_fired_c.swap(true, Ordering::AcqRel)
                {
                    if let Some(es) = estop_cell_c.lock().unwrap().as_ref() {
                        es.request_and_latch();
                    }
                }
            }));
        let _ = raw_drv.read_joint_states();
        let zero_drv = ZeroOffsetDriver::new(Box::new(raw_drv), &vec![0.0; cfg.dof]);
        let sup = Arc::new(ArmSupervisor::new(cfg.clone()).with_driver(Box::new(zero_drv)));
        *estop_cell.lock().unwrap() = Some(Arc::clone(&sup.estop_state));
        {
            let mut inner = sup.inner().write().await;
            inner.poses.path = Some(poses_path);
            inner.gripper_calib_path = Some(gripper_calib_path);
            let gate = inner.stopped_disable_gate();
            if let Some(d) = inner.driver.as_mut() {
                let _ = d.with_mut(|drv| drv.set_stopped_disable_gate(gate));
            }
        }
        for _ in 0..4 {
            bus.with(|s| s.push_all_feedback());
            let mut inner = sup.inner().write().await;
            if let Some(d) = inner.driver.as_mut() {
                let _ = d.with_mut(|drv| drv.read_joint_states());
            }
        }
        bus.clear_log();

        let feeder_stop = Arc::new(AtomicBool::new(false));
        let b = bus.clone();
        let stop = Arc::clone(&feeder_stop);
        let feeder_jh = tokio::spawn(async move {
            while !stop.load(Ordering::Acquire) {
                b.with(|s| {
                    if s.rx.len() < 16 && s.pending.len() < 16 {
                        s.push_all_feedback();
                    }
                });
                tokio::time::sleep(Duration::from_millis(8)).await;
            }
        });

        let mut ecfg = ExecutorConfig::for_tests(Duration::from_millis(8));
        ecfg.alloc_timing = true;
        let handle = sup.start_executor(ecfg).await.unwrap();
        tokio::time::sleep(Duration::from_millis(40)).await;

        let det_up = armd::api::proxy::DetectorUpstream::parse("http://127.0.0.1:9").unwrap();
        let grasp_deps = GraspDeps::http(det_up.clone(), Some(setup_path));
        let cal_deps = CalibrationDeps::new(calib_path.clone(), None, det_up.clone());
        let router = armd::api::create_router_calibration(
            sup.clone(),
            det_up,
            grasp_deps,
            armd::diagnostics::DiagnosticsState::global(),
            cal_deps,
            OperatorAuth::disabled(),
        );
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        let server_jh = tokio::spawn(async move {
            let _ = axum::serve(listener, router).await;
        });

        let (mut ws, _) = tokio_tungstenite::connect_async(format!("ws://{addr}/api/v1/ws"))
            .await
            .unwrap();
        let (gen, epoch) = ws_acquire_lease(&mut ws, "b4-pre-tx-revoke").await;
        let res = ws_cmd(
            &mut ws,
            1,
            gen,
            epoch,
            mit::TIMEOUT_PATH,
            json!({ "joints": [1], "timeout_counts": 2000, "dry_run": false }),
        )
        .await;
        assert!(hook_fired.load(Ordering::Acquire), "pre-TX hook must have run right before REG_WRITE");
        assert_eq!(res["type"], "command_reject", "revoked pre-TX must reject: {res}");
        assert_eq!(res["code"], "STOPPED", "{res}");
        let reg_writes = bus.tx_log().iter().filter(|f| {
            f.id == 0x7FF && f.data[2] == damiao::REG_WRITE && f.data[3] == damiao::RID_TIMEOUT
        }).count();
        assert_eq!(
            reg_writes, 0,
            "authority revocation immediately before send MUST emit 0 REG_WRITE frames"
        );
        assert_eq!(
            bus.with(|s| s.motors[0].reg_u32(damiao::RID_TIMEOUT)),
            0,
            "motor TIMEOUT RAM must remain untouched"
        );
        assert!(
            !pending_commit_marker_path(&calib_path).exists(),
            "unmutated pre-TX revocation should cleanly remove .calibration_commit.pending"
        );

        feeder_stop.store(true, Ordering::Release);
        let _ = feeder_jh.await;
        server_jh.abort();
        tokio::task::spawn_blocking(move || handle.stop_and_join())
            .await
            .unwrap();
    }

    // Part A2: Transaction deadline expiry injected strictly AFTER bookkeeping and immediately
    // before `can.transmit` MUST abort before any `REG_WRITE` hits the bus.
    {
        use std::sync::atomic::AtomicU64;
        struct OffsetExecClock {
            base: Instant,
            offset_ns: AtomicU64,
        }
        impl armd::rt_core::executor::ExecutorClock for OffsetExecClock {
            fn now(&self) -> Instant {
                Instant::now() + Duration::from_nanos(self.offset_ns.load(Ordering::Acquire))
            }
            fn now_ns(&self) -> u64 {
                self.base
                    .elapsed()
                    .as_nanos()
                    .min(u128::from(u64::MAX)) as u64
            }
            fn sleep_until_ns(&self, target_ns: u64, stop: &AtomicBool) -> bool {
                let now = self.now_ns();
                if now >= target_ns {
                    return false;
                }
                let rem = Duration::from_nanos(target_ns - now).min(Duration::from_millis(5));
                if !stop.load(Ordering::Acquire) {
                    std::thread::sleep(rem);
                }
                true
            }
            fn sleep(&self, dur: Duration, stop: &AtomicBool) {
                if !stop.load(Ordering::Acquire) {
                    std::thread::sleep(dur.min(Duration::from_millis(5)));
                }
            }
        }

        let dir = tempfile::tempdir().unwrap();
        let calib_path = dir.path().join("calibration.json");
        let setup_path = dir.path().join("grasp_setup.json");
        let poses_path = dir.path().join("poses.json");
        let gripper_calib_path = dir.path().join("gripper_calib.json");

        let mut cfg = ArmConfig::rebot_dm();
        cfg.lease_timeout_ms = 60_000;
        let rest_pos = cfg.resting_positions();
        let limits = b601_limits();
        let bus = FakeBus::new(&limits);
        bus.with(|s| {
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = rest_pos.get(i).copied().unwrap_or(0.0);
                m.v = 0.0;
                m.enabled = false;
            }
            s.immediate_replies = true;
            s.push_all_feedback();
        });

        let exec_clock = Arc::new(OffsetExecClock {
            base: Instant::now(),
            offset_ns: AtomicU64::new(0),
        });
        let hook_fired = Arc::new(AtomicBool::new(false));
        let hook_fired_c = Arc::clone(&hook_fired);
        let clock_c = Arc::clone(&exec_clock);
        let mut raw_drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits)
            .unwrap()
            .with_before_final_tx_gate_hook(Arc::new(move |frame: &CanFrame| {
                if frame.id == 0x7FF
                    && frame.data[2] == damiao::REG_WRITE
                    && frame.data[3] == damiao::RID_TIMEOUT
                    && !hook_fired_c.swap(true, Ordering::AcqRel)
                {
                    // Jump owner clock +20 s (> 15 s COMMISSIONING_TXN_TIMEOUT) strictly AFTER
                    // section-6 bookkeeping and right BEFORE the terminal pre-TX check + transmit.
                    clock_c.offset_ns.store(20_000_000_000, Ordering::Release);
                }
            }));
        let _ = raw_drv.read_joint_states();
        let zero_drv = ZeroOffsetDriver::new(Box::new(raw_drv), &vec![0.0; cfg.dof]);
        let sup = Arc::new(ArmSupervisor::new(cfg.clone()).with_driver(Box::new(zero_drv)));
        {
            let mut inner = sup.inner().write().await;
            inner.poses.path = Some(poses_path);
            inner.gripper_calib_path = Some(gripper_calib_path);
            let gate = inner.stopped_disable_gate();
            if let Some(d) = inner.driver.as_mut() {
                let _ = d.with_mut(|drv| drv.set_stopped_disable_gate(gate));
            }
        }
        for _ in 0..4 {
            bus.with(|s| s.push_all_feedback());
            let mut inner = sup.inner().write().await;
            if let Some(d) = inner.driver.as_mut() {
                let _ = d.with_mut(|drv| drv.read_joint_states());
            }
        }
        bus.clear_log();

        let feeder_stop = Arc::new(AtomicBool::new(false));
        let b = bus.clone();
        let stop = Arc::clone(&feeder_stop);
        let feeder_jh = tokio::spawn(async move {
            while !stop.load(Ordering::Acquire) {
                b.with(|s| {
                    if s.rx.len() < 16 && s.pending.len() < 16 {
                        s.push_all_feedback();
                    }
                });
                tokio::time::sleep(Duration::from_millis(8)).await;
            }
        });

        let handle = sup
            .start_executor_with_hooks(
                ExecutorConfig::for_tests(Duration::from_millis(8)),
                armd::rt_core::executor::PersistTestHooks {
                    clock: Some(exec_clock),
                    ..Default::default()
                },
            )
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(40)).await;

        let det_up = armd::api::proxy::DetectorUpstream::parse("http://127.0.0.1:9").unwrap();
        let grasp_deps = GraspDeps::http(det_up.clone(), Some(setup_path));
        let cal_deps = CalibrationDeps::new(calib_path.clone(), None, det_up.clone());
        let router = armd::api::create_router_calibration(
            sup.clone(),
            det_up,
            grasp_deps,
            armd::diagnostics::DiagnosticsState::global(),
            cal_deps,
            OperatorAuth::disabled(),
        );
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        let server_jh = tokio::spawn(async move {
            let _ = axum::serve(listener, router).await;
        });

        let (mut ws, _) = tokio_tungstenite::connect_async(format!("ws://{addr}/api/v1/ws"))
            .await
            .unwrap();
        let (gen, epoch) = ws_acquire_lease(&mut ws, "b4-txn-dl").await;
        let res = ws_cmd(
            &mut ws,
            1,
            gen,
            epoch,
            mit::TIMEOUT_PATH,
            json!({ "joints": [1], "timeout_counts": 2000, "dry_run": false }),
        )
        .await;
        assert!(hook_fired.load(Ordering::Acquire), "pre-TX hook must have run right before REG_WRITE");
        assert_eq!(res["type"], "command_reject", "expired txn_deadline before send must reject: {res}");
        assert_eq!(res["code"], "MIT_TIMEOUT_FAILED", "{res}");
        let reg_writes = bus.tx_log().iter().filter(|f| {
            f.id == 0x7FF && f.data[2] == damiao::REG_WRITE && f.data[3] == damiao::RID_TIMEOUT
        }).count();
        assert_eq!(
            reg_writes, 0,
            "expired txn_deadline immediately before send MUST emit 0 REG_WRITE frames"
        );
        assert_eq!(bus.with(|s| s.motors[0].reg_u32(damiao::RID_TIMEOUT)), 0);
        assert!(
            !pending_commit_marker_path(&calib_path).exists(),
            "unmutated pre-TX deadline expiry should cleanly remove .calibration_commit.pending"
        );

        feeder_stop.store(true, Ordering::Release);
        let _ = feeder_jh.await;
        server_jh.abort();
        tokio::task::spawn_blocking(move || handle.stop_and_join())
            .await
            .unwrap();
    }

    // Part B: Late reply (arriving after `COMM_REG_REPLY_TIMEOUT` = 50 ms) MUST NOT be accepted
    // even when already present in the RX buffer during `AwaitWriteAck`.
    {
        let rig = HwRig::new(RtOwner::Executor, true, None, |_| {}).await;
        let mut ws = rig.connect_ws().await;
        let (gen, epoch) = ws_acquire_lease(&mut ws, "b4-late-reply").await;

        // Pause the periodic feeder so the first RX read in AwaitWriteAck triggers our 70 ms
        // delayed valid ACK strictly past the 50 ms COMM_REG_REPLY_TIMEOUT.
        rig.bus.with(|s| {
            s.immediate_replies = true;
            s.delay_next_reg_write_ack = Some(Duration::from_millis(70));
        });

        let res = ws_cmd(
            &mut ws,
            1,
            gen,
            epoch,
            mit::TIMEOUT_PATH,
            json!({ "joints": [1], "timeout_counts": 2000, "dry_run": false }),
        )
        .await;
        assert_eq!(
            res["type"], "command_reject",
            "late REG_WRITE ACK past req_deadline MUST be rejected, never accepted: {res}"
        );
        assert_eq!(res["code"], "MIT_TIMEOUT_FAILED", "{res}");
        assert_eq!(rig.hold_state().await, Some(HoldState::Reconciling));
        assert!(pending_commit_marker_path(&rig.calib_path).exists());
        rig.shutdown().await;
    }

    // Part C: Duplicate reply sitting BEYOND the 16-frame per-release RX drain budget MUST be
    // caught by completing the RX drain before accepting, failing closed as Uncertain/Ambiguous.
    {
        let rig = HwRig::new(RtOwner::Executor, true, None, |_| {}).await;
        let mut ws = rig.connect_ws().await;
        let (gen, epoch) = ws_acquire_lease(&mut ws, "b4-dup-beyond-drain").await;

        rig.bus.with(|s| {
            s.immediate_replies = true;
            let [lo, hi] = 1u16.to_le_bytes();
            let val = 2000u32.to_le_bytes();
            let first_ack = CanFrame::new_standard(
                0x11,
                &[lo, hi, damiao::REG_WRITE, damiao::RID_TIMEOUT, val[0], val[1], val[2], val[3]],
            )
            .unwrap();
            // Queue: [ACK #1, 19 filler feedback frames (> 16-frame drain budget)], then
            // `FakeBus::handle` appends the real ACK #2 at index 20 (frame #21!).
            s.inject_on_reg_write.clear();
            s.inject_on_reg_write.push(first_ack);
            for _ in 0..19 {
                s.inject_on_reg_write.push(s.motors[1].feedback_frame());
            }
        });

        let res = ws_cmd(
            &mut ws,
            1,
            gen,
            epoch,
            mit::TIMEOUT_PATH,
            json!({ "joints": [1], "timeout_counts": 2000, "dry_run": false }),
        )
        .await;
        assert_eq!(
            res["type"], "command_reject",
            "duplicate ACK beyond 16-frame RX drain budget MUST fail closed, never accept: {res}"
        );
        assert_eq!(res["code"], "MIT_TIMEOUT_FAILED", "{res}");
        assert!(
            res["message"]
                .as_str()
                .unwrap_or("")
                .contains("ambiguous"),
            "expected ambiguous duplicate rejection message, got: {res}"
        );
        assert_eq!(rig.hold_state().await, Some(HoldState::Reconciling));
        assert!(pending_commit_marker_path(&rig.calib_path).exists());
        rig.assert_zero_alloc("Blocker 4 duplicate beyond RX drain budget");
        rig.shutdown().await;
    }
}

// ============================================================================
// 11. Fix 1: Disarmed idle TIMEOUT commissioning actively gathers K=3 rest evidence
//     via bounded read-only 0xCC polls (no artificial unsolicited feedback feeder)
// ============================================================================

/// Builds an executor-owned 50 Hz `DamiaoArmDriver<FakeBus>` rig with NO unsolicited feedback
/// feeder — motors reply ONLY to host frames (`0xCC`, `0x33`, `0x55`), matching real B601-DM
/// idle disarmed behavior where only the 1-slot `RefreshScheduler` runs.
async fn spawn_disarmed_idle_50hz_rig(
    configure_bus: impl FnOnce(&mut mit_support::FakeBusState, &[f32]),
) -> HwRig {
    let dir = tempfile::tempdir().unwrap();
    let calib_path = dir.path().join("calibration.json");
    let setup_path = dir.path().join("grasp_setup.json");
    let poses_path = dir.path().join("poses.json");
    let gripper_calib_path = dir.path().join("gripper_calib.json");

    let mut cfg = ArmConfig::rebot_dm();
    cfg.lease_timeout_ms = 60_000;
    let rest_pos = cfg.resting_positions();
    let limits = b601_limits();
    let bus = FakeBus::new(&limits);
    bus.with(|s| {
        for (i, m) in s.motors.iter_mut().enumerate() {
            m.q = rest_pos.get(i).copied().unwrap_or(0.0);
            m.v = 0.0;
            m.enabled = false;
        }
        configure_bus(s, &rest_pos);
        // Single initial feedback snapshot at boot, no unsolicited feeder afterward.
        s.push_all_feedback();
    });

    let mut raw_drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap();
    let _ = raw_drv.read_joint_states();
    let zero_drv = ZeroOffsetDriver::new(Box::new(raw_drv), &vec![0.0; cfg.dof]);
    let sup = Arc::new(ArmSupervisor::new(cfg.clone()).with_driver(Box::new(zero_drv)));
    {
        let mut inner = sup.inner().write().await;
        inner.poses.path = Some(poses_path);
        inner.gripper_calib_path = Some(gripper_calib_path);
        let gate = inner.stopped_disable_gate();
        if let Some(d) = inner.driver.as_mut() {
            let _ = d.with_mut(|drv| drv.set_stopped_disable_gate(gate));
        }
    }
    bus.clear_log();

    let mut ecfg = ExecutorConfig::for_tests(Duration::from_millis(20));
    ecfg.alloc_timing = true;
    let handle = sup.start_executor(ecfg).await.unwrap();
    let dl = Instant::now() + Duration::from_secs(3);
    while sup.hub().latest().is_none() {
        assert!(Instant::now() < dl, "executor never published initial snapshot");
        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    let det_up = armd::api::proxy::DetectorUpstream::parse("http://127.0.0.1:9").unwrap();
    let grasp_deps = GraspDeps::http(det_up.clone(), Some(setup_path.clone()));
    let cal_deps = CalibrationDeps::new(calib_path.clone(), Some(setup_path.clone()), det_up.clone());
    let router = armd::api::create_router_calibration(
        sup.clone(),
        det_up,
        grasp_deps,
        armd::diagnostics::DiagnosticsState::global(),
        cal_deps.clone(),
        OperatorAuth::disabled(),
    );
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let server_jh = tokio::spawn(async move {
        let _ = axum::serve(listener, router).await;
    });
    let ws_url = format!("ws://{addr}/api/v1/ws");

    HwRig {
        sup,
        bus,
        handle: Some(handle),
        feeder_stop: Arc::new(AtomicBool::new(false)),
        feeder_jh: None,
        server_jh,
        ws_url,
        _dir: dir,
        calib_path,
        setup_path,
        cal_deps,
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn mit_timeout_disarmed_idle_10s_at_rest_succeeds_all_axes_applied() {
    let rig = spawn_disarmed_idle_50hz_rig(|_, _| {}).await;

    // Sit disarmed and Idle for 10 s at 50 Hz with only the 1-slot RefreshScheduler running.
    tokio::time::sleep(Duration::from_secs(10)).await;

    // Connect, acquire lease, and commission TIMEOUT = 2000 counts on J1-J6 (RAM only, torque off).
    let mut ws = rig.connect_ws().await;
    let (gen, epoch) = ws_acquire_lease(&mut ws, "c-idle-10s").await;

    rig.bus.clear_log();
    let wr = ws_cmd(
        &mut ws,
        1,
        gen,
        epoch,
        mit::TIMEOUT_PATH,
        json!({ "joints": [1, 2, 3, 4, 5, 6], "timeout_counts": 2000, "dry_run": false }),
    )
    .await;
    eprintln!("FAKEBUS_TIMEOUT_IDLE_10S_OUTPUT={wr}");
    assert_eq!(
        wr["type"], "command_ack",
        "disarmed 10s-idle arm at rest must gather rest evidence and succeed: {wr}"
    );
    assert_eq!(wr["dry_run"], false);
    assert_eq!(wr["timeout_counts"], 2000);
    let comm = wr["commissioning"].as_array().unwrap();
    assert_eq!(comm.len(), 6, "all 6 arm axes J1-J6 must be Applied: {wr}");
    for (idx, row) in comm.iter().enumerate() {
        let j = idx + 1;
        assert_eq!(row["before"]["joint"], j, "row {idx}: {row}");
        assert_eq!(row["written_raw"], 2000, "row {idx}: {row}");
        assert_eq!(row["ack_raw"], 2000, "row {idx}: {row}");
        assert_eq!(row["readback"]["joint"], j, "row {idx}: {row}");
        assert_eq!(row["readback"]["timeout_raw"], 2000, "row {idx}: {row}");
    }

    let log = rig.bus.tx_log();
    assert_no_flash_write(&log);
    let cc_probes = log
        .iter()
        .filter(|f| f.id == 0x7FF && f.data[2] == 0xCC)
        .count();
    assert!(
        cc_probes >= 18,
        "expected bounded read-only 0xCC rest-evidence probes across J1-J6 (got {cc_probes})"
    );
    let writes: Vec<CanFrame> = log
        .iter()
        .copied()
        .filter(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE && f.data[3] == damiao::RID_TIMEOUT)
        .collect();
    assert_eq!(writes.len(), 6, "expected exactly 1 RAM REG_WRITE per axis J1-J6");
    for j in 0..6 {
        assert_eq!(
            rig.bus.with(|s| s.motors[j].reg_u32(damiao::RID_TIMEOUT)),
            2000,
            "motor J{} RAM RID_TIMEOUT must equal 2000",
            j + 1
        );
    }
    // J7 (gripper) must remain untouched (TIMEOUT = 0) and rejected if requested.
    assert_eq!(rig.bus.with(|s| s.motors[6].reg_u32(damiao::RID_TIMEOUT)), 0);
    let j7_err = ws_cmd(
        &mut ws,
        2,
        gen,
        epoch,
        mit::TIMEOUT_PATH,
        json!({ "joints": [7], "timeout_counts": 2000, "dry_run": false }),
    )
    .await;
    assert_eq!(j7_err["type"], "command_reject", "J7 must be excluded: {j7_err}");
    assert_eq!(j7_err["status"], 400);

    assert!(!pending_commit_marker_path(&rig.calib_path).exists());
    assert_eq!(rig.hold_state().await, None);
    rig.assert_zero_alloc("disarmed idle 10s TIMEOUT commissioning");
    rig.shutdown().await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn mit_timeout_disarmed_idle_one_axis_off_rest_rejected_zero_writes() {
    // Subcase A: axis J3 off rest after idle refresh -> rejected with 0 writes.
    // Subcase B: axis J3 moves off rest while its cached feedback is stale (> 100 ms), so
    // GatherRestEvidence must actively probe 0xCC and still reject with 0 writes.
    for move_after_stale in [false, true] {
        let rig = spawn_disarmed_idle_50hz_rig(|s, rest_pos| {
            if !move_after_stale {
                s.motors[2].q = rest_pos[2] + 0.35;
            }
        })
        .await;

        tokio::time::sleep(Duration::from_millis(350)).await;
        if move_after_stale {
            let rest_j3 = ArmConfig::rebot_dm().resting_positions()[2];
            rig.bus.with(|s| {
                s.motors[2].q = rest_j3 + 0.35;
            });
        }

        let mut ws = rig.connect_ws().await;
        let (gen, epoch) = ws_acquire_lease(&mut ws, "c-off-rest").await;

        rig.bus.clear_log();
        let err = ws_cmd(
            &mut ws,
            1,
            gen,
            epoch,
            mit::TIMEOUT_PATH,
            json!({ "joints": [1, 2, 3, 4, 5, 6], "timeout_counts": 2000, "dry_run": false }),
        )
        .await;
        assert_eq!(
            err["type"], "command_reject",
            "one axis off rest (move_after_stale={move_after_stale}) must reject: {err}"
        );
        assert_eq!(err["status"], 409, "{err}");
        assert_eq!(err["code"], "ARM_NOT_AT_RESTING_POSE", "{err}");
        let log = rig.bus.tx_log();
        assert_no_flash_write(&log);
        assert!(
            !log.iter()
                .any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE),
            "off-rest refusal must emit 0 register writes (move_after_stale={move_after_stale})"
        );
        for j in 0..7 {
            assert_eq!(rig.bus.with(|s| s.motors[j].reg_u32(damiao::RID_TIMEOUT)), 0);
        }
        assert!(!pending_commit_marker_path(&rig.calib_path).exists());
        assert_eq!(rig.hold_state().await, None);
        rig.assert_zero_alloc("disarmed idle one axis off rest");
        rig.shutdown().await;
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn mit_timeout_disarmed_idle_silent_axis_rejected_zero_writes() {
    let rig = spawn_disarmed_idle_50hz_rig(|_, _| {}).await;
    tokio::time::sleep(Duration::from_millis(350)).await;
    // Silence J4 after boot while the arm sits disarmed at rest.
    rig.bus.with(|s| {
        s.motors[3].silent = true;
    });
    tokio::time::sleep(Duration::from_millis(200)).await;

    let mut ws = rig.connect_ws().await;
    let (gen, epoch) = ws_acquire_lease(&mut ws, "c-silent-axis").await;

    rig.bus.clear_log();
    let err = ws_cmd(
        &mut ws,
        1,
        gen,
        epoch,
        mit::TIMEOUT_PATH,
        json!({ "joints": [1, 2, 3, 4, 5, 6], "timeout_counts": 2000, "dry_run": false }),
    )
    .await;
    assert_eq!(
        err["type"], "command_reject",
        "silent axis must reject TIMEOUT commissioning: {err}"
    );
    let status = err["status"].as_u64().unwrap_or(0);
    let code = err["code"].as_str().unwrap_or("");
    assert!(
        matches!((status, code), (409, "ARM_NOT_AT_RESTING_POSE") | (502, "MIT_TIMEOUT_FAILED")),
        "expected fail-closed rest/feedback refusal on silent axis, got: {err}"
    );
    let log = rig.bus.tx_log();
    assert_no_flash_write(&log);
    assert!(
        log.iter().any(|f| f.id == 0x7FF && f.data[2] == 0xCC),
        "expected read-only 0xCC probes before failing closed on silent axis"
    );
    assert!(
        !log.iter()
            .any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE),
        "silent axis refusal must emit 0 register writes"
    );
    for j in 0..7 {
        assert_eq!(rig.bus.with(|s| s.motors[j].reg_u32(damiao::RID_TIMEOUT)), 0);
    }
    assert!(!pending_commit_marker_path(&rig.calib_path).exists());
    assert_eq!(rig.hold_state().await, None);
    rig.assert_zero_alloc("disarmed idle silent axis");
    rig.shutdown().await;

    // Subcase B (Blocker regression): J4 stops replying right before WriteTimeout while its
    // pre-existing K=3 rest window is STILL fresh (< 100 ms). GatherRestEvidence MUST require a
    // causal post-probe reply from every arm axis and refuse with 0 writes instead of reusing
    // pre-gather cached samples.
    let rig2 = HwRig::new(RtOwner::Executor, true, None, |_| {}).await;
    // Stop the artificial feeder and immediately silence J4 while its K=3 window is fresh (< 40 ms).
    rig2.feeder_stop.store(true, Ordering::Release);
    tokio::time::sleep(Duration::from_millis(12)).await;
    rig2.bus.with(|s| {
        s.motors[3].silent = true;
    });
    let mut ws2 = rig2.connect_ws().await;
    let (gen2, epoch2) = ws_acquire_lease(&mut ws2, "c-silent-fresh-cache").await;
    rig2.bus.clear_log();
    let err2 = ws_cmd(
        &mut ws2,
        1,
        gen2,
        epoch2,
        mit::TIMEOUT_PATH,
        json!({ "joints": [1], "timeout_counts": 2000, "dry_run": false }),
    )
    .await;
    assert_eq!(
        err2["type"], "command_reject",
        "newly silent J4 with still-fresh cached K=3 window MUST be rejected: {err2}"
    );
    let log2 = rig2.bus.tx_log();
    assert_no_flash_write(&log2);
    assert!(
        !log2
            .iter()
            .any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE),
        "newly silent axis must emit 0 REG_WRITE frames"
    );
    assert_eq!(rig2.bus.with(|s| s.motors[0].reg_u32(damiao::RID_TIMEOUT)), 0);
    assert!(!pending_commit_marker_path(&rig2.calib_path).exists());
    assert_eq!(rig2.hold_state().await, None);
    rig2.assert_zero_alloc("newly silent axis cached-window regression");
    rig2.shutdown().await;
}
