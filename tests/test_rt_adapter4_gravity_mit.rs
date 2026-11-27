//! Adapter 4: Hardware gravity compensation (drag teach) AND MIT tracking runtime on the RT
//! executor (`OWNER_EXECUTOR`) with dual-owner parity (`OWNER_LEGACY` vs `OWNER_EXECUTOR`).
//!
//! Uses `FakeBus` with Damiao `RID_TIMEOUT = 2000` (100 ms watchdog on J1–J6, 0 on J7),
//! `poll_feeds_watchdog = false`, SG2002-like `tx_step`, and `TimedAlloc` as global allocator
//! so every executor test proves:
//! - zero heap allocations/deallocations (`total_allocs() == AllocCounts::ZERO`) on `armd-control`,
//! - zero `SupervisorLock` attempts from `armd-control` (`control_thread_attempts() == 0`),
//! - max committed control-frame gap `<= 40 ms` across all arm joints J1–J6,
//! - zero Damiao `0xD` watchdog expiries,
//! - zero wrong-mode control frames,
//! - zero `CMD_DISABLE` frames during mode switches, E-stop, or SAFE-HOLD,
//! - at most 1 J7 `0x307` frame per release,
//! - identical WS/REST replies, auth checks, error codes, and Rule A′ rest-pose enforcement
//!   between `OWNER_LEGACY` and `OWNER_EXECUTOR`.

mod mit_support;

#[global_allocator]
static ALLOC: armd::diagnostics::alloc::TimedAlloc = armd::diagnostics::alloc::TimedAlloc;

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{mpsc, Arc, Condvar, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use armd::auth::{OperatorAuth, OperatorGrant, OperatorRequirement};
use armd::calibration::{CalibrationDeps, CalibrationFile};
use armd::diagnostics::alloc::AllocCounts;
use armd::driver::{
    gripper_drive_from_profile, ArmDriver, ArmDriverError, DamiaoArmDriver, DisableKind,
    DriverCapabilities, EnableState, JointMode, JointTarget, TxBudget,
};
use armd::grasp::GraspDeps;
use armd::mit::{self, MitJointQualification, MitQualificationFile};
use armd::model::{ArmConfig, ControlMode, JointState, JointTrackingConfig, KeepoutBox, TrackingStrategy};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::broker::{ClientId, DeliveryDecision, Reply};
use armd::rt_core::executor::{
    ExecutorClock, ExecutorConfig, ExecutorStats, PanicPolicy, PersistTestHooks, RtExecutorHandle,
    ESTOP_ACK_CAPACITY, MAINT_QUEUE_CAPACITY, RECLAIM_STATE_CAPACITY, RECLAIM_TRAJ_CAPACITY,
};
use armd::rt_core::requests::config::{
    capture_config_admission, gravity_comp_command_with_auth, run_executor_mit_hand_back,
};
use armd::rt_core::urgent::URGENT_STOP_CAPACITY;
use armd::safety::{FaultCode, FaultKind, SafetyState};
use armd::supervisor::{ArmSupervisor, PendingZeroCheckPersist, ZeroCheckPersistContext};
use armd::transport::{CanFrame, CanRxTime, CanTransport, DriverClock, MockClock, RxClockSample, TransportError};
use futures_util::{SinkExt, StreamExt};
use mit_support::{FakeBus, SW_VER};
use serde_json::{json, Value};
use tokio::net::TcpListener;
use tokio_tungstenite::tungstenite::Message;

const TIMEOUT_2000_COUNTS: u32 = 2000; // 100 ms at 50 us/count
const PERIOD_50HZ: Duration = Duration::from_millis(20);
const RAISED_POSE: [f32; 7] = [0.20, 0.35, -0.35, 0.15, -0.15, 0.10, -0.14];
/// Raised pose strictly inside B601-DM workspace (`INSPECT` with J7 open) for user-exit tests.
const INSIDE_RAISED_POSE: [f32; 7] = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0, -0.14];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum OwnerKind {
    Legacy,
    Executor,
}

fn b601_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm()
        .damiao_ranges
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect()
}

fn rest_pose_of(cfg: &ArmConfig) -> [f32; 7] {
    let mut out = [0.0f32; 7];
    if let Some(ref rp) = cfg.resting_pose {
        if let Some(first) = rp.positions.as_ref() {
            for (i, q) in first.iter().enumerate().take(7) {
                out[i] = *q;
            }
        }
    }
    out[6] = -0.14;
    out
}

fn mit_config_j4() -> ArmConfig {
    let mut jt = JointTrackingConfig::rebot_dm();
    jt.enabled = true;
    for j in &mut jt.joints {
        j.strategy = TrackingStrategy::Position;
    }
    let j4 = &mut jt.joints[3];
    j4.strategy = TrackingStrategy::Mit;
    j4.kp = 10.0;
    j4.kd = 0.3;
    j4.hold_kp = Some(12.0);
    j4.hold_kd = Some(0.4);
    j4.effort_limit_nm = 3.0;
    j4.gravity_scale = 1.0;
    let mut cfg = ArmConfig::rebot_dm();
    cfg.lease_timeout_ms = 600_000;
    cfg.workspace = None;
    cfg.joint_tracking = Some(jt);
    cfg
}

fn valid_mit_qualification_j4(cfg: &ArmConfig) -> MitQualificationFile {
    let limits = b601_limits();
    let l = limits[3];
    MitQualificationFile {
        version: mit::QUALIFICATION_VERSION,
        joints: vec![MitJointQualification {
            profile_id: "rebot_b601dm".into(),
            joint: 4,
            joint_name: cfg.joint_names[3].clone(),
            zero_fingerprint: mit::zero_fingerprint(0.0),
            motor_id: 4,
            reply_id: 0x14,
            esc_id: 4,
            mst_id: 0x14,
            sw_ver: SW_VER,
            pmax: l.p_max,
            vmax: l.v_max,
            tmax: l.t_max,
            timeout_raw: TIMEOUT_2000_COUNTS,
            timeout_unit_us: damiao::TIMEOUT_UNIT_US_DOCUMENTED,
            timeout_unit_verified: true,
            tested_response: "disabled".into(),
            measured_expiry_ms: 101.0,
            worst_latency_ms: 8.0,
            load_envelope: "unloaded, supported".into(),
            max_kp: 15.0,
            max_kd: 0.5,
            effort_ceiling_nm: 4.0,
            date: "2026-11-26T00:00:00Z".into(),
            trace_id: "adapter4-test".into(),
        }],
        invalidated: None,
    }
}

fn configure_damiao_bus(pose: [f32; 7], tx_step: Duration, arm_timeout_counts: u32) -> FakeBus {
    let limits = b601_limits();
    let bus = FakeBus::new(&limits);
    bus.with(|s| {
        s.tx_step = tx_step;
        s.poll_feeds_watchdog = false;
        // Real 1 Mbps CAN register/feedback replies arrive in <1 ms (within the next 50 Hz RX drain),
        // rather than stalling for 2 full 20 ms ticks (40-60 ms > 50 ms REGISTER_REPLY_TIMEOUT).
        s.immediate_replies = true;
        s.follow_commands = false;
        s.tx_log.reserve(16_384);
        s.tx_times.reserve(16_384);
        for (i, m) in s.motors.iter_mut().enumerate() {
            m.q = pose[i];
            m.v = 0.0;
            m.tau = 0.0;
            m.enabled = false;
            m.set_reg_u32(
                damiao::RID_TIMEOUT,
                if i < 6 { arm_timeout_counts } else { 0 },
            );
            m.set_reg_u32(
                damiao::RID_CTRL_MODE,
                if i == 6 {
                    damiao::CTRL_MODE_POS_FORCE
                } else {
                    damiao::CTRL_MODE_POS_VEL
                },
            );
        }
        s.push_all_feedback();
    });
    bus
}

fn build_damiao_driver_with_hook(
    bus: FakeBus,
    cfg: &ArmConfig,
    hook: Option<Arc<dyn Fn(&CanFrame) + Send + Sync>>,
) -> DamiaoArmDriver<FakeBus> {
    let limits = b601_limits();
    let mut drv = DamiaoArmDriver::new(bus, vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap()
        .with_gripper_drive(gripper_drive_from_profile(cfg).unwrap())
        .unwrap();
    if let Some(h) = hook {
        drv = drv.with_before_final_tx_gate_hook(h);
    }
    let mut b = TxBudget::default();
    b.max_frames = 16;
    drv.set_tx_budget(b);
    drv.set_rt_mode(true);
    let _ = drv.check_mapping_ranges();
    let _ = drv.read_joint_states();
    drv
}

fn build_damiao_driver(bus: FakeBus, cfg: &ArmConfig) -> DamiaoArmDriver<FakeBus> {
    build_damiao_driver_with_hook(bus, cfg, None)
}

fn build_standalone_mit_recovery_driver<C: CanTransport>(
    can: C,
    rest: &[f32; 7],
) -> DamiaoArmDriver<C> {
    let limits = b601_limits();
    let mut drv = DamiaoArmDriver::new(can, vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap();
    let mut tb = TxBudget::default();
    tb.max_frames = 16;
    drv.set_tx_budget(tb);
    let mut rest_arr = [0.0f32; armd::rt_core::mailbox::MAX_DOF];
    rest_arr[..7].copy_from_slice(rest);
    let mut arm_j = [0usize; armd::rt_core::mailbox::MAX_DOF];
    for (i, slot) in arm_j.iter_mut().take(6).enumerate() {
        *slot = i;
    }
    drv.set_stopped_disable_gate(armd::driver::StoppedDisableGate {
        safety_stopped: Arc::new(AtomicBool::new(false)),
        has_resting_pose: true,
        arm_joints: arm_j,
        arm_joints_len: 6,
        rest_positions: rest_arr,
        rest_pose_tolerance_rad: 0.25,
    });
    drv.set_rt_mode(true);
    drv
}

fn pause_hub(sup: &Arc<ArmSupervisor>) -> (mpsc::SyncSender<()>, thread::JoinHandle<()>) {
    let (held_tx, held_rx) = mpsc::sync_channel::<()>(1);
    let (rel_tx, rel_rx) = mpsc::sync_channel::<()>(1);
    let hub = sup.hub().clone();
    let jh = thread::spawn(move || {
        let _guard = hub.reader_guard();
        let _ = held_tx.send(());
        let _ = rel_rx.recv();
    });
    held_rx
        .recv_timeout(Duration::from_secs(2))
        .expect("acquire hub reader_guard");
    (rel_tx, jh)
}

struct SyncMitSimLikeDriver {
    pose: [f32; 7],
    enabled: bool,
    modes: Vec<JointMode>,
    mode_history: Arc<Mutex<Vec<Vec<JointMode>>>>,
    fail_next_switch: Arc<AtomicBool>,
}

impl ArmDriver for SyncMitSimLikeDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            velocity_feedback: true,
            torque_feedback: true,
            temperature_feedback: true,
            torque_feedforward: true,
            enable_feedback: true,
            load_feedback: false,
        }
    }
    fn supports_rt_executor(&self) -> Result<(), &'static str> {
        Ok(())
    }
    fn supports_mit_mode_switch(&self) -> bool {
        true
    }
    fn supports_nonblocking_mode_switch(&self) -> bool {
        false
    }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let mut out = Vec::with_capacity(7);
        self.read_joint_states_into(&mut out)?;
        Ok(out)
    }
    fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
        out.clear();
        for i in 0..7 {
            out.push(JointState {
                position: self.pose[i],
                velocity: 0.0,
                torque: 0.0,
                temperature: 30.0,
            });
        }
        Ok(())
    }
    fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        Ok(())
    }
    fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
        self.enabled = enable;
        Ok(())
    }
    fn disable_torque(&mut self, _kind: DisableKind) -> Result<(), ArmDriverError> {
        self.enabled = false;
        Ok(())
    }
    fn enable_state(&self) -> EnableState {
        if self.enabled {
            EnableState::Enabled
        } else {
            EnableState::Disabled
        }
    }
    fn is_enabled(&self) -> bool {
        self.enabled
    }
    fn is_feedback_valid(&self) -> bool {
        true
    }
    fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
        dof == 7 && max_age >= Duration::from_millis(2)
    }
    fn feedback_joints_within(&self, joints: &[usize], max_age: Duration) -> bool {
        max_age >= Duration::from_millis(2) && joints.iter().all(|&j| j < 7)
    }
    fn fresh_joint_positions_into(&self, joints: &[usize], max_age: Duration, out: &mut [f32]) -> bool {
        if max_age < Duration::from_millis(2) || out.len() < joints.len() {
            return false;
        }
        for (slot, &j) in out.iter_mut().zip(joints.iter()) {
            let Some(&p) = self.pose.get(j) else { return false };
            *slot = p;
        }
        true
    }
    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        vec![Some(Duration::from_millis(2)); 7]
    }
    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
        out.clear();
        for _ in 0..7 {
            out.push(Some(Duration::from_millis(2)));
        }
    }
    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
        out.clear();
        for _ in 0..7 {
            out.push(Some(2));
        }
    }
    fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
        out.clear();
        for i in 0..7 {
            out.push(JointState {
                position: self.pose[i],
                velocity: 0.0,
                torque: 0.0,
                temperature: 30.0,
            });
        }
        Some(())
    }
    fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        out.clear();
    }
    fn feedback_seqs_into(&self, out: &mut Vec<u64>) {
        out.clear();
    }
    fn feedback_queue_bounds_into(&self, out: &mut Vec<Option<Instant>>) {
        out.clear();
    }
    fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        out.clear();
    }
    fn take_recovery_writes_into(&mut self, out: &mut Vec<usize>) {
        out.clear();
    }
    fn joint_loads_into(&self, out: &mut Vec<f32>) -> bool {
        out.clear();
        false
    }
    fn motor_status_into(&self, out: &mut Vec<u8>) -> bool {
        out.clear();
        let code = u8::from(self.enabled);
        for _ in 0..7 {
            out.push(code);
        }
        true
    }
    fn clear_fault_disables_arm_joint(&self, _arm_dof: usize) -> bool {
        false
    }
    fn has_hardware_fault(&self) -> bool {
        false
    }
    fn set_joint_modes(
        &mut self,
        modes: &[JointMode],
        _hold: &[JointTarget],
    ) -> Result<(), ArmDriverError> {
        if self.fail_next_switch.swap(false, Ordering::SeqCst) {
            return Err(ArmDriverError::HardwareFault(
                "injected sync mode switch failure".into(),
            ));
        }
        self.modes = modes.to_vec();
        self.mode_history.lock().unwrap().push(modes.to_vec());
        Ok(())
    }
}

struct LiveRig {
    kind: OwnerKind,
    sup: Arc<ArmSupervisor>,
    bus: FakeBus,
    tx_wall_times: Arc<std::sync::Mutex<Vec<(Instant, u64)>>>,
    exec: Option<RtExecutorHandle>,
    legacy_stop: Option<Arc<AtomicBool>>,
    legacy_thread: Option<std::thread::JoinHandle<()>>,
    base_url: String,
    ws_url: String,
    server_jh: tokio::task::JoinHandle<()>,
    _dir: tempfile::TempDir,
}

impl LiveRig {
    async fn start(
        kind: OwnerKind,
        cfg: ArmConfig,
        pose: [f32; 7],
        auth: OperatorAuth,
        mit_qual: Option<MitQualificationFile>,
        pre_arm: bool,
    ) -> Self {
        Self::start_with_hook(kind, cfg, pose, auth, mit_qual, pre_arm, None).await
    }

    async fn start_with_hook(
        kind: OwnerKind,
        mut cfg: ArmConfig,
        pose: [f32; 7],
        auth: OperatorAuth,
        mit_qual: Option<MitQualificationFile>,
        pre_arm: bool,
        before_final_tx_gate_hook: Option<Arc<dyn Fn(&CanFrame) + Send + Sync>>,
    ) -> Self {
        let dir = tempfile::tempdir().unwrap();
        cfg.control_frequency_hz = 50;
        if cfg.lease_timeout_ms >= 2500 {
            cfg.lease_timeout_ms = 600_000;
        }
        if let Some(gc) = cfg.gravity_comp.as_mut() {
            // Shorten ramp/exit hold slightly so multi-owner tests stay deterministic and snappy
            // while still exercising multi-tick RampIn -> Active -> Exiting -> ModeSwitch.
            gc.ramp_in_s = 0.06;
            gc.exit_hold_ms = 60;
        }
        let bus = configure_damiao_bus(pose, Duration::from_micros(300), TIMEOUT_2000_COUNTS);
        let tx_wall_times = Arc::new(std::sync::Mutex::new(Vec::with_capacity(65_536)));
        let stats_slot: Arc<std::sync::OnceLock<Arc<ExecutorStats>>> =
            Arc::new(std::sync::OnceLock::new());
        {
            let times_cb = Arc::clone(&tx_wall_times);
            let stats_cb = Arc::clone(&stats_slot);
            bus.with(|s| {
                s.on_tx = Some(Arc::new(move || {
                    let rel = stats_cb
                        .get()
                        .map_or(0, |st| st.releases_started.load(Ordering::Relaxed));
                    times_cb.lock().unwrap().push((Instant::now(), rel));
                }));
            });
        }
        let drv = build_damiao_driver_with_hook(bus.clone(), &cfg, before_final_tx_gate_hook);
        let sup = Arc::new(ArmSupervisor::new(cfg.clone()).with_driver(Box::new(drv)));
        {
            let mut inner = sup.inner().write().await;
            inner.operator_auth_enforced = auth.is_enforced();
            if let Some(qual) = mit_qual {
                inner.set_calibration(Some(CalibrationFile {
                    mit_qualification: serde_json::to_value(&qual).ok(),
                    ..Default::default()
                }));
            }
        }
        if pre_arm {
            bus.with(|s| {
                s.immediate_replies = true;
                s.push_all_feedback();
            });
            sup.enable_torque(sup.capture_admission_stamp())
                .await
                .expect("pre_arm enable_torque");
            bus.with(|s| {
                s.push_all_feedback();
            });
        }

        let (exec, legacy_stop, legacy_thread) = match kind {
            OwnerKind::Executor => {
                let mut ecfg = ExecutorConfig::for_tests(PERIOD_50HZ);
                ecfg.alloc_timing = true;
                ecfg.panic_policy = PanicPolicy::StopExecutor;
                let handle = sup.start_executor(ecfg).await.expect("start_executor");
                let _ = stats_slot.set(Arc::clone(handle.stats()));
                let dl = Instant::now() + Duration::from_secs(3);
                while sup.hub().latest().is_none() {
                    assert!(Instant::now() < dl, "timed out waiting for StateHub snapshot");
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
                (Some(handle), None, None)
            }
            OwnerKind::Legacy => {
                let stop = Arc::new(AtomicBool::new(false));
                let stop_t = Arc::clone(&stop);
                let sup_t = Arc::clone(&sup);
                let stats = Arc::new(ExecutorStats::default());
                let _ = stats_slot.set(Arc::clone(&stats));
                let rt = tokio::runtime::Handle::current();
                let jh = std::thread::Builder::new()
                    .name("legacy-adapter4-test".into())
                    .spawn(move || {
                        sup_t.run_legacy_control_loop_blocking(
                            PERIOD_50HZ,
                            stop_t,
                            stats,
                            rt,
                            None,
                        );
                    })
                    .unwrap();
                let dl = Instant::now() + Duration::from_secs(3);
                while sup.hub().latest().is_none() {
                    assert!(Instant::now() < dl, "timed out waiting for legacy StateHub snapshot");
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
                (None, Some(stop), Some(jh))
            }
        };

        let det_up = armd::api::proxy::DetectorUpstream::parse("http://127.0.0.1:1").unwrap();
        let setup_path = dir.path().join("grasp_setup.json");
        let calib_path = dir.path().join("calibration.json");
        let grasp_deps = GraspDeps::http(det_up.clone(), Some(setup_path.clone()));
        let cal_deps = CalibrationDeps::new(calib_path, Some(setup_path), det_up.clone());
        let router = armd::api::create_router_calibration(
            sup.clone(),
            det_up,
            grasp_deps,
            armd::diagnostics::DiagnosticsState::global(),
            cal_deps,
            auth,
        );
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        let server_jh = tokio::spawn(async move {
            let _ = axum::serve(listener, router).await;
        });
        let base_url = format!("http://{addr}");
        let ws_url = format!("ws://{addr}/api/v1/ws");

        Self {
            kind,
            sup,
            bus,
            tx_wall_times,
            exec,
            legacy_stop,
            legacy_thread,
            base_url,
            ws_url,
            server_jh,
            _dir: dir,
        }
    }

    fn clear_bus_log(&self) {
        self.bus.with(|s| {
            let mut times = self.tx_wall_times.lock().unwrap();
            s.tx_log.clear();
            s.tx_times.clear();
            times.clear();
        });
    }

    fn snapshot_bus_log(&self) -> (Vec<CanFrame>, Vec<Option<(Instant, u64)>>, usize, bool) {
        self.bus.with(|s| {
            let times = self.tx_wall_times.lock().unwrap();
            s.run_watchdogs();
            let exp: usize = s.motors.iter().map(|m| m.watchdog_expiry_count).sum();
            let en = s.motors.iter().all(|m| m.enabled);
            let tx_meta: Vec<Option<(Instant, u64)>> = times.iter().copied().map(Some).collect();
            (s.tx_log.clone(), tx_meta, exp, en)
        })
    }

    fn assert_rt_clean(&self, context: &str) {
        if let Some(ref h) = self.exec {
            assert_eq!(
                h.stats().total_allocs(),
                AllocCounts::ZERO,
                "{context}: RT thread must have 0 heap allocs/deallocs"
            );
            assert_eq!(
                self.sup.inner().control_thread_attempts(),
                0,
                "{context}: RT thread must have 0 SupervisorLock attempts"
            );
        }
    }

    async fn shutdown(self) {
        self.shutdown_inner(false).await;
    }

    async fn shutdown_emergency(self) {
        self.shutdown_inner(true).await;
    }

    async fn shutdown_inner(mut self, emergency: bool) {
        self.server_jh.abort();
        match self.kind {
            OwnerKind::Executor => {
                if let Some(exec) = self.exec.take() {
                    tokio::task::spawn_blocking(move || {
                        if emergency {
                            let _ = exec.emergency_disable_and_join();
                        } else {
                            exec.stop_and_join();
                        }
                    })
                    .await
                    .unwrap();
                }
            }
            OwnerKind::Legacy => {
                let sh = self.sup.legacy_safe_hold_control();
                sh.request_process_exit_disable(Instant::now() + Duration::from_secs(2));
                if let Some(stop) = self.legacy_stop.take() {
                    stop.store(true, Ordering::Release);
                }
                if let Some(jh) = self.legacy_thread.take() {
                    tokio::task::spawn_blocking(move || {
                        let _ = jh.join();
                    })
                    .await
                    .unwrap();
                }
            }
        }
    }
}

type WsStream =
    tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>>;

async fn ws_connect(url: &str) -> WsStream {
    let (ws, _) = tokio_tungstenite::connect_async(url).await.unwrap();
    ws
}

async fn ws_send_wait_types(ws: &mut WsStream, frame: Value, want_types: &[&str]) -> Value {
    let expected_req_id = frame.get("request_id").cloned();
    ws.send(Message::Text(frame.to_string().into())).await.unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        let rem = deadline.saturating_duration_since(Instant::now());
        assert!(!rem.is_zero(), "timed out waiting for WS types {want_types:?}");
        let msg = tokio::time::timeout(rem, ws.next())
            .await
            .expect("WS recv timeout")
            .expect("WS stream ended")
            .expect("WS frame err");
        if let Message::Text(t) = msg {
            let v: Value = serde_json::from_str(&t).unwrap();
            // Major 6 invariant: P-F1 session terminals (SESSION_REQUEST_ID_BIT) must NEVER
            // be rendered as unsolicited synthetic `obs.action` frames on WebSocket connections.
            if let Some(aid) = v
                .get("action")
                .and_then(|a| a.get("id"))
                .and_then(Value::as_u64)
            {
                assert_eq!(
                    aid & armd::rt_core::reply_router::SESSION_REQUEST_ID_BIT,
                    0,
                    "unsolicited synthetic obs.action frame for session terminal on WS: {v}"
                );
            }
            let ty = v.get("type").and_then(Value::as_str).unwrap_or("");
            if want_types.contains(&ty)
                && (expected_req_id.is_none()
                    || v.get("request_id") == expected_req_id.as_ref()
                    || !matches!(ty, "command_ack" | "command_reject"))
            {
                return v;
            }
        }
    }
}

async fn ws_rpc(
    ws: &mut WsStream,
    req_id: u64,
    gen: u64,
    epoch: u64,
    path: &str,
    body: Value,
) -> Value {
    ws_send_wait_types(
        ws,
        json!({
            "type": "command",
            "request_id": req_id,
            "generation": gen,
            "epoch_id": epoch,
            "path": path,
            "body": body,
        }),
        &["command_ack", "command_reject"],
    )
    .await
}

async fn unlock_operator_token(base_url: &str, pin: &str) -> String {
    let resp: Value = reqwest::Client::new()
        .post(format!("{base_url}/api/v1/auth/unlock"))
        .json(&json!({ "pin": pin }))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    resp["token"].as_str().unwrap().to_string()
}

struct WireAudit {
    max_gap: [Duration; 6],
    wrong_mode_frames: usize,
    disable_frames: usize,
    j7_total_frames: usize,
    j7_max_per_release: usize,
}

/// Audits `FakeBus` wire traffic across an energized window using per-frame `(timestamp, release_id)`.
fn audit_bus_log(
    tx_log: &[CanFrame],
    tx_meta: &[Option<(Instant, u64)>],
    initial_modes: [u32; 6],
    window_start: Instant,
    window_end: Instant,
) -> WireAudit {
    assert_eq!(
        tx_log.len(),
        tx_meta.len(),
        "audit_bus_log requires complete per-frame metadata without truncation ({} vs {})",
        tx_log.len(),
        tx_meta.len()
    );
    let mut active_mode = initial_modes;
    let mut last_ctrl = [window_start; 6];
    let mut max_gap = [Duration::ZERO; 6];
    let mut wrong_mode_frames = 0usize;
    let mut disable_frames = 0usize;
    let mut j7_total_frames = 0usize;
    let mut j7_per_release: std::collections::HashMap<u64, usize> = std::collections::HashMap::new();

    for (idx, (frame, meta_opt)) in tx_log.iter().zip(tx_meta.iter()).enumerate() {
        let (t, release_id) = meta_opt.unwrap_or_else(|| {
            panic!("audit_bus_log missing metadata for frame #{idx}: {frame:?}")
        });
        if t < window_start || t > window_end {
            continue;
        }
        if frame.data == damiao::CMD_DISABLE {
            disable_frames += 1;
            continue;
        }
        if frame.id == 0x7FF
            && frame.data[2] == damiao::REG_WRITE
            && frame.data[3] == damiao::RID_CTRL_MODE
        {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            if (1..=6).contains(&id) {
                let val = u32::from_le_bytes([
                    frame.data[4],
                    frame.data[5],
                    frame.data[6],
                    frame.data[7],
                ]);
                active_mode[(id - 1) as usize] = val;
            }
            continue;
        }
        if frame.id == 0x307 {
            j7_total_frames += 1;
            *j7_per_release.entry(release_id).or_insert(0) += 1;
            continue;
        }
        if (0x101..=0x106).contains(&frame.id) {
            let j = (frame.id - 0x101) as usize;
            if active_mode[j] != damiao::CTRL_MODE_POS_VEL {
                wrong_mode_frames += 1;
            }
            max_gap[j] = max_gap[j].max(t.saturating_duration_since(last_ctrl[j]));
            last_ctrl[j] = t;
        } else if (1..=6).contains(&frame.id)
            && frame.data != damiao::CMD_ENABLE
            && frame.data != damiao::CMD_CLEAR_FAULTS
            && frame.data != damiao::CMD_SET_ZERO
        {
            let j = (frame.id - 1) as usize;
            if active_mode[j] != damiao::CTRL_MODE_MIT {
                wrong_mode_frames += 1;
            }
            max_gap[j] = max_gap[j].max(t.saturating_duration_since(last_ctrl[j]));
            last_ctrl[j] = t;
        }
    }
    for j in 0..6 {
        max_gap[j] = max_gap[j].max(window_end.saturating_duration_since(last_ctrl[j]));
    }
    let j7_max_per_release = j7_per_release.values().copied().max().unwrap_or(0);
    WireAudit {
        max_gap,
        wrong_mode_frames,
        disable_frames,
        j7_total_frames,
        j7_max_per_release,
    }
}

/// Helper: induce J4 0xD expiry on a live `LiveRig` and wait until `SafetyState::Fault(_)` latches.
async fn induce_j4_mit_expiry_fault(rig: &LiveRig, owner: OwnerKind) {
    rig.bus.with(|s| {
        s.motors[3].silent = true;
    });
    tokio::time::sleep(Duration::from_millis(90)).await;
    rig.bus.with(|s| {
        s.motors[3].silent = false;
        s.motors[3].enabled = false;
        s.motors[3].state_override = Some(0xD);
        s.push_feedback(3);
    });
    let dl = Instant::now() + Duration::from_secs(3);
    loop {
        let obs = rig.sup.get_observation().await;
        if matches!(obs.safety_state, SafetyState::Fault(_)) {
            break;
        }
        assert!(
            Instant::now() < dl,
            "{owner:?}: J4 0xD expiry did not latch Fault: {:?}",
            obs.safety_state
        );
        tokio::time::sleep(Duration::from_millis(15)).await;
    }
}

// ============================================================================
// 1. Dual-owner REST parity: POST /api/v1/mode/gravity_comp
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_rest_gravity_comp_parity() {
    let mut results = Vec::new();
    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let rig = LiveRig::start(
            owner,
            ArmConfig::rebot_dm(),
            INSIDE_RAISED_POSE,
            OperatorAuth::enforced("123456", 1800),
            None,
            false,
        )
        .await;
        let client = reqwest::Client::new();
        let url = format!("{}/api/v1/mode/gravity_comp", rig.base_url);

        // 1. Operator locked -> 403 OPERATOR_REQUIRED
        let r_unauth = client.post(&url).json(&json!({"enable": true})).send().await.unwrap();
        let s_unauth = r_unauth.status().as_u16();
        let b_unauth: Value = r_unauth.json().await.unwrap();
        assert_eq!(s_unauth, 403, "{owner:?}: {b_unauth}");
        assert_eq!(b_unauth["code"], "OPERATOR_REQUIRED", "{owner:?}: {b_unauth}");

        // 2. Unlocked operator, no active lease -> 409 LEASE_REQUIRED
        let tok = unlock_operator_token(&rig.base_url, "123456").await;
        let r_nolease = client
            .post(&url)
            .bearer_auth(&tok)
            .json(&json!({"enable": true}))
            .send()
            .await
            .unwrap();
        let s_nolease = r_nolease.status().as_u16();
        let b_nolease: Value = r_nolease.json().await.unwrap();
        assert_eq!(s_nolease, 409, "{owner:?}: {b_nolease}");
        assert_eq!(b_nolease["code"], "LEASE_REQUIRED", "{owner:?}: {b_nolease}");

        // 3. Unlocked operator + active WS lease -> 409 LEASE_ACTIVE
        let mut ws = ws_connect(&rig.ws_url).await;
        let _granted = ws_send_wait_types(
            &mut ws,
            json!({"type": "acquire_lease", "client_id": "rest-parity", "seq": 1}),
            &["lease_granted"],
        )
        .await;
        let r_active = client
            .post(&url)
            .bearer_auth(&tok)
            .json(&json!({"enable": true}))
            .send()
            .await
            .unwrap();
        let s_active = r_active.status().as_u16();
        let b_active: Value = r_active.json().await.unwrap();
        assert_eq!(s_active, 409, "{owner:?}: {b_active}");
        assert_eq!(b_active["code"], "LEASE_ACTIVE", "{owner:?}: {b_active}");

        rig.assert_rt_clean(&format!("rest parity {owner:?}"));
        results.push((
            s_unauth,
            b_unauth["code"].clone(),
            s_nolease,
            b_nolease["code"].clone(),
            s_active,
            b_active["code"].clone(),
        ));
        rig.shutdown().await;
    }
    assert_eq!(results[0], results[1], "REST gravity_comp parity mismatch between Legacy and Executor");
}

// ============================================================================
// 2. Dual-owner WS hardware gravity comp lifecycle, auth, & TIMEOUT<=40ms parity
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_ws_gravity_comp_hardware_lifecycle_and_auth_parity() {
    let mut summaries = Vec::new();
    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let rig = LiveRig::start(
            owner,
            ArmConfig::rebot_dm(),
            INSIDE_RAISED_POSE,
            OperatorAuth::enforced("123456", 1800),
            None,
            false,
        )
        .await;

        let mut ws = ws_connect(&rig.ws_url).await;
        let granted = ws_send_wait_types(
            &mut ws,
            json!({"type": "acquire_lease", "client_id": "gc-ws-parity", "seq": 1}),
            &["lease_granted"],
        )
        .await;
        let gen = granted["generation"].as_u64().unwrap();
        let epoch = rig.sup.get_observation().await.epoch_id;

        // 1. Operator locked -> 403 OPERATOR_REQUIRED
        let rej_op = ws_rpc(&mut ws, 1, gen, epoch, "/api/v1/mode/gravity_comp", json!({"enable": true})).await;
        assert_eq!(rej_op["type"], "command_reject", "{owner:?}: {rej_op}");
        assert_eq!(rej_op["status"], 403, "{owner:?}: {rej_op}");
        assert_eq!(rej_op["code"], "OPERATOR_REQUIRED", "{owner:?}: {rej_op}");

        // Unlock operator on same WS connection
        let tok = unlock_operator_token(&rig.base_url, "123456").await;
        let auth_ack = ws_send_wait_types(
            &mut ws,
            json!({"type": "auth", "seq": 2, "token": tok}),
            &["auth_ack"],
        )
        .await;
        assert_eq!(auth_ack["role"], "operator", "{owner:?}: {auth_ack}");

        // 2. Disarmed -> 412 NOT_ARMED (even with malformed damping_kd, state blocker takes
        //    precedence identically across Legacy and Executor: R1 Major 9).
        let rej_disarmed = ws_rpc(
            &mut ws,
            2,
            gen,
            epoch,
            "/api/v1/mode/gravity_comp",
            json!({"enable": true, "damping_kd": [0.0]}),
        )
        .await;
        assert_eq!(rej_disarmed["type"], "command_reject", "{owner:?}: {rej_disarmed}");
        assert_eq!(rej_disarmed["status"], 412, "{owner:?}: {rej_disarmed}");
        assert_eq!(rej_disarmed["code"], "NOT_ARMED", "{owner:?}: {rej_disarmed}");

        // 3. Arm via WS /api/v1/torque
        let arm_ack = ws_rpc(&mut ws, 3, gen, epoch, "/api/v1/torque", json!({"enable": true})).await;
        assert_eq!(arm_ack["type"], "command_ack", "{owner:?}: {arm_ack}");
        assert_eq!(arm_ack["is_torqued"], true, "{owner:?}: {arm_ack}");

        // Wait 2 ticks to establish fresh energized baseline, then start continuous wire audit window
        tokio::time::sleep(Duration::from_millis(50)).await;
        let audit_start = Instant::now();
        rig.clear_bus_log();

        // 4. Invalid scale -> 400 INVALID_COMMAND
        let rej_scale = ws_rpc(
            &mut ws,
            4,
            gen,
            epoch,
            "/api/v1/mode/gravity_comp",
            json!({"enable": true, "scale": 99.0}),
        )
        .await;
        assert_eq!(rej_scale["type"], "command_reject", "{owner:?}: {rej_scale}");
        assert_eq!(rej_scale["status"], 400, "{owner:?}: {rej_scale}");
        assert_eq!(rej_scale["code"], "INVALID_COMMAND", "{owner:?}: {rej_scale}");

        // 5. Wrong epoch -> 409 EPOCH_MISMATCH
        let rej_epoch = ws_rpc(
            &mut ws,
            5,
            gen,
            epoch + 99,
            "/api/v1/mode/gravity_comp",
            json!({"enable": true}),
        )
        .await;
        assert_eq!(rej_epoch["type"], "command_reject", "{owner:?}: {rej_epoch}");
        assert_eq!(rej_epoch["status"], 409, "{owner:?}: {rej_epoch}");
        assert_eq!(rej_epoch["code"], "EPOCH_MISMATCH", "{owner:?}: {rej_epoch}");

        // 6. Hardware gravity_comp enter (POS_VEL -> MIT via ModeSwitch + RampIn -> Active)
        let ack_enter = ws_rpc(
            &mut ws,
            6,
            gen,
            epoch,
            "/api/v1/mode/gravity_comp",
            json!({"enable": true}),
        )
        .await;
        assert_eq!(ack_enter["type"], "command_ack", "{owner:?}: {ack_enter}");
        assert_eq!(ack_enter["control_mode"], "gravity_comp", "{owner:?}: {ack_enter}");
        assert_eq!(ack_enter["is_torqued"], true, "{owner:?}: {ack_enter}");

        // Verify all J1..J6 switched to MIT on wire
        rig.bus.with(|s| {
            for j in 0..6 {
                assert_eq!(
                    s.motors[j].ctrl_mode(),
                    damiao::CTRL_MODE_MIT,
                    "{owner:?}: J{} must be in MIT mode after gravity_comp enter",
                    j + 1
                );
            }
        });

        // Wait through RampIn -> Active
        let dl = Instant::now() + Duration::from_secs(2);
        loop {
            let obs = rig.sup.get_observation().await;
            if obs.gravity_comp.phase.as_deref() == Some("active") {
                break;
            }
            assert!(Instant::now() < dl, "{owner:?}: gravity_comp never reached Active: {:?}", obs.gravity_comp);
            tokio::time::sleep(Duration::from_millis(15)).await;
        }

        // 7. Idempotent re-enter (even with invalid damping_kd, idempotence takes precedence: R1 Major 9)
        let ack_reenter = ws_rpc(
            &mut ws,
            7,
            gen,
            epoch,
            "/api/v1/mode/gravity_comp",
            json!({"enable": true, "damping_kd": [0.0]}),
        )
        .await;
        assert_eq!(ack_reenter["type"], "command_ack", "{owner:?}: {ack_reenter}");
        assert_eq!(ack_reenter["control_mode"], "gravity_comp", "{owner:?}: {ack_reenter}");

        // 8. Hardware gravity_comp exit (stiff hold Exiting -> MIT -> POS_VEL)
        let ack_exit = ws_rpc(
            &mut ws,
            8,
            gen,
            epoch,
            "/api/v1/mode/gravity_comp",
            json!({"enable": false}),
        )
        .await;
        assert_eq!(ack_exit["type"], "command_ack", "{owner:?}: {ack_exit}");
        assert_eq!(ack_exit["is_torqued"], true, "{owner:?}: {ack_exit}");

        // Wait until POS_VEL handback finishes on both owners
        let dl = Instant::now() + Duration::from_secs(3);
        loop {
            let obs = rig.sup.get_observation().await;
            if obs.gravity_comp.phase.is_none() && obs.control_mode == ControlMode::Position {
                break;
            }
            assert!(Instant::now() < dl, "{owner:?}: gravity_comp exit never returned to position: {:?}", obs.gravity_comp);
            tokio::time::sleep(Duration::from_millis(15)).await;
        }

        // Verify all J1..J6 confirmed back in POS_VEL
        rig.bus.with(|s| {
            for j in 0..6 {
                assert_eq!(
                    s.motors[j].ctrl_mode(),
                    damiao::CTRL_MODE_POS_VEL,
                    "{owner:?}: J{} must be back in POS_VEL after gravity_comp exit",
                    j + 1
                );
            }
        });

        // 9. Idempotent re-exit -> command_ack position
        let ack_reexit = ws_rpc(
            &mut ws,
            9,
            gen,
            epoch,
            "/api/v1/mode/gravity_comp",
            json!({"enable": false}),
        )
        .await;
        assert_eq!(ack_reexit["type"], "command_ack", "{owner:?}: {ack_reexit}");
        assert_eq!(ack_reexit["control_mode"], "position", "{owner:?}: {ack_reexit}");

        let audit_end = Instant::now();
        let (tx_log, tx_meta, expiries, all_enabled) = rig.snapshot_bus_log();
        assert_eq!(expiries, 0, "{owner:?}: 0 Damiao TIMEOUT=2000 watchdog expiries allowed");
        assert!(all_enabled, "{owner:?}: all motors must stay enabled");

        let audit = audit_bus_log(
            &tx_log,
            &tx_meta,
            [damiao::CTRL_MODE_POS_VEL; 6],
            audit_start,
            audit_end,
        );
        assert_eq!(audit.wrong_mode_frames, 0, "{owner:?}: 0 wrong-mode frames allowed");
        assert_eq!(audit.disable_frames, 0, "{owner:?}: 0 CMD_DISABLE frames allowed");
        assert!(
            audit.j7_total_frames > 0,
            "{owner:?}: J7 must actually emit 0x307 frames during active window"
        );
        assert!(
            audit.j7_max_per_release <= 1,
            "{owner:?}: J7 must emit <= 1 0x307 frame per release ID (got {})",
            audit.j7_max_per_release
        );
        // Both Legacy and Executor run PR0C ModeSwitch with 50 Hz (20 ms) releases;
        // enforce the strict 40 ms watchdog-continuity bound on every arm motor (0 ms slack).
        let gap_bound = Duration::from_millis(40);
        for j in 0..6 {
            assert!(
                audit.max_gap[j] <= gap_bound,
                "{owner:?}: J{} max gap {:?} exceeded {:?} (all: {:?})",
                j + 1,
                audit.max_gap[j],
                gap_bound,
                audit.max_gap
            );
        }

        rig.assert_rt_clean(&format!("ws gravity_comp lifecycle {owner:?}"));

        summaries.push(json!([
            rej_op["status"],
            rej_op["code"],
            rej_disarmed["status"],
            rej_disarmed["code"],
            rej_scale["status"],
            rej_scale["code"],
            rej_epoch["status"],
            rej_epoch["code"],
            ack_enter["control_mode"],
            ack_reenter["control_mode"],
            ack_reexit["control_mode"],
        ]));
        rig.shutdown().await;
    }

    assert_eq!(
        summaries[0], summaries[1],
        "WS gravity_comp reply/error parity mismatch between Legacy and Executor"
    );
}

// ============================================================================
// 3. Dual-owner safety exit (lease loss) & ambiguous ModeSwitch -> SAFE-HOLD
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_gravity_comp_safety_exits_and_ambiguous_safe_hold_parity() {
    // Part A: Lease loss while floating -> stiff hold Exiting -> POS_VEL handback on both owners.
    let mut lease_exit_codes = Vec::new();
    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let rig = LiveRig::start(
            owner,
            ArmConfig::rebot_dm(),
            RAISED_POSE,
            OperatorAuth::disabled(),
            None,
            true,
        )
        .await;
        let mut ws = ws_connect(&rig.ws_url).await;
        let granted = ws_send_wait_types(
            &mut ws,
            json!({"type": "acquire_lease", "client_id": "gc-lease-exit", "seq": 1}),
            &["lease_granted"],
        )
        .await;
        let gen = granted["generation"].as_u64().unwrap();
        let epoch = rig.sup.get_observation().await.epoch_id;

        let ack = ws_rpc(&mut ws, 1, gen, epoch, "/api/v1/mode/gravity_comp", json!({"enable": true})).await;
        assert_eq!(ack["type"], "command_ack", "{owner:?}: {ack}");

        // Wait for Active
        let dl = Instant::now() + Duration::from_secs(2);
        while rig.sup.get_observation().await.gravity_comp.phase.as_deref() != Some("active") {
            assert!(Instant::now() < dl);
            tokio::time::sleep(Duration::from_millis(15)).await;
        }

        // Release lease while floating -> must exit cleanly back to POS_VEL with 0 expiries
        let _rel = ws_send_wait_types(
            &mut ws,
            json!({"type": "release_lease", "generation": gen, "seq": 2}),
            &["lease_released"],
        )
        .await;

        let dl = Instant::now() + Duration::from_secs(3);
        let obs = loop {
            let o = rig.sup.get_observation().await;
            if o.gravity_comp.phase.is_none() && o.control_mode == ControlMode::Position {
                break o;
            }
            assert!(Instant::now() < dl, "{owner:?}: lease-loss gravity_comp exit stalled: {:?}", o.gravity_comp);
            tokio::time::sleep(Duration::from_millis(15)).await;
        };
        let exit_code = obs.gravity_comp.last_exit.as_ref().map(|e| e.code);
        assert_eq!(exit_code.as_ref().map(|c| c.as_str()), Some("LEASE_LOST"), "{owner:?}: {obs:?}");
        rig.bus.with(|s| {
            s.run_watchdogs();
            let exp: usize = s.motors.iter().map(|m| m.watchdog_expiry_count).sum();
            assert_eq!(exp, 0, "{owner:?}: 0 expiries during lease-loss exit");
            for j in 0..6 {
                assert_eq!(s.motors[j].ctrl_mode(), damiao::CTRL_MODE_POS_VEL);
            }
        });
        rig.assert_rt_clean(&format!("lease-loss exit {owner:?}"));
        lease_exit_codes.push(exit_code);
        rig.shutdown().await;
    }
    assert_eq!(lease_exit_codes[0], lease_exit_codes[1]);

    // Part B: Ambiguous ModeSwitch during hardware gravity-comp enter on BOTH Legacy and Executor
    // Damiao -> SAFE-HOLD (R1 Major 10 convergence), 0 CMD_DISABLE frames, confirmed axes held alive.
    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let rig = LiveRig::start(
            owner,
            ArmConfig::rebot_dm(),
            RAISED_POSE,
            OperatorAuth::disabled(),
            None,
            true,
        )
        .await;
        // Exercise ambiguity through the production WS `/api/v1/mode/gravity_comp` route
        // on BOTH OWNER_LEGACY and OWNER_EXECUTOR (verifies R2 B1 convergence).
        let mut ws = ws_connect(&rig.ws_url).await;
        let granted = ws_send_wait_types(
            &mut ws,
            json!({"type": "acquire_lease", "client_id": "gc-ambig"}),
            &["lease_granted"],
        )
        .await;
        let gen = granted["generation"].as_u64().unwrap();
        let epoch = rig.sup.get_observation().await.epoch_id;

        // Make J2 drop its REG_WRITE ACK -> ModeSwitch terminates Ambiguous & escalates SAFE-HOLD.
        rig.bus.clear_log();
        rig.bus.with(|s| {
            s.motors[1].no_write_ack = true;
        });

        let rej = ws_rpc(
            &mut ws,
            1,
            gen,
            epoch,
            "/api/v1/mode/gravity_comp",
            json!({"enable": true}),
        )
        .await;
        assert_eq!(
            rej["type"], "command_reject",
            "{owner:?}: ambiguous WS enter must return command_reject: {rej}"
        );
        assert_eq!(rej["status"], 409, "{owner:?}: {rej}");

        let disables = rig.bus.with(|s| {
            s.tx_log
                .iter()
                .filter(|f| f.data == damiao::CMD_DISABLE)
                .count()
        });
        assert_eq!(disables, 0, "{owner:?}: SAFE-HOLD on ModeSwitch ambiguity must emit 0 CMD_DISABLE frames");
        assert!(
            rig.sup.internal_fault_lane().is_fenced(),
            "{owner:?}: ambiguous WS ModeSwitch must fence internal_fault_lane (SAFE-HOLD)"
        );
        if let Some(ref h) = rig.exec {
            let dl_sh = Instant::now() + Duration::from_millis(500);
            while !h.is_in_safe_hold() && Instant::now() < dl_sh {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
            assert!(h.is_in_safe_hold(), "{owner:?}: Executor must enter SAFE-HOLD");
        }
        drop(ws);
        rig.assert_rt_clean(&format!("ambiguous WS ModeSwitch SAFE-HOLD {owner:?}"));
        rig.shutdown_emergency().await;
    }
}

// ============================================================================
// 3b. Dual-owner parity: Stop during active gravity comp & E-stop during switching
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_gravity_comp_stop_and_estop_during_switching_parity() {
    let mut stop_outcomes = Vec::new();
    let mut estop_switch_outcomes = Vec::new();
    // Part B: E-stop while gravity_comp hardware ModeSwitch (POS_VEL -> MIT) is mid-flight.
    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let rig = LiveRig::start(
            owner,
            ArmConfig::rebot_dm(),
            RAISED_POSE,
            OperatorAuth::disabled(),
            None,
            true,
        )
        .await;
        let mut ws = ws_connect(&rig.ws_url).await;
        let granted = ws_send_wait_types(
            &mut ws,
            json!({"type": "acquire_lease", "client_id": "gc-estop-sw", "seq": 1}),
            &["lease_granted"],
        )
        .await;
        let gen = granted["generation"].as_u64().unwrap();
        let epoch = rig.sup.get_observation().await.epoch_id;

        // Pause ModeSwitch cleanly mid-flight at the driver write boundary (stall REG_WRITE
        // without leaving an unacknowledged REG_WRITE on the wire) so E-stop lands cleanly
        // mid-switch before completion or ambiguity.
        rig.bus.with(|s| {
            s.immediate_replies = true;
            s.stall_reg_writes = true;
        });
        rig.clear_bus_log();

        ws.send(Message::Text(
            json!({
                "type": "command",
                "request_id": 1,
                "generation": gen,
                "epoch_id": epoch,
                "path": "/api/v1/mode/gravity_comp",
                "body": { "enable": true },
            })
            .to_string()
            .into(),
        ))
        .await
        .unwrap();
        let dl_sw = Instant::now() + Duration::from_secs(2);
        while rig.bus.with(|s| {
            !s.tx_log
                .iter()
                .any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE)
        }) {
            assert!(Instant::now() < dl_sw, "{owner:?}: ModeSwitch did not attempt REG_WRITE");
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        let es_sync = ws_send_wait_types(&mut ws, json!({"type": "e_stop", "seq": 2}), &["epoch_sync"]).await;
        let es_epoch = es_sync["epoch_id"].as_u64().unwrap();
        assert!(es_epoch > epoch, "{owner:?}: e_stop must advance epoch");
        rig.bus.with(|s| {
            s.stall_reg_writes = false;
        });

        let dl_es = Instant::now() + Duration::from_secs(3);
        let obs_es = loop {
            let o = rig.sup.get_observation().await;
            if o.safety_state == SafetyState::EStop && o.epoch_id >= es_epoch {
                break o;
            }
            assert!(Instant::now() < dl_es, "{owner:?}: EStop never published: {o:?}");
            tokio::time::sleep(Duration::from_millis(10)).await;
        };
        rig.bus.with(|s| {
            s.run_watchdogs();
            let exp: usize = s.motors.iter().map(|m| m.watchdog_expiry_count).sum();
            let dis = s.tx_log.iter().filter(|f| f.data == damiao::CMD_DISABLE).count();
            assert_eq!(exp, 0, "{owner:?}: 0 expiries on E-stop during switching");
            assert_eq!(dis, 0, "{owner:?}: 0 CMD_DISABLE frames on E-stop during switching");
        });

        // Clear E-stop and verify clean return to Idle + Position mode on both owners.
        let clr = ws_rpc(&mut ws, 2, gen, obs_es.epoch_id, "/api/v1/safety/clear_fault", json!({})).await;
        assert_eq!(clr["type"], "command_ack", "{owner:?}: clear after E-stop during switching: {clr}");
        let obs_idle = rig.sup.get_observation().await;
        assert_eq!(obs_idle.safety_state, SafetyState::Idle, "{owner:?}");
        assert_eq!(obs_idle.control_mode, ControlMode::Position, "{owner:?}");
        rig.assert_rt_clean(&format!("gravity_comp estop during switching {owner:?}"));
        estop_switch_outcomes.push((obs_es.safety_state, clr["type"].clone(), obs_idle.safety_state));
        rig.shutdown().await;
    }

    // Part A: Stop while floating in active gravity_comp -> stiff hold -> MIT -> POS_VEL handback.
    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let rig = LiveRig::start(
            owner,
            ArmConfig::rebot_dm(),
            RAISED_POSE,
            OperatorAuth::disabled(),
            None,
            true,
        )
        .await;
        let mut ws = ws_connect(&rig.ws_url).await;
        let granted = ws_send_wait_types(
            &mut ws,
            json!({"type": "acquire_lease", "client_id": "gc-stop-parity", "seq": 1}),
            &["lease_granted"],
        )
        .await;
        let gen = granted["generation"].as_u64().unwrap();
        let epoch = rig.sup.get_observation().await.epoch_id;

        let enter_ack = ws_rpc(&mut ws, 1, gen, epoch, "/api/v1/mode/gravity_comp", json!({"enable": true})).await;
        assert_eq!(enter_ack["type"], "command_ack", "{owner:?}: {enter_ack}");
        let dl = Instant::now() + Duration::from_secs(2);
        while rig.sup.get_observation().await.gravity_comp.phase.as_deref() != Some("active") {
            assert!(Instant::now() < dl);
            tokio::time::sleep(Duration::from_millis(15)).await;
        }

        rig.clear_bus_log();
        let stop_ack = ws_rpc(&mut ws, 2, gen, epoch, "/api/v1/motion/stop", json!({})).await;
        assert_eq!(stop_ack["type"], "command_ack", "{owner:?}: stop during gravity_comp: {stop_ack}");

        let dl_pos = Instant::now() + Duration::from_secs(3);
        let obs_after = loop {
            let obs = rig.sup.get_observation().await;
            if obs.gravity_comp.phase.is_none() && obs.control_mode == ControlMode::Position {
                break obs;
            }
            assert!(Instant::now() < dl_pos, "{owner:?}: stop handback stalled: {:?}", obs.gravity_comp);
            tokio::time::sleep(Duration::from_millis(15)).await;
        };
        let exit_code = obs_after.gravity_comp.last_exit.as_ref().map(|e| e.code.as_str().to_string());
        assert_eq!(exit_code.as_deref(), Some("STOPPED"), "{owner:?}: {obs_after:?}");
        rig.bus.with(|s| {
            s.run_watchdogs();
            let exp: usize = s.motors.iter().map(|m| m.watchdog_expiry_count).sum();
            let dis = s.tx_log.iter().filter(|f| f.data == damiao::CMD_DISABLE).count();
            assert_eq!(exp, 0, "{owner:?}: 0 expiries during stop exit");
            assert_eq!(dis, 0, "{owner:?}: 0 CMD_DISABLE frames during stop exit");
            for j in 0..6 {
                assert_eq!(s.motors[j].ctrl_mode(), damiao::CTRL_MODE_POS_VEL);
            }
        });
        rig.assert_rt_clean(&format!("gravity_comp stop parity {owner:?}"));
        stop_outcomes.push((stop_ack["type"].clone(), exit_code, obs_after.control_mode));
        rig.shutdown().await;
    }
    assert_eq!(stop_outcomes[0], stop_outcomes[1], "Legacy vs Executor stop parity mismatch");
    assert_eq!(estop_switch_outcomes[0], estop_switch_outcomes[1], "Legacy vs Executor E-stop during switching parity mismatch");
}

// ============================================================================
// 4. Executor E-stop during gravity comp: 0 ModeWrite while latched + clean clear_fault handback
//    AND same-socket urgent e_stop during in-flight hardware gravity-comp enter
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_gravity_comp_estop_zero_mode_write_and_clear_fault_handback() {
    let rig = LiveRig::start(
        OwnerKind::Executor,
        ArmConfig::rebot_dm(),
        RAISED_POSE,
        OperatorAuth::disabled(),
        None,
        true,
    )
    .await;
    let mut ws = ws_connect(&rig.ws_url).await;
    let granted = ws_send_wait_types(
        &mut ws,
        json!({"type": "acquire_lease", "client_id": "gc-estop", "seq": 1}),
        &["lease_granted"],
    )
    .await;
    let gen = granted["generation"].as_u64().unwrap();
    let epoch = rig.sup.get_observation().await.epoch_id;

    let ack = ws_rpc(&mut ws, 1, gen, epoch, "/api/v1/mode/gravity_comp", json!({"enable": true})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let dl = Instant::now() + Duration::from_secs(2);
    while rig.sup.get_observation().await.gravity_comp.phase.as_deref() != Some("active") {
        assert!(Instant::now() < dl);
        tokio::time::sleep(Duration::from_millis(15)).await;
    }

    // Clear bus log right before urgent E-stop so we audit EVERY frame while E-stop is latched!
    rig.clear_bus_log();
    let _estop_ack = ws_send_wait_types(
        &mut ws,
        json!({"type": "e_stop", "seq": 10}),
        &["epoch_sync"],
    )
    .await;

    // Hold E-stop latched for 180 ms (> 100 ms Damiao TIMEOUT=2000 AND > 60 ms exit_hold_ms).
    tokio::time::sleep(Duration::from_millis(180)).await;

    let (mode_writes_during_estop, disables_during_estop, expiries_during_estop, obs_estop) = {
        let obs = rig.sup.get_observation().await;
        let (mw, dis, exp) = rig.bus.with(|s| {
            s.run_watchdogs();
            let mw = s
                .tx_log
                .iter()
                .filter(|f| {
                    f.id == 0x7FF
                        && f.data[2] == damiao::REG_WRITE
                        && f.data[3] == damiao::RID_CTRL_MODE
                })
                .count();
            let dis = s.tx_log.iter().filter(|f| f.data == damiao::CMD_DISABLE).count();
            let exp: usize = s.motors.iter().map(|m| m.watchdog_expiry_count).sum();
            (mw, dis, exp)
        });
        (mw, dis, exp, obs)
    };
    assert_eq!(obs_estop.safety_state, SafetyState::EStop);
    assert_eq!(
        obs_estop.gravity_comp.phase.as_deref(),
        Some("exiting"),
        "while E-stop is latched, gravity_comp must stay in stiff MIT Exiting (not ExitFailed deadlock): {:?}",
        obs_estop.gravity_comp
    );
    assert_eq!(mode_writes_during_estop, 0, "0 ModeWrite frames permitted while E-stop is latched");
    assert_eq!(disables_during_estop, 0, "0 CMD_DISABLE frames permitted during E-stop hold");
    assert_eq!(expiries_during_estop, 0, "stiff MIT hold must keep TIMEOUT=2000 alive during E-stop");

    // Clear E-stop via WS /api/v1/safety/clear_fault -> must succeed (no deadlock!) and trigger
    // automatic MIT -> POS_VEL ModeSwitch handback!
    let estop_epoch = obs_estop.epoch_id;
    let clear_ack = ws_rpc(
        &mut ws,
        2,
        gen,
        estop_epoch,
        "/api/v1/safety/clear_fault",
        json!({}),
    )
    .await;
    assert_eq!(clear_ack["type"], "command_ack", "clear_fault after gravity_comp E-stop must succeed: {clear_ack}");

    let dl = Instant::now() + Duration::from_secs(3);
    loop {
        let obs = rig.sup.get_observation().await;
        if obs.gravity_comp.phase.is_none() && obs.control_mode == ControlMode::Position {
            break;
        }
        assert!(Instant::now() < dl, "MIT->POS_VEL handback after E-stop clear stalled: {:?}", obs.gravity_comp);
        tokio::time::sleep(Duration::from_millis(15)).await;
    }
    rig.bus.with(|s| {
        s.run_watchdogs();
        let exp: usize = s.motors.iter().map(|m| m.watchdog_expiry_count).sum();
        assert_eq!(exp, 0, "0 expiries across E-stop + clear_fault + handback");
        for j in 0..6 {
            assert_eq!(s.motors[j].ctrl_mode(), damiao::CTRL_MODE_POS_VEL);
        }
    });

    // Same-socket urgent e_stop responsiveness while a multi-tick hardware gravity_comp enter is in flight:
    let new_epoch = rig.sup.get_observation().await.epoch_id;
    // Re-arm first
    let rearm = ws_rpc(&mut ws, 3, gen, new_epoch, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(rearm["type"], "command_ack", "{rearm}");
    // Slow down J3 write ack so gravity_comp enter takes multiple ticks
    rig.bus.with(|s| {
        s.delay_next_reg_write_ack = Some(Duration::from_millis(60));
    });
    // Send enter command WITHOUT waiting for its ack, then immediately send urgent e_stop on the SAME socket!
    ws.send(Message::Text(
        json!({
            "type": "command",
            "request_id": 4,
            "generation": gen,
            "epoch_id": new_epoch,
            "path": "/api/v1/mode/gravity_comp",
            "body": { "enable": true },
        })
        .to_string()
        .into(),
    ))
    .await
    .unwrap();
    let t_estop = Instant::now();
    let estop_fast = ws_send_wait_types(
        &mut ws,
        json!({"type": "e_stop", "seq": 11}),
        &["epoch_sync"],
    )
    .await;
    assert_eq!(estop_fast["type"], "epoch_sync");
    assert!(
        t_estop.elapsed() < Duration::from_millis(500),
        "urgent e_stop on same socket must not be blocked by in-flight gravity_comp enter"
    );

    rig.assert_rt_clean("executor gravity_comp E-stop + same-socket urgent");
    rig.shutdown().await;
}

// ============================================================================
// 5. Executor P-F1 terminal credit delivery & zero RT drop/dealloc across repeated cycles
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_gravity_comp_router_terminal_credit_and_zero_rt_reclaim() {
    let rig = LiveRig::start(
        OwnerKind::Executor,
        ArmConfig::rebot_dm(),
        INSIDE_RAISED_POSE,
        OperatorAuth::disabled(),
        None,
        true,
    )
    .await;
    let exec = rig.exec.as_ref().unwrap().clone();
    let delivered: Arc<std::sync::Mutex<Vec<(ClientId, Reply)>>> = Arc::default();
    let d_cb = Arc::clone(&delivered);
    let sink = Arc::new(move |client: ClientId, reply: Reply| {
        d_cb.lock().unwrap().push((client, reply));
        DeliveryDecision::Delivered
    });
    let caller = ClientId(808);
    exec.open_connection_sync(808).unwrap().unwrap();
    let conn_handle = armd::rt_core::requests::lease::ConnectionHandle::try_new(808, "gc-credit").unwrap();
    let _tok = exec.register_connection_reply_sink(caller, sink, Some(16)).unwrap();

    // Acquire lease directly on ClientId(808) so conn_id matches active_lease_conn_id.
    let lease_gen = match rig
        .sup
        .acquire_lease_ws(conn_handle, "gc-credit", 1)
        .await
        .expect("acquire_lease_ws")
    {
        armd::rt_core::requests::lease::LeaseOutcome::Granted { generation, .. } => generation,
        other => panic!("expected Granted, got {other:?}"),
    };
    let lease = Some((armd::model::HolderLabel::new("gc-credit"), lease_gen));
    let grant = OperatorGrant::unenforced();

    // Run 3 full hardware enter/exit cycles + 2 idempotent inactive exits (verifies zero leaked credits
    // and zero RT frees when GravityCompRun/GravityCompPolicy/tickets are retired via OffloadReclaim).
    for _ in 0..3 {
        let adm = capture_config_admission(&rig.sup, 808, lease_gen, None);
        let enter_rep = gravity_comp_command_with_auth(
            &rig.sup,
            adm,
            lease,
            OperatorRequirement::Required(grant),
            caller,
            true,
            None,
            None,
        )
        .await
        .expect("enter hardware gravity_comp");
        assert!(enter_rep.active);

        let adm_ex = capture_config_admission(&rig.sup, 808, lease_gen, None);
        let _exit_rep = gravity_comp_command_with_auth(
            &rig.sup,
            adm_ex,
            lease,
            OperatorRequirement::NotRequired,
            caller,
            false,
            None,
            None,
        )
        .await
        .expect("exit hardware gravity_comp");
        // Wait for stiff hold Exiting -> MIT -> POS_VEL ModeSwitch to finish
        let dl = Instant::now() + Duration::from_secs(3);
        while rig.sup.get_observation().await.gravity_comp.phase.is_some() {
            assert!(Instant::now() < dl, "timed out waiting for gravity_comp exit handback");
            tokio::time::sleep(Duration::from_millis(10)).await;
        }

        // Inactive idempotent exit must NOT leak a P-F1 terminal credit
        let adm_idle = capture_config_admission(&rig.sup, 808, lease_gen, None);
        let idle_rep = gravity_comp_command_with_auth(
            &rig.sup,
            adm_idle,
            lease,
            OperatorRequirement::NotRequired,
            caller,
            false,
            None,
            None,
        )
        .await
        .expect("idempotent inactive exit");
        assert!(!idle_rep.active);
    }

    // Confirm P-F1 router received exactly 3 session terminals across the 3 client enters
    // (exits are owner-internal so 0 client router credits are retained or leaked on Exit/ExitFailed).
    let term_count = delivered
        .lock()
        .unwrap()
        .iter()
        .filter(|(c, r)| *c == caller && matches!(r, Reply::Terminal { .. }))
        .count();
    assert_eq!(
        term_count, 3,
        "expected exactly 3 P-F1 router terminals across 3 enter/exit cycles, got {term_count}"
    );

    rig.assert_rt_clean("router terminal credit + zero RT reclaim");
    rig.shutdown().await;
}

// ============================================================================
// 6. Dual-owner MIT profile handover, qualification, activation, & handback parity
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_mit_handover_qualification_runtime_and_handback_parity() {
    // 6a. Transfer gate: active MIT (MitOwner::Active) and lost MIT (MitOwner::Lost) MUST be
    // refused by start_executor with 409 TRANSFER_BLOCKED, while inactive (MitOwner::Off) succeeds.
    {
        let cfg = mit_config_j4();
        let bus = configure_damiao_bus(RAISED_POSE, Duration::from_micros(300), TIMEOUT_2000_COUNTS);
        let drv = build_damiao_driver(bus.clone(), &cfg);
        let sup = Arc::new(ArmSupervisor::new(cfg.clone()).with_driver(Box::new(drv)));
        {
            let mut inner = sup.inner().write().await;
            inner.set_calibration(Some(CalibrationFile {
                mit_qualification: serde_json::to_value(valid_mit_qualification_j4(&cfg)).ok(),
                ..Default::default()
            }));
        }
        bus.with(|s| {
            s.immediate_replies = true;
            s.push_all_feedback();
        });
        sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
        for _ in 0..10 {
            sup.tick(0.02).await;
            if sup.inner().read().await.mit_active() {
                break;
            }
        }
        assert!(sup.inner().read().await.mit_active(), "expected MIT Active on Legacy before transfer check");
        // Now MIT is Active on Legacy -> start_executor MUST return 409 TRANSFER_BLOCKED
        match sup
            .start_executor(ExecutorConfig::for_tests(PERIOD_50HZ))
            .await
        {
            Err(armd::supervisor::ArmError::Rejected { status: 409, code, .. })
                if code == "TRANSFER_BLOCKED" => {}
            Err(other) => panic!("active MIT handover must be 409 TRANSFER_BLOCKED, got {other:?}"),
            Ok(_) => panic!("active MIT handover unexpectedly succeeded"),
        }

        // Force MitOwner::Lost via simulated Damiao 0xD expiry -> start_executor MUST also refuse 409 TRANSFER_BLOCKED
        bus.with(|s| {
            s.motors[3].enabled = false;
            s.motors[3].state_override = Some(0xD);
            s.push_feedback(3);
        });
        sup.tick(0.02).await;
        match sup
            .start_executor(ExecutorConfig::for_tests(PERIOD_50HZ))
            .await
        {
            Err(armd::supervisor::ArmError::Rejected { status: 409, code, .. })
                if code == "TRANSFER_BLOCKED" => {}
            Err(other) => panic!("lost MIT handover must be 409 TRANSFER_BLOCKED, got {other:?}"),
            Ok(_) => panic!("lost MIT handover unexpectedly succeeded"),
        }
    }

    // 6b. Dual-owner qualification refusal (missing qualification record -> MIT_UNQUALIFIED)
    let mut unqual_codes = Vec::new();
    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let rig = LiveRig::start(
            owner,
            mit_config_j4(),
            RAISED_POSE,
            OperatorAuth::disabled(),
            None, // no qualification record!
            false,
        )
        .await;
        let mut ws = ws_connect(&rig.ws_url).await;
        let granted = ws_send_wait_types(
            &mut ws,
            json!({"type": "acquire_lease", "client_id": "mit-unqual", "seq": 1}),
            &["lease_granted"],
        )
        .await;
        let gen = granted["generation"].as_u64().unwrap();
        let epoch = rig.sup.get_observation().await.epoch_id;
        let rej = ws_rpc(&mut ws, 1, gen, epoch, "/api/v1/torque", json!({"enable": true})).await;
        assert_eq!(rej["type"], "command_reject", "{owner:?}: unqualified MIT enable must reject: {rej}");
        let msg = rej["message"].as_str().unwrap_or("");
        let code = rej["code"].as_str().unwrap_or("");
        assert!(
            code == "MIT_UNQUALIFIED" || msg.contains("MIT_UNQUALIFIED"),
            "{owner:?}: expected MIT_UNQUALIFIED, got {rej}"
        );
        unqual_codes.push(rej["status"].clone());
        rig.assert_rt_clean(&format!("mit unqualified {owner:?}"));
        rig.shutdown().await;
    }
    assert_eq!(unqual_codes[0], unqual_codes[1]);

    // 6c. Dual-owner qualified MIT enable -> activation -> invalid gravity_comp does NOT hand back
    //     MIT -> valid gravity_comp hands back MIT cleanly (no false mit_supervise trip!) -> exit.
    let mut parity_outcomes = Vec::new();
    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let cfg = mit_config_j4();
        let qual = valid_mit_qualification_j4(&cfg);
        let rig = LiveRig::start(
            owner,
            cfg,
            RAISED_POSE,
            OperatorAuth::disabled(),
            Some(qual),
            false,
        )
        .await;
        let mut ws = ws_connect(&rig.ws_url).await;
        let granted = ws_send_wait_types(
            &mut ws,
            json!({"type": "acquire_lease", "client_id": "mit-runtime", "seq": 1}),
            &["lease_granted"],
        )
        .await;
        let gen = granted["generation"].as_u64().unwrap();
        let epoch = rig.sup.get_observation().await.epoch_id;

        // Enable torque -> runs MIT qualification + ReadIdentityForEnable + arms + activates MIT on J4!
        let arm_ack = ws_rpc(&mut ws, 1, gen, epoch, "/api/v1/torque", json!({"enable": true})).await;
        assert_eq!(arm_ack["type"], "command_ack", "{owner:?}: qualified MIT enable must succeed: {arm_ack}");
        assert_eq!(arm_ack["is_torqued"], true);

        // Wait until J4 activates MIT mode on the wire
        let dl = Instant::now() + Duration::from_secs(3);
        loop {
            let j4_mit = rig.bus.with(|s| s.motors[3].ctrl_mode() == damiao::CTRL_MODE_MIT);
            if j4_mit {
                break;
            }
            assert!(Instant::now() < dl, "{owner:?}: J4 did not activate MIT mode");
            tokio::time::sleep(Duration::from_millis(15)).await;
        }

        // Hold MIT active for 100 ms to confirm healthy tracking & watchdog continuity
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert_eq!(rig.sup.get_observation().await.safety_state, SafetyState::Armed);

        // Invalid gravity_comp request (scale=99.0) MUST be rejected AND MUST NOT hand back MIT!
        let bad_gc = ws_rpc(
            &mut ws,
            2,
            gen,
            epoch,
            "/api/v1/mode/gravity_comp",
            json!({"enable": true, "scale": 99.0}),
        )
        .await;
        assert_eq!(bad_gc["type"], "command_reject", "{owner:?}: {bad_gc}");
        assert_eq!(bad_gc["code"], "INVALID_COMMAND", "{owner:?}: {bad_gc}");
        assert_eq!(
            rig.bus.with(|s| s.motors[3].ctrl_mode()),
            damiao::CTRL_MODE_MIT,
            "{owner:?}: invalid gravity_comp request must NOT hand back active MIT on J4"
        );

        // Valid gravity_comp enter -> hands back MIT on J4 -> enters gravity_comp across J1..J6
        // WITHOUT tripping mit_supervise!
        let ok_gc = ws_rpc(
            &mut ws,
            3,
            gen,
            epoch,
            "/api/v1/mode/gravity_comp",
            json!({"enable": true}),
        )
        .await;
        assert_eq!(ok_gc["type"], "command_ack", "{owner:?}: {ok_gc}");
        assert_eq!(ok_gc["control_mode"], "gravity_comp", "{owner:?}: {ok_gc}");
        assert!(
            matches!(
                rig.sup.get_observation().await.safety_state,
                SafetyState::Armed | SafetyState::Moving
            ),
            "{owner:?}: gravity_comp must be operational (Armed/Moving), got {:?}",
            rig.sup.get_observation().await.safety_state
        );

        // Exit gravity_comp cleanly back to POS_VEL (and J4 stays operational)
        let ex_gc = ws_rpc(
            &mut ws,
            4,
            gen,
            epoch,
            "/api/v1/mode/gravity_comp",
            json!({"enable": false}),
        )
        .await;
        assert_eq!(ex_gc["type"], "command_ack", "{owner:?}: {ex_gc}");

        let dl = Instant::now() + Duration::from_secs(3);
        while rig.sup.get_observation().await.gravity_comp.phase.is_some() {
            assert!(Instant::now() < dl);
            tokio::time::sleep(Duration::from_millis(15)).await;
        }
        assert_eq!(rig.sup.get_observation().await.safety_state, SafetyState::Armed);

        rig.bus.with(|s| {
            s.run_watchdogs();
            let exp: usize = s.motors.iter().map(|m| m.watchdog_expiry_count).sum();
            assert_eq!(exp, 0, "{owner:?}: 0 Damiao TIMEOUT=2000 expiries during MIT + gravity_comp handover");
        });
        rig.assert_rt_clean(&format!("mit runtime + handback {owner:?}"));
        parity_outcomes.push((
            arm_ack["is_torqued"].clone(),
            bad_gc["code"].clone(),
            ok_gc["control_mode"].clone(),
        ));
        rig.shutdown().await;
    }
    assert_eq!(parity_outcomes[0], parity_outcomes[1]);
}

// ============================================================================
// 7. Dual-owner MIT expiry recovery (`MitOwner::Lost`) + Rule A′ rest-pose parity
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_mit_expiry_recovery_and_rule_a_prime_parity() {
    let mut outcomes = Vec::new();
    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let cfg = mit_config_j4();
        let rest = rest_pose_of(&cfg);
        let qual = valid_mit_qualification_j4(&cfg);
        let rig = LiveRig::start(
            owner,
            cfg.clone(),
            RAISED_POSE, // start AWAY from rest pose!
            OperatorAuth::disabled(),
            Some(qual),
            false,
        )
        .await;
        let mut ws = ws_connect(&rig.ws_url).await;
        let granted = ws_send_wait_types(
            &mut ws,
            json!({"type": "acquire_lease", "client_id": "mit-recovery", "seq": 1}),
            &["lease_granted"],
        )
        .await;
        let gen = granted["generation"].as_u64().unwrap();
        let epoch = rig.sup.get_observation().await.epoch_id;

        // Arm & activate MIT on J4
        let arm_ack = ws_rpc(&mut ws, 1, gen, epoch, "/api/v1/torque", json!({"enable": true})).await;
        assert_eq!(arm_ack["type"], "command_ack", "{owner:?}: {arm_ack}");
        let dl = Instant::now() + Duration::from_secs(3);
        while rig.bus.with(|s| s.motors[3].ctrl_mode() != damiao::CTRL_MODE_MIT) {
            assert!(Instant::now() < dl);
            tokio::time::sleep(Duration::from_millis(15)).await;
        }
        // Let the causal post-write ModeSwitch confirmation settle into active MIT tracking
        // before injecting a runtime J4 bus stall.
        tokio::time::sleep(Duration::from_millis(80)).await;

        induce_j4_mit_expiry_fault(&rig, owner).await;
        // Wait past MIT_TRIP_HOLDOFF_MIN (250 ms) so clear_fault is past the MIT quiet period,
        // then feed fresh raised-pose feedback so BOTH owners test the real non-rest pose Rule A′ gate.
        tokio::time::sleep(Duration::from_millis(300)).await;
        for _ in 0..3 {
            rig.bus.with(|s| s.push_all_feedback());
            tokio::time::sleep(Duration::from_millis(25)).await;
        }
        let fault_epoch = rig.sup.get_observation().await.epoch_id;

        // 1. Confirmed clear_fault AWAY from rest pose -> Rule A′ MUST refuse with TORQUE_OFF_WOULD_DROP
        //    and emit 0 CMD_DISABLE frames!
        rig.bus.clear_log();
        let rej_drop = ws_rpc(
            &mut ws,
            2,
            gen,
            fault_epoch,
            "/api/v1/safety/clear_fault",
            json!({"mit_recovery": "torque_off_all"}),
        )
        .await;
        assert_eq!(rej_drop["type"], "command_reject", "{owner:?}: {rej_drop}");
        let drop_code = rej_drop["code"].as_str().unwrap_or("");
        let drop_msg = rej_drop["message"].as_str().unwrap_or("");
        assert!(
            drop_code == "TORQUE_OFF_WOULD_DROP" || drop_msg.contains("resting pose"),
            "{owner:?}: Rule A′ must refuse MIT recovery away from rest pose, got {rej_drop}"
        );
        let disables_away = rig.bus.with(|s| {
            s.tx_log.iter().filter(|f| f.data == damiao::CMD_DISABLE).count()
        });
        assert_eq!(disables_away, 0, "{owner:?}: 0 CMD_DISABLE frames may be sent away from rest pose");

        // 2. Move arm to rest pose & feed fresh samples across >= 4 ticks so K=3 rest-pose evidence
        //    settles without aging past 100 ms; verify UNCONFIRMED clear_fault -> MIT_RECOVERY_CONFIRM_REQUIRED.
        for _ in 0..5 {
            rig.bus.with(|s| {
                for (i, m) in s.motors.iter_mut().enumerate() {
                    m.q = rest[i];
                    m.v = 0.0;
                }
                s.push_all_feedback();
            });
            tokio::time::sleep(Duration::from_millis(25)).await;
        }
        rig.bus.with(|s| s.push_all_feedback());

        let rej_unconfirmed = ws_rpc(
            &mut ws,
            3,
            gen,
            rig.sup.get_observation().await.epoch_id,
            "/api/v1/safety/clear_fault",
            json!({}),
        )
        .await;
        assert_eq!(rej_unconfirmed["type"], "command_reject", "{owner:?}: {rej_unconfirmed}");
        let unconf_code = rej_unconfirmed["code"].as_str().unwrap_or("");
        let unconf_msg = rej_unconfirmed["message"].as_str().unwrap_or("");
        assert!(
            unconf_code == "MIT_RECOVERY_CONFIRM_REQUIRED"
                || unconf_msg.contains("MIT_RECOVERY_CONFIRM_REQUIRED"),
            "{owner:?}: unconfirmed clear_fault in MitOwner::Lost must require confirmation, got {rej_unconfirmed}"
        );

        // 3. Confirmed clear_fault AT rest pose -> clears errors, disables all motors, confirms state 0,
        //    transitions to Idle + MitOwner::Off!
        for _ in 0..3 {
            rig.bus.with(|s| s.push_all_feedback());
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        let ok_recover = ws_rpc(
            &mut ws,
            4,
            gen,
            rig.sup.get_observation().await.epoch_id,
            "/api/v1/safety/clear_fault",
            json!({"mit_recovery": "torque_off_all"}),
        )
        .await;
        assert_eq!(
            ok_recover["type"], "command_ack",
            "{owner:?}: confirmed MIT recovery at rest pose must succeed: {ok_recover}"
        );

        let after_obs = rig.sup.get_observation().await;
        assert_eq!(after_obs.safety_state, SafetyState::Idle, "{owner:?}: must return to Idle");
        assert!(!after_obs.is_torqued, "{owner:?}: torque must be off after MIT recovery");
        rig.bus.with(|s| {
            for (i, m) in s.motors.iter().enumerate() {
                assert!(!m.enabled, "{owner:?}: J{} must be disabled after MIT recovery", i + 1);
                assert_eq!(m.state_override, None, "{owner:?}: J{} fault state must be cleared", i + 1);
            }
        });

        rig.assert_rt_clean(&format!("mit expiry recovery {owner:?}"));
        outcomes.push((
            rej_drop["type"].clone(),
            rej_unconfirmed["type"].clone(),
            ok_recover["type"].clone(),
            after_obs.is_torqued,
        ));
        rig.shutdown().await;
    }
    assert_eq!(outcomes[0], outcomes[1]);
}

// ============================================================================
// 8. Blocker 1 & Major 2: RequestMitHandBack authority enforcement & moving rearm zero-alloc
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_mit_handback_stale_authority_and_moving_rearm_zero_alloc() {
    use armd::supervisor::MitHandBackReason;

    let cfg = mit_config_j4();
    let qual = valid_mit_qualification_j4(&cfg);
    let rig = LiveRig::start(
        OwnerKind::Executor,
        cfg.clone(),
        RAISED_POSE,
        OperatorAuth::enforced("123456", 1800),
        Some(qual),
        false,
    )
    .await;

    // Acquire lease directly on ClientId(818) so we hold the exact connection label & conn_id
    // for testing all 4 stale-authority gates on RequestMitHandBack (Blocker 1).
    let exec = rig.exec.as_ref().unwrap().clone();
    let caller = ClientId(818);
    exec.open_connection_sync(818).unwrap().unwrap();
    let _sink_tok = exec
        .register_connection_reply_sink(
            caller,
            Arc::new(|_, _| DeliveryDecision::Delivered),
            Some(16),
        )
        .unwrap();
    let conn_handle =
        armd::rt_core::requests::lease::ConnectionHandle::try_new(818, "mit-auth-rearm").unwrap();
    let gen = match rig
        .sup
        .acquire_lease_ws(conn_handle, "mit-auth-rearm", 1)
        .await
        .expect("acquire_lease_ws")
    {
        armd::rt_core::requests::lease::LeaseOutcome::Granted { generation, .. } => generation,
        other => panic!("expected Granted, got {other:?}"),
    };
    let epoch = rig.sup.get_observation().await.epoch_id;

    // Arm via torque_ws -> MIT activates on J4.
    let arm_adm = capture_config_admission(&rig.sup, 818, gen, None);
    let arm_out = rig
        .sup
        .torque_ws(
            conn_handle,
            gen,
            Some(epoch),
            true,
            1,
            Ok(true),
            armd::rt_core::requests::lease::OwnerAuthPolicy::None,
            OperatorGrant::unenforced(),
            arm_adm,
        )
        .await
        .expect("torque_ws");
    assert!(
        format!("{arm_out:?}").contains("Accepted"),
        "arm via torque_ws failed: {arm_out:?}"
    );
    let dl = Instant::now() + Duration::from_secs(3);
    loop {
        if rig.bus.with(|s| s.motors[3].ctrl_mode() == damiao::CTRL_MODE_MIT) {
            break;
        }
        assert!(Instant::now() < dl, "MIT never activated on J4");
        tokio::time::sleep(Duration::from_millis(15)).await;
    }
    tokio::time::sleep(Duration::from_millis(60)).await;

    let conn_id = 818u64;
    let lease_holder = Some((armd::model::HolderLabel::new("mit-auth-rearm"), gen));
    let valid_adm = capture_config_admission(&rig.sup, conn_id, gen, None);
    let expired_grant = OperatorGrant::DENIED;

    // (a) Wrong epoch -> rejected BEFORE mutating MIT state (J4 remains MIT).
    let mut bad_epoch_adm = valid_adm;
    bad_epoch_adm.estop_epoch += 99;
    let err_ep = run_executor_mit_hand_back(
        &rig.sup,
        MitHandBackReason::Rearm,
        bad_epoch_adm,
        lease_holder,
        OperatorRequirement::NotRequired,
        ClientId(conn_id),
        Duration::from_secs(1),
    )
    .await
    .expect_err("stale epoch must be rejected");
    assert!(format!("{err_ep:?}").contains("EpochMismatch") || format!("{err_ep}").contains("epoch"), "{err_ep:?}");
    assert_eq!(rig.bus.with(|s| s.motors[3].ctrl_mode()), damiao::CTRL_MODE_MIT);

    // (b) Stale stop_gen -> rejected BEFORE mutating MIT state.
    let mut bad_stop_adm = valid_adm;
    bad_stop_adm.stop_gen += 7;
    let err_stop = run_executor_mit_hand_back(
        &rig.sup,
        MitHandBackReason::Rearm,
        bad_stop_adm,
        lease_holder,
        OperatorRequirement::NotRequired,
        ClientId(conn_id),
        Duration::from_secs(1),
    )
    .await
    .expect_err("stale stop_gen must be rejected");
    assert!(
        format!("{err_stop:?}").contains("STOPPED") || format!("{err_stop:?}").contains("Stopped"),
        "{err_stop:?}"
    );
    assert_eq!(rig.bus.with(|s| s.motors[3].ctrl_mode()), damiao::CTRL_MODE_MIT);

    // (c) Wrong lease generation -> rejected BEFORE mutating MIT state.
    let bad_lease = Some((armd::model::HolderLabel::new("mit-auth-rearm"), gen + 5));
    let err_lease = run_executor_mit_hand_back(
        &rig.sup,
        MitHandBackReason::Rearm,
        valid_adm,
        bad_lease,
        OperatorRequirement::NotRequired,
        ClientId(conn_id),
        Duration::from_secs(1),
    )
    .await
    .expect_err("wrong lease must be rejected");
    assert!(format!("{err_lease:?}").contains("LEASE_LOST"), "{err_lease:?}");
    assert_eq!(rig.bus.with(|s| s.motors[3].ctrl_mode()), damiao::CTRL_MODE_MIT);

    // (c2) Valid lease_holder tuple paired with stale/mismatched admission.lease_gen -> rejected (R2 M1).
    let mut mismatched_adm = valid_adm;
    mismatched_adm.lease_gen = gen + 99;
    let err_mismatch = run_executor_mit_hand_back(
        &rig.sup,
        MitHandBackReason::Rearm,
        mismatched_adm,
        lease_holder,
        OperatorRequirement::NotRequired,
        ClientId(conn_id),
        Duration::from_secs(1),
    )
    .await
    .expect_err("mismatched admission lease_gen must be rejected");
    assert!(format!("{err_mismatch:?}").contains("LEASE_LOST"), "{err_mismatch:?}");
    assert_eq!(rig.bus.with(|s| s.motors[3].ctrl_mode()), damiao::CTRL_MODE_MIT);

    // (d) Expired operator grant when OperatorRequirement::Required -> 403 OPERATOR_REQUIRED, MIT unchanged.
    let err_op = run_executor_mit_hand_back(
        &rig.sup,
        MitHandBackReason::GravityCompEntry,
        valid_adm,
        lease_holder,
        OperatorRequirement::Required(expired_grant),
        ClientId(conn_id),
        Duration::from_secs(1),
    )
    .await
    .expect_err("expired operator grant must be rejected");
    assert!(format!("{err_op:?}").contains("OPERATOR_REQUIRED"), "{err_op:?}");
    assert_eq!(rig.bus.with(|s| s.motors[3].ctrl_mode()), damiao::CTRL_MODE_MIT);

    // Release programmatic lease, acquire on WS, jog while MIT is active, and trigger rearm
    // (`/api/v1/torque` enable:true) while moving -> exercises moving rearm refusal &
    // `replace_active_trajectory(None)` with `AllocCounts::ZERO` on `armd-control` (Major 2)!
    let _ = rig.sup.release_lease_ws(conn_handle, gen).await;
    let mut ws = ws_connect(&rig.ws_url).await;
    let granted = ws_send_wait_types(
        &mut ws,
        json!({"type": "acquire_lease", "client_id": "mit-rearm-ws", "seq": 1}),
        &["lease_granted"],
    )
    .await;
    let ws_gen = granted["generation"].as_u64().unwrap();
    let ws_epoch = rig.sup.get_observation().await.epoch_id;

    let mut target = RAISED_POSE.to_vec();
    target[3] += 0.35;
    let jog_ack = ws_rpc(
        &mut ws,
        1,
        ws_gen,
        ws_epoch,
        "/api/v1/motion/jog",
        json!({"target_positions": target, "duration_secs": 1.5, "speed_scale": 1.0}),
    )
    .await;
    assert_eq!(jog_ack["type"], "command_ack", "jog while MIT active: {jog_ack}");
    tokio::time::sleep(Duration::from_millis(60)).await;

    let rearm_rej = ws_rpc(&mut ws, 2, ws_gen, ws_epoch, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(
        rearm_rej["type"], "command_reject",
        "rearm while MIT moving must be refused: {rearm_rej}"
    );
    let msg = rearm_rej["message"].as_str().unwrap_or("");
    assert!(
        msg.contains("MIT_HAND_BACK_FAILED") || msg.contains("at rest"),
        "expected MIT_HAND_BACK_FAILED while moving, got {rearm_rej}"
    );
    // Wait for automatic brake-to-rest + AuthorizationRevoked handback back to POS_VEL.
    let dl = Instant::now() + Duration::from_secs(3);
    while rig.bus.with(|s| s.motors[3].ctrl_mode()) != damiao::CTRL_MODE_POS_VEL {
        assert!(Instant::now() < dl, "MIT never handed back after moving rearm refusal");
        tokio::time::sleep(Duration::from_millis(15)).await;
    }

    rig.assert_rt_clean("MIT stale authority + moving rearm zero-alloc");
    rig.shutdown().await;
}

// ============================================================================
// 9. Major 1 & Major 3: Mid-tick ModeSwitch / ExitFailed watchdog continuity &
//    tracking rejection fallback zero-alloc on RT executor
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_watchdog_continuity_exit_failed_and_tracking_rejection_zero_alloc() {
    // Part A: Force ExitFailed (non-stationary velocity when exit_hold_ms finishes) ->
    // verify 0 Damiao TIMEOUT expiries and continuous stiff MIT holds (no 40ms gap on the
    // ExitFailed transition tick nor on the ModeSwitch admission tick), then settle to rest
    // and verify clean retry -> POS_VEL handback with 0 RT allocs!
    {
        let mut cfg = ArmConfig::rebot_dm();
        let mut jt = JointTrackingConfig::rebot_dm();
        jt.enabled = true;
        for j in &mut jt.joints {
            j.strategy = TrackingStrategy::Position;
        }
        cfg.joint_tracking = Some(jt);
        let rig = LiveRig::start(
            OwnerKind::Executor,
            cfg,
            INSIDE_RAISED_POSE,
            OperatorAuth::disabled(),
            None,
            true,
        )
        .await;
        let mut ws = ws_connect(&rig.ws_url).await;
        let granted = ws_send_wait_types(
            &mut ws,
            json!({"type": "acquire_lease", "client_id": "gc-exitfail", "seq": 1}),
            &["lease_granted"],
        )
        .await;
        let gen = granted["generation"].as_u64().unwrap();
        let epoch = rig.sup.get_observation().await.epoch_id;

        let ack = ws_rpc(&mut ws, 1, gen, epoch, "/api/v1/mode/gravity_comp", json!({"enable": true})).await;
        assert_eq!(ack["type"], "command_ack", "{ack}");
        let dl = Instant::now() + Duration::from_secs(2);
        while rig.sup.get_observation().await.gravity_comp.phase.as_deref() != Some("active") {
            assert!(Instant::now() < dl);
            tokio::time::sleep(Duration::from_millis(15)).await;
        }

        // Request exit while reporting non-stationary velocity so exit_hold + settle expires
        // into ExitFailed (testing Major 1: ExitFailed tick must STILL emit stiff MIT hold!).
        rig.clear_bus_log();
        rig.bus.with(|s| {
            s.motors[2].v = 0.40; // exceeds entry_max_velocity
        });
        let ack_ex = ws_rpc(&mut ws, 2, gen, epoch, "/api/v1/mode/gravity_comp", json!({"enable": false})).await;
        assert_eq!(ack_ex["type"], "command_ack", "{ack_ex}");

        let dl_fail = Instant::now() + Duration::from_secs(3);
        loop {
            let obs = rig.sup.get_observation().await;
            if obs.gravity_comp.phase.as_deref() == Some("exit_failed") {
                break;
            }
            assert!(Instant::now() < dl_fail, "expected ExitFailed under sustained velocity: {:?}", obs.gravity_comp);
            tokio::time::sleep(Duration::from_millis(20)).await;
        }

        // Stop velocity so the periodic retry succeeds and completes MIT -> POS_VEL ModeSwitch.
        rig.bus.with(|s| {
            s.motors[2].v = 0.0;
        });
        let dl_done = Instant::now() + Duration::from_secs(3);
        while rig.sup.get_observation().await.gravity_comp.phase.is_some() {
            assert!(Instant::now() < dl_done, "ExitFailed retry never completed POS_VEL handback");
            tokio::time::sleep(Duration::from_millis(20)).await;
        }

        rig.bus.with(|s| {
            s.run_watchdogs();
            let exp: usize = s.motors.iter().map(|m| m.watchdog_expiry_count).sum();
            assert_eq!(exp, 0, "0 TIMEOUT expiries across ExitFailed + retry ModeSwitch");
            for j in 0..6 {
                assert_eq!(s.motors[j].ctrl_mode(), damiao::CTRL_MODE_POS_VEL);
            }
        });
        rig.assert_rt_clean("ExitFailed stiff hold continuity + retry ModeSwitch");
        rig.shutdown().await;
    }

    // Part B (Major 3): Trigger TrackingReject::JointLimit on RT executor (PositionLead) and
    // verify BOTH that the fallback ramp path executed AND that it performed 0 heap ops!
    {
        let mut cfg = ArmConfig::rebot_dm();
        let mut jt = JointTrackingConfig::rebot_dm();
        jt.enabled = true;
        for j in &mut jt.joints {
            j.strategy = TrackingStrategy::Position;
        }
        // Make J1 PositionLead with a huge lead gain so a normal jog near the limit triggers
        // TrackingReject::JointLimit inside the RT tick.
        jt.joints[0].strategy = TrackingStrategy::PositionLead;
        jt.joints[0].lag_s = 0.08;
        jt.joints[0].max_lead_rad = 0.03;
        cfg.joint_tracking = Some(jt);
        // Set tight J1 upper limit just past RAISED_POSE[0] so lead overshoots it.
        cfg.limits[0].max_position = RAISED_POSE[0] + 0.025;

        let rig = LiveRig::start(
            OwnerKind::Executor,
            cfg,
            RAISED_POSE,
            OperatorAuth::disabled(),
            None,
            true,
        )
        .await;
        let mut ws = ws_connect(&rig.ws_url).await;
        let granted = ws_send_wait_types(
            &mut ws,
            json!({"type": "acquire_lease", "client_id": "trk-rej-alloc", "seq": 1}),
            &["lease_granted"],
        )
        .await;
        let gen = granted["generation"].as_u64().unwrap();
        let epoch = rig.sup.get_observation().await.epoch_id;

        // Jog J1 right up to max_position; PositionLead pushes target past limit -> fallback ramp.
        let mut target = RAISED_POSE.to_vec();
        target[0] = RAISED_POSE[0] + 0.024;
        let ack = ws_rpc(
            &mut ws,
            1,
            gen,
            epoch,
            "/api/v1/motion/jog",
            json!({"target_positions": target, "duration_secs": 0.25, "speed_scale": 1.0}),
        )
        .await;
        assert_eq!(ack["type"], "command_ack", "{ack}");
        let dl_rej = Instant::now() + Duration::from_secs(2);
        let obs_rej = loop {
            let obs = rig.sup.get_observation().await;
            if matches!(obs.safety_state, SafetyState::Fault(_)) {
                break obs;
            }
            assert!(Instant::now() < dl_rej, "PositionLead TrackingReject did not latch Fault: {obs:?}");
            tokio::time::sleep(Duration::from_millis(15)).await;
        };
        assert!(
            obs_rej.fault.as_ref().is_some_and(|f| {
                (f.code == FaultCode::JointLimit
                    && f.message.as_str().contains("Joint tracking output rejected"))
                    || (f.code == FaultCode::ServoNoBrake
                        && f.message.as_str().contains("Joint tracking fallback cannot brake"))
            }),
            "expected TrackingReject / fallback braking fault detail, got {:?}",
            obs_rej.fault
        );

        rig.assert_rt_clean("PositionLead tracking limit/fallback zero-alloc");
        rig.shutdown().await;
    }
}

// ============================================================================
// 10. Majors 7 & 8: Multi-release delayed-disable MIT recovery & 1 s expiry reset
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_mit_recovery_multirelease_delayed_disable_and_1s_expiry() {
    let cfg = mit_config_j4();
    let rest = rest_pose_of(&cfg);
    let qual = valid_mit_qualification_j4(&cfg);
    let rig = LiveRig::start(
        OwnerKind::Executor,
        cfg.clone(),
        rest,
        OperatorAuth::disabled(),
        Some(qual),
        false,
    )
    .await;

    let mut ws = ws_connect(&rig.ws_url).await;
    let granted = ws_send_wait_types(
        &mut ws,
        json!({"type": "acquire_lease", "client_id": "mit-rec-retry", "seq": 1}),
        &["lease_granted"],
    )
    .await;
    let gen = granted["generation"].as_u64().unwrap();

    // Helper: arm, wait for MIT Active on J4, then induce 0xD expiry -> MitOwner::Lost.
    let mut req_id = 1u64;
    for case_idx in 0..2 {
        let ep = rig.sup.get_observation().await.epoch_id;
        let arm_ack = ws_rpc(&mut ws, req_id, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
        req_id += 1;
        assert_eq!(arm_ack["type"], "command_ack", "case {case_idx}: {arm_ack}");

        let dl = Instant::now() + Duration::from_secs(3);
        while rig.bus.with(|s| s.motors[3].ctrl_mode()) != damiao::CTRL_MODE_MIT {
            assert!(Instant::now() < dl, "case {case_idx}: MIT did not activate");
            tokio::time::sleep(Duration::from_millis(15)).await;
        }
        tokio::time::sleep(Duration::from_millis(80)).await;

        induce_j4_mit_expiry_fault(&rig, OwnerKind::Executor).await;
        // Wait past MIT_TRIP_HOLDOFF_MIN (250 ms) and feed fresh rest-pose feedback.
        tokio::time::sleep(Duration::from_millis(300)).await;
        for _ in 0..5 {
            rig.bus.with(|s| {
                for (i, m) in s.motors.iter_mut().enumerate() {
                    m.q = rest[i];
                    m.v = 0.0;
                }
                s.push_all_feedback();
            });
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        let fault_epoch = rig.sup.get_observation().await.epoch_id;

        if case_idx == 0 {
            // Case 0: Delay J1 disable reply across multiple RT releases (`drop_torque_frames = 2`)
            // while keeping rest-pose feedback fresh (30 ms cadence) -> spans multiple bounded RT
            // releases while single WS clear_fault retries within the 1 s lifetime and succeeds!
            rig.bus.with(|s| {
                s.motors[0].drop_torque_frames = 2;
                s.push_all_feedback();
            });
            let keep_fresh = Arc::new(AtomicBool::new(true));
            let kf_flag = Arc::clone(&keep_fresh);
            let kf_bus = rig.bus.clone();
            let feeder = tokio::spawn(async move {
                while kf_flag.load(Ordering::Acquire) {
                    kf_bus.with(|s| {
                        for (i, m) in s.motors.iter_mut().enumerate() {
                            m.q = rest[i];
                            m.v = 0.0;
                        }
                        s.push_all_feedback();
                    });
                    tokio::time::sleep(Duration::from_millis(30)).await;
                }
            });
            let ok = ws_rpc(
                &mut ws,
                req_id,
                gen,
                fault_epoch,
                "/api/v1/safety/clear_fault",
                json!({"mit_recovery": "torque_off_all"}),
            )
            .await;
            keep_fresh.store(false, Ordering::Release);
            let _ = feeder.await;
            req_id += 1;
            assert_eq!(ok["type"], "command_ack", "multi-release delayed disable must succeed: {ok}");
            assert_eq!(rig.sup.get_observation().await.safety_state, SafetyState::Idle);
        } else {
            // Case 1: Drop 500 disable frames on J1 while keeping fresh rest-pose feedback flowing
            // so Rule A′ freshness passes and the 1 s `DisableRequestExpired` lifetime is what
            // terminates recovery & resets driver state; restoring delivery allows a fresh RPC!
            rig.bus.with(|s| {
                s.motors[0].drop_torque_frames = 500;
            });
            let keep_fresh = Arc::new(AtomicBool::new(true));
            let kf_flag = Arc::clone(&keep_fresh);
            let kf_bus = rig.bus.clone();
            let feeder = tokio::spawn(async move {
                while kf_flag.load(Ordering::Acquire) {
                    kf_bus.with(|s| {
                        for (i, m) in s.motors.iter_mut().enumerate() {
                            m.q = rest[i];
                            m.v = 0.0;
                        }
                        s.push_all_feedback();
                    });
                    tokio::time::sleep(Duration::from_millis(30)).await;
                }
            });
            let t0 = Instant::now();
            let rej = ws_rpc(
                &mut ws,
                req_id,
                gen,
                fault_epoch,
                "/api/v1/safety/clear_fault",
                json!({"mit_recovery": "torque_off_all"}),
            )
            .await;
            keep_fresh.store(false, Ordering::Release);
            let _ = feeder.await;
            req_id += 1;
            assert_eq!(rej["type"], "command_reject", "unresponsive motor must expire at 1 s: {rej}");
            assert!(
                t0.elapsed() >= Duration::from_millis(900),
                "1 s lifetime should not expire prematurely ({:?}): {rej}",
                t0.elapsed()
            );

            // Restore J1 disable delivery and feed fresh rest-pose samples across 3 ticks so
            // Rule A′ freshness passes, then issue a brand-new clear_fault -> starts fresh & succeeds!
            rig.bus.with(|s| {
                s.motors[0].drop_torque_frames = 0;
            });
            for _ in 0..4 {
                rig.bus.with(|s| {
                    for (i, m) in s.motors.iter_mut().enumerate() {
                        m.q = rest[i];
                        m.v = 0.0;
                    }
                    s.push_all_feedback();
                });
                tokio::time::sleep(Duration::from_millis(20)).await;
            }
            let ep2 = rig.sup.get_observation().await.epoch_id;
            let ok2 = ws_rpc(
                &mut ws,
                req_id,
                gen,
                ep2,
                "/api/v1/safety/clear_fault",
                json!({"mit_recovery": "torque_off_all"}),
            )
            .await;
            assert_eq!(ok2["type"], "command_ack", "fresh clear_fault after 1 s expiry must succeed: {ok2}");
            assert_eq!(rig.sup.get_observation().await.safety_state, SafetyState::Idle);
        }
    }

    rig.assert_rt_clean("multi-release MIT recovery + 1 s expiry reset");
    rig.shutdown().await;
}

// ============================================================================
// 11. Major 11: Executor Park from holding_after_clear MIT qualification gate
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_park_after_clear_mit_qualification_gate() {
    // 11a. Qualified MIT profile: arm -> e_stop -> clear_fault (enters holding_after_clear) ->
    //      WS /api/v1/motion/unload must run run_mit_qualification_for_enable_executor, pass
    //      enable_prepare_rt_precheck, and receive command_ack!
    {
        let cfg = mit_config_j4();
        let qual = valid_mit_qualification_j4(&cfg);
        let rig = LiveRig::start(
            OwnerKind::Executor,
            cfg,
            RAISED_POSE,
            OperatorAuth::disabled(),
            Some(qual),
            false,
        )
        .await;
        let mut ws = ws_connect(&rig.ws_url).await;
        let granted = ws_send_wait_types(
            &mut ws,
            json!({"type": "acquire_lease", "client_id": "park-mit-ok", "seq": 1}),
            &["lease_granted"],
        )
        .await;
        let gen = granted["generation"].as_u64().unwrap();
        let epoch = rig.sup.get_observation().await.epoch_id;

        // Arm (stays in POS_VEL before MIT activates), then E-stop while torqued -> clear_fault
        // enters Idle + is_torqued + holding_after_clear!
        let arm_ack = ws_rpc(&mut ws, 1, gen, epoch, "/api/v1/torque", json!({"enable": true})).await;
        assert_eq!(arm_ack["type"], "command_ack", "{arm_ack}");

        // E-stop while torqued, then clear_fault -> Idle + is_torqued + holding_after_clear
        let _es = ws_send_wait_types(&mut ws, json!({"type": "e_stop", "seq": 2}), &["epoch_sync"]).await;
        let dl_es = Instant::now() + Duration::from_secs(2);
        let es_epoch = loop {
            let obs = rig.sup.get_observation().await;
            if obs.safety_state == SafetyState::EStop {
                break obs.epoch_id;
            }
            assert!(Instant::now() < dl_es);
            tokio::time::sleep(Duration::from_millis(10)).await;
        };
        let clr = ws_rpc(&mut ws, 2, gen, es_epoch, "/api/v1/safety/clear_fault", json!({})).await;
        assert_eq!(clr["type"], "command_ack", "{clr}");

        let dl_hold = Instant::now() + Duration::from_secs(2);
        let clear_epoch = loop {
            let snap = rig.sup.hub().latest().expect("snap");
            if snap.rt.holding_after_clear && snap.obs.safety_state == SafetyState::Idle {
                break snap.obs.epoch_id;
            }
            assert!(Instant::now() < dl_hold, "never entered holding_after_clear");
            tokio::time::sleep(Duration::from_millis(15)).await;
        };
        // Allow energized stationarity evidence to settle after clear_fault.
        tokio::time::sleep(Duration::from_millis(160)).await;

        // WS /api/v1/motion/unload from holding_after_clear MUST pass MIT qualification & ack!
        let unload_ack = ws_rpc(&mut ws, 3, gen, clear_epoch, "/api/v1/motion/unload", json!({})).await;
        if unload_ack["type"] != "command_ack" {
            let dbg_obs = rig.sup.get_observation().await;
            let dbg_modes: Vec<u32> = rig.bus.with(|s| s.motors.iter().map(|m| m.ctrl_mode()).collect());
            let dbg_cmds: Vec<usize> = rig.bus.with(|s| s.motors.iter().map(|m| m.commands).collect());
            let dbg_en: Vec<bool> = rig.bus.with(|s| s.motors.iter().map(|m| m.enabled).collect());
            panic!(
                "qualified MIT Executor Park from holding_after_clear failed: {unload_ack}\nobs.is_torqued={} obs.safety={:?} obs.vel={:?} modes={dbg_modes:?} cmds={dbg_cmds:?} en={dbg_en:?}",
                dbg_obs.is_torqued, dbg_obs.safety_state, dbg_obs.velocities
            );
        }
        rig.assert_rt_clean("Executor Park holding_after_clear qualified MIT");
        rig.shutdown().await;
    }

    // 11b. Unqualified MIT profile (stale/mismatched TIMEOUT on motor after clear):
    //      WS /api/v1/motion/unload from holding_after_clear MUST fail closed with MIT_UNQUALIFIED!
    {
        let cfg = mit_config_j4();
        let qual = valid_mit_qualification_j4(&cfg);
        let rig = LiveRig::start(
            OwnerKind::Executor,
            cfg,
            RAISED_POSE,
            OperatorAuth::disabled(),
            Some(qual),
            false,
        )
        .await;
        let mut ws = ws_connect(&rig.ws_url).await;
        let granted = ws_send_wait_types(
            &mut ws,
            json!({"type": "acquire_lease", "client_id": "park-mit-bad", "seq": 1}),
            &["lease_granted"],
        )
        .await;
        let gen = granted["generation"].as_u64().unwrap();
        let epoch = rig.sup.get_observation().await.epoch_id;

        let arm_ack = ws_rpc(&mut ws, 1, gen, epoch, "/api/v1/torque", json!({"enable": true})).await;
        assert_eq!(arm_ack["type"], "command_ack", "{arm_ack}");

        let _es = ws_send_wait_types(&mut ws, json!({"type": "e_stop", "seq": 2}), &["epoch_sync"]).await;
        let dl_es = Instant::now() + Duration::from_secs(2);
        let es_epoch = loop {
            let obs = rig.sup.get_observation().await;
            if obs.safety_state == SafetyState::EStop {
                break obs.epoch_id;
            }
            assert!(Instant::now() < dl_es);
            tokio::time::sleep(Duration::from_millis(10)).await;
        };
        let clr = ws_rpc(&mut ws, 2, gen, es_epoch, "/api/v1/safety/clear_fault", json!({})).await;
        assert_eq!(clr["type"], "command_ack", "{clr}");

        let dl_hold = Instant::now() + Duration::from_secs(2);
        let clear_epoch = loop {
            let snap = rig.sup.hub().latest().expect("snap");
            if snap.rt.holding_after_clear && snap.obs.safety_state == SafetyState::Idle {
                break snap.obs.epoch_id;
            }
            assert!(Instant::now() < dl_hold);
            tokio::time::sleep(Duration::from_millis(15)).await;
        };
        tokio::time::sleep(Duration::from_millis(160)).await;

        // Tamper J4 TIMEOUT register to 0 so qualification gate rejects Park rearm fail-closed!
        rig.bus.with(|s| {
            s.motors[3].set_reg_u32(damiao::RID_TIMEOUT, 0);
        });
        let unload_rej = ws_rpc(&mut ws, 3, gen, clear_epoch, "/api/v1/motion/unload", json!({})).await;
        assert_eq!(
            unload_rej["type"], "command_reject",
            "unqualified MIT Executor Park from holding_after_clear must reject: {unload_rej}"
        );
        let msg = unload_rej["message"].as_str().unwrap_or("");
        assert!(
            msg.contains("MIT_UNQUALIFIED") || msg.contains("TIMEOUT"),
            "expected MIT_UNQUALIFIED refusal, got {unload_rej}"
        );
        rig.assert_rt_clean("Executor Park holding_after_clear unqualified MIT");
        rig.shutdown().await;
    }
}

// ============================================================================
// 12. Major 4 regression: PositionLead, park ramp-out (keepout_only), and
//     fallback with keepout boxes + >256 workspace constraint slots must execute
//     AND do zero heap allocations/deallocations on armd-control.
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_tracking_keepout_and_large_workspace_zero_alloc() {
    let mut cfg = ArmConfig::rebot_dm();
    let mut jt = JointTrackingConfig::rebot_dm();
    jt.enabled = true;
    jt.joints[0].strategy = TrackingStrategy::PositionLead;
    jt.joints[0].lag_s = 0.08;
    jt.joints[0].max_lead_rad = 0.03;
    cfg.joint_tracking = Some(jt);

    // Configure > 256 workspace constraint slots (30 end points + 2 keepout boxes =>
    // (1 + 30 + link_capsules) * (7 + 2) >= 279 > MAX_STACK_WORKSPACE_SLOTS = 256)
    // AND keepout boxes well outside the arm's working envelope so normal motion passes.
    let ws = cfg.workspace.as_mut().expect("rebot_dm has workspace");
    ws.floor_z_m = -1.0;
    ws.box_min_m = [-1.5, -1.5, -1.0];
    ws.box_max_m = [1.5, 1.5, 1.5];
    ws.max_reach_m = Some(2.0);
    ws.check_points_end_m = (0..30)
        .map(|i| [0.0001 * i as f32, 0.0, 0.0])
        .collect();
    ws.keepout_boxes = vec![
        KeepoutBox {
            name: "far_obstacle_a".into(),
            min_m: [0.80, 0.80, 0.80],
            max_m: [0.95, 0.95, 0.95],
        },
        KeepoutBox {
            name: "far_obstacle_b".into(),
            min_m: [-0.95, 0.80, 0.80],
            max_m: [-0.80, 0.95, 0.95],
        },
    ];
    // Confirm total_slots > 256 so SlotBuf::Heap path is guaranteed active.
    let eval = armd::chain::WorkspaceEvaluator::new_with_tcp(
        &cfg.arm_model,
        ws,
        &cfg.camera_mount,
        cfg.tcp_offset,
    );
    assert!(
        eval.total_slots > armd::kinematics::MAX_STACK_WORKSPACE_SLOTS,
        "test workspace must exceed 256 slots, got {}",
        eval.total_slots
    );

    let rig = LiveRig::start(
        OwnerKind::Executor,
        cfg,
        RAISED_POSE,
        OperatorAuth::disabled(),
        None,
        true,
    )
    .await;
    let mut ws_sock = ws_connect(&rig.ws_url).await;
    let granted = ws_send_wait_types(
        &mut ws_sock,
        json!({"type": "acquire_lease", "client_id": "trk-large-ws", "seq": 1}),
        &["lease_granted"],
    )
    .await;
    let gen = granted["generation"].as_u64().unwrap();
    let epoch = rig.sup.get_observation().await.epoch_id;

    // 1) PositionLead normal motion (>256 slots): exercises compute_position_lead_output_inline.
    let mut target = RAISED_POSE.to_vec();
    target[0] = RAISED_POSE[0] + 0.12;
    let ack1 = ws_rpc(
        &mut ws_sock,
        1,
        gen,
        epoch,
        "/api/v1/motion/jog",
        json!({"target_positions": target, "duration_secs": 0.60, "speed_scale": 1.0}),
    )
    .await;
    assert_eq!(ack1["type"], "command_ack", "{ack1}");
    let dl_move = Instant::now() + Duration::from_secs(1);
    while rig.sup.get_observation().await.commanded_positions[0] <= RAISED_POSE[0] + 0.01 {
        assert!(Instant::now() < dl_move, "PositionLead jog never advanced J1");
        tokio::time::sleep(Duration::from_millis(15)).await;
    }
    rig.assert_rt_clean("PositionLead with >256 workspace slots");

    // 2) Park mid-motion while residual lead is active: exercises compute_lead_ramp_out_inline
    //    with keepout_only = true (no ws.keepout_only() clone + >256 slots)!
    let obs_pre_park = rig.sup.get_observation().await;
    let park_ack = ws_rpc(&mut ws_sock, 2, gen, obs_pre_park.epoch_id, "/api/v1/motion/unload", json!({})).await;
    assert_eq!(
        park_ack["type"],
        "command_ack",
        "pre_park={:?}/ep={} post_park={:?}/ep={} reply={park_ack}",
        obs_pre_park.safety_state,
        obs_pre_park.epoch_id,
        rig.sup.get_observation().await.safety_state,
        rig.sup.get_observation().await.epoch_id,
    );
    tokio::time::sleep(Duration::from_millis(200)).await;
    rig.assert_rt_clean("PositionLead + park ramp-out with keepout boxes and >256 workspace slots");
    rig.shutdown().await;

    // 3) Rejection fallback path with keepout boxes + >256 workspace slots:
    //    tight J1 upper limit + fast lead triggers fallback_workspace_check_inline under Executor.
    let mut cfg_fb = ArmConfig::rebot_dm();
    let mut jt_fb = JointTrackingConfig::rebot_dm();
    jt_fb.enabled = true;
    jt_fb.joints[0].strategy = TrackingStrategy::PositionLead;
    jt_fb.joints[0].lag_s = 0.08;
    jt_fb.joints[0].max_lead_rad = 0.03;
    cfg_fb.joint_tracking = Some(jt_fb);
    cfg_fb.limits[0].max_position = RAISED_POSE[0] + 0.025;
    let ws_fb = cfg_fb.workspace.as_mut().expect("rebot_dm has workspace");
    ws_fb.check_points_end_m = (0..30).map(|i| [0.0001 * i as f32, 0.0, 0.0]).collect();
    ws_fb.keepout_boxes = vec![KeepoutBox {
        name: "far_obstacle_fb".into(),
        min_m: [0.80, 0.80, 0.80],
        max_m: [0.95, 0.95, 0.95],
    }];
    let eval_fb = armd::chain::WorkspaceEvaluator::new_with_tcp(
        &cfg_fb.arm_model,
        ws_fb,
        &cfg_fb.camera_mount,
        cfg_fb.tcp_offset,
    );
    assert!(
        eval_fb.total_slots > armd::kinematics::MAX_STACK_WORKSPACE_SLOTS,
        "fallback workspace must exceed 256 slots, got {}",
        eval_fb.total_slots
    );

    let rig_fb = LiveRig::start(
        OwnerKind::Executor,
        cfg_fb,
        RAISED_POSE,
        OperatorAuth::disabled(),
        None,
        true,
    )
    .await;
    let mut ws_fb_sock = ws_connect(&rig_fb.ws_url).await;
    let granted_fb = ws_send_wait_types(
        &mut ws_fb_sock,
        json!({"type": "acquire_lease", "client_id": "trk-large-fb", "seq": 1}),
        &["lease_granted"],
    )
    .await;
    let gen_fb = granted_fb["generation"].as_u64().unwrap();
    let ep_fb = rig_fb.sup.get_observation().await.epoch_id;
    let mut target_fb = RAISED_POSE.to_vec();
    target_fb[0] = RAISED_POSE[0] + 0.024;
    rig_fb.clear_bus_log();
    let ack_fb = ws_rpc(
        &mut ws_fb_sock,
        1,
        gen_fb,
        ep_fb,
        "/api/v1/motion/jog",
        json!({"target_positions": target_fb, "duration_secs": 0.25, "speed_scale": 1.0}),
    )
    .await;
    assert_eq!(ack_fb["type"], "command_ack", "{ack_fb}");
    let dl_fb = Instant::now() + Duration::from_secs(2);
    let obs_fb = loop {
        let obs = rig_fb.sup.get_observation().await;
        if matches!(obs.safety_state, SafetyState::Fault(_)) {
            break obs;
        }
        assert!(Instant::now() < dl_fb, "tracking rejection fallback did not execute: {obs:?}");
        tokio::time::sleep(Duration::from_millis(15)).await;
    };
    // Explicitly assert TrackingReject + fallback braking executed (Minor 2 — not just zero allocs!).
    assert!(
        obs_fb.fault.as_ref().is_some_and(|f| {
            (f.code == FaultCode::JointLimit
                && f.message.as_str().contains("Joint tracking output rejected"))
                || (f.code == FaultCode::ServoNoBrake
                    && f.message.as_str().contains("Joint tracking fallback cannot brake"))
        }),
        "TrackingReject + fallback braking must record fault message, got {:?}",
        obs_fb.fault
    );
    assert!(
        obs_fb.commanded_positions[0] <= RAISED_POSE[0] + 0.025 + 1e-4,
        "fallback braking must clamp/hold J1 within limit (got {})",
        obs_fb.commanded_positions[0]
    );
    let ctrl_frames = rig_fb.bus.with(|s| {
        s.tx_log
            .iter()
            .filter(|f| (0x101..=0x106).contains(&f.id))
            .count()
    });
    assert!(
        ctrl_frames > 0,
        "fallback braking must emit control frames after TrackingReject"
    );

    rig_fb.assert_rt_clean("Fallback with keepout boxes and >256 workspace slots");
    rig_fb.shutdown().await;
}

// ============================================================================
// 13. Major 2 regression: mit_expiry_recover_budgeted must subtract Step 2's
//     already-spent release_rx_frames from the 64-frame per-release RX budget.
// ============================================================================

#[test]
fn mit_expiry_recover_budgeted_respects_step2_release_rx_frame_budget() {
    let cfg = ArmConfig::rebot_dm();
    let rest = rest_pose_of(&cfg);
    let limits = b601_limits();
    let bus = FakeBus::new(&limits);
    bus.with(|s| {
        s.poll_feeds_watchdog = false;
        s.immediate_replies = true;
        s.tx_step = Duration::from_micros(50);
        for (i, m) in s.motors.iter_mut().enumerate() {
            m.q = rest[i];
            m.v = 0.0;
            m.enabled = i != 3;
            if i == 3 {
                m.state_override = Some(0xD);
            }
        }
        s.push_all_feedback();
    });
    let mut drv = build_standalone_mit_recovery_driver(bus.clone(), &rest);
    let started_at = Instant::now();
    // Warm up K=3 causal rest-pose evidence window strictly after `started_at` so Rule A′ passes.
    for rel in 1..=3 {
        std::thread::sleep(Duration::from_millis(25));
        drv.begin_control_release(rel);
        bus.with(|s| s.push_all_feedback());
        drv.read_joint_states().unwrap();
    }

    // Release 4: Step 2 consumes 50 RX frames (leaving 64 - 50 = 14 frames in the release budget).
    std::thread::sleep(Duration::from_millis(25));
    drv.begin_control_release(4);
    bus.with(|s| {
        s.rx.clear();
        for k in 0..50 {
            s.push_feedback(k % 7);
        }
    });
    let rel4_now = Instant::now();
    drv.set_tx_release_deadline(Some(rel4_now + Duration::from_millis(500)));
    drv.read_joint_states().unwrap();
    assert_eq!(bus.with(|s| s.rx.len()), 0, "Step 2 should drain all 50 frames");

    // Enqueue 30 more RX frames before mit_expiry_recover_budgeted runs in the SAME release.
    bus.with(|s| {
        for k in 0..30 {
            s.push_feedback(k % 7);
        }
    });
    let now = Instant::now();
    let mut gate = || Ok(());
    drv.set_tx_release_deadline(Some(now + Duration::from_millis(500)));
    let res = drv.mit_expiry_recover_budgeted(
        now + Duration::from_millis(500),
        started_at,
        &mut gate,
    );
    drv.set_tx_release_deadline(None);
    let remaining_rx = bus.with(|s| s.rx.len());
    let consumed_by_recover = 30usize.saturating_sub(remaining_rx);
    let total_rx_in_release = 50 + consumed_by_recover;
    assert!(
        !res.unwrap(),
        "mit_expiry_recover_budgeted must pause (Pending) once the shared 64-frame release RX budget is exhausted"
    );
    assert!(
        total_rx_in_release <= usize::from(armd::rt_core::ops::RX_FRAME_BUDGET),
        "total RX frames in release 4 ({total_rx_in_release} = 50 Step 2 + {consumed_by_recover} recover) exceeded RX_FRAME_BUDGET ({})",
        armd::rt_core::ops::RX_FRAME_BUDGET
    );
    assert_eq!(
        remaining_rx, 16,
        "expected exactly 14 frames consumed by recovery (16 remaining in RX queue), got remaining={remaining_rx}"
    );

    // Release 5+: Step 2 drains the remaining 16 frames with a fresh 64-frame budget and
    // mit_expiry_recover_budgeted completes recovery cleanly.
    let mut finished = false;
    for rel in 5..=10 {
        drv.begin_control_release(rel);
        let t = Instant::now();
        drv.set_tx_release_deadline(Some(t + Duration::from_millis(500)));
        drv.read_joint_states().unwrap();
        finished = drv
            .mit_expiry_recover_budgeted(t + Duration::from_millis(500), started_at, &mut gate)
            .unwrap();
        drv.set_tx_release_deadline(None);
        if finished {
            break;
        }
    }
    assert!(
        finished,
        "recovery must complete cleanly on subsequent releases; bus={:?}",
        bus.with(|s| (
            s.tx_log.len(),
            s.rx.len(),
            s.motors.iter().map(|m| (m.enabled, m.state_override)).collect::<Vec<_>>()
        ))
    );
    assert!(bus.with(|s| s.motors.iter().all(|m| !m.enabled)));
}

// ============================================================================
// 14. Major 1 regression: A fault predating a fault-permitted gravity-comp
//     handback session must NOT satisfy mode_switch_stop_applied on timeout
//     between axes (must classify partial switch as Ambiguous + SAFE-HOLD,
//     not StoppedRecovered).
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn predating_fault_does_not_prove_applied_stop_for_gravity_comp_handback_timeout() {
    use armd::motor_mode::ModeProgress;
    use armd::rt_core::session::{SessionKind, SessionSlot};

    let cfg = ArmConfig::rebot_dm();
    let bus = configure_damiao_bus(RAISED_POSE, Duration::ZERO, 0);
    let drv = build_damiao_driver(bus.clone(), &cfg);
    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;
    sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();

    // Enter hardware gravity comp via PR0C ModeSwitch (J1..J6 -> MIT).
    let enter_sup = sup.clone();
    let enter_bus = bus.clone();
    let enter_task = tokio::spawn(async move {
        gravity_comp_command_with_auth(
            &enter_sup,
            enter_sup.capture_admission_stamp(),
            None,
            OperatorRequirement::Required(OperatorGrant::unenforced()),
            ClientId(51),
            true,
            Some(0.5),
            None,
        )
        .await
    });
    let dl = Instant::now() + Duration::from_secs(3);
    while !enter_task.is_finished() {
        assert!(Instant::now() < dl, "gravity comp enter timed out");
        enter_bus.with(|s| s.push_all_feedback());
        let _ = sup.tick(0.02).await;
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    let enter_reply = enter_task.await.unwrap().expect("gravity comp enter must succeed");
    assert!(enter_reply.active);
    assert!(bus.with(|s| (0..6).all(|i| s.motors[i].ctrl_mode() == damiao::CTRL_MODE_MIT)));

    // Switch FakeBus to deferred replies (`immediate_replies = false`) and latch a predating
    // non-E-stop fault BEFORE gravity-comp handback starts.
    bus.with(|s| {
        s.immediate_replies = false;
    });
    {
        let mut inner = sup.inner().write().await;
        inner.safety.latch(
            FaultKind::Other,
            armd::safety::FaultDetail::formatted(armd::safety::FaultCode::Other, |b| {
                let _ = std::fmt::Write::write_str(b, "predating fault");
            }),
        );
    }

    // Trigger fault-permitted GravityCompExit handback session; once admitted, cap driver TX
    // budget to 6 frames/release and space ticks by 25 ms so energized hold refreshes consume all
    // 6 frames on the tick J1 confirms, pausing strictly between J1 Confirmed and J2 NeedWrite.
    let mut exit_sid: Option<u64> = None;
    let mut j1_switched_between_axes = false;
    for _ in 0..40 {
        if exit_sid.is_some() && bus.with(|s| s.motors[0].ctrl_mode() == damiao::CTRL_MODE_POS_VEL) {
            tokio::time::sleep(Duration::from_millis(25)).await;
        }
        bus.with(|s| s.push_all_feedback());
        let _ = sup.tick(0.02).await;
        if exit_sid.is_none() {
            let mut inner = sup.inner().write().await;
            if let Some(s) = inner.sessions().slot(SessionSlot::ExclusiveOutput) {
                if s.is_live() && s.kind == SessionKind::ModeSwitch {
                    exit_sid = Some(s.id.0);
                    if let Some(slot) = inner.driver.as_mut() {
                        let mut b = TxBudget::default();
                        // Energized 6 arm + J7 requires max_frames >= 8 (7 continuity + 1 register slot).
                        b.max_frames = 8;
                        let _ = slot.try_with_mut(|d| d.set_tx_budget(b));
                    }
                }
            }
        }
        if let Some(sid) = exit_sid {
            let snap = sup.poll_mode_switch_session(sid).await.unwrap().unwrap();
            if snap.running && (snap.confirmed_mask & 1) != 0 && (snap.confirmed_mask & 2) == 0 {
                j1_switched_between_axes = true;
                break;
            }
        }
    }
    let sid = exit_sid.expect("fault must launch fault-permitted GravityCompExit ModeSwitch session");
    assert!(
        j1_switched_between_axes,
        "expected J1 confirmed in POS_VEL while J2 is still in MIT during handback; snap={:?}",
        sup.poll_mode_switch_session(sid).await
    );

    // Wait past the 2 s handback session deadline WITHOUT any new stop/E-stop/fault so
    // SessionDriver revokes it with TIMEOUT right between axes.
    tokio::time::sleep(Duration::from_millis(2100)).await;
    bus.with(|s| s.push_all_feedback());
    let _ = sup.tick(0.02).await;

    let terminal = sup
        .poll_mode_switch_session(sid)
        .await
        .unwrap()
        .expect("terminal mode switch snapshot");
    assert!(!terminal.running, "session must be terminal after timeout");
    assert!(
        matches!(terminal.progress, ModeProgress::Ambiguous { .. }),
        "predating fault must NOT count as applied-stop proof on timeout between axes; expected Ambiguous, got {:?}",
        terminal.progress
    );
    assert!(
        sup.inner().read().await.sessions().is_safe_hold_requested(),
        "partial ModeSwitch timeout with only a predating fault must escalate to SAFE-HOLD"
    );
}

// ============================================================================
// 15. Major 3 regression: Dual-owner MIT expiry recovery parity when an E-stop
//     latch is ALREADY applied (only a NEW unapplied E-stop request may abort
//     clear_fault's MIT recovery; an applied E-stop latch must clear identically
//     on Legacy and Executor).
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_mit_expiry_recovery_after_applied_estop_latch() {
    let mut results = Vec::new();
    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let cfg = mit_config_j4();
        let rest = rest_pose_of(&cfg);
        let qual = valid_mit_qualification_j4(&cfg);
        let rig = LiveRig::start(
            owner,
            cfg,
            rest,
            OperatorAuth::disabled(),
            Some(qual),
            false,
        )
        .await;

        let mut ws = ws_connect(&rig.ws_url).await;
        let granted = ws_send_wait_types(
            &mut ws,
            json!({"type": "acquire_lease", "client_id": "mit-estop-rec", "seq": 1}),
            &["lease_granted"],
        )
        .await;
        let gen = granted["generation"].as_u64().unwrap();
        let ep0 = rig.sup.get_observation().await.epoch_id;

        let arm_ack = ws_rpc(&mut ws, 1, gen, ep0, "/api/v1/torque", json!({"enable": true})).await;
        assert_eq!(arm_ack["type"], "command_ack", "{owner:?}: {arm_ack}");

        let dl_mit = Instant::now() + Duration::from_secs(3);
        while rig.bus.with(|s| s.motors[3].ctrl_mode()) != damiao::CTRL_MODE_MIT {
            assert!(Instant::now() < dl_mit, "{owner:?}: MIT did not activate");
            tokio::time::sleep(Duration::from_millis(15)).await;
        }
        tokio::time::sleep(Duration::from_millis(80)).await;

        // Induce J4 0xD expiry -> MitOwner::Lost + Fault.
        induce_j4_mit_expiry_fault(&rig, owner).await;
        let fault_epoch = rig.sup.get_observation().await.epoch_id;

        // Now apply E-stop while MIT is Lost: produces an ALREADY-APPLIED E-stop latch.
        let es_msg = ws_send_wait_types(&mut ws, json!({"type": "e_stop", "seq": 2}), &["epoch_sync"]).await;
        let es_epoch = es_msg["epoch_id"].as_u64().unwrap();
        assert!(es_epoch > fault_epoch, "{owner:?}: E-stop must advance epoch past {fault_epoch}");
        let dl_es = Instant::now() + Duration::from_secs(2);
        loop {
            let obs = rig.sup.get_observation().await;
            if obs.epoch_id >= es_epoch {
                break;
            }
            assert!(Instant::now() < dl_es, "{owner:?}: E-stop epoch did not publish");
            tokio::time::sleep(Duration::from_millis(15)).await;
        }
        // Wait out the MIT quiet period (~200 ms) while pushing periodic rest-pose feedback.
        for _ in 0..16 {
            rig.bus.with(|s| s.push_all_feedback());
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        let clear_epoch = rig.sup.get_observation().await.epoch_id;

        // Call 1: clear_fault with mit_recovery = "torque_off_all" while MIT is Lost AND E-stop
        // latch is already applied. Both Legacy and Executor must run MIT expiry recovery (clearing
        // 0xD and disabling all motors, resetting MitOwner::Off), re-latch EStop, and return
        // 409 ESTOP_ACTIVE (never stranding Executor at pre_tx_gate with motors still enabled).
        let clr1 = ws_rpc(
            &mut ws,
            2,
            gen,
            clear_epoch,
            "/api/v1/safety/clear_fault",
            json!({"mit_recovery": "torque_off_all"}),
        )
        .await;
        assert_eq!(
            clr1["type"], "command_reject",
            "{owner:?}: first clear_fault with applied E-stop latch must preserve EStopLatch, got {clr1}"
        );
        assert_eq!(clr1["code"], "ESTOP_ACTIVE", "{owner:?}: {clr1}");
        assert_eq!(clr1["safety_state"], "EStop", "{owner:?}: {clr1}");
        assert!(
            rig.bus.with(|s| s.motors.iter().all(|m| !m.enabled && m.state_override.is_none())),
            "{owner:?}: first clear_fault must execute MIT recovery (clear 0xD and disable all motors)"
        );

        // Call 2: clear_fault at the new EStop epoch now clears EStop cleanly to Idle on BOTH owners!
        for _ in 0..4 {
            rig.bus.with(|s| s.push_all_feedback());
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        let estop_epoch2 = rig.sup.get_observation().await.epoch_id;
        let clr2 = ws_rpc(
            &mut ws,
            3,
            gen,
            estop_epoch2,
            "/api/v1/safety/clear_fault",
            json!({}),
        )
        .await;
        assert_eq!(
            clr2["type"], "command_ack",
            "{owner:?}: second clear_fault after MIT recovery must clear EStop to Idle, got {clr2}"
        );
        let obs_after = rig.sup.get_observation().await;
        assert_eq!(obs_after.safety_state, SafetyState::Idle, "{owner:?}: must return to Idle");
        assert!(!obs_after.is_torqued, "{owner:?}: torque must be off after MIT expiry recovery");
        assert!(
            rig.bus.with(|s| s.motors.iter().all(|m| !m.enabled)),
            "{owner:?}: all motors must be disabled after MIT expiry recovery"
        );
        rig.assert_rt_clean(&format!("MIT expiry recovery after applied E-stop ({owner:?})"));
        results.push((
            clr1["code"].clone(),
            clr1["safety_state"].clone(),
            clr2["type"].clone(),
            obs_after.is_torqued,
        ));
        rig.shutdown().await;
    }
    assert_eq!(results[0], results[1], "Legacy and Executor MIT recovery after applied E-stop must match");
}

// ============================================================================
// 16. Major 5 regression: GravityCompOp::Enter (driverless / non-ModeSwitch
//     fallback) must carry the live OperatorGrant and re-check it at owner
//     admission on BOTH Legacy and Executor (revoked/expired grant fails closed
//     with 403 OPERATOR_REQUIRED).
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_gravity_comp_op_enter_rechecks_revoked_operator_grant() {
    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let cfg = ArmConfig::rebot_dm();
        let sup = Arc::new(ArmSupervisor::new(cfg));
        sup.set_lease_enabled(false).await;
        sup.set_operator_auth_enforced(true).await;
        {
            let mut inner = sup.inner().write().await;
            inner.measured_positions = RAISED_POSE.to_vec();
            inner.commanded_positions = RAISED_POSE.to_vec();
            inner.target_positions = RAISED_POSE.to_vec();
            inner.measured_velocities = vec![0.0; 7];
            inner.feedback_numeric_valid = true;
        }
        sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();

        let exec = if owner == OwnerKind::Executor {
            let mut ecfg = ExecutorConfig::for_tests(PERIOD_50HZ);
            ecfg.alloc_timing = true;
            ecfg.panic_policy = PanicPolicy::StopExecutor;
            let h = sup.start_executor(ecfg).await.unwrap();
            let dl = Instant::now() + Duration::from_secs(2);
            while sup.hub().latest().is_none() {
                assert!(Instant::now() < dl);
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
            Some(h)
        } else {
            None
        };

        let auth = OperatorAuth::enforced("123456", 1800);
        let (tok, _) = auth.unlock("123456").unwrap();
        let grant = auth.operator_grant(Some(&tok));
        assert!(grant.is_valid_at(Instant::now(), true));
        // Revoke the operator session between WS/REST precheck and owner admission.
        auth.lock(Some(&tok));
        assert!(!grant.is_valid_at(Instant::now(), true));

        let stamp = capture_config_admission(&sup, 0, 0, None);
        let err = gravity_comp_command_with_auth(
            &sup,
            stamp,
            None,
            OperatorRequirement::Required(grant),
            ClientId(0),
            true,
            Some(0.5),
            None,
        )
        .await
        .expect_err(&format!(
            "{owner:?}: GravityCompOp::Enter with revoked OperatorGrant must fail closed"
        ));
        match err {
            armd::ArmError::Rejected { status, code, .. } => {
                assert_eq!(status, 403, "{owner:?}: expected 403, got {status}/{code}");
                assert_eq!(code, "OPERATOR_REQUIRED", "{owner:?}: expected OPERATOR_REQUIRED, got {code}");
            }
            other => panic!("{owner:?}: expected 403 OPERATOR_REQUIRED, got {other:?}"),
        }
        assert_ne!(
            sup.get_observation().await.control_mode,
            ControlMode::GravityComp,
            "{owner:?}: gravity_comp must remain inactive after revoked-grant refusal"
        );

        // With a fresh live grant, GravityCompOp::Enter must succeed.
        let (tok2, _) = auth.unlock("123456").unwrap();
        let valid_grant = auth.operator_grant(Some(&tok2));
        let stamp2 = capture_config_admission(&sup, 0, 0, None);
        let ok = gravity_comp_command_with_auth(
            &sup,
            stamp2,
            None,
            OperatorRequirement::Required(valid_grant),
            ClientId(0),
            true,
            Some(0.5),
            None,
        )
        .await
        .unwrap_or_else(|e| panic!("{owner:?}: live OperatorGrant must be admitted: {e:?}"));
        assert!(ok.active, "{owner:?}: gravity_comp should be active with valid grant");

        if let Some(h) = exec {
            assert_eq!(h.stats().total_allocs(), AllocCounts::ZERO, "Executor GravityCompOp::Enter must be zero-alloc");
            tokio::task::spawn_blocking(move || h.stop_and_join()).await.unwrap();
        }
    }
}

// ============================================================================
// 17. Major 6 regression: A Legacy driver with synchronous MIT mode switching
//     (`supports_mit_mode_switch() == true`) and NO non-blocking PR0C support
//     (`supports_nonblocking_mode_switch() == false`, like `SimDriver`) must
//     switch driver modes via `set_joint_modes` on gravity-comp entry instead
//     of falling into the driverless no-op branch.
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn legacy_sync_mit_driver_gravity_comp_enter_switches_driver_modes() {
    let mode_history = Arc::new(Mutex::new(Vec::new()));
    let fail_next = Arc::new(AtomicBool::new(false));
    let drv = SyncMitSimLikeDriver {
        pose: INSIDE_RAISED_POSE,
        enabled: false,
        modes: vec![JointMode::Position; 7],
        mode_history: mode_history.clone(),
        fail_next_switch: fail_next.clone(),
    };
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;
    sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
    let _ = sup.tick(0.02).await;

    // 1) If sync `set_joint_modes` fails, entry MUST fail closed (never ack active=true).
    fail_next.store(true, Ordering::SeqCst);
    let stamp_fail = capture_config_admission(&sup, 0, 0, None);
    let fail_res = gravity_comp_command_with_auth(
        &sup,
        stamp_fail,
        None,
        OperatorRequirement::Required(OperatorGrant::unenforced()),
        ClientId(0),
        true,
        Some(0.5),
        None,
    )
    .await;
    assert!(
        fail_res.is_err(),
        "Legacy sync-MIT driver failure in set_joint_modes must reject gravity comp entry, got {fail_res:?}"
    );
    assert_ne!(sup.get_observation().await.control_mode, ControlMode::GravityComp);

    // Clear the latched fault from failed switch, re-arm, and enter cleanly.
    let _ = sup.tick(0.02).await;
    let _ = sup.clear_fault().await;
    sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
    let _ = sup.tick(0.02).await;
    mode_history.lock().unwrap().clear();

    // 2) Healthy sync-MIT driver entry MUST call `set_joint_modes([Mit; 6, Position])`.
    let stamp_ok = capture_config_admission(&sup, 0, 0, None);
    let ok = gravity_comp_command_with_auth(
        &sup,
        stamp_ok,
        None,
        OperatorRequirement::Required(OperatorGrant::unenforced()),
        ClientId(0),
        true,
        Some(0.5),
        None,
    )
    .await
    .expect("Legacy sync-MIT driver gravity comp entry must succeed");
    assert!(ok.active, "gravity comp must be active");
    {
        let hist = mode_history.lock().unwrap();
        assert_eq!(
            hist.len(),
            1,
            "sync-MIT driver must receive exactly 1 set_joint_modes call on entry, got {hist:?}"
        );
        let expected_mit = vec![
            JointMode::Mit,
            JointMode::Mit,
            JointMode::Mit,
            JointMode::Mit,
            JointMode::Mit,
            JointMode::Mit,
            JointMode::Position,
        ];
        assert_eq!(hist[0], expected_mit, "entry must switch J1..J6 to MIT and keep J7 Position");
    }

    // 3) Exiting gravity comp switches J1..J7 back to Position via set_joint_modes.
    let stamp_exit = capture_config_admission(&sup, 0, 0, None);
    let _ = gravity_comp_command_with_auth(
        &sup,
        stamp_exit,
        None,
        OperatorRequirement::NotRequired,
        ClientId(0),
        false,
        None,
        None,
    )
    .await
    .expect("Legacy sync-MIT driver gravity comp exit request must succeed");
    for _ in 0..20 {
        let _ = sup.tick(0.02).await;
        if sup.get_observation().await.control_mode != ControlMode::GravityComp {
            break;
        }
    }
    assert_ne!(sup.get_observation().await.control_mode, ControlMode::GravityComp);
    let hist = mode_history.lock().unwrap();
    assert!(
        hist.last() == Some(&vec![JointMode::Position; 7]),
        "exit must switch sync-MIT driver back to Position on all joints, history={hist:?}"
    );
}

// ============================================================================
// 18. Major 7 regression: On OWNER_EXECUTOR, `NoDestination` from router
//     terminal reservation MUST fail closed (`503 EXECUTOR_ROUTING_PENDING`)
//     and NEVER admit a ModeSwitch session without a staged router terminal.
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_gravity_comp_no_destination_fails_closed_without_admitting_session() {
    let cfg = ArmConfig::rebot_dm();
    let bus = configure_damiao_bus(RAISED_POSE, Duration::ZERO, TIMEOUT_2000_COUNTS);
    let drv = build_damiao_driver(bus.clone(), &cfg);
    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;
    sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();

    // Start Executor WITHOUT attaching a global sink and WITHOUT registering ClientId(777)
    // in ActionReplyRouter, so both ClientId(0) and ClientId(777) hit `ExternalSubmitError::NoDestination`.
    let mut ecfg = ExecutorConfig::for_tests(PERIOD_50HZ);
    ecfg.alloc_timing = true;
    ecfg.panic_policy = PanicPolicy::StopExecutor;
    let exec = sup.start_executor(ecfg).await.unwrap();
    let dl = Instant::now() + Duration::from_secs(2);
    while sup.hub().latest().is_none() {
        assert!(Instant::now() < dl);
        tokio::time::sleep(Duration::from_millis(5)).await;
    }

    for client in [ClientId(0), ClientId(777)] {
        let stamp = capture_config_admission(&sup, client.0, 0, None);
        let err = gravity_comp_command_with_auth(
            &sup,
            stamp,
            None,
            OperatorRequirement::Required(OperatorGrant::unenforced()),
            client,
            true,
            Some(0.5),
            None,
        )
        .await
        .expect_err(&format!(
            "NoDestination for {client:?} must fail closed without admitting a ModeSwitch session"
        ));
        match err {
            armd::ArmError::Rejected { status, code, .. } => {
                assert_eq!(status, 503, "{client:?}: expected 503, got {status}/{code}");
                assert_eq!(
                    code, "EXECUTOR_ROUTING_PENDING",
                    "{client:?}: expected EXECUTOR_ROUTING_PENDING, got {code}"
                );
            }
            other => panic!("{client:?}: expected 503 EXECUTOR_ROUTING_PENDING, got {other:?}"),
        }
        assert_ne!(
            sup.get_observation().await.control_mode,
            ControlMode::GravityComp,
            "{client:?}: gravity comp must NOT be active after NoDestination refusal"
        );
        assert!(
            bus.with(|s| (0..6).all(|i| s.motors[i].ctrl_mode() == damiao::CTRL_MODE_POS_VEL)),
            "{client:?}: motors must remain in POS_VEL (zero mode switches admitted)"
        );
    }

    assert_eq!(exec.stats().total_allocs(), AllocCounts::ZERO);
    tokio::task::spawn_blocking(move || exec.stop_and_join()).await.unwrap();
}

// ============================================================================
// 19. Major 8 regression: Per-release J7 audit catches duplicate 0x307 frames
//     in the same release (even spaced >10 ms apart, which a 10 ms sliding
//     window misses), and strict 40 ms gap check rejects 41 ms gaps.
// ============================================================================

#[test]
fn audit_bus_log_per_release_j7_and_strict_40ms_gap_check() {
    let t0 = Instant::now();
    let j7 = CanFrame::new_standard(0x307, &[0; 8]).unwrap();
    let j1 = CanFrame::new_standard(0x101, &[0; 8]).unwrap();

    // Case 1: Two J7 frames emitted 15 ms apart inside the SAME release_id (rel=7).
    // A 10 ms sliding window would report 1, but per-release ID audit MUST report 2!
    let log_dup = vec![j7, j7];
    let meta_dup = vec![Some((t0 + Duration::from_millis(1), 7)), Some((t0 + Duration::from_millis(16), 7))];
    let audit_dup = audit_bus_log(&log_dup, &meta_dup, [damiao::CTRL_MODE_POS_VEL; 6], t0, t0 + Duration::from_millis(20));
    assert_eq!(audit_dup.j7_total_frames, 2);
    assert_eq!(
        audit_dup.j7_max_per_release, 2,
        "two J7 frames in the same release_id must be caught even when spaced >10 ms apart"
    );

    // Case 2: Two J7 frames emitted 5 ms apart across DISTINCT release_ids (rel=7 and rel=8)
    // must pass with j7_max_per_release == 1.
    let meta_ok = vec![Some((t0 + Duration::from_millis(1), 7)), Some((t0 + Duration::from_millis(6), 8))];
    let audit_ok = audit_bus_log(&log_dup, &meta_ok, [damiao::CTRL_MODE_POS_VEL; 6], t0, t0 + Duration::from_millis(20));
    assert_eq!(audit_ok.j7_max_per_release, 1);

    // Case 3: A 41 ms gap on J1 MUST exceed the strict 40 ms bound (0 ms jitter slack).
    let log_gap = vec![j1, j1];
    let meta_gap = vec![Some((t0, 1)), Some((t0 + Duration::from_millis(41), 3))];
    let audit_gap = audit_bus_log(&log_gap, &meta_gap, [damiao::CTRL_MODE_POS_VEL; 6], t0, t0 + Duration::from_millis(41));
    assert!(
        audit_gap.max_gap[0] > Duration::from_millis(40),
        "41 ms gap must exceed strict 40 ms watchdog continuity bound (got {:?})",
        audit_gap.max_gap[0]
    );
}

// ============================================================================
// 20. Blocker 1 & Minor 1: Causal post-disable confirmation, incomplete
//     post-disable RX drain, and ClearFault flush failure retry
// ============================================================================

#[test]
fn mit_expiry_recover_causal_confirmation_and_flush_failure_retry() {
    let cfg = ArmConfig::rebot_dm();
    let rest = rest_pose_of(&cfg);
    let limits = b601_limits();

    // (a1) Blocker 1 part 1: Stale `state == 0` feedback predating CMD_DISABLE MUST NOT commit.
    {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&limits, clock.clone());
        bus.with(|s| {
            s.poll_feeds_watchdog = false;
            s.immediate_replies = true;
            s.tx_step = Duration::from_micros(50);
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = rest[i];
                m.v = 0.0;
                m.enabled = i != 3;
                if i == 3 {
                    m.state_override = Some(0xD);
                }
            }
            s.push_all_feedback();
        });
        let mut drv = build_standalone_mit_recovery_driver(bus.clone(), &rest);
        let started_at = clock.now();
        for rel in 1..=3 {
            clock.advance(Duration::from_millis(25));
            drv.begin_control_release(rel);
            bus.with(|s| s.push_all_feedback());
            drv.read_joint_states().unwrap();
        }

        // Delay J1's real disable reply by 20 ms AND inject a pre-dated stale `state == 0`
        // feedback frame (timestamped 5 ms BEFORE CMD_DISABLE) when J1's CMD_DISABLE runs.
        clock.advance(Duration::from_millis(25));
        let stale_injected = Arc::new(AtomicBool::new(false));
        let stale_cb = Arc::clone(&stale_injected);
        let clk_cb = clock.clone();
        let stale_j1 = bus.with(|s| {
            s.motors[0].torque_reply_delay = Duration::from_millis(20);
            let prev = s.motors[0].enabled;
            s.motors[0].enabled = false;
            let f = s.motors[0].feedback_frame();
            s.motors[0].enabled = prev;
            s.on_disable_state = Some(Arc::new(move |st| {
                if !stale_cb.swap(true, Ordering::SeqCst) {
                    let pre_tx_ts = clk_cb.now().checked_sub(Duration::from_millis(5)).unwrap();
                    st.push_rx_at(f, Some(pre_tx_ts));
                }
            }));
            f
        });
        let _ = stale_j1;

        drv.begin_control_release(4);
        bus.with(|s| s.push_all_feedback());
        drv.read_joint_states().unwrap();
        let dl4 = clock.now() + Duration::from_millis(500);
        let mut gate = || Ok(());
        drv.set_tx_release_deadline(Some(dl4));
        let r4 = drv.mit_expiry_recover_budgeted(dl4, started_at, &mut gate).unwrap();
        drv.set_tx_release_deadline(None);
        assert!(stale_injected.load(Ordering::SeqCst), "stale state==0 frame must have been injected");
        assert!(
            !r4,
            "stale state==0 feedback predating CMD_DISABLE must NOT commit mit_expiry_recover_budgeted"
        );

        // Advance past the 20 ms torque_reply_delay so real post-disable J1 feedback arrives -> commits!
        clock.advance(Duration::from_millis(25));
        bus.with(|s| {
            s.on_disable_state = None;
        });
        drv.begin_control_release(5);
        drv.read_joint_states().unwrap();
        let dl5 = clock.now() + Duration::from_millis(500);
        drv.set_tx_release_deadline(Some(dl5));
        let r5 = drv.mit_expiry_recover_budgeted(dl5, started_at, &mut gate).unwrap();
        drv.set_tx_release_deadline(None);
        assert!(r5, "real post-disable feedback after CMD_DISABLE must complete recovery");
        assert!(bus.with(|s| s.motors.iter().all(|m| !m.enabled && m.state_override.is_none())));
    }

    // (a2) Blocker 1 part 2: Incomplete post-disable RX drain (extra RX frames still queued
    //      after CMD_DISABLE) CANNOT commit `mit_expiry_recover_budgeted`.
    {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&limits, clock.clone());
        bus.with(|s| {
            s.poll_feeds_watchdog = false;
            s.immediate_replies = true;
            s.tx_step = Duration::from_micros(50);
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = rest[i];
                m.v = 0.0;
                m.enabled = i != 3;
                if i == 3 {
                    m.state_override = Some(0xD);
                }
            }
            s.push_all_feedback();
        });
        let mut drv = build_standalone_mit_recovery_driver(bus.clone(), &rest);
        let started_at = clock.now();
        for rel in 1..=3 {
            clock.advance(Duration::from_millis(25));
            drv.begin_control_release(rel);
            bus.with(|s| s.push_all_feedback());
            drv.read_joint_states().unwrap();
        }

        // On the 7th CMD_DISABLE frame, push 70 extra feedback frames into `rx` so post-disable
        // RX drain exceeds RX_FRAME_BUDGET (64) and leaves frames queued in `rx`.
        clock.advance(Duration::from_millis(25));
        let dis_seen = Arc::new(AtomicUsize::new(0));
        let dis_cb = Arc::clone(&dis_seen);
        let clk_cb = clock.clone();
        bus.with(|s| {
            s.on_disable_state = Some(Arc::new(move |st| {
                if dis_cb.fetch_add(1, Ordering::SeqCst) + 1 == 7 {
                    let ts = clk_cb.now();
                    for k in 0..70 {
                        let f = st.motors[k % 7].feedback_frame();
                        st.push_rx_at(f, Some(ts));
                    }
                }
            }));
        });

        drv.begin_control_release(4);
        bus.with(|s| s.push_all_feedback());
        drv.read_joint_states().unwrap();
        let dl4 = clock.now() + Duration::from_millis(500);
        let mut gate = || Ok(());
        drv.set_tx_release_deadline(Some(dl4));
        let r4 = drv.mit_expiry_recover_budgeted(dl4, started_at, &mut gate).unwrap();
        drv.set_tx_release_deadline(None);
        assert!(
            !r4,
            "incomplete post-disable RX drain must return Ok(false) and NOT commit"
        );
        assert!(
            bus.with(|s| !s.rx.is_empty()),
            "extra RX frames must still remain queued after release 4 budget exhaustion"
        );

        // Subsequent releases drain remaining frames to empty and complete 2-round confirmation
        // under the 16-frame TX release budget cleanly.
        bus.with(|s| s.on_disable_state = None);
        let mut committed = false;
        for rel in 5..=7 {
            clock.advance(Duration::from_millis(10));
            drv.begin_control_release(rel);
            drv.read_joint_states().unwrap();
            let dl = clock.now() + Duration::from_millis(500);
            drv.set_tx_release_deadline(Some(dl));
            let r = drv.mit_expiry_recover_budgeted(dl, started_at, &mut gate).unwrap();
            drv.set_tx_release_deadline(None);
            if r {
                committed = true;
                break;
            }
        }
        assert!(committed, "completing post-disable RX drain across budgeted releases must commit recovery");
        assert!(bus.with(|s| s.motors.iter().all(|m| !m.enabled && m.state_override.is_none())));
    }

    // (b) Minor 1: Bus flush failure on `ClearFault` must NOT set `mit_expiry_cleared_mask`,
    //     so the next release re-sends `ClearFault` and finishes cleanly.
    {
        #[derive(Clone)]
        struct ClearFaultFlushFailBus {
            inner: FakeBus,
            fail_next_clear_fault_flush: Arc<AtomicBool>,
            armed_flush_err: Arc<AtomicBool>,
        }

        impl CanTransport for ClearFaultFlushFailBus {
            fn clock(&self) -> Option<Arc<dyn DriverClock>> {
                self.inner.clock()
            }
            fn rx_clock_now(&self) -> Option<CanRxTime> {
                self.inner.rx_clock_now()
            }
            fn rx_clock_sample(&self) -> Option<RxClockSample> {
                self.inner.rx_clock_sample()
            }
            fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
                if frame.data == damiao::CMD_CLEAR_FAULTS
                    && self.fail_next_clear_fault_flush.swap(false, Ordering::SeqCst)
                {
                    // Simulate that the ClearFault frame failed to flush onto the wire:
                    // fail the next flush() AND preserve motor 3's 0xD fault state.
                    self.armed_flush_err.store(true, Ordering::SeqCst);
                    self.inner.with(|s| {
                        s.tx_log.push(*frame);
                        s.tx_times.push(s.now());
                    });
                    return Ok(());
                }
                self.inner.transmit(frame)
            }
            fn receive(&mut self) -> Result<CanFrame, TransportError> {
                self.inner.receive()
            }
            fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
                self.inner.receive_timestamped()
            }
            fn receive_with_rx_time(&mut self) -> Result<(CanFrame, Option<CanRxTime>), TransportError> {
                self.inner.receive_with_rx_time()
            }
            fn flush(&mut self) -> Result<(), TransportError> {
                if self.armed_flush_err.swap(false, Ordering::SeqCst) {
                    return Err(TransportError::Io(std::io::Error::from(
                        std::io::ErrorKind::BrokenPipe,
                    )));
                }
                self.inner.flush()
            }
            fn supports_bounded_flush(&self) -> bool {
                self.inner.supports_bounded_flush()
            }
            fn supports_nonblocking_rt(&self) -> bool {
                self.inner.supports_nonblocking_rt()
            }
            fn flush_is_noop(&self) -> bool {
                false
            }
        }

        let clock = MockClock::new();
        let inner = FakeBus::with_clock(&limits, clock.clone());
        inner.with(|s| {
            s.poll_feeds_watchdog = false;
            s.immediate_replies = true;
            s.tx_step = Duration::from_micros(50);
            s.real_flush = true;
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = rest[i];
                m.v = 0.0;
                m.enabled = i != 3;
                if i == 3 {
                    m.state_override = Some(0xD);
                }
            }
            s.push_all_feedback();
        });
        let wrapped = ClearFaultFlushFailBus {
            inner: inner.clone(),
            fail_next_clear_fault_flush: Arc::new(AtomicBool::new(false)),
            armed_flush_err: Arc::new(AtomicBool::new(false)),
        };
        let mut drv = build_standalone_mit_recovery_driver(wrapped.clone(), &rest);
        let started_at = clock.now();
        for rel in 1..=3 {
            clock.advance(Duration::from_millis(25));
            drv.begin_control_release(rel);
            inner.with(|s| s.push_all_feedback());
            drv.read_joint_states().unwrap();
        }

        clock.advance(Duration::from_millis(25));
        inner.clear_log();
        wrapped.fail_next_clear_fault_flush.store(true, Ordering::SeqCst);

        drv.begin_control_release(4);
        inner.with(|s| s.push_all_feedback());
        drv.read_joint_states().unwrap();
        let dl4 = clock.now() + Duration::from_millis(500);
        let mut gate = || Ok(());
        drv.set_tx_release_deadline(Some(dl4));
        let r4 = drv.mit_expiry_recover_budgeted(dl4, started_at, &mut gate).unwrap();
        drv.set_tx_release_deadline(None);
        assert!(!r4, "failed ClearFault flush must pause recovery (Ok(false))");
        let clears_after_fail = inner.with(|s| {
            s.tx_log.iter().filter(|f| f.data == damiao::CMD_CLEAR_FAULTS).count()
        });
        assert_eq!(clears_after_fail, 1, "first ClearFault attempt must be logged");

        // Releases 5..=10: bus recovers -> `mit_expiry_cleared_mask` was NOT set so ClearFault IS
        // re-sent, then Disable + Barrier + 0xCC complete across releases under max_frames = 16.
        let mut finished = false;
        for rel in 5..=10 {
            clock.advance(Duration::from_millis(20));
            drv.begin_control_release(rel);
            inner.with(|s| s.push_all_feedback());
            drv.read_joint_states().unwrap();
            let dl = clock.now() + Duration::from_millis(500);
            drv.set_tx_release_deadline(Some(dl));
            let r = drv.mit_expiry_recover_budgeted(dl, started_at, &mut gate).unwrap();
            drv.set_tx_release_deadline(None);
            if r {
                finished = true;
                break;
            }
        }
        assert!(finished, "recovery must succeed once ClearFault flush recovers");
        let clears_total = inner.with(|s| {
            s.tx_log.iter().filter(|f| f.data == damiao::CMD_CLEAR_FAULTS).count()
        });
        assert_eq!(
            clears_total, 2,
            "ClearFault MUST be re-sent after flush failure (mit_expiry_cleared_mask not set)"
        );
        assert!(inner.with(|s| s.motors.iter().all(|m| !m.enabled && m.state_override.is_none())));
    }

    // (c) R1 Blocker 2 + R2 Blocker 1: Delayed-wire transport (`flush_is_noop() == true`)
    //     separating host `transmit()` enqueue from actual wire delivery to motors.
    //     Verifies:
    //     1. Delayed `state == 0` feedback after host `tx_at` while `CMD_DISABLE` + barrier sit
    //        in `pending_wire` MUST NOT commit.
    //     2. Older pre-TX matching `REG_READ(RID_CTRL_MODE)` reply (timestamp strictly before
    //        `b_tx_at`) injected while `CMD_DISABLE` is still queued MUST be rejected as
    //        `Ambiguous` by `harvest_mit_expiry_barrier_acks_into` and MUST NOT commit.
    //     3. Selective delivery of `CMD_CLEAR_FAULTS` + `CMD_DISABLE` without barrier MUST NOT commit.
    //     4. Delivering barrier `REG_READ(RID_CTRL_MODE)` while `0xCC` stays queued in
    //        `pending_wire` MUST NOT commit even if `state == 0` feedback advances past
    //        `barrier_ack_seq` BEFORE the explicit post-barrier `0xCC` reaches wire and replies.
    {
        #[derive(Clone)]
        struct DelayedWireBus {
            inner: FakeBus,
            pending_wire: Arc<Mutex<VecDeque<CanFrame>>>,
            hold_wire: Arc<AtomicBool>,
        }

        impl DelayedWireBus {
            fn deliver_only_matching(&mut self, mut pred: impl FnMut(&CanFrame) -> bool) -> usize {
                let mut q = self.pending_wire.lock().unwrap();
                let mut matched = Vec::new();
                let mut kept = VecDeque::with_capacity(q.len());
                while let Some(f) = q.pop_front() {
                    if pred(&f) {
                        matched.push(f);
                    } else {
                        kept.push_back(f);
                    }
                }
                *q = kept;
                drop(q);
                let n = matched.len();
                for f in matched {
                    self.inner.transmit(&f).unwrap();
                }
                n
            }

            fn deliver_wire(&mut self) -> usize {
                self.deliver_only_matching(|_| true)
            }
        }

        impl CanTransport for DelayedWireBus {
            fn clock(&self) -> Option<Arc<dyn DriverClock>> {
                self.inner.clock()
            }
            fn rx_clock_now(&self) -> Option<CanRxTime> {
                self.inner.rx_clock_now()
            }
            fn rx_clock_sample(&self) -> Option<RxClockSample> {
                self.inner.rx_clock_sample()
            }
            fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
                if self.hold_wire.load(Ordering::Acquire) {
                    self.pending_wire.lock().unwrap().push_back(*frame);
                    Ok(())
                } else {
                    self.inner.transmit(frame)
                }
            }
            fn receive(&mut self) -> Result<CanFrame, TransportError> {
                self.inner.receive()
            }
            fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
                self.inner.receive_timestamped()
            }
            fn receive_with_rx_time(&mut self) -> Result<(CanFrame, Option<CanRxTime>), TransportError> {
                self.inner.receive_with_rx_time()
            }
            fn flush(&mut self) -> Result<(), TransportError> {
                Ok(())
            }
            fn supports_bounded_flush(&self) -> bool {
                self.inner.supports_bounded_flush()
            }
            fn supports_nonblocking_rt(&self) -> bool {
                self.inner.supports_nonblocking_rt()
            }
            fn flush_is_noop(&self) -> bool {
                true
            }
        }

        let clock = MockClock::new();
        let inner = FakeBus::with_clock(&limits, clock.clone());
        inner.with(|s| {
            s.poll_feeds_watchdog = false;
            s.immediate_replies = true;
            s.tx_step = Duration::from_micros(50);
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = rest[i];
                m.v = 0.0;
                m.enabled = i != 3;
                if i == 3 {
                    m.state_override = Some(0xD);
                }
            }
            s.push_all_feedback();
        });
        let pending_wire = Arc::new(Mutex::new(VecDeque::new()));
        let hold_wire = Arc::new(AtomicBool::new(false));
        let mut bus = DelayedWireBus {
            inner: inner.clone(),
            pending_wire: Arc::clone(&pending_wire),
            hold_wire: Arc::clone(&hold_wire),
        };
        let mut drv = build_standalone_mit_recovery_driver(bus.clone(), &rest);
        let started_at = clock.now();
        for rel in 1..=3 {
            clock.advance(Duration::from_millis(25));
            drv.begin_control_release(rel);
            inner.with(|s| s.push_all_feedback());
            drv.read_joint_states().unwrap();
        }

        // Switch to delayed-wire mode: host transmit() enqueues into `pending_wire` only.
        hold_wire.store(true, Ordering::Release);
        clock.advance(Duration::from_millis(25));
        // Timestamp strictly BEFORE release 4's `b_tx_at`.
        let pre_b_tx_ts = clock.now();
        clock.advance(Duration::from_millis(1));
        drv.begin_control_release(4);
        inner.with(|s| s.push_all_feedback());
        drv.read_joint_states().unwrap();
        let dl4 = clock.now() + Duration::from_millis(500);
        let mut gate = || Ok(());
        drv.set_tx_release_deadline(Some(dl4));
        let r4 = drv.mit_expiry_recover_budgeted(dl4, started_at, &mut gate).unwrap();
        drv.set_tx_release_deadline(None);
        assert!(!r4, "recovery must not commit while CMD_DISABLE + barrier sit in pending_wire");
        assert_eq!(
            pending_wire.lock().unwrap().iter().filter(|f| f.data == damiao::CMD_DISABLE).count(),
            7,
            "7 CMD_DISABLE frames must be queued in pending_wire"
        );
        assert_eq!(
            pending_wire
                .lock()
                .unwrap()
                .iter()
                .filter(|f| f.id == 0x7FF && f.data[2] == damiao::REG_READ && f.data[3] == damiao::RID_CTRL_MODE)
                .count(),
            7,
            "7 REG_READ(RID_CTRL_MODE) barrier frames must be queued in pending_wire"
        );
        assert!(
            inner.with(|s| s.motors.iter().enumerate().all(|(i, m)| i == 3 || m.enabled)),
            "motors must still be enabled on wire before deliver_only_matching"
        );

        // Advance clock by 2 ms (strictly AFTER host `tx_at`!) and inject delayed `state == 0`
        // feedback for all 7 motors while motors are still enabled in `pending_wire`.
        clock.advance(Duration::from_millis(2));
        let delayed_ts = clock.now();
        inner.with(|s| {
            let mut frames = Vec::with_capacity(s.motors.len());
            for m in s.motors.iter_mut() {
                let prev_en = m.enabled;
                let prev_ovr = m.state_override;
                m.enabled = false;
                m.state_override = None;
                frames.push(m.feedback_frame());
                m.enabled = prev_en;
                m.state_override = prev_ovr;
            }
            for fb in frames {
                s.push_rx_at(fb, Some(delayed_ts));
            }
        });
        drv.begin_control_release(5);
        drv.read_joint_states().unwrap();
        let dl5 = clock.now() + Duration::from_millis(500);
        drv.set_tx_release_deadline(Some(dl5));
        let r5 = drv.mit_expiry_recover_budgeted(dl5, started_at, &mut gate).unwrap();
        drv.set_tx_release_deadline(None);
        assert!(
            !r5,
            "delayed state==0 feedback after tx_at MUST NOT commit before CMD_DISABLE + barrier reach wire"
        );
        assert!(inner.with(|s| s.rx.is_empty()), "all 7 delayed state==0 frames must be drained");

        // R2 Blocker 1a: Inject older pre-TX matching REG_READ(RID_CTRL_MODE) replies
        // (timestamp `pre_b_tx_ts < b_tx_at`) while `CMD_DISABLE` is still queued ->
        // `harvest_mit_expiry_barrier_acks_into` rejects as Ambiguous and MUST NOT commit!
        clock.advance(Duration::from_millis(2));
        let post_pre_reg_fb_ts = clock.now();
        inner.with(|s| {
            let reg_reps: Vec<CanFrame> = s
                .motors
                .iter()
                .map(|m| {
                    let [lo, hi] = m.id.to_le_bytes();
                    let val = m.ctrl_mode().to_le_bytes();
                    CanFrame::new_standard(
                        m.reply_id,
                        &[lo, hi, damiao::REG_READ, damiao::RID_CTRL_MODE, val[0], val[1], val[2], val[3]],
                    )
                    .unwrap()
                })
                .collect();
            for reg_rep in reg_reps {
                s.push_rx_at(reg_rep, Some(pre_b_tx_ts));
            }
            let mut fb_frames = Vec::with_capacity(s.motors.len());
            for m in s.motors.iter_mut() {
                let prev_en = m.enabled;
                let prev_ovr = m.state_override;
                m.enabled = false;
                m.state_override = None;
                fb_frames.push(m.feedback_frame());
                m.enabled = prev_en;
                m.state_override = prev_ovr;
            }
            for fb in fb_frames {
                s.push_rx_at(fb, Some(post_pre_reg_fb_ts));
            }
        });
        drv.begin_control_release(6);
        drv.read_joint_states().unwrap();
        let dl6 = clock.now() + Duration::from_millis(500);
        drv.set_tx_release_deadline(Some(dl6));
        let r6 = drv.mit_expiry_recover_budgeted(dl6, started_at, &mut gate).unwrap();
        drv.set_tx_release_deadline(None);
        assert!(
            !r6,
            "pre-TX RID_CTRL_MODE reply (rx_at < b_tx_at) must be rejected as Ambiguous and NOT commit"
        );
        // Remove the superseded Release 4 CMD_DISABLE (7) and REG_READ(RID_CTRL_MODE) (7)
        // frames from the front of `pending_wire` (keeping ClearFault for J4), leaving
        // Release 6's fresh 7 CMD_DISABLE + 7 REG_READ barrier frames on the wire.
        {
            let mut q = pending_wire.lock().unwrap();
            let mut drop_dis = 7usize;
            let mut drop_reg = 7usize;
            let mut kept = VecDeque::with_capacity(q.len());
            while let Some(f) = q.pop_front() {
                if f.data == damiao::CMD_DISABLE && drop_dis > 0 {
                    drop_dis -= 1;
                } else if f.id == 0x7FF
                    && f.data[2] == damiao::REG_READ
                    && f.data[3] == damiao::RID_CTRL_MODE
                    && drop_reg > 0
                {
                    drop_reg -= 1;
                } else {
                    kept.push_back(f);
                }
            }
            *q = kept;
        }
        // Deliver ONLY ClearFault + CMD_DISABLE onto wire while holding barriers in pending_wire.
        let delivered_dis = bus.deliver_only_matching(|f| {
            f.data == damiao::CMD_CLEAR_FAULTS || f.data == damiao::CMD_DISABLE
        });
        assert_eq!(delivered_dis, 8, "1 ClearFault + 7 CMD_DISABLE must be delivered onto wire");
        clock.advance(Duration::from_millis(2));
        drv.begin_control_release(7);
        drv.read_joint_states().unwrap();
        let dl7 = clock.now() + Duration::from_millis(500);
        drv.set_tx_release_deadline(Some(dl7));
        let r7 = drv.mit_expiry_recover_budgeted(dl7, started_at, &mut gate).unwrap();
        drv.set_tx_release_deadline(None);
        assert!(
            !r7,
            "recovery must not commit while barrier REG_READ(RID_CTRL_MODE) stays queued in pending_wire"
        );

        // Deliver ONLY REG_READ(RID_CTRL_MODE) barrier frames -> 0x33 replies arrive, AND
        // push `state == 0` feedback AFTER the 0x33 replies (so `last_feedback_seq` advances past
        // `barrier_ack_seq` BEFORE 0xCC is sent!). Release 8 still MUST enqueue 0xCC refreshes
        // (held in pending_wire) and return Ok(false)!
        let delivered_bar = bus.deliver_only_matching(|f| {
            f.id == 0x7FF && f.data[2] == damiao::REG_READ && f.data[3] == damiao::RID_CTRL_MODE
        });
        assert!(delivered_bar >= 7, "barrier REG_READ(RID_CTRL_MODE) must be delivered onto wire");
        clock.advance(Duration::from_millis(1));
        let pre_cc_fb_ts = clock.now();
        inner.with(|s| {
            for i in 0..s.motors.len() {
                let fb = s.motors[i].feedback_frame();
                s.push_rx_at(fb, Some(pre_cc_fb_ts));
            }
        });
        clock.advance(Duration::from_millis(1));
        drv.begin_control_release(8);
        drv.read_joint_states().unwrap();
        let dl8 = clock.now() + Duration::from_millis(500);
        drv.set_tx_release_deadline(Some(dl8));
        let r8 = drv.mit_expiry_recover_budgeted(dl8, started_at, &mut gate).unwrap();
        drv.set_tx_release_deadline(None);
        assert!(
            !r8,
            "recovery must not commit when state==0 advanced past barrier_ack_seq while 0xCC sits in pending_wire"
        );
        assert_eq!(
            pending_wire
                .lock()
                .unwrap()
                .iter()
                .filter(|f| f.id == 0x7FF && f.data[2] == 0xCC)
                .count(),
            7,
            "7 0xCC post-barrier refreshes must be queued in pending_wire"
        );

        // R2 Blocker 1b: Even if a delayed pre-0xCC `state == 0` feedback arrives after
        // `ref_tx_seq` (with `rx_at == pre_cc_fb_ts < ref_tx_at`) while `0xCC` stays queued
        // in `pending_wire`, recovery MUST NOT commit!
        clock.advance(Duration::from_millis(2));
        inner.with(|s| {
            for i in 0..s.motors.len() {
                let fb = s.motors[i].feedback_frame();
                s.push_rx_at(fb, Some(pre_cc_fb_ts));
            }
        });
        drv.begin_control_release(9);
        drv.read_joint_states().unwrap();
        let dl9 = clock.now() + Duration::from_millis(500);
        drv.set_tx_release_deadline(Some(dl9));
        let r9 = drv.mit_expiry_recover_budgeted(dl9, started_at, &mut gate).unwrap();
        drv.set_tx_release_deadline(None);
        assert!(
            !r9,
            "delayed pre-0xCC state==0 feedback arriving before post-barrier 0xCC delivery MUST NOT commit"
        );
        assert_eq!(
            pending_wire
                .lock()
                .unwrap()
                .iter()
                .filter(|f| f.id == 0x7FF && f.data[2] == 0xCC)
                .count(),
            7,
            "0xCC probes must still be pending on wire"
        );

        // Now deliver Round 1 0xCC refreshes onto wire -> Round 1 confirms and queues Round 2 0xCC
        // (must NOT commit after only 1 round!).
        assert_eq!(bus.deliver_wire(), 7);
        clock.advance(Duration::from_millis(2));
        drv.begin_control_release(10);
        drv.read_joint_states().unwrap();
        let dl10 = clock.now() + Duration::from_millis(500);
        drv.set_tx_release_deadline(Some(dl10));
        let r10 = drv.mit_expiry_recover_budgeted(dl10, started_at, &mut gate).unwrap();
        drv.set_tx_release_deadline(None);
        assert!(
            !r10,
            "recovery must NOT commit after only Round 1 0xCC while Round 2 0xCC stays queued in pending_wire"
        );
        assert_eq!(
            pending_wire
                .lock()
                .unwrap()
                .iter()
                .filter(|f| f.id == 0x7FF && f.data[2] == 0xCC)
                .count(),
            7,
            "Round 2 0xCC probes must be queued strictly after Round 1 replies arrive"
        );

        // Now deliver Round 2 0xCC refreshes onto wire -> second consecutive state==0 confirms -> Ok(true)!
        assert_eq!(bus.deliver_wire(), 7);
        clock.advance(Duration::from_millis(2));
        drv.begin_control_release(11);
        drv.read_joint_states().unwrap();
        let dl11 = clock.now() + Duration::from_millis(500);
        drv.set_tx_release_deadline(Some(dl11));
        let r11 = drv.mit_expiry_recover_budgeted(dl11, started_at, &mut gate).unwrap();
        drv.set_tx_release_deadline(None);
        assert!(r11, "recovery must commit once both sequential post-barrier 0xCC rounds confirm state==0");
        assert!(inner.with(|s| s.motors.iter().all(|m| !m.enabled && m.state_override.is_none())));
    }

    // (d) R2 Major 2: Dropped barrier REG_READ(RID_CTRL_MODE) reply & dropped 0xCC status refresh
    //     reply are retried after 50 ms (`REGISTER_REPLY_TIMEOUT`) and complete cleanly.
    {
        #[derive(Clone)]
        struct DropOnceReplyBus {
            inner: FakeBus,
            drop_reg_read_motor_once: Arc<Mutex<Option<u16>>>,
            drop_cc_motor_once: Arc<Mutex<Option<u16>>>,
        }

        impl CanTransport for DropOnceReplyBus {
            fn clock(&self) -> Option<Arc<dyn DriverClock>> {
                self.inner.clock()
            }
            fn rx_clock_now(&self) -> Option<CanRxTime> {
                self.inner.rx_clock_now()
            }
            fn rx_clock_sample(&self) -> Option<RxClockSample> {
                self.inner.rx_clock_sample()
            }
            fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
                if frame.id == 0x7FF {
                    let mid = u16::from_le_bytes([frame.data[0], frame.data[1]]);
                    let drop_reg = frame.data[2] == damiao::REG_READ
                        && frame.data[3] == damiao::RID_CTRL_MODE
                        && self
                            .drop_reg_read_motor_once
                            .lock()
                            .unwrap()
                            .take_if(|id| *id == mid)
                            .is_some();
                    let drop_cc = frame.data[2] == 0xCC
                        && self
                            .drop_cc_motor_once
                            .lock()
                            .unwrap()
                            .take_if(|id| *id == mid)
                            .is_some();
                    if drop_reg || drop_cc {
                        self.inner.with(|s| {
                            let t = s.now();
                            s.tx_log.push(*frame);
                            s.tx_times.push(t);
                        });
                        return Ok(());
                    }
                }
                self.inner.transmit(frame)
            }
            fn receive(&mut self) -> Result<CanFrame, TransportError> {
                self.inner.receive()
            }
            fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
                self.inner.receive_timestamped()
            }
            fn receive_with_rx_time(&mut self) -> Result<(CanFrame, Option<CanRxTime>), TransportError> {
                self.inner.receive_with_rx_time()
            }
            fn flush(&mut self) -> Result<(), TransportError> {
                self.inner.flush()
            }
            fn supports_bounded_flush(&self) -> bool {
                self.inner.supports_bounded_flush()
            }
            fn supports_nonblocking_rt(&self) -> bool {
                self.inner.supports_nonblocking_rt()
            }
            fn flush_is_noop(&self) -> bool {
                self.inner.flush_is_noop()
            }
        }

        // Subcase (d1): Drop 1 REG_READ(RID_CTRL_MODE) barrier reply for J4 (motor_id=4) once.
        {
            let clock = MockClock::new();
            let inner = FakeBus::with_clock(&limits, clock.clone());
            inner.with(|s| {
                s.poll_feeds_watchdog = false;
                s.immediate_replies = true;
                s.tx_step = Duration::from_micros(50);
                for (i, m) in s.motors.iter_mut().enumerate() {
                    m.q = rest[i];
                    m.v = 0.0;
                    m.enabled = i != 3;
                    if i == 3 {
                        m.state_override = Some(0xD);
                    }
                }
                s.push_all_feedback();
            });
            let drop_reg = Arc::new(Mutex::new(None));
            let bus = DropOnceReplyBus {
                inner: inner.clone(),
                drop_reg_read_motor_once: Arc::clone(&drop_reg),
                drop_cc_motor_once: Arc::new(Mutex::new(None)),
            };
            let mut drv = build_standalone_mit_recovery_driver(bus, &rest);
            let started_at = clock.now();
            for rel in 1..=3 {
                clock.advance(Duration::from_millis(25));
                drv.begin_control_release(rel);
                inner.with(|s| s.push_all_feedback());
                drv.read_joint_states().unwrap();
            }

            inner.clear_log();
            *drop_reg.lock().unwrap() = Some(4);
            clock.advance(Duration::from_millis(25));
            drv.begin_control_release(4);
            inner.with(|s| s.push_all_feedback());
            drv.read_joint_states().unwrap();
            let dl4 = clock.now() + Duration::from_millis(500);
            let mut gate = || Ok(());
            drv.set_tx_release_deadline(Some(dl4));
            let r4 = drv.mit_expiry_recover_budgeted(dl4, started_at, &mut gate).unwrap();
            drv.set_tx_release_deadline(None);
            assert!(!r4, "dropped J4 barrier reply must prevent premature commit");

            // Advance past 50 ms (`REGISTER_REPLY_TIMEOUT`) so J4 barrier read retries.
            clock.advance(Duration::from_millis(55));
            let mut finished = false;
            for rel in 5..=8 {
                if rel > 5 {
                    clock.advance(Duration::from_millis(5));
                }
                drv.begin_control_release(rel);
                drv.read_joint_states().unwrap();
                let dl = clock.now() + Duration::from_millis(500);
                drv.set_tx_release_deadline(Some(dl));
                let r = drv.mit_expiry_recover_budgeted(dl, started_at, &mut gate).unwrap();
                drv.set_tx_release_deadline(None);
                if r {
                    finished = true;
                    break;
                }
            }
            assert!(finished, "J4 barrier read retry after 50 ms timeout must complete recovery");
            let j4_barrier_reads = inner.with(|s| {
                s.tx_log
                    .iter()
                    .filter(|f| {
                        f.id == 0x7FF
                            && u16::from_le_bytes([f.data[0], f.data[1]]) == 4
                            && f.data[2] == damiao::REG_READ
                            && f.data[3] == damiao::RID_CTRL_MODE
                    })
                    .count()
            });
            assert_eq!(
                j4_barrier_reads, 2,
                "J4 barrier REG_READ(RID_CTRL_MODE) must be retried exactly once after 50 ms timeout"
            );
            assert!(inner.with(|s| s.motors.iter().all(|m| !m.enabled && m.state_override.is_none())));
        }

        // Subcase (d2): Drop 1 post-barrier 0xCC refresh reply for J4 (motor_id=4) once.
        {
            let clock = MockClock::new();
            let inner = FakeBus::with_clock(&limits, clock.clone());
            inner.with(|s| {
                s.poll_feeds_watchdog = false;
                s.immediate_replies = true;
                s.tx_step = Duration::from_micros(50);
                for (i, m) in s.motors.iter_mut().enumerate() {
                    m.q = rest[i];
                    m.v = 0.0;
                    m.enabled = i != 3;
                    if i == 3 {
                        m.state_override = Some(0xD);
                    }
                }
                s.push_all_feedback();
            });
            let drop_cc = Arc::new(Mutex::new(None));
            let bus = DropOnceReplyBus {
                inner: inner.clone(),
                drop_reg_read_motor_once: Arc::new(Mutex::new(None)),
                drop_cc_motor_once: Arc::clone(&drop_cc),
            };
            let mut drv = build_standalone_mit_recovery_driver(bus, &rest);
            let started_at = clock.now();
            for rel in 1..=3 {
                clock.advance(Duration::from_millis(10));
                drv.begin_control_release(rel);
                inner.with(|s| s.push_all_feedback());
                drv.read_joint_states().unwrap();
            }

            inner.clear_log();
            *drop_cc.lock().unwrap() = Some(4);
            clock.advance(Duration::from_millis(10));
            drv.begin_control_release(4);
            inner.with(|s| s.push_all_feedback());
            drv.read_joint_states().unwrap();
            let dl4 = clock.now() + Duration::from_millis(500);
            let mut gate = || Ok(());
            drv.set_tx_release_deadline(Some(dl4));
            let r4 = drv.mit_expiry_recover_budgeted(dl4, started_at, &mut gate).unwrap();
            drv.set_tx_release_deadline(None);
            assert!(!r4, "release 4 runs ClearFault + Disable + Barrier (15 frames), pending 0xCC");

            clock.advance(Duration::from_millis(2));
            drv.begin_control_release(5);
            drv.read_joint_states().unwrap();
            let dl5 = clock.now() + Duration::from_millis(500);
            drv.set_tx_release_deadline(Some(dl5));
            let r5 = drv.mit_expiry_recover_budgeted(dl5, started_at, &mut gate).unwrap();
            drv.set_tx_release_deadline(None);
            assert!(!r5, "dropped J4 0xCC refresh reply must prevent commit on release 5");

            // Advance past 50 ms (`REGISTER_REPLY_TIMEOUT`) without injecting unsolicited J4 feedback,
            // so ONLY J4's retried 0xCC refresh can satisfy post-refresh causality. Because strict
            // `p2_tx_at > p1_ack_at` defers Round 2 when J4 is the sole Round 1 axis at rx_step == 0,
            // allow release 6 (Round 1 retry) + release 7 (Round 2 strictly after Round 1 reply).
            clock.advance(Duration::from_millis(55));
            let mut finished_d2 = false;
            for rel in 6..=7 {
                if rel > 6 {
                    clock.advance(Duration::from_millis(1));
                }
                drv.begin_control_release(rel);
                drv.read_joint_states().unwrap();
                let dl = clock.now() + Duration::from_millis(500);
                drv.set_tx_release_deadline(Some(dl));
                let r = drv.mit_expiry_recover_budgeted(dl, started_at, &mut gate).unwrap();
                drv.set_tx_release_deadline(None);
                if r {
                    finished_d2 = true;
                    break;
                }
            }
            assert!(finished_d2, "J4 0xCC retry after 50 ms timeout must complete recovery");
            let j4_cc_count = inner.with(|s| {
                s.tx_log
                    .iter()
                    .filter(|f| {
                        f.id == 0x7FF
                            && u16::from_le_bytes([f.data[0], f.data[1]]) == 4
                            && f.data[2] == 0xCC
                    })
                    .count()
            });
            assert_eq!(
                j4_cc_count, 3,
                "J4 post-barrier 0xCC must retry Round 1 once after 50 ms timeout and then complete Round 2 (3 total)"
            );
            assert!(inner.with(|s| s.motors.iter().all(|m| !m.enabled && m.state_override.is_none())));
        }
    }
}

// ============================================================================
// 21. Plan Review Major 1 & Major 3: Dual-owner MIT recovery authority
//     revocation (ordinary & applied E-stop latch) + NEW E-stop mid-recovery
//     abort on Executor
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_mit_recovery_authority_revocation_and_new_estop_abort() {
    // Subcase C: NEW E-stop mid-recovery on Executor under an applied E-stop latch ->
    // `new_estop_since(recovery_baseline)` fires at the pre-TX gate and aborts cleanly!
    {
        let cfg = mit_config_j4();
        let rest = rest_pose_of(&cfg);
        let qual = valid_mit_qualification_j4(&cfg);

        // Hook arms on the 2nd CMD_DISABLE attempt to J1 (so release 1 already captured
        // `recovery_baseline` and paused mid-flight!), triggers `sup.trigger_e_stop()`, and
        // spins briefly until `test_requested_estop_count()` advances so `pre_tx_gate` sees
        // `owner.new_estop_since(recovery_baseline) == true` at the exact CAN TX gate!
        let fire_estop = Arc::new(AtomicBool::new(false));
        let estop_fired = Arc::new(AtomicBool::new(false));
        let j1_dis_attempts = Arc::new(AtomicUsize::new(0));
        let sup_slot: Arc<std::sync::OnceLock<Arc<ArmSupervisor>>> = Arc::new(std::sync::OnceLock::new());
        let trigger_tx_slot: Arc<std::sync::Mutex<Option<tokio::sync::mpsc::UnboundedSender<()>>>> =
            Arc::new(std::sync::Mutex::new(None));

        let fire_cb = Arc::clone(&fire_estop);
        let fired_cb = Arc::clone(&estop_fired);
        let j1_cnt_cb = Arc::clone(&j1_dis_attempts);
        let sup_hook_cb = Arc::clone(&sup_slot);
        let tx_hook_cb = Arc::clone(&trigger_tx_slot);
        let hook: Arc<dyn Fn(&CanFrame) + Send + Sync> = Arc::new(move |frame: &CanFrame| {
            if !fire_cb.load(Ordering::Acquire) {
                return;
            }
            if frame.id == 1 && frame.data == damiao::CMD_DISABLE {
                let n = j1_cnt_cb.fetch_add(1, Ordering::SeqCst) + 1;
                if n == 2 && !fired_cb.swap(true, Ordering::SeqCst) {
                    if let Some(sup) = sup_hook_cb.get() {
                        let base_cnt = sup.test_requested_estop_count();
                        if let Some(tx) = tx_hook_cb.lock().unwrap().as_ref() {
                            let _ = tx.send(());
                        }
                        let dl = Instant::now() + Duration::from_millis(100);
                        while sup.test_requested_estop_count() == base_cnt && Instant::now() < dl {
                            std::hint::spin_loop();
                        }
                    }
                }
            }
        });

        let rig = LiveRig::start_with_hook(
            OwnerKind::Executor,
            cfg,
            rest,
            OperatorAuth::disabled(),
            Some(qual),
            false,
            Some(hook),
        )
        .await;
        let _ = sup_slot.set(Arc::clone(&rig.sup));
        let (trig_tx, mut trig_rx) = tokio::sync::mpsc::unbounded_channel::<()>();
        *trigger_tx_slot.lock().unwrap() = Some(trig_tx);
        let sup_trig = Arc::clone(&rig.sup);
        let trig_task = tokio::spawn(async move {
            if trig_rx.recv().await.is_some() {
                let _ = sup_trig.trigger_e_stop().await;
            }
        });

        let mut ws = ws_connect(&rig.ws_url).await;
        let granted = ws_send_wait_types(
            &mut ws,
            json!({"type": "acquire_lease", "client_id": "mit-new-estop", "seq": 1}),
            &["lease_granted"],
        )
        .await;
        let gen = granted["generation"].as_u64().unwrap();
        let ep0 = rig.sup.get_observation().await.epoch_id;
        let arm_ack = ws_rpc(&mut ws, 1, gen, ep0, "/api/v1/torque", json!({"enable": true})).await;
        assert_eq!(arm_ack["type"], "command_ack", "{arm_ack}");

        let dl_mit = Instant::now() + Duration::from_secs(3);
        while rig.bus.with(|s| s.motors[3].ctrl_mode()) != damiao::CTRL_MODE_MIT {
            assert!(Instant::now() < dl_mit);
            tokio::time::sleep(Duration::from_millis(15)).await;
        }
        tokio::time::sleep(Duration::from_millis(80)).await;

        induce_j4_mit_expiry_fault(&rig, OwnerKind::Executor).await;
        let es_msg = ws_send_wait_types(&mut ws, json!({"type": "e_stop", "seq": 2}), &["epoch_sync"]).await;
        let es_ep = es_msg["epoch_id"].as_u64().unwrap();
        let dl_es = Instant::now() + Duration::from_secs(2);
        while rig.sup.get_observation().await.epoch_id < es_ep {
            assert!(Instant::now() < dl_es);
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        tokio::time::sleep(Duration::from_millis(300)).await;
        for _ in 0..5 {
            rig.bus.with(|s| s.push_all_feedback());
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        let applied_estop_epoch = rig.sup.get_observation().await.epoch_id;

        // Pause recovery mid-flight across releases by dropping J1's first disable frame,
        // and arm `fire_estop` so the 2nd J1 disable attempt triggers a NEW E-stop right at
        // the pre-TX gate (`new_estop_since(recovery_baseline)`).
        rig.bus.with(|s| {
            s.motors[0].drop_torque_frames = 1;
        });
        fire_estop.store(true, Ordering::Release);

        let keep_fresh = Arc::new(AtomicBool::new(true));
        let kf_flag = Arc::clone(&keep_fresh);
        let kf_bus = rig.bus.clone();
        let feeder = tokio::spawn(async move {
            while kf_flag.load(Ordering::Acquire) {
                kf_bus.with(|s| {
                    for (i, m) in s.motors.iter_mut().enumerate() {
                        m.q = rest[i];
                        m.v = 0.0;
                    }
                    s.push_all_feedback();
                });
                tokio::time::sleep(Duration::from_millis(25)).await;
            }
        });

        let aborted = ws_rpc(
            &mut ws,
            2,
            gen,
            applied_estop_epoch,
            "/api/v1/safety/clear_fault",
            json!({"mit_recovery": "torque_off_all"}),
        )
        .await;
        keep_fresh.store(false, Ordering::Release);
        let _ = feeder.await;
        let _ = trig_task.await;

        assert!(
            estop_fired.load(Ordering::Acquire),
            "pre-TX gate hook must have triggered new E-stop mid-recovery"
        );
        assert_eq!(
            aborted["type"], "command_reject",
            "new E-stop mid-recovery must abort cleanly: {aborted}"
        );
        let code = aborted["code"].as_str().unwrap_or("");
        assert!(
            matches!(code, "ESTOP_ACTIVE" | "EPOCH_MISMATCH"),
            "expected ESTOP_ACTIVE or EPOCH_MISMATCH on mid-recovery E-stop abort, got {aborted}"
        );
        let obs_aborted = rig.sup.get_observation().await;
        assert!(
            matches!(obs_aborted.safety_state, SafetyState::Fault(_)),
            "new E-stop mid-recovery must abort before clearing MIT_LOST fault, got {:?}",
            obs_aborted.safety_state
        );
        assert_eq!(
            obs_aborted.gravity_comp.blocker.as_ref().map(|b| b.code.as_str()),
            Some("ESTOP_ACTIVE"),
            "new E-stop mid-recovery must latch ESTOP_ACTIVE blocker: {obs_aborted:?}"
        );
        assert!(
            obs_aborted.epoch_id > applied_estop_epoch,
            "new E-stop mid-recovery must advance epoch past {applied_estop_epoch} (got {})",
            obs_aborted.epoch_id
        );
        // J1 was NOT confirmed disabled because the 2nd disable aborted at pre_tx_gate,
        // and a subsequent clear_fault at the new EStop epoch now completes recovery!
        assert!(
            rig.bus.with(|s| s.motors[0].enabled),
            "J1 disable TX must have been aborted by pre_tx_gate new_estop_since"
        );
        for _ in 0..4 {
            rig.bus.with(|s| s.push_all_feedback());
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        let clr_new1 = ws_rpc(
            &mut ws,
            3,
            gen,
            obs_aborted.epoch_id,
            "/api/v1/safety/clear_fault",
            json!({"mit_recovery": "torque_off_all"}),
        )
        .await;
        assert_eq!(clr_new1["code"], "ESTOP_ACTIVE", "{clr_new1}");
        for _ in 0..4 {
            rig.bus.with(|s| s.push_all_feedback());
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        let clr_new2 = ws_rpc(
            &mut ws,
            4,
            gen,
            rig.sup.get_observation().await.epoch_id,
            "/api/v1/safety/clear_fault",
            json!({}),
        )
        .await;
        assert_eq!(clr_new2["type"], "command_ack", "{clr_new2}");
        assert_eq!(rig.sup.get_observation().await.safety_state, SafetyState::Idle);
        rig.assert_rt_clean("Executor MIT recovery mid-flight new E-stop abort");
        rig.shutdown().await;
    }

    // Subcase A1 (R1 Major 1): Mid-TX per-frame `pre_tx_gate` revocation on BOTH Legacy and Executor.
    // Revoke operator token on the 1st CMD_DISABLE frame so the 2nd CMD_DISABLE aborts at `pre_tx_gate`,
    // proving per-frame pre-TX authority gating aborts remaining CAN TX identically on both owners.
    for owner in [OwnerKind::Executor, OwnerKind::Legacy] {
        let cfg = mit_config_j4();
        let rest = rest_pose_of(&cfg);
        let qual = valid_mit_qualification_j4(&cfg);
        let auth = OperatorAuth::enforced("123456", 1800);
        let rig = LiveRig::start(
            owner,
            cfg,
            rest,
            auth.clone(),
            Some(qual),
            false,
        )
        .await;

        let tok = unlock_operator_token(&rig.base_url, "123456").await;
        let mut ws = ws_connect(&rig.ws_url).await;
        let granted = ws_send_wait_types(
            &mut ws,
            json!({"type": "acquire_lease", "client_id": "mit-midtx-rev", "seq": 1}),
            &["lease_granted"],
        )
        .await;
        let gen = granted["generation"].as_u64().unwrap();
        let auth_ack = ws_send_wait_types(
            &mut ws,
            json!({"type": "auth", "seq": 2, "token": tok}),
            &["auth_ack"],
        )
        .await;
        assert_eq!(auth_ack["role"], "operator");
        let ep0 = rig.sup.get_observation().await.epoch_id;

        let arm_ack = ws_rpc(&mut ws, 1, gen, ep0, "/api/v1/torque", json!({"enable": true})).await;
        assert_eq!(arm_ack["type"], "command_ack", "{owner:?}: {arm_ack}");
        let dl_mit = Instant::now() + Duration::from_secs(3);
        while rig.bus.with(|s| s.motors[3].ctrl_mode()) != damiao::CTRL_MODE_MIT {
            assert!(Instant::now() < dl_mit);
            tokio::time::sleep(Duration::from_millis(15)).await;
        }
        tokio::time::sleep(Duration::from_millis(80)).await;

        induce_j4_mit_expiry_fault(&rig, owner).await;
        tokio::time::sleep(Duration::from_millis(300)).await;
        for _ in 0..5 {
            rig.bus.with(|s| s.push_all_feedback());
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        let rec_epoch = rig.sup.get_observation().await.epoch_id;
        rig.clear_bus_log();

        let dis_cnt = Arc::new(AtomicUsize::new(0));
        let dis_cb = Arc::clone(&dis_cnt);
        let grant_live = auth.operator_grant(Some(&tok));
        let (rev_tx, rev_rx) = mpsc::sync_channel::<()>(1);
        let auth_task = auth.clone();
        let tok_task = tok.clone();
        let rev_jh = thread::spawn(move || {
            if rev_rx.recv().is_ok() {
                auth_task.lock(Some(&tok_task));
            }
        });
        rig.bus.with(|s| {
            s.on_disable = Some(Arc::new(move || {
                if dis_cb.fetch_add(1, Ordering::SeqCst) + 1 == 1 {
                    let _ = rev_tx.send(());
                    let dl = Instant::now() + Duration::from_millis(200);
                    while grant_live.is_valid_at(Instant::now(), true) && Instant::now() < dl {
                        std::hint::spin_loop();
                    }
                }
            }));
        });

        let rej = ws_rpc(
            &mut ws,
            2,
            gen,
            rec_epoch,
            "/api/v1/safety/clear_fault",
            json!({"mit_recovery": "torque_off_all"}),
        )
        .await;
        rig.bus.with(|s| s.on_disable = None);
        let _ = rev_jh.join();
        assert!(!grant_live.is_valid_at(Instant::now(), true));
        assert_eq!(rej["type"], "command_reject", "{owner:?}: {rej}");
        assert_eq!(rej["status"], 403, "{owner:?}: {rej}");
        assert_eq!(rej["code"], "OPERATOR_REQUIRED", "{owner:?}: {rej}");

        let (disable_count, barrier_non_j4_after_dis, first_dis_idx, cc_after_dis) = rig.bus.with(|s| {
            let dis = s.tx_log.iter().filter(|f| f.data == damiao::CMD_DISABLE).count();
            let first_dis = s.tx_log.iter().position(|f| f.data == damiao::CMD_DISABLE);
            let (bar_post, cc_post) = first_dis.map_or((0, 0), |idx| {
                let tail = &s.tx_log[idx + 1..];
                // Background L2 `enforce_unverified_mode_readback_parts` only probes unverified J4 (MIT),
                // whereas recovery barrier probes disabled axes starting from J1 (`data[0] == 1`).
                let bar = tail
                    .iter()
                    .filter(|f| {
                        f.id == 0x7FF
                            && f.data[0] != 4
                            && f.data[2] == damiao::REG_READ
                            && f.data[3] == damiao::RID_CTRL_MODE
                    })
                    .count();
                let cc = tail
                    .iter()
                    .filter(|f| f.id == 0x7FF && f.data[2] == 0xCC)
                    .count();
                (bar, cc)
            });
            (dis, bar_post, first_dis, cc_post)
        });
        assert!(first_dis_idx.is_some(), "{owner:?}: expected 1st CMD_DISABLE before revocation");
        assert!(
            disable_count < 7,
            "{owner:?}: mid-TX pre_tx_gate revocation must abort remaining CMD_DISABLE frames (got {disable_count})"
        );
        assert_eq!(
            barrier_non_j4_after_dis, 0,
            "{owner:?}: zero post-disable REG_READ(RID_CTRL_MODE) recovery barrier frames allowed after revocation"
        );
        assert_eq!(
            cc_after_dis, 0,
            "{owner:?}: zero post-disable 0xCC frames allowed after revocation"
        );

        let obs_after = rig.sup.get_observation().await;
        assert!(
            matches!(obs_after.safety_state, SafetyState::Fault(_)),
            "{owner:?}: safety fault must stay latched after mid-TX revocation, got {:?}",
            obs_after.safety_state
        );

        // Re-unlock operator and verify subsequent clear_fault completes cleanly to Idle.
        let tok2 = unlock_operator_token(&rig.base_url, "123456").await;
        let auth_ack2 = ws_send_wait_types(
            &mut ws,
            json!({"type": "auth", "seq": 3, "token": tok2}),
            &["auth_ack"],
        )
        .await;
        assert_eq!(auth_ack2["role"], "operator");
        for _ in 0..4 {
            rig.bus.with(|s| s.push_all_feedback());
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        let clr_ok = ws_rpc(
            &mut ws,
            3,
            gen,
            rig.sup.get_observation().await.epoch_id,
            "/api/v1/safety/clear_fault",
            json!({"mit_recovery": "torque_off_all"}),
        )
        .await;
        assert_eq!(clr_ok["type"], "command_ack", "{owner:?}: {clr_ok}");
        assert_eq!(rig.sup.get_observation().await.safety_state, SafetyState::Idle);
        rig.assert_rt_clean(&format!("MIT recovery mid-TX revocation ({owner:?})"));
        rig.shutdown().await;
    }

    // Subcase A2 (ordinary completion branch) & Subcase B (applied E-stop latch branch):
    // Post-confirmation revocation AFTER the 7th post-barrier `0xCC` frame passes `pre_tx_gate`
    // (before final logical clear) across BOTH OwnerKind::Executor and OwnerKind::Legacy.
    for applied_estop in [false, true] {
        for owner in [OwnerKind::Executor, OwnerKind::Legacy] {
            let cfg = mit_config_j4();
            let rest = rest_pose_of(&cfg);
            let qual = valid_mit_qualification_j4(&cfg);
            let auth = OperatorAuth::enforced("123456", 1800);

            let watch_post_confirm = Arc::new(AtomicBool::new(false));
            let arm_revoke_after_tx = Arc::new(AtomicBool::new(false));
            let barrier_cnt = Arc::new(AtomicUsize::new(0));
            let cc_cnt = Arc::new(AtomicUsize::new(0));
            let watch_cb = Arc::clone(&watch_post_confirm);
            let arm_cb = Arc::clone(&arm_revoke_after_tx);
            let bar_cb = Arc::clone(&barrier_cnt);
            let cc_cb = Arc::clone(&cc_cnt);
            let hook: Arc<dyn Fn(&CanFrame) + Send + Sync> = Arc::new(move |frame: &CanFrame| {
                if !watch_cb.load(Ordering::Acquire) {
                    return;
                }
                if frame.id == 0x7FF
                    && frame.data[2] == damiao::REG_READ
                    && frame.data[3] == damiao::RID_CTRL_MODE
                {
                    bar_cb.fetch_add(1, Ordering::SeqCst);
                } else if bar_cb.load(Ordering::SeqCst) >= 7
                    && frame.id == 0x7FF
                    && frame.data[2] == 0xCC
                    && cc_cb.fetch_add(1, Ordering::SeqCst) + 1 == 14
                {
                    arm_cb.store(true, Ordering::SeqCst);
                }
            });

            let rig = LiveRig::start_with_hook(
                owner,
                cfg,
                rest,
                auth.clone(),
                Some(qual),
                false,
                Some(hook),
            )
            .await;

            let tok = unlock_operator_token(&rig.base_url, "123456").await;
            let mut ws = ws_connect(&rig.ws_url).await;
            let granted = ws_send_wait_types(
                &mut ws,
                json!({"type": "acquire_lease", "client_id": "mit-auth-rev", "seq": 1}),
                &["lease_granted"],
            )
            .await;
            let gen = granted["generation"].as_u64().unwrap();
            let auth_ack = ws_send_wait_types(
                &mut ws,
                json!({"type": "auth", "seq": 2, "token": tok}),
                &["auth_ack"],
            )
            .await;
            assert_eq!(auth_ack["role"], "operator");
            let ep0 = rig.sup.get_observation().await.epoch_id;

            let arm_ack = ws_rpc(&mut ws, 1, gen, ep0, "/api/v1/torque", json!({"enable": true})).await;
            assert_eq!(arm_ack["type"], "command_ack", "{owner:?}: {arm_ack}");
            let dl_mit = Instant::now() + Duration::from_secs(3);
            while rig.bus.with(|s| s.motors[3].ctrl_mode()) != damiao::CTRL_MODE_MIT {
                assert!(Instant::now() < dl_mit);
                tokio::time::sleep(Duration::from_millis(15)).await;
            }
            tokio::time::sleep(Duration::from_millis(80)).await;

            induce_j4_mit_expiry_fault(&rig, owner).await;
            if applied_estop {
                let es_msg = ws_send_wait_types(&mut ws, json!({"type": "e_stop", "seq": 3}), &["epoch_sync"]).await;
                let es_ep = es_msg["epoch_id"].as_u64().unwrap();
                let dl_es = Instant::now() + Duration::from_secs(2);
                while rig.sup.get_observation().await.epoch_id < es_ep {
                    assert!(Instant::now() < dl_es);
                    tokio::time::sleep(Duration::from_millis(10)).await;
                }
            }
            // Wait past MIT quiet period and feed fresh rest-pose feedback.
            tokio::time::sleep(Duration::from_millis(300)).await;
            for _ in 0..5 {
                rig.bus.with(|s| s.push_all_feedback());
                tokio::time::sleep(Duration::from_millis(20)).await;
            }
            let obs_pre_rec = rig.sup.get_observation().await;
            let rec_epoch = obs_pre_rec.epoch_id;

            // Revoke operator token strictly AFTER the 7th post-barrier `0xCC` frame passes
            // `pre_tx_gate` (all 7 motors disabled and confirmed, BEFORE final logical clear).
            let grant_live = auth.operator_grant(Some(&tok));
            let (rev_tx, rev_rx) = mpsc::sync_channel::<()>(1);
            let auth_task = auth.clone();
            let tok_task = tok.clone();
            let rev_jh = thread::spawn(move || {
                if rev_rx.recv().is_ok() {
                    auth_task.lock(Some(&tok_task));
                }
            });
            let arm_tx_cb = Arc::clone(&arm_revoke_after_tx);
            let grant_tx_cb = grant_live;
            let prev_on_tx = rig.bus.with(|s| s.on_tx.take());
            rig.bus.with(|s| {
                s.on_tx = Some(Arc::new(move || {
                    if let Some(ref prev) = prev_on_tx {
                        prev();
                    }
                    if arm_tx_cb.swap(false, Ordering::SeqCst) {
                        let _ = rev_tx.send(());
                        let dl = Instant::now() + Duration::from_millis(200);
                        while grant_tx_cb.is_valid_at(Instant::now(), true) && Instant::now() < dl {
                            std::hint::spin_loop();
                        }
                    }
                }));
            });
            watch_post_confirm.store(true, Ordering::Release);

            let rej = ws_rpc(
                &mut ws,
                2,
                gen,
                rec_epoch,
                "/api/v1/safety/clear_fault",
                json!({"mit_recovery": "torque_off_all"}),
            )
            .await;
            watch_post_confirm.store(false, Ordering::Release);
            let _ = rev_jh.join();
            assert!(
                !grant_live.is_valid_at(Instant::now(), true),
                "{owner:?} applied_estop={applied_estop}: operator grant must have been revoked on 7th post-barrier 0xCC (motor_errors={:?})",
                obs_pre_rec.motor_errors
            );
            assert_eq!(
                rej["type"], "command_reject",
                "{owner:?} applied_estop={applied_estop}: revoked operator before commit must reject: {rej}"
            );
            assert_eq!(rej["status"], 403, "{owner:?}: {rej}");
            assert_eq!(rej["code"], "OPERATOR_REQUIRED", "{owner:?}: {rej}");

            // (i) Physical disable sync MUST be preserved!
            let obs_after_rev = rig.sup.get_observation().await;
            assert!(
                !obs_after_rev.is_torqued,
                "{owner:?} applied_estop={applied_estop}: physical disable sync must set !is_torqued"
            );
            if owner == OwnerKind::Legacy {
                assert!(
                    !rig.sup.inner().read().await.is_torqued,
                    "Legacy inner.is_torqued must be false after physical disable"
                );
            }
            assert!(
                rig.bus.with(|s| s.motors.iter().all(|m| !m.enabled)),
                "{owner:?} applied_estop={applied_estop}: all motors must be disabled on bus"
            );

            // (ii) Logical fault / E-stop latch MUST remain latched!
            if applied_estop {
                assert!(
                    matches!(obs_after_rev.safety_state, SafetyState::EStop | SafetyState::Fault(_)),
                    "{owner:?}: EStop/fault latch must stay latched on operator revocation, got {:?}",
                    obs_after_rev.safety_state
                );
                assert_eq!(
                    obs_after_rev.gravity_comp.blocker.as_ref().map(|b| b.code.as_str()),
                    Some("ESTOP_ACTIVE"),
                    "{owner:?}: applied E-stop latch must remain active on operator revocation: {obs_after_rev:?}"
                );
            } else {
                assert!(
                    matches!(obs_after_rev.safety_state, SafetyState::Fault(_)),
                    "{owner:?}: safety fault must remain latched on operator revocation, got {:?}",
                    obs_after_rev.safety_state
                );
            }

            // Re-unlock operator and verify subsequent clear_fault recovers cleanly to Idle.
            let tok2 = unlock_operator_token(&rig.base_url, "123456").await;
            let auth_ack2 = ws_send_wait_types(
                &mut ws,
                json!({"type": "auth", "seq": 4, "token": tok2}),
                &["auth_ack"],
            )
            .await;
            assert_eq!(auth_ack2["role"], "operator");
            for _ in 0..4 {
                rig.bus.with(|s| s.push_all_feedback());
                tokio::time::sleep(Duration::from_millis(20)).await;
            }
            let clr1 = ws_rpc(
                &mut ws,
                3,
                gen,
                rig.sup.get_observation().await.epoch_id,
                "/api/v1/safety/clear_fault",
                json!({"mit_recovery": "torque_off_all"}),
            )
            .await;
            if applied_estop {
                let settle_ep = if owner == OwnerKind::Legacy {
                    assert_eq!(clr1["type"], "command_ack", "{owner:?}: {clr1}");
                    assert_eq!(clr1["safety_state"], "EStop", "{owner:?}: {clr1}");
                    clr1["epoch_id"].as_u64().unwrap()
                } else {
                    assert_eq!(clr1["code"], "ESTOP_ACTIVE", "{owner:?}: {clr1}");
                    0
                };
                let dl_settle = Instant::now() + Duration::from_secs(2);
                let obs_settled = loop {
                    let o = rig.sup.get_observation().await;
                    if o.safety_state == SafetyState::EStop && o.epoch_id >= settle_ep {
                        break o;
                    }
                    assert!(
                        Instant::now() < dl_settle,
                        "{owner:?}: first clear must settle into EStop, got {o:?}"
                    );
                    tokio::time::sleep(Duration::from_millis(10)).await;
                };
                for _ in 0..4 {
                    rig.bus.with(|s| s.push_all_feedback());
                    tokio::time::sleep(Duration::from_millis(20)).await;
                }
                let clr2 = ws_rpc(
                    &mut ws,
                    4,
                    gen,
                    obs_settled.epoch_id,
                    "/api/v1/safety/clear_fault",
                    json!({}),
                )
                .await;
                assert_eq!(clr2["type"], "command_ack", "{owner:?}: {clr2}");
            } else {
                assert_eq!(clr1["type"], "command_ack", "{owner:?}: {clr1}");
            }
            assert_eq!(rig.sup.get_observation().await.safety_state, SafetyState::Idle);
            rig.assert_rt_clean(&format!("MIT recovery operator revocation ({owner:?}, estop={applied_estop})"));
            rig.shutdown().await;
        }
    }

    // R2 Major 1: A driver implementing only legacy `mit_expiry_recover()` without
    // overriding `mit_expiry_recover_with_gate()` MUST fail closed (never fall back
    // to ungated recovery).
    {
        struct LegacyOnlyMitRecoverDriver {
            legacy_called: Arc<AtomicBool>,
        }
        impl ArmDriver for LegacyOnlyMitRecoverDriver {
            fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
                Ok(Vec::new())
            }
            fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> {
                Ok(())
            }
            fn set_torque(&mut self, _enabled: bool) -> Result<(), ArmDriverError> {
                Ok(())
            }
            fn mit_expiry_recover(&mut self) -> Result<(), String> {
                self.legacy_called.store(true, Ordering::SeqCst);
                Ok(())
            }
        }
        let legacy_called = Arc::new(AtomicBool::new(false));
        let mut drv = LegacyOnlyMitRecoverDriver {
            legacy_called: Arc::clone(&legacy_called),
        };
        let mut gate = || Ok(());
        let res = drv.mit_expiry_recover_with_gate(&mut gate);
        assert!(
            res.is_err(),
            "mit_expiry_recover_with_gate default must fail closed when not overridden, got {res:?}"
        );
        assert!(
            !legacy_called.load(Ordering::SeqCst),
            "default mit_expiry_recover_with_gate must NOT call ungated mit_expiry_recover()"
        );
    }
}

// ============================================================================
// 22. Plan Review Major 2: Combined RX + TX budget exhaustion across
//     Step 2 `read_joint_states()` + `commissioning_drain_rx_step` + `mit_expiry_recover_budgeted`
// ============================================================================

#[test]
fn mit_expiry_recover_combined_rx_and_tx_budget_exhaustion() {
    let cfg = ArmConfig::rebot_dm();
    let rest = rest_pose_of(&cfg);
    let limits = b601_limits();
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&limits, clock.clone());
    bus.with(|s| {
        s.poll_feeds_watchdog = false;
        s.immediate_replies = true;
        s.tx_step = Duration::from_micros(50);
        for (i, m) in s.motors.iter_mut().enumerate() {
            m.q = rest[i];
            m.v = 0.0;
            m.enabled = !matches!(i, 2..=4);
            if matches!(i, 2..=4) {
                // 3 faulted motors -> 3 ClearFault + 7 Disable = 10 TX frames > max_frames(4)
                m.state_override = Some(0xD);
            }
        }
        s.push_all_feedback();
    });

    let mut drv = build_standalone_mit_recovery_driver(bus.clone(), &rest);
    let mut invalid_tb = TxBudget::default();
    // 3 faulted motors + 4 energized (3 arm + J7) requires max_frames >= 5; verify admission refusal at 4.
    invalid_tb.max_frames = 4;
    drv.set_tx_budget(invalid_tb);

    let started_at = clock.now();
    for rel in 1..=3 {
        clock.advance(Duration::from_millis(25));
        drv.begin_control_release(rel);
        bus.with(|s| s.push_all_feedback());
        drv.read_joint_states().unwrap();
    }
    {
        let dl_refuse = clock.now() + Duration::from_millis(500);
        let mut gate_refuse = || Ok(());
        let refused = drv.mit_expiry_recover_budgeted(dl_refuse, started_at, &mut gate_refuse);
        assert!(
            matches!(
                refused,
                Err(ArmDriverError::WatchdogBudgetInsufficient {
                    max_frames: 4,
                    required: 5
                })
            ),
            "expected WatchdogBudgetInsufficient(4 < 5) at MIT recovery admission, got {refused:?}"
        );
    }
    // Use the exact minimum valid cap (`max_frames = 5` for 4 energized axes + 1):
    // 3 ClearFault + 7 MitRecoveryDisable + 7 Barrier(0x33) + 7 Refresh(0xCC) = 24 TX frames
    // (> max_frames=5), proving EVERY TX frame respects the hard TxBudget.max_frames = 5 ceiling.
    let mut tight_tb = TxBudget::default();
    tight_tb.max_frames = 5;
    drv.set_tx_budget(tight_tb);

    // Release 4: Step 2 `read_joint_states` consumes 30 RX frames, `commissioning_drain_rx_step`
    // consumes 20 RX frames (50 total, 14 remaining), then 30 more RX frames are queued so
    // `mit_expiry_recover_budgeted` exhausts the shared 64-frame RX_FRAME_BUDGET cleanly!
    clock.advance(Duration::from_millis(25));
    bus.clear_log();
    drv.begin_control_release(4);
    let dl4 = clock.now() + Duration::from_millis(500);
    drv.set_tx_release_deadline(Some(dl4));
    bus.with(|s| {
        s.rx.clear();
        for k in 0..30 {
            s.push_feedback(k % 7);
        }
    });
    drv.read_joint_states().unwrap();
    bus.with(|s| {
        for k in 0..20 {
            s.push_feedback(k % 7);
        }
    });
    let (drained_mid, is_empty_mid) = drv.commissioning_drain_rx_step(64, dl4).unwrap();
    assert_eq!(drained_mid, 20);
    assert!(is_empty_mid);

    bus.with(|s| {
        for k in 0..30 {
            s.push_feedback(k % 7);
        }
    });
    let mut gate = || Ok(());
    let r4 = drv.mit_expiry_recover_budgeted(dl4, started_at, &mut gate).unwrap();
    drv.set_tx_release_deadline(None);
    assert!(!r4, "release 4 must pause on shared 64-frame RX budget exhaustion");
    let rx_rem4 = bus.with(|s| s.rx.len());
    let total_rx4 = 30 + 20 + (30 - rx_rem4);
    assert!(
        total_rx4 <= usize::from(armd::rt_core::ops::RX_FRAME_BUDGET),
        "release 4 RX frames ({total_rx4}) exceeded RX_FRAME_BUDGET"
    );
    assert_eq!(rx_rem4, 16, "expected 14 recovery RX frames consumed (16 left)");
    let tx_rel4 = bus.with(|s| s.tx_log.len());
    assert!(tx_rel4 <= 5, "release 4 TX ({tx_rel4}) exceeded TxBudget.max_frames=5");

    // Releases 5..=25: Step 2 drains remaining 16 frames, then recovery advances under
    // `TxBudget { max_frames: 5 }` across multiple releases with EVERY TX frame counted.
    let mut finished = false;
    let mut paused_on_tx_budget = false;
    for rel in 5..=25 {
        clock.advance(Duration::from_millis(10));
        let tx_before = bus.with(|s| s.tx_log.len());
        drv.begin_control_release(rel);
        let dl = clock.now() + Duration::from_millis(500);
        drv.set_tx_release_deadline(Some(dl));
        drv.read_joint_states().unwrap();
        let step = drv.mit_expiry_recover_budgeted(dl, started_at, &mut gate).unwrap();
        drv.set_tx_release_deadline(None);
        let total_tx_this_rel = bus.with(|s| s.tx_log[tx_before..].len());
        assert!(
            total_tx_this_rel <= 5,
            "release {rel} sent {total_tx_this_rel} TX frames, exceeding TxBudget.max_frames=5"
        );
        if !step && total_tx_this_rel == 5 {
            paused_on_tx_budget = true;
        }
        if step {
            finished = true;
            break;
        }
    }
    assert!(
        paused_on_tx_budget,
        "expected recovery to pause at least once after hitting TxBudget.max_frames=5"
    );
    assert!(finished, "recovery must finish across subsequent TX-budgeted releases");

    let (clear_count, disable_count, all_disabled_clean) = bus.with(|s| {
        let clr = s.tx_log.iter().filter(|f| f.data == damiao::CMD_CLEAR_FAULTS).count();
        let dis = s.tx_log.iter().filter(|f| f.data == damiao::CMD_DISABLE).count();
        let ok = s.motors.iter().all(|m| !m.enabled && m.state_override.is_none());
        (clr, dis, ok)
    });
    assert_eq!(
        clear_count, 3,
        "ClearFault must be sent exactly once per faulted motor (3) across TX-budget pauses, got {clear_count}"
    );
    assert_eq!(
        disable_count, 7,
        "CMD_DISABLE must be sent once per motor (7) across TX-budget pauses, got {disable_count}"
    );
    assert!(all_disabled_clean, "all 7 motors must end disabled with no fault state");
}

// ============================================================================
// 23. Major 6 Executor half: Sync-MIT-only driver on OWNER_EXECUTOR fails
//     closed with 422 UNSUPPORTED, advertises gravity_comp unsupported,
//     issues 0 `set_joint_modes` calls, and keeps 0 RT allocs/locks.
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_sync_mit_only_driver_fails_closed_unsupported() {
    let mode_history = Arc::new(Mutex::new(Vec::new()));
    let drv = SyncMitSimLikeDriver {
        pose: RAISED_POSE,
        enabled: false,
        modes: vec![JointMode::Position; 7],
        mode_history: mode_history.clone(),
        fail_next_switch: Arc::new(AtomicBool::new(false)),
    };
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;
    sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();

    let mut ecfg = ExecutorConfig::for_tests(PERIOD_50HZ);
    ecfg.alloc_timing = true;
    ecfg.panic_policy = PanicPolicy::StopExecutor;
    let exec = sup.start_executor(ecfg).await.unwrap();
    let dl = Instant::now() + Duration::from_secs(2);
    while sup.hub().latest().is_none() {
        assert!(Instant::now() < dl);
        tokio::time::sleep(Duration::from_millis(5)).await;
    }

    assert!(
        !sup.get_observation().await.gravity_comp.supported,
        "Executor obs.gravity_comp.supported must be false for sync-MIT-only driver"
    );
    assert!(
        !sup.get_schema().await.capabilities.gravity_comp,
        "Executor schema.capabilities.gravity_comp must be false for sync-MIT-only driver"
    );
    assert!(
        !sup.published_config().await.unwrap().gravity_comp_supported,
        "Executor published_config.gravity_comp_supported must be false for sync-MIT-only driver"
    );

    let stamp = capture_config_admission(&sup, 0, 0, None);
    let err = gravity_comp_command_with_auth(
        &sup,
        stamp,
        None,
        OperatorRequirement::Required(OperatorGrant::unenforced()),
        ClientId(0),
        true,
        Some(0.5),
        None,
    )
    .await
    .expect_err("sync-MIT-only driver on Executor must fail closed with 422 UNSUPPORTED");
    match err {
        armd::ArmError::Rejected { status, code, .. } => {
            assert_eq!(status, 422, "expected 422, got {status}/{code}");
            assert_eq!(code, "UNSUPPORTED", "expected UNSUPPORTED, got {code}");
        }
        other => panic!("expected 422 UNSUPPORTED, got {other:?}"),
    }
    assert!(
        mode_history.lock().unwrap().is_empty(),
        "0 set_joint_modes calls allowed on Executor for sync-MIT-only driver"
    );
    assert_eq!(exec.stats().total_allocs(), AllocCounts::ZERO);
    assert_eq!(sup.inner().control_thread_attempts(), 0);
    tokio::task::spawn_blocking(move || exec.stop_and_join()).await.unwrap();
}

// ============================================================================
// 24. Plan Review Major 3: Whole-release and wake zero-alloc proof under
//     OWNER_EXECUTOR with occupied >256-slot workspace buffers, stalled consumers,
//     gravity-comp lifecycle, tracking rejection, operator revocation, MIT
//     expiry recovery, and terminal retirement.
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_whole_release_and_wake_zero_alloc_lock_stalled_consumers_gravity_mit() {
    let mut cfg = mit_config_j4();
    let rest = rest_pose_of(&cfg);
    let qual = valid_mit_qualification_j4(&cfg);
    // Add PositionLead on J1 + >256-slot workspace (box + 30 end points + 2 keepout boxes)
    if let Some(jt) = cfg.joint_tracking.as_mut() {
        jt.joints[0].strategy = TrackingStrategy::PositionLead;
        jt.joints[0].lag_s = 0.08;
        jt.joints[0].max_lead_rad = 0.03;
    }
    cfg.limits[0].max_position = RAISED_POSE[0] + 0.14;
    let mut ws_cfg = ArmConfig::rebot_dm().workspace.expect("rebot_dm has workspace");
    ws_cfg.floor_z_m = -1.0;
    ws_cfg.box_min_m = [-1.5, -1.5, -1.0];
    ws_cfg.box_max_m = [1.5, 1.5, 1.5];
    ws_cfg.max_reach_m = Some(2.0);
    ws_cfg.check_points_end_m = (0..30).map(|i| [0.0001 * i as f32, 0.0, 0.0]).collect();
    ws_cfg.keepout_boxes = vec![
        KeepoutBox {
            name: "stalled_obs_a".into(),
            min_m: [0.80, 0.80, 0.80],
            max_m: [0.95, 0.95, 0.95],
        },
        KeepoutBox {
            name: "stalled_obs_b".into(),
            min_m: [-0.95, 0.80, 0.80],
            max_m: [-0.80, 0.95, 0.95],
        },
    ];
    let eval = armd::chain::WorkspaceEvaluator::new_with_tcp(
        &cfg.arm_model,
        &ws_cfg,
        &cfg.camera_mount,
        cfg.tcp_offset,
    );
    assert!(
        eval.total_slots > armd::kinematics::MAX_STACK_WORKSPACE_SLOTS,
        "workspace must exceed 256 slots, got {}",
        eval.total_slots
    );
    cfg.workspace = Some(ws_cfg);

    let auth = OperatorAuth::enforced("123456", 1800);
    let rig = LiveRig::start(
        OwnerKind::Executor,
        cfg,
        RAISED_POSE,
        auth.clone(),
        Some(qual),
        false,
    )
    .await;
    let exec = rig.exec.as_ref().unwrap().clone();

    // 5. Operator grant revocation refusal on gravity-comp entry (explicitly asserted).
    {
        let client_rev = ClientId(909);
        exec.open_connection_sync(909).unwrap().unwrap();
        let _rev_sink = exec
            .register_connection_reply_sink(
                client_rev,
                Arc::new(|_, _| DeliveryDecision::Delivered),
                Some(8),
            )
            .unwrap();
        let conn_rev = armd::rt_core::requests::lease::ConnectionHandle::try_new(909, "rev-gc").unwrap();
        let rev_gen = match rig
            .sup
            .acquire_lease_ws(conn_rev, "rev-gc", 1)
            .await
            .expect("acquire_lease_ws")
        {
            armd::rt_core::requests::lease::LeaseOutcome::Granted { generation, .. } => generation,
            other => panic!("expected Granted, got {other:?}"),
        };
        let (tok_rev, _) = auth.unlock("123456").unwrap();
        let grant_rev = auth.operator_grant(Some(&tok_rev));
        auth.lock(Some(&tok_rev));
        let adm_rev = capture_config_admission(&rig.sup, 909, rev_gen, None);
        let err_rev = gravity_comp_command_with_auth(
            &rig.sup,
            adm_rev,
            Some((armd::model::HolderLabel::new("rev-gc"), rev_gen)),
            OperatorRequirement::Required(grant_rev),
            client_rev,
            true,
            None,
            None,
        )
        .await
        .expect_err("revoked OperatorGrant must refuse gravity-comp entry");
        match err_rev {
            armd::ArmError::Rejected { status: 403, code, .. } => {
                assert_eq!(code, "OPERATOR_REQUIRED");
            }
            other => panic!("expected 403 OPERATOR_REQUIRED, got {other:?}"),
        }
        assert_ne!(rig.sup.get_observation().await.control_mode, ControlMode::GravityComp);
        let _ = rig.sup.release_lease_ws(conn_rev, rev_gen).await;
    }

    let mut ws = ws_connect(&rig.ws_url).await;
    let granted = ws_send_wait_types(
        &mut ws,
        json!({"type": "acquire_lease", "client_id": "whole-rel-gc-mit", "seq": 1}),
        &["lease_granted"],
    )
    .await;
    let gen = granted["generation"].as_u64().unwrap();

    // Unlock operator, authenticate WS, and arm (J4 enters MIT).
    let tok = unlock_operator_token(&rig.base_url, "123456").await;
    let auth_ack = ws_send_wait_types(&mut ws, json!({"type": "auth", "seq": 2, "token": tok}), &["auth_ack"]).await;
    assert_eq!(auth_ack["role"], "operator");
    let ep0 = rig.sup.get_observation().await.epoch_id;
    let arm_ack = ws_rpc(&mut ws, 1, gen, ep0, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(arm_ack["type"], "command_ack", "{arm_ack}");
    let dl_mit = Instant::now() + Duration::from_secs(3);
    while rig.bus.with(|s| s.motors[3].ctrl_mode()) != damiao::CTRL_MODE_MIT {
        assert!(Instant::now() < dl_mit);
        tokio::time::sleep(Duration::from_millis(15)).await;
    }
    tokio::time::sleep(Duration::from_millis(60)).await;

    // 4. Tracking rejection / fallback braking with >256 workspace boxes while consumer is stalled.
    //    Runs with J4 MIT active right after initial arm; after clearing JointLimit + re-arming,
    //    we exercise gravity-comp entry/active/exit, park to rest, and MIT expiry recovery.
    let mut target_rej = RAISED_POSE.to_vec();
    target_rej[0] = RAISED_POSE[0] + 0.138;
    let jog_ack = ws_rpc(
        &mut ws,
        2,
        gen,
        rig.sup.get_observation().await.epoch_id,
        "/api/v1/motion/jog",
        json!({"target_positions": target_rej, "duration_secs": 0.25, "speed_scale": 1.0}),
    )
    .await;
    assert_eq!(jog_ack["type"], "command_ack", "{jog_ack}");
    let seq_fb0 = rig.sup.hub().published_seq();
    let (rel_fb, jh_fb) = pause_hub(&rig.sup);
    let dl_stalled = Instant::now() + Duration::from_secs(2);
    while rig.sup.hub().published_seq() < seq_fb0 + 16 {
        assert!(Instant::now() < dl_stalled);
        tokio::time::sleep(Duration::from_millis(15)).await;
    }
    rel_fb.send(()).unwrap();
    jh_fb.join().unwrap();

    let dl_fault = Instant::now() + Duration::from_secs(2);
    let obs_rej = loop {
        let o = rig.sup.get_observation().await;
        if matches!(o.safety_state, SafetyState::Fault(_)) {
            break o;
        }
        assert!(Instant::now() < dl_fault, "TrackingReject did not fire: {o:?}");
        tokio::time::sleep(Duration::from_millis(15)).await;
    };
    assert!(
        obs_rej.fault.as_ref().is_some_and(|f| {
            (f.code == FaultCode::JointLimit
                && f.message.as_str().contains("Joint tracking output rejected"))
                || (f.code == FaultCode::ServoNoBrake
                    && f.message.as_str().contains("Joint tracking fallback cannot brake"))
        }),
        "expected TrackingReject / fallback braking fault detail, got {:?}",
        obs_rej.fault
    );

    // Clear JointLimit fault and re-arm (activates MIT on J4).
    let clr_jl = ws_rpc(
        &mut ws,
        3,
        gen,
        obs_rej.epoch_id,
        "/api/v1/safety/clear_fault",
        json!({}),
    )
    .await;
    assert_eq!(clr_jl["type"], "command_ack", "{clr_jl}");
    let rearm_gc = ws_rpc(
        &mut ws,
        4,
        gen,
        rig.sup.get_observation().await.epoch_id,
        "/api/v1/torque",
        json!({"enable": true}),
    )
    .await;
    assert_eq!(rearm_gc["type"], "command_ack", "{rearm_gc}");
    let dl_mit_gc = Instant::now() + Duration::from_secs(3);
    while rig.bus.with(|s| s.motors[3].ctrl_mode()) != damiao::CTRL_MODE_MIT {
        assert!(Instant::now() < dl_mit_gc);
        tokio::time::sleep(Duration::from_millis(15)).await;
    }
    tokio::time::sleep(Duration::from_millis(60)).await;

    // 1. Gravity-comp entry (hands back MIT on J4 -> ModeSwitch POS_VEL->MIT -> Active).
    let enter_ack = ws_rpc(
        &mut ws,
        5,
        gen,
        rig.sup.get_observation().await.epoch_id,
        "/api/v1/mode/gravity_comp",
        json!({"enable": true}),
    )
    .await;
    assert_eq!(enter_ack["type"], "command_ack", "{enter_ack}");
    assert_eq!(enter_ack["control_mode"], "gravity_comp");

    // 2. Active gravity-comp output ticks while `StateHub` consumer is stalled (`reader_guard()`).
    let dl_act = Instant::now() + Duration::from_secs(2);
    while rig.sup.get_observation().await.gravity_comp.phase.as_deref() != Some("active") {
        assert!(Instant::now() < dl_act);
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    let seq_gc0 = rig.sup.hub().published_seq();
    let (rel_gc, jh_gc) = pause_hub(&rig.sup);
    tokio::time::sleep(Duration::from_millis(80)).await;
    assert!(
        rig.sup.hub().published_seq() >= seq_gc0 + 3,
        "gravity_comp output releases must keep publishing while reader_guard is held"
    );
    rel_gc.send(()).unwrap();
    jh_gc.join().unwrap();
    assert_eq!(
        rig.sup.get_observation().await.gravity_comp.phase.as_deref(),
        Some("active")
    );

    // 3. Gravity-comp exit/handback (ModeSwitch MIT -> Position).
    let exit_ack = ws_rpc(
        &mut ws,
        6,
        gen,
        rig.sup.get_observation().await.epoch_id,
        "/api/v1/mode/gravity_comp",
        json!({"enable": false}),
    )
    .await;
    assert_eq!(exit_ack["type"], "command_ack", "{exit_ack}");
    let dl_pos = Instant::now() + Duration::from_secs(3);
    loop {
        let o = rig.sup.get_observation().await;
        if o.gravity_comp.phase.is_none()
            && o.control_mode == ControlMode::Position
            && o.gravity_comp.blocker.is_none()
        {
            break;
        }
        assert!(Instant::now() < dl_pos);
        tokio::time::sleep(Duration::from_millis(15)).await;
    }

    // Move cleanly to rest pose while J4 MIT is re-activated after gravity_comp exit so Step 6
    // exercises MIT expiry recovery at rest (Rule A′), then restore follow_commands = false.
    rig.bus.with(|s| {
        s.follow_commands = true;
    });
    let ep_move = rig.sup.get_observation().await.epoch_id;
    let move_rest = ws_rpc(
        &mut ws,
        7,
        gen,
        ep_move,
        "/api/v1/motion/jog",
        json!({"target_positions": rest.to_vec(), "duration_secs": 0.20, "speed_scale": 1.0}),
    )
    .await;
    assert_eq!(move_rest["type"], "command_ack", "{move_rest}");
    let move_id = move_rest["action_id"].as_u64();
    let dl_rest = Instant::now() + Duration::from_secs(3);
    loop {
        let o = rig.sup.get_observation().await;
        if o.action.as_ref().is_some_and(|a| Some(a.id) == move_id && !a.is_running()) {
            assert_eq!(o.action.as_ref().map(|a| a.state.as_str()), Some("done"), "{o:?}");
            break;
        }
        assert!(Instant::now() < dl_rest, "jog to rest stalled: {o:?}");
        tokio::time::sleep(Duration::from_millis(15)).await;
    }
    rig.bus.with(|s| {
        s.follow_commands = false;
    });
    let dl_mit2 = Instant::now() + Duration::from_secs(3);
    while rig.bus.with(|s| s.motors[3].ctrl_mode()) != damiao::CTRL_MODE_MIT {
        assert!(Instant::now() < dl_mit2);
        tokio::time::sleep(Duration::from_millis(15)).await;
    }
    tokio::time::sleep(Duration::from_millis(60)).await;

    // 6. MIT expiry recovery (at rest pose).
    induce_j4_mit_expiry_fault(&rig, OwnerKind::Executor).await;
    tokio::time::sleep(Duration::from_millis(300)).await;
    for _ in 0..5 {
        rig.bus.with(|s| {
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = rest[i];
                m.v = 0.0;
            }
            s.push_all_feedback();
        });
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    let rec_ack = ws_rpc(
        &mut ws,
        8,
        gen,
        rig.sup.get_observation().await.epoch_id,
        "/api/v1/safety/clear_fault",
        json!({"mit_recovery": "torque_off_all"}),
    )
    .await;
    assert_eq!(rec_ack["type"], "command_ack", "MIT expiry recovery failed: {rec_ack}");
    let obs_idle = rig.sup.get_observation().await;
    assert_eq!(obs_idle.safety_state, SafetyState::Idle);
    assert!(!obs_idle.is_torqued);

    // 7. Stalled consumers:
    //    (a) Stalled `StateHub::reader_guard()` holds `armd-dispatch` on `hub.latest()` while
    //        `armd-control` finishes `jog_ret` across >= 8 releases with 0 RT allocs/locks.
    //    (b) Stalled router `ConnectionReplySink` (`DeliveryDecision::Backpressured`) holds
    //        delivery while the exact `jog_stall_id` terminal ALREADY retires from `broker`
    //        (`broker.stats().retired` increments STRICTLY BEFORE dropping backpressure!),
    //        then un-stalling delivers that exact terminal (`terminals_delivered` increments).
    let rearm2 = ws_rpc(&mut ws, 9, gen, obs_idle.epoch_id, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(rearm2["type"], "command_ack", "{rearm2}");
    let dl_mit3 = Instant::now() + Duration::from_secs(3);
    while rig.bus.with(|s| s.motors[3].ctrl_mode()) != damiao::CTRL_MODE_MIT
        || rig.sup.get_observation().await.gravity_comp.blocker.is_some()
    {
        assert!(Instant::now() < dl_mit3);
        tokio::time::sleep(Duration::from_millis(15)).await;
    }

    rig.bus.with(|s| {
        s.follow_commands = true;
    });
    let _ = ws_send_wait_types(
        &mut ws,
        json!({"type": "release_lease", "client_id": "whole-rel-gc-mit", "generation": gen, "seq": 11}),
        &["lease_released"],
    )
    .await;
    {
        let client_hub = ClientId(911);
        exec.open_connection_sync(911).unwrap().unwrap();
        let hub_stall_terminals = Arc::new(Mutex::new(Vec::<(u64, armd::rt_core::broker::Outcome)>::new()));
        let hub_term_cb = Arc::clone(&hub_stall_terminals);
        let _hub_sink_guard = exec
            .register_connection_reply_sink(
                client_hub,
                Arc::new(move |_, reply| {
                    if let Reply::Terminal { action, outcome, .. } = reply {
                        hub_term_cb.lock().unwrap().push((action.0, outcome));
                    }
                    DeliveryDecision::Delivered
                }),
                Some(8),
            )
            .unwrap();
        let conn_hub =
            armd::rt_core::requests::lease::ConnectionHandle::try_new(911, "stall-hub").unwrap();
        let hub_gen = match rig
            .sup
            .acquire_lease_ws(conn_hub, "stall-hub", 1)
            .await
            .expect("acquire_lease_ws")
        {
            armd::rt_core::requests::lease::LeaseOutcome::Granted { generation, .. } => generation,
            other => panic!("expected Granted, got {other:?}"),
        };
        let retired_before_7a = {
            let mut b = exec.broker().lock().unwrap();
            b.reclaim_plans();
            b.stats().retired
        };
        let mut adm_h = rig.sup.capture_admission_stamp();
        adm_h.connection_id = 911;
        adm_h.lease_gen = hub_gen;
        let snap_h = armd::rt_core::requests::actions::ActionSnapshot::capture(
            &rig.sup,
            adm_h,
            Instant::now() + Duration::from_secs(5),
        )
        .await
        .unwrap();
        let g_hub = armd::rt_core::requests::actions::prepare_gripper(
            &snap_h,
            &json!({"command": "move", "fraction": 0.15, "speed": 1.0}),
            armd::supervisor::GripperBaselineCmd::None,
        )
        .unwrap();
        let slot_h = g_hub.to_plan_slot(None).unwrap();
        let write_h = |dst: &mut armd::rt_core::ops::PlanSlot| *dst = slot_h.clone();

        // Acquire `StateHub::reader_guard()` STRICTLY BEFORE submitting the action so admission,
        // ACK, execution, retirement, and exact terminal observation all happen strictly while
        // the StateHub stall is in effect (zero pre-stall completion race).
        let seq_ret0 = rig.sup.hub().published_seq();
        let (rel_ret, jh_ret) = pause_hub(&rig.sup);
        let (_req_h, mut ack_rx_h, guard_h) = exec
            .submit_external_action(
                client_hub,
                1,
                snap_h.owner.authority,
                Instant::now() + Duration::from_secs(5),
                armd::rt_core::requests::actions::ActionSubmit::InstallPrepared {
                    write: &write_h,
                    kind: armd::rt_core::requests::actions::PreparedActionKind::Gripper,
                    prepared_boot: exec.boot(),
                    admission_request_id: adm_h.request_id,
                    check_workspace_revision: false,
                    poses_revision: None,
                    calibration_revision: None,
                    gripper_calib_revision: g_hub.stamp.gripper_calib_revision,
                    power_budget_revision: None,
                },
                armd::rt_core::reply_router::ExternalDestinationPolicy::ConnectionOnly,
                false,
                None,
            )
            .unwrap();
        let ack_dl = Instant::now() + Duration::from_secs(2);
        let ack_h = loop {
            exec.pump_broker_replies();
            match ack_rx_h.try_recv() {
                Ok(r) => break r.unwrap(),
                Err(tokio::sync::oneshot::error::TryRecvError::Empty) => {
                    assert!(
                        Instant::now() < ack_dl,
                        "timed out waiting for StateHub-stall action ACK before terminal check"
                    );
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
                Err(tokio::sync::oneshot::error::TryRecvError::Closed) => {
                    panic!("ACK channel closed before StateHub-stall action ACK arrived");
                }
            }
        };
        let _ = guard_h.confirm_enqueued();
        let hub_action_id = match ack_h {
            armd::rt_core::broker::Reply::Ack { action: Some(aid), .. } => aid.0,
            other => panic!("expected Ack(Some(action_id)), got {other:?}"),
        };

        let dl_ret = Instant::now() + Duration::from_secs(3);
        loop {
            exec.pump_broker_replies();
            let retired_now = {
                let mut b = exec.broker().lock().unwrap();
                b.reclaim_plans();
                b.stats().retired
            };
            let exact_term_seen = hub_stall_terminals
                .lock()
                .unwrap()
                .iter()
                .any(|&(aid, out)| aid == hub_action_id && out == armd::rt_core::broker::Outcome::Completed);
            if rig.sup.hub().published_seq() >= seq_ret0 + 16
                && retired_now > retired_before_7a
                && exact_term_seen
            {
                break;
            }
            assert!(
                Instant::now() < dl_ret,
                "action {hub_action_id} did not emit its exact terminal and retire while StateHub reader_guard was held (terminals={:?}, retired={retired_now} vs {retired_before_7a})",
                hub_stall_terminals.lock().unwrap()
            );
            tokio::time::sleep(Duration::from_millis(15)).await;
        }
        assert!(
            hub_stall_terminals
                .lock()
                .unwrap()
                .contains(&(hub_action_id, armd::rt_core::broker::Outcome::Completed)),
            "exact target terminal id {hub_action_id} must be observed strictly BEFORE dropping StateHub::reader_guard()"
        );
        let retired_during_stall = {
            let mut b = exec.broker().lock().unwrap();
            b.reclaim_plans();
            b.stats().retired
        };
        assert!(
            retired_during_stall > retired_before_7a,
            "action {hub_action_id} plan must retire in broker strictly BEFORE dropping StateHub::reader_guard() ({retired_during_stall} vs {retired_before_7a})"
        );
        rel_ret.send(()).unwrap();
        jh_ret.join().unwrap();

        let dl_done = Instant::now() + Duration::from_secs(2);
        loop {
            let obs_done = rig.sup.get_observation().await;
            if obs_done
                .action
                .as_ref()
                .is_some_and(|a| a.id == hub_action_id && !a.is_running())
            {
                assert_eq!(obs_done.action.as_ref().map(|a| a.state.as_str()), Some("done"), "{obs_done:?}");
                break;
            }
            assert!(Instant::now() < dl_done, "exact action {hub_action_id} did not settle: obs={obs_done:?}");
            tokio::time::sleep(Duration::from_millis(15)).await;
        }
        let _ = rig.sup.release_lease_ws(conn_hub, hub_gen).await;
    }

    // (b) Stalled router sink (`DeliveryDecision::Backpressured`): verify exact `stall_action_id`
    //     is observed in `backpressured_ids` and retired BEFORE `stall_sink` is released, and
    //     that exact `stall_action_id` is delivered after releasing backpressure!
    {
        let client_stall = ClientId(910);
        exec.open_connection_sync(910).unwrap().unwrap();
        let stall_sink = Arc::new(AtomicBool::new(true));
        let stall_cb = Arc::clone(&stall_sink);
        let backpressured_ids = Arc::new(Mutex::new(Vec::<u64>::new()));
        let backpressured_cb = Arc::clone(&backpressured_ids);
        let delivered_ids = Arc::new(Mutex::new(Vec::<u64>::new()));
        let delivered_cb = Arc::clone(&delivered_ids);
        let _stall_guard = exec
            .register_connection_reply_sink(
                client_stall,
                Arc::new(move |_, reply| {
                    if stall_cb.load(Ordering::Acquire) {
                        if let Reply::Terminal { action, .. } = reply {
                            backpressured_cb.lock().unwrap().push(action.0);
                        }
                        return DeliveryDecision::Backpressured;
                    }
                    if let Reply::Terminal { action, .. } = reply {
                        delivered_cb.lock().unwrap().push(action.0);
                    }
                    DeliveryDecision::Delivered
                }),
                Some(8),
            )
            .unwrap();
        let conn_stall =
            armd::rt_core::requests::lease::ConnectionHandle::try_new(910, "stall-sink").unwrap();
        let stall_gen = match rig
            .sup
            .acquire_lease_ws(conn_stall, "stall-sink", 1)
            .await
            .expect("acquire_lease_ws")
        {
            armd::rt_core::requests::lease::LeaseOutcome::Granted { generation, .. } => generation,
            other => panic!("expected Granted, got {other:?}"),
        };
        let retired_before = exec.broker().lock().unwrap().stats().retired;
        let delivered_before = exec.reply_router_stats().terminals_delivered;
        let mut adm_s = rig.sup.capture_admission_stamp();
        adm_s.connection_id = 910;
        adm_s.lease_gen = stall_gen;
        let snap_s = armd::rt_core::requests::actions::ActionSnapshot::capture(
            &rig.sup,
            adm_s,
            Instant::now() + Duration::from_secs(5),
        )
        .await
        .unwrap();
        let g_prep = armd::rt_core::requests::actions::prepare_gripper(
            &snap_s,
            &json!({"command": "move", "fraction": 0.3, "speed": 1.0}),
            armd::supervisor::GripperBaselineCmd::None,
        )
        .unwrap();
        let slot_s = g_prep.to_plan_slot(None).unwrap();
        let write_s = |dst: &mut armd::rt_core::ops::PlanSlot| *dst = slot_s.clone();
        let (_req_s, mut ack_rx_s, guard_s) = exec
            .submit_external_action(
                client_stall,
                1,
                snap_s.owner.authority,
                Instant::now() + Duration::from_secs(5),
                armd::rt_core::requests::actions::ActionSubmit::InstallPrepared {
                    write: &write_s,
                    kind: armd::rt_core::requests::actions::PreparedActionKind::Gripper,
                    prepared_boot: exec.boot(),
                    admission_request_id: adm_s.request_id,
                    check_workspace_revision: false,
                    poses_revision: None,
                    calibration_revision: None,
                    gripper_calib_revision: g_prep.stamp.gripper_calib_revision,
                    power_budget_revision: None,
                },
                armd::rt_core::reply_router::ExternalDestinationPolicy::ConnectionOnly,
                false,
                None,
            )
            .unwrap();
        let ack_s = loop {
            exec.pump_broker_replies();
            if let Ok(r) = ack_rx_s.try_recv() {
                break r.unwrap();
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        };
        let _ = guard_s.confirm_enqueued();
        let stall_action_id = match ack_s {
            armd::rt_core::broker::Reply::Ack { action: Some(aid), .. } => aid.0,
            other => panic!("expected Ack(Some(action_id)), got {other:?}"),
        };
        let dl_retire = Instant::now() + Duration::from_secs(2);
        loop {
            exec.pump_broker_replies();
            let ret_now = exec.broker().lock().unwrap().stats().retired;
            let st_now = exec.reply_router_stats();
            let bp_seen = backpressured_ids.lock().unwrap().contains(&stall_action_id);
            if ret_now > retired_before && st_now.pending_external_deliveries > 0 && bp_seen {
                break;
            }
            assert!(
                Instant::now() < dl_retire,
                "action {stall_action_id} terminal did not hit Backpressured and retire while router sink stalled (retired {ret_now} vs {retired_before}, st={st_now:?})"
            );
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        assert!(
            backpressured_ids.lock().unwrap().contains(&stall_action_id),
            "exact target action_id {stall_action_id} must be observed in backpressured_ids STRICTLY BEFORE releasing backpressure"
        );
        assert!(
            delivered_ids.lock().unwrap().is_empty(),
            "terminal must NOT be delivered while router sink returns Backpressured"
        );
        assert_eq!(exec.reply_router_stats().terminals_delivered, delivered_before);

        stall_sink.store(false, Ordering::Release);
        let dl_deliv = Instant::now() + Duration::from_secs(2);
        while !delivered_ids.lock().unwrap().contains(&stall_action_id) {
            exec.pump_broker_replies();
            assert!(Instant::now() < dl_deliv, "action {stall_action_id} terminal was not delivered after un-stall");
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        assert!(exec.reply_router_stats().terminals_delivered > delivered_before);
    }

    assert_eq!(
        exec.stats().total_allocs(),
        AllocCounts::ZERO,
        "whole-release and wake across all 7 gravity/MIT/stalled-consumer phases must have 0 heap allocs/deallocs"
    );
    assert_eq!(
        rig.sup.inner().control_thread_attempts(),
        0,
        "armd-control must make 0 SupervisorLock attempts"
    );
    rig.shutdown().await;

    // 8. R1 Major 3: Gravity-comp reclaim queue backpressure (`503 RECLAIM_BACKPRESSURE`,
    //    0 `mem::forget`, 0 RT allocs/deallocs) while `offload_reclaim_tx` consumer is stalled.
    {
        struct ScriptedClock {
            base: Instant,
            state: Mutex<(u64, u64, Option<u64>)>,
            cv: Condvar,
        }
        impl ScriptedClock {
            fn new() -> Arc<Self> {
                Arc::new(Self {
                    base: Instant::now(),
                    state: Mutex::new((0, 0, None)),
                    cv: Condvar::new(),
                })
            }
            fn run_steps(&self, period_ns: u64, steps: u64) {
                let mut s = self.state.lock().unwrap();
                let target = s.1 + period_ns * steps;
                s.1 = target;
                let expect_park = target + period_ns;
                self.cv.notify_all();
                let dl = Instant::now() + Duration::from_secs(5);
                while s.2 != Some(expect_park) {
                    let rem = dl.saturating_duration_since(Instant::now());
                    assert!(!rem.is_zero(), "scripted clock park timeout");
                    let (next, _) = self.cv.wait_timeout(s, rem).unwrap();
                    s = next;
                }
            }
            fn step_if_parked(&self, period_ns: u64) {
                let mut s = self.state.lock().unwrap();
                if let Some(parked) = s.2 {
                    s.1 = parked;
                } else {
                    s.1 = s.1.saturating_add(period_ns);
                }
                self.cv.notify_all();
            }
        }
        impl ExecutorClock for ScriptedClock {
            fn now(&self) -> Instant {
                self.base + Duration::from_nanos(self.state.lock().unwrap().0)
            }
            fn now_ns(&self) -> u64 {
                self.state.lock().unwrap().0
            }
            fn sleep_until_ns(&self, target_ns: u64, stop: &AtomicBool) -> bool {
                let mut s = self.state.lock().unwrap();
                if s.0 >= target_ns {
                    self.cv.notify_all();
                    return false;
                }
                while !stop.load(Ordering::Acquire) && s.0 < target_ns && s.1 < target_ns {
                    s.2 = Some(target_ns);
                    self.cv.notify_all();
                    s = self.cv.wait(s).unwrap();
                }
                s.2 = None;
                if !stop.load(Ordering::Acquire) && s.0 < target_ns {
                    s.0 = target_ns;
                }
                self.cv.notify_all();
                true
            }
            fn sleep(&self, dur: Duration, stop: &AtomicBool) {
                if stop.load(Ordering::Acquire) {
                    return;
                }
                let mut s = self.state.lock().unwrap();
                s.0 = s.0.saturating_add(dur.as_nanos().min(u128::from(u64::MAX)) as u64);
                self.cv.notify_all();
            }
            fn wake(&self) {
                let _g = self.state.lock().unwrap();
                self.cv.notify_all();
            }
        }

        let dir = tempfile::tempdir().unwrap();
        let offload_cap = MAINT_QUEUE_CAPACITY
            + 1
            + 6
            + 12
            + ESTOP_ACK_CAPACITY
            + RECLAIM_STATE_CAPACITY
            + URGENT_STOP_CAPACITY
            + RECLAIM_TRAJ_CAPACITY
            + 4
            + 2;
        let mk_bp_job = |id: u64| PendingZeroCheckPersist {
            action_id: id,
            offset_rad: 0.01,
            base_revision: 1,
            connection_id: 0,
            lease_owner: None,
            estop_epoch: 1,
            stop_gen: 0,
            deadline: Instant::now() + Duration::from_secs(5),
            ctx: Box::new(ZeroCheckPersistContext {
                path: dir.path().join(format!("bp_{id}.json")),
                jaw_opening: None,
                stale: None,
                tool_id: "stock".into(),
                tool_config_digest: String::new(),
                active_tool_is_stock: true,
                base_gripper_policy: Some(armd::model::GripperPolicy::rebot_dm()),
                gripper_limit: None,
                operator_auth: None,
                sat_msg_buf: String::with_capacity(64),
            }),
        };

        let mut cfg_bp = ArmConfig::rebot_dm();
        cfg_bp.lease_timeout_ms = 600_000;
        if let Some(gc) = cfg_bp.gravity_comp.as_mut() {
            gc.ramp_in_s = 0.02;
            gc.exit_hold_ms = 0;
        }
        let clock_bp = ScriptedClock::new();
        let pause_persist = Arc::new(AtomicBool::new(true));
        let job_slot: Arc<Mutex<Option<(PendingZeroCheckPersist, Option<Box<dyn std::any::Any + Send>>)>>> =
            Arc::new(Mutex::new(None));
        let hooks = PersistTestHooks {
            clock: Some(clock_bp.clone()),
            pause_persist_worker: Some(pause_persist.clone()),
            inject_pending_persist_job_once: Some(job_slot.clone()),
            ..Default::default()
        };
        let sup_bp = Arc::new(ArmSupervisor::new(cfg_bp.clone()));
        sup_bp.set_lease_enabled(false).await;
        sup_bp.enable_torque(sup_bp.capture_admission_stamp()).await.unwrap();

        let period_ns = 10_000_000u64;
        let mut ecfg_bp = ExecutorConfig::for_tests(Duration::from_nanos(period_ns));
        ecfg_bp.alloc_timing = true;
        ecfg_bp.panic_policy = PanicPolicy::StopExecutor;
        let exec_bp = sup_bp.start_executor_with_hooks(ecfg_bp, hooks).await.unwrap();
        clock_bp.run_steps(period_ns, 2);

        // Stall `armd-dispatch` on `StateHub::reader_guard()` so `offload_reclaim_rx` is not drained.
        let (rel_bp, jh_bp) = pause_hub(&sup_bp);
        let dl_wait = Instant::now() + Duration::from_secs(5);
        while sup_bp.hub().latest_waiters() < 1 {
            assert!(Instant::now() < dl_wait, "dispatcher did not block on StateHub::latest()");
            sup_bp.hub().wake();
            tokio::time::sleep(Duration::from_millis(1)).await;
        }

        // Saturate `persist_job_tx` (4) + `offload_reclaim_tx` (OFFLOAD_RECLAIM_CAPACITY) completely
        // so all retired `GravityCompRun`s remain inside `SupervisorInner::retired_gravity_comp` (cap 8).
        for id in 1..=(4 + offload_cap) as u64 {
            *job_slot.lock().unwrap() = Some((mk_bp_job(id), None));
            clock_bp.run_steps(period_ns, 1);
        }

        let run_gc_cmd = |sup_arg: &Arc<ArmSupervisor>, clk_arg: &Arc<ScriptedClock>, enable: bool| {
            let sup_c = Arc::clone(sup_arg);
            let clk_c = Arc::clone(clk_arg);
            async move {
                let adm = capture_config_admission(&sup_c, 0, 0, None);
                let fut = tokio::spawn(async move {
                    gravity_comp_command_with_auth(
                        &sup_c,
                        adm,
                        None,
                        OperatorRequirement::NotRequired,
                        ClientId(0),
                        enable,
                        None,
                        None,
                    )
                    .await
                });
                while !fut.is_finished() {
                    tokio::time::sleep(Duration::from_millis(1)).await;
                    clk_c.run_steps(period_ns, 1);
                }
                fut.await.unwrap()
            }
        };

        // Run EXACTLY 8 gravity-comp enter+exit cycles (all 8 must succeed, filling `retired_gravity_comp` to 8).
        for cycle in 1..=8 {
            let rep = run_gc_cmd(&sup_bp, &clock_bp, true)
                .await
                .unwrap_or_else(|e| panic!("gravity_comp cycle {cycle} enter must succeed: {e:?}"));
            assert!(rep.active, "cycle {cycle} enter must activate gravity_comp");
            clock_bp.run_steps(period_ns, 4);
            let _ex = run_gc_cmd(&sup_bp, &clock_bp, false)
                .await
                .unwrap_or_else(|e| panic!("gravity_comp cycle {cycle} exit must succeed: {e:?}"));
            clock_bp.run_steps(period_ns, 6);
        }

        // The 9th entry MUST IMMEDIATELY reject with 503 RECLAIM_BACKPRESSURE (0 RT allocs/deallocs).
        match run_gc_cmd(&sup_bp, &clock_bp, true).await {
            Err(armd::ArmError::Rejected { status: 503, code, .. })
                if code == "RECLAIM_BACKPRESSURE" => {}
            other => panic!("9th gravity_comp entry must fail closed immediately with 503 RECLAIM_BACKPRESSURE, got {other:?}"),
        }
        assert_eq!(exec_bp.stats().total_allocs(), AllocCounts::ZERO);

        // Resume reclaim consumer & persistence worker and drain all 8 retired items LIVE BEFORE `stop_and_join()`.
        pause_persist.store(false, Ordering::Release);
        rel_bp.send(()).unwrap();
        jh_bp.join().unwrap();
        for _ in 0..16 {
            clock_bp.run_steps(period_ns, 1);
            tokio::time::sleep(Duration::from_millis(3)).await;
        }
        // Re-pause and re-saturate `offload_reclaim_tx` while live to prove all 8 slots of
        // `retired_gravity_comp` drained to (0, 0) LIVE BEFORE calling `stop_and_join()`:
        // all 8 new cycles must succeed before the next 9th rejects with 503 RECLAIM_BACKPRESSURE!
        pause_persist.store(true, Ordering::Release);
        let (rel_bp2, jh_bp2) = pause_hub(&sup_bp);
        let dl_wait2 = Instant::now() + Duration::from_secs(5);
        while sup_bp.hub().latest_waiters() < 1 {
            assert!(Instant::now() < dl_wait2, "dispatcher did not re-block on StateHub::latest()");
            sup_bp.hub().wake();
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
        for id in 1001..=(1000 + 4 + offload_cap) as u64 {
            *job_slot.lock().unwrap() = Some((mk_bp_job(id), None));
            clock_bp.run_steps(period_ns, 1);
        }
        for cycle in 1..=8 {
            let rep = run_gc_cmd(&sup_bp, &clock_bp, true)
                .await
                .unwrap_or_else(|e| panic!("post-drain cycle {cycle} enter must succeed (proving live drain to 0/8): {e:?}"));
            assert!(rep.active);
            clock_bp.run_steps(period_ns, 4);
            let _ex = run_gc_cmd(&sup_bp, &clock_bp, false).await.unwrap();
            clock_bp.run_steps(period_ns, 6);
        }
        match run_gc_cmd(&sup_bp, &clock_bp, true).await {
            Err(armd::ArmError::Rejected { status: 503, code, .. })
                if code == "RECLAIM_BACKPRESSURE" => {}
            other => panic!("9th post-drain entry must reject with 503 RECLAIM_BACKPRESSURE, got {other:?}"),
        }
        pause_persist.store(false, Ordering::Release);
        rel_bp2.send(()).unwrap();
        jh_bp2.join().unwrap();
        for _ in 0..16 {
            clock_bp.run_steps(period_ns, 1);
            tokio::time::sleep(Duration::from_millis(3)).await;
        }
        let resumed_enter = run_gc_cmd(&sup_bp, &clock_bp, true)
            .await
            .expect("gravity_comp enter must succeed after live reclaim");
        assert!(resumed_enter.active);
        clock_bp.run_steps(period_ns, 4);
        let _resumed_exit = run_gc_cmd(&sup_bp, &clock_bp, false).await.unwrap();
        clock_bp.run_steps(period_ns, 6);
        tokio::time::sleep(Duration::from_millis(15)).await;

        assert_eq!(exec_bp.stats().total_allocs(), AllocCounts::ZERO);
        assert_eq!(sup_bp.inner().control_thread_attempts(), 0);
        tokio::task::spawn_blocking(move || exec_bp.stop_and_join()).await.unwrap();

        // Full-ring shutdown non-leaking reclamation tests:
        // (1) Handback while `offload_reclaim_tx` is 100% saturated and 2 `GravityCompRun`s are in
        //     `retired_gravity_comp`: 0 RT allocs/frees, `retired_gravity_comp_queue_lens() == (2, 0)`
        //     preserved and inspectable before `ArmSupervisor` drops cleanly.
        {
            let clock_full = ScriptedClock::new();
            let pause_full = Arc::new(AtomicBool::new(true));
            let slot_full: Arc<Mutex<Option<(PendingZeroCheckPersist, Option<Box<dyn std::any::Any + Send>>)>>> =
                Arc::new(Mutex::new(None));
            let hooks_full = PersistTestHooks {
                clock: Some(clock_full.clone()),
                pause_persist_worker: Some(pause_full.clone()),
                inject_pending_persist_job_once: Some(slot_full.clone()),
                ..Default::default()
            };
            let sup_full = Arc::new(ArmSupervisor::new(cfg_bp.clone()));
            sup_full.set_lease_enabled(false).await;
            sup_full.enable_torque(sup_full.capture_admission_stamp()).await.unwrap();
            let mut ecfg_full = ExecutorConfig::for_tests(Duration::from_nanos(period_ns));
            ecfg_full.alloc_timing = true;
            ecfg_full.panic_policy = PanicPolicy::StopExecutor;
            let exec_full = sup_full.start_executor_with_hooks(ecfg_full, hooks_full).await.unwrap();
            clock_full.run_steps(period_ns, 2);

            let (rel_full, jh_full) = pause_hub(&sup_full);
            let mut jh_full = Some(jh_full);
            let dl_w = Instant::now() + Duration::from_secs(5);
            while sup_full.hub().latest_waiters() < 1 {
                assert!(Instant::now() < dl_w);
                sup_full.hub().wake();
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
            for id in 2001..=(2000 + 4 + offload_cap) as u64 {
                *slot_full.lock().unwrap() = Some((mk_bp_job(id), None));
                clock_full.run_steps(period_ns, 1);
            }
            for _ in 0..2 {
                let rep = run_gc_cmd(&sup_full, &clock_full, true).await.unwrap();
                assert!(rep.active);
                clock_full.run_steps(period_ns, 4);
                let _ex = run_gc_cmd(&sup_full, &clock_full, false).await.unwrap();
                clock_full.run_steps(period_ns, 6);
            }

            let hb_fut = tokio::spawn({
                let sup_c = Arc::clone(&sup_full);
                async move {
                    sup_c
                        .handback_to_legacy_with_live_loop(
                            Duration::from_secs(2),
                            Duration::from_millis(10),
                        )
                        .await
                }
            });
            while !hb_fut.is_finished() {
                if exec_full.is_exited() {
                    if let Some(jh) = jh_full.take() {
                        pause_full.store(false, Ordering::Release);
                        rel_full.send(()).unwrap();
                        jh.join().unwrap();
                    }
                }
                clock_full.step_if_parked(period_ns);
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
            if let Some(jh) = jh_full.take() {
                pause_full.store(false, Ordering::Release);
                rel_full.send(()).unwrap();
                jh.join().unwrap();
            }
            let (legacy_stop, legacy_jh) = hb_fut
                .await
                .unwrap()
                .expect("handback under full offload_reclaim_tx must succeed");
            assert_eq!(exec_full.stats().total_allocs(), AllocCounts::ZERO);
            assert_eq!(
                sup_full.inner().read().await.retired_gravity_comp_queue_lens(),
                (2, 0),
                "full-ring shutdown/handback must preserve retired_gravity_comp (never mem::forget)"
            );
            legacy_stop.store(true, Ordering::Release);
            let _ = tokio::task::spawn_blocking(move || legacy_jh.join()).await;
            drop(sup_full);
        }

        // (2) Direct `stop_and_join()` while `offload_reclaim_tx` is 100% full and 2 `GravityCompRun`s
        //     are in `retired_gravity_comp`: 0 RT frees during shutdown.
        {
            let clock_stop = ScriptedClock::new();
            let pause_stop = Arc::new(AtomicBool::new(true));
            let slot_stop: Arc<Mutex<Option<(PendingZeroCheckPersist, Option<Box<dyn std::any::Any + Send>>)>>> =
                Arc::new(Mutex::new(None));
            let hooks_stop = PersistTestHooks {
                clock: Some(clock_stop.clone()),
                pause_persist_worker: Some(pause_stop.clone()),
                inject_pending_persist_job_once: Some(slot_stop.clone()),
                ..Default::default()
            };
            let sup_stop = Arc::new(ArmSupervisor::new(cfg_bp));
            sup_stop.set_lease_enabled(false).await;
            sup_stop.enable_torque(sup_stop.capture_admission_stamp()).await.unwrap();
            let mut ecfg_stop = ExecutorConfig::for_tests(Duration::from_nanos(period_ns));
            ecfg_stop.alloc_timing = true;
            ecfg_stop.panic_policy = PanicPolicy::StopExecutor;
            let exec_stop = sup_stop.start_executor_with_hooks(ecfg_stop, hooks_stop).await.unwrap();
            clock_stop.run_steps(period_ns, 2);

            let (rel_stop, jh_stop) = pause_hub(&sup_stop);
            let dl_w = Instant::now() + Duration::from_secs(5);
            while sup_stop.hub().latest_waiters() < 1 {
                assert!(Instant::now() < dl_w);
                sup_stop.hub().wake();
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
            for id in 3001..=(3000 + 4 + offload_cap) as u64 {
                *slot_stop.lock().unwrap() = Some((mk_bp_job(id), None));
                clock_stop.run_steps(period_ns, 1);
            }
            for _ in 0..2 {
                let rep = run_gc_cmd(&sup_stop, &clock_stop, true).await.unwrap();
                assert!(rep.active);
                clock_stop.run_steps(period_ns, 4);
                let _ex = run_gc_cmd(&sup_stop, &clock_stop, false).await.unwrap();
                clock_stop.run_steps(period_ns, 6);
            }

            let stats_stop = Arc::clone(exec_stop.stats());
            let exec_stop_watch = exec_stop.clone();
            let stop_jh = tokio::task::spawn_blocking(move || exec_stop.stop_and_join());
            let dl_exit = Instant::now() + Duration::from_secs(5);
            while !exec_stop_watch.is_exited() {
                assert!(Instant::now() < dl_exit, "armd-control did not exit on full-ring stop_and_join");
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
            assert_eq!(
                stats_stop.total_allocs(),
                AllocCounts::ZERO,
                "full-ring stop_and_join must execute 0 RT allocs/frees"
            );
            pause_stop.store(false, Ordering::Release);
            rel_stop.send(()).unwrap();
            jh_stop.join().unwrap();
            stop_jh.await.unwrap();
        }
    }
}

// ============================================================================
// 25. R1 Blocker 1: Legacy Damiao rearm and NormalUnload use PR0B/PR0C
//     ModeSwitch (never synchronous dual-format rollback)
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn legacy_damiao_rearm_and_normal_unload_use_mode_switch_never_dual_rollback() {
    let cfg = mit_config_j4();
    let rest = rest_pose_of(&cfg);
    let qual = valid_mit_qualification_j4(&cfg);
    let rig = LiveRig::start(
        OwnerKind::Legacy,
        cfg,
        rest,
        OperatorAuth::disabled(),
        Some(qual),
        false,
    )
    .await;

    let mut ws = ws_connect(&rig.ws_url).await;
    let granted = ws_send_wait_types(
        &mut ws,
        json!({"type": "acquire_lease", "client_id": "legacy-ms-rearm", "seq": 1}),
        &["lease_granted"],
    )
    .await;
    let gen = granted["generation"].as_u64().unwrap();
    let ep0 = rig.sup.get_observation().await.epoch_id;

    let arm_ack = ws_rpc(&mut ws, 1, gen, ep0, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(arm_ack["type"], "command_ack", "{arm_ack}");
    let dl_mit = Instant::now() + Duration::from_secs(3);
    loop {
        let hw_mit = rig.bus.with(|s| s.motors[3].ctrl_mode()) == damiao::CTRL_MODE_MIT;
        let owner_active = rig.sup.inner().read().await.mit_active();
        if hw_mit && owner_active {
            break;
        }
        assert!(Instant::now() < dl_mit);
        tokio::time::sleep(Duration::from_millis(15)).await;
    }

    // 1. Direct synchronous `mit_hand_back` is forbidden on DamiaoArmDriver.
    {
        let mut inner = rig.sup.inner().write().await;
        let sync_err = inner
            .mit_hand_back("test")
            .expect_err("synchronous mit_hand_back must be forbidden on DamiaoArmDriver");
        assert!(sync_err.contains("ModeSwitch") || sync_err.contains("PosVelHandback"), "{sync_err}");
    }

    // 2. WS `/api/v1/torque` (`enable: true`) rearm while MIT is active runs through PR0B/PR0C
    //    PosVelHandback (J4 -> POS_VEL precedes RID_TIMEOUT readback, 0 CMD_DISABLE frames),
    //    then re-arms MIT cleanly.
    rig.clear_bus_log();
    let ep_rearm = rig.sup.get_observation().await.epoch_id;
    let rearm_ack = ws_rpc(&mut ws, 2, gen, ep_rearm, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(rearm_ack["type"], "command_ack", "{rearm_ack}");
    let (back_idx, read_idx, disable_cnt) = rig.bus.with(|s| {
        let back = s
            .tx_log
            .iter()
            .position(|f| f.id == 0x7FF && f.data == damiao::build_set_control_mode_frame(4, damiao::CTRL_MODE_POS_VEL).data);
        let read = s
            .tx_log
            .iter()
            .position(|f| f.id == 0x7FF && f.data[..4] == [4, 0, damiao::REG_READ, damiao::RID_TIMEOUT]);
        let dis = s.tx_log.iter().filter(|f| f.data == damiao::CMD_DISABLE).count();
        (back, read, dis)
    });
    let back_i = back_idx.expect("PosVelHandback must write J4 CTRL_MODE_POS_VEL before rearm");
    let read_i = read_idx.expect("rearm must read J4 RID_TIMEOUT");
    assert!(back_i < read_i, "J4 PosVelHandback must complete before RID_TIMEOUT readback");
    assert_eq!(disable_cnt, 0, "rearm must emit 0 CMD_DISABLE frames");

    let dl_mit2 = Instant::now() + Duration::from_secs(3);
    loop {
        let hw_mit = rig.bus.with(|s| s.motors[3].ctrl_mode()) == damiao::CTRL_MODE_MIT;
        let owner_active = rig.sup.inner().read().await.mit_active();
        if hw_mit && owner_active {
            break;
        }
        assert!(Instant::now() < dl_mit2);
        tokio::time::sleep(Duration::from_millis(15)).await;
    }

    // 3. `StopPolicy::NormalUnload` while MIT is active defers to `PosVelHandback`, hands J4 back
    //    to `POS_VEL`, and parks cleanly with all motors disabled.
    rig.clear_bus_log();
    rig.sup.apply_stop_policy(armd::StopPolicy::NormalUnload).await;
    let dl_park = Instant::now() + Duration::from_secs(4);
    loop {
        let obs = rig.sup.get_observation().await;
        if !obs.is_torqued && rig.bus.with(|s| s.motors.iter().all(|m| !m.enabled)) {
            break;
        }
        assert!(Instant::now() < dl_park, "NormalUnload park did not finish: {obs:?}");
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    rig.bus.with(|s| {
        assert_eq!(s.motors[3].ctrl_mode(), damiao::CTRL_MODE_POS_VEL);
        assert!(s.tx_log.iter().any(|f| {
            f.id == 0x7FF && f.data == damiao::build_set_control_mode_frame(4, damiao::CTRL_MODE_POS_VEL).data
        }));
        assert!(s.motors.iter().all(|m| !m.enabled));
    });
    rig.shutdown().await;
}

// ============================================================================
// 26. R3 Blocker 1: Sequential 2-round causal confirmation of post-recovery
//     disable (both executor and legacy owners)
// ============================================================================

#[test]
fn mit_expiry_recovery_sequential_two_round_confirmation_rejects_in_window_stale_replies() {
    use armd::transport::{CanRxTime, CanTransport, DriverClock, RxClockSample, TransportError};

    let cfg = mit_config_j4();
    let rest = rest_pose_of(&cfg);
    let limits = b601_limits();

    #[derive(Clone)]
    struct DelayedWireBus {
        inner: FakeBus,
        pending_wire: Arc<Mutex<VecDeque<CanFrame>>>,
        hold_wire: Arc<AtomicBool>,
    }

    impl CanTransport for DelayedWireBus {
        fn clock(&self) -> Option<Arc<dyn DriverClock>> {
            self.inner.clock()
        }
        fn rx_clock_now(&self) -> Option<CanRxTime> {
            self.inner.rx_clock_now()
        }
        fn rx_clock_sample(&self) -> Option<RxClockSample> {
            self.inner.rx_clock_sample()
        }
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
            if self.hold_wire.load(Ordering::Acquire) {
                self.pending_wire.lock().unwrap().push_back(*frame);
                Ok(())
            } else {
                self.inner.transmit(frame)
            }
        }
        fn receive(&mut self) -> Result<CanFrame, TransportError> {
            self.inner.receive()
        }
        fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
            self.inner.receive_timestamped()
        }
        fn receive_with_rx_time(&mut self) -> Result<(CanFrame, Option<CanRxTime>), TransportError> {
            self.inner.receive_with_rx_time()
        }
        fn flush(&mut self) -> Result<(), TransportError> {
            Ok(())
        }
        fn supports_bounded_flush(&self) -> bool {
            self.inner.supports_bounded_flush()
        }
        fn supports_nonblocking_rt(&self) -> bool {
            self.inner.supports_nonblocking_rt()
        }
        fn flush_is_noop(&self) -> bool {
            true
        }
    }

    // (a) Executor (`mit_expiry_recover_budgeted`): inject an old RID_CTRL_MODE reply inside the
    //     new barrier window AND an old state==0 status frame inside the new Round 1 0xCC window
    //     while CMD_DISABLE + barrier + 0xCC stay queued in `pending_wire` -> MUST NOT confirm!
    {
        let clock = MockClock::new();
        let inner = FakeBus::with_clock(&limits, clock.clone());
        inner.with(|s| {
            s.poll_feeds_watchdog = false;
            s.immediate_replies = true;
            s.tx_step = Duration::from_micros(50);
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = rest[i];
                m.v = 0.0;
                m.enabled = i != 3;
                if i == 3 {
                    m.state_override = Some(0xD);
                }
            }
            s.push_all_feedback();
        });
        let pending_wire = Arc::new(Mutex::new(VecDeque::new()));
        let hold_wire = Arc::new(AtomicBool::new(false));
        let bus = DelayedWireBus {
            inner: inner.clone(),
            pending_wire: Arc::clone(&pending_wire),
            hold_wire: Arc::clone(&hold_wire),
        };
        let mut drv = build_standalone_mit_recovery_driver(bus, &rest);
        let started_at = clock.now();
        for rel in 1..=3 {
            clock.advance(Duration::from_millis(25));
            drv.begin_control_release(rel);
            inner.with(|s| s.push_all_feedback());
            drv.read_joint_states().unwrap();
        }

        // Hold all TX in `pending_wire` (SocketCAN queued TX with no-op flush).
        hold_wire.store(true, Ordering::Release);
        clock.advance(Duration::from_millis(25));
        drv.begin_control_release(4);
        inner.with(|s| s.push_all_feedback());
        drv.read_joint_states().unwrap();
        let dl4 = clock.now() + Duration::from_millis(500);
        let mut gate = || Ok(());
        drv.set_tx_release_deadline(Some(dl4));
        let r4 = drv.mit_expiry_recover_budgeted(dl4, started_at, &mut gate).unwrap();
        drv.set_tx_release_deadline(None);
        assert!(!r4, "must not confirm while ClearFault + CMD_DISABLE + barrier stay queued");

        // Inject OLD RID_CTRL_MODE replies whose arrival timestamp falls INSIDE the new barrier window!
        clock.advance(Duration::from_millis(2));
        let in_win_reg_ts = clock.now();
        inner.with(|s| {
            let reg_reps: Vec<CanFrame> = s
                .motors
                .iter()
                .map(|m| {
                    let [lo, hi] = m.id.to_le_bytes();
                    let val = m.ctrl_mode().to_le_bytes();
                    CanFrame::new_standard(
                        m.reply_id,
                        &[lo, hi, damiao::REG_READ, damiao::RID_CTRL_MODE, val[0], val[1], val[2], val[3]],
                    )
                    .unwrap()
                })
                .collect();
            for rep in reg_reps {
                s.push_rx_at(rep, Some(in_win_reg_ts));
            }
        });
        drv.begin_control_release(5);
        drv.read_joint_states().unwrap();
        let dl5 = clock.now() + Duration::from_millis(500);
        drv.set_tx_release_deadline(Some(dl5));
        let r5 = drv.mit_expiry_recover_budgeted(dl5, started_at, &mut gate).unwrap();
        drv.set_tx_release_deadline(None);
        assert!(!r5, "old in-window RID_CTRL_MODE reply alone must not confirm");
        let cc_after_r5 = pending_wire
            .lock()
            .unwrap()
            .iter()
            .filter(|f| f.id == 0x7FF && f.data[2] == 0xCC)
            .count();
        assert_eq!(cc_after_r5, 7, "Round 1 must queue 7 0xCC probes");

        // Inject OLD state==0 status frames whose arrival timestamp falls INSIDE the new Round 1 0xCC window
        // while CMD_DISABLE + barrier + Round 1 0xCC are STILL queued in `pending_wire`!
        clock.advance(Duration::from_millis(2));
        let in_win_status_ts = clock.now();
        inner.with(|s| {
            let mut frames = Vec::with_capacity(s.motors.len());
            for m in s.motors.iter_mut() {
                let prev_en = m.enabled;
                let prev_ovr = m.state_override;
                m.enabled = false;
                m.state_override = None;
                frames.push(m.feedback_frame());
                m.enabled = prev_en;
                m.state_override = prev_ovr;
            }
            for fb in frames {
                s.push_rx_at(fb, Some(in_win_status_ts));
            }
        });
        drv.begin_control_release(6);
        drv.read_joint_states().unwrap();
        let dl6 = clock.now() + Duration::from_millis(500);
        drv.set_tx_release_deadline(Some(dl6));
        let r6 = drv.mit_expiry_recover_budgeted(dl6, started_at, &mut gate).unwrap();
        drv.set_tx_release_deadline(None);
        assert!(
            !r6,
            "old in-window RID_CTRL_MODE reply + old in-window state==0 status MUST NOT confirm while TX stays queued"
        );
        // R2 Major 4: When `clock.now() == p1_ack_at`, Round 2 MUST defer until strictly later
        // (`p2_tx_at > p1_ack_at`) rather than submitting at an equal timestamp!
        let cc_after_r6_equal_ts = pending_wire
            .lock()
            .unwrap()
            .iter()
            .filter(|f| f.id == 0x7FF && f.data[2] == 0xCC)
            .count();
        assert_eq!(
            cc_after_r6_equal_ts, 7,
            "Round 2 MUST defer when clock.now() == p1_ack_at (no equal-timestamp Round 2 submission)"
        );
        // Advance clock strictly past `p1_ack_at` so Round 2 submits 7 fresh 0xCC probes.
        clock.advance(Duration::from_micros(50));
        drv.begin_control_release(60);
        drv.read_joint_states().unwrap();
        let dl60 = clock.now() + Duration::from_millis(500);
        drv.set_tx_release_deadline(Some(dl60));
        let r60 = drv.mit_expiry_recover_budgeted(dl60, started_at, &mut gate).unwrap();
        drv.set_tx_release_deadline(None);
        assert!(!r60, "Round 2 submission without Round 2 replies must not confirm");
        let cc_after_r6 = pending_wire
            .lock()
            .unwrap()
            .iter()
            .filter(|f| f.id == 0x7FF && f.data[2] == 0xCC)
            .count();
        assert_eq!(
            cc_after_r6, 14,
            "Round 2 must queue 7 fresh 0xCC probes strictly after Round 1 reply timestamp"
        );
        assert!(
            inner.with(|s| s.motors.iter().enumerate().all(|(i, m)| i == 3 || m.enabled)),
            "motors must still be enabled on wire while TX stays queued"
        );

        // Retry after Round 2 timeout resets back to Round 1 so a delayed reply from the timed-out
        // Round 2 probe landing inside the retry window can at most credit new Round 1 and NEVER
        // complete confirmation by itself!
        clock.advance(Duration::from_millis(55));
        drv.begin_control_release(7);
        drv.read_joint_states().unwrap();
        let dl7 = clock.now() + Duration::from_millis(500);
        drv.set_tx_release_deadline(Some(dl7));
        let r7 = drv.mit_expiry_recover_budgeted(dl7, started_at, &mut gate).unwrap();
        drv.set_tx_release_deadline(None);
        assert!(!r7, "Round 2 retry without fresh replies must NOT reuse Round 1 reply identity");
        let cc_after_r7 = pending_wire
            .lock()
            .unwrap()
            .iter()
            .filter(|f| f.id == 0x7FF && f.data[2] == 0xCC)
            .count();
        assert_eq!(cc_after_r7, 21, "Retry must submit 7 fresh 0xCC probes");

        // Inject a delayed reply from the timed-out probe inside the retry window (strictly > tx_at),
        // then advance 50 us so `clock.now() > delayed_retry_ts` for the fresh Round 2 submission.
        clock.advance(Duration::from_millis(2));
        let delayed_retry_ts = clock.now();
        inner.with(|s| {
            let mut frames = Vec::with_capacity(s.motors.len());
            for m in s.motors.iter_mut() {
                let prev_en = m.enabled;
                let prev_ovr = m.state_override;
                m.enabled = false;
                m.state_override = None;
                frames.push(m.feedback_frame());
                m.enabled = prev_en;
                m.state_override = prev_ovr;
            }
            for fb in frames {
                s.push_rx_at(fb, Some(delayed_retry_ts));
            }
        });
        clock.advance(Duration::from_micros(50));
        drv.begin_control_release(8);
        drv.read_joint_states().unwrap();
        let dl8_retry = clock.now() + Duration::from_millis(500);
        drv.set_tx_release_deadline(Some(dl8_retry));
        let r8_retry = drv.mit_expiry_recover_budgeted(dl8_retry, started_at, &mut gate).unwrap();
        drv.set_tx_release_deadline(None);
        assert!(
            !r8_retry,
            "delayed reply from timed-out Round 2 probe landing in retry window MUST NOT confirm"
        );
        let cc_after_r8 = pending_wire
            .lock()
            .unwrap()
            .iter()
            .filter(|f| f.id == 0x7FF && f.data[2] == 0xCC)
            .count();
        assert_eq!(cc_after_r8, 28, "Retry must still require a fresh second round (7 more 0xCC)");

        // Phase expiry fails closed (DisableRequestExpired), never reports success.
        clock.advance(Duration::from_millis(1000));
        drv.begin_control_release(9);
        drv.read_joint_states().unwrap();
        let dl9 = clock.now() + Duration::from_millis(500);
        drv.set_tx_release_deadline(Some(dl9));
        let r9 = drv.mit_expiry_recover_budgeted(dl9, started_at, &mut gate);
        drv.set_tx_release_deadline(None);
        assert!(
            matches!(r9, Err(ArmDriverError::DisableRequestExpired)),
            "unconfirmed recovery past phase window must fail closed with DisableRequestExpired, got {r9:?}"
        );
    }

    // (b) Legacy (`mit_expiry_recover_with_gate`): same in-window stale RID_CTRL_MODE + stale
    //     state==0 status attack while TX frames stay queued -> MUST fail closed (Err), never Ok!
    {
        #[derive(Clone)]
        struct LegacyInWindowAttackBus {
            inner: FakeBus,
            pending_wire: Arc<Mutex<VecDeque<CanFrame>>>,
            injected_reg: Arc<AtomicBool>,
            injected_status: Arc<AtomicBool>,
        }

        impl CanTransport for LegacyInWindowAttackBus {
            fn clock(&self) -> Option<Arc<dyn DriverClock>> {
                self.inner.clock()
            }
            fn rx_clock_now(&self) -> Option<CanRxTime> {
                self.inner.rx_clock_now()
            }
            fn rx_clock_sample(&self) -> Option<RxClockSample> {
                self.inner.rx_clock_sample()
            }
            fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
                let mut q = self.pending_wire.lock().unwrap();
                q.push_back(*frame);
                let reg_cnt = q
                    .iter()
                    .filter(|f| f.id == 0x7FF && f.data[2] == damiao::REG_READ && f.data[3] == damiao::RID_CTRL_MODE)
                    .count();
                let cc_cnt = q.iter().filter(|f| f.id == 0x7FF && f.data[2] == 0xCC).count();
                drop(q);
                if reg_cnt == 7 && !self.injected_reg.swap(true, Ordering::AcqRel) {
                    self.inner.with(|s| {
                        if let Some(c) = &s.clock {
                            c.advance(Duration::from_micros(50));
                        }
                        let ts = s.now();
                        let reg_reps: Vec<CanFrame> = s
                            .motors
                            .iter()
                            .map(|m| {
                                let [lo, hi] = m.id.to_le_bytes();
                                let val = m.ctrl_mode().to_le_bytes();
                                CanFrame::new_standard(
                                    m.reply_id,
                                    &[lo, hi, damiao::REG_READ, damiao::RID_CTRL_MODE, val[0], val[1], val[2], val[3]],
                                )
                                .unwrap()
                            })
                            .collect();
                        for rep in reg_reps {
                            s.push_rx_at(rep, ts);
                        }
                    });
                }
                if cc_cnt == 7 && !self.injected_status.swap(true, Ordering::AcqRel) {
                    self.inner.with(|s| {
                        if let Some(c) = &s.clock {
                            c.advance(Duration::from_micros(50));
                        }
                        let ts = s.now();
                        let mut frames = Vec::with_capacity(s.motors.len());
                        for m in s.motors.iter_mut() {
                            let prev_en = m.enabled;
                            let prev_ovr = m.state_override;
                            m.enabled = false;
                            m.state_override = None;
                            frames.push(m.feedback_frame());
                            m.enabled = prev_en;
                            m.state_override = prev_ovr;
                        }
                        for fb in frames {
                            s.push_rx_at(fb, ts);
                        }
                    });
                }
                Ok(())
            }
            fn receive(&mut self) -> Result<CanFrame, TransportError> {
                self.inner.receive()
            }
            fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
                self.inner.receive_timestamped()
            }
            fn receive_with_rx_time(&mut self) -> Result<(CanFrame, Option<CanRxTime>), TransportError> {
                self.inner.receive_with_rx_time()
            }
            fn flush(&mut self) -> Result<(), TransportError> {
                Ok(())
            }
            fn supports_bounded_flush(&self) -> bool {
                self.inner.supports_bounded_flush()
            }
            fn supports_nonblocking_rt(&self) -> bool {
                self.inner.supports_nonblocking_rt()
            }
            fn flush_is_noop(&self) -> bool {
                true
            }
        }

        let clock = MockClock::new();
        let inner = FakeBus::with_clock(&limits, clock.clone());
        inner.with(|s| {
            s.poll_feeds_watchdog = false;
            s.immediate_replies = true;
            s.tx_step = Duration::from_micros(50);
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = rest[i];
                m.v = 0.0;
                m.enabled = i != 3;
                if i == 3 {
                    m.state_override = Some(0xD);
                }
            }
            s.push_all_feedback();
        });
        let pending_wire = Arc::new(Mutex::new(VecDeque::new()));
        let bus = LegacyInWindowAttackBus {
            inner: inner.clone(),
            pending_wire: Arc::clone(&pending_wire),
            injected_reg: Arc::new(AtomicBool::new(false)),
            injected_status: Arc::new(AtomicBool::new(false)),
        };
        let mut drv = build_standalone_mit_recovery_driver(bus, &rest);
        drv.read_joint_states().unwrap();
        for rel in 1..=3 {
            clock.advance(Duration::from_millis(25));
            drv.begin_control_release(rel);
            inner.with(|s| s.push_all_feedback());
            drv.read_joint_states().unwrap();
        }
        let mut gate = || Ok(());
        let res = drv.mit_expiry_recover_with_gate(&mut gate);
        assert!(
            res.is_err(),
            "legacy mit_expiry_recover_with_gate MUST fail closed on single in-window stale reply + stale status, got {res:?}"
        );
        let cc_sent = pending_wire
            .lock()
            .unwrap()
            .iter()
            .filter(|f| f.id == 0x7FF && f.data[2] == 0xCC)
            .count();
        assert!(
            cc_sent >= 14,
            "legacy recovery must attempt at least 2 sequential 0xCC probe rounds, got {cc_sent}"
        );
    }

    // (c) Genuine sequence still confirms cleanly on BOTH Executor and Legacy owners.
    for owner_legacy in [false, true] {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&limits, clock.clone());
        bus.with(|s| {
            s.poll_feeds_watchdog = false;
            s.immediate_replies = true;
            s.tx_step = Duration::from_micros(50);
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = rest[i];
                m.v = 0.0;
                m.enabled = i != 3;
                if i == 3 {
                    m.state_override = Some(0xD);
                }
            }
            s.push_all_feedback();
        });
        let mut drv = build_standalone_mit_recovery_driver(bus.clone(), &rest);
        drv.read_joint_states().unwrap();
        let started_at = clock.now();
        for rel in 1..=3 {
            clock.advance(Duration::from_millis(25));
            drv.begin_control_release(rel);
            bus.with(|s| s.push_all_feedback());
            drv.read_joint_states().unwrap();
        }
        bus.clear_log();
        let mut gate = || Ok(());
        if owner_legacy {
            drv.mit_expiry_recover_with_gate(&mut gate)
                .expect("genuine legacy MIT expiry recovery must confirm");
        } else {
            let mut done = false;
            for rel in 4..=8 {
                clock.advance(Duration::from_millis(10));
                drv.begin_control_release(rel);
                drv.read_joint_states().unwrap();
                let dl = clock.now() + Duration::from_millis(500);
                drv.set_tx_release_deadline(Some(dl));
                let step = drv.mit_expiry_recover_budgeted(dl, started_at, &mut gate).unwrap();
                drv.set_tx_release_deadline(None);
                if step {
                    done = true;
                    break;
                }
            }
            assert!(done, "genuine executor MIT expiry recovery must confirm");
        }
        let cc_cnt = bus.with(|s| {
            s.tx_log
                .iter()
                .filter(|f| f.id == 0x7FF && f.data[2] == 0xCC)
                .count()
        });
        assert_eq!(
            cc_cnt, 14,
            "genuine 2-round confirmation across 7 axes must issue exactly 14 0xCC probes (owner_legacy={owner_legacy}), got {cc_cnt}"
        );
        assert!(bus.with(|s| s.motors.iter().all(|m| !m.enabled && m.state_override.is_none())));
    }

    // (d) R3 Major 1: Equal-timestamp replies (`rx_at == tx_at` and `rx_at == prev_ack_at`)
    //     MUST be rejected (strict `>` timestamp ordering on barrier, Round 1, and Round 2).
    {
        let clock = MockClock::new();
        let inner = FakeBus::with_clock(&limits, clock.clone());
        inner.with(|s| {
            s.poll_feeds_watchdog = false;
            s.immediate_replies = true;
            // Deliberately keep `tx_step == ZERO` so immediate replies carry `rx_at == tx_at`!
            s.tx_step = Duration::ZERO;
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = rest[i];
                m.v = 0.0;
                m.enabled = i != 3;
                if i == 3 {
                    m.state_override = Some(0xD);
                }
            }
            s.push_all_feedback();
        });
        let mut drv = build_standalone_mit_recovery_driver(inner.clone(), &rest);
        drv.read_joint_states().unwrap();
        let started_at = clock.now();
        for rel in 1..=3 {
            clock.advance(Duration::from_millis(25));
            drv.begin_control_release(rel);
            inner.with(|s| s.push_all_feedback());
            drv.read_joint_states().unwrap();
        }
        let mut gate = || Ok(());
        drv.begin_control_release(4);
        drv.read_joint_states().unwrap();
        let dl4 = clock.now() + Duration::from_millis(500);
        drv.set_tx_release_deadline(Some(dl4));
        let r4 = drv.mit_expiry_recover_budgeted(dl4, started_at, &mut gate).unwrap();
        drv.set_tx_release_deadline(None);
        assert!(!r4, "equal-timestamp barrier/probe replies (rx_at == tx_at) MUST NOT confirm");

        // Now verify Round 2 rejects a reply whose timestamp equals Round 1's confirming reply.
        let pending_wire = Arc::new(Mutex::new(VecDeque::new()));
        let hold_wire = Arc::new(AtomicBool::new(true));
        let inner2 = FakeBus::with_clock(&limits, clock.clone());
        inner2.with(|s| {
            s.poll_feeds_watchdog = false;
            s.immediate_replies = true;
            s.tx_step = Duration::from_micros(50);
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = rest[i];
                m.v = 0.0;
                m.enabled = i != 3;
                if i == 3 {
                    m.state_override = Some(0xD);
                }
            }
            s.push_all_feedback();
        });
        let bus2 = DelayedWireBus {
            inner: inner2.clone(),
            pending_wire: Arc::clone(&pending_wire),
            hold_wire: Arc::clone(&hold_wire),
        };
        let mut drv2 = build_standalone_mit_recovery_driver(bus2, &rest);
        drv2.read_joint_states().unwrap();
        let started2 = clock.now();
        for rel in 1..=3 {
            clock.advance(Duration::from_millis(25));
            drv2.begin_control_release(rel);
            inner2.with(|s| s.push_all_feedback());
            drv2.read_joint_states().unwrap();
        }
        // Release 4 queues ClearFault + CMD_DISABLE + barrier.
        drv2.begin_control_release(4);
        drv2.read_joint_states().unwrap();
        let dl = clock.now() + Duration::from_millis(500);
        drv2.set_tx_release_deadline(Some(dl));
        assert!(!drv2.mit_expiry_recover_budgeted(dl, started2, &mut gate).unwrap());
        drv2.set_tx_release_deadline(None);
        // Inject strictly-later barrier replies.
        clock.advance(Duration::from_millis(1));
        let bar_ts = clock.now();
        inner2.with(|s| {
            let reps: Vec<CanFrame> = s.motors.iter().map(|m| {
                let [lo, hi] = m.id.to_le_bytes();
                let val = m.ctrl_mode().to_le_bytes();
                CanFrame::new_standard(m.reply_id, &[lo, hi, damiao::REG_READ, damiao::RID_CTRL_MODE, val[0], val[1], val[2], val[3]]).unwrap()
            }).collect();
            for rep in reps { s.push_rx_at(rep, Some(bar_ts)); }
        });
        drv2.begin_control_release(5);
        drv2.read_joint_states().unwrap();
        let dl5 = clock.now() + Duration::from_millis(500);
        drv2.set_tx_release_deadline(Some(dl5));
        assert!(!drv2.mit_expiry_recover_budgeted(dl5, started2, &mut gate).unwrap());
        drv2.set_tx_release_deadline(None);
        // Inject strictly-later Round 1 replies (`p1_ts`).
        clock.advance(Duration::from_millis(1));
        let p1_ts = clock.now();
        inner2.with(|s| {
            let mut frames = Vec::with_capacity(s.motors.len());
            for m in s.motors.iter_mut() {
                m.enabled = false;
                m.state_override = None;
                frames.push(m.feedback_frame());
            }
            for fb in frames { s.push_rx_at(fb, Some(p1_ts)); }
        });
        drv2.begin_control_release(6);
        drv2.read_joint_states().unwrap();
        let dl6 = clock.now() + Duration::from_millis(500);
        drv2.set_tx_release_deadline(Some(dl6));
        assert!(!drv2.mit_expiry_recover_budgeted(dl6, started2, &mut gate).unwrap());
        drv2.set_tx_release_deadline(None);
        // Inject Round 2 replies with timestamp EQUAL to `p1_ts` (== `p2_tx_at` == `p1_ack_at`) -> MUST NOT confirm!
        inner2.with(|s| {
            let frames: Vec<CanFrame> = s.motors.iter().map(|m| m.feedback_frame()).collect();
            for fb in frames { s.push_rx_at(fb, Some(p1_ts)); }
        });
        drv2.begin_control_release(7);
        drv2.read_joint_states().unwrap();
        let dl7 = clock.now() + Duration::from_millis(500);
        drv2.set_tx_release_deadline(Some(dl7));
        let r7 = drv2.mit_expiry_recover_budgeted(dl7, started2, &mut gate).unwrap();
        drv2.set_tx_release_deadline(None);
        assert!(!r7, "Round 2 reply with timestamp == Round 1 reply / probe 2 TX MUST NOT confirm");
    }

    // (e) R2 Minor 1: Phase deadline boundary — explicitly assert the probe sequence so Round 2
    //     is actively outstanding within 50 ms of `phase_deadline`, then pair exact-deadline
    //     rejection (`clock.now() == phase_deadline` -> `Err(DisableRequestExpired)`) with an
    //     otherwise identical pre-deadline (`phase_deadline - 1 ms` -> `Ok(true)`) success case.
    for reply_at_deadline in [true, false] {
        let clock = MockClock::new();
        let inner = FakeBus::with_clock(&limits, clock.clone());
        inner.with(|s| {
            s.poll_feeds_watchdog = false;
            s.immediate_replies = true;
            s.tx_step = Duration::from_micros(50);
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = rest[i];
                m.v = 0.0;
                m.enabled = i != 3;
                if i == 3 {
                    m.state_override = Some(0xD);
                }
            }
            s.push_all_feedback();
        });
        let pending_wire = Arc::new(Mutex::new(VecDeque::new()));
        let hold_wire = Arc::new(AtomicBool::new(true));
        let bus = DelayedWireBus {
            inner: inner.clone(),
            pending_wire: Arc::clone(&pending_wire),
            hold_wire: Arc::clone(&hold_wire),
        };
        let mut drv = build_standalone_mit_recovery_driver(bus, &rest);
        drv.read_joint_states().unwrap();
        for rel in 1..=3 {
            clock.advance(Duration::from_millis(25));
            drv.begin_control_release(rel);
            inner.with(|s| s.push_all_feedback());
            drv.read_joint_states().unwrap();
        }
        let started_at = clock.now();
        let phase_deadline = started_at + Duration::from_secs(1);
        let mut gate = || Ok(());

        // Start recovery right near `phase_deadline - 25 ms` (within 50 ms of phase_deadline!)
        // after refreshing rest-pose evidence so CMD_DISABLE + barrier + Round 1 + Round 2 all
        // execute strictly inside the final 50 ms window.
        for step in 1..=19 {
            clock.advance(Duration::from_millis(50));
            drv.begin_control_release(10 + step);
            inner.with(|s| s.push_all_feedback());
            drv.read_joint_states().unwrap();
        }
        // Now at `started_at + 950 ms` (`phase_deadline - 50 ms`). Advance 25 ms to `phase_deadline - 25 ms`.
        clock.advance(Duration::from_millis(25));
        drv.begin_control_release(40);
        inner.with(|s| s.push_all_feedback());
        drv.read_joint_states().unwrap();

        // Release 40: queue ClearFault + CMD_DISABLE + barrier (still held on wire).
        let dl40 = phase_deadline + Duration::from_millis(50);
        drv.set_tx_release_deadline(Some(dl40));
        assert!(!drv.mit_expiry_recover_budgeted(dl40, started_at, &mut gate).unwrap());
        drv.set_tx_release_deadline(None);
        // Deliver ClearFault + CMD_DISABLE + barrier frames on wire so motors actually disable
        // and emit real causal barrier replies.
        let mut drained = Vec::new();
        {
            let mut q = pending_wire.lock().unwrap();
            drained.extend(q.drain(..));
        }
        assert!(
            drained.iter().any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_READ),
            "barrier REG_READ frames must be queued before Round 1"
        );
        for f in drained {
            inner.clone().transmit(&f).unwrap();
        }

        // Release 41: harvest barrier replies and queue Round 1 `0xCC` probes (held on wire).
        clock.advance(Duration::from_millis(2));
        drv.begin_control_release(41);
        drv.read_joint_states().unwrap();
        drv.set_tx_release_deadline(Some(dl40));
        assert!(!drv.mit_expiry_recover_budgeted(dl40, started_at, &mut gate).unwrap());
        drv.set_tx_release_deadline(None);
        let p1_frames: Vec<CanFrame> = pending_wire.lock().unwrap().drain(..).collect();
        let p1_cc = p1_frames.iter().filter(|f| f.id == 0x7FF && f.data[2] == 0xCC).count();
        assert_eq!(p1_cc, 7, "Round 1 must queue exactly 7 0xCC probes");
        for f in p1_frames {
            inner.clone().transmit(&f).unwrap();
        }

        // Release 42: harvest Round 1 replies and submit Round 2 `0xCC` probes (held on wire)
        // strictly within 50 ms of `phase_deadline`!
        clock.advance(Duration::from_millis(2));
        assert!(
            phase_deadline.saturating_duration_since(clock.now()) < Duration::from_millis(50),
            "Round 2 probe must be submitted strictly within 50 ms of phase_deadline"
        );
        drv.begin_control_release(42);
        drv.read_joint_states().unwrap();
        drv.set_tx_release_deadline(Some(dl40));
        assert!(!drv.mit_expiry_recover_budgeted(dl40, started_at, &mut gate).unwrap());
        drv.set_tx_release_deadline(None);
        let p2_frames: Vec<CanFrame> = pending_wire.lock().unwrap().drain(..).collect();
        let p2_cc = p2_frames.iter().filter(|f| f.id == 0x7FF && f.data[2] == 0xCC).count();
        assert_eq!(p2_cc, 7, "Round 2 must be actively outstanding (7 fresh 0xCC probes) before deadline");

        // Now deliver Round 2 replies either AT exact `phase_deadline` (must reject) or at
        // `phase_deadline - 1 ms` (must commit Ok(true)) from the exact same Round 2 state.
        let target_ts = if reply_at_deadline {
            phase_deadline
        } else {
            phase_deadline - Duration::from_millis(1)
        };
        let adv = target_ts.saturating_duration_since(clock.now());
        clock.advance(adv);
        assert_eq!(clock.now(), target_ts);
        inner.with(|s| {
            let frames: Vec<CanFrame> = s.motors.iter().map(|m| m.feedback_frame()).collect();
            for fb in frames {
                s.push_rx_at(fb, Some(target_ts));
            }
        });
        drv.begin_control_release(43);
        drv.read_joint_states().unwrap();
        let dl43 = phase_deadline + Duration::from_millis(50);
        drv.set_tx_release_deadline(Some(dl43));
        let r43 = drv.mit_expiry_recover_budgeted(dl43, started_at, &mut gate);
        drv.set_tx_release_deadline(None);
        if reply_at_deadline {
            assert!(
                matches!(r43, Err(ArmDriverError::DisableRequestExpired)),
                "Round 2 reply arriving AT exact phase_deadline MUST fail closed with DisableRequestExpired, got {r43:?}"
            );
        } else {
            assert!(
                matches!(r43, Ok(true)),
                "otherwise identical Round 2 reply arriving at phase_deadline - 1 ms MUST confirm Ok(true), got {r43:?}"
            );
        }
    }
}

// ============================================================================
// 27. R3 Major 2: Legacy MIT expiry recovery preserves watchdog continuity
//     (per-axis hold frame gap <= 40 ms) under lost or queued disable
// ============================================================================

#[test]
fn legacy_mit_expiry_recovery_preserves_watchdog_continuity_under_lost_or_queued_disable() {
    use armd::transport::{CanRxTime, CanTransport, DriverClock, RxClockSample, TransportError};

    let cfg = mit_config_j4();
    let rest = rest_pose_of(&cfg);
    let limits = b601_limits();

    #[derive(Clone)]
    struct ContinuityTestBus {
        inner: FakeBus,
        /// When true, queue all TX without delivering to motors (simulates queued disable/barrier).
        queue_all: Arc<AtomicBool>,
        /// Remaining `CMD_DISABLE` drops for motor_id 1 (J1).
        drop_j1_disable_remaining: Arc<Mutex<usize>>,
        /// Drop J1's first `REG_READ(RID_CTRL_MODE)` barrier once (forces >= 50 ms wait).
        drop_j1_barrier_once: Arc<AtomicBool>,
        /// All transmitted `(timestamp, frame)` recorded at host TX submission.
        recorded_tx: Arc<Mutex<Vec<(Instant, CanFrame)>>>,
    }

    impl CanTransport for ContinuityTestBus {
        fn clock(&self) -> Option<Arc<dyn DriverClock>> {
            self.inner.clock()
        }
        fn rx_clock_now(&self) -> Option<CanRxTime> {
            self.inner.rx_clock_now()
        }
        fn rx_clock_sample(&self) -> Option<RxClockSample> {
            self.inner.rx_clock_sample()
        }
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
            let now = self.inner.with(|s| s.now()).unwrap_or_else(Instant::now);
            self.recorded_tx.lock().unwrap().push((now, *frame));
            if self.queue_all.load(Ordering::Acquire) {
                return Ok(());
            }
            if frame.id == 1 && frame.data == damiao::CMD_DISABLE {
                let mut rem = self.drop_j1_disable_remaining.lock().unwrap();
                if *rem > 0 {
                    *rem -= 1;
                    return Ok(());
                }
            }
            if frame.id == 0x7FF
                && u16::from_le_bytes([frame.data[0], frame.data[1]]) == 1
                && frame.data[2] == damiao::REG_READ
                && frame.data[3] == damiao::RID_CTRL_MODE
                && self.drop_j1_barrier_once.swap(false, Ordering::AcqRel)
            {
                return Ok(());
            }
            self.inner.transmit(frame)
        }
        fn receive(&mut self) -> Result<CanFrame, TransportError> {
            self.inner.receive()
        }
        fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
            self.inner.receive_timestamped()
        }
        fn receive_with_rx_time(&mut self) -> Result<(CanFrame, Option<CanRxTime>), TransportError> {
            self.inner.receive_with_rx_time()
        }
        fn flush(&mut self) -> Result<(), TransportError> {
            Ok(())
        }
        fn supports_bounded_flush(&self) -> bool {
            self.inner.supports_bounded_flush()
        }
        fn supports_nonblocking_rt(&self) -> bool {
            self.inner.supports_nonblocking_rt()
        }
        fn flush_is_noop(&self) -> bool {
            true
        }
    }

    let check_axis_max_gap = |recorded: &[(Instant, CanFrame)],
                              motor_id: u16,
                              window_start: Instant,
                              window_end: Instant|
     -> (usize, Duration) {
        let ctrl_id = 0x100u32 + u32::from(motor_id);
        let mut times: Vec<Instant> = recorded
            .iter()
            .filter(|(t, f)| *t >= window_start && *t <= window_end && f.id == ctrl_id)
            .map(|(t, _)| *t)
            .collect();
        let count = times.len();
        times.insert(0, window_start);
        times.push(window_end);
        let max_gap = times
            .windows(2)
            .map(|w| w[1].saturating_duration_since(w[0]))
            .max()
            .unwrap_or(Duration::ZERO);
        (count, max_gap)
    };

    // Subcase A: Lost CMD_DISABLE (dropped 2x) + dropped barrier (50 ms wait) on energized J1 ->
    //            recovers cleanly on retry AND keeps per-axis hold frame gap <= 40 ms throughout!
    {
        let clock = MockClock::new();
        let inner = FakeBus::with_clock(&limits, clock.clone());
        inner.with(|s| {
            s.poll_feeds_watchdog = false;
            s.immediate_replies = true;
            s.tx_step = Duration::from_micros(50);
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = rest[i];
                m.v = 0.0;
                m.enabled = i != 3;
                if i == 3 {
                    m.state_override = Some(0xD);
                }
            }
            s.push_all_feedback();
        });
        let recorded_tx = Arc::new(Mutex::new(Vec::new()));
        let bus = ContinuityTestBus {
            inner: inner.clone(),
            queue_all: Arc::new(AtomicBool::new(false)),
            drop_j1_disable_remaining: Arc::new(Mutex::new(2)),
            drop_j1_barrier_once: Arc::new(AtomicBool::new(true)),
            recorded_tx: Arc::clone(&recorded_tx),
        };
        let mut drv = build_standalone_mit_recovery_driver(bus, &rest);
        for i in 0..7 {
            if i != 3 {
                drv.set_axis_confirmed_mode(i, Some(armd::motor_mode::ControlMode::PositionVelocity));
            }
        }
        drv.read_joint_states().unwrap();
        for rel in 1..=3 {
            clock.advance(Duration::from_millis(25));
            drv.begin_control_release(rel);
            inner.with(|s| s.push_all_feedback());
            drv.read_joint_states().unwrap();
        }
        let t_start = clock.now();
        let mut gate = || Ok(());
        drv.mit_expiry_recover_with_gate(&mut gate)
            .expect("lost disable + dropped barrier must still recover on retry");
        let t_end = clock.now();
        assert!(
            t_end.saturating_duration_since(t_start) >= Duration::from_millis(50),
            "dropped barrier must exercise >= 50 ms wait window, elapsed={:?}",
            t_end.saturating_duration_since(t_start)
        );
        let rec = recorded_tx.lock().unwrap();
        let (j1_holds, j1_max_gap) = check_axis_max_gap(&rec, 1, t_start, t_end);
        assert!(
            j1_holds >= 2,
            "energized J1 with lost disable + 50 ms barrier wait must receive continuity holds, got {j1_holds}"
        );
        assert!(
            j1_max_gap <= Duration::from_millis(40),
            "J1 per-axis hold frame gap during legacy recovery exceeded 40 ms: {j1_max_gap:?}"
        );
        assert!(inner.with(|s| s.motors.iter().all(|m| !m.enabled && m.state_override.is_none())));
    }

    // Subcase B: Queued CMD_DISABLE on all energized arm axes (J1..J3, J5..J6) for entire 200 ms
    //            phase window -> fails closed AND keeps every energized axis frame gap <= 40 ms!
    {
        let clock = MockClock::new();
        let inner = FakeBus::with_clock(&limits, clock.clone());
        inner.with(|s| {
            s.poll_feeds_watchdog = false;
            s.immediate_replies = true;
            s.tx_step = Duration::from_micros(50);
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = rest[i];
                m.v = 0.0;
                m.enabled = i != 3;
                if i == 3 {
                    m.state_override = Some(0xD);
                }
            }
            s.push_all_feedback();
        });
        let recorded_tx = Arc::new(Mutex::new(Vec::new()));
        let queue_all = Arc::new(AtomicBool::new(false));
        let bus = ContinuityTestBus {
            inner: inner.clone(),
            queue_all: Arc::clone(&queue_all),
            drop_j1_disable_remaining: Arc::new(Mutex::new(0)),
            drop_j1_barrier_once: Arc::new(AtomicBool::new(false)),
            recorded_tx: Arc::clone(&recorded_tx),
        };
        let mut drv = build_standalone_mit_recovery_driver(bus, &rest);
        for i in 0..7 {
            if i != 3 {
                drv.set_axis_confirmed_mode(i, Some(armd::motor_mode::ControlMode::PositionVelocity));
            }
        }
        drv.read_joint_states().unwrap();
        for rel in 1..=3 {
            clock.advance(Duration::from_millis(25));
            drv.begin_control_release(rel);
            inner.with(|s| s.push_all_feedback());
            drv.read_joint_states().unwrap();
        }
        queue_all.store(true, Ordering::Release);
        let t_start = clock.now();
        let mut gate = || Ok(());
        let res = drv.mit_expiry_recover_with_gate(&mut gate);
        let t_end = clock.now();
        assert!(res.is_err(), "queued disable must fail closed after phase window");
        assert!(
            t_end.saturating_duration_since(t_start) >= Duration::from_millis(190),
            "queued disable must run full phase window (~200 ms), elapsed={:?}",
            t_end.saturating_duration_since(t_start)
        );
        let rec = recorded_tx.lock().unwrap();
        for motor_id in [1u16, 2, 3, 5, 6] {
            let (holds, max_gap) = check_axis_max_gap(&rec, motor_id, t_start, t_end);
            assert!(
                holds >= 9,
                "energized motor {motor_id} over 200 ms queued recovery must receive >= 9 holds, got {holds}"
            );
            assert!(
                max_gap <= Duration::from_millis(40),
                "energized motor {motor_id} per-axis frame gap exceeded 40 ms: {max_gap:?}"
            );
        }
        assert_eq!(
            drv.disable_dispatched_mask(),
            0,
            "failed recovery must clear unconfirmed disable_dispatched_mask so SAFE-HOLD continues holding"
        );
    }

    // Subcase C (R3 Major 3): Queued CMD_DISABLE on energized J1 + injected stale `state == 0`
    //            status while continuity hold frames ARE delivered to the fake motor ->
    //            provisional `state == 0` MUST NOT clear `motor_energized_mask`/`motor_last_ctrl_tx`,
    //            and J1 keeps receiving real wire hold frames with per-axis gap <= 40 ms!
    {
        #[derive(Clone)]
        struct StaleZeroDeliverHoldsBus {
            inner: FakeBus,
            injected_stale_zero_j1: Arc<AtomicBool>,
            delivered_j1_holds: Arc<Mutex<Vec<Instant>>>,
        }

        impl CanTransport for StaleZeroDeliverHoldsBus {
            fn clock(&self) -> Option<Arc<dyn DriverClock>> {
                self.inner.clock()
            }
            fn rx_clock_now(&self) -> Option<CanRxTime> {
                self.inner.rx_clock_now()
            }
            fn rx_clock_sample(&self) -> Option<RxClockSample> {
                self.inner.rx_clock_sample()
            }
            fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
                // Queue CMD_DISABLE and barrier for J1 (never reaching motor 1) and inject one
                // stale `state == 0` feedback frame for J1 to try to clear `motor_energized_mask`.
                if frame.id == 1 && frame.data == damiao::CMD_DISABLE {
                    if !self.injected_stale_zero_j1.swap(true, Ordering::AcqRel) {
                        self.inner.with(|s| {
                            if let Some(c) = &s.clock {
                                c.advance(Duration::from_micros(50));
                            }
                            let ts = s.now();
                            let m = &mut s.motors[0];
                            let prev_en = m.enabled;
                            let prev_ovr = m.state_override;
                            m.enabled = false;
                            m.state_override = None;
                            let fb = m.feedback_frame();
                            m.enabled = prev_en;
                            m.state_override = prev_ovr;
                            s.push_rx_at(fb, ts);
                        });
                    }
                    return Ok(());
                }
                if frame.id == 0x7FF && u16::from_le_bytes([frame.data[0], frame.data[1]]) == 1 {
                    return Ok(());
                }
                // Deliver all continuity hold frames (`0x101`) to the actual fake motor!
                if frame.id == 0x101 {
                    let ts = self.inner.with(|s| s.now()).unwrap_or_else(Instant::now);
                    self.delivered_j1_holds.lock().unwrap().push(ts);
                }
                self.inner.transmit(frame)
            }
            fn receive(&mut self) -> Result<CanFrame, TransportError> {
                self.inner.receive()
            }
            fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
                self.inner.receive_timestamped()
            }
            fn receive_with_rx_time(&mut self) -> Result<(CanFrame, Option<CanRxTime>), TransportError> {
                self.inner.receive_with_rx_time()
            }
            fn flush(&mut self) -> Result<(), TransportError> {
                Ok(())
            }
            fn supports_bounded_flush(&self) -> bool {
                self.inner.supports_bounded_flush()
            }
            fn supports_nonblocking_rt(&self) -> bool {
                self.inner.supports_nonblocking_rt()
            }
            fn flush_is_noop(&self) -> bool {
                true
            }
        }

        let clock = MockClock::new();
        let inner = FakeBus::with_clock(&limits, clock.clone());
        inner.with(|s| {
            s.poll_feeds_watchdog = false;
            s.immediate_replies = true;
            s.tx_step = Duration::from_micros(50);
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = rest[i];
                m.v = 0.0;
                m.enabled = i != 3;
                if i == 3 {
                    m.state_override = Some(0xD);
                }
            }
            s.push_all_feedback();
        });
        let delivered_j1_holds = Arc::new(Mutex::new(Vec::new()));
        let injected_stale = Arc::new(AtomicBool::new(false));
        let bus = StaleZeroDeliverHoldsBus {
            inner: inner.clone(),
            injected_stale_zero_j1: Arc::clone(&injected_stale),
            delivered_j1_holds: Arc::clone(&delivered_j1_holds),
        };
        let mut drv = build_standalone_mit_recovery_driver(bus, &rest);
        for i in 0..7 {
            if i != 3 {
                drv.set_axis_confirmed_mode(i, Some(armd::motor_mode::ControlMode::PositionVelocity));
            }
        }
        drv.read_joint_states().unwrap();
        for rel in 1..=3 {
            clock.advance(Duration::from_millis(25));
            drv.begin_control_release(rel);
            inner.with(|s| s.push_all_feedback());
            drv.read_joint_states().unwrap();
        }
        let t_start = clock.now();
        let mut gate = || Ok(());
        let res = drv.mit_expiry_recover_with_gate(&mut gate);
        let t_end = clock.now();
        assert!(res.is_err(), "queued J1 disable must fail closed after phase window");
        assert!(injected_stale.load(Ordering::Acquire), "stale state==0 must have been injected");
        let mut times = delivered_j1_holds.lock().unwrap().clone();
        assert!(
            times.len() >= 9,
            "stale state==0 MUST NOT stop continuity holds delivered to J1 motor, got {} holds",
            times.len()
        );
        times.insert(0, t_start);
        times.push(t_end);
        let max_gap = times
            .windows(2)
            .map(|w| w[1].saturating_duration_since(w[0]))
            .max()
            .unwrap_or(Duration::ZERO);
        assert!(
            max_gap <= Duration::from_millis(40),
            "J1 delivered wire hold gap exceeded 40 ms after stale state==0: {max_gap:?}"
        );
        assert!(
            inner.with(|s| s.motors[0].enabled && s.motors[0].state_override.is_none()),
            "J1 must remain cleanly enabled on wire with zero 0xD watchdog expiry"
        );
    }

    // Subcase D (R2 Major 2): Clock advances during BOTH RX (500 us/frame) AND TX (500 us/frame),
    // with a burst of pre-seeded RX frames forcing multi-pass initial boundary capture while
    // disable is queued across the entire 200 ms phase window -> assert EVERY energized axis
    // maintains per-axis hold-to-hold gap <= 20 ms throughout!
    {
        let clock = MockClock::new();
        let inner = FakeBus::with_clock(&limits, clock.clone());
        inner.with(|s| {
            s.poll_feeds_watchdog = false;
            s.immediate_replies = true;
            s.tx_step = Duration::from_micros(500);
            s.rx_step = Duration::from_micros(500);
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = rest[i];
                m.v = 0.0;
                m.enabled = i != 3;
                if i == 3 {
                    m.state_override = Some(0xD);
                }
            }
            s.push_all_feedback();
        });
        let recorded_tx = Arc::new(Mutex::new(Vec::new()));
        let queue_all = Arc::new(AtomicBool::new(false));
        let bus = ContinuityTestBus {
            inner: inner.clone(),
            queue_all: Arc::clone(&queue_all),
            drop_j1_disable_remaining: Arc::new(Mutex::new(0)),
            drop_j1_barrier_once: Arc::new(AtomicBool::new(false)),
            recorded_tx: Arc::clone(&recorded_tx),
        };
        let mut drv = build_standalone_mit_recovery_driver(bus, &rest);
        for i in 0..7 {
            if i != 3 {
                drv.set_axis_confirmed_mode(i, Some(armd::motor_mode::ControlMode::PositionVelocity));
            }
        }
        drv.read_joint_states().unwrap();
        for rel in 1..=3 {
            clock.advance(Duration::from_millis(10));
            drv.begin_control_release(rel);
            inner.with(|s| s.push_all_feedback());
            drv.read_joint_states().unwrap();
        }
        // Pre-seed 24 frames in RX right before entry so initial boundary capture exercises
        // multiple bounded drain passes with clock advancing 500 us per RX frame (12 ms total).
        inner.with(|s| {
            for _ in 0..4 {
                s.push_all_feedback();
            }
        });
        queue_all.store(true, Ordering::Release);
        let t_start = clock.now();
        let mut gate = || Ok(());
        let res = drv.mit_expiry_recover_with_gate(&mut gate);
        let t_end = clock.now();
        assert!(res.is_err(), "queued disable must fail closed after phase window");
        let rec = recorded_tx.lock().unwrap();
        for motor_id in [1u16, 2, 3, 5, 6] {
            let (holds, max_gap) = check_axis_max_gap(&rec, motor_id, t_start, t_end);
            assert!(
                holds >= 10,
                "energized J{motor_id} under RX/TX clock advance must receive continuous holds, got {holds}"
            );
            assert!(
                max_gap <= Duration::from_millis(20),
                "J{motor_id} per-axis hold gap with RX/TX clock advance exceeded 20 ms: {max_gap:?}"
            );
        }
    }
}

// ============================================================================
// 28. R2 Review Regressions: Duplicate Round 2 replies before hold, post-ack
//     extra reply, executor initial-drain provisional-zero guard, and per-axis
//     pre-probe RX drain.
// ============================================================================

#[test]
fn mit_expiry_recovery_r2_ambiguity_initial_drain_and_per_axis_drain() {
    use armd::transport::{CanRxTime, CanTransport, DriverClock, RxClockSample, TransportError};

    let cfg = mit_config_j4();
    let rest = rest_pose_of(&cfg);
    let limits = b601_limits();

    // (a) R2 Blocker 1 (Path 1): Two Round 2 replies arrive BEFORE a continuity hold, and RX is
    //     drained AFTER the hold. The hold MUST NOT waive the duplicate check -> fails closed &
    //     resets the axis back to CMD_DISABLE!
    {
        #[derive(Clone)]
        struct HoldWireBus {
            inner: FakeBus,
            pending_wire: Arc<Mutex<VecDeque<CanFrame>>>,
            hold_wire: Arc<AtomicBool>,
        }
        impl CanTransport for HoldWireBus {
            fn clock(&self) -> Option<Arc<dyn DriverClock>> { self.inner.clock() }
            fn rx_clock_now(&self) -> Option<CanRxTime> { self.inner.rx_clock_now() }
            fn rx_clock_sample(&self) -> Option<RxClockSample> { self.inner.rx_clock_sample() }
            fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
                if self.hold_wire.load(Ordering::Acquire) {
                    self.inner.with(|s| {
                        if let Some(c) = &s.clock { c.advance(s.tx_step); }
                    });
                    self.pending_wire.lock().unwrap().push_back(*frame);
                    return Ok(());
                }
                self.inner.transmit(frame)
            }
            fn receive(&mut self) -> Result<CanFrame, TransportError> { self.inner.receive() }
            fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
                self.inner.receive_timestamped()
            }
            fn receive_with_rx_time(&mut self) -> Result<(CanFrame, Option<CanRxTime>), TransportError> {
                self.inner.receive_with_rx_time()
            }
            fn flush(&mut self) -> Result<(), TransportError> { Ok(()) }
            fn supports_bounded_flush(&self) -> bool { self.inner.supports_bounded_flush() }
            fn supports_nonblocking_rt(&self) -> bool { self.inner.supports_nonblocking_rt() }
            fn flush_is_noop(&self) -> bool { true }
        }

        let clock = MockClock::new();
        let inner = FakeBus::with_clock(&limits, clock.clone());
        inner.with(|s| {
            s.poll_feeds_watchdog = false;
            s.immediate_replies = true;
            s.tx_step = Duration::from_micros(50);
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = rest[i];
                m.v = 0.0;
                m.enabled = i != 3;
                if i == 3 { m.state_override = Some(0xD); }
            }
            s.push_all_feedback();
        });
        let pending_wire = Arc::new(Mutex::new(VecDeque::new()));
        let hold_wire = Arc::new(AtomicBool::new(true));
        let bus = HoldWireBus {
            inner: inner.clone(),
            pending_wire: Arc::clone(&pending_wire),
            hold_wire: Arc::clone(&hold_wire),
        };
        let mut drv = build_standalone_mit_recovery_driver(bus, &rest);
        for i in 0..7 {
            if i != 3 {
                drv.set_axis_confirmed_mode(i, Some(armd::motor_mode::ControlMode::PositionVelocity));
            }
        }
        drv.read_joint_states().unwrap();
        for rel in 1..=3 {
            clock.advance(Duration::from_millis(25));
            drv.begin_control_release(rel);
            inner.with(|s| s.push_all_feedback());
            drv.read_joint_states().unwrap();
        }
        let started_at = clock.now();
        let mut gate = || Ok(());

        // Step 1: Queue ClearFault + CMD_DISABLE + barrier; deliver on wire.
        drv.begin_control_release(4);
        drv.read_joint_states().unwrap();
        let dl4 = clock.now() + Duration::from_millis(500);
        drv.set_tx_release_deadline(Some(dl4));
        assert!(!drv.mit_expiry_recover_budgeted(dl4, started_at, &mut gate).unwrap());
        drv.set_tx_release_deadline(None);
        for f in pending_wire.lock().unwrap().drain(..).collect::<Vec<_>>() {
            inner.clone().transmit(&f).unwrap();
        }

        // Step 2: Harvest barrier -> queue Round 1 0xCC; deliver on wire.
        clock.advance(Duration::from_millis(2));
        drv.begin_control_release(5);
        drv.read_joint_states().unwrap();
        let dl5 = clock.now() + Duration::from_millis(500);
        drv.set_tx_release_deadline(Some(dl5));
        assert!(!drv.mit_expiry_recover_budgeted(dl5, started_at, &mut gate).unwrap());
        drv.set_tx_release_deadline(None);
        for f in pending_wire.lock().unwrap().drain(..).collect::<Vec<_>>() {
            inner.clone().transmit(&f).unwrap();
        }

        // Step 3: Harvest Round 1 -> queue Round 2 0xCC (leave queued on wire!).
        clock.advance(Duration::from_millis(2));
        drv.begin_control_release(6);
        drv.read_joint_states().unwrap();
        let dl6 = clock.now() + Duration::from_millis(500);
        drv.set_tx_release_deadline(Some(dl6));
        assert!(!drv.mit_expiry_recover_budgeted(dl6, started_at, &mut gate).unwrap());
        drv.set_tx_release_deadline(None);
        let p2_cc = pending_wire
            .lock()
            .unwrap()
            .iter()
            .filter(|f| f.id == 0x7FF && f.data[2] == 0xCC)
            .count();
        assert_eq!(p2_cc, 7, "Round 2 must queue 7 0xCC probes");
        pending_wire.lock().unwrap().clear();

        // Inject TWO state==0 replies for J1 (at T0+1ms and T0+2ms) BEFORE a continuity hold!
        clock.advance(Duration::from_millis(1));
        let t_rep1 = clock.now();
        clock.advance(Duration::from_millis(1));
        let t_rep2 = clock.now();
        inner.with(|s| {
            let frames: Vec<CanFrame> = s.motors.iter().map(|m| m.feedback_frame()).collect();
            for (idx, fb) in frames.into_iter().enumerate() {
                s.push_rx_at(fb, Some(t_rep1));
                if idx == 0 {
                    s.push_rx_at(fb, Some(t_rep2));
                }
            }
        });

        // Now send a continuity hold to J1 at T0+12ms (strictly AFTER BOTH replies!) BEFORE RX is drained.
        clock.advance(Duration::from_millis(10));
        drv.begin_control_release(61);
        let hold_dl = clock.now() + Duration::from_millis(50);
        let hold_res = drv
            .mode_switch_service_watchdog_holds(Duration::from_millis(1), 0, 0x01, hold_dl, 1)
            .unwrap();
        assert!(hold_res.refreshed_mask & 0x01 != 0, "continuity hold must be emitted for J1");

        // Now drain RX AFTER the hold and invoke `mit_expiry_recover_budgeted` -> MUST NOT confirm!
        clock.advance(Duration::from_millis(2));
        drv.begin_control_release(7);
        drv.read_joint_states().unwrap();
        let dl7 = clock.now() + Duration::from_millis(500);
        drv.set_tx_release_deadline(Some(dl7));
        let r7 = drv.mit_expiry_recover_budgeted(dl7, started_at, &mut gate).unwrap();
        drv.set_tx_release_deadline(None);
        assert!(
            !r7,
            "two Round 2 replies arriving before a continuity hold MUST fail closed as ambiguous"
        );
        let j1_disables_queued = pending_wire
            .lock()
            .unwrap()
            .iter()
            .filter(|f| f.id == 1 && f.data == damiao::CMD_DISABLE)
            .count();
        assert!(
            j1_disables_queued >= 1,
            "ambiguous J1 must reset all the way back to CMD_DISABLE, got {j1_disables_queued}"
        );
    }

    // (b) R2 Blocker 1 (Path 2): Extra reply arriving for J1 AFTER J1 already latched `probe2_ack`
    //     while J4 is still pending MUST invalidate J1 and force J1 to re-confirm from CMD_DISABLE.
    {
        #[derive(Clone)]
        struct DropBarrierOnceBus {
            inner: FakeBus,
            drop_j4_barrier_once: Arc<AtomicBool>,
        }
        impl CanTransport for DropBarrierOnceBus {
            fn clock(&self) -> Option<Arc<dyn DriverClock>> { self.inner.clock() }
            fn rx_clock_now(&self) -> Option<CanRxTime> { self.inner.rx_clock_now() }
            fn rx_clock_sample(&self) -> Option<RxClockSample> { self.inner.rx_clock_sample() }
            fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
                if frame.id == 0x7FF
                    && u16::from_le_bytes([frame.data[0], frame.data[1]]) == 4
                    && frame.data[2] == damiao::REG_READ
                    && frame.data[3] == damiao::RID_CTRL_MODE
                    && self.drop_j4_barrier_once.swap(false, Ordering::AcqRel)
                {
                    self.inner.with(|s| {
                        if let Some(c) = &s.clock { c.advance(s.tx_step); }
                    });
                    return Ok(());
                }
                self.inner.transmit(frame)
            }
            fn receive(&mut self) -> Result<CanFrame, TransportError> { self.inner.receive() }
            fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
                self.inner.receive_timestamped()
            }
            fn receive_with_rx_time(&mut self) -> Result<(CanFrame, Option<CanRxTime>), TransportError> {
                self.inner.receive_with_rx_time()
            }
            fn flush(&mut self) -> Result<(), TransportError> { Ok(()) }
            fn supports_bounded_flush(&self) -> bool { self.inner.supports_bounded_flush() }
            fn supports_nonblocking_rt(&self) -> bool { self.inner.supports_nonblocking_rt() }
            fn flush_is_noop(&self) -> bool { true }
        }

        let clock = MockClock::new();
        let inner = FakeBus::with_clock(&limits, clock.clone());
        inner.with(|s| {
            s.poll_feeds_watchdog = false;
            s.immediate_replies = true;
            s.tx_step = Duration::from_micros(50);
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = rest[i];
                m.v = 0.0;
                m.enabled = i != 3;
                if i == 3 { m.state_override = Some(0xD); }
            }
            s.push_all_feedback();
        });
        let bus = DropBarrierOnceBus {
            inner: inner.clone(),
            drop_j4_barrier_once: Arc::new(AtomicBool::new(true)),
        };
        let mut drv = build_standalone_mit_recovery_driver(bus, &rest);
        drv.read_joint_states().unwrap();
        let started_at = clock.now();
        for rel in 1..=3 {
            clock.advance(Duration::from_millis(25));
            drv.begin_control_release(rel);
            inner.with(|s| s.push_all_feedback());
            drv.read_joint_states().unwrap();
        }
        inner.clear_log();
        // Releases 4..=5: J1..J3, J5..J7 complete ClearFault + Disable + Barrier + Round 1 +
        // Round 2 (`probe2_ack` latched), while J4's barrier reply is dropped.
        let mut gate = || Ok(());
        for rel in 4..=5 {
            clock.advance(Duration::from_millis(2));
            drv.begin_control_release(rel);
            drv.read_joint_states().unwrap();
            let dl = clock.now() + Duration::from_millis(500);
            drv.set_tx_release_deadline(Some(dl));
            assert!(!drv.mit_expiry_recover_budgeted(dl, started_at, &mut gate).unwrap());
            drv.set_tx_release_deadline(None);
        }
        let j1_cc_before_extra = inner.with(|s| {
            s.tx_log
                .iter()
                .filter(|f| f.id == 0x7FF && u16::from_le_bytes([f.data[0], f.data[1]]) == 1 && f.data[2] == 0xCC)
                .count()
        });
        assert_eq!(
            j1_cc_before_extra, 2,
            "J1 must have completed both Round 1 and Round 2 probes (probe2_ack latched) before extra reply"
        );

        // Inject ONE extra unexpected `state == 0` frame for J1 (no hold sent to J1!) while J4 is pending.
        clock.advance(Duration::from_millis(2));
        let extra_ts = clock.now();
        inner.with(|s| {
            let fb = s.motors[0].feedback_frame();
            s.push_rx_at(fb, Some(extra_ts));
        });

        // Advance past J4's 50 ms barrier timeout and finish recovery.
        clock.advance(Duration::from_millis(52));
        let mut done = false;
        for rel in 6..=10 {
            clock.advance(Duration::from_millis(2));
            drv.begin_control_release(rel);
            drv.read_joint_states().unwrap();
            let dl = clock.now() + Duration::from_millis(500);
            drv.set_tx_release_deadline(Some(dl));
            if drv.mit_expiry_recover_budgeted(dl, started_at, &mut gate).unwrap() {
                done = true;
                break;
            }
            drv.set_tx_release_deadline(None);
        }
        drv.set_tx_release_deadline(None);
        assert!(done, "recovery must complete after invalidating and re-confirming J1");
        let j1_disables = inner.with(|s| {
            s.tx_log.iter().filter(|f| f.id == 1 && f.data == damiao::CMD_DISABLE).count()
        });
        assert!(
            j1_disables >= 2,
            "extra reply after J1 probe2_ack MUST invalidate J1 and force fresh CMD_DISABLE, got {j1_disables}"
        );
    }

    // (c) R2 Major 1: Stale `state == 0` decoded during executor's initial (partial) drain MUST NOT
    //     clear `motor_energized_mask` or `motor_last_ctrl_tx` on an energized axis.
    {
        let clock = MockClock::new();
        let inner = FakeBus::with_clock(&limits, clock.clone());
        inner.with(|s| {
            s.poll_feeds_watchdog = false;
            s.immediate_replies = true;
            s.tx_step = Duration::from_micros(50);
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = rest[i];
                m.v = 0.0;
                m.enabled = i != 3;
                if i == 3 { m.state_override = Some(0xD); }
            }
            s.push_all_feedback();
        });
        let mut drv = build_standalone_mit_recovery_driver(inner.clone(), &rest);
        for i in 0..7 {
            if i != 3 {
                drv.set_axis_confirmed_mode(i, Some(armd::motor_mode::ControlMode::PositionVelocity));
            }
        }
        drv.read_joint_states().unwrap();
        for rel in 1..=3 {
            clock.advance(Duration::from_millis(25));
            drv.begin_control_release(rel);
            inner.with(|s| s.push_all_feedback());
            drv.read_joint_states().unwrap();
        }
        assert!(drv.axis_last_ctrl_tx(0).is_some(), "J1 must start with energized ctrl_tx");

        // Pre-seed a stale `state == 0` frame for J1 plus > RX_FRAME_BUDGET frames so the initial
        // boundary drain in `mit_expiry_recover_budgeted` decodes the stale `state == 0` and yields
        // mid-drain BEFORE `mit_expiry_budgeted_boundary` is set!
        clock.advance(Duration::from_millis(5));
        let stale_ts = clock.now();
        inner.with(|s| {
            let prev = s.motors[0].enabled;
            s.motors[0].enabled = false;
            let stale_zero = s.motors[0].feedback_frame();
            s.motors[0].enabled = prev;
            s.push_rx_at(stale_zero, Some(stale_ts));
            // Push enough extra frames (> RX_FRAME_BUDGET = 64) to force a real partial drain
            // across release RX budget BEFORE `mit_expiry_budgeted_boundary` can be established.
            for _ in 0..80 {
                let fb = s.motors[1].feedback_frame();
                s.push_rx_at(fb, Some(stale_ts));
            }
        });
        inner.clear_log();
        let started_at = clock.now();
        drv.begin_control_release(4);
        let dl4 = clock.now() + Duration::from_millis(500);
        let mut gate = || Ok(());
        drv.set_tx_release_deadline(Some(dl4));
        let r4 = drv.mit_expiry_recover_budgeted(dl4, started_at, &mut gate).unwrap();
        drv.set_tx_release_deadline(None);
        assert!(!r4, "partial initial drain must yield cleanly");
        assert!(
            !inner.tx_log().iter().any(|f| {
                f.data == damiao::CMD_CLEAR_FAULTS
                    || f.data == damiao::CMD_DISABLE
                    || f.id == 0x7FF
            }),
            "partial initial drain must yield BEFORE sending any ClearFault/CMD_DISABLE/probe TX, got {:?}",
            inner.tx_log()
        );
        assert!(
            drv.axis_last_ctrl_tx(0).is_some(),
            "stale state==0 decoded during executor initial partial drain MUST NOT clear J1 motor_last_ctrl_tx"
        );
        assert!(
            drv.axis_is_energized(0),
            "stale state==0 decoded during executor initial partial drain MUST NOT clear J1 motor_energized_mask"
        );
        clock.advance(Duration::from_millis(20));
        drv.begin_control_release(5);
        let hold_dl = clock.now() + Duration::from_millis(50);
        let hold_res = drv
            .mode_switch_service_watchdog_holds(Duration::from_millis(10), 0, 0, hold_dl, 6)
            .unwrap();
        assert!(
            hold_res.refreshed_mask & 0x01 != 0,
            "J1 must remain eligible for continuity holds after stale state==0 in initial drain: hold_res={hold_res:?}"
        );
    }

    // (d) R2 Major 3: A queued frame for J2 arriving during J1's Round 1 submission MUST be drained
    //     immediately before J2's probe so J2 does NOT false-trip as ambiguous.
    {
        #[derive(Clone)]
        struct InjectJ2OnJ1ProbeBus {
            inner: FakeBus,
            injected: Arc<AtomicBool>,
        }
        impl CanTransport for InjectJ2OnJ1ProbeBus {
            fn clock(&self) -> Option<Arc<dyn DriverClock>> { self.inner.clock() }
            fn rx_clock_now(&self) -> Option<CanRxTime> { self.inner.rx_clock_now() }
            fn rx_clock_sample(&self) -> Option<RxClockSample> { self.inner.rx_clock_sample() }
            fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
                self.inner.transmit(frame)?;
                if frame.id == 0x7FF
                    && u16::from_le_bytes([frame.data[0], frame.data[1]]) == 1
                    && frame.data[2] == 0xCC
                    && !self.injected.swap(true, Ordering::AcqRel)
                {
                    // Queue a stale `state == 0` frame for J2 right as J1's Round 1 0xCC transmits!
                    self.inner.with(|s| {
                        let ts = s.now();
                        let fb = s.motors[1].feedback_frame();
                        s.push_rx_at(fb, ts);
                    });
                }
                Ok(())
            }
            fn receive(&mut self) -> Result<CanFrame, TransportError> { self.inner.receive() }
            fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
                self.inner.receive_timestamped()
            }
            fn receive_with_rx_time(&mut self) -> Result<(CanFrame, Option<CanRxTime>), TransportError> {
                self.inner.receive_with_rx_time()
            }
            fn flush(&mut self) -> Result<(), TransportError> { Ok(()) }
            fn supports_bounded_flush(&self) -> bool { self.inner.supports_bounded_flush() }
            fn supports_nonblocking_rt(&self) -> bool { self.inner.supports_nonblocking_rt() }
            fn flush_is_noop(&self) -> bool { true }
        }

        let clock = MockClock::new();
        let inner = FakeBus::with_clock(&limits, clock.clone());
        inner.with(|s| {
            s.poll_feeds_watchdog = false;
            s.immediate_replies = true;
            s.tx_step = Duration::from_micros(50);
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = rest[i];
                m.v = 0.0;
                m.enabled = i != 3;
                if i == 3 { m.state_override = Some(0xD); }
            }
            s.push_all_feedback();
        });
        let injected = Arc::new(AtomicBool::new(false));
        let bus = InjectJ2OnJ1ProbeBus {
            inner: inner.clone(),
            injected: Arc::clone(&injected),
        };
        let mut drv = build_standalone_mit_recovery_driver(bus, &rest);
        drv.read_joint_states().unwrap();
        for rel in 1..=3 {
            clock.advance(Duration::from_millis(25));
            drv.begin_control_release(rel);
            inner.with(|s| s.push_all_feedback());
            drv.read_joint_states().unwrap();
        }
        inner.clear_log();
        clock.advance(Duration::from_millis(5));
        let mut gate = || Ok(());
        drv.mit_expiry_recover_with_gate(&mut gate)
            .expect("per-axis pre-probe drain must clear queued J2 frame before J2's probe");
        assert!(injected.load(Ordering::Acquire), "J2 frame must have been injected during J1 probe");
        let j2_disables = inner.with(|s| {
            s.tx_log.iter().filter(|f| f.id == 2 && f.data == damiao::CMD_DISABLE).count()
        });
        assert_eq!(
            j2_disables, 1,
            "J2 must confirm on first pass without spurious ambiguity reset when drained per-axis"
        );
    }
}

// ============================================================================
// 26. Drag-teach exit workspace gate, obs fields, and unblocked safety exits
// ============================================================================

#[test]
fn gc_outside_workspace_evaluator_covers_box_reach_floor_keepout() {
    use armd::chain::{write_gc_exit_outside_workspace_message, EvalBufs, WorkspaceEvaluator};
    use armd::model::CameraMount;

    let cfg = ArmConfig::rebot_dm();
    let cam = CameraMount::default();
    let incident = [0.0f32, -2.046, -1.855, -0.297, -0.053, 0.123, 0.0];
    let safe_limit_outside = [0.0f32, -1.95, -1.855, -0.297, -0.053, 0.123, 0.0];
    let eval_pose = |ws: &armd::model::WorkspacePolicy, q: &[f32]| {
        let mut bufs = EvalBufs::default();
        bufs.prepare_for(&cfg.arm_model, ws, &cam, cfg.tcp_offset);
        let eval = WorkspaceEvaluator::new_with_tcp_in(&cfg.arm_model, ws, &cam, cfg.tcp_offset, &mut bufs);
        let (mut clr, mut world_caps) = (vec![0.0; eval.total_slots], Vec::new());
        eval.deepest_workspace_outside_rt(q, &mut clr, &mut world_caps).unwrap()
    };
    let fmt = |info: &armd::chain::WorkspaceOutsideInfo| {
        let mut msg = String::new();
        write_gc_exit_outside_workspace_message(&mut msg, info);
        msg
    };

    let ws = cfg.workspace.clone().unwrap();
    assert!(eval_pose(&ws, &INSIDE_RAISED_POSE).is_none());
    for pose in [incident, safe_limit_outside] {
        let info = eval_pose(&ws, &pose).expect("pose outside workspace");
        assert!(info.depth_m > 0.01);
        let msg = fmt(&info);
        assert!(msg.contains(info.edge.as_str()) && msg.contains("cm"), "{msg}");
    }
    let mut ws_wide = ws.clone();
    ws_wide.box_min_m = [-2.0; 3];
    ws_wide.box_max_m = [2.0; 3];
    let info_reach = eval_pose(&ws_wide, &incident).expect("max_reach");
    assert_eq!(info_reach.edge.as_str(), "max_reach");
    assert!(fmt(&info_reach).contains("move closer to the base"));

    let mut ws_floor = ws_wide.clone();
    ws_floor.max_reach_m = None;
    ws_floor.floor_z_m = 0.80;
    let info_floor = eval_pose(&ws_floor, &INSIDE_RAISED_POSE).expect("floor_z");
    assert_eq!(info_floor.edge.as_str(), "floor_z");
    assert!(fmt(&info_floor).contains("move up"));

    let mut ws_ko = ws_wide.clone();
    ws_ko.max_reach_m = None;
    ws_ko.keepout_boxes.push(KeepoutBox { name: "camera".into(), min_m: [-1.0; 3], max_m: [1.0; 3] });
    let info_ko = eval_pose(&ws_ko, &INSIDE_RAISED_POSE).expect("keepout");
    assert_eq!(info_ko.edge.as_str(), "keepout:camera");
    assert!(fmt(&info_ko).contains("move clear"));

    let mut ws_box = ws_wide;
    ws_box.max_reach_m = None;
    ws_box.box_max_m[0] = 0.05;
    let info_box = eval_pose(&ws_box, &incident).expect("box_max_x");
    assert_eq!(info_box.edge.as_str(), "box_max_x");
    let box_msg = fmt(&info_box);
    assert!(box_msg.contains("front edge (box_max_x)") && box_msg.contains("move back"), "{box_msg}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_gc_exit_outside_workspace_obs_and_safety_exits() {
    // J2 = -1.95 stays inside the 0.05 rad joint_limit_margin (-2.0944 + 0.05 = -2.0444)
    // while extending past the B601-DM workspace boundary.
    const OUTSIDE_POSE: [f32; 7] = [0.0, -1.95, -1.855, -0.297, -0.053, 0.123, -0.14];

    let set_pose = |bus: &FakeBus, q: [f32; 7]| {
        bus.with(|s| {
            for (i, m) in s.motors.iter_mut().enumerate() { m.q = q[i]; m.v = 0.0; }
            s.push_all_feedback();
        });
    };
    let wait_gc_active = |sup: Arc<ArmSupervisor>| async move {
        let dl = Instant::now() + Duration::from_secs(3);
        loop {
            let o = sup.get_observation().await;
            if o.control_mode == ControlMode::GravityComp && o.gravity_comp.phase.as_deref() == Some("active") { return o; }
            assert!(Instant::now() < dl, "gc active timeout: {:?}", o.gravity_comp);
            tokio::time::sleep(Duration::from_millis(15)).await;
        }
    };
    let wait_position = |sup: Arc<ArmSupervisor>| async move {
        let dl = Instant::now() + Duration::from_secs(3);
        loop {
            let o = sup.get_observation().await;
            if o.control_mode == ControlMode::Position && o.gravity_comp.phase.is_none() { return o; }
            assert!(Instant::now() < dl, "position handback timeout: {:?}", o.gravity_comp);
            tokio::time::sleep(Duration::from_millis(15)).await;
        }
    };

    let mut parity = Vec::new();
    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let rig = LiveRig::start(owner, ArmConfig::rebot_dm(), INSIDE_RAISED_POSE, OperatorAuth::disabled(), None, true).await;
        let mut ws = ws_connect(&rig.ws_url).await;
        let granted = ws_send_wait_types(&mut ws, json!({"type": "acquire_lease", "client_id": "gc-ws-gate", "seq": 1}), &["lease_granted"]).await;
        let mut gen = granted["generation"].as_u64().unwrap();
        let mut req_id = 1u64;

        // 1. Enter inside -> obs reports workspace_outside=false.
        let epoch = rig.sup.get_observation().await.epoch_id;
        let ack = ws_rpc(&mut ws, req_id, gen, epoch, "/api/v1/mode/gravity_comp", json!({"enable": true})).await;
        req_id += 1;
        assert_eq!(ack["type"], "command_ack", "{owner:?}: {ack}");
        let obs_in = wait_gc_active(Arc::clone(&rig.sup)).await;
        assert!(!obs_in.gravity_comp.workspace_outside);
        assert_eq!((obs_in.gravity_comp.workspace_edge, obs_in.gravity_comp.workspace_depth_m), (None, None));

        // 2. Drag outside -> obs reports outside + edge + depth; user exit refused with GC_EXIT_OUTSIDE_WORKSPACE.
        set_pose(&rig.bus, OUTSIDE_POSE);
        let dl = Instant::now() + Duration::from_secs(2);
        let obs_out = loop {
            let o = rig.sup.get_observation().await;
            if o.gravity_comp.workspace_outside { break o; }
            assert!(Instant::now() < dl, "{owner:?}: obs never reported workspace_outside");
            tokio::time::sleep(Duration::from_millis(15)).await;
        };
        let edge = obs_out.gravity_comp.workspace_edge.expect("edge").as_str().to_string();
        assert!(obs_out.gravity_comp.workspace_depth_m.expect("depth") > 0.01);

        let rej = ws_rpc(&mut ws, req_id, gen, obs_out.epoch_id, "/api/v1/mode/gravity_comp", json!({"enable": false})).await;
        req_id += 1;
        assert_eq!((rej["type"].as_str(), rej["status"].as_u64(), rej["code"].as_str()), (Some("command_reject"), Some(409), Some("GC_EXIT_OUTSIDE_WORKSPACE")), "{owner:?}: {rej}");
        let msg = rej["message"].as_str().unwrap_or("");
        assert!(msg.contains(&edge) && (msg.contains("cm") || msg.contains("mm")), "{owner:?}: {msg}");
        assert_eq!(rig.sup.get_observation().await.control_mode, ControlMode::GravityComp);
        parity.push((edge, rej["code"].as_str().unwrap().to_string()));

        // 3. Drag back inside -> obs clears, user exit succeeds cleanly.
        set_pose(&rig.bus, INSIDE_RAISED_POSE);
        let dl = Instant::now() + Duration::from_secs(2);
        while rig.sup.get_observation().await.gravity_comp.workspace_outside {
            assert!(Instant::now() < dl, "{owner:?}: workspace_outside did not clear");
            tokio::time::sleep(Duration::from_millis(15)).await;
        }
        let obs_back = rig.sup.get_observation().await;
        assert_eq!((obs_back.gravity_comp.workspace_edge, obs_back.gravity_comp.workspace_depth_m), (None, None));
        let ack = ws_rpc(&mut ws, req_id, gen, obs_back.epoch_id, "/api/v1/mode/gravity_comp", json!({"enable": false})).await;
        req_id += 1;
        assert_eq!(ack["type"], "command_ack", "{owner:?}: {ack}");
        let obs_pos = wait_position(Arc::clone(&rig.sup)).await;
        assert_eq!(obs_pos.gravity_comp.last_exit.as_ref().map(|e| e.code.as_str()), Some("REQUESTED"));

        // 4–7. Safety exits while outside workspace: Stop, E-stop, Lease loss, Hardware fault.
        for kind in ["STOPPED", "ESTOP", "LEASE_LOST", "FAULT"] {
            set_pose(&rig.bus, INSIDE_RAISED_POSE);
            tokio::time::sleep(Duration::from_millis(40)).await;
            let ep = rig.sup.get_observation().await.epoch_id;
            let ack = ws_rpc(&mut ws, req_id, gen, ep, "/api/v1/mode/gravity_comp", json!({"enable": true})).await;
            req_id += 1;
            assert_eq!(ack["type"], "command_ack", "{owner:?} {kind}: {ack}");
            wait_gc_active(Arc::clone(&rig.sup)).await;
            set_pose(&rig.bus, OUTSIDE_POSE);
            tokio::time::sleep(Duration::from_millis(60)).await;
            assert!(rig.sup.get_observation().await.gravity_comp.workspace_outside, "{owner:?} {kind}");
            match kind {
                "STOPPED" => {
                    let ack = ws_rpc(&mut ws, req_id, gen, rig.sup.get_observation().await.epoch_id, "/api/v1/motion/stop", json!({})).await;
                    req_id += 1;
                    assert_eq!(ack["type"], "command_ack", "{owner:?}: {ack}");
                    assert_eq!(wait_position(Arc::clone(&rig.sup)).await.gravity_comp.last_exit.unwrap().code.as_str(), "STOPPED");
                }
                "ESTOP" => {
                    ws_send_wait_types(&mut ws, json!({"type": "estop"}), &["epoch_sync"]).await;
                    let clr = ws_rpc(&mut ws, req_id, gen, rig.sup.get_observation().await.epoch_id, "/api/v1/safety/clear_fault", json!({})).await;
                    req_id += 1;
                    assert_eq!(clr["type"], "command_ack", "{owner:?}: {clr}");
                    let obs_es = wait_position(Arc::clone(&rig.sup)).await;
                    assert_eq!(obs_es.gravity_comp.last_exit.unwrap().code.as_str(), "ESTOP");
                    let rearm = ws_rpc(&mut ws, req_id, gen, obs_es.epoch_id, "/api/v1/torque", json!({"enable": true})).await;
                    req_id += 1;
                    assert_eq!(rearm["type"], "command_ack", "{owner:?}: {rearm}");
                }
                "LEASE_LOST" => {
                    ws_send_wait_types(&mut ws, json!({"type": "release_lease", "generation": gen, "seq": 99}), &["lease_released"]).await;
                    assert_eq!(wait_position(Arc::clone(&rig.sup)).await.gravity_comp.last_exit.unwrap().code.as_str(), "LEASE_LOST");
                    let g2 = ws_send_wait_types(&mut ws, json!({"type": "acquire_lease", "client_id": "gc-ws-gate-2", "seq": 100}), &["lease_granted"]).await;
                    gen = g2["generation"].as_u64().unwrap();
                }
                _ => {
                    rig.bus.with(|s| { s.motors[0].state_override = Some(0x8); s.push_all_feedback(); });
                    let dl = Instant::now() + Duration::from_secs(2);
                    loop {
                        let o = rig.sup.get_observation().await;
                        if o.gravity_comp.last_exit.as_ref().is_some_and(|e| e.code.as_str() == "FAULT") { break; }
                        assert!(Instant::now() < dl, "{owner:?}: fault exit stalled: {:?}", o.gravity_comp);
                        tokio::time::sleep(Duration::from_millis(15)).await;
                    }
                    rig.bus.with(|s| { s.motors[0].state_override = None; });
                }
            }
        }
        rig.assert_rt_clean(&format!("gc outside workspace + safety exits {owner:?}"));
        rig.shutdown_emergency().await;

        // 8. Safety exits: Watchdog timeout, owner handback (Executor), and shutdown while outside.
        let mut wd_cfg = ArmConfig::rebot_dm();
        wd_cfg.lease_timeout_ms = 500;
        let rig_wd = LiveRig::start(owner, wd_cfg, INSIDE_RAISED_POSE, OperatorAuth::disabled(), None, true).await;
        let mut ws_wd = ws_connect(&rig_wd.ws_url).await;
        let g = ws_send_wait_types(&mut ws_wd, json!({"type": "acquire_lease", "client_id": "gc-wd", "seq": 1}), &["lease_granted"]).await;
        let g_gen = g["generation"].as_u64().unwrap();
        let ack = ws_rpc(&mut ws_wd, 1, g_gen, rig_wd.sup.get_observation().await.epoch_id, "/api/v1/mode/gravity_comp", json!({"enable": true})).await;
        assert_eq!(ack["type"], "command_ack", "{owner:?}: {ack}");
        let dl = Instant::now() + Duration::from_secs(2);
        while rig_wd.sup.get_observation().await.gravity_comp.phase.as_deref() != Some("active") {
            assert!(Instant::now() < dl);
            let _ = ws_wd.send(Message::Text(json!({"type": "heartbeat", "generation": g_gen}).to_string().into())).await;
            tokio::time::sleep(Duration::from_millis(30)).await;
        }
        set_pose(&rig_wd.bus, OUTSIDE_POSE);
        let obs_wd = wait_position(Arc::clone(&rig_wd.sup)).await;
        assert_eq!(obs_wd.gravity_comp.last_exit.as_ref().map(|e| e.code.as_str()), Some("WATCHDOG"));
        rig_wd.assert_rt_clean(&format!("gc watchdog exit outside {owner:?}"));
        rig_wd.shutdown_emergency().await;

        // 9. Owner handback (Executor -> Legacy) & shutdown while actively floating outside workspace.
        let mut rig_hb = LiveRig::start(owner, ArmConfig::rebot_dm(), INSIDE_RAISED_POSE, OperatorAuth::disabled(), None, true).await;
        let mut ws_hb = ws_connect(&rig_hb.ws_url).await;
        let gh = ws_send_wait_types(&mut ws_hb, json!({"type": "acquire_lease", "client_id": "gc-hb", "seq": 1}), &["lease_granted"]).await;
        let ack = ws_rpc(&mut ws_hb, 1, gh["generation"].as_u64().unwrap(), rig_hb.sup.get_observation().await.epoch_id, "/api/v1/mode/gravity_comp", json!({"enable": true})).await;
        assert_eq!(ack["type"], "command_ack", "{owner:?}: {ack}");
        wait_gc_active(Arc::clone(&rig_hb.sup)).await;
        set_pose(&rig_hb.bus, OUTSIDE_POSE);
        tokio::time::sleep(Duration::from_millis(60)).await;
        assert!(rig_hb.sup.get_observation().await.gravity_comp.workspace_outside);
        if owner == OwnerKind::Executor {
            let (stop_leg, jh_leg) = rig_hb.sup.handback_to_legacy_with_live_loop(Duration::from_secs(2), PERIOD_50HZ).await.expect("handback outside workspace must not be blocked");
            rig_hb.assert_rt_clean("gc handback outside workspace");
            rig_hb.exec = None;
            rig_hb.sup.legacy_safe_hold_control().request_process_exit_disable(Instant::now() + Duration::from_secs(2));
            stop_leg.store(true, Ordering::Release);
            let _ = tokio::task::spawn_blocking(move || jh_leg.join()).await;
        }
        rig_hb.shutdown_emergency().await;
    }
    assert_eq!(parity[0], parity[1], "Legacy vs Executor parity mismatch");
}
