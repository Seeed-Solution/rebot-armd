//! Stage 3 §3 PR B3 proof: zero-alloc LogWriter, deferred safety logging with subscriber enabled,
//! bounded owned fault-state storage, and typed bounded `clear_fault` refusals with identical API text.

mod mit_support;

#[global_allocator]
static ALLOC: armd::diagnostics::alloc::TimedAlloc = armd::diagnostics::alloc::TimedAlloc;

use std::io::{self, Write};
use std::sync::atomic::Ordering;
use std::sync::{Arc, Condvar, Mutex, OnceLock};
use std::time::{Duration, Instant};

use axum::http::StatusCode;
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

use armd::api::rest::ApiError;
use armd::diagnostics::alloc;
use armd::diagnostics::logwriter::{NonBlocking, INLINE_LINE_BYTES};
use armd::driver::{ArmDriver, ArmDriverError, DamiaoArmDriver, DriverCapabilities, EnableState, JointTarget};
use armd::model::JointState;
use armd::protocol::damiao::DamiaoLimits;
use armd::rt_core::executor::ExecutorConfig;
use armd::rt_core::ops::{CoreOps, ServiceBudget};
use armd::safety::{
    BoundedFaultReason, BoundedSafetyState, BoundedText, ClearFaultDriverError, ClearFaultError,
    ClearFaultRefusal, FaultCode, FaultKind, FaultReason, SafetyError, SafetyManager, SafetyState,
    StaticFault, DETAIL_UNAVAILABLE, MAX_RESTING_SUPPORT_ITEMS, SAFETY_LOG_CAPACITY,
};
use armd::transport::{MockClock, TransportError};
use armd::{ArmConfig, ArmError, ArmSupervisor, StopPolicy, SupervisorInner};
use mit_support::FakeBus;

#[derive(Clone, Default)]
struct SharedBuf(Arc<Mutex<Vec<u8>>>);
impl Write for SharedBuf {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> { self.0.lock().unwrap().extend_from_slice(buf); Ok(buf.len()) }
    fn flush(&mut self) -> io::Result<()> { Ok(()) }
}

#[derive(Clone)]
struct GatedSink {
    gate: Arc<(Mutex<bool>, Condvar)>,
    entered: Arc<std::sync::atomic::AtomicBool>,
    out: Arc<Mutex<Vec<u8>>>,
}
impl GatedSink {
    fn new() -> Self {
        Self {
            gate: Arc::new((Mutex::new(false), Condvar::new())),
            entered: Arc::new(std::sync::atomic::AtomicBool::new(false)),
            out: Arc::default(),
        }
    }
}
impl Write for GatedSink {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.entered.store(true, Ordering::Release);
        let (m, cv) = &*self.gate;
        let mut open = m.lock().unwrap();
        while !*open { open = cv.wait(open).unwrap(); }
        self.out.lock().unwrap().extend_from_slice(buf);
        Ok(buf.len())
    }
    fn flush(&mut self) -> io::Result<()> { Ok(()) }
}

struct GlobalLog { writer: NonBlocking, buf: Arc<Mutex<Vec<u8>>>, guard: tokio::sync::Mutex<()> }

fn global_log() -> &'static GlobalLog {
    static LOG: OnceLock<GlobalLog> = OnceLock::new();
    LOG.get_or_init(|| {
        let sink = SharedBuf::default();
        let buf = Arc::clone(&sink.0);
        let writer = NonBlocking::spawn_unregistered("b3-test-log", sink, 256).unwrap();
        let _ = tracing_subscriber::registry()
            .with(tracing_subscriber::EnvFilter::new("armd=info"))
            .with(tracing_subscriber::fmt::layer().with_ansi(false).with_writer(writer.clone()))
            .try_init();
        GlobalLog { writer, buf, guard: tokio::sync::Mutex::new(()) }
    })
}

fn take_global_logs(log: &GlobalLog) -> String {
    assert!(log.writer.flush(Duration::from_secs(2)));
    let mut g = log.buf.lock().unwrap();
    let s = String::from_utf8(g.clone()).unwrap();
    g.clear();
    s
}

fn dm_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm().damiao_ranges.unwrap().iter().map(|r| r.limits()).collect()
}

fn assert_api_clear_fault_text(err: ClearFaultError, expected_msg: &str) {
    let safety_err = SafetyError::from(err);
    assert_eq!(safety_err, SafetyError::ClearFaultRejected(expected_msg.to_string()));
    let api = ApiError::from(ArmError::Safety(safety_err));
    assert_eq!(api.status, StatusCode::CONFLICT);
    assert_eq!(api.error, "ClearFaultRejected");
    assert_eq!(api.code.as_deref(), Some("CLEAR_FAULT_REJECTED"));
    assert_eq!(api.safety_state.as_deref(), Some("Fault"));
    assert_eq!(api.message, expected_msg);
}

async fn wait_until_async(timeout: Duration, mut pred: impl FnMut() -> bool) {
    let start = Instant::now();
    while !pred() {
        assert!(start.elapsed() < timeout, "timed out waiting for async condition");
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
}

struct RestingSupportDriver {
    dof: usize,
    enable: EnableState,
    feedback_valid: bool,
    age: Option<Duration>,
    disables_arm: bool,
    hw_fault: bool,
    timeout_on_reset: bool,
    positions: Vec<f32>,
    temperature: Arc<Mutex<f32>>,
    clock: MockClock,
}

impl ArmDriver for RestingSupportDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            velocity_feedback: true, torque_feedback: true, temperature_feedback: true,
            torque_feedforward: false, enable_feedback: true, load_feedback: false,
        }
    }
    fn enable_state(&self) -> EnableState { self.enable }
    fn is_feedback_valid(&self) -> bool { self.feedback_valid }
    fn has_any_joint_enabled(&self) -> bool { self.enable == EnableState::Enabled }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![self.age; self.dof] }
    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
        out.clear(); for _ in 0..self.dof { out.push(self.age); }
    }
    fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
        self.dof == dof && self.age.is_some_and(|a| a <= max_age)
    }
    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
        out.clear(); let ms = self.age.map(|a| a.as_millis() as u64); for _ in 0..self.dof { out.push(ms); }
    }
    fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        out.clear(); let t = self.age.and_then(|a| Instant::now().checked_sub(a)); for _ in 0..self.dof { out.push(t); }
    }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let mut out = Vec::with_capacity(self.dof); self.read_joint_states_into(&mut out)?; Ok(out)
    }
    fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
        out.clear();
        let temp = *self.temperature.lock().unwrap();
        for i in 0..self.dof {
            let mut st = JointState::default();
            st.position = self.positions.get(i).copied().unwrap_or(0.0);
            st.temperature = temp;
            out.push(st);
        }
        Ok(())
    }
    fn read_joint_states_budgeted(&mut self, out: &mut Vec<JointState>, _: Instant, _: u16) -> Result<(u16, bool), ArmDriverError> {
        self.read_joint_states_into(out)?; Ok((self.dof as u16, false))
    }
    fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
    fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
        self.enable = if enable { EnableState::Enabled } else { EnableState::Disabled };
        Ok(())
    }
    fn begin_torque_budgeted(&mut self, enable: bool, _: u16) -> Result<armd::driver::TorqueProgress, ArmDriverError> {
        self.enable = if enable { EnableState::Enabled } else { EnableState::Disabled };
        Ok(armd::driver::TorqueProgress::Done(armd::driver::TorqueOutcome::Confirmed))
    }
    fn supports_rt_executor(&self) -> Result<(), &'static str> { Ok(()) }
    fn clear_fault_disables_joints(&self) -> Vec<usize> { if self.disables_arm { vec![0] } else { Vec::new() } }
    fn clear_fault_disables_arm_joint(&self, arm_dof: usize) -> bool { self.disables_arm && arm_dof > 0 }
    fn has_hardware_fault(&self) -> bool { self.hw_fault }
    fn check_hardware_status(&self) -> Result<(), ArmDriverError> {
        if self.hw_fault { Err(ArmDriverError::Timeout(1)) } else { Ok(()) }
    }
    fn clear_fault_budgeted(&mut self, deadline: Instant) -> Result<(), ArmDriverError> {
        if self.timeout_on_reset || self.clock.now() >= deadline {
            Err(ArmDriverError::Transport(TransportError::Timeout))
        } else { Ok(()) }
    }
    fn clock_now(&self) -> Instant { self.clock.now() }
}

#[test]
fn logwriter_zero_alloc_inline_and_zero_copy_on_reject() {
    let sink = GatedSink::new();
    let w = NonBlocking::spawn_unregistered("b3-lw", sink.clone(), 4).unwrap();
    let (inline, oversized) = ([b'x'; 128], [b'y'; INLINE_LINE_BYTES * 4]);
    alloc::time_this_thread();

    let _ = alloc::take();
    tracing_subscriber::fmt::MakeWriter::make_writer(&w).write_all(&inline).unwrap();
    assert_eq!(alloc::take().calls, 0, "accepted inline write must do 0 heap ops");

    // Wait until the writer thread is blocked inside the gated sink so the queue fill is deterministic.
    let wait_start = Instant::now();
    while !sink.entered.load(Ordering::Acquire) {
        assert!(wait_start.elapsed() < Duration::from_secs(2), "writer did not reach gated sink");
        std::thread::sleep(Duration::from_millis(1));
    }

    for _ in 0..8 { tracing_subscriber::fmt::MakeWriter::make_writer(&w).write_all(b"fill\n").unwrap(); }
    let dropped_before = w.dropped();
    assert_eq!(dropped_before, 4, "queue of 4 with writer blocked drops 4 of 8 writes");
    let _ = alloc::take();
    tracing_subscriber::fmt::MakeWriter::make_writer(&w).write_all(&inline).unwrap();
    tracing_subscriber::fmt::MakeWriter::make_writer(&w).write_all(&oversized).unwrap();
    assert_eq!(alloc::take().calls, 0, "rejected inline + oversized writes must do 0 heap ops");
    assert_eq!(w.dropped(), dropped_before + 2);

    *sink.gate.0.lock().unwrap() = true;
    sink.gate.1.notify_all();
    assert!(w.flush(Duration::from_secs(2)));
    let text = String::from_utf8(sink.out.lock().unwrap().clone()).unwrap();
    assert!(text.contains("armd log:") && text.contains("lines dropped"), "{text}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn safety_logging_estop_fault_clear_zero_alloc_with_subscriber_enabled_and_same_text() {
    let log = global_log();
    let _serial = log.guard.lock().await;
    let _ = take_global_logs(log);

    for st in [
        SafetyState::Booting, SafetyState::Idle, SafetyState::Armed, SafetyState::Moving, SafetyState::EStop,
        SafetyState::Fault(FaultReason::BusLoss), SafetyState::Fault(FaultReason::WatchdogTimeout),
        SafetyState::Fault(FaultReason::ThermalTrip { joint_idx: 2, temperature: 92.5, limit: 85.0 }),
        SafetyState::Fault(FaultReason::JointLimitExceeded { joint_idx: 1, joint_name: "joint_2".into(), value: 3.1, min: -2.0, max: 2.0 }),
        SafetyState::Fault(FaultReason::Other("motor fault 0x0A".into())),
    ] {
        assert_eq!(format!("{st:?}"), format!("{:?}", BoundedSafetyState::from_state(&st)));
        if let SafetyState::Fault(ref r) = st {
            assert_eq!(format!("{r:?}"), format!("{:?}", BoundedFaultReason::from_reason(r)));
        }
    }

    // Positive control on a fresh thread: synchronous logging with subscriber ENABLED allocates.
    let sync_allocs = std::thread::spawn(|| {
        let mut sm = SafetyManager::new();
        alloc::time_this_thread();
        let _ = alloc::take();
        sm.trigger_e_stop();
        let _ = sm.clear_fault().unwrap();
        sm.trigger_fault(FaultReason::ThermalTrip { joint_idx: 1, temperature: 92.5, limit: 85.0 });
        let _ = sm.clear_fault().unwrap();
        alloc::take()
    }).join().unwrap();
    assert!(sync_allocs.calls > 0, "positive control: sync logging with subscriber enabled must allocate");
    let sync_text = take_global_logs(log);
    assert!(sync_text.contains("Safety: Idle -> EStop"), "{sync_text}");
    assert!(sync_text.contains("Safety: Idle -> Fault(ThermalTrip { joint_idx: 1, temperature: 92.5, limit: 85.0 }) [ThermalTrip]"), "{sync_text}");
    assert!(sync_text.contains("Safety: cleared -> Idle"), "{sync_text}");

    // Deferred mode + owned fault-state construction with subscriber ENABLED -> 0 heap ops.
    let mut sm = std::thread::spawn(|| {
        let mut sm = SafetyManager::new();
        sm.set_off_thread_reclaim(true);
        alloc::time_this_thread();
        let _ = alloc::take();
        sm.trigger_e_stop();
        assert_eq!(sm.clear_fault_bounded().unwrap(), SafetyState::Idle);
        sm.trigger_fault(FaultReason::BusLoss);
        assert_eq!(sm.clear_fault_bounded().unwrap(), SafetyState::Idle);
        sm.trigger_fault(FaultReason::WatchdogTimeout);
        assert_eq!(sm.clear_fault_bounded().unwrap(), SafetyState::Idle);
        sm.try_trigger_fault_coded(
            FaultCode::ThermalTrip,
            FaultReason::ThermalTrip { joint_idx: 1, temperature: 92.5, limit: 85.0 },
        ).unwrap();
        sm.trigger_e_stop();
        assert_eq!(sm.clear_fault_bounded().unwrap(), SafetyState::Idle);
        let a = alloc::take();
        assert_eq!(a.calls, 0, "deferred E-stop/fault/clear + owned fault construction allocated: {a:?}");
        sm
    }).join().unwrap();
    assert_eq!(take_global_logs(log), "", "no log line may be formatted on the control thread");

    sm.drain_log_events_off_thread();
    sm.drain_retired_state_arcs_off_thread();
    let deferred_text = take_global_logs(log);
    for expected in [
        "Safety: Idle -> EStop", "Safety: cleared -> Idle",
        "Safety: Idle -> Fault(BusLoss) [BusLoss]",
        "Safety: Idle -> Fault(WatchdogTimeout) [WatchdogTimeout]",
        "Safety: Idle -> Fault(ThermalTrip { joint_idx: 1, temperature: 92.5, limit: 85.0 }) [ThermalTrip]",
        "Safety: Fault(ThermalTrip { joint_idx: 1, temperature: 92.5, limit: 85.0 }) -> EStop",
    ] {
        assert!(deferred_text.contains(expected), "missing `{expected}` in:\n{deferred_text}");
    }

    // Overflowing SAFETY_LOG_CAPACITY stays 0-alloc and reports dropped count off-thread.
    alloc::time_this_thread();
    let _ = alloc::take();
    for _ in 0..(SAFETY_LOG_CAPACITY + 5) {
        sm.trigger_e_stop();
        let _ = sm.clear_fault_bounded().unwrap();
    }
    assert_eq!(alloc::take().calls, 0, "bounded safety log overflow allocated");
    sm.drain_log_events_off_thread();
    let ov_text = take_global_logs(log);
    assert!(ov_text.contains("Safety log:") && ov_text.contains("events dropped"), "{ov_text}");

    // Production RtExecutor: E-stop + clear log lines appear off-thread via `armd-dispatch`.
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    sup.set_lease_enabled(false).await;
    let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(4));
    cfg.alloc_timing = true;
    let handle = sup.start_executor(cfg).await.unwrap();
    wait_until_async(Duration::from_secs(1), || handle.stats().steps.load(Ordering::Relaxed) >= 4).await;
    let _ = take_global_logs(log);
    assert_eq!(sup.trigger_e_stop_checked().await.unwrap().0, SafetyState::EStop);
    assert_eq!(sup.clear_fault().await.unwrap(), SafetyState::Idle);
    handle.stop_and_join();
    let rt_text = take_global_logs(log);
    assert!(rt_text.contains("Safety: Idle -> EStop") && rt_text.contains("Safety: cleared -> Idle"), "{rt_text}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn clear_fault_refusals_zero_alloc_unchanged_outcome_and_identical_api_text() {
    let _serial = global_log().guard.lock().await;
    assert_eq!(ClearFaultDriverError::EstopInhibited.to_string(), armd::driver::ESTOP_INHIBITED);

    // 1. Real DamiaoArmDriver<FakeBus>: missing feedback, expired budget, and reset sent.
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&dm_limits(), clock.clone());
    bus.with(|s| { s.immediate_replies = true; s.push_all_feedback(); });
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7]).with_joint_limits(dm_limits()).unwrap();
    drv.read_joint_states().unwrap();
    drv.check_mapping_ranges();
    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv));
    inner.lease_enabled = false;
    inner.safety.set_off_thread_reclaim(true);
    inner.set_torque(true).unwrap();
    inner.tick(0.02);

    // (a) Missing/stale feedback -> 0 alloc, fault stays latched, identical API text.
    inner.safety.trigger_fault(FaultReason::BusLoss);
    clock.advance(Duration::from_millis(400));
    let ep_before = inner.safety.epoch_id();
    alloc::time_this_thread();
    let _ = alloc::take();
    let err_missing = inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap_err();
    assert_eq!(alloc::take().calls, 0, "missing feedback refusal allocated");
    assert_eq!(err_missing, ClearFaultError::Rejected(ClearFaultRefusal::FreshFeedbackRequired));
    assert_eq!((inner.safety.state(), inner.safety.epoch_id()), (&SafetyState::Fault(FaultReason::BusLoss), ep_before));
    assert_api_clear_fault_text(err_missing, "Fresh feedback is required before resetting a fault");

    // (b) Expired budget + Reset sent with motor 0 latched fault 0x0A.
    bus.with(|s| { s.motors[0].state_override = Some(0x0A); s.push_all_feedback(); });
    let _ = inner.poll_hardware(&ServiceBudget::for_release(clock.now(), clock.now() + Duration::from_millis(20)));
    bus.clear_log();
    let _ = alloc::take();
    let err_budget = inner.clear_fault_executor_bounded(Instant::now() - Duration::from_millis(1)).unwrap_err();
    assert_eq!(alloc::take().calls, 0, "expired budget refusal allocated");
    assert_eq!(err_budget, ClearFaultError::Rejected(ClearFaultRefusal::BudgetExpired));
    assert!(bus.tx_log().is_empty() && matches!(inner.safety.state(), SafetyState::Fault(_)));
    assert_api_clear_fault_text(err_budget, "Clear fault budget expired");

    let _ = alloc::take();
    let err_reset = inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap_err();
    assert_eq!(alloc::take().calls, 0, "reset sent refusal allocated");
    assert_eq!(err_reset, ClearFaultError::Rejected(ClearFaultRefusal::HardwareResetSent));
    assert!(!bus.tx_log().is_empty() && matches!(inner.safety.state(), SafetyState::Fault(_)));
    assert_api_clear_fault_text(err_reset, "Hardware reset sent; wait for fresh fault-free feedback before retrying");

    // 2. Resting support refusals + driver timeout + shutdown/disable_pending: 0 alloc + legacy text parity.
    let mut cfg = ArmConfig::rebot_dm();
    cfg.resting_pose = Some(armd::model::RestingPosePolicy::rebot_dm());
    let rs_drv = RestingSupportDriver {
        dof: cfg.dof, enable: EnableState::Enabled, feedback_valid: true,
        age: Some(Duration::from_millis(10)), disables_arm: true, hw_fault: false,
        timeout_on_reset: false, positions: cfg.resting_positions().to_vec(),
        temperature: Arc::new(Mutex::new(30.0)), clock: MockClock::new(),
    };
    let mut rs = SupervisorInner::new(cfg.clone()).with_driver(Box::new(rs_drv));
    rs.lease_enabled = false;
    rs.is_torqued = true;
    rs.feedback_numeric_valid = true;
    rs.safety.set_off_thread_reclaim(true);
    rs.safety.trigger_fault(FaultReason::BusLoss);
    rs.measured_positions = rs.config.resting_positions().to_vec();
    rs.measured_velocities = vec![0.0; rs.config.dof];
    rs.torques = vec![0.0; rs.config.dof];

    rs.feedback_numeric_valid = false;
    let _ = alloc::take();
    let err_rs_fb = rs.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap_err();
    assert_eq!(alloc::take().calls, 0, "resting-support missing feedback allocated");
    assert_api_clear_fault_text(err_rs_fb, "Fresh feedback is required before checking resting pose support");
    rs.feedback_numeric_valid = true;

    // Velocity + off-home + torque violation -> exact parity with legacy.
    rs.measured_velocities[0] = 0.25;
    rs.measured_positions[1] = rs.config.resting_positions()[1] + 0.5;
    rs.torques[1] = 4.0;
    let legacy_reason = rs.resting_pose_support_violation_with_driver(Some(rs.driver.as_ref().unwrap().lock().unwrap().as_ref())).unwrap();
    let _ = alloc::take();
    let err_rs = rs.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap_err();
    assert_eq!(alloc::take().calls, 0, "resting support refusal allocated");
    assert_eq!(*rs.safety.state(), SafetyState::Fault(FaultReason::BusLoss));
    assert_api_clear_fault_text(err_rs, &format!("Cannot disable faulted supporting arm joint away from resting pose ({legacy_reason}); support the arm at rest first"));

    // Total deviation violation -> exact parity with legacy.
    rs.measured_velocities.fill(0.0);
    rs.torques.fill(0.0);
    rs.measured_positions = rs.config.resting_positions().to_vec();
    for j in 0..5 { rs.measured_positions[j] += 0.06; }
    let legacy_tot = rs.resting_pose_support_violation_with_driver(Some(rs.driver.as_ref().unwrap().lock().unwrap().as_ref())).unwrap();
    let _ = alloc::take();
    let err_tot = rs.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap_err();
    assert_eq!(alloc::take().calls, 0, "total-deviation resting refusal allocated");
    assert_api_clear_fault_text(err_tot, &format!("Cannot disable faulted supporting arm joint away from resting pose ({legacy_tot}); support the arm at rest first"));

    // Workspace violation -> exact parity with legacy.
    rs.measured_positions = rs.config.resting_positions().to_vec();
    if let Some(ws) = rs.config.workspace.as_mut() { ws.floor_z_m = 2.0; }
    let legacy_ws = rs.resting_pose_support_violation_with_driver(Some(rs.driver.as_ref().unwrap().lock().unwrap().as_ref())).unwrap();
    let _ = alloc::take();
    let err_ws = rs.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap_err();
    assert_eq!(alloc::take().calls, 0, "workspace resting refusal allocated");
    assert_api_clear_fault_text(err_ws, &format!("Cannot disable faulted supporting arm joint away from resting pose ({legacy_ws}); support the arm at rest first"));

    // Budget expired inside `clear_fault_budgeted` -> TransportTimeout (0 alloc).
    let to_drv = RestingSupportDriver {
        dof: cfg.dof, enable: EnableState::Enabled, feedback_valid: true, age: Some(Duration::from_millis(10)),
        disables_arm: false, hw_fault: true, timeout_on_reset: true,
        positions: cfg.resting_positions().to_vec(), temperature: Arc::new(Mutex::new(30.0)), clock: MockClock::new(),
    };
    let mut to = SupervisorInner::new(cfg.clone()).with_driver(Box::new(to_drv));
    to.safety.set_off_thread_reclaim(true);
    to.safety.trigger_fault(FaultReason::BusLoss);
    let _ = alloc::take();
    let err_to = to.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap_err();
    assert_eq!(alloc::take().calls, 0, "in-driver clear_fault timeout allocated");
    assert_eq!(err_to, ClearFaultError::Rejected(ClearFaultRefusal::DriverError(ClearFaultDriverError::TransportTimeout)));
    assert_api_clear_fault_text(err_to, "Transport error: Timeout error");

    // DisablePending + ShutdownCommitted -> 0 alloc + exact API text.
    rs.disable_pending = true;
    let _ = alloc::take();
    let err_dp = rs.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap_err();
    assert_eq!(alloc::take().calls, 0);
    assert_api_clear_fault_text(err_dp, "Torque disable is still unconfirmed");
    rs.disable_pending = false;
    rs.shutdown_committed = true;
    let _ = alloc::take();
    let err_sd = rs.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap_err();
    assert_eq!(alloc::take().calls, 0);
    assert_api_clear_fault_text(err_sd, "Daemon shutdown is committed");

    // 3. Production RtExecutor end-to-end: ClearFault refusal runs on `armd-control` with
    //    `lock_alloc_calls == 0` and renders exact API error text off-thread.
    let mut off_home = cfg.resting_positions().to_vec();
    off_home[1] += 0.5;
    let rt_drv = RestingSupportDriver {
        dof: cfg.dof, enable: EnableState::Enabled, feedback_valid: true,
        age: Some(Duration::from_millis(10)), disables_arm: true, hw_fault: false,
        timeout_on_reset: false, positions: off_home.clone(), temperature: Arc::new(Mutex::new(30.0)), clock: MockClock::new(),
    };
    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(rt_drv)));
    {
        let mut g = sup.inner().write().await;
        g.lease_enabled = false;
        g.is_torqued = true;
        g.feedback_numeric_valid = true;
        g.safety.trigger_fault(FaultReason::BusLoss);
        g.measured_positions = off_home;
        g.measured_velocities = vec![0.0; g.config.dof];
        g.torques = vec![0.0; g.config.dof];
    }
    let mut exec_cfg = ExecutorConfig::for_tests(Duration::from_millis(4));
    exec_cfg.alloc_timing = true;
    let handle = sup.start_executor(exec_cfg).await.unwrap();
    wait_until_async(Duration::from_secs(1), || handle.stats().steps.load(Ordering::Relaxed) >= 8).await;
    let before = sup.tick_samples().len();
    let res = sup.clear_fault().await;
    let after_step = handle.stats().steps.load(Ordering::Relaxed);
    wait_until_async(Duration::from_secs(1), || handle.stats().steps.load(Ordering::Relaxed) >= after_step + 3).await;
    let api = ApiError::from(res.unwrap_err());
    assert_eq!((api.status, api.code.as_deref()), (StatusCode::CONFLICT, Some("CLEAR_FAULT_REJECTED")));
    assert!(api.message.starts_with("Cannot disable faulted supporting arm joint away from resting pose ("), "{}", api.message);
    assert!(api.message.ends_with("); support the arm at rest first"), "{}", api.message);
    assert_eq!(sup.tick_samples()[before..].iter().map(|s| s.usage.lock_alloc_calls).max().unwrap_or(0), 0);
    handle.stop_and_join();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn exhausted_pool_still_latches_thermal_trip_during_bus_loss_and_watchdog_and_blocks_clear_until_cooldown() {
    let _serial = global_log().guard.lock().await;
    for (initial_reason, initial_kind, initially_recoverable) in [
        (FaultReason::BusLoss, FaultKind::BusLoss, false),
        (FaultReason::WatchdogTimeout, FaultKind::WatchdogTimeout, true),
    ] {
        // Test both RT (`clear_fault_executor_bounded`) and legacy (`clear_fault`) paths.
        for use_rt in [true, false] {
            let mut cfg = ArmConfig::rebot_dm();
            cfg.max_temperature = 75.0;
            let temp_ctl = Arc::new(Mutex::new(30.0f32));
            let drv = RestingSupportDriver {
                dof: cfg.dof,
                enable: EnableState::Enabled,
                feedback_valid: true,
                age: Some(Duration::from_millis(10)),
                disables_arm: false,
                hw_fault: false,
                timeout_on_reset: false,
                positions: cfg.resting_positions().to_vec(),
                temperature: Arc::clone(&temp_ctl),
                clock: MockClock::new(),
            };
            let mut inner = SupervisorInner::new(cfg.clone()).with_driver(Box::new(drv));
            inner.lease_enabled = false;
            inner.is_torqued = true;
            inner.feedback_numeric_valid = true;
            inner.rt_executor_owned = use_rt;
            inner.temperatures = vec![30.0; cfg.dof];
            inner.safety.set_off_thread_reclaim(use_rt);

            // Exhaust the entire preallocated fault-slot pool before latching any fault.
            let held_pool = inner.safety.drain_fault_arc_pool_for_test();
            assert_eq!(inner.safety.fault_arc_pool_len(), 0);

            // Latch BusLoss or WatchdogTimeout, then raise temperature above limit so ThermalTrip fires.
            inner.safety.trigger_fault(initial_reason.clone());
            assert_eq!(*inner.safety.state(), SafetyState::Fault(initial_reason.clone()));
            assert_eq!(inner.safety.fault().unwrap().recoverable, initially_recoverable);
            *temp_ctl.lock().unwrap() = 85.0;
            inner.temperatures[0] = 85.0;

            alloc::time_this_thread();
            let _ = alloc::take();
            inner.apply_stop_policy(StopPolicy::ThermalTrip);
            let trip_allocs = alloc::take();
            if use_rt {
                assert_eq!(trip_allocs.calls, 0, "ThermalTrip while pool exhausted must do 0 heap ops: {trip_allocs:?}");
            }

            // Current Arc stays the existing fault (no Arc::new), while BOTH kinds are recorded
            // and `recoverable` reflects the non-recoverable ThermalTrip.
            assert_eq!(*inner.safety.state(), SafetyState::Fault(initial_reason.clone()));
            assert!(inner.safety.has_latched_fault_kind(initial_kind));
            assert!(inner.safety.has_latched_fault_kind(FaultKind::ThermalTrip));
            assert!(!inner.safety.fault().unwrap().recoverable);

            // A second thermal trip does not churn epoch_id once ThermalTrip is already recorded.
            let ep_after_trip = inner.safety.epoch_id();
            inner.apply_stop_policy(StopPolicy::ThermalTrip);
            assert_eq!(inner.safety.epoch_id(), ep_after_trip);

            // clear_fault MUST check EVERY recorded kind: stale bus feedback fails first,
            // then fresh feedback with hot temperature STILL fails thermal cooldown.
            if use_rt {
                inner.feedback_numeric_valid = false;
                let _ = alloc::take();
                let err_fb = inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap_err();
                assert_eq!(alloc::take().calls, 0, "stale feedback refusal must do 0 heap ops");
                assert_eq!(err_fb, ClearFaultError::Rejected(ClearFaultRefusal::FreshFeedbackRequired));
                inner.feedback_numeric_valid = true;

                let _ = alloc::take();
                let err = inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap_err();
                assert_eq!(alloc::take().calls, 0, "thermal clear refusal must do 0 heap ops");
                assert_eq!(err, ClearFaultError::Rejected(ClearFaultRefusal::TemperatureConfirmationRequired));
                assert_api_clear_fault_text(err, "Temperature must be confirmed below the reset threshold");
            } else {
                // Legacy path with driver checks fresh driver temperatures against recorded ThermalTrip.
                let err_drv = inner.clear_fault().unwrap_err();
                assert_eq!(
                    err_drv,
                    SafetyError::ClearFaultRejected("Temperature must be confirmed below the reset threshold".into())
                );
                // Also verify the tick -> check_thermal_protection -> legacy no-driver clear_fault path.
                let mut sim = SupervisorInner::new(cfg.clone());
                sim.lease_enabled = false;
                sim.is_torqued = true;
                sim.temperatures = vec![30.0; cfg.dof];
                let _held_sim = sim.safety.drain_fault_arc_pool_for_test();
                sim.safety.trigger_fault(initial_reason.clone());
                sim.temperatures[0] = 85.0;
                sim.tick(0.02);
                assert_eq!(*sim.safety.state(), SafetyState::Fault(initial_reason.clone()));
                assert!(sim.safety.has_latched_fault_kind(FaultKind::ThermalTrip));
                let ep_sim = sim.safety.epoch_id();
                sim.tick(0.02);
                assert_eq!(sim.safety.epoch_id(), ep_sim, "repeat tick must not churn epoch once ThermalTrip recorded");
                let err = sim.clear_fault().unwrap_err();
                assert_eq!(err, SafetyError::ClearFaultRejected("Temperature is above the reset threshold".into()));
                sim.temperatures[0] = 50.0;
                assert_eq!(sim.clear_fault().unwrap(), SafetyState::Idle);
                assert!(!sim.safety.has_latched_fault());
            }

            // Cool below reset threshold -> clear_fault now succeeds and resets the latched-kind record.
            *temp_ctl.lock().unwrap() = 50.0;
            inner.temperatures[0] = 50.0;
            if use_rt {
                assert_eq!(inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap(), SafetyState::Idle);
            } else {
                assert_eq!(inner.clear_fault().unwrap(), SafetyState::Idle);
            }
            assert!(!inner.safety.has_latched_fault());
            assert_eq!(inner.safety.latched_fault_kinds(), 0);
            drop(held_pool);
        }
    }
}

#[test]
fn every_fault_kind_latches_with_zero_alloc_while_pool_is_exhausted() {
    let mut sm = SafetyManager::new();
    sm.set_off_thread_reclaim(true);
    let held_pool = sm.drain_fault_arc_pool_for_test();
    assert_eq!(sm.fault_arc_pool_len(), 0);

    // Prebuild heap-bearing reasons off-measurement so we measure ONLY SafetyManager latching.
    let jl = FaultReason::JointLimitExceeded {
        joint_idx: 2,
        joint_name: "joint_3".into(),
        value: 3.2,
        min: -2.0,
        max: 2.0,
    };
    let other = FaultReason::Other("motor overcurrent".into());

    alloc::time_this_thread();
    let _ = alloc::take();

    // 1. First fault from Idle with empty pool: NEVER calls Arc::new; moves to preallocated
    //    `detail unavailable` fault Arc while recording exact kind + detail.
    let res_jl = sm.try_trigger_fault_coded(FaultCode::JointLimit, jl);
    let first_allocs = alloc::take();
    assert_eq!(first_allocs.calls, 0, "first fault with empty pool must never allocate: {first_allocs:?}");
    assert_eq!(*sm.state(), SafetyState::Fault(FaultReason::Other(DETAIL_UNAVAILABLE.into())));
    assert_eq!(sm.fault_code(), Some(FaultCode::JointLimit));
    assert!(sm.has_latched_fault_kind(FaultKind::JointLimitExceeded));
    assert!(matches!(
        sm.latched_fault_detail(FaultKind::JointLimitExceeded).map(|d| d.reason),
        Some(BoundedFaultReason::JointLimitExceeded { joint_idx: 2, .. })
    ));

    // 2. Latch every remaining FaultKind while the pool stays completely empty (0 heap ops).
    let _ = alloc::take();
    let res_th = sm.try_trigger_fault_coded(
        FaultCode::ThermalTrip,
        FaultReason::ThermalTrip { joint_idx: 1, temperature: 92.0, limit: 80.0 },
    );
    let res_bus = sm.try_trigger_fault_coded(FaultCode::BusLoss, FaultReason::BusLoss);
    let res_wd = sm.try_trigger_fault_coded(FaultCode::WatchdogTimeout, FaultReason::WatchdogTimeout);
    let res_other = sm.try_trigger_fault_coded(FaultCode::HardwareFault, other);
    let rest_allocs = alloc::take();
    assert_eq!(rest_allocs.calls, 0, "latching all fault kinds with empty pool must do 0 heap ops: {rest_allocs:?}");

    assert!(res_bus.is_ok() && res_wd.is_ok(), "cached BusLoss/WatchdogTimeout arcs do not need pool slots");
    let (_, ret_jl) = res_jl.unwrap_err();
    let (_, ret_th) = res_th.unwrap_err();
    let (_, ret_other) = res_other.unwrap_err();

    for kind in FaultKind::ALL {
        assert!(sm.has_latched_fault_kind(kind), "missing latched kind {kind:?}");
        assert!(sm.latched_fault_detail(kind).is_some(), "missing detail for {kind:?}");
    }

    // 3. Lazy fault when pool is empty skips closure (0 alloc) and records kind with "detail unavailable".
    let mut sm_lazy = SafetyManager::new();
    sm_lazy.set_off_thread_reclaim(true);
    let held_lazy = sm_lazy.drain_fault_arc_pool_for_test();
    let mut called = false;
    let _ = alloc::take();
    sm_lazy.trigger_fault_coded_lazy(FaultCode::ThermalTrip, || {
        called = true;
        FaultReason::ThermalTrip { joint_idx: 0, temperature: 99.0, limit: 80.0 }
    });
    let lazy_allocs = alloc::take();
    assert!(!called);
    assert_eq!(lazy_allocs.calls, 0);
    assert!(sm_lazy.has_latched_fault_kind(FaultKind::ThermalTrip));
    assert_eq!(*sm_lazy.state(), SafetyState::Fault(FaultReason::Other(DETAIL_UNAVAILABLE.into())));
    assert_eq!(
        sm_lazy.latched_fault_detail(FaultKind::ThermalTrip).map(|d| format!("{:?}", d.reason)),
        Some(format!("Other(\"{DETAIL_UNAVAILABLE}\")"))
    );

    // Clearing resets all latched kinds.
    assert_eq!(sm.clear_fault_bounded().unwrap(), SafetyState::Idle);
    assert!(!sm.has_latched_fault());
    assert_eq!(sm.latched_fault_kinds(), 0);
    for kind in FaultKind::ALL {
        assert!(sm.latched_fault_detail(kind).is_none());
    }
    drop((ret_jl, ret_th, ret_other, held_pool, held_lazy));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn remaining_safety_paths_zero_alloc_fault_producers_gravity_comp_estop_and_large_workspace() {
    let log = global_log();
    let _serial = log.guard.lock().await;
    let _ = take_global_logs(log);

    // 1. Static + dynamically formatted fault producers & FaultInfo construction -> 0 heap ops.
    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
    inner.lease_enabled = false;
    inner.is_torqued = true;
    inner.safety.set_off_thread_reclaim(true);
    let _ = inner.safety.set_torque(true).unwrap();

    // Warm one control tick (populates lazy scratch), then trigger ControlDeadline -> 0 heap ops.
    inner.tick_control(0.02);
    alloc::time_this_thread();
    let _ = alloc::take();
    // Control loop deadline exceeded (src/supervisor.rs:2278) -> StaticFault::ControlDeadline.
    inner.tick_control(0.5);
    let deadline_allocs = alloc::take();
    assert_eq!(deadline_allocs.calls, 0, "ControlDeadline tick fault path allocated: {deadline_allocs:?}");
    let fi = inner.safety.fault().unwrap();
    assert_eq!(fi.code, FaultCode::ControlDeadline);
    assert_eq!(fi.message, "Control loop deadline exceeded");

    // StaticFault::InvalidTelemetry + trigger_fault_formatted + FaultInfo build -> 0 heap ops.
    let _ = alloc::take();
    inner.safety.trigger_static_fault(StaticFault::InvalidTelemetry);
    let fi_inv = inner.safety.fault().unwrap();
    inner.safety.trigger_fault_formatted(FaultCode::FollowingError, FaultKind::Other, |b| {
        let _ = core::fmt::write(b, format_args!("Persistent joint tracking error: joint 2 cmd 0.500 meas 0.100 rad"));
    });
    let fi_fe = inner.safety.fault().unwrap();
    let prod_allocs = alloc::take();
    assert_eq!(prod_allocs.calls, 0, "static + formatted fault producers allocated: {prod_allocs:?}");
    assert_eq!((fi_inv.code, fi_inv.message.as_str()), (FaultCode::InvalidTelemetry, "Invalid hardware telemetry"));
    assert_eq!(fi_fe.code, FaultCode::FollowingError);
    assert_eq!(fi_fe.message, "Persistent joint tracking error: joint 2 cmd 0.500 meas 0.100 rad");
    inner.safety.drain_retired_state_arcs_off_thread();
    inner.safety.drain_log_events_off_thread();

    // 2. E-stop during floating gravity compensation (src/supervisor/motion.rs:161) -> 0 heap ops.
    let mut gc_cfg = ArmConfig::rebot_dm();
    gc_cfg.joint_tracking = None;
    let mut gc = SupervisorInner::new(gc_cfg);
    gc.lease_enabled = false;
    gc.set_torque(true).unwrap();
    gc.tick(0.02);
    gc.enter_gravity_comp(armd::gravity_comp::GravityCompRequest::default()).unwrap();
    gc.tick(0.02);
    assert!(gc.gravity_comp.is_some());
    gc.safety.set_off_thread_reclaim(true);
    let _ = take_global_logs(log);

    let _ = alloc::take();
    gc.apply_stop_policy(StopPolicy::EStopLatch);
    let gc_estop_allocs = alloc::take();
    assert_eq!(gc_estop_allocs.calls, 0, "E-stop during gravity comp allocated: {gc_estop_allocs:?}");
    assert_eq!(*gc.safety.state(), SafetyState::EStop);
    let last_exit = gc.gravity_comp_last_exit.expect("gravity_comp_last_exit must be recorded");
    assert_eq!((last_exit.code.as_str(), last_exit.message.as_str()), ("ESTOP", "E-stop"));
    assert_eq!(take_global_logs(log), "", "gravity comp E-stop must not format logs on control thread");
    gc.safety.drain_log_events_off_thread();
    let gc_logs = take_global_logs(log);
    assert!(gc_logs.contains("Gravity compensation exit: ESTOP E-stop"), "{gc_logs}");

    // 3. clear_fault with workspace exceeding 256 slots (src/supervisor/driver_transitions.rs:1613) -> 0 heap ops.
    let mut ws_cfg = ArmConfig::rebot_dm();
    ws_cfg.resting_pose = Some(armd::model::RestingPosePolicy::rebot_dm());
    {
        let ws = ws_cfg.workspace.as_mut().expect("rebot-dm workspace");
        ws.keepout_boxes.clear();
        for i in 0..16 {
            ws.keepout_boxes.push(armd::model::KeepoutBox {
                name: format!("kb_{i}"),
                min_m: [0.5 + i as f32 * 0.02, 0.5, 0.5],
                max_m: [0.51 + i as f32 * 0.02, 0.55, 0.55],
            });
        }
        // Add extra end check points so total_slots > MAX_STACK_WORKSPACE_SLOTS (256).
        for i in 0..10 {
            ws.check_points_end_m.push([0.0, 0.0, 0.01 * (i as f32 + 1.0)]);
        }
        ws.floor_z_m = 2.0; // force a workspace violation at resting pose
        let eval = armd::chain::WorkspaceEvaluator::new_with_tcp(
            &ws_cfg.arm_model, ws, &ws_cfg.camera_mount, ws_cfg.tcp_offset,
        );
        assert!(
            eval.total_slots > armd::kinematics::MAX_STACK_WORKSPACE_SLOTS,
            "test must exceed 256 workspace slots, got {}", eval.total_slots
        );
    }
    let ws_drv = RestingSupportDriver {
        dof: ws_cfg.dof,
        enable: EnableState::Enabled,
        feedback_valid: true,
        age: Some(Duration::from_millis(10)),
        disables_arm: true,
        hw_fault: false,
        timeout_on_reset: false,
        positions: ws_cfg.resting_positions().to_vec(),
        temperature: Arc::new(Mutex::new(30.0)),
        clock: MockClock::new(),
    };
    let mut ws_inner = SupervisorInner::new(ws_cfg).with_driver(Box::new(ws_drv));
    ws_inner.lease_enabled = false;
    ws_inner.is_torqued = true;
    ws_inner.feedback_numeric_valid = true;
    ws_inner.safety.set_off_thread_reclaim(true);
    ws_inner.safety.trigger_fault(FaultReason::BusLoss);
    ws_inner.measured_positions = ws_inner.config.resting_positions().to_vec();
    ws_inner.measured_velocities = vec![0.0; ws_inner.config.dof];
    ws_inner.torques = vec![0.0; ws_inner.config.dof];
    let legacy_ws_msg = ws_inner
        .resting_pose_support_violation_with_driver(Some(ws_inner.driver.as_ref().unwrap().lock().unwrap().as_ref()))
        .unwrap();

    let _ = alloc::take();
    let err_ws = ws_inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap_err();
    let ws_allocs = alloc::take();
    assert_eq!(ws_allocs.calls, 0, ">256-slot workspace clear_fault allocated: {ws_allocs:?}");
    assert_api_clear_fault_text(
        err_ws,
        &format!("Cannot disable faulted supporting arm joint away from resting pose ({legacy_ws_msg}); support the arm at rest first"),
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn safety_and_logwriter_drop_counters_are_monotonic_published_and_reported_at_flush_and_shutdown() {
    let log = global_log();
    let _serial = log.guard.lock().await;
    let _ = take_global_logs(log);

    // 1. LogWriter drop counter is monotonic (never resets) and reports drops on flush() and shutdown
    //    even without another line being written.
    let sink = GatedSink::new();
    let w = NonBlocking::spawn_unregistered("b3-mono-lw", sink.clone(), 2).unwrap();
    for i in 0..10 {
        tracing_subscriber::fmt::MakeWriter::make_writer(&w).write_all(format!("l{i}\n").as_bytes()).unwrap();
    }
    let dropped_after_burst = w.dropped();
    assert!(dropped_after_burst >= 7, "expected >= 7 drops, got {dropped_after_burst}");
    *sink.gate.0.lock().unwrap() = true;
    sink.gate.1.notify_all();
    assert!(w.flush(Duration::from_secs(2)));
    // Counter MUST remain monotonic after flush/report.
    assert_eq!(w.dropped(), dropped_after_burst, "logwriter dropped counter must never reset on flush");
    let flushed_text = String::from_utf8(sink.out.lock().unwrap().clone()).unwrap();
    assert!(flushed_text.contains("armd log:") && flushed_text.contains("lines dropped"), "flush must report drops: {flushed_text}");

    // Second burst while gated again -> drop count strictly increases, and drop(w) reports on shutdown.
    *sink.gate.0.lock().unwrap() = false;
    for i in 0..8 {
        tracing_subscriber::fmt::MakeWriter::make_writer(&w).write_all(format!("m{i}\n").as_bytes()).unwrap();
    }
    let dropped_second = w.dropped();
    assert!(dropped_second > dropped_after_burst);
    *sink.gate.0.lock().unwrap() = true;
    sink.gate.1.notify_all();
    drop(w);
    wait_until_async(Duration::from_secs(2), || {
        let s = String::from_utf8(sink.out.lock().unwrap().clone()).unwrap();
        s.matches("lines dropped").count() >= 2
    }).await;

    // 2. SafetyManager inline drop counter is monotonic across drains, and live RtExecutor publishes
    //    it in ExecutorStats, TickSnapshot diagnostics, and ArmObservation, and reports at shutdown.
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let expected_drops = {
        let mut inner = sup.inner().write().await;
        inner.lease_enabled = false;
        inner.safety.set_off_thread_reclaim(true);
        // Overflow the inline safety log ring before handing off to the executor so drops are
        // guaranteed even if the dispatcher thread drains promptly.
        for _ in 0..(SAFETY_LOG_CAPACITY + 10) {
            inner.safety.trigger_e_stop();
            let _ = inner.safety.clear_fault_bounded().unwrap();
        }
        let d = inner.safety.log_dropped();
        assert!(d >= 10, "expected >= 10 inline safety log drops, got {d}");
        d
    };
    let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(4));
    cfg.alloc_timing = true;
    let handle = sup.start_executor(cfg).await.unwrap();
    wait_until_async(Duration::from_secs(2), || {
        handle.stats().safety_log_dropped.load(Ordering::Relaxed) >= expected_drops
            && sup.tick_stats().safety_log_dropped >= expected_drops
    }).await;

    // Even after dispatch has drained and emitted best-effort warnings, every counter remains
    // monotonic and authoritative (NEVER reset to 0).
    let stats_drops = handle.stats().safety_log_dropped.load(Ordering::Relaxed);
    let diag_drops = sup.tick_stats().safety_log_dropped;
    let obs = sup.observation().await.unwrap();
    assert_eq!(stats_drops, expected_drops);
    assert_eq!(diag_drops, expected_drops);
    assert_eq!(obs.safety_log_dropped, expected_drops);
    let obs_json = serde_json::to_value(&obs).unwrap();
    assert_eq!(obs_json["safety_log_dropped"], expected_drops);

    // Clear captured logs, then stop executor: shutdown summary must still report the monotonic total
    // and counters must still equal expected_drops.
    let _ = take_global_logs(log);
    handle.stop_and_join();
    assert_eq!(handle.stats().safety_log_dropped.load(Ordering::Relaxed), expected_drops);
    assert_eq!(sup.tick_stats().safety_log_dropped, expected_drops);
    let shutdown_logs = take_global_logs(log);
    assert!(
        shutdown_logs.contains(&format!("Safety log shutdown: {expected_drops} total events dropped")),
        "missing shutdown summary in:\n{shutdown_logs}"
    );
}

#[test]
fn api_clear_fault_messages_mark_resting_overflow_and_utf8_driver_truncation() {
    // 1. More than 32 resting-policy failures (plus workspace violation as an extra entry):
    //    lists the first 32 entries then ` (+N more)`, keeps CLEAR_FAULT_REJECTED, 0 heap ops on RT,
    //    and matches legacy output verbatim.
    let mut cfg = ArmConfig::rebot_dm();
    let mut rp = armd::model::RestingPosePolicy::rebot_dm();
    rp.support_torque_limits.clear();
    for _ in 0..37 {
        rp.support_torque_limits.push((0, 0.1));
    }
    cfg.resting_pose = Some(rp);
    if let Some(ws) = cfg.workspace.as_mut() {
        ws.floor_z_m = 2.0; // 38th failing entry
    }
    let drv = RestingSupportDriver {
        dof: cfg.dof,
        enable: EnableState::Enabled,
        feedback_valid: true,
        age: Some(Duration::from_millis(10)),
        disables_arm: true,
        hw_fault: false,
        timeout_on_reset: false,
        positions: cfg.resting_positions().to_vec(),
        temperature: Arc::new(Mutex::new(30.0)),
        clock: MockClock::new(),
    };
    let mut inner = SupervisorInner::new(cfg.clone()).with_driver(Box::new(drv));
    inner.lease_enabled = false;
    inner.is_torqued = true;
    inner.feedback_numeric_valid = true;
    inner.safety.set_off_thread_reclaim(true);
    inner.safety.trigger_fault(FaultReason::BusLoss);
    inner.measured_positions = inner.config.resting_positions().to_vec();
    inner.measured_velocities = vec![0.0; inner.config.dof];
    inner.torques = vec![5.0; inner.config.dof]; // fails all 37 torque limits

    let legacy_reason = inner
        .resting_pose_support_violation_with_driver(Some(inner.driver.as_ref().unwrap().lock().unwrap().as_ref()))
        .unwrap();
    assert!(legacy_reason.ends_with(" (+6 more)"), "37 torque + 1 workspace = 38 -> +6 more: {legacy_reason}");
    assert_eq!(legacy_reason.matches("; ").count(), MAX_RESTING_SUPPORT_ITEMS - 1, "{legacy_reason}");

    alloc::time_this_thread();
    let _ = alloc::take();
    let err = inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap_err();
    assert_eq!(alloc::take().calls, 0, ">32 resting-policy refusal must do 0 heap ops");
    assert_api_clear_fault_text(
        err,
        &format!("Cannot disable faulted supporting arm joint away from resting pose ({legacy_reason}); support the arm at rest first"),
    );

    // 2. Driver error strings > 256 bytes: cut at 256 bytes on a UTF-8 char boundary with `…` (U+2026),
    //    while strings <= 256 bytes are NOT suffixed.
    let exact_256 = "a".repeat(256);
    let exact_bt = BoundedText::<256>::new(&exact_256);
    assert!(!exact_bt.is_truncated());
    assert_eq!(exact_bt.to_string(), exact_256);

    // 255 ASCII bytes + 2-byte 'é' (257 bytes total) -> must cut at byte 255 + '…' (valid UTF-8!).
    let mut utf8_boundary = "x".repeat(255);
    utf8_boundary.push('é');
    utf8_boundary.push_str("tail");
    let trunc_bt = BoundedText::<256>::new(&utf8_boundary);
    assert!(trunc_bt.is_truncated());
    assert_eq!(trunc_bt.as_str().len(), 255);
    assert_eq!(trunc_bt.to_string(), format!("{}…", "x".repeat(255)));

    // 254 ASCII bytes + 3-byte '中' (257 bytes) -> must cut at byte 254 + '…', keeps CLEAR_FAULT_REJECTED.
    let mut cjk_long = "z".repeat(254);
    cjk_long.push('中');
    cjk_long.push('余');
    let drv_err = ClearFaultError::Rejected(ClearFaultRefusal::DriverError(
        ClearFaultDriverError::HardwareFault(BoundedText::<256>::new(&cjk_long)),
    ));
    assert_api_clear_fault_text(drv_err, &format!("Hardware fault: {}…", "z".repeat(254)));

    // DriverConfigError > 256 bytes also truncates cleanly with `…` and keeps CLEAR_FAULT_REJECTED.
    let mut cfg_err_inner = SupervisorInner::new(cfg);
    cfg_err_inner.safety.set_off_thread_reclaim(true);
    cfg_err_inner.safety.trigger_fault(FaultReason::BusLoss);
    cfg_err_inner.driver_config_error = Some(cjk_long);
    let _ = alloc::take();
    let cfg_err = cfg_err_inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap_err();
    assert_eq!(alloc::take().calls, 0);
    assert_api_clear_fault_text(cfg_err, &format!("Driver limit configuration error: {}…", "z".repeat(254)));
}
