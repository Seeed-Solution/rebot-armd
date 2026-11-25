//! Stage 3 §3 PR B3 proof: zero-alloc LogWriter, deferred safety logging with subscriber enabled,
//! bounded owned fault-state storage, and typed bounded `clear_fault` refusals with identical API text.

mod mit_support;

#[global_allocator]
static ALLOC: armd::diagnostics::alloc::TimedAlloc = armd::diagnostics::alloc::TimedAlloc;

use std::io::{self, Write};
use std::sync::atomic::{AtomicBool, AtomicPtr, Ordering};
use std::sync::{Arc, Condvar, Mutex, OnceLock};
use std::time::{Duration, Instant};

use axum::http::StatusCode;
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

use armd::api::rest::ApiError;
use armd::diagnostics::alloc;
use armd::diagnostics::logwriter::{NonBlocking, INLINE_LINE_BYTES};
use armd::diagnostics::tick::{
    TickSample, RELEASE_FLAG_CLEAR_OK, RELEASE_FLAG_CLEAR_REFUSED, RELEASE_FLAG_ESTOP_APPLIED,
    RELEASE_FLAG_INJECT_ALLOC, RELEASE_FLAG_THERMAL_TRIP, RELEASE_FLAG_WATCHDOG_LATCH,
};
use armd::driver::{ArmDriver, ArmDriverError, DamiaoArmDriver, DriverCapabilities, EnableState, JointTarget};
use armd::model::JointState;
use armd::protocol::damiao::DamiaoLimits;
use armd::rt_core::executor::{ExecutorConfig, PersistTestHooks};
use armd::rt_core::ops::{CoreOps, ServiceBudget};
use armd::safety::{
    BoundedFaultReason, BoundedSafetyState, BoundedText, ClearContext, ClearFaultDriverError,
    ClearFaultError, ClearFaultRefusal, ClearRefusal, FaultCode, FaultDetail, FaultKind,
    FaultReason, SafetyError, SafetyManager, SafetyState, StaticFault, DETAIL_UNAVAILABLE,
    MAX_RESTING_SUPPORT_ITEMS, SAFETY_LOG_CAPACITY,
};
use armd::transport::{MockClock, TransportError};
use armd::{ArmConfig, ArmError, ArmSupervisor, OperatorAuth, StopPolicy, SupervisorInner};
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

/// Waits for and returns the SPECIFIC [`TickSample`] tagged with `release_seq`.
/// Fails explicitly if `release_seq == 0` or if that tagged sample is missing from `tick_samples()`.
async fn expect_release_sample(sup: &ArmSupervisor, release_seq: u64, label: &str) -> TickSample {
    assert!(release_seq > 0, "{label}: expected non-zero tagged release_seq");
    let start = Instant::now();
    loop {
        if let Some(sample) = sup.tick_samples().into_iter().find(|s| s.release_seq == release_seq) {
            return sample;
        }
        assert!(
            start.elapsed() < Duration::from_secs(2),
            "{label}: tagged release_seq={release_seq} sample missing from tick_samples()"
        );
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
        SafetyState::Fault(FaultReason::CanTxStalled),
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
        sm.trigger_e_stop();
        assert_eq!(sm.clear_fault_bounded().unwrap(), SafetyState::Idle);
        sm.trigger_e_stop();
        sm.try_trigger_fault_coded(
            FaultCode::ThermalTrip,
            FaultReason::ThermalTrip { joint_idx: 1, temperature: 92.5, limit: 85.0 },
        ).unwrap();
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
        "Safety: Fault(WatchdogTimeout) -> EStop",
        "Safety: EStop -> Fault(ThermalTrip { joint_idx: 1, temperature: 92.5, limit: 85.0 }) [ThermalTrip]",
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

    // Direct positive control: injecting exactly 1 allocation without in-window drop measures count == 1.
    let (single_alloc, boxed) = std::thread::spawn(|| {
        alloc::time_this_thread();
        let _ = alloc::take();
        let b = Box::new(0xB3_u64);
        std::hint::black_box(&*b);
        (alloc::take(), b)
    }).join().unwrap();
    drop(boxed);
    assert_eq!(single_alloc.calls, 1, "positive control: one allocation must measure count == 1: {single_alloc:?}");

    // Production RtExecutor: real ThermalTrip fault + refused clear + cooldown clear + E-stop + clear
    // all run with 0 heap ops across a non-empty sample window, logs appear off-thread via `armd-dispatch`,
    // and injecting one release allocation produces exact lock_alloc_calls == 1.
    let mut rt_arm_cfg = ArmConfig::rebot_dm();
    rt_arm_cfg.max_temperature = 75.0;
    rt_arm_cfg.joint_tracking = None;
    let temp_ctl = Arc::new(Mutex::new(30.0f32));
    let rt_drv = RestingSupportDriver {
        dof: rt_arm_cfg.dof,
        enable: EnableState::Enabled,
        feedback_valid: true,
        age: Some(Duration::from_millis(10)),
        disables_arm: false,
        hw_fault: false,
        timeout_on_reset: false,
        positions: rt_arm_cfg.resting_positions().to_vec(),
        temperature: Arc::clone(&temp_ctl),
        clock: MockClock::new(),
    };
    let sup = Arc::new(ArmSupervisor::new(rt_arm_cfg.clone()).with_driver(Box::new(rt_drv)));
    {
        let mut g = sup.inner().write().await;
        g.lease_enabled = false;
        g.is_torqued = true;
        g.feedback_numeric_valid = true;
        g.measured_positions = rt_arm_cfg.resting_positions().to_vec();
        g.commanded_positions = rt_arm_cfg.resting_positions().to_vec();
        g.target_positions = rt_arm_cfg.resting_positions().to_vec();
        g.measured_velocities = vec![0.0; g.config.dof];
        g.temperatures = vec![30.0; g.config.dof];
        let _ = g.safety.set_torque(true).unwrap();
    }
    let inject_once = Arc::new((AtomicBool::new(false), AtomicPtr::new(std::ptr::null_mut())));
    let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(4));
    cfg.alloc_timing = true;
    let hooks = PersistTestHooks {
        inject_release_alloc_once: Some(Arc::clone(&inject_once)),
        ..Default::default()
    };
    let handle = sup.start_executor_with_hooks(cfg, hooks).await.unwrap();
    wait_until_async(Duration::from_secs(2), || handle.stats().steps.load(Ordering::Relaxed) >= 8).await;
    let _ = take_global_logs(log);

    // Measure SPECIFIC tagged safety releases covering a real ThermalTrip fault, hot refusal,
    // cooldown clear, E-stop latch, and E-stop clear (failing explicitly if any sample is missing).
    *temp_ctl.lock().unwrap() = 85.0;
    wait_until_async(Duration::from_secs(2), || {
        sup.hub().latest().is_some_and(|s| matches!(s.obs.safety_state, SafetyState::Fault(FaultReason::ThermalTrip { .. })))
            && handle.stats().last_fault_latch_release_seq.load(Ordering::Acquire) > 0
    }).await;
    let thermal_seq = handle.stats().last_fault_latch_release_seq.load(Ordering::Acquire);
    let thermal_sample = expect_release_sample(&sup, thermal_seq, "ThermalTrip latch").await;
    assert_ne!(thermal_sample.release_flags & RELEASE_FLAG_THERMAL_TRIP, 0, "expected THERMAL_TRIP flag on release {thermal_seq}");
    assert_eq!(thermal_sample.usage.lock_alloc_calls, 0, "ThermalTrip latch release allocated: {thermal_sample:?}");

    let hot_err = ApiError::from(sup.clear_fault().await.unwrap_err());
    assert_eq!((hot_err.status, hot_err.code.as_deref()), (StatusCode::CONFLICT, Some("CLEAR_FAULT_REJECTED")));
    let hot_refuse_seq = handle.stats().last_clear_release_seq.load(Ordering::Acquire);
    let hot_refuse_sample = expect_release_sample(&sup, hot_refuse_seq, "hot clear refusal").await;
    assert_ne!(hot_refuse_sample.release_flags & RELEASE_FLAG_CLEAR_REFUSED, 0, "expected CLEAR_REFUSED flag on release {hot_refuse_seq}");
    assert_eq!(hot_refuse_sample.usage.lock_alloc_calls, 0, "hot clear refusal release allocated: {hot_refuse_sample:?}");

    *temp_ctl.lock().unwrap() = 50.0;
    let cool_step = handle.stats().steps.load(Ordering::Relaxed);
    wait_until_async(Duration::from_secs(2), || handle.stats().steps.load(Ordering::Relaxed) >= cool_step + 4).await;
    assert_eq!(sup.clear_fault().await.unwrap(), SafetyState::Idle);
    let cool_clear_seq = handle.stats().last_clear_release_seq.load(Ordering::Acquire);
    assert!(cool_clear_seq > hot_refuse_seq);
    let cool_clear_sample = expect_release_sample(&sup, cool_clear_seq, "cooldown clear").await;
    assert_ne!(cool_clear_sample.release_flags & RELEASE_FLAG_CLEAR_OK, 0, "expected CLEAR_OK flag on release {cool_clear_seq}");
    assert_eq!(cool_clear_sample.usage.lock_alloc_calls, 0, "cooldown clear release allocated: {cool_clear_sample:?}");

    assert_eq!(sup.trigger_e_stop_checked().await.unwrap().0, SafetyState::EStop);
    let estop_seq = handle.stats().last_estop_release_seq.load(Ordering::Acquire);
    let estop_sample = expect_release_sample(&sup, estop_seq, "E-stop latch").await;
    assert_ne!(estop_sample.release_flags & RELEASE_FLAG_ESTOP_APPLIED, 0, "expected ESTOP_APPLIED flag on release {estop_seq}");
    assert_eq!(estop_sample.usage.lock_alloc_calls, 0, "E-stop latch release allocated: {estop_sample:?}");

    assert_eq!(sup.clear_fault().await.unwrap(), SafetyState::Idle);
    let estop_clear_seq = handle.stats().last_clear_release_seq.load(Ordering::Acquire);
    assert!(estop_clear_seq > cool_clear_seq);
    let estop_clear_sample = expect_release_sample(&sup, estop_clear_seq, "E-stop clear").await;
    assert_ne!(estop_clear_sample.release_flags & RELEASE_FLAG_CLEAR_OK, 0, "expected CLEAR_OK flag on release {estop_clear_seq}");
    assert_eq!(estop_clear_sample.usage.lock_alloc_calls, 0, "E-stop clear release allocated: {estop_clear_sample:?}");
    assert_eq!(handle.stats().wake_alloc_calls.load(Ordering::Relaxed), 0);

    // Positive control in live RtExecutor: inject exactly 1 allocation and verify exact tagged release has count == 1.
    inject_once.0.store(true, Ordering::Release);
    wait_until_async(Duration::from_secs(2), || !inject_once.0.load(Ordering::Acquire)).await;
    let inject_seq = handle.stats().last_inject_alloc_release_seq.load(Ordering::Acquire);
    let inject_sample = expect_release_sample(&sup, inject_seq, "injected alloc positive control").await;
    assert_ne!(inject_sample.release_flags & RELEASE_FLAG_INJECT_ALLOC, 0, "expected INJECT_ALLOC flag on release {inject_seq}");
    assert_eq!(
        inject_sample.usage.lock_alloc_calls,
        1,
        "positive control: 1 injected release alloc must measure lock_alloc_calls == 1 on tagged release {inject_seq}: {inject_sample:?}"
    );
    let leaked_ptr = inject_once.1.swap(std::ptr::null_mut(), Ordering::AcqRel);
    assert!(!leaked_ptr.is_null(), "injected allocation pointer must be stored for reclamation");
    // SAFETY: pointer was produced once by `Box::into_raw(Box::new(0xB3_u64))` and reclaimed once here.
    unsafe { drop(Box::from_raw(leaked_ptr)); }

    handle.stop_and_join();
    let rt_text = take_global_logs(log);
    assert!(
        rt_text.contains("ThermalTrip")
            && rt_text.contains("Safety: Idle -> EStop")
            && rt_text.contains("Safety: cleared -> Idle"),
        "expected ThermalTrip, E-stop, and clear log lines off-thread:\n{rt_text}"
    );
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
    let stamp = inner.current_admission_stamp();
    let adm = inner.check_admission(stamp).unwrap();
    armd::SupervisorInner::enable_torque(adm).unwrap();
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
    to.feedback_numeric_valid = true;
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
    let res = sup.clear_fault().await;
    let refuse_seq = handle.stats().last_clear_release_seq.load(Ordering::Acquire);
    let api = ApiError::from(res.unwrap_err());
    assert_eq!((api.status, api.code.as_deref()), (StatusCode::CONFLICT, Some("CLEAR_FAULT_REJECTED")));
    assert!(api.message.starts_with("Cannot disable faulted supporting arm joint away from resting pose ("), "{}", api.message);
    assert!(api.message.ends_with("); support the arm at rest first"), "{}", api.message);
    let refuse_sample = expect_release_sample(&sup, refuse_seq, "resting-pose clear refusal").await;
    assert_ne!(refuse_sample.release_flags & RELEASE_FLAG_CLEAR_REFUSED, 0);
    assert_eq!(refuse_sample.usage.lock_alloc_calls, 0, "resting-pose clear refusal release allocated: {refuse_sample:?}");
    assert_eq!(handle.stats().wake_alloc_calls.load(Ordering::Relaxed), 0);
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

            // Current Arc stays the existing fault (no Arc::new), while BOTH kinds are recorded,
            // reported fault code reflects the higher-severity ThermalTrip, and `recoverable()` is false.
            assert_eq!(*inner.safety.state(), SafetyState::Fault(initial_reason.clone()));
            assert!(inner.safety.has_latched_fault_kind(initial_kind));
            assert!(inner.safety.has_latched_fault_kind(FaultKind::ThermalTrip));
            assert_eq!(inner.safety.fault_code(), Some(FaultCode::ThermalTrip));
            assert!(!inner.safety.recoverable());
            assert!(!inner.safety.fault().unwrap().recoverable);

            // Every explicit latch (including a second ThermalTrip) advances epoch_id with 0 heap ops.
            let ep_after_trip = inner.safety.epoch_id();
            let _ = alloc::take();
            inner.apply_stop_policy(StopPolicy::ThermalTrip);
            let repeat_allocs = alloc::take();
            if use_rt {
                assert_eq!(repeat_allocs.calls, 0, "repeat ThermalTrip latch must do 0 heap ops: {repeat_allocs:?}");
            }
            assert_eq!(inner.safety.epoch_id(), ep_after_trip + 1);

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
    assert!(sm.has_latched_fault_kind(FaultKind::JointLimit));
    assert!(matches!(
        sm.latched_fault_detail(FaultKind::JointLimit).map(|d| d.reason),
        Some(BoundedFaultReason::JointLimitExceeded { joint_idx: 2, .. })
    ));

    // 2. Latch EVERY remaining FaultKind via `latch` while all pools stay completely empty (0 heap ops).
    let _ = alloc::take();
    let res_th = sm.try_trigger_fault_coded(
        FaultCode::ThermalTrip,
        FaultReason::ThermalTrip { joint_idx: 1, temperature: 92.0, limit: 80.0 },
    );
    let res_bus = sm.try_trigger_fault_coded(FaultCode::BusLoss, FaultReason::BusLoss);
    let res_wd = sm.try_trigger_fault_coded(FaultCode::WatchdogTimeout, FaultReason::WatchdogTimeout);
    let res_tx = sm.try_trigger_fault_coded(FaultCode::CanTxStalled, FaultReason::CanTxStalled);
    let res_j7 = sm.try_trigger_fault_coded(FaultCode::J7FeedbackStale, FaultReason::J7FeedbackStale);
    let res_other = sm.try_trigger_fault_coded(FaultCode::HardwareFault, other);
    sm.latch(
        FaultKind::CollisionLatched,
        FaultDetail::formatted(FaultCode::Collision, |b| b.push_str("collision escalated")),
    );
    sm.latch(FaultKind::Estop, FaultDetail::Estop);
    let rest_allocs = alloc::take();
    assert_eq!(rest_allocs.calls, 0, "latching all fault kinds with empty pool must do 0 heap ops: {rest_allocs:?}");

    assert!(res_bus.is_ok() && res_wd.is_ok() && res_tx.is_ok() && res_j7.is_ok(), "cached BusLoss/WatchdogTimeout/CanTxStalled/J7FeedbackStale arcs do not need pool slots");
    let (_, ret_jl) = res_jl.unwrap_err();
    let (_, ret_th) = res_th.unwrap_err();
    let (_, ret_other) = res_other.unwrap_err();

    for kind in FaultKind::ALL {
        assert!(sm.has_latched_fault_kind(kind), "missing latched kind {kind:?}");
        assert!(sm.latched_fault_detail(kind).is_some(), "missing detail for {kind:?}");
    }
    assert_eq!(sm.fault_code(), Some(FaultCode::ThermalTrip), "most severe recorded kind must win reported code");
    assert!(!sm.recoverable());

    // 2b. J7_FEEDBACK_STALE (cached arc) followed by ThermalTrip with exhausted pools matches
    //     the baseline pool-exhaustion contract (`test_rt_b3_alloc.rs:680`): 0 heap ops, existing
    //     stopped Arc preserved without allocation, BOTH kinds recorded, `fault_code()` and
    //     `fault()` report `ThermalTrip`, and `recoverable()` becomes `false`.
    let mut sm_j7 = SafetyManager::new();
    sm_j7.set_off_thread_reclaim(true);
    let held_j7 = sm_j7.drain_fault_arc_pool_for_test();
    let _ = alloc::take();
    sm_j7.latch(FaultKind::J7FeedbackStale, FaultDetail::J7FeedbackStale(None));
    assert_eq!(*sm_j7.state(), SafetyState::Fault(FaultReason::J7FeedbackStale));
    sm_j7.latch(
        FaultKind::ThermalTrip,
        FaultDetail::ThermalTrip { joint_idx: 0, temperature: 91.0, limit: 75.0 },
    );
    let j7_upgrade_allocs = alloc::take();
    assert_eq!(j7_upgrade_allocs.calls, 0, "J7 -> ThermalTrip with empty pool must do 0 heap ops");
    assert_eq!(*sm_j7.state(), SafetyState::Fault(FaultReason::J7FeedbackStale));
    assert!(sm_j7.has_latched_fault_kind(FaultKind::J7FeedbackStale));
    assert!(sm_j7.has_latched_fault_kind(FaultKind::ThermalTrip));
    assert_eq!(sm_j7.fault_code(), Some(FaultCode::ThermalTrip));
    assert_eq!(sm_j7.fault().unwrap().code, FaultCode::ThermalTrip);
    assert!(!sm_j7.recoverable());
    assert!(!sm_j7.fault().unwrap().recoverable);
    drop(held_j7);

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
    let stamp = gc.current_admission_stamp();
    let adm = gc.check_admission(stamp).unwrap();
    armd::SupervisorInner::enable_torque(adm).unwrap();
    gc.tick(0.02);
    let stamp = gc.current_admission_stamp();
    let adm = gc.check_admission(stamp).unwrap();
    armd::SupervisorInner::enter_gravity_comp(adm, armd::gravity_comp::GravityCompRequest::default()).unwrap();
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

// ============================================================================
// B3 round 3: one latch, one clear_check, one recoverability rule, inline allocs
// ============================================================================

#[test]
fn clear_check_table_over_every_fault_kind_and_unknown_bit_fail_closed() {
    let base_met = ClearContext::all_conditions_met();
    let driver_met = ClearContext { has_driver: true, ..base_met };

    // 1. Exhaustive table covering EVERY FaultKind variant: condition unmet -> refused; met -> cleared.
    let table: Vec<(
        FaultKind,
        FaultDetail,
        ClearContext,
        ClearRefusal,
        ClearContext,
    )> = vec![
        // ThermalTrip: above threshold
        (
            FaultKind::ThermalTrip,
            FaultDetail::ThermalTrip { joint_idx: 0, temperature: 86.0, limit: 75.0 },
            ClearContext { thermal_below_threshold: false, ..driver_met },
            ClearRefusal::TemperatureConfirmationRequired,
            driver_met,
        ),
        // ThermalTrip: below threshold, but cooldown dwell time not elapsed
        (
            FaultKind::ThermalTrip,
            FaultDetail::ThermalTrip { joint_idx: 0, temperature: 86.0, limit: 75.0 },
            ClearContext {
                thermal_below_threshold: true,
                thermal_cooldown_elapsed_ms: 400,
                thermal_cooldown_required_ms: 1000,
                ..driver_met
            },
            ClearRefusal::ThermalCooldownNotElapsed { elapsed_ms: 400, required_ms: 1000 },
            ClearContext {
                thermal_below_threshold: true,
                thermal_cooldown_elapsed_ms: 1000,
                thermal_cooldown_required_ms: 1000,
                ..driver_met
            },
        ),
        // BusLoss: stale/missing feedback
        (
            FaultKind::BusLoss,
            FaultDetail::BusLoss,
            ClearContext { feedback_fresh: false, ..driver_met },
            ClearRefusal::FreshFeedbackRequired,
            driver_met,
        ),
        // CanTxStalled: pending frames in TX queue -> refused; drained + fresh feedback -> cleared
        (
            FaultKind::CanTxStalled,
            FaultDetail::CanTxStalled(Some(armd::safety::TxStallDetail {
                queued: 4,
                dropped: 0,
                age_ms: 60,
                limit_ms: 45,
                consecutive_ticks: 3,
                is_hold: false,
            })),
            ClearContext { tx_queue_drained: false, ..driver_met },
            ClearRefusal::TxQueueNotDrained,
            driver_met,
        ),
        // CanTxStalled: drained queue but stale/missing feedback -> refused; both met -> cleared
        (
            FaultKind::CanTxStalled,
            FaultDetail::CanTxStalled(None),
            ClearContext { feedback_fresh: false, tx_queue_drained: true, ..driver_met },
            ClearRefusal::FreshFeedbackRequired,
            driver_met,
        ),
        // CanTxStalled: drained queue + fresh feedback, no TX probe yet -> TxProbeRequired
        (
            FaultKind::CanTxStalled,
            FaultDetail::CanTxStalled(None),
            ClearContext { tx_probe_confirmed: false, tx_probe_sent: false, ..driver_met },
            ClearRefusal::TxProbeRequired,
            driver_met,
        ),
        // CanTxStalled: TX probe sent, awaiting post-probe feedback -> TxProbeFeedbackRequired
        (
            FaultKind::CanTxStalled,
            FaultDetail::CanTxStalled(None),
            ClearContext { tx_probe_confirmed: false, tx_probe_sent: true, ..driver_met },
            ClearRefusal::TxProbeFeedbackRequired,
            driver_met,
        ),
        // WatchdogTimeout: active lease with stale heartbeat -> refused; fresh heartbeat -> cleared
        (
            FaultKind::WatchdogTimeout,
            FaultDetail::WatchdogTimeout,
            ClearContext { has_active_lease: true, lease_heartbeat_fresh: false, ..base_met },
            ClearRefusal::WatchdogHeartbeatRequired,
            ClearContext { has_active_lease: true, lease_heartbeat_fresh: true, ..base_met },
        ),
        // WatchdogTimeout: no active lease -> cleared
        (
            FaultKind::WatchdogTimeout,
            FaultDetail::WatchdogTimeout,
            ClearContext { has_active_lease: true, lease_heartbeat_fresh: false, ..base_met },
            ClearRefusal::WatchdogHeartbeatRequired,
            ClearContext { has_active_lease: false, lease_heartbeat_fresh: false, ..base_met },
        ),
        // JointLimit: measured joints outside limits (plus margin) -> refused; back within limits -> cleared
        (
            FaultKind::JointLimit,
            FaultDetail::JointLimitExceeded {
                joint_idx: 1,
                joint_name: BoundedText::new("joint_2"),
                value: 3.5,
                min: -2.0,
                max: 2.0,
            },
            ClearContext {
                joint_limit_violation: Some(armd::safety::JointLimitClearViolation::OutOfBounds {
                    joint_idx: 1,
                    joint_name: BoundedText::new("joint_2"),
                    value: 3.5,
                    min: -2.0,
                    max: 2.0,
                    margin: armd::supervisor::JOINT_LIMIT_CLEAR_MARGIN_RAD,
                }),
                ..base_met
            },
            ClearRefusal::JointLimitsNotSatisfied(armd::safety::JointLimitClearViolation::OutOfBounds {
                joint_idx: 1,
                joint_name: BoundedText::new("joint_2"),
                value: 3.5,
                min: -2.0,
                max: 2.0,
                margin: armd::supervisor::JOINT_LIMIT_CLEAR_MARGIN_RAD,
            }),
            ClearContext { joint_limit_violation: None, ..base_met },
        ),
        // CollisionLatched: keeps existing rule (enable state unknown / hw fault / stale feedback)
        (
            FaultKind::CollisionLatched,
            FaultDetail::formatted(FaultCode::Collision, |b| b.push_str("COLLISION: escalated")),
            ClearContext { enable_state_known: false, ..driver_met },
            ClearRefusal::EnableStateUnknown,
            driver_met,
        ),
        // ESTOP: keeps existing rule (fresh feedback / enable state known)
        (
            FaultKind::Estop,
            FaultDetail::Estop,
            ClearContext { feedback_fresh: false, ..driver_met },
            ClearRefusal::FreshFeedbackRequired,
            driver_met,
        ),
        // J7FeedbackStale: requires provably fresh J7 pos-force feedback (<= 100 ms)
        (
            FaultKind::J7FeedbackStale,
            FaultDetail::J7FeedbackStale(None),
            ClearContext { j7_feedback_fresh: false, ..driver_met },
            ClearRefusal::J7FeedbackStale,
            driver_met,
        ),
        // Other: keeps existing rule (active hardware error)
        (
            FaultKind::Other,
            FaultDetail::Static(StaticFault::InvalidTelemetry),
            ClearContext {
                hardware_error: Some(ClearFaultDriverError::JointTimeout(1)),
                ..driver_met
            },
            ClearRefusal::DriverError(ClearFaultDriverError::JointTimeout(1)),
            driver_met,
        ),
    ];

    // Confirm EVERY FaultKind::ALL variant appears in the table.
    for kind in FaultKind::ALL {
        assert!(
            table.iter().any(|(k, ..)| *k == kind),
            "clear_check table missing FaultKind::{kind:?}"
        );
    }

    for (kind, detail, unmet, expected_refusal, met) in table {
        let mut sm = SafetyManager::new();
        sm.latch(kind, detail);
        assert!(sm.has_latched_fault_kind(kind));
        assert_eq!(sm.clear_check(&unmet), Err(expected_refusal), "kind={kind:?} unmet must refuse");
        assert_eq!(
            sm.clear_with_check(&unmet),
            Err(expected_refusal),
            "kind={kind:?} clear_with_check unmet must not reset record"
        );
        assert!(sm.has_latched_fault_kind(kind), "kind={kind:?} must remain latched after refusal");
        assert_eq!(sm.clear_check(&met), Ok(()), "kind={kind:?} met must pass clear_check");
        assert_eq!(sm.clear_with_check(&met), Ok(SafetyState::Idle), "kind={kind:?} met must clear");
        assert_eq!(sm.latched_fault_kinds(), 0, "kind={kind:?} record must be fully reset");
    }

    // 2. Fail closed on any unknown kind bit without an explicit rule.
    let mut sm_unknown = SafetyManager::new();
    sm_unknown.latch(FaultKind::WatchdogTimeout, FaultDetail::WatchdogTimeout);
    sm_unknown.record_unknown_kind_bit_for_test(1u16 << FaultKind::COUNT);
    assert!(!sm_unknown.recoverable(), "unknown kind bit must be non-recoverable (fail closed)");
    assert_eq!(sm_unknown.clear_check(&base_met), Err(ClearRefusal::UnknownFaultKind));
    assert_eq!(sm_unknown.clear_with_check(&base_met), Err(ClearRefusal::UnknownFaultKind));

    // 3. Supervisor end-to-end checks for JointLimit, WatchdogTimeout heartbeat, and ThermalTrip cooldown dwell.
    let mut cfg = ArmConfig::rebot_dm();
    cfg.max_temperature = 75.0;
    cfg.lease_timeout_ms = 30;
    let mut inner = SupervisorInner::new(cfg.clone());
    inner.thermal_cooldown_duration = Duration::from_millis(40);
    inner.temperatures = vec![30.0; cfg.dof];
    // JointLimit: out of bounds -> refused naming joint/position/limits; back within limits + margin -> cleared.
    let j0_lim = inner.config.limits[0].clone();
    let over_q = j0_lim.max_position + 0.2;
    inner.measured_positions[0] = over_q;
    inner.safety.latch(
        FaultKind::JointLimit,
        FaultDetail::formatted(FaultCode::JointLimit, |b| b.push_str("joint 1 out of bounds")),
    );
    let jl_err = inner.clear_fault().unwrap_err().to_string();
    assert!(
        jl_err.contains(&j0_lim.name)
            && jl_err.contains(&format!("{over_q:.4}"))
            && jl_err.contains(&format!("{:.4}", j0_lim.min_position))
            && jl_err.contains(&format!("{:.4}", j0_lim.max_position))
            && jl_err.contains("within limits"),
        "JointLimit refusal must name joint, position, and limits: {jl_err}"
    );
    assert_eq!(
        inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)),
        Err(ClearFaultError::Rejected(ClearRefusal::JointLimitsNotSatisfied(
            armd::safety::JointLimitClearViolation::OutOfBounds {
                joint_idx: 0,
                joint_name: BoundedText::new(&j0_lim.name),
                value: over_q,
                min: j0_lim.min_position,
                max: j0_lim.max_position,
                margin: armd::supervisor::JOINT_LIMIT_CLEAR_MARGIN_RAD,
            }
        )))
    );
    // Just inside outward margin (0.005 < JOINT_LIMIT_CLEAR_MARGIN_RAD = 0.01) -> clears.
    inner.measured_positions[0] = j0_lim.max_position + 0.005;
    assert_eq!(inner.clear_fault().unwrap(), SafetyState::Idle);

    // WatchdogTimeout: active (non-expired) lease with stale heartbeat -> refused; fresh heartbeat or expired lease -> cleared.
    let _gen = inner.acquire_lease("client-w").unwrap();
    inner.safety.latch(FaultKind::WatchdogTimeout, FaultDetail::WatchdogTimeout);
    inner.last_heartbeat = Instant::now() - Duration::from_millis(45);
    assert!(inner.is_lease_active(), "lease must still be active when testing stale heartbeat");
    assert_eq!(
        inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)),
        Err(ClearFaultError::Rejected(ClearRefusal::WatchdogHeartbeatRequired))
    );
    assert!(inner.clear_fault().unwrap_err().to_string().contains("Fresh heartbeat"));
    // Once the lease expires (acquired_at >= lease_timeout), it counts as NO lease -> clears!
    std::thread::sleep(Duration::from_millis(45));
    assert!(!inner.is_lease_active());
    assert_eq!(inner.clear_fault().unwrap(), SafetyState::Idle);

    // ThermalTrip with non-zero cooldown dwell: hot -> refused, cool < 40 ms -> refused, cool >= 40 ms -> cleared.
    inner.temperatures[0] = 85.0;
    inner.apply_stop_policy(StopPolicy::ThermalTrip);
    assert!(inner.clear_fault().is_err());
    inner.temperatures[0] = 50.0;
    assert_eq!(
        inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)),
        Err(ClearFaultError::Rejected(ClearRefusal::ThermalCooldownNotElapsed {
            elapsed_ms: 0,
            required_ms: 40,
        }))
    );
    std::thread::sleep(Duration::from_millis(50));
    assert_eq!(inner.clear_fault().unwrap(), SafetyState::Idle);
}

#[test]
fn paired_fault_kinds_refuse_clear_until_both_conditions_hold_and_report_most_severe() {
    let driver_met = ClearContext { has_driver: true, ..ClearContext::all_conditions_met() };

    // Pair 1: ThermalTrip + BusLoss (both latch orders, unit clear_check + live detector without skip!)
    for thermal_first in [true, false] {
        let mut sm = SafetyManager::new();
        if thermal_first {
            sm.latch(FaultKind::ThermalTrip, FaultDetail::ThermalTrip { joint_idx: 0, temperature: 88.0, limit: 75.0 });
            sm.latch(FaultKind::BusLoss, FaultDetail::BusLoss);
        } else {
            sm.latch(FaultKind::BusLoss, FaultDetail::BusLoss);
            sm.latch(FaultKind::ThermalTrip, FaultDetail::ThermalTrip { joint_idx: 0, temperature: 88.0, limit: 75.0 });
        }
        assert_eq!(sm.fault_code(), Some(FaultCode::ThermalTrip), "ThermalTrip outranks BusLoss");
        assert!(!sm.recoverable());
        // Only bus fresh (still hot) -> refused
        assert_eq!(
            sm.clear_check(&ClearContext { thermal_below_threshold: false, ..driver_met }),
            Err(ClearRefusal::TemperatureConfirmationRequired)
        );
        // Only cooled (bus stale) -> refused
        assert_eq!(
            sm.clear_check(&ClearContext { feedback_fresh: false, ..driver_met }),
            Err(ClearRefusal::FreshFeedbackRequired)
        );
        // Both hold -> cleared
        assert_eq!(sm.clear_with_check(&driver_met), Ok(SafetyState::Idle));
    }

    // Real supervisor detector check: BusLoss detector does NOT skip when ThermalTrip is already active!
    let mut cfg = ArmConfig::rebot_dm();
    cfg.max_temperature = 75.0;
    let temp = Arc::new(Mutex::new(85.0f32));
    let bus_age = Arc::new(Mutex::new(Some(Duration::from_millis(10))));
    struct AgeTempDriver {
        dof: usize,
        temp: Arc<Mutex<f32>>,
        age: Arc<Mutex<Option<Duration>>>,
    }
    impl ArmDriver for AgeTempDriver {
        fn capabilities(&self) -> DriverCapabilities {
            DriverCapabilities {
                velocity_feedback: true, torque_feedback: false, temperature_feedback: true,
                torque_feedforward: false, enable_feedback: true, load_feedback: false,
            }
        }
        fn enable_state(&self) -> EnableState { EnableState::Enabled }
        fn is_feedback_valid(&self) -> bool { self.age.lock().unwrap().is_some_and(|a| a <= Duration::from_millis(250)) }
        fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![*self.age.lock().unwrap(); self.dof] }
        fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
            self.dof == dof && self.age.lock().unwrap().is_some_and(|a| a <= max_age)
        }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            let t = *self.temp.lock().unwrap();
            Ok((0..self.dof).map(|_| JointState { temperature: t, ..JointState::default() }).collect())
        }
        fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
        fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
    }
    let mut sup_inner = SupervisorInner::new(cfg.clone()).with_driver(Box::new(AgeTempDriver {
        dof: cfg.dof,
        temp: Arc::clone(&temp),
        age: Arc::clone(&bus_age),
    }));
    sup_inner.lease_enabled = false;
    sup_inner.is_torqued = true;
    sup_inner.feedback_numeric_valid = true;
    let _ = sup_inner.safety.set_torque(true).unwrap();
    sup_inner.tick(0.02); // latches ThermalTrip
    assert!(sup_inner.safety.has_latched_fault_kind(FaultKind::ThermalTrip));
    // Now bus drops while ThermalTrip is already active -> BusLoss detector MUST still latch BusLoss!
    *bus_age.lock().unwrap() = Some(Duration::from_millis(700));
    sup_inner.tick(0.02);
    assert!(sup_inner.safety.has_latched_fault_kind(FaultKind::ThermalTrip));
    assert!(sup_inner.safety.has_latched_fault_kind(FaultKind::BusLoss), "bus-loss detector must not skip when ThermalTrip is active");
    assert_eq!(sup_inner.safety.fault_code(), Some(FaultCode::ThermalTrip));
    // Clear disable_pending for clear_fault test (driver stayed enabled in mock)
    sup_inner.disable_pending = false;
    // Cool temperature while bus still stale -> refused!
    *temp.lock().unwrap() = 50.0;
    assert!(sup_inner.clear_fault().is_err(), "ThermalTrip+BusLoss must stay refused while bus stale");
    // Restore bus while hot -> refused!
    *bus_age.lock().unwrap() = Some(Duration::from_millis(10));
    *temp.lock().unwrap() = 85.0;
    assert!(sup_inner.clear_fault().is_err(), "ThermalTrip+BusLoss must stay refused while hot");
    // Both cooled and fresh -> clears!
    *temp.lock().unwrap() = 50.0;
    assert_eq!(sup_inner.clear_fault().unwrap(), SafetyState::Idle);

    // Pair 2: ThermalTrip + WatchdogTimeout (both latch orders)
    for thermal_first in [true, false] {
        let mut sm = SafetyManager::new();
        if thermal_first {
            sm.latch(FaultKind::ThermalTrip, FaultDetail::ThermalTrip { joint_idx: 0, temperature: 85.0, limit: 75.0 });
            sm.latch(FaultKind::WatchdogTimeout, FaultDetail::WatchdogTimeout);
        } else {
            sm.latch(FaultKind::WatchdogTimeout, FaultDetail::WatchdogTimeout);
            sm.latch(FaultKind::ThermalTrip, FaultDetail::ThermalTrip { joint_idx: 0, temperature: 85.0, limit: 75.0 });
        }
        assert_eq!(sm.fault_code(), Some(FaultCode::ThermalTrip));
        assert!(!sm.recoverable());
        // Cooled, but active lease heartbeat stale -> refused
        assert_eq!(
            sm.clear_check(&ClearContext { has_active_lease: true, lease_heartbeat_fresh: false, ..driver_met }),
            Err(ClearRefusal::WatchdogHeartbeatRequired)
        );
        // Fresh heartbeat, but still hot -> refused
        assert_eq!(
            sm.clear_check(&ClearContext {
                has_active_lease: true,
                lease_heartbeat_fresh: true,
                thermal_below_threshold: false,
                ..driver_met
            }),
            Err(ClearRefusal::TemperatureConfirmationRequired)
        );
        // Both hold -> cleared
        assert_eq!(
            sm.clear_with_check(&ClearContext { has_active_lease: true, lease_heartbeat_fresh: true, ..driver_met }),
            Ok(SafetyState::Idle)
        );
    }

    // Pair 3: JointLimit + BusLoss (both latch orders)
    for jl_first in [true, false] {
        let mut sm = SafetyManager::new();
        let jl_detail = FaultDetail::JointLimitExceeded {
            joint_idx: 2,
            joint_name: BoundedText::new("joint_3"),
            value: 4.0,
            min: -2.0,
            max: 2.0,
        };
        if jl_first {
            sm.latch(FaultKind::JointLimit, jl_detail);
            sm.latch(FaultKind::BusLoss, FaultDetail::BusLoss);
        } else {
            sm.latch(FaultKind::BusLoss, FaultDetail::BusLoss);
            sm.latch(FaultKind::JointLimit, jl_detail);
        }
        assert_eq!(sm.fault_code(), Some(FaultCode::BusLoss), "BusLoss outranks JointLimit");
        assert!(!sm.recoverable());
        let jl_violation = armd::safety::JointLimitClearViolation::OutOfBounds {
            joint_idx: 0,
            joint_name: BoundedText::new("joint_1"),
            value: 3.2,
            min: -2.0,
            max: 2.0,
            margin: armd::supervisor::JOINT_LIMIT_CLEAR_MARGIN_RAD,
        };
        // Bus fresh, joints still outside limits -> refused
        assert_eq!(
            sm.clear_check(&ClearContext { joint_limit_violation: Some(jl_violation), ..driver_met }),
            Err(ClearRefusal::JointLimitsNotSatisfied(jl_violation))
        );
        // Joints back within limits, bus still stale -> refused
        assert_eq!(
            sm.clear_check(&ClearContext { feedback_fresh: false, ..driver_met }),
            Err(ClearRefusal::FreshFeedbackRequired)
        );
        // Both hold -> cleared
        assert_eq!(sm.clear_with_check(&driver_met), Ok(SafetyState::Idle));
    }

    // Pair 4: ThermalTrip + JointLimit
    let jl_violation = armd::safety::JointLimitClearViolation::OutOfBounds {
        joint_idx: 0,
        joint_name: BoundedText::new("joint_1"),
        value: 3.2,
        min: -2.0,
        max: 2.0,
        margin: armd::supervisor::JOINT_LIMIT_CLEAR_MARGIN_RAD,
    };
    let mut sm = SafetyManager::new();
    sm.latch(FaultKind::JointLimit, FaultDetail::formatted(FaultCode::JointLimit, |b| b.push_str("jl")));
    sm.latch(FaultKind::ThermalTrip, FaultDetail::ThermalTrip { joint_idx: 0, temperature: 90.0, limit: 75.0 });
    assert_eq!(sm.fault_code(), Some(FaultCode::ThermalTrip));
    assert_eq!(
        sm.clear_check(&ClearContext { joint_limit_violation: Some(jl_violation), ..driver_met }),
        Err(ClearRefusal::JointLimitsNotSatisfied(jl_violation))
    );
    assert_eq!(
        sm.clear_check(&ClearContext { thermal_below_threshold: false, ..driver_met }),
        Err(ClearRefusal::TemperatureConfirmationRequired)
    );
    assert_eq!(sm.clear_with_check(&driver_met), Ok(SafetyState::Idle));
}

#[test]
fn mit_expiry_recovery_with_thermal_trip_recorded_is_refused_until_cooldown() {
    use armd::mit::{self, MitJointQualification, MitQualificationFile};
    use armd::protocol::damiao;
    use armd::tracking::{JointTrackingConfig, TrackingStrategy};

    const DT: f64 = 0.02;
    let clock = MockClock::new();
    let limits = dm_limits();
    let bus = FakeBus::with_clock(&limits, clock.clone());
    bus.with(|s| {
        s.motors[3].set_reg_u32(damiao::RID_TIMEOUT, 2000);
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits.clone())
        .unwrap();
    drv.read_joint_states().unwrap();
    drv.check_mapping_ranges();

    let mut jt = JointTrackingConfig::rebot_dm();
    jt.enabled = true;
    for j in &mut jt.joints { j.strategy = TrackingStrategy::Position; }
    let j4 = &mut jt.joints[3];
    j4.strategy = TrackingStrategy::Mit;
    j4.kp = 10.0;
    j4.kd = 0.3;
    j4.hold_kp = Some(12.0);
    j4.hold_kd = Some(0.4);
    j4.effort_limit_nm = 3.0;
    j4.gravity_scale = 1.0;
    let mut cfg = ArmConfig {
        lease_timeout_ms: 600_000,
        workspace: None,
        joint_tracking: Some(jt),
        max_temperature: 75.0,
        ..ArmConfig::rebot_dm()
    };
    cfg.max_temperature = 75.0;

    let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
    let l4 = limits[3];
    let rec = MitQualificationFile {
        version: mit::QUALIFICATION_VERSION,
        joints: vec![MitJointQualification {
            profile_id: "rebot_b601dm".into(),
            joint: 4,
            joint_name: ArmConfig::rebot_dm().joint_names[3].clone(),
            zero_fingerprint: mit::zero_fingerprint(0.0),
            motor_id: 4,
            reply_id: 0x14,
            esc_id: 4,
            mst_id: 0x14,
            sw_ver: mit_support::SW_VER,
            pmax: l4.p_max,
            vmax: l4.v_max,
            tmax: l4.t_max,
            timeout_raw: 2000,
            timeout_unit_us: damiao::TIMEOUT_UNIT_US_DOCUMENTED,
            timeout_unit_verified: true,
            tested_response: "disabled".into(),
            measured_expiry_ms: 101.0,
            worst_latency_ms: 8.0,
            load_envelope: "unloaded, supported".into(),
            max_kp: 15.0,
            max_kd: 0.5,
            effort_ceiling_nm: 4.0,
            date: "2026-11-22T00:00:00Z".into(),
            trace_id: "b3-fix3-test".into(),
        }],
        invalidated: None,
    };
    inner.set_calibration(Some(armd::calibration::CalibrationFile {
        mit_qualification: serde_json::to_value(&rec).ok(),
        ..Default::default()
    }));
    inner.lease_enabled = false;
    let stamp = inner.current_admission_stamp();
    let adm = inner.check_admission(stamp).unwrap();
    armd::SupervisorInner::enable_torque(adm).unwrap();
    for _ in 0..3 { inner.tick(DT); }
    assert!(inner.mit_active(), "MIT should be active before injected TX failure");

    // Inject partial TX failure -> MIT enters MitOwner::Lost (BusLoss) and J4 command watchdog expires (0x0D).
    bus.with(|s| s.fail_tx_after = Some(3));
    inner.tick(DT);
    assert_eq!(inner.mit.owner, mit::MitOwner::Lost);
    bus.with(|s| s.fail_tx_after = None);
    for _ in 0..8 {
        clock.advance(Duration::from_millis(50));
        inner.tick(DT);
    }
    assert!(bus.with(|s| s.motors[3].state_override == Some(0xD) && !s.motors[3].enabled));

    // Now motor 0 overheats -> ThermalTrip is ALSO recorded while MitOwner::Lost!
    bus.with(|s| s.motors[0].temperature_c = 85);
    inner.temperatures[0] = 85.0;
    inner.safety.latch(
        FaultKind::ThermalTrip,
        FaultDetail::ThermalTrip { joint_idx: 0, temperature: 85.0, limit: 75.0 },
    );
    assert!(inner.safety.has_latched_fault_kind(FaultKind::ThermalTrip));
    assert!(inner.safety.has_latched_fault_kind(FaultKind::BusLoss));
    assert_eq!(inner.safety.fault_code(), Some(FaultCode::ThermalTrip));

    // Attempt confirmed MIT recovery while motor 0 is STILL 85°C -> MUST be refused with ZERO recovery commands!
    bus.clear_log();
    let hot_err = inner.clear_fault_confirmed(true).unwrap_err();
    assert!(
        hot_err.to_string().contains("Temperature must be confirmed below the reset threshold"),
        "MIT recovery with hot ThermalTrip must be refused by clear_check, got: {hot_err}"
    );
    let recovery_frames = bus
        .tx_log()
        .iter()
        .filter(|f| (1..=7).contains(&f.id) && [damiao::CMD_DISABLE, damiao::CMD_CLEAR_FAULTS].contains(&f.data))
        .count();
    assert_eq!(recovery_frames, 0, "hot ThermalTrip must block MIT recovery before any motor commands are sent");
    assert!(inner.safety.has_latched_fault_kind(FaultKind::ThermalTrip));
    assert!(inner.safety.has_latched_fault_kind(FaultKind::BusLoss));

    // Cool motor 0 below reset threshold -> confirmed MIT recovery now passes clear_check and clears all at once!
    bus.with(|s| s.motors[0].temperature_c = 35);
    let mut cleared = false;
    for _ in 0..6 {
        match inner.clear_fault_confirmed(true) {
            Ok(SafetyState::Idle) => { cleared = true; break; }
            Ok(other) => panic!("unexpected state after MIT recovery: {other:?}"),
            Err(_) => { clock.advance(Duration::from_millis(120)); inner.tick(DT); }
        }
    }
    assert!(cleared, "MIT recovery must succeed once ThermalTrip cooled down: {:?}", inner.clear_fault_confirmed(true));
    assert_eq!(inner.safety.latched_fault_kinds(), 0);
    assert!(!inner.is_torqued);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn recoverable_worst_of_across_record_other_details_observation_and_ws_api() {
    use futures_util::{SinkExt, StreamExt};
    use tokio_tungstenite::tungstenite::Message;

    // 1. Unit tests for worst-of recoverability across kinds AND across two `Other` details in BOTH orders.
    let mut sm = SafetyManager::new();
    sm.latch(FaultKind::WatchdogTimeout, FaultDetail::WatchdogTimeout);
    assert!(sm.recoverable());
    assert!(sm.fault().unwrap().recoverable);

    // ControlDeadline (recoverable Other) alone -> recoverable == true.
    let mut sm_cd = SafetyManager::new();
    sm_cd.latch(FaultKind::Other, FaultDetail::Static(StaticFault::ControlDeadline));
    assert!(sm_cd.recoverable());
    assert_eq!(sm_cd.fault_code(), Some(FaultCode::ControlDeadline));

    // ControlDeadline (recoverable) THEN HardwareFault (non-recoverable) -> recoverable == false!
    sm_cd.latch(
        FaultKind::Other,
        FaultDetail::formatted(FaultCode::HardwareFault, |b| b.push_str("motor fault 0x0A")),
    );
    assert!(!sm_cd.recoverable());
    assert!(!sm_cd.fault().unwrap().recoverable);
    assert_eq!(sm_cd.fault_code(), Some(FaultCode::HardwareFault));

    // HardwareFault (non-recoverable) THEN ControlDeadline (recoverable) -> MUST STAY recoverable == false!
    let mut sm_rev = SafetyManager::new();
    sm_rev.latch(
        FaultKind::Other,
        FaultDetail::formatted(FaultCode::HardwareFault, |b| b.push_str("motor fault 0x0B")),
    );
    assert!(!sm_rev.recoverable());
    sm_rev.latch(FaultKind::Other, FaultDetail::Static(StaticFault::ControlDeadline));
    assert!(!sm_rev.recoverable(), "recoverable Other after non-recoverable Other must keep recoverable == false");
    assert!(!sm_rev.fault().unwrap().recoverable);
    assert_eq!(sm_rev.fault_code(), Some(FaultCode::HardwareFault));

    // Pool exhausted + WatchdogTimeout -> ThermalTrip: observation.fault.recoverable MUST be false.
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    {
        let mut inner = sup.inner().write().await;
        let _held = inner.safety.drain_fault_arc_pool_for_test();
        inner.safety.latch(FaultKind::WatchdogTimeout, FaultDetail::WatchdogTimeout);
        assert!(inner.safety.recoverable());
        inner.safety.latch(
            FaultKind::ThermalTrip,
            FaultDetail::ThermalTrip { joint_idx: 0, temperature: 88.0, limit: 75.0 },
        );
        assert!(!inner.safety.recoverable());
        drop(_held);
    }
    let obs = sup.get_observation().await;
    let fi = obs.fault.expect("observation.fault must be present");
    assert_eq!(fi.code, FaultCode::ThermalTrip);
    assert!(!fi.recoverable, "observation.fault.recoverable must match SafetyManager::recoverable()");

    // 2. WebSocket operator-auth API uses SafetyManager::recoverable() (two Other faults in reverse order!).
    let sup_ws = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let router = armd::api::create_router(sup_ws.clone(), OperatorAuth::enforced("123456", 1800));
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });

    let (mut ws, _) = tokio_tungstenite::connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
    ws.send(Message::Text(serde_json::json!({"type":"acquire_lease","seq":1,"client_id":"prog"}).to_string().into())).await.unwrap();
    let lease_gen = loop {
        let txt = ws.next().await.unwrap().unwrap().into_text().unwrap();
        let v: serde_json::Value = serde_json::from_str(&txt).unwrap();
        if v["type"] == "lease_granted" { break v["generation"].as_u64().unwrap(); }
    };

    // Latch non-recoverable HardwareFault, THEN recoverable ControlDeadline under FaultKind::Other.
    {
        let mut inner = sup_ws.inner().write().await;
        inner.safety.latch(
            FaultKind::Other,
            FaultDetail::formatted(FaultCode::HardwareFault, |b| b.push_str("motor overcurrent")),
        );
        inner.safety.latch(FaultKind::Other, FaultDetail::Static(StaticFault::ControlDeadline));
        assert!(!inner.safety.recoverable());
    }
    let ep = sup_ws.get_observation().await.epoch_id;
    let cmd = serde_json::json!({
        "type": "command", "request_id": 1, "path": "/api/v1/safety/clear_fault",
        "body": {}, "generation": lease_gen, "epoch_id": ep, "client_id": "prog"
    });
    ws.send(Message::Text(cmd.to_string().into())).await.unwrap();
    let reply = loop {
        let txt = ws.next().await.unwrap().unwrap().into_text().unwrap();
        let v: serde_json::Value = serde_json::from_str(&txt).unwrap();
        if v["type"] == "command_reject" || v["type"] == "command_ack" { break v; }
    };
    assert_eq!(reply["type"], "command_reject", "{reply}");
    assert_eq!(reply["code"], "OPERATOR_REQUIRED", "WS API must use SafetyManager::recoverable(): {reply}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn watchdog_latch_and_executor_clear_retry_zero_alloc_on_tagged_live_releases() {
    let _serial = global_log().guard.lock().await;

    // Part A: Live RtExecutor watchdog latching (`latch_watchdog_fault`) -> 0 heap ops on the SPECIFIC tagged release.
    let mut wd_cfg = ArmConfig::rebot_dm();
    wd_cfg.resting_pose = None;
    wd_cfg.joint_tracking = None;
    wd_cfg.lease_timeout_ms = 25;
    let sup_wd = Arc::new(ArmSupervisor::new(wd_cfg.clone()));
    {
        let mut g = sup_wd.inner().write().await;
        g.lease_enabled = true;
        g.is_torqued = true;
        g.feedback_numeric_valid = true;
        g.measured_positions = wd_cfg.home_positions.clone();
        g.commanded_positions = wd_cfg.home_positions.clone();
        g.target_positions = wd_cfg.home_positions.clone();
        g.measured_velocities = vec![0.0; g.config.dof];
        g.temperatures = vec![30.0; g.config.dof];
        let _ = g.safety.set_torque(true).unwrap();
        g.last_heartbeat = Instant::now();
    }
    let mut exec_cfg = ExecutorConfig::for_tests(Duration::from_millis(4));
    exec_cfg.alloc_timing = true;
    let handle_wd = sup_wd.start_executor(exec_cfg.clone()).await.unwrap();
    wait_until_async(Duration::from_secs(2), || {
        sup_wd.hub().latest().is_some_and(|s| matches!(s.obs.safety_state, SafetyState::Fault(FaultReason::WatchdogTimeout)))
            && handle_wd.stats().last_fault_latch_release_seq.load(Ordering::Acquire) > 0
    }).await;
    let wd_seq = handle_wd.stats().last_fault_latch_release_seq.load(Ordering::Acquire);
    let wd_sample = expect_release_sample(&sup_wd, wd_seq, "watchdog latch").await;
    assert_ne!(wd_sample.release_flags & RELEASE_FLAG_WATCHDOG_LATCH, 0, "expected WATCHDOG_LATCH flag on release {wd_seq}");
    assert_eq!(wd_sample.usage.lock_alloc_calls, 0, "watchdog latch release allocated: {wd_sample:?}");
    assert_eq!(handle_wd.stats().wake_alloc_calls.load(Ordering::Relaxed), 0);
    handle_wd.stop_and_join();

    // Part B: Live RtExecutor clear retry with `driver_config_error` (`reapply_driver_limits_bounded`)
    // -> 0 heap ops on BOTH the refused retry release AND the resolved clear release!
    struct DynamicRtLimitsDriver {
        dof: usize,
        pos_limits: Arc<Mutex<Vec<Option<(f32, f32)>>>>,
        clock: MockClock,
    }
    impl ArmDriver for DynamicRtLimitsDriver {
        fn capabilities(&self) -> DriverCapabilities {
            DriverCapabilities {
                velocity_feedback: true, torque_feedback: false, temperature_feedback: false,
                torque_feedforward: false, enable_feedback: true, load_feedback: false,
            }
        }
        fn enable_state(&self) -> EnableState { EnableState::Disabled }
        fn is_feedback_valid(&self) -> bool { true }
        fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); self.dof] }
        fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
            out.clear(); for _ in 0..self.dof { out.push(Some(Duration::ZERO)); }
        }
        fn feedback_all_within(&self, dof: usize, _: Duration) -> bool { self.dof == dof }
        fn feedback_joints_within(&self, joints: &[usize], _: Duration) -> bool {
            joints.iter().all(|&j| j < self.dof)
        }
        fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
            out.clear(); for _ in 0..self.dof { out.push(Some(0)); }
        }
        fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) {
            let now = Instant::now();
            out.clear(); for _ in 0..self.dof { out.push(Some(now)); }
        }
        fn position_limits(&self, joint: usize) -> Option<(f32, f32)> {
            self.pos_limits.lock().unwrap().get(joint).copied().flatten()
        }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            Ok(vec![JointState::default(); self.dof])
        }
        fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
            out.clear(); for _ in 0..self.dof { out.push(JointState::default()); } Ok(())
        }
        fn read_joint_states_budgeted(&mut self, out: &mut Vec<JointState>, _: Instant, _: u16) -> Result<(u16, bool), ArmDriverError> {
            self.read_joint_states_into(out)?; Ok((self.dof as u16, false))
        }
        fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
        fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
        fn supports_rt_executor(&self) -> Result<(), &'static str> { Ok(()) }
        fn clear_fault_disables_arm_joint(&self, _: usize) -> bool { false }
        fn clock_now(&self) -> Instant { self.clock.now() }
    }

    let mut cfg = ArmConfig::default();
    cfg.dof = 1;
    cfg.joint_names = vec!["j1".into()];
    cfg.limits = vec![armd::model::JointLimit::new("j1", -2.0, 2.0, 2.0, 5.0)];
    cfg.home_positions = vec![0.0];
    cfg.resting_pose = None;
    let lim_ctl = Arc::new(Mutex::new(vec![Some((0.5, 1.5))])); // conflicts with home_positions [0.0]
    let sup_lim = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(DynamicRtLimitsDriver {
        dof: 1,
        pos_limits: Arc::clone(&lim_ctl),
        clock: MockClock::new(),
    })));
    {
        let mut g = sup_lim.inner().write().await;
        g.lease_enabled = false;
        g.feedback_numeric_valid = true;
        assert!(g.driver_config_error.is_some());
        assert!(g.safety.has_latched_fault_kind(FaultKind::JointLimit));
    }
    let handle_lim = sup_lim.start_executor(exec_cfg).await.unwrap();
    wait_until_async(Duration::from_secs(1), || handle_lim.stats().steps.load(Ordering::Relaxed) >= 6).await;

    // Refused retry while limits still conflict -> 0 alloc on the EXACT tagged release!
    let err = sup_lim.clear_fault().await.unwrap_err();
    assert!(err.to_string().contains("Driver limit configuration error"), "{err}");
    let retry_refuse_seq = handle_lim.stats().last_clear_release_seq.load(Ordering::Acquire);
    let retry_refuse_sample = expect_release_sample(&sup_lim, retry_refuse_seq, "executor clear retry refusal").await;
    assert_ne!(retry_refuse_sample.release_flags & RELEASE_FLAG_CLEAR_REFUSED, 0);
    assert_eq!(retry_refuse_sample.usage.lock_alloc_calls, 0, "executor clear retry refusal allocated: {retry_refuse_sample:?}");

    // Resolve driver limits -> retry applies limits inline, retires error off-thread, clears with 0 alloc!
    *lim_ctl.lock().unwrap() = vec![Some((-1.0, 1.0))];
    assert_eq!(sup_lim.clear_fault().await.unwrap(), SafetyState::Idle);
    let retry_ok_seq = handle_lim.stats().last_clear_release_seq.load(Ordering::Acquire);
    assert!(retry_ok_seq > retry_refuse_seq);
    let retry_ok_sample = expect_release_sample(&sup_lim, retry_ok_seq, "executor clear retry resolved").await;
    assert_ne!(retry_ok_sample.release_flags & RELEASE_FLAG_CLEAR_OK, 0);
    assert_eq!(retry_ok_sample.usage.lock_alloc_calls, 0, "executor clear retry resolved allocated: {retry_ok_sample:?}");
    assert_eq!(handle_lim.stats().wake_alloc_calls.load(Ordering::Relaxed), 0);
    handle_lim.stop_and_join();
}

#[test]
fn arm_driver_error_write_bounded_and_clear_fault_error_cover_all_variants_zero_alloc() {
    use armd::driver::{MitFenceTripReason, TxFailKind};

    // Compile-time exhaustive witness: adding a variant to `ArmDriverError` updates this list.
    fn variant_tag(e: &ArmDriverError) -> &'static str {
        match e {
            ArmDriverError::Transport(_) => "Transport",
            ArmDriverError::Protocol(_) => "Protocol",
            ArmDriverError::JointCountMismatch { .. } => "JointCountMismatch",
            ArmDriverError::HardwareFault(_) => "HardwareFault",
            ArmDriverError::Timeout(_) => "Timeout",
            ArmDriverError::TxDeferred { .. } => "TxDeferred",
            ArmDriverError::EstopInhibited => "EstopInhibited",
            ArmDriverError::MitFenceTripped(_) => "MitFenceTripped",
            ArmDriverError::TorqueTransitionPending => "TorqueTransitionPending",
            ArmDriverError::PendingTxDisabledUnderMitFence => "PendingTxDisabledUnderMitFence",
            ArmDriverError::PosForceModeUnconfirmed { .. } => "PosForceModeUnconfirmed",
            ArmDriverError::PosForceFeedbackStale { .. } => "PosForceFeedbackStale",
            ArmDriverError::PosForceFeedbackStaleBeforeTransmit { .. } => "PosForceFeedbackStaleBeforeTransmit",
            ArmDriverError::PosForceTargetUnavailable { .. } => "PosForceTargetUnavailable",
            ArmDriverError::PosForceTargetNotFinite { .. } => "PosForceTargetNotFinite",
            ArmDriverError::PosForceSpeedZero { .. } => "PosForceSpeedZero",
            ArmDriverError::PosForceReadbackTxFailed { .. } => "PosForceReadbackTxFailed",
            ArmDriverError::PosForcePreEnableHoldBudgetExhausted { .. } => "PosForcePreEnableHoldBudgetExhausted",
            ArmDriverError::PosForcePreEnableHoldTxFailed { .. } => "PosForcePreEnableHoldTxFailed",
            ArmDriverError::TxQueueFull { .. } => "TxQueueFull",
            ArmDriverError::LeaseFenced => "LeaseFenced",
        }
    }

    let mut samples: Vec<ArmDriverError> = vec![
        ArmDriverError::Transport(TransportError::Timeout),
        ArmDriverError::Transport(TransportError::WouldBlock),
        ArmDriverError::Transport(TransportError::NoBufferSpace),
        ArmDriverError::Transport(TransportError::NotOpen),
        ArmDriverError::Transport(TransportError::ChecksumMismatch { expected: 0xAA, actual: 0x55 }),
        ArmDriverError::Transport(TransportError::FrameTooShort { expected: 8, actual: 3 }),
        ArmDriverError::Transport(TransportError::InvalidCanFrame { reason: "dlc > 8", id: 0x101 }),
        ArmDriverError::Transport(TransportError::CanBusErrorFrame { id: 0x2000_0004 }),
        ArmDriverError::Transport(TransportError::SocketCanOs {
            op: "send",
            errno: None,
            kind: std::io::ErrorKind::BrokenPipe,
        }),
        ArmDriverError::Protocol("bad register reply".into()),
        ArmDriverError::JointCountMismatch { expected: 7, actual: 6 },
        ArmDriverError::HardwareFault("motor 2 overvoltage".into()),
        ArmDriverError::Timeout(4),
        ArmDriverError::TxDeferred { pending: 5 },
        ArmDriverError::EstopInhibited,
        ArmDriverError::MitFenceTripped(MitFenceTripReason::FeedbackStale {
            joint: 2,
            age: Some(Duration::from_millis(50)),
            deadline: Duration::from_millis(45),
        }),
        ArmDriverError::MitFenceTripped(MitFenceTripReason::TransmitFailed {
            joint: 3,
            completed_joints: 2,
            total_joints: 7,
            kind: TxFailKind::Enobufs,
        }),
        ArmDriverError::MitFenceTripped(MitFenceTripReason::TxBudgetExceededPreflight {
            needed: 7,
            remaining: 4,
            max: 16,
        }),
        ArmDriverError::TorqueTransitionPending,
        ArmDriverError::PendingTxDisabledUnderMitFence,
        ArmDriverError::PosForceModeUnconfirmed { joint: 6 },
        ArmDriverError::PosForceFeedbackStale { joint: 6 },
        ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint: 6 },
        ArmDriverError::PosForceTargetUnavailable { joint: 6 },
        ArmDriverError::PosForceTargetNotFinite { joint: 6 },
        ArmDriverError::PosForceSpeedZero { joint: 6 },
        ArmDriverError::PosForceReadbackTxFailed { joint: 6, kind: TxFailKind::WouldBlock },
        ArmDriverError::PosForcePreEnableHoldBudgetExhausted { joint: 6 },
        ArmDriverError::PosForcePreEnableHoldTxFailed { joint: 6, kind: TxFailKind::Enobufs },
        ArmDriverError::TxQueueFull { queued: 30, new: 7, capacity: 32 },
        ArmDriverError::LeaseFenced,
    ];
    #[cfg(unix)]
    {
        samples.push(ArmDriverError::Transport(TransportError::SocketCanOs {
            op: "send",
            errno: Some(libc::ENETDOWN),
            kind: std::io::ErrorKind::Other,
        }));
        samples.push(ArmDriverError::Transport(TransportError::Io(
            std::io::Error::from_raw_os_error(libc::ENETDOWN),
        )));
    }

    alloc::time_this_thread();
    for err in &samples {
        let tag = variant_tag(err);
        let _ = alloc::take();
        let mut buf = BoundedText::<256>::empty();
        err.write_bounded(&mut buf);
        let wb_allocs = alloc::take();
        assert_eq!(wb_allocs.calls, 0, "write_bounded allocated for {tag}: {wb_allocs:?}");

        let _ = alloc::take();
        let cf = err.to_clear_fault_error();
        let cf_allocs = alloc::take();
        assert_eq!(cf_allocs.calls, 0, "to_clear_fault_error allocated for {tag}: {cf_allocs:?}");

        let expected = err.to_string();
        assert_eq!(buf.to_string(), expected, "write_bounded text mismatch for {tag}");
        assert_eq!(cf.to_string(), expected, "ClearFaultDriverError text mismatch for {tag}");
    }

    #[cfg(unix)]
    {
        let os_err = ArmDriverError::Transport(TransportError::SocketCanOs {
            op: "send",
            errno: Some(libc::ENETDOWN),
            kind: std::io::ErrorKind::Other,
        });
        assert_eq!(
            os_err.to_string(),
            format!("Transport error: SocketCAN send failed: {}", std::io::Error::from_raw_os_error(libc::ENETDOWN))
        );
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn every_latch_advances_epoch_and_stale_clear_refuses_newer_estop_with_epoch_mismatch() {
    use futures_util::{SinkExt, StreamExt};
    use tokio_tungstenite::tungstenite::Message;

    // 1. Unit invariant: EVERY latch (new kind, E-stop over existing fault, or repeat of same kind
    // including repeat E-stop) advances epoch_id with 0 heap allocations.
    let mut sm = SafetyManager::new();
    alloc::time_this_thread();
    let e0 = sm.epoch_id();
    let _ = alloc::take();
    let e1 = sm.latch(FaultKind::Estop, FaultDetail::Estop);
    let e2 = sm.latch(FaultKind::Estop, FaultDetail::Estop);
    let e3 = sm.latch(FaultKind::BusLoss, FaultDetail::BusLoss);
    let e4 = sm.latch(FaultKind::Estop, FaultDetail::Estop);
    let e5 = sm.latch(FaultKind::BusLoss, FaultDetail::BusLoss);
    let latch_allocs = alloc::take();
    assert_eq!((e1, e2, e3, e4, e5), (e0 + 1, e0 + 2, e0 + 3, e0 + 4, e0 + 5));
    assert_eq!(latch_allocs.calls, 0, "repeated/stacked latches must do 0 heap ops: {latch_allocs:?}");
    assert_eq!(
        sm.clear_check(&ClearContext { expected_epoch: Some(e4), ..ClearContext::all_conditions_met() }),
        Err(ClearRefusal::EpochMismatch { expected: e5, received: e4 })
    );
    assert!(sm.has_latched_fault_kind(FaultKind::Estop));
    assert!(sm.has_latched_fault_kind(FaultKind::BusLoss));

    // 2. Live WebSocket operator session: E-stop A -> read epoch e -> E-stop B -> clear with e ->
    // refused with EPOCH_MISMATCH and latch kept (both applied and pending E-stop B, and E-stop over fault).
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let router = armd::api::create_router(sup.clone(), OperatorAuth::enforced("123456", 1800));
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });

    let http = reqwest::Client::new();
    let unlock: serde_json::Value = http
        .post(format!("http://{addr}/api/v1/auth/unlock"))
        .json(&serde_json::json!({"pin": "123456"}))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let op_token = unlock["token"].as_str().unwrap().to_string();

    let (mut ws, _) = tokio_tungstenite::connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
    ws.send(Message::Text(serde_json::json!({"type": "auth", "token": op_token}).to_string().into())).await.unwrap();
    loop {
        let txt = ws.next().await.unwrap().unwrap().into_text().unwrap();
        let v: serde_json::Value = serde_json::from_str(&txt).unwrap();
        if v["type"] == "auth_ack" {
            assert_eq!(v["role"], "operator", "{v}");
            break;
        }
    }
    ws.send(Message::Text(serde_json::json!({"type":"acquire_lease","seq":1,"client_id":"op"}).to_string().into())).await.unwrap();
    let lease_gen = loop {
        let txt = ws.next().await.unwrap().unwrap().into_text().unwrap();
        let v: serde_json::Value = serde_json::from_str(&txt).unwrap();
        if v["type"] == "lease_granted" { break v["generation"].as_u64().unwrap(); }
    };

    async fn send_clear(
        ws: &mut tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>>,
        req_id: u64,
        lease_gen: u64,
        epoch_id: u64,
    ) -> serde_json::Value {
        let cmd = serde_json::json!({
            "type": "command",
            "request_id": req_id,
            "path": "/api/v1/safety/clear_fault",
            "body": {},
            "generation": lease_gen,
            "epoch_id": epoch_id,
            "client_id": "op"
        });
        ws.send(Message::Text(cmd.to_string().into())).await.unwrap();
        loop {
            let txt = ws.next().await.unwrap().unwrap().into_text().unwrap();
            let v: serde_json::Value = serde_json::from_str(&txt).unwrap();
            if v["type"] == "command_reject" || v["type"] == "command_ack" {
                return v;
            }
        }
    }

    // (a) E-stop A -> client reads epoch e -> second E-stop B -> clear with e -> EPOCH_MISMATCH, latch kept.
    let (st_a, ep_a) = sup.trigger_e_stop().await;
    assert_eq!(st_a, SafetyState::EStop);
    assert_eq!(sup.get_observation().await.epoch_id, ep_a);

    let (st_b, ep_b) = sup.trigger_e_stop().await;
    assert_eq!(st_b, SafetyState::EStop);
    assert_eq!(ep_b, ep_a + 1, "second E-stop B must advance epoch");

    let rej_stale = send_clear(&mut ws, 10, lease_gen, ep_a).await;
    assert_eq!(rej_stale["type"], "command_reject", "{rej_stale}");
    assert_eq!(rej_stale["code"], "EPOCH_MISMATCH", "{rej_stale}");
    assert_eq!(rej_stale["epoch_id"], ep_b, "{rej_stale}");
    assert_eq!(sup.get_observation().await.safety_state, SafetyState::EStop);
    assert!(sup.e_stop_latched.load(Ordering::SeqCst), "stale clear must keep E-stop latch");

    // (b) Pending (unconsumed) E-stop C arrives before operator WS clear carrying ep_b ->
    // WS consumes pending E-stop before reading epoch -> refused with EPOCH_MISMATCH, latch kept!
    {
        let inner = sup.inner().read().await;
        inner.requested_estop_count.fetch_add(1, Ordering::SeqCst);
    }
    let rej_pending = send_clear(&mut ws, 11, lease_gen, ep_b).await;
    assert_eq!(rej_pending["type"], "command_reject", "{rej_pending}");
    assert_eq!(rej_pending["code"], "EPOCH_MISMATCH", "{rej_pending}");
    let ep_c = ep_b + 1;
    assert_eq!(rej_pending["epoch_id"], ep_c, "{rej_pending}");
    assert_eq!(sup.get_observation().await.safety_state, SafetyState::EStop);
    assert!(sup.e_stop_latched.load(Ordering::SeqCst), "pending E-stop C must stay latched");

    // Clear with exact latest epoch ep_c succeeds.
    let ok_c = send_clear(&mut ws, 12, lease_gen, ep_c).await;
    assert_eq!(ok_c["type"], "command_ack", "{ok_c}");
    assert_eq!(sup.get_observation().await.safety_state, SafetyState::Idle);
    assert!(!sup.e_stop_latched.load(Ordering::SeqCst));

    // (c) E-stop over existing fault (BusLoss) advances epoch -> clear with pre-E-stop fault epoch is refused!
    {
        let mut inner = sup.inner().write().await;
        inner.safety.latch(FaultKind::BusLoss, FaultDetail::BusLoss);
    }
    let ep_fault = sup.get_observation().await.epoch_id;
    let (st_es_over_fault, ep_es_over_fault) = sup.trigger_e_stop().await;
    assert_eq!(st_es_over_fault, SafetyState::Fault(FaultReason::BusLoss));
    assert_eq!(ep_es_over_fault, ep_fault + 1, "E-stop over existing fault must advance epoch");

    let rej_over_fault = send_clear(&mut ws, 13, lease_gen, ep_fault).await;
    assert_eq!(rej_over_fault["type"], "command_reject", "{rej_over_fault}");
    assert_eq!(rej_over_fault["code"], "EPOCH_MISMATCH", "{rej_over_fault}");
    assert_eq!(rej_over_fault["epoch_id"], ep_es_over_fault, "{rej_over_fault}");
    assert_eq!(sup.get_observation().await.safety_state, SafetyState::Fault(FaultReason::BusLoss));
    assert!(sup.e_stop_latched.load(Ordering::SeqCst));
}

#[test]
fn clear_check_runs_before_motor_reset_on_legacy_and_executor_paths() {
    use armd::protocol::damiao;

    for use_executor in [false, true] {
        let clock = MockClock::new();
        let limits = dm_limits();
        let bus = FakeBus::with_clock(&limits, clock.clone());
        bus.with(|s| {
            s.immediate_replies = true;
            s.push_all_feedback();
        });
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits.clone())
            .unwrap();
        drv.read_joint_states().unwrap();
        drv.check_mapping_ranges();
        let mut cfg = ArmConfig::rebot_dm();
        cfg.max_temperature = 75.0;
        cfg.resting_pose = None;
        let mut inner = SupervisorInner::new(cfg.clone()).with_driver(Box::new(drv));
        inner.lease_enabled = false;
        inner.safety.set_off_thread_reclaim(use_executor);
        inner.tick(0.02);

        // Latch ThermalTrip (hot 85 C) + JointLimit + motor 0 fault bit (0x0A) with torque off
        // (so `disable_pending` is false and per-kind `clear_check` conditions gate the motor reset).
        assert!(!inner.disable_pending);
        bus.with(|s| {
            s.motors[0].state_override = Some(0x0A);
            s.motors[0].temperature_c = 85;
            s.motors[0].q = cfg.limits[0].max_position + 0.3;
            s.push_all_feedback();
        });
        let _ = inner.poll_hardware(&ServiceBudget::for_release(clock.now(), clock.now() + Duration::from_millis(20)));
        inner.safety.latch(
            FaultKind::ThermalTrip,
            FaultDetail::ThermalTrip { joint_idx: 0, temperature: 85.0, limit: 75.0 },
        );
        inner.safety.latch(
            FaultKind::JointLimit,
            FaultDetail::formatted(FaultCode::JointLimit, |b| b.push_str("joint 1 out of bounds")),
        );

        alloc::time_this_thread();
        let count_resets = || {
            bus.tx_log()
                .iter()
                .filter(|f| f.data == damiao::CMD_CLEAR_FAULTS)
                .count()
        };

        // 1. Hot ThermalTrip MUST refuse BEFORE any motor reset command is transmitted!
        bus.clear_log();
        if use_executor {
            let _ = alloc::take();
            let err = inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap_err();
            assert_eq!(alloc::take().calls, 0, "executor thermal refusal before reset allocated");
            assert_eq!(err, ClearFaultError::Rejected(ClearRefusal::TemperatureConfirmationRequired));
        } else {
            let err = inner.clear_fault().unwrap_err();
            assert!(err.to_string().contains("Temperature must be confirmed below the reset threshold"), "{err}");
        }
        assert_eq!(count_resets(), 0, "use_executor={use_executor}: hot ThermalTrip must block motor reset");

        // 2. Cool motor 0, keep joint 0 out of limits -> JointLimit MUST refuse BEFORE any motor reset!
        bus.with(|s| {
            s.motors[0].temperature_c = 35;
            s.push_all_feedback();
        });
        let _ = inner.poll_hardware(&ServiceBudget::for_release(clock.now(), clock.now() + Duration::from_millis(20)));
        bus.clear_log();
        if use_executor {
            let _ = alloc::take();
            let err = inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap_err();
            assert_eq!(alloc::take().calls, 0, "executor joint-limit refusal before reset allocated");
            assert!(matches!(
                err,
                ClearFaultError::Rejected(ClearRefusal::JointLimitsNotSatisfied(_))
            ), "expected JointLimitsNotSatisfied, got {err:?}");
        } else {
            let err = inner.clear_fault().unwrap_err();
            assert!(err.to_string().contains("within limits"), "{err}");
        }
        assert_eq!(count_resets(), 0, "use_executor={use_executor}: JointLimit must block motor reset");

        // 3. Bring joint 0 back within limits, but inject a pending E-stop -> EStopActive MUST refuse BEFORE reset!
        bus.with(|s| {
            s.motors[0].q = cfg.home_positions[0];
            s.push_all_feedback();
        });
        let _ = inner.poll_hardware(&ServiceBudget::for_release(clock.now(), clock.now() + Duration::from_millis(20)));
        inner.requested_estop_count.fetch_add(1, Ordering::SeqCst);
        bus.clear_log();
        if use_executor {
            let _ = alloc::take();
            let err = inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap_err();
            assert_eq!(alloc::take().calls, 0, "executor pending-estop refusal before reset allocated");
            assert_eq!(err, ClearFaultError::EStopActive);
        } else {
            let err = inner.clear_fault().unwrap_err();
            assert_eq!(err, SafetyError::EStopActive);
        }
        assert_eq!(count_resets(), 0, "use_executor={use_executor}: pending E-stop must block motor reset");

        // 4. Now every OTHER recorded kind passes -> motor reset IS permitted once, and clear_check
        // re-evaluates after reset returning HardwareResetSent.
        bus.clear_log();
        if use_executor {
            let _ = alloc::take();
            let err = inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap_err();
            assert_eq!(alloc::take().calls, 0, "executor HardwareResetSent allocated");
            assert_eq!(err, ClearFaultError::Rejected(ClearRefusal::HardwareResetSent));
        } else {
            let err = inner.clear_fault().unwrap_err();
            assert!(err.to_string().contains("Hardware reset sent"), "{err}");
        }
        assert!(count_resets() >= 1, "use_executor={use_executor}: permitted reset must transmit CMD_CLEAR_FAULTS");

        // 5. Motor clears fault bit & sends fresh fault-free feedback -> subsequent clear passes full clear_check!
        bus.with(|s| {
            s.motors[0].state_override = None;
            s.push_all_feedback();
        });
        let _ = inner.poll_hardware(&ServiceBudget::for_release(clock.now(), clock.now() + Duration::from_millis(20)));
        // Two-stage clear because step 3 consumed pending E-stop into `e_stop_latched`.
        if use_executor {
            let s1 = inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap();
            assert_eq!(s1, SafetyState::EStop);
            inner.safety.drain_retired_state_arcs_off_thread();
            let s2 = inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap();
            assert_eq!(s2, SafetyState::Idle);
        } else {
            let s1 = inner.clear_fault().unwrap();
            assert_eq!(s1, SafetyState::EStop);
            let s2 = inner.clear_fault().unwrap();
            assert_eq!(s2, SafetyState::Idle);
        }
    }
}

#[test]
fn mit_expiry_recovery_final_check_uses_post_recovery_feedback() {
    use armd::mit::{self, MitJointQualification, MitQualificationFile};
    use armd::protocol::damiao;
    use armd::tracking::{JointTrackingConfig, TrackingStrategy};

    const DT: f64 = 0.02;
    let setup_expired_mit = || {
        let clock = MockClock::new();
        let limits = dm_limits();
        let bus = FakeBus::with_clock(&limits, clock.clone());
        bus.with(|s| {
            s.motors[3].set_reg_u32(damiao::RID_TIMEOUT, 2000);
            s.immediate_replies = true;
            s.push_all_feedback();
        });
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits.clone())
            .unwrap();
        drv.read_joint_states().unwrap();
        drv.check_mapping_ranges();

        let mut jt = JointTrackingConfig::rebot_dm();
        jt.enabled = true;
        for j in &mut jt.joints { j.strategy = TrackingStrategy::Position; }
        let j4 = &mut jt.joints[3];
        j4.strategy = TrackingStrategy::Mit;
        j4.kp = 10.0;
        j4.kd = 0.3;
        j4.hold_kp = Some(12.0);
        j4.hold_kd = Some(0.4);
        j4.effort_limit_nm = 3.0;
        j4.gravity_scale = 1.0;
        let mut cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            workspace: None,
            joint_tracking: Some(jt),
            max_temperature: 75.0,
            ..ArmConfig::rebot_dm()
        };
        cfg.max_temperature = 75.0;
        let mut inner = SupervisorInner::new(cfg.clone()).with_driver(Box::new(drv));
        let l4 = limits[3];
        let rec = MitQualificationFile {
            version: mit::QUALIFICATION_VERSION,
            joints: vec![MitJointQualification {
                profile_id: "rebot_b601dm".into(),
                joint: 4,
                joint_name: ArmConfig::rebot_dm().joint_names[3].clone(),
                zero_fingerprint: mit::zero_fingerprint(0.0),
                motor_id: 4,
                reply_id: 0x14,
                esc_id: 4,
                mst_id: 0x14,
                sw_ver: mit_support::SW_VER,
                pmax: l4.p_max,
                vmax: l4.v_max,
                tmax: l4.t_max,
                timeout_raw: 2000,
                timeout_unit_us: damiao::TIMEOUT_UNIT_US_DOCUMENTED,
                timeout_unit_verified: true,
                tested_response: "disabled".into(),
                measured_expiry_ms: 101.0,
                worst_latency_ms: 8.0,
                load_envelope: "unloaded, supported".into(),
                max_kp: 15.0,
                max_kd: 0.5,
                effort_ceiling_nm: 4.0,
                date: "2026-11-22T00:00:00Z".into(),
                trace_id: "b3-fix4-test".into(),
            }],
            invalidated: None,
        };
        inner.set_calibration(Some(armd::calibration::CalibrationFile {
            mit_qualification: serde_json::to_value(&rec).ok(),
            ..Default::default()
        }));
        inner.lease_enabled = false;
        let stamp = inner.current_admission_stamp();
        let adm = inner.check_admission(stamp).unwrap();
        armd::SupervisorInner::enable_torque(adm).unwrap();
        for _ in 0..3 { inner.tick(DT); }
        assert!(inner.mit_active());
        // Inject partial TX failure -> MIT enters MitOwner::Lost (BusLoss) and J4 command watchdog expires (0x0D).
        bus.with(|s| s.fail_tx_after = Some(3));
        inner.tick(DT);
        assert_eq!(inner.mit.owner, mit::MitOwner::Lost);
        bus.with(|s| s.fail_tx_after = None);
        for _ in 0..8 {
            clock.advance(Duration::from_millis(50));
            inner.tick(DT);
        }
        assert!(bus.with(|s| s.motors[3].state_override == Some(0xD) && !s.motors[3].enabled));
        (inner, bus, cfg)
    };

    // (a) Motor 0 is cool (35 C) BEFORE recovery, then jumps to 85 C DURING `mit_expiry_recover()` ->
    // final check MUST inspect post-recovery temperatures and refuse to clear!
    {
        let (mut inner, bus, _cfg) = setup_expired_mit();
        inner.safety.latch(
            FaultKind::ThermalTrip,
            FaultDetail::ThermalTrip { joint_idx: 0, temperature: 80.0, limit: 75.0 },
        );
        // Pre-recovery temperature is cool (35 C). During disable, motor 0 spikes to 85 C!
        bus.with(|s| {
            s.motors[0].temperature_c = 35;
            s.on_disable_state = Some(Arc::new(|st| {
                st.motors[0].temperature_c = 85;
            }));
        });
        let err = inner.clear_fault_confirmed(true).unwrap_err();
        assert!(
            err.to_string().contains("Temperature must be confirmed below the reset threshold"),
            "post-recovery hot temperature must refuse final MIT clear: {err}"
        );
        assert!(inner.safety.state().is_stopped(), "fault must stay latched after hot post-recovery feedback");
        assert!(inner.safety.has_latched_fault_kind(FaultKind::ThermalTrip));
    }

    // (b) Joint 0 is at home BEFORE recovery, then moves beyond soft limit DURING `mit_expiry_recover()` ->
    // final check MUST inspect post-recovery joint positions and refuse to clear!
    {
        let (mut inner, bus, cfg) = setup_expired_mit();
        inner.safety.latch(
            FaultKind::JointLimit,
            FaultDetail::formatted(FaultCode::JointLimit, |b| b.push_str("joint 1 out of limits")),
        );
        let over_limit = cfg.limits[0].max_position + 0.3;
        bus.with(|s| {
            s.motors[0].q = cfg.home_positions[0];
            s.on_disable_state = Some(Arc::new(move |st| {
                st.motors[0].q = over_limit;
            }));
        });
        let err = inner.clear_fault_confirmed(true).unwrap_err();
        assert!(
            err.to_string().contains("within limits"),
            "post-recovery out-of-limit joint position must refuse final MIT clear: {err}"
        );
        assert!(inner.safety.state().is_stopped(), "fault must stay latched after out-of-limit post-recovery feedback");
        assert!(inner.safety.has_latched_fault_kind(FaultKind::JointLimit));
    }

    // (c) R2 Blocker 3: MIT expiry recovery with `J7FeedbackStale` recorded: if J7 goes stale
    //     during `mit_expiry_recover()`, final `clear_with_check` MUST refuse with J7FeedbackStale
    //     WITHOUT committing sampled post-recovery feedback positions or thermal cooldown!
    {
        let clock = MockClock::new();
        let limits = dm_limits();
        let bus = FakeBus::with_clock(&limits, clock.clone());
        bus.with(|s| {
            s.motors[3].set_reg_u32(damiao::RID_TIMEOUT, 2000);
            s.motors[6].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_FORCE);
            s.immediate_replies = true;
            s.push_all_feedback();
        });
        let mut jt = JointTrackingConfig::rebot_dm();
        jt.enabled = true;
        for j in &mut jt.joints { j.strategy = TrackingStrategy::Position; }
        let j4 = &mut jt.joints[3];
        j4.strategy = TrackingStrategy::Mit;
        j4.kp = 10.0;
        j4.kd = 0.3;
        j4.hold_kp = Some(12.0);
        j4.hold_kd = Some(0.4);
        j4.effort_limit_nm = 3.0;
        j4.gravity_scale = 1.0;
        let mut cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            workspace: None,
            joint_tracking: Some(jt),
            max_temperature: 75.0,
            ..ArmConfig::rebot_dm()
        };
        cfg.max_temperature = 75.0;
        let grip = armd::driver::gripper_drive_from_profile(&cfg).expect("pos-force drive");
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits.clone())
            .unwrap()
            .with_gripper_drive(grip)
            .unwrap();
        drv.read_joint_states().unwrap();
        drv.read_joint_states().unwrap();
        drv.check_mapping_ranges();
        let mut inner = SupervisorInner::new(cfg.clone()).with_driver(Box::new(drv));
        let l4 = limits[3];
        let rec = MitQualificationFile {
            version: mit::QUALIFICATION_VERSION,
            joints: vec![MitJointQualification {
                profile_id: "rebot_b601dm".into(),
                joint: 4,
                joint_name: ArmConfig::rebot_dm().joint_names[3].clone(),
                zero_fingerprint: mit::zero_fingerprint(0.0),
                motor_id: 4,
                reply_id: 0x14,
                esc_id: 4,
                mst_id: 0x14,
                sw_ver: mit_support::SW_VER,
                pmax: l4.p_max,
                vmax: l4.v_max,
                tmax: l4.t_max,
                timeout_raw: 2000,
                timeout_unit_us: damiao::TIMEOUT_UNIT_US_DOCUMENTED,
                timeout_unit_verified: true,
                tested_response: "disabled".into(),
                measured_expiry_ms: 101.0,
                worst_latency_ms: 8.0,
                load_envelope: "unloaded, supported".into(),
                max_kp: 15.0,
                max_kd: 0.5,
                effort_ceiling_nm: 4.0,
                date: "2026-11-22T00:00:00Z".into(),
                trace_id: "r2-blocker3-test".into(),
            }],
            invalidated: None,
        };
        inner.set_calibration(Some(armd::calibration::CalibrationFile {
            mit_qualification: serde_json::to_value(&rec).ok(),
            ..Default::default()
        }));
        inner.lease_enabled = false;
        {
            let stamp = inner.current_admission_stamp();
            let adm = inner.check_admission(stamp).unwrap();
            armd::SupervisorInner::enable_torque(adm).unwrap();
        }
        for _ in 0..3 { inner.tick(DT); }
        assert!(inner.mit_active());
        bus.with(|s| s.fail_tx_after = Some(3));
        inner.tick(DT);
        assert_eq!(inner.mit.owner, mit::MitOwner::Lost);
        bus.with(|s| s.fail_tx_after = None);
        for _ in 0..8 {
            clock.advance(Duration::from_millis(50));
            inner.tick(DT);
        }
        assert!(bus.with(|s| s.motors[3].state_override == Some(0xD) && !s.motors[3].enabled));
        inner.safety.latch(FaultKind::J7FeedbackStale, FaultDetail::J7FeedbackStale(None));

        // During `mit_expiry_recover()`, move motor 0 to a distinct position AND mute J7 after
        // advancing > 100 ms so post-recovery J7 freshness fails!
        let prev_measured_j0 = inner.measured_positions[0];
        let clock_cb = clock.clone();
        bus.with(|s| {
            s.motors[0].q = prev_measured_j0 + 0.25;
            s.on_disable_state = Some(Arc::new(move |st| {
                st.motors[0].q = prev_measured_j0 + 0.35;
                st.motors[6].mute_feedback = true;
                clock_cb.advance(Duration::from_millis(120));
            }));
        });
        let err = inner.clear_fault_confirmed(true).unwrap_err();
        assert!(
            err.to_string().contains("J7") || err.to_string().contains("feedback"),
            "post-recovery stale J7 must refuse MIT recovery clear: {err}"
        );
        assert!(inner.safety.state().is_stopped(), "fault must remain latched");
        assert_eq!(
            inner.measured_positions[0], prev_measured_j0,
            "refused MIT recovery clear must NOT commit sampled feedback positions"
        );
    }
}

#[test]
fn clear_evidence_stale_temp_resets_cooldown_expired_lease_clears_and_joint_limit_names_joint() {
    // 1. Thermal cooldown accrues ONLY while temperature feedback is fresh; stale temp resets timer!
    let clock = MockClock::new();
    let limits = dm_limits();
    let bus = FakeBus::with_clock(&limits, clock.clone());
    bus.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits.clone())
        .unwrap();
    drv.read_joint_states().unwrap();
    drv.check_mapping_ranges();
    let mut cfg = ArmConfig::rebot_dm();
    cfg.max_temperature = 75.0;
    cfg.resting_pose = None;
    let mut inner = SupervisorInner::new(cfg.clone()).with_driver(Box::new(drv));
    inner.lease_enabled = false;
    inner.thermal_cooldown_duration = Duration::from_millis(80);
    inner.safety.set_off_thread_reclaim(true);

    // Latch ThermalTrip, then deliver fresh cool feedback (40 C) so cooldown timer starts.
    inner.safety.latch(
        FaultKind::ThermalTrip,
        FaultDetail::ThermalTrip { joint_idx: 0, temperature: 85.0, limit: 75.0 },
    );
    bus.with(|s| {
        s.motors[0].temperature_c = 40;
        s.push_all_feedback();
    });
    let _ = inner.poll_hardware(&ServiceBudget::for_release(clock.now(), clock.now() + Duration::from_millis(20)));
    inner.tick(0.02);
    // Advance 50 ms on driver clock with a second fresh cool sample -> 50 ms dwell (< 80 ms required).
    clock.advance(Duration::from_millis(50));
    bus.with(|s| s.push_all_feedback());
    let _ = inner.poll_hardware(&ServiceBudget::for_release(clock.now(), clock.now() + Duration::from_millis(20)));
    let err_mid = inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap_err();
    assert_eq!(
        err_mid,
        ClearFaultError::Rejected(ClearRefusal::ThermalCooldownNotElapsed {
            elapsed_ms: 50,
            required_ms: 80,
        })
    );

    // Now make temperature feedback stale (> 250 ms old) and tick: cooldown timer MUST reset!
    clock.advance(Duration::from_millis(300));
    let _ = inner.poll_hardware(&ServiceBudget::for_release(clock.now(), clock.now() + Duration::from_millis(20)));
    inner.tick(0.02);
    let err_stale = inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap_err();
    assert_eq!(err_stale, ClearFaultError::Rejected(ClearRefusal::FreshFeedbackRequired));

    // Fresh cool feedback resumes: because stale temp reset the timer, cooldown MUST start from 0 ms
    // and refuse immediately (must NOT inherit pre-stale dwell!).
    bus.with(|s| {
        s.motors[0].temperature_c = 40;
        s.push_all_feedback();
    });
    let _ = inner.poll_hardware(&ServiceBudget::for_release(clock.now(), clock.now() + Duration::from_millis(20)));
    inner.tick(0.02);
    alloc::time_this_thread();
    let _ = alloc::take();
    let err_restarted = inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap_err();
    assert_eq!(alloc::take().calls, 0, "restarted cooldown refusal must do 0 heap ops");
    assert_eq!(
        err_restarted,
        ClearFaultError::Rejected(ClearRefusal::ThermalCooldownNotElapsed {
            elapsed_ms: 0,
            required_ms: 80,
        })
    );
    // After a FULL fresh 85 ms dwell (<= 250 ms sample gap) with a fresh cool sample -> clears cleanly!
    clock.advance(Duration::from_millis(85));
    bus.with(|s| s.push_all_feedback());
    let _ = inner.poll_hardware(&ServiceBudget::for_release(clock.now(), clock.now() + Duration::from_millis(20)));
    assert_eq!(
        inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap(),
        SafetyState::Idle
    );

    // 2. Expired lease counts as NO lease for WatchdogTimeout, while active lease with stale heartbeat refuses.
    let mut wd_cfg = ArmConfig::rebot_dm();
    wd_cfg.lease_timeout_ms = 30;
    let mut wd_inner = SupervisorInner::new(wd_cfg);
    let _ = wd_inner.acquire_lease("holder-1").unwrap();
    wd_inner.safety.latch(FaultKind::WatchdogTimeout, FaultDetail::WatchdogTimeout);
    // Lease is still active (`acquired_at` is fresh), but `last_heartbeat` is stale -> refused!
    wd_inner.last_heartbeat = Instant::now() - Duration::from_millis(50);
    assert!(wd_inner.is_lease_active());
    assert_eq!(
        wd_inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)),
        Err(ClearFaultError::Rejected(ClearRefusal::WatchdogHeartbeatRequired))
    );
    // Fresh heartbeat from active holder -> clears!
    let wd_ep = wd_inner.safety.epoch_id();
    wd_inner.heartbeat("holder-1", wd_ep).unwrap();
    assert_eq!(wd_inner.clear_fault().unwrap(), SafetyState::Idle);
    // Re-latch WatchdogTimeout and let lease expire without releasing -> active_lease is Some(..)
    // but expired (`!is_lease_active()`), which MUST count as NO lease and clear!
    wd_inner.safety.latch(FaultKind::WatchdogTimeout, FaultDetail::WatchdogTimeout);
    std::thread::sleep(Duration::from_millis(45));
    assert!(wd_inner.active_lease.is_some() && !wd_inner.is_lease_active());
    assert_eq!(wd_inner.clear_fault().unwrap(), SafetyState::Idle);

    // 3. JointLimit with torque OFF and arm resting beyond soft limit + 0.01 rad:
    // keep refusing (never allow software clear into out-of-limit pose), and name joint, position, and limit!
    let mut jl_inner = SupervisorInner::new(ArmConfig::rebot_dm());
    jl_inner.lease_enabled = false;
    jl_inner.is_torqued = false;
    jl_inner.safety.set_off_thread_reclaim(true);
    let j2_lim = jl_inner.config.limits[2].clone();
    let resting_beyond = j2_lim.min_position - 0.05; // beyond soft min - 0.01 rad
    jl_inner.measured_positions[2] = resting_beyond;
    jl_inner.safety.latch(
        FaultKind::JointLimit,
        FaultDetail::JointLimitExceeded {
            joint_idx: 2,
            joint_name: BoundedText::new(&j2_lim.name),
            value: resting_beyond,
            min: j2_lim.min_position,
            max: j2_lim.max_position,
        },
    );
    // Legacy clear_fault refuses with joint name, position, and limits.
    let legacy_msg = jl_inner.clear_fault().unwrap_err().to_string();
    assert!(
        legacy_msg.contains(&j2_lim.name)
            && legacy_msg.contains("joint 3")
            && legacy_msg.contains(&format!("{resting_beyond:.4}"))
            && legacy_msg.contains(&format!("{:.4}", j2_lim.min_position))
            && legacy_msg.contains(&format!("{:.4}", j2_lim.max_position))
            && legacy_msg.contains("by hand"),
        "JointLimit torque-off refusal must name joint, position, and limits: {legacy_msg}"
    );
    assert!(jl_inner.safety.has_latched_fault_kind(FaultKind::JointLimit));
    // RT executor clear_fault_executor_bounded refuses with 0 heap allocations and renders identical message.
    let _ = alloc::take();
    let rt_err = jl_inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap_err();
    assert_eq!(alloc::take().calls, 0, "JointLimit refusal on RT path must do 0 heap ops");
    assert_eq!(SafetyError::from(rt_err).to_string(), legacy_msg);
    // Operator moves joint 3 back within soft limit + 0.01 rad by hand -> clears!
    jl_inner.measured_positions[2] = j2_lim.min_position - 0.005;
    assert_eq!(jl_inner.clear_fault().unwrap(), SafetyState::Idle);
}

#[test]
fn step5_write_output_error_is_bounded_zero_alloc_on_normal_and_estop_hold() {
    #[derive(Clone, Copy)]
    enum WriteMode {
        Ok,
        SocketCanErr,
        PosForceSpeedZero,
        EstopInhibited,
    }
    struct FailingWriteDriver {
        dof: usize,
        mode: Arc<Mutex<WriteMode>>,
        clock: MockClock,
    }
    impl FailingWriteDriver {
        fn make_err(&self) -> Result<(), ArmDriverError> {
            match *self.mode.lock().unwrap() {
                WriteMode::Ok => Ok(()),
                WriteMode::SocketCanErr => Err(ArmDriverError::Transport(TransportError::SocketCanOs {
                    op: "send",
                    errno: {
                        #[cfg(unix)]
                        { Some(libc::ENETDOWN) }
                        #[cfg(not(unix))]
                        { None }
                    },
                    kind: std::io::ErrorKind::BrokenPipe,
                })),
                WriteMode::PosForceSpeedZero => Err(ArmDriverError::PosForceSpeedZero { joint: 6 }),
                WriteMode::EstopInhibited => Err(ArmDriverError::EstopInhibited),
            }
        }
    }
    impl ArmDriver for FailingWriteDriver {
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
        fn enable_state(&self) -> EnableState { EnableState::Enabled }
        fn is_feedback_valid(&self) -> bool { true }
        fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); self.dof] }
        fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
            out.clear();
            for _ in 0..self.dof { out.push(Some(Duration::ZERO)); }
        }
        fn feedback_all_within(&self, dof: usize, _: Duration) -> bool { self.dof == dof }
        fn feedback_joints_within(&self, joints: &[usize], _: Duration) -> bool {
            joints.iter().all(|&j| j < self.dof)
        }
        fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
            out.clear();
            for _ in 0..self.dof { out.push(Some(0)); }
        }
        fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) {
            let now = Instant::now();
            out.clear();
            for _ in 0..self.dof { out.push(Some(now)); }
        }
        fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) {
            let now = Instant::now();
            out.clear();
            for _ in 0..self.dof { out.push(Some(now)); }
        }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            Ok(vec![JointState { temperature: 30.0, ..JointState::default() }; self.dof])
        }
        fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
            out.clear();
            for _ in 0..self.dof {
                out.push(JointState { temperature: 30.0, ..JointState::default() });
            }
            Ok(())
        }
        fn read_joint_states_budgeted(
            &mut self,
            out: &mut Vec<JointState>,
            _: Instant,
            _: u16,
        ) -> Result<(u16, bool), ArmDriverError> {
            self.read_joint_states_into(out)?;
            Ok((self.dof as u16, false))
        }
        fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> {
            self.make_err()
        }
        fn write_estop_hold(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> {
            self.make_err()
        }
        fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
        fn supports_rt_executor(&self) -> Result<(), &'static str> { Ok(()) }
        fn clock_now(&self) -> Instant { self.clock.now() }
    }

    let mut cfg = ArmConfig::rebot_dm();
    cfg.joint_tracking = None;
    cfg.resting_pose = None;
    let mode = Arc::new(Mutex::new(WriteMode::Ok));
    let mut inner = SupervisorInner::new(cfg.clone()).with_driver(Box::new(FailingWriteDriver {
        dof: cfg.dof,
        mode: Arc::clone(&mode),
        clock: MockClock::new(),
    }));
    inner.lease_enabled = false;
    let stamp = inner.current_admission_stamp();
    let adm = inner.check_admission(stamp).unwrap();
    armd::SupervisorInner::enable_torque(adm).unwrap();
    inner.safety.set_off_thread_reclaim(true);
    // Warm scratch buffers with two clean ticks.
    for _ in 0..2 {
        let _ = inner.tick_control(0.02);
    }
    assert!(inner.safety.state().is_operational());

    alloc::time_this_thread();

    // 1. Operational Step-5 write_joint_targets failure -> 0 heap allocs, latches DriverWriteFailed with bounded text.
    *mode.lock().unwrap() = WriteMode::SocketCanErr;
    let _ = alloc::take();
    let _ = inner.tick_control(0.02);
    let write_err_allocs = alloc::take();
    assert_eq!(
        write_err_allocs.calls, 0,
        "Step-5 write_joint_targets error must do 0 heap allocations: {write_err_allocs:?}"
    );
    let fi = inner.safety.fault().expect("DriverWriteFailed must be latched");
    assert_eq!(fi.code, FaultCode::DriverWriteFailed);
    assert!(
        fi.message.as_str().starts_with("Control output failed: Transport error: SocketCAN send failed:"),
        "unexpected bounded message: {}",
        fi.message
    );

    // 2. Stopped Step-5 write_estop_hold failure -> 0 heap allocs on bounded error path.
    *mode.lock().unwrap() = WriteMode::PosForceSpeedZero;
    let _ = alloc::take();
    let _ = inner.tick_control(0.02);
    let hold_err_allocs = alloc::take();
    assert_eq!(
        hold_err_allocs.calls, 0,
        "Step-5 write_estop_hold error must do 0 heap allocations: {hold_err_allocs:?}"
    );

    // 3. EstopInhibited error during urgent stop -> 0 heap allocs and not treated as DriverWriteFailed.
    inner.safety.drain_retired_state_arcs_off_thread();
    inner.safety.drain_log_events_off_thread();
    *mode.lock().unwrap() = WriteMode::Ok;
    assert_eq!(inner.clear_fault().unwrap(), SafetyState::Idle);
    let stamp = inner.current_admission_stamp();
    let adm = inner.check_admission(stamp).unwrap();
    armd::SupervisorInner::enable_torque(adm).unwrap();
    inner.tick_control(0.02);
    *mode.lock().unwrap() = WriteMode::EstopInhibited;
    let _ = alloc::take();
    let _ = inner.tick_control(0.02);
    let inhib_allocs = alloc::take();
    assert_eq!(
        inhib_allocs.calls, 0,
        "Step-5 EstopInhibited error must do 0 heap allocations: {inhib_allocs:?}"
    );
}

#[test]
fn thermal_cooldown_restarts_across_unobserved_stale_gap_without_tick() {
    let make_rig = |use_executor: bool| {
        let clock = MockClock::new();
        let limits = dm_limits();
        let bus = FakeBus::with_clock(&limits, clock.clone());
        bus.with(|s| {
            s.immediate_replies = true;
            for m in &mut s.motors { m.temperature_c = 40; }
            s.push_all_feedback();
        });
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits)
            .unwrap();
        drv.read_joint_states().unwrap();
        drv.check_mapping_ranges();
        let mut cfg = ArmConfig::rebot_dm();
        cfg.max_temperature = 75.0;
        cfg.resting_pose = None;
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
        inner.lease_enabled = false;
        inner.thermal_cooldown_duration = Duration::from_millis(80);
        inner.safety.set_off_thread_reclaim(use_executor);
        inner.safety.latch(
            FaultKind::ThermalTrip,
            FaultDetail::ThermalTrip { joint_idx: 0, temperature: 85.0, limit: 75.0 },
        );
        (inner, bus, clock)
    };

    // 1. Legacy clear_fault AND RT executor clear_fault_executor_bounded:
    //    An unobserved gap (> 250 ms) with NO intermediate tick/check between two fresh cool samples
    //    MUST restart the cooldown dwell from the second fresh sample (never continuing the old dwell).
    for use_executor in [false, true] {
        let (mut inner, bus, clock) = make_rig(use_executor);
        // Sample 1 at t = 0 ms (fresh, cool).
        bus.with(|s| s.push_all_feedback());
        if use_executor {
            let _ = inner.poll_hardware(&ServiceBudget::for_release(clock.now(), clock.now() + Duration::from_millis(20)));
        } else {
            let _ = inner.clear_fault().unwrap_err();
        }
        // Sample 2 at t = 60 ms (gap 60 ms <= 250 ms -> dwell = 60 ms < 80 ms).
        clock.advance(Duration::from_millis(60));
        bus.with(|s| s.push_all_feedback());
        if use_executor {
            let _ = inner.poll_hardware(&ServiceBudget::for_release(clock.now(), clock.now() + Duration::from_millis(20)));
            let err = inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap_err();
            assert_eq!(err, ClearFaultError::Rejected(ClearRefusal::ThermalCooldownNotElapsed { elapsed_ms: 60, required_ms: 80 }));
        } else {
            let err = inner.clear_fault().unwrap_err();
            assert!(err.to_string().contains("60 ms of 80 ms"), "expected 60 ms dwell before gap: {err}");
        }

        // Unobserved 300 ms stale gap (> 250 ms) with ZERO ticks or intermediate checks!
        // A fresh cool sample then arrives BEFORE the next check (age = 0 ms <= 250 ms).
        clock.advance(Duration::from_millis(300));
        bus.with(|s| s.push_all_feedback());

        if use_executor {
            let _ = inner.poll_hardware(&ServiceBudget::for_release(clock.now(), clock.now() + Duration::from_millis(20)));
            alloc::time_this_thread();
            let _ = alloc::take();
            let err = inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap_err();
            assert_eq!(alloc::take().calls, 0, "restarted cooldown refusal after unobserved gap must do 0 heap ops");
            assert_eq!(
                err,
                ClearFaultError::Rejected(ClearRefusal::ThermalCooldownNotElapsed { elapsed_ms: 0, required_ms: 80 }),
                "unobserved >250ms gap without tick must restart dwell to 0 ms"
            );
        } else {
            // Legacy clear_fault reads the new fresh sample directly with zero ticks in between.
            let err = inner.clear_fault().unwrap_err();
            assert!(
                err.to_string().contains("0 ms of 80 ms"),
                "legacy clear_fault must restart dwell to 0 ms after unobserved gap: {err}"
            );
        }
        assert!(inner.safety.has_latched_fault_kind(FaultKind::ThermalTrip));

        // 2. Single fresh sample does NOT accrue dwell merely by waiting (no new sample arrived):
        //    advance 100 ms (< 250 ms fresh) without pushing any new frame -> dwell stays 0 ms!
        clock.advance(Duration::from_millis(100));
        if use_executor {
            let _ = inner.poll_hardware(&ServiceBudget::for_release(clock.now(), clock.now() + Duration::from_millis(20)));
            let err = inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap_err();
            assert_eq!(
                err,
                ClearFaultError::Rejected(ClearRefusal::ThermalCooldownNotElapsed { elapsed_ms: 0, required_ms: 80 }),
                "elapsed time without a newer fresh sample must not accrue cooldown dwell"
            );
        } else {
            let err = inner.clear_fault().unwrap_err();
            assert!(err.to_string().contains("0 ms of 80 ms"), "no new sample must keep dwell at 0 ms: {err}");
        }

        // 3. Consecutive fresh cool sample arrives 100 ms after restart (gap = 100 ms <= 250 ms -> dwell = 100 ms >= 80 ms) -> clears!
        bus.with(|s| s.push_all_feedback());
        if use_executor {
            let _ = inner.poll_hardware(&ServiceBudget::for_release(clock.now(), clock.now() + Duration::from_millis(20)));
            assert_eq!(
                inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)).unwrap(),
                SafetyState::Idle
            );
        } else {
            assert_eq!(inner.clear_fault().unwrap(), SafetyState::Idle);
        }
    }

    // 4. Per-joint unobserved gap: J1..J6 keep sending fresh samples while J0 has a 280 ms gap (> 250 ms)
    //    and then sends a fresh sample before the next check -> dwell restarts!
    let (mut inner, bus, clock) = make_rig(false);
    bus.with(|s| s.push_all_feedback());
    let _ = inner.clear_fault().unwrap_err(); // t = 0
    clock.advance(Duration::from_millis(60));
    bus.with(|s| s.push_all_feedback());
    let err60 = inner.clear_fault().unwrap_err();
    assert!(err60.to_string().contains("60 ms of 80 ms"), "{err60}");
    // J1..J6 update at t = 220 ms (gap 160 ms), J0 stays at t = 60 ms.
    clock.advance(Duration::from_millis(160));
    bus.with(|s| { for i in 1..7 { s.push_feedback(i); } });
    let err_same = inner.clear_fault().unwrap_err(); // oldest is still J0 at 60 ms (age 160 ms <= 250 ms)
    assert!(err_same.to_string().contains("60 ms of 80 ms"), "{err_same}");
    // Now advance 120 ms (t = 340 ms): J0 gap since 60 ms is 280 ms (> 250 ms!), while J1..J6 gap is 120 ms.
    // All 7 push fresh frames at t = 340 ms with no intermediate check -> J0's 280 ms gap restarts dwell!
    clock.advance(Duration::from_millis(120));
    bus.with(|s| s.push_all_feedback());
    let err_restart = inner.clear_fault().unwrap_err();
    assert!(
        err_restart.to_string().contains("0 ms of 80 ms"),
        "per-joint >250ms gap must restart dwell to 0 ms: {err_restart}"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_clear_fault_carries_epoch_and_refuses_after_newer_estop() {
    use armd::rt_core::executor::ExecutorClock;

    struct StepClock {
        origin: Instant,
        state: Mutex<(u64, u64)>, // (now_ns, sleeping_waiters)
        cv: Condvar,
    }
    impl StepClock {
        fn new() -> Arc<Self> {
            Arc::new(Self {
                origin: Instant::now(),
                state: Mutex::new((0, 0)),
                cv: Condvar::new(),
            })
        }
        fn wait_until_parked(&self) {
            let mut g = self.state.lock().unwrap();
            let deadline = Instant::now() + Duration::from_secs(2);
            while g.1 == 0 {
                let rem = deadline.saturating_duration_since(Instant::now());
                assert!(!rem.is_zero(), "timed out waiting for armd-control to park on StepClock");
                g = self.cv.wait_timeout(g, rem).unwrap().0;
            }
        }
        fn advance(&self, dur: Duration) {
            let mut g = self.state.lock().unwrap();
            g.0 = g.0.saturating_add(dur.as_nanos() as u64);
            self.cv.notify_all();
        }
    }
    impl ExecutorClock for StepClock {
        fn now(&self) -> Instant {
            let ns = self.state.lock().unwrap().0;
            self.origin + Duration::from_nanos(ns)
        }
        fn now_ns(&self) -> u64 {
            self.state.lock().unwrap().0
        }
        fn sleep_until_ns(&self, target_ns: u64, stop: &AtomicBool) -> bool {
            let mut g = self.state.lock().unwrap();
            if g.0 >= target_ns {
                return false;
            }
            g.1 += 1;
            self.cv.notify_all();
            while !stop.load(Ordering::Acquire) && g.0 < target_ns {
                g = self.cv.wait_timeout(g, Duration::from_millis(5)).unwrap().0;
            }
            g.1 -= 1;
            true
        }
        fn sleep(&self, dur: Duration, stop: &AtomicBool) {
            let target = self.now_ns().saturating_add(dur.as_nanos() as u64);
            let _ = self.sleep_until_ns(target, stop);
        }
        fn wake(&self) {
            self.cv.notify_all();
        }
    }

    // 1. Live RtExecutor: queue ClearFault(Some(e)), latch a newer E-stop B before the release runs,
    //    then step armd-control -> refused with EpochMismatch, latch kept, 0 heap allocs.
    let period = Duration::from_millis(20);
    let step_clock = StepClock::new();
    let cfg = ArmConfig::rebot_dm();
    let sup = Arc::new(ArmSupervisor::new(cfg));
    {
        let mut g = sup.inner().write().await;
        g.lease_enabled = false;
    }
    let mut exec_cfg = ExecutorConfig::for_tests(period);
    exec_cfg.alloc_timing = true;
    let hooks = PersistTestHooks {
        clock: Some(step_clock.clone()),
        ..Default::default()
    };
    let handle = sup.start_executor_with_hooks(exec_cfg, hooks).await.unwrap();
    let step_one = |target_steps: u64| {
        step_clock.wait_until_parked();
        step_clock.advance(period);
        let deadline = Instant::now() + Duration::from_secs(2);
        while handle.stats().steps.load(Ordering::Acquire) < target_steps {
            assert!(Instant::now() < deadline, "timed out waiting for step {target_steps}");
            std::thread::sleep(Duration::from_millis(1));
        }
    };
    // Warm up every StateHub triple-buffer slot (including the reader front slot via `latest()`)
    // and per-tick scratch buffer before measuring release allocations.
    step_clock.wait_until_parked();
    let mut expected_steps = handle.stats().steps.load(Ordering::Acquire);
    for _ in 0..8 {
        expected_steps += 1;
        step_one(expected_steps);
        let _ = sup.hub().latest();
    }
    step_clock.wait_until_parked();

    // Latch E-stop A and observe epoch e.
    let estop_a = tokio::spawn({
        let sup = Arc::clone(&sup);
        async move { sup.trigger_e_stop_checked().await }
    });
    tokio::time::sleep(Duration::from_millis(10)).await;
    expected_steps += 1;
    step_one(expected_steps);
    let (state_a, epoch_a) = estop_a.await.unwrap().unwrap();
    assert_eq!(state_a, SafetyState::EStop);
    step_clock.wait_until_parked();

    // While armd-control is parked between releases:
    // (a) Queue ClearFault carrying client-observed epoch_a.
    let clear_stale = tokio::spawn({
        let sup = Arc::clone(&sup);
        async move { sup.clear_fault_for_epoch(Some(epoch_a)).await }
    });
    tokio::time::sleep(Duration::from_millis(10)).await;
    // (b) Latch a newer E-stop B before the queued clear is applied.
    let estop_b = tokio::spawn({
        let sup = Arc::clone(&sup);
        async move { sup.trigger_e_stop_checked().await }
    });
    tokio::time::sleep(Duration::from_millis(10)).await;

    // Step one release: Step 1 applies E-stop B (epoch_a + 1), Step 2b evaluates queued ClearFault(epoch_a).
    expected_steps += 1;
    step_one(expected_steps);
    let (state_b, epoch_b) = estop_b.await.unwrap().unwrap();
    assert_eq!(state_b, SafetyState::EStop);
    assert_eq!(epoch_b, epoch_a + 1);

    let clear_res = clear_stale.await.unwrap();
    assert!(
        matches!(
            clear_res,
            Err(ArmError::Safety(SafetyError::EpochMismatch { expected, received }))
                if expected == epoch_b && received == epoch_a
        ),
        "queued ClearFault with stale epoch_a must be refused after newer E-stop B: {clear_res:?}"
    );
    let obs_after_refuse = sup.hub().latest().expect("published observation");
    assert_eq!(obs_after_refuse.obs.safety_state, SafetyState::EStop, "latch must stay active");
    assert_eq!(obs_after_refuse.obs.epoch_id, epoch_b);

    let refuse_seq = handle.stats().last_clear_release_seq.load(Ordering::Acquire);
    let refuse_sample = expect_release_sample(&sup, refuse_seq, "executor stale-epoch clear refusal").await;
    assert_ne!(refuse_sample.release_flags & RELEASE_FLAG_CLEAR_REFUSED, 0);
    assert_eq!(refuse_sample.usage.lock_alloc_calls, 0, "stale-epoch clear refusal allocated: {refuse_sample:?}");

    // Queuing ClearFault with the exact current epoch_b clears cleanly with 0 allocs.
    step_clock.wait_until_parked();
    let clear_exact = tokio::spawn({
        let sup = Arc::clone(&sup);
        async move { sup.clear_fault_for_epoch(Some(epoch_b)).await }
    });
    tokio::time::sleep(Duration::from_millis(10)).await;
    expected_steps += 1;
    step_one(expected_steps);
    assert_eq!(clear_exact.await.unwrap().unwrap(), SafetyState::Idle);
    let ok_seq = handle.stats().last_clear_release_seq.load(Ordering::Acquire);
    assert!(ok_seq > refuse_seq);
    let ok_sample = expect_release_sample(&sup, ok_seq, "executor exact-epoch clear ok").await;
    assert_ne!(ok_sample.release_flags & RELEASE_FLAG_CLEAR_OK, 0);
    assert_eq!(ok_sample.usage.lock_alloc_calls, 0, "exact-epoch clear allocated: {ok_sample:?}");

    step_clock.advance(period);
    handle.stop_and_join();

    // 2. Owner-level direct check: pending unconsumed E-stop B arriving right before
    //    clear_fault_executor_bounded_for_epoch(Some(e)) -> EpochMismatch (0 heap allocs).
    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
    inner.lease_enabled = false;
    inner.safety.set_off_thread_reclaim(true);
    inner.apply_stop_policy(StopPolicy::EStopLatch);
    let e = inner.safety.epoch_id();
    // Simulate a newer E-stop B queued before clear_fault runs on owner.
    inner.requested_estop_count.fetch_add(1, Ordering::SeqCst);
    alloc::time_this_thread();
    let _ = alloc::take();
    let err = inner
        .clear_fault_executor_bounded_for_epoch(Instant::now() + Duration::from_millis(50), Some(e))
        .unwrap_err();
    assert_eq!(alloc::take().calls, 0, "pending-estop EpochMismatch on executor path allocated");
    assert_eq!(err, ClearFaultError::EpochMismatch { expected: e + 1, received: e });
    assert_eq!(*inner.safety.state(), SafetyState::EStop);
    assert_eq!(inner.safety.epoch_id(), e + 1);
    // Exact epoch clears cleanly with 0 heap allocs.
    let _ = alloc::take();
    let cleared = inner
        .clear_fault_executor_bounded_for_epoch(Instant::now() + Duration::from_millis(50), Some(e + 1))
        .unwrap();
    assert_eq!(alloc::take().calls, 0, "exact-epoch clear_fault_executor_bounded_for_epoch allocated");
    assert_eq!(cleared, SafetyState::Idle);
}

#[test]
fn mit_recovery_no_fence_ack_or_state_change_before_clear_check_passes() {
    use armd::mit::{self, MitJointQualification, MitQualificationFile};
    use armd::protocol::damiao;
    use armd::tracking::{JointTrackingConfig, TrackingStrategy};

    const DT: f64 = 0.02;
    let setup_mit_lost = |advance_past_quiet: bool| {
        let clock = MockClock::new();
        let limits = dm_limits();
        let bus = FakeBus::with_clock(&limits, clock.clone());
        bus.with(|s| {
            s.motors[3].set_reg_u32(damiao::RID_TIMEOUT, 2000);
            s.immediate_replies = true;
            s.push_all_feedback();
        });
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits.clone())
            .unwrap();
        drv.read_joint_states().unwrap();
        drv.check_mapping_ranges();

        let mut jt = JointTrackingConfig::rebot_dm();
        jt.enabled = true;
        for j in &mut jt.joints { j.strategy = TrackingStrategy::Position; }
        let j4 = &mut jt.joints[3];
        j4.strategy = TrackingStrategy::Mit;
        j4.kp = 10.0;
        j4.kd = 0.3;
        j4.hold_kp = Some(12.0);
        j4.hold_kd = Some(0.4);
        j4.effort_limit_nm = 3.0;
        j4.gravity_scale = 1.0;
        let mut cfg = ArmConfig {
            lease_timeout_ms: 50,
            workspace: None,
            joint_tracking: Some(jt),
            max_temperature: 75.0,
            ..ArmConfig::rebot_dm()
        };
        cfg.max_temperature = 75.0;
        let mut inner = SupervisorInner::new(cfg.clone()).with_driver(Box::new(drv));
        let l4 = limits[3];
        let rec = MitQualificationFile {
            version: mit::QUALIFICATION_VERSION,
            joints: vec![MitJointQualification {
                profile_id: "rebot_b601dm".into(),
                joint: 4,
                joint_name: ArmConfig::rebot_dm().joint_names[3].clone(),
                zero_fingerprint: mit::zero_fingerprint(0.0),
                motor_id: 4,
                reply_id: 0x14,
                esc_id: 4,
                mst_id: 0x14,
                sw_ver: mit_support::SW_VER,
                pmax: l4.p_max,
                vmax: l4.v_max,
                tmax: l4.t_max,
                timeout_raw: 2000,
                timeout_unit_us: damiao::TIMEOUT_UNIT_US_DOCUMENTED,
                timeout_unit_verified: true,
                tested_response: "disabled".into(),
                measured_expiry_ms: 101.0,
                worst_latency_ms: 8.0,
                load_envelope: "unloaded, supported".into(),
                max_kp: 15.0,
                max_kd: 0.5,
                effort_ceiling_nm: 4.0,
                date: "2026-11-22T00:00:00Z".into(),
                trace_id: "b3f-item3".into(),
            }],
            invalidated: None,
        };
        inner.set_calibration(Some(armd::calibration::CalibrationFile {
            mit_qualification: serde_json::to_value(&rec).ok(),
            ..Default::default()
        }));
        inner.lease_enabled = false;
        let stamp = inner.current_admission_stamp();
        let adm = inner.check_admission(stamp).unwrap();
        armd::SupervisorInner::enable_torque(adm).unwrap();
        for _ in 0..3 { inner.tick(DT); }
        assert!(inner.mit_active());
        bus.with(|s| s.fail_tx_after = Some(3));
        inner.tick(DT);
        assert_eq!(inner.mit.owner, mit::MitOwner::Lost);
        bus.with(|s| s.fail_tx_after = None);
        if advance_past_quiet {
            for _ in 0..8 {
                clock.advance(Duration::from_millis(50));
                inner.tick(DT);
            }
            assert!(bus.with(|s| s.motors[3].state_override == Some(0xD) && !s.motors[3].enabled));
        }
        (inner, bus, clock)
    };

    // (a) Quiet-period refusal MUST NOT ack the fence: after holdoff elapses, background ticks
    //     must still emit 0 autonomous 0xCC CAN refresh frames.
    {
        let (mut inner, bus, clock) = setup_mit_lost(false);
        bus.clear_log();
        let err = inner.clear_fault_confirmed(true).unwrap_err();
        assert!(err.to_string().contains("quiet") || err.to_string().contains("MIT_QUIET_PERIOD") || err.to_string().contains("ms"), "{err}");
        assert!(bus.tx_log().is_empty(), "quiet-period refusal sent frames: {:?}", bus.tx_log());
        clock.advance(Duration::from_millis(350));
        for _ in 0..4 {
            clock.advance(Duration::from_millis(120));
            inner.tick(DT);
        }
        assert!(
            bus.tx_log().is_empty(),
            "failed quiet-period clear must NOT ack fence or unlock autonomous CAN refresh: {:?}",
            bus.tx_log()
        );
    }

    // (b) WatchdogTimeout with active lease + stale heartbeat MUST refuse in pre_ctx BEFORE
    //     even polling 0xCC, leaving fence unacked and state unchanged.
    {
        let (mut inner, bus, clock) = setup_mit_lost(true);
        inner.lease_enabled = true;
        let _ = inner.acquire_lease("mit-wd").unwrap();
        inner.last_heartbeat = Instant::now() - Duration::from_millis(80);
        inner.safety.latch(FaultKind::WatchdogTimeout, FaultDetail::WatchdogTimeout);
        bus.clear_log();
        let err = inner.clear_fault_confirmed(true).unwrap_err();
        assert!(err.to_string().contains("heartbeat"), "expected heartbeat refusal: {err}");
        assert!(bus.tx_log().is_empty(), "stale heartbeat must refuse before any CAN poll: {:?}", bus.tx_log());
        assert_eq!(inner.mit.owner, mit::MitOwner::Lost);
        for _ in 0..4 {
            clock.advance(Duration::from_millis(120));
            inner.tick(DT);
        }
        assert!(bus.tx_log().is_empty(), "fence must stay unacked after heartbeat refusal: {:?}", bus.tx_log());
    }

    // (c) Hot ThermalTrip during MIT recovery MUST refuse in full clear_check with ZERO fence ack
    //     and ZERO supervisor state mutation (temperatures/positions/mit.owner unchanged).
    {
        let (mut inner, bus, clock) = setup_mit_lost(true);
        inner.safety.latch(
            FaultKind::ThermalTrip,
            FaultDetail::ThermalTrip { joint_idx: 0, temperature: 82.0, limit: 75.0 },
        );
        let temps_before = inner.temperatures.clone();
        let pos_before = inner.measured_positions.clone();
        bus.with(|s| {
            s.motors[0].temperature_c = 85;
            s.motors[0].q = 0.25;
        });
        bus.clear_log();
        let err = inner.clear_fault_confirmed(true).unwrap_err();
        assert!(err.to_string().contains("Temperature must be confirmed below the reset threshold"), "{err}");
        assert_eq!(inner.mit.owner, mit::MitOwner::Lost, "mit.owner must not change before clear_check passes");
        assert_eq!(inner.temperatures, temps_before, "temperatures must not mutate when clear_check refuses");
        assert_eq!(inner.measured_positions, pos_before, "measured_positions must not mutate when clear_check refuses");
        assert!(
            bus.tx_log().iter().all(|f| f.id == 0x7FF && f.data[2] == 0xCC),
            "no recovery commands allowed when clear_check refuses: {:?}",
            bus.tx_log()
        );
        // Subsequent ticks MUST send 0 autonomous CAN refresh frames (fence was NOT acked!).
        bus.clear_log();
        for _ in 0..4 {
            clock.advance(Duration::from_millis(120));
            inner.tick(DT);
        }
        assert!(
            bus.tx_log().is_empty(),
            "autonomous CAN refresh must stay blocked after ThermalTrip refused MIT clear: {:?}",
            bus.tx_log()
        );

        // (d) Cool motor 0 to 35 C and move J0 to 0.40 rad, but omit confirmation -> refused with
        //     RECOVERY_CONFIRM_CODE, still ZERO fence ack and ZERO state mutation!
        let pos_before_unconf = inner.measured_positions.clone();
        let temps_before_unconf = inner.temperatures.clone();
        bus.with(|s| {
            s.motors[0].temperature_c = 35;
            s.motors[0].q = 0.40;
        });
        let err_unconf = inner.clear_fault_confirmed(false).unwrap_err();
        assert!(matches!(err_unconf, ArmError::Rejected { ref code, .. } if code == mit::RECOVERY_CONFIRM_CODE), "{err_unconf}");
        assert_eq!(inner.mit.owner, mit::MitOwner::Lost);
        assert_eq!(inner.measured_positions, pos_before_unconf);
        assert_eq!(inner.temperatures, temps_before_unconf);
        bus.clear_log();
        for _ in 0..4 {
            clock.advance(Duration::from_millis(120));
            inner.tick(DT);
        }
        assert!(
            bus.tx_log().is_empty(),
            "autonomous CAN refresh must stay blocked when recovery confirmation is missing: {:?}",
            bus.tx_log()
        );

        // (e) Now full clear_check passes + confirmed -> recovery succeeds, clears to Idle,
        //     mit.owner becomes Off, and fence is now acknowledged so read-only polls may resume.
        let cleared = inner.clear_fault_confirmed(true).unwrap();
        assert_eq!(cleared, SafetyState::Idle);
        assert_eq!(inner.mit.owner, mit::MitOwner::Off);
        assert!(!inner.is_torqued);
        bus.clear_log();
        clock.advance(Duration::from_millis(150));
        inner.tick(DT);
        assert!(
            bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data[2] == 0xCC),
            "after full clear_check passes and recovery clears, read-only 0xCC polling resumes"
        );
    }
}

#[test]
fn mit_recovery_final_read_rejects_cached_feedback_without_post_recovery_rx_seq() {
    use armd::mit;

    // 1. Driver whose `mit_expiry_recover()` returns Ok(()) without receiving any post-recovery
    //    feedback frame (feedback_seqs stays unchanged, cached age is 5 ms <= 250 ms):
    //    final MIT check MUST refuse with FreshFeedbackRequired and keep the fault latched!
    struct CachedMitRecoveryDriver {
        dof: usize,
        seqs: Vec<u64>,
        states: Vec<JointState>,
        recovered_called: Arc<AtomicBool>,
    }
    impl ArmDriver for CachedMitRecoveryDriver {
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
        fn enable_state(&self) -> EnableState { EnableState::Disabled }
        fn is_feedback_valid(&self) -> bool { true }
        fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::from_millis(5)); self.dof] }
        fn feedback_seqs(&self) -> Vec<u64> { self.seqs.clone() }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            // Returns pre-recovery cached states without advancing `seqs`.
            Ok(self.states.clone())
        }
        fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
        fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
        fn mit_fresh_status(&mut self) -> Result<Vec<u8>, String> {
            // Simulate J4 expired (0x0D), others enabled (1) -> triggers MIT expiry recovery path.
            let mut st = vec![1u8; self.dof];
            st[3] = 0x0D;
            Ok(st)
        }
        fn mit_expiry_recover(&mut self) -> Result<(), String> {
            self.recovered_called.store(true, Ordering::SeqCst);
            // Intentionally does NOT advance `self.seqs` (simulates stale/cached readback).
            Ok(())
        }
    }

    let cfg = ArmConfig {
        resting_pose: None,
        workspace: None,
        ..ArmConfig::rebot_dm()
    };
    let recovered_called = Arc::new(AtomicBool::new(false));
    let cached_states: Vec<JointState> = cfg
        .home_positions
        .iter()
        .map(|&p| JointState {
            position: p,
            velocity: 0.0,
            torque: 0.0,
            temperature: 30.0,
        })
        .collect();
    let drv = CachedMitRecoveryDriver {
        dof: cfg.dof,
        seqs: vec![7; cfg.dof],
        states: cached_states,
        recovered_called: Arc::clone(&recovered_called),
    };
    let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
    inner.lease_enabled = false;
    inner.is_torqued = true;
    inner.feedback_numeric_valid = true;
    inner.mit.owner = mit::MitOwner::Lost;
    inner.safety.latch(FaultKind::BusLoss, FaultDetail::BusLoss);

    let err = inner.clear_fault_confirmed(true).unwrap_err();
    assert!(recovered_called.load(Ordering::SeqCst), "mit_expiry_recover must have run");
    assert!(
        err.to_string().contains("Fresh feedback is required"),
        "final MIT read must reject cached feedback whose receive sequence did not advance after recovery request: {err}"
    );
    assert!(inner.safety.state().is_stopped(), "fault must stay latched when post-recovery receive sequence did not advance");
    assert_eq!(inner.mit.owner, mit::MitOwner::Lost, "mit.owner must remain Lost when final clear_check fails");

    // 2. Real DamiaoArmDriver: pre-recovery state-0 frame sitting unread in RX before recovery
    //    cannot satisfy post-recovery feedback when motor 0 drops its disable reply!
    let clock = MockClock::new();
    let limits = dm_limits();
    let bus = FakeBus::with_clock(&limits, clock.clone());
    bus.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap();
    dm.read_joint_states().unwrap();
    dm.check_mapping_ranges();
    dm.set_torque(true).unwrap();
    // Queue a pre-recovery state-0 frame for motor 0 BEFORE mit_expiry_recover() is requested,
    // and make motor 0 drop its disable command reply.
    let stale_state0_j1 = bus.with(|s| {
        let prev_en = s.motors[0].enabled;
        s.motors[0].enabled = false;
        let f = s.motors[0].feedback_frame();
        s.motors[0].enabled = prev_en;
        s.motors[0].no_feedback = true;
        s.rx.push_back(f);
        f
    });
    let _ = stale_state0_j1;
    let err_dm = dm.mit_expiry_recover().unwrap_err();
    assert!(err_dm.contains("J1"), "pre-recovery RX frame must not satisfy J1 post-recovery boundary: {err_dm}");
    assert!(
        !dm.mit_feedback_after_recovery(7),
        "mit_feedback_after_recovery must be false when recovery did not receive post-request feedback for every joint"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn collision_escalation_and_gravity_comp_output_zero_alloc_on_control_path() {
    let log = global_log();
    let _serial = log.guard.lock().await;
    let _ = take_global_logs(log);

    // 1. Collision trip, quiet release, and escalation (both operational and while E-stop latched)
    //    must perform ZERO heap allocations on the control thread with tracing subscriber enabled.
    struct CollisionAndGravityDriver {
        dof: usize,
        torques: Arc<Mutex<[f32; 7]>>,
        positions: Arc<Mutex<[f32; 7]>>,
        last_targets: Arc<Mutex<[JointTarget; 7]>>,
        write_count: Arc<std::sync::atomic::AtomicUsize>,
        estop_hold_count: Arc<std::sync::atomic::AtomicUsize>,
        clock: MockClock,
    }
    impl ArmDriver for CollisionAndGravityDriver {
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
        fn supports_mit_mode_switch(&self) -> bool { true }
        fn set_joint_modes(&mut self, _: &[armd::driver::JointMode], _: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
        fn enable_state(&self) -> EnableState { EnableState::Enabled }
        fn is_feedback_valid(&self) -> bool { true }
        fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); self.dof] }
        fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
            out.clear();
            for _ in 0..self.dof { out.push(Some(Duration::ZERO)); }
        }
        fn feedback_all_within(&self, dof: usize, _: Duration) -> bool { self.dof == dof }
        fn feedback_joints_within(&self, joints: &[usize], _: Duration) -> bool {
            joints.iter().all(|&j| j < self.dof)
        }
        fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
            out.clear();
            for _ in 0..self.dof { out.push(Some(0)); }
        }
        fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) {
            let now = self.clock.now();
            out.clear();
            for _ in 0..self.dof { out.push(Some(now)); }
        }
        fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) {
            let now = self.clock.now();
            out.clear();
            for _ in 0..self.dof { out.push(Some(now)); }
        }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            let mut out = Vec::with_capacity(self.dof);
            self.read_joint_states_into(&mut out)?;
            Ok(out)
        }
        fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
            let qs = *self.positions.lock().unwrap();
            let taus = *self.torques.lock().unwrap();
            out.clear();
            for i in 0..self.dof {
                out.push(JointState {
                    position: qs[i],
                    velocity: 0.0,
                    torque: taus[i],
                    temperature: 30.0,
                });
            }
            Ok(())
        }
        fn read_joint_states_budgeted(&mut self, out: &mut Vec<JointState>, _: Instant, _: u16) -> Result<(u16, bool), ArmDriverError> {
            self.read_joint_states_into(out)?;
            Ok((self.dof as u16, false))
        }
        fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
            let mut dst = self.last_targets.lock().unwrap();
            for (d, s) in dst.iter_mut().zip(targets.iter()) { *d = s.clone(); }
            self.write_count.fetch_add(1, Ordering::Relaxed);
            Ok(())
        }
        fn write_estop_hold(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
            let mut dst = self.last_targets.lock().unwrap();
            for (d, s) in dst.iter_mut().zip(targets.iter()) { *d = s.clone(); }
            self.estop_hold_count.fetch_add(1, Ordering::Relaxed);
            Ok(())
        }
        fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
        fn supports_rt_executor(&self) -> Result<(), &'static str> { Ok(()) }
        fn clock_now(&self) -> Instant { self.clock.now() }
    }

    let mut cfg = ArmConfig::rebot_dm();
    cfg.joint_tracking = None;
    cfg.resting_pose = None;
    cfg.workspace = None;
    if let Some(col) = cfg.collision.as_mut() {
        col.enabled = true;
        col.persist_ticks = 2;
        col.resume_after_s = 0.04;
        col.max_auto_resumes = 1;
        col.escalate_window_s = 30.0;
    }
    if let Some(gc) = cfg.gravity_comp.as_mut() {
        gc.ramp_in_s = 0.04;
        gc.exit_hold_ms = 200;
    }

    let q_init = [0.0, -0.5, -0.5, 0.3, 0.0, 0.0, 0.0];
    let g_model = cfg.arm_model.gravity_torques(&q_init[..6]).unwrap();
    let mut tau_init = [0.0f32; 7];
    tau_init[..6].copy_from_slice(&g_model[..6]);

    let torques = Arc::new(Mutex::new(tau_init));
    let positions = Arc::new(Mutex::new(q_init));
    let last_targets = Arc::new(Mutex::new(std::array::from_fn(|_| JointTarget::position(0.0))));
    let write_count = Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let estop_hold_count = Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let drv = CollisionAndGravityDriver {
        dof: cfg.dof,
        torques: Arc::clone(&torques),
        positions: Arc::clone(&positions),
        last_targets: Arc::clone(&last_targets),
        write_count: Arc::clone(&write_count),
        estop_hold_count: Arc::clone(&estop_hold_count),
        clock: MockClock::new(),
    };

    let mut inner = SupervisorInner::new(cfg.clone()).with_driver(Box::new(drv));
    inner.lease_enabled = false;
    inner.gravity_compensated = true;
    inner.collision.enabled = true;
    inner.collision.torque_feedback = true;
    inner.measured_positions = q_init.to_vec();
    inner.commanded_positions = q_init.to_vec();
    inner.target_positions = q_init.to_vec();
    let stamp = inner.current_admission_stamp();
    let adm = inner.check_admission(stamp).unwrap();
    armd::SupervisorInner::enable_torque(adm).unwrap();
    inner.safety.set_off_thread_reclaim(true);
    // Warm up per-tick scratch buffers.
    for _ in 0..3 {
        let _ = inner.tick_control(0.02);
    }

    alloc::time_this_thread();

    // (a) First collision trip -> protective hold (0 allocs).
    torques.lock().unwrap()[0] = g_model[0] + 15.0;
    let _ = alloc::take();
    let _ = inner.tick_control(0.02);
    let _ = inner.tick_control(0.02);
    let trip1_allocs = alloc::take();
    assert_eq!(trip1_allocs.calls, 0, "collision protective-stop trip allocated: {trip1_allocs:?}");
    assert!(inner.collision.hold.is_some(), "expected protective hold on first collision trip");

    // (b) Quiet period -> collision hold release (0 allocs).
    torques.lock().unwrap()[0] = g_model[0];
    let _ = alloc::take();
    for _ in 0..3 {
        let _ = inner.tick_control(0.02);
    }
    let rel_allocs = alloc::take();
    assert_eq!(rel_allocs.calls, 0, "collision hold release allocated: {rel_allocs:?}");
    assert!(inner.collision.hold.is_none(), "expected collision hold to release after quiet period");

    // (c) Second collision trip in window -> escalation latches FaultKind::CollisionLatched (0 allocs!).
    torques.lock().unwrap()[0] = g_model[0] + 15.0;
    let _ = alloc::take();
    let _ = inner.tick_control(0.02);
    let _ = inner.tick_control(0.02);
    let esc_allocs = alloc::take();
    assert_eq!(esc_allocs.calls, 0, "collision escalation allocated: {esc_allocs:?}");
    assert!(inner.safety.has_latched_fault_kind(FaultKind::CollisionLatched));
    let fi = inner.safety.fault().expect("collision fault info");
    assert_eq!(fi.code, FaultCode::Collision);
    assert!(
        fi.message.as_str().contains("COLLISION: Collision on joint 1:") && fi.message.as_str().contains("2 trips within 30 s"),
        "unexpected collision fault message: {}",
        fi.message
    );
    // Off-thread deferred logs emit exact collision lines.
    inner.safety.drain_log_events_off_thread();
    inner.safety.drain_retired_state_arcs_off_thread();
    let col_logs = take_global_logs(log);
    assert!(
        col_logs.contains("Collision hold ended after quiet period")
            && col_logs.contains("Collision on joint 1:")
            && col_logs.contains("latching Fault COLLISION"),
        "missing deferred collision logs:\n{col_logs}"
    );

    // 2. Gravity-comp tick + Step-5 inline output (RampIn, RampIn -> Active, steady Active,
    //    and E-stop Exiting hold): ZERO heap allocations across every control tick!
    torques.lock().unwrap()[0] = g_model[0];
    assert_eq!(inner.clear_fault().unwrap(), SafetyState::Idle);
    let stamp = inner.current_admission_stamp();
    let adm = inner.check_admission(stamp).unwrap();
    armd::SupervisorInner::enable_torque(adm).unwrap();
    let _ = inner.tick_control(0.02);
    let stamp = inner.current_admission_stamp();
    let adm = inner.check_admission(stamp).unwrap();
    armd::SupervisorInner::enter_gravity_comp(adm, armd::gravity_comp::GravityCompRequest::default()).unwrap();
    // Warm up one tick after entering gravity_comp (not strictly required, keeps measurement focused on steady/ramp RT path).
    let before_writes = write_count.load(Ordering::Relaxed);
    let _ = alloc::take();
    let direct_targets = inner.gravity_comp_targets().expect("gravity_comp_targets inline");
    assert_eq!(alloc::take().calls, 0, "gravity_comp_targets() must be bounded/inline with 0 heap allocs");
    assert!(direct_targets[0].kp.is_some() && direct_targets[6].kp.is_none());

    // Measure RampIn, RampIn -> Active transition, and steady Active ticks (5 ticks = 0.10 s > 0.04 s ramp).
    let _ = alloc::take();
    for _ in 0..5 {
        let _ = inner.tick_control(0.02);
    }
    let gc_allocs = alloc::take();
    assert_eq!(gc_allocs.calls, 0, "gravity_comp tick + Step-5 output allocated: {gc_allocs:?}");
    assert!(
        write_count.load(Ordering::Relaxed) >= before_writes + 5,
        "gravity_comp Step-5 must deliver MIT targets to driver"
    );
    let sent = last_targets.lock().unwrap().clone();
    assert!(sent[1].kp == Some(0.0) && sent[1].torque.is_some_and(f32::is_finite), "expected Active MIT target: {:?}", sent[1]);

    // Trigger E-stop during gravity comp -> Exiting phase Step-5 `write_estop_hold` must also do 0 heap allocs!
    let before_holds = estop_hold_count.load(Ordering::Relaxed);
    let _ = alloc::take();
    inner.apply_stop_policy(StopPolicy::EStopLatch);
    let _ = inner.tick_control(0.02);
    let gc_estop_allocs = alloc::take();
    assert_eq!(gc_estop_allocs.calls, 0, "gravity_comp E-stop exit + hold output allocated: {gc_estop_allocs:?}");
    assert!(
        estop_hold_count.load(Ordering::Relaxed) >= before_holds + 2,
        "expected immediate + Step-5 write_estop_hold calls"
    );
    inner.safety.drain_log_events_off_thread();
    inner.safety.drain_retired_state_arcs_off_thread();
    let gc_logs = take_global_logs(log);
    assert!(
        gc_logs.contains("Gravity compensation ramp complete")
            && gc_logs.contains("Gravity compensation exit: ESTOP E-stop"),
        "missing deferred gravity_comp logs:\n{gc_logs}"
    );
}
