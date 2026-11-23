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
    let res_other = sm.try_trigger_fault_coded(FaultCode::HardwareFault, other);
    sm.latch(
        FaultKind::CollisionLatched,
        FaultDetail::formatted(FaultCode::Collision, |b| b.push_str("collision escalated")),
    );
    sm.latch(FaultKind::Estop, FaultDetail::Estop);
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
    assert_eq!(sm.fault_code(), Some(FaultCode::ThermalTrip), "most severe recorded kind must win reported code");
    assert!(!sm.recoverable());

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
            ClearContext { joints_within_limits: false, ..base_met },
            ClearRefusal::JointLimitsNotSatisfied,
            ClearContext { joints_within_limits: true, ..base_met },
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
    sm_unknown.record_unknown_kind_bit_for_test(1u8 << 7);
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
    // JointLimit: out of bounds -> refused; back within limits + margin -> cleared.
    inner.measured_positions[0] = inner.config.limits[0].max_position + 0.2;
    inner.safety.latch(
        FaultKind::JointLimit,
        FaultDetail::formatted(FaultCode::JointLimit, |b| b.push_str("joint 1 out of bounds")),
    );
    assert!(inner.clear_fault().unwrap_err().to_string().contains("within limits"));
    assert_eq!(
        inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)),
        Err(ClearFaultError::Rejected(ClearRefusal::JointLimitsNotSatisfied))
    );
    // Just inside outward margin (0.005 < JOINT_LIMIT_CLEAR_MARGIN_RAD = 0.01) -> clears.
    inner.measured_positions[0] = inner.config.limits[0].max_position + 0.005;
    assert_eq!(inner.clear_fault().unwrap(), SafetyState::Idle);

    // WatchdogTimeout: active lease with stale heartbeat -> refused; fresh heartbeat or release -> cleared.
    let gen = inner.acquire_lease("client-w").unwrap();
    inner.safety.latch(FaultKind::WatchdogTimeout, FaultDetail::WatchdogTimeout);
    std::thread::sleep(Duration::from_millis(45));
    assert_eq!(
        inner.clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50)),
        Err(ClearFaultError::Rejected(ClearRefusal::WatchdogHeartbeatRequired))
    );
    assert!(inner.clear_fault().unwrap_err().to_string().contains("Fresh heartbeat"));
    // Re-acquiring lease refreshes heartbeat -> clears cleanly.
    let _ = inner.release_lease("client-w", gen);
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
        // Bus fresh, joints still outside limits -> refused
        assert_eq!(
            sm.clear_check(&ClearContext { joints_within_limits: false, ..driver_met }),
            Err(ClearRefusal::JointLimitsNotSatisfied)
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
    let mut sm = SafetyManager::new();
    sm.latch(FaultKind::JointLimit, FaultDetail::formatted(FaultCode::JointLimit, |b| b.push_str("jl")));
    sm.latch(FaultKind::ThermalTrip, FaultDetail::ThermalTrip { joint_idx: 0, temperature: 90.0, limit: 75.0 });
    assert_eq!(sm.fault_code(), Some(FaultCode::ThermalTrip));
    assert_eq!(
        sm.clear_check(&ClearContext { joints_within_limits: false, ..driver_met }),
        Err(ClearRefusal::JointLimitsNotSatisfied)
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
    inner.set_torque(true).unwrap();
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
