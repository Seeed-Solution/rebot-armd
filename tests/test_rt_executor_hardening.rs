//! Targeted regression tests for RT Stage 3 Step 1 R3 Class-A hardening:
//! 1. Legacy Damiao mode-restore retry E-stop gate (`transmit_with_retry_guarded`)
//!    + preservation of legacy gravity-comp exit `set_joint_modes` after E-stop.
//! 2. Final E-stop recheck before `can.flush()` on Damiao target batches (normal + MIT fenced).
//! 3. Bounded multi-slot off-thread fault reclamation (`SafetyManager::RETIRED_STATE_CAPACITY`)
//!    with no overwrite on consecutive faults and fail-closed refusal when full.
//! 4. PositionLead enable completion on a sub-release wake with 0 alloc/free across enable/disable cycles.
//! 5. SimArmDriver single transport writer (owner-routed keepalive, 0 `armd-sim-rx` sends after handoff)
//!    and strictly non-blocking RT reads when `armd-sim-rx` holds the shared state mutex.

mod mit_support;

use std::net::UdpSocket;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use armd::diagnostics::alloc::TimedAlloc;
use armd::driver::{
    is_estop_inhibited, ArmDriver, ArmDriverError, DamiaoArmDriver, DriverCapabilities, EnableState,
    GripperPosForce, JointMode, JointTarget, TorqueOutcome, TorqueProgress,
};
use armd::model::{ArmConfig, JointState};
use armd::protocol::damiao::DamiaoLimits;
use armd::rt_core::broker::{ClientId, Outcome, Reply, RequestId, Submit, TransitionOp};
use armd::rt_core::executor::{ExecutorConfig, PanicPolicy, PersistTestHooks, RtExecutorHandle};
use armd::rt_core::urgent::EstopGate;
use armd::safety::{FaultCode, FaultReason, SafetyError, SafetyManager, SafetyState, RETIRED_STATE_CAPACITY};
use armd::sim_driver::{SimArmDriver, SimCommand, SimState};
use armd::supervisor::{ArmSupervisor, PanicDisableOutcome, PreemptError};
use armd::tracking::JointTrackingConfig;
use armd::transport::{CanFrame, CanTransport, DriverClock, MockCanTransport, MockClock, TransportError};
use mit_support::FakeBus;

#[global_allocator]
static ALLOC: TimedAlloc = TimedAlloc;

fn dm_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm()
        .damiao_ranges
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect()
}

async fn wait_until_async(timeout: Duration, mut pred: impl FnMut() -> bool) {
    let deadline = Instant::now() + timeout;
    while !pred() {
        assert!(Instant::now() < deadline, "condition timed out after {timeout:?}");
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
}

fn submit_broker_transition(handle: &RtExecutorHandle, seq: u64, op: TransitionOp) -> RequestId {
    let snap = handle.planning_snapshot().expect("planning_snapshot");
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut broker = handle.broker().lock().unwrap();
    broker
        .submit(ClientId(1), seq, snap.authority, deadline, Submit::Transition(op))
        .expect("submit transition")
}

fn wait_broker_terminal(handle: &RtExecutorHandle, req_id: RequestId, timeout: Duration) -> Outcome {
    let deadline = Instant::now() + timeout;
    loop {
        let mut found = None;
        {
            let mut broker = handle.broker().lock().unwrap();
            broker.poll(|_, reply| {
                if let Reply::Terminal { request, outcome, .. } = reply {
                    if request == req_id {
                        found = Some(outcome);
                    }
                }
                true
            });
        }
        if let Some(outcome) = found {
            return outcome;
        }
        assert!(Instant::now() < deadline, "timed out waiting for broker terminal for req {req_id:?}");
        thread::sleep(Duration::from_millis(2));
    }
}

/// Clock wrapper that optionally triggers a hook (e.g., latching E-stop) inside `sleep()`.
#[derive(Clone)]
struct HookClock {
    inner: MockClock,
    on_sleep: Option<Arc<dyn Fn() + Send + Sync>>,
    sleeps: Arc<AtomicUsize>,
}

impl DriverClock for HookClock {
    fn clock_id(&self) -> u64 { self.inner.clock_id() }
    fn now(&self) -> Instant { self.inner.now() }
    fn sleep(&self, d: Duration) {
        self.sleeps.fetch_add(1, Ordering::SeqCst);
        if let Some(ref hook) = self.on_sleep {
            hook();
        }
        self.inner.sleep(d);
    }
    fn yield_now(&self) { self.inner.yield_now() }
}

/// Controllable CAN wrapper over `FakeBus` for deterministic WouldBlock + flush E-stop injection.
#[derive(Clone)]
struct FaultInjectBus {
    inner: FakeBus,
    clock: HookClock,
    tx_calls: Arc<AtomicUsize>,
    flush_calls: Arc<AtomicUsize>,
    would_block_first_n: Arc<AtomicUsize>,
    estop_on_tx_ordinal: Option<(Arc<AtomicU64>, Arc<AtomicUsize>)>,
}

impl FaultInjectBus {
    fn new(inner: FakeBus, clock: HookClock) -> Self {
        Self {
            inner,
            clock,
            tx_calls: Arc::new(AtomicUsize::new(0)),
            flush_calls: Arc::new(AtomicUsize::new(0)),
            would_block_first_n: Arc::new(AtomicUsize::new(0)),
            estop_on_tx_ordinal: None,
        }
    }
}

impl CanTransport for FaultInjectBus {
    fn clock(&self) -> Option<Arc<dyn DriverClock>> { Some(Arc::new(self.clock.clone())) }
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        let ord = self.tx_calls.fetch_add(1, Ordering::SeqCst) + 1;
        let wb = self.would_block_first_n.load(Ordering::SeqCst);
        if ord <= wb {
            return Err(TransportError::WouldBlock);
        }
        let r = self.inner.transmit(frame);
        if let Some((ref ctr, ref target)) = self.estop_on_tx_ordinal {
            let t = target.load(Ordering::SeqCst);
            if t > 0 && ord == t {
                ctr.fetch_add(1, Ordering::SeqCst);
            }
        }
        r
    }
    fn receive(&mut self) -> Result<CanFrame, TransportError> { self.inner.receive() }
    fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> { self.inner.receive_timestamped() }
    fn flush(&mut self) -> Result<(), TransportError> {
        self.flush_calls.fetch_add(1, Ordering::SeqCst);
        self.inner.flush()
    }
    fn supports_bounded_flush(&self) -> bool { true }
    fn supports_nonblocking_rt(&self) -> bool { true }
}

// ============================================================================
// 1 & 2. Damiao legacy mode-restore retry E-stop gate + final flush E-stop check
// ============================================================================

#[test]
fn damiao_legacy_mode_restore_retry_and_final_target_flush_respect_estop_gate() {
    let base_clock = MockClock::new();

    // (a) Legacy mode-restore retry (`rt_no_sleep == false`): 1st transmit returns WouldBlock,
    // E-stop latches inside the 100 us retry sleep -> attempt 2 MUST NOT transmit, flush MUST NOT run,
    // and `begin_torque(true)` returns `EstopInhibited`!
    let raw_r = FakeBus::with_clock(&dm_limits(), base_clock.clone());
    raw_r.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let estop_req_r = Arc::new(AtomicU64::new(0));
    let estop_app_r = Arc::new(AtomicU64::new(0));
    let latch_hook = estop_req_r.clone();
    let hook_clock = HookClock {
        inner: base_clock.clone(),
        on_sleep: Some(Arc::new(move || {
            latch_hook.fetch_add(1, Ordering::SeqCst);
        })),
        sleeps: Arc::new(AtomicUsize::new(0)),
    };
    let bus_r = FaultInjectBus::new(raw_r.clone(), hook_clock.clone());
    let mut drv_r = DamiaoArmDriver::new(bus_r.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap()
        .with_gripper_pos_force(GripperPosForce {
            joint: 6,
            current_raw: 1500,
            close_speed_rad_s: 1.0,
            closed_rad: -0.2,
            open_rad: 0.6,
            calibrated: true,
            ceiling_torque_nm: Some(1.0),
            torque_reference: Some("bench".into()),
        })
        .unwrap();
    drv_r.set_estop_gate(EstopGate::new(estop_req_r.clone(), estop_app_r));
    drv_r.read_joint_states().unwrap();
    // Establish an empty-queue receive bound before pushing fresh J7 feedback so
    // `fresh_measured_position(6)` passes and `begin_torque(true)` enters `write_modes_send`.
    raw_r.with(|s| s.push_all_feedback());
    drv_r.read_joint_states().unwrap();
    // Legacy path (`rt_no_sleep` remains false); inject WouldBlock on attempt 1 of mode restore.
    bus_r.tx_calls.store(0, Ordering::SeqCst);
    bus_r.flush_calls.store(0, Ordering::SeqCst);
    hook_clock.sleeps.store(0, Ordering::SeqCst);
    bus_r.would_block_first_n.store(1, Ordering::SeqCst);
    let err = drv_r.begin_torque(true).unwrap_err();
    assert!(is_estop_inhibited(&err), "expected EstopInhibited on retry pause E-stop, got {err:?}");
    assert_eq!(hook_clock.sleeps.load(Ordering::SeqCst), 1, "expected exactly one 100us retry sleep");
    assert_eq!(bus_r.tx_calls.load(Ordering::SeqCst), 1, "retry attempt 2 must NOT transmit after E-stop");
    assert_eq!(bus_r.flush_calls.load(Ordering::SeqCst), 0, "flush must NOT run after retry E-stop");
    assert!(!drv_r.torque_transition_pending());

    // Legacy `set_joint_modes` rollback:
    // - While E-stop is latched (`abort_flag = true`), CTRL_MODE writes are `ModeWrite` and are
    //   refused with 0 TX frames (no mode register changes during latched E-stop).
    // - Once the E-stop latch is cleared, `set_joint_modes` still retries through `WouldBlock`
    //   and succeeds on the off-tick legacy path.
    let raw_gc = FakeBus::with_clock(&dm_limits(), base_clock.clone());
    raw_gc.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let gc_clock = HookClock {
        inner: base_clock.clone(),
        on_sleep: None,
        sleeps: Arc::new(AtomicUsize::new(0)),
    };
    let bus_gc = FaultInjectBus::new(raw_gc.clone(), gc_clock.clone());
    let mut drv_gc = DamiaoArmDriver::new(bus_gc.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    let hold = vec![JointTarget::position(0.0); 7];
    // Enter MIT mode on J1..J6 first (like gravity_comp), then latch E-stop and attempt rollback.
    let mit_modes = vec![
        JointMode::Mit, JointMode::Mit, JointMode::Mit,
        JointMode::Mit, JointMode::Mit, JointMode::Mit,
        JointMode::Position,
    ];
    drv_gc.read_joint_states().unwrap();
    drv_gc.set_joint_modes(&mit_modes, &hold).unwrap();
    let gc_req = Arc::new(AtomicU64::new(1));
    let gc_app = Arc::new(AtomicU64::new(1));
    let gc_abort = Arc::new(AtomicBool::new(true));
    drv_gc.set_estop_gate(EstopGate::new(gc_req, gc_app));
    drv_gc.set_abort_flag(gc_abort.clone());
    // Refresh fresh feedback (empty-queue boundary + state).
    drv_gc.read_joint_states().unwrap();
    raw_gc.with(|s| s.push_all_feedback());
    drv_gc.read_joint_states().unwrap();
    bus_gc.tx_calls.store(0, Ordering::SeqCst);
    gc_clock.sleeps.store(0, Ordering::SeqCst);
    let pos_modes = vec![JointMode::Position; 7];
    let latched_err = drv_gc
        .set_joint_modes(&pos_modes, &hold)
        .expect_err("latched E-stop must refuse CTRL_MODE rollback writes");
    assert!(is_estop_inhibited(&latched_err), "expected EstopInhibited, got {latched_err:?}");
    assert_eq!(bus_gc.tx_calls.load(Ordering::SeqCst), 0, "latched rollback must transmit 0 CTRL_MODE frames");

    // After latch clearance, legacy off-tick rollback retries WouldBlock and succeeds.
    gc_abort.store(false, Ordering::SeqCst);
    bus_gc.would_block_first_n.store(1, Ordering::SeqCst);
    drv_gc
        .set_joint_modes(&pos_modes, &hold)
        .expect("legacy gravity-comp exit set_joint_modes retries WouldBlock after latch clear");
    assert!(gc_clock.sleeps.load(Ordering::SeqCst) >= 1, "legacy gravity-comp exit should retry WouldBlock");

    // (b1) Normal PosVel target batch: E-stop arrives during the 7th (final) target transmit ->
    // `write_joint_targets` MUST return `EstopInhibited` and skip `can.flush()` (`flush_calls == 0`)!
    let raw_t = FakeBus::with_clock(&dm_limits(), base_clock.clone());
    raw_t.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let estop_req_t = Arc::new(AtomicU64::new(0));
    let estop_app_t = Arc::new(AtomicU64::new(0));
    let t_clock = HookClock { inner: base_clock.clone(), on_sleep: None, sleeps: Arc::new(AtomicUsize::new(0)) };
    let trig_t = Arc::new(AtomicUsize::new(0));
    let mut bus_t = FaultInjectBus::new(raw_t.clone(), t_clock);
    bus_t.estop_on_tx_ordinal = Some((estop_req_t.clone(), trig_t.clone()));
    let mut drv_t = DamiaoArmDriver::new(bus_t.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv_t.set_estop_gate(EstopGate::new(estop_req_t.clone(), estop_app_t));
    drv_t.read_joint_states().unwrap();
    drv_t.check_mapping_ranges();
    drv_t.set_torque(true).unwrap();
    bus_t.tx_calls.store(0, Ordering::SeqCst);
    bus_t.flush_calls.store(0, Ordering::SeqCst);
    estop_req_t.store(0, Ordering::SeqCst);
    trig_t.store(7, Ordering::SeqCst); // latches during final frame #7!
    let targets = vec![JointTarget::position(0.0); 7];
    let err_t = drv_t.write_joint_targets(&targets).unwrap_err();
    assert!(is_estop_inhibited(&err_t), "final-frame E-stop must return EstopInhibited, got {err_t:?}");
    assert_eq!(bus_t.tx_calls.load(Ordering::SeqCst), 7);
    assert_eq!(bus_t.flush_calls.load(Ordering::SeqCst), 0, "can.flush() must be skipped on final-frame E-stop");

    // (b2) Fenced MIT target batch: E-stop arrives during the 7th (final) target transmit ->
    // `write_joint_targets` MUST skip `can.flush()`, trip the MIT fence, and return `EstopInhibited`!
    let raw_f = FakeBus::with_clock(&dm_limits(), base_clock.clone());
    raw_f.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let estop_req_f = Arc::new(AtomicU64::new(0));
    let estop_app_f = Arc::new(AtomicU64::new(0));
    let f_clock = HookClock { inner: base_clock.clone(), on_sleep: None, sleeps: Arc::new(AtomicUsize::new(0)) };
    let trig_f = Arc::new(AtomicUsize::new(0));
    let mut bus_f = FaultInjectBus::new(raw_f.clone(), f_clock);
    bus_f.estop_on_tx_ordinal = Some((estop_req_f.clone(), trig_f.clone()));
    let mut drv_f = DamiaoArmDriver::new(bus_f.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv_f.set_estop_gate(EstopGate::new(estop_req_f.clone(), estop_app_f));
    drv_f.read_joint_states().unwrap();
    drv_f.check_mapping_ranges();
    drv_f.set_torque(true).unwrap();
    drv_f.set_mit_deadline_fence(Some(Duration::from_millis(45)), Duration::from_millis(10));
    bus_f.tx_calls.store(0, Ordering::SeqCst);
    bus_f.flush_calls.store(0, Ordering::SeqCst);
    estop_req_f.store(0, Ordering::SeqCst);
    trig_f.store(7, Ordering::SeqCst); // latches during final frame #7!
    let err_f = drv_f.write_joint_targets(&targets).unwrap_err();
    assert!(is_estop_inhibited(&err_f), "fenced final-frame E-stop must return EstopInhibited, got {err_f:?}");
    assert_eq!(bus_f.tx_calls.load(Ordering::SeqCst), 7);
    assert_eq!(bus_f.flush_calls.load(Ordering::SeqCst), 0, "fenced can.flush() must be skipped on final-frame E-stop");
    assert!(drv_f.mit_fence_trip().is_some(), "interrupted fenced batch must trip MIT fence");
}

// ============================================================================
// 3. Bounded multi-slot off-thread fault reclamation + fail-closed when full
// ============================================================================

#[test]
fn safety_multi_slot_off_thread_reclaim_preserves_consecutive_faults_and_fails_closed_when_full() {
    let mut sm = SafetyManager::new();
    sm.set_off_thread_reclaim(true);

    // (a) Consecutive faults (`Other(String)` -> `ThermalTrip` -> `ClearFault`) without draining in between:
    // neither slot overwrites the other, and `clear_fault` + popping retired arcs does 0 alloc/free!
    sm.trigger_fault_coded(FaultCode::Other, FaultReason::Other("sole-ref hardware fault".into()));
    let hw_arc_weak = Arc::downgrade(&sm.state_arc());
    sm.trigger_fault_coded(
        FaultCode::ThermalTrip,
        FaultReason::ThermalTrip { joint_idx: 0, temperature: 95.0, limit: 85.0 },
    );
    assert!(hw_arc_weak.upgrade().is_some(), "consecutive ThermalTrip must NOT drop previous heap Fault");
    assert_eq!(sm.retired_state_arcs_len(), 1);

    armd::diagnostics::alloc::time_this_thread();
    let _ = armd::diagnostics::alloc::take();
    let cleared = sm.clear_fault().expect("clear_fault succeeds while reclaim ring has room");
    let first = sm.pop_retired_state_arc();
    let second = sm.pop_retired_state_arc();
    let clear_allocs = armd::diagnostics::alloc::take();
    assert_eq!(cleared, SafetyState::Idle);
    assert_eq!(clear_allocs.calls, 0, "clear_fault + pop_retired_state_arc must do 0 alloc/free: {clear_allocs:?}");
    assert!(matches!(first.as_deref(), Some(SafetyState::Fault(FaultReason::Other(s))) if s == "sole-ref hardware fault"));
    assert!(matches!(second.as_deref(), Some(SafetyState::Fault(FaultReason::ThermalTrip { .. }))));
    assert_eq!(sm.retired_state_arcs_len(), 0);
    drop(first);
    drop(second);
    assert!(hw_arc_weak.upgrade().is_none(), "retired Fault dropped cleanly once released off-thread");

    // (b) Fill all `RETIRED_STATE_CAPACITY` (16) slots + latch a 17th heap Fault -> FAIL CLOSED!
    let mut weaks = Vec::new();
    for i in 0..RETIRED_STATE_CAPACITY {
        sm.trigger_fault_coded(FaultCode::Other, FaultReason::Other(format!("fault-{i}")));
        weaks.push(Arc::downgrade(&sm.state_arc()));
        sm.clear_fault().unwrap();
    }
    assert!(sm.retired_state_arcs_full());
    assert_eq!(sm.retired_state_arcs_len(), RETIRED_STATE_CAPACITY);

    sm.trigger_fault_coded(FaultCode::Other, FaultReason::Other("latched-17".into()));
    let latched_weak = Arc::downgrade(&sm.state_arc());

    // `clear_fault()` MUST fail closed: keep `latched-17` faulted and drop 0 retired arcs!
    let err = sm.clear_fault().unwrap_err();
    assert!(matches!(err, SafetyError::ClearFaultRejected(_)), "expected ClearFaultRejected when full, got {err:?}");
    assert!(matches!(sm.state(), SafetyState::Fault(FaultReason::Other(s)) if s == "latched-17"));
    assert_eq!(sm.retired_state_arcs_len(), RETIRED_STATE_CAPACITY);
    assert!(weaks.iter().all(|w| w.upgrade().is_some()), "no queued Fault Arc may be overwritten/dropped");

    // Heap-bearing faults (`Other(String)` and `JointLimitExceeded { joint_name: String }`) while full:
    //   - `trigger_fault_coded_lazy` MUST NOT invoke the closure (0 alloc / 0 free!)
    //   - `try_trigger_fault_coded` with a prebuilt heap FaultReason MUST return `Err((epoch, reason))`
    //     without freeing `reason` on the RT thread (0 alloc / 0 free until dropped off-measurement!)
    assert!(!sm.can_accept_fault_transition());
    let ep_before = sm.epoch_id();
    let lazy_called = AtomicBool::new(false);
    let prebuilt_other = FaultReason::Other("prebuilt-heap-other-payload".into());
    let prebuilt_jl = FaultReason::JointLimitExceeded {
        joint_idx: 2,
        joint_name: "prebuilt_joint_3_elbow".into(),
        value: 3.1,
        min: -2.0,
        max: 2.0,
    };
    let _ = armd::diagnostics::alloc::take();
    let ep_after_lazy = sm.trigger_fault_coded_lazy(FaultCode::Other, || {
        lazy_called.store(true, Ordering::SeqCst);
        FaultReason::Other("must-never-allocate".into())
    });
    let res_other = sm.try_trigger_fault_coded(FaultCode::Other, prebuilt_other);
    let res_jl = sm.try_trigger_fault_coded(FaultCode::JointLimit, prebuilt_jl);
    let ep_after_th = sm.trigger_fault_coded(
        FaultCode::ThermalTrip,
        FaultReason::ThermalTrip { joint_idx: 1, temperature: 96.0, limit: 85.0 },
    );
    let overflow_allocs = armd::diagnostics::alloc::take();
    assert!(!lazy_called.load(Ordering::SeqCst), "lazy closure must not run when reclaim queue is full");
    assert!(ep_after_lazy > ep_before);
    assert!(ep_after_th > ep_after_lazy);
    assert_eq!(
        overflow_allocs.calls, 0,
        "fail-closed lazy/fallible heap fault transitions while full must do 0 alloc/free: {overflow_allocs:?}"
    );
    let (_, ret_other) = res_other.expect_err("try_trigger_fault_coded must return Err when full");
    let (_, ret_jl) = res_jl.expect_err("try_trigger_fault_coded(JointLimit) must return Err when full");
    drop(ret_other);
    drop(ret_jl);
    assert!(matches!(sm.state(), SafetyState::Fault(FaultReason::Other(s)) if s == "latched-17"));
    assert!(latched_weak.upgrade().is_some());

    // Draining 1 slot off-thread restores room so `clear_fault()` now succeeds!
    let popped = sm.pop_retired_state_arc().unwrap();
    drop(popped);
    assert_eq!(sm.clear_fault().unwrap(), SafetyState::Idle);
    sm.drain_retired_state_arcs_off_thread();
    assert!(latched_weak.upgrade().is_none());
}

// ============================================================================
// 4. PositionLead enable completion on a sub-release wake: 0 alloc / 0 free
// ============================================================================

struct DelayedWakeDriver {
    dof: usize,
    confirm_delay: Duration,
    states: Vec<JointState>,
    enabled: bool,
    pending: Option<(bool, Instant)>,
}

impl DelayedWakeDriver {
    fn new(dof: usize, confirm_delay: Duration) -> Self {
        Self {
            dof,
            confirm_delay,
            states: vec![
                JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 30.0 };
                dof
            ],
            enabled: false,
            pending: None,
        }
    }
}

impl ArmDriver for DelayedWakeDriver {
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
    fn enable_state(&self) -> EnableState {
        if self.enabled { EnableState::Enabled } else { EnableState::Disabled }
    }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> { Ok(self.states.clone()) }
    fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
        out.clear();
        out.extend_from_slice(&self.states);
        Ok(())
    }
    fn read_joint_states_budgeted(&mut self, out: &mut Vec<JointState>, _: Instant, _: u16) -> Result<(u16, bool), ArmDriverError> {
        self.read_joint_states_into(out)?;
        Ok((self.dof as u16, false))
    }
    fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
        out.clear();
        out.extend_from_slice(&self.states);
        Some(())
    }
    fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        self.enabled = enabled;
        Ok(())
    }
    fn begin_torque(&mut self, enabled: bool) -> Result<TorqueProgress, ArmDriverError> {
        let ready_at = Instant::now() + self.confirm_delay;
        self.pending = Some((enabled, ready_at));
        Ok(TorqueProgress::Pending { wake: ready_at })
    }
    fn begin_torque_budgeted(&mut self, enabled: bool, _: u16) -> Result<TorqueProgress, ArmDriverError> {
        self.begin_torque(enabled)
    }
    fn poll_torque(&mut self, _: Instant) -> Result<TorqueProgress, ArmDriverError> {
        let Some((enabled, ready_at)) = self.pending else {
            return Err(ArmDriverError::Protocol("no transition".into()));
        };
        if Instant::now() >= ready_at {
            self.pending = None;
            self.enabled = enabled;
            Ok(TorqueProgress::Done(TorqueOutcome::Confirmed))
        } else {
            Ok(TorqueProgress::Pending { wake: ready_at })
        }
    }
    fn poll_torque_budgeted(&mut self, deadline: Instant, _: u16) -> Result<TorqueProgress, ArmDriverError> {
        self.poll_torque(deadline)
    }
    fn torque_transition_pending(&self) -> bool { self.pending.is_some() }
    fn supports_rt_executor(&self) -> Result<(), &'static str> { Ok(()) }
    fn is_feedback_valid(&self) -> bool { true }
    fn is_enabled(&self) -> bool { self.enabled }
    fn fresh_joint_positions_into(&self, joints: &[usize], max_age: Duration, out: &mut [f32]) -> bool {
        if joints.is_empty() || out.len() < joints.len() || Duration::from_millis(1) > max_age {
            return false;
        }
        for (dst, &j) in out.iter_mut().zip(joints.iter()) {
            let Some(s) = self.states.get(j) else {
                return false;
            };
            if !s.position.is_finite() {
                return false;
            }
            *dst = s.position;
        }
        true
    }
    fn feedback_all_within(&self, dof: usize, _: Duration) -> bool { dof == self.dof }
    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) { out.clear(); out.resize(self.dof, Some(Duration::from_millis(1))); }
    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) { out.clear(); out.resize(self.dof, Some(1)); }
    fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) { out.clear(); out.resize(self.dof, Some(Instant::now())); }
    fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) { out.clear(); out.resize(self.dof, Some(Instant::now())); }
    fn feedback_seqs_into(&self, out: &mut Vec<u64>) { out.clear(); }
    fn feedback_queue_bounds_into(&self, out: &mut Vec<Option<Instant>>) { out.clear(); }
    fn motor_status_into(&self, out: &mut Vec<u8>) -> bool { out.clear(); out.resize(self.dof, u8::from(self.enabled)); true }
    fn joint_loads_into(&self, out: &mut Vec<f32>) -> bool { out.clear(); false }
    fn take_recovery_writes_into(&mut self, out: &mut Vec<usize>) { out.clear(); }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn position_lead_enable_completion_on_sub_release_wake_does_zero_allocs() {
    let mut cfg = ArmConfig::rebot_dm();
    let mut jt = JointTrackingConfig::rebot_dm();
    jt.enabled = true; // PositionLead on all 6 arm joints (no MIT joints -> admitted by RT gate!)
    cfg.joint_tracking = Some(jt);

    let drv = DelayedWakeDriver::new(7, Duration::from_millis(14));
    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;
    sup.register_ws_connection(1).await.unwrap();
    let mut exec_cfg = ExecutorConfig::for_tests(Duration::from_millis(10));
    exec_cfg.alloc_timing = true;
    let handle = sup.start_executor(exec_cfg).await.expect("PositionLead profile admitted");
    wait_until_async(Duration::from_secs(1), || handle.stats().steps.load(Ordering::Relaxed) >= 4).await;

    // Cycle 1: enable completes on sub-release wake (`TrackingRuntime::seed()` + `arm_joints()`).
    let r1 = submit_broker_transition(&handle, 1, TransitionOp::TorqueOn);
    assert_eq!(wait_broker_terminal(&handle, r1, Duration::from_secs(2)), Outcome::Completed);
    assert!(handle.stats().wakes.load(Ordering::Relaxed) >= 1, "expected >= 1 real sub-release wake");
    assert_eq!(
        handle.stats().wake_alloc_calls.load(Ordering::Relaxed),
        0,
        "PositionLead enable completion wake must perform 0 alloc/free"
    );

    // Cycle 2: disable (`invalidate()` parks spare buffers) -> re-enable (`seed()` reuses spares) -> STILL 0!
    let r2 = submit_broker_transition(&handle, 2, TransitionOp::TorqueOff);
    assert_eq!(wait_broker_terminal(&handle, r2, Duration::from_secs(2)), Outcome::Completed);
    let r3 = submit_broker_transition(&handle, 3, TransitionOp::TorqueOn);
    assert_eq!(wait_broker_terminal(&handle, r3, Duration::from_secs(2)), Outcome::Completed);
    assert_eq!(
        handle.stats().wake_alloc_calls.load(Ordering::Relaxed),
        0,
        "PositionLead disable -> re-enable wake cycle must perform 0 alloc/free"
    );
    handle.stop_and_join();
}

// ============================================================================
// 5 & 6. SimArmDriver single transport writer + non-blocking RT reads
// ============================================================================

struct PollCountingSim {
    addr: String,
    stop: Arc<AtomicBool>,
    polls_seen: Arc<AtomicUsize>,
    thread: Option<thread::JoinHandle<()>>,
}

impl PollCountingSim {
    fn start(dof: usize, init_q: Vec<f32>) -> Self {
        let sock = UdpSocket::bind("127.0.0.1:0").unwrap();
        let addr = format!("udp://{}", sock.local_addr().unwrap());
        sock.set_read_timeout(Some(Duration::from_millis(10))).unwrap();
        let stop = Arc::new(AtomicBool::new(false));
        let polls_seen = Arc::new(AtomicUsize::new(0));
        let (stop_c, polls_c) = (stop.clone(), polls_seen.clone());
        let thread = thread::spawn(move || {
            let mut buf = [0u8; 4096];
            let mut seq = 0u64;
            let mut ack = 0u64;
            let mut enabled = false;
            while !stop_c.load(Ordering::Acquire) {
                let Ok((n, peer)) = sock.recv_from(&mut buf) else { continue };
                let Ok(cmd) = serde_json::from_slice::<SimCommand>(&buf[..n]) else { continue };
                if cmd.kind == "poll" {
                    polls_c.fetch_add(1, Ordering::SeqCst);
                } else if cmd.kind == "cmd" {
                    ack = cmd.seq;
                    if let Some(en) = cmd.enable {
                        enabled = en;
                    }
                }
                seq += 1;
                let state = SimState {
                    v: 1,
                    kind: "state".into(),
                    session: 1,
                    seq,
                    ack,
                    enabled,
                    q: init_q.clone(),
                    dq: vec![0.0; dof],
                    tau: vec![0.1; dof],
                    temp: vec![30.0; dof],
                    loads: None,
                    t: seq as f64 * 0.01,
                };
                if let Ok(bytes) = serde_json::to_vec(&state) {
                    let _ = sock.send_to(&bytes, peer);
                }
            }
        });
        Self { addr, stop, polls_seen, thread: Some(thread) }
    }
}

impl Drop for PollCountingSim {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(t) = self.thread.take() { let _ = t.join(); }
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn sim_driver_rt_single_transport_writer_and_nonblocking_under_contended_rx_lock() {
    let init_q = vec![0.05, -0.02, 0.02, -0.02, 0.02, -0.02, 0.0];

    // (a) Single transport writer under RT executor (through `ZeroOffsetDriver` delegation):
    // after `start_executor`, `armd-sim-rx` sends ZERO keepalive polls across > 120 ms idle
    // while `armd-control` sends keepalive polls and keeps fresh feedback alive!
    let sim = PollCountingSim::start(7, init_q.clone());
    let drv = SimArmDriver::connect(&sim.addr, 7).unwrap();
    let rx_keepalives = drv.rx_keepalives_counter();
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;
    let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(5));
    cfg.alloc_timing = true;
    let handle = sup.start_executor(cfg).await.unwrap();

    // Wait 1 step so `set_rt_mode(true)` has completed before snapshotting counters.
    wait_until_async(Duration::from_secs(1), || handle.stats().steps.load(Ordering::Relaxed) >= 3).await;
    let rx_ka_at_handoff = rx_keepalives.load(Ordering::Acquire);
    let sim_polls_at_handoff = sim.polls_seen.load(Ordering::Acquire);

    // Stay completely idle (torque off, no commands) for 130 ms (> 6x the 20 ms keepalive period).
    tokio::time::sleep(Duration::from_millis(130)).await;

    assert_eq!(
        rx_keepalives.load(Ordering::Acquire),
        rx_ka_at_handoff,
        "armd-sim-rx must send 0 keepalive datagrams after RT ownership transfer"
    );
    let owner_polls = sim.polls_seen.load(Ordering::Acquire) - sim_polls_at_handoff;
    assert!(
        owner_polls >= 3,
        "RT owner (armd-control) must transmit keepalive polls during idle periods (saw {owner_polls})"
    );
    let obs = sup.observation().await.unwrap();
    assert!(obs.feedback_valid, "owner-driven keepalives must keep simulator feedback fresh");
    assert!((obs.positions[0] - 0.05).abs() < 1e-3);
    handle.stop_and_join();

    // (b) Non-blocking under contended RX mutex: hold the RX mutex on a helper thread while BOTH
    // direct `read_joint_states_budgeted` AND live RT executor releases continue advancing on schedule!
    let sim_b = PollCountingSim::start(7, init_q.clone());
    let mut drv_b = SimArmDriver::connect(&sim_b.addr, 7).unwrap();
    let rx_lock_b = drv_b.test_rx_lock_handle();
    drv_b.set_rt_mode(true);
    let mut states = Vec::with_capacity(7);
    let mut ages = Vec::with_capacity(7);
    let mut age_ms = Vec::with_capacity(7);
    let mut instants = Vec::with_capacity(7);
    let mut status = Vec::with_capacity(7);
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        drv_b.read_joint_states_budgeted(&mut states, Instant::now() + Duration::from_millis(5), 64).unwrap();
        if drv_b.is_feedback_valid() && (states[0].position - 0.05).abs() < 1e-3 {
            break;
        }
        assert!(Instant::now() < deadline, "sim_b feedback did not arrive");
        thread::sleep(Duration::from_millis(5));
    }

    // Hold `shared` mutex from another thread while calling `&mut self` (`read_joint_states_budgeted`)
    // AND `&self` RT methods -> MUST return immediately without blocking and do 0 alloc/free!
    let (held_tx, held_rx) = std::sync::mpsc::sync_channel::<()>(1);
    let (rel_tx, rel_rx) = std::sync::mpsc::sync_channel::<()>(1);
    let holder = thread::spawn(move || {
        let _g = rx_lock_b.lock();
        let _ = held_tx.send(());
        let _ = rel_rx.recv();
    });
    held_rx.recv_timeout(Duration::from_secs(2)).unwrap();

    armd::diagnostics::alloc::time_this_thread();
    let _ = armd::diagnostics::alloc::take();
    let t0 = Instant::now();
    drv_b.read_joint_states_budgeted(&mut states, Instant::now() + Duration::from_millis(5), 64).unwrap();
    assert_eq!(drv_b.enable_state(), EnableState::Disabled);
    assert!(drv_b.is_feedback_valid());
    assert!(drv_b.feedback_all_within(7, Duration::from_secs(1)));
    assert_eq!(drv_b.rejected_datagrams(), 0);
    drv_b.feedback_ages_into(&mut ages);
    drv_b.feedback_age_ms_into(&mut age_ms);
    drv_b.feedback_instants_into(&mut instants);
    assert!(drv_b.motor_status_into(&mut status));
    let elapsed = t0.elapsed();
    let allocs = armd::diagnostics::alloc::take();
    rel_tx.send(()).unwrap();
    holder.join().unwrap();
    assert!(elapsed < Duration::from_millis(10), "RT reads blocked behind RX lock: {elapsed:?}");
    assert_eq!(allocs.calls, 0, "contended RT reads allocated: {allocs:?}");
    assert!((states[0].position - 0.05).abs() < 1e-3);
    assert_eq!(ages.len(), 7);
    assert_eq!(status, vec![0u8; 7]);

    // (b2) Keepalive startup handoff synchronization: pause inside the RX keepalive window while
    // `set_rt_mode(true)` runs on another thread -> waits cleanly on the condvar (no priority
    // inversion spin) and returns immediately once RX releases the keepalive guard!
    let mut drv_ka = SimArmDriver::connect(&sim_b.addr, 7).unwrap();
    let ka_guard = drv_ka.test_hold_rx_keepalive();
    let (done_tx, done_rx) = std::sync::mpsc::sync_channel::<()>(1);
    let handoff_thread = thread::spawn(move || {
        drv_ka.set_rt_mode(true);
        let _ = done_tx.send(());
    });
    // While RX keepalive guard is held, `set_rt_mode(true)` MUST wait cleanly on the condvar.
    assert!(
        done_rx.recv_timeout(Duration::from_millis(25)).is_err(),
        "set_rt_mode(true) must wait until in-flight RX keepalive completes"
    );
    drop(ka_guard);
    done_rx
        .recv_timeout(Duration::from_secs(1))
        .expect("set_rt_mode(true) must wake immediately after RX keepalive finishes");
    handoff_thread.join().unwrap();

    // (c) Live RT executor releases continue advancing while the RX mutex is held continuously!
    let sim_c = PollCountingSim::start(7, init_q);
    let drv_c = SimArmDriver::connect(&sim_c.addr, 7).unwrap();
    let rx_lock_c = drv_c.test_rx_lock_handle();
    let sup_c = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_c)));
    sup_c.set_lease_enabled(false).await;
    let h_c = sup_c.start_executor(ExecutorConfig::for_tests(Duration::from_millis(5))).await.unwrap();
    wait_until_async(Duration::from_secs(2), || {
        h_c.observation().is_ok_and(|o| o.feedback_valid && (o.positions[0] - 0.05).abs() < 1e-3)
    })
    .await;
    let (h2_tx, h2_rx) = std::sync::mpsc::sync_channel::<()>(1);
    let (r2_tx, r2_rx) = std::sync::mpsc::sync_channel::<()>(1);
    let holder_c = thread::spawn(move || {
        let _g = rx_lock_c.lock();
        let _ = h2_tx.send(());
        let _ = r2_rx.recv();
    });
    h2_rx.recv_timeout(Duration::from_secs(2)).unwrap();
    let steps_before = h_c.stats().steps.load(Ordering::Relaxed);
    wait_until_async(Duration::from_secs(1), || {
        h_c.stats().steps.load(Ordering::Relaxed) >= steps_before + 8
    })
    .await;
    r2_tx.send(()).unwrap();
    holder_c.join().unwrap();
    h_c.stop_and_join();
}

// ============================================================================
// 5. Fresh Damiao enable_state Unknown, shutdown gate refusal, and panic disable
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn damiao_fresh_unknown_shutdown_gate_and_panic_disable_paths() {
    let feedback_frame = |id: u8, state: u8| {
        CanFrame::new_standard(0x10 + id as u16, &[state << 4 | id, 0x80, 0, 0x80, 0x08, 0, 30, 32]).unwrap()
    };

    // (a) Fresh Damiao driver with no feedback MUST NOT report Disabled; partial feedback is still Unknown.
    let mut fresh = DamiaoArmDriver::new(MockCanTransport::new(), vec![1, 2]);
    assert_eq!(fresh.enable_state(), EnableState::Unknown, "fresh Damiao driver without feedback must be Unknown");
    fresh.can_mut().provide_rx_frame(feedback_frame(1, 0));
    fresh.read_joint_states().unwrap();
    assert_eq!(
        fresh.enable_state(),
        EnableState::Unknown,
        "partial startup feedback must not report Disabled"
    );
    fresh.can_mut().provide_rx_frame(feedback_frame(2, 0));
    fresh.read_joint_states().unwrap();
    assert_eq!(
        fresh.enable_state(),
        EnableState::Disabled,
        "telemetry reports Disabled once all motors report fresh state=0"
    );

    // On a frozen MockClock, begin_torque(false) stays Pending across bounded 20 ms repeats.
    let frozen_clock = MockClock::new();
    let frozen_bus = FakeBus::with_clock(&dm_limits(), frozen_clock.clone());
    frozen_bus.with(|s| {
        s.immediate_replies = false;
        s.push_all_feedback();
    });
    let mut frozen_drv = DamiaoArmDriver::new(frozen_bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    frozen_drv.read_joint_states().unwrap();
    assert_eq!(
        frozen_drv.enable_state(),
        EnableState::Disabled,
        "fresh state-0 feedback from all motors reports Disabled telemetry"
    );
    let mut arm_joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
    for (i, slot) in arm_joints.iter_mut().enumerate().take(6) {
        *slot = i;
    }
    frozen_drv.set_stopped_disable_gate(armd::driver::StoppedDisableGate {
        safety_stopped: Arc::new(AtomicBool::new(false)),
        has_resting_pose: true,
        arm_joints,
        arm_joints_len: 6,
        rest_positions: [0.0; armd::rt_core::mailbox::MAX_DOF],
        rest_pose_tolerance_rad: 0.05,
    });
    for step in 0..3 {
        if step > 0 {
            frozen_clock.advance(Duration::from_millis(10));
        }
        frozen_bus.with(|s| s.push_all_feedback());
        frozen_drv.read_joint_states().unwrap();
    }
    let prog = frozen_drv.begin_torque(false).unwrap();
    assert!(matches!(prog, TorqueProgress::Pending { .. }));
    assert!(frozen_drv.torque_transition_pending());
    let prog0 = frozen_drv.poll_torque(frozen_clock.now() + Duration::from_millis(50)).unwrap();
    assert!(matches!(prog0, TorqueProgress::Pending { .. }));
    frozen_clock.advance(Duration::from_millis(20));
    let prog2 = frozen_drv.poll_torque(frozen_clock.now() + Duration::from_millis(50)).unwrap();
    assert!(matches!(prog2, TorqueProgress::Pending { .. }));
    frozen_clock.advance(Duration::from_millis(20));
    let prog3 = frozen_drv.poll_torque(frozen_clock.now() + Duration::from_millis(50)).unwrap();
    assert_eq!(prog3, TorqueProgress::Done(TorqueOutcome::Confirmed));
    assert!(!frozen_drv.torque_transition_pending());
    assert_eq!(frozen_drv.enable_state(), EnableState::Disabled);

    // (b) Shutdown gate refuses until accepted disable sets torque_intent = Off (Legacy + Executor).
    let bus_gate = FakeBus::new(&dm_limits());
    bus_gate.with(|s| {
        for m in &mut s.motors {
            m.silent = true;
        }
    });
    let drv_gate = DamiaoArmDriver::new(bus_gate.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    let sup_gate = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_gate)));
    sup_gate.set_lease_enabled(false).await;
    assert!(!sup_gate.shutdown_ready().await, "shutdown_ready must refuse fresh Damiao Unknown state");
    assert!(
        !sup_gate.request_shutdown().await.unwrap(),
        "request_shutdown must refuse Unknown state before any feedback arrives"
    );
    let h_gate = sup_gate
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .unwrap();
    wait_until_async(Duration::from_secs(1), || h_gate.stats().steps.load(Ordering::Relaxed) >= 3).await;
    assert!(
        !sup_gate.shutdown_ready().await,
        "executor shutdown_ready must continue refusing Unknown with silent motors"
    );
    assert!(
        !sup_gate.request_shutdown().await.unwrap(),
        "executor request_shutdown must refuse Unknown until disable feedback arrives"
    );
    // Once state-0 feedback arrives from every motor, shutdown gate confirms.
    bus_gate.with(|s| {
        for m in &mut s.motors {
            m.silent = false;
        }
        s.push_all_feedback();
    });
    wait_until_async(Duration::from_secs(1), || {
        h_gate.shutdown_ready().unwrap_or(false)
    })
    .await;
    assert!(sup_gate.request_shutdown().await.unwrap());
    h_gate.shutdown_and_join().unwrap();

    // (c) Executor panic path issues disable frames (CMD_DISABLE) to all 7 motors on virtual bus and confirms.
    let bus_panic = FakeBus::new(&dm_limits());
    bus_panic.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let mut drv_panic = DamiaoArmDriver::new(bus_panic.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv_panic.read_joint_states().unwrap();
    drv_panic.check_mapping_ranges();
    let sup_panic = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_panic)));
    sup_panic.set_lease_enabled(false).await;
    // Enable torque before transfer so the arm is physically energized when the executor panics.
    sup_panic.enable_torque(sup_panic.capture_admission_stamp()).await.unwrap();
    assert!(bus_panic.with(|s| s.motors.iter().all(|m| m.enabled)));
    bus_panic.with(|s| s.tx_log.clear());

    let mut cfg_panic = ExecutorConfig::for_tests(Duration::from_millis(5));
    cfg_panic.panic_policy = PanicPolicy::StopExecutor;
    let hooks_panic = PersistTestHooks { panic_on_release: Some(2), ..Default::default() };
    let h_panic = sup_panic.start_executor_with_hooks(cfg_panic, hooks_panic).await.unwrap();
    wait_until_async(Duration::from_secs(2), || h_panic.is_in_safe_hold()).await;
    assert!(sup_panic.e_stop_latched.load(Ordering::SeqCst));
    assert!(
        bus_panic.with(|s| {
            s.motors.iter().all(|m| m.enabled)
                && !s.tx_log.iter().any(|f| f.data == armd::protocol::damiao::CMD_DISABLE)
        }),
        "DRV-F1: executor panic while energized must enter SAFE-HOLD without disabling torque"
    );
    let h_panic_em = h_panic.clone();
    let outcome_panic = tokio::task::spawn_blocking(move || h_panic_em.emergency_disable_and_join())
        .await
        .unwrap();
    assert_eq!(outcome_panic, PanicDisableOutcome::Confirmed);
    let (all_disabled, disable_ids) = bus_panic.with(|s| {
        let ids: Vec<u32> = s
            .tx_log
            .iter()
            .filter(|f| f.data == armd::protocol::damiao::CMD_DISABLE)
            .map(|f| f.id)
            .collect();
        (s.motors.iter().all(|m| !m.enabled), ids)
    });
    assert!(all_disabled, "all 7 virtual motors must be disabled by ProcessExit from SAFE-HOLD");
    for motor_id in 1u32..=7 {
        assert!(
            disable_ids.contains(&motor_id),
            "missing CMD_DISABLE frame for motor {motor_id} in ProcessExit tx_log: {disable_ids:?}"
        );
    }
    h_panic.stop_and_join();

    // (d) Executor panic with completely silent motors enters SAFE-HOLD (0 disables on bus loss),
    // and deliberate ProcessExit disable stays bounded within timeout (never hangs).
    let bus_silent = FakeBus::new(&dm_limits());
    bus_silent.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let mut drv_silent = DamiaoArmDriver::new(bus_silent.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv_silent.read_joint_states().unwrap();
    drv_silent.check_mapping_ranges();
    let sup_silent = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_silent)));
    sup_silent.set_lease_enabled(false).await;
    sup_silent.enable_torque(sup_silent.capture_admission_stamp()).await.unwrap();
    bus_silent.with(|s| {
        for m in &mut s.motors {
            m.silent = true;
        }
        s.tx_log.clear();
    });
    let mut cfg_silent = ExecutorConfig::for_tests(Duration::from_millis(5));
    cfg_silent.panic_policy = PanicPolicy::StopExecutor;
    let h_silent = sup_silent
        .start_executor_with_hooks(cfg_silent, PersistTestHooks { panic_on_release: Some(2), ..Default::default() })
        .await
        .unwrap();
    wait_until_async(Duration::from_secs(2), || h_silent.is_in_safe_hold()).await;
    assert!(
        bus_silent.with(|s| !s.tx_log.iter().any(|f| f.data == armd::protocol::damiao::CMD_DISABLE)),
        "DRV-F1: bus loss in SAFE-HOLD must not issue CMD_DISABLE"
    );
    let t_silent = Instant::now();
    let h_silent_em = h_silent.clone();
    let _ = tokio::task::spawn_blocking(move || h_silent_em.emergency_disable_and_join())
        .await
        .unwrap();
    assert!(t_silent.elapsed() < Duration::from_millis(1200), "silent ProcessExit disable took {:?}", t_silent.elapsed());
    assert!(
        bus_silent.with(|s| s.tx_log.iter().any(|f| f.data == armd::protocol::damiao::CMD_DISABLE)),
        "silent ProcessExit disable path must still issue CMD_DISABLE frames"
    );
    h_silent.stop_and_join();

    // (e) Legacy panic disable helper issues disable frames on virtual bus and bounds lock contention.
    let bus_leg = FakeBus::new(&dm_limits());
    bus_leg.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let mut drv_leg = DamiaoArmDriver::new(bus_leg.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv_leg.read_joint_states().unwrap();
    drv_leg.check_mapping_ranges();
    let sup_leg = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_leg)));
    sup_leg.set_lease_enabled(false).await;
    sup_leg.enable_torque(sup_leg.capture_admission_stamp()).await.unwrap();
    bus_leg.with(|s| s.tx_log.clear());

    let sup_leg_ref = sup_leg.clone();
    sup_leg_ref.authorize_process_exit_for_test();
    let outcome = tokio::task::spawn_blocking(move || {
        sup_leg_ref.attempt_legacy_panic_disable(Duration::from_millis(250))
    })
    .await
    .unwrap();
    assert_eq!(outcome, PanicDisableOutcome::Confirmed);
    assert!(bus_leg.with(|s| s.motors.iter().all(|m| !m.enabled)));
    assert!(bus_leg.with(|s| {
        (1u32..=7).all(|id| s.tx_log.iter().any(|f| f.id == id && f.data == armd::protocol::damiao::CMD_DISABLE))
    }));

    // Held supervisor write lock -> bounded TimedOut without blocking indefinitely.
    let _hold = sup_leg.inner().write().await;
    let sup_leg_timeout = sup_leg.clone();
    let t_to = Instant::now();
    let outcome_to = tokio::task::spawn_blocking(move || {
        sup_leg_timeout.attempt_legacy_panic_disable(Duration::from_millis(60))
    })
    .await
    .unwrap();
    assert_eq!(outcome_to, PanicDisableOutcome::TimedOut);
    assert!(t_to.elapsed() < Duration::from_millis(400), "bounded timeout overran: {:?}", t_to.elapsed());
}

// ============================================================================
// 6. SimArmDriver pending-E-stop send gate & pre-publish maintenance ordering
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn sim_driver_estop_gate_and_pre_publish_maintenance_ordering() {
    // (a) Direct SimArmDriver UDP peer capturing every outbound "cmd" datagram.
    let peer = UdpSocket::bind("127.0.0.1:0").unwrap();
    peer.set_read_timeout(Some(Duration::from_millis(40))).unwrap();
    let addr = format!("udp://{}", peer.local_addr().unwrap());

    let mut drv = SimArmDriver::connect(&addr, 3).unwrap();
    let req = Arc::new(AtomicU64::new(0));
    let app = Arc::new(AtomicU64::new(0));
    let latched = Arc::new(AtomicBool::new(false));
    drv.set_estop_gate(EstopGate::new(req.clone(), app.clone()));
    drv.set_abort_flag(latched.clone());

    let drain_cmds = |sock: &UdpSocket| -> Vec<SimCommand> {
        let mut cmds = Vec::new();
        let mut buf = [0u8; 4096];
        let until = Instant::now() + Duration::from_millis(45);
        while Instant::now() < until {
            let rem = until.saturating_duration_since(Instant::now());
            let _ = sock.set_read_timeout(Some(rem.max(Duration::from_millis(1))));
            match sock.recv_from(&mut buf) {
                Ok((n, _)) => {
                    if let Ok(cmd) = serde_json::from_slice::<SimCommand>(&buf[..n]) {
                        if cmd.kind == "cmd" {
                            cmds.push(cmd);
                        }
                    }
                }
                Err(_) => break,
            }
        }
        cmds
    };
    let _ = drain_cmds(&peer);

    // Exercise the EXACT post-serialization pre-send gate inside `send_ref`: E-stop is clear
    // (`req == 0, latched == false`) before every driver entry point so all early checks pass,
    // and the hook injects E-stop strictly AFTER JSON serialization and right BEFORE the final
    // gate + `send_nonblocking`!
    let targets = [JointTarget::position(0.1), JointTarget::position(0.2), JointTarget::position(0.3)];
    let hook_hits = Arc::new(AtomicUsize::new(0));
    {
        let req_h = req.clone();
        let hits = hook_hits.clone();
        drv.set_before_send_hook(Some(Arc::new(move || {
            hits.fetch_add(1, Ordering::SeqCst);
            req_h.store(1, Ordering::SeqCst);
        })));
    }
    let err_t = drv.write_joint_targets(&targets).unwrap_err();
    assert!(is_estop_inhibited(&err_t), "expected EstopInhibited at final target send gate: {err_t:?}");
    assert_eq!(hook_hits.load(Ordering::SeqCst), 1, "must reach post-serialization send_ref boundary");

    // Reset E-stop before `set_joint_modes`; hook injects pending E-stop after serialization ->
    // 0 datagrams sent AND modes restored to Position!
    req.store(0, Ordering::SeqCst);
    let err_m = drv
        .set_joint_modes(&[JointMode::Mit, JointMode::Mit, JointMode::Mit], &targets)
        .unwrap_err();
    assert!(is_estop_inhibited(&err_m), "expected EstopInhibited at final set_joint_modes gate: {err_m:?}");
    assert_eq!(hook_hits.load(Ordering::SeqCst), 2, "must reach post-serialization send_ref boundary");
    assert!(
        drv.max_command_velocity(0).is_some(),
        "post-serialization E-stop refusal in set_joint_modes must restore joint modes"
    );

    // Reset E-stop before `begin_torque(true)`; hook injects latched E-stop after serialization ->
    // 0 datagrams sent AND no pending torque transition!
    req.store(0, Ordering::SeqCst);
    {
        let latched_h = latched.clone();
        let hits = hook_hits.clone();
        drv.set_before_send_hook(Some(Arc::new(move || {
            hits.fetch_add(1, Ordering::SeqCst);
            latched_h.store(true, Ordering::SeqCst);
        })));
    }
    let err_en = drv.begin_torque(true).unwrap_err();
    assert!(is_estop_inhibited(&err_en), "expected EstopInhibited at final enable send gate: {err_en:?}");
    assert_eq!(hook_hits.load(Ordering::SeqCst), 3, "must reach post-serialization send_ref boundary");
    assert!(!drv.torque_transition_pending(), "refused enable must not leave pending torque transition");
    assert!(
        drain_cmds(&peer).is_empty(),
        "zero UDP cmd datagrams may be transmitted when E-stop arrives after serialization"
    );

    // Requested disable (`begin_torque(false)`) without a calibrated rest-pose gate / fresh rest
    // feedback is refused by Rule A′ when E-stop arrives right before `send_nonblocking`, whereas
    // `DisableProtective` (thermal / process-exit) is NEVER inhibited even when BOTH pending and
    // latched E-stop arrive right before `send_nonblocking`!
    {
        let req_h = req.clone();
        let latched_h = latched.clone();
        let hits = hook_hits.clone();
        drv.set_before_send_hook(Some(Arc::new(move || {
            hits.fetch_add(1, Ordering::SeqCst);
            req_h.store(1, Ordering::SeqCst);
            latched_h.store(true, Ordering::SeqCst);
        })));
    }
    req.store(0, Ordering::SeqCst);
    latched.store(false, Ordering::SeqCst);
    let err_dis_req = drv.begin_torque(false).unwrap_err();
    assert!(
        matches!(err_dis_req, armd::driver::ArmDriverError::TorqueOffWouldDrop),
        "DisableRequested without rest pose must fail closed under E-stop: {err_dis_req:?}"
    );
    assert_eq!(hook_hits.load(Ordering::SeqCst), 4);
    assert!(
        drain_cmds(&peer).is_empty(),
        "zero UDP cmd datagrams may be transmitted when DisableRequested is refused"
    );
    let _ = drv
        .begin_disable_torque(armd::driver::DisableKind::DisableProtective {
            reason: armd::driver::DisableProtectiveReason::Thermal,
        })
        .unwrap();
    assert_eq!(hook_hits.load(Ordering::SeqCst), 5);
    let dis_cmds = drain_cmds(&peer);
    assert_eq!(dis_cmds.len(), 1, "protective disable cmd must still transmit under E-stop: {dis_cmds:?}");
    assert_eq!(dis_cmds[0].enable, Some(false));
    assert!(dis_cmds[0].targets.is_none());

    // Initially enabled driver against a peer that becomes disabled while E-stop stays latched:
    // a hold target packet MUST omit `enable` (preserving the peer's torque state) so it NEVER
    // re-energizes a disabled or restarted peer under latched E-stop!
    let peer2 = UdpSocket::bind("127.0.0.1:0").unwrap();
    peer2.set_read_timeout(Some(Duration::from_millis(80))).unwrap();
    let mut drv2 = SimArmDriver::connect(&format!("udp://{}", peer2.local_addr().unwrap()), 3).unwrap();
    let req2 = Arc::new(AtomicU64::new(0));
    let app2 = Arc::new(AtomicU64::new(0));
    let latched2 = Arc::new(AtomicBool::new(false));
    drv2.set_estop_gate(EstopGate::new(req2.clone(), app2.clone()));
    drv2.set_abort_flag(latched2.clone());

    // 1. Enable torque and have peer acknowledge `enabled: true`.
    let mut buf2 = [0u8; 4096];
    let _ = drv2.begin_torque(true).unwrap();
    let (en_cmd, armd_addr) = loop {
        let (n, from) = peer2.recv_from(&mut buf2).expect("peer2 recv enable cmd");
        if let Ok(cmd) = serde_json::from_slice::<SimCommand>(&buf2[..n]) {
            if cmd.kind == "cmd" {
                break (cmd, from);
            }
        }
    };
    assert_eq!(en_cmd.enable, Some(true));
    let mut peer_enabled = en_cmd.enable.unwrap_or(false);
    assert!(peer_enabled);
    let ack_state = SimState {
        v: 1,
        kind: "state".into(),
        session: 1,
        seq: 1,
        ack: en_cmd.seq,
        enabled: true,
        q: vec![0.0; 3],
        dq: vec![0.0; 3],
        tau: vec![0.0; 3],
        temp: vec![30.0; 3],
        loads: None,
        t: 0.01,
    };
    peer2.send_to(&serde_json::to_vec(&ack_state).unwrap(), armd_addr).unwrap();
    let confirm_deadline = Instant::now() + Duration::from_millis(500);
    while drv2.torque_transition_pending() {
        assert!(Instant::now() < confirm_deadline, "sim enable confirmation timed out");
        let _ = drv2.poll_torque(Instant::now() + Duration::from_millis(20)).unwrap();
        thread::sleep(Duration::from_millis(2));
    }
    assert_eq!(drv2.enable_state(), EnableState::Enabled);

    // 2. Latch and apply E-stop (`req == app == 1, latched == true`), then peer disables/restarts.
    req2.store(1, Ordering::SeqCst);
    app2.store(1, Ordering::SeqCst);
    latched2.store(true, Ordering::SeqCst);
    peer_enabled = false;
    let _ = drain_cmds(&peer2);

    // Explicit enable is still inhibited by latched E-stop.
    assert!(is_estop_inhibited(&drv2.begin_torque(true).unwrap_err()));
    assert!(drain_cmds(&peer2).is_empty(), "latched E-stop must inhibit enable send");

    // Applied E-stop hold target packet transmits targets WITHOUT `enable`, so the disabled peer
    // preserves `peer_enabled == false` and is NEVER energized by a hold packet!
    drv2.write_joint_targets(&targets).unwrap();
    let hold_cmds = drain_cmds(&peer2);
    assert_eq!(hold_cmds.len(), 1, "applied E-stop hold write must succeed: {hold_cmds:?}");
    assert!(hold_cmds[0].targets.is_some());
    assert_eq!(
        hold_cmds[0].enable,
        None,
        "target hold packet must omit `enable` so it never energizes a disabled peer"
    );
    if let Some(en) = hold_cmds[0].enable {
        peer_enabled = en;
    }
    assert!(
        !peer_enabled,
        "peer that became disabled while E-stop stayed latched must stay disabled after hold packet"
    );

    // (b) Maintenance mutating state runs BEFORE publication: when `request_shutdown()` (refused ->
    // `StopPolicy::EStopLatch` while torqued) or `clear_fault()` returns, the already-published
    // `StateHub` snapshot MUST immediately reflect the new state without waiting for another tick!
    let bus_m = FakeBus::new(&dm_limits());
    bus_m.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let mut drv_m = DamiaoArmDriver::new(bus_m.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv_m.read_joint_states().unwrap();
    drv_m.check_mapping_ranges();
    let sup_m = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_m)));
    sup_m.set_lease_enabled(false).await;
    sup_m.register_ws_connection(1).await.unwrap();
    // Use a long period (80 ms) so if maintenance had mutated state AFTER Step 6 publish,
    // immediately inspecting `hub().latest()` upon receiving the reply would observe the stale
    // pre-mutation state for up to 80 ms!
    let h_m = sup_m
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(80)))
        .await
        .unwrap();
    let req_on = submit_broker_transition(&h_m, 1, TransitionOp::TorqueOn);
    assert_eq!(wait_broker_terminal(&h_m, req_on, Duration::from_secs(2)), Outcome::Completed);

    // Refused `request_shutdown` runs `EStopLatch` BEFORE Step 6 publish -> `hub().latest()`
    // IMMEDIATELY reports `SafetyState::EStop` the instant the maintenance reply arrives!
    let seq_before_sd = sup_m.hub().published_seq();
    assert!(!sup_m.request_shutdown().await.unwrap());
    let snap_sd = sup_m.hub().latest().expect("published snapshot after refused request_shutdown");
    assert!(
        snap_sd.meta.seq > seq_before_sd,
        "refused request_shutdown must publish a fresh snapshot before replying ({} <= {})",
        snap_sd.meta.seq,
        seq_before_sd
    );
    assert_eq!(
        snap_sd.obs.safety_state,
        SafetyState::EStop,
        "StateHub snapshot must already reflect EStop when refused request_shutdown replies"
    );

    // `clear_fault` runs BEFORE Step 6 publish -> `hub().latest()` IMMEDIATELY reports `SafetyState::Idle`!
    let seq_before_cf = sup_m.hub().published_seq();
    assert_eq!(sup_m.clear_fault().await.unwrap(), SafetyState::Idle);
    let snap_cf = sup_m.hub().latest().expect("published snapshot after clear_fault");
    assert!(snap_cf.meta.seq > seq_before_cf);
    assert_eq!(
        snap_cf.obs.safety_state,
        SafetyState::Idle,
        "StateHub snapshot must already reflect Idle when clear_fault replies"
    );

    h_m.stop_and_join();
}

// ============================================================================
// 7. Detached shell lease calls refused after executor transfer
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn detached_shell_lease_calls_refuse_after_executor_transfer_without_shadowing_owner() {
    let cfg = ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() };
    let sup = Arc::new(ArmSupervisor::new(cfg));
    // Establish an active lease on the real owner BEFORE transferring ownership.
    let (owner_gen, _) = sup.preempt_lease("owner-conn", "owner-client").await.unwrap();
    let owner_epoch = sup.get_observation().await.epoch_id;
    assert!(sup.is_lease_active().await);

    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .unwrap();
    wait_until_async(Duration::from_secs(1), || handle.stats().steps.load(Ordering::Relaxed) >= 2).await;

    // Every legacy library lease call on the detached shell MUST refuse with an explicit error
    // (or `false` for boolean release/validate) and NEVER create a shadow lease on the shell!
    assert_eq!(
        sup.acquire_lease("shadow-client").await,
        Err(("OWNER_TRANSFERRED".to_string(), 0)),
        "acquire_lease on detached shell must refuse with OWNER_TRANSFERRED"
    );
    assert_eq!(
        sup.preempt_lease("shadow-conn", "shadow-client").await,
        Err(PreemptError::OwnerTransferred),
        "preempt_lease on detached shell must refuse with OwnerTransferred"
    );
    assert_eq!(
        sup.heartbeat("owner-conn", owner_epoch).await,
        Err(SafetyError::OwnerTransferred),
        "heartbeat on detached shell must refuse with OwnerTransferred"
    );
    assert!(
        !sup.release_lease("owner-conn", owner_gen).await,
        "release_lease on detached shell must return false"
    );
    assert!(
        !sup.validate_servo_lease("owner-conn", owner_gen).await,
        "validate_servo_lease on detached shell must return false"
    );

    // Direct `SupervisorInner` calls on the detached shell also refuse and keep shell empty.
    {
        let mut shell = sup.inner().write().await;
        assert_eq!(shell.acquire_lease("direct-shadow"), Err(("OWNER_TRANSFERRED".to_string(), 0)));
        assert_eq!(shell.preempt_lease("direct-shadow", "direct"), Err(PreemptError::OwnerTransferred));
        assert_eq!(shell.heartbeat("owner-conn", owner_epoch), Err(SafetyError::OwnerTransferred));
        assert!(!shell.release_lease("owner-conn", owner_gen));
        assert!(!shell.validate_servo_lease("owner-conn", owner_gen));
        assert!(shell.active_lease.is_none(), "detached shell must never acquire a shadow lease");
    }

    // Meanwhile the real RT owner still holds the original `"owner-client"` lease!
    let obs = sup.observation().await.expect("live observation from RT owner");
    assert_eq!(obs.lease_holder.as_deref(), Some("owner-client"));
    assert!(sup.is_lease_active().await);

    handle.stop_and_join();
    // Once the RT owner has exited, `is_lease_active()` must NOT keep reporting an active lease
    // from the stale final snapshot.
    assert!(
        !sup.is_lease_active().await,
        "exited executor must not report an active lease from stale final snapshot"
    );
}

// ============================================================================
// 8. Emergency teardown disable reaches driver while persistence stalls in dir_sync
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn emergency_disable_reaches_driver_while_persistence_stalled_in_dir_sync() {
    let dir = tempfile::tempdir().unwrap();
    let calib_path = dir.path().join("gripper_calib_stalled.json");

    let bus = FakeBus::new(&dm_limits());
    bus.with(|s| {
        s.immediate_replies = true;
        // Simulate jaw stop torque on the gripper (motor 7) so `start_gripper_zero_check`
        // detects stall and queues a real persistence job onto `armd-persist`.
        s.motors[6].q = 0.05;
        s.motors[6].tau = 2.0;
        s.push_all_feedback();
    });
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv.read_joint_states().unwrap();
    drv.check_mapping_ranges();

    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
    {
        let mut inner = sup.inner().write().await;
        inner.lease_enabled = false;
        { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| armd::SupervisorInner::enable_torque(__a)) }.unwrap();
        inner.gripper_calib_path = Some(calib_path.clone());
        let epoch = inner.safety.epoch_id();
        let _ = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_gripper_zero_check(__a,
                &serde_json::json!({"threshold_rad": 0.10, "store_offset": true}),
                epoch,
            )) } }
            .unwrap();
    }
    assert!(bus.with(|s| s.motors.iter().all(|m| m.enabled)));
    bus.with(|s| s.tx_log.clear());

    let (in_sync_tx, in_sync_rx) = std::sync::mpsc::sync_channel::<()>(1);
    let (release_sync_tx, release_sync_rx) = std::sync::mpsc::sync_channel::<()>(1);
    let in_sync_tx = Arc::new(std::sync::Mutex::new(Some(in_sync_tx)));
    let release_sync_rx = Arc::new(std::sync::Mutex::new(Some(release_sync_rx)));

    let hooks = PersistTestHooks {
        dir_sync_override: Some(Arc::new(move |_| {
            if let Some(tx) = in_sync_tx.lock().unwrap().take() {
                let _ = tx.send(());
            }
            if let Some(rx) = release_sync_rx.lock().unwrap().take() {
                let _ = rx.recv_timeout(Duration::from_secs(5));
            }
            Ok(())
        })),
        ..Default::default()
    };

    let handle = sup
        .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks)
        .await
        .unwrap();

    // Wait until `armd-persist` is actively blocked inside directory `sync_all()`.
    tokio::task::spawn_blocking(move || {
        in_sync_rx
            .recv_timeout(Duration::from_secs(3))
            .expect("armd-persist must enter dir_sync_override");
    })
    .await
    .unwrap();
    assert!(!handle.persist_worker_exited(), "persistence worker must still be held in dir_sync");

    // Trigger emergency teardown WHILE the dir-sync barrier is STILL HELD.
    // Disable MUST reach all 7 motors and return Confirmed WITHOUT waiting on dir_sync!
    let h_em = handle.clone();
    let outcome = tokio::task::spawn_blocking(move || h_em.emergency_disable_and_join())
        .await
        .unwrap();
    assert_eq!(
        outcome,
        PanicDisableOutcome::Confirmed,
        "emergency disable must confirm through RT owner while persistence is stalled in dir_sync"
    );
    assert!(handle.is_exited(), "RT owner must exit after bounded disable even while dir_sync is held");
    assert!(
        !handle.persist_worker_exited(),
        "test must still be holding persistence in dir_sync when disable is verified"
    );
    let (all_disabled, disable_ids) = bus.with(|s| {
        let ids: Vec<u32> = s
            .tx_log
            .iter()
            .filter(|f| f.data == armd::protocol::damiao::CMD_DISABLE)
            .map(|f| f.id)
            .collect();
        (s.motors.iter().all(|m| !m.enabled), ids)
    });
    assert!(all_disabled, "all 7 virtual motors must be physically disabled while dir_sync is held");
    for id in 1u32..=7 {
        assert!(
            disable_ids.contains(&id),
            "missing CMD_DISABLE frame for motor {id} while dir_sync is held: {disable_ids:?}"
        );
    }

    // Release the dir-sync barrier so `armd-persist` finishes cleanly before `handle` drops.
    let _ = release_sync_tx.send(());
    wait_until_async(Duration::from_secs(2), || handle.persist_worker_exited()).await;
    handle.stop_and_join();
}

// ============================================================================
// 9. Panic deadline observer bounds blocking driver calls & preserves single bus writer
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn panic_disable_deadline_observer_fires_on_blocking_driver_and_preserves_single_writer() {
    struct BlockingDisableDriver {
        enabled: bool,
        driver_tids: Arc<std::sync::Mutex<Vec<std::thread::ThreadId>>>,
        entered_disable: Arc<std::sync::Mutex<Option<std::sync::mpsc::SyncSender<()>>>>,
        release_disable: Arc<std::sync::Mutex<Option<std::sync::mpsc::Receiver<()>>>>,
    }

    impl ArmDriver for BlockingDisableDriver {
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
        fn feedback_ages(&self) -> Vec<Option<Duration>> {
            vec![Some(Duration::ZERO); 7]
        }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            self.driver_tids.lock().unwrap().push(thread::current().id());
            Ok(vec![JointState::default(); 7])
        }
        fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> {
            self.driver_tids.lock().unwrap().push(thread::current().id());
            Ok(())
        }
        fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
            self.driver_tids.lock().unwrap().push(thread::current().id());
            if enable {
                self.enabled = true;
                return Ok(());
            }
            if let Some(tx) = self.entered_disable.lock().unwrap().take() {
                let _ = tx.send(());
            }
            if let Some(rx) = self.release_disable.lock().unwrap().take() {
                let _ = rx.recv_timeout(Duration::from_secs(5));
            }
            self.enabled = false;
            Ok(())
        }
        fn supports_rt_executor(&self) -> Result<(), &'static str> {
            Ok(())
        }
    }

    // (a) RT executor panic path with a blocking `set_torque(false)` (used via default `begin_torque`):
    // independent deadline observer must fire at ~60 ms while the owner is still blocked inside
    // `set_torque(false)`, without ever touching the driver (single-bus-writer preserved).
    let driver_tids = Arc::new(std::sync::Mutex::new(Vec::new()));
    let (entered_tx, entered_rx) = std::sync::mpsc::sync_channel::<()>(1);
    let (release_tx, release_rx) = std::sync::mpsc::sync_channel::<()>(1);
    let drv = BlockingDisableDriver {
        enabled: false,
        driver_tids: driver_tids.clone(),
        entered_disable: Arc::new(std::sync::Mutex::new(Some(entered_tx))),
        release_disable: Arc::new(std::sync::Mutex::new(Some(release_rx))),
    };
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;
    sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
    driver_tids.lock().unwrap().clear();

    let (obs_tx, obs_rx) = std::sync::mpsc::sync_channel::<std::thread::ThreadId>(1);
    let obs_tx = Arc::new(std::sync::Mutex::new(Some(obs_tx)));
    let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(5));
    cfg.panic_policy = PanicPolicy::StopExecutor;
    let hooks = PersistTestHooks {
        panic_on_release: Some(2),
        panic_disable_timeout: Some(Duration::from_millis(60)),
        on_panic_disable_timeout: Some(Arc::new(move || {
            if let Some(tx) = obs_tx.lock().unwrap().take() {
                let _ = tx.send(thread::current().id());
            }
        })),
        ..Default::default()
    };
    let handle = sup.start_executor_with_hooks(cfg, hooks).await.unwrap();
    let exec_tid = handle.executor_thread_id();
    wait_until_async(Duration::from_secs(2), || handle.is_in_safe_hold()).await;
    assert!(
        entered_rx.try_recv().is_err(),
        "DRV-F1: panic must enter SAFE-HOLD without calling set_torque(false)"
    );
    let t_start = Instant::now();
    let h_em = handle.clone();
    let em_task = tokio::task::spawn_blocking(move || h_em.emergency_disable_and_join());

    tokio::task::spawn_blocking(move || {
        entered_rx
            .recv_timeout(Duration::from_secs(2))
            .expect("owner must enter blocking set_torque(false) on ProcessExit");
    })
    .await
    .unwrap();

    // While `release_tx` is STILL NOT SENT (driver call is still blocked!), the independent
    // deadline observer must fire within bounded time and mark the executor exited.
    let observer_tid = tokio::task::spawn_blocking(move || {
        obs_rx
            .recv_timeout(Duration::from_millis(600))
            .expect("independent deadline observer must fire while blocking driver stalls")
    })
    .await
    .unwrap();
    assert!(
        t_start.elapsed() < Duration::from_millis(800),
        "panic deadline observer took {:?}",
        t_start.elapsed()
    );
    assert_ne!(observer_tid, exec_tid, "deadline observer must be an independent thread");
    assert!(handle.is_exited(), "deadline observer must mark executor exited on timeout");
    let outcome = em_task.await.unwrap();
    assert_eq!(outcome, PanicDisableOutcome::TimedOut);
    {
        let tids = driver_tids.lock().unwrap();
        assert!(!tids.is_empty());
        assert!(
            tids.iter().all(|&tid| tid == exec_tid),
            "only the RT owner thread may touch the driver (single-bus-writer): {tids:?} vs {exec_tid:?}"
        );
    }

    // Release the blocked driver call so the thread finishes cleanly before Drop joins it.
    let _ = release_tx.send(());
    handle.stop_and_join();

    // (b) `emergency_disable_and_join()` against a blocking driver also returns `TimedOut`
    // in bounded time while the driver call is still blocked.
    let driver_tids2 = Arc::new(std::sync::Mutex::new(Vec::new()));
    let (entered_tx2, entered_rx2) = std::sync::mpsc::sync_channel::<()>(1);
    let (release_tx2, release_rx2) = std::sync::mpsc::sync_channel::<()>(1);
    let drv2 = BlockingDisableDriver {
        enabled: false,
        driver_tids: driver_tids2.clone(),
        entered_disable: Arc::new(std::sync::Mutex::new(Some(entered_tx2))),
        release_disable: Arc::new(std::sync::Mutex::new(Some(release_rx2))),
    };
    let sup2 = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv2)));
    sup2.set_lease_enabled(false).await;
    sup2.enable_torque(sup2.capture_admission_stamp()).await.unwrap();
    driver_tids2.lock().unwrap().clear();
    let hooks2 = PersistTestHooks {
        panic_disable_timeout: Some(Duration::from_millis(60)),
        ..Default::default()
    };
    let handle2 = sup2
        .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks2)
        .await
        .unwrap();
    let exec_tid2 = handle2.executor_thread_id();
    let h2 = handle2.clone();
    let t_em = Instant::now();
    let outcome2 = tokio::task::spawn_blocking(move || h2.emergency_disable_and_join())
        .await
        .unwrap();
    let _ = entered_rx2.recv_timeout(Duration::from_millis(200));
    assert_eq!(
        outcome2,
        PanicDisableOutcome::TimedOut,
        "emergency_disable_and_join must return TimedOut when driver blocks past bound"
    );
    assert!(t_em.elapsed() < Duration::from_millis(600), "emergency disable took {:?}", t_em.elapsed());
    assert!(
        driver_tids2.lock().unwrap().iter().all(|&tid| tid == exec_tid2),
        "single-bus-writer must hold during emergency teardown"
    );
    let _ = release_tx2.send(());
    handle2.stop_and_join();

    // (c) Legacy `attempt_legacy_panic_disable` against a blocking driver also bounds to TimedOut.
    let (entered_tx3, entered_rx3) = std::sync::mpsc::sync_channel::<()>(1);
    let (release_tx3, release_rx3) = std::sync::mpsc::sync_channel::<()>(1);
    let drv3 = BlockingDisableDriver {
        enabled: false,
        driver_tids: Arc::new(std::sync::Mutex::new(Vec::new())),
        entered_disable: Arc::new(std::sync::Mutex::new(Some(entered_tx3))),
        release_disable: Arc::new(std::sync::Mutex::new(Some(release_rx3))),
    };
    let sup3 = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv3)));
    sup3.set_lease_enabled(false).await;
    sup3.enable_torque(sup3.capture_admission_stamp()).await.unwrap();
    let sup3_c = sup3.clone();
    sup3_c.authorize_process_exit_for_test();
    let t_leg = Instant::now();
    let outcome3 = tokio::task::spawn_blocking(move || {
        sup3_c.attempt_legacy_panic_disable(Duration::from_millis(60))
    })
    .await
    .unwrap();
    let _ = entered_rx3.recv_timeout(Duration::from_millis(200));
    assert_eq!(outcome3, PanicDisableOutcome::TimedOut);
    assert!(t_leg.elapsed() < Duration::from_millis(500), "legacy panic disable took {:?}", t_leg.elapsed());
    let _ = release_tx3.send(());
}

// ============================================================================
// 10. Unified Damiao fresh feedback boundary (virtual clock) & Step 2 thermal protection
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn damiao_unified_fresh_boundary_and_stale_shutdown_gate_virtual_clock() {
    // (a) Frozen virtual clock + MIT fence: state-0 feedback is telemetry only and never
    // short-circuits bounded disable repeats or releases the MIT fence early; the fence
    // releases only once all `DAMIAO_DISABLE_REPEATS` spaced repeats finish.
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&dm_limits(), clock.clone());
    bus.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    let mut arm_joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
    for (i, slot) in arm_joints.iter_mut().enumerate().take(6) {
        *slot = i;
    }
    drv.set_stopped_disable_gate(armd::driver::StoppedDisableGate {
        safety_stopped: Arc::new(AtomicBool::new(false)),
        has_resting_pose: true,
        arm_joints,
        arm_joints_len: 6,
        rest_positions: [0.0; armd::rt_core::mailbox::MAX_DOF],
        rest_pose_tolerance_rad: 0.05,
    });
    drv.read_joint_states().unwrap();
    drv.set_torque(true).unwrap();
    assert_eq!(drv.enable_state(), EnableState::Enabled);
    drv.set_mit_deadline_fence(Some(Duration::from_millis(500)), Duration::from_millis(200));

    // Inject state-0 feedback before disable: telemetry updates to Disabled, but the MIT fence
    // stays armed and bounded disable repeats must still execute.
    bus.with(|s| {
        s.immediate_replies = false;
        for m in &mut s.motors {
            m.enabled = false;
        }
        s.push_all_feedback();
    });
    drv.read_joint_states().unwrap();
    for _ in 0..2 {
        clock.advance(Duration::from_millis(10));
        bus.with(|s| s.push_all_feedback());
        drv.read_joint_states().unwrap();
    }
    assert_eq!(drv.enable_state(), EnableState::Disabled);

    let p0 = drv.begin_torque(false).unwrap();
    assert!(matches!(p0, TorqueProgress::Pending { .. }));

    // Polling at the SAME frozen timestamp (before the 20 ms repeat interval) MUST stay Pending
    // and MUST keep the MIT fence armed!
    let p1 = drv.poll_torque(clock.now() + Duration::from_millis(50)).unwrap();
    assert!(
        matches!(p1, TorqueProgress::Pending { .. }),
        "disable repeats must stay Pending until spaced repeats finish: {p1:?}"
    );
    assert!(drv.mit_fence_trip().is_none(), "fence must stay armed, not tripped yet");
    let hold = vec![JointTarget::position(0.0); 7];
    let mode_err = drv
        .set_joint_modes(&[JointMode::Position; 7], &hold)
        .unwrap_err();
    assert!(
        mode_err.to_string().contains("MIT tracking holds the deadline fence"),
        "MIT fence must stay armed while disable repeats are pending: {mode_err}"
    );

    // Repeat #2 at +20 ms: still Pending and MIT fence still armed.
    clock.advance(Duration::from_millis(20));
    let p2 = drv.poll_torque(clock.now() + Duration::from_millis(50)).unwrap();
    assert!(matches!(p2, TorqueProgress::Pending { .. }));
    assert!(drv.set_joint_modes(&[JointMode::Position; 7], &hold).is_err());

    // Repeat #3 at +20 ms: completes bounded repeats and releases the MIT fence!
    clock.advance(Duration::from_millis(20));
    let p_ok = drv.poll_torque(clock.now() + Duration::from_millis(50)).unwrap();
    assert_eq!(p_ok, TorqueProgress::Done(TorqueOutcome::Confirmed));
    drv.set_joint_modes(&[JointMode::Position; 7], &hold)
        .expect("completed bounded disable repeats must release MIT fence");

    // (b) Startup unsolicited 0x0 feedback is telemetry only: owner `torque_intent` defaults to
    // `On`, so `shutdown_ready()` refuses until an explicit disable is accepted by the owner.
    let clock2 = MockClock::new();
    let bus2 = FakeBus::with_clock(&dm_limits(), clock2.clone());
    bus2.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let mut drv2 = DamiaoArmDriver::new(bus2.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv2.read_joint_states().unwrap();
    assert_eq!(
        drv2.enable_state(),
        EnableState::Disabled,
        "telemetry reports motor state 0 as Disabled"
    );
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv2)));
    sup.set_lease_enabled(false).await;
    assert!(
        !sup.shutdown_ready().await,
        "unsolicited startup 0x0 without accepted disable must refuse shutdown_ready"
    );
    // Explicit accepted disable via the supervisor goes through the K=3 rest-pose gate (R12 B5)
    // and records `torque_intent = Off` -> `shutdown_ready`.
    for _ in 0..3 {
        clock2.advance(Duration::from_millis(15));
        bus2.with(|s| s.push_all_feedback());
        sup.tick(0.02).await;
    }
    sup.disable_torque().await.unwrap();
    assert!(sup.shutdown_ready().await);
    assert!(sup.request_shutdown().await.unwrap());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn step2_simultaneous_disable_confirm_and_overtemp_latches_thermal_trip() {
    // Driver that reports disabled (`EnableState::Disabled`) AND over-temperature on the very
    // same Step 2 poll while `disable_pending == true` (simulating a hot motor whose disable
    // confirmation arrives together with the over-temperature feedback sample).
    struct HotDisableConfirmDriver {
        temp_c: Arc<std::sync::Mutex<f32>>,
        state: Arc<std::sync::Mutex<EnableState>>,
        disables_sent: Arc<AtomicUsize>,
    }

    impl ArmDriver for HotDisableConfirmDriver {
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
        fn enable_state(&self) -> EnableState {
            *self.state.lock().unwrap()
        }
        fn is_enabled(&self) -> bool {
            *self.state.lock().unwrap() == EnableState::Enabled
        }
        fn is_feedback_valid(&self) -> bool {
            true
        }
        fn feedback_ages(&self) -> Vec<Option<Duration>> {
            vec![Some(Duration::ZERO); 7]
        }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            let t = *self.temp_c.lock().unwrap();
            Ok((0..7)
                .map(|i| JointState {
                    position: 0.0,
                    velocity: 0.0,
                    torque: 0.0,
                    temperature: if i == 2 { t } else { 30.0 },
                })
                .collect())
        }
        fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> {
            Ok(())
        }
        fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
            if !enable {
                self.disables_sent.fetch_add(1, Ordering::SeqCst);
                *self.state.lock().unwrap() = EnableState::Disabled;
            } else {
                *self.state.lock().unwrap() = EnableState::Enabled;
            }
            Ok(())
        }
        fn supports_rt_executor(&self) -> Result<(), &'static str> {
            Ok(())
        }
    }

    let temp_c = Arc::new(std::sync::Mutex::new(35.0f32));
    let state = Arc::new(std::sync::Mutex::new(EnableState::Unknown));
    let disables_sent = Arc::new(AtomicUsize::new(0));
    let drv = HotDisableConfirmDriver {
        temp_c: temp_c.clone(),
        state: state.clone(),
        disables_sent: disables_sent.clone(),
    };
    let mut cfg = ArmConfig::rebot_dm();
    cfg.max_temperature = 70.0;
    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
    {
        let mut inner = sup.inner().write().await;
        inner.lease_enabled = false;
        // Background disable in flight (`disable_pending == true`, `is_torqued == false`),
        // not yet faulted with ThermalTrip.
        inner.is_torqued = false;
        inner.disable_pending = true;
    }

    // Before Step 2 polls: deliver BOTH confirmed disable (`Disabled`) AND over-temperature (92°C)
    // in the exact same feedback poll.
    *state.lock().unwrap() = EnableState::Disabled;
    *temp_c.lock().unwrap() = 92.0;

    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .unwrap();
    wait_until_async(Duration::from_secs(1), || handle.stats().steps.load(Ordering::Relaxed) >= 2).await;

    let obs = sup.observation().await.unwrap();
    assert!(
        matches!(obs.safety_state, SafetyState::Fault(FaultReason::ThermalTrip { joint_idx: 2, .. })),
        "simultaneous disable confirmation and over-temp in Step 2 must latch ThermalTrip, got {:?}",
        obs.safety_state
    );
    assert!(disables_sent.load(Ordering::SeqCst) >= 1);
    // And Step 2b `clear_fault()` must refuse while temperature stays above reset threshold.
    assert!(sup.clear_fault().await.is_err());

    handle.stop_and_join();
}

// ============================================================================
// 11. E-stop acknowledgement waits for Step 6 StateHub publication barrier
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn estop_ack_waits_for_step6_publication_barrier() {
    let bus = FakeBus::new(&dm_limits());
    bus.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv.read_joint_states().unwrap();
    drv.check_mapping_ranges();

    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;
    sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();

    // Arm the publication barrier once `trigger_e_stop_checked()` has bumped `requested_estop_count`
    // and Step 1 has applied E-stop: pause right BEFORE Step 6 `core.publish(...)`!
    let armed = Arc::new(AtomicBool::new(false));
    let (at_pub_tx, at_pub_rx) = std::sync::mpsc::sync_channel::<()>(1);
    let (release_pub_tx, release_pub_rx) = std::sync::mpsc::sync_channel::<()>(1);
    let armed_h = armed.clone();
    let estop_latched_ref = sup.e_stop_latched.clone();
    let at_pub_tx = Arc::new(std::sync::Mutex::new(Some(at_pub_tx)));
    let release_pub_rx = Arc::new(std::sync::Mutex::new(Some(release_pub_rx)));

    let hooks = PersistTestHooks {
        before_publish: Some(Arc::new(move || {
            if armed_h.load(Ordering::Acquire) && estop_latched_ref.load(Ordering::Acquire) {
                if let Some(tx) = at_pub_tx.lock().unwrap().take() {
                    let _ = tx.send(());
                    if let Some(rx) = release_pub_rx.lock().unwrap().take() {
                        let _ = rx.recv_timeout(Duration::from_secs(5));
                    }
                }
            }
        })),
        ..Default::default()
    };

    let handle = sup
        .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks)
        .await
        .unwrap();
    wait_until_async(Duration::from_secs(1), || handle.stats().steps.load(Ordering::Relaxed) >= 3).await;

    let snap_before = sup.hub().latest().expect("operational snapshot before E-stop");
    assert_eq!(snap_before.obs.safety_state, SafetyState::Armed);
    let epoch_before = snap_before.obs.epoch_id;

    // Arm barrier right before triggering E-stop.
    armed.store(true, Ordering::Release);
    let sup_es = sup.clone();
    let estop_task = tokio::spawn(async move { sup_es.trigger_e_stop_checked().await });

    // Wait until `armd-control` has applied E-stop in Step 1 and is now paused strictly BEFORE Step 6 publish!
    tokio::task::spawn_blocking(move || {
        at_pub_rx
            .recv_timeout(Duration::from_secs(2))
            .expect("armd-control must pause at Step 6 before_publish barrier");
    })
    .await
    .unwrap();

    // While paused BEFORE Step 6 publication:
    // 1. StateHub MUST still expose the previous operational Armed snapshot.
    // 2. `trigger_e_stop_checked()` MUST NOT have received its acknowledgement yet!
    let snap_paused = sup.hub().latest().expect("snapshot while paused before publish");
    assert_eq!(snap_paused.obs.safety_state, SafetyState::Armed);
    assert_eq!(snap_paused.obs.epoch_id, epoch_before);
    tokio::time::sleep(Duration::from_millis(35)).await;
    assert!(
        !estop_task.is_finished(),
        "E-stop acknowledgement must NOT be delivered before Step 6 StateHub publication"
    );

    // Release Step 6 publication -> E-stop ack now resolves, and `StateHub` MUST ALREADY expose
    // the stopped snapshot and new epoch at the exact moment the ack returns!
    let _ = release_pub_tx.send(());
    let (ack_state, ack_epoch) = estop_task.await.unwrap().expect("E-stop ack");
    assert_eq!(ack_state, SafetyState::EStop);
    assert!(ack_epoch > epoch_before);

    let snap_after = sup.hub().latest().expect("snapshot immediately after E-stop ack");
    assert_eq!(
        snap_after.obs.safety_state,
        SafetyState::EStop,
        "StateHub must already expose SafetyState::EStop when E-stop ack resolves"
    );
    assert_eq!(
        snap_after.obs.epoch_id,
        ack_epoch,
        "StateHub epoch must already match acknowledged E-stop epoch"
    );

    handle.stop_and_join();
}

// ============================================================================
// 12. Panic disable retries transient per-motor CAN TX failure within budget
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn panic_disable_retries_transient_j3_send_failure_and_disables_all_motors() {
    /// Virtual CAN bus wrapper that fails the first `fail_j3_remaining` disable transmits to J3
    /// (CAN ID 3) while forwarding every other motor normally.
    #[derive(Clone)]
    struct FailJ3DisableBus {
        inner: FakeBus,
        fail_j3_remaining: Arc<AtomicUsize>,
        j3_failures: Arc<AtomicUsize>,
        j3_successes: Arc<AtomicUsize>,
    }

    impl CanTransport for FailJ3DisableBus {
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
            if frame.id == 3 && frame.data == armd::protocol::damiao::CMD_DISABLE {
                let prev = self.fail_j3_remaining.fetch_update(Ordering::SeqCst, Ordering::SeqCst, |v| {
                    v.checked_sub(1)
                });
                if prev.is_ok() {
                    self.j3_failures.fetch_add(1, Ordering::SeqCst);
                    return Err(TransportError::Io(std::io::Error::new(
                        std::io::ErrorKind::WouldBlock,
                        "transient CAN TX failure on J3",
                    )));
                }
                self.j3_successes.fetch_add(1, Ordering::SeqCst);
            }
            self.inner.transmit(frame)
        }
        fn receive(&mut self) -> Result<CanFrame, TransportError> {
            self.inner.receive()
        }
        fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
            self.inner.receive_timestamped()
        }
        fn flush(&mut self) -> Result<(), TransportError> {
            self.inner.flush()
        }
        fn supports_nonblocking_rt(&self) -> bool {
            self.inner.supports_nonblocking_rt()
        }
        fn supports_bounded_flush(&self) -> bool {
            self.inner.supports_bounded_flush()
        }
        fn clock(&self) -> Option<Arc<dyn DriverClock>> {
            self.inner.clock()
        }
    }

    // (a) Executor panic path: first disable TX to J3 fails transiently; all other motors receive
    // theirs on the first attempt and J3 is retried and disabled before the deadline.
    let inner_bus = FakeBus::new(&dm_limits());
    inner_bus.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let bus = FailJ3DisableBus {
        inner: inner_bus.clone(),
        fail_j3_remaining: Arc::new(AtomicUsize::new(1)),
        j3_failures: Arc::new(AtomicUsize::new(0)),
        j3_successes: Arc::new(AtomicUsize::new(0)),
    };
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv.read_joint_states().unwrap();
    drv.check_mapping_ranges();
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;
    sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
    assert!(inner_bus.with(|s| s.motors.iter().all(|m| m.enabled)));
    inner_bus.with(|s| s.tx_log.clear());

    let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(5));
    cfg.panic_policy = PanicPolicy::StopExecutor;
    let handle = sup
        .start_executor_with_hooks(cfg, PersistTestHooks { panic_on_release: Some(2), ..Default::default() })
        .await
        .unwrap();
    wait_until_async(Duration::from_secs(2), || handle.is_in_safe_hold()).await;
    assert_eq!(bus.j3_failures.load(Ordering::SeqCst), 0, "panic alone must hold without disabling");
    let h_em = handle.clone();
    let outcome = tokio::task::spawn_blocking(move || h_em.emergency_disable_and_join())
        .await
        .unwrap();
    assert_eq!(outcome, PanicDisableOutcome::Confirmed);

    assert_eq!(bus.j3_failures.load(Ordering::SeqCst), 1, "first J3 disable send must have failed");
    assert!(bus.j3_successes.load(Ordering::SeqCst) >= 1, "J3 disable must be retried and succeed");
    let (all_disabled, disable_ids) = inner_bus.with(|s| {
        let ids: Vec<u32> = s
            .tx_log
            .iter()
            .filter(|f| f.data == armd::protocol::damiao::CMD_DISABLE)
            .map(|f| f.id)
            .collect();
        (s.motors.iter().all(|m| !m.enabled), ids)
    });
    assert!(all_disabled, "all 7 motors including J3 must be physically disabled after panic retry");
    for id in 1u32..=7 {
        assert!(
            disable_ids.contains(&id),
            "missing CMD_DISABLE frame for motor J{id}: {disable_ids:?}"
        );
    }
    handle.stop_and_join();

    // (b) Legacy panic disable path: first disable TX to J3 fails transiently; still retries J3
    // and disables every motor with `PanicDisableOutcome::Confirmed`.
    let inner_leg = FakeBus::new(&dm_limits());
    inner_leg.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let bus_leg = FailJ3DisableBus {
        inner: inner_leg.clone(),
        fail_j3_remaining: Arc::new(AtomicUsize::new(1)),
        j3_failures: Arc::new(AtomicUsize::new(0)),
        j3_successes: Arc::new(AtomicUsize::new(0)),
    };
    let mut drv_leg = DamiaoArmDriver::new(bus_leg.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv_leg.read_joint_states().unwrap();
    drv_leg.check_mapping_ranges();
    let sup_leg = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_leg)));
    sup_leg.set_lease_enabled(false).await;
    sup_leg.enable_torque(sup_leg.capture_admission_stamp()).await.unwrap();
    inner_leg.with(|s| s.tx_log.clear());

    let sup_leg_c = sup_leg.clone();
    sup_leg_c.authorize_process_exit_for_test();
    let outcome = tokio::task::spawn_blocking(move || {
        sup_leg_c.attempt_legacy_panic_disable(Duration::from_millis(300))
    })
    .await
    .unwrap();
    assert_eq!(outcome, PanicDisableOutcome::Confirmed);
    assert_eq!(bus_leg.j3_failures.load(Ordering::SeqCst), 1);
    assert!(bus_leg.j3_successes.load(Ordering::SeqCst) >= 1);
    assert!(inner_leg.with(|s| s.motors.iter().all(|m| !m.enabled)));

    // (c) Persistent J3 TX failure: all other motors (J1,J2,J4..J7) still receive disable and
    // disable cleanly, J3 is retried across the budget until the deadline is spent, and the
    // outcome is `Unconfirmed` (never exiting early on the first error).
    let inner_perm = FakeBus::new(&dm_limits());
    inner_perm.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let bus_perm = FailJ3DisableBus {
        inner: inner_perm.clone(),
        fail_j3_remaining: Arc::new(AtomicUsize::new(usize::MAX)),
        j3_failures: Arc::new(AtomicUsize::new(0)),
        j3_successes: Arc::new(AtomicUsize::new(0)),
    };
    let mut drv_perm = DamiaoArmDriver::new(bus_perm.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv_perm.read_joint_states().unwrap();
    drv_perm.check_mapping_ranges();
    let sup_perm = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_perm)));
    sup_perm.set_lease_enabled(false).await;
    sup_perm.enable_torque(sup_perm.capture_admission_stamp()).await.unwrap();
    inner_perm.with(|s| s.tx_log.clear());

    let sup_perm_c = sup_perm.clone();
    sup_perm_c.authorize_process_exit_for_test();
    let t_perm = Instant::now();
    let out_perm = tokio::task::spawn_blocking(move || {
        sup_perm_c.attempt_legacy_panic_disable(Duration::from_millis(80))
    })
    .await
    .unwrap();
    assert_eq!(out_perm, PanicDisableOutcome::Unconfirmed);
    assert!(
        t_perm.elapsed() >= Duration::from_millis(70),
        "panic disable must keep retrying unconfirmed J3 until budget is spent (only took {:?})",
        t_perm.elapsed()
    );
    assert!(
        bus_perm.j3_failures.load(Ordering::SeqCst) >= 2,
        "J3 must be retried multiple times within budget"
    );
    inner_perm.with(|s| {
        for (i, m) in s.motors.iter().enumerate() {
            if i != 2 {
                assert!(!m.enabled, "motor J{} must still be disabled despite J3 TX failures", i + 1);
            }
        }
    });
}

// ============================================================================
// 13. Sticky confirmed disable after MIT fence trip & repeated Unknown shutdown disable
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn mit_fence_trip_confirmed_disable_stays_shutdown_ready_and_repeated_unknown_shutdown_disables() {
    // (a) MIT fence trip (unacknowledged -> `poll_tx_allowed() == false`) -> disable confirmed ->
    // 1 s with ZERO polls -> `enable_state()` stays sticky `Disabled` and `shutdown_ready()` is true!
    // Output MIT trip/ack logic remains intact (targets still refused until explicit recovery).
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&dm_limits(), clock.clone());
    bus.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv.read_joint_states().unwrap();
    drv.check_mapping_ranges();
    drv.set_torque(true).unwrap();
    assert_eq!(drv.enable_state(), EnableState::Enabled);

    // Arm MIT fence, advance past the 45 ms deadline, and trigger a fence trip (unacknowledged!).
    drv.set_mit_deadline_fence(Some(Duration::from_millis(45)), Duration::from_millis(200));
    clock.advance(Duration::from_millis(50));
    let _ = drv.read_joint_states();
    assert!(drv.mit_fence_trip().is_some(), "MIT deadline fence must be tripped");

    // Disable torque via supervisor while leaving the MIT fence trip UNACKNOWLEDGED
    // (`poll_tx_allowed()` stays false): satisfy K=3 rest-pose gate (R12 B5) with arriving
    // feedback across 3 releases, then disable records `torque_intent = Off`.
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;
    for _ in 0..3 {
        clock.advance(Duration::from_millis(15));
        bus.with(|s| s.push_all_feedback());
        sup.tick(0.02).await;
    }
    sup.disable_torque().await.unwrap();
    assert!(sup.shutdown_ready().await);

    // Advance virtual clock by 1 full second with ZERO polls (and verify `read_joint_states()`
    // transmits 0 refresh polls because the MIT fence trip is still unacknowledged!).
    bus.with(|s| {
        s.immediate_replies = false;
        s.tx_log.clear();
    });
    clock.advance(Duration::from_secs(1));
    {
        let mut inner = sup.inner().write().await;
        let mut drv = inner.driver.as_mut().unwrap().write().unwrap();
        drv.read_joint_states().unwrap();
        assert!(!drv.is_feedback_valid(), "1 s without polls must make feedback ages stale (>250 ms)");
        assert!(drv.mit_fence_trip().is_some(), "MIT fence trip for output must stay active");
        assert!(
            drv.write_joint_targets(&vec![JointTarget::position(0.0); 7]).is_err(),
            "output must still be refused while MIT fence trip is active"
        );
        assert_eq!(
            drv.enable_state(),
            EnableState::Unknown,
            "telemetry enable_state must report Unknown after 1 s without polls"
        );
        drop(drv);
        assert_eq!(
            inner.torque_intent(),
            armd::TorqueIntent::Off,
            "accepted disable decision torque_intent must remain Off after 1 s without polls"
        );
    }
    assert!(
        bus.with(|s| s.tx_log.is_empty()),
        "unacknowledged MIT fence trip must not transmit refresh polls"
    );
    assert!(
        sup.shutdown_ready().await,
        "shutdown_ready must remain true 1 s after accepted disable following an MIT fence trip"
    );
    assert!(
        sup.request_shutdown().await.unwrap(),
        "request_shutdown must succeed by decision (torque_intent == Off)"
    );

    // Verify a later enable request sets `torque_intent = On` and revokes shutdown readiness.
    {
        let inner = sup.inner().write().await;
        inner.e_stop_latched.store(false, Ordering::SeqCst);
    }
    let _ = sup.enable_torque(sup.capture_admission_stamp()).await;
    assert_eq!(
        sup.inner().read().await.torque_intent(),
        armd::TorqueIntent::On,
        "starting a later enable must set torque_intent = On"
    );
    assert!(!sup.shutdown_ready().await);

    // (b) Repeated shutdown signal when state is Unknown issues a new bounded disable request
    // (Legacy path): fresh Damiao driver starts with no feedback (`Unknown`) and `immediate_replies`
    // enabled so motors only answer explicit commands.
    let bus_rep = FakeBus::new(&dm_limits());
    bus_rep.with(|s| {
        s.immediate_replies = true;
        s.tx_log.clear();
    });
    let drv_rep = DamiaoArmDriver::new(bus_rep.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    let sup_rep = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_rep)));
    sup_rep.set_lease_enabled(false).await;
    assert!(!sup_rep.shutdown_ready().await);

    // 1st shutdown signal: refused because state is Unknown; latches EStop.
    assert!(!sup_rep.request_shutdown().await.unwrap());
    assert!(
        !bus_rep.with(|s| s.tx_log.iter().any(|f| f.data == armd::protocol::damiao::CMD_DISABLE)),
        "first refused shutdown signal only latches EStop"
    );
    // Repeated shutdown signal while still Unknown: MUST issue a bounded disable request!
    assert!(
        sup_rep.request_shutdown().await.unwrap(),
        "repeated shutdown signal while Unknown must send bounded disable and confirm"
    );
    for id in 1u32..=7 {
        assert!(
            bus_rep.with(|s| s.tx_log.iter().any(|f| f.id == id && f.data == armd::protocol::damiao::CMD_DISABLE)),
            "repeated Unknown shutdown must transmit CMD_DISABLE to motor J{id}"
        );
    }
    assert!(sup_rep.shutdown_ready().await);

    // (c) Repeated shutdown signal while Unknown on the RT executor path also issues a bounded
    // non-blocking disable request rather than only re-checking.
    let bus_exec = FakeBus::new(&dm_limits());
    bus_exec.with(|s| {
        s.immediate_replies = true;
        for m in &mut s.motors {
            m.silent = true;
        }
    });
    let drv_exec = DamiaoArmDriver::new(bus_exec.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    let sup_exec = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_exec)));
    sup_exec.set_lease_enabled(false).await;
    let h_exec = sup_exec
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .unwrap();
    wait_until_async(Duration::from_secs(1), || h_exec.stats().steps.load(Ordering::Relaxed) >= 2).await;
    assert!(!sup_exec.request_shutdown().await.unwrap());
    bus_exec.with(|s| {
        s.tx_log.clear();
        for m in &mut s.motors {
            m.silent = false;
        }
    });
    // Repeated shutdown signal while Unknown: issues bounded disable frames (`CMD_DISABLE`).
    let _ = sup_exec.request_shutdown().await.unwrap();
    for id in 1u32..=7 {
        assert!(
            bus_exec.with(|s| s.tx_log.iter().any(|f| f.id == id && f.data == armd::protocol::damiao::CMD_DISABLE)),
            "executor repeated Unknown shutdown must transmit CMD_DISABLE to motor J{id}"
        );
    }
    wait_until_async(Duration::from_secs(1), || h_exec.shutdown_ready().unwrap_or(false)).await;
    assert!(sup_exec.request_shutdown().await.unwrap());
    h_exec.shutdown_and_join().unwrap();
}

// ============================================================================
// 14. Emergency / panic timeout detaches stalled thread so final Drop is bounded
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn emergency_timeout_detaches_stalled_control_thread_so_drop_is_bounded() {
    struct StalledDisableDriver {
        enabled: bool,
        entered: Arc<std::sync::Mutex<Option<std::sync::mpsc::SyncSender<()>>>>,
        release: Arc<std::sync::Mutex<Option<std::sync::mpsc::Receiver<()>>>>,
    }

    impl ArmDriver for StalledDisableDriver {
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
        fn feedback_ages(&self) -> Vec<Option<Duration>> {
            vec![Some(Duration::ZERO); 7]
        }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            Ok(vec![JointState::default(); 7])
        }
        fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> {
            Ok(())
        }
        fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
            if enable {
                self.enabled = true;
                return Ok(());
            }
            if let Some(tx) = self.entered.lock().unwrap().take() {
                let _ = tx.send(());
            }
            if let Some(rx) = self.release.lock().unwrap().take() {
                let _ = rx.recv_timeout(Duration::from_secs(10));
            }
            self.enabled = false;
            Ok(())
        }
        fn supports_rt_executor(&self) -> Result<(), &'static str> {
            Ok(())
        }
    }

    // (a) Emergency teardown timeout: driver blocks past the 60 ms deadline -> emergency teardown
    // returns `TimedOut` within its bound -> dropping the LAST handle (and supervisor) while the
    // driver call is STILL BLOCKED returns within a bound (detached, never an unbounded join!).
    let (entered_tx, entered_rx) = std::sync::mpsc::sync_channel::<()>(1);
    let (release_tx, release_rx) = std::sync::mpsc::sync_channel::<()>(1);
    let drv = StalledDisableDriver {
        enabled: false,
        entered: Arc::new(std::sync::Mutex::new(Some(entered_tx))),
        release: Arc::new(std::sync::Mutex::new(Some(release_rx))),
    };
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;
    sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
    let hooks = PersistTestHooks {
        panic_disable_timeout: Some(Duration::from_millis(60)),
        ..Default::default()
    };
    let handle = sup
        .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks)
        .await
        .unwrap();

    let h_em = handle.clone();
    let t_em = Instant::now();
    let outcome = tokio::task::spawn_blocking(move || h_em.emergency_disable_and_join())
        .await
        .unwrap();
    entered_rx
        .recv_timeout(Duration::from_millis(200))
        .expect("control thread must have entered blocked set_torque(false)");
    assert_eq!(outcome, PanicDisableOutcome::TimedOut);
    assert!(
        t_em.elapsed() < Duration::from_millis(600),
        "emergency teardown overran bound: {:?}",
        t_em.elapsed()
    );

    // Drop the LAST handle and supervisor while `release_tx` is STILL HELD (driver STILL blocked!):
    // `Drop` MUST return immediately rather than turning into an unbounded join!
    let t_drop = Instant::now();
    tokio::task::spawn_blocking(move || {
        drop(handle);
        drop(sup);
    })
    .await
    .unwrap();
    assert!(
        t_drop.elapsed() < Duration::from_millis(300),
        "dropping last handle after emergency timeout blocked for {:?} (unbounded join!)",
        t_drop.elapsed()
    );
    // Unblock the detached thread cleanly now that Drop boundedness has been proved.
    let _ = release_tx.send(());

    // (b) Panic deadline timeout: same guarantee when dropping the last handle while the
    // panicked owner thread is STILL blocked inside `set_torque(false)`.
    let (entered_tx2, entered_rx2) = std::sync::mpsc::sync_channel::<()>(1);
    let (release_tx2, release_rx2) = std::sync::mpsc::sync_channel::<()>(1);
    let (obs_tx2, obs_rx2) = std::sync::mpsc::sync_channel::<()>(1);
    let obs_tx2 = Arc::new(std::sync::Mutex::new(Some(obs_tx2)));
    let drv2 = StalledDisableDriver {
        enabled: false,
        entered: Arc::new(std::sync::Mutex::new(Some(entered_tx2))),
        release: Arc::new(std::sync::Mutex::new(Some(release_rx2))),
    };
    let sup2 = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv2)));
    sup2.set_lease_enabled(false).await;
    sup2.enable_torque(sup2.capture_admission_stamp()).await.unwrap();
    let mut cfg2 = ExecutorConfig::for_tests(Duration::from_millis(5));
    cfg2.panic_policy = PanicPolicy::StopExecutor;
    let hooks2 = PersistTestHooks {
        panic_on_release: Some(2),
        panic_disable_timeout: Some(Duration::from_millis(60)),
        on_panic_disable_timeout: Some(Arc::new(move || {
            if let Some(tx) = obs_tx2.lock().unwrap().take() {
                let _ = tx.send(());
            }
        })),
        ..Default::default()
    };
    let handle2 = sup2.start_executor_with_hooks(cfg2, hooks2).await.unwrap();
    wait_until_async(Duration::from_secs(2), || handle2.is_in_safe_hold()).await;
    let h2_em = handle2.clone();
    let em2_task = tokio::task::spawn_blocking(move || h2_em.emergency_disable_and_join());
    tokio::task::spawn_blocking(move || {
        entered_rx2.recv_timeout(Duration::from_secs(2)).expect("entered blocked disable");
        obs_rx2.recv_timeout(Duration::from_millis(600)).expect("observer fired TimedOut");
    })
    .await
    .unwrap();
    assert_eq!(em2_task.await.unwrap(), PanicDisableOutcome::TimedOut);

    // Drop the LAST handle and supervisor while `release_tx2` is STILL HELD!
    let t_drop2 = Instant::now();
    tokio::task::spawn_blocking(move || {
        drop(handle2);
        drop(sup2);
    })
    .await
    .unwrap();
    assert!(
        t_drop2.elapsed() < Duration::from_millis(300),
        "dropping last handle after panic timeout blocked for {:?} (unbounded join!)",
        t_drop2.elapsed()
    );
    let _ = release_tx2.send(());
}

// ============================================================================
// 11. Post-merge Item 1: J7 refresh boundary drain counts against release RX budget
// ============================================================================

#[test]
fn j7_refresh_drain_counts_against_release_rx_budget_and_reports_consumed_frames() {
    use armd::protocol::damiao;
    use armd::rt_core::broker::{BootId, Capacities, ClientId, Submit, TransitionOp};
    use armd::rt_core::ops::{CoreOps, ReleaseFixture, ServiceBudget};
    use armd::supervisor::SupervisorInner;
    use armd::transport::{CanFrame, CanTransport, DriverClock, TransportError};

    #[derive(Clone)]
    struct RxCountingInjectBus {
        inner: FakeBus,
        rx_dequeued: Arc<AtomicUsize>,
        inject_after_wouldblock: Arc<std::sync::Mutex<Option<Vec<CanFrame>>>>,
    }

    impl CanTransport for RxCountingInjectBus {
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
            self.inner.transmit(frame)
        }
        fn receive(&mut self) -> Result<CanFrame, TransportError> {
            self.receive_timestamped().map(|(f, _)| f)
        }
        fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
            match self.inner.receive_timestamped() {
                Ok(f) => {
                    self.rx_dequeued.fetch_add(1, Ordering::SeqCst);
                    Ok(f)
                }
                Err(e @ (TransportError::Timeout | TransportError::WouldBlock)) => {
                    if let Some(frames) = self.inject_after_wouldblock.lock().unwrap().take() {
                        self.inner.with(|s| {
                            for f in frames {
                                s.rx.push_back(f);
                            }
                        });
                    }
                    Err(e)
                }
                Err(e) => Err(e),
            }
        }
        fn flush(&mut self) -> Result<(), TransportError> {
            self.inner.flush()
        }
        fn clock(&self) -> Option<Arc<dyn DriverClock>> {
            self.inner.clock()
        }
        fn supports_nonblocking_rt(&self) -> bool {
            true
        }
        fn supports_bounded_flush(&self) -> bool {
            true
        }
    }

    let make_pf_cfg = || GripperPosForce {
        joint: 6,
        current_raw: 220,
        close_speed_rad_s: 0.30,
        closed_rad: 0.0,
        open_rad: -5.7,
        calibrated: false,
        ceiling_torque_nm: None,
        torque_reference: None,
    };

    // (a) RefreshBeforeRestore (boundary: None): `drain_rx_for_boundary` consumes N = 10 frames
    // inside `poll_hardware` with budget 20 -> NO extra RX pass runs in `poll_hardware_rx`,
    // total bus frames dequeued == 10, and `poll.rx_frames == 10` (not 0 or 10+extra).
    let clock = MockClock::new();
    let raw = FakeBus::with_clock(&dm_limits(), clock.clone());
    raw.with(|s| s.motors[6].mute_feedback = true);
    let bus = RxCountingInjectBus {
        inner: raw.clone(),
        rx_dequeued: Arc::new(AtomicUsize::new(0)),
        inject_after_wouldblock: Arc::new(std::sync::Mutex::new(None)),
    };
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap()
        .with_gripper_pos_force(make_pf_cfg())
        .unwrap();
    // Establish drained state with stale/missing J7 feedback so `begin_torque_budgeted(true, 0)`
    // enters `RefreshBeforeRestore { boundary: None }`.
    raw.with(|s| s.push_all_feedback());
    drv.read_joint_states().unwrap();
    drv.set_rt_mode(true);

    let mut inner = SupervisorInner::new(ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() })
        .with_driver(Box::new(drv));
    inner.lease_enabled = false;
    inner.register_live_connection(1).unwrap();
    let (mut fx, mut broker, _tw, _stop) =
        ReleaseFixture::new(inner, BootId(11), Capacities { commands: 4, plans: 2, replies: 8 });
    let a = fx.core.authority(&fx.ctx(fx.now()));
    broker
        .submit(ClientId(1), 1, a, fx.now() + Duration::from_secs(5), Submit::Transition(TransitionOp::TorqueOn))
        .unwrap();
    // Release 1 commits TorqueOn in Step 3 (`begin_torque_budgeted(true, 0)` -> `RefreshBeforeRestore { boundary: None }`).
    let rep1 = fx.step();
    assert!(rep1.command.unwrap().1.is_ok());

    // Queue N = 10 non-J7 frames before Release 2's `poll_hardware` with budget 20.
    let filler = raw.with(|s| s.motors[0].feedback_frame());
    raw.with(|s| {
        for _ in 0..10 {
            s.rx.push_back(filler);
        }
    });
    bus.rx_dequeued.store(0, Ordering::SeqCst);
    raw.clear_log();
    let budget20 = ServiceBudget {
        now: fx.now(),
        deadline: fx.now() + Duration::from_millis(20),
        max_rx_frames: 20,
    };
    let poll_a = fx.core.poll_hardware(&budget20);
    assert_eq!(
        bus.rx_dequeued.load(Ordering::SeqCst),
        10,
        "refresh drain consumed 10 frames; poll_hardware_rx must not run another full RX pass"
    );
    assert_eq!(poll_a.rx_frames, 10, "poll.rx_frames must include all 10 refresh-drain frames: {poll_a:?}");
    assert!(!poll_a.budget_exhausted, "10 < 20 budget should not exhaust: {poll_a:?}");
    assert!(
        raw.tx_log().iter().any(|f| f.id == 0x7FF && f.data[0] == 7 && f.data[2] == 0xCC),
        "expected 0xCC refresh after boundary drain"
    );

    // (b) `poll_readback` -> `start_feedback_refresh` -> `drain_rx_for_boundary` in ONE release:
    // budget = 12; `poll_readback`'s `receive_feedback` consumes N1 = 5 frames (4 filler + 1
    // CTRL_MODE=4 readback reply) + WouldBlock, which injects 10 more frames right before
    // `drain_rx_for_boundary`. The refresh drain MUST be reduced by N1 = 5 to 12 - 5 = 7 frames
    // (total dequeued == 12, NOT 15), with `poll.rx_frames == 12` and `budget_exhausted == true`.
    let clock_b = MockClock::new();
    let raw_b = FakeBus::with_clock(&dm_limits(), clock_b.clone());
    raw_b.with(|s| {
        s.motors[6].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_FORCE);
        s.motors[6].mute_feedback = true;
    });
    let bus_b = RxCountingInjectBus {
        inner: raw_b.clone(),
        rx_dequeued: Arc::new(AtomicUsize::new(0)),
        inject_after_wouldblock: Arc::new(std::sync::Mutex::new(None)),
    };
    let mut drv_b = DamiaoArmDriver::new(bus_b.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap()
        .with_gripper_pos_force(make_pf_cfg())
        .unwrap();
    drv_b.read_joint_states().unwrap();
    raw_b.with(|s| s.push_all_feedback());
    drv_b.read_joint_states().unwrap();
    drv_b.check_mapping_ranges(); // verifies J7 already in CTRL_MODE 4 -> skip Restore, enter Readback
    // Age J7 past 100 ms so readback reply triggers `PreEnableRefresh` -> `drain_rx_for_boundary`.
    clock_b.advance(Duration::from_millis(120));
    drv_b.read_joint_states().unwrap();
    drv_b.set_rt_mode(true);

    let mut inner_b = SupervisorInner::new(ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() })
        .with_driver(Box::new(drv_b));
    inner_b.lease_enabled = false;
    inner_b.register_live_connection(1).unwrap();
    let (mut fx_b, mut broker_b, _tw_b, _stop_b) =
        ReleaseFixture::new(inner_b, BootId(12), Capacities { commands: 4, plans: 2, replies: 8 });
    let a_b = fx_b.core.authority(&fx_b.ctx(fx_b.now()));
    broker_b
        .submit(ClientId(1), 1, a_b, fx_b.now() + Duration::from_secs(5), Submit::Transition(TransitionOp::TorqueOn))
        .unwrap();
    // Release 1 commits TorqueOn -> Readback { boundary: None }.
    assert!(fx_b.step().command.unwrap().1.is_ok());
    // Poll once to send the CTRL_MODE readback request (boundary becomes Some).
    raw_b.with(|s| s.motors[6].silent = true);
    let _ = fx_b.core.poll_hardware(&ServiceBudget::for_release(fx_b.now(), fx_b.now() + Duration::from_millis(20)));
    assert!(
        raw_b.tx_log().iter().any(|f| f.id == 0x7FF && f.data[0] == 7 && f.data[2] == damiao::REG_READ),
        "expected CTRL_MODE readback request"
    );

    // Queue N1 = 5 frames (4 filler + 1 valid CTRL_MODE=4 reply) and arm 10 extra frames on
    // WouldBlock so `drain_rx_for_boundary` runs immediately in the SAME release.
    let filler_b = raw_b.with(|s| s.motors[0].feedback_frame());
    let mode_bytes = damiao::CTRL_MODE_POS_FORCE.to_le_bytes();
    let readback_reply = CanFrame::new_standard(
        0x17,
        &[7, 0, damiao::REG_READ, damiao::RID_CTRL_MODE, mode_bytes[0], mode_bytes[1], mode_bytes[2], mode_bytes[3]],
    )
    .unwrap();
    raw_b.with(|s| {
        for _ in 0..4 {
            s.rx.push_back(filler_b);
        }
        s.rx.push_back(readback_reply);
    });
    *bus_b.inject_after_wouldblock.lock().unwrap() = Some(vec![filler_b; 10]);
    bus_b.rx_dequeued.store(0, Ordering::SeqCst);
    raw_b.clear_log();

    let budget12 = ServiceBudget {
        now: fx_b.now(),
        deadline: fx_b.now() + Duration::from_millis(20),
        max_rx_frames: 12,
    };
    let poll_b = fx_b.core.poll_hardware(&budget12);
    assert_eq!(
        bus_b.rx_dequeued.load(Ordering::SeqCst),
        12,
        "combined receive_feedback (5) + drain_rx_for_boundary (capped at 12-5=7) must not exceed budget 12"
    );
    assert_eq!(poll_b.rx_frames, 12, "reported rx_frames must include both passes: {poll_b:?}");
    assert!(poll_b.budget_exhausted, "hitting combined budget 12 must set budget_exhausted: {poll_b:?}");
    assert!(
        !raw_b.tx_log().iter().any(|f| f.id == 0x7FF && f.data[0] == 7 && f.data[2] == 0xCC),
        "must not send 0xCC when refresh drain exhausted remaining budget before queue emptied"
    );

    // (c) Non-caching driver reporting N = 7 frames from `take_last_poll_rx()`: `poll_hardware_rx`
    // must reduce its `read_joint_states_budgeted` call to `max_rx_frames - 7` and sum both counts.
    struct NonCachingAccountingDriver {
        last_transition_rx: Option<(u16, bool)>,
        seen_read_budget: Arc<AtomicUsize>,
        read_returns: u16,
    }
    impl ArmDriver for NonCachingAccountingDriver {
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
        fn is_enabled(&self) -> bool { false }
        fn enable_state(&self) -> EnableState { EnableState::Disabled }
        fn is_feedback_valid(&self) -> bool { true }
        fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); 7] }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            Ok(vec![JointState::default(); 7])
        }
        fn read_joint_states_budgeted(
            &mut self,
            out: &mut Vec<JointState>,
            _deadline: Instant,
            max_rx_frames: u16,
        ) -> Result<(u16, bool), ArmDriverError> {
            self.seen_read_budget.store(usize::from(max_rx_frames), Ordering::SeqCst);
            out.clear();
            out.resize(7, JointState::default());
            let used = self.read_returns.min(max_rx_frames);
            Ok((used, self.read_returns >= max_rx_frames))
        }
        fn read_cached_joint_states_into(&self, _out: &mut Vec<JointState>) -> Option<()> { None }
        fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
        fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
        fn begin_torque_budgeted(&mut self, _: bool, _: u16) -> Result<TorqueProgress, ArmDriverError> {
            Ok(TorqueProgress::Pending { wake: Instant::now() })
        }
        fn poll_torque_budgeted(&mut self, _: Instant, _: u16) -> Result<TorqueProgress, ArmDriverError> {
            self.last_transition_rx = Some((7, false));
            Ok(TorqueProgress::Pending { wake: Instant::now() + Duration::from_millis(5) })
        }
        fn take_last_poll_rx(&mut self) -> Option<(u16, bool)> {
            self.last_transition_rx.take()
        }
        fn supports_rt_executor(&self) -> Result<(), &'static str> { Ok(()) }
    }

    let seen_budget = Arc::new(AtomicUsize::new(0));
    let nc_drv = NonCachingAccountingDriver {
        last_transition_rx: None,
        seen_read_budget: seen_budget.clone(),
        read_returns: 4,
    };
    let mut inner_c = SupervisorInner::new(ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() })
        .with_driver(Box::new(nc_drv));
    inner_c.lease_enabled = false;
    inner_c.register_live_connection(1).unwrap();
    let (mut fx_c, mut broker_c, _tw_c, _stop_c) =
        ReleaseFixture::new(inner_c, BootId(13), Capacities { commands: 4, plans: 2, replies: 8 });
    let a_c = fx_c.core.authority(&fx_c.ctx(fx_c.now()));
    broker_c
        .submit(ClientId(1), 1, a_c, fx_c.now() + Duration::from_secs(5), Submit::Transition(TransitionOp::TorqueOn))
        .unwrap();
    assert!(fx_c.step().command.unwrap().1.is_ok());

    let poll_c = fx_c.core.poll_hardware(&ServiceBudget {
        now: fx_c.now(),
        deadline: fx_c.now() + Duration::from_millis(20),
        max_rx_frames: 20,
    });
    assert_eq!(seen_budget.load(Ordering::SeqCst), 13, "RX pass budget must be reduced by 7 (20 - 7 = 13)");
    assert_eq!(poll_c.rx_frames, 11, "reported rx_frames must sum transition (7) + read pass (4) = 11: {poll_c:?}");
    assert!(!poll_c.budget_exhausted);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn panic_disable_rt_persistent_j3_unconfirmed_and_damiao_stale_refresh_and_rx_fence() {
    #[derive(Clone)]
    struct FailJ3DisableBus {
        inner: FakeBus,
        fail_j3_remaining: Arc<AtomicUsize>,
        j3_failures: Arc<AtomicUsize>,
        j3_successes: Arc<AtomicUsize>,
    }

    impl CanTransport for FailJ3DisableBus {
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
            if frame.id == 3 && frame.data == armd::protocol::damiao::CMD_DISABLE {
                let prev = self.fail_j3_remaining.fetch_update(Ordering::SeqCst, Ordering::SeqCst, |v| {
                    v.checked_sub(1)
                });
                if prev.is_ok() {
                    self.j3_failures.fetch_add(1, Ordering::SeqCst);
                    return Err(TransportError::Io(std::io::Error::new(
                        std::io::ErrorKind::WouldBlock,
                        "persistent CAN TX failure on J3",
                    )));
                }
                self.j3_successes.fetch_add(1, Ordering::SeqCst);
            }
            self.inner.transmit(frame)
        }
        fn receive(&mut self) -> Result<CanFrame, TransportError> {
            self.inner.receive()
        }
        fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
            self.inner.receive_timestamped()
        }
        fn flush(&mut self) -> Result<(), TransportError> {
            self.inner.flush()
        }
        fn supports_nonblocking_rt(&self) -> bool {
            self.inner.supports_nonblocking_rt()
        }
        fn supports_bounded_flush(&self) -> bool {
            self.inner.supports_bounded_flush()
        }
        fn clock(&self) -> Option<Arc<dyn DriverClock>> {
            self.inner.clock()
        }
    }

    // (a) RT executor panic path with persistent J3 TX failure: cooperative exhaustion must
    // finish cleanly before the hard deadline observer triggers `on_panic_disable_timeout`.
    let inner_rt = FakeBus::new(&dm_limits());
    inner_rt.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let bus_rt = FailJ3DisableBus {
        inner: inner_rt.clone(),
        fail_j3_remaining: Arc::new(AtomicUsize::new(usize::MAX)),
        j3_failures: Arc::new(AtomicUsize::new(0)),
        j3_successes: Arc::new(AtomicUsize::new(0)),
    };
    let mut drv_rt = DamiaoArmDriver::new(bus_rt.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv_rt.read_joint_states().unwrap();
    drv_rt.check_mapping_ranges();
    let sup_rt = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_rt)));
    sup_rt.set_lease_enabled(false).await;
    sup_rt.enable_torque(sup_rt.capture_admission_stamp()).await.unwrap();
    inner_rt.with(|s| s.tx_log.clear());

    let timeout_fired = Arc::new(AtomicBool::new(false));
    let timeout_fired_cb = Arc::clone(&timeout_fired);
    let mut cfg_rt = ExecutorConfig::for_tests(Duration::from_millis(5));
    cfg_rt.panic_policy = PanicPolicy::StopExecutor;
    let handle = sup_rt
        .start_executor_with_hooks(
            cfg_rt,
            PersistTestHooks {
                panic_on_release: Some(2),
                panic_disable_timeout: Some(Duration::from_millis(80)),
                on_panic_disable_timeout: Some(Arc::new(move || {
                    timeout_fired_cb.store(true, Ordering::SeqCst);
                })),
                ..Default::default()
            },
        )
        .await
        .unwrap();
    wait_until_async(Duration::from_secs(2), || handle.is_in_safe_hold()).await;
    assert_eq!(bus_rt.j3_failures.load(Ordering::SeqCst), 0, "panic alone must hold without disabling");
    let h_em = handle.clone();
    let outcome = tokio::task::spawn_blocking(move || h_em.emergency_disable_and_join())
        .await
        .unwrap();
    assert_eq!(outcome, PanicDisableOutcome::Unconfirmed);

    assert!(
        !timeout_fired.load(Ordering::SeqCst),
        "on_panic_disable_timeout must NOT fire when cooperative exhaustion completes cleanly"
    );
    assert_eq!(
        handle.last_teardown_disable_outcome(),
        Some(PanicDisableOutcome::Unconfirmed),
        "RT teardown disable outcome must be published as Unconfirmed"
    );
    assert!(
        bus_rt.j3_failures.load(Ordering::SeqCst) >= 2,
        "J3 must be retried multiple times within cooperative budget"
    );
    inner_rt.with(|s| {
        for (i, m) in s.motors.iter().enumerate() {
            if i != 2 {
                assert!(!m.enabled, "motor J{} must be disabled despite persistent J3 TX failures", i + 1);
            }
        }
    });
    handle.stop_and_join();

    // (b) DamiaoArmDriver::panic_disable_step with MockClock:
    // - Partial-J3 stall > 250ms refreshes stale J1,J2,J4..J7 alongside J3.
    // - Post-confirmed sticky expiry > 250ms refreshes all 7 motors instead of early-returning.
    let clock = MockClock::new();
    let inner_clk = FakeBus::with_clock(&dm_limits(), clock.clone());
    inner_clk.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let bus_clk = FailJ3DisableBus {
        inner: inner_clk.clone(),
        fail_j3_remaining: Arc::new(AtomicUsize::new(1)),
        j3_failures: Arc::new(AtomicUsize::new(0)),
        j3_successes: Arc::new(AtomicUsize::new(0)),
    };
    let mut drv_clk = DamiaoArmDriver::new(bus_clk.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv_clk.read_joint_states().unwrap();
    drv_clk.check_mapping_ranges();
    drv_clk.set_torque(true).unwrap();

    // Pass 1 at t = 0: J1,J2,J4..J7 transmit CMD_DISABLE (J3 fails TX once) -> returns [2].
    let rem_pass1 = drv_clk
        .panic_disable_step(true, clock.now() + Duration::from_millis(10))
        .unwrap()
        .unwrap();
    assert_eq!(rem_pass1, vec![2]);
    assert_eq!(bus_clk.j3_failures.load(Ordering::SeqCst), 1);

    // Advance past the 250 ms feedback validity window so feedback telemetry becomes Unknown.
    clock.advance(Duration::from_millis(300));
    assert!(!drv_clk.is_feedback_valid());
    assert_eq!(drv_clk.enable_state(), EnableState::Unknown);
    assert_eq!(drv_clk.unconfirmed_disable_joints(), vec![2]);
    inner_clk.clear_log();

    // Pass 2: sends CMD_DISABLE to pending J3, completing immediately.
    let rem_pass2 = drv_clk
        .panic_disable_step(false, clock.now() + Duration::from_millis(10))
        .unwrap()
        .unwrap();
    assert!(rem_pass2.is_empty(), "pass 2 must complete after J3 CMD_DISABLE TX succeeds: {rem_pass2:?}");
    assert!(
        inner_clk.tx_log().iter().any(|f| f.id == 3 && f.data == armd::protocol::damiao::CMD_DISABLE),
        "pass 2 must send CMD_DISABLE to pending J3"
    );
    assert_eq!(bus_clk.j3_successes.load(Ordering::SeqCst), 1);
    assert!(drv_clk.unconfirmed_disable_joints().is_empty());

    // Already-expired step deadline on a fresh panic_disable_step(true, ...) transmits 0 CMD_DISABLE frames (Damiao TX deadline gate).
    clock.advance(Duration::from_millis(300));
    assert!(!drv_clk.is_feedback_valid());
    assert_eq!(drv_clk.enable_state(), EnableState::Unknown);
    inner_clk.clear_log();
    let rem_expired = drv_clk
        .panic_disable_step(true, clock.now())
        .unwrap()
        .unwrap();
    assert_eq!(rem_expired, vec![0, 1, 2, 3, 4, 5, 6]);
    assert!(
        inner_clk
            .tx_log()
            .iter()
            .all(|f| f.data != armd::protocol::damiao::CMD_DISABLE),
        "expired step deadline must transmit 0 CMD_DISABLE frames"
    );

    // Follow-up step with a valid deadline disables all 7 motors and restores fresh Disabled telemetry.
    inner_clk.clear_log();
    let rem_retry = drv_clk
        .panic_disable_step(false, clock.now() + Duration::from_millis(10))
        .unwrap()
        .unwrap();
    assert!(rem_retry.is_empty());
    assert!(drv_clk.is_feedback_valid());
    assert_eq!(drv_clk.enable_state(), EnableState::Disabled);

    // (c) Undrained RX boundary on single `panic_disable_step(true, ...)`:
    // Queue stale state-0 feedback without draining (`last_read_drained == false`), then run one
    // `panic_disable_step(true, ...)` with J3 failing once.
    let clock_rx = MockClock::new();
    let inner_rx = FakeBus::with_clock(&dm_limits(), clock_rx.clone());
    inner_rx.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let bus_rx = FailJ3DisableBus {
        inner: inner_rx.clone(),
        fail_j3_remaining: Arc::new(AtomicUsize::new(1)),
        j3_failures: Arc::new(AtomicUsize::new(0)),
        j3_successes: Arc::new(AtomicUsize::new(0)),
    };
    let mut drv_rx = DamiaoArmDriver::new(bus_rx.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv_rx.read_joint_states().unwrap();
    drv_rx.check_mapping_ranges();
    drv_rx.set_torque(true).unwrap();

    // Switch to non-immediate replies and push 8 stale state-0 feedback frames; read only 1 so
    // `last_read_drained == false` and 7 stale state-0 frames remain undrained in `rx`.
    inner_rx.with(|s| {
        s.immediate_replies = false;
        for m in &s.motors {
            let stale = CanFrame::new_standard(m.reply_id, &[m.id as u8 & 0x0f, 0, 0, 0, 0, 0, 0, 0]).unwrap();
            s.rx.push_back(stale);
        }
        let extra = CanFrame::new_standard(s.motors[0].reply_id, &[1, 0, 0, 0, 0, 0, 0, 0]).unwrap();
        s.rx.push_back(extra);
    });
    let mut scratch = Vec::new();
    let (read_n, exhausted) = drv_rx
        .read_joint_states_budgeted(&mut scratch, clock_rx.now() + Duration::from_millis(10), 1)
        .unwrap();
    assert_eq!(read_n, 1);
    assert!(exhausted);
    inner_rx.clear_log();

    let rem_rx = drv_rx
        .panic_disable_step(true, clock_rx.now() + Duration::from_millis(2))
        .unwrap()
        .unwrap();
    assert_eq!(bus_rx.j3_failures.load(Ordering::SeqCst), 1, "J3 disable must fail once");
    let rx_disable_ids: Vec<u32> = inner_rx
        .tx_log()
        .iter()
        .filter(|f| f.data == armd::protocol::damiao::CMD_DISABLE)
        .map(|f| f.id)
        .collect();
    for id in [1u32, 2, 4, 5, 6, 7] {
        assert!(
            rx_disable_ids.contains(&id),
            "J1,J2,J4..J7 must receive CMD_DISABLE (missing J{id}): {rx_disable_ids:?}"
        );
    }
    assert!(!rem_rx.is_empty(), "pre-boundary stale state-0 feedback must not confirm disable");
    assert_ne!(
        drv_rx.enable_state(),
        EnableState::Disabled,
        "pre-boundary stale state-0 feedback must not mark driver Disabled"
    );
}
