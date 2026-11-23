//! Stage 3 step 1 ownership transfer verification:
//! - Strictly one bus writer (`armd-control`) and safe one-way Legacy -> Executor handoff
//! - No Tokio runtime entry during a release (with thread-local guard + negative panic tests)
//! - Exact per-release order via `ReleaseEngine` & single budgeted RX pass
//! - Absolute monotonic release grid, `skipped_releases` accounting, real elapsed `dt` across wakes
//! - Lock-serialized per-waiter pinned E-stop acknowledgement (> 64 epochs & handoff race)
//! - Bounded `clear_fault_budgeted` semantics
//! - Legacy vs Executor parity across mock, `SimArmDriver` UDP loopback (`127.0.0.1:0`), and
//!   `DamiaoArmDriver<FakeBus>` (publication, watchdog park, torque confirmation/failure, shutdown)
//! - Two-phase off-thread persistence (all outcomes + B1 race + Persistence #5 + saturation)
//! - Off-thread diagnostics & payload reclamation under paused dispatcher
//! - CLI startup gate smoke test (`ARMD_RT_OWNER=executor`)

mod common;
mod mit_support;

#[global_allocator]
static ALLOC: armd::diagnostics::alloc::TimedAlloc = armd::diagnostics::alloc::TimedAlloc;

use std::net::{SocketAddr, TcpListener, UdpSocket};
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::thread::{self, ThreadId};
use std::time::{Duration, Instant};

use axum::body::Body;
use axum::http::{Request, StatusCode};
use serde_json::json;
use tower::ServiceExt;

use armd::driver::{
    ArmDriver, ArmDriverError, DamiaoArmDriver, DriverCapabilities, EnableState, FashionStarArmDriver,
    FeetechArmDriver, JointTarget, TorqueOutcome, TorqueProgress,
};
use armd::model::JointState;
use armd::protocol::damiao::DamiaoLimits;
use armd::rt_core::broker::{
    Authority, BootId, Capacities, ClientId, ImmediateOp, Outcome, Reply, RequestId, Submit,
    TransitionOp,
};
use armd::rt_core::config_iface::{ConfigAuthority, ConfigDomain};
use armd::rt_core::executor::{
    assert_not_in_rt_release, in_rt_release, ExecutorConfig, MaintenanceOp, OwnerError, PanicPolicy,
    PersistTestHooks, ReleaseGuard, RtExecutorHandle,
};
use armd::rt_core::ids::ArmId;
use armd::rt_core::mailbox::{Refusal, StreamId, TargetFrame, MAX_DOF};
use armd::rt_core::ops::{
    Checkpoint, Committed, CoreOps, HardwarePoll, PlanSlot, PlanningSnapshot, Primitive,
    ReleaseEngine, ReleaseFixture, ServiceBudget,
};
use armd::sim_driver::{SimArmDriver, SimCommand, SimState};
use armd::supervisor::{OWNER_EXECUTOR, OWNER_FAILED, OWNER_LEGACY};
use armd::supervisor_lock::SupervisorLock;
use armd::transport::{CanFrame, CanTransport, DriverClock, MockClock, MockTransport, TransportError};
use armd::{
    create_router_with_auth, ArmConfig, ArmError, ArmSupervisor, OperatorAuth, SafetyState,
    SupervisorInner,
};
use common::INSPECT;
use mit_support::FakeBus;

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
    while Instant::now() < deadline {
        if pred() {
            return;
        }
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
    assert!(pred(), "async condition not satisfied within {timeout:?}");
}

// ============================================================================
// Shared test doubles
// ============================================================================

/// Driver double recording every calling `ThreadId` and verifying no Tokio runtime is active
/// once ownership has been handed over to the RT executor.
struct ThreadAuditDriver {
    dof: usize,
    positions: Vec<f32>,
    enabled: bool,
    enforce_rt_context: Arc<AtomicBool>,
    threads: Arc<Mutex<Vec<ThreadId>>>,
    tokio_entered: Arc<AtomicBool>,
    missing_guard: Arc<AtomicBool>,
    write_count: Arc<AtomicUsize>,
    read_count: Arc<AtomicUsize>,
    torque_pending: Option<(bool, Instant)>,
    confirm_delay: Duration,
}

impl ThreadAuditDriver {
    fn new(dof: usize) -> (
        Self,
        Arc<AtomicBool>,
        Arc<Mutex<Vec<ThreadId>>>,
        Arc<AtomicBool>,
        Arc<AtomicBool>,
        Arc<AtomicUsize>,
        Arc<AtomicUsize>,
    ) {
        let enforce = Arc::new(AtomicBool::new(false));
        let threads = Arc::new(Mutex::new(Vec::with_capacity(1024)));
        let tokio_entered = Arc::new(AtomicBool::new(false));
        let missing_guard = Arc::new(AtomicBool::new(false));
        let writes = Arc::new(AtomicUsize::new(0));
        let reads = Arc::new(AtomicUsize::new(0));
        let drv = Self {
            dof,
            positions: vec![0.0; dof],
            enabled: false,
            enforce_rt_context: enforce.clone(),
            threads: threads.clone(),
            tokio_entered: tokio_entered.clone(),
            missing_guard: missing_guard.clone(),
            write_count: writes.clone(),
            read_count: reads.clone(),
            torque_pending: None,
            confirm_delay: Duration::from_millis(14),
        };
        (drv, enforce, threads, tokio_entered, missing_guard, writes, reads)
    }

    fn record_access(&self) {
        if self.enforce_rt_context.load(Ordering::Acquire) {
            self.threads.lock().unwrap().push(thread::current().id());
            if tokio::runtime::Handle::try_current().is_ok() {
                self.tokio_entered.store(true, Ordering::SeqCst);
            }
            if !in_rt_release() {
                self.missing_guard.store(true, Ordering::SeqCst);
            }
        }
    }
}

impl ArmDriver for ThreadAuditDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            enable_feedback: true,
            velocity_feedback: true,
            torque_feedback: true,
            temperature_feedback: true,
            ..DriverCapabilities::default()
        }
    }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        self.record_access();
        self.read_count.fetch_add(1, Ordering::Relaxed);
        Ok(self.positions.iter().map(|&p| JointState { position: p, velocity: 0.0, torque: 0.0, temperature: 30.0 }).collect())
    }
    fn read_joint_states_budgeted(
        &mut self,
        out: &mut Vec<JointState>,
        _deadline: Instant,
        _max_rx_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        self.record_access();
        self.read_count.fetch_add(1, Ordering::Relaxed);
        out.clear();
        out.extend(self.positions.iter().map(|&p| JointState { position: p, velocity: 0.0, torque: 0.0, temperature: 30.0 }));
        Ok((self.dof as u16, false))
    }
    fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
        out.clear();
        out.extend(self.positions.iter().map(|&p| JointState { position: p, velocity: 0.0, torque: 0.0, temperature: 30.0 }));
        Some(())
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.record_access();
        self.write_count.fetch_add(1, Ordering::Relaxed);
        for (p, t) in self.positions.iter_mut().zip(targets) {
            *p = t.position;
        }
        Ok(())
    }
    fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
        self.record_access();
        self.enabled = enable;
        Ok(())
    }
    fn begin_torque_budgeted(&mut self, enable: bool, _max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
        self.record_access();
        let wake = Instant::now() + self.confirm_delay;
        self.torque_pending = Some((enable, wake));
        Ok(TorqueProgress::Pending { wake })
    }
    fn poll_torque_budgeted(
        &mut self,
        _deadline: Instant,
        _max_rx_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.record_access();
        if let Some((enable, ready_at)) = self.torque_pending {
            if Instant::now() >= ready_at {
                self.enabled = enable;
                self.torque_pending = None;
                return Ok(TorqueProgress::Done(TorqueOutcome::Confirmed));
            }
            return Ok(TorqueProgress::Pending { wake: ready_at });
        }
        Err(ArmDriverError::Protocol("no torque transition pending".into()))
    }
    fn torque_transition_pending(&self) -> bool { self.torque_pending.is_some() }
    fn supports_rt_executor(&self) -> Result<(), &'static str> { Ok(()) }
    fn is_feedback_valid(&self) -> bool { true }
    fn is_enabled(&self) -> bool { self.enabled }
    fn enable_state(&self) -> EnableState {
        if self.enabled { EnableState::Enabled } else { EnableState::Disabled }
    }
    fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
        dof == self.dof && Duration::from_millis(1) <= max_age
    }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::from_millis(1)); self.dof] }
    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
        out.clear();
        out.resize(self.dof, Some(Duration::from_millis(1)));
    }
    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
        out.clear();
        out.resize(self.dof, Some(1));
    }
    fn feedback_instants(&self) -> Vec<Option<Instant>> { vec![Some(Instant::now()); self.dof] }
    fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        out.clear();
        out.resize(self.dof, Some(Instant::now()));
    }
    fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        out.clear();
        out.resize(self.dof, Some(Instant::now()));
    }
    fn feedback_seqs(&self) -> Vec<u64> { vec![1; self.dof] }
    fn feedback_seqs_into(&self, out: &mut Vec<u64>) {
        out.clear();
        out.resize(self.dof, 1);
    }
    fn feedback_queue_bounds_into(&self, out: &mut Vec<Option<Instant>>) { out.clear(); }
    fn take_recovery_writes_into(&mut self, out: &mut Vec<usize>) { out.clear(); }
    fn joint_loads_into(&self, out: &mut Vec<f32>) -> bool { out.clear(); false }
    fn motor_status_into(&self, out: &mut Vec<u8>) -> bool {
        out.clear();
        out.resize(self.dof, if self.enabled { 1 } else { 0 });
        true
    }
}

/// Drop probe recording the `ThreadId` on which `Drop::drop` ran.
struct DropProbe {
    dropped_on: Arc<Mutex<Vec<ThreadId>>>,
}

impl Drop for DropProbe {
    fn drop(&mut self) {
        self.dropped_on.lock().unwrap().push(thread::current().id());
    }
}

/// Clock wrapper counting every `DriverClock::sleep` call (proves 0 retry sleeps in RT mode).
#[derive(Clone)]
struct CountingClock {
    inner: MockClock,
    pauses: Arc<AtomicUsize>,
}

impl DriverClock for CountingClock {
    fn clock_id(&self) -> u64 { self.inner.clock_id() }
    fn now(&self) -> Instant { self.inner.now() }
    fn sleep(&self, d: Duration) {
        self.pauses.fetch_add(1, Ordering::Relaxed);
        self.inner.sleep(d);
    }
    fn yield_now(&self) { self.inner.yield_now(); }
}

/// Counting wrapper over `FakeBus` (or custom CAN transport) to verify exact RX call bounds.
#[derive(Clone)]
struct CountingBus {
    inner: FakeBus,
    rx_calls: Arc<AtomicUsize>,
    supports_flush: bool,
    supports_rt: bool,
    stall_flush: Arc<AtomicBool>,
    clock: Option<MockClock>,
    clock_pauses: Arc<AtomicUsize>,
    estop_on_tx: Option<(Arc<std::sync::atomic::AtomicU64>, usize)>,
    tx_seen: Arc<AtomicUsize>,
}

impl CountingBus {
    fn new(inner: FakeBus, clock: Option<MockClock>) -> Self {
        Self {
            inner,
            rx_calls: Arc::new(AtomicUsize::new(0)),
            supports_flush: true,
            supports_rt: true,
            stall_flush: Arc::new(AtomicBool::new(false)),
            clock,
            clock_pauses: Arc::new(AtomicUsize::new(0)),
            estop_on_tx: None,
            tx_seen: Arc::new(AtomicUsize::new(0)),
        }
    }
}

impl CanTransport for CountingBus {
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        let n = self.tx_seen.fetch_add(1, Ordering::Relaxed) + 1;
        if let Some((req_count, after)) = &self.estop_on_tx {
            if n >= *after {
                req_count.fetch_add(1, Ordering::SeqCst);
            }
        }
        self.inner.transmit(frame)
    }

    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        self.rx_calls.fetch_add(1, Ordering::Relaxed);
        self.inner.receive()
    }

    fn flush(&mut self) -> Result<(), TransportError> {
        if self.stall_flush.load(Ordering::Acquire) {
            if let Some(c) = &self.clock {
                c.advance(Duration::from_millis(10));
            }
            return Err(TransportError::WouldBlock);
        }
        self.inner.flush()
    }

    fn clock(&self) -> Option<Arc<dyn DriverClock>> {
        self.clock.clone().map(|c| {
            Arc::new(CountingClock {
                inner: c,
                pauses: self.clock_pauses.clone(),
            }) as Arc<dyn DriverClock>
        })
    }

    fn supports_bounded_flush(&self) -> bool {
        self.supports_flush
    }

    fn supports_nonblocking_rt(&self) -> bool {
        self.supports_rt
    }
}

/// Loopback UDP responder for `SimArmDriver` parity tests (always binds `127.0.0.1:0`).
struct LoopbackSim {
    addr: String,
    stop: Arc<AtomicBool>,
    enabled: Arc<AtomicBool>,
    thread: Option<thread::JoinHandle<()>>,
}

impl LoopbackSim {
    fn start(dof: usize, initial_q: Vec<f32>) -> Self {
        let socket = UdpSocket::bind("127.0.0.1:0").unwrap();
        let addr = format!("udp://{}", socket.local_addr().unwrap());
        socket.set_read_timeout(Some(Duration::from_millis(10))).unwrap();
        let stop = Arc::new(AtomicBool::new(false));
        let enabled = Arc::new(AtomicBool::new(false));
        let q = Arc::new(Mutex::new(initial_q));
        let (stop_c, enabled_c, q_c) = (stop.clone(), enabled.clone(), q);
        let handle = thread::Builder::new()
            .name("test-sim-udp".into())
            .spawn(move || {
                let mut buf = [0u8; 4096];
                let mut seq = 0u64;
                let mut ack = 0u64;
                let mut peer: Option<SocketAddr> = None;
                let mut last_tx = Instant::now() - Duration::from_secs(1);
                while !stop_c.load(Ordering::Acquire) {
                    if let Ok((n, from)) = socket.recv_from(&mut buf) {
                        peer = Some(from);
                        if let Ok(cmd) = serde_json::from_slice::<SimCommand>(&buf[..n]) {
                            if cmd.kind == "cmd" {
                                ack = ack.max(cmd.seq);
                                if let Some(en) = cmd.enable {
                                    enabled_c.store(en, Ordering::Release);
                                }
                                if let Some(targets) = cmd.targets {
                                    let mut cur = q_c.lock().unwrap();
                                    for (i, t) in targets.iter().enumerate().take(dof) {
                                        cur[i] = match t {
                                            armd::sim_driver::SimTarget::PosVel { q, .. } => *q,
                                            armd::sim_driver::SimTarget::Mit { q, .. } => *q,
                                        };
                                    }
                                }
                            }
                            seq += 1;
                            let state = SimState {
                                v: 1,
                                kind: "state".into(),
                                session: 1,
                                seq,
                                ack,
                                enabled: enabled_c.load(Ordering::Acquire),
                                q: q_c.lock().unwrap().clone(),
                                dq: vec![0.0; dof],
                                tau: vec![0.0; dof],
                                temp: vec![30.0; dof],
                                loads: None,
                                t: seq as f64 * 0.005,
                            };
                            if let Ok(bytes) = serde_json::to_vec(&state) {
                                let _ = socket.send_to(&bytes, from);
                                last_tx = Instant::now();
                            }
                        }
                    } else if let Some(p) = peer {
                        if last_tx.elapsed() >= Duration::from_millis(5) {
                            seq += 1;
                            let state = SimState {
                                v: 1,
                                kind: "state".into(),
                                session: 1,
                                seq,
                                ack,
                                enabled: enabled_c.load(Ordering::Acquire),
                                q: q_c.lock().unwrap().clone(),
                                dq: vec![0.0; dof],
                                tau: vec![0.0; dof],
                                temp: vec![30.0; dof],
                                loads: None,
                                t: seq as f64 * 0.005,
                            };
                            if let Ok(bytes) = serde_json::to_vec(&state) {
                                let _ = socket.send_to(&bytes, p);
                                last_tx = Instant::now();
                            }
                        }
                    }
                }
            })
            .unwrap();
        Self { addr, stop, enabled, thread: Some(handle) }
    }
}

impl Drop for LoopbackSim {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

fn submit_broker_transition(handle: &RtExecutorHandle, seq: u64, op: TransitionOp) -> RequestId {
    let snap = handle.planning_snapshot().unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut b = handle.broker().lock().unwrap();
    b.submit(ClientId(1), seq, snap.authority, deadline, Submit::Transition(op)).unwrap()
}

fn wait_broker_terminal(handle: &RtExecutorHandle, req: RequestId, timeout: Duration) -> Outcome {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        let mut found = None;
        handle.broker().lock().unwrap().poll(|_, r| {
            if let Reply::Terminal { request, outcome, .. } = r {
                if request == req {
                    found = Some(outcome);
                }
            }
            true
        });
        if let Some(o) = found {
            return o;
        }
        thread::sleep(Duration::from_millis(2));
    }
    panic!("broker request {req:?} did not reach terminal within {timeout:?}");
}

// ============================================================================
// 1. Single bus writer, no Tokio entry during release, and handoff lifecycle
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn single_bus_writer_no_tokio_entry_and_handoff_gates() {
    let (drv, enforce, threads, tokio_entered, missing_guard, writes, reads) = ThreadAuditDriver::new(7);
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;

    enforce.store(true, Ordering::Release);
    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .expect("initial transfer succeeds");
    assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);

    // Duplicate transfer is rejected with 409 OWNER_ALREADY_TRANSFERRED.
    match sup.start_executor(ExecutorConfig::for_tests(Duration::from_millis(5))).await {
        Err(ArmError::Rejected { status: 409, code, .. }) => assert_eq!(code, "OWNER_ALREADY_TRANSFERRED"),
        Ok(_) => panic!("duplicate start_executor unexpectedly succeeded"),
        Err(other) => panic!("unexpected error: {other:?}"),
    }

    // Exercise torque transition (includes sub-release wakes) and periodic output writes.
    let req_on = submit_broker_transition(&handle, 1, TransitionOp::TorqueOn);
    assert_eq!(wait_broker_terminal(&handle, req_on, Duration::from_secs(2)), Outcome::Completed);
    wait_until_async(Duration::from_secs(1), || {
        writes.load(Ordering::Relaxed) >= 5 && reads.load(Ordering::Relaxed) >= 5
    })
    .await;

    // Every driver touch after handoff came strictly from `executor_thread_id`, outside Tokio,
    // and inside `ReleaseGuard`.
    let exec_tid = handle.executor_thread_id();
    let seen = threads.lock().unwrap().clone();
    assert!(seen.len() >= 10, "expected >= 10 driver calls, got {}", seen.len());
    assert!(
        seen.iter().all(|&tid| tid == exec_tid),
        "found bus access from non-executor thread: {seen:?} (executor={exec_tid:?})"
    );
    assert!(!tokio_entered.load(Ordering::SeqCst), "driver observed active Tokio runtime inside RT release");
    assert!(!missing_guard.load(Ordering::SeqCst), "driver called without ReleaseGuard active");

    // Legacy mutators fail closed with 503 OWNER_TRANSFERRED.
    let (dummy_drv, ..) = ThreadAuditDriver::new(7);
    assert!(matches!(
        sup.try_set_driver(Box::new(dummy_drv)).await,
        Err(ArmError::Rejected { status: 503, ref code, .. }) if code == "OWNER_TRANSFERRED"
    ));
    assert!(matches!(
        sup.set_torque(false).await,
        Err(ArmError::Rejected { status: 503, ref code, .. }) if code == "OWNER_TRANSFERRED"
    ));
    assert!(matches!(
        sup.servo(vec![0.0; 7], 0).await,
        Err(ArmError::Rejected { status: 503, ref code, .. }) if code == "OWNER_TRANSFERRED"
    ));
    assert!(matches!(
        sup.jog(vec![0.0; 7], Some(1.0), None).await,
        Err(ArmError::Rejected { status: 503, ref code, .. }) if code == "OWNER_TRANSFERRED"
    ));

    // `shutdown_and_join` refuses until `request_shutdown` confirms safe exit.
    assert_eq!(handle.shutdown_and_join(), Err(OwnerError::ShutdownNotConfirmed));

    // Refused shutdown while torqued latches E-stop and keeps executor alive.
    assert!(!sup.request_shutdown().await.unwrap());
    assert!(!handle.is_exited());
    assert_eq!(handle.shutdown_and_join(), Err(OwnerError::ShutdownNotConfirmed));

    // Clear fault + disable torque -> shutdown confirms -> `shutdown_and_join` succeeds.
    assert_eq!(sup.clear_fault().await.unwrap(), SafetyState::Idle);
    let req_off = submit_broker_transition(&handle, 2, TransitionOp::TorqueOff);
    assert_eq!(wait_broker_terminal(&handle, req_off, Duration::from_secs(2)), Outcome::Completed);
    assert!(sup.request_shutdown().await.unwrap());
    handle.shutdown_and_join().expect("shutdown_and_join after confirmation");
    assert!(handle.is_exited());
}

// ============================================================================
// 2. Queued legacy callers across handoff, cancel rollback, init failures, panic
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn queued_legacy_callers_cancel_rollback_and_init_failures() {
    // (a) Async cancellation while waiting for `inner.write()` rolls back Transferring -> Legacy.
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    {
        let _hold = sup.inner().write().await;
        let fut = sup.start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)));
        let res = tokio::time::timeout(Duration::from_millis(20), fut).await;
        assert!(res.is_err(), "start_executor should be parked on inner.write()");
        // `fut` is dropped here; `TransferGuard` rolls back to `OWNER_LEGACY`.
    }
    assert_eq!(sup.owner_mode(), OWNER_LEGACY);

    // (b) Recoverable init failure (`fail_init: Some(true)`) restores `OWNER_LEGACY` and real state.
    {
        let mut inner = sup.inner().write().await;
        inner.measured_positions[0] = 0.42;
        inner.commanded_positions[0] = 0.42;
        inner.target_positions[0] = 0.42;
    }
    let hooks_rec = PersistTestHooks { fail_init: Some(true), ..Default::default() };
    let err = sup
        .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks_rec)
        .await;
    assert!(matches!(err, Err(ArmError::Internal(_))));
    assert_eq!(sup.owner_mode(), OWNER_LEGACY);
    assert!((sup.inner().read().await.measured_positions[0] - 0.42).abs() < 1e-6);

    // (c) Queued legacy callers waiting on `inner.write()` while `start_executor` transfers.
    let guard = sup.inner().write().await;
    let sup_t = sup.clone();
    let transfer = tokio::spawn(async move {
        sup_t.start_executor(ExecutorConfig::for_tests(Duration::from_millis(5))).await
    });
    tokio::time::sleep(Duration::from_millis(15)).await;
    let sup_m1 = sup.clone();
    let queued_set_torque = tokio::spawn(async move { sup_m1.set_torque(true).await });
    let sup_m2 = sup.clone();
    let queued_try_drv = tokio::spawn(async move {
        let (d, ..) = ThreadAuditDriver::new(7);
        sup_m2.try_set_driver(Box::new(d)).await
    });
    let sup_tick = sup.clone();
    let queued_tick = tokio::spawn(async move { sup_tick.control_tick(0.02).await });
    tokio::time::sleep(Duration::from_millis(15)).await;
    drop(guard);

    let handle = transfer.await.unwrap().expect("transfer succeeds");
    assert!(matches!(
        queued_set_torque.await.unwrap(),
        Err(ArmError::Rejected { status: 503, ref code, .. }) if code == "OWNER_TRANSFERRED"
    ));
    assert!(matches!(
        queued_try_drv.await.unwrap(),
        Err(ArmError::Rejected { status: 503, ref code, .. }) if code == "OWNER_TRANSFERRED"
    ));
    queued_tick.await.unwrap();
    // Observation serves live executor publication (positions[0] == 0.42, NOT detached 0.0!).
    wait_until_async(Duration::from_secs(1), || handle.stats().steps.load(Ordering::Relaxed) >= 2).await;
    let obs = sup.observation().await.expect("live observation from StateHub");
    assert!((obs.positions[0] - 0.42).abs() < 1e-4, "obs read detached shell: {:?}", obs.positions);
    handle.stop_and_join();

    // (d) Non-recoverable init failure (`fail_init: Some(false)`) enters terminal `OWNER_FAILED`.
    let sup_fail = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let hooks_lost = PersistTestHooks { fail_init: Some(false), ..Default::default() };
    let err = sup_fail
        .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks_lost)
        .await;
    assert!(matches!(err, Err(ArmError::Internal(_))));
    assert_eq!(sup_fail.owner_mode(), OWNER_FAILED);
    assert!(sup_fail.observation().await.is_err());
    assert!(sup_fail.request_shutdown().await.is_err());
    assert!(sup_fail.set_torque(true).await.is_err());
    assert!(sup_fail.start_executor(ExecutorConfig::for_tests(Duration::from_millis(5))).await.is_err());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_panic_under_stop_policy_latches_estop_and_wakes_waiters() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(4));
    cfg.panic_policy = PanicPolicy::StopExecutor;
    let hooks = PersistTestHooks { panic_on_release: Some(2), ..Default::default() };
    let handle = sup.start_executor_with_hooks(cfg, hooks).await.unwrap();

    wait_until_async(Duration::from_secs(2), || handle.is_exited()).await;
    assert_eq!(handle.observation(), Err(OwnerError::Stopped));
    assert_eq!(handle.request_shutdown(), Err(OwnerError::Stopped));
    match sup.trigger_e_stop_checked().await {
        Err(ArmError::Rejected { status: 503, code, .. }) => assert_eq!(code, "EXECUTOR_STOPPED"),
        other => panic!("expected 503 EXECUTOR_STOPPED after executor panic, got {other:?}"),
    }
    handle.stop_and_join();
}

// ============================================================================
// 3. Negative panic tests for ReleaseGuard / no Tokio / no legacy lock entry
// ============================================================================

#[test]
#[should_panic(expected = "forbidden Tokio or legacy supervisor lock entry during RT release")]
fn assert_not_in_rt_release_panics_inside_guard() {
    let _g = ReleaseGuard::enter();
    assert_not_in_rt_release("unit_probe");
}

#[tokio::test]
#[should_panic(expected = "forbidden Tokio or legacy supervisor lock entry during RT release")]
async fn supervisor_lock_write_tick_panics_inside_release_guard() {
    let lock = SupervisorLock::new(0u32);
    let _g = ReleaseGuard::enter();
    let _ = lock.write_tick().await;
}

#[tokio::test]
#[should_panic(expected = "forbidden Tokio or legacy supervisor lock entry during RT release")]
async fn supervisor_control_tick_panics_inside_release_guard() {
    let sup = ArmSupervisor::new(ArmConfig::rebot_dm());
    let _g = ReleaseGuard::enter();
    sup.control_tick(0.02).await;
}

#[tokio::test]
#[should_panic(expected = "forbidden Tokio or legacy supervisor lock entry during RT release")]
async fn supervisor_trigger_e_stop_panics_inside_release_guard() {
    let sup = ArmSupervisor::new(ArmConfig::rebot_dm());
    let _g = ReleaseGuard::enter();
    let _ = sup.trigger_e_stop().await;
}

// ============================================================================
// 4. Exact per-release order & unconditional checkpoints on ReleaseEngine
// ============================================================================

/// Minimal 1-poll-delayed torque driver for deterministic release-order testing.
struct StepConfirmDriver {
    dof: usize,
    enabled: bool,
    pending: Option<bool>,
}

impl ArmDriver for StepConfirmDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            enable_feedback: true,
            velocity_feedback: true,
            torque_feedback: true,
            temperature_feedback: true,
            ..DriverCapabilities::default()
        }
    }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        Ok(vec![JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 30.0 }; self.dof])
    }
    fn read_joint_states_budgeted(
        &mut self,
        out: &mut Vec<JointState>,
        _deadline: Instant,
        _max_rx_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        out.clear();
        out.resize(self.dof, JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 30.0 });
        Ok((self.dof as u16, false))
    }
    fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
        out.clear();
        out.resize(self.dof, JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 30.0 });
        Some(())
    }
    fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
    fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> { self.enabled = enable; Ok(()) }
    fn begin_torque_budgeted(&mut self, enable: bool, _max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
        self.pending = Some(enable);
        Ok(TorqueProgress::Pending { wake: Instant::now() + Duration::from_secs(1) })
    }
    fn poll_torque_budgeted(
        &mut self,
        _deadline: Instant,
        _max_rx_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        if let Some(enable) = self.pending.take() {
            self.enabled = enable;
            return Ok(TorqueProgress::Done(TorqueOutcome::Confirmed));
        }
        Err(ArmDriverError::Protocol("no torque transition pending".into()))
    }
    fn torque_transition_pending(&self) -> bool { self.pending.is_some() }
    fn is_feedback_valid(&self) -> bool { true }
    fn is_enabled(&self) -> bool { self.enabled }
    fn enable_state(&self) -> EnableState {
        if self.enabled { EnableState::Enabled } else { EnableState::Disabled }
    }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::from_millis(1)); self.dof] }
    fn feedback_instants(&self) -> Vec<Option<Instant>> { vec![Some(Instant::now()); self.dof] }
    fn feedback_seqs(&self) -> Vec<u64> { vec![1; self.dof] }
}

struct OrderRecordingCore {
    inner: SupervisorInner,
    log: Arc<Mutex<Vec<&'static str>>>,
    published: bool,
    /// Shared broker handle polled INSIDE `publish()` so any terminal prematurely emitted
    /// before publication fails inside the release itself.
    broker: Option<Arc<Mutex<armd::rt_core::broker::Broker<PlanSlot>>>>,
    replies_seen_during_publish: Vec<Reply>,
}

impl CoreOps for OrderRecordingCore {
    fn authority(&self, ctx: &armd::rt_core::ops::CommitCtx) -> Authority { self.inner.authority(ctx) }
    fn planning_snapshot(&self, ctx: &armd::rt_core::ops::CommitCtx) -> PlanningSnapshot {
        self.inner.planning_snapshot(ctx)
    }
    fn apply_estop(&mut self) -> bool {
        let applied = self.inner.apply_estop();
        if applied {
            self.log.lock().unwrap().push("apply_estop");
        }
        applied
    }
    fn apply_stop(&mut self) {
        self.log.lock().unwrap().push("apply_stop");
        self.inner.apply_stop();
    }
    fn poll_hardware(&mut self, budget: &ServiceBudget) -> HardwarePoll {
        self.log.lock().unwrap().push("poll_hardware");
        self.inner.poll_hardware(budget)
    }
    fn poll_transition_wake(&mut self, budget: &ServiceBudget) -> HardwarePoll {
        self.log.lock().unwrap().push("poll_transition_wake");
        self.inner.poll_transition_wake(budget)
    }
    fn commit_primitive(
        &mut self,
        d: &armd::rt_core::broker::Descriptor,
        p: &Primitive,
        ctx: &armd::rt_core::ops::CommitCtx,
    ) -> Result<Committed, &'static str> {
        self.log.lock().unwrap().push("commit_primitive");
        self.inner.commit_primitive(d, p, ctx)
    }
    fn commit_immediate(
        &mut self,
        d: &armd::rt_core::broker::Descriptor,
        op: ImmediateOp,
        ctx: &armd::rt_core::ops::CommitCtx,
    ) -> Result<Committed, &'static str> {
        self.log.lock().unwrap().push("commit_immediate");
        self.inner.commit_immediate(d, op, ctx)
    }
    fn commit_transition(
        &mut self,
        d: &armd::rt_core::broker::Descriptor,
        op: TransitionOp,
        ctx: &armd::rt_core::ops::CommitCtx,
    ) -> Result<Committed, &'static str> {
        self.log.lock().unwrap().push("commit_transition");
        self.inner.commit_transition(d, op, ctx)
    }
    fn consume_target(&mut self, f: &TargetFrame, ctx: &armd::rt_core::ops::CommitCtx) -> Result<(), Refusal> {
        self.log.lock().unwrap().push("consume_target");
        self.inner.consume_target(f, ctx)
    }
    fn advance(&mut self, dt: f64, gen: u64) { self.inner.advance(dt, gen); }
    fn advance_output(&mut self, dt: f64, gen: u64) -> f64 {
        self.log.lock().unwrap().push("advance_output");
        self.inner.advance_output(dt, gen)
    }
    fn publish(&mut self, ts: f64, gen: u64) {
        self.published = true;
        self.log.lock().unwrap().push("publish");
        self.inner.publish(ts, gen);
        // Sample the broker reply queue INSIDE Step 6 `publish()` (before Step 7 runs) so a
        // reversed Step 7 -> Step 6 order is caught directly inside the release.
        if let Some(ref broker) = self.broker {
            broker.lock().unwrap().poll(|_, r| {
                self.replies_seen_during_publish.push(r);
                true
            });
        }
    }
    fn now_timestamp(&self) -> f64 {
        self.inner.now_timestamp()
    }
    fn action_outcome(&self, action: armd::rt_core::broker::ActionId) -> Option<Outcome> {
        self.log.lock().unwrap().push("action_outcome");
        assert!(self.published, "action_outcome must run strictly AFTER publish");
        self.inner.action_outcome(action)
    }
}

#[test]
fn exact_release_order_and_unconditional_checkpoints() {
    let log = Arc::new(Mutex::new(Vec::new()));
    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(StepConfirmDriver {
        dof: 7,
        enabled: false,
        pending: None,
    }));
    inner.lease_enabled = false;
    inner.tick(0.02);

    let (mut engine, broker, mut tw, stop) =
        ReleaseEngine::new(BootId(7), Capacities { commands: 4, plans: 2, replies: 8 });
    let broker = Arc::new(Mutex::new(broker));
    let mut core = OrderRecordingCore {
        inner,
        log: log.clone(),
        published: false,
        broker: Some(broker.clone()),
        replies_seen_during_publish: Vec::new(),
    };
    let log_hook = log.clone();
    engine.on_checkpoint(move |cp| {
        log_hook.lock().unwrap().push(match cp {
            Checkpoint::BeforeCommit => "cp:BeforeCommit",
            Checkpoint::BeforeTarget => "cp:BeforeTarget",
            Checkpoint::BeforeOutput => "cp:BeforeOutput",
        });
    });

    let now = Instant::now();
    let deadline = now + Duration::from_secs(5);
    let auth0 = core.authority(&engine.ctx(now));

    // Release 1: install a pending torque transition so Release 2 tests poll-boundary delivery before Step 3.
    let req_tr = broker
        .lock()
        .unwrap()
        .submit(ClientId(1), 1, auth0, deadline, Submit::Transition(TransitionOp::TorqueOn))
        .unwrap();
    let r1 = engine.step(&mut core, || now, 0.02, now + Duration::from_millis(20), 1);
    assert!(r1.command.unwrap().1.is_ok(), "{r1:?}");
    // Unconditional BeforeTarget fired even with no target queued!
    assert!(log.lock().unwrap().contains(&"cp:BeforeTarget"));
    broker.lock().unwrap().poll(|_, _| true);

    // Release 2: authorized stop at Step 1 + transition completes on Step 2 poll (arm becomes torqued!)
    // + 2 queued ordinary commands (at most 1 taken: short quintic finishing in 1 tick)
    // + 2 targets coalesced to 1 (consumed after BeforeTarget) + advance_output + publish + action_outcome.
    log.lock().unwrap().clear();
    core.published = false;
    core.replies_seen_during_publish.clear();
    stop.request();
    let auth1 = Authority { stop_generation: stop.current(), ..core.authority(&engine.ctx(now)) };
    let start = core.inner.commanded_positions.clone();
    let mut target_q = start.clone();
    target_q[0] += 0.001;
    let prim = Primitive::joint_quintic(&start, &target_q, 0.01).unwrap();
    let req_install = broker
        .lock()
        .unwrap()
        .submit(ClientId(1), 2, auth1, deadline, Submit::Install(&move |slot: &mut PlanSlot| slot.set(prim)))
        .unwrap();
    let req_imm = broker
        .lock()
        .unwrap()
        .submit(ClientId(1), 3, auth1, deadline, Submit::Immediate(ImmediateOp::Heartbeat))
        .unwrap();
    let frame = |seq| TargetFrame {
        arm: ArmId::LOCAL,
        boot: BootId(7),
        stream: StreamId(1),
        seq,
        authority: auth1,
        policy_generation: 0,
        input_at: now,
        vision_at: None,
        vision_frame_id: None,
        deadline,
        dof: 7,
        q: [0.0; MAX_DOF],
        replaceable: true,
    };
    tw.submit(frame(10));
    tw.submit(frame(11));

    let r2 = engine.step(&mut core, || now, 0.05, now + Duration::from_millis(20), 2);
    assert!(r2.stopped);
    assert_eq!(r2.command.unwrap().0, req_install, "at most one ordinary descriptor committed: {r2:?}");
    assert!(r2.command.unwrap().1.is_ok(), "{r2:?}");
    assert_eq!(r2.target.map(|t| t.0), Some(11), "latest coalesced target consumed");
    assert!(r2.transition_ended.is_some());

    assert_eq!(
        *log.lock().unwrap(),
        vec![
            "apply_stop",
            "poll_hardware",
            "cp:BeforeCommit",
            "commit_primitive",
            "cp:BeforeTarget",
            "consume_target",
            "cp:BeforeOutput",
            "advance_output",
            "publish",
            "action_outcome",
        ]
    );

    // Assert INSIDE the release (via `publish()` hook): Step 3's non-superseding `req_install` ACK
    // was already queued by Step 6, while `req_tr`'s transition terminal was NOT emitted yet.
    assert!(
        matches!(
            core.replies_seen_during_publish.as_slice(),
            [Reply::Ack { request, .. }] if *request == req_install
        ),
        "transition terminal must NOT be delivered before/during Step 6 publish: {:?}",
        core.replies_seen_during_publish
    );
    // Strictly AFTER Step 6 publish (in Step 7), `req_tr`'s transition terminal is delivered!
    let mut replies = Vec::new();
    broker.lock().unwrap().poll(|_, r| {
        replies.push(r);
        true
    });
    assert!(
        matches!(
            replies.as_slice(),
            [Reply::Terminal { request, outcome: Outcome::Completed, .. }] if *request == req_tr
        ),
        "transition terminal must be emitted strictly after Step 6 publish: {replies:?}"
    );

    // Also verify sub-release `service_wake` publishes strictly BEFORE delivering a transition terminal
    // by polling `w_broker` directly INSIDE `wake_core::publish()`!
    let (mut w_eng, w_broker, _w_tw, _w_stop) =
        ReleaseEngine::new(BootId(8), Capacities { commands: 4, plans: 2, replies: 8 });
    let w_broker = Arc::new(Mutex::new(w_broker));
    let mut wake_core = OrderRecordingCore {
        inner: {
            let mut i = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(StepConfirmDriver {
                dof: 7,
                enabled: false,
                pending: None,
            }));
            i.lease_enabled = false;
            i.tick(0.02);
            i
        },
        log: Arc::new(Mutex::new(Vec::new())),
        published: false,
        broker: Some(w_broker.clone()),
        replies_seen_during_publish: Vec::new(),
    };
    let w_auth = wake_core.authority(&w_eng.ctx(now));
    let w_req = w_broker
        .lock()
        .unwrap()
        .submit(ClientId(1), 1, w_auth, deadline, Submit::Transition(TransitionOp::TorqueOn))
        .unwrap();
    let mut w_rep = w_eng.step(&mut wake_core, || now, 0.02, now + Duration::from_millis(20), 1);
    assert!(w_rep.command.unwrap().1.is_ok());
    w_broker.lock().unwrap().poll(|_, _| true);
    wake_core.log.lock().unwrap().clear();
    wake_core.published = false;
    wake_core.replies_seen_during_publish.clear();
    let wake_published = w_eng.service_wake(&mut wake_core, now, now + Duration::from_millis(20), 2, &mut w_rep);
    assert!(wake_published, "completing wake must publish updated state before terminal");
    assert!(wake_core.published);
    assert_eq!(*wake_core.log.lock().unwrap(), vec!["poll_transition_wake", "publish"]);
    assert!(
        wake_core.replies_seen_during_publish.is_empty(),
        "service_wake transition terminal must NOT be emitted before publish(): {:?}",
        wake_core.replies_seen_during_publish
    );
    let mut wake_replies = Vec::new();
    w_broker.lock().unwrap().poll(|_, r| {
        wake_replies.push(r);
        true
    });
    assert!(
        matches!(wake_replies.as_slice(), [Reply::Terminal { request, outcome: Outcome::Completed, .. }] if *request == w_req),
        "{wake_replies:?}"
    );
    assert!(wake_core.inner.core.hub().latest().unwrap().obs.is_torqued);

    // Advance settle window (SETTLE_HOLD_S = 0.2s, servo_dt capped at 0.1s) so `req_install`
    // completes cleanly (`Outcome::Completed`) and `req_imm` is consumed on Release 3.
    let mut install_ended = r2.ended;
    let mut r3_cmd = None;
    for step_idx in 3..=6 {
        log.lock().unwrap().clear();
        core.published = false;
        core.replies_seen_during_publish.clear();
        let r = engine.step(&mut core, || now, 0.1, now + Duration::from_millis(20), step_idx);
        if step_idx == 3 {
            r3_cmd = r.command.map(|(req, _)| req);
        }
        if let Some(ended) = r.ended {
            install_ended = Some(ended);
            assert_eq!(log.lock().unwrap().last(), Some(&"action_outcome"));
            break;
        }
    }
    assert_eq!(r3_cmd, Some(req_imm), "second queued command must be taken on Release 3");
    assert!(matches!(install_ended, Some((_, Outcome::Completed))), "install must finish Completed: {install_ended:?}");
    replies.clear();
    broker.lock().unwrap().poll(|_, r| {
        replies.push(r);
        true
    });
    assert!(
        replies.iter().any(|r| matches!(r, Reply::Terminal { request, outcome: Outcome::Completed, .. } if *request == req_install)),
        "{replies:?}"
    );

    // Next release: E-stop AND authorized stop both pending -> `apply_estop` runs strictly BEFORE
    // `apply_stop` and `poll_hardware`.
    log.lock().unwrap().clear();
    core.published = false;
    core.inner.e_stop_latched.store(true, Ordering::SeqCst);
    core.inner.requested_estop_count.fetch_add(1, Ordering::SeqCst);
    stop.request();
    let r_es = engine.step(&mut core, || now, 0.02, now + Duration::from_millis(20), 7);
    assert!(r_es.stopped);
    let log_es = log.lock().unwrap().clone();
    assert_eq!(
        log_es,
        vec![
            "apply_estop",
            "apply_stop",
            "poll_hardware",
            "cp:BeforeCommit",
            "cp:BeforeTarget",
            "cp:BeforeOutput",
            "advance_output",
            "publish",
        ]
    );
}

// ============================================================================
// 5. Single budgeted RX pass, zero-RX begin_torque_budgeted, unsupported drivers
// ============================================================================

#[test]
fn single_budgeted_rx_pass_and_zero_rx_torque_begin_and_unsupported_serial() {
    let clock = MockClock::new();
    let raw_bus = FakeBus::with_clock(&dm_limits(), clock.clone());
    let bus = CountingBus::new(raw_bus.clone(), Some(clock.clone()));
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    raw_bus.with(|s| s.push_all_feedback());
    drv.read_joint_states().unwrap();
    drv.check_mapping_ranges();

    let mut inner = SupervisorInner::new(ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() })
        .with_driver(Box::new(drv));
    inner.lease_enabled = false;
    inner.tick(0.02);

    // (a) Queue 100 feedback frames -> Step 2 poll caps at 64 frames (`budget_exhausted == true`),
    // and Step 5 `tick_control` performs ZERO extra `can.receive()` calls!
    raw_bus.with(|s| {
        for _ in 0..15 {
            s.push_all_feedback(); // 15 * 7 = 105 frames
        }
    });
    bus.rx_calls.store(0, Ordering::SeqCst);
    let now = Instant::now();
    let poll = inner.poll_hardware(&ServiceBudget::for_release(now, now + Duration::from_millis(20)));
    let rx_after_poll = bus.rx_calls.load(Ordering::SeqCst);
    assert!(poll.rx_frames <= 64 && rx_after_poll <= 64, "poll={poll:?}, rx_calls={rx_after_poll}");
    assert!(poll.budget_exhausted, "expected budget_exhausted on 105 queued frames: {poll:?}");

    let _ts = inner.advance_output(0.02, 1);
    let rx_after_output = bus.rx_calls.load(Ordering::SeqCst);
    assert_eq!(rx_after_output, rx_after_poll, "Step 5 advance_output must reuse Step 2 poll with 0 extra RX");

    // (b) `begin_torque_budgeted(true, 0)` after an UN-DRAINED read enters `DrainBeforeEnable`
    // with ZERO `can.receive()` calls and ZERO enable TX frames until Step 2 drains!
    raw_bus.clear_log();
    let rx_before_begin = bus.rx_calls.load(Ordering::SeqCst);
    let (mut fx, mut broker, _tw, _stop) =
        ReleaseFixture::new(inner, BootId(9), Capacities { commands: 4, plans: 2, replies: 8 });
    // Put a fresh backlog so Step 2 ends with `last_read_drained == false`, then commit TorqueOn in Step 3.
    raw_bus.with(|s| {
        for _ in 0..12 {
            s.push_all_feedback();
        }
    });
    let a = fx.core.authority(&fx.ctx(fx.now()));
    broker
        .submit(ClientId(1), 1, a, fx.now() + Duration::from_secs(5), Submit::Transition(TransitionOp::TorqueOn))
        .unwrap();
    let rep = fx.step();
    assert!(rep.command.unwrap().1.is_ok(), "{rep:?}");
    assert!(rep.poll.budget_exhausted);
    let enable_tx_during_backlog = raw_bus
        .tx_log()
        .iter()
        .filter(|f| f.data == armd::protocol::damiao::CMD_ENABLE)
        .count();
    assert_eq!(
        enable_tx_during_backlog, 0,
        "begin_torque_budgeted(_, 0) with undrained RX must defer enable to DrainBeforeEnable"
    );
    assert!(bus.rx_calls.load(Ordering::SeqCst) - rx_before_begin <= 64);

    // Next releases drain the remaining backlog, send enables, and confirm!
    let mut ended = None;
    for _ in 0..25 {
        clock.advance(Duration::from_millis(20));
        let r = fx.step();
        if let Some((_, o)) = r.transition_ended {
            ended = Some(o);
            break;
        }
    }
    assert_eq!(ended, Some(Outcome::Completed));
    assert!(fx.core.is_torqued);

    // (c) Direct `poll_hardware` with `max_rx_frames = 1` caps RX at 1 frame even during a transition!
    raw_bus.with(|s| s.motors[0].torque_reply_delay = Duration::from_millis(200));
    let a2 = fx.core.authority(&fx.ctx(fx.now()));
    broker
        .submit(ClientId(1), 2, a2, fx.now() + Duration::from_secs(5), Submit::Transition(TransitionOp::TorqueOff))
        .unwrap();
    let r_off = fx.step();
    assert!(r_off.command.unwrap().1.is_ok());
    raw_bus.with(|s| s.push_all_feedback());
    bus.rx_calls.store(0, Ordering::SeqCst);
    let p_cap1 = fx.core.poll_hardware(&ServiceBudget {
        now: fx.now(),
        deadline: fx.now() + Duration::from_millis(5),
        max_rx_frames: 1,
    });
    assert!(p_cap1.rx_frames <= 1 && bus.rx_calls.load(Ordering::SeqCst) <= 1, "{p_cap1:?}");

    // (d) Direct zero-RX `begin_torque_budgeted(false, 0)` does 0 `can.receive()` calls.
    let mut drv2 = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    bus.rx_calls.store(0, Ordering::SeqCst);
    let _ = drv2.begin_torque_budgeted(false, 0).unwrap();
    assert_eq!(bus.rx_calls.load(Ordering::SeqCst), 0, "begin_torque_budgeted(false, 0) must do 0 RX");

    // (e) Serial drivers fail closed with UNSUPPORTED on budgeted methods.
    let mut ft = FeetechArmDriver::new(MockTransport::new(), vec![1, 2, 3, 4, 5, 6]);
    let mut fs = FashionStarArmDriver::new(MockTransport::new(), vec![1, 2, 3, 4, 5, 6]);
    let mut buf = Vec::new();
    let dl = Instant::now() + Duration::from_millis(5);
    assert!(ft.read_joint_states_budgeted(&mut buf, dl, 64).unwrap_err().to_string().contains("UNSUPPORTED"));
    assert!(ft.begin_torque_budgeted(true, 0).unwrap_err().to_string().contains("UNSUPPORTED"));
    assert!(ft.clear_fault_budgeted(dl).unwrap_err().to_string().contains("UNSUPPORTED"));
    assert!(fs.read_joint_states_budgeted(&mut buf, dl, 64).unwrap_err().to_string().contains("UNSUPPORTED"));
    assert!(fs.begin_torque_budgeted(true, 0).unwrap_err().to_string().contains("UNSUPPORTED"));
    assert!(fs.clear_fault_budgeted(dl).unwrap_err().to_string().contains("UNSUPPORTED"));
}

// ============================================================================
// 6. Release clock across overruns (`skipped_releases`) and sub-release wakes
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn release_clock_tracks_skipped_releases_and_preserves_real_dt_across_wakes() {
    let (drv, ..) = ThreadAuditDriver::new(7);
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;
    let delay = Arc::new(Mutex::new(None));
    let hooks = PersistTestHooks {
        release_delay_once: Some(delay.clone()),
        ..Default::default()
    };
    let handle = sup
        .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(10)), hooks)
        .await
        .unwrap();

    // Let 2 clean releases run, then inject a 45ms overrun (> 4 periods).
    wait_until_async(Duration::from_secs(1), || handle.stats().steps.load(Ordering::Relaxed) >= 2).await;
    *delay.lock().unwrap() = Some(Duration::from_millis(45));
    let steps_before = handle.stats().steps.load(Ordering::Relaxed);
    wait_until_async(Duration::from_secs(2), || {
        handle.stats().steps.load(Ordering::Relaxed) >= steps_before + 3
    })
    .await;

    let skipped = handle.stats().skipped_releases.load(Ordering::Relaxed);
    assert!(skipped >= 2, "expected skipped_releases >= 2 after 45ms overrun on 10ms period, got {skipped}");
    let samples = sup.tick_samples();
    assert!(
        samples.iter().any(|s| s.period_us >= 40_000),
        "expected real elapsed dt >= 40ms after overrun; samples={:?}",
        samples.iter().map(|s| s.period_us).collect::<Vec<_>>()
    );

    // Sub-release torque confirmation wake (confirm_delay = 14ms on a 10ms period -> fires ~4ms
    // into Release N+1 as a genuine sub-release wake!) must execute at least 1 measured wake
    // and NOT collapse the release `dt` spanning that wake!
    let wakes_before = handle.stats().wakes.load(Ordering::Relaxed);
    let steps_before_wake = handle.stats().steps.load(Ordering::Relaxed);
    let samples_before_wake = sup.tick_samples().len();
    let req = submit_broker_transition(&handle, 1, TransitionOp::TorqueOn);
    assert_eq!(wait_broker_terminal(&handle, req, Duration::from_secs(2)), Outcome::Completed);
    let wakes_after = handle.stats().wakes.load(Ordering::Relaxed);
    assert!(
        wakes_after > wakes_before,
        "expected a genuine sub-release wake (wakes {wakes_before} -> {wakes_after})"
    );
    let steps_at_terminal = handle.stats().steps.load(Ordering::Relaxed);
    let target_steps = steps_at_terminal + 2;
    let min_wake_samples = (target_steps.saturating_sub(steps_before_wake) as usize).max(2);
    wait_until_async(Duration::from_secs(1), || {
        handle.stats().steps.load(Ordering::Relaxed) >= target_steps
            && sup.tick_samples().len() >= samples_before_wake + min_wake_samples
    })
    .await;
    // Slice the exact non-empty diagnostic window produced strictly across the injected wake.
    let all_samples = sup.tick_samples();
    let recent: Vec<u32> = all_samples[samples_before_wake..]
        .iter()
        .map(|s| s.period_us)
        .collect();
    assert!(
        recent.len() >= min_wake_samples,
        "wake diagnostic window must be non-empty and cover the injected wake (got {} < {}): {recent:?}",
        recent.len(),
        min_wake_samples
    );
    assert!(
        recent.iter().all(|&p| p >= 7_000),
        "sub-release wake reset release dt below ~10ms period: {recent:?}"
    );
    handle.stop_and_join();
}

// ============================================================================
// 7. Lock-serialized per-waiter E-stop (> 64 epochs, handoff race, saturated maint)
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn estop_registry_pins_over_64_distinct_epochs_and_survives_handoff() {
    // (a) Concurrent Legacy -> Executor handoff with TWO E-stop callers waiting on `inner.write()`.
    let sup_handoff = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let hold = sup_handoff.inner().write().await;
    let s1 = sup_handoff.clone();
    let e1 = tokio::spawn(async move { s1.trigger_e_stop_checked().await });
    let s2 = sup_handoff.clone();
    let e2 = tokio::spawn(async move { s2.trigger_e_stop_checked().await });
    tokio::time::sleep(Duration::from_millis(15)).await;
    let s3 = sup_handoff.clone();
    let xfer = tokio::spawn(async move {
        s3.start_executor(ExecutorConfig::for_tests(Duration::from_millis(4))).await
    });
    tokio::time::sleep(Duration::from_millis(15)).await;
    drop(hold);

    let (r1, r2, h_res) = tokio::time::timeout(Duration::from_secs(3), async {
        tokio::join!(e1, e2, xfer)
    })
    .await
    .expect("handoff + concurrent E-stop callers must never hang");
    let (st1, ep1) = r1.unwrap().expect("first E-stop resolved cleanly");
    let (st2, ep2) = r2.unwrap().expect("second E-stop resolved cleanly");
    assert_eq!(st1, SafetyState::EStop);
    assert_eq!(st2, SafetyState::EStop);
    assert!(ep1 > 0 && ep2 >= ep1);
    let handle = h_res.unwrap().expect("executor transfer succeeded");

    // (b) 66 distinct applied E-stops (> ESTOP_ACK_CAPACITY = 64), each pinned to its exact epoch!
    let mut prev_epoch = ep2;
    for _ in 0..66 {
        assert_eq!(sup_handoff.clear_fault().await.unwrap(), SafetyState::Idle);
        let (st, ep) = sup_handoff.trigger_e_stop_checked().await.expect("E-stop pinned");
        assert_eq!(st, SafetyState::EStop);
        assert!(ep > prev_epoch, "epoch must strictly advance across distinct E-stops ({ep} <= {prev_epoch})");
        prev_epoch = ep;
    }

    // (c) E-stop succeeds promptly even while maintenance queue is actively busy.
    assert_eq!(sup_handoff.clear_fault().await.unwrap(), SafetyState::Idle);
    let mut maint_futs = Vec::new();
    for _ in 0..16 {
        let h = handle.clone();
        maint_futs.push(tokio::spawn(async move { h.call_maintenance_async(MaintenanceOp::Observation).await }));
    }
    let (st, ep) = tokio::time::timeout(Duration::from_secs(2), sup_handoff.trigger_e_stop_checked())
        .await
        .expect("E-stop must not stall behind maintenance traffic")
        .unwrap();
    assert_eq!(st, SafetyState::EStop);
    assert!(ep > prev_epoch);
    for f in maint_futs {
        let _ = f.await;
    }
    handle.stop_and_join();
}

// ============================================================================
// 8. Bounded `clear_fault_budgeted` semantics
// ============================================================================

#[test]
fn bounded_clear_fault_budgeted_deadlines_flush_estop_and_latch_preservation() {
    let clock = MockClock::new();
    let raw_bus = FakeBus::with_clock(&dm_limits(), clock.clone());

    // (a) Transport without bounded flush fails closed with UNSUPPORTED.
    let mut no_flush_bus = CountingBus::new(raw_bus.clone(), Some(clock.clone()));
    no_flush_bus.supports_flush = false;
    let mut drv_nf = DamiaoArmDriver::new(no_flush_bus, vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    assert!(
        drv_nf
            .clear_fault_budgeted(clock.now() + Duration::from_millis(50))
            .unwrap_err()
            .to_string()
            .contains("UNSUPPORTED")
    );

    // (b) Already-expired deadline returns Timeout immediately.
    let bus = CountingBus::new(raw_bus.clone(), Some(clock.clone()));
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    clock.advance(Duration::from_millis(10));
    let past = clock.now() - Duration::from_millis(1);
    assert!(matches!(drv.clear_fault_budgeted(past), Err(ArmDriverError::Transport(TransportError::Timeout))));

    // (c) Stalled final flush advancing MockClock past deadline returns Timeout.
    bus.stall_flush.store(true, Ordering::Release);
    let err_stall = drv.clear_fault_budgeted(clock.now() + Duration::from_millis(5));
    assert!(matches!(err_stall, Err(ArmDriverError::Transport(TransportError::Timeout))), "{err_stall:?}");
    bus.stall_flush.store(false, Ordering::Release);

    // (d) Mid-reset E-stop aborts remaining transmits with ESTOP_INHIBITED.
    let estop_req = Arc::new(std::sync::atomic::AtomicU64::new(0));
    let estop_applied = Arc::new(std::sync::atomic::AtomicU64::new(0));
    let mut bus_es = CountingBus::new(raw_bus.clone(), Some(clock.clone()));
    bus_es.estop_on_tx = Some((estop_req.clone(), 2));
    let tx_seen_es = bus_es.tx_seen.clone();
    let mut drv_es = DamiaoArmDriver::new(bus_es, vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv_es.set_estop_gate(armd::rt_core::urgent::EstopGate::new(estop_req, estop_applied));
    let err_es = drv_es.clear_fault_budgeted(clock.now() + Duration::from_millis(50)).unwrap_err();
    assert!(err_es.to_string().contains(armd::driver::ESTOP_INHIBITED), "{err_es:?}");
    assert_eq!(tx_seen_es.load(Ordering::Relaxed), 2, "must abort remaining transmits after E-stop on frame 2");

    // (e) Preserve fault latch after reset write until next fresh fault-free feedback poll!
    let raw_bus2 = FakeBus::with_clock(&dm_limits(), clock.clone());
    raw_bus2.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let mut drv2 = DamiaoArmDriver::new(raw_bus2.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv2.read_joint_states().unwrap();
    drv2.check_mapping_ranges();

    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv2));
    inner.lease_enabled = false;
    inner.set_torque(true).unwrap();
    // Inject motor 0 over-temperature error and poll -> latches hardware fault.
    raw_bus2.with(|s| {
        s.motors[0].state_override = Some(0x0A);
        s.push_all_feedback();
    });
    inner.tick(0.02);
    assert!(matches!(inner.safety.state(), SafetyState::Fault(_)), "{:?}", inner.safety.state());

    // Clear motor error on fake motor, then call `clear_fault_executor` BEFORE fresh feedback arrives:
    // the reset frame goes out, but cached status is still 0x0A -> MUST refuse clear until fresh poll!
    raw_bus2.with(|s| s.motors[0].state_override = None);
    let still_faulted = inner.clear_fault_executor(clock.now() + Duration::from_millis(50));
    assert!(
        still_faulted.is_err(),
        "clear_fault_executor must preserve fault latch until fresh fault-free feedback arrives: {still_faulted:?}"
    );
    assert!(matches!(inner.safety.state(), SafetyState::Fault(_)));

    // Push fresh fault-free feedback and poll -> now `clear_fault_executor` clears cleanly to Idle!
    raw_bus2.with(|s| s.push_all_feedback());
    let poll = inner.poll_hardware(&ServiceBudget::for_release(clock.now(), clock.now() + Duration::from_millis(20)));
    assert!(poll.feedback_valid);
    let cleared = inner.clear_fault_executor(clock.now() + Duration::from_millis(50));
    assert_eq!(cleared.unwrap(), SafetyState::Idle);
}

// ============================================================================
// 9. Parity across mock, SimArmDriver UDP loopback, and DamiaoArmDriver<FakeBus>
//    + router middleware allowlist
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn parity_sim_udp_loopback_and_damiao_and_router_allowlist() {
    // (a) SimArmDriver UDP loopback (`127.0.0.1:0`): Legacy vs Executor parity for publication,
    // torque confirmation, refused shutdown while torqued, recovery, and confirmed shutdown.
    let init_q = vec![0.05, -0.02, 0.02, -0.02, 0.02, -0.02, 0.0];
    let sim_leg = LoopbackSim::start(7, init_q.clone());
    let drv_leg = SimArmDriver::connect(&sim_leg.addr, 7).unwrap();
    let sup_leg = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_leg)));
    sup_leg.set_lease_enabled(false).await;
    for _ in 0..10 {
        sup_leg.control_tick(0.02).await;
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    sup_leg.set_torque(true).await.expect("legacy sim enable");
    let obs_leg = sup_leg.observation().await.unwrap();
    assert!((obs_leg.positions[0] - 0.05).abs() < 1e-3);
    assert!(obs_leg.is_torqued);
    assert!(!sup_leg.request_shutdown().await.unwrap());
    assert_eq!(sup_leg.clear_fault().await.unwrap(), SafetyState::Idle);
    sup_leg.set_torque(false).await.expect("legacy sim disable");
    assert!(sup_leg.request_shutdown().await.unwrap());

    // Same trace under Executor with SimArmDriver over UDP loopback (`127.0.0.1:0`).
    let sim_exec = LoopbackSim::start(7, init_q.clone());
    let drv_exec = SimArmDriver::connect(&sim_exec.addr, 7).unwrap();
    let sup_exec = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_exec)));
    sup_exec.set_lease_enabled(false).await;
    let h_sim = sup_exec
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(10)))
        .await
        .unwrap();
    // Wait until UDP state feedback arrives and publishes.
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        if let Ok(obs) = sup_exec.observation().await {
            if (obs.positions[0] - 0.05).abs() < 1e-3 {
                break;
            }
        }
        assert!(Instant::now() < deadline, "sim UDP feedback did not publish under executor");
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    let req_on = submit_broker_transition(&h_sim, 1, TransitionOp::TorqueOn);
    assert_eq!(wait_broker_terminal(&h_sim, req_on, Duration::from_secs(2)), Outcome::Completed);
    // Publication runs strictly BEFORE terminal delivery (both in Step 6->7 and sub-release wakes),
    // so the very first observation read after `Outcome::Completed` MUST already report `is_torqued == true`.
    assert!(sim_exec.enabled.load(Ordering::Acquire));
    let obs_exec = sup_exec.observation().await.expect("observation available immediately on terminal");
    assert!(obs_exec.is_torqued, "transition terminal was delivered before publishing is_torqued=true");
    assert_eq!(obs_exec.is_torqued, obs_leg.is_torqued);
    assert!((obs_exec.positions[0] - obs_leg.positions[0]).abs() < 1e-3);
    assert!(!sup_exec.request_shutdown().await.unwrap());
    assert!(!h_sim.is_exited());
    assert_eq!(sup_exec.clear_fault().await.unwrap(), SafetyState::Idle);
    let req_off = submit_broker_transition(&h_sim, 2, TransitionOp::TorqueOff);
    assert_eq!(wait_broker_terminal(&h_sim, req_off, Duration::from_secs(2)), Outcome::Completed);
    assert!(sup_exec.request_shutdown().await.unwrap());
    h_sim.shutdown_and_join().unwrap();
    assert!(h_sim.is_exited());

    // (b) DamiaoArmDriver<FakeBus> under Executor: enable failure, clear, confirmed enable,
    // refused shutdown -> recover -> disable -> confirmed shutdown on the SAME handle, plus
    // watchdog park on expired lease!
    let bus = FakeBus::new(&dm_limits());
    bus.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    dm.read_joint_states().unwrap();
    dm.check_mapping_ranges();
    let cfg = ArmConfig { lease_timeout_ms: 80, ..ArmConfig::rebot_dm() };
    let sup_dm = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(dm)));
    sup_dm.set_lease_enabled(false).await;
    let h_dm = sup_dm
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(10)))
        .await
        .unwrap();
    wait_until_async(Duration::from_secs(1), || h_dm.stats().steps.load(Ordering::Relaxed) >= 2).await;

    // Enable failure when motor 4 is silent.
    bus.with(|s| s.motors[3].silent = true);
    let r_fail = submit_broker_transition(&h_dm, 1, TransitionOp::TorqueOn);
    assert_eq!(wait_broker_terminal(&h_dm, r_fail, Duration::from_secs(2)), Outcome::Failed("ENABLE_FAILED"));
    bus.with(|s| {
        s.motors[3].silent = false;
        s.push_all_feedback();
    });
    wait_until_async(Duration::from_secs(1), || h_dm.stats().steps.load(Ordering::Relaxed) >= 6).await;
    assert_eq!(sup_dm.clear_fault().await.unwrap(), SafetyState::Idle);

    // Enable succeeds once motor 4 responds.
    let r_ok = submit_broker_transition(&h_dm, 2, TransitionOp::TorqueOn);
    assert_eq!(wait_broker_terminal(&h_dm, r_ok, Duration::from_secs(2)), Outcome::Completed);

    // Refused shutdown while torqued keeps executor live; recover + disable + retry confirms!
    assert!(!sup_dm.request_shutdown().await.unwrap());
    assert_eq!(h_dm.shutdown_and_join(), Err(OwnerError::ShutdownNotConfirmed));
    assert_eq!(sup_dm.clear_fault().await.unwrap(), SafetyState::Idle);
    let r_off = submit_broker_transition(&h_dm, 3, TransitionOp::TorqueOff);
    assert_eq!(wait_broker_terminal(&h_dm, r_off, Duration::from_secs(2)), Outcome::Completed);
    assert!(sup_dm.request_shutdown().await.unwrap());
    h_dm.shutdown_and_join().unwrap();

    // (c) Mock watchdog park parity: start BOTH Legacy and Executor from a genuinely moved pose
    // away from resting, verify both actively move along the park trajectory while torqued (never
    // immediately disabling at the start pose), and compare their final resting pose & state!
    let wd_cfg = ArmConfig { lease_timeout_ms: 40, ..ArmConfig::rebot_dm() };
    let resting = wd_cfg.resting_positions().to_vec();
    let max_dist = |a: &[f32], b: &[f32]| -> f32 {
        a.iter().zip(b).map(|(x, y)| (x - y).abs()).fold(0.0f32, f32::max)
    };
    assert!(
        max_dist(&INSPECT, &resting) > 0.5,
        "watchdog park test must start from a moved pose far from resting: start={INSPECT:?}, resting={resting:?}"
    );

    // Reference Legacy watchdog park trace from the moved pose.
    let sup_wd_leg = Arc::new(ArmSupervisor::new(wd_cfg.clone()));
    {
        let mut inner = sup_wd_leg.inner().write().await;
        inner.lease_enabled = true;
        inner.acquire_lease("wd-client").unwrap();
        inner.measured_positions = INSPECT.to_vec();
        inner.commanded_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        inner.set_torque(true).unwrap();
    }
    tokio::time::sleep(Duration::from_millis(60)).await;
    let mut saw_motion_leg = false;
    let mut final_leg = None;
    for _ in 0..400 {
        let obs = sup_wd_leg.tick(0.02).await;
        if obs.is_torqued && max_dist(&obs.positions, &INSPECT) > 0.1 {
            saw_motion_leg = true;
        }
        if !obs.is_torqued && obs.safety_state == SafetyState::Idle {
            final_leg = Some(obs);
            break;
        }
    }
    let final_leg = final_leg.expect("legacy watchdog park did not complete");
    assert!(saw_motion_leg, "legacy watchdog park must move away from start pose while torqued");
    assert!(
        max_dist(&final_leg.positions, &resting) < 0.02,
        "legacy watchdog park did not reach resting pose: got={:?}, resting={resting:?}",
        final_leg.positions
    );

    // Executor watchdog park trace from the exact same moved pose.
    let sup_wd = Arc::new(ArmSupervisor::new(wd_cfg));
    {
        let mut inner = sup_wd.inner().write().await;
        inner.lease_enabled = true;
        inner.acquire_lease("wd-client").unwrap();
        inner.measured_positions = INSPECT.to_vec();
        inner.commanded_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        inner.set_torque(true).unwrap();
    }
    let h_wd = sup_wd
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(10)))
        .await
        .unwrap();
    let wd_deadline = Instant::now() + Duration::from_secs(8);
    let mut saw_motion_exec = false;
    let mut last_obs = None;
    let final_exec = loop {
        if let Ok(obs) = sup_wd.observation().await {
            if obs.is_torqued && max_dist(&obs.positions, &INSPECT) > 0.1 {
                saw_motion_exec = true;
            }
            if !obs.is_torqued && obs.safety_state == SafetyState::Idle {
                break obs;
            }
            last_obs = Some((obs.is_torqued, obs.safety_state, obs.positions));
        }
        assert!(Instant::now() < wd_deadline, "watchdog park did not complete under executor; last_obs={last_obs:?}");
        tokio::time::sleep(Duration::from_millis(5)).await;
    };
    assert!(
        saw_motion_exec,
        "executor watchdog park disabled without moving along park trajectory from start pose"
    );
    assert!(
        max_dist(&final_exec.positions, &resting) < 0.02,
        "executor watchdog park did not reach resting pose: got={:?}, resting={resting:?}",
        final_exec.positions
    );
    assert!(
        max_dist(&final_exec.positions, &final_leg.positions) < 0.02,
        "executor watchdog park diverged from legacy trace: exec={:?}, legacy={:?}",
        final_exec.positions,
        final_leg.positions
    );

    // (d) Router middleware allowlist check in `OWNER_EXECUTOR`.
    let app = create_router_with_auth(sup_wd.clone(), OperatorAuth::disabled());
    let obs_resp = app
        .clone()
        .oneshot(Request::builder().uri("/api/v1/observation").body(Body::empty()).unwrap())
        .await
        .unwrap();
    assert_eq!(obs_resp.status(), StatusCode::OK);

    let diag_resp = app
        .clone()
        .oneshot(Request::builder().uri("/api/v1/diagnostics/tick").body(Body::empty()).unwrap())
        .await
        .unwrap();
    assert_eq!(diag_resp.status(), StatusCode::OK);

    let estop_resp = app
        .clone()
        .oneshot(Request::builder().method("POST").uri("/api/v1/safety/e_stop").body(Body::empty()).unwrap())
        .await
        .unwrap();
    assert_eq!(estop_resp.status(), StatusCode::OK);

    let torque_resp = app
        .clone()
        .oneshot(
            Request::builder()
                .method("POST")
                .uri("/api/v1/torque")
                .header("content-type", "application/json")
                .body(Body::from(r#"{"enable":true}"#))
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(torque_resp.status(), StatusCode::SERVICE_UNAVAILABLE);
    let body = axum::body::to_bytes(torque_resp.into_body(), 4096).await.unwrap();
    assert!(String::from_utf8_lossy(&body).contains("EXECUTOR_ROUTING_PENDING"), "{body:?}");

    // Stage 3 §2 A1: `/api/v1/ws` is allowed through the executor gate (for WS E-stop and
    // urgent Stop); a plain HTTP GET without WS upgrade headers fails WebSocket extraction
    // rather than `503 EXECUTOR_ROUTING_PENDING`.
    let ws_resp = app
        .oneshot(Request::builder().uri("/api/v1/ws").body(Body::empty()).unwrap())
        .await
        .unwrap();
    assert_ne!(ws_resp.status(), StatusCode::SERVICE_UNAVAILABLE);

    h_wd.stop_and_join();
}

// ============================================================================
// 10. Two-phase off-thread persistence (all outcomes + B1 + Persistence #5)
// ============================================================================

async fn prepare_zero_check_supervisor(path: std::path::PathBuf) -> (Arc<ArmSupervisor>, u64) {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let action_id = {
        let mut inner = sup.inner().write().await;
        inner.lease_enabled = false;
        inner.set_torque(true).unwrap();
        inner.measured_positions = INSPECT.to_vec();
        inner.commanded_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        inner.measured_positions[6] = -0.05;
        inner.commanded_positions[6] = -0.05;
        inner.target_positions[6] = -0.05;
        inner.gripper_calib_path = Some(path);
        inner.set_mock_gripper_stall_offset(Some(0.05));
        let epoch = inner.safety.epoch_id();
        let (id, _) = inner
            .start_gripper_zero_check(&json!({"threshold_rad": 0.10, "store_offset": true}), epoch)
            .unwrap();
        id
    };
    (sup, action_id)
}

async fn wait_action_terminal(handle: &RtExecutorHandle, action_id: u64) -> armd::motion::ActionStatus {
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        if let Ok(obs) = handle.observation_async().await {
            if let Some(a) = obs.action {
                if a.id == action_id && !a.is_running() {
                    return a;
                }
            }
        }
        assert!(Instant::now() < deadline, "action {action_id} did not terminate in time");
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn two_phase_persistence_success_and_dir_sync_unconfirmed() {
    // (a) Durable success.
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("gripper_calib.json");
    let (sup, id) = prepare_zero_check_supervisor(path.clone()).await;
    let h = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .unwrap();
    let a = wait_action_terminal(&h, id).await;
    assert_eq!(a.state, "done", "{a:?}");
    assert_eq!(a.durability_confirmed, Some(true), "{a:?}");
    assert!(path.exists());
    let file_json: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    assert!((file_json["offset_rad"].as_f64().unwrap() - 0.05).abs() < 1e-3);
    assert!(!h.planning_snapshot().unwrap().config_inhibited);
    h.stop_and_join();

    // (b) Dir-sync EIO -> DurableUnconfirmed (warning recorded, live state active, not reconciling).
    let dir2 = tempfile::tempdir().unwrap();
    let path2 = dir2.path().join("gripper_calib.json");
    let (sup2, id2) = prepare_zero_check_supervisor(path2.clone()).await;
    let hooks = PersistTestHooks {
        dir_sync_override: Some(Arc::new(|_| Err(std::io::Error::other("simulated EIO")))),
        ..Default::default()
    };
    let h2 = sup2
        .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks)
        .await
        .unwrap();
    let a2 = wait_action_terminal(&h2, id2).await;
    assert_eq!(a2.state, "done", "{a2:?}");
    assert_eq!(a2.durability_confirmed, Some(false), "{a2:?}");
    assert!(a2.durability_warning.as_deref().unwrap_or("").contains("simulated EIO"), "{a2:?}");
    assert!(path2.exists());
    assert!(!h2.planning_snapshot().unwrap().config_inhibited);
    h2.stop_and_join();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn two_phase_persistence_revocation_before_and_after_rename_and_disconnects() {
    // (c) Revocation BEFORE grant: hold `config_write_lock` so worker waits before staging/reserving,
    // trigger E-stop, release lock -> disk file untouched and hold Cleared (not Reconciling).
    let dir_c = tempfile::tempdir().unwrap();
    let path_c = dir_c.path().join("gripper_calib.json");
    let (sup_c, _id_c) = prepare_zero_check_supervisor(path_c.clone()).await;
    let cw_lock = sup_c.config_write_lock();
    let cw_guard = cw_lock.lock().await;
    let h_c = sup_c
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .unwrap();
    // Wait until zero_check reaches stall (checked) and pushes its persist job.
    let dl = Instant::now() + Duration::from_secs(3);
    loop {
        let obs = h_c.observation_async().await.unwrap();
        if obs.gripper_zero.as_ref().is_some_and(|g| g.state == "checked") {
            break;
        }
        assert!(Instant::now() < dl, "zero_check did not reach stall");
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    sup_c.trigger_e_stop_checked().await.unwrap();
    drop(cw_guard);
    tokio::time::sleep(Duration::from_millis(60)).await;
    assert!(!path_c.exists(), "revocation before grant must not rename file onto disk");
    assert!(!h_c.planning_snapshot().unwrap().config_inhibited, "hold must be Cleared, not Reconciling");
    h_c.stop_and_join();

    // (d) Post-rename revocation: barrier after grant before rename -> trigger E-stop -> allow rename ->
    // disk has file, live policy unchanged, hold enters Reconciling (`config_inhibited == true`).
    let dir_d = tempfile::tempdir().unwrap();
    let path_d = dir_d.path().join("gripper_calib.json");
    let (sup_d, _id_d) = prepare_zero_check_supervisor(path_d.clone()).await;
    let (at_barrier_tx, at_barrier_rx) = mpsc::sync_channel::<()>(1);
    let (go_tx, go_rx) = mpsc::sync_channel::<()>(1);
    let go_rx = Arc::new(Mutex::new(go_rx));
    let hooks_d = PersistTestHooks {
        after_grant_before_rename: Some(Arc::new(move || {
            let _ = at_barrier_tx.send(());
            let _ = go_rx.lock().unwrap().recv();
            false
        })),
        ..Default::default()
    };
    let h_d = sup_d
        .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks_d)
        .await
        .unwrap();
    at_barrier_rx.recv_timeout(Duration::from_secs(3)).unwrap();
    sup_d.trigger_e_stop_checked().await.unwrap();
    go_tx.send(()).unwrap();
    wait_until_async(Duration::from_secs(2), || {
        h_d.planning_snapshot().is_ok_and(|s| s.config_inhibited)
    })
    .await;
    assert!(path_d.exists(), "rename happened before post-rename revocation");
    h_d.stop_and_join();

    // (e) Worker disconnect BEFORE rename -> hold enters Reconciling, action fails PERSIST_FAILED.
    let dir_e = tempfile::tempdir().unwrap();
    let path_e = dir_e.path().join("gripper_calib.json");
    let (sup_e, id_e) = prepare_zero_check_supervisor(path_e.clone()).await;
    let hooks_e = PersistTestHooks {
        after_grant_before_rename: Some(Arc::new(|| true)),
        ..Default::default()
    };
    let h_e = sup_e
        .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks_e)
        .await
        .unwrap();
    let a_e = wait_action_terminal(&h_e, id_e).await;
    assert_eq!(a_e.state, "failed", "{a_e:?}");
    assert_eq!(a_e.code.as_deref(), Some("PERSIST_FAILED"));
    assert!(h_e.planning_snapshot().unwrap().config_inhibited);
    assert!(!path_e.exists());
    h_e.stop_and_join();

    // (f) Worker disconnect AFTER rename -> hold enters Reconciling, action fails PERSIST_FAILED.
    let dir_f = tempfile::tempdir().unwrap();
    let path_f = dir_f.path().join("gripper_calib.json");
    let (sup_f, id_f) = prepare_zero_check_supervisor(path_f.clone()).await;
    let hooks_f = PersistTestHooks {
        after_rename_before_activate: Some(Arc::new(|| true)),
        ..Default::default()
    };
    let h_f = sup_f
        .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks_f)
        .await
        .unwrap();
    let a_f = wait_action_terminal(&h_f, id_f).await;
    assert_eq!(a_f.state, "failed", "{a_f:?}");
    assert_eq!(a_f.code.as_deref(), Some("PERSIST_FAILED"));
    assert!(h_f.planning_snapshot().unwrap().config_inhibited);
    assert!(path_f.exists());
    h_f.stop_and_join();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn two_phase_persistence_b1_race_persistence5_and_saturation() {
    // (g1) B1 barrier after `is_abandoned()` and before `pop()` with queued `Activate` + worker exit:
    // MUST pop and activate cleanly (single terminal `done`, `config_inhibited == false`).
    let dir_g1 = tempfile::tempdir().unwrap();
    let path_g1 = dir_g1.path().join("gripper_calib.json");
    let (sup_g1, id_g1) = prepare_zero_check_supervisor(path_g1.clone()).await;
    let renamed = Arc::new(AtomicBool::new(false));
    let released_worker = Arc::new(AtomicBool::new(false));
    let (go_push_tx, go_push_rx) = mpsc::sync_channel::<()>(1);
    let go_push_rx = Arc::new(Mutex::new(go_push_rx));
    let renamed_w = renamed.clone();
    let renamed_c = renamed.clone();
    let released_c = released_worker.clone();
    let hooks_g1 = PersistTestHooks {
        after_rename_before_activate: Some(Arc::new(move || {
            renamed_w.store(true, Ordering::Release);
            let _ = go_push_rx.lock().unwrap().recv();
            false
        })),
        after_abandoned_check_before_pop: Some(Arc::new(move || {
            if renamed_c.load(Ordering::Acquire) && !released_c.swap(true, Ordering::AcqRel) {
                let _ = go_push_tx.send(());
                // Give worker a moment to push `Activate` and exit (`cmd_tx` dropped).
                thread::sleep(Duration::from_millis(15));
            }
        })),
        exit_after_first_job: true,
        leave_final_reply_queued: true,
        ..Default::default()
    };
    let h_g1 = sup_g1
        .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks_g1)
        .await
        .unwrap();
    let a_g1 = wait_action_terminal(&h_g1, id_g1).await;
    assert_eq!(a_g1.state, "done", "B1 Activate must not double-fail or reconcile: {a_g1:?}");
    wait_until_async(Duration::from_secs(1), || h_g1.persist_worker_exited()).await;
    // Even after worker exits and subsequent releases see `is_abandoned() == true`, hold stays clean!
    tokio::time::sleep(Duration::from_millis(25)).await;
    assert!(!h_g1.planning_snapshot().unwrap().config_inhibited);
    h_g1.stop_and_join();

    // (g2) B1 with queued `Abort` + worker exit (target path is an existing directory so rename fails):
    // single terminal `PERSIST_FAILED`, hold released (`config_inhibited == false`, NOT Reconciling!).
    let dir_g2 = tempfile::tempdir().unwrap();
    let target_is_dir = dir_g2.path().join("existing_dir");
    std::fs::create_dir(&target_is_dir).unwrap();
    let (sup_g2, id_g2) = prepare_zero_check_supervisor(target_is_dir).await;
    let hooks_g2 = PersistTestHooks {
        exit_after_first_job: true,
        leave_final_reply_queued: true,
        ..Default::default()
    };
    let h_g2 = sup_g2
        .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks_g2)
        .await
        .unwrap();
    let a_g2 = wait_action_terminal(&h_g2, id_g2).await;
    assert_eq!(a_g2.state, "failed", "{a_g2:?}");
    assert_eq!(a_g2.code.as_deref(), Some("PERSIST_FAILED"));
    wait_until_async(Duration::from_secs(1), || h_g2.persist_worker_exited()).await;
    tokio::time::sleep(Duration::from_millis(25)).await;
    assert!(!h_g2.planning_snapshot().unwrap().config_inhibited, "Abort before rename must release hold cleanly");
    h_g2.stop_and_join();

    // (h) Persistence #5 (`fail_persisted_once`) + off-thread drop of unused reply payload!
    let dir_h = tempfile::tempdir().unwrap();
    let path_h = dir_h.path().join("gripper_calib.json");
    let (sup_h, id_h) = prepare_zero_check_supervisor(path_h).await;
    let dropped_on = Arc::new(Mutex::new(Vec::new()));
    let probe: Box<dyn std::any::Any + Send> = Box::new(DropProbe { dropped_on: dropped_on.clone() });
    let hooks_h = PersistTestHooks {
        fail_persisted_once: Some(Arc::new(AtomicBool::new(true))),
        exit_after_first_job: true,
        leave_final_reply_queued: true,
        reply_drop_probe: Some(Arc::new(Mutex::new(Some(probe)))),
        ..Default::default()
    };
    let h_h = sup_h
        .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks_h)
        .await
        .unwrap();
    let exec_tid = h_h.executor_thread_id();
    let a_h = wait_action_terminal(&h_h, id_h).await;
    assert_eq!(a_h.state, "failed", "{a_h:?}");
    assert_eq!(a_h.code.as_deref(), Some("PERSIST_FAILED"));
    assert!(h_h.planning_snapshot().unwrap().config_inhibited, "Persistence #5 must enter Reconciling");
    // Orphaned reply (left queued when worker exited) is drained & dropped off-thread by dispatcher/teardown!
    wait_until_async(Duration::from_secs(2), || !dropped_on.lock().unwrap().is_empty()).await;
    let tids = dropped_on.lock().unwrap().clone();
    assert_eq!(tids.len(), 1);
    assert_ne!(tids[0], exec_tid, "PersistReply probe was dropped on armd-control!");
    h_h.stop_and_join();

    // (i) Persistence saturation / unavailable fails immediately with PERSIST_FAILED and no stranding.
    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
    inner.lease_enabled = false;
    inner.set_torque(true).unwrap();
    inner.measured_positions = INSPECT.to_vec();
    inner.commanded_positions = INSPECT.to_vec();
    inner.target_positions = INSPECT.to_vec();
    inner.measured_positions[6] = -0.05;
    inner.commanded_positions[6] = -0.05;
    inner.target_positions[6] = -0.05;
    inner.set_mock_gripper_stall_offset(Some(0.05));
    inner.defer_zero_check_persist = true;
    inner.persist_capacity_available = false;
    let rev_before = ConfigAuthority::revision(&inner, ConfigDomain::GripperCalibration);
    let ep = inner.safety.epoch_id();
    let (sat_id, _) = inner
        .start_gripper_zero_check(&json!({"threshold_rad": 0.10, "store_offset": true}), ep)
        .unwrap();
    for _ in 0..200 {
        inner.tick(0.02);
        if inner.last_action.as_ref().is_some_and(|a| a.id == sat_id && !a.is_running()) {
            break;
        }
    }
    let sat_a = inner.last_action.as_ref().unwrap();
    assert_eq!(sat_a.state, "failed", "{sat_a:?}");
    assert_eq!(sat_a.code.as_deref(), Some("PERSIST_FAILED"));
    assert!(inner.zero_check_persisting.is_none());
    assert!(inner.pending_zero_check_persist.is_none());
    assert_eq!(ConfigAuthority::revision(&inner, ConfigDomain::GripperCalibration), rev_before);
}

// ============================================================================
// 11. Off-thread reclamation (> 30 ops) and diagnostics under paused dispatcher
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn off_thread_reclamation_and_diagnostics_under_paused_dispatcher() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(2)))
        .await
        .unwrap();
    let exec_tid = handle.executor_thread_id();
    // Wait until dispatcher has started and served at least one sample.
    wait_until_async(Duration::from_secs(1), || handle.stats().steps.load(Ordering::Relaxed) >= 5).await;

    // Pause `armd-dispatch` by holding `hub.reader_guard()` on a helper thread.
    let (held_tx, held_rx) = mpsc::sync_channel::<()>(1);
    let (rel_tx, rel_rx) = mpsc::sync_channel::<()>(1);
    let hub = sup.hub().clone();
    let pauser = thread::spawn(move || {
        let _g = hub.reader_guard();
        let _ = held_tx.send(());
        let _ = rel_rx.recv();
    });
    held_rx.recv_timeout(Duration::from_secs(2)).unwrap();

    // Issue 32 maintenance calls with heap/DropProbe payloads while dispatcher is paused!
    // Must never deadlock and EVERY DropProbe must drop on a thread != `exec_tid`.
    let dropped_on = Arc::new(Mutex::new(Vec::new()));
    for i in 0..32 {
        let probe = Box::new(DropProbe { dropped_on: dropped_on.clone() });
        handle.probe_owned_payload(probe).expect("probe maintenance op succeeds while dispatcher paused");
        if i % 4 == 0 {
            handle.set_joint_zero_offsets(vec![0.0; 7]).unwrap().unwrap();
        }
    }
    let tids = dropped_on.lock().unwrap().clone();
    assert_eq!(tids.len(), 32);
    assert!(
        tids.iter().all(|&tid| tid != exec_tid),
        "maintenance payload dropped on armd-control ({exec_tid:?}): {tids:?}"
    );

    // Let >= 320 releases run while dispatcher is paused -> 256-slot diag ring overflows cleanly!
    let start_steps = handle.stats().steps.load(Ordering::Relaxed);
    wait_until_async(Duration::from_secs(3), || {
        handle.stats().steps.load(Ordering::Relaxed) >= start_steps + 320
    })
    .await;
    assert!(
        handle.stats().diag_dropped.load(Ordering::Relaxed) > 0,
        "expected diag_dropped > 0 while dispatcher paused past 256-slot ring"
    );

    // Unpause dispatcher and verify diagnostics drain into `TickStats`.
    rel_tx.send(()).unwrap();
    pauser.join().unwrap();
    wait_until_async(Duration::from_secs(2), || sup.tick_stats().window > 0).await;
    assert!(!sup.tick_samples().is_empty());

    handle.stop_and_join();
}

// ============================================================================
// 12. CLI startup gate smoke test (`ARMD_RT_OWNER=executor`)
// ============================================================================

#[tokio::test]
async fn cli_startup_gate_allows_mock_and_rejects_non_mock_and_invalid_env() {
    let run_fail = |env_val: &str, follower: &str| -> String {
        let out = Command::new(env!("CARGO_BIN_EXE_armd"))
            .args(["--follower", follower, "--port", "0", "--no-operator-auth"])
            .env("ARMD_RT_OWNER", env_val)
            .env_remove("PORT")
            .output()
            .unwrap();
        assert!(!out.status.success(), "expected failure for ARMD_RT_OWNER={env_val} --follower {follower}");
        format!("{}{}", String::from_utf8_lossy(&out.stdout), String::from_utf8_lossy(&out.stderr))
    };

    assert!(run_fail("executor", "sim").contains("gated to '--follower mock'"));
    assert!(run_fail("executor", "rebot-dm").contains("gated to '--follower mock'"));
    assert!(run_fail("invalid_owner", "mock").contains("ARMD_RT_OWNER"));

    // Positive smoke: `ARMD_RT_OWNER=executor --follower mock` starts on an ephemeral port,
    // serves `/api/v1/observation` (200), gates `/api/v1/torque` (503), and exits cleanly on SIGTERM.
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    assert!(port != 18765 && port != 18001 && port != 18990);
    drop(listener);

    let temp_dir = tempfile::tempdir().unwrap();
    let mut child = Command::new(env!("CARGO_BIN_EXE_armd"))
        .args([
            "--follower",
            "mock",
            "--port",
            &port.to_string(),
            "--no-operator-auth",
            "--log-dir",
            temp_dir.path().to_str().unwrap(),
            "--blackbox-dir",
            temp_dir.path().to_str().unwrap(),
        ])
        .env("ARMD_RT_OWNER", "executor")
        .env_remove("PORT")
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();

    let client = reqwest::Client::builder()
        .timeout(Duration::from_millis(500))
        .build()
        .unwrap();
    let obs_url = format!("http://127.0.0.1:{port}/api/v1/observation");
    let torque_url = format!("http://127.0.0.1:{port}/api/v1/torque");

    let ready_deadline = Instant::now() + Duration::from_secs(5);
    let mut ready = false;
    while Instant::now() < ready_deadline {
        if let Ok(resp) = client.get(&obs_url).send().await {
            if resp.status() == reqwest::StatusCode::OK {
                ready = true;
                break;
            }
        }
        if let Some(status) = child.try_wait().unwrap() {
            panic!("armd exited early with {status}");
        }
        tokio::time::sleep(Duration::from_millis(25)).await;
    }
    assert!(ready, "armd with ARMD_RT_OWNER=executor did not serve /api/v1/observation");

    let torque_resp = client.post(&torque_url).json(&json!({"enable": true})).send().await.unwrap();
    assert_eq!(torque_resp.status(), reqwest::StatusCode::SERVICE_UNAVAILABLE);
    assert!(torque_resp.text().await.unwrap().contains("EXECUTOR_ROUTING_PENDING"));

    // Send SIGTERM and verify clean shutdown exit (0).
    // SAFETY: valid PID of child process spawned above.
    unsafe {
        libc::kill(child.id() as libc::pid_t, libc::SIGTERM);
    }
    wait_until_async(Duration::from_secs(5), || child.try_wait().unwrap().is_some()).await;
    let status = child.wait().unwrap();
    assert!(status.success(), "armd executor did not exit cleanly on SIGTERM: {status}");
}

// ============================================================================
// 13. Safety, non-blocking torque, auth-before-rename, off-thread drops & legacy E-stop regressions
// ============================================================================

#[test]
fn estop_aborts_pending_damiao_enable_without_tx_and_preserves_torque_state() {
    let clock = MockClock::new();

    // (a) E-stop during `DrainBeforeEnable`: ZERO `CMD_ENABLE` frames transmitted,
    // transition ends `Outcome::Failed("ESTOP_ACTIVE")`, `is_torqued == false`.
    let raw_bus_a = FakeBus::with_clock(&dm_limits(), clock.clone());
    raw_bus_a.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let mut drv_a = DamiaoArmDriver::new(raw_bus_a.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv_a.read_joint_states().unwrap();
    drv_a.check_mapping_ranges();
    let mut inner_a = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_a));
    inner_a.lease_enabled = false;
    inner_a.rt_executor_owned = true;
    inner_a.tick(0.02);

    let (mut fx_a, mut broker_a, _tw_a, _stop_a) =
        ReleaseFixture::new(inner_a, BootId(31), Capacities { commands: 4, plans: 2, replies: 8 });
    raw_bus_a.clear_log();
    // Queue backlog so Step 2 leaves RX undrained -> Step 3 enters `DrainBeforeEnable` with 0 enable TX.
    raw_bus_a.with(|s| {
        for _ in 0..12 {
            s.push_all_feedback();
        }
    });
    let auth_a = fx_a.core.authority(&fx_a.ctx(fx_a.now()));
    let req_a = broker_a
        .submit(ClientId(1), 1, auth_a, fx_a.now() + Duration::from_secs(5), Submit::Transition(TransitionOp::TorqueOn))
        .unwrap();
    let r1 = fx_a.step();
    assert!(r1.command.unwrap().1.is_ok());
    assert_eq!(
        raw_bus_a.tx_log().iter().filter(|f| f.data == armd::protocol::damiao::CMD_ENABLE).count(),
        0
    );
    // Latch E-stop BEFORE the next release drains the backlog.
    fx_a.core.e_stop_latched.store(true, Ordering::SeqCst);
    fx_a.core.requested_estop_count.fetch_add(1, Ordering::SeqCst);
    let r2 = fx_a.step();
    assert_eq!(r2.transition_ended, Some((armd::rt_core::broker::ActionId(req_a.0), Outcome::Failed("ESTOP_ACTIVE"))));
    assert_eq!(
        raw_bus_a.tx_log().iter().filter(|f| f.data == armd::protocol::damiao::CMD_ENABLE).count(),
        0,
        "E-stop during DrainBeforeEnable must transmit ZERO CMD_ENABLE frames"
    );
    assert_eq!(*fx_a.core.safety.state(), SafetyState::EStop);
    assert!(!fx_a.core.is_torqued, "arm was never enabled; is_torqued stays false");

    // (b) E-stop AFTER `CMD_ENABLE` transmitted and motors enabled, BEFORE confirmation poll finishes:
    // transition ends `Outcome::Failed("ESTOP_ACTIVE")`, zero extra enable frames, AND honest
    // physical torque state (`is_torqued == true`, `pending_rearm == true`) is preserved!
    let raw_bus_b = FakeBus::with_clock(&dm_limits(), clock.clone());
    raw_bus_b.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let mut drv_b = DamiaoArmDriver::new(raw_bus_b.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv_b.read_joint_states().unwrap();
    drv_b.check_mapping_ranges();
    let mut inner_b = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_b));
    inner_b.lease_enabled = false;
    inner_b.rt_executor_owned = true;
    inner_b.tick(0.02);

    let (mut fx_b, mut broker_b, _tw_b, _stop_b) =
        ReleaseFixture::new(inner_b, BootId(32), Capacities { commands: 4, plans: 2, replies: 8 });
    // Delay torque reply beyond release 1 so `begin_torque_budgeted` sends CMD_ENABLE and stays Pending.
    raw_bus_b.with(|s| {
        for m in &mut s.motors {
            m.torque_reply_delay = Duration::from_millis(50);
        }
    });
    raw_bus_b.clear_log();
    let auth_b = fx_b.core.authority(&fx_b.ctx(fx_b.now()));
    let req_b = broker_b
        .submit(ClientId(1), 1, auth_b, fx_b.now() + Duration::from_secs(5), Submit::Transition(TransitionOp::TorqueOn))
        .unwrap();
    let rb1 = fx_b.step();
    assert!(rb1.command.unwrap().1.is_ok());
    let tx_after_begin = raw_bus_b
        .tx_log()
        .iter()
        .filter(|f| f.data == armd::protocol::damiao::CMD_ENABLE)
        .count();
    assert_eq!(tx_after_begin, 7, "all 7 motors received initial CMD_ENABLE");

    // Motors physically enabled (replies queued after 50ms), then E-stop triggers before confirmation!
    clock.advance(Duration::from_millis(60));
    fx_b.core.e_stop_latched.store(true, Ordering::SeqCst);
    fx_b.core.requested_estop_count.fetch_add(1, Ordering::SeqCst);
    let rb2 = fx_b.step();
    assert_eq!(rb2.transition_ended, Some((armd::rt_core::broker::ActionId(req_b.0), Outcome::Failed("ESTOP_ACTIVE"))));
    assert_eq!(
        raw_bus_b.tx_log().iter().filter(|f| f.data == armd::protocol::damiao::CMD_ENABLE).count(),
        tx_after_begin,
        "no further CMD_ENABLE frames allowed after E-stop"
    );
    assert_eq!(*fx_b.core.safety.state(), SafetyState::EStop);
    assert!(fx_b.core.is_torqued, "honest physical torque state must stay true after partial/complete enable + E-stop");
    assert!(fx_b.core.pending_rearm);

    // (c) Concurrent `e_stop_latched` inside `send_enable` TX loop aborts remaining motors with ESTOP_ACTIVE
    // and does NOT mutate safety state into Fault(EnableFailed).
    let raw_bus_c2 = FakeBus::with_clock(&dm_limits(), clock.clone());
    raw_bus_c2.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let mut inner_c2 = SupervisorInner::new(ArmConfig::rebot_dm());
    inner_c2.lease_enabled = false;
    inner_c2.rt_executor_owned = true;
    let mut bus_c2 = CountingBus::new(raw_bus_c2.clone(), Some(clock.clone()));
    bus_c2.estop_on_tx = Some((inner_c2.requested_estop_count.clone(), 3));
    let mut drv_c2 = DamiaoArmDriver::new(bus_c2.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv_c2.read_joint_states().unwrap();
    drv_c2.check_mapping_ranges();
    // Reset counters after setup range-check TXs so E-stop triggers on the 3rd CMD_ENABLE frame.
    bus_c2.tx_seen.store(0, Ordering::SeqCst);
    inner_c2.requested_estop_count.store(0, Ordering::SeqCst);
    inner_c2 = inner_c2.with_driver(Box::new(drv_c2));
    inner_c2.tick(0.02);
    bus_c2.tx_seen.store(0, Ordering::SeqCst);
    inner_c2.requested_estop_count.store(0, Ordering::SeqCst);
    raw_bus_c2.clear_log();
    let (mut fx_c, mut broker_c, _tw_c, _stop_c) =
        ReleaseFixture::new(inner_c2, BootId(33), Capacities { commands: 4, plans: 2, replies: 8 });
    let auth_c = fx_c.core.authority(&fx_c.ctx(fx_c.now()));
    broker_c
        .submit(ClientId(1), 1, auth_c, fx_c.now() + Duration::from_secs(5), Submit::Transition(TransitionOp::TorqueOn))
        .unwrap();
    let rc1 = fx_c.step();
    assert_eq!(rc1.command.unwrap().1, Err("ESTOP_ACTIVE"));
    assert_eq!(
        raw_bus_c2.tx_log().iter().filter(|f| f.data == armd::protocol::damiao::CMD_ENABLE).count(),
        3,
        "must stop transmitting CMD_ENABLE immediately when E-stop latches on frame 3"
    );
    assert!(!matches!(fx_c.core.safety.state(), SafetyState::Fault(_)), "must not fault EnableFailed on E-stop");
    assert!(fx_c.core.is_torqued, "partial enable before E-stop must preserve is_torqued=true");
}

#[test]
fn rt_torque_paths_refuse_blocking_immediate_and_use_budgeted_disable_cleanup() {
    let clock = MockClock::new();
    let raw_bus = FakeBus::with_clock(&dm_limits(), clock.clone());
    raw_bus.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let bus = CountingBus::new(raw_bus.clone(), Some(clock.clone()));
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv.read_joint_states().unwrap();
    drv.check_mapping_ranges();

    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv));
    inner.lease_enabled = false;
    inner.rt_executor_owned = true;
    inner.tick(0.02);

    let (mut fx, mut broker, _tw, _stop) =
        ReleaseFixture::new(inner, BootId(34), Capacities { commands: 4, plans: 2, replies: 8 });

    // (a) ImmediateOp::TorqueOn / TorqueOff refused with UNSUPPORTED when driver is attached (0 TX/RX).
    raw_bus.clear_log();
    let auth = fx.core.authority(&fx.ctx(fx.now()));
    broker
        .submit(ClientId(1), 1, auth, fx.now() + Duration::from_secs(5), Submit::Immediate(ImmediateOp::TorqueOn))
        .unwrap();
    let r_on = fx.step();
    assert_eq!(r_on.command.unwrap().1, Err("UNSUPPORTED"));
    broker
        .submit(ClientId(1), 2, auth, fx.now() + Duration::from_secs(5), Submit::Immediate(ImmediateOp::TorqueOff))
        .unwrap();
    let r_off = fx.step();
    assert_eq!(r_off.command.unwrap().1, Err("UNSUPPORTED"));
    assert_eq!(raw_bus.tx_log().len(), 0, "refused ImmediateOp torque must transmit 0 bus frames");

    // (b) Enable failure uses non-blocking `enable_failed_budgeted` (no blocking read_joint_states /
    // sleep) and completes background disable through Step 2's single RX pass!
    raw_bus.with(|s| s.motors[3].silent = true);
    let auth_fail = fx.core.authority(&fx.ctx(fx.now()));
    broker
        .submit(ClientId(1), 3, auth_fail, fx.now() + Duration::from_secs(5), Submit::Transition(TransitionOp::TorqueOn))
        .unwrap();
    let r_start = fx.step();
    assert!(r_start.command.unwrap().1.is_ok());

    // Advance past Damiao enable confirmation timeout -> transition fails without extra blocking RX!
    let mut failed_outcome = None;
    for _ in 0..30 {
        clock.advance(Duration::from_millis(20));
        bus.rx_calls.store(0, Ordering::SeqCst);
        let r = fx.step();
        assert!(bus.rx_calls.load(Ordering::SeqCst) <= 64, "step exceeded single RX budget");
        if let Some((_, o)) = r.transition_ended {
            failed_outcome = Some(o);
            break;
        }
    }
    assert_eq!(failed_outcome, Some(Outcome::Failed("ENABLE_FAILED")));
    assert!(fx.core.disable_pending, "unconfirmed disable after enable failure must stay disable_pending");

    // Motor 4 recovers and sends fresh disabled feedback -> Step 2's `service_background_disable`
    // confirms disable cleanly within the single RX budget!
    raw_bus.with(|s| {
        s.motors[3].silent = false;
        s.push_all_feedback();
    });
    for _ in 0..10 {
        clock.advance(Duration::from_millis(20));
        bus.rx_calls.store(0, Ordering::SeqCst);
        let _ = fx.step();
        assert!(bus.rx_calls.load(Ordering::SeqCst) <= 64);
        if !fx.core.disable_pending && !fx.core.is_torqued {
            break;
        }
    }
    assert!(!fx.core.disable_pending && !fx.core.is_torqued, "background disable must confirm in Step 2");

    // (c) Thermal trip under `rt_executor_owned == true` triggers non-blocking `request_disable()`
    // with 0 RX in Step 5 and finishes disable confirmation in Step 2!
    let cleared = fx.core.clear_fault_executor(fx.now() + Duration::from_millis(50));
    assert_eq!(cleared.unwrap(), SafetyState::Idle);
    let auth_re = fx.core.authority(&fx.ctx(fx.now()));
    broker
        .submit(ClientId(1), 4, auth_re, fx.now() + Duration::from_secs(5), Submit::Transition(TransitionOp::TorqueOn))
        .unwrap();
    for _ in 0..10 {
        clock.advance(Duration::from_millis(20));
        let r = fx.step();
        if r.transition_ended.is_some() {
            break;
        }
    }
    assert!(fx.core.is_torqued);

    // Inject over-temperature on motor 0 -> Step 5 triggers ThermalTrip -> non-blocking request_disable().
    clock.advance(Duration::from_millis(20));
    raw_bus.with(|s| s.push_all_feedback());
    let _ = fx.core.poll_hardware(&ServiceBudget::for_release(fx.now(), fx.now() + Duration::from_millis(20)));
    fx.core.temperatures[0] = 95.0;
    bus.rx_calls.store(0, Ordering::SeqCst);
    let _ = fx.core.advance_output(0.02, 99);
    assert_eq!(
        bus.rx_calls.load(Ordering::SeqCst),
        0,
        "ThermalTrip request_disable under rt_executor_owned must perform 0 RX calls"
    );
    assert!(matches!(fx.core.safety.state(), SafetyState::Fault(_)));
    raw_bus.with(|s| s.push_all_feedback());
    for _ in 0..10 {
        clock.advance(Duration::from_millis(20));
        let _ = fx.step();
        if !fx.core.disable_pending && !fx.core.is_torqued {
            break;
        }
    }
    assert!(!fx.core.disable_pending && !fx.core.is_torqued, "thermal-trip background disable must confirm");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn two_phase_persistence_rechecks_operator_auth_before_rename_and_drops_payloads_off_thread() {
    // (a) Enforced OperatorAuth unlocked at zero_check start, then locked AFTER reservation grant
    // and BEFORE `staged.rename_only()` -> file NOT renamed on disk, action fails OPERATOR_REQUIRED,
    // config hold released (`config_inhibited == false`).
    let dir_a = tempfile::tempdir().unwrap();
    let path_a = dir_a.path().join("gripper_calib_auth.json");
    let auth = OperatorAuth::enforced("4321", 60);
    let (token, _ttl) = auth.unlock("4321").unwrap();

    let sup_a = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let id_a = {
        let mut inner = sup_a.inner().write().await;
        inner.lease_enabled = false;
        inner.set_torque(true).unwrap();
        inner.measured_positions = INSPECT.to_vec();
        inner.commanded_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        inner.measured_positions[6] = -0.05;
        inner.commanded_positions[6] = -0.05;
        inner.target_positions[6] = -0.05;
        inner.gripper_calib_path = Some(path_a.clone());
        inner.set_mock_gripper_stall_offset(Some(0.05));
        let epoch = inner.safety.epoch_id();
        let (id, _) = inner
            .start_gripper_zero_check_with_auth(
                &json!({"threshold_rad": 0.10, "store_offset": true}),
                epoch,
                Some((auth.clone(), Some(token.clone()))),
            )
            .unwrap();
        id
    };

    let (at_barrier_tx, at_barrier_rx) = mpsc::sync_channel::<()>(1);
    let (go_tx, go_rx) = mpsc::sync_channel::<()>(1);
    let go_rx = Arc::new(Mutex::new(go_rx));
    let payload_drops_a = Arc::new(Mutex::new(Vec::new()));
    let payload_probe_a: Box<dyn std::any::Any + Send> =
        Box::new(DropProbe { dropped_on: payload_drops_a.clone() });
    let hooks_a = PersistTestHooks {
        after_grant_before_rename: Some(Arc::new(move || {
            let _ = at_barrier_tx.send(());
            let _ = go_rx.lock().unwrap().recv();
            false
        })),
        payload_drop_probe: Some(Arc::new(Mutex::new(Some(payload_probe_a)))),
        ..Default::default()
    };
    let h_a = sup_a
        .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks_a)
        .await
        .unwrap();
    let exec_tid_a = h_a.executor_thread_id();
    at_barrier_rx.recv_timeout(Duration::from_secs(3)).unwrap();
    // Lock operator session while worker is paused between grant and rename!
    auth.lock(Some(&token));
    go_tx.send(()).unwrap();

    let a_res = wait_action_terminal(&h_a, id_a).await;
    assert_eq!(a_res.state, "failed", "{a_res:?}");
    assert_eq!(a_res.code.as_deref(), Some("OPERATOR_REQUIRED"), "{a_res:?}");
    assert!(!path_a.exists(), "locked operator session before rename must NOT rename file onto disk");
    assert!(!h_a.planning_snapshot().unwrap().config_inhibited, "abort before rename must clear config hold");
    let tids_a = payload_drops_a.lock().unwrap().clone();
    assert_eq!(tids_a.len(), 1);
    assert_ne!(tids_a[0], exec_tid_a, "Reserve payload probe dropped on armd-control!");
    h_a.stop_and_join();

    // (b) Payload probe on REFUSED Reserve (E-stop before grant) also drops strictly off `armd-control`.
    let dir_b = tempfile::tempdir().unwrap();
    let path_b = dir_b.path().join("gripper_calib_refused.json");
    let (sup_b, _id_b) = prepare_zero_check_supervisor(path_b.clone()).await;
    let cw_lock = sup_b.config_write_lock();
    let cw_guard = cw_lock.lock().await;
    let payload_drops_b = Arc::new(Mutex::new(Vec::new()));
    let payload_probe_b: Box<dyn std::any::Any + Send> =
        Box::new(DropProbe { dropped_on: payload_drops_b.clone() });
    let hooks_b = PersistTestHooks {
        payload_drop_probe: Some(Arc::new(Mutex::new(Some(payload_probe_b)))),
        ..Default::default()
    };
    let h_b = sup_b
        .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks_b)
        .await
        .unwrap();
    let exec_tid_b = h_b.executor_thread_id();
    wait_until_async(Duration::from_secs(3), || {
        h_b.observation().is_ok_and(|o| o.gripper_zero.as_ref().is_some_and(|g| g.state == "checked"))
    })
    .await;
    sup_b.trigger_e_stop_checked().await.unwrap();
    drop(cw_guard);
    wait_until_async(Duration::from_secs(2), || !payload_drops_b.lock().unwrap().is_empty()).await;
    let tids_b = payload_drops_b.lock().unwrap().clone();
    assert_eq!(tids_b.len(), 1);
    assert_ne!(tids_b[0], exec_tid_b, "refused Reserve payload probe dropped on armd-control!");
    assert!(!path_b.exists());
    h_b.stop_and_join();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn legacy_estop_waiter_keeps_pinned_epoch_after_intervening_clear_fault() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    // Hold `inner.write()` so FIFO write waiters queue in exact order:
    //   1. Caller B (`trigger_e_stop_checked`, req = 1)
    //   2. `clear_fault()` (waits behind Caller B)
    //   3. Caller A (`trigger_e_stop_checked`, req = 2; registered BEFORE Caller B runs!)
    // When `hold` drops:
    //   - Caller B runs `consume_pending_estop()` which sees `req = 2` and resolves BOTH Caller B
    //     and Caller A at `(EStop, epoch_B)`.
    //   - `clear_fault()` then runs, transitioning the supervisor to `(Idle, epoch_after_clear)`.
    //   - Caller A wakes up last and MUST return its pinned `(EStop, epoch_B)`, NOT `(Idle, epoch_after_clear)`!
    let hold = sup.inner().write().await;

    let sup_b = sup.clone();
    let task_b = tokio::spawn(async move { sup_b.trigger_e_stop_checked().await });
    tokio::time::sleep(Duration::from_millis(15)).await;

    let sup_clear = sup.clone();
    let task_clear = tokio::spawn(async move { sup_clear.clear_fault().await });
    tokio::time::sleep(Duration::from_millis(15)).await;

    let sup_a = sup.clone();
    let task_a = tokio::spawn(async move { sup_a.trigger_e_stop_checked().await });
    tokio::time::sleep(Duration::from_millis(15)).await;

    drop(hold);

    let (res_b, res_clear, res_a) = tokio::time::timeout(Duration::from_secs(3), async {
        tokio::join!(task_b, task_clear, task_a)
    })
    .await
    .expect("legacy E-stop + intervening clear_fault must not hang");

    let (st_b, ep_b) = res_b.unwrap().expect("Caller B succeeded");
    let cleared_state = res_clear.unwrap().expect("intervening clear_fault succeeded");
    let (st_a, ep_a) = res_a.unwrap().expect("Caller A succeeded");
    let final_epoch = sup.inner().read().await.safety.epoch_id();

    assert_eq!(st_b, SafetyState::EStop);
    assert_eq!(cleared_state, SafetyState::Idle);
    assert!(final_epoch > ep_b, "clear_fault must have advanced epoch past E-stop ({final_epoch} <= {ep_b})");
    assert_eq!(st_a, SafetyState::EStop, "Caller A must return pinned EStop, not post-clear Idle");
    assert_eq!(ep_a, ep_b, "Caller A must return pinned E-stop epoch, not post-clear epoch");
}

// ============================================================================
// 14. Convergence invariants (R2 + Plan Review): mid-retry/mode-restore E-stop,
//     immutable E-stop journal + full-ring ClearFault, RT capability gates & legacy
//     SLCAN preservation, post-publish supersession, zero-alloc releases & wakes,
//     atomic CommitShutdown, and stop-aware persistence lock
// ============================================================================

#[test]
fn damiao_mid_retry_and_mode_restore_estop_and_legacy_gravity_comp_exit() {
    use armd::driver::GripperPosForce;
    use armd::protocol::damiao::CMD_ENABLE;

    let clock = MockClock::new();

    // (a) Mid-retry E-stop inside `retransmit_enable`: motors drop the initial enable frame so
    // `poll_enable` enters `Service::Retry` at +25ms; E-stop latches on retry frame 2 ->
    // 0 further CMD_ENABLE frames sent, `torque_hs` abandoned, 0 alloc/free under TimedAlloc!
    let raw_bus_r = FakeBus::with_clock(&dm_limits(), clock.clone());
    raw_bus_r.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
        for m in &mut s.motors {
            m.drop_torque_frames = 1;
        }
    });
    let mut inner_r = SupervisorInner::new(ArmConfig::rebot_dm());
    inner_r.lease_enabled = false;
    inner_r.rt_executor_owned = true;
    let mut bus_r = CountingBus::new(raw_bus_r.clone(), Some(clock.clone()));
    bus_r.estop_on_tx = Some((inner_r.requested_estop_count.clone(), 9)); // 7 initial + 2 retries
    let mut drv_r = DamiaoArmDriver::new(bus_r.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv_r.read_joint_states().unwrap();
    drv_r.check_mapping_ranges();
    drv_r.set_rt_mode(true);
    bus_r.tx_seen.store(0, Ordering::SeqCst);
    inner_r.requested_estop_count.store(0, Ordering::SeqCst);
    inner_r = inner_r.with_driver(Box::new(drv_r));
    inner_r.tick(0.02);
    bus_r.tx_seen.store(0, Ordering::SeqCst);
    inner_r.requested_estop_count.store(0, Ordering::SeqCst);
    raw_bus_r.clear_log();

    let (mut fx_r, mut broker_r, _tw_r, _stop_r) =
        ReleaseFixture::new(inner_r, BootId(41), Capacities { commands: 4, plans: 2, replies: 8 });
    let auth_r = fx_r.core.authority(&fx_r.ctx(fx_r.now()));
    let req_r = broker_r
        .submit(ClientId(1), 1, auth_r, fx_r.now() + Duration::from_secs(5), Submit::Transition(TransitionOp::TorqueOn))
        .unwrap();
    let r1 = fx_r.step();
    assert!(r1.command.unwrap().1.is_ok());
    assert_eq!(raw_bus_r.tx_log().iter().filter(|f| f.data == CMD_ENABLE).count(), 7);

    // Advance 30ms past first retry offset (25ms), time allocator across the retry step!
    clock.advance(Duration::from_millis(30));
    armd::diagnostics::alloc::time_this_thread();
    let _ = armd::diagnostics::alloc::take();
    let p_retry = fx_r.core.poll_hardware(&ServiceBudget::for_release(fx_r.now(), fx_r.now() + Duration::from_millis(20)));
    let retry_allocs = armd::diagnostics::alloc::take();
    assert_eq!(p_retry.torque, Some(Err("ESTOP_ACTIVE")), "{p_retry:?}");
    assert_eq!(
        raw_bus_r.tx_log().iter().filter(|f| f.data == CMD_ENABLE).count(),
        9,
        "mid-retry E-stop on retry frame 2 must stop remaining 5 retries (7 initial + 2 retries = 9)"
    );
    assert!(!fx_r.core.torque_transition_pending(), "torque_hs must be abandoned after mid-retry E-stop");
    assert_eq!(retry_allocs.calls, 0, "mid-retry E-stop abandonment must perform 0 alloc/free: {retry_allocs:?}");
    let _ = req_r;

    // (b) Unwarmed Damiao PosForce mode-restore completion AND mid-TX E-stop abandonment:
    // 0 alloc/free on the very first restore/readback without prior buffer warming!
    let make_pf_drv = |bus: CountingBus| {
        let mut d = DamiaoArmDriver::new(bus, vec![1, 2, 3, 4, 5, 6, 7])
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
        d.read_joint_states().unwrap();
        d.set_rt_mode(true);
        d
    };

    // (b1) First-time unwarmed mode-restore + readback completion -> 0 alloc/free!
    let raw_pf1 = FakeBus::with_clock(&dm_limits(), clock.clone());
    raw_pf1.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let bus_pf1 = CountingBus::new(raw_pf1.clone(), Some(clock.clone()));
    let mut drv_pf1 = make_pf_drv(bus_pf1.clone());
    // Clear range check to verified without warming register_replies/register_reply_log!
    // Mark range check done via check_mapping_ranges? No — test directly on a fresh driver whose
    // range_check is None (only set when check_mapping_ranges ran), so register_replies is UNWARMED!
    assert_eq!(drv_pf1.range_check(), None);
    let _ = armd::diagnostics::alloc::take();
    let p0 = drv_pf1.begin_torque_budgeted(true, 0).unwrap();
    assert!(matches!(p0, TorqueProgress::Pending { .. }));
    let mut done = false;
    for _ in 0..15 {
        clock.advance(Duration::from_millis(5));
        match drv_pf1.poll_torque_budgeted(clock.now() + Duration::from_millis(20), 64).unwrap() {
            TorqueProgress::Done(TorqueOutcome::Confirmed) => {
                done = true;
                break;
            }
            TorqueProgress::Pending { .. } => {}
            other => panic!("unexpected torque outcome: {other:?}"),
        }
    }
    let pf1_allocs = armd::diagnostics::alloc::take();
    assert!(done, "unwarmed PosForce mode-restore + readback + enable must confirm");
    assert_eq!(pf1_allocs.calls, 0, "unwarmed mode-restore + readback + enable must do 0 alloc/free: {pf1_allocs:?}");

    // (b2) Mid-TX E-stop inside `write_modes_send` (after initial urgent checkpoint) -> 0 alloc/free,
    // 0 further frames, `EstopInhibited`!
    let raw_pf2 = FakeBus::with_clock(&dm_limits(), clock.clone());
    raw_pf2.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let estop_req_pf2 = Arc::new(std::sync::atomic::AtomicU64::new(0));
    let estop_app_pf2 = Arc::new(std::sync::atomic::AtomicU64::new(0));
    let mut bus_pf2 = CountingBus::new(raw_pf2.clone(), Some(clock.clone()));
    bus_pf2.estop_on_tx = Some((estop_req_pf2.clone(), 1)); // latch on very first mode-write frame!
    let mut drv_pf2 = make_pf_drv(bus_pf2.clone());
    // Establish an empty-queue receive bound before pushing fresh J7 feedback so
    // `begin_torque_budgeted(true, 0)` enters `write_modes_send` directly (testing mid-TX E-stop).
    raw_pf2.with(|s| s.push_all_feedback());
    drv_pf2.read_joint_states().unwrap();
    drv_pf2.set_estop_gate(armd::rt_core::urgent::EstopGate::new(estop_req_pf2.clone(), estop_app_pf2));
    // Ensure flush gets inhibited right after the 1st mode write frame!
    bus_pf2.tx_seen.store(0, Ordering::SeqCst);
    estop_req_pf2.store(0, Ordering::SeqCst);
    raw_pf2.clear_log();
    let _ = armd::diagnostics::alloc::take();
    let err_pf2 = drv_pf2.begin_torque_budgeted(true, 0).unwrap_err();
    let pf2_allocs = armd::diagnostics::alloc::take();
    assert!(armd::driver::is_estop_inhibited(&err_pf2), "{err_pf2:?}");
    assert!(!drv_pf2.torque_transition_pending());
    assert_eq!(raw_pf2.tx_log().len(), 1, "must abort before flush / further frames on frame 1 E-stop");
    assert_eq!(pf2_allocs.calls, 0, "mid-mode-restore E-stop abandonment must do 0 alloc/free: {pf2_allocs:?}");

    // (c) Legacy `set_joint_modes` (used by gravity-comp exit AFTER E-stop has latched) STILL succeeds!
    let raw_gc = FakeBus::with_clock(&dm_limits(), clock.clone());
    raw_gc.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let mut drv_gc = DamiaoArmDriver::new(raw_gc.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    let latched = Arc::new(AtomicBool::new(true));
    drv_gc.set_abort_flag(latched);
    drv_gc.read_joint_states().unwrap();
    let modes = vec![armd::driver::JointMode::Position; 7];
    let hold: Vec<JointTarget> = vec![JointTarget::position(0.0); 7];
    drv_gc
        .set_joint_modes(&modes, &hold)
        .expect("legacy set_joint_modes (gravity-comp exit after E-stop) must remain allowed");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn immutable_estop_ack_journal_clear_fault_full_ring_and_legacy_handoff() {
    use armd::safety::{FaultCode, FaultReason};

    // (a) `consume_pending_estop` + `apply_stop_policy(EStopLatch)` under TimedAlloc (with default
    // `policy-runtime` enabled) from Idle, Armed, EStop, Fault(Other(String)), and Fault(JointLimitExceeded) -> 0 alloc/free!
    armd::diagnostics::alloc::time_this_thread();
    for case in ["idle", "armed", "estop", "fault_other", "fault_joint_limit"] {
        let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
        inner.lease_enabled = false;
        match case {
            "idle" => {}
            "armed" => {
                inner.set_torque(true).unwrap();
            }
            "estop" => {
                inner.safety.trigger_e_stop();
            }
            "fault_other" => {
                inner.safety.trigger_fault_coded(
                    FaultCode::Other,
                    FaultReason::Other("pre-existing heap fault reason".into()),
                );
            }
            "fault_joint_limit" => {
                inner.safety.trigger_fault_coded(
                    FaultCode::JointLimit,
                    FaultReason::JointLimitExceeded {
                        joint_idx: 2,
                        joint_name: "joint_3_elbow".into(),
                        value: 3.2,
                        min: -2.0,
                        max: 2.0,
                    },
                );
            }
            _ => unreachable!(),
        }
        inner.requested_estop_count.fetch_add(1, Ordering::SeqCst);
        let _ = armd::diagnostics::alloc::take();
        assert!(inner.apply_estop());
        let allocs = armd::diagnostics::alloc::take();
        assert_eq!(allocs.calls, 0, "consume_pending_estop allocated/freed in state {case}: {allocs:?}");

        // Also directly prove `clear_fault_executor` + `take_retired_state_arc` does 0 alloc/free
        // even when `SafetyManager::state_arc` holds the SOLE strong reference (refcount == 1)
        // to a heap-bearing Fault!
        if matches!(case, "fault_other" | "fault_joint_limit") {
            let mut sole = SupervisorInner::new(ArmConfig::rebot_dm());
            sole.lease_enabled = false;
            sole.safety.set_off_thread_reclaim(true);
            match case {
                "fault_other" => {
                    sole.safety.trigger_fault_coded(
                        FaultCode::Other,
                        FaultReason::Other("sole-ref heap fault reason".into()),
                    );
                }
                "fault_joint_limit" => {
                    sole.safety.trigger_fault_coded(
                        FaultCode::JointLimit,
                        FaultReason::JointLimitExceeded {
                            joint_idx: 2,
                            joint_name: "joint_3_elbow".into(),
                            value: 3.2,
                            min: -2.0,
                            max: 2.0,
                        },
                    );
                }
                _ => unreachable!(),
            }
            let _ = armd::diagnostics::alloc::take();
            let cleared = sole.clear_fault_executor(Instant::now() + Duration::from_millis(20)).unwrap();
            let retired = sole.safety.pop_retired_state_arc();
            let clear_allocs = armd::diagnostics::alloc::take();
            assert_eq!(cleared, SafetyState::Idle);
            assert!(retired.is_some(), "heap-bearing Fault must be retired via pop_retired_state_arc");
            assert_eq!(
                clear_allocs.calls, 0,
                "sole-ref clear_fault_executor allocated/freed in state {case}: {clear_allocs:?}"
            );
            drop(retired);
        }
    }

    // (b) Full 64-slot ack ring + heap-bearing Fault(Other(String)) AND Fault(JointLimitExceeded) +
    // overflow + ClearFault maintenance:
    //   - All 72 full-ring / overflow Fault E-stop releases do 0 alloc/free on `armd-control`
    //   - `trigger_e_stop_checked()` before ClearFault pins exact `(Fault(fault_case), fault_epoch)`
    //   - Two-step ClearFault transitions `Fault -> EStop -> Idle` while `armd-dispatch` is paused,
    //     retiring the Fault `Arc<SafetyState>` off-thread via `MaintReply`
    for fault_case in [
        FaultReason::Other("heap fault other payload".into()),
        FaultReason::JointLimitExceeded {
            joint_idx: 1,
            joint_name: "joint_2_shoulder".into(),
            value: 3.1,
            min: -2.5,
            max: 2.5,
        },
    ] {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let req_ctr = {
            let mut inner = sup.inner().write().await;
            inner.lease_enabled = false;
            inner.safety.trigger_fault_coded(FaultCode::Other, fault_case.clone());
            inner.requested_estop_count.clone()
        };
        let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(4));
        cfg.alloc_timing = true;
        let handle = sup.start_executor(cfg.clone()).await.unwrap();
        // Warm all 3 observation triple-buffer slots before measuring steady-state Fault E-stop releases.
        wait_until_async(Duration::from_secs(1), || handle.stats().steps.load(Ordering::Relaxed) >= 6).await;

        let fault_epoch = sup.observation().await.unwrap().epoch_id;

        // First verify a real `trigger_e_stop_checked()` while in Fault pins `(Fault(fault_case), fault_epoch)`.
        let (es0_st, es0_ep) = sup.trigger_e_stop_checked().await.unwrap();
        assert_eq!(es0_st, SafetyState::Fault(fault_case.clone()));
        assert_eq!(es0_ep, fault_epoch);
        // Let all 3 triple-buffer observation slots settle after the initial E-stop latch.
        let settled_at = handle.stats().steps.load(Ordering::Relaxed);
        wait_until_async(Duration::from_secs(1), || {
            handle.stats().steps.load(Ordering::Relaxed) >= settled_at + 4
        })
        .await;
        let _ = sup.observation().await;
        let settled_at2 = handle.stats().steps.load(Ordering::Relaxed);
        wait_until_async(Duration::from_secs(1), || {
            handle.stats().steps.load(Ordering::Relaxed) >= settled_at2 + 4
        })
        .await;

        // Pause `armd-dispatch` so the 64-slot `estop_ack_tx` SPSC ring is NOT drained!
        let (held_tx, held_rx) = mpsc::sync_channel::<()>(1);
        let (rel_tx, rel_rx) = mpsc::sync_channel::<()>(1);
        let hub = sup.hub().clone();
        let pauser = thread::spawn(move || {
            let _g = hub.reader_guard();
            let _ = held_tx.send(());
            let _ = rel_rx.recv();
        });
        held_rx.recv_timeout(Duration::from_secs(2)).unwrap();

        let pre_flood_samples = sup.tick_samples().len();
        // Pump 72 distinct E-stop generations across 72 releases while `armd-dispatch` is paused
        // (no caller calls `reg.drain()`): fills ALL 64 slots of `estop_ack_tx` with `Arc<SafetyState>`
        // clones of `Fault(fault_case)` and overflows 8 more into `estop_overflow_gen`!
        for _ in 0..72 {
            req_ctr.fetch_add(1, Ordering::SeqCst);
            let st_now = handle.stats().steps.load(Ordering::Relaxed);
            wait_until_async(Duration::from_secs(1), || {
                handle.stats().steps.load(Ordering::Relaxed) > st_now
            })
            .await;
        }
        // Unpause dispatcher briefly to inspect the 72 full-ring + overflow releases' alloc counts,
        // wait: keep it paused through ClearFault too so ClearFault runs WHILE the ring is full!
        let clear_1 = handle.clear_fault_async().await.unwrap();
        assert_eq!(clear_1.unwrap(), SafetyState::EStop, "Fault + latched EStop clears to EStop first");
        let clear_2 = handle.clear_fault_async().await.unwrap();
        assert_eq!(clear_2.unwrap(), SafetyState::Idle, "second clear_fault clears EStop to Idle");

        // Unpause dispatcher so off-thread draining consumes all 64 queued Fault acks + overflow watermark
        // and drops every `Arc<SafetyState>` strictly off `armd-control`!
        rel_tx.send(()).unwrap();
        pauser.join().unwrap();

        // Wait for diagnostics ring drain and assert ZERO allocator calls across all 72 full-ring + overflow
        // Fault E-stop releases (skipping the initial triple-buffer warm-up and final ClearFault triple-buffer
        // slot string transition in pre-existing `write_observation_into`, which is Step 5 scope).
        let st_done = handle.stats().steps.load(Ordering::Relaxed);
        wait_until_async(Duration::from_secs(1), || {
            handle.stats().steps.load(Ordering::Relaxed) >= st_done + 3
        })
        .await;
        let samples = sup.tick_samples();
        assert!(samples.len() >= pre_flood_samples + 72, "expected >= 72 flood samples, got {}", samples.len());
        let nonzero: Vec<(usize, u32)> = samples
            .iter()
            .enumerate()
            .filter(|(_, s)| s.usage.lock_alloc_calls > 0)
            .map(|(i, s)| (i, s.usage.lock_alloc_calls))
            .collect();
        let flood_slice = &samples[pre_flood_samples..pre_flood_samples + 72];
        let max_flood_allocs = flood_slice.iter().map(|s| s.usage.lock_alloc_calls).max().unwrap_or(0);
        assert_eq!(
            max_flood_allocs, 0,
            "all 72 full-ring + overflow Fault E-stop releases must do 0 alloc/free on armd-control (pre={pre_flood_samples}, total={}, nonzero={nonzero:?})",
            samples.len()
        );
        assert_eq!(sup.observation().await.unwrap().safety_state, SafetyState::Idle);
        handle.stop_and_join();
    }

    // (c) Legacy `control_tick()` applies E-stop BEFORE `start_executor` handoff ->
    // queued `trigger_e_stop_checked()` resolves to exact `(EStop, epoch)` without hanging across handoff!
    let sup_leg = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let hold = sup_leg.inner().write().await;
    let s_tick = sup_leg.clone();
    let t_tick = tokio::spawn(async move { s_tick.control_tick(0.02).await });
    tokio::time::sleep(Duration::from_millis(10)).await;
    let s_es = sup_leg.clone();
    let t_es = tokio::spawn(async move { s_es.trigger_e_stop_checked().await });
    tokio::time::sleep(Duration::from_millis(10)).await;
    let s_xf = sup_leg.clone();
    let t_xf = tokio::spawn(async move {
        s_xf.start_executor(ExecutorConfig::for_tests(Duration::from_millis(5))).await
    });
    tokio::time::sleep(Duration::from_millis(10)).await;
    drop(hold);

    let (tick_res, es_res, xf_res) = tokio::time::timeout(Duration::from_secs(3), async {
        tokio::join!(t_tick, t_es, t_xf)
    })
    .await
    .expect("legacy tick E-stop + handoff must not hang waiter");
    tick_res.unwrap();
    let (st, ep) = es_res.unwrap().expect("waiter resolved with exact pinned E-stop");
    assert_eq!(st, SafetyState::EStop);
    assert!(ep > 0);
    xf_res.unwrap().unwrap().stop_and_join();

    // (d) Unapplied urgent E-stop arriving AFTER Step 6 publish and BEFORE `PersistCmd::Activate`:
    // we inject `e_stop_latched + requested_estop_count` inside `after_rename_before_activate`
    // (without waiting for the next release Step 1 to apply it!) -> `Activate` MUST refuse,
    // keep live policy/revision unchanged, and enter `Reconciling` (`config_inhibited == true`)!
    let dir_u = tempfile::tempdir().unwrap();
    let path_u = dir_u.path().join("gripper_calib_urgent.json");
    let (sup_u, id_u) = prepare_zero_check_supervisor(path_u.clone()).await;
    let estop_latch_u = sup_u.inner().read().await.e_stop_latched.clone();
    let estop_req_u = sup_u.inner().read().await.requested_estop_count.clone();
    let hooks_u = PersistTestHooks {
        after_rename_before_activate: Some(Arc::new(move || {
            // Set urgent E-stop latch + counter directly right as Activate is pushed (unapplied yet!).
            estop_latch_u.store(true, Ordering::SeqCst);
            estop_req_u.fetch_add(1, Ordering::SeqCst);
            false
        })),
        ..Default::default()
    };
    let h_u = sup_u
        .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(10)), hooks_u)
        .await
        .unwrap();
    let res_u = wait_action_terminal(&h_u, id_u).await;
    assert!(
        matches!(res_u.state.as_str(), "failed" | "cancelled"),
        "post-rename unapplied urgent E-stop must refuse activation (never completed): {res_u:?}"
    );
    assert!(
        matches!(res_u.code.as_deref(), Some("STOPPED" | "ESTOP_ACTIVE" | "ESTOP")),
        "expected STOPPED/ESTOP refusal code, got {res_u:?}"
    );
    wait_until_async(Duration::from_secs(3), || {
        h_u.planning_snapshot().is_ok_and(|s| s.config_inhibited)
    })
    .await;
    assert!(path_u.exists(), "rename happened before unapplied urgent E-stop");
    assert_eq!(
        sup_u.observation().await.unwrap().safety_state,
        SafetyState::EStop,
        "urgent E-stop must be latched"
    );
    h_u.stop_and_join();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn rt_capability_gates_legacy_slcan_preservation_zero_offset_no_sleep_and_sessions() {
    let clock = MockClock::new();

    // (a) Blocking CAN transport (`supports_rt = false`, `supports_flush = false`, like SlcanTransport):
    //   - Legacy `set_torque(true)` and `set_torque(false)` STILL SUCCEED!
    //   - Budgeted RT entry points fail closed with UNSUPPORTED!
    //   - `start_executor` fails closed with `503 UNSUPPORTED_RT_DRIVER` leaving `OWNER_LEGACY` intact!
    let raw_blk = FakeBus::with_clock(&dm_limits(), clock.clone());
    raw_blk.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let mut bus_blk = CountingBus::new(raw_blk.clone(), Some(clock.clone()));
    bus_blk.supports_rt = false;
    bus_blk.supports_flush = false;
    let mut drv_blk = DamiaoArmDriver::new(bus_blk.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv_blk.read_joint_states().unwrap();
    drv_blk.check_mapping_ranges();
    // Legacy blocking set_torque(true/false) works cleanly!
    drv_blk.set_torque(true).expect("legacy blocking-transport set_torque(true) must succeed");
    drv_blk.set_torque(false).expect("legacy blocking-transport set_torque(false) must succeed");
    // Explicitly budgeted RT methods fail closed with UNSUPPORTED!
    let mut states_buf = Vec::new();
    let dl = clock.now() + Duration::from_millis(20);
    assert!(drv_blk.read_joint_states_budgeted(&mut states_buf, dl, 64).unwrap_err().to_string().contains("UNSUPPORTED"));
    assert!(drv_blk.begin_torque_budgeted(true, 0).unwrap_err().to_string().contains("UNSUPPORTED"));
    assert!(drv_blk.poll_torque_budgeted(dl, 64).unwrap_err().to_string().contains("UNSUPPORTED"));
    assert!(drv_blk.clear_fault_budgeted(dl).unwrap_err().to_string().contains("UNSUPPORTED"));

    let sup_blk = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_blk)));
    let err_blk = sup_blk
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await;
    assert!(
        matches!(err_blk, Err(ArmError::Rejected { status: 503, ref code, .. }) if code == "UNSUPPORTED_RT_DRIVER"),
        "expected 503 UNSUPPORTED_RT_DRIVER, got {:?}",
        err_blk.as_ref().err()
    );
    assert_eq!(sup_blk.owner_mode(), OWNER_LEGACY, "rejected handoff must keep OWNER_LEGACY intact");

    // (b1) MIT-configured profile rejected at handoff with `503 UNSUPPORTED_RT_CONFIG`, keeping OWNER_LEGACY.
    let mut mit_cfg = ArmConfig::rebot_dm();
    let mut jt = armd::tracking::JointTrackingConfig::rebot_dm();
    jt.enabled = true;
    jt.joints[0].strategy = armd::tracking::TrackingStrategy::Mit;
    jt.joints[0].kp = 10.0;
    jt.joints[0].kd = 0.5;
    jt.joints[0].effort_limit_nm = 5.0;
    jt.joints[0].hold_kp = Some(10.0);
    jt.joints[0].hold_kd = Some(0.5);
    mit_cfg.joint_tracking = Some(jt);
    let sup_mit = Arc::new(ArmSupervisor::new(mit_cfg));
    let err_mit = sup_mit
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await;
    assert!(
        matches!(err_mit, Err(ArmError::Rejected { status: 503, ref code, .. }) if code == "UNSUPPORTED_RT_CONFIG"),
        "expected 503 UNSUPPORTED_RT_CONFIG, got {:?}",
        err_mit.as_ref().err()
    );
    assert_eq!(sup_mit.owner_mode(), OWNER_LEGACY);

    // (b2) Active gravity_comp session rejected at handoff with `409 TRANSFER_BLOCKED`, keeping OWNER_LEGACY
    // and leaving `gravity_comp` session state intact.
    let sup_gc = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    {
        let mut inner = sup_gc.inner().write().await;
        inner.gravity_comp = Some(armd::gravity_comp::GravityCompRun {
            phase: armd::gravity_comp::GravityCompPhase::Active,
            policy: armd::model::GravityCompPolicy::rebot_dm(),
            elapsed_s: 1.0,
            scale: 1.0,
            damping_kd: [0.2; 6],
            entry_pose: [0.0; 6],
            entry_torque: [0.0; 6],
            tau_ff: [0.0; 6],
            kp: [0.0; 6],
            kd: [0.2; 6],
            position: [0.0; 6],
            overspeed_count: 0,
            prev_measured: None,
            lease_generation: None,
        });
    }
    let err_gc = sup_gc
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await;
    assert!(
        matches!(err_gc, Err(ArmError::Rejected { status: 409, ref code, .. }) if code == "TRANSFER_BLOCKED"),
        "expected 409 TRANSFER_BLOCKED for active gravity_comp, got {:?}",
        err_gc.as_ref().err()
    );
    assert_eq!(sup_gc.owner_mode(), OWNER_LEGACY);
    assert!(sup_gc.inner().read().await.gravity_comp.is_some(), "rejected handoff must preserve gravity_comp");

    // (c) Normal `ArmSupervisor::with_driver(...)` (wrapped in `ZeroOffsetDriver`) -> `start_executor(...)`
    // propagates `set_rt_mode(true)` through `ZeroOffsetDriver`: injected `WouldBlock` on target TX
    // performs 1 attempt per frame and ZERO `clock_pause` sleeps!
    let raw_wb = FakeBus::with_clock(&dm_limits(), clock.clone());
    raw_wb.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let bus_wb = CountingBus::new(raw_wb.clone(), Some(clock.clone()));
    let pauses_wb = bus_wb.clock_pauses.clone();
    let mut drv_wb = DamiaoArmDriver::new(bus_wb.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv_wb.read_joint_states().unwrap();
    drv_wb.check_mapping_ranges();
    let sup_wb = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_wb)));
    sup_wb.set_lease_enabled(false).await;
    let h_wb = sup_wb
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(8)))
        .await
        .unwrap();
    let r_on = submit_broker_transition(&h_wb, 1, TransitionOp::TorqueOn);
    assert_eq!(wait_broker_terminal(&h_wb, r_on, Duration::from_secs(2)), Outcome::Completed);

    // Inject WouldBlock on TX; observe next release attempts TX ONCE with ZERO clock pauses!
    pauses_wb.store(0, Ordering::SeqCst);
    bus_wb.tx_seen.store(0, Ordering::SeqCst);
    raw_wb.with(|s| s.tx_stall = true);
    let step0 = h_wb.stats().steps.load(Ordering::Relaxed);
    wait_until_async(Duration::from_secs(1), || h_wb.stats().steps.load(Ordering::Relaxed) >= step0 + 2).await;
    assert_eq!(pauses_wb.load(Ordering::SeqCst), 0, "ZeroOffsetDriver->DamiaoArmDriver in RT mode must perform 0 clock_pause sleeps on WouldBlock");
    assert!(bus_wb.tx_seen.load(Ordering::SeqCst) >= 1, "expected at least one TX attempt under WouldBlock");
    raw_wb.with(|s| s.tx_stall = false);
    h_wb.stop_and_join();
}

#[test]
fn superseded_install_delivers_terminal_and_ack_strictly_after_publish() {
    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
    inner.lease_enabled = false;
    inner.set_torque(true).unwrap();
    inner.tick(0.02);

    let (mut engine, mut broker, _tw, _stop) =
        ReleaseEngine::new(BootId(42), Capacities { commands: 4, plans: 4, replies: 16 });
    let now = Instant::now();
    let deadline = now + Duration::from_secs(5);
    let auth = inner.authority(&engine.ctx(now));
    let start = inner.commanded_positions.clone();
    let mut q1 = start.clone();
    q1[0] += 0.15;
    let p1 = Primitive::joint_quintic(&start, &q1, 1.0).unwrap();
    let req1 = broker
        .submit(ClientId(1), 1, auth, deadline, Submit::Install(&move |slot: &mut PlanSlot| slot.set(p1)))
        .unwrap();
    let r1 = engine.step(&mut inner, || now, 0.02, now + Duration::from_millis(20), 1);
    assert!(r1.command.unwrap().1.is_ok());
    broker.poll(|_, _| true);

    // Release 2: submit req2 superseding active req1. Wrap `CoreOps` to assert the reply ring is
    // COMPLETELY EMPTY during `publish()`, proving NEITHER `Terminal(req1, Superseded)` NOR
    // `Ack(req2)` is delivered prior to Step 6 publication!
    let broker_shared = Arc::new(Mutex::new(broker));
    struct PublishCheckCore {
        inner: SupervisorInner,
        broker: Arc<Mutex<armd::rt_core::broker::Broker<PlanSlot>>>,
        replies_seen_at_publish: usize,
    }
    impl CoreOps for PublishCheckCore {
        fn authority(&self, ctx: &armd::rt_core::ops::CommitCtx) -> Authority { self.inner.authority(ctx) }
        fn planning_snapshot(&self, ctx: &armd::rt_core::ops::CommitCtx) -> PlanningSnapshot {
            self.inner.planning_snapshot(ctx)
        }
        fn apply_estop(&mut self) -> bool { self.inner.apply_estop() }
        fn apply_stop(&mut self) { self.inner.apply_stop(); }
        fn poll_hardware(&mut self, b: &ServiceBudget) -> HardwarePoll { self.inner.poll_hardware(b) }
        fn commit_primitive(
            &mut self,
            d: &armd::rt_core::broker::Descriptor,
            p: &Primitive,
            ctx: &armd::rt_core::ops::CommitCtx,
        ) -> Result<Committed, &'static str> {
            self.inner.commit_primitive(d, p, ctx)
        }
        fn commit_immediate(
            &mut self,
            d: &armd::rt_core::broker::Descriptor,
            op: ImmediateOp,
            ctx: &armd::rt_core::ops::CommitCtx,
        ) -> Result<Committed, &'static str> {
            self.inner.commit_immediate(d, op, ctx)
        }
        fn commit_transition(
            &mut self,
            d: &armd::rt_core::broker::Descriptor,
            op: TransitionOp,
            ctx: &armd::rt_core::ops::CommitCtx,
        ) -> Result<Committed, &'static str> {
            self.inner.commit_transition(d, op, ctx)
        }
        fn consume_target(&mut self, f: &TargetFrame, ctx: &armd::rt_core::ops::CommitCtx) -> Result<(), Refusal> {
            self.inner.consume_target(f, ctx)
        }
        fn advance(&mut self, dt: f64, gen: u64) { self.inner.advance(dt, gen); }
        fn advance_output(&mut self, dt: f64, gen: u64) -> f64 { self.inner.advance_output(dt, gen) }
        fn publish(&mut self, ts: f64, gen: u64) {
            self.broker.lock().unwrap().poll(|_, _| {
                self.replies_seen_at_publish += 1;
                true
            });
            self.inner.publish(ts, gen);
        }
        fn now_timestamp(&self) -> f64 { self.inner.now_timestamp() }
        fn action_outcome(&self, action: armd::rt_core::broker::ActionId) -> Option<Outcome> {
            self.inner.action_outcome(action)
        }
    }

    let cur_start = inner.commanded_positions.clone();
    let mut q2 = cur_start.clone();
    q2[0] -= 0.05;
    let p2 = Primitive::joint_quintic(&cur_start, &q2, 1.0).unwrap();
    let auth2 = inner.authority(&engine.ctx(now));
    let req2 = broker_shared
        .lock()
        .unwrap()
        .submit(ClientId(1), 2, auth2, deadline, Submit::Install(&move |slot: &mut PlanSlot| slot.set(p2)))
        .unwrap();
    let mut check_core = PublishCheckCore {
        inner,
        broker: broker_shared.clone(),
        replies_seen_at_publish: 0,
    };
    let r2 = engine.step(&mut check_core, || now, 0.02, now + Duration::from_millis(20), 2);
    assert!(r2.command.unwrap().1.is_ok());
    assert_eq!(
        check_core.replies_seen_at_publish, 0,
        "neither superseded Terminal nor deferred Ack may be emitted before Step 6 publish"
    );

    let mut replies = Vec::new();
    broker_shared.lock().unwrap().poll(|_, r| {
        replies.push(r);
        true
    });
    assert_eq!(replies.len(), 2, "{replies:?}");
    assert!(
        matches!(replies[0], Reply::Terminal { request, outcome: Outcome::Superseded, .. } if request == req1),
        "Terminal(old, Superseded) must strictly precede Ack(new): {replies:?}"
    );
    assert!(
        matches!(replies[1], Reply::Ack { request, .. } if request == req2),
        "Ack(new) must immediately follow Terminal(old, Superseded): {replies:?}"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn zero_alloc_steady_state_wakes_positive_control_commit_shutdown_and_stop_aware_persist() {
    // (a1) Mock steady-state enabled releases: 0 allocator calls (`lock_alloc_calls == 0`).
    let sup_mock = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    sup_mock.set_lease_enabled(false).await;
    sup_mock.set_torque(true).await.unwrap();
    let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(5));
    cfg.alloc_timing = true;
    let h_mock = sup_mock.start_executor(cfg.clone()).await.unwrap();
    wait_until_async(Duration::from_secs(1), || h_mock.stats().steps.load(Ordering::Relaxed) >= 12).await;
    let mock_steady: Vec<u32> = sup_mock
        .tick_samples()
        .into_iter()
        .rev()
        .take(5)
        .map(|s| s.usage.lock_alloc_calls)
        .collect();
    assert_eq!(mock_steady.len(), 5);
    assert!(mock_steady.iter().all(|&c| c == 0), "mock steady-state allocated: {mock_steady:?}");
    // Driverless mock arm with torque on still confirms shutdown and commits `shutdown_and_join()`,
    // while `clear_fault()` invalidates any stale confirmation.
    assert!(sup_mock.request_shutdown().await.unwrap());
    assert_eq!(sup_mock.clear_fault().await.unwrap(), SafetyState::Idle);
    assert_eq!(h_mock.shutdown_and_join(), Err(OwnerError::ShutdownNotConfirmed));
    assert!(!h_mock.is_exited());
    assert!(sup_mock.request_shutdown().await.unwrap());
    h_mock.shutdown_and_join().unwrap();
    assert!(h_mock.is_exited());

    // (a2) SimArmDriver (both normal AND load_only_feedback): 0 allocator calls in enabled steady-state!
    for load_only in [false, true] {
        let init_q = vec![0.05, -0.02, 0.02, -0.02, 0.02, -0.02, 0.0];
        let sim = LoopbackSim::start(7, init_q);
        let drv = SimArmDriver::connect(&sim.addr, 7).unwrap().with_load_only_feedback(load_only);
        let sup_sim = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
        sup_sim.set_lease_enabled(false).await;
        let h_sim = sup_sim.start_executor(cfg.clone()).await.unwrap();
        wait_until_async(Duration::from_secs(2), || {
            h_sim.observation().is_ok_and(|o| (o.positions[0] - 0.05).abs() < 1e-3)
        })
        .await;
        let req_on = submit_broker_transition(&h_sim, 1, TransitionOp::TorqueOn);
        assert_eq!(wait_broker_terminal(&h_sim, req_on, Duration::from_secs(2)), Outcome::Completed);
        let s0 = h_sim.stats().steps.load(Ordering::Relaxed);
        wait_until_async(Duration::from_secs(2), || h_sim.stats().steps.load(Ordering::Relaxed) >= s0 + 12).await;
        let sim_steady: Vec<u32> = sup_sim
            .tick_samples()
            .into_iter()
            .rev()
            .take(5)
            .map(|s| s.usage.lock_alloc_calls)
            .collect();
        assert_eq!(sim_steady.len(), 5);
        assert!(
            sim_steady.iter().all(|&c| c == 0),
            "SimArmDriver (load_only={load_only}) steady-state allocated: {sim_steady:?}"
        );
        h_sim.stop_and_join();
    }

    // (a3) DamiaoArmDriver<FakeBus> enabled steady-state (allocation-neutral FakeBus): 0 allocator calls!
    let bus_dm = FakeBus::new(&dm_limits());
    bus_dm.with(|s| {
        s.immediate_replies = true;
        s.record_tx_log = false;
        s.push_all_feedback();
    });
    let mut drv_dm = DamiaoArmDriver::new(bus_dm.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv_dm.read_joint_states().unwrap();
    drv_dm.check_mapping_ranges();
    let sup_dm = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_dm)));
    sup_dm.set_lease_enabled(false).await;
    let h_dm = sup_dm.start_executor(cfg.clone()).await.unwrap();
    let req_dm_on = submit_broker_transition(&h_dm, 1, TransitionOp::TorqueOn);
    assert_eq!(wait_broker_terminal(&h_dm, req_dm_on, Duration::from_secs(2)), Outcome::Completed);
    let dm_s0 = h_dm.stats().steps.load(Ordering::Relaxed);
    wait_until_async(Duration::from_secs(2), || h_dm.stats().steps.load(Ordering::Relaxed) >= dm_s0 + 12).await;
    let dm_steady: Vec<u32> = sup_dm
        .tick_samples()
        .into_iter()
        .rev()
        .take(5)
        .map(|s| s.usage.lock_alloc_calls)
        .collect();
    assert_eq!(dm_steady.len(), 5);
    assert!(dm_steady.iter().all(|&c| c == 0), "DamiaoArmDriver<FakeBus> steady-state allocated: {dm_steady:?}");
    h_dm.stop_and_join();

    // (b) Sub-release torque confirmation wake: 0 allocator calls across full wake body + positive control!
    let (drv_w, ..) = ThreadAuditDriver::new(7);
    let sup_w = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_w)));
    sup_w.set_lease_enabled(false).await;
    let inject_flag = Arc::new(AtomicBool::new(false));
    let hooks_w = PersistTestHooks {
        inject_wake_alloc_once: Some(inject_flag.clone()),
        ..Default::default()
    };
    let mut cfg_w = ExecutorConfig::for_tests(Duration::from_millis(10));
    cfg_w.alloc_timing = true;
    let h_w = sup_w.start_executor_with_hooks(cfg_w, hooks_w).await.unwrap();
    wait_until_async(Duration::from_secs(1), || h_w.stats().steps.load(Ordering::Relaxed) >= 4).await;
    let r_w1 = submit_broker_transition(&h_w, 1, TransitionOp::TorqueOn);
    assert_eq!(wait_broker_terminal(&h_w, r_w1, Duration::from_secs(2)), Outcome::Completed);
    assert!(h_w.stats().wakes.load(Ordering::Relaxed) >= 1, "expected >= 1 real sub-release wake");
    assert_eq!(
        h_w.stats().wake_alloc_calls.load(Ordering::Relaxed),
        0,
        "normal sub-release torque confirmation wake must do 0 alloc/free"
    );
    // Positive control: arm deliberate allocation inside the next wake -> MUST report wake_alloc_calls > 0!
    inject_flag.store(true, Ordering::Release);
    let r_w2 = submit_broker_transition(&h_w, 2, TransitionOp::TorqueOff);
    assert_eq!(wait_broker_terminal(&h_w, r_w2, Duration::from_secs(2)), Outcome::Completed);
    assert!(
        h_w.stats().wake_alloc_calls.load(Ordering::Relaxed) > 0,
        "positive control failed: deliberate wake allocation/free was not observed by wake_alloc_calls"
    );

    // (c) Atomic CommitShutdown: confirming shutdown while disabled, then clearing fault + rearming
    // MUST make `shutdown_and_join()` fail with `ShutdownNotConfirmed` WITHOUT stopping the executor!
    assert!(sup_w.request_shutdown().await.unwrap(), "shutdown confirms while torque is off");
    assert_eq!(sup_w.clear_fault().await.unwrap(), SafetyState::Idle);
    let r_rearm = submit_broker_transition(&h_w, 3, TransitionOp::TorqueOn);
    assert_eq!(wait_broker_terminal(&h_w, r_rearm, Duration::from_secs(2)), Outcome::Completed);
    assert_eq!(
        h_w.shutdown_and_join(),
        Err(OwnerError::ShutdownNotConfirmed),
        "stale shutdown confirmation must NOT stop executor after rearming"
    );
    assert!(!h_w.is_exited(), "executor must remain live when CommitShutdown rejects rearmed state");
    // Disabling + confirming shutdown again now commits and joins cleanly!
    let r_dis = submit_broker_transition(&h_w, 4, TransitionOp::TorqueOff);
    assert_eq!(wait_broker_terminal(&h_w, r_dis, Duration::from_secs(2)), Outcome::Completed);
    assert!(sup_w.request_shutdown().await.unwrap());
    h_w.shutdown_and_join().unwrap();
    assert!(h_w.is_exited());

    // (d) Stop-aware persistence lock: hold `config_write_lock()`, let persistence job reach lock
    // acquisition, call `stop_and_join()` WHILE STILL HOLDING `config_write_lock()` -> zero deadlock!
    let dir_dl = tempfile::tempdir().unwrap();
    let path_dl = dir_dl.path().join("gripper_calib_nolock_deadlock.json");
    let (sup_dl, _id_dl) = prepare_zero_check_supervisor(path_dl.clone()).await;
    let cw_lock = sup_dl.config_write_lock();
    let _cw_guard = cw_lock.lock().await;
    let h_dl = sup_dl
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .unwrap();
    wait_until_async(Duration::from_secs(3), || {
        h_dl.observation().is_ok_and(|o| o.gripper_zero.as_ref().is_some_and(|g| g.state == "checked"))
    })
    .await;
    // Give `armd-persist` a moment to enter its stop-aware `config_write_mutex` wait loop, then tear down
    // on a blocking task WHILE `_cw_guard` is STILL HELD!
    tokio::time::sleep(Duration::from_millis(20)).await;
    let h_dl_stop = h_dl.clone();
    tokio::time::timeout(
        Duration::from_secs(2),
        tokio::task::spawn_blocking(move || h_dl_stop.stop_and_join()),
    )
    .await
    .expect("stop_and_join deadlocked while config_write_mutex was held!")
    .unwrap();
    assert!(!path_dl.exists());
}
