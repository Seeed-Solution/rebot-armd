//! Stage 3 §3 PR B1: proof of zero `SupervisorLock` attempts on the RT control thread.
//!
//! Covers:
//! 1. `ControlThreadGuard` independence from `ReleaseGuard` (nesting, panic unwind, and
//!    real `armd-control` thread marking from entry to exit outside `ReleaseGuard`).
//! 2. All 7 `SupervisorLock` acquisition forms (`read`, `try_read`, `write`, `try_write`,
//!    `blocking_write`, `write_owned`, `write_tick`) counted and rejected on a marked thread
//!    BEFORE asserting and BEFORE touching the underlying `tokio::sync::RwLock`, both when
//!    contended (another thread holds the writer) and uncontended, plus callsite timing.
//! 3. All 4 async forms (`read`, `write`, `write_owned`, `write_tick`) constructed off-thread
//!    and polled on a marked thread — both on first poll and on subsequent polls after an
//!    off-thread `Poll::Pending`.
//! 4. Production executor fixtures (`--follower mock` owner and `DamiaoArmDriver<FakeBus>`
//!    CAN bus owner) holding the detached legacy writer on another OS thread throughout:
//!    requiring >= 3 publication advances, matching applied urgent stop generation,
//!    inhibited dynamic output (including mid-batch and post-stop CAN bus trace), and
//!    strictly ZERO `SupervisorLock` attempts before releasing the writer.

mod mit_support;

use std::any::Any;
use std::collections::VecDeque;
use std::future::Future;
use std::pin::Pin;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicUsize, Ordering};
use std::sync::{mpsc, Arc, Mutex, OnceLock};
use std::task::{Context, Poll, Waker};
use std::thread::{self, ThreadId};
use std::time::{Duration, Instant};

use armd::diagnostics::alloc::{AllocCounts, TimedAlloc};
use armd::driver::{
    ArmDriver, ArmDriverError, DamiaoArmDriver, DriverCapabilities, EnableState, JointTarget,
    TorqueOutcome, TorqueProgress, TxBudget,
};
use armd::model::JointState;
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::broker::{
    Authority, BootId, Capacities, ClientId, Outcome, Reply, RequestId, Submit, TransitionOp,
};
use armd::rt_core::executor::{
    in_rt_release, on_rt_control_thread, validate_startup_owner, ControlThreadGuard,
    ExecutorConfig, PanicPolicy, PersistTestHooks, ReleaseGuard, RtExecutorHandle, RtOwner,
};
use armd::rt_core::ops::{PlanSlot, ReleaseFixture};
use armd::rt_core::urgent::StopRequests;
use armd::supervisor::{PanicDisableOutcome, SupervisorInner, OWNER_EXECUTOR, OWNER_FAILED, OWNER_LEGACY};
use armd::supervisor_lock::{DriverLock, DriverSlot, HoldKind, SupervisorLock};
use armd::transport::{CanFrame, CanTransport, DriverClock, MockClock, TransportError};
use armd::{ArmConfig, ArmSupervisor, SafetyState};
use mit_support::FakeBus;

#[global_allocator]
static ALLOC: TimedAlloc = TimedAlloc;

// ============================================================================
// Helpers
// ============================================================================

fn noop_waker() -> &'static Waker {
    Waker::noop()
}

fn panic_payload_to_string(payload: Box<dyn Any + Send>) -> String {
    if let Some(s) = payload.downcast_ref::<&str>() {
        (*s).to_string()
    } else if let Some(s) = payload.downcast_ref::<String>() {
        s.clone()
    } else {
        "<non-string panic>".to_string()
    }
}

fn assert_panics_with(expected_substrings: &[&str], f: impl FnOnce()) {
    let err = std::panic::catch_unwind(std::panic::AssertUnwindSafe(f))
        .expect_err("expected panic on marked thread");
    let msg = panic_payload_to_string(err);
    for needle in expected_substrings {
        assert!(
            msg.contains(needle),
            "panic message {msg:?} did not contain expected substring {needle:?}"
        );
    }
}

fn run_bounded_fixture(
    timeout: Duration,
    name: &'static str,
    f: impl FnOnce(Instant) + Send + 'static,
) {
    let deadline = Instant::now() + timeout;
    let (done_tx, done_rx) = mpsc::sync_channel::<Result<(), String>>(1);
    let jh = thread::Builder::new()
        .name(format!("fixture-{name}"))
        .spawn(move || {
            let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| f(deadline)));
            let _ = done_tx.send(res.map_err(panic_payload_to_string));
        })
        .expect("spawn bounded fixture thread");
    match done_rx.recv_timeout(timeout) {
        Ok(Ok(())) => jh.join().expect("fixture thread join"),
        Ok(Err(msg)) => {
            let _ = jh.join();
            panic!("fixture {name} failed: {msg}");
        }
        Err(mpsc::RecvTimeoutError::Timeout) => {
            // Do not block on `jh.join()`: fail immediately if startup or teardown stalled.
            panic!(
                "fixture {name} exceeded bounded test deadline ({timeout:?}) during startup, assertions, or teardown"
            );
        }
        Err(mpsc::RecvTimeoutError::Disconnected) => {
            panic!("fixture {name} thread terminated unexpectedly");
        }
    }
}

fn wait_until_deadline(deadline: Instant, desc: &str, mut pred: impl FnMut() -> bool) {
    while Instant::now() < deadline {
        if pred() {
            return;
        }
        thread::sleep(Duration::from_millis(2));
    }
    assert!(pred(), "timed out before deadline waiting for: {desc}");
}

async fn wait_until_deadline_async(deadline: Instant, desc: &str, mut pred: impl FnMut() -> bool) {
    while Instant::now() < deadline {
        if pred() {
            return;
        }
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
    assert!(pred(), "timed out before deadline waiting for: {desc}");
}

fn dm_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm()
        .damiao_ranges
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect()
}

fn submit_broker_transition(handle: &RtExecutorHandle, seq: u64, op: TransitionOp) -> RequestId {
    let snap = handle.planning_snapshot().unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut b = handle.broker().lock().unwrap();
    b.submit(ClientId(1), seq, snap.authority, deadline, Submit::Transition(op))
        .unwrap()
}

fn wait_broker_reply(
    handle: &RtExecutorHandle,
    req: RequestId,
    deadline: Instant,
    mut accept: impl FnMut(Reply) -> Option<Reply>,
) -> Reply {
    while Instant::now() < deadline {
        let mut found = None;
        handle.broker().lock().unwrap().poll(|_, r| {
            let matches_req = r.request() == req;
            if matches_req && found.is_none() {
                found = accept(r);
            }
            true
        });
        if let Some(reply) = found {
            return reply;
        }
        thread::sleep(Duration::from_millis(2));
    }
    panic!("broker request {req:?} did not receive expected reply before deadline");
}

fn wait_broker_terminal(handle: &RtExecutorHandle, req: RequestId, deadline: Instant) -> Outcome {
    match wait_broker_reply(handle, req, deadline, |r| {
        matches!(r, Reply::Terminal { .. }).then_some(r)
    }) {
        Reply::Terminal { outcome, .. } => outcome,
        other => unreachable!("unexpected reply: {other:?}"),
    }
}

fn submit_joint_quintic(
    sup: &ArmSupervisor,
    handle: &RtExecutorHandle,
    seq: u64,
    authority: Authority,
    start: &[f32],
    goal: &[f32],
    duration_s: f64,
) -> RequestId {
    use armd::rt_core::requests::motion::{self as req_motion, MotionPlanSnapshot, MotionRequest};
    let deadline = Instant::now() + Duration::from_secs(5);
    let published = sup.hub().latest().expect("published snapshot for jog prepare");
    let cfg = sup.hub().motion_config().expect("motion config cached on hub");
    let mut snap = MotionPlanSnapshot::from_published(&published, cfg).expect("from_published");
    snap.commanded_positions = start.to_vec();
    snap.authority.epoch = authority.epoch;
    snap.authority.lease_generation = authority.lease_generation;
    snap.authority.stop_generation = authority.stop_generation;
    snap.authority.deadline = deadline;
    let plan = req_motion::prepare_motion(&snap, &MotionRequest::jog(goal.to_vec(), Some(duration_s), None))
        .expect("prepare_motion");
    let mut b = handle.broker().lock().unwrap();
    b.submit(
        ClientId(1),
        seq,
        authority,
        deadline,
        Submit::Install(&move |slot: &mut PlanSlot| req_motion::stage_jog(slot, &plan.slot)),
    )
    .unwrap()
}

/// Holds the detached legacy `SupervisorLock` write guard on a dedicated OS thread until
/// explicitly released after all assertions have been collected (with RAII cleanup on panic).
struct DetachedLegacyWriterHold {
    release_tx: Option<mpsc::SyncSender<()>>,
    thread: Option<thread::JoinHandle<()>>,
    held: Arc<AtomicBool>,
}

impl DetachedLegacyWriterHold {
    fn acquire(sup: Arc<ArmSupervisor>, deadline: Instant) -> Self {
        let (ready_tx, ready_rx) = mpsc::sync_channel::<bool>(1);
        let (release_tx, release_rx) = mpsc::sync_channel::<()>(1);
        let held = Arc::new(AtomicBool::new(false));
        let held_bg = Arc::clone(&held);

        let jh = thread::Builder::new()
            .name("test-legacy-writer-hold".into())
            .spawn(move || {
                let guard = sup.inner().blocking_write();
                let detached_shell = sup.owner_mode() == OWNER_EXECUTOR && !guard.is_torqued;
                held_bg.store(true, Ordering::SeqCst);
                let _ = ready_tx.send(detached_shell);
                // Keep the write guard held across the test's assertion window, with a bounded
                // failsafe so a stuck caller can never wedge the process forever.
                let wait = deadline
                    .saturating_duration_since(Instant::now())
                    .saturating_add(Duration::from_secs(2));
                let _ = release_rx.recv_timeout(wait);
                held_bg.store(false, Ordering::SeqCst);
                drop(guard);
            })
            .expect("spawn legacy writer holder thread");

        let wait = deadline.saturating_duration_since(Instant::now());
        let detached = ready_rx
            .recv_timeout(wait)
            .expect("legacy writer holder thread failed to acquire detached SupervisorLock before deadline");
        assert!(detached, "expected legacy SupervisorLock shell to be detached in executor mode");

        Self {
            release_tx: Some(release_tx),
            thread: Some(jh),
            held,
        }
    }

    fn is_held(&self) -> bool {
        self.held.load(Ordering::SeqCst)
    }

    fn release(mut self) {
        assert!(
            self.is_held(),
            "legacy writer guard was released before assertions completed"
        );
        if let Some(tx) = self.release_tx.take() {
            let _ = tx.send(());
        }
        if let Some(jh) = self.thread.take() {
            jh.join().expect("legacy writer holder thread panicked");
        }
        assert!(!self.is_held());
    }
}

impl Drop for DetachedLegacyWriterHold {
    fn drop(&mut self) {
        if let Some(tx) = self.release_tx.take() {
            let _ = tx.send(());
        }
        if let Some(jh) = self.thread.take() {
            let _ = jh.join();
        }
    }
}

// ============================================================================
// 1. Thread-marking guard independence & entry-to-exit coverage on armd-control
// ============================================================================

#[test]
fn control_thread_guard_is_independent_of_release_guard() {
    assert!(!on_rt_control_thread());
    assert!(!in_rt_release());

    // ControlThreadGuard alone marks only ON_RT_CONTROL_THREAD, leaving IN_RT_RELEASE false.
    {
        let _outer = ControlThreadGuard::enter();
        assert!(on_rt_control_thread());
        assert!(!in_rt_release());

        {
            let _inner = ControlThreadGuard::enter();
            assert!(on_rt_control_thread());
            assert!(!in_rt_release());
        }
        // Dropping nested guard keeps outer active.
        assert!(on_rt_control_thread());
        assert!(!in_rt_release());

        // ReleaseGuard inside ControlThreadGuard toggles independently.
        {
            let _rel = ReleaseGuard::enter();
            assert!(on_rt_control_thread());
            assert!(in_rt_release());
        }
        assert!(on_rt_control_thread());
        assert!(!in_rt_release());
    }
    assert!(!on_rt_control_thread());
    assert!(!in_rt_release());

    // ReleaseGuard alone does NOT mark ON_RT_CONTROL_THREAD.
    {
        let _rel = ReleaseGuard::enter();
        assert!(!on_rt_control_thread());
        assert!(in_rt_release());
    }
    assert!(!on_rt_control_thread());
    assert!(!in_rt_release());

    // Panic unwind restores ON_RT_CONTROL_THREAD cleanly.
    let res = std::panic::catch_unwind(|| {
        let _g = ControlThreadGuard::enter();
        assert!(on_rt_control_thread());
        panic!("unwind check");
    });
    assert!(res.is_err());
    assert!(!on_rt_control_thread());
}

#[derive(Default)]
struct LifecycleMarks {
    entry_rt_mode: Option<(ThreadId, bool, bool)>,
    release_reads: Vec<(ThreadId, bool, bool)>,
    drop_mark: Option<(ThreadId, bool, bool)>,
}

struct LifecycleAuditDriver {
    dof: usize,
    marks: Arc<Mutex<LifecycleMarks>>,
}

impl ArmDriver for LifecycleAuditDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            enable_feedback: true,
            velocity_feedback: true,
            torque_feedback: true,
            temperature_feedback: true,
            ..DriverCapabilities::default()
        }
    }
    fn set_rt_mode(&mut self, enabled: bool) {
        if enabled {
            self.marks.lock().unwrap().entry_rt_mode =
                Some((thread::current().id(), on_rt_control_thread(), in_rt_release()));
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
        self.marks.lock().unwrap().release_reads.push((
            thread::current().id(),
            on_rt_control_thread(),
            in_rt_release(),
        ));
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
    fn set_torque(&mut self, _enable: bool) -> Result<(), ArmDriverError> { Ok(()) }
    fn supports_rt_executor(&self) -> Result<(), &'static str> { Ok(()) }
    fn is_feedback_valid(&self) -> bool { true }
    fn is_enabled(&self) -> bool { false }
    fn enable_state(&self) -> EnableState { EnableState::Disabled }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::from_millis(1)); self.dof] }
    fn feedback_instants(&self) -> Vec<Option<Instant>> { vec![Some(Instant::now()); self.dof] }
    fn feedback_seqs(&self) -> Vec<u64> { vec![1; self.dof] }
}

impl Drop for LifecycleAuditDriver {
    fn drop(&mut self) {
        self.marks.lock().unwrap().drop_mark = Some((
            thread::current().id(),
            on_rt_control_thread(),
            in_rt_release(),
        ));
    }
}

#[test]
fn control_thread_marked_from_entry_to_exit_independent_of_release_guard() {
    run_bounded_fixture(Duration::from_secs(15), "entry-to-exit", |deadline| {
        let rt = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(2)
            .enable_all()
            .build()
            .unwrap();
        rt.block_on(async move {
            // (a) Full lifecycle: entry (`set_rt_mode`), releases, and exit (`Drop` of core).
            let marks = Arc::new(Mutex::new(LifecycleMarks::default()));
            let drv = LifecycleAuditDriver { dof: 7, marks: Arc::clone(&marks) };
            let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
            sup.set_lease_enabled(false).await;
            sup.register_ws_connection(1).await.unwrap();
            let handle = sup
                .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
                .await
                .expect("start executor");
            let exec_tid = handle.executor_thread_id();

            wait_until_deadline_async(deadline, ">= 3 releases observed", || {
                handle.stats().steps.load(Ordering::Relaxed) >= 3
            })
            .await;

            handle.stop_and_join();

            {
                let m = marks.lock().unwrap();
                assert_eq!(
                    m.entry_rt_mode,
                    Some((exec_tid, true, false)),
                    "control thread must be marked on entry before any ReleaseGuard"
                );
                assert!(
                    m.release_reads.len() >= 3,
                    "expected >= 3 release reads, got {}",
                    m.release_reads.len()
                );
                assert!(
                    m.release_reads
                        .iter()
                        .all(|&(tid, on_ctrl, in_rel)| tid == exec_tid && on_ctrl && in_rel),
                    "unexpected release marks: {:?}",
                    m.release_reads
                );
                assert_eq!(
                    m.drop_mark,
                    Some((exec_tid, true, false)),
                    "control thread must remain marked through core Drop on thread exit, outside ReleaseGuard"
                );
            }
            assert_eq!(sup.inner().control_thread_attempts(), 0);

            // (b) Early `fail_init` exit path: `core` drops on `armd-control` before any release.
            let fail_marks = Arc::new(Mutex::new(LifecycleMarks::default()));
            let drv_fail = LifecycleAuditDriver { dof: 7, marks: Arc::clone(&fail_marks) };
            let sup_fail = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_fail)));
            let hooks = PersistTestHooks { fail_init: Some(false), ..Default::default() };
            assert!(
                sup_fail
                    .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks)
                    .await
                    .is_err()
            );
            let fm = fail_marks.lock().unwrap();
            let (drop_tid, on_ctrl, in_rel) = fm.drop_mark.expect("driver dropped on fail_init");
            assert_ne!(drop_tid, thread::current().id());
            assert!(on_ctrl, "control thread must be marked even on early fail_init exit");
            assert!(!in_rel, "ReleaseGuard must not be active during early fail_init exit");
            assert_eq!(sup_fail.inner().control_thread_attempts(), 0);
        });
    });
}

// ============================================================================
// 2. All 7 SupervisorLock forms on marked thread + off-thread futures + timing
// ============================================================================

fn assert_all_seven_forms_blocked(
    lock: &Arc<SupervisorLock<u32>>,
    expected_msg: &str,
    expected_gen: u64,
    expect_count_increment: bool,
) {
    let cases: [(&str, fn(&Arc<SupervisorLock<u32>>)); 7] = [
        ("SupervisorLock::read", |l| drop(l.read())),
        ("SupervisorLock::try_read", |l| { let _ = l.try_read(); }),
        ("SupervisorLock::write", |l| drop(l.write())),
        ("SupervisorLock::try_write", |l| { let _ = l.try_write(); }),
        ("SupervisorLock::blocking_write", |l| { let _ = l.blocking_write(); }),
        ("SupervisorLock::write_owned", |l| drop(Arc::clone(l).write_owned())),
        ("SupervisorLock::write_tick", |l| drop(l.write_tick())),
    ];
    for (site, invoke) in cases {
        let before = lock.control_thread_attempts();
        assert_panics_with(&[expected_msg, site], || invoke(lock));
        let expected_after = if expect_count_increment { before + 1 } else { before };
        assert_eq!(
            lock.control_thread_attempts(),
            expected_after,
            "{site} unexpected attempt count delta (expect_increment={expect_count_increment})"
        );
        assert_eq!(
            lock.generation(),
            expected_gen,
            "{site} must not touch lock or bump generation"
        );
    }
}

#[test]
fn all_seven_forms_count_and_assert_on_marked_thread_before_touching_lock() {
    let deadline = Instant::now() + Duration::from_secs(5);

    // (a) Contended lock: another thread holds the writer throughout so touching the lock
    // would deadlock (`blocking_write`), return `WouldBlock` (`try_*`), or queue a waiter.
    let lock = Arc::new(SupervisorLock::new(0u32));
    let (held_tx, held_rx) = mpsc::sync_channel::<()>(1);
    let (rel_tx, rel_rx) = mpsc::sync_channel::<()>(1);
    let lock_holder = Arc::clone(&lock);
    let holder = thread::Builder::new()
        .name("test-lock-holder".into())
        .spawn(move || {
            let _w = lock_holder.blocking_write();
            let _ = held_tx.send(());
            let _ = rel_rx.recv_timeout(Duration::from_secs(5));
        })
        .unwrap();
    held_rx
        .recv_timeout(deadline.saturating_duration_since(Instant::now()))
        .expect("holder acquired writer");
    assert_eq!((lock.generation(), lock.control_thread_attempts()), (1, 0));

    let lock_marked = Arc::clone(&lock);
    thread::Builder::new()
        .name("test-marked-7forms".into())
        .spawn(move || {
            let _guard = ControlThreadGuard::enter();
            assert!(on_rt_control_thread() && !in_rt_release());
            assert_all_seven_forms_blocked(&lock_marked, "RT control thread", 1, true);
        })
        .unwrap()
        .join()
        .expect("marked thread checks");
    let _ = rel_tx.send(());
    holder.join().expect("holder thread");
    assert_eq!(lock.control_thread_attempts(), 7);

    // (b) Uncontended lock: none of the 7 forms may acquire the free lock, bump generation,
    // or log a hold record when invoked on a marked thread.
    let free_lock = Arc::new(SupervisorLock::new(42u32));
    free_lock.take_log();
    {
        let _guard = ControlThreadGuard::enter();
        assert!(!in_rt_release());
        assert_all_seven_forms_blocked(&free_lock, "RT control thread", 0, true);
    }
    assert_eq!((free_lock.control_thread_attempts(), free_lock.generation()), (7, 0));
    assert!(free_lock.take_log().unwrap().last_release.is_none());

    // (c) ReleaseGuard alone (unmarked thread): enforces the release assertion for all 7
    // forms WITHOUT misattributing the attempt to `control_thread_attempts()`.
    let rel_lock = Arc::new(SupervisorLock::new(0u32));
    {
        let _rel = ReleaseGuard::enter();
        assert!(!on_rt_control_thread() && in_rt_release());
        assert_all_seven_forms_blocked(&rel_lock, "during RT release", 0, false);
    }
    assert_eq!((rel_lock.control_thread_attempts(), rel_lock.generation()), (0, 0));
}

fn poll_once_on_marked_thread<F: Future + Send + 'static>(
    fut: Pin<Box<F>>,
    site: &'static str,
) -> Pin<Box<F>> {
    thread::Builder::new()
        .name("test-marked-poll".into())
        .spawn(move || {
            let _guard = ControlThreadGuard::enter();
            assert!(on_rt_control_thread());
            assert!(!in_rt_release());
            let mut fut = fut;
            let waker = noop_waker();
            let mut cx = Context::from_waker(waker);
            assert_panics_with(&["RT control thread", site], || {
                let _ = fut.as_mut().poll(&mut cx);
            });
            fut
        })
        .unwrap()
        .join()
        .expect("marked poll thread")
}

#[test]
fn async_futures_constructed_off_thread_count_and_assert_on_every_marked_poll() {
    let lock = Arc::new(SupervisorLock::new(0u32));

    // (a) Futures constructed off-thread (unpolled) and polled twice on a marked thread:
    // construction counts 0; each poll on the marked thread counts +1 before asserting
    // and before touching the free lock.
    let owned_fut = Box::pin(Arc::clone(&lock).write_owned());
    assert_eq!(lock.control_thread_attempts(), 0);

    // Poll `write_owned` directly on marked thread twice.
    let owned_fut = poll_once_on_marked_thread(owned_fut, "SupervisorLock::write_owned");
    assert_eq!(lock.control_thread_attempts(), 1);
    assert_eq!(lock.generation(), 0);
    let _owned_fut = poll_once_on_marked_thread(owned_fut, "SupervisorLock::write_owned");
    assert_eq!(lock.control_thread_attempts(), 2);
    assert_eq!(lock.generation(), 0);

    // Also test borrowed-lifetime futures (`read`, `write`, `write_tick`) constructed off-thread
    // on the current (unmarked) thread, then polled on a marked thread via `thread::scope`!
    let mut read_direct = Box::pin(lock.read());
    let mut write_direct = Box::pin(lock.write());
    let mut tick_direct = Box::pin(lock.write_tick());
    assert_eq!(lock.control_thread_attempts(), 2);

    thread::scope(|s| {
        s.spawn(|| {
            let _guard = ControlThreadGuard::enter();
            assert!(!in_rt_release());
            let waker = noop_waker();
            let mut cx = Context::from_waker(waker);

            for (site, expected_after) in [
                ("SupervisorLock::read", 3u64),
                ("SupervisorLock::read", 4u64),
            ] {
                assert_panics_with(&["RT control thread", site], || {
                    let _ = read_direct.as_mut().poll(&mut cx);
                });
                assert_eq!(lock.control_thread_attempts(), expected_after);
            }
            for (site, expected_after) in [
                ("SupervisorLock::write", 5u64),
                ("SupervisorLock::write", 6u64),
            ] {
                assert_panics_with(&["RT control thread", site], || {
                    let _ = write_direct.as_mut().poll(&mut cx);
                });
                assert_eq!(lock.control_thread_attempts(), expected_after);
            }
            for (site, expected_after) in [
                ("SupervisorLock::write_tick", 7u64),
                ("SupervisorLock::write_tick", 8u64),
            ] {
                assert_panics_with(&["RT control thread", site], || {
                    let _ = tick_direct.as_mut().poll(&mut cx);
                });
                assert_eq!(lock.control_thread_attempts(), expected_after);
            }
        });
    });
    assert_eq!(lock.generation(), 0, "free lock must never be acquired on marked thread");
    drop((read_direct, write_direct, tick_direct));

    // (b) Futures constructed AND polled ONCE off-thread while contended (`Poll::Pending`),
    // then the lock is released so the underlying Tokio future WOULD grant immediately if
    // polled — polling on the marked thread must still intercept, count, and panic BEFORE
    // touching the underlying future!
    let lock2 = Arc::new(SupervisorLock::new(0u32));
    let waker = noop_waker();
    let mut cx = Context::from_waker(waker);

    let mut read_pending = Box::pin(lock2.read());
    let mut write_pending = Box::pin(lock2.write());
    let mut owned_pending = Box::pin(Arc::clone(&lock2).write_owned());
    let mut tick_pending = Box::pin(lock2.write_tick());

    {
        let _held = lock2.blocking_write();
        assert!(matches!(read_pending.as_mut().poll(&mut cx), Poll::Pending));
        assert!(matches!(write_pending.as_mut().poll(&mut cx), Poll::Pending));
        assert!(matches!(owned_pending.as_mut().poll(&mut cx), Poll::Pending));
        assert!(matches!(tick_pending.as_mut().poll(&mut cx), Poll::Pending));
        assert_eq!(lock2.control_thread_attempts(), 0);
    }
    // `_held` is dropped: lock2 is now FREE (generation == 1).
    assert_eq!(lock2.generation(), 1);

    thread::scope(|s| {
        s.spawn(|| {
            let _guard = ControlThreadGuard::enter();
            assert!(!in_rt_release());
            let waker = noop_waker();
            let mut cx = Context::from_waker(waker);

            assert_panics_with(&["RT control thread", "SupervisorLock::read"], || {
                let _ = read_pending.as_mut().poll(&mut cx);
            });
            assert_eq!(lock2.control_thread_attempts(), 1);

            assert_panics_with(&["RT control thread", "SupervisorLock::write"], || {
                let _ = write_pending.as_mut().poll(&mut cx);
            });
            assert_eq!(lock2.control_thread_attempts(), 2);

            assert_panics_with(&["RT control thread", "SupervisorLock::write_owned"], || {
                let _ = owned_pending.as_mut().poll(&mut cx);
            });
            assert_eq!(lock2.control_thread_attempts(), 3);

            assert_panics_with(&["RT control thread", "SupervisorLock::write_tick"], || {
                let _ = tick_pending.as_mut().poll(&mut cx);
            });
            assert_eq!(lock2.control_thread_attempts(), 4);
        });
    });

    assert_eq!(
        lock2.generation(),
        1,
        "second poll on marked thread must not touch underlying Tokio future or bump generation"
    );
    drop((read_pending, write_pending, owned_pending, tick_pending));
    // Lock remains usable and uncorrupted on unmarked threads.
    assert!(lock2.try_write().is_ok());
    assert_eq!(lock2.generation(), 2);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn callsite_timing_and_location_preserved_for_all_timed_forms() {
    let lock = Arc::new(SupervisorLock::new(0u32));
    lock.take_log();

    // Sync `try_read`, `try_write`, `blocking_write` preserve callsite line and hold duration.
    let try_read_line = line!() + 1;
    let g = lock.try_read().unwrap();
    thread::sleep(Duration::from_millis(4));
    drop(g);
    let log = lock.take_log().unwrap();
    let rec = log.last_release.unwrap();
    assert_eq!((rec.kind, rec.location.file(), rec.location.line()), (HoldKind::Read, file!(), try_read_line));
    assert!(rec.hold() >= Duration::from_millis(4), "{rec:?}");

    let try_write_line = line!() + 1;
    let g = lock.try_write().unwrap();
    thread::sleep(Duration::from_millis(4));
    drop(g);
    let rec = lock.take_log().unwrap().last_release.unwrap();
    assert_eq!((rec.kind, rec.location.file(), rec.location.line()), (HoldKind::Write, file!(), try_write_line));
    assert!(rec.hold() >= Duration::from_millis(4), "{rec:?}");

    // `blocking_write` must run off the Tokio runtime worker thread.
    let lock_b = Arc::clone(&lock);
    let blocking_line = tokio::task::spawn_blocking(move || {
        let call_line = line!() + 1;
        let g = lock_b.blocking_write();
        thread::sleep(Duration::from_millis(4));
        drop(g);
        call_line
    })
    .await
    .unwrap();
    let rec = lock.take_log().unwrap().last_release.unwrap();
    assert_eq!((rec.kind, rec.location.file(), rec.location.line()), (HoldKind::Write, file!(), blocking_line));
    assert!(rec.hold() >= Duration::from_millis(4), "{rec:?}");

    // Async `read`, `write`, `write_owned` capture `called` at callsite (NOT at first poll):
    // delay first poll by 10 ms against an AVAILABLE lock so only callsite capture satisfies `acquire >= 8ms`.
    let read_line = line!() + 1;
    let fut = lock.read();
    tokio::time::sleep(Duration::from_millis(10)).await;
    let g = fut.await;
    thread::sleep(Duration::from_millis(4));
    drop(g);
    let rec = lock.take_log().unwrap().last_release.unwrap();
    assert_eq!((rec.kind, rec.location.file(), rec.location.line()), (HoldKind::Read, file!(), read_line));
    assert!(rec.acquire() >= Duration::from_millis(8), "read() must capture called at callsite: {rec:?}");
    assert!(rec.hold() >= Duration::from_millis(4), "{rec:?}");

    let write_line = line!() + 1;
    let fut = lock.write();
    tokio::time::sleep(Duration::from_millis(10)).await;
    let g = fut.await;
    thread::sleep(Duration::from_millis(4));
    drop(g);
    let rec = lock.take_log().unwrap().last_release.unwrap();
    assert_eq!((rec.kind, rec.location.file(), rec.location.line()), (HoldKind::Write, file!(), write_line));
    assert!(rec.acquire() >= Duration::from_millis(8), "write() must capture called at callsite: {rec:?}");
    assert!(rec.hold() >= Duration::from_millis(4), "{rec:?}");

    let owned_line = line!() + 1;
    let fut = Arc::clone(&lock).write_owned();
    tokio::time::sleep(Duration::from_millis(10)).await;
    let g = fut.await;
    thread::sleep(Duration::from_millis(4));
    drop(g);
    let rec = lock.take_log().unwrap().last_release.unwrap();
    assert_eq!((rec.kind, rec.location.file(), rec.location.line()), (HoldKind::Write, file!(), owned_line));
    assert!(rec.acquire() >= Duration::from_millis(8), "write_owned() must capture called at callsite: {rec:?}");
    assert!(rec.hold() >= Duration::from_millis(4), "{rec:?}");
    assert_eq!(lock.control_thread_attempts(), 0);
}

// ============================================================================
// 3. Production executor fixtures holding the detached legacy writer on another thread
// ============================================================================

#[test]
fn production_executor_mock_owner_zero_attempts_while_legacy_writer_held() {
    run_bounded_fixture(Duration::from_secs(15), "mock-owner", |deadline| {
        let rt = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(2)
            .enable_all()
            .build()
            .unwrap();
        rt.block_on(async move {
            // Match the production `--follower mock` executor startup owner gate.
            validate_startup_owner(RtOwner::Executor, "mock").expect("--follower mock owner allowed");

            let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
            sup.set_lease_enabled(false).await;
            sup.register_ws_connection(1).await.unwrap();
            sup.enable_torque(sup.capture_admission_stamp()).await.expect("enable torque on mock arm");

            // Lock-free executor-side trigger inside `before_publish`: accumulates `dt`-clamped
            // trajectory time (matching `SupervisorInner::tick_control`'s 80 ms clamp) after the
            // jog is Ack'd and triggers the urgent stop at >= 3 releases and >= 700 ms of a 10 s
            // quintic (zero race with test-thread scheduling or tick jitter).
            let base = Instant::now();
            let stop_cell: Arc<OnceLock<StopRequests>> = Arc::new(OnceLock::new());
            let jog_armed = Arc::new(AtomicBool::new(false));
            let active_pubs = Arc::new(AtomicU64::new(0));
            let prev_pub_us = Arc::new(AtomicU64::new(u64::MAX));
            let sim_us = Arc::new(AtomicU64::new(0));
            let triggered_gen = Arc::new(AtomicU64::new(0));

            let stop_cell_h = Arc::clone(&stop_cell);
            let armed_h = Arc::clone(&jog_armed);
            let pubs_h = Arc::clone(&active_pubs);
            let prev_h = Arc::clone(&prev_pub_us);
            let sim_h = Arc::clone(&sim_us);
            let trig_h = Arc::clone(&triggered_gen);
            let hooks = PersistTestHooks {
                before_publish: Some(Arc::new(move || {
                    if !armed_h.load(Ordering::Acquire) || trig_h.load(Ordering::Acquire) != 0 {
                        return;
                    }
                    let now_us = base.elapsed().as_micros() as u64;
                    let prev = prev_h.swap(now_us, Ordering::AcqRel);
                    let dt_us = if prev == u64::MAX {
                        0
                    } else {
                        now_us.saturating_sub(prev).min(80_000)
                    };
                    let n = pubs_h.fetch_add(1, Ordering::AcqRel) + 1;
                    let acc = sim_h.fetch_add(dt_us, Ordering::AcqRel) + dt_us;
                    if n >= 3 && acc >= 700_000 {
                        if let Some(stops) = stop_cell_h.get() {
                            let gen = stops.request();
                            let _ = trig_h.compare_exchange(0, gen, Ordering::AcqRel, Ordering::Acquire);
                        }
                    }
                })),
                ..Default::default()
            };

            let handle = sup
                .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks)
                .await
                .expect("start production executor for --follower mock");
            assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);
            stop_cell
                .set(handle.stop_requests().clone())
                .expect("attach StopRequests to mock before_publish hook");

            // Acquire and hold the detached legacy writer FIRST before starting motion.
            let writer_hold = DetachedLegacyWriterHold::acquire(Arc::clone(&sup), deadline);
            assert!(
                sup.inner().try_read().is_err(),
                "detached legacy SupervisorLock must be exclusively held by the other thread"
            );

            // Require >= 3 publication advances while the detached legacy writer is held.
            let seq_before = sup.hub().published_seq();
            wait_until_deadline(deadline, ">= 3 initial publication advances while writer held", || {
                sup.hub().published_seq() >= seq_before + 3
            });

            // Start a 10 s quintic jog while the legacy writer is held; executor-side hook
            // triggers the urgent stop at ~600 ms while the plan is guaranteed active.
            let snap = handle.planning_snapshot().expect("planning snapshot while writer held");
            let start = snap.commanded.clone();
            let mut goal = start.clone();
            goal[0] += 0.50;
            let req_jog = submit_joint_quintic(&sup, &handle, 1, snap.authority, &start, &goal, 10.0);
            assert!(matches!(
                wait_broker_reply(&handle, req_jog, deadline, |r| matches!(r, Reply::Ack { .. }).then_some(r)),
                Reply::Ack { .. }
            ));
            jog_armed.store(true, Ordering::Release);

            // Wait for executor-side urgent stop to fire and be applied.
            wait_until_deadline(deadline, "urgent stop requested and applied while writer held", || {
                let gen = triggered_gen.load(Ordering::Acquire);
                gen > 0 && handle.stop_requests().applied() == gen
            });
            let stop_gen = triggered_gen.load(Ordering::Acquire);

            // Wait for bounded deceleration to reach rest, then require >= 3 more publication
            // advances with dynamic output inhibited (positions stay frozen well short of goal[0]).
            wait_until_deadline(
                deadline,
                "mock motion settled to Armed with zero velocity after urgent stop",
                || {
                    sup.hub().latest().is_some_and(|s| {
                        s.obs.safety_state == SafetyState::Armed
                            && s.obs.velocities.iter().all(|v| v.abs() < 1e-4)
                    })
                },
            );
            let frozen_snap = sup.hub().latest().expect("published snapshot after stop");
            let frozen_seq = frozen_snap.meta.seq;
            let frozen_pos = frozen_snap.obs.positions.clone();
            assert!(
                frozen_pos[0] > start[0] + 2e-4,
                "expected active motion before urgent stop: start={}, frozen={}",
                start[0],
                frozen_pos[0]
            );
            assert!(
                frozen_pos[0] < goal[0] - 0.05,
                "urgent stop did not inhibit motion before reaching target: frozen={}, goal={}",
                frozen_pos[0],
                goal[0]
            );

            // A stale-generation command stamped with the pre-stop authority must be refused `STOPPED`.
            let req_stale = submit_joint_quintic(&sup, &handle, 2, snap.authority, &frozen_pos, &goal, 1.0);
            assert!(matches!(
                wait_broker_reply(&handle, req_stale, deadline, |r| matches!(r, Reply::Refused { .. }).then_some(r)),
                Reply::Refused { code: "STOPPED", .. }
            ));

            wait_until_deadline(deadline, ">= 3 post-stop publication advances while writer held", || {
                sup.hub().published_seq() >= frozen_seq + 3
            });
            let after_snap = sup.hub().latest().expect("post-stop snapshot");
            assert!(
                after_snap
                    .obs
                    .positions
                    .iter()
                    .zip(&frozen_pos)
                    .all(|(a, b)| (a - b).abs() <= 1e-5),
                "dynamic output continued after urgent stop: before={frozen_pos:?}, after={:?}",
                after_snap.obs.positions
            );

            // Collect final invariants strictly BEFORE releasing the detached legacy writer.
            assert_eq!(handle.stop_requests().applied(), stop_gen);
            assert!(!handle.is_exited(), "control thread must stay alive without panicking");
            assert_eq!(
                sup.inner().control_thread_attempts(),
                0,
                "RT control thread attempted SupervisorLock"
            );
            assert!(
                sup.inner().try_read().is_err(),
                "detached legacy writer must remain held until after all assertions"
            );

            writer_hold.release();
            handle.stop_and_join();
            assert_eq!(sup.inner().control_thread_attempts(), 0);
        });
    });
}

/// Recording CAN bus wrapper over `FakeBus` that captures every transmitted frame and
/// triggers a lock-free mid-batch urgent stop right after `0x102` once the bus itself has
/// observed >= 3 advancing `0x101` frames past the configured threshold.
#[derive(Clone)]
struct TracingBus {
    inner: FakeBus,
    tx_trace: Arc<Mutex<Vec<CanFrame>>>,
    stop_cell: Arc<OnceLock<StopRequests>>,
    arm_after_j1_threshold_bits: Arc<AtomicU64>,
    advancing_j1_seen: Arc<AtomicU64>,
    triggered_gen: Arc<AtomicU64>,
    trigger_index: Arc<AtomicUsize>,
}

impl TracingBus {
    fn new(inner: FakeBus) -> Self {
        Self {
            inner,
            tx_trace: Arc::new(Mutex::new(Vec::with_capacity(2048))),
            stop_cell: Arc::new(OnceLock::new()),
            arm_after_j1_threshold_bits: Arc::new(AtomicU64::new(0)),
            advancing_j1_seen: Arc::new(AtomicU64::new(0)),
            triggered_gen: Arc::new(AtomicU64::new(0)),
            trigger_index: Arc::new(AtomicUsize::new(usize::MAX)),
        }
    }

    fn arm_mid_batch_stop_after_j1_progress(&self, threshold_rad: f32) {
        // Store a non-zero sentinel encoding `(1 << 32) | threshold_rad.to_bits()`.
        self.advancing_j1_seen.store(0, Ordering::Release);
        let encoded = (1u64 << 32) | u64::from(threshold_rad.to_bits());
        self.arm_after_j1_threshold_bits.store(encoded, Ordering::Release);
    }

    fn trace_len(&self) -> usize {
        self.tx_trace.lock().unwrap().len()
    }

    fn trace_slice_from(&self, start: usize) -> Vec<CanFrame> {
        let log = self.tx_trace.lock().unwrap();
        log[start.min(log.len())..].to_vec()
    }
}

impl CanTransport for TracingBus {
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        self.inner.transmit(frame)?;
        let idx = {
            let mut log = self.tx_trace.lock().unwrap();
            let idx = log.len();
            log.push(*frame);
            idx
        };
        let encoded = self.arm_after_j1_threshold_bits.load(Ordering::Acquire);
        if encoded != 0 {
            if frame.id == 0x101 {
                let threshold = f32::from_bits(encoded as u32);
                if decode_pos_vel_position(frame) > threshold {
                    self.advancing_j1_seen.fetch_add(1, Ordering::AcqRel);
                }
            } else if frame.id == 0x102
                && self.advancing_j1_seen.load(Ordering::Acquire) >= 3
                && self
                    .arm_after_j1_threshold_bits
                    .compare_exchange(encoded, 0, Ordering::AcqRel, Ordering::Acquire)
                    .is_ok()
            {
                if let Some(stops) = self.stop_cell.get() {
                    let gen = stops.request();
                    self.trigger_index.store(idx, Ordering::Release);
                    self.triggered_gen.store(gen, Ordering::Release);
                }
            }
        }
        Ok(())
    }

    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        self.inner.receive()
    }

    fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<std::time::Instant>), TransportError> {
        self.inner.receive_timestamped()
    }

    fn flush(&mut self) -> Result<(), TransportError> {
        self.inner.flush()
    }

    fn flush_is_noop(&self) -> bool {
        self.inner.flush_is_noop()
    }

    fn clock(&self) -> Option<Arc<dyn DriverClock>> {
        self.inner.clock()
    }

    fn supports_bounded_flush(&self) -> bool {
        self.inner.supports_bounded_flush()
    }

    fn supports_nonblocking_rt(&self) -> bool {
        self.inner.supports_nonblocking_rt()
    }
}

fn decode_pos_vel_position(frame: &CanFrame) -> f32 {
    f32::from_le_bytes([frame.data[0], frame.data[1], frame.data[2], frame.data[3]])
}

fn is_damiao_target_frame(id: u32) -> bool {
    (0x101..=0x107).contains(&id) || (0x301..=0x307).contains(&id)
}

#[test]
fn production_executor_bus_trace_dynamic_output_inhibited_while_legacy_writer_held() {
    run_bounded_fixture(Duration::from_secs(15), "damiao-bus-trace", |deadline| {
        let rt = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(2)
            .enable_all()
            .build()
            .unwrap();
        rt.block_on(async move {
            // Use an injected driver clock and a 20 ms release period matching `ArmConfig::rebot_dm()`
            // (50 Hz) so OS scheduling jitter under CPU load cannot spuriously trip wall-clock
            // `DAMIAO_TX_FRAME_MAX_AGE` (45 ms), `resolved_tx_stall_ticks()` (4), or `BUS_LOSS_TIMEOUT`.
            let fake_bus = FakeBus::with_clock(&dm_limits(), MockClock::new());
            fake_bus.with(|s| {
                s.immediate_replies = true;
                s.record_tx_log = false;
                s.follow_commands = true;
                s.push_all_feedback();
            });
            let bus = TracingBus::new(fake_bus);
            let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
                .with_joint_limits(dm_limits())
                .unwrap();
            drv.read_joint_states().unwrap();
            drv.check_mapping_ranges();

            let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
            sup.set_lease_enabled(false).await;
            sup.register_ws_connection(1).await.unwrap();
            let handle = sup
                .start_executor(ExecutorConfig::for_tests(Duration::from_millis(20)))
                .await
                .expect("start production executor with DamiaoArmDriver<TracingBus>");
            assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);
            bus.stop_cell
                .set(handle.stop_requests().clone())
                .expect("attach StopRequests to TracingBus");

            // Enable torque via broker before acquiring the detached writer.
            let req_on = submit_broker_transition(&handle, 1, TransitionOp::TorqueOn);
            assert_eq!(wait_broker_terminal(&handle, req_on, deadline), Outcome::Completed);

            // Hold the detached legacy writer FIRST before starting the jog.
            let writer_hold = DetachedLegacyWriterHold::acquire(Arc::clone(&sup), deadline);
            assert!(
                sup.inner().try_read().is_err(),
                "detached legacy SupervisorLock must be exclusively held by the other thread"
            );

            // 1. Verify >= 3 publication advances while the detached legacy writer is held.
            let seq_before = sup.hub().published_seq();
            wait_until_deadline(
                deadline,
                ">= 3 initial publication advances while legacy writer held",
                || sup.hub().published_seq() >= seq_before + 3,
            );

            // 2. Arm the bus-driven mid-batch stop and start a 10 s quintic jog while the
            // detached legacy writer is held. `TracingBus` fires `stops.request()` right after
            // `0x102` once >= 3 advancing `0x101` frames exceed `start[0] + 5e-4` (zero race
            // with test-thread scheduling).
            let snap = handle.planning_snapshot().expect("planning snapshot while writer held");
            let start = snap.commanded.clone();
            let mut goal = start.clone();
            goal[0] += 0.50;
            let trace_before_jog = bus.trace_len();
            bus.arm_mid_batch_stop_after_j1_progress(start[0] + 5e-4);

            let req_jog = submit_joint_quintic(&sup, &handle, 2, snap.authority, &start, &goal, 10.0);
            assert!(matches!(
                wait_broker_reply(&handle, req_jog, deadline, |r| matches!(r, Reply::Ack { .. }).then_some(r)),
                Reply::Ack { .. }
            ));

            wait_until_deadline(deadline, "bus-triggered mid-batch urgent stop applied", || {
                assert!(!handle.is_exited(), "executor exited unexpectedly before mid-batch stop");
                if let Some(s) = sup.hub().latest() {
                    assert!(
                        !matches!(s.obs.safety_state, SafetyState::Fault(_)),
                        "executor faulted before mid-batch stop: state={:?} j1_seen={} triggered_gen={} applied={}",
                        s.obs.safety_state,
                        bus.advancing_j1_seen.load(Ordering::Acquire),
                        bus.triggered_gen.load(Ordering::Acquire),
                        handle.stop_requests().applied(),
                    );
                }
                let gen = bus.triggered_gen.load(Ordering::Acquire);
                gen > 0 && handle.stop_requests().applied() == gen
            });
            let stop_gen = bus.triggered_gen.load(Ordering::Acquire);
            let trigger_idx = bus.trigger_index.load(Ordering::Acquire);

            // Confirm pre-stop bus trace recorded >= 3 advancing `0x101` frames before the stop.
            let pre_stop_j1: Vec<f32> = bus
                .trace_slice_from(trace_before_jog)
                .into_iter()
                .take(trigger_idx.saturating_sub(trace_before_jog) + 1)
                .filter(|f| f.id == 0x101)
                .map(|f| decode_pos_vel_position(&f))
                .collect();
            assert!(
                pre_stop_j1.len() >= 3 && *pre_stop_j1.last().unwrap() > start[0] + 5e-4,
                "expected >= 3 advancing 0x101 frames before stop: {pre_stop_j1:?}"
            );

            // 3. Wait for bounded braking to reach rest (`SafetyState::Armed`, i.e. commanded_velocities == 0),
            // then capture >= 3 post-settle publications and bus frames while the legacy writer is STILL held.
            wait_until_deadline(deadline, "bus motion settled to Armed after urgent stop", || {
                sup.hub()
                    .latest()
                    .is_some_and(|s| s.obs.safety_state == SafetyState::Armed)
            });
            let post_settle_seq = sup.hub().published_seq();
            let post_settle_trace_start = bus.trace_len();

            // Stale-generation jog using the pre-stop authority must be refused `STOPPED`.
            let frozen_pos = sup
                .hub()
                .latest()
                .expect("snapshot after stop")
                .obs
                .commanded_positions
                .clone();
            let req_stale = submit_joint_quintic(&sup, &handle, 3, snap.authority, &frozen_pos, &goal, 1.0);
            assert!(matches!(
                wait_broker_reply(&handle, req_stale, deadline, |r| matches!(r, Reply::Refused { .. }).then_some(r)),
                Reply::Refused { code: "STOPPED", .. }
            ));

            wait_until_deadline(
                deadline,
                ">= 3 post-settle publication advances and hold frames on bus while writer held",
                || {
                    sup.hub().published_seq() >= post_settle_seq + 3
                        && bus
                            .trace_slice_from(post_settle_trace_start)
                            .iter()
                            .filter(|f| f.id == 0x101)
                            .count()
                            >= 3
                },
            );

            // 4. Inspect CAN bus trace:
            // (a) In the interrupted batch, joints 3..7 (0x103..=0x107) were inhibited mid-batch:
            //     the very next target frame after `0x102` at `trigger_idx` must be `0x101`.
            let full_after_trigger = bus.trace_slice_from(trigger_idx);
            assert_eq!(full_after_trigger[0].id, 0x102);
            let next_target_after_trigger = full_after_trigger[1..]
                .iter()
                .find(|f| is_damiao_target_frame(f.id))
                .expect("expected subsequent hold batch after mid-batch stop");
            assert_eq!(
                next_target_after_trigger.id, 0x101,
                "mid-batch urgent stop failed to inhibit remaining joints 0x103..=0x107: next target was 0x{:X}",
                next_target_after_trigger.id
            );

            // (b) After braking settled, every 0x101 bus frame holds constant position well short of goal.
            let post_j1: Vec<f32> = bus
                .trace_slice_from(post_settle_trace_start)
                .into_iter()
                .filter(|f| f.id == 0x101)
                .map(|f| decode_pos_vel_position(&f))
                .collect();
            assert!(post_j1.len() >= 3, "expected >= 3 post-stop hold frames, got {}", post_j1.len());
            let held_p = post_j1[0];
            assert!(
                held_p > start[0] + 5e-4 && held_p < goal[0] - 0.05,
                "urgent stop did not inhibit dynamic trajectory before target: start={}, held={held_p}, goal={}",
                start[0],
                goal[0]
            );
            assert!(
                post_j1.iter().all(|&p| (p - held_p).abs() <= 1e-5),
                "dynamic output continued on CAN bus after urgent stop: {post_j1:?}"
            );

            // 5. Collect final invariants strictly BEFORE releasing the detached legacy writer.
            assert_eq!(handle.stop_requests().applied(), stop_gen);
            assert!(!handle.is_exited(), "control thread must stay alive without panicking");
            assert_eq!(
                sup.inner().control_thread_attempts(),
                0,
                "RT control thread attempted SupervisorLock"
            );
            assert!(
                sup.inner().try_read().is_err(),
                "detached legacy writer must remain held until after all assertions"
            );

            writer_hold.release();
            handle.stop_and_join();
            assert_eq!(sup.inner().control_thread_attempts(), 0);
        });
    });
}

#[test]
fn production_executor_mock_positive_control_counts_and_catches_try_read_on_armd_control() {
    run_bounded_fixture(Duration::from_secs(15), "mock-positive-control", |deadline| {
        let rt = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(2)
            .enable_all()
            .build()
            .unwrap();
        rt.block_on(async move {
            validate_startup_owner(RtOwner::Executor, "mock").expect("--follower mock owner allowed");

            let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
            sup.set_lease_enabled(false).await;
            sup.register_ws_connection(1).await.unwrap();
            sup.enable_torque(sup.capture_admission_stamp()).await.expect("enable torque on mock arm");

            let inner_lock = Arc::clone(sup.inner());
            let probe_armed = Arc::new(AtomicBool::new(false));
            let probe_done = Arc::new(AtomicBool::new(false));
            let probe_record: Arc<Mutex<Option<(ThreadId, bool, u64, u64, String)>>> =
                Arc::new(Mutex::new(None));

            let armed_h = Arc::clone(&probe_armed);
            let done_h = Arc::clone(&probe_done);
            let rec_h = Arc::clone(&probe_record);
            let hooks = PersistTestHooks {
                before_publish: Some(Arc::new(move || {
                    if !armed_h.swap(false, Ordering::AcqRel) {
                        return;
                    }
                    // Assert the production control-thread mark directly inside the hook on armd-control.
                    assert!(
                        on_rt_control_thread(),
                        "before_publish hook must execute on the marked armd-control thread"
                    );
                    let before = inner_lock.control_thread_attempts();
                    let panic_msg = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        let _ = inner_lock.try_read();
                    }))
                    .expect_err("SupervisorLock::try_read must panic on armd-control");
                    let after = inner_lock.control_thread_attempts();
                    *rec_h.lock().unwrap() = Some((
                        thread::current().id(),
                        on_rt_control_thread(),
                        before,
                        after,
                        panic_payload_to_string(panic_msg),
                    ));
                    done_h.store(true, Ordering::Release);
                })),
                ..Default::default()
            };

            let handle = sup
                .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks)
                .await
                .expect("start production executor for mock positive control");
            assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);
            let exec_tid = handle.executor_thread_id();

            // Hold the same detached `sup.inner()` lock as the contention fixture and confirm 0
            // attempts across baseline releases before arming the one-shot probe.
            let writer_hold = DetachedLegacyWriterHold::acquire(Arc::clone(&sup), deadline);
            let seq_before = sup.hub().published_seq();
            wait_until_deadline(deadline, ">= 2 baseline releases before positive control", || {
                sup.hub().published_seq() >= seq_before + 2
            });
            assert_eq!(sup.inner().control_thread_attempts(), 0);

            // Trigger exactly one `try_read()` from `before_publish` on `armd-control`.
            probe_armed.store(true, Ordering::Release);
            wait_until_deadline(deadline, "positive-control probe executed on armd-control", || {
                probe_done.load(Ordering::Acquire)
            });

            // Executor must keep running and advancing publications after catching the panic.
            let seq_after_probe = sup.hub().published_seq();
            wait_until_deadline(deadline, ">= 3 releases after caught positive-control panic", || {
                sup.hub().published_seq() >= seq_after_probe + 3
            });
            assert!(!handle.is_exited(), "executor must keep running after catching probe panic");

            let (hook_tid, marked, before, after, msg) = probe_record
                .lock()
                .unwrap()
                .clone()
                .expect("probe record populated on armd-control");
            assert_eq!(hook_tid, exec_tid, "probe must run on armd-control thread");
            assert!(marked, "armd-control thread must be marked inside before_publish");
            assert_eq!((before, after), (0, 1), "control_thread_attempts must advance 0 -> 1");
            assert!(
                msg.contains("RT control thread") && msg.contains("SupervisorLock::try_read"),
                "unexpected positive-control panic message: {msg:?}"
            );
            assert_eq!(
                sup.inner().control_thread_attempts(),
                1,
                "control_thread_attempts must stay at exactly 1 after single probe"
            );

            writer_hold.release();
            handle.stop_and_join();
            assert_eq!(sup.inner().control_thread_attempts(), 1);
        });
    });
}

#[test]
fn terminal_capture_lock_proof_action_completion_dwell_saturation_and_positive_controls() {
    use armd::driver::{ArmDriverError, DriverCapabilities, EnableState, JointTarget};
    use armd::model::JointState;
    use std::sync::atomic::AtomicUsize;

    struct CaptureAuditDriver {
        dof: usize,
        positions: Vec<f32>,
        in_capture: Arc<AtomicBool>,
        driver_calls_during_capture: Arc<AtomicUsize>,
    }

    impl CaptureAuditDriver {
        fn note_call(&self) {
            if self.in_capture.load(Ordering::Acquire) {
                self.driver_calls_during_capture.fetch_add(1, Ordering::AcqRel);
            }
        }
    }

    impl armd::driver::ArmDriver for CaptureAuditDriver {
        fn capabilities(&self) -> DriverCapabilities {
            self.note_call();
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
            self.note_call();
            EnableState::Enabled
        }
        fn is_feedback_valid(&self) -> bool {
            self.note_call();
            true
        }
        fn feedback_ages(&self) -> Vec<Option<Duration>> {
            self.note_call();
            vec![Some(Duration::from_millis(5)); self.dof]
        }
        fn feedback_instants(&self) -> Vec<Option<Instant>> {
            self.note_call();
            vec![Some(Instant::now()); self.dof]
        }
        fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
            self.note_call();
            dof == self.dof && max_age >= Duration::from_millis(5)
        }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            self.note_call();
            Ok(self
                .positions
                .iter()
                .map(|&p| JointState {
                    position: p,
                    velocity: 0.0,
                    torque: 0.0,
                    temperature: 30.0,
                })
                .collect())
        }
        fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
            self.note_call();
            for (dst, t) in self.positions.iter_mut().zip(targets.iter()) {
                *dst = t.position;
            }
            Ok(())
        }
        fn set_torque(&mut self, _enable: bool) -> Result<(), ArmDriverError> {
            self.note_call();
            Ok(())
        }
        fn supports_rt_executor(&self) -> Result<(), &'static str> {
            Ok(())
        }
    }

    run_bounded_fixture(Duration::from_secs(15), "terminal-capture-lock-proof", |deadline| {
        let rt = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(2)
            .enable_all()
            .build()
            .unwrap();
        rt.block_on(async move {
            let in_capture = Arc::new(AtomicBool::new(false));
            let driver_calls_during_capture = Arc::new(AtomicUsize::new(0));
            let drv = CaptureAuditDriver {
                dof: 7,
                positions: vec![0.0; 7],
                in_capture: Arc::clone(&in_capture),
                driver_calls_during_capture: Arc::clone(&driver_calls_during_capture),
            };
            let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
            sup.set_lease_enabled(false).await;
            sup.test_set_terminal_capture_drain_paused(true);
            sup.test_set_terminal_capture_push_cap(3);

            // 1) Hold `TerminalBroadcaster`'s internal mutex on another OS thread throughout
            //    action completion, 30 dwell ticks, and ring saturation (6 actions > cap 3).
            //    If `tick_shared`, `finish_action` (with defer), or `capture_edge_terminals`
            //    ever locked `TerminalBroadcaster`, this would deadlock / trip timeout.
            let (held_tx, held_rx) = mpsc::channel();
            let (rel_tx, rel_rx) = mpsc::channel();
            let sup_h = Arc::clone(&sup);
            let tb_holder = thread::spawn(move || {
                let _guard = sup_h.test_lock_terminal_broadcaster();
                let _ = held_tx.send(());
                let _ = rel_rx.recv();
            });
            held_rx.recv_timeout(Duration::from_secs(2)).expect("broadcaster lock held");

            // Track driver calls strictly across the entire `capture_edge_terminals` body.
            let in_cap_hook = Arc::clone(&in_capture);
            sup.inner().write().await.terminal_capture_test_hook =
                Some(Arc::new(move |_, entering| {
                    in_cap_hook.store(entering, Ordering::Release);
                }));

            // Also exercise an in-tick supersession (early `capture_from` via
            // `record_terminal_transition_for`) so that path is audited for 0 driver locks too.
            {
                let mut inner = sup.inner().write().await;
                let _superseded = inner.test_start_zero_heap_jog_rt(0.02, 0.20);
                let _replacement = inner.test_start_zero_heap_jog_rt(-0.02, 0.04);
            }
            for _ in 0..40 {
                sup.control_tick(0.02).await;
                let st = sup.test_terminal_capture_stats();
                if st.captured + st.saturated >= 2 {
                    break;
                }
            }

            // Real in-tick trajectory completions + 30 dwell ticks + saturation (6 total actions
            // into push_cap=3). `terminal_capture_test_hook` spans `capture_edge_terminals` AND
            // early `capture_from` paths so any driver mutex call in record construction or ring
            // flush is caught.
            let baseline_total = {
                let st = sup.test_terminal_capture_stats();
                st.captured + st.saturated
            };
            for i in 0..6 {
                let delta = if i % 2 == 0 { 0.02 } else { -0.02 };
                sup.inner().write().await.test_start_zero_heap_jog_rt(delta, 0.04);
                let target_total = baseline_total + (i + 1) as u64;
                for _ in 0..40 {
                    sup.control_tick(0.02).await;
                    let st = sup.test_terminal_capture_stats();
                    if st.captured + st.saturated >= target_total {
                        break;
                    }
                }
                if i == 0 {
                    let cap_after_first = sup.test_terminal_capture_stats().captured;
                    // 30 dwell ticks after first completion: 0 extra captures, 0 broadcaster locks.
                    for _ in 0..30 {
                        sup.control_tick(0.02).await;
                    }
                    assert_eq!(sup.test_terminal_capture_stats().captured, cap_after_first);
                }
            }
            let stats = sup.test_terminal_capture_stats();
            assert_eq!(stats.captured, 3, "push_cap=3: {stats:?}");
            assert!(stats.saturated >= 3, "actions past push_cap=3 must saturate: {stats:?}");
            assert!(stats.ever_saturated);
            assert_eq!(
                driver_calls_during_capture.load(Ordering::Acquire),
                0,
                "capture_edge_terminals must acquire ZERO driver mutexes"
            );

            // 2) Positive controls inside `capture_edge_terminals`:
            //    (a) Injected broadcaster mutex try_lock fails (`WouldBlock`) while held;
            //    (b) Injected driver mutex call (`hardware_feedback_fresh`) increments driver counter 0 -> 1.
            let sup_probe = Arc::clone(&sup);
            let tb_try_locked = Arc::new(AtomicBool::new(true));
            let tb_try_locked_c = Arc::clone(&tb_try_locked);
            let in_cap_pos = Arc::clone(&in_capture);
            sup.inner().write().await.terminal_capture_test_hook =
                Some(Arc::new(move |inner, entering| {
                    in_cap_pos.store(entering, Ordering::Release);
                    if entering {
                        tb_try_locked_c.store(
                            sup_probe.test_try_lock_terminal_broadcaster(),
                            Ordering::Release,
                        );
                        let _ = inner.hardware_feedback_fresh(Duration::from_millis(250));
                    }
                }));
            sup.control_tick(0.02).await;
            sup.inner().write().await.terminal_capture_test_hook = None;

            assert!(
                !tb_try_locked.load(Ordering::Acquire),
                "positive control: injected broadcaster lock inside capture MUST fail contention check"
            );
            assert!(
                driver_calls_during_capture.load(Ordering::Acquire) >= 1,
                "positive control: injected driver mutex call inside capture MUST be detected"
            );

            let _ = rel_tx.send(());
            tb_holder.join().unwrap();

            // 3) RT Executor lock proof + positive controls on `armd-control`:
            //    Attach `CaptureAuditDriver`, hold `TerminalCaptureChannel::rx`,
            //    `TerminalCaptureChannel::rich_action_cache`, `TerminalBroadcaster`, AND
            //    `DetachedLegacyWriterHold` simultaneously while completing a real RT jog + dwell.
            //    Verify 0 `SupervisorLock`, 0 `TerminalCaptureChannel` (`rx` / `rich_action_cache`),
            //    and 0 driver mutex calls during capture; then verify positive controls catch
            //    injected `SupervisorLock::try_read`, uncontended `channel.rx.lock()`, and
            //    uncontended `rich_action_cache.lock()` on `armd-control`.
            let rt_in_capture = Arc::new(AtomicBool::new(false));
            let rt_driver_calls = Arc::new(AtomicUsize::new(0));
            let rt_drv = CaptureAuditDriver {
                dof: 7,
                in_capture: Arc::clone(&rt_in_capture),
                driver_calls_during_capture: Arc::clone(&rt_driver_calls),
                positions: vec![0.0; 7],
            };
            let sup_rt = Arc::new(
                ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(rt_drv)),
            );
            sup_rt.set_lease_enabled(false).await;
            sup_rt.register_ws_connection(1).await.unwrap();
            sup_rt
                .enable_torque(sup_rt.capture_admission_stamp())
                .await
                .unwrap();
            let inner_lock = Arc::clone(sup_rt.inner());
            let sup_rt_for_hook = Arc::clone(&sup_rt);
            let rt_in_capture_hook = Arc::clone(&rt_in_capture);
            let probe_armed = Arc::new(AtomicBool::new(false));
            let probe_caught = Arc::new(AtomicBool::new(false));
            let armed_c = Arc::clone(&probe_armed);
            let caught_c = Arc::clone(&probe_caught);
            sup_rt.inner().write().await.terminal_capture_test_hook =
                Some(Arc::new(move |inner, entering| {
                    // Span the ENTIRE `capture_edge_terminals` body (`capture_from` + SPSC push):
                    // `entering == true` arms the driver audit window BEFORE record construction,
                    // `entering == false` clears it strictly AFTER the push loop completes.
                    rt_in_capture_hook.store(entering, Ordering::Release);
                    if !entering || !armed_c.swap(false, Ordering::AcqRel) {
                        return;
                    }
                    assert!(on_rt_control_thread());
                    let res_sup = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        let _ = inner_lock.try_read();
                    }));
                    assert!(
                        res_sup.is_err(),
                        "injected SupervisorLock::try_read in capture must panic"
                    );
                    let res_rx = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        sup_rt_for_hook.test_touch_terminal_capture_rx_lock();
                    }));
                    assert!(
                        res_rx.is_err(),
                        "injected uncontended TerminalCaptureChannel::rx.lock() in capture must panic"
                    );
                    let res_cache = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        sup_rt_for_hook.test_touch_terminal_capture_rich_cache_lock();
                    }));
                    assert!(
                        res_cache.is_err(),
                        "injected uncontended TerminalCaptureChannel::rich_action_cache.lock() in capture must panic"
                    );
                    // Positive control: injected driver mutex acquisition strictly inside the
                    // capture window MUST be detected by `rt_driver_calls`.
                    let _ = inner.hardware_feedback_fresh(Duration::from_millis(250));
                    caught_c.store(true, Ordering::Release);
                }));

            let cfg_snap = Arc::new(
                armd::rt_core::requests::motion::MotionConfigSnapshot::from_supervisor(
                    &*sup_rt.inner().read().await,
                ),
            );
            let handle = sup_rt
                .start_executor_with_hooks(
                    ExecutorConfig::for_tests(Duration::from_millis(5)),
                    PersistTestHooks::default(),
                )
                .await
                .unwrap();
            let writer_hold = DetachedLegacyWriterHold::acquire(Arc::clone(&sup_rt), deadline);
            // Hold ALL three capture/broadcaster mutexes simultaneously while `armd-control`
            // runs jog completion + dwell: any lock attempt would deadlock/fail before deadline.
            let _hold_rx = sup_rt.test_lock_terminal_capture_rx();
            let _hold_rich = sup_rt.test_lock_terminal_capture_rich_cache();
            let _hold_tb = sup_rt.test_lock_terminal_broadcaster();
            // Drive an actual RT jog completion + dwell under all four held locks.
            wait_until_deadline(deadline, "initial RT publication", || {
                sup_rt.hub().published_seq() >= 2
            });
            let snap = sup_rt.hub().latest().unwrap();
            let from = snap.rt.commanded_positions.clone();
            let mut to = from.clone();
            to[0] += 0.02;
            let msnap = armd::rt_core::requests::motion::MotionPlanSnapshot {
                config: cfg_snap,
                commanded_positions: from.clone(),
                commanded_velocities: vec![0.0; from.len()],
                measured_positions: from,
                torque_hold: None,
                power_hold: None,
                holding_after_clear: false,
                mit_active: false,
                authority: armd::rt_core::requests::motion::PlanAuthority {
                    boot: snap.rt.boot_id,
                    epoch: snap.obs.epoch_id,
                    lease_generation: 0,
                    stop_generation: snap.rt.stop_generation,
                    deadline: Instant::now() + Duration::from_secs(5),
                },
                workspace_revision: snap.rt.workspace_revision,
                motion_config_revision: snap.rt.motion_config_revision,
            };
            let jog_slot = armd::rt_core::requests::motion::prepare_motion(
                &msnap,
                &armd::rt_core::requests::motion::MotionRequest::jog(to, Some(0.05), None),
            )
            .unwrap()
            .slot;
            let auth = armd::rt_core::broker::Authority {
                lease_generation: 0,
                epoch: snap.obs.epoch_id,
                stop_generation: snap.rt.stop_generation,
                config_revision: 0,
                parent_generation: 0,
            };
            let _ = handle
                .broker()
                .lock()
                .unwrap()
                .submit(
                    armd::rt_core::broker::ClientId(1),
                    1,
                    auth,
                    Instant::now() + Duration::from_secs(5),
                    armd::rt_core::broker::Submit::Install(&move |s: &mut armd::rt_core::ops::PlanSlot| {
                        armd::rt_core::requests::motion::stage_jog(s, &jog_slot);
                    }),
                )
                .unwrap();
            wait_until_deadline(deadline, "RT jog completion captured under all held locks", || {
                sup_rt.test_terminal_capture_stats().captured >= 1
            });
            let cap_after_jog = sup_rt.test_terminal_capture_stats().captured;
            let seq0 = sup_rt.hub().published_seq();
            wait_until_deadline(deadline, "RT dwell releases under all held locks", || {
                sup_rt.hub().published_seq() >= seq0 + 5
            });
            assert_eq!(
                sup_rt.test_terminal_capture_stats().captured,
                cap_after_jog,
                "RT dwell must not re-capture completed jog"
            );
            assert_eq!(sup_rt.inner().control_thread_attempts(), 0);
            assert_eq!(
                sup_rt.test_terminal_capture_control_lock_attempts(),
                0,
                "armd-control must perform 0 TerminalCaptureChannel rx/rich_cache lock attempts"
            );
            assert_eq!(
                rt_driver_calls.load(Ordering::Acquire),
                0,
                "armd-control capture must perform 0 driver mutex calls"
            );

            // Release the test mutex holds so the positive controls exercise UNCONTENDED
            // channel.rx.lock() and rich_action_cache.lock() on `armd-control`.
            drop(_hold_tb);
            drop(_hold_rich);
            drop(_hold_rx);

            probe_armed.store(true, Ordering::Release);
            wait_until_deadline(deadline, "capture hook lock positive controls", || {
                probe_caught.load(Ordering::Acquire)
            });
            assert_eq!(sup_rt.inner().control_thread_attempts(), 1);
            assert_eq!(
                sup_rt.test_terminal_capture_control_lock_attempts(),
                2,
                "positive controls must catch BOTH uncontended rx.lock() and rich_action_cache.lock() on armd-control"
            );
            assert!(
                rt_driver_calls.load(Ordering::Acquire) >= 1,
                "positive control: injected driver call inside RT capture window MUST be detected"
            );
            assert_eq!(
                sup_rt.driver_lock_attempts(),
                0,
                "exclusive RT executor must perform 0 DriverLock attempts"
            );

            writer_hold.release();
            handle.stop_and_join();
        });
    });
}

// ============================================================================
// 4. DRV-LOCK: DriverLock positive control, lock-free scripted CAN transport
//    (0 lock attempts + AllocCounts::ZERO), Legacy vs Executor bus-trace parity,
//    and single-writer ownership across start failures & shutdown paths.
// ============================================================================

#[derive(Clone, Copy)]
struct ScriptedMotor {
    id: u16,
    q: f32,
    v: f32,
    tau: f32,
    enabled: bool,
    limits: DamiaoLimits,
    ctrl_mode: u32,
    temperature_c: u8,
}

impl ScriptedMotor {
    fn new(id: u16, limits: DamiaoLimits) -> Self {
        Self {
            id,
            q: 0.0,
            v: 0.0,
            tau: 0.0,
            enabled: false,
            limits,
            ctrl_mode: damiao::CTRL_MODE_POS_VEL,
            temperature_c: 30,
        }
    }

    fn reply_id(&self) -> u16 {
        0x10 + self.id
    }

    fn feedback_frame(&self) -> CanFrame {
        let l = &self.limits;
        let p = damiao::float_to_uint(self.q, l.p_min, l.p_max, 16);
        let v = damiao::float_to_uint(self.v, l.v_min, l.v_max, 12);
        let t = damiao::float_to_uint(self.tau, l.t_min, l.t_max, 12);
        let state = u8::from(self.enabled);
        let data = [
            (state << 4) | (self.id as u8 & 0x0f),
            (p >> 8) as u8,
            p as u8,
            (v >> 4) as u8,
            (((v & 0x0f) << 4) | ((t >> 8) & 0x0f)) as u8,
            t as u8,
            self.temperature_c,
            self.temperature_c,
        ];
        CanFrame::new_standard(self.reply_id(), &data).unwrap()
    }

    fn reg_reply(&self, op: u8, rid: u8, value: [u8; 4]) -> CanFrame {
        let [lo, hi] = self.id.to_le_bytes();
        CanFrame::new_standard(
            self.reply_id(),
            &[lo, hi, op, rid, value[0], value[1], value[2], value[3]],
        )
        .unwrap()
    }

    fn read_reg(&self, rid: u8) -> Option<[u8; 4]> {
        match rid {
            damiao::RID_ESC_ID => Some(u32::from(self.id).to_le_bytes()),
            damiao::RID_MST_ID => Some((0x10 + u32::from(self.id)).to_le_bytes()),
            damiao::RID_SW_VER => Some(mit_support::SW_VER.to_le_bytes()),
            damiao::RID_CTRL_MODE => Some(self.ctrl_mode.to_le_bytes()),
            damiao::RID_PMAX => Some(self.limits.p_max.to_le_bytes()),
            damiao::RID_VMAX => Some(self.limits.v_max.to_le_bytes()),
            damiao::RID_TMAX => Some(self.limits.t_max.to_le_bytes()),
            damiao::RID_TIMEOUT => Some(0u32.to_le_bytes()),
            _ => None,
        }
    }
}

/// Pure-atomic controls for [`LockFreeScriptedBus`] (zero mutexes, zero allocations).
struct LockFreeBusControls {
    stop_cell: OnceLock<StopRequests>,
    arm_after_j1_threshold_bits: AtomicU64,
    advancing_j1_seen: AtomicU64,
    triggered_gen: AtomicU64,
    trigger_tx_seq: AtomicU64,
    enobufs_target_id: AtomicU32,
    enobufs_remaining: AtomicUsize,
    enobufs_injected: AtomicU64,
    tx_seq: AtomicU64,
    tx_ring_dropped: AtomicU64,
    immediate_replies: AtomicBool,
    follow_commands: AtomicBool,
}

impl LockFreeBusControls {
    fn new(immediate_replies: bool, follow_commands: bool) -> Arc<Self> {
        Arc::new(Self {
            stop_cell: OnceLock::new(),
            arm_after_j1_threshold_bits: AtomicU64::new(0),
            advancing_j1_seen: AtomicU64::new(0),
            triggered_gen: AtomicU64::new(0),
            trigger_tx_seq: AtomicU64::new(u64::MAX),
            enobufs_target_id: AtomicU32::new(0),
            enobufs_remaining: AtomicUsize::new(0),
            enobufs_injected: AtomicU64::new(0),
            tx_seq: AtomicU64::new(0),
            tx_ring_dropped: AtomicU64::new(0),
            immediate_replies: AtomicBool::new(immediate_replies),
            follow_commands: AtomicBool::new(follow_commands),
        })
    }

    fn arm_mid_batch_stop_after_j1_progress(&self, threshold_rad: f32) {
        self.advancing_j1_seen.store(0, Ordering::Release);
        let encoded = (1u64 << 32) | u64::from(threshold_rad.to_bits());
        self.arm_after_j1_threshold_bits.store(encoded, Ordering::Release);
    }

    fn arm_enobufs(&self, target_id: u32, count: usize) {
        self.enobufs_target_id.store(target_id, Ordering::Release);
        self.enobufs_remaining.store(count, Ordering::Release);
    }
}

/// Lock-free, zero-allocation scripted Damiao CAN transport.
///
/// Holds no `Mutex` or `RwLock` anywhere: motors live in a fixed `[ScriptedMotor; 7]` array,
/// RX/pending queues are preallocated `VecDeque`s, and TX frames stream to the test thread
/// through an `rtrb` SPSC ring buffer.
struct LockFreeScriptedBus {
    motors: [ScriptedMotor; 7],
    rx: VecDeque<CanFrame>,
    pending: VecDeque<CanFrame>,
    tx_ring: rtrb::Producer<CanFrame>,
    clock: Option<MockClock>,
    ctrl: Arc<LockFreeBusControls>,
}

impl LockFreeScriptedBus {
    fn new(
        limits: &[DamiaoLimits],
        clock: Option<MockClock>,
        immediate_replies: bool,
        follow_commands: bool,
    ) -> (Self, rtrb::Consumer<CanFrame>, Arc<LockFreeBusControls>) {
        assert_eq!(limits.len(), 7, "LockFreeScriptedBus expects 7-DOF Damiao limits");
        let motors = std::array::from_fn(|i| ScriptedMotor::new(i as u16 + 1, limits[i]));
        let (prod, cons) = rtrb::RingBuffer::<CanFrame>::new(8192);
        let ctrl = LockFreeBusControls::new(immediate_replies, follow_commands);
        let mut bus = Self {
            motors,
            rx: VecDeque::with_capacity(256),
            pending: VecDeque::with_capacity(256),
            tx_ring: prod,
            clock,
            ctrl: Arc::clone(&ctrl),
        };
        bus.push_all_feedback_direct();
        (bus, cons, ctrl)
    }

    fn push_all_feedback_direct(&mut self) {
        for i in 0..7 {
            let f = self.motors[i].feedback_frame();
            if self.rx.len() < self.rx.capacity() {
                self.rx.push_back(f);
            }
        }
    }

    fn push_reply(&mut self, frame: CanFrame) {
        if self.ctrl.immediate_replies.load(Ordering::Acquire) {
            if self.rx.len() < self.rx.capacity() {
                self.rx.push_back(frame);
            }
        } else if self.pending.len() < self.pending.capacity() {
            self.pending.push_back(frame);
        }
    }

    fn handle_frame(&mut self, frame: &CanFrame) {
        if frame.id == 0x7FF {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            let (op, rid) = (frame.data[2], frame.data[3]);
            let Some(k) = self.motors.iter().position(|m| m.id == id) else {
                return;
            };
            let reply = match op {
                0xCC => Some(self.motors[k].feedback_frame()),
                damiao::REG_READ => self.motors[k]
                    .read_reg(rid)
                    .map(|val| self.motors[k].reg_reply(damiao::REG_READ, rid, val)),
                damiao::REG_WRITE => {
                    let val = [frame.data[4], frame.data[5], frame.data[6], frame.data[7]];
                    if rid == damiao::RID_CTRL_MODE {
                        self.motors[k].ctrl_mode = u32::from_le_bytes(val);
                    }
                    Some(self.motors[k].reg_reply(damiao::REG_WRITE, rid, val))
                }
                _ => None,
            };
            if let Some(r) = reply {
                self.push_reply(r);
            }
            return;
        }
        let cmd = frame.data;
        if let Some(k) = self.motors.iter().position(|m| u32::from(m.id) == frame.id) {
            if cmd == damiao::CMD_ENABLE {
                self.motors[k].enabled = true;
            } else if cmd == damiao::CMD_DISABLE {
                self.motors[k].enabled = false;
            }
            let fb = self.motors[k].feedback_frame();
            self.push_reply(fb);
            return;
        }
        if let Some(k) = self
            .motors
            .iter()
            .position(|m| u32::from(m.id) + 0x100 == frame.id)
        {
            if self.motors[k].enabled && self.ctrl.follow_commands.load(Ordering::Acquire) {
                self.motors[k].q = f32::from_le_bytes([cmd[0], cmd[1], cmd[2], cmd[3]]);
                self.motors[k].v = 0.0;
            }
            let fb = self.motors[k].feedback_frame();
            self.push_reply(fb);
            return;
        }
        if let Some(k) = self
            .motors
            .iter()
            .position(|m| u32::from(m.id) + 0x300 == frame.id)
        {
            if self.motors[k].enabled && self.ctrl.follow_commands.load(Ordering::Acquire) {
                self.motors[k].q = f32::from_le_bytes([cmd[0], cmd[1], cmd[2], cmd[3]]);
                self.motors[k].v = 0.0;
            }
            let fb = self.motors[k].feedback_frame();
            self.push_reply(fb);
        }
    }
}

impl CanTransport for LockFreeScriptedBus {
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        let target_id = self.ctrl.enobufs_target_id.load(Ordering::Acquire);
        if target_id != 0 && frame.id == target_id {
            let hit = self
                .ctrl
                .enobufs_remaining
                .fetch_update(Ordering::AcqRel, Ordering::Acquire, |n| n.checked_sub(1))
                .is_ok();
            if hit {
                self.ctrl.enobufs_injected.fetch_add(1, Ordering::AcqRel);
                return Err(TransportError::NoBufferSpace);
            }
        }
        self.handle_frame(frame);
        let seq = self.ctrl.tx_seq.fetch_add(1, Ordering::AcqRel);
        if self.tx_ring.push(*frame).is_err() {
            self.ctrl.tx_ring_dropped.fetch_add(1, Ordering::AcqRel);
        }

        let encoded = self.ctrl.arm_after_j1_threshold_bits.load(Ordering::Acquire);
        if encoded != 0 {
            if frame.id == 0x101 {
                let threshold = f32::from_bits(encoded as u32);
                if decode_pos_vel_position(frame) > threshold {
                    self.ctrl.advancing_j1_seen.fetch_add(1, Ordering::AcqRel);
                }
            } else if frame.id == 0x102
                && self.ctrl.advancing_j1_seen.load(Ordering::Acquire) >= 3
                && self
                    .ctrl
                    .arm_after_j1_threshold_bits
                    .compare_exchange(encoded, 0, Ordering::AcqRel, Ordering::Acquire)
                    .is_ok()
            {
                if let Some(stops) = self.ctrl.stop_cell.get() {
                    let gen = stops.request();
                    self.ctrl.trigger_tx_seq.store(seq, Ordering::Release);
                    self.ctrl.triggered_gen.store(gen, Ordering::Release);
                }
            }
        }
        Ok(())
    }

    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        if let Some(f) = self.rx.pop_front() {
            return Ok(f);
        }
        while let Some(f) = self.pending.pop_front() {
            if self.rx.len() < self.rx.capacity() {
                self.rx.push_back(f);
            }
        }
        Err(TransportError::Timeout)
    }

    fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
        let now = self
            .clock
            .as_ref()
            .map_or_else(Instant::now, DriverClock::now);
        self.receive().map(|f| (f, Some(now)))
    }

    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }

    fn flush_is_noop(&self) -> bool {
        true
    }

    fn clock(&self) -> Option<Arc<dyn DriverClock>> {
        self.clock
            .clone()
            .map(|c| Arc::new(c) as Arc<dyn DriverClock>)
    }

    fn supports_bounded_flush(&self) -> bool {
        true
    }

    fn supports_nonblocking_rt(&self) -> bool {
        true
    }
}

fn drain_tx_ring(cons: &mut rtrb::Consumer<CanFrame>, out: &mut Vec<CanFrame>) {
    while let Ok(f) = cons.pop() {
        out.push(f);
    }
}

#[test]
fn production_executor_driver_lock_positive_control_counts_and_catches_on_armd_control() {
    run_bounded_fixture(Duration::from_secs(15), "driver-lock-pos-ctrl", |deadline| {
        let rt = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(2)
            .enable_all()
            .build()
            .unwrap();
        rt.block_on(async move {
            let (bus, _rx_trace, _ctrl) = LockFreeScriptedBus::new(&dm_limits(), None, true, true);
            let mut drv = DamiaoArmDriver::new(bus, vec![1, 2, 3, 4, 5, 6, 7])
                .with_joint_limits(dm_limits())
                .unwrap();
            drv.read_joint_states().unwrap();
            drv.check_mapping_ranges();

            let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
            sup.set_lease_enabled(false).await;
            sup.register_ws_connection(1).await.unwrap();

            // Build a separate contended DriverLock wired to the supervisor's shared
            // `driver_lock_attempts` counter so touching the mutex from `armd-control` would block
            // unless DriverLock intercepts, counts, and panics BEFORE touching std::sync::Mutex.
            let (probe_bus, _, _) = LockFreeScriptedBus::new(&dm_limits(), None, true, true);
            let probe_drv = DamiaoArmDriver::new(probe_bus, vec![1, 2, 3, 4, 5, 6, 7])
                .with_joint_limits(dm_limits())
                .unwrap();
            let probe_lock = Arc::new(DriverLock::with_counter(
                Box::new(probe_drv),
                sup.driver_lock_attempts_counter(),
            ));
            let (held_tx, held_rx) = mpsc::sync_channel::<()>(1);
            let (rel_tx, rel_rx) = mpsc::sync_channel::<()>(1);
            let holder_lock = Arc::clone(&probe_lock);
            let holder = thread::Builder::new()
                .name("test-drv-lock-holder".into())
                .spawn(move || {
                    let _g = holder_lock.lock().unwrap();
                    let _ = held_tx.send(());
                    let _ = rel_rx.recv_timeout(Duration::from_secs(10));
                })
                .unwrap();
            held_rx
                .recv_timeout(deadline.saturating_duration_since(Instant::now()))
                .expect("holder acquired probe DriverLock");

            let probe_armed = Arc::new(AtomicBool::new(false));
            let probe_done = Arc::new(AtomicBool::new(false));
            let probe_record: Arc<Mutex<Option<(ThreadId, u64, u64, u64, String, String)>>> =
                Arc::new(Mutex::new(None));

            let armed_h = Arc::clone(&probe_armed);
            let done_h = Arc::clone(&probe_done);
            let rec_h = Arc::clone(&probe_record);
            let lock_h = Arc::clone(&probe_lock);
            let hooks = PersistTestHooks {
                before_publish: Some(Arc::new(move || {
                    if !armed_h.swap(false, Ordering::AcqRel) {
                        return;
                    }
                    assert!(on_rt_control_thread());
                    let c0 = lock_h.control_thread_attempts();
                    let p1 = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        drop(lock_h.lock());
                    }))
                    .expect_err("DriverLock::lock must panic on armd-control");
                    let c1 = lock_h.control_thread_attempts();
                    let p2 = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        drop(lock_h.try_lock());
                    }))
                    .expect_err("DriverLock::try_lock must panic on armd-control");
                    let c2 = lock_h.control_thread_attempts();
                    *rec_h.lock().unwrap() = Some((
                        thread::current().id(),
                        c0,
                        c1,
                        c2,
                        panic_payload_to_string(p1),
                        panic_payload_to_string(p2),
                    ));
                    done_h.store(true, Ordering::Release);
                })),
                ..Default::default()
            };

            let handle = sup
                .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks)
                .await
                .expect("start executor for DriverLock positive control");
            let exec_tid = handle.executor_thread_id();
            let writer_hold = DetachedLegacyWriterHold::acquire(Arc::clone(&sup), deadline);

            let seq0 = sup.hub().published_seq();
            wait_until_deadline(deadline, ">= 3 baseline releases before DriverLock probe", || {
                sup.hub().published_seq() >= seq0 + 3
            });
            assert_eq!(sup.driver_lock_attempts(), 0);
            assert_eq!(sup.inner().control_thread_attempts(), 0);

            probe_armed.store(true, Ordering::Release);
            wait_until_deadline(deadline, "DriverLock probe executed on armd-control", || {
                probe_done.load(Ordering::Acquire)
            });

            let seq1 = sup.hub().published_seq();
            wait_until_deadline(deadline, ">= 3 releases after DriverLock probe", || {
                sup.hub().published_seq() >= seq1 + 3
            });
            assert!(!handle.is_exited());

            let (tid, c0, c1, c2, msg1, msg2) = probe_record.lock().unwrap().clone().unwrap();
            assert_eq!(tid, exec_tid);
            assert_eq!((c0, c1, c2), (0, 1, 2));
            assert!(msg1.contains("RT control thread") && msg1.contains("DriverLock::lock"), "{msg1}");
            assert!(msg2.contains("RT control thread") && msg2.contains("DriverLock::try_lock"), "{msg2}");
            assert_eq!(sup.driver_lock_attempts(), 2);
            assert_eq!(sup.inner().control_thread_attempts(), 0);

            let _ = rel_tx.send(());
            holder.join().unwrap();
            writer_hold.release();
            handle.stop_and_join();
            assert_eq!(sup.driver_lock_attempts(), 2);
        });
    });

    // Public SupervisorInner::tick + fill_observation positive control under
    // ControlThreadGuard + ReleaseGuard: Legacy DriverSlot panics + counts; Exclusive succeeds
    // with 0 attempts and produces observable CAN TX frames.
    let make_inner = |exclusive: bool| {
        let clock = MockClock::new();
        let (bus, tx_cons, _ctrl) = LockFreeScriptedBus::new(&dm_limits(), Some(clock), true, true);
        let mut drv = DamiaoArmDriver::new(bus, vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(dm_limits())
            .unwrap();
        drv.read_joint_states().unwrap();
        drv.check_mapping_ranges();
        drv.set_torque(true).unwrap();
        drv.set_rt_mode(true);
        let counter = Arc::new(AtomicU64::new(0));
        let mut inner = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv));
        inner.lease_enabled = false;
        inner.is_torqued = true;
        inner.safety.set_torque(true).unwrap();
        if exclusive {
            inner.driver = Some(inner.driver.take().unwrap().into_exclusive().ok().unwrap());
        } else {
            inner
                .driver
                .as_mut()
                .unwrap()
                .attach_counter(Arc::clone(&counter));
        }
        (inner, tx_cons, counter)
    };

    // Legacy slot fails positive control on tick() and fill_observation().
    {
        let (mut inner, mut tx_cons, counter) = make_inner(false);
        let mut obs = inner.current_observation();
        let mut drained = Vec::new();
        drain_tx_ring(&mut tx_cons, &mut drained);
        drained.clear();
        let _cg = ControlThreadGuard::enter();
        let _rg = ReleaseGuard::enter();
        assert_panics_with(&["RT control thread", "DriverLock::"], || {
            let _ = inner.tick(0.02);
        });
        assert_eq!(counter.load(Ordering::Acquire), 1);
        assert_panics_with(&["RT control thread", "DriverLock::"], || {
            inner.fill_observation(1.0, &mut obs);
        });
        assert_eq!(counter.load(Ordering::Acquire), 2);
    }

    // Exclusive slot succeeds with 0 attempts and emits 7 joint TX frames.
    {
        let (mut inner, mut tx_cons, counter) = make_inner(true);
        let mut drained = Vec::new();
        drain_tx_ring(&mut tx_cons, &mut drained);
        drained.clear();
        let _cg = ControlThreadGuard::enter();
        let _rg = ReleaseGuard::enter();
        let mut obs = inner.tick(0.02);
        inner.fill_observation(1.0, &mut obs);
        drain_tx_ring(&mut tx_cons, &mut drained);
        assert_eq!(counter.load(Ordering::Acquire), 0);
        assert!(obs.driver_capabilities.enable_feedback);
        let target_ids: Vec<u32> = drained
            .iter()
            .filter(|f| is_damiao_target_frame(f.id))
            .map(|f| f.id)
            .collect();
        assert_eq!(target_ids, vec![0x101, 0x102, 0x103, 0x104, 0x105, 0x106, 0x107]);
    }
}

#[test]
fn production_executor_lock_free_transport_zero_locks_and_zero_allocs() {
    run_bounded_fixture(Duration::from_secs(20), "lock-free-zero-alloc", |deadline| {
        let rt = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(2)
            .enable_all()
            .build()
            .unwrap();
        rt.block_on(async move {
            let (bus, mut tx_cons, ctrl) =
                LockFreeScriptedBus::new(&dm_limits(), None, true, true);
            let mut drv = DamiaoArmDriver::new(bus, vec![1, 2, 3, 4, 5, 6, 7])
                .with_joint_limits(dm_limits())
                .unwrap();
            drv.read_joint_states().unwrap();
            drv.check_mapping_ranges();

            let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
            sup.set_lease_enabled(false).await;
            sup.register_ws_connection(1).await.unwrap();
            let init_epoch = {
                let mut inner = sup.inner().write().await;
                let stamp = inner.current_admission_stamp();
                let admitted = inner.check_admission(stamp).expect("admit torque on");
                SupervisorInner::enable_torque(admitted).expect("enable torque before handover");
                inner.tick(0.02);
                inner.safety.epoch_id()
            };

            let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(5));
            cfg.alloc_timing = true;
            let handle = sup
                .start_executor(cfg)
                .await
                .expect("start production executor with LockFreeScriptedBus");
            assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);
            ctrl.stop_cell
                .set(handle.stop_requests().clone())
                .expect("attach StopRequests to LockFreeScriptedBus");

            let writer_hold = DetachedLegacyWriterHold::acquire(Arc::clone(&sup), deadline);
            assert!(sup.inner().try_read().is_err());

            // 1. Baseline releases while legacy writer is held.
            let seq0 = sup.hub().published_seq();
            wait_until_deadline(deadline, ">= 3 baseline releases", || {
                sup.hub().published_seq() >= seq0 + 3
            });

            // 2. Partial-TX ENOBUFS backpressure carry-over on 0x104 while holding torque.
            let mut tx_trace = Vec::with_capacity(4096);
            drain_tx_ring(&mut tx_cons, &mut tx_trace);
            let trace_before_enobufs = tx_trace.len();
            ctrl.arm_enobufs(0x104, 1);
            wait_until_deadline(deadline, "ENOBUFS injected and carried-over batch drained", || {
                drain_tx_ring(&mut tx_cons, &mut tx_trace);
                ctrl.enobufs_injected.load(Ordering::Acquire) >= 1
                    && tx_trace[trace_before_enobufs..]
                        .iter()
                        .filter(|f| f.id == 0x107)
                        .count()
                        >= 2
            });
            let enobufs_slice: Vec<u32> = tx_trace[trace_before_enobufs..]
                .iter()
                .filter(|f| is_damiao_target_frame(f.id))
                .map(|f| f.id)
                .collect();
            assert!(
                enobufs_slice.windows(7).any(|w| w == [0x101, 0x102, 0x103, 0x104, 0x105, 0x106, 0x107]),
                "carried-over batch after ENOBUFS on 0x104 must complete 0x101..=0x107 in order: {enobufs_slice:?}"
            );

            // 3. Quintic jog + bus-triggered mid-batch urgent stop inhibiting 0x103..=0x107.
            //    Prepare off-thread directly from the published hub snapshot (zero maintenance RPC).
            let pub_snap = sup.hub().latest().expect("published hub snapshot");
            let start = pub_snap.obs.commanded_positions.clone();
            let jog_auth = Authority {
                lease_generation: 0,
                epoch: init_epoch,
                stop_generation: handle.stop_requests().applied(),
                config_revision: 0,
                parent_generation: 0,
            };
            let mut goal = start.clone();
            goal[0] += 0.50;
            ctrl.arm_mid_batch_stop_after_j1_progress(start[0] + 5e-4);

            let req_jog = submit_joint_quintic(&sup, &handle, 2, jog_auth, &start, &goal, 10.0);
            assert!(matches!(
                wait_broker_reply(&handle, req_jog, deadline, |r| matches!(r, Reply::Ack { .. }).then_some(r)),
                Reply::Ack { .. }
            ));

            wait_until_deadline(deadline, "mid-batch urgent stop applied on lock-free bus", || {
                let gen = ctrl.triggered_gen.load(Ordering::Acquire);
                gen > 0 && handle.stop_requests().applied() == gen
            });
            let stop_gen = ctrl.triggered_gen.load(Ordering::Acquire);
            let trigger_seq = ctrl.trigger_tx_seq.load(Ordering::Acquire) as usize;

            wait_until_deadline(deadline, "settled to Armed after mid-batch stop", || {
                sup.hub()
                    .latest()
                    .is_some_and(|s| s.obs.safety_state == SafetyState::Armed)
            });
            let post_seq = sup.hub().published_seq();
            let frozen_pos = sup.hub().latest().unwrap().obs.commanded_positions.clone();

            // Stale-generation command refused STOPPED.
            let req_stale = submit_joint_quintic(&sup, &handle, 3, jog_auth, &frozen_pos, &goal, 1.0);
            assert!(matches!(
                wait_broker_reply(&handle, req_stale, deadline, |r| matches!(r, Reply::Refused { .. }).then_some(r)),
                Reply::Refused { code: "STOPPED", .. }
            ));

            wait_until_deadline(deadline, ">= 3 post-settle releases", || {
                drain_tx_ring(&mut tx_cons, &mut tx_trace);
                sup.hub().published_seq() >= post_seq + 3
            });
            drain_tx_ring(&mut tx_cons, &mut tx_trace);

            // Mid-batch inhibition proof: right after 0x102 at `trigger_seq`, next target is 0x101.
            assert_eq!(tx_trace[trigger_seq].id, 0x102);
            let next_target = tx_trace[trigger_seq + 1..]
                .iter()
                .find(|f| is_damiao_target_frame(f.id))
                .expect("post-stop target batch");
            assert_eq!(
                next_target.id, 0x101,
                "mid-batch urgent stop must inhibit 0x103..=0x107, got 0x{:X}",
                next_target.id
            );
            assert!(
                frozen_pos[0] > start[0] + 5e-4 && frozen_pos[0] < goal[0] - 0.05,
                "expected mid-trajectory freeze: start={}, frozen={}, goal={}",
                start[0],
                frozen_pos[0],
                goal[0]
            );

            // End-to-end DRV-LOCK + B1 + B2 proof BEFORE releasing legacy writer:
            assert_eq!(handle.stop_requests().applied(), stop_gen);
            assert!(!handle.is_exited());
            assert_eq!(
                sup.inner().control_thread_attempts(),
                0,
                "zero SupervisorLock attempts on armd-control"
            );
            assert_eq!(
                sup.driver_lock_attempts(),
                0,
                "zero DriverLock attempts on armd-control"
            );
            assert_eq!(
                ctrl.tx_ring_dropped.load(Ordering::Acquire),
                0,
                "lock-free TX trace ring must not drop any frames"
            );
            assert_eq!(
                handle.stats().total_allocs(),
                AllocCounts::ZERO,
                "zero allocator ops across all releases and wakes on lock-free transport: {:?}",
                handle.stats().total_allocs()
            );

            writer_hold.release();
            handle.stop_and_join();
            assert_eq!(sup.inner().control_thread_attempts(), 0);
            assert_eq!(sup.driver_lock_attempts(), 0);
            assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);
        });
    });
}

#[test]
fn legacy_vs_executor_bus_trace_parity_mid_batch_stop_and_partial_tx_carryover() {
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum OwnerKind {
        Legacy,
        Executor,
    }

    fn run_mid_batch_stop(owner: OwnerKind) -> (Vec<(u32, [u8; 8])>, usize, f32) {
        let clock = MockClock::new();
        let (bus, mut tx_cons, ctrl) =
            LockFreeScriptedBus::new(&dm_limits(), Some(clock.clone()), true, true);
        let mut drv = DamiaoArmDriver::new(bus, vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(dm_limits())
            .unwrap();
        drv.read_joint_states().unwrap();
        drv.check_mapping_ranges();
        drv.set_torque(true).unwrap();

        let mut inner = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv));
        inner.lease_enabled = false;
        inner.is_torqued = true;
        inner.safety.set_torque(true).unwrap();
        let lock_counter = Arc::new(AtomicU64::new(0));
        match owner {
            OwnerKind::Legacy => {
                inner
                    .driver
                    .as_mut()
                    .unwrap()
                    .attach_counter(Arc::clone(&lock_counter));
            }
            OwnerKind::Executor => {
                inner.driver.as_mut().unwrap().write().unwrap().set_rt_mode(true);
                inner.rt_executor_owned = true;
                inner.safety.set_off_thread_reclaim(true);
                inner.driver = Some(inner.driver.take().unwrap().into_exclusive().ok().unwrap());
            }
        }

        let stops = StopRequests::new();
        inner.attach_stop_requests(stops.clone());
        let gate = inner.estop_gate();
        inner.driver.as_mut().unwrap().write().unwrap().set_estop_gate(gate);
        ctrl.stop_cell.set(stops.clone()).unwrap();

        // Warm up 1 tick at rest so measured/commanded are synced.
        inner.tick(0.02);
        let mut tx_trace = Vec::new();
        drain_tx_ring(&mut tx_cons, &mut tx_trace);
        tx_trace.clear();
        ctrl.tx_seq.store(0, Ordering::Release);

        let start = inner.commanded_positions.clone();
        let mut goal = start.clone();
        goal[0] += 0.40;
        let ep = inner.safety.epoch_id();
        {
            let s = inner.current_admission_stamp();
            let a = inner.check_admission(s).unwrap();
            SupervisorInner::jog_action(a, goal, Some(2.0), Some(ep), "joint_move").unwrap();
        }
        ctrl.arm_mid_batch_stop_after_j1_progress(start[0] + 1e-3);

        match owner {
            OwnerKind::Legacy => {
                for _ in 0..40 {
                    clock.advance(Duration::from_millis(20));
                    let _ = inner.tick(0.02);
                    drain_tx_ring(&mut tx_cons, &mut tx_trace);
                }
            }
            OwnerKind::Executor => {
                let (mut fx, _broker, _tw, _fx_stops) =
                    ReleaseFixture::new(inner, BootId(1), Capacities::default());
                fx.dt = 0.02;
                let _cg = ControlThreadGuard::enter();
                for _ in 0..40 {
                    clock.advance(Duration::from_millis(20));
                    let _rg = ReleaseGuard::enter();
                    let _ = fx.step();
                    drain_tx_ring(&mut tx_cons, &mut tx_trace);
                }
                assert_eq!(lock_counter.load(Ordering::Acquire), 0);
            }
        }

        assert_eq!(ctrl.tx_ring_dropped.load(Ordering::Acquire), 0);
        let trigger_seq = ctrl.trigger_tx_seq.load(Ordering::Acquire) as usize;
        assert!(trigger_seq < tx_trace.len(), "{owner:?}: mid-batch stop never triggered");
        assert_eq!(tx_trace[trigger_seq].id, 0x102);
        let target_frames: Vec<(u32, [u8; 8])> = tx_trace
            .iter()
            .filter(|f| is_damiao_target_frame(f.id))
            .map(|f| (f.id, f.data))
            .collect();
        let trigger_target_pos = tx_trace[..=trigger_seq]
            .iter()
            .filter(|f| is_damiao_target_frame(f.id))
            .count()
            - 1;
        let final_j1 = tx_trace
            .iter()
            .rev()
            .find(|f| f.id == 0x101)
            .map(decode_pos_vel_position)
            .unwrap();
        (target_frames, trigger_target_pos, final_j1)
    }

    let (leg_frames, leg_trig, leg_final) = run_mid_batch_stop(OwnerKind::Legacy);
    let (exe_frames, exe_trig, exe_final) = run_mid_batch_stop(OwnerKind::Executor);
    assert_eq!(leg_frames[leg_trig].0, 0x102);
    assert_eq!(leg_frames[leg_trig + 1].0, 0x101, "Legacy must inhibit 0x103..=0x107");
    assert_eq!(exe_frames[exe_trig].0, 0x102);
    assert_eq!(exe_frames[exe_trig + 1].0, 0x101, "Executor must inhibit 0x103..=0x107");
    assert_eq!(leg_trig, exe_trig, "Legacy vs Executor mid-batch trigger index must match");
    assert_eq!(
        leg_frames, exe_frames,
        "Legacy vs Executor mid-batch stop CAN (id, payload) trace must match exactly"
    );
    assert!(
        (leg_final - exe_final).abs() < 1e-4,
        "Legacy vs Executor held J1 position must match: legacy={leg_final}, exec={exe_final}"
    );

    fn run_partial_tx_carryover(owner: OwnerKind) -> Vec<(u32, u32, [u8; 8])> {
        let clock = MockClock::new();
        let (bus, mut tx_cons, ctrl) =
            LockFreeScriptedBus::new(&dm_limits(), Some(clock.clone()), true, true);
        let mut drv = DamiaoArmDriver::new(bus, vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(dm_limits())
            .unwrap();
        drv.set_tx_budget(TxBudget {
            max_frames: 7,
            max_duration: Duration::from_millis(10),
            frame_max_age: Duration::from_millis(45),
        });
        drv.read_joint_states().unwrap();
        drv.check_mapping_ranges();
        drv.set_torque(true).unwrap();

        let mut inner = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv));
        inner.lease_enabled = false;
        inner.is_torqued = true;
        inner.safety.set_torque(true).unwrap();
        let lock_counter = Arc::new(AtomicU64::new(0));
        match owner {
            OwnerKind::Legacy => {
                inner
                    .driver
                    .as_mut()
                    .unwrap()
                    .attach_counter(Arc::clone(&lock_counter));
            }
            OwnerKind::Executor => {
                inner.driver.as_mut().unwrap().write().unwrap().set_rt_mode(true);
                inner.rt_executor_owned = true;
                inner.safety.set_off_thread_reclaim(true);
                inner.driver = Some(inner.driver.take().unwrap().into_exclusive().ok().unwrap());
            }
        }
        inner.tick(0.02);
        let mut tx_trace = Vec::new();
        drain_tx_ring(&mut tx_cons, &mut tx_trace);
        tx_trace.clear();

        let start = inner.commanded_positions.clone();
        let mut goal = start.clone();
        goal[0] += 0.25;
        let ep = inner.safety.epoch_id();
        {
            let s = inner.current_admission_stamp();
            let a = inner.check_admission(s).unwrap();
            SupervisorInner::jog_action(a, goal, Some(1.0), Some(ep), "joint_move").unwrap();
        }
        // Inject 1 ENOBUFS on 0x104 on tick 1 -> tick 1 emits 0x101..=0x103 and defers 0x104..=0x107;
        // tick 2 drains carried-over 0x104..=0x107 and sends fresh joints within budget.
        ctrl.arm_enobufs(0x104, 1);

        let mut per_tick_frames = Vec::new();
        match owner {
            OwnerKind::Legacy => {
                for tick in 1u32..=4 {
                    clock.advance(Duration::from_millis(20));
                    let _ = inner.tick(0.02);
                    drain_tx_ring(&mut tx_cons, &mut tx_trace);
                    for f in tx_trace.drain(..) {
                        if is_damiao_target_frame(f.id) {
                            per_tick_frames.push((tick, f.id, f.data));
                        }
                    }
                }
            }
            OwnerKind::Executor => {
                let (mut fx, _broker, _tw, _fx_stops) =
                    ReleaseFixture::new(inner, BootId(2), Capacities::default());
                fx.dt = 0.02;
                let _cg = ControlThreadGuard::enter();
                for tick in 1u32..=4 {
                    clock.advance(Duration::from_millis(20));
                    let _rg = ReleaseGuard::enter();
                    let _ = fx.step();
                    drain_tx_ring(&mut tx_cons, &mut tx_trace);
                    for f in tx_trace.drain(..) {
                        if is_damiao_target_frame(f.id) {
                            per_tick_frames.push((tick, f.id, f.data));
                        }
                    }
                }
                assert_eq!(lock_counter.load(Ordering::Acquire), 0);
            }
        }
        assert_eq!(ctrl.tx_ring_dropped.load(Ordering::Acquire), 0);
        assert_eq!(ctrl.enobufs_injected.load(Ordering::Acquire), 1);
        per_tick_frames
    }

    let leg_carry = run_partial_tx_carryover(OwnerKind::Legacy);
    let exe_carry = run_partial_tx_carryover(OwnerKind::Executor);
    // Tick 1 emitted only 0x101, 0x102, 0x103; tick 2 led with carried-over 0x104..=0x107.
    let leg_t1: Vec<u32> = leg_carry.iter().filter(|&&(t, _, _)| t == 1).map(|&(_, id, _)| id).collect();
    let leg_t2: Vec<u32> = leg_carry.iter().filter(|&&(t, _, _)| t == 2).map(|&(_, id, _)| id).collect();
    assert_eq!(leg_t1, vec![0x101, 0x102, 0x103]);
    assert_eq!(&leg_t2[..4], &[0x104, 0x105, 0x106, 0x107]);
    assert_eq!(
        leg_carry, exe_carry,
        "Legacy vs Executor partial-TX ENOBUFS carry-over (tick, id, payload) trace must match exactly"
    );
}

struct OwnershipAuditDriver {
    dof: usize,
    panic_on_rt_mode: bool,
    panic_on_set_estop_gate: Arc<AtomicBool>,
    panic_on_enable_state: Arc<AtomicBool>,
    unsupported_reason: Option<&'static str>,
    block_pos_limits_gate: Option<(Arc<AtomicBool>, Arc<(Mutex<bool>, std::sync::Condvar)>)>,
    block_supports_rt_gate: Option<Arc<(Mutex<bool>, std::sync::Condvar)>>,
    block_set_estop_gate: Option<(Arc<AtomicBool>, Arc<(Mutex<bool>, std::sync::Condvar)>)>,
    block_rt_mode_gate: Option<Arc<(Mutex<bool>, std::sync::Condvar)>>,
    block_enable_state_gate: Option<(Arc<AtomicBool>, Arc<(Mutex<bool>, std::sync::Condvar)>)>,
    block_release_read_gate: Option<(Arc<AtomicBool>, Arc<(Mutex<bool>, std::sync::Condvar)>)>,
    block_disable_gate: Option<(Arc<AtomicBool>, Arc<(Mutex<bool>, std::sync::Condvar)>)>,
    block_drop_gate: Option<Arc<(Mutex<bool>, std::sync::Condvar)>>,
    release_reads: Arc<AtomicUsize>,
    enabled: Arc<AtomicBool>,
    disable_calls: Arc<AtomicUsize>,
    drop_thread: Arc<Mutex<Option<(ThreadId, bool)>>>,
}

impl OwnershipAuditDriver {
    fn new(
        enabled: Arc<AtomicBool>,
        disable_calls: Arc<AtomicUsize>,
        drop_thread: Arc<Mutex<Option<(ThreadId, bool)>>>,
    ) -> Self {
        Self {
            dof: 7,
            panic_on_rt_mode: false,
            panic_on_set_estop_gate: Arc::new(AtomicBool::new(false)),
            panic_on_enable_state: Arc::new(AtomicBool::new(false)),
            unsupported_reason: None,
            block_pos_limits_gate: None,
            block_supports_rt_gate: None,
            block_set_estop_gate: None,
            block_rt_mode_gate: None,
            block_enable_state_gate: None,
            block_release_read_gate: None,
            block_disable_gate: None,
            block_drop_gate: None,
            release_reads: Arc::new(AtomicUsize::new(0)),
            enabled,
            disable_calls,
            drop_thread,
        }
    }
}

impl ArmDriver for OwnershipAuditDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            enable_feedback: true,
            velocity_feedback: true,
            torque_feedback: true,
            temperature_feedback: true,
            ..DriverCapabilities::default()
        }
    }
    fn position_limits(&self, _joint: usize) -> Option<(f32, f32)> {
        if let Some((armed, gate)) = &self.block_pos_limits_gate {
            if armed.load(Ordering::Acquire) {
                let (lock, cvar) = &**gate;
                let mut released = lock.lock().unwrap();
                while !*released {
                    released = cvar.wait(released).unwrap();
                }
            }
        }
        None
    }
    fn set_rt_mode(&mut self, enabled: bool) {
        if enabled && self.panic_on_rt_mode {
            panic!("injected startup set_rt_mode panic");
        }
        if enabled {
            if let Some(gate) = &self.block_rt_mode_gate {
                let (lock, cvar) = &**gate;
                let mut released = lock.lock().unwrap();
                while !*released {
                    released = cvar.wait(released).unwrap();
                }
            }
        }
    }
    fn set_estop_gate(&mut self, _gate: armd::rt_core::urgent::EstopGate) {
        if self.panic_on_set_estop_gate.load(Ordering::Acquire) {
            panic!("injected startup set_estop_gate panic");
        }
        if let Some((armed, gate)) = &self.block_set_estop_gate {
            if armed.load(Ordering::Acquire) {
                let (lock, cvar) = &**gate;
                let mut released = lock.lock().unwrap();
                while !*released {
                    released = cvar.wait(released).unwrap();
                }
            }
        }
    }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        Ok(vec![
            JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 30.0 };
            self.dof
        ])
    }
    fn read_joint_states_budgeted(
        &mut self,
        out: &mut Vec<JointState>,
        _deadline: Instant,
        _max_rx_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        let count = self.release_reads.fetch_add(1, Ordering::SeqCst) + 1;
        if count >= 2 {
            if let Some((entered, gate)) = &self.block_release_read_gate {
                entered.store(true, Ordering::Release);
                let (lock, cvar) = &**gate;
                let mut released = lock.lock().unwrap();
                while !*released {
                    released = cvar.wait(released).unwrap();
                }
            }
        }
        out.clear();
        out.resize(
            self.dof,
            JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 30.0 },
        );
        Ok((self.dof as u16, false))
    }
    fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
        out.clear();
        out.resize(
            self.dof,
            JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 30.0 },
        );
        Some(())
    }
    fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        Ok(())
    }
    fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
        if !enable {
            self.disable_calls.fetch_add(1, Ordering::SeqCst);
        }
        self.enabled.store(enable, Ordering::SeqCst);
        Ok(())
    }
    fn begin_torque_budgeted(
        &mut self,
        enable: bool,
        _max_rx_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        if !enable {
            self.disable_calls.fetch_add(1, Ordering::SeqCst);
            if let Some((entered, gate)) = &self.block_disable_gate {
                entered.store(true, Ordering::Release);
                let (lock, cvar) = &**gate;
                let mut released = lock.lock().unwrap();
                while !*released {
                    released = cvar.wait(released).unwrap();
                }
            }
        }
        self.enabled.store(enable, Ordering::SeqCst);
        Ok(TorqueProgress::Done(TorqueOutcome::Confirmed))
    }
    fn supports_rt_executor(&self) -> Result<(), &'static str> {
        if let Some(gate) = &self.block_supports_rt_gate {
            let (lock, cvar) = &**gate;
            let mut released = lock.lock().unwrap();
            while !*released {
                released = cvar.wait(released).unwrap();
            }
        }
        match self.unsupported_reason {
            Some(r) => Err(r),
            None => Ok(()),
        }
    }
    fn is_feedback_valid(&self) -> bool {
        true
    }
    fn is_enabled(&self) -> bool {
        self.enabled.load(Ordering::SeqCst)
    }
    fn enable_state(&self) -> EnableState {
        if self.panic_on_enable_state.load(Ordering::Acquire) {
            panic!("injected enable_state probe panic");
        }
        if let Some((entered, gate)) = &self.block_enable_state_gate {
            entered.store(true, Ordering::Release);
            let (lock, cvar) = &**gate;
            let mut released = lock.lock().unwrap();
            while !*released {
                released = cvar.wait(released).unwrap();
            }
        }
        if self.enabled.load(Ordering::SeqCst) {
            EnableState::Enabled
        } else {
            EnableState::Disabled
        }
    }
    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        vec![Some(Duration::from_millis(1)); self.dof]
    }
    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
        out.clear();
        out.resize(self.dof, Some(Duration::from_millis(1)));
    }
    fn feedback_instants(&self) -> Vec<Option<Instant>> {
        vec![Some(Instant::now()); self.dof]
    }
    fn feedback_seqs(&self) -> Vec<u64> {
        vec![1; self.dof]
    }
}

impl Drop for OwnershipAuditDriver {
    fn drop(&mut self) {
        if let Some(gate) = &self.block_drop_gate {
            let (lock, cvar) = &**gate;
            let mut released = lock.lock().unwrap();
            while !*released {
                released = cvar.wait(released).unwrap();
            }
        }
        *self.drop_thread.lock().unwrap() =
            Some((thread::current().id(), on_rt_control_thread()));
    }
}

#[test]
fn single_writer_ownership_across_start_failures_and_shutdown_paths() {
    run_bounded_fixture(Duration::from_secs(20), "ownership-lifecycle", |deadline| {
        let rt = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(2)
            .enable_all()
            .build()
            .unwrap();
        rt.block_on(async move {
            let main_tid = thread::current().id();

            // 1. Recoverable spawn failure (`fail_control_spawn: true`): restores OWNER_LEGACY,
            //    driver returns to DriverSlot::Legacy with shared counter, executor routing is NOT
            //    leaked (`!is_rt_owner`), and legacy stop + tick work cleanly.
            {
                let drop_t = Arc::new(Mutex::new(None));
                let drv = OwnershipAuditDriver::new(
                    Arc::new(AtomicBool::new(false)),
                    Arc::new(AtomicUsize::new(0)),
                    Arc::clone(&drop_t),
                );
                let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
                sup.register_ws_connection(1).await.unwrap();
                let lease_gen = sup.acquire_lease("client-1").await.unwrap();
                let hooks = PersistTestHooks { fail_control_spawn: true, ..Default::default() };
                assert!(sup
                    .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks)
                    .await
                    .is_err());
                assert_eq!(sup.owner_mode(), OWNER_LEGACY);
                assert!(drop_t.lock().unwrap().is_none(), "driver must not drop on recoverable spawn rollback");
                {
                    let mut inner = sup.inner().write().await;
                    assert!(!inner.rt_executor_owned, "recoverable spawn rollback must not leak RT owner routing");
                    assert!(inner.driver.as_ref().is_some_and(DriverSlot::is_legacy));
                    let obs = inner.tick(0.02);
                    assert!(obs.driver_capabilities.enable_feedback);
                }
                let stop_before = sup.stop_generation();
                let stop_out = sup
                    .urgent_stop(armd::rt_core::urgent::UrgentStop {
                        connection: armd::model::HolderLabel::new("client-1"),
                        lease_generation: lease_gen,
                        epoch: Some(sup.estop_epoch()),
                        request_id: 1,
                        request_id_valid: true,
                        body_error: None,
                    })
                    .await
                    .expect("legacy urgent_stop must succeed after recoverable spawn rollback");
                assert!(matches!(stop_out, armd::rt_core::urgent::UrgentStopOutcome::Applied { .. }), "{stop_out:?}");
                assert!(sup.stop_generation() > stop_before, "legacy stop must advance stop_generation after rollback");
                assert_eq!(sup.driver_lock_attempts(), 0);
            }

            // 2. Recoverable init failure (`fail_init: Some(true)`): restores OWNER_LEGACY,
            //    driver returns to DriverSlot::Legacy with shared counter, executor routing is NOT
            //    leaked (`!rt_executor_owned`), and legacy stop + tick work cleanly.
            {
                let drop_t = Arc::new(Mutex::new(None));
                let drv = OwnershipAuditDriver::new(
                    Arc::new(AtomicBool::new(false)),
                    Arc::new(AtomicUsize::new(0)),
                    Arc::clone(&drop_t),
                );
                let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
                sup.register_ws_connection(1).await.unwrap();
                let lease_gen = sup.acquire_lease("client-1").await.unwrap();
                let hooks = PersistTestHooks { fail_init: Some(true), ..Default::default() };
                assert!(sup
                    .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks)
                    .await
                    .is_err());
                assert_eq!(sup.owner_mode(), OWNER_LEGACY);
                assert!(drop_t.lock().unwrap().is_none(), "driver must not drop on recoverable init rollback");
                {
                    let mut inner = sup.inner().write().await;
                    assert!(!inner.rt_executor_owned, "recoverable init rollback must not leak RT owner routing");
                    assert!(inner.driver.as_ref().is_some_and(DriverSlot::is_legacy));
                    let obs = inner.tick(0.02);
                    assert!(obs.driver_capabilities.enable_feedback);
                }
                let stop_before = sup.stop_generation();
                let stop_out = sup
                    .urgent_stop(armd::rt_core::urgent::UrgentStop {
                        connection: armd::model::HolderLabel::new("client-1"),
                        lease_generation: lease_gen,
                        epoch: Some(sup.estop_epoch()),
                        request_id: 1,
                        request_id_valid: true,
                        body_error: None,
                    })
                    .await
                    .expect("legacy urgent_stop must succeed after recoverable init rollback");
                assert!(matches!(stop_out, armd::rt_core::urgent::UrgentStopOutcome::Applied { .. }), "{stop_out:?}");
                assert!(sup.stop_generation() > stop_before, "legacy stop must advance stop_generation after rollback");
                assert_eq!(sup.driver_lock_attempts(), 0);
            }

            // 3. Poisoned legacy driver at handover: refused 503 UNSUPPORTED_RT_DRIVER,
            //    OWNER_LEGACY restored, poison state + driver preserved intact.
            {
                let drop_t = Arc::new(Mutex::new(None));
                let drv = OwnershipAuditDriver::new(
                    Arc::new(AtomicBool::new(false)),
                    Arc::new(AtomicUsize::new(0)),
                    Arc::clone(&drop_t),
                );
                let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
                {
                    let inner = sup.inner().write().await;
                    let lock = inner.driver.as_ref().unwrap().as_legacy_lock().unwrap();
                    let _ = thread::scope(|s| {
                        s.spawn(|| {
                            let _g = lock.lock().unwrap();
                            panic!("intentional poison before handover");
                        })
                        .join()
                    });
                }
                let err = sup
                    .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
                    .await
                    .err()
                    .expect("poisoned driver handover must fail closed");
                assert!(err.to_string().contains("poisoned"), "unexpected error: {err}");
                assert_eq!(sup.owner_mode(), OWNER_LEGACY);
                assert!(drop_t.lock().unwrap().is_none(), "poisoned driver must not drop");
                let inner = sup.inner().read().await;
                let lock = inner.driver.as_ref().unwrap().as_legacy_lock().unwrap();
                assert!(lock.is_poisoned(), "poison state must remain intact after refusal");
            }

            // 4a. Non-recoverable init failure (`fail_init: Some(false)`) when torqued:
            //     runs bounded disable on `armd-control`, drops driver on `armd-control`,
            //     transitions to OWNER_FAILED, detached shell has `driver.is_none()`.
            // 4b. Non-recoverable init failure when `is_torqued == false` and `disable_pending == None`
            //     but hardware driver reports `EnableState::Enabled` (partial/unconfirmed energization):
            //     still runs bounded disable fail-closed!
            for supervisor_torqued in [true, false] {
                let enabled = Arc::new(AtomicBool::new(true));
                let disables = Arc::new(AtomicUsize::new(0));
                let drop_t = Arc::new(Mutex::new(None));
                let drv = OwnershipAuditDriver::new(
                    Arc::clone(&enabled),
                    Arc::clone(&disables),
                    Arc::clone(&drop_t),
                );
                let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
                if supervisor_torqued {
                    let mut inner = sup.inner().write().await;
                    inner.is_torqued = true;
                    inner.safety.set_torque(true).unwrap();
                }
                let hooks = PersistTestHooks { fail_init: Some(false), ..Default::default() };
                assert!(sup
                    .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks)
                    .await
                    .is_err());
                assert_eq!(sup.owner_mode(), OWNER_FAILED);
                assert!(sup.inner().read().await.driver.is_none(), "detached shell must not retain driver");
                assert!(
                    disables.load(Ordering::SeqCst) >= 1,
                    "non-recoverable init fail with enabled driver must attempt disable (supervisor_torqued={supervisor_torqued})"
                );
                assert!(!enabled.load(Ordering::SeqCst));
                let (dtid, on_ctrl) = drop_t.lock().unwrap().expect("driver dropped on control thread");
                assert_ne!(dtid, main_tid);
                assert!(on_ctrl);
                assert_eq!(sup.driver_lock_attempts(), 0);
            }

            // 4c. Non-recoverable init failure (`fail_init: Some(false)`) with owner torque flags
            //     clear and a driver that PANICS inside `enable_state()`:
            //     probe panic is caught as unknown enable state, bounded disable still runs,
            //     startup returns Err, OWNER_FAILED, SupervisorLock released, Legacy never gets driver.
            {
                let enabled = Arc::new(AtomicBool::new(true));
                let disables = Arc::new(AtomicUsize::new(0));
                let drop_t = Arc::new(Mutex::new(None));
                let panic_probe = Arc::new(AtomicBool::new(false));
                let mut drv = OwnershipAuditDriver::new(
                    Arc::clone(&enabled),
                    Arc::clone(&disables),
                    Arc::clone(&drop_t),
                );
                drv.panic_on_enable_state = Arc::clone(&panic_probe);
                let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
                panic_probe.store(true, Ordering::Release);
                let hooks = PersistTestHooks {
                    fail_init: Some(false),
                    panic_disable_timeout: Some(Duration::from_millis(80)),
                    ..Default::default()
                };
                assert!(sup
                    .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks)
                    .await
                    .is_err());
                assert_eq!(sup.owner_mode(), OWNER_FAILED);
                assert!(sup.inner().try_write().is_ok(), "SupervisorLock must be released");
                assert!(sup.inner().read().await.driver.is_none(), "Legacy must never receive driver");
                assert!(disables.load(Ordering::SeqCst) >= 1, "panicking enable_state probe must trigger bounded disable");
                assert!(!enabled.load(Ordering::SeqCst));
                let (dtid, on_ctrl) = drop_t.lock().unwrap().expect("driver dropped on control thread");
                assert_ne!(dtid, main_tid);
                assert!(on_ctrl);
                assert_eq!(sup.driver_lock_attempts(), 0);
            }

            // 4d. Non-recoverable init failure (`fail_init: Some(false)`) with owner torque flags
            //     clear and a driver that STALLS inside `enable_state()` past `panic_disable_timeout`,
            //     AND (4e) a driver whose `Drop` stalls past `panic_disable_timeout`:
            //     startup returns Err within bound, OWNER_FAILED, SupervisorLock released, Legacy
            //     never receives driver, and stalled `armd-control` retains sole ownership until unblocked.
            for stall_in_drop in [false, true] {
                let enabled = Arc::new(AtomicBool::new(false));
                let disables = Arc::new(AtomicUsize::new(0));
                let drop_t = Arc::new(Mutex::new(None));
                let gate = Arc::new((Mutex::new(false), std::sync::Condvar::new()));
                let entered_probe = Arc::new(AtomicBool::new(false));
                let mut drv = OwnershipAuditDriver::new(
                    Arc::clone(&enabled),
                    Arc::clone(&disables),
                    Arc::clone(&drop_t),
                );
                if stall_in_drop {
                    drv.block_drop_gate = Some(Arc::clone(&gate));
                } else {
                    drv.block_enable_state_gate = Some((Arc::clone(&entered_probe), Arc::clone(&gate)));
                }
                let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
                let hooks = PersistTestHooks {
                    fail_init: Some(false),
                    panic_disable_timeout: Some(Duration::from_millis(40)),
                    ..Default::default()
                };
                let t0 = Instant::now();
                let res = sup
                    .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks)
                    .await;
                let elapsed = t0.elapsed();
                assert!(res.is_err(), "stalled probe/drop must return Err (stall_in_drop={stall_in_drop})");
                assert!(
                    elapsed < Duration::from_millis(500),
                    "stalled probe/drop must return within bound, took {elapsed:?} (stall_in_drop={stall_in_drop})"
                );
                assert_eq!(sup.owner_mode(), OWNER_FAILED);
                assert!(sup.inner().try_write().is_ok(), "SupervisorLock must be released");
                assert!(sup.inner().read().await.driver.is_none(), "Legacy must never receive driver");
                if !stall_in_drop {
                    assert!(
                        entered_probe.load(Ordering::Acquire),
                        "driver must have entered enable_state() probe"
                    );
                    assert!(
                        sup.inner().read().await.e_stop_latched.load(Ordering::SeqCst),
                        "stalled enable_state() probe observer must latch E-stop BEFORE publishing timeout"
                    );
                }
                assert!(
                    drop_t.lock().unwrap().is_none(),
                    "stalled control thread must keep sole ownership until unblocked (stall_in_drop={stall_in_drop})"
                );
                {
                    let (lock, cvar) = &*gate;
                    *lock.lock().unwrap() = true;
                    cvar.notify_all();
                }
                wait_until_deadline(deadline, "stalled probe/drop thread exited after unblock", || {
                    drop_t.lock().unwrap().is_some()
                });
                let (dtid, on_ctrl) = drop_t.lock().unwrap().unwrap();
                assert_ne!(dtid, main_tid);
                assert!(on_ctrl);
                assert_eq!(sup.driver_lock_attempts(), 0);
            }

            // 4f. Normal startup stall inside `set_estop_gate(...)` AND inside `set_rt_mode(true)`
            //     (no `fail_init`): readiness handshake times out within bound, latches E-stop,
            //     commits OWNER_FAILED, releases SupervisorLock, Legacy never receives driver;
            //     once unblocked, late worker NEVER enters release loop, runs bounded disable,
            //     and drops driver on `armd-control`.
            for stall_in_gate in [true, false] {
                let enabled = Arc::new(AtomicBool::new(true));
                let disables = Arc::new(AtomicUsize::new(0));
                let drop_t = Arc::new(Mutex::new(None));
                let gate = Arc::new((Mutex::new(false), std::sync::Condvar::new()));
                let armed = Arc::new(AtomicBool::new(false));
                let mut drv = OwnershipAuditDriver::new(
                    Arc::clone(&enabled),
                    Arc::clone(&disables),
                    Arc::clone(&drop_t),
                );
                let release_reads = Arc::clone(&drv.release_reads);
                if stall_in_gate {
                    drv.block_set_estop_gate = Some((Arc::clone(&armed), Arc::clone(&gate)));
                } else {
                    drv.block_rt_mode_gate = Some(Arc::clone(&gate));
                }
                let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
                if stall_in_gate {
                    armed.store(true, Ordering::Release);
                }
                {
                    let mut inner = sup.inner().write().await;
                    inner.is_torqued = true;
                    inner.safety.set_torque(true).unwrap();
                }
                let hooks = PersistTestHooks {
                    panic_disable_timeout: Some(Duration::from_millis(40)),
                    ..Default::default()
                };
                let t0 = Instant::now();
                let res = sup
                    .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks)
                    .await;
                let elapsed = t0.elapsed();
                assert!(res.is_err(), "normal startup stall must return Err (stall_in_gate={stall_in_gate})");
                assert!(
                    elapsed < Duration::from_millis(500),
                    "normal startup stall must return within bound, took {elapsed:?} (stall_in_gate={stall_in_gate})"
                );
                assert_eq!(sup.owner_mode(), OWNER_FAILED);
                assert!(
                    sup.inner().read().await.e_stop_latched.load(Ordering::SeqCst),
                    "normal startup timeout must latch E-stop"
                );
                assert!(sup.inner().try_write().is_ok(), "SupervisorLock must be released");
                assert!(sup.inner().read().await.driver.is_none());
                assert!(drop_t.lock().unwrap().is_none(), "stalled worker must retain sole ownership");
                {
                    let (lock, cvar) = &*gate;
                    *lock.lock().unwrap() = true;
                    cvar.notify_all();
                }
                wait_until_deadline(deadline, "late startup worker tore down after unblock", || {
                    drop_t.lock().unwrap().is_some()
                });
                assert_eq!(
                    release_reads.load(Ordering::SeqCst),
                    0,
                    "late startup worker must NEVER enter release loop after timeout"
                );
                assert!(
                    disables.load(Ordering::SeqCst) >= 1,
                    "late startup worker must run bounded disable before dropping driver"
                );
                let (dtid, on_ctrl) = drop_t.lock().unwrap().unwrap();
                assert_ne!(dtid, main_tid);
                assert!(on_ctrl);
                assert_eq!(sup.driver_lock_attempts(), 0);
            }

            // 4g. On-time preflight `supports_rt_executor()` explicit refusal:
            //     returns `503 UNSUPPORTED_RT_DRIVER`, NO E-stop latch, restores `OWNER_LEGACY`,
            //     and returns driver to `DriverSlot::Legacy` with shared counter.
            {
                let drop_t = Arc::new(Mutex::new(None));
                let mut drv = OwnershipAuditDriver::new(
                    Arc::new(AtomicBool::new(false)),
                    Arc::new(AtomicUsize::new(0)),
                    Arc::clone(&drop_t),
                );
                drv.unsupported_reason = Some("mock unsupported driver");
                let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
                let err = sup
                    .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
                    .await
                    .err()
                    .expect("unsupported driver must fail with 503 UNSUPPORTED_RT_DRIVER");
                assert!(
                    matches!(err, armd::ArmError::Rejected { status: 503, ref code, .. } if code == "UNSUPPORTED_RT_DRIVER"),
                    "expected 503 UNSUPPORTED_RT_DRIVER, got {err:?}"
                );
                assert_eq!(sup.owner_mode(), OWNER_LEGACY);
                assert!(!sup.inner().read().await.e_stop_latched.load(Ordering::SeqCst));
                assert!(drop_t.lock().unwrap().is_none());
                assert!(sup.inner().read().await.driver.as_ref().is_some_and(DriverSlot::is_legacy));
            }

            // 4h. Preflight stalls in BOTH `position_limits()` AND `supports_rt_executor()` (including
            //     a late `Err` returned AFTER caller timeout!): caller returns Err within bound,
            //     latches E-stop, commits OWNER_FAILED, releases SupervisorLock; late worker NEVER
            //     enters release loop and ALWAYS runs bounded disable before dropping driver.
            for stall_in_supports in [false, true] {
                let enabled = Arc::new(AtomicBool::new(true));
                let disables = Arc::new(AtomicUsize::new(0));
                let drop_t = Arc::new(Mutex::new(None));
                let gate = Arc::new((Mutex::new(false), std::sync::Condvar::new()));
                let mut drv = OwnershipAuditDriver::new(
                    Arc::clone(&enabled),
                    Arc::clone(&disables),
                    Arc::clone(&drop_t),
                );
                let release_reads = Arc::clone(&drv.release_reads);
                let pos_armed = Arc::new(AtomicBool::new(false));
                if stall_in_supports {
                    drv.block_supports_rt_gate = Some(Arc::clone(&gate));
                    // Return `Err` AFTER unblocking to prove a late recoverable refusal STILL runs
                    // bounded disable instead of dropping a torqued driver!
                    drv.unsupported_reason = Some("late unsupported after timeout");
                } else {
                    drv.block_pos_limits_gate = Some((Arc::clone(&pos_armed), Arc::clone(&gate)));
                }
                let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
                if !stall_in_supports {
                    // Arm AFTER `with_driver` so only handover preflight triggers the stall.
                    pos_armed.store(true, Ordering::Release);
                }
                {
                    let mut inner = sup.inner().write().await;
                    inner.is_torqued = true;
                    inner.safety.set_torque(true).unwrap();
                }
                let hooks = PersistTestHooks {
                    panic_disable_timeout: Some(Duration::from_millis(40)),
                    ..Default::default()
                };
                let t0 = Instant::now();
                let res = sup
                    .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks)
                    .await;
                let elapsed = t0.elapsed();
                assert!(res.is_err(), "stalled preflight must return Err (stall_in_supports={stall_in_supports})");
                assert!(
                    elapsed < Duration::from_millis(500),
                    "stalled preflight must return within bound, took {elapsed:?}"
                );
                assert_eq!(sup.owner_mode(), OWNER_FAILED);
                assert!(sup.inner().read().await.e_stop_latched.load(Ordering::SeqCst));
                assert!(sup.inner().try_write().is_ok());
                assert!(sup.inner().read().await.driver.is_none());
                assert!(drop_t.lock().unwrap().is_none());
                {
                    let (lock, cvar) = &*gate;
                    *lock.lock().unwrap() = true;
                    cvar.notify_all();
                }
                wait_until_deadline(deadline, "late preflight worker disabled and dropped driver", || {
                    drop_t.lock().unwrap().is_some()
                });
                assert_eq!(release_reads.load(Ordering::SeqCst), 0);
                assert!(
                    disables.load(Ordering::SeqCst) >= 1,
                    "late preflight worker must run bounded disable even on late Err (stall_in_supports={stall_in_supports})"
                );
                assert!(!enabled.load(Ordering::SeqCst));
                let (dtid, on_ctrl) = drop_t.lock().unwrap().unwrap();
                assert_ne!(dtid, main_tid);
                assert!(on_ctrl);
                assert_eq!(sup.driver_lock_attempts(), 0);
            }
            // 5b. Startup panic inside `set_estop_gate(...)` (on `armd-control`, never caller thread):
            //     caught cleanly, runs bounded panic-disable on `armd-control`, drops driver on
            //     `armd-control`, transitions to OWNER_FAILED (NEVER OWNER_LEGACY), 0 DriverLock attempts.
            for panic_in_gate in [false, true] {
                let enabled = Arc::new(AtomicBool::new(true));
                let disables = Arc::new(AtomicUsize::new(0));
                let drop_t = Arc::new(Mutex::new(None));
                let panic_gate_arm = Arc::new(AtomicBool::new(false));
                let mut drv = OwnershipAuditDriver::new(
                    Arc::clone(&enabled),
                    Arc::clone(&disables),
                    Arc::clone(&drop_t),
                );
                drv.panic_on_rt_mode = !panic_in_gate;
                drv.panic_on_set_estop_gate = Arc::clone(&panic_gate_arm);
                let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
                if panic_in_gate {
                    // Arm AFTER `with_driver` so only handover's `attach_stop_requests -> set_estop_gate`
                    // triggers the panic, proving it executes inside `armd-control`'s `catch_unwind`.
                    panic_gate_arm.store(true, Ordering::Release);
                }
                {
                    let mut inner = sup.inner().write().await;
                    inner.is_torqued = true;
                    inner.safety.set_torque(true).unwrap();
                }
                let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(5));
                cfg.panic_policy = PanicPolicy::StopExecutor;
                assert!(sup.start_executor(cfg).await.is_err());
                assert_eq!(sup.owner_mode(), OWNER_FAILED);
                assert!(sup.inner().read().await.driver.is_none());
                assert!(
                    disables.load(Ordering::SeqCst) >= 1,
                    "startup panic must run bounded panic-disable (panic_in_gate={panic_in_gate})"
                );
                assert!(!enabled.load(Ordering::SeqCst));
                let (dtid, on_ctrl) = drop_t
                    .lock()
                    .unwrap()
                    .expect("driver dropped on control thread after startup panic");
                assert_ne!(dtid, main_tid);
                assert!(on_ctrl);
                assert_eq!(sup.driver_lock_attempts(), 0);
            }

            // 5c. Startup disable hang (`fail_init: Some(false)` with `begin_torque_budgeted(false, 0)`
            //     blocking past `panic_disable_timeout`): `start_executor` must return `Err` within
            //     the bounded timeout, transition to OWNER_FAILED, release SupervisorLock, and leave
            //     sole driver ownership on the stalled control thread until it unblocks.
            {
                let enabled = Arc::new(AtomicBool::new(true));
                let disables = Arc::new(AtomicUsize::new(0));
                let drop_t = Arc::new(Mutex::new(None));
                let gate = Arc::new((Mutex::new(false), std::sync::Condvar::new()));
                let mut drv = OwnershipAuditDriver::new(
                    Arc::clone(&enabled),
                    Arc::clone(&disables),
                    Arc::clone(&drop_t),
                );
                let disable_entered = Arc::new(AtomicBool::new(false));
                drv.block_disable_gate = Some((Arc::clone(&disable_entered), Arc::clone(&gate)));
                let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
                {
                    let mut inner = sup.inner().write().await;
                    inner.is_torqued = true;
                    inner.safety.set_torque(true).unwrap();
                }
                let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(5));
                cfg.panic_policy = PanicPolicy::StopExecutor;
                let hooks = PersistTestHooks {
                    fail_init: Some(false),
                    panic_disable_timeout: Some(Duration::from_millis(40)),
                    ..Default::default()
                };
                let t0 = Instant::now();
                let res = sup.start_executor_with_hooks(cfg, hooks).await;
                let elapsed = t0.elapsed();
                assert!(res.is_err(), "stalled startup teardown must return Err");
                assert!(
                    elapsed < Duration::from_millis(500),
                    "stalled startup teardown must return within bounded timeout, took {elapsed:?}"
                );
                assert_eq!(sup.owner_mode(), OWNER_FAILED);
                assert!(
                    sup.inner().try_write().is_ok(),
                    "SupervisorLock must be released (not stuck in OWNER_TRANSFERRING)"
                );
                assert!(sup.inner().read().await.driver.is_none());
                assert!(
                    drop_t.lock().unwrap().is_none(),
                    "stalled control thread must retain sole ownership until unblocked"
                );
                // Release the stalled driver so the thread exits cleanly before test end.
                {
                    let (lock, cvar) = &*gate;
                    *lock.lock().unwrap() = true;
                    cvar.notify_all();
                }
                wait_until_deadline(deadline, "stalled control thread dropped driver after unblock", || {
                    drop_t.lock().unwrap().is_some()
                });
                let (dtid, on_ctrl) = drop_t.lock().unwrap().unwrap();
                assert_ne!(dtid, main_tid);
                assert!(on_ctrl);
                assert_eq!(sup.driver_lock_attempts(), 0);
            }

            // 6. Normal shutdown AND emergency teardown (`emergency_disable_and_join`):
            //    runs exclusively on `armd-control` with 0 DriverLock attempts and detached shell
            //    retains `driver: None`.
            for emergency in [false, true] {
                let enabled = Arc::new(AtomicBool::new(true));
                let disables = Arc::new(AtomicUsize::new(0));
                let drop_t = Arc::new(Mutex::new(None));
                let drv = OwnershipAuditDriver::new(
                    Arc::clone(&enabled),
                    Arc::clone(&disables),
                    Arc::clone(&drop_t),
                );
                let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
                sup.set_lease_enabled(false).await;
                {
                    let mut inner = sup.inner().write().await;
                    inner.is_torqued = true;
                    inner.safety.set_torque(true).unwrap();
                }
                let handle = sup
                    .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
                    .await
                    .expect("start executor for shutdown test");
                let exec_tid = handle.executor_thread_id();
                wait_until_deadline_async(deadline, ">= 3 releases before shutdown", || {
                    handle.stats().steps.load(Ordering::Relaxed) >= 3
                })
                .await;
                assert!(sup.inner().read().await.driver.is_none());
                if emergency {
                    let outcome = handle.emergency_disable_and_join();
                    assert_eq!(outcome, PanicDisableOutcome::Confirmed);
                    assert!(disables.load(Ordering::SeqCst) >= 1);
                    assert!(!enabled.load(Ordering::SeqCst));
                } else {
                    handle.stop_and_join();
                }
                let (dtid, on_ctrl) = drop_t.lock().unwrap().expect("driver dropped on armd-control");
                assert_eq!(dtid, exec_tid);
                assert!(on_ctrl);
                assert!(sup.inner().read().await.driver.is_none());
                assert_eq!(sup.driver_lock_attempts(), 0);
                assert_eq!(sup.inner().control_thread_attempts(), 0);
            }

            // 7a. Emergency teardown where disable succeeds (`Confirmed`), THEN driver destructor
            //     stalls: `emergency_disable_and_join()` must return bounded without entering an
            //     unbounded `join()`, and driver remains owned by `armd-control` until unblocked.
            {
                let enabled = Arc::new(AtomicBool::new(true));
                let disables = Arc::new(AtomicUsize::new(0));
                let drop_t = Arc::new(Mutex::new(None));
                let drop_gate = Arc::new((Mutex::new(false), std::sync::Condvar::new()));
                let mut drv = OwnershipAuditDriver::new(
                    Arc::clone(&enabled),
                    Arc::clone(&disables),
                    Arc::clone(&drop_t),
                );
                drv.block_drop_gate = Some(Arc::clone(&drop_gate));
                let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
                sup.set_lease_enabled(false).await;
                {
                    let mut inner = sup.inner().write().await;
                    inner.is_torqued = true;
                    inner.safety.set_torque(true).unwrap();
                }
                let hooks = PersistTestHooks {
                    panic_disable_timeout: Some(Duration::from_millis(40)),
                    ..Default::default()
                };
                let handle = sup
                    .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks)
                    .await
                    .expect("start executor for emergency blocked-drop test");
                let exec_tid = handle.executor_thread_id();
                wait_until_deadline_async(deadline, ">= 2 releases before emergency blocked drop", || {
                    handle.stats().steps.load(Ordering::Relaxed) >= 2
                })
                .await;
                let t0 = Instant::now();
                let outcome = handle.emergency_disable_and_join();
                let elapsed = t0.elapsed();
                assert_eq!(outcome, PanicDisableOutcome::Confirmed);
                assert!(
                    elapsed < Duration::from_millis(500),
                    "emergency teardown must stay bounded when driver Drop stalls, took {elapsed:?}"
                );
                assert!(disables.load(Ordering::SeqCst) >= 1);
                assert!(!enabled.load(Ordering::SeqCst));
                assert!(
                    drop_t.lock().unwrap().is_none(),
                    "stalled destructor must keep driver owned on armd-control until unblocked"
                );
                {
                    let (lock, cvar) = &*drop_gate;
                    *lock.lock().unwrap() = true;
                    cvar.notify_all();
                }
                wait_until_deadline(deadline, "blocked emergency Drop finished after unblock", || {
                    drop_t.lock().unwrap().is_some()
                });
                let (dtid, on_ctrl) = drop_t.lock().unwrap().unwrap();
                assert_eq!(dtid, exec_tid);
                assert!(on_ctrl);
                assert_eq!(sup.driver_lock_attempts(), 0);
            }

            // 7b. Ordinary teardown (`stop_and_join`) racing a release panic whose panic-disable
            //     stalls: `stop_and_join` is invoked BEFORE the timeout observer fires and must
            //     still wake on timeout and detach instead of blocking in an unbounded `join()`.
            {
                let enabled = Arc::new(AtomicBool::new(true));
                let disables = Arc::new(AtomicUsize::new(0));
                let drop_t = Arc::new(Mutex::new(None));
                let disable_entered = Arc::new(AtomicBool::new(false));
                let disable_gate = Arc::new((Mutex::new(false), std::sync::Condvar::new()));
                let mut drv = OwnershipAuditDriver::new(
                    Arc::clone(&enabled),
                    Arc::clone(&disables),
                    Arc::clone(&drop_t),
                );
                drv.block_disable_gate = Some((Arc::clone(&disable_entered), Arc::clone(&disable_gate)));
                let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
                sup.set_lease_enabled(false).await;
                {
                    let mut inner = sup.inner().write().await;
                    inner.is_torqued = true;
                    inner.safety.set_torque(true).unwrap();
                }
                let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(5));
                cfg.panic_policy = PanicPolicy::StopExecutor;
                let hooks = PersistTestHooks {
                    panic_on_release: Some(2),
                    panic_disable_timeout: Some(Duration::from_millis(80)),
                    ..Default::default()
                };
                let handle = sup
                    .start_executor_with_hooks(cfg, hooks)
                    .await
                    .expect("start executor for ordinary teardown racing release panic");
                let exec_tid = handle.executor_thread_id();
                wait_until_deadline(deadline, "release panic entered stalled disable", || {
                    disable_entered.load(Ordering::Acquire)
                });
                // Call ordinary `stop_and_join()` immediately while disable is stalled and BEFORE
                // the 80 ms timeout observer has recorded `TimedOut`.
                let t0 = Instant::now();
                handle.stop_and_join();
                let elapsed = t0.elapsed();
                assert!(
                    elapsed < Duration::from_millis(500),
                    "ordinary stop_and_join racing stalled release panic must detach cleanly, took {elapsed:?}"
                );
                assert!(
                    drop_t.lock().unwrap().is_none(),
                    "stalled panic-disable thread must retain sole ownership until unblocked"
                );
                {
                    let (lock, cvar) = &*disable_gate;
                    *lock.lock().unwrap() = true;
                    cvar.notify_all();
                }
                wait_until_deadline(deadline, "stalled release-panic disable finished after unblock", || {
                    drop_t.lock().unwrap().is_some()
                });
                let (dtid, on_ctrl) = drop_t.lock().unwrap().unwrap();
                assert_eq!(dtid, exec_tid);
                assert!(on_ctrl);
                assert_eq!(sup.driver_lock_attempts(), 0);
            }

            // 7c. Forced ordinary teardown (`stop_and_join`) detaching a stalled, still-torqued
            //     release: must latch shared E-stop BEFORE detaching, and once unblocked the late
            //     worker must run bounded disable before destroying `core`.
            {
                let enabled = Arc::new(AtomicBool::new(true));
                let disables = Arc::new(AtomicUsize::new(0));
                let drop_t = Arc::new(Mutex::new(None));
                let read_entered = Arc::new(AtomicBool::new(false));
                let read_gate = Arc::new((Mutex::new(false), std::sync::Condvar::new()));
                let mut drv = OwnershipAuditDriver::new(
                    Arc::clone(&enabled),
                    Arc::clone(&disables),
                    Arc::clone(&drop_t),
                );
                drv.block_release_read_gate = Some((Arc::clone(&read_entered), Arc::clone(&read_gate)));
                let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
                sup.set_lease_enabled(false).await;
                {
                    let mut inner = sup.inner().write().await;
                    inner.is_torqued = true;
                    inner.safety.set_torque(true).unwrap();
                }
                let hooks = PersistTestHooks {
                    panic_disable_timeout: Some(Duration::from_millis(40)),
                    ..Default::default()
                };
                let handle = sup
                    .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks)
                    .await
                    .expect("start executor for forced teardown stalled-release test");
                let exec_tid = handle.executor_thread_id();
                wait_until_deadline(deadline, "release #2 entered stalled read", || {
                    read_entered.load(Ordering::Acquire)
                });
                let t0 = Instant::now();
                handle.stop_and_join();
                let elapsed = t0.elapsed();
                assert!(
                    elapsed < Duration::from_millis(500),
                    "forced stop_and_join must detach stalled release within bound, took {elapsed:?}"
                );
                assert!(
                    sup.inner().read().await.e_stop_latched.load(Ordering::SeqCst),
                    "detaching unconfirmed torqued owner must latch E-stop BEFORE unblocking"
                );
                assert!(drop_t.lock().unwrap().is_none());
                {
                    let (lock, cvar) = &*read_gate;
                    *lock.lock().unwrap() = true;
                    cvar.notify_all();
                }
                wait_until_deadline(deadline, "resumed worker ran disable and dropped driver", || {
                    drop_t.lock().unwrap().is_some()
                });
                assert!(
                    disables.load(Ordering::SeqCst) >= 1,
                    "resumed detached torqued worker must run bounded disable before drop"
                );
                assert!(!enabled.load(Ordering::SeqCst));
                let (dtid, on_ctrl) = drop_t.lock().unwrap().unwrap();
                assert_eq!(dtid, exec_tid);
                assert!(on_ctrl);
                assert_eq!(sup.driver_lock_attempts(), 0);
            }

            // 7d. Ordinary `stop_and_join()` when `armd-persist` is held inside parent-directory
            //     `fsync` past the teardown bound: must complete within bound and detach persistence
            //     instead of hanging indefinitely.
            {
                let dir = tempfile::tempdir().unwrap();
                let calib_path = dir.path().join("gripper_calib_ordinary_stall.json");
                let bus = FakeBus::new(&dm_limits());
                bus.with(|s| {
                    s.immediate_replies = true;
                    s.motors[6].q = 0.05;
                    s.motors[6].tau = 2.0;
                    s.push_all_feedback();
                });
                let mut drv = DamiaoArmDriver::new(bus, vec![1, 2, 3, 4, 5, 6, 7])
                    .with_joint_limits(dm_limits())
                    .unwrap();
                drv.read_joint_states().unwrap();
                drv.check_mapping_ranges();
                let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
                {
                    let mut inner = sup.inner().write().await;
                    inner.lease_enabled = false;
                    let s = inner.current_admission_stamp();
                    let a = inner.check_admission(s).unwrap();
                    SupervisorInner::enable_torque(a).unwrap();
                    inner.gripper_calib_path = Some(calib_path);
                    let ep = inner.safety.epoch_id();
                    let s2 = inner.current_admission_stamp();
                    let a2 = inner.check_admission(s2).unwrap();
                    let _ = SupervisorInner::start_gripper_zero_check(
                        a2,
                        &serde_json::json!({"threshold_rad": 0.10, "store_offset": true}),
                        ep,
                    );
                }
                let (in_sync_tx, in_sync_rx) = mpsc::sync_channel::<()>(1);
                let sync_gate = Arc::new((Mutex::new(false), std::sync::Condvar::new()));
                let in_sync_tx = Arc::new(Mutex::new(Some(in_sync_tx)));
                let sync_gate_h = Arc::clone(&sync_gate);
                let hooks = PersistTestHooks {
                    panic_disable_timeout: Some(Duration::from_millis(40)),
                    dir_sync_override: Some(Arc::new(move |_| {
                        if let Some(tx) = in_sync_tx.lock().unwrap().take() {
                            let _ = tx.send(());
                        }
                        let (lock, cvar) = &*sync_gate_h;
                        let mut released = lock.lock().unwrap();
                        while !*released {
                            released = cvar.wait(released).unwrap();
                        }
                        Ok(())
                    })),
                    ..Default::default()
                };
                let handle = sup
                    .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks)
                    .await
                    .expect("start executor for ordinary teardown stalled-fsync test");
                in_sync_rx
                    .recv_timeout(Duration::from_secs(3))
                    .expect("armd-persist must enter dir_sync_override");
                let t0 = Instant::now();
                handle.stop_and_join();
                let elapsed = t0.elapsed();
                assert!(
                    elapsed < Duration::from_millis(500),
                    "ordinary stop_and_join must stay bounded while dir_sync is stalled, took {elapsed:?}"
                );
                let (lock, cvar) = &*sync_gate;
                *lock.lock().unwrap() = true;
                cvar.notify_all();
            }
        });
    });
}

/// L3 lock-proof:
/// 1. Under `DriverSlot::Exclusive` (production `armd-control`) with `ControlThreadGuard` +
///    `ReleaseGuard`, stopped-hold TX and empty-mask stopped-hold never touch any mutex and
///    complete with zero lock attempts.
/// 2. Under `DriverSlot::Legacy` with a contended driver mutex on another OS thread, stopped-hold
///    TX uses non-blocking `try_lock` (skips the tick and increments
///    `TxDiagnostics::stopped_hold_lock_contended`), and an empty hold mask takes NO lock at all.
#[test]
fn l3_stopped_hold_never_blocks_on_driver_mutex_and_empty_mask_takes_no_lock() {
    use armd::diagnostics::tick::{
        reset_stopped_hold_lock_contended, stopped_hold_lock_contended_count, TickStats,
    };
    use armd::rt_core::ops::{CoreOps, ServiceBudget};
    use armd::safety::StopPolicy;
    use armd::transport::MockClock;
    use armd::SupervisorInner;

    run_bounded_fixture(Duration::from_secs(5), "l3_stopped_hold_lock_proof", |_deadline| {
        let build_estopped = |exclusive: bool| {
            let clock = MockClock::new();
            let bus = FakeBus::with_clock(&dm_limits(), clock.clone());
            bus.with(|s| {
                s.immediate_replies = true;
                for (i, m) in s.motors.iter_mut().enumerate() {
                    m.q = if i < 6 { 0.25 } else { 0.0 };
                }
                s.push_all_feedback();
            });
            let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
                .with_joint_limits(dm_limits())
                .unwrap();
            drv.read_joint_states().unwrap();
            drv.check_mapping_ranges();
            let mut inner = SupervisorInner::new(ArmConfig {
                lease_timeout_ms: 600_000,
                ..ArmConfig::rebot_dm()
            })
            .with_driver(Box::new(drv));
            if exclusive {
                inner.driver = Some(inner.driver.take().unwrap().into_exclusive().ok().unwrap());
                inner.rt_executor_owned = true;
            }
            inner.lease_enabled = false;
            inner.tick(0.02);
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).unwrap();
            SupervisorInner::enable_torque(admitted).unwrap();
            inner.apply_stop_policy(StopPolicy::EStopLatch);

            bus.with(|s| s.push_all_feedback());
            let now = Instant::now();
            let budget = ServiceBudget::for_release(now, now + Duration::from_millis(20));
            let _ = inner.poll_hardware(&budget);
            (inner, bus)
        };

        // Part A: Exclusive RT owner under ControlThreadGuard + ReleaseGuard — zero mutexes.
        {
            let (mut inner, _bus) = build_estopped(true);
            let _rt_thread = ControlThreadGuard::enter();
            let _release = ReleaseGuard::enter();
            assert!(inner.service_stopped_measured_hold_complete(0.02, false));
            assert_ne!(inner.owner_hold_mask(), 0);
            assert_eq!(inner.owner_pending_hold_mask(), 0);
            inner.invalidate_owner_driver_snapshot_for_test();
            assert!(!inner.service_stopped_measured_hold_complete(0.02, false));
            assert_eq!(inner.owner_hold_mask(), 0);
            assert!(inner.driver.as_ref().unwrap().is_exclusive());
        }

        // Part B: Legacy owner with contended mutex — non-blocking try_lock + empty-mask zero-lock.
        {
            let (mut inner, _bus) = build_estopped(false);
            assert!(inner.service_stopped_measured_hold_complete(0.02, false));
            assert_ne!(inner.owner_hold_mask(), 0);
            assert_eq!(inner.owner_pending_hold_mask(), 0);

            reset_stopped_hold_lock_contended();
            inner.with_contended_driver_for_test(|core| {
                // 1. Contended non-empty mask: try_lock returns immediately, skips send, counts contention.
                let t0 = Instant::now();
                let sent = core.service_stopped_measured_hold_complete(0.02, false);
                assert!(!sent, "L3: contended stopped-hold must skip send");
                assert!(
                    t0.elapsed() < Duration::from_millis(20),
                    "L3: contended stopped-hold must never block"
                );
                assert_eq!(stopped_hold_lock_contended_count(), 1);
                let snap = TickStats::default().snapshot();
                assert_eq!(snap.tx.stopped_hold_lock_contended, 1);

                // 2. Empty mask while driver mutex is STILL held: takes NO lock at all (counter stays 1).
                core.invalidate_owner_driver_snapshot_for_test();
                let t1 = Instant::now();
                let sent_empty = core.service_stopped_measured_hold_complete(0.02, false);
                assert!(!sent_empty);
                assert_eq!(core.owner_hold_mask(), 0);
                assert!(t1.elapsed() < Duration::from_millis(20));
                assert_eq!(
                    stopped_hold_lock_contended_count(),
                    1,
                    "L3: empty mask must NOT attempt try_lock"
                );
            });
        }
    });
}

/// M4 lock-proof:
/// 1. Under `DriverSlot::Exclusive` (production `armd-control`) with `ControlThreadGuard` +
///    `ReleaseGuard`, RT disable dispatch and disable polling never touch any mutex (`0` attempts).
/// 2. Under `DriverSlot::Legacy` with a contended driver mutex on another OS thread, RT disable
///    dispatch and disable polling use non-blocking `try_lock`, count skips in
///    `TxDiagnostics::disable_lock_contended`, and escalate at N=25 skips.
#[test]
fn r11_m4_rt_disable_dispatch_and_polling_never_block_on_driver_mutex() {
    use armd::diagnostics::tick::{
        disable_contention_escalations_count, disable_lock_contended_count,
        reset_disable_contention_escalations, reset_disable_lock_contended, TickStats,
    };
    use armd::rt_core::broker::{BootId, CommandKind, Descriptor, Stamp};
    use armd::rt_core::ids::ArmId;
    use armd::rt_core::ops::{CommitCtx, CoreOps, ServiceBudget, TorqueProgress};
    use armd::safety::StopPolicy;
    use armd::transport::MockClock;
    use armd::SupervisorInner;

    run_bounded_fixture(Duration::from_secs(5), "r11_m4_disable_lock_proof", |_deadline| {
        let build_ready_at_rest = |exclusive: bool| {
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
            let mut inner = SupervisorInner::new(ArmConfig {
                lease_timeout_ms: 600_000,
                ..ArmConfig::rebot_dm()
            })
            .with_driver(Box::new(drv));
            if exclusive {
                inner.driver = Some(inner.driver.take().unwrap().into_exclusive().ok().unwrap());
            }
            inner.rt_executor_owned = true;
            inner.lease_enabled = false;
            inner.register_live_connection(1).unwrap();
            inner.tick(0.02);
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).unwrap();
            SupervisorInner::enable_torque(admitted).unwrap();
            inner.apply_stop_policy(StopPolicy::EStopLatch);

            // Clear 20 ms settle and collect K=3 rest samples.
            clock.advance(Duration::from_millis(25));
            for step in 0..3 {
                if step > 0 {
                    clock.advance(Duration::from_millis(10));
                }
                bus.with(|s| s.push_all_feedback());
                inner.tick(0.02);
            }
            (inner, clock)
        };

        // Part A: Exclusive RT owner under ControlThreadGuard + ReleaseGuard — zero mutexes.
        {
            let (mut inner, clock) = build_ready_at_rest(true);
            let _rt_thread = ControlThreadGuard::enter();
            let _release = ReleaseGuard::enter();
            let now = clock.now();
            let cctx = CommitCtx {
                arm: ArmId(1),
                boot: BootId(1),
                now,
                stop_generation: 0,
            };
            let desc = Descriptor {
                stamp: Stamp {
                    arm: ArmId(1),
                    boot: BootId(1),
                    request: RequestId(1),
                    client: ClientId(1),
                    client_seq: 1,
                    authority: Authority {
                        lease_generation: 0,
                        epoch: inner.safety.epoch_id(),
                        stop_generation: 0,
                        config_revision: 0,
                        parent_generation: 0,
                    },
                    deadline: now + Duration::from_secs(5),
                },
                kind: CommandKind::Transition(TransitionOp::TorqueOff),
            };
            assert!(inner.commit_transition(&desc, TransitionOp::TorqueOff, &cctx).is_ok());
            assert!(inner.torque_transition_pending());
            let budget = ServiceBudget::for_release(clock.now(), clock.now() + Duration::from_millis(20));
            let poll = inner.poll_hardware(&budget);
            assert!(matches!(poll.torque, Some(Ok(TorqueProgress::Pending { .. } | TorqueProgress::Done))));
            assert!(inner.driver.as_ref().unwrap().is_exclusive());
        }

        // Part B: Legacy owner with contended mutex — non-blocking try_lock + N=25 escalation.
        {
            let (mut inner, clock) = build_ready_at_rest(false);

            // 1. Contended RT disable DISPATCH: never blocks, returns DRIVER_BUSY, counts contention.
            reset_disable_lock_contended();
            reset_disable_contention_escalations();
            inner.with_contended_driver_for_test(|core| {
                let now = clock.now();
                let cctx = CommitCtx {
                    arm: ArmId(1),
                    boot: BootId(1),
                    now,
                    stop_generation: 0,
                };
                let desc = Descriptor {
                    stamp: Stamp {
                        arm: ArmId(1),
                        boot: BootId(1),
                        request: RequestId(1),
                        client: ClientId(1),
                        client_seq: 1,
                        authority: Authority {
                            lease_generation: 0,
                            epoch: core.safety.epoch_id(),
                            stop_generation: 0,
                            config_revision: 0,
                            parent_generation: 0,
                        },
                        deadline: now + Duration::from_secs(5),
                    },
                    kind: CommandKind::Transition(TransitionOp::TorqueOff),
                };
                let t0 = Instant::now();
                let res = core.commit_transition(&desc, TransitionOp::TorqueOff, &cctx);
                assert_eq!(res, Err("DRIVER_BUSY"));
                assert!(
                    t0.elapsed() < Duration::from_millis(20),
                    "M4: contended RT disable dispatch must never block"
                );
                assert_eq!(disable_lock_contended_count(), 1);
                assert_eq!(TickStats::default().snapshot().tx.disable_lock_contended, 1);
            });

            // 2. Uncontended dispatch starts Pending disable repeats; then contended RT disable POLLING
            //    never blocks and escalates at N=25 consecutive skips.
            let now = clock.now();
            let cctx = CommitCtx {
                arm: ArmId(1),
                boot: BootId(1),
                now,
                stop_generation: 0,
            };
            let desc = Descriptor {
                stamp: Stamp {
                    arm: ArmId(1),
                    boot: BootId(1),
                    request: RequestId(2),
                    client: ClientId(1),
                    client_seq: 2,
                    authority: Authority {
                        lease_generation: 0,
                        epoch: inner.safety.epoch_id(),
                        stop_generation: 0,
                        config_revision: 0,
                        parent_generation: 0,
                    },
                    deadline: now + Duration::from_secs(5),
                },
                kind: CommandKind::Transition(TransitionOp::TorqueOff),
            };
            assert!(inner.commit_transition(&desc, TransitionOp::TorqueOff, &cctx).is_ok());
            assert!(inner.torque_transition_pending());

            reset_disable_lock_contended();
            reset_disable_contention_escalations();
            inner.with_contended_driver_for_test(|core| {
                let budget = ServiceBudget::for_release(clock.now(), clock.now() + Duration::from_millis(20));
                for skip in 1..=armd::STOPPED_HOLD_CONTENTION_ESCALATE_SKIPS {
                    let t0 = Instant::now();
                    let poll = core.poll_hardware(&budget);
                    assert!(
                        t0.elapsed() < Duration::from_millis(20),
                        "M4: contended RT disable poll #{skip} must never block"
                    );
                    assert!(matches!(poll.torque, Some(Ok(TorqueProgress::Pending { .. }))));
                    assert_eq!(disable_lock_contended_count(), u64::from(skip));
                }
                assert_eq!(disable_contention_escalations_count(), 1);
                let snap = TickStats::default().snapshot();
                assert_eq!(snap.tx.disable_lock_contended, u64::from(armd::STOPPED_HOLD_CONTENTION_ESCALATE_SKIPS));
                assert_eq!(snap.tx.disable_contention_escalations, 1);
                assert!(core.safety.has_latched_fault_code(armd::FaultCode::HardwareFault));
            });
        }
    });
}

/// Shadow PR1 lock proof: with `torque_shadow_enabled = true` on the production RT executor
/// (`DamiaoArmDriver<FakeBus>`), while the detached legacy `SupervisorLock` writer is held on
/// another OS thread AND an off-thread reader continuously polls `torque_shadow_diagnostics()`,
/// `armd-control` performs strictly ZERO `SupervisorLock` attempts, advances publications, and
/// publishes live `owner == Executor` shadow diagnostics.
#[test]
fn rt_executor_torque_shadow_on_zero_locks_and_diagnostics_readable() {
    run_bounded_fixture(Duration::from_secs(10), "torque_shadow_zero_locks", |deadline| {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        let mut cfg = ArmConfig::rebot_dm();
        cfg.torque_shadow_enabled = true;
        let limits = dm_limits();
        let bus = FakeBus::new(&limits);
        let mut drv = DamiaoArmDriver::new(bus.clone(), (1..=7).collect())
            .with_joint_limits(limits)
            .unwrap();
        drv.check_mapping_ranges();
        let sup = Arc::new(ArmSupervisor::new(cfg));
        rt.block_on(async {
            sup.set_driver(Box::new(drv)).await;
            sup.start_dispatcher().unwrap();
        });
        let handle = rt
            .block_on(sup.start_executor_with_hooks(
                ExecutorConfig::for_tests(Duration::from_millis(10)),
                PersistTestHooks::default(),
            ))
            .unwrap();
        assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);

        // Hold the detached legacy SupervisorLock writer on a separate OS thread throughout!
        let hold_stop = Arc::new(AtomicBool::new(false));
        let hold_ready = Arc::new(AtomicBool::new(false));
        let sup_hold = Arc::clone(&sup);
        let hs = Arc::clone(&hold_stop);
        let hr = Arc::clone(&hold_ready);
        let holder = thread::spawn(move || {
            let _w = sup_hold.inner().blocking_write();
            hr.store(true, Ordering::Release);
            while !hs.load(Ordering::Acquire) {
                thread::sleep(Duration::from_millis(2));
            }
        });
        wait_until_deadline(deadline, "detached writer held", || {
            hold_ready.load(Ordering::Acquire)
        });

        let start_seq = sup.hub().latest().map_or(0, |s| s.meta.seq);
        let mut saw_shadow_events = false;
        wait_until_deadline(deadline, "RT advances with shadow ON and writer held", || {
            let d = sup.torque_shadow_diagnostics();
            if d.enabled && d.owner == armd::torque::shadow::ShadowOwner::Executor && d.events_processed > 0 {
                saw_shadow_events = true;
            }
            let seq = sup.hub().latest().map_or(0, |s| s.meta.seq);
            saw_shadow_events && seq >= start_seq + 5
        });

        assert_eq!(
            sup.inner().control_thread_attempts(),
            0,
            "RT control thread must perform strictly 0 SupervisorLock attempts with torque shadow ON"
        );
        let diag = sup.torque_shadow_diagnostics();
        assert!(diag.enabled);
        assert_eq!(diag.owner, armd::torque::shadow::ShadowOwner::Executor);
        assert!(diag.events_processed > 0);
        assert_eq!(diag.taxonomy.invariant_violation, 0);

        hold_stop.store(true, Ordering::Release);
        holder.join().unwrap();
        handle.stop_and_join();
        // Verify off-thread reclamation of `ShadowRuntimeInner` allocations retired on `armd-control` exit (M19).
        sup.torque_shadow_hub().drain_now();
        assert!(
            sup.torque_shadow_hub().test_retired_reclaimed_count() >= 1,
            "ShadowRuntimeInner must be retired lock-free on armd-control exit and reclaimed off-thread"
        );
    });
}
