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
use std::future::Future;
use std::pin::Pin;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{mpsc, Arc, Mutex, OnceLock};
use std::task::{Context, Poll, Waker};
use std::thread::{self, ThreadId};
use std::time::{Duration, Instant};

use armd::driver::{
    ArmDriver, ArmDriverError, DamiaoArmDriver, DriverCapabilities, EnableState, JointTarget,
};
use armd::model::JointState;
use armd::protocol::damiao::DamiaoLimits;
use armd::rt_core::broker::{
    Authority, ClientId, Outcome, Reply, RequestId, Submit, TransitionOp,
};
use armd::rt_core::executor::{
    in_rt_release, on_rt_control_thread, validate_startup_owner, ControlThreadGuard,
    ExecutorConfig, PersistTestHooks, ReleaseGuard, RtExecutorHandle, RtOwner,
};
use armd::rt_core::ops::{PlanSlot, Primitive};
use armd::rt_core::urgent::StopRequests;
use armd::supervisor::OWNER_EXECUTOR;
use armd::supervisor_lock::{HoldKind, SupervisorLock};
use armd::transport::{CanFrame, CanTransport, DriverClock, TransportError};
use armd::{ArmConfig, ArmSupervisor, SafetyState};
use mit_support::FakeBus;

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
            let matches_req = match r {
                Reply::Ack { request, .. }
                | Reply::Refused { request, .. }
                | Reply::Terminal { request, .. } => request == req,
            };
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
    handle: &RtExecutorHandle,
    seq: u64,
    authority: Authority,
    start: &[f32],
    goal: &[f32],
    duration_s: f64,
) -> RequestId {
    let prim = Primitive::joint_quintic(start, goal, duration_s).unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut b = handle.broker().lock().unwrap();
    b.submit(
        ClientId(1),
        seq,
        authority,
        deadline,
        Submit::Install(&move |slot: &mut PlanSlot| slot.set(prim)),
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
            sup.set_torque(true).await.expect("enable torque on mock arm");

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
            let req_jog = submit_joint_quintic(&handle, 1, snap.authority, &start, &goal, 10.0);
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
            let req_stale = submit_joint_quintic(&handle, 2, snap.authority, &frozen_pos, &goal, 1.0);
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

    fn flush(&mut self) -> Result<(), TransportError> {
        self.inner.flush()
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
            let fake_bus = FakeBus::new(&dm_limits());
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
            let handle = sup
                .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
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

            let req_jog = submit_joint_quintic(&handle, 2, snap.authority, &start, &goal, 10.0);
            assert!(matches!(
                wait_broker_reply(&handle, req_jog, deadline, |r| matches!(r, Reply::Ack { .. }).then_some(r)),
                Reply::Ack { .. }
            ));

            wait_until_deadline(deadline, "bus-triggered mid-batch urgent stop applied", || {
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
            let req_stale = submit_joint_quintic(&handle, 3, snap.authority, &frozen_pos, &goal, 1.0);
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
