//! Stage 3 PR B2: whole-release and sub-release-wake allocation counter proof.
//!
//! Installs [`TimedAlloc`] as this binary's global allocator and runs every measured RT executor
//! fixture with `cfg.alloc_timing = true`, proving:
//! 1. Per-operation (`alloc`, `alloc_zeroed`, `realloc`, `dealloc`) counting is exact and
//!    independent of mid-release `alloc::take()` resets and the droppable `diag_tx` ring.
//! 2. Positive controls in the release tail (after `diag_tx.push` + `hub.wake()`) and in
//!    sub-release wakes detect exact counts for `alloc`, `alloc_zeroed`, `realloc`, and
//!    scope-exit local destruction (`dealloc`), with disjoint release/wake windows.
//! 3. With consumers stalled (observation hub `reader_guard`, broker poll paused, persist worker
//!    paused, saturated diagnostic/E-stop/persistence rings), normal commits, refused commits,
//!    pre-frame transition refusals, authorized urgent stops, terminal retirement, and saturated
//!    observation/E-stop/persistence/diagnostics queues perform strictly zero heap operations on
//!    `armd-control`, keep advancing publication, and reclaim payloads off-thread.
//! 4. Known allocations in in-flight B3/A8-owned files are captured as `#[ignore]` tests with
//!    exact `file:line` attribution and unblock conditions (affected families remain gated).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Condvar, Mutex};
use std::thread::{self, ThreadId};
use std::time::{Duration, Instant};

use armd::diagnostics::alloc::{self, AllocCounts, TimedAlloc};
use armd::driver::{
    ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget, TorqueOutcome,
    TorqueProgress,
};
use armd::model::JointState;
use armd::motion::{plan_cartesian, prepare_cartesian_path, resolve_goal, GoalSpec, Speed};
use armd::rt_core::broker::{
    Authority, ClientId, ImmediateOp, Outcome, Reply, RequestId, Submit, TransitionOp,
};
use armd::rt_core::executor::{
    ExecutorClock, ExecutorConfig, ParkedAlloc, PersistTestHooks, RtExecutorHandle, TailAllocProbe,
    DIAG_RING_CAPACITY, ESTOP_ACK_CAPACITY,
};
use armd::rt_core::ops::{CoreOps, PlanSlot};
use armd::supervisor::PendingZeroCheckPersist;
use armd::{ArmConfig, ArmSupervisor};

#[global_allocator]
static ALLOC: TimedAlloc = TimedAlloc;

// ============================================================================
// Deterministic Scripted Clock & Allocation-Free Wake Driver
// ============================================================================

struct ScriptedClock {
    base: Instant,
    state: Mutex<ClockState>,
    cv: Condvar,
}

struct ClockState {
    now_ns: u64,
    limit_ns: u64,
    waiting_for_ns: Option<u64>,
}

impl ScriptedClock {
    fn new() -> Arc<Self> {
        Arc::new(Self {
            base: Instant::now(),
            state: Mutex::new(ClockState {
                now_ns: 0,
                limit_ns: 0,
                waiting_for_ns: None,
            }),
            cv: Condvar::new(),
        })
    }

    fn run_steps(&self, period_ns: u64, steps: u64) {
        let mut s = self.state.lock().unwrap();
        let target = s.limit_ns + period_ns * steps;
        s.limit_ns = target;
        let expect_park = target + period_ns;
        self.cv.notify_all();
        let deadline = Instant::now() + Duration::from_secs(5);
        while s.waiting_for_ns != Some(expect_park) {
            let rem = deadline.saturating_duration_since(Instant::now());
            assert!(
                !rem.is_zero(),
                "timed out waiting for scripted clock park at {expect_park} ns (now={}, limit={}, wait={:?})",
                s.now_ns,
                s.limit_ns,
                s.waiting_for_ns
            );
            let (next, _) = self.cv.wait_timeout(s, rem).unwrap();
            s = next;
        }
    }
}

impl ExecutorClock for ScriptedClock {
    fn now(&self) -> Instant {
        self.base + Duration::from_nanos(self.state.lock().unwrap().now_ns)
    }

    fn now_ns(&self) -> u64 {
        self.state.lock().unwrap().now_ns
    }

    fn sleep_until_ns(&self, target_ns: u64, stop: &AtomicBool) -> bool {
        let mut s = self.state.lock().unwrap();
        if s.now_ns >= target_ns {
            self.cv.notify_all();
            return false;
        }
        while !stop.load(Ordering::Acquire) && s.now_ns < target_ns && s.limit_ns < target_ns {
            s.waiting_for_ns = Some(target_ns);
            self.cv.notify_all();
            s = self.cv.wait(s).unwrap();
        }
        s.waiting_for_ns = None;
        if !stop.load(Ordering::Acquire) && s.now_ns < target_ns {
            s.now_ns = target_ns;
        }
        self.cv.notify_all();
        true
    }

    fn sleep(&self, dur: Duration, stop: &AtomicBool) {
        if stop.load(Ordering::Acquire) {
            return;
        }
        let mut s = self.state.lock().unwrap();
        s.now_ns = s.now_ns.saturating_add(dur.as_nanos().min(u128::from(u64::MAX)) as u64);
        self.cv.notify_all();
    }

    fn wake(&self) {
        let _g = self.state.lock().unwrap();
        self.cv.notify_all();
    }
}

/// Allocation-free driver supporting real sub-release torque confirmation wakes and
/// optionally failing torque transitions on demand (for B3-blocked ignored tests).
struct WakeDriver {
    dof: usize,
    q: Vec<f32>,
    enabled: bool,
    rx_at: Vec<Option<Instant>>,
    seqs: Vec<u64>,
    pending_target: Option<(bool, Instant)>,
    confirm_delay: Duration,
    clock: Option<Arc<ScriptedClock>>,
    fail_next_enable: Arc<AtomicBool>,
}

impl WakeDriver {
    fn new(dof: usize, confirm_delay: Duration, clock: Option<Arc<ScriptedClock>>) -> (Self, Arc<AtomicBool>) {
        let fail_next_enable = Arc::new(AtomicBool::new(false));
        let now = clock.as_ref().map_or_else(Instant::now, |c| c.now());
        (
            Self {
                dof,
                q: vec![0.0; dof],
                enabled: false,
                rx_at: vec![Some(now); dof],
                seqs: vec![1; dof],
                pending_target: None,
                confirm_delay,
                clock,
                fail_next_enable: fail_next_enable.clone(),
            },
            fail_next_enable,
        )
    }

    fn now(&self) -> Instant {
        self.clock.as_ref().map_or_else(Instant::now, |c| c.now())
    }
}

impl ArmDriver for WakeDriver {
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

    fn supports_rt_executor(&self) -> Result<(), &'static str> {
        Ok(())
    }

    fn clock_now(&self) -> Instant {
        self.now()
    }

    fn enable_state(&self) -> EnableState {
        if self.enabled {
            EnableState::Enabled
        } else {
            EnableState::Disabled
        }
    }

    fn is_feedback_valid(&self) -> bool {
        true
    }

    fn has_driver_fresh_position_api(&self) -> bool {
        true
    }

    fn fresh_joint_positions_into(
        &self,
        joints: &[usize],
        max_age: Duration,
        out: &mut [f32],
    ) -> bool {
        if joints.is_empty() || out.len() < joints.len() {
            return false;
        }
        let now = self.now();
        for (slot, &i) in out.iter_mut().zip(joints.iter()) {
            let fresh = self
                .rx_at
                .get(i)
                .copied()
                .flatten()
                .is_some_and(|t| now.saturating_duration_since(t) <= max_age);
            if !fresh {
                return false;
            }
            let Some(&p) = self.q.get(i) else {
                return false;
            };
            if !p.is_finite() {
                return false;
            }
            *slot = p;
        }
        true
    }

    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        let now = self.now();
        self.rx_at.iter().map(|t| t.map(|t| now.saturating_duration_since(t))).collect()
    }

    fn feedback_instants(&self) -> Vec<Option<Instant>> {
        self.rx_at.clone()
    }

    fn feedback_seqs(&self) -> Vec<u64> {
        self.seqs.clone()
    }

    fn feedback_receive_instants(&self) -> Vec<Option<Instant>> {
        self.rx_at.clone()
    }

    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
        let now = self.now();
        out.clear();
        for t in &self.rx_at {
            out.push(t.map(|t| now.saturating_duration_since(t)));
        }
    }

    fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
        let now = self.now();
        self.rx_at.len() == dof
            && self.rx_at.iter().all(|t| t.is_some_and(|t| now.saturating_duration_since(t) <= max_age))
    }

    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
        let now = self.now();
        out.clear();
        for t in &self.rx_at {
            out.push(t.map(|t| now.saturating_duration_since(t).as_millis() as u64));
        }
    }

    fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        out.clear();
        out.extend_from_slice(&self.rx_at);
    }

    fn feedback_seqs_into(&self, out: &mut Vec<u64>) {
        out.clear();
        out.extend_from_slice(&self.seqs);
    }

    fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        out.clear();
        out.extend_from_slice(&self.rx_at);
    }

    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let mut out = Vec::with_capacity(self.dof);
        self.read_joint_states_into(&mut out)?;
        Ok(out)
    }

    fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
        let now = self.now();
        out.clear();
        for i in 0..self.dof {
            out.push(JointState {
                position: self.q[i],
                velocity: 0.0,
                torque: 0.0,
                temperature: 30.0,
            });
            self.rx_at[i] = Some(now);
            self.seqs[i] += 1;
        }
        Ok(())
    }

    fn read_joint_states_budgeted(
        &mut self,
        out: &mut Vec<JointState>,
        _deadline: Instant,
        _max_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        self.read_joint_states_into(out)?;
        Ok((self.dof as u16, false))
    }

    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        for (i, t) in targets.iter().enumerate().take(self.dof) {
            self.q[i] = t.position;
        }
        Ok(())
    }

    fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
        self.enabled = enable;
        Ok(())
    }

    fn begin_torque_budgeted(
        &mut self,
        enable: bool,
        _max_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        let wake = self.now() + self.confirm_delay;
        self.pending_target = Some((enable, wake));
        Ok(TorqueProgress::Pending { wake })
    }

    fn poll_torque_budgeted(
        &mut self,
        _deadline: Instant,
        _max_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        let Some((target, wake)) = self.pending_target else {
            return Ok(TorqueProgress::Done(TorqueOutcome::Confirmed));
        };
        if self.now() < wake {
            return Ok(TorqueProgress::Pending { wake });
        }
        self.pending_target = None;
        if target && self.fail_next_enable.swap(false, Ordering::AcqRel) {
            return Err(ArmDriverError::Timeout(1));
        }
        self.enabled = target;
        Ok(TorqueProgress::Done(TorqueOutcome::Confirmed))
    }

    fn abort_enable(&mut self) {
        if matches!(self.pending_target, Some((true, _))) {
            self.pending_target = None;
        }
    }

    fn torque_transition_pending(&self) -> bool {
        self.pending_target.is_some()
    }
}

fn alloc_cfg(period: Duration) -> ExecutorConfig {
    let mut cfg = ExecutorConfig::for_tests(period);
    cfg.alloc_timing = true;
    cfg
}

async fn wait_until(timeout: Duration, mut pred: impl FnMut() -> bool) {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        if pred() {
            return;
        }
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
    assert!(pred(), "timed out waiting for predicate after {timeout:?}");
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
    held_rx.recv_timeout(Duration::from_secs(2)).expect("acquire hub reader_guard");
    (rel_tx, jh)
}

fn wait_for_dispatcher_blocked_on_hub(sup: &Arc<ArmSupervisor>, timeout: Duration) {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        if sup.hub().latest_waiters() >= 1 {
            return;
        }
        sup.hub().wake();
        thread::sleep(Duration::from_millis(1));
    }
    assert!(
        sup.hub().latest_waiters() >= 1,
        "timed out waiting for armd-dispatch to block on StateHub::latest() after {timeout:?}"
    );
}

fn drain_broker(handle: &RtExecutorHandle) -> Vec<Reply> {
    let mut out = Vec::new();
    handle.broker().lock().unwrap().poll(|_, r| {
        out.push(r);
        true
    });
    out
}

fn sample_persist_job(action_id: u64, dir: &std::path::Path) -> PendingZeroCheckPersist {
    PendingZeroCheckPersist {
        action_id,
        offset_rad: 0.01,
        base_revision: 1,
        connection_id: 0,
        lease_owner: None,
        estop_epoch: 1,
        stop_gen: 0,
        deadline: std::time::Instant::now() + std::time::Duration::from_secs(5),
        ctx: Box::new(armd::supervisor::ZeroCheckPersistContext {
            path: dir.join(format!("gripper_calib_{action_id}.json")),
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
    }
}

// ============================================================================
// 1. Per-op TimedAlloc unit verification
// ============================================================================

#[test]
fn timed_alloc_counts_all_four_ops_independently_of_take() {
    thread::spawn(|| {
        // Non-opted-in thread stays zero across all four ops.
        let p = ParkedAlloc::new_alloc(64);
        let z = ParkedAlloc::new_alloc_zeroed(64);
        let p = p.realloc_to(128);
        drop(p);
        drop(z);
        assert_eq!(alloc::snapshot_counts(), AllocCounts::ZERO);

        alloc::time_this_thread();
        let c0 = alloc::snapshot_counts();
        let _ = alloc::take();

        let p = ParkedAlloc::new_alloc(64);
        let c1 = alloc::snapshot_counts();
        assert_eq!(c1.delta_since(c0), AllocCounts { alloc: 1, alloc_zeroed: 0, realloc: 0, dealloc: 0 });

        // Mid-window `alloc::take()` resets AllocTotals without resetting monotonic snapshot_counts.
        let taken = alloc::take();
        assert_eq!(taken.calls, 1);
        assert_eq!(alloc::take().calls, 0);
        assert_eq!(alloc::snapshot_counts(), c1);

        let z = ParkedAlloc::new_alloc_zeroed(64);
        let c2 = alloc::snapshot_counts();
        assert_eq!(c2.delta_since(c1), AllocCounts { alloc: 0, alloc_zeroed: 1, realloc: 0, dealloc: 0 });

        let p = p.realloc_to(128);
        let c3 = alloc::snapshot_counts();
        assert_eq!(c3.delta_since(c2), AllocCounts { alloc: 0, alloc_zeroed: 0, realloc: 1, dealloc: 0 });

        drop(p);
        drop(z);
        let c4 = alloc::snapshot_counts();
        assert_eq!(c4.delta_since(c3), AllocCounts { alloc: 0, alloc_zeroed: 0, realloc: 0, dealloc: 2 });
        assert_eq!(
            c4.delta_since(c0),
            AllocCounts { alloc: 1, alloc_zeroed: 1, realloc: 1, dealloc: 2 }
        );
    })
    .join()
    .unwrap();
}

// ============================================================================
// 2. Positive controls: exact counts in release tail (incl. saturated diag) & wake
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn positive_controls_detect_exact_ops_in_release_tail_saturated_diag_and_wakes() {
    let clock = ScriptedClock::new();
    // 14 ms confirm delay on a 10 ms period: Step 3 starts at T, Step 2 at T+10ms returns
    // Pending{wake: T+14ms} < deadline T+20ms, firing a real sub-release wake at T+14ms.
    let (drv, _) = WakeDriver::new(7, Duration::from_millis(14), Some(clock.clone()));
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;
    sup.register_ws_connection(1).await.unwrap();

    let rel_probe: Arc<Mutex<Option<TailAllocProbe>>> = Arc::new(Mutex::new(None));
    let wake_probe: Arc<Mutex<Option<TailAllocProbe>>> = Arc::new(Mutex::new(None));
    let hooks = PersistTestHooks {
        clock: Some(clock.clone()),
        inject_release_tail_once: Some(rel_probe.clone()),
        inject_wake_tail_once: Some(wake_probe.clone()),
        ..Default::default()
    };

    let period_ns = 10_000_000u64; // 10 ms
    let handle = sup
        .start_executor_with_hooks(alloc_cfg(Duration::from_nanos(period_ns)), hooks)
        .await
        .unwrap();

    // Baseline: 5 clean releases -> strictly 0 heap ops from release #1 onward!
    clock.run_steps(period_ns, 5);
    assert_eq!(handle.stats().completed_releases.load(Ordering::Acquire), 5);
    assert_eq!(handle.stats().release_allocs.snapshot(), AllocCounts::ZERO);
    assert_eq!(handle.stats().wake_allocs.snapshot(), AllocCounts::ZERO);
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);

    // (a) Release-tail exact controls: alloc, zeroed alloc, realloc, final free, alloc+final free.
    let park = Arc::new(Mutex::new(None::<ParkedAlloc>));
    let run_rel_probe = |probe: TailAllocProbe| -> AllocCounts {
        let before = handle.stats().release_allocs.snapshot();
        *rel_probe.lock().unwrap() = Some(probe);
        clock.run_steps(period_ns, 1);
        let after = handle.stats().release_allocs.snapshot();
        after.delta_since(before)
    };

    assert_eq!(
        run_rel_probe(TailAllocProbe::AllocOnce { park: park.clone() }),
        AllocCounts { alloc: 1, alloc_zeroed: 0, realloc: 0, dealloc: 0 }
    );
    let parked_for_realloc = park.lock().unwrap().take().expect("parked alloc");

    assert_eq!(
        run_rel_probe(TailAllocProbe::AllocZeroedOnce { park: park.clone() }),
        AllocCounts { alloc: 0, alloc_zeroed: 1, realloc: 0, dealloc: 0 }
    );
    let _drop_zeroed_off_thread = park.lock().unwrap().take().expect("parked zeroed");

    assert_eq!(
        run_rel_probe(TailAllocProbe::ReallocOnce { existing: Some(parked_for_realloc), park: park.clone() }),
        AllocCounts { alloc: 0, alloc_zeroed: 0, realloc: 1, dealloc: 0 }
    );
    let parked_for_final_free = park.lock().unwrap().take().expect("parked reallocated");

    assert_eq!(
        run_rel_probe(TailAllocProbe::FinalFreeLocal { local: Some(parked_for_final_free) }),
        AllocCounts { alloc: 0, alloc_zeroed: 0, realloc: 0, dealloc: 1 }
    );

    let prealloc_free = ParkedAlloc::new_alloc(64);
    assert_eq!(
        run_rel_probe(TailAllocProbe::AllocAndFinalFreeLocal {
            local: Some(prealloc_free),
            park: park.clone(),
        }),
        AllocCounts { alloc: 1, alloc_zeroed: 0, realloc: 0, dealloc: 1 }
    );
    drop(park.lock().unwrap().take());
    assert_eq!(handle.stats().wake_allocs.snapshot(), AllocCounts::ZERO);

    // (b) Positive control WHILE `diag_tx` (cap 256) is saturated and dropping samples:
    // cumulative counters outside the droppable ring MUST still record exact counts!
    let (rel_hub, hub_jh) = pause_hub(&sup);
    // Prime one clean release and wait until `armd-dispatch` has exited `drain_sinks()` and
    // blocked on `hub.latest()`, so it cannot drain `diag_rx` partway through the burst.
    clock.run_steps(period_ns, 1);
    wait_for_dispatcher_blocked_on_hub(&sup, Duration::from_secs(5));
    clock.run_steps(period_ns, DIAG_RING_CAPACITY as u64 + 20);
    let dropped_before = handle.stats().diag_dropped.load(Ordering::Relaxed);
    assert!(dropped_before >= 10, "expected diag_dropped >= 10, got {dropped_before}");

    let prealloc_sat = ParkedAlloc::new_alloc(64);
    let sat_delta = run_rel_probe(TailAllocProbe::AllocAndFinalFreeLocal {
        local: Some(prealloc_sat),
        park: park.clone(),
    });
    assert_eq!(
        sat_delta,
        AllocCounts { alloc: 1, alloc_zeroed: 0, realloc: 0, dealloc: 1 },
        "cumulative counter must report exact release-tail alloc+free even when TickSample is dropped"
    );
    assert!(
        handle.stats().diag_dropped.load(Ordering::Relaxed) > dropped_before,
        "confirm the probed release's TickSample was actually dropped by the full diag ring"
    );
    drop(park.lock().unwrap().take());
    rel_hub.send(()).unwrap();
    hub_jh.join().unwrap();

    // (c) Sub-release wake controls:
    // confirm_delay = 4ms on a 10ms period -> wake fires 4ms into the release!
    let submit_transition = |seq: u64, op: TransitionOp| -> RequestId {
        let now = clock.now();
        let auth = Authority {
            lease_generation: 0,
            epoch: sup.hub().latest().map_or(1, |s| s.obs.epoch_id),
            stop_generation: handle.stop_requests().current(),
            config_revision: 0,
            parent_generation: 0,
        };
        handle
            .broker()
            .lock()
            .unwrap()
            .submit(ClientId(1), seq, auth, now + Duration::from_secs(5), Submit::Transition(op))
            .unwrap()
    };

    // Normal wake -> 0 allocs/frees in BOTH release and wake windows!
    let rel_before_clean = handle.stats().release_allocs.snapshot();
    let wake_before_clean = handle.stats().wake_allocs.snapshot();
    let wakes_before = handle.stats().completed_wakes.load(Ordering::Acquire);
    let r1 = submit_transition(1, TransitionOp::TorqueOn);
    clock.run_steps(period_ns, 3);
    let replies1 = drain_broker(&handle);
    assert!(replies1.iter().any(|r| matches!(r, Reply::Terminal { request, outcome: Outcome::Completed, .. } if *request == r1)), "{replies1:?}");
    assert!(handle.stats().completed_wakes.load(Ordering::Acquire) > wakes_before, "expected >= 1 real sub-release wake");
    assert_eq!(handle.stats().release_allocs.snapshot().delta_since(rel_before_clean), AllocCounts::ZERO);
    assert_eq!(handle.stats().wake_allocs.snapshot().delta_since(wake_before_clean), AllocCounts::ZERO);

    // Wake tail positive control: inject 1 alloc + 1 final free in wake ->
    // MUST increment `wake_allocs` by (1,0,0,1) AND leave `release_allocs` delta at ZERO (disjoint)!
    let rel_before_probe = handle.stats().release_allocs.snapshot();
    let wake_before_probe = handle.stats().wake_allocs.snapshot();
    let wake_calls_before = handle.stats().wake_alloc_calls.load(Ordering::Acquire);
    let total_before_probe = handle.stats().total_allocs();
    *wake_probe.lock().unwrap() = Some(TailAllocProbe::AllocAndFinalFreeLocal {
        local: Some(ParkedAlloc::new_alloc(64)),
        park: park.clone(),
    });
    let r2 = submit_transition(2, TransitionOp::TorqueOff);
    clock.run_steps(period_ns, 3);
    let replies2 = drain_broker(&handle);
    assert!(replies2.iter().any(|r| matches!(r, Reply::Terminal { request, outcome: Outcome::Completed, .. } if *request == r2)), "{replies2:?}");
    assert_eq!(
        handle.stats().wake_allocs.snapshot().delta_since(wake_before_probe),
        AllocCounts { alloc: 1, alloc_zeroed: 0, realloc: 0, dealloc: 1 }
    );
    assert_eq!(
        handle.stats().release_allocs.snapshot().delta_since(rel_before_probe),
        AllocCounts::ZERO,
        "wake allocation must NOT double-count into release_allocs"
    );
    assert_eq!(
        handle.stats().total_allocs().delta_since(total_before_probe),
        AllocCounts { alloc: 1, alloc_zeroed: 0, realloc: 0, dealloc: 1 }
    );
    assert_eq!(handle.stats().wake_alloc_calls.load(Ordering::Acquire) - wake_calls_before, 2);
    drop(park.lock().unwrap().take());

    // Also verify `AllocZeroedOnce` and `ReallocOnce` in sub-release wakes.
    let wake_before_z = handle.stats().wake_allocs.snapshot();
    *wake_probe.lock().unwrap() = Some(TailAllocProbe::AllocZeroedOnce { park: park.clone() });
    let r3 = submit_transition(3, TransitionOp::TorqueOn);
    clock.run_steps(period_ns, 3);
    let _ = drain_broker(&handle);
    let _ = r3;
    assert_eq!(
        handle.stats().wake_allocs.snapshot().delta_since(wake_before_z),
        AllocCounts { alloc: 0, alloc_zeroed: 1, realloc: 0, dealloc: 0 }
    );
    let z_buf = park.lock().unwrap().take().expect("parked zeroed from wake");

    let wake_before_r = handle.stats().wake_allocs.snapshot();
    *wake_probe.lock().unwrap() = Some(TailAllocProbe::ReallocOnce { existing: Some(z_buf), park: park.clone() });
    let _r4 = submit_transition(4, TransitionOp::TorqueOff);
    clock.run_steps(period_ns, 3);
    let _ = drain_broker(&handle);
    assert_eq!(
        handle.stats().wake_allocs.snapshot().delta_since(wake_before_r),
        AllocCounts { alloc: 0, alloc_zeroed: 0, realloc: 1, dealloc: 0 }
    );
    drop(park.lock().unwrap().take());

    handle.stop_and_join();
}

// ============================================================================
// 3. Stalled-consumer matrix: normal & refused commits, urgent stop, terminal
//    retirement, contended observation prewarm, and saturated observation/diag
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn stalled_consumers_normal_refused_commits_urgent_stop_terminal_retirement_and_obs_diag_saturation() {
    let clock = ScriptedClock::new();
    // 14 ms confirm delay on a 10 ms period: Step 3 starts at T, Step 2 at T+10ms returns
    // Pending{wake: T+14ms} < deadline T+20ms, firing a real sub-release wake at T+14ms.
    let (drv, _) = WakeDriver::new(7, Duration::from_millis(14), Some(clock.clone()));
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;
    // Prepare a valid Cartesian path off-thread BEFORE transferring ownership.
    let (cart_slot, start_pos, init_epoch, motion_cfg, after_cart_q, after_cart_v) = {
        let mut inner = sup.inner().write().await;
        inner.register_live_connection(1).unwrap();
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).unwrap();
        armd::SupervisorInner::enable_torque(admitted).unwrap();
        inner.tick(0.02);
        let input = inner.plan_input().unwrap();
        let p0 = armd::motion::start_pose(&input).p;
        let goal = resolve_goal(&input, &GoalSpec::Absolute { p: [p0[0] + 0.02, p0[1], p0[2]], r: None });
        let path = plan_cartesian(&input, goal, Speed::default()).unwrap();
        let snap = inner.planning_snapshot(&armd::rt_core::ops::CommitCtx {
            arm: armd::rt_core::ids::ArmId::LOCAL,
            boot: armd::rt_core::broker::BootId(1),
            now: clock.now(),
            stop_generation: 0,
        });
        let slot = prepare_cartesian_path(&snap, &path, None).unwrap();
        let start = inner.commanded_positions.clone();
        let ep = inner.safety.epoch_id();
        let motion_cfg = Arc::new(armd::rt_core::requests::motion::MotionConfigSnapshot::from_supervisor(&inner));
        // Measure the exact (commanded_positions, commanded_velocities) produced after 1 tick of `cart_slot`
        // so `r_jog` (queued behind `r_cart` while `hub` is paused) matches the post-tick start state exactly.
        let (after_cart_q, after_cart_v) = {
            let mut probe = armd::SupervisorInner::new(inner.config.clone());
            probe.lease_enabled = false;
            probe.commanded_positions = start.clone();
            probe.measured_positions = start.clone();
            probe.target_positions = start.clone();
            let s = probe.current_admission_stamp();
            armd::SupervisorInner::enable_torque(probe.check_admission(s).unwrap()).unwrap();
            probe.tick(0.02);
            let (mut pw, pr) = armd::rt_core::broker::plan_slots(1, PlanSlot::default);
            let h = pw.stage(|s| s.copy_from(&slot).unwrap()).unwrap();
            let pctx = armd::rt_core::ops::CommitCtx {
                arm: armd::rt_core::ids::ArmId::LOCAL,
                boot: armd::rt_core::broker::BootId(1),
                now: clock.now(),
                stop_generation: 0,
            };
            probe.set_boot_id(armd::rt_core::broker::BootId(1));
            probe.register_live_connection(1).unwrap();
            let d = armd::rt_core::broker::Descriptor {
                stamp: armd::rt_core::broker::Stamp {
                    arm: armd::rt_core::ids::ArmId::LOCAL,
                    boot: armd::rt_core::broker::BootId(1),
                    request: armd::rt_core::broker::RequestId(1),
                    client: ClientId(1),
                    client_seq: 1,
                    authority: probe.authority(&pctx),
                    deadline: clock.now() + Duration::from_secs(5),
                },
                kind: armd::rt_core::broker::CommandKind::Install { plan: h },
            };
            probe.commit_prepared_slot(&d, pr.pin(h).unwrap(), &pctx).unwrap();
            probe.tick(0.01);
            (probe.commanded_positions.clone(), probe.commanded_velocities.clone())
        };
        (slot, start, ep, motion_cfg, after_cart_q, after_cart_v)
    };

    // Hold `hub.reader_guard()` BEFORE `start_executor` to prove contended startup prewarm
    // never blocks AND keeps release #1 onward at 0 heap ops across slot rotation!
    let (rel_hub, hub_jh) = pause_hub(&sup);
    let period_ns = 10_000_000u64;
    let handle = sup
        .start_executor_with_hooks(
            alloc_cfg(Duration::from_nanos(period_ns)),
            PersistTestHooks { clock: Some(clock.clone()), ..Default::default() },
        )
        .await
        .unwrap();

    let seq0 = sup.hub().published_seq();
    clock.run_steps(period_ns, 4);
    assert!(sup.hub().published_seq() >= seq0 + 4, "publication must advance while reader_guard is held");
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO, "contended startup releases must not allocate");

    let base_auth = |stop_gen: u64| Authority {
        lease_generation: 0,
        epoch: init_epoch,
        stop_generation: stop_gen,
        config_revision: 0,
        parent_generation: 0,
    };

    // (a) Refused commits while consumers are stalled (expired deadline, wrong epoch, stale start).
    let now = clock.now();
    let expired_dl = now.checked_sub(Duration::from_millis(5)).unwrap_or(now);
    let r_exp = handle.broker().lock().unwrap()
        .submit(ClientId(1), 1, base_auth(0), expired_dl, Submit::Immediate(ImmediateOp::Heartbeat))
        .unwrap();
    let r_ep = handle.broker().lock().unwrap()
        .submit(ClientId(1), 2, Authority { epoch: init_epoch + 99, ..base_auth(0) }, now + Duration::from_secs(5), Submit::Transition(TransitionOp::TorqueOff))
        .unwrap();
    let mut bad_start = start_pos.clone();
    bad_start[0] += 0.5;
    let mk_jog = |from: &[f32], v0: &[f32], to: &[f32]| {
        use armd::rt_core::requests::motion::{self as req_motion, MotionPlanSnapshot, MotionRequest, PlanAuthority};
        let snap = MotionPlanSnapshot {
            config: Arc::clone(&motion_cfg),
            commanded_positions: from.to_vec(),
            commanded_velocities: v0.to_vec(),
            measured_positions: from.to_vec(),
            torque_hold: None,
            power_hold: None,
            holding_after_clear: false,
            mit_active: false,
            authority: PlanAuthority {
                boot: armd::rt_core::broker::BootId(1),
                epoch: init_epoch,
                lease_generation: 0,
                stop_generation: 0,
                deadline: now + Duration::from_secs(5),
            },
            workspace_revision: motion_cfg.workspace_revision,
            motion_config_revision: motion_cfg.motion_config_revision,
        };
        req_motion::prepare_motion(&snap, &MotionRequest::jog(to.to_vec(), Some(0.2), None)).unwrap().slot
    };
    let zeros = vec![0.0f32; start_pos.len()];
    let stale_slot = mk_jog(&bad_start, &zeros, &start_pos);
    let r_stale = handle.broker().lock().unwrap()
        .submit(ClientId(1), 3, base_auth(0), now + Duration::from_secs(5), Submit::Install(&move |s: &mut PlanSlot| armd::rt_core::requests::motion::stage_jog(s, &stale_slot)))
        .unwrap();

    // (b) Normal commits: prepared Cartesian path (pinned PlanSlotRef) -> superseded by prepared jog -> authorized urgent stop!
    let r_cart = handle.broker().lock().unwrap()
        .submit(ClientId(1), 4, base_auth(0), now + Duration::from_secs(5), Submit::Install(&move |s: &mut PlanSlot| s.copy_from(&cart_slot).unwrap()))
        .unwrap();
    let mut q_jog = after_cart_q.clone();
    q_jog[0] += 0.04;
    let jog_slot = mk_jog(&after_cart_q, &after_cart_v, &q_jog);
    let r_jog = handle.broker().lock().unwrap()
        .submit(ClientId(1), 5, base_auth(0), now + Duration::from_secs(5), Submit::Install(&move |s: &mut PlanSlot| armd::rt_core::requests::motion::stage_jog(s, &jog_slot)))
        .unwrap();

    // Advance 5 releases so all 5 descriptors commit/refuse while broker & hub stay stalled!
    clock.run_steps(period_ns, 5);

    // Issue authorized urgent stop while `r_jog` is active and all consumers stay stalled!
    let stop_gen = handle.stop_requests().request();
    clock.run_steps(period_ns, 4);
    assert_eq!(handle.stop_requests().applied(), stop_gen);

    // (c) Saturate diagnostics ring (> 256 releases) while observation hub + broker stay stalled!
    // Wait until `armd-dispatch` has exited its pre-lock `drain_sinks()` call and blocked on
    // `StateHub::latest()`, so it cannot drain `diag_rx` partway through the saturation burst.
    wait_for_dispatcher_blocked_on_hub(&sup, Duration::from_secs(5));
    clock.run_steps(period_ns, DIAG_RING_CAPACITY as u64 + 30);
    assert!(
        handle.stats().diag_dropped.load(Ordering::Relaxed) >= 25,
        "expected diag_dropped >= 25 under stalled dispatcher"
    );
    assert!(
        sup.hub().published_seq() >= seq0 + DIAG_RING_CAPACITY as u64 + 40,
        "publication must keep advancing across full obs/diag/broker stall"
    );
    assert_eq!(sup.hub().tick_frees(), 0, "observation triple buffer must never free on RT");
    assert_eq!(
        handle.stats().total_allocs(),
        AllocCounts::ZERO,
        "stalled consumers across commits/refusals/supersession/urgent-stop/saturation must perform 0 heap ops"
    );

    // Unstall observation hub (rotates consumer's front slot back into producer!) & poll broker:
    rel_hub.send(()).unwrap();
    hub_jh.join().unwrap();
    let latest_obs = sup.hub().latest().expect("published snapshot after unstall");
    assert!(latest_obs.meta.seq >= seq0 + DIAG_RING_CAPACITY as u64 + 40);

    // Run additional releases AFTER slot 2 rotated in -> still 0 heap ops (proves off-thread front prewarm)!
    clock.run_steps(period_ns, 6);
    assert_eq!(
        handle.stats().total_allocs(),
        AllocCounts::ZERO,
        "post-unstall slot rotation must remain 0 heap ops on armd-control"
    );

    // Poll broker off-thread and verify exact replies + plan-slot reclamation!
    let replies = drain_broker(&handle);
    assert!(replies.iter().any(|r| matches!(r, Reply::Refused { request, code: "DEADLINE_EXPIRED", .. } if *request == r_exp)), "{replies:?}");
    assert!(replies.iter().any(|r| matches!(r, Reply::Refused { request, code: "EPOCH_MISMATCH", .. } if *request == r_ep)), "{replies:?}");
    assert!(replies.iter().any(|r| matches!(r, Reply::Refused { request, code: "PLAN_STALE", .. } if *request == r_stale)), "{replies:?}");
    // Verify post-publish supersession ordering: Terminal(r_cart, Superseded) precedes Ack(r_jog),
    // followed by Terminal(r_jog, Stopped)!
    let idx_sup = replies.iter().position(|r| matches!(r, Reply::Terminal { request, outcome: Outcome::Superseded, .. } if *request == r_cart))
        .expect("cart superseded terminal");
    let idx_jog_ack = replies.iter().position(|r| matches!(r, Reply::Ack { request, .. } if *request == r_jog))
        .expect("jog ack");
    let idx_jog_stop = replies.iter().position(|r| matches!(r, Reply::Terminal { request, outcome: Outcome::Stopped, .. } if *request == r_jog))
        .expect("jog stopped terminal");
    assert!(idx_sup < idx_jog_ack && idx_jog_ack < idx_jog_stop, "wrong ordering: {replies:?}");
    assert_eq!(
        handle.broker().lock().unwrap().stats().retired,
        3,
        "all 3 staged plan slots (stale + superseded cart + stopped jog) must be reclaimed off-thread"
    );

    // (d) Normal completion terminal retirement + transition-busy refusal (with stalled consumers):
    let (rel_hub2, hub_jh2) = pause_hub(&sup);
    let now2 = clock.now();
    let cur_q = sup.hub().try_latest();
    let _ = cur_q;
    // Submit TorqueOff + concurrent TorqueOn (second gets TRANSITION_BUSY pre-frame refusal!).
    let auth2 = base_auth(stop_gen);
    let r_off = handle.broker().lock().unwrap()
        .submit(ClientId(1), 10, auth2, now2 + Duration::from_secs(5), Submit::Transition(TransitionOp::TorqueOff))
        .unwrap();
    let r_busy = handle.broker().lock().unwrap()
        .submit(ClientId(1), 11, auth2, now2 + Duration::from_secs(5), Submit::Transition(TransitionOp::TorqueOn))
        .unwrap();
    clock.run_steps(period_ns, 5);
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);
    rel_hub2.send(()).unwrap();
    hub_jh2.join().unwrap();

    let replies2 = drain_broker(&handle);
    assert!(replies2.iter().any(|r| matches!(r, Reply::Terminal { request, outcome: Outcome::Completed, .. } if *request == r_off)), "{replies2:?}");
    assert!(replies2.iter().any(|r| matches!(r, Reply::Refused { request, .. } if *request == r_busy)), "{replies2:?}");
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);

    handle.stop_and_join();
}

// ============================================================================
// 4. Saturated E-stop queue (64-slot ring full + overflow) & saturated
//    persistence queue (4-slot job ring + reply saturation + off-thread reclaim)
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn saturated_estop_and_persistence_queues_zero_heap_ops_and_off_thread_reclaim() {
    // (a) Saturated E-stop queue (64-slot ring full + 12 overflows) while `hub.reader_guard()` is
    // held. Starts already latched in `EStop` so `fill_observation` does not hit B3's
    // `SafetyManager::fault()` String allocation (`src/safety.rs:368`); off-thread reclamation of
    // retired heap-bearing `Arc<SafetyState::Fault>` under saturated rings is also proved in
    // `rt_core::executor::tests::saturated_estop_and_reclaim_rings_retain_retired_fault_and_offload_on_drain_or_exit`.
    let clock_e = ScriptedClock::new();
    let sup_e = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let req_ctr = {
        let mut inner = sup_e.inner().write().await;
        inner.lease_enabled = false;
        inner.safety.trigger_e_stop();
        inner.requested_estop_count.clone()
    };
    let period_ns = 10_000_000u64;
    let h_e = sup_e
        .start_executor_with_hooks(
            alloc_cfg(Duration::from_nanos(period_ns)),
            PersistTestHooks { clock: Some(clock_e.clone()), ..Default::default() },
        )
        .await
        .unwrap();

    let (rel_e, jh_e) = pause_hub(&sup_e);
    let seq_e0 = sup_e.hub().published_seq();
    for _ in 0..(ESTOP_ACK_CAPACITY + 12) {
        req_ctr.fetch_add(1, Ordering::SeqCst);
        clock_e.run_steps(period_ns, 1);
    }
    assert_eq!(
        h_e.stats().total_allocs(),
        AllocCounts::ZERO,
        "76 full-ring + overflow Fault E-stops must do 0 heap ops on armd-control"
    );
    assert!(sup_e.hub().published_seq() >= seq_e0 + ESTOP_ACK_CAPACITY as u64 + 12);
    rel_e.send(()).unwrap();
    jh_e.join().unwrap();
    h_e.stop_and_join();

    // (b) Saturated persistence queue: pause `armd-persist`, seed a running zero-check action for
    // job #1 so `armd-persist` completes real `Reserve` + `Activate` after unpause, inject 6
    // pre-built `PendingZeroCheckPersist` jobs (4 fill `persist_job_tx`, 2 are rejected into
    // `offload_reclaim_tx` carrying `DropTid` witnesses), verify 0 heap ops on `armd-control`,
    // advancing publication, UNCONDITIONAL off-thread drop of BOTH rejected jobs AND the Reserve
    // payload, and full `Reserve` + `Activate` completion on disk (proving `armd-persist` was NOT
    // terminated by unsolicited replies).
    let dir = tempfile::tempdir().unwrap();
    let clock_p = ScriptedClock::new();
    let sup_p = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    {
        let mut inner = sup_p.inner().write().await;
        inner.lease_enabled = false;
        inner.gripper_calib_path = Some(dir.path().join("gripper_calib_1.json"));
        inner.last_action = Some(
            serde_json::from_value(serde_json::json!({
                "id": 1,
                "kind": "gripper_zero_check",
                "state": "running",
                "progress": 1.0,
                "started_at": 0.0,
            }))
            .expect("valid ActionStatus"),
        );
    }
    let pause_persist = Arc::new(AtomicBool::new(true));
    let job_slot: Arc<
        Mutex<Option<(PendingZeroCheckPersist, Option<Box<dyn std::any::Any + Send>>)>>,
    > = Arc::new(Mutex::new(None));
    let payload_dropped_on: Arc<Mutex<Option<ThreadId>>> = Arc::new(Mutex::new(None));
    let rej5_dropped_on: Arc<Mutex<Option<ThreadId>>> = Arc::new(Mutex::new(None));
    let rej6_dropped_on: Arc<Mutex<Option<ThreadId>>> = Arc::new(Mutex::new(None));
    struct DropTid(Arc<Mutex<Option<ThreadId>>>);
    impl Drop for DropTid {
        fn drop(&mut self) {
            *self.0.lock().unwrap() = Some(thread::current().id());
        }
    }
    let hooks_p = PersistTestHooks {
        clock: Some(clock_p.clone()),
        pause_persist_worker: Some(pause_persist.clone()),
        inject_pending_persist_job_once: Some(job_slot.clone()),
        payload_drop_probe: Some(Arc::new(Mutex::new(Some(Box::new(DropTid(
            payload_dropped_on.clone(),
        )))))),
        ..Default::default()
    };
    let h_p = sup_p
        .start_executor_with_hooks(alloc_cfg(Duration::from_nanos(period_ns)), hooks_p)
        .await
        .unwrap();
    let exec_tid = h_p.executor_thread_id();

    let (rel_p, jh_p) = pause_hub(&sup_p);
    let seq_p0 = sup_p.hub().published_seq();
    for id in 1..=6u64 {
        let probe: Option<Box<dyn std::any::Any + Send>> = match id {
            5 => Some(Box::new(DropTid(rej5_dropped_on.clone()))),
            6 => Some(Box::new(DropTid(rej6_dropped_on.clone()))),
            _ => None,
        };
        *job_slot.lock().unwrap() = Some((sample_persist_job(id, dir.path()), probe));
        clock_p.run_steps(period_ns, 1);
    }
    assert_eq!(
        h_p.stats().total_allocs(),
        AllocCounts::ZERO,
        "saturated persist_job_tx + repeated release-tail job rejections must do 0 heap ops on armd-control"
    );
    assert!(sup_p.hub().published_seq() >= seq_p0 + 6);
    assert!(rej5_dropped_on.lock().unwrap().is_none(), "rejected job #5 must remain alive while hub is paused");
    assert!(rej6_dropped_on.lock().unwrap().is_none(), "rejected job #6 must remain alive while hub is paused");

    // Unpause persistence worker & hub, allow off-thread drain, rejected-job drop, AND Reserve+Activate!
    pause_persist.store(false, Ordering::Release);
    rel_p.send(()).unwrap();
    jh_p.join().unwrap();
    let calib_1_path = dir.path().join("gripper_calib_1.json");
    let mut ticks = 0;
    while (rej5_dropped_on.lock().unwrap().is_none()
        || rej6_dropped_on.lock().unwrap().is_none()
        || payload_dropped_on.lock().unwrap().is_none()
        || !calib_1_path.exists())
        && ticks < 80
    {
        clock_p.run_steps(period_ns, 1);
        tokio::time::sleep(Duration::from_millis(5)).await;
        ticks += 1;
    }
    let obs_after = sup_p.get_observation().await;
    h_p.stop_and_join();
    assert_eq!(
        h_p.stats().total_allocs(),
        AllocCounts::ZERO,
        "post-unpause ticks={ticks} calib_exists={} action={:?} gripper_zero={:?} release={:?} wake={:?}",
        calib_1_path.exists(),
        obs_after.action,
        obs_after.gripper_zero,
        h_p.stats().release_allocs.snapshot(),
        h_p.stats().wake_allocs.snapshot(),
    );
    let rej5_tid = rej5_dropped_on
        .lock()
        .unwrap()
        .expect("rejected job #5 payload MUST drop off-thread");
    let rej6_tid = rej6_dropped_on
        .lock()
        .unwrap()
        .expect("rejected job #6 payload MUST drop off-thread");
    let reserve_tid = payload_dropped_on
        .lock()
        .unwrap()
        .expect("armd-persist Reserve payload MUST drop off-thread");
    assert_ne!(rej5_tid, exec_tid, "rejected job #5 must not drop on armd-control");
    assert_ne!(rej6_tid, exec_tid, "rejected job #6 must not drop on armd-control");
    assert_ne!(reserve_tid, exec_tid, "Reserve payload must not drop on armd-control");
    assert!(
        calib_1_path.exists(),
        "valid queued job #1 must complete Reserve+Activate on armd-persist after rejections"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn cold_start_driverless_default_resting_pose_and_legacy_j6_zero_alloc() {
    let period_ns = 10_000_000u64;

    // (a) Driverless profile with `zero_reference: Some(...)` and `resting_pose: None`:
    // default resting pose fallback must be prewarmed before release #1 (0 heap ops on release #1).
    let mut cfg_no_rest = ArmConfig::rebot_dm();
    cfg_no_rest.resting_pose = None;
    let clock_a = ScriptedClock::new();
    let sup_a = Arc::new(ArmSupervisor::new(cfg_no_rest));
    sup_a.set_lease_enabled(false).await;
    let h_a = sup_a
        .start_executor_with_hooks(
            alloc_cfg(Duration::from_nanos(period_ns)),
            PersistTestHooks { clock: Some(clock_a.clone()), ..Default::default() },
        )
        .await
        .unwrap();
    clock_a.run_steps(period_ns, 5);
    assert_eq!(h_a.stats().total_allocs(), AllocCounts::ZERO);
    h_a.stop_and_join();

    // (b) Legacy J6 detection (+90 deg in rest pose) on release #1 must format & log off-thread
    // (0 heap ops on `armd-control`) while preserving exact JSON message formatting.
    let clock_b = ScriptedClock::new();
    let sup_b = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    {
        let mut inner = sup_b.inner().write().await;
        inner.lease_enabled = false;
        inner.commanded_positions[5] = 1.574;
        inner.measured_positions[5] = 1.574;
        inner.target_positions[5] = 1.574;
    }
    let h_b = sup_b
        .start_executor_with_hooks(
            alloc_cfg(Duration::from_nanos(period_ns)),
            PersistTestHooks { clock: Some(clock_b.clone()), ..Default::default() },
        )
        .await
        .unwrap();
    clock_b.run_steps(period_ns, 5);
    assert_eq!(
        h_b.stats().total_allocs(),
        AllocCounts::ZERO,
        "legacy_j6 classification in step_zero_convention must perform 0 heap ops on armd-control"
    );
    h_b.stop_and_join();

    // Verify off-thread JSON serialization still renders the exact legacy_j6 warning text.
    let zc = armd::joint_zero::check_convention(
        &[0.049, 0.011, 0.027, 0.024, 0.001, 1.574, 0.052],
        &[0.0; 7],
        &[0.15, 0.08, 0.08, 0.08, 0.08, 0.08],
    )
    .expect("legacy_j6 verdict");
    let v = serde_json::to_value(zc).unwrap();
    assert_eq!(v["status"], "legacy_j6");
    assert!(
        v["message"].as_str().unwrap().contains("J6 reads 90.2 deg"),
        "unexpected legacy_j6 JSON: {v}"
    );
}

// ============================================================================
// 5. Post-B3 unblocked proofs (first E-stop, ClearFault refusal, LogWriter)
//    + remaining blocked allocations (driver_transitions enable fail, A8 actions.rs).
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn first_idle_to_estop_transition_zero_alloc() {
    let clock = ScriptedClock::new();
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    sup.set_lease_enabled(false).await;
    let req_ctr = sup.inner().read().await.requested_estop_count.clone();
    let period_ns = 10_000_000u64;
    let handle = sup
        .start_executor_with_hooks(
            alloc_cfg(Duration::from_nanos(period_ns)),
            PersistTestHooks { clock: Some(clock.clone()), ..Default::default() },
        )
        .await
        .unwrap();
    clock.run_steps(period_ns, 3);
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);
    let snap0 = sup.hub().latest().expect("initial published observation");
    assert_eq!(snap0.obs.safety_state, armd::safety::SafetyState::Idle);
    let epoch0 = snap0.obs.epoch_id;

    req_ctr.fetch_add(1, Ordering::SeqCst);
    clock.run_steps(period_ns, 3);
    let snap1 = sup.hub().latest().expect("post-estop published observation");
    assert_eq!(
        snap1.obs.safety_state,
        armd::safety::SafetyState::EStop,
        "first E-stop request must transition Idle -> EStop"
    );
    assert!(snap1.obs.epoch_id > epoch0, "E-stop latch must advance safety epoch ({} <= {epoch0})", snap1.obs.epoch_id);
    let counts = handle.stats().total_allocs();
    handle.stop_and_join();
    assert_eq!(counts, AllocCounts::ZERO, "first Idle->EStop allocated: {counts:?}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn clear_fault_refusal_zero_alloc() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    {
        let mut inner = sup.inner().write().await;
        inner.lease_enabled = false;
        inner.temperatures[0] = 95.0;
        inner.safety.latch(
            armd::safety::FaultKind::ThermalTrip,
            armd::safety::FaultDetail::from_reason(
                armd::safety::FaultCode::ThermalTrip,
                &armd::safety::FaultReason::ThermalTrip { joint_idx: 0, temperature: 95.0, limit: 85.0 },
            ),
        );
    }
    let handle = sup.start_executor(alloc_cfg(Duration::from_millis(5))).await.unwrap();
    wait_until(Duration::from_secs(1), || handle.stats().completed_releases.load(Ordering::Acquire) >= 4).await;
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);

    let err = handle
        .clear_fault_async()
        .await
        .unwrap()
        .expect_err("expected ClearFault refusal while ThermalTrip temperature exceeds cooldown");
    let expected = armd::safety::SafetyError::from(armd::safety::ClearFaultError::from(
        armd::safety::ClearRefusal::TemperatureAboveThreshold,
    ));
    assert_eq!(err, expected, "expected exact driverless TemperatureAboveThreshold refusal");
    let after = handle.stats().completed_releases.load(Ordering::Acquire);
    wait_until(Duration::from_secs(1), || handle.stats().completed_releases.load(Ordering::Acquire) >= after + 2).await;
    let snap = sup.hub().latest().expect("published observation after ClearFault refusal");
    assert!(
        matches!(
            snap.obs.safety_state,
            armd::safety::SafetyState::Fault(armd::safety::FaultReason::ThermalTrip { .. })
        ),
        "ThermalTrip fault must remain latched after refusal: {:?}",
        snap.obs.safety_state
    );
    let counts = handle.stats().total_allocs();
    handle.stop_and_join();
    assert_eq!(counts, AllocCounts::ZERO, "ClearFault refusal allocated: {counts:?}");
}

#[test]
fn logwriter_tracing_on_rt_zero_alloc() {
    #[derive(Clone, Default)]
    struct SharedBuf(Arc<Mutex<Vec<u8>>>);
    impl std::io::Write for SharedBuf {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.0.lock().unwrap().extend_from_slice(buf);
            Ok(buf.len())
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    thread::spawn(|| {
        let sink = SharedBuf::default();
        let captured = Arc::clone(&sink.0);
        let writer = armd::diagnostics::logwriter::NonBlocking::spawn_unregistered("b2-test", sink, 64).unwrap();
        let sub = tracing_subscriber::fmt().with_writer(writer.clone()).with_ansi(false).finish();
        let _guard = tracing::subscriber::set_default(sub);
        tracing::callsite::rebuild_interest_cache();
        let mut inner = armd::SupervisorInner::new(ArmConfig::rebot_dm());
        inner.safety.set_off_thread_reclaim(true);
        alloc::time_this_thread();
        let before = alloc::snapshot_counts();
        inner.safety.trigger_e_stop();
        let delta = alloc::snapshot_counts().delta_since(before);
        assert_eq!(delta, AllocCounts::ZERO, "safety log via LogWriter allocated: {delta:?}");

        // Verify the E-stop log event was actually queued, drained off the measured path,
        // and delivered to the underlying writer sink.
        let ev = inner.safety.pop_log_event().expect("deferred E-stop log event must be queued");
        ev.emit();
        assert!(writer.flush(Duration::from_secs(2)), "LogWriter failed to flush deferred event");
        let text = String::from_utf8(captured.lock().unwrap().clone()).unwrap();
        assert!(text.contains("Safety: Idle -> EStop"), "expected E-stop log in sink, got: {text:?}");
    })
    .join()
    .unwrap();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
#[ignore = "still allocates after B3: enable_step_failed allocates String via e.to_string() at src/supervisor/driver_transitions.rs:1509 and enable_failed_budgeted allocates format!()/SafetyError::ClearFaultRejected(String) at src/supervisor/driver_transitions.rs:1156,1163"]
async fn b3_blocked_torque_transition_failure_zero_alloc() {
    let clock = ScriptedClock::new();
    let (drv, fail_next) = WakeDriver::new(7, Duration::from_millis(4), Some(clock.clone()));
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;
    sup.register_ws_connection(1).await.unwrap();
    let period_ns = 10_000_000u64;
    let handle = sup
        .start_executor_with_hooks(
            alloc_cfg(Duration::from_nanos(period_ns)),
            PersistTestHooks { clock: Some(clock.clone()), ..Default::default() },
        )
        .await
        .unwrap();
    clock.run_steps(period_ns, 3);
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);

    fail_next.store(true, Ordering::Release);
    let now = clock.now();
    let auth = Authority {
        lease_generation: 0,
        epoch: sup.hub().latest().map_or(1, |s| s.obs.epoch_id),
        stop_generation: handle.stop_requests().current(),
        config_revision: 0,
        parent_generation: 0,
    };
    let _req = handle
        .broker()
        .lock()
        .unwrap()
        .submit(ClientId(1), 1, auth, now + Duration::from_secs(5), Submit::Transition(TransitionOp::TorqueOn))
        .unwrap();
    clock.run_steps(period_ns, 4);
    let counts = handle.stats().total_allocs();
    handle.stop_and_join();
    assert_eq!(counts, AllocCounts::ZERO, "transition failure allocated: {counts:?}");
}

#[test]
fn a8_blocked_zero_check_store_offset_serialization_zero_alloc() {
    for capacity_available in [false, true] {
        thread::spawn(move || {
            let dir = tempfile::tempdir().unwrap();
            let mut inner = armd::SupervisorInner::new(ArmConfig::rebot_dm());
            inner.lease_enabled = false;
            inner.gripper_calib_path = Some(dir.path().join("calib.json"));
            inner.defer_zero_check_persist = true;
            inner.persist_capacity_available = capacity_available;
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).unwrap();
            armd::SupervisorInner::enable_torque(admitted).unwrap();
            let epoch = inner.safety.epoch_id();
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).unwrap();
            let (id, _) = armd::SupervisorInner::start_gripper_zero_check(
                admitted,
                &serde_json::json!({"threshold_rad": 0.10, "store_offset": true}),
                epoch,
            )
            .unwrap();
            alloc::time_this_thread();
            let before = alloc::snapshot_counts();
            inner.finish_action(id, "done", None, None, None, None, None);
            let delta = alloc::snapshot_counts().delta_since(before);
            assert_eq!(
                delta,
                AllocCounts::ZERO,
                "zero_check(store_offset=true, capacity={capacity_available}) allocated on RT: {delta:?}"
            );
            if capacity_available {
                assert!(inner.pending_zero_check_persist.is_some());
                assert_eq!(inner.zero_check_persisting, Some(id));
            } else {
                assert!(inner.pending_zero_check_persist.is_none());
                assert_eq!(
                    inner.last_action.as_ref().and_then(|a| a.code.as_deref()),
                    Some("PERSIST_FAILED")
                );
            }
        })
        .join()
        .unwrap();
    }
}

#[test]
fn phase_marks_csw_mark_and_staged_publish_zero_alloc() {
    use armd::diagnostics::tick::{
        mid_csw_delta, stash_periodic_sample_with_post_end, take_periodic_sample_with_post_end,
        us, CswMark, Holder, LockFreeTickStats, ReleasePhasesUs, ThreadUsage, TickSample,
        UsageMark, WakeLatenessGuard,
    };

    let (delta, snap) = thread::spawn(|| {
        let lf = LockFreeTickStats::new();
        let loc = std::panic::Location::caller();
        alloc::time_this_thread();

        // Warm up TLS slots once before measuring.
        {
            let _g = WakeLatenessGuard::enter_periodic(100, true);
            stash_periodic_sample_with_post_end(TickSample::default(), Instant::now());
            let _ = take_periodic_sample_with_post_end();
        }
        let _ = CswMark::now();
        let _ = UsageMark::now();
        let _ = alloc::take();

        let before = alloc::snapshot_counts();
        for i in 0..200u32 {
            let _g = WakeLatenessGuard::enter_periodic(u64::from(i) * 1_000, true);
            let started = Instant::now();
            let usage_start = UsageMark::now();
            let locked = Instant::now();
            let usage_locked = UsageMark::now();
            let mid_csw_start = CswMark::now();
            let lock_end = Instant::now();
            let _allocs = alloc::take();
            let after_alloc_read = Instant::now();
            let after_ack_flush = Instant::now();
            let after_terminal = Instant::now();
            let released = Instant::now();
            let usage_released = UsageMark::now();
            let (mid_nvcsw, mid_nivcsw) = mid_csw_delta(&mid_csw_start, &usage_released);
            let done = Instant::now();
            let holder = Holder {
                kind: "write",
                location: loc,
                command: Some("/api/v1/motion/jog"),
                direction: None,
                action_id: Some(u64::from(i)),
                io_timing: None,
                hold_us: 50,
                acquire_us: 2,
                acquired_before_wait_us: 10,
                released_before_lock_us: 5,
            };
            let sample = TickSample {
                tick_us: us(done.saturating_duration_since(started)),
                release_us: 0,
                wait_us: i,
                lock_us: us(lock_end.saturating_duration_since(locked)),
                post_us: us(done.saturating_duration_since(released)),
                phases_us: ReleasePhasesUs {
                    alloc_read: us(after_alloc_read.saturating_duration_since(lock_end)),
                    ack_flush: us(after_ack_flush.saturating_duration_since(after_alloc_read)),
                    terminal: us(after_terminal.saturating_duration_since(after_ack_flush)),
                    guard_drop: us(released.saturating_duration_since(after_terminal)),
                    post: us(done.saturating_duration_since(released)),
                    publish: 0,
                },
                usage: ThreadUsage {
                    mid_nvcsw,
                    mid_nivcsw,
                    ..armd::diagnostics::tick::thread_usage(&usage_start, &usage_locked, &usage_released)
                },
                waited_for: Some(holder),
                longest_hold: Some(holder),
                ..Default::default()
            };
            stash_periodic_sample_with_post_end(sample, done);
            let (stashed, post_end) = take_periodic_sample_with_post_end().expect("stashed sample");
            let seq = lf
                .record_staged_with_publish(&stashed, || {
                    let now = Instant::now();
                    (
                        us(now.saturating_duration_since(started)),
                        us(now.saturating_duration_since(post_end)),
                    )
                })
                .expect("record_staged_with_publish");
            lf.amend_slot_release_and_publish_us(seq, 1, 1);
            lf.amend_slot_post_overrun(
                seq,
                16_000,
                200,
                armd::diagnostics::tick::PostOverrunPhasesUs {
                    step_to_ring_us: 100,
                    ring_publish_us: 50,
                    ring_to_seal_us: 30,
                    seal_us: 20,
                    ..Default::default()
                },
                40,
            );
        }
        let delta = alloc::snapshot_counts().delta_since(before);
        (delta, lf.snapshot())
    })
    .join()
    .unwrap();

    assert_eq!(
        delta,
        AllocCounts::ZERO,
        "phase marks + CswMark + record_staged_with_publish must allocate 0 bytes: {delta:?}"
    );
    let mw = snap.max_wait.expect("max_wait");
    assert_eq!(mw.sample.wait_us, 199);
    assert_eq!(mw.handoff_us, Some(5));
    assert!(snap.max_release.unwrap().sample.phases_us.publish >= 1);
}

#[test]
fn terminal_capture_action_completion_dwell_saturation_and_positive_control_zero_alloc() {
    use std::sync::atomic::AtomicUsize;

    // 1) Legacy control release (`control_tick`, `keep = false`): verify zero allocations on
    //    action completion, 50 dwell ticks (0 captures per tick), ring saturation (counted &
    //    marked, no dwell retry), and positive control proving `alloc::take()` covers capture.
    let (legacy_clean, legacy_dwell, legacy_sat, legacy_pos_sample_calls, legacy_pos_mono) =
        thread::spawn(|| {
            let rt = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .unwrap();
            rt.block_on(async {
                let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
                sup.set_lease_enabled(false).await;
                sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
                sup.inner().write().await.prewarm_observation_slots();
                // Warm up all 3 triple-buffer slots + TLS stats before opting in.
                for _ in 0..4 {
                    sup.control_tick(0.02).await;
                }

                alloc::time_this_thread();
                let _ = alloc::take();

                // (a) Real in-tick trajectory completion on legacy control release:
                //     `tick_control_inner` -> `finish_action` -> `capture_edge_terminals`
                //     (`capture_from` + SPSC push) all execute strictly inside the measured window.
                sup.inner().write().await.test_start_zero_heap_jog_rt(0.02, 0.04);
                let b_complete = alloc::snapshot_counts();
                for _ in 0..40 {
                    sup.control_tick(0.02).await;
                    if sup.test_terminal_capture_stats().captured >= 1 {
                        break;
                    }
                }
                let d_complete = alloc::snapshot_counts().delta_since(b_complete);
                let s_complete = sup.tick_samples().last().copied().unwrap();
                assert_eq!(s_complete.usage.lock_alloc_calls, 0, "completion lock_alloc_calls must be 0");
                let st_complete = sup.test_terminal_capture_stats();
                assert_eq!((st_complete.captured, st_complete.saturated, st_complete.ever_saturated), (1, 0, false));

                // (b) 50 dwell ticks after terminal action: 0 additional captures, 0 heap ops.
                let b_dwell = alloc::snapshot_counts();
                for _ in 0..50 {
                    sup.control_tick(0.02).await;
                }
                let d_dwell = alloc::snapshot_counts().delta_since(b_dwell);
                let s_dwell = sup.tick_samples().last().copied().unwrap();
                assert_eq!(s_dwell.usage.lock_alloc_calls, 0, "dwell lock_alloc_calls must be 0");
                let st_dwell = sup.test_terminal_capture_stats();
                assert_eq!(st_dwell.captured, 1, "dwell must perform 0 captures per tick");
                assert_eq!(st_dwell.saturated, 0);

                // (c) Ring saturation: pause drain, cap ring at 4, complete 8 more real in-tick
                //     jogs -> 4 captured, 5 saturated, ever_saturated=true, 0 heap ops, and
                //     20 dwell ticks do not retry.
                sup.test_set_terminal_capture_drain_paused(true);
                sup.test_set_terminal_capture_push_cap(4);
                let mut d_sat = AllocCounts::ZERO;
                for k in 0..8 {
                    let delta = if k % 2 == 0 { -0.02 } else { 0.02 };
                    sup.inner().write().await.test_start_zero_heap_jog_rt(delta, 0.04);
                    let target_total = 1 + (k + 1) as u64;
                    for _ in 0..40 {
                        let before_tick = alloc::snapshot_counts();
                        sup.control_tick(0.02).await;
                        let delta_tick = alloc::snapshot_counts().delta_since(before_tick);
                        d_sat.alloc += delta_tick.alloc;
                        d_sat.alloc_zeroed += delta_tick.alloc_zeroed;
                        d_sat.realloc += delta_tick.realloc;
                        d_sat.dealloc += delta_tick.dealloc;
                        assert_eq!(
                            sup.tick_samples().last().copied().unwrap().usage.lock_alloc_calls,
                            0,
                            "saturated completion release lock_alloc_calls must be 0"
                        );
                        let st = sup.test_terminal_capture_stats();
                        if st.captured + st.saturated >= target_total {
                            break;
                        }
                    }
                }
                let b_sat_dwell = alloc::snapshot_counts();
                for _ in 0..20 {
                    sup.control_tick(0.02).await;
                }
                let d_sat_dwell = alloc::snapshot_counts().delta_since(b_sat_dwell);
                d_sat.alloc += d_sat_dwell.alloc;
                d_sat.alloc_zeroed += d_sat_dwell.alloc_zeroed;
                d_sat.realloc += d_sat_dwell.realloc;
                d_sat.dealloc += d_sat_dwell.dealloc;
                let st_sat = sup.test_terminal_capture_stats();
                assert_eq!(st_sat.captured, 4, "cap=4 (1 already queued + 3 more)");
                assert_eq!(st_sat.saturated, 5, "5 saturated edges must be counted exactly once each");
                assert!(st_sat.ever_saturated, "ever_saturated must latch true");
                let s_sat = sup.tick_samples().last().copied().unwrap();
                assert_eq!(s_sat.usage.lock_alloc_calls, 0, "saturated release lock_alloc_calls must be 0");

                // (d) Positive control: inject a heap allocation strictly inside capture_edge_terminals
                //     and verify BOTH `lock_alloc_calls` (moved `alloc::take()`) AND monotonic counts fail.
                let fired = Arc::new(AtomicUsize::new(0));
                let fired_c = Arc::clone(&fired);
                sup.inner().write().await.terminal_capture_test_hook = Some(Arc::new(move |_, entering| {
                    if entering && fired_c.fetch_add(1, Ordering::AcqRel) == 0 {
                        let b = Box::new([0xA5u8; 128]);
                        std::hint::black_box(&b);
                        drop(b);
                    }
                }));
                let b_pos = alloc::snapshot_counts();
                sup.control_tick(0.02).await;
                let d_pos = alloc::snapshot_counts().delta_since(b_pos);
                let s_pos = sup.tick_samples().last().copied().unwrap();
                sup.inner().write().await.terminal_capture_test_hook = None;
                (d_complete, d_dwell, d_sat, s_pos.usage.lock_alloc_calls, d_pos)
            })
        })
        .join()
        .unwrap();

    assert_eq!(legacy_clean, AllocCounts::ZERO, "legacy terminal completion must allocate 0 bytes");
    assert_eq!(legacy_dwell, AllocCounts::ZERO, "legacy dwell must allocate 0 bytes");
    assert_eq!(legacy_sat, AllocCounts::ZERO, "legacy saturated ring must allocate 0 bytes");
    assert!(
        legacy_pos_sample_calls >= 2,
        "positive control: injected alloc in capture MUST be caught by TickSample.usage.lock_alloc_calls (got {legacy_pos_sample_calls})"
    );
    assert!(
        legacy_pos_mono.alloc >= 1 && legacy_pos_mono.dealloc >= 1,
        "positive control: injected alloc in capture MUST be caught by monotonic AllocCounts (got {legacy_pos_mono:?})"
    );

    // 2) RT Executor (`start_executor_with_hooks`): action completion, dwell (0 captures/tick),
    //    ring saturation, and positive control on `armd-control`.
    let rt = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .enable_all()
        .build()
        .unwrap();
    rt.block_on(async {
        let clock = ScriptedClock::new();
        let (drv, _) = WakeDriver::new(7, Duration::from_millis(14), Some(clock.clone()));
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
        sup.set_lease_enabled(false).await;
        sup.test_set_terminal_capture_drain_paused(true);
        sup.test_set_terminal_capture_push_cap(2);
        sup.register_ws_connection(1).await.unwrap();

        let hook_arm = Arc::new(AtomicBool::new(false));
        let hook_arm_c = Arc::clone(&hook_arm);
        sup.inner().write().await.terminal_capture_test_hook = Some(Arc::new(move |_, entering| {
            if entering && hook_arm_c.swap(false, Ordering::AcqRel) {
                let b = Box::new(0x55AA_u64);
                std::hint::black_box(&b);
                drop(b);
            }
        }));

        // Capture motion config snapshot before starting executor.
        let motion_cfg = {
            let inner = sup.inner().read().await;
            Arc::new(armd::rt_core::requests::motion::MotionConfigSnapshot::from_supervisor(&inner))
        };

        let hooks = PersistTestHooks {
            clock: Some(clock.clone()),
            ..Default::default()
        };
        let handle = sup
            .start_executor_with_hooks(
                ExecutorConfig {
                    alloc_timing: true,
                    ..ExecutorConfig::for_tests(Duration::from_millis(10))
                },
                hooks,
            )
            .await
            .unwrap();
        let period_ns = 10_000_000u64;
        clock.run_steps(period_ns, 3);
        assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);

        // Enable torque (sub-release wake) then run 4 short jogs -> 2 captured, 2 saturated.
        let auth0 = Authority {
            lease_generation: 0,
            epoch: sup.capture_admission_stamp().estop_epoch,
            stop_generation: 0,
            config_revision: 0,
            parent_generation: 0,
        };
        let _tok = handle
            .broker()
            .lock()
            .unwrap()
            .submit(
                ClientId(1),
                1,
                auth0,
                clock.now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOn),
            )
            .unwrap();
        clock.run_steps(period_ns, 4);

        let mut client_seq = 10u64;
        for k in 0..4u64 {
            let snap = sup.hub().latest().unwrap();
            let from = snap.rt.commanded_positions.clone();
            let v0 = vec![0.0f32; from.len()];
            let mut to = from.clone();
            to[0] += if k % 2 == 0 { 0.02 } else { -0.02 };
            let msnap = armd::rt_core::requests::motion::MotionPlanSnapshot {
                config: Arc::clone(&motion_cfg),
                commanded_positions: from.clone(),
                commanded_velocities: v0,
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
                    deadline: clock.now() + Duration::from_secs(5),
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
            client_seq += 1;
            let auth = Authority {
                lease_generation: 0,
                epoch: snap.obs.epoch_id,
                stop_generation: snap.rt.stop_generation,
                config_revision: 0,
                parent_generation: 0,
            };
            let req = handle
                .broker()
                .lock()
                .unwrap()
                .submit(
                    ClientId(1),
                    client_seq,
                    auth,
                    clock.now() + Duration::from_secs(5),
                    Submit::Install(&move |s: &mut PlanSlot| {
                        armd::rt_core::requests::motion::stage_jog(s, &jog_slot);
                    }),
                )
                .unwrap();
            // Advance until this jog reaches a terminal state (bounded).
            let mut finished = false;
            for _ in 0..80 {
                clock.run_steps(period_ns, 1);
                let _ = drain_broker(&handle);
                if let Some(s) = sup.hub().latest() {
                    if s.obs
                        .action
                        .as_ref()
                        .is_some_and(|a| a.id > k && !a.is_running())
                    {
                        finished = true;
                        break;
                    }
                }
            }
            assert!(finished, "jog {k} ({req:?}) did not reach terminal state");
        }

        let stats_after_moves = sup.test_terminal_capture_stats();
        assert_eq!(stats_after_moves.captured, 2, "push_cap=2: {stats_after_moves:?}");
        assert_eq!(stats_after_moves.saturated, 2, "2 actions must saturate: {stats_after_moves:?}");
        assert!(stats_after_moves.ever_saturated);
        assert_eq!(
            handle.stats().total_allocs(),
            AllocCounts::ZERO,
            "RT executor action completion + saturation must perform 0 heap ops"
        );

        // 20 dwell releases: 0 additional captures, 0 additional saturates, 0 heap ops.
        clock.run_steps(period_ns, 20);
        let stats_after_dwell = sup.test_terminal_capture_stats();
        assert_eq!(stats_after_dwell.captured, stats_after_moves.captured);
        assert_eq!(stats_after_dwell.saturated, stats_after_moves.saturated);
        assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);

        // Positive control on RT executor: arm the capture hook allocation -> total_allocs() > 0.
        hook_arm.store(true, Ordering::Release);
        clock.run_steps(period_ns, 1);
        let pos_allocs = handle.stats().total_allocs();
        assert!(
            pos_allocs.alloc >= 1 && pos_allocs.dealloc >= 1,
            "positive control: injected capture alloc on RT executor must be caught: {pos_allocs:?}"
        );

        handle.stop_and_join();
    });
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn cf1_calibration_and_software_zero_executor_zero_rt_alloc() {
    use armd::calibration::{CalibrationDeps, ExecutorCalibAuth};
    use armd::model::HolderLabel;
    use armd::motor_zero::{self, MotorZeroRequest};
    use armd::rt_core::requests::config::capture_config_admission;

    let dir = tempfile::tempdir().unwrap();
    let calib_path = dir.path().join("calibration.json");
    let setup_path = dir.path().join("grasp_setup.json");

    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let lease_conn = "alloc-cf1-conn".to_string();
    let (lease_gen, epoch) = {
        let mut inner = sup.inner().write().await;
        inner.measured_positions = vec![0.0, 0.25, -0.25, 0.0, 0.0, 0.0, 0.0];
        inner.commanded_positions = inner.measured_positions.clone();
        inner.target_positions = inner.measured_positions.clone();
        // Seed populated tracking history WITH occupied spares so executor handoff
        // (`normalize_spares_off_thread`) normalizes colliding spares while preserving active
        // history, and the first RT tick + software-zero activation execute 0 allocs / 0 frees.
        inner.tracking.record_issued(vec![0.1; 7], vec![0.01; 7]);
        inner.tracking.invalidate(); // moves vectors into spare_issued_*
        inner.tracking.record_issued(vec![0.2; 7], vec![0.02; 7]);
        inner.tracking.last_nominal = Some((vec![0.2; 7], vec![0.0; 7]));
        let g = inner.acquire_lease(&lease_conn).unwrap();
        (g, inner.safety.epoch_id())
    };
    let handle = sup
        .start_executor(alloc_cfg(Duration::from_millis(2)))
        .await
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(3);
    while sup.hub().latest().is_none() {
        assert!(Instant::now() < deadline);
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);

    let det_up = armd::api::proxy::DetectorUpstream::parse("http://127.0.0.1:9").unwrap();
    let cal_deps = CalibrationDeps::new(calib_path, Some(setup_path), det_up);

    // Software joint zero + joint-zero restore on Executor MUST perform 0 RT allocs/frees.
    let auth_zero = ExecutorCalibAuth {
        admission: capture_config_admission(&sup, 1, lease_gen, Some(epoch)),
        lease_holder: Some((HolderLabel::new(&lease_conn), lease_gen)),
        operator_grant: armd::auth::OperatorGrant::unenforced(),
        auth_check: Arc::new(|| Ok(())),
        txn_id: None,
    };
    let sw_req = MotorZeroRequest {
        joints: vec![1, 2],
        dry_run: false,
        force: false,
        method: Some("software".into()),
    };
    let sw_out = motor_zero::run_software_executor(&sup, &cal_deps, sw_req, auth_zero)
        .await
        .expect("software zero on executor");
    let backup = sw_out["restore"]["backup"].as_str().unwrap().to_string();

    let auth_restore = ExecutorCalibAuth {
        admission: capture_config_admission(&sup, 1, lease_gen, Some(epoch)),
        lease_holder: Some((HolderLabel::new(&lease_conn), lease_gen)),
        operator_grant: armd::auth::OperatorGrant::unenforced(),
        auth_check: Arc::new(|| Ok(())),
        txn_id: None,
    };
    let _restored = motor_zero::restore_executor(&sup, &cal_deps, backup, auth_restore)
        .await
        .expect("joint-zero restore on executor");

    let counts = handle.stats().total_allocs();
    let lock_attempts = sup.inner().control_thread_attempts();
    tokio::task::spawn_blocking(move || handle.stop_and_join())
        .await
        .unwrap();
    assert_eq!(
        counts,
        AllocCounts::ZERO,
        "C-F1 software zero + restore allocated on RT: {counts:?}"
    );
    assert_eq!(lock_attempts, 0, "C-F1 attempted SupervisorLock on RT");

    // Directly verify zero RT alloc/free when TrackingRuntime holds occupied active history,
    // occupied spares, heap-backed SlotBuf depths (len > 16), AND a populated FallbackRamp
    // simultaneously at the moment of software-zero invalidation.
    let mut inner = armd::supervisor::SupervisorInner::new(ArmConfig::rebot_dm());
    inner.tracking.record_issued(vec![0.1; 7], vec![0.01; 7]);
    inner.tracking.invalidate(); // fills spares
    inner.tracking.record_issued_with_ws_depths(
        vec![0.2; 7],
        vec![0.02; 7],
        Some((1, armd::kinematics::SlotBuf::zeros(32))),
        Some((1, armd::kinematics::SlotBuf::zeros(32))),
    );
    inner.tracking.last_nominal = Some((vec![0.2; 7], vec![0.0; 7]));
    inner.tracking.fallback_ramp = Some(armd::tracking::FallbackRamp {
        hold: vec![0.0; 7],
        keepout_only: false,
        limits: Vec::with_capacity(7),
    });
    alloc::time_this_thread();
    let before = alloc::snapshot_counts();
    let retired = inner.tracking.invalidate_retiring_for_test();
    let delta = alloc::snapshot_counts().delta_since(before);
    assert_eq!(
        delta,
        AllocCounts::ZERO,
        "invalidate_retiring allocated/freed on RT with occupied spares + heap depths + fallback_ramp: {delta:?}"
    );
    // Dropping `retired` off-RT frees the displaced buffers (positive control: dealloc > 0).
    let before_drop = alloc::snapshot_counts();
    drop(retired);
    let drop_delta = alloc::snapshot_counts().delta_since(before_drop);
    assert!(
        drop_delta.dealloc >= 6,
        "retired tracking payload must actually own displaced heap buffers for off-RT drop: {drop_delta:?}"
    );
    drop(inner);
}

// ============================================================================
// Primitive 5: Reusable RT Allocation & Lock Gate Suite
// (Success, Rejection, Recovery, Park, and Lock-Boundary Positive Controls)
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn rt_gate_success_rejection_recovery_and_park_zero_alloc_zero_lock() {
    use armd::rt_core::rt_gate::RtReleaseGate;
    use armd::rt_core::requests::motion::{
        prepare_motion, stage_jog, stage_park, MotionConfigSnapshot, MotionPlanSnapshot,
        MotionRequest,
    };
    use armd::safety::SafetyState;

    let clock = ScriptedClock::new();
    let (drv, _) = WakeDriver::new(7, Duration::from_millis(4), Some(clock.clone()));
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;
    sup.register_ws_connection(1).await.unwrap();

    let motion_cfg = {
        let inner = sup.inner().read().await;
        Arc::new(MotionConfigSnapshot::from_supervisor(&inner))
    };

    let estop_req = sup.inner().read().await.requested_estop_count.clone();
    let tail_probe: Arc<Mutex<Option<TailAllocProbe>>> = Arc::new(Mutex::new(None));
    let parked_alloc: Arc<Mutex<Option<ParkedAlloc>>> = Arc::new(Mutex::new(None));

    let period_ns = 10_000_000u64;
    let handle = sup
        .start_executor_with_hooks(
            alloc_cfg(Duration::from_nanos(period_ns)),
            PersistTestHooks {
                clock: Some(clock.clone()),
                inject_release_tail_once: Some(Arc::clone(&tail_probe)),
                ..Default::default()
            },
        )
        .await
        .unwrap();
    let gate = RtReleaseGate::new(&sup, &handle);

    // Warm up initial publication under the gate.
    let (_, warm_delta) = gate.run_phase("warmup_publish", || {
        clock.run_steps(period_ns, 3);
        ((), sup.hub().latest().is_some())
    });
    assert!(warm_delta.releases >= 3);

    let mut client_seq = 1u64;
    let next_auth = || {
        let s = sup.hub().latest().expect("published obs");
        Authority {
            lease_generation: 0,
            epoch: s.obs.epoch_id,
            stop_generation: s.rt.stop_generation,
            config_revision: 0,
            parent_generation: 0,
        }
    };

    // 1. SUCCESS: TorqueOn with real sub-release confirmation wake.
    let (_, en_delta) = gate.run_phase("success_torque_on_subrelease_wake", || {
        let auth = next_auth();
        client_seq += 1;
        let req = handle
            .broker()
            .lock()
            .unwrap()
            .submit(
                ClientId(1),
                client_seq,
                auth,
                clock.now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOn),
            )
            .unwrap();
        let mut completed = false;
        for _ in 0..8 {
            clock.run_steps(period_ns, 1);
            let rs = drain_broker(&handle);
            if rs.iter().any(|r| {
                matches!(r, Reply::Terminal { request, outcome: Outcome::Completed, .. } if *request == req)
            }) {
                completed = true;
                break;
            }
        }
        let enabled = sup
            .hub()
            .latest()
            .is_some_and(|s| s.obs.enable_state == EnableState::Enabled);
        ((), completed && enabled)
    });
    assert!(en_delta.is_clean());

    // 2. SUCCESS: Motion install + execution + terminal completion.
    let (_, jog_delta) = gate.run_phase("success_jog_install_and_complete", || {
        let snap = sup.hub().latest().unwrap();
        let msnap = MotionPlanSnapshot::from_published(&snap, Arc::clone(&motion_cfg)).unwrap();
        let mut to = snap.rt.commanded_positions.clone();
        to[0] += 0.04;
        let prepared = prepare_motion(&msnap, &MotionRequest::jog(to.clone(), Some(0.05), None)).unwrap();
        let auth = next_auth();
        client_seq += 1;
        let req = handle
            .broker()
            .lock()
            .unwrap()
            .submit(
                ClientId(1),
                client_seq,
                auth,
                clock.now() + Duration::from_secs(5),
                Submit::Install(&|s: &mut PlanSlot| stage_jog(s, &prepared.slot)),
            )
            .unwrap();
        let mut completed = false;
        for _ in 0..100 {
            clock.run_steps(period_ns, 1);
            let rs = drain_broker(&handle);
            if rs.iter().any(|r| {
                matches!(r, Reply::Terminal { request, outcome: Outcome::Completed, .. } if *request == req)
            }) {
                completed = true;
                break;
            }
        }
        let reached = sup
            .hub()
            .latest()
            .is_some_and(|s| (s.rt.commanded_positions[0] - to[0]).abs() < 1e-3);
        ((), completed && reached)
    });
    assert!(jog_delta.is_clean());

    // 3. REJECTION: Expired deadline & wrong epoch (zero alloc, zero lock).
    let (_, rej_delta) = gate.run_phase("rejection_expired_deadline_and_wrong_epoch", || {
        let auth = next_auth();
        let expired_dl = clock.now().checked_sub(Duration::from_millis(1)).unwrap_or(clock.now());
        client_seq += 1;
        let r_exp = handle
            .broker()
            .lock()
            .unwrap()
            .submit(
                ClientId(1),
                client_seq,
                auth,
                expired_dl,
                Submit::Transition(TransitionOp::TorqueOff),
            )
            .unwrap();
        let bad_epoch_auth = Authority {
            epoch: auth.epoch.saturating_add(99),
            ..auth
        };
        client_seq += 1;
        let r_ep = handle
            .broker()
            .lock()
            .unwrap()
            .submit(
                ClientId(1),
                client_seq,
                bad_epoch_auth,
                clock.now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOff),
            )
            .unwrap();
        clock.run_steps(period_ns, 4);
        let rs = drain_broker(&handle);
        let saw_expired = rs.iter().any(|r| {
            matches!(r, Reply::Refused { request, code: "DEADLINE_EXPIRED", .. } if *request == r_exp)
        });
        let saw_epoch = rs.iter().any(|r| {
            matches!(r, Reply::Refused { request, code: "EPOCH_MISMATCH", .. } if *request == r_ep)
        });
        ((), saw_expired && saw_epoch)
    });
    assert!(rej_delta.is_clean());

    // 4. PARK: Client InstallPark (moves to rest, settles, disables through TorqueTransition).
    let (_, park_delta) = gate.run_phase("park_install_move_settle_disable", || {
        let snap = sup.hub().latest().unwrap();
        let msnap = MotionPlanSnapshot::from_published(&snap, Arc::clone(&motion_cfg)).unwrap();
        let park_plan = prepare_motion(&msnap, &MotionRequest::park()).unwrap();
        let auth = next_auth();
        client_seq += 1;
        let r_park = handle
            .broker()
            .lock()
            .unwrap()
            .submit(
                ClientId(1),
                client_seq,
                auth,
                clock.now() + Duration::from_secs(10),
                Submit::InstallPark(&|s: &mut PlanSlot| stage_park(s, &park_plan.slot)),
            )
            .unwrap();
        let mut park_done = false;
        for _ in 0..320 {
            clock.run_steps(period_ns, 2);
            let rs = drain_broker(&handle);
            if rs.iter().any(|r| {
                matches!(r, Reply::Terminal { request, outcome: Outcome::Completed, .. } if *request == r_park)
            }) {
                park_done = true;
                break;
            }
        }
        let s_after = sup.hub().latest().unwrap();
        let parked_ok =
            park_done && !s_after.obs.is_torqued && s_after.obs.safety_state == SafetyState::Idle;
        ((), parked_ok)
    });
    assert!(park_delta.is_clean());

    // 5. RECOVERY: E-Stop latch (Idle -> EStop) followed by ClearFault recovery (EStop -> Idle).
    let epoch_before_estop = sup.hub().latest().unwrap().obs.epoch_id;
    let (_, estop_delta) = gate.run_phase("recovery_estop_latch", || {
        estop_req.fetch_add(1, Ordering::SeqCst);
        clock.run_steps(period_ns, 3);
        let s = sup.hub().latest().unwrap();
        (
            (),
            s.obs.safety_state == SafetyState::EStop && s.obs.epoch_id > epoch_before_estop,
        )
    });
    assert!(estop_delta.is_clean());

    let (_, clear_delta) = gate
        .run_phase_async("recovery_clear_fault_to_idle", async {
            let clock_pump = Arc::clone(&clock);
            let clear_fut = handle.clear_fault_async();
            tokio::pin!(clear_fut);
            let mut cleared = None;
            for _ in 0..20 {
                let c = Arc::clone(&clock_pump);
                let _ = tokio::task::spawn_blocking(move || c.run_steps(period_ns, 1)).await;
                if let Ok(res) = tokio::time::timeout(Duration::from_millis(5), &mut clear_fut).await
                {
                    cleared = Some(res);
                    break;
                }
            }
            let ok = matches!(cleared, Some(Ok(Ok(_))));
            let back_idle = sup
                .hub()
                .latest()
                .is_some_and(|s| s.obs.safety_state == SafetyState::Idle);
            ((), ok && back_idle)
        })
        .await;
    assert!(clear_delta.is_clean());

    // 6. POSITIVE CONTROL (Allocation): inject one release-tail allocation and confirm the gate catches it.
    let prealloc_free = ParkedAlloc::new_alloc(64);
    let pos_guard = gate.begin_phase("positive_control_injected_alloc");
    *tail_probe.lock().unwrap() = Some(TailAllocProbe::AllocAndFinalFreeLocal {
        local: Some(prealloc_free),
        park: Arc::clone(&parked_alloc),
    });
    clock.run_steps(period_ns, 1);
    let pos_delta = pos_guard.finish();
    assert!(
        !pos_delta.is_clean() && pos_delta.release_allocs.alloc >= 1 && pos_delta.release_allocs.dealloc >= 1,
        "gate positive control must catch injected RT heap allocation/deallocation: {pos_delta:?}"
    );
    drop(parked_alloc.lock().unwrap().take());

    tokio::task::spawn_blocking(move || handle.stop_and_join())
        .await
        .unwrap();
}

#[test]
fn rt_gate_marked_release_mode_switch_success_and_rollback_zero_alloc_zero_lock() {
    use std::collections::VecDeque;
    use armd::driver::{gripper_drive_from_profile, DamiaoArmDriver, TxBudget};
    use armd::motor_mode::{
        ControlMode, ModeProgress, ModeSwitch, ModeSwitchPlan, ModeSwitchRecovery,
        ReleaseBudget as ModeSwitchBudget,
    };
    use armd::protocol::damiao::{self, DamiaoLimits};
    use armd::rt_core::budget::ReleaseBudget;
    use armd::rt_core::rt_gate::measure_marked_release;
    use armd::transport::{CanFrame, CanTransport, DriverClock, MockClock, TransportError};

    /// Exclusively owned, lock-free, pre-allocated CAN transport for marked RT release proofs.
    /// Acquires zero mutexes anywhere (not even test-harness locks) and performs zero allocations
    /// once constructed.
    struct LockFreeCanBus {
        clock: Arc<MockClock>,
        limits: [DamiaoLimits; 7],
        enabled: [bool; 7],
        ctrl_mode: [u32; 7],
        tmax_override: [Option<f32>; 7],
        rx: VecDeque<(CanFrame, Instant)>,
    }

    impl LockFreeCanBus {
        fn new(clock: Arc<MockClock>, limits: [DamiaoLimits; 7], j2_bad_tmax: Option<f32>) -> Self {
            let mut ctrl_mode = [damiao::CTRL_MODE_POS_VEL; 7];
            ctrl_mode[6] = damiao::CTRL_MODE_POS_FORCE;
            let mut tmax_override = [None; 7];
            tmax_override[1] = j2_bad_tmax;
            let mut bus = Self {
                clock,
                limits,
                enabled: [false; 7],
                ctrl_mode,
                tmax_override,
                rx: VecDeque::with_capacity(256),
            };
            bus.push_all_feedback();
            bus
        }

        fn feedback_frame(&self, idx: usize) -> CanFrame {
            let l = &self.limits[idx];
            let p = damiao::float_to_uint(0.0, l.p_min, l.p_max, 16);
            let v = damiao::float_to_uint(0.0, l.v_min, l.v_max, 12);
            let t = damiao::float_to_uint(0.0, l.t_min, l.t_max, 12);
            let state = u8::from(self.enabled[idx]);
            let id = (idx + 1) as u8;
            let data = [
                ((state & 0x0F) << 4) | (id & 0x0F),
                (p >> 8) as u8,
                p as u8,
                (v >> 4) as u8,
                (((v & 0x0F) << 4) | ((t >> 8) & 0x0F)) as u8,
                t as u8,
                30,
                30,
            ];
            CanFrame::new_standard(0x10 + u16::from(id), &data).unwrap()
        }

        fn push_all_feedback(&mut self) {
            let now = self.clock.now();
            for i in 0..7 {
                self.rx.push_back((self.feedback_frame(i), now));
            }
        }
    }

    impl CanTransport for LockFreeCanBus {
        fn clock(&self) -> Option<Arc<dyn DriverClock>> {
            Some(Arc::clone(&self.clock) as Arc<dyn DriverClock>)
        }
        fn supports_bounded_flush(&self) -> bool {
            true
        }
        fn supports_nonblocking_rt(&self) -> bool {
            true
        }
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
            self.clock.advance(Duration::from_micros(200));
            let now = self.clock.now();
            if frame.id == 0x7FF && frame.dlc == 8 {
                let mid = u16::from_le_bytes([frame.data[0], frame.data[1]]);
                let idx = usize::from(mid.saturating_sub(1));
                if idx < 7 {
                    let op = frame.data[2];
                    let rid = frame.data[3];
                    if op == damiao::REG_READ {
                        let bytes: [u8; 4] = match rid {
                            damiao::RID_CTRL_MODE => self.ctrl_mode[idx].to_le_bytes(),
                            damiao::RID_TIMEOUT => (if idx < 6 { 2000u32 } else { 0u32 }).to_le_bytes(),
                            damiao::RID_PMAX => self.limits[idx].p_max.to_le_bytes(),
                            damiao::RID_VMAX => self.limits[idx].v_max.to_le_bytes(),
                            damiao::RID_TMAX => (if self.enabled[idx] {
                                self.tmax_override[idx].unwrap_or(self.limits[idx].t_max)
                            } else {
                                self.limits[idx].t_max
                            })
                            .to_le_bytes(),
                            _ => 0u32.to_le_bytes(),
                        };
                        let data = [
                            frame.data[0],
                            frame.data[1],
                            damiao::REG_READ,
                            rid,
                            bytes[0],
                            bytes[1],
                            bytes[2],
                            bytes[3],
                        ];
                        self.rx
                            .push_back((CanFrame::new_standard(0x10 + mid, &data).unwrap(), now));
                    } else if op == damiao::REG_WRITE && rid == damiao::RID_CTRL_MODE {
                        let val = u32::from_le_bytes([
                            frame.data[4],
                            frame.data[5],
                            frame.data[6],
                            frame.data[7],
                        ]);
                        self.ctrl_mode[idx] = val;
                        self.rx.push_back((
                            CanFrame::new_standard(0x10 + mid, &frame.data[..8]).unwrap(),
                            now,
                        ));
                    } else if op == 0xCC {
                        self.rx.push_back((self.feedback_frame(idx), now));
                    }
                }
                return Ok(());
            }
            let base = if (0x101..=0x107).contains(&frame.id) {
                Some((frame.id - 0x101) as usize)
            } else if (0x301..=0x307).contains(&frame.id) {
                Some((frame.id - 0x301) as usize)
            } else if (0x001..=0x007).contains(&frame.id) {
                Some((frame.id - 0x001) as usize)
            } else {
                None
            };
            if let Some(idx) = base {
                if frame.data == damiao::CMD_ENABLE {
                    self.enabled[idx] = true;
                } else if frame.data == damiao::CMD_DISABLE {
                    self.enabled[idx] = false;
                }
                self.rx.push_back((self.feedback_frame(idx), now));
            }
            Ok(())
        }
        fn flush(&mut self) -> Result<(), TransportError> {
            Ok(())
        }
        fn receive(&mut self) -> Result<CanFrame, TransportError> {
            self.receive_timestamped().map(|(f, _)| f)
        }
        fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
            match self.rx.pop_front() {
                Some((f, ts)) => Ok((f, Some(ts))),
                None => Err(TransportError::WouldBlock),
            }
        }
    }

    let setup_driver = |j2_bad_tmax: Option<f32>| {
        let clock = Arc::new(MockClock::new());
        let cfg = ArmConfig::rebot_dm();
        let limits_vec: Vec<DamiaoLimits> = cfg
            .damiao_ranges
            .as_ref()
            .unwrap()
            .iter()
            .map(|r| r.limits())
            .collect();
        let limits: [DamiaoLimits; 7] = limits_vec.clone().try_into().unwrap();
        let bus = LockFreeCanBus::new(Arc::clone(&clock), limits, j2_bad_tmax);
        let mut drv = DamiaoArmDriver::new(bus, vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits_vec)
            .unwrap()
            .with_gripper_drive(gripper_drive_from_profile(&cfg).unwrap())
            .unwrap();
        let mut b = TxBudget::default();
        b.max_frames = 16;
        drv.set_tx_budget(b);
        drv.set_rt_mode(true);
        let _ = drv.check_mapping_ranges();
        let _ = drv.read_joint_states();
        drv.set_torque(true).expect("enable torque");
        let _ = drv.read_joint_states();
        (clock, drv)
    };

    thread::spawn(move || {
        // 1. Success: real Damiao ModeSwitch confirm under marked RT release (zero mutexes, zero allocs).
        let (clock, mut drv) = setup_driver(None);
        let plan = ModeSwitchPlan::confirm_gripper(6, ControlMode::PositionForce).unwrap();
        let mut ms = ModeSwitch::new(plan);
        let mut allow_tx = || Ok(());
        let no_stop = || false;
        let (_, delta_ok) = measure_marked_release("mode_switch_confirm_success", || {
            let mut confirmed = false;
            for seq in 1..=15u64 {
                clock.advance(Duration::from_millis(20));
                drv.begin_control_release(seq);
                let now = clock.now();
                drv.set_tx_release_deadline(Some(now + Duration::from_millis(10)));
                let mut ledger = ReleaseBudget::new(seq, now, now + Duration::from_millis(10), 64, 16);
                let b = ModeSwitchBudget::from_ledger(&ledger, now, now + Duration::from_millis(10), Duration::from_millis(20), 16);
                let p = ms.step_with_gate_and_stop_proof_ledger(&mut drv, &mut allow_tx, &no_stop, b, &mut ledger);
                drv.set_tx_release_deadline(None);
                if p == ModeProgress::Confirmed {
                    confirmed = true;
                    break;
                }
            }
            ((), confirmed)
        });
        delta_ok.assert_clean();

        // 2. Recovery: real Damiao ModeSwitch rollback on J2 RangeMismatch after J1 switched (zero mutexes, zero allocs).
        let (clock_rb, mut drv_rb) = setup_driver(Some(99.0));
        drv_rb.set_axis_impedance_ranges_verified(1, false);
        let plan_rb = ModeSwitchPlan::switch_axes(
            &[(0, ControlMode::Impedance), (1, ControlMode::Impedance)],
            ModeSwitchRecovery::RollbackToPrevious,
        )
        .unwrap();
        let mut ms_rb = ModeSwitch::new(plan_rb);
        let (_, delta_rb) = measure_marked_release("mode_switch_rollback_recovery", || {
            let mut rolled_back = false;
            for seq in 1..=40u64 {
                clock_rb.advance(Duration::from_millis(20));
                drv_rb.begin_control_release(seq);
                let now = clock_rb.now();
                drv_rb.set_tx_release_deadline(Some(now + Duration::from_millis(10)));
                let mut ledger = ReleaseBudget::new(seq, now, now + Duration::from_millis(10), 64, 16);
                let b = ModeSwitchBudget::from_ledger(&ledger, now, now + Duration::from_millis(10), Duration::from_millis(20), 16);
                let p = ms_rb.step_with_gate_and_stop_proof_ledger(&mut drv_rb, &mut allow_tx, &no_stop, b, &mut ledger);
                drv_rb.set_tx_release_deadline(None);
                if matches!(
                    p,
                    ModeProgress::RolledBack {
                        failed_joint: 1,
                        reason: armd::motor_mode::ModeSwitchFailure::RangeMismatch,
                    }
                ) {
                    rolled_back = true;
                    break;
                }
            }
            let j1_restored = drv_rb.axis_confirmed_mode(0) == Some(ControlMode::PositionVelocity);
            ((), rolled_back && j1_restored)
        });
        delta_rb.assert_clean();

        // 3. Positive control: injected heap allocation inside measure_marked_release is caught.
        let (_, delta_pos) = measure_marked_release("positive_control_marked_release_alloc", || {
            let b = Box::new([0xAAu8; 64]);
            std::hint::black_box(&b);
            drop(b);
            ((), true)
        });
        assert!(
            delta_pos.allocs.alloc >= 1 && delta_pos.allocs.dealloc >= 1,
            "positive control must detect injected alloc/dealloc: {delta_pos:?}"
        );
    })
    .join()
    .unwrap();
}

#[test]
fn rt_gate_lock_boundary_assertions_reject_before_touching_lock() {
    use armd::rt_core::executor::{ControlThreadGuard, ReleaseGuard};
    use armd::rt_core::publish::state_channel;
    use armd::rt_core::record_ring::RecordRing;
    use armd::rt_core::reply_router::WsWireAckTracker;
    use armd::rt_core::requests::config::{PublishedConfig, PublishedConfigHub};
    use armd::rt_core::requests::lease::conn_lifecycle_channel;
    use armd::rt_core::rt_gate::thread_rt_lock_attempts;
    use armd::rt_core::urgent::urgent_stop_channel;
    use armd::supervisor_lock::{DriverLock, SupervisorLock};

    fn assert_rejected_on_both_marks(label: &str, mut touch: impl FnMut()) {
        // 1. Inside ControlThreadGuard (marked RT control thread).
        let before_t = thread_rt_lock_attempts();
        let res_t = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _tg = ControlThreadGuard::enter();
            touch();
        }));
        assert!(res_t.is_err(), "{label}: must panic on marked RT control thread");
        assert!(
            thread_rt_lock_attempts() > before_t,
            "{label}: must increment thread_rt_lock_attempts() strictly before panicking on RT control thread"
        );

        // 2. Inside ReleaseGuard alone (marked RT release).
        let before_r = thread_rt_lock_attempts();
        let res_r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _rg = ReleaseGuard::enter();
            touch();
        }));
        assert!(res_r.is_err(), "{label}: must panic inside marked RT release");
        assert!(
            thread_rt_lock_attempts() > before_r,
            "{label}: must increment thread_rt_lock_attempts() strictly before panicking inside RT release"
        );
    }

    // Hold StateHub::state and RecordRing::assembly on this thread while spawning a marked RT
    // thread to prove rejection happens strictly BEFORE touching the underlying mutex (no deadlock).
    let (_pub, hub) = state_channel(7);
    let _hub_hold = hub.reader_guard();
    let ring = Arc::new(RecordRing::new_for_test(7));
    let _ring_hold = ring.consumer_guard();
    let (urgent_client, _urgent_intake) = urgent_stop_channel(4, Arc::new(AtomicBool::new(false)));
    let (conn_client, _conn_intake) = conn_lifecycle_channel(Arc::new(AtomicBool::new(false)));
    let cfg_hub = Arc::new(PublishedConfigHub::new(
        PublishedConfig::from_inner(&armd::SupervisorInner::new(ArmConfig::rebot_dm())),
        0,
    ));
    let wire_ack = Arc::new(WsWireAckTracker::new());
    let sup_lock = Arc::new(SupervisorLock::new(0u32));
    let drv_lock = Arc::new(DriverLock::new(
        Box::new(WakeDriver::new(7, Duration::from_millis(1), None).0),
    ));

    let hub_t = Arc::clone(&hub);
    let ring_t = Arc::clone(&ring);
    let cfg_t = Arc::clone(&cfg_hub);
    let wire_t = Arc::clone(&wire_ack);
    let sup_t = Arc::clone(&sup_lock);
    let drv_t = Arc::clone(&drv_lock);

    thread::spawn(move || {
        assert_rejected_on_both_marks("StateHub::state.latest (contended)", || {
            let _ = hub_t.latest();
        });
        assert_rejected_on_both_marks("StateHub::motion_config", || {
            let _ = hub_t.motion_config();
        });
        assert_rejected_on_both_marks("StateHub::service_identity", || {
            let _ = hub_t.service_identity();
        });
        assert_rejected_on_both_marks("RecordRing::assembly.drain (contended)", || {
            let _ = ring_t.drain();
        });
        assert_rejected_on_both_marks("UrgentStopClient::inner", || {
            let _ = urgent_client.in_flight();
        });
        assert_rejected_on_both_marks("ConnLifecycleClient::inner", || {
            let _ = conn_client.live_slot_count();
        });
        assert_rejected_on_both_marks("PublishedConfigHub::slot", || {
            let _ = cfg_t.latest();
        });
        assert_rejected_on_both_marks("WsWireAckTracker::inner", || {
            let _ = wire_t.is_ack_barrier_active();
        });
        assert_rejected_on_both_marks("SupervisorLock::try_read", || {
            let _ = sup_t.try_read();
        });
        assert_rejected_on_both_marks("DriverLock::try_lock", || {
            drop(drv_t.try_lock());
        });
    })
    .join()
    .unwrap();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn rt_gate_tracking_position_lead_and_ramp_out_zero_alloc_zero_lock() {
    use armd::rt_core::requests::motion::{
        prepare_motion, stage_jog, stage_park, MotionConfigSnapshot, MotionPlanSnapshot,
        MotionRequest,
    };
    use armd::rt_core::rt_gate::RtReleaseGate;
    use armd::safety::SafetyState;
    use armd::tracking::{JointTrackingConfig, JointTrackingJoint, TrackingStrategy};

    let mut cfg = ArmConfig::rebot_dm();
    cfg.control_frequency_hz = 50;
    cfg.joint_tracking = Some(JointTrackingConfig {
        enabled: true,
        joints: vec![
            JointTrackingJoint {
                strategy: TrackingStrategy::PositionLead,
                lag_s: 0.06,
                max_lead_rad: 0.03,
                ..Default::default()
            };
            6
        ],
    });
    let dof = cfg.dof;
    let clock = ScriptedClock::new();
    let (drv, _fail_en) = WakeDriver::new(dof, Duration::from_millis(4), Some(Arc::clone(&clock)));
    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;
    sup.register_ws_connection(1).await.unwrap();
    let motion_cfg = {
        let inner = sup.inner().read().await;
        Arc::new(MotionConfigSnapshot::from_supervisor(&inner))
    };

    let period = Duration::from_millis(20);
    let period_ns = period.as_nanos() as u64;
    let handle = sup
        .start_executor_with_hooks(
            alloc_cfg(period),
            PersistTestHooks {
                clock: Some(clock.clone()),
                ..Default::default()
            },
        )
        .await
        .unwrap();

    // Warm up 2 releases so initial snapshot is published.
    clock.run_steps(period_ns, 2);
    wait_until(Duration::from_secs(2), || sup.hub().latest().is_some()).await;

    let gate = RtReleaseGate::new(&sup, &handle);
    let mut client_seq = 0u64;
    let next_auth = || {
        let s = sup.hub().latest().unwrap();
        Authority {
            lease_generation: 0,
            epoch: s.obs.epoch_id,
            stop_generation: handle.stop_requests().current(),
            config_revision: 0,
            parent_generation: 0,
        }
    };

    // 1. Enable torque with position-lead tracking enabled (seeds tracking history zero-alloc).
    let (_, en_delta) = gate.run_phase("tracking_enable_torque", || {
        let auth = next_auth();
        client_seq += 1;
        let req = handle
            .broker()
            .lock()
            .unwrap()
            .submit(
                ClientId(1),
                client_seq,
                auth,
                clock.now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOn),
            )
            .unwrap();
        let mut confirmed = false;
        for _ in 0..20 {
            clock.run_steps(period_ns, 1);
            let rs = drain_broker(&handle);
            if rs.iter().any(|r| {
                matches!(r, Reply::Terminal { request, outcome: Outcome::Completed, .. } if *request == req)
            }) {
                confirmed = true;
                break;
            }
        }
        let s = sup.hub().latest().unwrap();
        ((), confirmed && s.obs.is_torqued)
    });
    assert!(en_delta.is_clean(), "enable with tracking must be zero-alloc/zero-lock: {en_delta:?}");

    // 2. Active position-lead tracking during a jog motion + residual lead ramp-out on settling.
    let (_, track_delta) = gate.run_phase("tracking_position_lead_and_ramp_out", || {
        let snap = sup.hub().latest().unwrap();
        let msnap = MotionPlanSnapshot::from_published(&snap, Arc::clone(&motion_cfg)).unwrap();
        let mut to = snap.rt.commanded_positions.clone();
        to[0] += 0.25;
        let prepared = prepare_motion(
            &msnap,
            &MotionRequest::jog(to.clone(), Some(0.5), None),
        )
        .unwrap();
        let auth = next_auth();
        client_seq += 1;
        let req = handle
            .broker()
            .lock()
            .unwrap()
            .submit(
                ClientId(1),
                client_seq,
                auth,
                clock.now() + Duration::from_secs(5),
                Submit::Install(&|s: &mut PlanSlot| stage_jog(s, &prepared.slot)),
            )
            .unwrap();
        let mut completed = false;
        let mut saw_positive_lead = false;
        for _ in 0..120 {
            clock.run_steps(period_ns, 1);
            if let Some(s) = sup.hub().latest() {
                // WakeDriver sets measured q[0] = written target (nominal + lead); while moving
                // positively with PositionLead, written target exceeds nominal commanded_positions[0].
                if s.rt.positions[0] > s.rt.commanded_positions[0] + 1e-4 {
                    saw_positive_lead = true;
                }
            }
            let rs = drain_broker(&handle);
            if rs.iter().any(|r| {
                matches!(r, Reply::Terminal { request, outcome: Outcome::Completed, .. } if *request == req)
            }) {
                completed = true;
                break;
            }
        }
        // Run additional releases at rest so any residual lead ramps out cleanly to nominal.
        clock.run_steps(period_ns, 10);
        let s_after = sup.hub().latest().unwrap();
        let reached = (s_after.rt.commanded_positions[0] - to[0]).abs() < 1e-3
            && (s_after.rt.positions[0] - to[0]).abs() < 1e-3;
        ((), completed && saw_positive_lead && reached)
    });
    assert!(
        track_delta.is_clean(),
        "tracking-active position-lead and lead ramp-out must be zero-alloc/zero-lock: {track_delta:?}"
    );

    // 3. Park with tracking active (exercises keepout_only ramp-out + park motion + disable).
    let (_, park_delta) = gate.run_phase("tracking_park_ramp_out_and_disable", || {
        let snap = sup.hub().latest().unwrap();
        let msnap = MotionPlanSnapshot::from_published(&snap, Arc::clone(&motion_cfg)).unwrap();
        let park_plan = prepare_motion(&msnap, &MotionRequest::park()).unwrap();
        let auth = next_auth();
        client_seq += 1;
        let r_park = handle
            .broker()
            .lock()
            .unwrap()
            .submit(
                ClientId(1),
                client_seq,
                auth,
                clock.now() + Duration::from_secs(10),
                Submit::InstallPark(&|s: &mut PlanSlot| stage_park(s, &park_plan.slot)),
            )
            .unwrap();
        let mut park_done = false;
        for _ in 0..320 {
            clock.run_steps(period_ns, 2);
            let rs = drain_broker(&handle);
            if rs.iter().any(|r| {
                matches!(r, Reply::Terminal { request, outcome: Outcome::Completed, .. } if *request == r_park)
            }) {
                park_done = true;
                break;
            }
        }
        let s_after = sup.hub().latest().unwrap();
        let parked_ok =
            park_done && !s_after.obs.is_torqued && s_after.obs.safety_state == SafetyState::Idle;
        ((), parked_ok)
    });
    assert!(
        park_delta.is_clean(),
        "park with tracking active must be zero-alloc/zero-lock: {park_delta:?}"
    );

    tokio::task::spawn_blocking(move || handle.stop_and_join())
        .await
        .unwrap();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn rt_gate_counts_enabled_post_publish_delay_once_hook_lock() {
    use armd::rt_core::rt_gate::RtReleaseGate;
    let clock = ScriptedClock::new();
    let (drv, _fail) = WakeDriver::new(7, Duration::from_millis(2), Some(clock.clone()));
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;
    let period_ns = 10_000_000u64;
    let delay_hook = Arc::new(Mutex::new(Some(Duration::from_micros(50))));
    let handle = sup
        .start_executor_with_hooks(
            alloc_cfg(Duration::from_nanos(period_ns)),
            PersistTestHooks {
                clock: Some(clock.clone()),
                post_publish_delay_once: Some(Arc::clone(&delay_hook)),
                ..Default::default()
            },
        )
        .await
        .unwrap();

    let gate = RtReleaseGate::new(&sup, &handle);
    let guard = gate.begin_phase("post_publish_delay_once_enabled");
    clock.run_steps(period_ns, 2);
    let delta = guard.finish();
    assert!(
        !delta.is_clean(),
        "enabled post_publish_delay_once test-hook mutex must be caught by RT gate"
    );
    assert!(
        delta.rt_lock_attempts >= 2,
        "expected >= 2 recorded RT lock attempts across 2 releases with enabled hook, got {}",
        delta.rt_lock_attempts
    );
    assert!(
        handle.stats().rt_lock_attempts.load(Ordering::Acquire) >= 2,
        "executor stats must record RT lock attempts when post_publish_delay_once is enabled"
    );

    tokio::task::spawn_blocking(move || handle.stop_and_join())
        .await
        .unwrap();
}
