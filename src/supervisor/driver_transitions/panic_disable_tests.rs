//! Unit tests for bounded panic disable and legacy torque timeout semantics.

use super::*;
use crate::driver::{ArmDriverError, TorqueOutcome, TorqueProgress};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Condvar, Mutex};

impl ArmDriver for SelectiveMockDriver {
    crate::ext_accessors!(bus_recovery);
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            enable_feedback: true,
            ..DriverCapabilities::default()
        }
    }
    fn is_enabled(&self) -> bool { true }
    fn enable_state(&self) -> EnableState { EnableState::Unknown }
    fn is_feedback_valid(&self) -> bool { false }
    fn read_joint_states(&mut self) -> Result<Vec<crate::model::JointState>, ArmDriverError> {
        Ok(Vec::new())
    }
    fn write_joint_targets(&mut self, _: &[crate::driver::JointTarget]) -> Result<(), ArmDriverError> {
        Ok(())
    }
    fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
    fn begin_torque_budgeted(
        &mut self,
        _: bool,
        _: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.generic_begin_calls.fetch_add(1, Ordering::SeqCst);
        Ok(TorqueProgress::Done(TorqueOutcome::Unconfirmed))
    }
    fn unconfirmed_disable_joints(&self) -> Vec<usize> { vec![2] }
}

impl crate::driver::BusRecoveryExt for SelectiveMockDriver {
    fn panic_disable_step(
        &mut self,
        _first: bool,
        _deadline: Instant,
    ) -> Option<Result<Vec<usize>, ArmDriverError>> {
        self.selective_calls.fetch_add(1, Ordering::SeqCst);
        if !self.sleep_on_step.is_zero() {
            std::thread::sleep(self.sleep_on_step);
        }
        Some(match &self.step_result {
            Ok(v) => Ok(v.clone()),
            Err(e) => Err(ArmDriverError::HardwareFault(e.to_string())),
        })
    }
}

impl ArmDriver for GenericMockDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            enable_feedback: true,
            ..DriverCapabilities::default()
        }
    }
    fn is_enabled(&self) -> bool { true }
    fn enable_state(&self) -> EnableState { EnableState::Unknown }
    fn is_feedback_valid(&self) -> bool { false }
    fn read_joint_states(&mut self) -> Result<Vec<crate::model::JointState>, ArmDriverError> {
        Ok(Vec::new())
    }
    fn read_joint_states_budgeted(
        &mut self,
        out: &mut Vec<crate::model::JointState>,
        _deadline: Instant,
        _max_rx_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        out.clear();
        Ok((0, false))
    }
    fn write_joint_targets(&mut self, _: &[crate::driver::JointTarget]) -> Result<(), ArmDriverError> {
        Ok(())
    }
    fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
    fn begin_torque_budgeted(
        &mut self,
        _: bool,
        _: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.generic_begin_calls.fetch_add(1, Ordering::SeqCst);
        if !self.sleep_on_begin.is_zero() {
            std::thread::sleep(self.sleep_on_begin);
        }
        Ok(TorqueProgress::Done(TorqueOutcome::Unconfirmed))
    }
    fn unconfirmed_disable_joints(&self) -> Vec<usize> {
        vec![0]
    }
}

impl ArmDriver for ProbeThenGenericMockDriver {
    crate::ext_accessors!(bus_recovery);
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            enable_feedback: true,
            ..DriverCapabilities::default()
        }
    }
    fn is_enabled(&self) -> bool { true }
    fn enable_state(&self) -> EnableState { EnableState::Unknown }
    fn is_feedback_valid(&self) -> bool { false }
    fn read_joint_states(&mut self) -> Result<Vec<crate::model::JointState>, ArmDriverError> { Ok(Vec::new()) }
    fn write_joint_targets(&mut self, _: &[crate::driver::JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
    fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
    fn begin_torque_budgeted(
        &mut self,
        _: bool,
        _: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.generic_begin_calls.fetch_add(1, Ordering::SeqCst);
        Ok(TorqueProgress::Done(TorqueOutcome::Unconfirmed))
    }
}

impl crate::driver::BusRecoveryExt for ProbeThenGenericMockDriver {
    fn panic_disable_step(
        &mut self,
        _first: bool,
        _deadline: Instant,
    ) -> Option<Result<Vec<usize>, ArmDriverError>> {
        self.selective_calls.fetch_add(1, Ordering::SeqCst);
        if !self.sleep_on_probe.is_zero() {
            std::thread::sleep(self.sleep_on_probe);
        }
        None
    }
}

struct SelectiveMockDriver {
    selective_calls: Arc<AtomicUsize>,
    generic_begin_calls: Arc<AtomicUsize>,
    sleep_on_step: Duration,
    step_result: Result<Vec<usize>, ArmDriverError>,
}

struct GenericMockDriver {
    generic_begin_calls: Arc<AtomicUsize>,
    sleep_on_begin: Duration,
}

struct ProbeThenGenericMockDriver {
    selective_calls: Arc<AtomicUsize>,
    generic_begin_calls: Arc<AtomicUsize>,
    sleep_on_probe: Duration,
}

#[test]
fn panic_disable_deadlines_reserves_tail_within_hard_bound() {
    let origin = Instant::now();
    assert_eq!(
        panic_disable_deadlines(origin, Duration::from_millis(500)),
        (origin + Duration::from_millis(475), origin + Duration::from_millis(500))
    );
    assert_eq!(
        panic_disable_deadlines(origin, Duration::from_millis(80)),
        (origin + Duration::from_millis(72), origin + Duration::from_millis(80))
    );
    assert_eq!(
        panic_disable_deadlines(origin, Duration::from_millis(60)),
        (origin + Duration::from_millis(54), origin + Duration::from_millis(60))
    );
    assert_eq!(
        panic_disable_deadlines(origin, Duration::ZERO),
        (origin, origin)
    );
}

#[test]
fn bounded_panic_disable_driver_no_extra_step_after_deadline_and_no_generic_fallthrough() {
    let selective_calls = Arc::new(AtomicUsize::new(0));
    let generic_begin_calls = Arc::new(AtomicUsize::new(0));
    let mut drv = SelectiveMockDriver {
        selective_calls: Arc::clone(&selective_calls),
        generic_begin_calls: Arc::clone(&generic_begin_calls),
        sleep_on_step: Duration::from_millis(25),
        step_result: Ok(vec![2]),
    };
    let origin = Instant::now();
    let coop_budget = Duration::from_millis(15);
    let coop_deadline = origin + coop_budget;
    let hard_deadline = origin + Duration::from_millis(40);
    let outcome = bounded_panic_disable_driver_until(
        &mut drv,
        coop_deadline,
        hard_deadline,
        coop_budget,
    );
    assert_eq!(outcome, PanicDisableOutcome::Unconfirmed);
    assert_eq!(selective_calls.load(Ordering::SeqCst), 1);
    assert_eq!(generic_begin_calls.load(Ordering::SeqCst), 0);

    // Top-of-loop sleep wakeup case: step 1 completes immediately, sleep wakes after
    // `coop_budget` (1ms), and top-of-loop check exits without calling step 2.
    let selective_calls_wake = Arc::new(AtomicUsize::new(0));
    let generic_begin_calls_wake = Arc::new(AtomicUsize::new(0));
    let mut drv_wake = SelectiveMockDriver {
        selective_calls: Arc::clone(&selective_calls_wake),
        generic_begin_calls: Arc::clone(&generic_begin_calls_wake),
        sleep_on_step: Duration::ZERO,
        step_result: Ok(vec![2]),
    };
    let origin_wake = Instant::now();
    let coop_budget_wake = Duration::from_millis(1);
    let outcome_wake = bounded_panic_disable_driver_until(
        &mut drv_wake,
        origin_wake + coop_budget_wake,
        origin_wake + Duration::from_millis(50),
        coop_budget_wake,
    );
    assert_eq!(outcome_wake, PanicDisableOutcome::Unconfirmed);
    assert_eq!(selective_calls_wake.load(Ordering::SeqCst), 1);
    assert_eq!(generic_begin_calls_wake.load(Ordering::SeqCst), 0);
}

#[test]
fn bounded_panic_disable_driver_expired_at_entry_and_generic_probe_expiry() {
    // Expired at entry: zero selective or generic calls, returns TimedOut.
    let selective_calls = Arc::new(AtomicUsize::new(0));
    let generic_begin_calls = Arc::new(AtomicUsize::new(0));
    let mut drv = SelectiveMockDriver {
        selective_calls: Arc::clone(&selective_calls),
        generic_begin_calls: Arc::clone(&generic_begin_calls),
        sleep_on_step: Duration::ZERO,
        step_result: Ok(vec![2]),
    };
    let expired = Instant::now();
    let outcome = bounded_panic_disable_driver_until(&mut drv, expired, expired, Duration::ZERO);
    assert_eq!(outcome, PanicDisableOutcome::TimedOut);
    assert_eq!(selective_calls.load(Ordering::SeqCst), 0);
    assert_eq!(generic_begin_calls.load(Ordering::SeqCst), 0);

    // Selective probe returns `None` after sleeping past `hard_deadline`: generic fallback
    // must see `now >= hard_deadline` and return `TimedOut` with zero `begin_torque_budgeted` calls.
    let probe_calls = Arc::new(AtomicUsize::new(0));
    let generic_calls = Arc::new(AtomicUsize::new(0));
    let mut drv_probe = ProbeThenGenericMockDriver {
        selective_calls: Arc::clone(&probe_calls),
        generic_begin_calls: Arc::clone(&generic_calls),
        sleep_on_probe: Duration::from_millis(25),
    };
    let origin = Instant::now();
    let outcome_probe = bounded_panic_disable_driver_until(
        &mut drv_probe,
        origin + Duration::from_millis(10),
        origin + Duration::from_millis(15),
        Duration::from_millis(10),
    );
    assert_eq!(outcome_probe, PanicDisableOutcome::TimedOut);
    assert_eq!(probe_calls.load(Ordering::SeqCst), 1);
    assert_eq!(generic_calls.load(Ordering::SeqCst), 0);
}

#[test]
fn bounded_panic_disable_driver_generic_no_extra_attempt_after_deadline() {
    let generic_begin_calls = Arc::new(AtomicUsize::new(0));
    let mut drv = GenericMockDriver {
        generic_begin_calls: Arc::clone(&generic_begin_calls),
        sleep_on_begin: Duration::from_millis(25),
    };
    let origin = Instant::now();
    let coop_budget = Duration::from_millis(15);
    let coop_deadline = origin + coop_budget;
    let hard_deadline = origin + Duration::from_millis(40);
    let outcome = bounded_panic_disable_driver_until(
        &mut drv,
        coop_deadline,
        hard_deadline,
        coop_budget,
    );
    assert_eq!(outcome, PanicDisableOutcome::Unconfirmed);
    assert_eq!(generic_begin_calls.load(Ordering::SeqCst), 1);
}

#[test]
fn attempt_panic_disable_until_rejects_new_hardware_tx_after_hard_deadline() {
    let selective_calls = Arc::new(AtomicUsize::new(0));
    let generic_begin_calls = Arc::new(AtomicUsize::new(0));
    let drv = SelectiveMockDriver {
        selective_calls: Arc::clone(&selective_calls),
        generic_begin_calls: Arc::clone(&generic_begin_calls),
        sleep_on_step: Duration::ZERO,
        step_result: Ok(vec![2]),
    };
    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv));
    let expired = Instant::now();
    let outcome = inner.attempt_panic_disable_until(expired, expired, Duration::ZERO);
    assert_eq!(outcome, PanicDisableOutcome::TimedOut);
    assert_eq!(selective_calls.load(Ordering::SeqCst), 0);
    assert_eq!(generic_begin_calls.load(Ordering::SeqCst), 0);

    // Contended driver mutex held across `hard_deadline` returns TimedOut with 0 hardware TX.
    let drv_held = SelectiveMockDriver {
        selective_calls: Arc::clone(&selective_calls),
        generic_begin_calls: Arc::clone(&generic_begin_calls),
        sleep_on_step: Duration::ZERO,
        step_result: Ok(vec![2]),
    };
    let mut inner_held = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_held));
    // Safe Rust cannot hold a borrow of `inner_held.driver` across `&mut inner_held`;
    // leaking the guard keeps `try_lock()` contended until `hard_deadline` expires.
    let leaked_guard = inner_held
        .driver
        .as_ref()
        .unwrap()
        .as_legacy_lock()
        .unwrap()
        .lock()
        .unwrap();
    std::mem::forget(leaked_guard);
    let origin_held = Instant::now();
    let outcome_held = inner_held.attempt_panic_disable_until(
        origin_held + Duration::from_millis(5),
        origin_held + Duration::from_millis(10),
        Duration::from_millis(5),
    );
    assert_eq!(outcome_held, PanicDisableOutcome::TimedOut);
    assert_eq!(selective_calls.load(Ordering::SeqCst), 0);
    assert_eq!(generic_begin_calls.load(Ordering::SeqCst), 0);

    // Legacy lock contention clearing after `hard_deadline`: `inner` write lock held past
    // `timeout` and then released -> `attempt_legacy_panic_disable` returns `TimedOut`
    // with 0 hardware TX.
    let sup_calls = Arc::new(AtomicUsize::new(0));
    let sup_generic = Arc::new(AtomicUsize::new(0));
    let drv_sup = SelectiveMockDriver {
        selective_calls: Arc::clone(&sup_calls),
        generic_begin_calls: Arc::clone(&sup_generic),
        sleep_on_step: Duration::ZERO,
        step_result: Ok(vec![2]),
    };
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_sup)));
    let inner_arc = Arc::clone(sup.inner());
    let (locked_tx, locked_rx) = std::sync::mpsc::channel();
    let (rel_tx, rel_rx) = std::sync::mpsc::channel();
    let holder = std::thread::spawn(move || {
        let _guard = inner_arc.blocking_write();
        let _ = locked_tx.send(());
        let _ = rel_rx.recv();
    });
    locked_rx.recv().unwrap();
    sup.authorize_process_exit_for_test();
    let legacy_outcome = sup.attempt_legacy_panic_disable(Duration::from_millis(12));
    rel_tx.send(()).unwrap();
    holder.join().unwrap();
    assert_eq!(legacy_outcome, PanicDisableOutcome::TimedOut);
    assert_eq!(sup_calls.load(Ordering::SeqCst), 0);
    assert_eq!(sup_generic.load(Ordering::SeqCst), 0);
}

#[test]
fn run_with_panic_deadline_observer_until_completes_before_hard_deadline_and_fires_when_blocked() {
    let complete_a = Arc::new(AtomicBool::new(false));
    let complete_a_cb = Arc::clone(&complete_a);
    let timeout_a = Arc::new(AtomicBool::new(false));
    let timeout_a_cb = Arc::clone(&timeout_a);
    let res_a = run_with_panic_deadline_observer_until(
        Instant::now() + Duration::from_millis(100),
        || PanicDisableOutcome::Unconfirmed,
        |outcome| {
            assert_eq!(*outcome, PanicDisableOutcome::Unconfirmed);
            complete_a_cb.store(true, Ordering::SeqCst);
            true
        },
        move || {
            timeout_a_cb.store(true, Ordering::SeqCst);
        },
    );
    assert_eq!(res_a, Some(PanicDisableOutcome::Unconfirmed));
    assert!(complete_a.load(Ordering::SeqCst));
    assert!(!timeout_a.load(Ordering::SeqCst));

    let gate = Arc::new((Mutex::new(false), Condvar::new()));
    let gate_cb = Arc::clone(&gate);
    let timeout_b = Arc::new(AtomicBool::new(false));
    let timeout_b_cb = Arc::clone(&timeout_b);
    let complete_b = Arc::new(AtomicBool::new(false));
    let complete_b_cb = Arc::clone(&complete_b);
    let res_b = run_with_panic_deadline_observer_until(
        Instant::now() + Duration::from_millis(15),
        || {
            let mut done = gate.0.lock().unwrap();
            while !*done {
                done = gate.1.wait(done).unwrap();
            }
            PanicDisableOutcome::Unconfirmed
        },
        |_| {
            complete_b_cb.store(true, Ordering::SeqCst);
            true
        },
        move || {
            timeout_b_cb.store(true, Ordering::SeqCst);
            let mut done = gate_cb.0.lock().unwrap();
            *done = true;
            gate_cb.1.notify_all();
        },
    );
    assert_eq!(res_b, None);
    assert!(!complete_b.load(Ordering::SeqCst));
    assert!(timeout_b.load(Ordering::SeqCst));

    // Subcase (c): work completes slightly past `hard_deadline` (racing observer wake):
    // single-winner arbitration must return `None`, NOT fire `on_complete`, and fire `on_timeout` once.
    let complete_c = Arc::new(AtomicBool::new(false));
    let complete_c_cb = Arc::clone(&complete_c);
    let timeout_c_count = Arc::new(AtomicUsize::new(0));
    let timeout_c_cb = Arc::clone(&timeout_c_count);
    let res_c = run_with_panic_deadline_observer_until(
        Instant::now(),
        || {
            std::thread::sleep(Duration::from_millis(2));
            PanicDisableOutcome::Unconfirmed
        },
        |_| {
            complete_c_cb.store(true, Ordering::SeqCst);
            true
        },
        move || {
            timeout_c_cb.fetch_add(1, Ordering::SeqCst);
        },
    );
    assert_eq!(res_c, None);
    assert!(!complete_c.load(Ordering::SeqCst));
    assert_eq!(timeout_c_count.load(Ordering::SeqCst), 1);

    // Subcase (d): `on_timeout` blocks on a barrier while `work()` unblocks and returns;
    // `run_with_panic_deadline_observer_until` must NOT return `None` until `on_timeout` finishes.
    let (started_tx, started_rx) = std::sync::mpsc::channel();
    let (work_done_tx, work_done_rx) = std::sync::mpsc::channel();
    let (release_tx, release_rx) = std::sync::mpsc::channel();
    let timeout_finished = Arc::new(AtomicBool::new(false));
    let timeout_finished_cb = Arc::clone(&timeout_finished);
    let runner_returned = Arc::new(AtomicBool::new(false));
    let runner_returned_th = Arc::clone(&runner_returned);
    let runner = std::thread::spawn(move || {
        let res = run_with_panic_deadline_observer_until(
            Instant::now() + Duration::from_millis(5),
            || {
                started_rx.recv().unwrap();
                let _ = work_done_tx.send(());
                PanicDisableOutcome::Unconfirmed
            },
            |_| true,
            move || {
                let _ = started_tx.send(());
                release_rx.recv().unwrap();
                timeout_finished_cb.store(true, Ordering::SeqCst);
            },
        );
        runner_returned_th.store(true, Ordering::SeqCst);
        res
    });
    work_done_rx.recv().unwrap();
    std::thread::sleep(Duration::from_millis(5));
    assert!(
        !runner_returned.load(Ordering::SeqCst),
        "observer_until must wait for in-flight on_timeout before returning None"
    );
    release_tx.send(()).unwrap();
    let res_d = runner.join().unwrap();
    assert_eq!(res_d, None);
    assert!(timeout_finished.load(Ordering::SeqCst));
}

#[test]
fn legacy_timeout_aborts_enable_and_detaches_dispatched_disable_without_aborting_protective() {
    use crate::driver::{ArmDriver, ArmDriverError, DisableKind, DisableProtectiveReason, DriverCapabilities, EnableState, JointTarget, TorqueOutcome, TorqueProgress};
    use crate::model::JointState;

    struct StubPendingDriver {
        dof: usize,
        pending: bool,
        dispatched_mask: u64,
        dispatch_mask_on_begin: u64,
        protective: bool,
        abort_enable_calls: usize,
        abort_disable_calls: usize,
        complete_disable_on_poll: Arc<AtomicBool>,
        enabled: bool,
    }

    impl ArmDriver for StubPendingDriver {
        fn capabilities(&self) -> DriverCapabilities {
            DriverCapabilities {
                velocity_feedback: true,
                torque_feedback: true,
                temperature_feedback: true,
                enable_feedback: true,
                ..DriverCapabilities::default()
            }
        }
        fn supports_rt_executor(&self) -> Result<(), &'static str> { Ok(()) }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            Ok(vec![JointState::default(); self.dof])
        }
        fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
            out.clear();
            out.resize(self.dof, JointState::default());
            Some(())
        }
        fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
        fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
            self.enabled = enabled;
            Ok(())
        }
        fn is_feedback_valid(&self) -> bool { true }
        fn feedback_all_within(&self, _dof: usize, _max_age: Duration) -> bool { true }
        fn feedback_ages(&self) -> Vec<Option<Duration>> {
            vec![Some(Duration::ZERO); self.dof]
        }
        fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
            out.clear();
            out.resize(self.dof, Some(Duration::ZERO));
        }
        fn has_driver_fresh_position_api(&self) -> bool { true }
        fn fresh_measured_position_for(&self, _joint: usize, _max_age: Duration) -> Option<f32> {
            Some(0.0)
        }
        fn fresh_joint_positions_into(
            &self,
            joints: &[usize],
            _max_age: Duration,
            out: &mut [f32],
        ) -> bool {
            for slot in out.iter_mut().take(joints.len()) {
                *slot = 0.0;
            }
            true
        }
        fn rest_pose_gate_status(&self) -> crate::driver::RestPoseGateStatus {
            crate::driver::RestPoseGateStatus::Confirmed
        }
        fn enable_state(&self) -> EnableState {
            if self.enabled { EnableState::Enabled } else { EnableState::Disabled }
        }
        fn begin_torque_budgeted(&mut self, _enable: bool, _max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
            self.pending = true;
            Ok(TorqueProgress::Pending { wake: Instant::now() + Duration::from_millis(5) })
        }
        fn begin_torque_budgeted_with_range_retry(&mut self, enable: bool, max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
            self.begin_torque_budgeted(enable, max_rx_frames)
        }
        fn begin_disable_torque_budgeted(&mut self, kind: DisableKind, _max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
            self.pending = true;
            self.protective = matches!(kind, DisableKind::DisableProtective { .. });
            self.dispatched_mask = self.dispatch_mask_on_begin;
            Ok(TorqueProgress::Pending { wake: Instant::now() + Duration::from_millis(5) })
        }
        fn poll_torque_budgeted(&mut self, _deadline: Instant, _max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
            if !self.pending {
                return Err(ArmDriverError::ProtocolStatic("no torque transition pending"));
            }
            if self.complete_disable_on_poll.load(Ordering::Acquire) {
                self.pending = false;
                self.enabled = false;
                self.dispatched_mask = 0;
                return Ok(TorqueProgress::Done(TorqueOutcome::Confirmed));
            }
            Ok(TorqueProgress::Pending { wake: Instant::now() + Duration::from_millis(5) })
        }
        fn torque_transition_pending(&self) -> bool { self.pending }
        fn disable_frames_dispatched(&self) -> bool { self.dispatched_mask != 0 }
        fn disable_dispatched_mask(&self) -> u64 { self.dispatched_mask }
        fn abort_enable(&mut self) {
            self.abort_enable_calls += 1;
            self.pending = false;
        }
        fn abort_pending_disable(&mut self) -> bool {
            self.abort_disable_calls += 1;
            if self.protective || self.dispatched_mask != 0 {
                return false;
            }
            self.pending = false;
            true
        }
    }

    // 1. Stalled-ticker enable timeout aborts driver enable (clearing driver pending state),
    //    clears supervisor transition, and settles slot.
    {
        let mut inner = SupervisorInner::new(crate::ArmConfig::rebot_dm()).with_driver(Box::new(StubPendingDriver {
            dof: 7,
            pending: false,
            dispatched_mask: 0,
            dispatch_mask_on_begin: 0,
            protective: false,
            abort_enable_calls: 0,
            abort_disable_calls: 0,
            complete_disable_on_poll: Arc::new(AtomicBool::new(false)),
            enabled: false,
        }));
        inner.lease_enabled = false;
        inner.tick_control(0.02);
        let stamp = inner.current_admission_stamp();
        let start = inner
            .check_admission(stamp)
            .and_then(|a| {
                SupervisorInner::begin_enable_torque_transition_legacy(
                    a,
                    Instant::now() + Duration::from_secs(5),
                )
            })
            .expect("start enable");
        let (token, slot_idx, slot) = match start {
            LegacyTorqueStart::Pending { token, slot_idx, slot } => (token, slot_idx, slot),
            _ => panic!("expected Pending enable"),
        };
        assert!(inner.torque_transition.is_some());
        assert!(inner.driver.as_ref().unwrap().read().unwrap().torque_transition_pending());
        let out = inner.timeout_legacy_torque_transition(token, slot_idx, true);
        assert!(matches!(out, LegacyTorqueOutcome::Failed { .. }), "{out:?}");
        assert!(inner.torque_transition.is_none(), "timed-out enable must not leave orphan transition");
        assert!(
            !inner.driver.as_ref().unwrap().read().unwrap().torque_transition_pending(),
            "driver handshake must actually be cleared on enable timeout"
        );
        assert!(slot.take_settled(token).is_none(), "slot already consumed/released");
    }

    // 2. Dispatched disable timeout does NOT abort driver disable; retags to Detached and
    //    completes to `is_torqued == false` once remaining motors confirm.
    {
        let complete_flag = Arc::new(AtomicBool::new(false));
        let mut inner = SupervisorInner::new(crate::ArmConfig::rebot_dm()).with_driver(Box::new(StubPendingDriver {
            dof: 7,
            pending: false,
            dispatched_mask: 0,
            dispatch_mask_on_begin: 0b0000_0111, // 3 joints dispatched during begin_disable
            protective: false,
            abort_enable_calls: 0,
            abort_disable_calls: 0,
            complete_disable_on_poll: Arc::clone(&complete_flag),
            enabled: true,
        }));
        inner.lease_enabled = false;
        inner.is_torqued = true;
        inner.tick_control(0.02);
        let start = inner
            .begin_disable_torque_transition_legacy(Instant::now() + Duration::from_secs(5))
            .expect("start disable");
        let (token, slot_idx, _slot) = match start {
            LegacyTorqueStart::Pending { token, slot_idx, slot } => (token, slot_idx, slot),
            _ => panic!("expected Pending disable"),
        };
        let out = inner.timeout_legacy_torque_transition(token, slot_idx, false);
        assert!(matches!(
            out,
            LegacyTorqueOutcome::Failed {
                failure: LegacyTorqueFailure::StaticCode("DEADLINE_EXPIRED"),
                ..
            }
        ));
        assert!(
            matches!(
                inner.torque_transition,
                Some(TorqueTransition::Disable {
                    origin: DisableOrigin::Detached { .. },
                    ..
                })
            ),
            "dispatched disable must remain active as DisableOrigin::Detached, got {:?}",
            inner.torque_transition
        );
        // Flip stub to confirm remaining joints and verify Detached disable completes cleanly.
        // Use a generous wall-clock budget so CPU contention under parallel test runs cannot
        // expire `budget.deadline` before `StubPendingDriver::poll_torque_budgeted` runs.
        complete_flag.store(true, Ordering::Release);
        let now = Instant::now();
        let budget = crate::rt_core::ops::ServiceBudget {
            now,
            deadline: now + Duration::from_secs(1),
            max_rx_frames: 64,
        };
        let poll = inner
            .service_torque_transition(&budget)
            .expect("expected Detached disable poll result");
        assert!(
            matches!(poll.progress, Ok(crate::rt_core::ops::TorqueProgress::Done)),
            "Detached disable must complete cleanly: {:?}",
            poll.progress
        );
        assert!(inner.torque_transition.is_none());
        assert!(!inner.is_torqued);
    }

    // 3. Protective disable is NEVER aborted by `abort_pending_disable`.
    {
        let mut drv = StubPendingDriver {
            dof: 7,
            pending: false,
            dispatched_mask: 0,
            dispatch_mask_on_begin: 0,
            protective: false,
            abort_enable_calls: 0,
            abort_disable_calls: 0,
            complete_disable_on_poll: Arc::new(AtomicBool::new(false)),
            enabled: true,
        };
        let _ = drv.begin_disable_torque_budgeted(
            DisableKind::DisableProtective {
                reason: DisableProtectiveReason::Thermal,
            },
            0,
        );
        assert!(
            !drv.abort_pending_disable(),
            "protective disable must NEVER be aborted"
        );
        assert!(drv.torque_transition_pending(), "protective disable must remain live");
    }
}

