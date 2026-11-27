//! Mixed driver TX choke-point, rest-pose gating, and J7 emitter inline unit tests.

use super::*;
use crate::rt_core::urgent::{EstopGate, StopRequests};
use crate::transport::{CanFrame, CanTransport, DriverClock, MockClock, TransportError};
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU64};
use std::sync::{Arc, Mutex};

#[derive(Clone)]
struct TestBus {
    clock: Arc<MockClock>,
    tx: Arc<Mutex<Vec<CanFrame>>>,
    rx: Arc<Mutex<VecDeque<(CanFrame, Option<Instant>)>>>,
    on_tx: Arc<Mutex<Option<Box<dyn FnMut(&CanFrame) -> Result<(), TransportError> + Send>>>>,
}

impl TestBus {
    fn new() -> (Self, Arc<MockClock>) {
        let clock = Arc::new(MockClock::new());
        let bus = Self {
            clock: clock.clone(),
            tx: Arc::new(Mutex::new(Vec::new())),
            rx: Arc::new(Mutex::new(VecDeque::new())),
            on_tx: Arc::new(Mutex::new(None)),
        };
        (bus, clock)
    }

    fn tx_len(&self) -> usize {
        self.tx.lock().unwrap().len()
    }

    fn tx_frames(&self) -> Vec<CanFrame> {
        self.tx.lock().unwrap().clone()
    }

    fn clear_tx(&self) {
        self.tx.lock().unwrap().clear();
    }

    fn set_on_tx(&self, cb: impl FnMut(&CanFrame) -> Result<(), TransportError> + Send + 'static) {
        *self.on_tx.lock().unwrap() = Some(Box::new(cb));
    }

    fn push_feedback(&self, motor_id: u16, pos: f32, state: u8, limits: &DamiaoLimits) {
        let p = damiao::float_to_uint(pos, limits.p_min, limits.p_max, 16);
        let v = damiao::float_to_uint(0.0, limits.v_min, limits.v_max, 12);
        let t = damiao::float_to_uint(0.0, limits.t_min, limits.t_max, 12);
        let data = [
            ((state & 0x0f) << 4) | (motor_id as u8 & 0x0f),
            (p >> 8) as u8,
            p as u8,
            (v >> 4) as u8,
            (((v & 0x0f) << 4) | ((t >> 8) & 0x0f)) as u8,
            t as u8,
            30,
            30,
        ];
        let ts = self.clock.now();
        self.rx
            .lock()
            .unwrap()
            .push_back((CanFrame::new_standard(0x10 + motor_id, &data).unwrap(), Some(ts)));
    }
}

impl CanTransport for TestBus {
    fn clock(&self) -> Option<Arc<dyn DriverClock>> {
        Some(self.clock.clone())
    }
    fn supports_nonblocking_rt(&self) -> bool { true }
    fn supports_bounded_flush(&self) -> bool { true }
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        if let Some(cb) = self.on_tx.lock().unwrap().as_mut() {
            cb(frame)?;
        }
        self.tx.lock().unwrap().push(*frame);
        Ok(())
    }
    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        self.receive_timestamped().map(|(f, _)| f)
    }
    fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
        self.rx.lock().unwrap().pop_front().ok_or(TransportError::WouldBlock)
    }
    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }
}

fn all_tx_kinds(started_at: Instant) -> [TxKind; 14] {
    [
        TxKind::DisableRequested { started_at },
        TxKind::MitRecoveryDisable { started_at },
        TxKind::ClientParkDisable { started_at },
        TxKind::DisableProtective { reason: DisableProtectiveReason::Thermal },
        TxKind::DisableProtective { reason: DisableProtectiveReason::ProcessExit },
        TxKind::DisableProbe,
        TxKind::Target,
        TxKind::Hold,
        TxKind::RollbackHold(0),
        TxKind::ModeWrite,
        TxKind::Refresh,
        TxKind::Recovery,
        TxKind::ClearFault,
        TxKind::Handshake,
    ]
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum StopCase {
    NoStop,
    Pending,
    Latched,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum BudgetCase {
    Available,
    Exhausted,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DeadlineCase {
    Ok,
    Passed,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Expected {
    Sent,
    Deferred,
    RefusedDeadline,
    RefusedStop,
}

fn expected_outcome(
    kind: TxKind,
    stop: StopCase,
    budget: BudgetCase,
    deadline: DeadlineCase,
) -> Expected {
    // Step 1: Deadline (if given) always runs first for every TxKind.
    if deadline == DeadlineCase::Passed {
        return Expected::RefusedDeadline;
    }
    // Step 2: Tick TX budget runs before stop gate (DisableRequested / ClientParkDisable / DisableProtective / DisableProbe exempt).
    if budget == BudgetCase::Exhausted
        && !matches!(
            kind,
            TxKind::DisableRequested { .. }
                | TxKind::ClientParkDisable { .. }
                | TxKind::DisableProtective { .. }
                | TxKind::DisableProbe
        )
    {
        return Expected::Deferred;
    }
    // Step 3: Stop gate by TxKind.
    match kind {
        TxKind::DisableRequested { .. }
        | TxKind::MitRecoveryDisable { .. }
        | TxKind::DisableProtective { .. }
        | TxKind::DisableProbe => Expected::Sent,
        TxKind::ClientParkDisable { .. } => match stop {
            StopCase::NoStop => Expected::Sent,
            StopCase::Pending | StopCase::Latched => Expected::RefusedStop,
        },
        TxKind::Refresh | TxKind::RollbackHold(_) | TxKind::MeasuredHold(_) | TxKind::ClearFault => match stop {
            StopCase::Pending => Expected::RefusedStop,
            StopCase::NoStop | StopCase::Latched => Expected::Sent,
        },
        TxKind::Target
        | TxKind::Hold
        | TxKind::ModeWrite
        | TxKind::Recovery
        | TxKind::Handshake => match stop {
            StopCase::NoStop => Expected::Sent,
            StopCase::Pending | StopCase::Latched => Expected::RefusedStop,
        },
    }
}

#[test]
fn send_chokepoint_table_all_kinds_stop_budget_deadline() {
    let limits = DamiaoLimits::default();
    let mut cells_checked = 0usize;

    let dummy_start = Instant::now();
    let kinds = all_tx_kinds(dummy_start);
    for &kind in &kinds {
        for &stop in &[StopCase::NoStop, StopCase::Pending, StopCase::Latched] {
            for &budget in &[BudgetCase::Available, BudgetCase::Exhausted] {
                for &deadline_case in &[DeadlineCase::Ok, DeadlineCase::Passed] {
                    let (bus, clock) = TestBus::new();
                    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1])
                        .with_limits(limits);

                    // Configure stop/E-stop state and resting-pose gate matching J1's fresh
                    // feedback (0.25 rad) BEFORE feeding K=3 spaced rest samples (S2) so
                    // `DisableRequested` passes Rule A′ in this table; separate tests below
                    // verify raised/stale/expired/no-gate refusals on `DisableRequested`.
                    let req = Arc::new(AtomicU64::new(match stop {
                        StopCase::NoStop => 0,
                        StopCase::Pending | StopCase::Latched => 1,
                    }));
                    let app = Arc::new(AtomicU64::new(match stop {
                        StopCase::NoStop => 0,
                        StopCase::Pending => 0,
                        StopCase::Latched => 1,
                    }));
                    let latched = Arc::new(AtomicBool::new(stop == StopCase::Latched));
                    drv.set_estop_gate(EstopGate::new(req, app));
                    drv.set_abort_flag(latched);
                    let mut rest_positions = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
                    rest_positions[0] = 0.25;
                    drv.set_stopped_disable_gate(StoppedDisableGate {
                        safety_stopped: Arc::new(AtomicBool::new(stop != StopCase::NoStop)),
                        has_resting_pose: true,
                        arm_joints: [0; crate::rt_core::mailbox::MAX_DOF],
                        arm_joints_len: 1,
                        rest_positions,
                        rest_pose_tolerance_rad: 0.05,
                    });

                    // Establish an empty-queue receive bound + K=3 spaced fresh samples on J1
                    // so both RollbackHold(0) freshness and S2 rest-pose K-window are satisfied.
                    drv.read_joint_states().unwrap();
                    for step in 0..3 {
                        if step > 0 {
                            clock.advance(Duration::from_millis(10));
                        }
                        bus.push_feedback(1, 0.25, 1, &limits);
                        drv.read_joint_states().unwrap();
                    }
                    bus.clear_tx();
                    let kind = match kind {
                        TxKind::DisableRequested { .. } => {
                            TxKind::DisableRequested { started_at: clock.now() }
                        }
                        TxKind::MitRecoveryDisable { .. } => {
                            TxKind::MitRecoveryDisable { started_at: clock.now() }
                        }
                        TxKind::ClientParkDisable { .. } => {
                            TxKind::ClientParkDisable { started_at: clock.now() }
                        }
                        other => other,
                    };

                    // Configure per-tick TX budget.
                    drv.set_tx_budget(TxBudget {
                        max_frames: 1,
                        max_duration: Duration::from_millis(5),
                        frame_max_age: Duration::from_millis(45),
                    });
                    if budget == BudgetCase::Exhausted {
                        drv.record_tx_pass_attempt();
                    }

                    // Configure deadline.
                    let now = clock.now();
                    let deadline = match deadline_case {
                        DeadlineCase::Ok => Some(Deadline::after(now, Duration::from_millis(10))),
                        DeadlineCase::Passed => Some(Deadline { at: now }),
                    };

                    let stamp = drv.current_stop_stamp();
                    let mut build_called = false;
                    let res = drv.send_plain(kind, Some(stamp), deadline, false, |d| {
                        build_called = true;
                        Some(damiao::build_pos_vel_frame(0x100, d.motor_ids[0], 0.25, 1.0))
                    });

                    let want = expected_outcome(kind, stop, budget, deadline_case);
                    match want {
                        Expected::Sent => {
                            assert!(
                                res.is_ok(),
                                "cell ({kind:?}, {stop:?}, {budget:?}, {deadline_case:?}) expected Sent, got {res:?}"
                            );
                            assert_eq!(bus.tx_len(), 1, "expected 1 TX for {kind:?}/{stop:?}/{budget:?}/{deadline_case:?}");
                            assert!(build_called, "build must run for Sent cell");
                        }
                        Expected::Deferred => {
                            assert!(
                                matches!(res, Err(TxRefusal::Deferred)),
                                "cell ({kind:?}, {stop:?}, {budget:?}, {deadline_case:?}) expected Deferred, got {res:?}"
                            );
                            assert_eq!(bus.tx_len(), 0);
                            assert!(!build_called, "budget check must run before build");
                        }
                        Expected::RefusedDeadline => {
                            assert!(
                                matches!(res, Err(TxRefusal::DeadlinePassed)),
                                "cell ({kind:?}, {stop:?}, {budget:?}, {deadline_case:?}) expected DeadlinePassed, got {res:?}"
                            );
                            assert_eq!(bus.tx_len(), 0);
                            assert!(!build_called, "deadline check must run before build");
                        }
                        Expected::RefusedStop => {
                            assert!(
                                matches!(res, Err(TxRefusal::StopInhibited)),
                                "cell ({kind:?}, {stop:?}, {budget:?}, {deadline_case:?}) expected StopInhibited, got {res:?}"
                            );
                            assert_eq!(bus.tx_len(), 0);
                            assert!(!build_called, "stop gate must run before build");
                        }
                    }
                    cells_checked += 1;
                }
            }
        }
    }
    assert_eq!(cells_checked, kinds.len() * 3 * 2 * 2);
}

#[test]
fn send_chokepoint_enforces_fresh_rollback_stop_stamp_and_drain_deadline() {
    let limits = DamiaoLimits::default();

    // 1. Latched RollbackHold(0) with STALE feedback (> 100 ms) fails closed in step 4 even
    //    if the caller's build closure would return a frame; fresh feedback allows send.
    let (bus, clock) = TestBus::new();
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1]).with_limits(limits);
    drv.read_joint_states().unwrap();
    bus.push_feedback(1, 0.35, 1, &limits);
    drv.read_joint_states().unwrap();
    drv.set_estop_gate(EstopGate::new(Arc::new(AtomicU64::new(1)), Arc::new(AtomicU64::new(1))));
    drv.set_abort_flag(Arc::new(AtomicBool::new(true)));

    clock.advance(Duration::from_millis(101));
    bus.clear_tx();
    let mut called = false;
    let stale_res = drv.send_plain(TxKind::RollbackHold(0), None, None, false, |d| {
        called = true;
        Some(damiao::build_pos_vel_frame(0x100, d.motor_ids[0], 0.35, 1.0))
    });
    assert!(matches!(stale_res, Err(TxRefusal::BuildRefused)), "stale latched rollback must fail closed: {stale_res:?}");
    assert!(!called, "step 4 latched freshness guard must refuse before build");
    assert_eq!(bus.tx_len(), 0);

    // Refresh feedback -> now fresh (<= 100 ms) latched RollbackHold(0) sends.
    drv.read_joint_states().unwrap();
    bus.push_feedback(1, 0.35, 1, &limits);
    drv.read_joint_states().unwrap();
    bus.clear_tx();
    assert!(drv
        .send_plain(TxKind::RollbackHold(0), None, None, false, |d| {
            let pos = d.fresh_measured_position(0, POS_FORCE_FEEDBACK_MAX_AGE)?;
            Some(d.pos_vel_frame(0, pos))
        })
        .is_ok());
    assert_eq!(bus.tx_len(), 1);

    // 2. Step 4 `build == None` fails closed without consuming tick TX budget.
    drv.begin_tx_tick();
    drv.set_tx_budget(TxBudget {
        max_frames: 1,
        max_duration: Duration::from_millis(5),
        frame_max_age: Duration::from_millis(45),
    });
    drv.set_abort_flag(Arc::new(AtomicBool::new(false)));
    bus.clear_tx();
    assert!(matches!(
        drv.send_plain(TxKind::Target, None, None, false, |_| None),
        Err(TxRefusal::BuildRefused)
    ));
    assert_eq!(bus.tx_len(), 0);
    // Budget slot was NOT consumed by BuildRefused, so the next valid frame succeeds.
    assert!(drv
        .send_plain(TxKind::Target, None, None, false, |d| Some(d.pos_vel_frame(0, 0.0)))
        .is_ok());
    assert_eq!(bus.tx_len(), 1);

    // 3. Step 5 stop-generation stamp check: applied stop between admission and send
    //    refuses every non-Disable kind with StopStampChanged (0 TX), while Disable sends.
    for &kind in &all_tx_kinds(Instant::now()) {
        let (bus, clock) = TestBus::new();
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1]).with_limits(limits);
        let mut rest_positions = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
        rest_positions[0] = 0.0;
        drv.set_stopped_disable_gate(StoppedDisableGate {
            safety_stopped: Arc::new(AtomicBool::new(false)),
            has_resting_pose: true,
            arm_joints: [0; crate::rt_core::mailbox::MAX_DOF],
            arm_joints_len: 1,
            rest_positions,
            rest_pose_tolerance_rad: 0.05,
        });
        drv.read_joint_states().unwrap();
        for step in 0..3 {
            if step > 0 {
                clock.advance(Duration::from_millis(10));
            }
            bus.push_feedback(1, 0.0, 1, &limits);
            drv.read_joint_states().unwrap();
        }
        let kind = match kind {
            TxKind::DisableRequested { .. } => {
                TxKind::DisableRequested { started_at: clock.now() }
            }
            TxKind::MitRecoveryDisable { .. } => {
                TxKind::MitRecoveryDisable { started_at: clock.now() }
            }
            other => other,
        };
        let stops = StopRequests::new();
        drv.set_estop_gate(
            EstopGate::new(Arc::new(AtomicU64::new(0)), Arc::new(AtomicU64::new(0)))
                .with_stops(stops.clone()),
        );
        let old_stamp = drv.current_stop_stamp();
        let g = stops.request();
        stops.mark_applied(g);
        bus.clear_tx();
        let res = drv.send_plain(kind, Some(old_stamp), None, false, |d| {
            Some(damiao::build_disable_frame(d.motor_ids[0]))
        });
        if matches!(
            kind,
            TxKind::DisableRequested { .. }
                | TxKind::MitRecoveryDisable { .. }
                | TxKind::DisableProtective { .. }
                | TxKind::DisableProbe
        ) {
            assert!(res.is_ok(), "Disable must ignore stop stamp: {res:?}");
            assert_eq!(bus.tx_len(), 1);
        } else {
            assert!(
                matches!(res, Err(TxRefusal::StopStampChanged)),
                "{kind:?} must fail with StopStampChanged after applied stop: {res:?}"
            );
            assert_eq!(bus.tx_len(), 0);
        }
    }

    // 4. Deferred restore drain stops once w.deadline passes mid-drain (0 frames after deadline).
    let (bus, clock) = TestBus::new();
    let clock_hook = clock.clone();
    *bus.on_tx.lock().unwrap() = Some(Box::new(move |_| {
        // Advance clock past the 50 ms restore deadline on the very first transmitted frame.
        clock_hook.advance(Duration::from_millis(60));
        Ok(())
    }));
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2]).with_limits(limits);
    drv.joint_modes[0] = Some(DamiaoControlMode::Mit);
    drv.joint_modes[1] = Some(DamiaoControlMode::Mit);
    let mut w = ModeWrite {
        modes: [DamiaoControlMode::PosVel; MAX_MODE_JOINTS],
        sent: [0; MAX_MODE_JOINTS],
        sent_len: 0,
        unsent: [0; MAX_MODE_JOINTS],
        unsent_len: 0,
        hold_failed: [0; MAX_MODE_JOINTS],
        hold_failed_len: 0,
        deferred: [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        deferred_stage: [0; MAX_MODE_JOINTS],
        deferred_len: 2,
        hold_pos: [0.0; MAX_MODE_JOINTS],
        dual_hold: false,
        flush_ok: true,
        deadline: Deadline::after(clock.now(), REGISTER_REPLY_TIMEOUT),
    };
    drv.drain_restore_deferred(&mut w).unwrap();
    assert_eq!(
        bus.tx_frames().len(),
        1,
        "drain_restore_deferred must stop starting frames once deadline passes: {:?}",
        bus.tx_frames()
    );
}

#[test]
fn send_chokepoint_rechecks_latch_and_pending_after_build() {
    let limits = DamiaoLimits::default();

    // A. Flip the physical E-stop latch (`abort_flag`) INSIDE `build` WITHOUT advancing the
    //    stop generation (simulates `trigger_e_stop_checked` setting `e_stop_latched` before
    //    acquiring `estop_reg` and incrementing `requested_estop_count`).
    //    Every latch-inhibited kind AND an unlatched->latched `RollbackHold` MUST be refused
    //    with `StopInhibited` and transmit ZERO frames.
    for &kind in &[
        TxKind::Target,
        TxKind::Hold,
        TxKind::ModeWrite,
        TxKind::Recovery,
        TxKind::Handshake,
        TxKind::RollbackHold(0),
    ] {
        let (bus, _clock) = TestBus::new();
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1]).with_limits(limits);
        drv.read_joint_states().unwrap();
        bus.push_feedback(1, 0.25, 1, &limits);
        drv.read_joint_states().unwrap();
        bus.clear_tx();

        let req = Arc::new(AtomicU64::new(0));
        let app = Arc::new(AtomicU64::new(0));
        let latched = Arc::new(AtomicBool::new(false));
        drv.set_estop_gate(EstopGate::new(req, app));
        drv.set_abort_flag(latched.clone());

        let stamp = drv.current_stop_stamp();
        let mut build_ran = false;
        let res = drv.send_plain(kind, Some(stamp), None, false, |d| {
            build_ran = true;
            // Flip latch mid-build; generation (`req`/`app`) deliberately unchanged!
            latched.store(true, Ordering::SeqCst);
            Some(d.pos_vel_frame(0, 0.25))
        });
        assert!(build_ran, "build must have executed before post-build latch check for {kind:?}");
        assert!(
            matches!(res, Err(TxRefusal::StopInhibited)),
            "latch flip during build must refuse {kind:?} with StopInhibited, got {res:?}"
        );
        assert_eq!(
            bus.tx_len(),
            0,
            "latch flip during build must transmit 0 frames for {kind:?}"
        );
    }

    // B. Flip pending stop (`req > app`) INSIDE `build` with `expected_stamp = None` so only
    //    the post-build `stop_gate_for_kind` re-read catches it: even latch-allowed kinds
    //    (`Refresh`, latched `RollbackHold`, `ClearFault`) must refuse with `StopInhibited` and 0 TX.
    for &kind in &[TxKind::Refresh, TxKind::RollbackHold(0), TxKind::ClearFault] {
        let (bus, _clock) = TestBus::new();
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1]).with_limits(limits);
        drv.read_joint_states().unwrap();
        bus.push_feedback(1, 0.25, 1, &limits);
        drv.read_joint_states().unwrap();
        bus.clear_tx();

        let req = Arc::new(AtomicU64::new(1));
        let app = Arc::new(AtomicU64::new(1));
        let latched = Arc::new(AtomicBool::new(true));
        drv.set_estop_gate(EstopGate::new(req.clone(), app));
        drv.set_abort_flag(latched);

        let res = drv.send_plain(kind, None, None, false, |d| {
            req.fetch_add(1, Ordering::SeqCst);
            Some(damiao::build_refresh_frame(d.motor_ids[0]))
        });
        assert!(
            matches!(res, Err(TxRefusal::StopInhibited)),
            "pending stop raised during build must refuse {kind:?} with StopInhibited, got {res:?}"
        );
        assert_eq!(bus.tx_len(), 0);
    }

    // C. Protective `DisableProtective` still transmits even if both latch and pending flip during `build`.
    let (bus, _clock) = TestBus::new();
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1]).with_limits(limits);
    let req = Arc::new(AtomicU64::new(0));
    let app = Arc::new(AtomicU64::new(0));
    let latched = Arc::new(AtomicBool::new(false));
    drv.set_estop_gate(EstopGate::new(req.clone(), app));
    drv.set_abort_flag(latched.clone());
    let res = drv.send_plain(
        TxKind::DisableProtective { reason: DisableProtectiveReason::ProcessExit },
        None,
        None,
        false,
        |d| {
            latched.store(true, Ordering::SeqCst);
            req.fetch_add(1, Ordering::SeqCst);
            Some(damiao::build_disable_frame(d.motor_ids[0]))
        },
    );
    assert!(res.is_ok(), "DisableProtective must remain ungated across build-time stop flip: {res:?}");
    assert_eq!(bus.tx_len(), 1);

    // D. `DisableRequested` on a raised arm refuses if stop flips DURING `build` (Step 5 re-check):
    //    0 CAN frames transmitted.
    let (bus, clock) = TestBus::new();
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1]).with_limits(limits);
    drv.read_joint_states().unwrap();
    bus.push_feedback(1, 0.50, 1, &limits);
    drv.read_joint_states().unwrap();
    bus.clear_tx();
    let safety_stopped = Arc::new(AtomicBool::new(false));
    drv.set_stopped_disable_gate(StoppedDisableGate {
        safety_stopped: safety_stopped.clone(),
        has_resting_pose: true,
        arm_joints: [0; crate::rt_core::mailbox::MAX_DOF],
        arm_joints_len: 1,
        rest_positions: [0.0; crate::rt_core::mailbox::MAX_DOF],
        rest_pose_tolerance_rad: 0.05,
    });
    let started_at = clock.now();
    let res = drv.send_plain(
        TxKind::DisableRequested { started_at },
        None,
        None,
        false,
        |d| {
            safety_stopped.store(true, Ordering::SeqCst);
            Some(damiao::build_disable_frame(d.motor_ids[0]))
        },
    );
    assert!(
        matches!(res, Err(TxRefusal::TorqueOffWouldDrop)),
        "DisableRequested on raised arm must refuse even before stop flips: {res:?}"
    );
    assert_eq!(bus.tx_len(), 0);
}

#[test]
fn send_chokepoint_enforces_rest_pose_and_1s_lifetime_for_disable_requested() {
    let limits = DamiaoLimits::default();
    let (bus, clock) = TestBus::new();
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1]).with_limits(limits);
    let safety_stopped = Arc::new(AtomicBool::new(true));
    drv.set_stopped_disable_gate(StoppedDisableGate {
        safety_stopped: safety_stopped.clone(),
        has_resting_pose: true,
        arm_joints: [0; crate::rt_core::mailbox::MAX_DOF],
        arm_joints_len: 1,
        rest_positions: [0.0; crate::rt_core::mailbox::MAX_DOF],
        rest_pose_tolerance_rad: 0.05,
    });

    // 1. Raised arm -> refused with TorqueOffWouldDrop, 0 TX.
    drv.read_joint_states().unwrap();
    bus.push_feedback(1, 0.40, 1, &limits);
    drv.read_joint_states().unwrap();
    bus.clear_tx();
    let t0 = clock.now();
    assert!(matches!(
        drv.send_plain(TxKind::DisableRequested { started_at: t0 }, None, None, false, |d| {
            Some(damiao::build_disable_frame(d.motor_ids[0]))
        }),
        Err(TxRefusal::TorqueOffWouldDrop)
    ));
    assert_eq!(bus.tx_len(), 0);

    // 2. At rest with K=3 spaced causal samples (S2) -> sent.
    drv.read_joint_states().unwrap();
    for step in 0..3 {
        if step > 0 {
            clock.advance(Duration::from_millis(10));
        }
        bus.push_feedback(1, 0.01, 1, &limits);
        drv.read_joint_states().unwrap();
    }
    bus.clear_tx();
    assert!(drv
        .send_plain(TxKind::DisableRequested { started_at: t0 }, None, None, false, |d| {
            Some(damiao::build_disable_frame(d.motor_ids[0]))
        })
        .is_ok());
    assert_eq!(bus.tx_len(), 1);

    // 3. Stale feedback (> 100 ms) within 1 s lifetime -> defers with DisableWaitEvidence (M3), 0 TX.
    clock.advance(Duration::from_millis(101));
    bus.clear_tx();
    assert!(matches!(
        drv.send_plain(TxKind::DisableRequested { started_at: t0 }, None, None, false, |d| {
            Some(damiao::build_disable_frame(d.motor_ids[0]))
        }),
        Err(TxRefusal::DisableWaitEvidence)
    ));
    assert_eq!(bus.tx_len(), 0);

    // 4. Fresh feedback at rest, but 1 s total request lifetime elapsed -> DisableRequestExpired, 0 TX.
    clock.advance(Duration::from_millis(950));
    drv.read_joint_states().unwrap();
    for step in 0..3 {
        if step > 0 {
            clock.advance(Duration::from_millis(10));
        }
        bus.push_feedback(1, 0.0, 1, &limits);
        drv.read_joint_states().unwrap();
    }
    bus.clear_tx();
    assert!(matches!(
        drv.send_plain(TxKind::DisableRequested { started_at: t0 }, None, None, false, |d| {
            Some(damiao::build_disable_frame(d.motor_ids[0]))
        }),
        Err(TxRefusal::DisableRequestExpired)
    ));
    assert_eq!(bus.tx_len(), 0);

    // 5. NO already-off bypass on a raised arm: fresh status 0x0 (disabled), 0xD (watchdog),
    //    0x1 (enabled), and 0x8 (overvoltage) are ALL refused with TorqueOffWouldDrop and 0 TX.
    let (bus2, clock2) = TestBus::new();
    let mut drv2 = DamiaoArmDriver::new(bus2.clone(), vec![1, 2, 3, 4]).with_limits(limits);
    let mut arm_joints = [0usize; crate::rt_core::mailbox::MAX_DOF];
    arm_joints[0] = 0;
    arm_joints[1] = 1;
    arm_joints[2] = 2;
    arm_joints[3] = 3;
    drv2.set_stopped_disable_gate(StoppedDisableGate {
        safety_stopped: Arc::new(AtomicBool::new(true)),
        has_resting_pose: true,
        arm_joints,
        arm_joints_len: 4,
        rest_positions: [0.0; crate::rt_core::mailbox::MAX_DOF],
        rest_pose_tolerance_rad: 0.05,
    });
    drv2.read_joint_states().unwrap();
    bus2.push_feedback(1, 0.50, 0x0, &limits);
    bus2.push_feedback(2, 0.50, 0xD, &limits);
    bus2.push_feedback(3, 0.50, 0x1, &limits);
    bus2.push_feedback(4, 0.50, 0x8, &limits);
    drv2.read_joint_states().unwrap();
    bus2.clear_tx();
    let t2 = clock2.now();
    for joint in 0..4 {
        assert!(matches!(
            drv2.send_plain(TxKind::DisableRequested { started_at: t2 }, None, None, false, |d| {
                Some(damiao::build_disable_frame(d.motor_ids[joint]))
            }),
            Err(TxRefusal::TorqueOffWouldDrop)
        ));
    }
    assert_eq!(bus2.tx_len(), 0, "raised DisableRequested must send 0 frames regardless of motor status");

    // 6. Operational (not stopped) ALSO enforces the rest-pose gate and 1 s lifetime (B5):
    //    raised pose and expired request are BOTH refused with 0 TX even when safety_stopped == false.
    safety_stopped.store(false, Ordering::SeqCst);
    drv.read_joint_states().unwrap();
    bus.push_feedback(1, 0.80, 1, &limits);
    drv.read_joint_states().unwrap();
    let t_op = clock.now();
    bus.clear_tx();
    assert!(matches!(
        drv.send_plain(TxKind::DisableRequested { started_at: t_op }, None, None, false, |d| {
            Some(damiao::build_disable_frame(d.motor_ids[0]))
        }),
        Err(TxRefusal::TorqueOffWouldDrop)
    ));
    assert_eq!(bus.tx_len(), 0, "operational DisableRequested at raised pose must send 0 TX (B5)");
    clock.advance(Duration::from_millis(500));
    bus.clear_tx();
    assert!(matches!(
        drv.send_plain(TxKind::DisableRequested { started_at: t0 }, None, None, false, |d| {
            Some(damiao::build_disable_frame(d.motor_ids[0]))
        }),
        Err(TxRefusal::DisableRequestExpired)
    ));
    assert_eq!(bus.tx_len(), 0, "operational DisableRequested past 1 s lifetime must send 0 TX (B5)");
}

#[test]
fn write_measured_hold_masked_holds_every_joint_even_when_feedback_claims_state_0() {
    let limits = damiao::DamiaoLimits::default();
    let (bus, clock) = TestBus::new();
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3])
        .with_joint_limits(vec![limits; 3])
        .unwrap();
    drv.verified_modes.fill(Some(DamiaoControlMode::PosVel));
    let mut arm_joints = [0usize; crate::rt_core::mailbox::MAX_DOF];
    arm_joints[0] = 0;
    arm_joints[1] = 1;
    arm_joints[2] = 2;
    drv.set_stopped_disable_gate(StoppedDisableGate {
        safety_stopped: Arc::new(AtomicBool::new(false)),
        has_resting_pose: true,
        arm_joints,
        arm_joints_len: 3,
        rest_positions: [0.50, 0.50, 0.50, 0.0, 0.0, 0.0, 0.0, 0.0],
        rest_pose_tolerance_rad: 0.05,
    });
    drv.read_joint_states().unwrap();
    for step in 0..3 {
        if step > 0 {
            clock.advance(Duration::from_millis(10));
        }
        for id in 1..=3 {
            bus.push_feedback(id, 0.50, 0x0, &limits);
        }
        drv.read_joint_states().unwrap();
    }
    // Feedback reports state=0 (telemetry Disabled), yet holds are commanded to EVERY joint.
    assert_eq!(drv.enable_state(), EnableState::Disabled);
    let targets = [
        JointTarget::position(0.50),
        JointTarget::position(0.50),
        JointTarget::position(0.50),
    ];
    bus.clear_tx();
    let report = drv.write_measured_hold_masked(0b111, &targets).unwrap();
    assert_eq!(report.joint_status[..3], [JointHoldStatus::Held; 3]);
    assert_eq!(bus.tx_len(), 3, "holds must be emitted to every joint even when state=0");

    // Bounded disable repeats: 3 repeats spaced 20 ms apart complete via decision without waiting
    // on 150 ms feedback confirmation.
    bus.clear_tx();
    let started_at = clock.now();
    let mut prog = drv
        .begin_disable_torque(DisableKind::DisableRequested { started_at })
        .unwrap();
    assert!(matches!(prog, TorqueProgress::Pending { .. }));
    assert_eq!(bus.tx_len(), 3, "repeat 1 sent to all 3 motors");
    clock.advance(DAMIAO_DISABLE_REPEAT_INTERVAL);
    prog = drv.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
    assert!(matches!(prog, TorqueProgress::Pending { .. }));
    assert_eq!(bus.tx_len(), 6, "repeat 2 sent to all 3 motors");
    clock.advance(DAMIAO_DISABLE_REPEAT_INTERVAL);
    prog = drv.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
    assert_eq!(prog, TorqueProgress::Done(TorqueOutcome::Confirmed));
    assert_eq!(bus.tx_len(), 9, "repeat 3 sent to all 3 motors");
}

#[test]
fn fashionstar_and_robstride_gate_disable_per_write_across_estop_freshness_and_lifetime() {
    // Hookable serial transport for FashionStar per-write gating tests.
    #[derive(Clone)]
    struct HookSerial {
        clock: MockClock,
        writes: Arc<Mutex<Vec<Vec<u8>>>>,
        on_write: Arc<Mutex<Option<Box<dyn FnMut(usize) + Send>>>>,
    }
    impl HookSerial {
        fn new(clock: MockClock) -> Self {
            Self {
                clock,
                writes: Arc::new(Mutex::new(Vec::new())),
                on_write: Arc::new(Mutex::new(None)),
            }
        }
        fn set_on_write(&self, cb: impl FnMut(usize) + Send + 'static) {
            *self.on_write.lock().unwrap() = Some(Box::new(cb));
        }
        fn clear_writes(&self) {
            self.writes.lock().unwrap().clear();
        }
        fn write_count(&self) -> usize {
            self.writes.lock().unwrap().len()
        }
    }
    impl Transport for HookSerial {
        fn write(&mut self, data: &[u8]) -> Result<usize, TransportError> {
            self.write_all(data)?;
            Ok(data.len())
        }
        fn write_all(&mut self, data: &[u8]) -> Result<(), TransportError> {
            let n = {
                let mut w = self.writes.lock().unwrap();
                w.push(data.to_vec());
                w.len()
            };
            if let Some(cb) = self.on_write.lock().unwrap().as_mut() {
                cb(n);
            }
            Ok(())
        }
        fn read(&mut self, _buf: &mut [u8]) -> Result<usize, TransportError> {
            Err(TransportError::Timeout)
        }
        fn flush(&mut self) -> Result<(), TransportError> {
            Ok(())
        }
        fn clear_input_buffer(&mut self) {}
        fn clock(&self) -> Option<Arc<dyn DriverClock>> {
            Some(Arc::new(self.clock.clone()))
        }
    }

    let mut arm3 = [0usize; crate::rt_core::mailbox::MAX_DOF];
    arm3[0] = 0;
    arm3[1] = 1;
    arm3[2] = 2;
    let seed_rest_window = |windows: &mut [CausalWindow], now: Instant| {
        let t0 = now.checked_sub(Duration::from_millis(20)).unwrap_or(now);
        let t1 = now.checked_sub(Duration::from_millis(10)).unwrap_or(now);
        for w in windows.iter_mut() {
            w.reset();
            let _ = w.record_agree(t0, 1, STOPPED_DISABLE_FEEDBACK_MAX_AGE, None);
            let _ = w.record_agree(t1, 2, STOPPED_DISABLE_FEEDBACK_MAX_AGE, None);
            let _ = w.record_agree(now, 3, STOPPED_DISABLE_FEEDBACK_MAX_AGE, None);
        }
    };

    // --- A. FashionStarArmDriver per-write gating ---
    {
        let clock = MockClock::new();
        let serial = HookSerial::new(clock.clone());
        let mut drv = FashionStarArmDriver::new(serial.clone(), vec![0, 1, 2]);
        let stopped = Arc::new(AtomicBool::new(false));
        drv.set_stopped_disable_gate(StoppedDisableGate {
            safety_stopped: Arc::clone(&stopped),
            has_resting_pose: true,
            arm_joints: arm3,
            arm_joints_len: 3,
            rest_positions: [0.0; crate::rt_core::mailbox::MAX_DOF],
            rest_pose_tolerance_rad: 0.05,
        });
        let t0 = clock.now();
        // 1. Raised arm when NOT stopped -> write #1 refused immediately (0 writes, B5)!
        for i in 0..3 {
            drv.cached_states[i].position = 0.50;
            drv.last_feedback_instant[i] = Some(t0);
        }
        let res = drv.disable_torque(DisableKind::DisableRequested { started_at: t0 });
        assert!(
            matches!(res, Err(ArmDriverError::TorqueOffWouldDrop)),
            "FashionStar raised operational DisableRequested must refuse write #1, got {res:?}"
        );
        assert_eq!(serial.write_count(), 0, "FashionStar must transmit 0 writes on raised arm (B5)");

        // 2. Mid-batch freshness cutoff (> 100 ms) at rest -> writes #2,#3 refused!
        serial.clear_writes();
        clock.advance(Duration::from_millis(20));
        let t1 = clock.now();
        for i in 0..3 {
            drv.cached_states[i].position = 0.0;
            drv.last_feedback_instant[i] = Some(t1);
        }
        seed_rest_window(&mut drv.rest_pose_window, t1);
        let clk_cb = clock.clone();
        serial.set_on_write(move |n| {
            if n == 1 {
                clk_cb.advance(Duration::from_millis(105));
            }
        });
        let res = drv.disable_torque(DisableKind::DisableRequested { started_at: t1 });
        assert!(
            matches!(res, Err(ArmDriverError::TorqueOffWouldDrop)),
            "FashionStar mid-batch freshness cutoff must refuse write #2+, got {res:?}"
        );
        assert_eq!(serial.write_count(), 1);

        // 3. Mid-batch 1 s lifetime cutoff at rest (feedback stays fresh!) -> DisableRequestExpired!
        serial.clear_writes();
        let t_req = clock.now();
        clock.advance(Duration::from_millis(995));
        let t_fb = clock.now();
        for i in 0..3 {
            drv.cached_states[i].position = 0.0;
            drv.last_feedback_instant[i] = Some(t_fb);
        }
        seed_rest_window(&mut drv.rest_pose_window, t_fb);
        let clk_cb2 = clock.clone();
        serial.set_on_write(move |n| {
            if n == 1 {
                clk_cb2.advance(Duration::from_millis(10)); // past 1 s, feedback still only 10 ms old
            }
        });
        let res = drv.disable_torque(DisableKind::DisableRequested { started_at: t_req });
        assert!(
            matches!(res, Err(ArmDriverError::DisableRequestExpired)),
            "FashionStar mid-batch 1 s lifetime cutoff must refuse write #2+, got {res:?}"
        );
        assert_eq!(serial.write_count(), 1);

        // 4. DisableProtective sends all 3 writes regardless of E-stop / staleness.
        serial.clear_writes();
        serial.set_on_write(|_| {});
        drv.disable_torque(DisableKind::DisableProtective {
            reason: DisableProtectiveReason::Thermal,
        })
        .unwrap();
        assert_eq!(serial.write_count(), 3);
    }

    // --- B. RobStrideArmDriver per-write gating ---
    {
        let (bus, clock) = TestBus::new();
        let mut drv = RobStrideArmDriver::new(bus.clone(), vec![1, 2, 3]);
        let stopped = Arc::new(AtomicBool::new(false));
        drv.set_stopped_disable_gate(StoppedDisableGate {
            safety_stopped: Arc::clone(&stopped),
            has_resting_pose: true,
            arm_joints: arm3,
            arm_joints_len: 3,
            rest_positions: [0.0; crate::rt_core::mailbox::MAX_DOF],
            rest_pose_tolerance_rad: 0.05,
        });
        let t0 = clock.now();
        // 1. Raised arm when NOT stopped -> frame #1 refused immediately (0 TX, B5)!
        for i in 0..3 {
            drv.cached_states[i].position = 0.50;
            drv.last_feedback_instant[i] = Some(t0);
            drv.pose_feedback_instant[i] = Some(t0);
            drv.feedback_queue_bound[i] = Some(t0);
        }
        let res = drv.disable_torque(DisableKind::DisableRequested { started_at: t0 });
        assert!(
            matches!(res, Err(ArmDriverError::TorqueOffWouldDrop)),
            "RobStride raised operational DisableRequested must refuse frame #1, got {res:?}"
        );
        assert_eq!(bus.tx_len(), 0, "RobStride must transmit 0 frames on raised arm (B5)");

        // 2. Mid-batch freshness cutoff (> 100 ms) at rest -> frames #2,#3 refused!
        bus.clear_tx();
        clock.advance(Duration::from_millis(20));
        let t1 = clock.now();
        for i in 0..3 {
            drv.cached_states[i].position = 0.0;
            drv.last_feedback_instant[i] = Some(t1);
            drv.pose_feedback_instant[i] = Some(t1);
            drv.feedback_queue_bound[i] = Some(t1);
        }
        seed_rest_window(&mut drv.rest_pose_window, t1);
        let clk_cb = clock.clone();
        let mut count2 = 0usize;
        bus.set_on_tx(move |_| {
            count2 += 1;
            if count2 == 1 {
                clk_cb.advance(Duration::from_millis(105));
            }
            Ok(())
        });
        let res = drv.disable_torque(DisableKind::DisableRequested { started_at: t1 });
        assert!(
            matches!(res, Err(ArmDriverError::TorqueOffWouldDrop)),
            "RobStride mid-batch freshness cutoff must refuse frame #2+, got {res:?}"
        );
        assert_eq!(bus.tx_len(), 1);

        // 3. Mid-batch 1 s lifetime cutoff at rest (feedback stays fresh!) -> DisableRequestExpired!
        bus.clear_tx();
        let t_req = clock.now();
        clock.advance(Duration::from_millis(995));
        let t_fb = clock.now();
        for i in 0..3 {
            drv.cached_states[i].position = 0.0;
            drv.last_feedback_instant[i] = Some(t_fb);
            drv.pose_feedback_instant[i] = Some(t_fb);
            drv.feedback_queue_bound[i] = Some(t_fb);
        }
        seed_rest_window(&mut drv.rest_pose_window, t_fb);
        let clk_cb2 = clock.clone();
        let mut count3 = 0usize;
        bus.set_on_tx(move |_| {
            count3 += 1;
            if count3 == 1 {
                clk_cb2.advance(Duration::from_millis(10));
            }
            Ok(())
        });
        let res = drv.disable_torque(DisableKind::DisableRequested { started_at: t_req });
        assert!(
            matches!(res, Err(ArmDriverError::DisableRequestExpired)),
            "RobStride mid-batch 1 s lifetime cutoff must refuse frame #2+, got {res:?}"
        );
        assert_eq!(bus.tx_len(), 1);

        // 4. DisableProtective sends all 3 frames regardless of E-stop / staleness.
        bus.clear_tx();
        bus.set_on_tx(|_| Ok(()));
        drv.disable_torque(DisableKind::DisableProtective {
            reason: DisableProtectiveReason::Thermal,
        })
        .unwrap();
        assert_eq!(bus.tx_len(), 3);

        // 5. RobStride `clear_fault()` (type-4 stop/fault-clear frames) routes through the
        //    exact same per-write disable choke point: refused raised / stale / mid-batch
        //    E-stop / freshness / 1 s lifetime, allowed at fresh rest pose.
        stopped.store(true, Ordering::SeqCst);
        let t_cf = clock.now();
        for i in 0..3 {
            drv.cached_states[i].position = 0.50;
            drv.last_feedback_instant[i] = Some(t_cf);
            drv.pose_feedback_instant[i] = Some(t_cf);
            drv.feedback_queue_bound[i] = Some(t_cf);
        }
        bus.clear_tx();
        assert!(matches!(
            drv.clear_fault(),
            Err(ArmDriverError::TorqueOffWouldDrop)
        ));
        assert_eq!(bus.tx_len(), 0, "RobStride clear_fault while stopped & raised must send 0 frames");

        // At rest with stale (> 100 ms) feedback -> refused with 0 frames.
        for i in 0..3 {
            drv.cached_states[i].position = 0.0;
        }
        clock.advance(Duration::from_millis(105));
        bus.clear_tx();
        assert!(matches!(
            drv.clear_fault(),
            Err(ArmDriverError::TorqueOffWouldDrop)
        ));
        assert_eq!(bus.tx_len(), 0, "RobStride clear_fault while stopped & stale must send 0 frames");

        // Raised arm when NOT stopped -> clear_fault refuses frame #1 immediately (0 TX, B5)!
        stopped.store(false, Ordering::SeqCst);
        let t_cf2 = clock.now();
        for i in 0..3 {
            drv.cached_states[i].position = 0.50;
            drv.last_feedback_instant[i] = Some(t_cf2);
            drv.pose_feedback_instant[i] = Some(t_cf2);
            drv.feedback_queue_bound[i] = Some(t_cf2);
        }
        bus.set_on_tx(|_| Ok(()));
        bus.clear_tx();
        assert!(matches!(
            drv.clear_fault(),
            Err(ArmDriverError::TorqueOffWouldDrop)
        ));
        assert_eq!(bus.tx_len(), 0, "RobStride clear_fault on raised operational arm must transmit 0 frames (B5)");

        // Mid-batch freshness cutoff (> 100 ms) while stopped at rest stops after frame #1.
        clock.advance(Duration::from_millis(20));
        let t_cf3 = clock.now();
        for i in 0..3 {
            drv.cached_states[i].position = 0.0;
            drv.last_feedback_instant[i] = Some(t_cf3);
            drv.pose_feedback_instant[i] = Some(t_cf3);
            drv.feedback_queue_bound[i] = Some(t_cf3);
        }
        seed_rest_window(&mut drv.rest_pose_window, t_cf3);
        let clk_cf = clock.clone();
        let mut count_cf2 = 0usize;
        bus.set_on_tx(move |_| {
            count_cf2 += 1;
            if count_cf2 == 1 {
                clk_cf.advance(Duration::from_millis(105));
            }
            Ok(())
        });
        bus.clear_tx();
        assert!(matches!(
            drv.clear_fault(),
            Err(ArmDriverError::TorqueOffWouldDrop)
        ));
        assert_eq!(bus.tx_len(), 1, "RobStride clear_fault must stop after frame #1 on mid-batch staleness");

        // Fresh rest pose while stopped -> transmits all 3 type-4 clear-fault frames.
        clock.advance(Duration::from_millis(20));
        let t_cf4 = clock.now();
        for i in 0..3 {
            drv.cached_states[i].position = 0.0;
            drv.last_feedback_instant[i] = Some(t_cf4);
            drv.pose_feedback_instant[i] = Some(t_cf4);
            drv.feedback_queue_bound[i] = Some(t_cf4);
        }
        seed_rest_window(&mut drv.rest_pose_window, t_cf4);
        bus.set_on_tx(|_| Ok(()));
        bus.clear_tx();
        drv.clear_fault().expect("RobStride clear_fault at fresh rest pose must succeed");
        assert_eq!(bus.tx_len(), 3);
    }
}

/// Round 8 D1: Boot fallback (`PosForceBootFallback::NoOutputUntilFresh`).
/// 1. With no provably fresh J7 sample since process start (`pf_last_fresh_position == None`),
///    emit ZERO `0x307` frames while keeping `0xCC` refresh requests active (J7 keeps its last
///    firmware command under `TIMEOUT=0`).
/// 2. After the first provably fresh J7 sample is decoded, emit a stationary `0x307` Hold at
///    that measured position (`vel_raw = 0`) with the configured `current_raw` cap (even if a
///    `Target` intent is submitted on that first fresh tick).
/// 3. A STOP before the first fresh sample still sends NO `0x307` frame (while sending `0xCC`),
///    and is applied as Hold at the measured position with the cap once fresh.
#[test]
fn pos_force_boot_fallback_no_output_until_fresh_then_holds_measured_with_cap() {
    let limits = damiao::DamiaoLimits::default();
    let (bus, clock) = TestBus::new();
    let pf = GripperPosForce {
        joint: 0,
        current_raw: 460,
        close_speed_rad_s: 0.30,
        closed_rad: 0.0,
        open_rad: -5.7,
        calibrated: false,
        ceiling_torque_nm: None,
        torque_reference: None,
    };
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![7])
        .with_limits(limits)
        .with_gripper_pos_force(pf)
        .unwrap();
    // Simulate verified PosForce mode with NO provably fresh J7 sample ever decoded since boot.
    drv.pos_force_verified = true;
    drv.joint_modes[0] = Some(DamiaoControlMode::PosForce);
    drv.pf_last_fresh_position = None;
    drv.is_enabled = true;

    assert_eq!(POS_FORCE_BOOT_FALLBACK, PosForceBootFallback::NoOutputUntilFresh);
    assert_eq!(drv.pos_force_freeze_params(0), None);

    // 1. Before first fresh sample: Target intent -> ZERO 0x307 frames, 0xCC refresh active.
    bus.clear_tx();
    drv.begin_tx_tick();
    let stamp = drv.current_j7_intent_stamp();
    let out0 = drv
        .emit_pos_force_j7(PosForceIntent::target(0.0, Some(0.30), None, stamp), stamp)
        .expect("boot fallback awaiting fresh");
    assert!(
        matches!(
            out0,
            PosForceEmitOutcome::AwaitingFirstFresh {
                joint: 0,
                pf_status: PosForceTxStatus::Stale { joint: 0, .. }
            }
        ),
        "expected AwaitingFirstFresh with Stale status, got {out0:?}"
    );
    let log0: Vec<CanFrame> = bus.tx.lock().unwrap().clone();
    assert!(
        log0.iter().all(|f| f.id != 0x307),
        "must send NO 0x307 frame before first fresh sample: {log0:?}"
    );
    assert!(
        log0.iter().any(|f| f.id == 0x7FF && f.data[0] == 7 && f.data[2] == 0xCC),
        "expected 0xCC refresh while awaiting first fresh sample: {log0:?}"
    );

    // 2. First fresh J7 sample arrives (-2.40 rad) -> next tick emits Hold at -2.40 rad with
    //    vel_raw = 0 and configured current_raw = 460 (even if a Target intent is submitted).
    let now1 = clock.now();
    drv.cached_states[0].position = -2.40;
    drv.last_feedback_instant[0] = Some(now1);
    drv.feedback_queue_bound[0] = Some(now1);
    drv.pf_last_fresh_position = Some(-2.40);
    bus.clear_tx();
    drv.begin_tx_tick();
    let out1 = drv
        .emit_pos_force_j7(PosForceIntent::target(0.0, Some(0.30), None, stamp), stamp)
        .expect("first fresh hold");
    assert!(matches!(out1, PosForceEmitOutcome::Sent { joint: 0, .. }));
    let log1: Vec<CanFrame> = bus.tx.lock().unwrap().clone();
    let j7_1: Vec<CanFrame> = log1.iter().copied().filter(|f| f.id == 0x307).collect();
    assert_eq!(j7_1.len(), 1, "expected 1 Hold 0x307 frame on first fresh sample: {log1:?}");
    assert!((f32::from_le_bytes(j7_1[0].data[0..4].try_into().unwrap()) - (-2.40)).abs() < 1e-6);
    assert_eq!(u16::from_le_bytes([j7_1[0].data[4], j7_1[0].data[5]]), 0);
    assert_eq!(u16::from_le_bytes([j7_1[0].data[6], j7_1[0].data[7]]), 460);

    // 3. STOP before the first fresh sample: still sends NO 0x307 frame (sends 0xCC), and
    //    once the first fresh sample arrives (-1.75 rad), applies as Hold with cap 460.
    drv.reset_pos_force_boot_state();
    let req = Arc::new(AtomicU64::new(1));
    let app = Arc::new(AtomicU64::new(1));
    let latched = Arc::new(AtomicBool::new(true));
    drv.set_estop_gate(EstopGate::new(req, app));
    drv.set_abort_flag(latched);

    bus.clear_tx();
    drv.begin_tx_tick();
    let targets = [JointTarget::position(0.0)];
    let outcome = drv.write_estop_hold_bounded(&targets).expect("stop before first fresh");
    assert_eq!(outcome, TxBatchOutcome::Complete);
    let log_stop_pre: Vec<CanFrame> = bus.tx.lock().unwrap().clone();
    assert!(
        log_stop_pre.iter().all(|f| f.id != 0x307),
        "STOP before first fresh sample must send NO 0x307 frame: {log_stop_pre:?}"
    );
    assert!(
        log_stop_pre.iter().any(|f| f.id == 0x7FF && f.data[0] == 7 && f.data[2] == 0xCC),
        "STOP before first fresh sample must still request 0xCC refresh: {log_stop_pre:?}"
    );

    let now2 = clock.now();
    drv.cached_states[0].position = -1.75;
    drv.last_feedback_instant[0] = Some(now2);
    drv.feedback_queue_bound[0] = Some(now2);
    drv.pf_last_fresh_position = Some(-1.75);
    bus.clear_tx();
    drv.begin_tx_tick();
    let outcome2 = drv.write_estop_hold_bounded(&targets).expect("stop applied once fresh");
    assert_eq!(outcome2, TxBatchOutcome::Complete);
    let log_stop_post: Vec<CanFrame> = bus.tx.lock().unwrap().clone();
    let j7_post: Vec<CanFrame> = log_stop_post.iter().copied().filter(|f| f.id == 0x307).collect();
    assert_eq!(j7_post.len(), 1, "STOP must apply as Hold once first fresh sample arrives: {log_stop_post:?}");
    assert!((f32::from_le_bytes(j7_post[0].data[0..4].try_into().unwrap()) - (-1.75)).abs() < 1e-6);
    assert_eq!(u16::from_le_bytes([j7_post[0].data[4], j7_post[0].data[5]]), 0);
    assert_eq!(u16::from_le_bytes([j7_post[0].data[6], j7_post[0].data[7]]), 460);
}

/// Round 8 M1: Type-system capability guard enforcement.
/// 1. `DamiaoArmDriver::send` always passes `pos_force_token = None`, so even a forged
///    raw `0x307` `CanFrame` built without `PosForceSendToken` is refused with `BuildRefused`
///    and never reaches the transport.
/// 2. `pos_force_emitter::emit_standalone_recovery_hold` refuses with `BuildRefused` when
///    `external_pos_force_emitter == true` (owner tick epilogue active).
#[test]
fn send_without_pos_force_emitter_token_rejects_forged_0x307_frame() {
    let limits = damiao::DamiaoLimits::default();
    let (bus, clock) = TestBus::new();
    let pf = GripperPosForce {
        joint: 0,
        current_raw: 460,
        close_speed_rad_s: 0.30,
        closed_rad: 0.0,
        open_rad: -5.7,
        calibrated: false,
        ceiling_torque_nm: None,
        torque_reference: None,
    };
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![7])
        .with_limits(limits)
        .with_gripper_pos_force(pf)
        .unwrap();
    let now = clock.now();
    drv.pos_force_verified = true;
    drv.joint_modes[0] = Some(DamiaoControlMode::PosForce);
    drv.is_enabled = true;
    drv.cached_states[0].position = -1.0;
    drv.last_feedback_instant[0] = Some(now);
    drv.feedback_queue_bound[0] = Some(now);
    drv.pf_last_fresh_position = Some(-1.0);

    // 1. Even with fresh feedback and valid payload, `drv.send(...)` (no `PosForceSendToken`)
    //    refuses any `0x307` frame with `BuildRefused` and emits 0 wire frames.
    bus.clear_tx();
    let forged = CanFrame::new_standard(
        damiao::POS_FORCE_BASE_ID + 7,
        &damiao::encode_pos_force_payload(-1.0, 0, 460),
    )
    .unwrap();
    let res = drv.send_plain(TxKind::Target, None, None, false, |_| Some(forged));
    assert!(
        matches!(res, Err(TxRefusal::BuildRefused)),
        "forged 0x307 through send() without PosForceSendToken must fail with BuildRefused, got {res:?}"
    );
    assert_eq!(bus.tx_len(), 0, "forged 0x307 must never reach CAN transport");

    // 2. Narrow `emit_standalone_recovery_hold` fails closed when an external tick owner is active.
    drv.set_external_pos_force_emitter(true);
    let rec_res = pos_force_emitter::emit_standalone_recovery_hold(&mut drv, None, -1.0, 0.30);
    assert!(
        matches!(rec_res, Err(TxRefusal::BuildRefused)),
        "emit_standalone_recovery_hold must refuse when external_pos_force_emitter is true, got {rec_res:?}"
    );
    assert_eq!(bus.tx_len(), 0, "standalone recovery must emit 0 frames under external owner");

    // 3. The single tick emitter succeeds and transmits exactly one 0x307 frame (both Target
    //    and Hold).
    drv.begin_tx_tick();
    let stamp = drv.current_j7_intent_stamp();
    let out_t = pos_force_emitter::emit_tick(
        &mut drv,
        PosForceIntent::target(-0.8, Some(0.30), None, stamp),
        stamp,
        ContinuityBypass::None,
    )
    .expect("emit_tick target");
    assert!(matches!(out_t, PosForceEmitOutcome::Sent { joint: 0, freeze: false, .. }));
    let tx_t = bus.tx.lock().unwrap().clone();
    assert_eq!(tx_t.len(), 1);
    assert_eq!(tx_t[0].id, 0x307);

    bus.clear_tx();
    drv.begin_tx_tick();
    let out_h = pos_force_emitter::emit_tick(
        &mut drv,
        PosForceIntent::hold(stamp),
        stamp,
        ContinuityBypass::None,
    )
    .expect("emit_tick hold");
    assert!(matches!(out_h, PosForceEmitOutcome::Sent { joint: 0, .. }));
    let tx_h = bus.tx.lock().unwrap().clone();
    assert_eq!(tx_h.len(), 1);
    assert_eq!(tx_h[0].id, 0x307);
    assert_eq!(u16::from_le_bytes([tx_h[0].data[4], tx_h[0].data[5]]), 0);
    assert_eq!(u16::from_le_bytes([tx_h[0].data[6], tx_h[0].data[7]]), 460);
}
