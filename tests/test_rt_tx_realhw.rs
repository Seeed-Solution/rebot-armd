//! Real-hardware TX timing, dual-owner (Legacy + RT Executor) carryover, `CAN_TX_STALLED`,
//! bounded stopped-hold, and control-gate integration tests (`fix/rt-tx-realhw`).

mod mit_support;

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use armd::diagnostics::tick::{reset_tx_metrics, take_tx_metrics};
use armd::driver::{
    is_estop_inhibited, ArmDriver, DamiaoArmDriver, JointTarget,
    TxBatchOutcome, TxBudget, DAMIAO_TX_FRAME_MAX_AGE, DAMIAO_TX_MAX_DURATION,
    DAMIAO_TX_MAX_FRAMES,
};
use armd::motion::{GripperCommand, GripperRequest};
use armd::protocol::damiao::DamiaoLimits;
use armd::rt_core::broker::{BootId, Broker, Capacities, ClientId, Submit, TransitionOp};
use armd::rt_core::ops::{CoreOps, PlanSlot, ReleaseFixture};
use armd::rt_core::urgent::{EstopGate, StopRequests};
use armd::safety::{FaultCode, FaultReason, SafetyState};
use armd::supervisor::HeldObject;
use armd::transport::MockClock;
use armd::{ArmConfig, SupervisorInner};
use mit_support::FakeBus;

const DT: f64 = 0.02;

fn ms(n: u64) -> Duration { Duration::from_millis(n) }
fn us(n: u64) -> Duration { Duration::from_micros(n) }

fn dm_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm()
        .damiao_ranges
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Owner {
    Legacy,
    Executor,
}

enum RigOwner {
    Legacy(Box<SupervisorInner>),
    Executor {
        fx: Box<ReleaseFixture<SupervisorInner>>,
        broker: Box<Broker<PlanSlot>>,
        next_req: u64,
    },
}

struct DualRig {
    owner: RigOwner,
    bus: FakeBus,
    clock: MockClock,
}

impl DualRig {
    fn new(kind: Owner, cfg: ArmConfig, budget: Option<TxBudget>) -> Self {
        let limits = dm_limits();
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&limits, clock.clone());
        bus.with(|s| {
            s.immediate_replies = true;
            s.follow_commands = true;
            s.push_all_feedback();
        });
        let tx_budget = budget.unwrap_or_else(|| cfg.damiao_tx_budget());
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits)
            .unwrap()
            .with_tx_budget(tx_budget);
        drv.read_joint_states().unwrap();
        drv.check_mapping_ranges();

        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
        inner.lease_enabled = false;
        inner.tick(DT);
        inner.set_torque(true).expect("initial arm");
        // Only run warmup output ticks when the configured budget can complete a full 7-joint batch
        // in one tick; constrained-budget tests (e.g. max_frames = 4) start armed immediately.
        if tx_budget.max_frames >= 7 {
            for _ in 0..3 {
                clock.advance(ms(20));
                bus.with(|s| s.push_all_feedback());
                inner.tick(DT);
            }
        }
        assert!(inner.safety.state().is_operational(), "{:?}", inner.safety.state());

        let owner = match kind {
            Owner::Legacy => RigOwner::Legacy(Box::new(inner)),
            Owner::Executor => {
                inner.rt_executor_owned = true;
                let (fx, broker, _tw, _stop) = ReleaseFixture::new(
                    inner,
                    BootId(77),
                    Capacities { commands: 8, plans: 4, replies: 16 },
                );
                RigOwner::Executor { fx: Box::new(fx), broker: Box::new(broker), next_req: 1 }
            }
        };
        bus.clear_log();
        reset_tx_metrics();
        Self { owner, bus, clock }
    }

    fn inner(&self) -> &SupervisorInner {
        match &self.owner {
            RigOwner::Legacy(inner) => inner,
            RigOwner::Executor { fx, .. } => &fx.core,
        }
    }

    fn inner_mut(&mut self) -> &mut SupervisorInner {
        match &mut self.owner {
            RigOwner::Legacy(inner) => inner,
            RigOwner::Executor { fx, .. } => &mut fx.core,
        }
    }

    fn step_dt(&mut self, dt: f64, push_feedback: bool) {
        self.clock.advance(Duration::from_secs_f64(dt));
        if push_feedback {
            self.bus.with(|s| s.push_all_feedback());
        }
        match &mut self.owner {
            RigOwner::Legacy(inner) => {
                inner.tick(dt);
            }
            RigOwner::Executor { fx, .. } => {
                fx.dt = dt;
                fx.step();
            }
        }
    }

    fn step(&mut self) {
        self.step_dt(DT, true);
    }

    fn clear_fault_and_enable(&mut self) {
        match &mut self.owner {
            RigOwner::Legacy(inner) => {
                inner.clear_fault().expect("legacy clear_fault");
                assert_eq!(*inner.safety.state(), SafetyState::Idle);
                self.clock.advance(ms(20));
                self.bus.with(|s| s.push_all_feedback());
                inner.set_torque(true).expect("legacy re-enable");
                for _ in 0..3 {
                    self.clock.advance(ms(20));
                    self.bus.with(|s| s.push_all_feedback());
                    inner.tick(DT);
                }
            }
            RigOwner::Executor { fx, broker, next_req } => {
                // ClearFault via the RT executor's budgeted maintenance op, then TorqueOn via broker.
                self.clock.advance(ms(20));
                self.bus.with(|s| s.push_all_feedback());
                let dl = armd::transport::DriverClock::now(&self.clock) + ms(200);
                fx.core.clear_fault_executor(dl).expect("executor clear_fault_executor");
                assert_eq!(*fx.core.safety.state(), SafetyState::Idle);

                self.clock.advance(ms(20));
                self.bus.with(|s| s.push_all_feedback());
                let auth = fx.core.authority(&fx.ctx(fx.now()));
                let id = *next_req;
                *next_req += 1;
                broker
                    .submit(
                        ClientId(1),
                        id,
                        auth,
                        fx.now() + Duration::from_secs(5),
                        Submit::Transition(TransitionOp::TorqueOn),
                    )
                    .expect("submit TorqueOn transition");
                let rep = fx.step();
                assert!(rep.command.as_ref().is_some_and(|(_, r)| r.is_ok()), "TorqueOn: {rep:?}");
                for _ in 0..5 {
                    self.clock.advance(ms(20));
                    self.bus.with(|s| s.push_all_feedback());
                    fx.step();
                }
            }
        }
        assert!(
            self.inner().safety.state().is_operational(),
            "expected operational after clear_fault + enable: {:?}",
            self.inner().safety.state()
        );
    }
}

fn base_cfg() -> ArmConfig {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.lease_timeout_ms = 600_000;
    cfg.workspace = None;
    cfg.collision = None;
    cfg.torque_guard = None;
    cfg.power_budget = None;
    cfg
}

/// Item 2 + Item 4:
/// 1. Deterministic frame-start comparison on the exact same realistic 0.45 ms/frame + 1 ms
///    periodic spike bus: 5 ms cutoff defers on spike ticks, whereas the 10 ms default completes
///    with 0 deferrals and 0 faults.
/// 2. Full 7-motor jog followed by home under 0.45 ms/frame + 1 ms periodic spikes completes
///    with 0 faults on BOTH owners (Legacy and Executor).
#[test]
fn dual_owner_7_motor_jog_and_home_with_realistic_tx_timing_and_budget_comparison() {
    // Part A: deterministic frame-start comparison (5 ms vs 10 ms) on 0.45 ms/frame + 1 ms spikes.
    // On a 7-motor batch where every 3rd frame incurs a +1 ms SPI spike:
    //   frame 0 -> 0.45 ms
    //   frame 1 -> 0.90 ms
    //   frame 2 -> 2.35 ms (spike)
    //   frame 3 -> 2.80 ms
    //   frame 4 -> 3.25 ms
    //   frame 5 -> 4.70 ms (spike)
    //   frame 6 -> starts at 4.70 ms, finishes at 5.15 ms -> if a 2-frame spike or 0.45 ms + 1 ms
    //   every 2nd frame runs: frame 0=0.45, frame 1=1.90, frame 2=2.35, frame 3=3.80,
    //   frame 4=4.25, frame 5=5.70 -> frame 6 start is 5.70 ms (>= 5 ms cutoff, < 10 ms cutoff)!
    for (budget_ms, expect_deferred) in [(5u64, true), (10u64, false)] {
        let limits = dm_limits();
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&limits, clock.clone());
        bus.with(|s| {
            s.immediate_replies = true;
            s.push_all_feedback();
        });
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits)
            .unwrap()
            .with_tx_budget(TxBudget {
                max_frames: DAMIAO_TX_MAX_FRAMES,
                max_duration: ms(budget_ms),
                frame_max_age: DAMIAO_TX_FRAME_MAX_AGE,
            });
        drv.read_joint_states().unwrap();
        drv.set_torque(true).unwrap();
        bus.with(|s| {
            s.tx_attempts = 0;
            s.tx_step = us(450);
            s.tx_spike_every = 2;
            s.tx_spike_step = ms(1);
        });
        drv.begin_tx_tick();
        reset_tx_metrics();
        let targets: Vec<JointTarget> = (0..7).map(|_| JointTarget::position(0.1)).collect();
        let out = drv.write_joint_targets_bounded(&targets).unwrap();
        if expect_deferred {
            assert_eq!(out, TxBatchOutcome::Deferred { pending: 1 }, "5 ms cutoff must defer frame 6");
        } else {
            assert_eq!(out, TxBatchOutcome::Complete, "10 ms default must complete 7 frames without deferring");
            assert_eq!(take_tx_metrics().deferred, 0);
        }
    }

    // Part B: 7-motor jog + home under realistic 0.45 ms/frame + periodic spikes on BOTH owners.
    // - 10 ms default with frequent 1 ms spikes (every 2nd frame): 0 deferrals, 0 faults.
    // - 5 ms budget with occasional 2.5 ms spikes (every 14th frame, i.e. every 2nd tick):
    //   defers transiently on spike ticks, drains on the next tick, and completes with 0 faults.
    for owner in [Owner::Legacy, Owner::Executor] {
        for (budget_ms, spike_every, spike_step) in [
            (10u64, 2usize, ms(1)),
            (5u64, 14usize, us(2_500)),
        ] {
            let mut cfg = base_cfg();
            cfg.tx_budget_ms = Some(budget_ms);
            let mut rig = DualRig::new(owner, cfg, None);
            rig.bus.with(|s| {
                s.tx_attempts = 0;
                s.tx_step = us(450);
                s.tx_spike_every = spike_every;
                s.tx_spike_step = spike_step;
            });

            // 1. 7-motor jog to nonzero pose across all 7 joints (including gripper).
            let jog_goal = vec![0.18, -0.15, -0.18, 0.20, -0.15, 0.22, -0.60];
            let epoch = rig.inner().safety.epoch_id();
            let (_, jog_id, _) = rig
                .inner_mut()
                .jog_action(jog_goal.clone(), Some(0.5), Some(epoch), "joint_move")
                .expect("start 7-motor jog");

            let mut jog_done = false;
            for _ in 0..80 {
                rig.step();
                assert!(
                    rig.inner().safety.state().is_operational(),
                    "{owner:?} budget={budget_ms}ms faulted during jog: {:?}",
                    rig.inner().safety.fault()
                );
                if rig.inner().last_action.as_ref().is_some_and(|a| a.id == jog_id && a.state == "done") {
                    jog_done = true;
                    break;
                }
            }
            assert!(jog_done, "{owner:?} budget={budget_ms}ms: 7-motor jog did not finish");
            for (j, &goal) in jog_goal.iter().enumerate() {
                assert!(
                    (rig.inner().measured_positions[j] - goal).abs() < 0.03,
                    "{owner:?} budget={budget_ms}ms J{}: measured={} vs goal={goal}",
                    j + 1,
                    rig.inner().measured_positions[j]
                );
            }

            // 2. Home back to [0; 7].
            let home_goal = vec![0.0; 7];
            let epoch = rig.inner().safety.epoch_id();
            let (_, home_id, _) = rig
                .inner_mut()
                .jog_action(home_goal.clone(), Some(0.5), Some(epoch), "home")
                .expect("start 7-motor home");

            let mut home_done = false;
            for _ in 0..80 {
                rig.step();
                assert!(
                    rig.inner().safety.state().is_operational(),
                    "{owner:?} budget={budget_ms}ms faulted during home: {:?}",
                    rig.inner().safety.fault()
                );
                if rig.inner().last_action.as_ref().is_some_and(|a| a.id == home_id && a.state == "done") {
                    home_done = true;
                    break;
                }
            }
            assert!(home_done, "{owner:?} budget={budget_ms}ms: 7-motor home did not finish");
            for (j, &goal) in home_goal.iter().enumerate() {
                assert!(
                    (rig.inner().measured_positions[j] - goal).abs() < 0.03,
                    "{owner:?} budget={budget_ms}ms J{}: measured={} vs home={goal}",
                    j + 1,
                    rig.inner().measured_positions[j]
                );
            }
            if budget_ms == 10 {
                assert_eq!(
                    take_tx_metrics().deferred,
                    0,
                    "{owner:?} 10 ms default should complete 0.45 ms/frame + 1 ms spike with 0 deferrals"
                );
            }
        }
    }
}

/// Item 1 + Item 4:
/// Sustained deferral beyond 45 ms (with preserved `queued_at` across continuous 20 ms target
/// replacement) or beyond `tx_stall_ticks` latches exactly ONE `CAN_TX_STALLED` on both owners,
/// with a message naming queued/dropped frames and age, and `epoch_id` stays constant on further
/// stalled ticks. Also verifies mid-drain head expiry breaks immediately (0 later motion sends).
#[test]
fn dual_owner_sustained_deferral_beyond_45ms_latches_single_can_tx_stalled() {
    for owner in [Owner::Legacy, Owner::Executor] {
        // (a) High tx_stall_ticks (100): proves 45 ms age limit fires even under continuous
        //     20 ms target coalescing (preserved `queued_at` prevents deadline renewal).
        let mut cfg = base_cfg();
        cfg.tx_stall_ticks = Some(100);
        let mut rig = DualRig::new(owner, cfg, Some(TxBudget {
            max_frames: 16,
            max_duration: ms(10),
            frame_max_age: ms(45),
        }));
        rig.bus.with(|s| s.tx_enobufs = true);

        // t = 20 ms (tick 1): queued at t=20 (age 0) -> Deferred, NOT faulted.
        rig.step();
        assert!(rig.inner().safety.state().is_operational(), "{owner:?}: tick 1 should defer");
        // t = 40 ms (tick 2): queued_at still t=20 (age 20 ms < 45 ms) -> Deferred, NOT faulted.
        rig.step();
        assert!(rig.inner().safety.state().is_operational(), "{owner:?}: tick 2 (20 ms) should defer");
        // t = 60 ms (tick 3): queued_at still t=20 (age 40 ms < 45 ms) -> Deferred, NOT faulted.
        rig.step();
        assert!(rig.inner().safety.state().is_operational(), "{owner:?}: tick 3 (40 ms) should defer");
        // t = 80 ms (tick 4): age 60 ms >= 45 ms -> Expired -> latches CAN_TX_STALLED!
        rig.step();
        assert_eq!(
            *rig.inner().safety.state(),
            SafetyState::Fault(FaultReason::CanTxStalled),
            "{owner:?}: age >= 45 ms must latch CanTxStalled"
        );
        let fault = rig.inner().safety.fault().expect("fault info");
        assert_eq!(fault.code, FaultCode::CanTxStalled);
        assert!(!fault.recoverable);
        assert!(
            fault.message.contains("expired") && fault.message.contains("ms") && fault.message.contains("frames"),
            "{owner:?}: fault message should name frames & age: {}",
            fault.message
        );
        let latched_epoch = rig.inner().safety.epoch_id();

        // Further stalled ticks while stopped must NOT re-latch or bump epoch_id.
        for _ in 0..8 {
            rig.step();
            assert_eq!(
                rig.inner().safety.epoch_id(),
                latched_epoch,
                "{owner:?}: stalled ticks after CAN_TX_STALLED must not re-latch"
            );
            assert_eq!(rig.inner().safety.fault().map(|f| f.code), Some(FaultCode::CanTxStalled));
        }

        // (b) Custom tx_stall_ticks = Some(2): latches CAN_TX_STALLED on exact tick 2.
        let mut cfg2 = base_cfg();
        cfg2.tx_stall_ticks = Some(2);
        let mut rig2 = DualRig::new(owner, cfg2, None);
        rig2.bus.with(|s| s.tx_enobufs = true);
        rig2.step();
        assert!(rig2.inner().safety.state().is_operational(), "{owner:?}: tick 1 < 2");
        rig2.step();
        assert_eq!(rig2.inner().safety.fault().map(|f| f.code), Some(FaultCode::CanTxStalled));
        let msg = rig2.inner().safety.fault().unwrap().message;
        assert!(msg.contains("queued") && msg.contains("2 consecutive ticks"), "{msg}");
    }

    // (c) Mid-drain head expiry followed by a younger fresh slot: breaks immediately on the
    //     first expired head and submits ZERO later motion frames in that pass.
    let limits = dm_limits();
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&limits, clock.clone());
    bus.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap()
        .with_tx_budget(TxBudget { max_frames: 4, max_duration: ms(10), frame_max_age: ms(45) });
    drv.read_joint_states().unwrap();
    drv.set_torque(true).unwrap();

    // Tick 1 (t=0): sends J1..J4, queues J5..J7 with queued_at = 0.
    drv.begin_tx_tick();
    assert_eq!(
        drv.write_joint_targets_bounded(&(0..7).map(|_| JointTarget::position(0.1)).collect::<Vec<_>>()).unwrap(),
        TxBatchOutcome::Deferred { pending: 3 }
    );
    // Tick 2 (t=20 ms): J5..J7 (at head, queued_at=0) + J1..J4 (pushed at back, queued_at=20 ms).
    // Give budget max_frames=1 and tx_step=25 ms so J5 transmits at t=20 -> finishes at t=45 ms!
    // Wait: raise max_frames=16 so after J5 finishes at t=45 ms, J6 (queued_at=0, age=45 ms)
    // hits DeadlinePassed -> MUST break immediately and NOT transmit J7 or fresh J1..J4!
    clock.advance(ms(20));
    bus.clear_log();
    bus.with(|s| s.tx_step = ms(25));
    drv.set_tx_budget(TxBudget { max_frames: 16, max_duration: ms(100), frame_max_age: ms(45) });
    drv.begin_tx_tick();
    let out = drv
        .write_joint_targets_bounded(&(0..7).map(|_| JointTarget::position(0.2)).collect::<Vec<_>>())
        .unwrap();
    // J6 hits DeadlinePassed in the loop (1) and J7 (also queued at t=0) expires in post-flush drop_stale (1);
    // fresh J1..J4 (queued at t=20 ms) are discarded with ZERO subsequent motion frames transmitted.
    assert_eq!(out, TxBatchOutcome::Expired { dropped: 2, pending: 0 }, "{out:?}");
    assert_eq!(drv.pending_tx_len(), 0);
    let sent_ids: Vec<u32> = bus.tx_log().into_iter().map(|f| f.id).collect();
    assert_eq!(sent_ids, vec![0x105], "mid-drain head expiry must send zero subsequent motion frames");
}

/// Item 3 + Item 4:
/// Stopped-hold deferral in Fault on BOTH owners:
/// (a) Partial deferral (max_frames = 4) at 50 Hz: 0 repeated faults, constant epoch, all 7 joints
///     receive fresh holds, and `clear_fault` + `enable` works cleanly.
/// (b) Slow-cadence (100 ms) partial progress (`sent_any == true`): `hold_cursor` rotates across
///     all 7 joints with 0 false `CAN_TX_STALLED`.
/// (c) Zero-progress hold (`tx_enobufs = true`) > 45 ms: latches `CAN_TX_STALLED` once, logs <= 1/s,
///     resumes holds once bus clears, and `clear_fault` + `enable` works cleanly.
/// (d) Fault precedence: zero-progress hold > 45 ms never overwrites `EStop`, `ThermalTrip`, or
///     `MitOwner::Lost`.
/// (e) Stop arriving mid-hold-tx drops `pending_hold_tx` carryover cleanly.
#[test]
fn dual_owner_stopped_hold_deferral_escalation_precedence_and_recovery() {
    for owner in [Owner::Legacy, Owner::Executor] {
        // (a) Partial hold deferral in Fault -> 0 repeated faults + clear_fault + enable succeeds.
        let mut rig = DualRig::new(
            owner,
            base_cfg(),
            Some(TxBudget { max_frames: 4, max_duration: ms(10), frame_max_age: ms(45) }),
        );
        rig.inner_mut()
            .safety
            .trigger_fault_coded(FaultCode::Other, FaultReason::Other("injected test fault".into()));
        let fault_epoch = rig.inner().safety.epoch_id();
        rig.bus.clear_log();
        for _ in 0..6 {
            rig.step();
            assert_eq!(rig.inner().safety.epoch_id(), fault_epoch, "{owner:?}: partial hold deferral must not re-latch");
            assert_eq!(rig.inner().safety.fault().map(|f| f.code), Some(FaultCode::Other));
        }
        // Every joint 1..=7 must have received at least one hold frame (0x101..=0x107).
        let ids: std::collections::HashSet<u32> = rig.bus.tx_log().into_iter().map(|f| f.id).collect();
        for j in 1u32..=7 {
            assert!(ids.contains(&(0x100 + j)), "{owner:?}: joint {j} starved during partial hold deferral");
        }
        // In Part (a), `max_frames = 4` stays active (only 4/7 frames per tick): verify
        // clear_fault transitions to Idle and enable returns to Armed/operational cleanly!
        match &mut rig.owner {
            RigOwner::Legacy(inner) => {
                inner.clear_fault().expect("legacy clear_fault");
                assert_eq!(*inner.safety.state(), SafetyState::Idle);
                rig.clock.advance(ms(20));
                rig.bus.with(|s| s.push_all_feedback());
                inner.set_torque(true).expect("legacy re-enable");
                assert!(inner.safety.state().is_operational(), "{:?}", inner.safety.state());
            }
            RigOwner::Executor { fx, broker, next_req } => {
                rig.clock.advance(ms(20));
                rig.bus.with(|s| s.push_all_feedback());
                let dl = armd::transport::DriverClock::now(&rig.clock) + ms(200);
                fx.core.clear_fault_executor(dl).expect("executor clear_fault_executor");
                assert_eq!(*fx.core.safety.state(), SafetyState::Idle);

                rig.clock.advance(ms(20));
                rig.bus.with(|s| s.push_all_feedback());
                let auth = fx.core.authority(&fx.ctx(fx.now()));
                let id = *next_req;
                *next_req += 1;
                broker
                    .submit(
                        ClientId(1),
                        id,
                        auth,
                        fx.now() + Duration::from_secs(5),
                        Submit::Transition(TransitionOp::TorqueOn),
                    )
                    .expect("submit TorqueOn");
                let rep = fx.step();
                assert!(rep.command.as_ref().is_some_and(|(_, r)| r.is_ok()), "{rep:?}");
                // Advance until TorqueOn confirms Armed (at most 10 releases; enable handshake
                // uses its own budgeted passes and does not run operational target ticks until Armed).
                for _ in 0..10 {
                    if fx.core.safety.state().is_operational() {
                        break;
                    }
                    rig.clock.advance(ms(20));
                    rig.bus.with(|s| s.push_all_feedback());
                    fx.step();
                }
                assert!(fx.core.safety.state().is_operational(), "{:?}", fx.core.safety.state());
            }
        }

        // (b) Slow-cadence (100 ms) partial progress: every tick sends 4 frames (sent_any == true)
        //     even though carried frames from 100 ms ago expire -> hold_cursor rotates through all
        //     7 joints with ZERO false CAN_TX_STALLED.
        let mut rig_slow = DualRig::new(
            owner,
            base_cfg(),
            Some(TxBudget { max_frames: 4, max_duration: ms(10), frame_max_age: ms(45) }),
        );
        rig_slow.inner_mut()
            .safety
            .trigger_fault_coded(FaultCode::Other, FaultReason::Other("slow-cadence hold test".into()));
        let slow_epoch = rig_slow.inner().safety.epoch_id();
        rig_slow.bus.clear_log();
        for _ in 0..4 {
            // Advance 90 ms (keeps feedback <= 100 ms fresh while exceeding 45 ms hold frame age!)
            rig_slow.step_dt(0.09, true);
            assert_eq!(
                rig_slow.inner().safety.epoch_id(),
                slow_epoch,
                "{owner:?}: slow-cadence partial hold progress must not falsely escalate"
            );
            assert_eq!(rig_slow.inner().safety.fault().map(|f| f.code), Some(FaultCode::Other));
        }
        let slow_ids: std::collections::HashSet<u32> = rig_slow.bus.tx_log().into_iter().map(|f| f.id).collect();
        for j in 1u32..=7 {
            assert!(slow_ids.contains(&(0x100 + j)), "{owner:?}: slow-cadence hold_cursor starved joint {j}");
        }

        // (c) Zero-progress hold > 45 ms -> latches CAN_TX_STALLED ONCE, resumes when bus clears,
        //     and clear_fault + enable recovers.
        let mut rig_stall = DualRig::new(owner, base_cfg(), None);
        rig_stall.inner_mut()
            .safety
            .trigger_fault_coded(FaultCode::Other, FaultReason::Other("initial fault".into()));
        let init_epoch = rig_stall.inner().safety.epoch_id();
        rig_stall.bus.with(|s| s.tx_enobufs = true);

        rig_stall.step(); // t = 20 (age 0)
        assert_eq!(rig_stall.inner().safety.epoch_id(), init_epoch);
        rig_stall.step(); // t = 40 (age 20)
        assert_eq!(rig_stall.inner().safety.epoch_id(), init_epoch);
        rig_stall.step(); // t = 60 (age 40)
        assert_eq!(rig_stall.inner().safety.epoch_id(), init_epoch);
        rig_stall.step(); // t = 80 (age 60 >= 45) -> escalates ONCE to CAN_TX_STALLED!
        assert_eq!(
            rig_stall.inner().safety.fault().map(|f| f.code),
            Some(FaultCode::CanTxStalled),
            "{owner:?}: zero-progress hold > 45 ms must latch CAN_TX_STALLED"
        );
        let stalled_epoch = rig_stall.inner().safety.epoch_id();
        assert_eq!(stalled_epoch, init_epoch + 1);

        // Next 10 ticks remain stalled: epoch must never advance again.
        for _ in 0..10 {
            rig_stall.step();
            assert_eq!(rig_stall.inner().safety.epoch_id(), stalled_epoch);
            assert_eq!(rig_stall.inner().safety.fault().map(|f| f.code), Some(FaultCode::CanTxStalled));
        }

        // Bus clears -> fresh holds resume immediately, and clear_fault + enable succeeds!
        rig_stall.bus.with(|s| s.tx_enobufs = false);
        rig_stall.bus.clear_log();
        rig_stall.step();
        let resumed_ids: std::collections::HashSet<u32> = rig_stall.bus.tx_log().into_iter().map(|f| f.id).collect();
        for j in 1u32..=7 {
            assert!(resumed_ids.contains(&(0x100 + j)), "{owner:?}: joint {j} did not resume hold after bus cleared");
        }
        rig_stall.clear_fault_and_enable();

        // (d) Fault precedence: EStop, ThermalTrip, and MitOwner::Lost are NEVER overwritten.
        for kind in ["estop", "thermal", "mit_lost"] {
            let mut rig_p = DualRig::new(owner, base_cfg(), None);
            match kind {
                "estop" => {
                    rig_p.inner_mut().e_stop_latched.store(true, Ordering::SeqCst);
                    rig_p.inner_mut().safety.trigger_e_stop();
                }
                "thermal" => {
                    rig_p.inner_mut().safety.trigger_fault(FaultReason::ThermalTrip {
                        joint_idx: 0,
                        temperature: 85.0,
                        limit: 70.0,
                    });
                }
                "mit_lost" => {
                    rig_p.inner_mut().mit.owner = armd::mit::MitOwner::Lost;
                    rig_p.inner_mut().safety.trigger_fault_coded(
                        FaultCode::BusLoss,
                        FaultReason::Other("MIT_LOST test".into()),
                    );
                }
                _ => unreachable!(),
            }
            let expected_state = rig_p.inner().safety.state().clone();
            let expected_epoch = rig_p.inner().safety.epoch_id();
            rig_p.bus.with(|s| s.tx_enobufs = true);
            for _ in 0..6 {
                rig_p.step();
                assert_eq!(
                    *rig_p.inner().safety.state(),
                    expected_state,
                    "{owner:?} {kind}: stopped-hold stall must not overwrite stronger fault"
                );
                assert_eq!(rig_p.inner().safety.epoch_id(), expected_epoch);
            }
        }
    }

    // (e) Urgent stop arriving mid-hold-tx drops pending_hold_tx cleanly.
    let limits = dm_limits();
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&limits, clock);
    bus.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap()
        .with_tx_budget(TxBudget { max_frames: 4, max_duration: ms(10), frame_max_age: ms(45) });
    drv.read_joint_states().unwrap();
    drv.set_torque(true).unwrap();
    let stops = StopRequests::new();
    let req = Arc::new(AtomicU64::new(0));
    let app = Arc::new(AtomicU64::new(0));
    drv.set_estop_gate(EstopGate::new(req.clone(), app).with_stops(stops.clone()));
    let hold_targets: Vec<JointTarget> = (0..7).map(|_| JointTarget::position(0.0)).collect();
    drv.begin_tx_tick();
    assert_eq!(drv.write_estop_hold_bounded(&hold_targets).unwrap(), TxBatchOutcome::Deferred { pending: 3 });
    assert_eq!(drv.pending_hold_tx_len(), 3);

    // Raise urgent stop during next hold transmit -> EstopInhibited + pending_hold_tx cleared.
    let req_cb = req.clone();
    bus.with(|s| {
        s.on_tx = Some(Arc::new(move || {
            req_cb.fetch_add(1, Ordering::SeqCst);
        }));
    });
    drv.begin_tx_tick();
    let err = drv.write_estop_hold_bounded(&hold_targets).unwrap_err();
    assert!(is_estop_inhibited(&err), "{err:?}");
    assert_eq!(drv.pending_hold_tx_len(), 0, "mid-hold urgent stop must discard pending_hold_tx");
}

/// Item 1 + Item 4 control-gate regression coverage on BOTH owners (tracking off & on):
/// - Deferred final gripper release waits for Complete before publishing `done` / clearing `releasing`
/// - Gravity-comp `Exiting` does not advance `exit_hold` or switch back on `Deferred` ticks
/// - Park `unload_settle` invalidates continuous stationary window on a `Deferred` output gap
#[test]
fn dual_owner_deferred_action_gravity_and_park_gates() {
    for owner in [Owner::Legacy, Owner::Executor] {
        // 1. Deferred final gripper release with tracking OFF (real B601-DM default).
        //    Measure exact `done_tick` on a clean run, then inject transient ENOBUFS on `done_tick`:
        //    release MUST stay `running` with `releasing == true` and `held.is_some()`, then commit
        //    `done` and clear `releasing`/`held` on the very next `Complete` tick!
        let open_rad = base_cfg().gripper_policy.as_ref().unwrap().open_rad;
        let make_release_rig = || {
            let mut r = DualRig::new(owner, base_cfg(), None);
            r.bus.with(|s| {
                s.motors[6].q = open_rad;
                s.push_all_feedback();
            });
            r.inner_mut().commanded_positions[6] = open_rad;
            r.inner_mut().measured_positions[6] = open_rad;
            r.inner_mut().held = Some(HeldObject { label: "cup".into(), spill_safe: false });
            r.inner_mut().releasing = true;
            let epoch = r.inner().safety.epoch_id();
            let (rel_id, _) = r
                .inner_mut()
                .start_gripper(
                    GripperRequest {
                        command: GripperCommand::Open,
                        fraction: None,
                        grasp: false,
                        speed: 5.0,
                        timeout_s: None,
                        tag: None,
                    },
                    epoch,
                )
                .expect("start release");
            (r, rel_id)
        };
        let done_tick = {
            let (mut probe, rel_id) = make_release_rig();
            (1..=40)
                .find(|_| {
                    probe.step();
                    probe.inner().last_action.as_ref().is_some_and(|a| a.id == rel_id && a.state == "done")
                })
                .expect("clean release reaches done")
        };
        let (mut rig_rel, rel_id) = make_release_rig();
        for _ in 1..done_tick {
            rig_rel.step();
            assert_eq!(rig_rel.inner().last_action.as_ref().unwrap().state, "running");
        }
        // Final tick: transient ENOBUFS deferral -> trajectory finishes into pending_deferred_done,
        // action stays running, releasing & held preserved.
        rig_rel.bus.with(|s| s.tx_enobufs = true);
        rig_rel.step();
        assert!(rig_rel.inner().safety.state().is_operational());
        assert!(rig_rel.inner().active_trajectory.is_none());
        assert_eq!(rig_rel.inner().last_action.as_ref().unwrap().id, rel_id);
        assert_eq!(rig_rel.inner().last_action.as_ref().unwrap().state, "running");
        assert!(rig_rel.inner().releasing, "{owner:?}: deferred final release must not clear releasing");
        assert!(rig_rel.inner().held.is_some(), "{owner:?}: deferred final release must not clear held");
        // Next tick: bus clears -> Complete -> commits `done` and clears releasing/held!
        rig_rel.bus.with(|s| s.tx_enobufs = false);
        rig_rel.step();
        assert_eq!(rig_rel.inner().last_action.as_ref().unwrap().state, "done");
        assert!(!rig_rel.inner().releasing);
        assert!(rig_rel.inner().held.is_none());

        // 2. Gravity-comp Exiting: deferred stiff holds must NOT advance exit_hold or switch back.
        let mut rig_gc = DualRig::new(owner, base_cfg(), None);
        rig_gc.inner_mut().enter_gravity_comp(Default::default()).expect("enter gravity comp");
        rig_gc.step();
        rig_gc.inner_mut().exit_gravity_comp("REQUESTED", "test");
        assert!(rig_gc.inner().gravity_comp.is_some());
        // Inject 2 transient deferred ticks (< 3 stall threshold): Exiting must not switch back
        // or advance elapsed_s.
        rig_gc.bus.with(|s| s.tx_enobufs = true);
        for _ in 0..2 {
            rig_gc.step();
            assert!(rig_gc.inner().safety.state().is_operational());
            match rig_gc.inner().gravity_comp.as_ref().map(|g| &g.phase) {
                Some(armd::gravity_comp::GravityCompPhase::Exiting { elapsed_s, .. }) => {
                    assert_eq!(*elapsed_s, 0.0, "{owner:?}: deferred stiff hold must not advance Exiting elapsed_s");
                }
                other => panic!("{owner:?}: expected Exiting with elapsed_s == 0, got {other:?}"),
            }
        }
        // Bus clears -> 6 Complete ticks (120 ms > 100 ms exit_hold_ms) complete handback cleanly!
        rig_gc.bus.with(|s| s.tx_enobufs = false);
        for _ in 0..6 {
            rig_gc.step();
        }
        assert!(rig_gc.inner().gravity_comp.is_none(), "{owner:?}: gravity comp should hand back after Complete ticks");
        assert!(rig_gc.inner().safety.state().is_operational());

        // 3. Park unload_settle: a Deferred output gap invalidates the continuous stationary window
        //    and does not cut torque on the deferred tick.
        let mut rig_pk = DualRig::new(owner, base_cfg(), None);
        rig_pk.inner_mut().pending_normal_unload = true;
        // Accumulate 10 Complete ticks (~200 ms) of resting-pose settle.
        for _ in 0..10 {
            rig_pk.step();
        }
        let held_before = rig_pk
            .inner()
            .unload_settle
            .as_ref()
            .expect("unload_settle active")
            .held_s;
        assert!(held_before > 0.1, "{owner:?}: expected positive held_s before gap, got {held_before}");
        // Inject a single Deferred tick -> continuous settle window MUST reset!
        rig_pk.bus.with(|s| s.tx_enobufs = true);
        rig_pk.step();
        assert!(rig_pk.inner().is_torqued, "{owner:?}: must not cut torque on deferred settle tick");
        let settle = rig_pk.inner().unload_settle.as_ref().expect("settle still active");
        assert_eq!(settle.held_s, 0.0, "{owner:?}: output gap must invalidate continuous settle held_s");
        assert!(settle.samples.is_empty(), "{owner:?}: output gap must clear continuous settle samples");
    }
}

/// Item 2 + Item 4: profile `tx_budget_ms` and `tx_stall_ticks` validation and `damiao_tx_budget()`.
#[test]
fn profile_tx_budget_and_stall_ticks_validation() {
    let dm = ArmConfig::rebot_dm();
    assert_eq!(dm.tx_budget_ms, Some(10));
    assert_eq!(dm.resolved_tx_stall_ticks(), 3);
    let b = dm.damiao_tx_budget();
    assert_eq!(b.max_duration, DAMIAO_TX_MAX_DURATION);
    assert_eq!(b.max_duration, ms(10));
    assert_eq!(b.max_frames, DAMIAO_TX_MAX_FRAMES);
    assert_eq!(b.frame_max_age, DAMIAO_TX_FRAME_MAX_AGE);

    let mut cfg = ArmConfig::rebot_dm();
    cfg.tx_budget_ms = Some(0);
    assert!(cfg.validate().is_err(), "tx_budget_ms = 0 must be rejected");
    cfg.tx_budget_ms = Some(21); // > 20 ms control period at 50 Hz
    assert!(cfg.validate().is_err(), "tx_budget_ms > period must be rejected");
    cfg.control_frequency_hz = 10; // 100 ms period, still capped by 45 ms frame max age
    cfg.tx_budget_ms = Some(46);
    assert!(cfg.validate().is_err(), "tx_budget_ms > 45 ms must be rejected");
    cfg.tx_budget_ms = Some(45);
    assert!(cfg.validate().is_ok());
    assert_eq!(cfg.damiao_tx_budget().max_duration, ms(45));

    let mut cfg2 = ArmConfig::rebot_dm();
    cfg2.tx_stall_ticks = Some(0);
    assert!(cfg2.validate().is_err(), "tx_stall_ticks = 0 must be rejected");
    cfg2.tx_stall_ticks = Some(4);
    assert!(cfg2.validate().is_ok());
    assert_eq!(cfg2.resolved_tx_stall_ticks(), 4);

    cfg2.tx_stall_ticks = None;
    cfg2.control_frequency_hz = 100; // period 10 ms -> ceil(45/10) = 5
    assert_eq!(cfg2.resolved_tx_stall_ticks(), 5);
    cfg2.control_frequency_hz = 25; // period 40 ms -> ceil(45/40) = 2
    assert_eq!(cfg2.resolved_tx_stall_ticks(), 2);
}
