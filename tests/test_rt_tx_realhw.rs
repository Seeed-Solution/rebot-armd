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
    DAMIAO_TX_MAX_FRAMES, DAMIAO_TX_RELEASE_PUBLISH_MARGIN,
};
use armd::motion::{GripperCommand, GripperRequest};
use armd::protocol::damiao::DamiaoLimits;
use armd::rt_core::broker::{BootId, Broker, Capacities, ClientId, Submit, TransitionOp};
use armd::rt_core::ops::{Checkpoint, CoreOps, PlanSlot, ReleaseFixture, VirtualClock};
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
            s.immediate_replies = false;
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
                // Drive BOTH FakeBus/DamiaoArmDriver and ReleaseFixture from ONE shared clock.
                let (fx, broker, _tw, _stop) = ReleaseFixture::new_with_clock(
                    inner,
                    BootId(77),
                    Capacities { commands: 8, plans: 4, replies: 16 },
                    VirtualClock::from_mock_clock(clock.clone()),
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
        let started = self.clock.now();
        let release_deadline = started + Duration::from_secs_f64(dt);
        if push_feedback {
            self.bus.with(|s| s.push_all_feedback());
        }
        match &mut self.owner {
            RigOwner::Legacy(inner) => {
                inner.tick_with_release_deadline(dt, self.clock.now(), release_deadline);
                self.clock.advance(release_deadline.saturating_duration_since(self.clock.now()));
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
                self.bus.with(|s| s.push_all_feedback());
                let dl = armd::transport::DriverClock::now(&self.clock) + ms(200);
                fx.core.clear_fault_executor(dl).expect("executor clear_fault_executor");
                assert_eq!(*fx.core.safety.state(), SafetyState::Idle);

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
    // Part A1: Direct driver 7 targets + 1 due 0xCC refresh (8 frames total, ~0.45 ms/frame +
    // 1 ms spike every 2nd frame) under 5 ms cutoff vs 10 ms default.
    // Make J1 silent during read_joint_states after advancing > refresh threshold so 0xCC refresh
    // consumes 1 attempt inside the same tick's TX pass before the 7 target frames.
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
        // Advance clock past the 150 ms feedback-refresh threshold so read_joint_states emits a
        // due 0xCC status refresh for J1 (keeping J2..J7 fresh via push_feedback).
        clock.advance(ms(160));
        bus.with(|s| {
            for i in 1..7 { s.push_feedback(i); }
            s.tx_log.clear();
            s.tx_attempts = 0;
            s.tx_step = us(450);
            s.tx_spike_every = 2;
            s.tx_spike_step = ms(1);
        });
        reset_tx_metrics();
        drv.read_joint_states().unwrap();
        let refresh_sent = bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data[2] == 0xCC);
        assert!(refresh_sent, "budget={budget_ms}ms: expected a due 0xCC refresh frame in this tick");
        let targets: Vec<JointTarget> = (0..7).map(|_| JointTarget::position(0.1)).collect();
        let out = drv.write_joint_targets_bounded(&targets).unwrap();
        if expect_deferred {
            assert!(
                matches!(out, TxBatchOutcome::Deferred { pending } if pending >= 1),
                "5 ms cutoff must defer with 1 due refresh + 7 targets, got {out:?}"
            );
            assert!(take_tx_metrics().deferred > 0, "5 ms cutoff must record deferred frames");
        } else {
            assert_eq!(out, TxBatchOutcome::Complete, "10 ms default must complete due refresh + 7 targets");
            assert_eq!(take_tx_metrics().deferred, 0);
        }
    }

    // Part A2: Isolated 11 ms in-flight frame tail (matching real B601-DM `max_tx` 11.381 ms)
    // started before the 10 ms cutoff finishes cleanly and rejects starting the next frame
    // (`Deferred { pending: 6 }`), then drains cleanly on the next tick with 0 faults.
    {
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
                max_duration: ms(10),
                frame_max_age: DAMIAO_TX_FRAME_MAX_AGE,
            });
        drv.read_joint_states().unwrap();
        drv.set_torque(true).unwrap();
        bus.with(|s| {
            s.tx_log.clear();
            s.tx_attempts = 0;
            s.tx_step = ms(11);
        });
        drv.begin_tx_tick();
        let targets: Vec<JointTarget> = (0..7).map(|_| JointTarget::position(0.1)).collect();
        let out = drv.write_joint_targets_bounded(&targets).unwrap();
        assert_eq!(
            out,
            TxBatchOutcome::Deferred { pending: 6 },
            "11 ms in-flight frame 0 must finish cleanly and reject starting frame 1"
        );
        assert_eq!(bus.tx_log().len(), 1, "only frame 0 started before 10 ms cutoff");
        // Next tick (normal 0.45 ms/frame): drains carried + fresh batch cleanly!
        clock.advance(ms(9));
        bus.with(|s| {
            s.tx_step = us(450);
            s.push_all_feedback();
        });
        drv.read_joint_states().unwrap();
        assert_eq!(drv.write_joint_targets_bounded(&targets).unwrap(), TxBatchOutcome::Complete);
    }

    // Part B: 7-motor jog + home under realistic 0.45 ms/frame + periodic spikes on BOTH owners.
    // - 10 ms default with frequent 1 ms spikes (every 2nd frame): 0 deferrals, 0 faults.
    // - 5 ms budget with occasional 2.5 ms spikes (every 14th frame, i.e. every 2nd tick):
    //   defers transiently on spike ticks, drains on the next tick, and completes with 0 faults.
    for owner in [Owner::Legacy, Owner::Executor] {
        for (budget_ms, spike_every, spike_step) in [
            (10u64, 2usize, ms(1)),
            (5u64, 10usize, us(2_500)),
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
            reset_tx_metrics();

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
            let m = take_tx_metrics();
            if budget_ms == 10 {
                assert_eq!(
                    m.deferred,
                    0,
                    "{owner:?} 10 ms default should complete 0.45 ms/frame + 1 ms spike with 0 deferrals"
                );
            } else {
                assert!(
                    m.deferred > 0,
                    "{owner:?} 5 ms budget with 2.5 ms spikes must actually exercise deferral carryover"
                );
            }
        }
    }

    // Part C: Exercise the real `ArmSupervisor::home` async entry point under 10 ms default with
    // 0.45 ms/frame + 1 ms spikes: completes back to `config.home_positions` with 0 faults.
    {
        let rt = tokio::runtime::Builder::new_current_thread().enable_time().build().unwrap();
        rt.block_on(async {
            let limits = dm_limits();
            let clock = MockClock::new();
            let bus = FakeBus::with_clock(&limits, clock.clone());
            bus.with(|s| {
                s.immediate_replies = true;
                s.follow_commands = true;
                s.push_all_feedback();
            });
            let cfg = base_cfg();
            let home_goal = cfg.home_positions.clone();
            let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
                .with_joint_limits(limits)
                .unwrap()
                .with_tx_budget(cfg.damiao_tx_budget());
            drv.read_joint_states().unwrap();
            drv.check_mapping_ranges();
            let sup = armd::ArmSupervisor::new(cfg).with_driver(Box::new(drv));
            {
                let mut inner = sup.inner().write().await;
                inner.lease_enabled = false;
                inner.tick(DT);
                inner.set_torque(true).expect("arm");
                inner.jog_action(vec![0.15, -0.1, -0.1, 0.1, 0.0, 0.0, 0.0], Some(0.2), None, "joint_move").unwrap();
                for _ in 0..20 {
                    clock.advance(ms(20));
                    bus.with(|s| s.push_all_feedback());
                    inner.tick(DT);
                }
            }
            bus.with(|s| {
                s.tx_attempts = 0;
                s.tx_step = us(450);
                s.tx_spike_every = 2;
                s.tx_spike_step = ms(1);
            });
            reset_tx_metrics();
            sup.home(Some(0.4), None).await.expect("ArmSupervisor::home");
            let mut home_done = false;
            for _ in 0..50 {
                clock.advance(ms(20));
                bus.with(|s| s.push_all_feedback());
                let mut inner = sup.inner().write().await;
                inner.tick(DT);
                assert!(inner.safety.state().is_operational(), "home faulted: {:?}", inner.safety.fault());
                if inner.last_action.as_ref().is_some_and(|a| a.kind == "home" && a.state == "done") {
                    home_done = true;
                    break;
                }
            }
            assert!(home_done, "ArmSupervisor::home did not finish");
            let inner = sup.inner().read().await;
            for (j, &goal) in home_goal.iter().enumerate() {
                assert!((inner.measured_positions[j] - goal).abs() < 0.03, "home J{} drifted", j + 1);
            }
            assert_eq!(take_tx_metrics().deferred, 0);
        });
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

        // (b) Custom tx_stall_ticks = Some(4) (minimum at 50 Hz = ceil(45/20) + 1) under per-motor
        //     starvation (max_frames = 1 so J5..J7 miss 4 consecutive ticks while frame_max_age = 150 ms
        //     prevents expiry before tick 4): latches CAN_TX_STALLED on exact tick 4.
        let mut cfg2 = base_cfg();
        cfg2.tx_stall_ticks = Some(4);
        let mut rig2 = DualRig::new(
            owner,
            cfg2,
            Some(TxBudget {
                max_frames: 1,
                max_duration: ms(10),
                frame_max_age: ms(150),
            }),
        );
        for tick in 1..=3 {
            rig2.step();
            assert!(
                rig2.inner().safety.state().is_operational(),
                "{owner:?}: partial deferral tick {tick} < 4 must stay operational"
            );
        }
        rig2.step();
        assert_eq!(rig2.inner().safety.fault().map(|f| f.code), Some(FaultCode::CanTxStalled));
        let msg = rig2.inner().safety.fault().unwrap().message;
        assert!(msg.contains("queued") && msg.contains("4 consecutive ticks"), "{msg}");
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
                rig.bus.with(|s| s.push_all_feedback());
                let dl = armd::transport::DriverClock::now(&rig.clock) + ms(200);
                fx.core.clear_fault_executor(dl).expect("executor clear_fault_executor");
                assert_eq!(*fx.core.safety.state(), SafetyState::Idle);

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

        // (c) Zero-progress hold > 45 ms -> latches CAN_TX_STALLED ONCE with a SINGLE log event
        //     on transition and <= 1/s log rate across 60 subsequent stalled ticks (1.2 s),
        //     resumes when bus clears, and clear_fault + enable recovers.
        let mut rig_stall = DualRig::new(owner, base_cfg(), None);
        rig_stall.inner_mut()
            .safety
            .trigger_fault_coded(FaultCode::Other, FaultReason::Other("initial fault".into()));
        let init_epoch = rig_stall.inner().safety.epoch_id();
        rig_stall.bus.with(|s| s.tx_enobufs = true);

        struct CountWarnErrorSubscriber(Arc<AtomicU64>);
        impl tracing::Subscriber for CountWarnErrorSubscriber {
            fn enabled(&self, meta: &tracing::Metadata<'_>) -> bool {
                *meta.level() <= tracing::Level::WARN
            }
            fn new_span(&self, _: &tracing::span::Attributes<'_>) -> tracing::span::Id {
                tracing::span::Id::from_u64(1)
            }
            fn record(&self, _: &tracing::span::Id, _: &tracing::span::Record<'_>) {}
            fn record_follows_from(&self, _: &tracing::span::Id, _: &tracing::span::Id) {}
            fn event(&self, event: &tracing::Event<'_>) {
                if *event.metadata().level() <= tracing::Level::WARN {
                    self.0.fetch_add(1, Ordering::SeqCst);
                }
            }
            fn enter(&self, _: &tracing::span::Id) {}
            fn exit(&self, _: &tracing::span::Id) {}
        }

        let log_events = Arc::new(AtomicU64::new(0));
        let sub = CountWarnErrorSubscriber(Arc::clone(&log_events));
        let stalled_epoch = tracing::subscriber::with_default(sub, || {
            rig_stall.step(); // t = 20 (age 0)
            assert_eq!(rig_stall.inner().safety.epoch_id(), init_epoch);
            rig_stall.step(); // t = 40 (age 20)
            assert_eq!(rig_stall.inner().safety.epoch_id(), init_epoch);
            rig_stall.step(); // t = 60 (age 40)
            assert_eq!(rig_stall.inner().safety.epoch_id(), init_epoch);
            assert_eq!(log_events.load(Ordering::SeqCst), 0, "{owner:?}: no stall logs before 45 ms");

            rig_stall.step(); // t = 80 (age 60 >= 45) -> escalates ONCE to CAN_TX_STALLED!
            assert_eq!(
                rig_stall.inner().safety.fault().map(|f| f.code),
                Some(FaultCode::CanTxStalled),
                "{owner:?}: zero-progress hold > 45 ms must latch CAN_TX_STALLED"
            );
            let stalled_epoch = rig_stall.inner().safety.epoch_id();
            assert_eq!(stalled_epoch, init_epoch + 1);
            assert_eq!(
                log_events.load(Ordering::SeqCst),
                1,
                "{owner:?}: first stopped-hold escalation must emit exactly 1 log event (no duplicate WARN+ERROR)"
            );

            // Next 60 stalled ticks (1.2 s on driver clock): epoch never advances again, and
            // rate-limited logs emit at most once more (~1.0 s after transition -> <= 2 total).
            for _ in 0..60 {
                rig_stall.step();
                assert_eq!(rig_stall.inner().safety.epoch_id(), stalled_epoch);
                assert_eq!(rig_stall.inner().safety.fault().map(|f| f.code), Some(FaultCode::CanTxStalled));
            }
            let total_events = log_events.load(Ordering::SeqCst);
            assert!(
                total_events <= 2,
                "{owner:?}: expected <= 2 stall log events across 1.2 s (<= 1/s), got {total_events}"
            );
            stalled_epoch
        });
        assert_eq!(stalled_epoch, init_epoch + 1);

        // Bus clears -> fresh holds resume immediately, and clear_fault + enable succeeds!
        rig_stall.bus.with(|s| s.tx_enobufs = false);
        rig_stall.bus.clear_log();
        rig_stall.step();
        let resumed_ids: std::collections::HashSet<u32> = rig_stall.bus.tx_log().into_iter().map(|f| f.id).collect();
        for j in 1u32..=7 {
            assert!(resumed_ids.contains(&(0x100 + j)), "{owner:?}: joint {j} did not resume hold after bus cleared");
        }
        rig_stall.clear_fault_and_enable();

        // (c2) Major 5: Hard transport errors (`tx_error = true`) while stopped with fresh RX:
        //      - Zero progress (`sent_any == false`) >= 45 ms escalates ONCE to CAN_TX_STALLED.
        //      - Partial submission (`fail_tx_after = Some(2)` on no-op flush bus, `sent_any == true`)
        //        resets the no-progress timer and does NOT escalate.
        {
            let mut rig_err = DualRig::new(owner, base_cfg(), None);
            rig_err.inner_mut()
                .safety
                .trigger_fault_coded(FaultCode::Other, FaultReason::Other("hard-error hold test".into()));
            let e0 = rig_err.inner().safety.epoch_id();
            // Partial submission: 2 frames accepted then hard error on frame 3 across 6 ticks (120 ms) ->
            // must NOT escalate to CAN_TX_STALLED because sent_any == true every tick!
            for _ in 0..6 {
                rig_err.bus.with(|s| s.fail_tx_after = Some(2));
                rig_err.step();
                assert_eq!(rig_err.inner().safety.epoch_id(), e0, "{owner:?}: partial hard-error submission must not escalate");
                assert_eq!(rig_err.inner().safety.fault().map(|f| f.code), Some(FaultCode::Other));
            }
            // Zero progress: every transmit returns hard error -> escalates ONCE after >= 45 ms!
            rig_err.bus.with(|s| {
                s.fail_tx_after = None;
                s.tx_error = true;
            });
            for _ in 0..3 {
                rig_err.step(); // t = 0, 20, 40 ms -> < 45 ms
                assert_eq!(rig_err.inner().safety.epoch_id(), e0);
            }
            rig_err.step(); // t = 60 ms >= 45 ms -> escalates ONCE!
            assert_eq!(
                rig_err.inner().safety.fault().map(|f| f.code),
                Some(FaultCode::CanTxStalled),
                "{owner:?}: persistent hard TX errors with 0 progress >= 45 ms must escalate once to CAN_TX_STALLED"
            );
            let e1 = rig_err.inner().safety.epoch_id();
            assert_eq!(e1, e0 + 1);
            for _ in 0..5 {
                rig_err.step();
                assert_eq!(rig_err.inner().safety.epoch_id(), e1, "{owner:?}: hard-error escalation must latch only once");
            }
        }

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

        // 4. Major 1: Macro contact guard (success-on-contact, `stop_code: None`) on a Deferred
        //    tick stops the trajectory and locks target to measured immediately, but withholds
        //    `SubState::Done` until the replacement hold completes (`Complete`).
        {
            let mut rig_mc = DualRig::new(owner, base_cfg(), None);
            let epoch = rig_mc.inner().safety.epoch_id();
            let id = rig_mc
                .inner_mut()
                .begin_orchestrated_action("grasp", None, epoch)
                .expect("begin macro");
            let mut max_res = [0.0f32; armd::rt_core::mailbox::MAX_DOF];
            max_res[1] = 2.0; // J2 contact threshold 2.0 N·m
            let guard = armd::macro_action::ContactGuard { max_residual_nm: max_res, stop_code: None };
            rig_mc.inner_mut().guard_next_macro_child(id, Some(guard));
            let mut goal = rig_mc.inner().commanded_positions.clone();
            goal[0] += 0.35;
            let limits = rig_mc.inner().planning_limits();
            rig_mc
                .inner_mut()
                .install_macro_quintic(id, goal, &limits)
                .expect("install macro child");
            let ticket = rig_mc.inner().macro_run.as_ref().unwrap().ticket().unwrap();
            rig_mc.step();
            assert_eq!(rig_mc.inner().macro_child_state(ticket), Some(armd::macro_action::SubState::Running));

            // Inject high J2 torque (creating residual > 2.0 N·m) AND transient ENOBUFS deferral!
            rig_mc.bus.with(|s| {
                s.motors[1].tau = 8.0;
                s.tx_enobufs = true;
            });
            rig_mc.step();
            assert!(rig_mc.inner().safety.state().is_operational());
            assert!(
                rig_mc.inner().active_trajectory.is_none(),
                "{owner:?}: contact guard must immediately stop descent trajectory even when TX defers"
            );
            assert_eq!(
                rig_mc.inner().macro_child_watch(ticket).and_then(|w| w.contact_joint),
                Some(1),
                "{owner:?}: contact_joint must be recorded on contact tick"
            );
            assert_eq!(
                rig_mc.inner().macro_child_state(ticket),
                Some(armd::macro_action::SubState::Running),
                "{owner:?}: SubState::Done must be withheld while replacement hold TX is Deferred"
            );
            // Bus clears -> replacement hold completes -> SubState::Done commits!
            rig_mc.bus.with(|s| s.tx_enobufs = false);
            rig_mc.step();
            assert!(
                matches!(rig_mc.inner().macro_child_state(ticket), Some(armd::macro_action::SubState::Done(_))),
                "{owner:?}: SubState::Done must commit once replacement hold completes, got {:?}",
                rig_mc.inner().macro_child_state(ticket)
            );
        }

        // 5. Major 2: Gravity-comp with tracking enabled + arm moving past `exit_hold_ms`:
        //    on the first stationary tick, if stiff hold TX returns `Deferred`, POS_VEL handback
        //    must NOT start before TX acceptance; once `Complete`, the dedicated transition
        //    release completes handback without resetting the active TX pass.
        {
            let mut jt = armd::model::JointTrackingConfig::rebot_dm();
            jt.enabled = true;
            let cfg_jt = ArmConfig {
                joint_tracking: Some(jt),
                ..base_cfg()
            };
            let mut rig_jt = DualRig::new(owner, cfg_jt, None);
            rig_jt.inner_mut().enter_gravity_comp(Default::default()).expect("enter gravity comp");
            rig_jt.step();
            rig_jt.inner_mut().exit_gravity_comp("REQUESTED", "major2");
            // Keep arm moving for 6 ticks (120 ms > 100 ms exit_hold_ms).
            rig_jt.bus.with(|s| {
                s.follow_commands = false;
                s.motors[0].v = 0.20;
            });
            for _ in 0..6 {
                rig_jt.step();
                assert!(rig_jt.inner().gravity_comp.is_some());
            }
            // First stationary tick, BUT TX returns Deferred -> must NOT hand back to POS_VEL!
            rig_jt.bus.with(|s| {
                s.motors[0].v = 0.0;
                s.tx_enobufs = true;
            });
            rig_jt.step();
            assert!(
                rig_jt.inner().gravity_comp.is_some(),
                "{owner:?}: stationary tick whose stiff hold deferred must NOT start POS_VEL handback"
            );
            assert!(rig_jt.inner().safety.state().is_operational());
            // Bus clears -> 1 Complete tick arms handback + 1 dedicated transition release completes it.
            rig_jt.bus.with(|s| s.tx_enobufs = false);
            rig_jt.step();
            assert!(rig_jt.inner().gravity_comp.is_some(), "{owner:?}: handback waits for scheduled transition release");
            rig_jt.step();
            assert!(rig_jt.inner().gravity_comp.is_none(), "{owner:?}: scheduled transition release completes handback");
            assert!(rig_jt.inner().safety.state().is_operational());
        }

        // 6. Major 8: Off-tick gravity exit deferral must NOT count toward periodic `tx_stall_ticks`,
        //    and an intervening unattempted release (`TickOutput::None` with no pending motion)
        //    resets the periodic deferral streak.
        {
            let mut cfg_s2 = base_cfg();
            cfg_s2.tx_stall_ticks = Some(4); // minimum at 50 Hz = ceil(45/20) + 1
            let mut rig_s2 = DualRig::new(owner, cfg_s2, None);
            rig_s2.inner_mut().enter_gravity_comp(Default::default()).expect("enter gravity comp");
            rig_s2.step();
            // Inject ENOBUFS during off-tick `exit_gravity_comp` -> immediate write defers,
            // must NOT increment periodic `tx_deferral_streak`!
            rig_s2.bus.with(|s| s.tx_enobufs = true);
            rig_s2.inner_mut().exit_gravity_comp("REQUESTED", "major8");
            // Clear the off-tick pending queue so periodic ticks start fresh from tick 1 without
            // hitting 45 ms expiry early; if off-tick exit had counted as streak 1, then 3 periodic
            // deferred releases would reach streak 4 and trip CAN_TX_STALLED.
            rig_s2.inner().driver.as_ref().unwrap().lock().unwrap().discard_pending_tx();
            for tick in 1..=3 {
                rig_s2.step();
                assert!(
                    rig_s2.inner().safety.state().is_operational(),
                    "{owner:?}: off-tick exit deferral + {tick} control releases must not trip tx_stall_ticks=4"
                );
            }
            // Bus clears on next tick after discarding aged frames -> recovers cleanly!
            rig_s2.inner().driver.as_ref().unwrap().lock().unwrap().discard_pending_tx();
            rig_s2.bus.with(|s| s.tx_enobufs = false);
            for _ in 0..7 {
                rig_s2.step();
            }
            assert!(rig_s2.inner().gravity_comp.is_none());
            assert!(rig_s2.inner().safety.state().is_operational());

            // Also verify `TickOutput::None` with no pending motion resets an active streak:
            // with tracking enabled, `recovery_suspended = true` produces `TickOutput::None`
            // while torqued and operational. 2 deferred releases (streak = 2) -> discard pending &
            // skip output (resets streak to 0) -> 3 deferred releases (ages 0, 20, 40 ms; streak = 3 < 4)
            // MUST stay operational (without reset, streak would hit 4 on release 2)!
            let mut jt = armd::model::JointTrackingConfig::rebot_dm();
            jt.enabled = true;
            let mut cfg_none = base_cfg();
            cfg_none.tx_stall_ticks = Some(4);
            cfg_none.joint_tracking = Some(jt);
            let mut rig_none = DualRig::new(owner, cfg_none, None);
            rig_none.bus.with(|s| s.tx_enobufs = true);
            rig_none.step(); // streak = 1 (< 4)
            rig_none.step(); // streak = 2 (< 4)
            assert!(rig_none.inner().safety.state().is_operational());
            if let Some(d) = rig_none.inner().driver.as_ref() {
                d.lock().unwrap().discard_pending_tx();
            }
            rig_none.inner_mut().tracking.recovery_suspended = true;
            rig_none.inner_mut().tracking.recovery_joints.push((0, std::time::Instant::now()));
            // Advance 20 ms without pushing fresh feedback (age = 20 ms > recovery since.elapsed()
            // while still < 250 ms BusLoss), so recovery stays suspended -> TickOutput::None!
            rig_none.step_dt(0.02, false);
            rig_none.inner_mut().tracking.recovery_joints.clear();
            rig_none.inner_mut().tracking.recovery_suspended = false;
            for tick in 1..=3 {
                rig_none.step(); // streak = 1, 2, 3 (< 4; ages 0, 20, 40 ms < 45 ms)
                assert!(
                    rig_none.inner().safety.state().is_operational(),
                    "{owner:?}: TickOutput::None without pending motion must reset tx_deferral_streak (tick {tick}): {:?}",
                    rig_none.inner().safety.fault()
                );
            }
        }
    }
}

/// Item 2 + Item 4: profile `tx_budget_ms` and `tx_stall_ticks` validation and `damiao_tx_budget()`.
#[test]
fn profile_tx_budget_and_stall_ticks_validation() {
    let dm = ArmConfig::rebot_dm();
    assert_eq!(dm.tx_budget_ms, Some(10));
    assert_eq!(dm.resolved_tx_budget_ms(), 10);
    assert_eq!(dm.min_tx_stall_ticks(), 4);
    assert_eq!(dm.resolved_tx_stall_ticks(), 4);
    let b = dm.damiao_tx_budget();
    assert_eq!(b.max_duration, DAMIAO_TX_MAX_DURATION);
    assert_eq!(b.max_duration, ms(10));
    assert_eq!(b.max_frames, DAMIAO_TX_MAX_FRAMES);
    assert_eq!(b.frame_max_age, DAMIAO_TX_FRAME_MAX_AGE);

    let mut cfg = ArmConfig::rebot_dm();
    cfg.tx_budget_ms = Some(0);
    assert!(cfg.validate().is_err(), "tx_budget_ms = 0 must be rejected");
    cfg.tx_budget_ms = Some(11); // > period/2 (10 ms) at 50 Hz
    assert!(cfg.validate().is_err(), "tx_budget_ms > period/2 (11 ms at 50 Hz) must be rejected");
    cfg.tx_budget_ms = Some(10); // == period/2 (10 ms) at 50 Hz
    assert!(cfg.validate().is_ok(), "tx_budget_ms = period/2 (10 ms at 50 Hz) must pass");
    cfg.control_frequency_hz = 200; // 5 ms period -> period/2 = 2 ms
    cfg.tx_budget_ms = None; // omitted default (10 ms) exceeds period/2 -> must be rejected!
    assert!(cfg.validate().is_err(), "200 Hz profile omitting tx_budget_ms (effective 10 ms > 2 ms) must be rejected");
    cfg.tx_budget_ms = Some(10);
    assert!(cfg.validate().is_err(), "200 Hz profile with explicit tx_budget_ms = 10 must be rejected");
    cfg.tx_budget_ms = Some(3);
    assert!(cfg.validate().is_err(), "200 Hz profile with tx_budget_ms = 3 (> period/2 = 2 ms) must be rejected");
    cfg.tx_budget_ms = Some(2);
    assert!(cfg.validate().is_ok(), "200 Hz profile with tx_budget_ms = 2 (== period/2) must pass");
    assert_eq!(cfg.damiao_tx_budget().max_duration, ms(2));
    cfg.control_frequency_hz = 1000; // 1 ms period -> period/2 = 0 ms -> rejected
    cfg.tx_budget_ms = Some(1);
    assert!(cfg.validate().is_err(), "1000 Hz profile (period/2 < 1 ms) must be rejected");
    cfg.control_frequency_hz = 10; // 100 ms period, period/2 = 50 ms, still capped by 45 ms frame max age
    cfg.tx_budget_ms = Some(46);
    assert!(cfg.validate().is_err(), "tx_budget_ms > 45 ms must be rejected");
    cfg.tx_budget_ms = Some(45);
    assert!(cfg.validate().is_ok());
    assert_eq!(cfg.damiao_tx_budget().max_duration, ms(45));

    let mut cfg2 = ArmConfig::rebot_dm();
    for bad in [0, 1, 2, 3] {
        cfg2.tx_stall_ticks = Some(bad);
        let err = cfg2.validate().unwrap_err();
        assert!(
            err.contains("tx_stall_ticks") && err.contains(">= 4"),
            "50 Hz tx_stall_ticks={bad} must be rejected with clear error, got: {err}"
        );
    }
    cfg2.tx_stall_ticks = Some(4);
    assert!(cfg2.validate().is_ok());
    assert_eq!(cfg2.resolved_tx_stall_ticks(), 4);
    cfg2.tx_stall_ticks = Some(7);
    assert!(cfg2.validate().is_ok());
    assert_eq!(cfg2.resolved_tx_stall_ticks(), 7);

    cfg2.tx_stall_ticks = None;
    cfg2.control_frequency_hz = 100; // period 10 ms -> ceil(45/10) + 1 = 6
    cfg2.tx_budget_ms = Some(5);
    assert_eq!(cfg2.min_tx_stall_ticks(), 6);
    assert_eq!(cfg2.resolved_tx_stall_ticks(), 6);
    cfg2.tx_stall_ticks = Some(5);
    assert!(cfg2.validate().is_err(), "100 Hz tx_stall_ticks = 5 (< 6) must be rejected");
    cfg2.tx_stall_ticks = Some(6);
    assert!(cfg2.validate().is_ok());

    cfg2.tx_stall_ticks = None;
    cfg2.control_frequency_hz = 25; // period 40 ms -> ceil(45/40) + 1 = 3
    cfg2.tx_budget_ms = Some(10);
    assert_eq!(cfg2.min_tx_stall_ticks(), 3);
    assert_eq!(cfg2.resolved_tx_stall_ticks(), 3);
    cfg2.tx_stall_ticks = Some(2);
    assert!(cfg2.validate().is_err(), "25 Hz tx_stall_ticks = 2 (< 3) must be rejected");
    cfg2.tx_stall_ticks = Some(3);
    assert!(cfg2.validate().is_ok());

    cfg2.tx_stall_ticks = None;
    cfg2.control_frequency_hz = 10; // period 100 ms -> ceil(45/100) + 1 = 2
    cfg2.tx_budget_ms = Some(10);
    assert_eq!(cfg2.min_tx_stall_ticks(), 2);
    assert_eq!(cfg2.resolved_tx_stall_ticks(), 2);
    cfg2.tx_stall_ticks = Some(1);
    let err_10hz = cfg2.validate().unwrap_err();
    assert!(
        err_10hz.contains("tx_stall_ticks") && err_10hz.contains(">= 2") && err_10hz.contains("10 Hz"),
        "10 Hz tx_stall_ticks = 1 must be rejected with clear error, got: {err_10hz}"
    );
    cfg2.tx_stall_ticks = Some(2);
    assert!(cfg2.validate().is_ok());
}

/// B3 + TX-realhw integration: `FaultKind::CanTxStalled` latched via `SafetyManager::latch`
/// enforces its `clear_check` rule on both Legacy and RT Executor owners — refusing while the
/// driver's TX queue has pending frames or feedback is stale, and clearing once drained + fresh.
#[test]
fn dual_owner_can_tx_stalled_clear_check_requires_drained_tx_queue_and_fresh_feedback() {
    for owner in [Owner::Legacy, Owner::Executor] {
        let mut rig = DualRig::new(owner, base_cfg(), None);
        rig.bus.with(|s| s.tx_enobufs = true);
        // Drive 4 stalled ticks (80 ms > 45 ms) to latch FaultKind::CanTxStalled via SafetyManager::latch.
        for _ in 0..4 {
            rig.step();
        }
        assert!(rig.inner().safety.has_latched_fault_kind(armd::safety::FaultKind::CanTxStalled));
        assert_eq!(rig.inner().safety.fault_code(), Some(FaultCode::CanTxStalled));
        // Run one stopped tick while still backpressured so pending_hold_tx is non-empty.
        rig.step();
        assert!(
            rig.inner().driver.as_ref().unwrap().lock().unwrap().pending_tx_len_total() > 0,
            "{owner:?}: expected pending hold TX frames while bus is backpressured"
        );

        // 1. Fresh feedback, but TX queue NOT drained -> MUST refuse with TxQueueNotDrained!
        match &mut rig.owner {
            RigOwner::Legacy(inner) => {
                let err = inner.clear_fault().unwrap_err().to_string();
                assert!(
                    err.contains("CAN TX queue still has pending frames"),
                    "{owner:?}: expected TxQueueNotDrained refusal, got: {err}"
                );
            }
            RigOwner::Executor { fx, .. } => {
                let dl = armd::transport::DriverClock::now(&rig.clock) + ms(200);
                let err = fx.core.clear_fault_executor_bounded(dl).unwrap_err();
                assert_eq!(
                    err,
                    armd::safety::ClearFaultError::Rejected(armd::safety::ClearRefusal::TxQueueNotDrained),
                    "{owner:?}: expected bounded TxQueueNotDrained refusal"
                );
            }
        }
        assert!(rig.inner().safety.has_latched_fault_kind(armd::safety::FaultKind::CanTxStalled));

        // 2. Bus recovers and drains pending hold TX on next tick, then advance clock past 250 ms
        //    without new feedback -> drained queue, stale feedback -> MUST refuse with FreshFeedbackRequired!
        rig.bus.with(|s| s.tx_enobufs = false);
        rig.step();
        assert_eq!(
            rig.inner().driver.as_ref().unwrap().lock().unwrap().pending_tx_len_total(),
            0,
            "{owner:?}: expected TX queue drained after bus recovery step"
        );
        rig.bus.with(|s| {
            s.immediate_replies = false;
            s.rx.clear();
        });
        rig.clock.advance(ms(400));
        match &mut rig.owner {
            RigOwner::Legacy(inner) => {
                let err = inner.clear_fault().unwrap_err().to_string();
                assert!(
                    err.contains("Fresh feedback is required"),
                    "{owner:?}: expected FreshFeedbackRequired refusal, got: {err}"
                );
            }
            RigOwner::Executor { fx, .. } => {
                let dl = armd::transport::DriverClock::now(&rig.clock) + ms(200);
                let err = fx.core.clear_fault_executor_bounded(dl).unwrap_err();
                assert_eq!(
                    err,
                    armd::safety::ClearFaultError::Rejected(armd::safety::ClearRefusal::FreshFeedbackRequired),
                    "{owner:?}: expected bounded FreshFeedbackRequired refusal"
                );
            }
        }

        // 3. Push fresh feedback + drained TX queue -> clear_fault succeeds and resets latched kinds!
        rig.step();
        rig.clear_fault_and_enable();
        assert_eq!(rig.inner().safety.latched_fault_kinds(), 0);
    }
}

/// Round-2 Item 1:
/// The TX start-cutoff is bounded by `min(first_tx + tx_budget, release_deadline - margin)` on
/// BOTH owners (Legacy and RT Executor), driven from ONE shared clock:
/// 1. Executor `ReleaseFixture` and `FakeBus`/`DamiaoArmDriver` share the exact same clock instance.
/// 2. When output starts late in a 20 ms release (+15 ms, with 10 ms `tx_budget_ms` and 2 ms
///    publish margin), TX stops at `release_deadline - 2 ms` (+18 ms, sending only 3 of 7 frames at
///    1 ms/frame) instead of running past the release, carries over the remaining 4 frames, and
///    completes them cleanly on the next release with 0 faults.
/// 3. When output starts at/past `release_deadline - 2 ms` (+19 ms), 0 frames start in that
///    release and all 7 frames carry over cleanly to the next release.
#[test]
fn dual_owner_late_output_start_stops_tx_at_release_deadline_and_carries_over() {
    assert_eq!(DAMIAO_TX_RELEASE_PUBLISH_MARGIN, ms(2));
    for owner in [Owner::Legacy, Owner::Executor] {
        let mut rig = DualRig::new(owner, base_cfg(), None);
        if let RigOwner::Executor { fx, .. } = &rig.owner {
            assert_eq!(fx.now(), rig.clock.now(), "executor and driver must share one clock");
            rig.clock.advance(ms(5));
            assert_eq!(fx.now(), rig.clock.now(), "advancing shared clock must advance executor clock identically");
        }

        // Part A: Output starts 15 ms into a 20 ms release (tx_budget = 10 ms, publish margin = 2 ms).
        // Effective cutoff is min(15 + 10, 20 - 2) = 18 ms -> at 1 ms/frame, exactly 3 frames start
        // (at 15, 16, 17 ms -> ending at 18 ms), and the remaining 4 frames carry over.
        rig.bus.clear_log();
        rig.bus.with(|s| {
            s.tx_step = ms(1);
            s.tx_spike_every = 0;
            s.tx_spike_step = Duration::ZERO;
            s.push_all_feedback();
        });
        let release_start = rig.clock.now();
        let release_deadline = release_start + ms(20);
        let cutoff = release_deadline - DAMIAO_TX_RELEASE_PUBLISH_MARGIN;
        let tx_end_observed: std::time::Instant;
        match &mut rig.owner {
            RigOwner::Legacy(inner) => {
                rig.clock.advance(ms(15));
                inner.tick_with_release_deadline(DT, rig.clock.now(), release_deadline);
                tx_end_observed = rig.clock.now();
                rig.clock.advance(release_deadline.saturating_duration_since(rig.clock.now()));
            }
            RigOwner::Executor { fx, .. } => {
                let late_clock = rig.clock.clone();
                let armed = Arc::new(std::sync::atomic::AtomicBool::new(true));
                let armed_cb = Arc::clone(&armed);
                fx.on_checkpoint(move |cp| {
                    if cp == Checkpoint::BeforeOutput && armed_cb.swap(false, Ordering::SeqCst) {
                        late_clock.advance(ms(15));
                    }
                });
                fx.dt = DT;
                // Record the exact clock after `advance_output` via bus tx timestamping:
                // 3 frames × 1 ms starting at +15 ms finishes at +18 ms (never past cutoff).
                fx.step();
                // Re-clear hook for subsequent releases.
                fx.on_checkpoint(|_| {});
                tx_end_observed = release_start + ms(15) + ms(rig.bus.tx_log().len() as u64);
                assert_eq!(fx.now(), release_deadline, "Executor release must end cleanly on release_deadline");
            }
        }
        let sent_late = rig.bus.tx_log();
        assert_eq!(
            sent_late.len(),
            3,
            "{owner:?}: late output start (+15 ms in 20 ms release, 2 ms margin) must stop after 3 frames, sent {:?}",
            sent_late.iter().map(|f| f.id).collect::<Vec<_>>()
        );
        assert!(
            tx_end_observed <= cutoff,
            "{owner:?}: TX must stop at release_deadline - margin ({cutoff:?}), observed {tx_end_observed:?}"
        );
        assert_eq!(
            rig.inner().driver.as_ref().unwrap().lock().unwrap().pending_motion_tx_len(),
            4,
            "{owner:?}: remaining 4 frames must carry over in pending_tx"
        );
        assert!(rig.inner().safety.state().is_operational(), "{owner:?}: single late release must not fault");

        // Next on-time release drains all 4 carried-over frames + fresh batch cleanly!
        rig.bus.clear_log();
        rig.bus.with(|s| s.tx_step = Duration::ZERO);
        rig.step();
        assert_eq!(
            rig.inner().driver.as_ref().unwrap().lock().unwrap().pending_motion_tx_len(),
            0,
            "{owner:?}: next on-time release must drain carried-over frames"
        );
        assert!(rig.inner().safety.state().is_operational(), "{owner:?}: recovery release must stay operational");

        // Part B: Output starts at +19 ms (past release_deadline - 2 ms = +18 ms) ->
        // 0 frames start in that release, all 7 frames carry over, next release drains all 7 cleanly.
        rig.bus.clear_log();
        rig.bus.with(|s| {
            s.tx_step = ms(1);
            s.push_all_feedback();
        });
        let release2_start = rig.clock.now();
        let release2_deadline = release2_start + ms(20);
        match &mut rig.owner {
            RigOwner::Legacy(inner) => {
                rig.clock.advance(ms(19));
                inner.tick_with_release_deadline(DT, rig.clock.now(), release2_deadline);
                rig.clock.advance(release2_deadline.saturating_duration_since(rig.clock.now()));
            }
            RigOwner::Executor { fx, .. } => {
                let late_clock = rig.clock.clone();
                let armed = Arc::new(std::sync::atomic::AtomicBool::new(true));
                let armed_cb = Arc::clone(&armed);
                fx.on_checkpoint(move |cp| {
                    if cp == Checkpoint::BeforeOutput && armed_cb.swap(false, Ordering::SeqCst) {
                        late_clock.advance(ms(19));
                    }
                });
                fx.dt = DT;
                fx.step();
                fx.on_checkpoint(|_| {});
            }
        }
        assert_eq!(
            rig.bus.tx_log().len(),
            0,
            "{owner:?}: output starting past release_deadline - margin must transmit 0 frames"
        );
        assert_eq!(
            rig.inner().driver.as_ref().unwrap().lock().unwrap().pending_motion_tx_len(),
            7,
            "{owner:?}: all 7 frames must carry over when output starts past TX cutoff"
        );
        assert!(rig.inner().safety.state().is_operational());

        rig.bus.clear_log();
        rig.bus.with(|s| s.tx_step = Duration::ZERO);
        rig.step();
        assert_eq!(
            rig.inner().driver.as_ref().unwrap().lock().unwrap().pending_motion_tx_len(),
            0
        );
        assert_eq!(rig.bus.tx_log().len(), 7);
        assert!(rig.inner().safety.state().is_operational());
    }
}

/// Round-2 Item 2:
/// `CAN_TX_STALLED` clear MUST prove that TX works on BOTH owners (Legacy and RT Executor):
/// 1. Right after latching `CAN_TX_STALLED`, pending motion frames were discarded (empty software
///    queue) and feedback is fresh, yet `ENOBUFS` persists -> clear MUST be refused.
/// 2. When `ENOBUFS` clears, a `0xCC` refresh/status probe is transmitted with no backpressure or
///    error, but without post-probe feedback (`immediate_replies = false`) -> clear MUST be
///    refused with `TxProbeFeedbackRequired`.
/// 3. If `ENOBUFS` returns after a probe was sent, the probe is invalidated and clear is refused.
/// 4. When `ENOBUFS` is gone and fresh feedback arrives strictly after the probe -> clear passes
///    to `Idle` and re-enable succeeds.
#[test]
fn dual_owner_can_tx_stalled_clear_requires_tx_probe_and_post_probe_feedback() {
    for owner in [Owner::Legacy, Owner::Executor] {
        let mut rig = DualRig::new(owner, base_cfg(), None);
        rig.bus.with(|s| s.tx_enobufs = true);
        for _ in 0..8 {
            rig.step();
            if rig.inner().safety.has_latched_fault_kind(armd::safety::FaultKind::CanTxStalled) {
                break;
            }
        }
        assert!(rig.inner().safety.has_latched_fault_kind(armd::safety::FaultKind::CanTxStalled));
        assert_eq!(rig.inner().safety.fault_code(), Some(FaultCode::CanTxStalled));
        let latched_epoch = rig.inner().safety.epoch_id();

        // Stall handler discarded pending motion frames -> software queue is empty right now,
        // and feedback from the latching tick is fresh, while ENOBUFS still persists!
        assert_eq!(
            rig.inner().driver.as_ref().unwrap().lock().unwrap().pending_tx_len_total(),
            0,
            "{owner:?}: software TX queue is empty immediately after stall latch"
        );

        // 1. ENOBUFS persists on empty queue + fresh feedback -> clear MUST be refused!
        rig.bus.clear_log();
        match &mut rig.owner {
            RigOwner::Legacy(inner) => {
                let err = inner.clear_fault().unwrap_err().to_string();
                assert!(
                    err.contains("ENOBUFS") || err.contains("Transmit queue full"),
                    "{owner:?}: expected ENOBUFS refusal while TX is still failing, got: {err}"
                );
            }
            RigOwner::Executor { fx, .. } => {
                let dl = armd::transport::DriverClock::now(&rig.clock) + ms(200);
                let err = fx.core.clear_fault_executor_bounded(dl).unwrap_err();
                assert!(
                    matches!(
                        err,
                        armd::safety::ClearFaultError::Rejected(armd::safety::ClearRefusal::DriverError(_))
                    ),
                    "{owner:?}: expected bounded DriverError(ENOBUFS) refusal, got: {err:?}"
                );
                assert!(
                    err.to_string().contains("ENOBUFS"),
                    "{owner:?}: expected ENOBUFS in refusal text, got: {err}"
                );
            }
        }
        assert!(rig.inner().safety.has_latched_fault_kind(armd::safety::FaultKind::CanTxStalled));
        assert_eq!(rig.inner().safety.epoch_id(), latched_epoch);

        // 2. ENOBUFS gone, but NO feedback reply arrives after the probe (motor 0 silent):
        //    probe transmits a 0xCC refresh frame (0x7FF) cleanly, yet clear MUST be refused with
        //    TxProbeFeedbackRequired until fresh feedback arrives strictly after that probe.
        rig.bus.clear_log();
        rig.bus.with(|s| {
            s.tx_enobufs = false;
            s.motors[0].silent = true;
        });
        match &mut rig.owner {
            RigOwner::Legacy(inner) => {
                let err = inner.clear_fault().unwrap_err().to_string();
                assert!(
                    err.contains("CAN TX probe sent; wait for fresh feedback received after the probe"),
                    "{owner:?}: expected TxProbeFeedbackRequired refusal, got: {err}"
                );
            }
            RigOwner::Executor { fx, .. } => {
                let dl = armd::transport::DriverClock::now(&rig.clock) + ms(200);
                let err = fx.core.clear_fault_executor_bounded(dl).unwrap_err();
                assert_eq!(
                    err,
                    armd::safety::ClearFaultError::Rejected(armd::safety::ClearRefusal::TxProbeFeedbackRequired),
                    "{owner:?}: expected bounded TxProbeFeedbackRequired refusal"
                );
            }
        }
        let probe_frames = rig.bus.tx_log();
        assert!(
            probe_frames.iter().any(|f| f.id == 0x7FF && f.data[2] == 0xCC),
            "{owner:?}: expected 0xCC refresh probe frame on 0x7FF, got: {probe_frames:?}"
        );
        assert!(rig.inner().safety.has_latched_fault_kind(armd::safety::FaultKind::CanTxStalled));

        // 3. If ENOBUFS returns on a subsequent tick, the earlier pending probe is invalidated and
        //    clearing (even after discarding pending frames with fresh feedback) is refused again.
        rig.bus.with(|s| {
            s.motors[0].silent = false;
            s.tx_enobufs = true;
        });
        rig.step(); // stopped-hold hits ENOBUFS -> invalidates probe
        rig.inner().driver.as_ref().unwrap().lock().unwrap().discard_pending_tx();
        match &mut rig.owner {
            RigOwner::Legacy(inner) => {
                let err = inner.clear_fault().unwrap_err().to_string();
                assert!(err.contains("ENOBUFS"), "{owner:?}: expected ENOBUFS after probe invalidation, got: {err}");
            }
            RigOwner::Executor { fx, .. } => {
                let dl = armd::transport::DriverClock::now(&rig.clock) + ms(200);
                let err = fx.core.clear_fault_executor_bounded(dl).unwrap_err();
                assert!(err.to_string().contains("ENOBUFS"), "{owner:?}: expected ENOBUFS, got: {err}");
            }
        }

        // 4. ENOBUFS gone + replies active -> probe transmits cleanly, fresh feedback arrives
        //    strictly after the post-write empty boundary, clear_fault passes to Idle and re-enable succeeds!
        rig.bus.clear_log();
        rig.bus.with(|s| {
            s.tx_enobufs = false;
            s.motors[0].silent = false;
            s.immediate_replies = false;
        });
        rig.step();
        rig.clear_fault_and_enable();
        assert_eq!(rig.inner().safety.latched_fault_kinds(), 0);
        assert!(rig.inner().safety.state().is_operational());
    }
}

/// Round-2 Item 3:
/// At 10 Hz (period = 100 ms), `min_tx_stall_ticks() == default == ceil(45/100) + 1 = 2`:
/// - `tx_stall_ticks = 1` is rejected at profile load (`validate()`).
/// - A single deferral does NOT latch `CAN_TX_STALLED` on either Legacy or RT Executor.
/// - A sustained deferral latches `CAN_TX_STALLED` on both owners (both under the 45 ms age limit
///   and under the `N = 2` consecutive-tick threshold).
#[test]
fn dual_owner_10hz_single_deferral_does_not_latch_sustained_does() {
    let mut check_cfg = base_cfg();
    check_cfg.control_frequency_hz = 10;
    check_cfg.tx_stall_ticks = Some(1);
    assert!(
        check_cfg.validate().is_err(),
        "10 Hz profile with tx_stall_ticks = 1 must be rejected at profile load"
    );
    check_cfg.tx_stall_ticks = None;
    assert!(check_cfg.validate().is_ok());
    assert_eq!(check_cfg.resolved_tx_stall_ticks(), 2);

    const DT_10HZ: f64 = 0.1;
    for owner in [Owner::Legacy, Owner::Executor] {
        // (a) Standard 10 Hz with default 45 ms frame_max_age:
        //     Tick 1 deferral (age 0 ms, streak 1 < 2) MUST NOT latch;
        //     Tick 2 sustained deferral MUST latch CAN_TX_STALLED.
        let mut cfg = base_cfg();
        cfg.control_frequency_hz = 10;
        cfg.tx_stall_ticks = None;
        let mut rig = DualRig::new(owner, cfg, None);
        rig.bus.with(|s| s.tx_enobufs = true);

        rig.step_dt(DT_10HZ, true);
        assert!(
            rig.inner().safety.state().is_operational(),
            "{owner:?}: single deferral at 10 Hz must NOT latch CAN_TX_STALLED: {:?}",
            rig.inner().safety.fault()
        );
        assert_eq!(
            rig.inner().driver.as_ref().unwrap().lock().unwrap().pending_motion_tx_len(),
            7,
            "{owner:?}: deferred frames should remain queued after single 10 Hz deferral"
        );

        rig.step_dt(DT_10HZ, true);
        assert_eq!(
            *rig.inner().safety.state(),
            SafetyState::Fault(FaultReason::CanTxStalled),
            "{owner:?}: sustained deferral at 10 Hz must latch CAN_TX_STALLED"
        );
        assert_eq!(rig.inner().safety.fault_code(), Some(FaultCode::CanTxStalled));

        // (b) Isolate consecutive-tick threshold N = 2 at 10 Hz (frame_max_age = 150 ms > 100 ms
        //     period): single deferral recovers cleanly on next tick; two consecutive deferrals
        //     latch CAN_TX_STALLED on exact tick 2 ("2 consecutive ticks").
        let mut cfg_streak = base_cfg();
        cfg_streak.control_frequency_hz = 10;
        cfg_streak.tx_stall_ticks = None;
        let mut rig_s = DualRig::new(
            owner,
            cfg_streak,
            Some(TxBudget {
                max_frames: DAMIAO_TX_MAX_FRAMES,
                max_duration: ms(10),
                frame_max_age: ms(150),
            }),
        );
        // Single transient deferral followed by recovery -> stays operational with 0 faults!
        rig_s.bus.with(|s| s.tx_enobufs = true);
        rig_s.step_dt(DT_10HZ, true);
        assert!(
            rig_s.inner().safety.state().is_operational(),
            "{owner:?}: single transient deferral at 10 Hz must stay operational"
        );
        rig_s.bus.with(|s| s.tx_enobufs = false);
        rig_s.step_dt(DT_10HZ, true);
        assert!(
            rig_s.inner().safety.state().is_operational(),
            "{owner:?}: recovery tick after single 10 Hz deferral must stay operational"
        );
        assert_eq!(
            rig_s.inner().driver.as_ref().unwrap().lock().unwrap().pending_motion_tx_len(),
            0
        );

        // Sustained 2-tick deferral -> tick 1 stays operational, tick 2 latches CAN_TX_STALLED!
        rig_s.bus.with(|s| s.tx_enobufs = true);
        rig_s.step_dt(DT_10HZ, true);
        assert!(
            rig_s.inner().safety.state().is_operational(),
            "{owner:?}: 1st of 2 consecutive 10 Hz deferrals must not latch"
        );
        rig_s.step_dt(DT_10HZ, true);
        assert_eq!(
            rig_s.inner().safety.fault_code(),
            Some(FaultCode::CanTxStalled),
            "{owner:?}: 2nd consecutive 10 Hz deferral must latch CAN_TX_STALLED"
        );
        let msg = rig_s.inner().safety.fault().unwrap().message;
        assert!(
            msg.contains("2 consecutive ticks"),
            "{owner:?}: expected '2 consecutive ticks' in stall message, got: {msg}"
        );
    }
}

/// Round-3 Item 1:
/// Healthy TX deferring every tick when output starts late (+16 ms at 50 Hz, 0.4 ms/frame cost):
/// (a) Carry-over frames are sent FIRST on the next pass, oldest first, with their coalesced
///     target VALUE updated in place without moving them to the back.
/// (b) Stall detection counts per-motor lack of progress (not "any deferral in the tick"):
///     every motor receives a target at least every 2 ticks and NO `CAN_TX_STALLED` latches
///     across 24 consecutive late ticks on BOTH owners (Legacy and RT Executor).
/// (c) A truly blocked bus (`tx_enobufs = true`) still latches `CAN_TX_STALLED`.
#[test]
fn late_release_healthy_deferral_prioritizes_carryover_and_never_false_faults() {
    // Part A: Direct driver verification of oldest-first carry-over priority and in-place value
    // coalescing (queued target value updated in place without moving to the back).
    let limits = dm_limits();
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&limits, clock.clone());
    bus.with(|s| {
        s.immediate_replies = true;
        s.tx_step = Duration::from_micros(400);
        s.push_all_feedback();
    });
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap()
        .with_tx_budget(TxBudget {
            max_frames: DAMIAO_TX_MAX_FRAMES,
            max_duration: ms(10),
            frame_max_age: ms(45),
        });
    drv.read_joint_states().unwrap();
    drv.set_torque(true).unwrap();
    bus.clear_log();

    let decode_pos = |frame: &armd::transport::CanFrame| -> f32 {
        f32::from_le_bytes([frame.data[0], frame.data[1], frame.data[2], frame.data[3]])
    };

    // Tick 1 (t = 16 ms, cutoff = 18 ms -> 2.0 ms -> 5 frames at 0.4 ms/frame):
    // sends J1..J5 (0x101..=0x105), defers J6 & J7 (0x106, 0x107).
    clock.advance(ms(16));
    drv.set_tx_release_deadline(Some(clock.now() + ms(2)));
    drv.begin_tx_tick();
    let t1: Vec<JointTarget> = (0..7).map(|j| JointTarget::position(0.10 + 0.01 * j as f32)).collect();
    assert_eq!(drv.write_joint_targets_bounded(&t1).unwrap(), TxBatchOutcome::Deferred { pending: 2 });
    assert_eq!(drv.last_target_tx_sent_mask(), 0b001_1111);
    let log1 = bus.tx_log();
    assert_eq!(
        log1.iter().map(|f| f.id).collect::<Vec<_>>(),
        vec![0x101, 0x102, 0x103, 0x104, 0x105]
    );

    // Tick 2 (+20 ms later, t = 36 ms, cutoff = 38 ms):
    // J6 & J7 MUST go out FIRST (oldest carry-over first, NOT moved to the back) with their
    // UPDATED coalesced target values (0.25, 0.26), followed by J1..J3 (0x101..=0x103).
    bus.clear_log();
    clock.advance(ms(18)); // 18 ms -> 36 ms
    bus.with(|s| s.push_all_feedback());
    drv.read_joint_states().unwrap();
    drv.set_tx_release_deadline(Some(clock.now() + ms(2)));
    drv.begin_tx_tick();
    let t2: Vec<JointTarget> = (0..7).map(|j| JointTarget::position(0.20 + 0.01 * j as f32)).collect();
    assert_eq!(drv.write_joint_targets_bounded(&t2).unwrap(), TxBatchOutcome::Deferred { pending: 2 });
    assert_eq!(drv.last_target_tx_sent_mask(), 0b110_0111);
    let log2 = bus.tx_log();
    assert_eq!(
        log2.iter().map(|f| f.id).collect::<Vec<_>>(),
        vec![0x106, 0x107, 0x101, 0x102, 0x103],
        "carry-over frames J6, J7 must transmit first before new targets J1..J3"
    );
    assert!((decode_pos(&log2[0]) - 0.25).abs() < 1e-4, "J6 must carry coalesced value 0.25, got {}", decode_pos(&log2[0]));
    assert!((decode_pos(&log2[1]) - 0.26).abs() < 1e-4, "J7 must carry coalesced value 0.26, got {}", decode_pos(&log2[1]));

    // Tick 3 (+20 ms later, t = 56 ms, cutoff = 58 ms):
    // Carry-over J4, J5 MUST go out FIRST with coalesced values (0.33, 0.34), followed by J1..J3.
    bus.clear_log();
    clock.advance(ms(18)); // 38 ms -> 56 ms
    bus.with(|s| s.push_all_feedback());
    drv.read_joint_states().unwrap();
    drv.set_tx_release_deadline(Some(clock.now() + ms(2)));
    drv.begin_tx_tick();
    let t3: Vec<JointTarget> = (0..7).map(|j| JointTarget::position(0.30 + 0.01 * j as f32)).collect();
    assert_eq!(drv.write_joint_targets_bounded(&t3).unwrap(), TxBatchOutcome::Deferred { pending: 2 });
    assert_eq!(drv.last_target_tx_sent_mask(), 0b001_1111);
    let log3 = bus.tx_log();
    assert_eq!(
        log3.iter().map(|f| f.id).collect::<Vec<_>>(),
        vec![0x104, 0x105, 0x101, 0x102, 0x103],
        "carry-over frames J4, J5 must transmit first before new targets J1..J3"
    );
    assert!((decode_pos(&log3[0]) - 0.33).abs() < 1e-4, "J4 must carry coalesced value 0.33, got {}", decode_pos(&log3[0]));
    assert!((decode_pos(&log3[1]) - 0.34).abs() < 1e-4, "J5 must carry coalesced value 0.34, got {}", decode_pos(&log3[1]));

    // Part B: Full dual-owner (Legacy + Executor) 50 Hz loop with realistic-delay bus:
    // Output starts at +16 ms EVERY tick with 0.4 ms/frame cost -> 5 frames/tick, 2 deferred/tick.
    // Every motor receives a target at least every 2 ticks and NO stall latches across 24 ticks.
    for owner in [Owner::Legacy, Owner::Executor] {
        let mut cfg = base_cfg();
        cfg.control_frequency_hz = 50;
        cfg.tx_stall_ticks = Some(4);
        let mut rig = DualRig::new(owner, cfg, None);
        rig.bus.with(|s| {
            s.tx_step = Duration::from_micros(400);
            s.tx_spike_every = 0;
            s.tx_spike_step = Duration::ZERO;
        });

        let mut last_seen_tick: [Option<usize>; 7] = [None; 7];
        let mut max_gap: [usize; 7] = [0; 7];
        let mut send_counts: [usize; 7] = [0; 7];

        if let RigOwner::Executor { fx, .. } = &mut rig.owner {
            let late_clock = rig.clock.clone();
            fx.on_checkpoint(move |cp| {
                if cp == Checkpoint::BeforeOutput {
                    late_clock.advance(ms(16));
                }
            });
        }

        const LATE_TICKS: usize = 24;
        for tick in 1..=LATE_TICKS {
            rig.bus.clear_log();
            rig.bus.with(|s| s.push_all_feedback());
            let release_start = rig.clock.now();
            let release_deadline = release_start + ms(20);
            match &mut rig.owner {
                RigOwner::Legacy(inner) => {
                    rig.clock.advance(ms(16));
                    inner.tick_with_release_deadline(DT, rig.clock.now(), release_deadline);
                    rig.clock.advance(release_deadline.saturating_duration_since(rig.clock.now()));
                }
                RigOwner::Executor { fx, .. } => {
                    fx.dt = DT;
                    fx.step();
                }
            }
            assert!(
                rig.inner().safety.state().is_operational(),
                "{owner:?}: healthy late-release deferral on tick {tick} must NOT latch CAN_TX_STALLED: {:?}",
                rig.inner().safety.fault()
            );
            let tx_frames: Vec<u32> = rig
                .bus
                .tx_log()
                .into_iter()
                .filter(|f| (0x101..=0x107).contains(&f.id))
                .map(|f| f.id)
                .collect();
            assert_eq!(
                tx_frames.len(),
                5,
                "{owner:?}: tick {tick} (+16 ms start, 2 ms before cutoff, 0.4 ms/frame) must send 5 frames, got {tx_frames:?}"
            );
            for fid in tx_frames {
                let j = (fid - 0x101) as usize;
                if let Some(prev) = last_seen_tick[j] {
                    max_gap[j] = max_gap[j].max(tick - prev);
                } else {
                    max_gap[j] = max_gap[j].max(tick);
                }
                last_seen_tick[j] = Some(tick);
                send_counts[j] += 1;
            }
        }

        for j in 0..7 {
            assert!(
                send_counts[j] >= LATE_TICKS / 2,
                "{owner:?}: motor {} received too few targets ({send_counts:?})",
                j + 1
            );
            assert!(
                max_gap[j] <= 2,
                "{owner:?}: motor {} gap exceeded 2 ticks (max_gap={max_gap:?})",
                j + 1
            );
            assert!(
                LATE_TICKS - last_seen_tick[j].unwrap() <= 1,
                "{owner:?}: motor {} did not receive a target within the last 2 ticks",
                j + 1
            );
        }

        // Part C: A bus that is truly blocked STILL latches CAN_TX_STALLED.
        rig.bus.with(|s| s.tx_enobufs = true);
        let mut latched = false;
        for _ in 0..6 {
            rig.bus.with(|s| s.push_all_feedback());
            let release_start = rig.clock.now();
            let release_deadline = release_start + ms(20);
            match &mut rig.owner {
                RigOwner::Legacy(inner) => {
                    rig.clock.advance(ms(16));
                    inner.tick_with_release_deadline(DT, rig.clock.now(), release_deadline);
                    rig.clock.advance(release_deadline.saturating_duration_since(rig.clock.now()));
                }
                RigOwner::Executor { fx, .. } => {
                    fx.dt = DT;
                    fx.step();
                }
            }
            if rig.inner().safety.fault_code() == Some(FaultCode::CanTxStalled) {
                latched = true;
                break;
            }
        }
        assert!(
            latched,
            "{owner:?}: truly blocked bus (ENOBUFS) must still latch CAN_TX_STALLED, got {:?}",
            rig.inner().safety.state()
        );
    }
}

/// Round-3 Item 2:
/// Legacy `ArmSupervisor::control_tick_with_deadline` caps TX to the caller's absolute scheduled
/// release grid deadline (`release_deadline - margin`), NOT `started + period - margin`.
#[tokio::test]
async fn legacy_control_tick_with_deadline_uses_scheduled_grid_not_started_plus_period() {
    let limits = dm_limits();
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&limits, clock.clone());
    bus.with(|s| {
        s.immediate_replies = true;
        s.follow_commands = true;
        s.tx_step = ms(1);
        s.push_all_feedback();
    });
    let cfg = base_cfg();
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap()
        .with_tx_budget(cfg.damiao_tx_budget());
    drv.read_joint_states().unwrap();
    drv.check_mapping_ranges();

    let sup = armd::supervisor::ArmSupervisor::new(cfg).with_driver(Box::new(drv));
    {
        let mut inner = sup.inner().write().await;
        inner.lease_enabled = false;
        inner.tick(DT);
        inner.set_torque(true).expect("initial arm");
        for _ in 0..3 {
            clock.advance(ms(20));
            bus.with(|s| s.push_all_feedback());
            inner.tick(DT);
        }
        assert!(inner.safety.state().is_operational());
    }

    // Case A: Scheduled grid deadline is only 1 ms away (<= 2 ms publish margin), as after waking
    // at +19 ms into a 20 ms slot. `started + period` would give 18 ms and send all 7 frames;
    // the scheduled grid deadline leaves 0 ms and MUST transmit 0 frames while deferring all 7.
    bus.clear_log();
    clock.advance(ms(19));
    bus.with(|s| s.push_all_feedback());
    let past_cutoff_grid = std::time::Instant::now() + ms(1);
    sup.control_tick_with_deadline(DT, past_cutoff_grid).await;
    assert_eq!(
        bus.tx_log().len(),
        0,
        "scheduled grid deadline within publish margin must transmit 0 frames, not started + period"
    );

    // Case B: Next on-time grid deadline (20 ms away -> ~18 ms TX window) drains all 7 frames
    // cleanly at 1 ms/frame with 0 faults.
    bus.clear_log();
    clock.advance(ms(1));
    bus.with(|s| s.push_all_feedback());
    let on_time_grid = std::time::Instant::now() + ms(20);
    sup.control_tick_with_deadline(DT, on_time_grid).await;
    assert_eq!(
        bus.tx_log().len(),
        7,
        "on-time scheduled grid deadline must drain all 7 frames"
    );

    // Case C: Scheduled grid deadline 4 ms away (<= 2 ms TX budget after 2 ms margin, at 1 ms/frame):
    // transmits at most 2 frames (< 7), proving partial capping tracks the scheduled grid deadline.
    bus.clear_log();
    clock.advance(ms(16));
    bus.with(|s| s.push_all_feedback());
    let partial_grid = std::time::Instant::now() + ms(4);
    sup.control_tick_with_deadline(DT, partial_grid).await;
    let partial_sent = bus.tx_log().len();
    assert!(
        partial_sent <= 2,
        "scheduled grid deadline +4 ms (<= 2 ms after publish margin, 1 ms/frame) must cap TX to <= 2 frames, sent {partial_sent}"
    );
    let obs = sup.observation().await.expect("observation");
    assert!(
        obs.fault.is_none(),
        "scheduled-grid partial deferral must not fault: {:?}",
        obs.fault
    );
}

/// Round-3 Item 3:
/// Every TX in a release uses the same margin-capped deadline (`release_deadline - margin`):
/// (a) Enable-transition commit (Step 3) and enable-transition polling (Step 2 + sub-release wake)
///     stop transmitting at `release_deadline - margin` and carry remaining enable frames cleanly.
/// (b) `probe_tx_ready_inner` preserves an active `tx_release_deadline` cap (does not clear it),
///     and Step 2b `clear_fault_executor_bounded_for_release` refuses without transmitting inside
///     the publish margin window (`now >= release_deadline - margin`).
#[test]
fn enable_transition_polling_and_clear_fault_probe_obey_release_margin_cap() {
    use armd::rt_core::ops::{CoreOps, ServiceBudget};

    // Part A: Enable-transition commit (Step 3), Step 2 polling, and sub-release wake all obey
    // `release_deadline - margin`.
    let mut rig = DualRig::new(Owner::Executor, base_cfg(), None);
    // Start from torque off (Idle) so we can exercise an enable handshake under late releases.
    rig.inner_mut().set_torque(false).expect("torque off before enable test");
    for _ in 0..4 {
        if !rig.inner().disable_pending {
            break;
        }
        rig.step();
    }
    assert!(!rig.inner().disable_pending, "disable must confirm before late TorqueOn test");
    rig.bus.with(|s| {
        s.tx_step = ms(1);
        s.tx_spike_every = 0;
        s.tx_spike_step = Duration::ZERO;
        s.push_all_feedback();
    });

    // Submit TorqueOn via the RT broker; Step 3 runs at +16 ms into a 20 ms release ->
    // 2 ms left before `release_deadline - 2 ms` -> at 1 ms/frame, at most 2 enable frames start
    // (NOT all 7!) and the transition remains pending.
    let late_clock = rig.clock.clone();
    let armed = Arc::new(std::sync::atomic::AtomicBool::new(true));
    let armed_cb = Arc::clone(&armed);
    rig.bus.clear_log();
    if let RigOwner::Executor { fx, broker, next_req } = &mut rig.owner {
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
        fx.on_checkpoint(move |cp| {
            if cp == Checkpoint::BeforeCommit && armed_cb.swap(false, Ordering::SeqCst) {
                late_clock.advance(ms(16));
            }
        });
        fx.dt = DT;
        let rep = fx.step();
        fx.on_checkpoint(|_| {});
        assert!(rep.command.as_ref().is_some_and(|(_, r)| r.is_ok()), "TorqueOn commit: {rep:?}");
        assert!(
            fx.core.torque_transition_pending(),
            "late Step 3 TorqueOn commit must cap enable TX and keep transition pending"
        );
    }
    let step3_enables: Vec<u32> = rig
        .bus
        .tx_log()
        .into_iter()
        .filter(|f| f.data == [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
        .map(|f| f.id)
        .collect();
    assert!(
        step3_enables.len() <= 2,
        "late Step 3 (+16 ms, 2 ms margin, 1 ms/frame) must send <= 2 enable frames, sent {step3_enables:?}"
    );

    // Step 2 enable-transition polling starting at +19 ms (past `release_deadline - 2 ms = +18 ms`):
    // MUST transmit 0 enable frames and keep the transition pending without faulting!
    rig.bus.clear_log();
    rig.bus.with(|s| s.push_all_feedback());
    let late_now = rig.clock.now();
    let late_release_deadline = late_now + ms(1); // 1 ms left <= 2 ms publish margin
    let late_budget = ServiceBudget::for_release(late_now, late_release_deadline);
    let poll = rig.inner_mut().poll_hardware_for_release(&late_budget, late_release_deadline);
    assert!(
        rig.bus.tx_log().is_empty(),
        "Step 2 enable-transition polling past release_deadline - margin must transmit 0 frames, sent {:?}",
        rig.bus.tx_log()
    );
    assert!(
        rig.inner().torque_transition_pending(),
        "deferred late Step 2 enable poll must keep transition pending (poll={poll:?})"
    );
    // Also verify sub-release wake past `release_deadline - margin` transmits 0 frames.
    let wake_poll = rig
        .inner_mut()
        .poll_transition_wake_for_release(&late_budget, late_release_deadline);
    assert!(
        rig.bus.tx_log().is_empty(),
        "sub-release transition wake past release_deadline - margin must transmit 0 frames (poll={wake_poll:?})"
    );
    assert!(rig.inner().torque_transition_pending());

    // Next on-time releases finish the remaining enable frames and confirm enabled with 0 faults.
    rig.bus.with(|s| s.tx_step = Duration::ZERO);
    for _ in 0..6 {
        rig.step();
    }
    assert!(!rig.inner().torque_transition_pending(), "on-time releases must complete enable transition");
    assert!(rig.inner().safety.state().is_operational(), "enable must complete operational: {:?}", rig.inner().safety.state());

    // Part B1: Direct driver `probe_tx_ready_budgeted` preserves an active `tx_release_deadline`
    // (must NOT clear it) and transmits 0 frames once `clock.now() >= tx_release_deadline`.
    let limits = dm_limits();
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&limits, clock.clone());
    bus.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap();
    drv.read_joint_states().unwrap();
    bus.clear_log();
    let cap = clock.now(); // already elapsed
    drv.set_tx_release_deadline(Some(cap));
    let probe_err = drv.probe_tx_ready_budgeted(clock.now() + ms(200)).unwrap_err();
    assert!(
        matches!(probe_err, armd::driver::ArmDriverError::Transport(armd::transport::TransportError::Timeout)),
        "probe_tx_ready_budgeted must respect active tx_release_deadline cap, got {probe_err:?}"
    );
    assert!(
        bus.tx_log().is_empty(),
        "probe_tx_ready_budgeted past tx_release_deadline must transmit 0 frames"
    );
    drv.set_tx_release_deadline(None);

    // Part B2: Executor Step 2b `clear_fault_executor_bounded_for_release` refuses with
    // BudgetExpired and 0 TX frames when called inside the 2 ms publish margin window, then
    // succeeds on an on-time release.
    let mut rig_cf = DualRig::new(Owner::Executor, base_cfg(), None);
    rig_cf.bus.with(|s| s.tx_enobufs = true);
    for _ in 0..5 {
        rig_cf.step();
        if rig_cf.inner().safety.fault_code() == Some(FaultCode::CanTxStalled) {
            break;
        }
    }
    assert_eq!(rig_cf.inner().safety.fault_code(), Some(FaultCode::CanTxStalled));
    rig_cf.bus.with(|s| {
        s.tx_enobufs = false;
        s.push_all_feedback();
    });
    rig_cf.step();
    rig_cf.bus.clear_log();
    rig_cf.bus.with(|s| s.push_all_feedback());
    let now = rig_cf.clock.now();
    // Only 1 ms left before release_deadline (<= 2 ms publish margin) -> BudgetExpired, 0 TX!
    let res_late = rig_cf
        .inner_mut()
        .clear_fault_executor_bounded_for_release(now, now + ms(1), None);
    assert_eq!(
        res_late,
        Err(armd::safety::ClearFaultError::Rejected(
            armd::safety::ClearRefusal::BudgetExpired
        )),
        "clear_fault probe inside publish margin must fail with BudgetExpired"
    );
    assert!(
        rig_cf.bus.tx_log().is_empty(),
        "clear_fault probe inside publish margin must transmit 0 frames, sent {:?}",
        rig_cf.bus.tx_log()
    );

    // On-time release (20 ms left -> 18 ms TX budget) runs the 0xCC probe and clears to Idle!
    rig_cf.bus.with(|s| s.push_all_feedback());
    let res_ok = rig_cf
        .inner_mut()
        .clear_fault_executor_bounded_for_release(now, now + ms(20), None);
    assert_eq!(res_ok, Ok(SafetyState::Idle));
    assert!(
        rig_cf.bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data[2] == 0xCC),
        "on-time clear_fault_executor_bounded_for_release must transmit the 0xCC probe"
    );
}

/// Round-3 Item 4:
/// Probe proof uses the RX sequence boundary captured immediately AFTER the probe write returns
/// successfully (because SocketCAN gives no arrival timestamps).
/// A feedback reply queued between the pre-drain and the probe write MUST NOT satisfy the clear;
/// only feedback arriving strictly after that post-write RX sequence boundary counts.
#[test]
fn dual_owner_probe_rejects_pre_probe_reply_queued_between_drain_and_write() {
    let build_motor0_feedback = |pos: f32, enabled: bool| -> armd::transport::CanFrame {
        let l = dm_limits()[0];
        let p = armd::protocol::damiao::float_to_uint(pos, l.p_min, l.p_max, 16);
        let v = armd::protocol::damiao::float_to_uint(0.0, l.v_min, l.v_max, 12);
        let t = armd::protocol::damiao::float_to_uint(0.0, l.t_min, l.t_max, 12);
        let data = [
            (u8::from(enabled) << 4) | 1u8,
            (p >> 8) as u8,
            p as u8,
            (v >> 4) as u8,
            (((v & 0x0f) << 4) | ((t >> 8) & 0x0f)) as u8,
            t as u8,
            30,
            30,
        ];
        armd::transport::CanFrame::new_standard(0x11, &data).unwrap()
    };

    // Part A: Direct driver check — a reply queued between the pre-drain and the 0xCC probe write
    // (`inject_on_next_tx`, with motor 0 silent so no genuine post-probe reply arrives) is drained
    // before the post-write sequence snapshot and MUST NOT confirm the probe. A subsequent genuine
    // feedback frame decoded after the post-write boundary DOES confirm it.
    let limits = dm_limits();
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&limits, clock.clone());
    bus.with(|s| {
        s.immediate_replies = false;
        s.push_all_feedback();
    });
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap();
    drv.read_joint_states().unwrap();
    bus.clear_log();
    bus.with(|s| {
        s.motors[0].silent = true;
        s.inject_on_next_tx = vec![build_motor0_feedback(0.05, false)];
    });
    let confirmed = drv.probe_tx_ready_budgeted(clock.now() + ms(200)).expect("probe write ok");
    assert!(!confirmed, "reply queued between pre-drain and probe write must NOT confirm probe");
    assert!(!drv.tx_probe_confirmed(), "tx_probe_confirmed must stay false for pre-probe reply");
    assert!(
        bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data[2] == 0xCC),
        "probe must have transmitted 0xCC refresh frame"
    );
    assert!(
        bus.with(|s| s.inject_on_next_tx.is_empty()),
        "injected pre-probe reply must have been queued on probe write and consumed by post-write drain"
    );

    // Now push a genuine post-probe reply strictly after the post-write sequence boundary -> confirms!
    bus.with(|s| {
        s.motors[0].silent = false;
        s.push_feedback(0);
    });
    drv.read_joint_states().unwrap();
    assert!(
        drv.tx_probe_confirmed(),
        "genuine post-probe feedback after post-write sequence boundary must confirm probe"
    );

    // Part B: Full dual-owner (Legacy + Executor) end-to-end CAN_TX_STALLED clear test.
    for owner in [Owner::Legacy, Owner::Executor] {
        let mut rig = DualRig::new(owner, base_cfg(), None);
        rig.bus.with(|s| s.tx_enobufs = true);
        for _ in 0..8 {
            rig.step();
            if rig.inner().safety.fault_code() == Some(FaultCode::CanTxStalled) {
                break;
            }
        }
        assert_eq!(rig.inner().safety.fault_code(), Some(FaultCode::CanTxStalled));
        let latched_epoch = rig.inner().safety.epoch_id();

        // Clear ENOBUFS, silence motor 0's genuine 0xCC reply, and inject a reply right between
        // the pre-drain and the probe write (`inject_on_next_tx`). Clear MUST be refused with
        // TxProbeFeedbackRequired even though that pre-probe reply arrived after the pre-drain!
        rig.bus.clear_log();
        rig.bus.with(|s| {
            s.tx_enobufs = false;
            s.motors[0].silent = true;
            s.inject_on_next_tx = vec![build_motor0_feedback(0.0, true)];
        });
        match &mut rig.owner {
            RigOwner::Legacy(inner) => {
                let err = inner.clear_fault().unwrap_err().to_string();
                assert!(
                    err.contains("CAN TX probe sent; wait for fresh feedback received after the probe"),
                    "{owner:?}: pre-probe reply queued between drain and write must be rejected with TxProbeFeedbackRequired, got: {err}"
                );
            }
            RigOwner::Executor { fx, .. } => {
                let dl = armd::transport::DriverClock::now(&rig.clock) + ms(200);
                let err = fx.core.clear_fault_executor_bounded(dl).unwrap_err();
                assert_eq!(
                    err,
                    armd::safety::ClearFaultError::Rejected(
                        armd::safety::ClearRefusal::TxProbeFeedbackRequired
                    ),
                    "{owner:?}: pre-probe reply queued between drain and write must be rejected"
                );
            }
        }
        assert!(
            rig.bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data[2] == 0xCC),
            "{owner:?}: probe must transmit 0xCC"
        );
        assert!(
            rig.bus.with(|s| s.inject_on_next_tx.is_empty()),
            "{owner:?}: pre-probe reply must have been injected on probe write"
        );
        assert_eq!(rig.inner().safety.fault_code(), Some(FaultCode::CanTxStalled));
        assert_eq!(rig.inner().safety.epoch_id(), latched_epoch);

        // Once motor 0 replies strictly AFTER the post-write boundary, clear succeeds cleanly.
        rig.bus.with(|s| {
            s.motors[0].silent = false;
            s.push_all_feedback();
        });
        rig.step();
        rig.clear_fault_and_enable();
        assert!(rig.inner().safety.state().is_operational());
    }
}
