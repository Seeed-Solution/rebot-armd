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
        inner.register_live_connection(1).unwrap();
        inner.tick(DT);
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).unwrap();
        SupervisorInner::enable_torque(admitted).expect("initial arm");
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
                let stamp = inner.current_admission_stamp();
                let admitted = inner.check_admission(stamp).unwrap();
                SupervisorInner::enable_torque(admitted).expect("legacy re-enable");
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
                for _ in 0..20 {
                    if fx.core.safety.state().is_operational() {
                        break;
                    }
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
            let (_, jog_id, _) = {
                let inner = rig.inner_mut();
                let stamp = inner.current_admission_stamp();
                let admitted = inner.check_admission(stamp).unwrap();
                SupervisorInner::jog_action(admitted, jog_goal.clone(), Some(0.5), Some(epoch), "joint_move")
            }
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
            let (_, home_id, _) = {
                let inner = rig.inner_mut();
                let stamp = inner.current_admission_stamp();
                let admitted = inner.check_admission(stamp).unwrap();
                SupervisorInner::jog_action(admitted, home_goal.clone(), Some(0.5), Some(epoch), "home")
            }
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
                let stamp = inner.current_admission_stamp();
                let admitted = inner.check_admission(stamp).unwrap();
                SupervisorInner::enable_torque(admitted).expect("arm");
                let stamp = inner.current_admission_stamp();
                let admitted = inner.check_admission(stamp).unwrap();
                SupervisorInner::jog_action(admitted, vec![0.15, -0.1, -0.1, 0.1, 0.0, 0.0, 0.0], Some(0.2), None, "joint_move").unwrap();
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
            sup.home(sup.capture_admission_stamp(), Some(0.4), None).await.expect("ArmSupervisor::home");
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
                let stamp = inner.current_admission_stamp();
                let admitted = inner.check_admission(stamp).unwrap();
                SupervisorInner::enable_torque(admitted).expect("legacy re-enable");
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

        // (c2) Hard transport errors with live RX (`fail_tx_after = Some(2)` partial hard error
        //      and `tx_error = true` full hard error): per-motor lack of progress on failing
        //      motors latches `CAN_TX_STALLED` once lack of progress reaches >= 45 ms, and
        //      recovers cleanly once the bus clears.
        {
            let mut rig_err = DualRig::new(owner, base_cfg(), None);
            rig_err.inner_mut()
                .safety
                .trigger_fault_coded(FaultCode::Other, FaultReason::Other("hard-error hold test".into()));
            // Partial submission: 2 frames accepted (J1-J2) then hard error on frame 3 (J3-J7
            // fail to reach transport) -> latches CAN_TX_STALLED once >= 45 ms elapses.
            let mut partial_stalled = false;
            for _ in 0..6 {
                rig_err.bus.with(|s| s.fail_tx_after = Some(2));
                rig_err.step();
                if rig_err.inner().safety.fault().map(|f| f.code) == Some(FaultCode::CanTxStalled) {
                    partial_stalled = true;
                    break;
                }
            }
            assert!(
                partial_stalled,
                "{owner:?}: partial hard-error stopped-hold streak (J3-J7 failing) must latch CAN_TX_STALLED"
            );
            rig_err.bus.with(|s| s.fail_tx_after = None);
            rig_err.step();
            rig_err.clear_fault_and_enable();

            // Hard error on frame 0 (`tx_error = true`, live RX) -> latches CAN_TX_STALLED once
            // >= 45 ms elapses, and resumes cleanly when bus clears.
            rig_err.inner_mut()
                .safety
                .trigger_fault_coded(FaultCode::Other, FaultReason::Other("hard-error-0 hold test".into()));
            rig_err.bus.with(|s| s.tx_error = true);
            let mut full_stalled = false;
            for _ in 0..6 {
                rig_err.step();
                if rig_err.inner().safety.fault().map(|f| f.code) == Some(FaultCode::CanTxStalled) {
                    full_stalled = true;
                    break;
                }
            }
            assert!(
                full_stalled,
                "{owner:?}: full hard-error stopped-hold streak with live RX must latch CAN_TX_STALLED"
            );
            rig_err.bus.with(|s| s.tx_error = false);
            rig_err.bus.clear_log();
            rig_err.step();
            let resumed: std::collections::HashSet<u32> =
                rig_err.bus.tx_log().into_iter().map(|f| f.id).collect();
            for j in 1u32..=7 {
                assert!(resumed.contains(&(0x100 + j)), "{owner:?}: joint {j} did not resume hold after tx_error cleared");
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
            let (rel_id, _) = {
                let inner = r.inner_mut();
                let stamp = inner.current_admission_stamp();
                let admitted = inner.check_admission(stamp).unwrap();
                SupervisorInner::start_gripper(
                    admitted,
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
            }
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
        {
            let inner = rig_gc.inner_mut();
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).unwrap();
            SupervisorInner::enter_gravity_comp(admitted, Default::default()).expect("enter gravity comp");
        }
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
            let id = {
                let inner = rig_mc.inner_mut();
                let stamp = inner.current_admission_stamp();
                let admitted = inner.check_admission(stamp).unwrap();
                SupervisorInner::begin_orchestrated_action(admitted, "grasp", None, epoch)
            }
            .expect("begin macro");
            let mut max_res = [0.0f32; armd::rt_core::mailbox::MAX_DOF];
            max_res[1] = 2.0; // J2 contact threshold 2.0 N·m
            let guard = armd::macro_action::ContactGuard { max_residual_nm: max_res, stop_code: None };
            rig_mc.inner_mut().guard_next_macro_child(id, Some(guard));
            let mut goal = rig_mc.inner().commanded_positions.clone();
            goal[0] += 0.35;
            let limits = rig_mc.inner().planning_limits();
            {
                let inner = rig_mc.inner_mut();
                let stamp = inner.current_admission_stamp();
                let admitted = inner.check_admission(stamp).unwrap();
                SupervisorInner::install_macro_quintic(admitted, id, goal, &limits)
            }
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
            {
                let inner = rig_jt.inner_mut();
                let stamp = inner.current_admission_stamp();
                let admitted = inner.check_admission(stamp).unwrap();
                SupervisorInner::enter_gravity_comp(admitted, Default::default()).expect("enter gravity comp");
            }
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
            {
                let inner = rig_s2.inner_mut();
                let stamp = inner.current_admission_stamp();
                let admitted = inner.check_admission(stamp).unwrap();
                SupervisorInner::enter_gravity_comp(admitted, Default::default()).expect("enter gravity comp");
            }
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
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).unwrap();
        SupervisorInner::enable_torque(admitted).expect("initial arm");
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
    rig.inner_mut().disable_torque().expect("torque off before enable test");
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

impl DualRig {
    /// Builds a 7-DOF B601-DM rig with J7 (index 6) configured in Damiao `PosForce` mode
    /// (`CTRL_MODE_POS_FORCE`), matching production `ArmConfig::rebot_dm()`.
    fn new_pos_force(kind: Owner, cfg: ArmConfig, budget: Option<TxBudget>) -> Self {
        let limits = dm_limits();
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&limits, clock.clone());
        bus.with(|s| {
            s.motors[6].set_reg_u32(
                armd::protocol::damiao::RID_CTRL_MODE,
                armd::protocol::damiao::CTRL_MODE_POS_FORCE,
            );
            s.immediate_replies = true;
            s.follow_commands = true;
            s.push_all_feedback();
        });
        let tx_budget = budget.unwrap_or_else(|| cfg.damiao_tx_budget());
        let grip = armd::driver::gripper_drive_from_profile(&cfg).expect("pos-force gripper drive");
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits)
            .unwrap()
            .with_gripper_drive(grip)
            .unwrap()
            .with_tx_budget(tx_budget);
        drv.read_joint_states().unwrap();
        drv.read_joint_states().unwrap();
        drv.check_mapping_ranges();

        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
        inner.lease_enabled = false;
        inner.register_live_connection(1).unwrap();
        inner.tick(DT);
        {
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).expect("admit initial pos-force arm");
            SupervisorInner::enable_torque(admitted).expect("initial pos-force arm");
        }
        bus.with(|s| {
            s.immediate_replies = false;
            s.push_all_feedback();
        });
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
                let (fx, broker, _tw, _stop) = ReleaseFixture::new_with_clock(
                    inner,
                    BootId(88),
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
}

fn j7_pf_frames(log: &[armd::transport::CanFrame]) -> Vec<armd::transport::CanFrame> {
    log.iter().copied().filter(|f| f.id == 0x307).collect()
}

fn j7_pf_current_raw(cfg: &ArmConfig) -> u16 {
    cfg.damiao_gripper
        .as_ref()
        .expect("pos-force gripper config")
        .current_ceiling_raw
}

fn assert_j7_freeze_frames(
    frames: &[armd::transport::CanFrame],
    expected_pos: f32,
    expected_cur: u16,
    label: &str,
) {
    assert!(
        !frames.is_empty(),
        "{label}: expected at least one 0x307 freeze frame while J7 is stale"
    );
    for f in frames {
        assert_eq!(f.id, 0x307, "{label}: unexpected frame id {f:?}");
        let pos = f32::from_le_bytes(f.data[0..4].try_into().unwrap());
        let vel = u16::from_le_bytes([f.data[4], f.data[5]]);
        let cur = u16::from_le_bytes([f.data[6], f.data[7]]);
        assert!(
            (pos - expected_pos).abs() < 1e-3,
            "{label}: freeze frame pos {pos} != last fresh measured pos {expected_pos}"
        );
        assert_eq!(vel, 0, "{label}: freeze frame must carry vel_raw == 0, got {vel}");
        assert_eq!(cur, expected_cur, "{label}: freeze frame current cap {cur} != {expected_cur}");
    }
}

fn j7_refresh_frames(log: &[armd::transport::CanFrame]) -> Vec<armd::transport::CanFrame> {
    log.iter()
        .copied()
        .filter(|f| f.id == 0x7FF && f.data[0] == 7 && f.data[2] == 0xCC)
        .collect()
}

/// Item 1 + Item 4 (Scenario 1 + config validation + full 6-frame budget refresh liveness):
/// - Config bounds: `feedback_stale_fault_ms` accepts `200..=450` and rejects `< 200` and `> 450`.
/// - Single 80 ms control-loop stall during `home` (with J7 reply queued in RX so
///   `feedback_queue_bound` age > 100 ms while decode age is 0 ms) on realistic
///   `0.45 ms/frame + 1 ms` spike bus: 0 faults, 0 `0x307` frames on stale tick, J7 `0xCC`
///   refresh emitted, J1–J6 continue, `0x307` resumes once fresh, and `home` completes `done`
///   at `[0; 7]` on BOTH Legacy and Executor.
/// - Continuously full 6-frame budget across motion AND stopped-hold: Tick 1 sends J1–J6 first,
///   Tick 2 prioritizes J7 `0xCC` refresh so refresh never starves and J1–J6 never false-stall.
#[test]
fn dual_owner_pos_force_80ms_stall_during_home_skips_j7_refreshes_and_completes() {
    // Config bounds check (`200..=450 ms`, >= 2x 100 ms freshness, < 500 ms BUS_LOSS_TIMEOUT).
    for (bad, ok) in [(199, false), (200, true), (250, true), (450, true), (451, false)] {
        let mut c = base_cfg();
        c.damiao_gripper.as_mut().unwrap().feedback_stale_fault_ms = Some(bad);
        assert_eq!(c.validate().is_ok(), ok, "feedback_stale_fault_ms={bad}");
    }

    for owner in [Owner::Legacy, Owner::Executor] {
        // Part A: Realistic 0.45 ms/frame + 1 ms spikes during `home` with a single 80+ ms stall.
        let mut rig = DualRig::new_pos_force(owner, base_cfg(), None);
        rig.bus.with(|s| {
            s.tx_attempts = 0;
            s.tx_step = us(450);
            s.tx_spike_every = 2;
            s.tx_spike_step = ms(1);
        });

        // Move away from home across all 7 joints (including J7 pos-force gripper).
        let away = vec![0.20, -0.16, -0.18, 0.22, -0.15, 0.20, -1.20];
        let ep = rig.inner().safety.epoch_id();
        let (_, jog_id, _) = {
            let inner = rig.inner_mut();
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).expect("admit away jog");
            SupervisorInner::jog_action(admitted, away.clone(), Some(0.6), Some(ep), "joint_move")
                .expect("start away jog")
        };
        let mut away_done = false;
        for _ in 0..90 {
            rig.step();
            assert!(rig.inner().safety.state().is_operational(), "{owner:?}: {:?}", rig.inner().safety.fault());
            if rig.inner().last_action.as_ref().is_some_and(|a| a.id == jog_id && a.state == "done") {
                away_done = true;
                break;
            }
        }
        assert!(away_done, "{owner:?}: away jog did not finish");

        // Start `home` back to [0; 7] and run 5 ticks so motion is in flight.
        let ep = rig.inner().safety.epoch_id();
        let (_, home_id, _) = {
            let inner = rig.inner_mut();
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).expect("admit home");
            SupervisorInner::jog_action(admitted, vec![0.0; 7], Some(0.6), Some(ep), "home")
                .expect("start home")
        };
        for _ in 0..5 {
            rig.step();
            assert!(rig.inner().safety.state().is_operational());
        }
        let home_epoch = rig.inner().safety.epoch_id();

        // Inject a realistic > 80 ms control-loop stall (total gap 105 ms since last empty RX
        // boundary) with motor replies ALREADY sitting in the RX buffer so J7's decode age is
        // 0 ms while its `feedback_queue_bound` age is 105 ms (> 100 ms!).
        let last_fresh_j7 = rig.inner().measured_positions[6];
        let pf_cur = j7_pf_current_raw(&base_cfg());
        rig.clock.advance(ms(105));
        rig.bus.with(|s| s.push_all_feedback());
        rig.bus.clear_log();
        rig.step_dt(DT, false);

        assert!(
            rig.inner().safety.state().is_operational(),
            "{owner:?}: transient 80+ ms stall during home must NOT fault: {:?}",
            rig.inner().safety.fault()
        );
        assert_eq!(rig.inner().safety.epoch_id(), home_epoch);
        let stale_log = rig.bus.tx_log();
        assert_j7_freeze_frames(
            &j7_pf_frames(&stale_log),
            last_fresh_j7,
            pf_cur,
            &format!("{owner:?} home 105 ms stall"),
        );
        for id in 1u32..=6 {
            assert!(
                stale_log.iter().any(|f| f.id == 0x100 + id),
                "{owner:?}: J{id} must continue during transient J7 stall: {stale_log:?}"
            );
        }
        assert!(
            !j7_refresh_frames(&stale_log).is_empty(),
            "{owner:?}: expected J7 0xCC refresh on stale tick: {stale_log:?}"
        );

        // Next ticks with fresh J7 feedback: moving 0x307 frames (vel_raw > 0) resume and `home` completes cleanly.
        rig.bus.clear_log();
        let mut resumed_j7 = false;
        let mut home_done = false;
        for _ in 0..100 {
            rig.step();
            assert!(
                rig.inner().safety.state().is_operational(),
                "{owner:?}: fault after recovering from stall: {:?}",
                rig.inner().safety.fault()
            );
            if j7_pf_frames(&rig.bus.tx_log())
                .iter()
                .any(|f| u16::from_le_bytes([f.data[4], f.data[5]]) > 0)
            {
                resumed_j7 = true;
            }
            if rig.inner().last_action.as_ref().is_some_and(|a| a.id == home_id && a.state == "done") {
                home_done = true;
                break;
            }
        }
        assert!(resumed_j7, "{owner:?}: moving 0x307 frames did not resume after fresh J7 feedback");
        assert!(home_done, "{owner:?}: home did not complete after transient stall");
        for j in 0..7 {
            assert!(
                rig.inner().measured_positions[j].abs() < 0.03,
                "{owner:?}: J{} not at home: {}",
                j + 1,
                rig.inner().measured_positions[j]
            );
        }

        // Part B: Tight 6-frame budget on a 7-motor arm under Round-8 M2 priority-by-change:
        // Class 1 (motion -> stop / hold / freeze, ANY joint including J7) > Class 2 (motion
        // targets) > Class 3 (unchanged hold re-sends) > Class 4 (J7 0xCC refresh and reads).
        let tight = TxBudget { max_frames: 6, max_duration: ms(10), frame_max_age: ms(45) };
        let mut rig_full = DualRig::new_pos_force(owner, base_cfg(), Some(tight));
        let full_last_fresh_j7 = rig_full.inner().measured_positions[6];
        rig_full.bus.with(|s| {
            s.motors[6].mute_feedback = true;
            s.immediate_replies = false;
        });
        rig_full.clock.advance(ms(110));
        // Tick 1 (operational): J7 was executing motion and now needs FREEZE (Class 1 > Class 2),
        // so J7's freeze frame transmits immediately within ceil(7/6) = 2 ticks (on tick 1!)
        // ahead of Class-2 arm motion targets; Class-4 0xCC does not steal a slot.
        rig_full.bus.clear_log();
        rig_full.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
        rig_full.step_dt(DT, false);
        let t1 = rig_full.bus.tx_log();
        assert_j7_freeze_frames(
            &j7_pf_frames(&t1),
            full_last_fresh_j7,
            pf_cur,
            &format!("{owner:?} 6-frame motion tick 1 (Class-1 J7 freeze)"),
        );
        assert!(j7_refresh_frames(&t1).is_empty(), "{owner:?}: Class-4 0xCC must not steal a slot on tick 1: {t1:?}");
        assert!(rig_full.inner().safety.state().is_operational());

        // Tick 2 (operational): J7's unchanged freeze is now Class 3 < Class 2, so all 6 arm
        // joints J1-J6 transmit their Class-2 targets with zero fault!
        rig_full.bus.clear_log();
        rig_full.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
        rig_full.step_dt(DT, false);
        let t2 = rig_full.bus.tx_log();
        for id in 1u32..=6 {
            assert!(t2.iter().any(|f| f.id == 0x100 + id), "{owner:?}: tick 2 did not send Class-2 J{id}: {t2:?}");
        }
        assert!(j7_refresh_frames(&t2).is_empty(), "{owner:?}: Class-4 0xCC must not steal a Class-2 slot: {t2:?}");
        assert!(rig_full.inner().safety.state().is_operational());

        // Unmute J7 -> recovers and resumes fresh 0x307 with 0 faults (give 7-slot budget so all 7 send each tick).
        rig_full.inner_mut()
            .driver
            .as_ref()
            .unwrap()
            .lock()
            .unwrap()
            .set_tx_budget(TxBudget { max_frames: 8, max_duration: ms(10), frame_max_age: ms(45) });
        rig_full.bus.with(|s| {
            s.motors[6].mute_feedback = false;
            s.push_all_feedback();
        });
        rig_full.bus.clear_log();
        rig_full.step();
        rig_full.step();
        assert!(rig_full.inner().safety.state().is_operational());
        assert!(
            j7_pf_frames(&rig_full.bus.tx_log())
                .iter()
                .any(|f| u16::from_le_bytes([f.data[4], f.data[5]]) > 0),
            "{owner:?}: J7 did not resume moving/fresh 0x307 after recovery"
        );

        // Stopped-hold priority-by-change check (Round-8 M2): keep J1-J6 fresh while aging J7
        // past 100 ms, then restore the 6-frame budget and fault. All 7 joints (J1-J6 holds
        // + J7 freeze) transmit within ceil(7/6) = 2 ticks with 0 Class-4 0xCC stealing slots.
        rig_full.bus.with(|s| s.motors[6].mute_feedback = true);
        for _ in 0..6 {
            rig_full.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            rig_full.step_dt(DT, false);
        }
        rig_full.inner_mut()
            .driver
            .as_ref()
            .unwrap()
            .lock()
            .unwrap()
            .set_tx_budget(tight);
        rig_full.inner_mut()
            .safety
            .trigger_fault_coded(FaultCode::Other, FaultReason::Other("6-frame hold test".into()));
        let mut h6_arm_seen = std::collections::BTreeSet::new();
        let mut h6_j7_freeze_seen = false;
        for tick in 1..=2 {
            rig_full.bus.clear_log();
            rig_full.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            rig_full.step_dt(DT, false);
            let h6 = rig_full.bus.tx_log();
            assert_eq!(h6.len(), 6, "{owner:?}: tick {tick} must fill all 6 slots with Class 1/3: {h6:?}");
            assert!(j7_refresh_frames(&h6).is_empty(), "{owner:?}: Class-4 0xCC must not steal a Class-1/3 slot: {h6:?}");
            for id in 1u32..=6 {
                if h6.iter().any(|f| f.id == 0x100 + id) {
                    h6_arm_seen.insert(id);
                }
            }
            let pf_now = j7_pf_frames(&h6);
            if !pf_now.is_empty() {
                assert_eq!(u16::from_le_bytes([pf_now[0].data[4], pf_now[0].data[5]]), 0);
                h6_j7_freeze_seen = true;
            }
        }
        assert_eq!(
            h6_arm_seen,
            (1u32..=6).collect(),
            "{owner:?}: all 6 arm joints J1-J6 must receive Class-1 holds within ceil(7/6)=2 ticks"
        );
        assert!(
            h6_j7_freeze_seen,
            "{owner:?}: J7 freeze must transmit within ceil(7/6)=2 ticks under 6-frame budget"
        );

        // With 8-frame capacity (6 arm holds + J7 freeze + J7 0xCC), all 6 arm holds go first,
        // followed by J7 freeze AND J7 0xCC refresh with no false CAN_TX_STALLED!
        let mut rig_h8 = DualRig::new_pos_force(
            owner,
            base_cfg(),
            Some(TxBudget { max_frames: 8, max_duration: ms(10), frame_max_age: ms(45) }),
        );
        let h8_last_fresh_j7 = rig_h8.inner().measured_positions[6];
        rig_h8.inner_mut()
            .safety
            .trigger_fault_coded(FaultCode::Other, FaultReason::Other("8-frame hold test".into()));
        let h8_ep = rig_h8.inner().safety.epoch_id();
        rig_h8.bus.with(|s| s.motors[6].mute_feedback = true);
        for _ in 0..6 {
            rig_h8.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            rig_h8.step_dt(DT, false);
        }
        rig_h8.bus.clear_log();
        rig_h8.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
        rig_h8.step_dt(DT, false);
        let h8 = rig_h8.bus.tx_log();
        for id in 1u32..=6 {
            assert!(h8.iter().any(|f| f.id == 0x100 + id), "{owner:?}: 8-frame hold missed J{id}: {h8:?}");
        }
        assert_j7_freeze_frames(
            &j7_pf_frames(&h8),
            h8_last_fresh_j7,
            pf_cur,
            &format!("{owner:?} 8-frame stopped-hold"),
        );
        assert!(
            !j7_refresh_frames(&h8).is_empty(),
            "{owner:?}: 8-frame hold must send J7 0xCC refresh once capacity allows: {h8:?}"
        );
        assert_eq!(rig_h8.inner().safety.epoch_id(), h8_ep, "{owner:?}: no false CAN_TX_STALLED in 8-frame hold");
    }
}

/// Item 1 + Item 4 (carry-over stale J7 vs mixed-queue expiry + mid-batch clock crossing 100 ms):
/// - J7-only carried-over frame crossing both 45 ms FIFO expiry and 100 ms freshness: stale-J7 wins
///   over FIFO expiry (0 faults, 0 `0x307`, J7 `0xCC` emitted, J1-J6 continue).
/// - Mixed-queue negative control (J6 + stale J7): stale J7 is dropped by Item 1, while J6's
///   genuine > 45 ms FIFO expiry STILL latches `CAN_TX_STALLED`.
/// - Clock crossing 100 ms during frame construction/submission: J1-J6 sent, 0 `0x307`, `0xCC` sent,
///   still operational.
#[test]
fn dual_owner_pos_force_carryover_stale_j7_vs_mixed_queue_and_mid_batch_expiry() {
    for owner in [Owner::Legacy, Owner::Executor] {
        // (a) J7-only carry-over crossing 45 ms + 100 ms -> converts to freeze frame, 0 faults.
        let pf_cur = j7_pf_current_raw(&base_cfg());
        let b6 = TxBudget { max_frames: 6, max_duration: ms(10), frame_max_age: ms(45) };
        let mut rig_j7 = DualRig::new_pos_force(owner, base_cfg(), Some(b6));
        // Tick 1: sends J1-J6, defers ONLY J7 in `pending_tx`.
        rig_j7.step();
        let last_fresh_j7 = rig_j7.inner().measured_positions[6];
        assert!(rig_j7.inner().safety.state().is_operational());
        // Advance 105 ms with J7 muted and J1-J6 fresh: J7 is both > 45 ms queued AND > 100 ms stale.
        rig_j7.bus.with(|s| s.motors[6].mute_feedback = true);
        rig_j7.clock.advance(ms(105));
        rig_j7.bus.clear_log();
        rig_j7.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
        rig_j7.step_dt(DT, false);
        assert!(
            rig_j7.inner().safety.state().is_operational(),
            "{owner:?}: J7-only stale carry-over must NOT latch CAN_TX_STALLED: {:?}",
            rig_j7.inner().safety.fault()
        );
        assert_j7_freeze_frames(
            &j7_pf_frames(&rig_j7.bus.tx_log()),
            last_fresh_j7,
            pf_cur,
            &format!("{owner:?} J7-only stale carry-over"),
        );

        // (b) Mixed-queue negative control (J6 + stale J7): J6 genuine expiry MUST latch CAN_TX_STALLED.
        let b5 = TxBudget { max_frames: 5, max_duration: ms(10), frame_max_age: ms(45) };
        let mut rig_mix = DualRig::new_pos_force(owner, base_cfg(), Some(b5));
        rig_mix.step(); // Sends J1-J5, defers J6 + J7.
        assert!(rig_mix.inner().safety.state().is_operational());
        rig_mix.bus.with(|s| s.motors[6].mute_feedback = true);
        rig_mix.clock.advance(ms(105));
        rig_mix.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
        rig_mix.step_dt(DT, false);
        assert_eq!(
            rig_mix.inner().safety.fault_code(),
            Some(FaultCode::CanTxStalled),
            "{owner:?}: genuine J6 FIFO expiry in mixed queue must still latch CAN_TX_STALLED"
        );

        // (c) Mid-batch clock crossing 100 ms during frame construction/submission -> J7 freeze frame!
        let mut rig_mid = DualRig::new_pos_force(owner, base_cfg(), None);
        rig_mid.step(); // fresh J7 at t = t0
        let mid_last_fresh_j7 = rig_mid.inner().measured_positions[6];
        rig_mid.bus.with(|s| {
            s.motors[6].mute_feedback = true;
            s.tx_step = ms(1);
        });
        rig_mid.clock.advance(ms(99));
        rig_mid.bus.clear_log();
        rig_mid.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
        rig_mid.step_dt(DT, false);
        let mid_log = rig_mid.bus.tx_log();
        assert!(
            rig_mid.inner().safety.state().is_operational(),
            "{owner:?}: mid-batch J7 crossing 100 ms must not fault: {:?}",
            rig_mid.inner().safety.fault()
        );
        assert_j7_freeze_frames(
            &j7_pf_frames(&mid_log),
            mid_last_fresh_j7,
            pf_cur,
            &format!("{owner:?} mid-batch 100 ms crossing"),
        );
        for id in 1u32..=6 {
            assert!(mid_log.iter().any(|f| f.id == 0x100 + id), "{owner:?}: J{id} missing on mid-batch tick");
        }
        assert!(!j7_refresh_frames(&mid_log).is_empty(), "{owner:?}: expected 0xCC refresh on mid-batch tick");
    }
}

/// Item 1 + Item 4 (Scenario 2 + configurable 450 ms + absent queue_bound + > 500 ms BUS_LOSS +
/// arm-joint loss negative control + macro terminal code + atomic live `clear_fault`):
/// - J7 stale 300 ms (default 250 ms limit) latches recoverable `J7_FEEDBACK_STALE`, terminates
///   running motion AND running macro with exact `"J7_FEEDBACK_STALE"` code, keeps J1-J6 holding
///   with stable epoch, refuses `clear_fault` while J7 is stale (including live expiry before
///   commit with zero mutation to `is_torqued` / limits), and recovers cleanly once J7 is fresh.
/// - Configurable `450 ms` stays operational through 420 ms and latches recoverable
///   `J7_FEEDBACK_STALE` by 460 ms (<= 500 ms).
/// - Complete CAN silence > 500 ms STILL latches non-recoverable `BUS_LOSS` (unchanged).
/// - Arm-joint J1 silence latches `BUS_LOSS`, never `J7_FEEDBACK_STALE`.
/// - Recent J7 RX with frozen/stale queue-bound bounds staleness to recoverable `J7_FEEDBACK_STALE`
///   without triggering `BUS_LOSS`.
#[test]
fn dual_owner_pos_force_sustained_300ms_staleness_recoverable_j7_fault_macro_and_atomic_clear() {
    for owner in [Owner::Legacy, Owner::Executor] {
        // (a) Default 250 ms limit: 300 ms stale J7 during running motion -> recoverable J7_FEEDBACK_STALE.
        let mut rig = DualRig::new_pos_force(owner, base_cfg(), None);
        let ep = rig.inner().safety.epoch_id();
        let (_, jog_id, _) = {
            let inner = rig.inner_mut();
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).expect("admit motion");
            SupervisorInner::jog_action(
                admitted,
                vec![0.3, -0.2, 0.0, 0.0, 0.0, 0.0, -1.0],
                Some(0.3),
                Some(ep),
                "joint_move",
            )
            .expect("start motion")
        };
        rig.step();
        rig.bus.with(|s| s.motors[6].mute_feedback = true);

        // Stay below 250 ms (220 ms): still operational, 0 faults.
        for _ in 0..11 {
            rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            rig.step_dt(DT, false);
            assert!(
                rig.inner().safety.state().is_operational(),
                "{owner:?}: must not fault before 250 ms: {:?}",
                rig.inner().safety.fault()
            );
        }
        // Push past 250 ms (to ~300 ms total): latches recoverable J7_FEEDBACK_STALE.
        for _ in 0..4 {
            rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            rig.step_dt(DT, false);
        }
        assert_eq!(
            rig.inner().safety.fault_code(),
            Some(FaultCode::J7FeedbackStale),
            "{owner:?}: 300 ms J7 staleness must latch J7_FEEDBACK_STALE"
        );
        assert_eq!(*rig.inner().safety.state(), SafetyState::Fault(FaultReason::J7FeedbackStale));
        assert!(rig.inner().safety.recoverable(), "{owner:?}: J7_FEEDBACK_STALE must be recoverable");
        assert!(!rig.inner().output_suspended, "{owner:?}: J1-J6 output must stay active while <= 500 ms");
        let act = rig.inner().last_action.as_ref().expect("terminated action");
        assert_eq!(act.id, jog_id);
        assert_eq!(act.state, "failed");
        assert_eq!(act.code.as_deref(), Some("J7_FEEDBACK_STALE"));
        assert_eq!(
            rig.inner().action_outcome(armd::rt_core::broker::ActionId(jog_id)),
            Some(armd::rt_core::broker::Outcome::Failed("J7_FEEDBACK_STALE"))
        );

        // Subsequent stale ticks (still <= 440 ms < 500 ms BUS_LOSS): epoch stays stable, J1-J6 hold.
        let latched_ep = rig.inner().safety.epoch_id();
        rig.bus.clear_log();
        for _ in 0..3 {
            rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            rig.step_dt(DT, false);
            assert_eq!(rig.inner().safety.epoch_id(), latched_ep, "{owner:?}: epoch must stay stable");
            assert_eq!(rig.inner().safety.fault_code(), Some(FaultCode::J7FeedbackStale));
        }
        let hold_log = rig.bus.tx_log();
        for id in 1u32..=6 {
            assert!(hold_log.iter().any(|f| f.id == 0x100 + id), "{owner:?}: J{id} hold stopped during J7_FEEDBACK_STALE");
        }
        let j7_hold_frames = j7_pf_frames(&hold_log);
        assert!(
            !j7_hold_frames.is_empty(),
            "{owner:?}: J7 must receive 0x307 freeze frames during J7_FEEDBACK_STALE"
        );
        for f in &j7_hold_frames {
            assert_eq!(
                u16::from_le_bytes([f.data[4], f.data[5]]),
                0,
                "{owner:?}: J7 stopped-hold freeze frame must have vel_raw == 0"
            );
            assert_eq!(
                u16::from_le_bytes([f.data[6], f.data[7]]),
                j7_pf_current_raw(&base_cfg())
            );
        }

        // Clear refused while J7 is STILL stale (> 100 ms), leaving `is_torqued` and `limits` unchanged.
        let prev_torqued = rig.inner().is_torqued;
        let prev_limits = rig.inner().config.limits.clone();
        match &mut rig.owner {
            RigOwner::Legacy(inner) => {
                let err = inner.clear_fault().unwrap_err().to_string();
                assert!(err.contains("J7") || err.contains("feedback"), "{err}");
            }
            RigOwner::Executor { fx, .. } => {
                let dl = armd::transport::DriverClock::now(&rig.clock) + ms(200);
                let err = fx.core.clear_fault_executor_bounded(dl).unwrap_err();
                assert_eq!(
                    err,
                    armd::safety::ClearFaultError::Rejected(armd::safety::ClearRefusal::J7FeedbackStale)
                );
            }
        }
        assert_eq!(rig.inner().is_torqued, prev_torqued);
        assert_eq!(rig.inner().config.limits, prev_limits);
        assert_eq!(rig.inner().safety.epoch_id(), latched_ep);

        // Live refusal when J7 was fresh then crosses 100 ms (150 ms, still <= 250 ms bus-fresh):
        // verify ZERO preflight commits across torque, limits, epoch, feedback publication, AND
        // thermal cooldown!
        rig.inner_mut().thermal_cooldown_duration = ms(80);
        rig.bus.with(|s| {
            s.motors[6].mute_feedback = false;
            for m in &mut s.motors {
                m.temperature_c = 40;
            }
            s.push_all_feedback();
        });
        rig.step(); // J7 becomes fresh
        rig.bus.with(|s| s.motors[6].mute_feedback = true);
        // Move J1 position on the bus to a distinct value so any premature Legacy feedback
        // publication on a refused clear is caught immediately.
        let prev_measured = rig.inner().measured_positions.clone();
        let prev_velocities = rig.inner().measured_velocities.clone();
        let prev_fb_valid = rig.inner().feedback_numeric_valid;
        let prev_last_fb_inst = rig.inner().last_can_feedback_instant;
        let prev_limits_snap: Vec<(f32, f32)> = rig
            .inner()
            .config
            .limits
            .iter()
            .map(|l| (l.min_position, l.max_position))
            .collect();
        rig.clock.advance(ms(140));
        rig.bus.with(|s| {
            s.motors[0].q = prev_measured[0] + 0.25;
            for i in 0..6 {
                s.push_feedback(i);
            }
        });
        match &mut rig.owner {
            RigOwner::Legacy(inner) => {
                let err = inner.clear_fault().unwrap_err().to_string();
                assert!(err.contains("J7") || err.contains("feedback"), "{err}");
            }
            RigOwner::Executor { fx, .. } => {
                let dl = armd::transport::DriverClock::now(&rig.clock) + ms(200);
                let err = fx.core.clear_fault_executor_bounded(dl).unwrap_err();
                assert_eq!(
                    err,
                    armd::safety::ClearFaultError::Rejected(armd::safety::ClearRefusal::J7FeedbackStale)
                );
            }
        }
        assert_eq!(rig.inner().is_torqued, prev_torqued);
        assert_eq!(rig.inner().safety.epoch_id(), latched_ep);
        assert_eq!(
            rig.inner().measured_positions,
            prev_measured,
            "{owner:?}: refused stale-J7 clear must NOT publish sampled feedback positions"
        );
        assert_eq!(rig.inner().measured_velocities, prev_velocities);
        assert_eq!(rig.inner().feedback_numeric_valid, prev_fb_valid);
        assert_eq!(rig.inner().last_can_feedback_instant, prev_last_fb_inst);
        let after_limits_snap: Vec<(f32, f32)> = rig
            .inner()
            .config
            .limits
            .iter()
            .map(|l| (l.min_position, l.max_position))
            .collect();
        assert_eq!(after_limits_snap, prev_limits_snap);

        // Prove thermal cooldown was NOT started/committed during the refused J7 clear:
        // start warm (72 C: above 70 C cool threshold so thermal_cooldown resets, below 75 C trip
        // threshold so only J7FeedbackStale is latched), cool to 40 C at t = 140 ms during a
        // refused stale-J7 clear, then advance 100 ms (>= 80 ms cooldown!) with fresh cool J1–J7
        // feedback and latch ThermalTrip: dwell MUST be 0 ms (not 100 ms -> Ok(Idle))!
        {
            let mut probe_rig = DualRig::new_pos_force(owner, base_cfg(), None);
            probe_rig.inner_mut().thermal_cooldown_duration = ms(80);
            // Warm sample (above `max_temperature - 5 C` cool threshold, strictly below `max_temperature`
            // trip threshold) resets thermal_cooldown cleanly without latching ThermalTrip.
            let warm_c = (probe_rig.inner().config.max_temperature - 2.0).floor() as u8;
            probe_rig.bus.with(|s| {
                s.motors[0].temperature_c = warm_c;
                s.push_all_feedback();
            });
            probe_rig.step();
            assert!(
                probe_rig.inner().safety.state().is_operational(),
                "warm sample {warm_c} C must not latch ThermalTrip"
            );
            probe_rig.inner_mut().safety.latch(
                armd::safety::FaultKind::J7FeedbackStale,
                armd::safety::FaultDetail::J7FeedbackStale(None),
            );
            // Cool down to 40 C while J7 is aged 140 ms (stale > 100 ms, still <= 250 ms bus-fresh).
            probe_rig.bus.with(|s| {
                s.motors[6].mute_feedback = true;
                for m in &mut s.motors {
                    m.temperature_c = 40;
                }
            });
            probe_rig.clock.advance(ms(140));
            probe_rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            if owner == Owner::Executor {
                // Directly read into driver without calling poll_hardware (which updates cooldown on ticks)
                probe_rig.inner().driver.as_ref().unwrap().lock().unwrap().read_joint_states().unwrap();
                probe_rig.inner_mut().temperatures.fill(40.0);
            }
            // Refused stale-J7 clear MUST NOT record cooldown timestamp!
            match &mut probe_rig.owner {
                RigOwner::Legacy(inner) => {
                    let err = inner.clear_fault().unwrap_err().to_string();
                    assert!(err.contains("J7"), "{err}");
                }
                RigOwner::Executor { fx, .. } => {
                    let dl = armd::transport::DriverClock::now(&probe_rig.clock) + ms(200);
                    assert_eq!(
                        fx.core.clear_fault_executor_bounded(dl).unwrap_err(),
                        armd::safety::ClearFaultError::Rejected(armd::safety::ClearRefusal::J7FeedbackStale)
                    );
                }
            }
            // Advance 100 ms (>= 80 ms cooldown required), push fresh cool J1–J7 feedback, and
            // latch ThermalTrip: elapsed dwell MUST be 0 ms (had the refused J7 clear committed
            // `thermal_cooldown`, dwell would be 100 ms >= 80 ms and clear would succeed!).
            probe_rig.clock.advance(ms(100));
            probe_rig.bus.with(|s| {
                s.motors[6].mute_feedback = false;
                s.push_all_feedback();
            });
            if owner == Owner::Executor {
                probe_rig.inner().driver.as_ref().unwrap().lock().unwrap().read_joint_states().unwrap();
            }
            probe_rig.inner_mut().safety.latch(
                armd::safety::FaultKind::ThermalTrip,
                armd::safety::FaultDetail::ThermalTrip { joint_idx: 0, temperature: 85.0, limit: 75.0 },
            );
            match &mut probe_rig.owner {
                RigOwner::Legacy(inner) => {
                    let err = inner.clear_fault().unwrap_err().to_string();
                    assert!(
                        err.contains("0 ms of 80 ms"),
                        "Legacy refused J7 clear must not have started thermal cooldown: {err}"
                    );
                }
                RigOwner::Executor { fx, .. } => {
                    let dl = armd::transport::DriverClock::now(&probe_rig.clock) + ms(200);
                    assert_eq!(
                        fx.core.clear_fault_executor_bounded(dl).unwrap_err(),
                        armd::safety::ClearFaultError::Rejected(
                            armd::safety::ClearRefusal::ThermalCooldownNotElapsed {
                                elapsed_ms: 0,
                                required_ms: 80,
                            }
                        ),
                        "Executor refused J7 clear must not have started thermal cooldown"
                    );
                }
            }
        }
        // Restore motor 0 position before continuing recovery check.
        rig.bus.with(|s| {
            s.motors[0].q = prev_measured[0];
        });

        // Fresh J7 feedback returns (<= 100 ms across an empty-RX boundary) -> clear_fault + torque_on recover!
        rig.bus.with(|s| {
            s.motors[6].mute_feedback = false;
            s.immediate_replies = true;
            s.push_all_feedback();
        });
        rig.step(); // establishes fresh empty-RX boundary
        rig.step(); // decodes post-boundary J7 sample with queue_bound <= 20 ms
        rig.clear_fault_and_enable();
        rig.bus.with(|s| s.immediate_replies = false);
        assert!(rig.inner().safety.state().is_operational());

        // (a2) Running macro (`MacroRun`) gets exact terminal code `"J7_FEEDBACK_STALE"`.
        let ep2 = rig.inner().safety.epoch_id();
        let macro_id = {
            let inner = rig.inner_mut();
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).expect("admit macro");
            SupervisorInner::begin_sequence_action(admitted, Some("test-macro".into()), ep2)
                .expect("begin macro")
        };
        assert!(rig.inner().macro_run.is_some());
        rig.bus.with(|s| s.motors[6].mute_feedback = true);
        for _ in 0..16 {
            rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            rig.step_dt(DT, false);
            if rig.inner().safety.fault_code() == Some(FaultCode::J7FeedbackStale) {
                break;
            }
        }
        assert_eq!(rig.inner().safety.fault_code(), Some(FaultCode::J7FeedbackStale));
        assert!(rig.inner().macro_run.is_none(), "{owner:?}: MacroRun must be retired on J7_FEEDBACK_STALE");
        assert_eq!(
            rig.inner().action_outcome(armd::rt_core::broker::ActionId(macro_id)),
            Some(armd::rt_core::broker::Outcome::Failed("J7_FEEDBACK_STALE"))
        );

        // (b) Configurable 450 ms limit: 420 ms operational, 460 ms latches recoverable J7_FEEDBACK_STALE,
        //     and (c) past 500 ms complete silence STILL escalates to non-recoverable BUS_LOSS!
        let mut cfg450 = base_cfg();
        cfg450.damiao_gripper.as_mut().unwrap().feedback_stale_fault_ms = Some(450);
        let mut rig450 = DualRig::new_pos_force(owner, cfg450, None);
        rig450.bus.with(|s| s.motors[6].mute_feedback = true);
        for _ in 0..21 {
            // 420 ms < 450 ms
            rig450.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            rig450.step_dt(DT, false);
            assert!(rig450.inner().safety.state().is_operational(), "{owner:?}: 450 ms config faulted early");
        }
        for _ in 0..3 {
            // 460 ms >= 450 ms, still <= 500 ms BUS_LOSS_TIMEOUT
            rig450.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            rig450.step_dt(DT, false);
        }
        assert_eq!(rig450.inner().safety.fault_code(), Some(FaultCode::J7FeedbackStale));
        assert!(rig450.inner().safety.recoverable());
        // Continue past 500 ms complete silence (540 ms) -> BUS_LOSS unchanged!
        for _ in 0..4 {
            rig450.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            rig450.step_dt(DT, false);
        }
        assert_eq!(
            rig450.inner().safety.fault_code(),
            Some(FaultCode::BusLoss),
            "{owner:?}: > 500 ms complete CAN silence must still latch BUS_LOSS"
        );
        assert!(!rig450.inner().safety.recoverable());

        // (d) Arm-joint J1 silence negative control: never latches J7_FEEDBACK_STALE; latches BUS_LOSS > 500 ms.
        let mut rig_arm = DualRig::new_pos_force(owner, base_cfg(), None);
        rig_arm.bus.with(|s| s.motors[0].mute_feedback = true);
        for _ in 0..16 {
            // 320 ms: J7 is fresh, J1 is silent -> neither J7_FEEDBACK_STALE nor BUS_LOSS yet.
            rig_arm.bus.with(|s| for i in 1..7 { s.push_feedback(i); });
            rig_arm.step_dt(DT, false);
            assert_ne!(rig_arm.inner().safety.fault_code(), Some(FaultCode::J7FeedbackStale));
        }
        for _ in 0..12 {
            rig_arm.bus.with(|s| for i in 1..7 { s.push_feedback(i); });
            rig_arm.step_dt(DT, false);
        }
        assert_eq!(rig_arm.inner().safety.fault_code(), Some(FaultCode::BusLoss));

        // (e) Continuous saturated RX (queue never empties so J7 `rx_at` stays <= 20 ms fresh and
        //     `bus_feedback_fresh == true`, while `feedback_queue_bound[J7]` stays stuck old):
        //     bounded by `feedback_stale_fault_ms` -> latches recoverable J7_FEEDBACK_STALE without BUS_LOSS.
        let mut rig_sat = DualRig::new_pos_force(owner, base_cfg(), None);
        for _ in 0..16 {
            // Keep RX saturated (> 260 frames) so receive_feedback hits its cap without observing
            // an empty queue, refreshing `last_feedback_instant` while `queue_bound` stays stale.
            rig_sat.bus.with(|s| {
                for _ in 0..45 {
                    s.push_all_feedback();
                }
            });
            rig_sat.step_dt(DT, false);
        }
        assert_eq!(
            rig_sat.inner().safety.fault_code(),
            Some(FaultCode::J7FeedbackStale),
            "{owner:?}: stale queue-bound with fresh RX must latch recoverable J7_FEEDBACK_STALE"
        );
        assert!(rig_sat.inner().safety.recoverable());
        assert_ne!(rig_sat.inner().safety.fault_code(), Some(FaultCode::BusLoss));
    }
}

/// Item 2 + Item 4 (Scenario 3 + expired-plus-new replacement + dual-slot independent progress):
/// - Stopped hold with 0 queued frames and stale J7 -> NEVER latches `CAN_TX_STALLED`.
/// - Expired-plus-new hold replacement on healthy bus -> NO instant false `CAN_TX_STALLED`.
/// - Dual-slot joint (uncertain mode emitting `0x101` + `0x001`): slot 0 succeeds (`sent_any = true`)
///   while slot 1 hits persistent backpressure (`backpressure_tx_ids = [0x001]`): latches
///   `CAN_TX_STALLED` once slot 1 stays queued >= `max_age` at BOTH 50 Hz and slow 90 ms cadence.
#[test]
fn dual_owner_stopped_hold_zero_queued_stale_j7_and_dual_slot_backlog() {
    for owner in [Owner::Legacy, Owner::Executor] {
        // (a) Task Scenario 3: stopped hold with 0 queued frames and stale J7 -> sends J7 freeze
        //     frames (`vel_raw == 0`, last fresh pos), NEVER latches `CAN_TX_STALLED`.
        let pf_cur = j7_pf_current_raw(&base_cfg());
        let mut rig = DualRig::new_pos_force(owner, base_cfg(), None);
        let last_fresh_j7 = rig.inner().measured_positions[6];
        rig.inner_mut()
            .safety
            .trigger_fault_coded(FaultCode::Other, FaultReason::Other("stopped hold stale J7".into()));
        rig.bus.with(|s| s.motors[6].mute_feedback = true);
        for tick in 0..16 {
            // 16 normal 20 ms ticks = 320 ms: J1-J6 stay fresh (queue_bound <= 20 ms) while J7
            // goes stale past 100 ms (tick 6) and latches J7_FEEDBACK_STALE past 250 ms.
            rig.bus.clear_log();
            rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            rig.step_dt(DT, false);
            assert_ne!(
                rig.inner().safety.fault_code(),
                Some(FaultCode::CanTxStalled),
                "{owner:?}: stopped hold with 0 queued frames and stale J7 must NEVER latch CAN_TX_STALLED"
            );
            let pending_hold = rig
                .inner()
                .driver
                .as_ref()
                .unwrap()
                .lock()
                .unwrap()
                .pending_hold_tx_len();
            assert_eq!(pending_hold, 0, "{owner:?}: expected 0 queued hold frames");
            let log = rig.bus.tx_log();
            if tick >= 6 {
                assert_j7_freeze_frames(
                    &j7_pf_frames(&log),
                    last_fresh_j7,
                    pf_cur,
                    &format!("{owner:?} stale stopped-hold tick {tick}"),
                );
            }
            for id in 1u32..=6 {
                assert!(log.iter().any(|f| f.id == 0x100 + id), "{owner:?}: J{id} hold missing on tick {tick}");
            }
        }
        // By ~320 ms total J7 staleness, Item 1 records FaultKind::J7FeedbackStale (while
        // preserving existing equal-severity FaultCode::Other and NEVER latching CAN_TX_STALLED).
        assert!(rig.inner().safety.has_latched_fault_kind(armd::safety::FaultKind::J7FeedbackStale));
        assert_ne!(rig.inner().safety.fault_code(), Some(FaultCode::CanTxStalled));

        // (b) Expired-plus-new hold replacement (budget 4, advance 60 ms > 45 ms): no false CAN_TX_STALLED.
        let b4 = TxBudget { max_frames: 4, max_duration: ms(10), frame_max_age: ms(45) };
        let mut rig_rep = DualRig::new(owner, base_cfg(), Some(b4));
        rig_rep.inner_mut()
            .safety
            .trigger_fault_coded(FaultCode::Other, FaultReason::Other("expired-plus-new".into()));
        let rep_ep = rig_rep.inner().safety.epoch_id();
        rig_rep.step(); // Sends J1-J4, defers J5-J7.
        rig_rep.step_dt(0.06, true); // 60 ms > 45 ms -> drops old J5-J7, stages fresh, sends J5-J7 + J1!
        assert_eq!(rig_rep.inner().safety.epoch_id(), rep_ep);
        assert_eq!(rig_rep.inner().safety.fault_code(), Some(FaultCode::Other));

        // (c) Per-motor stopped-hold independent progress at BOTH 50 Hz (20 ms) and slow cadence
        //     (90 ms): J1 (`0x101`) sends every tick (`sent_any == true`) while J2 (`0x102`) hits
        //     persistent ENOBUFS backpressure -> MUST latch `CAN_TX_STALLED` once J2 age >= 45 ms!
        //     (Under R10, SupervisorInner stopped-hold enforces L2 verified-mode-only holds and
        //     resolves unverified modes via RID_CTRL_MODE readback instead of guessing dual-slot;
        //     driver-level dual-slot progress on `write_estop_hold_bounded` is also verified below.)
        for step_s in [0.02f64, 0.09f64] {
            let mut dual_rig = DualRig::new(owner, base_cfg(), None);
            dual_rig.inner_mut().safety.trigger_fault_coded(
                FaultCode::Other,
                FaultReason::Other("per-motor independent hold progress test".into()),
            );
            // Block ONLY J2 (`0x102`) while J1 (`0x101`) and peers succeed every tick (`sent_any == true`).
            dual_rig.bus.with(|s| s.backpressure_tx_ids = vec![0x102]);
            dual_rig.bus.clear_log();
            let mut latched = false;
            for _ in 0..8 {
                dual_rig.step_dt(step_s, true);
                if dual_rig.inner().safety.fault_code() == Some(FaultCode::CanTxStalled) {
                    latched = true;
                    break;
                }
            }
            assert!(
                latched,
                "{owner:?} step={step_s}s: blocked J2 0x102 must latch CAN_TX_STALLED even while J1 0x101 sends"
            );
            assert!(
                dual_rig.bus.tx_log().iter().any(|f| f.id == 0x101),
                "{owner:?} step={step_s}s: peer joint 0x101 must have transmitted"
            );
            let detail = dual_rig.inner().safety.tx_stall_detail().expect("tx_stall_detail");
            assert!(detail.is_hold && detail.queued > 0, "{owner:?}: expected queued > 0 hold stall, got {detail:?}");
        }
        // Driver-level dual-slot independent progress: on direct `write_estop_hold_bounded` with
        // uncertain J1 mode (`joint_modes[0] = None`), blocking slot 1 (`0x001`) while slot 0
        // (`0x101`) sends must report slot 1 due-but-unsent in `last_hold_tx_due_mask`/`sent_mask`.
        if owner == Owner::Legacy {
            let limits = dm_limits();
            let clock = MockClock::new();
            let bus = FakeBus::with_clock(&limits, clock.clone());
            bus.with(|s| {
                s.immediate_replies = true;
                s.follow_commands = true;
                s.push_all_feedback();
            });
            let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
                .with_joint_limits(limits)
                .unwrap();
            drv.read_joint_states().unwrap();
            drv.check_mapping_ranges();
            drv.set_torque(true).expect("enable");
            bus.with(|s| {
                s.immediate_replies = false;
                s.motors[0].no_write_ack = true;
            });
            let mut modes = vec![armd::driver::JointMode::Position; 7];
            modes[0] = armd::driver::JointMode::Mit;
            let holds = vec![JointTarget::position(0.0); 7];
            let _ = drv.set_joint_modes(&modes, &holds);
            bus.with(|s| {
                s.motors[0].no_write_ack = false;
                s.backpressure_tx_ids = vec![0x001];
                s.push_all_feedback();
            });
            assert!(drv.joint_modes_uncertain(), "J1 mode must be uncertain to emit dual hold slots");
            drv.read_joint_states().unwrap();
            let out = drv.write_estop_hold_bounded(&holds).expect("write_estop_hold_bounded");
            assert!(matches!(out, TxBatchOutcome::Deferred { .. }), "expected deferred dual-slot hold, got {out:?}");
            assert_eq!(drv.last_hold_tx_due_mask() & 0b11, 0b11, "both J1 hold slots must be due");
            assert_eq!(drv.last_hold_tx_sent_mask() & 0b11, 0b01, "only J1 slot 0 (0x101) sent while slot 1 (0x001) blocked");
        }
    }
}

/// Item 3 + Item 4 (rate-limited `"Stopped hold output failed"` <= 1/s with exact suppressed
/// counter on BOTH Legacy and Executor deferred-log paths, rapid clear/re-fault window continuity,
/// and deferred-ring overflow retention).
#[test]
fn dual_owner_stopped_hold_output_failed_rate_limit_and_suppressed_counter() {
    use std::sync::Mutex;

    struct CaptureHoldErrors(Arc<Mutex<Vec<String>>>);
    struct MsgVisitor(String);
    impl tracing::field::Visit for MsgVisitor {
        fn record_debug(&mut self, field: &tracing::field::Field, value: &dyn std::fmt::Debug) {
            if field.name() == "message" {
                self.0 = format!("{value:?}");
            }
        }
    }
    impl tracing::Subscriber for CaptureHoldErrors {
        fn enabled(&self, meta: &tracing::Metadata<'_>) -> bool {
            *meta.level() <= tracing::Level::ERROR
        }
        fn new_span(&self, _: &tracing::span::Attributes<'_>) -> tracing::span::Id {
            tracing::span::Id::from_u64(1)
        }
        fn record(&self, _: &tracing::span::Id, _: &tracing::span::Record<'_>) {}
        fn record_follows_from(&self, _: &tracing::span::Id, _: &tracing::span::Id) {}
        fn event(&self, event: &tracing::Event<'_>) {
            let mut v = MsgVisitor(String::new());
            event.record(&mut v);
            if v.0.contains("Stopped hold output") {
                self.0.lock().unwrap().push(v.0);
            }
        }
        fn enter(&self, _: &tracing::span::Id) {}
        fn exit(&self, _: &tracing::span::Id) {}
    }

    for owner in [Owner::Legacy, Owner::Executor] {
        let mut rig = DualRig::new(owner, base_cfg(), None);
        if owner == Owner::Executor {
            rig.inner_mut().safety.set_off_thread_reclaim(true);
        }
        rig.inner_mut()
            .safety
            .trigger_fault_coded(FaultCode::Other, FaultReason::Other("rate limit test".into()));
        rig.inner_mut().safety.drain_log_events_off_thread();
        rig.bus.with(|s| s.tx_error = true);

        let lines = Arc::new(Mutex::new(Vec::<String>::new()));
        let sub = CaptureHoldErrors(Arc::clone(&lines));
        tracing::subscriber::with_default(sub, || {
            // Run 105 ticks at 20 ms = 2.1 s on driver clock:
            // tick 0 (t=0 ms): emit #1 (suppressed = 0)
            // ticks 1..=49 (49 ticks): suppressed
            // tick 50 (t=1000 ms): emit #2 (suppressed = 49)
            // ticks 51..=99 (49 ticks): suppressed
            // tick 100 (t=2000 ms): emit #3 (suppressed = 49)
            // ticks 101..=104 (4 ticks): suppressed (buffered = 4)
            for _ in 0..105 {
                rig.step();
                if owner == Owner::Executor {
                    rig.inner_mut().safety.drain_log_events_off_thread();
                }
            }
            let captured = lines.lock().unwrap().clone();
            assert_eq!(
                captured.len(),
                3,
                "{owner:?}: expected exactly 3 error logs across 2.1 s (<= 1/s), got: {captured:?}"
            );
            assert!(
                !captured[0].contains("suppressed"),
                "{owner:?}: first log must have 0 suppressed: {}",
                captured[0]
            );
            assert!(
                captured[1].contains("suppressed 49 events"),
                "{owner:?}: second log must report 49 suppressed: {}",
                captured[1]
            );
            assert!(
                captured[2].contains("suppressed 49 events"),
                "{owner:?}: third log must report 49 suppressed: {}",
                captured[2]
            );
            assert_eq!(rig.inner().hold_tx_error_suppressed(), 4);
            assert_eq!(
                rig.inner().safety.fault_code(),
                Some(FaultCode::CanTxStalled),
                "{owner:?}: sustained hard TX error streak with live RX must escalate to CAN_TX_STALLED"
            );

            // Rapid clear / re-fault within the same 1 s window: MUST keep 1/s throttle AND
            // preserve the buffered suppressed count (no reset on clear_fault!).
            rig.bus.with(|s| {
                s.tx_error = false;
                s.push_all_feedback();
            });
            match &mut rig.owner {
                RigOwner::Legacy(inner) => {
                    inner.clear_fault().expect("legacy clear");
                }
                RigOwner::Executor { fx, .. } => {
                    let dl = armd::transport::DriverClock::now(&rig.clock) + ms(200);
                    fx.core.clear_fault_executor_bounded(dl).expect("executor clear");
                }
            }
            rig.inner_mut()
                .safety
                .trigger_fault_coded(FaultCode::Other, FaultReason::Other("re-fault".into()));
            rig.bus.with(|s| s.tx_error = true);
            lines.lock().unwrap().clear();
            // 5 more ticks (t = 2100..2180 ms, still < 3000 ms): 0 logs, suppressed increases 4 -> 9.
            for _ in 0..5 {
                rig.step();
                if owner == Owner::Executor {
                    rig.inner_mut().safety.drain_log_events_off_thread();
                }
            }
            assert!(
                lines.lock().unwrap().is_empty(),
                "{owner:?}: rapid clear/re-fault must not bypass 1/s window: {:?}",
                lines.lock().unwrap()
            );
            assert_eq!(rig.inner().hold_tx_error_suppressed(), 9);

            // Advance past t = 3000 ms while preserving an observed-empty RX queue bound <= 100 ms
            // (R10 stopped-hold requires provably fresh feedback): next error emits with exact
            // `suppressed 9 events`!
            rig.clock.advance(ms(830));
            let _ = rig.inner().driver.as_ref().unwrap().lock().unwrap().read_joint_states();
            rig.clock.advance(ms(20));
            rig.step();
            if owner == Owner::Executor {
                rig.inner_mut().safety.drain_log_events_off_thread();
            }
            let after_clear = lines.lock().unwrap().clone();
            assert_eq!(after_clear.len(), 1, "{owner:?}: expected 1 log after 1 s elapsed: {after_clear:?}");
            assert!(
                after_clear[0].contains("suppressed 9 events"),
                "{owner:?}: expected suppressed 9 events across clear/re-fault, got: {}",
                after_clear[0]
            );
            assert_eq!(rig.inner().hold_tx_error_suppressed(), 0);

            // Deferred-ring overflow retention (Executor path): fill SAFETY_LOG_CAPACITY with
            // unrelated events so `log_stopped_hold_output_failed` returns false -> count retained,
            // 0 dropped, and emitted immediately once ring drains!
            if owner == Owner::Executor {
                lines.lock().unwrap().clear();
                rig.clock.advance(ms(1030));
                let _ = rig.inner().driver.as_ref().unwrap().lock().unwrap().read_joint_states();
                rig.clock.advance(ms(20));
                for _ in 0..armd::safety::SAFETY_LOG_CAPACITY {
                    rig.inner_mut().safety.log_gravity_comp_ramp_complete();
                }
                let dropped_before = rig.inner().safety.log_dropped();
                rig.step(); // Ring full -> retains count (+1), does not drop or advance throttle!
                assert_eq!(rig.inner().safety.log_dropped(), dropped_before);
                assert_eq!(rig.inner().hold_tx_error_suppressed(), 1);
                // Drain unrelated events; very next error tick retries immediately (suppressed = 1)!
                rig.inner_mut().safety.drain_log_events_off_thread();
                rig.step();
                rig.inner_mut().safety.drain_log_events_off_thread();
                let flushed = lines.lock().unwrap().clone();
                assert_eq!(flushed.len(), 1, "expected immediate retry emit after ring drained: {flushed:?}");
                assert!(
                    flushed[0].contains("suppressed 1 events"),
                    "expected retained suppressed 1 events, got: {}",
                    flushed[0]
                );
                assert_eq!(rig.inner().hold_tx_error_suppressed(), 0);
            }
        });
    }
}

/// Item 1 + Item 4 (gravity compensation transient & sustained J7 staleness + real_flush +
/// stop-stamp change during J7 refresh):
/// - Gravity comp with transient (< 250 ms) J7 loss stays Active (arm-only feedback freshness)
///   and exits/hands back cleanly without wedging.
/// - Gravity comp with sustained (>= 250 ms) J7 loss latches recoverable `J7_FEEDBACK_STALE` and
///   exits gravity comp cleanly via `gravity_comp_exit_cause`.
/// - Real-flush transport (`real_flush = true`) flushes J7 `0xCC` refresh without miscounting it
///   as a hold frame, and a mid-refresh `StopRequests::request()` returns `EstopInhibited`.
#[test]
fn pos_force_gravity_comp_and_real_flush_j7_staleness() {
    // Part A: Gravity comp transient (< 250 ms) & sustained (>= 250 ms) J7 staleness.
    let mut cfg = base_cfg();
    if let Some(ref mut gc) = cfg.gravity_comp {
        gc.ramp_in_s = 0.04;
        gc.exit_hold_ms = 40;
    }
    let mut rig = DualRig::new_pos_force(Owner::Legacy, cfg, None);
    rig.bus.with(|s| s.immediate_replies = true);
    {
        let inner = rig.inner_mut();
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).expect("admit enter gravity comp");
        SupervisorInner::enter_gravity_comp(
            admitted,
            armd::gravity_comp::GravityCompRequest::default(),
        )
        .expect("enter gravity comp");
    }
    rig.bus.with(|s| s.immediate_replies = false);
    for _ in 0..5 {
        rig.step();
    }
    assert!(rig.inner().gravity_comp.is_some(), "expected active gravity comp");

    // Transient 120 ms J7 loss (< 250 ms) while J1-J6 stay fresh: gravity comp stays active!
    rig.bus.with(|s| s.motors[6].mute_feedback = true);
    for _ in 0..6 {
        rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
        rig.step_dt(DT, false);
        assert!(
            rig.inner().safety.state().is_operational(),
            "transient J7 staleness in gravity comp must not fault: {:?}",
            rig.inner().safety.fault()
        );
        assert!(rig.inner().gravity_comp.is_some(), "transient J7 staleness must not kick out gravity comp");
    }
    // Exit gravity comp while J1-J6 are fresh (J7 still transiently stale, total 200 ms < 250 ms):
    // arm-only handback must complete without wedging or faulting!
    rig.bus.with(|s| s.immediate_replies = true);
    rig.inner_mut().exit_gravity_comp("STOPPED", "operator exit");
    for _ in 0..4 {
        rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
        rig.step_dt(DT, false);
    }
    assert!(rig.inner().gravity_comp.is_none(), "arm-only handback must complete without wedging on stale J7");
    assert!(rig.inner().safety.state().is_operational(), "transient J7 loss (< 250 ms) must stay operational");

    // Re-enter with fresh J7, then mute J7 >= 250 ms -> latches J7_FEEDBACK_STALE and exits cleanly.
    rig.bus.with(|s| {
        s.motors[6].mute_feedback = false;
        s.push_all_feedback();
    });
    rig.step();
    {
        let inner = rig.inner_mut();
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).expect("admit re-enter gravity comp");
        SupervisorInner::enter_gravity_comp(
            admitted,
            armd::gravity_comp::GravityCompRequest::default(),
        )
        .expect("re-enter gravity comp");
    }
    rig.bus.with(|s| s.immediate_replies = false);
    for _ in 0..4 {
        rig.step();
    }
    rig.bus.with(|s| s.motors[6].mute_feedback = true);
    for _ in 0..16 {
        rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
        rig.step_dt(DT, false);
    }
    assert_eq!(rig.inner().safety.fault_code(), Some(FaultCode::J7FeedbackStale));
    assert!(rig.inner().safety.recoverable());

    // Part B: Real-flush (`real_flush = true`) & stop-stamp change during J7 0xCC refresh.
    let limits = dm_limits();
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&limits, clock.clone());
    bus.with(|s| {
        s.motors[6].set_reg_u32(
            armd::protocol::damiao::RID_CTRL_MODE,
            armd::protocol::damiao::CTRL_MODE_POS_FORCE,
        );
        s.immediate_replies = true;
        s.real_flush = true;
        s.push_all_feedback();
    });
    let grip = armd::driver::gripper_drive_from_profile(&base_cfg()).unwrap();
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap()
        .with_gripper_drive(grip)
        .unwrap();
    let stops = StopRequests::new();
    drv.set_estop_gate(EstopGate::new(
        Arc::new(AtomicU64::new(0)),
        Arc::new(AtomicU64::new(0)),
    ).with_stops(stops.clone()));
    drv.read_joint_states().unwrap();
    drv.read_joint_states().unwrap();
    drv.check_mapping_ranges();
    drv.set_torque(true).unwrap();

    // Advance 110 ms across two RX passes with J1-J6 fresh and J7 muted so J1-J6 have
    // queue_bound <= 10 ms (fresh) while J7 is > 100 ms stale -> stopped-hold sends J1-J6 + 0xCC.
    bus.with(|s| s.motors[6].mute_feedback = true);
    clock.advance(ms(100));
    bus.with(|s| for i in 0..6 { s.push_feedback(i); });
    drv.read_joint_states().unwrap();
    clock.advance(ms(10));
    bus.with(|s| for i in 0..6 { s.push_feedback(i); });
    drv.read_joint_states().unwrap();
    bus.clear_log();
    drv.begin_tx_tick();
    let holds = vec![JointTarget::position(0.0); 7];
    assert_eq!(drv.write_estop_hold_bounded(&holds).unwrap(), TxBatchOutcome::Complete);
    assert!(drv.last_hold_tx_sent_any(), "J1-J7 hold/freeze frames were sent and flushed");
    assert!(!j7_refresh_frames(&bus.tx_log()).is_empty(), "0xCC refresh sent on real_flush bus");
    assert_j7_freeze_frames(
        &j7_pf_frames(&bus.tx_log()),
        0.0,
        j7_pf_current_raw(&base_cfg()),
        "real_flush stale J7 stopped-hold",
    );

    // Stop-stamp changes during J7 refresh attempt -> returns EstopInhibited!
    let stops_trigger = stops.clone();
    bus.with(|s| {
        s.on_tx = Some(Arc::new(move || {
            let _ = stops_trigger.request();
        }));
    });
    drv.begin_tx_tick();
    let err = drv.write_estop_hold_bounded(&holds).unwrap_err();
    assert!(is_estop_inhibited(&err), "stop during refresh/hold must return EstopInhibited, got {err:?}");
}

/// Blocker 1 regression (both Legacy and RT Executor):
/// When J7 reaches `feedback_stale_fault_ms` (250 ms) in the SAME release that a J1–J6 motion
/// transmit fails (`TickOutput::Error`) or motion FIFO expires (`TickOutput::Expired`), the
/// recoverable `J7_FEEDBACK_STALE` MUST NOT mask the non-recoverable `DRIVER_WRITE_FAILED` /
/// `CAN_TX_STALLED` latch. Both conditions are recorded, the non-recoverable fault wins, and
/// clearing still enforces fresh J7 feedback.
#[test]
fn dual_owner_same_release_j7_staleness_never_masks_motion_write_or_expiry_failure() {
    use armd::safety::FaultKind;

    for owner in [Owner::Legacy, Owner::Executor] {
        // (a) Same-release J7 >= 250 ms + hard motion TX write failure -> DRIVER_WRITE_FAILED wins.
        let mut rig = DualRig::new_pos_force(owner, base_cfg(), None);
        let ep = rig.inner().safety.epoch_id();
        {
            let inner = rig.inner_mut();
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).expect("admit motion");
            SupervisorInner::jog_action(
                admitted,
                vec![0.3, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
                Some(0.6),
                Some(ep),
                "joint_move",
            )
            .expect("start motion");
        }
        rig.bus.with(|s| s.motors[6].mute_feedback = true);
        // Advance 11 normal ticks (220 ms < 250 ms): J7 stale-skipped, J1–J6 moving, operational.
        for _ in 0..11 {
            rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            rig.step_dt(DT, false);
            assert!(rig.inner().safety.state().is_operational(), "{owner:?}: faulted early");
        }
        // Advance to 260 ms (>= 250 ms threshold) AND inject hard TX error on the exact same release!
        rig.clock.advance(ms(40));
        rig.bus.with(|s| {
            for i in 0..6 {
                s.push_feedback(i);
            }
            s.tx_error = true;
        });
        rig.step_dt(DT, false);
        assert_eq!(
            rig.inner().safety.fault_code(),
            Some(FaultCode::DriverWriteFailed),
            "{owner:?}: same-release J7 staleness MUST NOT mask non-recoverable DRIVER_WRITE_FAILED"
        );
        assert!(!rig.inner().safety.recoverable(), "{owner:?}: combined fault must be non-recoverable");
        assert!(rig.inner().safety.has_latched_fault_kind(FaultKind::Other));
        assert!(rig.inner().safety.has_latched_fault_kind(FaultKind::J7FeedbackStale));
        assert!(rig.inner().safety.has_latched_fault_code(FaultCode::DriverWriteFailed));
        assert!(rig.inner().safety.has_latched_fault_code(FaultCode::J7FeedbackStale));

        // Even with J1–J6 fresh and tx_error cleared, clear_fault still refuses while J7 is stale!
        rig.bus.with(|s| {
            s.tx_error = false;
            for i in 0..6 {
                s.push_feedback(i);
            }
        });
        rig.step_dt(DT, false);
        match &mut rig.owner {
            RigOwner::Legacy(inner) => {
                let err = inner.clear_fault().unwrap_err().to_string();
                assert!(err.contains("J7") || err.contains("feedback"), "{err}");
            }
            RigOwner::Executor { fx, .. } => {
                let dl = armd::transport::DriverClock::now(&rig.clock) + ms(200);
                assert_eq!(
                    fx.core.clear_fault_executor_bounded(dl).unwrap_err(),
                    armd::safety::ClearFaultError::Rejected(armd::safety::ClearRefusal::J7FeedbackStale)
                );
            }
        }

        // (b) Same-release J7 >= 250 ms + motion FIFO expiry -> CAN_TX_STALLED wins (non-recoverable).
        let b4 = TxBudget { max_frames: 4, max_duration: ms(10), frame_max_age: ms(45) };
        let mut rig_exp = DualRig::new_pos_force(owner, base_cfg(), Some(b4));
        let ep_exp = rig_exp.inner().safety.epoch_id();
        {
            let inner = rig_exp.inner_mut();
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).expect("admit motion");
            SupervisorInner::jog_action(
                admitted,
                vec![0.3, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
                Some(0.6),
                Some(ep_exp),
                "joint_move",
            )
            .expect("start motion");
        }
        rig_exp.bus.with(|s| s.motors[6].mute_feedback = true);
        // Advance 11 ticks (220 ms): J1–J6 drain cleanly (budget 4 -> 2 deferred per tick, drain next tick).
        for _ in 0..10 {
            rig_exp.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            rig_exp.step_dt(DT, false);
        }
        // Ensure a motion frame is queued in pending_tx, then advance 50 ms (> 45 ms frame_max_age,
        // and J7 crosses >= 250 ms) so THIS release returns `TickOutput::Expired` + J7 >= 250 ms!
        rig_exp.bus.with(|s| {
            s.tx_stall = true;
            for i in 0..6 {
                s.push_feedback(i);
            }
        });
        rig_exp.step_dt(DT, false);
        rig_exp.clock.advance(ms(55));
        rig_exp.bus.with(|s| {
            s.tx_stall = false;
            for i in 0..6 {
                s.push_feedback(i);
            }
        });
        rig_exp.step_dt(DT, false);
        assert_eq!(
            rig_exp.inner().safety.fault_code(),
            Some(FaultCode::CanTxStalled),
            "{owner:?}: same-release J7 staleness MUST NOT mask non-recoverable CAN_TX_STALLED"
        );
        assert!(!rig_exp.inner().safety.recoverable());
        assert!(rig_exp.inner().safety.has_latched_fault_kind(FaultKind::CanTxStalled));
        assert!(rig_exp.inner().safety.has_latched_fault_kind(FaultKind::J7FeedbackStale));
    }
}

/// Blocker 2 dual-owner regression (Legacy + RT Executor):
/// Invalidating J7 verification/mode (e.g. disabling torque) after `J7_FEEDBACK_STALE` latches
/// MUST NOT bypass the `<= 100 ms` clear check; J7 at 101..=250 ms is refused, and J7 at exact
/// 100 ms succeeds.
#[test]
fn dual_owner_j7_clear_fail_closed_when_unverified_and_at_100ms_boundary() {
    for owner in [Owner::Legacy, Owner::Executor] {
        let mut rig = DualRig::new_pos_force(owner, base_cfg(), None);
        rig.bus.with(|s| s.motors[6].mute_feedback = true);
        for _ in 0..16 {
            rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            rig.step_dt(DT, false);
            if rig.inner().safety.fault_code() == Some(FaultCode::J7FeedbackStale) {
                break;
            }
        }
        assert_eq!(rig.inner().safety.fault_code(), Some(FaultCode::J7FeedbackStale));

        // Disable torque so driver invalidates active pos-force mode / verification, then deliver
        // J7 feedback aged 150 ms (101..=250 ms, so `is_feedback_valid()` is true while J7 > 100 ms).
        // Run 3 control releases at rest so R10's K=3 rest-pose gate authorizes `set_torque(false)`.
        rig.bus.with(|s| {
            s.motors[6].mute_feedback = false;
            s.immediate_replies = true;
            s.push_all_feedback();
        });
        for _ in 0..3 {
            rig.step();
        }
        {
            let mut drv = rig.inner().driver.as_ref().unwrap().lock().unwrap();
            drv.set_torque(false).expect("disable torque");
        }
        rig.bus.with(|s| {
            s.immediate_replies = false;
            s.motors[6].mute_feedback = true;
        });
        rig.clock.advance(ms(140));
        rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
        rig.step_dt(0.01, false);
        assert!(
            rig.inner().driver.as_ref().unwrap().lock().unwrap().is_feedback_valid(),
            "{owner:?}: general 250 ms feedback validity is true at 150 ms"
        );
        match &mut rig.owner {
            RigOwner::Legacy(inner) => {
                let err = inner.clear_fault().unwrap_err().to_string();
                assert!(err.contains("J7") || err.contains("feedback"), "{owner:?}: {err}");
            }
            RigOwner::Executor { fx, .. } => {
                let dl = armd::transport::DriverClock::now(&rig.clock) + ms(200);
                assert_eq!(
                    fx.core.clear_fault_executor_bounded(dl).unwrap_err(),
                    armd::safety::ClearFaultError::Rejected(armd::safety::ClearRefusal::J7FeedbackStale),
                    "Executor must refuse unverified J7 at 150 ms"
                );
            }
        }

        // Exact inclusive 100 ms boundary -> clears cleanly!
        rig.bus.with(|s| {
            s.motors[6].mute_feedback = false;
            s.push_all_feedback();
        });
        // Establish fresh empty-RX boundary + decode sample at t=0, then advance exact 100 ms.
        {
            let mut drv = rig.inner().driver.as_ref().unwrap().lock().unwrap();
            drv.read_joint_states().unwrap();
        }
        rig.bus.with(|s| s.push_all_feedback());
        if owner == Owner::Executor {
            let now = armd::transport::DriverClock::now(&rig.clock);
            let _ = rig.inner_mut().poll_hardware(&armd::rt_core::ops::ServiceBudget::for_release(now, now + ms(20)));
        } else {
            rig.inner().driver.as_ref().unwrap().lock().unwrap().read_joint_states().unwrap();
        }
        rig.clock.advance(ms(100));
        match &mut rig.owner {
            RigOwner::Legacy(inner) => {
                assert_eq!(inner.clear_fault().expect("exact 100 ms must clear"), SafetyState::Idle);
            }
            RigOwner::Executor { fx, .. } => {
                let dl = armd::transport::DriverClock::now(&rig.clock) + ms(200);
                assert_eq!(
                    fx.core.clear_fault_executor_bounded(dl).expect("exact 100 ms must clear"),
                    SafetyState::Idle
                );
            }
        }
    }
}

/// R2 Major 2 dual-owner regression (Legacy + RT Executor):
/// Backlog stall accounting (`CAN_TX_STALLED` transition or subsequent backlog stall tick —
/// including suppressed RT deferred-logging ticks that emit nothing) MUST use its own throttle
/// and NEVER suppress a genuine `"Stopped hold output failed"` hard write error on the next release.
#[test]
fn dual_owner_backlog_stall_diagnostic_never_suppresses_stopped_hold_output_failed() {
    use std::sync::Mutex;

    struct CaptureHoldErrors(Arc<Mutex<Vec<String>>>);
    struct MsgVisitor(String);
    impl tracing::field::Visit for MsgVisitor {
        fn record_debug(&mut self, field: &tracing::field::Field, value: &dyn std::fmt::Debug) {
            if field.name() == "message" {
                self.0 = format!("{value:?}");
            }
        }
    }
    impl tracing::Subscriber for CaptureHoldErrors {
        fn enabled(&self, meta: &tracing::Metadata<'_>) -> bool {
            *meta.level() <= tracing::Level::ERROR
        }
        fn new_span(&self, _: &tracing::span::Attributes<'_>) -> tracing::span::Id {
            tracing::span::Id::from_u64(1)
        }
        fn record(&self, _: &tracing::span::Id, _: &tracing::span::Record<'_>) {}
        fn record_follows_from(&self, _: &tracing::span::Id, _: &tracing::span::Id) {}
        fn event(&self, event: &tracing::Event<'_>) {
            let mut v = MsgVisitor(String::new());
            event.record(&mut v);
            if v.0.contains("Stopped hold output") {
                self.0.lock().unwrap().push(v.0);
            }
        }
        fn enter(&self, _: &tracing::span::Id) {}
        fn exit(&self, _: &tracing::span::Id) {}
    }

    for owner in [Owner::Legacy, Owner::Executor] {
        let mut rig = DualRig::new(owner, base_cfg(), None);
        if owner == Owner::Executor {
            rig.inner_mut().safety.set_off_thread_reclaim(true);
        }
        rig.inner_mut()
            .safety
            .trigger_fault_coded(FaultCode::Other, FaultReason::Other("backlog-then-error".into()));
        rig.inner_mut().safety.drain_log_events_off_thread();

        // 1. Stall stopped-hold backlog until `CAN_TX_STALLED` latches, plus 1 extra backlog tick
        //    (testing both transition and already-latched backlog accounting).
        rig.bus.with(|s| s.tx_stall = true);
        let mut stalled = false;
        for _ in 0..8 {
            rig.step();
            if owner == Owner::Executor {
                rig.inner_mut().safety.drain_log_events_off_thread();
            }
            if rig.inner().safety.fault_code() == Some(FaultCode::CanTxStalled) {
                stalled = true;
                break;
            }
        }
        assert!(stalled, "{owner:?}: expected stopped-hold CAN_TX_STALLED");
        // One more already-latched backlog tick right before the hard write error.
        rig.step();
        if owner == Owner::Executor {
            rig.inner_mut().safety.drain_log_events_off_thread();
        }

        // 2. Immediately on the very next release (20 ms later, well inside 1 s!), inject a hard
        //    write error: `"Stopped hold output failed"` MUST emit immediately (NOT suppressed)!
        let lines = Arc::new(Mutex::new(Vec::<String>::new()));
        let sub = CaptureHoldErrors(Arc::clone(&lines));
        tracing::subscriber::with_default(sub, || {
            rig.bus.with(|s| {
                s.tx_stall = false;
                s.tx_error = true;
            });
            rig.step();
            if owner == Owner::Executor {
                rig.inner_mut().safety.drain_log_events_off_thread();
            }
        });
        let captured = lines.lock().unwrap().clone();
        assert_eq!(
            captured.len(),
            1,
            "{owner:?}: backlog stall must NEVER suppress immediate Stopped hold output failed: {captured:?}"
        );
        assert_eq!(rig.inner().hold_tx_error_suppressed(), 0);
    }
}

/// R3 Major 2: a provably fresh J7 RX sample arriving between two stale output passes MUST start
/// a fresh staleness episode (bound to `pf_fresh_gen`), so an earlier stale output cannot make
/// the next stale output prematurely trip `J7_FEEDBACK_STALE` when the newest sample is < limit.
#[test]
fn dual_owner_fresh_j7_rx_between_two_stale_outputs_resets_staleness_episode() {
    for owner in [Owner::Legacy, Owner::Executor] {
        let mut rig = DualRig::new_pos_force(owner, base_cfg(), None);
        // 1. Sample #1 at t = 0; mute J7 and advance 105 ms -> first stale output (105 ms < 250 ms).
        rig.bus.with(|s| s.motors[6].mute_feedback = true);
        rig.clock.advance(ms(105));
        rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
        rig.step_dt(DT, false);
        assert!(
            rig.inner().safety.state().is_operational(),
            "{owner:?}: 105 ms stale must not fault yet: {:?}",
            rig.inner().safety.fault()
        );

        // 2. At t = 240 ms, a provably fresh J7 RX sample (#2) arrives via RX poll ONLY (no
        //    output pass observes it while fresh!), establishing a new empty-queue bound and
        //    advancing `pf_fresh_gen`.
        rig.clock.advance(ms(135));
        rig.bus.with(|s| {
            s.motors[6].mute_feedback = false;
            s.push_all_feedback();
        });
        {
            let drv = rig.inner().driver.as_ref().unwrap();
            let mut g = drv.lock().unwrap();
            // First poll drains/sets rx_empty_instant at t = 240 ms; second decodes bounded fresh J7.
            let _ = g.read_joint_states().unwrap();
            rig.bus.with(|s| s.push_all_feedback());
            let _ = g.read_joint_states().unwrap();
            assert!(g.pos_force_feedback_fresh(), "{owner:?}: sample #2 must be provably fresh at t=240 ms");
        }
        rig.bus.with(|s| s.motors[6].mute_feedback = true);

        // 3. Output resumes at t = 350 ms: 110 ms since fresh sample #2 (350 ms since sample #1!).
        //    Because sample #2 started a new episode (`pf_fresh_gen` advanced), J7 is only 110 ms
        //    stale (< 250 ms) and MUST NOT prematurely latch `J7_FEEDBACK_STALE`!
        rig.clock.advance(ms(110));
        rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
        rig.step_dt(DT, false);
        assert!(
            rig.inner().safety.state().is_operational(),
            "{owner:?}: 110 ms since fresh sample #2 must NOT latch J7_FEEDBACK_STALE (old timer leaked): {:?}",
            rig.inner().safety.fault()
        );

        // 4. Advance 150 ms more (260 ms since sample #2 >= 250 ms limit) -> NOW latches recoverable
        //    `J7_FEEDBACK_STALE` cleanly!
        rig.clock.advance(ms(150));
        rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
        rig.step_dt(DT, false);
        assert_eq!(
            rig.inner().safety.fault_code(),
            Some(FaultCode::J7FeedbackStale),
            "{owner:?}: expected J7_FEEDBACK_STALE once sample #2 exceeds 250 ms"
        );
        assert!(rig.inner().safety.recoverable());
    }
}

/// R3 Major 3: on RT Executor, an authorized stop applied before Step 2 RX causes `receive_feedback`
/// to drop older-generation holds via `drop_stopped_hold_tx`. Step 5's newly queued holds under the
/// new stop generation MUST NOT inherit carried `hold_key_ages` from the previous stop generation.
#[test]
fn rt_executor_stop_generation_emptied_in_rx_never_inherits_prior_hold_backlog_age() {
    let mut rig = DualRig::new(Owner::Executor, base_cfg(), None);
    // Wire the executor's `StopRequests` into `core` and `drv.estop_gate` exactly like `spawn_executor`.
    match &mut rig.owner {
        RigOwner::Executor { fx, .. } => {
            let stops = fx.stops().clone();
            fx.core.attach_stop_requests(stops);
            let gate = fx.core.estop_gate();
            fx.core.driver.as_ref().unwrap().lock().unwrap().set_estop_gate(gate);
        }
        RigOwner::Legacy(_) => unreachable!(),
    }
    rig.inner_mut()
        .safety
        .trigger_fault_coded(FaultCode::Other, FaultReason::Other("stop-gen-reset".into()));
    assert_eq!(rig.inner().safety.fault_code(), Some(FaultCode::Other));

    // 1. Stall stopped-hold TX for 2 ticks (20 ms elapsed on queued hold keys, < 45 ms limit):
    //    `CAN_TX_STALLED` must NOT latch yet, and `hold_key_ages` now holds ~20 ms of age.
    rig.bus.with(|s| s.tx_stall = true);
    rig.step(); // tick 1: stages keys at t0 (age 0 ms)
    rig.step(); // tick 2: +20 ms (age 20 ms < 45 ms)
    assert_eq!(rig.inner().safety.fault_code(), Some(FaultCode::Other));

    // 2. Request an authorized stop (bumping `stop_gen`!) and advance 30 ms (> 45 ms combined with
    //    prior 20 ms if the old age leaked!). During this release: Step 1 applies the stop, Step 2
    //    `receive_feedback` calls `drop_stopped_hold_tx` (clearing both old holds AND old ages),
    //    and Step 5 stages fresh holds under the new generation (age 0 ms!).
    match &mut rig.owner {
        RigOwner::Executor { fx, .. } => {
            fx.stops().request();
        }
        RigOwner::Legacy(_) => unreachable!(),
    }
    rig.clock.advance(ms(30));
    rig.bus.with(|s| s.push_all_feedback());
    rig.step_dt(DT, false);
    assert_eq!(
        rig.inner().safety.fault_code(),
        Some(FaultCode::Other),
        "new stop generation must NOT inherit prior-generation hold backlog age and latch CAN_TX_STALLED"
    );

    // 3. Only after a genuine new >= 45 ms backlog accumulates under the new generation (3 × 20 ms
    //    = 60 ms >= 45 ms) does `CAN_TX_STALLED` latch.
    let mut stalled = false;
    for _ in 0..4 {
        rig.step();
        if rig.inner().safety.fault_code() == Some(FaultCode::CanTxStalled) {
            stalled = true;
            break;
        }
    }
    assert!(stalled, "genuine new >= 45 ms backlog under the new stop generation must latch CAN_TX_STALLED");
}

/// R3 Major 1: when `J7_FEEDBACK_STALE` is co-latched with `HardwareFault` and J7 feedback is
/// stale (`> 100 ms`), BOTH Legacy `clear_fault` and RT Executor `clear_fault_executor` refuse
/// with `J7_FEEDBACK_STALE` BEFORE issuing any physical motor fault-reset (`0xFB`) command.
#[test]
fn clear_fault_refuses_hardware_reset_when_j7_feedback_is_stale() {
    for owner in [Owner::Legacy, Owner::Executor] {
        let mut rig = DualRig::new_pos_force(owner, base_cfg(), None);
        // Latch both J7FeedbackStale and HardwareFault (motor error code 0x0B on J1), clearing any
        // pre-fault buffered `FakeBus` replies from rig setup so `clear_fault`'s read sees live state.
        rig.bus.with(|s| {
            s.rx.clear();
            s.pending.clear();
            s.motors[0].state_override = Some(0x0B);
            s.motors[6].mute_feedback = true;
        });
        rig.clock.advance(ms(260));
        rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
        rig.step_dt(DT, false);
        assert!(rig.inner().safety.has_latched_fault_kind(armd::safety::FaultKind::J7FeedbackStale));
        // Push one more live J1..J6 cycle (with J1 still at 0x0B, J7 muted) so `s.pending` has no
        // non-fault replies and `clear_fault` observes both live HardwareFault and stale J7.
        rig.bus.with(|s| {
            s.rx.clear();
            s.pending.clear();
            for i in 0..6 {
                s.push_feedback(i);
            }
        });

        rig.bus.clear_log();
        let err = match owner {
            Owner::Legacy => rig.inner_mut().clear_fault().unwrap_err(),
            Owner::Executor => {
                let deadline = std::time::Instant::now() + Duration::from_secs(1);
                rig.inner_mut().clear_fault_executor(deadline).unwrap_err()
            }
        };
        let err_msg = err.to_string();
        assert!(
            err_msg.contains("J7") || err_msg.contains("gripper") || err_msg.contains("pos-force"),
            "{owner:?}: stale J7 must refuse before hardware fault reset: {err:?} ({err_msg})"
        );
        let reset_frames: Vec<_> = rig
            .bus
            .tx_log()
            .into_iter()
            .filter(|f| f.data == [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB])
            .collect();
        assert!(
            reset_frames.is_empty(),
            "{owner:?}: must emit ZERO 0xFB hardware-reset frames when J7 is stale: {reset_frames:?}"
        );
    }
}

/// R3 Minor 1: unfenced J1–J6 MIT (`0x001..=0x006`) + J7 PosForce (`0x307`) dual-owner `home`
/// regression verifying that during an 80+ ms control-loop stall all six MIT frames continue
/// transmitting while `0x307` is absent and `0xCC` polls J7, then `0x307` resumes cleanly.
#[test]
fn dual_owner_mit_j1_j6_posforce_j7_80ms_stall_during_home_continues_mit_frames() {
    for owner in [Owner::Legacy, Owner::Executor] {
        let mut rig = DualRig::new_pos_force(owner, base_cfg(), None);
        // Switch J1–J6 into Damiao MIT mode while J7 remains verified `PosForce`.
        {
            rig.bus.with(|s| {
                s.immediate_replies = true;
                for m in &mut s.motors[0..6] {
                    m.set_reg_u32(
                        armd::protocol::damiao::RID_CTRL_MODE,
                        armd::protocol::damiao::CTRL_MODE_MIT,
                    );
                }
            });
            let holds: Vec<armd::driver::JointTarget> = (0..7)
                .map(|i| armd::driver::JointTarget {
                    position: rig.inner().measured_positions[i],
                    velocity: Some(0.0),
                    kp: Some(40.0),
                    kd: Some(2.0),
                    torque: Some(0.0),
                })
                .collect();
            let modes = [
                armd::driver::JointMode::Mit,
                armd::driver::JointMode::Mit,
                armd::driver::JointMode::Mit,
                armd::driver::JointMode::Mit,
                armd::driver::JointMode::Mit,
                armd::driver::JointMode::Mit,
                armd::driver::JointMode::Position,
            ];
            let drv = rig.inner().driver.as_ref().unwrap();
            drv.lock().unwrap().set_joint_modes(&modes, &holds).expect("switch J1–J6 to MIT");
            rig.bus.with(|s| {
                s.immediate_replies = false;
                s.push_all_feedback();
            });
            rig.step();
        }

        let ep = rig.inner().safety.epoch_id();
        let (_, _home_id, _) = {
            let inner = rig.inner_mut();
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).expect("admit MIT+PosForce home");
            SupervisorInner::jog_action(admitted, vec![0.0; 7], Some(0.6), Some(ep), "home")
                .expect("start MIT+PosForce home")
        };
        for _ in 0..3 {
            rig.step();
            assert!(rig.inner().safety.state().is_operational());
        }

        // Inject 80+ ms stall (105 ms queue-bound gap): J7 > 100 ms sends freeze frame (`vel=0`,
        // last fresh pos) + 0xCC refresh while all six J1–J6 MIT frames (`0x001..=0x006`) transmit cleanly!
        let last_fresh_j7 = rig.inner().measured_positions[6];
        rig.bus.with(|s| s.push_all_feedback());
        rig.clock.advance(ms(105));
        rig.bus.clear_log();
        rig.step_dt(DT, false);

        assert!(
            rig.inner().safety.state().is_operational(),
            "{owner:?}: MIT+PosForce 80+ ms stall must not fault: {:?}",
            rig.inner().safety.fault()
        );
        let stale_log = rig.bus.tx_log();
        assert_j7_freeze_frames(
            &j7_pf_frames(&stale_log),
            last_fresh_j7,
            j7_pf_current_raw(&base_cfg()),
            &format!("{owner:?} MIT+PosForce 105 ms stall"),
        );
        for id in 1u32..=6 {
            assert!(
                stale_log.iter().any(|f| f.id == id),
                "{owner:?}: expected MIT wire frame 0x{id:03X} for J{id} while J7 frozen: {stale_log:?}"
            );
        }
        assert!(
            !j7_refresh_frames(&stale_log).is_empty(),
            "{owner:?}: expected J7 0xCC refresh: {stale_log:?}"
        );

        // Next fresh tick: moving/fresh 0x307 resumes alongside J1–J6 MIT frames!
        rig.bus.clear_log();
        rig.step();
        let fresh_log = rig.bus.tx_log();
        assert!(
            j7_pf_frames(&fresh_log)
                .iter()
                .any(|f| u16::from_le_bytes([f.data[4], f.data[5]]) > 0),
            "{owner:?}: expected fresh moving 0x307 (vel_raw > 0) to resume once J7 feedback is fresh"
        );
        for id in 1u32..=6 {
            assert!(
                fresh_log.iter().any(|f| f.id == id),
                "{owner:?}: expected MIT wire frame 0x{id:03X} after J7 recovery"
            );
        }
    }
}

/// Round 4 Blocker regression (Legacy + RT Executor):
/// 1. Stale J7 (> 100 ms) while closing, then STOP -> next tick sends a `0x307` FREEZE frame
///    (`vel_raw == 0`, `pos == last fresh measured J7 position`, same current cap), NEVER the
///    closing target.
/// 2. Same for E-stop (including past 250 ms while < 500 ms `BUS_LOSS`).
/// 3. While J7 is stale (> 100 ms, before 250 ms fault): a running gripper action ends immediately
///    with `J7_FEEDBACK_STALE`, no new gripper targets are accepted (`start_gripper`, `jog_action`
///    changing J7 refused with `J7_FEEDBACK_STALE`; `servo` holds J7 frozen while J1–J6 continue),
///    and NO moving `0x307` target is ever sent on the wire while stale.
#[test]
fn dual_owner_stale_j7_closing_stop_and_estop_send_freeze_frame_and_refuse_new_targets() {
    use armd::safety::StopPolicy;

    let pf_cur = j7_pf_current_raw(&base_cfg());
    for owner in [Owner::Legacy, Owner::Executor] {
        // --- (1) Stale J7 while closing, then STOP -> next tick sends 0x307 freeze frame ---
        {
            let mut rig = DualRig::new_pos_force(owner, base_cfg(), None);
            // Wire `StopRequests` + `EstopGate` on Executor so `fx.stops().request()` exercises
            // the full urgent-stop path end to end.
            if let RigOwner::Executor { fx, .. } = &mut rig.owner {
                let stops = fx.stops().clone();
                fx.core.attach_stop_requests(stops);
                let gate = fx.core.estop_gate();
                fx.core.driver.as_ref().unwrap().lock().unwrap().set_estop_gate(gate);
            }
            // Position J7 open at -3.0 rad with fresh feedback.
            rig.bus.with(|s| {
                s.motors[6].q = -3.0;
                s.push_all_feedback();
            });
            rig.step();
            rig.inner_mut().commanded_positions[6] = -3.0;
            rig.inner_mut().target_positions[6] = -3.0;

            let ep = rig.inner().safety.epoch_id();
            let req = GripperRequest {
                command: GripperCommand::Close,
                fraction: None,
                grasp: false,
                speed: 0.30,
                timeout_s: Some(15.0),
                tag: None,
            };
            let (grip_id, _) = {
                let inner = rig.inner_mut();
                let stamp = inner.current_admission_stamp();
                let admitted = inner.check_admission(stamp).expect("admit gripper close");
                SupervisorInner::start_gripper(admitted, req, ep).expect("start gripper close")
            };
            // Run 5 fresh ticks so J7 actively closes toward 0.0 rad (moving 0x307 frames).
            for _ in 0..5 {
                rig.step();
                assert!(rig.inner().safety.state().is_operational());
            }
            assert!(
                rig.inner().last_action.as_ref().is_some_and(|a| a.id == grip_id && a.is_running()),
                "{owner:?}: gripper close should still be running"
            );
            let last_fresh_pos = rig.inner().measured_positions[6];
            assert!(
                last_fresh_pos < -2.0,
                "{owner:?}: J7 should still be mid-stroke far from closed 0.0, got {last_fresh_pos}"
            );

            // J7 goes stale (> 100 ms) while J1–J6 stay fresh, then STOP arrives!
            // Clear FakeBus `rx`/`pending` so no pre-mute J7 reply leaks out on a later WouldBlock.
            rig.bus.with(|s| {
                s.rx.clear();
                s.pending.clear();
                s.motors[6].mute_feedback = true;
            });
            rig.clock.advance(ms(115));
            rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            match &mut rig.owner {
                RigOwner::Legacy(inner) => {
                    assert!(inner.authorized_stop());
                }
                RigOwner::Executor { fx, .. } => {
                    fx.stops().request();
                }
            }
            rig.bus.clear_log();
            rig.step_dt(DT, false);

            let stop_log = rig.bus.tx_log();
            let stop_j7 = j7_pf_frames(&stop_log);
            assert_j7_freeze_frames(
                &stop_j7,
                last_fresh_pos,
                pf_cur,
                &format!("{owner:?} stale J7 closing + STOP"),
            );
            assert!(
                !j7_refresh_frames(&stop_log).is_empty(),
                "{owner:?}: expected J7 0xCC refresh after STOP on stale J7"
            );
        }

        // --- (2) Stale J7 while closing, then E-stop -> next tick sends 0x307 freeze frame ---
        {
            let mut rig = DualRig::new_pos_force(owner, base_cfg(), None);
            if let RigOwner::Executor { fx, .. } = &mut rig.owner {
                let stops = fx.stops().clone();
                fx.core.attach_stop_requests(stops);
                let gate = fx.core.estop_gate();
                fx.core.driver.as_ref().unwrap().lock().unwrap().set_estop_gate(gate);
            }
            rig.bus.with(|s| {
                s.motors[6].q = -3.0;
                s.push_all_feedback();
            });
            rig.step();
            rig.inner_mut().commanded_positions[6] = -3.0;
            rig.inner_mut().target_positions[6] = -3.0;

            let ep = rig.inner().safety.epoch_id();
            let req = GripperRequest {
                command: GripperCommand::Close,
                fraction: None,
                grasp: false,
                speed: 0.30,
                timeout_s: Some(15.0),
                tag: None,
            };
            {
                let inner = rig.inner_mut();
                let stamp = inner.current_admission_stamp();
                let admitted = inner.check_admission(stamp).expect("admit gripper close");
                SupervisorInner::start_gripper(admitted, req, ep).expect("start gripper close");
            }
            for _ in 0..5 {
                rig.step();
                assert!(rig.inner().safety.state().is_operational());
            }
            // J7 goes stale (> 100 ms) while J1–J6 stay fresh across normal ticks, then E-stop triggers!
            rig.bus.with(|s| {
                s.rx.clear();
                s.pending.clear();
                s.motors[6].mute_feedback = true;
            });
            for _ in 0..6 {
                rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
                rig.step_dt(DT, false);
            }
            // Capture the last fresh measured J7 position after any reply already queued prior to
            // muting J7 has been decoded (still mid-stroke far from closed 0.0).
            let last_fresh_pos = rig.inner().measured_positions[6];
            assert!(last_fresh_pos < -2.0);
            match &mut rig.owner {
                RigOwner::Legacy(inner) => {
                    inner.apply_stop_policy(StopPolicy::EStopLatch);
                }
                RigOwner::Executor { fx, .. } => {
                    fx.core.e_stop_latched.store(true, Ordering::SeqCst);
                    fx.core.requested_estop_count.fetch_add(1, Ordering::SeqCst);
                }
            }
            rig.bus.clear_log();
            rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            rig.step_dt(DT, false);
            assert_eq!(*rig.inner().safety.state(), SafetyState::EStop);
            let estop_log = rig.bus.tx_log();
            assert_j7_freeze_frames(
                &j7_pf_frames(&estop_log),
                last_fresh_pos,
                pf_cur,
                &format!("{owner:?} stale J7 closing + E-stop"),
            );
            for j in 1u32..=6 {
                assert!(
                    estop_log.iter().any(|f| f.id == 0x100 + j),
                    "{owner:?}: expected fresh J{j} stopped-hold alongside J7 freeze on E-stop"
                );
            }

            // Continue past 250 ms (to ~300 ms, still < 500 ms BUS_LOSS): E-stop STILL sends J7 freeze!
            for tick in 0..8 {
                rig.bus.clear_log();
                rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
                rig.step_dt(DT, false);
                assert_j7_freeze_frames(
                    &j7_pf_frames(&rig.bus.tx_log()),
                    last_fresh_pos,
                    pf_cur,
                    &format!("{owner:?} E-stop sustained stale tick {tick}"),
                );
            }

            // Extreme case: single 115 ms all-joint stall right before an E-stop tick (even when
            // J1–J6 feedback queue bound is > 100 ms stale), J7 MUST STILL receive its 0x307 freeze frame!
            rig.clock.advance(ms(115));
            rig.bus.clear_log();
            rig.step_dt(DT, false);
            assert_j7_freeze_frames(
                &j7_pf_frames(&rig.bus.tx_log()),
                last_fresh_pos,
                pf_cur,
                &format!("{owner:?} 115 ms all-joint stall + E-stop"),
            );
        }

        // --- (3) Running gripper action ends with J7_FEEDBACK_STALE; no new gripper target accepted/sent ---
        {
            let mut rig = DualRig::new_pos_force(owner, base_cfg(), None);
            rig.bus.with(|s| {
                s.motors[6].q = -3.0;
                s.push_all_feedback();
            });
            rig.step();
            rig.inner_mut().commanded_positions[6] = -3.0;
            rig.inner_mut().target_positions[6] = -3.0;

            let ep = rig.inner().safety.epoch_id();
            let req = GripperRequest {
                command: GripperCommand::Close,
                fraction: None,
                grasp: false,
                speed: 0.30,
                timeout_s: Some(15.0),
                tag: None,
            };
            let (grip_id, _) = {
                let inner = rig.inner_mut();
                let stamp = inner.current_admission_stamp();
                let admitted = inner.check_admission(stamp).expect("admit gripper close");
                SupervisorInner::start_gripper(admitted, req, ep).expect("start gripper close")
            };
            for _ in 0..4 {
                rig.step();
                assert!(rig.inner().safety.state().is_operational());
            }
            let last_fresh_pos = rig.inner().measured_positions[6];

            // J7 becomes stale (115 ms > 100 ms, still < 250 ms fault limit):
            // running gripper action MUST end immediately with J7_FEEDBACK_STALE, and wire sends
            // ONLY the 0x307 freeze frame (vel_raw == 0 at last_fresh_pos)!
            rig.bus.with(|s| {
                s.rx.clear();
                s.pending.clear();
                s.motors[6].mute_feedback = true;
            });
            rig.clock.advance(ms(115));
            rig.bus.clear_log();
            rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            rig.step_dt(DT, false);

            assert!(
                rig.inner().safety.state().is_operational(),
                "{owner:?}: 115 ms staleness must not prematurely latch 250 ms fault: {:?}",
                rig.inner().safety.fault()
            );
            let act = rig.inner().last_action.as_ref().expect("finished gripper action");
            assert_eq!(act.id, grip_id);
            assert_eq!(act.state, "failed");
            assert_eq!(act.code.as_deref(), Some("J7_FEEDBACK_STALE"));
            assert_eq!(
                rig.inner().action_outcome(armd::rt_core::broker::ActionId(grip_id)),
                Some(armd::rt_core::broker::Outcome::Failed("J7_FEEDBACK_STALE"))
            );
            assert_j7_freeze_frames(
                &j7_pf_frames(&rig.bus.tx_log()),
                last_fresh_pos,
                pf_cur,
                &format!("{owner:?} running gripper stale termination"),
            );

            // While J7 is still stale (< 250 ms, Operational): new gripper targets are REFUSED!
            let ep_now = rig.inner().safety.epoch_id();
            for cmd in [GripperCommand::Close, GripperCommand::Open] {
                let err = {
                    let inner = rig.inner_mut();
                    let stamp = inner.current_admission_stamp();
                    let admitted = inner.check_admission(stamp).expect("admit check");
                    SupervisorInner::start_gripper(
                        admitted,
                        GripperRequest {
                            command: cmd,
                            fraction: None,
                            grasp: false,
                            speed: 0.30,
                            timeout_s: Some(10.0),
                            tag: None,
                        },
                        ep_now,
                    )
                    .unwrap_err()
                };
                match err {
                    armd::ArmError::RejectedStatic { status: 409, code, .. } => {
                        assert_eq!(code, "J7_FEEDBACK_STALE", "{owner:?}: wrong reject code");
                    }
                    other => panic!("{owner:?}: expected 409 J7_FEEDBACK_STALE for {cmd:?}, got {other:?}"),
                }
            }
            // Jog changing J7 target is also refused with J7_FEEDBACK_STALE!
            let jog_err = {
                let inner = rig.inner_mut();
                let mut target = inner.commanded_positions.clone();
                target[6] = 0.0;
                let stamp = inner.current_admission_stamp();
                let admitted = inner.check_admission(stamp).expect("admit jog check");
                SupervisorInner::jog_action(admitted, target, Some(0.5), Some(ep_now), "joint_move")
                    .unwrap_err()
            };
            match jog_err {
                armd::ArmError::RejectedStatic { status: 409, code, .. } => {
                    assert_eq!(code, "J7_FEEDBACK_STALE", "{owner:?}: jog changing J7 must reject J7_FEEDBACK_STALE");
                }
                other => panic!("{owner:?}: expected 409 J7_FEEDBACK_STALE on jog changing J7, got {other:?}"),
            }
            // Servo attempting a new J7 target (0.0) holds J7 frozen while J1–J6 continue, and
            // wire emits ONLY the freeze frame (vel_raw == 0 at last_fresh_pos, NEVER 0.0)!
            {
                let inner = rig.inner_mut();
                let mut servo_t = inner.commanded_positions.clone();
                servo_t[0] += 0.05;
                servo_t[6] = 0.0;
                let stamp = inner.current_admission_stamp();
                let admitted = inner.check_admission(stamp).expect("admit servo");
                SupervisorInner::servo(admitted, servo_t, ep_now).expect("servo J1-J6 continues");
            }
            rig.bus.clear_log();
            rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            rig.step_dt(DT, false);
            assert_j7_freeze_frames(
                &j7_pf_frames(&rig.bus.tx_log()),
                last_fresh_pos,
                pf_cur,
                &format!("{owner:?} servo while J7 stale"),
            );
        }
    }
}

/// Round 4 Major regression (Legacy + RT Executor):
/// `CAN_TX_STALLED` counts per-motor lack of progress on stopped-hold (ticks where a motor's due
/// hold frame did not reach the transport — queued, failed hard, or expired), while idle /
/// nothing-due never counts:
/// 1. Hard-error streak with live RX (`tx_error = true`) -> latches `CAN_TX_STALLED`.
/// 2. All-expired streak (`TxBatchOutcome::Expired { dropped > 0, pending: 0 }`) -> latches `CAN_TX_STALLED`.
/// 3. Idle / disabled with nothing due (even with `tx_error = true` on bus) -> NO fault.
#[test]
fn dual_owner_stopped_hold_per_motor_lack_of_progress_hard_error_all_expired_and_idle() {
    for owner in [Owner::Legacy, Owner::Executor] {
        // --- (1) Hard-error streak with live RX -> CAN_TX_STALLED ---
        {
            let mut rig = DualRig::new(owner, base_cfg(), None);
            rig.inner_mut()
                .safety
                .trigger_fault_coded(FaultCode::Other, FaultReason::Other("hard-error streak".into()));
            rig.bus.with(|s| s.tx_error = true);
            let mut latched = false;
            for _ in 0..6 {
                rig.step();
                if rig.inner().safety.fault_code() == Some(FaultCode::CanTxStalled) {
                    latched = true;
                    break;
                }
            }
            assert!(
                latched,
                "{owner:?}: hard TX error streak with live RX must latch CAN_TX_STALLED"
            );
            let detail = rig.inner().safety.tx_stall_detail().expect("tx_stall_detail");
            assert!(detail.is_hold && detail.queued == 0 && detail.consecutive_ticks >= 2, "{detail:?}");
        }

        // --- (2) All-expired streak (`pending == 0`, every due frame dropped expired) -> CAN_TX_STALLED ---
        {
            let budget = TxBudget {
                max_frames: 14,
                max_duration: ms(10),
                frame_max_age: ms(45),
            };
            let mut rig = DualRig::new(owner, base_cfg(), Some(budget));
            rig.inner_mut()
                .safety
                .trigger_fault_coded(FaultCode::Other, FaultReason::Other("all-expired streak".into()));
            // `tx_stall = true` + `stall_step = 50 ms` (> 45 ms `frame_max_age`): attempt 1 stalls
            // and advances clock 50 ms so post-pass `drop_stale` drops EVERY hold frame
            // (`TxBatchOutcome::Expired { dropped: 7, pending: 0 }`) on every tick!
            rig.bus.with(|s| {
                s.tx_stall = true;
                s.stall_step = ms(50);
            });
            let mut latched = false;
            for _ in 0..5 {
                rig.bus.with(|s| s.push_all_feedback());
                rig.step_dt(DT, false);
                let pending_hold = rig
                    .inner()
                    .driver
                    .as_ref()
                    .unwrap()
                    .lock()
                    .unwrap()
                    .pending_hold_tx_len();
                assert_eq!(pending_hold, 0, "{owner:?}: all-expired pass must leave pending == 0");
                if rig.inner().safety.fault_code() == Some(FaultCode::CanTxStalled) {
                    latched = true;
                    break;
                }
            }
            assert!(
                latched,
                "{owner:?}: all-expired stopped-hold streak (pending == 0) must latch CAN_TX_STALLED"
            );
            let detail = rig.inner().safety.tx_stall_detail().expect("tx_stall_detail");
            assert!(detail.is_hold && detail.queued == 0 && detail.dropped > 0, "{detail:?}");
        }

        // --- (3) Idle / nothing due (un-torqued Idle AND disabled Fault) -> NEVER CAN_TX_STALLED ---
        {
            // (3a) Idle with torque off (`!is_torqued`): nothing due even if bus has tx_error = true.
            let mut rig_idle = DualRig::new(owner, base_cfg(), None);
            rig_idle.bus.with(|s| s.immediate_replies = true);
            {
                let mut drv = rig_idle.inner().driver.as_ref().unwrap().lock().unwrap();
                drv.set_torque(false).expect("disable torque");
            }
            rig_idle.inner_mut().is_torqued = false;
            rig_idle.inner_mut().torque_intent = armd::TorqueIntent::Off;
            rig_idle.inner_mut().safety.set_torque(false).expect("idle state");
            rig_idle.bus.with(|s| {
                s.immediate_replies = false;
                s.tx_error = true;
            });
            for _ in 0..10 {
                rig_idle.step();
                assert_eq!(
                    *rig_idle.inner().safety.state(),
                    SafetyState::Idle,
                    "{owner:?}: idle with nothing due must NEVER fault: {:?}",
                    rig_idle.inner().safety.fault()
                );
            }

            // (3b) Stopped Fault with torque disabled (`EnableState::Disabled`, 0 frames due):
            //      even across 10 ticks with `tx_error = true`, must NEVER escalate to CAN_TX_STALLED.
            rig_idle
                .inner_mut()
                .safety
                .trigger_fault_coded(FaultCode::Other, FaultReason::Other("disabled fault".into()));
            let ep = rig_idle.inner().safety.epoch_id();
            for _ in 0..10 {
                rig_idle.step();
                assert_eq!(rig_idle.inner().safety.epoch_id(), ep);
                assert_eq!(
                    rig_idle.inner().safety.fault_code(),
                    Some(FaultCode::Other),
                    "{owner:?}: disabled stopped state with nothing due must NEVER latch CAN_TX_STALLED"
                );
            }
        }
    }
}

/// Round 5 B1 regression:
/// A J7-only feedback outage trips the 45 ms MIT deadline fence before the 100 ms J7 freeze
/// threshold. Both driver-level fenced `write_estop_hold_bounded` AND supervisor-level STOP and
/// E-stop (Legacy + RT Executor) MUST still emit J7's `0x307` freeze/hold frame BEFORE any MIT
/// fence early-return, while suppressing ONLY the MIT/arm joints' output (ZERO MIT frames).
#[test]
fn dual_owner_tripped_mit_fence_j7_outage_stop_and_estop_still_send_freeze() {
    use armd::driver::{ArmDriver, ArmDriverError, DamiaoArmDriver, DamiaoControlMode, JointMode, JointTarget};
    use armd::mit::{MitOwner, MIT_DEADLINE};
    use armd::safety::StopPolicy;
    use armd::transport::CanFrame;

    let pf_cur = j7_pf_current_raw(&base_cfg());
    let is_mit_or_arm_cmd = |f: &CanFrame| {
        (1..=6).contains(&f.id) || (0x101..=0x106).contains(&f.id)
    };

    // --- (1) Driver-level fenced `write_estop_hold_bounded` after J7-only outage trips 45 ms fence ---
    {
        let clock = MockClock::new();
        let limits = dm_limits();
        let bus = FakeBus::with_clock(&limits, clock.clone());
        bus.with(|s| {
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = 0.1 * i as f32;
            }
            s.motors[6].q = -2.5;
            s.motors[3].set_reg_u32(armd::protocol::damiao::RID_TIMEOUT, 2000);
            s.immediate_replies = true;
            s.push_all_feedback();
        });
        let pf_cfg = armd::driver::GripperPosForce {
            joint: 6,
            current_raw: pf_cur,
            close_speed_rad_s: 0.30,
            closed_rad: 0.0,
            open_rad: -5.7,
            calibrated: false,
            ceiling_torque_nm: None,
            torque_reference: None,
        };
        let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits)
            .unwrap()
            .with_gripper_pos_force(pf_cfg)
            .unwrap();
        d.read_joint_states().unwrap();
        bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        d.check_mapping_ranges();
        d.set_torque(true).expect("enable");
        let hold: Vec<JointTarget> = (0..7)
            .map(|i| JointTarget::mit(if i == 6 { -2.5 } else { 0.1 * i as f32 }, 0.0, 12.0, 0.4, 0.0))
            .collect();
        let modes: Vec<JointMode> = (0..7)
            .map(|i| if i == 3 { JointMode::Mit } else { JointMode::Position })
            .collect();
        d.set_joint_modes_confirmed(&modes, &hold).expect("handover J4 to MIT");
        assert_eq!(d.joint_control_modes()[3], Some(DamiaoControlMode::Mit));
        assert_eq!(d.joint_control_modes()[6], Some(DamiaoControlMode::PosForce));
        d.set_mit_deadline_fence(Some(MIT_DEADLINE), ms(300));
        d.read_joint_states().unwrap();
        bus.with(|s| {
            s.immediate_replies = false;
            s.rx.clear();
            s.pending.clear();
            s.push_all_feedback();
        });
        d.read_joint_states().unwrap();

        // J7-only outage -> advance 50 ms (>= 45 ms MIT fence, <= 100 ms J7 fresh) -> fence trips!
        bus.with(|s| {
            s.rx.clear();
            s.pending.clear();
            s.motors[6].mute_feedback = true;
        });
        clock.advance(ms(50));
        bus.with(|s| for i in 0..6 { s.push_feedback(i); });
        let _ = d.read_joint_states();
        assert!(
            d.mit_fence_trip().is_some(),
            "50 ms J7 outage must trip 45 ms MIT deadline fence"
        );

        // At 50 ms (J7 still <= 100 ms fresh, MIT fence tripped): fenced `write_estop_hold_bounded`
        // suppresses MIT/arm joints (returns `Err(MitFenceTripped)`) AND emits J7's 0x307 stop hold!
        bus.clear_log();
        d.begin_tx_tick();
        let err_fresh = d.write_estop_hold_bounded(&hold).unwrap_err();
        assert!(matches!(err_fresh, ArmDriverError::MitFenceTripped(_)), "{err_fresh:?}");
        let log_50 = bus.tx_log();
        assert!(
            !log_50.iter().any(is_mit_or_arm_cmd),
            "tripped MIT fence must suppress all MIT/arm frames at 50 ms: {log_50:?}"
        );
        let j7_50 = j7_pf_frames(&log_50);
        assert_eq!(j7_50.len(), 1, "expected 1 J7 0x307 hold frame at 50 ms: {log_50:?}");
        let p50 = f32::from_le_bytes(j7_50[0].data[0..4].try_into().unwrap());
        assert!((p50 - (-2.5)).abs() < 1e-3, "expected J7 hold at -2.5, got {p50}");

        // Advance to 120 ms (> 100 ms, J7 now stale): fenced `write_estop_hold_bounded` suppresses
        // MIT/arm joints AND emits J7's exact 0x307 FREEZE frame (`vel_raw == 0`, pos -2.5)!
        clock.advance(ms(70));
        bus.with(|s| for i in 0..6 { s.push_feedback(i); });
        let _ = d.read_joint_states();
        bus.clear_log();
        d.begin_tx_tick();
        let err_stale = d.write_estop_hold_bounded(&hold).unwrap_err();
        assert!(matches!(err_stale, ArmDriverError::MitFenceTripped(_)), "{err_stale:?}");
        let log_120 = bus.tx_log();
        assert!(
            !log_120.iter().any(is_mit_or_arm_cmd),
            "tripped MIT fence must suppress all MIT/arm frames at 120 ms: {log_120:?}"
        );
        assert_j7_freeze_frames(&j7_pf_frames(&log_120), -2.5, pf_cur, "fenced driver 120 ms");
    }

    // --- (2) Supervisor dual-owner (Legacy + RT Executor): J7 outage trips 45 ms MIT fence,
    //         then STOP and E-stop both still send J7 freeze frames with ZERO MIT frames ---
    let mut jt = armd::tracking::JointTrackingConfig::rebot_dm();
    jt.enabled = true;
    for j in &mut jt.joints {
        j.strategy = armd::tracking::TrackingStrategy::Position;
    }
    jt.joints[3].strategy = armd::tracking::TrackingStrategy::Mit;
    jt.joints[3].kp = 10.0;
    jt.joints[3].kd = 0.3;
    jt.joints[3].hold_kp = Some(12.0);
    jt.joints[3].hold_kd = Some(0.4);
    jt.joints[3].effort_limit_nm = 3.0;
    jt.joints[3].gravity_scale = 1.0;
    for owner in [Owner::Legacy, Owner::Executor] {
        for is_estop in [false, true] {
            let mut rig = DualRig::new_pos_force(owner, base_cfg(), None);
            rig.inner_mut().config.joint_tracking = Some(jt.clone());
            if let RigOwner::Executor { fx, .. } = &mut rig.owner {
                let stops = fx.stops().clone();
                fx.core.attach_stop_requests(stops);
                let gate = fx.core.estop_gate();
                fx.core.driver.as_ref().unwrap().lock().unwrap().set_estop_gate(gate);
            }
            // Start J7 open at -3.0 rad, hand J4 over to MIT with 45 ms fence armed.
            rig.bus.with(|s| {
                s.motors[6].q = -3.0;
                s.motors[3].set_reg_u32(armd::protocol::damiao::RID_TIMEOUT, 2000);
                s.immediate_replies = true;
                s.push_all_feedback();
            });
            rig.step();
            rig.inner_mut().commanded_positions[6] = -3.0;
            rig.inner_mut().target_positions[6] = -3.0;
            {
                let hold: Vec<JointTarget> = (0..7)
                    .map(|i| JointTarget::mit(rig.inner().measured_positions[i], 0.0, 12.0, 0.4, 0.0))
                    .collect();
                let modes: Vec<JointMode> = (0..7)
                    .map(|i| if i == 3 { JointMode::Mit } else { JointMode::Position })
                    .collect();
                let mut drv = rig.inner().driver.as_ref().unwrap().lock().unwrap();
                drv.set_joint_modes_confirmed(&modes, &hold).expect("handover J4 to MIT");
                drv.set_mit_deadline_fence(Some(MIT_DEADLINE), ms(300));
            }
            rig.inner_mut().mit.owner = MitOwner::Active;
            rig.inner_mut().mit.joints = vec![3];
            rig.inner_mut().mit.last_output = Some(armd::transport::DriverClock::now(&rig.clock));
            rig.bus.with(|s| {
                s.immediate_replies = false;
                s.rx.clear();
                s.pending.clear();
                s.push_all_feedback();
            });
            rig.step();

            // Begin closing J7 so J7 has an active closing target when the outage hits.
            let ep = rig.inner().safety.epoch_id();
            {
                let inner = rig.inner_mut();
                let stamp = inner.current_admission_stamp();
                let admitted = inner.check_admission(stamp).expect("admit gripper close");
                SupervisorInner::start_gripper(
                    admitted,
                    GripperRequest {
                        command: GripperCommand::Close,
                        fraction: None,
                        grasp: false,
                        speed: 0.30,
                        timeout_s: Some(15.0),
                        tag: None,
                    },
                    ep,
                )
                .expect("start gripper close");
            }
            let last_fresh_pos = rig.inner().measured_positions[6];
            assert!(last_fresh_pos < -2.0);

            // J7-only outage -> advance 55 ms -> MIT fence trips BEFORE 100 ms!
            rig.bus.with(|s| {
                s.rx.clear();
                s.pending.clear();
                s.motors[6].mute_feedback = true;
            });
            rig.clock.advance(ms(55));
            rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            rig.step_dt(DT, false);
            assert!(
                rig.inner().driver.as_ref().unwrap().lock().unwrap().mit_fence_trip().is_some(),
                "{owner:?} (estop={is_estop}): 55 ms J7 outage must trip 45 ms MIT fence"
            );
            assert_eq!(
                rig.inner().mit.owner,
                MitOwner::Lost,
                "{owner:?} (estop={is_estop}): MIT tracking must transition to Lost"
            );

            // Advance past 100 ms (J7 now stale), then issue STOP or E-stop!
            rig.clock.advance(ms(60));
            rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            match (&mut rig.owner, is_estop) {
                (RigOwner::Legacy(inner), false) => {
                    assert!(inner.authorized_stop());
                }
                (RigOwner::Executor { fx, .. }, false) => {
                    fx.stops().request();
                }
                (RigOwner::Legacy(inner), true) => {
                    inner.apply_stop_policy(StopPolicy::EStopLatch);
                }
                (RigOwner::Executor { fx, .. }, true) => {
                    fx.core.e_stop_latched.store(true, Ordering::SeqCst);
                    fx.core.requested_estop_count.fetch_add(1, Ordering::SeqCst);
                }
            }
            rig.bus.clear_log();
            rig.step_dt(DT, false);

            let log = rig.bus.tx_log();
            assert!(
                !log.iter().any(is_mit_or_arm_cmd),
                "{owner:?} (estop={is_estop}): tripped MIT fence must suppress all MIT/arm frames: {log:?}"
            );
            assert_j7_freeze_frames(
                &j7_pf_frames(&log),
                last_fresh_pos,
                pf_cur,
                &format!("{owner:?} tripped MIT fence + {}", if is_estop { "E-stop" } else { "STOP" }),
            );
        }
    }
}

/// Round 5 M2 regression (Legacy + RT Executor, both standard POS_VEL and PosForce rigs):
/// Stale J1 (`BuildRefused` on every tick) + J2 `WouldBlock` (`backpressure_tx_ids = [0x102]`)
/// streak MUST NOT allow J1's refusal to clear J2's per-motor stopped-hold progress accounting:
/// J1 stays at 0 ticks (refused, not TX-stalled), while J2's streak advances past 45 ms and
/// latches `CAN_TX_STALLED`!
#[test]
fn dual_owner_stale_j1_refusal_never_hides_j2_wouldblock_can_tx_stalled() {
    for owner in [Owner::Legacy, Owner::Executor] {
        for use_pos_force in [false, true] {
            let mut rig = if use_pos_force {
                DualRig::new_pos_force(owner, base_cfg(), None)
            } else {
                DualRig::new(owner, base_cfg(), None)
            };
            rig.inner_mut().safety.trigger_fault_coded(
                FaultCode::Other,
                FaultReason::Other("M2 stale J1 + J2 WouldBlock".into()),
            );
            rig.step();

            // Make ONLY J1 stale (> 100 ms, < 500 ms BUS_LOSS) while J2–J7 stay fresh across
            // normal ticks, and inject persistent WouldBlock on J2 (0x102)!
            rig.bus.with(|s| {
                s.rx.clear();
                s.pending.clear();
                s.motors[0].mute_feedback = true;
            });
            // Run 6 healthy ticks for J2–J7 (120 ms elapsed on J1 -> J1 hits BuildRefused).
            for _ in 0..6 {
                rig.bus.with(|s| for i in 1..7 { s.push_feedback(i); });
                rig.step_dt(DT, false);
            }
            assert_eq!(
                rig.inner().safety.fault_code(),
                Some(FaultCode::Other),
                "{owner:?} (pf={use_pos_force}): stale J1 alone without TX backpressure must not latch CAN_TX_STALLED"
            );

            // Now inject WouldBlock on J2 (0x102) while J1 remains stale (`BuildRefused` every tick).
            rig.bus.with(|s| s.backpressure_tx_ids = vec![0x102]);
            let mut latched = false;
            for _ in 0..6 {
                rig.bus.with(|s| for i in 1..7 { s.push_feedback(i); });
                rig.step_dt(DT, false);
                assert_eq!(
                    rig.inner().hold_tx_no_progress_ticks()[0],
                    0,
                    "{owner:?} (pf={use_pos_force}): refused J1 slot 0 must not accumulate TX stall ticks"
                );
                if rig.inner().safety.fault_code() == Some(FaultCode::CanTxStalled) {
                    latched = true;
                    break;
                }
            }
            assert!(
                latched,
                "{owner:?} (pf={use_pos_force}): stale J1 BuildRefused must NEVER hide J2 WouldBlock -> CAN_TX_STALLED"
            );
            assert!(
                rig.inner().hold_tx_no_progress_ticks()[2] >= 3,
                "{owner:?} (pf={use_pos_force}): expected J2 slot 0 (index 2) to accumulate stall streak, got {:?}",
                rig.inner().hold_tx_no_progress_ticks()
            );
            let detail = rig.inner().safety.tx_stall_detail().expect("tx_stall_detail");
            assert!(detail.is_hold && detail.age_ms >= 45, "{detail:?}");
        }
    }
}

/// Round 5 M3 regression (Legacy + RT Executor):
/// 1. A running `gripper=true` replay terminates immediately with `J7_FEEDBACK_STALE` when J7
///    goes stale (> 100 ms, before 250 ms fault) instead of silently freezing, emitting ONLY the
///    `0x307` freeze frame (`vel_raw == 0` at `last_fresh_pos`).
/// 2. While J7 is stale (> 100 ms), ALL J7 target producers route through the single admission
///    check `check_pos_force_gripper_fresh` and refuse with `409 J7_FEEDBACK_STALE`:
///    - standalone replay (`gripper=true`),
///    - sequence/macro replay (`gripper=true`),
///    - macro gripper (`start_macro_gripper`) and macro quintic moving J7 (`install_macro_quintic`),
///    - policy live start (`begin_policy_run(Live)`) and live policy servo frames (`policy_step`).
/// 3. No J7 target submitted while stale (including `servo` clamping J7 to freeze_pos while J1–J6
///    continue) ever reaches the wire after J7 feedback recovers!
#[test]
fn dual_owner_stale_j7_gates_replay_sequence_policy_macros_and_never_sends_stale_target_after_recovery() {
    use armd::recording::{EpisodeStore, RecordFrame};
    use armd::replay::{plan_frames, ReplayInput, ReplayRequest};

    let pf_cur = j7_pf_current_raw(&base_cfg());
    let dir = tempfile::tempdir().unwrap();
    let store = EpisodeStore::new(Some(dir.path().to_path_buf()));

    let make_replay_plan = |inner: &SupervisorInner, gripper: bool, target_j7: f32| {
        let q0 = inner.commanded_positions.clone();
        let mut q1 = q0.clone();
        q1[6] = target_j7;
        let frames = vec![
            RecordFrame { timestamp: 0.0, observation_state: q0.clone(), action: q0.clone() },
            RecordFrame { timestamp: 0.2, observation_state: q0.clone(), action: q0.clone() },
            RecordFrame { timestamp: 3.0, observation_state: q1.clone(), action: q1 },
        ];
        let input = ReplayInput {
            request: ReplayRequest {
                episode: "m3_ep".into(),
                speed: 1.0,
                gripper,
                tag: None,
                allow_stale: false,
            },
            path: dir.path().join("m3_ep.parquet"),
            profile_id: inner.config.profile_id.clone(),
            dof: inner.config.dof,
            arm_dof: inner.config.arm_dof(),
            gripper_idx: inner.config.gripper_index(),
            joint_names: inner.config.limits.iter().map(|l| l.name.clone()).collect(),
            limits: inner.config.limits.clone(),
            speed_limits: inner.config.limits.clone(),
            ws: None,
            arm: inner.config.arm_model.clone(),
            start: q0,
            control_hz: inner.config.control_frequency_hz,
            epoch: inner.safety.epoch_id(),
            workspace_revision: inner.workspace_revision,
            power: None,
            store: store.clone(),
            zero_offsets: inner.joint_zero_offsets.clone(),
            tcp_offset: inner.config.tcp_offset,
            active_tool: inner.config.active_tool.clone(),
        };
        plan_frames(input, &frames).expect("plan replay frames")
    };

    let assert_j7_stale_reject = |res: Result<(), armd::ArmError>, label: &str| {
        match res.expect_err(&format!("{label}: expected 409 J7_FEEDBACK_STALE")) {
            armd::ArmError::RejectedStatic { status: 409, code, .. } => {
                assert_eq!(code, "J7_FEEDBACK_STALE", "{label}: wrong reject code");
            }
            other => panic!("{label}: expected 409 J7_FEEDBACK_STALE, got {other:?}"),
        }
    };

    for owner in [Owner::Legacy, Owner::Executor] {
        let mut rig = DualRig::new_pos_force(owner, base_cfg(), None);
        // Place J7 open at -3.0 rad with fresh feedback.
        rig.bus.with(|s| {
            s.motors[6].q = -3.0;
            s.push_all_feedback();
        });
        rig.step();
        rig.inner_mut().commanded_positions[6] = -3.0;
        rig.inner_mut().target_positions[6] = -3.0;

        // 1. Start a `gripper=true` replay while J7 is fresh, run several ticks, then mute J7
        //    past 100 ms (< 250 ms fault): running replay MUST terminate immediately with
        //    `J7_FEEDBACK_STALE` (not silently freeze), emitting ONLY the 0x307 freeze frame!
        let plan_running = make_replay_plan(rig.inner(), true, 0.0);
        let (replay_id, _) = {
            let inner = rig.inner_mut();
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).expect("admit replay");
            SupervisorInner::install_replay(admitted, plan_running).expect("install fresh replay")
        };
        for _ in 0..4 {
            rig.step();
            assert!(rig.inner().safety.state().is_operational());
        }
        assert!(
            rig.inner().last_action.as_ref().is_some_and(|a| a.id == replay_id && a.is_running()),
            "{owner:?}: replay should be running before staleness"
        );
        let last_fresh_pos = rig.inner().measured_positions[6];
        assert!(last_fresh_pos < -2.0, "{owner:?}: expected open J7 < -2.0, got {last_fresh_pos}");

        rig.bus.with(|s| {
            s.rx.clear();
            s.pending.clear();
            s.motors[6].mute_feedback = true;
        });
        rig.clock.advance(ms(115));
        rig.bus.clear_log();
        rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
        rig.step_dt(DT, false);

        assert!(
            rig.inner().safety.state().is_operational(),
            "{owner:?}: 115 ms staleness must not prematurely latch 250 ms fault"
        );
        let ended = rig.inner().last_action.as_ref().expect("finished replay action");
        assert_eq!(ended.id, replay_id);
        assert_eq!(ended.state, "failed", "{owner:?}: running gripper=true replay must fail on stale J7");
        assert_eq!(ended.code.as_deref(), Some("J7_FEEDBACK_STALE"));
        assert!(
            rig.inner().active_trajectory.is_none(),
            "{owner:?}: failed replay must clear active_trajectory"
        );
        assert_j7_freeze_frames(
            &j7_pf_frames(&rig.bus.tx_log()),
            last_fresh_pos,
            pf_cur,
            &format!("{owner:?} running gripper=true replay stale termination"),
        );

        // 2. While J7 is stale (> 100 ms, Operational), ALL J7 target producers are refused:
        // (2a) Standalone `install_replay` with `gripper=true` -> 409 J7_FEEDBACK_STALE.
        let plan_standalone = make_replay_plan(rig.inner(), true, 0.0);
        let res_standalone = {
            let inner = rig.inner_mut();
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).expect("admit standalone replay");
            SupervisorInner::install_replay(admitted, plan_standalone).map(drop)
        };
        assert_j7_stale_reject(res_standalone, &format!("{owner:?} standalone replay gripper=true"));

        // (2b) Sequence/macro replay (`install_macro_replay`), macro gripper (`start_macro_gripper`),
        //      and macro quintic moving J7 (`install_macro_quintic`) -> ALL 409 J7_FEEDBACK_STALE!
        let ep_now = rig.inner().safety.epoch_id();
        let seq_id = {
            let inner = rig.inner_mut();
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).expect("admit sequence macro");
            SupervisorInner::begin_sequence_action(admitted, Some("m3-seq".into()), ep_now)
                .expect("begin sequence macro")
        };
        let plan_macro = make_replay_plan(rig.inner(), true, 0.0);
        let res_macro_replay = {
            let inner = rig.inner_mut();
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).expect("admit macro replay");
            SupervisorInner::install_macro_replay(admitted, seq_id, plan_macro).map(drop)
        };
        assert_j7_stale_reject(res_macro_replay, &format!("{owner:?} sequence macro replay gripper=true"));

        let res_macro_grip = {
            let inner = rig.inner_mut();
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).expect("admit macro gripper");
            SupervisorInner::start_macro_gripper(
                admitted,
                seq_id,
                GripperRequest {
                    command: GripperCommand::Close,
                    fraction: None,
                    grasp: false,
                    speed: 0.30,
                    timeout_s: Some(10.0),
                    tag: None,
                },
            )
            .map(drop)
        };
        assert_j7_stale_reject(res_macro_grip, &format!("{owner:?} macro gripper"));

        let res_macro_quintic = {
            let inner = rig.inner_mut();
            let mut q_close = inner.commanded_positions.clone();
            q_close[6] = 0.0;
            let limits = inner.config.limits.clone();
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).expect("admit macro quintic");
            SupervisorInner::install_macro_quintic(admitted, seq_id, q_close, &limits).map(drop)
        };
        assert_j7_stale_reject(res_macro_quintic, &format!("{owner:?} macro quintic moving J7"));

        // Finish the sequence macro cleanly before testing policy + servo.
        rig.inner_mut().finish_macro(seq_id, "cancelled", Some("STOPPED"), None, None, None);

        // (2c) Policy live start (`begin_policy_run(Live)`) while J7 is stale -> 409 J7_FEEDBACK_STALE!
        #[cfg(feature = "policy-runtime")]
        {
            use armd::policy_runtime::{PolicyMode, PolicyRunStart};
            rig.inner_mut().follower = Some("sim".into());
            rig.inner_mut().lease_enabled = true;
            let lease_gen = rig.inner_mut().acquire_lease("m3-policy").expect("lease");
            let ep_pol = rig.inner().safety.epoch_id();
            let res_begin_pol = {
                let inner = rig.inner_mut();
                let stamp = inner.current_admission_stamp();
                let admitted = inner.check_admission(stamp).expect("admit policy start");
                SupervisorInner::begin_policy_run(
                    admitted,
                    PolicyRunStart {
                        epoch_id: ep_pol,
                        connection_id: "m3-policy".into(),
                        lease_generation: lease_gen,
                        policy_id: "m3-pol".into(),
                        artifact_digest: "0".repeat(64),
                        backend: "mlp".into(),
                        mode: PolicyMode::Live,
                        sim_only: false,
                        max_first_step_rad: None,
                        max_step_rad: None,
                        tag: None,
                        i_am_at_the_arm: false,
                        verified_release_gate: None,
                    },
                )
                .map(drop)
            };
            assert_j7_stale_reject(res_begin_pol, &format!("{owner:?} begin_policy_run(Live) while stale"));
            rig.inner_mut().lease_enabled = false;
            rig.inner_mut().follower = None;
        }

        // (2d) Servo attempting a new J7 target (0.0) while stale holds J7 frozen at last_fresh_pos.
        {
            let inner = rig.inner_mut();
            let mut servo_t = inner.commanded_positions.clone();
            servo_t[0] += 0.05;
            servo_t[6] = 0.0;
            let ep_s = inner.safety.epoch_id();
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).expect("admit servo");
            SupervisorInner::servo(admitted, servo_t, ep_s).expect("servo J1-J6 continues");
        }
        rig.bus.clear_log();
        rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
        rig.step_dt(DT, false);
        assert_j7_freeze_frames(
            &j7_pf_frames(&rig.bus.tx_log()),
            last_fresh_pos,
            pf_cur,
            &format!("{owner:?} servo while J7 stale"),
        );

        // 3. After J7 recovers to fresh feedback, ZERO J7 targets submitted while stale (0.0)
        //    may reach the wire!
        rig.bus.with(|s| {
            s.motors[6].mute_feedback = false;
            s.motors[6].q = last_fresh_pos;
            s.push_all_feedback();
        });
        rig.bus.clear_log();
        for tick in 0..5 {
            rig.step();
            assert!(
                rig.inner().safety.state().is_operational(),
                "{owner:?}: recovery tick {tick} must stay operational: {:?}",
                rig.inner().safety.fault()
            );
        }
        let recovered_j7 = j7_pf_frames(&rig.bus.tx_log());
        assert!(!recovered_j7.is_empty(), "{owner:?}: expected J7 0x307 frames after recovery");
        for f in &recovered_j7 {
            let p = f32::from_le_bytes(f.data[0..4].try_into().unwrap());
            assert!(
                (p - last_fresh_pos).abs() < 1e-2,
                "{owner:?}: stale-admitted J7 target leaked after recovery: wire pos {p} != {last_fresh_pos}"
            );
        }

        // 4. Policy servo frame (`policy_step`) while a live policy run is ALREADY active and J7
        //    goes stale -> MUST reject with `409 J7_FEEDBACK_STALE`, terminate the run, and NEVER
        //    send the stale policy J7 target after J7 recovers!
        #[cfg(feature = "policy-runtime")]
        {
            use armd::policy_runtime::{PolicyMode, PolicyRunStart};
            rig.inner_mut().stop_motion();
            rig.inner_mut().commanded_velocities.fill(0.0);
            rig.inner_mut().safety.mark_motion_finished();
            rig.step();
            rig.inner_mut().follower = Some("sim".into());
            rig.inner_mut().lease_enabled = true;
            let lease_gen = rig.inner_mut().acquire_lease("m3-policy").expect("re-acquire lease");
            let ep_pol = rig.inner().safety.epoch_id();
            let pol_id = {
                let inner = rig.inner_mut();
                let stamp = inner.current_admission_stamp();
                let admitted = inner.check_admission(stamp).expect("admit fresh policy");
                SupervisorInner::begin_policy_run(
                    admitted,
                    PolicyRunStart {
                        epoch_id: ep_pol,
                        connection_id: "m3-policy".into(),
                        lease_generation: lease_gen,
                        policy_id: "m3-pol-step".into(),
                        artifact_digest: "0".repeat(64),
                        backend: "mlp".into(),
                        mode: PolicyMode::Live,
                        sim_only: false,
                        max_first_step_rad: None,
                        max_step_rad: None,
                        tag: None,
                        i_am_at_the_arm: false,
                        verified_release_gate: None,
                    },
                )
                .expect("start fresh policy run")
            };
            // J7 goes stale (> 100 ms), then policy sends a frame targeting J7 = last_fresh_pos + 0.05!
            rig.bus.with(|s| {
                s.rx.clear();
                s.pending.clear();
                s.motors[6].mute_feedback = true;
            });
            rig.clock.advance(ms(115));
            rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            rig.step_dt(DT, false);

            let mut pol_target = rig.inner().measured_positions.clone();
            pol_target[6] = (last_fresh_pos + 0.05).min(0.0);
            let res_step = rig.inner_mut().policy_step(pol_id, pol_target, Some(0));
            assert_j7_stale_reject(res_step, &format!("{owner:?} live policy_step while stale"));
            assert!(!rig.inner().policy_run_active(), "{owner:?}: policy run must terminate on J7_FEEDBACK_STALE");
            assert_eq!(
                rig.inner().last_action.as_ref().and_then(|a| a.code.as_deref()),
                Some("J7_FEEDBACK_STALE")
            );

            // Recover J7 and verify the rejected policy J7 target NEVER reaches the wire!
            rig.bus.with(|s| {
                s.motors[6].mute_feedback = false;
                s.motors[6].q = last_fresh_pos;
                s.push_all_feedback();
            });
            rig.bus.clear_log();
            for _ in 0..4 {
                rig.step();
            }
            for f in j7_pf_frames(&rig.bus.tx_log()) {
                let p = f32::from_le_bytes(f.data[0..4].try_into().unwrap());
                assert!(
                    (p - last_fresh_pos).abs() < 1e-2,
                    "{owner:?}: rejected policy J7 target leaked after recovery: wire pos {p} != {last_fresh_pos}"
                );
            }
        }
    }
}

/// R1 architectural restructure regression: ONE single J7 `PosForce` (`0x307`) emitter called
/// once per control tick from the tick epilogue on BOTH owners (Legacy + RT Executor), plus
/// direct-driver verification across EVERY review case:
/// 1. Fenced motion write with J7 precheck stale (> 100 ms) -> 0 MIT frames, exact 1 `0x307` FREEZE frame;
/// 2. Fenced motion write with MIT fence already tripped (fresh AND stale J7) -> 0 MIT frames,
///    exact 1 `0x307` Hold (`vel_raw == 0`) or FREEZE frame;
/// 3. Fenced motion write with MIT fence tripping MID-BATCH -> MIT aborts, exact 1 `0x307` Hold (`vel_raw == 0`) frame;
/// 4. Invalid caller J7 target (`NaN` / sub-quantum `0.005 rad/s`) in stopped hold (direct driver
///    unfenced & fenced + dual-owner) AND operational motion (dual-owner) -> NEVER suppresses J7;
///    exact 1 `0x307` Hold/Stop (`vel_raw == 0`) or FREEZE frame;
/// 5. `send_to_bus == false` (> 500 ms `BUS_LOSS` while torqued) -> 0 J1-J6 frames, exact 1 `0x307` FREEZE frame.
#[test]
fn dual_owner_single_j7_emitter_covers_all_fenced_and_invalid_target_paths() {
    use armd::driver::{ArmDriver, ArmDriverError, DamiaoArmDriver, DamiaoControlMode, JointMode, JointTarget};
    use armd::mit::{MitOwner, MIT_DEADLINE};
    use armd::transport::CanFrame;

    let pf_cur = j7_pf_current_raw(&base_cfg());
    let is_mit_or_arm_cmd = |f: &CanFrame| (1..=6).contains(&f.id) || (0x101..=0x106).contains(&f.id);

    let assert_exact_one_j7 = |log: &[CanFrame], expected_pos: f32, expected_vel: u16, expected_cur: u16, label: &str| {
        let j7 = j7_pf_frames(log);
        assert_eq!(j7.len(), 1, "{label}: expected exact 1 J7 0x307 frame per tick, got {j7:?}");
        let pos = f32::from_le_bytes(j7[0].data[0..4].try_into().unwrap());
        let vel = u16::from_le_bytes([j7[0].data[4], j7[0].data[5]]);
        let cur = u16::from_le_bytes([j7[0].data[6], j7[0].data[7]]);
        assert!((pos - expected_pos).abs() < 1e-2, "{label}: J7 pos {pos} != expected {expected_pos}");
        assert_eq!(vel, expected_vel, "{label}: J7 vel_raw {vel} != expected {expected_vel}");
        assert_eq!(cur, expected_cur, "{label}: J7 cur {cur} != expected {expected_cur}");
    };

    let make_fenced_driver = |j7_start: f32| {
        let clock = MockClock::new();
        let limits = dm_limits();
        let bus = FakeBus::with_clock(&limits, clock.clone());
        bus.with(|s| {
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = 0.1 * i as f32;
            }
            s.motors[6].q = j7_start;
            s.motors[3].set_reg_u32(armd::protocol::damiao::RID_TIMEOUT, 2000);
            s.immediate_replies = true;
            s.push_all_feedback();
        });
        let pf_cfg = armd::driver::GripperPosForce {
            joint: 6,
            current_raw: pf_cur,
            close_speed_rad_s: 0.30,
            closed_rad: 0.0,
            open_rad: -5.7,
            calibrated: false,
            ceiling_torque_nm: None,
            torque_reference: None,
        };
        let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits)
            .unwrap()
            .with_gripper_pos_force(pf_cfg)
            .unwrap();
        d.read_joint_states().unwrap();
        bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        d.check_mapping_ranges();
        d.set_torque(true).expect("enable");
        let hold: Vec<JointTarget> = (0..7)
            .map(|i| JointTarget::mit(if i == 6 { j7_start } else { 0.1 * i as f32 }, 0.0, 12.0, 0.4, 0.0))
            .collect();
        let modes: Vec<JointMode> = (0..7)
            .map(|i| if i == 3 { JointMode::Mit } else { JointMode::Position })
            .collect();
        d.set_joint_modes_confirmed(&modes, &hold).expect("handover J4 to MIT");
        assert_eq!(d.joint_control_modes()[3], Some(DamiaoControlMode::Mit));
        assert_eq!(d.joint_control_modes()[6], Some(DamiaoControlMode::PosForce));
        d.set_mit_deadline_fence(Some(MIT_DEADLINE), ms(300));
        d.read_joint_states().unwrap();
        bus.with(|s| {
            s.immediate_replies = false;
            s.rx.clear();
            s.pending.clear();
            s.push_all_feedback();
        });
        d.read_joint_states().unwrap();
        (d, bus, clock, hold)
    };

    // === Part A: Direct-driver verification for all 4 review cases ===
    // A1. Fenced motion write: J7 precheck stale (> 100 ms) while J1-J6 fresh -> 0 MIT frames + 1 FREEZE frame.
    {
        let (mut d, bus, clock, mut tgt) = make_fenced_driver(-2.5);
        tgt[6] = JointTarget::position(-0.5);
        bus.with(|s| {
            s.rx.clear();
            s.pending.clear();
            s.motors[6].mute_feedback = true;
        });
        // Keep J1-J6 fresh while aging J7 to 115 ms WITHOUT calling read_joint_states (so fence
        // is NOT yet tripped before write_joint_targets_bounded — hits the J7 precheck stale path!).
        clock.advance(ms(115));
        bus.clear_log();
        d.begin_tx_tick();
        let err = d.write_joint_targets_bounded(&tgt).unwrap_err();
        assert!(matches!(err, ArmDriverError::PosForceFeedbackStale { joint: 6 }), "expected PosForceFeedbackStale, got {err:?}");
        let log = bus.tx_log();
        assert!(!log.iter().any(is_mit_or_arm_cmd), "A1 must send 0 MIT/arm frames: {log:?}");
        assert_exact_one_j7(&log, -2.5, 0, pf_cur, "A1 direct fenced motion precheck stale");
    }

    // A2. Fenced motion write: fence ALREADY tripped before batch (J7 still fresh!) -> 0 MIT frames + 1 Hold frame.
    {
        let (mut d, bus, clock, mut tgt) = make_fenced_driver(-2.5);
        tgt[6] = JointTarget::position(-0.5);
        // Trip fence via J4-only 50 ms outage while J7 stays fresh!
        bus.with(|s| {
            s.rx.clear();
            s.pending.clear();
            s.motors[3].mute_feedback = true;
        });
        clock.advance(ms(50));
        bus.with(|s| {
            for i in [0, 1, 2, 4, 5, 6] {
                s.push_feedback(i);
            }
        });
        let _ = d.read_joint_states();
        assert!(d.mit_fence_trip().is_some(), "J4 outage must trip fence while J7 stays fresh");
        assert!(d.pos_force_feedback_fresh(), "J7 must still be fresh");
        bus.clear_log();
        d.begin_tx_tick();
        let err = d.write_joint_targets_bounded(&tgt).unwrap_err();
        assert!(matches!(err, ArmDriverError::MitFenceTripped(_)), "expected MitFenceTripped, got {err:?}");
        let log = bus.tx_log();
        assert!(!log.iter().any(is_mit_or_arm_cmd), "A2 must send 0 MIT/arm frames: {log:?}");
        assert_exact_one_j7(&log, -2.5, 0, pf_cur, "A2 direct fenced motion fence already tripped (fresh J7)");
    }

    // A3. Fenced motion write: fence trips MID-BATCH during J2 transmit -> MIT aborts + 1 Hold frame.
    {
        let (mut d, bus, clock, mut tgt) = make_fenced_driver(-2.5);
        tgt[6] = JointTarget::position(-0.5);
        // Age mit_fence_last_cmd to 44 ms (< 45 ms MIT deadline so pre-batch fence check passes),
        // while keeping feedback fresh and tick elapsed duration (2 ms) well inside max_duration.
        clock.advance(ms(44));
        bus.with(|s| s.push_all_feedback());
        let _ = d.read_joint_states();
        let clock_mid = clock.clone();
        let tx_count = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let tx_count_cb = tx_count.clone();
        bus.with(|s| {
            s.on_tx = Some(Arc::new(move || {
                // On frame 2 (J2), advance clock by 2 ms (44 + 2 = 46 ms > 45 ms MIT fence) so
                // the fence trips mid-batch before J4 without exhausting tx_budget.max_duration!
                if tx_count_cb.fetch_add(1, Ordering::SeqCst) == 1 {
                    clock_mid.advance(ms(2));
                }
            }));
        });
        bus.clear_log();
        d.begin_tx_tick();
        let err = d.write_joint_targets_bounded(&tgt).unwrap_err();
        assert!(matches!(err, ArmDriverError::MitFenceTripped(_)), "expected mid-batch MitFenceTripped, got {err:?}");
        let log = bus.tx_log();
        assert!(log.iter().any(|f| f.id == 0x101), "A3 should have started batch (J1 sent) before mid-batch trip: {log:?}");
        assert!(!log.iter().any(|f| f.id == 4), "A3 must abort before J4 MIT frame: {log:?}");
        assert_exact_one_j7(&log, -2.5, 0, pf_cur, "A3 direct fenced motion mid-batch fence trip");
    }

    // A4. Stopped-hold writer (`write_estop_hold_bounded`, BOTH unfenced and fenced) with invalid
    //     caller J7 targets (NaN position, Inf velocity, sub-quantum 0.005 velocity): NEVER suppresses J7!
    for fenced in [false, true] {
        for (label, bad_pos, bad_vel) in [
            ("NaN pos", f32::NAN, Some(0.30)),
            ("Inf vel", -0.5, Some(f32::INFINITY)),
            ("sub-quantum vel 0.005", -0.5, Some(0.005)),
        ] {
            let (mut d, bus, _clock, mut hold) = make_fenced_driver(-2.5);
            if !fenced {
                d.set_mit_deadline_fence(None, Duration::ZERO);
            }
            hold[6].position = bad_pos;
            hold[6].velocity = bad_vel;
            bus.clear_log();
            d.begin_tx_tick();
            let res = d.write_estop_hold_bounded(&hold);
            assert!(res.is_ok(), "A4 fenced={fenced} {label}: stopped hold must not fail on invalid caller J7 target: {res:?}");
            assert_exact_one_j7(&bus.tx_log(), -2.5, 0, pf_cur, &format!("A4 direct stopped-hold fenced={fenced} {label}"));
        }
    }

    // === Part B: Dual-owner (Legacy + RT Executor) tick-epilogue verification ===
    let mut jt = armd::tracking::JointTrackingConfig::rebot_dm();
    jt.enabled = true;
    for j in &mut jt.joints {
        j.strategy = armd::tracking::TrackingStrategy::Position;
    }
    jt.joints[3].strategy = armd::tracking::TrackingStrategy::Mit;
    jt.joints[3].kp = 10.0;
    jt.joints[3].kd = 0.3;
    jt.joints[3].hold_kp = Some(12.0);
    jt.joints[3].hold_kd = Some(0.4);
    jt.joints[3].effort_limit_nm = 3.0;
    jt.joints[3].gravity_scale = 1.0;

    let setup_mit_rig = |owner: Owner, j7_start: f32| -> DualRig {
        let mut rig = DualRig::new_pos_force(owner, base_cfg(), None);
        rig.inner_mut().config.joint_tracking = Some(jt.clone());
        if let RigOwner::Executor { fx, .. } = &mut rig.owner {
            let stops = fx.stops().clone();
            fx.core.attach_stop_requests(stops);
            let gate = fx.core.estop_gate();
            fx.core.driver.as_ref().unwrap().lock().unwrap().set_estop_gate(gate);
        }
        rig.inner_mut().commanded_positions[6] = j7_start;
        rig.inner_mut().target_positions[6] = j7_start;
        rig.inner_mut().measured_positions[6] = j7_start;
        rig.bus.with(|s| {
            s.motors[6].q = j7_start;
            s.motors[3].set_reg_u32(armd::protocol::damiao::RID_TIMEOUT, 2000);
            s.immediate_replies = true;
            s.push_all_feedback();
        });
        rig.step();
        {
            let hold: Vec<JointTarget> = (0..7)
                .map(|i| JointTarget::mit(rig.inner().measured_positions[i], 0.0, 12.0, 0.4, 0.0))
                .collect();
            let modes: Vec<JointMode> = (0..7)
                .map(|i| if i == 3 { JointMode::Mit } else { JointMode::Position })
                .collect();
            let mut drv = rig.inner().driver.as_ref().unwrap().lock().unwrap();
            drv.set_joint_modes_confirmed(&modes, &hold).expect("handover J4 to MIT");
            drv.set_mit_deadline_fence(Some(MIT_DEADLINE), ms(300));
        }
        rig.inner_mut().mit.owner = MitOwner::Active;
        rig.inner_mut().mit.joints = vec![3];
        rig.inner_mut().mit.last_output = Some(armd::transport::DriverClock::now(&rig.clock));
        rig.bus.with(|s| {
            s.immediate_replies = false;
            s.motors[6].q = j7_start;
            s.rx.clear();
            s.pending.clear();
            s.push_all_feedback();
        });
        rig.step();
        assert!((rig.bus.with(|s| s.motors[6].q) - j7_start).abs() < 1e-2);
        rig
    };

    for owner in [Owner::Legacy, Owner::Executor] {
        // B1. Fenced motion tick with J7 precheck stale (> 100 ms) -> 0 MIT frames + exact 1 FREEZE frame.
        {
            let mut rig = setup_mit_rig(owner, -2.5);
            rig.bus.with(|s| {
                s.rx.clear();
                s.pending.clear();
                s.motors[6].mute_feedback = true;
            });
            rig.clock.advance(ms(115));
            rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            rig.bus.clear_log();
            rig.step_dt(DT, false);
            let log = rig.bus.tx_log();
            assert!(!log.iter().any(is_mit_or_arm_cmd), "{owner:?} B1 sent MIT/arm frame: {log:?}");
            assert_exact_one_j7(&log, -2.5, 0, pf_cur, &format!("{owner:?} B1 fenced motion precheck stale"));
        }

        // B2. Fenced motion tick with fence ALREADY tripped (fresh J7 AND stale J7) -> 0 MIT + exact 1 Hold/Freeze.
        {
            let mut rig = setup_mit_rig(owner, -2.5);
            rig.bus.with(|s| {
                s.rx.clear();
                s.pending.clear();
                s.motors[3].mute_feedback = true;
            });
            rig.clock.advance(ms(50));
            rig.bus.with(|s| for i in [0, 1, 2, 4, 5, 6] { s.push_feedback(i); });
            // Tick 1 trips fence via J4 outage (J7 still fresh at -2.5) -> 0 MIT + exact 1 Hold frame (vel 0).
            rig.bus.clear_log();
            rig.step_dt(DT, false);
            let log1 = rig.bus.tx_log();
            assert!(!log1.iter().any(is_mit_or_arm_cmd), "{owner:?} B2a sent MIT/arm frame: {log1:?}");
            assert_exact_one_j7(&log1, -2.5, 0, pf_cur, &format!("{owner:?} B2a fence tripped (fresh J7)"));

            // Tick 2 while MitOwner::Lost and J7 now stale > 100 ms -> 0 MIT + exact 1 FREEZE frame (vel 0).
            rig.bus.with(|s| s.motors[6].mute_feedback = true);
            rig.clock.advance(ms(115));
            rig.bus.with(|s| for i in [0, 1, 2, 4, 5] { s.push_feedback(i); });
            rig.bus.clear_log();
            rig.step_dt(DT, false);
            let log2 = rig.bus.tx_log();
            assert!(!log2.iter().any(is_mit_or_arm_cmd), "{owner:?} B2b sent MIT/arm frame: {log2:?}");
            assert_exact_one_j7(&log2, -2.5, 0, pf_cur, &format!("{owner:?} B2b fence already tripped (stale J7)"));
        }

        // B3. Fenced motion tick with fence tripping MID-BATCH -> MIT aborts + exact 1 Hold frame (vel 0).
        {
            let mut rig = setup_mit_rig(owner, -2.5);
            // Age last MIT command to exactly 44 ms (< 45 ms MIT deadline so pre-batch check passes
            // on BOTH Legacy and Executor), refresh feedback, then advance +2 ms during J2 transmit
            // so MIT fence trips mid-batch before J4 while tick wall duration stays within max_duration.
            let last_out = rig.inner().mit.last_output.expect("MIT last_output recorded");
            let now = armd::transport::DriverClock::now(&rig.clock);
            let elapsed = now.saturating_duration_since(last_out);
            if elapsed < ms(44) {
                rig.clock.advance(ms(44) - elapsed);
            }
            let clock_mid = rig.clock.clone();
            let tx_count = Arc::new(std::sync::atomic::AtomicUsize::new(0));
            let tx_count_cb = tx_count.clone();
            rig.bus.with(|s| {
                s.push_all_feedback();
                s.on_tx = Some(Arc::new(move || {
                    if tx_count_cb.fetch_add(1, Ordering::SeqCst) == 1 {
                        clock_mid.advance(ms(2));
                    }
                }));
            });
            rig.bus.clear_log();
            rig.step_dt(DT, false);
            let log = rig.bus.tx_log();
            assert!(log.iter().any(|f| f.id == 0x101), "{owner:?} B3 should have started batch before mid-batch trip: {log:?}");
            assert!(!log.iter().any(|f| f.id == 4), "{owner:?} B3 sent J4 MIT frame after mid-batch trip: {log:?}");
            assert_exact_one_j7(&log, -2.5, 0, pf_cur, &format!("{owner:?} B3 mid-batch fence trip"));
        }

        // B4. Invalid caller J7 target on BOTH owners: (a) stopped hold with NaN, (b) operational motion with 0.005 ceiling.
        {
            let mut rig = DualRig::new_pos_force(owner, base_cfg(), None);
            rig.inner_mut().commanded_positions[6] = -2.5;
            rig.inner_mut().target_positions[6] = -2.5;
            rig.inner_mut().measured_positions[6] = -2.5;
            rig.bus.with(|s| {
                s.motors[6].q = -2.5;
                s.push_all_feedback();
            });
            rig.step();

            // B4a: Operational motion with unrepresentable sub-quantum J7 velocity limit (0.005 rad/s).
            rig.inner_mut().config.limits[6].max_velocity = 0.005;
            rig.inner_mut().config.joint_tracking = None;
            let mut tgt = rig.inner().commanded_positions.clone();
            tgt[6] = -0.5;
            rig.inner_mut().servo_target = Some(tgt);
            rig.inner_mut().last_motion_frame = std::time::Instant::now();
            rig.bus.clear_log();
            rig.step();
            let log_motion = rig.bus.tx_log();
            assert!(
                !log_motion.iter().any(|f| (0x101..=0x106).contains(&f.id)),
                "{owner:?} B4a invalid J7 target sent partial arm frames: {log_motion:?}"
            );
            assert_exact_one_j7(&log_motion, -2.5, 0, pf_cur, &format!("{owner:?} B4a invalid caller J7 target in motion"));
            assert!(rig.inner().safety.state().is_stopped(), "{owner:?} B4a must latch fault");

            // B4b: Stopped hold with NaN in commanded_positions[6] -> NEVER suppresses J7 hold!
            rig.inner_mut().commanded_positions[6] = f32::NAN;
            rig.bus.clear_log();
            rig.step();
            assert_exact_one_j7(&rig.bus.tx_log(), -2.5, 0, pf_cur, &format!("{owner:?} B4b invalid NaN J7 target in stopped hold"));
        }

        // B5. `send_to_bus == false` (> 500 ms BUS_LOSS while still torqued) -> 0 J1-J6 frames, exact 1 FREEZE frame!
        {
            let mut rig = DualRig::new_pos_force(owner, base_cfg(), None);
            rig.inner_mut().commanded_positions[6] = -2.5;
            rig.inner_mut().target_positions[6] = -2.5;
            rig.inner_mut().measured_positions[6] = -2.5;
            rig.bus.with(|s| {
                s.motors[6].q = -2.5;
                s.push_all_feedback();
            });
            rig.step();
            rig.bus.with(|s| {
                s.rx.clear();
                s.pending.clear();
                for m in &mut s.motors {
                    m.mute_feedback = true;
                }
            });
            rig.clock.advance(ms(550));
            rig.bus.clear_log();
            rig.step_dt(DT, false);
            assert_eq!(rig.inner().safety.fault_code(), Some(FaultCode::BusLoss));
            let log = rig.bus.tx_log();
            assert!(!log.iter().any(is_mit_or_arm_cmd), "{owner:?} B5 BUS_LOSS sent arm frames: {log:?}");
            assert_exact_one_j7(&log, -2.5, 0, pf_cur, &format!("{owner:?} B5 send_to_bus==false (BUS_LOSS) still emits J7 freeze"));
        }
    }
}

/// Round 7 B1 regression: every J7 `PosForceIntent` carries the `(stop_gen, estop_epoch)` stamp
/// at which it was set, and the single J7 emitter treats any `Target` whose stamp is older than
/// the current generation/epoch as `Hold` (measured hold when fresh, freeze when stale), never
/// re-stamping an old `Target` with a newer stop generation.
///
/// Covers:
/// 1. Direct driver `emit_pos_force_j7`: older `stop_gen` (fresh & stale) and older `estop_epoch`
///    degrade `Target` to `Hold`/`Freeze` (`vel_raw == 0` at measured/last-fresh pos).
/// 2. Dual-owner (Legacy + RT Executor, fresh AND stale J7): `Target` set (closing J7 toward 0.0)
///    -> authorized `STOP` arrives mid-arm-write during J2 transmit -> tick applies `authorized_stop`
///    and epilogue runs on that same tick -> emits exact 1 `0x307` Hold/Freeze frame (`vel_raw == 0`
///    at `-2.5`), NEVER the pre-STOP closing `Target`!
#[test]
fn dual_owner_b1_stop_during_arm_write_degrades_prestamped_j7_target_to_hold() {
    use armd::driver::{
        ArmDriver, DamiaoArmDriver, PosForceEmitOutcome, PosForceIntent, PosForceIntentStamp,
    };
    use armd::rt_core::urgent::StopRequests;

    let pf_cur = j7_pf_current_raw(&base_cfg());

    // 1. Direct-driver check: older `stop_gen` and older `estop_epoch` both degrade `Target` -> `Hold`/`Freeze`.
    {
        let clock = MockClock::new();
        let limits = dm_limits();
        let bus = FakeBus::with_clock(&limits, clock.clone());
        bus.with(|s| {
            s.motors[6].q = -2.5;
            s.immediate_replies = true;
            s.push_all_feedback();
        });
        let pf_cfg = armd::driver::GripperPosForce {
            joint: 6,
            current_raw: pf_cur,
            close_speed_rad_s: 0.30,
            closed_rad: 0.0,
            open_rad: -5.7,
            calibrated: false,
            ceiling_torque_nm: None,
            torque_reference: None,
        };
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits)
            .unwrap()
            .with_gripper_pos_force(pf_cfg)
            .unwrap();
        drv.read_joint_states().unwrap();
        drv.read_joint_states().unwrap();
        drv.check_mapping_ranges();
        drv.set_torque(true).expect("enable");
        bus.with(|s| {
            s.immediate_replies = false;
            s.motors[6].q = -2.5;
            s.rx.clear();
            s.pending.clear();
            s.push_all_feedback();
        });
        drv.read_joint_states().unwrap();

        // Older stop_gen (0 vs current 1), fresh J7 -> Hold at -2.5 with vel_raw = 0 (NOT closing target 0.0).
        let stale_stop_intent = PosForceIntent::target(
            0.0,
            Some(0.30),
            None,
            PosForceIntentStamp { stop_gen: 0, estop_epoch: 1 },
        );
        let newer_stop_current = PosForceIntentStamp { stop_gen: 1, estop_epoch: 1 };
        bus.clear_log();
        drv.begin_tx_tick();
        let out = drv.emit_pos_force_j7(stale_stop_intent, newer_stop_current).unwrap();
        assert!(matches!(out, PosForceEmitOutcome::Sent { joint: 6, .. }));
        let j7 = j7_pf_frames(&bus.tx_log());
        assert_eq!(j7.len(), 1);
        assert!((f32::from_le_bytes(j7[0].data[0..4].try_into().unwrap()) - (-2.5)).abs() < 1e-2);
        assert_eq!(u16::from_le_bytes([j7[0].data[4], j7[0].data[5]]), 0);

        // Older estop_epoch (1 vs current 2), fresh J7 -> Hold at -2.5 with vel_raw = 0.
        let newer_epoch_current = PosForceIntentStamp { stop_gen: 0, estop_epoch: 2 };
        bus.clear_log();
        drv.begin_tx_tick();
        let out2 = drv.emit_pos_force_j7(stale_stop_intent, newer_epoch_current).unwrap();
        assert!(matches!(out2, PosForceEmitOutcome::Sent { joint: 6, .. }));
        let j7_2 = j7_pf_frames(&bus.tx_log());
        assert_eq!(j7_2.len(), 1);
        assert!((f32::from_le_bytes(j7_2[0].data[0..4].try_into().unwrap()) - (-2.5)).abs() < 1e-2);
        assert_eq!(u16::from_le_bytes([j7_2[0].data[4], j7_2[0].data[5]]), 0);

        // Older stop_gen with STALE J7 (> 100 ms) -> Freeze frame at -2.5 with vel_raw = 0.
        bus.with(|s| {
            s.motors[6].mute_feedback = true;
            s.rx.clear();
            s.pending.clear();
        });
        clock.advance(ms(120));
        bus.clear_log();
        drv.begin_tx_tick();
        let out3 = drv.emit_pos_force_j7(stale_stop_intent, newer_stop_current).unwrap();
        assert!(matches!(out3, PosForceEmitOutcome::Sent { joint: 6, freeze: true, .. }));
        let j7_3 = j7_pf_frames(&bus.tx_log());
        assert_eq!(j7_3.len(), 1);
        assert!((f32::from_le_bytes(j7_3[0].data[0..4].try_into().unwrap()) - (-2.5)).abs() < 1e-2);
        assert_eq!(u16::from_le_bytes([j7_3[0].data[4], j7_3[0].data[5]]), 0);
        assert_eq!(u16::from_le_bytes([j7_3[0].data[6], j7_3[0].data[7]]), pf_cur);
    }

    // 2. Dual-owner (Legacy + RT Executor, BOTH fresh J7 and stale J7):
    //    Closing Target set -> STOP fires mid-arm-write during J2 transmit -> authorized_stop
    //    applies -> epilogue emits exact 1 Hold/Freeze frame (vel_raw == 0 at -2.5), NEVER target!
    for owner in [Owner::Legacy, Owner::Executor] {
        for j7_stale in [false, true] {
            let mut rig = DualRig::new_pos_force(owner, base_cfg(), None);
            let stops = match &mut rig.owner {
                RigOwner::Legacy(inner) => {
                    let s = StopRequests::new();
                    inner.attach_stop_requests(s.clone());
                    let gate = inner.estop_gate();
                    inner.driver.as_ref().unwrap().lock().unwrap().set_estop_gate(gate);
                    s
                }
                RigOwner::Executor { fx, .. } => {
                    let s = fx.stops().clone();
                    fx.core.attach_stop_requests(s.clone());
                    let gate = fx.core.estop_gate();
                    fx.core.driver.as_ref().unwrap().lock().unwrap().set_estop_gate(gate);
                    s
                }
            };
            rig.inner_mut().commanded_positions[6] = -2.5;
            rig.inner_mut().target_positions[6] = -2.5;
            rig.inner_mut().measured_positions[6] = -2.5;
            rig.bus.with(|s| {
                s.motors[6].q = -2.5;
                s.push_all_feedback();
            });
            rig.step();

            // Command J7 to close toward 0.0 rad (nonzero velocity -> Target intent).
            {
                let inner = rig.inner_mut();
                let ep = inner.safety.epoch_id();
                let mut goal = inner.commanded_positions.clone();
                goal[6] = 0.0;
                let stamp = inner.current_admission_stamp();
                let admitted = inner.check_admission(stamp).expect("admit close jog");
                SupervisorInner::jog_action(admitted, goal, Some(1.0), Some(ep), "joint_move")
                    .expect("start J7 close motion");
            }
            if j7_stale {
                rig.bus.with(|s| {
                    s.rx.clear();
                    s.pending.clear();
                    s.motors[6].mute_feedback = true;
                });
                rig.clock.advance(ms(115));
                rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            } else {
                rig.bus.with(|s| s.push_all_feedback());
            }

            // Inject authorized STOP mid-arm-write (on J2 transmit, after Target intent was stamped!).
            let stop_gen_before = rig.inner().current_stop_gen();
            let stops_cb = stops.clone();
            let tx_count = Arc::new(std::sync::atomic::AtomicUsize::new(0));
            let tx_count_cb = tx_count.clone();
            rig.bus.with(|s| {
                s.on_tx = Some(Arc::new(move || {
                    if tx_count_cb.fetch_add(1, Ordering::SeqCst) == 1 {
                        let _ = stops_cb.request();
                    }
                }));
            });

            rig.bus.clear_log();
            rig.step_dt(DT, false);
            assert!(
                rig.inner().current_stop_gen() > stop_gen_before,
                "{owner:?} stale={j7_stale}: STOP must have advanced stop_gen"
            );
            assert_eq!(
                stops.current(),
                stops.applied(),
                "{owner:?} stale={j7_stale}: STOP must be applied in the same tick"
            );
            let log = rig.bus.tx_log();
            let j7 = j7_pf_frames(&log);
            assert_eq!(
                j7.len(),
                1,
                "{owner:?} stale={j7_stale}: epilogue must emit exact 1 J7 0x307 hold/freeze frame after mid-write STOP: {log:?}"
            );
            let pos = f32::from_le_bytes(j7[0].data[0..4].try_into().unwrap());
            let vel = u16::from_le_bytes([j7[0].data[4], j7[0].data[5]]);
            let cur = u16::from_le_bytes([j7[0].data[6], j7[0].data[7]]);
            assert!(
                (pos - (-2.5)).abs() < 1e-2,
                "{owner:?} stale={j7_stale}: pre-STOP closing Target leaked after STOP! wire pos={pos} != -2.5"
            );
            assert_eq!(
                vel, 0,
                "{owner:?} stale={j7_stale}: epilogue must emit vel_raw==0 Hold/Freeze after mid-write STOP, got vel_raw={vel}"
            );
            assert_eq!(cur, pf_cur);
        }
    }
}

/// Round-8 B1: a J7 `Target` intent carries `(stop_gen, estop_epoch)` from the A1 `AdmissionStamp`
/// of the command that produced it — never re-stamped at output time and never stamped with
/// `max(local, requested)`. Injecting a STOP right between `mit.rs`'s `urgent_stop_unapplied()`
/// check and intent stamping on BOTH owners (Legacy and RT Executor, fresh and stale J7, plus
/// Legacy without attached `StopRequests`) must emit NO target frame after the STOP — only Hold/Freeze.
#[test]
fn dual_owner_r8_b1_stop_between_pending_check_and_stamping_sends_no_target() {
    use armd::rt_core::urgent::StopRequests;

    let pf_cur = j7_pf_current_raw(&base_cfg());

    // 1. Dual-owner (Legacy + RT Executor, fresh + stale J7) with `StopRequests` injected right
    //    between `mit.rs`'s pending-STOP check and J7 intent stamping.
    for owner in [Owner::Legacy, Owner::Executor] {
        for j7_stale in [false, true] {
            let mut rig = DualRig::new_pos_force(owner, base_cfg(), None);
            let stops = match &mut rig.owner {
                RigOwner::Legacy(inner) => {
                    let s = StopRequests::new();
                    inner.attach_stop_requests(s.clone());
                    let gate = inner.estop_gate();
                    inner.driver.as_ref().unwrap().lock().unwrap().set_estop_gate(gate);
                    s
                }
                RigOwner::Executor { fx, .. } => {
                    let s = fx.stops().clone();
                    fx.core.attach_stop_requests(s.clone());
                    let gate = fx.core.estop_gate();
                    fx.core.driver.as_ref().unwrap().lock().unwrap().set_estop_gate(gate);
                    s
                }
            };
            rig.inner_mut().commanded_positions[6] = -2.5;
            rig.inner_mut().target_positions[6] = -2.5;
            rig.inner_mut().measured_positions[6] = -2.5;
            rig.bus.with(|s| {
                s.motors[6].q = -2.5;
                s.push_all_feedback();
            });
            rig.step();

            // Admit a closing J7 jog at the pre-STOP A1 AdmissionStamp.
            let stop_gen_before = rig.inner().current_stop_gen();
            {
                let inner = rig.inner_mut();
                let ep = inner.safety.epoch_id();
                let mut goal = inner.commanded_positions.clone();
                goal[6] = 0.0;
                let stamp = inner.current_admission_stamp();
                assert_eq!(stamp.stop_gen, stop_gen_before);
                let admitted = inner.check_admission(stamp).expect("admit close jog");
                SupervisorInner::jog_action(admitted, goal, Some(1.0), Some(ep), "joint_move")
                    .expect("start J7 close motion");
            }
            if j7_stale {
                rig.bus.with(|s| {
                    s.rx.clear();
                    s.pending.clear();
                    s.motors[6].mute_feedback = true;
                });
                rig.clock.advance(ms(115));
                rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            } else {
                rig.bus.with(|s| s.push_all_feedback());
            }

            // Inject STOP right between `if self.urgent_stop_unapplied()` and J7 intent stamping.
            // Under the old code (`max(local, requested)` at stamping time), this re-stamped the
            // closing Target with the NEW stop_gen so the epilogue transmitted a closing Target!
            let fired = Arc::new(std::sync::atomic::AtomicBool::new(false));
            let fired_cb = fired.clone();
            let stops_cb = stops.clone();
            rig.inner_mut().post_stop_check_hook = Some(Arc::new(move || {
                if !fired_cb.swap(true, Ordering::SeqCst) {
                    let _ = stops_cb.request();
                }
            }));

            rig.bus.clear_log();
            rig.step_dt(DT, false);
            rig.inner_mut().post_stop_check_hook = None;

            assert!(fired.load(Ordering::SeqCst), "{owner:?} stale={j7_stale}: stop hook did not fire");
            assert!(
                rig.inner().current_stop_gen() > stop_gen_before,
                "{owner:?} stale={j7_stale}: STOP must advance stop_gen"
            );
            assert_eq!(
                stops.current(),
                stops.applied(),
                "{owner:?} stale={j7_stale}: STOP must be applied in the same tick"
            );

            let log = rig.bus.tx_log();
            let j7 = j7_pf_frames(&log);
            assert_eq!(
                j7.len(),
                1,
                "{owner:?} stale={j7_stale}: expected exact 1 J7 Hold/Freeze frame after STOP: {log:?}"
            );
            let pos = f32::from_le_bytes(j7[0].data[0..4].try_into().unwrap());
            let vel = u16::from_le_bytes([j7[0].data[4], j7[0].data[5]]);
            let cur = u16::from_le_bytes([j7[0].data[6], j7[0].data[7]]);
            assert!(
                (pos - (-2.5)).abs() < 1e-2,
                "{owner:?} stale={j7_stale}: pre-STOP closing Target leaked! pos={pos} != -2.5"
            );
            assert_eq!(
                vel, 0,
                "{owner:?} stale={j7_stale}: STOP must degrade Target to vel_raw==0 Hold/Freeze, got {vel}"
            );
            assert_eq!(cur, pf_cur);

            // Next tick with no new admission must STILL emit Hold/Freeze (never a Target).
            rig.bus.clear_log();
            if j7_stale {
                rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            } else {
                rig.bus.with(|s| s.push_all_feedback());
            }
            rig.step_dt(DT, false);
            let j7_next = j7_pf_frames(&rig.bus.tx_log());
            assert_eq!(j7_next.len(), 1, "{owner:?} stale={j7_stale}: follow-up tick must emit 1 Hold/Freeze");
            assert_eq!(
                u16::from_le_bytes([j7_next[0].data[4], j7_next[0].data[5]]),
                0,
                "{owner:?} stale={j7_stale}: follow-up tick without new admission must stay Hold/Freeze"
            );
        }
    }

    // 2. Legacy owner WITHOUT attached `StopRequests`: `stop_gen` advances between the pending-STOP
    //    check and J7 intent stamping -> admission stamp (0) != epilogue current_stamp (1) -> Hold!
    {
        let mut rig = DualRig::new_pos_force(Owner::Legacy, base_cfg(), None);
        rig.inner_mut().commanded_positions[6] = -2.5;
        rig.inner_mut().target_positions[6] = -2.5;
        rig.inner_mut().measured_positions[6] = -2.5;
        rig.bus.with(|s| {
            s.motors[6].q = -2.5;
            s.push_all_feedback();
        });
        rig.step();

        let stop_gen_arc = rig.inner().stop_gen.clone();
        let stop_gen_before = rig.inner().current_stop_gen();
        {
            let inner = rig.inner_mut();
            let ep = inner.safety.epoch_id();
            let mut goal = inner.commanded_positions.clone();
            goal[6] = 0.0;
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).expect("admit close jog");
            SupervisorInner::jog_action(admitted, goal, Some(1.0), Some(ep), "joint_move")
                .expect("start J7 close motion");
        }
        rig.bus.with(|s| s.push_all_feedback());

        let fired = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let fired_cb = fired.clone();
        rig.inner_mut().post_stop_check_hook = Some(Arc::new(move || {
            if !fired_cb.swap(true, Ordering::SeqCst) {
                stop_gen_arc.fetch_add(1, Ordering::SeqCst);
            }
        }));

        rig.bus.clear_log();
        rig.step_dt(DT, false);
        rig.inner_mut().post_stop_check_hook = None;

        assert!(fired.load(Ordering::SeqCst));
        assert_eq!(rig.inner().current_stop_gen(), stop_gen_before + 1);
        let log = rig.bus.tx_log();
        let j7 = j7_pf_frames(&log);
        assert_eq!(j7.len(), 1, "Legacy local stop_gen advance must emit 1 Hold: {log:?}");
        let pos = f32::from_le_bytes(j7[0].data[0..4].try_into().unwrap());
        let vel = u16::from_le_bytes([j7[0].data[4], j7[0].data[5]]);
        assert!((pos - (-2.5)).abs() < 1e-2, "pre-STOP target leaked on Legacy: pos={pos}");
        assert_eq!(vel, 0, "Legacy local stop_gen advance must degrade Target to vel_raw==0 Hold");
    }
}

/// Round-8 M2: transmit priority is by CHANGE, not by joint:
/// 1. Motion -> stop / hold / freeze for ANY joint including J7 (oldest pending first, round-robin).
/// 2. Motion targets.
/// 3. Re-sends of an unchanged hold.
/// 4. J7 `0xCC` refresh and other reads.
///
/// Asserts that a J7 stop or freeze superseding a closing command transmits within
/// `N = ceil(joints / max_frames)` ticks for `max_frames = 1, 2, 6`, with six fresh arm joints,
/// on BOTH owners (Legacy and RT Executor).
#[test]
fn dual_owner_r8_m2_tx_priority_by_change_class1_j7_within_ceil_bound_1_2_6() {
    let pf_cur = j7_pf_current_raw(&base_cfg());
    let joints = 7usize;

    for owner in [Owner::Legacy, Owner::Executor] {
        for max_frames in [1usize, 2, 6] {
            let bound_ticks = joints.div_ceil(max_frames);

            // (a) STOP / E-stop superseding active motion (all 6 arm joints moving + J7 closing):
            //     ALL 7 joints are Class 1 -> ALL 7 (including J7 vel_raw == 0 hold) must reach
            //     the bus within `bound_ticks = ceil(7 / max_frames)`, with 0 Class-4 0xCC frames.
            {
                let mut rig = DualRig::new_pos_force(owner, base_cfg(), None);
                rig.inner_mut().commanded_positions[6] = -2.5;
                rig.inner_mut().target_positions[6] = -2.5;
                rig.inner_mut().measured_positions[6] = -2.5;
                rig.bus.with(|s| {
                    s.motors[6].q = -2.5;
                    s.push_all_feedback();
                });
                rig.step();

                // Admit a 7-joint jog (J1-J6 moving + J7 closing toward 0.0 rad) and run 1 tick
                // so all 7 motors are executing motion on the wire (`vel_raw > 0` on J7).
                {
                    let inner = rig.inner_mut();
                    let ep = inner.safety.epoch_id();
                    let mut goal = inner.commanded_positions.clone();
                    for g in goal.iter_mut().take(6) {
                        *g += 0.25;
                    }
                    goal[6] = 0.0;
                    let stamp = inner.current_admission_stamp();
                    let admitted = inner.check_admission(stamp).expect("admit 7-joint move");
                    SupervisorInner::jog_action(admitted, goal, Some(1.0), Some(ep), "joint_move")
                        .expect("start 7-joint close move");
                }
                rig.bus.clear_log();
                rig.bus.with(|s| s.push_all_feedback());
                rig.step_dt(DT, false);
                let pre_j7 = j7_pf_frames(&rig.bus.tx_log());
                assert_eq!(pre_j7.len(), 1, "{owner:?} max_frames={max_frames}: expected closing J7 frame");
                assert!(
                    u16::from_le_bytes([pre_j7[0].data[4], pre_j7[0].data[5]]) > 0,
                    "{owner:?} max_frames={max_frames}: J7 must be executing closing motion before STOP"
                );

                // Tighten TX budget to `max_frames` and latch E-stop (all 7 joints -> Class 1).
                rig.inner_mut()
                    .driver
                    .as_ref()
                    .unwrap()
                    .lock()
                    .unwrap()
                    .set_tx_budget(TxBudget {
                        max_frames,
                        max_duration: ms(10),
                        frame_max_age: ms(45),
                    });
                rig.inner_mut().apply_stop_policy(armd::StopPolicy::EStopLatch);

                let mut seen_arm = std::collections::BTreeSet::new();
                let mut j7_stop_tick: Option<usize> = None;
                for tick in 1..=bound_ticks {
                    rig.bus.clear_log();
                    rig.bus.with(|s| s.push_all_feedback());
                    rig.step_dt(DT, false);
                    let log = rig.bus.tx_log();
                    assert!(
                        log.len() <= max_frames,
                        "{owner:?} max_frames={max_frames} tick={tick}: exceeded budget: {log:?}"
                    );
                    assert!(
                        j7_refresh_frames(&log).is_empty(),
                        "{owner:?} max_frames={max_frames} tick={tick}: Class-4 0xCC stole slot: {log:?}"
                    );
                    for id in 1u32..=6 {
                        if log.iter().any(|f| f.id == 0x100 + id) {
                            seen_arm.insert(id);
                        }
                    }
                    let j7 = j7_pf_frames(&log);
                    if !j7.is_empty() {
                        assert_eq!(
                            u16::from_le_bytes([j7[0].data[4], j7[0].data[5]]),
                            0,
                            "{owner:?} max_frames={max_frames} tick={tick}: J7 stop frame must have vel_raw == 0"
                        );
                        assert_eq!(u16::from_le_bytes([j7[0].data[6], j7[0].data[7]]), pf_cur);
                        j7_stop_tick.get_or_insert(tick);
                    }
                }
                assert_eq!(
                    seen_arm,
                    (1u32..=6).collect(),
                    "{owner:?} max_frames={max_frames}: all 6 fresh arm joints must receive Class-1 holds within {bound_ticks} ticks"
                );
                assert!(
                    j7_stop_tick.is_some_and(|t| t <= bound_ticks),
                    "{owner:?} max_frames={max_frames}: J7 Class-1 stop must transmit within ceil(7/{max_frames})={bound_ticks} ticks, got {j7_stop_tick:?}"
                );
            }

            // (b) J7 stale freeze superseding a closing command while 6 fresh arm joints continue
            //     moving (Class 1 J7 freeze > Class 2 arm motion targets): J7 freeze transmits on
            //     Tick 1 (strictly <= ceil(7 / max_frames)), then on Tick 2 J7 unchanged freeze is
            //     Class 3 < Class 2 so arm motion targets run ahead of J7 re-sends!
            {
                let mut rig = DualRig::new_pos_force(owner, base_cfg(), None);
                rig.inner_mut().commanded_positions[6] = -2.5;
                rig.inner_mut().target_positions[6] = -2.5;
                rig.inner_mut().measured_positions[6] = -2.5;
                rig.bus.with(|s| {
                    s.motors[6].q = -2.5;
                    s.push_all_feedback();
                });
                rig.step();

                {
                    let inner = rig.inner_mut();
                    let ep = inner.safety.epoch_id();
                    let mut goal = inner.commanded_positions.clone();
                    for g in goal.iter_mut().take(6) {
                        *g += 0.25;
                    }
                    goal[6] = 0.0;
                    let stamp = inner.current_admission_stamp();
                    let admitted = inner.check_admission(stamp).expect("admit close move");
                    SupervisorInner::jog_action(admitted, goal, Some(1.0), Some(ep), "joint_move")
                        .expect("start close move");
                }
                rig.bus.clear_log();
                rig.bus.with(|s| s.push_all_feedback());
                rig.step_dt(DT, false);
                let last_fresh_j7 = rig.inner().measured_positions[6];
                let pre_j7 = j7_pf_frames(&rig.bus.tx_log());
                assert_eq!(pre_j7.len(), 1);
                assert!(u16::from_le_bytes([pre_j7[0].data[4], pre_j7[0].data[5]]) > 0);

                // Mute J7 past 100 ms while keeping all 6 arm joints fresh, and set budget = max_frames.
                rig.bus.with(|s| {
                    s.rx.clear();
                    s.pending.clear();
                    s.motors[6].mute_feedback = true;
                });
                rig.clock.advance(ms(115));
                rig.inner_mut()
                    .driver
                    .as_ref()
                    .unwrap()
                    .lock()
                    .unwrap()
                    .set_tx_budget(TxBudget {
                        max_frames,
                        max_duration: ms(10),
                        frame_max_age: ms(45),
                    });

                // Tick 1: Class-1 J7 freeze MUST preempt Class-2 arm motion targets on Tick 1!
                rig.bus.clear_log();
                rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
                rig.step_dt(DT, false);
                let t1 = rig.bus.tx_log();
                assert!(t1.len() <= max_frames, "{owner:?} max_frames={max_frames}: tick 1 over budget: {t1:?}");
                assert_j7_freeze_frames(
                    &j7_pf_frames(&t1),
                    last_fresh_j7,
                    pf_cur,
                    &format!("{owner:?} max_frames={max_frames} Class-1 J7 freeze over Class-2 motion"),
                );
                assert!(
                    j7_refresh_frames(&t1).is_empty(),
                    "{owner:?} max_frames={max_frames}: Class-4 0xCC must not steal a slot: {t1:?}"
                );

                // Tick 2: J7's unchanged freeze is now Class 3 < Class 2 -> all `max_frames` slots
                // go to Class-2 arm motion targets (0 J7 unchanged freeze re-sends, 0 Class-4 0xCC)!
                rig.bus.clear_log();
                rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
                rig.step_dt(DT, false);
                let t2 = rig.bus.tx_log();
                let arm_t2 = t2.iter().filter(|f| (0x101..=0x106).contains(&f.id)).count();
                assert_eq!(
                    arm_t2, max_frames,
                    "{owner:?} max_frames={max_frames}: Class-2 arm motion must take all {max_frames} slots over Class-3 unchanged freeze: {t2:?}"
                );
                assert!(
                    j7_pf_frames(&t2).is_empty(),
                    "{owner:?} max_frames={max_frames}: Class-3 unchanged J7 freeze must not preempt Class-2 arm motion: {t2:?}"
                );
                assert!(
                    j7_refresh_frames(&t2).is_empty(),
                    "{owner:?} max_frames={max_frames}: Class-4 0xCC must not steal a Class-2 slot: {t2:?}"
                );
            }
        }
    }
}

/// Round 8 D1 (`PosForceBootFallback::NoOutputUntilFresh`) on BOTH owners (Legacy and RT Executor):
/// 1. When J7 has had NO fresh feedback since this `armd` process started (`pf_last_fresh_position == None`),
///    NO `0x307` frame is transmitted while `0xCC` refresh requests stay active (J7 keeps its own
///    last firmware command under `TIMEOUT=0`).
/// 2. Once the first fresh J7 feedback sample is decoded, the next tick emits a `0x307` Hold at
///    that measured position (`vel_raw = 0`) with the configured `current_ceiling_raw`.
/// 3. A STOP issued BEFORE the first fresh sample still sends NO `0x307` frame (while keeping
///    `0xCC` refreshes active), and is applied as Hold at the measured position with the cap once
///    the first fresh sample arrives.
#[test]
fn dual_owner_r8_d1_boot_fallback_no_output_until_fresh_and_stop_before_first_sample() {
    use armd::driver::{PosForceBootFallback, POS_FORCE_BOOT_FALLBACK};
    assert_eq!(POS_FORCE_BOOT_FALLBACK, PosForceBootFallback::NoOutputUntilFresh);
    let pf_cur = j7_pf_current_raw(&base_cfg());

    for owner in [Owner::Legacy, Owner::Executor] {
        // --- (a) & (b): No 0x307 before first fresh sample (0xCC active), then Hold at measured pos with cap ---
        {
            let mut rig = DualRig::new_pos_force(owner, base_cfg(), None);
            // Simulate process startup with J7 already enabled in mode 4, never having decoded a
            // fresh J7 sample yet (and J7 currently muted on the bus).
            rig.bus.with(|s| {
                s.rx.clear();
                s.pending.clear();
                s.motors[6].q = -2.35;
                s.motors[6].mute_feedback = true;
            });
            rig.inner_mut()
                .driver
                .as_ref()
                .unwrap()
                .lock()
                .unwrap()
                .reset_pos_force_boot_state();

            rig.bus.clear_log();
            for _ in 0..3 {
                rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
                rig.step_dt(DT, false);
            }
            let pre_log = rig.bus.tx_log();
            assert!(
                j7_pf_frames(&pre_log).is_empty(),
                "{owner:?}: must emit ZERO 0x307 frames before first fresh J7 sample: {pre_log:?}"
            );
            assert!(
                !j7_refresh_frames(&pre_log).is_empty(),
                "{owner:?}: must keep requesting 0xCC refresh before first fresh J7 sample: {pre_log:?}"
            );

            // First fresh J7 sample arrives at -2.35 rad -> next tick emits Hold at -2.35 rad,
            // vel_raw = 0, i_des = pf_cur.
            rig.bus.clear_log();
            rig.bus.with(|s| {
                s.motors[6].mute_feedback = false;
                s.motors[6].q = -2.35;
                s.push_all_feedback();
            });
            rig.step_dt(DT, false);
            let post_log = rig.bus.tx_log();
            let j7_post = j7_pf_frames(&post_log);
            assert_eq!(
                j7_post.len(),
                1,
                "{owner:?}: expected 1 Hold 0x307 frame after first fresh J7 sample: {post_log:?}"
            );
            let p_wire = f32::from_le_bytes(j7_post[0].data[0..4].try_into().unwrap());
            let v_wire = u16::from_le_bytes([j7_post[0].data[4], j7_post[0].data[5]]);
            let i_wire = u16::from_le_bytes([j7_post[0].data[6], j7_post[0].data[7]]);
            assert!(
                (p_wire - (-2.35)).abs() < 1e-2,
                "{owner:?}: expected Hold at measured -2.35 rad, got {p_wire}"
            );
            assert_eq!(v_wire, 0, "{owner:?}: first fresh Hold must have vel_raw == 0");
            assert_eq!(i_wire, pf_cur, "{owner:?}: first fresh Hold must use configured cap {pf_cur}");
        }

        // --- (c): STOP (both regular stop_motion and E-stop) before the first fresh sample still
        //          sends nothing to J7, and is applied as Hold once fresh ---
        for is_estop in [false, true] {
            let mut rig = DualRig::new_pos_force(owner, base_cfg(), None);
            rig.bus.with(|s| {
                s.rx.clear();
                s.pending.clear();
                s.motors[6].q = -1.65;
                s.motors[6].mute_feedback = true;
            });
            rig.inner_mut()
                .driver
                .as_ref()
                .unwrap()
                .lock()
                .unwrap()
                .reset_pos_force_boot_state();

            if is_estop {
                rig.inner_mut().apply_stop_policy(armd::StopPolicy::EStopLatch);
            } else {
                rig.inner_mut().stop_motion();
            }
            rig.bus.clear_log();
            for _ in 0..3 {
                rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
                rig.step_dt(DT, false);
            }
            let stop_pre_log = rig.bus.tx_log();
            assert!(
                j7_pf_frames(&stop_pre_log).is_empty(),
                "{owner:?} is_estop={is_estop}: STOP before first fresh J7 sample must send ZERO 0x307 frames: {stop_pre_log:?}"
            );
            assert!(
                !j7_refresh_frames(&stop_pre_log).is_empty(),
                "{owner:?} is_estop={is_estop}: STOP before first fresh J7 sample must still request 0xCC refresh: {stop_pre_log:?}"
            );

            // First fresh J7 sample arrives at -1.65 rad -> STOP is applied as Hold at -1.65 rad,
            // vel_raw = 0, i_des = pf_cur.
            rig.bus.clear_log();
            rig.bus.with(|s| {
                s.motors[6].mute_feedback = false;
                s.motors[6].q = -1.65;
                s.push_all_feedback();
            });
            rig.step_dt(DT, false);
            let stop_post_log = rig.bus.tx_log();
            let j7_stop_post = j7_pf_frames(&stop_post_log);
            assert_eq!(
                j7_stop_post.len(),
                1,
                "{owner:?} is_estop={is_estop}: STOP must apply as 1 Hold 0x307 frame once first fresh sample arrives: {stop_post_log:?}"
            );
            let p_wire = f32::from_le_bytes(j7_stop_post[0].data[0..4].try_into().unwrap());
            let v_wire = u16::from_le_bytes([j7_stop_post[0].data[4], j7_stop_post[0].data[5]]);
            let i_wire = u16::from_le_bytes([j7_stop_post[0].data[6], j7_stop_post[0].data[7]]);
            assert!(
                (p_wire - (-1.65)).abs() < 1e-2,
                "{owner:?} is_estop={is_estop}: expected STOP Hold at measured -1.65 rad, got {p_wire}"
            );
            assert_eq!(v_wire, 0, "{owner:?} is_estop={is_estop}: STOP Hold once fresh must have vel_raw == 0");
            assert_eq!(i_wire, pf_cur, "{owner:?} is_estop={is_estop}: STOP Hold once fresh must use configured cap {pf_cur}");
        }
    }
}

/// Torque-on pre-enable J7 Hold invariant on BOTH owners (Legacy and RT Executor):
/// The very first `0x307` transmitted after fresh J7 feedback on the torque-on path MUST be a
/// zero-velocity `Hold` (`vel_raw == 0`, `position == measured`, `i_des == current_ceiling_raw`),
/// obeying `NoOutputUntilFresh -> Hold` rather than emitting a `Target` carrying `close_speed`.
/// Verified for both cold mode-4 restore (`already_mode_4 == false`) and host-restart readback
/// (`already_mode_4 == true`), starting with J7 silent (no fresh feedback yet -> 0 `0x307` frames)
/// until fresh J7 feedback arrives during the torque-on handshake.
#[test]
fn dual_owner_torque_on_first_0x307_is_zero_vel_hold_at_measured_position() {
    let pf_cur = j7_pf_current_raw(&base_cfg());
    let j7_q = -1.85_f32;

    for owner in [Owner::Legacy, Owner::Executor] {
        for already_mode_4 in [false, true] {
            let cfg = base_cfg();
            let limits = dm_limits();
            let clock = MockClock::new();
            let bus = FakeBus::with_clock(&limits, clock.clone());
            bus.with(|s| {
                s.motors[6].q = j7_q;
                s.motors[6].set_reg_u32(
                    armd::protocol::damiao::RID_CTRL_MODE,
                    if already_mode_4 {
                        armd::protocol::damiao::CTRL_MODE_POS_FORCE
                    } else {
                        armd::protocol::damiao::CTRL_MODE_POS_VEL
                    },
                );
                // Start with J7 feedback muted so no fresh J7 sample exists yet.
                s.motors[6].mute_feedback = true;
                s.immediate_replies = true;
                s.follow_commands = true;
                for i in 0..6 {
                    s.push_feedback(i);
                }
            });
            let tx_budget = cfg.damiao_tx_budget();
            let grip = armd::driver::gripper_drive_from_profile(&cfg).expect("pos-force gripper drive");
            let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
                .with_joint_limits(limits)
                .unwrap()
                .with_gripper_drive(grip)
                .unwrap()
                .with_tx_budget(tx_budget);
            drv.read_joint_states().unwrap();
            drv.read_joint_states().unwrap();
            drv.check_mapping_ranges();
            assert_eq!(drv.pos_force_last_fresh_position(), None);

            let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
            inner.lease_enabled = false;
            inner.tick(DT);
            bus.clear_log();

            match owner {
                Owner::Legacy => {
                    // Before fresh J7 feedback arrives, idle ticks send ZERO 0x307 frames.
                    for _ in 0..2 {
                        clock.advance(ms(20));
                        bus.with(|s| for i in 0..6 { s.push_feedback(i); });
                        inner.tick(DT);
                    }
                    assert!(
                        j7_pf_frames(&bus.tx_log()).is_empty(),
                        "Legacy already_mode_4={already_mode_4}: must emit ZERO 0x307 before fresh feedback"
                    );

                    // Deliver fresh stationary torque-off J7 feedback and run torque-on.
                    bus.clear_log();
                    clock.advance(ms(20));
                    bus.with(|s| {
                        s.motors[6].mute_feedback = false;
                        s.motors[6].q = j7_q;
                        s.push_all_feedback();
                    });
                    inner.tick(DT);
                    assert!(
                        j7_pf_frames(&bus.tx_log()).is_empty(),
                        "Legacy already_mode_4={already_mode_4}: disabled tick must not emit 0x307"
                    );

                    let stamp = inner.current_admission_stamp();
                    let admitted = inner.check_admission(stamp).expect("admit torque-on");
                    SupervisorInner::enable_torque(admitted).expect("legacy torque-on");

                    let log = bus.tx_log();
                    let first_pf_idx = log
                        .iter()
                        .position(|f| f.id == 0x307)
                        .expect("Legacy: expected pre-enable 0x307 Hold");
                    let enable_idx = log
                        .iter()
                        .position(|f| f.id == 7 && f.data == armd::protocol::damiao::CMD_ENABLE)
                        .expect("Legacy: expected J7 CMD_ENABLE");
                    assert!(
                        first_pf_idx < enable_idx,
                        "Legacy already_mode_4={already_mode_4}: first 0x307 ({first_pf_idx}) must precede CMD_ENABLE ({enable_idx})"
                    );
                    let first = log[first_pf_idx];
                    let p_wire = f32::from_le_bytes(first.data[0..4].try_into().unwrap());
                    let v_wire = u16::from_le_bytes([first.data[4], first.data[5]]);
                    let i_wire = u16::from_le_bytes([first.data[6], first.data[7]]);
                    assert!(
                        (p_wire - j7_q).abs() < 1e-2,
                        "Legacy already_mode_4={already_mode_4}: first 0x307 pos={p_wire} != measured {j7_q}"
                    );
                    assert_eq!(
                        v_wire, 0,
                        "Legacy already_mode_4={already_mode_4}: first 0x307 on torque-on must be zero-velocity Hold (vel_raw=0), got {v_wire}"
                    );
                    assert_eq!(i_wire, pf_cur);

                    // Confirm subsequent operational tick succeeds with zero faults after torque-on.
                    bus.clear_log();
                    bus.with(|s| {
                        s.immediate_replies = false;
                        s.push_all_feedback();
                    });
                    clock.advance(ms(20));
                    inner.tick(DT);
                    assert!(
                        inner.safety.state().is_operational(),
                        "Legacy already_mode_4={already_mode_4}: expected operational after torque-on: {:?}",
                        inner.safety.state()
                    );
                }
                Owner::Executor => {
                    inner.rt_executor_owned = true;
                    inner.register_live_connection(1).unwrap();
                    let (mut fx, mut broker, _tw, _stop) = ReleaseFixture::new_with_clock(
                        inner,
                        BootId(89),
                        Capacities { commands: 8, plans: 4, replies: 16 },
                        VirtualClock::from_mock_clock(clock.clone()),
                    );

                    // Submit TorqueOn while J7 is STILL muted: step 1 commits the transition
                    // (`budget = 0`), step 2 polls the handshake which sends 0xCC refresh and
                    // emits ZERO 0x307 frames until fresh J7 feedback arrives!
                    let auth = fx.core.authority(&fx.ctx(fx.now()));
                    broker
                        .submit(
                            ClientId(1),
                            1,
                            auth,
                            fx.now() + Duration::from_secs(5),
                            Submit::Transition(TransitionOp::TorqueOn),
                        )
                        .expect("submit TorqueOn");
                    fx.dt = DT;
                    let rep = fx.step();
                    assert!(
                        rep.command.as_ref().is_some_and(|(_, r)| r.is_ok()),
                        "Executor already_mode_4={already_mode_4}: TorqueOn submit: {rep:?}"
                    );
                    // Step 2 polls `RefreshBeforeRestore` (when `!already_mode_4`, 20 ms < 50 ms
                    // reply deadline) or sends `REG_READ` readback (`already_mode_4`); step 3
                    // enters `PreEnableRefresh` (`already_mode_4`) so both paths have sent `0xCC`
                    // while staying inside the 50 ms reply window with ZERO `0x307` frames.
                    let pre_steps = if already_mode_4 { 2 } else { 1 };
                    for _ in 0..pre_steps {
                        bus.with(|s| for i in 0..6 { s.push_feedback(i); });
                        fx.step();
                    }
                    assert!(
                        fx.core.torque_transition_pending(),
                        "Executor already_mode_4={already_mode_4}: TorqueOn must stay pending while J7 lacks fresh feedback"
                    );
                    let pre_log = bus.tx_log();
                    assert!(
                        j7_pf_frames(&pre_log).is_empty(),
                        "Executor already_mode_4={already_mode_4}: must emit ZERO 0x307 before first fresh J7 sample: {pre_log:?}"
                    );
                    assert!(
                        !j7_refresh_frames(&pre_log).is_empty(),
                        "Executor already_mode_4={already_mode_4}: expected 0xCC refresh while awaiting fresh J7 sample: {pre_log:?}"
                    );

                    // Unmute J7 and deliver fresh stationary torque-off feedback -> TorqueOn completes,
                    // and the VERY FIRST 0x307 on the wire is a zero-velocity Hold at j7_q with pf_cur!
                    bus.clear_log();
                    bus.with(|s| {
                        s.motors[6].mute_feedback = false;
                        s.motors[6].q = j7_q;
                        s.push_all_feedback();
                    });
                    for _ in 0..10 {
                        if !fx.core.torque_transition_pending() {
                            break;
                        }
                        bus.with(|s| s.push_all_feedback());
                        fx.step();
                    }
                    assert!(
                        fx.core.is_torqued && !fx.core.torque_transition_pending(),
                        "Executor already_mode_4={already_mode_4}: TorqueOn must complete once fresh J7 feedback arrives"
                    );

                    let log = bus.tx_log();
                    let first_pf_idx = log
                        .iter()
                        .position(|f| f.id == 0x307)
                        .expect("Executor: expected pre-enable 0x307 Hold");
                    let enable_idx = log
                        .iter()
                        .position(|f| f.id == 7 && f.data == armd::protocol::damiao::CMD_ENABLE)
                        .expect("Executor: expected J7 CMD_ENABLE");
                    assert!(
                        first_pf_idx < enable_idx,
                        "Executor already_mode_4={already_mode_4}: first 0x307 ({first_pf_idx}) must precede CMD_ENABLE ({enable_idx})"
                    );
                    let first = log[first_pf_idx];
                    let p_wire = f32::from_le_bytes(first.data[0..4].try_into().unwrap());
                    let v_wire = u16::from_le_bytes([first.data[4], first.data[5]]);
                    let i_wire = u16::from_le_bytes([first.data[6], first.data[7]]);
                    assert!(
                        (p_wire - j7_q).abs() < 1e-2,
                        "Executor already_mode_4={already_mode_4}: first 0x307 pos={p_wire} != measured {j7_q}"
                    );
                    assert_eq!(
                        v_wire, 0,
                        "Executor already_mode_4={already_mode_4}: first 0x307 on torque-on must be zero-velocity Hold (vel_raw=0), got {v_wire}"
                    );
                    assert_eq!(i_wire, pf_cur);

                    // Confirm subsequent operational RT tick succeeds with zero faults after torque-on.
                    bus.clear_log();
                    bus.with(|s| {
                        s.immediate_replies = false;
                        s.push_all_feedback();
                    });
                    fx.step();
                    assert!(
                        fx.core.safety.state().is_operational(),
                        "Executor already_mode_4={already_mode_4}: expected operational after torque-on: {:?}",
                        fx.core.safety.state()
                    );
                }
            }
        }
    }
}

/// PR P + J7 single effective TX cutoff & stop/hold/freeze exception on BOTH owners
/// (Legacy and RT Executor):
/// (a) Fresh J7 `Target` motion respects the active action/macro deadline (`drv_action_cutoff`):
///     ZERO `0x307` target frames reach the wire after the deadline, with no `CAN_TX_STALLED`.
/// (b) Stale J7 (`> 100 ms`) FREEZE frame STILL goes out after an expired action/macro deadline
///     (while arm motion frames are refused by the cutoff).
/// (c) STOP arriving at/after an expired action/macro deadline STILL transmits J7's zero-velocity
///     STOP HOLD frame (`vel_raw == 0`, measured pos, configured cap).
/// (d) Mid-batch arm TX crossing the action/macro deadline (after `TickTxPass` was already opened
///     under `drv_action_cutoff`) refuses remaining arm motion frames while STILL transmitting
///     J7's FREEZE frame (proving `TickTxPass::deadline` is not poisoned by an expired action cutoff).
#[test]
fn dual_owner_tx_cutoff_blocks_j7_target_but_allows_stop_hold_freeze() {
    use std::sync::atomic::AtomicBool;
    let pf_cur = j7_pf_current_raw(&base_cfg());
    let wide_budget = TxBudget { max_frames: 16, max_duration: ms(15), frame_max_age: ms(45) };

    // Helper: create a DualRig with an active sequence macro whose parent deadline is `now + 5 ms`
    // and whose running child drives J1..J6 + closes J7 toward 0.0 rad.
    let setup_macro_close = |owner: Owner| -> (DualRig, u64) {
        let mut rig = DualRig::new_pos_force(owner, base_cfg(), Some(wide_budget));
        rig.inner_mut().commanded_positions[6] = -2.5;
        rig.inner_mut().target_positions[6] = -2.5;
        rig.inner_mut().measured_positions[6] = -2.5;
        rig.bus.with(|s| {
            s.motors[6].q = -2.5;
            s.push_all_feedback();
        });
        rig.step();

        let macro_id = {
            let inner = rig.inner_mut();
            let ep = inner.safety.epoch_id();
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).expect("admit sequence macro");
            let id = SupervisorInner::begin_sequence_action(admitted, Some("cutoff-test".into()), ep)
                .expect("begin sequence macro");
            let mut goal = inner.commanded_positions.clone();
            for g in goal.iter_mut().take(6) {
                *g += 0.25;
            }
            goal[6] = 0.0;
            let limits = inner.planning_limits();
            let stamp2 = inner.current_admission_stamp();
            let admitted2 = inner.check_admission(stamp2).expect("admit macro child");
            SupervisorInner::install_macro_quintic(admitted2, id, goal, &limits)
                .expect("install macro child");
            id
        };
        // Step 1 tick before deadline so the macro child is actively executing motion on J1..J7.
        rig.bus.clear_log();
        rig.step();
        let pre_j7 = j7_pf_frames(&rig.bus.tx_log());
        assert_eq!(pre_j7.len(), 1, "{owner:?}: expected closing 0x307 before deadline");
        assert!(
            u16::from_le_bytes([pre_j7[0].data[4], pre_j7[0].data[5]]) > 0,
            "{owner:?}: J7 must be executing Target motion before cutoff test"
        );
        (rig, macro_id)
    };

    for owner in [Owner::Legacy, Owner::Executor] {
        // --- (a) Fresh J7 Target motion blocked by expired action/macro deadline ---
        {
            let (mut rig, macro_id) = setup_macro_close(owner);
            let dl = rig.clock.now() + ms(5);
            if let Some(m) = rig.inner_mut().macro_run.as_mut() {
                m.deadline = Some(dl);
                m.timeout_code = armd::safety::BoundedText::new("SEQUENCE_TIMEOUT");
            }
            let clk = rig.clock.clone();
            let fired = Arc::new(AtomicBool::new(false));
            let fired_cb = Arc::clone(&fired);
            rig.inner_mut().post_stop_check_hook = Some(Arc::new(move || {
                if !fired_cb.swap(true, Ordering::SeqCst) {
                    clk.advance(ms(8)); // 8 ms > 5 ms action cutoff, < 15 ms tx budget < 18 ms release cap
                }
            }));

            rig.bus.clear_log();
            rig.bus.with(|s| s.push_all_feedback());
            rig.step_dt(DT, false);
            rig.inner_mut().post_stop_check_hook = None;

            assert!(fired.load(Ordering::SeqCst), "{owner:?} (a): hook must have fired");
            let log = rig.bus.tx_log();
            assert!(
                log.iter().all(|f| !(0x101..=0x106).contains(&f.id)),
                "{owner:?} (a): expected ZERO arm motion frames past action cutoff: {log:?}"
            );
            assert!(
                j7_pf_frames(&log).is_empty(),
                "{owner:?} (a): fresh J7 Target MUST respect action/macro deadline cutoff (0 frames): {log:?}"
            );
            assert!(
                rig.inner().safety.state().is_operational(),
                "{owner:?} (a): action cutoff must NOT latch CAN_TX_STALLED: {:?}",
                rig.inner().safety.state()
            );
            assert!(
                rig.inner().macro_run.as_ref().is_none_or(|m| m.id != macro_id || m.finishing),
                "{owner:?} (a): expired macro must terminate cleanly after cutoff"
            );
        }

        // --- (b) EXCEPTION: Stale J7 (> 100 ms) FREEZE frame STILL goes out after action deadline ---
        {
            let (mut rig, _macro_id) = setup_macro_close(owner);
            let last_fresh_j7 = rig.inner().measured_positions[6];
            rig.bus.with(|s| {
                s.rx.clear();
                s.pending.clear();
                s.motors[6].mute_feedback = true;
            });
            rig.clock.advance(ms(115));

            let dl = rig.clock.now() + ms(5);
            if let Some(m) = rig.inner_mut().macro_run.as_mut() {
                m.deadline = Some(dl);
                m.timeout_code = armd::safety::BoundedText::new("SEQUENCE_TIMEOUT");
            }
            let clk = rig.clock.clone();
            let fired = Arc::new(AtomicBool::new(false));
            let fired_cb = Arc::clone(&fired);
            rig.inner_mut().post_stop_check_hook = Some(Arc::new(move || {
                if !fired_cb.swap(true, Ordering::SeqCst) {
                    clk.advance(ms(8));
                }
            }));

            rig.bus.clear_log();
            rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            rig.step_dt(DT, false);
            rig.inner_mut().post_stop_check_hook = None;

            assert!(fired.load(Ordering::SeqCst), "{owner:?} (b): hook must have fired");
            let log = rig.bus.tx_log();
            assert!(
                log.iter().all(|f| !(0x101..=0x106).contains(&f.id)),
                "{owner:?} (b): arm motion frames must still be blocked past action cutoff: {log:?}"
            );
            assert_j7_freeze_frames(
                &j7_pf_frames(&log),
                last_fresh_j7,
                pf_cur,
                &format!("{owner:?} (b) J7 FREEZE after expired action deadline"),
            );
            assert!(
                rig.inner().safety.state().is_operational(),
                "{owner:?} (b): stale-J7 freeze past action cutoff must NOT latch CAN_TX_STALLED: {:?}",
                rig.inner().safety.state()
            );
        }

        // --- (c) EXCEPTION: STOP arriving at/after action deadline STILL sends J7 STOP HOLD ---
        {
            let (mut rig, _macro_id) = setup_macro_close(owner);
            let stops = armd::rt_core::urgent::StopRequests::new();
            rig.inner_mut().attach_stop_requests(stops.clone());
            let dl = rig.clock.now() + ms(5);
            if let Some(m) = rig.inner_mut().macro_run.as_mut() {
                m.deadline = Some(dl);
                m.timeout_code = armd::safety::BoundedText::new("SEQUENCE_TIMEOUT");
            }
            let clk = rig.clock.clone();
            let stops_cb = stops.clone();
            let fired = Arc::new(AtomicBool::new(false));
            let fired_cb = Arc::clone(&fired);
            rig.inner_mut().post_stop_check_hook = Some(Arc::new(move || {
                if !fired_cb.swap(true, Ordering::SeqCst) {
                    clk.advance(ms(8));
                    let _ = stops_cb.request();
                }
            }));

            rig.bus.clear_log();
            rig.bus.with(|s| s.push_all_feedback());
            rig.step_dt(DT, false);
            rig.inner_mut().post_stop_check_hook = None;

            assert!(fired.load(Ordering::SeqCst), "{owner:?} (c): hook must have fired");
            let log = rig.bus.tx_log();
            assert!(
                log.iter().all(|f| !(0x101..=0x106).contains(&f.id)),
                "{owner:?} (c): arm motion must not go out after cutoff + STOP: {log:?}"
            );
            let j7 = j7_pf_frames(&log);
            assert_eq!(
                j7.len(),
                1,
                "{owner:?} (c): STOP after expired action deadline MUST still emit J7 Hold frame: {log:?}"
            );
            let v_wire = u16::from_le_bytes([j7[0].data[4], j7[0].data[5]]);
            let i_wire = u16::from_le_bytes([j7[0].data[6], j7[0].data[7]]);
            assert_eq!(v_wire, 0, "{owner:?} (c): J7 STOP Hold must have vel_raw == 0");
            assert_eq!(i_wire, pf_cur, "{owner:?} (c): J7 STOP Hold must carry configured cap");
        }

        // --- (d) EXCEPTION: mid-batch arm TX crosses action deadline (TickTxPass already open
        //         under drv_action_cutoff) + stale J7 -> arm stops at cutoff, J7 FREEZE STILL sends ---
        {
            let (mut rig, _macro_id) = setup_macro_close(owner);
            let last_fresh_j7 = rig.inner().measured_positions[6];
            rig.bus.with(|s| {
                s.rx.clear();
                s.pending.clear();
                s.motors[6].mute_feedback = true;
            });
            rig.clock.advance(ms(115));

            let dl = rig.clock.now() + ms(5);
            if let Some(m) = rig.inner_mut().macro_run.as_mut() {
                m.deadline = Some(dl);
                m.timeout_code = armd::safety::BoundedText::new("SEQUENCE_TIMEOUT");
            }
            let clk = rig.clock.clone();
            let fired = Arc::new(AtomicBool::new(false));
            let fired_cb = Arc::clone(&fired);
            rig.bus.with(|s| {
                s.on_tx = Some(Arc::new(move || {
                    if !fired_cb.swap(true, Ordering::SeqCst) {
                        // Frame 0 (J1) sent before action cutoff; advance clock across 5 ms cutoff
                        // while staying strictly inside 15 ms tx budget and 18 ms release cap.
                        clk.advance(ms(8));
                    }
                }));
            });

            rig.bus.clear_log();
            rig.bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            rig.step_dt(DT, false);
            rig.bus.with(|s| s.on_tx = None);

            assert!(fired.load(Ordering::SeqCst), "{owner:?} (d): mid-batch on_tx hook must fire");
            let log = rig.bus.tx_log();
            let arm_frames: Vec<_> = log.iter().filter(|f| (0x101..=0x106).contains(&f.id)).collect();
            assert_eq!(
                arm_frames.len(),
                1,
                "{owner:?} (d): mid-batch cutoff must stop arm motion after frame 0: {log:?}"
            );
            assert_j7_freeze_frames(
                &j7_pf_frames(&log),
                last_fresh_j7,
                pf_cur,
                &format!("{owner:?} (d) J7 FREEZE after mid-batch action cutoff (unpoisoned TickTxPass)"),
            );
            assert!(
                rig.inner().safety.state().is_operational(),
                "{owner:?} (d): mid-batch cutoff + J7 freeze must NOT latch CAN_TX_STALLED: {:?}",
                rig.inner().safety.state()
            );
        }
    }
}
