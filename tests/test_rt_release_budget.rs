//! Primitive 4 (`ReleaseBudget`) verification suite.
//!
//! Covers:
//! 1. Single mutable `ReleaseBudget` ledger charging discarded RX frames and budgeted TX attempts
//!    where the transport consumes them, capping combined Step 2 + transition + ModeSwitch +
//!    Window-2 wake + output bus I/O to one ledger.
//! 2. Exhaustion on RX, TX, or monotonic deadline returns pending with zero extra bus I/O and
//!    resumes cleanly at the next eligible release.
//! 3. Sub-budgets (`SubBudget`, `ModeSwitch::step_with_gate_and_stop_proof_ledger`) may only
//!    tighten limits and draw directly from the parent ledger.
//! 4. Dual-owner parity (`OWNER_LEGACY` vs `OWNER_EXECUTOR`) under tight budget limits.

mod mit_support;

use std::sync::Arc;
use std::time::{Duration, Instant};

use armd::auth::OperatorGrant;
use armd::driver::{
    gripper_drive_from_profile, ArmDriver, DamiaoArmDriver, EnableState, TxBudget,
};
use armd::model::ArmConfig;
use armd::motor_mode::{
    ControlMode, ModeProgress, ModeSwitch, ModeSwitchPlan, ReleaseBudget as ModeSwitchBudget,
};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::broker::{
    Authority, BootId, Capacities, ClientId, Outcome, Reply, Submit, TransitionOp,
};
use armd::rt_core::budget::ReleaseBudget;
use armd::rt_core::executor::ExecutorConfig;
use armd::rt_core::ops::{CoreOps, ReleaseFixture};
use armd::rt_core::requests::config::ModeSwitchStartRequest;
use armd::supervisor::{ArmSupervisor, SupervisorInner};
use armd::transport::{CanFrame, MockClock};
use mit_support::FakeBus;

const BOOT: BootId = BootId(0xB400_0004);
const TIMEOUT_2000_COUNTS: u32 = 2000;

fn b601_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm()
        .damiao_ranges
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect()
}

fn make_bus_and_driver(
    clock: Option<MockClock>,
    tx_max_frames: usize,
    immediate_replies: bool,
) -> (FakeBus, DamiaoArmDriver<FakeBus>, ArmConfig) {
    let cfg = ArmConfig::rebot_dm();
    let limits = b601_limits();
    let bus = match clock {
        Some(c) => FakeBus::with_clock(&limits, c),
        None => FakeBus::new(&limits),
    };
    bus.with(|s| {
        s.tx_step = Duration::from_micros(200);
        s.poll_feeds_watchdog = false;
        s.immediate_replies = immediate_replies;
        for (i, m) in s.motors.iter_mut().enumerate() {
            m.q = 0.0;
            m.v = 0.0;
            m.enabled = false;
            m.set_reg_u32(
                damiao::RID_TIMEOUT,
                if i < 6 { TIMEOUT_2000_COUNTS } else { 0 },
            );
            m.set_reg_u32(
                damiao::RID_CTRL_MODE,
                if i == 6 {
                    damiao::CTRL_MODE_POS_FORCE
                } else {
                    damiao::CTRL_MODE_POS_VEL
                },
            );
        }
        s.push_all_feedback();
    });
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap()
        .with_gripper_drive(gripper_drive_from_profile(&cfg).unwrap())
        .unwrap();
    let mut b = TxBudget::default();
    b.max_frames = tx_max_frames;
    drv.set_tx_budget(b);
    drv.set_rt_mode(true);
    let _ = drv.check_mapping_ranges();
    let _ = drv.read_joint_states();
    (bus, drv, cfg)
}

fn stray_frame(tag: u8) -> CanFrame {
    // Standard CAN ID not matching any motor feedback/register reply ID — decoded & discarded.
    CanFrame::new_standard(0x555, &[tag, 0, 0, 0, 0, 0, 0, 0]).unwrap()
}

fn all_motors_enabled(bus: &FakeBus) -> bool {
    bus.with(|s| s.motors.iter().all(|m| m.enabled))
}

#[test]
fn discarded_rx_frames_charged_and_rx_exhaustion_resumes_next_release() {
    let clock = MockClock::new();
    let (bus, drv, cfg) = make_bus_and_driver(Some(clock.clone()), 16, true);
    let mut core = SupervisorInner::new(cfg).with_driver(Box::new(drv));
    core.lease_enabled = false;
    core.rt_executor_owned = true;
    core.register_live_connection(1).unwrap();
    core.set_boot_id(BOOT);

    let (mut fix, mut broker, _tw, _stops) =
        ReleaseFixture::new(core, BOOT, Capacities::default());
    fix = fix.with_release_budget_limits(8, Some(16));

    // Warm up 1 normal release so SupervisorInner has baseline valid feedback.
    let _ = fix.step_with_clock(|| clock.now());

    // Seed 12 discarded/stray RX frames ahead of valid feedback.
    bus.with(|s| {
        s.rx.clear();
        for i in 0..12u8 {
            s.rx.push_back(stray_frame(i));
        }
        s.push_all_feedback();
    });
    let queued_before = bus.with(|s| s.rx.len());
    assert_eq!(queued_before, 19);

    // Release 2 must stop at rx_limit = 8 (all 8 frames were discarded stray frames).
    clock.advance(Duration::from_millis(10));
    let _rep1 = fix.step_with_clock(|| clock.now());
    let b1 = fix.last_release_budget().expect("release 1 ledger");
    let queued_after_r1 = bus.with(|s| s.rx.len());
    assert_eq!(
        queued_before - queued_after_r1,
        8,
        "transport must consume at most rx_limit=8 frames including discarded frames"
    );
    assert_eq!(
        b1.rx_used(),
        8,
        "discarded RX frames must be charged to ReleaseBudget"
    );
    assert_eq!(b1.rx_remaining(), 0);
    assert!(b1.rx_exhausted());

    // Submit TorqueOn while stray frames saturate RX in the next release -> stays pending.
    let auth = fix.core.authority(&fix.ctx(clock.now()));
    let _req = broker
        .submit(
            ClientId(1),
            1,
            auth,
            clock.now() + Duration::from_secs(5),
            Submit::Transition(TransitionOp::TorqueOn),
        )
        .unwrap();
    bus.with(|s| {
        s.rx.clear();
        for i in 0..12u8 {
            s.rx.push_back(stray_frame(i));
        }
    });
    clock.advance(Duration::from_millis(10));
    let _rep2 = fix.step_with_clock(|| clock.now());
    let b2 = fix.last_release_budget().expect("release 2 ledger");
    assert_eq!(
        b2.rx_used(),
        8,
        "release 2 combined poll+transition must stay within rx_limit=8"
    );
    assert!(b2.rx_exhausted());
    assert!(
        !all_motors_enabled(&bus),
        "RX exhaustion must leave transition pending"
    );

    // Next eligible releases without stray flood: transition resumes and completes cleanly
    // even while rx_limit remains tightened at 8 frames/release.
    bus.with(|s| {
        s.rx.clear();
        s.push_all_feedback();
    });
    let mut terminal_completed = false;
    let mut seen_replies = Vec::new();
    for _ in 0..20 {
        clock.advance(Duration::from_millis(10));
        bus.with(|s| {
            if s.rx.is_empty() {
                s.push_all_feedback();
            }
        });
        let _rep = fix.step_with_clock(|| clock.now());
        let b = fix.last_release_budget().expect("release ledger");
        assert!(b.rx_used() <= 8, "every release must stay within rx_limit=8 (got {})", b.rx_used());
        broker.poll(|_, r| {
            if matches!(r, Reply::Terminal { outcome: Outcome::Completed, .. }) {
                terminal_completed = true;
            }
            seen_replies.push(r);
            true
        });
        if terminal_completed && all_motors_enabled(&bus) {
            break;
        }
    }
    assert!(
        terminal_completed && all_motors_enabled(&bus),
        "transition must resume and complete on subsequent eligible releases (completed={terminal_completed}, enabled={}, replies={seen_replies:?}, last_action={:?}, safety={:?})",
        all_motors_enabled(&bus),
        fix.core.last_action,
        fix.core.safety.state(),
    );
}

#[test]
fn tx_exhaustion_caps_attempts_per_release_and_resumes_cleanly() {
    let clock = MockClock::new();
    let (bus, drv, cfg) = make_bus_and_driver(Some(clock.clone()), 16, true);
    let mut core = SupervisorInner::new(cfg).with_driver(Box::new(drv));
    core.lease_enabled = false;
    core.rt_executor_owned = true;
    core.register_live_connection(1).unwrap();
    core.set_boot_id(BOOT);

    // Tight TX cap of 4 attempts per release (fewer than 7 motors; driver allows 16, ledger tightens to 4).
    let (mut fix, mut broker, _tw, _stops) =
        ReleaseFixture::new(core, BOOT, Capacities::default());
    fix = fix.with_release_budget_limits(64, Some(4));

    let auth = fix.core.authority(&fix.ctx(clock.now()));
    let _req = broker
        .submit(
            ClientId(1),
            1,
            auth,
            clock.now() + Duration::from_secs(5),
            Submit::Transition(TransitionOp::TorqueOn),
        )
        .unwrap();

    bus.with(|s| {
        s.tx_log.clear();
        s.push_all_feedback();
    });
    clock.advance(Duration::from_millis(10));
    let _rep1 = fix.step_with_clock(|| clock.now());
    let tx_r1 = bus.with(|s| s.tx_log.len());
    let b1 = fix.last_release_budget().expect("release 1 ledger");
    assert!(
        tx_r1 <= 4,
        "release 1 must send at most tx_limit=4 TX frames, sent {tx_r1}"
    );
    assert!(
        b1.tx_used() <= 4,
        "ledger tx_used must not exceed tx_limit=4 (got {})",
        b1.tx_used()
    );
    assert!(
        !all_motors_enabled(&bus),
        "7-motor enable cannot finish in a single tx_limit=4 release"
    );

    // Subsequent eligible releases resume and complete TorqueOn while every single release
    // respects the tx_limit=4 ledger cap.
    let mut terminal_completed = false;
    let mut seen_replies = Vec::new();
    for _ in 0..25 {
        clock.advance(Duration::from_millis(10));
        bus.with(|s| {
            s.tx_log.clear();
            s.push_all_feedback();
        });
        let _rep = fix.step_with_clock(|| clock.now());
        let tx_this = bus.with(|s| s.tx_log.len());
        let b = fix.last_release_budget().expect("release ledger");
        assert!(
            tx_this <= 4,
            "every release must respect tx_limit=4, sent {tx_this} (ledger={b:?})"
        );
        broker.poll(|_, r| {
            if matches!(r, Reply::Terminal { outcome: Outcome::Completed, .. }) {
                terminal_completed = true;
            }
            seen_replies.push(r);
            true
        });
        if terminal_completed && all_motors_enabled(&bus) {
            break;
        }
    }
    assert!(
        terminal_completed && all_motors_enabled(&bus),
        "TorqueOn must resume and complete across tight-TX releases (completed={terminal_completed}, enabled={}, replies={seen_replies:?}, last_action={:?}, safety={:?})",
        all_motors_enabled(&bus),
        fix.core.last_action,
        fix.core.safety.state(),
    );
}

#[test]
fn sub_budget_only_tightens_and_deadline_exhaustion_returns_pending_then_resumes() {
    let clock = MockClock::new();
    let (bus, mut drv, _cfg) = make_bus_and_driver(Some(clock.clone()), 16, true);
    drv.set_torque(true).expect("enable torque");

    // 1. Direct SubBudget monotonic tightening & parent debit check.
    let t0 = clock.now();
    let mut parent = ReleaseBudget::new(1, t0, t0 + Duration::from_millis(10), 12, 5);
    parent.charge_rx(7, false); // 5 remaining
    parent.charge_tx(2); // 3 remaining
    {
        // Attempt to widen deadline, RX, and TX -> must clamp strictly to parent remaining.
        let mut sub = parent.sub_budget(t0 + Duration::from_secs(10), 100, 100);
        assert_eq!(sub.deadline(), t0 + Duration::from_millis(10));
        assert_eq!(sub.rx_remaining(), 5);
        assert_eq!(sub.tx_remaining(), 3);
        assert!(sub.try_charge_rx_one(t0));
        assert!(sub.try_charge_tx_one(t0));
        sub.charge_rx(2, false);
        sub.charge_tx(1);
    }
    assert_eq!(parent.rx_used(), 10);
    assert_eq!(parent.rx_remaining(), 2);
    assert_eq!(parent.tx_used(), 4);
    assert_eq!(parent.tx_remaining(), 1);

    // 2. ModeSwitch::step_with_gate_and_stop_proof_ledger with an already-expired deadline:
    //    0 bus I/O, returns Pending.
    let plan = ModeSwitchPlan::confirm_gripper(6, ControlMode::PositionForce).unwrap();
    let mut ms = ModeSwitch::new(plan);
    drv.begin_control_release(10);
    bus.with(|s| {
        s.immediate_replies = false;
        s.tx_log.clear();
        s.rx.clear();
        for i in 0..10u8 {
            s.rx.push_back(stray_frame(i));
        }
    });
    let mut allow_tx = || Ok(());
    let no_stop = || false;
    let expired_at = clock.now();
    let mut expired_ledger = ReleaseBudget::new(10, expired_at, expired_at, 64, 16);
    let ms_expired = ModeSwitchBudget::from_ledger(
        &expired_ledger,
        expired_at,
        expired_at,
        Duration::from_millis(20),
        16,
    );
    let p_expired = ms.step_with_gate_and_stop_proof_ledger(
        &mut drv,
        &mut allow_tx,
        &no_stop,
        ms_expired,
        &mut expired_ledger,
    );
    assert!(
        matches!(p_expired, ModeProgress::Pending { .. }),
        "expired deadline must return Pending, got {p_expired:?}"
    );
    assert_eq!(
        bus.with(|s| s.tx_log.len()),
        0,
        "expired deadline must emit 0 TX frames"
    );
    assert_eq!(
        bus.with(|s| s.rx.len()),
        10,
        "expired deadline must consume 0 RX frames"
    );

    // 3. ModeSwitch under tight remaining RX (4 frames left in parent ledger):
    //    even if ModeSwitchBudget asks for 64 RX frames, sub-budget tightens to parent's 4 frames,
    //    consumes exactly 4 frames from the transport, charges parent, stays Pending.
    clock.advance(Duration::from_millis(20));
    drv.begin_control_release(11);
    let t1 = clock.now();
    let mut tight_ledger = ReleaseBudget::new(11, t1, t1 + Duration::from_millis(10), 64, 16);
    tight_ledger.charge_rx(60, false); // only 4 RX frames left in parent ledger
    drv.sync_release_budget_limits(tight_ledger.rx_remaining(), tight_ledger.tx_remaining());
    let mut ms_wide = ModeSwitchBudget::from_ledger(
        &tight_ledger,
        t1,
        t1 + Duration::from_millis(10),
        Duration::from_millis(20),
        16,
    );
    ms_wide.max_rx_frames = 64; // attempt to widen -> sub-budget must tighten to 4
    let p_tight = ms.step_with_gate_and_stop_proof_ledger(
        &mut drv,
        &mut allow_tx,
        &no_stop,
        ms_wide,
        &mut tight_ledger,
    );
    assert!(
        matches!(p_tight, ModeProgress::Pending { .. }),
        "tight remaining RX with 10 stray frames must stay Pending, got {p_tight:?}"
    );
    assert_eq!(
        bus.with(|s| s.rx.len()),
        6,
        "ModeSwitch must consume at most parent's remaining 4 RX frames"
    );
    assert_eq!(tight_ledger.rx_used(), 64);
    assert!(tight_ledger.rx_exhausted());

    // 4. Next eligible releases with clean RX: ModeSwitch resumes and confirms!
    bus.with(|s| {
        s.immediate_replies = true;
        s.rx.clear();
        s.push_all_feedback();
    });
    let mut confirmed = false;
    for seq in 12..25u64 {
        clock.advance(Duration::from_millis(20));
        drv.begin_control_release(seq);
        let t = clock.now();
        let mut ledger = ReleaseBudget::new(seq, t, t + Duration::from_millis(10), 64, 16);
        drv.sync_release_budget_limits(ledger.rx_remaining(), ledger.tx_remaining());
        let ms_b = ModeSwitchBudget::from_ledger(
            &ledger,
            t,
            t + Duration::from_millis(10),
            Duration::from_millis(20),
            16,
        );
        let p = ms.step_with_gate_and_stop_proof_ledger(
            &mut drv,
            &mut allow_tx,
            &no_stop,
            ms_b,
            &mut ledger,
        );
        if p == ModeProgress::Confirmed {
            confirmed = true;
            break;
        }
    }
    assert!(
        confirmed,
        "ModeSwitch must resume and confirm on subsequent eligible releases"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_parity_under_tight_release_budget() {
    async fn run_owner(use_executor: bool) -> (bool, ModeProgress, usize) {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.control_frequency_hz = 50;
        let (bus, drv, _) = make_bus_and_driver(None, 8, true);
        let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
        sup.set_lease_enabled(false).await;
        let exec = if use_executor {
            sup.register_ws_connection(77).await.unwrap();
            let h = sup
                .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
                .await
                .unwrap();
            // Wait for initial observation publication, then submit TorqueOn via broker.
            let dl_obs = Instant::now() + Duration::from_secs(2);
            while sup.hub().latest().is_none() {
                assert!(Instant::now() < dl_obs, "executor never published initial obs");
                tokio::time::sleep(Duration::from_millis(2)).await;
            }
            let auth = Authority {
                lease_generation: 0,
                epoch: sup.hub().latest().unwrap().obs.epoch_id,
                stop_generation: h.stop_requests().current(),
                config_revision: 0,
                parent_generation: 0,
            };
            h.broker()
                .lock()
                .unwrap()
                .submit(
                    ClientId(77),
                    1,
                    auth,
                    Instant::now() + Duration::from_secs(3),
                    Submit::Transition(TransitionOp::TorqueOn),
                )
                .unwrap();
            let dl_en = Instant::now() + Duration::from_secs(3);
            while sup.hub().latest().map(|s| s.obs.enable_state) != Some(EnableState::Enabled) {
                assert!(Instant::now() < dl_en, "executor TorqueOn did not complete");
                bus.with(|s| s.push_all_feedback());
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
            Some(h)
        } else {
            sup.enable_torque(sup.capture_admission_stamp())
                .await
                .expect("legacy enable torque");
            None
        };

        // Run a J7 PositionForce confirm session under tight driver TX budget (8 frames).
        let plan = ModeSwitchPlan::confirm_gripper(6, ControlMode::PositionForce).unwrap();
        let sid = sup
            .start_mode_switch_session(ModeSwitchStartRequest {
                plan,
                admission: sup.capture_admission_stamp(),
                operator_required: false,
                operator_grant: OperatorGrant::unenforced(),
                deadline: Instant::now() + Duration::from_secs(3),
                client: ClientId(77),
            })
            .await
            .unwrap();

        let mut max_tx_per_tick = 0usize;
        let mut final_progress = ModeProgress::Pending { wake: None };
        let dl = Instant::now() + Duration::from_secs(3);
        while Instant::now() < dl {
            bus.with(|s| {
                s.push_all_feedback();
            });
            if !use_executor {
                bus.with(|s| s.tx_log.clear());
                sup.control_tick(0.02).await;
                let n = bus.with(|s| s.tx_log.len());
                max_tx_per_tick = max_tx_per_tick.max(n);
            } else {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
            if let Some(snap) = sup.poll_mode_switch_session(sid).await.unwrap() {
                final_progress = snap.progress;
                if !snap.running {
                    break;
                }
            }
        }

        let motors_enabled = all_motors_enabled(&bus);
        if let Some(h) = exec {
            tokio::task::spawn_blocking(move || h.stop_and_join())
                .await
                .unwrap();
        }
        (motors_enabled, final_progress, max_tx_per_tick)
    }

    let (leg_en, leg_prog, leg_max_tx) = run_owner(false).await;
    let (exec_en, exec_prog, _) = run_owner(true).await;

    assert!(leg_en, "legacy owner must reach all motors enabled");
    assert!(exec_en, "executor owner must reach all motors enabled");
    assert_eq!(leg_prog, ModeProgress::Confirmed);
    assert_eq!(exec_prog, ModeProgress::Confirmed);
    assert!(
        leg_max_tx <= 8,
        "legacy owner must stay within tight TX budget of 8 frames/tick (saw {leg_max_tx})"
    );
}
