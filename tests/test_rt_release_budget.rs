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
        s.tx_times.clear();
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
    drv.set_tx_release_deadline(Some(t1 + Duration::from_millis(10)));
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
    drv.set_tx_release_deadline(None);
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
    use std::collections::HashMap;
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::sync::Mutex;

    async fn run_owner(use_executor: bool) -> (bool, ModeProgress, usize) {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.control_frequency_hz = 50;
        let (bus, drv, _) = make_bus_and_driver(None, 8, true);
        let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
        sup.set_lease_enabled(false).await;

        // Track every transport TX attempt (including transition wakes and any backpressured
        // attempts) bucketed by the exact release index (Window 1 + Window 2 of each release).
        let legacy_release_seq = Arc::new(AtomicU64::new(0));
        let per_release_attempts: Arc<Mutex<HashMap<u64, usize>>> =
            Arc::new(Mutex::new(HashMap::new()));

        let exec = if use_executor {
            sup.register_ws_connection(77).await.unwrap();
            let h = sup
                .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
                .await
                .unwrap();
            let stats = Arc::clone(h.stats());
            let map_c = Arc::clone(&per_release_attempts);
            bus.with(|s| {
                s.on_tx = Some(Arc::new(move || {
                    let rel = stats.steps.load(Ordering::Acquire);
                    if rel > 0 {
                        *map_c.lock().unwrap().entry(rel).or_insert(0) += 1;
                    }
                }));
            });
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
            let seq_c = Arc::clone(&legacy_release_seq);
            let map_c = Arc::clone(&per_release_attempts);
            bus.with(|s| {
                s.on_tx = Some(Arc::new(move || {
                    let rel = seq_c.load(Ordering::Acquire);
                    if rel > 0 {
                        *map_c.lock().unwrap().entry(rel).or_insert(0) += 1;
                    }
                }));
            });
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

        let mut final_progress = ModeProgress::Pending { wake: None };
        let dl = Instant::now() + Duration::from_secs(3);
        while Instant::now() < dl {
            bus.with(|s| {
                s.push_all_feedback();
            });
            if !use_executor {
                legacy_release_seq.fetch_add(1, Ordering::AcqRel);
                sup.control_tick(0.02).await;
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

        // Detach TX observer before `stop_and_join`, which transitions an energized executor into
        // SAFE-HOLD (where `stats.steps` stops advancing while SAFE-HOLD runs its own release loop).
        bus.with(|s| s.on_tx = None);
        let motors_enabled = all_motors_enabled(&bus);
        if let Some(h) = exec {
            tokio::task::spawn_blocking(move || h.stop_and_join())
                .await
                .unwrap();
        }
        let max_attempts = per_release_attempts
            .lock()
            .unwrap()
            .values()
            .copied()
            .max()
            .unwrap_or(0);
        (motors_enabled, final_progress, max_attempts)
    }

    let (leg_en, leg_prog, leg_max_tx) = run_owner(false).await;
    let (exec_en, exec_prog, exec_max_tx) = run_owner(true).await;

    assert!(leg_en, "legacy owner must reach all motors enabled");
    assert!(exec_en, "executor owner must reach all motors enabled");
    assert_eq!(leg_prog, ModeProgress::Confirmed);
    assert_eq!(exec_prog, ModeProgress::Confirmed);
    assert!(
        leg_max_tx > 0 && leg_max_tx <= 8,
        "legacy owner must attempt >0 and <=8 TX frames per release (saw {leg_max_tx})"
    );
    assert!(
        exec_max_tx > 0 && exec_max_tx <= 8,
        "executor owner must attempt >0 and <=8 TX frames per release (saw {exec_max_tx})"
    );
}

#[test]
fn armed_mit_fence_watchdog_continuity_never_deferred_across_consecutive_tight_releases() {
    use armd::driver::{JointMode, JointTarget, TxBatchOutcome};

    let clock = MockClock::new();
    let (bus, mut drv, _cfg) = make_bus_and_driver(Some(clock.clone()), 16, true);
    bus.with(|s| {
        for m in s.motors.iter_mut().take(6) {
            m.set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_MIT);
        }
    });
    drv.set_torque(true).expect("enable torque");

    let targets = vec![
        JointTarget {
            position: 0.0,
            velocity: Some(0.0),
            kp: Some(10.0),
            kd: Some(1.0),
            torque: Some(0.0),
        };
        7
    ];
    let modes = [
        JointMode::Mit,
        JointMode::Mit,
        JointMode::Mit,
        JointMode::Mit,
        JointMode::Mit,
        JointMode::Mit,
        JointMode::Position,
    ];
    drv.set_joint_modes(&modes, &targets)
        .expect("set MIT modes");
    // 41 ms MIT deadline fence at 50 Hz (20 ms releases): even ONE skipped release produces
    // a 40 ms nominal gap + TX jitter and trips the fence; consecutive skipped releases ALWAYS trip.
    drv.set_mit_deadline_fence(Some(Duration::from_millis(41)), Duration::from_millis(300));

    // Capture ACTUAL per-axis TX timestamps from `FakeBusState::tx_times` (recorded on every
    // transmitted arm frame `id` 1..=6) rather than measuring release-start timestamps.
    let mut last_axis_tx: [Option<Instant>; 6] = [None; 6];
    let mut max_axis_gap: [Duration; 6] = [Duration::ZERO; 6];
    let record_release_frames = |release_id: u64,
                                 last: &mut [Option<Instant>; 6],
                                 max_gap: &mut [Duration; 6]| {
        let mut seen_mask = 0u8;
        let mut release_axis_ts: [Option<Instant>; 6] = [None; 6];
        bus.with(|s| {
            for (f, ts) in s.tx_log.iter().zip(s.tx_times.iter()) {
                if (1..=6).contains(&f.id) {
                    let axis = (f.id - 1) as usize;
                    seen_mask |= 1u8 << axis;
                    if let Some(t) = *ts {
                        // Use the latest (successful) transmit timestamp for this axis in this release.
                        release_axis_ts[axis] = Some(t);
                    }
                }
            }
            s.tx_log.clear();
            s.tx_times.clear();
        });
        assert_eq!(
            seen_mask, 0x3F,
            "release {release_id}: every armed MIT axis (J1..J6) must receive a continuity frame (mask=0x{seen_mask:02x})"
        );
        for axis in 0..6 {
            let ts = release_axis_ts[axis]
                .unwrap_or_else(|| panic!("release {release_id}: missing TX timestamp for axis {}", axis + 1));
            if let Some(prev) = last[axis] {
                let gap = ts.saturating_duration_since(prev);
                if gap > max_gap[axis] {
                    max_gap[axis] = gap;
                }
            }
            last[axis] = Some(ts);
        }
    };

    // Release 1 (t = 0 ms): full cap establishes initial armed MIT batch.
    bus.clear_log();
    bus.with(|s| {
        s.push_all_feedback();
    });
    let t1 = clock.now();
    drv.begin_control_release(1);
    drv.set_tx_release_deadline(Some(t1 + Duration::from_millis(10)));
    drv.sync_release_budget_limits(64, 16);
    assert_eq!(
        drv.write_joint_targets_bounded(&targets).unwrap(),
        TxBatchOutcome::Complete
    );
    drv.set_tx_release_deadline(None);
    assert!(drv.mit_fence_trip().is_none());
    record_release_frames(1, &mut last_axis_tx, &mut max_axis_gap);

    // Consecutive 50 Hz releases (release_id = 2, 3, 4) with tightened ledger caps (3, 0, 2):
    // watchdog-continuity frames are NEVER deferrable by the ledger. Each release must send
    // valid holds for all 6 armed axes (plus J7) without tripping the fence, keeping every
    // actual per-axis TX gap <= 40 ms.
    for (release_id, tight_cap) in [(2u64, 3u16), (3, 0), (4, 2)] {
        clock.advance(Duration::from_millis(20));
        bus.with(|s| {
            s.tx_log.clear();
            s.push_all_feedback();
        });
        let _ = drv.read_joint_states();
        let tr = clock.now();
        drv.begin_control_release(release_id);
        drv.set_tx_release_deadline(Some(tr + Duration::from_millis(10)));
        drv.sync_release_budget_limits(64, tight_cap);

        let r_target = drv.write_joint_targets_bounded(&targets).unwrap();
        assert!(
            matches!(r_target, TxBatchOutcome::Deferred { pending } if pending >= 6),
            "release {release_id}: tightened ledger defers motion target while sending holds, got {r_target:?}"
        );
        assert!(
            drv.mit_fence_trip().is_none(),
            "release {release_id}: tightened ledger must not trip MIT fence: {:?}",
            drv.mit_fence_trip()
        );
        drv.set_tx_release_deadline(None);
        record_release_frames(release_id, &mut last_axis_tx, &mut max_axis_gap);
    }

    // Release 5 (t = 80 ms): stopped-hold and measured-hold paths under tightened cap also send
    // complete continuity holds for all armed axes without tripping the fence.
    clock.advance(Duration::from_millis(20));
    bus.with(|s| {
        s.tx_log.clear();
        s.tx_times.clear();
        s.push_all_feedback();
    });
    let _ = drv.read_joint_states();
    let t5 = clock.now();
    drv.begin_control_release(5);
    drv.set_tx_release_deadline(Some(t5 + Duration::from_millis(10)));
    drv.sync_release_budget_limits(64, 1);
    let r_mhold = drv.write_measured_hold_masked(0x7F, &targets).unwrap();
    assert_eq!(r_mhold.outcome, TxBatchOutcome::Complete);
    assert!(r_mhold.sent_any);
    assert!(drv.mit_fence_trip().is_none());
    drv.set_tx_release_deadline(None);
    record_release_frames(5, &mut last_axis_tx, &mut max_axis_gap);

    // Release 6 (t = 100 ms): Step 5 skipped -> `service_release_watchdog_fallback` with a tight
    // 2.5 ms release cutoff and 1 ms/frame TX step (mid-batch cutoff expiry + one-shot WouldBlock)
    // must still complete ALL 6 armed MIT axes and update fence timestamps without tripping.
    clock.advance(Duration::from_millis(20));
    bus.with(|s| {
        s.tx_log.clear();
        s.push_all_feedback();
        s.tx_step = Duration::from_millis(1);
        s.fail_tx_once_after = Some(2);
    });
    let _ = drv.read_joint_states();
    let t6 = clock.now();
    drv.begin_control_release(6);
    drv.set_tx_release_deadline(Some(t6 + Duration::from_micros(2_500)));
    drv.sync_release_budget_limits(64, 0);
    drv.service_release_watchdog_fallback(0x3F, None);
    let _ = drv.emit_pos_force_j7(
        armd::driver::PosForceIntent::hold(armd::driver::PosForceIntentStamp::default()),
        armd::driver::PosForceIntentStamp::default(),
    );
    assert!(
        drv.mit_fence_trip().is_none(),
        "generic watchdog fallback under mid-batch cutoff expiry + one-shot WouldBlock must not trip fence: {:?}",
        drv.mit_fence_trip()
    );
    drv.set_tx_release_deadline(None);
    record_release_frames(6, &mut last_axis_tx, &mut max_axis_gap);

    // Release 7 (t = 120 ms): partial batch mid-send deadline expiry (release deadline expires
    // after 2 frames) must fall back to holds for the uncovered axes without tripping the fence.
    clock.advance(Duration::from_millis(15));
    bus.with(|s| {
        s.tx_log.clear();
        s.push_all_feedback();
        s.tx_step = Duration::from_millis(1);
    });
    let _ = drv.read_joint_states();
    let t7 = clock.now();
    drv.begin_control_release(7);
    // 2.5 ms release deadline with 1 ms/frame -> frames 0,1,2 transmit, frame 3 crosses release deadline.
    drv.set_tx_release_deadline(Some(t7 + Duration::from_micros(2_500)));
    drv.sync_release_budget_limits(64, 16);
    let r_partial_dl = drv.write_joint_targets_bounded(&targets).unwrap();
    assert!(
        matches!(r_partial_dl, TxBatchOutcome::Deferred { .. }),
        "partial deadline fallback should report deferred targets while covering all axes: {r_partial_dl:?}"
    );
    assert!(
        drv.mit_fence_trip().is_none(),
        "mid-batch release deadline expiry must not leave partial batch or trip fence: {:?}",
        drv.mit_fence_trip()
    );
    drv.set_tx_release_deadline(None);
    record_release_frames(7, &mut last_axis_tx, &mut max_axis_gap);

    // Release 8 (t = 140 ms): exact physical cap (`tx_budget.max_frames == 7`, matching the
    // 7-frame batch size) with a transient one-shot transport error after 2 sends must still
    // recover via bounded recovery capacity and cover all axes without tripping the fence.
    let mut exact_cap_budget = TxBudget::default();
    exact_cap_budget.max_frames = 7;
    drv.set_tx_budget(exact_cap_budget);
    clock.advance(Duration::from_millis(15));
    bus.with(|s| {
        s.tx_log.clear();
        s.tx_times.clear();
        s.push_all_feedback();
        s.tx_step = Duration::from_micros(200);
        s.fail_tx_once_after = Some(2);
    });
    let _ = drv.read_joint_states();
    let t8 = clock.now();
    drv.begin_control_release(8);
    drv.set_tx_release_deadline(Some(t8 + Duration::from_millis(10)));
    drv.sync_release_budget_limits(64, 7);
    let r_partial_tx = drv.write_joint_targets_bounded(&targets).unwrap();
    assert!(
        matches!(r_partial_tx, TxBatchOutcome::Deferred { .. }),
        "exact-cap partial transport fallback should report deferred targets while covering all axes: {r_partial_tx:?}"
    );
    assert!(
        drv.mit_fence_trip().is_none(),
        "exact-cap mid-batch transient transport error must fall back to holds without tripping fence: {:?}",
        drv.mit_fence_trip()
    );
    drv.set_tx_release_deadline(None);
    record_release_frames(8, &mut last_axis_tx, &mut max_axis_gap);

    // Release 9 (t = 160 ms): full cap resumes full MIT target batch cleanly.
    let mut restored_budget = TxBudget::default();
    restored_budget.max_frames = 16;
    drv.set_tx_budget(restored_budget);
    clock.advance(Duration::from_millis(20));
    bus.with(|s| {
        s.tx_log.clear();
        s.tx_times.clear();
        s.push_all_feedback();
    });
    let _ = drv.read_joint_states();
    let t9 = clock.now();
    drv.begin_control_release(9);
    drv.set_tx_release_deadline(Some(t9 + Duration::from_millis(10)));
    drv.sync_release_budget_limits(64, 16);
    assert_eq!(
        drv.write_joint_targets_bounded(&targets).unwrap(),
        TxBatchOutcome::Complete,
        "armed MIT target batch must resume and complete on the next eligible release"
    );
    drv.set_tx_release_deadline(None);
    assert!(drv.mit_fence_trip().is_none());
    record_release_frames(9, &mut last_axis_tx, &mut max_axis_gap);

    bus.with(|s| s.on_tx = None);
    for (axis, gap) in max_axis_gap.iter().enumerate() {
        assert!(
            *gap <= Duration::from_millis(40),
            "axis {} exceeded 40 ms actual TX gap: {:?}",
            axis + 1,
            gap
        );
    }
}

#[test]
fn armed_mit_continuity_hold_rejects_pending_target_torque_and_j7_motion_past_cutoff() {
    use armd::driver::{
        ArmDriver, JointMode, JointTarget, PosForceEmitOutcome, PosForceIntent,
        PosForceIntentKind, TxBatchOutcome,
    };

    let clock = MockClock::new();
    let (bus, mut drv, _cfg) = make_bus_and_driver(Some(clock.clone()), 16, true);
    bus.with(|s| {
        for m in s.motors.iter_mut().take(6) {
            m.set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_MIT);
        }
    });
    drv.set_torque(true).expect("enable torque");

    // Release 1: establish authorized MIT parameters at position 0.0, torque 0.0.
    let mut base_targets = vec![JointTarget::position(0.0); 7];
    for t in base_targets.iter_mut().take(6) {
        t.kp = Some(12.0);
        t.kd = Some(0.8);
        t.torque = Some(0.0);
    }
    let modes = [
        JointMode::Mit,
        JointMode::Mit,
        JointMode::Mit,
        JointMode::Mit,
        JointMode::Mit,
        JointMode::Mit,
        JointMode::Position,
    ];
    drv.set_joint_modes(&modes, &base_targets)
        .expect("set MIT modes");
    drv.set_mit_deadline_fence(Some(Duration::from_millis(45)), Duration::from_millis(300));
    drv.set_external_pos_force_emitter(true);
    bus.with(|s| {
        s.tx_log.clear();
        s.tx_times.clear();
        s.push_all_feedback();
    });
    let _ = drv.read_joint_states();
    let t1 = clock.now();
    drv.begin_control_release(1);
    drv.set_tx_release_deadline(Some(t1 + Duration::from_millis(10)));
    drv.sync_release_budget_limits(64, 16);
    assert_eq!(
        drv.write_joint_targets_bounded(&base_targets).unwrap(),
        TxBatchOutcome::Complete
    );
    drv.set_tx_release_deadline(None);

    // Release 2 (Finding 2): pending motion targets carry large feedforward torque (4.5 Nm)
    // and displaced position (0.45 rad), while ledger tightens TX cap below the 6-frame arm batch
    // so continuity holds run. Every emitted MIT continuity frame must keep torque ~ 0.0 Nm and
    // position ~ 0.0 rad (never copying pending target position or feedforward torque).
    clock.advance(Duration::from_millis(20));
    bus.with(|s| {
        s.tx_log.clear();
        s.tx_times.clear();
        s.push_all_feedback();
    });
    let _ = drv.read_joint_states();
    let mut aggressive_targets = vec![JointTarget::position(0.45); 7];
    for t in aggressive_targets.iter_mut().take(6) {
        t.kp = Some(40.0);
        t.kd = Some(2.0);
        t.torque = Some(4.5);
    }
    let t2 = clock.now();
    drv.begin_control_release(2);
    drv.set_tx_release_deadline(Some(t2 + Duration::from_millis(10)));
    drv.sync_release_budget_limits(64, 3);
    let r2 = drv.write_joint_targets_bounded(&aggressive_targets).unwrap();
    assert!(matches!(r2, TxBatchOutcome::Deferred { .. }));
    assert!(drv.mit_fence_trip().is_none());
    drv.set_tx_release_deadline(None);
    let r2_frames = bus.with(|s| s.tx_log.clone());
    assert_eq!(r2_frames.len(), 6, "all 6 MIT axes must receive continuity holds");
    for frame in &r2_frames {
        let p_raw = ((frame.data[0] as u16) << 8) | (frame.data[1] as u16);
        let t_raw = (((frame.data[6] as u16) & 0x0F) << 8) | (frame.data[7] as u16);
        assert!(
            (i32::from(p_raw) - 0x8000).abs() <= 4,
            "continuity hold for id=0x{:x} must hold ~0.0 rad, not pending target 0.45 rad (p_raw=0x{p_raw:04x})",
            frame.id
        );
        assert!(
            (i32::from(t_raw) - 0x800).abs() <= 2,
            "continuity hold for id=0x{:x} must carry ~0.0 Nm torque, not pending target 4.5 Nm (t_raw=0x{t_raw:03x})",
            frame.id
        );
    }

    // Release 3 (Finding 1 + Finding 3):
    // - Zero-send cutoff: `tx_release_deadline` is already reached on entry (while `pass.deadline`
    //   is still open); arm write must fall back to continuity holds without tripping the fence.
    // - J7 motion target past cutoff: submitting `PosForceIntentKind::Target` with `velocity > 0`
    //   after `tx_release_deadline` must degrade to stationary Hold (`vel_raw == 0`), never
    //   transmitting a J7 motion frame past cutoff.
    clock.advance(Duration::from_millis(20));
    bus.with(|s| {
        s.tx_log.clear();
        s.tx_times.clear();
        s.push_all_feedback();
    });
    let _ = drv.read_joint_states();
    let t3 = clock.now();
    drv.begin_control_release(3);
    drv.set_tx_release_deadline(Some(t3)); // already at cutoff before first send
    drv.sync_release_budget_limits(64, 16);
    let r3 = drv.write_joint_targets_bounded(&aggressive_targets).unwrap();
    assert!(
        matches!(r3, TxBatchOutcome::Deferred { .. }),
        "zero-send release cutoff must defer targets and emit continuity holds: {r3:?}"
    );
    assert!(
        drv.mit_fence_trip().is_none(),
        "zero-send release cutoff must not trip armed MIT fence: {:?}",
        drv.mit_fence_trip()
    );
    let stamp = drv.current_j7_intent_stamp();
    let j7_intent = PosForceIntent {
        kind: PosForceIntentKind::Target {
            position: 0.35,
            velocity: Some(1.0),
            current_raw: None,
        },
        stamp,
    };
    let j7_out = drv.emit_pos_force_j7(j7_intent, stamp).unwrap();
    assert!(
        matches!(j7_out, PosForceEmitOutcome::SentHoldInsteadOfTarget { .. }),
        "J7 past cutoff under armed MIT must degrade to stationary Hold and return SentHoldInsteadOfTarget: {j7_out:?}"
    );
    drv.set_tx_release_deadline(None);
    let r3_frames = bus.with(|s| s.tx_log.clone());
    let j7_frame = r3_frames
        .iter()
        .find(|f| f.id == 0x307)
        .expect("degraded J7 Hold frame 0x307 must be emitted");
    let j7_vel_raw = u16::from_le_bytes([j7_frame.data[4], j7_frame.data[5]]);
    assert_eq!(
        j7_vel_raw, 0,
        "J7 motion target past release cutoff must degrade to stationary Hold (vel_raw=0), got {j7_vel_raw}"
    );
}

#[test]
fn tightened_zero_tx_release_does_not_leak_to_standalone_driver_tx() {
    use armd::driver::{JointTarget, TxBatchOutcome};

    let clock = MockClock::new();
    let (bus, mut drv, _cfg) = make_bus_and_driver(Some(clock.clone()), 16, true);
    drv.set_torque(true).expect("enable torque");

    // Simulate a managed release tightened to 0 TX attempts.
    let t0 = clock.now();
    drv.begin_control_release(42);
    drv.set_tx_release_deadline(Some(t0 + Duration::from_millis(10)));
    drv.sync_release_budget_limits(64, 0);
    let targets = vec![JointTarget::position(0.0); 7];
    bus.with(|s| s.tx_log.clear());
    let in_rel = drv.write_joint_targets_bounded(&targets).unwrap();
    assert!(
        matches!(in_rel, TxBatchOutcome::Deferred { .. }),
        "active release tightened to tx_limit=0 must defer TX, got {in_rel:?}"
    );
    assert_eq!(
        bus.with(|s| s.tx_log.len()),
        0,
        "active release tightened to tx_limit=0 must emit 0 CAN TX frames"
    );

    // End the managed release window (`set_tx_release_deadline(None)`):
    // standalone driver operations must immediately regain full `tx_budget.max_frames` (16),
    // never starved by stale per-release tightening.
    drv.set_tx_release_deadline(None);
    clock.advance(Duration::from_millis(20));
    bus.with(|s| {
        s.tx_log.clear();
        s.tx_times.clear();
        s.push_all_feedback();
    });
    let _ = drv.read_joint_states().unwrap();
    let standalone = drv.write_joint_targets_bounded(&targets).unwrap();
    assert_eq!(
        standalone,
        TxBatchOutcome::Complete,
        "standalone TX after a zero-TX release must not inherit stale release_tx_limit"
    );
    assert_eq!(
        bus.with(|s| s.tx_log.len()),
        7,
        "standalone TX must send all 7 motor frames"
    );
}

#[test]
fn clear_fault_maintenance_shares_release_ledger_and_begin_tx_tick_preserves_spent_attempts() {
    use armd::driver::JointTarget;
    use armd::safety::{FaultCode, FaultKind, FaultReason};

    let clock = MockClock::new();
    let (bus, mut drv, cfg) = make_bus_and_driver(Some(clock.clone()), 16, true);
    drv.set_torque(true).expect("enable torque");

    // Part 1: Direct driver proof — `begin_tx_tick` and `probe_tx_ready_budgeted` inside an
    // active managed control release MUST NOT reset TX attempts already spent in that release.
    let t0 = clock.now();
    drv.begin_control_release(101);
    drv.set_tx_release_deadline(Some(t0 + Duration::from_millis(10)));
    drv.sync_release_budget_limits(64, 4);
    let targets = vec![JointTarget::position(0.0); 7];
    bus.with(|s| {
        s.tx_log.clear();
        s.tx_times.clear();
        s.push_all_feedback();
    });
    // Spend all 4 TX slots of the release on partial output (4 frames sent, 3 deferred).
    let _ = drv.write_joint_targets_bounded(&targets).unwrap();
    let (_, tx_spent_before) = drv.release_budget_consumed();
    assert_eq!(tx_spent_before, 4, "must have spent 4 TX attempts in release 101");

    // Calling `begin_tx_tick()` inside the same active control release must NOT wipe spent attempts.
    drv.begin_tx_tick();
    let (_, tx_spent_after_begin) = drv.release_budget_consumed();
    assert_eq!(
        tx_spent_after_begin, 4,
        "begin_tx_tick in active control release must preserve already-spent TX attempts"
    );

    // Calling `probe_tx_ready_budgeted` inside the same exhausted release must see 0 remaining
    // TX and defer (emitting 0 additional CAN TX frames), never regaining spent TX.
    drv.discard_pending_tx();
    bus.with(|s| s.tx_log.clear());
    let probe_res = drv.probe_tx_ready_budgeted(t0 + Duration::from_millis(10));
    assert!(
        matches!(probe_res, Err(armd::driver::ArmDriverError::TxDeferred { .. })),
        "probe_tx_ready_budgeted at 0 remaining release TX must defer, got {probe_res:?}"
    );
    assert_eq!(
        bus.with(|s| s.tx_log.len()),
        0,
        "probe_tx_ready_budgeted must emit 0 wire TX when release TX budget is already spent"
    );
    drv.set_tx_release_deadline(None);

    // Part 2: Supervisor `clear_fault_executor_bounded_for_release_with_budget` debits the
    // shared `ReleaseBudget` ledger and refuses to transmit when TX is already exhausted.
    let (bus2, mut drv2, _) = make_bus_and_driver(Some(clock.clone()), 16, true);
    drv2.set_torque(true).expect("enable");
    bus2.with(|s| s.push_all_feedback());
    let _ = drv2.read_joint_states();
    let mut core = SupervisorInner::new(cfg).with_driver(Box::new(drv2));
    core.lease_enabled = false;
    core.rt_executor_owned = true;
    // Run one normal poll so `core` has fresh measured feedback and clear_check reaches TxProbeRequired.
    {
        let t_init = clock.now();
        let mut init_ledger = ReleaseBudget::new(199, t_init, t_init + Duration::from_millis(15), 64, 16);
        let mut init_now = || clock.now();
        let _ = core.poll_hardware_with_budget(&mut init_ledger, &mut init_now);
    }
    core.safety.trigger_fault_coded(
        FaultCode::Other,
        FaultReason::Other("injected can tx stall".into()),
    );
    core.safety.latch(
        FaultKind::CanTxStalled,
        armd::safety::FaultDetail::from_reason(
            FaultCode::Other,
            &FaultReason::Other("injected can tx stall".into()),
        ),
    );

    let t1 = clock.now();
    let mut ledger = ReleaseBudget::new(200, t1, t1 + Duration::from_millis(15), 64, 1);
    core.driver
        .as_mut()
        .unwrap()
        .try_with_mut(|d| d.begin_control_release(200))
        .unwrap();
    // Spend the single TX slot in `ledger` BEFORE ClearFault maintenance runs.
    ledger.charge_tx(1);
    assert_eq!(ledger.tx_remaining(), 0);
    bus2.with(|s| {
        s.tx_log.clear();
        s.push_all_feedback();
    });
    let mut now_fn = || clock.now();
    let res_exhausted = core.clear_fault_executor_bounded_for_release_with_budget(
        t1,
        &mut ledger,
        None,
        &mut now_fn,
    );
    assert!(
        matches!(
            res_exhausted,
            Err(armd::safety::ClearFaultError::Rejected(
                armd::safety::ClearRefusal::DriverError(_)
            ))
        ),
        "ClearFault at 0 remaining ledger TX must reach probe and defer cleanly, got {res_exhausted:?}"
    );
    assert_eq!(
        bus2.with(|s| s.tx_log.len()),
        0,
        "ClearFault at 0 remaining ledger TX must emit 0 wire frames"
    );

    // Next release with 2 remaining TX slots: ClearFault sends 1 probe frame and charges it to `ledger`.
    clock.advance(Duration::from_millis(20));
    let t2 = clock.now();
    let mut ledger2 = ReleaseBudget::new(201, t2, t2 + Duration::from_millis(15), 64, 2);
    core.driver
        .as_mut()
        .unwrap()
        .try_with_mut(|d| d.begin_control_release(201))
        .unwrap();
    bus2.with(|s| {
        s.tx_log.clear();
        s.push_all_feedback();
    });
    let res_ok = core.clear_fault_executor_bounded_for_release_with_budget(
        t2,
        &mut ledger2,
        None,
        &mut now_fn,
    );
    assert!(
        matches!(
            res_ok,
            Ok(_)
                | Err(armd::safety::ClearFaultError::Rejected(
                    armd::safety::ClearRefusal::TxProbeFeedbackRequired
                ))
        ),
        "ClearFault with remaining TX budget must send 0xCC probe (and either clear or await probe reply), got {res_ok:?}"
    );
    let sent = bus2.with(|s| s.tx_log.len());
    assert_eq!(sent, 1, "ClearFault probe should emit exactly 1 0xCC frame");
    assert_eq!(
        ledger2.tx_used(),
        1,
        "ClearFault probe TX attempt must be charged to the shared ReleaseBudget"
    );
    assert_eq!(ledger2.tx_remaining(), 1);
}

#[test]
fn zero_remaining_rx_still_runs_inline_watchdog_holds_and_protective_disable_retries() {
    use armd::driver::{DisableKind, DisableProtectiveReason, TorqueProgress};

    let clock = MockClock::new();
    let (bus, mut drv, cfg) = make_bus_and_driver(Some(clock.clone()), 16, true);
    drv.set_torque(true).expect("enable torque");

    // 1. Protective disable retries at 0 remaining RX in `poll_hardware_with_budget` and
    //    `poll_transition_wake_with_budget`: skip RX reading (0 RX consumed) while STILL
    //    transmitting `CMD_DISABLE` retry frames!
    bus.with(|s| {
        s.immediate_replies = false;
        s.rx.clear();
        for i in 0..8u8 {
            s.rx.push_back(stray_frame(i));
        }
        s.tx_log.clear();
    });
    let prog = drv
        .begin_disable_torque_budgeted(
            DisableKind::DisableProtective {
                reason: DisableProtectiveReason::Thermal,
            },
            0,
        )
        .expect("begin protective disable");
    assert!(matches!(prog, TorqueProgress::Pending { .. }));
    let mut core = SupervisorInner::new(cfg.clone()).with_driver(Box::new(drv));
    core.lease_enabled = false;
    core.rt_executor_owned = true;
    core.disable_pending = true;

    // Advance past DAMIAO_DISABLE_REPEAT_INTERVAL (20 ms) so the protective repeat pass is due.
    clock.advance(Duration::from_millis(25));
    bus.with(|s| s.tx_log.clear());
    let rx_before = bus.with(|s| s.rx.len());
    let t0 = clock.now();
    // Ledger with 0 RX remaining and positive deadline/TX:
    let mut zero_rx_ledger = ReleaseBudget::new(300, t0, t0 + Duration::from_millis(10), 0, 16);
    let mut now_fn = || clock.now();
    let poll = core.poll_hardware_with_budget(&mut zero_rx_ledger, &mut now_fn);
    assert!(poll.budget_exhausted);
    assert_eq!(
        bus.with(|s| s.rx.len()),
        rx_before,
        "0 remaining RX must skip reading RX frames"
    );
    let disable_tx_sent = bus.with(|s| {
        s.tx_log
            .iter()
            .filter(|f| f.data == damiao::CMD_DISABLE)
            .count()
    });
    assert!(
        disable_tx_sent >= 1,
        "protective disable retries must still transmit CMD_DISABLE at 0 remaining RX (sent {disable_tx_sent})"
    );

    // 2. Inline watchdog holds during Window 2 wake (`poll_transition_wake_with_budget`) at
    //    0 remaining RX: skip RX reading while STILL transmitting watchdog hold frames!
    let (bus2, mut drv2, cfg2) = make_bus_and_driver(Some(clock.clone()), 16, true);
    drv2.set_torque(true).expect("enable torque");
    let mut core2 = SupervisorInner::new(cfg2).with_driver(Box::new(drv2));
    core2.lease_enabled = false;
    core2.rt_executor_owned = true;
    // Enter pre-all disable wait (`AwaitRestGate`) while motors are energized (RX not drained),
    // then advance past WATCHDOG_DISABLE_FALLBACK_INTERVAL (30 ms) so inline watchdog holds are due.
    bus2.with(|s| {
        s.immediate_replies = false;
        s.rx.clear();
        for i in 0..8u8 {
            s.rx.push_back(stray_frame(i));
        }
    });
    core2
        .driver
        .as_mut()
        .unwrap()
        .try_with_mut(|d| {
            let mut states = Vec::new();
            let _ = d.read_joint_states_budgeted(&mut states, clock.now() + Duration::from_millis(5), 1);
            let started_at = clock.now();
            let p2 = d
                .begin_disable_torque_budgeted(DisableKind::DisableRequested { started_at }, 0)
                .expect("begin disable awaiting rest gate");
            assert!(matches!(p2, TorqueProgress::Pending { .. }));
        })
        .unwrap();
    core2.disable_pending = true;
    // Directly invoke `poll_torque_budgeted(deadline, 0)` after 35 ms elapsed -> must emit
    // inline watchdog holds for energized arm joints while consuming 0 RX frames.
    clock.advance(Duration::from_millis(35));
    bus2.with(|s| s.tx_log.clear());
    let rx2_before = bus2.with(|s| s.rx.len());
    let t1 = clock.now();
    let zero_rx_wake = ReleaseBudget::new(301, t1, t1 + Duration::from_millis(10), 0, 16);
    core2
        .driver
        .as_mut()
        .unwrap()
        .try_with_mut(|d| {
            d.begin_control_release(301);
            d.set_tx_release_deadline(Some(t1 + Duration::from_millis(8)));
            let _ = d.poll_torque_budgeted(t1 + Duration::from_millis(8), zero_rx_wake.rx_remaining());
            d.set_tx_release_deadline(None);
        })
        .unwrap();
    assert_eq!(
        bus2.with(|s| s.rx.len()),
        rx2_before,
        "0 remaining RX must consume 0 RX frames during inline watchdog hold poll"
    );
    let hold_frames = bus2.with(|s| {
        s.tx_log
            .iter()
            .filter(|f| (0x101..=0x106).contains(&f.id))
            .count()
    });
    assert_eq!(
        hold_frames, 6,
        "inline watchdog holds must refresh all 6 energized arm axes even at 0 remaining RX"
    );

    // 3. Also verify `poll_transition_wake_with_budget` does not early-return on 0 RX when
    //    protective disable retries are pending (consumes 0 RX, transmits CMD_DISABLE).
    clock.advance(Duration::from_millis(25));
    let t3 = clock.now();
    bus.with(|s| s.tx_log.clear());
    let rx_wake_before = bus.with(|s| s.rx.len());
    let mut wake_ledger = ReleaseBudget::new(303, t3, t3 + Duration::from_millis(10), 0, 16);
    let wake_poll = core.poll_transition_wake_with_budget(&mut wake_ledger, &mut || clock.now());
    assert!(wake_poll.budget_exhausted);
    assert_eq!(
        bus.with(|s| s.rx.len()),
        rx_wake_before,
        "Window 2 transition wake at 0 RX must not read RX frames"
    );
    let wake_disable_tx = bus.with(|s| {
        s.tx_log
            .iter()
            .filter(|f| f.data == damiao::CMD_DISABLE)
            .count()
    });
    assert!(
        wake_disable_tx >= 1,
        "Window 2 transition wake at 0 RX must still transmit protective CMD_DISABLE retries (sent {wake_disable_tx})"
    );
}

#[test]
fn pre_command_drain_bounded_by_release_deadline_minus_publish_margin() {
    use armd::driver::{DisableKind, TorqueProgress, DAMIAO_TX_RELEASE_PUBLISH_MARGIN};

    let clock = MockClock::new();
    let (bus, drv, cfg) = make_bus_and_driver(Some(clock.clone()), 16, true);
    let mut core = SupervisorInner::new(cfg).with_driver(Box::new(drv));
    core.lease_enabled = false;
    core.rt_executor_owned = true;
    let margin = DAMIAO_TX_RELEASE_PUBLISH_MARGIN;

    // Seed a 20-frame RX backlog with 1 ms per receive step: an unbounded 5 ms pre-command
    // drain would read 5 frames and advance the clock by 5 ms, crossing `ledger.deadline() - margin`.
    bus.with(|s| {
        s.rx_step = Duration::from_millis(1);
        s.rx.clear();
        for i in 0..20u8 {
            s.rx.push_back(stray_frame(i));
        }
    });

    // 1. Enable pre-command drain under a `(margin + 2 ms)` release ledger (cutoff = t0 + 2 ms):
    //    must stop at <= t0 + 2 ms (<= 2 frames), never running the full 5 ms drain window.
    let t0 = clock.now();
    let release_deadline = t0 + margin + Duration::from_millis(2);
    let expected_cutoff = release_deadline - margin;
    let rx_before = bus.with(|s| s.rx.len());
    let enable_res = core
        .driver
        .as_mut()
        .unwrap()
        .try_with_mut(|d| {
            d.begin_control_release(400);
            d.set_tx_release_deadline(Some(expected_cutoff));
            let res = d.begin_torque_budgeted(true, 64);
            d.set_tx_release_deadline(None);
            res
        })
        .unwrap();
    assert!(
        matches!(enable_res, Err(armd::driver::ArmDriverError::RxDrainTimeout)),
        "pre-command drain interrupted by release cutoff must return RxDrainTimeout, got {enable_res:?}"
    );
    let after_enable = clock.now();
    assert!(
        after_enable <= expected_cutoff,
        "enable pre-command drain crossed ledger deadline - publish_margin: elapsed {:?} > 2ms",
        after_enable.saturating_duration_since(t0)
    );
    let enable_consumed = rx_before - bus.with(|s| s.rx.len());
    assert_eq!(
        enable_consumed, 2,
        "enable pre-command drain must stop at 2 frames (2 ms cutoff), not 5 frames (5 ms window)"
    );

    // 2. Disable pre-command drain under a `(margin + 1 ms)` release ledger (cutoff = t1 + 1 ms):
    //    must stop at <= t1 + 1 ms (<= 1 frame) and hold in Pending (`AwaitRestGate`).
    bus.with(|s| {
        s.rx_step = Duration::ZERO;
        s.rx.clear();
        s.push_all_feedback();
    });
    core.driver
        .as_mut()
        .unwrap()
        .try_with_mut(|d| {
            d.set_tx_release_deadline(None);
            d.set_torque(true).expect("enable torque");
        })
        .unwrap();
    bus.with(|s| {
        s.rx_step = Duration::from_millis(1);
        s.immediate_replies = false;
        s.rx.clear();
        for i in 0..20u8 {
            s.rx.push_back(stray_frame(i));
        }
    });
    let t1 = clock.now();
    let disable_release_deadline = t1 + margin + Duration::from_millis(1);
    let expected_disable_cutoff = disable_release_deadline - margin;
    let rx_before_dis = bus.with(|s| s.rx.len());
    let dis_res = core
        .driver
        .as_mut()
        .unwrap()
        .try_with_mut(|d| {
            d.begin_control_release(401);
            d.set_tx_release_deadline(Some(expected_disable_cutoff));
            let res = d.begin_disable_torque_budgeted(
                DisableKind::DisableRequested { started_at: t1 },
                64,
            );
            d.set_tx_release_deadline(None);
            res
        })
        .unwrap();
    assert!(
        matches!(dis_res, Ok(TorqueProgress::Pending { .. })),
        "disable pre-command drain truncated by release cutoff must remain Pending, got {dis_res:?}"
    );
    let after_dis = clock.now();
    assert!(
        after_dis <= expected_disable_cutoff,
        "disable pre-command drain crossed ledger deadline - publish_margin: elapsed {:?} > 1ms",
        after_dis.saturating_duration_since(t1)
    );
    let dis_consumed = rx_before_dis - bus.with(|s| s.rx.len());
    assert_eq!(
        dis_consumed, 1,
        "disable pre-command drain must stop at 1 frame (1 ms cutoff), not 5 frames"
    );
}

#[test]
fn safe_hold_keeps_holding_unconfirmed_on_deferral_and_j7_only_traffic() {
    use armd::driver::{
        ArmDriver, ArmDriverError, DriverCapabilities, HoldPassReport, JointHoldStatus,
        JointTarget, TxBatchOutcome,
    };
    use armd::model::JointState;
    use armd::rt_core::safe_hold::{SafeHoldBuffers, SafeHoldControl};

    struct ScriptedHoldDriver {
        dof: usize,
        next_report: HoldPassReport,
        clock: MockClock,
    }

    impl ArmDriver for ScriptedHoldDriver {
        fn capabilities(&self) -> DriverCapabilities {
            DriverCapabilities::default()
        }
        fn set_torque(&mut self, _enabled: bool) -> Result<(), ArmDriverError> {
            Ok(())
        }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            Ok(vec![JointState::default(); self.dof])
        }
        fn read_joint_states_safe_hold(
            &mut self,
            out: &mut Vec<JointState>,
            _deadline: Instant,
            _max_rx_frames: u16,
        ) -> Result<(u16, bool), ArmDriverError> {
            out.clear();
            out.resize(self.dof, JointState::default());
            Ok((self.dof as u16, false))
        }
        fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> {
            Ok(())
        }
        fn write_measured_hold_masked(
            &mut self,
            _hold_mask: u64,
            _targets: &[JointTarget],
        ) -> Result<HoldPassReport, ArmDriverError> {
            Ok(self.next_report)
        }
        fn is_feedback_valid(&self) -> bool {
            true
        }
        fn feedback_all_within(&self, _dof: usize, _max_age: Duration) -> bool {
            true
        }
        fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
            out.clear();
            out.resize(self.dof, Some(Duration::from_millis(1)));
        }
        fn clock_now(&self) -> Instant {
            self.clock.now()
        }
    }

    let clock = MockClock::new();
    let cfg = ArmConfig::rebot_dm();
    let dof = cfg.dof;
    assert_eq!(dof, 7);
    let mut runner = SafeHoldBuffers::from_config(&cfg, Duration::from_millis(20));
    let ctrl = SafeHoldControl::new();

    let mut drv = ScriptedHoldDriver {
        dof,
        next_report: HoldPassReport::empty(dof),
        clock: clock.clone(),
    };
    let mut step_once = |drv: &mut ScriptedHoldDriver| {
        let now = clock.now();
        let mut now_fn = || clock.now();
        runner.step_driver(drv, &ctrl, now + Duration::from_millis(20), &mut now_fn);
    };

    // 1. Fully deferred arm batch (`Deferred { pending: 6 }`, `sent_any: false`):
    //    must keep `holding_unconfirmed == true`.
    let mut deferred_all = HoldPassReport::empty(dof);
    deferred_all.outcome = TxBatchOutcome::Deferred { pending: 6 };
    deferred_all.sent_any = false;
    for j in 0..6 {
        deferred_all.joint_status[j] = JointHoldStatus::Queued;
    }
    drv.next_report = deferred_all;
    step_once(&mut drv);
    assert!(
        ctrl.holding_unconfirmed(),
        "deferred arm hold batch must keep holding_unconfirmed set"
    );

    // 2. Partial deferral (4 arm axes held, 2 queued, `Deferred { pending: 2 }`):
    //    must keep `holding_unconfirmed == true`.
    let mut partial_defer = HoldPassReport::empty(dof);
    partial_defer.outcome = TxBatchOutcome::Deferred { pending: 2 };
    partial_defer.sent_any = true;
    for j in 0..4 {
        partial_defer.joint_status[j] = JointHoldStatus::Held;
    }
    partial_defer.joint_status[4] = JointHoldStatus::Queued;
    partial_defer.joint_status[5] = JointHoldStatus::Queued;
    drv.next_report = partial_defer;
    step_once(&mut drv);
    assert!(
        ctrl.holding_unconfirmed(),
        "partial deferred arm hold batch must keep holding_unconfirmed set"
    );

    // 3. J7-only traffic (`Complete`, `sent_any: true`, J7 = Held, 0 arm axes held):
    //    J7 traffic MUST NOT count toward arm-axis continuity -> `holding_unconfirmed == true`.
    let mut j7_only = HoldPassReport::empty(dof);
    j7_only.outcome = TxBatchOutcome::Complete;
    j7_only.sent_any = true;
    j7_only.joint_status[6] = JointHoldStatus::Held;
    drv.next_report = j7_only;
    step_once(&mut drv);
    assert!(
        ctrl.holding_unconfirmed(),
        "J7-only hold traffic with zero arm frames must NOT confirm arm holding"
    );

    // 4. Expired outcome (`Expired { dropped: 1, pending: 0 }`) with `sent_any: true`:
    //    must keep `holding_unconfirmed == true`.
    let mut expired = HoldPassReport::empty(dof);
    expired.outcome = TxBatchOutcome::Expired {
        dropped: 1,
        pending: 0,
    };
    expired.sent_any = true;
    for j in 0..5 {
        expired.joint_status[j] = JointHoldStatus::Held;
    }
    drv.next_report = expired;
    step_once(&mut drv);
    assert!(
        ctrl.holding_unconfirmed(),
        "expired hold batch outcome must keep holding_unconfirmed set"
    );

    // 5. Complete arm hold batch (all 6 arm axes `Held` + J7 `Held`, `Complete`):
    //    clears `holding_unconfirmed` to `false`.
    let mut complete = HoldPassReport::empty(dof);
    complete.outcome = TxBatchOutcome::Complete;
    complete.sent_any = true;
    for j in 0..7 {
        complete.joint_status[j] = JointHoldStatus::Held;
    }
    drv.next_report = complete;
    step_once(&mut drv);
    assert!(
        !ctrl.holding_unconfirmed(),
        "complete arm hold batch with fresh bus must clear holding_unconfirmed"
    );

    // 6. End-to-end with real `DamiaoArmDriver`: tightened `tx_budget.max_frames = 2` causes
    //    `write_measured_hold_masked` to return `Deferred` while J7/arm traffic is partial ->
    //    SAFE-HOLD must keep `holding_unconfirmed == true`, then clear it once budget allows
    //    all arm axes to complete.
    let (bus, mut damiao_drv, cfg2) = make_bus_and_driver(Some(clock.clone()), 2, true);
    damiao_drv.set_torque(true).expect("enable torque");
    damiao_drv.set_external_pos_force_emitter(false);
    let mut runner2 = SafeHoldBuffers::from_config(&cfg2, Duration::from_millis(20));
    let ctrl2 = SafeHoldControl::new();
    bus.with(|s| {
        s.rx.clear();
        s.push_all_feedback();
    });
    let t0 = clock.now();
    let mut now_fn2 = || clock.now();
    runner2.step_driver(&mut damiao_drv, &ctrl2, t0 + Duration::from_millis(20), &mut now_fn2);
    assert!(
        ctrl2.holding_unconfirmed(),
        "real DamiaoArmDriver with max_frames=2 defers holds and must keep holding_unconfirmed set"
    );
    let mut wide_budget = TxBudget::default();
    wide_budget.max_frames = 16;
    damiao_drv.set_tx_budget(wide_budget);
    clock.advance(Duration::from_millis(20));
    bus.with(|s| {
        s.rx.clear();
        s.push_all_feedback();
    });
    let t1 = clock.now();
    runner2.step_driver(&mut damiao_drv, &ctrl2, t1 + Duration::from_millis(20), &mut now_fn2);
    assert!(
        !ctrl2.holding_unconfirmed(),
        "real DamiaoArmDriver completing all arm holds must clear holding_unconfirmed"
    );
}

#[test]
fn j7_continuity_bypass_never_leaks_to_motion_target_after_arm_batch_cutoff() {
    use armd::driver::{
        ArmDriver, JointMode, JointTarget, PosForceEmitOutcome, PosForceIntent,
        PosForceIntentKind, TxBatchOutcome,
    };
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Arc;

    // Part A: Standalone armed MIT (`external_pos_force_emitter == false`).
    // Arm batch (J1..J6) completes before `tx_release_deadline`; cutoff expires immediately
    // after J6 submits (before J7 emission). The standalone path MUST NOT leak continuity
    // bypass flags to the J7 motion `Target`: NO J7 `Target` (`vel_raw > 0`) may be sent;
    // only a stationary `Hold` (`vel_raw == 0`) may be sent.
    {
        let clock = MockClock::new();
        let (bus, drv, _cfg) = make_bus_and_driver(Some(clock.clone()), 16, true);
        let armed_expire = Arc::new(AtomicBool::new(false));
        let hook_expire = armed_expire.clone();
        let hook_clock = clock.clone();
        let mut drv = drv.with_before_final_tx_gate_hook(Arc::new(move |frame| {
            // Expire the release cutoff right as the 6th arm frame submits (arm batch completes,
            // cutoff is expired before J7 Target is emitted).
            if frame.id == 6 && hook_expire.swap(false, Ordering::SeqCst) {
                hook_clock.advance(Duration::from_millis(12));
            }
        }));
        bus.with(|s| {
            for m in s.motors.iter_mut().take(6) {
                m.set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_MIT);
            }
        });
        drv.set_torque(true).expect("enable torque");

        let mut base_targets = vec![JointTarget::position(0.0); 7];
        for t in base_targets.iter_mut().take(6) {
            t.kp = Some(12.0);
            t.kd = Some(0.8);
            t.torque = Some(0.0);
        }
        let modes = [
            JointMode::Mit,
            JointMode::Mit,
            JointMode::Mit,
            JointMode::Mit,
            JointMode::Mit,
            JointMode::Mit,
            JointMode::Position,
        ];
        drv.set_joint_modes(&modes, &base_targets)
            .expect("set MIT modes");
        drv.set_mit_deadline_fence(Some(Duration::from_millis(45)), Duration::from_millis(300));
        drv.set_external_pos_force_emitter(false);

        clock.advance(Duration::from_millis(20));
        bus.with(|s| {
            s.tx_log.clear();
            s.tx_times.clear();
            s.push_all_feedback();
        });
        let _ = drv.read_joint_states();

        let mut motion_targets = base_targets.clone();
        motion_targets[6].position = 0.35;
        motion_targets[6].velocity = Some(1.0);

        let t0 = clock.now();
        drv.begin_control_release(1);
        drv.set_tx_release_deadline(Some(t0 + Duration::from_millis(10)));
        drv.sync_release_budget_limits(64, 16);
        armed_expire.store(true, Ordering::SeqCst);

        let _ = drv
            .write_joint_targets_bounded(&motion_targets)
            .expect("standalone armed MIT write must not trip fence when J7 degrades to Hold");
        drv.set_tx_release_deadline(None);
        assert!(
            drv.mit_fence_trip().is_none(),
            "degrading J7 Target to stationary Hold past cutoff must not trip armed MIT fence"
        );

        let tx_frames = bus.with(|s| s.tx_log.clone());
        let arm_count = tx_frames.iter().filter(|f| (1..=6).contains(&f.id)).count();
        assert_eq!(arm_count, 6, "arm batch J1..J6 must have completed before cutoff");
        let j7_frames: Vec<_> = tx_frames.iter().filter(|f| f.id == 0x307).collect();
        assert_eq!(j7_frames.len(), 1, "exactly one J7 0x307 frame must be emitted");
        let vel_raw = u16::from_le_bytes([j7_frames[0].data[4], j7_frames[0].data[5]]);
        assert_eq!(
            vel_raw, 0,
            "standalone J7 motion Target past cutoff must NOT be sent; only Hold (vel_raw=0) allowed, got vel_raw={vel_raw}"
        );
    }

    // Part B: External emitter (`external_pos_force_emitter == true`), armed MIT.
    // Arm batch completes before cutoff; cutoff expires inside `before_final_tx_gate_hook`
    // immediately before final CAN submission of the J7 Target frame (after pre-build checks).
    // The emitter must pass the explicit deadline and check it immediately before `can.transmit`:
    // NO J7 Target (`vel_raw > 0`) may reach the wire; only a stationary Hold (`vel_raw == 0`) may.
    {
        let clock = MockClock::new();
        let (bus, drv, _cfg) = make_bus_and_driver(Some(clock.clone()), 16, true);
        let fire_on_j7_target = Arc::new(AtomicBool::new(false));
        let hook_fire = fire_on_j7_target.clone();
        let hook_clock = clock.clone();
        let mut drv = drv.with_before_final_tx_gate_hook(Arc::new(move |frame| {
            if frame.id == 0x307 {
                let vel_raw = u16::from_le_bytes([frame.data[4], frame.data[5]]);
                if vel_raw > 0 && hook_fire.swap(false, Ordering::SeqCst) {
                    hook_clock.advance(Duration::from_millis(12));
                }
            }
        }));
        bus.with(|s| {
            for m in s.motors.iter_mut().take(6) {
                m.set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_MIT);
            }
        });
        drv.set_torque(true).expect("enable torque");

        let mut base_targets = vec![JointTarget::position(0.0); 7];
        for t in base_targets.iter_mut().take(6) {
            t.kp = Some(12.0);
            t.kd = Some(0.8);
            t.torque = Some(0.0);
        }
        let modes = [
            JointMode::Mit,
            JointMode::Mit,
            JointMode::Mit,
            JointMode::Mit,
            JointMode::Mit,
            JointMode::Mit,
            JointMode::Position,
        ];
        drv.set_joint_modes(&modes, &base_targets)
            .expect("set MIT modes");
        drv.set_mit_deadline_fence(Some(Duration::from_millis(45)), Duration::from_millis(300));
        drv.set_external_pos_force_emitter(true);

        clock.advance(Duration::from_millis(20));
        bus.with(|s| {
            s.tx_log.clear();
            s.tx_times.clear();
            s.push_all_feedback();
        });
        let _ = drv.read_joint_states();

        let t0 = clock.now();
        drv.begin_control_release(2);
        drv.set_tx_release_deadline(Some(t0 + Duration::from_millis(10)));
        drv.sync_release_budget_limits(64, 16);
        assert_eq!(
            drv.write_joint_targets_bounded(&base_targets).unwrap(),
            TxBatchOutcome::Complete
        );

        let stamp = drv.current_j7_intent_stamp();
        let j7_intent = PosForceIntent {
            kind: PosForceIntentKind::Target {
                position: 0.35,
                velocity: Some(1.0),
                current_raw: None,
            },
            stamp,
        };
        fire_on_j7_target.store(true, Ordering::SeqCst);
        let _ = drv
            .emit_pos_force_j7(j7_intent, stamp)
            .expect("external J7 emitter past cutoff under armed MIT must degrade to Hold");
        drv.set_tx_release_deadline(None);

        let j7_frames: Vec<_> = bus.with(|s| {
            s.tx_log
                .iter()
                .filter(|f| f.id == 0x307)
                .cloned()
                .collect()
        });
        assert_eq!(
            j7_frames.len(),
            1,
            "external emitter must refuse Target before can.transmit and emit only 1 Hold frame, got {} frames",
            j7_frames.len()
        );
        let vel_raw = u16::from_le_bytes([j7_frames[0].data[4], j7_frames[0].data[5]]);
        assert_eq!(
            vel_raw, 0,
            "external emitter must NOT transmit J7 Target (vel_raw > 0) once cutoff expires before CAN submit, got vel_raw={vel_raw}"
        );
    }

    // Part C: External emitter (`external_pos_force_emitter == true`), unfenced POS_VEL.
    // Cutoff expires right before final CAN submission of J7 Target -> must refuse with Deferred
    // and transmit 0 J7 frames.
    {
        let clock = MockClock::new();
        let (bus, drv, _cfg) = make_bus_and_driver(Some(clock.clone()), 16, true);
        let fire_on_j7_target = Arc::new(AtomicBool::new(false));
        let hook_fire = fire_on_j7_target.clone();
        let hook_clock = clock.clone();
        let mut drv = drv.with_before_final_tx_gate_hook(Arc::new(move |frame| {
            if frame.id == 0x307 {
                let vel_raw = u16::from_le_bytes([frame.data[4], frame.data[5]]);
                if vel_raw > 0 && hook_fire.swap(false, Ordering::SeqCst) {
                    hook_clock.advance(Duration::from_millis(12));
                }
            }
        }));
        drv.set_torque(true).expect("enable torque");
        drv.set_external_pos_force_emitter(true);

        clock.advance(Duration::from_millis(20));
        bus.with(|s| {
            s.tx_log.clear();
            s.tx_times.clear();
            s.push_all_feedback();
        });
        let _ = drv.read_joint_states();

        let targets = vec![JointTarget::position(0.0); 7];
        let t0 = clock.now();
        drv.begin_control_release(3);
        drv.set_tx_release_deadline(Some(t0 + Duration::from_millis(10)));
        drv.sync_release_budget_limits(64, 16);
        assert_eq!(
            drv.write_joint_targets_bounded(&targets).unwrap(),
            TxBatchOutcome::Complete
        );

        let stamp = drv.current_j7_intent_stamp();
        let j7_intent = PosForceIntent {
            kind: PosForceIntentKind::Target {
                position: 0.35,
                velocity: Some(1.0),
                current_raw: None,
            },
            stamp,
        };
        fire_on_j7_target.store(true, Ordering::SeqCst);
        let out = drv.emit_pos_force_j7(j7_intent, stamp).unwrap();
        drv.set_tx_release_deadline(None);
        assert!(
            matches!(out, PosForceEmitOutcome::Deferred { .. }),
            "unfenced external J7 Target past cutoff before CAN submit must return Deferred, got {out:?}"
        );
        let j7_count = bus.with(|s| s.tx_log.iter().filter(|f| f.id == 0x307).count());
        assert_eq!(
            j7_count, 0,
            "unfenced external J7 Target past cutoff must transmit 0 frames"
        );
    }
}

#[test]
fn j7_continuity_hold_fallback_does_not_count_as_accepted_motion_in_supervisor() {
    use armd::driver::{
        ArmDriver, JointMode, JointTarget, PosForceEmitOutcome, PosForceIntent,
        PosForceIntentKind,
    };
    use armd::motion::{GripperCommand, GripperRequest};
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Arc;

    // Run under BOTH owners (Legacy and RT Executor) to verify bookkeeping parity.
    for is_executor in [false, true] {
        let clock = MockClock::new();
        let (bus, drv, cfg) = make_bus_and_driver(Some(clock.clone()), 16, true);
        let expire_j7_target = Arc::new(AtomicBool::new(false));
        let hook_expire = expire_j7_target.clone();
        let hook_clock = clock.clone();
        let mut drv = drv.with_before_final_tx_gate_hook(Arc::new(move |frame| {
            if frame.id == 0x307 {
                let vel_raw = u16::from_le_bytes([frame.data[4], frame.data[5]]);
                if vel_raw > 0 && hook_expire.swap(false, Ordering::SeqCst) {
                    hook_clock.advance(Duration::from_millis(15));
                }
            }
        }));
        bus.with(|s| {
            for m in s.motors.iter_mut().take(6) {
                m.set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_MIT);
            }
            s.motors[6].q = 0.0;
            s.push_all_feedback();
        });
        drv.set_torque(true).expect("enable torque");

        let mut base_targets = vec![JointTarget::position(0.0); 7];
        for t in base_targets.iter_mut().take(6) {
            t.kp = Some(12.0);
            t.kd = Some(0.8);
            t.torque = Some(0.0);
        }
        let modes = [
            JointMode::Mit,
            JointMode::Mit,
            JointMode::Mit,
            JointMode::Mit,
            JointMode::Mit,
            JointMode::Mit,
            JointMode::Position,
        ];
        let mut core = SupervisorInner::new(cfg).with_driver(Box::new(drv));
        core.lease_enabled = false;
        core.rt_executor_owned = is_executor;
        // Enable torque through SupervisorInner so safety transitions cleanly to Armed.
        {
            let t0 = clock.now();
            let mut now_fn = || clock.now();
            core.tick_control_with_clock(0.02, &mut now_fn, t0 + Duration::from_millis(10));
        }
        {
            let s = core.current_admission_stamp();
            core.check_admission(s)
                .and_then(SupervisorInner::enable_torque)
                .expect("enable torque on supervisor");
        }
        // Switch J1..J6 to MIT mode and arm the MIT deadline fence on the driver.
        core.driver
            .as_mut()
            .unwrap()
            .try_with_mut(|d| {
                d.set_joint_modes(&modes, &base_targets)
                    .expect("set MIT modes");
                d.set_mit_deadline_fence(Some(Duration::from_millis(80)), Duration::from_millis(300));
                clock.advance(Duration::from_millis(10));
                bus.with(|s| {
                    s.tx_log.clear();
                    s.push_all_feedback();
                });
                let _ = d.read_joint_states();
                assert_eq!(
                    d.write_joint_targets_bounded(&base_targets).unwrap(),
                    armd::driver::TxBatchOutcome::Complete
                );
            })
            .unwrap();

        // Warm up 1 release so feedback timing is fresh and valid in the supervisor.
        clock.advance(Duration::from_millis(20));
        bus.with(|s| {
            s.tx_log.clear();
            s.push_all_feedback();
        });
        {
            let t0 = clock.now();
            let mut now_fn = || clock.now();
            core.tick_control_with_clock(0.02, &mut now_fn, t0 + Duration::from_millis(10));
        }

        // Start a gripper Move to current closed position (fraction = 0.0) so Release 1 finishes
        // the gripper reference immediately and produces `pending_deferred_done` while emitting a
        // J7 `PosForceIntent::Target` (with quantized wire speed `vel_raw > 0`).
        let epoch = core.safety.epoch_id();
        let stamp = core.current_admission_stamp();
        let req = GripperRequest {
            command: GripperCommand::Move,
            fraction: Some(0.0),
            grasp: false,
            speed: 1.0,
            timeout_s: Some(5.0),
            tag: Some("j7_hold_bookkeeping".into()),
        };
        let (action_id, _) = core
            .check_admission(stamp)
            .map_err(armd::ArmError::from)
            .and_then(|admitted| SupervisorInner::start_gripper(admitted, req, epoch))
            .expect("start gripper move");

        // Advance 10 pre-ticks (1 enter-settle + 9 settle ticks) cleanly so the next tick
        // (Release 1) is the exact completing release that produces `pending_deferred_done`.
        for k in 0..10 {
            clock.advance(Duration::from_millis(20));
            bus.with(|s| {
                s.tx_log.clear();
                s.push_all_feedback();
            });
            let tk = clock.now();
            let mut now_fn = || clock.now();
            core.tick_control_with_clock(0.02, &mut now_fn, tk + Duration::from_millis(10));
            assert!(
                core.last_action.as_ref().is_some_and(|a| a.is_running()),
                "owner executor={is_executor}: gripper action must still be running at settle tick {k}/10"
            );
        }

        // Release 1 (10th settle tick): cutoff expires right before J7 Target submits -> armed MIT
        // continuity fallback emits stationary Hold (`vel_raw == 0`) instead of the requested Target.
        // Supervisor MUST NOT mark the requested J7 target accepted (`last_action` stays running).
        clock.advance(Duration::from_millis(20));
        bus.with(|s| {
            s.tx_log.clear();
            s.push_all_feedback();
        });
        expire_j7_target.store(true, Ordering::SeqCst);
        {
            let t1 = clock.now();
            let mut now_fn = || clock.now();
            core.tick_control_with_clock(0.02, &mut now_fn, t1 + Duration::from_millis(10));
        }

        let r1_j7: Vec<_> = bus.with(|s| s.tx_log.iter().filter(|f| f.id == 0x307).cloned().collect());
        assert_eq!(
            r1_j7.len(),
            1,
            "owner executor={is_executor}: Release 1 must emit exactly 1 J7 continuity Hold frame"
        );
        let r1_vel_raw = u16::from_le_bytes([r1_j7[0].data[4], r1_j7[0].data[5]]);
        assert_eq!(
            r1_vel_raw, 0,
            "owner executor={is_executor}: Release 1 J7 frame must be stationary Hold (vel_raw=0), got {r1_vel_raw}"
        );
        assert!(
            !core.safety.state().is_stopped(),
            "owner executor={is_executor}: continuity Hold fallback must not fault/stop arm: {:?}",
            core.safety.state()
        );
        let a1 = core.last_action.as_ref().expect("gripper action present");
        assert_eq!(a1.id, action_id);
        assert!(
            a1.is_running(),
            "owner executor={is_executor}: J7 continuity Hold fallback MUST NOT count as accepted motion (action finished prematurely: {a1:?})"
        );

        // Release 2: cutoff stays open -> J7 Target (`vel_raw > 0`) transmits cleanly and
        // supervisor now accepts the output and completes the deferred gripper action.
        clock.advance(Duration::from_millis(20));
        bus.with(|s| {
            s.tx_log.clear();
            s.push_all_feedback();
        });
        {
            let t2 = clock.now();
            let mut now_fn = || clock.now();
            core.tick_control_with_clock(0.02, &mut now_fn, t2 + Duration::from_millis(10));
        }
        let r2_j7: Vec<_> = bus.with(|s| s.tx_log.iter().filter(|f| f.id == 0x307).cloned().collect());
        assert_eq!(
            r2_j7.len(),
            1,
            "owner executor={is_executor}: Release 2 must emit J7 Target frame"
        );
        let r2_vel_raw = u16::from_le_bytes([r2_j7[0].data[4], r2_j7[0].data[5]]);
        assert!(
            r2_vel_raw > 0,
            "owner executor={is_executor}: Release 2 J7 frame must carry motion Target (vel_raw > 0), got {r2_vel_raw}"
        );
        let a2 = core.last_action.as_ref().expect("gripper action present");
        assert_eq!(
            a2.state.as_str(),
            "done",
            "owner executor={is_executor}: Release 2 accepted J7 Target must complete deferred action: {a2:?}"
        );
    }

    // Direct driver outcome check: `emit_pos_force_j7` returns `SentHoldInsteadOfTarget`,
    // does not set J7's target-sent bit, and retains pending deferral age.
    {
        let clock = MockClock::new();
        let (bus, mut drv, _cfg) = make_bus_and_driver(Some(clock.clone()), 16, true);
        bus.with(|s| {
            for m in s.motors.iter_mut().take(6) {
                m.set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_MIT);
            }
        });
        drv.set_torque(true).expect("enable torque");
        let mut base_targets = vec![JointTarget::position(0.0); 7];
        for t in base_targets.iter_mut().take(6) {
            t.kp = Some(12.0);
            t.kd = Some(0.8);
            t.torque = Some(0.0);
        }
        let modes = [
            JointMode::Mit,
            JointMode::Mit,
            JointMode::Mit,
            JointMode::Mit,
            JointMode::Mit,
            JointMode::Mit,
            JointMode::Position,
        ];
        drv.set_joint_modes(&modes, &base_targets)
            .expect("set MIT modes");
        drv.set_mit_deadline_fence(Some(Duration::from_millis(80)), Duration::from_millis(300));
        drv.set_external_pos_force_emitter(true);

        clock.advance(Duration::from_millis(20));
        bus.with(|s| {
            s.tx_log.clear();
            s.push_all_feedback();
        });
        let _ = drv.read_joint_states();

        let t0 = clock.now();
        drv.begin_control_release(50);
        drv.set_tx_release_deadline(Some(t0)); // already expired
        drv.sync_release_budget_limits(64, 16);
        let stamp = drv.current_j7_intent_stamp();
        let out = drv
            .emit_pos_force_j7(
                PosForceIntent {
                    kind: PosForceIntentKind::Target {
                        position: -0.2,
                        velocity: Some(0.5),
                        current_raw: None,
                    },
                    stamp,
                },
                stamp,
            )
            .unwrap();
        assert!(
            matches!(out, PosForceEmitOutcome::SentHoldInsteadOfTarget { joint: 6, .. }),
            "cutoff fallback must return SentHoldInsteadOfTarget, got {out:?}"
        );
        assert_eq!(
            drv.last_target_tx_sent_mask() & (1u64 << 6),
            0,
            "J7 continuity Hold fallback must not set J7 in last_target_tx_sent_mask"
        );
        assert!(
            drv.oldest_pending_tx_age().is_some(),
            "J7 continuity Hold fallback must keep pending deferral tracked for next release"
        );
    }
}
