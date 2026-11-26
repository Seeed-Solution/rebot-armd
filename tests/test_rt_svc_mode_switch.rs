//! PR0C `ModeSwitch` service & dual-owner parity test suite (spec item 6):
//!
//! 1. FakeBus Damiao `TIMEOUT = 2000` (100 ms on J1–J6, 0 on J7) SG2002 timing emulation:
//!    - 6-axis energized switch `PositionVelocity -> Impedance -> PositionVelocity`
//!    - Verified max control-frame gap `<= 40 ms` on every arm axis, 0 `0xD` watchdog expiries
//!    - Every wire control frame matches the axis's last confirmed mode (0 unverified frames)
//!    - Zero heap allocations (`alloc == alloc_zeroed == realloc == dealloc == 0`) on RT step
//! 2. Watchdog-budget-first under tight TX budget, ACK-just-after-RX predictive abort boundary,
//!    and `require_post_write_readback` admission gating (50 Hz refusal vs 100 Hz success).
//! 3. Causal confirmation rejections: mandatory `Impedance` wire-range check, partial-switch
//!    causal rollback (`RolledBack`), failed flush after `REG_WRITE` (`Ambiguous`), duplicate
//!    write ACK (`Ambiguous`), J7 wrong reply CAN ID, and energized `ConfirmOnly` contradiction.
//! 4. PR0B session lifecycle: saturated off-RT `rt_core/stream` (authoritative poll + P-F1
//!    terminal intact), stale cancel/clear targeting replaced session, and external revocation
//!    with write in flight immediately invalidating driver verified mode.
//! 5. Dual-owner parity (`OWNER_LEGACY` vs `OWNER_EXECUTOR` identical results):
//!    - `dual_owner_j7_gripper_mode_confirmation_pos_vel_and_pos_force`
//!    - `dual_owner_energized_mode_switch_and_rollback_parity`
//!    - `dual_owner_mode_switch_authority_and_ambiguity_safe_hold_parity`

mod mit_support;

#[global_allocator]
static ALLOC: armd::diagnostics::alloc::TimedAlloc = armd::diagnostics::alloc::TimedAlloc;

use std::sync::atomic::AtomicBool;
use std::sync::Arc;
use std::time::{Duration, Instant};

use armd::auth::{OperatorGrant, OperatorRequirement};
use armd::diagnostics::alloc::{snapshot_counts, time_this_thread};
use armd::driver::{gripper_drive_from_profile, ArmDriver, DamiaoArmDriver, TxBudget};
use armd::model::ArmConfig;
use armd::motor_mode::{
    AxisTarget, ControlMode, ModeProgress, ModeSwitch, ModeSwitchFailure, ModeSwitchIntent,
    ModeSwitchPlan, ModeSwitchRecovery, ReleaseBudget,
};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::broker::ClientId;
use armd::rt_core::executor::{ExecutorConfig, ExecutorStats, PanicPolicy};
use armd::rt_core::requests::config::{ConfigError, ModeSwitchStartRequest};
use armd::rt_core::safe_hold::SafeHoldReason;
use armd::supervisor::ArmSupervisor;
use armd::supervisor_lock::ExclusiveDriver;
use armd::transport::{CanFrame, MockClock};
use mit_support::FakeBus;

const TIMEOUT_2000_COUNTS: u32 = 2000; // 100 ms at 50 us/count
const PERIOD_50HZ: Duration = Duration::from_millis(20);
const MAX_ALLOWED_GAP: Duration = Duration::from_millis(40);

fn b601_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm()
        .damiao_ranges
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect()
}

fn configure_damiao_bus(
    clock: Option<MockClock>,
    tx_step: Duration,
    j7_pos_force: bool,
    arm_timeout_counts: u32,
) -> FakeBus {
    let limits = b601_limits();
    let bus = match clock {
        Some(c) => FakeBus::with_clock(&limits, c),
        None => FakeBus::new(&limits),
    };
    let raised = [0.30, -0.25, 0.40, 0.15, -0.15, 0.10, -0.14];
    bus.with(|s| {
        s.tx_step = tx_step;
        s.poll_feeds_watchdog = false;
        s.immediate_replies = false;
        for (i, m) in s.motors.iter_mut().enumerate() {
            m.q = raised[i];
            m.v = 0.0;
            m.enabled = false;
            m.set_reg_u32(
                damiao::RID_TIMEOUT,
                if i < 6 { arm_timeout_counts } else { 0 },
            );
            m.set_reg_u32(
                damiao::RID_CTRL_MODE,
                if i == 6 && j7_pos_force {
                    damiao::CTRL_MODE_POS_FORCE
                } else {
                    damiao::CTRL_MODE_POS_VEL
                },
            );
        }
        s.push_all_feedback();
    });
    bus
}

fn build_damiao_driver(bus: FakeBus, cfg: &ArmConfig, max_frames: usize) -> DamiaoArmDriver<FakeBus> {
    let limits = b601_limits();
    let mut drv = DamiaoArmDriver::new(bus, vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap()
        .with_gripper_drive(gripper_drive_from_profile(cfg).unwrap())
        .unwrap();
    let mut b = TxBudget::default();
    b.max_frames = max_frames;
    drv.set_tx_budget(b);
    drv.set_rt_mode(true);
    let _ = drv.read_joint_states();
    drv
}

fn enable_driver_sync(drv: &mut DamiaoArmDriver<FakeBus>, bus: &FakeBus, clock: Option<&MockClock>) {
    bus.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let _ = drv.check_mapping_ranges();
    let _ = drv.read_joint_states();
    drv.set_torque(true).expect("enable torque on FakeBus");
    if let Some(c) = clock {
        c.advance(Duration::from_millis(2));
    }
    bus.with(|s| {
        s.immediate_replies = false;
        s.push_all_feedback();
    });
    let _ = drv.read_joint_states();
}

/// Computes the maximum gap between mode-matching control frames on each arm axis J1–J6 from
/// `FakeBus`'s `(tx_log, tx_times)` and asserts zero frames were ever emitted in the wrong mode.
fn audit_wire_control_gaps(
    tx_log: &[CanFrame],
    tx_times: &[Option<Instant>],
    start_at: Instant,
    end_at: Instant,
) -> ([Duration; 6], usize, usize) {
    let mut active_mode = [damiao::CTRL_MODE_POS_VEL; 6];
    let mut last_ctrl = [Some(start_at); 6];
    let mut max_gap = [Duration::ZERO; 6];
    let mut wrong_mode_frames = 0usize;
    let mut disable_frames = 0usize;

    for (frame, ts_opt) in tx_log.iter().zip(tx_times.iter()) {
        let Some(t) = *ts_opt else { continue };
        if frame.data == damiao::CMD_DISABLE {
            disable_frames += 1;
            continue;
        }
        if frame.id == 0x7FF && frame.data[2] == damiao::REG_WRITE && frame.data[3] == damiao::RID_CTRL_MODE {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            if (1..=6).contains(&id) {
                let val = u32::from_le_bytes([frame.data[4], frame.data[5], frame.data[6], frame.data[7]]);
                active_mode[(id - 1) as usize] = val;
            }
            continue;
        }
        if (0x101..=0x106).contains(&frame.id) {
            let j = (frame.id - 0x101) as usize;
            if active_mode[j] != damiao::CTRL_MODE_POS_VEL {
                wrong_mode_frames += 1;
            }
            if let Some(prev) = last_ctrl[j] {
                max_gap[j] = max_gap[j].max(t.saturating_duration_since(prev));
            }
            last_ctrl[j] = Some(t);
        } else if (1..=6).contains(&frame.id) && frame.data != damiao::CMD_ENABLE {
            let j = (frame.id - 1) as usize;
            if active_mode[j] != damiao::CTRL_MODE_MIT {
                wrong_mode_frames += 1;
            }
            if let Some(prev) = last_ctrl[j] {
                max_gap[j] = max_gap[j].max(t.saturating_duration_since(prev));
            }
            last_ctrl[j] = Some(t);
        }
    }
    for j in 0..6 {
        if let Some(prev) = last_ctrl[j] {
            max_gap[j] = max_gap[j].max(end_at.saturating_duration_since(prev));
        }
    }
    (max_gap, wrong_mode_frames, disable_frames)
}

fn step_release_on_exclusive(
    drv: &mut ExclusiveDriver,
    ms: &mut ModeSwitch,
    clock: &MockClock,
    period: Duration,
    max_tx: u8,
    seq: u64,
) -> ModeProgress {
    let tick_start = clock.now();
    let d = drv.get_mut().as_mut();
    d.begin_control_release(seq);
    let cutoff = tick_start + period.saturating_sub(Duration::from_millis(3));
    d.set_tx_release_deadline(Some(cutoff));
    let _ = d.read_joint_states();
    let budget = ReleaseBudget::new(clock.now(), cutoff, period, max_tx);
    let prog = ms.step(drv, budget);
    drv.get_mut().as_mut().set_tx_release_deadline(None);
    prog
}

// ---------------------------------------------------------------------------
// 1. FakeBus Damiao TIMEOUT=2000 6-axis switch: gap <= 40ms, 0 expiries, 0 alloc
// ---------------------------------------------------------------------------

#[test]
fn fakebus_damiao_timeout_6axis_switch_watchdog_gap_le_40ms_and_zero_alloc() {
    time_this_thread();
    let clock = MockClock::new();
    let cfg = ArmConfig::rebot_dm();
    let bus = configure_damiao_bus(Some(clock.clone()), Duration::from_micros(500), true, TIMEOUT_2000_COUNTS);
    let mut raw_drv = build_damiao_driver(bus.clone(), &cfg, 8);
    enable_driver_sync(&mut raw_drv, &bus, Some(&clock));

    let mut ex_drv = ExclusiveDriver::new(Box::new(raw_drv));

    // Seed confirmed control frames at t0 across J1..J6 and keep 100% wire logging continuously
    // across BOTH forward (PosVel -> Impedance) AND reverse (Impedance -> PosVel) switches AND
    // the first post-terminal normal control release (Major 8: no synthetic baseline!).
    let mut seq = 1u64;
    let start_at = clock.now();
    bus.clear_log();

    // Forward switch: all 6 arm axes PositionVelocity -> Impedance.
    let to_mit = ModeSwitchPlan::switch_axes(
        &[
            (0, ControlMode::Impedance),
            (1, ControlMode::Impedance),
            (2, ControlMode::Impedance),
            (3, ControlMode::Impedance),
            (4, ControlMode::Impedance),
            (5, ControlMode::Impedance),
        ],
        ModeSwitchRecovery::RollbackToPrevious,
    )
    .unwrap();
    let mut ms = ModeSwitch::new(to_mit);

    let mut final_prog = ModeProgress::Pending { wake: None };
    let mut tick_idx = 0u32;
    for _ in 0..40 {
        let t_tick = start_at + PERIOD_50HZ * tick_idx;
        if t_tick > clock.now() {
            clock.advance(t_tick - clock.now());
        }
        bus.with(|s| s.run_watchdogs());
        seq += 1;
        final_prog = step_release_on_exclusive(&mut ex_drv, &mut ms, &clock, PERIOD_50HZ, 8, seq);
        tick_idx += 1;
        if final_prog.is_terminal() {
            break;
        }
    }
    assert_eq!(final_prog, ModeProgress::Confirmed, "6-axis PosVel->Impedance must confirm");
    for j in 0..6 {
        assert_eq!(ex_drv.get_ref().axis_confirmed_mode(j), Some(ControlMode::Impedance));
    }
    assert_eq!(ex_drv.get_ref().mode_write_uncertain_mask(), 0);

    // Return switch: Impedance -> PositionVelocity (continuous log, no clear_log, no synthetic t0!).
    let to_pv = ModeSwitchPlan::switch_axes(
        &[
            (0, ControlMode::PositionVelocity),
            (1, ControlMode::PositionVelocity),
            (2, ControlMode::PositionVelocity),
            (3, ControlMode::PositionVelocity),
            (4, ControlMode::PositionVelocity),
            (5, ControlMode::PositionVelocity),
        ],
        ModeSwitchRecovery::RollbackToPrevious,
    )
    .unwrap();
    let mut ms_back = ModeSwitch::new(to_pv);
    let mut back_prog = ModeProgress::Pending { wake: None };
    for _ in 0..40 {
        let t_tick = start_at + PERIOD_50HZ * tick_idx;
        if t_tick > clock.now() {
            clock.advance(t_tick - clock.now());
        }
        bus.with(|s| s.run_watchdogs());
        seq += 1;
        back_prog = step_release_on_exclusive(&mut ex_drv, &mut ms_back, &clock, PERIOD_50HZ, 8, seq);
        tick_idx += 1;
        if back_prog.is_terminal() {
            break;
        }
    }
    assert_eq!(back_prog, ModeProgress::Confirmed, "6-axis Impedance->PosVel must confirm");
    for j in 0..6 {
        assert_eq!(ex_drv.get_ref().axis_confirmed_mode(j), Some(ControlMode::PositionVelocity));
    }

    // One post-terminal normal control release (verifies gap through handback to normal holds).
    {
        let t_tick = start_at + PERIOD_50HZ * tick_idx;
        if t_tick > clock.now() {
            clock.advance(t_tick - clock.now());
        }
        bus.with(|s| s.run_watchdogs());
        seq += 1;
        let d = ex_drv.get_mut().as_mut();
        d.begin_control_release(seq);
        let cutoff = clock.now() + PERIOD_50HZ.saturating_sub(Duration::from_millis(3));
        d.set_tx_release_deadline(Some(cutoff));
        let _ = d.read_joint_states();
        let holds = vec![armd::driver::JointTarget::position(0.0); 7];
        let _ = d.write_measured_hold_masked(0x3F, &holds);
        d.set_tx_release_deadline(None);
    }
    let full_end = clock.now();

    let (tx_log, tx_times, expiries, all_enabled) = bus.with(|s| {
        s.run_watchdogs();
        let exp: usize = s.motors.iter().map(|m| m.watchdog_expiry_count).sum();
        let en = s.motors.iter().all(|m| m.enabled);
        (s.tx_log.clone(), s.tx_times.clone(), exp, en)
    });
    assert_eq!(expiries, 0, "Damiao TIMEOUT=2000 must suffer 0 watchdog expiries");
    assert!(all_enabled, "all motors must remain enabled across round-trip switch");

    let (max_gap, wrong_mode, disables) =
        audit_wire_control_gaps(&tx_log, &tx_times, start_at, full_end);
    for j in 0..6 {
        assert!(
            max_gap[j] <= MAX_ALLOWED_GAP,
            "J{} continuous round-trip watchdog gap {:?} exceeded 40 ms bound (all gaps: {:?})",
            j + 1,
            max_gap[j],
            max_gap
        );
    }
    assert_eq!(wrong_mode, 0, "0 control frames may be sent in an unverified/wrong mode");
    assert_eq!(disables, 0, "0 CMD_DISABLE frames may be sent during mode switch");

    // Separate 4-counter zero-alloc proof (`alloc == alloc_zeroed == realloc == dealloc == 0`)
    // across an active RT step (with tx_log recording disabled so test FakeBus Vec doesn't grow).
    {
        let mut ms_alloc = ModeSwitch::new(
            ModeSwitchPlan::switch_axes(&[(0, ControlMode::Impedance)], ModeSwitchRecovery::FailClosed)
                .unwrap(),
        );
        let mut rx_buf = Vec::with_capacity(16);
        bus.with(|s| s.record_tx_log = false);
        for _ in 0..4 {
            clock.advance(PERIOD_50HZ);
            seq += 1;
            let before = snapshot_counts();
            let tick_start = clock.now();
            let d = ex_drv.get_mut().as_mut();
            d.begin_control_release(seq);
            let cutoff = tick_start + PERIOD_50HZ.saturating_sub(Duration::from_millis(3));
            d.set_tx_release_deadline(Some(cutoff));
            let _ = d.read_joint_states_into(&mut rx_buf);
            let budget = ReleaseBudget::new(clock.now(), cutoff, PERIOD_50HZ, 8);
            let p = ms_alloc.step(&mut ex_drv, budget);
            ex_drv.get_mut().as_mut().set_tx_release_deadline(None);
            let delta = snapshot_counts().delta_since(before);
            assert_eq!(
                (delta.alloc, delta.alloc_zeroed, delta.realloc, delta.dealloc),
                (0, 0, 0, 0),
                "ModeSwitch::step + RX intake must perform 0 heap allocs/frees on RT: {delta:?}"
            );
            if p.is_terminal() {
                break;
            }
        }
        bus.with(|s| s.record_tx_log = true);
    }

    // Major 2 proof: a configured J7 gripper verified in PosVel STILL receives PosVel hold
    // frames during an energized arm switch (not dropped by the PosForce-only branch).
    {
        bus.with(|s| {
            s.motors[6].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_VEL);
        });
        ex_drv
            .get_mut()
            .as_mut()
            .set_axis_confirmed_mode(6, Some(ControlMode::PositionVelocity));
        bus.clear_log();
        let mut ms_j7_hold = ModeSwitch::new(
            ModeSwitchPlan::switch_axes(&[(1, ControlMode::Impedance)], ModeSwitchRecovery::FailClosed)
                .unwrap(),
        );
        for _ in 0..6 {
            clock.advance(PERIOD_50HZ);
            seq += 1;
            let p = step_release_on_exclusive(&mut ex_drv, &mut ms_j7_hold, &clock, PERIOD_50HZ, 8, seq);
            if p.is_terminal() {
                break;
            }
        }
        let j7_posvel_holds = bus.with(|s| s.tx_log.iter().filter(|f| f.id == 0x107).count());
        assert!(
            j7_posvel_holds >= 1,
            "verified PosVel J7 must keep receiving PosVel hold frames during arm mode switch"
        );
    }
}

// ---------------------------------------------------------------------------
// 2. Watchdog budget first under tight TX, ACK-just-after-RX, and readback gate
// ---------------------------------------------------------------------------

#[test]
fn fakebus_watchdog_budget_first_tight_tx_ack_after_rx_and_readback_gate() {
    let clock = MockClock::new();
    let cfg = ArmConfig::rebot_dm();
    let bus = configure_damiao_bus(Some(clock.clone()), Duration::from_micros(500), true, TIMEOUT_2000_COUNTS);
    let mut raw_drv = build_damiao_driver(bus.clone(), &cfg, 8);
    enable_driver_sync(&mut raw_drv, &bus, Some(&clock));
    let mut ex_drv = ExclusiveDriver::new(Box::new(raw_drv));

    // (a) Tight TX budget = 6 frames/tick for 6 energized arm axes: holds MUST preempt register TX.
    // Advance past proactive watchdog interval (15 ms) so all 6 energized arm axes are due on tick 0.
    clock.advance(Duration::from_millis(16));
    let start = clock.now();
    bus.clear_log();
    let plan = ModeSwitchPlan::switch_axes(
        &[(0, ControlMode::Impedance), (1, ControlMode::Impedance)],
        ModeSwitchRecovery::RollbackToPrevious,
    )
    .unwrap();
    let mut ms = ModeSwitch::new(plan);
    let mut seq = 10u64;
    for tick in 0..4 {
        let t = start + PERIOD_50HZ * tick;
        if t > clock.now() { clock.advance(t - clock.now()); }
        bus.with(|s| s.run_watchdogs());
        seq += 1;
        let p = step_release_on_exclusive(&mut ex_drv, &mut ms, &clock, PERIOD_50HZ, 6, seq);
        assert!(matches!(p, ModeProgress::Pending { .. }), "tight 6-frame budget must defer register TX");
    }
    let reg_writes_while_tight = bus.with(|s| {
        s.tx_log
            .iter()
            .filter(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE)
            .count()
    });
    assert_eq!(reg_writes_while_tight, 0, "watchdog holds must consume all 6 slots before any REG_WRITE");
    let (gaps, wrong, disables) = bus.with(|s| {
        audit_wire_control_gaps(&s.tx_log, &s.tx_times, start, clock.now())
    });
    assert_eq!((wrong, disables), (0, 0));
    for (j, g) in gaps.iter().enumerate() {
        assert!(*g <= MAX_ALLOWED_GAP, "tight budget J{} gap {:?} > 40ms", j + 1, g);
    }

    // Open budget to 8 frames/tick -> switch completes cleanly with 0 expiries.
    let mut done = ModeProgress::Pending { wake: None };
    for tick in 4..20 {
        let t = start + PERIOD_50HZ * tick;
        if t > clock.now() { clock.advance(t - clock.now()); }
        bus.with(|s| s.run_watchdogs());
        seq += 1;
        done = step_release_on_exclusive(&mut ex_drv, &mut ms, &clock, PERIOD_50HZ, 8, seq);
        if done.is_terminal() { break; }
    }
    assert_eq!(done, ModeProgress::Confirmed);

    // (b) Blocker 1 boundary: ACK delayed just past release N+1 RX pass (~21 ms after write) ->
    // predictive watchdog guard MUST fail closed as Ambiguous on release N+1 (NOT wait to N+2 > 40 ms!).
    bus.with(|s| {
        s.motors[0].no_write_ack = true;
    });
    let mut ms_late = ModeSwitch::new(
        ModeSwitchPlan::switch_axes(
            &[(0, ControlMode::PositionVelocity)],
            ModeSwitchRecovery::FailClosed,
        )
        .unwrap(),
    );
    let t_b = clock.now() + PERIOD_50HZ;
    clock.advance(t_b - clock.now());
    seq += 1;
    let p_n = step_release_on_exclusive(&mut ex_drv, &mut ms_late, &clock, PERIOD_50HZ, 8, seq);
    assert!(matches!(p_n, ModeProgress::Pending { .. }));
    assert_ne!(
        ex_drv.get_ref().mode_write_uncertain_mask() & 1,
        0,
        "driver must mark J1 uncertain immediately at REG_WRITE TX boundary"
    );
    assert_eq!(
        ex_drv.get_ref().axis_confirmed_mode(0),
        None,
        "driver must clear verified_hold_mode immediately at REG_WRITE TX boundary"
    );

    // Blocker 1 proof: even if J1 feedback becomes stale (> 45 ms), ordinary RX recovery MUST NOT
    // transmit any guessed-mode control frame for uncertain J1!
    bus.clear_log();
    clock.advance(Duration::from_millis(50));
    {
        let d = ex_drv.get_mut().as_mut();
        d.begin_control_release(seq + 100);
        let _ = d.read_joint_states();
    }
    let j1_recovery_frames = bus.with(|s| {
        s.tx_log
            .iter()
            .filter(|f| f.id == 0x101 || (f.id == 1 && f.data != damiao::CMD_ENABLE))
            .count()
    });
    assert_eq!(
        j1_recovery_frames, 0,
        "ordinary RX recovery must transmit 0 control frames for mode-uncertain J1"
    );

    // Advance & step ms_late -> MUST abort immediately as Ambiguous { WriteAckTimeout }!
    seq += 1;
    let p_n1 = step_release_on_exclusive(&mut ex_drv, &mut ms_late, &clock, PERIOD_50HZ, 8, seq);
    assert_eq!(
        p_n1,
        ModeProgress::Ambiguous {
            failed_joint: 0,
            reason: ModeSwitchFailure::WriteAckTimeout,
        },
        "must abort fail-closed as Ambiguous when ACK misses predictive watchdog deadline"
    );
    assert_ne!(ex_drv.get_ref().mode_write_uncertain_mask() & 1, 0);

    // Restore J1 cleanly for part (c).
    bus.with(|s| {
        s.motors[0].no_write_ack = false;
        s.motors[0].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_VEL);
        s.motors[0].enabled = true;
        s.push_all_feedback();
    });
    ex_drv.get_mut().as_mut().set_axis_confirmed_mode(0, Some(ControlMode::PositionVelocity));

    // (c) Required post-write readback:
    // - At 50 Hz (2 * 20 + 5 = 45 > 40 ms): refused fail-closed at admission with 0 REG_WRITE frames!
    let mut plan_rb = ModeSwitchPlan::switch_axes(
        &[(0, ControlMode::Impedance)],
        ModeSwitchRecovery::FailClosed,
    )
    .unwrap();
    plan_rb.axes[0].require_post_write_readback = true;
    bus.clear_log();
    let mut ms_rb_50 = ModeSwitch::new(plan_rb);
    clock.advance(PERIOD_50HZ);
    seq += 1;
    let p_50 = step_release_on_exclusive(&mut ex_drv, &mut ms_rb_50, &clock, PERIOD_50HZ, 8, seq);
    assert_eq!(
        p_50,
        ModeProgress::FailedClean {
            failed_joint: 0,
            reason: ModeSwitchFailure::WatchdogBudgetBreached,
        }
    );
    assert_eq!(
        bus.with(|s| s.tx_log.iter().filter(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE).count()),
        0,
        "50 Hz admission refusal must emit 0 REG_WRITE frames"
    );

    // - At 100 Hz (period = 10 ms; 2 * 10 + 5 = 25 <= 40 ms): admitted and confirmed causally with REG_READ!
    let period_100hz = Duration::from_millis(10);
    let mut ms_rb_100 = ModeSwitch::new(plan_rb);
    let mut p_100 = ModeProgress::Pending { wake: None };
    for _ in 0..12 {
        clock.advance(period_100hz);
        bus.with(|s| s.run_watchdogs());
        seq += 1;
        p_100 = step_release_on_exclusive(&mut ex_drv, &mut ms_rb_100, &clock, period_100hz, 8, seq);
        if p_100.is_terminal() { break; }
    }
    assert_eq!(p_100, ModeProgress::Confirmed);
    let saw_ctrl_read = bus.with(|s| {
        s.tx_log
            .iter()
            .any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_READ && f.data[3] == damiao::RID_CTRL_MODE)
    });
    assert!(saw_ctrl_read, "post-write readback must have transmitted REG_READ(RID_CTRL_MODE)");

    // (d) Blocker 6 proof: 100 Hz post-write readback enters `NeedReadback`, then TX backpressure
    // (`max_tx = 0`) prevents transmitting `REG_READ` -> single absolute `post_write_deadline`
    // MUST abort fail-closed as `Ambiguous { ReadbackTimeout }` (never stranded!).
    let mut plan_rb_back = ModeSwitchPlan::switch_axes(
        &[(0, ControlMode::PositionVelocity)],
        ModeSwitchRecovery::FailClosed,
    )
    .unwrap();
    plan_rb_back.axes[0].require_post_write_readback = true;
    let mut ms_strand = ModeSwitch::new(plan_rb_back);
    clock.advance(period_100hz);
    seq += 1;
    let p_w = step_release_on_exclusive(&mut ex_drv, &mut ms_strand, &clock, period_100hz, 8, seq);
    assert!(matches!(p_w, ModeProgress::Pending { .. }));
    let mut p_strand = ModeProgress::Pending { wake: None };
    for _ in 0..5 {
        clock.advance(period_100hz);
        seq += 1;
        // Starve TX budget (max_tx = 0) so `NeedReadback` cannot transmit `REG_READ`!
        p_strand = step_release_on_exclusive(&mut ex_drv, &mut ms_strand, &clock, period_100hz, 0, seq);
        if p_strand.is_terminal() {
            break;
        }
    }
    assert_eq!(
        p_strand,
        ModeProgress::Ambiguous {
            failed_joint: 0,
            reason: ModeSwitchFailure::ReadbackTimeout,
        },
        "NeedReadback under TX starvation must abort at post_write_deadline, never strand"
    );
}

// ---------------------------------------------------------------------------
// 3. Causal rejections, mandatory ranges, rollback, flush fail, duplicates, ConfirmOnly contradiction
// ---------------------------------------------------------------------------

#[test]
fn fakebus_causal_rejections_ranges_rollback_flush_fail_duplicates_and_confirm_only() {
    let clock = MockClock::new();
    let cfg = ArmConfig::rebot_dm();
    let bus = configure_damiao_bus(Some(clock.clone()), Duration::from_micros(500), true, TIMEOUT_2000_COUNTS);
    let mut raw_drv = build_damiao_driver(bus.clone(), &cfg, 8);
    enable_driver_sync(&mut raw_drv, &bus, Some(&clock));
    let mut ex_drv = ExclusiveDriver::new(Box::new(raw_drv));
    let mut seq = 100u64;

    // Minor 1 & Major 7: typed plan construction errors, empty direct plan, & overflowing timeout!
    assert_eq!(
        ModeSwitchPlan::switch_axes(&[(0, ControlMode::Impedance), (99, ControlMode::Impedance)], ModeSwitchRecovery::FailClosed),
        Err(ModeSwitchFailure::InvalidPlan)
    );
    assert_eq!(
        ModeSwitchPlan::switch_axes(&[(0, ControlMode::Impedance), (0, ControlMode::Impedance)], ModeSwitchRecovery::FailClosed),
        Err(ModeSwitchFailure::InvalidPlan)
    );
    assert_eq!(
        ModeSwitchPlan::confirm_gripper(99, ControlMode::PositionForce),
        Err(ModeSwitchFailure::InvalidPlan)
    );
    let mut ms_empty = ModeSwitch::new(ModeSwitchPlan::empty(ModeSwitchIntent::Switch));
    clock.advance(PERIOD_50HZ);
    seq += 1;
    assert_eq!(
        step_release_on_exclusive(&mut ex_drv, &mut ms_empty, &clock, PERIOD_50HZ, 8, seq),
        ModeProgress::FailedClean {
            failed_joint: 0,
            reason: ModeSwitchFailure::InvalidPlan,
        },
        "direct runner must reject empty plan with InvalidPlan (never Confirmed)"
    );
    let mut bad_timeout_plan =
        ModeSwitchPlan::switch_axes(&[(0, ControlMode::Impedance)], ModeSwitchRecovery::FailClosed)
            .unwrap();
    bad_timeout_plan.per_axis_timeout = Duration::MAX;
    assert_eq!(bad_timeout_plan.validate(), Err(ModeSwitchFailure::InvalidPlan));
    let mut ms_bad_to = ModeSwitch::new(bad_timeout_plan);
    clock.advance(PERIOD_50HZ);
    seq += 1;
    assert_eq!(
        step_release_on_exclusive(&mut ex_drv, &mut ms_bad_to, &clock, PERIOD_50HZ, 8, seq),
        ModeProgress::FailedClean {
            failed_joint: 0,
            reason: ModeSwitchFailure::InvalidPlan,
        }
    );

    // Blocker 2: PositionForce on a non-PosForce joint (J1) refused fail-closed!
    let mut ms_pf_j1 = ModeSwitch::new(
        ModeSwitchPlan::confirm_gripper(0, ControlMode::PositionForce).unwrap(),
    );
    clock.advance(PERIOD_50HZ);
    seq += 1;
    assert_eq!(
        step_release_on_exclusive(&mut ex_drv, &mut ms_pf_j1, &clock, PERIOD_50HZ, 8, seq),
        ModeProgress::FailedClean {
            failed_joint: 0,
            reason: ModeSwitchFailure::UnsupportedMode,
        }
    );

    // (a) Mandatory wire-range check + partial switch causal rollback:
    // J1 ranges verified; J2 ranges unverified and hardware reports mismatched TMAX (99.0 vs configured 28.0).
    // J1 switches to Impedance -> J2 fails mandatory range check BEFORE writing J2 -> J1 rolls back to PosVel!
    ex_drv.get_mut().as_mut().set_axis_impedance_ranges_verified(1, false);
    bus.with(|s| {
        s.motors[1].regs.insert(damiao::RID_TMAX, 99.0f32.to_le_bytes());
    });
    bus.clear_log();
    let mut ms_rb = ModeSwitch::new(
        ModeSwitchPlan::switch_axes(
            &[(0, ControlMode::Impedance), (1, ControlMode::Impedance)],
            ModeSwitchRecovery::RollbackToPrevious,
        )
        .unwrap(),
    );
    let mut prog = ModeProgress::Pending { wake: None };
    for _ in 0..30 {
        clock.advance(PERIOD_50HZ);
        bus.with(|s| s.run_watchdogs());
        seq += 1;
        prog = step_release_on_exclusive(&mut ex_drv, &mut ms_rb, &clock, PERIOD_50HZ, 8, seq);
        if prog.is_terminal() { break; }
    }
    assert_eq!(
        prog,
        ModeProgress::RolledBack {
            failed_joint: 1,
            reason: ModeSwitchFailure::RangeMismatch,
        }
    );
    assert_eq!(ex_drv.get_ref().axis_confirmed_mode(0), Some(ControlMode::PositionVelocity));
    assert_eq!(ex_drv.get_ref().axis_confirmed_mode(1), Some(ControlMode::PositionVelocity));
    assert_eq!(ex_drv.get_ref().mode_write_uncertain_mask(), 0);
    // Verify J2 NEVER received a REG_WRITE(RID_CTRL_MODE)!
    let j2_writes = bus.with(|s| {
        s.tx_log
            .iter()
            .filter(|f| f.id == 0x7FF && f.data[0] == 2 && f.data[2] == damiao::REG_WRITE)
            .count()
    });
    assert_eq!(j2_writes, 0, "range mismatch must forbid REG_WRITE on J2");

    // Blocker 3 proof: ConfirmOnly(Impedance) with unverified mismatched TMAX on J2 MUST fail
    // with RangeMismatch and NEVER publish Impedance as eligible for MIT holds!
    let mut co_mit_plan = ModeSwitchPlan::empty(ModeSwitchIntent::ConfirmOnly);
    co_mit_plan.axes[0] = AxisTarget::new(1, ControlMode::Impedance);
    co_mit_plan.len = 1;
    let mut ms_co_mit = ModeSwitch::new(co_mit_plan);
    let mut p_co_mit = ModeProgress::Pending { wake: None };
    for _ in 0..12 {
        clock.advance(PERIOD_50HZ);
        bus.with(|s| s.run_watchdogs());
        seq += 1;
        p_co_mit = step_release_on_exclusive(&mut ex_drv, &mut ms_co_mit, &clock, PERIOD_50HZ, 8, seq);
        if p_co_mit.is_terminal() { break; }
    }
    assert_eq!(
        p_co_mit,
        ModeProgress::FailedClean {
            failed_joint: 1,
            reason: ModeSwitchFailure::RangeMismatch,
        },
        "ConfirmOnly(Impedance) must refuse unverified mismatched wire ranges"
    );

    // (b) Failed flush after REG_WRITE (`real_flush = true`, `fail_next_reg_write_flush = true`):
    // Must immediately set uncertain mask & return Ambiguous!
    bus.with(|s| {
        s.real_flush = true;
        s.fail_next_reg_write_flush = true;
    });
    let mut ms_fl = ModeSwitch::new(
        ModeSwitchPlan::switch_axes(&[(0, ControlMode::Impedance)], ModeSwitchRecovery::FailClosed)
            .unwrap(),
    );
    clock.advance(PERIOD_50HZ);
    seq += 1;
    let p_fl = step_release_on_exclusive(&mut ex_drv, &mut ms_fl, &clock, PERIOD_50HZ, 8, seq);
    assert_eq!(
        p_fl,
        ModeProgress::Ambiguous {
            failed_joint: 0,
            reason: ModeSwitchFailure::TxFailed,
        }
    );
    assert_ne!(ex_drv.get_ref().mode_write_uncertain_mask() & 1, 0);
    assert_eq!(ex_drv.get_ref().axis_confirmed_mode(0), None);
    bus.with(|s| {
        s.real_flush = false;
        s.fail_flushes = 0;
        s.motors[0].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_VEL);
    });
    ex_drv.get_mut().as_mut().set_axis_confirmed_mode(0, Some(ControlMode::PositionVelocity));

    // (c) Duplicate REG_WRITE ACK in the same window -> Ambiguous!
    let dup_ack = CanFrame::new_standard(
        0x11,
        &[1, 0, damiao::REG_WRITE, damiao::RID_CTRL_MODE, 1, 0, 0, 0],
    )
    .unwrap();
    bus.with(|s| {
        s.inject_on_reg_write.push(dup_ack);
    });
    let mut ms_dup = ModeSwitch::new(
        ModeSwitchPlan::switch_axes(&[(0, ControlMode::Impedance)], ModeSwitchRecovery::FailClosed)
            .unwrap(),
    );
    let mut p_dup = ModeProgress::Pending { wake: None };
    for _ in 0..5 {
        clock.advance(PERIOD_50HZ);
        bus.with(|s| s.run_watchdogs());
        seq += 1;
        p_dup = step_release_on_exclusive(&mut ex_drv, &mut ms_dup, &clock, PERIOD_50HZ, 8, seq);
        if p_dup.is_terminal() { break; }
    }
    assert_eq!(
        p_dup,
        ModeProgress::Ambiguous {
            failed_joint: 0,
            reason: ModeSwitchFailure::AmbiguousReply,
        }
    );
    assert_eq!(ex_drv.get_ref().axis_confirmed_mode(0), None);
    bus.with(|s| s.motors[0].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_VEL));
    ex_drv.get_mut().as_mut().set_axis_confirmed_mode(0, Some(ControlMode::PositionVelocity));

    // (d) Blocker 3/4: Energized ConfirmOnly contradiction on J7!
    // Driver cache believes J7 is verified PosForce, but motor causally answers POS_VEL.
    // Must invalidate cached PosForce mode & return Ambiguous!
    assert_eq!(ex_drv.get_ref().axis_confirmed_mode(6), Some(ControlMode::PositionForce));
    bus.with(|s| {
        s.motors[6].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_VEL);
    });
    let mut ms_j7_contra =
        ModeSwitch::new(ModeSwitchPlan::confirm_gripper(6, ControlMode::PositionForce).unwrap());
    let mut p_contra = ModeProgress::Pending { wake: None };
    for _ in 0..6 {
        clock.advance(PERIOD_50HZ);
        bus.with(|s| s.run_watchdogs());
        seq += 1;
        p_contra = step_release_on_exclusive(&mut ex_drv, &mut ms_j7_contra, &clock, PERIOD_50HZ, 8, seq);
        if p_contra.is_terminal() { break; }
    }
    assert_eq!(
        p_contra,
        ModeProgress::Ambiguous {
            failed_joint: 6,
            reason: ModeSwitchFailure::ReadbackMismatch,
        }
    );
    assert_eq!(
        ex_drv.get_ref().axis_confirmed_mode(6),
        None,
        "contradictory energized ConfirmOnly reply MUST invalidate cached PosForce mode"
    );
    assert!(
        !ex_drv.get_ref().gripper_pos_force_info().unwrap().active,
        "pos_force_verified must be cleared on contradiction"
    );

    // (e) Blocker 4: Energized J7 ConfirmOnly wrong reply CAN ID (`reply_id != 0x17`) ->
    // invalidates mode and escalates as Ambiguous { AmbiguousReply } while energized!
    bus.with(|s| {
        s.motors[6].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_FORCE);
        s.motors[6].reply_id = 0x18; // wrong master reply CAN ID
    });
    let mut ms_j7_id =
        ModeSwitch::new(ModeSwitchPlan::confirm_gripper(6, ControlMode::PositionForce).unwrap());
    let mut p_id = ModeProgress::Pending { wake: None };
    for _ in 0..6 {
        clock.advance(PERIOD_50HZ);
        bus.with(|s| s.run_watchdogs());
        seq += 1;
        p_id = step_release_on_exclusive(&mut ex_drv, &mut ms_j7_id, &clock, PERIOD_50HZ, 8, seq);
        if p_id.is_terminal() { break; }
    }
    assert_eq!(
        p_id,
        ModeProgress::Ambiguous {
            failed_joint: 6,
            reason: ModeSwitchFailure::AmbiguousReply,
        }
    );
    assert!(!ex_drv.get_ref().gripper_pos_force_info().unwrap().active);
}

// ---------------------------------------------------------------------------
// 4. Stream saturation, stale cancel/clear, and in-flight revocation
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn session_stream_saturation_stale_cancel_clear_and_inflight_revocation() {
    let cfg = ArmConfig::rebot_dm();
    let bus = configure_damiao_bus(None, Duration::ZERO, true, 0);
    let drv = build_damiao_driver(bus.clone(), &cfg, 8);
    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;

    // Attach capacity-1 off-RT stream (deliberately saturated!) before starting executor.
    let mut ev_rx = {
        let mut g = sup.inner().write().await;
        g.attach_mode_switch_event_stream(1)
    };
    let mut ecfg = ExecutorConfig::for_tests(Duration::from_millis(8));
    ecfg.alloc_timing = true;
    let exec = sup.start_executor(ecfg).await.unwrap();
    let dl = Instant::now() + Duration::from_secs(2);
    while sup.hub().latest().is_none() {
        assert!(Instant::now() < dl);
        tokio::time::sleep(Duration::from_millis(5)).await;
    }

    // Run a multi-axis confirm (emits Started + AxisConfirmed + AxisConfirmed + Terminal = 4 events > cap 1!).
    let mut plan = ModeSwitchPlan::empty(ModeSwitchIntent::ConfirmOnly);
    plan.axes[0] = AxisTarget::new(0, ControlMode::PositionVelocity);
    plan.axes[0].require_post_confirm_feedback = false;
    plan.axes[1] = AxisTarget::new(1, ControlMode::PositionVelocity);
    plan.axes[1].require_post_confirm_feedback = false;
    plan.len = 2;

    let before_allocs = exec.stats().release_allocs.snapshot();
    let sid1 = sup
        .start_mode_switch_session(ModeSwitchStartRequest {
            plan,
            admission: sup.capture_admission_stamp(),
            operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
            deadline: Instant::now() + Duration::from_secs(2),
            client: ClientId(7),
        })
        .await
        .unwrap();

    // Poll via StateHub::read_services until terminal despite saturated capacity-1 event stream.
    let wait_dl = Instant::now() + Duration::from_secs(2);
    let snap1 = loop {
        if let Some(s) = sup.poll_mode_switch_session(sid1).await.unwrap() {
            if !s.running {
                break s;
            }
        }
        assert!(Instant::now() < wait_dl);
        tokio::time::sleep(Duration::from_millis(5)).await;
    };
    assert_eq!(snap1.progress, ModeProgress::Confirmed);
    // Major 5 proof: owner-side retirement frees `ExclusiveOutput` automatically after terminal
    // publication, so a dropped caller who NEVER calls `clear_mode_switch_session` cannot wedge
    // the slot! (Wait one tick for post-publish flush, then start sid2 WITHOUT calling clear.)
    tokio::time::sleep(Duration::from_millis(20)).await;
    // Stream dropped excess events cleanly without blocking authoritative poll/retirement.
    let mut popped = 0usize;
    while ev_rx.try_pop().is_some() {
        popped += 1;
    }
    assert_eq!(popped, 1);
    assert!(ev_rx.stats().dropped_full >= 1, "capacity-1 stream must record drop-newest loss");

    // Start a second session (silent J1 so it stays live) WITHOUT having called clear on sid1,
    // and verify stale cancel/clear for sid1 are ignored and do NOT disturb sid2!
    bus.with(|s| s.motors[0].silent = true);
    let sid2 = sup
        .start_mode_switch_session(ModeSwitchStartRequest {
            plan,
            admission: sup.capture_admission_stamp(),
            operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
            deadline: Instant::now() + Duration::from_secs(2),
            client: ClientId(7),
        })
        .await
        .unwrap();
    assert_ne!(sid1, sid2);
    assert!(
        sup.cancel_mode_switch_session(sid1, "STOPPED").await.unwrap().is_none(),
        "stale cancel for old session must return None"
    );
    assert!(
        !sup.clear_mode_switch_session(sid1).await.unwrap(),
        "stale clear for old session must return false"
    );
    // Wait until StateHub publishes sid2 running snapshot.
    let wait_live = Instant::now() + Duration::from_secs(1);
    let live2 = loop {
        if let Some(s) = sup.poll_mode_switch_session(sid2).await.unwrap() {
            break s;
        }
        assert!(Instant::now() < wait_live);
        tokio::time::sleep(Duration::from_millis(5)).await;
    };
    assert!(live2.running, "stale cancel/clear must never disturb active sid2");
    // R2 Major 6 proof: even AFTER sid2 started, sid1's terminal snapshot remains readable from
    // StateHub's 4-slot recent_mode_switch_terminals ring!
    let retained_s1 = sup
        .poll_mode_switch_session(sid1)
        .await
        .unwrap()
        .expect("sid1 terminal result must remain readable after sid2 starts");
    assert_eq!(retained_s1.progress, ModeProgress::Confirmed);

    // R2 Major 8 proof: overflowing wrapper timeout is rejected BEFORE admitting any session!
    let err_overflow = sup
        .confirm_gripper_mode_session(
            6,
            ControlMode::PositionVelocity,
            sup.capture_admission_stamp(),
            OperatorGrant::unenforced(),
            Duration::MAX,
        )
        .await
        .unwrap_err();
    assert!(matches!(
        err_overflow,
        ConfigError::CalibPrecondition {
            code: "INVALID_COMMAND",
            ..
        }
    ));

    // Clean cancel & clear of sid2.
    let c2 = sup.cancel_mode_switch_session(sid2, "STOPPED").await.unwrap().unwrap();
    assert_eq!(
        c2.progress,
        ModeProgress::StoppedRecovered {
            reason: ModeSwitchFailure::Stopped
        }
    );
    assert!(sup.clear_mode_switch_session(sid2).await.unwrap());
    // R4 Major 2 proof: after sid2 ALSO completes, BOTH sid2 (latest terminal) AND sid1 (previous
    // terminal) remain readable from StateHub::read_services!
    let wait_both = Instant::now() + Duration::from_secs(1);
    loop {
        let p2 = sup.poll_mode_switch_session(sid2).await.unwrap();
        let p1 = sup.poll_mode_switch_session(sid1).await.unwrap();
        if p2.as_ref().is_some_and(|s| !s.running) && p1.as_ref().is_some_and(|s| !s.running) {
            assert_eq!(
                p2.unwrap().progress,
                ModeProgress::StoppedRecovered {
                    reason: ModeSwitchFailure::Stopped
                }
            );
            assert_eq!(p1.unwrap().progress, ModeProgress::Confirmed);
            break;
        }
        assert!(
            Instant::now() < wait_both,
            "both sid1 and sid2 terminals must remain readable after sid2 finishes: p1={p1:?} p2={p2:?}"
        );
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    let after_allocs = exec.stats().release_allocs.snapshot();
    assert_eq!(
        (
            after_allocs.alloc.saturating_sub(before_allocs.alloc),
            after_allocs.alloc_zeroed.saturating_sub(before_allocs.alloc_zeroed),
            after_allocs.realloc.saturating_sub(before_allocs.realloc),
            after_allocs.dealloc.saturating_sub(before_allocs.dealloc),
        ),
        (0, 0, 0, 0),
        "Executor armd-control releases across admission, stepping, cancel, and terminal publish must do 0 allocs/frees"
    );
    assert_eq!(sup.inner().control_thread_attempts(), 0, "0 SupervisorLock attempts on armd-control");

    exec.stop_and_join();
}

// ---------------------------------------------------------------------------
// 5. Dual-owner harness & parity tests (Legacy vs Executor identical results)
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum OwnerKind {
    Legacy,
    Executor,
}

struct OwnerHarness {
    kind: OwnerKind,
    sup: Arc<ArmSupervisor>,
    bus: FakeBus,
    exec: Option<armd::rt_core::executor::RtExecutorHandle>,
    legacy_stop: Option<Arc<AtomicBool>>,
    legacy_thread: Option<std::thread::JoinHandle<()>>,
}

impl OwnerHarness {
    async fn start(
        kind: OwnerKind,
        j7_pos_force: bool,
        arm_timeout_counts: u32,
        arm_enabled: bool,
        lease_enabled: bool,
    ) -> Self {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.control_frequency_hz = 50;
        let bus = configure_damiao_bus(None, Duration::ZERO, j7_pos_force, arm_timeout_counts);
        let mut drv = build_damiao_driver(bus.clone(), &cfg, 8);
        if arm_enabled {
            bus.with(|s| {
                s.immediate_replies = true;
                s.push_all_feedback();
            });
            let _ = drv.check_mapping_ranges();
            let _ = drv.read_joint_states();
        }
        let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
        sup.set_lease_enabled(lease_enabled).await;
        if lease_enabled {
            let _ = sup.acquire_lease("mode-switch-test-client").await.unwrap();
        }
        if arm_enabled {
            sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
            bus.with(|s| {
                s.immediate_replies = false;
                s.push_all_feedback();
            });
        }
        match kind {
            OwnerKind::Executor => {
                let mut ecfg = ExecutorConfig::for_tests(Duration::from_millis(20));
                ecfg.alloc_timing = true;
                ecfg.panic_policy = PanicPolicy::StopExecutor;
                let exec = sup.start_executor(ecfg).await.unwrap();
                let dl = Instant::now() + Duration::from_secs(2);
                while sup.hub().latest().is_none() {
                    assert!(Instant::now() < dl);
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
                Self {
                    kind,
                    sup,
                    bus,
                    exec: Some(exec),
                    legacy_stop: None,
                    legacy_thread: None,
                }
            }
            OwnerKind::Legacy => {
                let stop = Arc::new(AtomicBool::new(false));
                let stop_t = Arc::clone(&stop);
                let sup_t = Arc::clone(&sup);
                let stats = Arc::new(ExecutorStats::default());
                let rt = tokio::runtime::Handle::current();
                let jh = std::thread::Builder::new()
                    .name("legacy-mode-switch-test".into())
                    .spawn(move || {
                        sup_t.run_legacy_control_loop_blocking(
                            Duration::from_millis(20),
                            stop_t,
                            stats,
                            rt,
                            None,
                        );
                    })
                    .unwrap();
                let dl = Instant::now() + Duration::from_secs(2);
                while sup.hub().latest().is_none() {
                    assert!(Instant::now() < dl);
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
                Self {
                    kind,
                    sup,
                    bus,
                    exec: None,
                    legacy_stop: Some(stop),
                    legacy_thread: Some(jh),
                }
            }
        }
    }

    async fn shutdown(mut self, force_emergency_disable: bool) {
        match self.kind {
            OwnerKind::Executor => {
                if let Some(exec) = self.exec.take() {
                    if force_emergency_disable {
                        let _ = exec.emergency_disable_and_join();
                    } else {
                        exec.stop_and_join();
                    }
                }
            }
            OwnerKind::Legacy => {
                if force_emergency_disable {
                    let sh = self.sup.legacy_safe_hold_control();
                    sh.request_process_exit_disable(Instant::now() + Duration::from_secs(2));
                }
                if let Some(stop) = self.legacy_stop.take() {
                    stop.store(true, std::sync::atomic::Ordering::Release);
                }
                if let Some(jh) = self.legacy_thread.take() {
                    let _ = jh.join();
                }
            }
        }
    }
}

/// Dual-owner test 1: J7 gripper mode confirmation reference adapter (`PositionForce`,
/// `PositionVelocity`, and mismatch refusal) produces identical results on Legacy & Executor.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_j7_gripper_mode_confirmation_pos_vel_and_pos_force() {
    let mut results = Vec::new();
    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let h = OwnerHarness::start(owner, true, 0, false, true).await;

        // 1. Confirm J7 in PositionForce (disarmed stationary + expected master reply ID 0x17).
        let snap_pf = h
            .sup
            .confirm_gripper_mode_session(
                6,
                ControlMode::PositionForce,
                h.sup.capture_admission_stamp(),
                OperatorGrant::unenforced(),
                Duration::from_secs(1),
            )
            .await
            .unwrap();
        assert_eq!(snap_pf.progress, ModeProgress::Confirmed, "{owner:?}: J7 PosForce confirm");

        // 2. Confirm J7 mismatch: ask for PositionVelocity while hardware is PositionForce -> FailedClean.
        let snap_mis = h
            .sup
            .confirm_gripper_mode_session(
                6,
                ControlMode::PositionVelocity,
                h.sup.capture_admission_stamp(),
                OperatorGrant::unenforced(),
                Duration::from_secs(1),
            )
            .await
            .unwrap();
        assert_eq!(
            snap_mis.progress,
            ModeProgress::FailedClean {
                failed_joint: 6,
                reason: ModeSwitchFailure::ReadbackMismatch,
            },
            "{owner:?}: J7 mismatch must fail clean"
        );

        // 3. Hardware in PositionVelocity -> ConfirmOnly(PositionVelocity) confirms cleanly.
        h.bus.with(|s| {
            s.motors[6].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_VEL);
        });
        let snap_pv = h
            .sup
            .confirm_gripper_mode_session(
                6,
                ControlMode::PositionVelocity,
                h.sup.capture_admission_stamp(),
                OperatorGrant::unenforced(),
                Duration::from_secs(1),
            )
            .await
            .unwrap();
        assert_eq!(snap_pv.progress, ModeProgress::Confirmed, "{owner:?}: J7 PosVel confirm");

        if owner == OwnerKind::Executor {
            assert_eq!(h.sup.inner().control_thread_attempts(), 0);
        }
        results.push((snap_pf.progress, snap_mis.progress, snap_pv.progress));
        h.shutdown(false).await;
    }
    assert_eq!(results[0], results[1], "Legacy and Executor J7 confirmation must be identical");
}

/// Dual-owner test 2: Energized 3-axis `PositionVelocity -> Impedance -> PositionVelocity` switch
/// AND partial-switch range-mismatch rollback produce identical outcomes on Legacy & Executor
/// with live leases enabled.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_energized_mode_switch_and_rollback_parity() {
    let mut results = Vec::new();
    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let h = OwnerHarness::start(owner, true, TIMEOUT_2000_COUNTS, true, true).await;
        h.bus.clear_log();

        // 1. Switch J1..J3 PosVel -> Impedance.
        let plan_mit = ModeSwitchPlan::switch_axes(
            &[
                (0, ControlMode::Impedance),
                (1, ControlMode::Impedance),
                (2, ControlMode::Impedance),
            ],
            ModeSwitchRecovery::RollbackToPrevious,
        )
        .unwrap();
        let snap_mit = h
            .sup
            .run_mode_switch_session(
                ModeSwitchStartRequest {
                    plan: plan_mit,
                    admission: h.sup.capture_admission_stamp(),
                    operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
                    deadline: Instant::now() + Duration::from_secs(2),
                    client: ClientId(10),
                },
                Duration::from_secs(2),
            )
            .await
            .unwrap();
        assert_eq!(snap_mit.progress, ModeProgress::Confirmed, "{owner:?}: PosVel->Impedance");

        // 2. Switch J1..J3 back Impedance -> PositionVelocity.
        let plan_pv = ModeSwitchPlan::switch_axes(
            &[
                (0, ControlMode::PositionVelocity),
                (1, ControlMode::PositionVelocity),
                (2, ControlMode::PositionVelocity),
            ],
            ModeSwitchRecovery::RollbackToPrevious,
        )
        .unwrap();
        let snap_pv = h
            .sup
            .run_mode_switch_session(
                ModeSwitchStartRequest {
                    plan: plan_pv,
                    admission: h.sup.capture_admission_stamp(),
                    operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
                    deadline: Instant::now() + Duration::from_secs(2),
                    client: ClientId(10),
                },
                Duration::from_secs(2),
            )
            .await
            .unwrap();
        assert_eq!(snap_pv.progress, ModeProgress::Confirmed, "{owner:?}: Impedance->PosVel");

        let (disables, all_enabled, modes_after, expiries) = h.bus.with(|s| {
            let dis = s.tx_log.iter().filter(|f| f.data == damiao::CMD_DISABLE).count();
            let en = s.motors.iter().all(|m| m.enabled);
            let modes = [s.motors[0].ctrl_mode(), s.motors[1].ctrl_mode(), s.motors[2].ctrl_mode()];
            let exp: usize = s.motors.iter().map(|m| m.watchdog_expiry_count).sum();
            (dis, en, modes, exp)
        });
        assert_eq!(disables, 0, "{owner:?}: 0 CMD_DISABLE frames");
        assert_eq!(expiries, 0, "{owner:?}: 0 watchdog expiries");
        assert!(all_enabled, "{owner:?}: all motors remain enabled");
        assert_eq!(modes_after, [damiao::CTRL_MODE_POS_VEL; 3]);

        // 3. Partial-switch rollback parity: J1 confirms Impedance, J2 fails mandatory range check
        //    (mismatched TMAX=99.0) before writing J2 -> J1 rolls back causally to PosVel.
        h.shutdown(true).await;

        let snap_rb = {
            let mut cfg = ArmConfig::rebot_dm();
            cfg.control_frequency_hz = 50;
            let bus_rb = configure_damiao_bus(None, Duration::ZERO, true, TIMEOUT_2000_COUNTS);
            let mut drv_rb = build_damiao_driver(bus_rb.clone(), &cfg, 8);
            bus_rb.with(|s| {
                s.immediate_replies = true;
                s.push_all_feedback();
            });
            let _ = drv_rb.check_mapping_ranges();
            let _ = drv_rb.read_joint_states();
            drv_rb.set_axis_impedance_ranges_verified(1, false);
            bus_rb.with(|s| {
                s.motors[1].regs.insert(damiao::RID_TMAX, 99.0f32.to_le_bytes());
            });
            let sup_rb = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv_rb)));
            sup_rb.set_lease_enabled(true).await;
            let _ = sup_rb.acquire_lease("rb-client").await.unwrap();
            sup_rb.enable_torque(sup_rb.capture_admission_stamp()).await.unwrap();
            bus_rb.with(|s| {
                s.immediate_replies = false;
                s.push_all_feedback();
            });
            let mut exec_rb = None;
            let mut stop_rb = None;
            let mut jh_rb = None;
            match owner {
                OwnerKind::Executor => {
                    let mut ecfg = ExecutorConfig::for_tests(Duration::from_millis(20));
                    ecfg.alloc_timing = true;
                    ecfg.panic_policy = PanicPolicy::StopExecutor;
                    exec_rb = Some(sup_rb.start_executor(ecfg).await.unwrap());
                }
                OwnerKind::Legacy => {
                    let stop = Arc::new(AtomicBool::new(false));
                    let stop_t = Arc::clone(&stop);
                    let sup_t = Arc::clone(&sup_rb);
                    let stats = Arc::new(ExecutorStats::default());
                    let rt = tokio::runtime::Handle::current();
                    jh_rb = Some(
                        std::thread::Builder::new()
                            .name("legacy-mode-rb-test".into())
                            .spawn(move || {
                                sup_t.run_legacy_control_loop_blocking(
                                    Duration::from_millis(20),
                                    stop_t,
                                    stats,
                                    rt,
                                    None,
                                );
                            })
                            .unwrap(),
                    );
                    stop_rb = Some(stop);
                }
            }
            let dl = Instant::now() + Duration::from_secs(2);
            while sup_rb.hub().latest().is_none() {
                assert!(Instant::now() < dl);
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
            let snap = sup_rb
                .run_mode_switch_session(
                    ModeSwitchStartRequest {
                        plan: ModeSwitchPlan::switch_axes(
                            &[(0, ControlMode::Impedance), (1, ControlMode::Impedance)],
                            ModeSwitchRecovery::RollbackToPrevious,
                        )
                        .unwrap(),
                        admission: sup_rb.capture_admission_stamp(),
                        operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
                        deadline: Instant::now() + Duration::from_secs(2),
                        client: ClientId(11),
                    },
                    Duration::from_secs(2),
                )
                .await
                .unwrap();
            assert_eq!(
                snap.progress,
                ModeProgress::RolledBack {
                    failed_joint: 1,
                    reason: ModeSwitchFailure::RangeMismatch,
                },
                "{owner:?}: partial switch must roll back cleanly"
            );
            let modes_rb = bus_rb.with(|s| [s.motors[0].ctrl_mode(), s.motors[1].ctrl_mode()]);
            assert_eq!(modes_rb, [damiao::CTRL_MODE_POS_VEL, damiao::CTRL_MODE_POS_VEL]);
            if owner == OwnerKind::Executor {
                assert_eq!(sup_rb.inner().control_thread_attempts(), 0);
            }
            if let Some(ex) = exec_rb {
                let _ = ex.emergency_disable_and_join();
            }
            if let Some(st) = stop_rb {
                sup_rb
                    .legacy_safe_hold_control()
                    .request_process_exit_disable(Instant::now() + Duration::from_secs(2));
                st.store(true, std::sync::atomic::Ordering::Release);
            }
            if let Some(jh) = jh_rb {
                let _ = jh.join();
            }
            snap
        };

        results.push((snap_mit.progress, snap_pv.progress, snap_rb.progress, modes_after));
    }
    assert_eq!(results[0], results[1], "Legacy and Executor energized switch parity");
}

/// Dual-owner test 3: Live lease enforcement (stale lease / epoch / operator required),
/// energized mid-write ambiguity -> same-release SAFE-HOLD (`SafeHoldReason::SessionUncertainState`),
/// SAFE-HOLD causal readback recovery of the uncertain axis (0 watchdog expiries, per-axis holds
/// across J1–J6, 0 `CMD_DISABLE` frames), terminal poll coherence after SAFE-HOLD transition,
/// and Legacy off-thread read-lock contention parity.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_mode_switch_authority_and_ambiguity_safe_hold_parity() {
    let mut results = Vec::new();
    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let h = OwnerHarness::start(owner, true, TIMEOUT_2000_COUNTS, true, true).await;

        let plan = ModeSwitchPlan::switch_axes(
            &[(0, ControlMode::Impedance)],
            ModeSwitchRecovery::FailClosed,
        )
        .unwrap();

        // 1. Authority check: stale lease generation refused identically on both owners.
        let mut stale_lease = h.sup.capture_admission_stamp();
        stale_lease.lease_gen = stale_lease.lease_gen.wrapping_add(77);
        let err_lease = h
            .sup
            .start_mode_switch_session(ModeSwitchStartRequest {
                plan,
                admission: stale_lease,
                operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
                deadline: Instant::now() + Duration::from_secs(2),
                client: ClientId(20),
            })
            .await
            .unwrap_err();
        assert!(
            matches!(err_lease, ConfigError::LeaseLost),
            "{owner:?}: expected LeaseLost, got {err_lease:?}"
        );

        // 2. Authority check: stale epoch refused identically on both owners.
        let mut stale_stamp = h.sup.capture_admission_stamp();
        stale_stamp.estop_epoch = stale_stamp.estop_epoch.wrapping_add(99);
        let err_epoch = h
            .sup
            .start_mode_switch_session(ModeSwitchStartRequest {
                plan,
                admission: stale_stamp,
                operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
                deadline: Instant::now() + Duration::from_secs(2),
                client: ClientId(20),
            })
            .await
            .unwrap_err();
        assert!(
            matches!(err_epoch, ConfigError::EpochMismatch { .. }),
            "{owner:?}: expected EpochMismatch, got {err_epoch:?}"
        );

        // 3. Authority check: operator required with denied grant.
        let err_op = h
            .sup
            .start_mode_switch_session(ModeSwitchStartRequest {
                plan,
                admission: h.sup.capture_admission_stamp(),
                operator: OperatorRequirement::Required(OperatorGrant::DENIED),
                deadline: Instant::now() + Duration::from_secs(2),
                client: ClientId(20),
            })
            .await
            .unwrap_err();
        let op_is_rejected = matches!(
            err_op,
            ConfigError::CalibPrecondition {
                code: "OPERATOR_REQUIRED",
                ..
            }
        );
        assert!(op_is_rejected, "{owner:?}: expected OPERATOR_REQUIRED, got {err_op:?}");

        // 4. Plan validation check: Duration::MAX timeout refused at admission (Major 7).
        let mut bad_plan = plan;
        bad_plan.per_axis_timeout = Duration::MAX;
        let err_bad = h
            .sup
            .start_mode_switch_session(ModeSwitchStartRequest {
                plan: bad_plan,
                admission: h.sup.capture_admission_stamp(),
                operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
                deadline: Instant::now() + Duration::from_secs(2),
                client: ClientId(20),
            })
            .await
            .unwrap_err();
        assert!(
            matches!(
                err_bad,
                ConfigError::CalibPrecondition {
                    code: "INVALID_COMMAND",
                    ..
                }
            ),
            "{owner:?}: expected INVALID_COMMAND for overflowing timeout, got {err_bad:?}"
        );

        // 5. Energized mid-write ambiguity (J1 drops write ACK) -> same-release SAFE-HOLD
        //    (`SafeHoldReason::SessionUncertainState`), SAFE-HOLD causally probes J1 CTRL_MODE
        //    and resumes holding ALL 6 axes (0 watchdog expiries, 0 CMD_DISABLE frames).
        //    On Legacy, also hold an off-thread read guard right after admission to prove Blocker 7
        //    single-guard handoff completes without stalling on supervisor contention!
        h.bus.clear_log();
        h.bus.with(|s| s.motors[0].no_write_ack = true);
        let sid = h
            .sup
            .start_mode_switch_session(ModeSwitchStartRequest {
                plan,
                admission: h.sup.capture_admission_stamp(),
                operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
                deadline: Instant::now() + Duration::from_secs(2),
                client: ClientId(20),
            })
            .await
            .unwrap();

        let sh = match owner {
            OwnerKind::Executor => h.exec.as_ref().unwrap().safe_hold_control(),
            OwnerKind::Legacy => h.sup.legacy_safe_hold_control(),
        };
        // R2 Major 4 proof: on Legacy, contend `sup.inner().read()` off-thread while SAFE-HOLD
        // activates so we prove zero-contention single-guard handoff without post-tick lock stalls!
        let contend_stop = Arc::new(AtomicBool::new(false));
        let contend_task = if owner == OwnerKind::Legacy {
            let sup_c = Arc::clone(&h.sup);
            let stop_c = Arc::clone(&contend_stop);
            Some(tokio::spawn(async move {
                while !stop_c.load(std::sync::atomic::Ordering::Acquire) {
                    let _g = sup_c.inner().read().await;
                    tokio::time::sleep(Duration::from_millis(2)).await;
                }
            }))
        } else {
            None
        };
        let dl = Instant::now() + Duration::from_secs(3);
        while (!sh.is_active() || sh.ticks() < 4) && Instant::now() < dl {
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        contend_stop.store(true, std::sync::atomic::Ordering::Release);
        if let Some(t) = contend_task {
            let _ = t.await;
        }
        assert!(sh.is_active(), "{owner:?}: energized ambiguity must enter SAFE-HOLD");
        assert_eq!(
            sh.reason(),
            SafeHoldReason::SessionUncertainState,
            "{owner:?}: SAFE-HOLD reason must be SessionUncertainState"
        );
        assert!(sh.hold_tx_count() >= 2, "{owner:?}: SAFE-HOLD must emit hold frames");
        assert!(h.sup.internal_fault_fenced(), "{owner:?}: internal fault fence must be latched");

        // Major 6 proof: polling the session AFTER SAFE-HOLD transition still returns the final
        // Ambiguous terminal snapshot from StateHub!
        let term_snap = h
            .sup
            .poll_mode_switch_session(sid)
            .await
            .unwrap()
            .expect("terminal snapshot must remain readable from StateHub after SAFE-HOLD");
        assert!(!term_snap.running);
        assert!(term_snap.progress.is_ambiguous(), "{owner:?}: expected Ambiguous, got {:?}", term_snap.progress);

        // Blocker 5 + Major 8 proof: per-axis hold frames observed on ALL 6 arm axes (including
        // J1 recovered causally via SAFE-HOLD REG_READ(RID_CTRL_MODE)!), 0 CMD_DISABLE frames,
        // 0 watchdog expiries, all motors enabled.
        let (all_enabled, disables, expiries, per_axis_holds, saw_readback) = h.bus.with(|s| {
            let dis = s.tx_log.iter().filter(|f| f.data == damiao::CMD_DISABLE).count();
            let exp: usize = s.motors.iter().map(|m| m.watchdog_expiry_count).sum();
            let mut per_axis = [0usize; 6];
            for f in &s.tx_log {
                if (0x101..=0x106).contains(&f.id) {
                    per_axis[(f.id - 0x101) as usize] += 1;
                } else if (1..=6).contains(&f.id) && f.data != damiao::CMD_ENABLE {
                    per_axis[(f.id - 1) as usize] += 1;
                }
            }
            let rb = s.tx_log.iter().any(|f| {
                f.id == 0x7FF && f.data[0] == 1 && f.data[2] == damiao::REG_READ && f.data[3] == damiao::RID_CTRL_MODE
            });
            (s.motors.iter().all(|m| m.enabled), dis, exp, per_axis, rb)
        });
        assert!(all_enabled, "{owner:?}: all motors must remain enabled in SAFE-HOLD away from rest");
        assert_eq!(disables, 0, "{owner:?}: 0 CMD_DISABLE frames on SAFE-HOLD escalation");
        assert_eq!(expiries, 0, "{owner:?}: 0 watchdog expiries (SAFE-HOLD causally recovered J1)");
        assert!(saw_readback, "{owner:?}: SAFE-HOLD must causally probe REG_READ(RID_CTRL_MODE) for uncertain J1");
        for (j, count) in per_axis_holds.iter().enumerate() {
            assert!(*count >= 2, "{owner:?}: J{} must receive hold frames (got {count})", j + 1);
        }
        if owner == OwnerKind::Executor {
            assert_eq!(h.sup.inner().control_thread_attempts(), 0, "0 SupervisorLock attempts on armd-control");
        }

        results.push((true, op_is_rejected, sh.reason(), disables, term_snap.progress.is_ambiguous()));
        h.shutdown(true).await;
    }
    assert_eq!(results[0], results[1], "Legacy and Executor authority & SAFE-HOLD parity");
}

/// R4 Blocker 1 + Blocker 2 + Blocker 3 regression test:
/// 1. When a SAFE-HOLD recovery `REG_READ(RID_CTRL_MODE)` reply is lost at 50 Hz (20 ms ticks),
///    `request_mode_readback_budgeted` retries on the very next 20 ms tick (15 ms retry window)
///    and recovers J1 well before Damiao's 100 ms `TIMEOUT = 2000` watchdog expires.
/// 2. Manually ticked Legacy (`sup.tick(0.02)` without threaded `legacy_safe_hold_handoff`) after
///    `ModeSwitch` requests SAFE-HOLD still holds all confirmed axes, causally probes/recovers
///    uncertain J1, and emits 0 `CMD_DISABLE` frames.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn safe_hold_lost_recovery_reply_retries_next_tick_and_manual_legacy_safe_hold_keeps_holding() {
    // Part 1: FakeBus + MockClock at 50 Hz — drop write ACK + drop 1st SAFE-HOLD REG_READ reply.
    let clock = MockClock::new();
    let cfg = ArmConfig::rebot_dm();
    let bus = configure_damiao_bus(Some(clock.clone()), Duration::from_micros(200), true, TIMEOUT_2000_COUNTS);
    let mut drv = build_damiao_driver(bus.clone(), &cfg, 16);
    enable_driver_sync(&mut drv, &bus, Some(&clock));

    // Confirm J1 in PosVel, then latch J1 uncertain via a lost REG_WRITE ACK.
    let plan_j1 =
        ModeSwitchPlan::switch_axes(&[(0, ControlMode::Impedance)], ModeSwitchRecovery::FailClosed)
            .unwrap();
    let mut ms = ModeSwitch::new(plan_j1);
    bus.with(|s| {
        s.motors[0].no_write_ack = true;
        s.immediate_replies = false;
    });
    let mut rel = 1u64;
    let mut entered_ambiguous = false;
    for _ in 0..8 {
        clock.advance(PERIOD_50HZ);
        bus.with(|s| s.push_all_feedback());
        drv.begin_control_release(rel);
        rel += 1;
        let _ = drv.read_joint_states();
        let now = clock.now();
        let mut gate = || Ok(());
        let mut adapter = armd::motor_mode::ArmDriverModeAdapter::new(&mut drv, &mut gate);
        let p = ms.step_driver(&mut adapter, ReleaseBudget::new(now, now + Duration::from_millis(12), PERIOD_50HZ, 16));
        if p.is_ambiguous() {
            entered_ambiguous = true;
            break;
        }
    }
    assert!(entered_ambiguous, "dropped REG_WRITE ACK must terminate ModeSwitch as Ambiguous");
    assert_ne!(drv.mode_write_uncertain_mask() & 1, 0, "J1 must be mode-uncertain");

    // Re-enable register replies (realistic CAN RTT: replies arrive before the next 20 ms tick),
    // then simulate SAFE-HOLD tick 1 whose REG_READ reply is LOST on the wire.
    let hold_targets: Vec<armd::driver::JointTarget> = [0.30, -0.25, 0.40, 0.15, -0.15, 0.10, -0.14]
        .into_iter()
        .map(armd::driver::JointTarget::position)
        .collect();
    let reg_reads_before = bus.with(|s| {
        s.motors[0].no_write_ack = false;
        s.immediate_replies = true;
        s.tx_log
            .iter()
            .filter(|f| f.id == 0x7FF && f.data[0] == 1 && f.data[2] == damiao::REG_READ && f.data[3] == damiao::RID_CTRL_MODE)
            .count()
    });

    // Tick 1 (+20 ms): sends recovery REG_READ #1, then drop its reply from bus RX/pending queues!
    clock.advance(PERIOD_50HZ);
    bus.with(|s| s.push_all_feedback());
    drv.begin_control_release(rel);
    rel += 1;
    let _ = drv.read_joint_states();
    let rep1 = drv.write_measured_hold_masked(0b11_1111, &hold_targets).unwrap();
    assert!(matches!(rep1.joint_status[0], armd::driver::JointHoldStatus::UnheldNoVerifiedMode));
    assert!(drv.request_mode_readback_budgeted(1), "tick 1 must send recovery REG_READ #1");
    bus.with(|s| {
        // Drop the reply to probe #1 (master_id reply with REG_READ/RID_CTRL_MODE) to simulate a
        // lost recovery CAN frame on the wire.
        let is_mode_read_reply = |f: &CanFrame| {
            f.dlc == 8 && f.data[2] == damiao::REG_READ && f.data[3] == damiao::RID_CTRL_MODE
        };
        s.rx.retain(|f| !is_mode_read_reply(f));
        s.pending.retain(|f| !is_mode_read_reply(f));
    });

    // Tick 2 (+20 ms): retry interval (15 ms) elapsed -> MUST immediately retry REG_READ #2 on the
    // very next 50 Hz release instead of stalling 50 ms!
    clock.advance(PERIOD_50HZ);
    bus.with(|s| {
        s.run_watchdogs();
        s.push_all_feedback();
    });
    drv.begin_control_release(rel);
    rel += 1;
    let _ = drv.read_joint_states();
    let _ = drv.write_measured_hold_masked(0b11_1111, &hold_targets).unwrap();
    assert!(
        drv.request_mode_readback_budgeted(1),
        "tick 2 (+20 ms >= 15 ms) MUST immediately retry recovery REG_READ #2 after lost reply #1"
    );
    let reg_reads_after_tick2 = bus.with(|s| {
        s.tx_log
            .iter()
            .filter(|f| f.id == 0x7FF && f.data[0] == 1 && f.data[2] == damiao::REG_READ && f.data[3] == damiao::RID_CTRL_MODE)
            .count()
    });
    assert_eq!(
        reg_reads_after_tick2 - reg_reads_before,
        2,
        "expected exactly 2 recovery REG_READ probes across 2 ticks"
    );

    // Tick 3 (+20 ms, t = 80 ms total from last hold): reply #2 clears J1 uncertainty and tick 3
    // emits confirmed-mode hold on J1 (< 100 ms Damiao TIMEOUT = 2000 -> 0 watchdog expiries!).
    clock.advance(PERIOD_50HZ);
    bus.with(|s| {
        s.run_watchdogs();
        s.push_all_feedback();
    });
    drv.begin_control_release(rel);
    let _ = drv.read_joint_states();
    assert_eq!(drv.mode_write_uncertain_mask() & 1, 0, "J1 uncertainty must clear on tick 3");
    let rep3 = drv.write_measured_hold_masked(0b11_1111, &hold_targets).unwrap();
    assert!(
        matches!(rep3.joint_status[0], armd::driver::JointHoldStatus::Held | armd::driver::JointHoldStatus::HeldStaleTarget),
        "J1 must resume confirmed-mode hold on tick 3, got {:?}",
        rep3.joint_status[0]
    );
    bus.with(|s| s.run_watchdogs());
    let expiries = bus.with(|s| s.motors.iter().map(|m| m.watchdog_expiry_count).sum::<usize>());
    assert_eq!(expiries, 0, "0 watchdog expiries even with lost ACK + lost first recovery reply");

    // Part 2: Manually ticked Legacy (`sup.tick(0.02)` without threaded handoff loop) after
    // `ModeSwitch` requests SAFE-HOLD must keep holding confirmed axes and recover uncertain J1.
    let mut cfg2 = cfg.clone();
    cfg2.control_frequency_hz = 50;
    let bus2 = configure_damiao_bus(None, Duration::ZERO, true, TIMEOUT_2000_COUNTS);
    let mut drv2 = build_damiao_driver(bus2.clone(), &cfg2, 16);
    bus2.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let _ = drv2.check_mapping_ranges();
    let _ = drv2.read_joint_states();
    let sup = ArmSupervisor::new(cfg2).with_driver(Box::new(drv2));
    sup.set_lease_enabled(false).await;
    sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
    bus2.with(|s| {
        s.immediate_replies = false;
        s.push_all_feedback();
    });
    let _ = sup.tick(0.02).await;

    // Drop write ACK so ModeSwitch enters Ambiguous and sets `sessions.is_safe_hold_requested()`.
    bus2.with(|s| {
        s.motors[0].no_write_ack = true;
        s.immediate_replies = false;
    });
    let sid = sup
        .start_mode_switch_session(ModeSwitchStartRequest {
            plan: plan_j1,
            admission: sup.capture_admission_stamp(),
            operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
            deadline: Instant::now() + Duration::from_secs(2),
            client: ClientId(21),
        })
        .await
        .unwrap();
    let mut saw_ambig = false;
    for _ in 0..8 {
        bus2.with(|s| s.push_all_feedback());
        let _ = sup.tick(0.02).await;
        if let Some(s) = sup.poll_mode_switch_session(sid).await.unwrap() {
            if !s.running {
                assert!(s.progress.is_ambiguous(), "expected Ambiguous, got {:?}", s.progress);
                saw_ambig = true;
                break;
            }
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    assert!(saw_ambig, "manual Legacy ModeSwitch must enter Ambiguous on dropped write ACK");

    // Re-enable register replies and continue manual `sup.tick(0.02)` calls; inline SAFE-HOLD
    // fallback must hold J2..J6 AND causally recover + hold J1 with 0 CMD_DISABLE.
    let tx_before_manual_sh = bus2.with(|s| {
        s.motors[0].no_write_ack = false;
        s.immediate_replies = false;
        s.tx_log.len()
    });
    for _ in 0..4 {
        bus2.with(|s| s.push_all_feedback());
        let _ = sup.tick(0.02).await;
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    let (disables, expiries, per_axis_after, saw_rb) = bus2.with(|s| {
        let dis = s.tx_log.iter().filter(|f| f.data == damiao::CMD_DISABLE).count();
        let exp: usize = s.motors.iter().map(|m| m.watchdog_expiry_count).sum();
        let mut per_axis = [0usize; 6];
        for f in &s.tx_log[tx_before_manual_sh..] {
            if (0x101..=0x106).contains(&f.id) {
                per_axis[(f.id - 0x101) as usize] += 1;
            } else if (1..=6).contains(&f.id) && f.data != damiao::CMD_ENABLE {
                per_axis[(f.id - 1) as usize] += 1;
            }
        }
        let rb = s.tx_log[tx_before_manual_sh..].iter().any(|f| {
            f.id == 0x7FF && f.data[0] == 1 && f.data[2] == damiao::REG_READ && f.data[3] == damiao::RID_CTRL_MODE
        });
        (dis, exp, per_axis, rb)
    });
    assert_eq!(disables, 0, "manual Legacy SAFE-HOLD must emit 0 CMD_DISABLE frames");
    assert_eq!(expiries, 0, "manual Legacy SAFE-HOLD must suffer 0 watchdog expiries");
    assert!(saw_rb, "manual Legacy SAFE-HOLD must causally probe REG_READ(RID_CTRL_MODE) for J1");
    for (j, count) in per_axis_after.iter().enumerate() {
        assert!(*count >= 1, "manual Legacy SAFE-HOLD must emit holds on J{} (got {count})", j + 1);
    }
}

// ---------------------------------------------------------------------------
// 6. Review round 5 regressions
// ---------------------------------------------------------------------------

/// Runs one 50 Hz-style release on a raw Damiao driver with a caller-supplied pre-TX gate.
fn step_raw_with_gate(
    drv: &mut DamiaoArmDriver<FakeBus>,
    ms: &mut ModeSwitch,
    clock: &MockClock,
    budget_of: impl Fn(Instant, Instant) -> ReleaseBudget,
    period: Duration,
    seq: u64,
    gate: &mut dyn FnMut() -> Result<(), &'static str>,
) -> ModeProgress {
    let tick_start = clock.now();
    drv.begin_control_release(seq);
    let cutoff = tick_start + period.saturating_sub(Duration::from_millis(3));
    drv.set_tx_release_deadline(Some(cutoff));
    let _ = drv.read_joint_states();
    let budget = budget_of(clock.now(), cutoff);
    let prog = ms.step_with_gate(drv, gate, budget);
    drv.set_tx_release_deadline(None);
    prog
}

/// Blocker 1: a stop that is only PENDING (not yet applied by the owner) revokes the next register
/// TX after J1 has already switched forward. The pre-TX gate code `"STOPPED"` is not proof that the
/// stop was applied, so the partial energized switch must escalate to SAFE-HOLD (`Ambiguous`)
/// instead of reporting `StoppedRecovered`.
#[test]
fn pending_stop_after_first_axis_confirms_is_not_stopped_recovered() {
    let clock = MockClock::new();
    let cfg = ArmConfig::rebot_dm();
    let bus = configure_damiao_bus(Some(clock.clone()), Duration::from_micros(500), true, TIMEOUT_2000_COUNTS);
    let mut drv = build_damiao_driver(bus.clone(), &cfg, 8);
    enable_driver_sync(&mut drv, &bus, Some(&clock));
    let plan = ModeSwitchPlan::switch_axes(
        &[(0, ControlMode::Impedance), (1, ControlMode::Impedance)],
        ModeSwitchRecovery::RollbackToPrevious,
    )
    .unwrap();
    let mut ms = ModeSwitch::new(plan);
    // The stop becomes pending once J1 runs in MIT on the wire; the next register TX (J2's first)
    // is revoked by the live gate before the owner applies the stop.
    let revoked = std::cell::Cell::new(0u32);
    let gate_bus = bus.clone();
    let mut prog = ModeProgress::Pending { wake: None };
    let mut seq = 1u64;
    for _ in 0..40 {
        clock.advance(PERIOD_50HZ);
        bus.with(|s| s.run_watchdogs());
        seq += 1;
        let mut gate = || {
            if gate_bus.with(|s| s.motors[0].ctrl_mode()) == damiao::CTRL_MODE_MIT {
                revoked.set(revoked.get() + 1);
                Err("STOPPED")
            } else {
                Ok(())
            }
        };
        prog = step_raw_with_gate(
            &mut drv,
            &mut ms,
            &clock,
            |now, cutoff| ReleaseBudget::new(now, cutoff, PERIOD_50HZ, 8),
            PERIOD_50HZ,
            seq,
            &mut gate,
        );
        if prog.is_terminal() {
            break;
        }
    }
    pending_stop_race_asserts(&bus, &revoked, prog);
}

fn pending_stop_race_asserts(bus: &FakeBus, revoked: &std::cell::Cell<u32>, prog: ModeProgress) {
    let is_mode_write_to = |log: &[CanFrame], id: u8| {
        log.iter().any(|f| {
            f.id == 0x7FF && f.data[0] == id && f.data[2] == damiao::REG_WRITE && f.data[3] == damiao::RID_CTRL_MODE
        })
    };
    let log = bus.tx_log();
    assert!(is_mode_write_to(&log, 1), "J1 must have switched forward");
    assert!(!is_mode_write_to(&log, 2), "J2 register TX must have been revoked");
    assert!(revoked.get() >= 1, "the live gate must have revoked a register TX");
    assert!(
        prog.is_ambiguous(),
        "pending (unapplied) stop with an energized partial switch must escalate to SAFE-HOLD, got {prog:?}"
    );
}

/// Blocker 1 control: the same race with owner proof that the stop was applied recovers cleanly.
#[test]
fn applied_stop_proof_after_first_axis_confirms_is_stopped_recovered() {
    let clock = MockClock::new();
    let cfg = ArmConfig::rebot_dm();
    let bus = configure_damiao_bus(Some(clock.clone()), Duration::from_micros(500), true, TIMEOUT_2000_COUNTS);
    let mut drv = build_damiao_driver(bus.clone(), &cfg, 8);
    enable_driver_sync(&mut drv, &bus, Some(&clock));
    let plan = ModeSwitchPlan::switch_axes(
        &[(0, ControlMode::Impedance), (1, ControlMode::Impedance)],
        ModeSwitchRecovery::RollbackToPrevious,
    )
    .unwrap();
    let mut ms = ModeSwitch::new(plan);
    let gate_bus = bus.clone();
    let mut prog = ModeProgress::Pending { wake: None };
    for seq in 2..42u64 {
        clock.advance(PERIOD_50HZ);
        bus.with(|s| s.run_watchdogs());
        let mut gate = || {
            if gate_bus.with(|s| s.motors[0].ctrl_mode()) == damiao::CTRL_MODE_MIT {
                Err("STOPPED")
            } else {
                Ok(())
            }
        };
        let applied = || gate_bus.with(|s| s.motors[0].ctrl_mode()) == damiao::CTRL_MODE_MIT;
        drv.begin_control_release(seq);
        let cutoff = clock.now() + Duration::from_millis(17);
        drv.set_tx_release_deadline(Some(cutoff));
        let _ = drv.read_joint_states();
        let budget = ReleaseBudget::new(clock.now(), cutoff, PERIOD_50HZ, 8);
        prog = ms.step_with_gate_and_stop_proof(&mut drv, &mut gate, &applied, budget);
        drv.set_tx_release_deadline(None);
        if prog.is_terminal() {
            break;
        }
    }
    assert_eq!(prog, ModeProgress::StoppedRecovered { reason: ModeSwitchFailure::Stopped });
}

/// Blocker 2: an energized axis whose cached mode is Impedance answers a causal ConfirmOnly
/// readback with PositionVelocity (the requested mode). The reply contradicts the cached mode, so
/// it must not be published: the axis is fenced and the session escalates to SAFE-HOLD.
#[test]
fn confirm_only_reply_contradicting_cached_mode_on_energized_axis_escalates() {
    let clock = MockClock::new();
    let cfg = ArmConfig::rebot_dm();
    let bus = configure_damiao_bus(Some(clock.clone()), Duration::from_micros(500), true, TIMEOUT_2000_COUNTS);
    let mut raw_drv = build_damiao_driver(bus.clone(), &cfg, 8);
    enable_driver_sync(&mut raw_drv, &bus, Some(&clock));
    let mut ex_drv = ExclusiveDriver::new(Box::new(raw_drv));
    // Driver cache: J1 verified Impedance; wire: J1 is POS_VEL.
    ex_drv.get_mut().as_mut().set_axis_confirmed_mode(0, Some(ControlMode::Impedance));
    let mut plan = ModeSwitchPlan::empty(ModeSwitchIntent::ConfirmOnly);
    plan.axes[0] = AxisTarget::new(0, ControlMode::PositionVelocity);
    plan.len = 1;
    let mut ms = ModeSwitch::new(plan);
    let mut prog = ModeProgress::Pending { wake: None };
    let mut seq = 1u64;
    for _ in 0..12 {
        clock.advance(PERIOD_50HZ);
        bus.with(|s| s.run_watchdogs());
        seq += 1;
        prog = step_release_on_exclusive(&mut ex_drv, &mut ms, &clock, PERIOD_50HZ, 8, seq);
        if prog.is_terminal() {
            break;
        }
    }
    assert_eq!(
        prog,
        ModeProgress::Ambiguous {
            failed_joint: 0,
            reason: ModeSwitchFailure::ReadbackMismatch,
        }
    );
    assert_eq!(ex_drv.get_ref().axis_confirmed_mode(0), None, "contradicting reply must not be published");
    assert_ne!(ex_drv.get_ref().mode_write_uncertain_mask() & 1, 0, "J1 must be fenced");
}

/// Major 1: 20 ms accepted gap, 10 ms release, 15 ms proactive interval, 5 ms delivery margin.
/// Every energized watchdog axis (all outside the J7-only plan) must get a hold each release so
/// no control gap exceeds `gap - margin`; a fixed 15 ms interval skips the 10 ms hold and lands
/// the next one past the deadline.
#[test]
fn watchdog_hold_due_time_derives_from_gap_period_and_margin_for_axes_outside_plan() {
    let clock = MockClock::new();
    let cfg = ArmConfig::rebot_dm();
    let bus = configure_damiao_bus(Some(clock.clone()), Duration::from_micros(200), false, TIMEOUT_2000_COUNTS);
    let mut drv = build_damiao_driver(bus.clone(), &cfg, 8);
    enable_driver_sync(&mut drv, &bus, Some(&clock));
    let period = Duration::from_millis(10);
    let gap = Duration::from_millis(20);
    let margin = Duration::from_millis(5);
    // J7 never answers, so the session stays pending for the whole window.
    bus.with(|s| s.motors[6].silent = true);
    let mut plan = ModeSwitchPlan::empty(ModeSwitchIntent::ConfirmOnly);
    plan.axes[0] = AxisTarget::new(6, ControlMode::PositionVelocity);
    plan.len = 1;
    plan.per_axis_timeout = Duration::from_millis(500);
    let mut ms = ModeSwitch::new(plan);
    let mut seq = 1u64;
    // Warm-up release seeds a hold on every arm axis.
    let budget_of = |now: Instant, cutoff: Instant| {
        let mut b = ReleaseBudget::new(now, cutoff, period, 8);
        b.max_watchdog_gap = gap;
        b.watchdog_proactive_interval = Duration::from_millis(15);
        b.hold_delivery_margin = margin;
        b
    };
    let mut allow = || Ok(());
    let _ = step_raw_with_gate(&mut drv, &mut ms, &clock, budget_of, period, seq, &mut allow);
    let window_start = clock.now();
    bus.clear_log();
    let mut pending_ticks = 0;
    for _ in 0..8 {
        clock.advance(period);
        bus.with(|s| s.run_watchdogs());
        seq += 1;
        let p = step_raw_with_gate(&mut drv, &mut ms, &clock, budget_of, period, seq, &mut allow);
        if p.is_terminal() {
            break;
        }
        pending_ticks += 1;
    }
    let end = clock.now();
    assert!(pending_ticks >= 6, "session must stay pending across the window ({pending_ticks})");
    let (tx_log, tx_times) = bus.with(|s| (s.tx_log.clone(), s.tx_times.clone()));
    let mut last = [window_start; 6];
    let mut max_gap = [Duration::ZERO; 6];
    for (f, t) in tx_log.iter().zip(tx_times.iter()) {
        let Some(t) = *t else { continue };
        if (0x101..=0x106).contains(&f.id) {
            let j = (f.id - 0x101) as usize;
            max_gap[j] = max_gap[j].max(t.saturating_duration_since(last[j]));
            last[j] = t;
        }
    }
    for j in 0..6 {
        max_gap[j] = max_gap[j].max(end.saturating_duration_since(last[j]));
        assert!(
            max_gap[j] <= gap - margin,
            "J{} hold gap {:?} exceeds gap - margin {:?} (all: {max_gap:?})",
            j + 1,
            max_gap[j],
            gap - margin
        );
    }
}

/// Major 2: a caller that never polls (falls behind while later sessions evict the bounded
/// snapshot history) still receives its session terminal through the shared P-F1 router.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn mode_switch_terminal_reaches_caller_that_falls_behind_via_router_credit() {
    use armd::rt_core::broker::{DeliveryDecision, Outcome, Reply};
    let h = OwnerHarness::start(OwnerKind::Executor, true, 0, false, false).await;
    let exec = h.exec.clone().unwrap();
    let delivered: Arc<std::sync::Mutex<Vec<(ClientId, Reply)>>> = Arc::default();
    let d_cb = Arc::clone(&delivered);
    let sink = Arc::new(move |client: ClientId, reply: Reply| {
        d_cb.lock().unwrap().push((client, reply));
        DeliveryDecision::Delivered
    });
    let caller = ClientId(707);
    exec.open_connection_sync(707).unwrap().unwrap();
    let _tok = exec.register_connection_reply_sink(caller, sink, Some(4)).unwrap();

    let plan = ModeSwitchPlan::confirm_gripper(6, ControlMode::PositionForce).unwrap();
    let sid1 = h
        .sup
        .start_mode_switch_session(ModeSwitchStartRequest {
            plan,
            admission: h.sup.capture_admission_stamp(),
            operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
            deadline: Instant::now() + Duration::from_secs(1),
            client: caller,
        })
        .await
        .unwrap();
    // The caller falls behind: three later sessions run to completion first.
    for _ in 0..3 {
        let dl = Instant::now() + Duration::from_secs(3);
        loop {
            match h
                .sup
                .confirm_gripper_mode_session(
                    6,
                    ControlMode::PositionForce,
                    h.sup.capture_admission_stamp(),
                    OperatorGrant::unenforced(),
                    Duration::from_secs(1),
                )
                .await
            {
                Ok(s) => {
                    assert_eq!(s.progress, ModeProgress::Confirmed);
                    break;
                }
                Err(ConfigError::Busy(_)) => {
                    assert!(Instant::now() < dl, "previous session never retired");
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
                Err(e) => panic!("later session failed: {e:?}"),
            }
        }
    }
    assert!(
        h.sup.poll_mode_switch_session(sid1).await.unwrap().is_none(),
        "sid1 snapshot must have been evicted from the bounded history"
    );
    exec.pump_broker_replies();
    let got: Vec<_> = delivered.lock().unwrap().clone();
    let terminals: Vec<_> = got
        .iter()
        .filter(|(c, r)| *c == caller && matches!(r, Reply::Terminal { .. }))
        .collect();
    assert_eq!(terminals.len(), 1, "caller must receive exactly one terminal: {got:?}");
    assert!(matches!(terminals[0].1, Reply::Terminal { outcome: Outcome::Completed, .. }));
    assert_eq!(exec.router_global_reserved(), 0, "terminal credit recovered after delivery");
    h.shutdown(false).await;
}

/// Minor: SAFE-HOLD with fresh feedback but an axis that has no verified mode (unheld) must keep
/// `holding_unconfirmed` set.
#[test]
fn safe_hold_keeps_holding_unconfirmed_while_axis_unheld_without_verified_mode() {
    use armd::rt_core::safe_hold::{SafeHoldBuffers, SafeHoldControl};
    use armd::supervisor::SupervisorInner;
    let clock = MockClock::new();
    let cfg = ArmConfig::rebot_dm();
    let bus = configure_damiao_bus(Some(clock.clone()), Duration::from_micros(200), true, TIMEOUT_2000_COUNTS);
    let mut drv = build_damiao_driver(bus.clone(), &cfg, 16);
    enable_driver_sync(&mut drv, &bus, Some(&clock));
    let mut core = SupervisorInner::new(cfg.clone());
    core.lease_enabled = false;
    core.measured_positions = vec![0.30, -0.25, 0.40, 0.15, -0.15, 0.10, -0.14];
    core.feedback_numeric_valid = true;
    let period = PERIOD_50HZ;
    let mut bufs = SafeHoldBuffers::from_config(&cfg, period);
    bufs.mirror_from_core(&core);
    bufs.seed_on_entry(&core);
    bufs.prepare_driver_on_entry(&mut drv);
    let ctrl = SafeHoldControl::new();
    ctrl.mark_active(SafeHoldReason::ControlPanic);
    // J1's mode is unknown (e.g. lost REG_WRITE ACK) and its CTRL_MODE probe is never answered.
    drv.latch_axis_mode_uncertain(0);
    bus.with(|s| s.motors[0].ignore_reg_writes = true);
    for _ in 0..3 {
        clock.advance(period);
        bus.with(|s| {
            s.push_all_feedback();
            // Drop any CTRL_MODE read reply for J1 so it stays unverified.
            let is_j1_mode_reply = |f: &CanFrame| {
                f.dlc == 8
                    && f.data[0] == 1
                    && f.data[2] == damiao::REG_READ
                    && f.data[3] == damiao::RID_CTRL_MODE
            };
            s.rx.retain(|f| !is_j1_mode_reply(f));
            s.pending.retain(|f| !is_j1_mode_reply(f));
        });
        let mut now_fn = || clock.now();
        let dl = clock.now() + period;
        bufs.step_driver(&mut drv, &ctrl, dl, &mut now_fn);
        assert_ne!(drv.mode_write_uncertain_mask() & 1, 0, "J1 must stay unverified");
        assert!(
            ctrl.holding_unconfirmed(),
            "SAFE-HOLD must not report confirmed holding while J1 is unheld"
        );
    }
}

// ---------------------------------------------------------------------------
// 7. Review round 6 regressions
// ---------------------------------------------------------------------------

/// Round 6 B1: an ordinary STOP applied by the owner mid-switch (production `urgent_stop`, no
/// E-stop, no fault) is applied-stop proof through the production callback: J1 already switched
/// forward and awaits post-confirm feedback, so the session recovers (`StoppedRecovered`) and
/// does not escalate to SAFE-HOLD.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn ordinary_applied_stop_mid_switch_is_stopped_recovered_via_production_proof() {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.control_frequency_hz = 50;
    let bus = configure_damiao_bus(None, Duration::ZERO, true, TIMEOUT_2000_COUNTS);
    let mut drv = build_damiao_driver(bus.clone(), &cfg, 16);
    bus.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let _ = drv.check_mapping_ranges();
    let _ = drv.read_joint_states();
    let sup = ArmSupervisor::new(cfg).with_driver(Box::new(drv));
    sup.set_lease_enabled(true).await;
    let lease_gen = sup.acquire_lease("client-1").await.unwrap();
    sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
    bus.with(|s| {
        s.immediate_replies = false;
        s.push_all_feedback();
    });
    let _ = sup.tick(0.02).await;
    // J1 accepts the mode write, then sends no post-confirm feedback: it stays switched forward.
    bus.with(|s| s.motors[0].mute_feedback_after_mode_ack = true);
    let mut plan =
        ModeSwitchPlan::switch_axes(&[(0, ControlMode::Impedance)], ModeSwitchRecovery::FailClosed)
            .unwrap();
    plan.per_axis_timeout = Duration::from_secs(2);
    let sid = sup
        .start_mode_switch_session(ModeSwitchStartRequest {
            plan,
            admission: sup.capture_admission_stamp(),
            operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
            deadline: Instant::now() + Duration::from_secs(4),
            client: ClientId(31),
        })
        .await
        .unwrap();
    let mut acked_ticks = 0;
    for _ in 0..30 {
        bus.with(|s| s.push_all_feedback());
        let _ = sup.tick(0.02).await;
        let snap = sup.poll_mode_switch_session(sid).await.unwrap().unwrap();
        assert!(snap.running, "switch ended early: {:?}", snap.progress);
        if bus.with(|s| s.motors[0].ctrl_mode()) == damiao::CTRL_MODE_MIT {
            acked_ticks += 1;
            if acked_ticks >= 3 {
                break;
            }
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    assert!(acked_ticks >= 3, "J1 must have switched forward");
    // Ordinary authorized STOP (`/api/v1/motion/stop` semantics): no E-stop, no fault.
    let out = sup
        .urgent_stop(armd::rt_core::urgent::UrgentStop {
            connection: armd::model::HolderLabel::new("client-1"),
            lease_generation: lease_gen,
            epoch: Some(sup.estop_epoch()),
            request_id: 1,
            request_id_valid: true,
            body_error: None,
        })
        .await
        .unwrap();
    assert!(matches!(out, armd::rt_core::urgent::UrgentStopOutcome::Applied { .. }), "{out:?}");
    let mut done = None;
    for _ in 0..10 {
        bus.with(|s| s.push_all_feedback());
        let _ = sup.tick(0.02).await;
        if let Some(s) = sup.poll_mode_switch_session(sid).await.unwrap() {
            if !s.running {
                done = Some(s.progress);
                break;
            }
        }
    }
    assert_eq!(
        done,
        Some(ModeProgress::StoppedRecovered { reason: ModeSwitchFailure::Stopped }),
        "an ordinary applied STOP must not end the switch Ambiguous (spurious SAFE-HOLD)"
    );
}

/// Round 6 M1: a plan whose watchdog gap is shorter than one release period plus the delivery
/// margin is infeasible and is refused before any hardware TX.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn infeasible_watchdog_gap_is_rejected_at_admission() {
    // Machine preflight.
    let clock = MockClock::new();
    let cfg = ArmConfig::rebot_dm();
    let bus = configure_damiao_bus(Some(clock.clone()), Duration::from_micros(200), false, TIMEOUT_2000_COUNTS);
    let mut drv = build_damiao_driver(bus.clone(), &cfg, 8);
    enable_driver_sync(&mut drv, &bus, Some(&clock));
    let mut plan = ModeSwitchPlan::empty(ModeSwitchIntent::ConfirmOnly);
    plan.axes[0] = AxisTarget::new(6, ControlMode::PositionVelocity);
    plan.len = 1;
    plan.max_watchdog_gap = Duration::from_millis(15);
    let mut ms = ModeSwitch::new(plan);
    bus.clear_log();
    let mut allow = || Ok(());
    let mut prog = ModeProgress::Pending { wake: None };
    for seq in 2..8u64 {
        clock.advance(PERIOD_50HZ);
        prog = step_raw_with_gate(
            &mut drv,
            &mut ms,
            &clock,
            |now, cutoff| ReleaseBudget::new(now, cutoff, PERIOD_50HZ, 8),
            PERIOD_50HZ,
            seq,
            &mut allow,
        );
        if prog.is_terminal() {
            break;
        }
    }
    assert!(
        matches!(prog, ModeProgress::FailedClean { reason, .. } if reason.code() == "WATCHDOG_GAP_INFEASIBLE"),
        "15 ms gap at a 20 ms release must be refused, got {prog:?}"
    );
    let reg_tx = bus.with(|s| s.tx_log.iter().filter(|f| f.id == 0x7FF).count());
    assert_eq!(reg_tx, 0, "no register TX for an infeasible plan");

    // Owner admission returns the same code.
    let mut cfg2 = ArmConfig::rebot_dm();
    cfg2.control_frequency_hz = 50;
    let bus2 = configure_damiao_bus(None, Duration::ZERO, false, TIMEOUT_2000_COUNTS);
    let drv2 = build_damiao_driver(bus2.clone(), &cfg2, 8);
    let sup = ArmSupervisor::new(cfg2).with_driver(Box::new(drv2));
    sup.set_lease_enabled(false).await;
    let err = sup
        .start_mode_switch_session(ModeSwitchStartRequest {
            plan,
            admission: sup.capture_admission_stamp(),
            operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
            deadline: Instant::now() + Duration::from_secs(1),
            client: ClientId(32),
        })
        .await
        .unwrap_err();
    assert!(
        matches!(err, ConfigError::CalibPrecondition { code: "WATCHDOG_GAP_INFEASIBLE", .. }),
        "{err:?}"
    );
}

/// Round 6 M2: the reserved terminal credit is released when the start is cancelled after the
/// reservation, when the owner rejects the start, when the caller's connection closes, and on
/// executor handback.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn mode_switch_terminal_credit_released_on_cancel_reject_close_and_handback() {
    use armd::rt_core::broker::{DeliveryDecision, Reply};
    use std::future::Future;
    let mut h = OwnerHarness::start(OwnerKind::Executor, true, 0, false, false).await;
    let exec = h.exec.take().unwrap();
    let open_sink = |id: u64| {
        exec.open_connection_sync(id).unwrap().unwrap();
        let sink = Arc::new(|_c: ClientId, _r: Reply| DeliveryDecision::Delivered);
        exec.register_connection_reply_sink(ClientId(id), sink, Some(4)).unwrap()
    };
    let _tok1 = open_sink(708);
    // J7 never answers: the first session stays pending and holds its credit.
    h.bus.with(|s| s.motors[6].silent = true);
    let mut plan = ModeSwitchPlan::confirm_gripper(6, ControlMode::PositionForce).unwrap();
    plan.per_axis_timeout = Duration::from_secs(3);
    let req = |client: u64| ModeSwitchStartRequest {
        plan,
        admission: h.sup.capture_admission_stamp(),
        operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
        deadline: Instant::now() + Duration::from_secs(4),
        client: ClientId(client),
    };
    let sid1 = h.sup.start_mode_switch_session(req(708)).await.unwrap();
    assert_eq!(exec.router_global_reserved(), 1);

    // Rejection: the owner refuses a second session (Busy); its credit is released.
    let rej = h.sup.start_mode_switch_session(req(708)).await;
    assert!(matches!(rej, Err(ConfigError::Busy(_))), "{rej:?}");
    assert_eq!(exec.router_global_reserved(), 1, "rejected start must release its credit");

    // Cancellation: drop the start future after it reserved its credit, before the owner reply.
    {
        let fut = h.sup.start_mode_switch_session(req(708));
        let mut fut = std::pin::pin!(fut);
        let waker = std::task::Waker::noop();
        let mut cx = std::task::Context::from_waker(waker);
        let mut reserved = false;
        for _ in 0..10_000 {
            if let std::task::Poll::Ready(r) = fut.as_mut().poll(&mut cx) {
                panic!("start completed before cancellation: {r:?}");
            }
            if exec.router_global_reserved() == 2 {
                reserved = true;
                break;
            }
            tokio::task::yield_now().await;
        }
        assert!(reserved, "start must reserve before awaiting the owner");
    }
    let dl = Instant::now() + Duration::from_secs(1);
    while exec.router_global_reserved() != 1 {
        assert!(Instant::now() < dl, "cancelled start leaked its credit: {}", exec.router_global_reserved());
        tokio::time::sleep(Duration::from_millis(5)).await;
    }

    // Connection close: the bound session's credit is recovered when its client disconnects.
    exec.disconnect_client(ClientId(708));
    let dl = Instant::now() + Duration::from_secs(1);
    while exec.router_global_reserved() != 0 {
        assert!(Instant::now() < dl, "closed connection leaked its credit");
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    let _ = h.sup.cancel_mode_switch_session(sid1, "CANCELLED").await;
    let dl = Instant::now() + Duration::from_secs(1);
    while !h.sup.clear_mode_switch_session(sid1).await.unwrap_or(false) {
        if h.sup.poll_mode_switch_session(sid1).await.ok().flatten().is_none() {
            break;
        }
        assert!(Instant::now() < dl);
        tokio::time::sleep(Duration::from_millis(5)).await;
    }

    // Handback: a pending bound session's credit is recovered when the executor stops.
    let _tok2 = open_sink(709);
    let dl = Instant::now() + Duration::from_secs(2);
    let _sid2 = loop {
        match h.sup.start_mode_switch_session(req(709)).await {
            Ok(s) => break s,
            Err(ConfigError::Busy(_)) => {
                assert!(Instant::now() < dl);
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
            Err(e) => panic!("{e:?}"),
        }
    };
    assert_eq!(exec.router_global_reserved(), 1);
    exec.stop_and_join();
    assert_eq!(exec.router_global_reserved(), 0, "handback must recover the credit");
    h.shutdown(false).await;
}

/// Round 6 Minor: fresh feedback must not clear `holding_unconfirmed` when a required axis's hold
/// frame fails at the transport (hold-write error or `UnheldTxFailed`).
#[test]
fn safe_hold_keeps_holding_unconfirmed_after_hold_tx_failure() {
    use armd::rt_core::safe_hold::{SafeHoldBuffers, SafeHoldControl};
    use armd::supervisor::SupervisorInner;
    let clock = MockClock::new();
    let cfg = ArmConfig::rebot_dm();
    let bus = configure_damiao_bus(Some(clock.clone()), Duration::from_micros(200), true, TIMEOUT_2000_COUNTS);
    let mut drv = build_damiao_driver(bus.clone(), &cfg, 16);
    enable_driver_sync(&mut drv, &bus, Some(&clock));
    let mut core = SupervisorInner::new(cfg.clone());
    core.lease_enabled = false;
    core.measured_positions = vec![0.30, -0.25, 0.40, 0.15, -0.15, 0.10, -0.14];
    core.feedback_numeric_valid = true;
    let mut bufs = SafeHoldBuffers::from_config(&cfg, PERIOD_50HZ);
    bufs.mirror_from_core(&core);
    bufs.seed_on_entry(&core);
    bufs.prepare_driver_on_entry(&mut drv);
    let ctrl = SafeHoldControl::new();
    ctrl.mark_active(SafeHoldReason::ControlPanic);
    // J1's PosVel hold frame (0x101) fails at the transport.
    bus.with(|s| s.fail_tx_ids = vec![0x101]);
    for _ in 0..3 {
        clock.advance(PERIOD_50HZ);
        bus.with(|s| s.push_all_feedback());
        let mut now_fn = || clock.now();
        let dl = clock.now() + PERIOD_50HZ;
        bufs.step_driver(&mut drv, &ctrl, dl, &mut now_fn);
        assert!(
            ctrl.holding_unconfirmed(),
            "SAFE-HOLD must not report confirmed holding while J1's hold TX fails"
        );
    }
}

// ---------------------------------------------------------------------------
// 8. Review round 7 regressions (owner-recorded applied-stop proof)
// ---------------------------------------------------------------------------

/// Builds a manually ticked Legacy supervisor with J1 switched forward to Impedance and held in
/// post-confirm feedback (no feedback after the mode ACK), returning `(sup, bus, sid)`.
async fn legacy_mid_switch(lease: bool) -> (ArmSupervisor, FakeBus, u64) {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.control_frequency_hz = 50;
    let bus = configure_damiao_bus(None, Duration::ZERO, true, TIMEOUT_2000_COUNTS);
    let mut drv = build_damiao_driver(bus.clone(), &cfg, 16);
    bus.with(|s| {
        s.immediate_replies = true;
        s.push_all_feedback();
    });
    let _ = drv.check_mapping_ranges();
    let _ = drv.read_joint_states();
    let sup = ArmSupervisor::new(cfg).with_driver(Box::new(drv));
    sup.set_lease_enabled(lease).await;
    if lease {
        sup.acquire_lease("client-1").await.unwrap();
    }
    sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
    bus.with(|s| {
        s.immediate_replies = false;
        s.push_all_feedback();
    });
    let _ = sup.tick(0.02).await;
    bus.with(|s| s.motors[0].mute_feedback_after_mode_ack = true);
    let mut plan =
        ModeSwitchPlan::switch_axes(&[(0, ControlMode::Impedance)], ModeSwitchRecovery::FailClosed)
            .unwrap();
    plan.per_axis_timeout = Duration::from_secs(2);
    let sid = sup
        .start_mode_switch_session(ModeSwitchStartRequest {
            plan,
            admission: sup.capture_admission_stamp(),
            operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
            deadline: Instant::now() + Duration::from_secs(4),
            client: ClientId(31),
        })
        .await
        .unwrap();
    let mut acked = 0;
    for _ in 0..30 {
        bus.with(|s| s.push_all_feedback());
        let _ = sup.tick(0.02).await;
        assert!(sup.poll_mode_switch_session(sid).await.unwrap().unwrap().running);
        if bus.with(|s| s.motors[0].ctrl_mode()) == damiao::CTRL_MODE_MIT {
            acked += 1;
            if acked >= 3 {
                break;
            }
        }
    }
    assert!(acked >= 3, "J1 must have switched forward");
    (sup, bus, sid)
}

async fn legacy_tick_until_terminal(sup: &ArmSupervisor, bus: &FakeBus, sid: u64) -> Option<ModeProgress> {
    for _ in 0..10 {
        bus.with(|s| s.push_all_feedback());
        let _ = sup.tick(0.02).await;
        if let Some(s) = sup.poll_mode_switch_session(sid).await.unwrap() {
            if !s.running {
                return Some(s.progress);
            }
        }
    }
    None
}

/// Round 7: a stop generation that reads as requested AND applied but was never applied by the
/// owner (the request landed after the pending check) is no proof: the energized partial switch
/// takes the SAFE-HOLD path. `StopRequests` is the deterministic hook for that interleaving.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn stop_generation_not_applied_by_owner_is_not_stop_proof() {
    let (sup, bus, sid) = legacy_mid_switch(false).await;
    {
        let mut inner = sup.inner().write().await;
        let stops = armd::rt_core::urgent::StopRequests::new();
        inner.attach_stop_requests(stops.clone());
        let g = stops.request();
        // The generation advances and reads as applied, but the owner never halted motion.
        stops.mark_applied(g);
    }
    let done = legacy_tick_until_terminal(&sup, &bus, sid).await;
    assert_eq!(
        done,
        Some(ModeProgress::Ambiguous { failed_joint: 0, reason: ModeSwitchFailure::Stopped }),
        "a generation the owner did not apply must not prove a stop"
    );
}

/// Round 7: a policy-only STOP (no policy running) advances and acknowledges the stop generation
/// without a general stop; mid-switch it must take the SAFE-HOLD path, not `StoppedRecovered`.
#[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn policy_only_stop_mid_switch_is_not_stop_proof() {
    use armd::rt_core::broker::{Authority, Reply};
    use armd::rt_core::requests::lease::ConnectionHandle;
    use armd::rt_core::requests::policy::{EndPolicySpec, PolicyCommand};
    let h = OwnerHarness::start(OwnerKind::Executor, true, TIMEOUT_2000_COUNTS, true, true).await;
    let exec = h.exec.clone().unwrap();
    h.bus.with(|s| s.motors[0].mute_feedback_after_mode_ack = true);
    let mut plan =
        ModeSwitchPlan::switch_axes(&[(0, ControlMode::Impedance)], ModeSwitchRecovery::FailClosed)
            .unwrap();
    plan.per_axis_timeout = Duration::from_secs(2);
    let sid = h
        .sup
        .start_mode_switch_session(ModeSwitchStartRequest {
            plan,
            admission: h.sup.capture_admission_stamp(),
            operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
            deadline: Instant::now() + Duration::from_secs(4),
            client: ClientId(33),
        })
        .await
        .unwrap();
    let dl = Instant::now() + Duration::from_secs(2);
    while h.bus.with(|s| s.motors[0].ctrl_mode()) != damiao::CTRL_MODE_MIT {
        assert!(Instant::now() < dl, "J1 must switch forward");
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    tokio::time::sleep(Duration::from_millis(80)).await;
    assert!(h.sup.poll_mode_switch_session(sid).await.unwrap().unwrap().running);

    let lg = h.sup.lease_generation();
    let conn = ConnectionHandle::try_new(1, "mode-switch-test-client").unwrap();
    exec.open_connection_sync(41).unwrap().unwrap();
    let reply = exec
        .submit_policy_command_async(
            ClientId(41),
            1,
            Authority {
                lease_generation: lg,
                epoch: h.sup.estop_epoch(),
                stop_generation: h.sup.current_stop_gen(),
                config_revision: 0,
                parent_generation: 0,
            },
            Instant::now() + Duration::from_secs(2),
            PolicyCommand::EndPolicy(EndPolicySpec::Stop { connection: conn, lease_generation: lg }),
            |_| {},
        )
        .await
        .unwrap();
    assert!(matches!(reply, Reply::Ack { .. }), "policy stop must be accepted: {reply:?}");
    let dl = Instant::now() + Duration::from_secs(2);
    let done = loop {
        if let Some(s) = h.sup.poll_mode_switch_session(sid).await.unwrap() {
            if !s.running {
                break s.progress;
            }
        }
        assert!(Instant::now() < dl, "switch must end after the policy stop");
        tokio::time::sleep(Duration::from_millis(5)).await;
    };
    assert_eq!(
        done,
        ModeProgress::Ambiguous { failed_joint: 0, reason: ModeSwitchFailure::Stopped },
        "a policy-only STOP must not prove a general stop"
    );
    h.shutdown(true).await;
}
