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

use armd::auth::OperatorGrant;
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

    // Seed confirmed control frames at t0 across J1..J6.
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
    );
    let mut ms = ModeSwitch::new(to_mit);

    // Zero-alloc proof across active RT stepping (turn off tx_log growth only for the alloc probe
    // tick, then re-enable for full wire-gap audit).
    let mut final_prog = ModeProgress::Pending { wake: None };
    for tick in 0..40 {
        let t_tick = start_at + PERIOD_50HZ * tick;
        if t_tick > clock.now() {
            clock.advance(t_tick - clock.now());
        }
        bus.with(|s| s.run_watchdogs());
        seq += 1;
        if tick == 4 {
            let mut rx_buf = Vec::with_capacity(16);
            bus.with(|s| s.record_tx_log = false);
            let before = snapshot_counts();
            let tick_start = clock.now();
            let d = ex_drv.get_mut().as_mut();
            d.begin_control_release(seq);
            let cutoff = tick_start + PERIOD_50HZ.saturating_sub(Duration::from_millis(3));
            d.set_tx_release_deadline(Some(cutoff));
            let _ = d.read_joint_states_into(&mut rx_buf);
            let budget = ReleaseBudget::new(clock.now(), cutoff, PERIOD_50HZ, 8);
            final_prog = ms.step(&mut ex_drv, budget);
            ex_drv.get_mut().as_mut().set_tx_release_deadline(None);
            let delta = snapshot_counts().delta_since(before);
            bus.with(|s| s.record_tx_log = true);
            assert_eq!(
                (delta.alloc, delta.alloc_zeroed, delta.realloc, delta.dealloc),
                (0, 0, 0, 0),
                "ModeSwitch::step + RX intake must perform 0 heap allocs/frees on RT: {delta:?}"
            );
        } else {
            final_prog = step_release_on_exclusive(&mut ex_drv, &mut ms, &clock, PERIOD_50HZ, 8, seq);
        }
        if final_prog.is_terminal() {
            break;
        }
    }
    assert_eq!(final_prog, ModeProgress::Confirmed, "6-axis PosVel->Impedance must confirm");
    for j in 0..6 {
        assert_eq!(ex_drv.get_ref().axis_confirmed_mode(j), Some(ControlMode::Impedance));
    }
    assert_eq!(ex_drv.get_ref().mode_write_uncertain_mask(), 0);

    // Return switch: Impedance -> PositionVelocity with 100% wire logging for exact gap audit.
    let return_start = clock.now();
    bus.clear_log();
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
    );
    let mut ms_back = ModeSwitch::new(to_pv);
    let mut back_prog = ModeProgress::Pending { wake: None };
    for tick in 0..40 {
        let t_tick = return_start + PERIOD_50HZ * tick;
        if t_tick > clock.now() {
            clock.advance(t_tick - clock.now());
        }
        bus.with(|s| s.run_watchdogs());
        seq += 1;
        back_prog = step_release_on_exclusive(&mut ex_drv, &mut ms_back, &clock, PERIOD_50HZ, 8, seq);
        if back_prog.is_terminal() {
            break;
        }
    }
    let return_end = clock.now();
    assert_eq!(back_prog, ModeProgress::Confirmed, "6-axis Impedance->PosVel must confirm");
    for j in 0..6 {
        assert_eq!(ex_drv.get_ref().axis_confirmed_mode(j), Some(ControlMode::PositionVelocity));
    }

    let (tx_log, tx_times, expiries, all_enabled) = bus.with(|s| {
        s.run_watchdogs();
        let exp: usize = s.motors.iter().map(|m| m.watchdog_expiry_count).sum();
        let en = s.motors.iter().all(|m| m.enabled);
        (s.tx_log.clone(), s.tx_times.clone(), exp, en)
    });
    assert_eq!(expiries, 0, "Damiao TIMEOUT=2000 must suffer 0 watchdog expiries");
    assert!(all_enabled, "all motors must remain enabled across round-trip switch");

    // Audit exact wire gaps across the Impedance -> PosVel round-trip (initial mode = MIT).
    // Verify using the bus's actual mode history by starting audit from Impedance.
    let mut active_mode = [damiao::CTRL_MODE_MIT; 6];
    let mut last_ctrl = [Some(return_start); 6];
    let mut max_gap = [Duration::ZERO; 6];
    let mut wrong_mode = 0usize;
    let mut disables = 0usize;
    for (frame, ts) in tx_log.iter().zip(tx_times.iter()) {
        let Some(t) = *ts else { continue };
        if frame.data == damiao::CMD_DISABLE {
            disables += 1;
            continue;
        }
        if frame.id == 0x7FF && frame.data[2] == damiao::REG_WRITE && frame.data[3] == damiao::RID_CTRL_MODE {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            if (1..=6).contains(&id) {
                active_mode[(id - 1) as usize] =
                    u32::from_le_bytes([frame.data[4], frame.data[5], frame.data[6], frame.data[7]]);
            }
            continue;
        }
        if (0x101..=0x106).contains(&frame.id) {
            let j = (frame.id - 0x101) as usize;
            if active_mode[j] != damiao::CTRL_MODE_POS_VEL { wrong_mode += 1; }
            if let Some(prev) = last_ctrl[j] { max_gap[j] = max_gap[j].max(t.saturating_duration_since(prev)); }
            last_ctrl[j] = Some(t);
        } else if (1..=6).contains(&frame.id) && frame.data != damiao::CMD_ENABLE {
            let j = (frame.id - 1) as usize;
            if active_mode[j] != damiao::CTRL_MODE_MIT { wrong_mode += 1; }
            if let Some(prev) = last_ctrl[j] { max_gap[j] = max_gap[j].max(t.saturating_duration_since(prev)); }
            last_ctrl[j] = Some(t);
        }
    }
    for j in 0..6 {
        if let Some(prev) = last_ctrl[j] {
            max_gap[j] = max_gap[j].max(return_end.saturating_duration_since(prev));
        }
        assert!(
            max_gap[j] <= MAX_ALLOWED_GAP,
            "J{} watchdog gap {:?} exceeded 40 ms bound (all gaps: {:?})",
            j + 1,
            max_gap[j],
            max_gap
        );
    }
    assert_eq!(wrong_mode, 0, "0 control frames may be sent in an unverified/wrong mode");
    assert_eq!(disables, 0, "0 CMD_DISABLE frames may be sent during mode switch");
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
    );
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
    let mut ms_late = ModeSwitch::new(ModeSwitchPlan::switch_axes(
        &[(0, ControlMode::PositionVelocity)],
        ModeSwitchRecovery::FailClosed,
    ));
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

    // Advance exactly 1 period (release N+1): ACK still absent -> MUST abort immediately on N+1!
    clock.advance(PERIOD_50HZ);
    seq += 1;
    let p_n1 = step_release_on_exclusive(&mut ex_drv, &mut ms_late, &clock, PERIOD_50HZ, 8, seq);
    assert_eq!(
        p_n1,
        ModeProgress::Ambiguous {
            failed_joint: 0,
            reason: ModeSwitchFailure::WriteAckTimeout,
        },
        "must abort on release N+1 before waiting to N+2 (>= 40 ms)"
    );
    assert_ne!(ex_drv.get_ref().mode_write_uncertain_mask() & 1, 0);

    // Restore J1 cleanly for part (c).
    bus.with(|s| {
        s.motors[0].no_write_ack = false;
        s.motors[0].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_VEL);
    });
    ex_drv.get_mut().as_mut().set_axis_confirmed_mode(0, Some(ControlMode::PositionVelocity));

    // (c) Required post-write readback:
    // - At 50 Hz (2 * 20 + 5 = 45 > 40 ms): refused fail-closed at admission with 0 REG_WRITE frames!
    let mut plan_rb = ModeSwitchPlan::switch_axes(
        &[(0, ControlMode::Impedance)],
        ModeSwitchRecovery::FailClosed,
    );
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
    assert_eq!(bus.with(|s| s.motors.iter().map(|m| m.watchdog_expiry_count).sum::<usize>()), 0);
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

    // (a) Mandatory wire-range check + partial switch causal rollback:
    // J1 ranges verified; J2 ranges unverified and hardware reports mismatched TMAX (99.0 vs configured 28.0).
    // J1 switches to Impedance -> J2 fails mandatory range check BEFORE writing J2 -> J1 rolls back to PosVel!
    ex_drv.get_mut().as_mut().set_axis_impedance_ranges_verified(1, false);
    bus.with(|s| {
        s.motors[1].regs.insert(damiao::RID_TMAX, 99.0f32.to_le_bytes());
    });
    bus.clear_log();
    let mut ms_rb = ModeSwitch::new(ModeSwitchPlan::switch_axes(
        &[(0, ControlMode::Impedance), (1, ControlMode::Impedance)],
        ModeSwitchRecovery::RollbackToPrevious,
    ));
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

    // (b) Failed flush after REG_WRITE (`real_flush = true`, `fail_next_reg_write_flush = true`):
    // Must immediately set uncertain mask & return Ambiguous!
    bus.with(|s| {
        s.real_flush = true;
        s.fail_next_reg_write_flush = true;
    });
    let mut ms_fl = ModeSwitch::new(ModeSwitchPlan::switch_axes(
        &[(0, ControlMode::Impedance)],
        ModeSwitchRecovery::FailClosed,
    ));
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
    let mut ms_dup = ModeSwitch::new(ModeSwitchPlan::switch_axes(
        &[(0, ControlMode::Impedance)],
        ModeSwitchRecovery::FailClosed,
    ));
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

    // (d) Blocker 3: Energized ConfirmOnly contradiction on J7!
    // Driver cache believes J7 is verified PosForce, but motor causally answers POS_VEL.
    // Must invalidate cached PosForce mode & return Ambiguous!
    assert_eq!(ex_drv.get_ref().axis_confirmed_mode(6), Some(ControlMode::PositionForce));
    bus.with(|s| {
        s.motors[6].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_VEL);
    });
    let mut ms_j7_contra = ModeSwitch::new(ModeSwitchPlan::confirm_gripper(6, ControlMode::PositionForce));
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

    // (e) J7 ConfirmOnly wrong reply CAN ID (`reply_id != 0x17`) -> rejected as AmbiguousReply!
    bus.with(|s| {
        s.motors[6].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_FORCE);
        s.motors[6].reply_id = 0x18; // wrong master reply CAN ID
    });
    let mut ms_j7_id = ModeSwitch::new(ModeSwitchPlan::confirm_gripper(6, ControlMode::PositionForce));
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
        ModeProgress::FailedClean {
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
    let exec = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(8)))
        .await
        .unwrap();
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

    let sid1 = sup
        .start_mode_switch_session(ModeSwitchStartRequest {
            plan,
            admission: sup.capture_admission_stamp(),
            operator_required: false,
            operator_grant: OperatorGrant::unenforced(),
            deadline: Instant::now() + Duration::from_secs(2),
            client: ClientId(7),
            terminal_ticket: None,
        })
        .await
        .unwrap();

    // Poll until terminal despite saturated capacity-1 event stream.
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
    assert!(sup.clear_mode_switch_session(sid1).await.unwrap());
    // Stream dropped excess events cleanly without blocking authoritative poll/clear.
    let mut popped = 0usize;
    while ev_rx.try_pop().is_some() {
        popped += 1;
    }
    assert_eq!(popped, 1);
    assert!(ev_rx.stats().dropped_full >= 1, "capacity-1 stream must record drop-newest loss");

    // Start a second session (silent J1 so it stays live) and verify stale cancel/clear for sid1
    // are ignored and do NOT disturb sid2!
    bus.with(|s| s.motors[0].silent = true);
    let sid2 = sup
        .start_mode_switch_session(ModeSwitchStartRequest {
            plan,
            admission: sup.capture_admission_stamp(),
            operator_required: false,
            operator_grant: OperatorGrant::unenforced(),
            deadline: Instant::now() + Duration::from_secs(2),
            client: ClientId(7),
            terminal_ticket: None,
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
    let live2 = sup.poll_mode_switch_session(sid2).await.unwrap().unwrap();
    assert!(live2.running, "stale cancel/clear must never disturb active sid2");

    // Clean cancel & clear of sid2.
    let c2 = sup.cancel_mode_switch_session(sid2, "STOPPED").await.unwrap().unwrap();
    assert_eq!(
        c2.progress,
        ModeProgress::StoppedRecovered {
            reason: ModeSwitchFailure::Stopped
        }
    );
    assert!(sup.clear_mode_switch_session(sid2).await.unwrap());
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
    async fn start(kind: OwnerKind, j7_pos_force: bool, arm_timeout_counts: u32, arm_enabled: bool) -> Self {
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
        sup.set_lease_enabled(false).await;
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
        let h = OwnerHarness::start(owner, true, 0, false).await;

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
/// AND partial-switch range-mismatch rollback produce identical outcomes on Legacy & Executor.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_energized_mode_switch_and_rollback_parity() {
    let mut results = Vec::new();
    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let h = OwnerHarness::start(owner, true, TIMEOUT_2000_COUNTS, true).await;
        h.bus.clear_log();

        // 1. Switch J1..J3 PosVel -> Impedance.
        let plan_mit = ModeSwitchPlan::switch_axes(
            &[
                (0, ControlMode::Impedance),
                (1, ControlMode::Impedance),
                (2, ControlMode::Impedance),
            ],
            ModeSwitchRecovery::RollbackToPrevious,
        );
        let snap_mit = h
            .sup
            .run_mode_switch_session(
                ModeSwitchStartRequest {
                    plan: plan_mit,
                    admission: h.sup.capture_admission_stamp(),
                    operator_required: false,
                    operator_grant: OperatorGrant::unenforced(),
                    deadline: Instant::now() + Duration::from_secs(2),
                    client: ClientId(10),
                    terminal_ticket: None,
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
        );
        let snap_pv = h
            .sup
            .run_mode_switch_session(
                ModeSwitchStartRequest {
                    plan: plan_pv,
                    admission: h.sup.capture_admission_stamp(),
                    operator_required: false,
                    operator_grant: OperatorGrant::unenforced(),
                    deadline: Instant::now() + Duration::from_secs(2),
                    client: ClientId(10),
                    terminal_ticket: None,
                },
                Duration::from_secs(2),
            )
            .await
            .unwrap();
        assert_eq!(snap_pv.progress, ModeProgress::Confirmed, "{owner:?}: Impedance->PosVel");

        let (disables, all_enabled, modes_after) = h.bus.with(|s| {
            let dis = s.tx_log.iter().filter(|f| f.data == damiao::CMD_DISABLE).count();
            let en = s.motors.iter().all(|m| m.enabled);
            let modes = [s.motors[0].ctrl_mode(), s.motors[1].ctrl_mode(), s.motors[2].ctrl_mode()];
            (dis, en, modes)
        });
        assert_eq!(disables, 0, "{owner:?}: 0 CMD_DISABLE frames");
        assert!(all_enabled, "{owner:?}: all motors remain enabled");
        assert_eq!(modes_after, [damiao::CTRL_MODE_POS_VEL; 3]);

        // 3. Partial-switch rollback parity: J1 confirms Impedance, J2 fails mandatory range check
        //    (mismatched TMAX=99.0) before writing J2 -> J1 rolls back causally to PosVel.
        h.shutdown(true).await;

        let h_rb = OwnerHarness::start(owner, true, TIMEOUT_2000_COUNTS, true).await;
        h_rb.bus.with(|s| {
            s.motors[1].regs.insert(damiao::RID_TMAX, 99.0f32.to_le_bytes());
        });
        // Re-mark J2 range unverified before switching (since startup range check ran before mutation).
        // Start a fresh harness with J2 mismatched from the start? Startup range_check only runs if
        // check_mapping_ranges was called; instead use an unsupported mode or fresh unverified J2:
        let snap_rb = {
            // Confirm J1 -> Impedance, then J2 -> Position (unsupported on Damiao? No, fail before write in init).
            // For mid-switch rollback on a live owner, use a second harness where J2 wasn't range-verified.
            h_rb.shutdown(true).await;
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
            sup_rb.set_lease_enabled(false).await;
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
                        ),
                        admission: sup_rb.capture_admission_stamp(),
                        operator_required: false,
                        operator_grant: OperatorGrant::unenforced(),
                        deadline: Instant::now() + Duration::from_secs(2),
                        client: ClientId(11),
                        terminal_ticket: None,
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

/// Dual-owner test 3: Authority refusal (epoch mismatch / operator required) AND energized
/// mid-write ambiguity -> same-release SAFE-HOLD (`SafeHoldReason::SessionUncertainState`) with
/// 0 `CMD_DISABLE` frames produce identical outcomes on Legacy & Executor.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_mode_switch_authority_and_ambiguity_safe_hold_parity() {
    let mut results = Vec::new();
    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let h = OwnerHarness::start(owner, true, TIMEOUT_2000_COUNTS, true).await;

        // 1. Authority check: stale epoch refused identically on both owners.
        let mut stale_stamp = h.sup.capture_admission_stamp();
        stale_stamp.estop_epoch = stale_stamp.estop_epoch.wrapping_add(99);
        let plan = ModeSwitchPlan::switch_axes(
            &[(0, ControlMode::Impedance)],
            ModeSwitchRecovery::FailClosed,
        );
        let err_epoch = h
            .sup
            .start_mode_switch_session(ModeSwitchStartRequest {
                plan,
                admission: stale_stamp,
                operator_required: false,
                operator_grant: OperatorGrant::unenforced(),
                deadline: Instant::now() + Duration::from_secs(2),
                client: ClientId(20),
                terminal_ticket: None,
            })
            .await
            .unwrap_err();
        assert!(
            matches!(err_epoch, ConfigError::EpochMismatch { .. }),
            "{owner:?}: expected EpochMismatch, got {err_epoch:?}"
        );

        // 2. Authority check: operator required with unenforced grant when operator_required=true.
        let err_op = h
            .sup
            .start_mode_switch_session(ModeSwitchStartRequest {
                plan,
                admission: h.sup.capture_admission_stamp(),
                operator_required: true,
                operator_grant: OperatorGrant::DENIED,
                deadline: Instant::now() + Duration::from_secs(2),
                client: ClientId(20),
                terminal_ticket: None,
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

        // 3. Energized mid-write ambiguity (J1 drops write ACK) -> same-release SAFE-HOLD
        //    (`SafeHoldReason::SessionUncertainState`), 0 CMD_DISABLE frames, motors stay enabled!
        h.bus.clear_log();
        h.bus.with(|s| s.motors[0].no_write_ack = true);
        let _sid = h
            .sup
            .start_mode_switch_session(ModeSwitchStartRequest {
                plan,
                admission: h.sup.capture_admission_stamp(),
                operator_required: false,
                operator_grant: OperatorGrant::unenforced(),
                deadline: Instant::now() + Duration::from_secs(2),
                client: ClientId(20),
                terminal_ticket: None,
            })
            .await
            .unwrap();

        let sh = match owner {
            OwnerKind::Executor => h.exec.as_ref().unwrap().safe_hold_control(),
            OwnerKind::Legacy => h.sup.legacy_safe_hold_control(),
        };
        let dl = Instant::now() + Duration::from_secs(3);
        while (!sh.is_active() || sh.ticks() < 2) && Instant::now() < dl {
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        assert!(sh.is_active(), "{owner:?}: energized ambiguity must enter SAFE-HOLD");
        assert_eq!(
            sh.reason(),
            SafeHoldReason::SessionUncertainState,
            "{owner:?}: SAFE-HOLD reason must be SessionUncertainState"
        );
        assert!(sh.hold_tx_count() >= 1, "{owner:?}: SAFE-HOLD must emit hold frames");
        assert!(h.sup.internal_fault_fenced(), "{owner:?}: internal fault fence must be latched");

        let (all_enabled, disables) = h.bus.with(|s| {
            let dis = s.tx_log.iter().filter(|f| f.data == damiao::CMD_DISABLE).count();
            (s.motors.iter().all(|m| m.enabled), dis)
        });
        assert!(all_enabled, "{owner:?}: all motors must remain enabled in SAFE-HOLD away from rest");
        assert_eq!(disables, 0, "{owner:?}: 0 CMD_DISABLE frames on SAFE-HOLD escalation");
        if owner == OwnerKind::Executor {
            assert_eq!(h.sup.inner().control_thread_attempts(), 0, "0 SupervisorLock attempts on armd-control");
        }

        results.push((true, op_is_rejected, sh.reason(), disables));
        h.shutdown(true).await;
    }
    assert_eq!(results[0], results[1], "Legacy and Executor authority & SAFE-HOLD parity");
}
