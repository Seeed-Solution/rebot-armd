//! Comprehensive verification suite for the pure `armd::torque` reducer core:
//! - Unit tests for every CiA 402 state, transition, role, boundary, and refusal code
//! - Synthetic underactuated hand group next to the arm group
//! - Exhaustive short-race permutations (3rd sample vs stop, partial dispatch, thermal, expiry, enable interruption)
//! - Seeded random event-sequence model test: 10,000 sequences × up to 128 events with invariants after every event
//! - Automatic trace shrinker (event-span deletion, channel/motor reduction, timestamp boundary shrinking)
//! - Permanent JSON regression fixture replay (`tests/fixtures/torque_reducer/*.json`)
//! - Zero-allocation proof on `step()` and `check_invariants()` using `armd::diagnostics::alloc::TimedAlloc`

#[global_allocator]
static ALLOC: armd::diagnostics::alloc::TimedAlloc = armd::diagnostics::alloc::TimedAlloc;

use std::fs;
use std::path::Path;

use serde::{Deserialize, Serialize};

use armd::diagnostics::alloc::{snapshot_counts, time_this_thread, AllocCounts};
use armd::motor_mode::ControlMode;
use armd::torque::{
    check_invariants, compute_thermal_derate_scale, event_now_us, step, ActionPriority, ActionSet,
    ActiveToolConfig, ActiveToolSafeOutput, CausalSample, ChannelConfig, ChannelMask, ConfigError,
    DampingConfig, DeviceConfig, DeviceMask, DisableKind, DisableOrigin, DriveState, EnablePhase,
    Event, GroupConfig, GroupMask, HandshakeOutcome, HoldKind, JointRole,
    MotorWriteOutcome, PanicTier, RecipientConfig, RefusalCode, ReleaseEvidence, StopKind,
    ThermalConfig, TorqueConfig, TorqueFaultKind, TorqueIntent, TorqueState, VendorKind,
    Violation, WriteKind, WriteResult, CLOCK_OFFSET_MAX_DRIFT_US, CONTENTION_ESCALATION_RELEASES,
    DAMIAO_DISABLE_REPEATS, DAMIAO_DISABLE_REPEAT_INTERVAL_US, DAMIAO_TIMEOUT_MAX_REG,
    DEFAULT_CAPTURE_MAX_AGE_US, DEFAULT_DISABLE_DEADLINE_US, DEFAULT_ENABLE_DEADLINE_US,
    DEFAULT_PANIC_BUS_LOSS_LIMIT_US, FEETECH_POISON_SILENCE_US, MAX_CHANNELS, MAX_DEVICES,
    MAX_GROUPS, REST_SETTLE_US, REST_WINDOW_MIN_SPAN_US, US_PER_MS,
};

// =========================================================================
// Helpers
// =========================================================================

fn checked_step(state: &mut TorqueState, event: Event) -> ActionSet {
    let prev = *state;
    let actions = step(state, event);
    if let Err(v) = check_invariants(state, &prev, &event, &actions) {
        panic!("invariant violated: {v:?}\nevent: {event:?}\nprev: {prev:?}\nstate: {state:?}\nactions: {actions:?}");
    }
    assert_priority_order(state, &actions);
    actions
}

fn assert_priority_order(state: &TorqueState, actions: &ActionSet) {
    let mut last: Option<ActionPriority> = None;
    actions.visit_by_priority(state, |a| {
        let p = a.priority();
        if let Some(prev_p) = last {
            assert!(
                p >= prev_p,
                "action priority regressed: {prev_p:?} -> {p:?} on {a:?}"
            );
        }
        last = Some(p);
    });
}

fn sample_at(
    channel: u8,
    sample_id: u64,
    ts_us: u64,
    pos_rad: f32,
) -> CausalSample {
    CausalSample {
        channel,
        sample_id,
        rx_timestamp_us: ts_us,
        now_us: ts_us,
        clock_offset_drift_us: 0,
        position_rad: pos_rad,
        velocity_rad_s: Some(0.0),
        temperature_c: None,
        reported_enabled: None,
        correlated_after_write_done_us: None,
        release_evidence: None,
        is_handshake_ack_only: false,
    }
}

fn verify_all_modes(state: &mut TorqueState, now_us: u64) {
    for c in 0..state.config.channel_count {
        let ch = state.config.channels[c as usize];
        if !matches!(ch.role, JointRole::PassiveSensor) {
            checked_step(
                state,
                Event::ModeVerified {
                    channel: c,
                    mode: ch.desired_mode,
                    now_us,
                },
            );
            if let Some(reg) = ch.damiao_timeout_reg {
                checked_step(
                    state,
                    Event::TimeoutReadback {
                        channel: c,
                        timeout_reg: reg,
                        now_us,
                    },
                );
            }
        }
    }
}

fn enable_group_clean(state: &mut TorqueState, group: u8, now_us: u64) {
    verify_all_modes(state, now_us);
    let act = checked_step(state, Event::EnableStart { group, now_us });
    assert!(act.enable_for(group).is_some(), "enable should be accepted");
    let chs = state.config.groups[group as usize].channels;
    checked_step(
        state,
        Event::HandshakeResult {
            group,
            now_us: now_us + 5_000,
            confirmed_mask: chs,
            outcome: HandshakeOutcome::Confirmed,
        },
    );
    assert_eq!(
        state.groups[group as usize].drive_state,
        DriveState::OperationEnabled
    );
    assert_eq!(
        state.groups[group as usize].torque_intent,
        TorqueIntent::On
    );
}

fn single_arm_config(joints: u8) -> TorqueConfig {
    let mut cfg = TorqueConfig::default();
    cfg.channel_count = joints;
    cfg.recipient_count = joints;
    cfg.group_count = 1;
    cfg.device_count = 1;
    cfg.devices[0] = DeviceConfig::default();
    cfg.groups[0] = GroupConfig {
        id: 0,
        channels: ChannelMask::from_count(joints),
        recipients: ChannelMask::from_count(joints),
        devices: DeviceMask::bit(0),
        disable_repeats: DAMIAO_DISABLE_REPEATS,
        disable_repeat_interval_us: DAMIAO_DISABLE_REPEAT_INTERVAL_US,
        disable_deadline_us: DEFAULT_DISABLE_DEADLINE_US,
        enable_deadline_us: DEFAULT_ENABLE_DEADLINE_US,
        has_park_path: true,
    };
    for i in 0..joints {
        cfg.channels[i as usize] = ChannelConfig {
            id: i,
            device: 0,
            group: 0,
            recipient: i,
            role: JointRole::LoadBearing,
            supports_mit_damping: true,
            ..ChannelConfig::default()
        };
        cfg.recipients[i as usize] = RecipientConfig {
            id: i,
            device: 0,
            group: 0,
            channels: ChannelMask::bit(i),
            grouped_packet_mask: ChannelMask::bit(i),
            role: JointRole::LoadBearing,
            vendor: VendorKind::Damiao,
            rest_evidence_qualified: true,
        };
    }
    cfg
}

// =========================================================================
// 1. Unit Tests Per Transition & Policy
// =========================================================================

#[test]
fn state_size_and_topology_validation() {
    // Spec estimates ~5 KiB of state before frame buffers; keep the inline config+runtime compact.
    let state_bytes = std::mem::size_of::<TorqueState>();
    let actions_bytes = std::mem::size_of::<ActionSet>();
    assert!(
        state_bytes <= 16_384,
        "TorqueState unexpectedly large: {state_bytes} bytes"
    );
    assert!(
        actions_bytes <= 8_192,
        "ActionSet unexpectedly large: {actions_bytes} bytes"
    );

    // Valid default profiles
    assert!(TorqueState::new(TorqueConfig::rebot_dm_default()).is_ok());
    assert!(TorqueState::new(TorqueConfig::arm_plus_underactuated_hand()).is_ok());

    // Exact maximum capacity (32 channels, 4 devices, 8 groups) accepted
    let mut max_cfg = TorqueConfig::default();
    max_cfg.channel_count = MAX_CHANNELS as u8;
    max_cfg.recipient_count = MAX_CHANNELS as u8;
    max_cfg.group_count = MAX_GROUPS as u8;
    max_cfg.device_count = MAX_DEVICES as u8;
    for d in 0..MAX_DEVICES as u8 {
        max_cfg.devices[d as usize] = DeviceConfig {
            id: d,
            bus_id: d,
            ..DeviceConfig::default()
        };
    }
    for g in 0..MAX_GROUPS as u8 {
        let mut ch_mask = ChannelMask::EMPTY;
        for offset in 0..4u8 {
            ch_mask.insert(g * 4 + offset);
        }
        let dev = g / 2;
        max_cfg.groups[g as usize] = GroupConfig {
            id: g,
            channels: ch_mask,
            recipients: ch_mask,
            devices: DeviceMask::bit(dev),
            ..GroupConfig::default()
        };
    }
    for c in 0..MAX_CHANNELS as u8 {
        let g = c / 4;
        let d = g / 2;
        max_cfg.channels[c as usize] = ChannelConfig {
            id: c,
            device: d,
            group: g,
            recipient: c,
            ..ChannelConfig::default()
        };
        max_cfg.recipients[c as usize] = RecipientConfig {
            id: c,
            device: d,
            group: g,
            channels: ChannelMask::bit(c),
            grouped_packet_mask: ChannelMask::bit(c),
            ..RecipientConfig::default()
        };
    }
    assert!(TorqueState::new(max_cfg).is_ok());

    // Capacity overflow rejected
    let mut bad = TorqueConfig::rebot_dm_default();
    bad.channel_count = (MAX_CHANNELS + 1) as u8;
    assert_eq!(
        bad.validate(),
        Err(ConfigError::ChannelCapacityExceeded {
            count: 33,
            max: MAX_CHANNELS
        })
    );

    let mut bad = TorqueConfig::rebot_dm_default();
    bad.device_count = (MAX_DEVICES + 1) as u8;
    assert_eq!(
        bad.validate(),
        Err(ConfigError::DeviceCapacityExceeded {
            count: 5,
            max: MAX_DEVICES
        })
    );

    let mut bad = TorqueConfig::rebot_dm_default();
    bad.group_count = (MAX_GROUPS + 1) as u8;
    assert_eq!(
        bad.validate(),
        Err(ConfigError::GroupCapacityExceeded {
            count: 9,
            max: MAX_GROUPS
        })
    );

    // Active tool without qualified safe output rejected at profile load
    let mut bad = single_arm_config(1);
    bad.channels[0].role = JointRole::ActiveTool(ActiveToolSafeOutput::StopOutput);
    bad.recipients[0].role = JointRole::ActiveTool(ActiveToolSafeOutput::StopOutput);
    bad.channels[0].active_tool_qualification = ActiveToolConfig::Unqualified;
    assert_eq!(
        bad.validate(),
        Err(ConfigError::UnqualifiedActiveTool { channel: 0 })
    );

    // Zero bounded retention rejected
    let mut bad = single_arm_config(1);
    let ret = ActiveToolSafeOutput::BoundedRetention {
        max_retention_us: 0,
    };
    bad.channels[0].role = JointRole::ActiveTool(ret);
    bad.recipients[0].role = JointRole::ActiveTool(ret);
    bad.channels[0].active_tool_qualification = ActiveToolConfig::Qualified(ret);
    assert_eq!(
        bad.validate(),
        Err(ConfigError::InvalidActiveToolRetention { channel: 0 })
    );

    // Invalid thermal config (derate_start >= trip) rejected
    let mut bad = single_arm_config(1);
    bad.channels[0].thermal = ThermalConfig {
        derate_start_c: Some(70.0),
        trip_c: 70.0,
    };
    assert_eq!(
        bad.validate(),
        Err(ConfigError::InvalidThermalConfig { channel: 0 })
    );

    // Invalid damping config rejected
    let mut bad = single_arm_config(1);
    bad.channels[0].damping = DampingConfig {
        duration_us: 20_000,
        kd: 0.0,
        max_torque_nm: 3.0,
    };
    assert_eq!(
        bad.validate(),
        Err(ConfigError::InvalidDampingConfig { channel: 0 })
    );

    // Invalid Damiao TIMEOUT reg (0 or > 8000) rejected; 8000 accepted
    let mut bad = single_arm_config(1);
    bad.channels[0].damiao_timeout_reg = Some(0);
    assert_eq!(
        bad.validate(),
        Err(ConfigError::InvalidDamiaoTimeoutReg { channel: 0, reg: 0 })
    );
    bad.channels[0].damiao_timeout_reg = Some(DAMIAO_TIMEOUT_MAX_REG + 1);
    assert_eq!(
        bad.validate(),
        Err(ConfigError::InvalidDamiaoTimeoutReg {
            channel: 0,
            reg: DAMIAO_TIMEOUT_MAX_REG + 1
        })
    );
    bad.channels[0].damiao_timeout_reg = Some(DAMIAO_TIMEOUT_MAX_REG);
    assert!(bad.validate().is_ok());

    // Cross-group grouped_packet_mask bleed rejected
    let mut bad = TorqueConfig::arm_plus_underactuated_hand();
    bad.recipients[6].grouped_packet_mask.insert(0);
    assert_eq!(
        bad.validate(),
        Err(ConfigError::CrossGroupRecipientBleed {
            recipient: 6,
            target: 0
        })
    );

    // Passive sensor listed as physical recipient rejected
    let mut bad = single_arm_config(1);
    bad.channels[0].role = JointRole::PassiveSensor;
    bad.recipients[0].role = JointRole::PassiveSensor;
    assert_eq!(
        bad.validate(),
        Err(ConfigError::PassiveSensorListedAsRecipient {
            channel: 0,
            recipient: 0
        })
    );
}

#[test]
fn enable_handshake_mode_and_timeout_readback_gate() {
    let mut cfg = single_arm_config(2);
    cfg.channels[0].damiao_timeout_reg = Some(900); // 45 ms at 50 us/count
    cfg.channels[1].damiao_timeout_reg = Some(900);
    let mut s = TorqueState::new(cfg).unwrap();

    // Verify modes but NOT timeout readback -> EnableStart MUST refuse with TIMEOUT_MISMATCH.
    checked_step(
        &mut s,
        Event::ModeVerified {
            channel: 0,
            mode: ControlMode::PositionVelocity,
            now_us: 1_000,
        },
    );
    checked_step(
        &mut s,
        Event::ModeVerified {
            channel: 1,
            mode: ControlMode::PositionVelocity,
            now_us: 1_000,
        },
    );
    let act = checked_step(
        &mut s,
        Event::EnableStart {
            group: 0,
            now_us: 2_000,
        },
    );
    assert!(act.enable_for(0).is_none());
    assert_eq!(
        act.refusal_for(0).map(|r| r.code),
        Some(RefusalCode::TimeoutMismatch)
    );
    assert_eq!(s.groups[0].torque_intent, TorqueIntent::Off);
    assert_eq!(s.groups[0].drive_state, DriveState::SwitchOnDisabled);

    // Wrong readback value -> still refused
    checked_step(
        &mut s,
        Event::TimeoutReadback {
            channel: 0,
            timeout_reg: 899,
            now_us: 3_000,
        },
    );
    checked_step(
        &mut s,
        Event::TimeoutReadback {
            channel: 1,
            timeout_reg: 900,
            now_us: 3_000,
        },
    );
    let act = checked_step(
        &mut s,
        Event::EnableStart {
            group: 0,
            now_us: 4_000,
        },
    );
    assert_eq!(
        act.refusal_for(0).map(|r| r.code),
        Some(RefusalCode::TimeoutMismatch)
    );
    assert_eq!(act.refusal_for(0).and_then(|r| r.channel), Some(0));

    // Exact readback match -> accepted, TorqueIntent::On immediately
    checked_step(
        &mut s,
        Event::TimeoutReadback {
            channel: 0,
            timeout_reg: 900,
            now_us: 5_000,
        },
    );
    let act = checked_step(
        &mut s,
        Event::EnableStart {
            group: 0,
            now_us: 6_000,
        },
    );
    assert!(act.enable_for(0).is_some());
    assert_eq!(s.groups[0].torque_intent, TorqueIntent::On);
    assert_eq!(s.groups[0].drive_state, DriveState::ReadyToSwitchOn);

    // Partial pending confirmation keeps ReadyToSwitchOn
    checked_step(
        &mut s,
        Event::HandshakeResult {
            group: 0,
            now_us: 10_000,
            confirmed_mask: ChannelMask::bit(0),
            outcome: HandshakeOutcome::Pending,
        },
    );
    assert_eq!(s.groups[0].drive_state, DriveState::ReadyToSwitchOn);

    // Full confirmation transitions to OperationEnabled
    checked_step(
        &mut s,
        Event::HandshakeResult {
            group: 0,
            now_us: 15_000,
            confirmed_mask: ChannelMask::bit(1),
            outcome: HandshakeOutcome::Confirmed,
        },
    );
    assert_eq!(s.groups[0].drive_state, DriveState::OperationEnabled);
    assert_eq!(s.groups[0].enable_phase, EnablePhase::Armed);
    assert_eq!(s.groups[0].torque_intent, TorqueIntent::On);
}

#[test]
fn enable_failure_interruption_and_expiry_enter_failed_hold_without_rollback() {
    let cases: [(&str, Event); 6] = [
        (
            "handshake_failed",
            Event::HandshakeResult {
                group: 0,
                now_us: 50_000,
                confirmed_mask: ChannelMask::EMPTY,
                outcome: HandshakeOutcome::Failed,
            },
        ),
        (
            "handshake_fault",
            Event::HandshakeResult {
                group: 0,
                now_us: 50_000,
                confirmed_mask: ChannelMask::EMPTY,
                outcome: HandshakeOutcome::FaultReported { channel: 1 },
            },
        ),
        (
            "interrupted_by_stop",
            Event::Stop {
                groups: GroupMask::bit(0),
                kind: StopKind::Stop,
                now_us: 50_000,
            },
        ),
        (
            "interrupted_by_disable_request",
            Event::DisableRequest {
                group: 0,
                now_us: 50_000,
                explicit_release_authorized: false,
                recipient_mask_override: None,
            },
        ),
        (
            "interrupted_by_mit_change",
            Event::MitOwnershipChange {
                group: 0,
                channels: ChannelMask::bit(0),
                target_mode: ControlMode::Impedance,
                device_wide: false,
                now_us: 50_000,
            },
        ),
        (
            "expired_on_release",
            Event::Release {
                now_us: 10_000 + DEFAULT_ENABLE_DEADLINE_US,
            },
        ),
    ];

    for (name, trigger) in cases {
        let mut s = TorqueState::new(single_arm_config(2)).unwrap();
        verify_all_modes(&mut s, 5_000);
        // Feed fresh samples so joints are captured
        checked_step(&mut s, Event::CausalSample(sample_at(0, 1, 9_000, 0.42)));
        checked_step(&mut s, Event::CausalSample(sample_at(1, 2, 9_000, -0.31)));
        checked_step(
            &mut s,
            Event::EnableStart {
                group: 0,
                now_us: 10_000,
            },
        );
        assert_eq!(s.groups[0].drive_state, DriveState::ReadyToSwitchOn, "{name}");

        let act = checked_step(&mut s, trigger);
        assert_eq!(s.groups[0].drive_state, DriveState::FailedHold, "{name}");
        assert_eq!(s.groups[0].enable_phase, EnablePhase::FailedHold, "{name}");
        assert_eq!(s.groups[0].torque_intent, TorqueIntent::On, "{name}");
        assert!(
            s.groups[0].faults.contains(TorqueFaultKind::EnableFailed),
            "{name}"
        );
        assert!(!act.any_disable(), "{name}: must NOT roll back with disable");
        assert_eq!(
            act.refusal_for(0).map(|r| r.code),
            Some(RefusalCode::EnableFailed),
            "{name}"
        );

        // Next fresh release emits holds for every captured joint even if reported_enabled == false
        let t_next = s.now_us + 5_000;
        let mut s0 = sample_at(0, 10, t_next, 0.43);
        s0.reported_enabled = Some(false);
        let mut s1 = sample_at(1, 11, t_next, -0.30);
        s1.reported_enabled = Some(false);
        checked_step(&mut s, Event::CausalSample(s0));
        checked_step(&mut s, Event::CausalSample(s1));
        let rel = checked_step(&mut s, Event::Release { now_us: t_next });
        assert!(rel.hold_for(0).is_some(), "{name}: ch0 must be held");
        assert!(rel.hold_for(1).is_some(), "{name}: ch1 must be held");
    }
}

#[test]
fn hold_anchoring_freshness_staleness_mode_and_reported_enable() {
    let mut s = TorqueState::new(single_arm_config(2)).unwrap();
    enable_group_clean(&mut s, 0, 10_000);

    checked_step(&mut s, Event::CausalSample(sample_at(0, 1, 20_000, 1.0)));
    checked_step(&mut s, Event::CausalSample(sample_at(1, 2, 20_000, -0.5)));
    checked_step(
        &mut s,
        Event::Stop {
            groups: GroupMask::bit(0),
            kind: StopKind::Stop,
            now_us: 25_000,
        },
    );
    assert_eq!(s.groups[0].drive_state, DriveState::QuickStopActive);

    // Anchored position stays fixed while fresh samples arrive with slight sag + reported_enabled=false
    let mut sag = sample_at(0, 3, 40_000, 1.05);
    sag.reported_enabled = Some(false);
    checked_step(&mut s, Event::CausalSample(sag));
    let rel1 = checked_step(&mut s, Event::Release { now_us: 40_000 });
    assert_eq!(rel1.hold_for(0).unwrap().position_rad, 1.0);
    assert_eq!(rel1.hold_for(1).unwrap().position_rad, -0.5);
    assert_eq!(s.groups[0].torque_intent, TorqueIntent::On);

    // Exact 100 ms boundary for ch1 (sample at 20_000, now 120_000 -> age == 100_000 <= 100 ms): still held!
    let rel2 = checked_step(
        &mut s,
        Event::Release {
            now_us: 20_000 + DEFAULT_CAPTURE_MAX_AGE_US,
        },
    );
    assert!(rel2.hold_for(1).is_some(), "age == 100 ms stays captured");

    // 1 us past 100 ms boundary -> ch1 capture dropped, reported unheld, refresh requested!
    let rel3 = checked_step(
        &mut s,
        Event::Release {
            now_us: 20_000 + DEFAULT_CAPTURE_MAX_AGE_US + 1,
        },
    );
    assert!(rel3.hold_for(1).is_none(), "age > 100 ms drops capture");
    assert!(rel3.telemetry.unheld_mask.contains(1));
    assert!(rel3.refresh_requests.contains(1));
    // ch0 had fresh sample at 40_000 -> age 80_001 <= 100_000 -> STILL held!
    assert!(rel3.hold_for(0).is_some());

    // Fresh sample returns for ch1 at -0.4 -> recaptures -0.4!
    checked_step(
        &mut s,
        Event::CausalSample(sample_at(1, 4, 130_000, -0.4)),
    );
    let rel4 = checked_step(&mut s, Event::Release { now_us: 130_000 });
    assert_eq!(rel4.hold_for(1).unwrap().position_rad, -0.4);

    // Unverified mode clears verified_mode on HoldCommand and reports channel unheld
    checked_step(
        &mut s,
        Event::MitOwnershipChange {
            group: 0,
            channels: ChannelMask::bit(0),
            target_mode: ControlMode::Impedance,
            device_wide: false,
            now_us: 135_000,
        },
    );
    let rel5 = checked_step(&mut s, Event::Release { now_us: 135_000 });
    let h0 = rel5.hold_for(0).expect("captured ch0 still gets hold intent");
    assert_eq!(h0.verified_mode, None);
    assert!(rel5.telemetry.unheld_mask.contains(0));
}

#[test]
fn fault_clear_never_disables_and_preserves_intent() {
    let mut s = TorqueState::new(single_arm_config(1)).unwrap();
    verify_all_modes(&mut s, 1_000);
    checked_step(&mut s, Event::CausalSample(sample_at(0, 1, 2_000, 0.0)));
    checked_step(
        &mut s,
        Event::EnableStart {
            group: 0,
            now_us: 3_000,
        },
    );
    checked_step(
        &mut s,
        Event::HandshakeResult {
            group: 0,
            now_us: 4_000,
            confirmed_mask: ChannelMask::EMPTY,
            outcome: HandshakeOutcome::Failed,
        },
    );
    assert_eq!(s.groups[0].drive_state, DriveState::FailedHold);
    assert_eq!(s.groups[0].torque_intent, TorqueIntent::On);

    // FaultClear while TorqueIntent::On -> NEVER disables, preserves On, moves to QuickStopActive
    let act = checked_step(
        &mut s,
        Event::FaultClear {
            groups: GroupMask::bit(0),
            now_us: 10_000,
        },
    );
    assert!(!act.any_disable());
    assert_eq!(s.groups[0].torque_intent, TorqueIntent::On);
    assert_eq!(s.groups[0].drive_state, DriveState::QuickStopActive);

    // Still holds every release
    let rel = checked_step(&mut s, Event::Release { now_us: 15_000 });
    assert!(rel.hold_for(0).is_some());

    // Can now re-enable from QuickStopActive
    let en = checked_step(
        &mut s,
        Event::EnableStart {
            group: 0,
            now_us: 20_000,
        },
    );
    assert!(en.enable_for(0).is_some());
    assert_eq!(s.groups[0].drive_state, DriveState::ReadyToSwitchOn);

    // Thermal active refuses FaultClear
    checked_step(
        &mut s,
        Event::Thermal {
            channel: 0,
            temperature_c: 75.0,
            now_us: 25_000,
        },
    );
    let clr_hot = checked_step(
        &mut s,
        Event::FaultClear {
            groups: GroupMask::bit(0),
            now_us: 30_000,
        },
    );
    assert_eq!(
        clr_hot.refusal_for(0).map(|r| r.code),
        Some(RefusalCode::ThermalActive)
    );
}

#[test]
fn rest_gate_k3_boundaries_causality_feetech_and_fashionstar() {
    // 1. Exact settle boundary (> 20 ms), stationary hold preservation, duplicate/same-release/drift/future/NaN,
    //    exact span >= 20 ms, non-rest WOULD_DROP, and 1 s timeout.
    let mut s = TorqueState::new(single_arm_config(1)).unwrap();
    enable_group_clean(&mut s, 0, 1_000);

    // Motion write completes at t = 10_000
    checked_step(
        &mut s,
        Event::WriteResult(WriteResult::uniform(
            0,
            0,
            WriteKind::Motion,
            0,
            10_000,
            10_000,
            ChannelMask::bit(0),
            MotorWriteOutcome::Sent,
        )),
    );
    checked_step(
        &mut s,
        Event::DisableRequest {
            group: 0,
            now_us: 20_000,
            explicit_release_authorized: false,
            recipient_mask_override: None,
        },
    );

    // Stationary hold write at 25_000 MUST preserve motion write boundary (10_000)!
    checked_step(
        &mut s,
        Event::WriteResult(WriteResult::uniform(
            0,
            0,
            WriteKind::Hold,
            0,
            25_000,
            25_000,
            ChannelMask::bit(0),
            MotorWriteOutcome::Sent,
        )),
    );
    assert_eq!(s.joints[0].last_motion_write_us, Some(10_000));

    // Sample at exact t = 10_000 + 20_000 = 30_000 (NOT strictly > 20 ms): rejected!
    checked_step(&mut s, Event::Release { now_us: 30_000 });
    checked_step(&mut s, Event::CausalSample(sample_at(0, 1, 10_000 + REST_SETTLE_US, 0.0)));
    assert_eq!(s.rest_windows[0].count, 0);

    // Sample at 30_001 (strictly > 20 ms): accepted -> count = 1
    checked_step(&mut s, Event::Release { now_us: 30_001 });
    checked_step(
        &mut s,
        Event::CausalSample(sample_at(0, 2, 10_000 + REST_SETTLE_US + 1, 0.0)),
    );
    assert_eq!(s.rest_windows[0].count, 1);

    // Same release second sample -> does NOT increment count
    checked_step(
        &mut s,
        Event::CausalSample(sample_at(0, 3, 30_001, 0.0)),
    );
    assert_eq!(s.rest_windows[0].count, 1);

    // Next release, duplicate sample_id (2) -> does NOT increment count
    checked_step(&mut s, Event::Release { now_us: 40_000 });
    checked_step(&mut s, Event::CausalSample(sample_at(0, 2, 40_000, 0.0)));
    assert_eq!(s.rest_windows[0].count, 1);

    // Future-dated sample (rx_ts = now_us + 1) -> rejected without clamping, resets window!
    let mut future = sample_at(0, 4, 40_000, 0.0);
    future.rx_timestamp_us = 40_001;
    checked_step(&mut s, Event::CausalSample(future));
    assert_eq!(s.rest_windows[0].count, 0);

    // Clock offset drift == 1_000 us accepted; 1_001 us (> 1 ms) resets window!
    checked_step(&mut s, Event::Release { now_us: 50_000 });
    let mut drift_ok = sample_at(0, 5, 50_000, 0.0);
    drift_ok.clock_offset_drift_us = CLOCK_OFFSET_MAX_DRIFT_US;
    checked_step(&mut s, Event::CausalSample(drift_ok));
    assert_eq!(s.rest_windows[0].count, 1);

    checked_step(&mut s, Event::Release { now_us: 60_000 });
    let mut drift_bad = sample_at(0, 6, 60_000, 0.0);
    drift_bad.clock_offset_drift_us = CLOCK_OFFSET_MAX_DRIFT_US + 1;
    checked_step(&mut s, Event::CausalSample(drift_bad));
    assert_eq!(s.rest_windows[0].count, 0);

    // NaN resets window
    checked_step(&mut s, Event::Release { now_us: 70_000 });
    checked_step(&mut s, Event::CausalSample(sample_at(0, 7, 70_000, 0.0)));
    assert_eq!(s.rest_windows[0].count, 1);
    checked_step(&mut s, Event::Release { now_us: 80_000 });
    checked_step(&mut s, Event::CausalSample(sample_at(0, 8, 80_000, f32::NAN)));
    assert_eq!(s.rest_windows[0].count, 0);

    // 3 distinct samples spanning 19_999 us (< 20 ms) -> NOT yet authorized; 4th at +20_000 us -> AUTHORIZED!
    checked_step(&mut s, Event::Release { now_us: 100_000 });
    checked_step(&mut s, Event::CausalSample(sample_at(0, 10, 100_000, 0.0)));
    checked_step(&mut s, Event::Release { now_us: 110_000 });
    checked_step(&mut s, Event::CausalSample(sample_at(0, 11, 110_000, 0.0)));
    checked_step(&mut s, Event::Release { now_us: 119_999 });
    let act_short = checked_step(&mut s, Event::CausalSample(sample_at(0, 12, 119_999, 0.0)));
    assert!(
        act_short.disable_for(0).is_none(),
        "span 19_999 us < 20 ms must NOT pass"
    );

    checked_step(
        &mut s,
        Event::Release {
            now_us: 100_000 + REST_WINDOW_MIN_SPAN_US,
        },
    );
    let act_pass = checked_step(
        &mut s,
        Event::CausalSample(sample_at(0, 13, 100_000 + REST_WINDOW_MIN_SPAN_US, 0.0)),
    );
    let dcmd = act_pass.disable_for(0).expect("span == 20 ms passes rest gate");
    assert_eq!(dcmd.kind, DisableKind::Requested);
    assert_eq!(s.groups[0].torque_intent, TorqueIntent::On);

    // 2. Definite non-rest pose -> TORQUE_OFF_WOULD_DROP
    let mut s2 = TorqueState::new(single_arm_config(1)).unwrap();
    enable_group_clean(&mut s2, 0, 1_000);
    checked_step(
        &mut s2,
        Event::DisableRequest {
            group: 0,
            now_us: 5_000,
            explicit_release_authorized: false,
            recipient_mask_override: None,
        },
    );
    checked_step(&mut s2, Event::Release { now_us: 30_000 });
    let bad_pose = checked_step(&mut s2, Event::CausalSample(sample_at(0, 1, 30_000, 0.50)));
    assert_eq!(
        bad_pose.refusal_for(0).map(|r| r.code),
        Some(RefusalCode::TorqueOffWouldDrop)
    );
    assert!(!bad_pose.any_disable());
    assert_eq!(s2.groups[0].torque_intent, TorqueIntent::On);

    // 3. Missing evidence defers up to 1 s then expires
    let mut s3 = TorqueState::new(single_arm_config(1)).unwrap();
    enable_group_clean(&mut s3, 0, 1_000);
    checked_step(
        &mut s3,
        Event::DisableRequest {
            group: 0,
            now_us: 10_000,
            explicit_release_authorized: false,
            recipient_mask_override: None,
        },
    );
    let before = checked_step(
        &mut s3,
        Event::Release {
            now_us: 10_000 + DEFAULT_DISABLE_DEADLINE_US - 1,
        },
    );
    assert!(s3.groups[0].disable_tx.is_some());
    assert!(before.refusal_for(0).is_none());
    let at_dl = checked_step(
        &mut s3,
        Event::Release {
            now_us: 10_000 + DEFAULT_DISABLE_DEADLINE_US,
        },
    );
    assert!(s3.groups[0].disable_tx.is_none());
    assert_eq!(
        at_dl.refusal_for(0).map(|r| r.code),
        Some(RefusalCode::RestGateTimeout)
    );
    assert_eq!(s3.groups[0].torque_intent, TorqueIntent::On);

    // 4. FashionStar unqualified rest evidence refused immediately
    let mut fs_cfg = single_arm_config(1);
    fs_cfg.channels[0].vendor = VendorKind::FashionStar;
    fs_cfg.recipients[0].vendor = VendorKind::FashionStar;
    fs_cfg.recipients[0].rest_evidence_qualified = false;
    let mut s_fs = TorqueState::new(fs_cfg).unwrap();
    enable_group_clean(&mut s_fs, 0, 1_000);
    let act_fs = checked_step(
        &mut s_fs,
        Event::DisableRequest {
            group: 0,
            now_us: 10_000,
            explicit_release_authorized: false,
            recipient_mask_override: None,
        },
    );
    assert_eq!(
        act_fs.refusal_for(0).map(|r| r.code),
        Some(RefusalCode::RestEvidenceUnqualified)
    );
    assert_eq!(s_fs.groups[0].torque_intent, TorqueIntent::On);

    // 5. Feetech serial timeout poisons line until 20 ms of silence, then resumes within 1 s
    let mut ft_cfg = single_arm_config(1);
    ft_cfg.devices[0].poison_silence_us = FEETECH_POISON_SILENCE_US;
    ft_cfg.channels[0].vendor = VendorKind::Feetech;
    ft_cfg.recipients[0].vendor = VendorKind::Feetech;
    let mut s_ft = TorqueState::new(ft_cfg).unwrap();
    enable_group_clean(&mut s_ft, 0, 1_000);
    checked_step(
        &mut s_ft,
        Event::DisableRequest {
            group: 0,
            now_us: 10_000,
            explicit_release_authorized: false,
            recipient_mask_override: None,
        },
    );
    checked_step(&mut s_ft, Event::Release { now_us: 40_000 });
    checked_step(&mut s_ft, Event::CausalSample(sample_at(0, 1, 40_000, 0.0)));
    assert_eq!(s_ft.rest_windows[0].count, 1);

    // SerialTimeout at 50_000 -> poisoned until 70_000, resets window!
    checked_step(
        &mut s_ft,
        Event::SerialTimeout {
            device: 0,
            now_us: 50_000,
        },
    );
    assert_eq!(s_ft.rest_windows[0].count, 0);

    // Late reply at 60_000 breaks silence -> extends poison to 80_000, rejected!
    checked_step(&mut s_ft, Event::Release { now_us: 60_000 });
    checked_step(&mut s_ft, Event::CausalSample(sample_at(0, 2, 60_000, 0.0)));
    assert_eq!(s_ft.rest_windows[0].count, 0);
    assert_eq!(s_ft.devices[0].poisoned_until_us, 80_000);

    // After full 20 ms silence (t = 80_000, 90_000, 100_000), 3 clean releases pass!
    for (i, t) in [(3u64, 80_000u64), (4, 90_000), (5, 100_000)] {
        checked_step(&mut s_ft, Event::Release { now_us: t });
        let a = checked_step(&mut s_ft, Event::CausalSample(sample_at(0, i, t, 0.0)));
        if i == 5 {
            assert!(a.disable_for(0).is_some(), "resumed Feetech should pass");
        }
    }
}

#[test]
fn dispatch_commit_bounded_repeats_fencing_and_partial_outcomes() {
    let mut s = TorqueState::new(single_arm_config(2)).unwrap();
    enable_group_clean(&mut s, 0, 1_000);
    checked_step(
        &mut s,
        Event::DisableRequest {
            group: 0,
            now_us: 10_000,
            explicit_release_authorized: false,
            recipient_mask_override: None,
        },
    );

    // Motion and enable are fenced while disable tx is active
    let m_ref = checked_step(
        &mut s,
        Event::MotionCommand {
            group: 0,
            channels: ChannelMask::from_count(2),
            now_us: 15_000,
        },
    );
    assert_eq!(
        m_ref.refusal_for(0).map(|r| r.code),
        Some(RefusalCode::DisableInProgress)
    );
    let e_ref = checked_step(
        &mut s,
        Event::EnableStart {
            group: 0,
            now_us: 16_000,
        },
    );
    assert_eq!(
        e_ref.refusal_for(0).map(|r| r.code),
        Some(RefusalCode::DisableInProgress)
    );

    // Pass rest gate on 3 releases
    let mut auth = None;
    for (k, t) in [(1u64, 40_000u64), (2, 50_000), (3, 60_000)] {
        checked_step(&mut s, Event::Release { now_us: t });
        checked_step(&mut s, Event::CausalSample(sample_at(0, k * 10, t, 0.0)));
        let a = checked_step(&mut s, Event::CausalSample(sample_at(1, k * 10 + 1, t, 0.0)));
        if k == 3 {
            auth = a.disable_for(0);
        }
    }
    let d0 = auth.expect("gate authorized");
    assert_eq!(d0.repeat_index, 0);
    assert_eq!(s.groups[0].torque_intent, TorqueIntent::On);

    // Zero writes (all Deferred) keeps TorqueIntent::On!
    checked_step(
        &mut s,
        Event::WriteResult(WriteResult::uniform(
            0,
            0,
            WriteKind::DisableRequested,
            d0.generation,
            61_000,
            61_000,
            ChannelMask::from_count(2),
            MotorWriteOutcome::Deferred,
        )),
    );
    assert_eq!(s.groups[0].torque_intent, TorqueIntent::On);

    // Next release re-authorizes (still fresh & valid)
    let rel_reauth = checked_step(&mut s, Event::Release { now_us: 70_000 });
    let d1 = rel_reauth.disable_for(0).expect("re-authorized on next release");

    // Partial + Ambiguous write on repeat 0: ch0 Ambiguous, ch1 Failed -> IMMEDIATELY commits Off!
    let mut partial = WriteResult::uniform(
        0,
        0,
        WriteKind::DisableRequested,
        d1.generation,
        71_000,
        71_000,
        ChannelMask::from_count(2),
        MotorWriteOutcome::Failed,
    );
    partial.outcomes[0] = MotorWriteOutcome::Ambiguous;
    checked_step(&mut s, Event::WriteResult(partial));
    assert_eq!(
        s.groups[0].torque_intent,
        TorqueIntent::Off,
        "first possibly-sent disable commits Off immediately"
    );

    // Repeat 1 at +20 ms (91_000) and Repeat 2 at +20 ms (112_000); no holds emitted since Off!
    let r_early = checked_step(&mut s, Event::Release { now_us: 90_999 });
    assert!(!r_early.any_hold());
    assert!(r_early.disable_for(0).is_none(), "before 20 ms repeat interval");

    let r_rep1 = checked_step(&mut s, Event::Release { now_us: 91_000 });
    assert_eq!(r_rep1.disable_for(0).unwrap().repeat_index, 1);
    checked_step(
        &mut s,
        Event::WriteResult(WriteResult::uniform(
            0,
            0,
            WriteKind::DisableRequested,
            d1.generation,
            92_000,
            92_000,
            ChannelMask::from_count(2),
            MotorWriteOutcome::Sent,
        )),
    );

    let r_rep2 = checked_step(&mut s, Event::Release { now_us: 112_000 });
    assert_eq!(r_rep2.disable_for(0).unwrap().repeat_index, 2);
    let done = checked_step(
        &mut s,
        Event::WriteResult(WriteResult::uniform(
            0,
            0,
            WriteKind::DisableRequested,
            d1.generation,
            113_000,
            113_000,
            ChannelMask::from_count(2),
            MotorWriteOutcome::Sent,
        )),
    );
    let comp = done.completion_for(0).expect("completed after 3 repeats");
    assert!(comp.committed_off);
    assert_eq!(comp.repeats_completed, DAMIAO_DISABLE_REPEATS);
    assert!(
        comp.partial_or_ambiguous,
        "ambiguous/failed outcomes from repeat 0 must stay surfaced"
    );
    assert!(comp.ambiguous_mask.contains(0));
    assert!(comp.failed_mask.contains(1));
    assert_eq!(s.groups[0].drive_state, DriveState::SwitchOnDisabled);
    assert!(s.groups[0].disable_tx.is_none());
}

#[test]
fn thermal_linear_derate_protective_damping_and_n1_preemption() {
    // 1. Default ThermalConfig: no derate before trip
    let def_t = ThermalConfig::default();
    assert_eq!(compute_thermal_derate_scale(25.0, def_t), 1.0);
    assert_eq!(compute_thermal_derate_scale(69.9, def_t), 1.0);
    assert_eq!(compute_thermal_derate_scale(70.0, def_t), 0.0);

    // 2. Configured linear derate: 50 C -> 70 C
    let lin_t = ThermalConfig {
        derate_start_c: Some(50.0),
        trip_c: 70.0,
    };
    assert!((compute_thermal_derate_scale(50.0, lin_t) - 1.0).abs() < 1e-6);
    assert!((compute_thermal_derate_scale(60.0, lin_t) - 0.5).abs() < 1e-6);
    assert!((compute_thermal_derate_scale(65.0, lin_t) - 0.25).abs() < 1e-6);
    assert_eq!(compute_thermal_derate_scale(70.0, lin_t), 0.0);

    // 3. Protective MIT damping precedes DisableProtective ONLY when configured & verified Impedance
    let mut cfg = single_arm_config(1);
    cfg.channels[0].desired_mode = ControlMode::Impedance;
    cfg.channels[0].supports_mit_damping = true;
    cfg.channels[0].damping = DampingConfig {
        duration_us: 40_000,
        kd: 1.5,
        max_torque_nm: 4.0,
    };
    cfg.channels[0].thermal = lin_t;
    let mut s = TorqueState::new(cfg).unwrap();
    enable_group_clean(&mut s, 0, 1_000);

    // 60 C -> derate 0.5, still operational
    let a_warm = checked_step(
        &mut s,
        Event::Thermal {
            channel: 0,
            temperature_c: 60.0,
            now_us: 10_000,
        },
    );
    assert!((a_warm.telemetry.thermal_derate_scales[0] - 0.5).abs() < 1e-6);
    assert_eq!(s.groups[0].drive_state, DriveState::OperationEnabled);

    // 70 C -> thermal trip enters ProtectiveDamping with kd=1.5, kp=0.0, reserved budget!
    let a_trip = checked_step(
        &mut s,
        Event::Thermal {
            channel: 0,
            temperature_c: 70.0,
            now_us: 20_000,
        },
    );
    assert_eq!(s.groups[0].drive_state, DriveState::ProtectiveDamping);
    let damp = a_trip.damp_for(0).expect("MIT damping emitted");
    assert_eq!(damp.kp, 0.0);
    assert_eq!(damp.kd, 1.5);
    assert_eq!(damp.max_torque_nm, 4.0);
    assert!(damp.use_reserved_budget);
    assert!(a_trip.disable_for(0).is_none());

    // Mid-damping release at 40_000 -> still damping
    let r_mid = checked_step(&mut s, Event::Release { now_us: 40_000 });
    assert!(r_mid.damp_for(0).is_some());
    assert!(r_mid.disable_for(0).is_none());

    // End of damping at 60_000 (20_000 + 40_000) -> transitions immediately to DisableProtective!
    let r_end = checked_step(&mut s, Event::Release { now_us: 60_000 });
    assert_eq!(s.groups[0].drive_state, DriveState::ProtectiveDisabling);
    let dcmd = r_end.disable_for(0).expect("DisableProtective scheduled");
    assert_eq!(dcmd.kind, DisableKind::Protective);
    assert!(dcmd.use_reserved_budget);

    // 4. When verified mode is PositionVelocity (NOT Impedance), damping is skipped -> immediate N=1 DisableProtective!
    let mut cfg_pv = single_arm_config(1);
    cfg_pv.channels[0].damping = DampingConfig {
        duration_us: 40_000,
        kd: 1.5,
        max_torque_nm: 4.0,
    };
    let mut s_pv = TorqueState::new(cfg_pv).unwrap();
    enable_group_clean(&mut s_pv, 0, 1_000);
    let a_imm = checked_step(
        &mut s_pv,
        Event::Thermal {
            channel: 0,
            temperature_c: 71.0,
            now_us: 20_000,
        },
    );
    assert_eq!(s_pv.groups[0].drive_state, DriveState::ProtectiveDisabling);
    assert!(a_imm.damp_for(0).is_none());
    let d_imm = a_imm.disable_for(0).expect("immediate protective disable");
    assert_eq!(d_imm.kind, DisableKind::Protective);
    assert!(d_imm.use_reserved_budget);
    // Next serviced release (N=1) keeps protective disable scheduled if not yet written
    let r_n1 = checked_step(&mut s_pv, Event::Release { now_us: 40_000 });
    assert!(r_n1
        .disable_for(0)
        .is_some_and(|d| d.kind == DisableKind::Protective && d.use_reserved_budget));
}

#[test]
fn fu_panic_tiers_and_ordinary_bus_loss() {
    // 1. Ordinary BusLoss grants NO protective authority
    let mut s = TorqueState::new(single_arm_config(2)).unwrap();
    enable_group_clean(&mut s, 0, 1_000);
    checked_step(&mut s, Event::CausalSample(sample_at(0, 1, 5_000, 0.2)));
    let bl = checked_step(
        &mut s,
        Event::BusLoss {
            device: 0,
            now_us: 10_000,
        },
    );
    assert!(!s.groups[0].protective_latch);
    assert!(!bl.any_disable());
    assert_eq!(s.groups[0].torque_intent, TorqueIntent::On);
    assert_eq!(s.groups[0].drive_state, DriveState::Fault);

    // 2. Tier 1 WorkerPanic: arm parks, failed park keeps holding; succeeded park -> rest-gated disable
    let mut s1 = TorqueState::new(TorqueConfig::arm_plus_underactuated_hand()).unwrap();
    enable_group_clean(&mut s1, 0, 1_000);
    enable_group_clean(&mut s1, 1, 1_000);
    for c in 0..12u8 {
        checked_step(&mut s1, Event::CausalSample(sample_at(c, c as u64 + 1, 5_000, 0.0)));
    }
    let wp = checked_step(
        &mut s1,
        Event::Panic {
            tier: PanicTier::WorkerPanic { thread_tag: 7 },
            now_us: 10_000,
        },
    );
    assert!(wp.park_requests.contains(0), "arm group requests park");
    assert!(!wp.park_requests.contains(1), "hand group must NOT park");
    assert_eq!(s1.groups[0].drive_state, DriveState::FaultReactionActive);
    assert_eq!(s1.groups[1].drive_state, DriveState::Fault);

    // New commands fenced with INTERNAL_FAULT
    let m_fenced = checked_step(
        &mut s1,
        Event::MotionCommand {
            group: 0,
            channels: ChannelMask::from_count(6),
            now_us: 12_000,
        },
    );
    assert_eq!(
        m_fenced.refusal_for(0).map(|r| r.code),
        Some(RefusalCode::InternalFault)
    );

    // Park failure keeps holding and emits 0 disables
    let mut s1_fail = s1;
    let pf = checked_step(
        &mut s1_fail,
        Event::ParkSettled {
            group: 0,
            succeeded: false,
            now_us: 20_000,
        },
    );
    assert!(!pf.any_disable());
    assert_eq!(s1_fail.groups[0].torque_intent, TorqueIntent::On);
    assert_eq!(s1_fail.groups[0].drive_state, DriveState::Fault);

    // Park success opens REST-GATED disable on arm ONLY; hand group stays holding!
    checked_step(
        &mut s1,
        Event::ParkSettled {
            group: 0,
            succeeded: true,
            now_us: 20_000,
        },
    );
    assert!(s1.groups[0].disable_tx.is_some());
    assert!(s1.groups[1].disable_tx.is_none());
    assert_eq!(s1.groups[1].torque_intent, TorqueIntent::On);

    // 3. Tier 2 CaughtControlPanic -> SafeHold; holds continue; cuts ONLY on thermal or bus-loss limit
    let mut s2 = TorqueState::new(single_arm_config(1)).unwrap();
    enable_group_clean(&mut s2, 0, 1_000);
    checked_step(&mut s2, Event::CausalSample(sample_at(0, 1, 5_000, 0.3)));
    checked_step(
        &mut s2,
        Event::Panic {
            tier: PanicTier::CaughtControlPanic,
            now_us: 10_000,
        },
    );
    assert_eq!(s2.groups[0].drive_state, DriveState::SafeHold);
    // Refuses DisableRequest and FaultClear
    let dr = checked_step(
        &mut s2,
        Event::DisableRequest {
            group: 0,
            now_us: 15_000,
            explicit_release_authorized: true,
            recipient_mask_override: None,
        },
    );
    assert_eq!(
        dr.refusal_for(0).map(|r| r.code),
        Some(RefusalCode::ControlPanicSafeHold)
    );
    // Fresh release holds at 0.3
    let r_hold = checked_step(&mut s2, Event::Release { now_us: 20_000 });
    assert_eq!(r_hold.hold_for(0).unwrap().position_rad, 0.3);
    assert!(!r_hold.any_disable());

    // Lose feedback (> 100 ms -> bus loss timer starts at 110_000); before limit no cut; at limit -> Protective!
    let r_lost = checked_step(&mut s2, Event::Release { now_us: 110_000 });
    assert!(!r_lost.any_disable());
    let r_just_before = checked_step(
        &mut s2,
        Event::Release {
            now_us: 110_000 + DEFAULT_PANIC_BUS_LOSS_LIMIT_US - 1,
        },
    );
    assert!(!r_just_before.any_disable());
    let r_trip = checked_step(
        &mut s2,
        Event::Release {
            now_us: 110_000 + DEFAULT_PANIC_BUS_LOSS_LIMIT_US,
        },
    );
    let dcmd = r_trip
        .disable_for(0)
        .expect("SafeHold cuts at panic bus-loss limit");
    assert_eq!(dcmd.kind, DisableKind::Protective);
    assert_eq!(dcmd.origin, DisableOrigin::ProtectivePanicBusLoss);
    assert!(dcmd.use_reserved_budget);

    // 4. Tier 3 ProcessDeath
    let mut s3 = TorqueState::new(single_arm_config(1)).unwrap();
    checked_step(
        &mut s3,
        Event::Panic {
            tier: PanicTier::ProcessDeath,
            now_us: 5_000,
        },
    );
    assert!(s3.process_death_recorded);
}

#[test]
fn contention_escalation_25_releases_and_shared_bus_budgets() {
    let mut cfg = TorqueConfig::arm_plus_underactuated_hand();
    // Put both devices on shared bus_id = 0 to verify shared-bus budget accounting
    cfg.devices[0].bus_id = 0;
    cfg.devices[1].bus_id = 0;
    let mut s = TorqueState::new(cfg).unwrap();
    enable_group_clean(&mut s, 0, 1_000);
    checked_step(
        &mut s,
        Event::Stop {
            groups: GroupMask::bit(0),
            kind: StopKind::Stop,
            now_us: 2_000,
        },
    );

    for i in 1..CONTENTION_ESCALATION_RELEASES {
        let act = checked_step(
            &mut s,
            Event::WriteResult(WriteResult::uniform(
                0,
                0,
                WriteKind::Hold,
                1,
                2_000 + i as u64 * 20_000,
                2_000 + i as u64 * 20_000,
                ChannelMask::from_count(6),
                MotorWriteOutcome::LockContended,
            )),
        );
        assert!(act.contention_escalations.is_empty());
        assert!(!s.groups[0].faults.contains(TorqueFaultKind::DriverWriteFailed));
    }
    // 25th skip escalates and latches DriverWriteFailed (non-protective, TorqueIntent stays On!)
    let esc = checked_step(
        &mut s,
        Event::WriteResult(WriteResult::uniform(
            0,
            0,
            WriteKind::Hold,
            1,
            600_000,
            600_000,
            ChannelMask::from_count(6),
            MotorWriteOutcome::LockContended,
        )),
    );
    assert!(esc.contention_escalations.contains(0));
    assert!(s.groups[0].faults.contains(TorqueFaultKind::DriverWriteFailed));
    assert!(!s.groups[0].protective_latch);
    assert_eq!(s.groups[0].torque_intent, TorqueIntent::On);

    // Successful write resets counter
    checked_step(
        &mut s,
        Event::WriteResult(WriteResult::uniform(
            0,
            0,
            WriteKind::Hold,
            1,
            620_000,
            620_000,
            ChannelMask::from_count(6),
            MotorWriteOutcome::Sent,
        )),
    );
    assert_eq!(s.devices[0].consecutive_contention_skips, 0);
}

#[test]
fn active_tool_stop_output_and_bounded_retention() {
    let mut cfg = single_arm_config(2);
    let stop_out = ActiveToolSafeOutput::StopOutput;
    let retain_out = ActiveToolSafeOutput::BoundedRetention {
        max_retention_us: 100_000,
    };
    cfg.channels[0].role = JointRole::ActiveTool(stop_out);
    cfg.channels[0].active_tool_qualification = ActiveToolConfig::Qualified(stop_out);
    cfg.recipients[0].role = JointRole::ActiveTool(stop_out);

    cfg.channels[1].role = JointRole::ActiveTool(retain_out);
    cfg.channels[1].active_tool_qualification = ActiveToolConfig::Qualified(retain_out);
    cfg.recipients[1].role = JointRole::ActiveTool(retain_out);

    let mut s = TorqueState::new(cfg).unwrap();
    enable_group_clean(&mut s, 0, 1_000);
    checked_step(&mut s, Event::CausalSample(sample_at(0, 1, 5_000, 0.0)));
    checked_step(&mut s, Event::CausalSample(sample_at(1, 2, 5_000, 0.0)));
    checked_step(
        &mut s,
        Event::Stop {
            groups: GroupMask::bit(0),
            kind: StopKind::Stop,
            now_us: 10_000,
        },
    );

    // Within retention window (at 50_000, elapsed 40_000 < 100_000)
    let r1 = checked_step(&mut s, Event::Release { now_us: 50_000 });
    assert_eq!(r1.hold_for(0).unwrap().hold_kind, HoldKind::ActiveToolStop);
    assert_eq!(
        r1.hold_for(1).unwrap().hold_kind,
        HoldKind::ActiveToolRetain {
            remaining_us: 60_000
        }
    );

    // Without explicit release authorization, disable is refused for BoundedRetention
    let dr = checked_step(
        &mut s,
        Event::DisableRequest {
            group: 0,
            now_us: 60_000,
            explicit_release_authorized: false,
            recipient_mask_override: None,
        },
    );
    assert_eq!(
        dr.refusal_for(0).map(|r| r.code),
        Some(RefusalCode::ActiveToolReleaseNotAuthorized)
    );

    // Past retention bound (at 120_000, elapsed 110_000 >= 100_000) -> transitions to ActiveToolStop
    checked_step(&mut s, Event::CausalSample(sample_at(0, 3, 115_000, 0.0)));
    checked_step(&mut s, Event::CausalSample(sample_at(1, 4, 115_000, 0.0)));
    let r2 = checked_step(&mut s, Event::Release { now_us: 120_000 });
    assert_eq!(r2.hold_for(1).unwrap().hold_kind, HoldKind::ActiveToolStop);
}

// =========================================================================
// 2. Synthetic Underactuated Hand Group Next to Arm Group
// =========================================================================

#[test]
fn synthetic_underactuated_hand_next_to_arm_group() {
    let mut s = TorqueState::new(TorqueConfig::arm_plus_underactuated_hand()).unwrap();
    verify_all_modes(&mut s, 1_000);

    // Enable arm (group 0) and hand (group 1)
    enable_group_clean(&mut s, 0, 2_000);

    // Hand enable completed by whole-hand ACK (is_handshake_ack_only = true):
    // completes handshake WITHOUT creating position captures or rest evidence!
    checked_step(
        &mut s,
        Event::EnableStart {
            group: 1,
            now_us: 10_000,
        },
    );
    for c in 6..12u8 {
        let mut ack = sample_at(c, 500, 12_000, 0.0);
        ack.is_handshake_ack_only = true;
        ack.reported_enabled = Some(true);
        checked_step(&mut s, Event::CausalSample(ack));
        assert!(
            s.joints[c as usize].captured_position_rad.is_none(),
            "whole-hand ACK must NOT populate position capture"
        );
    }
    assert_eq!(s.groups[1].drive_state, DriveState::OperationEnabled);

    // Set bounded grasp state on hand while operational
    checked_step(
        &mut s,
        Event::GraspStateUpdate {
            channel: 6,
            setpoint_rad: -0.8,
            force_limit_raw: Some(220),
            now_us: 15_000,
        },
    );

    // Feed fresh positions for arm (0..6), hand (6..12), and passive tactile sensor (12)
    for c in 0..13u8 {
        let mut smp = sample_at(c, 100 + c as u64, 20_000, if c < 6 { 0.0 } else { -0.8 });
        if (6..12).contains(&c) {
            smp.release_evidence = Some(ReleaseEvidence::HoldingUnsupportedPayload);
        }
        checked_step(&mut s, Event::CausalSample(smp));
    }

    // Stop BOTH groups; hand must freeze without raising force even if GraspStateUpdate is sent
    checked_step(
        &mut s,
        Event::Stop {
            groups: GroupMask::bit(0).union(GroupMask::bit(1)),
            kind: StopKind::Stop,
            now_us: 25_000,
        },
    );
    checked_step(
        &mut s,
        Event::GraspStateUpdate {
            channel: 6,
            setpoint_rad: 0.0, // attempt to open
            force_limit_raw: Some(900), // attempt to raise force
            now_us: 26_000,
        },
    );
    let rel = checked_step(&mut s, Event::Release { now_us: 27_000 });
    assert_eq!(
        rel.hold_for(6).unwrap().hold_kind,
        HoldKind::GripperFreeze {
            max_force_limit_raw: Some(220)
        }
    );
    assert!(rel.hold_for(12).is_none(), "passive sensor never commanded");

    // 1) Disable hand WITHOUT explicit release authorization -> refused!
    let no_auth = checked_step(
        &mut s,
        Event::DisableRequest {
            group: 1,
            now_us: 30_000,
            explicit_release_authorized: false,
            recipient_mask_override: None,
        },
    );
    assert_eq!(
        no_auth.refusal_for(1).map(|r| r.code),
        Some(RefusalCode::HandReleaseNotAuthorized)
    );

    // 2) Invalid recipient mask bleeding into arm group -> refused!
    let bad_mask = checked_step(
        &mut s,
        Event::DisableRequest {
            group: 1,
            now_us: 31_000,
            explicit_release_authorized: true,
            recipient_mask_override: Some(ChannelMask::bit(0).union(ChannelMask::bit(6))),
        },
    );
    assert_eq!(
        bad_mask.refusal_for(1).map(|r| r.code),
        Some(RefusalCode::RecipientMaskInvalid)
    );

    // 3) Explicit release auth + HoldingUnsupportedPayload -> TORQUE_OFF_WOULD_DROP!
    checked_step(
        &mut s,
        Event::DisableRequest {
            group: 1,
            now_us: 35_000,
            explicit_release_authorized: true,
            // Passing only recipient 6 expands via grouped_packet_mask to BOTH {6, 7}!
            recipient_mask_override: Some(ChannelMask::bit(6)),
        },
    );
    assert_eq!(
        s.groups[1].disable_tx.unwrap().recipient_mask,
        ChannelMask::bit(6).union(ChannelMask::bit(7))
    );
    checked_step(&mut s, Event::Release { now_us: 60_000 });
    let mut unsup = sample_at(6, 200, 60_000, -0.8);
    unsup.release_evidence = Some(ReleaseEvidence::HoldingUnsupportedPayload);
    let drop_ref = checked_step(&mut s, Event::CausalSample(unsup));
    assert_eq!(
        drop_ref.refusal_for(1).map(|r| r.code),
        Some(RefusalCode::TorqueOffWouldDrop)
    );

    // 4) Explicit release auth + SupportedObject across ALL 6 finger joints of grouped recipients {6, 7}
    //    over 3 distinct releases (>= 20 ms) -> disables hand ONLY; arm stays On and held!
    checked_step(
        &mut s,
        Event::DisableRequest {
            group: 1,
            now_us: 65_000,
            explicit_release_authorized: true,
            recipient_mask_override: Some(ChannelMask::bit(6)),
        },
    );
    let mut hand_disable = None;
    for (k, t) in [(1u64, 70_000u64), (2, 80_000), (3, 90_000)] {
        // Keep arm samples fresh too
        let rel_k = checked_step(&mut s, Event::Release { now_us: t });
        for arm_c in 0..6u8 {
            assert!(rel_k.hold_for(arm_c).is_some(), "arm held during hand gate");
            checked_step(&mut s, Event::CausalSample(sample_at(arm_c, 300 + k * 20 + arm_c as u64, t, 0.0)));
        }
        // Device snapshot for hand shares one sample_id per release across ch 6..12
        for hc in 6..12u8 {
            let mut hs = sample_at(hc, 700 + k, t, -0.8);
            hs.release_evidence = Some(ReleaseEvidence::SupportedObject);
            let act = checked_step(&mut s, Event::CausalSample(hs));
            if let Some(d) = act.disable_for(1) {
                hand_disable = Some(d);
            }
        }
    }
    let hd = hand_disable.expect("grouped hand disable authorized when both recipients pass");
    assert_eq!(hd.recipients, ChannelMask::bit(6).union(ChannelMask::bit(7)));

    checked_step(
        &mut s,
        Event::WriteResult(WriteResult::uniform(
            1,
            1,
            WriteKind::DisableRequested,
            hd.generation,
            91_000,
            91_000,
            hd.recipients,
            MotorWriteOutcome::Sent,
        )),
    );
    assert_eq!(s.groups[1].torque_intent, TorqueIntent::Off);
    assert_eq!(s.groups[1].drive_state, DriveState::SwitchOnDisabled);
    assert_eq!(s.groups[0].torque_intent, TorqueIntent::On);

    // Next release: arm holds active, hand has zero holds!
    let rel_after = checked_step(&mut s, Event::Release { now_us: 100_000 });
    for arm_c in 0..6u8 {
        assert!(rel_after.hold_for(arm_c).is_some());
    }
    for hand_c in 6..12u8 {
        assert!(rel_after.hold_for(hand_c).is_none());
    }

    // 5) Device-wide mode change on Device 1 fences Group 1 without fencing Group 0
    checked_step(
        &mut s,
        Event::MitOwnershipChange {
            group: 1,
            channels: ChannelMask::bit(6),
            target_mode: ControlMode::PositionForce,
            device_wide: false, // device 1 has device_wide_mode_switch = true
            now_us: 105_000,
        },
    );
    let en_hand_fenced = checked_step(
        &mut s,
        Event::EnableStart {
            group: 1,
            now_us: 106_000,
        },
    );
    assert_eq!(
        en_hand_fenced.refusal_for(1).map(|r| r.code),
        Some(RefusalCode::ModeUnverified)
    );
    // Group 0 (on Device 0) is completely unaffected and can re-enable!
    let en_arm = checked_step(
        &mut s,
        Event::EnableStart {
            group: 0,
            now_us: 107_000,
        },
    );
    assert!(en_arm.enable_for(0).is_some());
}

// =========================================================================
// 3. Exhaustive Short-Race Permutations
// =========================================================================

fn for_each_permutation<T: Clone>(items: &mut [T], mut visit: impl FnMut(&[T])) {
    fn heap<T>(k: usize, a: &mut [T], visit: &mut impl FnMut(&[T])) {
        if k == 1 {
            visit(a);
            return;
        }
        heap(k - 1, a, visit);
        for i in 0..(k - 1) {
            if k.is_multiple_of(2) {
                a.swap(i, k - 1);
            } else {
                a.swap(0, k - 1);
            }
            heap(k - 1, a, visit);
        }
    }
    if !items.is_empty() {
        heap(items.len(), items, &mut visit);
    }
}

#[test]
fn exhaustive_race_3rd_sample_vs_stop() {
    let mut base = TorqueState::new(single_arm_config(1)).unwrap();
    enable_group_clean(&mut base, 0, 1_000);
    checked_step(
        &mut base,
        Event::DisableRequest {
            group: 0,
            now_us: 10_000,
            explicit_release_authorized: false,
            recipient_mask_override: None,
        },
    );
    checked_step(&mut base, Event::Release { now_us: 40_000 });
    checked_step(&mut base, Event::CausalSample(sample_at(0, 1, 40_000, 0.0)));
    checked_step(&mut base, Event::Release { now_us: 50_000 });
    checked_step(&mut base, Event::CausalSample(sample_at(0, 2, 50_000, 0.0)));
    checked_step(&mut base, Event::Release { now_us: 60_000 });

    let gen0 = base.groups[0].stop_generation;
    let mut race = vec![
        Event::CausalSample(sample_at(0, 3, 60_000, 0.0)),
        Event::Stop {
            groups: GroupMask::bit(0),
            kind: StopKind::Stop,
            now_us: 60_000,
        },
        Event::WriteResult(WriteResult::uniform(
            0,
            0,
            WriteKind::DisableRequested,
            gen0,
            60_500,
            60_500,
            ChannelMask::bit(0),
            MotorWriteOutcome::Sent,
        )),
        Event::WriteResult(WriteResult::uniform(
            0,
            0,
            WriteKind::DisableRequested,
            gen0,
            60_500,
            60_500,
            ChannelMask::bit(0),
            MotorWriteOutcome::RejectedStale,
        )),
        Event::Release { now_us: 80_000 },
    ];

    let mut count = 0usize;
    for_each_permutation(&mut race, |perm| {
        count += 1;
        let mut s = base;
        let mut saw_authorized_disable = false;
        let mut saw_sent_after_auth = false;
        for &ev in perm {
            let act = checked_step(&mut s, ev);
            if act.disable_for(0).is_some() {
                saw_authorized_disable = true;
            }
            if saw_authorized_disable {
                if let Event::WriteResult(w) = ev {
                    if w.outcomes[0] == MotorWriteOutcome::Sent {
                        saw_sent_after_auth = true;
                    }
                }
            }
        }
        if !saw_sent_after_auth {
            assert_eq!(
                s.groups[0].torque_intent,
                TorqueIntent::On,
                "zero sent writes after auth must keep TorqueIntent::On"
            );
        }
    });
    assert_eq!(count, 120);
}

#[test]
fn exhaustive_race_partial_dispatch_vs_release_enable_motion() {
    let mut base = TorqueState::new(single_arm_config(2)).unwrap();
    enable_group_clean(&mut base, 0, 1_000);
    checked_step(
        &mut base,
        Event::DisableRequest {
            group: 0,
            now_us: 10_000,
            explicit_release_authorized: false,
            recipient_mask_override: None,
        },
    );
    for (k, t) in [(1u64, 40_000u64), (2, 50_000), (3, 60_000)] {
        checked_step(&mut base, Event::Release { now_us: t });
        checked_step(&mut base, Event::CausalSample(sample_at(0, k * 2, t, 0.0)));
        checked_step(&mut base, Event::CausalSample(sample_at(1, k * 2 + 1, t, 0.0)));
    }
    let gen0 = base.groups[0].stop_generation;
    let mut partial = WriteResult::uniform(
        0,
        0,
        WriteKind::DisableRequested,
        gen0,
        61_000,
        61_000,
        ChannelMask::from_count(2),
        MotorWriteOutcome::Deferred,
    );
    partial.outcomes[0] = MotorWriteOutcome::Ambiguous;

    let mut race = vec![
        Event::WriteResult(partial),
        Event::WriteResult(WriteResult::uniform(
            0,
            0,
            WriteKind::DisableRequested,
            gen0,
            82_000,
            82_000,
            ChannelMask::from_count(2),
            MotorWriteOutcome::Sent,
        )),
        Event::EnableStart {
            group: 0,
            now_us: 70_000,
        },
        Event::MotionCommand {
            group: 0,
            channels: ChannelMask::from_count(2),
            now_us: 75_000,
        },
        Event::Release { now_us: 90_000 },
    ];

    let mut count = 0usize;
    for_each_permutation(&mut race, |perm| {
        count += 1;
        let mut s = base;
        for &ev in perm {
            checked_step(&mut s, ev);
        }
    });
    assert_eq!(count, 120);
}

#[test]
fn exhaustive_race_thermal_preemption_and_damping() {
    for damp_us in [0u64, 40_000] {
        let mut cfg = single_arm_config(1);
        cfg.channels[0].desired_mode = ControlMode::Impedance;
        cfg.channels[0].supports_mit_damping = true;
        if damp_us > 0 {
            cfg.channels[0].damping = DampingConfig {
                duration_us: damp_us,
                kd: 1.0,
                max_torque_nm: 3.0,
            };
        }
        let mut base = TorqueState::new(cfg).unwrap();
        verify_all_modes(&mut base, 1_000);
        checked_step(&mut base, Event::CausalSample(sample_at(0, 1, 2_000, 0.0)));

        let mut race = vec![
            Event::EnableStart {
                group: 0,
                now_us: 10_000,
            },
            Event::DisableRequest {
                group: 0,
                now_us: 12_000,
                explicit_release_authorized: false,
                recipient_mask_override: None,
            },
            Event::Thermal {
                channel: 0,
                temperature_c: 75.0,
                now_us: 15_000,
            },
            Event::Release { now_us: 35_000 },
            Event::Release { now_us: 65_000 },
        ];

        let mut count = 0usize;
        for_each_permutation(&mut race, |perm| {
            count += 1;
            let mut s = base;
            for &ev in perm {
                checked_step(&mut s, ev);
            }
            assert!(s.groups[0].protective_latch);
        });
        assert_eq!(count, 120);
    }
}

#[test]
fn exhaustive_race_deadline_expiry_boundaries() {
    let mut base = TorqueState::new(single_arm_config(1)).unwrap();
    enable_group_clean(&mut base, 0, 1_000);
    checked_step(
        &mut base,
        Event::DisableRequest {
            group: 0,
            now_us: 10_000,
            explicit_release_authorized: false,
            recipient_mask_override: None,
        },
    );
    let dl = 10_000 + DEFAULT_DISABLE_DEADLINE_US;
    checked_step(&mut base, Event::Release { now_us: dl - 40_000 });
    checked_step(
        &mut base,
        Event::CausalSample(sample_at(0, 1, dl - 40_000, 0.0)),
    );
    checked_step(&mut base, Event::Release { now_us: dl - 20_000 });
    checked_step(
        &mut base,
        Event::CausalSample(sample_at(0, 2, dl - 20_000, 0.0)),
    );

    let mut total = 0usize;
    for t in [dl - 1, dl, dl + 1] {
        let mut race = vec![
            Event::Release { now_us: t },
            Event::CausalSample(sample_at(0, 3, t, 0.0)),
            Event::WriteResult(WriteResult::uniform(
                0,
                0,
                WriteKind::DisableRequested,
                base.groups[0].stop_generation,
                t,
                t,
                ChannelMask::bit(0),
                MotorWriteOutcome::Deferred,
            )),
            Event::Stop {
                groups: GroupMask::bit(0),
                kind: StopKind::Stop,
                now_us: t,
            },
        ];
        for_each_permutation(&mut race, |perm| {
            total += 1;
            let mut s = base;
            for &ev in perm {
                checked_step(&mut s, ev);
            }
        });
    }
    assert_eq!(total, 72);
}

#[test]
fn exhaustive_race_enable_interruption() {
    let mut base = TorqueState::new(single_arm_config(2)).unwrap();
    verify_all_modes(&mut base, 1_000);
    checked_step(&mut base, Event::CausalSample(sample_at(0, 1, 2_000, 0.1)));
    checked_step(&mut base, Event::CausalSample(sample_at(1, 2, 2_000, 0.2)));
    checked_step(
        &mut base,
        Event::EnableStart {
            group: 0,
            now_us: 10_000,
        },
    );

    let mut race = vec![
        Event::HandshakeResult {
            group: 0,
            now_us: 20_000,
            confirmed_mask: ChannelMask::from_count(2),
            outcome: HandshakeOutcome::Confirmed,
        },
        Event::Stop {
            groups: GroupMask::bit(0),
            kind: StopKind::EStop,
            now_us: 20_000,
        },
        Event::DisableRequest {
            group: 0,
            now_us: 20_000,
            explicit_release_authorized: false,
            recipient_mask_override: None,
        },
        Event::MitOwnershipChange {
            group: 0,
            channels: ChannelMask::bit(0),
            target_mode: ControlMode::Impedance,
            device_wide: false,
            now_us: 20_000,
        },
        Event::Release {
            now_us: 10_000 + DEFAULT_ENABLE_DEADLINE_US,
        },
    ];

    let mut count = 0usize;
    for_each_permutation(&mut race, |perm| {
        count += 1;
        let mut s = base;
        for &ev in perm {
            checked_step(&mut s, ev);
        }
        // Zero disable writes ever sent in this race -> TorqueIntent MUST stay On!
        assert_eq!(s.groups[0].torque_intent, TorqueIntent::On);
    });
    assert_eq!(count, 120);
}

// =========================================================================
// 4. Deterministic Seeded Random Event-Sequence Model Test (10,000 × <= 128)
// =========================================================================

#[derive(Clone, Copy)]
struct SplitMix64(u64);

impl SplitMix64 {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    fn gen_range(&mut self, upper: u64) -> u64 {
        if upper <= 1 {
            0
        } else {
            self.next_u64() % upper
        }
    }
}

fn model_topology(idx: usize) -> TorqueConfig {
    match idx % 4 {
        0 => TorqueConfig::rebot_dm_default(),
        1 => TorqueConfig::arm_plus_underactuated_hand(),
        2 => {
            let mut cfg = single_arm_config(3);
            cfg.devices[0].poison_silence_us = FEETECH_POISON_SILENCE_US;
            cfg.channels[0].damping = DampingConfig {
                duration_us: 40_000,
                kd: 1.2,
                max_torque_nm: 3.5,
            };
            cfg.channels[0].thermal = ThermalConfig {
                derate_start_c: Some(50.0),
                trip_c: 70.0,
            };
            cfg.channels[0].damiao_timeout_reg = Some(900);
            cfg.channels[1].vendor = VendorKind::Feetech;
            cfg.recipients[1].vendor = VendorKind::Feetech;
            cfg.channels[2].role = JointRole::ActiveTool(ActiveToolSafeOutput::BoundedRetention {
                max_retention_us: 120_000,
            });
            cfg.channels[2].active_tool_qualification =
                ActiveToolConfig::Qualified(ActiveToolSafeOutput::BoundedRetention {
                    max_retention_us: 120_000,
                });
            cfg.recipients[2].role = cfg.channels[2].role;
            cfg
        }
        _ => {
            let mut cfg = single_arm_config(2);
            cfg.channels[1].vendor = VendorKind::FashionStar;
            cfg.recipients[1].vendor = VendorKind::FashionStar;
            cfg.recipients[1].rest_evidence_qualified = false;
            cfg
        }
    }
}

fn gen_random_event(
    rng: &mut SplitMix64,
    state: &TorqueState,
    sample_seq: &mut u64,
) -> Event {
    let dt = match rng.gen_range(8) {
        0 => 0,
        1 => 1,
        2 => 10_000,
        3 => 19_999,
        4 => 20_000,
        5 => 20_001,
        6 => 100_001,
        _ => 300_000,
    };
    let now_us = state.now_us.saturating_add(dt);
    let g = rng.gen_range(state.config.group_count as u64) as u8;
    let c = rng.gen_range(state.config.channel_count as u64) as u8;
    let d = rng.gen_range(state.config.device_count as u64) as u8;

    match rng.gen_range(24) {
        0..=4 => Event::Release { now_us },
        5..=9 => {
            if rng.gen_range(6) != 0 {
                *sample_seq += 1;
            }
            let rx_ts = match rng.gen_range(10) {
                0 => now_us.saturating_add(1), // future timestamp
                1 => now_us.saturating_sub(100_001), // stale
                2 => now_us.saturating_sub(100_000), // exact 100 ms boundary
                _ => now_us,
            };
            let pos = match rng.gen_range(6) {
                0 => f32::NAN,
                1 => 0.45, // non-rest
                2 => 0.08, // exact boundary
                _ => 0.0,  // rest
            };
            let drift = match rng.gen_range(8) {
                0 => CLOCK_OFFSET_MAX_DRIFT_US + 1,
                1 => CLOCK_OFFSET_MAX_DRIFT_US,
                _ => 0,
            };
            let ev = match rng.gen_range(5) {
                0 => Some(ReleaseEvidence::SupportedObject),
                1 => Some(ReleaseEvidence::VerifiedEmptyGrasp),
                2 => Some(ReleaseEvidence::HoldingUnsupportedPayload),
                3 => Some(ReleaseEvidence::Unknown),
                _ => None,
            };
            let rep_en = match rng.gen_range(3) {
                0 => Some(false),
                1 => Some(true),
                _ => None,
            };
            let temp = match rng.gen_range(12) {
                0 => Some(72.0),
                1 => Some(60.0),
                2 => Some(f32::NAN),
                _ => None,
            };
            Event::CausalSample(CausalSample {
                channel: c,
                sample_id: *sample_seq,
                rx_timestamp_us: rx_ts,
                now_us,
                clock_offset_drift_us: drift,
                position_rad: pos,
                velocity_rad_s: Some(if rng.gen_range(5) == 0 { 0.2 } else { 0.0 }),
                temperature_c: temp,
                reported_enabled: rep_en,
                correlated_after_write_done_us: None,
                release_evidence: ev,
                is_handshake_ack_only: rng.gen_range(10) == 0,
            })
        }
        10..=11 => {
            let kind = match rng.gen_range(6) {
                0 => WriteKind::Hold,
                1 => WriteKind::Motion,
                2 => WriteKind::Enable,
                3 => WriteKind::DisableRequested,
                4 => WriteKind::DisableProtective,
                _ => WriteKind::Refresh,
            };
            let target_mask = if matches!(kind, WriteKind::DisableRequested | WriteKind::DisableProtective) {
                state.config.groups[g as usize].recipients
            } else {
                state.config.groups[g as usize].channels
            };
            let mut wr = WriteResult {
                group: g,
                device: d,
                kind,
                generation: state.groups[g as usize].stop_generation,
                write_done_us: now_us,
                now_us,
                target_mask,
                outcomes: [MotorWriteOutcome::NotTargeted; MAX_CHANNELS],
            };
            for idx in target_mask.iter() {
                wr.outcomes[idx as usize] = match rng.gen_range(7) {
                    0 => MotorWriteOutcome::Sent,
                    1 => MotorWriteOutcome::Ambiguous,
                    2 => MotorWriteOutcome::Deferred,
                    3 => MotorWriteOutcome::RejectedStale,
                    4 => MotorWriteOutcome::LockContended,
                    5 => MotorWriteOutcome::Failed,
                    _ => MotorWriteOutcome::Sent,
                };
            }
            Event::WriteResult(wr)
        }
        12 => Event::EnableStart { group: g, now_us },
        13 => Event::HandshakeResult {
            group: g,
            now_us,
            confirmed_mask: state.config.groups[g as usize].channels,
            outcome: match rng.gen_range(4) {
                0 => HandshakeOutcome::Pending,
                1 => HandshakeOutcome::Confirmed,
                2 => HandshakeOutcome::Failed,
                _ => HandshakeOutcome::FaultReported { channel: c },
            },
        },
        14 => Event::DisableRequest {
            group: g,
            now_us,
            explicit_release_authorized: rng.gen_range(2) == 0,
            recipient_mask_override: if rng.gen_range(5) == 0 {
                Some(ChannelMask::bit(c))
            } else {
                None
            },
        },
        15 => Event::Stop {
            groups: GroupMask::bit(g),
            kind: match rng.gen_range(5) {
                0 => StopKind::Stop,
                1 => StopKind::EStop,
                2 => StopKind::Watchdog,
                3 => StopKind::LeaseLoss,
                _ => StopKind::Fault(TorqueFaultKind::HardwareFault),
            },
            now_us,
        },
        16 => Event::FaultClear {
            groups: GroupMask::bit(g),
            now_us,
        },
        17 => Event::Thermal {
            channel: c,
            temperature_c: match rng.gen_range(5) {
                0 => 40.0,
                1 => 55.0,
                2 => 65.0,
                3 => 72.0,
                _ => f32::NAN,
            },
            now_us,
        },
        18 => Event::MitOwnershipChange {
            group: g,
            channels: ChannelMask::bit(c),
            target_mode: if rng.gen_range(2) == 0 {
                ControlMode::Impedance
            } else {
                ControlMode::PositionVelocity
            },
            device_wide: rng.gen_range(3) == 0,
            now_us,
        },
        19 => Event::ModeVerified {
            channel: c,
            mode: state.joints[c as usize].desired_mode,
            now_us,
        },
        20 => Event::TimeoutReadback {
            channel: c,
            timeout_reg: if rng.gen_range(2) == 0 { 900 } else { 850 },
            now_us,
        },
        21 => Event::MotionCommand {
            group: g,
            channels: state.config.groups[g as usize].channels,
            now_us,
        },
        22 => match rng.gen_range(4) {
            0 => Event::SerialTimeout { device: d, now_us },
            1 => Event::ParkSettled {
                group: g,
                succeeded: rng.gen_range(2) == 0,
                now_us,
            },
            2 => Event::BusLoss { device: d, now_us },
            _ => Event::GraspStateUpdate {
                channel: c,
                setpoint_rad: -0.5,
                force_limit_raw: Some((rng.gen_range(400) + 100) as u16),
                now_us,
            },
        },
        _ => match rng.gen_range(5) {
            0 => Event::Panic {
                tier: PanicTier::WorkerPanic { thread_tag: 1 },
                now_us,
            },
            1 => Event::Panic {
                tier: PanicTier::CaughtControlPanic,
                now_us,
            },
            2 => Event::Panic {
                tier: PanicTier::ProcessDeath,
                now_us,
            },
            _ => Event::ProcessExit { now_us },
        },
    }
}

#[test]
fn model_test_10000_seeded_sequences_up_to_128_events() {
    const SEQUENCES: usize = 10_000;
    const MAX_EVENTS: usize = 128;
    let mut rng = SplitMix64(0x5EED_70A0_2026_1125);
    let mut total_events: usize = 0;

    for seq in 0..SEQUENCES {
        let cfg = model_topology(seq);
        let mut state = TorqueState::new(cfg).unwrap();
        let len = 1 + (rng.gen_range(MAX_EVENTS as u64) as usize);
        let mut trace = Vec::with_capacity(len);
        let mut sample_seq = 0u64;

        for step_idx in 0..len {
            let ev = gen_random_event(&mut rng, &state, &mut sample_seq);
            trace.push(ev);
            let prev = state;
            let actions = step(&mut state, ev);
            assert_priority_order(&state, &actions);
            if let Err(v) = check_invariants(&state, &prev, &ev, &actions) {
                let (min_cfg, min_trace) = shrink_trace(cfg, &trace, |c, evs| {
                    replay_for_violation(c, evs).is_some()
                });
                let fixture = RegressionFixture {
                    name: format!("shrunk_seq_{seq}"),
                    description: format!("auto-shrunk failure: {v:?}"),
                    topology: FixtureTopology::Custom(Box::new(min_cfg)),
                    events: min_trace,
                    expected_group_intents: Vec::new(),
                    expected_group_states: Vec::new(),
                };
                let json = serde_json::to_string_pretty(&fixture).unwrap_or_default();
                panic!(
                    "model test violation at seq={seq} step={step_idx}: {v:?}\nminimized fixture:\n{json}"
                );
            }
            total_events += 1;
        }
    }
    assert!(total_events >= SEQUENCES * 32);
}

// =========================================================================
// 5. Trace Shrinker & Permanent Regression Fixtures
// =========================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum FixtureTopology {
    SingleArm {
        joints: u8,
        #[serde(default)]
        poison_silence_us: u64,
        #[serde(default)]
        damping_duration_us: u64,
        #[serde(default)]
        impedance_mode: bool,
    },
    RebotDm,
    ArmPlusUnderactuatedHand,
    Custom(Box<TorqueConfig>),
}

impl FixtureTopology {
    fn build(&self) -> TorqueConfig {
        match self {
            Self::SingleArm {
                joints,
                poison_silence_us,
                damping_duration_us,
                impedance_mode,
            } => {
                let mut cfg = single_arm_config(*joints);
                cfg.devices[0].poison_silence_us = *poison_silence_us;
                if *poison_silence_us > 0 {
                    for i in 0..*joints as usize {
                        cfg.channels[i].vendor = VendorKind::Feetech;
                        cfg.recipients[i].vendor = VendorKind::Feetech;
                    }
                }
                if *impedance_mode {
                    for i in 0..*joints as usize {
                        cfg.channels[i].desired_mode = ControlMode::Impedance;
                    }
                }
                if *damping_duration_us > 0 {
                    for i in 0..*joints as usize {
                        cfg.channels[i].damping = DampingConfig {
                            duration_us: *damping_duration_us,
                            kd: 1.5,
                            max_torque_nm: 4.0,
                        };
                    }
                }
                cfg
            }
            Self::RebotDm => TorqueConfig::rebot_dm_default(),
            Self::ArmPlusUnderactuatedHand => TorqueConfig::arm_plus_underactuated_hand(),
            Self::Custom(cfg) => **cfg,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct RegressionFixture {
    name: String,
    description: String,
    topology: FixtureTopology,
    events: Vec<Event>,
    expected_group_intents: Vec<TorqueIntent>,
    expected_group_states: Vec<DriveState>,
}

fn replay_for_violation(cfg: TorqueConfig, events: &[Event]) -> Option<(usize, Violation)> {
    let mut state = TorqueState::new(cfg).ok()?;
    for (idx, &ev) in events.iter().enumerate() {
        let prev = state;
        let actions = step(&mut state, ev);
        if let Err(v) = check_invariants(&state, &prev, &ev, &actions) {
            return Some((idx, v));
        }
    }
    None
}

fn set_event_now_us(ev: &mut Event, new_now: u64) {
    match ev {
        Event::Release { now_us }
        | Event::HandshakeResult { now_us, .. }
        | Event::EnableStart { now_us, .. }
        | Event::DisableRequest { now_us, .. }
        | Event::Stop { now_us, .. }
        | Event::FaultClear { now_us, .. }
        | Event::Thermal { now_us, .. }
        | Event::MitOwnershipChange { now_us, .. }
        | Event::ModeVerified { now_us, .. }
        | Event::TimeoutReadback { now_us, .. }
        | Event::MotionCommand { now_us, .. }
        | Event::SerialTimeout { now_us, .. }
        | Event::GraspStateUpdate { now_us, .. }
        | Event::ParkSettled { now_us, .. }
        | Event::Panic { now_us, .. }
        | Event::ProcessExit { now_us }
        | Event::BusLoss { now_us, .. } => *now_us = new_now,
        Event::CausalSample(s) => {
            let delta = s.now_us.saturating_sub(s.rx_timestamp_us);
            let future_delta = s.rx_timestamp_us.saturating_sub(s.now_us);
            s.now_us = new_now;
            s.rx_timestamp_us = if future_delta > 0 {
                new_now.saturating_add(future_delta)
            } else {
                new_now.saturating_sub(delta)
            };
        }
        Event::WriteResult(w) => {
            let delta = w.now_us.saturating_sub(w.write_done_us);
            w.now_us = new_now;
            w.write_done_us = new_now.saturating_sub(delta);
        }
    }
}

/// Try removing trailing channel `last_ch` from a single-group 1:1 config if still valid.
fn try_drop_last_channel(cfg: &TorqueConfig) -> Option<TorqueConfig> {
    if cfg.channel_count <= 1 || cfg.group_count != 1 || cfg.channel_count != cfg.recipient_count {
        return None;
    }
    let new_n = cfg.channel_count - 1;
    let mut c = *cfg;
    c.channel_count = new_n;
    c.recipient_count = new_n;
    let mask = ChannelMask::from_count(new_n);
    c.groups[0].channels = mask;
    c.groups[0].recipients = mask;
    for r in 0..new_n {
        c.recipients[r as usize].grouped_packet_mask =
            c.recipients[r as usize].grouped_packet_mask.intersect(mask);
    }
    c.validate().ok()?;
    Some(c)
}

/// General trace shrinker per spec:
/// 1. Delete event spans (binary chunk sizes down to 1)
/// 2. Reduce motor/channel count
/// 3. Move timestamps toward boundaries (0, previous timestamp, +1 us, +20 ms, +100 ms)
fn shrink_trace(
    mut cfg: TorqueConfig,
    events: &[Event],
    mut fails: impl FnMut(TorqueConfig, &[Event]) -> bool,
) -> (TorqueConfig, Vec<Event>) {
    let mut cur = events.to_vec();
    if !fails(cfg, &cur) {
        return (cfg, cur);
    }

    // Pass 1: delete event spans (delta debugging)
    let mut chunk = (cur.len() / 2).max(1);
    while chunk >= 1 {
        let mut i = 0;
        let mut progressed = false;
        while i < cur.len() && cur.len() > 1 {
            let end = (i + chunk).min(cur.len());
            let mut candidate = Vec::with_capacity(cur.len() - (end - i));
            candidate.extend_from_slice(&cur[..i]);
            candidate.extend_from_slice(&cur[end..]);
            if !candidate.is_empty() && fails(cfg, &candidate) {
                cur = candidate;
                progressed = true;
            } else {
                i += chunk;
            }
        }
        if !progressed {
            if chunk == 1 {
                break;
            }
            chunk /= 2;
        }
    }

    // Pass 2: reduce motor/channel count
    while let Some(smaller_cfg) = try_drop_last_channel(&cfg) {
        let mask = ChannelMask::from_count(smaller_cfg.channel_count);
        let projected: Vec<Event> = cur
            .iter()
            .copied()
            .filter_map(|mut ev| match &mut ev {
                Event::CausalSample(s) if s.channel >= smaller_cfg.channel_count => None,
                Event::Thermal { channel, .. }
                | Event::ModeVerified { channel, .. }
                | Event::TimeoutReadback { channel, .. }
                | Event::GraspStateUpdate { channel, .. }
                    if *channel >= smaller_cfg.channel_count =>
                {
                    None
                }
                Event::WriteResult(w) => {
                    w.target_mask = w.target_mask.intersect(mask);
                    Some(ev)
                }
                Event::HandshakeResult { confirmed_mask, .. } => {
                    *confirmed_mask = confirmed_mask.intersect(mask);
                    Some(ev)
                }
                Event::MotionCommand { channels, .. }
                | Event::MitOwnershipChange { channels, .. } => {
                    *channels = channels.intersect(mask);
                    Some(ev)
                }
                _ => Some(ev),
            })
            .collect();
        if !projected.is_empty() && fails(smaller_cfg, &projected) {
            cfg = smaller_cfg;
            cur = projected;
        } else {
            break;
        }
    }

    // Pass 3: shrink timestamps toward boundaries
    for idx in 0..cur.len() {
        let prev_ts = if idx == 0 {
            0
        } else {
            event_now_us(&cur[idx - 1])
        };
        let orig_ts = event_now_us(&cur[idx]);
        let candidates = [
            0u64,
            prev_ts,
            prev_ts.saturating_add(1),
            prev_ts.saturating_add(REST_SETTLE_US),
            prev_ts.saturating_add(REST_SETTLE_US + 1),
            prev_ts.saturating_add(DEFAULT_CAPTURE_MAX_AGE_US),
            prev_ts.saturating_add(DEFAULT_CAPTURE_MAX_AGE_US + 1),
        ];
        for &cand_ts in &candidates {
            if cand_ts >= orig_ts {
                continue;
            }
            let mut trial = cur.clone();
            set_event_now_us(&mut trial[idx], cand_ts);
            if fails(cfg, &trial) {
                cur = trial;
                break;
            }
        }
    }

    (cfg, cur)
}

#[test]
fn shrinker_minimizes_spans_channels_and_timestamps() {
    // Construct a noisy 48-event 7-motor trace where a synthetic property detects
    // "Group 0 entered FailedHold after an interrupted EnableStart".
    let cfg = single_arm_config(7);
    let mut noisy = Vec::new();
    let mut t = 500_000u64;
    for i in 0..20u64 {
        t += 50_000;
        noisy.push(Event::Release { now_us: t });
        noisy.push(Event::CausalSample(sample_at((i % 7) as u8, i + 1, t, 0.0)));
    }
    t += 200_000;
    noisy.push(Event::EnableStart {
        group: 0,
        now_us: t,
    });
    t += 50_000;
    noisy.push(Event::Release { now_us: t });
    t += 50_000;
    noisy.push(Event::Stop {
        groups: GroupMask::bit(0),
        kind: StopKind::EStop,
        now_us: t,
    });
    for i in 0..5u64 {
        t += 30_000;
        noisy.push(Event::Release { now_us: t });
        noisy.push(Event::CausalSample(sample_at((i % 7) as u8, 100 + i, t, 0.0)));
    }

    let predicate = |c: TorqueConfig, evs: &[Event]| -> bool {
        let Ok(mut s) = TorqueState::new(c) else {
            return false;
        };
        for &ev in evs {
            step(&mut s, ev);
            if s.groups[0].drive_state == DriveState::FailedHold {
                return true;
            }
        }
        false
    };
    assert!(predicate(cfg, &noisy));

    let (min_cfg, min_trace) = shrink_trace(cfg, &noisy, predicate);
    assert_eq!(min_cfg.channel_count, 1, "motor count shrunk from 7 to 1");
    assert_eq!(
        min_trace.len(),
        2,
        "48 noisy events shrunk to minimal 2-event trace: {min_trace:?}"
    );
    assert_eq!(
        event_now_us(&min_trace[0]),
        0,
        "initial timestamp shrunk to boundary 0"
    );
    assert_eq!(
        event_now_us(&min_trace[1]),
        0,
        "second timestamp shrunk to boundary 0"
    );
}

#[test]
fn replay_regression_fixtures_from_disk() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/torque_reducer");
    let mut files: Vec<_> = fs::read_dir(&dir)
        .expect("tests/fixtures/torque_reducer directory must exist")
        .map(|e| e.unwrap().path())
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("json"))
        .collect();
    files.sort();
    assert!(
        files.len() >= 5,
        "expected >= 5 regression fixtures in {}, found {}",
        dir.display(),
        files.len()
    );

    for path in files {
        let raw = fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("failed to read {}: {e}", path.display()));
        let fx: RegressionFixture = serde_json::from_str(&raw)
            .unwrap_or_else(|e| panic!("failed to parse {}: {e}", path.display()));
        let mut s = TorqueState::new(fx.topology.build())
            .unwrap_or_else(|e| panic!("invalid config in {}: {e:?}", fx.name));
        for (i, &ev) in fx.events.iter().enumerate() {
            let prev = s;
            let actions = step(&mut s, ev);
            assert_priority_order(&s, &actions);
            if let Err(v) = check_invariants(&s, &prev, &ev, &actions) {
                panic!("fixture {} violated invariant at step {i}: {v:?}", fx.name);
            }
        }
        for (g, &exp_intent) in fx.expected_group_intents.iter().enumerate() {
            assert_eq!(
                s.groups[g].torque_intent, exp_intent,
                "fixture {} group {g} intent mismatch",
                fx.name
            );
        }
        for (g, &exp_state) in fx.expected_group_states.iter().enumerate() {
            assert_eq!(
                s.groups[g].drive_state, exp_state,
                "fixture {} group {g} drive_state mismatch",
                fx.name
            );
        }
    }
}

// =========================================================================
// 6. Zero-Allocation Test on step() and check_invariants() with TimedAlloc
// =========================================================================

#[test]
fn step_and_check_invariants_zero_alloc_under_timed_alloc() {
    time_this_thread();

    // Positive control proving TimedAlloc is installed and counting on this thread.
    let before_ctrl = snapshot_counts();
    let boxed = std::hint::black_box(Box::new(0xDEAD_BEEFu64));
    drop(boxed);
    let ctrl_delta = snapshot_counts().delta_since(before_ctrl);
    assert!(
        ctrl_delta.alloc >= 1 && ctrl_delta.dealloc >= 1,
        "positive control failed: TimedAlloc did not observe Box alloc/dealloc: {ctrl_delta:?}"
    );

    // Pre-build realistic dual-group state + full array of all Event variants outside counted region.
    let cfg = TorqueConfig::arm_plus_underactuated_hand();
    let mut state = TorqueState::new(cfg).unwrap();

    let events: [Event; 22] = [
        Event::ModeVerified {
            channel: 0,
            mode: ControlMode::PositionVelocity,
            now_us: 1_000,
        },
        Event::TimeoutReadback {
            channel: 0,
            timeout_reg: 900,
            now_us: 1_500,
        },
        Event::EnableStart {
            group: 0,
            now_us: 2_000,
        },
        Event::HandshakeResult {
            group: 0,
            now_us: 5_000,
            confirmed_mask: ChannelMask::from_count(6),
            outcome: HandshakeOutcome::Pending,
        },
        Event::CausalSample(sample_at(0, 1, 10_000, 0.0)),
        Event::Release { now_us: 20_000 },
        Event::MotionCommand {
            group: 0,
            channels: ChannelMask::from_count(6),
            now_us: 21_000,
        },
        Event::WriteResult(WriteResult::uniform(
            0,
            0,
            WriteKind::Motion,
            0,
            22_000,
            22_000,
            ChannelMask::from_count(6),
            MotorWriteOutcome::Sent,
        )),
        Event::GraspStateUpdate {
            channel: 6,
            setpoint_rad: -0.5,
            force_limit_raw: Some(200),
            now_us: 23_000,
        },
        Event::Stop {
            groups: GroupMask::bit(0),
            kind: StopKind::Stop,
            now_us: 25_000,
        },
        Event::Release { now_us: 40_000 },
        Event::DisableRequest {
            group: 0,
            now_us: 45_000,
            explicit_release_authorized: false,
            recipient_mask_override: None,
        },
        Event::WriteResult(WriteResult::uniform(
            0,
            0,
            WriteKind::Hold,
            1,
            46_000,
            46_000,
            ChannelMask::from_count(6),
            MotorWriteOutcome::Sent,
        )),
        Event::SerialTimeout {
            device: 1,
            now_us: 47_000,
        },
        Event::MitOwnershipChange {
            group: 1,
            channels: ChannelMask::bit(6),
            target_mode: ControlMode::PositionForce,
            device_wide: true,
            now_us: 48_000,
        },
        Event::BusLoss {
            device: 1,
            now_us: 50_000,
        },
        Event::FaultClear {
            groups: GroupMask::bit(1),
            now_us: 55_000,
        },
        Event::Thermal {
            channel: 0,
            temperature_c: 62.0,
            now_us: 60_000,
        },
        Event::Thermal {
            channel: 0,
            temperature_c: 75.0,
            now_us: 65_000,
        },
        Event::ParkSettled {
            group: 0,
            succeeded: false,
            now_us: 70_000,
        },
        Event::Panic {
            tier: PanicTier::WorkerPanic { thread_tag: 2 },
            now_us: 75_000,
        },
        Event::ProcessExit { now_us: 80_000 },
    ];

    let before = snapshot_counts();
    let mut visited = 0u64;
    for round in 0..10u64 {
        let mut s = state;
        for &base_ev in &events {
            let mut ev = base_ev;
            set_event_now_us(&mut ev, event_now_us(&base_ev) + round * 100 * US_PER_MS);
            let prev = s;
            let actions = step(&mut s, ev);
            assert!(check_invariants(&s, &prev, &ev, &actions).is_ok());
            actions.visit_by_priority(&s, |_a| {
                visited += 1;
            });
        }
        state = s;
    }
    let delta = snapshot_counts().delta_since(before);
    assert!(visited > 0);
    assert_eq!(
        delta,
        AllocCounts::ZERO,
        "step() / check_invariants() / visit_by_priority() MUST perform zero heap ops, got {delta:?}"
    );
}
