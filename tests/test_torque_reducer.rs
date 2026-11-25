//! Comprehensive verification suite for the pure `armd::torque` reducer core:
//! - Unit tests for every CiA 402 state, transition, role, boundary, and refusal code
//! - Regression tests for every review Blocker (B1–B13), Major (M1–M16), and Minor (m1–m2)
//! - Synthetic underactuated hand group next to the arm group
//! - Exhaustive short-race permutations (3rd sample vs stop, partial dispatch, thermal, expiry, enable interruption)
//! - Seeded random event-sequence model test: 10,000 sequences × up to 128 events with an
//!   independent capability-tracking oracle and invariants checked after every event
//! - Automatic trace shrinker preserving exact violation kind (event-span deletion, channel/motor
//!   reduction, timestamp boundary shrinking)
//! - Event-indexed JSON regression fixture replay (`tests/fixtures/torque_reducer/*.json`)
//! - Zero-allocation proof on `step()`, `is_disable_authorized()`, and `check_invariants()` using `TimedAlloc`

#[global_allocator]
static ALLOC: armd::diagnostics::alloc::TimedAlloc = armd::diagnostics::alloc::TimedAlloc;

use std::fs;
use std::path::Path;

use serde::{Deserialize, Serialize};

use armd::diagnostics::alloc::{snapshot_counts, time_this_thread, AllocCounts};
use armd::motor_mode::ControlMode;
use armd::torque::{
    check_invariants, event_now_us, is_disable_authorized, step, ActionPriority, ActionSet,
    ActiveToolConfig, ActiveToolSafeOutput, CausalSample, ChannelConfig, ChannelMask, ConfigError,
    DampingConfig, DeviceConfig, DeviceMask, DisableCommand, DisableKind, DisableOrigin,
    DriveState, EnableCommand, EnablePhase, Event, GroupConfig, GroupMask, HandshakeOutcome,
    HoldKind, JointRole, MotorWriteOutcome, PanicTier, ParkRequest, RecipientConfig, RefusalCode,
    ReleaseEvidence, StopKind, ThermalConfig, TorqueConfig, TorqueFaultKind, TorqueIntent,
    TorqueState, VendorKind, Violation, WriteKind, WriteResult, CLOCK_OFFSET_MAX_DRIFT_US,
    DAMIAO_DISABLE_REPEATS, DAMIAO_DISABLE_REPEAT_INTERVAL_US, DAMIAO_TIMEOUT_MAX_REG,
    DEFAULT_CAPTURE_MAX_AGE_US, DEFAULT_DISABLE_DEADLINE_US, DEFAULT_ENABLE_DEADLINE_US,
    FEETECH_POISON_SILENCE_US, MAX_CHANNELS, MAX_DEVICES, MAX_GROUPS, REST_SETTLE_US, US_PER_MS,
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

fn sample_at(channel: u8, sample_id: u64, ts_us: u64, pos_rad: f32) -> CausalSample {
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
        poll_issued_us: None,
        release_evidence: None,
        is_handshake_ack_only: false,
    }
}

fn feetech_sample_at(
    channel: u8,
    sample_id: u64,
    poll_us: u64,
    rx_us: u64,
    pos_rad: f32,
) -> CausalSample {
    let mut s = sample_at(channel, sample_id, rx_us, pos_rad);
    s.poll_issued_us = Some(poll_us);
    s.correlated_after_write_done_us = Some(poll_us);
    s
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
    let ecmd = act.enable_for(group).expect("enable should be accepted");
    let chs = state.config.groups[group as usize].channels;
    checked_step(
        state,
        Event::HandshakeResult {
            group,
            attempt_id: ecmd.attempt_id,
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
// 1. Unit Tests Per Transition & Policy (incl. B4, M4, M9, M14, m2)
// =========================================================================

#[test]
fn state_size_and_topology_validation() {
    let state_bytes = std::mem::size_of::<TorqueState>();
    let actions_bytes = std::mem::size_of::<ActionSet>();
    assert!(
        state_bytes <= 20_480,
        "TorqueState unexpectedly large: {state_bytes} bytes"
    );
    assert!(
        actions_bytes <= 10_240,
        "ActionSet unexpectedly large: {actions_bytes} bytes"
    );

    assert!(TorqueState::new(TorqueConfig::rebot_dm_default()).is_ok());
    assert!(TorqueState::new(TorqueConfig::arm_plus_underactuated_hand()).is_ok());

    // Exact max capacity (32 channels, 4 devices, 8 groups) accepted
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

    // B4: Cross-group channel injection into GroupConfig::channels MUST be rejected
    let mut bad = TorqueConfig::arm_plus_underactuated_hand();
    bad.groups[0].channels.insert(6); // hand channel leaked into arm group mask
    assert_eq!(
        bad.validate(),
        Err(ConfigError::MismatchedGroupTopology { group: 0 })
    );

    // M4: max_capture_age_us > 100 ms MUST be rejected; <= 100 ms accepted
    let mut bad = single_arm_config(1);
    bad.channels[0].max_capture_age_us = DEFAULT_CAPTURE_MAX_AGE_US + 1;
    assert_eq!(
        bad.validate(),
        Err(ConfigError::InvalidCaptureAge { channel: 0 })
    );
    bad.channels[0].max_capture_age_us = 50_000;
    assert!(bad.validate().is_ok());

    // M9: Damiao request lifetime > 1 s, repeats > 3, or repeat spacing < 20 ms rejected
    let mut bad = single_arm_config(1);
    bad.groups[0].disable_deadline_us = DEFAULT_DISABLE_DEADLINE_US + 1;
    assert_eq!(
        bad.validate(),
        Err(ConfigError::InvalidGroupTiming { group: 0 })
    );
    let mut bad = single_arm_config(1);
    bad.groups[0].disable_repeats = DAMIAO_DISABLE_REPEATS + 1;
    assert_eq!(
        bad.validate(),
        Err(ConfigError::InvalidGroupTiming { group: 0 })
    );
    let mut bad = single_arm_config(1);
    bad.groups[0].disable_repeat_interval_us = DAMIAO_DISABLE_REPEAT_INTERVAL_US - 1;
    assert_eq!(
        bad.validate(),
        Err(ConfigError::InvalidGroupTiming { group: 0 })
    );

    // M14: Two devices sharing one bus_id with unequal quotas rejected
    let mut bad = TorqueConfig::arm_plus_underactuated_hand();
    bad.devices[1].bus_id = 0; // devices[0].tx_quota=16, devices[1].tx_quota=8
    assert_eq!(
        bad.validate(),
        Err(ConfigError::SharedBusBudgetMismatch { bus_id: 0 })
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

    // m2: GROUP_OUT_OF_RANGE refusal on 1-group profile is delivered by visit_by_priority!
    let mut s = TorqueState::new(single_arm_config(1)).unwrap();
    let act = checked_step(
        &mut s,
        Event::EnableStart {
            group: 1,
            now_us: 1_000,
        },
    );
    assert_eq!(
        act.refusal_for(1).map(|r| r.code),
        Some(RefusalCode::GroupOutOfRange)
    );
    let mut visited_refusal = None;
    act.visit_by_priority(&s, |a| {
        if let armd::torque::Action::Refusal(r) = a {
            visited_refusal = Some(r);
        }
    });
    assert_eq!(
        visited_refusal.map(|r| (r.group, r.code)),
        Some((1, RefusalCode::GroupOutOfRange))
    );
}

#[test]
fn enable_handshake_mode_timeout_session_and_attempt_correlation() {
    let mut cfg = single_arm_config(2);
    cfg.channels[0].damiao_timeout_reg = Some(900);
    cfg.channels[1].damiao_timeout_reg = Some(900);
    let mut s = TorqueState::new(cfg).unwrap();

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
    // Missing TIMEOUT readback -> refused
    let act = checked_step(
        &mut s,
        Event::EnableStart {
            group: 0,
            now_us: 2_000,
        },
    );
    assert_eq!(
        act.refusal_for(0).map(|r| r.code),
        Some(RefusalCode::TimeoutMismatch)
    );

    // Readback 900 on both -> start attempt A
    checked_step(
        &mut s,
        Event::TimeoutReadback {
            channel: 0,
            timeout_reg: 900,
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
    let en_a = checked_step(
        &mut s,
        Event::EnableStart {
            group: 0,
            now_us: 4_000,
        },
    )
    .enable_for(0)
    .unwrap();

    // B2: Ordinary CausalSample with reported_enabled=true MUST NOT arm enable!
    for c in 0..2u8 {
        let mut cs = sample_at(c, c as u64 + 1, 5_000, 0.0);
        cs.reported_enabled = Some(true);
        checked_step(&mut s, Event::CausalSample(cs));
    }
    assert_eq!(
        s.groups[0].drive_state,
        DriveState::ReadyToSwitchOn,
        "B2: ordinary reported_enabled must stay telemetry only"
    );

    // Interrupt attempt A with Stop -> FailedHold -> FaultClear -> start attempt B
    checked_step(
        &mut s,
        Event::Stop {
            groups: GroupMask::bit(0),
            kind: StopKind::Stop,
            now_us: 6_000,
        },
    );
    assert_eq!(s.groups[0].drive_state, DriveState::FailedHold);
    checked_step(
        &mut s,
        Event::FaultClear {
            groups: GroupMask::bit(0),
            now_us: 7_000,
        },
    );
    let en_b = checked_step(
        &mut s,
        Event::EnableStart {
            group: 0,
            now_us: 8_000,
        },
    )
    .enable_for(0)
    .unwrap();
    assert_ne!(en_a.attempt_id, en_b.attempt_id);

    // B2: Delayed positive HandshakeResult AND stale WriteResult(Enable, Failed) from attempt A
    // MUST be ignored during attempt B!
    checked_step(
        &mut s,
        Event::HandshakeResult {
            group: 0,
            attempt_id: en_a.attempt_id,
            now_us: 9_000,
            confirmed_mask: ChannelMask::from_count(2),
            outcome: HandshakeOutcome::Confirmed,
        },
    );
    assert_eq!(s.groups[0].drive_state, DriveState::ReadyToSwitchOn);

    checked_step(
        &mut s,
        Event::WriteResult(WriteResult::uniform(
            0,
            0,
            WriteKind::Enable,
            en_a.attempt_id,
            en_a.generation,
            9_500,
            9_500,
            ChannelMask::from_count(2),
            MotorWriteOutcome::Failed,
        )),
    );
    assert_eq!(s.groups[0].drive_state, DriveState::ReadyToSwitchOn);

    // Correlated HandshakeResult for attempt B arms the group cleanly!
    checked_step(
        &mut s,
        Event::HandshakeResult {
            group: 0,
            attempt_id: en_b.attempt_id,
            now_us: 10_000,
            confirmed_mask: ChannelMask::from_count(2),
            outcome: HandshakeOutcome::Confirmed,
        },
    );
    assert_eq!(s.groups[0].drive_state, DriveState::OperationEnabled);

    // M11: BusLoss invalidates cached TIMEOUT session! Clearing fault and re-enabling without
    // fresh TIMEOUT readback in the new session MUST refuse with TimeoutMismatch!
    checked_step(
        &mut s,
        Event::BusLoss {
            device: 0,
            now_us: 12_000,
        },
    );
    checked_step(
        &mut s,
        Event::FaultClear {
            groups: GroupMask::bit(0),
            now_us: 13_000,
        },
    );
    let en_after_loss = checked_step(
        &mut s,
        Event::EnableStart {
            group: 0,
            now_us: 14_000,
        },
    );
    assert_eq!(
        en_after_loss.refusal_for(0).map(|r| r.code),
        Some(RefusalCode::TimeoutMismatch)
    );
}

#[test]
fn enable_failure_interruption_and_expiry_enter_failed_hold_without_rollback() {
    for idx in 0..6usize {
        let mut s = TorqueState::new(single_arm_config(2)).unwrap();
        verify_all_modes(&mut s, 5_000);
        checked_step(&mut s, Event::CausalSample(sample_at(0, 1, 9_000, 0.42)));
        checked_step(&mut s, Event::CausalSample(sample_at(1, 2, 9_000, -0.31)));
        let ecmd = checked_step(
            &mut s,
            Event::EnableStart {
                group: 0,
                now_us: 10_000,
            },
        )
        .enable_for(0)
        .unwrap();

        let trigger = match idx {
            0 => Event::HandshakeResult {
                group: 0,
                attempt_id: ecmd.attempt_id,
                now_us: 50_000,
                confirmed_mask: ChannelMask::EMPTY,
                outcome: HandshakeOutcome::Failed,
            },
            1 => Event::HandshakeResult {
                group: 0,
                attempt_id: ecmd.attempt_id,
                now_us: 50_000,
                confirmed_mask: ChannelMask::EMPTY,
                outcome: HandshakeOutcome::FaultReported { channel: 1 },
            },
            2 => Event::Stop {
                groups: GroupMask::bit(0),
                kind: StopKind::Stop,
                now_us: 50_000,
            },
            3 => Event::DisableRequest {
                group: 0,
                now_us: 50_000,
                explicit_release_authorized: false,
                recipient_mask_override: None,
            },
            4 => Event::MitOwnershipChange {
                group: 0,
                channels: ChannelMask::bit(0),
                target_mode: ControlMode::Impedance,
                device_wide: false,
                now_us: 50_000,
            },
            _ => Event::Release {
                now_us: 10_000 + DEFAULT_ENABLE_DEADLINE_US,
            },
        };

        let act = checked_step(&mut s, trigger);
        assert_eq!(s.groups[0].drive_state, DriveState::FailedHold, "case {idx}");
        assert_eq!(s.groups[0].enable_phase, EnablePhase::FailedHold);
        assert_eq!(s.groups[0].torque_intent, TorqueIntent::On);
        assert!(s.groups[0].faults.contains(TorqueFaultKind::EnableFailed));
        assert!(!act.any_disable());
        assert_eq!(
            act.refusal_for(0).map(|r| r.code),
            Some(RefusalCode::EnableFailed)
        );

        let t_next = s.now_us + 5_000;
        let mut s0 = sample_at(0, 10, t_next, 0.43);
        s0.reported_enabled = Some(false);
        let mut s1 = sample_at(1, 11, t_next, -0.30);
        s1.reported_enabled = Some(false);
        checked_step(&mut s, Event::CausalSample(s0));
        checked_step(&mut s, Event::CausalSample(s1));
        let rel = checked_step(&mut s, Event::Release { now_us: t_next });
        assert!(rel.hold_for(0).is_some());
        assert!(rel.hold_for(1).is_some());
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

    let mut sag = sample_at(0, 3, 40_000, 1.05);
    sag.reported_enabled = Some(false);
    checked_step(&mut s, Event::CausalSample(sag));
    let rel1 = checked_step(&mut s, Event::Release { now_us: 40_000 });
    assert_eq!(rel1.hold_for(0).unwrap().position_rad, 1.0);
    assert_eq!(rel1.hold_for(1).unwrap().position_rad, -0.5);

    let rel2 = checked_step(
        &mut s,
        Event::Release {
            now_us: 20_000 + DEFAULT_CAPTURE_MAX_AGE_US,
        },
    );
    assert!(rel2.hold_for(1).is_some());

    let rel3 = checked_step(
        &mut s,
        Event::Release {
            now_us: 20_000 + DEFAULT_CAPTURE_MAX_AGE_US + 1,
        },
    );
    assert!(rel3.hold_for(1).is_none());
    assert!(rel3.telemetry.unheld_mask.contains(1));
    assert!(rel3.refresh_requests.contains(1));
    assert!(rel3.hold_for(0).is_some());

    checked_step(&mut s, Event::CausalSample(sample_at(1, 4, 130_000, -0.4)));
    let rel4 = checked_step(&mut s, Event::Release { now_us: 130_000 });
    assert_eq!(rel4.hold_for(1).unwrap().position_rad, -0.4);

    // B13: device_wide=true on a device whose static device_wide_mode_switch==false MUST fence ALL
    // siblings until every channel on the device is re-verified!
    checked_step(
        &mut s,
        Event::MitOwnershipChange {
            group: 0,
            channels: ChannelMask::bit(0),
            target_mode: ControlMode::Impedance,
            device_wide: true,
            now_us: 135_000,
        },
    );
    // Re-verify ONLY channel 0 (sibling ch1 still unverified!) -> group 0 MUST remain fenced!
    checked_step(
        &mut s,
        Event::ModeVerified {
            channel: 0,
            mode: ControlMode::Impedance,
            now_us: 136_000,
        },
    );
    let en_fenced = checked_step(
        &mut s,
        Event::EnableStart {
            group: 0,
            now_us: 137_000,
        },
    );
    assert_eq!(
        en_fenced.refusal_for(0).map(|r| r.code),
        Some(RefusalCode::ModeUnverified)
    );
    // Re-verify sibling ch1 -> fence clears!
    checked_step(
        &mut s,
        Event::ModeVerified {
            channel: 1,
            mode: ControlMode::PositionVelocity,
            now_us: 138_000,
        },
    );
    let en_ok = checked_step(
        &mut s,
        Event::EnableStart {
            group: 0,
            now_us: 139_000,
        },
    );
    assert!(en_ok.enable_for(0).is_some());
}

#[test]
fn fault_clear_never_disables_and_preserves_intent() {
    let mut s = TorqueState::new(single_arm_config(1)).unwrap();
    verify_all_modes(&mut s, 1_000);
    checked_step(&mut s, Event::CausalSample(sample_at(0, 1, 2_000, 0.0)));
    let ecmd = checked_step(
        &mut s,
        Event::EnableStart {
            group: 0,
            now_us: 3_000,
        },
    )
    .enable_for(0)
    .unwrap();
    checked_step(
        &mut s,
        Event::HandshakeResult {
            group: 0,
            attempt_id: ecmd.attempt_id,
            now_us: 4_000,
            confirmed_mask: ChannelMask::EMPTY,
            outcome: HandshakeOutcome::Failed,
        },
    );

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

    let rel = checked_step(&mut s, Event::Release { now_us: 15_000 });
    assert!(rel.hold_for(0).is_some());
}

#[test]
fn rest_gate_k3_boundaries_causality_feetech_b7_b8_b9_and_fashionstar() {
    let mut s = TorqueState::new(single_arm_config(1)).unwrap();
    enable_group_clean(&mut s, 0, 1_000);

    // M3: Operational DisableRequest bumps stop_generation to invalidate outstanding motion!
    let gen_before = s.groups[0].stop_generation;
    checked_step(
        &mut s,
        Event::WriteResult(WriteResult::uniform(
            0,
            0,
            WriteKind::Motion,
            0,
            gen_before,
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
    assert!(
        s.groups[0].stop_generation > gen_before,
        "M3: DisableRequest from OperationEnabled must bump stop_generation"
    );

    // Stationary hold preserves motion boundary
    let cur_gen = s.groups[0].stop_generation;
    checked_step(
        &mut s,
        Event::WriteResult(WriteResult::uniform(
            0,
            0,
            WriteKind::Hold,
            0,
            cur_gen,
            25_000,
            25_000,
            ChannelMask::bit(0),
            MotorWriteOutcome::Sent,
        )),
    );
    assert_eq!(s.joints[0].last_motion_write_us, Some(10_000));

    // Exact t = 10_000 + 20_000 = 30_000 rejected; 30_001 accepted
    checked_step(&mut s, Event::Release { now_us: 30_000 });
    checked_step(
        &mut s,
        Event::CausalSample(sample_at(0, 1, 10_000 + REST_SETTLE_US, 0.0)),
    );
    assert_eq!(s.rest_windows[0].count, 0);

    checked_step(&mut s, Event::Release { now_us: 30_001 });
    checked_step(
        &mut s,
        Event::CausalSample(sample_at(0, 2, 10_000 + REST_SETTLE_US + 1, 0.0)),
    );
    assert_eq!(s.rest_windows[0].count, 1);

    // B8: Stale oldest snapshot at (1, 61, 121 ms) MUST NOT pass because oldest sample (1 ms) is > 100 ms old!
    let mut s_b8 = TorqueState::new(single_arm_config(1)).unwrap();
    enable_group_clean(&mut s_b8, 0, 0);
    checked_step(
        &mut s_b8,
        Event::DisableRequest {
            group: 0,
            now_us: 1_000,
            explicit_release_authorized: false,
            recipient_mask_override: None,
        },
    );
    checked_step(&mut s_b8, Event::Release { now_us: 1_000 });
    checked_step(&mut s_b8, Event::CausalSample(sample_at(0, 1, 1_000, 0.0)));
    checked_step(&mut s_b8, Event::Release { now_us: 61_000 });
    checked_step(&mut s_b8, Event::CausalSample(sample_at(0, 2, 61_000, 0.0)));
    checked_step(&mut s_b8, Event::Release { now_us: 121_000 });
    let act_b8 = checked_step(&mut s_b8, Event::CausalSample(sample_at(0, 3, 121_000, 0.0)));
    assert!(
        act_b8.disable_for(0).is_none(),
        "B8: oldest snapshot at 1 ms is 120 ms old (> 100 ms) and must NOT authorize"
    );

    // B9: Revocation of unconsumed AuthorizedForRelease on newer non-rest sample or motion write
    checked_step(&mut s_b8, Event::Release { now_us: 141_000 });
    let act_ok = checked_step(&mut s_b8, Event::CausalSample(sample_at(0, 4, 141_000, 0.0)));
    let dcmd = act_ok.disable_for(0).expect("sliding window [61,121,141] is all <= 80ms fresh");
    assert!(is_disable_authorized(&s_b8, 0, dcmd.dispatch_id, 141_000));
    // Before TX, a motion write arrives -> revokes live authorization!
    let b8_gen = s_b8.groups[0].stop_generation;
    checked_step(
        &mut s_b8,
        Event::WriteResult(WriteResult::uniform(
            0,
            0,
            WriteKind::Motion,
            0,
            b8_gen,
            141_500,
            141_500,
            ChannelMask::bit(0),
            MotorWriteOutcome::Sent,
        )),
    );
    assert!(
        !is_disable_authorized(&s_b8, 0, dcmd.dispatch_id, 141_500),
        "B9: live write-time authorization check must return false after motion invalidation"
    );

    // B7: Feetech causality — uncorrelated or pre-settle poll rejected; correlated post-settle passes!
    let mut ft_cfg = single_arm_config(1);
    ft_cfg.devices[0].poison_silence_us = FEETECH_POISON_SILENCE_US;
    ft_cfg.channels[0].vendor = VendorKind::Feetech;
    ft_cfg.recipients[0].vendor = VendorKind::Feetech;
    let mut s_ft = TorqueState::new(ft_cfg).unwrap();
    enable_group_clean(&mut s_ft, 0, 1_000);
    // Motion write at t = 10_000 -> settle boundary is strictly > 30_000
    checked_step(
        &mut s_ft,
        Event::WriteResult(WriteResult::uniform(
            0,
            0,
            WriteKind::Motion,
            0,
            0,
            10_000,
            10_000,
            ChannelMask::bit(0),
            MotorWriteOutcome::Sent,
        )),
    );
    checked_step(
        &mut s_ft,
        Event::DisableRequest {
            group: 0,
            now_us: 15_000,
            explicit_release_authorized: false,
            recipient_mask_override: None,
        },
    );
    // Uncorrelated sample (poll_issued_us = None) MUST NOT count!
    checked_step(&mut s_ft, Event::Release { now_us: 40_000 });
    checked_step(&mut s_ft, Event::CausalSample(sample_at(0, 1, 40_000, 0.0)));
    assert_eq!(s_ft.rest_windows[0].count, 0, "B7: uncorrelated Feetech sample rejected");

    // Poll issued BEFORE settle (poll_us = 25_000 <= 30_000), reply arrives after settle (rx = 45_000) -> REJECTED!
    checked_step(&mut s_ft, Event::Release { now_us: 45_000 });
    checked_step(
        &mut s_ft,
        Event::CausalSample(feetech_sample_at(0, 2, 25_000, 45_000, 0.0)),
    );
    assert_eq!(
        s_ft.rest_windows[0].count, 0,
        "B7: Feetech poll issued before settle must be rejected even if reply arrives after settle"
    );

    // SerialTimeout at 50_000 poisons until 70_000; 3 correlated post-silence polls pass!
    checked_step(
        &mut s_ft,
        Event::SerialTimeout {
            device: 0,
            now_us: 50_000,
        },
    );
    for (id, t) in [(3u64, 70_000u64), (4, 80_000), (5, 90_000)] {
        checked_step(&mut s_ft, Event::Release { now_us: t });
        let a = checked_step(
            &mut s_ft,
            Event::CausalSample(feetech_sample_at(0, id, t, t, 0.0)),
        );
        if id == 5 {
            assert!(a.disable_for(0).is_some(), "correlated post-silence Feetech passes");
        }
    }

    // FashionStar unqualified rest evidence refused immediately
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
}

#[test]
fn dispatch_correlation_b1_b5_b10_repeats_and_partial_outcomes() {
    let mut s = TorqueState::new(single_arm_config(2)).unwrap();
    enable_group_clean(&mut s, 0, 1_000);

    // B5: Requesting only 1 singleton recipient of a 2-recipient group MUST be refused!
    let partial_req = checked_step(
        &mut s,
        Event::DisableRequest {
            group: 0,
            now_us: 5_000,
            explicit_release_authorized: false,
            recipient_mask_override: Some(ChannelMask::bit(0)),
        },
    );
    assert_eq!(
        partial_req.refusal_for(0).map(|r| r.code),
        Some(RefusalCode::RecipientMaskInvalid)
    );

    // Full-group disable request
    checked_step(
        &mut s,
        Event::DisableRequest {
            group: 0,
            now_us: 10_000,
            explicit_release_authorized: false,
            recipient_mask_override: None,
        },
    );
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

    // B1: Split callbacks (device/recipient 0 then recipient 1) aggregate into ONE repeat (not 2!),
    // AND duplicate callback for recipient 0 is ignored!
    let r0_sent = WriteResult::uniform(
        0,
        0,
        WriteKind::DisableRequested,
        d0.dispatch_id,
        d0.generation,
        61_000,
        61_000,
        ChannelMask::bit(0),
        MotorWriteOutcome::Sent,
    );
    checked_step(&mut s, Event::WriteResult(r0_sent));
    assert_eq!(
        s.groups[0].torque_intent,
        TorqueIntent::Off,
        "first Sent commits Off immediately"
    );
    // Duplicate callback for r0 MUST NOT advance repeat count!
    checked_step(&mut s, Event::WriteResult(r0_sent));
    assert_eq!(s.groups[0].disable_tx.unwrap().repeats_completed, 0);

    // Second recipient reports Deferred -> aggregates with r0 into repeat 1 (B1), r1 still NOT physically Sent off!
    let r1_def = WriteResult::uniform(
        0,
        0,
        WriteKind::DisableRequested,
        d0.dispatch_id,
        d0.generation,
        61_500,
        61_500,
        ChannelMask::bit(1),
        MotorWriteOutcome::Deferred,
    );
    checked_step(&mut s, Event::WriteResult(r1_def));
    assert_eq!(s.groups[0].disable_tx.unwrap().repeats_completed, 1);

    // B10: Recipient 1 (which was Deferred!) now trips thermal while group intent is ALREADY Off!
    // Thermal MUST preempt ordinary repeats and emit DisableProtective with reserved budget!
    let hot = checked_step(
        &mut s,
        Event::Thermal {
            channel: 1,
            temperature_c: 75.0,
            now_us: 65_000,
        },
    );
    let dp = hot
        .disable_for(0)
        .expect("B10: thermal on unresolved recipient must schedule DisableProtective");
    assert_eq!(dp.kind, DisableKind::Protective);
    assert!(dp.use_reserved_budget);
    assert!(dp.recipients.contains(1));

    // B1: Delayed stale callback with old d0.dispatch_id MUST NOT match new protective dispatch dp!
    checked_step(
        &mut s,
        Event::WriteResult(WriteResult::uniform(
            0,
            0,
            WriteKind::DisableRequested,
            d0.dispatch_id,
            d0.generation,
            66_000,
            66_000,
            ChannelMask::from_count(2),
            MotorWriteOutcome::Sent,
        )),
    );
    assert!(
        s.groups[0].pending_disable_dispatch.is_some_and(|p| p.dispatch_id == dp.dispatch_id),
        "B1: stale dispatch_id must not reconcile live dispatch"
    );
}

#[test]
fn thermal_derate_motion_m7_per_channel_damping_m8_and_delayed_cool_m6() {
    let mut cfg = single_arm_config(2);
    for c in 0..2 {
        cfg.channels[c].desired_mode = ControlMode::Impedance;
        cfg.channels[c].supports_mit_damping = true;
        cfg.channels[c].thermal = ThermalConfig {
            derate_start_c: Some(50.0),
            trip_c: 70.0,
        };
    }
    // M8: Two MIT channels with DIFFERENT damping durations (10 ms vs 50 ms)
    cfg.channels[0].damping = DampingConfig {
        duration_us: 10_000,
        kd: 1.0,
        max_torque_nm: 3.0,
    };
    cfg.channels[1].damping = DampingConfig {
        duration_us: 50_000,
        kd: 2.0,
        max_torque_nm: 5.0,
    };
    let mut s = TorqueState::new(cfg).unwrap();
    enable_group_clean(&mut s, 0, 1_000);

    // M7: Warm temperature scales MotionAllowed.thermal_derate_scales authoritatively
    checked_step(
        &mut s,
        Event::Thermal {
            channel: 0,
            temperature_c: 60.0, // 0.5
            now_us: 10_000,
        },
    );
    let m_act = checked_step(
        &mut s,
        Event::MotionCommand {
            group: 0,
            channels: ChannelMask::from_count(2),
            now_us: 11_000,
        },
    );
    let ma = m_act.motion_allowed[0].expect("motion allowed");
    assert!((ma.thermal_derate_scales[0] - 0.5).abs() < 1e-6);
    assert!((ma.thermal_derate_scales[1] - 1.0).abs() < 1e-6);

    // M6: Hot sample at t=20_000 trips thermal; delayed older cool sample (rx=15_000) MUST NOT clear hot mask!
    let mut hot_s = sample_at(0, 10, 20_000, 0.0);
    hot_s.temperature_c = Some(72.0);
    let a_trip = checked_step(&mut s, Event::CausalSample(hot_s));
    assert_eq!(s.groups[0].drive_state, DriveState::ProtectiveDamping);
    assert_eq!(a_trip.damp_for(0).unwrap().end_us, 30_000); // 20_000 + 10_000
    assert_eq!(a_trip.damp_for(1).unwrap().end_us, 70_000); // 20_000 + 50_000

    let mut old_cool = sample_at(0, 11, 15_000, 0.0);
    old_cool.now_us = 25_000;
    old_cool.temperature_c = Some(40.0);
    checked_step(&mut s, Event::CausalSample(old_cool));
    assert!(
        s.groups[0].thermal_tripped_mask.contains(0),
        "M6: out-of-order cool sample must NOT clear thermal trip"
    );

    // M8: At t = 35_000 (past ch0's 30_000 end, before ch1's 70_000 end): ONLY ch1 still damps!
    let r_mid = checked_step(&mut s, Event::Release { now_us: 35_000 });
    assert!(r_mid.damp_for(0).is_none(), "M8: ch0 stopped damping at its 10 ms end");
    assert!(r_mid.damp_for(1).is_some(), "M8: ch1 still damping until 50 ms end");

    // At t = 70_000: ch1 ends -> transitions to DisableProtective with reserved budget!
    let r_done = checked_step(&mut s, Event::Release { now_us: 70_000 });
    assert_eq!(s.groups[0].drive_state, DriveState::ProtectiveDisabling);
    assert!(r_done
        .disable_for(0)
        .is_some_and(|d| d.kind == DisableKind::Protective && d.use_reserved_budget));
}

#[test]
fn fu_panic_park_authority_b3_m1_m2_and_safehold_b11_b12() {
    // B3, M1, M2: WorkerPanic park handoff, Stop cancellation, and external DisableRequest fencing
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
    let park_req = wp.park_for(0).expect("arm group requests park");
    assert!(wp.park_for(1).is_none(), "hand group must NOT park");

    // M1: Before ParkAccepted, releases MUST continue emitting holds (zero hold-to-park gap)!
    let r_pre_accept = checked_step(&mut s1, Event::Release { now_us: 12_000 });
    assert!(
        r_pre_accept.hold_for(0).is_some(),
        "M1: holds continue until park capability is accepted"
    );

    // M2: External DisableRequest is refused with INTERNAL_FAULT while worker panic is latched!
    let ext_dis = checked_step(
        &mut s1,
        Event::DisableRequest {
            group: 0,
            now_us: 13_000,
            explicit_release_authorized: false,
            recipient_mask_override: None,
        },
    );
    assert_eq!(
        ext_dis.refusal_for(0).map(|r| r.code),
        Some(RefusalCode::InternalFault)
    );

    // B3: Stop cancels pending park; delayed ParkSettled(park_req.park_id, true) MUST NOT open disable!
    let mut s_cancelled = s1;
    checked_step(
        &mut s_cancelled,
        Event::Stop {
            groups: GroupMask::bit(0),
            kind: StopKind::Stop,
            now_us: 15_000,
        },
    );
    checked_step(
        &mut s_cancelled,
        Event::ParkSettled {
            group: 0,
            park_id: park_req.park_id,
            succeeded: true,
            now_us: 20_000,
        },
    );
    assert!(
        s_cancelled.groups[0].disable_tx.is_none(),
        "B3: Stop-cancelled ParkSettled must NOT open automatic disable"
    );

    // Valid flow: ParkAccepted -> holds suppressed while parking -> ParkSettled(true) opens arm disable only!
    checked_step(
        &mut s1,
        Event::ParkAccepted {
            group: 0,
            park_id: park_req.park_id,
            now_us: 14_000,
        },
    );
    let r_parking = checked_step(&mut s1, Event::Release { now_us: 16_000 });
    assert!(r_parking.hold_for(0).is_none(), "holds paused while accepted park executes");
    assert!(r_parking.hold_for(6).is_some(), "hand group still holding grasp");
    checked_step(
        &mut s1,
        Event::ParkSettled {
            group: 0,
            park_id: park_req.park_id,
            succeeded: true,
            now_us: 20_000,
        },
    );
    assert!(s1.groups[0].disable_tx.is_some());
    assert!(s1.groups[1].disable_tx.is_none());

    // B11: Two devices with different panic_bus_loss_limit_us in SafeHold track independent timers!
    let mut cfg = TorqueConfig::default();
    cfg.channel_count = 2;
    cfg.recipient_count = 2;
    cfg.group_count = 1;
    cfg.device_count = 2;
    cfg.devices[0] = DeviceConfig {
        id: 0,
        bus_id: 0,
        panic_bus_loss_limit_us: 500_000,
        ..DeviceConfig::default()
    };
    cfg.devices[1] = DeviceConfig {
        id: 1,
        bus_id: 1,
        panic_bus_loss_limit_us: 200_000,
        ..DeviceConfig::default()
    };
    cfg.groups[0] = GroupConfig {
        id: 0,
        channels: ChannelMask::from_count(2),
        recipients: ChannelMask::from_count(2),
        devices: DeviceMask::bit(0).union(DeviceMask::bit(1)),
        ..GroupConfig::default()
    };
    for i in 0..2u8 {
        cfg.channels[i as usize] = ChannelConfig {
            id: i,
            device: i,
            group: 0,
            recipient: i,
            ..ChannelConfig::default()
        };
        cfg.recipients[i as usize] = RecipientConfig {
            id: i,
            device: i,
            group: 0,
            channels: ChannelMask::bit(i),
            grouped_packet_mask: ChannelMask::bit(i),
            ..RecipientConfig::default()
        };
    }
    let mut s2 = TorqueState::new(cfg).unwrap();
    enable_group_clean(&mut s2, 0, 1_000);
    checked_step(&mut s2, Event::CausalSample(sample_at(0, 1, 5_000, 0.0)));
    checked_step(&mut s2, Event::CausalSample(sample_at(1, 2, 5_000, 0.0)));
    checked_step(
        &mut s2,
        Event::Panic {
            tier: PanicTier::CaughtControlPanic,
            now_us: 10_000,
        },
    );
    // Device 0 goes stale at t = 120_000 while Device 1 stays fresh!
    checked_step(&mut s2, Event::CausalSample(sample_at(1, 3, 120_000, 0.0)));
    let r_a = checked_step(&mut s2, Event::Release { now_us: 120_000 });
    assert!(!r_a.any_disable());
    // Keep Dev 1 fresh until 250_000, then Dev 1 goes stale at 360_000 (Dev 0 elapsed 240ms < 500ms; Dev 1 elapsed 0ms < 200ms!)
    checked_step(&mut s2, Event::CausalSample(sample_at(1, 4, 250_000, 0.0)));
    let r_b = checked_step(&mut s2, Event::Release { now_us: 360_000 });
    assert!(
        !r_b.any_disable(),
        "B11: Device 1 must NOT borrow Device 0's elapsed loss time!"
    );
    // Repeated CaughtControlPanic MUST be idempotent and NOT clear Device 0/1 timers!
    checked_step(
        &mut s2,
        Event::Panic {
            tier: PanicTier::CaughtControlPanic,
            now_us: 370_000,
        },
    );
    assert_eq!(s2.devices[0].safe_hold_bus_loss_since_us, Some(120_000));
    assert_eq!(s2.devices[1].safe_hold_bus_loss_since_us, Some(360_000));

    // At t = 560_000 (Dev 1 elapsed 200ms >= 200ms): trips ProtectivePanicBusLoss!
    let r_trip = checked_step(&mut s2, Event::Release { now_us: 560_000 });
    let dp = r_trip.disable_for(0).expect("Dev 1 crossed its own 200 ms limit");
    assert_eq!(dp.origin, DisableOrigin::ProtectivePanicBusLoss);

    // B12: All protective writes fail/defer through deadline expiry -> resumes SafeHold and holds once feedback returns!
    checked_step(
        &mut s2,
        Event::WriteResult(WriteResult::uniform(
            0,
            0,
            WriteKind::DisableProtective,
            dp.dispatch_id,
            dp.generation,
            560_000 + DEFAULT_DISABLE_DEADLINE_US,
            560_000 + DEFAULT_DISABLE_DEADLINE_US,
            ChannelMask::from_count(2),
            MotorWriteOutcome::Deferred,
        )),
    );
    assert_eq!(
        s2.groups[0].drive_state,
        DriveState::SafeHold,
        "B12: zero-write expiry resumes SafeHold instead of stranding ProtectiveDisabling"
    );
    let t_rec = s2.now_us + 10_000;
    checked_step(&mut s2, Event::CausalSample(sample_at(0, 20, t_rec, 0.1)));
    checked_step(&mut s2, Event::CausalSample(sample_at(1, 21, t_rec, 0.2)));
    let r_resumed = checked_step(&mut s2, Event::Release { now_us: t_rec });
    assert!(r_resumed.hold_for(0).is_some());
    assert!(r_resumed.hold_for(1).is_some());
}

#[test]
fn contention_escalation_once_per_release_m10_and_shared_bus_m14() {
    let mut cfg = TorqueConfig::arm_plus_underactuated_hand();
    // Put both devices on bus 0 with matching shared-bus quotas (M14)
    cfg.devices[1].bus_id = 0;
    cfg.devices[1].tx_quota_per_release = cfg.devices[0].tx_quota_per_release;
    cfg.devices[1].reserved_safety_tx = cfg.devices[0].reserved_safety_tx;
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

    // M10: 30 LockContended callbacks inside ONE release count as ONLY 1 skipped release!
    let mut t = 20_000u64;
    checked_step(&mut s, Event::Release { now_us: t });
    for _ in 0..30 {
        checked_step(
            &mut s,
            Event::WriteResult(WriteResult::uniform(
                0,
                0,
                WriteKind::Hold,
                0,
                1,
                t,
                t,
                ChannelMask::from_count(6),
                MotorWriteOutcome::LockContended,
            )),
        );
    }
    // Unrelated successful Refresh MUST NOT reset hold contention count (M10)!
    checked_step(
        &mut s,
        Event::WriteResult(WriteResult::uniform(
            0,
            0,
            WriteKind::Refresh,
            0,
            1,
            t + 500,
            t + 500,
            ChannelMask::bit(0),
            MotorWriteOutcome::Sent,
        )),
    );
    t += 20_000;
    checked_step(&mut s, Event::Release { now_us: t });
    assert_eq!(s.devices[0].consecutive_contention_skips, 1);

    // 23 more contended releases -> total 24, still NOT escalated
    for _ in 0..23 {
        checked_step(
            &mut s,
            Event::WriteResult(WriteResult::uniform(
                0,
                0,
                WriteKind::Hold,
                0,
                1,
                t,
                t,
                ChannelMask::from_count(6),
                MotorWriteOutcome::LockContended,
            )),
        );
        t += 20_000;
        let r = checked_step(&mut s, Event::Release { now_us: t });
        assert!(r.contention_escalations.is_empty());
    }
    assert_eq!(s.devices[0].consecutive_contention_skips, 24);

    // 25th contended release escalates ONCE and bumps stop_generation!
    let gen_before = s.groups[0].stop_generation;
    checked_step(
        &mut s,
        Event::WriteResult(WriteResult::uniform(
            0,
            0,
            WriteKind::Hold,
            0,
            gen_before,
            t,
            t,
            ChannelMask::from_count(6),
            MotorWriteOutcome::LockContended,
        )),
    );
    t += 20_000;
    let esc = checked_step(&mut s, Event::Release { now_us: t });
    assert!(esc.contention_escalations.contains(0));
    assert!(s.groups[0].faults.contains(TorqueFaultKind::DriverWriteFailed));
    assert!(s.groups[0].stop_generation > gen_before);
    assert!(!s.groups[0].protective_latch);

    // 26th contended release does NOT re-escalate repeatedly (M10)!
    let gen26 = s.groups[0].stop_generation;
    checked_step(
        &mut s,
        Event::WriteResult(WriteResult::uniform(
            0,
            0,
            WriteKind::Hold,
            0,
            gen26,
            t,
            t,
            ChannelMask::from_count(6),
            MotorWriteOutcome::LockContended,
        )),
    );
    t += 20_000;
    let r26 = checked_step(&mut s, Event::Release { now_us: t });
    assert!(r26.contention_escalations.is_empty());
}

#[test]
fn active_tool_positionless_stop_m5_and_bounded_retention() {
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
    // M5: NO positional sample sent for ch0 (StopOutput)! Stop MUST STILL emit ActiveToolStop!
    checked_step(&mut s, Event::CausalSample(sample_at(1, 2, 5_000, 0.0)));
    checked_step(
        &mut s,
        Event::Stop {
            groups: GroupMask::bit(0),
            kind: StopKind::Stop,
            now_us: 10_000,
        },
    );
    let r1 = checked_step(&mut s, Event::Release { now_us: 50_000 });
    let h0 = r1.hold_for(0).expect("M5: StopOutput emitted even with no position capture");
    assert_eq!(h0.hold_kind, HoldKind::ActiveToolStop);
    assert!(!h0.uses_position_capture);
    assert_eq!(
        r1.hold_for(1).unwrap().hold_kind,
        HoldKind::ActiveToolRetain {
            remaining_us: 60_000
        }
    );

    // Past retention bound (at 120_000) without fresh position -> ch1 STILL emits ActiveToolStop!
    let r2 = checked_step(&mut s, Event::Release { now_us: 120_000 });
    assert_eq!(r2.hold_for(1).unwrap().hold_kind, HoldKind::ActiveToolStop);
}

// =========================================================================
// 2. Synthetic Underactuated Hand Group Next to Arm Group (incl. B6, B8, M14)
// =========================================================================

#[test]
fn synthetic_underactuated_hand_next_to_arm_group() {
    let mut s = TorqueState::new(TorqueConfig::arm_plus_underactuated_hand()).unwrap();
    verify_all_modes(&mut s, 1_000);
    enable_group_clean(&mut s, 0, 2_000);
    enable_group_clean(&mut s, 1, 10_000);

    // Observe VerifiedEmptyGrasp, then grasp a payload (GraspStateUpdate -> invalidates old empty grasp, B6!)
    for c in 6..12u8 {
        let mut emp = sample_at(c, 50, 15_000, 0.0);
        emp.release_evidence = Some(ReleaseEvidence::VerifiedEmptyGrasp);
        checked_step(&mut s, Event::CausalSample(emp));
    }
    checked_step(
        &mut s,
        Event::GraspStateUpdate {
            channel: 6,
            setpoint_rad: -0.8,
            force_limit_raw: Some(220),
            now_us: 16_000,
        },
    );
    assert_eq!(
        s.joints[6].last_release_evidence,
        ReleaseEvidence::Unknown,
        "B6: GraspStateUpdate invalidates prior empty-grasp evidence"
    );

    // Stop both groups; open hand disable with explicit_release_authorized = true
    for c in 0..13u8 {
        checked_step(
            &mut s,
            Event::CausalSample(sample_at(c, 100 + c as u64, 20_000, if c < 6 { 0.0 } else { -0.8 })),
        );
    }
    checked_step(
        &mut s,
        Event::Stop {
            groups: GroupMask::bit(0).union(GroupMask::bit(1)),
            kind: StopKind::Stop,
            now_us: 25_000,
        },
    );

    // B6: 3 fresh position samples with release_evidence = None MUST defer (NOT inherit old empty grasp)!
    checked_step(
        &mut s,
        Event::DisableRequest {
            group: 1,
            now_us: 30_000,
            explicit_release_authorized: true,
            recipient_mask_override: Some(ChannelMask::bit(6)), // expands to {6, 7} == group 1!
        },
    );
    for (k, t) in [(1u64, 40_000u64), (2, 50_000), (3, 60_000)] {
        checked_step(&mut s, Event::Release { now_us: t });
        for hc in 6..12u8 {
            let act = checked_step(&mut s, Event::CausalSample(sample_at(hc, 200 + k, t, -0.8)));
            assert!(
                act.disable_for(1).is_none(),
                "B6: missing release_evidence must defer, never inherit stale empty grasp"
            );
        }
    }

    // B8: Multi-channel replay attack on recipient 6 (ch6,7,8):
    // rel 1: all 6..12 fresh (id 301); rel 2: all 6..12 fresh (id 302);
    // rel 3: ch6 replays id 302 while ch7..11 have fresh id 303 -> MUST NOT PASS!
    for (id, t) in [(301u64, 70_000u64), (302, 80_000)] {
        checked_step(&mut s, Event::Release { now_us: t });
        for hc in 6..12u8 {
            let mut hs = sample_at(hc, id, t, -0.8);
            hs.release_evidence = Some(ReleaseEvidence::SupportedObject);
            checked_step(&mut s, Event::CausalSample(hs));
        }
    }
    checked_step(&mut s, Event::Release { now_us: 90_000 });
    for hc in 6..12u8 {
        let sid = if hc == 6 { 302 } else { 303 }; // ch6 replays sample_id 302!
        let mut hs = sample_at(hc, sid, 90_000, -0.8);
        hs.release_evidence = Some(ReleaseEvidence::SupportedObject);
        let act = checked_step(&mut s, Event::CausalSample(hs));
        assert!(
            act.disable_for(1).is_none(),
            "B8: replayed sample_id on ch6 must prevent recipient 6 from passing K=3"
        );
    }

    // Now supply genuine unused sample_id 304 for ch6 in release 90_000 -> passes and emits 1 grouped packet!
    let mut ch6_fresh = sample_at(6, 304, 90_000, -0.8);
    ch6_fresh.release_evidence = Some(ReleaseEvidence::SupportedObject);
    let act_pass = checked_step(&mut s, Event::CausalSample(ch6_fresh));
    let hd = act_pass.disable_for(1).expect("hand disable authorized once all mapped channels have unused sample_ids");
    assert_eq!(hd.recipients, ChannelMask::bit(6).union(ChannelMask::bit(7)));
    assert_eq!(hd.packet_count, 1, "M14: grouped hand packet charged once");
    assert_eq!(act_pass.telemetry.bus_budgets[1].stop_hold_demand, 1);

    // Commit hand Off; arm remains On and held!
    checked_step(
        &mut s,
        Event::WriteResult(WriteResult::uniform(
            1,
            1,
            WriteKind::DisableRequested,
            hd.dispatch_id,
            hd.generation,
            91_000,
            91_000,
            hd.recipients,
            MotorWriteOutcome::Sent,
        )),
    );
    assert_eq!(s.groups[1].torque_intent, TorqueIntent::Off);
    assert_eq!(s.groups[0].torque_intent, TorqueIntent::On);

    // Keep arm fresh and verify arm holds active while hand has zero holds
    for arm_c in 0..6u8 {
        checked_step(&mut s, Event::CausalSample(sample_at(arm_c, 900 + arm_c as u64, 95_000, 0.0)));
    }
    let rel_after = checked_step(&mut s, Event::Release { now_us: 100_000 });
    for arm_c in 0..6u8 {
        assert!(rel_after.hold_for(arm_c).is_some());
    }
    for hand_c in 6..12u8 {
        assert!(rel_after.hold_for(hand_c).is_none());
    }
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
    let next_id = base.next_capability_id;
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
            next_id,
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
            next_id,
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
        let mut auth_id: Option<u64> = None;
        let mut first_outcome_was_sent = false;
        let mut resolved = false;
        for &ev in perm {
            let act = checked_step(&mut s, ev);
            if let Some(d) = act.disable_for(0) {
                auth_id = Some(d.dispatch_id);
            }
            if let (Some(id), Event::WriteResult(w)) = (auth_id, ev) {
                if !resolved && w.dispatch_id == id && w.generation == gen0 {
                    resolved = true;
                    first_outcome_was_sent = w.outcomes[0] == MotorWriteOutcome::Sent;
                }
            }
        }
        if first_outcome_was_sent {
            assert_eq!(s.groups[0].torque_intent, TorqueIntent::Off);
        } else {
            assert_eq!(s.groups[0].torque_intent, TorqueIntent::On);
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
    let mut d0 = None;
    for (k, t) in [(1u64, 40_000u64), (2, 50_000), (3, 60_000)] {
        checked_step(&mut base, Event::Release { now_us: t });
        checked_step(&mut base, Event::CausalSample(sample_at(0, k * 2, t, 0.0)));
        let a = checked_step(&mut base, Event::CausalSample(sample_at(1, k * 2 + 1, t, 0.0)));
        if k == 3 {
            d0 = a.disable_for(0);
        }
    }
    let d0 = d0.unwrap();
    let mut partial = WriteResult::uniform(
        0,
        0,
        WriteKind::DisableRequested,
        d0.dispatch_id,
        d0.generation,
        61_000,
        61_000,
        ChannelMask::from_count(2),
        MotorWriteOutcome::Deferred,
    );
    partial.outcomes[0] = MotorWriteOutcome::Ambiguous;

    let mut race = vec![
        Event::WriteResult(partial),
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
        Event::Release { now_us: 1_020_000 },
    ];

    let mut count = 0usize;
    let mut saw_repeat1_perms = 0usize;
    let mut saw_post_complete_enable = 0usize;
    for_each_permutation(&mut race, |perm| {
        count += 1;
        let mut s = base;
        let mut saw_repeat1 = false;
        for &ev in perm {
            let was_fenced = s.groups[0].disable_tx.is_some()
                || s.groups[0].pending_disable_dispatch.is_some();
            let act = checked_step(&mut s, ev);
            // Enable and motion MUST be refused while disable_tx or pending_disable_dispatch is active
            if was_fenced {
                assert!(act.enable_for(0).is_none());
                assert!(act.motion_allowed[0].is_none());
            } else if act.enable_for(0).is_some() {
                saw_post_complete_enable += 1;
            }
            if let Some(d) = act.disable_for(0) {
                if d.repeat_index == 1 {
                    saw_repeat1 = true;
                    // Feed correlated write result for the actually emitted repeat 1 (M16)!
                    let t_rep = s.now_us + 500;
                    checked_step(
                        &mut s,
                        Event::WriteResult(WriteResult::uniform(
                            0,
                            0,
                            WriteKind::DisableRequested,
                            d.dispatch_id,
                            d.generation,
                            t_rep,
                            t_rep,
                            d.recipients,
                            MotorWriteOutcome::Sent,
                        )),
                    );
                }
            }
        }
        if saw_repeat1 {
            saw_repeat1_perms += 1;
        }
    });
    assert_eq!(count, 120);
    assert!(saw_repeat1_perms > 0);
    assert!(saw_post_complete_enable > 0);
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
    checked_step(&mut base, Event::CausalSample(sample_at(0, 1, dl - 40_000, 0.0)));
    checked_step(&mut base, Event::Release { now_us: dl - 20_000 });
    checked_step(&mut base, Event::CausalSample(sample_at(0, 2, dl - 20_000, 0.0)));

    let mut total = 0usize;
    let next_id = base.next_capability_id;
    for t in [dl - 1, dl, dl + 1] {
        let mut race = vec![
            Event::Release { now_us: t },
            Event::CausalSample(sample_at(0, 3, t, 0.0)),
            Event::WriteResult(WriteResult::uniform(
                0,
                0,
                WriteKind::DisableRequested,
                next_id,
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
    let ecmd = checked_step(
        &mut base,
        Event::EnableStart {
            group: 0,
            now_us: 10_000,
        },
    )
    .enable_for(0)
    .unwrap();

    let mut race = vec![
        Event::HandshakeResult {
            group: 0,
            attempt_id: ecmd.attempt_id,
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
        assert_eq!(s.groups[0].torque_intent, TorqueIntent::On);
    });
    assert_eq!(count, 120);
}

// =========================================================================
// 4. Seeded Random Event-Sequence Model Test (10,000 × <= 128) with Independent Oracle (M15)
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

/// Independent capability oracle recording emitted commands so the generator produces BOTH
/// correlated receipts and adversarial stale/duplicate receipts (M15).
#[derive(Default)]
struct CommandOracle {
    emitted_disables: [Option<DisableCommand>; MAX_GROUPS],
    stale_disables: [Option<DisableCommand>; MAX_GROUPS],
    emitted_enables: [Option<EnableCommand>; MAX_GROUPS],
    stale_enables: [Option<EnableCommand>; MAX_GROUPS],
    emitted_parks: [Option<ParkRequest>; MAX_GROUPS],
    stale_parks: [Option<ParkRequest>; MAX_GROUPS],
}

impl CommandOracle {
    fn record(&mut self, actions: &ActionSet) {
        for g in 0..MAX_GROUPS {
            if let Some(d) = actions.disables[g] {
                if self.emitted_disables[g].is_some_and(|old| old.dispatch_id != d.dispatch_id) {
                    self.stale_disables[g] = self.emitted_disables[g];
                }
                self.emitted_disables[g] = Some(d);
            }
            if let Some(e) = actions.enables[g] {
                if self.emitted_enables[g].is_some_and(|old| old.attempt_id != e.attempt_id) {
                    self.stale_enables[g] = self.emitted_enables[g];
                }
                self.emitted_enables[g] = Some(e);
            }
            if let Some(p) = actions.park_requests[g] {
                if self.emitted_parks[g].is_some_and(|old| old.park_id != p.park_id) {
                    self.stale_parks[g] = self.emitted_parks[g];
                }
                self.emitted_parks[g] = Some(p);
            }
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
    oracle: &CommandOracle,
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

    match rng.gen_range(26) {
        0..=4 => Event::Release { now_us },
        5..=9 => {
            if rng.gen_range(6) != 0 {
                *sample_seq += 1;
            }
            let rx_ts = match rng.gen_range(10) {
                0 => now_us.saturating_add(1),
                1 => now_us.saturating_sub(100_001),
                2 => now_us.saturating_sub(100_000),
                _ => now_us,
            };
            let pos = match rng.gen_range(6) {
                0 => f32::NAN,
                1 => 0.45,
                2 => 0.08,
                _ => 0.0,
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
            let poll_us = if matches!(state.config.channels[c as usize].vendor, VendorKind::Feetech)
                && rng.gen_range(3) != 0
            {
                Some(rx_ts)
            } else {
                None
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
                correlated_after_write_done_us: poll_us,
                poll_issued_us: poll_us,
                release_evidence: ev,
                is_handshake_ack_only: rng.gen_range(10) == 0,
            })
        }
        10..=12 => {
            // M15: Generate WriteResults from recorded live OR stale emitted commands!
            let use_stale = rng.gen_range(4) == 0;
            let d_cmd = if use_stale {
                oracle.stale_disables[g as usize].or(oracle.emitted_disables[g as usize])
            } else {
                oracle.emitted_disables[g as usize]
            };
            let (kind, dispatch_id, gen, target_mask) = if let Some(dc) = d_cmd.filter(|_| rng.gen_range(2) == 0) {
                let k = match dc.kind {
                    DisableKind::Requested => WriteKind::DisableRequested,
                    DisableKind::Protective => WriteKind::DisableProtective,
                };
                // Sometimes split callbacks per recipient!
                let mask = if rng.gen_range(3) == 0 {
                    dc.recipients
                        .iter()
                        .next()
                        .map_or(dc.recipients, ChannelMask::bit)
                } else {
                    dc.recipients
                };
                (k, dc.dispatch_id, dc.generation, mask)
            } else {
                let kind = match rng.gen_range(4) {
                    0 => WriteKind::Hold,
                    1 => WriteKind::Motion,
                    2 => WriteKind::Enable,
                    _ => WriteKind::Refresh,
                };
                let e_id = if use_stale {
                    oracle.stale_enables[g as usize].map_or(0, |e| e.attempt_id)
                } else {
                    oracle.emitted_enables[g as usize].map_or(0, |e| e.attempt_id)
                };
                (
                    kind,
                    if kind == WriteKind::Enable { e_id } else { 0 },
                    state.groups[g as usize].stop_generation,
                    state.config.groups[g as usize].channels,
                )
            };
            let mut wr = WriteResult {
                group: g,
                device: d,
                kind,
                dispatch_id,
                generation: gen,
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
        13 => Event::EnableStart { group: g, now_us },
        14 => {
            let attempt_id = if rng.gen_range(4) == 0 {
                oracle.stale_enables[g as usize].map_or(999, |e| e.attempt_id)
            } else {
                oracle.emitted_enables[g as usize].map_or(0, |e| e.attempt_id)
            };
            Event::HandshakeResult {
                group: g,
                attempt_id,
                now_us,
                confirmed_mask: state.config.groups[g as usize].channels,
                outcome: match rng.gen_range(4) {
                    0 => HandshakeOutcome::Pending,
                    1 => HandshakeOutcome::Confirmed,
                    2 => HandshakeOutcome::Failed,
                    _ => HandshakeOutcome::FaultReported { channel: c },
                },
            }
        }
        15 => Event::DisableRequest {
            group: g,
            now_us,
            explicit_release_authorized: rng.gen_range(2) == 0,
            recipient_mask_override: if rng.gen_range(5) == 0 {
                Some(ChannelMask::bit(c))
            } else {
                None
            },
        },
        16 => Event::Stop {
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
        17 => Event::FaultClear {
            groups: GroupMask::bit(g),
            now_us,
        },
        18 => Event::Thermal {
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
        19 => Event::MitOwnershipChange {
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
        20 => Event::ModeVerified {
            channel: c,
            mode: state.joints[c as usize].desired_mode,
            now_us,
        },
        21 => Event::TimeoutReadback {
            channel: c,
            timeout_reg: if rng.gen_range(2) == 0 { 900 } else { 850 },
            now_us,
        },
        22 => Event::MotionCommand {
            group: g,
            channels: state.config.groups[g as usize].channels,
            now_us,
        },
        23..=24 => {
            let pid = if rng.gen_range(4) == 0 {
                oracle.stale_parks[g as usize].map_or(777, |p| p.park_id)
            } else {
                oracle.emitted_parks[g as usize].map_or(0, |p| p.park_id)
            };
            match rng.gen_range(5) {
                0 => Event::SerialTimeout { device: d, now_us },
                1 => Event::ParkAccepted {
                    group: g,
                    park_id: pid,
                    now_us,
                },
                2 => Event::ParkSettled {
                    group: g,
                    park_id: pid,
                    succeeded: rng.gen_range(2) == 0,
                    now_us,
                },
                3 => Event::BusLoss { device: d, now_us },
                _ => Event::GraspStateUpdate {
                    channel: c,
                    setpoint_rad: -0.5,
                    force_limit_raw: Some((rng.gen_range(400) + 100) as u16),
                    now_us,
                },
            }
        }
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
        let mut oracle = CommandOracle::default();
        let len = 1 + (rng.gen_range(MAX_EVENTS as u64) as usize);
        let mut trace = Vec::with_capacity(len);
        let mut sample_seq = 0u64;

        for step_idx in 0..len {
            let ev = gen_random_event(&mut rng, &state, &oracle, &mut sample_seq);
            trace.push(ev);
            let prev = state;
            let actions = step(&mut state, ev);
            oracle.record(&actions);
            assert_priority_order(&state, &actions);
            if let Err(v) = check_invariants(&state, &prev, &ev, &actions) {
                // m1: Shrink strictly against the ORIGINAL violation kind and report serialization errors!
                let orig_kind = v.kind;
                let (min_cfg, min_trace) = shrink_trace(cfg, &trace, |c, evs| {
                    replay_for_violation(c, evs).is_some_and(|(_, rv)| rv.kind == orig_kind)
                });
                let fixture = RegressionFixture {
                    name: format!("shrunk_seq_{seq}"),
                    description: format!("auto-shrunk failure: {v:?}"),
                    topology: FixtureTopology::Custom(Box::new(min_cfg)),
                    events: min_trace,
                    step_checks: Vec::new(),
                    expected_group_intents: Vec::new(),
                    expected_group_states: Vec::new(),
                };
                let json = serde_json::to_string_pretty(&fixture)
                    .unwrap_or_else(|e| format!("<fixture serialization failed: {e}>"));
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
// 5. Trace Shrinker (m1) & Event-Indexed Regression Fixtures (M16)
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

/// Event-indexed assertion inside a regression fixture (M16).
#[derive(Debug, Clone, Serialize, Deserialize)]
struct StepCheck {
    step: usize,
    group: usize,
    expect_intent: TorqueIntent,
    expect_state: DriveState,
    expect_disable: bool,
    #[serde(default)]
    expect_hold_ch0: Option<bool>,
    #[serde(default)]
    expect_completion_partial: Option<bool>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct RegressionFixture {
    name: String,
    description: String,
    topology: FixtureTopology,
    events: Vec<Event>,
    #[serde(default)]
    step_checks: Vec<StepCheck>,
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
        | Event::ParkAccepted { now_us, .. }
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

fn shrink_trace(
    mut cfg: TorqueConfig,
    events: &[Event],
    mut fails: impl FnMut(TorqueConfig, &[Event]) -> bool,
) -> (TorqueConfig, Vec<Event>) {
    let mut cur = events.to_vec();
    if !fails(cfg, &cur) {
        return (cfg, cur);
    }

    // Pass 1: delete event spans
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

    // Pass 3: move timestamps toward boundaries
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
    assert_eq!(min_cfg.channel_count, 1);
    assert_eq!(min_trace.len(), 2);
    assert_eq!(event_now_us(&min_trace[0]), 0);
    assert_eq!(event_now_us(&min_trace[1]), 0);

    // m1: Round-trip serialized minimized fixture through JSON cleanly
    let fx = RegressionFixture {
        name: "shrunk_roundtrip".into(),
        description: "roundtrip test".into(),
        topology: FixtureTopology::SingleArm {
            joints: min_cfg.channel_count,
            poison_silence_us: 0,
            damping_duration_us: 0,
            impedance_mode: false,
        },
        events: min_trace,
        step_checks: Vec::new(),
        expected_group_intents: vec![TorqueIntent::On],
        expected_group_states: vec![DriveState::FailedHold],
    };
    let json = serde_json::to_string_pretty(&fx).expect("fixture must serialize");
    let parsed: RegressionFixture = serde_json::from_str(&json).expect("fixture must deserialize");
    assert!(predicate(parsed.topology.build(), &parsed.events));
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
        files.len() >= 7,
        "expected >= 7 regression fixtures in {}, found {}",
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
            // M16: Enforce event-indexed step assertions
            for chk in fx.step_checks.iter().filter(|c| c.step == i) {
                assert_eq!(
                    s.groups[chk.group].torque_intent, chk.expect_intent,
                    "fixture {} step {i} group {} intent mismatch",
                    fx.name, chk.group
                );
                assert_eq!(
                    s.groups[chk.group].drive_state, chk.expect_state,
                    "fixture {} step {i} group {} state mismatch",
                    fx.name, chk.group
                );
                assert_eq!(
                    actions.disable_for(chk.group as u8).is_some(),
                    chk.expect_disable,
                    "fixture {} step {i} group {} disable mismatch",
                    fx.name, chk.group
                );
                if let Some(exp_hold) = chk.expect_hold_ch0 {
                    assert_eq!(
                        actions.hold_for(0).is_some(),
                        exp_hold,
                        "fixture {} step {i} hold ch0 mismatch",
                        fx.name
                    );
                }
                if let Some(exp_partial) = chk.expect_completion_partial {
                    let comp = actions
                        .completion_for(chk.group as u8)
                        .unwrap_or_else(|| panic!("fixture {} step {i} expected completion", fx.name));
                    assert_eq!(comp.partial_or_ambiguous, exp_partial);
                }
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
// 6. Zero-Allocation Test on step(), is_disable_authorized(), and check_invariants()
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

    let cfg = TorqueConfig::arm_plus_underactuated_hand();
    let mut state = TorqueState::new(cfg).unwrap();

    let events: [Event; 23] = [
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
            attempt_id: 1,
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
            0,
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
        Event::ParkAccepted {
            group: 0,
            park_id: 2,
            now_us: 68_000,
        },
        Event::ParkSettled {
            group: 0,
            park_id: 2,
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
            let _ = std::hint::black_box(is_disable_authorized(&s, 0, 1, s.now_us));
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
        "step() / is_disable_authorized() / check_invariants() MUST perform zero heap ops, got {delta:?}"
    );
}
