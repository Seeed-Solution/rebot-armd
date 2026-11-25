//! Machine-checked invariants for every transition `(prev, event) -> (state, actions)` of the
//! [`crate::torque`] reducer.
//!
//! Every safety property from `torque-reducer-spec.md` is verified in zero allocations by
//! [`check_invariants`].

use serde::{Deserialize, Serialize};

use crate::motor_mode::ControlMode;

use super::reducer::{
    device_wide_mode_fenced, group_active_channels, is_sample_fresh, recipient_passes_gate,
};
use super::types::*;

/// Categorized invariant violation reported by [`check_invariants`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ViolationKind {
    // Structural & monotonicity
    StopGenerationRegressed,
    ReleaseIdMismatch,
    InvalidActiveMask,
    ThermalDerateOutOfRange,
    // Hold invariants
    MissingHoldForCapturedJoint,
    HoldWithStaleOrNonFiniteCapture,
    UncapturedOrUnverifiedNotReportedUnheld,
    HoldEmittedWhileIntentOff,
    GripperOpenedOrRaisedForceWhileStopped,
    PassiveSensorCommanded,
    // Rest-gate invariants
    RequestedDisableFailedRestGate,
    RequestedDisableGenerationOrReleaseMismatch,
    StationaryHoldMutatedMotionBoundary,
    InvalidOrFutureSampleAdvancedRestWindow,
    HandshakeAckTreatedAsPositionEvidence,
    DefiniteNonRestDidNotRefuseWouldDrop,
    ArmParkAuthorizedHandRelease,
    CrossGroupDisableBleed,
    // Dispatch invariants
    IntentCommittedOffWithoutSentOrAmbiguousWrite,
    FirstSentOrAmbiguousDidNotCommitOff,
    DisableRepeatOutOfBoundsOrPastDeadline,
    MotionOrEnableNotFencedDuringDisable,
    PartialOrAmbiguousDispatchNotSurfaced,
    // Enable invariants
    EnableStartDidNotSetIntentOn,
    EnableAllowedWithTimeoutMismatch,
    EnableFailureDidNotEnterFailedHoldOrRolledBack,
    ReportedEnableMutatedTorquePolicy,
    FaultClearDispatchedDisableOrChangedIntent,
    // Thermal, damping & panic invariants
    ThermalDidNotScheduleProtectiveByNextRelease,
    DampCommandWithoutConfiguredVerifiedMitMode,
    OrdinaryBusLossGrantedProtectiveAuthority,
    SafeHoldEmittedDisallowedDisable,
    FailedParkDispatchedDisableOrDroppedHold,
    DeviceWideModeSwitchDidNotFenceGroup,
}

/// Concrete invariant violation record (stack-allocated, zero heap allocation).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Violation {
    pub kind: ViolationKind,
    pub group: Option<u8>,
    pub channel: Option<u8>,
    pub recipient: Option<u8>,
}

impl Violation {
    #[inline]
    const fn new(
        kind: ViolationKind,
        group: Option<u8>,
        channel: Option<u8>,
        recipient: Option<u8>,
    ) -> Self {
        Self {
            kind,
            group,
            channel,
            recipient,
        }
    }
}

/// Machine-check every specification invariant across one reducer step:
/// `(prev, event) -> (state, actions)`.
pub fn check_invariants(
    state: &TorqueState,
    prev: &TorqueState,
    event: &Event,
    actions: &ActionSet,
) -> Result<(), Violation> {
    check_structural_invariants(state, prev, event, actions)?;
    check_hold_invariants(state, prev, event, actions)?;
    check_rest_gate_invariants(state, prev, event, actions)?;
    check_dispatch_invariants(state, prev, event, actions)?;
    check_enable_invariants(state, prev, event, actions)?;
    check_thermal_and_panic_invariants(state, prev, event, actions)?;
    Ok(())
}

// =========================================================================
// 1. Structural & Monotonicity Invariants
// =========================================================================

fn check_structural_invariants(
    state: &TorqueState,
    prev: &TorqueState,
    event: &Event,
    actions: &ActionSet,
) -> Result<(), Violation> {
    // Release ID advances by 1 iff Event::Release.
    let expected_release = match event {
        Event::Release { .. } => prev.current_release_id.saturating_add(1),
        _ => prev.current_release_id,
    };
    if state.current_release_id != expected_release {
        return Err(Violation::new(
            ViolationKind::ReleaseIdMismatch,
            None,
            None,
            None,
        ));
    }

    let valid_channels = state.valid_channels();
    let valid_recipients = state.valid_recipients();
    let valid_groups = state.valid_groups();
    let valid_devices = state.valid_devices();

    if !actions.refresh_requests.is_subset_of(valid_channels)
        || !actions.telemetry.unheld_mask.is_subset_of(valid_channels)
        || !actions.telemetry.reported_enabled_mask.is_subset_of(valid_channels)
        || !actions.park_requests.intersect(valid_groups).eq(&actions.park_requests)
        || !actions
            .contention_escalations
            .intersect(valid_devices)
            .eq(&actions.contention_escalations)
    {
        return Err(Violation::new(
            ViolationKind::InvalidActiveMask,
            None,
            None,
            None,
        ));
    }

    for g in 0..state.config.group_count {
        let gi = g as usize;
        if state.groups[gi].stop_generation < prev.groups[gi].stop_generation {
            return Err(Violation::new(
                ViolationKind::StopGenerationRegressed,
                Some(g),
                None,
                None,
            ));
        }
        if let Some(dcmd) = actions.disables[gi] {
            if !dcmd.recipients.is_subset_of(valid_recipients)
                || !dcmd.recipients.is_subset_of(state.config.groups[gi].recipients)
            {
                return Err(Violation::new(
                    ViolationKind::InvalidActiveMask,
                    Some(g),
                    None,
                    None,
                ));
            }
        }
    }

    for c in 0..state.config.channel_count {
        let ci = c as usize;
        let scale = state.joints[ci].thermal_derate_scale;
        if !scale.is_finite() || !(0.0..=1.0).contains(&scale) {
            return Err(Violation::new(
                ViolationKind::ThermalDerateOutOfRange,
                Some(state.config.channels[ci].group),
                Some(c),
                None,
            ));
        }
        let ch_cfg = &state.config.channels[ci];
        let tripped = state.groups[ch_cfg.group as usize]
            .thermal_tripped_mask
            .contains(c);
        if ch_cfg.thermal.derate_start_c.is_none() && !tripped && (scale - 1.0).abs() > 1e-6 {
            return Err(Violation::new(
                ViolationKind::ThermalDerateOutOfRange,
                Some(ch_cfg.group),
                Some(c),
                None,
            ));
        }
    }

    Ok(())
}

// =========================================================================
// 2. Hold Invariants
// =========================================================================

fn check_hold_invariants(
    state: &TorqueState,
    prev: &TorqueState,
    event: &Event,
    actions: &ActionSet,
) -> Result<(), Violation> {
    // Passive sensors are NEVER commanded (hold, damp, enable, disable).
    for c in 0..state.config.channel_count {
        let ci = c as usize;
        if matches!(state.config.channels[ci].role, JointRole::PassiveSensor)
            && (actions.holds[ci].is_some() || actions.damps[ci].is_some())
        {
            return Err(Violation::new(
                ViolationKind::PassiveSensorCommanded,
                Some(state.config.channels[ci].group),
                Some(c),
                None,
            ));
        }
    }
    for g in 0..state.config.group_count {
        let gi = g as usize;
        if let Some(ecmd) = actions.enables[gi] {
            for c in ecmd.channels.iter() {
                if c < state.config.channel_count
                    && matches!(state.config.channels[c as usize].role, JointRole::PassiveSensor)
                {
                    return Err(Violation::new(
                        ViolationKind::PassiveSensorCommanded,
                        Some(g),
                        Some(c),
                        None,
                    ));
                }
            }
        }
    }

    // Check every emitted hold command for validity, freshness, intent, and role safety.
    for c in 0..state.config.channel_count {
        let ci = c as usize;
        let ch_cfg = &state.config.channels[ci];
        let rt = &state.joints[ci];

        // Any active capture in state must be finite and <= max_capture_age_us on Release.
        if matches!(event, Event::Release { .. }) {
            if let Some(pos) = rt.captured_position_rad {
                let fresh = rt
                    .last_sample_us
                    .is_some_and(|ts| is_sample_fresh(ts, state.now_us, ch_cfg.max_capture_age_us));
                if !pos.is_finite() || !fresh {
                    return Err(Violation::new(
                        ViolationKind::HoldWithStaleOrNonFiniteCapture,
                        Some(ch_cfg.group),
                        Some(c),
                        None,
                    ));
                }
            }
        }

        if let Some(hcmd) = actions.holds[ci] {
            let gi = ch_cfg.group as usize;
            if state.groups[gi].torque_intent != TorqueIntent::On {
                return Err(Violation::new(
                    ViolationKind::HoldEmittedWhileIntentOff,
                    Some(ch_cfg.group),
                    Some(c),
                    None,
                ));
            }
            if !hcmd.position_rad.is_finite()
                || !is_sample_fresh(hcmd.last_sample_us, state.now_us, ch_cfg.max_capture_age_us)
            {
                return Err(Violation::new(
                    ViolationKind::HoldWithStaleOrNonFiniteCapture,
                    Some(ch_cfg.group),
                    Some(c),
                    None,
                ));
            }
            // Gripper / hand: stop and FailedHold NEVER open jaws and NEVER raise force.
            if matches!(ch_cfg.role, JointRole::Gripper(_)) {
                match hcmd.hold_kind {
                    HoldKind::GripperFreeze {
                        max_force_limit_raw,
                    } => {
                        if let (Some(prev_f), Some(new_f)) =
                            (prev.joints[ci].grasp_force_limit_raw, max_force_limit_raw)
                        {
                            if new_f > prev_f {
                                return Err(Violation::new(
                                    ViolationKind::GripperOpenedOrRaisedForceWhileStopped,
                                    Some(ch_cfg.group),
                                    Some(c),
                                    None,
                                ));
                            }
                        }
                    }
                    _ => {
                        return Err(Violation::new(
                            ViolationKind::GripperOpenedOrRaisedForceWhileStopped,
                            Some(ch_cfg.group),
                            Some(c),
                            None,
                        ));
                    }
                }
            }
        }
    }

    // Grasp force cap must never increase when the group is stopped or in FailedHold.
    for c in 0..state.config.channel_count {
        let ci = c as usize;
        if matches!(state.config.channels[ci].role, JointRole::Gripper(_)) {
            let g = state.config.channels[ci].group;
            let ds = state.groups[g as usize].drive_state;
            if ds.requires_holds_when_on() {
                if let (Some(pf), Some(nf)) = (
                    prev.joints[ci].grasp_force_limit_raw,
                    state.joints[ci].grasp_force_limit_raw,
                ) {
                    if nf > pf {
                        return Err(Violation::new(
                            ViolationKind::GripperOpenedOrRaisedForceWhileStopped,
                            Some(g),
                            Some(c),
                            None,
                        ));
                    }
                }
            }
        }
    }

    // On Release: Intent On (stopped or FailedHold or SafeHold) => every captured joint gets a hold
    // intent every release, whatever the reported enable state; uncaptured/unverified are reported unheld.
    if matches!(event, Event::Release { .. }) {
        for g in 0..state.config.group_count {
            let gi = g as usize;
            let grp = state.groups[gi];
            let needs_holds = grp.torque_intent == TorqueIntent::On
                && (grp.drive_state.requires_holds_when_on()
                    || (grp.drive_state == DriveState::FaultReactionActive && !grp.park_in_flight));
            if !needs_holds {
                continue;
            }
            let active = group_active_channels(state, g);
            let dev_fenced = device_wide_mode_fenced(state, state.config.groups[gi].devices);
            for c in active.iter() {
                let ci = c as usize;
                let rt = &state.joints[ci];
                if rt.captured_position_rad.is_some() {
                    if actions.holds[ci].is_none() {
                        return Err(Violation::new(
                            ViolationKind::MissingHoldForCapturedJoint,
                            Some(g),
                            Some(c),
                            None,
                        ));
                    }
                    let mode_ok = rt.verified_mode == Some(rt.desired_mode) && !dev_fenced;
                    if !mode_ok && !actions.telemetry.unheld_mask.contains(c) {
                        return Err(Violation::new(
                            ViolationKind::UncapturedOrUnverifiedNotReportedUnheld,
                            Some(g),
                            Some(c),
                            None,
                        ));
                    }
                } else if !actions.telemetry.unheld_mask.contains(c) {
                    return Err(Violation::new(
                        ViolationKind::UncapturedOrUnverifiedNotReportedUnheld,
                        Some(g),
                        Some(c),
                        None,
                    ));
                }
            }
        }
    }

    Ok(())
}

// =========================================================================
// 3. Rest-Gate & Causality Invariants
// =========================================================================

fn check_rest_gate_invariants(
    state: &TorqueState,
    prev: &TorqueState,
    event: &Event,
    actions: &ActionSet,
) -> Result<(), Violation> {
    // Any initial Requested disable (repeat_index == 0) must pass every actual recipient in the
    // current release and match the current stop_generation and release_id.
    for g in 0..state.config.group_count {
        let gi = g as usize;
        if let Some(dcmd) = actions.disables[gi] {
            if matches!(dcmd.kind, DisableKind::Requested) {
                if dcmd.generation != state.groups[gi].stop_generation
                    || dcmd.release_id != state.current_release_id
                {
                    return Err(Violation::new(
                        ViolationKind::RequestedDisableGenerationOrReleaseMismatch,
                        Some(g),
                        None,
                        None,
                    ));
                }
                if dcmd.repeat_index == 0 {
                    let Some(tx) = state.groups[gi].disable_tx else {
                        return Err(Violation::new(
                            ViolationKind::RequestedDisableFailedRestGate,
                            Some(g),
                            None,
                            None,
                        ));
                    };
                    if dcmd.recipients.is_empty() {
                        return Err(Violation::new(
                            ViolationKind::RequestedDisableFailedRestGate,
                            Some(g),
                            None,
                            None,
                        ));
                    }
                    for r in dcmd.recipients.iter() {
                        if !recipient_passes_gate(state, r, &tx, state.now_us) {
                            return Err(Violation::new(
                                ViolationKind::RequestedDisableFailedRestGate,
                                Some(g),
                                None,
                                Some(r),
                            ));
                        }
                    }
                }
            }
        }
    }

    // Stationary hold preserves last_motion_write_us boundary.
    if let Event::WriteResult(wr) = event {
        if wr.kind == WriteKind::Hold {
            for c in 0..state.config.channel_count {
                let ci = c as usize;
                if state.joints[ci].last_motion_write_us != prev.joints[ci].last_motion_write_us {
                    return Err(Violation::new(
                        ViolationKind::StationaryHoldMutatedMotionBoundary,
                        Some(state.config.channels[ci].group),
                        Some(c),
                        None,
                    ));
                }
            }
        }
    }

    // CausalSample invariants: future timestamps, >1ms drift, non-finite, handshake ACK, definite non-rest.
    if let Event::CausalSample(s) = event {
        if s.channel < state.config.channel_count {
            let ci = s.channel as usize;
            let ch_cfg = &state.config.channels[ci];
            if s.is_handshake_ack_only {
                if state.joints[ci].captured_position_rad != prev.joints[ci].captured_position_rad
                    || state.joints[ci].last_sample_us != prev.joints[ci].last_sample_us
                {
                    return Err(Violation::new(
                        ViolationKind::HandshakeAckTreatedAsPositionEvidence,
                        Some(ch_cfg.group),
                        Some(s.channel),
                        None,
                    ));
                }
                if !matches!(ch_cfg.role, JointRole::PassiveSensor)
                    && ch_cfg.recipient < state.config.recipient_count
                    && state.rest_windows[ch_cfg.recipient as usize]
                        != prev.rest_windows[ch_cfg.recipient as usize]
                {
                    return Err(Violation::new(
                        ViolationKind::HandshakeAckTreatedAsPositionEvidence,
                        Some(ch_cfg.group),
                        Some(s.channel),
                        Some(ch_cfg.recipient),
                    ));
                }
            } else if !matches!(ch_cfg.role, JointRole::PassiveSensor) {
                let rec = ch_cfg.recipient as usize;
                let invalid_sample = s.rx_timestamp_us > s.now_us
                    || s.rx_timestamp_us > state.now_us
                    || s.clock_offset_drift_us > CLOCK_OFFSET_MAX_DRIFT_US
                    || !s.position_rad.is_finite()
                    || s.velocity_rad_s.is_some_and(|v| !v.is_finite())
                    || s.rx_timestamp_us < prev.devices[ch_cfg.device as usize].poisoned_until_us
                    || s.correlated_after_write_done_us
                        .is_some_and(|wd| s.rx_timestamp_us < wd);
                if invalid_sample && state.rest_windows[rec].count != 0 {
                    return Err(Violation::new(
                        ViolationKind::InvalidOrFutureSampleAdvancedRestWindow,
                        Some(ch_cfg.group),
                        Some(s.channel),
                        Some(ch_cfg.recipient),
                    ));
                }

                // Definite non-rest pose during AwaitRestGate -> TORQUE_OFF_WOULD_DROP, 0 disables.
                let gi = ch_cfg.group as usize;
                if !invalid_sample
                    && is_sample_fresh(s.rx_timestamp_us, state.now_us, ch_cfg.max_capture_age_us)
                    && prev.joints[ci]
                        .last_sample_us
                        .is_none_or(|p| s.rx_timestamp_us >= p)
                {
                    if let Some(prev_tx) = prev.groups[gi].disable_tx {
                        let post_settle = prev.joints[ci]
                            .last_motion_write_us
                            .is_none_or(|w| s.rx_timestamp_us > w.saturating_add(REST_SETTLE_US));
                        let thermal_tripped_now = s
                            .temperature_c
                            .is_some_and(|t| t.is_finite() && t >= ch_cfg.thermal.trip_c);
                        if matches!(prev_tx.phase, DisablePhase::AwaitRestGate)
                            && !prev_tx.origin.is_protective()
                            && prev_tx.recipient_mask.contains(ch_cfg.recipient)
                            && state.now_us < prev_tx.deadline_us
                            && post_settle
                            && !thermal_tripped_now
                            && matches!(ch_cfg.role, JointRole::LoadBearing)
                            && state.config.recipients[rec].rest_evidence_qualified
                        {
                            let pos_bad = (s.position_rad - ch_cfg.rest_position_rad).abs()
                                > ch_cfg.rest_tolerance_rad;
                            let vel_bad = match (ch_cfg.rest_max_velocity_rad_s, s.velocity_rad_s) {
                                (Some(mv), Some(v)) => v.abs() > mv,
                                _ => false,
                            };
                            if pos_bad || vel_bad {
                                let refused_drop = actions.refusals[gi]
                                    .is_some_and(|r| r.code == RefusalCode::TorqueOffWouldDrop);
                                if !refused_drop
                                    || actions.disables[gi].is_some()
                                    || state.groups[gi].torque_intent != TorqueIntent::On
                                {
                                    return Err(Violation::new(
                                        ViolationKind::DefiniteNonRestDidNotRefuseWouldDrop,
                                        Some(ch_cfg.group),
                                        Some(s.channel),
                                        Some(ch_cfg.recipient),
                                    ));
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // Arm ParkSettled MUST NEVER authorize or disable any other group (e.g., hand group).
    if let Event::ParkSettled { group, .. } = event {
        for g in 0..state.config.group_count {
            if g == *group {
                continue;
            }
            let gi = g as usize;
            if state.groups[gi].disable_tx != prev.groups[gi].disable_tx
                || state.groups[gi].torque_intent != prev.groups[gi].torque_intent
            {
                return Err(Violation::new(
                    ViolationKind::ArmParkAuthorizedHandRelease,
                    Some(g),
                    None,
                    None,
                ));
            }
        }
    }

    // Non-protective DisableRequest or non-protective disable WriteResult on group g1 must never
    // flip TorqueIntent of any other group g2 != g1.
    match event {
        Event::DisableRequest { group, .. } => {
            for g in 0..state.config.group_count {
                if g != *group
                    && state.groups[g as usize].torque_intent
                        != prev.groups[g as usize].torque_intent
                {
                    return Err(Violation::new(
                        ViolationKind::CrossGroupDisableBleed,
                        Some(g),
                        None,
                        None,
                    ));
                }
            }
        }
        Event::WriteResult(wr) if wr.kind == WriteKind::DisableRequested => {
            for g in 0..state.config.group_count {
                if g != wr.group
                    && state.groups[g as usize].torque_intent
                        != prev.groups[g as usize].torque_intent
                {
                    return Err(Violation::new(
                        ViolationKind::CrossGroupDisableBleed,
                        Some(g),
                        None,
                        None,
                    ));
                }
            }
        }
        _ => {}
    }

    Ok(())
}

// =========================================================================
// 4. Dispatch Invariants
// =========================================================================

fn check_dispatch_invariants(
    state: &TorqueState,
    prev: &TorqueState,
    event: &Event,
    actions: &ActionSet,
) -> Result<(), Violation> {
    // Zero writes => intent stays On (Off is committed ONLY by a disable WriteResult with Sent or Ambiguous).
    for g in 0..state.config.group_count {
        let gi = g as usize;
        if prev.groups[gi].torque_intent == TorqueIntent::On
            && state.groups[gi].torque_intent == TorqueIntent::Off
        {
            let valid_commit = match event {
                Event::WriteResult(wr)
                    if wr.group == g
                        && matches!(
                            wr.kind,
                            WriteKind::DisableRequested | WriteKind::DisableProtective
                        ) =>
                {
                    wr.target_mask
                        .intersect(state.config.groups[gi].recipients)
                        .iter()
                        .any(|r| {
                            matches!(
                                wr.outcomes[r as usize],
                                MotorWriteOutcome::Sent | MotorWriteOutcome::Ambiguous
                            )
                        })
                }
                _ => false,
            };
            if !valid_commit {
                return Err(Violation::new(
                    ViolationKind::IntentCommittedOffWithoutSentOrAmbiguousWrite,
                    Some(g),
                    None,
                    None,
                ));
            }
        }
    }

    // First Sent or Ambiguous disable write matching active/authorized disable MUST commit Off immediately.
    if let Event::WriteResult(wr) = event {
        if wr.group < state.config.group_count
            && matches!(
                wr.kind,
                WriteKind::DisableRequested | WriteKind::DisableProtective
            )
        {
            let gi = wr.group as usize;
            let matched = prev.groups[gi].disable_tx.is_some()
                || prev.groups[gi].last_authorized_disable_gen == Some(wr.generation);
            let any_sent_or_ambiguous = wr
                .target_mask
                .intersect(state.config.groups[gi].recipients)
                .iter()
                .any(|r| {
                    matches!(
                        wr.outcomes[r as usize],
                        MotorWriteOutcome::Sent | MotorWriteOutcome::Ambiguous
                    )
                });
            if matched
                && any_sent_or_ambiguous
                && state.groups[gi].torque_intent != TorqueIntent::Off
            {
                return Err(Violation::new(
                    ViolationKind::FirstSentOrAmbiguousDidNotCommitOff,
                    Some(wr.group),
                    None,
                    None,
                ));
            }
        }
    }

    // Bounded repeats inside original deadline.
    for g in 0..state.config.group_count {
        let gi = g as usize;
        let grp_cfg = state.config.groups[gi];
        if let Some(dcmd) = actions.disables[gi] {
            if dcmd.repeat_index >= grp_cfg.disable_repeats {
                return Err(Violation::new(
                    ViolationKind::DisableRepeatOutOfBoundsOrPastDeadline,
                    Some(g),
                    None,
                    None,
                ));
            }
            if dcmd.repeat_index > 0 && state.now_us >= dcmd.deadline_us {
                return Err(Violation::new(
                    ViolationKind::DisableRepeatOutOfBoundsOrPastDeadline,
                    Some(g),
                    None,
                    None,
                ));
            }
        }
    }

    // Motion and enable are fenced while a disable transaction is in flight.
    match event {
        Event::EnableStart { group, .. } if *group < state.config.group_count => {
            let gi = *group as usize;
            if prev.groups[gi].disable_tx.is_some() && actions.enables[gi].is_some() {
                return Err(Violation::new(
                    ViolationKind::MotionOrEnableNotFencedDuringDisable,
                    Some(*group),
                    None,
                    None,
                ));
            }
        }
        Event::MotionCommand { group, .. } if *group < state.config.group_count => {
            let gi = *group as usize;
            if prev.groups[gi].disable_tx.is_some() && actions.motion_allowed[gi].is_some() {
                return Err(Violation::new(
                    ViolationKind::MotionOrEnableNotFencedDuringDisable,
                    Some(*group),
                    None,
                    None,
                ));
            }
        }
        _ => {}
    }

    // Partial or ambiguous dispatch MUST be surfaced explicitly on completion.
    for g in 0..state.config.group_count {
        let gi = g as usize;
        if let Some(comp) = actions.disable_completions[gi] {
            let not_all_clean = comp.sent_mask != comp.recipient_mask
                || !comp.ambiguous_mask.is_empty()
                || !comp.failed_mask.is_empty()
                || !comp.unsent_mask.is_empty();
            if not_all_clean && !comp.partial_or_ambiguous {
                return Err(Violation::new(
                    ViolationKind::PartialOrAmbiguousDispatchNotSurfaced,
                    Some(g),
                    None,
                    None,
                ));
            }
        }
    }

    Ok(())
}

// =========================================================================
// 5. Enable & FaultClear Invariants
// =========================================================================

fn check_enable_invariants(
    state: &TorqueState,
    prev: &TorqueState,
    event: &Event,
    actions: &ActionSet,
) -> Result<(), Violation> {
    // EnableStart accepted => TorqueIntent::On and ReadyToSwitchOn; TIMEOUT mismatch => refused.
    if let Event::EnableStart { group, .. } = event {
        if *group < state.config.group_count {
            let gi = *group as usize;
            let active = group_active_channels(prev, *group);
            let timeout_mismatch = active.iter().any(|c| {
                prev.config.channels[c as usize]
                    .damiao_timeout_reg
                    .is_some_and(|exp| prev.joints[c as usize].verified_timeout_reg != Some(exp))
            });
            if timeout_mismatch && actions.enables[gi].is_some() {
                return Err(Violation::new(
                    ViolationKind::EnableAllowedWithTimeoutMismatch,
                    Some(*group),
                    None,
                    None,
                ));
            }
            if actions.enables[gi].is_some()
                && (state.groups[gi].torque_intent != TorqueIntent::On
                    || state.groups[gi].drive_state != DriveState::ReadyToSwitchOn)
            {
                return Err(Violation::new(
                    ViolationKind::EnableStartDidNotSetIntentOn,
                    Some(*group),
                    None,
                    None,
                ));
            }
        }
    }

    // Failed, interrupted, or expired enable => FailedHold + ENABLE_FAILED, NO rollback
    // (unless preempted by protective thermal/process-exit or Tier-2 SafeHold).
    for g in 0..state.config.group_count {
        let gi = g as usize;
        if prev.groups[gi].drive_state == DriveState::ReadyToSwitchOn
            && state.groups[gi].drive_state != DriveState::ReadyToSwitchOn
            && state.groups[gi].drive_state != DriveState::OperationEnabled
            && !state.groups[gi].drive_state.is_protective()
            && state.groups[gi].drive_state != DriveState::SafeHold
            && (state.groups[gi].drive_state != DriveState::FailedHold
                || state.groups[gi].torque_intent != TorqueIntent::On
                || actions.disables[gi].is_some()
                || !state.groups[gi].faults.contains(TorqueFaultKind::EnableFailed))
        {
            return Err(Violation::new(
                ViolationKind::EnableFailureDidNotEnterFailedHoldOrRolledBack,
                Some(g),
                None,
                None,
            ));
        }
    }

    // Reported enable state feeds telemetry and handshake completion only.
    if let Event::CausalSample(s) = event {
        if s.channel < state.config.channel_count && s.reported_enabled == Some(false) {
            let g = state.config.channels[s.channel as usize].group;
            let gi = g as usize;
            let thermal_tripped_now = s.temperature_c.is_some_and(|t| {
                t.is_finite() && t >= state.config.channels[s.channel as usize].thermal.trip_c
            });
            if !thermal_tripped_now
                && prev.groups[gi].torque_intent == TorqueIntent::On
                && state.groups[gi].torque_intent != TorqueIntent::On
            {
                return Err(Violation::new(
                    ViolationKind::ReportedEnableMutatedTorquePolicy,
                    Some(g),
                    Some(s.channel),
                    None,
                ));
            }
        }
    }

    // FaultClear NEVER dispatches a disable and NEVER changes TorqueIntent.
    if matches!(event, Event::FaultClear { .. }) {
        if actions.any_disable() {
            return Err(Violation::new(
                ViolationKind::FaultClearDispatchedDisableOrChangedIntent,
                None,
                None,
                None,
            ));
        }
        for g in 0..state.config.group_count {
            let gi = g as usize;
            if state.groups[gi].torque_intent != prev.groups[gi].torque_intent {
                return Err(Violation::new(
                    ViolationKind::FaultClearDispatchedDisableOrChangedIntent,
                    Some(g),
                    None,
                    None,
                ));
            }
        }
    }

    Ok(())
}

// =========================================================================
// 6. Thermal, Damping, Panic & Mode-Fence Invariants
// =========================================================================

fn check_thermal_and_panic_invariants(
    state: &TorqueState,
    prev: &TorqueState,
    event: &Event,
    actions: &ActionSet,
) -> Result<(), Violation> {
    // Thermal preempts everything: protective disable scheduled by N=1 with reserved TX budget
    // (unless damping phase is actively running).
    if matches!(event, Event::Release { .. }) {
        for g in 0..state.config.group_count {
            let gi = g as usize;
            let grp = state.groups[gi];
            if !grp.thermal_tripped_mask.is_empty() && grp.torque_intent == TorqueIntent::On {
                if let Some(trip_rel) = grp.thermal_trip_release_id {
                    if state.current_release_id >= trip_rel.saturating_add(1) {
                        let damping_active = grp.drive_state == DriveState::ProtectiveDamping;
                        let protective_scheduled = actions.disables[gi].is_some_and(|d| {
                            matches!(d.kind, DisableKind::Protective) && d.use_reserved_budget
                        });
                        let already_awaiting_this_rel = grp.disable_tx.is_some_and(|tx| {
                            tx.origin.is_protective()
                                && tx.last_dispatched_release_id == Some(state.current_release_id)
                        });
                        if !damping_active && !protective_scheduled && !already_awaiting_this_rel {
                            return Err(Violation::new(
                                ViolationKind::ThermalDidNotScheduleProtectiveByNextRelease,
                                Some(g),
                                None,
                                None,
                            ));
                        }
                    }
                }
            }
        }
    }

    // MIT kd-only damping emitted ONLY when configured (duration > 0), supported, and verified Impedance.
    for c in 0..state.config.channel_count {
        let ci = c as usize;
        if let Some(damp) = actions.damps[ci] {
            let ch_cfg = &state.config.channels[ci];
            if ch_cfg.damping.duration_us == 0
                || !ch_cfg.supports_mit_damping
                || state.joints[ci].verified_mode != Some(ControlMode::Impedance)
                || damp.kp != 0.0
                || damp.kd != ch_cfg.damping.kd
                || damp.max_torque_nm != ch_cfg.damping.max_torque_nm
                || !damp.use_reserved_budget
            {
                return Err(Violation::new(
                    ViolationKind::DampCommandWithoutConfiguredVerifiedMitMode,
                    Some(ch_cfg.group),
                    Some(c),
                    None,
                ));
            }
        }
    }

    // Ordinary bus loss grants NO protective authority.
    if let Event::BusLoss { .. } = event {
        for g in 0..state.config.group_count {
            let gi = g as usize;
            if prev.groups[gi].drive_state != DriveState::SafeHold
                && (state.groups[gi].protective_latch != prev.groups[gi].protective_latch
                    || actions.disables[gi]
                        .is_some_and(|d| matches!(d.kind, DisableKind::Protective)))
            {
                return Err(Violation::new(
                    ViolationKind::OrdinaryBusLossGrantedProtectiveAuthority,
                    Some(g),
                    None,
                    None,
                ));
            }
        }
    }

    // SafeHold never emits DisableRequested, and only cuts on thermal or panic bus-loss limit.
    for g in 0..state.config.group_count {
        let gi = g as usize;
        if prev.groups[gi].drive_state == DriveState::SafeHold {
            if let Some(dcmd) = actions.disables[gi] {
                if matches!(dcmd.kind, DisableKind::Requested) {
                    return Err(Violation::new(
                        ViolationKind::SafeHoldEmittedDisallowedDisable,
                        Some(g),
                        None,
                        None,
                    ));
                }
                if !matches!(
                    dcmd.origin,
                    DisableOrigin::ProtectiveThermal
                        | DisableOrigin::ProtectivePanicBusLoss
                        | DisableOrigin::ProtectiveProcessExit
                ) {
                    return Err(Violation::new(
                        ViolationKind::SafeHoldEmittedDisallowedDisable,
                        Some(g),
                        None,
                        None,
                    ));
                }
            }
        }
    }

    // Failed park in Tier-1 worker panic must hold and never disable.
    if let Event::ParkSettled {
        group,
        succeeded: false,
        ..
    } = event
    {
        if *group < state.config.group_count {
            let gi = *group as usize;
            if prev.groups[gi].torque_intent == TorqueIntent::On
                && (state.groups[gi].torque_intent != TorqueIntent::On
                    || actions.disables[gi].is_some())
            {
                return Err(Violation::new(
                    ViolationKind::FailedParkDispatchedDisableOrDroppedHold,
                    Some(*group),
                    None,
                    None,
                ));
            }
        }
    }

    // Device-wide mode switch fences the whole group until verified.
    match event {
        Event::EnableStart { group, .. } if *group < state.config.group_count => {
            let gi = *group as usize;
            if device_wide_mode_fenced(prev, prev.config.groups[gi].devices)
                && actions.enables[gi].is_some()
            {
                return Err(Violation::new(
                    ViolationKind::DeviceWideModeSwitchDidNotFenceGroup,
                    Some(*group),
                    None,
                    None,
                ));
            }
        }
        Event::MotionCommand { group, .. } if *group < state.config.group_count => {
            let gi = *group as usize;
            if device_wide_mode_fenced(prev, prev.config.groups[gi].devices)
                && actions.motion_allowed[gi].is_some()
            {
                return Err(Violation::new(
                    ViolationKind::DeviceWideModeSwitchDidNotFenceGroup,
                    Some(*group),
                    None,
                    None,
                ));
            }
        }
        _ => {}
    }

    Ok(())
}
