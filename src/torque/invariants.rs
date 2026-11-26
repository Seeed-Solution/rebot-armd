//! Independent machine-checked invariants for every transition `(prev, event) -> (state, actions)`
//! of the [`crate::torque`] reducer (M15: checks rest-gate snapshots, capability correlation,
//! thermal coverage, and safety boundaries directly without reusing reducer predicates).

use serde::{Deserialize, Serialize};

use crate::motor_mode::ControlMode;

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
    RequestedDisableIncompleteGroupCoverage,
    StationaryHoldMutatedMotionBoundary,
    InvalidOrFutureSampleAdvancedRestWindow,
    HandshakeAckTreatedAsPositionEvidence,
    DefiniteNonRestDidNotRefuseWouldDrop,
    ArmParkAuthorizedHandRelease,
    CrossGroupDisableBleed,
    // Dispatch invariants
    IntentCommittedOffWithoutCorrelatedSentOrAmbiguousWrite,
    FirstCorrelatedSentOrAmbiguousDidNotCommitOff,
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
    UnsolicitedParkSettledMutatedPolicy,
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

#[inline]
const fn fresh_at(ts_us: u64, now_us: u64, max_age_us: u64) -> bool {
    ts_us <= now_us && (now_us - ts_us) <= max_age_us
}

fn indep_device_mode_fenced(state: &TorqueState, devices: DeviceMask) -> bool {
    for d in devices.iter() {
        if d >= state.config.device_count {
            continue;
        }
        if state.devices[d as usize].mode_fence_active {
            return true;
        }
        if state.config.devices[d as usize].device_wide_mode_switch {
            for c in 0..state.config.channel_count {
                let ch = &state.config.channels[c as usize];
                if ch.device == d && !matches!(ch.role, JointRole::PassiveSensor) {
                    let rt = &state.joints[c as usize];
                    if rt.verified_mode != Some(rt.desired_mode) {
                        return true;
                    }
                }
            }
        }
    }
    false
}

/// Independent check of whether recipient `r` satisfies the spec's rest/release gate (M15).
fn indep_recipient_passes(
    state: &TorqueState,
    r: u8,
    explicit_release_authorized: bool,
    now_us: u64,
) -> bool {
    if r >= state.config.recipient_count {
        return false;
    }
    let rec = &state.config.recipients[r as usize];
    let dev = &state.devices[rec.device as usize];
    let win = &state.rest_windows[r as usize];

    if matches!(rec.role, JointRole::LoadBearing) && !rec.rest_evidence_qualified {
        return false;
    }
    if now_us < dev.poisoned_until_us {
        return false;
    }
    // LegacyJ7 gripper never gates rest-pose disable (matches live arm drivers' arm-joint-only rest gate).
    if matches!(rec.role, JointRole::Gripper(GripperReleasePolicy::LegacyJ7)) {
        return true;
    }
    if win.count != REST_WINDOW_SAMPLES {
        return false;
    }
    let [s0, s1, s2] = win.snapshots;
    if !(s0.release_id < s1.release_id && s1.release_id < s2.release_id) {
        return false;
    }
    if !(s0.min_ts_us <= s1.min_ts_us && s1.min_ts_us <= s2.min_ts_us) {
        return false;
    }
    if now_us < dev.poisoned_until_us || s0.min_ts_us < dev.poisoned_until_us {
        return false;
    }
    if s2.min_ts_us.saturating_sub(s0.max_ts_us) < REST_WINDOW_MIN_SPAN_US {
        return false;
    }

    for c in rec.channels.iter() {
        let ch_cfg = &state.config.channels[c as usize];
        let rt = &state.joints[c as usize];
        // All 3 snapshots must be fresh under this channel's capture age ceiling (B8).
        if !fresh_at(s0.min_ts_us, now_us, ch_cfg.max_capture_age_us)
            || !fresh_at(s2.max_ts_us, now_us, ch_cfg.max_capture_age_us)
        {
            return false;
        }
        if let Some(last_mw) = rt.last_motion_write_us {
            if s0.min_ts_us <= last_mw.saturating_add(REST_SETTLE_US) {
                return false;
            }
        }
        match ch_cfg.role {
            JointRole::LoadBearing => {
                let Some(pos) = rt.last_measured_pos_rad else {
                    return false;
                };
                if !pos.is_finite()
                    || (pos - ch_cfg.rest_position_rad).abs() > ch_cfg.rest_tolerance_rad
                {
                    return false;
                }
                if let (Some(mv), Some(v)) =
                    (ch_cfg.rest_max_velocity_rad_s, rt.last_measured_vel_rad_s)
                {
                    if !v.is_finite() || v.abs() > mv {
                        return false;
                    }
                }
            }
            JointRole::Gripper(GripperReleasePolicy::LegacyJ7) => {
                if !rt.last_measured_pos_rad.is_some_and(f32::is_finite) {
                    return false;
                }
            }
            JointRole::Gripper(GripperReleasePolicy::ExplicitHand)
            | JointRole::ActiveTool(ActiveToolSafeOutput::BoundedRetention { .. }) => {
                if !explicit_release_authorized
                    || !matches!(
                        rt.last_release_evidence,
                        ReleaseEvidence::SupportedObject | ReleaseEvidence::VerifiedEmptyGrasp
                    )
                {
                    return false;
                }
            }
            JointRole::ActiveTool(ActiveToolSafeOutput::StopOutput) => {
                if !rt.last_measured_pos_rad.is_some_and(f32::is_finite) {
                    return false;
                }
            }
            JointRole::PassiveSensor => return false,
        }
    }
    true
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
    let valid_devices = state.valid_devices();

    if !actions.refresh_requests.is_subset_of(valid_channels)
        || !actions.telemetry.unheld_mask.is_subset_of(valid_channels)
        || !actions
            .telemetry
            .reported_enabled_mask
            .is_subset_of(valid_channels)
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

    for c in 0..state.config.channel_count {
        let ci = c as usize;
        let ch_cfg = &state.config.channels[ci];
        let rt = &state.joints[ci];

        if matches!(event, Event::Release { .. }) {
            if let Some(pos) = rt.captured_position_rad {
                let fresh = rt
                    .last_sample_us
                    .is_some_and(|ts| fresh_at(ts, state.now_us, ch_cfg.max_capture_age_us));
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
            // M5: Position-captured holds must carry fresh finite positions; ActiveToolStop may be positionless.
            if hcmd.uses_position_capture
                && (!hcmd.position_rad.is_finite()
                    || !fresh_at(hcmd.last_sample_us, state.now_us, ch_cfg.max_capture_age_us))
            {
                return Err(Violation::new(
                    ViolationKind::HoldWithStaleOrNonFiniteCapture,
                    Some(ch_cfg.group),
                    Some(c),
                    None,
                ));
            }
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

    if matches!(event, Event::Release { .. }) {
        for g in 0..state.config.group_count {
            let gi = g as usize;
            let grp = state.groups[gi];
            let needs_holds = grp.torque_intent == TorqueIntent::On
                && (grp.drive_state.requires_holds_when_on()
                    || (grp.drive_state == DriveState::FaultReactionActive
                        && !grp.pending_park.is_some_and(|p| p.accepted)));
            if !needs_holds {
                continue;
            }
            let dev_fenced = indep_device_mode_fenced(state, state.config.groups[gi].devices);
            for c in state.config.groups[gi].channels.iter() {
                let ci = c as usize;
                let ch_cfg = state.config.channels[ci];
                if matches!(ch_cfg.role, JointRole::PassiveSensor) {
                    continue;
                }
                let rt = &state.joints[ci];
                let mode_ok = rt.verified_mode == Some(rt.desired_mode) && !dev_fenced;
                let active_tool_stop_required = match ch_cfg.role {
                    JointRole::ActiveTool(ActiveToolSafeOutput::StopOutput) => true,
                    JointRole::ActiveTool(ActiveToolSafeOutput::BoundedRetention {
                        max_retention_us,
                    }) => rt
                        .retention_started_us
                        .is_some_and(|st| state.now_us.saturating_sub(st) >= max_retention_us),
                    _ => false,
                };
                if rt.captured_position_rad.is_some() {
                    if actions.holds[ci].is_none() {
                        return Err(Violation::new(
                            ViolationKind::MissingHoldForCapturedJoint,
                            Some(g),
                            Some(c),
                            None,
                        ));
                    }
                    if !mode_ok && !actions.telemetry.unheld_mask.contains(c) {
                        return Err(Violation::new(
                            ViolationKind::UncapturedOrUnverifiedNotReportedUnheld,
                            Some(g),
                            Some(c),
                            None,
                        ));
                    }
                } else if active_tool_stop_required {
                    // M5: Qualified StopOutput or expired BoundedRetention emits positionless
                    // ActiveToolStop even without positional encoder capture.
                    if !matches!(
                        actions.holds[ci],
                        Some(HoldCommand {
                            hold_kind: HoldKind::ActiveToolStop,
                            ..
                        })
                    ) {
                        return Err(Violation::new(
                            ViolationKind::MissingHoldForCapturedJoint,
                            Some(g),
                            Some(c),
                            None,
                        ));
                    }
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
                // B5: Requested disable must cover the entire independently switchable group.
                if dcmd.recipients != state.config.groups[gi].recipients {
                    return Err(Violation::new(
                        ViolationKind::RequestedDisableIncompleteGroupCoverage,
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
                    for r in dcmd.recipients.iter() {
                        if !indep_recipient_passes(
                            state,
                            r,
                            tx.explicit_release_authorized,
                            state.now_us,
                        ) {
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

    if let Event::CausalSample(s) = event {
        if s.channel < state.config.channel_count {
            let ci = s.channel as usize;
            let ch_cfg = &state.config.channels[ci];
            if s.is_handshake_ack_only {
                // B2: Handshake-only ACK never touches position capture, sample time, rest window, OR arms enable!
                if state.joints[ci].captured_position_rad != prev.joints[ci].captured_position_rad
                    || state.joints[ci].last_sample_us != prev.joints[ci].last_sample_us
                    || state.joints[ci].last_measured_pos_rad
                        != prev.joints[ci].last_measured_pos_rad
                    || state.groups[ch_cfg.group as usize].drive_state
                        != prev.groups[ch_cfg.group as usize].drive_state
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
                    || s.poll_issued_us.is_some_and(|p| {
                        p < prev.devices[ch_cfg.device as usize].poisoned_until_us
                            || s.rx_timestamp_us < p
                    })
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
                    && fresh_at(s.rx_timestamp_us, state.now_us, ch_cfg.max_capture_age_us)
                    && prev.joints[ci]
                        .last_sample_us
                        .is_none_or(|p| s.rx_timestamp_us >= p)
                {
                    if let Some(prev_tx) = prev.groups[gi].disable_tx {
                        let feetech_ok = !matches!(ch_cfg.vendor, VendorKind::Feetech)
                            || s.poll_issued_us
                                .zip(s.correlated_after_write_done_us)
                                .is_some_and(|(p, wd)| {
                                    p >= wd
                                        && p >= prev.devices[ch_cfg.device as usize].poisoned_until_us
                                });
                        let settle_ts = if matches!(ch_cfg.vendor, VendorKind::Feetech) {
                            s.poll_issued_us.unwrap_or(0)
                        } else {
                            s.rx_timestamp_us
                        };
                        let post_settle = prev.joints[ci]
                            .last_motion_write_us
                            .is_none_or(|w| settle_ts > w.saturating_add(REST_SETTLE_US));
                        let thermal_tripped_now = s
                            .temperature_c
                            .is_some_and(|t| t.is_finite() && t >= ch_cfg.thermal.trip_c);
                        if matches!(prev_tx.phase, DisablePhase::AwaitRestGate)
                            && !prev_tx.origin.is_protective()
                            && prev_tx.recipient_mask.contains(ch_cfg.recipient)
                            && state.now_us < prev_tx.deadline_us
                            && feetech_ok
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

    if let Event::ParkSettled { group, park_id, .. } = event {
        // B3: Unsolicited or superseded ParkSettled MUST leave policy unchanged.
        if *group < state.config.group_count {
            let gi = *group as usize;
            let matched = prev.groups[gi]
                .pending_park
                .is_some_and(|p| p.park_id == *park_id);
            if !matched
                && (state.groups[gi].drive_state != prev.groups[gi].drive_state
                    || state.groups[gi].disable_tx != prev.groups[gi].disable_tx
                    || state.groups[gi].torque_intent != prev.groups[gi].torque_intent)
            {
                return Err(Violation::new(
                    ViolationKind::UnsolicitedParkSettledMutatedPolicy,
                    Some(*group),
                    None,
                    None,
                ));
            }
        }
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

fn is_valid_pre_tx_enable_cancel(
    prev: &TorqueState,
    event: &Event,
    actions: &ActionSet,
    group: u8,
) -> bool {
    let gi = group as usize;
    matches!(
        event,
        Event::EnableCancelledPreTx {
            group: ev_group,
            attempt_id,
            ..
        } if *ev_group == group
            && prev.groups[gi].drive_state == DriveState::ReadyToSwitchOn
            && prev.groups[gi].enable_tx.is_some_and(|etx| {
                etx.attempt_id == *attempt_id
                    && !etx.any_write_sent
                    && etx.confirmed_channels.is_empty()
            })
            && actions.disables[gi].is_none()
    )
}

fn check_dispatch_invariants(
    state: &TorqueState,
    prev: &TorqueState,
    event: &Event,
    actions: &ActionSet,
) -> Result<(), Violation> {
    // B1 + Zero-write invariant: Off can ONLY be committed by a correlated WriteResult for the
    // live pending_disable_dispatch with at least one remaining recipient Sent or Ambiguous
    // (or a verified pre-TX enable cancellation where zero enable writes were ever sent).
    for g in 0..state.config.group_count {
        let gi = g as usize;
        if prev.groups[gi].torque_intent == TorqueIntent::On
            && state.groups[gi].torque_intent == TorqueIntent::Off
        {
            let valid_commit = is_valid_pre_tx_enable_cancel(prev, event, actions, g)
                || match (event, prev.groups[gi].pending_disable_dispatch) {
                    (Event::WriteResult(wr), Some(pend)) => {
                        let expected_kind = match pend.kind {
                            DisableKind::Requested => WriteKind::DisableRequested,
                            DisableKind::Protective => WriteKind::DisableProtective,
                        };
                        wr.group == g
                            && wr.kind == expected_kind
                            && wr.dispatch_id == pend.dispatch_id
                            && wr.generation == pend.generation
                            && wr
                                .target_mask
                                .intersect(pend.remaining_recipients)
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
                    ViolationKind::IntentCommittedOffWithoutCorrelatedSentOrAmbiguousWrite,
                    Some(g),
                    None,
                    None,
                ));
            }
        }
    }

    if let Event::WriteResult(wr) = event {
        if wr.group < state.config.group_count
            && matches!(
                wr.kind,
                WriteKind::DisableRequested | WriteKind::DisableProtective
            )
        {
            let gi = wr.group as usize;
            if let Some(pend) = prev.groups[gi].pending_disable_dispatch {
                let expected_kind = match pend.kind {
                    DisableKind::Requested => WriteKind::DisableRequested,
                    DisableKind::Protective => WriteKind::DisableProtective,
                };
                let correlated = wr.dispatch_id == pend.dispatch_id
                    && wr.kind == expected_kind
                    && wr.generation == pend.generation;
                let any_sent_or_ambiguous = wr
                    .target_mask
                    .intersect(pend.remaining_recipients)
                    .iter()
                    .any(|r| {
                        matches!(
                            wr.outcomes[r as usize],
                            MotorWriteOutcome::Sent | MotorWriteOutcome::Ambiguous
                        )
                    });
                if correlated
                    && any_sent_or_ambiguous
                    && state.groups[gi].torque_intent != TorqueIntent::Off
                {
                    return Err(Violation::new(
                        ViolationKind::FirstCorrelatedSentOrAmbiguousDidNotCommitOff,
                        Some(wr.group),
                        None,
                        None,
                    ));
                }
            }
        }
    }

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

    // B1: Motion and enable are fenced while EITHER disable_tx OR pending_disable_dispatch exists.
    match event {
        Event::EnableStart { group, .. } if *group < state.config.group_count => {
            let gi = *group as usize;
            if (prev.groups[gi].disable_tx.is_some()
                || prev.groups[gi].pending_disable_dispatch.is_some())
                && actions.enables[gi].is_some()
            {
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
            if (prev.groups[gi].disable_tx.is_some()
                || prev.groups[gi].pending_disable_dispatch.is_some())
                && actions.motion_allowed[gi].is_some()
            {
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
    if let Event::EnableStart { group, .. } = event {
        if *group < state.config.group_count {
            let gi = *group as usize;
            let timeout_mismatch = prev.config.groups[gi].channels.iter().any(|c| {
                let ch = &prev.config.channels[c as usize];
                if matches!(ch.role, JointRole::PassiveSensor) {
                    return false;
                }
                ch.damiao_timeout_reg.is_some_and(|exp| {
                    let cur_sess = prev.devices[ch.device as usize].session_id;
                    prev.joints[c as usize].verified_timeout_reg != Some(exp)
                        || prev.joints[c as usize].verified_timeout_session != Some(cur_sess)
                })
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

    for g in 0..state.config.group_count {
        let gi = g as usize;
        let clean_pre_tx_cancel = state.groups[gi].drive_state == DriveState::SwitchOnDisabled
            && state.groups[gi].torque_intent == TorqueIntent::Off
            && is_valid_pre_tx_enable_cancel(prev, event, actions, g);
        if prev.groups[gi].drive_state == DriveState::ReadyToSwitchOn
            && state.groups[gi].drive_state != DriveState::ReadyToSwitchOn
            && state.groups[gi].drive_state != DriveState::OperationEnabled
            && !clean_pre_tx_cancel
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

    // B2: Ordinary CausalSample.reported_enabled feeds telemetry only — NEVER arms or flips intent!
    if let Event::CausalSample(s) = event {
        if s.channel < state.config.channel_count {
            let ch_cfg = &state.config.channels[s.channel as usize];
            let gi = ch_cfg.group as usize;
            let thermal_tripped_now = s
                .temperature_c
                .is_some_and(|t| t.is_finite() && t >= ch_cfg.thermal.trip_c);
            if s.reported_enabled == Some(false)
                && !thermal_tripped_now
                && prev.groups[gi].torque_intent == TorqueIntent::On
                && state.groups[gi].torque_intent != TorqueIntent::On
            {
                return Err(Violation::new(
                    ViolationKind::ReportedEnableMutatedTorquePolicy,
                    Some(ch_cfg.group),
                    Some(s.channel),
                    None,
                ));
            }
            if prev.groups[gi].drive_state == DriveState::ReadyToSwitchOn
                && state.groups[gi].drive_state == DriveState::OperationEnabled
            {
                return Err(Violation::new(
                    ViolationKind::ReportedEnableMutatedTorquePolicy,
                    Some(ch_cfg.group),
                    Some(s.channel),
                    None,
                ));
            }
        }
    }

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
    // B10: Thermal N=1 preemption applies whenever hot AND (intent On OR unresolved physical off recipients!).
    if matches!(event, Event::Release { .. }) {
        for g in 0..state.config.group_count {
            let gi = g as usize;
            let grp = state.groups[gi];
            let unresolved_off = grp.torque_intent == TorqueIntent::On
                || !state.config.groups[gi]
                    .recipients
                    .is_subset_of(grp.physically_sent_off_mask);
            if !grp.thermal_tripped_mask.is_empty() && unresolved_off {
                if let Some(trip_rel) = grp.thermal_trip_release_id {
                    if state.current_release_id >= trip_rel.saturating_add(1) {
                        let damping_active = grp.drive_state == DriveState::ProtectiveDamping;
                        let protective_scheduled = actions.disables[gi].is_some_and(|d| {
                            matches!(d.kind, DisableKind::Protective) && d.use_reserved_budget
                        });
                        let awaiting_protective_tx = grp.pending_disable_dispatch.is_some_and(|p| {
                            matches!(p.kind, DisableKind::Protective) && p.tx_still_active
                        });
                        let between_protective_repeats = grp.disable_tx.is_some_and(|tx| {
                            tx.origin.is_protective()
                                && matches!(tx.phase, DisablePhase::Dispatched)
                                && state.now_us < tx.next_repeat_at_us
                        });
                        let closed_this_release = actions.disable_completions[gi]
                            .is_some_and(|c| c.origin.is_protective());
                        if !damping_active
                            && !protective_scheduled
                            && !awaiting_protective_tx
                            && !between_protective_repeats
                            && !closed_this_release
                        {
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
                    DisableOrigin::ProtectiveThermal | DisableOrigin::ProtectiveProcessExit
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

    match event {
        Event::EnableStart { group, .. } if *group < state.config.group_count => {
            let gi = *group as usize;
            if indep_device_mode_fenced(prev, prev.config.groups[gi].devices)
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
            if indep_device_mode_fenced(prev, prev.config.groups[gi].devices)
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
