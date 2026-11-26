//! Pure `TorqueController` policy reducer: `step(&mut TorqueState, Event) -> ActionSet`.
//!
//! Fixed-capacity arrays and bitsets, integer monotonic time (µs), zero heap allocation,
//! zero I/O, zero locks.
//!
//! Convergence primitives:
//! - Every wire disable dispatch, enable attempt, and park request carries a unique monotonic
//!   capability ID (`dispatch_id`, `attempt_id`, `park_id`) tracked in a single live pending slot.
//! - Every recipient rest gate evaluates a 3-slot sliding window of complete causal snapshots
//!   (`RestWindow`), requiring unused per-channel sample IDs, freshness of all 3 snapshots,
//!   post-motion settle, and vendor-specific poll/release causality.

use crate::motor_mode::ControlMode;

use super::types::*;

pub use super::invariants::{check_invariants, Violation, ViolationKind};

/// Execute one deterministic transition of the torque policy reducer.
///
/// Never allocates, performs no I/O, and takes no locks.
pub fn step(state: &mut TorqueState, event: Event) -> ActionSet {
    let mut actions = ActionSet::default();
    let ev_now = event_now_us(&event);
    let now_us = state.now_us.max(ev_now);
    state.now_us = now_us;

    match event {
        Event::Release { now_us: _ } => handle_release(state, now_us, &mut actions),
        Event::TransitionWake { now_us: _ } => handle_transition_wake(state, now_us, &mut actions),
        Event::CausalSample(sample) => handle_causal_sample(state, sample, now_us, &mut actions),
        Event::WriteResult(wr) => handle_write_result(state, wr, now_us, &mut actions),
        Event::HandshakeResult {
            group,
            attempt_id,
            now_us: _,
            confirmed_mask,
            outcome,
        } => handle_handshake_result(
            state,
            group,
            attempt_id,
            confirmed_mask,
            outcome,
            now_us,
            &mut actions,
        ),
        Event::EnableStart { group, now_us: _ } => {
            handle_enable_start(state, group, now_us, &mut actions)
        }
        Event::EnableCancelledPreTx {
            group,
            attempt_id,
            now_us: _,
        } => handle_enable_cancelled_pre_tx(state, group, attempt_id, now_us, &mut actions),
        Event::DisableRequest {
            group,
            now_us: _,
            explicit_release_authorized,
            recipient_mask_override,
        } => handle_disable_request(
            state,
            group,
            explicit_release_authorized,
            recipient_mask_override,
            now_us,
            &mut actions,
        ),
        Event::Stop {
            groups,
            kind,
            now_us: _,
        } => handle_stop(state, groups, kind, now_us, &mut actions),
        Event::FaultClear { groups, now_us: _ } => {
            handle_fault_clear(state, groups, now_us, &mut actions)
        }
        Event::Thermal {
            channel,
            temperature_c,
            now_us: _,
        } => apply_thermal(state, channel, temperature_c, now_us, now_us, &mut actions),
        Event::MitOwnershipChange {
            group,
            channels,
            target_mode,
            device_wide,
            now_us: _,
        } => handle_mit_ownership_change(
            state,
            group,
            channels,
            target_mode,
            device_wide,
            now_us,
            &mut actions,
        ),
        Event::ModeVerified {
            channel,
            mode,
            now_us: _,
        } => handle_mode_verified(state, channel, mode, now_us, &mut actions),
        Event::TimeoutReadback {
            channel,
            timeout_reg,
            now_us: _,
        } => handle_timeout_readback(state, channel, timeout_reg),
        Event::MotionCommand {
            group,
            channels,
            now_us: _,
        } => handle_motion_command(state, group, channels, &mut actions),
        Event::SerialTimeout { device, now_us: _ } => handle_serial_timeout(state, device, now_us),
        Event::GraspStateUpdate {
            channel,
            setpoint_rad,
            force_limit_raw,
            now_us: _,
        } => handle_grasp_state_update(state, channel, setpoint_rad, force_limit_raw),
        Event::ParkAccepted {
            group,
            park_id,
            now_us: _,
        } => handle_park_accepted(state, group, park_id),
        Event::ParkSettled {
            group,
            park_id,
            succeeded,
            now_us: _,
        } => handle_park_settled(state, group, park_id, succeeded, now_us),
        Event::Panic { tier, now_us: _ } => handle_panic(state, tier, now_us, &mut actions),
        Event::ProcessExit { now_us: _ } => handle_process_exit(state, now_us, &mut actions),
        Event::BusLoss { device, now_us: _ } => handle_bus_loss(state, device, now_us, &mut actions),
    }

    populate_telemetry(state, now_us, &mut actions);
    actions
}

/// Pure write-time recheck predicate for the capability-gated emitter (B9).
///
/// Verifies live dispatch identity, generation, release binding, deadline, and (for an initial
/// non-protective disable) that every physical recipient STILL passes its live causal gate.
pub fn is_disable_authorized(
    state: &TorqueState,
    group: u8,
    dispatch_id: u64,
    now_us: u64,
) -> bool {
    if group >= state.config.group_count {
        return false;
    }
    let gi = group as usize;
    let grp = &state.groups[gi];
    let Some(pend) = grp.pending_disable_dispatch else {
        return false;
    };
    if !pend.tx_still_active
        || pend.dispatch_id != dispatch_id
        || pend.generation != grp.stop_generation
        || pend.release_id != state.current_release_id
    {
        return false;
    }
    let Some(tx) = grp.disable_tx else {
        return false;
    };
    if tx.tx_id != pend.tx_id || now_us >= tx.deadline_us {
        return false;
    }
    match tx.phase {
        DisablePhase::AuthorizedForRelease {
            release_id,
            generation,
            dispatch_id: auth_id,
        } => {
            if release_id != state.current_release_id
                || generation != grp.stop_generation
                || auth_id != dispatch_id
            {
                return false;
            }
            for r in tx.recipient_mask.iter() {
                if !recipient_passes_gate(state, r, &tx, now_us) {
                    return false;
                }
            }
            true
        }
        DisablePhase::ProtectivePendingDispatch | DisablePhase::Dispatched => true,
        DisablePhase::AwaitRestGate | DisablePhase::ProtectiveDamping { .. } => false,
    }
}

/// Extract the monotonic timestamp carried by an [`Event`].
#[inline]
pub const fn event_now_us(event: &Event) -> u64 {
    match *event {
        Event::Release { now_us }
        | Event::TransitionWake { now_us }
        | Event::HandshakeResult { now_us, .. }
        | Event::EnableStart { now_us, .. }
        | Event::EnableCancelledPreTx { now_us, .. }
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
        | Event::BusLoss { now_us, .. } => now_us,
        Event::CausalSample(s) => s.now_us,
        Event::WriteResult(w) => w.now_us,
    }
}

/// Linear thermal derating multiplier in `0.0..=1.0`.
#[inline]
pub fn compute_thermal_derate_scale(temp_c: f32, thermal: ThermalConfig) -> f32 {
    if !temp_c.is_finite() {
        return 1.0;
    }
    if temp_c >= thermal.trip_c {
        return 0.0;
    }
    match thermal.derate_start_c {
        None => 1.0,
        Some(start_c) => {
            if temp_c <= start_c {
                1.0
            } else {
                let span = thermal.trip_c - start_c;
                if span <= 0.0 || !span.is_finite() {
                    0.0
                } else {
                    ((thermal.trip_c - temp_c) / span).clamp(0.0, 1.0)
                }
            }
        }
    }
}

#[inline]
pub(crate) const fn is_sample_fresh(sample_ts_us: u64, now_us: u64, max_age_us: u64) -> bool {
    sample_ts_us <= now_us && (now_us - sample_ts_us) <= max_age_us
}

/// Returns true if any device in `devices` has an active device-wide mode transition fence (B13)
/// or static `device_wide_mode_switch` with an unverified channel.
pub(crate) fn device_wide_mode_fenced(state: &TorqueState, devices: DeviceMask) -> bool {
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

/// Non-passive channels belonging to `group`.
#[inline]
pub(crate) fn group_active_channels(state: &TorqueState, group: u8) -> ChannelMask {
    let mut out = ChannelMask::EMPTY;
    if group >= state.config.group_count {
        return out;
    }
    for c in state.config.groups[group as usize].channels.iter() {
        if c < state.config.channel_count
            && !matches!(state.config.channels[c as usize].role, JointRole::PassiveSensor)
        {
            out.insert(c);
        }
    }
    out
}

/// Minimum `max_capture_age_us` across all logical channels mapped to physical recipient `r`.
pub(crate) fn recipient_min_capture_age_us(state: &TorqueState, r: u8) -> u64 {
    let rec = &state.config.recipients[r as usize];
    let mut min_age = u64::MAX;
    for c in rec.channels.iter() {
        if c < state.config.channel_count {
            min_age = min_age.min(state.config.channels[c as usize].max_capture_age_us);
        }
    }
    if min_age == u64::MAX {
        DEFAULT_CAPTURE_MAX_AGE_US
    } else {
        min_age
    }
}

/// Count distinct physical packets needed to address `recipients` (grouped packets counted once, M14).
pub(crate) fn count_distinct_packets(state: &TorqueState, recipients: ChannelMask) -> u16 {
    let mut remaining = recipients;
    let mut packets = 0u16;
    for r in recipients.iter() {
        if !remaining.contains(r) || r >= state.config.recipient_count {
            continue;
        }
        packets = packets.saturating_add(1);
        let pkt = state.config.recipients[r as usize].grouped_packet_mask;
        remaining = remaining.difference(pkt);
    }
    packets
}

/// Effective deadline capped by both `group_deadline_us` and the tightest addressed device deadline (M14).
fn effective_disable_deadline_us(
    state: &TorqueState,
    recipients: ChannelMask,
    now_us: u64,
    group_deadline_us: u64,
) -> u64 {
    let mut cap = group_deadline_us;
    for r in recipients.iter() {
        if r < state.config.recipient_count {
            let d = state.config.recipients[r as usize].device as usize;
            let dev_dl = now_us.saturating_add(state.config.devices[d].deadline_us);
            cap = cap.min(dev_dl);
        }
    }
    cap
}

/// True if a group still has torque intent On OR any recipient not yet confirmed `Sent` off (B10).
pub(crate) fn group_has_unresolved_off(state: &TorqueState, group: u8) -> bool {
    let gi = group as usize;
    state.groups[gi].torque_intent == TorqueIntent::On
        || !state.config.groups[gi]
            .recipients
            .is_subset_of(state.groups[gi].physically_sent_off_mask)
}

/// True if group `g` currently requires stopped-hold commands on a release (M1: includes unaccepted park).
#[inline]
pub(crate) fn group_requires_holds(grp: &GroupRuntime) -> bool {
    grp.torque_intent == TorqueIntent::On
        && (grp.drive_state.requires_holds_when_on()
            || (grp.drive_state == DriveState::FaultReactionActive
                && !grp.pending_park.is_some_and(|p| p.accepted)))
}

/// Reset recipient `r`'s sliding window AND per-channel window sample IDs (B8).
fn reset_recipient_window(state: &mut TorqueState, r: u8) {
    if r >= state.config.recipient_count {
        return;
    }
    state.rest_windows[r as usize].reset();
    let chs = state.config.recipients[r as usize].channels;
    for c in chs.iter() {
        if c < state.config.channel_count {
            state.joints[c as usize].last_window_sample_id = None;
            state.joints[c as usize].last_window_release_id = None;
        }
    }
}

/// Revoke an unconsumed `AuthorizedForRelease` authorization when qualifying evidence changes (B9).
fn revoke_unconsumed_rest_auth(state: &mut TorqueState, group: u8) {
    let gi = group as usize;
    let Some(tx) = state.groups[gi].disable_tx else {
        return;
    };
    if let DisablePhase::AuthorizedForRelease { dispatch_id, .. } = tx.phase {
        if !tx.committed_off {
            if let Some(tx_mut) = state.groups[gi].disable_tx.as_mut() {
                tx_mut.phase = DisablePhase::AwaitRestGate;
            }
            if let Some(pend) = state.groups[gi].pending_disable_dispatch.as_mut() {
                if pend.dispatch_id == dispatch_id {
                    pend.tx_still_active = false;
                }
            }
        }
    }
}

/// Drop stale captures (`> max_capture_age_us`) and prune/reset stale rest windows (B8).
fn refresh_captures_and_windows(state: &mut TorqueState, now_us: u64) {
    for c in 0..state.config.channel_count {
        let ch_cfg = state.config.channels[c as usize];
        if matches!(ch_cfg.role, JointRole::PassiveSensor) {
            continue;
        }
        let rt = &mut state.joints[c as usize];
        let fresh = rt
            .last_sample_us
            .is_some_and(|ts| is_sample_fresh(ts, now_us, ch_cfg.max_capture_age_us));
        if !fresh {
            rt.captured_position_rad = None;
            rt.captured_at_us = None;
        }
    }

    for r in 0..state.config.recipient_count {
        if state.rest_windows[r as usize].count == 0 {
            continue;
        }
        let min_age = recipient_min_capture_age_us(state, r);
        let latest_ts = state.rest_windows[r as usize].latest_ts_us();
        if !is_sample_fresh(latest_ts, now_us, min_age) {
            reset_recipient_window(state, r);
            let g = state.config.recipients[r as usize].group;
            revoke_unconsumed_rest_auth(state, g);
        }
    }
}

fn anchor_group_captures(state: &mut TorqueState, group: u8, now_us: u64) {
    let active = group_active_channels(state, group);
    for c in active.iter() {
        let ch_cfg = state.config.channels[c as usize];
        let rt = &mut state.joints[c as usize];
        let fresh = rt
            .last_sample_us
            .is_some_and(|ts| is_sample_fresh(ts, now_us, ch_cfg.max_capture_age_us));
        if fresh {
            if rt.captured_position_rad.is_none() {
                if let (Some(pos), Some(ts)) = (rt.last_measured_pos_rad, rt.last_sample_us) {
                    if pos.is_finite() {
                        rt.captured_position_rad = Some(pos);
                        rt.captured_at_us = Some(ts);
                        rt.had_valid_capture = true;
                    }
                }
            }
        } else {
            rt.captured_position_rad = None;
            rt.captured_at_us = None;
        }
        if matches!(
            ch_cfg.role,
            JointRole::ActiveTool(ActiveToolSafeOutput::BoundedRetention { .. })
        ) {
            rt.retention_started_us.get_or_insert(now_us);
        }
    }
}

fn capture_latest_for_stop(state: &mut TorqueState, group: u8, now_us: u64) {
    let active = group_active_channels(state, group);
    for c in active.iter() {
        let ch_cfg = state.config.channels[c as usize];
        let rt = &mut state.joints[c as usize];
        let fresh = rt
            .last_sample_us
            .is_some_and(|ts| is_sample_fresh(ts, now_us, ch_cfg.max_capture_age_us));
        if fresh {
            if let (Some(pos), Some(ts)) = (rt.last_measured_pos_rad, rt.last_sample_us) {
                if pos.is_finite() {
                    rt.captured_position_rad = Some(pos);
                    rt.captured_at_us = Some(ts);
                    rt.had_valid_capture = true;
                }
            }
        } else {
            rt.captured_position_rad = None;
            rt.captured_at_us = None;
        }
        if matches!(
            ch_cfg.role,
            JointRole::ActiveTool(ActiveToolSafeOutput::BoundedRetention { .. })
        ) {
            rt.retention_started_us.get_or_insert(now_us);
        }
    }
}

/// Expire an orphaned `pending_disable_dispatch` once its owning transaction is no longer active
/// and its effective wire deadline has elapsed (prevents permanent `DisableInProgress` wedging).
#[inline]
fn expire_orphaned_pending_disable_dispatch(state: &mut TorqueState, gi: usize, now_us: u64) {
    if let Some(pend) = state.groups[gi].pending_disable_dispatch {
        if !pend.tx_still_active && now_us >= pend.deadline_us {
            state.groups[gi].pending_disable_dispatch = None;
        }
    }
}

/// Sub-release transition-wake handler (Window 2 / headless transition polling): advances in-flight
/// disable repeats, rest-gate authorization, transaction deadlines, and orphaned dispatch expiry
/// without incrementing `current_release_id` or touching hold contention streaks.
fn handle_transition_wake(state: &mut TorqueState, now_us: u64, actions: &mut ActionSet) {
    refresh_captures_and_windows(state, now_us);

    for g in 0..state.config.group_count {
        let gi = g as usize;
        expire_orphaned_pending_disable_dispatch(state, gi, now_us);

        if state.groups[gi].drive_state == DriveState::ProtectiveDisabling {
            step_protective_disabling_progress(state, g, now_us, false, actions);
            expire_orphaned_pending_disable_dispatch(state, gi, now_us);
            continue;
        }
        if state.groups[gi].drive_state == DriveState::ProtectiveDamping
            || state.groups[gi].drive_state == DriveState::ProtectiveDisabled
        {
            continue;
        }

        if state.groups[gi].drive_state == DriveState::ReadyToSwitchOn {
            let expired = state.groups[gi]
                .enable_tx
                .is_some_and(|etx| now_us >= etx.deadline_us);
            if expired {
                enter_failed_hold(state, g, now_us, actions);
            }
        }

        if state.groups[gi]
            .disable_tx
            .is_some_and(|tx| !tx.origin.is_protective())
        {
            step_non_protective_disable_progress(state, g, now_us, false, actions);
            expire_orphaned_pending_disable_dispatch(state, gi, now_us);
        }
    }
}

// =========================================================================
// Event::Release
// =========================================================================

fn handle_release(state: &mut TorqueState, now_us: u64, actions: &mut ActionSet) {
    // Aggregate previous release's stopped-hold contention once per serviced release (M10).
    for d in 0..state.config.device_count {
        let di = d as usize;
        let contended = state.devices[di].hold_contended_in_release;
        let succeeded = state.devices[di].hold_succeeded_in_release;
        state.devices[di].hold_contended_in_release = false;
        state.devices[di].hold_succeeded_in_release = false;
        if contended && !succeeded {
            state.devices[di].consecutive_contention_skips = state.devices[di]
                .consecutive_contention_skips
                .saturating_add(1);
            if state.devices[di].consecutive_contention_skips >= CONTENTION_ESCALATION_RELEASES
                && !state.devices[di].contention_escalated
            {
                state.devices[di].contention_escalated = true;
                actions.contention_escalations.insert(d);
                for g in 0..state.config.group_count {
                    if state.config.groups[g as usize].devices.contains(d) {
                        latch_non_protective_fault(
                            state,
                            g,
                            TorqueFaultKind::DriverWriteFailed,
                            now_us,
                            actions,
                        );
                    }
                }
            }
        }
    }

    state.current_release_id = state.current_release_id.saturating_add(1);
    refresh_captures_and_windows(state, now_us);

    // Per-device SafeHold bus-loss evaluation (B11).
    let mut tripped_safe_hold_groups = GroupMask::EMPTY;
    for d in 0..state.config.device_count {
        let di = d as usize;
        let mut device_in_safe_hold = false;
        let mut any_stale_on_device = false;
        let mut touched_safe_groups = GroupMask::EMPTY;

        for c in 0..state.config.channel_count {
            let ch_cfg = state.config.channels[c as usize];
            if ch_cfg.device != d || matches!(ch_cfg.role, JointRole::PassiveSensor) {
                continue;
            }
            let grp = &state.groups[ch_cfg.group as usize];
            if grp.drive_state == DriveState::SafeHold
                && grp.torque_intent == TorqueIntent::On
                && grp.disable_tx.is_none()
            {
                device_in_safe_hold = true;
                touched_safe_groups.insert(ch_cfg.group);
                let rt = &state.joints[c as usize];
                let fresh = rt
                    .last_sample_us
                    .is_some_and(|ts| is_sample_fresh(ts, now_us, ch_cfg.max_capture_age_us));
                if !fresh {
                    any_stale_on_device = true;
                }
            }
        }

        if device_in_safe_hold {
            if any_stale_on_device {
                let since = *state.devices[di]
                    .safe_hold_bus_loss_since_us
                    .get_or_insert(now_us);
                if now_us.saturating_sub(since) >= state.config.devices[di].panic_bus_loss_limit_us
                {
                    tripped_safe_hold_groups = tripped_safe_hold_groups.union(touched_safe_groups);
                }
            } else {
                state.devices[di].safe_hold_bus_loss_since_us = None;
            }
        }
    }

    for g in 0..state.config.group_count {
        let gi = g as usize;
        expire_orphaned_pending_disable_dispatch(state, gi, now_us);

        // 1. Protective states preempt everything else on this group.
        if state.groups[gi].drive_state == DriveState::ProtectiveDamping {
            step_protective_damping_on_release(state, g, now_us, actions);
            continue;
        }
        if state.groups[gi].drive_state == DriveState::ProtectiveDisabling {
            step_protective_disabling_progress(state, g, now_us, true, actions);
            expire_orphaned_pending_disable_dispatch(state, gi, now_us);
            continue;
        }
        if state.groups[gi].drive_state == DriveState::ProtectiveDisabled {
            // B10: If thermal is still hot and any hot recipient is not confirmed Sent off, preempt.
            if !state.groups[gi].thermal_tripped_mask.is_empty()
                && group_has_unresolved_off(state, g)
            {
                trigger_protective_disable(
                    state,
                    g,
                    DisableOrigin::ProtectiveThermal,
                    now_us,
                    actions,
                );
            }
            continue;
        }

        // 2. Tier-2 SafeHold bus-loss trip (per-device timer crossed limit, B11):
        // DRV-F1 governing rule: protective disable is ONLY for Thermal and ProcessExit.
        // Bus loss alone latches a non-recoverable BusLoss fault and leaves holding unconfirmed
        // (SafeHold continues emitting holds / HeldStaleTarget; zero protective disable).
        if tripped_safe_hold_groups.contains(g)
            && state.groups[gi].drive_state == DriveState::SafeHold
            && state.groups[gi].torque_intent == TorqueIntent::On
            && state.groups[gi].disable_tx.is_none()
            && !state.groups[gi].faults.contains(TorqueFaultKind::BusLoss)
        {
            latch_non_protective_fault(state, g, TorqueFaultKind::BusLoss, now_us, actions);
        }

        // 3. Enable deadline expiry.
        if state.groups[gi].drive_state == DriveState::ReadyToSwitchOn {
            let expired = state.groups[gi]
                .enable_tx
                .is_some_and(|etx| now_us >= etx.deadline_us);
            if expired {
                enter_failed_hold(state, g, now_us, actions);
            }
        }

        // 4. Non-protective disable transaction progression.
        if state.groups[gi]
            .disable_tx
            .is_some_and(|tx| !tx.origin.is_protective())
        {
            step_non_protective_disable_progress(state, g, now_us, true, actions);
            expire_orphaned_pending_disable_dispatch(state, gi, now_us);
        }

        // 5. Hold emission for stopped / FailedHold / SafeHold / unaccepted-park groups with TorqueIntent::On.
        emit_group_holds(state, g, now_us, actions);
    }
}

fn step_protective_damping_on_release(
    state: &mut TorqueState,
    g: u8,
    now_us: u64,
    actions: &mut ActionSet,
) {
    let gi = g as usize;
    let Some(tx) = state.groups[gi].disable_tx else {
        if group_has_unresolved_off(state, g) {
            trigger_protective_disable(state, g, DisableOrigin::ProtectiveThermal, now_us, actions);
        }
        return;
    };

    let DisablePhase::ProtectiveDamping {
        started_us,
        dampable_mask,
        ..
    } = tx.phase
    else {
        return;
    };

    // M8: evaluate each channel against its OWN configured damping duration.
    let mut active_damp_mask = ChannelMask::EMPTY;
    for c in dampable_mask.iter() {
        let ch_cfg = state.config.channels[c as usize];
        let ch_end = started_us.saturating_add(ch_cfg.damping.duration_us);
        if ch_cfg.damping.duration_us > 0
            && ch_cfg.supports_mit_damping
            && state.joints[c as usize].verified_mode == Some(ControlMode::Impedance)
            && now_us < ch_end
        {
            active_damp_mask.insert(c);
        }
    }

    if active_damp_mask.is_empty() {
        // Every dampable channel reached its own end_us (or lost verified MIT mode): dispatch DisableProtective.
        state.groups[gi].drive_state = DriveState::ProtectiveDisabling;
        if let Some(tx_mut) = state.groups[gi].disable_tx.as_mut() {
            tx_mut.phase = DisablePhase::ProtectivePendingDispatch;
        }
        emit_disable_attempt(state, g, DisableKind::Protective, now_us, actions);
    } else {
        emit_damping_commands(state, g, active_damp_mask, started_us, actions);
    }
}

fn emit_damping_commands(
    state: &TorqueState,
    g: u8,
    dampable_mask: ChannelMask,
    started_us: u64,
    actions: &mut ActionSet,
) {
    let gen = state.groups[g as usize].stop_generation;
    for c in dampable_mask.iter() {
        let ch_cfg = state.config.channels[c as usize];
        let ch_end = started_us.saturating_add(ch_cfg.damping.duration_us);
        actions.damps[c as usize] = Some(DampCommand {
            channel: c,
            group: g,
            device: ch_cfg.device,
            kp: 0.0,
            kd: ch_cfg.damping.kd,
            max_torque_nm: ch_cfg.damping.max_torque_nm,
            end_us: ch_end,
            generation: gen,
            use_reserved_budget: true,
        });
    }
}

fn step_protective_disabling_progress(
    state: &mut TorqueState,
    g: u8,
    now_us: u64,
    is_release: bool,
    actions: &mut ActionSet,
) {
    let gi = g as usize;
    let grp_cfg = state.config.groups[gi];

    let Some(tx) = state.groups[gi].disable_tx else {
        // M13 / B12: If a previous protective attempt expired and thermal is STILL active, open a
        // fresh bounded attempt with a new tx_id and deadline.
        if is_release
            && !state.groups[gi].thermal_tripped_mask.is_empty()
            && group_has_unresolved_off(state, g)
        {
            trigger_protective_disable(state, g, DisableOrigin::ProtectiveThermal, now_us, actions);
        }
        return;
    };

    match tx.phase {
        DisablePhase::ProtectivePendingDispatch => {
            if now_us >= tx.deadline_us && !tx.committed_off {
                // Close expired zero-write attempt ONCE (B12, M13).
                close_protective_zero_write_expiry(state, g, now_us, actions);
            } else if let Some(pend) = state.groups[gi].pending_disable_dispatch {
                // If an initial protective dispatch from an earlier release received zero callbacks
                // (e.g. Thermal fired between releases and owner dispatches synchronously on Release),
                // re-surface the SAME dispatch_id on the N=1 serviced release without losing correlation!
                if is_release
                    && pend.tx_still_active
                    && pend.tx_id == tx.tx_id
                    && pend.remaining_recipients == pend.target_recipients
                    && pend.release_id < state.current_release_id
                {
                    let eff_dl = effective_disable_deadline_us(
                        state,
                        pend.target_recipients,
                        now_us,
                        tx.deadline_us,
                    );
                    if let Some(p_mut) = state.groups[gi].pending_disable_dispatch.as_mut() {
                        p_mut.release_id = state.current_release_id;
                        p_mut.deadline_us = eff_dl;
                    }
                    if let Some(tx_mut) = state.groups[gi].disable_tx.as_mut() {
                        tx_mut.last_dispatched_release_id = Some(state.current_release_id);
                    }
                    actions.disables[gi] = Some(DisableCommand {
                        group: g,
                        tx_id: tx.tx_id,
                        dispatch_id: pend.dispatch_id,
                        kind: DisableKind::Protective,
                        origin: tx.origin,
                        recipients: pend.target_recipients,
                        packet_count: count_distinct_packets(state, pend.target_recipients),
                        generation: pend.generation,
                        release_id: state.current_release_id,
                        deadline_us: eff_dl,
                        repeat_index: tx.repeats_completed,
                        use_reserved_budget: true,
                    });
                }
            } else if is_release {
                emit_disable_attempt(state, g, DisableKind::Protective, now_us, actions);
            }
        }
        DisablePhase::Dispatched => {
            if now_us >= tx.deadline_us {
                finish_disable_tx(state, g, true, actions);
            } else if tx.repeats_completed >= grp_cfg.disable_repeats {
                finish_disable_tx(state, g, false, actions);
            } else if now_us >= tx.next_repeat_at_us
                && state.groups[gi].pending_disable_dispatch.is_none()
            {
                emit_disable_attempt(state, g, DisableKind::Protective, now_us, actions);
            }
        }
        _ => {}
    }
}

/// Explicit zero-write protective expiry transition (B12, M13): closes the attempt exactly once;
/// resumes SafeHold if control panic is latched and thermal is clear; otherwise keeps protective latch.
fn close_protective_zero_write_expiry(
    state: &mut TorqueState,
    g: u8,
    now_us: u64,
    actions: &mut ActionSet,
) {
    let gi = g as usize;
    let Some(tx) = state.groups[gi].disable_tx.take() else {
        return;
    };
    if let Some(pend) = state.groups[gi].pending_disable_dispatch.as_mut() {
        if pend.tx_id == tx.tx_id {
            pend.tx_still_active = false;
        }
    }
    state.groups[gi].faults.insert(TorqueFaultKind::DriverWriteFailed);
    actions.disable_completions[gi] = Some(build_completion(g, &tx, true));

    // B12: If control panic is latched and thermal is NOT tripped, resume SafeHold so holds
    // resume immediately once feedback is fresh (or a fresh bounded bus-loss attempt starts).
    if state.control_panic_latched && state.groups[gi].thermal_tripped_mask.is_empty() {
        state.groups[gi].drive_state = DriveState::SafeHold;
        if state.groups[gi].torque_intent == TorqueIntent::On {
            anchor_group_captures(state, g, now_us);
            emit_group_holds(state, g, now_us, actions);
        }
    }
}

fn step_non_protective_disable_progress(
    state: &mut TorqueState,
    g: u8,
    now_us: u64,
    emit_refresh: bool,
    actions: &mut ActionSet,
) {
    let gi = g as usize;
    let grp_cfg = state.config.groups[gi];
    let Some(tx) = state.groups[gi].disable_tx else {
        return;
    };

    match tx.phase {
        DisablePhase::AuthorizedForRelease {
            release_id,
            generation,
            ..
        } => {
            if release_id < state.current_release_id {
                if generation != state.groups[gi].stop_generation {
                    abort_non_protective_tx(state, g, RefusalCode::PreemptedByStop, false, actions);
                } else if now_us >= tx.deadline_us {
                    let code = rest_timeout_code(state, &tx);
                    abort_non_protective_tx(state, g, code, true, actions);
                } else if state.groups[gi].pending_disable_dispatch.is_none() {
                    if let Some(tx_mut) = state.groups[gi].disable_tx.as_mut() {
                        tx_mut.phase = DisablePhase::AwaitRestGate;
                    }
                    try_authorize_rest_gate(state, g, now_us, actions);
                }
            } else if now_us >= tx.deadline_us {
                let code = rest_timeout_code(state, &tx);
                abort_non_protective_tx(state, g, code, true, actions);
            }
        }
        DisablePhase::AwaitRestGate => {
            if tx.auth_generation != state.groups[gi].stop_generation {
                abort_non_protective_tx(state, g, RefusalCode::PreemptedByStop, false, actions);
            } else if now_us >= tx.deadline_us {
                let code = rest_timeout_code(state, &tx);
                abort_non_protective_tx(state, g, code, true, actions);
            } else if state.groups[gi].pending_disable_dispatch.is_none() {
                try_authorize_rest_gate(state, g, now_us, actions);
                if emit_refresh {
                    if let Some(tx_now) = state.groups[gi].disable_tx {
                        if matches!(tx_now.phase, DisablePhase::AwaitRestGate) {
                            for r in tx_now.recipient_mask.iter() {
                                if !recipient_passes_gate(state, r, &tx_now, now_us) {
                                    actions.refresh_requests = actions
                                        .refresh_requests
                                        .union(state.config.recipients[r as usize].channels);
                                }
                            }
                        }
                    }
                }
            }
        }
        DisablePhase::Dispatched => {
            if now_us >= tx.deadline_us {
                finish_disable_tx(state, g, true, actions);
            } else if tx.repeats_completed >= grp_cfg.disable_repeats {
                finish_disable_tx(state, g, false, actions);
            } else if now_us >= tx.next_repeat_at_us
                && state.groups[gi].pending_disable_dispatch.is_none()
            {
                emit_disable_attempt(state, g, DisableKind::Requested, now_us, actions);
            }
        }
        _ => {}
    }
}

fn emit_disable_attempt(
    state: &mut TorqueState,
    g: u8,
    kind: DisableKind,
    now_us: u64,
    actions: &mut ActionSet,
) {
    let gi = g as usize;
    let Some(tx) = state.groups[gi].disable_tx else {
        return;
    };
    let dispatch_id = state.alloc_capability_id();
    let gen = state.groups[gi].stop_generation;
    let rel = state.current_release_id;
    let use_reserved = matches!(kind, DisableKind::Protective);
    let eff_dl = effective_disable_deadline_us(state, tx.recipient_mask, now_us, tx.deadline_us);

    if let Some(tx_mut) = state.groups[gi].disable_tx.as_mut() {
        tx_mut.last_dispatched_release_id = Some(rel);
        if matches!(kind, DisableKind::Requested) && !tx_mut.committed_off {
            tx_mut.phase = DisablePhase::AuthorizedForRelease {
                release_id: rel,
                generation: gen,
                dispatch_id,
            };
        }
    }
    state.groups[gi].pending_disable_dispatch = Some(PendingDisableDispatch {
        dispatch_id,
        tx_id: tx.tx_id,
        kind,
        origin: tx.origin,
        release_id: rel,
        generation: gen,
        target_recipients: tx.recipient_mask,
        remaining_recipients: tx.recipient_mask,
        sent_mask: ChannelMask::EMPTY,
        ambiguous_mask: ChannelMask::EMPTY,
        failed_mask: ChannelMask::EMPTY,
        deferred_mask: ChannelMask::EMPTY,
        rejected_stale_mask: ChannelMask::EMPTY,
        tx_still_active: true,
        deadline_us: eff_dl,
    });
    actions.disables[gi] = Some(DisableCommand {
        group: g,
        tx_id: tx.tx_id,
        dispatch_id,
        kind,
        origin: tx.origin,
        recipients: tx.recipient_mask,
        packet_count: count_distinct_packets(state, tx.recipient_mask),
        generation: gen,
        release_id: rel,
        deadline_us: eff_dl,
        repeat_index: tx.repeats_completed,
        use_reserved_budget: use_reserved,
    });
}

fn emit_group_holds(state: &mut TorqueState, g: u8, now_us: u64, actions: &mut ActionSet) {
    let gi = g as usize;
    if !group_requires_holds(&state.groups[gi]) {
        return;
    }

    let grp = state.groups[gi];
    let active = group_active_channels(state, g);
    let dev_wide_fenced = device_wide_mode_fenced(state, state.config.groups[gi].devices);
    for c in active.iter() {
        let ch_cfg = state.config.channels[c as usize];
        let rt = &mut state.joints[c as usize];
        let mode_ok = rt.verified_mode == Some(rt.desired_mode) && !dev_wide_fenced;

        // M5: ActiveTool safe outputs (StopOutput or expired BoundedRetention) emit their semantic
        // safe stop command even without positional encoder capture.
        if let JointRole::ActiveTool(safe_out) = ch_cfg.role {
            let hold_kind = match safe_out {
                ActiveToolSafeOutput::StopOutput => HoldKind::ActiveToolStop,
                ActiveToolSafeOutput::BoundedRetention { max_retention_us } => {
                    let started = *rt.retention_started_us.get_or_insert(now_us);
                    let elapsed = now_us.saturating_sub(started);
                    if elapsed < max_retention_us {
                        HoldKind::ActiveToolRetain {
                            remaining_us: max_retention_us - elapsed,
                        }
                    } else {
                        HoldKind::ActiveToolStop
                    }
                }
            };
            let (pos, cap_at, last_ts, uses_cap) = match (
                rt.captured_position_rad,
                rt.captured_at_us,
                rt.last_sample_us,
            ) {
                (Some(p), Some(ca), Some(ls)) => (p, ca, ls, true),
                _ => (0.0, now_us, now_us, false),
            };
            if matches!(hold_kind, HoldKind::ActiveToolStop) || uses_cap {
                actions.holds[c as usize] = Some(HoldCommand {
                    channel: c,
                    group: g,
                    device: ch_cfg.device,
                    position_rad: pos,
                    captured_at_us: cap_at,
                    last_sample_us: last_ts,
                    uses_position_capture: uses_cap,
                    verified_mode: if mode_ok { rt.verified_mode } else { None },
                    hold_kind,
                    torque_scale: rt.thermal_derate_scale,
                    generation: grp.stop_generation,
                });
                if !mode_ok {
                    actions.telemetry.unheld_mask.insert(c);
                    actions.refresh_requests.insert(c);
                }
            } else {
                actions.telemetry.unheld_mask.insert(c);
                actions.refresh_requests.insert(c);
            }
            continue;
        }

        if let (Some(pos), Some(cap_at), Some(last_ts)) = (
            rt.captured_position_rad,
            rt.captured_at_us,
            rt.last_sample_us,
        ) {
            let hold_kind = match ch_cfg.role {
                JointRole::LoadBearing => HoldKind::PositionHold,
                JointRole::Gripper(_) => HoldKind::GripperFreeze {
                    max_force_limit_raw: rt.grasp_force_limit_raw,
                },
                JointRole::ActiveTool(_) | JointRole::PassiveSensor => continue,
            };
            actions.holds[c as usize] = Some(HoldCommand {
                channel: c,
                group: g,
                device: ch_cfg.device,
                position_rad: pos,
                captured_at_us: cap_at,
                last_sample_us: last_ts,
                uses_position_capture: true,
                verified_mode: if mode_ok { rt.verified_mode } else { None },
                hold_kind,
                torque_scale: rt.thermal_derate_scale,
                generation: grp.stop_generation,
            });
            if !mode_ok {
                actions.telemetry.unheld_mask.insert(c);
                actions.refresh_requests.insert(c);
            }
        } else {
            actions.telemetry.unheld_mask.insert(c);
            actions.refresh_requests.insert(c);
        }
    }
}

/// Pure read-only projection of the hold channel mask that [`emit_group_holds`] produces for
/// `group` in the reducer's current state at `now_us`.
///
/// Used by shadow `LiveHoldCheckpoint` after Step-2 transition service and RX intake so hold-mask
/// comparison aligns with the live measured-hold selection point within the same release (rather
/// than comparing against the pre-transition/pre-RX `Release` snapshot from earlier in the tick).
pub fn predict_group_hold_mask(state: &TorqueState, group: u8, now_us: u64) -> ChannelMask {
    let gi = group as usize;
    if gi >= usize::from(state.config.group_count) || !group_requires_holds(&state.groups[gi]) {
        return ChannelMask::EMPTY;
    }
    let mut mask = ChannelMask::EMPTY;
    let active = group_active_channels(state, group);
    for c in active.iter() {
        let ch_cfg = state.config.channels[c as usize];
        let rt = &state.joints[c as usize];
        let uses_cap = rt.captured_position_rad.is_some()
            && rt.captured_at_us.is_some()
            && rt
                .last_sample_us
                .is_some_and(|ts| is_sample_fresh(ts, now_us, ch_cfg.max_capture_age_us));
        let held = match ch_cfg.role {
            JointRole::PassiveSensor => false,
            JointRole::ActiveTool(ActiveToolSafeOutput::StopOutput) => true,
            JointRole::ActiveTool(ActiveToolSafeOutput::BoundedRetention { max_retention_us }) => {
                let started = rt.retention_started_us.unwrap_or(now_us);
                let elapsed = now_us.saturating_sub(started);
                elapsed >= max_retention_us || uses_cap
            }
            JointRole::LoadBearing | JointRole::Gripper(_) => uses_cap,
        };
        if held {
            mask.insert(c);
        }
    }
    mask
}

// =========================================================================
// Event::CausalSample & Sliding-Window Rest Gate
// =========================================================================

fn handle_causal_sample(
    state: &mut TorqueState,
    sample: CausalSample,
    now_us: u64,
    actions: &mut ActionSet,
) {
    if sample.channel >= state.config.channel_count {
        return;
    }
    let ch = sample.channel as usize;
    let ch_cfg = state.config.channels[ch];
    let g = ch_cfg.group;
    let gi = g as usize;
    let d = ch_cfg.device as usize;

    // 1. Whole-hand / device ACK only: telemetry only; NEVER creates position/rest evidence
    //    and NEVER arms an enable attempt directly (B2: single attempt_id-bound HandshakeResult path).
    if sample.is_handshake_ack_only {
        if let Some(en) = sample.reported_enabled {
            state.joints[ch].reported_enabled = Some(en);
        }
        return;
    }

    // 2. Passive sensor: observations only.
    if matches!(ch_cfg.role, JointRole::PassiveSensor) {
        if sample.rx_timestamp_us <= sample.now_us
            && sample.rx_timestamp_us <= now_us
            && sample.position_rad.is_finite()
        {
            state.joints[ch].last_sample_id = Some(sample.sample_id);
            state.joints[ch].last_sample_us = Some(sample.rx_timestamp_us);
            state.joints[ch].last_sample_release_id = Some(state.current_release_id);
            state.joints[ch].last_measured_pos_rad = Some(sample.position_rad);
            if sample.velocity_rad_s.is_some_and(f32::is_finite) {
                state.joints[ch].last_measured_vel_rad_s = sample.velocity_rad_s;
            }
        }
        return;
    }

    let rec = ch_cfg.recipient;

    // 3. Future-dated timestamps are rejected and never clamped; invalid evidence resets window + revokes auth (B9).
    if sample.rx_timestamp_us > sample.now_us || sample.rx_timestamp_us > now_us {
        reset_recipient_window(state, rec);
        revoke_unconsumed_rest_auth(state, g);
        return;
    }

    // 4. Clock offset drift > 1 ms invalidates rest window and revokes unconsumed auth.
    if sample.clock_offset_drift_us > CLOCK_OFFSET_MAX_DRIFT_US {
        reset_recipient_window(state, rec);
        revoke_unconsumed_rest_auth(state, g);
        return;
    }

    // 5. Feetech / serial poisoned line: reject until silence completes; activity extends silence.
    let dev_cfg = state.config.devices[d];
    if sample.rx_timestamp_us < state.devices[d].poisoned_until_us
        || sample
            .poll_issued_us
            .is_some_and(|p| p < state.devices[d].poisoned_until_us)
    {
        reset_recipient_window(state, rec);
        revoke_unconsumed_rest_auth(state, g);
        if dev_cfg.poison_silence_us > 0 {
            let extended = sample.rx_timestamp_us.saturating_add(dev_cfg.poison_silence_us);
            state.devices[d].poisoned_until_us = state.devices[d].poisoned_until_us.max(extended);
        }
        return;
    }

    // 6. Finite numeric check & reply correlation check.
    if !sample.position_rad.is_finite()
        || sample.velocity_rad_s.is_some_and(|v| !v.is_finite())
        || sample
            .correlated_after_write_done_us
            .is_some_and(|wd| sample.rx_timestamp_us < wd)
        || sample
            .poll_issued_us
            .is_some_and(|pi| sample.rx_timestamp_us < pi)
    {
        reset_recipient_window(state, rec);
        revoke_unconsumed_rest_auth(state, g);
        return;
    }

    // 7. Out-of-order / backwards timestamp on this channel resets window and does not touch thermal/capture (M6).
    if state.joints[ch]
        .last_sample_us
        .is_some_and(|prev_ts| sample.rx_timestamp_us < prev_ts)
    {
        reset_recipient_window(state, rec);
        revoke_unconsumed_rest_auth(state, g);
        return;
    }

    // 8. Freshness check (<= max_capture_age_us, <= 100 ms) BEFORE applying thermal or capture (M6).
    if !is_sample_fresh(sample.rx_timestamp_us, now_us, ch_cfg.max_capture_age_us) {
        reset_recipient_window(state, rec);
        revoke_unconsumed_rest_auth(state, g);
        return;
    }

    // 9. Reported enable state: telemetry ONLY (B2).
    if let Some(en) = sample.reported_enabled {
        state.joints[ch].reported_enabled = Some(en);
    }

    // 10. Thermal telemetry carried on validated fresh in-order causal samples (M6).
    if let Some(temp_c) = sample.temperature_c {
        apply_thermal(
            state,
            sample.channel,
            temp_c,
            sample.rx_timestamp_us,
            now_us,
            actions,
        );
    }

    // 11. Update joint telemetry and anchored capture.
    {
        let rt = &mut state.joints[ch];
        let prev_was_fresh = rt
            .last_sample_us
            .is_some_and(|prev_ts| is_sample_fresh(prev_ts, now_us, ch_cfg.max_capture_age_us));
        if !prev_was_fresh {
            rt.captured_position_rad = None;
            rt.captured_at_us = None;
        }
        rt.last_sample_id = Some(sample.sample_id);
        rt.last_sample_us = Some(sample.rx_timestamp_us);
        rt.last_sample_release_id = Some(state.current_release_id);
        rt.last_measured_pos_rad = Some(sample.position_rad);
        rt.last_measured_vel_rad_s = sample.velocity_rad_s;
        // B6: do not preserve stale affirmative release evidence across samples without fresh proof.
        rt.last_release_evidence = sample.release_evidence.unwrap_or(ReleaseEvidence::Unknown);

        let holding_anchored = group_requires_holds(&state.groups[gi]);
        if holding_anchored {
            if rt.captured_position_rad.is_none() {
                rt.captured_position_rad = Some(sample.position_rad);
                rt.captured_at_us = Some(sample.rx_timestamp_us);
                rt.had_valid_capture = true;
            }
        } else {
            rt.captured_position_rad = Some(sample.position_rad);
            rt.captured_at_us = Some(sample.rx_timestamp_us);
            rt.had_valid_capture = true;
        }
    }

    // 12. If device was tracking SafeHold bus loss and ALL SafeHold channels on device are now fresh, clear timer (B11).
    let mut any_stale_on_dev = false;
    let mut dev_has_safe_hold = false;
    for c in 0..state.config.channel_count {
        let cc = state.config.channels[c as usize];
        if cc.device == ch_cfg.device && !matches!(cc.role, JointRole::PassiveSensor) {
            let grp = &state.groups[cc.group as usize];
            if grp.drive_state == DriveState::SafeHold && grp.torque_intent == TorqueIntent::On {
                dev_has_safe_hold = true;
                let fresh = state.joints[c as usize]
                    .last_sample_us
                    .is_some_and(|ts| is_sample_fresh(ts, now_us, cc.max_capture_age_us));
                if !fresh {
                    any_stale_on_dev = true;
                }
            }
        }
    }
    if dev_has_safe_hold && !any_stale_on_dev {
        state.devices[d].safe_hold_bus_loss_since_us = None;
    }

    // 13. Feed active disable transaction (AwaitRestGate or live invalidation of AuthorizedForRelease, B9).
    feed_rest_gate_sample(state, g, sample, now_us, actions);
}

fn push_rest_window_sample(state: &mut TorqueState, sample: CausalSample) {
    let ch = sample.channel as usize;
    let ch_cfg = state.config.channels[ch];
    let rec = ch_cfg.recipient;
    let rec_idx = rec as usize;
    let rec_cfg = state.config.recipients[rec_idx];

    // B8: Per-channel unused sample_id & distinct release check before contributing to snapshot.
    let current_rel = state.current_release_id;
    let rt = &state.joints[ch];
    if rt.last_window_sample_id == Some(sample.sample_id)
        || rt.last_window_release_id == Some(current_rel)
    {
        // Duplicate sample_id or already counted in this release for this channel.
        return;
    }

    let snap_ts = sample.rx_timestamp_us;
    let win = &mut state.rest_windows[rec_idx];
    if win.pending_release_id != current_rel || win.pending_agreed_channels.is_empty() {
        win.pending_release_id = current_rel;
        win.pending_agreed_channels = ChannelMask::EMPTY;
        win.pending_min_ts_us = snap_ts;
        win.pending_max_ts_us = snap_ts;
    }
    if win.pending_agreed_channels.contains(sample.channel) {
        return;
    }
    win.pending_agreed_channels.insert(sample.channel);
    win.pending_min_ts_us = win.pending_min_ts_us.min(snap_ts);
    win.pending_max_ts_us = win.pending_max_ts_us.max(snap_ts);

    // Record per-channel sample_id as consumed for this release's pending snapshot.
    state.joints[ch].last_window_sample_id = Some(sample.sample_id);
    state.joints[ch].last_window_release_id = Some(current_rel);

    if state.rest_windows[rec_idx].pending_agreed_channels == rec_cfg.channels {
        let min_age = recipient_min_capture_age_us(state, rec);
        let snap = RestSnapshot {
            release_id: current_rel,
            min_ts_us: state.rest_windows[rec_idx].pending_min_ts_us,
            max_ts_us: state.rest_windows[rec_idx].pending_max_ts_us,
        };
        let win = &mut state.rest_windows[rec_idx];
        if win.count == 0 {
            win.push_snapshot(snap);
        } else if win.latest_release_id() == current_rel {
            // Counts once per release.
        } else if snap.min_ts_us < win.latest_ts_us() {
            // Non-monotonic snapshot timestamp resets the window.
            reset_recipient_window(state, rec);
        } else if snap.min_ts_us.saturating_sub(win.latest_ts_us()) > min_age {
            // Stale blackout gap: restart window at this fresh snapshot.
            win.count = 0;
            win.push_snapshot(snap);
        } else {
            win.push_snapshot(snap);
        }
    }
}

fn check_sample_settle_causality(state: &TorqueState, sample: CausalSample) -> bool {
    let ch = sample.channel as usize;
    let ch_cfg = state.config.channels[ch];
    let dev_rt = state.devices[ch_cfg.device as usize];

    // B7: Feetech requires explicit correlated poll_issued_us AND correlated_after_write_done_us,
    // with poll issued strictly AFTER the 20 ms post-motion settle and AFTER poison silence.
    let causal_ts_for_settle = if matches!(ch_cfg.vendor, VendorKind::Feetech) {
        let (Some(poll_us), Some(wd_us)) = (
            sample.poll_issued_us,
            sample.correlated_after_write_done_us,
        ) else {
            return false;
        };
        if poll_us < wd_us
            || poll_us < dev_rt.poisoned_until_us
            || sample.rx_timestamp_us < poll_us
        {
            return false;
        }
        poll_us
    } else {
        sample.rx_timestamp_us
    };

    // Strict post-motion settle: strictly > 20 ms after last motion write.
    if let Some(last_mw) = state.joints[ch].last_motion_write_us {
        if causal_ts_for_settle <= last_mw.saturating_add(REST_SETTLE_US) {
            return false;
        }
    }
    true
}

fn feed_passive_rest_window_sample(state: &mut TorqueState, g: u8, sample: CausalSample) {
    let gi = g as usize;
    if state.groups[gi].torque_intent != TorqueIntent::On
        || state.groups[gi].drive_state.is_protective()
        || state.groups[gi].drive_state == DriveState::ReadyToSwitchOn
    {
        return;
    }
    let ch = sample.channel as usize;
    let ch_cfg = state.config.channels[ch];
    let rec = ch_cfg.recipient;
    let rec_cfg = state.config.recipients[rec as usize];

    if !check_sample_settle_causality(state, sample) {
        reset_recipient_window(state, rec);
        return;
    }

    let agrees = match ch_cfg.role {
        JointRole::LoadBearing => {
            if !rec_cfg.rest_evidence_qualified {
                false
            } else {
                let pos_ok = (sample.position_rad - ch_cfg.rest_position_rad).abs()
                    <= ch_cfg.rest_tolerance_rad;
                let vel_ok = match (ch_cfg.rest_max_velocity_rad_s, sample.velocity_rad_s) {
                    (Some(max_v), Some(v)) => v.abs() <= max_v,
                    _ => true,
                };
                pos_ok && vel_ok
            }
        }
        JointRole::Gripper(GripperReleasePolicy::LegacyJ7) => true,
        JointRole::Gripper(GripperReleasePolicy::ExplicitHand)
        | JointRole::ActiveTool(ActiveToolSafeOutput::BoundedRetention { .. }) => matches!(
            sample.release_evidence,
            Some(ReleaseEvidence::SupportedObject | ReleaseEvidence::VerifiedEmptyGrasp)
        ),
        JointRole::ActiveTool(ActiveToolSafeOutput::StopOutput) => {
            match (ch_cfg.rest_max_velocity_rad_s, sample.velocity_rad_s) {
                (Some(max_v), Some(v)) => v.abs() <= max_v,
                _ => true,
            }
        }
        JointRole::PassiveSensor => false,
    };

    if !agrees {
        reset_recipient_window(state, rec);
        return;
    }

    push_rest_window_sample(state, sample);
}

fn feed_rest_gate_sample(
    state: &mut TorqueState,
    g: u8,
    sample: CausalSample,
    now_us: u64,
    actions: &mut ActionSet,
) {
    let gi = g as usize;
    let Some(tx) = state.groups[gi].disable_tx else {
        // Continuously accumulate valid rest-window evidence while armed/holding so a subsequent
        // DisableRequest at an already-stationary rest pose matches live driver K=3 accumulation.
        feed_passive_rest_window_sample(state, g, sample);
        return;
    };
    if tx.origin.is_protective() || tx.committed_off {
        return;
    }
    let ch = sample.channel as usize;
    let ch_cfg = state.config.channels[ch];
    let rec = ch_cfg.recipient;
    let rec_idx = rec as usize;
    if !tx.recipient_mask.contains(rec) {
        return;
    }

    // B9: If currently AuthorizedForRelease and this newer sample invalidates rest/release proof,
    // revoke authorization (and abort on definite non-rest!).
    let was_authorized = matches!(tx.phase, DisablePhase::AuthorizedForRelease { .. });
    if !was_authorized && !matches!(tx.phase, DisablePhase::AwaitRestGate) {
        return;
    }

    if now_us >= tx.deadline_us {
        let code = rest_timeout_code(state, &tx);
        abort_non_protective_tx(state, g, code, true, actions);
        return;
    }

    let rec_cfg = state.config.recipients[rec_idx];

    if !check_sample_settle_causality(state, sample) {
        reset_recipient_window(state, rec);
        if was_authorized {
            revoke_unconsumed_rest_auth(state, g);
        }
        return;
    }

    // Role-specific agreement check (B6: affirmative fresh release evidence required on sample!).
    let agrees = match ch_cfg.role {
        JointRole::LoadBearing => {
            if !rec_cfg.rest_evidence_qualified {
                abort_non_protective_tx(
                    state,
                    g,
                    RefusalCode::RestEvidenceUnqualified,
                    false,
                    actions,
                );
                return;
            }
            let pos_ok = (sample.position_rad - ch_cfg.rest_position_rad).abs()
                <= ch_cfg.rest_tolerance_rad;
            let vel_ok = match (ch_cfg.rest_max_velocity_rad_s, sample.velocity_rad_s) {
                (Some(max_v), Some(v)) => v.abs() <= max_v,
                _ => true,
            };
            if !pos_ok || !vel_ok {
                reset_recipient_window(state, rec);
                revoke_unconsumed_rest_auth(state, g);
                abort_non_protective_tx_with_ch(
                    state,
                    g,
                    RefusalCode::TorqueOffWouldDrop,
                    Some(sample.channel),
                    false,
                    actions,
                );
                return;
            }
            true
        }
        JointRole::Gripper(GripperReleasePolicy::LegacyJ7) => true,
        JointRole::Gripper(GripperReleasePolicy::ExplicitHand) => {
            if !tx.explicit_release_authorized {
                abort_non_protective_tx_with_ch(
                    state,
                    g,
                    RefusalCode::HandReleaseNotAuthorized,
                    Some(sample.channel),
                    false,
                    actions,
                );
                return;
            }
            match sample.release_evidence {
                Some(ReleaseEvidence::HoldingUnsupportedPayload) => {
                    reset_recipient_window(state, rec);
                    revoke_unconsumed_rest_auth(state, g);
                    abort_non_protective_tx_with_ch(
                        state,
                        g,
                        RefusalCode::TorqueOffWouldDrop,
                        Some(sample.channel),
                        false,
                        actions,
                    );
                    return;
                }
                Some(ReleaseEvidence::SupportedObject | ReleaseEvidence::VerifiedEmptyGrasp) => {
                    true
                }
                Some(ReleaseEvidence::Unknown) | None => {
                    reset_recipient_window(state, rec);
                    if was_authorized {
                        revoke_unconsumed_rest_auth(state, g);
                    }
                    false
                }
            }
        }
        JointRole::ActiveTool(ActiveToolSafeOutput::StopOutput) => {
            let vel_ok = match (ch_cfg.rest_max_velocity_rad_s, sample.velocity_rad_s) {
                (Some(max_v), Some(v)) => v.abs() <= max_v,
                _ => true,
            };
            if !vel_ok {
                reset_recipient_window(state, rec);
                if was_authorized {
                    revoke_unconsumed_rest_auth(state, g);
                }
                false
            } else {
                true
            }
        }
        JointRole::ActiveTool(ActiveToolSafeOutput::BoundedRetention { .. }) => {
            if !tx.explicit_release_authorized {
                abort_non_protective_tx_with_ch(
                    state,
                    g,
                    RefusalCode::ActiveToolReleaseNotAuthorized,
                    Some(sample.channel),
                    false,
                    actions,
                );
                return;
            }
            match sample.release_evidence {
                Some(ReleaseEvidence::HoldingUnsupportedPayload) => {
                    reset_recipient_window(state, rec);
                    revoke_unconsumed_rest_auth(state, g);
                    abort_non_protective_tx_with_ch(
                        state,
                        g,
                        RefusalCode::TorqueOffWouldDrop,
                        Some(sample.channel),
                        false,
                        actions,
                    );
                    return;
                }
                Some(ReleaseEvidence::SupportedObject | ReleaseEvidence::VerifiedEmptyGrasp) => {
                    true
                }
                Some(ReleaseEvidence::Unknown) | None => {
                    reset_recipient_window(state, rec);
                    if was_authorized {
                        revoke_unconsumed_rest_auth(state, g);
                    }
                    false
                }
            }
        }
        JointRole::PassiveSensor => false,
    };

    if !agrees || was_authorized {
        return;
    }

    push_rest_window_sample(state, sample);

    if state.groups[gi].pending_disable_dispatch.is_none() {
        try_authorize_rest_gate(state, g, now_us, actions);
    }
}

/// Evaluate whether physical recipient `r` satisfies its role gate at `now_us` on its 3-slot sliding window (B8).
pub(crate) fn recipient_passes_gate(
    state: &TorqueState,
    r: u8,
    tx: &DisableTransaction,
    now_us: u64,
) -> bool {
    if r >= state.config.recipient_count {
        return false;
    }
    let rec = &state.config.recipients[r as usize];
    let dev_rt = &state.devices[rec.device as usize];
    let win = &state.rest_windows[r as usize];

    if matches!(rec.role, JointRole::LoadBearing) && !rec.rest_evidence_qualified {
        return false;
    }
    if win.count < REST_WINDOW_SAMPLES {
        return false;
    }
    let s0 = win.snapshots[0];
    let s1 = win.snapshots[1];
    let s2 = win.snapshots[2];
    if !(s0.release_id < s1.release_id && s1.release_id < s2.release_id) {
        return false;
    }
    if !(s0.min_ts_us <= s1.min_ts_us && s1.min_ts_us <= s2.min_ts_us) {
        return false;
    }
    if now_us < dev_rt.poisoned_until_us || s0.min_ts_us < dev_rt.poisoned_until_us {
        return false;
    }
    // B8: ALL THREE snapshots must be fresh at now_us (checking s0.min_ts_us enforces all 3!).
    let min_age = recipient_min_capture_age_us(state, r);
    if !is_sample_fresh(s0.min_ts_us, now_us, min_age)
        || !is_sample_fresh(s2.max_ts_us, now_us, min_age)
    {
        return false;
    }
    if win.span_us() < REST_WINDOW_MIN_SPAN_US {
        return false;
    }

    for c in rec.channels.iter() {
        let ch_cfg = &state.config.channels[c as usize];
        let rt = &state.joints[c as usize];
        if !rt
            .last_sample_us
            .is_some_and(|ts| is_sample_fresh(ts, now_us, ch_cfg.max_capture_age_us))
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
                if let (Some(max_v), Some(v)) =
                    (ch_cfg.rest_max_velocity_rad_s, rt.last_measured_vel_rad_s)
                {
                    if !v.is_finite() || v.abs() > max_v {
                        return false;
                    }
                }
            }
            JointRole::Gripper(GripperReleasePolicy::LegacyJ7) => {
                if !rt.last_measured_pos_rad.is_some_and(f32::is_finite) {
                    return false;
                }
            }
            JointRole::Gripper(GripperReleasePolicy::ExplicitHand) => {
                if !tx.explicit_release_authorized {
                    return false;
                }
                if !matches!(
                    rt.last_release_evidence,
                    ReleaseEvidence::SupportedObject | ReleaseEvidence::VerifiedEmptyGrasp
                ) {
                    return false;
                }
            }
            JointRole::ActiveTool(ActiveToolSafeOutput::StopOutput) => {
                if !rt.last_measured_pos_rad.is_some_and(f32::is_finite) {
                    return false;
                }
            }
            JointRole::ActiveTool(ActiveToolSafeOutput::BoundedRetention { .. }) => {
                if !tx.explicit_release_authorized {
                    return false;
                }
                if !matches!(
                    rt.last_release_evidence,
                    ReleaseEvidence::SupportedObject | ReleaseEvidence::VerifiedEmptyGrasp
                ) {
                    return false;
                }
            }
            JointRole::PassiveSensor => return false,
        }
    }

    true
}

fn try_authorize_rest_gate(
    state: &mut TorqueState,
    g: u8,
    now_us: u64,
    actions: &mut ActionSet,
) {
    let gi = g as usize;
    let Some(tx) = state.groups[gi].disable_tx else {
        return;
    };
    if !matches!(tx.phase, DisablePhase::AwaitRestGate)
        || tx.origin.is_protective()
        || state.groups[gi].pending_disable_dispatch.is_some()
    {
        return;
    }
    if now_us >= tx.deadline_us || tx.auth_generation != state.groups[gi].stop_generation {
        return;
    }
    if tx.recipient_mask.is_empty() {
        return;
    }

    for r in tx.recipient_mask.iter() {
        if !recipient_passes_gate(state, r, &tx, now_us) {
            return;
        }
    }

    emit_disable_attempt(state, g, DisableKind::Requested, now_us, actions);
}

// =========================================================================
// Event::WriteResult
// =========================================================================

fn handle_write_result(
    state: &mut TorqueState,
    wr: WriteResult,
    now_us: u64,
    actions: &mut ActionSet,
) {
    match wr.kind {
        WriteKind::Hold => {
            // Stationary hold preserves last_motion_write_us and rest windows!
            // M10: Track per-release stopped-hold contention on the device (aggregated once per Release).
            if wr.device < state.config.device_count
                && wr.group < state.config.group_count
                && group_requires_holds(&state.groups[wr.group as usize])
                && !wr.target_mask.is_empty()
            {
                let mut any_sent = false;
                let mut any_contended = false;
                for idx in wr.target_mask.iter() {
                    match wr.outcomes[idx as usize] {
                        MotorWriteOutcome::Sent | MotorWriteOutcome::Ambiguous => any_sent = true,
                        MotorWriteOutcome::LockContended => any_contended = true,
                        _ => {}
                    }
                }
                let dev_rt = &mut state.devices[wr.device as usize];
                if any_sent {
                    dev_rt.hold_succeeded_in_release = true;
                    dev_rt.consecutive_contention_skips = 0;
                    dev_rt.contention_escalated = false;
                } else if any_contended {
                    dev_rt.hold_contended_in_release = true;
                }
            }
        }
        WriteKind::Motion => {
            let boundary = wr.write_done_us.min(now_us);
            for c in wr.target_mask.iter() {
                if c >= state.config.channel_count {
                    continue;
                }
                if matches!(
                    wr.outcomes[c as usize],
                    MotorWriteOutcome::Sent | MotorWriteOutcome::Ambiguous
                ) {
                    let rt = &mut state.joints[c as usize];
                    rt.last_motion_write_us =
                        Some(rt.last_motion_write_us.map_or(boundary, |p| p.max(boundary)));
                    // B6: motion invalidates prior grasp release evidence.
                    rt.last_release_evidence = ReleaseEvidence::Unknown;
                    let ch_cfg = state.config.channels[c as usize];
                    if !matches!(ch_cfg.role, JointRole::PassiveSensor)
                        && ch_cfg.recipient < state.config.recipient_count
                    {
                        reset_recipient_window(state, ch_cfg.recipient);
                        // B9: motion write invalidates unconsumed AuthorizedForRelease.
                        revoke_unconsumed_rest_auth(state, ch_cfg.group);
                    }
                }
            }
        }
        WriteKind::Enable => {
            // B2: Only a failure correlated to the active attempt_id can fail this enable.
            if wr.group < state.config.group_count
                && state.groups[wr.group as usize].drive_state == DriveState::ReadyToSwitchOn
            {
                let matches_attempt = state.groups[wr.group as usize]
                    .enable_tx
                    .is_some_and(|etx| etx.attempt_id == wr.dispatch_id);
                if matches_attempt {
                    let any_sent_or_ambiguous = wr.target_mask.iter().any(|c| {
                        matches!(
                            wr.outcomes[c as usize],
                            MotorWriteOutcome::Sent | MotorWriteOutcome::Ambiguous
                        )
                    });
                    if any_sent_or_ambiguous {
                        if let Some(etx_mut) = state.groups[wr.group as usize].enable_tx.as_mut() {
                            etx_mut.any_write_sent = true;
                        }
                    }
                    let any_fatal = wr.target_mask.iter().any(|c| {
                        matches!(
                            wr.outcomes[c as usize],
                            MotorWriteOutcome::Failed | MotorWriteOutcome::RejectedStale
                        )
                    });
                    if any_fatal {
                        enter_failed_hold(state, wr.group, now_us, actions);
                    }
                }
            }
        }
        WriteKind::DisableRequested | WriteKind::DisableProtective => {
            handle_disable_write_result(state, wr, now_us, actions);
        }
        WriteKind::Damp | WriteKind::Refresh | WriteKind::ModeSwitch => {}
    }
}

fn handle_disable_write_result(
    state: &mut TorqueState,
    wr: WriteResult,
    now_us: u64,
    actions: &mut ActionSet,
) {
    if wr.group >= state.config.group_count {
        return;
    }
    let g = wr.group;
    let gi = g as usize;
    let grp_cfg = state.config.groups[gi];

    // B1: Exact dispatch_id + kind + generation match against the single pending wire dispatch.
    let Some(mut pend) = state.groups[gi].pending_disable_dispatch else {
        return;
    };
    let expected_kind = match pend.kind {
        DisableKind::Requested => WriteKind::DisableRequested,
        DisableKind::Protective => WriteKind::DisableProtective,
    };
    if wr.dispatch_id != pend.dispatch_id
        || wr.kind != expected_kind
        || wr.generation != pend.generation
    {
        return;
    }

    // Consume only recipients still remaining for this dispatch (prevents duplicate counting, B1).
    let matched_recipients = wr.target_mask.intersect(pend.remaining_recipients);
    if matched_recipients.is_empty() {
        return;
    }

    let mut sent_mask = ChannelMask::EMPTY;
    let mut ambiguous_mask = ChannelMask::EMPTY;
    let mut failed_mask = ChannelMask::EMPTY;
    let mut deferred_mask = ChannelMask::EMPTY;
    let mut rejected_stale_mask = ChannelMask::EMPTY;

    for r in matched_recipients.iter() {
        match wr.outcomes[r as usize] {
            MotorWriteOutcome::Sent => sent_mask.insert(r),
            MotorWriteOutcome::Ambiguous => ambiguous_mask.insert(r),
            MotorWriteOutcome::Failed => failed_mask.insert(r),
            MotorWriteOutcome::Deferred | MotorWriteOutcome::LockContended => {
                deferred_mask.insert(r)
            }
            MotorWriteOutcome::RejectedStale => rejected_stale_mask.insert(r),
            MotorWriteOutcome::NotTargeted => {}
        }
    }

    pend.remaining_recipients = pend.remaining_recipients.difference(matched_recipients);
    pend.sent_mask = pend.sent_mask.union(sent_mask);
    pend.ambiguous_mask = pend.ambiguous_mask.union(ambiguous_mask);
    pend.failed_mask = pend.failed_mask.union(failed_mask);
    pend.deferred_mask = pend.deferred_mask.union(deferred_mask);
    pend.rejected_stale_mask = pend.rejected_stale_mask.union(rejected_stale_mask);

    // Track confirmed physical off coverage per recipient (B10).
    state.groups[gi].physically_sent_off_mask =
        state.groups[gi].physically_sent_off_mask.union(sent_mask);

    let any_sent_or_ambiguous_now = !sent_mask.is_empty() || !ambiguous_mask.is_empty();
    if any_sent_or_ambiguous_now {
        // First successful or possibly-sent disable IMMEDIATELY commits TorqueIntent::Off!
        state.groups[gi].torque_intent = TorqueIntent::Off;
    }

    // Propagate partial outcomes to active tx immediately.
    if pend.tx_still_active {
        if let Some(tx_mut) = state.groups[gi].disable_tx.as_mut() {
            if tx_mut.tx_id == pend.tx_id {
                tx_mut.sent_mask = tx_mut.sent_mask.union(sent_mask);
                tx_mut.ambiguous_mask = tx_mut.ambiguous_mask.union(ambiguous_mask);
                tx_mut.failed_mask = tx_mut.failed_mask.union(failed_mask);
                tx_mut.deferred_mask = tx_mut.deferred_mask.union(deferred_mask);
                if any_sent_or_ambiguous_now {
                    tx_mut.committed_off = true;
                    tx_mut.phase = DisablePhase::Dispatched;
                }
            }
        }
    }

    // B1: Wait for all targeted recipients/devices of this dispatch before advancing repeat count.
    if !pend.remaining_recipients.is_empty() {
        state.groups[gi].pending_disable_dispatch = Some(pend);
        return;
    }

    // All expected recipients for this dispatch have reported once: clear pending dispatch.
    state.groups[gi].pending_disable_dispatch = None;

    let any_sent_or_ambiguous_total = !pend.sent_mask.is_empty() || !pend.ambiguous_mask.is_empty();

    if !pend.tx_still_active {
        // Reconciled a cancelled/revoked wire dispatch: if anything reached wire, emit completion.
        if any_sent_or_ambiguous_total {
            let covered = pend.sent_mask.union(pend.ambiguous_mask);
            let unsent = pend.target_recipients.difference(covered);
            let partial = pend.sent_mask != pend.target_recipients
                || !pend.ambiguous_mask.is_empty()
                || !pend.failed_mask.is_empty()
                || !unsent.is_empty();
            actions.disable_completions[gi] = Some(DisableCompletion {
                group: g,
                tx_id: pend.tx_id,
                origin: pend.origin,
                recipient_mask: pend.target_recipients,
                sent_mask: pend.sent_mask,
                ambiguous_mask: pend.ambiguous_mask,
                failed_mask: pend.failed_mask,
                unsent_mask: unsent,
                committed_off: true,
                partial_or_ambiguous: partial,
                timed_out: false,
                repeats_completed: 1,
            });
            update_post_off_drive_state(state, g, pend.origin.is_protective());
        }
        return;
    }

    let Some(tx) = state.groups[gi].disable_tx else {
        return;
    };
    if tx.tx_id != pend.tx_id {
        return;
    }

    if tx.committed_off {
        if any_sent_or_ambiguous_total {
            if let Some(tx_mut) = state.groups[gi].disable_tx.as_mut() {
                tx_mut.repeats_completed = tx_mut.repeats_completed.saturating_add(1);
                tx_mut.next_repeat_at_us = now_us.saturating_add(grp_cfg.disable_repeat_interval_us);
            }
        }
        let tx_now = state.groups[gi].disable_tx.unwrap();
        let done_repeats = tx_now.repeats_completed >= grp_cfg.disable_repeats;
        let past_deadline = now_us >= tx_now.deadline_us;
        if done_repeats || past_deadline {
            finish_disable_tx(state, g, past_deadline && !done_repeats, actions);
        }
    } else {
        // ZERO writes across the entire dispatch attempt: TorqueIntent stays On!
        if !tx.origin.is_protective() {
            if !pend.rejected_stale_mask.is_empty()
                || tx.auth_generation != state.groups[gi].stop_generation
            {
                abort_non_protective_tx(state, g, RefusalCode::PreemptedByStop, false, actions);
            } else if now_us >= tx.deadline_us {
                let code = rest_timeout_code(state, &tx);
                abort_non_protective_tx(state, g, code, true, actions);
            } else if !pend.failed_mask.is_empty() && pend.deferred_mask.is_empty() {
                abort_non_protective_tx(state, g, RefusalCode::ZeroWritesRefused, false, actions);
            } else if let Some(tx_mut) = state.groups[gi].disable_tx.as_mut() {
                tx_mut.phase = DisablePhase::AwaitRestGate;
            }
        } else if now_us >= tx.deadline_us {
            close_protective_zero_write_expiry(state, g, now_us, actions);
        } else if let Some(tx_mut) = state.groups[gi].disable_tx.as_mut() {
            tx_mut.phase = DisablePhase::ProtectivePendingDispatch;
        }
    }
}

// =========================================================================
// Enable, Handshake, Mode, Timeout, Motion
// =========================================================================

fn handle_enable_start(state: &mut TorqueState, group: u8, now_us: u64, actions: &mut ActionSet) {
    if group >= state.config.group_count {
        set_refusal_safe(actions, group, RefusalCode::GroupOutOfRange, None);
        return;
    }
    let gi = group as usize;
    let grp = state.groups[gi];
    let grp_cfg = state.config.groups[gi];

    if state.control_panic_latched || grp.drive_state == DriveState::SafeHold {
        actions.refusals[gi] = Some(RefusalEvent {
            group,
            code: RefusalCode::ControlPanicSafeHold,
            channel: None,
        });
        return;
    }
    if state.worker_panic_latched || grp.pending_park.is_some() {
        actions.refusals[gi] = Some(RefusalEvent {
            group,
            code: RefusalCode::InternalFault,
            channel: None,
        });
        return;
    }
    if !grp.thermal_tripped_mask.is_empty() {
        actions.refusals[gi] = Some(RefusalEvent {
            group,
            code: RefusalCode::ThermalActive,
            channel: None,
        });
        return;
    }
    if grp.protective_latch || grp.drive_state.is_protective() {
        actions.refusals[gi] = Some(RefusalEvent {
            group,
            code: RefusalCode::ProtectiveLatched,
            channel: None,
        });
        return;
    }
    // B1: Fence enable while ANY disable transaction OR unresolved wire disable dispatch exists.
    if grp.disable_tx.is_some() || grp.pending_disable_dispatch.is_some() {
        actions.refusals[gi] = Some(RefusalEvent {
            group,
            code: RefusalCode::DisableInProgress,
            channel: None,
        });
        return;
    }
    if !grp.faults.is_empty()
        || matches!(
            grp.drive_state,
            DriveState::Fault | DriveState::FaultReactionActive | DriveState::FailedHold
        )
    {
        actions.refusals[gi] = Some(RefusalEvent {
            group,
            code: RefusalCode::FaultLatched,
            channel: None,
        });
        return;
    }
    if grp.drive_state == DriveState::ReadyToSwitchOn {
        actions.refusals[gi] = Some(RefusalEvent {
            group,
            code: RefusalCode::AlreadyEnabled,
            channel: None,
        });
        return;
    }
    if device_wide_mode_fenced(state, grp_cfg.devices) {
        actions.refusals[gi] = Some(RefusalEvent {
            group,
            code: RefusalCode::ModeUnverified,
            channel: None,
        });
        return;
    }

    // M11: Damiao TIMEOUT readback gate requires matching readback in the CURRENT device session.
    let active = group_active_channels(state, group);
    for c in active.iter() {
        let ch_cfg = &state.config.channels[c as usize];
        if let Some(expected) = ch_cfg.damiao_timeout_reg {
            let cur_session = state.devices[ch_cfg.device as usize].session_id;
            let rt = &state.joints[c as usize];
            if rt.verified_timeout_reg != Some(expected)
                || rt.verified_timeout_session != Some(cur_session)
            {
                actions.refusals[gi] = Some(RefusalEvent {
                    group,
                    code: RefusalCode::TimeoutMismatch,
                    channel: Some(c),
                });
                return;
            }
        }
    }

    // Accept EnableStart (including idempotent re-enable from OperationEnabled): allocate unique
    // attempt_id and set TorqueIntent::On immediately. When already OperationEnabled, mark
    // `any_write_sent = true` so pre-TX cancellation can never disarm an already-energized arm.
    let was_already_enabled = grp.drive_state == DriveState::OperationEnabled;
    let attempt_id = state.alloc_capability_id();
    let deadline_us = now_us.saturating_add(grp_cfg.enable_deadline_us);
    let gen = grp.stop_generation;
    state.groups[gi].torque_intent = TorqueIntent::On;
    state.groups[gi].drive_state = DriveState::ReadyToSwitchOn;
    state.groups[gi].enable_phase = EnablePhase::Enabling;
    state.groups[gi].physically_sent_off_mask = ChannelMask::EMPTY;
    state.groups[gi].enable_tx = Some(EnableTransaction {
        attempt_id,
        started_us: now_us,
        deadline_us,
        generation: gen,
        target_channels: active,
        confirmed_channels: ChannelMask::EMPTY,
        any_write_sent: was_already_enabled,
    });
    if !was_already_enabled {
        for r in grp_cfg.recipients.iter() {
            reset_recipient_window(state, r);
        }
    }
    for c in active.iter() {
        state.joints[c as usize].retention_started_us = None;
        state.joints[c as usize].last_release_evidence = ReleaseEvidence::Unknown;
        if !was_already_enabled {
            state.joints[c as usize].had_valid_capture = false;
        }
    }
    actions.enables[gi] = Some(EnableCommand {
        group,
        attempt_id,
        channels: active,
        generation: gen,
        deadline_us,
    });
}

fn handle_enable_cancelled_pre_tx(
    state: &mut TorqueState,
    group: u8,
    attempt_id: u64,
    now_us: u64,
    actions: &mut ActionSet,
) {
    if group >= state.config.group_count {
        set_refusal_safe(actions, group, RefusalCode::GroupOutOfRange, None);
        return;
    }
    let gi = group as usize;
    if state.groups[gi].drive_state != DriveState::ReadyToSwitchOn {
        return;
    }
    let Some(etx) = state.groups[gi].enable_tx else {
        return;
    };
    if etx.attempt_id != attempt_id {
        return;
    }
    if !etx.any_write_sent && etx.confirmed_channels.is_empty() {
        state.groups[gi].drive_state = DriveState::SwitchOnDisabled;
        state.groups[gi].enable_phase = EnablePhase::Idle;
        state.groups[gi].enable_tx = None;
        state.groups[gi].torque_intent = TorqueIntent::Off;
        state.groups[gi].stop_generation = state.groups[gi].stop_generation.saturating_add(1);
        let active = group_active_channels(state, group);
        for c in active.iter() {
            state.joints[c as usize].had_valid_capture = false;
        }
    } else {
        enter_failed_hold(state, group, now_us, actions);
    }
}

fn handle_handshake_result(
    state: &mut TorqueState,
    group: u8,
    attempt_id: u64,
    confirmed_mask: ChannelMask,
    outcome: HandshakeOutcome,
    now_us: u64,
    actions: &mut ActionSet,
) {
    if group >= state.config.group_count {
        return;
    }
    let gi = group as usize;
    if state.groups[gi].drive_state != DriveState::ReadyToSwitchOn {
        return;
    }
    let Some(etx) = state.groups[gi].enable_tx else {
        return;
    };
    // B2: Ignore stale or mismatched attempt_id callbacks!
    if etx.attempt_id != attempt_id {
        return;
    }
    if now_us >= etx.deadline_us {
        enter_failed_hold(state, group, now_us, actions);
        return;
    }

    match outcome {
        HandshakeOutcome::Failed | HandshakeOutcome::FaultReported { .. } => {
            enter_failed_hold(state, group, now_us, actions);
        }
        HandshakeOutcome::Pending => {
            for c in confirmed_mask.intersect(etx.target_channels).iter() {
                state.joints[c as usize].reported_enabled = Some(true);
            }
            if let Some(etx_mut) = state.groups[gi].enable_tx.as_mut() {
                etx_mut.confirmed_channels = etx_mut
                    .confirmed_channels
                    .union(confirmed_mask.intersect(etx_mut.target_channels));
            }
            if state.groups[gi]
                .enable_tx
                .is_some_and(|e| e.confirmed_channels == e.target_channels)
            {
                state.groups[gi].enable_phase = EnablePhase::Validating;
                if validate_enable_postconditions(state, group) {
                    state.groups[gi].drive_state = DriveState::OperationEnabled;
                    state.groups[gi].enable_phase = EnablePhase::Armed;
                    state.groups[gi].enable_tx = None;
                    state.groups[gi].torque_intent = TorqueIntent::On;
                }
            }
        }
        HandshakeOutcome::Confirmed => {
            for c in confirmed_mask.intersect(etx.target_channels).iter() {
                state.joints[c as usize].reported_enabled = Some(true);
            }
            if let Some(etx_mut) = state.groups[gi].enable_tx.as_mut() {
                etx_mut.confirmed_channels = etx_mut
                    .confirmed_channels
                    .union(confirmed_mask.intersect(etx_mut.target_channels));
            }
            state.groups[gi].enable_phase = EnablePhase::Validating;
            let all_confirmed = state.groups[gi]
                .enable_tx
                .is_some_and(|e| e.confirmed_channels == e.target_channels);
            if all_confirmed && validate_enable_postconditions(state, group) {
                state.groups[gi].drive_state = DriveState::OperationEnabled;
                state.groups[gi].enable_phase = EnablePhase::Armed;
                state.groups[gi].enable_tx = None;
                state.groups[gi].torque_intent = TorqueIntent::On;
            } else {
                enter_failed_hold(state, group, now_us, actions);
            }
        }
    }
}

fn validate_enable_postconditions(state: &TorqueState, group: u8) -> bool {
    let gi = group as usize;
    let grp_cfg = state.config.groups[gi];
    if device_wide_mode_fenced(state, grp_cfg.devices) {
        return false;
    }
    let active = group_active_channels(state, group);
    for c in active.iter() {
        let ch_cfg = &state.config.channels[c as usize];
        let rt = &state.joints[c as usize];
        if rt.verified_mode != Some(rt.desired_mode) {
            return false;
        }
        if let Some(expected) = ch_cfg.damiao_timeout_reg {
            let cur_session = state.devices[ch_cfg.device as usize].session_id;
            if rt.verified_timeout_reg != Some(expected)
                || rt.verified_timeout_session != Some(cur_session)
            {
                return false;
            }
        }
    }
    true
}

/// Failed, interrupted, or expired enable -> `FailedHold` + `ENABLE_FAILED`, NO rollback.
fn enter_failed_hold(state: &mut TorqueState, group: u8, now_us: u64, actions: &mut ActionSet) {
    let gi = group as usize;
    state.groups[gi].drive_state = DriveState::FailedHold;
    state.groups[gi].enable_phase = EnablePhase::FailedHold;
    state.groups[gi].enable_tx = None;
    state.groups[gi].torque_intent = TorqueIntent::On;
    state.groups[gi].faults.insert(TorqueFaultKind::EnableFailed);
    state.groups[gi].stop_generation = state.groups[gi].stop_generation.saturating_add(1);
    anchor_group_captures(state, group, now_us);
    actions.refusals[gi] = Some(RefusalEvent {
        group,
        code: RefusalCode::EnableFailed,
        channel: None,
    });
}

fn handle_mit_ownership_change(
    state: &mut TorqueState,
    group: u8,
    channels: ChannelMask,
    target_mode: ControlMode,
    device_wide: bool,
    now_us: u64,
    actions: &mut ActionSet,
) {
    if group >= state.config.group_count {
        return;
    }
    let gi = group as usize;
    let grp_cfg = state.config.groups[gi];
    let target = channels.intersect(grp_cfg.channels);
    let mut affected_devices = DeviceMask::EMPTY;

    for c in target.iter() {
        if c < state.config.channel_count
            && !matches!(state.config.channels[c as usize].role, JointRole::PassiveSensor)
        {
            state.joints[c as usize].desired_mode = target_mode;
            state.joints[c as usize].verified_mode = None;
            affected_devices.insert(state.config.channels[c as usize].device);
        }
    }

    // B13: Track live device-wide mode transition fence on EVERY affected device (even when static
    // `device_wide_mode_switch` is false!) and interrupt EVERY affected group's active enable!
    let mut fenced_devices = DeviceMask::EMPTY;
    for d in affected_devices.iter() {
        if device_wide || state.config.devices[d as usize].device_wide_mode_switch {
            state.devices[d as usize].mode_fence_active = true;
            fenced_devices.insert(d);
            for c in 0..state.config.channel_count {
                if state.config.channels[c as usize].device == d
                    && !matches!(state.config.channels[c as usize].role, JointRole::PassiveSensor)
                {
                    state.joints[c as usize].verified_mode = None;
                }
            }
        }
    }

    // Interrupt affected enables through the common FailedHold path (B13).
    for g2 in 0..state.config.group_count {
        let g2i = g2 as usize;
        let touched = g2 == group
            || !state.config.groups[g2i]
                .devices
                .intersect(fenced_devices)
                .is_empty();
        if touched && state.groups[g2i].drive_state == DriveState::ReadyToSwitchOn {
            enter_failed_hold(state, g2, now_us, actions);
        }
    }
}

fn handle_mode_verified(
    state: &mut TorqueState,
    channel: u8,
    mode: ControlMode,
    now_us: u64,
    actions: &mut ActionSet,
) {
    if channel >= state.config.channel_count {
        return;
    }
    let ch = channel as usize;
    state.joints[ch].verified_mode = Some(mode);
    let dev = state.config.channels[ch].device;

    // B13: Clear live device-wide mode fence ONLY once EVERY active channel on `dev` is verified.
    if state.devices[dev as usize].mode_fence_active {
        let mut all_verified = true;
        for c in 0..state.config.channel_count {
            let cc = state.config.channels[c as usize];
            if cc.device == dev && !matches!(cc.role, JointRole::PassiveSensor) {
                let rt = &state.joints[c as usize];
                if rt.verified_mode != Some(rt.desired_mode) {
                    all_verified = false;
                    break;
                }
            }
        }
        if all_verified {
            state.devices[dev as usize].mode_fence_active = false;
        }
    }

    // Check any ReadyToSwitchOn group touching this device that was waiting in Validating.
    for g in 0..state.config.group_count {
        let gi = g as usize;
        if state.config.groups[gi].devices.contains(dev)
            && state.groups[gi].drive_state == DriveState::ReadyToSwitchOn
        {
            if let Some(etx) = state.groups[gi].enable_tx {
                if now_us >= etx.deadline_us {
                    enter_failed_hold(state, g, now_us, actions);
                } else if etx.confirmed_channels == etx.target_channels
                    && validate_enable_postconditions(state, g)
                {
                    state.groups[gi].drive_state = DriveState::OperationEnabled;
                    state.groups[gi].enable_phase = EnablePhase::Armed;
                    state.groups[gi].enable_tx = None;
                    state.groups[gi].torque_intent = TorqueIntent::On;
                }
            }
        }
    }
}

fn handle_timeout_readback(state: &mut TorqueState, channel: u8, timeout_reg: u16) {
    if channel >= state.config.channel_count {
        return;
    }
    let ch = channel as usize;
    let dev = state.config.channels[ch].device as usize;
    state.joints[ch].verified_timeout_reg = Some(timeout_reg);
    state.joints[ch].verified_timeout_session = Some(state.devices[dev].session_id);
}

fn handle_motion_command(
    state: &TorqueState,
    group: u8,
    channels: ChannelMask,
    actions: &mut ActionSet,
) {
    if group >= state.config.group_count {
        set_refusal_safe(actions, group, RefusalCode::GroupOutOfRange, None);
        return;
    }
    let gi = group as usize;
    let grp = state.groups[gi];
    let grp_cfg = state.config.groups[gi];

    if state.control_panic_latched || grp.drive_state == DriveState::SafeHold {
        actions.refusals[gi] = Some(RefusalEvent {
            group,
            code: RefusalCode::ControlPanicSafeHold,
            channel: None,
        });
        return;
    }
    if state.worker_panic_latched || grp.pending_park.is_some() {
        actions.refusals[gi] = Some(RefusalEvent {
            group,
            code: RefusalCode::InternalFault,
            channel: None,
        });
        return;
    }
    if grp.protective_latch || grp.drive_state.is_protective() {
        actions.refusals[gi] = Some(RefusalEvent {
            group,
            code: RefusalCode::ProtectiveLatched,
            channel: None,
        });
        return;
    }
    // B1: Fence motion while disable transaction OR unresolved wire dispatch exists.
    if grp.disable_tx.is_some() || grp.pending_disable_dispatch.is_some() {
        actions.refusals[gi] = Some(RefusalEvent {
            group,
            code: RefusalCode::DisableInProgress,
            channel: None,
        });
        return;
    }
    if !grp.faults.is_empty() {
        actions.refusals[gi] = Some(RefusalEvent {
            group,
            code: RefusalCode::FaultLatched,
            channel: None,
        });
        return;
    }
    if grp.drive_state != DriveState::OperationEnabled || grp.torque_intent != TorqueIntent::On {
        actions.refusals[gi] = Some(RefusalEvent {
            group,
            code: RefusalCode::NotEnabled,
            channel: None,
        });
        return;
    }
    if device_wide_mode_fenced(state, grp_cfg.devices) {
        actions.refusals[gi] = Some(RefusalEvent {
            group,
            code: RefusalCode::ModeUnverified,
            channel: None,
        });
        return;
    }

    let active = group_active_channels(state, group);
    let target = channels.intersect(active);
    let mut scales = [1.0f32; MAX_CHANNELS];
    for c in target.iter() {
        let rt = &state.joints[c as usize];
        if rt.verified_mode != Some(rt.desired_mode) {
            actions.refusals[gi] = Some(RefusalEvent {
                group,
                code: RefusalCode::ModeUnverified,
                channel: Some(c),
            });
            return;
        }
        scales[c as usize] = rt.thermal_derate_scale;
    }

    // M7: Carry authoritative per-channel thermal derate scales on MotionAllowed.
    actions.motion_allowed[gi] = Some(MotionAllowed {
        group,
        channels: target,
        generation: grp.stop_generation,
        thermal_derate_scales: scales,
    });
}

// =========================================================================
// DisableRequest, Stop, FaultClear
// =========================================================================

fn handle_disable_request(
    state: &mut TorqueState,
    group: u8,
    explicit_release_authorized: bool,
    recipient_mask_override: Option<ChannelMask>,
    now_us: u64,
    actions: &mut ActionSet,
) {
    if group >= state.config.group_count {
        set_refusal_safe(actions, group, RefusalCode::GroupOutOfRange, None);
        return;
    }
    let gi = group as usize;
    let grp = state.groups[gi];
    let grp_cfg = state.config.groups[gi];

    if state.control_panic_latched || grp.drive_state == DriveState::SafeHold {
        actions.refusals[gi] = Some(RefusalEvent {
            group,
            code: RefusalCode::ControlPanicSafeHold,
            channel: None,
        });
        return;
    }
    // M2: Fence ordinary external DisableRequest while Tier-1 worker panic is latched.
    if state.worker_panic_latched || grp.pending_park.is_some() {
        actions.refusals[gi] = Some(RefusalEvent {
            group,
            code: RefusalCode::InternalFault,
            channel: None,
        });
        return;
    }
    if grp.drive_state.is_protective() {
        actions.refusals[gi] = Some(RefusalEvent {
            group,
            code: RefusalCode::ProtectiveLatched,
            channel: None,
        });
        return;
    }
    if grp.drive_state == DriveState::ReadyToSwitchOn {
        enter_failed_hold(state, group, now_us, actions);
        return;
    }
    if grp.disable_tx.is_some() || grp.pending_disable_dispatch.is_some() {
        actions.refusals[gi] = Some(RefusalEvent {
            group,
            code: RefusalCode::DisableInProgress,
            channel: None,
        });
        return;
    }
    if grp.torque_intent == TorqueIntent::Off {
        return;
    }

    // Validate and expand recipient mask (B5: expanded mask MUST cover the full independently switchable group).
    let requested = recipient_mask_override.unwrap_or(grp_cfg.recipients);
    if requested.is_empty() || !requested.is_subset_of(grp_cfg.recipients) {
        actions.refusals[gi] = Some(RefusalEvent {
            group,
            code: RefusalCode::RecipientMaskInvalid,
            channel: None,
        });
        return;
    }
    let mut actual_recipients = ChannelMask::EMPTY;
    for r in requested.iter() {
        if r >= state.config.recipient_count {
            actions.refusals[gi] = Some(RefusalEvent {
                group,
                code: RefusalCode::RecipientMaskInvalid,
                channel: None,
            });
            return;
        }
        actual_recipients =
            actual_recipients.union(state.config.recipients[r as usize].grouped_packet_mask);
    }
    if actual_recipients != grp_cfg.recipients {
        actions.refusals[gi] = Some(RefusalEvent {
            group,
            code: RefusalCode::RecipientMaskInvalid,
            channel: None,
        });
        return;
    }

    // Immediate static/policy gates across all actual recipients.
    for r in actual_recipients.iter() {
        let rec = &state.config.recipients[r as usize];
        match rec.role {
            JointRole::LoadBearing if !rec.rest_evidence_qualified => {
                actions.refusals[gi] = Some(RefusalEvent {
                    group,
                    code: RefusalCode::RestEvidenceUnqualified,
                    channel: rec.channels.iter().next(),
                });
                return;
            }
            JointRole::Gripper(GripperReleasePolicy::ExplicitHand)
                if !explicit_release_authorized =>
            {
                actions.refusals[gi] = Some(RefusalEvent {
                    group,
                    code: RefusalCode::HandReleaseNotAuthorized,
                    channel: rec.channels.iter().next(),
                });
                return;
            }
            JointRole::ActiveTool(ActiveToolSafeOutput::BoundedRetention { .. })
                if !explicit_release_authorized =>
            {
                actions.refusals[gi] = Some(RefusalEvent {
                    group,
                    code: RefusalCode::ActiveToolReleaseNotAuthorized,
                    channel: rec.channels.iter().next(),
                });
                return;
            }
            _ => {}
        }
    }

    // M3: When operational, transition through the stop boundary and bump stop_generation to
    // invalidate any outstanding MotionAllowed capability before opening the rest gate!
    if state.groups[gi].drive_state == DriveState::OperationEnabled {
        state.groups[gi].stop_generation = state.groups[gi].stop_generation.saturating_add(1);
        state.groups[gi].drive_state = DriveState::QuickStopActive;
        capture_latest_for_stop(state, group, now_us);
    }

    open_rest_gated_disable(
        state,
        group,
        DisableOrigin::ExplicitRequest,
        actual_recipients,
        explicit_release_authorized,
        now_us,
    );
    try_authorize_rest_gate(state, group, now_us, actions);
}

fn open_rest_gated_disable(
    state: &mut TorqueState,
    group: u8,
    origin: DisableOrigin,
    actual_recipients: ChannelMask,
    explicit_release_authorized: bool,
    now_us: u64,
) {
    let gi = group as usize;
    let grp_cfg = state.config.groups[gi];
    let tx_id = state.alloc_capability_id();
    // Prune any stale rest windows at `now_us` while preserving valid continuously-accumulated
    // K=3 windows so already-stationary arms authorize immediately like live drivers.
    refresh_captures_and_windows(state, now_us);
    state.groups[gi].disable_tx = Some(DisableTransaction {
        tx_id,
        origin,
        phase: DisablePhase::AwaitRestGate,
        started_us: now_us,
        deadline_us: now_us.saturating_add(grp_cfg.disable_deadline_us),
        auth_generation: state.groups[gi].stop_generation,
        recipient_mask: actual_recipients,
        explicit_release_authorized,
        committed_off: false,
        repeats_completed: 0,
        next_repeat_at_us: 0,
        last_dispatched_release_id: None,
        sent_mask: ChannelMask::EMPTY,
        ambiguous_mask: ChannelMask::EMPTY,
        failed_mask: ChannelMask::EMPTY,
        deferred_mask: ChannelMask::EMPTY,
    });
}

fn handle_stop(
    state: &mut TorqueState,
    groups: GroupMask,
    kind: StopKind,
    now_us: u64,
    actions: &mut ActionSet,
) {
    for g in groups.iter() {
        if g >= state.config.group_count {
            continue;
        }
        let gi = g as usize;
        state.groups[gi].stop_generation = state.groups[gi].stop_generation.saturating_add(1);
        // B3: Cancel any pending park capability so late ParkSettled cannot open disable.
        state.groups[gi].pending_park = None;

        match kind {
            StopKind::Stop | StopKind::LeaseLoss => {}
            StopKind::EStop => state.groups[gi].faults.insert(TorqueFaultKind::EStop),
            StopKind::Watchdog => state.groups[gi]
                .faults
                .insert(TorqueFaultKind::WatchdogTimeout),
            StopKind::Fault(fk) => state.groups[gi].faults.insert(fk),
        }

        // Cancel uncommitted non-protective disable transaction (preserving pending wire dispatch for B1).
        if let Some(tx) = state.groups[gi].disable_tx {
            if !tx.origin.is_protective() && !tx.committed_off {
                abort_non_protective_tx(state, g, RefusalCode::PreemptedByStop, false, actions);
            }
        }

        if state.groups[gi].drive_state == DriveState::ReadyToSwitchOn {
            state.groups[gi].drive_state = DriveState::FailedHold;
            state.groups[gi].enable_phase = EnablePhase::FailedHold;
            state.groups[gi].enable_tx = None;
            state.groups[gi].torque_intent = TorqueIntent::On;
            state.groups[gi].faults.insert(TorqueFaultKind::EnableFailed);
            anchor_group_captures(state, g, now_us);
            actions.refusals[gi] = Some(RefusalEvent {
                group: g,
                code: RefusalCode::EnableFailed,
                channel: None,
            });
            continue;
        }

        if state.groups[gi].drive_state.is_protective()
            || state.groups[gi].drive_state == DriveState::SafeHold
        {
            continue;
        }

        if state.groups[gi].drive_state == DriveState::OperationEnabled {
            capture_latest_for_stop(state, g, now_us);
        } else if state.groups[gi].torque_intent == TorqueIntent::On {
            anchor_group_captures(state, g, now_us);
        }

        match kind {
            StopKind::Stop | StopKind::LeaseLoss => {
                if state.groups[gi].torque_intent == TorqueIntent::On
                    && !matches!(
                        state.groups[gi].drive_state,
                        DriveState::Fault | DriveState::FailedHold
                    )
                {
                    state.groups[gi].drive_state = if state.groups[gi].faults.is_empty() {
                        DriveState::QuickStopActive
                    } else {
                        DriveState::Fault
                    };
                }
            }
            StopKind::EStop | StopKind::Watchdog | StopKind::Fault(_) => {
                if state.groups[gi].drive_state != DriveState::FailedHold {
                    state.groups[gi].drive_state = DriveState::Fault;
                }
            }
        }
    }
}

fn handle_fault_clear(
    state: &mut TorqueState,
    groups: GroupMask,
    _now_us: u64,
    actions: &mut ActionSet,
) {
    // INVARIANT: FaultClear NEVER dispatches a disable and NEVER changes TorqueIntent.
    for g in groups.iter() {
        if g >= state.config.group_count {
            continue;
        }
        let gi = g as usize;
        if state.control_panic_latched || state.groups[gi].drive_state == DriveState::SafeHold {
            actions.refusals[gi] = Some(RefusalEvent {
                group: g,
                code: RefusalCode::ControlPanicSafeHold,
                channel: None,
            });
            continue;
        }
        if state.worker_panic_latched {
            actions.refusals[gi] = Some(RefusalEvent {
                group: g,
                code: RefusalCode::InternalFault,
                channel: None,
            });
            continue;
        }
        if !state.groups[gi].thermal_tripped_mask.is_empty() {
            actions.refusals[gi] = Some(RefusalEvent {
                group: g,
                code: RefusalCode::ThermalActive,
                channel: None,
            });
            continue;
        }
        if state.groups[gi].disable_tx.is_some()
            || state.groups[gi].pending_disable_dispatch.is_some()
        {
            actions.refusals[gi] = Some(RefusalEvent {
                group: g,
                code: RefusalCode::DisableInProgress,
                channel: None,
            });
            continue;
        }

        state.groups[gi].faults = TorqueFaultMask::EMPTY;
        state.groups[gi].protective_latch = false;
        state.groups[gi].thermal_trip_release_id = None;
        state.groups[gi].pending_park = None;
        if state.groups[gi].enable_phase == EnablePhase::FailedHold {
            state.groups[gi].enable_phase = EnablePhase::Idle;
        }

        if state.groups[gi].torque_intent == TorqueIntent::Off {
            state.groups[gi].drive_state = DriveState::SwitchOnDisabled;
        } else if matches!(
            state.groups[gi].drive_state,
            DriveState::Fault
                | DriveState::FailedHold
                | DriveState::FaultReactionActive
                | DriveState::ProtectiveDisabled
                | DriveState::ProtectiveDisabling
                | DriveState::ProtectiveDamping
        ) {
            state.groups[gi].drive_state = DriveState::QuickStopActive;
        }
    }
}

fn latch_non_protective_fault(
    state: &mut TorqueState,
    g: u8,
    fault: TorqueFaultKind,
    now_us: u64,
    actions: &mut ActionSet,
) {
    let gi = g as usize;
    state.groups[gi].faults.insert(fault);
    state.groups[gi].stop_generation = state.groups[gi].stop_generation.saturating_add(1);
    state.groups[gi].pending_park = None;

    if let Some(tx) = state.groups[gi].disable_tx {
        if !tx.origin.is_protective() && !tx.committed_off {
            abort_non_protective_tx(state, g, RefusalCode::PreemptedByStop, false, actions);
        }
    }

    if state.groups[gi].drive_state == DriveState::ReadyToSwitchOn {
        enter_failed_hold(state, g, now_us, actions);
        return;
    }
    if state.groups[gi].drive_state.is_protective()
        || state.groups[gi].drive_state == DriveState::SafeHold
    {
        return;
    }
    if state.groups[gi].drive_state == DriveState::OperationEnabled {
        capture_latest_for_stop(state, g, now_us);
    }
    if state.groups[gi].drive_state != DriveState::FailedHold {
        state.groups[gi].drive_state = DriveState::Fault;
    }
}

// =========================================================================
// Thermal, Damping, Protective Disable
// =========================================================================

fn apply_thermal(
    state: &mut TorqueState,
    channel: u8,
    temperature_c: f32,
    sample_ts_us: u64,
    now_us: u64,
    actions: &mut ActionSet,
) {
    if channel >= state.config.channel_count || !temperature_c.is_finite() {
        return;
    }
    let ch = channel as usize;
    // M6: Reject out-of-order older temperature readings so a delayed cool sample cannot clear hot state.
    if state.joints[ch]
        .last_temperature_us
        .is_some_and(|prev_ts| sample_ts_us < prev_ts)
    {
        return;
    }

    let ch_cfg = state.config.channels[ch];
    let g = ch_cfg.group;
    let gi = g as usize;

    state.joints[ch].last_temperature_c = Some(temperature_c);
    state.joints[ch].last_temperature_us = Some(sample_ts_us);
    let scale = compute_thermal_derate_scale(temperature_c, ch_cfg.thermal);
    state.joints[ch].thermal_derate_scale = scale;

    if temperature_c >= ch_cfg.thermal.trip_c {
        state.groups[gi].thermal_tripped_mask.insert(channel);
        trigger_protective_disable(state, g, DisableOrigin::ProtectiveThermal, now_us, actions);
    } else {
        state.groups[gi].thermal_tripped_mask.remove(channel);
    }
}

fn trigger_protective_disable(
    state: &mut TorqueState,
    g: u8,
    origin: DisableOrigin,
    now_us: u64,
    actions: &mut ActionSet,
) {
    let gi = g as usize;
    let grp_cfg = state.config.groups[gi];

    state.groups[gi].protective_latch = true;
    state.groups[gi].pending_park = None;
    if origin == DisableOrigin::ProtectiveThermal {
        state.groups[gi].faults.insert(TorqueFaultKind::ThermalTrip);
        state.groups[gi]
            .thermal_trip_release_id
            .get_or_insert(state.current_release_id);
    }

    let interrupted_enable = state.groups[gi].drive_state == DriveState::ReadyToSwitchOn;
    if interrupted_enable {
        state.groups[gi].enable_tx = None;
        state.groups[gi].enable_phase = EnablePhase::FailedHold;
        state.groups[gi].faults.insert(TorqueFaultKind::EnableFailed);
        actions.refusals[gi] = Some(RefusalEvent {
            group: g,
            code: RefusalCode::EnableFailed,
            channel: None,
        });
    }

    // B10: Only short-circuit to ProtectiveDisabled when BOTH TorqueIntent is Off AND every
    // physical recipient is already confirmed Sent off (no deferred/ambiguous recipients left!).
    if !group_has_unresolved_off(state, g) {
        if state.groups[gi].disable_tx.is_none() {
            state.groups[gi].drive_state = DriveState::ProtectiveDisabled;
        }
        return;
    }

    // Do not reset deadline or extend damping if a protective transaction is already active.
    if state.groups[gi]
        .disable_tx
        .is_some_and(|tx| tx.origin.is_protective())
    {
        return;
    }

    // Preempt any non-protective disable transaction (even after partial dispatch committed Off, B10!).
    if let Some(old_tx) = state.groups[gi].disable_tx.take() {
        if let Some(pend) = state.groups[gi].pending_disable_dispatch.as_mut() {
            if pend.tx_id == old_tx.tx_id {
                pend.tx_still_active = false;
            }
        }
        if !interrupted_enable {
            actions.refusals[gi] = Some(RefusalEvent {
                group: g,
                code: RefusalCode::PreemptedByProtective,
                channel: None,
            });
        }
        actions.disable_completions[gi] = Some(build_completion(g, &old_tx, false));
    }

    state.groups[gi].stop_generation = state.groups[gi].stop_generation.saturating_add(1);
    let gen = state.groups[gi].stop_generation;
    let tx_id = state.alloc_capability_id();

    // Target either full group (if Intent::On) or all recipients not yet confirmed Sent off (B10).
    let target_recipients = if state.groups[gi].torque_intent == TorqueIntent::On {
        grp_cfg.recipients
    } else {
        let unresolved = grp_cfg
            .recipients
            .difference(state.groups[gi].physically_sent_off_mask);
        if unresolved.is_empty() {
            grp_cfg.recipients
        } else {
            // Expand unresolved through grouped_packet_mask so packet coverage stays complete.
            let mut expanded = ChannelMask::EMPTY;
            for r in unresolved.iter() {
                expanded = expanded.union(state.config.recipients[r as usize].grouped_packet_mask);
            }
            expanded
        }
    };

    // Determine whether MIT kd-only damping precedes DisableProtective (M8: per-channel durations).
    let mut dampable_mask = ChannelMask::EMPTY;
    let mut max_damp_us = 0u64;
    if state.groups[gi].torque_intent == TorqueIntent::On
        && matches!(
            origin,
            DisableOrigin::ProtectiveThermal | DisableOrigin::ProtectiveProcessExit
        )
    {
        let active = group_active_channels(state, g);
        for c in active.iter() {
            let ch_cfg = state.config.channels[c as usize];
            if ch_cfg.damping.duration_us > 0
                && ch_cfg.supports_mit_damping
                && state.joints[c as usize].verified_mode == Some(ControlMode::Impedance)
            {
                dampable_mask.insert(c);
                max_damp_us = max_damp_us.max(ch_cfg.damping.duration_us);
            }
        }
    }

    if !dampable_mask.is_empty() && max_damp_us > 0 {
        let max_end_us = now_us.saturating_add(max_damp_us);
        let deadline_us = max_end_us.saturating_add(grp_cfg.disable_deadline_us);
        state.groups[gi].drive_state = DriveState::ProtectiveDamping;
        state.groups[gi].disable_tx = Some(DisableTransaction {
            tx_id,
            origin,
            phase: DisablePhase::ProtectiveDamping {
                started_us: now_us,
                max_end_us,
                dampable_mask,
            },
            started_us: now_us,
            deadline_us,
            auth_generation: gen,
            recipient_mask: target_recipients,
            explicit_release_authorized: false,
            committed_off: state.groups[gi].torque_intent == TorqueIntent::Off,
            repeats_completed: 0,
            next_repeat_at_us: 0,
            last_dispatched_release_id: None,
            sent_mask: ChannelMask::EMPTY,
            ambiguous_mask: ChannelMask::EMPTY,
            failed_mask: ChannelMask::EMPTY,
            deferred_mask: ChannelMask::EMPTY,
        });
        emit_damping_commands(state, g, dampable_mask, now_us, actions);
    } else {
        let deadline_us = now_us.saturating_add(grp_cfg.disable_deadline_us);
        state.groups[gi].drive_state = DriveState::ProtectiveDisabling;
        state.groups[gi].disable_tx = Some(DisableTransaction {
            tx_id,
            origin,
            phase: DisablePhase::ProtectivePendingDispatch,
            started_us: now_us,
            deadline_us,
            auth_generation: gen,
            recipient_mask: target_recipients,
            explicit_release_authorized: false,
            committed_off: state.groups[gi].torque_intent == TorqueIntent::Off,
            repeats_completed: 0,
            next_repeat_at_us: 0,
            last_dispatched_release_id: None,
            sent_mask: ChannelMask::EMPTY,
            ambiguous_mask: ChannelMask::EMPTY,
            failed_mask: ChannelMask::EMPTY,
            deferred_mask: ChannelMask::EMPTY,
        });
        emit_disable_attempt(state, g, DisableKind::Protective, now_us, actions);
    }
}

// =========================================================================
// Serial Timeout, Grasp Update, Park, FU-PANIC, ProcessExit, BusLoss
// =========================================================================

fn handle_serial_timeout(state: &mut TorqueState, device: u8, now_us: u64) {
    if device >= state.config.device_count {
        return;
    }
    let d = device as usize;
    let silence = state.config.devices[d].poison_silence_us;
    if silence > 0 {
        let until = now_us.saturating_add(silence);
        state.devices[d].poisoned_until_us = state.devices[d].poisoned_until_us.max(until);
    }
    for r in 0..state.config.recipient_count {
        if state.config.recipients[r as usize].device == device {
            reset_recipient_window(state, r);
            // B9: Serial timeout invalidates unconsumed AuthorizedForRelease.
            let g = state.config.recipients[r as usize].group;
            revoke_unconsumed_rest_auth(state, g);
        }
    }
}

fn handle_grasp_state_update(
    state: &mut TorqueState,
    channel: u8,
    setpoint_rad: f32,
    force_limit_raw: Option<u16>,
) {
    if channel >= state.config.channel_count || !setpoint_rad.is_finite() {
        return;
    }
    let ch = channel as usize;
    let ch_cfg = state.config.channels[ch];
    if !matches!(ch_cfg.role, JointRole::Gripper(_)) {
        return;
    }
    let grp = state.groups[ch_cfg.group as usize];
    if grp.drive_state != DriveState::OperationEnabled {
        return;
    }
    state.joints[ch].grasp_setpoint_rad = Some(setpoint_rad);
    state.joints[ch].grasp_force_limit_raw = force_limit_raw;
    // B6: Changing the grasp invalidates any prior release evidence.
    state.joints[ch].last_release_evidence = ReleaseEvidence::Unknown;
}

fn handle_park_accepted(state: &mut TorqueState, group: u8, park_id: u64) {
    if group >= state.config.group_count {
        return;
    }
    let gi = group as usize;
    if let Some(p) = state.groups[gi].pending_park.as_mut() {
        if p.park_id == park_id && p.generation == state.groups[gi].stop_generation {
            p.accepted = true;
        }
    }
}

fn handle_park_settled(
    state: &mut TorqueState,
    group: u8,
    park_id: u64,
    succeeded: bool,
    now_us: u64,
) {
    if group >= state.config.group_count {
        return;
    }
    let gi = group as usize;

    // B3: Accept and consume a park outcome ONLY for the currently live pending park_id!
    // Unsolicited or Stop-superseded ParkSettled leaves park & torque policy completely unchanged.
    let Some(pending) = state.groups[gi].pending_park else {
        return;
    };
    if pending.park_id != park_id {
        return;
    }
    state.groups[gi].pending_park = None;

    if state.groups[gi].drive_state.is_protective()
        || state.groups[gi].drive_state == DriveState::SafeHold
    {
        return;
    }

    if !succeeded {
        state.groups[gi].faults.insert(TorqueFaultKind::Other);
        if state.groups[gi].drive_state != DriveState::FailedHold {
            state.groups[gi].drive_state = DriveState::Fault;
        }
        anchor_group_captures(state, group, now_us);
        return;
    }

    // Park succeeded on this group ONLY (never authorizes releasing any separate hand group!).
    if state.groups[gi].torque_intent == TorqueIntent::On
        && state.groups[gi].disable_tx.is_none()
        && state.groups[gi].pending_disable_dispatch.is_none()
    {
        if state.groups[gi].drive_state == DriveState::FaultReactionActive
            || state.groups[gi].drive_state == DriveState::OperationEnabled
        {
            state.groups[gi].drive_state = if state.groups[gi].faults.is_empty() {
                DriveState::QuickStopActive
            } else {
                DriveState::Fault
            };
        }
        capture_latest_for_stop(state, group, now_us);

        let grp_cfg = state.config.groups[gi];
        let has_explicit_hand_or_retention = grp_cfg.recipients.iter().any(|r| {
            matches!(
                state.config.recipients[r as usize].role,
                JointRole::Gripper(GripperReleasePolicy::ExplicitHand)
                    | JointRole::ActiveTool(ActiveToolSafeOutput::BoundedRetention { .. })
            )
        });
        let has_unqualified = grp_cfg.recipients.iter().any(|r| {
            let rec = &state.config.recipients[r as usize];
            matches!(rec.role, JointRole::LoadBearing) && !rec.rest_evidence_qualified
        });
        if !has_explicit_hand_or_retention && !has_unqualified && !grp_cfg.recipients.is_empty() {
            open_rest_gated_disable(
                state,
                group,
                DisableOrigin::WorkerPanicParkAtRest,
                grp_cfg.recipients,
                false,
                now_us,
            );
        }
    }
}

fn handle_panic(state: &mut TorqueState, tier: PanicTier, now_us: u64, actions: &mut ActionSet) {
    match tier {
        PanicTier::WorkerPanic { thread_tag } => {
            state.worker_panic_latched = true;
            state.worker_panic_thread_tag = Some(thread_tag);
            for g in 0..state.config.group_count {
                let gi = g as usize;
                state.groups[gi].faults.insert(TorqueFaultKind::WorkerPanic);
                state.groups[gi].stop_generation =
                    state.groups[gi].stop_generation.saturating_add(1);

                if let Some(tx) = state.groups[gi].disable_tx {
                    if !tx.origin.is_protective() && !tx.committed_off {
                        abort_non_protective_tx(
                            state,
                            g,
                            RefusalCode::PreemptedByStop,
                            false,
                            actions,
                        );
                    }
                }

                if state.groups[gi].drive_state == DriveState::ReadyToSwitchOn {
                    enter_failed_hold(state, g, now_us, actions);
                    continue;
                }
                if state.groups[gi].drive_state.is_protective()
                    || state.groups[gi].drive_state == DriveState::SafeHold
                {
                    continue;
                }

                if state.groups[gi].torque_intent == TorqueIntent::On {
                    capture_latest_for_stop(state, g, now_us);
                    let grp_cfg = state.config.groups[gi];
                    if grp_cfg.has_park_path
                        && state.groups[gi].drive_state != DriveState::FailedHold
                    {
                        let park_id = state.alloc_capability_id();
                        let gen = state.groups[gi].stop_generation;
                        state.groups[gi].drive_state = DriveState::FaultReactionActive;
                        state.groups[gi].pending_park = Some(PendingPark {
                            park_id,
                            generation: gen,
                            accepted: false,
                        });
                        actions.park_requests[gi] = Some(ParkRequest {
                            group: g,
                            park_id,
                            generation: gen,
                        });
                    } else if state.groups[gi].drive_state != DriveState::FailedHold {
                        state.groups[gi].drive_state = DriveState::Fault;
                    }
                } else {
                    state.groups[gi].drive_state = DriveState::Fault;
                }
            }
        }
        PanicTier::CaughtControlPanic => {
            state.control_panic_latched = true;
            for g in 0..state.config.group_count {
                let gi = g as usize;
                state.groups[gi].faults.insert(TorqueFaultKind::ControlPanic);
                state.groups[gi].pending_park = None;

                // B11: Repeated CaughtControlPanic is idempotent on already-SafeHold groups
                // (preserves active per-device bus-loss timers!).
                if state.groups[gi].drive_state == DriveState::SafeHold
                    || state.groups[gi].drive_state.is_protective()
                {
                    continue;
                }

                state.groups[gi].stop_generation =
                    state.groups[gi].stop_generation.saturating_add(1);

                if let Some(tx) = state.groups[gi].disable_tx {
                    if !tx.origin.is_protective() && !tx.committed_off {
                        abort_non_protective_tx(
                            state,
                            g,
                            RefusalCode::ControlPanicSafeHold,
                            false,
                            actions,
                        );
                    }
                }

                if state.groups[gi].drive_state == DriveState::ReadyToSwitchOn {
                    state.groups[gi].enable_tx = None;
                    state.groups[gi].enable_phase = EnablePhase::FailedHold;
                    state.groups[gi].faults.insert(TorqueFaultKind::EnableFailed);
                    actions.refusals[gi] = Some(RefusalEvent {
                        group: g,
                        code: RefusalCode::EnableFailed,
                        channel: None,
                    });
                }

                if state.groups[gi].torque_intent == TorqueIntent::On {
                    capture_latest_for_stop(state, g, now_us);
                    state.groups[gi].drive_state = DriveState::SafeHold;
                } else {
                    state.groups[gi].drive_state = DriveState::Fault;
                }
            }
        }
        PanicTier::ProcessDeath => {
            state.process_death_recorded = true;
        }
    }
}

fn handle_process_exit(state: &mut TorqueState, now_us: u64, actions: &mut ActionSet) {
    for g in 0..state.config.group_count {
        trigger_protective_disable(
            state,
            g,
            DisableOrigin::ProtectiveProcessExit,
            now_us,
            actions,
        );
    }
}

fn handle_bus_loss(state: &mut TorqueState, device: u8, now_us: u64, actions: &mut ActionSet) {
    if device >= state.config.device_count {
        return;
    }
    // M11: Advance device session_id so cached Damiao TIMEOUT readbacks are invalidated.
    state.devices[device as usize].session_id =
        state.devices[device as usize].session_id.saturating_add(1);

    // M12: Invalidate ALL measurement provenance (captures + last_sample_us + last_measured_pos_rad
    // + release evidence) on this device so a subsequent stop cannot recapture pre-loss positions.
    for c in 0..state.config.channel_count {
        if state.config.channels[c as usize].device == device {
            let rt = &mut state.joints[c as usize];
            rt.captured_position_rad = None;
            rt.captured_at_us = None;
            rt.last_sample_us = None;
            rt.last_measured_pos_rad = None;
            rt.last_measured_vel_rad_s = None;
            rt.last_release_evidence = ReleaseEvidence::Unknown;
            rt.verified_timeout_session = None;
            rt.had_valid_capture = false;
        }
    }
    for r in 0..state.config.recipient_count {
        if state.config.recipients[r as usize].device == device {
            reset_recipient_window(state, r);
        }
    }

    // Ordinary bus loss grants NO protective authority!
    for g in 0..state.config.group_count {
        let gi = g as usize;
        if !state.config.groups[gi].devices.contains(device) {
            continue;
        }
        state.groups[gi].faults.insert(TorqueFaultKind::BusLoss);
        state.groups[gi].stop_generation = state.groups[gi].stop_generation.saturating_add(1);
        state.groups[gi].pending_park = None;

        if state.groups[gi].drive_state == DriveState::SafeHold {
            state.devices[device as usize]
                .safe_hold_bus_loss_since_us
                .get_or_insert(now_us);
            continue;
        }
        if state.groups[gi].drive_state.is_protective() {
            continue;
        }
        if state.groups[gi].drive_state == DriveState::ReadyToSwitchOn {
            enter_failed_hold(state, g, now_us, actions);
            continue;
        }
        // M12: Abort any uncommitted non-protective disable via common completion/refusal path,
        // preserving unresolved pending_disable_dispatch identity!
        if let Some(tx) = state.groups[gi].disable_tx {
            if !tx.origin.is_protective() && !tx.committed_off {
                abort_non_protective_tx(state, g, RefusalCode::PreemptedByStop, false, actions);
            }
        }
        if state.groups[gi].drive_state != DriveState::FailedHold {
            state.groups[gi].drive_state = DriveState::Fault;
        }
    }
}

// =========================================================================
// Helpers: Transaction Completion, Refusal, Telemetry & Shared Bus Budgets
// =========================================================================

fn build_completion(g: u8, tx: &DisableTransaction, timed_out: bool) -> DisableCompletion {
    let covered = tx.sent_mask.union(tx.ambiguous_mask);
    let unsent = tx.recipient_mask.difference(covered);
    let partial = tx.sent_mask != tx.recipient_mask
        || !tx.ambiguous_mask.is_empty()
        || !tx.failed_mask.is_empty()
        || !unsent.is_empty();
    DisableCompletion {
        group: g,
        tx_id: tx.tx_id,
        origin: tx.origin,
        recipient_mask: tx.recipient_mask,
        sent_mask: tx.sent_mask,
        ambiguous_mask: tx.ambiguous_mask,
        failed_mask: tx.failed_mask,
        unsent_mask: unsent,
        committed_off: tx.committed_off,
        partial_or_ambiguous: partial,
        timed_out,
        repeats_completed: tx.repeats_completed,
    }
}

fn finish_disable_tx(
    state: &mut TorqueState,
    g: u8,
    timed_out: bool,
    actions: &mut ActionSet,
) {
    let gi = g as usize;
    let Some(tx) = state.groups[gi].disable_tx.take() else {
        return;
    };
    if let Some(pend) = state.groups[gi].pending_disable_dispatch.as_mut() {
        if pend.tx_id == tx.tx_id {
            pend.tx_still_active = false;
        }
    }
    let is_protective = tx.origin.is_protective();
    actions.disable_completions[gi] = Some(build_completion(g, &tx, timed_out));
    if tx.committed_off {
        update_post_off_drive_state(state, g, is_protective);
    }
}

fn update_post_off_drive_state(state: &mut TorqueState, g: u8, is_protective: bool) {
    let gi = g as usize;
    let active = group_active_channels(state, g);
    for c in active.iter() {
        state.joints[c as usize].had_valid_capture = false;
    }
    if is_protective || state.groups[gi].protective_latch {
        state.groups[gi].drive_state = DriveState::ProtectiveDisabled;
    } else if !state.groups[gi].faults.is_empty() {
        if state.groups[gi].drive_state != DriveState::FailedHold {
            state.groups[gi].drive_state = DriveState::Fault;
        }
    } else {
        state.groups[gi].drive_state = DriveState::SwitchOnDisabled;
        state.groups[gi].enable_phase = EnablePhase::Idle;
    }
}

fn abort_non_protective_tx(
    state: &mut TorqueState,
    g: u8,
    code: RefusalCode,
    timed_out: bool,
    actions: &mut ActionSet,
) {
    abort_non_protective_tx_with_ch(state, g, code, None, timed_out, actions);
}

fn abort_non_protective_tx_with_ch(
    state: &mut TorqueState,
    g: u8,
    code: RefusalCode,
    channel: Option<u8>,
    timed_out: bool,
    actions: &mut ActionSet,
) {
    let gi = g as usize;
    if let Some(tx) = state.groups[gi].disable_tx.take() {
        if let Some(pend) = state.groups[gi].pending_disable_dispatch.as_mut() {
            if pend.tx_id == tx.tx_id {
                pend.tx_still_active = false;
            }
        }
        actions.disable_completions[gi] = Some(build_completion(g, &tx, timed_out));
    }
    actions.refusals[gi] = Some(RefusalEvent {
        group: g,
        code,
        channel,
    });
}

fn rest_timeout_code(state: &TorqueState, tx: &DisableTransaction) -> RefusalCode {
    for r in tx.recipient_mask.iter() {
        let rec = &state.config.recipients[r as usize];
        if matches!(
            rec.role,
            JointRole::Gripper(GripperReleasePolicy::ExplicitHand)
                | JointRole::ActiveTool(ActiveToolSafeOutput::BoundedRetention { .. })
        ) {
            for c in rec.channels.iter() {
                if state.joints[c as usize].last_release_evidence == ReleaseEvidence::Unknown {
                    return RefusalCode::ReleaseEvidenceTimeout;
                }
            }
        }
    }
    RefusalCode::RestGateTimeout
}

fn set_refusal_safe(
    actions: &mut ActionSet,
    group: u8,
    code: RefusalCode,
    channel: Option<u8>,
) {
    // m2: Store out-of-range group refusals in the dedicated slot so visit_by_priority never drops them.
    actions.invalid_group_refusal = Some(RefusalEvent {
        group,
        code,
        channel,
    });
}

fn populate_telemetry(state: &TorqueState, now_us: u64, actions: &mut ActionSet) {
    actions.telemetry.now_us = now_us;
    actions.telemetry.release_id = state.current_release_id;

    for g in 0..state.config.group_count {
        let gi = g as usize;
        actions.telemetry.group_states[gi] = state.groups[gi].drive_state;
        actions.telemetry.group_intents[gi] = state.groups[gi].torque_intent;
        actions.telemetry.thermal_tripped_mask = actions
            .telemetry
            .thermal_tripped_mask
            .union(state.groups[gi].thermal_tripped_mask);
    }

    for c in 0..state.config.channel_count {
        let ci = c as usize;
        let rt = &state.joints[ci];
        if rt.reported_enabled == Some(true) {
            actions.telemetry.reported_enabled_mask.insert(c);
        }
        if !matches!(state.config.channels[ci].role, JointRole::PassiveSensor)
            && rt.verified_mode != Some(rt.desired_mode)
        {
            actions.telemetry.mode_unverified_mask.insert(c);
        }
        actions.telemetry.thermal_derate_scales[ci] = rt.thermal_derate_scale;
    }

    // M14: Single authoritative shared-bus quota and packet-accurate demand accounting.
    let mut budgets = [BusBudgetSummary::default(); MAX_DEVICES];
    for d in 0..state.config.device_count {
        let dev = state.config.devices[d as usize];
        let b = dev.bus_id as usize;
        if b < MAX_DEVICES {
            budgets[b].normal_quota = dev.tx_quota_per_release;
            budgets[b].reserved_safety_quota = dev.reserved_safety_tx;
        }
    }

    for c in 0..state.config.channel_count {
        let b = state.config.devices[state.config.channels[c as usize].device as usize].bus_id
            as usize;
        if b >= MAX_DEVICES {
            continue;
        }
        if actions.damps[c as usize].is_some() {
            budgets[b].protective_demand = budgets[b].protective_demand.saturating_add(1);
        }
        if actions.holds[c as usize].is_some() {
            budgets[b].stop_hold_demand = budgets[b].stop_hold_demand.saturating_add(1);
        }
        if actions.refresh_requests.contains(c) {
            budgets[b].refresh_demand = budgets[b].refresh_demand.saturating_add(1);
        }
    }

    // Charge disable demand ONCE per distinct grouped packet on each bus (M14).
    for g in 0..state.config.group_count {
        if let Some(dcmd) = actions.disables[g as usize] {
            let mut remaining = dcmd.recipients;
            for r in dcmd.recipients.iter() {
                if !remaining.contains(r) || r >= state.config.recipient_count {
                    continue;
                }
                let rec = state.config.recipients[r as usize];
                remaining = remaining.difference(rec.grouped_packet_mask);
                let b = state.config.devices[rec.device as usize].bus_id as usize;
                if b < MAX_DEVICES {
                    if matches!(dcmd.kind, DisableKind::Protective) {
                        budgets[b].protective_demand =
                            budgets[b].protective_demand.saturating_add(1);
                    } else {
                        budgets[b].stop_hold_demand =
                            budgets[b].stop_hold_demand.saturating_add(1);
                    }
                }
            }
        }
    }

    actions.telemetry.bus_budgets = budgets;
}
