//! Pure `TorqueController` policy reducer: `step(&mut TorqueState, Event) -> ActionSet`.
//!
//! Fixed-capacity arrays and bitsets, integer monotonic time (µs), zero heap allocation,
//! zero I/O, zero locks.

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
        Event::CausalSample(sample) => handle_causal_sample(state, sample, now_us, &mut actions),
        Event::WriteResult(wr) => handle_write_result(state, wr, now_us, &mut actions),
        Event::HandshakeResult {
            group,
            now_us: _,
            confirmed_mask,
            outcome,
        } => handle_handshake_result(state, group, confirmed_mask, outcome, now_us, &mut actions),
        Event::EnableStart { group, now_us: _ } => {
            handle_enable_start(state, group, now_us, &mut actions)
        }
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
        } => apply_thermal(state, channel, temperature_c, now_us, &mut actions),
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
        Event::ParkSettled {
            group,
            succeeded,
            now_us: _,
        } => handle_park_settled(state, group, succeeded, now_us),
        Event::Panic { tier, now_us: _ } => handle_panic(state, tier, now_us, &mut actions),
        Event::ProcessExit { now_us: _ } => handle_process_exit(state, now_us, &mut actions),
        Event::BusLoss { device, now_us: _ } => handle_bus_loss(state, device, now_us),
    }

    populate_telemetry(state, now_us, &mut actions);
    actions
}

/// Extract the monotonic timestamp carried by an [`Event`].
#[inline]
pub const fn event_now_us(event: &Event) -> u64 {
    match *event {
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
        | Event::BusLoss { now_us, .. } => now_us,
        Event::CausalSample(s) => s.now_us,
        Event::WriteResult(w) => w.now_us,
    }
}

/// Linear thermal derating multiplier in `0.0..=1.0`.
///
/// - When `thermal.derate_start_c == None`: returns `1.0` for all `temp_c < trip_c` (default).
/// - When `thermal.derate_start_c == Some(start)`: scales linearly from `1.0` at `<= start`
///   down to `0.0` at `>= trip_c`.
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

/// Returns true if any device in `devices` has `device_wide_mode_switch == true` and at least one
/// non-passive channel on that device currently has `verified_mode != Some(desired_mode)`.
pub(crate) fn device_wide_mode_fenced(state: &TorqueState, devices: DeviceMask) -> bool {
    for c in 0..state.config.channel_count {
        let ch = &state.config.channels[c as usize];
        if matches!(ch.role, JointRole::PassiveSensor) {
            continue;
        }
        if devices.contains(ch.device) && state.config.devices[ch.device as usize].device_wide_mode_switch
        {
            let rt = &state.joints[c as usize];
            if rt.verified_mode != Some(rt.desired_mode) {
                return true;
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

/// Drop stale captures (`> max_capture_age_us`) and stale rest windows.
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
        let win = &mut state.rest_windows[r as usize];
        if !is_sample_fresh(win.last_ts_us, now_us, min_age) {
            win.reset();
        }
    }
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

/// Anchor captures for every active channel in `group` when entering a stopped / hold state.
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

/// Refresh captures directly from the latest fresh measured position when leaving operational motion.
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

// =========================================================================
// Event::Release
// =========================================================================

fn handle_release(state: &mut TorqueState, now_us: u64, actions: &mut ActionSet) {
    state.current_release_id = state.current_release_id.saturating_add(1);
    refresh_captures_and_windows(state, now_us);

    for g in 0..state.config.group_count {
        let gi = g as usize;

        // 1. Protective states preempt everything else on this group.
        if state.groups[gi].drive_state == DriveState::ProtectiveDamping {
            step_protective_damping_on_release(state, g, now_us, actions);
            continue;
        }
        if state.groups[gi].drive_state == DriveState::ProtectiveDisabling {
            step_protective_disabling_on_release(state, g, now_us, actions);
            continue;
        }
        if state.groups[gi].drive_state == DriveState::ProtectiveDisabled {
            // If thermal re-tripped while torque somehow became On (defensive), keep protective.
            continue;
        }

        // 2. Tier-2 SafeHold: check continuous bus-loss limit before holds.
        if state.groups[gi].drive_state == DriveState::SafeHold
            && state.groups[gi].torque_intent == TorqueIntent::On
            && state.groups[gi].disable_tx.is_none()
            && check_safe_hold_bus_loss_trip(state, g, now_us, actions)
        {
            continue;
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
            step_non_protective_disable_on_release(state, g, now_us, actions);
        }

        // 5. Hold emission for stopped / FailedHold / SafeHold groups with TorqueIntent::On.
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
        // Defensive recovery if damping state exists without tx
        if state.groups[gi].torque_intent == TorqueIntent::On {
            trigger_protective_disable(state, g, DisableOrigin::ProtectiveThermal, now_us, actions);
        }
        return;
    };

    let DisablePhase::ProtectiveDamping {
        end_us,
        dampable_mask,
        ..
    } = tx.phase
    else {
        return;
    };

    // Only keep damping channels whose verified mode still supports MIT damping.
    let mut still_dampable = ChannelMask::EMPTY;
    for c in dampable_mask.iter() {
        let ch_cfg = state.config.channels[c as usize];
        if ch_cfg.damping.duration_us > 0
            && ch_cfg.supports_mit_damping
            && state.joints[c as usize].verified_mode == Some(ControlMode::Impedance)
        {
            still_dampable.insert(c);
        }
    }

    if now_us >= end_us || still_dampable.is_empty() {
        // Damping finished (or verified MIT mode lost): advance immediately to DisableProtective.
        state.groups[gi].drive_state = DriveState::ProtectiveDisabling;
        let gen = state.groups[gi].stop_generation;
        if let Some(tx_mut) = state.groups[gi].disable_tx.as_mut() {
            tx_mut.phase = DisablePhase::ProtectivePendingDispatch;
            tx_mut.auth_generation = gen;
            tx_mut.awaiting_write_result = true;
            tx_mut.last_dispatched_release_id = Some(state.current_release_id);
            state.groups[gi].last_authorized_disable_gen = Some(gen);
            actions.disables[gi] = Some(DisableCommand {
                group: g,
                kind: DisableKind::Protective,
                origin: tx_mut.origin,
                recipients: tx_mut.recipient_mask,
                generation: gen,
                release_id: state.current_release_id,
                deadline_us: tx_mut.deadline_us,
                repeat_index: tx_mut.repeats_completed,
                use_reserved_budget: true,
            });
        }
    } else {
        emit_damping_commands(state, g, still_dampable, end_us, actions);
    }
}

fn emit_damping_commands(
    state: &TorqueState,
    g: u8,
    dampable_mask: ChannelMask,
    end_us: u64,
    actions: &mut ActionSet,
) {
    let gen = state.groups[g as usize].stop_generation;
    for c in dampable_mask.iter() {
        let ch_cfg = state.config.channels[c as usize];
        actions.damps[c as usize] = Some(DampCommand {
            channel: c,
            group: g,
            device: ch_cfg.device,
            kp: 0.0,
            kd: ch_cfg.damping.kd,
            max_torque_nm: ch_cfg.damping.max_torque_nm,
            end_us,
            generation: gen,
            use_reserved_budget: true,
        });
    }
}

fn step_protective_disabling_on_release(
    state: &mut TorqueState,
    g: u8,
    now_us: u64,
    actions: &mut ActionSet,
) {
    let gi = g as usize;
    let grp_cfg = state.config.groups[gi];

    let Some(tx) = state.groups[gi].disable_tx else {
        if state.groups[gi].torque_intent == TorqueIntent::On
            && !state.groups[gi].thermal_tripped_mask.is_empty()
        {
            trigger_protective_disable(state, g, DisableOrigin::ProtectiveThermal, now_us, actions);
        }
        return;
    };

    match tx.phase {
        DisablePhase::ProtectivePendingDispatch => {
            if now_us >= tx.deadline_us && !tx.committed_off {
                // Deadline expired with zero writes: TorqueIntent stays On, surface failure.
                state.groups[gi].faults.insert(TorqueFaultKind::DriverWriteFailed);
                let comp = build_completion(g, &tx, true);
                actions.disable_completions[gi] = Some(comp);
                if !state.groups[gi].thermal_tripped_mask.is_empty() {
                    // Thermal still hot and torque still On: open fresh reserved-budget retry window.
                    let new_deadline = now_us.saturating_add(grp_cfg.disable_deadline_us);
                    let gen = state.groups[gi].stop_generation;
                    if let Some(tx_mut) = state.groups[gi].disable_tx.as_mut() {
                        tx_mut.started_us = now_us;
                        tx_mut.deadline_us = new_deadline;
                        tx_mut.awaiting_write_result = true;
                        tx_mut.last_dispatched_release_id = Some(state.current_release_id);
                        state.groups[gi].last_authorized_disable_gen = Some(gen);
                        actions.disables[gi] = Some(DisableCommand {
                            group: g,
                            kind: DisableKind::Protective,
                            origin: tx_mut.origin,
                            recipients: tx_mut.recipient_mask,
                            generation: gen,
                            release_id: state.current_release_id,
                            deadline_us: new_deadline,
                            repeat_index: tx_mut.repeats_completed,
                            use_reserved_budget: true,
                        });
                    }
                } else {
                    state.groups[gi].disable_tx = None;
                }
            } else if !tx.awaiting_write_result
                || tx
                    .last_dispatched_release_id
                    .is_none_or(|r| r < state.current_release_id)
            {
                let gen = state.groups[gi].stop_generation;
                if let Some(tx_mut) = state.groups[gi].disable_tx.as_mut() {
                    tx_mut.awaiting_write_result = true;
                    tx_mut.last_dispatched_release_id = Some(state.current_release_id);
                    state.groups[gi].last_authorized_disable_gen = Some(gen);
                    actions.disables[gi] = Some(DisableCommand {
                        group: g,
                        kind: DisableKind::Protective,
                        origin: tx_mut.origin,
                        recipients: tx_mut.recipient_mask,
                        generation: gen,
                        release_id: state.current_release_id,
                        deadline_us: tx_mut.deadline_us,
                        repeat_index: tx_mut.repeats_completed,
                        use_reserved_budget: true,
                    });
                }
            }
        }
        DisablePhase::Dispatched => {
            if now_us >= tx.deadline_us {
                finish_disable_tx(state, g, true, actions);
            } else if tx.repeats_completed >= grp_cfg.disable_repeats {
                finish_disable_tx(state, g, false, actions);
            } else if now_us >= tx.next_repeat_at_us
                && (!tx.awaiting_write_result
                    || tx
                        .last_dispatched_release_id
                        .is_none_or(|r| r < state.current_release_id))
            {
                let gen = state.groups[gi].stop_generation;
                if let Some(tx_mut) = state.groups[gi].disable_tx.as_mut() {
                    tx_mut.awaiting_write_result = true;
                    tx_mut.last_dispatched_release_id = Some(state.current_release_id);
                    state.groups[gi].last_authorized_disable_gen = Some(gen);
                    actions.disables[gi] = Some(DisableCommand {
                        group: g,
                        kind: DisableKind::Protective,
                        origin: tx_mut.origin,
                        recipients: tx_mut.recipient_mask,
                        generation: gen,
                        release_id: state.current_release_id,
                        deadline_us: tx_mut.deadline_us,
                        repeat_index: tx_mut.repeats_completed,
                        use_reserved_budget: true,
                    });
                }
            }
        }
        _ => {}
    }
}

fn check_safe_hold_bus_loss_trip(
    state: &mut TorqueState,
    g: u8,
    now_us: u64,
    actions: &mut ActionSet,
) -> bool {
    let gi = g as usize;
    let active = group_active_channels(state, g);
    let mut any_stale = false;
    let mut min_limit_us = u64::MAX;

    for c in active.iter() {
        if state.joints[c as usize].captured_position_rad.is_none() {
            any_stale = true;
            let dev = state.config.channels[c as usize].device as usize;
            min_limit_us = min_limit_us.min(state.config.devices[dev].panic_bus_loss_limit_us);
        }
    }

    if any_stale {
        let since = *state.groups[gi]
            .safe_hold_bus_loss_since_us
            .get_or_insert(now_us);
        if now_us.saturating_sub(since) >= min_limit_us {
            trigger_protective_disable(
                state,
                g,
                DisableOrigin::ProtectivePanicBusLoss,
                now_us,
                actions,
            );
            return true;
        }
    } else {
        state.groups[gi].safe_hold_bus_loss_since_us = None;
    }
    false
}

fn step_non_protective_disable_on_release(
    state: &mut TorqueState,
    g: u8,
    now_us: u64,
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
        } => {
            if release_id < state.current_release_id {
                // Previous release ended with zero reported writes -> TorqueIntent stays On.
                if generation != state.groups[gi].stop_generation {
                    abort_non_protective_tx(state, g, RefusalCode::PreemptedByStop, false, actions);
                } else if now_us >= tx.deadline_us {
                    let code = rest_timeout_code(state, &tx);
                    abort_non_protective_tx(state, g, code, true, actions);
                } else {
                    if let Some(tx_mut) = state.groups[gi].disable_tx.as_mut() {
                        tx_mut.phase = DisablePhase::AwaitRestGate;
                        tx_mut.awaiting_write_result = false;
                    }
                    try_authorize_rest_gate(state, g, now_us, actions);
                }
            }
        }
        DisablePhase::AwaitRestGate => {
            if tx.auth_generation != state.groups[gi].stop_generation {
                abort_non_protective_tx(state, g, RefusalCode::PreemptedByStop, false, actions);
            } else if now_us >= tx.deadline_us {
                let code = rest_timeout_code(state, &tx);
                abort_non_protective_tx(state, g, code, true, actions);
            } else {
                try_authorize_rest_gate(state, g, now_us, actions);
                // If still awaiting rest gate, ask for refresh on incomplete/stale recipients.
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
        DisablePhase::Dispatched => {
            if now_us >= tx.deadline_us {
                finish_disable_tx(state, g, true, actions);
            } else if tx.repeats_completed >= grp_cfg.disable_repeats {
                finish_disable_tx(state, g, false, actions);
            } else if now_us >= tx.next_repeat_at_us
                && (!tx.awaiting_write_result
                    || tx
                        .last_dispatched_release_id
                        .is_none_or(|r| r < state.current_release_id))
            {
                let gen = state.groups[gi].stop_generation;
                if let Some(tx_mut) = state.groups[gi].disable_tx.as_mut() {
                    tx_mut.awaiting_write_result = true;
                    tx_mut.last_dispatched_release_id = Some(state.current_release_id);
                    state.groups[gi].last_authorized_disable_gen = Some(gen);
                    actions.disables[gi] = Some(DisableCommand {
                        group: g,
                        kind: DisableKind::Requested,
                        origin: tx_mut.origin,
                        recipients: tx_mut.recipient_mask,
                        generation: gen,
                        release_id: state.current_release_id,
                        deadline_us: tx_mut.deadline_us,
                        repeat_index: tx_mut.repeats_completed,
                        use_reserved_budget: false,
                    });
                }
            }
        }
        _ => {}
    }
}

fn emit_group_holds(state: &mut TorqueState, g: u8, now_us: u64, actions: &mut ActionSet) {
    let gi = g as usize;
    let grp = state.groups[gi];
    let needs_holds = grp.torque_intent == TorqueIntent::On
        && (grp.drive_state.requires_holds_when_on()
            || (grp.drive_state == DriveState::FaultReactionActive && !grp.park_in_flight));
    if !needs_holds {
        return;
    }

    let active = group_active_channels(state, g);
    let dev_wide_fenced = device_wide_mode_fenced(state, state.config.groups[gi].devices);
    for c in active.iter() {
        let ch_cfg = state.config.channels[c as usize];
        let rt = &mut state.joints[c as usize];
        let mode_ok = rt.verified_mode == Some(rt.desired_mode) && !dev_wide_fenced;
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
                JointRole::ActiveTool(ActiveToolSafeOutput::StopOutput) => HoldKind::ActiveToolStop,
                JointRole::ActiveTool(ActiveToolSafeOutput::BoundedRetention {
                    max_retention_us,
                }) => {
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
                JointRole::PassiveSensor => continue,
            };
            actions.holds[c as usize] = Some(HoldCommand {
                channel: c,
                group: g,
                device: ch_cfg.device,
                position_rad: pos,
                captured_at_us: cap_at,
                last_sample_us: last_ts,
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

// =========================================================================
// Event::CausalSample & Rest Gate
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

    // 1. Whole-hand / device ACK only: handshake completion and telemetry only; NEVER position evidence.
    if sample.is_handshake_ack_only {
        if let Some(en) = sample.reported_enabled {
            state.joints[ch].reported_enabled = Some(en);
            if en && state.groups[gi].drive_state == DriveState::ReadyToSwitchOn {
                advance_enable_confirmation(state, g, ChannelMask::bit(sample.channel), now_us, actions);
            }
        }
        return;
    }

    // 2. Passive sensor: observations only (no captures, no rest gate, no torque commands).
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

    let rec_idx = ch_cfg.recipient as usize;

    // 3. Future-dated timestamps are rejected and never clamped; invalid evidence resets window.
    if sample.rx_timestamp_us > sample.now_us || sample.rx_timestamp_us > now_us {
        state.rest_windows[rec_idx].reset();
        return;
    }

    // 4. Clock offset drift > 1 ms invalidates the rest window and is non-causal.
    if sample.clock_offset_drift_us > CLOCK_OFFSET_MAX_DRIFT_US {
        state.rest_windows[rec_idx].reset();
        return;
    }

    // 5. Feetech / serial poisoned line: reject until silence period completes; activity extends silence.
    let dev_cfg = state.config.devices[d];
    if sample.rx_timestamp_us < state.devices[d].poisoned_until_us {
        state.rest_windows[rec_idx].reset();
        if dev_cfg.poison_silence_us > 0 {
            let extended = sample.rx_timestamp_us.saturating_add(dev_cfg.poison_silence_us);
            state.devices[d].poisoned_until_us = state.devices[d].poisoned_until_us.max(extended);
        }
        return;
    }

    // 6. Thermal telemetry carried on valid non-future samples.
    if let Some(temp_c) = sample.temperature_c {
        apply_thermal(state, sample.channel, temp_c, now_us, actions);
    }

    // 7. Reported enable state: telemetry and enable handshake completion ONLY.
    if let Some(en) = sample.reported_enabled {
        state.joints[ch].reported_enabled = Some(en);
        if en && state.groups[gi].drive_state == DriveState::ReadyToSwitchOn {
            advance_enable_confirmation(state, g, ChannelMask::bit(sample.channel), now_us, actions);
        }
    }

    // 8. Finite numeric check & reply correlation check.
    if !sample.position_rad.is_finite()
        || sample.velocity_rad_s.is_some_and(|v| !v.is_finite())
        || sample
            .correlated_after_write_done_us
            .is_some_and(|wd| sample.rx_timestamp_us < wd)
    {
        state.rest_windows[rec_idx].reset();
        return;
    }

    // 9. Out-of-order / backwards timestamp on this channel resets window and does not overwrite capture.
    if state.joints[ch]
        .last_sample_us
        .is_some_and(|prev_ts| sample.rx_timestamp_us < prev_ts)
    {
        state.rest_windows[rec_idx].reset();
        return;
    }

    // 10. Freshness check (<= max_capture_age_us, default 100 ms).
    if !is_sample_fresh(sample.rx_timestamp_us, now_us, ch_cfg.max_capture_age_us) {
        state.rest_windows[rec_idx].reset();
        return;
    }

    // Valid, fresh, causal sample: update joint telemetry and anchored capture.
    {
        let rt = &mut state.joints[ch];
        rt.last_sample_id = Some(sample.sample_id);
        rt.last_sample_us = Some(sample.rx_timestamp_us);
        rt.last_sample_release_id = Some(state.current_release_id);
        rt.last_measured_pos_rad = Some(sample.position_rad);
        rt.last_measured_vel_rad_s = sample.velocity_rad_s;
        if let Some(ev) = sample.release_evidence {
            rt.last_release_evidence = ev;
        }

        let grp = state.groups[gi];
        let holding_anchored = grp.torque_intent == TorqueIntent::On
            && (grp.drive_state.requires_holds_when_on()
                || (grp.drive_state == DriveState::FaultReactionActive && !grp.park_in_flight));
        if holding_anchored {
            // Keep capture fixed while fresh; capture only if previously uncaptured.
            if rt.captured_position_rad.is_none() {
                rt.captured_position_rad = Some(sample.position_rad);
                rt.captured_at_us = Some(sample.rx_timestamp_us);
            }
        } else {
            rt.captured_position_rad = Some(sample.position_rad);
            rt.captured_at_us = Some(sample.rx_timestamp_us);
        }
    }

    // If in SafeHold and all active channels in this group are now fresh, clear bus-loss timer.
    if state.groups[gi].drive_state == DriveState::SafeHold {
        let active = group_active_channels(state, g);
        let all_fresh = active.iter().all(|c| {
            state.joints[c as usize].captured_position_rad.is_some()
        });
        if all_fresh {
            state.groups[gi].safe_hold_bus_loss_since_us = None;
        }
    }

    // 11. Feed active AwaitRestGate disable transaction on this group (if any).
    feed_rest_gate_sample(state, g, sample, now_us, actions);
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
        return;
    };
    if !matches!(tx.phase, DisablePhase::AwaitRestGate) || tx.origin.is_protective() {
        return;
    }

    let ch = sample.channel as usize;
    let ch_cfg = state.config.channels[ch];
    let rec_idx = ch_cfg.recipient as usize;
    if !tx.recipient_mask.contains(ch_cfg.recipient) {
        return;
    }

    // Expired transaction cannot accept samples.
    if now_us >= tx.deadline_us {
        let code = rest_timeout_code(state, &tx);
        abort_non_protective_tx(state, g, code, true, actions);
        return;
    }

    let rec_cfg = state.config.recipients[rec_idx];

    // Strict post-motion settle: strictly > 20 ms after last motion write.
    if let Some(last_mw) = state.joints[ch].last_motion_write_us {
        if sample.rx_timestamp_us <= last_mw.saturating_add(REST_SETTLE_US) {
            state.rest_windows[rec_idx].reset();
            return;
        }
    }

    // Role-specific agreement check.
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
                state.rest_windows[rec_idx].reset();
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
        JointRole::Gripper(GripperReleasePolicy::LegacyJ7) => {
            // Legacy J7: 3 fresh distinct causal samples, no rest position tolerance.
            true
        }
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
            let ev = sample
                .release_evidence
                .unwrap_or(state.joints[ch].last_release_evidence);
            match ev {
                ReleaseEvidence::HoldingUnsupportedPayload => {
                    state.rest_windows[rec_idx].reset();
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
                ReleaseEvidence::SupportedObject | ReleaseEvidence::VerifiedEmptyGrasp => true,
                ReleaseEvidence::Unknown => {
                    state.rest_windows[rec_idx].reset();
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
                state.rest_windows[rec_idx].reset();
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
            let ev = sample
                .release_evidence
                .unwrap_or(state.joints[ch].last_release_evidence);
            match ev {
                ReleaseEvidence::HoldingUnsupportedPayload => {
                    state.rest_windows[rec_idx].reset();
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
                ReleaseEvidence::SupportedObject | ReleaseEvidence::VerifiedEmptyGrasp => true,
                ReleaseEvidence::Unknown => {
                    state.rest_windows[rec_idx].reset();
                    false
                }
            }
        }
        JointRole::PassiveSensor => false,
    };

    if !agrees {
        return;
    }

    // Advance recipient K-window once per distinct release once all channels of the recipient agreed.
    let min_age = recipient_min_capture_age_us(state, ch_cfg.recipient);
    let current_rel = state.current_release_id;
    let win = &mut state.rest_windows[rec_idx];

    if win.release_agreed_id != current_rel {
        win.release_agreed_id = current_rel;
        win.release_agreed_channels = ChannelMask::EMPTY;
    }
    win.release_agreed_channels.insert(sample.channel);

    if win.release_agreed_channels == rec_cfg.channels {
        // Compute conservative timestamp across all channels of this recipient for this release.
        let mut min_ts = u64::MAX;
        let mut max_ts = 0u64;
        for c in rec_cfg.channels.iter() {
            if let Some(ts) = state.joints[c as usize].last_sample_us {
                min_ts = min_ts.min(ts);
                max_ts = max_ts.max(ts);
            }
        }
        if min_ts == u64::MAX {
            return;
        }
        let win = &mut state.rest_windows[rec_idx];
        if win.count == 0 {
            win.count = 1;
            win.first_ts_us = max_ts;
            win.last_ts_us = min_ts;
            win.last_sample_id = sample.sample_id;
            win.last_release_id = current_rel;
        } else if win.last_release_id == current_rel {
            // Same release: counts at most once per release.
        } else if win.last_sample_id == sample.sample_id {
            // Duplicate sample_id: does not advance K.
        } else if min_ts < win.last_ts_us {
            // Backwards timestamp across releases: invalid evidence resets window.
            win.reset();
        } else if min_ts.saturating_sub(win.last_ts_us) > min_age {
            // Stale blackout gap between consecutive samples: restart run at 1.
            win.count = 1;
            win.first_ts_us = max_ts;
            win.last_ts_us = min_ts;
            win.last_sample_id = sample.sample_id;
            win.last_release_id = current_rel;
        } else {
            win.count = win.count.saturating_add(1).min(REST_WINDOW_SAMPLES);
            win.last_ts_us = min_ts;
            win.last_sample_id = sample.sample_id;
            win.last_release_id = current_rel;
        }
    }

    // Re-evaluate whether every actual recipient now passes in the current release.
    try_authorize_rest_gate(state, g, now_us, actions);
}

/// Evaluate whether physical recipient `r` satisfies its role gate at `now_us`.
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
    if now_us < dev_rt.poisoned_until_us || win.first_ts_us < dev_rt.poisoned_until_us {
        return false;
    }
    if win.count < REST_WINDOW_SAMPLES {
        return false;
    }
    if win.span_us() < REST_WINDOW_MIN_SPAN_US {
        return false;
    }
    let min_age = recipient_min_capture_age_us(state, r);
    if !is_sample_fresh(win.last_ts_us, now_us, min_age) {
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
            if win.first_ts_us <= last_mw.saturating_add(REST_SETTLE_US) {
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
    if !matches!(tx.phase, DisablePhase::AwaitRestGate) || tx.origin.is_protective() {
        return;
    }
    if now_us >= tx.deadline_us || tx.auth_generation != state.groups[gi].stop_generation {
        return;
    }
    if tx.recipient_mask.is_empty() {
        return;
    }

    // All actual recipients MUST pass simultaneously in the current release.
    for r in tx.recipient_mask.iter() {
        if !recipient_passes_gate(state, r, &tx, now_us) {
            return;
        }
    }

    // Gate succeeded: authorize a generation-bound dispatch for the current release only.
    let gen = state.groups[gi].stop_generation;
    let rel = state.current_release_id;
    if let Some(tx_mut) = state.groups[gi].disable_tx.as_mut() {
        tx_mut.phase = DisablePhase::AuthorizedForRelease {
            release_id: rel,
            generation: gen,
        };
        tx_mut.awaiting_write_result = true;
        tx_mut.last_dispatched_release_id = Some(rel);
        state.groups[gi].last_authorized_disable_gen = Some(gen);
        actions.disables[gi] = Some(DisableCommand {
            group: g,
            kind: DisableKind::Requested,
            origin: tx_mut.origin,
            recipients: tx_mut.recipient_mask,
            generation: gen,
            release_id: rel,
            deadline_us: tx_mut.deadline_us,
            repeat_index: 0,
            use_reserved_budget: false,
        });
    }
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
    // 1. Contention escalation tracking (25 consecutive skips -> latch DriverWriteFailed).
    if wr.device < state.config.device_count && !wr.target_mask.is_empty() {
        let mut any_sent_or_ambiguous = false;
        let mut any_contended = false;
        for idx in wr.target_mask.iter() {
            match wr.outcomes[idx as usize] {
                MotorWriteOutcome::Sent | MotorWriteOutcome::Ambiguous => {
                    any_sent_or_ambiguous = true;
                }
                MotorWriteOutcome::LockContended => {
                    any_contended = true;
                }
                _ => {}
            }
        }
        let dev_rt = &mut state.devices[wr.device as usize];
        if any_sent_or_ambiguous {
            dev_rt.consecutive_contention_skips = 0;
            dev_rt.contention_escalated = false;
        } else if any_contended {
            dev_rt.consecutive_contention_skips =
                dev_rt.consecutive_contention_skips.saturating_add(1);
            if dev_rt.consecutive_contention_skips >= CONTENTION_ESCALATION_RELEASES {
                dev_rt.contention_escalated = true;
                actions.contention_escalations.insert(wr.device);
                for g in 0..state.config.group_count {
                    if state.config.groups[g as usize].devices.contains(wr.device) {
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

    match wr.kind {
        WriteKind::Hold => {
            // Stationary hold preserves last_motion_write_us and rest windows!
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
                    let ch_cfg = state.config.channels[c as usize];
                    if !matches!(ch_cfg.role, JointRole::PassiveSensor)
                        && ch_cfg.recipient < state.config.recipient_count
                    {
                        state.rest_windows[ch_cfg.recipient as usize].reset();
                    }
                }
            }
        }
        WriteKind::Enable => {
            if wr.group < state.config.group_count
                && state.groups[wr.group as usize].drive_state == DriveState::ReadyToSwitchOn
            {
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

    let matches_active_or_recent = state.groups[gi].disable_tx.is_some()
        || state.groups[gi].last_authorized_disable_gen == Some(wr.generation);
    if !matches_active_or_recent {
        return;
    }

    let valid_targets = wr.target_mask.intersect(grp_cfg.recipients);
    let mut sent_mask = ChannelMask::EMPTY;
    let mut ambiguous_mask = ChannelMask::EMPTY;
    let mut failed_mask = ChannelMask::EMPTY;
    let mut deferred_mask = ChannelMask::EMPTY;
    let mut rejected_stale_mask = ChannelMask::EMPTY;

    for r in valid_targets.iter() {
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

    let any_sent_or_ambiguous = !sent_mask.is_empty() || !ambiguous_mask.is_empty();

    if any_sent_or_ambiguous {
        // First successful or possibly-sent disable IMMEDIATELY commits TorqueIntent::Off.
        state.groups[gi].torque_intent = TorqueIntent::Off;

        if let Some(tx_mut) = state.groups[gi].disable_tx.as_mut() {
            tx_mut.committed_off = true;
            tx_mut.awaiting_write_result = false;
            tx_mut.sent_mask = tx_mut.sent_mask.union(sent_mask);
            tx_mut.ambiguous_mask = tx_mut.ambiguous_mask.union(ambiguous_mask);
            tx_mut.failed_mask = tx_mut.failed_mask.union(failed_mask);
            tx_mut.deferred_mask = tx_mut.deferred_mask.union(deferred_mask);
            tx_mut.repeats_completed = tx_mut.repeats_completed.saturating_add(1);
            tx_mut.next_repeat_at_us = now_us.saturating_add(grp_cfg.disable_repeat_interval_us);
            tx_mut.phase = DisablePhase::Dispatched;

            let done_repeats = tx_mut.repeats_completed >= grp_cfg.disable_repeats;
            let past_deadline = now_us >= tx_mut.deadline_us;
            if done_repeats || past_deadline {
                finish_disable_tx(state, g, past_deadline && !done_repeats, actions);
            }
        } else {
            // Late write result after a racing Stop cancelled AuthorizedForRelease:
            // still commit Off and surface exact per-motor outcomes.
            let unsent = grp_cfg
                .recipients
                .difference(sent_mask.union(ambiguous_mask));
            let partial = sent_mask != grp_cfg.recipients
                || !ambiguous_mask.is_empty()
                || !failed_mask.is_empty()
                || !unsent.is_empty();
            actions.disable_completions[gi] = Some(DisableCompletion {
                group: g,
                origin: if matches!(wr.kind, WriteKind::DisableProtective) {
                    DisableOrigin::ProtectiveThermal
                } else {
                    DisableOrigin::ExplicitRequest
                },
                recipient_mask: grp_cfg.recipients,
                sent_mask,
                ambiguous_mask,
                failed_mask,
                unsent_mask: unsent,
                committed_off: true,
                partial_or_ambiguous: partial,
                timed_out: false,
                repeats_completed: 1,
            });
            update_post_off_drive_state(state, g, matches!(wr.kind, WriteKind::DisableProtective));
        }
    } else {
        // ZERO writes: TorqueIntent stays On (if not already committed Off on an earlier repeat).
        let Some(tx) = state.groups[gi].disable_tx else {
            return;
        };
        if let Some(tx_mut) = state.groups[gi].disable_tx.as_mut() {
            tx_mut.awaiting_write_result = false;
            tx_mut.failed_mask = tx_mut.failed_mask.union(failed_mask);
            tx_mut.deferred_mask = tx_mut.deferred_mask.union(deferred_mask);
        }

        if tx.committed_off {
            // Earlier repeat already committed Off; bound retries by original deadline.
            if now_us >= tx.deadline_us {
                finish_disable_tx(state, g, true, actions);
            }
            return;
        }

        if !tx.origin.is_protective() {
            if !rejected_stale_mask.is_empty() || tx.auth_generation != state.groups[gi].stop_generation
            {
                abort_non_protective_tx(state, g, RefusalCode::PreemptedByStop, false, actions);
            } else if now_us >= tx.deadline_us {
                let code = rest_timeout_code(state, &tx);
                abort_non_protective_tx(state, g, code, true, actions);
            } else if !failed_mask.is_empty() && deferred_mask.is_empty() {
                abort_non_protective_tx(state, g, RefusalCode::ZeroWritesRefused, false, actions);
            } else if let Some(tx_mut) = state.groups[gi].disable_tx.as_mut() {
                // Deferred / contended with 0 writes: authorization was for that release only.
                tx_mut.phase = DisablePhase::AwaitRestGate;
            }
        } else if now_us >= tx.deadline_us {
            // Protective disable hit deadline with 0 writes: TorqueIntent stays On.
            state.groups[gi].faults.insert(TorqueFaultKind::DriverWriteFailed);
            let comp = build_completion(g, &tx, true);
            actions.disable_completions[gi] = Some(comp);
            if state.groups[gi].thermal_tripped_mask.is_empty() {
                state.groups[gi].disable_tx = None;
            }
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
    if state.worker_panic_latched {
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
    if grp.disable_tx.is_some() {
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
    if matches!(
        grp.drive_state,
        DriveState::ReadyToSwitchOn | DriveState::OperationEnabled
    ) {
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

    // Damiao TIMEOUT readback gate: refuse before changing intent when configured and mismatched.
    let active = group_active_channels(state, group);
    for c in active.iter() {
        if let Some(expected) = state.config.channels[c as usize].damiao_timeout_reg {
            if state.joints[c as usize].verified_timeout_reg != Some(expected) {
                actions.refusals[gi] = Some(RefusalEvent {
                    group,
                    code: RefusalCode::TimeoutMismatch,
                    channel: Some(c),
                });
                return;
            }
        }
    }

    // Accept EnableStart: TorqueIntent immediately becomes On.
    let deadline_us = now_us.saturating_add(grp_cfg.enable_deadline_us);
    let gen = grp.stop_generation;
    state.groups[gi].torque_intent = TorqueIntent::On;
    state.groups[gi].drive_state = DriveState::ReadyToSwitchOn;
    state.groups[gi].enable_phase = EnablePhase::Enabling;
    state.groups[gi].last_authorized_disable_gen = None;
    state.groups[gi].enable_tx = Some(EnableTransaction {
        started_us: now_us,
        deadline_us,
        generation: gen,
        target_channels: active,
        confirmed_channels: ChannelMask::EMPTY,
    });
    for c in active.iter() {
        state.joints[c as usize].retention_started_us = None;
    }
    actions.enables[gi] = Some(EnableCommand {
        group,
        channels: active,
        generation: gen,
        deadline_us,
    });
}

fn handle_handshake_result(
    state: &mut TorqueState,
    group: u8,
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
            advance_enable_confirmation(state, group, confirmed_mask, now_us, actions);
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

fn advance_enable_confirmation(
    state: &mut TorqueState,
    group: u8,
    confirmed_mask: ChannelMask,
    now_us: u64,
    actions: &mut ActionSet,
) {
    let gi = group as usize;
    let Some(mut etx) = state.groups[gi].enable_tx else {
        return;
    };
    if now_us >= etx.deadline_us {
        enter_failed_hold(state, group, now_us, actions);
        return;
    }
    etx.confirmed_channels = etx
        .confirmed_channels
        .union(confirmed_mask.intersect(etx.target_channels));
    state.groups[gi].enable_tx = Some(etx);

    if etx.confirmed_channels == etx.target_channels {
        state.groups[gi].enable_phase = EnablePhase::Validating;
        if validate_enable_postconditions(state, group) {
            state.groups[gi].drive_state = DriveState::OperationEnabled;
            state.groups[gi].enable_phase = EnablePhase::Armed;
            state.groups[gi].enable_tx = None;
            state.groups[gi].torque_intent = TorqueIntent::On;
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
            if rt.verified_timeout_reg != Some(expected) {
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

    // Device-wide mode change clears verified_mode on every channel of the affected device(s)
    // (fencing the whole group until verified).
    for d in affected_devices.iter() {
        if device_wide || state.config.devices[d as usize].device_wide_mode_switch {
            for c in 0..state.config.channel_count {
                if state.config.channels[c as usize].device == d
                    && !matches!(state.config.channels[c as usize].role, JointRole::PassiveSensor)
                {
                    state.joints[c as usize].verified_mode = None;
                }
            }
        }
    }

    // Interrupting an in-flight enable fails into FailedHold (no rollback).
    if state.groups[gi].drive_state == DriveState::ReadyToSwitchOn {
        enter_failed_hold(state, group, now_us, actions);
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
    let g = state.config.channels[ch].group;
    let gi = g as usize;
    if state.groups[gi].drive_state == DriveState::ReadyToSwitchOn {
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

fn handle_timeout_readback(state: &mut TorqueState, channel: u8, timeout_reg: u16) {
    if channel >= state.config.channel_count {
        return;
    }
    state.joints[channel as usize].verified_timeout_reg = Some(timeout_reg);
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
    if state.worker_panic_latched {
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
    if grp.disable_tx.is_some() {
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
    }

    actions.motion_allowed[gi] = Some(MotionAllowed {
        group,
        channels: target,
        generation: grp.stop_generation,
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
    if grp.drive_state.is_protective() {
        actions.refusals[gi] = Some(RefusalEvent {
            group,
            code: RefusalCode::ProtectiveLatched,
            channel: None,
        });
        return;
    }
    if grp.drive_state == DriveState::ReadyToSwitchOn {
        // Interrupting an enable enters FailedHold + ENABLE_FAILED (no rollback, no disable).
        enter_failed_hold(state, group, now_us, actions);
        return;
    }
    if grp.disable_tx.is_some() {
        actions.refusals[gi] = Some(RefusalEvent {
            group,
            code: RefusalCode::DisableInProgress,
            channel: None,
        });
        return;
    }
    if grp.torque_intent == TorqueIntent::Off {
        // Already disabled: idempotent no-op (zero writes, intent stays Off).
        return;
    }

    // Validate and expand recipient mask (including grouped packet recipients).
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
    if !actual_recipients.is_subset_of(grp_cfg.recipients) {
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

    // If operational, stop into QuickStopActive and anchor captures while awaiting rest gate.
    if state.groups[gi].drive_state == DriveState::OperationEnabled {
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
    for r in actual_recipients.iter() {
        state.rest_windows[r as usize].reset();
    }
    state.groups[gi].disable_tx = Some(DisableTransaction {
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
        awaiting_write_result: false,
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
        state.groups[gi].park_in_flight = false;

        // Latch fault kind if applicable (Stop & LeaseLoss do not latch a fault; Watchdog latches WatchdogTimeout).
        match kind {
            StopKind::Stop | StopKind::LeaseLoss => {}
            StopKind::EStop => state.groups[gi].faults.insert(TorqueFaultKind::EStop),
            StopKind::Watchdog => state.groups[gi]
                .faults
                .insert(TorqueFaultKind::WatchdogTimeout),
            StopKind::Fault(fk) => state.groups[gi].faults.insert(fk),
        }

        // Cancel uncommitted non-protective disable transaction (0 writes -> TorqueIntent stays On).
        if let Some(tx) = state.groups[gi].disable_tx {
            if !tx.origin.is_protective() && !tx.committed_off {
                abort_non_protective_tx(state, g, RefusalCode::PreemptedByStop, false, actions);
            }
        }

        // Interrupting an in-flight enable enters FailedHold + ENABLE_FAILED (no rollback).
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

        // Protective and SafeHold states are never downgraded by ordinary Stop/Fault.
        if state.groups[gi].drive_state.is_protective()
            || state.groups[gi].drive_state == DriveState::SafeHold
        {
            continue;
        }

        let was_operational = state.groups[gi].drive_state == DriveState::OperationEnabled;
        if was_operational {
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
                    state.groups[gi].drive_state = DriveState::QuickStopActive;
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
        if state.groups[gi].disable_tx.is_some() {
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
        state.groups[gi].park_in_flight = false;
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
            // Still torqued (TorqueIntent::On): stay safely holding in QuickStopActive.
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
    now_us: u64,
    actions: &mut ActionSet,
) {
    if channel >= state.config.channel_count || !temperature_c.is_finite() {
        return;
    }
    let ch = channel as usize;
    let ch_cfg = state.config.channels[ch];
    let g = ch_cfg.group;
    let gi = g as usize;

    state.joints[ch].last_temperature_c = Some(temperature_c);
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
    state.groups[gi].park_in_flight = false;
    if origin == DisableOrigin::ProtectiveThermal {
        state.groups[gi].faults.insert(TorqueFaultKind::ThermalTrip);
        state.groups[gi]
            .thermal_trip_release_id
            .get_or_insert(state.current_release_id);
    }

    // If interrupting an enable handshake, record ENABLE_FAILED (and preempt into protective).
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

    // Already Off and no protective tx in flight -> ProtectiveDisabled.
    if state.groups[gi].torque_intent == TorqueIntent::Off {
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

    // Preempt any non-protective disable transaction.
    if let Some(old_tx) = state.groups[gi].disable_tx {
        if !old_tx.origin.is_protective() {
            if !interrupted_enable {
                actions.refusals[gi] = Some(RefusalEvent {
                    group: g,
                    code: RefusalCode::PreemptedByProtective,
                    channel: None,
                });
            }
            if !old_tx.committed_off {
                actions.disable_completions[gi] = Some(build_completion(g, &old_tx, false));
            }
            state.groups[gi].disable_tx = None;
        }
    }

    state.groups[gi].stop_generation = state.groups[gi].stop_generation.saturating_add(1);
    let gen = state.groups[gi].stop_generation;

    // Determine whether MIT kd-only damping precedes DisableProtective.
    let mut dampable_mask = ChannelMask::EMPTY;
    let mut max_damp_us = 0u64;
    if matches!(
        origin,
        DisableOrigin::ProtectiveThermal | DisableOrigin::ProtectiveProcessExit
    ) {
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
        let end_us = now_us.saturating_add(max_damp_us);
        let deadline_us = end_us.saturating_add(grp_cfg.disable_deadline_us);
        state.groups[gi].drive_state = DriveState::ProtectiveDamping;
        state.groups[gi].disable_tx = Some(DisableTransaction {
            origin,
            phase: DisablePhase::ProtectiveDamping {
                started_us: now_us,
                end_us,
                dampable_mask,
            },
            started_us: now_us,
            deadline_us,
            auth_generation: gen,
            recipient_mask: grp_cfg.recipients,
            explicit_release_authorized: false,
            committed_off: false,
            repeats_completed: 0,
            next_repeat_at_us: 0,
            awaiting_write_result: false,
            last_dispatched_release_id: None,
            sent_mask: ChannelMask::EMPTY,
            ambiguous_mask: ChannelMask::EMPTY,
            failed_mask: ChannelMask::EMPTY,
            deferred_mask: ChannelMask::EMPTY,
        });
        emit_damping_commands(state, g, dampable_mask, end_us, actions);
    } else {
        let deadline_us = now_us.saturating_add(grp_cfg.disable_deadline_us);
        state.groups[gi].drive_state = DriveState::ProtectiveDisabling;
        state.groups[gi].last_authorized_disable_gen = Some(gen);
        state.groups[gi].disable_tx = Some(DisableTransaction {
            origin,
            phase: DisablePhase::ProtectivePendingDispatch,
            started_us: now_us,
            deadline_us,
            auth_generation: gen,
            recipient_mask: grp_cfg.recipients,
            explicit_release_authorized: false,
            committed_off: false,
            repeats_completed: 0,
            next_repeat_at_us: 0,
            awaiting_write_result: true,
            last_dispatched_release_id: Some(state.current_release_id),
            sent_mask: ChannelMask::EMPTY,
            ambiguous_mask: ChannelMask::EMPTY,
            failed_mask: ChannelMask::EMPTY,
            deferred_mask: ChannelMask::EMPTY,
        });
        actions.disables[gi] = Some(DisableCommand {
            group: g,
            kind: DisableKind::Protective,
            origin,
            recipients: grp_cfg.recipients,
            generation: gen,
            release_id: state.current_release_id,
            deadline_us,
            repeat_index: 0,
            use_reserved_budget: true,
        });
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
            state.rest_windows[r as usize].reset();
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
    // Only operational groups may update grasp setpoint or force ceiling:
    // stop and FailedHold NEVER open jaws or raise force.
    let grp = state.groups[ch_cfg.group as usize];
    if grp.drive_state != DriveState::OperationEnabled {
        return;
    }
    state.joints[ch].grasp_setpoint_rad = Some(setpoint_rad);
    state.joints[ch].grasp_force_limit_raw = force_limit_raw;
}

fn handle_park_settled(
    state: &mut TorqueState,
    group: u8,
    succeeded: bool,
    now_us: u64,
) {
    if group >= state.config.group_count {
        return;
    }
    let gi = group as usize;
    state.groups[gi].park_in_flight = false;

    if state.groups[gi].drive_state.is_protective()
        || state.groups[gi].drive_state == DriveState::SafeHold
    {
        return;
    }

    if !succeeded {
        // Park failed -> keep holding, latch Fault, NEVER disable!
        state.groups[gi].faults.insert(TorqueFaultKind::Other);
        if state.groups[gi].drive_state != DriveState::FailedHold {
            state.groups[gi].drive_state = DriveState::Fault;
        }
        anchor_group_captures(state, group, now_us);
        return;
    }

    // Park succeeded on this group ONLY (never authorizes releasing any separate hand group!).
    if state.groups[gi].torque_intent == TorqueIntent::On && state.groups[gi].disable_tx.is_none() {
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
        // Only start rest-gated disable if every recipient in this group can pass without
        // implicit hand release (arm rest NEVER authorizes releasing a hand payload!).
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
                        state.groups[gi].drive_state = DriveState::FaultReactionActive;
                        state.groups[gi].park_in_flight = true;
                        actions.park_requests.insert(g);
                    } else if state.groups[gi].drive_state != DriveState::FailedHold {
                        // Gripper / hand group: keep grasp held, never park or release payload!
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
                state.groups[gi].park_in_flight = false;
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

                if state.groups[gi].drive_state.is_protective() {
                    continue;
                }

                if state.groups[gi].torque_intent == TorqueIntent::On {
                    capture_latest_for_stop(state, g, now_us);
                    state.groups[gi].drive_state = DriveState::SafeHold;
                    state.groups[gi].safe_hold_bus_loss_since_us = None;
                } else {
                    state.groups[gi].drive_state = DriveState::Fault;
                }
            }
        }
        PanicTier::ProcessDeath => {
            // Tier 3: no in-process software recovery guaranteed; hardware TIMEOUT fallback owns it.
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

fn handle_bus_loss(state: &mut TorqueState, device: u8, now_us: u64) {
    if device >= state.config.device_count {
        return;
    }
    // Invalidate captures and rest windows for channels on this device.
    for c in 0..state.config.channel_count {
        if state.config.channels[c as usize].device == device {
            state.joints[c as usize].captured_position_rad = None;
            state.joints[c as usize].captured_at_us = None;
        }
    }
    for r in 0..state.config.recipient_count {
        if state.config.recipients[r as usize].device == device {
            state.rest_windows[r as usize].reset();
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
        state.groups[gi].park_in_flight = false;

        if state.groups[gi].drive_state == DriveState::SafeHold {
            state.groups[gi]
                .safe_hold_bus_loss_since_us
                .get_or_insert(now_us);
            continue;
        }
        if state.groups[gi].drive_state.is_protective() {
            continue;
        }
        if state.groups[gi].drive_state == DriveState::ReadyToSwitchOn {
            state.groups[gi].drive_state = DriveState::FailedHold;
            state.groups[gi].enable_phase = EnablePhase::FailedHold;
            state.groups[gi].enable_tx = None;
            state.groups[gi].torque_intent = TorqueIntent::On;
            state.groups[gi].faults.insert(TorqueFaultKind::EnableFailed);
            continue;
        }
        if let Some(tx) = state.groups[gi].disable_tx {
            if !tx.origin.is_protective() && !tx.committed_off {
                state.groups[gi].disable_tx = None;
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
    let is_protective = tx.origin.is_protective();
    actions.disable_completions[gi] = Some(build_completion(g, &tx, timed_out));
    if tx.committed_off {
        update_post_off_drive_state(state, g, is_protective);
    }
}

fn update_post_off_drive_state(state: &mut TorqueState, g: u8, is_protective: bool) {
    let gi = g as usize;
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
    let idx = if (group as usize) < MAX_GROUPS {
        group as usize
    } else {
        0
    };
    actions.refusals[idx] = Some(RefusalEvent {
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

    // Shared-bus TX budget accounting across devices sharing `bus_id`.
    let mut budgets = [BusBudgetSummary::default(); MAX_DEVICES];
    for d in 0..state.config.device_count {
        let dev = state.config.devices[d as usize];
        let b = dev.bus_id as usize;
        if b < MAX_DEVICES {
            // Shared bus uses the shared quota / reserved safety capacity.
            budgets[b].normal_quota = budgets[b].normal_quota.max(dev.tx_quota_per_release);
            budgets[b].reserved_safety_quota = budgets[b]
                .reserved_safety_quota
                .max(dev.reserved_safety_tx);
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
    for g in 0..state.config.group_count {
        if let Some(dcmd) = actions.disables[g as usize] {
            for r in dcmd.recipients.iter() {
                if r < state.config.recipient_count {
                    let b = state.config.devices[state.config.recipients[r as usize].device as usize]
                        .bus_id as usize;
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
    }

    actions.telemetry.bus_budgets = budgets;
}
