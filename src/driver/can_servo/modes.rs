//! Damiao control modes, `ModeSwitchExt`, and `JointModeExt`.

#[allow(unused_imports)]
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
#[allow(unused_imports)]
use std::sync::Arc;
#[allow(unused_imports)]
use std::time::{Duration, Instant};

#[allow(unused_imports)]
use crate::driver::*;
#[allow(unused_imports)]
use crate::driver::can_servo::*;
#[allow(unused_imports)]
use crate::model::JointState;
#[allow(unused_imports)]
use crate::protocol::damiao::{self, DamiaoLimits};
#[allow(unused_imports)]
use crate::transport::{CanFrame, CanRxTime, CanTransport, TransportError, MAX_REALTIME_MONOTONIC_OFFSET_DRIFT_NS};

pub(super) mod write;

impl<C: CanTransport> ModeSwitchExt for DamiaoArmDriver<C> {
    fn supports_nonblocking_mode_switch(&self) -> bool {
        self.can.supports_nonblocking_rt() && self.can.supports_bounded_flush()
    }

    fn driver_vendor_binding(&self) -> Option<crate::motor_mode::VendorBinding> {
        Some(crate::motor_mode::VendorBinding::Damiao)
    }

    fn mode_switch_dof(&self) -> usize {
        self.motor_ids.len()
    }

    fn axis_is_energized(&self, joint: usize) -> bool {
        if joint >= self.motor_ids.len() {
            return false;
        }
        let bit_on = joint < 32 && (self.motor_energized_mask & (1u32 << joint)) != 0;
        let status_on = self.motor_status.get(joint).copied() == Some(1);
        bit_on || status_on || (self.is_enabled && self.motor_status.get(joint).copied() != Some(0))
    }

    fn any_axis_energized(&self) -> bool {
        self.motor_energized_mask != 0
            || self.is_enabled
            || self.motor_status.contains(&1)
    }

    fn axis_has_watchdog(&self, joint: usize) -> bool {
        if joint >= self.motor_ids.len() {
            return false;
        }
        // Damiao J7 PosForce keeps TIMEOUT = 0 by design; arm joints carry TIMEOUT = 2000 (100 ms).
        self.base_mode(joint) != DamiaoControlMode::PosForce
            && self.pos_force.as_ref().is_none_or(|pf| pf.joint != joint)
    }

    fn axis_supports_pos_force(&self, joint: usize) -> bool {
        self.pos_force.as_ref().is_some_and(|pf| pf.joint == joint)
    }

    fn mode_switch_remaining_rx_budget(&self, default_max: u16) -> u16 {
        default_max.min(self.driver_remaining_release_rx())
    }

    fn axis_confirmed_mode(&self, joint: usize) -> Option<crate::motor_mode::ControlMode> {
        self.verified_hold_mode(joint)
            .map(damiao_mode_to_shadow_control_mode)
    }

    fn set_axis_confirmed_mode(
        &mut self,
        joint: usize,
        mode: Option<crate::motor_mode::ControlMode>,
    ) {
        if joint >= self.motor_ids.len() {
            return;
        }
        let prev = self.joint_modes[joint];
        match mode.and_then(control_mode_to_damiao) {
            Some(dm) => {
                if prev != Some(dm) {
                    self.pending_tx.discard_joint(joint);
                    self.pending_hold_tx.discard_joint(joint);
                }
                self.verified_modes[joint] = Some(dm);
                self.joint_modes[joint] = Some(dm);
                self.mode_readback_boundary[joint] = None;
                if joint < 32 {
                    self.mode_write_uncertain_mask &= !(1u32 << joint);
                }
                if let Some(pf) = self.pos_force.as_ref() {
                    if pf.joint == joint {
                        self.pos_force_verified = dm == DamiaoControlMode::PosForce;
                    }
                }
                #[cfg(not(target_arch = "wasm32"))]
                self.stage_damiao_mode_verified(joint, dm);
            }
            None => {
                self.latch_axis_mode_uncertain(joint);
            }
        }
    }

    fn latch_axis_mode_uncertain(&mut self, joint: usize) {
        if joint >= self.motor_ids.len() {
            return;
        }
        self.pending_tx.discard_joint(joint);
        self.pending_hold_tx.discard_joint(joint);
        self.verified_modes[joint] = None;
        self.joint_modes[joint] = None;
        self.mode_readback_boundary[joint] = None;
        if joint < MAX_MODE_JOINTS {
            self.uncertain_readback_probe[joint] = None;
        }
        if let Some(pf) = self.pos_force.as_ref() {
            if pf.joint == joint {
                self.pos_force_verified = false;
            }
        }
        if joint < 32 {
            self.mode_write_uncertain_mask |= 1u32 << joint;
        }
    }

    fn mode_write_uncertain_mask(&self) -> u32 {
        self.mode_write_uncertain_mask
    }

    fn set_mode_switch_active_mask(&mut self, mask: u32) {
        self.mode_switch_active_mask = mask;
        if mask == 0 {
            self.mode_switch_hold_targets = [None; MAX_MODE_JOINTS];
        }
    }

    fn axis_last_ctrl_tx(&self, joint: usize) -> Option<Instant> {
        self.motor_last_ctrl_tx.get(joint).copied().flatten()
    }

    fn axis_impedance_ranges_verified(&self, joint: usize) -> bool {
        self.mit_limits_verified.get(joint).copied().unwrap_or(false)
    }

    fn axis_configured_impedance_ranges(&self, joint: usize) -> Option<[f32; 3]> {
        let l = self.limits.get(joint)?;
        Some([l.p_max, l.v_max, l.t_max])
    }

    fn set_axis_impedance_ranges_verified(&mut self, joint: usize, verified: bool) {
        if let Some(slot) = self.mit_limits_verified.get_mut(joint) {
            *slot = verified;
        }
    }

    fn mode_switch_rx_drained(&self) -> bool {
        self.last_read_drained
    }

    fn mode_switch_service_watchdog_holds(
        &mut self,
        min_age: Duration,
        exclude_mask: u32,
        force_mask: u32,
        deadline: Instant,
        max_frames: u8,
    ) -> Result<crate::motor_mode::WatchdogHoldResult, crate::motor_mode::ModeSwitchFailure> {
        let n = self.motor_ids.len().min(MAX_MODE_JOINTS);
        let effective_exclude = exclude_mask | self.mode_write_uncertain_mask;
        let effective_max_frames = if self.in_active_control_release() {
            self.remaining_tick_tx_slots()
                .min(usize::from(max_frames)) as u8
        } else {
            max_frames
        };
        let now = self.now();

        // Collect due energized watchdog-bearing verified axes (oldest-unfed first), plus any
        // `force_mask` axis (refreshed in its last confirmed mode right before `REG_WRITE`).
        // Verified non-watchdog axes (e.g. a configured J7 gripper verified in PosVel — Major 2)
        // are collected separately and serviced AFTER arm watchdog axes so arm watchdogs keep
        // strict priority without starving J7 PosVel hold continuity.
        let mut due = [(0usize, Duration::ZERO, false); MAX_MODE_JOINTS];
        let mut due_len = 0usize;
        let mut non_wd_posvel = [0usize; MAX_MODE_JOINTS];
        let mut non_wd_len = 0usize;
        for i in 0..n {
            if !self.axis_is_energized(i) || (effective_exclude & (1u32 << i)) != 0 {
                continue;
            }
            if i < 64 && (self.disable_dispatched_mask & (1u64 << i)) != 0 {
                continue;
            }
            let Some(vmode) = self.verified_hold_mode(i) else {
                continue;
            };
            if vmode == DamiaoControlMode::PosForce {
                continue;
            }
            let already_this_release = (self.release_ctrl_tx_mask & (1u32 << i)) != 0
                || (self.staged_ctrl_tx_mask & (1u32 << i)) != 0;
            if already_this_release {
                continue;
            }
            if !self.axis_has_watchdog(i) {
                non_wd_posvel[non_wd_len] = i;
                non_wd_len += 1;
                continue;
            }
            if let Some(age) = self.axis_continuity_due_age(
                i,
                now,
                Some(min_age),
                0,
                effective_exclude,
                force_mask,
                true,
                false,
            ) {
                let forced = (force_mask & (1u32 << i)) != 0;
                due[due_len] = (i, age, forced);
                due_len += 1;
            }
        }
        // Insertion sort: oldest-unfed first (largest age first).
        for idx in 1..due_len {
            let key = due[idx];
            let mut j = idx;
            while j > 0 && due[j - 1].1 < key.1 {
                due[j] = due[j - 1];
                j -= 1;
            }
            due[j] = key;
        }

        let start_stamp = self.current_stop_stamp();
        self.discard_staged_ctrl_tx();
        let mut frames_sent: u8 = 0;
        let mut sent_mask: u32 = 0;
        let mut aborted_flush = false;
        let mut hit_stop = false;
        let mut missing_hold_for_due = false;
        // Every axis in `due[..due_len]` was selected by ModeSwitch's deadline-derived
        // `hold_due_age(budget)` or `force_mask`: treat ALL due watchdog-bearing arm holds as
        // protected continuity holds inside `tx_budget.max_frames` and transaction `deadline`.
        let due_bypass = ContinuityBypass::Ledger;
        let due_max_frames = if self.in_active_control_release() {
            self.remaining_tick_tx_slots_with_bypass(due_bypass)
                .min(usize::from(u8::MAX)) as u8
        } else {
            max_frames
        };
        self.in_urgent_watchdog_hold = true;
        // First pass: watchdog-bearing arm axes; second pass: verified PosVel non-watchdog axes (J7).
        for &(i, _, _) in &due[..due_len] {
            if frames_sent >= due_max_frames || self.now() >= deadline {
                break;
            }
            let target = if let Some(ht) = self.mode_switch_hold_targets.get(i).copied().flatten() {
                if self.verified_hold_mode(i) == Some(DamiaoControlMode::Mit) && ht.is_mit {
                    ht.to_joint_target()
                } else if ht.position.is_finite() {
                    JointTarget::position(ht.position)
                } else {
                    let Some(pos) = self.motor_keepalive_hold_pos[i].or_else(|| {
                        self.cached_states
                            .get(i)
                            .map(|s| s.position)
                            .filter(|p| p.is_finite())
                    }) else {
                        missing_hold_for_due = true;
                        continue;
                    };
                    JointTarget::position(pos)
                }
            } else {
                let Some(pos) = self.motor_keepalive_hold_pos[i].or_else(|| {
                    self.cached_states
                        .get(i)
                        .map(|s| s.position)
                        .filter(|p| p.is_finite())
                }) else {
                    missing_hold_for_due = true;
                    continue;
                };
                JointTarget::position(pos)
            };
            match self.send_plain_with_bypass(
                TxKind::MeasuredHold(i),
                Some(start_stamp),
                Some(Deadline { at: deadline }),
                false,
                due_bypass,
                |d| d.build_verified_hold_frame(i, &target),
            ) {
                Ok(_) => {
                    sent_mask |= 1u32 << i;
                    frames_sent = frames_sent.saturating_add(1);
                }
                Err(TxRefusal::DeadlinePassed | TxRefusal::Deferred) => break,
                Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => break,
                Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                    aborted_flush = true;
                    hit_stop = true;
                    break;
                }
                Err(TxRefusal::Transport(_)) => {
                    self.mark_tx_pass_backpressured();
                    aborted_flush = true;
                    break;
                }
                Err(_) => {}
            }
        }
        if !aborted_flush {
            for &i in &non_wd_posvel[..non_wd_len] {
                if frames_sent >= effective_max_frames || self.now() >= deadline {
                    break;
                }
                let Some(pos) = self.motor_keepalive_hold_pos[i].or_else(|| {
                    self.cached_states
                        .get(i)
                        .map(|s| s.position)
                        .filter(|p| p.is_finite())
                }) else {
                    continue;
                };
                let target = JointTarget::position(pos);
                match self.send_plain(
                    TxKind::MeasuredHold(i),
                    Some(start_stamp),
                    Some(Deadline { at: deadline }),
                    false,
                    |d| d.build_verified_hold_frame(i, &target),
                ) {
                    Ok(_) => {
                        sent_mask |= 1u32 << i;
                        frames_sent = frames_sent.saturating_add(1);
                    }
                    Err(TxRefusal::DeadlinePassed | TxRefusal::Deferred) => break,
                    Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => break,
                    Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                        aborted_flush = true;
                        hit_stop = true;
                        break;
                    }
                    Err(TxRefusal::Transport(_)) => {
                        self.mark_tx_pass_backpressured();
                        aborted_flush = true;
                        break;
                    }
                    Err(_) => {}
                }
            }
        }
        self.in_urgent_watchdog_hold = false;
        let mut refreshed_mask: u32 = 0;
        let mut flush_failed = false;
        if sent_mask != 0 {
            if aborted_flush
                || self.pending_stop_check(Some(start_stamp)).is_err()
                || self.flush_cutoff_exceeded()
                || (!self.can.flush_is_noop() && self.now() >= deadline)
            {
                self.discard_staged_ctrl_tx();
                if hit_stop {
                    return Err(crate::motor_mode::ModeSwitchFailure::Stopped);
                }
                flush_failed = true;
            } else if self.flush_can_and_commit().is_ok() {
                refreshed_mask = sent_mask;
                if self.mit_fence.is_some() {
                    let committed_at = self.now();
                    for idx in 0..n {
                        if (refreshed_mask & (1u32 << idx)) != 0 {
                            if let Some(slot) = self.mit_fence_last_cmd.get_mut(idx) {
                                *slot = Some(committed_at);
                            }
                        }
                    }
                }
            } else {
                flush_failed = true;
            }
        } else {
            self.discard_staged_ctrl_tx();
            if hit_stop {
                return Err(crate::motor_mode::ModeSwitchFailure::Stopped);
            }
        }
        if missing_hold_for_due || (flush_failed && due_len > 0) {
            return Err(crate::motor_mode::ModeSwitchFailure::WatchdogBudgetBreached);
        }

        // Also service an enabled, verified PosForce J7 joint via the single J7 emitter when
        // allowance remains and J7 is not excluded/uncertain. Stationary J7 Hold frames use
        // Hold-only `ContinuityBypass::Ledger` inside an active control release so zero ordinary
        // credits cannot starve J7 hold continuity (before or after MIT fence arming), while
        // ALWAYS enforcing `tx_budget.max_frames`, the release cutoff, and ModeSwitch `deadline`
        // at final submission and flush.
        let j7_bypass = if self.in_active_control_release() {
            ContinuityBypass::Ledger
        } else {
            self.j7_hold_continuity_bypass()
        };
        let j7_allowed = if self.in_active_control_release() {
            self.remaining_tick_tx_slots_with_bypass(j7_bypass) > 0
        } else {
            frames_sent < effective_max_frames
        };
        if j7_allowed && self.now() < deadline {
            if let Some(pf_j) = self.active_pos_force_joint() {
                if self.axis_is_energized(pf_j)
                    && (effective_exclude & (1u32 << pf_j)) == 0
                    && (self.release_ctrl_tx_mask & (1u32 << pf_j)) == 0
                {
                    let stamp = self.current_j7_intent_stamp();
                    if let Ok(
                        PosForceEmitOutcome::Sent { .. }
                        | PosForceEmitOutcome::SentHoldInsteadOfTarget { .. },
                    ) = self.emit_pos_force_j7_with_policy(
                        PosForceIntent::hold(stamp),
                        stamp,
                        Some(Deadline { at: deadline }),
                        j7_bypass,
                    )
                    {
                        frames_sent = frames_sent.saturating_add(1);
                        if pf_j < 32 {
                            refreshed_mask |= 1u32 << pf_j;
                        }
                        if self.mit_fence.is_some() {
                            let pf_now = self.now();
                            if let Some(slot) = self.mit_fence_last_cmd.get_mut(pf_j) {
                                *slot = Some(pf_now);
                            }
                        }
                    }
                }
            }
        }

        // Compute remaining max age across all energized watchdog-bearing confirmed axes and
        // check whether every due axis was refreshed.
        let after = self.now();
        let mut all_due_serviced = true;
        for &(i, _, _) in &due[..due_len] {
            if (refreshed_mask & (1u32 << i)) == 0 {
                all_due_serviced = false;
            }
        }
        let mut max_energized_age = Duration::ZERO;
        let mut oldest_joint: Option<u8> = None;
        for i in 0..n {
            if !self.axis_is_energized(i)
                || (effective_exclude & (1u32 << i)) != 0
                || !self.axis_has_watchdog(i)
                || self.verified_hold_mode(i).is_none()
            {
                continue;
            }
            let age = self.motor_last_ctrl_tx[i]
                .map_or(Duration::MAX, |t| after.saturating_duration_since(t));
            if oldest_joint.is_none() || age > max_energized_age {
                max_energized_age = age;
                oldest_joint = Some(i as u8);
            }
        }

        Ok(crate::motor_mode::WatchdogHoldResult {
            frames_sent,
            refreshed_mask,
            all_due_serviced,
            max_energized_age,
            oldest_joint,
        })
    }

    fn mode_switch_try_send_ctrl_mode_read(
        &mut self,
        joint: usize,
        deadline: Instant,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::motor_mode::ModeTxResult {
        if joint < self.mode_readback_boundary.len() {
            self.mode_readback_boundary[joint] = None;
        }
        match self.commissioning_try_send_register_read(
            joint,
            damiao::RID_CTRL_MODE,
            deadline,
            pre_tx_gate,
        ) {
            crate::mit::CommissioningTxStatus::Sent {
                tx_seq,
                tx_started,
                tx_offset_ns,
            } => crate::motor_mode::ModeTxResult::Sent {
                boundary_seq: tx_seq,
                tx_at: tx_started,
                tx_offset_ns,
            },
            crate::mit::CommissioningTxStatus::SentFlushFailed {
                tx_seq,
                tx_started,
                tx_offset_ns,
            } => crate::motor_mode::ModeTxResult::SentFlushFailed {
                boundary_seq: tx_seq,
                tx_at: tx_started,
                tx_offset_ns,
            },
            crate::mit::CommissioningTxStatus::Backpressure => {
                crate::motor_mode::ModeTxResult::Backpressure
            }
            crate::mit::CommissioningTxStatus::Revoked(c) => {
                crate::motor_mode::ModeTxResult::Revoked(c)
            }
            crate::mit::CommissioningTxStatus::Refused(c) => {
                crate::motor_mode::ModeTxResult::Refused(c)
            }
        }
    }

    fn mode_switch_try_send_ctrl_mode_write(
        &mut self,
        joint: usize,
        mode: crate::motor_mode::ControlMode,
        deadline: Instant,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::motor_mode::ModeTxResult {
        let Some(&id) = self.motor_ids.get(joint) else {
            return crate::motor_mode::ModeTxResult::Refused("joint out of range");
        };
        let Some(dm) = control_mode_to_damiao(mode) else {
            return crate::motor_mode::ModeTxResult::Refused("unsupported Damiao control mode");
        };
        // Unconditional wire-range verification gate before Impedance write.
        if dm == DamiaoControlMode::Mit
            && !self.mit_limits_verified.get(joint).copied().unwrap_or(false)
        {
            return crate::motor_mode::ModeTxResult::Refused("MIT wire ranges unverified");
        }
        let start_stamp = self.current_stop_stamp();
        let mode_val = Self::mode_value(dm);
        // RAM write opcode 0x55 only (NEVER flash store 0xAA).
        match self.send_with_commissioning_gate(
            TxKind::ModeWrite,
            Some(start_stamp),
            Some(Deadline { at: deadline }),
            pre_tx_gate,
            |_| Some(damiao::build_set_control_mode_frame(id, mode_val)),
        ) {
            Ok((_, tx_seq, tx_started, tx_offset_ns)) => {
                // Blocker 1 & 2: Frame was accepted by transport — IMMEDIATELY drop queued
                // targets for this joint, invalidate cached verified mode, and mark
                // `mode_write_uncertain_mask` BEFORE flush so even a failed flush or immediate
                // revocation can NEVER emit targets or holds in a stale mode.
                self.pending_tx.discard_joint(joint);
                self.pending_hold_tx.discard_joint(joint);
                self.verified_modes[joint] = None;
                self.joint_modes[joint] = None;
                self.mode_readback_boundary[joint] = None;
                if let Some(pf) = self.pos_force.as_ref() {
                    if pf.joint == joint {
                        self.pos_force_verified = false;
                    }
                }
                if joint < 32 {
                    self.mode_write_uncertain_mask |= 1u32 << joint;
                }
                if !self.can.flush_is_noop() {
                    if self.flush_cutoff_exceeded() || self.now() >= deadline {
                        return crate::motor_mode::ModeTxResult::SentFlushFailed {
                            boundary_seq: tx_seq,
                            tx_at: tx_started,
                            tx_offset_ns,
                        };
                    }
                    if self.flush_can_and_commit().is_err() {
                        return crate::motor_mode::ModeTxResult::SentFlushFailed {
                            boundary_seq: tx_seq,
                            tx_at: tx_started,
                            tx_offset_ns,
                        };
                    }
                }
                crate::motor_mode::ModeTxResult::Sent {
                    boundary_seq: tx_seq,
                    tx_at: tx_started,
                    tx_offset_ns,
                }
            }
            Err(Ok(code)) => crate::motor_mode::ModeTxResult::Revoked(code),
            Err(Err(TxRefusal::Deferred | TxRefusal::DeadlinePassed)) => {
                crate::motor_mode::ModeTxResult::Backpressure
            }
            Err(Err(TxRefusal::Transport(e))) if e.is_tx_backpressure() => {
                crate::motor_mode::ModeTxResult::Backpressure
            }
            Err(Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged)) => {
                crate::motor_mode::ModeTxResult::Revoked("STOPPED")
            }
            Err(Err(_)) => crate::motor_mode::ModeTxResult::Refused("CTRL_MODE write transmit failed"),
        }
    }

    fn mode_switch_try_send_range_read(
        &mut self,
        joint: usize,
        rid_idx: u8,
        deadline: Instant,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::motor_mode::ModeTxResult {
        let rid = match rid_idx {
            0 => damiao::RID_PMAX,
            1 => damiao::RID_VMAX,
            2 => damiao::RID_TMAX,
            _ => return crate::motor_mode::ModeTxResult::Refused("invalid range register index"),
        };
        match self.commissioning_try_send_register_read(joint, rid, deadline, pre_tx_gate) {
            crate::mit::CommissioningTxStatus::Sent {
                tx_seq,
                tx_started,
                tx_offset_ns,
            } => crate::motor_mode::ModeTxResult::Sent {
                boundary_seq: tx_seq,
                tx_at: tx_started,
                tx_offset_ns,
            },
            crate::mit::CommissioningTxStatus::SentFlushFailed {
                tx_seq,
                tx_started,
                tx_offset_ns,
            } => crate::motor_mode::ModeTxResult::SentFlushFailed {
                boundary_seq: tx_seq,
                tx_at: tx_started,
                tx_offset_ns,
            },
            crate::mit::CommissioningTxStatus::Backpressure => {
                crate::motor_mode::ModeTxResult::Backpressure
            }
            crate::mit::CommissioningTxStatus::Revoked(c) => {
                crate::motor_mode::ModeTxResult::Revoked(c)
            }
            crate::mit::CommissioningTxStatus::Refused(c) => {
                crate::motor_mode::ModeTxResult::Refused(c)
            }
        }
    }

    fn mode_switch_try_send_status_refresh(
        &mut self,
        joint: usize,
        deadline: Instant,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::motor_mode::ModeTxResult {
        let Some(&id) = self.motor_ids.get(joint) else {
            return crate::motor_mode::ModeTxResult::Refused("joint out of range");
        };
        let start_stamp = self.current_stop_stamp();
        match self.send_with_commissioning_gate(
            TxKind::Refresh,
            Some(start_stamp),
            Some(Deadline { at: deadline }),
            pre_tx_gate,
            |_| Some(damiao::build_refresh_frame(id)),
        ) {
            Ok((_, tx_seq, tx_started, tx_offset_ns)) => {
                if !self.can.flush_is_noop() {
                    if self.flush_cutoff_exceeded() || self.now() >= deadline {
                        return crate::motor_mode::ModeTxResult::SentFlushFailed {
                            boundary_seq: tx_seq,
                            tx_at: tx_started,
                            tx_offset_ns,
                        };
                    }
                    if self.flush_can_and_commit().is_err() {
                        return crate::motor_mode::ModeTxResult::SentFlushFailed {
                            boundary_seq: tx_seq,
                            tx_at: tx_started,
                            tx_offset_ns,
                        };
                    }
                }
                crate::motor_mode::ModeTxResult::Sent {
                    boundary_seq: tx_seq,
                    tx_at: tx_started,
                    tx_offset_ns,
                }
            }
            Err(Ok(code)) => crate::motor_mode::ModeTxResult::Revoked(code),
            Err(Err(TxRefusal::Deferred | TxRefusal::DeadlinePassed)) => {
                crate::motor_mode::ModeTxResult::Backpressure
            }
            Err(Err(TxRefusal::Transport(e))) if e.is_tx_backpressure() => {
                crate::motor_mode::ModeTxResult::Backpressure
            }
            Err(Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged)) => {
                crate::motor_mode::ModeTxResult::Revoked("STOPPED")
            }
            Err(Err(_)) => crate::motor_mode::ModeTxResult::Refused("status refresh transmit failed"),
        }
    }

    fn mode_switch_find_ctrl_mode_reply(
        &self,
        joint: usize,
        write: bool,
        after_seq: u64,
        min_rx_at: Instant,
        max_rx_at: Instant,
        tx_offset_ns: Option<i128>,
        require_expected_id: bool,
    ) -> crate::motor_mode::ModeRegMatch {
        // Major 1 confirmation closure: only conclude a reply match once RX has been observed
        // empty in the current pass so any duplicate reply in the same burst is also captured.
        if !self.last_read_drained {
            return crate::motor_mode::ModeRegMatch::Pending;
        }
        match self.commissioning_find_register_reply(
            joint,
            write,
            damiao::RID_CTRL_MODE,
            after_seq,
            min_rx_at,
            Some(max_rx_at),
            tx_offset_ns,
        ) {
            crate::mit::CommissioningRegMatch::Pending => crate::motor_mode::ModeRegMatch::Pending,
            crate::mit::CommissioningRegMatch::Ambiguous => {
                crate::motor_mode::ModeRegMatch::Ambiguous
            }
            crate::mit::CommissioningRegMatch::Matched(rep) => {
                if require_expected_id && rep.reply_id != self.expected_reply_id(joint) {
                    return crate::motor_mode::ModeRegMatch::Ambiguous;
                }
                let raw = u32::from_le_bytes(rep.value);
                let mode = Self::mode_from_u32(raw).map(damiao_mode_to_shadow_control_mode);
                crate::motor_mode::ModeRegMatch::Matched {
                    mode,
                    raw_value: raw,
                    rx_seq: rep.rx_seq,
                    rx_at: rep.rx_at,
                    decode_offset_ns: rep.decode_offset_ns,
                }
            }
        }
    }

    fn mode_switch_find_range_reply(
        &self,
        joint: usize,
        rid_idx: u8,
        after_seq: u64,
        min_rx_at: Instant,
        max_rx_at: Instant,
        tx_offset_ns: Option<i128>,
    ) -> crate::motor_mode::ModeRangeMatch {
        if !self.last_read_drained {
            return crate::motor_mode::ModeRangeMatch::Pending;
        }
        let rid = match rid_idx {
            0 => damiao::RID_PMAX,
            1 => damiao::RID_VMAX,
            2 => damiao::RID_TMAX,
            _ => return crate::motor_mode::ModeRangeMatch::Ambiguous,
        };
        match self.commissioning_find_register_reply(
            joint,
            false,
            rid,
            after_seq,
            min_rx_at,
            Some(max_rx_at),
            tx_offset_ns,
        ) {
            crate::mit::CommissioningRegMatch::Pending => {
                crate::motor_mode::ModeRangeMatch::Pending
            }
            crate::mit::CommissioningRegMatch::Ambiguous => {
                crate::motor_mode::ModeRangeMatch::Ambiguous
            }
            crate::mit::CommissioningRegMatch::Matched(rep) => {
                crate::motor_mode::ModeRangeMatch::Matched(f32::from_le_bytes(rep.value))
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn mode_switch_post_confirm_feedback(
        &self,
        joint: usize,
        mode: crate::motor_mode::ControlMode,
        after_seq: u64,
        after_at: Instant,
        max_rx_at: Instant,
        confirm_offset_ns: Option<i128>,
        now: Instant,
        require_disarmed_stationary: bool,
    ) -> crate::motor_mode::ModeFeedbackMatch {
        if !self.last_read_drained || joint >= self.motor_ids.len() {
            return crate::motor_mode::ModeFeedbackMatch::Pending;
        }
        let seq = self.last_feedback_seq.get(joint).copied().unwrap_or(0);
        if seq <= after_seq {
            return crate::motor_mode::ModeFeedbackMatch::Pending;
        }
        let Some(rx_at) = self.last_feedback_instant.get(joint).copied().flatten() else {
            return crate::motor_mode::ModeFeedbackMatch::Pending;
        };
        if rx_at < after_at || rx_at >= max_rx_at {
            return crate::motor_mode::ModeFeedbackMatch::Pending;
        }
        // R2 Major 7: enforce identical clock-offset domain presence AND drift rule as
        // `commissioning_find_register_reply` (reject mixed realtime/monotonic domains too!).
        let fb_offset_ns = self
            .last_feedback_decode_offset_ns
            .get(joint)
            .copied()
            .flatten();
        let offset_ok = match (confirm_offset_ns, fb_offset_ns) {
            (Some(c_off), Some(fb_off)) => {
                (fb_off - c_off).abs() <= MAX_REALTIME_MONOTONIC_OFFSET_DRIFT_NS
            }
            (None, None) => true,
            _ => false,
        };
        if !offset_ok {
            return crate::motor_mode::ModeFeedbackMatch::Pending;
        }
        let max_age = if mode == crate::motor_mode::ControlMode::PositionForce {
            POS_FORCE_FEEDBACK_MAX_AGE
        } else {
            DAMIAO_FEEDBACK_VALID_WINDOW
        };
        let Some(pos) = self.fresh_measured_position_at(joint, max_age, now) else {
            return crate::motor_mode::ModeFeedbackMatch::Pending;
        };
        if !pos.is_finite() {
            return crate::motor_mode::ModeFeedbackMatch::Unsafe;
        }
        let status = self.motor_status.get(joint).copied().unwrap_or(0xFF);
        if damiao_fault_name(status).is_some() {
            return crate::motor_mode::ModeFeedbackMatch::Unsafe;
        }
        if require_disarmed_stationary {
            let vel = self
                .cached_states
                .get(joint)
                .map_or(f32::NAN, |s| s.velocity);
            let still = vel.is_finite() && vel.abs() <= 0.05;
            if !still || status != 0 {
                return crate::motor_mode::ModeFeedbackMatch::Unsafe;
            }
        }
        crate::motor_mode::ModeFeedbackMatch::FreshConfirmed
    }
}

impl<C: CanTransport> JointModeExt for DamiaoArmDriver<C> {
    fn joint_hold_mode_verified(&self, joint: usize) -> bool {
        self.joint_hold_mode_verified_inner(joint)
    }

    fn request_mode_readback_budgeted(&mut self, unverified_mask: u64) -> bool {
        /// Retry spacing (< one 20 ms 50 Hz tick) so a lost SAFE-HOLD recovery `REG_READ` reply
        /// retries on the very next control release (t = 60 ms) and confirmed-mode holding resumes
        /// at t = 80 ms, safely below Damiao's 100 ms `TIMEOUT = 2000` hardware watchdog.
        const SAFE_HOLD_PROBE_RETRY_INTERVAL: Duration = Duration::from_millis(15);
        /// Maximum causal reply window for a SAFE-HOLD recovery probe (allows one 20 ms 50 Hz tick
        /// plus scheduling jitter while rejecting stale replies beyond 35 ms).
        const SAFE_HOLD_PROBE_DEADLINE: Duration = Duration::from_millis(35);

        // Never issue background/Step-2 recovery readbacks or clear `comm_reg_replies` for any
        // axis currently owned by an active `ModeSwitch` transaction.
        let unverified_mask = unverified_mask & !u64::from(self.mode_switch_active_mask);
        if unverified_mask == 0
            || self.torque_hs.is_some()
            || self.mit_expiry_budgeted_boundary.is_some()
            || self.awaiting_rx_boundary
            || !self.last_read_drained
        {
            return false;
        }
        let mut sent_any = false;
        let mut cleared_comm_log = false;
        let mut drained_before_retry = false;
        let mut staged_uncertain =
            [None::<(u64, Instant, Instant, Instant, Option<i128>)>; MAX_MODE_JOINTS];
        let dof = self.motor_ids.len().min(64);
        for i in 0..dof {
            if (unverified_mask & (1u64 << i)) == 0 || self.verified_hold_mode(i).is_some() {
                continue;
            }
            let uncertain = self.is_axis_mode_uncertain(i);
            let now = self.now();
            if uncertain
                && i < MAX_MODE_JOINTS
                && self.uncertain_readback_probe[i]
                    .is_some_and(|(_, _, retry_at, _, _)| now < retry_at)
            {
                // Avoid sending duplicate in-flight REG_READ probes within the same retry interval,
                // while retrying promptly on the very next 50 Hz release if the reply was lost.
                continue;
            }
            if uncertain && i < MAX_MODE_JOINTS && self.uncertain_readback_probe[i].is_some() {
                // Before issuing a retry probe, perform one bounded RX drain so any late/promoted
                // reply to the prior probe is matched (or cleared out of RX) BEFORE capturing the
                // retry's causal TX boundary.
                if !drained_before_retry {
                    let _ = self.receive_feedback();
                    drained_before_retry = true;
                }
                if !self.last_read_drained {
                    break;
                }
                if self.verified_hold_mode(i).is_some() {
                    continue;
                }
            }
            if uncertain && !cleared_comm_log {
                self.comm_reg_reply_len = 0;
                self.comm_reg_reply_overflow = false;
                cleared_comm_log = true;
            }
            let id = self.motor_ids[i];
            let mut revoked: Option<&'static str> = None;
            // Use `self.send(...)` directly so `seq_before`, `tx_started`, and `tx_offset_ns`
            // reflect the actual TX submission boundary (never a pre-send sample).
            match self.send(
                TxKind::Refresh,
                None,
                None,
                false,
                ContinuityBypass::None,
                None,
                &mut revoked,
                |_| Some(damiao::build_read_register_frame(id, damiao::RID_CTRL_MODE)),
            ) {
                Ok((_frame, seq_before, tx_started, tx_offset_ns)) => {
                    if uncertain && i < MAX_MODE_JOINTS {
                        let retry_at = tx_started
                            .checked_add(SAFE_HOLD_PROBE_RETRY_INTERVAL)
                            .unwrap_or(tx_started);
                        let probe_deadline = tx_started
                            .checked_add(SAFE_HOLD_PROBE_DEADLINE)
                            .unwrap_or(tx_started);
                        staged_uncertain[i] = Some((
                            seq_before,
                            tx_started,
                            retry_at,
                            probe_deadline,
                            tx_offset_ns,
                        ));
                    } else {
                        self.mode_readback_boundary[i].get_or_insert(seq_before);
                    }
                    sent_any = true;
                }
                Err(TxRefusal::Deferred | TxRefusal::DeadlinePassed) => break,
                Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => break,
                Err(_) => break,
            }
        }
        if sent_any && self.flush_can_and_commit().is_ok() {
            for (i, probe) in staged_uncertain.into_iter().enumerate() {
                if let Some(p) = probe {
                    self.uncertain_readback_probe[i] = Some(p);
                }
            }
            return true;
        }
        false
    }

    fn supports_mit_mode_switch(&self) -> bool {
        self.control_mode == DamiaoControlMode::PosVel
    }

    fn set_joint_modes(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        // MIT tracking owns the joints (fence armed) or lost them (tripped): only the fenced,
        // confirmed switch may touch the modes
        if self.mit_fence.is_some() || self.mit_fence_trip.is_some() {
            return Err(ArmDriverError::HardwareFault("MIT tracking holds the deadline fence: plain mode switch refused".into()));
        }
        self.discard_all_pending_tx();
        self.begin_tx_tick();
        self.set_joint_modes_impl(modes, hold)
    }

    fn set_joint_modes_in_tick(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        if self.mit_fence.is_some() || self.mit_fence_trip.is_some() {
            return Err(ArmDriverError::HardwareFault("MIT tracking holds the deadline fence: plain mode switch refused".into()));
        }
        self.discard_all_pending_tx();
        let _ = self.ensure_tx_pass();
        self.set_joint_modes_impl(modes, hold)
    }

    fn set_joint_modes_confirmed(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.switch_joint_modes_confirmed(modes, hold)
    }

    fn joint_modes_uncertain(&self) -> bool {
        self.joint_modes.iter().any(Option::is_none)
    }

    fn set_strict_mode_handover(&mut self, strict: bool) {
        self.strict_mode_handover = strict;
    }

    fn set_mode_switch_hold_targets(&mut self, targets: &[Option<ModeSwitchHoldTarget>]) {
        self.mode_switch_hold_targets = [None; MAX_MODE_JOINTS];
        let n = targets.len().min(MAX_MODE_JOINTS);
        self.mode_switch_hold_targets[..n].copy_from_slice(&targets[..n]);
    }
    fn take_recovery_writes_into(&mut self, out: &mut Vec<usize>) {
        out.clear();
        out.extend_from_slice(&self.recovery_written_joints);
        self.recovery_written_joints.clear();
    }
    fn with_range_check(&self, f: &mut dyn FnMut(Option<&DriverRangeCheck>)) { f(self.range_check.as_ref()) }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DamiaoControlMode {
    PosVel,
    Mit,
    /// Position-force (CTRL_MODE 4): every frame carries a motor current limit.
    PosForce,
}

/// Stop-gate policy for a mode-write transaction (`write_modes_send` / `write_modes_confirmed`).
#[derive(Clone, Copy, Debug)]
pub(in crate::driver::can_servo) enum ModeWritePolicy {
    /// Torque-enable base-mode restore handshake: enforces strict pending + latched +
    /// enable-stamp checks and tears down `torque_hs` on inhibition.
    EnableRestore,
    /// Forward mode switch (or non-base-restore rollback): refuses any transmit while a
    /// stop or E-stop is pending, latched, or advances past the captured stamp.
    Strict(StopStamp),
    /// Base-mode rollback (every changing joint restoring `base_mode(i)`):
    /// refused while a stop/E-stop is pending; allowed while latched ONLY with fresh
    /// (`<= 100 ms`) measured feedback, rebuilding every hold at the fresh measured position.
    BaseRollback { start_stamp: StopStamp, force_fresh_hold: bool },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(in crate::driver::can_servo) enum ReadbackStage {
    CtrlMode,
    Timeout,
}

/// Pre-enable `CTRL_MODE` and `TIMEOUT` readback (L2 + power-overload PR 1): verifies
/// `verified_modes[j] == Some(base_mode(j))` and fresh causal `RID_TIMEOUT` replies before any
/// `CMD_ENABLE` frame is transmitted. Stored inline with bitmasks (zero heap allocation).
#[derive(Debug)]
pub(in crate::driver::can_servo) struct ModeReadback {
    pub(in crate::driver::can_servo) stage: ReadbackStage,
    pub(in crate::driver::can_servo) remaining_mask: u32,
    pub(in crate::driver::can_servo) sent_mask: u32,
    pub(in crate::driver::can_servo) timeout_pending_mask: u32,
    pub(in crate::driver::can_servo) log_cleared: bool,
    pub(in crate::driver::can_servo) tx_evidence: [Option<(u64, Instant, Option<i128>)>; MAX_MODE_JOINTS],
    pub(in crate::driver::can_servo) deadline: Deadline,
}

/// Maximum joints supported in inline stack-allocated [`ModeWrite`] buffers.
pub(in crate::driver) const MAX_MODE_JOINTS: usize = 16;
