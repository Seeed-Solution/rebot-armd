//! Damiao bounded E-stop hold TX implementation.

use std::time::Instant;

use crate::driver::*;
use crate::driver::can_servo::*;
use crate::transport::{CanTransport, TransportError};

impl<C: CanTransport> DamiaoArmDriver<C> {
    pub(in crate::driver::can_servo) fn write_estop_hold_bounded_impl(&mut self, targets: &[JointTarget]) -> Result<TxBatchOutcome, ArmDriverError> {
        self.discard_pending_tx_stopped();
        self.last_hold_tx_sent_any = false;
        self.last_hold_tx_due_mask = 0;
        self.last_hold_tx_sent_mask = 0;
        self.last_expired_oldest_age = None;
        self.last_tx_pf_status = PosForceTxStatus::Unobserved;
        if targets.len() != self.motor_ids.len() {
            self.reject_target_tx(MitFenceTripReason::PreflightRejected);
            return Err(ArmDriverError::JointCountMismatch {
                expected: self.motor_ids.len(),
                actual: targets.len(),
            });
        }
        if self.torque_hs.is_some()
            && !matches!(
                self.torque_hs,
                Some(
                    DamiaoTorque::AwaitRestGate { .. }
                        | DamiaoTorque::Disable { repeats_sent: 0, .. }
                )
            )
        {
            self.reject_target_tx(MitFenceTripReason::PreflightRejected);
            return Err(ArmDriverError::TorqueTransitionPending);
        }
        let fenced = self.mit_fence.is_some() || self.mit_fence_trip.is_some();
        // Stopped-hold NEVER validates caller's J7 target (so an invalid caller target can NEVER
        // suppress J7 hold/freeze); it only requires mode confirmation (`targets = None`).
        if let Some(e) = self.target_frames_refused(None, true) {
            self.reject_target_tx(MitFenceTripReason::PreflightRejected);
            return Err(e);
        }
        let total_joints = targets.len().min(u8::MAX as usize) as u8;
        let arm_res: Result<TxBatchOutcome, ArmDriverError> = if fenced {
            self.discard_pending_hold_tx();
            if let Err(e) = self.pending_stop_check(None) {
                self.trip_fence(MitFenceTripReason::EstopBeforeBatch { total_joints });
                return Err(e);
            }
            let start_stamp = self.current_stop_stamp();
            (|| -> Result<TxBatchOutcome, ArmDriverError> {
                self.fence_check()?;
                let pass = self.ensure_tx_pass();
                let mut started = [None::<Instant>; 16];
                if targets.len() > started.len() {
                    self.reject_target_tx(MitFenceTripReason::PreflightRejected);
                    return Err(ArmDriverError::JointCountMismatch {
                        expected: started.len(),
                        actual: targets.len(),
                    });
                }
                let mut batch_frames = 0usize;
                let mut due_mask: u64 = 0;
                let mut active_mask: u64 = 0;
                let mut sent_any = false;
                let mut sent_mask: u64 = 0;
                let mut sent_slot_mask: u64 = 0;
                for (i, target) in targets.iter().enumerate() {
                    if self.joint_modes.get(i).copied().flatten() == Some(DamiaoControlMode::PosForce) {
                        continue;
                    }
                    let count = match self.target_frames_checked(i, target) {
                        Ok((frames, _)) => frames.into_iter().flatten().count(),
                        Err(e) => {
                            self.trip_fence(MitFenceTripReason::TargetUnavailable {
                                joint: (i + 1) as u8,
                                completed_joints: 0,
                                total_joints,
                            });
                            return Err(e);
                        }
                    };
                    if count > 0 && i < 64 {
                        active_mask |= 1u64 << i;
                    }
                    batch_frames += count;
                    for slot in 0..count.min(2) {
                        let bit = i.saturating_mul(2).saturating_add(slot);
                        if bit < 64 {
                            due_mask |= 1u64 << bit;
                        }
                    }
                }
                if pass.backpressured {
                    let reason = MitFenceTripReason::TxBackpressuredBeforeBatch;
                    self.trip_fence(reason);
                    return Err(ArmDriverError::MitFenceTripped(reason));
                }
                if pass.attempts.saturating_add(batch_frames) > self.tx_budget.max_frames {
                    let remaining = self.tx_budget.max_frames.saturating_sub(pass.attempts);
                    let reason = MitFenceTripReason::TxBudgetExceededPreflight {
                        needed: batch_frames.min(u16::MAX as usize) as u16,
                        remaining: remaining.min(u16::MAX as usize) as u16,
                        max: self.tx_budget.max_frames.min(u16::MAX as usize) as u16,
                    };
                    self.trip_fence(reason);
                    return Err(ArmDriverError::MitFenceTripped(reason));
                }
                if !self.in_active_control_release() && self.now() >= self.effective_tx_pass_deadline(pass) {
                    let reason = MitFenceTripReason::TxBudgetDeadlineBeforeBatch;
                    self.trip_fence(reason);
                    return Err(ArmDriverError::MitFenceTripped(reason));
                }
                if self.now() >= pass.deadline {
                    let reason = MitFenceTripReason::TxBudgetDeadlineBeforeBatch;
                    self.trip_fence(reason);
                    return Err(ArmDriverError::MitFenceTripped(reason));
                }
                // Watchdog-continuity holds for an armed MIT fence are NEVER deferrable by the
                // per-release ledger (`release_tx_limit`): send the complete hold batch within
                // `self.tx_budget.max_frames`.
                let hold_bypass = if self.in_active_control_release()
                    && self.tx_release_deadline.is_some_and(|cap| self.now() >= cap)
                {
                    ContinuityBypass::LedgerAndReleaseDeadline
                } else {
                    ContinuityBypass::Ledger
                };
                let mut fell_back_partial = false;
                let loop_res: Result<(), ArmDriverError> = (|| {
                    for (i, target) in targets.iter().enumerate() {
                        if self.joint_modes.get(i).copied().flatten() == Some(DamiaoControlMode::PosForce) {
                            continue;
                        }
                        let slot_count = match self.target_frames_checked(i, target) {
                            Ok((frames, _)) => frames.into_iter().flatten().count(),
                            Err(e) => {
                                self.trip_fence(MitFenceTripReason::TargetUnavailable {
                                    joint: (i + 1) as u8,
                                    completed_joints: i as u8,
                                    total_joints,
                                });
                                return Err(e);
                            }
                        };
                        for slot in 0..slot_count {
                            if started[i].is_none() {
                                started[i] = Some(self.now());
                            }
                            match self.send_plain_with_bypass(
                                TxKind::RollbackHold(i),
                                Some(start_stamp),
                                None,
                                true,
                                hold_bypass,
                                |d| {
                                    let fresh_pos = d.fresh_measured_position(i, POS_FORCE_FEEDBACK_MAX_AGE)?;
                                    let hold_target = JointTarget {
                                        position: fresh_pos,
                                        velocity: Some(0.0),
                                        kp: target.kp,
                                        kd: target.kd,
                                        torque: target.torque,
                                    };
                                    d.target_frames_checked(i, &hold_target)
                                        .ok()?
                                        .0
                                        .get(slot)
                                        .copied()
                                        .flatten()
                                },
                            ) {
                                Ok(sent_frame) => {
                                    let mit_params = self.resolved_mit_params(target);
                                    self.record_sent_target(i, sent_frame, Some(mit_params));
                                    sent_any = true;
                                    if i < 64 {
                                        sent_mask |= 1u64 << i;
                                    }
                                    let bit = i.saturating_mul(2).saturating_add(slot);
                                    if bit < 64 {
                                        sent_slot_mask |= 1u64 << bit;
                                    }
                                }
                                Err(TxRefusal::PosForceFeedbackExpired(joint)) => {
                                    self.trip_fence(MitFenceTripReason::TargetUnavailable {
                                        joint: (joint + 1) as u8,
                                        completed_joints: i as u8,
                                        total_joints,
                                    });
                                    return Err(ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint });
                                }
                                Err(TxRefusal::MitFenceTripped(reason)) => {
                                    return Err(ArmDriverError::MitFenceTripped(reason));
                                }
                                Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                                    self.trip_fence(MitFenceTripReason::EstopDuringBatch {
                                        completed_joints: i as u8,
                                        total_joints,
                                    });
                                    return Err(ArmDriverError::EstopInhibited);
                                }
                                Err(TxRefusal::BuildRefused) => {
                                    self.trip_fence(MitFenceTripReason::TargetUnavailable {
                                        joint: (i + 1) as u8,
                                        completed_joints: i as u8,
                                        total_joints,
                                    });
                                    if self.fresh_measured_position(i, POS_FORCE_FEEDBACK_MAX_AGE).is_none() {
                                        return Err(ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint: i });
                                    }
                                    return Err(ArmDriverError::PosForceTargetUnavailable { joint: i });
                                }
                                Err(TxRefusal::Deferred | TxRefusal::DeadlinePassed)
                                    if self.in_active_control_release() && sent_mask != 0 =>
                                {
                                    if self.pending_stop_check(Some(start_stamp)).is_err() {
                                        self.trip_fence(MitFenceTripReason::EstopDuringBatch {
                                            completed_joints: i as u8,
                                            total_joints,
                                        });
                                        return Err(ArmDriverError::EstopInhibited);
                                    }
                                    self.fence_check()?;
                                    let covered = sent_mask;
                                    match self.send_mit_continuity_holds_for_uncovered(
                                        active_mask,
                                        covered,
                                        None,
                                        start_stamp,
                                        true,
                                        &mut started,
                                        &mut sent_any,
                                        &mut sent_mask,
                                        &mut sent_slot_mask,
                                    ) {
                                        Ok(()) => {
                                            fell_back_partial = true;
                                            return Ok(());
                                        }
                                        Err((fj, TxRefusal::StopInhibited | TxRefusal::StopStampChanged)) => {
                                            self.trip_fence(MitFenceTripReason::EstopDuringBatch {
                                                completed_joints: fj as u8,
                                                total_joints,
                                            });
                                            return Err(ArmDriverError::EstopInhibited);
                                        }
                                        Err((_, TxRefusal::MitFenceTripped(reason))) => {
                                            return Err(ArmDriverError::MitFenceTripped(reason));
                                        }
                                        Err((fj, TxRefusal::Transport(e))) => {
                                            self.mark_tx_pass_backpressured();
                                            self.trip_fence(MitFenceTripReason::TransmitFailed {
                                                joint: (fj + 1) as u8,
                                                completed_joints: fj as u8,
                                                total_joints,
                                                kind: TxFailKind::from_transport(&e),
                                            });
                                            return Err(e.into());
                                        }
                                        Err((fj, _)) => {
                                            let reason = MitFenceTripReason::TxBudgetExhausted {
                                                completed_joints: fj as u8,
                                                total_joints,
                                            };
                                            self.trip_fence(reason);
                                            return Err(ArmDriverError::MitFenceTripped(reason));
                                        }
                                    }
                                }
                                Err(TxRefusal::Deferred) => {
                                    let reason = MitFenceTripReason::TxBudgetExhausted {
                                        completed_joints: i as u8,
                                        total_joints,
                                    };
                                    self.trip_fence(reason);
                                    return Err(ArmDriverError::MitFenceTripped(reason));
                                }
                                Err(TxRefusal::Transport(e)) => {
                                    if self.in_active_control_release() && sent_mask != 0 {
                                        if self.pending_stop_check(Some(start_stamp)).is_err() {
                                            self.trip_fence(MitFenceTripReason::EstopBetweenAttempts {
                                                joint: (i + 1) as u8,
                                                completed_joints: i as u8,
                                                total_joints,
                                            });
                                            return Err(ArmDriverError::EstopInhibited);
                                        }
                                        if self.fence_check().is_ok() {
                                            let covered = sent_mask;
                                            if self
                                                .send_mit_continuity_holds_for_uncovered(
                                                    active_mask,
                                                    covered,
                                                    None,
                                                    start_stamp,
                                                    true,
                                                    &mut started,
                                                    &mut sent_any,
                                                    &mut sent_mask,
                                                    &mut sent_slot_mask,
                                                )
                                                .is_ok()
                                            {
                                                fell_back_partial = true;
                                                return Ok(());
                                            }
                                        }
                                    }
                                    self.mark_tx_pass_backpressured();
                                    if self.pending_stop_check(Some(start_stamp)).is_err() {
                                        self.trip_fence(MitFenceTripReason::EstopBetweenAttempts {
                                            joint: (i + 1) as u8,
                                            completed_joints: i as u8,
                                            total_joints,
                                        });
                                        return Err(ArmDriverError::EstopInhibited);
                                    }
                                    self.last_hold_tx_due_mask = due_mask;
                                    self.last_hold_tx_sent_mask = if self.can.flush_is_noop() { sent_slot_mask } else { 0 };
                                    self.trip_fence(MitFenceTripReason::TransmitFailed {
                                        joint: (i + 1) as u8,
                                        completed_joints: i as u8,
                                        total_joints,
                                        kind: TxFailKind::from_transport(&e),
                                    });
                                    return Err(e.into());
                                }
                                Err(TxRefusal::DeadlinePassed) => {
                                    let reason = MitFenceTripReason::TxBudgetDeadlineBeforeBatch;
                                    self.trip_fence(reason);
                                    return Err(ArmDriverError::MitFenceTripped(reason));
                                }
                                Err(r @ (TxRefusal::TorqueOffWouldDrop | TxRefusal::DisableWaitEvidence | TxRefusal::DisableRequestExpired)) => {
                                    return Err(Self::refusal_to_driver_err(r));
                                }
                                Err(TxRefusal::LeaseFenced | TxRefusal::EnableAuthorityExpired) => {
                                    unreachable!("TxKind::RollbackHold is exempt from lease fence");
                                }
                            }
                        }
                    }
                    Ok(())
                })();
                loop_res?;
                if self.pending_stop_check(Some(start_stamp)).is_err() {
                    self.trip_fence(MitFenceTripReason::EstopBeforeFlush { total_joints });
                    return Err(ArmDriverError::EstopInhibited);
                }
                if sent_any {
                    let flush_blocked = if self.in_active_control_release() {
                        !self.can.flush_is_noop()
                            && self.tick_tx.is_some_and(|p| self.now() >= p.deadline)
                            && !fell_back_partial
                    } else {
                        self.flush_cutoff_exceeded()
                    };
                    if flush_blocked {
                        self.mark_tx_pass_backpressured();
                        self.discard_staged_ctrl_tx();
                        self.last_hold_tx_sent_any = false;
                        self.last_hold_tx_due_mask = due_mask;
                        self.last_hold_tx_sent_mask = 0;
                        self.trip_fence(MitFenceTripReason::FlushFailed {
                            kind: TxFailKind::Timeout,
                        });
                        return Err(TransportError::Timeout.into());
                    }
                    if let Err(e) = self.flush_can_and_commit() {
                        self.mark_tx_pass_backpressured();
                        self.last_hold_tx_sent_any = false;
                        if self.pending_stop_check(Some(start_stamp)).is_err() {
                            self.trip_fence(MitFenceTripReason::EstopAfterFlush { total_joints });
                            return Err(ArmDriverError::EstopInhibited);
                        }
                        self.last_hold_tx_due_mask = due_mask;
                        self.last_hold_tx_sent_mask = 0;
                        self.trip_fence(MitFenceTripReason::FlushFailed {
                            kind: TxFailKind::from_transport(&e),
                        });
                        return Err(e.into());
                    }
                }
                if self.pending_stop_check(Some(start_stamp)).is_err() {
                    self.trip_fence(MitFenceTripReason::EstopAfterFlush { total_joints });
                    return Err(ArmDriverError::EstopInhibited);
                }
                self.fence_check()?;
                for (i, t) in started.into_iter().enumerate().take(targets.len()) {
                    if t.is_some() {
                        self.mit_fence_last_cmd[i] = t;
                        self.record_flushed_joint_tx(i, 0, false);
                    }
                }
                self.last_hold_tx_sent_any = sent_any;
                self.last_hold_tx_due_mask = due_mask;
                self.last_hold_tx_sent_mask = sent_slot_mask;
                Ok(TxBatchOutcome::Complete)
            })()
        } else {
            // Unfenced bounded arm-only stopped-hold path: carries deferred holds for J1–J6 inline
            // in `pending_hold_tx` with round-robin `hold_cursor` rotation. Active PosForce J7 is
            // NEVER staged here; it is emitted strictly once via `emit_pos_force_j7_impl`.
            if let Err(e) = self.pending_stop_check(None) {
                self.discard_pending_hold_tx();
                return Err(e);
            }
            self.output_ran_since_rx = true;
            self.last_output_was_hold = true;
            let start_stamp = self.current_stop_stamp();
            let _ = self.drop_stopped_hold_tx(start_stamp);
            let cleanup_now = self.now();
            let (pre_dropped_stale, mut oldest_expired) =
                self.pending_hold_tx.drop_stale(cleanup_now, self.tx_budget.frame_max_age);
            if pre_dropped_stale > 0 {
                crate::diagnostics::tick::record_tx_dropped_stale(pre_dropped_stale as u32);
            }

            let n = targets.len();
            if n > MAX_MODE_JOINTS {
                self.discard_pending_hold_tx();
                return Err(ArmDriverError::JointCountMismatch { expected: MAX_MODE_JOINTS, actual: n });
            }
            let mut staged_ids: [[Option<u32>; 2]; MAX_MODE_JOINTS] = [[None; 2]; MAX_MODE_JOINTS];
            let mut new_keys = 0usize;
            let mut due_mask: u64 = 0;
            let mut first_build_refused_joint: Option<usize> = None;
            for (i, target) in targets.iter().enumerate() {
                if self.joint_modes.get(i).copied().flatten() == Some(DamiaoControlMode::PosForce) {
                    continue;
                }
                if self.fresh_measured_position_at(i, POS_FORCE_FEEDBACK_MAX_AGE, cleanup_now).is_none() {
                    first_build_refused_joint.get_or_insert(i);
                    self.pending_hold_tx.remove_joint(i);
                    continue;
                }
                let cmds = match self.pending_target_cmds_checked(i, target) {
                    Ok(v) => v,
                    Err(e) => {
                        self.discard_pending_hold_tx();
                        return Err(e);
                    }
                };
                for (slot, maybe_cmd) in cmds.into_iter().enumerate() {
                    if let Some((frame_id, _)) = maybe_cmd {
                        staged_ids[i][slot] = Some(frame_id);
                        let flat_bit = i.saturating_mul(2).saturating_add(slot);
                        if flat_bit < 64 {
                            due_mask |= 1u64 << flat_bit;
                        }
                        if self.pending_hold_tx.find_key(i, frame_id).is_none() {
                            new_keys += 1;
                        }
                    }
                }
            }
            if self.pending_hold_tx.len() + new_keys > DAMIAO_PENDING_TX_CAPACITY {
                let queued = self.pending_hold_tx.len();
                self.discard_pending_hold_tx();
                return Err(ArmDriverError::TxQueueFull {
                    queued,
                    new: new_keys,
                    capacity: DAMIAO_PENDING_TX_CAPACITY,
                });
            }

            let batch_ts = self.now();
            let total_slots = n.saturating_mul(2);
            let start_flat = if total_slots > 0 { self.hold_cursor % total_slots } else { 0 };
            for offset in 0..total_slots {
                let flat = (start_flat + offset) % total_slots;
                let i = flat / 2;
                let slot = flat % 2;
                if let Some(frame_id) = staged_ids[i][slot] {
                    let target = &targets[i];
                    let cmd = PendingTargetCmd::RollbackHold {
                        kp: target.kp,
                        kd: target.kd,
                        torque: target.torque,
                        slot: slot as u8,
                    };
                    let ok = self.pending_hold_tx.upsert(i, frame_id, cmd, batch_ts, start_stamp, 0);
                    debug_assert!(ok, "preflight checked pending_hold_tx capacity");
                }
            }

            let _ = self.ensure_tx_pass();
            self.discard_staged_ctrl_tx();
            let mut sent_any = false;
            let mut sent_slot_mask: u64 = 0;
            let mut sent_order = [(0usize, 0usize); MAX_MODE_JOINTS * 2];
            let mut sent_order_len = 0usize;
            let mut dropped_stale_in_pass = 0usize;
            (|| -> Result<TxBatchOutcome, ArmDriverError> {
                while let Some((idx, entry, arm_class, arm_seq)) = self
                    .pending_hold_tx
                    .peek_best_hold(&self.slot_in_unchanged_hold, &self.slot_last_tx_seq)
                {
                    let rem = self.remaining_tick_tx_slots();
                    if rem == 0 || !self.can_start_tick_tx() {
                        break;
                    }
                    // Priority by change across ALL joints (J1–J6 and J7): if taking this slot
                    // leaves 0 slots for the J7 epilogue, yield whenever J7 outranks `entry` by
                    // `(priority_class, slot_last_tx_seq)` (Class 1 transition before Class 3
                    // unchanged hold re-send; oldest waiting slot first within a class).
                    if rem == 1 {
                        if let Some((j7_class, j7_seq)) = self.j7_tx_priority(true) {
                            if (j7_class, j7_seq) < (arm_class, arm_seq) {
                                break;
                            }
                        }
                    }
                    let i = entry.joint;
                    let entry_deadline = Deadline::after(entry.queued_at, self.tx_budget.frame_max_age);
                    let PendingTargetCmd::RollbackHold { kp, kd, torque, slot } = entry.cmd else {
                        let _ = self.pending_hold_tx.remove_at(idx);
                        continue;
                    };
                    match self.send_plain(
                        TxKind::RollbackHold(i),
                        Some(entry.stop_stamp),
                        Some(entry_deadline),
                        false,
                        |d| {
                            let fresh_pos = d.fresh_measured_position(i, POS_FORCE_FEEDBACK_MAX_AGE)?;
                            let hold_target = JointTarget {
                                position: fresh_pos,
                                velocity: Some(0.0),
                                kp,
                                kd,
                                torque,
                            };
                            d.target_frames_checked(i, &hold_target)
                                .ok()?
                                .0
                                .get(usize::from(slot))
                                .copied()
                                .flatten()
                        },
                    ) {
                        Ok(sent_frame) => {
                            let _ = self.pending_hold_tx.remove_at(idx);
                            let mit_params = (
                                kp.filter(|k| k.is_finite()).unwrap_or(self.default_kp),
                                kd.filter(|k| k.is_finite()).unwrap_or(self.default_kd),
                                torque.filter(|t| t.is_finite()).unwrap_or(0.0),
                            );
                            self.record_sent_target(i, sent_frame, Some(mit_params));
                            sent_any = true;
                            if sent_order_len < sent_order.len() {
                                sent_order[sent_order_len] = (i, usize::from(slot));
                                sent_order_len += 1;
                            }
                            let flat_bit = i.saturating_mul(2).saturating_add(usize::from(slot));
                            if flat_bit < 64 {
                                sent_slot_mask |= 1u64 << flat_bit;
                            }
                            if total_slots > 0 {
                                self.hold_cursor = (i * 2 + usize::from(slot) + 1) % total_slots;
                            }
                        }
                        Err(TxRefusal::PosForceFeedbackExpired(_)) => {
                            let _ = self.pending_hold_tx.remove_at(idx);
                            continue;
                        }
                        Err(TxRefusal::DeadlinePassed) => {
                            let now = self.now();
                            let _ = self.pending_hold_tx.remove_at(idx);
                            let age = now.saturating_duration_since(entry.queued_at);
                            oldest_expired = Some(oldest_expired.map_or(age, |o| o.max(age)));
                            dropped_stale_in_pass += 1;
                            crate::diagnostics::tick::record_tx_dropped_stale(1);
                        }
                        Err(TxRefusal::Deferred) => break,
                        Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                            self.last_hold_tx_sent_any = sent_any && self.can.flush_is_noop();
                            self.last_hold_tx_due_mask = 0;
                            self.last_hold_tx_sent_mask = 0;
                            self.discard_pending_hold_tx();
                            return Err(ArmDriverError::EstopInhibited);
                        }
                        Err(TxRefusal::BuildRefused) => {
                            let flat_bit = i.saturating_mul(2).saturating_add(usize::from(slot));
                            if flat_bit < 64 {
                                due_mask &= !(1u64 << flat_bit);
                            }
                            first_build_refused_joint.get_or_insert(i);
                            let _ = self.pending_hold_tx.remove_at(idx);
                            continue;
                        }
                        Err(TxRefusal::MitFenceTripped(reason)) => {
                            self.last_hold_tx_sent_any = sent_any && self.can.flush_is_noop();
                            self.last_hold_tx_due_mask = 0;
                            self.last_hold_tx_sent_mask = 0;
                            self.discard_pending_hold_tx();
                            return Err(ArmDriverError::MitFenceTripped(reason));
                        }
                        Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => {
                            if self.pending_stop_check(Some(start_stamp)).is_err() {
                                self.last_hold_tx_sent_any = sent_any && self.can.flush_is_noop();
                                self.last_hold_tx_due_mask = 0;
                                self.last_hold_tx_sent_mask = 0;
                                self.discard_pending_hold_tx();
                                return Err(ArmDriverError::EstopInhibited);
                            }
                            break;
                        }
                        Err(TxRefusal::Transport(e)) => {
                            self.last_hold_tx_sent_any = sent_any && self.can.flush_is_noop();
                            self.mark_tx_pass_backpressured();
                            self.discard_pending_hold_tx();
                            if self.pending_stop_check(Some(start_stamp)).is_err() {
                                self.last_hold_tx_due_mask = 0;
                                self.last_hold_tx_sent_mask = 0;
                                return Err(ArmDriverError::EstopInhibited);
                            }
                            self.last_hold_tx_due_mask = due_mask;
                            self.last_hold_tx_sent_mask = if self.can.flush_is_noop() { sent_slot_mask } else { 0 };
                            return Err(e.into());
                        }
                        // Disable-only refusals; a RollbackHold never produces them, but fail closed.
                        Err(r @ (TxRefusal::TorqueOffWouldDrop | TxRefusal::DisableWaitEvidence | TxRefusal::DisableRequestExpired)) => {
                            self.last_hold_tx_sent_any = sent_any && self.can.flush_is_noop();
                            self.last_hold_tx_due_mask = 0;
                            self.last_hold_tx_sent_mask = 0;
                            self.discard_pending_hold_tx();
                            return Err(Self::refusal_to_driver_err(r));
                        }
                        Err(TxRefusal::LeaseFenced | TxRefusal::EnableAuthorityExpired) => {
                            unreachable!("TxKind::RollbackHold is exempt from lease fence");
                        }
                    }
                }

                if self.pending_stop_check(Some(start_stamp)).is_err() {
                    self.last_hold_tx_sent_any = sent_any && self.can.flush_is_noop();
                    self.last_hold_tx_due_mask = 0;
                    self.last_hold_tx_sent_mask = 0;
                    self.discard_pending_hold_tx();
                    return Err(ArmDriverError::EstopInhibited);
                }
                if sent_any {
                    if self.flush_cutoff_exceeded() {
                        self.mark_tx_pass_backpressured();
                        self.discard_staged_ctrl_tx();
                        self.discard_pending_hold_tx();
                        self.last_hold_tx_sent_any = false;
                        self.last_hold_tx_due_mask = due_mask;
                        self.last_hold_tx_sent_mask = 0;
                        return Err(TransportError::Timeout.into());
                    }
                    if let Err(e) = self.flush_can_and_commit() {
                        self.mark_tx_pass_backpressured();
                        self.discard_pending_hold_tx();
                        self.last_hold_tx_sent_any = false;
                        if self.pending_stop_check(Some(start_stamp)).is_err() {
                            self.last_hold_tx_due_mask = 0;
                            self.last_hold_tx_sent_mask = 0;
                            return Err(ArmDriverError::EstopInhibited);
                        }
                        self.last_hold_tx_due_mask = due_mask;
                        self.last_hold_tx_sent_mask = 0;
                        return Err(e.into());
                    }
                }
                if self.pending_stop_check(Some(start_stamp)).is_err() {
                    self.last_hold_tx_due_mask = 0;
                    self.last_hold_tx_sent_mask = 0;
                    self.discard_pending_hold_tx();
                    return Err(ArmDriverError::EstopInhibited);
                }
                for &(joint, slot) in &sent_order[..sent_order_len] {
                    self.record_flushed_joint_tx(joint, slot, false);
                }
                if let Some(refused_joint) = first_build_refused_joint {
                    let post_now = self.now();
                    let (late_stale, late_oldest) =
                        self.pending_hold_tx.drop_stale(post_now, self.tx_budget.frame_max_age);
                    if late_stale > 0 {
                        if let Some(age) = late_oldest {
                            oldest_expired = Some(oldest_expired.map_or(age, |o| o.max(age)));
                        }
                        self.last_expired_oldest_age = oldest_expired;
                        crate::diagnostics::tick::record_tx_dropped_stale(late_stale as u32);
                    }
                    let newly_deferred = self.pending_hold_tx.mark_newly_deferred();
                    crate::diagnostics::tick::record_tx_deferred(newly_deferred as u32, self.total_pending_tx_len());
                    self.last_hold_tx_sent_any = sent_any;
                    self.last_hold_tx_due_mask = due_mask;
                    self.last_hold_tx_sent_mask = sent_slot_mask;
                    if self.fresh_measured_position(refused_joint, POS_FORCE_FEEDBACK_MAX_AGE).is_none() {
                        return Err(ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint: refused_joint });
                    }
                    return Err(ArmDriverError::PosForceTargetUnavailable { joint: refused_joint });
                }
                let post_now = self.now();
                let (late_stale, late_oldest) =
                    self.pending_hold_tx.drop_stale(post_now, self.tx_budget.frame_max_age);
                if late_stale > 0 {
                    dropped_stale_in_pass += late_stale;
                    if let Some(age) = late_oldest {
                        oldest_expired = Some(oldest_expired.map_or(age, |o| o.max(age)));
                    }
                    crate::diagnostics::tick::record_tx_dropped_stale(late_stale as u32);
                }

                self.last_hold_tx_sent_any = sent_any;
                self.last_hold_tx_due_mask = due_mask;
                self.last_hold_tx_sent_mask = sent_slot_mask;
                let newly_deferred = self.pending_hold_tx.mark_newly_deferred();
                let pending = self.pending_hold_tx.len();
                crate::diagnostics::tick::record_tx_deferred(newly_deferred as u32, self.total_pending_tx_len());
                let total_dropped_stale = pre_dropped_stale.saturating_add(dropped_stale_in_pass);
                if total_dropped_stale > 0 {
                    self.last_expired_oldest_age = oldest_expired;
                    Ok(TxBatchOutcome::Expired { dropped: total_dropped_stale, pending })
                } else if pending == 0 {
                    self.last_expired_oldest_age = None;
                    Ok(TxBatchOutcome::Complete)
                } else {
                    self.last_expired_oldest_age = None;
                    Ok(TxBatchOutcome::Deferred { pending })
                }
            })()
        };
        if arm_res.is_err() {
            self.discard_staged_ctrl_tx();
        }

        // When attached to `SupervisorInner` (`external_pos_force_emitter == true`), the tick
        // epilogue invokes `emit_pos_force_j7` once per tick after every arm path. Direct driver
        // callers invoke the exact same ONE emitter right here after the arm stopped-hold.
        if self.external_pos_force_emitter || matches!(arm_res, Err(ArmDriverError::EstopInhibited)) {
            return arm_res;
        }
        let stop_intent_stamp = self.current_j7_intent_stamp();
        let hold_bypass = self.j7_hold_continuity_bypass();
        match pos_force_emitter::emit_tick(
            self,
            PosForceIntent::stop(stop_intent_stamp),
            stop_intent_stamp,
            hold_bypass,
        ) {
            Ok(PosForceEmitOutcome::NotApplicable) => arm_res,
            Ok(PosForceEmitOutcome::AwaitingFirstFresh { pf_status, .. }) => {
                self.last_tx_pf_status = pf_status;
                arm_res
            }
            Ok(
                PosForceEmitOutcome::Sent { joint, pf_status, .. }
                | PosForceEmitOutcome::SentHoldInsteadOfTarget { joint, pf_status, .. },
            ) => {
                let pf_bit = joint.saturating_mul(2);
                let pf_mask = if pf_bit < 64 { 1u64 << pf_bit } else { 0 };
                self.last_hold_tx_sent_any = true;
                self.last_hold_tx_due_mask |= pf_mask;
                self.last_hold_tx_sent_mask |= pf_mask;
                self.last_tx_pf_status = pf_status;
                arm_res
            }
            Ok(PosForceEmitOutcome::Deferred { joint, pf_status }) => {
                let pf_bit = joint.saturating_mul(2);
                let pf_mask = if pf_bit < 64 { 1u64 << pf_bit } else { 0 };
                self.last_hold_tx_due_mask |= pf_mask;
                self.last_tx_pf_status = pf_status;
                match arm_res {
                    Ok(TxBatchOutcome::Complete) => Ok(TxBatchOutcome::Deferred { pending: 1 }),
                    Ok(TxBatchOutcome::Deferred { pending }) => Ok(TxBatchOutcome::Deferred { pending: pending + 1 }),
                    Ok(TxBatchOutcome::Expired { dropped, pending }) => {
                        Ok(TxBatchOutcome::Expired { dropped, pending: pending + 1 })
                    }
                    Err(e) => Err(e),
                }
            }
            Err(ArmDriverError::EstopInhibited) => Err(ArmDriverError::EstopInhibited),
            Err(e) => {
                if let Some(pf_j) = self.active_pos_force_joint() {
                    let pf_bit = pf_j.saturating_mul(2);
                    if pf_bit < 64 {
                        self.last_hold_tx_due_mask |= 1u64 << pf_bit;
                    }
                }
                match arm_res {
                    Err(arm_err) => Err(arm_err),
                    Ok(_) => Err(e),
                }
            }
        }
    }

}
