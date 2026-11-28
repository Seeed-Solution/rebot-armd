//! Damiao masked measured-hold TX implementation.

use std::time::Instant;

use crate::driver::*;
use crate::driver::can_servo::*;
use crate::transport::{CanTransport, TransportError};

impl<C: CanTransport> DamiaoArmDriver<C> {
    pub(in crate::driver::can_servo) fn write_measured_hold_masked_impl(
        &mut self,
        hold_mask: u64,
        targets: &[JointTarget],
    ) -> Result<HoldPassReport, ArmDriverError> {
        self.discard_pending_tx_stopped();
        self.last_hold_tx_sent_any = false;
        self.last_hold_tx_due_mask = 0;
        self.last_hold_tx_sent_mask = 0;
        self.last_expired_oldest_age = None;
        self.last_output_was_hold = true;
        let n = self.motor_ids.len();
        if targets.len() != n {
            self.reject_target_tx(MitFenceTripReason::PreflightRejected);
            return Err(ArmDriverError::JointCountMismatch {
                expected: n,
                actual: targets.len(),
            });
        }
        if n > MAX_MODE_JOINTS || n > crate::rt_core::mailbox::MAX_DOF {
            self.discard_pending_hold_tx();
            return Err(ArmDriverError::JointCountMismatch {
                expected: MAX_MODE_JOINTS.min(crate::rt_core::mailbox::MAX_DOF),
                actual: n,
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

        let mut report = HoldPassReport::empty(n);
        let fenced = self.mit_fence.is_some() || self.mit_fence_trip.is_some();
        let total_joints = n.min(u8::MAX as usize) as u8;
        let pf_joint = self.active_pos_force_joint();

        // Classify every joint independently BEFORE any TX (zero cross-joint veto).
        // Any `PosForce` joint is strictly excluded from arm `pending_hold_tx` so `0x307` is
        // emitted exclusively by `pos_force_emitter`.
        let mut active_mask = 0u64;
        let mut stale_mask = 0u64;
        for i in 0..n {
            if i < 64 && (self.disable_dispatched_mask & (1u64 << i)) != 0 {
                let _ = self.pending_hold_tx.drop_joint(i);
                report.joint_status[i] = JointHoldStatus::SkippedOff;
                continue;
            }
            let fresh_ok = self.fresh_measured_position(i, STOPPED_DISABLE_FEEDBACK_MAX_AGE).is_some();
            let mode = self.verified_hold_mode(i);
            let mode_ok = mode.is_some();
            if (hold_mask & (1u64 << i)) == 0 {
                let _ = self.pending_hold_tx.drop_joint(i);
                report.joint_status[i] = if !fresh_ok {
                    JointHoldStatus::UnheldNoFreshFeedback
                } else if !mode_ok {
                    JointHoldStatus::UnheldNoVerifiedMode
                } else {
                    JointHoldStatus::SkippedOff
                };
                continue;
            }
            if mode == Some(DamiaoControlMode::PosForce) || pf_joint == Some(i) {
                let _ = self.pending_hold_tx.drop_joint(i);
                report.joint_status[i] = if !mode_ok {
                    JointHoldStatus::UnheldNoVerifiedMode
                } else if !fresh_ok {
                    JointHoldStatus::HeldStaleTarget
                } else {
                    JointHoldStatus::Queued
                };
                continue;
            }
            if !targets[i].position.is_finite() {
                let _ = self.pending_hold_tx.drop_joint(i);
                report.joint_status[i] = JointHoldStatus::UnheldNoFreshFeedback;
                continue;
            }
            if !mode_ok {
                let _ = self.pending_hold_tx.drop_joint(i);
                report.joint_status[i] = JointHoldStatus::UnheldNoVerifiedMode;
                continue;
            }
            let probe_target = JointTarget {
                position: targets[i].position,
                velocity: Some(0.0),
                kp: targets[i].kp,
                kd: targets[i].kd,
                torque: targets[i].torque,
            };
            if self.build_verified_hold_frame(i, &probe_target).is_none() {
                let _ = self.pending_hold_tx.drop_joint(i);
                report.joint_status[i] = JointHoldStatus::UnheldNoVerifiedMode;
                continue;
            }
            active_mask |= 1u64 << i;
            if !fresh_ok {
                stale_mask |= 1u64 << i;
                report.joint_status[i] = JointHoldStatus::HeldStaleTarget;
            } else {
                report.joint_status[i] = JointHoldStatus::Queued;
            }
        }

        // Only apply the MIT fence when at least one MIT-mode joint is in `active_mask`; when
        // MIT ownership is Lost and MIT joints are masked out, non-MIT position joints must still
        // receive their stationary holds through the unfenced hold path.
        let active_has_mit = (0..n).any(|i| {
            (active_mask & (1u64 << i)) != 0
                && self.verified_hold_mode(i) == Some(DamiaoControlMode::Mit)
        });
        let arm_res: Result<(), ArmDriverError> = if active_mask == 0 {
            let _ = self.pending_hold_tx.drop_outside_mask(0);
            Ok(())
        } else if fenced && active_has_mit {
            (|| -> Result<(), ArmDriverError> {
                self.discard_pending_hold_tx();
                self.fence_check()?;
                if let Err(e) = self.pending_stop_check(None) {
                    self.trip_fence(MitFenceTripReason::EstopBeforeBatch { total_joints });
                    return Err(e);
                }
                let start_stamp = self.current_stop_stamp();
                let pass = self.ensure_tx_pass();
                let mut started = [None::<Instant>; 16];
                let mut batch_frames = 0usize;
                let mut due_mask: u64 = 0;
                for (i, target) in targets.iter().enumerate() {
                    if (active_mask & (1u64 << i)) == 0 {
                        continue;
                    }
                    if self.build_verified_hold_frame(i, target).is_some() {
                        batch_frames += 1;
                        let bit = i.saturating_mul(2);
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
                if self.now() >= pass.deadline {
                    let reason = MitFenceTripReason::TxBudgetDeadlineBeforeBatch;
                    self.trip_fence(reason);
                    return Err(ArmDriverError::MitFenceTripped(reason));
                }
                // Watchdog-continuity measured holds under an armed MIT fence are NEVER deferrable
                // by the per-release ledger (`release_tx_limit`): send valid holds for every armed
                // axis in the same release.
                let mut sent_any = false;
                let mut sent_mask = 0u64;
                let mut sent_slot_mask = 0u64;
                let mut fell_back_partial = false;
                let hold_bypass = if self.in_active_control_release()
                    && self.tx_release_deadline.is_some_and(|cap| self.now() >= cap)
                {
                    ContinuityBypass::LedgerAndReleaseDeadline
                } else {
                    ContinuityBypass::Ledger
                };
                let loop_res: Result<(), ArmDriverError> = (|| {
                    for (i, target) in targets.iter().enumerate() {
                        if (active_mask & (1u64 << i)) == 0 {
                            continue;
                        }
                        if self.build_verified_hold_frame(i, target).is_none() {
                            report.joint_status[i] = JointHoldStatus::UnheldNoVerifiedMode;
                            continue;
                        }
                        let captured_pos = target.position;
                        if started[i].is_none() {
                            started[i] = Some(self.now());
                        }
                        match self.send_plain_with_bypass(
                            TxKind::MeasuredHold(i),
                            Some(start_stamp),
                            None,
                            true,
                            hold_bypass,
                            |d| {
                                if !captured_pos.is_finite() {
                                    return None;
                                }
                                let hold_target = JointTarget {
                                    position: captured_pos,
                                    velocity: Some(0.0),
                                    kp: target.kp,
                                    kd: target.kd,
                                    torque: target.torque,
                                };
                                d.build_verified_hold_frame(i, &hold_target)
                            },
                        ) {
                            Ok(sent_frame) => {
                                let mit_params = self.resolved_mit_params(target);
                                self.record_sent_target(i, sent_frame, Some(mit_params));
                                sent_any = true;
                                sent_mask |= 1u64 << i;
                                let bit = i.saturating_mul(2);
                                if bit < 64 {
                                    sent_slot_mask |= 1u64 << bit;
                                }
                            }
                            Err(TxRefusal::BuildRefused | TxRefusal::PosForceFeedbackExpired(_)) => {
                                let bit = i.saturating_mul(2);
                                if bit < 64 {
                                    due_mask &= !(1u64 << bit);
                                }
                                report.joint_status[i] =
                                    if self.fresh_measured_position(i, STOPPED_DISABLE_FEEDBACK_MAX_AGE).is_none() {
                                        JointHoldStatus::UnheldNoFreshFeedback
                                    } else {
                                        JointHoldStatus::UnheldNoVerifiedMode
                                    };
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
                                unreachable!("TxKind::MeasuredHold is exempt from lease fence");
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
                        self.trip_fence(MitFenceTripReason::FlushFailed { kind: TxFailKind::Timeout });
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
                        self.trip_fence(MitFenceTripReason::FlushFailed { kind: TxFailKind::from_transport(&e) });
                        return Err(e.into());
                    }
                }
                if self.pending_stop_check(Some(start_stamp)).is_err() {
                    self.trip_fence(MitFenceTripReason::EstopAfterFlush { total_joints });
                    return Err(ArmDriverError::EstopInhibited);
                }
                self.fence_check()?;
                for (i, t) in started.into_iter().enumerate().take(n) {
                    if t.is_some() {
                        self.mit_fence_last_cmd[i] = t;
                    }
                    if (sent_mask & (1u64 << i)) != 0 {
                        report.joint_status[i] = if (stale_mask & (1u64 << i)) != 0 {
                            JointHoldStatus::HeldStaleTarget
                        } else {
                            JointHoldStatus::Held
                        };
                        self.record_flushed_joint_tx(i, 0, false);
                    }
                }
                self.last_hold_tx_sent_any = sent_any;
                self.last_hold_tx_due_mask = due_mask;
                self.last_hold_tx_sent_mask = sent_slot_mask;
                report.sent_any = sent_any;
                report.outcome = TxBatchOutcome::Complete;
                Ok(())
            })()
        } else {
            // Unfenced masked bounded hold path with change-class TX scheduling (`peek_best_hold` +
            // J7 priority yield) and per-motor stopped-hold progress masks.
            (|| -> Result<(), ArmDriverError> {
                if let Err(e) = self.pending_stop_check(None) {
                    self.discard_pending_hold_tx();
                    return Err(e);
                }
                let start_stamp = self.current_stop_stamp();
                let stopped_carry = self.pending_hold_tx.drop_stopped(start_stamp);
                if stopped_carry > 0 {
                    crate::diagnostics::tick::record_tx_dropped_stopped(stopped_carry as u32);
                }
                let (pre_dropped_stale, mut oldest_expired) =
                    self.pending_hold_tx.drop_stale(self.now(), self.tx_budget.frame_max_age);
                if pre_dropped_stale > 0 {
                    crate::diagnostics::tick::record_tx_dropped_stale(pre_dropped_stale as u32);
                }

                let mut staged_ids: [Option<u32>; MAX_MODE_JOINTS] = [None; MAX_MODE_JOINTS];
                let mut due_mask: u64 = 0;
                let mut new_keys = 0usize;
                for (i, target) in targets.iter().enumerate() {
                    if (active_mask & (1u64 << i)) == 0 {
                        continue;
                    }
                    let probe_target = JointTarget {
                        position: target.position,
                        velocity: Some(0.0),
                        kp: target.kp,
                        kd: target.kd,
                        torque: target.torque,
                    };
                    let Some(frame) = self.build_verified_hold_frame(i, &probe_target) else {
                        let _ = self.pending_hold_tx.drop_joint(i);
                        report.joint_status[i] = JointHoldStatus::UnheldNoVerifiedMode;
                        continue;
                    };
                    staged_ids[i] = Some(frame.id);
                    let flat_bit = i.saturating_mul(2);
                    if flat_bit < 64 {
                        due_mask |= 1u64 << flat_bit;
                    }
                    if self.pending_hold_tx.find_key(i, frame.id).is_none() {
                        new_keys += 1;
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
                let start_joint = if n > 0 { self.hold_cursor % n } else { 0 };
                for offset in 0..n {
                    let i = (start_joint + offset) % n;
                    if let Some(frame_id) = staged_ids[i] {
                        let target = &targets[i];
                        let cmd = PendingTargetCmd::MeasuredHold {
                            position: target.position,
                            kp: target.kp,
                            kd: target.kd,
                            torque: target.torque,
                        };
                        let ok = self.pending_hold_tx.upsert(i, frame_id, cmd, batch_ts, start_stamp, 0);
                        debug_assert!(ok, "preflight checked pending_hold_tx capacity");
                    }
                }

                let _ = self.ensure_tx_pass();
                self.discard_staged_ctrl_tx();
                let mut sent_any = false;
                let mut sent_mask = 0u64;
                let mut sent_slot_mask = 0u64;
                let mut sent_order = [0usize; MAX_MODE_JOINTS];
                let mut sent_order_len = 0usize;
                let mut expired_mask = 0u64;
                let mut dropped_stale_in_pass = 0usize;
                while let Some((idx, entry, arm_class, arm_seq)) = self
                    .pending_hold_tx
                    .peek_best_hold(&self.slot_in_unchanged_hold, &self.slot_last_tx_seq)
                {
                    let rem = self.remaining_tick_tx_slots();
                    if rem == 0 || !self.can_start_tick_tx() {
                        break;
                    }
                    if rem == 1 {
                        if let Some((j7_class, j7_seq)) = self.j7_tx_priority(true) {
                            if (j7_class, j7_seq) < (arm_class, arm_seq) {
                                break;
                            }
                        }
                    }
                    let entry_deadline = Deadline::after(entry.queued_at, self.tx_budget.frame_max_age);
                    let i = entry.joint;
                    let PendingTargetCmd::MeasuredHold { position, kp, kd, torque } = entry.cmd else {
                        let _ = self.pending_hold_tx.remove_at(idx);
                        continue;
                    };
                    match self.send_plain(
                        TxKind::MeasuredHold(i),
                        Some(entry.stop_stamp),
                        Some(entry_deadline),
                        false,
                        |d| {
                            if !position.is_finite() {
                                return None;
                            }
                            let hold_target = JointTarget {
                                position,
                                velocity: Some(0.0),
                                kp,
                                kd,
                                torque,
                            };
                            d.build_verified_hold_frame(i, &hold_target)
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
                            sent_mask |= 1u64 << i;
                            let flat_bit = i.saturating_mul(2);
                            if flat_bit < 64 {
                                sent_slot_mask |= 1u64 << flat_bit;
                            }
                            if sent_order_len < sent_order.len() {
                                sent_order[sent_order_len] = i;
                                sent_order_len += 1;
                            }
                            if n > 0 {
                                self.hold_cursor = (i + 1) % n;
                            }
                        }
                        Err(TxRefusal::DeadlinePassed) => {
                            let _ = self.pending_hold_tx.remove_at(idx);
                            let age = self.now().saturating_duration_since(entry.queued_at);
                            oldest_expired = Some(oldest_expired.map_or(age, |o| o.max(age)));
                            dropped_stale_in_pass += 1;
                            expired_mask |= 1u64 << i;
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
                        Err(TxRefusal::BuildRefused | TxRefusal::PosForceFeedbackExpired(_)) => {
                            // Per-joint isolation: drop ONLY joint `i`'s queued holds and continue servicing
                            // the remaining joints!
                            let flat_bit = i.saturating_mul(2);
                            if flat_bit < 64 {
                                due_mask &= !(1u64 << flat_bit);
                            }
                            let _ = self.pending_hold_tx.drop_joint(i);
                            report.joint_status[i] =
                                if self.fresh_measured_position(i, STOPPED_DISABLE_FEEDBACK_MAX_AGE).is_none() {
                                    JointHoldStatus::UnheldNoFreshFeedback
                                } else {
                                    JointHoldStatus::UnheldNoVerifiedMode
                                };
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
                        Err(r @ (TxRefusal::TorqueOffWouldDrop | TxRefusal::DisableWaitEvidence | TxRefusal::DisableRequestExpired)) => {
                            self.last_hold_tx_sent_any = sent_any && self.can.flush_is_noop();
                            self.last_hold_tx_due_mask = 0;
                            self.last_hold_tx_sent_mask = 0;
                            self.discard_pending_hold_tx();
                            return Err(Self::refusal_to_driver_err(r));
                        }
                        Err(TxRefusal::LeaseFenced | TxRefusal::EnableAuthorityExpired) => {
                            unreachable!("TxKind::MeasuredHold is exempt from lease fence");
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
                for &joint in &sent_order[..sent_order_len] {
                    self.record_flushed_joint_tx(joint, 0, false);
                }
                let (late_stale, late_oldest) =
                    self.pending_hold_tx.drop_stale(self.now(), self.tx_budget.frame_max_age);
                if late_stale > 0 {
                    dropped_stale_in_pass += late_stale;
                    if let Some(age) = late_oldest {
                        oldest_expired = Some(oldest_expired.map_or(age, |o| o.max(age)));
                    }
                    crate::diagnostics::tick::record_tx_dropped_stale(late_stale as u32);
                }
                // Per-joint post-pass status reconciliation:
                // - Keep holding joints whose feedback went stale at their last captured hold
                //   target, reporting `HeldStaleTarget` ("hold target stale").
                // - Mark `Held` ONLY for fresh joints that actually transmitted & flushed in this
                //   pass with no remaining queued frame; fresh joints still in `pending_hold_tx`
                //   stay `Queued`; joints whose frame expired without transmitting become
                //   `UnheldTxFailed`.
                for i in 0..n {
                    let is_stale = (stale_mask & (1u64 << i)) != 0;
                    if self.pending_hold_tx.has_joint(i) {
                        report.joint_status[i] = if is_stale {
                            JointHoldStatus::HeldStaleTarget
                        } else {
                            JointHoldStatus::Queued
                        };
                    } else if (active_mask & (1u64 << i)) != 0 {
                        if (sent_mask & (1u64 << i)) != 0 {
                            report.joint_status[i] = if is_stale {
                                JointHoldStatus::HeldStaleTarget
                            } else {
                                JointHoldStatus::Held
                            };
                        } else if (expired_mask & (1u64 << i)) != 0
                            || matches!(
                                report.joint_status[i],
                                JointHoldStatus::Queued | JointHoldStatus::HeldStaleTarget
                            )
                        {
                            report.joint_status[i] = JointHoldStatus::UnheldTxFailed;
                        }
                    }
                }

                self.last_hold_tx_sent_any = sent_any;
                self.last_hold_tx_due_mask = due_mask;
                self.last_hold_tx_sent_mask = sent_slot_mask;
                report.sent_any = sent_any;
                let newly_deferred = self.pending_hold_tx.mark_newly_deferred();
                let pending = self.pending_hold_tx.len();
                crate::diagnostics::tick::record_tx_deferred(newly_deferred as u32, self.total_pending_tx_len());
                let total_dropped_stale = pre_dropped_stale.saturating_add(dropped_stale_in_pass);
                report.outcome = if total_dropped_stale > 0 {
                    self.last_expired_oldest_age = oldest_expired;
                    TxBatchOutcome::Expired { dropped: total_dropped_stale, pending }
                } else if pending == 0 {
                    self.last_expired_oldest_age = None;
                    TxBatchOutcome::Complete
                } else {
                    self.last_expired_oldest_age = None;
                    TxBatchOutcome::Deferred { pending }
                };
                Ok(())
            })()
        };
        if arm_res.is_err() {
            self.discard_staged_ctrl_tx();
        }

        // When attached to `SupervisorInner` (`external_pos_force_emitter == true`), the tick
        // epilogue invokes `emit_pos_force_j7` once per tick after every arm path. Standalone
        // direct-driver callers express J7 hold through the exact same single emitter right here.
        if self.external_pos_force_emitter
            || matches!(arm_res, Err(ArmDriverError::EstopInhibited))
            || pf_joint.is_none_or(|j| (hold_mask & (1u64 << j)) == 0)
        {
            return arm_res.map(|()| report);
        }
        let stop_stamp = self.current_j7_intent_stamp();
        match self.emit_pos_force_j7(PosForceIntent::stop(stop_stamp), stop_stamp) {
            Ok(PosForceEmitOutcome::NotApplicable) => arm_res.map(|()| report),
            Ok(PosForceEmitOutcome::AwaitingFirstFresh { joint, pf_status }) => {
                if joint < report.joint_status.len() {
                    report.joint_status[joint] = JointHoldStatus::UnheldNoFreshFeedback;
                }
                self.last_tx_pf_status = pf_status;
                arm_res.map(|()| report)
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
                report.sent_any = true;
                if joint < report.joint_status.len() {
                    report.joint_status[joint] = if matches!(pf_status, PosForceTxStatus::Stale { .. }) {
                        JointHoldStatus::HeldStaleTarget
                    } else {
                        JointHoldStatus::Held
                    };
                }
                arm_res.map(|()| report)
            }
            Ok(PosForceEmitOutcome::Deferred { joint, pf_status }) => {
                let pf_bit = joint.saturating_mul(2);
                let pf_mask = if pf_bit < 64 { 1u64 << pf_bit } else { 0 };
                self.last_hold_tx_due_mask |= pf_mask;
                self.last_tx_pf_status = pf_status;
                if joint < report.joint_status.len() {
                    report.joint_status[joint] = JointHoldStatus::Queued;
                }
                report.outcome = match report.outcome {
                    TxBatchOutcome::Complete => TxBatchOutcome::Deferred { pending: 1 },
                    TxBatchOutcome::Deferred { pending } => TxBatchOutcome::Deferred { pending: pending + 1 },
                    TxBatchOutcome::Expired { dropped, pending } => {
                        TxBatchOutcome::Expired { dropped, pending: pending + 1 }
                    }
                };
                arm_res.map(|()| report)
            }
            Err(ArmDriverError::EstopInhibited) => Err(ArmDriverError::EstopInhibited),
            Err(e) => {
                if let Some(pf_j) = pf_joint {
                    let pf_bit = pf_j.saturating_mul(2);
                    if pf_bit < 64 {
                        self.last_hold_tx_due_mask |= 1u64 << pf_bit;
                    }
                }
                match arm_res {
                    Err(arm_err) => Err(arm_err),
                    Ok(()) => Err(e),
                }
            }
        }
    }

}
