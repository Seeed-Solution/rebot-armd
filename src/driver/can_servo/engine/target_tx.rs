//! Damiao bounded target batch TX and target-frame validation helpers.

#[allow(unused_imports)]
use std::time::{Duration, Instant};

#[allow(unused_imports)]
use crate::driver::*;
#[allow(unused_imports)]
use crate::driver::can_servo::*;
#[allow(unused_imports)]
use crate::protocol::damiao;
#[allow(unused_imports)]
use crate::transport::{CanFrame, CanTransport};

impl<C: CanTransport> DamiaoArmDriver<C> {
    pub(in crate::driver::can_servo) fn write_joint_targets_bounded_impl(&mut self, targets: &[JointTarget]) -> Result<TxBatchOutcome, ArmDriverError> {
        let batch_lease_gen = self.output_lease_gen;
        self.output_lease_gen = 0;
        self.last_target_tx_sent_mask = 0;
        self.last_tx_pf_status = PosForceTxStatus::Unobserved;
        self.last_output_was_hold = false;
        self.discard_pending_hold_tx();
        if targets.len() != self.motor_ids.len() {
            self.reject_target_tx(MitFenceTripReason::PreflightRejected);
            return Err(ArmDriverError::JointCountMismatch {
                expected: self.motor_ids.len(),
                actual: targets.len(),
            });
        }
        if self.torque_hs.is_some() {
            self.reject_target_tx(MitFenceTripReason::PreflightRejected);
            return Err(ArmDriverError::TorqueTransitionPending);
        }
        let fenced = self.mit_fence.is_some() || self.mit_fence_trip.is_some();
        // Mode confirmation check always runs before frame 0. For direct-driver callers
        // (`!external_pos_force_emitter`), static J7 target validity (finite position/velocity,
        // non-zero wire speed) also fails closed before sending arm frames; under `SupervisorInner`,
        // `write_output_slice` validates the J7 target up front and degrades invalid targets to
        // `Hold`/`Stop` for the tick epilogue.
        if let Some(e) = self.target_frames_refused(None, true) {
            self.reject_target_tx(MitFenceTripReason::PreflightRejected);
            return Err(e);
        }
        if !self.external_pos_force_emitter {
            if let Some(pf_j) = self.active_pos_force_joint() {
                if let Err(e) = self.validate_pos_force_target_static(pf_j, &targets[pf_j]) {
                    self.reject_target_tx(MitFenceTripReason::PreflightRejected);
                    return Err(e);
                }
            }
        }

        let _ = self.ensure_tx_pass();

        if fenced {
            // MIT tracking owns arm joints: every arm frame goes through `send(TxKind::Target, .., fenced=true)`.
            // Active PosForce J7 is NEVER written inside this fenced loop; it is emitted strictly
            // AFTER this arm path via `emit_pos_force_j7_impl` (by `SupervisorInner`'s tick epilogue
            // or directly below for standalone driver callers), whether this path succeeds, refuses
            // on stale J7 precheck, or trips the fence before/mid-batch.
            self.discard_pending_tx_stopped();
            let total_joints = targets.len().min(u8::MAX as usize) as u8;
            if let Err(e) = self.strict_stop_check(None) {
                self.trip_fence(MitFenceTripReason::EstopBeforeBatch { total_joints });
                return Err(e);
            }
            if self.is_lease_fenced(batch_lease_gen) {
                return Err(ArmDriverError::LeaseFenced);
            }
            let pre_arm_intent_stamp = self.current_j7_intent_stamp();
            let arm_res: Result<TxBatchOutcome, ArmDriverError> = (|| {
                if let Some(pf_j) = self.active_pos_force_joint() {
                    if !self.pos_force_feedback_valid(pf_j, self.now()) {
                        self.reject_target_tx(MitFenceTripReason::PreflightRejected);
                        return Err(ArmDriverError::PosForceFeedbackStale { joint: pf_j });
                    }
                }
                self.fence_check()?;
                let mut started = [None::<Instant>; 16];
                if targets.len() > started.len() {
                    self.reject_target_tx(MitFenceTripReason::PreflightRejected);
                    return Err(ArmDriverError::JointCountMismatch {
                        expected: started.len(),
                        actual: targets.len(),
                    });
                }
                let start_stamp = self.current_stop_stamp();
                let mut batch_frames = 0usize;
                let mut active_mask: u64 = 0;
                for (i, target) in targets.iter().enumerate() {
                    if self.joint_modes.get(i).copied().flatten() == Some(DamiaoControlMode::PosForce) {
                        continue;
                    }
                    match self.target_frames_checked(i, target) {
                        Ok((frames, _)) => {
                            let cnt = frames.into_iter().flatten().count();
                            batch_frames += cnt;
                            if cnt > 0 && i < 64 {
                                active_mask |= 1u64 << i;
                            }
                        }
                        Err(e) => {
                            self.trip_fence(MitFenceTripReason::TargetUnavailable {
                                joint: (i + 1) as u8,
                                completed_joints: 0,
                                total_joints,
                            });
                            return Err(e);
                        }
                    }
                }
                if !self.external_pos_force_emitter && self.active_pos_force_joint().is_some() {
                    batch_frames = batch_frames.saturating_add(1);
                }
                let pass = self.ensure_tx_pass();
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
                // Watchdog-continuity frames for an armed MIT fence are NEVER deferrable by the
                // per-release ledger: when the ledger tightens `effective_tx_max_frames` or
                // `tx_release_deadline` below the target batch, defer the motion target and emit
                // valid stationary holds for every armed axis in the same release so no axis
                // misses its watchdog/fence window.
                let ledger_defers_targets = self.in_active_control_release()
                    && (pass.attempts.saturating_add(batch_frames) > self.effective_tx_max_frames()
                        || self.tx_release_deadline.is_some_and(|cap| self.now() >= cap));
                if ledger_defers_targets {
                    let mut sent_any = false;
                    let mut sent_mask = 0u64;
                    let mut sent_slot_mask = 0u64;
                    match self.send_mit_continuity_holds_for_uncovered(
                        active_mask,
                        0,
                        None,
                        start_stamp,
                        true,
                        &mut started,
                        &mut sent_any,
                        &mut sent_mask,
                        &mut sent_slot_mask,
                    ) {
                        Ok(()) => {}
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
                    if self.pending_stop_check(Some(start_stamp)).is_err() {
                        self.discard_staged_ctrl_tx();
                        self.trip_fence(MitFenceTripReason::EstopBeforeFlush { total_joints });
                        return Err(ArmDriverError::EstopInhibited);
                    }
                    if sent_any {
                        if let Err(e) = self.flush_can_and_commit() {
                            self.mark_tx_pass_backpressured();
                            if self.pending_stop_check(Some(start_stamp)).is_err() {
                                self.trip_fence(MitFenceTripReason::EstopAfterFlush { total_joints });
                                return Err(ArmDriverError::EstopInhibited);
                            }
                            self.trip_fence(MitFenceTripReason::FlushFailed {
                                kind: TxFailKind::from_transport(&e),
                            });
                            return Err(e.into());
                        }
                    }
                    self.fence_check()?;
                    for (i, t) in started.into_iter().enumerate().take(targets.len()) {
                        if t.is_some() && (sent_mask & (1u64 << i)) != 0 {
                            self.mit_fence_last_cmd[i] = t;
                            self.record_flushed_joint_tx(i, 0, false);
                        }
                    }
                    self.last_target_tx_sent_mask = 0;
                    return Ok(TxBatchOutcome::Deferred { pending: batch_frames });
                }
                if (!self.in_active_control_release()
                    && self.now() >= self.effective_tx_pass_deadline(pass))
                    || self.now() >= pass.deadline
                {
                    let reason = MitFenceTripReason::TxBudgetDeadlineBeforeBatch;
                    self.trip_fence(reason);
                    return Err(ArmDriverError::MitFenceTripped(reason));
                }
                let mut sent_mask: u64 = 0;
                let mut fell_back_partial_uncovered: usize = 0;
                'target_batch: for (i, target) in targets.iter().enumerate() {
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
                        match self.send_with_lease(
                            TxKind::Target,
                            Some(start_stamp),
                            batch_lease_gen,
                            None,
                            true,
                            |d| d.target_frames_checked(i, target).ok()?.0.get(slot).copied().flatten(),
                        ) {
                            Ok(sent_frame) => {
                                let mit_params = self.resolved_mit_params(target);
                                self.record_sent_target(i, sent_frame, Some(mit_params));
                                if i < 64 {
                                    sent_mask |= 1u64 << i;
                                }
                            }
                            Err(TxRefusal::Deferred | TxRefusal::DeadlinePassed)
                                if self.in_active_control_release()
                                    && (sent_mask != 0
                                        || self
                                            .tick_tx
                                            .is_none_or(|p| !p.backpressured && self.now() < p.deadline)) =>
                            {
                                if self.strict_stop_check(Some(start_stamp)).is_err() {
                                    self.trip_fence(MitFenceTripReason::EstopDuringBatch {
                                        completed_joints: i as u8,
                                        total_joints,
                                    });
                                    return Err(ArmDriverError::EstopInhibited);
                                }
                                self.fence_check()?;
                                let covered = sent_mask;
                                let uncovered = (active_mask & !covered).count_ones() as usize;
                                let mut dummy_any = false;
                                let mut dummy_slot = 0u64;
                                match self.send_mit_continuity_holds_for_uncovered(
                                    active_mask,
                                    covered,
                                    None,
                                    start_stamp,
                                    true,
                                    &mut started,
                                    &mut dummy_any,
                                    &mut sent_mask,
                                    &mut dummy_slot,
                                ) {
                                    Ok(()) => {
                                        fell_back_partial_uncovered = uncovered.max(1);
                                        break 'target_batch;
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
                            Err(TxRefusal::LeaseFenced) => {
                                return Err(ArmDriverError::LeaseFenced);
                            }
                            Err(TxRefusal::EnableAuthorityExpired) => {
                                return Err(ArmDriverError::EnableAuthorityExpired);
                            }
                            Err(TxRefusal::PosForceFeedbackExpired(joint)) => {
                                self.trip_fence(MitFenceTripReason::TargetUnavailable {
                                    joint: (joint + 1) as u8,
                                    completed_joints: i as u8,
                                    total_joints,
                                });
                                return Err(ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint });
                            }
                            Err(TxRefusal::BuildRefused) => {
                                self.trip_fence(MitFenceTripReason::TargetUnavailable {
                                    joint: (i + 1) as u8,
                                    completed_joints: i as u8,
                                    total_joints,
                                });
                                return Err(ArmDriverError::PosForceTargetUnavailable { joint: i });
                            }
                            Err(TxRefusal::Transport(e)) => {
                                if self.in_active_control_release() && sent_mask != 0 {
                                    if self.strict_stop_check(Some(start_stamp)).is_err() {
                                        self.trip_fence(MitFenceTripReason::EstopBetweenAttempts {
                                            joint: (i + 1) as u8,
                                            completed_joints: i as u8,
                                            total_joints,
                                        });
                                        return Err(ArmDriverError::EstopInhibited);
                                    }
                                    if self.fence_check().is_ok() {
                                        let covered = sent_mask;
                                        let uncovered = (active_mask & !covered).count_ones() as usize;
                                        let mut dummy_any = false;
                                        let mut dummy_slot = 0u64;
                                        if self
                                            .send_mit_continuity_holds_for_uncovered(
                                                active_mask,
                                                covered,
                                                None,
                                                start_stamp,
                                                true,
                                                &mut started,
                                                &mut dummy_any,
                                                &mut sent_mask,
                                                &mut dummy_slot,
                                            )
                                            .is_ok()
                                        {
                                            fell_back_partial_uncovered = uncovered.max(1);
                                            break 'target_batch;
                                        }
                                    }
                                }
                                self.mark_tx_pass_backpressured();
                                if self.strict_stop_check(Some(start_stamp)).is_err() {
                                    self.trip_fence(MitFenceTripReason::EstopBetweenAttempts {
                                        joint: (i + 1) as u8,
                                        completed_joints: i as u8,
                                        total_joints,
                                    });
                                    return Err(ArmDriverError::EstopInhibited);
                                }
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
                        }
                    }
                }
                if self.strict_stop_check(Some(start_stamp)).is_err() {
                    self.discard_staged_ctrl_tx();
                    self.trip_fence(MitFenceTripReason::EstopBeforeFlush { total_joints });
                    return Err(ArmDriverError::EstopInhibited);
                }
                if self.is_lease_fenced(batch_lease_gen) {
                    self.discard_staged_ctrl_tx();
                    return Err(ArmDriverError::LeaseFenced);
                }
                let flush_blocked = if self.in_active_control_release() {
                    !self.can.flush_is_noop()
                        && self.tick_tx.is_some_and(|p| self.now() >= p.deadline)
                        && fell_back_partial_uncovered == 0
                } else {
                    self.flush_cutoff_exceeded()
                };
                if flush_blocked {
                    self.mark_tx_pass_backpressured();
                    self.discard_staged_ctrl_tx();
                    self.trip_fence(MitFenceTripReason::FlushFailed {
                        kind: TxFailKind::Timeout,
                    });
                    return Err(TransportError::Timeout.into());
                }
                if let Err(e) = self.flush_can_and_commit() {
                    self.mark_tx_pass_backpressured();
                    if self.strict_stop_check(Some(start_stamp)).is_err() {
                        self.trip_fence(MitFenceTripReason::EstopAfterFlush { total_joints });
                        return Err(ArmDriverError::EstopInhibited);
                    }
                    self.trip_fence(MitFenceTripReason::FlushFailed {
                        kind: TxFailKind::from_transport(&e),
                    });
                    return Err(e.into());
                }
                if self.strict_stop_check(Some(start_stamp)).is_err() {
                    self.trip_fence(MitFenceTripReason::EstopAfterFlush { total_joints });
                    return Err(ArmDriverError::EstopInhibited);
                }
                if self.is_lease_fenced(batch_lease_gen) {
                    return Err(ArmDriverError::LeaseFenced);
                }
                self.fence_check()?;
                let mut sent_mask_final: u64 = 0;
                for (i, t) in started.into_iter().enumerate().take(targets.len()) {
                    if t.is_some() {
                        self.mit_fence_last_cmd[i] = t;
                        self.record_flushed_joint_tx(i, 0, fell_back_partial_uncovered == 0);
                        if i < 64 {
                            sent_mask_final |= 1u64 << i;
                        }
                    }
                }
                self.last_target_tx_sent_mask = if fell_back_partial_uncovered == 0 {
                    sent_mask_final
                } else {
                    0
                };
                if fell_back_partial_uncovered > 0 {
                    Ok(TxBatchOutcome::Deferred { pending: fell_back_partial_uncovered })
                } else {
                    Ok(TxBatchOutcome::Complete)
                }
            })();

            if self.external_pos_force_emitter || matches!(arm_res, Err(ArmDriverError::EstopInhibited)) {
                return arm_res;
            }
            let Some(pf_j) = self.active_pos_force_joint() else {
                return arm_res;
            };
            let j7_intent = match &arm_res {
                Ok(TxBatchOutcome::Complete) => PosForceIntent::target(
                    targets[pf_j].position,
                    targets[pf_j].velocity,
                    None,
                    pre_arm_intent_stamp,
                ),
                _ => PosForceIntent::hold(pre_arm_intent_stamp),
            };
            let current_intent_stamp = self.current_j7_intent_stamp();
            let hold_bypass = self.j7_hold_continuity_bypass();
            let j7_res = pos_force_emitter::emit_tick(
                self,
                j7_intent,
                current_intent_stamp,
                hold_bypass,
            );
            return match j7_res {
                Ok(PosForceEmitOutcome::NotApplicable) => arm_res,
                Ok(PosForceEmitOutcome::AwaitingFirstFresh { pf_status, .. }) => {
                    self.last_tx_pf_status = pf_status;
                    if arm_res.is_ok() {
                        self.trip_fence(MitFenceTripReason::TargetUnavailable {
                            joint: (pf_j + 1) as u8,
                            completed_joints: pf_j as u8,
                            total_joints,
                        });
                        return Err(ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint: pf_j });
                    }
                    arm_res
                }
                Ok(PosForceEmitOutcome::Sent { pf_status, .. }) => {
                    self.last_tx_pf_status = pf_status;
                    if arm_res.is_ok() {
                        self.mit_fence_last_cmd[pf_j] = Some(self.now());
                        if let PosForceTxStatus::Stale { .. } = pf_status {
                            self.trip_fence(MitFenceTripReason::TargetUnavailable {
                                joint: (pf_j + 1) as u8,
                                completed_joints: pf_j as u8,
                                total_joints,
                            });
                            return Err(ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint: pf_j });
                        }
                    }
                    arm_res
                }
                Ok(PosForceEmitOutcome::SentHoldInsteadOfTarget { pf_status, .. }) => {
                    self.last_tx_pf_status = pf_status;
                    if arm_res.is_ok() {
                        self.mit_fence_last_cmd[pf_j] = Some(self.now());
                        if let PosForceTxStatus::Stale { .. } = pf_status {
                            self.trip_fence(MitFenceTripReason::TargetUnavailable {
                                joint: (pf_j + 1) as u8,
                                completed_joints: pf_j as u8,
                                total_joints,
                            });
                            return Err(ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint: pf_j });
                        }
                    }
                    match arm_res {
                        Ok(TxBatchOutcome::Complete) => Ok(TxBatchOutcome::Deferred { pending: 1 }),
                        Ok(TxBatchOutcome::Deferred { pending }) => {
                            Ok(TxBatchOutcome::Deferred { pending: pending.saturating_add(1) })
                        }
                        Ok(TxBatchOutcome::Expired { dropped, pending }) => {
                            Ok(TxBatchOutcome::Expired {
                                dropped,
                                pending: pending.saturating_add(1),
                            })
                        }
                        Err(e) => Err(e),
                    }
                }
                Ok(PosForceEmitOutcome::Deferred { pf_status, .. }) => {
                    self.last_tx_pf_status = pf_status;
                    if arm_res.is_ok() {
                        let reason = MitFenceTripReason::TxBudgetExhausted {
                            completed_joints: pf_j as u8,
                            total_joints,
                        };
                        self.trip_fence(reason);
                        return Err(ArmDriverError::MitFenceTripped(reason));
                    }
                    arm_res
                }
                Err(ArmDriverError::EstopInhibited) => Err(ArmDriverError::EstopInhibited),
                Err(e) => match arm_res {
                    Err(arm_err) => Err(arm_err),
                    Ok(_) => Err(e),
                },
            };
        }

        // Unfenced arm-only motion path (POS_VEL, unfenced MIT): stages & drains J1–J6 in
        // `pending_tx`. Active PosForce J7 is NEVER staged in `pending_tx`; it is emitted strictly
        // AFTER via the single J7 emitter (`SupervisorInner`'s tick epilogue, or directly below
        // for standalone driver callers).
        if let Err(e) = self.strict_stop_check(None) {
            self.discard_pending_tx_stopped();
            return Err(e);
        }
        let fenced_carry = {
            let gate = self.estop_gate.as_ref();
            self.pending_tx
                .drop_fenced_lease(|g| gate.is_some_and(|gt| gt.is_lease_fenced(g)))
        };
        if fenced_carry > 0 {
            crate::diagnostics::tick::record_tx_dropped_stopped(fenced_carry as u32);
        }
        if self.is_lease_fenced(batch_lease_gen) {
            return Err(ArmDriverError::LeaseFenced);
        }
        self.output_ran_since_rx = true;
        let batch_stamp = self.current_stop_stamp();
        let stopped_carry = self.pending_tx.drop_stopped(batch_stamp);
        if stopped_carry > 0 {
            crate::diagnostics::tick::record_tx_dropped_stopped(stopped_carry as u32);
        }
        let cleanup_now = self.now();
        self.last_expired_oldest_age = None;
        let (stale, pre_oldest) = self.pending_tx.drop_stale(cleanup_now, self.tx_budget.frame_max_age);
        if stale > 0 {
            crate::diagnostics::tick::record_tx_dropped_stale(stale as u32);
            self.discard_pending_tx_stopped();
            self.last_expired_oldest_age = pre_oldest;
            return Ok(TxBatchOutcome::Expired { dropped: stale, pending: 0 });
        }
        let mut new_keys = 0usize;
        for (i, target) in targets.iter().enumerate() {
            if self.joint_modes.get(i).copied().flatten() == Some(DamiaoControlMode::PosForce) {
                continue;
            }
            let cmds = match self.pending_target_cmds_checked(i, target) {
                Ok(cmds) => cmds,
                Err(e) => {
                    self.reject_target_tx(MitFenceTripReason::PreflightRejected);
                    return Err(e);
                }
            };
            for (frame_id, _) in cmds.into_iter().flatten() {
                if self.pending_tx.find_key(i, frame_id).is_none() {
                    new_keys += 1;
                }
            }
        }
        if self.pending_tx.len() + new_keys > DAMIAO_PENDING_TX_CAPACITY {
            let queued = self.pending_tx.len();
            self.reject_target_tx(MitFenceTripReason::PreflightRejected);
            return Err(ArmDriverError::TxQueueFull {
                queued,
                new: new_keys,
                capacity: DAMIAO_PENDING_TX_CAPACITY,
            });
        }
        let batch_ts = self.now();
        for (i, target) in targets.iter().enumerate() {
            if self.joint_modes.get(i).copied().flatten() == Some(DamiaoControlMode::PosForce) {
                continue;
            }
            let cmds = match self.pending_target_cmds_checked(i, target) {
                Ok(cmds) => cmds,
                Err(e) => {
                    self.reject_target_tx(MitFenceTripReason::PreflightRejected);
                    return Err(e);
                }
            };
            for (frame_id, cmd) in cmds.into_iter().flatten() {
                let ok = self
                    .pending_tx
                    .upsert(i, frame_id, cmd, batch_ts, batch_stamp, batch_lease_gen);
                debug_assert!(ok, "preflight checked pending queue capacity");
            }
        }
        let pre_arm_intent_stamp = self.current_j7_intent_stamp();
        let arm_out = self.drain_pending_pass(batch_stamp, batch_lease_gen, 0, None, true)?;
        if self.external_pos_force_emitter {
            return Ok(arm_out);
        }
        let Some(pf_j) = self.active_pos_force_joint() else {
            return Ok(arm_out);
        };
        let j7_intent = match arm_out {
            TxBatchOutcome::Expired { .. } => PosForceIntent::hold(pre_arm_intent_stamp),
            _ => PosForceIntent::target(
                targets[pf_j].position,
                targets[pf_j].velocity,
                None,
                pre_arm_intent_stamp,
            ),
        };
        let current_intent_stamp = self.current_j7_intent_stamp();
        let hold_bypass = self.j7_hold_continuity_bypass();
        match pos_force_emitter::emit_tick(self, j7_intent, current_intent_stamp, hold_bypass)? {
            PosForceEmitOutcome::NotApplicable => Ok(arm_out),
            PosForceEmitOutcome::AwaitingFirstFresh { pf_status, .. } => {
                self.last_tx_pf_status = pf_status;
                Ok(arm_out)
            }
            PosForceEmitOutcome::Sent { pf_status, .. } => {
                self.last_tx_pf_status = pf_status;
                Ok(arm_out)
            }
            PosForceEmitOutcome::SentHoldInsteadOfTarget { pf_status, .. }
            | PosForceEmitOutcome::Deferred { pf_status, .. } => {
                self.last_tx_pf_status = pf_status;
                Ok(match arm_out {
                    TxBatchOutcome::Complete => TxBatchOutcome::Deferred { pending: 1 },
                    TxBatchOutcome::Deferred { pending } => TxBatchOutcome::Deferred { pending: pending + 1 },
                    TxBatchOutcome::Expired { dropped, pending } => TxBatchOutcome::Expired { dropped, pending },
                })
            }
        }
    }

    pub(in crate::driver::can_servo) fn build_pending_target_frame(&self, entry: &PendingTxFrame) -> Option<CanFrame> {
        if self.is_axis_mode_uncertain(entry.joint) {
            return None;
        }
        let active = self.joint_modes.get(entry.joint).copied().flatten();
        match entry.cmd {
            PendingTargetCmd::PosVel { position } => {
                if matches!(active, Some(DamiaoControlMode::Mit | DamiaoControlMode::PosForce)) {
                    return None;
                }
                Some(self.pos_vel_frame(entry.joint, position))
            }
            PendingTargetCmd::Mit { position, velocity, kp, kd, torque } => {
                if matches!(active, Some(DamiaoControlMode::PosVel | DamiaoControlMode::PosForce)) {
                    return None;
                }
                if !self.is_mit_hold_eligible(entry.joint) {
                    return None;
                }
                let target = JointTarget { position, velocity, kp, kd, torque };
                Some(self.mit_frame(entry.joint, &target))
            }
            PendingTargetCmd::RollbackHold { .. } | PendingTargetCmd::MeasuredHold { .. } => None,
        }
    }

    /// The frames of non-PosForce joint `i`'s target in its active/confirmed mode (one, or both
    /// kinds while its mode is unconfirmed), on the stack: the control tick's write does not
    /// allocate. Active `PosForce` joints are NEVER built here — they are emitted exclusively by
    /// the single J7 emitter ([`Self::emit_pos_force_j7_impl`]).
    pub(in crate::driver::can_servo) fn target_frames_checked(
        &self,
        i: usize,
        target: &JointTarget,
    ) -> Result<([Option<CanFrame>; 2], Option<usize>), ArmDriverError> {
        if self.is_axis_mode_uncertain(i) {
            return Ok(([None, None], None));
        }
        match self.joint_modes[i] {
            Some(DamiaoControlMode::PosVel) => Ok(([Some(self.pos_vel_frame(i, target.position)), None], None)),
            Some(DamiaoControlMode::Mit) => {
                if !self.is_mit_hold_eligible(i) {
                    return Ok(([None, None], None));
                }
                Ok(([Some(self.mit_frame(i, target)), None], None))
            }
            Some(DamiaoControlMode::PosForce) => Ok(([None, None], Some(i))),
            None if self.base_mode(i) == DamiaoControlMode::PosForce => {
                Err(ArmDriverError::PosForceModeUnconfirmed { joint: i })
            }
            None => Ok(([Some(self.pos_vel_frame(i, target.position)), Some(self.mit_frame(i, target))], None)),
        }
    }

    /// Unencoded target commands for non-PosForce arm joint `i` (stack-allocated, zero heap alloc)
    /// staged into [`PendingTxQueue`]. Active `PosForce` joints return `[None, None]` because
    /// they are emitted exclusively by the single J7 emitter ([`Self::emit_pos_force_j7_impl`]).
    pub(in crate::driver::can_servo) fn pending_target_cmds_checked(
        &self,
        i: usize,
        target: &JointTarget,
    ) -> Result<[Option<(u32, PendingTargetCmd)>; 2], ArmDriverError> {
        if self.is_axis_mode_uncertain(i) {
            return Ok([None, None]);
        }
        let id = self.motor_ids[i];
        match self.joint_modes[i] {
            Some(DamiaoControlMode::PosVel) => Ok([
                Some((0x100 + u32::from(id), PendingTargetCmd::PosVel { position: target.position })),
                None,
            ]),
            Some(DamiaoControlMode::Mit) => {
                if !self.is_mit_hold_eligible(i) {
                    return Ok([None, None]);
                }
                Ok([
                    Some((
                        u32::from(id),
                        PendingTargetCmd::Mit {
                            position: target.position,
                            velocity: target.velocity,
                            kp: target.kp,
                            kd: target.kd,
                            torque: target.torque,
                        },
                    )),
                    None,
                ])
            }
            Some(DamiaoControlMode::PosForce) => Ok([None, None]),
            None if self.base_mode(i) == DamiaoControlMode::PosForce => {
                Err(ArmDriverError::PosForceModeUnconfirmed { joint: i })
            }
            None => Ok([
                Some((0x100 + u32::from(id), PendingTargetCmd::PosVel { position: target.position })),
                Some((
                    u32::from(id),
                    PendingTargetCmd::Mit {
                        position: target.position,
                        velocity: target.velocity,
                        kp: target.kp,
                        kd: target.kd,
                        torque: target.torque,
                    },
                )),
            ]),
        }
    }

    /// A batch is refused before its first frame when the position-force gripper's mode is not
    /// confirmed, its target position or velocity is non-finite, or its requested speed encodes
    /// to 0 on the wire. When `allow_stale_pf` is `false` (MIT-fenced paths), stale J7 feedback
    /// also refuses the entire batch before frame 0; unfenced paths (`allow_stale_pf = true`)
    /// route stale J7 through [`Self::check_pos_force_freshness_for_tx`] so J1–J6 continue.
    pub(in crate::driver::can_servo) fn target_frames_refused(&self, targets: Option<&[JointTarget]>, allow_stale_pf: bool) -> Option<ArmDriverError> {
        let pf = self.pos_force.as_ref()?;
        let mode = self.joint_modes.get(pf.joint).copied().flatten();
        let allowed = (mode == Some(DamiaoControlMode::PosForce) && self.pos_force_verified)
            || mode == Some(DamiaoControlMode::Mit);
        if !allowed {
            return Some(ArmDriverError::PosForceModeUnconfirmed { joint: pf.joint });
        }
        if mode == Some(DamiaoControlMode::PosForce) {
            if let Some(target) = targets.and_then(|ts| ts.get(pf.joint)) {
                if let Err(err) = self.validate_pos_force_target_static(pf.joint, target) {
                    return Some(err);
                }
            }
            if !allow_stale_pf && !self.pos_force_feedback_valid(pf.joint, self.now()) {
                return Some(ArmDriverError::PosForceFeedbackStale { joint: pf.joint });
            }
        }
        None
    }

    pub(in crate::driver::can_servo) fn refusal_to_driver_err(r: TxRefusal) -> ArmDriverError {
        match r {
            TxRefusal::DeadlinePassed => ArmDriverError::Transport(TransportError::Timeout),
            TxRefusal::Deferred => ArmDriverError::Transport(TransportError::WouldBlock),
            TxRefusal::StopInhibited | TxRefusal::StopStampChanged => ArmDriverError::EstopInhibited,
            TxRefusal::TorqueOffWouldDrop | TxRefusal::DisableWaitEvidence => {
                ArmDriverError::TorqueOffWouldDrop
            }
            TxRefusal::DisableRequestExpired => ArmDriverError::DisableRequestExpired,
            TxRefusal::LeaseFenced => ArmDriverError::LeaseFenced,
            TxRefusal::EnableAuthorityExpired => ArmDriverError::EnableAuthorityExpired,
            TxRefusal::MitFenceTripped(reason) => ArmDriverError::MitFenceTripped(reason),
            TxRefusal::PosForceFeedbackExpired(joint) => {
                ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint }
            }
            TxRefusal::BuildRefused => ArmDriverError::HardwareFault("frame build refused (stale or invalid state)".into()),
            TxRefusal::Transport(e) => e.into(),
        }
    }

}
