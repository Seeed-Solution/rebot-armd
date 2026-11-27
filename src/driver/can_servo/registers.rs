//! Damiao CAN register read/write, mapping range checks, and `CommissioningExt`.

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

impl<C: CanTransport> DamiaoArmDriver<C> {
}

impl<C: CanTransport> CommissioningExt for DamiaoArmDriver<C> {
    fn read_motor_identity(&mut self, joints: &[usize]) -> Result<Vec<crate::mit::MotorIdentity>, ArmDriverError> {
        let n = self.motor_ids.len();
        if let Some(&j) = joints.iter().find(|&&j| j >= n) { return Err(ArmDriverError::JointCountMismatch { expected: n, actual: j + 1 }); }
        joints.iter().map(|&i| self.identity_of(i)).collect()
    }

    fn commission_motor_timeout(&mut self, joints: &[usize], counts: u32) -> Result<Vec<crate::mit::TimeoutCommissioning>, crate::mit::TimeoutBatchError> {
        let none_attempted = |e: ArmDriverError| crate::mit::TimeoutBatchError { not_attempted: joints.to_vec(), ..e.into() };
        let n = self.motor_ids.len();
        if let Some(&j) = joints.iter().find(|&&j| j >= n) { return Err(none_attempted(ArmDriverError::JointCountMismatch { expected: n, actual: j + 1 })); }
        if counts == 0 || counts > crate::mit::MAX_TIMEOUT_COUNTS {
            return Err(none_attempted(ArmDriverError::Protocol(format!("TIMEOUT counts must be in 1..={}", crate::mit::MAX_TIMEOUT_COUNTS))));
        }
        if self.is_enabled || self.is_enabling {
            return Err(none_attempted(ArmDriverError::Protocol("the TIMEOUT commissioning needs torque off".into())));
        }
        for &i in joints {
            let fresh = !self.awaiting_rx_boundary
                && self.last_feedback_instant[i].is_some_and(|t| self.since(t) <= ZERO_FEEDBACK_MAX_AGE);
            if !fresh || self.motor_status[i] != 0 {
                return Err(none_attempted(ArmDriverError::HardwareFault(format!("joint {} (CAN ID 0x{:X}) is not confirmed disabled (state {})",
                    i + 1, self.motor_ids[i], self.motor_status[i]))));
            }
        }
        let mut out = Vec::with_capacity(joints.len());
        for (k, &i) in joints.iter().enumerate() {
            let mut written = false;
            let res = self.commission_one(i, counts, &mut written);
            match res {
                Ok(c) => out.push(c),
                Err(error) => return Err(crate::mit::TimeoutBatchError { applied: out, uncertain: if written { vec![i] } else { Vec::new() },
                    not_attempted: joints[k + 1..].to_vec(), error }),
            }
        }
        Ok(out)
    }

    fn set_zero_position(&mut self, joint: usize) -> Result<(), ArmDriverError> {
        let start_stamp = self.current_stop_stamp();
        self.strict_stop_check(Some(start_stamp))?;
        let n = self.motor_ids.len();
        if joint >= n { return Err(ArmDriverError::JointCountMismatch { expected: n, actual: joint + 1 }); }
        let id = self.motor_ids[joint];
        if self.is_enabled || self.is_enabling {
            return Err(ArmDriverError::Protocol(format!("joint {} (CAN ID 0x{id:X}): set-zero needs torque off", joint + 1)));
        }
        let fresh = !self.awaiting_rx_boundary
            && self.last_feedback_instant[joint].is_some_and(|t| self.since(t) <= ZERO_FEEDBACK_MAX_AGE);
        if !fresh || self.motor_status[joint] != 0 {
            return Err(ArmDriverError::HardwareFault(format!(
                "joint {} (CAN ID 0x{id:X}) is not confirmed disabled (state {}, feedback age {:?})",
                joint + 1, self.motor_status[joint], self.last_feedback_instant[joint].map(|t| self.since(t)))));
        }
        self.send_with_retry(TxKind::ModeWrite, Some(start_stamp), None, false, |_| {
            Some(damiao::build_set_zero_frame(id))
        })
        .map_err(Self::refusal_to_driver_err)?;
        self.strict_stop_check(Some(start_stamp))?;
        self.flush_can_and_commit()?;
        tracing::warn!("Damiao joint {} (CAN ID 0x{id:X}): set-zero sent at reading {:.4} rad", joint + 1, self.cached_states[joint].position);
        Ok(())
    }

    fn supports_nonblocking_commissioning(&self) -> bool {
        self.can.supports_nonblocking_rt() && self.can.supports_bounded_flush()
    }

    fn supports_nonblocking_motor_zero(&self) -> bool {
        self.can.supports_nonblocking_rt() && self.can.supports_bounded_flush()
    }

    fn commissioning_motor_id(&self, joint: usize) -> Option<u16> {
        self.motor_ids.get(joint).copied()
    }

    fn commissioning_drain_rx_step(
        &mut self,
        max_frames: u16,
        deadline: Instant,
    ) -> Result<(u16, bool), ArmDriverError> {
        if !self.can.supports_nonblocking_rt() {
            return Err(ArmDriverError::Protocol("non-blocking CAN RX unsupported".into()));
        }
        let limit = usize::from(self.clamp_release_rx_limit(max_frames));
        let mut frames = 0u16;
        let mut drained = false;
        self.last_read_drained = false;
        while usize::from(frames) < limit && self.now() < deadline {
            let attempt = self.now();
            match self.receive_with_rx_time_timed() {
                Ok((frame, rx_time)) => {
                    frames = frames.saturating_add(1);
                    self.process_rx_frame(&frame, true, rx_time);
                }
                Err(TransportError::Timeout) | Err(TransportError::WouldBlock) => {
                    drained = true;
                    self.mark_rx_empty(attempt);
                    break;
                }
                Err(e) => return Err(e.into()),
            }
        }
        Ok((frames, drained))
    }

    fn commissioning_clear_register_log(&mut self) {
        self.comm_reg_reply_len = 0;
        self.comm_reg_reply_overflow = false;
    }

    fn commissioning_try_send_register_read(
        &mut self,
        joint: usize,
        rid: u8,
        deadline: Instant,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::mit::CommissioningTxStatus {
        let Some(&id) = self.motor_ids.get(joint) else {
            return crate::mit::CommissioningTxStatus::Refused("joint out of range");
        };
        let start_stamp = self.current_stop_stamp();
        match self.send_with_commissioning_gate(
            TxKind::Refresh,
            Some(start_stamp),
            Some(Deadline { at: deadline }),
            pre_tx_gate,
            |_| Some(damiao::build_read_register_frame(id, rid)),
        ) {
            Ok((_, tx_seq, tx_started, tx_offset_ns)) => {
                if !self.can.flush_is_noop() {
                    if self.flush_cutoff_exceeded() || self.now() >= deadline {
                        return crate::mit::CommissioningTxStatus::SentFlushFailed {
                            tx_seq,
                            tx_started,
                            tx_offset_ns,
                        };
                    }
                    if self.flush_can_and_commit().is_err() {
                        return crate::mit::CommissioningTxStatus::SentFlushFailed {
                            tx_seq,
                            tx_started,
                            tx_offset_ns,
                        };
                    }
                }
                crate::mit::CommissioningTxStatus::Sent {
                    tx_seq,
                    tx_started,
                    tx_offset_ns,
                }
            }
            Err(Ok(code)) => crate::mit::CommissioningTxStatus::Revoked(code),
            Err(Err(TxRefusal::Deferred | TxRefusal::DeadlinePassed)) => {
                crate::mit::CommissioningTxStatus::Backpressure
            }
            Err(Err(TxRefusal::Transport(e))) if e.is_tx_backpressure() => {
                crate::mit::CommissioningTxStatus::Backpressure
            }
            Err(Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged)) => {
                crate::mit::CommissioningTxStatus::Revoked("STOPPED")
            }
            Err(Err(_)) => crate::mit::CommissioningTxStatus::Refused("register read transmit failed"),
        }
    }

    fn commissioning_try_send_rest_probe(
        &mut self,
        joint: usize,
        deadline: Instant,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::mit::CommissioningTxStatus {
        let Some(&id) = self.motor_ids.get(joint) else {
            return crate::mit::CommissioningTxStatus::Refused("joint out of range");
        };
        if self.is_enabled || self.is_enabling {
            return crate::mit::CommissioningTxStatus::Refused(
                "torque must be off for TIMEOUT commissioning rest-pose probe",
            );
        }
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
                        return crate::mit::CommissioningTxStatus::SentFlushFailed {
                            tx_seq,
                            tx_started,
                            tx_offset_ns,
                        };
                    }
                    if self.flush_can_and_commit().is_err() {
                        return crate::mit::CommissioningTxStatus::SentFlushFailed {
                            tx_seq,
                            tx_started,
                            tx_offset_ns,
                        };
                    }
                }
                crate::mit::CommissioningTxStatus::Sent {
                    tx_seq,
                    tx_started,
                    tx_offset_ns,
                }
            }
            Err(Ok(code)) => crate::mit::CommissioningTxStatus::Revoked(code),
            Err(Err(TxRefusal::Deferred | TxRefusal::DeadlinePassed)) => {
                crate::mit::CommissioningTxStatus::Backpressure
            }
            Err(Err(TxRefusal::Transport(e))) if e.is_tx_backpressure() => {
                crate::mit::CommissioningTxStatus::Backpressure
            }
            Err(Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged)) => {
                crate::mit::CommissioningTxStatus::Revoked("STOPPED")
            }
            Err(Err(_)) => {
                crate::mit::CommissioningTxStatus::Refused("rest-pose probe transmit failed")
            }
        }
    }

    fn commissioning_try_send_timeout_write(
        &mut self,
        joint: usize,
        counts: u32,
        deadline: Instant,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::mit::CommissioningTxStatus {
        let Some(&id) = self.motor_ids.get(joint) else {
            return crate::mit::CommissioningTxStatus::Refused("joint out of range");
        };
        if counts == 0 || counts > crate::mit::MAX_TIMEOUT_COUNTS {
            return crate::mit::CommissioningTxStatus::Refused("TIMEOUT counts out of range");
        }
        if self.is_enabled || self.is_enabling {
            return crate::mit::CommissioningTxStatus::Refused("torque must be off for TIMEOUT commissioning");
        }
        let fresh = !self.awaiting_rx_boundary
            && self.last_feedback_instant[joint].is_some_and(|t| self.since(t) <= ZERO_FEEDBACK_MAX_AGE);
        if !fresh || self.motor_status[joint] != 0 {
            return crate::mit::CommissioningTxStatus::Refused("joint is not confirmed disabled");
        }
        let start_stamp = self.current_stop_stamp();
        // RAM write opcode 0x55 only (NEVER flash store 0xAA).
        match self.send_with_commissioning_gate(
            TxKind::ModeWrite,
            Some(start_stamp),
            Some(Deadline { at: deadline }),
            pre_tx_gate,
            |_| Some(damiao::build_write_register_u32_frame(id, damiao::RID_TIMEOUT, counts)),
        ) {
            Ok((_, tx_seq, tx_started, tx_offset_ns)) => {
                if !self.can.flush_is_noop() {
                    if self.flush_cutoff_exceeded() || self.now() >= deadline {
                        return crate::mit::CommissioningTxStatus::SentFlushFailed {
                            tx_seq,
                            tx_started,
                            tx_offset_ns,
                        };
                    }
                    if self.flush_can_and_commit().is_err() {
                        return crate::mit::CommissioningTxStatus::SentFlushFailed {
                            tx_seq,
                            tx_started,
                            tx_offset_ns,
                        };
                    }
                }
                crate::mit::CommissioningTxStatus::Sent {
                    tx_seq,
                    tx_started,
                    tx_offset_ns,
                }
            }
            Err(Ok(code)) => crate::mit::CommissioningTxStatus::Revoked(code),
            Err(Err(TxRefusal::Deferred | TxRefusal::DeadlinePassed)) => {
                crate::mit::CommissioningTxStatus::Backpressure
            }
            Err(Err(TxRefusal::Transport(e))) if e.is_tx_backpressure() => {
                crate::mit::CommissioningTxStatus::Backpressure
            }
            Err(Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged)) => {
                crate::mit::CommissioningTxStatus::Revoked("STOPPED")
            }
            Err(Err(_)) => crate::mit::CommissioningTxStatus::Refused("TIMEOUT write transmit failed"),
        }
    }

    fn commissioning_find_register_reply(
        &self,
        joint: usize,
        write: bool,
        rid: u8,
        after_tx_seq: u64,
        min_rx_at: Instant,
        max_rx_at: Option<Instant>,
        tx_offset_ns: Option<i128>,
    ) -> crate::mit::CommissioningRegMatch {
        let Some(&id) = self.motor_ids.get(joint) else {
            return crate::mit::CommissioningRegMatch::Pending;
        };
        if self.comm_reg_reply_overflow {
            return crate::mit::CommissioningRegMatch::Ambiguous;
        }
        let mut matched: Option<crate::mit::CommissioningRegReply> = None;
        for slot in &self.comm_reg_replies[..self.comm_reg_reply_len] {
            let Some(rep) = *slot else { continue };
            if rep.motor_id != id || rep.write != write || rep.rid != rid {
                continue;
            }
            if rep.rx_seq <= after_tx_seq {
                continue;
            }
            // B1: Any reply for `(motor_id, op, rid)` dequeued AFTER the empty-queue TX boundary
            // whose timestamp precedes `min_rx_at` or whose realtime-to-monotonic clock offset
            // drifted proves delayed/ambiguous in-flight bus traffic -> fail closed!
            if rep.rx_at < min_rx_at {
                return crate::mit::CommissioningRegMatch::Ambiguous;
            }
            let offset_ok = match (tx_offset_ns, rep.decode_offset_ns) {
                (Some(w_off), Some(d_off)) => {
                    (d_off - w_off).abs() <= MAX_REALTIME_MONOTONIC_OFFSET_DRIFT_NS
                }
                (None, None) => true,
                _ => false,
            };
            if !offset_ok {
                return crate::mit::CommissioningRegMatch::Ambiguous;
            }
            // Multiple replies for the same `(motor_id, op, rid)` in one request window ->
            // cannot distinguish delayed stale traffic from the current request -> fail closed!
            if matched.is_some() {
                return crate::mit::CommissioningRegMatch::Ambiguous;
            }
            // Blocker 4: Never accept a late reply whose arrival timestamp reached/passed the
            // request expiry deadline (`max_rx_at`).
            if max_rx_at.is_some_and(|dl| rep.rx_at >= dl) {
                continue;
            }
            matched = Some(rep);
        }
        match matched {
            Some(rep) => crate::mit::CommissioningRegMatch::Matched(rep),
            None => crate::mit::CommissioningRegMatch::Pending,
        }
    }

    fn commissioning_disable_step(
        &mut self,
        started_at: Instant,
        deadline: Instant,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::mit::CommissioningDisableStep {
        let n = self.motor_ids.len().min(64);
        let all_mask = if n >= 64 { u64::MAX } else { (1u64 << n) - 1 };
        if (self.disable_dispatched_mask & all_mask) == all_mask {
            self.disable_dispatched_mask = 0;
        }
        let start_stamp = self.current_stop_stamp();
        let tx_kind = TxKind::DisableRequested { started_at };
        let mut sent_any = false;
        let mut wait_evidence = false;
        let mut last_tx_seq = self.rx_seq;
        let mut last_tx_started = self.now();
        let mut last_tx_offset = None;
        let mut early_ret: Option<crate::mit::CommissioningDisableStep> = None;
        #[cfg(not(target_arch = "wasm32"))]
        let mut deferred_refusal_mask = crate::torque::ChannelMask::EMPTY;
        #[cfg(not(target_arch = "wasm32"))]
        let mut deferred_refusal_outcomes =
            [crate::torque::MotorWriteOutcome::NotTargeted; crate::torque::MAX_CHANNELS];
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.defer_disable_shadow = true;
            self.shadow_disable_pass_sent_mask = self.disable_dispatched_mask;
        }
        for idx in 0..n {
            if (self.disable_dispatched_mask & (1u64 << idx)) != 0 {
                continue;
            }
            if self.now() >= deadline {
                break;
            }
            let id = self.motor_ids[idx];
            match self.send_with_commissioning_gate(
                tx_kind,
                Some(start_stamp),
                Some(Deadline { at: deadline }),
                pre_tx_gate,
                |_| Some(damiao::build_disable_frame(id)),
            ) {
                Ok((_, tx_seq, tx_started, tx_offset)) => {
                    sent_any = true;
                    last_tx_seq = tx_seq;
                    last_tx_started = tx_started;
                    last_tx_offset = tx_offset;
                    self.disable_dispatched_mask |= 1u64 << idx;
                }
                Err(Ok(code)) => {
                    #[cfg(not(target_arch = "wasm32"))]
                    if idx < crate::torque::MAX_CHANNELS {
                        deferred_refusal_mask.insert(idx as u8);
                        deferred_refusal_outcomes[idx] =
                            crate::torque::MotorWriteOutcome::RejectedStale;
                    }
                    if sent_any && !self.can.flush_is_noop() && !self.flush_cutoff_exceeded() {
                        let _ = self.flush_can_and_commit();
                    }
                    early_ret = Some(crate::mit::CommissioningDisableStep::Revoked(code));
                    break;
                }
                Err(Err(TxRefusal::DisableWaitEvidence)) => {
                    wait_evidence = true;
                }
                Err(Err(TxRefusal::TorqueOffWouldDrop)) => {
                    #[cfg(not(target_arch = "wasm32"))]
                    if idx < crate::torque::MAX_CHANNELS {
                        deferred_refusal_mask.insert(idx as u8);
                        deferred_refusal_outcomes[idx] =
                            crate::torque::MotorWriteOutcome::RejectedStale;
                    }
                    early_ret = Some(crate::mit::CommissioningDisableStep::Refused(
                        "ARM_NOT_AT_RESTING_POSE",
                    ));
                    break;
                }
                Err(Err(TxRefusal::DisableRequestExpired)) => {
                    #[cfg(not(target_arch = "wasm32"))]
                    if idx < crate::torque::MAX_CHANNELS {
                        deferred_refusal_mask.insert(idx as u8);
                        deferred_refusal_outcomes[idx] =
                            crate::torque::MotorWriteOutcome::RejectedStale;
                    }
                    early_ret = Some(crate::mit::CommissioningDisableStep::Refused(
                        "disable request expired",
                    ));
                    break;
                }
                Err(Err(TxRefusal::Deferred | TxRefusal::DeadlinePassed)) => {
                    #[cfg(not(target_arch = "wasm32"))]
                    if idx < crate::torque::MAX_CHANNELS {
                        deferred_refusal_mask.insert(idx as u8);
                        deferred_refusal_outcomes[idx] =
                            crate::torque::MotorWriteOutcome::Deferred;
                    }
                    break;
                }
                Err(Err(TxRefusal::Transport(e))) if e.is_tx_backpressure() => break,
                Err(Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged)) => {
                    #[cfg(not(target_arch = "wasm32"))]
                    if idx < crate::torque::MAX_CHANNELS {
                        deferred_refusal_mask.insert(idx as u8);
                        deferred_refusal_outcomes[idx] =
                            crate::torque::MotorWriteOutcome::RejectedStale;
                    }
                    early_ret = Some(crate::mit::CommissioningDisableStep::Revoked("STOPPED"));
                    break;
                }
                Err(Err(r)) => {
                    #[cfg(not(target_arch = "wasm32"))]
                    if !matches!(r, TxRefusal::Transport(_)) && idx < crate::torque::MAX_CHANNELS {
                        deferred_refusal_mask.insert(idx as u8);
                        deferred_refusal_outcomes[idx] =
                            crate::torque::MotorWriteOutcome::RejectedStale;
                    }
                    early_ret = Some(crate::mit::CommissioningDisableStep::Refused(
                        "disable transmit failed",
                    ));
                    break;
                }
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.defer_disable_shadow = false;
            if self.shadow_staging.is_enabled() && !deferred_refusal_mask.is_empty() {
                PendingDisableShadowBatch::merge_batch(
                    &mut self.pending_disable_shadow,
                    PendingDisableShadowBatch {
                        kind: crate::torque::WriteKind::DisableRequested,
                        target_mask: deferred_refusal_mask,
                        outcomes: deferred_refusal_outcomes,
                        ambiguous_gap: false,
                        write_done: None,
                    },
                );
            }
        }
        if let Some(step) = early_ret {
            #[cfg(not(target_arch = "wasm32"))]
            self.flush_pending_disable_shadow_now();
            return step;
        }
        if sent_any && !self.can.flush_is_noop() {
            if self.flush_cutoff_exceeded() || self.now() >= deadline {
                #[cfg(not(target_arch = "wasm32"))]
                self.flush_pending_disable_shadow_now();
                return crate::mit::CommissioningDisableStep::InProgress {
                    dispatched_mask: self.disable_dispatched_mask,
                };
            }
            if self.flush_can_and_commit().is_err() {
                #[cfg(not(target_arch = "wasm32"))]
                self.flush_pending_disable_shadow_now();
                return crate::mit::CommissioningDisableStep::Refused("disable flush failed");
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        self.flush_pending_disable_shadow_now();
        if (self.disable_dispatched_mask & all_mask) == all_mask {
            self.disable_dispatched_mask = 0;
            crate::mit::CommissioningDisableStep::AllDispatched {
                boundary_seq: last_tx_seq,
                tx_started: last_tx_started,
                tx_offset_ns: last_tx_offset,
            }
        } else if wait_evidence && !sent_any {
            crate::mit::CommissioningDisableStep::WaitEvidence
        } else {
            crate::mit::CommissioningDisableStep::InProgress {
                dispatched_mask: self.disable_dispatched_mask,
            }
        }
    }

    fn commissioning_try_send_set_zero(
        &mut self,
        joint: usize,
        deadline: Instant,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::mit::CommissioningTxStatus {
        let Some(&id) = self.motor_ids.get(joint) else {
            return crate::mit::CommissioningTxStatus::Refused("joint out of range");
        };
        if self.is_enabled || self.is_enabling {
            return crate::mit::CommissioningTxStatus::Refused("set-zero needs torque off");
        }
        let fresh = !self.awaiting_rx_boundary
            && self.last_feedback_instant[joint].is_some_and(|t| self.since(t) <= ZERO_FEEDBACK_MAX_AGE);
        if !fresh || self.motor_status[joint] != 0 {
            return crate::mit::CommissioningTxStatus::Refused("joint is not confirmed disabled");
        }
        let start_stamp = self.current_stop_stamp();
        match self.send_with_commissioning_gate(
            TxKind::ModeWrite,
            Some(start_stamp),
            Some(Deadline { at: deadline }),
            pre_tx_gate,
            |_| Some(damiao::build_set_zero_frame(id)),
        ) {
            Ok((_, tx_seq, tx_started, tx_offset_ns)) => {
                if !self.can.flush_is_noop() {
                    if self.flush_cutoff_exceeded() || self.now() >= deadline {
                        return crate::mit::CommissioningTxStatus::SentFlushFailed {
                            tx_seq,
                            tx_started,
                            tx_offset_ns,
                        };
                    }
                    if self.flush_can_and_commit().is_err() {
                        return crate::mit::CommissioningTxStatus::SentFlushFailed {
                            tx_seq,
                            tx_started,
                            tx_offset_ns,
                        };
                    }
                }
                crate::mit::CommissioningTxStatus::Sent {
                    tx_seq,
                    tx_started,
                    tx_offset_ns,
                }
            }
            Err(Ok(code)) => crate::mit::CommissioningTxStatus::Revoked(code),
            Err(Err(TxRefusal::Deferred | TxRefusal::DeadlinePassed)) => {
                crate::mit::CommissioningTxStatus::Backpressure
            }
            Err(Err(TxRefusal::Transport(e))) if e.is_tx_backpressure() => {
                crate::mit::CommissioningTxStatus::Backpressure
            }
            Err(Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged)) => {
                crate::mit::CommissioningTxStatus::Revoked("STOPPED")
            }
            Err(Err(_)) => crate::mit::CommissioningTxStatus::Refused("set-zero transmit failed"),
        }
    }

    fn commissioning_raw_feedback(&self, joint: usize) -> Option<crate::mit::CommissioningFeedback> {
        let s = self.cached_states.get(joint)?;
        let rx_at = self.last_feedback_instant.get(joint).copied().flatten()?;
        let rx_seq = self.last_feedback_seq.get(joint).copied()?;
        let motor_status = self.motor_status.get(joint).copied()?;
        let queue_bound = self.feedback_queue_bound.get(joint).copied().flatten();
        Some(crate::mit::CommissioningFeedback {
            raw_position: s.position,
            joint_position: s.position,
            velocity: s.velocity,
            motor_status,
            rx_seq,
            rx_at,
            queue_bound,
        })
    }
}
