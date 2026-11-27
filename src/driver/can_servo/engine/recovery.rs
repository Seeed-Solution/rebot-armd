//! Damiao CAN RX timing, rest-pose boundaries, refresh scheduling, and `BusRecoveryExt`.

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

impl<C: CanTransport> BusRecoveryExt for DamiaoArmDriver<C> {
    fn set_pending_rearm(&mut self, pending: bool) {
        self.pending_rearm = pending;
    }

    fn drain_rx_only_bounded(&mut self, deadline: Option<Instant>) -> Result<(), ArmDriverError> {
        let blocking = deadline.is_none();
        if !blocking && (!self.can.supports_nonblocking_rt() || !self.can.supports_bounded_flush()) {
            return Err(ArmDriverError::ProtocolStatic(
                "UNSUPPORTED: transport does not support non-blocking RT I/O and bounded flush",
            ));
        }
        let dl = deadline.map(|at| Deadline { at });
        let release_cap_passed = |d: &Self| d.tx_release_deadline.is_some_and(|cap| d.now() >= cap);
        if dl.is_some_and(|d| d.passed(self.now())) || release_cap_passed(self) {
            return Err(ArmDriverError::Transport(TransportError::Timeout));
        }
        let mut drained = self.drain_rx_frames_only(dl, 64);
        if !drained && blocking {
            for _ in 0..7 {
                drained = self.drain_rx_frames_only(dl, 64);
                if drained {
                    break;
                }
            }
        }
        if !drained {
            return Err(ArmDriverError::Transport(TransportError::Timeout));
        }
        Ok(())
    }

    fn clear_carried_over_motor_errors(
        &mut self,
        deadline: Option<Instant>,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> Result<CarryoverMotorClearOutcome, ArmDriverError> {
        let blocking = deadline.is_none();
        if !blocking && (!self.can.supports_nonblocking_rt() || !self.can.supports_bounded_flush()) {
            return Err(ArmDriverError::ProtocolStatic(
                "UNSUPPORTED: transport does not support non-blocking RT I/O and bounded flush",
            ));
        }
        let n = self.motor_ids.len().min(64);
        if n == 0 {
            return Ok(CarryoverMotorClearOutcome::NoErrors);
        }
        // Fail closed unless EVERY motor (including J7) has fresh feedback and none is enabled.
        if self.is_enabled
            || self.is_enabling
            || self.torque_transition_pending()
            || !self.is_feedback_valid()
            || self.has_any_joint_enabled()
            || self.motor_status.iter().take(n).any(|&st| st == 1)
        {
            return Err(ArmDriverError::ProtocolStatic(
                "cannot clear carried-over motor errors while torque is enabled or motor state is stale",
            ));
        }
        if let Some((_, at)) = self.mit_fence_trip {
            if self.since(at) < self.mit_fence_holdoff {
                return Err(ArmDriverError::Transport(TransportError::WouldBlock));
            }
        }
        let dl = deadline.map(|at| Deadline { at });
        let release_cap_passed = |d: &Self| d.tx_release_deadline.is_some_and(|cap| d.now() >= cap);
        if dl.is_some_and(|d| d.passed(self.now())) || release_cap_passed(self) {
            return Err(ArmDriverError::Transport(TransportError::Timeout));
        }
        let start_stamp = self.current_stop_stamp();
        self.pending_stop_check(Some(start_stamp))?;

        let mut affected_mask: u64 = 0;
        for i in 0..n {
            if damiao_fault_name(self.motor_status[i]).is_some() {
                affected_mask |= 1u64 << i;
            }
        }
        if affected_mask == 0 {
            self.pending_stop_check(Some(start_stamp))?;
            if dl.is_some_and(|d| d.passed(self.now())) || release_cap_passed(self) {
                return Err(ArmDriverError::Transport(TransportError::Timeout));
            }
            if let Err(code) = pre_tx_gate() {
                return Ok(CarryoverMotorClearOutcome::Revoked(code));
            }
            return Ok(CarryoverMotorClearOutcome::NoErrors);
        }

        let mut tx_boundary_seq = [0u64; 64];
        let mut sent_mask: u64 = 0;
        self.discard_all_pending_tx();
        self.ensure_release_tx_tick();
        let mut tx_err: Option<ArmDriverError> = None;
        for idx in 0..n {
            if (affected_mask & (1u64 << idx)) == 0 {
                continue;
            }
            // Establish a per-motor pre-write RX watermark immediately before submitting this
            // motor's `CMD_CLEAR_FAULTS`, and re-verify live authority + E-stop inside
            // `send_with_commissioning_gate` right at the `can.transmit` boundary.
            tx_boundary_seq[idx] = self.rx_seq;
            match self.send_with_commissioning_gate(
                TxKind::ClearFault,
                Some(start_stamp),
                dl,
                pre_tx_gate,
                |d| Some(damiao::build_clear_faults_frame(d.motor_ids[idx])),
            ) {
                Ok(_) => sent_mask |= 1u64 << idx,
                Err(Ok(code)) => return Ok(CarryoverMotorClearOutcome::Revoked(code)),
                Err(Err(TxRefusal::DeadlinePassed)) => {
                    return Err(ArmDriverError::Transport(TransportError::Timeout));
                }
                Err(Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged)) => {
                    return Err(ArmDriverError::EstopInhibited);
                }
                Err(Err(TxRefusal::Deferred)) => {
                    let pending = (affected_mask >> idx).count_ones() as usize;
                    tx_err = Some(ArmDriverError::TxDeferred { pending });
                    break;
                }
                Err(Err(TxRefusal::Transport(e))) => {
                    tx_err = Some(ArmDriverError::Transport(e));
                    break;
                }
                Err(Err(r)) => {
                    tx_err = Some(Self::refusal_to_driver_err(r));
                    break;
                }
            }
        }
        self.pending_stop_check(Some(start_stamp))?;
        if sent_mask != 0 {
            if dl.is_some_and(|d| d.passed(self.now())) || self.flush_cutoff_exceeded() {
                return Err(ArmDriverError::Transport(TransportError::Timeout));
            }
            if let Err(e) = self.can.flush() {
                self.mark_tx_pass_backpressured();
                if tx_err.is_none() {
                    tx_err = Some(ArmDriverError::Transport(e));
                }
            }
        }
        if let Some(e) = tx_err {
            return Err(e);
        }

        // Verify on a fresh post-clear status reply (`last_feedback_seq[i] > tx_boundary_seq[i]`)
        // that every affected motor's error bit is gone and every motor stays disabled (`status == 0`).
        // Never sends `CMD_ENABLE`. If a motor did not automatically reply to `CMD_CLEAR_FAULTS`,
        // sends a single read-only `0xCC` status poll to the unanswered affected motor(s).
        let verify_start = self.now();
        let verify_deadline = deadline.unwrap_or_else(|| verify_start + REGISTER_REPLY_TIMEOUT);
        let verify_dl = Some(Deadline { at: verify_deadline });
        let refresh_after = verify_start + Duration::from_millis(5);
        let mut polled_refresh = false;
        loop {
            let drained_empty = self.drain_rx_frames_only(verify_dl, 64);
            self.pending_stop_check(Some(start_stamp))?;
            if self.now() >= verify_deadline || release_cap_passed(self) {
                return Err(ArmDriverError::Transport(TransportError::Timeout));
            }
            let mut first_unconfirmed: Option<usize> = None;
            for i in 0..n {
                if (affected_mask & (1u64 << i)) != 0
                    && self.last_feedback_seq[i] <= tx_boundary_seq[i]
                {
                    first_unconfirmed = Some(i);
                    break;
                }
            }
            if first_unconfirmed.is_none() {
                for i in 0..n {
                    if (affected_mask & (1u64 << i)) == 0 {
                        continue;
                    }
                    let st = self.motor_status[i];
                    if damiao_fault_name(st).is_some() {
                        return Ok(CarryoverMotorClearOutcome::ErrorPersists { joint: i, code: st });
                    }
                    if st == 1 {
                        return Ok(CarryoverMotorClearOutcome::MotorEnabled { joint: i });
                    }
                    if st != 0 {
                        return Ok(CarryoverMotorClearOutcome::ErrorPersists { joint: i, code: st });
                    }
                }
                for i in 0..n {
                    let st = self.motor_status[i];
                    if damiao_fault_name(st).is_some() {
                        return Ok(CarryoverMotorClearOutcome::ErrorPersists { joint: i, code: st });
                    }
                    if st == 1 {
                        return Ok(CarryoverMotorClearOutcome::MotorEnabled { joint: i });
                    }
                }
                self.pending_stop_check(Some(start_stamp))?;
                if self.now() >= verify_deadline || release_cap_passed(self) {
                    return Err(ArmDriverError::Transport(TransportError::Timeout));
                }
                return Ok(CarryoverMotorClearOutcome::Cleared);
            }
            if !polled_refresh && (!blocking || self.now() >= refresh_after) {
                let mut sent_poll = false;
                for idx in 0..n {
                    if (affected_mask & (1u64 << idx)) == 0
                        || self.last_feedback_seq[idx] > tx_boundary_seq[idx]
                    {
                        continue;
                    }
                    tx_boundary_seq[idx] = self.rx_seq;
                    match self.send_with_commissioning_gate(
                        TxKind::Refresh,
                        Some(start_stamp),
                        verify_dl,
                        pre_tx_gate,
                        |d| Some(damiao::build_refresh_frame(d.motor_ids[idx])),
                    ) {
                        Ok(_) => sent_poll = true,
                        Err(Ok(code)) => return Ok(CarryoverMotorClearOutcome::Revoked(code)),
                        Err(Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged)) => {
                            return Err(ArmDriverError::EstopInhibited);
                        }
                        Err(Err(_)) => break,
                    }
                }
                polled_refresh = true;
                if sent_poll {
                    self.pending_stop_check(Some(start_stamp))?;
                    if !self.flush_cutoff_exceeded() && self.now() < verify_deadline {
                        let _ = self.can.flush();
                    }
                    continue;
                }
            }
            if blocking {
                self.pause(Duration::from_micros(500));
            } else if drained_empty && polled_refresh {
                // Under a mock clock on the non-blocking RT path no background hardware thread
                // will inject replies while spinning; exit bounded instead of busy-looping.
                if self.clock.is_some() {
                    return Ok(CarryoverMotorClearOutcome::Unconfirmed {
                        joint: first_unconfirmed.unwrap_or(0),
                    });
                }
                std::thread::yield_now();
            }
        }
    }

    fn notify_bus_loss(&mut self) {
        // Require fresh causal pre-enable CTRL_MODE (and TIMEOUT when enabled) verification across
        // all motors on the next enable without stripping `verified_modes` / `pos_force_verified`
        // from active stopped holds while the BusLoss fault is still holding.
        self.startup_register_check_pending = true;
        if self.timeout_check_enabled {
            self.timeout_verified.fill(false);
        }
    }

    fn panic_disable_step(
        &mut self,
        first: bool,
        deadline: Instant,
    ) -> Option<Result<Vec<usize>, ArmDriverError>> {
        if first {
            self.refresh_capture = None;
            if let Some(hs) = self.torque_hs.take() {
                if !matches!(hs, DamiaoTorque::Disable { .. } | DamiaoTorque::AwaitRestGate { .. }) {
                    self.enable_failed();
                }
            }
            self.enable_stop_stamp = None;
            self.discard_all_pending_tx();
            self.refresh_scheduler.reset_sweep();
            self.pos_force_verified = false;
            self.clear_cached_targets();
            self.last_command_was_disable = true;
            self.is_enabled = false;
            self.pending_rearm = true;
            self.panic_disable_active = true;
            self.panic_disable_tx_boundary.fill(None);
            self.panic_disable_confirmed.fill(false);

            // Fence any undrained pre-disable RX frames BEFORE draining so stale `state == 0`
            // frames still sitting in the queue are discarded rather than updating `motor_status`
            // or `enable_state`, while reaching an empty queue clears `awaiting_rx_boundary`
            // prior to capturing each axis's causal TX boundary.
            self.awaiting_rx_boundary = !self.last_read_drained;
            self.last_poll_rx = None;
            self.rx_deadline = Some(deadline);
            self.rx_frame_limit = Some(64);
            let _ = self.receive_feedback();
            self.rx_deadline = None;
            self.rx_frame_limit = None;
            self.awaiting_rx_boundary = !self.last_read_drained;

            let cmd_start = self.now();
            self.last_enable_command = Some(cmd_start);
            self.last_enable_command_rx_seq = Some(self.rx_seq);
            self.last_query_instant = Some(cmd_start);
        } else {
            // Drain any replies that arrived since the previous step before resending.
            self.last_poll_rx = None;
            self.rx_deadline = Some(deadline);
            self.rx_frame_limit = Some(64);
            let _ = self.receive_feedback();
            self.rx_deadline = None;
            self.rx_frame_limit = None;
        }

        let pending_before = self.unconfirmed_disable_joints();
        if pending_before.is_empty() {
            self.panic_disable_active = false;
            self.disable_dispatched_mask = 0;
            self.mit_fence = None;
            return Some(Ok(Vec::new()));
        }

        let tx_deadline = Deadline { at: deadline };
        let protective = TxKind::DisableProtective {
            reason: DisableProtectiveReason::ProcessExit,
        };
        let mut sent_any = false;
        let mut newly_armed: Vec<usize> = Vec::new();
        let clean_rx_boundary = !self.awaiting_rx_boundary;
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.defer_disable_shadow = true;
            self.shadow_disable_pass_sent_mask = 0;
        }
        for i in pending_before {
            let Some(&id) = self.motor_ids.get(i) else { continue };
            let mut revoked: Option<&'static str> = None;
            match self.send(
                protective,
                None,
                Some(tx_deadline),
                false,
                ContinuityBypass::None,
                None,
                &mut revoked,
                |_| Some(damiao::build_disable_frame(id)),
            ) {
                Ok((_, tx_seq_mark, tx_started_mark, _)) => {
                    sent_any = true;
                    if clean_rx_boundary && self.panic_disable_tx_boundary[i].is_none() {
                        self.panic_disable_tx_boundary[i] = Some((tx_seq_mark, tx_started_mark));
                        newly_armed.push(i);
                    }
                }
                Err(TxRefusal::DeadlinePassed) => {
                    tracing::warn!(
                        "Panic disable step deadline passed before TX for Damiao joint J{} (CAN ID 0x{:X})",
                        i + 1,
                        id
                    );
                    break;
                }
                Err(r) => {
                    tracing::warn!(
                        "Panic disable TX failed for Damiao joint J{} (CAN ID 0x{:X}): {:?}",
                        i + 1,
                        id,
                        r
                    );
                }
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.defer_disable_shadow = false;
        }
        if sent_any {
            if let Err(e) = self.flush_can_and_commit() {
                tracing::warn!("Panic disable CAN flush failed: {e}");
                for i in newly_armed {
                    self.panic_disable_tx_boundary[i] = None;
                }
            }
        }

        self.last_poll_rx = None;
        self.rx_deadline = Some(deadline);
        self.rx_frame_limit = Some(64);
        let _ = self.receive_feedback();
        self.rx_deadline = None;
        self.rx_frame_limit = None;

        // Flush pending disable shadow batch AFTER bounded RX so shadow work never shifts
        // panic-disable RX deadline budgets (Major #1).
        #[cfg(not(target_arch = "wasm32"))]
        self.flush_pending_disable_shadow_now();

        let remaining = self.unconfirmed_disable_joints();
        if remaining.is_empty() {
            self.panic_disable_active = false;
            self.disable_dispatched_mask = 0;
            self.mit_fence = None;
        }
        Some(Ok(remaining))
    }

    fn finish_dispatched_disable(&mut self) -> Result<(), ArmDriverError> {
        let mut first = true;
        while matches!(self.torque_hs, Some(DamiaoTorque::Disable { .. })) {
            if !first {
                self.pause(Duration::from_micros(500));
            }
            first = false;
            let deadline = self.now() + DAMIAO_RX_PASS;
            if let TorqueProgress::Done(_) = self.poll_torque(deadline)? {
                return Ok(());
            }
        }
        Ok(())
    }

    fn service_pending_refresh(&mut self) -> Result<(), ArmDriverError> {
        if self.is_enabling {
            return Ok(());
        }
        let stop_stamp = self.current_stop_stamp();
        match self.service_refresh_scheduler(stop_stamp, None, true, u64::MAX) {
            Ok(_) | Err(ArmDriverError::EstopInhibited) => Ok(()),
            Err(e) => Err(e),
        }
    }
}
