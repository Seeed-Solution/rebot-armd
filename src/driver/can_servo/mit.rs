//! Damiao `MitExt` implementation.

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

pub(super) mod recovery;

impl<C: CanTransport> MitExt for DamiaoArmDriver<C> {
    fn set_mit_deadline_fence(&mut self, deadline: Option<Duration>, holdoff: Duration) {
        if deadline.is_some() {
            self.mit_tracking_used = true;
            self.discard_all_pending_tx();
        }
        self.mit_fence = deadline;
        self.mit_fence_holdoff = holdoff;
        let now = self.now();
        self.mit_fence_last_cmd.fill(deadline.map(|_| now));
    }

    fn set_mit_ownership_lost(&mut self, lost: bool) {
        self.mit_ownership_lost = lost;
    }

    fn mit_fence_trip_bounded(
        &self,
    ) -> Option<crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>> {
        self.mit_fence_trip.as_ref().map(|(r, _)| {
            let mut buf = crate::safety::BoundedText::empty();
            let _ = core::fmt::write(&mut buf, format_args!("{r}"));
            buf
        })
    }

    fn mit_fresh_status(&mut self) -> Result<Vec<u8>, String> {
        // Enforce the post-trip quiet period directly without requiring or mutating
        // `mit_fence_acked`: autonomous background CAN refreshes stay inhibited until
        // `acknowledge_mit_fence_trip()` runs AFTER `clear_check` passes.
        if let Some((_, at)) = &self.mit_fence_trip {
            if self.since(*at) < self.mit_fence_holdoff {
                let remaining = self.mit_fence_holdoff.saturating_sub(self.since(*at));
                return Err(format!("{}: the motors' command timeout may still be running; no frame is sent for {} ms more after the loss",
                    crate::mit::QUIET_PERIOD_CODE, remaining.as_millis().max(1)));
            }
        }
        let start_stamp = self.current_stop_stamp();
        self.pending_stop_check(Some(start_stamp)).map_err(|e| format!("status poll not sent: {e}"))?;
        let pre = self.rx_boundary().map_err(|e| e.to_string())?;
        self.pending_stop_check(Some(start_stamp)).map_err(|e| format!("status poll not sent: {e}"))?;
        for idx in 0..self.motor_ids.len() {
            match self.send_plain(TxKind::Refresh, Some(start_stamp), None, false, |d| {
                Some(damiao::build_refresh_frame(d.motor_ids[idx]))
            }) {
                Ok(_) => {}
                Err(TxRefusal::Deferred) => {
                    return Err("status poll not sent: TX budget exhausted".into());
                }
                Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                    return Err(format!("status poll not sent: {}", ArmDriverError::EstopInhibited));
                }
                Err(r) => {
                    return Err(format!("status poll not sent: {}", Self::refusal_to_driver_err(r)));
                }
            }
        }
        self.pending_stop_check(Some(start_stamp)).map_err(|e| format!("status poll not sent: {e}"))?;
        self.flush_can_and_commit().map_err(|e| format!("flush: {e}"))?;
        let start = self.now();
        loop {
            let _ = self.read_joint_states();
            let stale: Vec<String> = (0..self.motor_ids.len()).filter(|&i| self.last_feedback_seq[i] <= pre).map(|i| format!("J{}", i + 1)).collect();
            if stale.is_empty() {
                return Ok(self.motor_status.clone());
            }
            if self.since(start) >= REGISTER_REPLY_TIMEOUT {
                return Err(format!("no fresh feedback from {}", stale.join(", ")));
            }
            self.pause(Duration::from_micros(500));
        }
    }

    fn mit_expiry_recover(&mut self) -> Result<(), String> {
        self.last_mit_recovery_rx_seq = None;
        self.discard_all_pending_tx();
        self.begin_tx_tick();
        let start_stamp = self.current_stop_stamp();
        let req_boundary = self
            .rx_boundary()
            .map_err(|e| format!("receive boundary before MIT recovery: {e}"))?;
        // 1. clear-error (FF..FB on the motor's ID) to every motor reporting an error state
        let faulted: Vec<usize> = (0..self.motor_ids.len()).filter(|&i| self.motor_status[i] >= 8).collect();
        for &i in &faulted {
            match self.send_plain(TxKind::ClearFault, Some(start_stamp), None, false, |d| {
                Some(damiao::build_clear_faults_frame(d.motor_ids[i]))
            }) {
                Ok(_) => {}
                Err(TxRefusal::Deferred) => {
                    return Err(format!("clear-error to J{} not sent: TX budget exhausted", i + 1));
                }
                Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                    return Err(format!("clear-error to J{} inhibited: {}", i + 1, ArmDriverError::EstopInhibited));
                }
                Err(r) => {
                    return Err(format!("clear-error to J{} not sent: {}", i + 1, Self::refusal_to_driver_err(r)));
                }
            }
        }
        if !faulted.is_empty() {
            self.pending_stop_check(Some(start_stamp)).map_err(|e| format!("clear-error flush inhibited: {e}"))?;
            self.flush_can_and_commit().map_err(|e| format!("flush: {e}"))?;
        }
        // 2. no cached frame may ever be replayed; then the disable of every motor.
        // Callers (`SupervisorInner::clear_fault_for_epoch` / `mit_expiry_recovery_for_epoch`)
        // unconditionally gate this through `may_disable_torque_while_stopped` (rest pose required),
        // and `disable_torque(DisableRequested)` re-enforces Rule A′ at transmit time.
        self.clear_cached_targets();
        let started_at = self.now();
        self.disable_torque(DisableKind::DisableRequested { started_at })
            .map_err(|e| e.to_string())?;
        let boundary = self
            .last_enable_command_rx_seq
            .map_or(req_boundary, |s| s.max(req_boundary));
        let missing: Vec<String> = (0..self.motor_ids.len())
            .filter(|&i| self.last_feedback_seq[i] <= boundary)
            .map(|i| format!("J{}", i + 1))
            .collect();
        if !missing.is_empty() {
            return Err(format!(
                "{} did not report post-recovery feedback after disable",
                missing.join(", ")
            ));
        }
        self.last_mit_recovery_rx_seq = Some(boundary);
        self.mit_fence = None;
        Ok(())
    }

    fn mit_expiry_recover_with_gate(
        &mut self,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> Result<(), String> {
        if let Err((max_frames, required)) = self.check_watchdog_tx_budget_for_energized_inner() {
            return Err(format!(
                "WATCHDOG_BUDGET_INSUFFICIENT: tx_budget.max_frames ({max_frames}) < required ({required}) for energized MIT recovery continuity"
            ));
        }
        self.mit_expiry_recovery_active = true;
        self.mit_expiry_axes = [MitExpiryAxisState::default(); MAX_MODE_JOINTS];
        let res = (|| -> Result<(), String> {
        self.last_mit_recovery_rx_seq = None;
        let n = self.motor_ids.len().min(MAX_MODE_JOINTS);
        if n == 0 {
            self.mit_fence = None;
            return Ok(());
        }
        self.discard_all_pending_tx();
        self.begin_tx_tick();
        let start_stamp = self.current_stop_stamp();
        // Service continuity holds before and between each bounded initial boundary drain pass
        // so even saturated RX queues cannot overrun the <= 20 ms hold cadence.
        let _ = self.service_mit_recovery_continuity_holds(n, start_stamp, 0, pre_tx_gate)?;
        let req_boundary = {
            let mut boundary = None;
            for _ in 0..8 {
                let _ = self.drain_rx_only_record_registers();
                if self.last_read_drained {
                    boundary = Some(self.rx_empty_seq);
                    break;
                }
                let _ = self.service_mit_recovery_continuity_holds(n, start_stamp, 0, pre_tx_gate)?;
            }
            boundary.ok_or_else(|| {
                format!(
                    "receive boundary before MIT recovery: {}",
                    ArmDriverError::HardwareFault(
                        "receive queue not drained: no boundary for the register transaction".into()
                    )
                )
            })?
        };
        self.comm_reg_reply_len = 0;
        self.comm_reg_reply_overflow = false;
        let _ = self.service_mit_recovery_continuity_holds(n, start_stamp, 0, pre_tx_gate)?;

        // Phase 1: ClearFault to every motor reporting a fault state (>= 8) via pre_tx_gate.
        let mut clear_sent = false;
        for i in 0..n {
            if self.motor_status[i] >= 8 {
                match self.send_with_commissioning_gate(
                    TxKind::ClearFault,
                    Some(start_stamp),
                    None,
                    pre_tx_gate,
                    |d| Some(damiao::build_clear_faults_frame(d.motor_ids[i])),
                ) {
                    Ok(_) => clear_sent = true,
                    Err(Ok(code)) => return Err(format!("PRE_TX_GATE:{code}")),
                    Err(Err(TxRefusal::Deferred)) => {
                        return Err(format!("clear-error to J{} not sent: TX budget exhausted", i + 1));
                    }
                    Err(Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged)) => {
                        return Err(format!(
                            "clear-error to J{} inhibited: {}",
                            i + 1,
                            ArmDriverError::EstopInhibited
                        ));
                    }
                    Err(Err(r)) => {
                        return Err(format!(
                            "clear-error to J{} not sent: {}",
                            i + 1,
                            Self::refusal_to_driver_err(r)
                        ));
                    }
                }
            }
        }
        if clear_sent {
            self.pending_stop_check(Some(start_stamp))
                .map_err(|e| format!("clear-error flush inhibited: {e}"))?;
            self.flush_can_and_commit()
                .map_err(|e| format!("flush: {e}"))?;
        }
        let _ = self.service_mit_recovery_continuity_holds(n, start_stamp, 0, pre_tx_gate)?;

        // Ensure Rule A′ rest-pose evidence is confirmed before transmitting CMD_DISABLE.
        self.disable_rest_established = false;
        self.disable_dispatched_mask = 0;
        let started_at = self.now();
        loop {
            if let Err(code) = pre_tx_gate() {
                return Err(format!("PRE_TX_GATE:{code}"));
            }
            let _ = self.service_mit_recovery_continuity_holds(n, start_stamp, 0, pre_tx_gate)?;
            match self.check_stopped_disable_requested(started_at) {
                Ok(()) => break,
                Err(TxRefusal::DisableWaitEvidence) => {
                    if self.since(started_at) >= STOPPED_DISABLE_REQUEST_LIFETIME {
                        return Err(ArmDriverError::DisableRequestExpired.to_string());
                    }
                    if !self.in_active_control_release() {
                        self.begin_tx_tick();
                    }
                    for i in 0..n {
                        match self.send_with_commissioning_gate(
                            TxKind::DisableProbe,
                            Some(start_stamp),
                            None,
                            pre_tx_gate,
                            |d| Some(damiao::build_refresh_frame(d.motor_ids[i])),
                        ) {
                            Ok(_) => {}
                            Err(Ok(code)) => return Err(format!("PRE_TX_GATE:{code}")),
                            Err(Err(_)) => break,
                        }
                    }
                    let _ = self.flush_can_and_commit();
                    let _ = self.service_mit_recovery_continuity_holds(
                        n,
                        start_stamp,
                        0,
                        pre_tx_gate,
                    )?;
                    let wait = self.mit_recovery_capped_wait(n, 0, EVIDENCE_PROBE_INTERVAL);
                    if !wait.is_zero() {
                        self.pause(wait);
                    }
                    let _ = self.drain_rx_only_record_registers();
                }
                Err(r) => return Err(Self::refusal_to_driver_err(r).to_string()),
            }
        }
        self.disable_rest_established = true;

        // Phase 2 (2a CMD_DISABLE -> 2b per-axis Drain+REG_READ barrier -> 2c-1 per-axis
        // Drain+0xCC round 1 -> 2c-2 per-axis Drain+0xCC round 2): sequential causal confirmation
        // (zero alloc) using shared `self.mit_expiry_axes`, RX-only drains immediately before
        // each axis's probe submission, bounded continuity holds before/between work slices,
        // strict `p2_tx_at > p1_ack_at`, and strict phase-deadline enforcement.
        self.clear_cached_targets();
        self.mit_expiry_axes = [MitExpiryAxisState::default(); MAX_MODE_JOINTS];
        let phase2_start = self.now();
        let phase2_timeout = REGISTER_REPLY_TIMEOUT.saturating_mul(4);
        let phase_deadline = phase2_start
            .checked_add(phase2_timeout)
            .zip(started_at.checked_add(STOPPED_DISABLE_REQUEST_LIFETIME))
            .map_or(phase2_start, |(a, b)| a.min(b));

        let clear_unconfirmed_dispatched = |d: &mut Self| {
            let now = d.now();
            let axes = d.mit_expiry_axes;
            let confirmed = d.mit_expiry_confirmed_mask(now, phase_deadline, n, req_boundary, &axes);
            for (i, axis) in axes.iter().enumerate().take(n) {
                if i < 64 && axis.dis_tx.is_some() && (confirmed & (1u64 << i)) == 0 {
                    d.disable_dispatched_mask &= !(1u64 << i);
                }
            }
        };

        let harvest_sync = |d: &mut Self| {
            if !d.last_read_drained {
                return;
            }
            let now = d.now();
            let mut axes = d.mit_expiry_axes;
            let (reset_mask, _reclear_mask) = d.harvest_mit_expiry_confirmations_into(
                now,
                phase_deadline,
                n,
                req_boundary,
                REGISTER_REPLY_TIMEOUT,
                &mut axes,
            );
            d.mit_expiry_axes = axes;
            d.disable_dispatched_mask &= !reset_mask;
            if d.comm_reg_reply_overflow {
                d.comm_reg_reply_len = 0;
                d.comm_reg_reply_overflow = false;
            } else if reset_mask != 0 {
                let seq = d.rx_seq;
                for i in 0..n {
                    if i < 64 && (reset_mask & (1u64 << i)) != 0 {
                        let mid = d.motor_ids[i];
                        d.prune_comm_reg_replies_for_motor(mid, seq);
                    }
                }
            }
        };

        let service_holds_if_due = |d: &mut Self,
                                    gate: &mut dyn FnMut() -> Result<(), &'static str>|
         -> Result<u64, String> {
            let now = d.now();
            let confirmed =
                d.mit_expiry_confirmed_mask(now, phase_deadline, n, req_boundary, &d.mit_expiry_axes);
            if let Err(e) = d.service_mit_recovery_continuity_holds(
                n,
                start_stamp,
                confirmed,
                gate,
            ) {
                clear_unconfirmed_dispatched(d);
                return Err(e);
            }
            Ok(confirmed)
        };

        loop {
            if self.mit_expiry_axes[..n].iter().any(|a| a.dis_tx.is_some()) {
                let _ = self.drain_rx_only_record_registers();
                harvest_sync(self);
                let now = self.now();
                // Enforce phase deadline strictly BEFORE checking confirmation success.
                if now >= phase_deadline {
                    break;
                }
                if self.last_read_drained
                    && (0..n).all(|i| {
                        self.is_mit_expiry_axis_causally_disabled(
                            now,
                            phase_deadline,
                            i,
                            req_boundary,
                            &self.mit_expiry_axes[i],
                        )
                    })
                {
                    break;
                }
                let mut axes = self.mit_expiry_axes;
                let retried_barriers = self.expire_timed_out_mit_probes(
                    now,
                    n,
                    REGISTER_REPLY_TIMEOUT,
                    REGISTER_REPLY_TIMEOUT,
                    &mut axes,
                );
                self.mit_expiry_axes = axes;
                if retried_barriers != 0 {
                    let seq = self.rx_seq;
                    for i in 0..n {
                        if i < 64 && (retried_barriers & (1u64 << i)) != 0 {
                            let mid = self.motor_ids[i];
                            self.prune_comm_reg_replies_for_motor(mid, seq);
                        }
                    }
                }
            }
            if self.now() >= phase_deadline {
                break;
            }
            let _ = service_holds_if_due(self, pre_tx_gate)?;

            // Re-clear any axis that reset due to an adverse post-barrier fault state.
            if (0..n).any(|i| self.mit_expiry_axes[i].dis_tx.is_none() && self.motor_status[i] >= 8) {
                if !self.in_active_control_release() {
                    self.begin_tx_tick();
                }
                let mut re_clear_sent = false;
                for i in 0..n {
                    if self.mit_expiry_axes[i].dis_tx.is_none() && self.motor_status[i] >= 8 {
                        match self.send_with_commissioning_gate(
                            TxKind::ClearFault,
                            Some(start_stamp),
                            Some(Deadline { at: phase_deadline }),
                            pre_tx_gate,
                            |d| Some(damiao::build_clear_faults_frame(d.motor_ids[i])),
                        ) {
                            Ok(_) => re_clear_sent = true,
                            Err(Ok(code)) => {
                                clear_unconfirmed_dispatched(self);
                                return Err(format!("PRE_TX_GATE:{code}"));
                            }
                            Err(Err(TxRefusal::DeadlinePassed)) => break,
                            Err(Err(TxRefusal::Deferred)) => {
                                clear_unconfirmed_dispatched(self);
                                return Err(format!(
                                    "clear-error to J{} not sent: TX budget exhausted",
                                    i + 1
                                ));
                            }
                            Err(Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged)) => {
                                clear_unconfirmed_dispatched(self);
                                return Err(format!(
                                    "clear-error to J{} inhibited: {}",
                                    i + 1,
                                    ArmDriverError::EstopInhibited
                                ));
                            }
                            Err(Err(r)) => {
                                clear_unconfirmed_dispatched(self);
                                return Err(format!(
                                    "clear-error to J{} not sent: {}",
                                    i + 1,
                                    Self::refusal_to_driver_err(r)
                                ));
                            }
                        }
                    }
                }
                if re_clear_sent {
                    if let Err(e) = self.pending_stop_check(Some(start_stamp)) {
                        clear_unconfirmed_dispatched(self);
                        return Err(format!("clear-error flush inhibited: {e}"));
                    }
                    if let Err(code) = pre_tx_gate() {
                        clear_unconfirmed_dispatched(self);
                        return Err(format!("PRE_TX_GATE:{code}"));
                    }
                    if let Err(e) = self.flush_can_and_commit() {
                        clear_unconfirmed_dispatched(self);
                        return Err(format!("flush: {e}"));
                    }
                }
                let _ = service_holds_if_due(self, pre_tx_gate)?;
            }

            // Phase 2a: Send CMD_DISABLE to every motor without an active disable TX.
            if (0..n).any(|i| self.mit_expiry_axes[i].dis_tx.is_none()) {
                if !self.in_active_control_release() {
                    self.begin_tx_tick();
                }
                let mut dis_sent = false;
                let mut need_evidence_probe = false;
                let mut pass_dis_tx: [Option<(u64, Instant)>; MAX_MODE_JOINTS] =
                    [None; MAX_MODE_JOINTS];
                for i in 0..n {
                    if self.mit_expiry_axes[i].dis_tx.is_some() {
                        continue;
                    }
                    match self.send_with_commissioning_gate(
                        TxKind::DisableRequested { started_at },
                        Some(start_stamp),
                        Some(Deadline { at: phase_deadline }),
                        pre_tx_gate,
                        |d| Some(damiao::build_disable_frame(d.motor_ids[i])),
                    ) {
                        Ok((_, tx_seq, tx_started, _)) => {
                            dis_sent = true;
                            pass_dis_tx[i] = Some((tx_seq, tx_started));
                        }
                        Err(Ok(code)) => {
                            clear_unconfirmed_dispatched(self);
                            return Err(format!("PRE_TX_GATE:{code}"));
                        }
                        Err(Err(TxRefusal::DeadlinePassed)) => break,
                        Err(Err(TxRefusal::DisableWaitEvidence)) => {
                            need_evidence_probe = true;
                            break;
                        }
                        Err(Err(r)) => {
                            clear_unconfirmed_dispatched(self);
                            return Err(Self::refusal_to_driver_err(r).to_string());
                        }
                    }
                }
                if dis_sent {
                    if let Err(code) = pre_tx_gate() {
                        clear_unconfirmed_dispatched(self);
                        return Err(format!("PRE_TX_GATE:{code}"));
                    }
                    if let Err(e) = self.flush_can_and_commit() {
                        clear_unconfirmed_dispatched(self);
                        return Err(format!("disable flush: {e}"));
                    }
                    for i in 0..n {
                        if let Some(entry) = pass_dis_tx[i] {
                            self.mit_expiry_axes[i].dis_tx = Some(entry);
                            if i < 64 {
                                self.disable_dispatched_mask |= 1u64 << i;
                            }
                        }
                    }
                }
                if need_evidence_probe {
                    if !self.in_active_control_release() {
                        self.begin_tx_tick();
                    }
                    let mut probe_sent = false;
                    for i in 0..n {
                        if self.mit_expiry_axes[i].dis_tx.is_some() {
                            continue;
                        }
                        match self.send_with_commissioning_gate(
                            TxKind::DisableProbe,
                            Some(start_stamp),
                            Some(Deadline { at: phase_deadline }),
                            pre_tx_gate,
                            |d| Some(damiao::build_refresh_frame(d.motor_ids[i])),
                        ) {
                            Ok(_) => probe_sent = true,
                            Err(Ok(code)) => {
                                clear_unconfirmed_dispatched(self);
                                return Err(format!("PRE_TX_GATE:{code}"));
                            }
                            Err(Err(_)) => break,
                        }
                    }
                    if probe_sent {
                        let _ = self.flush_can_and_commit();
                    }
                }
                let _ = service_holds_if_due(self, pre_tx_gate)?;
            }

            // Phase 2b: Drain RX immediately before EACH axis's REG_READ(RID_CTRL_MODE) barrier probe.
            for i in 0..n {
                if self.mit_expiry_axes[i].dis_tx.is_none()
                    || self.mit_expiry_axes[i].barrier_tx.is_some()
                {
                    continue;
                }
                if self.now() >= phase_deadline {
                    break;
                }
                let _ = service_holds_if_due(self, pre_tx_gate)?;
                let _ = self.drain_rx_only_record_registers();
                harvest_sync(self);
                if !self.last_read_drained || self.now() >= phase_deadline {
                    break;
                }
                if self.mit_expiry_axes[i].dis_tx.is_none()
                    || self.mit_expiry_axes[i].barrier_tx.is_some()
                {
                    continue;
                }
                if !self.in_active_control_release() {
                    self.begin_tx_tick();
                }
                let entry = match self.send_with_commissioning_gate(
                    TxKind::Refresh,
                    Some(start_stamp),
                    Some(Deadline { at: phase_deadline }),
                    pre_tx_gate,
                    |d| {
                        Some(damiao::build_read_register_frame(
                            d.motor_ids[i],
                            damiao::RID_CTRL_MODE,
                        ))
                    },
                ) {
                    Ok((_, tx_seq, tx_started, tx_offset_ns)) => (tx_seq, tx_started, tx_offset_ns),
                    Err(Ok(code)) => {
                        clear_unconfirmed_dispatched(self);
                        return Err(format!("PRE_TX_GATE:{code}"));
                    }
                    Err(Err(TxRefusal::DeadlinePassed)) => break,
                    Err(Err(r)) => {
                        clear_unconfirmed_dispatched(self);
                        return Err(format!(
                            "barrier read to J{} failed: {}",
                            i + 1,
                            Self::refusal_to_driver_err(r)
                        ));
                    }
                };
                if let Err(e) = self.pending_stop_check(Some(start_stamp)) {
                    clear_unconfirmed_dispatched(self);
                    return Err(format!("barrier flush inhibited: {e}"));
                }
                if let Err(code) = pre_tx_gate() {
                    clear_unconfirmed_dispatched(self);
                    return Err(format!("PRE_TX_GATE:{code}"));
                }
                if let Err(e) = self.flush_can_and_commit() {
                    clear_unconfirmed_dispatched(self);
                    return Err(format!("barrier flush: {e}"));
                }
                self.mit_expiry_axes[i].barrier_tx = Some(entry);
                let _ = self.drain_rx_only_record_registers();
                harvest_sync(self);
            }

            // Phase 2c-1: Drain RX immediately before EACH axis's Round 1 `0xCC` status probe.
            for i in 0..n {
                if self.mit_expiry_axes[i].barrier_ack.is_none()
                    || self.mit_expiry_axes[i].probe1_tx.is_some()
                {
                    continue;
                }
                if self.now() >= phase_deadline {
                    break;
                }
                let _ = service_holds_if_due(self, pre_tx_gate)?;
                let _ = self.drain_rx_only_record_registers();
                harvest_sync(self);
                if !self.last_read_drained || self.now() >= phase_deadline {
                    break;
                }
                if self.mit_expiry_axes[i].barrier_ack.is_none()
                    || self.mit_expiry_axes[i].probe1_tx.is_some()
                {
                    continue;
                }
                if !self.in_active_control_release() {
                    self.begin_tx_tick();
                }
                let entry = match self.send_with_commissioning_gate(
                    TxKind::Refresh,
                    Some(start_stamp),
                    Some(Deadline { at: phase_deadline }),
                    pre_tx_gate,
                    |d| Some(damiao::build_refresh_frame(d.motor_ids[i])),
                ) {
                    Ok((_, tx_seq, tx_started, tx_offset_ns)) => (tx_seq, tx_started, tx_offset_ns),
                    Err(Ok(code)) => {
                        clear_unconfirmed_dispatched(self);
                        return Err(format!("PRE_TX_GATE:{code}"));
                    }
                    Err(Err(TxRefusal::DeadlinePassed)) => break,
                    Err(Err(r)) => {
                        clear_unconfirmed_dispatched(self);
                        return Err(format!(
                            "status refresh round 1 to J{} failed: {}",
                            i + 1,
                            Self::refusal_to_driver_err(r)
                        ));
                    }
                };
                if let Err(e) = self.pending_stop_check(Some(start_stamp)) {
                    clear_unconfirmed_dispatched(self);
                    return Err(format!("refresh round 1 flush inhibited: {e}"));
                }
                if let Err(e) = self.flush_can_and_commit() {
                    clear_unconfirmed_dispatched(self);
                    return Err(format!("refresh round 1 flush: {e}"));
                }
                self.mit_expiry_axes[i].probe1_tx = Some(entry);
                self.mit_expiry_axes[i].open_reply_window(entry.1);
                let _ = self.drain_rx_only_record_registers();
                harvest_sync(self);
            }

            // Phase 2c-2: Drain RX immediately before EACH axis's Round 2 `0xCC` status probe,
            // strictly AFTER Round 1's confirming reply timestamp (`p2_tx_at > p1_ack_at`).
            for i in 0..n {
                if self.mit_expiry_axes[i].probe1_ack.is_none()
                    || self.mit_expiry_axes[i].probe2_tx.is_some()
                {
                    continue;
                }
                if self.now() >= phase_deadline {
                    break;
                }
                let _ = service_holds_if_due(self, pre_tx_gate)?;
                let _ = self.drain_rx_only_record_registers();
                harvest_sync(self);
                if !self.last_read_drained || self.now() >= phase_deadline {
                    break;
                }
                let Some((_, p1_ack_at)) = self.mit_expiry_axes[i].probe1_ack else {
                    continue;
                };
                if self.mit_expiry_axes[i].probe2_tx.is_some() {
                    continue;
                }
                if self.now() <= p1_ack_at {
                    let confirmed = self.mit_expiry_confirmed_mask(
                        self.now(),
                        phase_deadline,
                        n,
                        req_boundary,
                        &self.mit_expiry_axes,
                    );
                    let tick = self.mit_recovery_capped_wait(
                        n,
                        confirmed,
                        Duration::from_micros(50),
                    );
                    let step = tick.min(phase_deadline.saturating_duration_since(self.now()));
                    if !step.is_zero() {
                        self.pause(step);
                    }
                    if self.now() >= phase_deadline {
                        break;
                    }
                    let _ = self.drain_rx_only_record_registers();
                    harvest_sync(self);
                    if !self.last_read_drained || self.now() >= phase_deadline {
                        break;
                    }
                    let Some((_, p1_ack_at_after)) = self.mit_expiry_axes[i].probe1_ack else {
                        continue;
                    };
                    if self.mit_expiry_axes[i].probe2_tx.is_some() || self.now() <= p1_ack_at_after {
                        continue;
                    }
                }
                if !self.in_active_control_release() {
                    self.begin_tx_tick();
                }
                let entry = match self.send_with_commissioning_gate(
                    TxKind::Refresh,
                    Some(start_stamp),
                    Some(Deadline { at: phase_deadline }),
                    pre_tx_gate,
                    |d| Some(damiao::build_refresh_frame(d.motor_ids[i])),
                ) {
                    Ok((_, tx_seq, tx_started, tx_offset_ns)) => (tx_seq, tx_started, tx_offset_ns),
                    Err(Ok(code)) => {
                        clear_unconfirmed_dispatched(self);
                        return Err(format!("PRE_TX_GATE:{code}"));
                    }
                    Err(Err(TxRefusal::DeadlinePassed)) => break,
                    Err(Err(r)) => {
                        clear_unconfirmed_dispatched(self);
                        return Err(format!(
                            "status refresh round 2 to J{} failed: {}",
                            i + 1,
                            Self::refusal_to_driver_err(r)
                        ));
                    }
                };
                if let Err(e) = self.pending_stop_check(Some(start_stamp)) {
                    clear_unconfirmed_dispatched(self);
                    return Err(format!("refresh round 2 flush inhibited: {e}"));
                }
                if let Err(e) = self.flush_can_and_commit() {
                    clear_unconfirmed_dispatched(self);
                    return Err(format!("refresh round 2 flush: {e}"));
                }
                self.mit_expiry_axes[i].probe2_tx = Some(entry);
                self.mit_expiry_axes[i].open_reply_window(entry.1);
                let _ = self.drain_rx_only_record_registers();
                harvest_sync(self);
            }

            let hold_now = self.now();
            if hold_now >= phase_deadline {
                break;
            }
            let confirmed_disabled_mask = service_holds_if_due(self, pre_tx_gate)?;
            let wait = self.mit_recovery_capped_wait(
                n,
                confirmed_disabled_mask,
                Duration::from_micros(500),
            );
            if !wait.is_zero() {
                self.pause(wait);
            }
        }

        let check_now = self.now();
        if check_now >= phase_deadline {
            clear_unconfirmed_dispatched(self);
            return Err("MIT expiry recovery phase deadline expired before disable confirmation".into());
        }
        let missing: Vec<String> = (0..n)
            .filter(|&i| {
                !self.is_mit_expiry_axis_causally_disabled(
                    check_now,
                    phase_deadline,
                    i,
                    req_boundary,
                    &self.mit_expiry_axes[i],
                )
            })
            .map(|i| format!("J{}", i + 1))
            .collect();
        if !missing.is_empty() {
            clear_unconfirmed_dispatched(self);
            return Err(format!(
                "{} did not report post-recovery feedback after disable",
                missing.join(", ")
            ));
        }
        let boundary = self
            .last_enable_command_rx_seq
            .map_or(req_boundary, |s| s.max(req_boundary));
        self.last_mit_recovery_rx_seq = Some(boundary);
        self.mit_fence = None;
        self.is_enabled = false;
        self.is_enabling = false;
        self.motor_energized_mask = 0;
        self.disable_dispatched_mask = 0;
        Ok(())
        })();
        self.mit_expiry_recovery_active = false;
        self.mit_expiry_axes = [MitExpiryAxisState::default(); MAX_MODE_JOINTS];
        self.disable_rest_established = false;
        res
    }

    fn acknowledge_mit_fence_trip(&mut self) {
        if self.mit_fence_trip.is_some() { self.mit_fence_acked = true; }
    }

    fn mit_feedback_after_recovery(&self, dof: usize) -> bool {
        !self.awaiting_rx_boundary
            && self.last_mit_recovery_rx_seq.is_some_and(|req| {
                self.last_feedback_seq.len() == dof
                    && self.last_feedback_seq.iter().all(|&seq| seq > req)
            })
    }

    fn mit_min_last_ctrl_tx(&self, mask: u64) -> Option<Instant> {
        let n = self.motor_ids.len().min(MAX_MODE_JOINTS);
        let mut oldest: Option<Instant> = None;
        for i in 0..n {
            if (mask & (1u64 << i)) == 0 {
                continue;
            }
            let ts = self.motor_last_ctrl_tx.get(i).copied().flatten()?;
            oldest = Some(match oldest {
                Some(prev) => prev.min(ts),
                None => ts,
            });
        }
        oldest
    }

    fn mit_expiry_recover_reset(&mut self) {
        if self.mit_expiry_budgeted_boundary.is_some() {
            let n = self.motor_ids.len().min(MAX_MODE_JOINTS);
            for i in 0..n {
                if i < 64 && self.mit_expiry_axes[i].dis_tx.is_some() {
                    self.disable_dispatched_mask &= !(1u64 << i);
                }
            }
        }
        self.mit_expiry_recovery_active = false;
        self.disable_rest_established = false;
        self.mit_expiry_budgeted_boundary = None;
        self.mit_expiry_cleared_mask = 0;
        self.mit_expiry_confirmed_disabled_mask = 0;
        self.mit_expiry_axes = [MitExpiryAxisState::default(); MAX_MODE_JOINTS];
    }

    // Draws TX from the active release ledger via `ensure_tx_pass()` (never `begin_tx_tick()`)
    // and gates every drain step on `driver_remaining_release_rx()` so recovery yields cleanly
    // when the shared release RX budget is 0 without resetting causal evidence.
    fn mit_expiry_recover_budgeted(
        &mut self,
        deadline: Instant,
        started_at: Instant,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> Result<bool, ArmDriverError> {
        let n = self.motor_ids.len().min(MAX_MODE_JOINTS);
        if n == 0 {
            self.mit_expiry_recover_reset();
            self.mit_fence = None;
            return Ok(true);
        }
        if let Err((max_frames, required)) = self.check_watchdog_tx_budget_for_energized_inner() {
            self.mit_expiry_recover_reset();
            return Err(ArmDriverError::WatchdogBudgetInsufficient {
                max_frames,
                required,
            });
        }
        let phase_deadline = started_at
            .checked_add(STOPPED_DISABLE_REQUEST_LIFETIME)
            .unwrap_or(started_at);
        let effective_deadline = deadline.min(phase_deadline);
        let now = self.now();
        if now >= phase_deadline {
            self.mit_expiry_recover_reset();
            return Err(ArmDriverError::DisableRequestExpired);
        }
        if now >= deadline {
            return Ok(false);
        }
        // Single Rule A′ send-time gate before any clear-error or disable TX (zero alloc).
        let wait_evidence = match self.check_stopped_disable_requested(started_at) {
            Ok(()) => false,
            Err(TxRefusal::TorqueOffWouldDrop) => return Err(ArmDriverError::TorqueOffWouldDrop),
            Err(TxRefusal::DisableRequestExpired) => {
                self.mit_expiry_recover_reset();
                return Err(ArmDriverError::DisableRequestExpired);
            }
            Err(TxRefusal::DisableWaitEvidence) => true,
            Err(r) => return Err(Self::refusal_to_driver_err(r)),
        };
        let start_stamp = self.current_stop_stamp();
        self.discard_all_pending_tx();
        let _ = self.ensure_tx_pass();
        // Activate provisional-feedback protection BEFORE servicing holds or initial drain (and
        // preserve it across partial drains) so stale `state == 0` frames decoded before boundary
        // capture never clear `motor_energized_mask` or `motor_last_ctrl_tx` on still-energized axes.
        self.mit_expiry_recovery_active = true;
        // Service due, verified, still-energized arm continuity holds FIRST from the protected
        // continuity allowance (with live `pre_tx_gate`, explicit `effective_deadline`, release
        // coalescing, and excluding J7 and causally confirmed disabled axes) BEFORE any
        // discretionary RX drain or recovery TX can return `Pending` on RX/TX exhaustion.
        let pre_confirmed_mask = if let Some(existing_boundary) = self.mit_expiry_budgeted_boundary {
            let axes = self.mit_expiry_axes;
            self.mit_expiry_confirmed_mask(self.now(), effective_deadline, n, existing_boundary, &axes)
        } else {
            0
        };
        self.mit_expiry_confirmed_disabled_mask = pre_confirmed_mask;
        match self.service_mit_recovery_continuity_holds_inner(
            n,
            start_stamp,
            pre_confirmed_mask,
            Some(Deadline { at: effective_deadline }),
            true,
            pre_tx_gate,
        ) {
            Ok(_) => {}
            Err(MitRecoveryHoldErr::PreTxGate(_)) => return Ok(false),
            Err(MitRecoveryHoldErr::StopInhibited(_)) => return Err(ArmDriverError::EstopInhibited),
            Err(MitRecoveryHoldErr::FlushInhibited(e)) => return Err(e),
            Err(MitRecoveryHoldErr::WatchdogMissed(joint)) => {
                self.mit_expiry_recover_reset();
                return Err(ArmDriverError::WatchdogContinuityMissed { joint });
            }
        }
        if wait_evidence {
            return Ok(false);
        }
        let post_hold_now = self.now();
        if post_hold_now >= phase_deadline {
            self.mit_expiry_recover_reset();
            return Err(ArmDriverError::DisableRequestExpired);
        }
        if post_hold_now >= deadline {
            return Ok(false);
        }
        let mut rx_empty_after_disable = false;
        if self.mit_expiry_budgeted_boundary.is_none() {
            let rx_budget = self.driver_remaining_release_rx();
            if rx_budget == 0 {
                return Ok(false);
            }
            match self.commissioning_drain_rx_step(rx_budget, effective_deadline) {
                Ok((_, true)) => {
                    self.mit_expiry_recover_reset();
                    self.mit_expiry_recovery_active = true;
                    self.mit_expiry_budgeted_boundary = Some(self.rx_empty_seq);
                    self.last_mit_recovery_rx_seq = None;
                    self.comm_reg_reply_len = 0;
                    self.comm_reg_reply_overflow = false;
                }
                Ok((_, false)) | Err(_) => return Ok(false),
            }
        }
        let req_boundary = self.mit_expiry_budgeted_boundary.unwrap_or(0);

        let sync_confirmed_dispatched = |d: &mut Self| {
            let sync_now = d.now();
            let axes = d.mit_expiry_axes;
            let confirmed = d.mit_expiry_confirmed_mask(
                sync_now,
                effective_deadline,
                n,
                req_boundary,
                &axes,
            );
            d.mit_expiry_confirmed_disabled_mask = confirmed;
            for (i, axis) in axes.iter().enumerate().take(n) {
                if i < 64 && axis.dis_tx.is_some() {
                    if (confirmed & (1u64 << i)) != 0 {
                        d.disable_dispatched_mask |= 1u64 << i;
                    } else {
                        d.disable_dispatched_mask &= !(1u64 << i);
                    }
                }
            }
        };

        let harvest_and_reset_adverse = |d: &mut Self| {
            if !d.last_read_drained {
                return;
            }
            let harvest_now = d.now();
            let mut axes = d.mit_expiry_axes;
            let (reset_mask, reclear_mask) = d.harvest_mit_expiry_confirmations_into(
                harvest_now,
                effective_deadline,
                n,
                req_boundary,
                REGISTER_REPLY_TIMEOUT,
                &mut axes,
            );
            d.mit_expiry_axes = axes;
            d.disable_dispatched_mask &= !reset_mask;
            d.mit_expiry_cleared_mask &= !reclear_mask;
            if d.comm_reg_reply_overflow {
                d.comm_reg_reply_len = 0;
                d.comm_reg_reply_overflow = false;
            } else if reset_mask != 0 {
                let seq = d.rx_seq;
                for i in 0..n {
                    if i < 64 && (reset_mask & (1u64 << i)) != 0 {
                        let mid = d.motor_ids[i];
                        d.prune_comm_reg_replies_for_motor(mid, seq);
                    }
                }
            }
            sync_confirmed_dispatched(d);
        };

        // Drain existing replies whenever any axis already transmitted CMD_DISABLE, then harvest
        // sequential barrier/Round-1/Round-2 confirmations and reset any adverse/ambiguous axis.
        if self.mit_expiry_axes[..n].iter().any(|a| a.dis_tx.is_some()) {
            let rx_budget = self.driver_remaining_release_rx();
            if rx_budget == 0 {
                return Ok(false);
            }
            match self.commissioning_drain_rx_step(rx_budget, effective_deadline) {
                Ok((_, true)) => {
                    rx_empty_after_disable = true;
                }
                Ok((_, false)) | Err(_) => return Ok(false),
            }
            harvest_and_reset_adverse(self);
            let exp_now = self.now();
            if exp_now >= phase_deadline {
                self.mit_expiry_recover_reset();
                return Err(ArmDriverError::DisableRequestExpired);
            }
            let mut axes = self.mit_expiry_axes;
            let retried_barriers = self.expire_timed_out_mit_probes(
                exp_now,
                n,
                REGISTER_REPLY_TIMEOUT,
                REGISTER_REPLY_TIMEOUT,
                &mut axes,
            );
            self.mit_expiry_axes = axes;
            if retried_barriers != 0 {
                let seq = self.rx_seq;
                for i in 0..n {
                    if i < 64 && (retried_barriers & (1u64 << i)) != 0 {
                        let mid = self.motor_ids[i];
                        self.prune_comm_reg_replies_for_motor(mid, seq);
                    }
                }
            }
            sync_confirmed_dispatched(self);
        }

        // Phase 1: send clear-error (FF..FB) to every motor reporting an error state (>= 8).
        let mut pass_cleared_mask = 0u64;
        let mut clear_sent = false;
        let mut any_clear_pending = false;
        for i in 0..n {
            if self.motor_status[i] >= 8
                && ((self.mit_expiry_cleared_mask | pass_cleared_mask) & (1u64 << i)) == 0
            {
                if self.now() >= effective_deadline {
                    any_clear_pending = true;
                    break;
                }
                match self.send_with_commissioning_gate(
                    TxKind::ClearFault,
                    Some(start_stamp),
                    Some(Deadline { at: effective_deadline }),
                    pre_tx_gate,
                    |d| Some(damiao::build_clear_faults_frame(d.motor_ids[i])),
                ) {
                    Ok(_) => {
                        pass_cleared_mask |= 1u64 << i;
                        clear_sent = true;
                    }
                    Err(Ok(_revoked)) => return Ok(false),
                    Err(Err(TxRefusal::Deferred | TxRefusal::DeadlinePassed)) => {
                        any_clear_pending = true;
                        break;
                    }
                    Err(Err(TxRefusal::Transport(e))) if e.is_tx_backpressure() => {
                        any_clear_pending = true;
                        break;
                    }
                    Err(Err(r)) => return Err(Self::refusal_to_driver_err(r)),
                }
            }
        }
        if clear_sent {
            self.pending_stop_check(Some(start_stamp))?;
            if self.flush_cutoff_exceeded() || self.flush_can_and_commit().is_err() {
                return Ok(false);
            }
            self.mit_expiry_cleared_mask |= pass_cleared_mask;
        }
        if any_clear_pending {
            return Ok(false);
        }

        // Phase 2a: Send CMD_DISABLE via TxKind::MitRecoveryDisable (budgeted against TxBudget
        // AND enforcing Rule A′ + pre_tx_gate at CAN submission). Do NOT mark
        // `disable_dispatched_mask` on host submission alone: keep energized axes eligible for
        // measured-position holds until full two-round confirmation succeeds.
        self.clear_cached_targets();
        let mut dis_sent = false;
        let mut any_disable_pending = false;
        let mut pass_disable_tx_seq: [Option<(u64, Instant)>; MAX_MODE_JOINTS] =
            [None; MAX_MODE_JOINTS];
        for i in 0..n {
            if self.mit_expiry_axes[i].dis_tx.is_some() {
                continue;
            }
            if self.now() >= effective_deadline {
                any_disable_pending = true;
                break;
            }
            match self.send_with_commissioning_gate(
                TxKind::MitRecoveryDisable { started_at },
                Some(start_stamp),
                Some(Deadline { at: effective_deadline }),
                pre_tx_gate,
                |d| Some(damiao::build_disable_frame(d.motor_ids[i])),
            ) {
                Ok((_, tx_seq, tx_started, _)) => {
                    dis_sent = true;
                    pass_disable_tx_seq[i] = Some((tx_seq, tx_started));
                }
                Err(Ok(_revoked)) => return Ok(false),
                Err(Err(TxRefusal::TorqueOffWouldDrop)) => return Err(ArmDriverError::TorqueOffWouldDrop),
                Err(Err(TxRefusal::DisableRequestExpired)) => {
                    self.mit_expiry_recover_reset();
                    return Err(ArmDriverError::DisableRequestExpired);
                }
                Err(Err(TxRefusal::DisableWaitEvidence | TxRefusal::Deferred | TxRefusal::DeadlinePassed)) => {
                    any_disable_pending = true;
                    break;
                }
                Err(Err(TxRefusal::Transport(e))) if e.is_tx_backpressure() => {
                    any_disable_pending = true;
                    break;
                }
                Err(Err(r)) => return Err(Self::refusal_to_driver_err(r)),
            }
        }
        if dis_sent {
            self.pending_stop_check(Some(start_stamp))?;
            if self.flush_cutoff_exceeded() || self.flush_can_and_commit().is_err() {
                return Ok(false);
            }
            for i in 0..n {
                if let Some(entry) = pass_disable_tx_seq[i] {
                    self.mit_expiry_axes[i].dis_tx = Some(entry);
                }
            }
            self.disable_rest_established = true;
            let rx_budget = self.driver_remaining_release_rx();
            if rx_budget == 0 {
                return Ok(false);
            }
            match self.commissioning_drain_rx_step(rx_budget, effective_deadline) {
                Ok((_, true)) => {
                    rx_empty_after_disable = true;
                }
                Ok((_, false)) | Err(_) => return Ok(false),
            }
            harvest_and_reset_adverse(self);
        }
        if any_disable_pending {
            return Ok(false);
        }

        // Phase 2b: Drain RX immediately before EACH axis's REG_READ(RID_CTRL_MODE) barrier probe.
        let mut any_barrier_pending = false;
        for i in 0..n {
            if self.mit_expiry_axes[i].dis_tx.is_none()
                || self.mit_expiry_axes[i].barrier_tx.is_some()
            {
                continue;
            }
            if self.now() >= effective_deadline {
                any_barrier_pending = true;
                break;
            }
            let rx_budget = self.driver_remaining_release_rx();
            if rx_budget == 0 {
                return Ok(false);
            }
            match self.commissioning_drain_rx_step(rx_budget, effective_deadline) {
                Ok((_, true)) => {
                    rx_empty_after_disable = true;
                }
                Ok((_, false)) | Err(_) => return Ok(false),
            }
            harvest_and_reset_adverse(self);
            if self.now() >= effective_deadline {
                any_barrier_pending = true;
                break;
            }
            if self.mit_expiry_axes[i].dis_tx.is_none()
                || self.mit_expiry_axes[i].barrier_tx.is_some()
            {
                continue;
            }
            let entry = match self.send_with_commissioning_gate(
                TxKind::Refresh,
                Some(start_stamp),
                Some(Deadline { at: effective_deadline }),
                pre_tx_gate,
                |d| Some(damiao::build_read_register_frame(d.motor_ids[i], damiao::RID_CTRL_MODE)),
            ) {
                Ok((_, tx_seq, tx_started, tx_offset_ns)) => (tx_seq, tx_started, tx_offset_ns),
                Err(Ok(_revoked)) => return Ok(false),
                Err(Err(TxRefusal::Deferred | TxRefusal::DeadlinePassed)) => {
                    any_barrier_pending = true;
                    break;
                }
                Err(Err(TxRefusal::Transport(e))) if e.is_tx_backpressure() => {
                    any_barrier_pending = true;
                    break;
                }
                Err(Err(r)) => return Err(Self::refusal_to_driver_err(r)),
            };
            self.pending_stop_check(Some(start_stamp))?;
            if self.flush_cutoff_exceeded() || self.flush_can_and_commit().is_err() {
                return Ok(false);
            }
            self.mit_expiry_axes[i].barrier_tx = Some(entry);
            let rx_budget = self.driver_remaining_release_rx();
            if rx_budget == 0 {
                return Ok(false);
            }
            match self.commissioning_drain_rx_step(rx_budget, effective_deadline) {
                Ok((_, true)) => {
                    rx_empty_after_disable = true;
                }
                Ok((_, false)) | Err(_) => return Ok(false),
            }
            harvest_and_reset_adverse(self);
        }
        if any_barrier_pending {
            return Ok(false);
        }

        // Phase 2c-1: Drain RX immediately before EACH axis's Round 1 `0xCC` status probe.
        let mut any_p1_pending = false;
        for i in 0..n {
            if self.mit_expiry_axes[i].barrier_ack.is_none()
                || self.mit_expiry_axes[i].probe1_tx.is_some()
            {
                continue;
            }
            if self.now() >= effective_deadline {
                any_p1_pending = true;
                break;
            }
            let rx_budget = self.driver_remaining_release_rx();
            if rx_budget == 0 {
                return Ok(false);
            }
            match self.commissioning_drain_rx_step(rx_budget, effective_deadline) {
                Ok((_, true)) => {
                    rx_empty_after_disable = true;
                }
                Ok((_, false)) | Err(_) => return Ok(false),
            }
            harvest_and_reset_adverse(self);
            if self.now() >= effective_deadline {
                any_p1_pending = true;
                break;
            }
            if self.mit_expiry_axes[i].barrier_ack.is_none()
                || self.mit_expiry_axes[i].probe1_tx.is_some()
            {
                continue;
            }
            let entry = match self.send_with_commissioning_gate(
                TxKind::Refresh,
                Some(start_stamp),
                Some(Deadline { at: effective_deadline }),
                pre_tx_gate,
                |d| Some(damiao::build_refresh_frame(d.motor_ids[i])),
            ) {
                Ok((_, tx_seq, tx_started, tx_offset_ns)) => (tx_seq, tx_started, tx_offset_ns),
                Err(Ok(_revoked)) => return Ok(false),
                Err(Err(TxRefusal::Deferred | TxRefusal::DeadlinePassed)) => {
                    any_p1_pending = true;
                    break;
                }
                Err(Err(TxRefusal::Transport(e))) if e.is_tx_backpressure() => {
                    any_p1_pending = true;
                    break;
                }
                Err(Err(r)) => return Err(Self::refusal_to_driver_err(r)),
            };
            self.pending_stop_check(Some(start_stamp))?;
            if self.flush_cutoff_exceeded() || self.flush_can_and_commit().is_err() {
                return Ok(false);
            }
            self.mit_expiry_axes[i].probe1_tx = Some(entry);
            self.mit_expiry_axes[i].open_reply_window(entry.1);
            let rx_budget = self.driver_remaining_release_rx();
            if rx_budget == 0 {
                return Ok(false);
            }
            match self.commissioning_drain_rx_step(rx_budget, effective_deadline) {
                Ok((_, true)) => {
                    rx_empty_after_disable = true;
                }
                Ok((_, false)) | Err(_) => return Ok(false),
            }
            harvest_and_reset_adverse(self);
        }
        if any_p1_pending {
            return Ok(false);
        }

        // Phase 2c-2: Drain RX immediately before EACH axis's Round 2 `0xCC` status probe,
        // strictly AFTER Round 1's confirming reply timestamp (`p2_tx_at > p1_ack_at`).
        let mut any_p2_pending = false;
        for i in 0..n {
            if self.mit_expiry_axes[i].probe1_ack.is_none()
                || self.mit_expiry_axes[i].probe2_tx.is_some()
            {
                continue;
            }
            if self.now() >= effective_deadline {
                any_p2_pending = true;
                break;
            }
            let rx_budget = self.driver_remaining_release_rx();
            if rx_budget == 0 {
                return Ok(false);
            }
            match self.commissioning_drain_rx_step(rx_budget, effective_deadline) {
                Ok((_, true)) => {
                    rx_empty_after_disable = true;
                }
                Ok((_, false)) | Err(_) => return Ok(false),
            }
            harvest_and_reset_adverse(self);
            if self.now() >= effective_deadline {
                any_p2_pending = true;
                break;
            }
            let Some((_, p1_ack_at)) = self.mit_expiry_axes[i].probe1_ack else {
                continue;
            };
            if self.mit_expiry_axes[i].probe2_tx.is_some() {
                continue;
            }
            // Defer Round 2 until submission timestamp is strictly greater than Round 1's reply.
            if self.now() <= p1_ack_at {
                any_p2_pending = true;
                continue;
            }
            let entry = match self.send_with_commissioning_gate(
                TxKind::Refresh,
                Some(start_stamp),
                Some(Deadline { at: effective_deadline }),
                pre_tx_gate,
                |d| Some(damiao::build_refresh_frame(d.motor_ids[i])),
            ) {
                Ok((_, tx_seq, tx_started, tx_offset_ns)) => (tx_seq, tx_started, tx_offset_ns),
                Err(Ok(_revoked)) => return Ok(false),
                Err(Err(TxRefusal::Deferred | TxRefusal::DeadlinePassed)) => {
                    any_p2_pending = true;
                    break;
                }
                Err(Err(TxRefusal::Transport(e))) if e.is_tx_backpressure() => {
                    any_p2_pending = true;
                    break;
                }
                Err(Err(r)) => return Err(Self::refusal_to_driver_err(r)),
            };
            self.pending_stop_check(Some(start_stamp))?;
            if self.flush_cutoff_exceeded() || self.flush_can_and_commit().is_err() {
                return Ok(false);
            }
            self.mit_expiry_axes[i].probe2_tx = Some(entry);
            self.mit_expiry_axes[i].open_reply_window(entry.1);
            let rx_budget = self.driver_remaining_release_rx();
            if rx_budget == 0 {
                return Ok(false);
            }
            match self.commissioning_drain_rx_step(rx_budget, effective_deadline) {
                Ok((_, true)) => {
                    rx_empty_after_disable = true;
                }
                Ok((_, false)) | Err(_) => return Ok(false),
            }
            harvest_and_reset_adverse(self);
        }
        if any_p2_pending {
            return Ok(false);
        }

        if !rx_empty_after_disable || !self.last_read_drained {
            return Ok(false);
        }
        let post_now = self.now();
        if post_now >= phase_deadline {
            self.mit_expiry_recover_reset();
            return Err(ArmDriverError::DisableRequestExpired);
        }
        if post_now >= deadline {
            return Ok(false);
        }
        for i in 0..n {
            if !self.is_mit_expiry_axis_causally_disabled(
                post_now,
                effective_deadline,
                i,
                req_boundary,
                &self.mit_expiry_axes[i],
            ) {
                return Ok(false);
            }
        }
        self.last_mit_recovery_rx_seq = Some(self.rx_empty_seq);
        self.mit_expiry_recover_reset();
        self.mit_fence = None;
        self.is_enabled = false;
        self.is_enabling = false;
        self.motor_energized_mask = 0;
        self.disable_dispatched_mask = 0;
        Ok(true)
    }
}
