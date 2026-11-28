//! Damiao enable handshake send, readback, restore, and confirmation polling.

use std::time::{Duration, Instant};

use crate::driver::*;
use crate::driver::can_servo::*;
use crate::protocol::damiao;
use crate::transport::{CanTransport, TransportError, MAX_REALTIME_MONOTONIC_OFFSET_DRIFT_NS};

impl<C: CanTransport> DamiaoArmDriver<C> {
    pub(in crate::driver::can_servo) fn abort_enable_for_estop(&mut self) {
        if matches!(self.torque_hs, Some(DamiaoTorque::DrainBeforeEnable(_))) {
            self.awaiting_rx_boundary = false;
        }
        self.torque_hs = None;
        self.enable_failed();
    }

    /// A failed or abandoned enable: the arm needs a full rearm.
    /// L2: leaves `pos_force_verified` untouched — if enable failed before `finish_pos_force_readback`
    /// succeeded, `readback_or_enable` already cleared it to `false` (leaving capped operation
    /// unavailable); if enable failed after `finish_pos_force_readback` causally verified J7 and
    /// emitted its pre-enable hold, `pos_force_verified` stays `true` so `FailedHold` keeps holding
    /// J7 through the single J7 emitter.
    pub(in crate::driver::can_servo) fn enable_failed(&mut self) {
        self.is_enabling = false;
        self.is_enabled = false;
        self.pending_rearm = true;
        self.refresh_capture = None;
        self.enable_stop_stamp = None;
        self.enable_tx_deadline = None;
        self.enable_lease_gen = None;
        self.enable_frames_sent_any = false;
        if self.startup_register_check_pending {
            if self.timeout_check_enabled {
                self.timeout_verified.fill(false);
            }
            let mut mask = self.attempt_verified_modes_mask;
            while mask != 0 {
                let idx = mask.trailing_zeros() as usize;
                mask &= mask - 1;
                if idx < self.verified_modes.len() {
                    self.verified_modes[idx] = None;
                }
            }
        }
        self.attempt_verified_modes_mask = 0;
    }

    /// Checks whether the installed enable authority (atomic lease fence or driver-clock ceiling
    /// `min(lease_expiry, request_deadline)`) has been revoked, aborting immediately before TX.
    #[inline]
    pub(in crate::driver::can_servo) fn check_enable_tx_authority(&mut self) -> Result<(), ArmDriverError> {
        if self.enable_lease_gen.is_some_and(|g| self.is_lease_fenced(g)) {
            self.torque_hs = None;
            self.enable_failed();
            return Err(ArmDriverError::LeaseFenced);
        }
        if self.enable_tx_deadline.is_some_and(|dl| self.now() >= dl) {
            self.torque_hs = None;
            self.enable_failed();
            return Err(ArmDriverError::EnableAuthorityExpired);
        }
        Ok(())
    }

    /// Aborts an enable handshake if E-stop is latched, an urgent stop is pending, or a
    /// stop/E-stop generation advanced since the handshake began (even after `requested == applied`).
    pub(in crate::driver::can_servo) fn enable_estop_inhibited(&mut self) -> Result<(), ArmDriverError> {
        if let Err(e) = self.strict_stop_check(self.enable_stop_stamp) {
            self.abort_enable_for_estop();
            return Err(e);
        }
        Ok(())
    }

    /// Enable, first step: fast path, range check, receive boundary, then either the base-mode
    /// restore (`Restore`) or the enable frames (`Enable`).
    pub(in crate::driver::can_servo) fn begin_enable(&mut self) -> Result<TorqueProgress, ArmDriverError> {
        self.begin_enable_budgeted_inner(None, false)
    }

    pub(in crate::driver::can_servo) fn begin_enable_budgeted(&mut self, max_rx_frames: Option<u16>) -> Result<TorqueProgress, ArmDriverError> {
        self.begin_enable_budgeted_inner(max_rx_frames, false)
    }

    pub(in crate::driver::can_servo) fn begin_enable_budgeted_inner(
        &mut self,
        max_rx_frames: Option<u16>,
        allow_ticked_range_retry: bool,
    ) -> Result<TorqueProgress, ArmDriverError> {
        if self.torque_hs.is_some() {
            return Err(ArmDriverError::ProtocolStatic("a torque transition is pending"));
        }
        if let Err((max_frames, required)) = self.check_watchdog_tx_budget_for_enable_inner() {
            return Err(ArmDriverError::WatchdogBudgetInsufficient {
                max_frames,
                required,
            });
        }
        self.discard_all_pending_tx();
        if max_rx_frames.is_none() {
            self.begin_tx_tick();
        }
        self.refresh_scheduler.reset_sweep();
        self.enable_stop_stamp = None;
        self.enable_frames_sent_any = false;
        self.attempt_verified_modes_mask = 0;
        self.enable_estop_inhibited()?;
        // Fast-path: only if !pending_rearm && is_enabled && feedback_valid && all state == 1
        // (never after an MIT deadline trip: the rearm restores the modes and re-enables;
        // a position-force gripper also requires verified mode 4 and fresh <= 100 ms feedback).
        // Checked BEFORE urgent watchdog holds so a fast-path enable lets Step 5 emit the single
        // normal control batch for this release without duplicate holds.
        if self.mit_fence_trip.is_none()
            && !self.pending_rearm
            && !self.startup_register_check_pending
            && self.is_enabled
            && (0..self.motor_ids.len()).all(|i| {
                self.verified_modes.get(i).copied().flatten() == Some(self.base_mode(i))
                    && self.joint_modes.get(i).copied().flatten() == Some(self.base_mode(i))
                    && (!self.timeout_check_enabled || self.timeout_verified.get(i).copied().unwrap_or(false))
            })
            && self.pos_force.as_ref().is_none_or(|pf| {
                self.pos_force_verified
                    && self.joint_modes.get(pf.joint).copied().flatten() == Some(DamiaoControlMode::PosForce)
                    && self.fresh_measured_position(pf.joint, POS_FORCE_FEEDBACK_MAX_AGE).is_some()
            })
            && self.is_feedback_valid()
            && self.enable_state() == EnableState::Enabled
            && self.motor_status.iter().all(|&st| st == 1)
        {
            // When no subsequent Step 5 output owns this release (off-tick blocking `set_torque(true)`
            // or Window 2 wake), service any already-due energized watchdog holds inline before
            // returning `Confirmed`.
            if !(self.output_step_pending_in_release && self.in_active_control_release())
                && self.motor_energized_mask != 0
            {
                let _ = self.service_energized_watchdog_holds(
                    WATCHDOG_HOLD_URGENT_INTERVAL,
                    None,
                    0,
                    false,
                );
            }
            self.enable_tx_deadline = None;
            self.enable_lease_gen = None;
            return Ok(TorqueProgress::Done(TorqueOutcome::Confirmed));
        }
        if self.motor_energized_mask != 0 {
            let _ = self.service_energized_watchdog_holds(
                WATCHDOG_HOLD_URGENT_INTERVAL,
                None,
                0,
                false,
            );
        }

        // A motor that did not answer the startup range read gets another (read-only)
        // try before each enable in the legacy path; budgeted RT commits fail closed
        // (`RangeCheckUnverified`), while budgeted legacy WS commits run a bounded ticked
        // `DamiaoTorque::RangeCheck` phase without blocking under the supervisor write guard.
        if self.range_check.as_ref().is_some_and(|c| c.status == RangeCheckStatus::Unverified) {
            if max_rx_frames == Some(0) {
                if allow_ticked_range_retry {
                    let mut pending_mask = 0u32;
                    if let Some(ref rc) = self.range_check {
                        let dof = self.motor_ids.len().min(handshake::MAX_MASK_JOINTS);
                        for (i, j) in rc.joints.iter().enumerate().take(dof) {
                            if j.status == RangeCheckStatus::Unverified {
                                pending_mask |= 1u32 << i;
                            }
                        }
                    }
                    if pending_mask != 0 {
                        self.enable_stop_stamp = Some(self.current_stop_stamp());
                        self.is_enabling = true;
                        self.last_command_was_disable = false;
                        self.disable_dispatched_mask = 0;
                        self.disable_rest_established = false;
                        self.disable_tx_deadline = None;
                        self.disable_lease_gen = None;
                        let now = self.now();
                        self.torque_hs = Some(DamiaoTorque::RangeCheck(RangeCheckPhase {
                            pending_mask,
                            active_joint: None,
                            boundary: None,
                            sent_rids: 0,
                            deadline: Deadline::after(now, DAMIAO_RX_PASS),
                        }));
                        return Ok(TorqueProgress::Pending { wake: now });
                    }
                } else {
                    self.is_enabled = false;
                    self.pending_rearm = true;
                    return Err(ArmDriverError::RangeCheckUnverified);
                }
            } else {
                self.check_mapping_ranges();
            }
        }
        if self.range_check.as_ref().is_some_and(|c| c.status == RangeCheckStatus::Mismatch) {
            self.is_enabled = false;
            self.pending_rearm = true;
            return Err(ArmDriverError::RangeCheckMismatch);
        }

        self.enable_stop_stamp = Some(self.current_stop_stamp());
        self.is_enabling = true;
        self.last_command_was_disable = false;
        self.disable_dispatched_mask = 0;
        self.disable_rest_established = false;
        self.disable_tx_deadline = None;
        self.disable_lease_gen = None;

        let budget_armed = match max_rx_frames {
            Some(limit) => {
                self.last_poll_rx = None;
                self.rx_frame_limit = Some(self.clamp_release_rx_limit(limit));
                true
            }
            None => false,
        };
        let res = (|| -> Result<TorqueProgress, ArmDriverError> {
            let drained_now = match max_rx_frames {
                Some(0) => {
                    if !self.last_read_drained {
                        // Do not transmit enable frames or drain RX in Step 3; defer draining
                        // to `poll_torque_budgeted` until an empty-queue boundary is reached.
                        self.awaiting_rx_boundary = true;
                        let now = self.now();
                        self.torque_hs = Some(DamiaoTorque::DrainBeforeEnable(Deadline::after(now, DAMIAO_RX_PASS)));
                        return Ok(TorqueProgress::Pending { wake: now });
                    }
                    false
                }
                Some(limit) => {
                    if !self.drain_rx_before_command_limit(usize::from(limit)) {
                        self.awaiting_rx_boundary = true;
                        self.enable_failed();
                        return Err(ArmDriverError::RxDrainTimeout);
                    }
                    true
                }
                None => {
                    if !self.drain_rx_before_command() {
                        self.awaiting_rx_boundary = true;
                        self.enable_failed();
                        return Err(ArmDriverError::RxDrainTimeout);
                    }
                    true
                }
            };
            self.continue_enable_after_drain(drained_now)
        })();
        if budget_armed {
            self.rx_frame_limit = None;
        }
        res
    }

    pub(in crate::driver::can_servo) fn continue_enable_after_drain(&mut self, drained_now: bool) -> Result<TorqueProgress, ArmDriverError> {
        self.enable_estop_inhibited()?;
        self.check_enable_tx_authority()?;
        self.awaiting_rx_boundary = false;

        let dof = self.motor_ids.len();
        if dof > MAX_MODE_JOINTS {
            self.enable_failed();
            return Err(ArmDriverError::Protocol("joint count exceeds MAX_MODE_JOINTS".into()));
        }
        let all_mask = if dof == 0 {
            0u32
        } else if dof >= 32 {
            u32::MAX
        } else {
            (1u32 << dof) - 1
        };
        let pf_joint = self.pos_force.as_ref().map(|p| p.joint);
        let needs_freshen = (0..dof).any(|i| {
            if pf_joint == Some(i) {
                self.fresh_measured_position(i, ENABLE_FRESHEN_ENTRY_PF_MAX_AGE)
                    .is_none()
            } else {
                !self.cached_states[i].position.is_finite()
                    || self.last_feedback_instant[i]
                        .is_none_or(|t| self.since(t) > ENABLE_FRESHEN_ENTRY_ARM_MAX_AGE)
            }
        });
        if needs_freshen && all_mask != 0 {
            return self.start_enable_freshen(all_mask, drained_now);
        }
        self.proceed_after_freshen(drained_now)
    }

    /// Every enable requires causal `verified_modes[j] == Some(base_mode(j))` for every joint it
    /// energizes (L2), fresh causal `CTRL_MODE` and `TIMEOUT` replies from all motors on startup
    /// and after BusLoss, plus a fresh `CTRL_MODE` readback for a position-force gripper on every
    /// enable (a motor power cycle resets RAM registers).
    pub(in crate::driver::can_servo) fn readback_or_enable(&mut self) -> Result<TorqueProgress, ArmDriverError> {
        let dof = self.motor_ids.len().min(handshake::MAX_MASK_JOINTS);
        let pf_joint = self.pos_force.as_ref().map(|p| p.joint);
        let mut mode_mask = 0u32;
        let mut timeout_mask = 0u32;
        for i in 0..dof {
            let base = self.base_mode(i);
            if self.startup_register_check_pending
                || self.verified_modes.get(i).copied().flatten() != Some(base)
                || self.joint_modes.get(i).copied().flatten() != Some(base)
                || pf_joint == Some(i)
            {
                mode_mask |= 1u32 << i;
            }
            if self.timeout_check_enabled
                && (self.startup_register_check_pending
                    || !self.timeout_verified.get(i).copied().unwrap_or(false))
            {
                timeout_mask |= 1u32 << i;
            }
        }
        if pf_joint.is_some() {
            self.pos_force_verified = false;
        }
        if mode_mask == 0 && timeout_mask == 0 {
            return self.send_enable();
        }
        let now = self.now();
        let mut dl_at = now + REGISTER_REPLY_TIMEOUT;
        if let Some(auth_dl) = self.enable_tx_deadline {
            dl_at = dl_at.min(auth_dl);
        }
        let (stage, remaining_mask, timeout_pending_mask) = if mode_mask != 0 {
            (ReadbackStage::CtrlMode, mode_mask, timeout_mask)
        } else {
            (ReadbackStage::Timeout, timeout_mask, 0)
        };
        self.torque_hs = Some(DamiaoTorque::Readback(ModeReadback {
            stage,
            remaining_mask,
            sent_mask: 0,
            timeout_pending_mask,
            log_cleared: false,
            tx_evidence: [None; MAX_MODE_JOINTS],
            deadline: Deadline { at: dl_at },
        }));
        Ok(TorqueProgress::Pending { wake: now })
    }

    /// Causal, zero-allocation register-read reply matcher for `DamiaoTorque::Readback`
    /// (`RID_CTRL_MODE` and `RID_TIMEOUT`). Filters strictly by `valid_register_reply_id`,
    /// per-axis `after_tx_seq`, `min_rx_at` (rejecting delayed pre-TX traffic as `Ambiguous`),
    /// clock-offset drift, and `< max_rx_at`; conflicting values in one window fail closed as
    /// `Ambiguous` while identical timely read-only values are accepted.
    #[allow(clippy::too_many_arguments)]
    pub(in crate::driver::can_servo) fn find_enable_readback_reply(
        &self,
        joint: usize,
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
            if rep.motor_id != id
                || rep.write
                || rep.rid != rid
                || !self.valid_register_reply_id(joint, rep.reply_id)
            {
                continue;
            }
            if rep.rx_seq <= after_tx_seq {
                continue;
            }
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
            if max_rx_at.is_some_and(|dl| rep.rx_at >= dl) {
                continue;
            }
            if let Some(prev) = matched {
                if prev.value != rep.value {
                    return crate::mit::CommissioningRegMatch::Ambiguous;
                }
            } else {
                matched = Some(rep);
            }
        }
        match matched {
            Some(rep) => crate::mit::CommissioningRegMatch::Matched(rep),
            None => crate::mit::CommissioningRegMatch::Pending,
        }
    }

    /// A failed enable (timeout / TX failure / pre-enable hold refusal) NEVER clears a
    /// `verified_modes[j]` flag that came from a causal readback after this daemon started (L2),
    /// while discarding any unconfirmed proofs gained during an aborted startup/post-BusLoss attempt.
    /// Non-PosForce readback timeouts also keep `joint_modes[j]` intact so a silent `RID_CTRL_MODE`
    /// read retries read-only `REG_READ` on the next attempt rather than blindly issuing `REG_WRITE`.
    pub(in crate::driver::can_servo) fn readback_failed_err(&mut self, j: usize, err: ArmDriverError) -> Result<TorqueProgress, ArmDriverError> {
        if self.pos_force.as_ref().is_some_and(|pf| pf.joint == j)
            && self.verified_modes.get(j).copied().flatten().is_none()
        {
            self.joint_modes[j] = None;
        }
        self.enable_failed();
        Err(err)
    }

    /// Causal post-boundary `RID_CTRL_MODE` readback explicitly proved joint `j` is in the wrong
    /// mode: invalidate its mode state so the next enable restores the base mode before energizing.
    pub(in crate::driver::can_servo) fn readback_failed_mismatch(
        &mut self,
        j: usize,
        expected: u32,
        observed: u32,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.joint_modes[j] = None;
        self.verified_modes[j] = None;
        self.pos_force_verified = false;
        self.enable_failed();
        Err(ArmDriverError::CtrlModeReadbackMismatch {
            joint: j,
            expected,
            observed,
        })
    }

    /// Serve the pre-enable `RID_CTRL_MODE` and `RID_TIMEOUT` readbacks (L2 + power-overload PR 1):
    /// each joint's request goes out only after an observed empty RX queue and records per-request
    /// causal TX evidence `(after_tx_seq, tx_at, tx_offset_ns)`; replies are matched only after
    /// bounded RX queue closure through [`Self::commissioning_find_register_reply`] with reply CAN
    /// ID validation so stale, pre-TX, clock-drifted, duplicate, or spoofed replies fail closed.
    /// Zero heap allocations on the polling path.
    pub(in crate::driver::can_servo) fn poll_readback(&mut self, mut r: ModeReadback) -> Result<TorqueProgress, ArmDriverError> {
        loop {
            self.enable_estop_inhibited()?;
            let expired_before_read = r.deadline.passed(self.now());
            if !expired_before_read {
                let _ = self.receive_feedback();
                self.enable_estop_inhibited()?;
            }
            if !r.log_cleared
                && !expired_before_read
                && !r.deadline.passed(self.now())
                && self.last_read_drained
            {
                self.commissioning_clear_register_log();
                r.log_cleared = true;
            }
            let rid = match r.stage {
                ReadbackStage::CtrlMode => damiao::RID_CTRL_MODE,
                ReadbackStage::Timeout => damiao::RID_TIMEOUT,
            };
            let mut sent_any = false;
            if r.log_cleared && self.last_read_drained && !expired_before_read && !r.deadline.passed(self.now()) {
                let unsent = r.remaining_mask & !r.sent_mask;
                if self.motor_energized_mask != 0 {
                    self.service_urgent_enable_watchdog_holds(0, unsent != 0)?;
                }
                let mut first_sent_joint = None;
                for j in handshake::mask_joints(unsent) {
                    if j >= MAX_MODE_JOINTS {
                        return self.readback_failed_err(
                            j,
                            ArmDriverError::ProtocolStatic("joint index exceeds readback capacity"),
                        );
                    }
                    let id = self.motor_ids[j];
                    let mut revoked: Option<&'static str> = None;
                    match self.send(
                        TxKind::Handshake,
                        self.enable_stop_stamp,
                        Some(r.deadline),
                        false,
                        ContinuityBypass::None,
                        None,
                        &mut revoked,
                        |_| Some(damiao::build_read_register_frame(id, rid)),
                    ) {
                        Ok((_, tx_seq, tx_at, tx_offset_ns)) => {
                            r.sent_mask |= 1u32 << j;
                            r.tx_evidence[j] = Some((tx_seq, tx_at, tx_offset_ns));
                            sent_any = true;
                            if first_sent_joint.is_none() {
                                first_sent_joint = Some(j);
                            }
                        }
                        Err(TxRefusal::DeadlinePassed | TxRefusal::Deferred) => break,
                        Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                            self.abort_enable_for_estop();
                            return Err(ArmDriverError::EstopInhibited);
                        }
                        Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => break,
                        Err(TxRefusal::Transport(e)) => {
                            let kind = TxFailKind::from_transport(&e);
                            return match r.stage {
                                ReadbackStage::CtrlMode => self.readback_failed_err(
                                    j,
                                    ArmDriverError::PosForceReadbackTxFailed { joint: j, kind },
                                ),
                                ReadbackStage::Timeout => {
                                    self.enable_failed();
                                    Err(ArmDriverError::TimeoutReadbackTxFailed { joint: j, kind })
                                }
                            };
                        }
                        Err(r_err) => {
                            let err = Self::refusal_to_driver_err(r_err);
                            return match r.stage {
                                ReadbackStage::CtrlMode => self.readback_failed_err(j, err),
                                ReadbackStage::Timeout => {
                                    self.enable_failed();
                                    Err(err)
                                }
                            };
                        }
                    }
                }
                if sent_any {
                    self.enable_estop_inhibited()?;
                    if let Err(e) = self.flush_can_and_commit() {
                        let j = first_sent_joint.unwrap_or(0);
                        let kind = TxFailKind::from_transport(&e);
                        return match r.stage {
                            ReadbackStage::CtrlMode => self.readback_failed_err(
                                j,
                                ArmDriverError::PosForceReadbackTxFailed { joint: j, kind },
                            ),
                            ReadbackStage::Timeout => {
                                self.enable_failed();
                                Err(ArmDriverError::TimeoutReadbackTxFailed { joint: j, kind })
                            }
                        };
                    }
                }
            }
            if r.log_cleared {
                let awaiting = r.remaining_mask & r.sent_mask;
                for j in handshake::mask_joints(awaiting) {
                    let Some((after_tx_seq, tx_at, tx_offset_ns)) =
                        r.tx_evidence.get(j).copied().flatten()
                    else {
                        continue;
                    };
                    let base_mode = self.base_mode(j);
                    let reg_match = self.find_enable_readback_reply(
                        j,
                        rid,
                        after_tx_seq,
                        tx_at,
                        Some(r.deadline.at),
                        tx_offset_ns,
                    );
                    let matched_val = match reg_match {
                        crate::mit::CommissioningRegMatch::Pending => None,
                        crate::mit::CommissioningRegMatch::Ambiguous => {
                            return match r.stage {
                                ReadbackStage::CtrlMode => self.readback_failed_err(
                                    j,
                                    ArmDriverError::CtrlModeReadbackAmbiguous { joint: j },
                                ),
                                ReadbackStage::Timeout => {
                                    self.enable_failed();
                                    Err(ArmDriverError::TimeoutReadbackAmbiguous { joint: j })
                                }
                            };
                        }
                        crate::mit::CommissioningRegMatch::Matched(rep) => {
                            Some(u32::from_le_bytes(rep.value))
                        }
                    };
                    match r.stage {
                        ReadbackStage::CtrlMode => {
                            let want = Self::mode_value(base_mode);
                            match matched_val {
                                Some(v) if v == want => {
                                    if self.verified_modes[j] != Some(base_mode) && j < 32 {
                                        self.attempt_verified_modes_mask |= 1u32 << j;
                                    }
                                    self.verified_modes[j] = Some(base_mode);
                                    self.joint_modes[j] = Some(base_mode);
                                    self.mode_readback_boundary[j] = None;
                                    if j < 32 {
                                        self.mode_write_uncertain_mask &= !(1u32 << j);
                                    }
                                    #[cfg(not(target_arch = "wasm32"))]
                                    self.stage_damiao_mode_verified(j, base_mode);
                                    r.remaining_mask &= !(1u32 << j);
                                }
                                Some(v) => {
                                    return self.readback_failed_mismatch(j, want, v);
                                }
                                None => {}
                            }
                        }
                        ReadbackStage::Timeout => {
                            if let Some(observed) = matched_val {
                                #[cfg(not(target_arch = "wasm32"))]
                                if self.shadow_staging.is_enabled() && j < crate::torque::MAX_CHANNELS {
                                    let now = self.now();
                                    self.shadow_staging.push(
                                        crate::torque::shadow::ShadowDriverObs::TimeoutReadback {
                                            channel: j as u8,
                                            raw_reg: observed,
                                            now,
                                        },
                                    );
                                }
                                if let Some(expected) = self.expected_timeout_counts.get(j).copied().flatten() {
                                    if observed != expected {
                                        self.enable_failed();
                                        return Err(ArmDriverError::TimeoutReadbackMismatch {
                                            joint: j,
                                            expected,
                                            observed,
                                        });
                                    }
                                }
                                self.timeout_verified[j] = true;
                                r.remaining_mask &= !(1u32 << j);
                            }
                        }
                    }
                }
                if r.remaining_mask == 0 {
                    if r.stage == ReadbackStage::CtrlMode && r.timeout_pending_mask != 0 {
                        self.check_enable_tx_authority()?;
                        let now = self.now();
                        let mut dl_at = now + REGISTER_REPLY_TIMEOUT;
                        if let Some(auth_dl) = self.enable_tx_deadline {
                            dl_at = dl_at.min(auth_dl);
                        }
                        r = ModeReadback {
                            stage: ReadbackStage::Timeout,
                            remaining_mask: r.timeout_pending_mask,
                            sent_mask: 0,
                            timeout_pending_mask: 0,
                            log_cleared: false,
                            tx_evidence: [None; MAX_MODE_JOINTS],
                            deadline: Deadline { at: dl_at },
                        };
                        continue;
                    }
                    if let Some(pf_j) = self.pos_force.as_ref().map(|p| p.joint) {
                        if self.last_read_drained
                            && self.fresh_measured_position(pf_j, POS_FORCE_READBACK_FRESH_MARGIN).is_some()
                        {
                            return self.finish_pos_force_readback(pf_j, None);
                        }
                        let drained_now = !expired_before_read && self.last_read_drained;
                        let refresh = match self.start_feedback_refresh(pf_j, drained_now) {
                            Ok(ref_state) => ref_state,
                            Err(e) if is_estop_inhibited(&e) => return Err(e),
                            Err(ArmDriverError::Transport(t)) => {
                                return self.readback_failed_err(
                                    pf_j,
                                    ArmDriverError::PosForcePreEnableRefreshTxFailed {
                                        joint: pf_j,
                                        kind: TxFailKind::from_transport(&t),
                                    },
                                );
                            }
                            Err(e) => return self.readback_failed_err(pf_j, e),
                        };
                        let wake = self.now().min(refresh.deadline.at);
                        self.torque_hs = Some(DamiaoTorque::PreEnableRefresh(refresh));
                        return Ok(TorqueProgress::Pending { wake });
                    }
                    return self.send_enable();
                }
            }
            if r.deadline.passed(self.now()) {
                let j = r.remaining_mask.trailing_zeros() as usize;
                return match r.stage {
                    ReadbackStage::CtrlMode => self.readback_failed_err(
                        j,
                        ArmDriverError::CtrlModeReadbackMissing {
                            joint: j,
                            timeout_ms: REGISTER_REPLY_TIMEOUT.as_millis() as u64,
                        },
                    ),
                    ReadbackStage::Timeout => {
                        self.enable_failed();
                        Err(ArmDriverError::TimeoutReadbackMissing {
                            joint: j,
                            timeout_ms: REGISTER_REPLY_TIMEOUT.as_millis() as u64,
                        })
                    }
                };
            }
            let wake = if !r.log_cleared || (r.remaining_mask & !r.sent_mask) != 0 {
                self.now().min(r.deadline.at)
            } else {
                r.deadline.at
            };
            self.torque_hs = Some(DamiaoTorque::Readback(r));
            return Ok(TorqueProgress::Pending { wake });
        }
    }

    pub(in crate::driver::can_servo) fn poll_pre_enable_refresh(&mut self, refresh: FeedbackRefresh) -> Result<TorqueProgress, ArmDriverError> {
        self.enable_estop_inhibited()?;
        let joint = refresh.joint;
        match self.poll_feedback_refresh(refresh) {
            Ok(RefreshStep::Ready(sample)) => self.finish_pos_force_readback(joint, Some(sample)),
            Ok(RefreshStep::Pending(r)) => {
                let wake = r.deadline.at;
                self.torque_hs = Some(DamiaoTorque::PreEnableRefresh(r));
                Ok(TorqueProgress::Pending { wake })
            }
            Ok(RefreshStep::TimedOut) => self.readback_failed_err(
                joint,
                ArmDriverError::PosForcePreEnableRefreshMissing { joint },
            ),
            Err(e) if is_estop_inhibited(&e) => Err(e),
            Err(ArmDriverError::Transport(t)) => self.readback_failed_err(
                joint,
                ArmDriverError::PosForcePreEnableRefreshTxFailed {
                    joint,
                    kind: TxFailKind::from_transport(&t),
                },
            ),
            Err(e) => self.readback_failed_err(joint, e),
        }
    }

    pub(in crate::driver::can_servo) fn finish_pos_force_readback(&mut self, joint: usize, refresh_sample: Option<RefreshSample>) -> Result<TorqueProgress, ArmDriverError> {
        self.enable_estop_inhibited()?;
        if let Some(sample) = refresh_sample {
            let sample_still = sample.velocity.is_finite() && sample.velocity.abs() <= 0.05;
            let sample_off = !self.is_enabled && sample.motor_status == 0;
            if !(sample_still && sample_off) {
                return self.readback_failed_err(
                    joint,
                    ArmDriverError::PosForcePreEnableHoldUnsafe {
                        joint,
                        still: sample_still,
                        off: sample_off,
                        status: sample.motor_status,
                    },
                );
            }
            let sample_fresh = sample.position.is_finite()
                && self.since(sample.rx_at) <= POS_FORCE_FEEDBACK_MAX_AGE
                && sample.queue_bound.is_some_and(|qb| self.since(qb) <= POS_FORCE_FEEDBACK_MAX_AGE);
            if !sample_fresh {
                return self.readback_failed_err(
                    joint,
                    ArmDriverError::PosForcePreEnableRefreshMissing { joint },
                );
            }
        }
        // Hold at the position of the newest fresh sample that passes the live freshness,
        // stationarity, and torque-off checks below (not an older first-timely sample).
        let Some(hold_pos) = self.fresh_measured_position(joint, POS_FORCE_FEEDBACK_MAX_AGE) else {
            return self.readback_failed_err(
                joint,
                ArmDriverError::PosForcePreEnableRefreshMissing { joint },
            );
        };
        let still = self.cached_states[joint].velocity.is_finite() && self.cached_states[joint].velocity.abs() <= 0.05;
        let status = self.motor_status[joint];
        let off = !self.is_enabled && status == 0;
        if !(still && off) {
            return self.readback_failed_err(
                joint,
                ArmDriverError::PosForcePreEnableHoldUnsafe {
                    joint,
                    still,
                    off,
                    status,
                },
            );
        }
        if !self.pos_force.as_ref().is_some_and(|pf| pf.joint == joint) {
            return self.readback_failed_err(
                joint,
                ArmDriverError::PosForcePreEnableHoldUnavailable { joint },
            );
        }
        // Build and emit the zero-velocity 0x307 pre-enable Hold frame through the private
        // `pos_force_emitter` narrow pre-enable API (`allow_freeze = false`: enable requires
        // provably fresh <= 100 ms stationary feedback; `vel_raw = 0`, configured `current_raw`).
        let hold = match pos_force_emitter::emit_pre_enable_hold(
            self,
            self.enable_stop_stamp,
        ) {
            Ok(frame) => frame,
            Err(TxRefusal::Deferred) => {
                return self.readback_failed_err(
                    joint,
                    ArmDriverError::PosForcePreEnableHoldBudgetExhausted { joint },
                );
            }
            Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                self.abort_enable_for_estop();
                return Err(ArmDriverError::EstopInhibited);
            }
            Err(TxRefusal::BuildRefused) => {
                return self.readback_failed_err(
                    joint,
                    ArmDriverError::PosForcePreEnableHoldUnavailable { joint },
                );
            }
            Err(TxRefusal::Transport(e)) => {
                return self.readback_failed_err(
                    joint,
                    ArmDriverError::PosForcePreEnableHoldTxFailed {
                        joint,
                        kind: TxFailKind::from_transport(&e),
                    },
                );
            }
            Err(r_err) => {
                return self.readback_failed_err(joint, Self::refusal_to_driver_err(r_err));
            }
        };
        self.enable_estop_inhibited()?;
        if let Err(e) = self.flush_can_and_commit() {
            return self.readback_failed_err(
                joint,
                ArmDriverError::PosForcePreEnableHoldTxFailed {
                    joint,
                    kind: TxFailKind::from_transport(&e),
                },
            );
        }
        self.record_sent_target(joint, hold, None);
        self.joint_modes[joint] = Some(DamiaoControlMode::PosForce);
        self.verified_modes[joint] = Some(DamiaoControlMode::PosForce);
        self.pos_force_verified = true;
        if joint < 32 {
            self.mode_write_uncertain_mask &= !(1u32 << joint);
        }
        #[cfg(not(target_arch = "wasm32"))]
        self.stage_damiao_mode_verified(joint, DamiaoControlMode::PosForce);
        tracing::info!("Damiao joint {} confirmed in position-force mode (CTRL_MODE 4); pre-enable hold sent at {:.4} rad", joint + 1, hold_pos);
        self.send_enable()
    }

    /// Transmit enable frames for joints in `mask` through [`Self::send`] (`TxKind::Handshake`),
    /// enforcing **deadline -> budget -> stop gate -> J7 freshness revalidation -> stamp -> transmit**
    /// on every frame and checking the stop gate before flush.
    ///
    /// Fair TX scheduling: when `prioritize_joint` is set in `mask` (e.g., a J7 enable carried
    /// across a previous release), it is attempted first so J7 cannot starve behind earlier joints
    /// across more than one release; remaining joints proceed round-robin from `*tx_cursor`.
    /// When `defer_stale_pf` is true and J7's feedback expires mid-batch while its CTRL_MODE is
    /// verified, J7's bit is left in the returned `mask` so the caller can run its bounded causal
    /// `0xCC` refresh slot instead of failing closed before trying a refresh.
    pub(in crate::driver::can_servo) fn send_enable_masked(
        &mut self,
        mask: u32,
        fail_on_error: bool,
        deadline: Option<Deadline>,
        tx_cursor: &mut u8,
        prioritize_joint: Option<usize>,
        defer_stale_pf: bool,
    ) -> Result<u32, ArmDriverError> {
        let res = self.send_enable_masked_inner(
            mask,
            fail_on_error,
            deadline,
            tx_cursor,
            prioritize_joint,
            defer_stale_pf,
        );
        if res.is_err() {
            self.discard_staged_ctrl_tx();
        }
        res
    }

    pub(in crate::driver::can_servo) fn send_enable_masked_inner(
        &mut self,
        mut mask: u32,
        fail_on_error: bool,
        deadline: Option<Deadline>,
        tx_cursor: &mut u8,
        prioritize_joint: Option<usize>,
        defer_stale_pf: bool,
    ) -> Result<u32, ArmDriverError> {
        let orig_mask = mask;
        self.discard_staged_ctrl_tx();
        self.enable_estop_inhibited()?;
        self.check_enable_tx_authority()?;
        let n = self.motor_ids.len().min(handshake::MAX_MASK_JOINTS);
        let mut order = [0u8; handshake::MAX_MASK_JOINTS];
        let mut order_len = 0usize;
        if let Some(prio) = prioritize_joint {
            if prio < n && (mask & (1u32 << prio)) != 0 {
                order[order_len] = prio as u8;
                order_len += 1;
            }
        }
        if n > 0 {
            let start = usize::from(*tx_cursor) % n;
            for offset in 0..n {
                let i = (start + offset) % n;
                if Some(i) == prioritize_joint {
                    continue;
                }
                if (mask & (1u32 << i)) != 0 {
                    order[order_len] = i as u8;
                    order_len += 1;
                }
            }
        }
        let mut sent_any = false;
        for &i_u8 in &order[..order_len] {
            let i = usize::from(i_u8);
            self.check_enable_tx_authority()?;
            let Some(&id) = self.motor_ids.get(i) else {
                mask &= !(1 << i);
                continue;
            };
            match self.send_plain(TxKind::Handshake, self.enable_stop_stamp, deadline, false, |d| {
                // L2: Never transmit `CMD_ENABLE` unless `verified_modes[i]` causally matches
                // `base_mode(i)` after this daemon started.
                if d.verified_modes.get(i).copied().flatten() != Some(d.base_mode(i))
                    || d.joint_modes.get(i).copied().flatten() != Some(d.base_mode(i))
                {
                    return None;
                }
                if !d.pf_enable_feedback_fresh(i) {
                    return None;
                }
                Some(damiao::build_enable_frame(id))
            }) {
                Ok(_) => {
                    mask &= !(1 << i);
                    sent_any = true;
                    self.enable_frames_sent_any = true;
                    if Some(i) != prioritize_joint && n > 0 {
                        *tx_cursor = ((i + 1) % n) as u8;
                    }
                }
                Err(TxRefusal::DeadlinePassed | TxRefusal::Deferred) => break,
                Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                    self.abort_enable_for_estop();
                    return Err(ArmDriverError::EstopInhibited);
                }
                Err(TxRefusal::BuildRefused | TxRefusal::PosForceFeedbackExpired(_)) => {
                    let mode_ok = self.verified_modes.get(i).copied().flatten() == Some(self.base_mode(i))
                        && self.joint_modes.get(i).copied().flatten() == Some(self.base_mode(i));
                    let status = self.motor_status.get(i).copied().unwrap_or(0);
                    let hw_fault = damiao_fault_name(status).is_some();
                    if mode_ok
                        && !hw_fault
                        && defer_stale_pf
                        && self.pos_force.as_ref().is_some_and(|pf| pf.joint == i)
                    {
                        continue;
                    }
                    self.torque_hs = None;
                    self.enable_failed();
                    if !mode_ok {
                        return Err(ArmDriverError::HardwareFault(format!(
                            "joint {} CTRL_MODE unverified before enable",
                            i + 1
                        )));
                    }
                    if hw_fault {
                        return Err(ArmDriverError::MotorHardwareFault {
                            joint: i,
                            motor_id: id,
                            status,
                        });
                    }
                    return Err(ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint: i });
                }
                Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => {
                    if fail_on_error {
                        self.enable_failed();
                        return Err(e.into());
                    }
                    tracing::warn!("Retry transmit enable frame backpressured for motor {}: {:?}", id, e);
                    break;
                }
                Err(TxRefusal::Transport(e)) => {
                    if fail_on_error {
                        self.enable_failed();
                        return Err(e.into());
                    }
                    tracing::warn!("Retry transmit enable frame failed for motor {}: {:?}", id, e);
                }
                Err(TxRefusal::MitFenceTripped(reason)) => {
                    self.torque_hs = None;
                    self.enable_failed();
                    return Err(ArmDriverError::MitFenceTripped(reason));
                }
                Err(r @ (TxRefusal::TorqueOffWouldDrop | TxRefusal::DisableWaitEvidence | TxRefusal::DisableRequestExpired)) => {
                    self.torque_hs = None;
                    self.enable_failed();
                    return Err(Self::refusal_to_driver_err(r));
                }
                Err(TxRefusal::LeaseFenced) => {
                    self.torque_hs = None;
                    self.enable_failed();
                    return Err(ArmDriverError::LeaseFenced);
                }
                Err(TxRefusal::EnableAuthorityExpired) => {
                    self.torque_hs = None;
                    self.enable_failed();
                    return Err(ArmDriverError::EnableAuthorityExpired);
                }
            }
        }
        if sent_any {
            if let Err(e) = self.enable_estop_inhibited() {
                self.discard_staged_ctrl_tx();
                return Err(e);
            }
            if let Err(e) = self.check_enable_tx_authority() {
                self.discard_staged_ctrl_tx();
                return Err(e);
            }
            if self.flush_cutoff_exceeded() {
                self.mark_tx_pass_backpressured();
                self.discard_staged_ctrl_tx();
                if fail_on_error {
                    self.enable_failed();
                    return Err(TransportError::Timeout.into());
                }
                tracing::warn!("Retry flush cutoff exceeded");
                return Ok(orig_mask);
            } else if let Err(e) = self.flush_can_and_commit() {
                if fail_on_error {
                    self.enable_failed();
                    return Err(e.into());
                }
                tracing::warn!("Retry flush failed: {:?}", e);
                return Ok(orig_mask);
            }
        }
        Ok(mask)
    }

    pub(in crate::driver::can_servo) fn enable_pending_wake(
        after: Instant,
        window_wake: Instant,
        actionable_unsent: u32,
        pf_refresh: &Option<FeedbackRefresh>,
        tx_release_deadline: Option<Instant>,
    ) -> Instant {
        let mut wake = window_wake;
        if let Some(ref r) = pf_refresh {
            wake = wake.min(r.deadline.at);
        }
        let needs_tx_now =
            actionable_unsent != 0 || pf_refresh.as_ref().is_some_and(|r| r.boundary.is_none());
        if needs_tx_now && tx_release_deadline.is_none_or(|cap| after < cap) {
            after
        } else {
            wake
        }
    }

    /// Enable frames to every motor (bounded by the per-tick TX budget, carrying any remaining
    /// motors into subsequent `poll_enable` ticks), then the confirmation window
    /// (DAMIAO_ENABLE_CONFIRM, bounded by time only).
    pub(in crate::driver::can_servo) fn send_enable(&mut self) -> Result<TorqueProgress, ArmDriverError> {
        self.enable_estop_inhibited()?;
        self.check_enable_tx_authority()?;
        let handshake_start = self.now();
        self.last_enable_command = Some(handshake_start);
        self.last_enable_command_rx_seq = Some(self.rx_seq);
        self.last_command_was_disable = false;

        let n = self.motor_ids.len().min(handshake::MAX_MASK_JOINTS);
        let all_mask = if n >= 32 { u32::MAX } else { (1u32 << n) - 1 };
        let initial_deadline = Deadline::after(handshake_start, DAMIAO_ENABLE_CONFIRM);
        let pf_joint = self.pos_force.as_ref().map(|p| p.joint).filter(|&j| j < n);
        let pf_bit = pf_joint.map_or(0u32, |j| 1u32 << j);
        let mut tx_cursor = 0u8;
        let mut pf_refresh: Option<FeedbackRefresh> = None;
        let mut pf_refresh_count = 0u8;

        self.ensure_enable_pf_refresh(
            pf_joint,
            all_mask,
            initial_deadline,
            &mut pf_refresh,
            &mut pf_refresh_count,
        )?;
        let send_mask = if pf_refresh.is_some() {
            all_mask & !pf_bit
        } else {
            all_mask
        };

        let defer_stale_pf = pf_refresh.is_none() && pf_refresh_count < MAX_ENABLE_PF_REFRESHES;
        let mut unsent_mask = self.send_enable_masked(
            send_mask,
            true,
            Some(initial_deadline),
            &mut tx_cursor,
            None,
            defer_stale_pf,
        )?;
        if pf_refresh.is_some() {
            unsent_mask |= pf_bit;
        } else {
            self.ensure_enable_pf_refresh(
                pf_joint,
                unsent_mask,
                initial_deadline,
                &mut pf_refresh,
                &mut pf_refresh_count,
            )?;
        }
        let j7_prioritize = (unsent_mask & pf_bit) != 0;
        let poll_start = self.now();
        let window = ConfirmWindow::new(
            TorqueDirection::Enable,
            self.motor_ids.len(),
            handshake_start,
            poll_start,
            DAMIAO_ENABLE_CONFIRM,
            &DAMIAO_ENABLE_RETRY_AT,
        );
        let held_j7 = if pf_refresh.is_some() { unsent_mask & pf_bit } else { 0 };
        let wake = Self::enable_pending_wake(
            poll_start,
            window.wake(),
            unsent_mask & !held_j7,
            &pf_refresh,
            self.tx_release_deadline,
        );
        self.torque_hs = Some(DamiaoTorque::Enable {
            window,
            unsent_mask,
            tx_cursor,
            j7_prioritize,
            pf_refresh,
            pf_refresh_count,
        });
        Ok(TorqueProgress::Pending { wake })
    }

    pub(in crate::driver::can_servo) fn finish_enable_confirmed(&mut self, started: Instant) -> TorqueProgress {
        tracing::info!("All Damiao motors confirmed enabled (state=1) in {:?}", self.since(started));
        self.is_enabling = false;
        self.is_enabled = true;
        self.pending_rearm = false;
        self.refresh_capture = None;
        self.disable_dispatched_mask = 0;
        self.disable_rest_established = false;
        self.disable_tx_deadline = None;
        self.disable_lease_gen = None;
        self.enable_stop_stamp = None;
        self.enable_tx_deadline = None;
        self.enable_lease_gen = None;
        self.enable_frames_sent_any = false;
        self.startup_register_check_pending = false;
        self.attempt_verified_modes_mask = 0;
        // confirmed enable with the base modes restored: a new MIT handover arms a new fence
        self.mit_fence = None;
        self.mit_fence_trip = None;
        self.mit_fence_acked = false;
        self.mit_ownership_lost = false;
        self.last_mit_recovery_rx_seq = None;
        self.mit_tracking_used = false;
        self.mit_fence_last_cmd.fill(None);
        self.slot_in_unchanged_hold = [false; MAX_MODE_JOINTS * 2];
        self.pf_next_intent_is_hold = false;
        TorqueProgress::Done(TorqueOutcome::Confirmed)
    }

    pub(in crate::driver::can_servo) fn enable_confirm_snapshot(&self, unsent_mask: u32) -> (u32, bool, Option<usize>) {
        const FRESHNESS_LIMIT: Duration = Duration::from_millis(250);
        let n = self.motor_ids.len().min(handshake::MAX_MASK_JOINTS);
        let mut confirmed_mask = 0u32;
        let mut all_post_fb = !self.motor_ids.is_empty() && !self.awaiting_rx_boundary && unsent_mask == 0;
        for i in 0..n {
            let post_fb = !self.awaiting_rx_boundary
                && self.motor_post_enable_feedback.get(i).copied().unwrap_or(false)
                && self
                    .last_feedback_instant
                    .get(i)
                    .copied()
                    .flatten()
                    .is_some_and(|t| self.since(t) <= FRESHNESS_LIMIT);
            if (unsent_mask & (1 << i)) == 0 && post_fb && self.motor_status.get(i).copied() == Some(1) {
                confirmed_mask |= 1 << i;
            }
            all_post_fb &= post_fb;
        }
        let fault = if all_post_fb {
            self.motor_status.iter().position(|&st| damiao_fault_name(st).is_some())
        } else {
            None
        };
        (confirmed_mask, all_post_fb, fault)
    }

    pub(in crate::driver::can_servo) fn poll_enable(
        &mut self,
        mut window: ConfirmWindow,
        mut unsent_mask: u32,
        mut tx_cursor: u8,
        mut j7_prioritize: bool,
        mut pf_refresh: Option<FeedbackRefresh>,
        mut pf_refresh_count: u8,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.enable_estop_inhibited()?;
        self.check_enable_tx_authority()?;
        let n = self.motor_ids.len().min(handshake::MAX_MASK_JOINTS);
        let pf_joint = self.pos_force.as_ref().map(|p| p.joint).filter(|&j| j < n);
        let pf_bit = pf_joint.map_or(0u32, |j| 1u32 << j);

        if window.expire(self.now()).is_some() {
            self.enable_failed();
            if let Some(pf_j) = pf_joint {
                if pf_refresh.is_some()
                    || ((unsent_mask & pf_bit) != 0 && !self.pf_enable_feedback_fresh(pf_j))
                {
                    return Err(ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint: pf_j });
                }
            }
            let feedback_valid = self.is_feedback_valid();
            let mut statuses = [0u8; crate::rt_core::mailbox::MAX_DOF];
            let dof = self.motor_status.len().min(crate::rt_core::mailbox::MAX_DOF);
            statuses[..dof].copy_from_slice(&self.motor_status[..dof]);
            return Err(ArmDriverError::EnableTimeout {
                feedback_valid,
                statuses,
                dof: dof as u8,
            });
        }

        let deadline = Deadline { at: window.deadline };

        // Single RX pass + bounded causal J7 0xCC refresh service.
        if let Some(r) = pf_refresh.take() {
            let r_joint = r.joint;
            if r.boundary.is_some() {
                match self.poll_feedback_refresh_opts(r, true) {
                    Ok(RefreshStep::Ready(sample)) => {
                        let pf_j = r_joint;
                        if damiao_fault_name(sample.motor_status).is_some() {
                            let motor_id = self.motor_ids[pf_j];
                            self.enable_failed();
                            return Err(ArmDriverError::MotorHardwareFault {
                                joint: pf_j,
                                motor_id,
                                status: sample.motor_status,
                            });
                        }
                        let state_ok = sample.motor_status == 0 || sample.motor_status == 1;
                        let vel_ok = if sample.motor_status == 0 {
                            sample.velocity.is_finite() && sample.velocity.abs() <= 0.05
                        } else {
                            sample.velocity.is_finite()
                        };
                        let sample_fresh = sample.position.is_finite()
                            && self.since(sample.rx_at) <= POS_FORCE_FEEDBACK_MAX_AGE
                            && sample
                                .queue_bound
                                .is_some_and(|qb| self.since(qb) <= POS_FORCE_FEEDBACK_MAX_AGE);
                        if !(state_ok && vel_ok && sample_fresh && self.pf_enable_feedback_fresh(pf_j)) {
                            self.enable_failed();
                            return Err(ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint: pf_j });
                        }
                        // Verified fresh reply accepted: `pf_refresh` remains `None` so J7
                        // revalidates and transmits CMD_ENABLE in THIS release if TX budget remains.
                    }
                    Ok(RefreshStep::Pending(r_next)) => {
                        pf_refresh = Some(r_next);
                    }
                    Ok(RefreshStep::TimedOut) => {
                        self.enable_failed();
                        return Err(ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint: r_joint });
                    }
                    Err(e) if is_estop_inhibited(&e) => return Err(e),
                    Err(e) => {
                        self.enable_failed();
                        return Err(e);
                    }
                }
                self.check_enable_tx_authority()?;
            } else {
                let _ = self.receive_feedback();
                self.enable_estop_inhibited()?;
                self.check_enable_tx_authority()?;
                let pf_j = r_joint;
                self.check_pf_enable_hardware_fault(Some(pf_j))?;
                if self.last_read_drained && self.pf_enable_feedback_fresh(pf_j) {
                    self.refresh_capture = None;
                } else {
                    match self.poll_feedback_refresh_opts(r, true) {
                        Ok(RefreshStep::Pending(r_next)) => {
                            pf_refresh = Some(r_next);
                        }
                        Ok(RefreshStep::Ready(_)) => unreachable!("boundary was None"),
                        Ok(RefreshStep::TimedOut) => {
                            self.enable_failed();
                            return Err(ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint: pf_j });
                        }
                        Err(e) if is_estop_inhibited(&e) => return Err(e),
                        Err(e) => {
                            self.enable_failed();
                            return Err(e);
                        }
                    }
                }
            }
        } else {
            let _ = self.receive_feedback();
            self.enable_estop_inhibited()?;
            self.check_enable_tx_authority()?;
            self.check_pf_enable_hardware_fault(pf_joint)?;
        }

        // Plan BEFORE urgent watchdog holds: if every motor is already confirmed enabled right
        // after RX (`unsent_mask == 0` and `pf_refresh.is_none()`), confirm immediately without
        // emitting redundant urgent holds so Step 5 sends the release's single normal control
        // batch (with end-of-release watchdog fallback if any motor's normal output is deferred
        // or absent). Do not consume retry windows here so retries preserve `fail_on_error = false`.
        if unsent_mask == 0 && pf_refresh.is_none() {
            let all_mask = if n >= 32 { u32::MAX } else { (1u32 << n) - 1 };
            let (confirmed_mask, all_post_fb, fault) = self.enable_confirm_snapshot(0);
            if fault.is_some() || (all_post_fb && (confirmed_mask & all_mask) == all_mask) {
                let now = self.now();
                match window.service(now, confirmed_mask, all_post_fb, fault) {
                    Service::Confirmed => return Ok(self.finish_enable_confirmed(window.started)),
                    Service::Fault { .. } => {
                        let e = self
                            .check_hardware_status()
                            .err()
                            .unwrap_or(ArmDriverError::MotorHardwareFault {
                                joint: 0,
                                motor_id: 0,
                                status: 0xFF,
                            });
                        self.enable_failed();
                        return Err(e);
                    }
                    Service::Retry { .. } | Service::Wait { .. } => {}
                }
            }
        }

        // Urgent watchdog refresh AFTER the RX pass (so `rx_deadline` is never consumed before
        // `receive_feedback`) and BEFORE enable/0xCC/retry TX so already-enabled arm joints
        // (>= 18 ms since last control frame, not yet serviced in this release) preempt new
        // enables / retries / 0xCC polls and never exceed 40 ms between control frames.
        if self.motor_energized_mask != 0 {
            let reserve_for_phase = unsent_mask != 0 || pf_refresh.is_some();
            self.service_urgent_enable_watchdog_holds(0, reserve_for_phase)?;
        }

        // If J7 is in `unsent_mask` and stale, reserve the bounded 0xCC refresh slot FIRST before
        // earlier joints consume TX budget. While ANY `pf_refresh` is active, J7 MUST stay held
        // out of `CMD_ENABLE` until `poll_feedback_refresh_opts` verifies a post-boundary reply
        // through an observed empty RX queue.
        self.ensure_enable_pf_refresh(
            pf_joint,
            unsent_mask,
            deadline,
            &mut pf_refresh,
            &mut pf_refresh_count,
        )?;

        if unsent_mask != 0 {
            let held_j7 = if pf_refresh.is_some() { unsent_mask & pf_bit } else { 0 };
            let tx_mask = unsent_mask & !held_j7;
            if tx_mask != 0 {
                let prio = pf_joint.filter(|_| j7_prioritize && (tx_mask & pf_bit) != 0);
                let defer_stale_pf =
                    pf_refresh.is_none() && pf_refresh_count < MAX_ENABLE_PF_REFRESHES;
                let rem = self.send_enable_masked(
                    tx_mask,
                    true,
                    Some(deadline),
                    &mut tx_cursor,
                    prio,
                    defer_stale_pf,
                )?;
                unsent_mask = rem | held_j7;
            }
            self.ensure_enable_pf_refresh(
                pf_joint,
                unsent_mask,
                deadline,
                &mut pf_refresh,
                &mut pf_refresh_count,
            )?;
        }

        let (confirmed_mask, all_post_fb, fault) = self.enable_confirm_snapshot(unsent_mask);
        let now = self.now();
        match window.service(now, confirmed_mask, all_post_fb, fault) {
            Service::Confirmed => Ok(self.finish_enable_confirmed(window.started)),
            Service::Fault { .. } => {
                let e = self
                    .check_hardware_status()
                    .err()
                    .unwrap_or(ArmDriverError::MotorHardwareFault {
                        joint: 0,
                        motor_id: 0,
                        status: 0xFF,
                    });
                self.enable_failed();
                Err(e)
            }
            Service::Retry { mask, wake } => {
                unsent_mask |= mask;
                self.ensure_enable_pf_refresh(
                    pf_joint,
                    unsent_mask,
                    deadline,
                    &mut pf_refresh,
                    &mut pf_refresh_count,
                )?;
                let held_j7 = if pf_refresh.is_some() { unsent_mask & pf_bit } else { 0 };
                let tx_mask = unsent_mask & !held_j7;
                if tx_mask != 0 {
                    let prio = pf_joint.filter(|_| j7_prioritize && (tx_mask & pf_bit) != 0);
                    let defer_stale_pf =
                        pf_refresh.is_none() && pf_refresh_count < MAX_ENABLE_PF_REFRESHES;
                    let rem = self.send_enable_masked(
                        tx_mask,
                        false,
                        Some(deadline),
                        &mut tx_cursor,
                        prio,
                        defer_stale_pf,
                    )?;
                    unsent_mask = rem | held_j7;
                }
                self.ensure_enable_pf_refresh(
                    pf_joint,
                    unsent_mask,
                    deadline,
                    &mut pf_refresh,
                    &mut pf_refresh_count,
                )?;
                j7_prioritize = (unsent_mask & pf_bit) != 0;
                let after = self.now();
                let final_held_j7 = if pf_refresh.is_some() { unsent_mask & pf_bit } else { 0 };
                let wake = Self::enable_pending_wake(
                    after,
                    wake,
                    unsent_mask & !final_held_j7,
                    &pf_refresh,
                    self.tx_release_deadline,
                );
                self.torque_hs = Some(DamiaoTorque::Enable {
                    window,
                    unsent_mask,
                    tx_cursor,
                    j7_prioritize,
                    pf_refresh,
                    pf_refresh_count,
                });
                Ok(TorqueProgress::Pending { wake })
            }
            Service::Wait { wake } => {
                j7_prioritize = (unsent_mask & pf_bit) != 0;
                let after = self.now();
                let final_held_j7 = if pf_refresh.is_some() { unsent_mask & pf_bit } else { 0 };
                let wake = Self::enable_pending_wake(
                    after,
                    wake,
                    unsent_mask & !final_held_j7,
                    &pf_refresh,
                    self.tx_release_deadline,
                );
                self.torque_hs = Some(DamiaoTorque::Enable {
                    window,
                    unsent_mask,
                    tx_cursor,
                    j7_prioritize,
                    pf_refresh,
                    pf_refresh_count,
                });
                Ok(TorqueProgress::Pending { wake })
            }
        }
    }

}
