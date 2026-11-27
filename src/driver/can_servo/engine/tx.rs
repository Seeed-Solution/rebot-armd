//! Damiao CAN send choke point, pending TX drain, `BoundedTxExt`, and bounded target TX.

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
    /// Single choke point for EVERY Damiao CAN transmit (including safety disable and
    /// panic-disable). Enforces all 6 invariants in fixed order immediately before
    /// `can.transmit`, without heap allocation:
    /// 1. Deadline check (if given): refuse once passed (includes drains and retries).
    /// 2. Tick TX budget check (`Disable` exempt): exhaustion returns `Deferred`, not failure.
    /// 3. Stop gate by `TxKind`.
    /// 4. Optional MIT deadline fence + build frame fresh at send time via `build`.
    /// 5. Post-build re-read of stop latch, pending gate, stop stamp, MIT fence, type-system
    ///    `PosForceSendToken` capability for `0x307` frames, submission freshness, deadline, and cutoff.
    /// 6. Account budget attempt, transmit, and record per-frame wall/CPU diagnostics.
    pub(in crate::driver::can_servo) fn send_with_lease(
        &mut self,
        kind: impl DamiaoTxKind,
        expected_stamp: Option<StopStamp>,
        expected_lease_gen: u64,
        deadline: Option<Deadline>,
        fenced: bool,
        build: impl FnOnce(&Self) -> Option<CanFrame>,
    ) -> Result<CanFrame, TxRefusal> {
        self.send_with_lease_and_bypass(
            kind,
            expected_stamp,
            expected_lease_gen,
            deadline,
            fenced,
            ContinuityBypass::None,
            build,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub(in crate::driver::can_servo) fn send_with_lease_and_bypass(
        &mut self,
        kind: impl DamiaoTxKind,
        expected_stamp: Option<StopStamp>,
        expected_lease_gen: u64,
        deadline: Option<Deadline>,
        fenced: bool,
        continuity_bypass: ContinuityBypass,
        build: impl FnOnce(&Self) -> Option<CanFrame>,
    ) -> Result<CanFrame, TxRefusal> {
        let prev = self.send_lease_gen;
        self.send_lease_gen = expected_lease_gen;
        let res = self.send_plain_with_bypass(
            kind,
            expected_stamp,
            deadline,
            fenced,
            continuity_bypass,
            build,
        );
        self.send_lease_gen = prev;
        res
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[inline]
    pub(in crate::driver::can_servo) fn stage_damiao_pretransport_refusal(
        &mut self,
        kind: TxKind,
        refusal: &TxRefusal,
        targeted_idx: Option<usize>,
    ) {
        if !self.shadow_staging.is_enabled()
            || matches!(
                refusal,
                TxRefusal::Transport(_) | TxRefusal::DisableWaitEvidence
            )
        {
            return;
        }
        let n = self.motor_ids.len().min(crate::torque::MAX_CHANNELS) as u8;
        let scoped_mask = match targeted_idx {
            Some(idx) if idx < crate::torque::MAX_CHANNELS => {
                crate::torque::ChannelMask::bit(idx as u8)
            }
            _ => crate::torque::ChannelMask::from_count(n),
        };
        let (wk, mask) = match kind {
            TxKind::DisableRequested { .. }
            | TxKind::MitRecoveryDisable { .. }
            | TxKind::ClientParkDisable { .. } => {
                (crate::torque::WriteKind::DisableRequested, scoped_mask)
            }
            TxKind::DisableProtective { .. } => {
                (crate::torque::WriteKind::DisableProtective, scoped_mask)
            }
            TxKind::RollbackHold(j) | TxKind::MeasuredHold(j) if j < crate::torque::MAX_CHANNELS => {
                (crate::torque::WriteKind::Hold, crate::torque::ChannelMask::bit(j as u8))
            }
            TxKind::Hold => (crate::torque::WriteKind::Hold, scoped_mask),
            TxKind::Target | TxKind::Recovery => {
                (crate::torque::WriteKind::Motion, scoped_mask)
            }
            _ => return,
        };
        if mask.is_empty() {
            return;
        }
        let outcome = match refusal {
            TxRefusal::Deferred => crate::torque::MotorWriteOutcome::Deferred,
            _ => crate::torque::MotorWriteOutcome::RejectedStale,
        };
        let mut outcomes = [crate::torque::MotorWriteOutcome::NotTargeted; crate::torque::MAX_CHANNELS];
        for c in mask.iter() {
            outcomes[c as usize] = outcome;
        }
        let now = self.now();
        self.shadow_staging.push_dedup_refusal(
            wk,
            mask,
            outcome,
            crate::torque::shadow::ShadowDriverObs::WriteResult {
                device: 0,
                kind: wk,
                write_done: now,
                now,
                target_mask: mask,
                outcomes,
                ambiguous_gap: false,
            },
        );
    }

    pub(in crate::driver) fn send_plain(
        &mut self,
        kind: impl DamiaoTxKind,
        expected_stamp: Option<StopStamp>,
        deadline: Option<Deadline>,
        fenced: bool,
        build: impl FnOnce(&Self) -> Option<CanFrame>,
    ) -> Result<CanFrame, TxRefusal> {
        self.send_plain_with_bypass(
            kind,
            expected_stamp,
            deadline,
            fenced,
            ContinuityBypass::None,
            build,
        )
    }

    pub(in crate::driver::can_servo) fn send_plain_with_bypass(
        &mut self,
        kind: impl DamiaoTxKind,
        expected_stamp: Option<StopStamp>,
        deadline: Option<Deadline>,
        fenced: bool,
        continuity_bypass: ContinuityBypass,
        build: impl FnOnce(&Self) -> Option<CanFrame>,
    ) -> Result<CanFrame, TxRefusal> {
        let mut revoked: Option<&'static str> = None;
        self.send(
            kind,
            expected_stamp,
            deadline,
            fenced,
            continuity_bypass,
            None,
            &mut revoked,
            build,
        )
        .map(|(f, _, _, _)| f)
    }

    pub(in crate::driver::can_servo) fn send_with_commissioning_gate(
        &mut self,
        kind: impl DamiaoTxKind,
        expected_stamp: Option<StopStamp>,
        deadline: Option<Deadline>,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
        build: impl FnOnce(&Self) -> Option<CanFrame>,
    ) -> Result<(CanFrame, u64, Instant, Option<i128>), Result<&'static str, TxRefusal>> {
        self.send_with_commissioning_gate_with_bypass(
            kind,
            expected_stamp,
            deadline,
            ContinuityBypass::None,
            pre_tx_gate,
            build,
        )
    }

    pub(in crate::driver::can_servo) fn send_with_commissioning_gate_with_bypass(
        &mut self,
        kind: impl DamiaoTxKind,
        expected_stamp: Option<StopStamp>,
        deadline: Option<Deadline>,
        continuity_bypass: ContinuityBypass,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
        build: impl FnOnce(&Self) -> Option<CanFrame>,
    ) -> Result<(CanFrame, u64, Instant, Option<i128>), Result<&'static str, TxRefusal>> {
        let mut revoked: Option<&'static str> = None;
        match self.send(
            kind,
            expected_stamp,
            deadline,
            false,
            continuity_bypass,
            Some(pre_tx_gate),
            &mut revoked,
            build,
        ) {
            Ok(ok) => Ok(ok),
            Err(r) => {
                if let Some(code) = revoked {
                    Err(Ok(code))
                } else {
                    Err(Err(r))
                }
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub(in crate::driver) fn send(
        &mut self,
        kind: impl DamiaoTxKind,
        expected_stamp: Option<StopStamp>,
        deadline: Option<Deadline>,
        fenced: bool,
        continuity_bypass: ContinuityBypass,
        mut pre_tx_gate: Option<&mut dyn FnMut() -> Result<(), &'static str>>,
        revoked_out: &mut Option<&'static str>,
        build: impl FnOnce(&Self) -> Option<CanFrame>,
    ) -> Result<(CanFrame, u64, Instant, Option<i128>), TxRefusal> {
        let tx_kind = kind.tx_kind();
        let effective_bypass = continuity_bypass.for_tx_kind(tx_kind);
        #[cfg(not(target_arch = "wasm32"))]
        let mut built_frame_for_shadow: Option<CanFrame> = None;
        let res = 'send_body: {
            let expected_lease_gen = self.send_lease_gen;
            let has_pos_force_token = kind.pos_force_token().is_some();
            let kind = tx_kind;
            // 1. Deadline check (inclusive: now >= deadline.at).
            if deadline.is_some_and(|d| d.passed(self.now())) {
                break 'send_body Err(TxRefusal::DeadlinePassed);
            }

            // 2. Tick TX budget check (DisableRequested / ClientParkDisable / DisableProtective / DisableProbe exempt from motion TX budget).
            let budgeted = !Self::is_disable_tx_kind(kind);
            if budgeted {
                let _ = self.ensure_tx_pass();
                let prebuild_ok = if matches!(kind, TxKind::Target)
                    && !(self.mit_expiry_recovery_active || self.mit_fence_trip.is_some())
                {
                    self.can_start_tick_tx_n_with_bypass(1, effective_bypass)
                } else {
                    self.can_start_tick_tx_for_kind(kind, fenced, effective_bypass, None, false)
                };
                if !prebuild_ok {
                    break 'send_body Err(TxRefusal::Deferred);
                }
            }

            // 3. Stop gate and lease fence by TxKind (stop/hold-class and Disable are lease-exempt).
            let latched_before = match self.stop_gate_for_kind(kind) {
                Ok(v) => v,
                Err(e) => break 'send_body Err(e),
            };
            if matches!(kind, TxKind::Target) && self.is_lease_fenced(expected_lease_gen) {
                break 'send_body Err(TxRefusal::LeaseFenced);
            }

            // 4. MIT deadline fence (when fenced) + build frame fresh at send time.
            if fenced {
                if let Err(ArmDriverError::MitFenceTripped(reason)) = self.fence_check() {
                    break 'send_body Err(TxRefusal::MitFenceTripped(reason));
                }
            }
            if let TxKind::RollbackHold(joint) = kind {
                if latched_before
                    && self.fresh_measured_position(joint, POS_FORCE_FEEDBACK_MAX_AGE).is_none()
                    && self.active_pos_force_joint() != Some(joint)
                {
                    break 'send_body Err(TxRefusal::BuildRefused);
                }
            }
            let Some(frame) = build(self) else {
                if let TxKind::RollbackHold(joint) = kind {
                    if self.active_pos_force_joint() == Some(joint)
                        && !self.pos_force_feedback_valid(joint, self.now())
                    {
                        break 'send_body Err(TxRefusal::PosForceFeedbackExpired(joint));
                    }
                }
                break 'send_body Err(TxRefusal::BuildRefused);
            };
            #[cfg(not(target_arch = "wasm32"))]
            {
                built_frame_for_shadow = Some(frame);
            }

            // 5. Re-read physical stop latch, pending gate, stop-generation stamp, lease fence,
            //    MIT fence, position-force submission freshness, deadline, and tick TX cutoff AFTER
            //    `build`, immediately before `can.transmit` (host submission boundary).
            let latched_after = match self.stop_gate_for_kind(kind) {
                Ok(v) => v,
                Err(e) => break 'send_body Err(e),
            };
            if matches!(kind, TxKind::RollbackHold(_) | TxKind::MeasuredHold(_))
                && !latched_before
                && latched_after
            {
                break 'send_body Err(TxRefusal::StopInhibited);
            }
            let skip_stamp_check = matches!(
                kind,
                TxKind::DisableRequested { .. }
                    | TxKind::MitRecoveryDisable { .. }
                    | TxKind::DisableProtective { .. }
                    | TxKind::DisableProbe
            );
            if !skip_stamp_check
                && expected_stamp.is_some_and(|stamp| self.current_stop_stamp() != stamp)
            {
                break 'send_body Err(TxRefusal::StopStampChanged);
            }
            if matches!(kind, TxKind::Target) && self.is_lease_fenced(expected_lease_gen) {
                break 'send_body Err(TxRefusal::LeaseFenced);
            }
            if fenced {
                if let Err(ArmDriverError::MitFenceTripped(reason)) = self.fence_check() {
                    break 'send_body Err(TxRefusal::MitFenceTripped(reason));
                }
            }
            let submit_now = self.now();
            let is_cmd_enable = !frame.is_extended
                && !frame.is_rtr
                && frame.dlc == 8
                && frame.data == damiao::CMD_ENABLE;
            let is_cmd_disable = !frame.is_extended
                && !frame.is_rtr
                && frame.dlc == 8
                && frame.data == damiao::CMD_DISABLE;
            if let Some(pf_joint) = self.frame_pos_force_joint(&frame) {
                // Type-system capability guard: NO caller outside private `mod pos_force_emitter`
                // can construct a `PosForceSendToken`, so any forged `0x307` frame is rejected here.
                if !has_pos_force_token {
                    break 'send_body Err(TxRefusal::BuildRefused);
                }
                if !self.pos_force_feedback_valid(pf_joint, submit_now)
                    && !self.is_pos_force_freeze_frame(&frame)
                {
                    break 'send_body Err(TxRefusal::PosForceFeedbackExpired(pf_joint));
                }
            }
            // Terminal enable-phase authority check (`CMD_ENABLE` and `TxKind::Handshake` such as
            // `0xCC` refreshes) MUST run BEFORE per-release `DeadlinePassed` so revoked/expired
            // authority never degrades into a retryable budget deferral.
            if is_cmd_enable || matches!(kind, TxKind::Handshake) {
                if self.enable_lease_gen.is_some_and(|g| self.is_lease_fenced(g)) {
                    break 'send_body Err(TxRefusal::LeaseFenced);
                }
                if self.enable_tx_deadline.is_some_and(|dl| submit_now >= dl) {
                    break 'send_body Err(TxRefusal::EnableAuthorityExpired);
                }
            }
            if deadline.is_some_and(|d| d.passed(submit_now)) {
                break 'send_body Err(TxRefusal::DeadlinePassed);
            }
            let frame_axis = self.damiao_command_motor_index(&frame);
            let is_pf_freeze = self.is_pos_force_freeze_frame(&frame);
            if budgeted
                && !self.can_start_tick_tx_for_kind(
                    kind,
                    fenced,
                    effective_bypass,
                    frame_axis,
                    is_pf_freeze,
                )
            {
                break 'send_body Err(TxRefusal::Deferred);
            }
            if let Some(gate) = pre_tx_gate.as_mut() {
                if let Err(code) = (gate)() {
                    *revoked_out = Some(code);
                    break 'send_body Err(TxRefusal::LeaseFenced);
                }
            }

            // 6. Precompute indices and CPU timestamp, reset non-enable boundaries, then run the
            //    final observable stop, authority, and cutoff checks immediately before
            //    `record_tx_pass_attempt()` + `can.transmit` so pre-transmit refusals charge 0
            //    attempts and every actual transport attempt (Ok / WouldBlock / Err) charges 1.
            let enable_motor_idx = if is_cmd_enable {
                self.motor_ids.iter().position(|&id| u32::from(id) == frame.id)
            } else {
                None
            };
            let disable_motor_idx = if is_cmd_disable {
                self.motor_ids.iter().position(|&id| u32::from(id) == frame.id)
            } else {
                None
            };
            let mut motion_idx: Option<usize> = None;
            if !frame.is_extended
                && !frame.is_rtr
                && frame.dlc == 8
                && !is_cmd_enable
                && !is_cmd_disable
            {
                if frame.id == 0x7FF
                    && frame.data[2] == damiao::REG_WRITE
                    && frame.data[3] == damiao::RID_CTRL_MODE
                {
                    let target_id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
                    if let Some(idx) = self.motor_ids.iter().position(|&id| id == target_id) {
                        self.mode_readback_boundary[idx] = None;
                    }
                } else if matches!(kind, TxKind::Target | TxKind::Recovery) {
                    if let Some(idx) = self.damiao_command_motor_index(&frame) {
                        self.rest_pose_window[idx].reset();
                        motion_idx = Some(idx);
                    }
                }
            }
            let cpu0 = crate::diagnostics::tick::thread_cpu_ns();
            let t0 = self.now();
            if is_cmd_enable || matches!(kind, TxKind::Handshake) {
                if self.enable_tx_deadline.is_some_and(|dl| t0 >= dl) {
                    break 'send_body Err(TxRefusal::EnableAuthorityExpired);
                }
                if self.enable_lease_gen.is_some_and(|g| self.is_lease_fenced(g)) {
                    break 'send_body Err(TxRefusal::LeaseFenced);
                }
            }
            if is_cmd_enable {
                if deadline.is_some_and(|d| d.passed(t0)) {
                    break 'send_body Err(TxRefusal::DeadlinePassed);
                }
                if let Some(idx) = enable_motor_idx {
                    if self.pos_force.as_ref().is_some_and(|pf| pf.joint == idx)
                        && !self.pf_enable_feedback_fresh_at(idx, t0)
                    {
                        break 'send_body Err(TxRefusal::PosForceFeedbackExpired(idx));
                    }
                }
            }
            if is_cmd_disable && deadline.is_some_and(|d| d.passed(t0)) {
                break 'send_body Err(TxRefusal::DeadlinePassed);
            }
            // B1: Sample clock offset, run optional test hook, and run terminal stop/authority
            // checks BEFORE capturing `(tx_seq_mark, tx_started_mark)` immediately adjacent to
            // `can.transmit(&frame)` with ZERO intervening transport callbacks or hooks so no
            // pre-TX frame arriving during clock sampling/hooks/checks can satisfy
            // `rx_at >= tx_started_mark`.
            let (_, tx_offset_ns_mark) = self.capture_rx_clock_sample(t0);
            if let Some(ref hook) = self.before_final_tx_gate_hook {
                hook(&frame);
            }
            let latched_final = match self.stop_gate_for_kind(kind) {
                Ok(v) => v,
                Err(e) => break 'send_body Err(e),
            };
            if matches!(kind, TxKind::RollbackHold(_) | TxKind::MeasuredHold(_))
                && !latched_before
                && latched_final
            {
                break 'send_body Err(TxRefusal::StopInhibited);
            }
            if !skip_stamp_check
                && expected_stamp.is_some_and(|stamp| self.current_stop_stamp() != stamp)
            {
                break 'send_body Err(TxRefusal::StopStampChanged);
            }
            if let Some(gate) = pre_tx_gate.as_mut() {
                if let Err(code) = (gate)() {
                    *revoked_out = Some(code);
                    break 'send_body Err(TxRefusal::LeaseFenced);
                }
            }
            let tx_seq_mark = self.rx_seq;
            let tx_started_mark = self.now();
            self.last_tx_mark = None;
            if deadline.is_some_and(|d| d.passed(tx_started_mark))
                || (budgeted
                    && !effective_bypass.bypasses_release_deadline()
                    && (self.tick_tx.is_some_and(|p| tx_started_mark >= p.deadline)
                        || self.tx_release_deadline.is_some_and(|cap| tx_started_mark >= cap)))
            {
                break 'send_body Err(TxRefusal::DeadlinePassed);
            }
            if budgeted {
                self.record_tx_pass_attempt();
            }
            let res = self.can.transmit(&frame);
            if res.is_ok() {
                self.last_tx_mark = Some((tx_seq_mark, tx_started_mark));
                if let Some(idx) = enable_motor_idx {
                    self.motor_post_enable_feedback[idx] = false;
                    self.motor_enable_seq[idx] = self.rx_seq;
                    self.motor_last_enable_tx[idx] = Some((tx_seq_mark, tx_started_mark));
                    self.motor_disarmed_after_enable[idx] = false;
                    self.panic_disable_active = false;
                    self.panic_disable_tx_boundary[idx] = None;
                    self.panic_disable_confirmed[idx] = false;
                    self.mode_readback_boundary[idx] = None;
                    if idx < MAX_MODE_JOINTS {
                        let bit = 1u32 << idx;
                        if (self.motor_energized_mask & bit) == 0 {
                            self.motor_energized_mask |= bit;
                            self.record_ctrl_tx_delivered(idx, tx_started_mark);
                        }
                        if self.cached_states.get(idx).is_some_and(|s| s.position.is_finite()) {
                            self.motor_keepalive_hold_pos[idx] = Some(self.cached_states[idx].position);
                        }
                    }
                } else if let Some(idx) = disable_motor_idx {
                    self.motor_post_enable_feedback[idx] = false;
                } else if let Some(idx) = self.damiao_command_motor_index(&frame) {
                    self.record_ctrl_tx_delivered(idx, tx_started_mark);
                    if let Some(wire_q) = self.extract_wire_target_pos(idx, &frame) {
                        self.record_wire_pos_delivered(idx, tx_started_mark, wire_q);
                    }
                }
                let frame_kind = if is_cmd_enable || is_cmd_disable {
                    crate::diagnostics::tick::TxFrameKind::EnableDisable
                } else if (frame.id == 0x7FF && frame.dlc >= 3 && frame.data[2] == 0xCC)
                    || matches!(kind, TxKind::Refresh | TxKind::DisableProbe)
                {
                    crate::diagnostics::tick::TxFrameKind::Refresh0xCc
                } else {
                    match kind {
                        TxKind::Target => crate::diagnostics::tick::TxFrameKind::Normal,
                        TxKind::Recovery => crate::diagnostics::tick::TxFrameKind::Recovery,
                        TxKind::MeasuredHold(_) if self.in_urgent_watchdog_hold => {
                            crate::diagnostics::tick::TxFrameKind::UrgentHold
                        }
                        TxKind::Hold | TxKind::RollbackHold(_) | TxKind::MeasuredHold(_) => {
                            crate::diagnostics::tick::TxFrameKind::Hold
                        }
                        TxKind::Handshake
                        | TxKind::ModeWrite
                        | TxKind::ClearFault
                        | TxKind::DisableRequested { .. }
                        | TxKind::MitRecoveryDisable { .. }
                        | TxKind::ClientParkDisable { .. }
                        | TxKind::DisableProtective { .. } => {
                            crate::diagnostics::tick::TxFrameKind::EnableDisable
                        }
                        TxKind::Refresh | TxKind::DisableProbe => {
                            crate::diagnostics::tick::TxFrameKind::Refresh0xCc
                        }
                    }
                };
                crate::diagnostics::tick::record_tx_frame_kind(frame_kind);
            } else if let Err(ref e) = res {
                // Non-backpressure transport error on CMD_ENABLE has ambiguous wire delivery:
                // invalidate any cached disarm proof on that axis fail-closed.
                if !e.is_tx_backpressure() {
                    if let Some(idx) = enable_motor_idx {
                        self.motor_last_enable_tx[idx] = Some((tx_seq_mark, tx_started_mark));
                        self.motor_disarmed_after_enable[idx] = false;
                        self.panic_disable_active = false;
                        self.panic_disable_tx_boundary[idx] = None;
                        self.panic_disable_confirmed[idx] = false;
                        if idx < MAX_MODE_JOINTS {
                            let bit = 1u32 << idx;
                            // Fail-closed: record possible energization, but do NOT advance
                            // `motor_last_ctrl_tx` on ambiguous hard-error delivery so a due
                            // watchdog hold is never postponed.
                            self.motor_energized_mask |= bit;
                        }
                    }
                }
            }
            let write_done = self.now();
            let write_done_clock = if motion_idx.is_some() && res.is_ok() {
                Some(self.capture_rx_clock_sample(write_done))
            } else {
                None
            };
            let wall_ns = write_done.saturating_duration_since(t0).as_nanos().min(u128::from(u64::MAX)) as u64;
            let cpu_ns = crate::diagnostics::tick::thread_cpu_ns().saturating_sub(cpu0);
            let attempt_kind = match &res {
                Ok(()) => crate::diagnostics::tick::TxAttemptKind::Sent,
                Err(TransportError::WouldBlock) => crate::diagnostics::tick::TxAttemptKind::WouldBlock,
                Err(TransportError::NoBufferSpace) => crate::diagnostics::tick::TxAttemptKind::Enobufs,
                Err(_) => crate::diagnostics::tick::TxAttemptKind::Error,
            };
            crate::diagnostics::tick::record_tx_attempt(attempt_kind, wall_ns, cpu_ns);
            let final_res = match res {
                Ok(()) => {
                    // Record the same-clock boundary + CLOCK_REALTIME - CLOCK_MONOTONIC offset AFTER
                    // `transmit()` returns for the rest-pose gate (after the last motion command).
                    if let (Some(idx), Some((write_done_rx_clock, write_done_offset_ns))) =
                        (motion_idx, write_done_clock)
                    {
                        self.rest_pose_boundary[idx] = Some(RestPoseBoundary {
                            write_done,
                            write_done_rx_clock: Some(write_done_rx_clock),
                            write_done_offset_ns,
                        });
                    }
                    Ok((frame, tx_seq_mark, tx_started_mark, tx_offset_ns_mark))
                }
                Err(e) => {
                    self.tx_probe = None;
                    if budgeted && e.is_tx_backpressure() {
                        self.mark_tx_pass_backpressured();
                    }
                    Err(TxRefusal::Transport(e))
                }
            };
            // Shadow staging runs strictly AFTER live RX clock capture and live TX bookkeeping so
            // the live Damiao 20 ms rest-settle boundary is never perturbed by shadow work.
            #[cfg(not(target_arch = "wasm32"))]
            if self.shadow_staging.is_enabled() {
                let shadow_info = match kind {
                    TxKind::DisableRequested { .. }
                    | TxKind::MitRecoveryDisable { .. }
                    | TxKind::ClientParkDisable { .. } => self
                        .motor_ids
                        .iter()
                        .position(|&id| u32::from(id) == frame.id)
                        .map(|idx| (crate::torque::WriteKind::DisableRequested, idx)),
                    TxKind::DisableProtective { .. } => self
                        .motor_ids
                        .iter()
                        .position(|&id| u32::from(id) == frame.id)
                        .map(|idx| (crate::torque::WriteKind::DisableProtective, idx)),
                    TxKind::Handshake if frame.data == damiao::CMD_ENABLE => self
                        .motor_ids
                        .iter()
                        .position(|&id| u32::from(id) == frame.id)
                        .map(|idx| (crate::torque::WriteKind::Enable, idx)),
                    TxKind::Target | TxKind::Recovery => {
                        motion_idx.map(|idx| (crate::torque::WriteKind::Motion, idx))
                    }
                    TxKind::RollbackHold(j) | TxKind::MeasuredHold(j) => {
                        Some((crate::torque::WriteKind::Hold, j))
                    }
                    TxKind::Hold => self
                        .damiao_command_motor_index(&frame)
                        .map(|idx| (crate::torque::WriteKind::Hold, idx)),
                    _ if !frame.is_extended
                        && !frame.is_rtr
                        && frame.dlc == 8
                        && frame.id == 0x7FF
                        && frame.data[2] == damiao::REG_WRITE
                        && frame.data[3] == damiao::RID_CTRL_MODE =>
                    {
                        let target_id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
                        self.motor_ids
                            .iter()
                            .position(|&id| id == target_id)
                            .map(|idx| (crate::torque::WriteKind::ModeSwitch, idx))
                    }
                    _ => None,
                };
                if let Some((wk, idx)) = shadow_info {
                    if idx < crate::torque::MAX_CHANNELS {
                        let outcome = match &final_res {
                            Ok(_) => crate::torque::MotorWriteOutcome::Sent,
                            Err(TxRefusal::Transport(
                                TransportError::WouldBlock | TransportError::NoBufferSpace,
                            )) => crate::torque::MotorWriteOutcome::Deferred,
                            Err(_) => crate::torque::MotorWriteOutcome::Failed,
                        };
                        if self.defer_disable_shadow
                            && matches!(
                                wk,
                                crate::torque::WriteKind::DisableRequested
                                    | crate::torque::WriteKind::DisableProtective
                            )
                        {
                            if outcome == crate::torque::MotorWriteOutcome::Sent && idx < 64 {
                                self.shadow_disable_pass_sent_mask |= 1u64 << idx;
                            }
                            // When a multi-tick requested disable already dispatched >= 1 motor in
                            // this pass and hits TX backpressure on the next motor, keep the paused
                            // motor unresolved in `pending_disable_dispatch.remaining_recipients`
                            // (matching budget-pause) so the next tick's Sent write reconciles it.
                            // Uses shadow-only pass progress (`shadow_disable_pass_sent_mask`) so
                            // Phase C repeats do not mistake prior-pass `disable_dispatched_mask`.
                            let paused_mid_multitick = matches!(
                                wk,
                                crate::torque::WriteKind::DisableRequested
                            ) && outcome == crate::torque::MotorWriteOutcome::Deferred
                                && (self.pending_disable_shadow.is_some()
                                    || self.shadow_disable_pass_sent_mask != 0);
                            if !paused_mid_multitick {
                                let mut outcomes = [crate::torque::MotorWriteOutcome::NotTargeted;
                                    crate::torque::MAX_CHANNELS];
                                outcomes[idx] = outcome;
                                PendingDisableShadowBatch::merge_batch(
                                    &mut self.pending_disable_shadow,
                                    PendingDisableShadowBatch {
                                        kind: wk,
                                        target_mask: crate::torque::ChannelMask::bit(idx as u8),
                                        outcomes,
                                        ambiguous_gap: false,
                                        write_done: Some(write_done),
                                    },
                                );
                            }
                        } else {
                            let mut outcomes = [crate::torque::MotorWriteOutcome::NotTargeted;
                                crate::torque::MAX_CHANNELS];
                            outcomes[idx] = outcome;
                            self.shadow_staging
                                .push(crate::torque::shadow::ShadowDriverObs::WriteResult {
                                    device: 0,
                                    kind: wk,
                                    write_done,
                                    now: write_done,
                                    target_mask: crate::torque::ChannelMask::bit(idx as u8),
                                    outcomes,
                                    ambiguous_gap: false,
                                });
                        }
                    }
                }
            }
            final_res
        };
        #[cfg(not(target_arch = "wasm32"))]
        if let Err(ref refusal) = res {
            if !self.defer_disable_shadow {
                let target_idx = match tx_kind {
                    TxKind::RollbackHold(j) | TxKind::MeasuredHold(j) => Some(j),
                    _ => built_frame_for_shadow.and_then(|f| {
                        self.motor_ids
                            .iter()
                            .position(|&id| u32::from(id) == f.id)
                            .or_else(|| self.damiao_command_motor_index(&f))
                    }),
                };
                self.stage_damiao_pretransport_refusal(tx_kind, refusal, target_idx);
            }
        }
        res
    }

    /// Off-tick retry wrapper over [`Self::send`]: retries transient backpressure up to 3 times
    /// 100 µs apart ONLY when `!rt_no_sleep && !fenced`, re-running ALL 6 steps of `send`
    /// (deadline, budget, stop gate, fresh `build`, stamp, transmit) on every attempt.
    pub(in crate::driver::can_servo) fn send_with_retry(
        &mut self,
        kind: TxKind,
        expected_stamp: Option<StopStamp>,
        deadline: Option<Deadline>,
        fenced: bool,
        build: impl FnMut(&Self) -> Option<CanFrame>,
    ) -> Result<CanFrame, TxRefusal> {
        self.send_with_retry_inner(kind, expected_stamp, deadline, fenced, None, build)
    }

    pub(in crate::driver::can_servo) fn send_with_retry_inner(
        &mut self,
        kind: TxKind,
        expected_stamp: Option<StopStamp>,
        deadline: Option<Deadline>,
        fenced: bool,
        pos_force_token: Option<&pos_force_emitter::PosForceSendToken>,
        mut build: impl FnMut(&Self) -> Option<CanFrame>,
    ) -> Result<CanFrame, TxRefusal> {
        let mut retries = 0u8;
        loop {
            let res = match pos_force_token {
                Some(tok) => self.send_plain(
                    pos_force_emitter::authorize_kind(tok, kind),
                    expected_stamp,
                    deadline,
                    fenced,
                    |d| build(d),
                ),
                None => self.send_plain(kind, expected_stamp, deadline, fenced, |d| build(d)),
            };
            match res {
                Ok(frame) => return Ok(frame),
                Err(TxRefusal::Transport(e))
                    if e.is_tx_backpressure() && !self.rt_no_sleep && !fenced && retries < 3 =>
                {
                    retries += 1;
                    // Allow off-tick retry attempts within the remaining max_frames / max_duration
                    // budget while keeping backpressure latched once retries exhaust.
                    if let Some(pass) = self.tick_tx.as_mut() {
                        pass.backpressured = false;
                    }
                    self.pause(Duration::from_micros(100));
                }
                Err(r) => return Err(r),
            }
        }
    }

    /// Drain queued arm (non-PosForce) motion frames within the active per-tick TX pass
    /// (zero heap allocation). Leaves [`Self::pos_force_reserved_slots`] available in the budget
    /// whenever J7 was deferred on the previous tick so J7 never starves under tight budgets.
    pub(in crate::driver::can_servo) fn drain_pending_pass(
        &mut self,
        pass_stop_stamp: StopStamp,
        pass_lease_gen: u64,
        pre_dropped_stale: usize,
        pre_oldest_expired: Option<Duration>,
        stop_on_expiry: bool,
    ) -> Result<TxBatchOutcome, ArmDriverError> {
        let res = self.drain_pending_pass_inner(
            pass_stop_stamp,
            pass_lease_gen,
            pre_dropped_stale,
            pre_oldest_expired,
            stop_on_expiry,
        );
        if res.is_err() {
            self.discard_staged_ctrl_tx();
        }
        res
    }

    pub(in crate::driver::can_servo) fn drain_pending_pass_inner(
        &mut self,
        pass_stop_stamp: StopStamp,
        pass_lease_gen: u64,
        pre_dropped_stale: usize,
        pre_oldest_expired: Option<Duration>,
        stop_on_expiry: bool,
    ) -> Result<TxBatchOutcome, ArmDriverError> {
        let _ = self.ensure_tx_pass();
        self.discard_staged_ctrl_tx();
        self.last_target_tx_sent_mask = 0;
        let mut sent_any = false;
        let mut sent_mask: u64 = 0;
        let mut sent_order = [0usize; MAX_MODE_JOINTS];
        let mut sent_order_len = 0usize;
        let mut dropped_stale_in_pass = 0usize;
        let mut oldest_expired = pre_oldest_expired;
        while let Some(entry) = self.pending_tx.front() {
            let rem = self.remaining_tick_tx_slots();
            if rem == 0 || !self.can_start_tick_tx() {
                break;
            }
            // If this is the last remaining TX slot of the tick, yield it to the J7 epilogue
            // whenever J7 outranks this Class-2 arm motion frame (e.g. J7 Class-1 freeze/stop,
            // or an older waiting Class-2 J7 target by `joint_last_tx_seq`).
            if rem == 1 {
                if let Some((j7_class, j7_seq)) = self.j7_tx_priority(false) {
                    let arm_seq = self.slot_last_tx_seq.get(entry.joint.saturating_mul(2)).copied().unwrap_or(0);
                    if (j7_class, j7_seq) < (TX_CLASS_MOTION_TARGET, arm_seq) {
                        break;
                    }
                }
            }
            let entry_deadline = Deadline::after(entry.queued_at, self.tx_budget.frame_max_age);
            match self.send_with_lease(
                TxKind::Target,
                Some(entry.stop_stamp),
                entry.lease_gen,
                Some(entry_deadline),
                false,
                |d| d.build_pending_target_frame(&entry),
            ) {
                Ok(sent_frame) => {
                    let _ = self.pending_tx.pop_front();
                    let mit_params = match entry.cmd {
                        PendingTargetCmd::Mit { kp, kd, torque, .. } => Some((
                            kp.filter(|k| k.is_finite()).unwrap_or(self.default_kp),
                            kd.filter(|k| k.is_finite()).unwrap_or(self.default_kd),
                            torque.filter(|t| t.is_finite()).unwrap_or(0.0),
                        )),
                        PendingTargetCmd::PosVel { .. }
                        | PendingTargetCmd::RollbackHold { .. }
                        | PendingTargetCmd::MeasuredHold { .. } => None,
                    };
                    self.record_sent_target(entry.joint, sent_frame, mit_params);
                    sent_any = true;
                    if sent_order_len < MAX_MODE_JOINTS {
                        sent_order[sent_order_len] = entry.joint;
                        sent_order_len += 1;
                    }
                    if entry.joint < 64 {
                        sent_mask |= 1u64 << entry.joint;
                    }
                }
                Err(TxRefusal::PosForceFeedbackExpired(_)) => {
                    let _ = self.pending_tx.pop_front();
                    continue;
                }
                Err(TxRefusal::DeadlinePassed) => {
                    let now = self.now();
                    let _ = self.pending_tx.pop_front();
                    let age = now.saturating_duration_since(entry.queued_at);
                    oldest_expired = Some(oldest_expired.map_or(age, |o| o.max(age)));
                    dropped_stale_in_pass += 1;
                    crate::diagnostics::tick::record_tx_dropped_stale(1);
                    if stop_on_expiry {
                        break;
                    }
                }
                Err(TxRefusal::Deferred) => break,
                Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                    self.discard_pending_tx_stopped();
                    return Err(ArmDriverError::EstopInhibited);
                }
                Err(TxRefusal::LeaseFenced) => {
                    let gate = self.estop_gate.as_ref();
                    let dropped = self
                        .pending_tx
                        .drop_fenced_lease(|g| gate.is_some_and(|gt| gt.is_lease_fenced(g)));
                    if dropped > 0 {
                        crate::diagnostics::tick::record_tx_dropped_stopped(dropped as u32);
                    }
                    crate::diagnostics::tick::record_tx_deferred(0, self.total_pending_tx_len());
                    return Err(ArmDriverError::LeaseFenced);
                }
                Err(TxRefusal::BuildRefused) => {
                    self.discard_pending_tx_stopped();
                    return Err(ArmDriverError::PosForceTargetUnavailable { joint: entry.joint });
                }
                Err(TxRefusal::MitFenceTripped(reason)) => {
                    self.discard_pending_tx_stopped();
                    return Err(ArmDriverError::MitFenceTripped(reason));
                }
                Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => {
                    if let Err(estop) = self.strict_stop_check(Some(pass_stop_stamp)) {
                        self.discard_pending_tx_stopped();
                        return Err(estop);
                    }
                    break;
                }
                Err(TxRefusal::Transport(e)) => {
                    self.mark_tx_pass_backpressured();
                    self.discard_pending_tx_stopped();
                    return Err(e.into());
                }
                Err(r @ (TxRefusal::TorqueOffWouldDrop | TxRefusal::DisableWaitEvidence | TxRefusal::DisableRequestExpired | TxRefusal::EnableAuthorityExpired)) => {
                    self.discard_pending_tx_stopped();
                    return Err(Self::refusal_to_driver_err(r));
                }
            }
        }
        // Pre-flush stop and lease-fence check: a stop/E-stop or disconnect arriving during the
        // final frame transmit skips flush and drops any remaining fenced entries.
        if let Err(e) = self.strict_stop_check(Some(pass_stop_stamp)) {
            self.discard_pending_tx_stopped();
            return Err(e);
        }
        if self.is_lease_fenced(pass_lease_gen) {
            let gate = self.estop_gate.as_ref();
            let dropped = self
                .pending_tx
                .drop_fenced_lease(|g| gate.is_some_and(|gt| gt.is_lease_fenced(g)));
            if dropped > 0 {
                crate::diagnostics::tick::record_tx_dropped_stopped(dropped as u32);
            }
            return Err(ArmDriverError::LeaseFenced);
        }
        if sent_any {
            if self.flush_cutoff_exceeded() {
                if self.tick_tx.is_some_and(|p| self.now() >= p.deadline) {
                    self.mark_tx_pass_backpressured();
                }
                self.discard_staged_ctrl_tx();
                self.discard_pending_tx_stopped();
                return Err(TransportError::Timeout.into());
            }
            if let Err(e) = self.flush_can_and_commit() {
                self.mark_tx_pass_backpressured();
                self.discard_pending_tx_stopped();
                return Err(e.into());
            }
        }
        // Post-flush stop & lease-fence check.
        if let Err(e) = self.strict_stop_check(Some(pass_stop_stamp)) {
            self.discard_pending_tx_stopped();
            return Err(e);
        }
        if self.is_lease_fenced(pass_lease_gen) {
            let gate = self.estop_gate.as_ref();
            let dropped = self
                .pending_tx
                .drop_fenced_lease(|g| gate.is_some_and(|gt| gt.is_lease_fenced(g)));
            if dropped > 0 {
                crate::diagnostics::tick::record_tx_dropped_stopped(dropped as u32);
            }
            return Err(ArmDriverError::LeaseFenced);
        }
        for &joint in &sent_order[..sent_order_len] {
            self.record_flushed_joint_tx(joint, 0, true);
        }
        // Record per-joint wire progress only after flush and post-flush stop checks succeed,
        // excluding any joint that still has an unsent frame remaining in `pending_tx`.
        let mut completed_mask = sent_mask;
        for entry in self.pending_tx.buf[..self.pending_tx.len].iter().flatten() {
            if entry.joint < 64 {
                completed_mask &= !(1u64 << entry.joint);
            }
        }
        self.last_target_tx_sent_mask = completed_mask;
        // Recheck freshness AFTER flush so frames expiring during send or flush never carry over.
        let post_now = self.now();
        let (late_stale, late_oldest) = self.pending_tx.drop_stale(post_now, self.tx_budget.frame_max_age);
        if late_stale > 0 {
            dropped_stale_in_pass += late_stale;
            if let Some(age) = late_oldest {
                oldest_expired = Some(oldest_expired.map_or(age, |o| o.max(age)));
            }
            crate::diagnostics::tick::record_tx_dropped_stale(late_stale as u32);
        }
        let total_dropped_stale = pre_dropped_stale.saturating_add(dropped_stale_in_pass);
        if stop_on_expiry && total_dropped_stale > 0 {
            self.discard_pending_tx_stopped();
            self.last_expired_oldest_age = oldest_expired;
            return Ok(TxBatchOutcome::Expired { dropped: total_dropped_stale, pending: 0 });
        }
        let newly_deferred = self.pending_tx.mark_newly_deferred();
        let pending = self.pending_tx.len();
        crate::diagnostics::tick::record_tx_deferred(newly_deferred as u32, self.total_pending_tx_len());
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
    }

    /// Drain carried-over pending target frames within the active tick's TX budget (no new targets staged).
    pub fn drain_pending_tx(&mut self) -> Result<TxBatchOutcome, ArmDriverError> {
        self.last_expired_oldest_age = None;
        self.last_tx_pf_status = PosForceTxStatus::Unobserved;
        if self.torque_hs.is_some() {
            self.reject_target_tx(MitFenceTripReason::PreflightRejected);
            return Err(ArmDriverError::TorqueTransitionPending);
        }
        if self.mit_fence.is_some() || self.mit_fence_trip.is_some() {
            self.reject_target_tx(MitFenceTripReason::PreflightRejected);
            return Err(ArmDriverError::PendingTxDisabledUnderMitFence);
        }
        if let Some(e) = self.target_frames_refused(None, true) {
            self.reject_target_tx(MitFenceTripReason::PreflightRejected);
            return Err(e);
        }
        if let Err(e) = self.strict_stop_check(None) {
            self.discard_pending_tx_stopped();
            return Err(e);
        }
        self.output_ran_since_rx = true;
        self.last_output_was_hold = false;
        let current_stamp = self.current_stop_stamp();
        if self.pending_tx.has_stopped_entry(current_stamp) {
            self.discard_pending_tx_stopped();
            return Err(ArmDriverError::EstopInhibited);
        }
        let fenced_dropped = {
            let gate = self.estop_gate.as_ref();
            self.pending_tx
                .drop_fenced_lease(|g| gate.is_some_and(|gt| gt.is_lease_fenced(g)))
        };
        if fenced_dropped > 0 {
            crate::diagnostics::tick::record_tx_dropped_stopped(fenced_dropped as u32);
        }
        let cleanup_now = self.now();
        let (stale, pre_oldest) = self.pending_tx.drop_stale(cleanup_now, self.tx_budget.frame_max_age);
        if stale > 0 {
            crate::diagnostics::tick::record_tx_dropped_stale(stale as u32);
        }
        let arm_out = self.drain_pending_pass(current_stamp, 0, stale, pre_oldest, false)?;
        if self.external_pos_force_emitter || !self.pf_deferred_motion {
            return Ok(arm_out);
        }
        let cur_intent_stamp = self.current_j7_intent_stamp();
        let intent = self.pf_deferred_intent.unwrap_or_else(|| PosForceIntent::hold(cur_intent_stamp));
        let hold_bypass = self.j7_hold_continuity_bypass();
        match pos_force_emitter::emit_tick(self, intent, cur_intent_stamp, hold_bypass)? {
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
                    TxBatchOutcome::Expired { dropped, pending } => TxBatchOutcome::Expired { dropped, pending: pending + 1 },
                })
            }
        }
    }

}

impl<C: CanTransport> BoundedTxExt for DamiaoArmDriver<C> {
    fn set_enable_authority(&mut self, deadline: Option<Instant>, expected_lease_gen: Option<u64>) {
        self.enable_tx_deadline = deadline;
        self.enable_lease_gen = expected_lease_gen;
    }

    fn set_disable_authority(&mut self, deadline: Option<Instant>, expected_lease_gen: Option<u64>) {
        self.disable_tx_deadline = deadline;
        self.disable_lease_gen = expected_lease_gen;
    }

    fn begin_wake_tx_pass(&mut self) {
        let now = self.now();
        if let Some(pass) = self.tick_tx.as_mut() {
            pass.deadline = now + self.tx_budget.max_duration;
            pass.backpressured = false;
        }
        self.discard_staged_ctrl_tx();
        self.output_step_pending_in_release = false;
    }

    fn reset_standalone_tx_pass(&mut self) {
        self.control_release_managed = false;
        self.begin_tx_tick();
    }

    fn tx_budget_max_frames(&self) -> Option<u16> {
        Some(self.tx_budget.max_frames.min(u16::MAX as usize) as u16)
    }

    fn check_watchdog_tx_budget_for_enable(&self) -> Result<(), (usize, usize)> {
        self.check_watchdog_tx_budget_for_enable_inner()
    }

    fn check_watchdog_tx_budget_for_energized(&self) -> Result<(), (usize, usize)> {
        self.check_watchdog_tx_budget_for_energized_inner()
    }

    fn release_budget_consumed(&self) -> (u16, u16) {
        let tx = self
            .tick_tx
            .map_or(0, |p| p.attempts.min(u16::MAX as usize) as u16);
        (self.release_rx_frames, tx)
    }

    fn sync_release_budget_limits(&mut self, rx_remaining: u16, tx_remaining: u16) {
        let new_rx_lim = self.release_rx_frames.saturating_add(rx_remaining);
        self.release_rx_limit = self.release_rx_limit.min(new_rx_lim);
        let cur_tx = self
            .tick_tx
            .map_or(0, |p| p.attempts.min(u16::MAX as usize) as u16);
        let new_tx_lim = cur_tx.saturating_add(tx_remaining);
        self.release_tx_limit = Some(
            self.release_tx_limit
                .map_or(new_tx_lim, |prev| prev.min(new_tx_lim)),
        );
    }

    fn next_watchdog_hold_wake(&self) -> Option<Instant> {
        DamiaoArmDriver::next_watchdog_hold_wake(self)
    }

    fn drop_pending_holds_outside_mask(&mut self, hold_mask: u64) {
        let dropped = self.pending_hold_tx.drop_outside_mask(hold_mask);
        if dropped > 0 {
            crate::diagnostics::tick::record_tx_dropped_stopped(dropped as u32);
            crate::diagnostics::tick::record_tx_deferred(0, self.total_pending_tx_len());
        }
    }

    fn pending_motion_tx_len(&self) -> usize {
        self.pending_tx.len() + usize::from(self.pf_deferred_motion)
    }

    fn pending_hold_tx_len(&self) -> usize {
        self.pending_hold_tx.len()
    }

    /// True only when a `0xCC` TX probe write & flush completed and fresh feedback for the probed
    /// motor arrived strictly after the post-write empty-queue RX sequence boundary
    /// (`last_feedback_seq[probe.joint] > probe.boundary_seq`). Because SocketCAN supplies no
    /// arrival timestamps (`CanTransport::receive_timestamped` returns `None`), this post-write
    /// RX sequence boundary is the proof that confirming feedback came after the probe write.
    fn tx_probe_confirmed(&self) -> bool {
        let Some(probe) = self.tx_probe else { return false };
        !self.awaiting_rx_boundary
            && self.total_pending_tx_len() == 0
            && probe.joint < self.last_feedback_seq.len()
            && self.last_feedback_seq[probe.joint] > probe.boundary_seq
            && self.last_feedback_instant[probe.joint]
                .is_some_and(|t| self.since(t) <= Duration::from_millis(250))
            && self.is_feedback_valid()
    }

    fn tx_probe_pending(&self) -> bool {
        let Some(probe) = self.tx_probe else { return false };
        !self.tx_probe_confirmed() && self.since(probe.sent_at) < REGISTER_REPLY_TIMEOUT
    }

    fn probe_tx_ready(&mut self) -> Result<bool, ArmDriverError> {
        self.probe_tx_ready_inner(None, true)
    }

    fn probe_tx_ready_budgeted(&mut self, deadline: Instant) -> Result<bool, ArmDriverError> {
        self.probe_tx_ready_inner(Some(Deadline { at: deadline }), false)
    }

    fn clear_tx_probe(&mut self) {
        self.tx_probe = None;
    }

    fn last_hold_tx_sent_any(&self) -> bool {
        self.last_hold_tx_sent_any
    }

    fn last_hold_tx_due_mask(&self) -> u64 {
        self.last_hold_tx_due_mask
    }

    fn last_hold_tx_sent_mask(&self) -> u64 {
        self.last_hold_tx_sent_mask
    }

    fn last_target_tx_sent_mask(&self) -> u64 {
        self.last_target_tx_sent_mask
    }

    fn discard_pending_tx(&mut self) {
        self.discard_all_pending_tx();
    }

    fn set_tx_budget(&mut self, budget: TxBudget) {
        self.tx_budget = budget;
    }

    fn tx_frame_max_age(&self) -> Duration {
        self.tx_budget.frame_max_age
    }

    fn last_tx_expired_age(&self) -> Option<Duration> {
        self.last_expired_oldest_age
    }

    fn oldest_pending_tx_age(&self) -> Option<Duration> {
        let now = self.now();
        let pf_age = self.pf_deferred_since.and_then(|since| {
            let pf_j = self.active_pos_force_joint()?;
            self.pos_force_feedback_valid(pf_j, now)
                .then(|| now.saturating_duration_since(since))
        });
        [
            self.pending_tx.oldest_age(now),
            self.pending_hold_tx.oldest_age(now),
            pf_age,
        ]
        .into_iter()
        .flatten()
        .max()
    }
    fn set_output_lease_gen(&mut self, lease_gen: u64) { self.output_lease_gen = lease_gen; }
}
