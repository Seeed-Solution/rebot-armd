//! Feetech STS3215 torque enable/disable, rest-pose gating, and causal readback.

use std::time::{Duration, Instant};

use crate::driver::*;
use crate::protocol::feetech;
use crate::transport::Transport;
use super::engine::PendingFeetechDisable;

impl<T: Transport> FeetechArmDriver<T> {
    /// R12 M1/M3: Drain and decode any queued position replies from `partial_pos_rx`, `rx_buffer`,
    /// and the transport BEFORE clearing the input buffer, honouring `deadline` and `rx_budget`
    /// with zero heap allocations. Returns `(decoded_frames, drain_complete)`: when `rx_budget`
    /// or `deadline` prevents completing the drain before clearing (or before `line_poisoned`
    /// reaches 20 ms quiet), leaves unread transport bytes intact and returns `drain_complete = false`
    /// so the caller defers cleanly instead of discarding unread replies or exceeding its budget.
    pub(super) fn drain_and_decode_pending_rx_for_disable_budgeted(
        &mut self,
        deadline: Option<Instant>,
        rx_budget: &mut u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.in_disable_drain = true;
        }
        let res = self.drain_and_decode_pending_rx_for_disable_budgeted_inner(deadline, rx_budget);
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.in_disable_drain = false;
        }
        res
    }

    pub(super) fn drain_and_decode_pending_rx_for_disable_budgeted_inner(
        &mut self,
        deadline: Option<Instant>,
        rx_budget: &mut u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        let now = self.clock_now();
        let mut total_decoded: u16 = 0;
        // 1. Move any previously buffered partial position bytes into `rx_buffer` without allocating.
        if !self.partial_pos_rx.is_empty() {
            let plen = self.partial_pos_rx.len();
            let tlen = self.partial_pos_rx_timestamps.len();
            for i in 0..plen {
                let b = self.partial_pos_rx[i];
                let t = if i < tlen {
                    self.partial_pos_rx_timestamps[i]
                } else {
                    now
                };
                self.rx_buffer.push(b);
                self.rx_timestamps.push(t);
            }
            self.partial_pos_rx.clear();
            self.partial_pos_rx_timestamps.clear();
            self.incomplete_chunk_idx = None;
            self.incomplete_frame_started_at = None;
        }
        // 2. Pull any immediately queued transport bytes (bounded by `rx_budget` and `deadline`)
        //    and decode complete 8-byte position replies into `rest_pose_window` FIRST whenever
        //    `clear_input_buffer()` would discard queued bytes. Never discard unread bytes when
        //    `rx_budget == 0` or `deadline` prevents draining.
        if self.transport.clears_input_buffer() {
            if *rx_budget == 0 || deadline.is_some_and(|dl| self.clock_now() >= dl) {
                return Ok((total_decoded, false));
            }
            let mut buf = [0u8; 32];
            let mut saw_empty = false;
            for _ in 0..16 {
                if *rx_budget == 0 || deadline.is_some_and(|dl| self.clock_now() >= dl) {
                    break;
                }
                match self.transport.read(&mut buf) {
                    Ok(n) if n > 0 => {
                        let rx_at = self.clock_now();
                        self.push_rx(&buf[..n], rx_at);
                        total_decoded = total_decoded.saturating_add(
                            self.decode_queued_position_replies_in_rx_budgeted(rx_at, rx_budget),
                        );
                    }
                    _ => {
                        saw_empty = true;
                        break;
                    }
                }
            }
            if !saw_empty {
                return Ok((total_decoded, false));
            }
        }
        let decode_ts = self.clock_now();
        total_decoded = total_decoded.saturating_add(
            self.decode_queued_position_replies_in_rx_budgeted(decode_ts, rx_budget),
        );
        // 3. Clear remaining non-position / partial noise from RX.
        self.transport.clear_input_buffer();
        self.clear_rx();
        // 4. If line was poisoned, drain until continuous 20 ms idle while decoding any arriving
        //    position replies and strictly honouring `deadline` and `rx_budget`.
        if self.line_poisoned {
            let start = self.clock_now();
            let mut quiet_since = self.line_poison_quiet_since.unwrap_or(start);
            let mut idle_buf = [0u8; 64];
            loop {
                if deadline.is_some_and(|dl| self.clock_now() >= dl) || *rx_budget == 0 {
                    self.clear_rx();
                    return Ok((total_decoded, false));
                }
                let before_read = self.clock_now();
                match self.transport.read(&mut idle_buf) {
                    Ok(n) if n > 0 => {
                        let rx_at = self.clock_now();
                        quiet_since = rx_at;
                        self.line_poison_quiet_since = Some(rx_at);
                        self.push_rx(&idle_buf[..n], rx_at);
                        total_decoded = total_decoded.saturating_add(
                            self.decode_queued_position_replies_in_rx_budgeted(rx_at, rx_budget),
                        );
                        if self.elapsed_since(start) >= Self::LINE_IDLE_MAX_WAIT {
                            self.clear_rx();
                            return Err(ArmDriverError::ProtocolStatic("FEETECH_LINE_BUSY"));
                        }
                    }
                    Ok(_) | Err(TransportError::Timeout) | Err(TransportError::WouldBlock) => {
                        if self.elapsed_since(quiet_since) >= Self::LINE_IDLE_QUIET {
                            self.line_poisoned = false;
                            self.line_poison_quiet_since = None;
                            self.transport.clear_input_buffer();
                            self.clear_rx();
                            return Ok((total_decoded, true));
                        }
                        if deadline.is_some_and(|dl| self.clock_now() >= dl) {
                            self.clear_rx();
                            return Ok((total_decoded, false));
                        }
                        if self.clock_now() == before_read && deadline.is_none() {
                            self.clock_sleep(Duration::from_millis(1));
                        }
                        if self.elapsed_since(start) >= Self::LINE_IDLE_MAX_WAIT {
                            self.clear_rx();
                            return Err(ArmDriverError::ProtocolStatic("FEETECH_LINE_IDLE_TIMEOUT"));
                        }
                        self.clock_yield();
                    }
                    Err(e) => {
                        self.clear_rx();
                        return Err(e.into());
                    }
                }
            }
        }
        Ok((total_decoded, true))
    }

    pub(super) fn evaluate_rest_pose_gate(&self) -> RestPoseGateStatus {
        let Some(ref gate) = self.stopped_disable_gate else {
            return RestPoseGateStatus::RefusedNonRest;
        };
        if !gate.has_resting_pose {
            return RestPoseGateStatus::RefusedNonRest;
        }
        let arm_joints = gate.arm_joint_indices();
        if arm_joints.is_empty() {
            return RestPoseGateStatus::RefusedNonRest;
        }
        let now = self.clock_now();
        let caps = self.capabilities();
        let mut missing_fresh = false;
        let mut window_incomplete = false;
        for &i in arm_joints {
            let Some(s) = self.cached_states.get(i) else {
                return RestPoseGateStatus::RefusedNonRest;
            };
            let fresh = self
                .last_feedback_instant
                .get(i)
                .copied()
                .flatten()
                .is_some_and(|t| now.saturating_duration_since(t) <= STOPPED_DISABLE_FEEDBACK_MAX_AGE);
            if fresh {
                if !joint_state_valid(s, caps) || !gate.joint_at_rest(i, s.position) {
                    return RestPoseGateStatus::RefusedNonRest;
                }
            } else {
                missing_fresh = true;
            }
            if !self
                .rest_pose_window
                .get(i)
                .is_some_and(|w| w.is_confirmed(now, STOPPED_DISABLE_FEEDBACK_MAX_AGE))
            {
                window_incomplete = true;
            }
        }
        if self.line_poisoned {
            RestPoseGateStatus::PendingDrainOrWindow
        } else if missing_fresh {
            RestPoseGateStatus::StaleOrMissing
        } else if window_incomplete {
            RestPoseGateStatus::PendingDrainOrWindow
        } else {
            RestPoseGateStatus::Confirmed
        }
    }

    /// Single transmit choke point for Feetech `TORQUE_ENABLE = 0` writes, returning `Ok(true)`
    /// when `TORQUE_ENABLE = 0` was transmitted and `Ok(false)` when a `DisableRequested` must
    /// DEFER within its 1 s request lifetime (e.g., because `rest_pose_window` aged out during
    /// the pre-write RX drain — R12 M1/M3). Zero heap allocations; honours `deadline` & `rx_budget`.
    pub(super) fn write_torque_off_checked_budgeted(
        &mut self,
        servo_id: u8,
        kind: DisableKind,
        client_park: Option<(Option<u64>, Option<u64>)>,
        deadline: Option<Instant>,
        rx_budget: &mut u16,
    ) -> Result<bool, ArmDriverError> {
        if let Some((initial_estop_gen, expected_stop_gen)) = client_park {
            self.client_park_stop_check(initial_estop_gen, expected_stop_gen)?;
        }
        if let DisableKind::DisableRequested { started_at } = kind {
            if self.elapsed_since(started_at) >= STOPPED_DISABLE_REQUEST_LIFETIME {
                return Err(ArmDriverError::DisableRequestExpired);
            }
            match self.evaluate_rest_pose_gate() {
                RestPoseGateStatus::Confirmed => {}
                RestPoseGateStatus::RefusedNonRest => {
                    return Err(ArmDriverError::TorqueOffWouldDrop);
                }
                RestPoseGateStatus::PendingDrainOrWindow | RestPoseGateStatus::StaleOrMissing => {
                    return Ok(false);
                }
            }
        }
        // R12 M1/M3 / E3: For `DisableRequested`, decode any queued position replies into
        // `rest_pose_window` BEFORE clearing RX (within `rx_budget` and `deadline`); for
        // `DisableProtective`, clear best-effort so noisy RX never blocks a thermal/exit cut.
        match kind {
            DisableKind::DisableRequested { .. } => {
                let (_, drain_complete) =
                    self.drain_and_decode_pending_rx_for_disable_budgeted(deadline, rx_budget)?;
                if !drain_complete {
                    return Ok(false);
                }
            }
            DisableKind::DisableProtective { .. } => {
                if *rx_budget == 0 || deadline.is_some() {
                    self.transport.clear_input_buffer();
                    self.clear_rx();
                } else {
                    let _ = self.discard_pending_rx_for_request();
                }
            }
        }
        if let Some((initial_estop_gen, expected_stop_gen)) = client_park {
            self.client_park_stop_check(initial_estop_gen, expected_stop_gen)?;
        }
        // B4 / R12 M1: Recheck 1 s request lifetime and K=3 rest evidence IMMEDIATELY before
        // mutating enable telemetry or transmitting `TORQUE_ENABLE = 0`, strictly AFTER the RX
        // drain & decode. If the window aged out during the drain (within the 1 s lifetime),
        // DEFER (`Ok(false)`) instead of returning `TorqueOffWouldDrop`!
        if let DisableKind::DisableRequested { started_at } = kind {
            if self.elapsed_since(started_at) >= STOPPED_DISABLE_REQUEST_LIFETIME {
                return Err(ArmDriverError::DisableRequestExpired);
            }
            match self.evaluate_rest_pose_gate() {
                RestPoseGateStatus::Confirmed => {}
                RestPoseGateStatus::RefusedNonRest => {
                    return Err(ArmDriverError::TorqueOffWouldDrop);
                }
                RestPoseGateStatus::PendingDrainOrWindow | RestPoseGateStatus::StaleOrMissing => {
                    return Ok(false);
                }
            }
        }
        let eff_deadline = match (deadline, self.tx_release_deadline) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        };
        if eff_deadline.is_some_and(|dl| self.clock_now() >= dl) {
            return Ok(false);
        }
        // Invalidate per-motor torque windows immediately BEFORE the torque-changing write reaches
        // the transport; a gate refusal above leaves evidence intact. (Keep `rest_pose_window`
        // intact across the atomic multi-servo disable batch so servo 0's write does not poison
        // servos 1..N microseconds later; non-rest feedback or >100 ms staleness still invalidates it.)
        let servo_idx = self.servo_ids.iter().position(|&id| id == servo_id);
        let prev_reported = servo_idx.and_then(|idx| self.reported_enable[idx]);
        let prev_confirmed = servo_idx.and_then(|idx| self.last_enable_confirmed[idx]);
        let prev_commanded = self.torque_enable_commanded;
        if let Some(idx) = servo_idx {
            self.reported_enable[idx] = None;
            self.last_enable_confirmed[idx] = None;
        }
        self.torque_enable_commanded = false;
        let pkt = feetech::build_set_torque_array(servo_id, false);
        if let Err(e) = self.transport.write_all(&pkt) {
            if matches!(e, TransportError::WouldBlock) {
                if let Some(idx) = servo_idx {
                    self.reported_enable[idx] = prev_reported;
                    self.last_enable_confirmed[idx] = prev_confirmed;
                }
                self.torque_enable_commanded = prev_commanded;
            }
            return Err(e.into());
        }
        let flush_res = self.transport.flush();
        let flush_ok = flush_res.is_ok();
        let _ = flush_res;
        if *rx_budget > 0 && eff_deadline.is_none_or(|dl| self.clock_now() < dl) {
            let _ = self.drain_tx_echo_until(&pkt, eff_deadline);
        }
        // Record pure stack-only flush ambiguity flag; shadow clock sampling and staging are
        // deferred strictly until AFTER the entire disable batch + readback deadline check.
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.last_disable_flush_ok = flush_ok;
        }
        Ok(true)
    }

    /// Record one combined Feetech disable `WriteResult` without sampling the clock or pushing to
    /// `shadow_staging` inside the disable/retry/watchdog window; flushed lazily on
    /// `drain_shadow_observations` strictly after live retry-wake / watchdog outcome sealing.
    #[cfg(not(target_arch = "wasm32"))]
    pub(super) fn stage_feetech_disable_batch(
        &mut self,
        kind: DisableKind,
        target_mask: crate::torque::ChannelMask,
        outcomes: [crate::torque::MotorWriteOutcome; crate::torque::MAX_CHANNELS],
        ambiguous_gap: bool,
    ) {
        if !self.shadow_staging.is_enabled() || target_mask.is_empty() {
            return;
        }
        let wk = match kind {
            DisableKind::DisableRequested { .. } => crate::torque::WriteKind::DisableRequested,
            DisableKind::DisableProtective { .. } => crate::torque::WriteKind::DisableProtective,
        };
        PendingDisableShadowBatch::merge_batch(
            &mut self.pending_disable_shadow,
            PendingDisableShadowBatch {
                kind: wk,
                target_mask,
                outcomes,
                ambiguous_gap,
                write_done: None,
            },
        );
    }

    /// Direct non-deferred wrapper around [`Self::write_torque_off_checked_budgeted`] (used by `clear_fault`).
    pub(super) fn write_torque_off(
        &mut self,
        servo_id: u8,
        kind: DisableKind,
    ) -> Result<(), ArmDriverError> {
        let mut rx_budget = u16::MAX;
        let res = self.write_torque_off_checked_budgeted(servo_id, kind, None, None, &mut rx_budget);
        #[cfg(not(target_arch = "wasm32"))]
        if self.shadow_staging.is_enabled() {
            if let Some(idx) = self.servo_ids.iter().position(|&id| id == servo_id) {
                if idx < crate::torque::MAX_CHANNELS {
                    let mut outcomes = [crate::torque::MotorWriteOutcome::NotTargeted;
                        crate::torque::MAX_CHANNELS];
                    let (outcome, ambiguous_gap) = match &res {
                        Ok(true) => {
                            if self.last_disable_flush_ok {
                                (crate::torque::MotorWriteOutcome::Sent, false)
                            } else {
                                (crate::torque::MotorWriteOutcome::Ambiguous, true)
                            }
                        }
                        Ok(false) => (crate::torque::MotorWriteOutcome::Deferred, false),
                        Err(ArmDriverError::Transport(
                            TransportError::WouldBlock | TransportError::NoBufferSpace,
                        )) => (crate::torque::MotorWriteOutcome::Deferred, false),
                        Err(ArmDriverError::Transport(TransportError::Io(io)))
                            if io.kind() == std::io::ErrorKind::WouldBlock =>
                        {
                            (crate::torque::MotorWriteOutcome::Deferred, false)
                        }
                        Err(ArmDriverError::TorqueOffWouldDrop | ArmDriverError::DisableRequestExpired) => {
                            (crate::torque::MotorWriteOutcome::RejectedStale, false)
                        }
                        Err(_) => (crate::torque::MotorWriteOutcome::Failed, false),
                    };
                    outcomes[idx] = outcome;
                    self.stage_feetech_disable_batch(
                        kind,
                        crate::torque::ChannelMask::bit(idx as u8),
                        outcomes,
                        ambiguous_gap,
                    );
                }
            }
        }
        if res? {
            Ok(())
        } else {
            Err(ArmDriverError::TorqueOffWouldDrop)
        }
    }

    /// 0-byte `PING` FIFO barrier on the half-duplex serial bus: discards any late 1-byte reply
    /// from an earlier timed-out read until servo `id` answers `PING` with a 0-param response.
    /// Zero heap allocations; honours `deadline` and `rx_budget` (R12 M3).
    pub(super) fn ping_servo_barrier_budgeted(
        &mut self,
        id: u8,
        deadline: Option<Instant>,
        rx_budget: &mut u16,
    ) -> Result<(), ArmDriverError> {
        self.assert_clock_matches_transport();
        if *rx_budget == 0 || deadline.is_some_and(|dl| self.clock_now() >= dl) {
            return Err(ArmDriverError::Timeout(id));
        }
        self.invalidate_partial_pos_rx();
        if deadline.is_some() {
            if !self.poll_poisoned_line_idle() {
                return Err(ArmDriverError::ProtocolStatic("FEETECH_LINE_BUSY"));
            }
            self.transport.clear_input_buffer();
            self.clear_rx();
        } else {
            self.discard_pending_rx_for_request()?;
        }
        let req = feetech::build_ping_array(id);
        self.transport.write_all(&req)?;
        self.transport.flush()?;
        let req_written_at = self.clock_now();
        let echo_drained = self.drain_tx_echo_until(&req, deadline)?;

        let mut temp = [0u8; 1];
        const PING_TIMEOUT: Duration = Duration::from_millis(25);
        let ping_deadline = match deadline {
            Some(dl) => (req_written_at + PING_TIMEOUT).min(dl),
            None => req_written_at + PING_TIMEOUT,
        };
        let require_echo = !echo_drained && self.tx_echo == Some(true);
        let mut can_skip_echo = !echo_drained && self.tx_echo != Some(false);
        let mut echo_seen = echo_drained;

        loop {
            while !self.rx_buffer.is_empty() && *rx_budget > 0 {
                match feetech::parse_response_ref(&self.rx_buffer) {
                    Ok((pkt, consumed)) => {
                        let pkt_id = pkt.id;
                        let pkt_params_empty = pkt.params.is_empty();
                        let is_req_echo = can_skip_echo
                            && consumed >= req.len()
                            && self.rx_buffer[consumed - req.len()..consumed] == req[..];
                        let pkt_rx_at = self
                            .rx_timestamps
                            .get(consumed - 1)
                            .copied()
                            .unwrap_or_else(|| self.clock_now());
                        self.drain_rx_prefix(consumed);
                        if is_req_echo {
                            can_skip_echo = false;
                            echo_seen = true;
                            self.tx_echo = Some(true);
                            self.tx_echo_auto = false;
                            continue;
                        }
                        if require_echo && !echo_seen {
                            continue;
                        }
                        *rx_budget = rx_budget.saturating_sub(1);
                        if pkt_rx_at < req_written_at
                            || pkt_rx_at >= ping_deadline
                        {
                            continue;
                        }
                        if pkt_id == id && pkt_params_empty {
                            return Ok(());
                        }
                    }
                    Err(feetech::FeetechRefParseError::Incomplete) => break,
                    Err(_) => {
                        self.drain_rx_prefix(1);
                    }
                }
            }
            if *rx_budget == 0 || self.clock_now() >= ping_deadline {
                break;
            }
            match self.transport.read(&mut temp) {
                Ok(n) if n > 0 => {
                    let rx_at = self.clock_now();
                    if rx_at < req_written_at || rx_at >= ping_deadline {
                        break;
                    }
                    self.push_rx(&temp[..n], rx_at);
                }
                _ => self.clock_yield(),
            }
        }
        self.poison_line();
        self.clear_rx();
        Err(ArmDriverError::Timeout(id))
    }

    /// Causally ordered `REG_TORQUE_ENABLE` readback for servo `idx`: if a previous 1-byte read
    /// timed out (`unmatched_1b_read[idx]`), requires a 0-byte `PING` FIFO barrier first so a
    /// late 1-byte reply cannot be misread as the new torque-register state. Returns `(value, rx_at)`
    /// stamped with the reply's own host monotonic receive time (E3/S2/M3). Zero heap allocations.
    pub(super) fn read_torque_enable_causal_budgeted(
        &mut self,
        idx: usize,
        id: u8,
        deadline: Option<Instant>,
        rx_budget: &mut u16,
    ) -> Result<(u8, Instant), ArmDriverError> {
        if self.unmatched_1b_read.get(idx).copied().unwrap_or(false) {
            self.ping_servo_barrier_budgeted(id, deadline, rx_budget)?;
            if let Some(flag) = self.unmatched_1b_read.get_mut(idx) {
                *flag = false;
            }
        }
        self.read_register_u8_budgeted(id, feetech::REG_TORQUE_ENABLE, deadline, rx_budget)
    }

    /// Record reported `REG_TORQUE_ENABLE` telemetry for servo `idx` (observation/UI only).
    pub(super) fn record_reported_enable(&mut self, idx: usize, val: u8, rx_at: Instant) {
        match val {
            0 | 1 => {
                self.reported_enable[idx] = Some(val);
                self.last_enable_confirmed[idx] = Some(rx_at);
                self.missed_enable_polls[idx] = 0;
                if val == 0 {
                    self.reported_torque_loss_sample_seq =
                        self.reported_torque_loss_sample_seq.wrapping_add(1).max(1);
                }
            }
            _ => {
                self.reported_enable[idx] = None;
                self.last_enable_confirmed[idx] = None;
            }
        }
    }

    /// Zero-allocation static error code classifier for Feetech register readback failures (R12 M3).
    pub(super) fn readback_error_code(err: &ArmDriverError) -> &'static str {
        match err {
            ArmDriverError::Timeout(_) => "FEETECH_READ_TIMEOUT",
            ArmDriverError::AmbiguousDuplicateReply(_) => "FEETECH_AMBIGUOUS_DUPLICATE_REPLY",
            ArmDriverError::EstopInhibited => "FEETECH_ESTOP_INHIBITED",
            ArmDriverError::ProtocolStatic(code) => code,
            ArmDriverError::Transport(TransportError::Timeout) => "FEETECH_TRANSPORT_TIMEOUT",
            ArmDriverError::Transport(TransportError::WouldBlock) => "FEETECH_TRANSPORT_WOULD_BLOCK",
            ArmDriverError::Transport(TransportError::NotOpen) => "FEETECH_TRANSPORT_NOT_OPEN",
            ArmDriverError::Transport(_) => "FEETECH_TRANSPORT_ERROR",
            _ => "FEETECH_READ_FAILED",
        }
    }

    /// Reads `REG_TORQUE_ENABLE` once per servo causally (preserving E3 request/reply
    /// correlation and 20 ms poisoned-line idle) to refresh reported enable telemetry without
    /// heap allocation, honouring optional `deadline` and `max_rx_frames` (R12 M3).
    pub(super) fn read_torque_enable_once_budgeted(
        &mut self,
        ids: &[u8],
        stop_stamp: Option<(u64, u64)>,
        deadline: Option<Instant>,
        max_rx_frames: Option<u16>,
    ) -> Result<(usize, usize, u16, bool, Option<(u8, &'static str)>), ArmDriverError> {
        let mut remaining_rx = max_rx_frames.unwrap_or(u16::MAX);
        let start_rx = remaining_rx;
        let mut budget_exhausted = false;
        let mut first_error: Option<(u8, &'static str)> = None;
        for (idx, &id) in ids.iter().enumerate() {
            if let Some(stamp) = stop_stamp {
                self.strict_stop_check(Some(stamp))?;
            }
            if remaining_rx == 0 {
                budget_exhausted = true;
                break;
            }
            if deadline.is_some_and(|dl| self.clock_now() >= dl) {
                break;
            }
            match self.read_torque_enable_causal_budgeted(idx, id, deadline, &mut remaining_rx) {
                Ok((v @ (0 | 1), rx_at)) => {
                    self.record_reported_enable(idx, v, rx_at);
                }
                Ok(_) => {
                    self.reported_enable[idx] = None;
                    self.last_enable_confirmed[idx] = None;
                    if first_error.is_none() {
                        first_error = Some((id, "FEETECH_UNEXPECTED_TORQUE_ENABLE_VALUE"));
                    }
                }
                Err(e) => {
                    self.reported_enable[idx] = None;
                    self.last_enable_confirmed[idx] = None;
                    if first_error.is_none() {
                        first_error = Some((id, Self::readback_error_code(&e)));
                    }
                }
            }
        }
        let count_0 = (0..ids.len())
            .filter(|&i| self.reported_enable.get(i).copied().flatten() == Some(0))
            .count();
        let count_1 = (0..ids.len())
            .filter(|&i| self.reported_enable.get(i).copied().flatten() == Some(1))
            .count();
        let rx_used = start_rx.saturating_sub(remaining_rx);
        self.last_enable_poll = Some(self.clock_now());
        Ok((count_0, count_1, rx_used, budget_exhausted, first_error))
    }

    pub(super) fn read_torque_enable_states(&mut self, ids: &[u8]) -> (usize, usize, Option<(u8, &'static str)>) {
        match self.read_torque_enable_once_budgeted(ids, None, None, None) {
            Ok((c0, c1, _, _, first_err)) => (c0, c1, first_err),
            Err(e) => (0, 0, Some((ids.first().copied().unwrap_or(0), Self::readback_error_code(&e)))),
        }
    }

    /// Executes Feetech `TORQUE_ENABLE = 0` writes through `write_torque_off_checked_budgeted`,
    /// returning `Ok((true, rx_used, exhausted))` when all servos completed, or
    /// `Ok((false, rx_used, exhausted))` when deferred within the 1 s lifetime (setting
    /// `self.pending_disable = Some(kind)` — R12 M1/M3). Zero heap allocations; honours `deadline`
    /// and `max_rx_frames`.
    pub(super) fn execute_disable_writes_budgeted(
        &mut self,
        kind: DisableKind,
        client_park: Option<(Option<u64>, Option<u64>)>,
        deadline: Option<Instant>,
        max_rx_frames: Option<u16>,
    ) -> Result<(bool, u16, bool), ArmDriverError> {
        self.assert_clock_matches_transport();
        if let Some((eg, sg)) = client_park {
            self.client_park_stop_check(eg, sg)?;
        }
        let n = self.servo_ids.len().min(crate::rt_core::mailbox::MAX_DOF);
        let all_mask = if n >= 64 { u64::MAX } else { (1u64 << n) - 1 };
        if (self.disable_dispatched_mask & all_mask) == all_mask {
            self.disable_dispatched_mask = 0;
        }
        let skip_dispatched = matches!(kind, DisableKind::DisableRequested { .. });
        let mut ids = [0u8; crate::rt_core::mailbox::MAX_DOF];
        ids[..n].copy_from_slice(&self.servo_ids[..n]);
        let mut rx_remaining = max_rx_frames.unwrap_or(u16::MAX);
        let rx_start = rx_remaining;
        let mut budget_exhausted = false;
        let mut wrote_any = false;
        let mut deferred = false;
        let mut refused_err = None;
        let mut write_err = None;
        #[cfg(not(target_arch = "wasm32"))]
        let mut shadow_target_mask = crate::torque::ChannelMask::EMPTY;
        #[cfg(not(target_arch = "wasm32"))]
        let mut shadow_outcomes =
            [crate::torque::MotorWriteOutcome::NotTargeted; crate::torque::MAX_CHANNELS];
        #[cfg(not(target_arch = "wasm32"))]
        let mut shadow_ambiguous_gap = false;
        let eff_deadline = match (deadline, self.tx_release_deadline) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        };
        for (idx, &id) in ids[..n].iter().enumerate() {
            if skip_dispatched && idx < 64 && (self.disable_dispatched_mask & (1u64 << idx)) != 0 {
                continue;
            }
            if let Some((eg, sg)) = client_park {
                if let Err(e) = self.client_park_stop_check(eg, sg) {
                    #[cfg(not(target_arch = "wasm32"))]
                    for rem in idx..n.min(crate::torque::MAX_CHANNELS) {
                        if !skip_dispatched || (self.disable_dispatched_mask & (1u64 << rem)) == 0 {
                            shadow_target_mask.insert(rem as u8);
                            shadow_outcomes[rem] = crate::torque::MotorWriteOutcome::RejectedStale;
                        }
                    }
                    if !wrote_any && self.disable_dispatched_mask == 0 {
                        #[cfg(not(target_arch = "wasm32"))]
                        self.stage_feetech_disable_batch(
                            kind,
                            shadow_target_mask,
                            shadow_outcomes,
                            shadow_ambiguous_gap,
                        );
                        return Err(e);
                    }
                    write_err = Some(ArmDriverError::HardwareFault(
                        "Client park disable stopped mid-batch after partial servo disable".into(),
                    ));
                    break;
                }
            }
            match self.write_torque_off_checked_budgeted(id, kind, client_park, eff_deadline, &mut rx_remaining) {
                Ok(true) => {
                    wrote_any = true;
                    if idx < 64 {
                        self.disable_dispatched_mask |= 1u64 << idx;
                    }
                    #[cfg(not(target_arch = "wasm32"))]
                    if idx < crate::torque::MAX_CHANNELS {
                        shadow_target_mask.insert(idx as u8);
                        if self.last_disable_flush_ok {
                            shadow_outcomes[idx] = crate::torque::MotorWriteOutcome::Sent;
                        } else {
                            shadow_outcomes[idx] = crate::torque::MotorWriteOutcome::Ambiguous;
                            shadow_ambiguous_gap = true;
                        }
                    }
                }
                Ok(false) => {
                    #[cfg(not(target_arch = "wasm32"))]
                    for rem in idx..n.min(crate::torque::MAX_CHANNELS) {
                        if !skip_dispatched || (self.disable_dispatched_mask & (1u64 << rem)) == 0 {
                            shadow_target_mask.insert(rem as u8);
                            shadow_outcomes[rem] = crate::torque::MotorWriteOutcome::Deferred;
                        }
                    }
                    deferred = true;
                    break;
                }
                Err(e) if client_park.is_some() && is_estop_inhibited(&e) => {
                    #[cfg(not(target_arch = "wasm32"))]
                    for rem in idx..n.min(crate::torque::MAX_CHANNELS) {
                        if !skip_dispatched || (self.disable_dispatched_mask & (1u64 << rem)) == 0 {
                            shadow_target_mask.insert(rem as u8);
                            shadow_outcomes[rem] = crate::torque::MotorWriteOutcome::RejectedStale;
                        }
                    }
                    if !wrote_any && self.disable_dispatched_mask == 0 {
                        #[cfg(not(target_arch = "wasm32"))]
                        self.stage_feetech_disable_batch(
                            kind,
                            shadow_target_mask,
                            shadow_outcomes,
                            shadow_ambiguous_gap,
                        );
                        return Err(e);
                    }
                    write_err = Some(ArmDriverError::HardwareFault(
                        "Client park disable stopped mid-batch after partial servo disable".into(),
                    ));
                    break;
                }
                Err(e @ (ArmDriverError::TorqueOffWouldDrop | ArmDriverError::DisableRequestExpired)) => {
                    #[cfg(not(target_arch = "wasm32"))]
                    if idx < crate::torque::MAX_CHANNELS {
                        shadow_target_mask.insert(idx as u8);
                        shadow_outcomes[idx] = crate::torque::MotorWriteOutcome::RejectedStale;
                    }
                    if refused_err.is_none() {
                        refused_err = Some(e);
                    }
                }
                Err(e) => {
                    #[cfg(not(target_arch = "wasm32"))]
                    if idx < crate::torque::MAX_CHANNELS {
                        shadow_target_mask.insert(idx as u8);
                        shadow_outcomes[idx] = match &e {
                            ArmDriverError::Transport(
                                TransportError::WouldBlock | TransportError::NoBufferSpace,
                            ) => crate::torque::MotorWriteOutcome::Deferred,
                            ArmDriverError::Transport(TransportError::Io(io))
                                if io.kind() == std::io::ErrorKind::WouldBlock =>
                            {
                                crate::torque::MotorWriteOutcome::Deferred
                            }
                            _ => crate::torque::MotorWriteOutcome::Failed,
                        };
                    }
                    write_err = Some(e);
                }
            }
        }
        if deferred && refused_err.is_none() && write_err.is_none() {
            if wrote_any && rx_remaining > 0 && deadline.is_none_or(|dl| self.clock_now() < dl) {
                if let Ok((_, _, rb_used, rb_ex, _)) =
                    self.read_torque_enable_once_budgeted(&ids[..n], None, deadline, Some(rx_remaining))
                {
                    rx_remaining = rx_remaining.saturating_sub(rb_used);
                    budget_exhausted |= rb_ex;
                }
            }
            self.pending_disable = Some(PendingFeetechDisable { kind, client_park });
            #[cfg(not(target_arch = "wasm32"))]
            self.stage_feetech_disable_batch(
                kind,
                shadow_target_mask,
                shadow_outcomes,
                shadow_ambiguous_gap,
            );
            return Ok((false, rx_start.saturating_sub(rx_remaining), budget_exhausted));
        }
        if !wrote_any && self.disable_dispatched_mask == 0 {
            if let Some(e) = refused_err.take().or_else(|| write_err.take()) {
                #[cfg(not(target_arch = "wasm32"))]
                self.stage_feetech_disable_batch(
                    kind,
                    shadow_target_mask,
                    shadow_outcomes,
                    shadow_ambiguous_gap,
                );
                return Err(e);
            }
        }
        if wrote_any && rx_remaining > 0 && deadline.is_none_or(|dl| self.clock_now() < dl) {
            if let Ok((_, _, rb_used, rb_ex, _)) =
                self.read_torque_enable_once_budgeted(&ids[..n], None, deadline, Some(rx_remaining))
            {
                rx_remaining = rx_remaining.saturating_sub(rb_used);
                budget_exhausted |= rb_ex;
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        self.stage_feetech_disable_batch(
            kind,
            shadow_target_mask,
            shadow_outcomes,
            shadow_ambiguous_gap,
        );
        if let Some(e) = refused_err.or(write_err) {
            return Err(e);
        }
        if (self.disable_dispatched_mask & all_mask) == all_mask {
            self.disable_dispatched_mask = 0;
        }
        Ok((true, rx_start.saturating_sub(rx_remaining), budget_exhausted))
    }

    /// Shared zero-allocation disable poll step for `poll_torque` and `poll_torque_budgeted`
    /// honouring both `deadline` and `max_rx_frames` (R12 B1/M1/M3).
    pub(super) fn poll_disable_step(
        &mut self,
        deadline: Instant,
        max_rx_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.assert_clock_matches_transport();
        let Some(PendingFeetechDisable { kind, client_park }) = self.pending_disable.take() else {
            return Err(ArmDriverError::ProtocolStatic("NO_TORQUE_TRANSITION_PENDING"));
        };
        if let Some((eg, sg)) = client_park {
            self.client_park_stop_check(eg, sg)?;
        }
        let started_at = match kind {
            DisableKind::DisableRequested { started_at } => started_at,
            DisableKind::DisableProtective { .. } => {
                let (_, rx_used, exhausted) =
                    self.execute_disable_writes_budgeted(kind, client_park, Some(deadline), Some(max_rx_frames))?;
                self.last_poll_rx = Some((rx_used, exhausted));
                return Ok(TorqueProgress::Done(TorqueOutcome::Confirmed));
            }
        };
        let exp = started_at + STOPPED_DISABLE_REQUEST_LIFETIME;
        if self.clock_now() >= exp {
            return Err(ArmDriverError::DisableRequestExpired);
        }
        // R12 B1: Re-check the rest-pose gate FIRST (the owner's `poll_hardware_rx` may already
        // have read fresh feedback in this control release); never sleep or block waiting for K=3.
        match self.evaluate_rest_pose_gate() {
            RestPoseGateStatus::Confirmed => {
                let (done, rx_used, exhausted) =
                    self.execute_disable_writes_budgeted(kind, client_park, Some(deadline), Some(max_rx_frames))?;
                self.last_poll_rx = Some((rx_used, exhausted));
                if done {
                    return Ok(TorqueProgress::Done(TorqueOutcome::Confirmed));
                }
                let wake = (self.clock_now() + EVIDENCE_PROBE_INTERVAL).min(exp);
                return Ok(TorqueProgress::Pending { wake });
            }
            RestPoseGateStatus::RefusedNonRest => {
                return Err(ArmDriverError::TorqueOffWouldDrop);
            }
            RestPoseGateStatus::PendingDrainOrWindow | RestPoseGateStatus::StaleOrMissing => {}
        }
        let already_read_this_release =
            self.control_release_managed && self.last_read_release_id == self.control_release_id;
        if max_rx_frames == 0 {
            self.last_poll_rx = Some((0, true));
            self.pending_disable = Some(PendingFeetechDisable { kind, client_park });
            let wake = (self.clock_now() + EVIDENCE_PROBE_INTERVAL).min(exp);
            return Ok(TorqueProgress::Pending { wake });
        }
        if already_read_this_release || self.clock_now() >= deadline {
            self.last_poll_rx = Some((0, false));
            self.pending_disable = Some(PendingFeetechDisable { kind, client_park });
            let wake = (self.clock_now() + EVIDENCE_PROBE_INTERVAL).min(exp);
            return Ok(TorqueProgress::Pending { wake });
        }
        if !self.control_release_managed {
            self.control_release_id = self.control_release_id.wrapping_add(1).max(1);
        }
        let (poll_frames, poll_exhausted) = self
            .read_joint_states_budgeted_pass(deadline, max_rx_frames)
            .unwrap_or((0, false));
        if self.clock_now() >= exp {
            self.last_poll_rx = Some((poll_frames, poll_exhausted));
            return Err(ArmDriverError::DisableRequestExpired);
        }
        if let Some((eg, sg)) = client_park {
            self.client_park_stop_check(eg, sg)?;
        }
        match self.evaluate_rest_pose_gate() {
            RestPoseGateStatus::Confirmed => {
                let rem_rx = max_rx_frames.saturating_sub(poll_frames);
                let (done, write_rx, write_exhausted) =
                    self.execute_disable_writes_budgeted(kind, client_park, Some(deadline), Some(rem_rx))?;
                self.last_poll_rx = Some((
                    poll_frames.saturating_add(write_rx),
                    poll_exhausted || write_exhausted,
                ));
                if done {
                    Ok(TorqueProgress::Done(TorqueOutcome::Confirmed))
                } else {
                    let wake = (self.clock_now() + EVIDENCE_PROBE_INTERVAL).min(exp);
                    Ok(TorqueProgress::Pending { wake })
                }
            }
            RestPoseGateStatus::RefusedNonRest => {
                self.last_poll_rx = Some((poll_frames, poll_exhausted));
                Err(ArmDriverError::TorqueOffWouldDrop)
            }
            RestPoseGateStatus::PendingDrainOrWindow | RestPoseGateStatus::StaleOrMissing => {
                self.last_poll_rx = Some((poll_frames, poll_exhausted));
                self.pending_disable = Some(PendingFeetechDisable { kind, client_park });
                let wake = (self.clock_now() + EVIDENCE_PROBE_INTERVAL).min(exp);
                Ok(TorqueProgress::Pending { wake })
            }
        }
    }

}
