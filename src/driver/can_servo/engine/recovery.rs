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
    /// Remaining per-release RX frames allowed on this driver under the active control release.
    #[inline]
    pub(in crate::driver::can_servo) fn driver_remaining_release_rx(&self) -> u16 {
        if self.in_active_control_release() {
            self.release_rx_limit.saturating_sub(self.release_rx_frames)
        } else {
            crate::rt_core::ops::RX_FRAME_BUDGET
        }
    }

    /// Tightens a caller-supplied RX frame limit against the active control release ledger.
    #[inline]
    pub(in crate::driver::can_servo) fn clamp_release_rx_limit(&self, max_rx_frames: u16) -> u16 {
        if self.in_active_control_release() {
            max_rx_frames.min(self.driver_remaining_release_rx())
        } else {
            max_rx_frames.min(crate::rt_core::ops::RX_FRAME_BUDGET)
        }
    }

    /// Compute the fixed `u64` bitmask of joints lacking recent feedback at `now` (zero alloc).
    ///
    /// Covers every joint J1–J7 (including J7 under `NoOutputUntilFresh`). When
    /// `defer_active_pf_freeze` is true and an enabled, verified `PosForce` joint is already
    /// emitting `0x307` freeze frames during an output pass (`pf_last_fresh_position.is_some()`),
    /// its `0xCC` slot is left for `emit_pos_force_j7_impl` after `0x307` freeze so tight
    /// stopped-hold budgets keep Class 1/3 arm holds + J7 freeze prioritized. Under
    /// `NoOutputUntilFresh` (`pf_last_fresh_position.is_none()`), `FailedHold`, `Idle`-torqued,
    /// stopped, or any stale arm joint J1–J6, the joint is served directly here before ordinary TX.
    pub(in crate::driver::can_servo) fn compute_stale_refresh_mask(&self, now: Instant, defer_active_pf_freeze: bool) -> u64 {
        let n = self.motor_ids.len().min(self.last_feedback_instant.len()).min(64);
        let active_pf = self.active_pos_force_joint();
        let pf_configured = self.pos_force.as_ref().map(|pf| pf.joint);
        // During active operational output (enabled, output running every tick, not stopped),
        // arm joints J1–J6 receive reply-producing motion frames in this very tick; reserve the
        // single 0xCC slot for joints lacking reply-producing traffic (NoOutputUntilFresh J7,
        // or post-freeze J7). In FailedHold, Idle-torqued (!output_ran_since_rx), and stopped
        // states, every stale joint J1–J7 is served directly here.
        let active_operational_output = self.is_enabled
            && self.output_ran_since_rx
            && !self.is_stopped_for_disable()
            && self.is_feedback_valid();
        let mut stale = 0u64;
        for i in 0..n {
            let is_stale = if pf_configured == Some(i) {
                if defer_active_pf_freeze
                    && self.is_enabled
                    && active_pf == Some(i)
                    && self.pf_last_fresh_position.is_some()
                    && (self.output_ran_since_rx || self.tx_budget.max_frames <= self.motor_ids.len())
                {
                    false
                } else {
                    !self.pos_force_feedback_valid(i, now)
                }
            } else if active_operational_output {
                false
            } else {
                self.last_feedback_instant[i]
                    .is_none_or(|t| now.saturating_duration_since(t) >= DAMIAO_REFRESH_AFTER)
            };
            if is_stale {
                stale |= 1u64 << i;
            }
        }
        stale
    }

    /// Service the unified owner-local `0xCC` refresh scheduler: transmits and flushes at most
    /// ONE `0xCC` read-only status refresh frame per tick, reserved before ordinary TX.
    ///
    /// Preserves all stop, MIT fence (`poll_tx_allowed`), deadline, and RX-provenance
    /// (`last_read_drained && !awaiting_rx_boundary`) checks with zero locks and zero heap
    /// allocations. (Enable-transition causal `0xCC` refreshes run through
    /// [`Self::send_refresh_with_boundary_opts`] sharing the same `slot_used_this_tick` cap.)
    pub(in crate::driver::can_servo) fn service_refresh_scheduler(
        &mut self,
        expected_stamp: StopStamp,
        deadline: Option<Deadline>,
        defer_active_pf_freeze: bool,
        allowed_mask: u64,
    ) -> Result<bool, ArmDriverError> {
        if self.is_enabling || !self.refresh_scheduler.slot_available() {
            return Ok(false);
        }
        if !self.last_read_drained || self.awaiting_rx_boundary || !self.poll_tx_allowed() {
            return Ok(false);
        }
        if self.pending_stop_check(Some(expected_stamp)).is_err() {
            return Ok(false);
        }
        let now = self.now();
        if deadline.is_some_and(|d| d.passed(now)) {
            return Ok(false);
        }
        // Optional background 0xCC refresh: defer to the next release when remaining release
        // budget is below the optional-work margin so it never pushes a release past deadline.
        if !self.has_optional_release_budget() {
            self.refresh_scheduler.record_deferred();
            return Ok(false);
        }
        let stale_mask = self.compute_stale_refresh_mask(now, defer_active_pf_freeze) & allowed_mask;
        let n = self.motor_ids.len().min(self.last_feedback_instant.len()).min(64);
        let Some(joint) =
            self.refresh_scheduler
                .pick_next(stale_mask, n, now, DAMIAO_REFRESH_AFTER)
        else {
            return Ok(false);
        };
        let _ = self.ensure_tx_pass();
        match self.send_plain(TxKind::Refresh, Some(expected_stamp), deadline, false, |d| {
            Some(damiao::build_refresh_frame(d.motor_ids[joint]))
        }) {
            Ok(_) => {
                if self.pending_stop_check(Some(expected_stamp)).is_err() {
                    self.discard_all_pending_tx();
                    self.refresh_scheduler.record_deferred();
                    return Err(ArmDriverError::EstopInhibited);
                }
                if self.flush_cutoff_exceeded() {
                    self.mark_tx_pass_backpressured();
                    self.discard_all_pending_tx();
                    self.refresh_scheduler.record_deferred();
                    return Err(TransportError::Timeout.into());
                }
                if let Err(e) = self.flush_can_and_commit() {
                    self.mark_tx_pass_backpressured();
                    self.discard_all_pending_tx();
                    self.refresh_scheduler.record_deferred();
                    return Err(e.into());
                }
                self.refresh_scheduler.commit_sent(joint, n, self.now());
                if self.active_pos_force_joint() == Some(joint) {
                    self.pf_refresh_pending = false;
                    self.pf_refresh_deferred_ticks = 0;
                }
                Ok(true)
            }
            Err(TxRefusal::Deferred | TxRefusal::DeadlinePassed) => {
                self.refresh_scheduler.record_deferred();
                if self.active_pos_force_joint() == Some(joint) {
                    self.pf_refresh_deferred_ticks =
                        self.pf_refresh_deferred_ticks.saturating_add(1);
                }
                Ok(false)
            }
            Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                self.discard_all_pending_tx();
                Err(ArmDriverError::EstopInhibited)
            }
            Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => {
                if self.pending_stop_check(Some(expected_stamp)).is_err() {
                    self.discard_all_pending_tx();
                    return Err(ArmDriverError::EstopInhibited);
                }
                self.refresh_scheduler.record_deferred();
                if self.active_pos_force_joint() == Some(joint) {
                    self.pf_refresh_deferred_ticks =
                        self.pf_refresh_deferred_ticks.saturating_add(1);
                }
                Ok(false)
            }
            Err(_) => {
                self.refresh_scheduler.skip_non_retryable(joint, n, self.now());
                Ok(false)
            }
        }
    }

    /// Receive boundary of an MIT transaction, established before its requests go out: read until
    /// the receive queue is observed empty (bounded). Every frame dequeued afterwards (RX sequence
    /// number larger than the returned one) was enqueued after the boundary, so frames queued
    /// before the transaction never count, and a reply arriving right after its request does.
    /// Pure RX-only drain (no autonomous recovery/refresh TX, no `begin_tx_tick`, zero heap
    /// allocation). Returns `true` when the transport explicitly reported an empty RX queue.
    pub(in crate::driver::can_servo) fn drain_rx_frames_only(&mut self, deadline: Option<Deadline>, max_frames: usize) -> bool {
        let limit = if self.in_active_control_release() {
            max_frames.min(usize::from(self.driver_remaining_release_rx()))
        } else {
            max_frames
        };
        let mut effective_deadline = deadline.map(|d| d.at);
        if let Some(tx_dl) = self.tx_release_deadline {
            effective_deadline = Some(effective_deadline.map_or(tx_dl, |d| d.min(tx_dl)));
        }
        if let Some(rx_dl) = self.rx_deadline {
            effective_deadline = Some(effective_deadline.map_or(rx_dl, |d| d.min(rx_dl)));
        }
        let mut frames = 0usize;
        let mut drained = false;
        while frames < limit {
            let attempt = self.now();
            if effective_deadline.is_some_and(|d| attempt >= d) {
                break;
            }
            match self.receive_with_rx_time_timed() {
                Ok((frame, rx_time)) => {
                    frames += 1;
                    self.process_rx_frame(&frame, false, rx_time);
                }
                Err(TransportError::Timeout) | Err(TransportError::WouldBlock) => {
                    self.mark_rx_empty(attempt);
                    drained = true;
                    break;
                }
                Err(_) => break,
            }
        }
        if frames > 0 {
            let f_u16 = u16::try_from(frames).unwrap_or(u16::MAX);
            if let Some(rem) = self.rx_frame_limit.as_mut() {
                *rem = rem.saturating_sub(f_u16);
            }
        }
        drained
    }

    /// Transmit a read-only `0xCC` status refresh probe on an empty software queue through the
    /// single `send()` chokepoint, flush, and check for fresh post-probe feedback.
    /// Zero heap allocations when `blocking == false` (RT executor path).
    pub(in crate::driver::can_servo) fn probe_tx_ready_inner(
        &mut self,
        deadline: Option<Deadline>,
        blocking: bool,
    ) -> Result<bool, ArmDriverError> {
        if !blocking && (!self.can.supports_nonblocking_rt() || !self.can.supports_bounded_flush()) {
            return Err(ArmDriverError::Protocol(
                "UNSUPPORTED: transport does not support non-blocking RT I/O and bounded flush".into(),
            ));
        }
        let pending = self.total_pending_tx_len() as usize;
        if pending > 0 {
            self.tx_probe = None;
            return Err(ArmDriverError::TxDeferred { pending });
        }
        if self.motor_ids.is_empty() {
            return Ok(true);
        }
        let start_stamp = self.current_stop_stamp();
        self.pending_stop_check(Some(start_stamp))?;
        if let Some((_, at)) = self.mit_fence_trip {
            if self.since(at) < self.mit_fence_holdoff {
                return Err(ArmDriverError::Transport(TransportError::WouldBlock));
            }
        }
        let release_cap_passed = |d: &Self| d.tx_release_deadline.is_some_and(|cap| d.now() >= cap);
        if deadline.is_some_and(|d| d.passed(self.now())) || release_cap_passed(self) {
            return Err(ArmDriverError::Transport(TransportError::Timeout));
        }
        // Pre-drain pre-existing queued RX backlog before writing the probe.
        let mut drained = self.drain_rx_frames_only(deadline, 64);
        if !drained && blocking {
            for _ in 0..7 {
                drained = self.drain_rx_frames_only(deadline, 64);
                if drained {
                    break;
                }
            }
        }
        if !drained {
            return Err(ArmDriverError::Transport(TransportError::Timeout));
        }
        self.pending_stop_check(Some(start_stamp))?;
        if deadline.is_some_and(|d| d.passed(self.now())) || release_cap_passed(self) {
            return Err(ArmDriverError::Transport(TransportError::Timeout));
        }
        self.tx_probe = None;
        // Preserve any active `self.tx_release_deadline` cap and already-spent release TX
        // attempts installed by the release owner so the clear-fault probe uses the exact same
        // release ledger and cutoff as every other TX in the release.
        self.ensure_release_tx_tick();
        let probe_joint = 0usize;
        match self.send_plain(TxKind::Refresh, Some(start_stamp), deadline, false, |d| {
            Some(damiao::build_refresh_frame(d.motor_ids[probe_joint]))
        }) {
            Ok(_) => {}
            Err(TxRefusal::DeadlinePassed) => {
                return Err(ArmDriverError::Transport(TransportError::Timeout));
            }
            Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                return Err(ArmDriverError::EstopInhibited);
            }
            Err(TxRefusal::Deferred) => {
                if release_cap_passed(self) {
                    return Err(ArmDriverError::Transport(TransportError::Timeout));
                }
                return Err(ArmDriverError::TxDeferred { pending: 1 });
            }
            Err(TxRefusal::Transport(e)) => {
                return Err(ArmDriverError::Transport(e));
            }
            Err(r) => {
                return Err(Self::refusal_to_driver_err(r));
            }
        }
        self.pending_stop_check(Some(start_stamp))?;
        if deadline.is_some_and(|d| d.passed(self.now())) || self.flush_cutoff_exceeded() {
            return Err(ArmDriverError::Transport(TransportError::Timeout));
        }
        if let Err(e) = self.flush_can_and_commit() {
            self.mark_tx_pass_backpressured();
            return Err(ArmDriverError::Transport(e));
        }
        // Snapshot the empty-queue RX sequence boundary strictly AFTER the probe write and flush
        // return successfully, draining any pre-probe frames that arrived between the pre-drain
        // and the probe write. Because Linux SocketCAN supplies no per-frame hardware arrival
        // timestamps (`CanTransport::receive_timestamped` returns `None`), this post-write empty
        // sequence boundary is the causal proof: only feedback decoded strictly after it
        // (`last_feedback_seq[probe_joint] > boundary_seq`) can satisfy `tx_probe_confirmed()`.
        let mut post_drained = self.drain_rx_frames_only(deadline, 64);
        if !post_drained && blocking {
            for _ in 0..7 {
                post_drained = self.drain_rx_frames_only(deadline, 64);
                if post_drained {
                    break;
                }
            }
        }
        if !post_drained {
            return Err(ArmDriverError::Transport(TransportError::Timeout));
        }
        let boundary_seq = self.rx_empty_seq;
        let sent_at = self.now();
        self.tx_probe = Some(TxProbeState {
            joint: probe_joint,
            boundary_seq,
            sent_at,
        });
        if blocking {
            loop {
                let _ = self.drain_rx_frames_only(deadline, 64);
                if self.tx_probe_confirmed() {
                    return Ok(true);
                }
                if self.since(sent_at) >= REGISTER_REPLY_TIMEOUT
                    || deadline.is_some_and(|d| d.passed(self.now()))
                {
                    return Ok(false);
                }
                self.pause(Duration::from_micros(500));
            }
        } else {
            let _ = self.drain_rx_frames_only(deadline, 64);
            Ok(self.tx_probe_confirmed())
        }
    }

    pub(in crate::driver::can_servo) fn rx_boundary(&mut self) -> Result<u64, ArmDriverError> {
        for _ in 0..8 {
            let _ = self.read_joint_states();
            if self.last_read_drained { return Ok(self.rx_empty_seq); }
        }
        Err(ArmDriverError::HardwareFault("receive queue not drained: no boundary for the register transaction".into()))
    }

    #[inline]
    pub(in crate::driver::can_servo) fn receive_with_rx_time_timed(&mut self) -> Result<(CanFrame, Option<crate::transport::CanRxTime>), TransportError> {
        let res = if crate::diagnostics::tick::hold_io_active() {
            let t0 = Instant::now();
            let res = self.can.receive_with_rx_time();
            let wall_ns = t0.elapsed().as_nanos().min(u128::from(u64::MAX)) as u64;
            crate::diagnostics::tick::record_hold_rx_ns(wall_ns);
            res
        } else {
            self.can.receive_with_rx_time()
        };
        // Single convergence point charging every managed-release CAN RX frame exactly once
        // (poll, transitions, wakes, ModeSwitch, commissioning, and MIT recovery — including
        // discarded frames and error exits) into the active release's shared ledger counter.
        if res.is_ok() && self.in_active_control_release() {
            self.release_rx_frames = self.release_rx_frames.saturating_add(1);
        }
        res
    }

    /// Recovery writes (cached MIT replay, POS_VEL hold at the measured pose): never while the
    /// fence is armed or tripped.
    pub(in crate::driver::can_servo) fn recovery_tx_allowed(&self) -> bool { self.mit_fence.is_none() && self.mit_fence_trip.is_none() }

    /// Read-only status polls: never while the fence is armed; after a trip only once the
    /// operator cleared it and its hold-off has passed (whether a poll refreshes the motor
    /// watchdog is not verified, so elapsed time alone never resumes them).
    pub(in crate::driver::can_servo) fn poll_tx_allowed(&self) -> bool {
        match &self.mit_fence_trip {
            Some((_, at)) => self.mit_fence_acked && self.since(*at) >= self.mit_fence_holdoff,
            None => self.mit_fence.is_none(),
        }
    }

    /// Remaining frames allowed in the current budgeted RX pass (defaults to 64 when unbudgeted,
    /// tightened by the active control release ledger when `control_release_managed`).
    pub(in crate::driver::can_servo) fn remaining_rx_frame_limit(&self) -> usize {
        let base = self
            .rx_frame_limit
            .unwrap_or(crate::rt_core::ops::RX_FRAME_BUDGET)
            .min(crate::rt_core::ops::RX_FRAME_BUDGET);
        usize::from(if self.in_active_control_release() {
            base.min(self.driver_remaining_release_rx())
        } else {
            base
        })
    }

    /// Account frames consumed by an RX pass (including discarded frames) into `last_poll_rx` and
    /// active `rx_frame_limit` (`release_rx_frames` is charged once per frame at the single
    /// `receive_with_rx_time_timed` convergence point).
    pub(in crate::driver::can_servo) fn account_rx_pass(&mut self, frames_processed: usize, exhausted: bool) {
        let frames = u16::try_from(frames_processed).unwrap_or(u16::MAX);
        if let Some(rem) = self.rx_frame_limit.as_mut() {
            *rem = rem.saturating_sub(frames);
        }
        let (prev_frames, prev_exhausted) = self.last_poll_rx.unwrap_or((0, false));
        self.last_poll_rx = Some((
            prev_frames.saturating_add(frames),
            prev_exhausted || exhausted,
        ));
    }

    /// Drain up to the remaining `rx_frame_limit` (max 64) RX frames within `rx_over` and
    /// `deadline`, stopping on `Timeout` / `WouldBlock` and marking the empty-queue boundary.
    /// Every frame consumed counts against the release's RX budget and `last_poll_rx`.
    pub(in crate::driver::can_servo) fn drain_rx_for_boundary(&mut self, deadline: Deadline) -> bool {
        let max_frames = self.remaining_rx_frame_limit();
        self.last_read_drained = false;
        let t0 = self.now();
        let mut frames_processed = 0usize;
        let mut rx_drained = false;
        let mut hit_err = false;
        while frames_processed < max_frames && !self.rx_over(t0) && !deadline.passed(self.now()) {
            let attempt = self.now();
            match self.receive_with_rx_time_timed() {
                Ok((frame, rx_time)) => {
                    frames_processed += 1;
                    self.process_rx_frame(&frame, true, rx_time);
                }
                Err(TransportError::Timeout) | Err(TransportError::WouldBlock) => {
                    rx_drained = true;
                    self.mark_rx_empty(attempt);
                    break;
                }
                Err(_) => {
                    hit_err = true;
                    break;
                }
            }
        }
        self.account_rx_pass(frames_processed, !rx_drained && !hit_err);
        rx_drained
    }

    #[inline]
    pub(in crate::driver::can_servo) fn damiao_command_motor_index(&self, frame: &CanFrame) -> Option<usize> {
        if frame.is_extended || frame.is_rtr || frame.dlc != 8 || frame.id == 0x7FF {
            return None;
        }
        if let Some(pf_idx) = self.frame_pos_force_joint(frame) {
            return Some(pf_idx);
        }
        self.motor_ids.iter().position(|&id| {
            let base = u32::from(id);
            frame.id == base || frame.id == (0x100 + base)
        })
    }

    #[inline]
    pub(in crate::driver::can_servo) fn capture_rx_clock_sample(&self, mono_now: Instant) -> (CanRxTime, Option<i128>) {
        if let Some(sample) = self.can.rx_clock_sample() {
            (sample.rx_now, sample.realtime_minus_monotonic_ns)
        } else {
            (CanRxTime::Monotonic(mono_now), None)
        }
    }

    /// A command acknowledgement must be separated from buffered pre-command telemetry.
    /// Drained telemetry frames update the feedback cache so pre-write checks (state 0,
    /// stationarity, position) reflect the newest bus state; stale register replies are
    /// ignored so they cannot satisfy a subsequent register transaction.
    pub(in crate::driver::can_servo) fn drain_rx_before_command(&mut self) -> bool {
        self.drain_rx_before_command_limit(128)
    }

    pub(in crate::driver::can_servo) fn drain_rx_before_command_limit(&mut self, max_frames: usize) -> bool {
        let base = self
            .rx_frame_limit
            .map_or(max_frames, |rem| max_frames.min(usize::from(rem)));
        let limit = if self.in_active_control_release() {
            base.min(usize::from(self.driver_remaining_release_rx()))
        } else {
            base
        };
        self.last_read_drained = false;
        let start = self.now();
        let mut effective_deadline = start + Duration::from_millis(5);
        if let Some(tx_dl) = self.tx_release_deadline {
            effective_deadline = effective_deadline.min(tx_dl);
        }
        if let Some(rx_dl) = self.rx_deadline {
            effective_deadline = effective_deadline.min(rx_dl);
        }
        let mut frames_processed = 0usize;
        let mut rx_drained = false;
        let mut hit_err = false;
        while frames_processed < limit {
            let attempt = self.now();
            if attempt >= effective_deadline {
                break;
            }
            match self.receive_with_rx_time_timed() {
                Ok((frame, rx_time)) => {
                    frames_processed += 1;
                    self.process_rx_frame(&frame, false, rx_time);
                }
                Err(TransportError::Timeout) | Err(TransportError::WouldBlock) => {
                    rx_drained = true;
                    self.mark_rx_empty(attempt);
                    break;
                }
                Err(_) => {
                    hit_err = true;
                    break;
                }
            }
        }
        if self.rx_frame_limit.is_some() {
            self.account_rx_pass(frames_processed, !rx_drained && !hit_err);
        }
        rx_drained
    }

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
