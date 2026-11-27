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
