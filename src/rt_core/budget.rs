//! Per-release mutable resource ledger ([`ReleaseBudget`]) and tightened sub-budgets
//! ([`SubBudget`]) shared across hardware polling, sub-release transition wakes, mode switching,
//! commissioning, recovery, and actuator output.
//!
//! One [`ReleaseBudget`] is owned by [`super::ops::ReleaseEngine`] per release (and constructed
//! per tick on the legacy owner and SAFE-HOLD loop for full parity). Every consumer draws from
//! the same ledger, sub-budgets may **only tighten** limits (never widen deadline, RX frame cap,
//! or TX attempt cap), and transport-level RX frames (including discarded frames) and TX attempts
//! are charged once where the transport consumes them.

use std::time::{Duration, Instant};

use super::ops::{ServiceBudget, RX_FRAME_BUDGET, RX_TIME_BUDGET};

/// Default per-release TX attempt allowance matching [`crate::driver::DAMIAO_TX_MAX_FRAMES`].
pub const DEFAULT_TX_ATTEMPT_BUDGET: u16 = 16;

/// Single-release mutable ledger for RX frames, TX attempts, and the monotonic release deadline.
///
/// `Copy` and zero-allocation so it lives inline on [`super::ops::ReleaseEngine`] and crosses
/// release phases without touching the heap or any lock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReleaseBudget {
    release_seq: u64,
    started_at: Instant,
    deadline: Instant,
    rx_limit: u16,
    rx_used: u16,
    rx_exhausted: bool,
    tx_limit: u16,
    tx_used: u16,
    tx_exhausted: bool,
}

impl ReleaseBudget {
    /// Creates a new per-release ledger with explicit RX frame, TX attempt, and deadline bounds.
    #[inline]
    pub const fn new(
        release_seq: u64,
        started_at: Instant,
        deadline: Instant,
        rx_limit: u16,
        tx_limit: u16,
    ) -> Self {
        Self {
            release_seq,
            started_at,
            deadline,
            rx_limit,
            rx_used: 0,
            rx_exhausted: rx_limit == 0,
            tx_limit,
            tx_used: 0,
            tx_exhausted: tx_limit == 0,
        }
    }

    /// Creates a per-release ledger with the default RX frame cap ([`RX_FRAME_BUDGET`]) and
    /// default TX attempt cap ([`DEFAULT_TX_ATTEMPT_BUDGET`]).
    #[inline]
    pub const fn for_release(
        release_seq: u64,
        started_at: Instant,
        release_deadline: Instant,
    ) -> Self {
        Self::new(
            release_seq,
            started_at,
            release_deadline,
            RX_FRAME_BUDGET,
            DEFAULT_TX_ATTEMPT_BUDGET,
        )
    }

    /// Creates a per-release ledger with the default RX frame cap ([`RX_FRAME_BUDGET`]) and a
    /// caller-configured TX attempt cap.
    #[inline]
    pub const fn for_release_with_tx(
        release_seq: u64,
        started_at: Instant,
        release_deadline: Instant,
        tx_limit: u16,
    ) -> Self {
        Self::new(
            release_seq,
            started_at,
            release_deadline,
            RX_FRAME_BUDGET,
            tx_limit,
        )
    }

    /// Release sequence number this ledger belongs to (`0` for standalone / ad-hoc steps).
    #[inline]
    pub const fn release_seq(&self) -> u64 {
        self.release_seq
    }

    /// Monotonic start instant of the release.
    #[inline]
    pub const fn started_at(&self) -> Instant {
        self.started_at
    }

    /// Current monotonic deadline of the release (may only move earlier via tightening).
    #[inline]
    pub const fn deadline(&self) -> Instant {
        self.deadline
    }

    /// Total RX frame cap for this release.
    #[inline]
    pub const fn rx_limit(&self) -> u16 {
        self.rx_limit
    }

    /// Cumulative RX frames consumed in this release (including discarded RX frames).
    #[inline]
    pub const fn rx_used(&self) -> u16 {
        self.rx_used
    }

    /// Remaining RX frames available in this release.
    #[inline]
    pub const fn rx_remaining(&self) -> u16 {
        self.rx_limit.saturating_sub(self.rx_used)
    }

    /// True if RX frames or a transport RX pass exhausted this ledger's RX budget.
    #[inline]
    pub const fn rx_exhausted(&self) -> bool {
        self.rx_exhausted || self.rx_used >= self.rx_limit
    }

    /// Total budgeted TX attempt cap for this release.
    #[inline]
    pub const fn tx_limit(&self) -> u16 {
        self.tx_limit
    }

    /// Cumulative budgeted TX attempts started in this release.
    #[inline]
    pub const fn tx_used(&self) -> u16 {
        self.tx_used
    }

    /// Remaining budgeted TX attempts available in this release.
    #[inline]
    pub const fn tx_remaining(&self) -> u16 {
        self.tx_limit.saturating_sub(self.tx_used)
    }

    /// True if budgeted TX attempts exhausted this ledger's TX budget.
    #[inline]
    pub const fn tx_exhausted(&self) -> bool {
        self.tx_exhausted || self.tx_used >= self.tx_limit
    }

    /// True when `now` has reached or passed the release deadline.
    #[inline]
    pub fn deadline_exceeded(&self, now: Instant) -> bool {
        now >= self.deadline
    }

    /// True when at least one RX frame and positive wall-time headroom remain.
    #[inline]
    pub fn can_rx(&self, now: Instant) -> bool {
        self.rx_remaining() > 0 && now < self.deadline
    }

    /// True when at least one budgeted TX attempt and positive wall-time headroom remain.
    #[inline]
    pub fn can_tx(&self, now: Instant) -> bool {
        self.tx_remaining() > 0 && now < self.deadline
    }

    /// Monotonically tightens the deadline (`deadline = min(self.deadline, deadline)`).
    /// Never loosens an already earlier deadline.
    #[inline]
    pub fn tighten_deadline(&mut self, deadline: Instant) {
        if deadline < self.deadline {
            self.deadline = deadline;
        }
    }

    /// Monotonically tightens the remaining RX allowance to at most `max_remaining_rx` frames.
    /// Never increases `rx_limit`.
    #[inline]
    pub fn tighten_rx_remaining(&mut self, max_remaining_rx: u16) {
        let tightened = self.rx_used.saturating_add(max_remaining_rx);
        if tightened < self.rx_limit {
            self.rx_limit = tightened;
        }
        if self.rx_used >= self.rx_limit {
            self.rx_exhausted = true;
        }
    }

    /// Monotonically tightens the remaining TX allowance to at most `max_remaining_tx` attempts.
    /// Never increases `tx_limit`.
    #[inline]
    pub fn tighten_tx_remaining(&mut self, max_remaining_tx: u16) {
        let tightened = self.tx_used.saturating_add(max_remaining_tx);
        if tightened < self.tx_limit {
            self.tx_limit = tightened;
        }
        if self.tx_used >= self.tx_limit {
            self.tx_exhausted = true;
        }
    }

    /// Charges `frames` consumed RX frames (including discarded frames) to the ledger.
    #[inline]
    pub fn charge_rx(&mut self, frames: u16, exhausted: bool) {
        self.rx_used = self.rx_used.saturating_add(frames);
        if exhausted || self.rx_used >= self.rx_limit {
            self.rx_exhausted = true;
        }
    }

    /// Attempts to reserve and charge one RX frame at `now`. Returns `false` and marks RX
    /// exhausted if the deadline has passed or no RX frame allowance remains.
    #[inline]
    pub fn try_charge_rx_one(&mut self, now: Instant) -> bool {
        if now >= self.deadline || self.rx_used >= self.rx_limit {
            self.rx_exhausted = true;
            return false;
        }
        self.rx_used = self.rx_used.saturating_add(1);
        if self.rx_used >= self.rx_limit {
            self.rx_exhausted = true;
        }
        true
    }

    /// Charges `attempts` budgeted TX attempts to the ledger.
    #[inline]
    pub fn charge_tx(&mut self, attempts: u16) {
        self.tx_used = self.tx_used.saturating_add(attempts);
        if self.tx_used >= self.tx_limit {
            self.tx_exhausted = true;
        }
    }

    /// Attempts to reserve and charge one budgeted TX attempt at `now`. Returns `false` and
    /// marks TX exhausted if the deadline has passed or no TX attempt allowance remains.
    #[inline]
    pub fn try_charge_tx_one(&mut self, now: Instant) -> bool {
        if now >= self.deadline || self.tx_used >= self.tx_limit {
            self.tx_exhausted = true;
            return false;
        }
        self.tx_used = self.tx_used.saturating_add(1);
        if self.tx_used >= self.tx_limit {
            self.tx_exhausted = true;
        }
        true
    }

    /// Marks the RX portion of this ledger exhausted (e.g. time budget hit before queue drained).
    #[inline]
    pub fn mark_rx_exhausted(&mut self) {
        self.rx_exhausted = true;
    }

    /// Marks the TX portion of this ledger exhausted (e.g. cutoff or backpressure).
    #[inline]
    pub fn mark_tx_exhausted(&mut self) {
        self.tx_exhausted = true;
    }

    /// Derives a tightened [`ServiceBudget`] for a single hardware poll pass at `now` bounded by
    /// `max_duration` and `max_rx` frames, drawing automatically from `self.rx_remaining()` and
    /// `self.deadline()`.
    #[inline]
    pub fn service_budget(&self, now: Instant, max_duration: Duration, max_rx: u16) -> ServiceBudget {
        ServiceBudget {
            now,
            deadline: (now + max_duration).min(self.deadline),
            max_rx_frames: self.rx_remaining().min(max_rx),
        }
    }

    /// Convenience helper using the standard [`RX_TIME_BUDGET`] and [`RX_FRAME_BUDGET`] caps,
    /// tightened against this ledger's remaining RX allowance and release deadline.
    #[inline]
    pub fn poll_service_budget(&self, now: Instant) -> ServiceBudget {
        self.service_budget(now, RX_TIME_BUDGET, RX_FRAME_BUDGET)
    }

    /// Creates a tightened scoped [`SubBudget`] that draws from and charges back into `self`.
    ///
    /// A sub-budget can only tighten limits:
    /// - `deadline = min(self.deadline, deadline)`
    /// - `rx_cap = min(self.rx_remaining(), max_rx)`
    /// - `tx_cap = min(self.tx_remaining(), max_tx)`
    #[inline]
    pub fn sub_budget(&mut self, deadline: Instant, max_rx: u16, max_tx: u16) -> SubBudget<'_> {
        let clamped_deadline = self.deadline.min(deadline);
        let rx_start = self.rx_used;
        let tx_start = self.tx_used;
        let rx_ceiling = self.rx_limit.min(rx_start.saturating_add(max_rx));
        let tx_ceiling = self.tx_limit.min(tx_start.saturating_add(max_tx));
        SubBudget {
            parent: self,
            deadline: clamped_deadline,
            rx_start,
            rx_ceiling,
            tx_start,
            tx_ceiling,
        }
    }
}

/// Tightened sub-budget view drawing from a parent [`ReleaseBudget`] ledger.
///
/// Scoped ceilings (`rx_ceiling`, `tx_ceiling`) are anchored to absolute root-ledger consumption
/// counters (`parent.rx_used`, `parent.tx_used`), so any charge made through a nested
/// [`SubBudget::tighten`] view automatically debits every enclosing sub-budget as well as the
/// root [`ReleaseBudget`].
#[derive(Debug)]
pub struct SubBudget<'a> {
    parent: &'a mut ReleaseBudget,
    deadline: Instant,
    rx_start: u16,
    rx_ceiling: u16,
    tx_start: u16,
    tx_ceiling: u16,
}

impl<'a> SubBudget<'a> {
    /// Effective deadline of this sub-budget (`<= parent.deadline()`).
    #[inline]
    pub const fn deadline(&self) -> Instant {
        self.deadline
    }

    /// Remaining RX frames available to this sub-budget (`<= parent.rx_remaining()`).
    #[inline]
    pub fn rx_remaining(&self) -> u16 {
        self.rx_ceiling
            .min(self.parent.rx_limit)
            .saturating_sub(self.parent.rx_used)
    }

    /// Remaining TX attempts available to this sub-budget (`<= parent.tx_remaining()`).
    #[inline]
    pub fn tx_remaining(&self) -> u16 {
        self.tx_ceiling
            .min(self.parent.tx_limit)
            .saturating_sub(self.parent.tx_used)
    }

    /// Cumulative RX frames consumed from the root ledger since this sub-budget was opened
    /// (including charges through nested sub-budgets).
    #[inline]
    pub fn rx_used(&self) -> u16 {
        self.parent.rx_used.saturating_sub(self.rx_start)
    }

    /// Cumulative TX attempts consumed from the root ledger since this sub-budget was opened
    /// (including charges through nested sub-budgets).
    #[inline]
    pub fn tx_used(&self) -> u16 {
        self.parent.tx_used.saturating_sub(self.tx_start)
    }

    /// True when `now` has reached or passed this sub-budget's deadline.
    #[inline]
    pub fn deadline_exceeded(&self, now: Instant) -> bool {
        now >= self.deadline
    }

    /// True when at least one RX frame and positive time headroom remain.
    #[inline]
    pub fn can_rx(&self, now: Instant) -> bool {
        self.rx_remaining() > 0 && now < self.deadline
    }

    /// True when at least one TX attempt and positive time headroom remain.
    #[inline]
    pub fn can_tx(&self, now: Instant) -> bool {
        self.tx_remaining() > 0 && now < self.deadline
    }

    /// Further tightens this sub-budget's deadline (never loosens).
    #[inline]
    pub fn tighten_deadline(&mut self, deadline: Instant) {
        if deadline < self.deadline {
            self.deadline = deadline;
        }
    }

    /// Further tightens this sub-budget's remaining RX cap (never loosens).
    #[inline]
    pub fn tighten_rx_remaining(&mut self, max_remaining_rx: u16) {
        let tightened = self.parent.rx_used.saturating_add(max_remaining_rx);
        if tightened < self.rx_ceiling {
            self.rx_ceiling = tightened;
        }
    }

    /// Further tightens this sub-budget's remaining TX cap (never loosens).
    #[inline]
    pub fn tighten_tx_remaining(&mut self, max_remaining_tx: u16) {
        let tightened = self.parent.tx_used.saturating_add(max_remaining_tx);
        if tightened < self.tx_ceiling {
            self.tx_ceiling = tightened;
        }
    }

    /// Charges `frames` RX frames to the root ledger (immediately reflected in this and all
    /// enclosing sub-budgets).
    #[inline]
    pub fn charge_rx(&mut self, frames: u16, exhausted: bool) {
        self.parent.charge_rx(frames, exhausted);
    }

    /// Attempts to charge one RX frame to the root ledger within this sub-budget's bounds.
    #[inline]
    pub fn try_charge_rx_one(&mut self, now: Instant) -> bool {
        if now >= self.deadline || self.rx_remaining() == 0 {
            if self.parent.rx_remaining() == 0 || now >= self.parent.deadline {
                self.parent.mark_rx_exhausted();
            }
            return false;
        }
        self.parent.charge_rx(1, false);
        true
    }

    /// Charges `attempts` TX attempts to the root ledger (immediately reflected in this and all
    /// enclosing sub-budgets).
    #[inline]
    pub fn charge_tx(&mut self, attempts: u16) {
        self.parent.charge_tx(attempts);
    }

    /// Attempts to charge one TX attempt to the root ledger within this sub-budget's bounds.
    #[inline]
    pub fn try_charge_tx_one(&mut self, now: Instant) -> bool {
        if now >= self.deadline || self.tx_remaining() == 0 {
            if self.parent.tx_remaining() == 0 || now >= self.parent.deadline {
                self.parent.mark_tx_exhausted();
            }
            return false;
        }
        self.parent.charge_tx(1);
        true
    }

    /// Creates a nested tightened [`SubBudget`] drawing from the same root [`ReleaseBudget`].
    /// Any charge through the nested view advances `parent.rx_used` / `parent.tx_used`, so
    /// enclosing sub-budgets automatically enforce their own ceilings when the nested view drops.
    #[inline]
    pub fn tighten(&mut self, deadline: Instant, max_rx: u16, max_tx: u16) -> SubBudget<'_> {
        let clamped_deadline = self.deadline.min(deadline);
        let rx_start = self.parent.rx_used;
        let tx_start = self.parent.tx_used;
        let rx_ceiling = self
            .rx_ceiling
            .min(self.parent.rx_limit)
            .min(rx_start.saturating_add(max_rx));
        let tx_ceiling = self
            .tx_ceiling
            .min(self.parent.tx_limit)
            .min(tx_start.saturating_add(max_tx));
        SubBudget {
            parent: self.parent,
            deadline: clamped_deadline,
            rx_start,
            rx_ceiling,
            tx_start,
            tx_ceiling,
        }
    }

    /// Read-only view of the parent ledger.
    #[inline]
    pub fn parent(&self) -> &ReleaseBudget {
        self.parent
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sub_budget_may_only_tighten_and_charges_parent_ledger() {
        let t0 = Instant::now();
        let dl = t0 + Duration::from_millis(20);
        let mut ledger = ReleaseBudget::new(1, t0, dl, 10, 6);

        // Requesting wider deadline, RX, or TX than parent must clamp to parent's bounds.
        {
            let mut sub = ledger.sub_budget(dl + Duration::from_secs(5), 64, 32);
            assert_eq!(sub.deadline(), dl);
            assert_eq!(sub.rx_remaining(), 10);
            assert_eq!(sub.tx_remaining(), 6);

            // Tighten further inside the sub-budget.
            sub.tighten_deadline(t0 + Duration::from_millis(12));
            sub.tighten_deadline(t0 + Duration::from_millis(18)); // ignored (looser)
            assert_eq!(sub.deadline(), t0 + Duration::from_millis(12));

            sub.tighten_rx_remaining(4);
            sub.tighten_rx_remaining(9); // ignored (looser)
            assert_eq!(sub.rx_remaining(), 4);

            sub.tighten_tx_remaining(3);
            sub.tighten_tx_remaining(5); // ignored (looser)
            assert_eq!(sub.tx_remaining(), 3);

            assert!(sub.try_charge_rx_one(t0));
            assert!(sub.try_charge_rx_one(t0));
            sub.charge_rx(2, false);
            assert_eq!(sub.rx_remaining(), 0);
            assert!(!sub.try_charge_rx_one(t0));

            sub.charge_tx(2);
            assert!(sub.try_charge_tx_one(t0));
            assert_eq!(sub.tx_remaining(), 0);
            assert!(!sub.try_charge_tx_one(t0));
        }

        // Parent ledger reflects exact sub-budget consumption automatically.
        assert_eq!(ledger.rx_used(), 4);
        assert_eq!(ledger.rx_remaining(), 6);
        assert!(!ledger.rx_exhausted());
        assert_eq!(ledger.tx_used(), 3);
        assert_eq!(ledger.tx_remaining(), 3);
        assert!(!ledger.tx_exhausted());

        // Subsequent phase sees only remaining RX/TX.
        let sb = ledger.poll_service_budget(t0);
        assert_eq!(sb.max_rx_frames, 6);
        assert_eq!(sb.deadline, t0 + RX_TIME_BUDGET);

        // Exhausting the rest marks ledger exhausted.
        ledger.charge_rx(6, false);
        assert_eq!(ledger.rx_remaining(), 0);
        assert!(ledger.rx_exhausted());
        assert!(!ledger.can_rx(t0));

        ledger.charge_tx(3);
        assert_eq!(ledger.tx_remaining(), 0);
        assert!(ledger.tx_exhausted());
        assert!(!ledger.can_tx(t0));
    }

    #[test]
    fn deadline_tightening_is_monotonic_and_blocks_rx_tx_past_deadline() {
        let t0 = Instant::now();
        let dl = t0 + Duration::from_millis(10);
        let mut ledger = ReleaseBudget::for_release(2, t0, dl);
        ledger.tighten_deadline(t0 + Duration::from_millis(5));
        ledger.tighten_deadline(t0 + Duration::from_millis(15)); // must NOT widen
        assert_eq!(ledger.deadline(), t0 + Duration::from_millis(5));
        assert!(ledger.can_rx(t0 + Duration::from_millis(4)));
        assert!(ledger.can_tx(t0 + Duration::from_millis(4)));
        assert!(!ledger.can_rx(t0 + Duration::from_millis(5)));
        assert!(!ledger.can_tx(t0 + Duration::from_millis(5)));
        assert!(!ledger.try_charge_rx_one(t0 + Duration::from_millis(5)));
        assert!(!ledger.try_charge_tx_one(t0 + Duration::from_millis(5)));
        assert_eq!(ledger.rx_used(), 0);
        assert_eq!(ledger.tx_used(), 0);
    }

    #[test]
    fn nested_sub_budget_charges_propagate_to_enclosing_and_sibling_scopes() {
        let t0 = Instant::now();
        let dl = t0 + Duration::from_millis(20);
        let mut root = ReleaseBudget::new(3, t0, dl, 10, 10);

        {
            let mut outer = root.sub_budget(dl, 2, 2);
            assert_eq!(outer.rx_remaining(), 2);
            assert_eq!(outer.tx_remaining(), 2);

            // Nested child consumes outer's entire RX and TX allowance (2 each).
            {
                let mut inner = outer.tighten(dl, 2, 2);
                assert!(inner.try_charge_rx_one(t0));
                assert!(inner.try_charge_rx_one(t0));
                assert!(!inner.try_charge_rx_one(t0));
                assert!(inner.try_charge_tx_one(t0));
                assert!(inner.try_charge_tx_one(t0));
                assert!(!inner.try_charge_tx_one(t0));
            }

            // Enclosing sub-budget must now observe 0 remaining RX and 0 remaining TX.
            assert_eq!(outer.rx_used(), 2);
            assert_eq!(outer.tx_used(), 2);
            assert_eq!(outer.rx_remaining(), 0);
            assert_eq!(outer.tx_remaining(), 0);
            assert!(!outer.try_charge_rx_one(t0));
            assert!(!outer.try_charge_tx_one(t0));

            // A sibling nested scope opened from `outer` must also see 0 remaining.
            {
                let mut sibling = outer.tighten(dl, 5, 5);
                assert_eq!(sibling.rx_remaining(), 0);
                assert_eq!(sibling.tx_remaining(), 0);
                assert!(!sibling.try_charge_rx_one(t0));
                assert!(!sibling.try_charge_tx_one(t0));
            }
        }

        // Root ledger reflects exact total consumption (2 RX, 2 TX) with 8 remaining each.
        assert_eq!(root.rx_used(), 2);
        assert_eq!(root.tx_used(), 2);
        assert_eq!(root.rx_remaining(), 8);
        assert_eq!(root.tx_remaining(), 8);

        // Partial consumption across nested + direct + sibling scopes sums against outer cap.
        {
            let mut outer = root.sub_budget(dl, 3, 3);
            {
                let mut child1 = outer.tighten(dl, 2, 2);
                child1.charge_rx(1, false);
                child1.charge_tx(1);
            }
            assert_eq!(outer.rx_remaining(), 2);
            assert_eq!(outer.tx_remaining(), 2);
            assert!(outer.try_charge_rx_one(t0));
            assert!(outer.try_charge_tx_one(t0));
            {
                let mut child2 = outer.tighten(dl, 5, 5);
                assert_eq!(child2.rx_remaining(), 1);
                assert_eq!(child2.tx_remaining(), 1);
                assert!(child2.try_charge_rx_one(t0));
                assert!(!child2.try_charge_rx_one(t0));
                assert!(child2.try_charge_tx_one(t0));
                assert!(!child2.try_charge_tx_one(t0));
            }
            assert_eq!(outer.rx_remaining(), 0);
            assert_eq!(outer.tx_remaining(), 0);
        }
        assert_eq!(root.rx_used(), 5);
        assert_eq!(root.tx_used(), 5);
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn legacy_sync_driver_budget_before_under_contention_no_double_charge_and_shared_comm_ledger() {
        use std::sync::{
            atomic::{AtomicU16, Ordering},
            Arc,
        };
        use crate::driver::{ArmDriver, ArmDriverError, DriverCapabilities, JointTarget};
        use crate::model::{ArmConfig, JointState};
        use crate::supervisor::SupervisorInner;

        struct StubBudgetDriver {
            rx_consumed: Arc<AtomicU16>,
            tx_consumed: Arc<AtomicU16>,
            last_synced_rx: Arc<AtomicU16>,
            last_synced_tx: Arc<AtomicU16>,
            last_poll_max_rx: Arc<AtomicU16>,
            set_zero_calls: Arc<AtomicU16>,
            dof: usize,
        }

        impl ArmDriver for StubBudgetDriver {
            fn capabilities(&self) -> DriverCapabilities {
                DriverCapabilities::default()
            }
            fn set_torque(&mut self, _enabled: bool) -> Result<(), ArmDriverError> {
                Ok(())
            }
            fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
                self.rx_consumed.fetch_add(3, Ordering::SeqCst);
                self.tx_consumed.fetch_add(2, Ordering::SeqCst);
                Ok(vec![JointState::default(); self.dof])
            }
            fn read_joint_states_budgeted(
                &mut self,
                out: &mut Vec<JointState>,
                _deadline: Instant,
                max_rx_frames: u16,
            ) -> Result<(u16, bool), ArmDriverError> {
                self.last_poll_max_rx.store(max_rx_frames, Ordering::SeqCst);
                out.clear();
                out.resize(self.dof, JointState::default());
                let used = max_rx_frames.min(2);
                self.rx_consumed.fetch_add(used, Ordering::SeqCst);
                Ok((used, max_rx_frames == 0))
            }
            fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> {
                self.tx_consumed.fetch_add(2, Ordering::SeqCst);
                Ok(())
            }
            fn sync_release_budget_limits(&mut self, rx_rem: u16, tx_rem: u16) {
                self.last_synced_rx.store(rx_rem, Ordering::SeqCst);
                self.last_synced_tx.store(tx_rem, Ordering::SeqCst);
            }
            fn release_budget_consumed(&self) -> (u16, u16) {
                (
                    self.rx_consumed.load(Ordering::SeqCst),
                    self.tx_consumed.load(Ordering::SeqCst),
                )
            }
            fn tx_budget_max_frames(&self) -> Option<u16> {
                Some(16)
            }
            fn supports_nonblocking_commissioning(&self) -> bool {
                true
            }
            fn supports_nonblocking_motor_zero(&self) -> bool {
                true
            }
            fn commissioning_motor_id(&self, joint: usize) -> Option<u16> {
                Some((joint + 1) as u16)
            }
            fn commissioning_drain_rx_step(
                &mut self,
                max_frames: u16,
                _deadline: Instant,
            ) -> Result<(u16, bool), ArmDriverError> {
                let used = max_frames.min(4);
                self.rx_consumed.fetch_add(used, Ordering::SeqCst);
                Ok((used, true))
            }
            fn commissioning_try_send_register_read(
                &mut self,
                _joint: usize,
                _rid: u8,
                _deadline: Instant,
                _pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
            ) -> crate::mit::CommissioningTxStatus {
                self.tx_consumed.fetch_add(1, Ordering::SeqCst);
                crate::mit::CommissioningTxStatus::Sent {
                    tx_seq: 1,
                    tx_started: Instant::now(),
                    tx_offset_ns: None,
                }
            }
            fn commissioning_try_send_set_zero(
                &mut self,
                _joint: usize,
                _deadline: Instant,
                _pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
            ) -> crate::mit::CommissioningTxStatus {
                self.set_zero_calls.fetch_add(1, Ordering::SeqCst);
                self.tx_consumed.fetch_add(1, Ordering::SeqCst);
                crate::mit::CommissioningTxStatus::Refused("stub set-zero")
            }
            fn commissioning_raw_feedback(
                &self,
                _joint: usize,
            ) -> Option<crate::mit::CommissioningFeedback> {
                let now = Instant::now();
                Some(crate::mit::CommissioningFeedback {
                    raw_position: 0.0,
                    joint_position: 0.0,
                    velocity: 0.0,
                    motor_status: 0,
                    rx_seq: 10,
                    rx_at: now,
                    queue_bound: Some(now),
                })
            }
        }

        let rx_consumed = Arc::new(AtomicU16::new(5));
        let tx_consumed = Arc::new(AtomicU16::new(3));
        let last_synced_rx = Arc::new(AtomicU16::new(u16::MAX));
        let last_synced_tx = Arc::new(AtomicU16::new(u16::MAX));
        let last_poll_max_rx = Arc::new(AtomicU16::new(u16::MAX));
        let set_zero_calls = Arc::new(AtomicU16::new(0));
        let cfg = ArmConfig::rebot_dm();
        let dof = cfg.dof;
        let drv = StubBudgetDriver {
            rx_consumed: Arc::clone(&rx_consumed),
            tx_consumed: Arc::clone(&tx_consumed),
            last_synced_rx: Arc::clone(&last_synced_rx),
            last_synced_tx: Arc::clone(&last_synced_tx),
            last_poll_max_rx: Arc::clone(&last_poll_max_rx),
            set_zero_calls: Arc::clone(&set_zero_calls),
            dof,
        };
        let mut core = SupervisorInner::new(cfg).with_driver(Box::new(drv));

        // 1. Contended `sync_driver_budget_before` MUST return `None` (never `(0, 0)`), and
        //    `charge_driver_budget_after(..., None, ...)` MUST NOT post-charge cumulative driver
        //    counters once contention clears.
        let t0 = Instant::now();
        let mut ledger = ReleaseBudget::new(500, t0, t0 + Duration::from_millis(20), 16, 16);
        ledger.charge_rx(5, false);
        ledger.charge_tx(3);

        let before_contended = core.with_contended_driver_for_test(|c| {
            let b = c.sync_driver_budget_before(&ledger);
            // Also verify `poll_hardware_with_budget_and_clock` under contention does not double-charge.
            let mut now_fn = || t0;
            let _ = c.poll_hardware_with_budget_and_clock(&mut ledger, &mut now_fn);
            b
        });
        assert_eq!(
            before_contended, None,
            "contended sync_driver_budget_before must return None, not (0, 0)"
        );
        // Now lock is uncontended again: calling `charge_driver_budget_after` with `None` must NOT
        // read cumulative `(5, 3)` from the driver and double-charge into `ledger`.
        core.charge_driver_budget_after(&mut ledger, before_contended, 0, false, 0);
        assert_eq!(
            (ledger.rx_used(), ledger.tx_used()),
            (5, 3),
            "failed pre-snapshot must not double-charge cumulative driver usage"
        );

        // Uncontended snapshot returns `Some((5, 3))` and charges only the incremental delta.
        let before_ok = core.sync_driver_budget_before(&ledger);
        assert_eq!(before_ok, Some((5, 3)));
        assert_eq!(last_synced_rx.load(Ordering::SeqCst), 11);
        assert_eq!(last_synced_tx.load(Ordering::SeqCst), 13);
        rx_consumed.fetch_add(2, Ordering::SeqCst);
        tx_consumed.fetch_add(1, Ordering::SeqCst);
        core.charge_driver_budget_after(&mut ledger, before_ok, 0, false, 0);
        assert_eq!((ledger.rx_used(), ledger.tx_used()), (7, 4));

        // 2. Legacy tick saves its `ReleaseBudget` (`last_legacy_release_budget`) reflecting the
        //    tick's RX/TX usage so post-tick commissioning draws from the exact same ledger.
        core.pending_rx_poll.active = false;
        rx_consumed.store(0, Ordering::SeqCst);
        tx_consumed.store(0, Ordering::SeqCst);
        core.defer_legacy_commissioning = true;
        let t1 = Instant::now();
        let mut clock_now = || t1;
        let _ = core.tick_control_legacy_with_clock(0.02, &mut clock_now);
        let mut post_ledger = core
            .last_legacy_release_budget
            .take()
            .expect("legacy tick must populate last_legacy_release_budget for post-tick commissioning");
        assert_eq!(
            post_ledger.rx_used(),
            3,
            "shared legacy ledger must carry forward tick RX consumption"
        );
        assert_eq!(
            post_ledger.tx_used(),
            2,
            "shared legacy ledger must carry forward tick TX consumption"
        );
        // Feeding that same ledger into `service_commissioning_step_with_budget` synchronizes the
        // remaining tick budget (not a fresh `RX_FRAME_BUDGET`) to the driver, and under
        // contention does not double-charge cumulative usage.
        let ctx = crate::rt_core::ops::CommitCtx {
            arm: core.core.id(),
            boot: crate::rt_core::broker::BootId::process(),
            now: t1,
            stop_generation: core.current_stop_gen(),
        };
        let _ = core.service_commissioning_step_with_budget(
            &ctx,
            &mut post_ledger,
            t1 + Duration::from_millis(18),
            1,
            Some(1),
            true,
            &mut clock_now,
        );
        assert_eq!(
            last_synced_rx.load(Ordering::SeqCst),
            RX_FRAME_BUDGET - 3,
            "legacy post-tick commissioning must tighten driver RX from the tick's shared ledger"
        );
        assert_eq!(
            last_synced_tx.load(Ordering::SeqCst),
            16 - 2,
            "legacy post-tick commissioning must tighten driver TX from the tick's shared ledger"
        );
        core.with_contended_driver_for_test(|c| {
            let _ = c.service_commissioning_step_with_budget(
                &ctx,
                &mut post_ledger,
                t1 + Duration::from_millis(18),
                1,
                Some(1),
                false,
                &mut clock_now,
            );
        });
        assert_eq!(
            (post_ledger.rx_used(), post_ledger.tx_used()),
            (3, 2),
            "contended post-tick commissioning must not double-charge cumulative usage"
        );

        // 3. Contention clearing BETWEEN `sync_driver_budget_before` and the operation borrow:
        //    the operation's own driver borrow must install the conservative `(rx=0, tx=0)` cap
        //    AND account the exact incremental TX consumed inside that borrow (neither 0 uncharged
        //    nor cumulative double-charged).
        rx_consumed.store(5, Ordering::SeqCst);
        tx_consumed.store(3, Ordering::SeqCst);
        last_synced_rx.store(u16::MAX, Ordering::SeqCst);
        last_synced_tx.store(u16::MAX, Ordering::SeqCst);
        let mut ledger3 = ReleaseBudget::new(501, t1, t1 + Duration::from_millis(20), 16, 16);
        ledger3.charge_rx(5, false);
        ledger3.charge_tx(3);
        let before_contended_out = core.with_contended_driver_for_test(|c| {
            let b = c.sync_driver_budget_before(&ledger3);
            c.arm_conservative_cap_if_contended(b);
            b
        });
        assert_eq!(before_contended_out, None);
        // Contention has now cleared before the output write runs!
        let mut targets = vec![JointTarget::position(0.0); dof];
        let _ = core.write_output_slice(&mut targets, 0.02, None);
        let (cons_rx, cons_tx) = core.take_conservative_step_deltas();
        core.charge_driver_budget_after(&mut ledger3, before_contended_out, cons_rx, false, cons_tx);
        assert_eq!(
            (
                last_synced_rx.load(Ordering::SeqCst),
                last_synced_tx.load(Ordering::SeqCst)
            ),
            (0, 0),
            "operation borrow after contended pre-snapshot must install conservative (0, 0) cap"
        );
        assert_eq!(
            cons_tx, 2,
            "incremental TX consumed inside borrow after contention cleared must be captured"
        );
        assert_eq!(
            (ledger3.rx_used(), ledger3.tx_used()),
            (5, 5),
            "ledger must charge exact incremental TX (3 + 2 = 5), never 3 (uncharged) or 8 (cumulative)"
        );

        // 4. Single-borrow commissioning budget enforcement:
        //    (a) `ReadIdentity` draining RX inside its single driver borrow charges the exact RX
        //        delta directly into `release_budget` (no post-op second lock window).
        //    (b) `MotorZeroWriteVerify` obeys `tx_remaining == 0` inside the driver borrow and
        //        must NEVER submit `commissioning_try_send_set_zero` when TX budget is exhausted.
        let t2 = Instant::now();
        let mut clock_t2 = || t2;
        core.lease_enabled = false;
        core.operator_auth_enforced = false;
        core.set_active_lease_in_place(1, "c1", "test", 1, t2);
        let auth_id = crate::mit::CommissioningAuthority {
            conn_id: 1,
            lease_holder: Some((crate::model::HolderLabel::new("c1"), 1)),
            operator_grant: crate::auth::OperatorGrant::unenforced(),
            expected_epoch: core.safety.epoch_id(),
            expected_stop_gen: core.current_stop_gen(),
            grant_id: None,
            txn_id: 0,
        };
        let mut joints = [0usize; crate::rt_core::mailbox::MAX_DOF];
        joints[0] = 0;
        let req_id = crate::mit::CommissioningStartRequest {
            kind: crate::mit::CommissioningOpKind::ReadIdentity,
            joints,
            joint_len: 1,
            timeout_counts: 0,
            references: [0.0; crate::rt_core::mailbox::MAX_DOF],
            authority: auth_id,
            txn_deadline: t2 + Duration::from_secs(5),
            prior_disk_mutated: false,
        };
        let op_id = core.start_commissioning_rt(req_id, t2).expect("start ReadIdentity");
        let mut ledger4 = ReleaseBudget::new(502, t2, t2 + Duration::from_millis(20), 10, 4);
        ledger4.charge_rx(2, false);
        ledger4.charge_tx(3); // 1 TX remaining, 8 RX remaining
        last_synced_rx.store(u16::MAX, Ordering::SeqCst);
        last_synced_tx.store(u16::MAX, Ordering::SeqCst);
        let ctx4 = crate::rt_core::ops::CommitCtx {
            arm: core.core.id(),
            boot: crate::rt_core::broker::BootId::process(),
            now: t2,
            stop_generation: core.current_stop_gen(),
        };
        let _ = core.service_commissioning_step_with_budget(
            &ctx4,
            &mut ledger4,
            t2 + Duration::from_millis(18),
            2,
            None,
            true,
            &mut clock_t2,
        );
        assert_eq!(
            (
                last_synced_rx.load(Ordering::SeqCst),
                last_synced_tx.load(Ordering::SeqCst)
            ),
            (8, 1),
            "single-borrow commissioning must tighten driver caps from live release_budget"
        );
        assert_eq!(
            ledger4.tx_used(),
            4,
            "single-borrow commissioning must charge exact TX delta (3 + 1 = 4)"
        );
        // Step 2 (AwaitReply): drains RX inside the single driver borrow and charges exact RX delta.
        let _ = core.service_commissioning_step_with_budget(
            &ctx4,
            &mut ledger4,
            t2 + Duration::from_millis(18),
            2,
            None,
            false,
            &mut clock_t2,
        );
        assert!(
            ledger4.rx_used() > 2,
            "single-borrow commissioning must charge exact RX delta before releasing driver lock, got {}",
            ledger4.rx_used()
        );
        let _ = core.cancel_commissioning_rt(op_id, "test");
        assert!(core.clear_commissioning_rt(op_id, true, false));

        // Now verify `MotorZeroWriteVerify` obeys `tx_remaining == 0` (0 set-zero TX).
        use crate::rt_core::config_iface::{
            CandidateId, ConfigAuthority, ConfigDomain, ConfigProposal, Reserved,
        };
        core.config.resting_pose = None;
        let cal_rev = core.revision(ConfigDomain::Calibration);
        let proposal = ConfigProposal {
            domain: ConfigDomain::Calibration,
            expected_revision: cal_rev,
            candidate: CandidateId(1),
            stamp: crate::rt_core::broker::Stamp {
                arm: ctx4.arm,
                boot: ctx4.boot,
                request: crate::rt_core::broker::RequestId(1),
                client: crate::rt_core::broker::ClientId(1),
                client_seq: 1,
                authority: crate::rt_core::broker::Authority {
                    stop_generation: ctx4.stop_generation,
                    epoch: core.safety.epoch_id(),
                    lease_generation: 0,
                    parent_generation: 0,
                    config_revision: cal_rev,
                },
                deadline: t2 + Duration::from_secs(5),
            },
        };
        let reservation = match core.reserve(&proposal, &ctx4) {
            Reserved::Granted(r) => r,
            other => panic!("expected Calibration reservation, got {other:?}"),
        };
        let grant_id = reservation.grant();
        core.core
            .config_mut()
            .bind_commissioning_marker(grant_id)
            .expect("bind marker");
        core.calibration_txn_id = Some(77);
        let auth_mz = crate::mit::CommissioningAuthority {
            conn_id: 1,
            lease_holder: Some((crate::model::HolderLabel::new("c1"), 1)),
            operator_grant: crate::auth::OperatorGrant::unenforced(),
            expected_epoch: core.safety.epoch_id(),
            expected_stop_gen: core.current_stop_gen(),
            grant_id: Some(grant_id),
            txn_id: 77,
        };
        let req_mz = crate::mit::CommissioningStartRequest {
            kind: crate::mit::CommissioningOpKind::MotorZeroWriteVerify,
            joints,
            joint_len: 1,
            timeout_counts: 0,
            references: [0.0; crate::rt_core::mailbox::MAX_DOF],
            authority: auth_mz,
            txn_deadline: t2 + Duration::from_secs(5),
            prior_disk_mutated: false,
        };
        let _mz_id = core
            .start_commissioning_rt(req_mz, t2)
            .expect("start MotorZeroWriteVerify");
        let mut ledger_zero_tx = ReleaseBudget::new(503, t2, t2 + Duration::from_millis(20), 10, 4);
        ledger_zero_tx.charge_tx(4); // 0 TX remaining
        set_zero_calls.store(0, Ordering::SeqCst);
        let _ = core.service_commissioning_step_with_budget(
            &ctx4,
            &mut ledger_zero_tx,
            t2 + Duration::from_millis(18),
            2,
            None,
            true,
            &mut clock_t2,
        );
        assert_eq!(
            set_zero_calls.load(Ordering::SeqCst),
            0,
            "MotorZeroWriteVerify must not submit set-zero TX when release_budget has 0 TX remaining"
        );
        let _ = core.release(reservation);
    }
}
