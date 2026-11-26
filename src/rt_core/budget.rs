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
}
