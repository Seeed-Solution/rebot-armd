//! Control tick timing (`GET /api/v1/diagnostics/tick`): a fixed ring of the last
//! [`TICK_WINDOW`] ticks, recorded by [`crate::ArmSupervisor::tick`].

use serde::Serialize;

/// Ticks the percentiles cover.
pub const TICK_WINDOW: usize = 1000;

/// Ring buffer of per-tick durations (µs). Recording is O(1); percentiles are computed
/// on request from a copy.
pub struct TickStats {
    tick_us: Box<[u32; TICK_WINDOW]>,
    lock_us: Box<[u32; TICK_WINDOW]>,
    len: usize,
    next: usize,
    late_count: u64,
    ticks: u64,
}

#[derive(Debug, Clone, Copy, Default, Serialize, PartialEq, Eq)]
pub struct Percentiles {
    pub p50: u32,
    pub p99: u32,
    pub max: u32,
}

#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
pub struct TickSnapshot {
    /// Ticks in the window (at most [`TICK_WINDOW`]).
    pub window: usize,
    /// Whole `ArmSupervisor::tick` call: lock wait, tick body and telemetry broadcast.
    pub tick_us: Percentiles,
    /// Time the supervisor write lock was held by the tick.
    pub lock_us: Percentiles,
    /// Ticks since start whose `dt` exceeded two control periods.
    pub late_count: u64,
    pub since_boot_ticks: u64,
}

impl Default for TickStats {
    fn default() -> Self {
        Self {
            tick_us: Box::new([0; TICK_WINDOW]),
            lock_us: Box::new([0; TICK_WINDOW]),
            len: 0,
            next: 0,
            late_count: 0,
            ticks: 0,
        }
    }
}

fn us(d: std::time::Duration) -> u32 {
    d.as_micros().min(u128::from(u32::MAX)) as u32
}

/// Nearest-rank percentiles of `values` (sorted in place).
fn percentiles(values: &mut [u32]) -> Percentiles {
    if values.is_empty() { return Percentiles::default(); }
    values.sort_unstable();
    let rank = |p: usize| values[(values.len() * p).div_ceil(100).max(1) - 1];
    Percentiles { p50: rank(50), p99: rank(99), max: values[values.len() - 1] }
}

impl TickStats {
    pub fn record(&mut self, tick: std::time::Duration, lock: std::time::Duration, late: bool) {
        self.tick_us[self.next] = us(tick);
        self.lock_us[self.next] = us(lock);
        self.next = (self.next + 1) % TICK_WINDOW;
        self.len = (self.len + 1).min(TICK_WINDOW);
        self.ticks += 1;
        if late { self.late_count += 1; }
    }

    pub fn snapshot(&self) -> TickSnapshot {
        let mut tick = self.tick_us[..self.len].to_vec();
        let mut lock = self.lock_us[..self.len].to_vec();
        TickSnapshot {
            window: self.len,
            tick_us: percentiles(&mut tick),
            lock_us: percentiles(&mut lock),
            late_count: self.late_count,
            since_boot_ticks: self.ticks,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn ring_keeps_the_last_window_and_counts_late_ticks() {
        let mut s = TickStats::default();
        assert_eq!(s.snapshot().window, 0);
        for i in 1..=(TICK_WINDOW as u64 + 100) {
            s.record(Duration::from_micros(i), Duration::from_micros(1), i % 10 == 0);
        }
        let snap = s.snapshot();
        assert_eq!(snap.window, TICK_WINDOW);
        assert_eq!(snap.since_boot_ticks, TICK_WINDOW as u64 + 100);
        assert_eq!(snap.late_count, 110);
        assert_eq!(snap.tick_us.max, 1100);
        assert_eq!(snap.tick_us.p50, 600);
        assert_eq!(snap.tick_us.p99, 1090);
        assert_eq!(snap.lock_us, Percentiles { p50: 1, p99: 1, max: 1 });
    }
}
