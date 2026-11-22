//! Control tick timing (`GET /api/v1/diagnostics/tick`): a fixed ring of the last
//! [`TICK_WINDOW`] ticks, recorded by [`crate::ArmSupervisor::tick`].

use serde::Serialize;

/// Ticks the percentiles cover.
pub const TICK_WINDOW: usize = 1000;

/// One tick split into phases (µs once recorded).
#[derive(Debug, Clone, Copy, Default)]
pub struct TickSample {
    /// Whole `ArmSupervisor::tick` call.
    pub tick: std::time::Duration,
    /// Waiting for the supervisor write lock.
    pub wait: std::time::Duration,
    /// Holding the supervisor write lock.
    pub lock: std::time::Duration,
    /// After the lock: telemetry broadcast, spawned persist job (the observation snapshot is
    /// published under the lock).
    pub post: std::time::Duration,
    /// `dt` of the tick: start-to-start interval of the control loop.
    pub period: std::time::Duration,
    /// Longest supervisor read-lock hold by another task that ended since the previous tick.
    pub reader_hold: std::time::Duration,
    /// Longest supervisor write-lock hold by another task that ended since the previous tick.
    pub writer_hold: std::time::Duration,
    /// Longest `read().await` of another task that completed since the previous tick: long
    /// when the lock was handed to a reader that was not scheduled.
    pub reader_acquire: std::time::Duration,
    /// Longest `write().await` of another task that completed since the previous tick.
    pub writer_acquire: std::time::Duration,
    /// Wait for the tick-stats mutex of the previous tick (not part of `tick`).
    pub stats_wait: std::time::Duration,
    pub late: bool,
}

/// Fields of the ring, one array each.
const FIELDS: usize = 10;

/// Ring buffer of per-tick durations (µs). Recording is O(1); percentiles are computed
/// on request from a copy, outside the mutex that guards the ring.
pub struct TickStats {
    us: Box<[[u32; FIELDS]; TICK_WINDOW]>,
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

/// Phases of the tick with the largest `tick_us` in the window.
#[derive(Debug, Clone, Copy, Default, Serialize, PartialEq, Eq)]
pub struct TickBreakdown {
    pub tick_us: u32,
    pub wait_us: u32,
    pub lock_us: u32,
    pub post_us: u32,
    pub period_us: u32,
    pub reader_hold_us: u32,
    pub writer_hold_us: u32,
    pub reader_acquire_us: u32,
    pub writer_acquire_us: u32,
    /// Ticks recorded after this one (0: the latest tick).
    pub ticks_ago: usize,
}

/// Scheduling of the control thread as read back after the real-time request.
#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
pub struct ControlThreadSched {
    /// `"fifo"`, `"rr"`, `"other"` or the raw policy number.
    pub policy: String,
    pub priority: i32,
    /// Whether the SCHED_FIFO request succeeded.
    pub realtime: bool,
}

/// Observations served by `GET /api/v1/observation` from the tick's snapshot (`hits`) or
/// computed under the supervisor read lock (`misses`); a hit after waiting for the read
/// lock (a tick published meanwhile) counts as a hit.
#[derive(Debug, Clone, Copy, Default, Serialize, PartialEq, Eq)]
pub struct ObservationCacheStats {
    pub hits: u64,
    pub misses: u64,
    /// Ticks that skipped publishing because a reader held the snapshot mutex.
    pub publish_skipped: u64,
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
    /// Time the tick waited for the supervisor write lock.
    pub wait_us: Percentiles,
    /// Time after the lock was released (telemetry broadcast).
    pub post_us: Percentiles,
    /// Start-to-start interval of the control loop (`dt`).
    pub period_us: Percentiles,
    /// Longest read-lock hold by another task per tick interval (includes preemption of that task).
    pub reader_hold_us: Percentiles,
    /// Longest write-lock hold by another task per tick interval.
    pub writer_hold_us: Percentiles,
    /// Wait for the tick-stats mutex (contention with this route).
    pub stats_wait_us: Percentiles,
    /// Longest `read().await` of another task per tick interval.
    pub reader_acquire_us: Percentiles,
    /// Longest `write().await` of another task per tick interval.
    pub writer_acquire_us: Percentiles,
    /// Phases of the slowest tick in the window.
    pub max_tick: Option<TickBreakdown>,
    /// Control thread scheduling; None until the control thread reported it (tests).
    pub control_thread: Option<ControlThreadSched>,
    pub observation_cache: ObservationCacheStats,
}

impl Default for TickStats {
    fn default() -> Self {
        Self { us: Box::new([[0; FIELDS]; TICK_WINDOW]), len: 0, next: 0, late_count: 0, ticks: 0 }
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

/// Copy of the ring taken under the mutex; [`TickWindow::snapshot`] sorts outside it.
pub struct TickWindow {
    rows: Vec<[u32; FIELDS]>,
    late_count: u64,
    ticks: u64,
}

impl TickStats {
    pub fn record(&mut self, s: &TickSample) {
        self.us[self.next] = [
            us(s.tick), us(s.lock), us(s.wait), us(s.post), us(s.period),
            us(s.reader_hold), us(s.writer_hold), us(s.stats_wait),
            us(s.reader_acquire), us(s.writer_acquire),
        ];
        self.next = (self.next + 1) % TICK_WINDOW;
        self.len = (self.len + 1).min(TICK_WINDOW);
        self.ticks += 1;
        if s.late { self.late_count += 1; }
    }

    /// Rows oldest first.
    pub fn window(&self) -> TickWindow {
        let start = if self.len < TICK_WINDOW { 0 } else { self.next };
        let rows = (0..self.len).map(|i| self.us[(start + i) % TICK_WINDOW]).collect();
        TickWindow { rows, late_count: self.late_count, ticks: self.ticks }
    }

    pub fn snapshot(&self) -> TickSnapshot { self.window().snapshot() }
}

impl TickWindow {
    pub fn snapshot(&self) -> TickSnapshot {
        let col = |f: usize| -> Percentiles {
            let mut v: Vec<u32> = self.rows.iter().map(|r| r[f]).collect();
            percentiles(&mut v)
        };
        // the latest of equal maxima
        let max_tick = self.rows.iter().enumerate().rev().max_by_key(|(_, r)| r[0]).map(|(i, r)| TickBreakdown {
            tick_us: r[0], lock_us: r[1], wait_us: r[2], post_us: r[3], period_us: r[4],
            reader_hold_us: r[5], writer_hold_us: r[6], reader_acquire_us: r[8], writer_acquire_us: r[9], ticks_ago: self.rows.len() - 1 - i,
        });
        TickSnapshot {
            window: self.rows.len(),
            tick_us: col(0),
            lock_us: col(1),
            late_count: self.late_count,
            since_boot_ticks: self.ticks,
            wait_us: col(2),
            post_us: col(3),
            period_us: col(4),
            reader_hold_us: col(5),
            writer_hold_us: col(6),
            stats_wait_us: col(7),
            reader_acquire_us: col(8),
            writer_acquire_us: col(9),
            max_tick,
            control_thread: None,
            observation_cache: ObservationCacheStats::default(),
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
        assert_eq!(s.snapshot().max_tick, None);
        for i in 1..=(TICK_WINDOW as u64 + 100) {
            s.record(&TickSample { tick: Duration::from_micros(i), lock: Duration::from_micros(1), late: i % 10 == 0, ..Default::default() });
        }
        let snap = s.snapshot();
        assert_eq!(snap.window, TICK_WINDOW);
        assert_eq!(snap.since_boot_ticks, TICK_WINDOW as u64 + 100);
        assert_eq!(snap.late_count, 110);
        assert_eq!(snap.tick_us.max, 1100);
        assert_eq!(snap.tick_us.p50, 600);
        assert_eq!(snap.tick_us.p99, 1090);
        assert_eq!(snap.lock_us, Percentiles { p50: 1, p99: 1, max: 1 });
        assert_eq!(snap.max_tick.unwrap().ticks_ago, 0);
    }

    #[test]
    fn max_tick_breakdown_belongs_to_the_slowest_tick() {
        let mut s = TickStats::default();
        let ms = Duration::from_millis;
        for i in 0..50u64 {
            let slow = i == 20;
            s.record(&TickSample {
                tick: if slow { ms(34) } else { ms(1) },
                wait: if slow { ms(32) } else { Duration::ZERO },
                lock: ms(1),
                post: Duration::from_micros(100),
                period: ms(20),
                reader_hold: if slow { ms(31) } else { Duration::from_micros(200) },
                writer_hold: Duration::from_micros(50),
                stats_wait: Duration::ZERO,
                reader_acquire: if slow { ms(33) } else { Duration::ZERO },
                writer_acquire: Duration::ZERO,
                late: false,
            });
        }
        let snap = s.snapshot();
        let m = snap.max_tick.unwrap();
        assert_eq!((m.tick_us, m.wait_us, m.lock_us, m.post_us), (34_000, 32_000, 1_000, 100));
        assert_eq!((m.period_us, m.reader_hold_us, m.writer_hold_us, m.ticks_ago), (20_000, 31_000, 50, 29));
        assert_eq!(snap.wait_us.max, 32_000);
        assert_eq!(snap.reader_hold_us.p50, 200);
        let json = serde_json::to_value(&snap).unwrap();
        for key in ["tick_us", "lock_us", "wait_us", "post_us", "period_us", "reader_hold_us", "writer_hold_us", "stats_wait_us", "reader_acquire_us", "writer_acquire_us"] {
            assert!(json[key]["max"].is_u64(), "{key}");
        }
        assert_eq!(json["max_tick"]["wait_us"], 32_000);
        assert_eq!(json["max_tick"]["reader_acquire_us"], 33_000);
    }
}
