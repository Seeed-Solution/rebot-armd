//! Control tick timing (`GET /api/v1/diagnostics/tick`): a fixed ring of the last
//! [`TICK_WINDOW`] ticks, recorded by [`crate::ArmSupervisor::tick`].

use serde::{Serialize, Serializer};

/// Ticks the percentiles cover.
pub const TICK_WINDOW: usize = 1000;

/// The guard of another task the tick waited for: the last one released before the tick
/// got the lock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Holder {
    pub kind: &'static str,
    pub location: &'static std::panic::Location<'static>,
    pub hold_us: u32,
    pub acquire_us: u32,
    /// How long before the tick started waiting the holder took the lock (negative: after).
    pub acquired_before_wait_us: i64,
    /// How long before the tick got the lock the holder released it: the tick's wake-up
    /// latency when the tick waited for this holder.
    pub released_before_lock_us: i64,
}

impl Serialize for Holder {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        use serde::ser::SerializeStruct;
        let mut st = s.serialize_struct("Holder", 6)?;
        st.serialize_field("kind", self.kind)?;
        st.serialize_field("location", &format!("{}:{}", self.location.file(), self.location.line()))?;
        st.serialize_field("hold_us", &self.hold_us)?;
        st.serialize_field("acquire_us", &self.acquire_us)?;
        st.serialize_field("acquired_before_wait_us", &self.acquired_before_wait_us)?;
        st.serialize_field("released_before_lock_us", &self.released_before_lock_us)?;
        st.end()
    }
}

/// One tick, µs. Serialized as the breakdown of `max_tick`.
#[derive(Debug, Clone, Copy, Default, Serialize, PartialEq, Eq)]
pub struct TickSample {
    /// Whole `ArmSupervisor::tick` call.
    pub tick_us: u32,
    /// Waiting for the supervisor write lock.
    pub wait_us: u32,
    /// Holding the supervisor write lock.
    pub lock_us: u32,
    /// After the lock: telemetry broadcast, spawned persist job.
    pub post_us: u32,
    /// `dt` of the tick: start-to-start interval of the control loop.
    pub period_us: u32,
    /// Longest read / write hold by another task released since the previous tick.
    pub reader_hold_us: u32,
    pub writer_hold_us: u32,
    /// Longest `read()` / `write()` of another task released since the previous tick: long
    /// when the lock was handed to a task that was not scheduled.
    pub reader_acquire_us: u32,
    pub writer_acquire_us: u32,
    /// Wait for the tick-stats mutex of the previous tick (not part of `tick_us`).
    pub stats_wait_us: u32,
    /// The guard the tick waited for; None when no other guard was released since the
    /// previous tick (or the log was busy).
    pub waited_for: Option<Holder>,
    /// The longest other hold since the previous tick (may differ from `waited_for`).
    pub longest_hold: Option<Holder>,
    #[serde(skip)]
    pub late: bool,
}

/// Ring buffer of per-tick samples. Recording is O(1); percentiles are computed on request
/// from a copy, outside the mutex that guards the ring.
pub struct TickStats {
    rows: Box<[TickSample]>,
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

/// A recorded tick with its position in the window.
#[derive(Debug, Clone, Copy, Serialize, PartialEq, Eq)]
pub struct TickBreakdown {
    #[serde(flatten)]
    pub sample: TickSample,
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
    /// The slowest tick in the window, all phases.
    pub max_tick: Option<TickBreakdown>,
    /// Control thread scheduling; None until the control thread reported it (tests).
    pub control_thread: Option<ControlThreadSched>,
    pub observation_cache: ObservationCacheStats,
}

impl Default for TickStats {
    fn default() -> Self {
        Self { rows: vec![TickSample::default(); TICK_WINDOW].into_boxed_slice(), len: 0, next: 0, late_count: 0, ticks: 0 }
    }
}

/// Microseconds, saturating.
pub fn us(d: std::time::Duration) -> u32 {
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
    rows: Vec<TickSample>,
    late_count: u64,
    ticks: u64,
}

impl TickStats {
    pub fn record(&mut self, s: &TickSample) {
        self.rows[self.next] = *s;
        self.next = (self.next + 1) % TICK_WINDOW;
        self.len = (self.len + 1).min(TICK_WINDOW);
        self.ticks += 1;
        if s.late { self.late_count += 1; }
    }

    /// Rows oldest first.
    pub fn window(&self) -> TickWindow {
        let start = if self.len < TICK_WINDOW { 0 } else { self.next };
        let rows = (0..self.len).map(|i| self.rows[(start + i) % TICK_WINDOW]).collect();
        TickWindow { rows, late_count: self.late_count, ticks: self.ticks }
    }

    pub fn snapshot(&self) -> TickSnapshot { self.window().snapshot() }
}

impl TickWindow {
    fn col(&self, f: impl Fn(&TickSample) -> u32) -> Percentiles {
        let mut v: Vec<u32> = self.rows.iter().map(f).collect();
        percentiles(&mut v)
    }

    /// The latest of equal maxima of `key`.
    fn max_by(&self, key: impl Fn(&TickSample) -> u32) -> Option<TickBreakdown> {
        let n = self.rows.len();
        self.rows.iter().enumerate().rev().max_by_key(|(_, r)| key(r))
            .map(|(i, r)| TickBreakdown { sample: *r, ticks_ago: n - 1 - i })
    }

    pub fn snapshot(&self) -> TickSnapshot {
        TickSnapshot {
            window: self.rows.len(),
            tick_us: self.col(|r| r.tick_us),
            lock_us: self.col(|r| r.lock_us),
            late_count: self.late_count,
            since_boot_ticks: self.ticks,
            wait_us: self.col(|r| r.wait_us),
            post_us: self.col(|r| r.post_us),
            period_us: self.col(|r| r.period_us),
            reader_hold_us: self.col(|r| r.reader_hold_us),
            writer_hold_us: self.col(|r| r.writer_hold_us),
            stats_wait_us: self.col(|r| r.stats_wait_us),
            reader_acquire_us: self.col(|r| r.reader_acquire_us),
            writer_acquire_us: self.col(|r| r.writer_acquire_us),
            max_tick: self.max_by(|r| r.tick_us),
            control_thread: None,
            observation_cache: ObservationCacheStats::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ring_keeps_the_last_window_and_counts_late_ticks() {
        let mut s = TickStats::default();
        assert_eq!(s.snapshot().window, 0);
        assert_eq!(s.snapshot().max_tick, None);
        for i in 1..=(TICK_WINDOW as u32 + 100) {
            s.record(&TickSample { tick_us: i, lock_us: 1, late: i % 10 == 0, ..Default::default() });
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
    fn max_tick_carries_its_phases_and_holder() {
        let mut s = TickStats::default();
        let holder = Holder {
            kind: "read",
            location: std::panic::Location::caller(),
            hold_us: 31_000,
            acquire_us: 3,
            acquired_before_wait_us: 1_000,
            released_before_lock_us: 40,
        };
        for i in 0..50u32 {
            let slow = i == 20;
            s.record(&TickSample {
                tick_us: if slow { 34_000 } else { 1_000 },
                wait_us: if slow { 32_000 } else { 0 },
                lock_us: 1_000,
                post_us: 100,
                period_us: 20_000,
                reader_hold_us: if slow { 31_000 } else { 200 },
                waited_for: slow.then_some(holder),
                ..Default::default()
            });
        }
        let snap = s.snapshot();
        let m = snap.max_tick.unwrap();
        assert_eq!((m.sample.tick_us, m.sample.wait_us, m.ticks_ago), (34_000, 32_000, 29));
        let json = serde_json::to_value(&snap).unwrap();
        for key in ["tick_us", "lock_us", "wait_us", "post_us", "period_us", "reader_hold_us", "writer_hold_us", "stats_wait_us", "reader_acquire_us", "writer_acquire_us"] {
            assert!(json[key]["max"].is_u64(), "{key}");
        }
        let mt = &json["max_tick"];
        assert_eq!(mt["wait_us"], 32_000);
        assert_eq!(mt["waited_for"]["kind"], "read");
        assert_eq!(mt["waited_for"]["hold_us"], 31_000);
        assert!(mt["waited_for"]["location"].as_str().unwrap().starts_with("src/diagnostics/tick.rs:"), "{mt}");
        assert!(mt["longest_hold"].is_null());
        assert!(mt.get("late").is_none());
        assert!(json["control_thread"].is_null());
    }
}
