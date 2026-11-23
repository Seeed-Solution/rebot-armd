//! Control tick timing (`GET /api/v1/diagnostics/tick`): a fixed ring of the last
//! [`TICK_WINDOW`] ticks, recorded by [`crate::ArmSupervisor::tick`].

use serde::{Serialize, Serializer};

/// Ticks the percentiles cover.
pub const TICK_WINDOW: usize = 1000;

/// Phases of the tick's own write-lock hold, µs. They cover the hold: `lock_us` is their sum
/// up to the clock reads between them.
#[derive(Debug, Clone, Copy, Default, Serialize, PartialEq, Eq)]
pub struct LockPhases {
    /// From the lock to `SupervisorInner::tick`: thread usage read, hold log taken.
    pub pre_us: u32,
    /// Driver feedback read (CAN RX) and its bookkeeping.
    pub rx_us: u32,
    /// Safety checks, guards, trajectory / servo step.
    pub control_us: u32,
    /// Driver output (CAN TX).
    pub tx_us: u32,
    /// Output error handling, action reconcile, simulation mirror, handover.
    pub finish_us: u32,
    /// Observation build and episode frame (and anything after an early return).
    pub obs_us: u32,
    /// After `SupervisorInner::tick` until the lock is released: snapshot expiry and publication.
    pub publish_us: u32,
}

/// What the control thread did while it waited and while it held the lock, from
/// `getrusage(RUSAGE_THREAD)` and `CLOCK_THREAD_CPUTIME_ID` around the phases (Linux; zero
/// elsewhere). Valid for the control thread, which polls the tick on one OS thread.
#[derive(Debug, Clone, Copy, Default, Serialize, PartialEq, Eq)]
pub struct ThreadUsage {
    /// CPU time of the thread during the lock wait (wall minus this: off CPU).
    pub wait_cpu_us: u32,
    /// CPU time of the thread while it held the lock.
    pub lock_cpu_us: u32,
    /// Voluntary / involuntary context switches during the wait (voluntary: parked).
    pub wait_vcsw: u32,
    pub wait_ivcsw: u32,
    /// Voluntary (blocked, slept) / involuntary (preempted) switches while holding the lock.
    pub lock_vcsw: u32,
    pub lock_ivcsw: u32,
    /// Major / minor page faults during the wait and during the hold.
    pub wait_majflt: u32,
    pub lock_majflt: u32,
    pub lock_minflt: u32,
}

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

/// One tick, µs. Serialized as the breakdown of `max_tick` / `max_lock`.
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
    #[serde(flatten)]
    pub phases: LockPhases,
    #[serde(flatten)]
    pub usage: ThreadUsage,
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

/// Percentiles of the tick's lock phases.
#[derive(Debug, Clone, Copy, Default, Serialize, PartialEq, Eq)]
pub struct LockPhasePercentiles {
    pub pre_us: Percentiles,
    pub rx_us: Percentiles,
    pub control_us: Percentiles,
    pub tx_us: Percentiles,
    pub finish_us: Percentiles,
    pub obs_us: Percentiles,
    pub publish_us: Percentiles,
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
    /// The tick's lock hold split into phases.
    pub lock_phases: LockPhasePercentiles,
    /// The slowest tick in the window, all phases.
    pub max_tick: Option<TickBreakdown>,
    /// The tick with the longest lock hold in the window, all phases.
    pub max_lock: Option<TickBreakdown>,
    /// Control thread scheduling; None until the control thread reported it (tests).
    pub control_thread: Option<ControlThreadSched>,
    pub observation_cache: ObservationCacheStats,
}

impl Default for TickStats {
    fn default() -> Self {
        Self { rows: vec![TickSample::default(); TICK_WINDOW].into_boxed_slice(), len: 0, next: 0, late_count: 0, ticks: 0 }
    }
}

/// Thread CPU time and counters at one instant (see [`ThreadUsage`]).
#[derive(Debug, Clone, Copy, Default)]
pub struct UsageMark {
    cpu_ns: u64,
    vcsw: i64,
    ivcsw: i64,
    majflt: i64,
    minflt: i64,
}

impl UsageMark {
    /// Two syscalls on Linux; zero elsewhere.
    pub fn now() -> Self {
        #[cfg(target_os = "linux")]
        {
            // SAFETY: plain syscalls on the calling thread with valid out-pointers.
            unsafe {
                let mut ts: libc::timespec = std::mem::zeroed();
                libc::clock_gettime(libc::CLOCK_THREAD_CPUTIME_ID, &mut ts);
                let mut ru: libc::rusage = std::mem::zeroed();
                libc::getrusage(libc::RUSAGE_THREAD, &mut ru);
                UsageMark {
                    cpu_ns: (ts.tv_sec as u64).saturating_mul(1_000_000_000).saturating_add(ts.tv_nsec as u64),
                    vcsw: ru.ru_nvcsw as i64,
                    ivcsw: ru.ru_nivcsw as i64,
                    majflt: ru.ru_majflt as i64,
                    minflt: ru.ru_minflt as i64,
                }
            }
        }
        #[cfg(not(target_os = "linux"))]
        UsageMark::default()
    }
}

/// [`ThreadUsage`] from marks at tick start, lock acquired and lock released.
pub fn thread_usage(start: &UsageMark, locked: &UsageMark, released: &UsageMark) -> ThreadUsage {
    let n = |a: i64, b: i64| u32::try_from(b.saturating_sub(a).max(0)).unwrap_or(u32::MAX);
    let cpu = |a: u64, b: u64| u32::try_from(b.saturating_sub(a) / 1000).unwrap_or(u32::MAX);
    ThreadUsage {
        wait_cpu_us: cpu(start.cpu_ns, locked.cpu_ns),
        lock_cpu_us: cpu(locked.cpu_ns, released.cpu_ns),
        wait_vcsw: n(start.vcsw, locked.vcsw),
        wait_ivcsw: n(start.ivcsw, locked.ivcsw),
        lock_vcsw: n(locked.vcsw, released.vcsw),
        lock_ivcsw: n(locked.ivcsw, released.ivcsw),
        wait_majflt: n(start.majflt, locked.majflt),
        lock_majflt: n(locked.majflt, released.majflt),
        lock_minflt: n(locked.minflt, released.minflt),
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
    /// The samples, oldest first.
    pub fn samples(&self) -> &[TickSample] { &self.rows }

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
            lock_phases: LockPhasePercentiles {
                pre_us: self.col(|r| r.phases.pre_us),
                rx_us: self.col(|r| r.phases.rx_us),
                control_us: self.col(|r| r.phases.control_us),
                tx_us: self.col(|r| r.phases.tx_us),
                finish_us: self.col(|r| r.phases.finish_us),
                obs_us: self.col(|r| r.phases.obs_us),
                publish_us: self.col(|r| r.phases.publish_us),
            },
            max_tick: self.max_by(|r| r.tick_us),
            max_lock: self.max_by(|r| r.lock_us),
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
    fn max_tick_and_max_lock_carry_their_phases_and_holder() {
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
            let (slow, heavy) = (i == 20, i == 30);
            s.record(&TickSample {
                tick_us: if slow { 34_000 } else if heavy { 15_000 } else { 1_000 },
                wait_us: if slow { 32_000 } else { 0 },
                lock_us: if heavy { 14_000 } else { 1_000 },
                post_us: 100,
                period_us: 20_000,
                reader_hold_us: if slow { 31_000 } else { 200 },
                phases: LockPhases { rx_us: if heavy { 12_000 } else { 300 }, ..Default::default() },
                usage: ThreadUsage { lock_majflt: u32::from(heavy) * 3, ..Default::default() },
                waited_for: slow.then_some(holder),
                ..Default::default()
            });
        }
        let snap = s.snapshot();
        let m = snap.max_tick.unwrap();
        assert_eq!((m.sample.tick_us, m.sample.wait_us, m.ticks_ago), (34_000, 32_000, 29));
        let l = snap.max_lock.unwrap();
        assert_eq!((l.sample.lock_us, l.sample.phases.rx_us, l.ticks_ago), (14_000, 12_000, 19));
        assert_eq!(snap.lock_phases.rx_us.max, 12_000);
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
        assert_eq!(json["max_lock"]["rx_us"], 12_000);
        assert_eq!(json["max_lock"]["lock_majflt"], 3);
        assert_eq!(json["lock_phases"]["rx_us"]["max"], 12_000);
        assert!(json["control_thread"].is_null());
    }
}
