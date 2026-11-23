//! The RT executor's release clock (design section 5): absolute monotonic release times,
//! missed releases skipped, elapsed time preserved.
//!
//! The executor sleeps until an absolute `CLOCK_MONOTONIC` release (`clock_nanosleep` with
//! `TIMER_ABSTIME` on Linux), so neither the tick's own duration nor a late wake-up moves
//! later releases. When a tick ends after the next release, the missed releases are skipped
//! (the next release is one period after the wake-up) and the step gets the real elapsed
//! time, so the control code still sees a stall (`CONTROL_DEADLINE`).
//!
//! Nothing here allocates, formats or logs on the executor thread: a late tick is counted in
//! [`ExecutorStats`], and a normal-priority thread logs it ([`ExecutorStats::spawn_monitor`]).

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Elapsed time above which a tick counts as late in the log (as in 1.5).
pub const LATE_LOG_AFTER: Duration = Duration::from_millis(80);

/// Counters the executor thread updates with plain atomic stores.
#[derive(Debug, Default)]
pub struct ExecutorStats {
    /// Steps run.
    pub steps: AtomicU64,
    /// Steps whose elapsed time exceeded [`LATE_LOG_AFTER`].
    pub late: AtomicU64,
    /// Elapsed time of the latest late step, microseconds.
    pub last_late_us: AtomicU64,
    /// Releases skipped because a step ended after them.
    pub skipped_releases: AtomicU64,
}

impl ExecutorStats {
    /// A normal-priority thread that logs late steps, at most once per second (the 1.5
    /// control loop logged them from the control thread itself).
    pub fn spawn_monitor(self: &Arc<Self>, stop: Arc<AtomicBool>) -> std::io::Result<std::thread::JoinHandle<()>> {
        let stats = self.clone();
        std::thread::Builder::new().name("armd-rt-monitor".into()).spawn(move || {
            let mut seen = stats.late.load(Ordering::Acquire);
            while !stop.load(Ordering::Acquire) {
                std::thread::sleep(Duration::from_secs(1));
                let late = stats.late.load(Ordering::Acquire);
                if late != seen {
                    let ms = stats.last_late_us.load(Ordering::Relaxed) as f64 / 1000.0;
                    tracing::warn!("Control tick late: {ms:.0} ms since previous tick ({} late ticks in the last second)", late - seen);
                    seen = late;
                }
            }
        })
    }
}

/// Monotonic nanoseconds of `CLOCK_MONOTONIC` (the clock of `Instant` on Linux).
#[cfg(target_os = "linux")]
fn mono_now_ns() -> u64 {
    // SAFETY: `ts` is a valid out-pointer.
    let mut ts: libc::timespec = unsafe { std::mem::zeroed() };
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
}

/// Sleeps until the absolute monotonic time `at_ns` (returns at once when it has passed).
#[cfg(target_os = "linux")]
fn sleep_until_ns(at_ns: u64) {
    let ts = libc::timespec { tv_sec: (at_ns / 1_000_000_000) as libc::time_t, tv_nsec: (at_ns % 1_000_000_000) as libc::c_long };
    loop {
        // SAFETY: `ts` is valid; the remaining-time pointer is unused with TIMER_ABSTIME.
        let rc = unsafe { libc::clock_nanosleep(libc::CLOCK_MONOTONIC, libc::TIMER_ABSTIME, &ts, std::ptr::null_mut()) };
        if rc != libc::EINTR { return; }
    }
}

#[cfg(not(target_os = "linux"))]
fn mono_now_ns() -> u64 {
    static START: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    START.get_or_init(Instant::now).elapsed().as_nanos() as u64
}

#[cfg(not(target_os = "linux"))]
fn sleep_until_ns(at_ns: u64) {
    let now = mono_now_ns();
    if at_ns > now { std::thread::sleep(Duration::from_nanos(at_ns - now)); }
}

/// The release schedule: the next absolute release and the skip rule.
#[derive(Debug, Clone, Copy)]
pub struct Releases {
    period_ns: u64,
    next_ns: u64,
}

impl Releases {
    /// First release one period from now.
    pub fn start(period: Duration) -> Self {
        let period_ns = (period.as_nanos() as u64).max(1);
        Self { period_ns, next_ns: mono_now_ns() + period_ns }
    }

    /// Advances past a release at `now_ns`: the next one is one period later, or, when that
    /// has passed already, one period after `now_ns` (missed releases are skipped). Returns
    /// the number of releases skipped.
    pub fn advance(&mut self, now_ns: u64) -> u64 {
        self.next_ns += self.period_ns;
        if self.next_ns < now_ns {
            let skipped = (now_ns - self.next_ns) / self.period_ns + 1;
            self.next_ns = now_ns + self.period_ns;
            skipped
        } else {
            0
        }
    }

    pub fn next_ns(&self) -> u64 { self.next_ns }
}

/// Runs `step(dt)` at every release until `stop` is set. `dt` is the real time since the
/// previous step's start (seconds). Allocation free.
pub fn run_periodic(period: Duration, stop: &AtomicBool, stats: &ExecutorStats, mut step: impl FnMut(f64)) {
    let mut releases = Releases::start(period);
    let mut last = Instant::now();
    while !stop.load(Ordering::Acquire) {
        sleep_until_ns(releases.next_ns());
        let woke = Instant::now();
        let skipped = releases.advance(mono_now_ns());
        if skipped > 0 { stats.skipped_releases.fetch_add(skipped, Ordering::Relaxed); }
        let elapsed = woke - last;
        last = woke;
        if elapsed > LATE_LOG_AFTER {
            stats.last_late_us.store(elapsed.as_micros() as u64, Ordering::Relaxed);
            stats.late.fetch_add(1, Ordering::Release);
        }
        stats.steps.fetch_add(1, Ordering::Relaxed);
        step(elapsed.as_secs_f64());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn releases_are_absolute_and_skip_missed_ones() {
        let mut r = Releases { period_ns: 20, next_ns: 100 };
        // on time: one period later, nothing skipped
        assert_eq!(r.advance(101), 0);
        assert_eq!(r.next_ns(), 120);
        // a late wake-up within the period keeps the grid
        assert_eq!(r.advance(135), 0);
        assert_eq!(r.next_ns(), 140);
        // a step that ran past two releases: skipped, next one period after now
        assert_eq!(r.advance(190), 2); // 160 and 180
        assert_eq!(r.next_ns(), 210);
    }

    #[test]
    fn steps_run_with_elapsed_time_until_stopped() {
        let stop = AtomicBool::new(false);
        let stats = ExecutorStats::default();
        let mut dts = Vec::with_capacity(16);
        run_periodic(Duration::from_millis(2), &stop, &stats, |dt| {
            dts.push(dt);
            if dts.len() == 5 { stop.store(true, Ordering::Release); }
        });
        assert_eq!(dts.len(), 5);
        assert!(dts.iter().all(|dt| *dt > 0.0005 && *dt < 0.5), "{dts:?}");
        assert_eq!(stats.steps.load(Ordering::Relaxed), 5);
    }
}
