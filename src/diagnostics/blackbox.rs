//! Black box: compact JSON lines of the arm state, written by a thread of their own so
//! that the last seconds before a power cut or reset are on the flash.
//!
//! The control loop is not involved: a tokio task takes observations from the telemetry
//! broadcast (which never blocks the sender), picks samples at [`ACTIVE_PERIOD`] while the
//! arm is torqued or moving and at [`IDLE_PERIOD`] otherwise, formats them and hands them
//! to the writer thread through a one-slot channel with `try_send`. A sample is dropped
//! (and counted in `dropped`) when the writer is still busy with the previous one.

use std::io;
use std::path::Path;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{sync_channel, SyncSender};
use std::sync::Arc;
use std::time::{Duration, Instant};

use serde::{Serialize, Serializer};

use super::rotating::RotatingFile;
use crate::model::ArmObservation;

pub const BB_FILE: &str = "bb.jsonl";
/// Rotation size of `bb.jsonl`.
pub const BB_MAX_BYTES: u64 = 512 * 1024;
/// Old files kept: `bb.jsonl.1`, `bb.jsonl.2`.
pub const BB_KEEP: usize = 2;
/// Sample period while torqued or moving (5 Hz).
pub const ACTIVE_PERIOD: Duration = Duration::from_millis(200);
/// Sample period while idle (0.2 Hz).
pub const IDLE_PERIOD: Duration = Duration::from_secs(5);
/// A joint faster than this (rad/s) counts as moving (hand-guided with torque off).
pub const MOVING_RAD_S: f32 = 0.05;

/// Torqued, or a joint moving.
pub fn is_active(obs: &ArmObservation) -> bool {
    obs.is_torqued || obs.velocities.iter().any(|v| v.abs() > MOVING_RAD_S)
}

/// Picks samples: one per period, the period set by the activity; becoming active takes a
/// sample at once.
#[derive(Debug, Default)]
pub struct Sampler {
    last: Option<Instant>,
    last_active: bool,
}

impl Sampler {
    pub fn due(&mut self, now: Instant, active: bool) -> bool {
        let period = if active { ACTIVE_PERIOD } else { IDLE_PERIOD };
        let due = match self.last {
            None => true,
            Some(last) => (active && !self.last_active) || now.saturating_duration_since(last) >= period,
        };
        if due {
            self.last = Some(now);
        }
        self.last_active = active;
        due
    }
}

/// f32 written with 3 decimals.
struct R3<'a>(&'a [f32]);
impl Serialize for R3<'_> {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.collect_seq(self.0.iter().map(|v| if v.is_finite() { (f64::from(*v) * 1000.0).round() / 1000.0 } else { f64::NAN }))
    }
}

#[derive(Serialize)]
struct Record<'a> {
    boot: u64,
    t_ms: u64,
    safety_state: &'a crate::safety::SafetyState,
    is_torqued: bool,
    enable_state: &'a crate::driver::EnableState,
    lease_holder: &'a Option<String>,
    control_mode: &'a crate::model::ControlMode,
    positions: R3<'a>,
    commanded_positions: R3<'a>,
    velocities: R3<'a>,
    torques: R3<'a>,
    temperatures: R3<'a>,
    /// Measured minus model gravity torque, J1..J6; null when unavailable.
    residual_torques: Option<R3<'a>>,
    /// Collision guard state (trip joint, residual, hold); omitted while the guard is off.
    #[serde(skip_serializing_if = "Option::is_none")]
    collision: Option<&'a crate::collision::CollisionObservation>,
    feedback_valid: bool,
    feedback_age_ms: &'a [Option<u64>],
    motor_errors: &'a [Option<crate::model::MotorError>],
    motion_limit: &'a Option<crate::model::MotionLimit>,
    /// Last command source: the latest action as `kind:state`.
    action: Option<String>,
    dropped: u64,
}

/// One black-box line (with the trailing newline). `boot` is armd's persisted start
/// counter, `t_ms` the monotonic time since armd started.
pub fn record_line(boot: u64, t_ms: u64, dropped: u64, obs: &ArmObservation) -> String {
    let record = Record {
        boot,
        t_ms,
        safety_state: &obs.safety_state,
        is_torqued: obs.is_torqued,
        enable_state: &obs.enable_state,
        lease_holder: &obs.lease_holder,
        control_mode: &obs.control_mode,
        positions: R3(&obs.positions),
        commanded_positions: R3(&obs.commanded_positions),
        velocities: R3(&obs.velocities),
        torques: R3(&obs.torques),
        temperatures: R3(&obs.temperatures),
        residual_torques: obs.residual_torques.as_ref().map(|r| R3(r)),
        collision: obs.collision.as_ref(),
        feedback_valid: obs.feedback_valid,
        feedback_age_ms: &obs.feedback_age_ms,
        motor_errors: &obs.motor_errors,
        motion_limit: &obs.motion_limit,
        action: obs.action.as_ref().map(|a| format!("{}:{}", a.kind, a.state)),
        dropped,
    };
    let mut line = serde_json::to_string(&record).unwrap_or_else(|_| format!("{{\"boot\":{boot},\"t_ms\":{t_ms}}}"));
    line.push('\n');
    line
}

/// Where the writer thread puts lines.
pub trait Sink: Send {
    /// Writes one line; `sync` asks for it to be on the medium before returning.
    fn write_line(&mut self, line: &[u8], sync: bool) -> io::Result<()>;
}

impl Sink for RotatingFile {
    fn write_line(&mut self, line: &[u8], sync: bool) -> io::Result<()> {
        self.write_record(line)?;
        if sync {
            self.sync_data()?;
        }
        Ok(())
    }
}

/// What [`BlackBox::offer`] did with an observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Offer {
    /// Not due yet.
    Skipped,
    /// Handed to the writer.
    Queued,
    /// Due, but the writer was busy: dropped.
    Dropped,
}

pub struct BlackBox {
    tx: SyncSender<(String, bool)>,
    sampler: Sampler,
    boot: u64,
    start: Instant,
    dropped: Arc<AtomicU64>,
    written: Arc<AtomicU64>,
}

impl BlackBox {
    /// Starts the writer thread. `start` is armd's start time (`t_ms` 0).
    pub fn start(mut sink: impl Sink + 'static, boot: u64, start: Instant) -> io::Result<Self> {
        let (tx, rx) = sync_channel::<(String, bool)>(1);
        let written = Arc::new(AtomicU64::new(0));
        let written_w = written.clone();
        std::thread::Builder::new().name("armd-blackbox".into()).spawn(move || {
            let mut warned = false;
            while let Ok((line, sync)) = rx.recv() {
                match sink.write_line(line.as_bytes(), sync) {
                    Ok(()) => {
                        written_w.fetch_add(1, Ordering::Relaxed);
                        warned = false;
                    }
                    Err(e) if !warned => {
                        tracing::warn!("Black box write failed: {e}");
                        warned = true;
                    }
                    Err(_) => {}
                }
            }
        })?;
        Ok(Self { tx, sampler: Sampler::default(), boot, start, dropped: Arc::default(), written })
    }

    /// Takes a sample of `obs` when one is due. Never blocks.
    pub fn offer(&mut self, obs: &ArmObservation, now: Instant) -> Offer {
        if !self.sampler.due(now, is_active(obs)) {
            return Offer::Skipped;
        }
        let t_ms = now.saturating_duration_since(self.start).as_millis() as u64;
        let line = record_line(self.boot, t_ms, self.dropped.load(Ordering::Relaxed), obs);
        match self.tx.try_send((line, obs.is_torqued)) {
            Ok(()) => Offer::Queued,
            Err(_) => {
                self.dropped.fetch_add(1, Ordering::Relaxed);
                Offer::Dropped
            }
        }
    }

    pub fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    pub fn written(&self) -> u64 {
        self.written.load(Ordering::Relaxed)
    }
}

/// Opens `dir/bb.jsonl` and starts the writer thread.
pub fn open(dir: &Path, boot: u64, start: Instant) -> io::Result<BlackBox> {
    let file = RotatingFile::open(dir.join(BB_FILE), BB_MAX_BYTES, BB_KEEP)?;
    BlackBox::start(file, boot, start)
}

/// Feeds the black box from the supervisor's telemetry broadcast until it closes.
pub fn spawn_recorder(supervisor: &crate::supervisor::ArmSupervisor, mut bb: BlackBox) -> tokio::task::JoinHandle<()> {
    let mut rx = supervisor.subscribe_telemetry();
    tokio::spawn(async move {
        use tokio::sync::broadcast::error::RecvError;
        loop {
            match rx.recv().await {
                Ok(obs) => {
                    bb.offer(&obs, Instant::now());
                }
                Err(RecvError::Lagged(_)) => continue,
                Err(RecvError::Closed) => break,
            }
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    fn obs(torqued: bool, vel: f32) -> ArmObservation {
        let mut o = crate::supervisor::SupervisorInner::new(crate::ArmConfig::rebot_dm()).current_observation();
        o.is_torqued = torqued;
        o.velocities = vec![vel; o.positions.len()];
        o
    }

    #[test]
    fn rate_follows_activity() {
        let mut s = Sampler::default();
        let t0 = Instant::now();
        let ms = |n: u64| t0 + Duration::from_millis(n);
        assert!(s.due(t0, false), "first sample at once");
        assert!(!s.due(ms(4_900), false));
        assert!(s.due(ms(5_000), false), "idle: 0.2 Hz");
        assert!(s.due(ms(5_050), true), "becoming active samples at once");
        assert!(!s.due(ms(5_200), true));
        assert!(s.due(ms(5_250), true), "active: 5 Hz");
        assert!(s.due(ms(5_450), true));
        assert!(!s.due(ms(5_700), false), "idle again: next sample 5 s after the last");
        assert!(!s.due(ms(10_400), false));
        assert!(s.due(ms(10_450), false));
        // Count over 10 s of 50 Hz ticks.
        let count = |active: bool| {
            let mut s = Sampler::default();
            (0..500).filter(|i| s.due(t0 + Duration::from_millis(i * 20), active)).count()
        };
        assert_eq!(count(true), 50);
        assert_eq!(count(false), 2);
    }

    #[test]
    fn activity_is_torque_or_motion() {
        assert!(is_active(&obs(true, 0.0)));
        assert!(is_active(&obs(false, 0.3)));
        assert!(!is_active(&obs(false, 0.01)));
    }

    #[test]
    fn line_is_compact_json_with_the_state() {
        let mut o = obs(true, 0.123456);
        o.lease_holder = Some("cockpit".into());
        let line = record_line(7, 1234, 2, &o);
        assert!(line.ends_with('\n') && !line[..line.len() - 1].contains('\n'));
        assert!(line.starts_with("{\"boot\":7,\"t_ms\":1234,"), "boot and t_ms first: {line}");
        let v: serde_json::Value = serde_json::from_str(&line).unwrap();
        assert_eq!(v["is_torqued"], true);
        assert_eq!(v["lease_holder"], "cockpit");
        assert_eq!(v["velocities"][0], 0.123);
        assert_eq!(v["positions"].as_array().unwrap().len(), 7);
        assert_eq!(v["dropped"], 2);
        for key in ["safety_state", "torques", "temperatures", "residual_torques", "feedback_age_ms", "motor_errors", "motion_limit", "action"] {
            assert!(v.get(key).is_some(), "{key}");
        }
        assert!(line.len() < 1024, "{} bytes", line.len());
        o.residual_torques = Some([0.5, -1.25, 0.0, 0.0, 0.0, 0.1234]);
        let v: serde_json::Value = serde_json::from_str(&record_line(7, 1234, 2, &o)).unwrap();
        assert_eq!(v["residual_torques"][1], -1.25);
        assert_eq!(v["residual_torques"][5], 0.123);
    }

    struct Collect(Arc<Mutex<Vec<(String, bool)>>>);
    impl Sink for Collect {
        fn write_line(&mut self, line: &[u8], sync: bool) -> io::Result<()> {
            self.0.lock().unwrap().push((String::from_utf8_lossy(line).into_owned(), sync));
            Ok(())
        }
    }

    #[test]
    fn writes_and_syncs_only_while_torqued() {
        let lines = Arc::new(Mutex::new(Vec::new()));
        let t0 = Instant::now();
        let mut bb = BlackBox::start(Collect(lines.clone()), 3, t0).unwrap();
        assert_eq!(bb.offer(&obs(false, 0.0), t0), Offer::Queued);
        std::thread::sleep(Duration::from_millis(50));
        assert_eq!(bb.offer(&obs(true, 0.0), t0 + Duration::from_millis(20)), Offer::Queued);
        std::thread::sleep(Duration::from_millis(50));
        assert_eq!(bb.offer(&obs(true, 0.0), t0 + Duration::from_millis(40)), Offer::Skipped);
        let got = lines.lock().unwrap().clone();
        assert_eq!(got.len(), 2);
        assert!(!got[0].1 && got[1].1, "fdatasync only while torqued");
        assert!(got[1].0.starts_with("{\"boot\":3,\"t_ms\":20,"));
        assert_eq!(bb.written(), 2);
    }

    struct Slow;
    impl Sink for Slow {
        fn write_line(&mut self, _: &[u8], _: bool) -> io::Result<()> {
            std::thread::sleep(Duration::from_millis(400));
            Ok(())
        }
    }

    #[test]
    fn a_slow_writer_never_blocks_the_caller() {
        let t0 = Instant::now();
        let mut bb = BlackBox::start(Slow, 1, t0).unwrap();
        let torqued = obs(true, 0.0);
        let mut worst = Duration::ZERO;
        let mut queued = 0;
        // 2 s of 50 Hz ticks (simulated clock) offered as fast as possible.
        for i in 0..100u64 {
            let before = Instant::now();
            if bb.offer(&torqued, t0 + Duration::from_millis(i * 20)) == Offer::Queued {
                queued += 1;
            }
            worst = worst.max(before.elapsed());
        }
        assert!(worst < Duration::from_millis(50), "offer took {worst:?} with a 400 ms writer");
        // 10 samples due; the writer takes one, the slot holds one, the rest are dropped.
        assert_eq!(queued + bb.dropped(), 10);
        assert!(bb.dropped() >= 7, "dropped {}", bb.dropped());
    }

    #[test]
    fn file_sink_rotates_at_the_configured_size() {
        let dir = tempfile::tempdir().unwrap();
        let mut f = RotatingFile::open(dir.path().join(BB_FILE), 2_000, BB_KEEP).unwrap();
        let line = record_line(1, 0, 0, &obs(true, 0.0));
        for _ in 0..(10_000 / line.len()) {
            f.write_line(line.as_bytes(), true).unwrap();
        }
        let p = dir.path().join(BB_FILE);
        assert!(RotatingFile::numbered(&p, 2).exists() && !RotatingFile::numbered(&p, 3).exists());
        for n in 0..=2 {
            assert!(std::fs::metadata(RotatingFile::numbered(&p, n)).unwrap().len() <= 2_000);
        }
    }
}
