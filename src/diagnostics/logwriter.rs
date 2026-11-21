//! Non-blocking log writer: the tracing layers hand each formatted line to a thread of its
//! own, so a log event never waits for stdout or the flash.
//!
//! The control tick logs while it holds the supervisor lock (for example `Action N (kind)
//! done` from `finish_action`). With a writer that writes in place, that `write` ran inside
//! the tick: on spark a single line took up to 45 ms under the lock (2026-11-21, mock
//! armd), and every other thread logging at the same time (HTTP requests) shared the file
//! layer's mutex. Here a line goes into a bounded queue with `try_send`; when the queue is
//! full the line is dropped and counted, and the writer thread reports the count with the
//! next line it writes.

use std::io::{self, Write};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{sync_channel, SyncSender, TrySendError};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

/// Lines waiting for the writer thread (about 150 B each).
pub const QUEUE_LINES: usize = 1024;

enum Msg {
    Line(Vec<u8>),
    Flush(SyncSender<()>),
}

/// `MakeWriter` for a tracing fmt layer; cheap to clone.
#[derive(Clone)]
pub struct NonBlocking {
    tx: SyncSender<Msg>,
    dropped: Arc<AtomicU64>,
}

fn registry() -> &'static Mutex<Vec<NonBlocking>> {
    static ALL: OnceLock<Mutex<Vec<NonBlocking>>> = OnceLock::new();
    ALL.get_or_init(Default::default)
}

impl NonBlocking {
    /// Starts a writer thread named `name` that owns `sink`. The writer is registered for
    /// [`flush_all`].
    pub fn spawn(name: &str, sink: impl Write + Send + 'static) -> io::Result<Self> {
        let w = Self::spawn_unregistered(name, sink, QUEUE_LINES)?;
        registry().lock().unwrap_or_else(|e| e.into_inner()).push(w.clone());
        Ok(w)
    }

    fn spawn_unregistered(name: &str, mut sink: impl Write + Send + 'static, capacity: usize) -> io::Result<Self> {
        let (tx, rx) = sync_channel::<Msg>(capacity);
        let dropped = Arc::new(AtomicU64::new(0));
        let dropped_w = dropped.clone();
        std::thread::Builder::new().name(name.into()).spawn(move || {
            while let Ok(msg) = rx.recv() {
                match msg {
                    Msg::Line(line) => {
                        let lost = dropped_w.swap(0, Ordering::Relaxed);
                        if lost > 0 {
                            let _ = writeln!(sink, "armd log: {lost} lines dropped (log writer busy)");
                        }
                        let _ = sink.write_all(&line);
                    }
                    Msg::Flush(done) => {
                        let _ = sink.flush();
                        let _ = done.send(());
                    }
                }
            }
            let _ = sink.flush();
        })?;
        Ok(Self { tx, dropped })
    }

    /// Lines dropped since the writer last reported them.
    pub fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    /// Waits until every line queued before this call is written, at most `timeout`.
    /// Returns false on timeout.
    pub fn flush(&self, timeout: Duration) -> bool {
        let deadline = Instant::now() + timeout;
        let (done_tx, done_rx) = sync_channel(1);
        let mut msg = Msg::Flush(done_tx);
        loop {
            match self.tx.try_send(msg) {
                Ok(()) => break,
                Err(TrySendError::Full(m)) if Instant::now() < deadline => {
                    msg = m;
                    std::thread::sleep(Duration::from_millis(2));
                }
                Err(_) => return false,
            }
        }
        done_rx.recv_timeout(deadline.saturating_duration_since(Instant::now())).is_ok()
    }
}

/// Flushes every writer started with [`NonBlocking::spawn`] (before the process exits).
pub fn flush_all(timeout: Duration) {
    let all = registry().lock().unwrap_or_else(|e| e.into_inner()).clone();
    for w in all {
        w.flush(timeout);
    }
}

/// Calls [`flush_all`] with this timeout when dropped.
pub struct FlushOnDrop(pub Duration);

impl Drop for FlushOnDrop {
    fn drop(&mut self) {
        flush_all(self.0);
    }
}

/// One formatted event; tracing-subscriber's fmt layer writes it with one `write_all`.
pub struct LineWriter<'a>(&'a NonBlocking);

impl Write for LineWriter<'_> {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        if self.0.tx.try_send(Msg::Line(buf.to_vec())).is_err() {
            self.0.dropped.fetch_add(1, Ordering::Relaxed);
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl<'a> tracing_subscriber::fmt::MakeWriter<'a> for NonBlocking {
    type Writer = LineWriter<'a>;
    fn make_writer(&'a self) -> Self::Writer {
        LineWriter(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    /// A sink that blocks every write until `gate` is released.
    #[derive(Clone)]
    struct Gated {
        gate: Arc<(Mutex<bool>, std::sync::Condvar)>,
        out: Arc<Mutex<Vec<u8>>>,
    }
    impl Write for Gated {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            let (m, cv) = &*self.gate;
            let mut open = m.lock().unwrap();
            while !*open { open = cv.wait(open).unwrap(); }
            self.out.lock().unwrap().extend_from_slice(buf);
            Ok(buf.len())
        }
        fn flush(&mut self) -> io::Result<()> { Ok(()) }
    }
    fn gated() -> Gated {
        Gated { gate: Arc::new((Mutex::new(false), std::sync::Condvar::new())), out: Arc::default() }
    }
    fn open(g: &Gated) {
        *g.gate.0.lock().unwrap() = true;
        g.gate.1.notify_all();
    }

    #[test]
    fn a_blocked_sink_does_not_block_the_logger_and_overflow_is_counted() {
        let sink = gated();
        let w = NonBlocking::spawn_unregistered("t-log", sink.clone(), 4).unwrap();
        let logged = Instant::now();
        for i in 0..20 {
            tracing_subscriber::fmt::MakeWriter::make_writer(&w).write_all(format!("line {i}\n").as_bytes()).unwrap();
        }
        assert!(logged.elapsed() < Duration::from_millis(500), "logging waited for the sink: {:?}", logged.elapsed());
        assert!(w.dropped() >= 15, "queue of 4 (+1 in the writer): {} dropped", w.dropped());
        open(&sink);
        assert!(w.flush(Duration::from_secs(2)));
        // The count goes out with the next line the writer takes, which may be the first one.
        tracing_subscriber::fmt::MakeWriter::make_writer(&w).write_all(b"after\n").unwrap();
        assert!(w.flush(Duration::from_secs(2)));
        let text = String::from_utf8(sink.out.lock().unwrap().clone()).unwrap();
        assert!(text.ends_with("after\n"), "{text}");
        let written: Vec<usize> = text.lines().filter_map(|l| l.strip_prefix("line ")).map(|n| n.parse().unwrap()).collect();
        let reported: u64 = text.lines().filter_map(|l| l.strip_prefix("armd log: ")?.strip_suffix(" lines dropped (log writer busy)")).map(|n| n.parse::<u64>().unwrap()).sum();
        assert!(written.windows(2).all(|p| p[0] < p[1]), "{text}");
        assert_eq!(written.len() as u64 + reported, 20, "every line is written or counted: {text}");
        assert_eq!(w.dropped(), 0);
    }

    /// The path the control tick takes (`tracing::info!` inside `finish_action`) through a
    /// fmt layer: it returns while the sink is blocked, and each event arrives as one line.
    #[test]
    fn a_tracing_event_does_not_wait_for_a_blocked_sink() {
        use tracing_subscriber::layer::SubscriberExt;
        let sink = gated();
        let w = NonBlocking::spawn_unregistered("t-log3", sink.clone(), 16).unwrap();
        let subscriber = tracing_subscriber::registry().with(tracing_subscriber::fmt::layer().with_ansi(false).with_writer(w.clone()));
        let t0 = Instant::now();
        tracing::subscriber::with_default(subscriber, || {
            for i in 0..3 {
                tracing::info!("Action {i} (joint_move) done");
            }
        });
        assert!(t0.elapsed() < Duration::from_millis(500), "the event waited for the sink: {:?}", t0.elapsed());
        open(&sink);
        assert!(w.flush(Duration::from_secs(2)));
        let text = String::from_utf8(sink.out.lock().unwrap().clone()).unwrap();
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines.len(), 3, "{text}");
        for (i, l) in lines.iter().enumerate() {
            assert!(l.ends_with(&format!("Action {i} (joint_move) done")), "{l}");
        }
    }

    #[test]
    fn lines_keep_their_order() {
        let sink = gated();
        open(&sink);
        let w = NonBlocking::spawn_unregistered("t-log2", sink.clone(), 64).unwrap();
        for i in 0..50 {
            tracing_subscriber::fmt::MakeWriter::make_writer(&w).write_all(format!("{i}\n").as_bytes()).unwrap();
        }
        assert!(w.flush(Duration::from_secs(2)));
        let text = String::from_utf8(sink.out.lock().unwrap().clone()).unwrap();
        let want: String = (0..50).map(|i| format!("{i}\n")).collect();
        assert_eq!(text, want);
    }
}
