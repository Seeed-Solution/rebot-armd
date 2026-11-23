//! Latest-target mailbox: servo / teleop / policy targets from the broker to the RT
//! (armd 1.6 RT core, step 2 foundation; design section 3, "Urgent control and servo").
//!
//! One writer (the broker, normal priority) and one reader (the RT). The reader takes the
//! newest unconsumed frame; a newer frame replaces an unconsumed one only when that one is
//! replaceable ([`TargetFrame::replaceable`]), otherwise the submit is refused (`Busy`). The
//! writer refuses a frame whose stream sequence does not advance (duplicate or wrapped).
//! Frames are `Copy` with fixed-size joint arrays: neither side allocates.
//!
//! The RT rechecks a frame's stream, lease, stop/epoch and policy generation at consumption
//! ([`TargetFrame::authority`], track D); the mailbox carries the original input and vision
//! times unchanged.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Instant;

use super::broker::Authority;
use super::triple::{triple_buffer, Consumer, Producer};

/// Joints a frame carries at most.
pub const MAX_DOF: usize = 8;

/// A target stream (one WS servo stream, one policy run).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct StreamId(pub u64);

/// One target.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TargetFrame {
    pub stream: StreamId,
    /// Stream sequence; must increase within a stream.
    pub seq: u64,
    pub authority: Authority,
    /// Policy authorization generation (0: not a policy frame).
    pub policy_generation: u64,
    /// When the input arrived (WS receive, policy observation).
    pub input_at: Instant,
    /// Camera sample time behind the frame (vision policies).
    pub vision_at: Option<Instant>,
    pub dof: u8,
    pub q: [f32; MAX_DOF],
    /// The protocol lets a newer frame replace this one before it is consumed.
    pub replaceable: bool,
}

impl TargetFrame {
    pub fn positions(&self) -> &[f32] { &self.q[..usize::from(self.dof).min(MAX_DOF)] }
}

/// Result of [`TargetWriter::submit`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Submitted {
    Accepted,
    /// Accepted and replaced the unconsumed frame with this sequence.
    Coalesced { replaced: u64 },
    /// Refused: the unconsumed frame is not replaceable.
    Busy,
    /// Refused: the sequence does not advance on this stream (duplicate or wrapped).
    OutOfOrder,
}

#[derive(Clone, Copy)]
struct Slot {
    /// Publication number of the frame (0: none).
    n: u64,
    frame: Option<TargetFrame>,
}

/// The broker's side.
pub struct TargetWriter {
    producer: Producer<Slot>,
    published: u64,
    last: Option<TargetFrame>,
    consumed: Arc<AtomicU64>,
    coalesced: u64,
}

/// The RT's side.
pub struct TargetReader {
    consumer: Consumer<Slot>,
    consumed: Arc<AtomicU64>,
}

pub fn target_mailbox() -> (TargetWriter, TargetReader) {
    let empty = Slot { n: 0, frame: None };
    let (producer, consumer) = triple_buffer(empty, empty, empty);
    let consumed = Arc::new(AtomicU64::new(0));
    (TargetWriter { producer, published: 0, last: None, consumed: consumed.clone(), coalesced: 0 }, TargetReader { consumer, consumed })
}

impl TargetWriter {
    pub fn submit(&mut self, frame: TargetFrame) -> Submitted {
        let unconsumed = self.consumed.load(Ordering::Acquire) < self.published;
        if let Some(last) = self.last.filter(|l| l.stream == frame.stream) {
            if frame.seq <= last.seq { return Submitted::OutOfOrder; }
        }
        let replaced = match self.last {
            Some(last) if unconsumed && !last.replaceable => return Submitted::Busy,
            Some(last) if unconsumed => Some(last.seq),
            _ => None,
        };
        self.published += 1;
        *self.producer.back_mut() = Slot { n: self.published, frame: Some(frame) };
        self.producer.publish();
        self.last = Some(frame);
        match replaced {
            Some(seq) => { self.coalesced += 1; Submitted::Coalesced { replaced: seq } }
            None => Submitted::Accepted,
        }
    }

    /// Frames replaced before consumption.
    pub fn coalesced(&self) -> u64 { self.coalesced }
}

impl TargetReader {
    /// The newest unconsumed frame (None when nothing new).
    pub fn take(&mut self) -> Option<TargetFrame> {
        if !self.consumer.refresh() { return None; }
        let slot = *self.consumer.front();
        if slot.n == 0 { return None; }
        self.consumed.store(slot.n, Ordering::Release);
        slot.frame
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(stream: u64, seq: u64, replaceable: bool) -> TargetFrame {
        TargetFrame { stream: StreamId(stream), seq, authority: Authority::default(), policy_generation: 0, input_at: Instant::now(),
            vision_at: None, dof: 2, q: [seq as f32; MAX_DOF], replaceable }
    }

    #[test]
    fn a_burst_coalesces_to_the_newest_frame() {
        let (mut w, mut r) = target_mailbox();
        assert_eq!(w.submit(frame(1, 1, true)), Submitted::Accepted);
        assert_eq!(w.submit(frame(1, 2, true)), Submitted::Coalesced { replaced: 1 });
        assert_eq!(w.submit(frame(1, 3, true)), Submitted::Coalesced { replaced: 2 });
        let f = r.take().unwrap();
        assert_eq!((f.seq, f.positions()), (3, &[3.0, 3.0][..]));
        assert_eq!(r.take(), None, "consumed once");
        assert_eq!(w.submit(frame(1, 4, true)), Submitted::Accepted);
        assert_eq!(w.coalesced(), 2);
    }

    #[test]
    fn duplicates_and_wrapped_sequences_are_refused() {
        let (mut w, mut r) = target_mailbox();
        w.submit(frame(1, u64::MAX - 1, true));
        assert_eq!(w.submit(frame(1, u64::MAX - 1, true)), Submitted::OutOfOrder);
        assert_eq!(w.submit(frame(1, 0, true)), Submitted::OutOfOrder, "wrapped");
        assert_eq!(r.take().unwrap().seq, u64::MAX - 1);
        // another stream starts its own sequence
        assert_eq!(w.submit(frame(2, 0, true)), Submitted::Accepted);
    }

    #[test]
    fn an_unconsumed_non_replaceable_frame_is_kept() {
        let (mut w, mut r) = target_mailbox();
        w.submit(frame(1, 1, false));
        assert_eq!(w.submit(frame(1, 2, true)), Submitted::Busy);
        assert_eq!(r.take().unwrap().seq, 1);
        assert_eq!(w.submit(frame(1, 2, true)), Submitted::Accepted);
    }

    #[test]
    fn delayed_consumption_reads_the_newest_frame_and_times_are_kept() {
        let (mut w, mut r) = target_mailbox();
        let mut f = frame(1, 1, true);
        let t = Instant::now() - std::time::Duration::from_millis(30);
        f.input_at = t;
        f.vision_at = Some(t);
        w.submit(f);
        std::thread::sleep(std::time::Duration::from_millis(2));
        let got = r.take().unwrap();
        assert_eq!((got.input_at, got.vision_at), (t, Some(t)));
    }
}
