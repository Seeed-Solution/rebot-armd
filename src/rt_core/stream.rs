//! Bounded lock-free off-RT SPSC streams over `rtrb` (executor services layer, item 3).
//!
//! Zero-allocation, zero-lock streaming of `Copy` records from `armd-control` to an off-RT
//! consumer (recording samples, grasp diagnostics patches, future terminal/shadow migrations).
//!
//! - **Drop-newest overflow**: full ring or closed consumer drops the newest sample without
//!   waiting or allocating, increments loss counters, and advances the producer's monotonic
//!   sequence so the consumer observes exact [`StreamSample::gap_before`].
//! - **Off-RT construction & destruction**: construct via [`channel`] off-RT. Dropping a
//!   [`StreamProducer`] after [`StreamConsumer`] frees the ring and counters, so any producer
//!   installed on `armd-control` must live for the owner's lifetime or be reclaimed off-thread.
//! - **`Sync` producer**: all `rtrb::Producer` access requires `&mut self`; `&self` only reads
//!   immutable `capacity` and atomic counters, so `StreamProducer<T>` embeds in `SupervisorInner`.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;

use rtrb::{Consumer, Producer, PushError, RingBuffer};

#[derive(Debug, Clone, Copy)]
struct StreamFrame<T: Copy> {
    seq: u64,
    payload: T,
}

#[derive(Debug, Default)]
struct StreamCounters {
    pushed: AtomicU64,
    dropped: AtomicU64,
    dropped_full: AtomicU64,
    dropped_closed: AtomicU64,
    popped: AtomicU64,
    sequence_gaps: AtomicU64,
    lost_in_gaps: AtomicU64,
    producer_closed: AtomicBool,
    consumer_closed: AtomicBool,
}

impl StreamCounters {
    fn snapshot(&self, capacity: usize) -> StreamStats {
        StreamStats {
            capacity,
            pushed: self.pushed.load(Ordering::Acquire),
            dropped: self.dropped.load(Ordering::Acquire),
            dropped_full: self.dropped_full.load(Ordering::Acquire),
            dropped_closed: self.dropped_closed.load(Ordering::Acquire),
            popped: self.popped.load(Ordering::Acquire),
            sequence_gaps: self.sequence_gaps.load(Ordering::Acquire),
            lost_in_gaps: self.lost_in_gaps.load(Ordering::Acquire),
            producer_closed: self.producer_closed.load(Ordering::Acquire),
            consumer_closed: self.consumer_closed.load(Ordering::Acquire),
        }
    }
}

/// Reason [`StreamProducer::try_push`] dropped the newest sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StreamPushError {
    Full,
    Closed,
}

/// One sample popped by [`StreamConsumer::try_pop`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StreamSample<T: Copy> {
    pub seq: u64,
    pub gap_before: u64,
    pub payload: T,
}

/// Lock-free counter snapshot for a stream channel.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct StreamStats {
    pub capacity: usize,
    pub pushed: u64,
    pub dropped: u64,
    pub dropped_full: u64,
    pub dropped_closed: u64,
    pub popped: u64,
    pub sequence_gaps: u64,
    pub lost_in_gaps: u64,
    pub producer_closed: bool,
    pub consumer_closed: bool,
}

/// Single-producer RT-safe sender for an off-RT stream.
pub struct StreamProducer<T: Copy> {
    tx: Producer<StreamFrame<T>>,
    next_seq: u64,
    pushed: u64,
    dropped_full: u64,
    dropped_closed: u64,
    capacity: usize,
    counters: Arc<StreamCounters>,
}

// SAFETY: `self.tx` is only ever accessed through `&mut self`; `&self` reads only immutable
// `capacity` and atomic `counters`.
unsafe impl<T: Copy + Send> Sync for StreamProducer<T> {}

impl<T: Copy> StreamProducer<T> {
    /// Non-blocking, zero-alloc push; returns assigned 1-based `seq` or drop-newest error.
    #[inline]
    pub fn try_push(&mut self, payload: T) -> Result<u64, StreamPushError> {
        let seq = self.next_seq;
        self.next_seq = seq.saturating_add(1);
        if self.counters.consumer_closed.load(Ordering::Acquire) || self.tx.is_abandoned() {
            self.dropped_closed = self.dropped_closed.saturating_add(1);
            let total = self.dropped_full.saturating_add(self.dropped_closed);
            self.counters.dropped_closed.store(self.dropped_closed, Ordering::Release);
            self.counters.dropped.store(total, Ordering::Release);
            return Err(StreamPushError::Closed);
        }
        match self.tx.push(StreamFrame { seq, payload }) {
            Ok(()) => {
                self.pushed = self.pushed.saturating_add(1);
                self.counters.pushed.store(self.pushed, Ordering::Release);
                Ok(seq)
            }
            Err(PushError::Full(_)) => {
                self.dropped_full = self.dropped_full.saturating_add(1);
                let total = self.dropped_full.saturating_add(self.dropped_closed);
                self.counters.dropped_full.store(self.dropped_full, Ordering::Release);
                self.counters.dropped.store(total, Ordering::Release);
                Err(StreamPushError::Full)
            }
        }
    }

    #[inline]
    pub fn capacity(&self) -> usize { self.capacity }

    #[inline]
    pub fn is_closed(&self) -> bool { self.counters.consumer_closed.load(Ordering::Acquire) }

    #[inline]
    pub fn stats(&self) -> StreamStats { self.counters.snapshot(self.capacity) }
}

impl<T: Copy> Drop for StreamProducer<T> {
    fn drop(&mut self) {
        self.counters.producer_closed.store(true, Ordering::Release);
    }
}

/// Single-consumer off-RT receiver for a stream.
pub struct StreamConsumer<T: Copy> {
    rx: Consumer<StreamFrame<T>>,
    expected_seq: u64,
    popped: u64,
    sequence_gaps: u64,
    lost_in_gaps: u64,
    capacity: usize,
    counters: Arc<StreamCounters>,
}

impl<T: Copy> StreamConsumer<T> {
    #[inline]
    pub fn try_pop(&mut self) -> Option<StreamSample<T>> {
        let frame = self.rx.pop().ok()?;
        let gap_before = frame.seq.saturating_sub(self.expected_seq);
        if gap_before > 0 {
            self.sequence_gaps = self.sequence_gaps.saturating_add(1);
            self.lost_in_gaps = self.lost_in_gaps.saturating_add(gap_before);
            self.counters.sequence_gaps.store(self.sequence_gaps, Ordering::Release);
            self.counters.lost_in_gaps.store(self.lost_in_gaps, Ordering::Release);
        }
        self.expected_seq = frame.seq.saturating_add(1);
        self.popped = self.popped.saturating_add(1);
        self.counters.popped.store(self.popped, Ordering::Release);
        Some(StreamSample { seq: frame.seq, gap_before, payload: frame.payload })
    }

    pub fn drain_up_to(&mut self, max: usize, mut f: impl FnMut(StreamSample<T>)) -> usize {
        let mut n = 0;
        while n < max {
            let Some(sample) = self.try_pop() else { break };
            f(sample);
            n += 1;
        }
        n
    }

    #[inline]
    pub fn capacity(&self) -> usize { self.capacity }

    #[inline]
    pub fn is_producer_closed(&self) -> bool { self.counters.producer_closed.load(Ordering::Acquire) }

    #[inline]
    pub fn stats(&self) -> StreamStats { self.counters.snapshot(self.capacity) }
}

impl<T: Copy> Drop for StreamConsumer<T> {
    fn drop(&mut self) {
        self.counters.consumer_closed.store(true, Ordering::Release);
    }
}

/// Creates a preallocated lock-free SPSC stream with capacity `cap.max(1)` (call off-RT).
pub fn channel<T: Copy>(cap: usize) -> (StreamProducer<T>, StreamConsumer<T>) {
    let capacity = cap.max(1);
    let (tx, rx) = RingBuffer::new(capacity);
    let counters = Arc::new(StreamCounters::default());
    let producer = StreamProducer {
        tx,
        next_seq: 1,
        pushed: 0,
        dropped_full: 0,
        dropped_closed: 0,
        capacity,
        counters: Arc::clone(&counters),
    };
    let consumer = StreamConsumer {
        rx,
        expected_seq: 1,
        popped: 0,
        sequence_gaps: 0,
        lost_in_gaps: 0,
        capacity,
        counters,
    };
    (producer, consumer)
}
