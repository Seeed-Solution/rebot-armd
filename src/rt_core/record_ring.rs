//! Recording ring: the control tick appends one sample per tick of a recording into a
//! preallocated single-producer single-consumer ring; the dispatcher thread (or the stop
//! path) drains it into the episode's frames off the tick.
//!
//! - Capacity [`RECORD_RING_CAPACITY`] samples: 5.12 s at 50 Hz between drains.
//! - Overflow: when the ring is full the tick drops the sample and counts it; the session
//!   is then incomplete ([`Finished::dropped`] > 0). Nothing waits.
//! - Fences: [`RecordProducer::begin`] and [`RecordProducer::end`] run under the
//!   supervisor write lock, so no tick runs between a fence and the state change it marks;
//!   neither takes a mutex. Every sample carries its session number and the tick's
//!   publication sequence; a session contains exactly the samples its ticks pushed between
//!   the two fences. [`Fence::collect`] assembles them after the lock is released.

use std::cell::UnsafeCell;
use std::collections::VecDeque;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use crate::recording::RecordFrame;

pub const RECORD_RING_CAPACITY: usize = 256;

struct RecSlot {
    session: u64,
    tick_seq: u64,
    timestamp: f64,
    state: Vec<f32>,
    action: Vec<f32>,
}

/// Frames of one session, assembled by the consumer.
struct SessionFrames {
    session: u64,
    frames: Vec<RecordFrame>,
    first_tick: Option<u64>,
    last_tick: Option<u64>,
}

/// Consumer state: sessions in start order (normally one; more when a new session starts
/// before the previous one is collected). Each [`Fence::collect`] removes only its own
/// session, so fences can be collected in any order.
#[derive(Default)]
struct Assembly {
    sessions: VecDeque<SessionFrames>,
    /// Sessions evicted beyond [`MAX_PENDING_SESSIONS`] and how many of their samples were
    /// discarded (counted as dropped when their fence is collected), the newest
    /// [`MAX_PENDING_SESSIONS`] of them.
    evicted: VecDeque<(u64, u64)>,
}

/// Uncollected sessions kept by the consumer; beyond it the oldest is discarded (a leak bound
/// for fences dropped without [`Fence::collect`]; every armd path collects its fence).
pub const MAX_PENDING_SESSIONS: usize = 4;

/// Ring storage shared by the producer (the tick) and the consumer side.
pub struct RecordRing {
    slots: Box<[UnsafeCell<RecSlot>]>,
    /// Samples written so far (producer; Release).
    head: AtomicU64,
    /// Samples consumed so far (consumer; Release). Written only with `assembly` held.
    tail: AtomicU64,
    /// Consumer state; the mutex also makes the consumer side single.
    assembly: Mutex<Assembly>,
}

// SAFETY: slot `i % CAP` is written by the producer only while `i >= tail` and
// `i < tail + CAP` (it checks `head - tail < CAP` with an Acquire load of `tail`), and read by
// the consumer only while `i < head` (Acquire load of `head`); the Release stores of `head`
// and `tail` order the accesses. The consumer side runs with `assembly` held.
unsafe impl Sync for RecordRing {}
unsafe impl Send for RecordRing {}

impl RecordRing {
    fn new(dof: usize) -> Self {
        let slots = (0..RECORD_RING_CAPACITY)
            .map(|_| UnsafeCell::new(RecSlot { session: 0, tick_seq: 0, timestamp: 0.0, state: Vec::with_capacity(dof), action: Vec::with_capacity(dof) }))
            .collect();
        Self {
            slots,
            head: AtomicU64::new(0),
            tail: AtomicU64::new(0),
            assembly: Mutex::new(Assembly::default()),
        }
    }

    /// Holds the consumer mutex (tests: a preempted dispatcher).
    #[doc(hidden)]
    pub fn consumer_guard(&self) -> impl Drop + '_ { self.assembly.lock().unwrap_or_else(|e| e.into_inner()) }

    /// Moves the pushed samples of the current session into its frames (allocates; never
    /// call it from the tick). Returns how many samples it consumed.
    pub fn drain(&self) -> usize {
        let mut asm = self.assembly.lock().unwrap_or_else(|e| e.into_inner());
        self.drain_locked(&mut asm)
    }

    fn drain_locked(&self, asm: &mut Assembly) -> usize {
        let head = self.head.load(Ordering::Acquire);
        let mut tail = self.tail.load(Ordering::Relaxed);
        let n = head.saturating_sub(tail) as usize;
        while tail < head {
            // SAFETY: tail < head: the producer finished this slot and does not reuse it
            // before `tail` passes it (see the Sync impl).
            let slot = unsafe { &*self.slots[(tail % RECORD_RING_CAPACITY as u64) as usize].get() };
            if let Some(e) = asm.evicted.iter_mut().find(|e| e.0 == slot.session) {
                // a late sample of an evicted session: discarded, counted
                e.1 += 1;
            } else if slot.session != 0 {
                if asm.sessions.back().is_none_or(|s| s.session != slot.session) {
                    if asm.sessions.len() >= MAX_PENDING_SESSIONS {
                        let old = asm.sessions.pop_front().expect("len >= MAX_PENDING_SESSIONS");
                        if asm.evicted.len() >= MAX_PENDING_SESSIONS { asm.evicted.pop_front(); }
                        asm.evicted.push_back((old.session, old.frames.len() as u64));
                    }
                    asm.sessions.push_back(SessionFrames { session: slot.session, frames: Vec::new(), first_tick: None, last_tick: None });
                }
                let s = asm.sessions.back_mut().expect("pushed above");
                s.frames.push(RecordFrame { timestamp: slot.timestamp, observation_state: slot.state.clone(), action: slot.action.clone() });
                s.first_tick.get_or_insert(slot.tick_seq);
                s.last_tick = Some(slot.tick_seq);
            }
            tail += 1;
        }
        self.tail.store(tail, Ordering::Release);
        n
    }
}

/// A finished session.
#[derive(Debug, Default)]
pub struct Finished {
    pub frames: Vec<RecordFrame>,
    /// Samples the tick dropped on a full ring: > 0 means the session is incomplete.
    pub dropped: u64,
    /// Publication sequence (`SampleMeta::seq`) of the first and last included tick.
    pub first_tick: Option<u64>,
    pub last_tick: Option<u64>,
}

/// The tick's side, owned by `ArmCore` (used under the supervisor write lock).
pub struct RecordProducer {
    dof: usize,
    ring: Option<Arc<RecordRing>>,
    head: u64,
    /// Current session (0: none).
    session: u64,
    next_session: u64,
    pushed: usize,
    dropped: u64,
}

impl RecordProducer {
    pub fn new(dof: usize) -> Self {
        Self { dof, ring: None, head: 0, session: 0, next_session: 1, pushed: 0, dropped: 0 }
    }

    /// The ring, for the consumer side (None until the first session).
    pub fn ring(&self) -> Option<&Arc<RecordRing>> { self.ring.as_ref() }

    /// Start fence: the next pushed sample is the session's first. Allocates the ring on the
    /// first session (not in the tick); takes no mutex.
    pub fn begin(&mut self) -> u64 {
        if self.ring.is_none() { self.ring = Some(Arc::new(RecordRing::new(self.dof))); }
        let session = self.next_session;
        self.next_session += 1;
        self.session = session;
        self.pushed = 0;
        self.dropped = 0;
        session
    }

    /// True while a session runs.
    pub fn active(&self) -> bool { self.session != 0 }

    /// Samples the ticks pushed in this session (dropped ones not counted).
    pub fn pushed(&self) -> usize { self.pushed }

    pub fn dropped(&self) -> u64 { self.dropped }

    /// Appends one sample of the running session; false when none runs or the ring is full
    /// (counted in [`dropped`](Self::dropped)). No allocation while `state` and `action` fit
    /// the profile's dof; never waits.
    pub fn push(&mut self, tick_seq: u64, timestamp: f64, state: &[f32], action: &[f32]) -> bool {
        let (Some(ring), true) = (self.ring.as_ref(), self.session != 0) else { return false; };
        let tail = ring.tail.load(Ordering::Acquire);
        if self.head - tail >= RECORD_RING_CAPACITY as u64 {
            self.dropped += 1;
            return false;
        }
        // SAFETY: head - tail < CAP: the consumer does not read this slot until `head` passes it.
        let slot = unsafe { &mut *ring.slots[(self.head % RECORD_RING_CAPACITY as u64) as usize].get() };
        slot.session = self.session;
        slot.tick_seq = tick_seq;
        slot.timestamp = timestamp;
        slot.state.clear();
        slot.state.extend_from_slice(state);
        slot.action.clear();
        slot.action.extend_from_slice(action);
        self.head += 1;
        ring.head.store(self.head, Ordering::Release);
        self.pushed += 1;
        true
    }

    /// Stop fence: ends the session (no tick pushes into it afterwards) without a mutex
    /// or an allocation; [`Fence::collect`] returns its frames. None when no session runs.
    pub fn end(&mut self) -> Option<Fence> {
        let ring = self.ring.clone().filter(|_| self.session != 0)?;
        let fence = Fence { ring, session: self.session, head: self.head, dropped: self.dropped };
        self.session = 0;
        Some(fence)
    }

    /// [`end`](Self::end) and [`Fence::collect`] at once (tests).
    pub fn finish(&mut self) -> Finished {
        self.end().map(Fence::collect).unwrap_or_default()
    }
}

/// A session ended by [`RecordProducer::end`]: the samples its ticks pushed.
#[must_use = "collect the fence (off the tick, without the supervisor lock) or its frames stay in the ring's consumer"]
pub struct Fence {
    ring: Arc<RecordRing>,
    session: u64,
    /// Samples written to the ring when the session ended.
    head: u64,
    dropped: u64,
}

impl Fence {
    /// The session's frames (drains the ring up to the fence and beyond; allocates and takes
    /// the consumer mutex, which a preempted dispatcher can hold: call it on a blocking thread,
    /// never on an async worker or the tick, and without the supervisor lock). Removes only
    /// this session's frames: other sessions' fences can be collected before or after it.
    pub fn collect(self) -> Finished {
        let mut asm = self.ring.assembly.lock().unwrap_or_else(|e| e.into_inner());
        self.ring.drain_locked(&mut asm);
        debug_assert!(self.ring.tail.load(Ordering::Relaxed) >= self.head);
        let mut done = Finished { dropped: self.dropped, ..Finished::default() };
        if let Some(i) = asm.sessions.iter().position(|s| s.session == self.session) {
            let s = asm.sessions.remove(i).expect("position is in range");
            done.frames = s.frames;
            done.first_tick = s.first_tick;
            done.last_tick = s.last_tick;
        } else if let Some(i) = asm.evicted.iter().position(|e| e.0 == self.session) {
            // evicted while uncollected: its samples are lost, the session is incomplete
            done.dropped += asm.evicted.remove(i).expect("position is in range").1;
        }
        done
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_session_holds_exactly_the_samples_between_its_fences() {
        let mut p = RecordProducer::new(2);
        assert!(!p.push(1, 0.0, &[1.0, 1.0], &[1.0, 1.0]), "no session");
        p.begin();
        for i in 2..10u64 {
            assert!(p.push(i, i as f64, &[i as f32, 0.0], &[0.0, i as f32]));
            if i == 5 { p.ring().unwrap().drain(); }
        }
        let done = p.finish();
        assert_eq!(done.frames.len(), 8);
        assert_eq!((done.first_tick, done.last_tick, done.dropped), (Some(2), Some(9), 0));
        assert_eq!(done.frames[3], RecordFrame { timestamp: 5.0, observation_state: vec![5.0, 0.0], action: vec![0.0, 5.0] });
        assert!(!p.push(10, 10.0, &[0.0; 2], &[0.0; 2]), "after the stop fence");
        // a new session starts empty
        p.begin();
        p.push(11, 11.0, &[0.0; 2], &[0.0; 2]);
        assert_eq!(p.finish().frames.len(), 1);
    }

    #[test]
    fn a_session_is_collected_after_the_next_one_began() {
        let mut p = RecordProducer::new(1);
        p.begin();
        p.push(1, 1.0, &[1.0], &[1.0]);
        let fence = p.end().unwrap();
        p.begin();
        p.push(2, 2.0, &[2.0], &[2.0]);
        p.ring().unwrap().drain();
        let first = fence.collect();
        assert_eq!((first.frames.len(), first.first_tick), (1, Some(1)));
        let second = p.finish();
        assert_eq!((second.frames.len(), second.first_tick), (1, Some(2)));
        assert!(p.end().is_none());
    }

    #[test]
    fn fences_are_collected_in_any_order() {
        let mut p = RecordProducer::new(1);
        p.begin();
        p.push(1, 1.0, &[1.0], &[1.0]);
        p.push(2, 1.1, &[1.0], &[1.0]);
        let a = p.end().unwrap();
        p.begin();
        p.push(3, 2.0, &[2.0], &[2.0]);
        let b = p.end().unwrap();
        // B first: A's frames stay with the consumer
        let second = b.collect();
        assert_eq!((second.frames.len(), second.first_tick, second.last_tick), (1, Some(3), Some(3)));
        let first = a.collect();
        assert_eq!((first.frames.len(), first.first_tick, first.last_tick), (2, Some(1), Some(2)));
    }

    #[test]
    fn uncollected_sessions_are_bounded() {
        let mut p = RecordProducer::new(1);
        let mut kept = Vec::new();
        for k in 0..(MAX_PENDING_SESSIONS as u64 + 2) {
            p.begin();
            p.push(k + 1, k as f64, &[0.0], &[0.0]);
            kept.push(p.end().unwrap());
            p.ring().unwrap().drain();
        }
        // the two oldest were discarded; the newest four are intact
        let mut sizes: Vec<usize> = kept.into_iter().map(|f| f.collect().frames.len()).collect();
        sizes.reverse();
        assert_eq!(sizes, vec![1, 1, 1, 1, 0, 0]);
    }

    #[test]
    fn overflow_is_counted_and_never_blocks() {
        let mut p = RecordProducer::new(1);
        p.begin();
        let n = RECORD_RING_CAPACITY + 10;
        let ok = (0..n as u64).filter(|i| p.push(*i, 0.0, &[0.0], &[0.0])).count();
        assert_eq!(ok, RECORD_RING_CAPACITY);
        assert_eq!(p.dropped(), 10);
        p.ring().unwrap().drain();
        assert!(p.push(999, 0.0, &[0.0], &[0.0]), "room again after a drain");
        let done = p.finish();
        assert_eq!((done.frames.len(), done.dropped), (RECORD_RING_CAPACITY + 1, 10));
    }

    #[test]
    fn concurrent_drain_keeps_order_and_count() {
        let mut p = RecordProducer::new(3);
        p.begin();
        let ring = p.ring().unwrap().clone();
        let stop = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let drainer = {
            let (ring, stop) = (ring.clone(), stop.clone());
            std::thread::spawn(move || while !stop.load(Ordering::Relaxed) { ring.drain(); std::thread::yield_now(); })
        };
        let mut pushed = 0u64;
        for i in 0..20_000u64 {
            if p.push(i, i as f64, &[i as f32; 3], &[i as f32; 3]) { pushed += 1; }
        }
        stop.store(true, Ordering::Relaxed);
        drainer.join().unwrap();
        let done = p.finish();
        assert_eq!(done.frames.len() as u64, pushed);
        assert_eq!(pushed + done.dropped, 20_000);
        assert!(done.frames.windows(2).all(|w| w[0].timestamp < w[1].timestamp));
        assert!(done.frames.iter().all(|f| f.observation_state.iter().all(|v| *v as f64 == f.timestamp)));
    }

    #[test]
    fn a_session_evicted_before_its_fence_is_collected_counts_its_samples_as_dropped() {
        let mut p = RecordProducer::new(1);
        let mut fences = Vec::new();
        for k in 0..=MAX_PENDING_SESSIONS as u64 {
            p.begin();
            for i in 0..3 { assert!(p.push(10 * k + i, 0.0, &[0.0], &[0.0])); }
            fences.push(p.end().unwrap());
        }
        // the oldest of five uncollected sessions is evicted when the fifth one is drained
        let mut done: Vec<Finished> = fences.into_iter().rev().map(Fence::collect).collect();
        done.reverse();
        let first = &done[0];
        assert_eq!((first.frames.len(), first.dropped), (0, 3), "the evicted session is reported incomplete");
        for d in &done[1..] { assert_eq!((d.frames.len(), d.dropped), (3, 0)); }
    }
}
