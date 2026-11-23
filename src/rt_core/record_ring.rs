//! Recording ring: the control tick appends one sample per tick of a recording into a
//! preallocated single-producer single-consumer ring; the dispatcher thread (or the stop
//! path) drains it into the episode's frames off the tick.
//!
//! - Capacity [`RECORD_RING_CAPACITY`] samples: 5.12 s at 50 Hz between drains.
//! - Overflow: when the ring is full the tick drops the sample and counts it; the session
//!   is then incomplete ([`Finished::dropped`] > 0). Nothing waits.
//! - Fences: [`RecordProducer::begin`] and [`RecordProducer::finish`] run under the
//!   supervisor write lock, so no tick runs between a fence and the state change it marks.
//!   Every sample carries its session number and the tick's publication sequence; a session
//!   contains exactly the samples its ticks pushed between the two fences.

use std::cell::UnsafeCell;
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

struct Assembly {
    /// Session whose samples are kept; others are skipped.
    session: u64,
    frames: Vec<RecordFrame>,
    first_tick: Option<u64>,
    last_tick: Option<u64>,
}

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
            assembly: Mutex::new(Assembly { session: 0, frames: Vec::new(), first_tick: None, last_tick: None }),
        }
    }

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
            if slot.session == asm.session && asm.session != 0 {
                asm.frames.push(RecordFrame { timestamp: slot.timestamp, observation_state: slot.state.clone(), action: slot.action.clone() });
                asm.first_tick.get_or_insert(slot.tick_seq);
                asm.last_tick = Some(slot.tick_seq);
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
    /// first session (not in the tick). Discards what an unfinished session left.
    pub fn begin(&mut self) -> u64 {
        let ring = self.ring.get_or_insert_with(|| Arc::new(RecordRing::new(self.dof))).clone();
        let session = self.next_session;
        self.next_session += 1;
        let mut asm = ring.assembly.lock().unwrap_or_else(|e| e.into_inner());
        ring.drain_locked(&mut asm);
        *asm = Assembly { session, frames: Vec::new(), first_tick: None, last_tick: None };
        drop(asm);
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

    /// Stop fence: ends the session and returns its frames (drains the rest of the ring;
    /// allocates, call it off the tick under the write lock). Empty when none runs.
    pub fn finish(&mut self) -> Finished {
        let Some(ring) = self.ring.clone().filter(|_| self.session != 0) else { return Finished::default(); };
        let mut asm = ring.assembly.lock().unwrap_or_else(|e| e.into_inner());
        ring.drain_locked(&mut asm);
        let done = Finished { frames: std::mem::take(&mut asm.frames), dropped: self.dropped, first_tick: asm.first_tick, last_tick: asm.last_tick };
        asm.session = 0;
        drop(asm);
        self.session = 0;
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
}
