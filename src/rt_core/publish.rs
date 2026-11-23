//! State publication: the control tick is the one producer of state samples; readers
//! (REST `GET /observation`, the WS observation stream, the black box, measurement polls)
//! get them through [`StateHub`] without the supervisor lock.
//!
//! # What a published sample means
//!
//! A [`Snapshot`] is the state the control tick computed at the end of one tick, under the
//! supervisor write lock, after that tick's feedback was read and its output was written:
//!
//! - `obs` is exactly what `SupervisorInner::current_observation` returned at that moment;
//!   `obs.timestamp` is its local monotonic sample time ([`super::ids::MonotonicS`]).
//! - `meta.seq` counts publications of this process (1, 2, ...; no gaps).
//! - `meta.generation` is the supervisor lock generation of the tick's write guard
//!   (`SupervisorLock::generation`): every write guard taken later has a higher one, so a
//!   sample answers for the state while the current generation still equals it, and a
//!   reader that saw generation `g` before a request gets read-your-writes from any sample
//!   with `generation >= g`.
//! - `meta.valid_until` is the local deadline after which time-derived fields would flip
//!   without a write (lease expiry, previous-action window, 250 ms feedback freshness behind
//!   `feedback_valid` / `enable_state`; `SupervisorInner::observation_valid_for`), capped at
//!   two control periods.
//! - `rt` holds the per-joint feedback the tick also used (positions, velocities, receive
//!   instants and per-joint feedback sequence numbers), sized for this arm's profile.
//!
//! The tick never waits for a reader: publication is one atomic swap of a preallocated
//! slot ([`super::triple`]). Readers share [`StateHub`]'s mutex with each other only.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::Instant;

use crate::model::ArmObservation;

use super::triple::{triple_buffer, Consumer, Producer};

/// Publication stamp of a sample (see the module docs).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SampleMeta {
    pub seq: u64,
    pub generation: u64,
    pub sampled_at: Instant,
    pub valid_until: Instant,
}

/// Per-joint feedback of a sample; every vector has the profile's dof.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct RtSample {
    pub positions: Vec<f32>,
    pub velocities: Vec<f32>,
    /// Hardware receive instant of each joint's latest feedback.
    pub feedback_instants: Vec<Option<Instant>>,
    /// Per-joint count of distinct feedback samples.
    pub feedback_seqs: Vec<u64>,
}

impl RtSample {
    fn with_dof(dof: usize) -> Self {
        Self {
            positions: Vec::with_capacity(dof),
            velocities: Vec::with_capacity(dof),
            feedback_instants: Vec::with_capacity(dof),
            feedback_seqs: Vec::with_capacity(dof),
        }
    }

    /// `SupervisorInner::arm_feedback_snapshot` of this sample: all positions, and the
    /// velocities, receive instants and feedback sequence numbers of the joints `arm`.
    pub fn arm_feedback(&self, arm: &[usize]) -> (Vec<f32>, Vec<f32>, Vec<Option<Instant>>, Vec<u64>) {
        (
            self.positions.clone(),
            arm.iter().map(|&i| self.velocities.get(i).copied().unwrap_or(0.0)).collect(),
            arm.iter().map(|&i| self.feedback_instants.get(i).copied().flatten()).collect(),
            arm.iter().map(|&i| self.feedback_seqs.get(i).copied().unwrap_or(0)).collect(),
        )
    }
}

/// Copies `src` into `dst` without allocating while `dst` has the capacity.
fn copy_into<T: Copy>(dst: &mut Vec<T>, src: &[T]) {
    dst.clear();
    dst.extend_from_slice(src);
}

/// One preallocated slot of the triple buffer.
struct Slot {
    meta: Option<SampleMeta>,
    obs: Option<ArmObservation>,
    rt: RtSample,
}

/// A sample as readers get it.
#[derive(Debug, Clone)]
pub struct Snapshot {
    pub meta: SampleMeta,
    pub obs: ArmObservation,
    pub rt: RtSample,
}

/// The tick's side: owned by `ArmCore`, used under the supervisor write lock.
pub struct StatePublisher {
    producer: Producer<Slot>,
    seq: u64,
    hub: Arc<StateHub>,
}

/// The per-joint inputs of [`StatePublisher::publish`].
pub struct RtInputs<'a> {
    pub positions: &'a [f32],
    pub velocities: &'a [f32],
    pub feedback_instants: &'a [Option<Instant>],
    pub feedback_seqs: &'a [u64],
}

impl StatePublisher {
    /// Publishes `obs` with its stamp and returns the observation the slot held before (the
    /// caller drops it after releasing the lock). Copies the per-joint inputs into the slot's
    /// preallocated vectors and swaps one atomic: no allocation, no deallocation, no wait
    /// (while the dof does not grow beyond the profile's).
    pub fn publish(&mut self, generation: u64, sampled_at: Instant, valid_until: Instant, obs: ArmObservation, rt: RtInputs<'_>) -> Option<ArmObservation> {
        self.seq += 1;
        let seq = self.seq;
        let slot = self.producer.back_mut();
        slot.meta = Some(SampleMeta { seq, generation, sampled_at, valid_until });
        let displaced = slot.obs.replace(obs);
        copy_into(&mut slot.rt.positions, rt.positions);
        copy_into(&mut slot.rt.velocities, rt.velocities);
        copy_into(&mut slot.rt.feedback_instants, rt.feedback_instants);
        copy_into(&mut slot.rt.feedback_seqs, rt.feedback_seqs);
        self.producer.publish();
        self.hub.published.store(seq, Ordering::Release);
        self.hub.wake();
        displaced
    }

    pub fn hub(&self) -> &Arc<StateHub> { &self.hub }
}

struct HubState {
    consumer: Consumer<Slot>,
    latest: Option<Arc<Snapshot>>,
}

/// The readers' side (the dispatcher): the newest sample, and the wake-up of the dispatcher
/// thread that forwards samples to subscribers.
pub struct StateHub {
    state: Mutex<HubState>,
    /// `seq` of the newest publication (0: none yet).
    published: AtomicU64,
    dispatcher: OnceLock<std::thread::Thread>,
    closed: AtomicBool,
}

/// A publisher for an arm with `dof` joints and its hub.
pub fn state_channel(dof: usize) -> (StatePublisher, Arc<StateHub>) {
    let slot = || Slot { meta: None, obs: None, rt: RtSample::with_dof(dof) };
    let (producer, consumer) = triple_buffer(slot(), slot(), slot());
    let hub = Arc::new(StateHub {
        state: Mutex::new(HubState { consumer, latest: None }),
        published: AtomicU64::new(0),
        dispatcher: OnceLock::new(),
        closed: AtomicBool::new(false),
    });
    (StatePublisher { producer, seq: 0, hub: hub.clone() }, hub)
}

impl StateHub {
    /// The newest published sample (None before the first tick). Takes the hub mutex,
    /// which only readers and the dispatcher share; the first reader after a publication
    /// moves the observation out of the slot (its deallocation then happens on the reader
    /// side when the last `Arc` drops, not in the tick).
    pub fn latest(&self) -> Option<Arc<Snapshot>> {
        let mut st = self.state.lock().unwrap_or_else(|e| e.into_inner());
        if st.consumer.refresh() {
            let slot = st.consumer.front_mut();
            if let (Some(meta), Some(obs)) = (slot.meta, slot.obs.take()) {
                let snap = Snapshot { meta, obs, rt: slot.rt.clone() };
                st.latest = Some(Arc::new(snap));
            }
        }
        st.latest.clone()
    }

    /// `seq` of the newest publication, without the hub mutex (0: none yet).
    pub fn published_seq(&self) -> u64 { self.published.load(Ordering::Acquire) }

    /// Registers the dispatcher thread woken by every publication. Only the first call counts.
    pub fn set_dispatcher(&self, thread: std::thread::Thread) -> bool { self.dispatcher.set(thread).is_ok() }

    pub fn has_dispatcher(&self) -> bool { self.dispatcher.get().is_some() }

    /// Wakes the dispatcher thread: an atomic swap and, when it sleeps, one futex wake.
    pub fn wake(&self) {
        if let Some(t) = self.dispatcher.get() { t.unpark(); }
    }

    /// Ends the dispatcher thread (the supervisor is dropped).
    pub fn close(&self) {
        self.closed.store(true, Ordering::Release);
        self.wake();
    }

    pub fn is_closed(&self) -> bool { self.closed.load(Ordering::Acquire) }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn obs(ts: f64) -> ArmObservation {
        let mut o = crate::supervisor::SupervisorInner::new(crate::ArmConfig::rebot_dm()).current_observation();
        o.timestamp = ts;
        o
    }

    fn inputs<'a>(p: &'a [f32], s: &'a [u64], i: &'a [Option<Instant>]) -> RtInputs<'a> {
        RtInputs { positions: p, velocities: p, feedback_instants: i, feedback_seqs: s }
    }

    #[test]
    fn readers_get_the_newest_sample_with_its_stamp() {
        let (mut publisher, hub) = state_channel(7);
        assert!(hub.latest().is_none());
        let now = Instant::now();
        let p = [0.5f32; 7];
        let s = [3u64; 7];
        let i = [Some(now); 7];
        assert!(publisher.publish(10, now, now + Duration::from_millis(40), obs(1.0), inputs(&p, &s, &i)).is_none());
        assert!(publisher.publish(12, now, now + Duration::from_millis(40), obs(2.0), inputs(&p, &s, &i)).is_none());
        let snap = hub.latest().unwrap();
        assert_eq!((snap.meta.seq, snap.meta.generation, snap.obs.timestamp), (2, 12, 2.0));
        assert_eq!(snap.rt.positions, p);
        assert_eq!(snap.rt.feedback_seqs, s);
        assert_eq!(hub.published_seq(), 2);
        // nothing new: the same Arc
        assert!(Arc::ptr_eq(&snap, &hub.latest().unwrap()));
        // slots rotate: the next publication gets back the unconsumed seq-1 observation;
        // a slot without an observation (initial, or taken by a reader) comes back empty
        let back = publisher.publish(13, now, now, obs(3.0), inputs(&p, &s, &i));
        assert_eq!(back.map(|o| o.timestamp), Some(1.0));
        assert!(publisher.publish(14, now, now, obs(4.0), inputs(&p, &s, &i)).is_none());
        assert_eq!(hub.latest().unwrap().obs.timestamp, 4.0);
    }

    #[test]
    fn arm_feedback_matches_the_supervisor_view() {
        let (mut publisher, hub) = state_channel(3);
        let now = Instant::now();
        publisher.publish(1, now, now, obs(0.0), RtInputs {
            positions: &[1.0, 2.0, 3.0], velocities: &[0.1, 0.2, 0.3],
            feedback_instants: &[Some(now), None, Some(now)], feedback_seqs: &[5, 6, 7],
        });
        let (p, v, i, s) = hub.latest().unwrap().rt.arm_feedback(&[0, 2]);
        assert_eq!((p, v, i, s), (vec![1.0, 2.0, 3.0], vec![0.1, 0.3], vec![Some(now), Some(now)], vec![5, 7]));
    }
}
