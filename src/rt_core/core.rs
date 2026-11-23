//! [`ArmCore`]: the RT-owned state of one arm, in private aggregates behind narrow
//! operations (armd 1.6 RT core, step 2 foundation).

use std::sync::Arc;
use std::time::Instant;

use super::ids::{ArmId, BusId};
use super::publish::{state_channel, RtInputs, StateHub, StatePublisher};
use super::record_ring::{Fence, Finished, RecordProducer, RecordRing};
use crate::driver::ArmDriver;
use crate::model::ArmObservation;
use crate::rt_core::scratch::{copy_into, FeedbackScratch};

/// Per-joint feedback bookkeeping (profile dof): receive instants and sample counters.
/// Written only by [`ArmCore::poll_feedback`] / [`ArmCore::simulate_feedback`].
pub struct FeedbackState {
    instants: Vec<Option<Instant>>,
    seqs: Vec<u64>,
}

impl FeedbackState {
    /// Monotonic hardware receive instant of each joint's latest feedback.
    pub fn instants(&self) -> &[Option<Instant>] { &self.instants }
    /// Per-joint count of distinct feedback samples.
    pub fn seqs(&self) -> &[u64] { &self.seqs }
    pub fn instant(&self, joint: usize) -> Option<Instant> { self.instants.get(joint).copied().flatten() }
    pub fn seq(&self, joint: usize) -> u64 { self.seqs.get(joint).copied().unwrap_or(0) }
}

/// The RT-owned state of one arm.
///
/// One `ArmCore` per arm, owned by exactly one local owner (during the migration
/// `SupervisorInner`, under the supervisor write lock; after the cutover the RT executor);
/// its bus endpoint names one physical bus. Its state lives in private aggregates reached
/// only through the narrow operations below; there is no general mutable accessor.
///
/// Aggregates today: feedback bookkeeping ([`FeedbackState`]), the state publication and the
/// recording producer. Tracks A-E move the remaining
/// RT-owned state here as further private aggregates (joint state, trajectories, servo
/// history, gripper state, torque and driver transitions, lease / watchdog / fault state,
/// numeric safety configuration); until a track moves one, it stays in `SupervisorInner`
/// and is not mirrored here.
pub struct ArmCore {
    id: ArmId,
    bus: BusId,
    feedback: FeedbackState,
    publisher: StatePublisher,
    /// Recording samples of the running teach session.
    record: RecordProducer,
}

impl ArmCore {
    /// The core of the single arm with `dof` joints.
    pub fn new(dof: usize) -> Self {
        let (publisher, _) = state_channel(dof);
        Self {
            id: ArmId::LOCAL,
            bus: BusId::LOCAL,
            feedback: FeedbackState { instants: vec![None; dof], seqs: vec![0; dof] },
            publisher,
            record: RecordProducer::new(dof),
        }
    }

    pub fn id(&self) -> ArmId { self.id }
    pub fn bus(&self) -> BusId { self.bus }

    // -- feedback (poll hardware) ---------------------------------------------------------

    pub fn feedback(&self) -> &FeedbackState { &self.feedback }

    /// Updates per-joint feedback instants and monotonic sequence counters from a hardware
    /// driver (read into `sc`, no allocation), and the driver's receive-queue bounds into
    /// `queue_bounds` (cleared when the driver reports none for every joint).
    pub fn poll_feedback(&mut self, queue_bounds: &mut Vec<Option<Instant>>, dof: usize, driver: &dyn ArmDriver, sc: &mut FeedbackScratch) {
        let FeedbackState { instants: joint_feedback_instants, seqs: joint_feedback_seqs } = &mut self.feedback;
        driver.feedback_instants_into(&mut sc.instants);
        driver.feedback_seqs_into(&mut sc.seqs);
        driver.feedback_queue_bounds_into(&mut sc.bounds);
        if sc.bounds.len() == dof {
            copy_into(queue_bounds, &sc.bounds);
        } else {
            queue_bounds.clear();
        }
        if joint_feedback_seqs.len() != dof {
            joint_feedback_seqs.resize(dof, 0);
        }
        if sc.seqs.len() == dof {
            copy_into(joint_feedback_seqs, &sc.seqs);
        } else {
            for i in 0..dof {
                let prev = joint_feedback_instants.get(i).copied().flatten();
                let cur = sc.instants.get(i).copied().flatten();
                if let Some(t) = cur {
                    if prev.is_none_or(|p| t > p) {
                        joint_feedback_seqs[i] = joint_feedback_seqs[i].wrapping_add(1);
                    }
                }
            }
        }
        copy_into(joint_feedback_instants, &sc.instants);
    }

    /// Driverless simulation: every joint got a fresh sample at `now`.
    pub fn simulate_feedback(&mut self, dof: usize, now: Instant) {
        let f = &mut self.feedback;
        f.instants.resize(dof, Some(now));
        f.instants.fill(Some(now));
        f.seqs.resize(dof, 0);
        for seq in &mut f.seqs {
            *seq = seq.wrapping_add(1);
        }
    }

    // -- publication ----------------------------------------------------------------------

    /// The readers' side of the state channel.
    pub fn hub(&self) -> &Arc<StateHub> { self.publisher.hub() }

    /// `seq` of the last publication (0: none yet).
    pub fn published_seq(&self) -> u64 { self.hub().published_seq() }

    /// The observation of the publication slot the next [`Self::publish`] writes, moved
    /// out to be filled in place (`SupervisorInner::fill_observation`) and handed back by
    /// `publish`: the tick reuses its memory instead of building a new observation. None
    /// before the slot held one.
    pub fn take_observation_buffer(&mut self) -> Option<ArmObservation> { self.publisher.take_back_observation() }

    /// Publishes one tick's observation (see [`StatePublisher::publish`]); returns the
    /// displaced observation for the caller to drop after releasing the lock.
    #[must_use]
    pub fn publish(&mut self, generation: u64, sampled_at: Instant, valid_until: Instant, obs: ArmObservation, positions: &[f32], velocities: &[f32]) -> Option<ArmObservation> {
        let rt = RtInputs { positions, velocities, feedback_instants: &self.feedback.instants, feedback_seqs: &self.feedback.seqs };
        self.publisher.publish(generation, sampled_at, valid_until, obs, rt)
    }

    // -- recording ------------------------------------------------------------------------

    /// Start fence of a recording session (`RecordProducer::begin`); the dispatcher drains
    /// the ring from now on.
    pub fn begin_recording(&mut self) -> u64 {
        let session = self.record.begin();
        if let Some(ring) = self.record.ring() { self.publisher.hub().set_record_ring(ring.clone()); }
        session
    }

    /// Stop fence (`RecordProducer::end`): collect the returned fence off the tick.
    pub fn end_recording(&mut self) -> Option<Fence> { self.record.end() }

    /// [`Self::end_recording`] and its collection at once (tests).
    pub fn finish_recording(&mut self) -> Finished { self.record.finish() }

    /// Pushes one tick's sample while a session runs (no allocation); false when the ring is
    /// full (the sample is dropped and counted) or no session runs.
    pub fn record_sample(&mut self, tick_seq: u64, timestamp: f64, state: &[f32], action: &[f32]) -> bool {
        self.record.active() && self.record.push(tick_seq, timestamp, state, action)
    }

    pub fn recording_active(&self) -> bool { self.record.active() }
    /// Samples pushed in the running session.
    pub fn record_pushed(&self) -> usize { self.record.pushed() }
    /// Samples dropped on a full ring in the running session.
    pub fn record_dropped(&self) -> u64 { self.record.dropped() }
    /// The recording ring (after the first session).
    pub fn record_ring(&self) -> Option<&Arc<RecordRing>> { self.record.ring() }
}
