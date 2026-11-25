//! [`ArmCore`]: the RT-owned state of one arm, in private aggregates behind narrow
//! operations (armd 1.6 RT core, step 2 foundation).

use std::sync::Arc;
use std::time::Instant;

use super::config_iface::{ConfigDomain, ConfigHold, HeldConfig};
use super::mailbox::TargetAuthority;
use super::ops::TorqueProgress;
use super::ids::{ArmId, BusId};
use super::publish::{state_channel, ObservationSpares, OwnerStateSnapshot, PublishedOwnerMeta, RtInputs, RtMotionMeta, StateHub, StatePublisher};
use super::record_ring::{Fence, Finished, RecordProducer, RecordRing};
use crate::driver::ArmDriver;
use crate::model::ArmObservation;
use crate::rt_core::scratch::{copy_into, FeedbackScratch};

/// The publication slot's observation, out for filling ([`ArmCore::take_observation_buffer`]);
/// only [`ArmCore::publish`] takes it back.
#[derive(Debug)]
pub struct ObservationBuffer(ArmObservation, ObservationSpares);

impl ObservationBuffer {
    /// Borrows the observation and its preallocated string spares together (0 alloc).
    pub fn split_mut(&mut self) -> (&mut ArmObservation, &mut ObservationSpares) {
        (&mut self.0, &mut self.1)
    }
}

impl std::ops::Deref for ObservationBuffer {
    type Target = ArmObservation;
    fn deref(&self) -> &ArmObservation { &self.0 }
}

impl std::ops::DerefMut for ObservationBuffer {
    fn deref_mut(&mut self) -> &mut ArmObservation { &mut self.0 }
}

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
/// One `ArmCore` per arm, owned by exactly one local owner (either `SupervisorInner`
/// under the legacy supervisor write lock, or exclusively by the synchronous RT executor
/// OS thread once ownership transfers at startup); its bus endpoint names one physical bus.
/// Its state lives in private aggregates reached only through the narrow operations below;
/// there is no general mutable accessor.
///
/// Aggregates today: feedback bookkeeping ([`FeedbackState`]), the state publication, the
/// recording producer and configuration reservations. Tracks A-E move the remaining
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
    /// An [`ObservationBuffer`] is out (taken, not yet published): the displacing publish
    /// is refused meanwhile, so the in-place publish never displaces anything.
    buffer_out: bool,
    config: ConfigHold,
    /// The target stream the core consumes from (`mailbox::TargetFrame::check`).
    targets: TargetAuthority,
    /// The driver transition in flight and its latest progress (`ops::CoreOps::commit_transition`).
    transition: Option<Result<TorqueProgress, &'static str>>,
    /// The terminal record of the last macro action that ended.
    last_macro: Option<MacroTerminal>,
}

/// How a macro action ended, kept after its action record is finished or replaced, so the
/// RT can send its terminal (outcome with cause, child count).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MacroTerminal {
    pub action: super::broker::ActionId,
    pub children: u32,
    pub outcome: super::broker::Outcome,
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
            buffer_out: false,
            config: ConfigHold::default(),
            targets: TargetAuthority::default(),
            transition: None,
            last_macro: None,
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

    /// Exact publication sequence number the next [`Self::publish_with_rt_meta`] will assign.
    #[inline]
    pub fn next_publish_seq(&self) -> u64 { self.publisher.next_seq() }

    /// Pre-warms empty triple-buffer observation slots before the RT release loop begins,
    /// without blocking on a contended reader or advancing `published_seq`.
    pub fn try_prewarm_observations(&mut self, make: impl FnMut() -> ArmObservation) {
        self.publisher.try_prewarm_observations(make);
    }

    /// Installs privately staged pre-warmed producer observation slots without locking `hub`.
    pub fn install_prewarmed_producer_slots(
        &mut self,
        back_obs: ArmObservation,
        mid_obs: ArmObservation,
    ) -> (Option<ArmObservation>, Option<ArmObservation>) {
        self.publisher
            .install_prewarmed_producer_slots(back_obs, mid_obs)
    }

    /// The observation of the publication slot the next [`Self::publish`] writes, moved
    /// out to be filled in place (`SupervisorInner::fill_observation`) and handed back by
    /// `publish`: the tick reuses its memory instead of building a new observation (a blank
    /// one, allocated, only before the slot held one: the first ticks).
    pub fn take_observation_buffer(&mut self) -> ObservationBuffer {
        self.buffer_out = true;
        let obs = self.publisher.take_back_observation().unwrap_or_else(ArmObservation::blank_rt);
        let spares = self.publisher.take_back_spares();
        ObservationBuffer(obs, spares)
    }

    /// Publishes one tick's observation, filled in place, back into the slot it came from:
    /// nothing is displaced, so the tick never frees an observation (whatever the readers
    /// and the retire ring do).
    pub fn publish(&mut self, buffer: ObservationBuffer, generation: u64, sampled_at: Instant, valid_until: Instant, positions: &[f32], velocities: &[f32]) {
        self.publish_with_rt_meta(
            buffer,
            generation,
            sampled_at,
            valid_until,
            OwnerStateSnapshot::default(),
            positions,
            velocities,
            positions,
            &[],
            &[],
            PublishedOwnerMeta::default(),
            RtMotionMeta::default(),
        );
    }

    /// In-place zero-alloc publication carrying [`OwnerStateSnapshot`], commanded joints, driver
    /// RX queue bounds, live `Copy` [`PublishedOwnerMeta`], and [`RtMotionMeta`] alongside the observation.
    #[allow(clippy::too_many_arguments)]
    pub fn publish_with_rt_meta(
        &mut self,
        buffer: ObservationBuffer,
        generation: u64,
        sampled_at: Instant,
        valid_until: Instant,
        owner_state: OwnerStateSnapshot,
        positions: &[f32],
        velocities: &[f32],
        commanded_positions: &[f32],
        commanded_velocities: &[f32],
        feedback_queue_bounds: &[Option<Instant>],
        owner: PublishedOwnerMeta,
        meta: RtMotionMeta,
    ) {
        self.buffer_out = false;
        self.publisher.restore_back_spares(buffer.1);
        let rt = RtInputs {
            positions,
            velocities,
            commanded_positions,
            commanded_velocities,
            feedback_instants: &self.feedback.instants,
            feedback_seqs: &self.feedback.seqs,
            feedback_queue_bounds,
            owner,
            meta,
        };
        let displaced = self.publisher.publish_with_owner(generation, sampled_at, valid_until, buffer.0, owner_state, rt);
        // the slot was emptied by take_observation_buffer and nothing wrote it since
        debug_assert!(displaced.is_none(), "an in-place publication displaced an observation");
        if let Some(d) = displaced { self.publisher.restore_back_observation(d); }
    }

    /// Publishes an observation built elsewhere (not the tick: tests, start-up). The one it
    /// displaces goes to the retire ring; when that ring is full it comes back, for the
    /// caller to drop off the control thread. Refused (the observation comes back as Err)
    /// while an [`ObservationBuffer`] is out.
    // the refused observation comes back by value (not on the tick; boxing it would allocate)
    #[allow(clippy::result_large_err)]
    pub fn publish_displacing(&mut self, generation: u64, sampled_at: Instant, valid_until: Instant, obs: ArmObservation, positions: &[f32], velocities: &[f32]) -> Result<Option<ArmObservation>, ArmObservation> {
        if self.buffer_out { return Err(obs); }
        let rt = RtInputs {
            positions,
            velocities,
            commanded_positions: positions,
            commanded_velocities: &[],
            feedback_instants: &self.feedback.instants,
            feedback_seqs: &self.feedback.seqs,
            feedback_queue_bounds: &[],
            owner: PublishedOwnerMeta::default(),
            meta: RtMotionMeta::default(),
        };
        Ok(self.publisher.publish(generation, sampled_at, valid_until, obs, rt))
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

    // -- configuration reservations -------------------------------------------------------

    /// The configuration hold state machine ([`super::config_iface::ConfigHold`]).
    pub fn config(&self) -> &ConfigHold { &self.config }
    /// The same, to grant, persist, activate, release or reconcile (the `ConfigAuthority` adapter).
    pub fn config_mut(&mut self) -> &mut ConfigHold { &mut self.config }

    /// The current hold, as a copyable view.
    pub fn config_hold(&self) -> Option<HeldConfig> { self.config.held() }

    /// The hold on `domain`.
    pub fn config_held(&self, domain: ConfigDomain) -> Option<HeldConfig> { self.config.held().filter(|h| h.domain == domain) }

    /// A hold inhibits motion (a motion domain is reserved or persisted, or any domain is
    /// reconciling): checked at motion admission and at commit.
    pub fn motion_inhibited_by_config(&self) -> bool { self.config.motion_inhibited() }

    // -- target streams -------------------------------------------------------------------

    /// What target frames must match to be consumed.
    pub fn target_authority(&self) -> TargetAuthority { self.targets }

    /// Consume frames of `a.stream` only, under `a`'s policy generation and age bounds
    /// (replaces the previous stream: its buffered frames are refused).
    pub fn authorize_stream(&mut self, a: TargetAuthority) { self.targets = a; }

    /// No stream is consumed any more (stream ended, policy revoked).
    pub fn revoke_stream(&mut self) { self.targets = TargetAuthority::default(); }

    // -- driver transitions ---------------------------------------------------------------

    /// A transition is in flight (started, not yet reported done or failed).
    pub fn transition_pending(&self) -> bool { self.transition.is_some() }

    /// A transition started with `progress` (the delegated path knows the outcome at once).
    pub fn begin_transition(&mut self, progress: Result<TorqueProgress, &'static str>) { self.transition = Some(progress); }

    /// The progress to report in this poll; a final one (done or failed) ends the transition.
    pub fn poll_transition(&mut self) -> Option<Result<TorqueProgress, &'static str>> {
        let p = self.transition?;
        if !matches!(p, Ok(TorqueProgress::Pending { .. })) { self.transition = None; }
        Some(p)
    }

    /// Consumes a failed/aborted transition outcome without touching a pending transition or
    /// synchronous Step-3 `Ok(TorqueProgress::Done)` completion.
    pub fn take_aborted_transition(&mut self) -> Option<&'static str> {
        if let Some(Err(code)) = self.transition {
            self.transition = None;
            Some(code)
        } else {
            None
        }
    }

    // -- macro terminals ------------------------------------------------------------------

    /// Records how a macro action ended (call where the macro finishes, track E).
    pub fn record_macro_terminal(&mut self, t: MacroTerminal) { self.last_macro = Some(t); }

    /// The last macro terminal record.
    pub fn last_macro(&self) -> Option<MacroTerminal> { self.last_macro }
}
