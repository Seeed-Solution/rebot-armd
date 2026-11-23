//! Per-arm real-time core (armd 1.6 RT core).
//!
//! Step 1 of the migration (`docs` of the RT core design, section 6): the ownership
//! boundary of one arm, the state publication channel and the recording ring. Everything
//! still runs inside the control tick under the supervisor write lock; what changes is how
//! state leaves the tick:
//!
//! - [`publish`]: the tick publishes each tick's observation into a preallocated triple
//!   buffer; REST, WS, the black box and measurement polls read it without the supervisor
//!   lock (it replaces the 1.5.0 observation cache).
//! - [`record_ring`]: the tick appends recording samples into a preallocated 256-sample
//!   ring that a normal-priority thread drains.
//! - [`ids`]: the owner identities and the meaning of time values.

pub mod ids;
#[cfg(not(target_arch = "wasm32"))]
pub mod publish;
#[cfg(not(target_arch = "wasm32"))]
pub mod record_ring;
#[cfg(not(target_arch = "wasm32"))]
pub mod triple;

#[cfg(not(target_arch = "wasm32"))]
pub use core::ArmCore;

#[cfg(not(target_arch = "wasm32"))]
mod core {
    use std::sync::Arc;
    use std::time::Instant;

    use super::ids::{ArmId, BusId};
    use super::publish::{state_channel, RtInputs, StateHub, StatePublisher};
    use super::record_ring::RecordProducer;
    use crate::model::ArmObservation;

    /// The RT-owned state of one arm.
    ///
    /// One `ArmCore` per arm, owned by exactly one local owner (today `SupervisorInner`,
    /// under the supervisor write lock); its bus endpoint names one physical bus. Step 1
    /// moves the feedback bookkeeping and the publication endpoints here. The rest of the
    /// RT-owned state the design assigns to the core still lives in `SupervisorInner` and
    /// moves in step 2 with the executor: measured / commanded / target joint state and
    /// velocities, trajectory cursor (`active_trajectory`, `quintic_settle`), servo history
    /// (`servo_target`, `servo_settled_ticks`, `servo_lead_stall`), gripper contact / stall /
    /// force state, torque / disable / rearm state, lease / watchdog / handover / epoch /
    /// fault state, gravity, tracking, collision and MIT runtime, and the limits, kinematics,
    /// joint zeros and keep-outs the next output needs.
    pub struct ArmCore {
        pub id: ArmId,
        pub bus: BusId,
        /// Monotonic hardware receive instant of each joint's latest feedback (profile dof).
        pub joint_feedback_instants: Vec<Option<Instant>>,
        /// Per-joint count of distinct feedback samples (profile dof).
        pub joint_feedback_seqs: Vec<u64>,
        publisher: StatePublisher,
        /// Recording samples of the running teach session.
        pub record: RecordProducer,
    }

    impl ArmCore {
        /// The core of the single arm with `dof` joints.
        pub fn new(dof: usize) -> Self {
            let (publisher, _) = state_channel(dof);
            Self {
                id: ArmId::LOCAL,
                bus: BusId::LOCAL,
                joint_feedback_instants: vec![None; dof],
                joint_feedback_seqs: vec![0; dof],
                publisher,
                record: RecordProducer::new(dof),
            }
        }

        /// The readers' side of the state channel.
        pub fn hub(&self) -> &Arc<StateHub> { self.publisher.hub() }

        /// `seq` of the last publication (0: none yet).
        pub fn published_seq(&self) -> u64 { self.hub().published_seq() }

        /// Publishes one tick's observation (see [`StatePublisher::publish`]); returns the
        /// displaced observation for the caller to drop after releasing the lock.
        #[must_use]
        pub fn publish(&mut self, generation: u64, sampled_at: Instant, valid_until: Instant, obs: ArmObservation, positions: &[f32], velocities: &[f32]) -> Option<ArmObservation> {
            let rt = RtInputs { positions, velocities, feedback_instants: &self.joint_feedback_instants, feedback_seqs: &self.joint_feedback_seqs };
            self.publisher.publish(generation, sampled_at, valid_until, obs, rt)
        }
    }
}
