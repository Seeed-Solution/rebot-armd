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
//! - `meta.seq` counts publications of this process (1, 2, ...; no gaps). The tick
//!   publishes once per tick, so it is the RT tick counter: a reader that sees it stop
//!   advancing knows the control tick stopped (`observation_cache.published` exposes it).
//! - `meta.generation` is the supervisor lock generation of the tick's write guard
//!   (`SupervisorLock::generation`): every write guard taken later has a higher one, so a
//!   sample answers for the state while the current generation still equals it, and a
//!   reader that saw generation `g` before a request gets read-your-writes from any sample
//!   with `generation >= g`.
//! - `meta.valid_until` is the local deadline after which time-derived fields would flip
//!   without a write (lease expiry, previous-action window, 250 ms feedback freshness behind
//!   `feedback_valid` / `enable_state`; `SupervisorInner::observation_valid_until`), capped at
//!   two control periods.
//! - `rt` holds the per-joint feedback the tick also used (positions, velocities, receive
//!   instants and per-joint feedback sequence numbers), sized for this arm's profile.
//!
//! The tick never waits for a reader: publication is one atomic swap of a preallocated
//! slot ([`super::triple`]). Readers share [`StateHub`]'s mutex with each other only.

use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::Instant;

use crate::macro_action::{ContactWatch, SubState};
use crate::model::{ArmObservation, HolderLabel, ObservationSchema};

use super::broker::{Authority, BootId};
use super::ids::ArmId;
use super::mailbox::MAX_DOF;
use super::triple::{triple_buffer, Consumer, Producer};

/// Fixed-size, `Copy` owner state published alongside every observation on [`StateHub`].
///
/// Zero heap allocations and zero locks on the RT control thread (`armd-control`); readers
/// (REST schema/lease checks, WS upgrade/ACK/reject decoration, off-thread request preparation)
/// project variable-length fields off-thread from this coherent snapshot.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OwnerStateSnapshot {
    pub arm: ArmId,
    pub boot: BootId,
    pub control_frequency_hz: u32,
    pub lease_enabled: bool,
    /// Active unfenced lease generation at `sampled_at`, or `0` when no valid lease is held.
    pub lease_generation: u64,
    /// Bounded internal connection handle of the active lease holder (`None` when unheld/expired).
    pub lease_owner: Option<HolderLabel>,
    /// Numeric WS connection ID (`>= 1`) of the active lease holder (`0` for non-WS / none).
    pub lease_owner_conn_id: u64,
    /// Display label of the active lease holder (`Some("")` preserved for empty legacy labels).
    pub lease_label: Option<HolderLabel>,
    /// Monotonic expiry instant of the active lease (`acquired_at + lease_timeout`).
    pub lease_expires_at: Option<Instant>,
    /// Internal connection handle of the lease most recently evicted by forced preemption.
    pub last_evicted_owner: Option<HolderLabel>,
    /// Generation of the lease most recently evicted by forced preemption (`0` if none).
    pub last_evicted_generation: u64,
    /// Highest lease generation fenced by release, expiry, preemption, or disconnect.
    pub fenced_lease_generation: u64,
    /// Timestamp of the last forced preemption (for off-thread cooldown visibility).
    pub last_preemption: Option<Instant>,
    pub epoch: u64,
    pub stop_generation: u64,
    pub config_revision: u64,
    pub parent_generation: u64,
    /// Revision pairing this published sample with [`StateHub`]'s off-thread schema cache.
    pub schema_revision: u64,
    pub is_torqued: bool,
    pub disable_pending: bool,
    pub torque_transition_pending: bool,
    pub motion_active: bool,
    pub handover_settling: bool,
    pub calibration_busy: bool,
    pub config_inhibited: bool,
    pub e_stop_latched: bool,
    pub operator_auth_enforced: bool,
    pub gravity_comp_active: bool,
    pub safety_stopped: bool,
    pub safety_recoverable: bool,
    pub dof: u8,
    pub commanded_positions: [f32; MAX_DOF],
    pub residual_torques: Option<[f32; 6]>,
    pub has_zero_reference: bool,
    pub zero_convention: crate::joint_zero::ZeroConvention,
}

impl Default for OwnerStateSnapshot {
    fn default() -> Self {
        Self {
            arm: ArmId::LOCAL,
            boot: BootId(0),
            control_frequency_hz: 50,
            lease_enabled: true,
            lease_generation: 0,
            lease_owner: None,
            lease_owner_conn_id: 0,
            lease_label: None,
            lease_expires_at: None,
            last_evicted_owner: None,
            last_evicted_generation: 0,
            fenced_lease_generation: 0,
            last_preemption: None,
            epoch: 0,
            stop_generation: 0,
            config_revision: 0,
            parent_generation: 0,
            schema_revision: 0,
            is_torqued: false,
            disable_pending: false,
            torque_transition_pending: false,
            motion_active: false,
            handover_settling: false,
            calibration_busy: false,
            config_inhibited: false,
            e_stop_latched: false,
            operator_auth_enforced: false,
            gravity_comp_active: false,
            safety_stopped: false,
            safety_recoverable: true,
            dof: 0,
            commanded_positions: [0.0; MAX_DOF],
            residual_torques: None,
            has_zero_reference: false,
            zero_convention: crate::joint_zero::ZeroConvention::default(),
        }
    }
}

impl OwnerStateSnapshot {
    /// True iff this snapshot reflects an active, unfenced lease not yet expired at `now`.
    #[inline]
    pub fn is_lease_active_at(&self, now: Instant) -> bool {
        self.lease_generation != 0
            && self.lease_generation > self.fenced_lease_generation
            && self.lease_expires_at.is_some_and(|exp| now < exp)
    }

    /// Why `connection` no longer holds `generation` according to this snapshot.
    #[inline]
    pub fn lease_loss_reason(&self, connection: &str, generation: u64) -> &'static str {
        match self.last_evicted_owner {
            Some(owner)
                if owner.as_str() == connection && self.last_evicted_generation == generation =>
            {
                "preempted"
            }
            _ => "expired",
        }
    }

    /// Authority tuple projected from this snapshot at `now`.
    #[inline]
    pub fn authority_at(&self, now: Instant) -> Authority {
        Authority {
            lease_generation: if self.is_lease_active_at(now) {
                self.lease_generation
            } else {
                0
            },
            epoch: self.epoch,
            stop_generation: self.stop_generation,
            config_revision: self.config_revision,
            parent_generation: self.parent_generation,
        }
    }
}

/// Publication stamp of a sample (see the module docs).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SampleMeta {
    pub seq: u64,
    pub generation: u64,
    pub sampled_at: Instant,
    pub valid_until: Instant,
}

/// Scalar motion/authority metadata published alongside [`RtSample`] (Copy, 0 alloc on RT).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RtMotionMeta {
    pub boot_id: super::broker::BootId,
    pub clock_now: Option<Instant>,
    pub workspace_revision: u64,
    pub motion_config_revision: u64,
    pub stop_generation: u64,
    pub lease_generation: u64,
    pub config_inhibited: bool,
    pub calibration_busy: bool,
    pub holding_after_clear: bool,
    pub watchdog_park_pending_gen: u64,
    pub watchdog_park_pending_attempt: u8,
}

impl Default for RtMotionMeta {
    fn default() -> Self {
        Self {
            boot_id: super::broker::BootId(0),
            clock_now: None,
            workspace_revision: 0,
            motion_config_revision: 0,
            stop_generation: 0,
            lease_generation: 0,
            config_inhibited: false,
            calibration_busy: false,
            holding_after_clear: false,
            watchdog_park_pending_gen: 0,
            watchdog_park_pending_attempt: 0,
        }
    }
}

/// Zero-alloc `Copy` snapshot of live held-object state published per tick.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PublishedHeld {
    pub label: HolderLabel,
    pub spill_safe: bool,
}

/// Zero-alloc `Copy` snapshot of the running macro parent and current child status.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PublishedMacroState {
    pub active_id: u64,
    pub alive: bool,
    pub children: u32,
    pub child_seq: u32,
    pub child_running: bool,
    pub child_sub: SubState,
    pub child_watch: ContactWatch,
}

impl Default for PublishedMacroState {
    fn default() -> Self {
        Self {
            active_id: 0,
            alive: false,
            children: 0,
            child_seq: 0,
            child_running: false,
            child_sub: SubState::Idle,
            child_watch: ContactWatch::default(),
        }
    }
}

/// Zero-alloc `Copy` owner metadata published every tick alongside `RtSample`, enabling
/// off-thread prepared-action and macro-orchestrator reads without acquiring `SupervisorLock`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PublishedOwnerMeta {
    pub arm: ArmId,
    pub boot: BootId,
    pub authority: Authority,
    pub calibration_revision: u64,
    pub poses_revision: u64,
    pub sequences_revision: u64,
    pub grasp_setup_revision: u64,
    pub gripper_calib_revision: u64,
    pub power_budget_revision: u64,
    pub tool_digest_hash: u64,
    pub motion_active: bool,
    pub config_inhibited: bool,
    pub calibration_busy: bool,
    pub feedback_valid: bool,
    pub is_torqued: bool,
    pub pending_rearm: bool,
    pub pending_normal_unload: bool,
    pub collision_hold: bool,
    pub handover_settling: bool,
    pub gravity_comp_active: bool,
    pub policy_busy: bool,
    pub zero_check_persisting: bool,
    pub torque_or_power_hold: bool,
    pub held: Option<PublishedHeld>,
    pub macro_state: PublishedMacroState,
}

impl Default for PublishedOwnerMeta {
    fn default() -> Self {
        Self {
            arm: ArmId::LOCAL,
            boot: BootId(0),
            authority: Authority::default(),
            calibration_revision: 0,
            poses_revision: 0,
            sequences_revision: 0,
            grasp_setup_revision: 0,
            gripper_calib_revision: 0,
            power_budget_revision: 0,
            tool_digest_hash: 0,
            motion_active: false,
            config_inhibited: false,
            calibration_busy: false,
            feedback_valid: false,
            is_torqued: false,
            pending_rearm: false,
            pending_normal_unload: false,
            collision_hold: false,
            handover_settling: false,
            gravity_comp_active: false,
            policy_busy: false,
            zero_check_persisting: false,
            torque_or_power_hold: false,
            held: None,
            macro_state: PublishedMacroState::default(),
        }
    }
}

/// Per-joint feedback and owner metadata of a sample; every vector has the profile's dof.
#[derive(Debug, Clone, PartialEq)]
pub struct RtSample {
    pub positions: Vec<f32>,
    pub velocities: Vec<f32>,
    /// Commanded joint positions (profile dof) at the end of this tick.
    pub commanded_positions: Vec<f32>,
    /// Commanded joint velocities (profile dof) at the end of this tick.
    pub commanded_velocities: Vec<f32>,
    /// Hardware receive instant of each joint's latest feedback.
    pub feedback_instants: Vec<Option<Instant>>,
    /// Per-joint count of distinct feedback samples.
    pub feedback_seqs: Vec<u64>,
    /// Per-joint driver RX-queue upper bound timestamps (for multiview receive-progress fence).
    pub feedback_queue_bounds: Vec<Option<Instant>>,
    /// Live owner authority, domain revisions, flags, held state, and macro child state.
    pub owner: PublishedOwnerMeta,
    pub boot_id: super::broker::BootId,
    pub clock_now: Option<Instant>,
    pub workspace_revision: u64,
    pub motion_config_revision: u64,
    pub stop_generation: u64,
    pub lease_generation: u64,
    pub config_inhibited: bool,
    pub calibration_busy: bool,
    pub holding_after_clear: bool,
    pub watchdog_park_pending_gen: u64,
    pub watchdog_park_pending_attempt: u8,
}

impl Default for RtSample {
    fn default() -> Self { Self::with_dof(0) }
}

impl RtSample {
    fn with_dof(dof: usize) -> Self {
        Self {
            positions: Vec::with_capacity(dof),
            velocities: Vec::with_capacity(dof),
            commanded_positions: Vec::with_capacity(dof),
            commanded_velocities: Vec::with_capacity(dof),
            feedback_instants: Vec::with_capacity(dof),
            feedback_seqs: Vec::with_capacity(dof),
            feedback_queue_bounds: Vec::with_capacity(dof),
            owner: PublishedOwnerMeta::default(),
            boot_id: super::broker::BootId(0),
            clock_now: None,
            workspace_revision: 0,
            motion_config_revision: 0,
            stop_generation: 0,
            lease_generation: 0,
            config_inhibited: false,
            calibration_busy: false,
            holding_after_clear: false,
            watchdog_park_pending_gen: 0,
            watchdog_park_pending_attempt: 0,
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

/// Preallocated per-slot spares reused across `None <-> Some(...)` action message and policy
/// observation transitions so RT publication never allocates or drops heap storage.
#[derive(Debug, Default)]
pub struct ObservationSpares {
    pub action_msg: String,
    pub prev_action_msg: String,
    #[cfg(feature = "policy-runtime")]
    pub policy_obs: Option<crate::model::PolicyObservation>,
    #[cfg(feature = "policy-runtime")]
    pub policy_shadow_vec: Vec<f32>,
}

impl ObservationSpares {
    pub fn preallocated() -> Self {
        Self {
            action_msg: String::with_capacity(256),
            prev_action_msg: String::with_capacity(256),
            #[cfg(feature = "policy-runtime")]
            policy_obs: Some(crate::model::PolicyObservation::preallocated_rt()),
            #[cfg(feature = "policy-runtime")]
            policy_shadow_vec: Vec::with_capacity(super::mailbox::MAX_DOF),
        }
    }
}

/// One preallocated slot of the triple buffer.
struct Slot {
    meta: Option<SampleMeta>,
    obs: Option<ArmObservation>,
    owner: OwnerStateSnapshot,
    /// Preallocated string spares reused across state/fault/action transitions (0 alloc on RT).
    spares: ObservationSpares,
    rt: RtSample,
}

/// A sample as readers get it.
#[derive(Debug, Clone)]
pub struct Snapshot {
    pub meta: SampleMeta,
    pub obs: ArmObservation,
    pub owner: OwnerStateSnapshot,
    pub rt: RtSample,
    /// Off-RT policy diagnostic overlay revision reflected in `obs` (`0` before first update).
    pub policy_diag_seq: u64,
}

/// Off-RT policy diagnostic overlay update emitted by the executor policy runner (never touched
/// on `armd-control`). Carries Shadow step/target telemetry and Live/Shadow timing `PolicyStats`.
#[cfg(feature = "policy-runtime")]
#[derive(Debug, Clone)]
pub struct PolicyDiagUpdate {
    pub action_id: u64,
    pub is_shadow: bool,
    pub steps: u64,
    pub stats: crate::motion::PolicyStats,
    pub last_input_age_ms: Option<u64>,
    pub last_vision_age_ms: Option<u64>,
    pub vision_frame_id: Option<u64>,
    pub last_shadow_target: Option<Vec<f32>>,
}

#[cfg(feature = "policy-runtime")]
#[derive(Debug, Clone)]
struct PolicyDiagEntry {
    action_id: u64,
    is_shadow: bool,
    steps: u64,
    stats: crate::motion::PolicyStats,
    last_input_age_ms: Option<u64>,
    last_vision_age_ms: Option<u64>,
    vision_frame_id: Option<u64>,
    last_shadow_target: Option<Vec<f32>>,
    sealed: bool,
}

#[cfg(feature = "policy-runtime")]
const POLICY_DIAG_CAPACITY: usize = 64;

/// Capacity of the retire ring.
pub const RETIRE_CAPACITY: usize = 8;

/// Observations the tick displaced from unconsumed slots, handed to the readers' side to
/// be freed there: a single-producer (the tick) single-consumer (the holder of the hub
/// mutex) ring of preallocated `Option` slots, so neither side allocates or waits.
struct RetireRing {
    /// Heap-allocated once at channel creation so constructing `StateHub` does not place
    /// `RETIRE_CAPACITY` full `ArmObservation` values on the caller's stack.
    slots: Box<[UnsafeCell<Option<ArmObservation>>]>,
    head: AtomicU64,
    tail: AtomicU64,
}

// SAFETY: `slots` has fixed length `RETIRE_CAPACITY`; slot `i % CAP` is written by the producer
// only while `tail <= i < tail + CAP` (Acquire load of `tail`) and taken by the consumer only
// while `i < head` (Acquire load of `head`); the consumer side runs under the hub mutex, so it is single.
unsafe impl Sync for RetireRing {}

impl RetireRing {
    fn new() -> Self {
        let mut slots = Vec::with_capacity(RETIRE_CAPACITY);
        for _ in 0..RETIRE_CAPACITY {
            slots.push(UnsafeCell::new(None));
        }
        Self { slots: slots.into_boxed_slice(), head: AtomicU64::new(0), tail: AtomicU64::new(0) }
    }

    /// Producer: hands `obs` over (None), or gives it back when the ring is full.
    fn push(&self, head: &mut u64, obs: ArmObservation) -> Option<ArmObservation> {
        if *head - self.tail.load(Ordering::Acquire) >= RETIRE_CAPACITY as u64 { return Some(obs); }
        // SAFETY: see the Sync impl; the replaced value is None (the consumer took it).
        let old = unsafe { (*self.slots[(*head % RETIRE_CAPACITY as u64) as usize].get()).replace(obs) };
        debug_assert!(old.is_none());
        *head += 1;
        self.head.store(*head, Ordering::Release);
        None
    }

    /// Consumer (hub mutex held): frees what the producer handed over.
    fn free_all(&self) {
        let head = self.head.load(Ordering::Acquire);
        let mut tail = self.tail.load(Ordering::Relaxed);
        while tail < head {
            // SAFETY: tail < head: the producer finished this slot.
            drop(unsafe { (*self.slots[(tail % RETIRE_CAPACITY as u64) as usize].get()).take() });
            tail += 1;
        }
        self.tail.store(tail, Ordering::Release);
    }
}

/// The tick's side: owned by `ArmCore`, used under the supervisor write lock.
pub struct StatePublisher {
    producer: Producer<Slot>,
    seq: u64,
    retire_head: u64,
    hub: Arc<StateHub>,
}

/// The per-joint inputs of [`StatePublisher::publish`].
pub struct RtInputs<'a> {
    pub positions: &'a [f32],
    pub velocities: &'a [f32],
    pub commanded_positions: &'a [f32],
    pub commanded_velocities: &'a [f32],
    pub feedback_instants: &'a [Option<Instant>],
    pub feedback_seqs: &'a [u64],
    pub feedback_queue_bounds: &'a [Option<Instant>],
    pub owner: PublishedOwnerMeta,
    pub meta: RtMotionMeta,
}

impl StatePublisher {
    /// Publishes `obs` with its stamp. Copies the per-joint inputs into the slot's
    /// preallocated vectors and swaps one atomic: no allocation, no deallocation, no wait
    /// (while the dof does not grow beyond the profile's). Steady state: the tick moved the
    /// slot's observation out ([`Self::take_back_observation`]), filled it in place and hands
    /// it back here, so nothing is displaced. An observation still in the slot (a caller that
    /// built a new one) goes to the retire ring and is freed by the readers' side; only
    /// when that ring is full (no reader for [`RETIRE_CAPACITY`] such publications) it is
    /// returned, the caller drops it after releasing the lock, and `StateHub::tick_frees`
    /// counts it. Readers copy the slot's observation and never take it, so a reader never
    /// empties the tick's buffer and the tick never waits for one.
    pub fn publish(&mut self, generation: u64, sampled_at: Instant, valid_until: Instant, obs: ArmObservation, rt: RtInputs<'_>) -> Option<ArmObservation> {
        self.publish_with_owner(generation, sampled_at, valid_until, obs, OwnerStateSnapshot::default(), rt)
    }

    pub fn publish_with_owner(
        &mut self,
        generation: u64,
        sampled_at: Instant,
        valid_until: Instant,
        obs: ArmObservation,
        owner: OwnerStateSnapshot,
        rt: RtInputs<'_>,
    ) -> Option<ArmObservation> {
        self.seq += 1;
        let seq = self.seq;
        let slot = self.producer.back_mut();
        slot.meta = Some(SampleMeta { seq, generation, sampled_at, valid_until });
        slot.owner = owner;
        let displaced = slot.obs.replace(obs).and_then(|old| self.hub.retire.push(&mut self.retire_head, old));
        if displaced.is_some() { self.hub.tick_frees.fetch_add(1, Ordering::Relaxed); }
        copy_into(&mut slot.rt.positions, rt.positions);
        copy_into(&mut slot.rt.velocities, rt.velocities);
        copy_into(&mut slot.rt.commanded_positions, rt.commanded_positions);
        copy_into(&mut slot.rt.commanded_velocities, rt.commanded_velocities);
        copy_into(&mut slot.rt.feedback_instants, rt.feedback_instants);
        copy_into(&mut slot.rt.feedback_seqs, rt.feedback_seqs);
        copy_into(&mut slot.rt.feedback_queue_bounds, rt.feedback_queue_bounds);
        slot.rt.owner = rt.owner;
        slot.rt.boot_id = rt.meta.boot_id;
        slot.rt.clock_now = rt.meta.clock_now;
        slot.rt.workspace_revision = rt.meta.workspace_revision;
        slot.rt.motion_config_revision = rt.meta.motion_config_revision;
        slot.rt.stop_generation = rt.meta.stop_generation;
        slot.rt.lease_generation = rt.meta.lease_generation;
        slot.rt.config_inhibited = rt.meta.config_inhibited;
        slot.rt.calibration_busy = rt.meta.calibration_busy;
        slot.rt.holding_after_clear = rt.meta.holding_after_clear;
        slot.rt.watchdog_park_pending_gen = rt.meta.watchdog_park_pending_gen;
        slot.rt.watchdog_park_pending_attempt = rt.meta.watchdog_park_pending_attempt;
        let _ = self.producer.publish();
        self.hub.published.store(seq, Ordering::Release);
        self.hub.wake();
        displaced
    }

    pub fn hub(&self) -> &Arc<StateHub> { &self.hub }

    /// Moves the observation out of the slot the next [`Self::publish`] writes (see
    /// `ArmCore::take_observation_buffer`).
    pub fn take_back_observation(&mut self) -> Option<ArmObservation> { self.producer.back_mut().obs.take() }

    /// Pre-populates empty triple-buffer observation slots off the release loop without blocking
    /// on a contended reader. Leaves `meta` and `published_seq` untouched.
    ///
    /// - If `hub.state` is uncontended (`try_lock`), warms all 3 slots directly.
    /// - If a reader holds `hub.state`, warms the producer's writable slot(s) without waiting
    ///   (both `back` and `middle` when nothing has been published yet via `swap_unfresh`) and
    ///   stores a template in `hub.prewarm_template` so `StateHub::refresh` warms `front`
    ///   off-thread before rotating it back to the producer.
    pub fn try_prewarm_observations(&mut self, mut make: impl FnMut() -> ArmObservation) {
        let _ = self.hub.prewarm_template.set(Box::new(make()));
        match self.hub.state.try_lock() {
            Ok(mut st) => {
                self.producer.for_each_slot_mut(&mut st.consumer, |slot| {
                    if slot.obs.is_none() {
                        slot.obs = Some(make());
                    }
                });
            }
            Err(std::sync::TryLockError::Poisoned(p)) => {
                let mut st = p.into_inner();
                self.producer.for_each_slot_mut(&mut st.consumer, |slot| {
                    if slot.obs.is_none() {
                        slot.obs = Some(make());
                    }
                });
            }
            Err(std::sync::TryLockError::WouldBlock) => {
                let back = self.producer.back_mut();
                if back.obs.is_none() {
                    back.obs = Some(make());
                }
                if self.producer.swap_unfresh() {
                    let other = self.producer.back_mut();
                    if other.obs.is_none() {
                        other.obs = Some(make());
                    }
                }
            }
        }
    }

    /// Moves the preallocated string spares out of the back slot (0 alloc).
    pub fn take_back_spares(&mut self) -> ObservationSpares {
        std::mem::take(&mut self.producer.back_mut().spares)
    }

    /// Returns the string spares to the back slot before publishing (0 alloc, 0 drop).
    pub fn restore_back_spares(&mut self, mut spares: ObservationSpares) {
        let slot = self.producer.back_mut();
        if spares.action_msg.capacity() > slot.spares.action_msg.capacity() {
            slot.spares.action_msg = std::mem::take(&mut spares.action_msg);
        }
        if spares.prev_action_msg.capacity() > slot.spares.prev_action_msg.capacity() {
            slot.spares.prev_action_msg = std::mem::take(&mut spares.prev_action_msg);
        }
        #[cfg(feature = "policy-runtime")]
        {
            if spares.policy_obs.is_some() && slot.spares.policy_obs.is_none() {
                slot.spares.policy_obs = spares.policy_obs.take();
            }
            if spares.policy_shadow_vec.capacity() > slot.spares.policy_shadow_vec.capacity() {
                slot.spares.policy_shadow_vec = std::mem::take(&mut spares.policy_shadow_vec);
            }
        }
    }

    /// Puts `obs` into the back slot when it is empty (else back to the retire ring; never
    /// freed here when that ring has room). `ArmCore::publish`'s fallback for a displaced
    /// observation, which its encapsulation makes unreachable.
    pub(super) fn restore_back_observation(&mut self, obs: ArmObservation) {
        let slot = self.producer.back_mut();
        if slot.obs.is_none() { slot.obs = Some(obs); return; }
        if let Some(back) = self.hub.retire.push(&mut self.retire_head, obs) {
            self.hub.tick_frees.fetch_add(1, Ordering::Relaxed);
            drop(back);
        }
    }
}

struct HubState {
    consumer: Consumer<Slot>,
    latest: Option<Arc<Snapshot>>,
    last_policy_diag_seq: u64,
}

/// Result of [`StateHub::try_latest`].
pub enum Poll {
    Ready(Option<Arc<Snapshot>>),
    Busy,
}

/// The readers' side (the dispatcher): the newest sample, and the wake-up of the dispatcher
/// thread that forwards samples to subscribers.
pub struct StateHub {
    state: Mutex<HubState>,
    /// `seq` of the newest publication (0: none yet).
    published: AtomicU64,
    dispatcher: OnceLock<std::thread::Thread>,
    closed: AtomicBool,
    /// The recording ring, drained by the dispatcher (set by the first recording).
    record: OnceLock<Arc<super::record_ring::RecordRing>>,
    retire: RetireRing,
    /// Displaced observations the tick had to free itself (retire ring full).
    tick_frees: AtomicU64,
    /// Pre-sized observation template recorded at executor startup so a contended reader warms
    /// its own `front` slot off-thread before handing it back to the RT producer.
    prewarm_template: OnceLock<Box<ArmObservation>>,
    /// Immutable motion planning config snapshot updated off-tick for zero-lock workers.
    motion_config: std::sync::RwLock<Option<Arc<super::requests::motion::MotionConfigSnapshot>>>,
    /// Off-thread schema cache paired with [`OwnerStateSnapshot::schema_revision`]. Never locked
    /// on the RT control thread.
    schema_cache: Mutex<Option<(u64, Arc<ObservationSchema>)>>,
    /// Revision-tagged slow-changing planning environment published off-thread (never touched
    /// on `armd-control`; read by off-thread action preparation and macro orchestrators).
    prep_env: Mutex<Option<Arc<super::requests::actions::ActionPrepEnv>>>,
    /// Bounded off-RT sidecar store of `measure_multiview` measurement payloads keyed by action
    /// ID (never touched on `armd-control`; bounded to 16 entries). Attached in [`Self::refresh`]
    /// to `obs.action` and `obs.previous_action` before any snapshot or terminal is delivered.
    action_measurements: Mutex<std::collections::HashMap<u64, serde_json::Value>>,
    /// Pre-publication policy diagnostic seal watermark shared with `SupervisorInner` (never
    /// locked on `armd-control`).
    #[cfg(feature = "policy-runtime")]
    policy_diag_sealed: OnceLock<Arc<AtomicU64>>,
    /// Bounded off-RT sidecar store of runner-emitted Shadow diagnostics and `PolicyStats` keyed
    /// by `action_id` (cap 64; never touched on `armd-control`). Lock order: `state -> policy_diag`.
    #[cfg(feature = "policy-runtime")]
    policy_diag: Mutex<std::collections::VecDeque<PolicyDiagEntry>>,
    /// Monotonic revision bumped on every accepted [`Self::update_policy_diag`] call.
    #[cfg(feature = "policy-runtime")]
    policy_diag_seq: AtomicU64,
    /// Number of off-RT readers currently waiting to acquire `self.state` inside `latest()`.
    latest_waiters: AtomicUsize,
}

/// A publisher for an arm with `dof` joints and its hub.
pub fn state_channel(dof: usize) -> (StatePublisher, Arc<StateHub>) {
    let slot = || Slot {
        meta: None,
        obs: None,
        owner: OwnerStateSnapshot::default(),
        spares: ObservationSpares::preallocated(),
        rt: RtSample::with_dof(dof),
    };
    let (producer, consumer) = triple_buffer(slot(), slot(), slot());
    let hub = Arc::new(StateHub {
        state: Mutex::new(HubState { consumer, latest: None, last_policy_diag_seq: 0 }),
        published: AtomicU64::new(0),
        dispatcher: OnceLock::new(),
        closed: AtomicBool::new(false),
        record: OnceLock::new(),
        retire: RetireRing::new(),
        tick_frees: AtomicU64::new(0),
        prewarm_template: OnceLock::new(),
        motion_config: std::sync::RwLock::new(None),
        schema_cache: Mutex::new(None),
        prep_env: Mutex::new(None),
        action_measurements: Mutex::new(std::collections::HashMap::new()),
        #[cfg(feature = "policy-runtime")]
        policy_diag_sealed: OnceLock::new(),
        #[cfg(feature = "policy-runtime")]
        policy_diag: Mutex::new(std::collections::VecDeque::with_capacity(POLICY_DIAG_CAPACITY)),
        #[cfg(feature = "policy-runtime")]
        policy_diag_seq: AtomicU64::new(0),
        latest_waiters: AtomicUsize::new(0),
    });
    (StatePublisher { producer, seq: 0, retire_head: 0, hub: hub.clone() }, hub)
}

impl StateHub {
    /// The newest published sample (None before the first tick). Takes the hub mutex,
    /// which only readers and the dispatcher share; the first reader after a publication
    /// copies the observation out of the slot (the copy is allocated and freed on the reader
    /// side; the slot keeps its memory, which the tick fills again).
    pub fn latest(&self) -> Option<Arc<Snapshot>> {
        struct WaiterGuard<'a>(&'a AtomicUsize);
        impl Drop for WaiterGuard<'_> {
            fn drop(&mut self) {
                self.0.fetch_sub(1, Ordering::Relaxed);
            }
        }
        self.latest_waiters.fetch_add(1, Ordering::Release);
        let _waiter = WaiterGuard(&self.latest_waiters);
        let mut st = self.state.lock().unwrap_or_else(|e| e.into_inner());
        drop(_waiter);
        self.refresh(&mut st)
    }

    /// Number of threads currently waiting on the hub mutex inside [`Self::latest`] (tests).
    #[doc(hidden)]
    pub fn latest_waiters(&self) -> usize {
        self.latest_waiters.load(Ordering::Acquire)
    }

    /// Displaced observations the tick freed itself because no reader drained the retire ring.
    pub fn tick_frees(&self) -> u64 { self.tick_frees.load(Ordering::Relaxed) }

    /// Frees the observations the tick retired and drains any deferred RT stop diagnostics
    /// (hub mutex held on the normal-priority dispatcher / reader path, never the RT tick).
    fn refresh(&self, st: &mut HubState) -> Option<Arc<Snapshot>> {
        self.retire.free_all();
        super::ops::stop_diag().drain_and_log_once();
        super::ops::gripper_force_diag().drain_and_log_once();
        // Acquire-check `has_fresh()` BEFORE inspecting `prewarm_template`: in executor mode
        // `try_prewarm_observations` sets `prewarm_template` (Release) before any release
        // calls `Producer::publish` (AcqRel), so observing `FRESH == 1` guarantees
        // `prewarm_template` is visible and prevents rotating an empty `front` slot into
        // `middle` for the RT producer.
        let has_fresh = st.consumer.has_fresh();
        let diag_seq = self.policy_diag_seq();
        let diag_changed = diag_seq != st.last_policy_diag_seq;
        let mut refreshed = false;
        if has_fresh {
            if st.consumer.front().obs.is_none() {
                if let Some(tmpl) = self.prewarm_template.get() {
                    let mut obs = (**tmpl).clone();
                    obs.ensure_rt_safety_text_spare();
                    let dof = obs.positions.len();
                    obs.gravity_torques.reserve(dof);
                    obs.motor_errors.reserve(dof);
                    st.consumer.front_mut().obs = Some(obs);
                }
            }
            refreshed = st.consumer.refresh();
        }
        if refreshed || (diag_changed && st.consumer.front().obs.is_some()) {
            let slot = st.consumer.front_mut();
            // copied, not moved: the slot keeps its memory for the tick to fill again (the
            // tick neither allocates nor frees observations)
            if let (Some(meta), Some(obs)) = (slot.meta, slot.obs.as_ref()) {
                let mut obs = obs.clone();
                self.attach_action_measurement(&mut obs);
                self.attach_policy_diag(&mut obs);
                st.last_policy_diag_seq = diag_seq;
                let snap = Snapshot {
                    meta,
                    obs,
                    owner: slot.owner,
                    rt: slot.rt.clone(),
                    policy_diag_seq: diag_seq,
                };
                st.latest = Some(Arc::new(snap));
            }
        }
        st.latest.clone()
    }

    /// Holds the readers' mutex (tests: a preempted reader).
    #[doc(hidden)]
    pub fn reader_guard(&self) -> impl Drop + '_ { self.state.lock().unwrap_or_else(|e| e.into_inner()) }

    /// [`latest`](Self::latest) without waiting: `Busy` while another reader holds the hub.
    pub fn try_latest(&self) -> Poll {
        match self.state.try_lock() {
            Ok(mut st) => Poll::Ready(self.refresh(&mut st)),
            Err(std::sync::TryLockError::Poisoned(p)) => Poll::Ready(self.refresh(&mut p.into_inner())),
            Err(std::sync::TryLockError::WouldBlock) => Poll::Busy,
        }
    }

    /// `seq` of the newest publication, without the hub mutex (0: none yet).
    pub fn published_seq(&self) -> u64 { self.published.load(Ordering::Acquire) }

    /// Monotonic sequence of off-RT policy diagnostic overlay updates (`0` before first update).
    pub fn policy_diag_seq(&self) -> u64 {
        #[cfg(feature = "policy-runtime")]
        {
            self.policy_diag_seq.load(Ordering::Acquire)
        }
        #[cfg(not(feature = "policy-runtime"))]
        {
            0
        }
    }

    /// Binds the RT pre-publication policy diagnostic seal watermark (`AtomicU64`).
    #[cfg(feature = "policy-runtime")]
    pub fn attach_policy_diag_seal(&self, seal: Arc<AtomicU64>) {
        let _ = self.policy_diag_sealed.set(seal);
    }

    /// Updates the off-RT policy diagnostic overlay for `update.action_id` and wakes `armd-dispatch`.
    /// Returns `false` (no-op) once RT seals `action_id` or a terminal snapshot freezes the entry.
    #[cfg(feature = "policy-runtime")]
    pub fn update_policy_diag(&self, update: PolicyDiagUpdate) -> bool {
        if update.action_id == 0 {
            return false;
        }
        {
            let mut entries = self.policy_diag.lock().unwrap_or_else(|e| e.into_inner());
            let sealed = self
                .policy_diag_sealed
                .get()
                .map_or(0, |a| a.load(Ordering::Acquire));
            if update.action_id <= sealed {
                return false;
            }
            if let Some(existing) = entries.iter_mut().find(|e| e.action_id == update.action_id) {
                if existing.sealed {
                    return false;
                }
                existing.is_shadow = update.is_shadow;
                existing.steps = existing.steps.max(update.steps);
                existing.stats = update.stats;
                existing.last_input_age_ms = update.last_input_age_ms;
                existing.last_vision_age_ms = update.last_vision_age_ms;
                existing.vision_frame_id = update.vision_frame_id;
                existing.last_shadow_target = update.last_shadow_target;
            } else {
                if entries.len() >= POLICY_DIAG_CAPACITY {
                    entries.pop_front();
                }
                entries.push_back(PolicyDiagEntry {
                    action_id: update.action_id,
                    is_shadow: update.is_shadow,
                    steps: update.steps,
                    stats: update.stats,
                    last_input_age_ms: update.last_input_age_ms,
                    last_vision_age_ms: update.last_vision_age_ms,
                    vision_frame_id: update.vision_frame_id,
                    last_shadow_target: update.last_shadow_target,
                    sealed: false,
                });
            }
            self.policy_diag_seq.fetch_add(1, Ordering::Release);
        }
        self.wake();
        true
    }

    /// Freezes sealed/inactive policy diagnostic entries and enriches `obs` in place (called in
    /// [`Self::refresh`] and in `MaintClientInner::pop_all_replies` before delivering `Observation`).
    #[cfg(feature = "policy-runtime")]
    pub fn attach_policy_diag(&self, obs: &mut ArmObservation) {
        let mut entries = self.policy_diag.lock().unwrap_or_else(|e| e.into_inner());
        let sealed = self
            .policy_diag_sealed
            .get()
            .map_or(0, |a| a.load(Ordering::Acquire));
        if sealed > 0 {
            for e in entries.iter_mut() {
                if e.action_id <= sealed {
                    e.sealed = true;
                }
            }
        }
        if let Some(ref p) = obs.policy {
            if !p.active {
                if let Some(e) = entries.iter_mut().find(|e| e.action_id == p.action_id) {
                    e.sealed = true;
                }
            }
        }
        if entries.is_empty() {
            return;
        }
        if let Some(a) = obs
            .action
            .as_mut()
            .filter(|a| a.kind.as_str() == "policy" && a.policy_stats.is_none())
        {
            if let Some(e) = entries.iter().find(|e| e.action_id == a.id) {
                a.policy_stats = Some(e.stats);
            }
        }
        if let Some(a) = obs
            .previous_action
            .as_mut()
            .filter(|a| a.kind.as_str() == "policy" && a.policy_stats.is_none())
        {
            if let Some(e) = entries.iter().find(|e| e.action_id == a.id) {
                a.policy_stats = Some(e.stats);
            }
        }
        if let Some(p) = obs.policy.as_mut() {
            if let Some(e) = entries.iter().find(|e| e.action_id == p.action_id) {
                if e.is_shadow || p.mode == "shadow" {
                    p.steps = p.steps.max(e.steps);
                    if e.last_input_age_ms.is_some() {
                        p.last_input_age_ms = e.last_input_age_ms;
                    }
                    if e.last_vision_age_ms.is_some() {
                        p.last_vision_age_ms = e.last_vision_age_ms;
                    }
                    if e.vision_frame_id.is_some() {
                        p.vision_frame_id = e.vision_frame_id;
                    }
                    if e.last_shadow_target.is_some() {
                        p.last_shadow_target = e.last_shadow_target.clone();
                    }
                }
            }
        }
    }

    #[cfg(not(feature = "policy-runtime"))]
    pub fn attach_policy_diag(&self, _obs: &mut ArmObservation) {}

    /// Registers the dispatcher thread woken by every publication. Only the first call counts.
    pub fn set_dispatcher(&self, thread: std::thread::Thread) -> bool { self.dispatcher.set(thread).is_ok() }

    pub fn has_dispatcher(&self) -> bool { self.dispatcher.get().is_some() }

    /// Wakes the dispatcher thread: an atomic swap and, when it sleeps, one futex wake.
    pub fn wake(&self) {
        if let Some(t) = self.dispatcher.get() { t.unpark(); }
    }

    /// Registers the recording ring the dispatcher drains (the first call counts).
    pub fn set_record_ring(&self, ring: Arc<super::record_ring::RecordRing>) { let _ = self.record.set(ring); }

    pub fn record_ring(&self) -> Option<&Arc<super::record_ring::RecordRing>> { self.record.get() }

    /// Ends the dispatcher thread (the supervisor is dropped).
    pub fn close(&self) {
        self.closed.store(true, Ordering::Release);
        self.wake();
    }

    pub fn is_closed(&self) -> bool { self.closed.load(Ordering::Acquire) }

    /// Publishes an immutable motion planning config snapshot off the RT tick.
    pub fn set_motion_config(&self, cfg: Arc<super::requests::motion::MotionConfigSnapshot>) {
        *self.motion_config.write().unwrap_or_else(|e| e.into_inner()) = Some(cfg);
    }

    /// Reads the latest immutable motion planning config snapshot without touching the owner lock.
    pub fn motion_config(&self) -> Option<Arc<super::requests::motion::MotionConfigSnapshot>> {
        self.motion_config.read().unwrap_or_else(|e| e.into_inner()).clone()
    }

    /// Records the off-thread [`ObservationSchema`] corresponding to `revision`.
    pub fn set_schema_cache(&self, revision: u64, schema: Arc<ObservationSchema>) {
        let mut g = self.schema_cache.lock().unwrap_or_else(|e| e.into_inner());
        if g.as_ref().is_none_or(|(rev, _)| revision >= *rev) {
            *g = Some((revision, schema));
        }
    }

    /// Returns the cached [`ObservationSchema`] only when its revision matches `revision`.
    pub fn cached_schema(&self, revision: u64) -> Option<Arc<ObservationSchema>> {
        if revision == 0 {
            return None;
        }
        let g = self.schema_cache.lock().unwrap_or_else(|e| e.into_inner());
        g.as_ref()
            .filter(|(rev, _)| *rev == revision)
            .map(|(_, schema)| Arc::clone(schema))
    }

    /// Returns the newest cached [`ObservationSchema`] regardless of revision (in-process fallback).
    pub fn latest_cached_schema(&self) -> Option<Arc<ObservationSchema>> {
        let g = self.schema_cache.lock().unwrap_or_else(|e| e.into_inner());
        g.as_ref().map(|(_, schema)| Arc::clone(schema))
    }

    /// Publishes an updated off-RT revision-tagged planning environment (never called on
    /// `armd-control`).
    pub fn update_prep_env(&self, env: Arc<super::requests::actions::ActionPrepEnv>) {
        *self.prep_env.lock().unwrap_or_else(|e| e.into_inner()) = Some(env);
    }

    /// Reads the latest published off-RT revision-tagged planning environment.
    pub fn prep_env(&self) -> Option<Arc<super::requests::actions::ActionPrepEnv>> {
        self.prep_env.lock().unwrap_or_else(|e| e.into_inner()).clone()
    }

    /// Records an off-RT measurement payload for action `id` (bounded to 16 entries) and patches
    /// the cached latest snapshot if it already carries `id` without a measurement. Never called
    /// on `armd-control`; lock order is strictly `state -> action_measurements` (never nested in
    /// reverse).
    pub fn record_action_measurement(&self, id: u64, value: serde_json::Value) {
        {
            let mut map = self
                .action_measurements
                .lock()
                .unwrap_or_else(|e| e.into_inner());
            map.insert(id, value.clone());
            while map.len() > 16 {
                if let Some(&oldest) = map.keys().min() {
                    map.remove(&oldest);
                } else {
                    break;
                }
            }
        }
        let mut st = self.state.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(latest) = st.latest.as_ref() {
            let needs_patch = latest
                .obs
                .action
                .as_ref()
                .is_some_and(|a| a.id == id && a.measurement.is_none())
                || latest
                    .obs
                    .previous_action
                    .as_ref()
                    .is_some_and(|a| a.id == id && a.measurement.is_none());
            if needs_patch {
                let mut snap = (**latest).clone();
                if let Some(a) = snap
                    .obs
                    .action
                    .as_mut()
                    .filter(|a| a.id == id && a.measurement.is_none())
                {
                    a.measurement = Some(value.clone());
                }
                if let Some(a) = snap
                    .obs
                    .previous_action
                    .as_mut()
                    .filter(|a| a.id == id && a.measurement.is_none())
                {
                    a.measurement = Some(value);
                }
                st.latest = Some(Arc::new(snap));
            }
        }
    }

    /// Reads back the off-RT sidecar measurement payload recorded for action `id`, if present.
    pub fn action_measurement(&self, id: u64) -> Option<serde_json::Value> {
        self.action_measurements
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(&id)
            .cloned()
    }

    /// Returns the latest recorded `PolicyStats` for `action_id`, if any (used by off-RT
    /// `OffThreadSinks::drain_rt_terminals` so the first terminal frame preserves `policy_stats`).
    #[cfg(feature = "policy-runtime")]
    pub fn action_policy_stats(&self, id: u64) -> Option<crate::motion::PolicyStats> {
        let g = self.policy_diag.lock().unwrap_or_else(|e| e.into_inner());
        g.iter().find(|e| e.action_id == id).map(|e| e.stats)
    }

    #[cfg(not(feature = "policy-runtime"))]
    pub fn action_policy_stats(&self, _id: u64) -> Option<crate::motion::PolicyStats> {
        None
    }

    /// Populates `obs.action.measurement` and `obs.previous_action.measurement` from the off-RT
    /// sidecar store when missing.
    pub fn attach_action_measurement(&self, obs: &mut ArmObservation) {
        let need_cur = obs.action.as_ref().is_some_and(|a| a.measurement.is_none());
        let need_prev = obs
            .previous_action
            .as_ref()
            .is_some_and(|a| a.measurement.is_none());
        if !need_cur && !need_prev {
            return;
        }
        let map = self
            .action_measurements
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        if map.is_empty() {
            return;
        }
        if let Some(a) = obs.action.as_mut().filter(|a| a.measurement.is_none()) {
            if let Some(v) = map.get(&a.id) {
                a.measurement = Some(v.clone());
            }
        }
        if let Some(a) = obs
            .previous_action
            .as_mut()
            .filter(|a| a.measurement.is_none())
        {
            if let Some(v) = map.get(&a.id) {
                a.measurement = Some(v.clone());
            }
        }
    }
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
        RtInputs {
            positions: p,
            velocities: p,
            commanded_positions: p,
            commanded_velocities: p,
            feedback_instants: i,
            feedback_seqs: s,
            feedback_queue_bounds: &[],
            owner: PublishedOwnerMeta::default(),
            meta: RtMotionMeta::default(),
        }
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
        // slots rotate: the unconsumed seq-1 observation goes to the retire ring, not back to
        // the tick; the next reader frees it
        assert!(publisher.publish(13, now, now, obs(3.0), inputs(&p, &s, &i)).is_none());
        assert!(publisher.publish(14, now, now, obs(4.0), inputs(&p, &s, &i)).is_none());
        assert_eq!(hub.retire.head.load(Ordering::Relaxed), 1);
        assert_eq!(hub.latest().unwrap().obs.timestamp, 4.0);
        assert_eq!(hub.retire.tail.load(Ordering::Relaxed), 1);
        assert_eq!(hub.tick_frees(), 0);
    }

    #[test]
    fn without_readers_the_retire_ring_fills_and_the_tick_frees_are_counted() {
        let (mut publisher, hub) = state_channel(7);
        let now = Instant::now();
        let (p, s, i) = ([0.0f32; 7], [0u64; 7], [None; 7]);
        let mut back = 0;
        // the first two publications fill empty slots; each later one displaces one
        for k in 0..(2 + RETIRE_CAPACITY + 3) {
            if publisher.publish(k as u64, now, now, obs(k as f64), inputs(&p, &s, &i)).is_some() { back += 1; }
        }
        assert_eq!((back, hub.tick_frees()), (3, 3));
        hub.latest();
        assert!(publisher.publish(99, now, now, obs(99.0), inputs(&p, &s, &i)).is_none(), "room again after a reader");
        assert_eq!(hub.latest().unwrap().obs.timestamp, 99.0);
    }

    #[test]
    fn arm_feedback_matches_the_supervisor_view() {
        let (mut publisher, hub) = state_channel(3);
        let now = Instant::now();
        publisher.publish(1, now, now, obs(0.0), RtInputs {
            positions: &[1.0, 2.0, 3.0], velocities: &[0.1, 0.2, 0.3],
            commanded_positions: &[1.0, 2.0, 3.0], commanded_velocities: &[0.0, 0.0, 0.0],
            feedback_instants: &[Some(now), None, Some(now)], feedback_seqs: &[5, 6, 7],
            feedback_queue_bounds: &[], owner: PublishedOwnerMeta::default(),
            meta: RtMotionMeta::default(),
        });
        let (p, v, i, s) = hub.latest().unwrap().rt.arm_feedback(&[0, 2]);
        assert_eq!((p, v, i, s), (vec![1.0, 2.0, 3.0], vec![0.1, 0.3], vec![Some(now), Some(now)], vec![5, 7]));
    }

    #[test]
    fn startup_reader_holding_lock_before_prewarm_never_rotates_empty_front_to_producer() {
        let (mut publisher, hub) = state_channel(7);
        let now = Instant::now();
        let (p, s, i) = ([0.0f32; 7], [1u64; 7], [Some(now); 7]);

        // Reader acquires `hub.state` BEFORE `try_prewarm_observations` runs and pauses.
        let (entered_tx, entered_rx) = std::sync::mpsc::sync_channel::<()>(1);
        let (resume_tx, resume_rx) = std::sync::mpsc::sync_channel::<()>(1);
        let hub_r = Arc::clone(&hub);
        let reader = std::thread::spawn(move || {
            let mut st = hub_r.state.lock().unwrap();
            assert!(st.consumer.front().obs.is_none());
            assert!(hub_r.prewarm_template.get().is_none());
            entered_tx.send(()).unwrap();
            // Pause while producer runs `try_prewarm_observations` (hits WouldBlock) + publishes!
            resume_rx.recv().unwrap();
            let snap = hub_r.refresh(&mut st).expect("published snapshot visible after resume");
            assert_eq!(snap.meta.seq, 2);
            assert!(st.consumer.front().obs.is_some());
        });

        entered_rx.recv().unwrap();
        // Contended startup prewarm warms back & middle without blocking and records template.
        publisher.try_prewarm_observations(|| obs(0.0));
        assert!(hub.prewarm_template.get().is_some());
        // Publish twice while reader still holds `hub.state`.
        for seq in 1..=2u64 {
            let mut o = publisher.take_back_observation().expect("back slot prewarmed");
            o.timestamp = seq as f64;
            assert!(publisher.publish(seq, now, now + Duration::from_millis(40), o, inputs(&p, &s, &i)).is_none());
        }
        // Resume reader: `refresh()` must warm `front` BEFORE rotating it into `middle`.
        resume_tx.send(()).unwrap();
        reader.join().unwrap();

        // Subsequent publications rotate that consumer slot back into the producer: NEVER empty!
        for seq in 3..=6u64 {
            let mut o = publisher
                .take_back_observation()
                .expect("rotated consumer front slot must be prewarmed off-thread");
            o.timestamp = seq as f64;
            assert!(publisher.publish(seq, now, now + Duration::from_millis(40), o, inputs(&p, &s, &i)).is_none());
        }
        assert_eq!(hub.tick_frees(), 0);
    }
}
