//! Broker contracts between normal-priority producers and the RT executor (armd 1.6 RT core,
//! step 2 foundation; design section 3 and its amendments).
//!
//! These types are frozen for tracks A-E. Nothing in production routes through them yet:
//! the WS / REST paths still call the supervisor under its lock, and the production adapter
//! ([`super::ops`]) delegates every operation to that existing behaviour.
//!
//! # Channels
//!
//! - **Command ring**: one [`Broker`] (normal priority) serializes every producer into one
//!   `rtrb` SPSC ring of `Copy` [`Descriptor`]s ([`COMMAND_CAPACITY`] = 32). The RT side
//!   ([`RtIntake`]) takes **at most one ordinary command per release** (amendment: one
//!   ordinary commit per release).
//! - **Plan slots**: [`PLAN_SLOTS`] = 8 pre-touched, generation-tagged, immutable slots
//!   ([`PlanSlots`]). The broker stages a plan into a free slot and publishes it; the RT
//!   reads it through a [`PlanHandle`] whose generation must match, and retires it when
//!   the primitive no longer needs it; the broker reclaims retired slots. A handle of a
//!   reused slot reads as stale (never as another plan).
//! - **Reply ring**: RT -> broker, `rtrb`. The broker reserves one reply (the ACK or
//!   refusal) before admitting a command and one more (the terminal result) for a command
//!   that starts an action, so the RT never finds the reply ring full and never waits.
//!   A reply whose client went away is dropped by the broker and counted
//!   ([`BrokerStats::abandoned`]); its reservation is released all the same.
//! - **Latest target** ([`super::mailbox`]), **urgent stop** ([`super::urgent`]) and
//!   **versioned configuration** ([`super::config_iface`]) are separate interfaces.
//!
//! # Identities
//!
//! Every descriptor carries the arm and boot identity, the broker's request id, the client's
//! own sequence number, the authority generations it was admitted under ([`Authority`]) and
//! its original monotonic deadline. A [`RequestId`] (admission) is distinct from the
//! [`ActionId`] of a committed action (ACK) and from the physical completion (terminal).
//!
//! # Admission refusals
//!
//! A refused admission ([`AdmitError`]) has no side effect on the RT side: nothing is
//! queued, no plan slot stays taken, active motion is untouched.

use std::collections::HashMap;
use std::cell::{Cell, UnsafeCell};
use std::marker::PhantomData;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Instant;

use super::ids::ArmId;

/// Command descriptors per executor.
pub const COMMAND_CAPACITY: usize = 32;
/// Immutable plan slots per executor.
pub const PLAN_SLOTS: usize = 8;
/// Replies the RT can have outstanding: an ACK per queued command plus a terminal per plan
/// slot (each action holds one slot until its terminal result).
pub const REPLY_CAPACITY: usize = COMMAND_CAPACITY + PLAN_SLOTS;

/// Identity of one armd process run or executor instance (non-zero; changes on every start).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BootId(pub u64);

static BOOT_BASE: std::sync::OnceLock<u64> = std::sync::OnceLock::new();
static NEXT_BOOT: AtomicU64 = AtomicU64::new(1);
static PROCESS_BOOT: std::sync::OnceLock<BootId> = std::sync::OnceLock::new();

impl BootId {
    fn base() -> u64 {
        *BOOT_BASE.get_or_init(|| {
            let time_ns = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(1u64, |d| d.as_nanos() as u64);
            let pid = u64::from(std::process::id());
            time_ns.wrapping_mul(0x9e37_79b9_7f4a_7c15) ^ pid.rotate_left(32)
        })
    }

    /// Generates a fresh, non-zero boot identity (unique across process restarts and
    /// strictly distinct across executors within the same process).
    pub fn fresh() -> Self {
        let base = Self::base();
        loop {
            let seq = NEXT_BOOT.fetch_add(1, Ordering::Relaxed);
            let id = base.wrapping_add(seq);
            if id != 0 { return Self(id); }
        }
    }

    /// Process-wide non-zero boot identity for the current `armd` run (replaces the
    /// `BootId(0)` placeholder at legacy boundaries until step 1 owns boot ids on `ArmCore`).
    pub fn process() -> Self {
        *PROCESS_BOOT.get_or_init(Self::fresh)
    }

    /// True for the uninitialized zero placeholder (`BootId(0)`), which commit checks refuse.
    #[inline]
    pub fn is_zero(self) -> bool { self.0 == 0 }
}

/// Broker-assigned id of one admitted request, unique within a boot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct RequestId(pub u64);

/// Id of a committed action (the supervisor's `ActionStatus::id`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ActionId(pub u64);

/// A producer of commands (a WS connection, a REST request, a worker).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ClientId(pub u64);

/// The authority generations a command was admitted under. The RT rechecks each at commit
/// against its current values ([`Authority::admits`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Authority {
    /// `ActiveLease::generation` of the holder (0: no lease needed).
    pub lease_generation: u64,
    /// Safety epoch (`SafetyManager::epoch_id`).
    pub epoch: u64,
    /// Authorized-stop generation ([`super::urgent::StopRequests`]); a later stop invalidates.
    pub stop_generation: u64,
    /// Configuration revision the plan was computed against (`workspace_revision`).
    pub config_revision: u64,
    /// Parent action generation for a child primitive (0: not a child). In the current
    /// authority: the generation of the parent action that may install children now (0: none).
    pub parent_generation: u64,
}

impl Authority {
    /// The rejection code when a command admitted under `self` may not commit under
    /// `current`, None when it may. Checked in this order: stop, epoch, lease, config, parent.
    ///
    /// `current` is the core's authority at the commit instant ([`super::ops::CoreOps::authority`]):
    /// its `lease_generation` is that of a lease **valid at that instant** (0 when none is, so
    /// a command queued before its lease expired is refused even before the watchdog runs).
    /// Lease generations are unique per acquisition, so an equal generation names the same
    /// holder. The parent is compared for children only (`parent_generation != 0`).
    pub fn admits(&self, current: &Authority) -> Option<&'static str> {
        if self.stop_generation != current.stop_generation { return Some("STOPPED"); }
        if self.epoch != current.epoch { return Some("EPOCH_MISMATCH"); }
        if self.lease_generation != current.lease_generation { return Some("LEASE_LOST"); }
        if self.config_revision != current.config_revision { return Some("PLAN_STALE"); }
        if self.parent_generation != 0 && self.parent_generation != current.parent_generation { return Some("PARENT_REPLACED"); }
        None
    }

    /// Authority check for stop-direction operations (such as torque disable) that must
    /// remain executable across an authorized Stop: exempt from `stop_generation`, while
    /// preserving epoch, lease, config, and parent checks.
    pub fn admits_stop_direction(&self, current: &Authority) -> Option<&'static str> {
        if self.epoch != current.epoch { return Some("EPOCH_MISMATCH"); }
        if self.lease_generation != current.lease_generation { return Some("LEASE_LOST"); }
        if self.config_revision != current.config_revision { return Some("PLAN_STALE"); }
        if self.parent_generation != 0 && self.parent_generation != current.parent_generation { return Some("PARENT_REPLACED"); }
        None
    }
}

/// Who sent a command and under which authority, until when.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Stamp {
    pub arm: ArmId,
    pub boot: BootId,
    pub request: RequestId,
    pub client: ClientId,
    /// The client's own sequence number (e.g. the WS `request_id` counter).
    pub client_seq: u64,
    pub authority: Authority,
    /// Original monotonic deadline: the RT refuses the command after it (`DEADLINE_EXPIRED`).
    pub deadline: Instant,
}

impl Stamp {
    /// The rejection code at commit (`now`, RT identity and current authority), None to commit.
    /// Refuses `BootId(0)` even when both stamp and target pass zero (fail-closed).
    pub fn check(&self, arm: ArmId, boot: BootId, now: Instant, current: &Authority) -> Option<&'static str> {
        if self.arm != arm || self.boot != boot || self.boot.is_zero() { return Some("WRONG_TARGET"); }
        if now > self.deadline { return Some("DEADLINE_EXPIRED"); }
        self.authority.admits(current)
    }

    /// Stop-direction stamp check (identity, deadline, and [`Authority::admits_stop_direction`]):
    /// exempt from `stop_generation` so a Stop never blocks a queued torque disable.
    pub fn check_stop_direction(&self, arm: ArmId, boot: BootId, now: Instant, current: &Authority) -> Option<&'static str> {
        if self.arm != arm || self.boot != boot || self.boot.is_zero() { return Some("WRONG_TARGET"); }
        if now > self.deadline { return Some("DEADLINE_EXPIRED"); }
        self.authority.admits_stop_direction(current)
    }
}

/// A handle of one published plan slot: the pool it belongs to, the slot index and the
/// generation it was staged in. A handle reads only from its own pool ([`PlanSlots`] get a
/// process-unique pool id), and a slot's generation never repeats: it does not wrap, a slot
/// whose generation reached [`MAX_PLAN_GENERATION`] is never staged again.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PlanHandle {
    pool: u64,
    index: u8,
    generation: u64,
}

impl PlanHandle {
    pub fn pool(&self) -> u64 { self.pool }
    pub fn index(&self) -> usize { usize::from(self.index) }
    pub fn generation(&self) -> u64 { self.generation }
}

/// Commands that do not need a plan slot. Tracks extend this list through the integrator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImmediateOp {
    Heartbeat,
    TorqueOn,
    TorqueOff,
}

/// A driver transition the bus owner carries out over several releases (track A): the ACK
/// says it started, the terminal reply says it was confirmed (`Completed`) or failed. The
/// client-facing torque command stays synchronous: its worker awaits the terminal.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransitionOp {
    TorqueOn,
    TorqueOff,
}

/// What a descriptor asks the RT to do.
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(clippy::large_enum_variant)] // Fixed-size `Copy` SPSC ring descriptor (32 slots): boxing would allocate on RT.
pub enum CommandKind {
    /// Install the primitive of a plan slot; starts an action (a terminal reply follows the ACK).
    Install { plan: PlanHandle },
    /// Install a prepared home motion from a plan slot; starts a `home` action.
    InstallHome { plan: PlanHandle },
    /// Install a prepared park motion (or `holding_after_clear` park rearm); starts a `park` action.
    InstallPark { plan: PlanHandle },
    /// Attach a prepared park continuation trajectory to an existing `park` action (ACK/Refused only).
    ContinuePark { plan: PlanHandle },
    /// A command without a plan; ACK or refusal only.
    Immediate(ImmediateOp),
    /// Start a driver transition; a terminal reply follows the ACK.
    Transition(TransitionOp),
    /// Lease lifecycle operation (`Acquire`, `Preempt`, `Release`, `Disconnect`); single `Reply::Lease` or `Refused`.
    Lease(super::requests::lease::LeaseRequest),
    /// Action-family prepared commands and macro lifecycle (A6 + A7).
    #[cfg(not(target_arch = "wasm32"))]
    Action(super::requests::actions::ActionCommand),
}

impl CommandKind {
    /// True for action-installing commands that supersede and own `ReleaseEngine::active`.
    pub fn is_install(&self) -> bool {
        matches!(self, Self::Install { .. } | Self::InstallHome { .. } | Self::InstallPark { .. })
    }
    /// A terminal reply follows the ACK (an install, transition, prepared action, or `BeginMacro`).
    pub fn starts_action(&self) -> bool {
        match self {
            Self::Install { .. }
            | Self::InstallHome { .. }
            | Self::InstallPark { .. }
            | Self::Transition(_) => true,
            #[cfg(not(target_arch = "wasm32"))]
            Self::Action(a) => a.starts_action(),
            Self::ContinuePark { .. } | Self::Immediate(_) | Self::Lease(_) => false,
        }
    }
    pub fn plan(&self) -> Option<PlanHandle> {
        match self {
            Self::Install { plan }
            | Self::InstallHome { plan }
            | Self::InstallPark { plan }
            | Self::ContinuePark { plan } => Some(*plan),
            #[cfg(not(target_arch = "wasm32"))]
            Self::Action(a) => a.plan(),
            Self::Immediate(_) | Self::Transition(_) | Self::Lease(_) => None,
        }
    }
}

/// One entry of the command ring.
#[derive(Debug, Clone, Copy)]
pub struct Descriptor {
    pub stamp: Stamp,
    pub kind: CommandKind,
}

/// How an action ended physically. Codes are the action's own `code` (`ActionStatus::code`),
/// never a generic placeholder when the action named one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Outcome {
    Completed,
    /// Ended with a failure code (the action's `code`; `FAILED` only when it named none or
    /// one outside [`super::ops::TERMINAL_CODES`]).
    Failed(&'static str),
    /// Replaced by a later commit before it completed (`SUPERSEDED`).
    Superseded,
    /// Ended by an authorized stop (`STOPPED`, or cancelled without a code).
    Stopped,
    /// Cancelled for another cause (`LEASE_LOST`, `ESTOP`, `COLLISION_HOLD`, ...).
    Cancelled(&'static str),
}

/// RT -> broker.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reply {
    /// Committed: `commit_seq` is the RT commit sequence, `action` the committed action.
    Ack { request: RequestId, commit_seq: u64, action: Option<ActionId> },
    /// Lease lifecycle outcome (`CommandKind::Lease`).
    Lease {
        request: RequestId,
        commit_seq: u64,
        outcome: super::requests::lease::LeaseOutcome,
    },
    /// Not committed (nothing changed on the RT side; a plan slot was retired).
    Refused { request: RequestId, code: &'static str },
    /// The physical completion of an acknowledged action; `children`: the child primitives
    /// a macro action ran (0 for a plain primitive or a transition).
    Terminal { request: RequestId, action: ActionId, outcome: Outcome, children: u32 },
}

impl Reply {
    pub fn request(&self) -> RequestId {
        match self {
            Self::Ack { request, .. }
            | Self::Lease { request, .. }
            | Self::Refused { request, .. }
            | Self::Terminal { request, .. } => *request,
        }
    }
}

// ---------------------------------------------------------------------------------------
// Plan slots

const FREE: u64 = 0;
const WRITING: u64 = 1;
const READY: u64 = 2;
const RETIRED: u64 = 3;

/// The last generation a slot is staged in (62 bits of the slot word; the low 2 hold the
/// state). Unreachable in practice (one install per release at 50 Hz: 2.9e9 years); a slot
/// that gets there stays out of use instead of reusing generation numbers.
pub const MAX_PLAN_GENERATION: u64 = (1 << 62) - 1;

fn pack(generation: u64, state: u64) -> u64 { debug_assert!(generation <= MAX_PLAN_GENERATION); (generation << 2) | state }
fn unpack(word: u64) -> (u64, u64) { (word >> 2, word & 3) }

/// Pool ids of [`plan_slots`]: unique in the process.
static NEXT_POOL: AtomicU64 = AtomicU64::new(1);

/// `PLAN_SLOTS` (or a test capacity) plan slots shared by one writer and one reader.
pub struct PlanSlots<P> {
    pool: u64,
    words: Box<[AtomicU64]>,
    /// Active immutable borrows ([`PlanSlotRef`]) per slot: [`PlanWriter::reclaim`] never
    /// moves a `RETIRED` slot back to `FREE` while its pin count is non-zero.
    pins: Box<[AtomicU32]>,
    plans: Box<[UnsafeCell<P>]>,
}

// SAFETY: a slot's plan is written only by the single `PlanWriter` while its state is
// WRITING (no handle to it exists yet, and reached only after `RETIRED` with `pins[i] == 0`
// observed via Acquire), and read only through `PlanReader::get` while the state is READY
// with the handle's generation (Acquire load after the writer's Release store) or through
// `PlanSlotRef` while `pins[i] > 0` (acquired via `PlanReader::pin`, which requires `P: Sync`,
// while READY and released on drop). The writer does not touch a READY or pinned RETIRED slot
// until the reader moved it to RETIRED (Release), all borrows dropped (Release), and the
// writer observed both (Acquire). `PlanReader` is neither `Clone` nor `Sync` (so `get` only
// ever hands out `&P` on one thread at a time for non-`Sync` `P`), and pinning requires
// `P: Sync` so moving `PlanReader` to another thread while a `PlanSlotRef` lives cannot give
// two threads concurrent `&P` to a `!Sync` type.
unsafe impl<P: Send> Sync for PlanSlots<P> {}

/// An immutable, pinned borrow of a published plan slot that can outlive a single release
/// until the active motion retires it (zero allocations, zero frees while the pool lives).
///
/// [`PlanWriter::reclaim`] will not overwrite or reuse the slot while any [`PlanSlotRef`]
/// to it exists, even if [`PlanReader::retire`] has already been called.
pub struct PlanSlotRef<P> {
    slots: Arc<PlanSlots<P>>,
    handle: PlanHandle,
}

// SAFETY: `PlanSlotRef` only hands out `&P` to an immutable slot held pinned (`pins[index] > 0`),
// and is `Clone`; like `Arc<P>`, sending or sharing it across threads requires `P: Send + Sync`.
unsafe impl<P: Send + Sync> Send for PlanSlotRef<P> {}
unsafe impl<P: Send + Sync> Sync for PlanSlotRef<P> {}

impl<P> PlanSlotRef<P> {
    pub fn handle(&self) -> PlanHandle { self.handle }
}

impl<P> std::ops::Deref for PlanSlotRef<P> {
    type Target = P;
    fn deref(&self) -> &P {
        // SAFETY: `self` holds a live pin on `self.handle.index()`, acquired while the slot
        // was READY at `self.handle.generation()`; `PlanWriter` cannot reclaim or rewrite it
        // until all pins drop with Release ordering.
        unsafe { &*self.slots.plans[self.handle.index()].get() }
    }
}

impl<P> Clone for PlanSlotRef<P> {
    fn clone(&self) -> Self {
        let prev = self.slots.pins[self.handle.index()].fetch_add(1, Ordering::Relaxed);
        assert!(prev < i32::MAX as u32, "plan slot pin count overflow");
        Self { slots: self.slots.clone(), handle: self.handle }
    }
}

impl<P> Drop for PlanSlotRef<P> {
    fn drop(&mut self) {
        self.slots.pins[self.handle.index()].fetch_sub(1, Ordering::Release);
    }
}

impl<P: std::fmt::Debug> std::fmt::Debug for PlanSlotRef<P> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PlanSlotRef").field("handle", &self.handle).field("plan", &**self).finish()
    }
}

/// The broker's side of the plan slots.
pub struct PlanWriter<P> {
    slots: Arc<PlanSlots<P>>,
    retired: u64,
}

/// The RT's side of the plan slots. Not `Clone` and not `Sync`: exactly one thread reads
/// through it. [`Self::get`] hands out `&P` for `P: Send` only, so two threads sharing a
/// `&PlanReader` could otherwise both reach the same slot (e.g. a `&Cell` of it). It is
/// `Send`: it can move to the RT thread.
///
/// ```compile_fail
/// fn sync<T: Sync>() {}
/// sync::<armd::rt_core::broker::PlanReader<std::cell::Cell<u32>>>();
/// ```
///
/// ```compile_fail
/// fn sync<T: Sync>() {}
/// sync::<armd::rt_core::broker::PlanReader<u32>>();
/// ```
///
/// ```
/// fn send<T: Send>() {}
/// send::<armd::rt_core::broker::PlanReader<std::cell::Cell<u32>>>();
/// ```
pub struct PlanReader<P> {
    slots: Arc<PlanSlots<P>>,
    _not_sync: PhantomData<Cell<()>>,
}

/// `capacity` pre-touched plan slots, each built (allocated and write-touched) by `make` now.
pub fn plan_slots<P: Send>(capacity: usize, mut make: impl FnMut() -> P) -> (PlanWriter<P>, PlanReader<P>) {
    assert!(capacity > 0 && capacity <= usize::from(u8::MAX));
    let mut words = Vec::with_capacity(capacity);
    let mut pins = Vec::with_capacity(capacity);
    let mut plans = Vec::with_capacity(capacity);
    for _ in 0..capacity {
        let w = AtomicU64::new(pack(0, FREE));
        w.store(pack(0, FREE), Ordering::Relaxed);
        words.push(w);
        let p = AtomicU32::new(0);
        p.store(0, Ordering::Relaxed);
        pins.push(p);
        plans.push(UnsafeCell::new(std::hint::black_box(make())));
    }
    let slots = Arc::new(PlanSlots {
        pool: NEXT_POOL.fetch_add(1, Ordering::Relaxed),
        words: words.into_boxed_slice(),
        pins: pins.into_boxed_slice(),
        plans: plans.into_boxed_slice(),
    });
    (PlanWriter { slots: slots.clone(), retired: 0 }, PlanReader { slots, _not_sync: PhantomData })
}

impl<P> PlanWriter<P> {
    /// Total plan slots in this pool.
    pub fn capacity(&self) -> usize { self.slots.words.len() }

    /// Moves retired slots with no active [`PlanSlotRef`] borrows back to free; returns how many.
    pub fn reclaim(&mut self) -> usize {
        let mut n = 0;
        for (w, pin) in self.slots.words.iter().zip(self.slots.pins.iter()) {
            let (g, s) = unpack(w.load(Ordering::Acquire));
            if s == RETIRED && pin.load(Ordering::Acquire) == 0 {
                w.store(pack(g, FREE), Ordering::Relaxed);
                n += 1;
            }
        }
        self.retired += n as u64;
        n
    }

    /// Slots reclaimed so far.
    pub fn retired_total(&self) -> u64 { self.retired }

    /// Free slots that can still be staged (after a reclaim).
    pub fn free(&self) -> usize {
        self.slots.words.iter().filter(|w| { let (g, s) = unpack(w.load(Ordering::Acquire)); s == FREE && g < MAX_PLAN_GENERATION }).count()
    }

    /// Writes a plan into a free slot (in place, reusing its memory) and publishes it
    /// immutable. None when every slot is taken.
    pub fn stage(&mut self, write: impl FnOnce(&mut P)) -> Option<PlanHandle> {
        self.reclaim();
        let i = self.slots.words.iter().position(|w| {
            let (g, s) = unpack(w.load(Ordering::Acquire));
            s == FREE && g < MAX_PLAN_GENERATION
        })?;
        let (g, _) = unpack(self.slots.words[i].load(Ordering::Relaxed));
        let generation = g + 1;
        self.slots.words[i].store(pack(generation, WRITING), Ordering::Relaxed);
        // SAFETY: WRITING and no handle exists: only this writer accesses the plan.
        write(unsafe { &mut *self.slots.plans[i].get() });
        self.slots.words[i].store(pack(generation, READY), Ordering::Release);
        Some(PlanHandle { pool: self.slots.pool, index: i as u8, generation })
    }

    /// Takes back a published slot whose descriptor was never queued (admission failed).
    fn unpublish(&mut self, h: PlanHandle) {
        if h.pool != self.slots.pool { return; }
        let idx = h.index();
        let w = &self.slots.words[idx];
        if w.load(Ordering::Acquire) == pack(h.generation, READY) && self.slots.pins[idx].load(Ordering::Acquire) == 0 {
            w.store(pack(h.generation, FREE), Ordering::Relaxed);
        }
    }

    /// Sets slot `index` to `generation`, free (tests: generation exhaustion).
    #[cfg(test)]
    fn force_generation(&mut self, index: usize, generation: u64) { self.slots.words[index].store(pack(generation, FREE), Ordering::Release); }
}

impl<P> PlanReader<P> {
    /// Total plan slots in this pool.
    pub fn capacity(&self) -> usize { self.slots.words.len() }

    /// The plan of `h`, None when the slot was retired or reused since (stale handle).
    pub fn get(&self, h: PlanHandle) -> Option<&P> {
        if h.pool != self.slots.pool { return None; }
        let w = self.slots.words.get(h.index())?;
        if w.load(Ordering::Acquire) != pack(h.generation, READY) { return None; }
        // SAFETY: READY at this generation: immutable until this reader retires it.
        Some(unsafe { &*self.slots.plans[h.index()].get() })
    }

    /// Ends the RT's use of `h` (no allocation, no free: the writer reclaims the slot).
    /// False when `h` was not live. Takes `&mut self` so no `&P` from [`Self::get`] can
    /// outlive the retirement (the writer may rewrite the slot right after).
    ///
    /// ```compile_fail
    /// let (mut w, mut r) = armd::rt_core::broker::plan_slots(1, || 0u32);
    /// let h = w.stage(|p| *p = 7).unwrap();
    /// let p = r.get(h).unwrap();
    /// r.retire(h);
    /// let _ = *p;
    /// ```
    pub fn retire(&mut self, h: PlanHandle) -> bool {
        if h.pool != self.slots.pool { return false; }
        let Some(w) = self.slots.words.get(h.index()) else { return false; };
        w.compare_exchange(pack(h.generation, READY), pack(h.generation, RETIRED), Ordering::Release, Ordering::Relaxed).is_ok()
    }
}

impl<P: Sync> PlanReader<P> {
    /// Pins a live slot for immutable borrowing past this release (zero allocations).
    /// None when `h` belongs to another pool or was retired / reused.
    ///
    /// Requires `P: Sync`: `PlanReader` is `Send`, so safe code can pin a slot on one thread
    /// and move `PlanReader` to another thread to call [`Self::get`]; requiring `P: Sync`
    /// prevents two threads from holding `&P` to a non-`Sync` plan simultaneously.
    ///
    /// ```compile_fail
    /// use std::cell::Cell;
    /// let (mut w, r) = armd::rt_core::broker::plan_slots(1, || Cell::new(0u32));
    /// let h = w.stage(|p| p.set(7)).unwrap();
    /// let _pinned = r.pin(h);
    /// ```
    ///
    /// ```
    /// let (mut w, r) = armd::rt_core::broker::plan_slots(1, || 0u32);
    /// let h = w.stage(|p| *p = 7).unwrap();
    /// let pinned = r.pin(h).unwrap();
    /// assert_eq!(*pinned, 7);
    /// ```
    pub fn pin(&self, h: PlanHandle) -> Option<PlanSlotRef<P>> {
        if h.pool != self.slots.pool { return None; }
        let idx = h.index();
        let w = self.slots.words.get(idx)?;
        if w.load(Ordering::Acquire) != pack(h.generation, READY) { return None; }
        let prev = self.slots.pins[idx].fetch_add(1, Ordering::Acquire);
        assert!(prev < i32::MAX as u32, "plan slot pin count overflow");
        if w.load(Ordering::Acquire) != pack(h.generation, READY) {
            self.slots.pins[idx].fetch_sub(1, Ordering::Release);
            return None;
        }
        Some(PlanSlotRef { slots: self.slots.clone(), handle: h })
    }
}

// ---------------------------------------------------------------------------------------
// Broker (normal priority) and RT intake

/// Why an admission was refused (nothing reached the RT).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdmitError {
    /// The command ring is full (the RT has not drained it).
    CommandsFull,
    /// Every plan slot is taken.
    PlansFull,
    /// Reply capacity is reserved by outstanding requests.
    RepliesFull,
    /// The client disconnected.
    Disconnected,
}

impl AdmitError {
    /// The protocol code of the refusal.
    pub fn code(&self) -> &'static str {
        match self { Self::Disconnected => "DISCONNECTED", _ => "MOTION_BUSY" }
    }
}

/// Counters of one broker.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct BrokerStats {
    pub admitted: u64,
    pub refused_admission: u64,
    pub acked: u64,
    pub refused_by_rt: u64,
    pub terminals: u64,
    /// Replies dropped because their client was gone.
    pub abandoned: u64,
    /// Plan slots reclaimed.
    pub retired: u64,
    /// Replies without a matching reservation (unknown request, a second ACK, a terminal
    /// before the ACK or twice): dropped, never delivered. Stays 0.
    pub unexpected: u64,
}

struct Pending {
    client: ClientId,
    /// ACK / refusal still to come.
    ack: bool,
    /// Terminal still to come (reserved).
    terminal: bool,
}

/// Outcome of one caller delivery attempt in [`Broker::try_poll`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeliveryDecision {
    /// Delivered to the downstream receiver; release its reservation.
    Delivered,
    /// Downstream writer is temporarily full; keep the reply stashed in the broker and retain
    /// its reservation (no drop, no unbounded queue, exact FIFO preserved).
    Backpressured,
    /// Downstream client is gone; drop the reply, count it as abandoned, and release its reservation.
    Abandoned,
}

/// What a command asks for, with the plan to stage for an install.
#[allow(clippy::large_enum_variant)] // Matches `CommandKind` zero-alloc value passing without heap boxing.
pub enum Submit<'a, P> {
    Install(&'a dyn Fn(&mut P)),
    InstallHome(&'a dyn Fn(&mut P)),
    InstallPark(&'a dyn Fn(&mut P)),
    ContinuePark(&'a dyn Fn(&mut P)),
    Immediate(ImmediateOp),
    Transition(TransitionOp),
    Lease(super::requests::lease::LeaseRequest),
    #[cfg(not(target_arch = "wasm32"))]
    Action(super::requests::actions::ActionSubmit<'a, P>),
}

/// Capacities of one broker (tests use tiny ones).
#[derive(Debug, Clone, Copy)]
pub struct Capacities {
    pub commands: usize,
    pub plans: usize,
    pub replies: usize,
}

impl Default for Capacities {
    fn default() -> Self { Self { commands: COMMAND_CAPACITY, plans: PLAN_SLOTS, replies: REPLY_CAPACITY } }
}

/// The normal-priority side: serializes producers, owns admission and reply delivery.
pub struct Broker<P> {
    arm: ArmId,
    boot: BootId,
    next_request: u64,
    commands: rtrb::Producer<Descriptor>,
    replies: rtrb::Consumer<Reply>,
    stashed_reply: Option<Reply>,
    plans: PlanWriter<P>,
    reply_capacity: usize,
    reserved: usize,
    pending: HashMap<RequestId, Pending>,
    disconnected: std::collections::HashSet<ClientId>,
    #[cfg(not(target_arch = "wasm32"))]
    closed_bitset: Arc<super::requests::actions::ClosedBitset>,
    stats: BrokerStats,
}

/// The RT side: one ordinary command per release, replies without waiting.
///
/// Replies can only be produced through tokens, so each reserved reply is sent exactly once:
/// [`Self::take`] returns a [`Taken`] that is answered once ([`Taken::ack`] or
/// [`Taken::refuse`]; dropping it unanswered refuses with `INTERNAL`), and an acknowledged
/// install yields an [`Installed`] that [`Self::terminal`] consumes. None of them is `Clone`.
pub struct RtIntake<P> {
    arm: ArmId,
    boot: BootId,
    commands: rtrb::Consumer<Descriptor>,
    replies: rtrb::Producer<Reply>,
    plans: PlanReader<P>,
    commit_seq: u64,
    /// The current release (0: none started).
    release: u64,
    /// The release whose ordinary command was taken.
    taken_in: u64,
    /// Acknowledged installs whose terminal is still to come.
    installed: usize,
    /// Replies the reply ring refused (a reservation bug; must stay 0).
    reply_overflow: u64,
}

/// One ordinary command taken in the current release. Answer it exactly once: [`Self::ack`]
/// with the [`super::ops::Committed`] proof made for this very request, or [`Self::refuse`].
/// Dropping it unanswered refuses the command with `INTERNAL` (no reply is ever missing).
#[must_use = "answer the command with ack or refuse"]
pub struct Taken<'a, P> {
    intake: &'a mut RtIntake<P>,
    d: Descriptor,
    answered: bool,
}

/// An acknowledged install: its terminal is still owed ([`RtIntake::terminal`] consumes it).
#[derive(Debug, PartialEq, Eq)]
#[must_use = "an acknowledged install owes its terminal reply (RtIntake::terminal)"]
pub struct Installed {
    request: RequestId,
    /// None for a transition.
    plan: Option<PlanHandle>,
    action: ActionId,
}

impl Installed {
    pub fn request(&self) -> RequestId { self.request }
    pub fn action(&self) -> ActionId { self.action }
    pub fn plan(&self) -> Option<PlanHandle> { self.plan }
}

/// Result of [`Taken::ack`].
#[derive(Debug, PartialEq, Eq)]
pub struct Acked {
    pub commit_seq: u64,
    /// The install's terminal token (None for an immediate command).
    pub installed: Option<Installed>,
}

/// A broker and its RT intake with `caps`; plan slots built by `make` (pre-touched).
pub fn channel<P: Send>(arm: ArmId, boot: BootId, caps: Capacities, make: impl FnMut() -> P) -> (Broker<P>, RtIntake<P>) {
    let (ctx, crx) = rtrb::RingBuffer::new(caps.commands);
    let (rtx, rrx) = rtrb::RingBuffer::new(caps.replies);
    let (pw, pr) = plan_slots(caps.plans, make);
    (
        Broker {
            arm,
            boot,
            next_request: 0,
            commands: ctx,
            replies: rrx,
            stashed_reply: None,
            plans: pw,
            reply_capacity: caps.replies,
            reserved: 0,
            pending: HashMap::new(),
            disconnected: Default::default(),
            #[cfg(not(target_arch = "wasm32"))]
            closed_bitset: Arc::new(super::requests::actions::ClosedBitset::new()),
            stats: BrokerStats::default(),
        },
        RtIntake { arm, boot, commands: crx, replies: rtx, plans: pr, commit_seq: 0, release: 0, taken_in: 0, installed: 0, reply_overflow: 0 },
    )
}

impl<P> Broker<P> {
    pub fn arm(&self) -> ArmId { self.arm }
    pub fn boot(&self) -> BootId { self.boot }
    pub fn plan_capacity(&self) -> usize { self.plans.capacity() }
    pub fn stats(&self) -> BrokerStats { BrokerStats { retired: self.plans.retired_total(), ..self.stats } }

    /// Shared lock-free closed-connection bitset read on `armd-control` before every action commit.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn closed_bitset(&self) -> Arc<super::requests::actions::ClosedBitset> {
        Arc::clone(&self.closed_bitset)
    }

    /// Reclaims retired plan slots whose [`PlanSlotRef`] pin count has reached zero.
    pub fn reclaim_plans(&mut self) -> usize { self.plans.reclaim() }

    /// Reply slots currently reserved.
    pub fn reserved(&self) -> usize { self.reserved }

    /// Requests waiting for an ACK or a terminal.
    pub fn outstanding(&self) -> usize { self.pending.len() }

    /// The client is gone: its later replies are dropped (counted as abandoned) and its
    /// connection ID is marked closed in the lock-free fence read on `armd-control`.
    pub fn disconnect(&mut self, client: ClientId) {
        #[cfg(not(target_arch = "wasm32"))]
        self.closed_bitset.mark_closed(client.0);
        if self.pending.values().any(|p| p.client == client) { self.disconnected.insert(client); }
    }

    /// Admits one command: reserves its replies, stages its plan, queues its descriptor.
    pub fn submit(&mut self, client: ClientId, client_seq: u64, authority: Authority, deadline: Instant, what: Submit<'_, P>) -> Result<RequestId, AdmitError> {
        if self.disconnected.contains(&client) {
            self.stats.refused_admission += 1;
            return Err(AdmitError::Disconnected);
        }
        let terminal = match &what {
            Submit::Install(_)
            | Submit::InstallHome(_)
            | Submit::InstallPark(_)
            | Submit::Transition(_) => true,
            #[cfg(not(target_arch = "wasm32"))]
            Submit::Action(a) => a.starts_action(),
            Submit::ContinuePark(_) | Submit::Immediate(_) | Submit::Lease(_) => false,
        };
        let need = 1 + usize::from(terminal);
        let refused = if self.commands.slots() == 0 { Some(AdmitError::CommandsFull) }
            else if self.reserved + need > self.reply_capacity { Some(AdmitError::RepliesFull) }
            else { None };
        if let Some(e) = refused {
            self.stats.refused_admission += 1;
            return Err(e);
        }
        let kind = match what {
            Submit::Install(write) => match self.plans.stage(write) {
                Some(plan) => CommandKind::Install { plan },
                None => {
                    self.stats.refused_admission += 1;
                    return Err(AdmitError::PlansFull);
                }
            },
            Submit::InstallHome(write) => match self.plans.stage(write) {
                Some(plan) => CommandKind::InstallHome { plan },
                None => {
                    self.stats.refused_admission += 1;
                    return Err(AdmitError::PlansFull);
                }
            },
            Submit::InstallPark(write) => match self.plans.stage(write) {
                Some(plan) => CommandKind::InstallPark { plan },
                None => {
                    self.stats.refused_admission += 1;
                    return Err(AdmitError::PlansFull);
                }
            },
            Submit::ContinuePark(write) => match self.plans.stage(write) {
                Some(plan) => CommandKind::ContinuePark { plan },
                None => {
                    self.stats.refused_admission += 1;
                    return Err(AdmitError::PlansFull);
                }
            },
            Submit::Immediate(op) => CommandKind::Immediate(op),
            Submit::Transition(op) => CommandKind::Transition(op),
            Submit::Lease(req) => CommandKind::Lease(req),
            #[cfg(not(target_arch = "wasm32"))]
            Submit::Action(a) => {
                use super::requests::actions::{ActionCommand, ActionSubmit};
                match a {
                    ActionSubmit::InstallPrepared {
                        write,
                        kind,
                        prepared_boot,
                        admission_request_id,
                        check_workspace_revision,
                        poses_revision,
                        calibration_revision,
                        gripper_calib_revision,
                        power_budget_revision,
                    } => match self.plans.stage(write) {
                        Some(plan) => CommandKind::Action(ActionCommand::InstallPrepared {
                            plan,
                            kind,
                            prepared_boot,
                            admission_request_id,
                            check_workspace_revision,
                            poses_revision,
                            calibration_revision,
                            gripper_calib_revision,
                            power_budget_revision,
                        }),
                        None => {
                            self.stats.refused_admission += 1;
                            return Err(AdmitError::PlansFull);
                        }
                    },
                    ActionSubmit::BeginMacro(spec) => CommandKind::Action(ActionCommand::BeginMacro(spec)),
                    ActionSubmit::InstallChild {
                        write,
                        prepared_boot,
                        check_workspace_revision,
                        calibration_revision,
                        gripper_calib_revision,
                        power_budget_revision,
                    } => match self.plans.stage(write) {
                        Some(plan) => CommandKind::Action(ActionCommand::InstallChild {
                            plan,
                            prepared_boot,
                            check_workspace_revision,
                            calibration_revision,
                            gripper_calib_revision,
                            power_budget_revision,
                        }),
                        None => {
                            self.stats.refused_admission += 1;
                            return Err(AdmitError::PlansFull);
                        }
                    },
                    ActionSubmit::FinishMacro(spec) => CommandKind::Action(ActionCommand::FinishMacro(spec)),
                }
            }
        };
        self.next_request += 1;
        let request = RequestId(self.next_request);
        let d = Descriptor { stamp: Stamp { arm: self.arm, boot: self.boot, request, client, client_seq, authority, deadline }, kind };
        if self.commands.push(d).is_err() {
            // single producer and slots() > 0 above: unreachable; keep it side-effect free
            if let Some(plan) = kind.plan() { self.plans.unpublish(plan); }
            self.stats.refused_admission += 1;
            return Err(AdmitError::CommandsFull);
        }
        self.reserved += need;
        self.pending.insert(request, Pending { client, ack: true, terminal });
        self.stats.admitted += 1;
        Ok(request)
    }

    /// Drains the RT's replies in exact global FIFO order with bounded backpressure: when
    /// `deliver` returns [`DeliveryDecision::Backpressured`], the reply is stashed in the broker
    /// with its reservation intact and polling stops until the caller retries. Returns the
    /// number of replies processed (delivered or abandoned).
    pub fn try_poll(&mut self, mut deliver: impl FnMut(ClientId, Reply) -> DeliveryDecision) -> usize {
        let mut n = 0;
        loop {
            let reply = match self.stashed_reply.take() {
                Some(r) => r,
                None => match self.replies.pop() {
                    Ok(r) => r,
                    Err(_) => break,
                },
            };
            let request = reply.request();
            let Some((client, expected)) = self.pending.get(&request).map(|p| {
                let expected = match reply {
                    Reply::Ack { .. } | Reply::Lease { .. } | Reply::Refused { .. } => p.ack,
                    Reply::Terminal { .. } => p.terminal && !p.ack,
                };
                (p.client, expected)
            }) else {
                n += 1;
                self.stats.unexpected += 1;
                continue;
            };
            if !expected {
                n += 1;
                self.stats.unexpected += 1;
                continue;
            }
            let decision = if self.disconnected.contains(&client) {
                DeliveryDecision::Abandoned
            } else {
                deliver(client, reply)
            };
            if decision == DeliveryDecision::Backpressured {
                self.stashed_reply = Some(reply);
                break;
            }
            n += 1;
            let done = {
                let p = self.pending.get_mut(&request).expect("checked above");
                match reply {
                    Reply::Ack { .. } | Reply::Lease { .. } => {
                        self.stats.acked += 1;
                        p.ack = false;
                        self.reserved -= 1;
                    }
                    Reply::Refused { .. } => {
                        self.stats.refused_by_rt += 1;
                        p.ack = false;
                        self.reserved -= 1;
                        if p.terminal { p.terminal = false; self.reserved -= 1; }
                    }
                    Reply::Terminal { .. } => {
                        self.stats.terminals += 1;
                        p.terminal = false;
                        self.reserved -= 1;
                    }
                }
                !p.ack && !p.terminal
            };
            if done { self.pending.remove(&request); }
            if decision == DeliveryDecision::Abandoned { self.stats.abandoned += 1; }
            if done && self.disconnected.contains(&client) && !self.pending.values().any(|p| p.client == client) {
                self.disconnected.remove(&client);
            }
        }
        self.plans.reclaim();
        n
    }

    /// Drains the RT's replies, delivering each with `deliver(client, reply)` (false: the
    /// client is gone, the reply is abandoned). Releases reservations and reclaims retired
    /// plan slots. Returns the number of replies drained.
    pub fn poll(&mut self, mut deliver: impl FnMut(ClientId, Reply) -> bool) -> usize {
        self.try_poll(|client, reply| {
            if deliver(client, reply) {
                DeliveryDecision::Delivered
            } else {
                DeliveryDecision::Abandoned
            }
        })
    }
}

impl<P> RtIntake<P> {
    /// Starts release `release` (the executor's release count, strictly increasing): allows
    /// one [`Self::take`] in it. False, and nothing changes, when `release` does not advance:
    /// calling it again within a release does not grant a second command.
    pub fn begin_release(&mut self, release: u64) -> bool {
        if release <= self.release { return false; }
        self.release = release;
        true
    }

    /// The current release (0 before the first [`Self::begin_release`]).
    pub fn release(&self) -> u64 { self.release }

    /// The next ordinary command, at most one per release (None after the first, and before
    /// the first [`Self::begin_release`]).
    pub fn take(&mut self) -> Option<Taken<'_, P>> {
        if self.release == 0 || self.taken_in == self.release { return None; }
        let d = self.commands.pop().ok()?;
        self.taken_in = self.release;
        Some(Taken { intake: self, d, answered: false })
    }

    /// Commands waiting.
    pub fn queued(&self) -> usize { self.commands.slots() }

    pub fn arm(&self) -> ArmId { self.arm }
    pub fn boot(&self) -> BootId { self.boot }

    /// Acknowledged installs whose terminal reply is still owed.
    pub fn installed(&self) -> usize { self.installed }

    /// The plan of a live install, None when stale.
    pub fn plan(&self, h: PlanHandle) -> Option<&P> { self.plans.get(h) }

    /// Replies the reply ring refused (0 unless a reservation is wrong).
    pub fn reply_overflow(&self) -> u64 { self.reply_overflow }

    fn reply(&mut self, r: Reply) {
        if self.replies.push(r).is_err() { self.reply_overflow += 1; }
    }

    /// The physical end of an acknowledged install (consumes its token: one terminal per
    /// install); retires its plan slot.
    pub fn terminal(&mut self, i: Installed, outcome: Outcome) { self.terminal_with_children(i, outcome, 0); }

    /// [`Self::terminal`] of a macro action that ran `children` child primitives.
    pub fn terminal_with_children(&mut self, i: Installed, outcome: Outcome, children: u32) {
        if let Some(plan) = i.plan { self.plans.retire(plan); }
        self.installed -= 1;
        self.reply(Reply::Terminal { request: i.request, action: i.action, outcome, children });
    }

    /// Retires a standalone or macro-child plan slot handle directly (zero alloc/free).
    pub fn retire_plan(&mut self, plan: PlanHandle) -> bool { self.plans.retire(plan) }

    /// Emits a deferred reply previously built by [`Taken::ack_deferred`] (used by
    /// `ReleaseEngine` so supersession terminals and their accompanying ACK are both
    /// delivered strictly after Step 6 publication while keeping `Terminal(old)` before `Ack(new)`).
    pub fn emit_reply(&mut self, reply: Reply) { self.reply(reply); }

    /// The broker is gone (dropped): the RT keeps running and drops replies.
    pub fn broker_gone(&self) -> bool { self.commands.is_abandoned() }
}

impl<P: Sync> RtIntake<P> {
    /// Pins a live plan slot for immutable borrowing until terminal retirement.
    pub fn plan_ref(&self, h: PlanHandle) -> Option<PlanSlotRef<P>> { self.plans.pin(h) }
}

impl<P> Taken<'_, P> {
    pub fn descriptor(&self) -> &Descriptor { &self.d }

    /// The plan of this install (None: not an install, or the slot is stale).
    pub fn plan(&self) -> Option<&P> {
        self.d.kind.plan().and_then(|plan| self.intake.plans.get(plan))
    }

    /// Sends the terminal of an earlier install before this command's answer (an install
    /// that supersedes the active one).
    pub fn terminal_before(&mut self, i: Installed, outcome: Outcome) { self.intake.terminal(i, outcome); }

    /// Acknowledges the commit proven by `committed`, which must have been made for this
    /// request: a proof of another request refuses the command (`COMMIT_MISMATCH`) instead.
    /// An install or transition acknowledged without an action is refused too
    /// (`COMMIT_MISMATCH`): its terminal could never be sent. Returns the commit sequence and the install's token.
    pub fn ack(mut self, committed: super::ops::Committed) -> Result<Acked, &'static str> {
        let terminal = self.d.kind.starts_action();
        if committed.request() != self.d.stamp.request || (terminal && committed.action().is_none()) {
            self.refuse_now("COMMIT_MISMATCH");
            return Err("COMMIT_MISMATCH");
        }
        self.answered = true;
        let intake = &mut *self.intake;
        intake.commit_seq += 1;
        let commit_seq = intake.commit_seq;
        let request = self.d.stamp.request;
        intake.reply(Reply::Ack { request, commit_seq, action: committed.action() });
        let installed = match (terminal, committed.action()) {
            (true, Some(action)) => { intake.installed += 1; Some(Installed { request, plan: self.d.kind.plan(), action }) }
            _ => {
                if let Some(plan) = self.d.kind.plan() { intake.plans.retire(plan); }
                None
            }
        };
        Ok(Acked { commit_seq, installed })
    }

    /// Validates and records the commit like [`Self::ack`], returning the `Reply::Ack` to be
    /// emitted later via [`RtIntake::emit_reply`] (after publication + any supersession terminal).
    pub fn ack_deferred(mut self, committed: super::ops::Committed) -> Result<(Acked, Reply), &'static str> {
        let terminal = self.d.kind.starts_action();
        if committed.request() != self.d.stamp.request || (terminal && committed.action().is_none()) {
            self.refuse_now("COMMIT_MISMATCH");
            return Err("COMMIT_MISMATCH");
        }
        self.answered = true;
        let intake = &mut *self.intake;
        intake.commit_seq += 1;
        let commit_seq = intake.commit_seq;
        let request = self.d.stamp.request;
        let reply = Reply::Ack { request, commit_seq, action: committed.action() };
        let installed = match (terminal, committed.action()) {
            (true, Some(action)) => { intake.installed += 1; Some(Installed { request, plan: self.d.kind.plan(), action }) }
            _ => {
                if let Some(plan) = self.d.kind.plan() { intake.plans.retire(plan); }
                None
            }
        };
        Ok((Acked { commit_seq, installed }, reply))
    }

    /// Records a lease commit without emitting the reply yet (emitted in Step 7 strictly after
    /// Step 6 `StateHub` publication via [`RtIntake::emit_reply`]).
    pub fn ack_lease_deferred(
        mut self,
        outcome: super::requests::lease::LeaseOutcome,
    ) -> (u64, Reply) {
        self.answered = true;
        let intake = &mut *self.intake;
        intake.commit_seq += 1;
        let commit_seq = intake.commit_seq;
        let request = self.d.stamp.request;
        (commit_seq, Reply::Lease { request, commit_seq, outcome })
    }

    /// Refuses the command without emitting the reply yet (retires any plan slot now; the
    /// `Reply::Refused` is emitted in Step 7 strictly after Step 6 publication).
    pub fn refuse_deferred(mut self, code: &'static str) -> Reply {
        self.answered = true;
        if let Some(plan) = self.d.kind.plan() {
            self.intake.plans.retire(plan);
        }
        Reply::Refused {
            request: self.d.stamp.request,
            code,
        }
    }

    /// Refuses the command with `code`; retires its plan slot.
    pub fn refuse(mut self, code: &'static str) { self.refuse_now(code); }

    fn refuse_now(&mut self, code: &'static str) {
        self.answered = true;
        if let Some(plan) = self.d.kind.plan() { self.intake.plans.retire(plan); }
        let request = self.d.stamp.request;
        self.intake.reply(Reply::Refused { request, code });
    }
}

impl<P: Sync> Taken<'_, P> {
    /// Pins this install's plan slot for immutable borrowing until terminal retirement.
    pub fn plan_ref(&self) -> Option<PlanSlotRef<P>> {
        self.d.kind.plan().and_then(|plan| self.intake.plans.pin(plan))
    }
}

impl<P> Drop for Taken<'_, P> {
    fn drop(&mut self) {
        if !self.answered { self.refuse_now("INTERNAL"); }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn tiny(commands: usize, plans: usize, replies: usize) -> (Broker<[f32; 4]>, RtIntake<[f32; 4]>) {
        channel(ArmId::LOCAL, BootId(7), Capacities { commands, plans, replies }, || [0.0; 4])
    }

    fn later() -> Instant { Instant::now() + Duration::from_secs(5) }

    fn plan(v: f32) -> impl Fn(&mut [f32; 4]) { move |p| *p = [v; 4] }

    // Compile-time: `PlanReader<P>` (and so `RtIntake<P>`) is never `Sync`, even for
    // `P: Sync`; `<T as NotSync<_>>::CHECK` is ambiguous, and fails to compile, when `T: Sync`.
    trait NotSync<A> { const CHECK: () = (); }
    impl<T: ?Sized> NotSync<()> for T {}
    #[allow(dead_code)]
    struct IsSync;
    impl<T: ?Sized + Sync> NotSync<IsSync> for T {}
    const _: () = <PlanReader<std::cell::Cell<u32>> as NotSync<_>>::CHECK;
    const _: () = <PlanReader<u32> as NotSync<_>>::CHECK;
    const _: () = <RtIntake<u32> as NotSync<_>>::CHECK;
    fn _reader_is_send<T: Send>() { fn send<S: Send>() {} send::<PlanReader<T>>(); }

    #[test]
    fn authority_rechecks_in_order() {
        let a = Authority { lease_generation: 3, epoch: 2, stop_generation: 1, config_revision: 9, parent_generation: 0 };
        assert_eq!(a.admits(&a), None);
        assert_eq!(a.admits(&Authority { stop_generation: 2, epoch: 5, ..a }), Some("STOPPED"));
        assert_eq!(a.admits(&Authority { epoch: 5, ..a }), Some("EPOCH_MISMATCH"));
        assert_eq!(a.admits(&Authority { lease_generation: 4, ..a }), Some("LEASE_LOST"));
        assert_eq!(a.admits(&Authority { config_revision: 10, ..a }), Some("PLAN_STALE"));
        assert_eq!(a.admits(&Authority { parent_generation: 1, ..a }), None, "not a child: the parent is not compared");
        let child = Authority { parent_generation: 7, ..a };
        assert_eq!(child.admits(&Authority { parent_generation: 7, ..a }), None);
        assert_eq!(child.admits(&Authority { parent_generation: 8, ..a }), Some("PARENT_REPLACED"));
        assert_eq!(child.admits(&a), Some("PARENT_REPLACED"), "the parent ended");
        let s = Stamp { arm: ArmId::LOCAL, boot: BootId(1), request: RequestId(1), client: ClientId(1), client_seq: 1, authority: a, deadline: Instant::now() };
        assert_eq!(s.check(ArmId::LOCAL, BootId(2), Instant::now(), &a), Some("WRONG_TARGET"));
        assert_eq!(s.check(ArmId::LOCAL, BootId(0), Instant::now(), &a), Some("WRONG_TARGET"));
        let zero = Stamp { boot: BootId(0), ..s };
        assert_eq!(zero.check(ArmId::LOCAL, BootId(0), Instant::now(), &a), Some("WRONG_TARGET"), "zero boot id refused fail-closed");
        assert!(!BootId::process().is_zero());
        assert_ne!(BootId::fresh(), BootId::fresh());
        assert_eq!(s.check(ArmId::LOCAL, BootId(1), s.deadline + Duration::from_millis(1), &a), Some("DEADLINE_EXPIRED"));
        assert_eq!(s.check(ArmId::LOCAL, BootId(1), s.deadline, &a), None);
    }

    #[test]
    fn one_ordinary_command_per_release() {
        let (mut b, mut rt) = tiny(4, 2, 8);
        for k in 0..3 { b.submit(ClientId(1), k, Authority::default(), later(), Submit::Immediate(ImmediateOp::Heartbeat)).unwrap(); }
        assert!(rt.take().is_none(), "no release started");
        assert!(rt.begin_release(1));
        rt.take().unwrap().refuse("X");
        assert!(rt.take().is_none(), "a second ordinary command in the same release");
        // the boundary is the intake's: restarting the same (or an older) release grants nothing
        assert!(!rt.begin_release(1));
        assert!(!rt.begin_release(0));
        assert!(rt.take().is_none(), "begin_release of the same release reset the allowance");
        assert!(rt.begin_release(2));
        let t = rt.take().unwrap();
        assert_eq!(t.descriptor().stamp.client_seq, 1);
        t.refuse("X");
        assert_eq!(rt.queued(), 1);
    }

    #[test]
    fn a_full_command_ring_refuses_without_side_effects() {
        let (mut b, mut rt) = tiny(2, 4, 16);
        let auth = Authority::default();
        b.submit(ClientId(1), 0, auth, later(), Submit::Install(&plan(1.0))).unwrap();
        b.submit(ClientId(1), 1, auth, later(), Submit::Immediate(ImmediateOp::Heartbeat)).unwrap();
        // stalled consumer: the RT takes nothing
        let free_before = b.plans.free();
        assert_eq!(b.submit(ClientId(1), 2, auth, later(), Submit::Install(&plan(2.0))), Err(AdmitError::CommandsFull));
        assert_eq!(b.plans.free(), free_before, "a refused admission keeps no plan slot");
        assert_eq!((b.reserved(), b.outstanding()), (3, 2));
        assert_eq!(b.stats().refused_admission, 1);
        // the queued commands are intact and in order
        rt.begin_release(1);
        let t = rt.take().unwrap();
        assert!(matches!(t.descriptor().kind, CommandKind::Install { .. }), "install first");
        assert_eq!(t.plan(), Some(&[1.0; 4]));
        t.refuse("X");
    }

    #[test]
    fn plan_slots_saturate_and_reply_reservations_bound_admission() {
        let (mut b, _rt) = tiny(8, 2, 8);
        let auth = Authority::default();
        b.submit(ClientId(1), 0, auth, later(), Submit::Install(&plan(1.0))).unwrap();
        b.submit(ClientId(1), 1, auth, later(), Submit::Install(&plan(2.0))).unwrap();
        assert_eq!(b.submit(ClientId(1), 2, auth, later(), Submit::Install(&plan(3.0))), Err(AdmitError::PlansFull));
        // replies: 2 installs reserve 4; capacity 8 leaves 4 immediate commands
        for k in 0..4 { b.submit(ClientId(1), 10 + k, auth, later(), Submit::Immediate(ImmediateOp::Heartbeat)).unwrap(); }
        assert_eq!(b.submit(ClientId(1), 20, auth, later(), Submit::Immediate(ImmediateOp::Heartbeat)), Err(AdmitError::RepliesFull));
        assert_eq!(b.reserved(), 8);
    }

    #[test]
    fn wraparound_keeps_ids_generations_and_retirement_counts() {
        let (mut b, mut rt) = tiny(2, 1, 2);
        let auth = Authority::default();
        let mut gens = Vec::new();
        for k in 0..100u64 {
            let r = b.submit(ClientId(1), k, auth, later(), Submit::Install(&plan(k as f32))).unwrap();
            assert_eq!(r, RequestId(k + 1));
            rt.begin_release(k + 1);
            let t = rt.take().unwrap();
            let CommandKind::Install { plan: h } = t.descriptor().kind else { panic!() };
            assert_eq!(t.plan(), Some(&[k as f32; 4]));
            gens.push(h.generation());
            let acked = t.ack(super::super::ops::Committed::for_tests(r, Some(ActionId(k)))).unwrap();
            assert_eq!(acked.commit_seq, k + 1);
            rt.terminal(acked.installed.unwrap(), Outcome::Completed);
            let mut got = Vec::new();
            assert_eq!(b.poll(|_, r| { got.push(r); true }), 2);
            assert_eq!(got, vec![Reply::Ack { request: r, commit_seq: k + 1, action: Some(ActionId(k)) }, Reply::Terminal { request: r, action: ActionId(k), outcome: Outcome::Completed, children: 0 }]);
        }
        assert_eq!(gens, (1..=100).collect::<Vec<u64>>(), "one slot, a new generation per reuse");
        let s = b.stats();
        assert_eq!((s.admitted, s.acked, s.terminals, s.retired, s.abandoned), (100, 100, 100, 100, 0));
        assert_eq!((b.reserved(), b.outstanding(), rt.reply_overflow()), (0, 0, 0));
    }

    #[test]
    fn a_reused_slot_reads_stale_through_an_old_handle() {
        let (mut w, mut r) = plan_slots(1, || [0.0f32; 4]);
        let h1 = w.stage(|p| *p = [1.0; 4]).unwrap();
        assert!(r.retire(h1));
        assert!(!r.retire(h1), "retired once");
        let h2 = w.stage(|p| *p = [2.0; 4]).unwrap();
        assert_eq!((h1.index(), h2.generation()), (h2.index(), h1.generation() + 1));
        assert_eq!(r.get(h1), None);
        assert_eq!(r.get(h2), Some(&[2.0; 4]));
        assert!(w.stage(|_| {}).is_none(), "the only slot is live");
    }

    #[test]
    fn a_handle_of_another_pool_neither_reads_nor_retires() {
        let (mut wa, mut ra) = plan_slots(1, || 0u32);
        let (mut wb, mut rb) = plan_slots(1, || 0u32);
        let ha = wa.stage(|p| *p = 1).unwrap();
        let hb = wb.stage(|p| *p = 2).unwrap();
        // same index and generation, different pools
        assert_eq!((ha.index(), ha.generation()), (hb.index(), hb.generation()));
        assert_ne!(ha.pool(), hb.pool());
        assert_eq!(rb.get(ha), None, "a foreign handle reads nothing");
        assert!(!rb.retire(ha), "a foreign handle retires nothing");
        assert_eq!(rb.get(hb), Some(&2), "the pool's own plan is intact");
        wb.unpublish(ha);
        assert_eq!(rb.get(hb), Some(&2), "a foreign handle unpublishes nothing");
        assert_eq!(ra.get(ha), Some(&1));
        assert!(ra.retire(ha) && rb.retire(hb));
    }

    #[test]
    fn a_slot_generation_never_wraps() {
        let (mut w, mut r) = plan_slots(2, || 0u32);
        w.force_generation(0, MAX_PLAN_GENERATION - 1);
        let last = w.stage(|p| *p = 1).unwrap();
        assert_eq!((last.index(), last.generation()), (0, MAX_PLAN_GENERATION));
        assert!(r.retire(last));
        // slot 0 is exhausted: the next stage takes slot 1, then nothing
        let h = w.stage(|p| *p = 2).unwrap();
        assert_eq!((h.index(), h.generation()), (1, 1));
        assert!(w.stage(|_| {}).is_none(), "an exhausted slot is not reused");
        assert_eq!(r.get(last), None, "the old handle stays stale");
        assert_eq!(r.get(h), Some(&2));
    }

    #[test]
    fn abandoned_and_disconnected_clients_release_their_reservations() {
        let (mut b, mut rt) = tiny(4, 2, 8);
        let auth = Authority::default();
        let r1 = b.submit(ClientId(1), 0, auth, later(), Submit::Install(&plan(1.0))).unwrap();
        b.submit(ClientId(2), 0, auth, later(), Submit::Immediate(ImmediateOp::Heartbeat)).unwrap();
        b.disconnect(ClientId(1));
        assert_eq!(b.submit(ClientId(1), 1, auth, later(), Submit::Immediate(ImmediateOp::Heartbeat)), Err(AdmitError::Disconnected));
        rt.begin_release(1);
        let t1 = rt.take().unwrap();
        let i1 = t1.ack(super::super::ops::Committed::for_tests(r1, Some(ActionId(5)))).unwrap().installed.unwrap();
        rt.begin_release(2);
        rt.take().unwrap().refuse("NOT_ARMED");
        // client 2's receiver is gone too: deliver says so
        let mut delivered = Vec::new();
        b.poll(|c, r| if c == ClientId(2) { false } else { delivered.push(r); true });
        assert!(delivered.is_empty(), "nothing reaches a disconnected client");
        assert_eq!(b.stats().abandoned, 2);
        assert_eq!(b.reserved(), 1, "the install's terminal is still reserved");
        rt.terminal(i1, Outcome::Stopped);
        b.poll(|_, _| true);
        assert_eq!((b.reserved(), b.outstanding(), b.stats().abandoned), (0, 0, 3));
        assert_eq!(r1, RequestId(1));
        // the client id can be used again once nothing of it is pending
        assert!(b.submit(ClientId(1), 2, auth, later(), Submit::Immediate(ImmediateOp::Heartbeat)).is_ok());
    }

    #[test]
    fn a_refused_install_retires_its_slot_and_its_terminal_reservation() {
        let (mut b, mut rt) = tiny(4, 1, 4);
        b.submit(ClientId(1), 0, Authority::default(), later(), Submit::Install(&plan(1.0))).unwrap();
        assert_eq!(b.reserved(), 2);
        rt.begin_release(1);
        rt.take().unwrap().refuse("STOPPED");
        assert_eq!(b.poll(|_, r| { assert_eq!(r, Reply::Refused { request: RequestId(1), code: "STOPPED" }); true }), 1);
        assert_eq!((b.reserved(), b.stats().retired), (0, 1));
        assert!(b.submit(ClientId(1), 1, Authority::default(), later(), Submit::Install(&plan(2.0))).is_ok(), "slot reused");
    }

    #[test]
    fn a_commit_proof_acks_only_its_own_request_and_only_once() {
        let (mut b, mut rt) = tiny(4, 2, 8);
        let auth = Authority::default();
        let r1 = b.submit(ClientId(1), 0, auth, later(), Submit::Install(&plan(1.0))).unwrap();
        let r2 = b.submit(ClientId(1), 1, auth, later(), Submit::Immediate(ImmediateOp::Heartbeat)).unwrap();
        rt.begin_release(1);
        // a proof made for request 2 cannot acknowledge request 1
        let t = rt.take().unwrap();
        assert_eq!(t.ack(super::super::ops::Committed::for_tests(r2, None)), Err("COMMIT_MISMATCH"));
        // an install acknowledged without an action could never send its terminal
        rt.begin_release(2);
        let t = rt.take().unwrap();
        assert_eq!(t.ack(super::super::ops::Committed::for_tests(r2, Some(ActionId(1)))).unwrap().installed, None);
        let mut got = Vec::new();
        b.poll(|_, r| { got.push(r); true });
        assert_eq!(got, vec![Reply::Refused { request: r1, code: "COMMIT_MISMATCH" }, Reply::Ack { request: r2, commit_seq: 1, action: Some(ActionId(1)) }]);
        assert_eq!((b.reserved(), b.outstanding(), b.stats().unexpected, rt.installed()), (0, 0, 0, 0));
    }

    #[test]
    fn an_unanswered_command_is_refused_and_reservations_match_replies_exactly() {
        let (mut b, mut rt) = tiny(4, 2, 4);
        let auth = Authority::default();
        let r1 = b.submit(ClientId(1), 0, auth, later(), Submit::Install(&plan(1.0))).unwrap();
        let r2 = b.submit(ClientId(1), 1, auth, later(), Submit::Install(&plan(2.0))).unwrap();
        assert_eq!(b.reserved(), 4, "ack + terminal per install");
        rt.begin_release(1);
        drop(rt.take().unwrap());
        rt.begin_release(2);
        let i2 = rt.take().unwrap().ack(super::super::ops::Committed::for_tests(r2, Some(ActionId(9)))).unwrap().installed.unwrap();
        // the ring holds exactly the reserved replies: never more than were reserved
        rt.terminal(i2, Outcome::Completed);
        let mut got = Vec::new();
        b.poll(|_, r| { got.push(r); true });
        assert_eq!(got, vec![
            Reply::Refused { request: r1, code: "INTERNAL" },
            Reply::Ack { request: r2, commit_seq: 1, action: Some(ActionId(9)) },
            Reply::Terminal { request: r2, action: ActionId(9), outcome: Outcome::Completed, children: 0 },
        ]);
        assert_eq!((b.reserved(), b.outstanding(), b.stats().unexpected, rt.reply_overflow()), (0, 0, 0, 0));
        assert_eq!(b.stats().retired, 2);
    }

    /// Writer and reader on two threads: a plan read through a live handle is always the
    /// complete plan staged under that handle (stress in place of a model checker).
    #[test]
    fn plan_slots_stress_two_threads() {
        let (mut w, mut r) = plan_slots(3, || vec![0u64; 64]);
        let (mut htx, mut hrx) = rtrb::RingBuffer::<(PlanHandle, u64)>::new(2);
        let reader = std::thread::spawn(move || {
            let mut seen = 0u64;
            while seen < 20_000 {
                if let Ok((h, v)) = hrx.pop() {
                    let p = r.get(h).expect("a queued handle is live");
                    assert!(p.iter().all(|x| *x == v), "torn plan");
                    assert!(r.retire(h));
                    seen += 1;
                } else if hrx.is_abandoned() { break; } else { std::thread::yield_now(); }
            }
            seen
        });
        let mut v = 0u64;
        while v < 20_000 {
            let Some(h) = w.stage(|p| p.iter_mut().for_each(|x| *x = v)) else { std::thread::yield_now(); continue; };
            while htx.push((h, v)).is_err() { std::thread::yield_now(); }
            v += 1;
        }
        assert_eq!(reader.join().unwrap(), 20_000);
        w.reclaim();
        assert_eq!(w.retired_total(), 20_000);
    }

    #[test]
    fn pinned_slot_cannot_be_reclaimed_until_borrow_drops() {
        let (mut w, mut r) = plan_slots(1, || [0.0f32; 4]);
        let h1 = w.stage(|p| *p = [42.0; 4]).unwrap();
        let pinned = r.pin(h1).expect("live slot pins");
        let cloned = pinned.clone();
        assert_eq!(*pinned, [42.0; 4]);
        assert_eq!(pinned.handle(), h1);
        // retiring prevents new pins / gets, but the writer cannot reclaim while borrows exist
        assert!(r.retire(h1));
        assert!(r.get(h1).is_none());
        assert!(r.pin(h1).is_none());
        assert_eq!(w.reclaim(), 0, "pinned slot must not be reclaimed");
        assert!(w.stage(|p| *p = [99.0; 4]).is_none(), "slot still held by active borrow");
        assert_eq!(*cloned, [42.0; 4]);
        drop(pinned);
        assert_eq!(w.reclaim(), 0, "second borrow still holds pin");
        drop(cloned);
        assert_eq!(w.reclaim(), 1, "reclaimed once all borrows dropped");
        let h2 = w.stage(|p| *p = [99.0; 4]).unwrap();
        assert_eq!(r.get(h2), Some(&[99.0; 4]));
        assert!(r.retire(h2));
    }

    #[test]
    fn eight_plan_slots_exhaust_and_recover_on_retire() {
        let (mut w, mut r) = plan_slots(PLAN_SLOTS, || 0u64);
        assert_eq!((w.capacity(), r.capacity()), (8, 8));
        let mut handles = Vec::new();
        for k in 0..PLAN_SLOTS {
            handles.push(w.stage(|p| *p = k as u64).expect("slot available"));
        }
        assert_eq!(w.free(), 0);
        assert!(w.stage(|_| {}).is_none(), "all 8 slots exhausted");
        assert!(r.retire(handles[3]));
        assert_eq!(w.reclaim(), 1);
        let h = w.stage(|p| *p = 999).expect("reclaimed slot reusable");
        assert_eq!(r.get(h), Some(&999));
        for h in handles.into_iter().enumerate().filter(|(i, _)| *i != 3).map(|(_, h)| h) {
            assert!(r.retire(h));
        }
        assert!(r.retire(h));
        assert_eq!(w.reclaim(), PLAN_SLOTS);
    }
}
