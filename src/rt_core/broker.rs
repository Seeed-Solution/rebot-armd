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
use std::sync::atomic::{AtomicU64, Ordering};
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

/// Identity of one armd process run (changes on every start).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BootId(pub u64);

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
    /// Parent action generation for a child primitive (0: none).
    pub parent_generation: u64,
}

impl Authority {
    /// The rejection code when a command admitted under `self` may not commit under
    /// `current`, None when it may. Checked in this order: stop, epoch, lease, config, parent.
    pub fn admits(&self, current: &Authority) -> Option<&'static str> {
        if self.stop_generation != current.stop_generation { return Some("STOPPED"); }
        if self.epoch != current.epoch { return Some("EPOCH_MISMATCH"); }
        if self.lease_generation != current.lease_generation { return Some("LEASE_LOST"); }
        if self.config_revision != current.config_revision { return Some("PLAN_STALE"); }
        if self.parent_generation != current.parent_generation { return Some("PARENT_REPLACED"); }
        None
    }
}

/// Who sent a command and under which authority, until when.
#[derive(Debug, Clone, Copy)]
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
    pub fn check(&self, arm: ArmId, boot: BootId, now: Instant, current: &Authority) -> Option<&'static str> {
        if self.arm != arm || self.boot != boot { return Some("WRONG_TARGET"); }
        if now > self.deadline { return Some("DEADLINE_EXPIRED"); }
        self.authority.admits(current)
    }
}

/// A handle of one published plan slot: slot index and the generation it was staged in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PlanHandle {
    index: u8,
    generation: u32,
}

impl PlanHandle {
    pub fn index(&self) -> usize { usize::from(self.index) }
    pub fn generation(&self) -> u32 { self.generation }
}

/// Commands that do not need a plan slot. Tracks extend this list through the integrator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImmediateOp {
    Heartbeat,
    TorqueOn,
    TorqueOff,
}

/// What a descriptor asks the RT to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandKind {
    /// Install the primitive of a plan slot; starts an action (a terminal reply follows the ACK).
    Install { plan: PlanHandle },
    /// A command without a plan; ACK or refusal only.
    Immediate(ImmediateOp),
}

impl CommandKind {
    pub fn starts_action(&self) -> bool { matches!(self, Self::Install { .. }) }
}

/// One entry of the command ring.
#[derive(Debug, Clone, Copy)]
pub struct Descriptor {
    pub stamp: Stamp,
    pub kind: CommandKind,
}

/// How an action ended physically.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Outcome {
    Completed,
    /// Ended with a failure code (the action's `code`).
    Failed(&'static str),
    /// Replaced by a later commit before it completed.
    Superseded,
    Stopped,
}

/// RT -> broker.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reply {
    /// Committed: `commit_seq` is the RT commit sequence, `action` the committed action.
    Ack { request: RequestId, commit_seq: u64, action: Option<ActionId> },
    /// Not committed (nothing changed on the RT side; a plan slot was retired).
    Refused { request: RequestId, code: &'static str },
    /// The physical completion of an acknowledged action.
    Terminal { request: RequestId, action: ActionId, outcome: Outcome },
}

impl Reply {
    pub fn request(&self) -> RequestId {
        match self { Self::Ack { request, .. } | Self::Refused { request, .. } | Self::Terminal { request, .. } => *request }
    }
}

// ---------------------------------------------------------------------------------------
// Plan slots

const FREE: u64 = 0;
const WRITING: u64 = 1;
const READY: u64 = 2;
const RETIRED: u64 = 3;

fn pack(generation: u32, state: u64) -> u64 { (u64::from(generation) << 2) | state }
fn unpack(word: u64) -> (u32, u64) { ((word >> 2) as u32, word & 3) }

/// `PLAN_SLOTS` (or a test capacity) plan slots shared by one writer and one reader.
pub struct PlanSlots<P> {
    words: Box<[AtomicU64]>,
    plans: Box<[UnsafeCell<P>]>,
}

// SAFETY: a slot's plan is written only by the single `PlanWriter` while its state is
// WRITING (no handle to it exists yet), and read only through `PlanReader::get` while the
// state is READY with the handle's generation (Acquire load after the writer's Release
// store); the writer does not touch a READY slot until the reader moved it to RETIRED
// (Release) and the writer saw that (Acquire). `P: Send` moves between the two threads.
// Only one thread reads: `PlanReader` is neither `Clone` nor `Sync`, so `&P` never reaches
// two threads at once through it (which `P: Send` alone would not make sound).
unsafe impl<P: Send> Sync for PlanSlots<P> {}

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

/// `capacity` pre-touched plan slots, each built (allocated and written) by `make` now.
pub fn plan_slots<P: Send>(capacity: usize, mut make: impl FnMut() -> P) -> (PlanWriter<P>, PlanReader<P>) {
    assert!(capacity > 0 && capacity <= usize::from(u8::MAX));
    let slots = Arc::new(PlanSlots {
        words: (0..capacity).map(|_| AtomicU64::new(pack(0, FREE))).collect(),
        plans: (0..capacity).map(|_| UnsafeCell::new(make())).collect(),
    });
    (PlanWriter { slots: slots.clone(), retired: 0 }, PlanReader { slots, _not_sync: PhantomData })
}

impl<P> PlanWriter<P> {
    /// Moves retired slots back to free; returns how many.
    pub fn reclaim(&mut self) -> usize {
        let mut n = 0;
        for w in self.slots.words.iter() {
            let (g, s) = unpack(w.load(Ordering::Acquire));
            if s == RETIRED {
                w.store(pack(g, FREE), Ordering::Relaxed);
                n += 1;
            }
        }
        self.retired += n as u64;
        n
    }

    /// Slots reclaimed so far.
    pub fn retired_total(&self) -> u64 { self.retired }

    /// Free slots (after a reclaim).
    pub fn free(&self) -> usize {
        self.slots.words.iter().filter(|w| unpack(w.load(Ordering::Acquire)).1 == FREE).count()
    }

    /// Writes a plan into a free slot (in place, reusing its memory) and publishes it
    /// immutable. None when every slot is taken.
    pub fn stage(&mut self, write: impl FnOnce(&mut P)) -> Option<PlanHandle> {
        self.reclaim();
        let i = self.slots.words.iter().position(|w| unpack(w.load(Ordering::Acquire)).1 == FREE)?;
        let (g, _) = unpack(self.slots.words[i].load(Ordering::Relaxed));
        let generation = g.wrapping_add(1);
        self.slots.words[i].store(pack(generation, WRITING), Ordering::Relaxed);
        // SAFETY: WRITING and no handle exists: only this writer accesses the plan.
        write(unsafe { &mut *self.slots.plans[i].get() });
        self.slots.words[i].store(pack(generation, READY), Ordering::Release);
        Some(PlanHandle { index: i as u8, generation })
    }

    /// Takes back a published slot whose descriptor was never queued (admission failed).
    fn unpublish(&mut self, h: PlanHandle) {
        let w = &self.slots.words[h.index()];
        if w.load(Ordering::Acquire) == pack(h.generation, READY) { w.store(pack(h.generation, FREE), Ordering::Relaxed); }
    }
}

impl<P> PlanReader<P> {
    /// The plan of `h`, None when the slot was retired or reused since (stale handle).
    pub fn get(&self, h: PlanHandle) -> Option<&P> {
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
        let Some(w) = self.slots.words.get(h.index()) else { return false; };
        w.compare_exchange(pack(h.generation, READY), pack(h.generation, RETIRED), Ordering::Release, Ordering::Relaxed).is_ok()
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
}

struct Pending {
    client: ClientId,
    /// ACK / refusal still to come.
    ack: bool,
    /// Terminal still to come (reserved).
    terminal: bool,
}

/// What a command asks for, with the plan to stage for an install.
pub enum Submit<'a, P> {
    Install(&'a dyn Fn(&mut P)),
    Immediate(ImmediateOp),
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
    plans: PlanWriter<P>,
    reply_capacity: usize,
    reserved: usize,
    pending: HashMap<RequestId, Pending>,
    disconnected: std::collections::HashSet<ClientId>,
    stats: BrokerStats,
}

/// The RT side: one ordinary command per release, replies without waiting.
pub struct RtIntake<P> {
    arm: ArmId,
    boot: BootId,
    commands: rtrb::Consumer<Descriptor>,
    replies: rtrb::Producer<Reply>,
    plans: PlanReader<P>,
    commit_seq: u64,
    taken: bool,
    /// Replies the reply ring refused (a reservation bug; must stay 0).
    reply_overflow: u64,
}

/// A broker and its RT intake with `caps`; plan slots built by `make` (pre-touched).
pub fn channel<P: Send>(arm: ArmId, boot: BootId, caps: Capacities, make: impl FnMut() -> P) -> (Broker<P>, RtIntake<P>) {
    let (ctx, crx) = rtrb::RingBuffer::new(caps.commands);
    let (rtx, rrx) = rtrb::RingBuffer::new(caps.replies);
    let (pw, pr) = plan_slots(caps.plans, make);
    (
        Broker { arm, boot, next_request: 0, commands: ctx, replies: rrx, plans: pw, reply_capacity: caps.replies, reserved: 0,
            pending: HashMap::new(), disconnected: Default::default(), stats: BrokerStats::default() },
        RtIntake { arm, boot, commands: crx, replies: rtx, plans: pr, commit_seq: 0, taken: false, reply_overflow: 0 },
    )
}

impl<P> Broker<P> {
    pub fn stats(&self) -> BrokerStats { BrokerStats { retired: self.plans.retired_total(), ..self.stats } }

    /// Reply slots currently reserved.
    pub fn reserved(&self) -> usize { self.reserved }

    /// Requests waiting for an ACK or a terminal.
    pub fn outstanding(&self) -> usize { self.pending.len() }

    /// The client is gone: its later replies are dropped (counted as abandoned); commands it
    /// already queued stay queued (lease loss is what invalidates them).
    pub fn disconnect(&mut self, client: ClientId) {
        if self.pending.values().any(|p| p.client == client) { self.disconnected.insert(client); }
    }

    /// Admits one command: reserves its replies, stages its plan, queues its descriptor.
    pub fn submit(&mut self, client: ClientId, client_seq: u64, authority: Authority, deadline: Instant, what: Submit<'_, P>) -> Result<RequestId, AdmitError> {
        if self.disconnected.contains(&client) {
            self.stats.refused_admission += 1;
            return Err(AdmitError::Disconnected);
        }
        let terminal = matches!(what, Submit::Install(_));
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
            Submit::Immediate(op) => CommandKind::Immediate(op),
        };
        self.next_request += 1;
        let request = RequestId(self.next_request);
        let d = Descriptor { stamp: Stamp { arm: self.arm, boot: self.boot, request, client, client_seq, authority, deadline }, kind };
        if self.commands.push(d).is_err() {
            // single producer and slots() > 0 above: unreachable; keep it side-effect free
            if let CommandKind::Install { plan } = kind { self.plans.unpublish(plan); }
            self.stats.refused_admission += 1;
            return Err(AdmitError::CommandsFull);
        }
        self.reserved += need;
        self.pending.insert(request, Pending { client, ack: true, terminal });
        self.stats.admitted += 1;
        Ok(request)
    }

    /// Drains the RT's replies, delivering each with `deliver(client, reply)` (false: the
    /// client is gone, the reply is abandoned). Releases reservations and reclaims retired
    /// plan slots. Returns the number of replies drained.
    pub fn poll(&mut self, mut deliver: impl FnMut(ClientId, Reply) -> bool) -> usize {
        let mut n = 0;
        while let Ok(reply) = self.replies.pop() {
            n += 1;
            let request = reply.request();
            let Some(p) = self.pending.get_mut(&request) else { continue; };
            match reply {
                Reply::Ack { .. } => {
                    self.stats.acked += 1;
                    if p.ack { p.ack = false; self.reserved -= 1; }
                }
                Reply::Refused { .. } => {
                    self.stats.refused_by_rt += 1;
                    if p.ack { p.ack = false; self.reserved -= 1; }
                    if p.terminal { p.terminal = false; self.reserved -= 1; }
                }
                Reply::Terminal { .. } => {
                    self.stats.terminals += 1;
                    if p.terminal { p.terminal = false; self.reserved -= 1; }
                }
            }
            let client = p.client;
            let done = !p.ack && !p.terminal;
            if done { self.pending.remove(&request); }
            if self.disconnected.contains(&client) || !deliver(client, reply) { self.stats.abandoned += 1; }
            if done && self.disconnected.contains(&client) && !self.pending.values().any(|p| p.client == client) {
                self.disconnected.remove(&client);
            }
        }
        self.plans.reclaim();
        n
    }
}

impl<P> RtIntake<P> {
    /// Starts a release: allows the next [`Self::take`].
    pub fn begin_release(&mut self) { self.taken = false; }

    /// The next ordinary command, at most one per release (None after the first).
    pub fn take(&mut self) -> Option<Descriptor> {
        if self.taken { return None; }
        let d = self.commands.pop().ok()?;
        self.taken = true;
        Some(d)
    }

    /// Commands waiting.
    pub fn queued(&self) -> usize { self.commands.slots() }

    pub fn arm(&self) -> ArmId { self.arm }
    pub fn boot(&self) -> BootId { self.boot }

    /// The plan of an install, None when stale.
    pub fn plan(&self, h: PlanHandle) -> Option<&P> { self.plans.get(h) }

    /// Ends the RT's use of a plan slot (`&mut self`: no plan reference may outlive it).
    pub fn retire(&mut self, h: PlanHandle) -> bool { self.plans.retire(h) }

    /// Replies the reply ring refused (0 unless a reservation is wrong).
    pub fn reply_overflow(&self) -> u64 { self.reply_overflow }

    fn reply(&mut self, r: Reply) {
        if self.replies.push(r).is_err() { self.reply_overflow += 1; }
    }

    /// Acknowledges a commit proven by `committed` (only an authorized core operation makes
    /// one, see [`super::ops::Committed`]); returns the commit sequence.
    pub fn ack(&mut self, d: &Descriptor, committed: super::ops::Committed) -> u64 {
        self.commit_seq += 1;
        let seq = self.commit_seq;
        self.reply(Reply::Ack { request: d.stamp.request, commit_seq: seq, action: committed.action() });
        seq
    }

    /// Refuses `d` with `code`; retires its plan slot.
    pub fn refuse(&mut self, d: &Descriptor, code: &'static str) {
        if let CommandKind::Install { plan } = d.kind { self.plans.retire(plan); }
        self.reply(Reply::Refused { request: d.stamp.request, code });
    }

    /// The physical end of an acknowledged install; retires its plan slot.
    pub fn terminal(&mut self, d: &Descriptor, action: ActionId, outcome: Outcome) {
        if let CommandKind::Install { plan } = d.kind { self.plans.retire(plan); }
        self.reply(Reply::Terminal { request: d.stamp.request, action, outcome });
    }

    /// The broker is gone (dropped): the RT keeps running and drops replies.
    pub fn broker_gone(&self) -> bool { self.commands.is_abandoned() }
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
        assert_eq!(a.admits(&Authority { parent_generation: 1, ..a }), Some("PARENT_REPLACED"));
        let s = Stamp { arm: ArmId::LOCAL, boot: BootId(1), request: RequestId(1), client: ClientId(1), client_seq: 1, authority: a, deadline: Instant::now() };
        assert_eq!(s.check(ArmId::LOCAL, BootId(2), Instant::now(), &a), Some("WRONG_TARGET"));
        assert_eq!(s.check(ArmId::LOCAL, BootId(1), s.deadline + Duration::from_millis(1), &a), Some("DEADLINE_EXPIRED"));
        assert_eq!(s.check(ArmId::LOCAL, BootId(1), s.deadline, &a), None);
    }

    #[test]
    fn one_ordinary_command_per_release() {
        let (mut b, mut rt) = tiny(4, 2, 8);
        for k in 0..3 { b.submit(ClientId(1), k, Authority::default(), later(), Submit::Immediate(ImmediateOp::Heartbeat)).unwrap(); }
        rt.begin_release();
        assert!(rt.take().is_some());
        assert!(rt.take().is_none(), "a second ordinary command in the same release");
        rt.begin_release();
        assert_eq!(rt.take().unwrap().stamp.client_seq, 1);
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
        rt.begin_release();
        let d = rt.take().unwrap();
        let CommandKind::Install { plan: h } = d.kind else { panic!("install first") };
        assert_eq!(rt.plan(h), Some(&[1.0; 4]));
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
            rt.begin_release();
            let d = rt.take().unwrap();
            let CommandKind::Install { plan: h } = d.kind else { panic!() };
            assert_eq!(rt.plan(h), Some(&[k as f32; 4]));
            gens.push(h.generation());
            let seq = rt.ack(&d, super::super::ops::Committed::for_tests(Some(ActionId(k))));
            assert_eq!(seq, k + 1);
            rt.terminal(&d, ActionId(k), Outcome::Completed);
            let mut got = Vec::new();
            assert_eq!(b.poll(|_, r| { got.push(r); true }), 2);
            assert_eq!(got, vec![Reply::Ack { request: r, commit_seq: k + 1, action: Some(ActionId(k)) }, Reply::Terminal { request: r, action: ActionId(k), outcome: Outcome::Completed }]);
        }
        assert_eq!(gens, (1..=100).collect::<Vec<u32>>(), "one slot, a new generation per reuse");
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
    fn abandoned_and_disconnected_clients_release_their_reservations() {
        let (mut b, mut rt) = tiny(4, 2, 8);
        let auth = Authority::default();
        let r1 = b.submit(ClientId(1), 0, auth, later(), Submit::Install(&plan(1.0))).unwrap();
        b.submit(ClientId(2), 0, auth, later(), Submit::Immediate(ImmediateOp::Heartbeat)).unwrap();
        b.disconnect(ClientId(1));
        assert_eq!(b.submit(ClientId(1), 1, auth, later(), Submit::Immediate(ImmediateOp::Heartbeat)), Err(AdmitError::Disconnected));
        rt.begin_release();
        let d1 = rt.take().unwrap();
        rt.ack(&d1, super::super::ops::Committed::for_tests(Some(ActionId(5))));
        rt.begin_release();
        let d2 = rt.take().unwrap();
        rt.refuse(&d2, "NOT_ARMED");
        // client 2's receiver is gone too: deliver says so
        let mut delivered = Vec::new();
        b.poll(|c, r| if c == ClientId(2) { false } else { delivered.push(r); true });
        assert!(delivered.is_empty(), "nothing reaches a disconnected client");
        assert_eq!(b.stats().abandoned, 2);
        assert_eq!(b.reserved(), 1, "the install's terminal is still reserved");
        rt.terminal(&d1, ActionId(5), Outcome::Stopped);
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
        rt.begin_release();
        let d = rt.take().unwrap();
        rt.refuse(&d, "STOPPED");
        assert_eq!(b.poll(|_, r| { assert_eq!(r, Reply::Refused { request: RequestId(1), code: "STOPPED" }); true }), 1);
        assert_eq!((b.reserved(), b.stats().retired), (0, 1));
        assert!(b.submit(ClientId(1), 1, Authority::default(), later(), Submit::Install(&plan(2.0))).is_ok(), "slot reused");
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
}
