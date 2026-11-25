//! Per-connection off-RT reply router for RT executor actions (P-F1).
//!
//! Unifies L's connection lifecycle (`ConnLifecycleClient` + `BrokerDispatcher`) with
//! bounded per-connection reply routing for external action submissions (`BeginMacro` and
//! `InstallPrepared`), while keeping internal macro RPC ACK waiters (`InstallChild`,
//! `FinishMacro`) strictly separate.
//!
//! # Key invariants
//! 1. **Bounded reservation at admission**: every external submission checks connection
//!    liveness, durable closed-client fences, bound destination validity, and both
//!    per-connection and global reply capacity *before* touching `Broker::submit`.
//! 2. **ACK before terminal**: each admitted external request transitions strictly through
//!    `AwaitingAck -> (AwaitingAckCommit ->) AckCommitted -> Terminal`. If a WS caller
//!    abandons or fails to enqueue `command_ack`, `AckCommitGuard::drop` suppresses the
//!    terminal and recovers its reservation exactly once.
//! 3. **At most one terminal per admitted request**: the route is removed upon consuming
//!    `Reply::Terminal` (or `Reply::Refused` / protocol anomaly).
//! 4. **Exact `request <-> action` mapping**: `Reply::Ack` records `ActionId`; `Reply::Terminal`
//!    must match that exact `ActionId`.
//! 5. **Single serial off-RT drain**: `SerialDrainGate` serializes all off-RT drain callers
//!    (`armd-dispatch`, `pump_broker_replies`, `poll_external_replies`, `finish_stopped`)
//!    while avoiding self-deadlock on same-thread callback re-entry and bounding emergency
//!    teardown.
//! 6. **Slot reuse safety**: fixed-size per-connection slots carry a strictly monotonic
//!    `slot_gen: u64` and a per-slot `closed: Arc<AtomicBool>`. Closing a connection blocks
//!    new admissions, retires its pending routes and queued replies, frees the slot index,
//!    and reconciles any in-flight delivery strictly by `(slot_idx, slot_gen)`.
//! 7. **Zero RT involvement**: `armd-control` never locks or touches this module. RT action
//!    completion records cross to `armd-dispatch` through a preallocated lock-free SPSC
//!    ring of `Copy` [`RtTerminalStatus`] records.

use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

use tokio::sync::oneshot;

use crate::model::HolderLabel;
use crate::motion::ActionStatus;
use crate::safety::BoundedText;

use super::broker::{ActionId, AdmitError, ClientId, DeliveryDecision, Reply, RequestId};
use super::executor::OwnerError;
use crate::supervisor::MAX_LIVE_CONNECTIONS;

/// Default bounded per-connection reply capacity for external action terminals.
pub const DEFAULT_PER_CONN_REPLY_CAP: usize = 16;

/// Single plug-in hook for external reply delivery (per-connection WS router or test sink).
pub trait ExternalReplySink: Send + Sync {
    fn deliver(&self, client: ClientId, reply: Reply) -> DeliveryDecision;
}

impl<F> ExternalReplySink for F
where
    F: Fn(ClientId, Reply) -> DeliveryDecision + Send + Sync,
{
    fn deliver(&self, client: ClientId, reply: Reply) -> DeliveryDecision {
        (self)(client, reply)
    }
}

/// Type alias emphasizing per-connection reply destination ownership.
pub type ConnectionReplySink = dyn ExternalReplySink;

/// Policy governing which destination an external action submission may bind to at admission.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExternalDestinationPolicy {
    /// Must bind to an open per-connection slot registered for `client`; never falls back to
    /// the global `ExternalReplySink` (used by all WebSocket action paths).
    ConnectionOnly,
    /// Prefers an open per-connection slot for `client`, falling back to an attached global
    /// `ExternalReplySink` only when `client` is open and not durably closed (used by direct
    /// non-WS API callers and integration tests).
    ConnectionOrGlobal,
}

/// Submission error returned by external action admission (`BeginMacro` / `InstallPrepared`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExternalSubmitError {
    /// Underlying broker or router admission refusal (bounded capacity or disconnected client).
    Admit(AdmitError),
    /// No open reply destination is bound for this client (maps to `503 EXECUTOR_ROUTING_PENDING`).
    NoDestination,
    /// Caller passed a non-action command to an external action admission entry point.
    NotAnExternalAction,
    /// RT executor has already stopped.
    Stopped,
}

/// Per-socket wire-ACK abandonment tracker shared between WS `worker_task`,
/// `subscribe_action_terminals`, and `writer_task` (telemetry + lag resync).
///
/// Guarantees that if a submitting WS socket fails, times out, or cancels before its
/// `command_ack` is enqueued onto `ordinary_out_tx`, no terminal `ArmObservation` for that
/// abandoned action (via `TerminalBroadcaster`, telemetry broadcast, or `hub.latest()` lag
/// resync) can leak to that submitting socket.
#[derive(Debug, Default)]
pub struct WsWireAckTracker {
    inner: Mutex<WsWireAckState>,
}

const ABANDONED_ACTION_HISTORY_CAP: usize = 64;

#[derive(Debug, Default)]
struct WsWireAckState {
    abandoned_ids: VecDeque<u64>,
    socket_closed: bool,
}

impl WsWireAckTracker {
    pub fn new() -> Self {
        Self::default()
    }

    /// Marks `action_id` as abandoned before wire `command_ack` enqueue on this socket.
    pub fn mark_abandoned(&self, action_id: u64) {
        if action_id == 0 {
            return;
        }
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        if !g.abandoned_ids.contains(&action_id) {
            if g.abandoned_ids.len() >= ABANDONED_ACTION_HISTORY_CAP {
                g.abandoned_ids.pop_front();
            }
            g.abandoned_ids.push_back(action_id);
        }
    }

    /// Marks this socket closed for terminal delivery (e.g. failed `ordinary_out_tx` enqueue).
    pub fn mark_socket_closed(&self) {
        self.inner
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .socket_closed = true;
    }

    /// True if `action_id` was abandoned before wire ACK enqueue on this socket.
    pub fn is_abandoned(&self, action_id: u64) -> bool {
        let g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        g.socket_closed || g.abandoned_ids.contains(&action_id)
    }

    /// True if this socket has been marked closed for terminal wire delivery.
    pub fn is_socket_closed(&self) -> bool {
        self.inner
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .socket_closed
    }

    /// Redacts any finished (`!is_running()`) abandoned action from `obs.action` and
    /// `obs.previous_action` before telemetry/lag-resync serialization on the submitting socket.
    pub fn redact_abandoned_obs(&self, obs: &mut crate::model::ArmObservation) {
        let g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        if g.abandoned_ids.is_empty() && !g.socket_closed {
            return;
        }
        if let Some(a) = obs.action.as_ref() {
            if !a.is_running() && (g.socket_closed || g.abandoned_ids.contains(&a.id)) {
                obs.action = None;
            }
        }
        if let Some(prev) = obs.previous_action.as_ref() {
            if !prev.is_running() && (g.socket_closed || g.abandoned_ids.contains(&prev.id)) {
                obs.previous_action = None;
            }
        }
    }
}

/// Zero-allocation, `Copy` terminal record pushed on `armd-control` into a preallocated lock-free
/// SPSC ring (`rtrb::RingBuffer<RtTerminalStatus>`) whenever an action reaches a terminal state.
///
/// Drained off-RT by `OffThreadSinks::drain_all()` into `TerminalBroadcaster` strictly before
/// telemetry broadcast so rapid consecutive completions are never lost when `armd-dispatch` stalls.
#[derive(Debug, Clone, Copy)]
pub struct RtTerminalStatus {
    pub id: u64,
    pub kind: HolderLabel,
    pub state: HolderLabel,
    pub code: Option<HolderLabel>,
    pub message: BoundedText<160>,
    pub has_message: bool,
    pub progress: f32,
    pub started_at: f64,
    pub finished_at: Option<f64>,
    pub holding: Option<bool>,
    pub gripper_gap_rad: Option<f32>,
    pub settled: Option<bool>,
    pub phase: Option<HolderLabel>,
    pub iteration: Option<u32>,
    pub error_px: Option<f32>,
    pub final_offset_px: Option<[f32; 2]>,
    pub acquire_height_m: Option<f32>,
    pub power_scale: Option<f32>,
    pub episode: Option<HolderLabel>,
    pub t_s: Option<f32>,
    pub step_index: Option<usize>,
    pub repeat_index: Option<u32>,
    pub repeats: Option<u32>,
    pub failed_step: Option<usize>,
    pub align_turn_rad: Option<f32>,
    pub angle_spread_deg: Option<f32>,
    pub grasp_width_m: Option<f32>,
    pub object_height_m: Option<f32>,
    pub close_height_m: Option<f32>,
}

impl RtTerminalStatus {
    /// Captures a terminal [`ActionStatus`] with zero heap allocations.
    pub fn from_action_status(a: &ActionStatus) -> Self {
        let (has_message, message) = match a.message.as_deref() {
            Some(m) => (true, BoundedText::new(m)),
            None => (false, BoundedText::default()),
        };
        Self {
            id: a.id,
            kind: a.kind,
            state: a.state,
            code: a.code,
            message,
            has_message,
            progress: a.progress,
            started_at: a.started_at,
            finished_at: a.finished_at,
            holding: a.holding,
            gripper_gap_rad: a.gripper_gap_rad,
            settled: a.settled,
            phase: a.phase,
            iteration: a.iteration,
            error_px: a.error_px,
            final_offset_px: a.final_offset_px,
            acquire_height_m: a.acquire_height_m,
            power_scale: a.power_scale,
            episode: a.episode,
            t_s: a.t_s,
            step_index: a.step_index,
            repeat_index: a.repeat_index,
            repeats: a.repeats,
            failed_step: a.failed_step,
            align_turn_rad: a.align_turn_rad,
            angle_spread_deg: a.angle_spread_deg,
            grasp_width_m: a.grasp_width_m,
            object_height_m: a.object_height_m,
            close_height_m: a.close_height_m,
        }
    }

    /// Reconstructs an off-RT [`ActionStatus`] for `TerminalBroadcaster`, attaching any sidecar
    /// measurement payload associated with `self.id`.
    pub fn into_action_status(self, measurement: Option<serde_json::Value>) -> ActionStatus {
        let mut status = ActionStatus::blank();
        status.id = self.id;
        status.kind = self.kind;
        status.started_at = self.started_at;
        status.state = self.state;
        status.code = self.code;
        status.message = self
            .has_message
            .then(|| self.message.as_str().to_owned());
        status.progress = self.progress;
        status.finished_at = self.finished_at;
        status.holding = self.holding;
        status.gripper_gap_rad = self.gripper_gap_rad;
        status.settled = self.settled;
        status.phase = self.phase;
        status.iteration = self.iteration;
        status.error_px = self.error_px;
        status.final_offset_px = self.final_offset_px;
        status.acquire_height_m = self.acquire_height_m;
        status.power_scale = self.power_scale;
        status.episode = self.episode;
        status.t_s = self.t_s;
        status.step_index = self.step_index;
        status.repeat_index = self.repeat_index;
        status.repeats = self.repeats;
        status.failed_step = self.failed_step;
        status.align_turn_rad = self.align_turn_rad;
        status.angle_spread_deg = self.angle_spread_deg;
        status.grasp_width_m = self.grasp_width_m;
        status.object_height_m = self.object_height_m;
        status.close_height_m = self.close_height_m;
        status.measurement = measurement;
        status
    }
}

/// Single-producer `Sync`-safe lock-free SPSC sender for [`RtTerminalStatus`].
///
/// Mirrors `PolicyTerminalTx`: all access to the inner `rtrb::Producer` requires `&mut self`,
/// while `free_slots(&self)` reads only atomic `pushed`/`popped` counters (zero locks, zero allocs).
pub struct RtTerminalTx {
    tx: rtrb::Producer<RtTerminalStatus>,
    cap: usize,
    pushed: AtomicU64,
    popped: Arc<AtomicU64>,
}

// SAFETY: `self.tx` is only ever accessed through `&mut self` (`push`); `&self` methods read
// only atomic counters and immutable `cap`, preserving single-thread exclusive producer access.
unsafe impl Sync for RtTerminalTx {}

impl RtTerminalTx {
    pub fn new(tx: rtrb::Producer<RtTerminalStatus>, cap: usize, popped: Arc<AtomicU64>) -> Self {
        Self {
            tx,
            cap,
            pushed: AtomicU64::new(0),
            popped,
        }
    }

    /// Approximate free slots available in the ring (lower bound safe for admission checks).
    #[inline]
    pub fn free_slots(&self) -> usize {
        let pushed = self.pushed.load(Ordering::Acquire);
        let popped = self.popped.load(Ordering::Acquire);
        let used = pushed.saturating_sub(popped) as usize;
        self.cap.saturating_sub(used)
    }

    #[inline]
    pub fn push(&mut self, rec: RtTerminalStatus) -> Result<(), rtrb::PushError<RtTerminalStatus>> {
        self.tx.push(rec)?;
        self.pushed.fetch_add(1, Ordering::Release);
        Ok(())
    }
}

/// Snapshot of reply router counters and reservations for diagnostics and tests.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ReplyRouterStats {
    pub active_connections: usize,
    pub active_external_routes: usize,
    pub active_internal_waiters: usize,
    pub global_reserved: usize,
    pub global_cap: usize,
    pub pending_external_deliveries: usize,
    pub pending_unrouted: usize,
    pub terminals_delivered: u64,
    pub terminals_abandoned: u64,
    pub mapping_mismatches: u64,
    pub protocol_anomalies: u64,
}

/// Bound destination token captured atomically at external action admission.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ReplyTarget {
    Connection {
        client: ClientId,
        slot_idx: usize,
        slot_gen: u64,
    },
    GlobalSink {
        client: ClientId,
        sink_gen: u64,
    },
}

/// Per-connection fixed-slot registration returned on `register_connection`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConnectionSlotToken {
    pub client: ClientId,
    pub slot_idx: usize,
    pub slot_gen: u64,
}

struct ConnSlot {
    client: ClientId,
    slot_idx: usize,
    slot_gen: u64,
    sink: Arc<ConnectionReplySink>,
    closed: Arc<AtomicBool>,
    cap: usize,
    reserved: usize,
    pending: VecDeque<Reply>,
}

enum ExternalRouteState {
    AwaitingAck {
        ack_tx: Option<oneshot::Sender<Result<Reply, OwnerError>>>,
        require_ack_commit: bool,
        wire_tracker: Option<Arc<WsWireAckTracker>>,
    },
    AwaitingAckCommit {
        action_id: ActionId,
        held_terminal: Option<Reply>,
        wire_tracker: Option<Arc<WsWireAckTracker>>,
    },
    AckCommitted {
        action_id: ActionId,
    },
    Abandoned {
        #[allow(dead_code)]
        action_id: Option<ActionId>,
        wire_tracker: Option<Arc<WsWireAckTracker>>,
    },
}

struct ExternalActionRoute {
    client: ClientId,
    target: ReplyTarget,
    /// Single-owner credit flag: true iff this route currently owns 1 reservation credit
    /// in `global_reserved` (and in `slot.reserved` when `target` is `Connection`).
    reserved_held: bool,
    state: ExternalRouteState,
}

pub(crate) struct InFlightDelivery {
    pub(crate) target: ReplyTarget,
    pub(crate) client: ClientId,
    pub(crate) sink: Arc<ConnectionReplySink>,
    pub(crate) closed: Option<Arc<AtomicBool>>,
    pub(crate) reply: Reply,
    /// True when this delivery owns 1 credit in `global_reserved` (and `slot.reserved` for `Connection`).
    pub(crate) owns_reservation: bool,
}

/// RAII wire-ACK enqueue barrier returned with every external action admission.
///
/// For WebSocket callers (`require_ack_commit == true`), call [`Self::confirm_enqueued`] strictly
/// *after* `command_ack` has been enqueued onto the socket's lossless `ordinary_out_tx` and
/// *before* dropping `CommandReplyGateGuard`. Dropping an unconfirmed guard abandons the terminal
/// in both the broker router and the socket's [`WsWireAckTracker`], recovering the reservation
/// credit exactly once.
pub struct AckCommitGuard {
    router: Arc<Mutex<ActionReplyRouter>>,
    request: RequestId,
    wire_tracker: Option<Arc<WsWireAckTracker>>,
    armed: bool,
}

impl std::fmt::Debug for AckCommitGuard {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AckCommitGuard")
            .field("request", &self.request)
            .field("armed", &self.armed)
            .finish()
    }
}

impl AckCommitGuard {
    pub(crate) fn new(
        router: Arc<Mutex<ActionReplyRouter>>,
        request: RequestId,
        wire_tracker: Option<Arc<WsWireAckTracker>>,
        armed: bool,
    ) -> Self {
        Self {
            router,
            request,
            wire_tracker,
            armed,
        }
    }

    /// Confirms that `command_ack` was enqueued onto the client's outbound control queue.
    /// Returns `true` if a held `Reply::Terminal` was released into the connection's pending
    /// delivery queue and is ready to be drained.
    pub fn confirm_enqueued(mut self) -> bool {
        if !self.armed {
            return false;
        }
        self.armed = false;
        let mut router = self.router.lock().unwrap_or_else(|e| e.into_inner());
        router.confirm_ack_enqueued(self.request)
    }

    /// Explicitly marks the socket closed (e.g. failed `ordinary_out_tx` enqueue) and abandons
    /// the uncommitted ACK barrier immediately.
    pub fn abandon_closed(mut self) {
        if let Some(t) = self.wire_tracker.as_ref() {
            t.mark_socket_closed();
        }
        self.abandon_inner();
    }

    fn abandon_inner(&mut self) {
        if !self.armed {
            return;
        }
        self.armed = false;
        let mut router = self.router.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(action_id) = router.abandon_uncommitted_ack(self.request, self.wire_tracker.clone()) {
            if let Some(t) = self.wire_tracker.as_ref() {
                t.mark_abandoned(action_id.0);
            }
        }
    }
}

impl Drop for AckCommitGuard {
    fn drop(&mut self) {
        self.abandon_inner();
    }
}

/// Re-entrancy-safe, emergency-bounded single serial off-RT drain gate.
///
/// Lock order: `SerialDrainGate` (0) -> `BrokerDispatcher::state` (1) ->
/// `ConnLifecycleClient::inner` (2) -> `broker` (3) -> `reply_router` (4).
pub(crate) struct SerialDrainGate {
    state: Mutex<SerialDrainState>,
    cv: Condvar,
    owner_tid: AtomicU64,
}

struct SerialDrainState {
    active: bool,
    /// Deferred `finish_stopped(rt_joined)` requested while another thread (emergency timeout)
    /// or the same thread (re-entrant sink callback calling `stop_and_join`) held the gate.
    deferred_finish_stopped: Option<bool>,
}

static NEXT_DRAIN_TID: AtomicU64 = AtomicU64::new(1);
thread_local! {
    static THIS_DRAIN_TID: u64 = NEXT_DRAIN_TID.fetch_add(1, Ordering::Relaxed);
}

fn current_drain_tid() -> u64 {
    THIS_DRAIN_TID.with(|id| *id)
}

pub(crate) enum DrainEnter {
    Entered,
    SameThreadReentrant,
    TimedOut,
}

impl SerialDrainGate {
    pub(crate) fn new() -> Self {
        Self {
            state: Mutex::new(SerialDrainState {
                active: false,
                deferred_finish_stopped: None,
            }),
            cv: Condvar::new(),
            owner_tid: AtomicU64::new(0),
        }
    }

    /// Acquires the serial drain gate, blocking if another thread holds it, or returning
    /// `DrainEnter::SameThreadReentrant` if the calling thread already holds the gate (callback
    /// re-entry).
    pub(crate) fn enter(&self) -> DrainEnter {
        let tid = current_drain_tid();
        if self.owner_tid.load(Ordering::Acquire) == tid {
            return DrainEnter::SameThreadReentrant;
        }
        let mut g = self.state.lock().unwrap_or_else(|e| e.into_inner());
        while g.active {
            g = self.cv.wait(g).unwrap_or_else(|e| e.into_inner());
        }
        g.active = true;
        self.owner_tid.store(tid, Ordering::Release);
        DrainEnter::Entered
    }

    /// Acquires the serial drain gate up to `timeout`; if same-thread re-entry or timeout occurs,
    /// atomically records `deferred_finish_stopped = Some(rt_joined)` under the gate mutex so the
    /// active owner executes finalization before releasing the gate.
    pub(crate) fn enter_for_shutdown(
        &self,
        emergency_bound: Option<Duration>,
        rt_joined: bool,
    ) -> DrainEnter {
        let tid = current_drain_tid();
        let mut g = self.state.lock().unwrap_or_else(|e| e.into_inner());
        if g.active && self.owner_tid.load(Ordering::Acquire) == tid {
            g.deferred_finish_stopped = Some(
                g.deferred_finish_stopped
                    .map_or(rt_joined, |prev| prev || rt_joined),
            );
            return DrainEnter::SameThreadReentrant;
        }
        if let Some(bound) = emergency_bound {
            let deadline = Instant::now() + bound.min(Duration::from_millis(50));
            while g.active {
                let rem = deadline.saturating_duration_since(Instant::now());
                if rem.is_zero() {
                    g.deferred_finish_stopped = Some(
                        g.deferred_finish_stopped
                            .map_or(rt_joined, |prev| prev || rt_joined),
                    );
                    return DrainEnter::TimedOut;
                }
                let (next, res) = self
                    .cv
                    .wait_timeout(g, rem)
                    .unwrap_or_else(|e| e.into_inner());
                g = next;
                if res.timed_out() && g.active {
                    g.deferred_finish_stopped = Some(
                        g.deferred_finish_stopped
                            .map_or(rt_joined, |prev| prev || rt_joined),
                    );
                    return DrainEnter::TimedOut;
                }
            }
        } else {
            while g.active {
                g = self.cv.wait(g).unwrap_or_else(|e| e.into_inner());
            }
        }
        g.active = true;
        self.owner_tid.store(tid, Ordering::Release);
        DrainEnter::Entered
    }

    /// Atomically checks for deferred shutdown finalization under the gate mutex; releases the
    /// gate ONLY when no deferred work remains. If `Some(rt_joined)` is returned, the caller
    /// still owns the gate, must run `finalize_stopped_locked(rt_joined)`, and then call
    /// `try_release_or_take_deferred()` again until it returns `None`.
    pub(crate) fn try_release_or_take_deferred(&self) -> Option<bool> {
        let mut g = self.state.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(rt_joined) = g.deferred_finish_stopped.take() {
            return Some(rt_joined);
        }
        self.owner_tid.store(0, Ordering::Release);
        g.active = false;
        self.cv.notify_all();
        None
    }
}

/// Unified off-RT action reply router (replaces PR P's stopgap global-only router).
pub(crate) struct ActionReplyRouter {
    /// Separate internal ACK waiters for non-action macro RPCs (`InstallChild`, `FinishMacro`).
    pub(crate) internal_waiters: HashMap<RequestId, oneshot::Sender<Reply>>,
    /// Active external action routes (`BeginMacro`, `InstallPrepared`) bound to a destination.
    external_routes: HashMap<RequestId, ExternalActionRoute>,
    /// Fixed-size per-connection reply slots (`<= MAX_LIVE_CONNECTIONS`).
    slots: Vec<Option<ConnSlot>>,
    /// Fast lookup from live `ClientId` to `slot_idx`.
    client_to_slot: HashMap<ClientId, usize>,
    /// Strictly monotonic generation assigned to every registered `ConnSlot`.
    next_slot_gen: u64,
    /// Round-robin cursor across `slots` so no single backpressured connection starves others.
    rr_cursor: usize,
    /// Durable set of closed `ClientId`s (never cleared on last reply; prevents re-admission
    /// via `ConnectionOrGlobal` after `disconnect_client_only`).
    closed_clients: HashSet<ClientId>,
    /// Fallback global sink for non-WS direct API / integration tests.
    global_sink: Option<Arc<ConnectionReplySink>>,
    global_sink_gen: u64,
    /// Monotonic arrival sequence shared between `global_pending` and `unrouted_external` so
    /// deliveries to `global_sink` preserve exact global broker FIFO order.
    next_global_seq: u64,
    /// Queued terminals targeting `global_sink` (bounded by pre-reserved credits).
    global_pending: VecDeque<(u64, ClientId, u64, Reply)>,
    /// Unrouted replies from low-level raw `broker.submit` fixtures (no bound route).
    unrouted_external: VecDeque<(u64, ClientId, Reply)>,
    /// Global bounded capacity (tracks `global_reserved + unrouted_external.len() <= global_cap`).
    global_cap: usize,
    /// Default per-connection bounded reply capacity.
    default_per_conn_cap: usize,
    /// Exact sum of all active route credits + queued `slot.pending` + `global_pending` + in-flight delivery.
    global_reserved: usize,
    /// Counters for invariants & diagnostics.
    terminals_delivered: u64,
    terminals_abandoned: u64,
    mapping_mismatches: u64,
    protocol_anomalies: u64,
}

impl ActionReplyRouter {
    pub(crate) fn new(global_cap: usize) -> Self {
        let cap = global_cap.max(1);
        let default_per_conn_cap = DEFAULT_PER_CONN_REPLY_CAP.min(cap).max(1);
        let mut slots = Vec::with_capacity(MAX_LIVE_CONNECTIONS);
        for _ in 0..MAX_LIVE_CONNECTIONS {
            slots.push(None);
        }
        Self {
            internal_waiters: HashMap::new(),
            external_routes: HashMap::with_capacity(cap),
            slots,
            client_to_slot: HashMap::with_capacity(MAX_LIVE_CONNECTIONS),
            next_slot_gen: 0,
            rr_cursor: 0,
            closed_clients: HashSet::new(),
            global_sink: None,
            global_sink_gen: 0,
            next_global_seq: 0,
            global_pending: VecDeque::with_capacity(cap),
            unrouted_external: VecDeque::with_capacity(cap),
            global_cap: cap,
            default_per_conn_cap,
            global_reserved: 0,
            terminals_delivered: 0,
            terminals_abandoned: 0,
            mapping_mismatches: 0,
            protocol_anomalies: 0,
        }
    }

    pub(crate) fn stats(&self) -> ReplyRouterStats {
        let pending_slots: usize = self
            .slots
            .iter()
            .filter_map(|s| s.as_ref())
            .map(|s| s.pending.len())
            .sum();
        ReplyRouterStats {
            active_connections: self.client_to_slot.len(),
            active_external_routes: self.external_routes.len(),
            active_internal_waiters: self.internal_waiters.len(),
            global_reserved: self.global_reserved,
            global_cap: self.global_cap,
            pending_external_deliveries: pending_slots + self.global_pending.len(),
            pending_unrouted: self.unrouted_external.len(),
            terminals_delivered: self.terminals_delivered,
            terminals_abandoned: self.terminals_abandoned,
            mapping_mismatches: self.mapping_mismatches,
            protocol_anomalies: self.protocol_anomalies,
        }
    }

    pub(crate) fn has_external_consumers(&self) -> bool {
        !self.internal_waiters.is_empty()
            || !self.external_routes.is_empty()
            || !self.client_to_slot.is_empty()
            || self.global_sink.is_some()
    }

    pub(crate) fn has_global_sink(&self) -> bool {
        self.global_sink.is_some()
    }

    pub(crate) fn has_connection_slot(&self, client: ClientId) -> bool {
        self.client_to_slot
            .get(&client)
            .and_then(|&idx| self.slots[idx].as_ref())
            .is_some_and(|s| s.client == client && !s.closed.load(Ordering::Acquire))
    }

    pub(crate) fn pending_external_count(&self) -> usize {
        let slot_pending: usize = self
            .slots
            .iter()
            .filter_map(|s| s.as_ref())
            .map(|s| s.pending.len())
            .sum();
        slot_pending + self.global_pending.len() + self.unrouted_external.len()
    }

    /// Registers a per-connection reply sink in a free fixed slot with a strictly monotonic
    /// `slot_gen`, rejecting zero IDs, durably closed clients, duplicate live registrations, or
    /// a full slot table.
    pub(crate) fn register_connection(
        &mut self,
        client: ClientId,
        sink: Arc<ConnectionReplySink>,
        per_conn_cap: Option<usize>,
    ) -> Result<ConnectionSlotToken, AdmitError> {
        if client.0 == 0 || self.closed_clients.contains(&client) {
            return Err(AdmitError::Disconnected);
        }
        if let Some(&idx) = self.client_to_slot.get(&client) {
            if let Some(existing) = self.slots[idx].as_ref() {
                if !existing.closed.load(Ordering::Acquire) {
                    return Ok(ConnectionSlotToken {
                        client,
                        slot_idx: idx,
                        slot_gen: existing.slot_gen,
                    });
                }
            }
        }
        let Some(slot_idx) = self.slots.iter().position(Option::is_none) else {
            return Err(AdmitError::RepliesFull);
        };
        self.next_slot_gen += 1;
        let slot_gen = self.next_slot_gen;
        let cap = per_conn_cap
            .unwrap_or(self.default_per_conn_cap)
            .min(self.global_cap)
            .max(1);
        self.slots[slot_idx] = Some(ConnSlot {
            client,
            slot_idx,
            slot_gen,
            sink,
            closed: Arc::new(AtomicBool::new(false)),
            cap,
            reserved: 0,
            pending: VecDeque::with_capacity(cap),
        });
        self.client_to_slot.insert(client, slot_idx);
        Ok(ConnectionSlotToken {
            client,
            slot_idx,
            slot_gen,
        })
    }

    /// Updates the default per-connection reply capacity (used by capacity-saturation tests).
    pub(crate) fn set_per_connection_cap(&mut self, cap: usize) {
        let clamped = cap.min(self.global_cap).max(1);
        self.default_per_conn_cap = clamped;
        for slot in self.slots.iter_mut().flatten() {
            slot.cap = clamped;
        }
    }

    /// Attaches (or replaces) the fallback global `ExternalReplySink`, advancing `global_sink_gen`
    /// and cleanly retiring any old-generation `global_pending` and `GlobalSink` routes.
    pub(crate) fn attach_global_sink(&mut self, sink: Arc<ConnectionReplySink>) {
        self.retire_global_sink_generation();
        self.global_sink_gen += 1;
        self.global_sink = Some(sink);
    }

    /// Detaches the fallback global `ExternalReplySink` and retires all old-generation credits.
    pub(crate) fn detach_global_sink(&mut self) {
        self.retire_global_sink_generation();
        self.global_sink_gen += 1;
        self.global_sink = None;
    }

    fn retire_global_sink_generation(&mut self) {
        let dropped_pending = self.global_pending.len();
        self.global_pending.clear();
        self.global_reserved = self.global_reserved.saturating_sub(dropped_pending);
        self.terminals_abandoned += dropped_pending as u64;

        let old_gen = self.global_sink_gen;
        for route in self.external_routes.values_mut() {
            if matches!(route.target, ReplyTarget::GlobalSink { sink_gen, .. } if sink_gen == old_gen)
            {
                if route.reserved_held {
                    route.reserved_held = false;
                    self.global_reserved = self.global_reserved.saturating_sub(1);
                }
                match &mut route.state {
                    ExternalRouteState::AwaitingAck { .. } => {
                        // Keep ack_tx alive so caller receives Ack/Refused, then terminal is abandoned
                        // because reserved_held is false and sink_gen no longer matches.
                    }
                    ExternalRouteState::AwaitingAckCommit {
                        action_id,
                        held_terminal,
                        wire_tracker,
                    } => {
                        if held_terminal.take().is_some() {
                            self.terminals_abandoned += 1;
                        }
                        let aid = *action_id;
                        let wt = wire_tracker.take();
                        route.state = ExternalRouteState::Abandoned {
                            action_id: Some(aid),
                            wire_tracker: wt,
                        };
                    }
                    ExternalRouteState::AckCommitted { action_id } => {
                        let aid = *action_id;
                        route.state = ExternalRouteState::Abandoned {
                            action_id: Some(aid),
                            wire_tracker: None,
                        };
                    }
                    ExternalRouteState::Abandoned { .. } => {}
                }
            }
        }
    }

    /// Closes a connection immediately: records durable closure, blocks future admissions,
    /// retires all routes and queued replies for `client` across both its `ConnSlot` and
    /// `global_pending`/`unrouted_external`, and frees `slot_idx` so future connections reuse it
    /// only with a strictly larger `slot_gen`.
    ///
    /// Does NOT touch any popped `InFlightDelivery` credit (that credit stays owned by the
    /// active drain until [`Self::reconcile_in_flight`] runs).
    pub(crate) fn close_connection(&mut self, client: ClientId) {
        if client.0 == 0 {
            return;
        }
        self.closed_clients.insert(client);

        // 1. Close & free per-connection slot (if registered).
        if let Some(slot_idx) = self.client_to_slot.remove(&client) {
            if let Some(mut slot) = self.slots[slot_idx].take() {
                slot.closed.store(true, Ordering::Release);
                let queued = slot.pending.len();
                slot.pending.clear();
                self.global_reserved = self.global_reserved.saturating_sub(queued);
                self.terminals_abandoned += queued as u64;
                // `slot` is dropped here; `slot_idx` is now `None` and reusable with a new `slot_gen`.
            }
        }

        // 2. Retire any queued global_pending and unrouted replies for this client.
        let before_global = self.global_pending.len();
        self.global_pending.retain(|(_, c, _, _)| *c != client);
        let removed_global = before_global - self.global_pending.len();
        self.global_reserved = self.global_reserved.saturating_sub(removed_global);
        self.terminals_abandoned += removed_global as u64;

        let before_unrouted = self.unrouted_external.len();
        self.unrouted_external.retain(|(_, c, _)| *c != client);
        self.terminals_abandoned += (before_unrouted - self.unrouted_external.len()) as u64;

        // 3. Retire all active external routes belonging to `client` (waking any awaiting ACK
        //    with `OwnerError::Stopped` and releasing their route-owned reservation credits).
        let req_ids: Vec<RequestId> = self
            .external_routes
            .iter()
            .filter_map(|(&req, r)| (r.client == client).then_some(req))
            .collect();
        for req in req_ids {
            if let Some(mut route) = self.external_routes.remove(&req) {
                if route.reserved_held {
                    route.reserved_held = false;
                    self.global_reserved = self.global_reserved.saturating_sub(1);
                }
                match route.state {
                    ExternalRouteState::AwaitingAck { ack_tx, .. } => {
                        if let Some(tx) = ack_tx {
                            let _ = tx.send(Err(OwnerError::Stopped));
                        }
                    }
                    ExternalRouteState::AwaitingAckCommit { held_terminal, .. } => {
                        if held_terminal.is_some() {
                            self.terminals_abandoned += 1;
                        }
                    }
                    ExternalRouteState::AckCommitted { .. }
                    | ExternalRouteState::Abandoned { .. } => {}
                }
            }
        }
    }

    /// Removes a newly-registered `ConnSlot` during rollback of a rejected/cancelled `Open`
    /// without marking `client` durably closed if RT never opened it (unless already closed).
    pub(crate) fn rollback_unopened_connection(&mut self, token: ConnectionSlotToken) {
        if let Some(slot) = self.slots[token.slot_idx].as_ref() {
            if slot.slot_gen == token.slot_gen && slot.client == token.client {
                slot.closed.store(true, Ordering::Release);
                if self.client_to_slot.get(&token.client) == Some(&token.slot_idx) {
                    self.client_to_slot.remove(&token.client);
                }
                let removed = self.slots[token.slot_idx].take();
                if let Some(s) = removed {
                    let q = s.pending.len();
                    self.global_reserved = self.global_reserved.saturating_sub(q);
                }
            }
        }
    }

    /// Validates destination and bounded reply capacity for an external action *before* calling
    /// `Broker::submit`. Returns the bound [`ReplyTarget`] on success without mutating state yet;
    /// call [`Self::commit_external_route`] strictly after `Broker::submit` returns `Ok(req_id)`.
    pub(crate) fn check_external_admission(
        &self,
        client: ClientId,
        policy: ExternalDestinationPolicy,
    ) -> Result<ReplyTarget, ExternalSubmitError> {
        if client.0 != 0 && self.closed_clients.contains(&client) {
            return Err(ExternalSubmitError::Admit(AdmitError::Disconnected));
        }
        if client.0 == 0 && policy == ExternalDestinationPolicy::ConnectionOnly {
            return Err(ExternalSubmitError::Admit(AdmitError::Disconnected));
        }
        // Global capacity accounts for both reserved routes/pending and any buffered unrouted replies.
        let global_used = self.global_reserved + self.unrouted_external.len();
        if global_used + 1 > self.global_cap {
            return Err(ExternalSubmitError::Admit(AdmitError::RepliesFull));
        }
        if client.0 != 0 {
            if let Some(&slot_idx) = self.client_to_slot.get(&client) {
                if let Some(slot) = self.slots[slot_idx].as_ref() {
                    if slot.closed.load(Ordering::Acquire) {
                        return Err(ExternalSubmitError::Admit(AdmitError::Disconnected));
                    }
                    if slot.reserved + 1 > slot.cap {
                        return Err(ExternalSubmitError::Admit(AdmitError::RepliesFull));
                    }
                    return Ok(ReplyTarget::Connection {
                        client,
                        slot_idx,
                        slot_gen: slot.slot_gen,
                    });
                }
            }
        }
        match policy {
            ExternalDestinationPolicy::ConnectionOnly => Err(ExternalSubmitError::NoDestination),
            ExternalDestinationPolicy::ConnectionOrGlobal => {
                if self.global_sink.is_some() {
                    Ok(ReplyTarget::GlobalSink {
                        client,
                        sink_gen: self.global_sink_gen,
                    })
                } else {
                    Err(ExternalSubmitError::NoDestination)
                }
            }
        }
    }

    /// Atomically reserves 1 reply credit and binds `request -> target` after `Broker::submit`
    /// succeeds (called under the same lock acquisition as [`Self::check_external_admission`]).
    pub(crate) fn commit_external_route(
        &mut self,
        request: RequestId,
        client: ClientId,
        target: ReplyTarget,
        ack_tx: oneshot::Sender<Result<Reply, OwnerError>>,
        require_ack_commit: bool,
        wire_tracker: Option<Arc<WsWireAckTracker>>,
    ) {
        self.global_reserved += 1;
        if let ReplyTarget::Connection {
            slot_idx,
            slot_gen,
            ..
        } = target
        {
            if let Some(slot) = self.slots[slot_idx].as_mut() {
                debug_assert_eq!(slot.slot_gen, slot_gen);
                slot.reserved += 1;
            }
        }
        self.external_routes.insert(
            request,
            ExternalActionRoute {
                client,
                target,
                reserved_held: true,
                state: ExternalRouteState::AwaitingAck {
                    ack_tx: Some(ack_tx),
                    require_ack_commit,
                    wire_tracker,
                },
            },
        );
    }

    /// Releases 1 reservation credit held by `target` (only decrementing `slot.reserved` if
    /// `slots[slot_idx]` still matches `(client, slot_gen)`).
    fn release_target_credit(&mut self, target: ReplyTarget) {
        self.global_reserved = self.global_reserved.saturating_sub(1);
        if let ReplyTarget::Connection {
            client,
            slot_idx,
            slot_gen,
        } = target
        {
            if let Some(slot) = self.slots[slot_idx].as_mut() {
                if slot.slot_gen == slot_gen && slot.client == client {
                    slot.reserved = slot.reserved.saturating_sub(1);
                }
            }
        }
    }

    /// Confirms wire-ACK enqueue for `request` (transitioning `AwaitingAckCommit -> AckCommitted`
    /// or moving a held `Reply::Terminal` into its bound pending queue).
    pub(crate) fn confirm_ack_enqueued(&mut self, request: RequestId) -> bool {
        let Some(route) = self.external_routes.get_mut(&request) else {
            return false;
        };
        let ExternalRouteState::AwaitingAckCommit {
            action_id,
            held_terminal,
            ..
        } = &mut route.state
        else {
            return false;
        };
        let aid = *action_id;
        let held = held_terminal.take();
        if let Some(term_reply) = held {
            let route = self.external_routes.remove(&request).expect("checked above");
            self.enqueue_routed_terminal(route, term_reply)
        } else {
            route.state = ExternalRouteState::AckCommitted { action_id: aid };
            false
        }
    }

    /// Abandons an uncommitted ACK barrier (`AckCommitGuard::drop`), returning `Some(action_id)`
    /// if the action ID is already known. Recovers the route's reservation credit exactly once.
    pub(crate) fn abandon_uncommitted_ack(
        &mut self,
        request: RequestId,
        tracker: Option<Arc<WsWireAckTracker>>,
    ) -> Option<ActionId> {
        let route = self.external_routes.get_mut(&request)?;
        let target = route.target;
        let release_credit = if route.reserved_held {
            route.reserved_held = false;
            true
        } else {
            false
        };
        let mut remove_now = false;
        let mut known_action = None;
        match &mut route.state {
            ExternalRouteState::AwaitingAck {
                ack_tx,
                wire_tracker,
                ..
            } => {
                ack_tx.take();
                let wt = tracker.or_else(|| wire_tracker.take());
                route.state = ExternalRouteState::Abandoned {
                    action_id: None,
                    wire_tracker: wt,
                };
            }
            ExternalRouteState::AwaitingAckCommit {
                action_id,
                held_terminal,
                wire_tracker,
            } => {
                known_action = Some(*action_id);
                if held_terminal.take().is_some() {
                    self.terminals_abandoned += 1;
                    remove_now = true;
                } else {
                    let aid = *action_id;
                    let wt = tracker.or_else(|| wire_tracker.take());
                    route.state = ExternalRouteState::Abandoned {
                        action_id: Some(aid),
                        wire_tracker: wt,
                    };
                }
            }
            ExternalRouteState::AckCommitted { .. } => {
                // Already confirmed earlier; nothing to abandon.
                if release_credit {
                    route.reserved_held = true;
                }
                return None;
            }
            ExternalRouteState::Abandoned {
                action_id,
                wire_tracker,
            } => {
                known_action = *action_id;
                if wire_tracker.is_none() {
                    *wire_tracker = tracker;
                }
            }
        }
        if remove_now {
            self.external_routes.remove(&request);
        }
        if release_credit {
            self.release_target_credit(target);
        }
        known_action
    }

    /// Routes one broker reply popped inside `Broker::try_poll` (not owned by `BrokerDispatcher::managed`).
    pub(crate) fn route_broker_reply(
        &mut self,
        client: ClientId,
        reply: Reply,
    ) -> DeliveryDecision {
        let req = reply.request();

        // 1. Separate internal ACK waiters (`InstallChild`, `FinishMacro`).
        if let Some(tx) = self.internal_waiters.remove(&req) {
            let _ = tx.send(reply);
            return DeliveryDecision::Delivered;
        }

        // 2. Managed external action routes (`BeginMacro`, `InstallPrepared`).
        if self.external_routes.contains_key(&req) {
            return self.route_external_action_reply(client, reply);
        }

        // 3. Unrouted replies from low-level raw `broker.submit` fixtures.
        if client.0 != 0 && self.closed_clients.contains(&client) {
            self.terminals_abandoned += 1;
            return DeliveryDecision::Abandoned;
        }
        if self.global_reserved + self.unrouted_external.len() < self.global_cap {
            self.next_global_seq += 1;
            let seq = self.next_global_seq;
            self.unrouted_external.push_back((seq, client, reply));
            DeliveryDecision::Delivered
        } else {
            DeliveryDecision::Backpressured
        }
    }

    fn route_external_action_reply(&mut self, client: ClientId, reply: Reply) -> DeliveryDecision {
        let req = reply.request();
        let mut route = self.external_routes.remove(&req).expect("checked contains_key");
        if route.client != client {
            self.mapping_mismatches += 1;
            if route.reserved_held {
                self.release_target_credit(route.target);
            }
            return DeliveryDecision::Abandoned;
        }

        match route.state {
            ExternalRouteState::AwaitingAck {
                mut ack_tx,
                require_ack_commit,
                wire_tracker,
            } => match reply {
                Reply::Ack {
                    action: Some(action_id),
                    ..
                } => {
                    let delivered_ack = ack_tx
                        .take()
                        .is_some_and(|tx| tx.send(Ok(reply)).is_ok());
                    if delivered_ack {
                        let target_still_live = self.is_target_live(route.target);
                        if !target_still_live {
                            // Destination closed/detached while ACK was in flight: release credit now
                            // and abandon future Terminal cleanly.
                            if route.reserved_held {
                                route.reserved_held = false;
                                self.release_target_credit(route.target);
                            }
                            route.state = ExternalRouteState::Abandoned {
                                action_id: Some(action_id),
                                wire_tracker,
                            };
                        } else if require_ack_commit {
                            route.state = ExternalRouteState::AwaitingAckCommit {
                                action_id,
                                held_terminal: None,
                                wire_tracker,
                            };
                        } else {
                            route.state = ExternalRouteState::AckCommitted { action_id };
                        }
                        self.external_routes.insert(req, route);
                        DeliveryDecision::Delivered
                    } else {
                        // Caller dropped `ack_rx` before receiving ACK: abandon terminal so
                        // terminal-without-ACK can never occur, and recover credit once.
                        if let Some(wt) = wire_tracker.as_ref() {
                            wt.mark_abandoned(action_id.0);
                        }
                        if route.reserved_held {
                            route.reserved_held = false;
                            self.release_target_credit(route.target);
                        }
                        route.state = ExternalRouteState::Abandoned {
                            action_id: Some(action_id),
                            wire_tracker,
                        };
                        self.external_routes.insert(req, route);
                        DeliveryDecision::Abandoned
                    }
                }
                Reply::Refused { .. } => {
                    let ok = ack_tx
                        .take()
                        .is_some_and(|tx| tx.send(Ok(reply)).is_ok());
                    if route.reserved_held {
                        self.release_target_credit(route.target);
                    }
                    if ok {
                        DeliveryDecision::Delivered
                    } else {
                        DeliveryDecision::Abandoned
                    }
                }
                Reply::Ack { action: None, .. } | Reply::Lease { .. } => {
                    // Malformed ACK / unexpected variant for a `starts_action()` request.
                    self.protocol_anomalies += 1;
                    let _ = ack_tx.take().map(|tx| tx.send(Ok(reply)));
                    if route.reserved_held {
                        self.release_target_credit(route.target);
                    }
                    DeliveryDecision::Abandoned
                }
                Reply::Terminal { .. } => {
                    // Protocol violation: terminal before ACK. Never deliver.
                    self.protocol_anomalies += 1;
                    self.terminals_abandoned += 1;
                    if let Some(tx) = ack_tx.take() {
                        let _ = tx.send(Err(OwnerError::Protocol));
                    }
                    if route.reserved_held {
                        self.release_target_credit(route.target);
                    }
                    DeliveryDecision::Abandoned
                }
            },
            ExternalRouteState::AwaitingAckCommit {
                action_id,
                mut held_terminal,
                wire_tracker,
            } => match reply {
                Reply::Terminal { action, .. } => {
                    if action != action_id {
                        self.mapping_mismatches += 1;
                        self.terminals_abandoned += 1;
                        if route.reserved_held {
                            self.release_target_credit(route.target);
                        }
                        return DeliveryDecision::Abandoned;
                    }
                    if held_terminal.is_some() {
                        // Duplicate terminal: drop without double delivery.
                        self.protocol_anomalies += 1;
                        self.terminals_abandoned += 1;
                        route.state = ExternalRouteState::AwaitingAckCommit {
                            action_id,
                            held_terminal,
                            wire_tracker,
                        };
                        self.external_routes.insert(req, route);
                        return DeliveryDecision::Abandoned;
                    }
                    held_terminal = Some(reply);
                    route.state = ExternalRouteState::AwaitingAckCommit {
                        action_id,
                        held_terminal,
                        wire_tracker,
                    };
                    self.external_routes.insert(req, route);
                    DeliveryDecision::Delivered
                }
                _ => {
                    self.protocol_anomalies += 1;
                    route.state = ExternalRouteState::AwaitingAckCommit {
                        action_id,
                        held_terminal,
                        wire_tracker,
                    };
                    self.external_routes.insert(req, route);
                    DeliveryDecision::Abandoned
                }
            },
            ExternalRouteState::AckCommitted { action_id } => match reply {
                Reply::Terminal { action, .. } => {
                    if action != action_id {
                        self.mapping_mismatches += 1;
                        self.terminals_abandoned += 1;
                        if route.reserved_held {
                            self.release_target_credit(route.target);
                        }
                        return DeliveryDecision::Abandoned;
                    }
                    // Route is already removed -> guarantees <= 1 terminal per admitted request.
                    if self.enqueue_routed_terminal(route, reply) {
                        DeliveryDecision::Delivered
                    } else {
                        DeliveryDecision::Abandoned
                    }
                }
                _ => {
                    self.protocol_anomalies += 1;
                    route.state = ExternalRouteState::AckCommitted { action_id };
                    self.external_routes.insert(req, route);
                    DeliveryDecision::Abandoned
                }
            },
            ExternalRouteState::Abandoned {
                action_id: _,
                wire_tracker,
            } => match reply {
                Reply::Ack {
                    action: Some(aid), ..
                } => {
                    if let Some(wt) = wire_tracker.as_ref() {
                        wt.mark_abandoned(aid.0);
                    }
                    route.state = ExternalRouteState::Abandoned {
                        action_id: Some(aid),
                        wire_tracker,
                    };
                    self.external_routes.insert(req, route);
                    DeliveryDecision::Abandoned
                }
                Reply::Terminal { action, .. } => {
                    if let Some(wt) = wire_tracker.as_ref() {
                        wt.mark_abandoned(action.0);
                    }
                    self.terminals_abandoned += 1;
                    if route.reserved_held {
                        self.release_target_credit(route.target);
                    }
                    DeliveryDecision::Abandoned
                }
                Reply::Refused { .. } | Reply::Ack { action: None, .. } | Reply::Lease { .. } => {
                    if route.reserved_held {
                        self.release_target_credit(route.target);
                    }
                    DeliveryDecision::Abandoned
                }
            },
        }
    }

    fn is_target_live(&self, target: ReplyTarget) -> bool {
        match target {
            ReplyTarget::Connection {
                client,
                slot_idx,
                slot_gen,
            } => self.slots[slot_idx].as_ref().is_some_and(|s| {
                s.slot_gen == slot_gen && s.client == client && !s.closed.load(Ordering::Acquire)
            }),
            ReplyTarget::GlobalSink { client, sink_gen } => {
                self.global_sink.is_some()
                    && self.global_sink_gen == sink_gen
                    && !self.closed_clients.contains(&client)
            }
        }
    }

    /// Moves a confirmed `Reply::Terminal` into its bound destination's pre-reserved pending queue.
    /// Returns `true` if queued for delivery, `false` if destination was closed/stale (abandoned).
    fn enqueue_routed_terminal(&mut self, route: ExternalActionRoute, reply: Reply) -> bool {
        match route.target {
            ReplyTarget::Connection {
                client,
                slot_idx,
                slot_gen,
            } => {
                if let Some(slot) = self.slots[slot_idx].as_mut() {
                    if slot.slot_gen == slot_gen
                        && slot.client == client
                        && !slot.closed.load(Ordering::Acquire)
                        && route.reserved_held
                    {
                        slot.pending.push_back(reply);
                        return true;
                    }
                }
                self.terminals_abandoned += 1;
                if route.reserved_held {
                    self.release_target_credit(route.target);
                }
                false
            }
            ReplyTarget::GlobalSink { client, sink_gen } => {
                if self.global_sink.is_some()
                    && self.global_sink_gen == sink_gen
                    && !(client.0 != 0 && self.closed_clients.contains(&client))
                    && route.reserved_held
                {
                    self.next_global_seq += 1;
                    let seq = self.next_global_seq;
                    self.global_pending.push_back((seq, client, sink_gen, reply));
                    return true;
                }
                self.terminals_abandoned += 1;
                if route.reserved_held {
                    self.release_target_credit(route.target);
                }
                false
            }
        }
    }

    /// Pops the next ready delivery across per-connection slots (round-robin, skipping slots
    /// already marked backpressured in this drain pass), then `global_pending`, then
    /// `unrouted_external` (when `global_sink` is attached).
    ///
    /// Keeps reservation credits intact on the returned [`InFlightDelivery`] so ownership transfers
    /// cleanly to the drain loop until [`Self::reconcile_in_flight`] runs.
    pub(crate) fn pop_next_delivery(
        &mut self,
        backpressured_slots: &mut [bool; MAX_LIVE_CONNECTIONS],
        global_backpressured: bool,
    ) -> Option<InFlightDelivery> {
        // 1. Per-connection slots (round-robin so one slow socket cannot starve others).
        let n = self.slots.len();
        for offset in 0..n {
            let idx = (self.rr_cursor + offset) % n;
            if backpressured_slots[idx] {
                continue;
            }
            if let Some(slot) = self.slots[idx].as_mut() {
                if slot.closed.load(Ordering::Acquire) {
                    continue;
                }
                if let Some(reply) = slot.pending.pop_front() {
                    self.rr_cursor = (idx + 1) % n;
                    return Some(InFlightDelivery {
                        target: ReplyTarget::Connection {
                            client: slot.client,
                            slot_idx: slot.slot_idx,
                            slot_gen: slot.slot_gen,
                        },
                        client: slot.client,
                        sink: Arc::clone(&slot.sink),
                        closed: Some(Arc::clone(&slot.closed)),
                        reply,
                        owns_reservation: true,
                    });
                }
            }
        }

        // 2. Shared global sink deliveries (`global_pending` and `unrouted_external` merged in
        //    exact broker arrival FIFO order via `seq`).
        if !global_backpressured {
            if let Some(sink) = self.global_sink.as_ref().map(Arc::clone) {
                loop {
                    let g_seq = self.global_pending.front().map(|(s, _, _, _)| *s);
                    let u_seq = self.unrouted_external.front().map(|(s, _, _)| *s);
                    let take_global = match (g_seq, u_seq) {
                        (Some(gs), Some(us)) => gs <= us,
                        (Some(_), None) => true,
                        (None, Some(_)) => false,
                        (None, None) => break,
                    };
                    if take_global {
                        let (_, client, sink_gen, reply) =
                            self.global_pending.pop_front().expect("checked front");
                        if sink_gen != self.global_sink_gen
                            || (client.0 != 0 && self.closed_clients.contains(&client))
                        {
                            self.global_reserved = self.global_reserved.saturating_sub(1);
                            self.terminals_abandoned += 1;
                            continue;
                        }
                        return Some(InFlightDelivery {
                            target: ReplyTarget::GlobalSink { client, sink_gen },
                            client,
                            sink,
                            closed: None,
                            reply,
                            owns_reservation: true,
                        });
                    } else {
                        let (_, client, reply) =
                            self.unrouted_external.pop_front().expect("checked front");
                        if client.0 != 0 && self.closed_clients.contains(&client) {
                            self.terminals_abandoned += 1;
                            continue;
                        }
                        return Some(InFlightDelivery {
                            target: ReplyTarget::GlobalSink {
                                client,
                                sink_gen: self.global_sink_gen,
                            },
                            client,
                            sink,
                            closed: None,
                            reply,
                            owns_reservation: false,
                        });
                    }
                }
            }
        }

        None
    }

    /// Reconciles one completed [`InFlightDelivery`] strictly by `(slot_idx, slot_gen)` or
    /// `sink_gen`, guaranteeing:
    /// - Reused slots never receive a requeued reply or have their `reserved` decremented.
    /// - Every reservation credit is released exactly once.
    pub(crate) fn reconcile_in_flight(
        &mut self,
        item: InFlightDelivery,
        decision: DeliveryDecision,
    ) {
        match item.target {
            ReplyTarget::Connection {
                client,
                slot_idx,
                slot_gen,
            } => {
                let matching_open_slot = self.slots[slot_idx].as_mut().filter(|s| {
                    s.slot_gen == slot_gen
                        && s.client == client
                        && !s.closed.load(Ordering::Acquire)
                });
                match (decision, matching_open_slot) {
                    (DeliveryDecision::Backpressured, Some(slot)) => {
                        slot.pending.push_front(item.reply);
                    }
                    (DeliveryDecision::Backpressured, None) => {
                        if item.owns_reservation {
                            self.global_reserved = self.global_reserved.saturating_sub(1);
                        }
                        self.terminals_abandoned += 1;
                    }
                    (DeliveryDecision::Delivered, Some(slot)) => {
                        if item.owns_reservation {
                            slot.reserved = slot.reserved.saturating_sub(1);
                            self.global_reserved = self.global_reserved.saturating_sub(1);
                        }
                        self.terminals_delivered += 1;
                    }
                    (DeliveryDecision::Abandoned, Some(slot)) => {
                        if item.owns_reservation {
                            slot.reserved = slot.reserved.saturating_sub(1);
                            self.global_reserved = self.global_reserved.saturating_sub(1);
                        }
                        self.terminals_abandoned += 1;
                    }
                    (DeliveryDecision::Delivered | DeliveryDecision::Abandoned, None) => {
                        if item.owns_reservation {
                            self.global_reserved = self.global_reserved.saturating_sub(1);
                        }
                        if decision == DeliveryDecision::Delivered {
                            self.terminals_delivered += 1;
                        } else {
                            self.terminals_abandoned += 1;
                        }
                    }
                }
            }
            ReplyTarget::GlobalSink { client, sink_gen } => {
                let live_global = self.global_sink.is_some()
                    && self.global_sink_gen == sink_gen
                    && !self.closed_clients.contains(&client);
                match (decision, live_global) {
                    (DeliveryDecision::Backpressured, true) => {
                        if item.owns_reservation {
                            self.global_pending
                                .push_front((0, client, sink_gen, item.reply));
                        } else {
                            self.unrouted_external.push_front((0, client, item.reply));
                        }
                    }
                    (DeliveryDecision::Backpressured, false) => {
                        if item.owns_reservation {
                            self.global_reserved = self.global_reserved.saturating_sub(1);
                        }
                        self.terminals_abandoned += 1;
                    }
                    (DeliveryDecision::Delivered, _) => {
                        if item.owns_reservation {
                            self.global_reserved = self.global_reserved.saturating_sub(1);
                        }
                        self.terminals_delivered += 1;
                    }
                    (DeliveryDecision::Abandoned, _) => {
                        if item.owns_reservation {
                            self.global_reserved = self.global_reserved.saturating_sub(1);
                        }
                        self.terminals_abandoned += 1;
                    }
                }
            }
        }
    }

    /// Pops one unrouted reply (from raw `broker.submit` fixtures) for `poll_external_replies`.
    /// Never steals replies bound to a per-connection `ConnSlot` or `GlobalSink`.
    pub(crate) fn pop_unrouted(&mut self) -> Option<(ClientId, Reply)> {
        while let Some((_, client, reply)) = self.unrouted_external.pop_front() {
            if client.0 != 0 && self.closed_clients.contains(&client) {
                self.terminals_abandoned += 1;
                continue;
            }
            return Some((client, reply));
        }
        None
    }

    pub(crate) fn push_front_unrouted(&mut self, client: ClientId, reply: Reply) {
        if !(client.0 != 0 && self.closed_clients.contains(&client)) {
            self.unrouted_external.push_front((0, client, reply));
        } else {
            self.terminals_abandoned += 1;
        }
    }

    /// Immediately wakes every `AwaitingAck` route and clears `internal_waiters` on shutdown
    /// (safe to run without waiting on `SerialDrainGate` so callers never hang behind a blocked
    /// sink callback).
    pub(crate) fn fail_pending_waiters_on_stop(&mut self) {
        self.internal_waiters.clear();
        let reqs: Vec<RequestId> = self.external_routes.keys().copied().collect();
        for req in reqs {
            if let Some(route) = self.external_routes.get_mut(&req) {
                if let ExternalRouteState::AwaitingAck { ack_tx, .. } = &mut route.state {
                    if let Some(tx) = ack_tx.take() {
                        let _ = tx.send(Err(OwnerError::Stopped));
                    }
                }
            }
        }
    }

    /// Final stopped-state cleanup after the last delivery pass (or on emergency teardown).
    /// Marks all slots closed, discards remaining routes and backpressured queues, and recovers
    /// every non-in-flight reservation credit.
    pub(crate) fn finalize_stopped(&mut self, has_in_flight_delivery: bool) {
        self.fail_pending_waiters_on_stop();
        for (_, mut route) in self.external_routes.drain() {
            if route.reserved_held {
                route.reserved_held = false;
                self.global_reserved = self.global_reserved.saturating_sub(1);
            }
            if let ExternalRouteState::AwaitingAckCommit {
                held_terminal: Some(_),
                ..
            } = route.state
            {
                self.terminals_abandoned += 1;
            }
        }
        for slot in self.slots.iter_mut() {
            if let Some(mut s) = slot.take() {
                s.closed.store(true, Ordering::Release);
                let q = s.pending.len();
                s.pending.clear();
                self.global_reserved = self.global_reserved.saturating_sub(q);
                self.terminals_abandoned += q as u64;
            }
        }
        self.client_to_slot.clear();
        let gq = self.global_pending.len();
        self.global_pending.clear();
        self.global_reserved = self.global_reserved.saturating_sub(gq);
        self.terminals_abandoned += gq as u64;
        let uq = self.unrouted_external.len();
        self.unrouted_external.clear();
        self.terminals_abandoned += uq as u64;
        if !has_in_flight_delivery {
            self.global_reserved = 0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rt_core::broker::Outcome;

    #[test]
    fn slot_reuse_never_delivers_stale_reply_or_corrupts_new_slot_credits() {
        let mut router = ActionReplyRouter::new(8);
        let delivered_a = Arc::new(Mutex::new(Vec::<Reply>::new()));
        let delivered_b = Arc::new(Mutex::new(Vec::<Reply>::new()));
        let da = Arc::clone(&delivered_a);
        let db = Arc::clone(&delivered_b);

        let tok_a = router
            .register_connection(
                ClientId(10),
                Arc::new(move |_, r| {
                    da.lock().unwrap().push(r);
                    DeliveryDecision::Backpressured
                }),
                Some(2),
            )
            .unwrap();
        assert_eq!(tok_a.slot_idx, 0);

        let target_a = router
            .check_external_admission(ClientId(10), ExternalDestinationPolicy::ConnectionOnly)
            .unwrap();
        let (tx, mut rx) = oneshot::channel();
        router.commit_external_route(RequestId(1), ClientId(10), target_a, tx, false, None);
        assert_eq!(router.stats().global_reserved, 1);

        // ACK -> Terminal queued in slot 0.
        assert_eq!(
            router.route_broker_reply(
                ClientId(10),
                Reply::Ack {
                    request: RequestId(1),
                    commit_seq: 1,
                    action: Some(ActionId(77)),
                }
            ),
            DeliveryDecision::Delivered
        );
        assert!(rx.try_recv().is_ok());
        assert_eq!(
            router.route_broker_reply(
                ClientId(10),
                Reply::Terminal {
                    request: RequestId(1),
                    action: ActionId(77),
                    outcome: Outcome::Completed,
                    children: 0,
                }
            ),
            DeliveryDecision::Delivered
        );

        // Pop in-flight delivery for A.
        let mut bp = [false; MAX_LIVE_CONNECTIONS];
        let in_flight = router.pop_next_delivery(&mut bp, false).unwrap();
        assert_eq!(router.stats().global_reserved, 1);

        // Close A and immediately open B reusing slot_idx == 0 with strictly larger slot_gen.
        router.close_connection(ClientId(10));
        assert_eq!(router.stats().global_reserved, 1, "in-flight delivery still owns its 1 credit");
        let tok_b = router
            .register_connection(
                ClientId(11),
                Arc::new(move |_, r| {
                    db.lock().unwrap().push(r);
                    DeliveryDecision::Delivered
                }),
                Some(2),
            )
            .unwrap();
        assert_eq!(tok_b.slot_idx, tok_a.slot_idx, "slot_idx reused");
        assert!(tok_b.slot_gen > tok_a.slot_gen, "slot_gen must strictly increase");

        // Reconcile A's backpressured in-flight delivery: must NOT requeue into B or corrupt B's reserved!
        assert!(in_flight.closed.as_ref().unwrap().load(Ordering::Acquire));
        router.reconcile_in_flight(in_flight, DeliveryDecision::Backpressured);
        assert_eq!(router.stats().global_reserved, 0);
        assert!(delivered_b.lock().unwrap().is_empty());
        assert_eq!(router.slots[tok_b.slot_idx].as_ref().unwrap().reserved, 0);
        assert!(router.slots[tok_b.slot_idx].as_ref().unwrap().pending.is_empty());
    }

    #[test]
    fn ack_commit_barrier_holds_terminal_and_abandon_recovers_credit_once() {
        let router = Arc::new(Mutex::new(ActionReplyRouter::new(8)));
        let tracker = Arc::new(WsWireAckTracker::new());
        let delivered = Arc::new(Mutex::new(Vec::<Reply>::new()));
        let dc = Arc::clone(&delivered);

        {
            let mut r = router.lock().unwrap();
            r.register_connection(
                ClientId(20),
                Arc::new(move |_, rep| {
                    dc.lock().unwrap().push(rep);
                    DeliveryDecision::Delivered
                }),
                Some(2),
            )
            .unwrap();
            let target = r
                .check_external_admission(ClientId(20), ExternalDestinationPolicy::ConnectionOnly)
                .unwrap();
            let (tx, _rx) = oneshot::channel();
            r.commit_external_route(
                RequestId(5),
                ClientId(20),
                target,
                tx,
                true,
                Some(Arc::clone(&tracker)),
            );
            assert_eq!(
                r.route_broker_reply(
                    ClientId(20),
                    Reply::Ack {
                        request: RequestId(5),
                        commit_seq: 1,
                        action: Some(ActionId(55)),
                    }
                ),
                DeliveryDecision::Delivered
            );
            // Terminal arrives BEFORE wire ACK enqueue is confirmed -> held in AwaitingAckCommit!
            assert_eq!(
                r.route_broker_reply(
                    ClientId(20),
                    Reply::Terminal {
                        request: RequestId(5),
                        action: ActionId(55),
                        outcome: Outcome::Completed,
                        children: 0,
                    }
                ),
                DeliveryDecision::Delivered
            );
            let mut bp = [false; MAX_LIVE_CONNECTIONS];
            assert!(
                r.pop_next_delivery(&mut bp, false).is_none(),
                "terminal must not be delivered before wire ACK enqueue confirmation"
            );
        }

        // Drop AckCommitGuard unconfirmed -> discards held terminal, marks tracker, recovers credit!
        let guard = AckCommitGuard::new(Arc::clone(&router), RequestId(5), Some(Arc::clone(&tracker)), true);
        drop(guard);

        assert!(tracker.is_abandoned(55));
        let mut r = router.lock().unwrap();
        assert_eq!(r.stats().global_reserved, 0);
        assert_eq!(r.stats().active_external_routes, 0);
        let mut bp = [false; MAX_LIVE_CONNECTIONS];
        assert!(r.pop_next_delivery(&mut bp, false).is_none());
        assert!(delivered.lock().unwrap().is_empty());
    }

    #[test]
    fn mapping_mismatch_and_malformed_replies_recover_credits() {
        let mut router = ActionReplyRouter::new(8);
        router
            .register_connection(
                ClientId(30),
                Arc::new(|_, _| DeliveryDecision::Delivered),
                Some(4),
            )
            .unwrap();

        // 1. Wrong action ID on terminal -> mapping_mismatch, credit recovered.
        let t1 = router
            .check_external_admission(ClientId(30), ExternalDestinationPolicy::ConnectionOnly)
            .unwrap();
        let (tx1, _rx1) = oneshot::channel();
        router.commit_external_route(RequestId(1), ClientId(30), t1, tx1, false, None);
        router.route_broker_reply(
            ClientId(30),
            Reply::Ack {
                request: RequestId(1),
                commit_seq: 1,
                action: Some(ActionId(100)),
            },
        );
        assert_eq!(
            router.route_broker_reply(
                ClientId(30),
                Reply::Terminal {
                    request: RequestId(1),
                    action: ActionId(999),
                    outcome: Outcome::Completed,
                    children: 0,
                }
            ),
            DeliveryDecision::Abandoned
        );
        assert_eq!(router.stats().mapping_mismatches, 1);
        assert_eq!(router.stats().global_reserved, 0);

        // 2. Terminal before ACK -> protocol_anomaly, credit recovered.
        let t2 = router
            .check_external_admission(ClientId(30), ExternalDestinationPolicy::ConnectionOnly)
            .unwrap();
        let (tx2, mut rx2) = oneshot::channel();
        router.commit_external_route(RequestId(2), ClientId(30), t2, tx2, false, None);
        assert_eq!(
            router.route_broker_reply(
                ClientId(30),
                Reply::Terminal {
                    request: RequestId(2),
                    action: ActionId(101),
                    outcome: Outcome::Completed,
                    children: 0,
                }
            ),
            DeliveryDecision::Abandoned
        );
        assert!(matches!(rx2.try_recv(), Ok(Err(OwnerError::Protocol))));
        assert_eq!(router.stats().protocol_anomalies, 1);
        assert_eq!(router.stats().global_reserved, 0);

        // 3. Malformed Ack { action: None } -> protocol_anomaly, credit recovered.
        let t3 = router
            .check_external_admission(ClientId(30), ExternalDestinationPolicy::ConnectionOnly)
            .unwrap();
        let (tx3, _rx3) = oneshot::channel();
        router.commit_external_route(RequestId(3), ClientId(30), t3, tx3, false, None);
        assert_eq!(
            router.route_broker_reply(
                ClientId(30),
                Reply::Ack {
                    request: RequestId(3),
                    commit_seq: 2,
                    action: None,
                }
            ),
            DeliveryDecision::Abandoned
        );
        assert_eq!(router.stats().protocol_anomalies, 2);
        assert_eq!(router.stats().global_reserved, 0);
    }

    #[test]
    fn serial_drain_gate_handles_same_thread_reentry_and_deferred_shutdown() {
        let gate = SerialDrainGate::new();
        assert!(matches!(gate.enter(), DrainEnter::Entered));
        // Same-thread re-entrant pump -> SameThreadReentrant (no deadlock).
        assert!(matches!(gate.enter(), DrainEnter::SameThreadReentrant));
        // Same-thread re-entrant shutdown -> defers finalization cleanly.
        assert!(matches!(
            gate.enter_for_shutdown(None, true),
            DrainEnter::SameThreadReentrant
        ));
        // Outer owner atomically picks up deferred finalization before releasing gate.
        assert_eq!(gate.try_release_or_take_deferred(), Some(true));
        assert_eq!(gate.try_release_or_take_deferred(), None);
    }
}
