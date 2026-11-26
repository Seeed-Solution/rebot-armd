//! Release-side session lifecycle driver extending PR P macros (executor services layer, item 2).
//!
//! Two independent preallocated inline slots (0 RT heap allocs, 0 `SupervisorLock`):
//! - [`SessionSlot::ExclusiveOutput`]: exclusive arm/hardware output session (C-F1 commissioning,
//!   future mode switch, gripper zero, drag teach, MIT handover, or `MacroBacked` co-owner).
//! - [`SessionSlot::Recorder`]: separate singleton recorder slot admitting/stepping independently.
//!
//! Invariants:
//! 1. [`admit_session`] consumes an A1 [`Admitted`] token and binds authority from [`AdmittedProof`]
//!    plus fail-closed owner policy (`lease_enabled`, `operator_auth_enforced`).
//! 2. Pre-step and pre-TX checks re-verify SAFE-HOLD, fault fence, E-stop/epoch, stop generation,
//!    connection liveness, lease, operator grant, and session/child deadlines before any bus TX.
//! 3. [`SessionDriver::earliest_output_deadline`] feeds `active_action_deadline` ->
//!    `install_effective_tx_cutoff` (stopped/`FailedHold` safety holds remain unclipped).
//! 4. Each session emits at most one [`SessionOutcome`]; router-bound terminals stay retained inline
//!    until flushed strictly after Step 6 `core.publish` into the preallocated SPSC ring, blocking
//!    slot reuse if the ring is full.
//! 5. [`SessionDriver::deliver_worker_reply`] checks `(session, step_seq, generation)` + live
//!    authority + live deadline before one-shot `AwaitingWorker -> Running` transition.
//! 6. Energized uncertain terminals with `escalate_safe_hold_on_uncertain` set `safe_hold_requested`
//!    and fence the same release (skipping Steps 3–5 and entering SAFE-HOLD without disarming).

use std::time::Instant;

use super::broker::{ActionId, BootId, ClientId, Outcome, RequestId};
use super::ids::ArmId;
use super::stream::{StreamConsumer, StreamProducer, StreamPushError};
use crate::safety::SafetyError;
use crate::supervisor::{Admitted, AdmittedProof, SupervisorInner};

pub const SESSION_TERMINAL_RING_CAPACITY: usize = 32;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SessionId(pub u64);

impl SessionId {
    #[inline]
    pub fn is_valid(self) -> bool { self.0 != 0 }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SessionSlot {
    ExclusiveOutput,
    Recorder,
}

impl SessionSlot {
    #[inline]
    const fn index(self) -> usize {
        match self {
            Self::ExclusiveOutput => 0,
            Self::Recorder => 1,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SessionKind {
    Commissioning,
    ModeSwitch,
    GripperZeroCheck,
    Recorder,
    MacroBacked,
    Custom(u16),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct WorkerTicket {
    pub session: SessionId,
    pub step_seq: u32,
    pub generation: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WorkerReply {
    pub slot: SessionSlot,
    pub ticket: WorkerTicket,
    pub ok: bool,
    pub code: &'static str,
    pub value: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WorkerReplyError {
    Stale,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RevocationScope {
    /// Ordinary motion stop (`STOPPED`): revokes only sessions with `revoke_on_stop`.
    MotionStop,
    /// E-stop or epoch bump (`ESTOP_ACTIVE`, `EPOCH_MISMATCH`): revokes only sessions with `revoke_on_epoch_change`.
    EpochOrEStop,
    /// Motion lease loss / preempt (`LEASE_LOST`): revokes only sessions with `require_lease`.
    LeaseLoss,
    /// Fatal fence or lifecycle teardown (`SAFE_HOLD`, `INTERNAL_FAULT`, `shutdown`, `handback`,
    /// `startup_timeout`): revokes every live slot including `Recorder`.
    FatalOrTeardown,
}

impl RevocationScope {
    #[inline]
    pub fn from_stop_reason(reason: &'static str) -> Self {
        match reason {
            "STOPPED" => Self::MotionStop,
            "ESTOP_ACTIVE" | "EPOCH_MISMATCH" => Self::EpochOrEStop,
            "LEASE_LOST" => Self::LeaseLoss,
            _ => Self::FatalOrTeardown,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SessionAuthority {
    pub baseline: super::authority::AuthorityBaseline,
    pub connection_id: u64,
    pub lease_generation: u64,
    pub epoch: u64,
    pub stop_generation: u64,
    pub require_lease: bool,
    pub operator: crate::auth::OperatorRequirement,
    pub revoke_on_stop: bool,
    pub revoke_on_epoch_change: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SessionTerminalBinding {
    /// Explicit internal / autonomous session with no external routed terminal reservation.
    Internal,
    /// Routed external session bound to one reserved P-F1 terminal credit.
    Routed {
        request: RequestId,
        route_gen: u64,
        client: ClientId,
        action_id: ActionId,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PreparedSession {
    pub slot: SessionSlot,
    pub kind: SessionKind,
    pub deadline: Option<Instant>,
    pub child_deadline: Option<Instant>,
    pub timeout_code: &'static str,
    pub child_timeout_code: &'static str,
    pub action_id: Option<ActionId>,
    pub client: ClientId,
    pub require_lease: bool,
    pub operator: crate::auth::OperatorRequirement,
    pub revoke_on_stop: bool,
    pub revoke_on_epoch_change: bool,
    pub escalate_safe_hold_on_uncertain: bool,
}

impl PreparedSession {
    pub fn exclusive(kind: SessionKind, client: ClientId) -> Self {
        Self {
            slot: SessionSlot::ExclusiveOutput,
            kind,
            deadline: None,
            child_deadline: None,
            timeout_code: "TIMEOUT",
            child_timeout_code: "TIMEOUT",
            action_id: None,
            client,
            require_lease: true,
            operator: crate::auth::OperatorRequirement::NotRequired,
            revoke_on_stop: true,
            revoke_on_epoch_change: true,
            escalate_safe_hold_on_uncertain: true,
        }
    }

    pub fn recorder(client: ClientId) -> Self {
        Self {
            slot: SessionSlot::Recorder,
            kind: SessionKind::Recorder,
            deadline: None,
            child_deadline: None,
            timeout_code: "TIMEOUT",
            child_timeout_code: "TIMEOUT",
            action_id: None,
            client,
            require_lease: false,
            operator: crate::auth::OperatorRequirement::NotRequired,
            revoke_on_stop: false,
            revoke_on_epoch_change: false,
            escalate_safe_hold_on_uncertain: false,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SessionAdmitError {
    SlotBusy,
    OutputBusy(&'static str),
    DeadlineExpired,
    InvalidSlotKind,
    InvalidReservation,
    ReservationCancelled,
    AuthorityRejected(&'static str),
}

impl SessionAdmitError {
    pub fn code(self) -> &'static str {
        match self {
            Self::SlotBusy => "SESSION_BUSY",
            Self::OutputBusy(c) | Self::AuthorityRejected(c) => c,
            Self::DeadlineExpired => "DEADLINE_EXPIRED",
            Self::InvalidSlotKind => "INVALID_COMMAND",
            Self::InvalidReservation => "NO_DESTINATION",
            Self::ReservationCancelled => "CANCELLED",
        }
    }

    fn from_safety(err: SafetyError) -> Self {
        match err {
            SafetyError::EStopActive => Self::AuthorityRejected("ESTOP_ACTIVE"),
            SafetyError::EpochMismatch { .. } => Self::AuthorityRejected("EPOCH_MISMATCH"),
            SafetyError::LeaseLost => Self::AuthorityRejected("LEASE_LOST"),
            SafetyError::Stopped => Self::AuthorityRejected("STOPPED"),
            SafetyError::MotionBusy => Self::OutputBusy("MOTION_BUSY"),
            _ => Self::AuthorityRejected("REJECTED"),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SessionTerminalReason {
    Completed,
    Failed(&'static str),
    Cancelled(&'static str),
    DeadlineExpired(&'static str),
    Revoked(&'static str),
    Superseded,
    SafeHoldUncertain(&'static str),
}

impl SessionTerminalReason {
    pub fn code(self) -> &'static str {
        match self {
            Self::Completed => "",
            Self::Failed(c)
            | Self::Cancelled(c)
            | Self::DeadlineExpired(c)
            | Self::Revoked(c)
            | Self::SafeHoldUncertain(c) => c,
            Self::Superseded => "SUPERSEDED",
        }
    }

    pub fn to_broker_outcome(self) -> Outcome {
        match self {
            Self::Completed => Outcome::Completed,
            Self::Failed(c) | Self::DeadlineExpired(c) => Outcome::Failed(c),
            Self::Superseded => Outcome::Superseded,
            Self::Cancelled("STOPPED") | Self::Revoked("STOPPED") => Outcome::Stopped,
            Self::Cancelled(c) | Self::Revoked(c) | Self::SafeHoldUncertain(c) => Outcome::Cancelled(c),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SessionOutcome {
    pub session: SessionId,
    pub slot: SessionSlot,
    pub reason: SessionTerminalReason,
    pub uncertain_hw_state: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SessionStep {
    Pending,
    AwaitWorker(WorkerTicket),
    Terminal(SessionOutcome),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhaseStepOutcome {
    Pending,
    AwaitWorker,
    Terminal { reason: SessionTerminalReason, uncertain_hw_state: bool },
}

/// Immutable per-release context with caller's already-margin-adjusted `base_tx_cutoff`.
#[derive(Debug, Clone, Copy)]
pub struct ReleaseCtx {
    pub arm: ArmId,
    pub boot: BootId,
    pub owner_generation: u64,
    pub now: Instant,
    pub release_deadline: Instant,
    pub base_tx_cutoff: Instant,
    pub epoch: u64,
    pub stop_generation: u64,
    pub estop_latched: bool,
    pub urgent_stop_pending: bool,
    pub internal_fault_fenced: bool,
    pub safe_hold_active: bool,
}

/// Compact `Copy` record transferred from RT to off-RT P-F1 router strictly after Step 6 publish.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SessionTerminalRecord {
    pub session: SessionId,
    pub slot: SessionSlot,
    pub request: RequestId,
    pub route_gen: u64,
    pub client: ClientId,
    pub action_id: ActionId,
    pub outcome: SessionOutcome,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SessionState {
    Running,
    AwaitingWorker(WorkerTicket),
    Terminal(SessionOutcome),
}

#[derive(Debug, Clone, Copy)]
pub struct ActiveSession {
    pub id: SessionId,
    pub slot: SessionSlot,
    pub kind: SessionKind,
    pub authority: SessionAuthority,
    pub deadline: Option<Instant>,
    pub child_deadline: Option<Instant>,
    pub timeout_code: &'static str,
    pub child_timeout_code: &'static str,
    pub action_id: Option<ActionId>,
    pub terminal: SessionTerminalBinding,
    pub client: ClientId,
    pub escalate_safe_hold_on_uncertain: bool,
    pub hw_state_uncertain: bool,
    pub state: SessionState,
    pub step_seq: u32,
    pub terminal_emitted: bool,
    pub last_worker_reply: Option<WorkerReply>,
}

impl ActiveSession {
    #[inline]
    pub fn is_live(&self) -> bool { !matches!(self.state, SessionState::Terminal(_)) }

    #[inline]
    pub fn effective_deadline(&self) -> Option<Instant> {
        if !self.is_live() { return None; }
        match (self.deadline, self.child_deadline) {
            (Some(s), Some(c)) => Some(s.min(c)),
            (Some(s), None) => Some(s),
            (None, Some(c)) => Some(c),
            (None, None) => None,
        }
    }

    #[inline]
    pub fn expired_code(&self, now: Instant) -> Option<&'static str> {
        if !self.is_live() { return None; }
        if self.deadline.is_some_and(|dl| now >= dl) {
            return Some(if self.timeout_code.is_empty() { "TIMEOUT" } else { self.timeout_code });
        }
        if self.child_deadline.is_some_and(|dl| now >= dl) {
            return Some(if self.child_timeout_code.is_empty() { "TIMEOUT" } else { self.child_timeout_code });
        }
        None
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SessionDriverStats {
    pub terminals_emitted: u64,
    pub duplicate_terminals_suppressed: u64,
    pub stale_worker_replies_rejected: u64,
    pub pending_terminal_flushes: usize,
    pub safe_hold_escalations: u64,
}

/// Preallocated, zero-alloc, zero-lock RT session lifecycle driver embedded in `SupervisorInner`.
pub struct SessionDriver {
    next_id: u64,
    next_worker_gen: u64,
    slots: [Option<ActiveSession>; 2],
    pending_terminals: [Option<SessionTerminalRecord>; 2],
    terminal_tx: Option<StreamProducer<SessionTerminalRecord>>,
    safe_hold_requested: bool,
    terminals_emitted: u64,
    duplicate_terminals_suppressed: u64,
    stale_worker_replies_rejected: u64,
    safe_hold_escalations: u64,
}

impl Default for SessionDriver {
    fn default() -> Self { Self::new() }
}

impl SessionDriver {
    pub fn new() -> Self {
        Self {
            next_id: 1,
            next_worker_gen: 1,
            slots: [None, None],
            pending_terminals: [None, None],
            terminal_tx: None,
            safe_hold_requested: false,
            terminals_emitted: 0,
            duplicate_terminals_suppressed: 0,
            stale_worker_replies_rejected: 0,
            safe_hold_escalations: 0,
        }
    }

    pub fn attach_terminal_channel(&mut self) -> StreamConsumer<SessionTerminalRecord> {
        let (tx, rx) = super::stream::channel(SESSION_TERMINAL_RING_CAPACITY);
        self.terminal_tx = Some(tx);
        rx
    }

    pub fn attach_terminal_producer(&mut self, tx: StreamProducer<SessionTerminalRecord>) {
        self.terminal_tx = Some(tx);
    }

    #[inline]
    pub fn slot(&self, slot: SessionSlot) -> Option<&ActiveSession> { self.slots[slot.index()].as_ref() }

    #[inline]
    pub fn is_slot_occupied(&self, slot: SessionSlot) -> bool {
        let idx = slot.index();
        self.slots[idx].is_some() || self.pending_terminals[idx].is_some()
    }

    #[inline]
    pub fn has_pending_terminal(&self, slot: SessionSlot) -> bool { self.pending_terminals[slot.index()].is_some() }

    #[inline]
    pub fn is_safe_hold_requested(&self) -> bool { self.safe_hold_requested }

    #[inline]
    pub fn take_safe_hold_requested(&mut self) -> bool { std::mem::replace(&mut self.safe_hold_requested, false) }

    #[inline]
    pub fn request_safe_hold(&mut self) {
        if !self.safe_hold_requested {
            self.safe_hold_requested = true;
            self.safe_hold_escalations = self.safe_hold_escalations.saturating_add(1);
        }
    }

    #[inline]
    pub fn stats(&self) -> SessionDriverStats {
        SessionDriverStats {
            terminals_emitted: self.terminals_emitted,
            duplicate_terminals_suppressed: self.duplicate_terminals_suppressed,
            stale_worker_replies_rejected: self.stale_worker_replies_rejected,
            pending_terminal_flushes: self.pending_terminals.iter().filter(|p| p.is_some()).count(),
            safe_hold_escalations: self.safe_hold_escalations,
        }
    }

    #[inline]
    pub fn earliest_output_deadline(&self) -> Option<Instant> {
        self.slots[SessionSlot::ExclusiveOutput.index()]
            .as_ref()
            .and_then(ActiveSession::effective_deadline)
    }

    #[inline]
    pub fn blocks_ordinary_output(&self, macro_id: Option<ActionId>) -> bool {
        let Some(active) = self.slots[SessionSlot::ExclusiveOutput.index()].as_ref() else { return false };
        if !active.is_live() { return false; }
        !(active.kind == SessionKind::MacroBacked && macro_id.is_some() && active.action_id == macro_id)
    }

    #[inline]
    pub fn effective_tx_cutoff(&self, slot: SessionSlot, base_tx_cutoff: Instant) -> Instant {
        match self.slot(slot).and_then(ActiveSession::effective_deadline) {
            Some(dl) => base_tx_cutoff.min(dl),
            None => base_tx_cutoff,
        }
    }

    pub(crate) fn admit_with_authority(
        &mut self,
        prepared: PreparedSession,
        authority: SessionAuthority,
        terminal: SessionTerminalBinding,
        now: Instant,
        reject_expired_at_admit: bool,
    ) -> Result<SessionId, SessionAdmitError> {
        self.admit_with_authority_handoff(
            prepared,
            authority,
            terminal,
            None,
            now,
            reject_expired_at_admit,
        )
    }

    pub(crate) fn admit_with_authority_handoff(
        &mut self,
        prepared: PreparedSession,
        authority: SessionAuthority,
        terminal: SessionTerminalBinding,
        handoff: Option<&super::reply_router::SessionTerminalHandoff>,
        now: Instant,
        reject_expired_at_admit: bool,
    ) -> Result<SessionId, SessionAdmitError> {
        let invalid = (prepared.slot == SessionSlot::Recorder && prepared.kind != SessionKind::Recorder)
            || (prepared.slot == SessionSlot::ExclusiveOutput && prepared.kind == SessionKind::Recorder);
        if invalid { return Err(SessionAdmitError::InvalidSlotKind); }
        let (client, action_id) = match terminal {
            SessionTerminalBinding::Internal => (prepared.client, prepared.action_id),
            SessionTerminalBinding::Routed {
                request,
                route_gen,
                client,
                action_id,
            } => {
                if (request.0 & super::reply_router::SESSION_REQUEST_ID_BIT) == 0
                    || route_gen == 0
                    || (prepared.client.0 != 0 && prepared.client != client)
                {
                    return Err(SessionAdmitError::InvalidReservation);
                }
                (client, Some(action_id))
            }
        };
        if self.is_slot_occupied(prepared.slot) { return Err(SessionAdmitError::SlotBusy); }
        if reject_expired_at_admit
            && (prepared.deadline.is_some_and(|dl| now >= dl)
                || prepared.child_deadline.is_some_and(|dl| now >= dl))
        {
            return Err(SessionAdmitError::DeadlineExpired);
        }
        // Single atomic ownership-handoff barrier: executed strictly AFTER every admission
        // precondition succeeds and strictly BEFORE making the session slot live.
        if let Some(h) = handoff {
            if !h.try_claim_admitted() {
                return Err(SessionAdmitError::ReservationCancelled);
            }
        }
        let id = SessionId(self.next_id);
        self.next_id = self.next_id.wrapping_add(1).max(1);
        let child_deadline = match (prepared.deadline, prepared.child_deadline) {
            (Some(s), Some(c)) => Some(s.min(c)),
            (_, c) => c,
        };
        self.slots[prepared.slot.index()] = Some(ActiveSession {
            id,
            slot: prepared.slot,
            kind: prepared.kind,
            authority,
            deadline: prepared.deadline,
            child_deadline,
            timeout_code: prepared.timeout_code,
            child_timeout_code: prepared.child_timeout_code,
            action_id,
            terminal,
            client,
            escalate_safe_hold_on_uncertain: prepared.escalate_safe_hold_on_uncertain,
            hw_state_uncertain: false,
            state: SessionState::Running,
            step_seq: 0,
            terminal_emitted: false,
            last_worker_reply: None,
        });
        Ok(id)
    }

    pub fn set_hw_state_uncertain(&mut self, slot: SessionSlot, id: SessionId, uncertain: bool) -> bool {
        let Some(active) = self.slots[slot.index()].as_mut() else { return false };
        if active.id != id || !active.is_live() { return false; }
        active.hw_state_uncertain = uncertain;
        true
    }

    pub fn arm_child_deadline(
        &mut self,
        slot: SessionSlot,
        id: SessionId,
        child_deadline: Option<Instant>,
        timeout_code: &'static str,
    ) -> bool {
        let Some(active) = self.slots[slot.index()].as_mut() else { return false };
        if active.id != id || !active.is_live() { return false; }
        active.child_deadline = match (active.deadline, child_deadline) {
            (Some(s), Some(c)) => Some(s.min(c)),
            (_, c) => c,
        };
        if !timeout_code.is_empty() { active.child_timeout_code = timeout_code; }
        true
    }

    /// Transitions a live `Running` session directly into `AwaitingWorker(ticket)` with an
    /// optional updated `child_deadline` and returns its unique [`WorkerTicket`].
    pub fn enter_await_worker(
        &mut self,
        slot: SessionSlot,
        id: SessionId,
        child_deadline: Option<Instant>,
        timeout_code: &'static str,
    ) -> Option<WorkerTicket> {
        let active = self.slots[slot.index()].as_mut()?;
        if active.id != id || !matches!(active.state, SessionState::Running) {
            return None;
        }
        active.child_deadline = match (active.deadline, child_deadline) {
            (Some(s), Some(c)) => Some(s.min(c)),
            (_, c) => c,
        };
        if !timeout_code.is_empty() {
            active.child_timeout_code = timeout_code;
        }
        active.step_seq = active.step_seq.saturating_add(1);
        let generation = self.next_worker_gen;
        self.next_worker_gen = generation.wrapping_add(1).max(1);
        let ticket = WorkerTicket {
            session: id,
            step_seq: active.step_seq,
            generation,
        };
        active.state = SessionState::AwaitingWorker(ticket);
        active.last_worker_reply = None;
        Some(ticket)
    }

    pub fn check_live_authority(
        &self,
        slot: SessionSlot,
        id: SessionId,
        ctx: &ReleaseCtx,
        is_conn_closed: impl Fn(u64) -> bool,
        is_lease_valid: impl Fn(u64, u64, Instant) -> bool,
        is_operator_valid: impl Fn(crate::auth::OperatorRequirement, Instant) -> bool,
    ) -> Result<(), SessionTerminalReason> {
        let Some(active) = self.slot(slot).filter(|a| a.id == id) else {
            return Err(SessionTerminalReason::Cancelled("CANCELLED"));
        };
        if !active.is_live() {
            return Err(SessionTerminalReason::Cancelled("CANCELLED"));
        }
        if ctx.safe_hold_active { return Err(SessionTerminalReason::Revoked("SAFE_HOLD")); }
        if ctx.internal_fault_fenced { return Err(SessionTerminalReason::Revoked("INTERNAL_FAULT")); }
        let auth = active.authority;
        if ctx.owner_generation != 0
            && auth.baseline.owner_generation != 0
            && ctx.owner_generation != auth.baseline.owner_generation
        {
            return Err(SessionTerminalReason::Revoked("OWNER_CHANGED"));
        }
        if auth.revoke_on_epoch_change && (ctx.estop_latched || ctx.epoch != auth.epoch) {
            return Err(SessionTerminalReason::Revoked("EPOCH_MISMATCH"));
        }
        if auth.revoke_on_stop && (ctx.urgent_stop_pending || ctx.stop_generation != auth.stop_generation) {
            return Err(SessionTerminalReason::Revoked("STOPPED"));
        }
        if auth.connection_id != 0 && is_conn_closed(auth.connection_id) {
            return Err(SessionTerminalReason::Revoked(if auth.require_lease { "LEASE_LOST" } else { "DISCONNECTED" }));
        }
        if auth.require_lease && !is_lease_valid(auth.connection_id, auth.lease_generation, ctx.now) {
            return Err(SessionTerminalReason::Revoked("LEASE_LOST"));
        }
        if !is_operator_valid(auth.operator, ctx.now) {
            return Err(SessionTerminalReason::Revoked("OPERATOR_REQUIRED"));
        }
        if let Some(code) = active.expired_code(ctx.now) {
            return Err(SessionTerminalReason::DeadlineExpired(code));
        }
        Ok(())
    }

    pub fn finish_slot(
        &mut self,
        slot: SessionSlot,
        id: SessionId,
        reason: SessionTerminalReason,
        uncertain_hw_state: bool,
    ) -> Option<SessionOutcome> {
        let idx = slot.index();
        let active = self.slots[idx].as_mut()?;
        if active.id != id { return None; }
        if active.terminal_emitted || !active.is_live() {
            self.duplicate_terminals_suppressed = self.duplicate_terminals_suppressed.saturating_add(1);
            return None;
        }
        let outcome = SessionOutcome { session: id, slot, reason, uncertain_hw_state };
        active.state = SessionState::Terminal(outcome);
        active.terminal_emitted = true;
        self.terminals_emitted = self.terminals_emitted.saturating_add(1);
        if let SessionTerminalBinding::Routed {
            request,
            route_gen,
            client,
            action_id,
        } = active.terminal
        {
            self.pending_terminals[idx] = Some(SessionTerminalRecord {
                session: id,
                slot,
                request,
                route_gen,
                client,
                action_id,
                outcome,
            });
        }
        Some(outcome)
    }

    pub fn step_slot(
        &mut self,
        slot: SessionSlot,
        ctx: &ReleaseCtx,
        is_conn_closed: impl Fn(u64) -> bool,
        is_lease_valid: impl Fn(u64, u64, Instant) -> bool,
        is_operator_valid: impl Fn(crate::auth::OperatorRequirement, Instant) -> bool,
        on_pre_phase_abort: impl FnOnce(SessionId, SessionTerminalReason) -> bool,
        phase_fn: impl FnOnce(SessionId, Instant, Option<WorkerReply>) -> PhaseStepOutcome,
    ) -> Option<SessionStep> {
        let idx = slot.index();
        let (id, state) = {
            let active = self.slots[idx].as_ref()?;
            (active.id, active.state)
        };
        if let SessionState::Terminal(outcome) = state {
            self.duplicate_terminals_suppressed = self.duplicate_terminals_suppressed.saturating_add(1);
            return Some(SessionStep::Terminal(outcome));
        }
        if let Err(reason) = self.check_live_authority(slot, id, ctx, is_conn_closed, is_lease_valid, is_operator_valid) {
            let uncertain = on_pre_phase_abort(id, reason);
            let outcome = self.finish_slot(slot, id, reason, uncertain).expect("checked live above");
            return Some(SessionStep::Terminal(outcome));
        }
        if let SessionState::AwaitingWorker(ticket) = state {
            return Some(SessionStep::AwaitWorker(ticket));
        }
        let effective_cutoff = self.effective_tx_cutoff(slot, ctx.base_tx_cutoff);
        let consumed_reply = self.slots[idx]
            .as_mut()
            .and_then(|a| a.last_worker_reply.take());
        match phase_fn(id, effective_cutoff, consumed_reply) {
            PhaseStepOutcome::Pending => Some(SessionStep::Pending),
            PhaseStepOutcome::AwaitWorker => {
                let active = self.slots[idx].as_mut().expect("active above");
                active.step_seq = active.step_seq.saturating_add(1);
                let generation = self.next_worker_gen;
                self.next_worker_gen = generation.wrapping_add(1).max(1);
                let ticket = WorkerTicket { session: id, step_seq: active.step_seq, generation };
                active.state = SessionState::AwaitingWorker(ticket);
                active.last_worker_reply = None;
                Some(SessionStep::AwaitWorker(ticket))
            }
            PhaseStepOutcome::Terminal { reason, uncertain_hw_state } => {
                let outcome = self
                    .finish_slot(slot, id, reason, uncertain_hw_state)
                    .unwrap_or(SessionOutcome { session: id, slot, reason, uncertain_hw_state });
                Some(SessionStep::Terminal(outcome))
            }
        }
    }

    pub fn deliver_worker_reply(
        &mut self,
        reply: WorkerReply,
        now: Instant,
        live_authority_ok: Result<(), &'static str>,
    ) -> Result<WorkerReply, WorkerReplyError> {
        let idx = reply.slot.index();
        let Some(active) = self.slots[idx].as_mut() else {
            self.stale_worker_replies_rejected = self.stale_worker_replies_rejected.saturating_add(1);
            return Err(WorkerReplyError::Stale);
        };
        let SessionState::AwaitingWorker(expected) = active.state else {
            self.stale_worker_replies_rejected = self.stale_worker_replies_rejected.saturating_add(1);
            return Err(WorkerReplyError::Stale);
        };
        if active.id != reply.ticket.session
            || expected != reply.ticket
            || live_authority_ok.is_err()
            || active.expired_code(now).is_some()
        {
            self.stale_worker_replies_rejected = self.stale_worker_replies_rejected.saturating_add(1);
            return Err(WorkerReplyError::Stale);
        }
        active.state = SessionState::Running;
        active.step_seq = active.step_seq.saturating_add(1);
        active.last_worker_reply = Some(reply);
        Ok(reply)
    }

    pub fn cancel_slot(
        &mut self,
        slot: SessionSlot,
        id: SessionId,
        reason: SessionTerminalReason,
        uncertain_hw_state: bool,
    ) -> Option<SessionOutcome> {
        self.finish_slot(slot, id, reason, uncertain_hw_state)
    }

    pub fn revoke_live_sessions_by_scope(
        &mut self,
        reason: SessionTerminalReason,
        scope: RevocationScope,
    ) -> [Option<SessionOutcome>; 2] {
        let mut out = [None, None];
        for slot in [SessionSlot::ExclusiveOutput, SessionSlot::Recorder] {
            let live = self.slot(slot).and_then(|s| {
                let applies = match scope {
                    RevocationScope::FatalOrTeardown => true,
                    RevocationScope::MotionStop => s.authority.revoke_on_stop,
                    RevocationScope::EpochOrEStop => s.authority.revoke_on_epoch_change,
                    RevocationScope::LeaseLoss => s.authority.require_lease,
                };
                (s.is_live() && applies).then_some((s.id, s.hw_state_uncertain))
            });
            if let Some((id, uncertain)) = live {
                out[slot.index()] = self.finish_slot(slot, id, reason, uncertain);
            }
        }
        out
    }

    pub fn revoke_live_sessions(
        &mut self,
        reason: SessionTerminalReason,
        is_ordinary_stop: bool,
    ) -> [Option<SessionOutcome>; 2] {
        let scope = if is_ordinary_stop {
            RevocationScope::MotionStop
        } else {
            RevocationScope::FatalOrTeardown
        };
        self.revoke_live_sessions_by_scope(reason, scope)
    }

    pub fn revoke_for_connection(
        &mut self,
        connection_id: u64,
        reason: SessionTerminalReason,
    ) -> [Option<SessionOutcome>; 2] {
        let mut out = [None, None];
        if connection_id == 0 { return out; }
        for slot in [SessionSlot::ExclusiveOutput, SessionSlot::Recorder] {
            let live = self.slot(slot).and_then(|s| {
                (s.is_live() && s.authority.connection_id == connection_id)
                    .then_some((s.id, s.hw_state_uncertain))
            });
            if let Some((id, uncertain)) = live {
                out[slot.index()] = self.finish_slot(slot, id, reason, uncertain);
            }
        }
        out
    }

    pub fn flush_published_terminals(&mut self) -> usize {
        let Some(tx) = self.terminal_tx.as_mut() else { return 0 };
        let mut pushed = 0usize;
        for slot in &mut self.pending_terminals {
            if let Some(rec) = *slot {
                match tx.try_push(rec) {
                    Ok(_) => { *slot = None; pushed += 1; }
                    Err(StreamPushError::Full) => {}
                    Err(StreamPushError::Closed) => { *slot = None; }
                }
            }
        }
        pushed
    }

    pub fn clear_terminal_slot(&mut self, slot: SessionSlot, id: SessionId) -> bool {
        let idx = slot.index();
        let Some(active) = self.slots[idx].as_ref() else { return false };
        if active.id != id || active.is_live() { return false; }
        if self.terminal_tx.is_some() && self.pending_terminals[idx].is_some() { return false; }
        self.slots[idx] = None;
        self.pending_terminals[idx] = None;
        true
    }
}

/// Admits a routed session by consuming a verified A1 [`Admitted`] token and a move-only
/// [`super::reply_router::SessionTerminalReservation`] obtained before enqueue.
///
/// Validates authority, deadlines, slot occupancy, and exclusive output conflicts BEFORE atomically
/// claiming the reservation handoff (`PENDING -> ADMITTED`) and making the slot live.
///
/// Off-RT / test entry point: drops the reservation's atomic handoff handle locally on return.
/// Real-time (`armd-control`) callers MUST use [`admit_routed_retiring`] so the handoff `Arc` is
/// unconditionally parked for off-RT retirement with 0 RT heap frees.
pub fn admit_routed(
    admitted: Admitted<'_>,
    prepared: PreparedSession,
    reservation: super::reply_router::SessionTerminalReservation,
    now: Instant,
) -> Result<SessionId, SessionAdmitError> {
    let mut local_handoff = None;
    admit_routed_retiring(admitted, prepared, reservation, &mut local_handoff, now)
}

/// RT-safe routed session admission that unconditionally parks the reservation's atomic handoff
/// handle in `retired_handoff` BEFORE any fallible step (including [`Admitted::consume`]), so
/// `armd-control` executes 0 heap allocations and 0 heap frees on every success and failure path.
pub fn admit_routed_retiring(
    admitted: Admitted<'_>,
    prepared: PreparedSession,
    reservation: super::reply_router::SessionTerminalReservation,
    retired_handoff: &mut Option<super::reply_router::SessionTerminalHandoff>,
    now: Instant,
) -> Result<SessionId, SessionAdmitError> {
    let (ticket, handoff) = reservation.into_parts();
    *retired_handoff = Some(handoff);
    let (inner, proof) = admitted.consume().map_err(SessionAdmitError::from_safety)?;
    inner.admit_routed_ticket_with_proof(prepared, ticket, retired_handoff.as_ref(), &proof, now)
}

/// Admits an explicit internal session (autonomous cleanup, recorder, Legacy polling-only session,
/// or action-broker-owned session) that has no external routed terminal reservation.
pub fn admit_internal(
    admitted: Admitted<'_>,
    prepared: PreparedSession,
    now: Instant,
) -> Result<SessionId, SessionAdmitError> {
    let (inner, proof) = admitted.consume().map_err(SessionAdmitError::from_safety)?;
    inner.admit_internal_with_proof(prepared, &proof, now)
}

impl SupervisorInner {
    #[inline]
    pub fn sessions(&self) -> &SessionDriver { &self.sessions }

    #[inline]
    pub fn sessions_mut(&mut self) -> &mut SessionDriver { &mut self.sessions }

    pub(crate) fn admit_routed_ticket_with_proof(
        &mut self,
        prepared: PreparedSession,
        ticket: super::reply_router::SessionTerminalTicket,
        handoff: Option<&super::reply_router::SessionTerminalHandoff>,
        proof: &AdmittedProof,
        now: Instant,
    ) -> Result<SessionId, SessionAdmitError> {
        let terminal = SessionTerminalBinding::Routed {
            request: ticket.request(),
            route_gen: ticket.route_gen(),
            client: ticket.client(),
            action_id: ticket.action_id(),
        };
        self.admit_session_inner(prepared, terminal, handoff, proof, now)
    }

    pub(crate) fn admit_internal_with_proof(
        &mut self,
        prepared: PreparedSession,
        proof: &AdmittedProof,
        now: Instant,
    ) -> Result<SessionId, SessionAdmitError> {
        self.admit_session_inner(prepared, SessionTerminalBinding::Internal, None, proof, now)
    }

    fn admit_session_inner(
        &mut self,
        prepared: PreparedSession,
        terminal: SessionTerminalBinding,
        handoff: Option<&super::reply_router::SessionTerminalHandoff>,
        proof: &AdmittedProof,
        now: Instant,
    ) -> Result<SessionId, SessionAdmitError> {
        if prepared.slot == SessionSlot::ExclusiveOutput {
            if let Some(code) = self.exclusive_output_conflict_for(prepared.kind, prepared.action_id) {
                return Err(SessionAdmitError::OutputBusy(code));
            }
        }
        let stamp = proof.stamp();
        // Preserve protected-operation policy independently of current enforcement state:
        // normalizing `NotRequired` -> `Required(OperatorGrant::unenforced())` admits while
        // `operator_auth_enforced == false` and automatically fails closed both at admission and
        // on every live step/TX check if enforcement is enabled later.
        let operator = if prepared.slot == SessionSlot::ExclusiveOutput
            && !matches!(
                prepared.kind,
                SessionKind::GripperZeroCheck | SessionKind::MacroBacked
            )
            && matches!(prepared.operator, crate::auth::OperatorRequirement::NotRequired)
        {
            crate::auth::OperatorRequirement::Required(crate::auth::OperatorGrant::unenforced())
        } else {
            prepared.operator
        };
        let authority = SessionAuthority {
            baseline: self.capture_baseline(),
            connection_id: if prepared.slot == SessionSlot::Recorder { 0 } else { stamp.connection_id },
            lease_generation: if prepared.require_lease { stamp.lease_gen } else { 0 },
            epoch: stamp.estop_epoch,
            stop_generation: stamp.stop_gen,
            require_lease: prepared.require_lease && self.lease_enabled,
            operator,
            revoke_on_stop: prepared.revoke_on_stop,
            revoke_on_epoch_change: prepared.revoke_on_epoch_change,
        };
        if let SessionTerminalBinding::Routed { client, .. } = terminal {
            if self.is_connection_closed(client.0) {
                return Err(SessionAdmitError::AuthorityRejected("CONNECTION_CLOSED"));
            }
        }
        if authority.require_lease
            && !self.is_session_lease_valid_at(authority.connection_id, authority.lease_generation, now)
        {
            return Err(SessionAdmitError::AuthorityRejected("LEASE_LOST"));
        }
        if let Err(code) = self.check_operator_requirement(authority.operator, now) {
            return Err(SessionAdmitError::AuthorityRejected(code));
        }
        let id = self
            .sessions
            .admit_with_authority_handoff(prepared, authority, terminal, handoff, now, true)?;
        if prepared.slot == SessionSlot::ExclusiveOutput && prepared.kind != SessionKind::MacroBacked {
            self.release_servo_target();
            self.servo_stream_active = false;
        }
        Ok(id)
    }

    #[inline]
    pub(crate) fn is_session_lease_valid_at(&self, conn_id: u64, lease_gen: u64, now: Instant) -> bool {
        if lease_gen == 0 || (conn_id != 0 && self.is_connection_closed(conn_id)) { return false; }
        self.lease_valid_at(now).is_some_and(|l| {
            l.generation == lease_gen
                && (conn_id == 0 || self.active_lease_conn_id == 0 || self.active_lease_conn_id == conn_id)
        })
    }

    pub(crate) fn exclusive_output_conflict_for(
        &self,
        kind: SessionKind,
        action_id: Option<ActionId>,
    ) -> Option<&'static str> {
        if self.sessions.is_slot_occupied(SessionSlot::ExclusiveOutput) { return Some("SESSION_BUSY"); }
        if self.torque_transition_pending() { return Some("TRANSITION_BUSY"); }
        let shares_live_macro = kind == SessionKind::MacroBacked
            && action_id.is_some()
            && self.macro_run.as_ref().is_some_and(|m| !m.stop_requested && !m.finishing && Some(ActionId(m.id)) == action_id);
        if !shares_live_macro {
            if self.active_trajectory.is_some()
                || self.quintic_settle.is_some()
                || self.park_release.is_some()
                || self.watchdog_park.is_some()
                || self.servo_target.is_some()
                || self.servo_stream_active
                || self.gravity_comp.is_some()
                || self.mit_active()
                || self.mit.park_after_hand_back
                || self.mit.revoke_pending
                || self.macro_run.is_some()
            {
                return Some("MOTION_BUSY");
            }
            #[cfg(feature = "policy-runtime")]
            if self.policy_run.is_some() { return Some("MOTION_BUSY"); }
        }
        if kind != SessionKind::Commissioning && self.core.motion_inhibited_by_config() {
            return Some("CONFIG_BUSY");
        }
        None
    }

    pub(crate) fn build_session_release_ctx(
        &self,
        ctx: &super::ops::CommitCtx,
        release_deadline: Instant,
        base_tx_cutoff: Instant,
    ) -> ReleaseCtx {
        ReleaseCtx {
            arm: ctx.arm,
            boot: ctx.boot,
            owner_generation: self.owner_generation(),
            now: ctx.now,
            release_deadline,
            base_tx_cutoff,
            epoch: self.safety.epoch_id(),
            stop_generation: self.current_stop_gen(),
            estop_latched: self.estop_request_unapplied()
                || matches!(self.safety.state(), crate::safety::SafetyState::EStop),
            urgent_stop_pending: self.urgent_stop_unapplied(),
            internal_fault_fenced: self.internal_fault_lane.is_fenced(),
            safe_hold_active: self.sessions.is_safe_hold_requested(),
        }
    }

    /// Live transmit-point gate for generic output sessions (equivalent to C-F1's per-TX gate).
    /// Requires a `Running` `ExclusiveOutput` owner matching `id`, live authority, and a live
    /// effective TX cutoff (`now < min(base_tx_cutoff, session_deadline, child_deadline)`).
    pub fn check_session_pre_tx(
        &self,
        slot: SessionSlot,
        id: SessionId,
        now: Instant,
        base_tx_cutoff: Instant,
    ) -> Result<Instant, &'static str> {
        if slot != SessionSlot::ExclusiveOutput {
            return Err("INVALID_COMMAND");
        }
        let Some(active) = self.sessions.slot(slot).filter(|a| a.id == id) else {
            return Err("CANCELLED");
        };
        match active.state {
            SessionState::Running => {}
            SessionState::AwaitingWorker(_) => return Err("MOTION_BUSY"),
            SessionState::Terminal(_) => return Err("CANCELLED"),
        }
        let ctx = ReleaseCtx {
            arm: self.core.id(),
            boot: BootId::process(),
            owner_generation: self.owner_generation(),
            now,
            release_deadline: base_tx_cutoff,
            base_tx_cutoff,
            epoch: self.safety.epoch_id(),
            stop_generation: self.current_stop_gen(),
            estop_latched: self.estop_request_unapplied()
                || matches!(self.safety.state(), crate::safety::SafetyState::EStop),
            urgent_stop_pending: self.urgent_stop_unapplied(),
            internal_fault_fenced: self.internal_fault_lane.is_fenced(),
            safe_hold_active: self.sessions.is_safe_hold_requested(),
        };
        self.sessions
            .check_live_authority(
                slot,
                id,
                &ctx,
                |c| self.is_connection_closed(c),
                |c, g, t| self.is_session_lease_valid_at(c, g, t),
                |req, t| self.check_operator_requirement(req, t).is_ok(),
            )
            .map_err(SessionTerminalReason::code)?;
        let effective = self.sessions.effective_tx_cutoff(slot, base_tx_cutoff);
        if now >= effective {
            return Err("DEADLINE_EXPIRED");
        }
        Ok(effective)
    }

    pub(crate) fn maybe_escalate_session_safe_hold(&mut self, outcome: SessionOutcome, escalate: bool) {
        if !escalate || !outcome.uncertain_hw_state { return; }
        if !crate::rt_core::executor::core_is_confirmed_disarmed(self) {
            self.estop_state.request_and_latch();
            self.internal_fault_lane.latch_control_safe_hold_fence();
            self.sessions.request_safe_hold();
        }
    }

    pub(crate) fn service_sessions_release_with_budget(
        &mut self,
        ctx: &super::ops::CommitCtx,
        budget: &mut super::budget::ReleaseBudget,
        owner_now_fn: &dyn Fn() -> Instant,
    ) {
        let release_deadline = budget.deadline();
        if let Some(active) = self.sessions.slot(SessionSlot::ExclusiveOutput).copied() {
            if active.kind == SessionKind::GripperZeroCheck {
                self.service_gripper_zero_check_session_with_budget(ctx, budget, owner_now_fn);
            } else if active.is_live() {
                match active.kind {
                    SessionKind::Commissioning => {
                        self.service_commissioning_release_with_budget(ctx, budget, owner_now_fn);
                    }
                    SessionKind::ModeSwitch => {
                        let mut f = || owner_now_fn();
                        self.service_mode_switch_release_with_budget(ctx, budget, &mut f);
                    }
                    SessionKind::GripperZeroCheck => unreachable!(),
                    _ => {
                        let now = owner_now_fn();
                        let total_span = release_deadline.saturating_duration_since(now);
                        let margin = self.tx_release_publish_margin().min(total_span / 2);
                        let base_cutoff = release_deadline.checked_sub(margin).unwrap_or(now);
                        let rctx = self.build_session_release_ctx(
                            &super::ops::CommitCtx { now, ..*ctx },
                            release_deadline,
                            base_cutoff,
                        );
                        let conn_closed = active.authority.connection_id != 0
                            && self.is_connection_closed(active.authority.connection_id);
                        let lease_ok = !active.authority.require_lease
                            || self.is_session_lease_valid_at(active.authority.connection_id, active.authority.lease_generation, now);
                        let op_ok = self.check_operator_requirement(active.authority.operator, now).is_ok();
                        if let Some(SessionStep::Terminal(outcome)) = self.sessions.step_slot(
                            SessionSlot::ExclusiveOutput,
                            &rctx,
                            |_| conn_closed,
                            |_, _, _| lease_ok,
                            |_, _| op_ok,
                            |_, _| active.hw_state_uncertain,
                            |_, _, _| PhaseStepOutcome::Pending,
                        ) {
                            self.maybe_escalate_session_safe_hold(outcome, active.escalate_safe_hold_on_uncertain);
                        }
                    }
                }
            }
        } else if self.commissioning.is_running() {
            self.service_commissioning_release_with_budget(ctx, budget, owner_now_fn);
        }

        if let Some(rec) = self.sessions.slot(SessionSlot::Recorder).copied() {
            if rec.is_live() {
                let now = owner_now_fn();
                let rctx = self.build_session_release_ctx(
                    &super::ops::CommitCtx { now, ..*ctx },
                    release_deadline,
                    release_deadline,
                );
                let conn_closed = rec.authority.connection_id != 0
                    && self.is_connection_closed(rec.authority.connection_id);
                let lease_ok = !rec.authority.require_lease
                    || self.is_session_lease_valid_at(rec.authority.connection_id, rec.authority.lease_generation, now);
                let op_ok = self.check_operator_requirement(rec.authority.operator, now).is_ok();
                if let Some(SessionStep::Terminal(_)) = self.sessions.step_slot(
                    SessionSlot::Recorder,
                    &rctx,
                    |_| conn_closed,
                    |_, _, _| lease_ok,
                    |_, _| op_ok,
                    |_, _| false,
                    |_, _, _| PhaseStepOutcome::Pending,
                ) {
                    let _ = self.close_rt_recorder_session(true);
                    let _ = self.sessions.clear_terminal_slot(SessionSlot::Recorder, rec.id);
                }
            }
        }
    }

    pub(crate) fn on_safe_hold_enter(&mut self, reason: &'static str) {
        self.cancel_active_commissioning_on_stop(reason);
        self.reconcile_mode_switch_on_revocation(reason);
        self.cancel_active_gripper_zero_check_on_stop(reason);
        let outcomes = self.sessions.revoke_live_sessions_by_scope(
            SessionTerminalReason::Revoked(reason),
            RevocationScope::FatalOrTeardown,
        );
        if let Some(rec_out) = outcomes[SessionSlot::Recorder.index()] {
            let _ = self.close_rt_recorder_session(true);
            let _ = self.sessions.clear_terminal_slot(SessionSlot::Recorder, rec_out.session);
        }
    }

    pub(crate) fn revoke_sessions_on_stop(&mut self, reason: &'static str) {
        self.cancel_active_commissioning_on_stop(reason);
        self.cancel_active_gripper_zero_check_on_stop(reason);
        let escalate = self
            .sessions
            .slot(SessionSlot::ExclusiveOutput)
            .is_some_and(|s| s.escalate_safe_hold_on_uncertain);
        let scope = RevocationScope::from_stop_reason(reason);
        if self
            .sessions
            .slot(SessionSlot::ExclusiveOutput)
            .is_some_and(|s| {
                s.kind == SessionKind::ModeSwitch
                    && s.is_live()
                    && match scope {
                        RevocationScope::FatalOrTeardown => true,
                        RevocationScope::MotionStop => s.authority.revoke_on_stop,
                        RevocationScope::EpochOrEStop => s.authority.revoke_on_epoch_change,
                        RevocationScope::LeaseLoss => s.authority.require_lease,
                    }
            })
        {
            self.reconcile_mode_switch_on_revocation(reason);
        }
        let outcomes = self
            .sessions
            .revoke_live_sessions_by_scope(SessionTerminalReason::Revoked(reason), scope);
        if let Some(outcome) = outcomes[SessionSlot::ExclusiveOutput.index()] {
            self.maybe_escalate_session_safe_hold(outcome, escalate);
        }
        if let Some(rec_out) = outcomes[SessionSlot::Recorder.index()] {
            let _ = self.close_rt_recorder_session(true);
            let _ = self.sessions.clear_terminal_slot(SessionSlot::Recorder, rec_out.session);
        }
    }

    pub(crate) fn revoke_sessions_for_connection(&mut self, conn_id: u64, reason: &'static str) {
        self.cancel_active_gripper_zero_check_for_connection(conn_id, reason);
        let escalate = self
            .sessions
            .slot(SessionSlot::ExclusiveOutput)
            .is_some_and(|s| s.escalate_safe_hold_on_uncertain);
        if conn_id != 0
            && self
                .sessions
                .slot(SessionSlot::ExclusiveOutput)
                .is_some_and(|s| {
                    s.kind == SessionKind::ModeSwitch
                        && s.is_live()
                        && s.authority.connection_id == conn_id
                })
        {
            self.reconcile_mode_switch_on_revocation(reason);
        }
        let outcomes = self
            .sessions
            .revoke_for_connection(conn_id, SessionTerminalReason::Revoked(reason));
        if let Some(outcome) = outcomes[SessionSlot::ExclusiveOutput.index()] {
            self.maybe_escalate_session_safe_hold(outcome, escalate);
        }
        if let Some(rec_out) = outcomes[SessionSlot::Recorder.index()] {
            let _ = self.close_rt_recorder_session(true);
            let _ = self.sessions.clear_terminal_slot(SessionSlot::Recorder, rec_out.session);
        }
    }

    /// Zero-alloc, zero-lock RT check verifying that macro parent `id` is alive, within its
    /// whole-macro deadline at `now`, and still holds valid connection/lease/epoch/stop authority.
    pub(crate) fn is_macro_parent_authoritative_at(&self, id: u64, now: Instant) -> bool {
        if id == 0 || !self.macro_alive(id) {
            return false;
        }
        let Some(run) = self.macro_run.as_ref().filter(|m| m.id == id) else {
            return false;
        };
        if run.deadline.is_some_and(|d| now >= d) {
            return false;
        }
        if self.estop_request_unapplied()
            || self.urgent_stop_unapplied()
            || self.internal_fault_lane.is_fenced()
            || self.sessions.is_safe_hold_requested()
        {
            return false;
        }
        if matches!(
            self.safety.state(),
            crate::safety::SafetyState::EStop | crate::safety::SafetyState::Fault(_)
        ) {
            return false;
        }
        if run.stamp.estop_epoch != self.safety.epoch_id() {
            return false;
        }
        if run.stamp.stop_gen != self.current_stop_gen() {
            return false;
        }
        if run.stamp.connection_id != 0 {
            if self.is_connection_closed(run.stamp.connection_id)
                || !self.is_connection_live(run.stamp.connection_id)
            {
                return false;
            }
        } else if self.lease_enabled && run.stamp.lease_gen != 0 {
            return false;
        }
        if self.lease_enabled {
            if run.stamp.lease_gen == 0
                || !self.is_session_lease_valid_at(
                    run.stamp.connection_id,
                    run.stamp.lease_gen,
                    now,
                )
            {
                return false;
            }
        } else if run.stamp.lease_gen != 0
            && self
                .check_descriptor_lease_owner(run.stamp.connection_id, run.stamp.lease_gen)
                .is_err()
        {
            return false;
        }
        true
    }

    /// Unconditionally revokes and closes any active `SessionSlot::Recorder` during executor
    /// teardown or handback so a close fence is always frozen and no occupied slot leaks across
    /// executor lifecycles (0 alloc, 0 lock).
    pub(crate) fn revoke_recorder_for_teardown(&mut self, reason: &'static str) {
        if let Some(rec) = self.sessions.slot(SessionSlot::Recorder).copied() {
            let _ = self.sessions.finish_slot(
                SessionSlot::Recorder,
                rec.id,
                SessionTerminalReason::Revoked(reason),
                false,
            );
            let _ = self.sessions.clear_terminal_slot(SessionSlot::Recorder, rec.id);
        }
        let _ = self.close_rt_recorder_session(true);
    }

    /// Admits a lease-free RT recorder session onto `SessionSlot::Recorder` (0 alloc, 0 lock).
    pub(crate) fn admit_rt_recorder_session(
        &mut self,
        op_id: u64,
        max_frames: usize,
        bound_action_id: u64,
        now: Instant,
    ) -> Result<super::recorder::RecorderStartAck, super::recorder::RecorderStartError> {
        use super::recorder::{RecorderStartAck, RecorderStartError};
        if self.internal_fault_lane.is_fenced() || self.sessions.is_safe_hold_requested() {
            return Err(RecorderStartError::FaultFenced);
        }
        if !self.rt_recorder.has_producer() {
            return Err(RecorderStartError::StreamUnattached);
        }
        if bound_action_id != 0 && !self.is_macro_parent_authoritative_at(bound_action_id, now) {
            return Err(RecorderStartError::ParentNotAlive);
        }
        if self.sessions.is_slot_occupied(SessionSlot::Recorder) {
            return Err(RecorderStartError::SlotBusy);
        }
        let prepared = PreparedSession::recorder(ClientId(0));
        let authority = SessionAuthority {
            baseline: self.capture_baseline(),
            connection_id: 0,
            lease_generation: 0,
            epoch: self.safety.epoch_id(),
            stop_generation: self.current_stop_gen(),
            require_lease: false,
            operator: crate::auth::OperatorRequirement::NotRequired,
            revoke_on_stop: false,
            revoke_on_epoch_change: false,
        };
        let session_id = self
            .sessions
            .admit_with_authority(
                prepared,
                authority,
                SessionTerminalBinding::Internal,
                now,
                true,
            )
            .map_err(|_| RecorderStartError::SlotBusy)?;
        self.rt_recorder.begin(op_id, session_id, max_frames, bound_action_id);
        Ok(RecorderStartAck {
            op_id,
            session_id,
            start_calib_rev: self.calibration_revision,
            start_tool_digest_hash: super::ops::tool_digest_hash(&self.config.active_tool.config_digest),
            start_dof: self.config.dof.min(crate::rt_core::mailbox::MAX_DOF) as u8,
            start_control_hz: self.config.control_frequency_hz.max(1),
        })
    }

    /// Single RT close transition for `RtRecorder` + `SessionSlot::Recorder` (0 alloc, 0 lock).
    pub(crate) fn close_rt_recorder_session(
        &mut self,
        revoked: bool,
    ) -> Option<super::recorder::RecorderStopFence> {
        let tool_hash = super::ops::tool_digest_hash(&self.config.active_tool.config_digest);
        let dof = self.config.dof.min(crate::rt_core::mailbox::MAX_DOF) as u8;
        let hz = self.config.control_frequency_hz.max(1);
        self.rt_recorder
            .close_active(self.calibration_revision, tool_hash, dof, hz, revoked)
    }

    /// Stops an active RT recorder session (0 alloc, 0 lock).
    /// - If `save && pushed == 0`: refuses `EmptyEpisode` WITHOUT stopping the session.
    /// - For a bound sequence recorder (`bound_action_id != 0`) or explicit `require_alive_action_id`,
    ///   when `save` is true, verifies full parent liveness, deadline, and authority at `now`;
    ///   if expired/revoked, atomically closes/discards at the RT boundary and returns `ParentNotAlive`.
    pub(crate) fn stop_rt_recorder_session(
        &mut self,
        op_id: u64,
        session_id: SessionId,
        save: bool,
        reason: &'static str,
        require_alive_action_id: u64,
        now: Instant,
    ) -> Result<super::recorder::RecorderStopFence, super::recorder::RecorderStopError> {
        use super::recorder::RecorderStopError;
        let Some(active) = self.sessions.slot(SessionSlot::Recorder).copied() else {
            if let Some(fence) = self.rt_recorder.last_closed_fence_for(op_id, session_id) {
                return Ok(fence);
            }
            return Err(RecorderStopError::NotActive);
        };
        if (session_id.0 != 0 && active.id != session_id)
            || (op_id != 0 && self.rt_recorder.active_op_id() != op_id)
        {
            return Err(RecorderStopError::SessionMismatch);
        }
        let bound_id = self.rt_recorder.bound_action_id();
        if save && bound_id != 0 && require_alive_action_id != 0 && require_alive_action_id != bound_id {
            return Err(RecorderStopError::SessionMismatch);
        }
        let required_parent = if bound_id != 0 {
            bound_id
        } else {
            require_alive_action_id
        };
        if save && required_parent != 0 && !self.is_macro_parent_authoritative_at(required_parent, now) {
            let term_reason = SessionTerminalReason::Cancelled("STOPPED");
            let _ = self.sessions.finish_slot(SessionSlot::Recorder, active.id, term_reason, false);
            let _ = self.sessions.clear_terminal_slot(SessionSlot::Recorder, active.id);
            let fence = self
                .close_rt_recorder_session(true)
                .ok_or(RecorderStopError::NotActive)?;
            return Err(RecorderStopError::ParentNotAlive { fence });
        }
        if save && self.rt_recorder.pushed() == 0 {
            return Err(RecorderStopError::EmptyEpisode);
        }
        let term_reason = if save {
            SessionTerminalReason::Completed
        } else {
            SessionTerminalReason::Cancelled(if reason.is_empty() { "DISCARDED" } else { reason })
        };
        let _ = self.sessions.finish_slot(SessionSlot::Recorder, active.id, term_reason, false);
        let _ = self.sessions.clear_terminal_slot(SessionSlot::Recorder, active.id);
        self.close_rt_recorder_session(false)
            .ok_or(RecorderStopError::NotActive)
    }

    pub(crate) fn flush_published_session_terminals(&mut self) -> usize {
        self.mode_switch.flush_events();
        let flushed = self.sessions.flush_published_terminals();
        if let Some(slot) = self.sessions.slot(SessionSlot::ExclusiveOutput).copied() {
            if slot.kind == SessionKind::ModeSwitch
                && matches!(slot.state, SessionState::Terminal(_))
                && self.sessions.clear_terminal_slot(SessionSlot::ExclusiveOutput, slot.id)
            {
                self.mode_switch.retire_if_session(slot.id);
            }
        }
        flushed
    }
}

#[cfg(test)]
mod recorder_deadline_tests {
    use super::*;
    use crate::macro_action::MacroRun;
    use crate::model::ArmConfig;
    use crate::rt_core::recorder::{RecorderSample, RecorderStopError};
    use crate::rt_core::stream;
    use crate::supervisor::AdmissionStamp;

    #[test]
    fn exact_parent_deadline_instant_rejects_recorder_save_and_closes_slot() {
        let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
        inner.lease_enabled = false;
        let (tx, _rx) = stream::channel::<RecorderSample>(8);
        inner.rt_recorder.attach_producer(tx);
        let t0 = Instant::now();
        let deadline = t0 + std::time::Duration::from_secs(5);
        let id = inner.begin_action("sequence", None);
        let stamp = AdmissionStamp {
            connection_id: 0,
            lease_gen: 0,
            estop_epoch: inner.safety.epoch_id(),
            stop_gen: inner.current_stop_gen(),
            request_id: 1,
        };
        inner.macro_run = Some(MacroRun::new(id, inner.safety.epoch_id(), stamp).with_deadline(
            Some(deadline),
            "SEQUENCE_TIMEOUT",
        ));

        let ack = inner
            .admit_rt_recorder_session(1, 100, id, t0)
            .expect("admit strictly before parent deadline must succeed");
        assert!(inner.rt_recorder.try_push_sample(1, 0.0, &[0.0; 7], &[0.0; 7]));

        // Exact inclusive boundary `now == deadline` MUST reject save and close/revoke the slot.
        let err = inner
            .stop_rt_recorder_session(1, ack.session_id, true, "saved", id, deadline)
            .expect_err("exact deadline instant must reject recorder save");
        assert!(
            matches!(err, RecorderStopError::ParentNotAlive { fence } if fence.revoked),
            "expected revoked ParentNotAlive at exact deadline, got {err:?}"
        );
        assert!(!inner.sessions.is_slot_occupied(SessionSlot::Recorder));
    }
}
