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
pub struct SessionAuthority {
    pub connection_id: u64,
    pub lease_generation: u64,
    pub epoch: u64,
    pub stop_generation: u64,
    pub require_lease: bool,
    pub operator_required: bool,
    pub operator_grant: Option<crate::auth::OperatorGrant>,
    pub revoke_on_stop: bool,
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
    pub request_id: Option<RequestId>,
    pub route_gen: u64,
    pub client: ClientId,
    pub operator_required: bool,
    pub operator_grant: Option<crate::auth::OperatorGrant>,
    pub revoke_on_stop: bool,
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
            request_id: None,
            route_gen: 0,
            client,
            operator_required: false,
            operator_grant: None,
            revoke_on_stop: true,
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
            request_id: None,
            route_gen: 0,
            client,
            operator_required: false,
            operator_grant: None,
            revoke_on_stop: false,
            escalate_safe_hold_on_uncertain: false,
        }
    }

    /// Binds a P-F1 [`super::reply_router::SessionTerminalTicket`] (request, client, action_id,
    /// route_gen) into this prepared session so RT terminal handoff carries exact identity.
    pub fn bind_terminal_ticket(&mut self, ticket: &super::reply_router::SessionTerminalTicket) {
        self.request_id = Some(ticket.request());
        self.client = ticket.client();
        self.action_id = Some(ticket.action_id());
        self.route_gen = ticket.route_gen();
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SessionAdmitError {
    SlotBusy,
    OutputBusy(&'static str),
    DeadlineExpired,
    InvalidSlotKind,
    AuthorityRejected(&'static str),
}

impl SessionAdmitError {
    pub fn code(self) -> &'static str {
        match self {
            Self::SlotBusy => "SESSION_BUSY",
            Self::OutputBusy(c) | Self::AuthorityRejected(c) => c,
            Self::DeadlineExpired => "DEADLINE_EXPIRED",
            Self::InvalidSlotKind => "INVALID_COMMAND",
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
    pub request_id: Option<RequestId>,
    pub route_gen: u64,
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
        now: Instant,
        reject_expired_at_admit: bool,
    ) -> Result<SessionId, SessionAdmitError> {
        let invalid = (prepared.slot == SessionSlot::Recorder && prepared.kind != SessionKind::Recorder)
            || (prepared.slot == SessionSlot::ExclusiveOutput && prepared.kind == SessionKind::Recorder);
        if invalid { return Err(SessionAdmitError::InvalidSlotKind); }
        if self.is_slot_occupied(prepared.slot) { return Err(SessionAdmitError::SlotBusy); }
        if reject_expired_at_admit
            && (prepared.deadline.is_some_and(|dl| now >= dl)
                || prepared.child_deadline.is_some_and(|dl| now >= dl))
        {
            return Err(SessionAdmitError::DeadlineExpired);
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
            action_id: prepared.action_id,
            request_id: prepared.request_id,
            route_gen: prepared.route_gen,
            client: prepared.client,
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

    pub fn check_live_authority(
        &self,
        slot: SessionSlot,
        id: SessionId,
        ctx: &ReleaseCtx,
        is_conn_closed: impl Fn(u64) -> bool,
        is_lease_valid: impl Fn(u64, u64, Instant) -> bool,
        is_operator_valid: impl Fn(Instant) -> bool,
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
        if ctx.estop_latched || ctx.epoch != auth.epoch {
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
        if auth.operator_required && !is_operator_valid(ctx.now) {
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
        if let Some(request) = active.request_id {
            let action_id = active.action_id.unwrap_or(ActionId(id.0));
            self.pending_terminals[idx] = Some(SessionTerminalRecord {
                session: id,
                slot,
                request,
                route_gen: active.route_gen,
                client: active.client,
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
        is_operator_valid: impl Fn(Instant) -> bool,
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

    pub fn revoke_live_sessions(
        &mut self,
        reason: SessionTerminalReason,
        is_ordinary_stop: bool,
    ) -> [Option<SessionOutcome>; 2] {
        let mut out = [None, None];
        for slot in [SessionSlot::ExclusiveOutput, SessionSlot::Recorder] {
            let live = self.slot(slot).and_then(|s| {
                (s.is_live() && (!is_ordinary_stop || s.authority.revoke_on_stop))
                    .then_some((s.id, s.hw_state_uncertain))
            });
            if let Some((id, uncertain)) = live {
                out[slot.index()] = self.finish_slot(slot, id, reason, uncertain);
            }
        }
        out
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

/// Admits a prepared session by consuming a verified A1 [`Admitted`] token, binding authority
/// strictly from [`AdmittedProof`] and live owner policy (fail-closed).
pub fn admit_session(
    admitted: Admitted<'_>,
    prepared: PreparedSession,
    now: Instant,
) -> Result<SessionId, SessionAdmitError> {
    let (inner, proof) = admitted.consume().map_err(SessionAdmitError::from_safety)?;
    inner.admit_session_with_proof(prepared, &proof, now)
}

impl SupervisorInner {
    #[inline]
    pub fn sessions(&self) -> &SessionDriver { &self.sessions }

    #[inline]
    pub fn sessions_mut(&mut self) -> &mut SessionDriver { &mut self.sessions }

    pub(crate) fn admit_session_with_proof(
        &mut self,
        prepared: PreparedSession,
        proof: &AdmittedProof,
        now: Instant,
    ) -> Result<SessionId, SessionAdmitError> {
        if prepared.slot == SessionSlot::ExclusiveOutput {
            if let Some(code) = self.exclusive_output_conflict_for(prepared.kind, prepared.action_id) {
                return Err(SessionAdmitError::OutputBusy(code));
            }
        }
        let stamp = proof.stamp();
        let authority = SessionAuthority {
            connection_id: stamp.connection_id,
            lease_generation: stamp.lease_gen,
            epoch: stamp.estop_epoch,
            stop_generation: stamp.stop_gen,
            require_lease: self.lease_enabled,
            operator_required: self.operator_auth_enforced || prepared.operator_required,
            operator_grant: prepared.operator_grant,
            revoke_on_stop: prepared.revoke_on_stop,
        };
        if authority.require_lease
            && !self.is_session_lease_valid_at(authority.connection_id, authority.lease_generation, now)
        {
            return Err(SessionAdmitError::AuthorityRejected("LEASE_LOST"));
        }
        if authority.operator_required && !self.is_session_operator_valid_at(authority.operator_grant, now) {
            return Err(SessionAdmitError::AuthorityRejected("OPERATOR_REQUIRED"));
        }
        let id = self.sessions.admit_with_authority(prepared, authority, now, true)?;
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

    #[inline]
    pub(crate) fn is_session_operator_valid_at(&self, grant: Option<crate::auth::OperatorGrant>, now: Instant) -> bool {
        grant.is_some_and(|g| g.is_valid_at(now, true))
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
        let grant = active.authority.operator_grant;
        self.sessions
            .check_live_authority(
                slot,
                id,
                &ctx,
                |c| self.is_connection_closed(c),
                |c, g, t| self.is_session_lease_valid_at(c, g, t),
                |t| self.is_session_operator_valid_at(grant, t),
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

    pub(crate) fn service_sessions_release(
        &mut self,
        ctx: &super::ops::CommitCtx,
        release_deadline: Instant,
        owner_now_fn: &dyn Fn() -> Instant,
    ) {
        if let Some(active) = self.sessions.slot(SessionSlot::ExclusiveOutput).copied() {
            if active.is_live() {
                match active.kind {
                    SessionKind::Commissioning => {
                        self.service_commissioning_release(ctx, release_deadline, owner_now_fn);
                    }
                    SessionKind::ModeSwitch => {
                        self.service_mode_switch_release(ctx, release_deadline, owner_now_fn);
                    }
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
                        let op_ok = !active.authority.operator_required
                            || self.is_session_operator_valid_at(active.authority.operator_grant, now);
                        if let Some(SessionStep::Terminal(outcome)) = self.sessions.step_slot(
                            SessionSlot::ExclusiveOutput,
                            &rctx,
                            |_| conn_closed,
                            |_, _, _| lease_ok,
                            |_| op_ok,
                            |_, _| active.hw_state_uncertain,
                            |_, _, _| PhaseStepOutcome::Pending,
                        ) {
                            self.maybe_escalate_session_safe_hold(outcome, active.escalate_safe_hold_on_uncertain);
                        }
                    }
                }
            }
        } else if self.commissioning.is_running() {
            self.service_commissioning_release(ctx, release_deadline, owner_now_fn);
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
                let op_ok = !rec.authority.operator_required
                    || self.is_session_operator_valid_at(rec.authority.operator_grant, now);
                let _ = self.sessions.step_slot(
                    SessionSlot::Recorder,
                    &rctx,
                    |_| conn_closed,
                    |_, _, _| lease_ok,
                    |_| op_ok,
                    |_, _| false,
                    |_, _, _| PhaseStepOutcome::Pending,
                );
            }
        }
    }

    pub(crate) fn on_safe_hold_enter(&mut self, reason: &'static str) {
        self.cancel_active_commissioning_on_stop(reason);
        self.reconcile_mode_switch_on_revocation(reason);
        let _ = self.sessions.revoke_live_sessions(SessionTerminalReason::Revoked(reason), false);
    }

    pub(crate) fn revoke_sessions_on_stop(&mut self, reason: &'static str) {
        self.cancel_active_commissioning_on_stop(reason);
        let escalate = self
            .sessions
            .slot(SessionSlot::ExclusiveOutput)
            .is_some_and(|s| s.escalate_safe_hold_on_uncertain);
        let is_ordinary_stop = reason == "STOPPED";
        if self
            .sessions
            .slot(SessionSlot::ExclusiveOutput)
            .is_some_and(|s| {
                s.kind == SessionKind::ModeSwitch
                    && s.is_live()
                    && (!is_ordinary_stop || s.authority.revoke_on_stop)
            })
        {
            self.reconcile_mode_switch_on_revocation(reason);
        }
        let outcomes = self
            .sessions
            .revoke_live_sessions(SessionTerminalReason::Revoked(reason), is_ordinary_stop);
        if let Some(outcome) = outcomes[SessionSlot::ExclusiveOutput.index()] {
            self.maybe_escalate_session_safe_hold(outcome, escalate);
        }
    }

    pub(crate) fn revoke_sessions_for_connection(&mut self, conn_id: u64, reason: &'static str) {
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
    }

    pub(crate) fn flush_published_session_terminals(&mut self) -> usize {
        self.mode_switch.flush_events();
        self.sessions.flush_published_terminals()
    }
}
