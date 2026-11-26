//! Causal owner authority baselines and applied-stop evidence (`AuthorityProof`).
//!
//! Zero-allocation, lock-free, `Copy` primitive embedded in [`SupervisorInner`] and captured
//! centrally at session admission via [`SupervisorInner::capture_baseline`].
//!
//! Invariants:
//! 1. [`AuthorityBaseline`] records `(owner_generation, applied_general_stop_seq,
//!    applied_estop_seq, qualifying_fault_epoch, lease_generation)`.
//! 2. [`SupervisorInner::stop_applied_since`] returns event-specific [`AuthorityProof`] strictly
//!    tied to the SAME `owner_generation` and strictly AFTER admission (`>` baseline):
//!    - A pre-existing fault or already-stopped safety state is NOT proof.
//!    - A bare `safety.latch(...)` without an owner-applied stop effect is NOT proof.
//!    - A pending E-stop or pending stop request fences output ([`SupervisorInner::has_pending_stop_fence`])
//!      but is NOT proof until the owner applies and records the stop effect.
//!    - Lease loss revokes authority ([`SupervisorInner::is_baseline_lease_live_at`]) but is NOT stop proof.
//! 3. Ownership transfers (Legacy <-> Executor) advance `owner_generation`, invalidating any
//!    baseline captured under a prior owner generation.

use std::time::Instant;

pub use crate::auth::{OperatorGrant, OperatorRequirement};
use crate::supervisor::SupervisorInner;

/// Causal owner baseline captured at session admission or recovery start.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AuthorityBaseline {
    pub owner_generation: u64,
    pub applied_general_stop_seq: u64,
    pub applied_estop_seq: u64,
    pub qualifying_fault_epoch: u64,
    pub lease_generation: u64,
}

/// Event-specific stop effect applied by the owner strictly after an [`AuthorityBaseline`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StopProofEvent {
    /// Owner applied an E-stop latch (`applied_estop_seq > baseline.applied_estop_seq`).
    EStopApplied { seq: u64 },
    /// Owner applied an authorized general stop (`applied_general_stop_seq > baseline.applied_general_stop_seq`).
    GeneralStopApplied { seq: u64 },
    /// Owner recorded a qualifying fault stop effect (`qualifying_fault_epoch > baseline.qualifying_fault_epoch`).
    QualifyingFaultStopApplied { epoch: u64 },
}

/// Event-specific causal proof that the same owner generation applied a stop effect after admission.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AuthorityProof {
    owner_generation: u64,
    event: StopProofEvent,
}

impl AuthorityProof {
    #[inline]
    pub(crate) const fn new(owner_generation: u64, event: StopProofEvent) -> Self {
        Self {
            owner_generation,
            event,
        }
    }

    #[inline]
    pub const fn owner_generation(&self) -> u64 {
        self.owner_generation
    }

    #[inline]
    pub const fn event(&self) -> StopProofEvent {
        self.event
    }
}

/// Inline, zero-alloc owner authority counter state embedded in [`SupervisorInner`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct OwnerAuthorityState {
    owner_generation: u64,
    applied_general_stop_seq: u64,
    applied_estop_seq: u64,
    qualifying_fault_epoch: u64,
}

impl Default for OwnerAuthorityState {
    fn default() -> Self {
        Self::new()
    }
}

impl OwnerAuthorityState {
    #[inline]
    pub(crate) const fn new() -> Self {
        Self {
            owner_generation: 1,
            applied_general_stop_seq: 0,
            applied_estop_seq: 0,
            qualifying_fault_epoch: 0,
        }
    }

    #[inline]
    pub(crate) const fn owner_generation(&self) -> u64 {
        self.owner_generation
    }

    #[inline]
    pub(crate) fn advance_owner_generation(&mut self) -> u64 {
        self.owner_generation = self.owner_generation.wrapping_add(1).max(1);
        self.owner_generation
    }

    #[inline]
    pub(crate) fn record_applied_general_stop(&mut self) -> u64 {
        self.applied_general_stop_seq = self.applied_general_stop_seq.saturating_add(1);
        self.applied_general_stop_seq
    }

    #[inline]
    pub(crate) fn record_applied_estop(&mut self) -> u64 {
        self.applied_estop_seq = self.applied_estop_seq.saturating_add(1);
        self.applied_estop_seq
    }

    #[inline]
    pub(crate) fn record_qualifying_fault_stop(&mut self, epoch: u64) {
        if epoch > self.qualifying_fault_epoch {
            self.qualifying_fault_epoch = epoch;
        }
    }

    #[inline]
    pub(crate) const fn capture_baseline(&self, lease_generation: u64) -> AuthorityBaseline {
        AuthorityBaseline {
            owner_generation: self.owner_generation,
            applied_general_stop_seq: self.applied_general_stop_seq,
            applied_estop_seq: self.applied_estop_seq,
            qualifying_fault_epoch: self.qualifying_fault_epoch,
            lease_generation,
        }
    }

    #[inline]
    pub(crate) fn stop_applied_since(&self, baseline: AuthorityBaseline) -> Option<AuthorityProof> {
        if baseline.owner_generation == 0 || self.owner_generation != baseline.owner_generation {
            return None;
        }
        if self.applied_estop_seq > baseline.applied_estop_seq {
            return Some(AuthorityProof::new(
                self.owner_generation,
                StopProofEvent::EStopApplied {
                    seq: self.applied_estop_seq,
                },
            ));
        }
        if self.applied_general_stop_seq > baseline.applied_general_stop_seq {
            return Some(AuthorityProof::new(
                self.owner_generation,
                StopProofEvent::GeneralStopApplied {
                    seq: self.applied_general_stop_seq,
                },
            ));
        }
        if self.qualifying_fault_epoch > baseline.qualifying_fault_epoch {
            return Some(AuthorityProof::new(
                self.owner_generation,
                StopProofEvent::QualifyingFaultStopApplied {
                    epoch: self.qualifying_fault_epoch,
                },
            ));
        }
        None
    }

    #[inline]
    pub(crate) fn new_estop_since(
        &self,
        baseline: AuthorityBaseline,
        estop_request_unapplied: bool,
    ) -> bool {
        baseline.owner_generation == 0
            || self.owner_generation != baseline.owner_generation
            || estop_request_unapplied
            || self.applied_estop_seq > baseline.applied_estop_seq
    }
}

impl SupervisorInner {
    /// Current monotonic owner generation (advances on Legacy <-> Executor ownership handoff).
    #[inline]
    pub fn owner_generation(&self) -> u64 {
        self.authority.owner_generation()
    }

    /// Advances `owner_generation` on an ownership handoff or handback (0 alloc, 0 lock).
    #[inline]
    pub(crate) fn advance_owner_generation(&mut self) -> u64 {
        self.authority.advance_owner_generation()
    }

    /// Captures the owner's causal [`AuthorityBaseline`] at admission (0 alloc, 0 lock).
    #[inline]
    pub fn capture_baseline(&self) -> AuthorityBaseline {
        self.authority.capture_baseline(self.current_active_lease_gen())
    }

    /// Returns event-specific [`AuthorityProof`] iff this SAME owner generation applied a general
    /// stop, E-stop, or qualifying fault stop strictly after `baseline` was captured.
    ///
    /// A pre-existing fault/stopped state, a pending stop/E-stop request, lease loss, or a
    /// different owner generation never qualifies as proof.
    #[inline]
    pub fn stop_applied_since(&self, baseline: AuthorityBaseline) -> Option<AuthorityProof> {
        self.authority.stop_applied_since(baseline)
    }

    /// True when any pending E-stop or authorized-stop request has not yet been applied by the
    /// owner (fences output without counting as applied-stop proof).
    #[inline]
    pub fn has_pending_stop_fence(&self) -> bool {
        self.urgent_stop_unapplied()
    }

    /// True when the lease recorded in `baseline` is still live and current on the same owner
    /// generation at `now` (lease loss revokes authority without providing stop proof).
    #[inline]
    pub fn is_baseline_lease_live_at(&self, baseline: AuthorityBaseline, now: Instant) -> bool {
        if baseline.owner_generation == 0
            || self.authority.owner_generation() != baseline.owner_generation
        {
            return false;
        }
        if self.lease_enabled {
            baseline.lease_generation != 0
                && self
                    .lease_valid_at(now)
                    .is_some_and(|l| l.generation == baseline.lease_generation)
        } else if baseline.lease_generation != 0 {
            self.lease_valid_at(now)
                .is_some_and(|l| l.generation == baseline.lease_generation)
        } else {
            true
        }
    }

    /// True when a new E-stop (applied or pending) arrived since `baseline` or the owner
    /// generation changed (for explicit recovery paths gated against new E-stops while tolerating
    /// an already-authorized pre-existing applied latch).
    #[inline]
    pub fn new_estop_since(&self, baseline: AuthorityBaseline) -> bool {
        self.authority
            .new_estop_since(baseline, self.estop_request_unapplied())
    }

    /// Records an owner-applied general stop strictly after motion has been halted.
    #[inline]
    pub(crate) fn record_applied_general_stop(&mut self) -> u64 {
        self.authority.record_applied_general_stop()
    }

    /// Records an owner-applied E-stop latch strictly after motion has been halted and latched.
    #[inline]
    pub(crate) fn record_applied_estop(&mut self) -> u64 {
        self.authority.record_applied_estop()
    }

    /// Records an owner-applied qualifying fault stop strictly after motion has been halted and
    /// `safety.latch(...)` has advanced the safety epoch.
    #[inline]
    pub(crate) fn record_applied_qualifying_fault_stop(&mut self) {
        self.authority
            .record_qualifying_fault_stop(self.safety.epoch_id());
    }

    /// Single owner-side check for [`OperatorRequirement`] at admission, before activation,
    /// and before subsequent privileged phases (0 alloc, 0 lock).
    #[inline]
    pub fn check_operator_requirement(
        &self,
        requirement: OperatorRequirement,
        now: Instant,
    ) -> Result<(), &'static str> {
        if requirement.is_satisfied_at(now, self.operator_auth_enforced) {
            Ok(())
        } else {
            Err("OPERATOR_REQUIRED")
        }
    }

    /// Like [`Self::check_operator_requirement`], failing closed when `operator_auth_enforced`
    /// is enabled and a protected operation passed [`OperatorRequirement::NotRequired`].
    #[inline]
    pub fn check_protected_operator_requirement(
        &self,
        requirement: OperatorRequirement,
        now: Instant,
    ) -> Result<(), &'static str> {
        let effective = match requirement {
            OperatorRequirement::NotRequired if self.operator_auth_enforced => {
                OperatorRequirement::Required(OperatorGrant::DENIED)
            }
            other => other,
        };
        self.check_operator_requirement(effective, now)
    }
}
