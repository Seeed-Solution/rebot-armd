//! Versioned configuration interface between configuration workers and the RT (armd 1.6
//! RT core, step 2 foundation; design section 4, last paragraph; track B).
//!
//! A worker validates and stages a complete candidate off the RT, then asks the RT to
//! **reserve** the domain at the revision the candidate was built from. The reservation is
//! checked against the proposal's stamp (identity, original deadline, stop generation, epoch,
//! and the lease when the proposal names one) and succeeds only at that revision; when the
//! domain affects motion it inhibits motion ([`ConfigHold::motion_inhibited`]) until the hold
//! ends. The worker persists the candidate and reports it ([`ConfigAuthority::persisted`]);
//! the RT **activates** it (the active revision advances) and the caller receives the outcome.
//!
//! # Grants and states
//!
//! A granted [`Reservation`] is a unique token (its [`GrantId`] is never reused within a
//! boot) that is neither `Clone` nor `Copy`: it is consumed by [`ConfigAuthority::activate`]
//! or [`ConfigAuthority::release`], so a delayed release cannot clear the hold of an
//! identical retry. [`HeldConfig`] is the copyable view of the current hold.
//!
//! | [`HoldState`] | Entered by | Motion | Leaves by |
//! |---|---|---|---|
//! | `Reserved` | `reserve` | inhibited (motion domains) | `persisted`, `activate` (volatile), `release` (hold ends) |
//! | `PersistedAwaitingActivation` | `persisted` (the durable rename happened) | inhibited | `activate`; `release` or a failed activation -> `Reconciling` |
//! | `Reconciling` | the above, or `begin_reconcile` at start (disk and active may differ) | inhibited (every domain) | `finish_reconcile` once active and durable revisions agree |
//!
//! # Candidate payloads
//!
//! Activation names the reservation, and the reservation names its [`CandidateId`]: the
//! payload stays in the worker-side candidate store keyed by that id (staged before
//! `reserve`). When activation moves into the RT, the store hands the RT a pre-staged
//! candidate by id (like a plan slot); the contract does not carry the payload.
//!
//! # Production today
//!
//! [`super::ops`] implements [`ConfigAuthority`] for the supervisor: `activate` confirms the
//! candidate the existing commit path applied (`NOT_APPLIED` when none was). Track C reads the hold at admission and commit through
//! `ArmCore::motion_inhibited_by_config` (also in `PlanningSnapshot::config_inhibited`).

use super::broker::{Authority, Stamp};

/// A configuration domain with its own revision.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ConfigDomain {
    /// Workspace limits, keep-outs and tool geometry (`workspace_revision`).
    Workspace,
    Keepout,
    Collision,
    Calibration,
    GripperCalibration,
    GraspSetup,
    ToolCommissioning,
    Poses,
    Sequences,
}

impl ConfigDomain {
    /// Changing it can invalidate planned or running motion.
    pub fn affects_motion(&self) -> bool {
        !matches!(self, Self::Poses | Self::Sequences)
    }
}

/// A staged candidate's identity (assigned by the worker; keys its candidate store).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CandidateId(pub u64);

/// The unique id of one granted reservation (never reused within a boot).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct GrantId(pub u64);

/// A request to reserve `domain` at `expected_revision` for `candidate`.
#[derive(Debug, Clone, Copy)]
pub struct ConfigProposal {
    pub domain: ConfigDomain,
    pub expected_revision: u64,
    pub candidate: CandidateId,
    /// Identity, the authority the candidate was prepared under and its original deadline.
    pub stamp: Stamp,
}

impl ConfigProposal {
    /// The refusal code of the stamp at `now` under `current`, None when it may reserve:
    /// `WRONG_TARGET`, `DEADLINE_EXPIRED`, `STOPPED`, `EPOCH_MISMATCH`, and `LEASE_LOST`
    /// when the proposal was prepared under a lease (`lease_generation != 0`) that is no
    /// longer the valid one. The revision is checked per domain (`expected_revision`), not
    /// through `Authority::config_revision`.
    pub fn check(&self, arm: super::ids::ArmId, boot: super::broker::BootId, now: std::time::Instant, current: &Authority) -> Option<&'static str> {
        let s = &self.stamp;
        if s.arm != arm || s.boot != boot { return Some("WRONG_TARGET"); }
        if now > s.deadline { return Some("DEADLINE_EXPIRED"); }
        let a = &s.authority;
        if a.stop_generation != current.stop_generation { return Some("STOPPED"); }
        if a.epoch != current.epoch { return Some("EPOCH_MISMATCH"); }
        if a.lease_generation != 0 && a.lease_generation != current.lease_generation { return Some("LEASE_LOST"); }
        None
    }
}

/// A granted reservation: the domain is held at `from`; activation moves it to `from + 1`.
/// A unique token: neither `Clone` nor `Copy` (see the module docs).
///
/// ```compile_fail
/// fn copy<T: Copy>() {}
/// copy::<armd::rt_core::config_iface::Reservation>();
/// ```
///
/// ```compile_fail
/// fn clone<T: Clone>() {}
/// clone::<armd::rt_core::config_iface::Reservation>();
/// ```
#[derive(Debug, PartialEq, Eq)]
#[must_use = "a reservation holds its domain until it is activated or released"]
pub struct Reservation {
    grant: GrantId,
    domain: ConfigDomain,
    from: u64,
    candidate: CandidateId,
    inhibits_motion: bool,
    stamp: Stamp,
}

impl Reservation {
    pub fn grant(&self) -> GrantId { self.grant }
    pub fn domain(&self) -> ConfigDomain { self.domain }
    pub fn from(&self) -> u64 { self.from }
    pub fn candidate(&self) -> CandidateId { self.candidate }
    /// Motion incompatible with the change is inhibited while the reservation is held.
    pub fn inhibits_motion(&self) -> bool { self.inhibits_motion }
    /// The proposal's stamp (request, client, authority, deadline).
    pub fn stamp(&self) -> &Stamp { &self.stamp }
}

/// Where a hold is (see the module table).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HoldState {
    Reserved,
    /// The candidate was renamed into place; activation has not happened.
    PersistedAwaitingActivation { durability: Durability },
    /// Active and durable revisions may differ (failed activation after the rename, a
    /// release after it, or an ambiguous restart): motion stays inhibited.
    Reconciling,
}

/// The copyable view of the current hold (`ArmCore::config_hold`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HeldConfig {
    /// None for a `Reconciling` hold begun at start (no grant).
    pub grant: Option<GrantId>,
    pub domain: ConfigDomain,
    pub from: u64,
    pub candidate: Option<CandidateId>,
    pub inhibits_motion: bool,
    pub state: HoldState,
}

/// Durability of a persisted candidate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Durability {
    /// Renamed into place and the directory sync confirmed.
    Durable,
    /// Renamed into place; the directory sync failed or was not confirmed (a power loss can
    /// still revert it). Reported to the caller as such.
    DurableUnconfirmed,
    /// Not persisted (no file configured).
    Volatile,
}

/// Outcome of [`ConfigAuthority::reserve`] (not `Copy`: it may carry the grant).
#[derive(Debug, PartialEq, Eq)]
pub enum Reserved {
    Granted(Reservation),
    /// `REVISION_MISMATCH` (`current` differs), `CONFIG_BUSY` (another hold), stamp codes of
    /// [`ConfigProposal::check`].
    Refused { code: &'static str, current: u64 },
}

/// Outcome of [`ConfigAuthority::activate`].
#[derive(Debug, PartialEq, Eq)]
pub enum Activated {
    Active { revision: u64, durability: Durability },
    /// Refused before anything was persisted: the reservation comes back (release or retry).
    Refused { code: &'static str, reservation: Reservation },
    /// Refused after the candidate was persisted: the hold is `Reconciling` (motion stays
    /// inhibited) until active and durable revisions are reconciled.
    Reconciling { code: &'static str },
}

/// Outcome of [`ConfigAuthority::release`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Released {
    /// The hold ended; motion is no longer inhibited by it.
    Cleared,
    /// It was persisted already: the hold is now `Reconciling`.
    Reconciling,
    /// The reservation does not hold the domain (another grant, or none).
    NotHeld,
}

/// The RT's side of versioned configuration.
pub trait ConfigAuthority {
    /// The active revision of `domain`.
    fn revision(&self, domain: ConfigDomain) -> u64;
    /// Reserves at the executor's clock and stop generation (`ctx`): stamp check first
    /// ([`ConfigProposal::check`]), then the revision, then the hold.
    fn reserve(&mut self, proposal: &ConfigProposal, ctx: &super::ops::CommitCtx) -> Reserved;
    /// The candidate of `reservation` was renamed into place (`Durable` or
    /// `DurableUnconfirmed`): the hold moves to `PersistedAwaitingActivation`.
    fn persisted(&mut self, reservation: &Reservation, durability: Durability) -> Result<(), &'static str>;
    fn activate(&mut self, reservation: Reservation) -> Activated;
    /// Gives a reservation up (staging or persistence failed).
    fn release(&mut self, reservation: Reservation) -> Released;
    /// At start, when disk and active revisions of `domain` may differ: inhibit motion until
    /// [`Self::finish_reconcile`].
    fn begin_reconcile(&mut self, domain: ConfigDomain) -> bool;
    /// Active and durable revisions of `domain` agree again; false when it was not reconciling.
    fn finish_reconcile(&mut self, domain: ConfigDomain) -> bool;
}

/// The hold state machine of one core (one hold at a time), allocation free.
#[derive(Debug, Default)]
pub struct ConfigHold {
    held: Option<HeldConfig>,
    next_grant: u64,
}

impl ConfigHold {
    pub fn held(&self) -> Option<HeldConfig> { self.held }

    /// Any hold inhibits motion when it is on a motion domain; `Reconciling` always does.
    pub fn motion_inhibited(&self) -> bool {
        self.held.is_some_and(|h| h.inhibits_motion || h.state == HoldState::Reconciling)
    }

    /// Grants a reservation of `p` at `from`; None when a hold exists.
    pub fn grant(&mut self, p: &ConfigProposal, from: u64) -> Option<Reservation> {
        if self.held.is_some() { return None; }
        self.next_grant += 1;
        let r = Reservation { grant: GrantId(self.next_grant), domain: p.domain, from, candidate: p.candidate, inhibits_motion: p.domain.affects_motion(), stamp: p.stamp };
        self.held = Some(HeldConfig { grant: Some(r.grant), domain: r.domain, from, candidate: Some(r.candidate), inhibits_motion: r.inhibits_motion, state: HoldState::Reserved });
        Some(r)
    }

    fn mine(&mut self, r: &Reservation) -> Option<&mut HeldConfig> { self.held.as_mut().filter(|h| h.grant == Some(r.grant)) }

    /// `Reserved` -> `PersistedAwaitingActivation`.
    pub fn persisted(&mut self, r: &Reservation, durability: Durability) -> Result<(), &'static str> {
        if durability == Durability::Volatile { return Err("INVALID_VALUE"); }
        let h = self.mine(r).ok_or("NOT_HELD")?;
        if h.state != HoldState::Reserved { return Err("INVALID_STATE"); }
        h.state = HoldState::PersistedAwaitingActivation { durability };
        Ok(())
    }

    /// Ends the hold of an activated reservation; false when `r` does not hold it (or it is
    /// reconciling).
    pub fn activated(&mut self, r: &Reservation) -> bool {
        match self.mine(r) {
            Some(h) if h.state != HoldState::Reconciling => { self.held = None; true }
            _ => false,
        }
    }

    /// A failed activation of `r`: back to the caller before persistence, `Reconciling` after.
    pub fn activation_failed(&mut self, r: Reservation, code: &'static str) -> Activated {
        match self.mine(&r) {
            Some(h) if matches!(h.state, HoldState::PersistedAwaitingActivation { .. }) => { h.state = HoldState::Reconciling; Activated::Reconciling { code } }
            _ => Activated::Refused { code, reservation: r },
        }
    }

    pub fn release(&mut self, r: Reservation) -> Released {
        let Some(h) = self.mine(&r) else { return Released::NotHeld; };
        match h.state {
            HoldState::Reserved => { self.held = None; Released::Cleared }
            HoldState::PersistedAwaitingActivation { .. } | HoldState::Reconciling => { h.state = HoldState::Reconciling; Released::Reconciling }
        }
    }

    /// Start-up reconciliation of `domain` (replaces nothing: false when another hold exists).
    pub fn begin_reconcile(&mut self, domain: ConfigDomain, active: u64) -> bool {
        match &mut self.held {
            Some(h) if h.domain == domain => { h.state = HoldState::Reconciling; true }
            Some(_) => false,
            None => {
                self.held = Some(HeldConfig { grant: None, domain, from: active, candidate: None, inhibits_motion: true, state: HoldState::Reconciling });
                true
            }
        }
    }

    /// Unconditionally enters `HoldState::Reconciling` for `domain`, inhibiting motion even
    /// when another hold occupies the slot (fail-closed after an ambiguous post-rename error).
    pub fn force_reconcile(&mut self, domain: ConfigDomain, active: u64) {
        self.held = Some(HeldConfig {
            grant: None,
            domain,
            from: active,
            candidate: None,
            inhibits_motion: true,
            state: HoldState::Reconciling,
        });
    }

    pub fn finish_reconcile(&mut self, domain: ConfigDomain) -> bool {
        if self.held.is_some_and(|h| h.domain == domain && h.state == HoldState::Reconciling) { self.held = None; return true; }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rt_core::broker::{BootId, ClientId, RequestId};
    use crate::rt_core::ids::ArmId;
    use std::time::{Duration, Instant};

    fn proposal(domain: ConfigDomain, lease_generation: u64) -> ConfigProposal {
        let stamp = Stamp { arm: ArmId::LOCAL, boot: BootId(1), request: RequestId(1), client: ClientId(1), client_seq: 1,
            authority: Authority { lease_generation, epoch: 2, stop_generation: 3, ..Authority::default() }, deadline: Instant::now() + Duration::from_secs(5) };
        ConfigProposal { domain, expected_revision: 4, candidate: CandidateId(9), stamp }
    }

    #[test]
    fn a_stale_authority_cannot_reserve() {
        let p = proposal(ConfigDomain::Keepout, 5);
        let cur = Authority { lease_generation: 5, epoch: 2, stop_generation: 3, ..Authority::default() };
        let now = Instant::now();
        assert_eq!(p.check(ArmId::LOCAL, BootId(1), now, &cur), None);
        assert_eq!(p.check(ArmId::LOCAL, BootId(2), now, &cur), Some("WRONG_TARGET"));
        assert_eq!(p.check(ArmId::LOCAL, BootId(1), p.stamp.deadline + Duration::from_millis(1), &cur), Some("DEADLINE_EXPIRED"));
        assert_eq!(p.check(ArmId::LOCAL, BootId(1), now, &Authority { stop_generation: 4, ..cur }), Some("STOPPED"));
        assert_eq!(p.check(ArmId::LOCAL, BootId(1), now, &Authority { epoch: 3, ..cur }), Some("EPOCH_MISMATCH"));
        assert_eq!(p.check(ArmId::LOCAL, BootId(1), now, &Authority { lease_generation: 0, ..cur }), Some("LEASE_LOST"));
        // a proposal without a lease is not refused because someone holds one
        assert_eq!(proposal(ConfigDomain::Keepout, 0).check(ArmId::LOCAL, BootId(1), now, &cur), None);
    }

    #[test]
    fn grants_are_unique_and_a_delayed_release_cannot_clear_a_retry() {
        let mut h = ConfigHold::default();
        let p = proposal(ConfigDomain::Keepout, 0);
        let first = h.grant(&p, 4).unwrap();
        assert!(h.grant(&p, 4).is_none(), "one hold at a time");
        assert_eq!(first.stamp().request, RequestId(1));
        assert_eq!(h.release(first), Released::Cleared);
        let retry = h.grant(&p, 4).unwrap();
        assert_ne!(retry.grant(), GrantId(1));
        // a stale token of the first grant (an identical-looking value) does not match
        let stale = Reservation { grant: GrantId(1), domain: p.domain, from: 4, candidate: p.candidate, inhibits_motion: true, stamp: p.stamp };
        assert_eq!(h.release(stale), Released::NotHeld);
        assert!(h.motion_inhibited(), "the retry still holds");
        assert!(h.activated(&retry));
        assert!(!h.motion_inhibited());
    }

    #[test]
    fn after_the_rename_a_failure_or_release_reconciles_and_keeps_motion_inhibited() {
        let mut h = ConfigHold::default();
        let p = proposal(ConfigDomain::Poses, 0);
        let r = h.grant(&p, 4).unwrap();
        assert!(!h.motion_inhibited(), "poses do not affect motion");
        // before persistence a failed activation hands the reservation back
        let Activated::Refused { reservation: r, .. } = h.activation_failed(r, "PERSIST_FAILED") else { panic!() };
        assert_eq!(h.persisted(&r, Durability::Volatile), Err("INVALID_VALUE"));
        h.persisted(&r, Durability::DurableUnconfirmed).unwrap();
        assert_eq!(h.held().unwrap().state, HoldState::PersistedAwaitingActivation { durability: Durability::DurableUnconfirmed });
        assert_eq!(h.activation_failed(r, "APPLY_FAILED"), Activated::Reconciling { code: "APPLY_FAILED" });
        assert_eq!(h.held().unwrap().state, HoldState::Reconciling);
        assert!(h.motion_inhibited(), "reconciling inhibits every domain");
        assert!(h.grant(&p, 4).is_none());
        assert!(h.finish_reconcile(ConfigDomain::Poses));
        // a release after the rename reconciles too
        let r = h.grant(&p, 5).unwrap();
        h.persisted(&r, Durability::Durable).unwrap();
        assert_eq!(h.release(r), Released::Reconciling);
        assert!(h.motion_inhibited());
    }

    #[test]
    fn restart_reconciliation_inhibits_until_finished() {
        let mut h = ConfigHold::default();
        assert!(h.begin_reconcile(ConfigDomain::Calibration, 7));
        assert!(h.motion_inhibited());
        assert!(!h.finish_reconcile(ConfigDomain::Keepout));
        assert!(h.finish_reconcile(ConfigDomain::Calibration));
        assert!(!h.motion_inhibited());
    }
}
