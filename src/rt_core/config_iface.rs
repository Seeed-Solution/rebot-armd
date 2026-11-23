//! Versioned configuration interface between configuration workers and the RT (armd 1.6
//! RT core, step 2 foundation; design section 4, last paragraph; track B).
//!
//! A worker validates and stages a complete candidate off the RT, then asks the RT to
//! **reserve** the domain at the revision the candidate was built from. The reservation
//! succeeds only at that revision and, when the domain affects motion, inhibits incompatible
//! motion until it is activated or released. The worker persists the candidate; the RT
//! **activates** it (the active revision advances) and the caller receives the outcome.
//! Every message carries the identity, the authority it was prepared under and its original
//! deadline ([`ConfigProposal::stamp`]).
//!
//! Production today: [`super::ops`] implements [`ConfigAuthority`] for the supervisor with
//! the revision check of the existing commit path; activation stays on that path (the
//! adapter refuses `activate` with `NOT_WIRED`) until track B moves it.

use super::broker::Stamp;

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

/// A staged candidate's identity (assigned by the worker).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CandidateId(pub u64);

/// A request to reserve `domain` at `expected_revision` for `candidate`.
#[derive(Debug, Clone, Copy)]
pub struct ConfigProposal {
    pub domain: ConfigDomain,
    pub expected_revision: u64,
    pub candidate: CandidateId,
    pub stamp: Stamp,
}

/// A granted reservation: the domain is held at `from`; activation moves it to `from + 1`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Reservation {
    pub domain: ConfigDomain,
    pub from: u64,
    pub candidate: CandidateId,
    /// Motion incompatible with the change is inhibited while the reservation is held.
    pub inhibits_motion: bool,
}

/// Durability of a candidate when it is activated.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Durability {
    /// Written and renamed into place (directory synced).
    Durable,
    /// Not persisted (no file configured).
    Volatile,
}

/// Outcome of [`ConfigAuthority::reserve`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reserved {
    Granted(Reservation),
    /// `REVISION_MISMATCH` (`current` differs), `CONFIG_BUSY` (another reservation),
    /// `DEADLINE_EXPIRED`, authority codes of [`super::broker::Authority::admits`].
    Refused { code: &'static str, current: u64 },
}

/// Outcome of [`ConfigAuthority::activate`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Activated {
    Active { revision: u64, durability: Durability },
    Refused { code: &'static str },
}

/// The RT's side of versioned configuration.
pub trait ConfigAuthority {
    /// The active revision of `domain`.
    fn revision(&self, domain: ConfigDomain) -> u64;
    fn reserve(&mut self, proposal: &ConfigProposal, now: std::time::Instant) -> Reserved;
    fn activate(&mut self, reservation: Reservation, durability: Durability) -> Activated;
    /// Gives a reservation up (staging or persistence failed); motion is no longer inhibited.
    fn release(&mut self, reservation: Reservation);
}
