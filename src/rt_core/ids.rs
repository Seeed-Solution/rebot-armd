//! Ownership identities and time meanings of the RT core.
//!
//! One armd process drives one arm on one bus. [`ArmId`] and [`BusId`] name the local
//! owner boundaries ([`super::ArmCore`] owns one arm's control state; one transmitter owns
//! one bus) without adding anything to observations, records or messages: an exporter that
//! needs a global identity can attach it at its own boundary.
//!
//! # Time values
//!
//! armd keeps its existing scalar representations; these aliases name what a value means.
//!
//! | Meaning | Type | Examples |
//! |---|---|---|
//! | local monotonic sample time | [`MonotonicS`] | `observation.timestamp`, `RecordFrame::timestamp`, `motion_limit.since_ms / 1000` |
//! | local monotonic deadline | [`LocalDeadline`] | snapshot validity, lease expiry, watchdog deadlines |
//! | simulation time | [`SimTimeS`] | `SupervisorInner::sim_time_s` (advanced by the tick's `dt` in sim) |
//! | wall time | [`WallTime`] | episode `recorded_at`, log lines |
//!
//! Local monotonic values are not comparable across processes or devices; mapping them into
//! another clock needs an offset and its uncertainty, which armd does not estimate.

use serde::{Deserialize, Serialize};

/// An arm owned by this armd process. One process owns one arm today: [`ArmId::LOCAL`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct ArmId(pub u16);

impl ArmId {
    /// The arm of a single-arm armd.
    pub const LOCAL: ArmId = ArmId(0);
}

/// A physical bus (CAN interface or serial port) with exactly one local transmitter.
/// One process owns one bus today: [`BusId::LOCAL`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct BusId(pub u16);

impl BusId {
    /// The bus of a single-arm armd.
    pub const LOCAL: BusId = BusId(0);
}

/// Which clock a time value belongs to (see the module table).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ClockDomain {
    /// This process's monotonic clock, zero at `SupervisorInner::start_time`.
    LocalMonotonic,
    /// Simulation time of `--follower sim`, advanced by the control tick's `dt`.
    Simulation,
    /// UTC wall clock.
    Wall,
}

/// Seconds on the local monotonic clock since this armd process started ([`ClockDomain::LocalMonotonic`]).
pub type MonotonicS = f64;

/// A deadline on the local monotonic clock.
#[cfg(not(target_arch = "wasm32"))]
pub type LocalDeadline = std::time::Instant;

/// Seconds of simulation time ([`ClockDomain::Simulation`]).
pub type SimTimeS = f64;

/// An RFC 3339 UTC timestamp ([`ClockDomain::Wall`]).
pub type WallTime = String;
