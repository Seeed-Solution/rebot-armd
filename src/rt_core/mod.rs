//! Per-arm real-time core (armd 1.6 RT core).
//!
//! Step 1 of the migration (`docs` of the RT core design, section 6): the ownership
//! boundary of one arm, the state publication channel and the recording ring. Everything
//! still runs inside the control tick under the supervisor write lock; what changes is how
//! state leaves the tick:
//!
//! - [`publish`]: the tick publishes each tick's observation into a preallocated triple
//!   buffer; REST, WS, the black box and measurement polls read it without the supervisor
//!   lock (it replaces the 1.5.0 observation cache).
//! - [`record_ring`]: the tick appends recording samples into a preallocated 256-sample
//!   ring that a normal-priority thread drains.
//! - [`ids`]: the owner identities and the meaning of time values.
//!
//! Step 2 foundation (frozen contracts for tracks A-E, production adapters delegate to the
//! supervisor's existing behaviour):
//!
//! - [`broker`]: the command ring (32 descriptors), 8 generation-tagged plan slots, reply
//!   reservations; one ordinary commit per release.
//! - [`mailbox`]: the latest-target mailbox of servo / policy streams.
//! - [`urgent`]: E-stop gate and authorized-stop generations.
//! - [`config_iface`]: versioned configuration reservations.
//! - [`ops`]: the narrow operations on one arm's core (planning snapshot, commit primitive,
//!   consume target, config reservation, poll hardware) and the release-stepping fixture.

pub mod ids;
#[cfg(not(target_arch = "wasm32"))]
pub mod publish;
#[cfg(not(target_arch = "wasm32"))]
pub mod executor;
#[cfg(not(target_arch = "wasm32"))]
pub mod safe_hold;
#[cfg(not(target_arch = "wasm32"))]
pub mod reply_router;
#[cfg(not(target_arch = "wasm32"))]
pub mod record_ring;
#[cfg(not(target_arch = "wasm32"))]
pub mod sched;
#[cfg(not(target_arch = "wasm32"))]
pub mod scratch;
#[cfg(not(target_arch = "wasm32"))]
pub mod urgent;
#[cfg(not(target_arch = "wasm32"))]
pub mod triple;
#[cfg(not(target_arch = "wasm32"))]
pub mod broker;
#[cfg(not(target_arch = "wasm32"))]
pub mod mailbox;
#[cfg(not(target_arch = "wasm32"))]
pub mod config_iface;
#[cfg(not(target_arch = "wasm32"))]
pub mod ops;
#[cfg(not(target_arch = "wasm32"))]
pub mod requests;
#[cfg(not(target_arch = "wasm32"))]
pub mod session;
#[cfg(not(target_arch = "wasm32"))]
pub mod stream;

#[cfg(not(target_arch = "wasm32"))]
pub use core::{ArmCore, FeedbackState, MacroTerminal, ObservationBuffer};

#[cfg(not(target_arch = "wasm32"))]
mod core;
