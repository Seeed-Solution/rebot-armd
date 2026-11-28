//! Long actions of the supervisor: action lifecycle, macro (grasp/place/sequence) sub-motions, gripper actions and zero check, table-capture stillness, action reconciliation and stepping (track E).
//!
//! Moved from `supervisor.rs` without changes (armd 1.6 RT-core foundation); private
//! methods are `pub(super)` so the rest of the supervisor keeps calling them.

#[allow(unused_imports)]
use super::*;
use crate::macro_action::{ChildTicket, ContactGuard, ContactWatch};

mod gripper_reconcile;
mod lifecycle;
mod macros;

impl SupervisorInner {
}
