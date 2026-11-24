//! Owner-routed RT request modules (Stage 3 A-chain).
//!
//! Each domain keeps its request types, replies, retired payloads, and single-entry-point
//! owner install functions in its own submodule so shared broker/ops/executor dispatch stays
//! minimal and non-overlapping.

pub mod actions;
pub mod config;
