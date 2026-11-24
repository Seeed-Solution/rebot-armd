//! Owner-routed RT request modules (stage 3 A-chain).
//!
//! Each domain keeps its request types, replies, retired payloads, and owner-side
//! install handlers in its own submodule so shared executor dispatch stays minimal.

pub mod config;
