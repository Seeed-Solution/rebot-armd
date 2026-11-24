//! Per-family RT request definitions and single-entry-point owner install functions.
//!
//! Each parallel RT migration track adds its own module here so shared broker/ops/executor
//! edits stay minimal and non-overlapping.

pub mod actions;
