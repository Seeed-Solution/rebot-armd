//! WS replay baseline harness: seeded-state setup, recorder/replayer against `armd --follower mock`
//! (and controlled-clock in-process runner), and equivalence-preserving transcript comparator.
#![allow(dead_code)]

pub mod comparator;
pub mod runner;
pub mod scenarios;
pub mod seed;
