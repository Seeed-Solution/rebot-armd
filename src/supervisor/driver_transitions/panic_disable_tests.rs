//! Unit tests for bounded panic disable and legacy torque timeout semantics.

use super::*;
use crate::driver::{ArmDriverError, TorqueOutcome, TorqueProgress};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Condvar, Mutex};

// scaffold-impl-0
impl ArmDriver for SelectiveMockDriver {
}

// scaffold-impl-1
impl crate::driver::BusRecoveryExt for SelectiveMockDriver {
}

// scaffold-impl-2
impl ArmDriver for GenericMockDriver {
}

// scaffold-impl-3
impl ArmDriver for ProbeThenGenericMockDriver {
}

// scaffold-impl-4
impl crate::driver::BusRecoveryExt for ProbeThenGenericMockDriver {
}

// scaffold-test-end
