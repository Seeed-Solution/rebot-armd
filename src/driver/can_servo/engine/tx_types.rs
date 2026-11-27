//! Damiao bounded TX queue, pass accounting, refresh scheduler, and TX priority types.

#[allow(unused_imports)]
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
#[allow(unused_imports)]
use std::sync::Arc;
#[allow(unused_imports)]
use std::time::{Duration, Instant};

#[allow(unused_imports)]
use crate::driver::*;
#[allow(unused_imports)]
use crate::driver::can_servo::*;
#[allow(unused_imports)]
use crate::model::JointState;
#[allow(unused_imports)]
use crate::protocol::damiao::{self, DamiaoLimits};
#[allow(unused_imports)]
use crate::transport::{CanFrame, CanRxTime, CanTransport, TransportError, MAX_REALTIME_MONOTONIC_OFFSET_DRIFT_NS};

/// Transmit priority classes (lower value = higher priority):
/// 1. Transition from motion to stop / hold / freeze for ANY joint (including J7).
/// 2. Motion targets.
/// 3. Re-sends of an unchanged hold / freeze.
///
/// (Class 4: J7 `0xCC` refresh and other reads — sent only when Classes 1–3 have no pending work
/// and spare TX budget remains.)
pub(in crate::driver::can_servo) const TX_CLASS_STOP_TRANSITION: u8 = 1;
pub(in crate::driver::can_servo) const TX_CLASS_MOTION_TARGET: u8 = 2;
pub(in crate::driver::can_servo) const TX_CLASS_HOLD_RESEND: u8 = 3;
