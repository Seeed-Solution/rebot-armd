//! Feetech STS3215 half-duplex RX framing, echo drain, line-poisoning, and reply correlation.

#[allow(unused_imports)]
use std::time::{Duration, Instant};

#[allow(unused_imports)]
use crate::driver::*;
#[allow(unused_imports)]
use crate::protocol::feetech;
#[allow(unused_imports)]
use crate::transport::{Transport, TransportError};

impl<T: Transport> FeetechArmDriver<T> {
}
