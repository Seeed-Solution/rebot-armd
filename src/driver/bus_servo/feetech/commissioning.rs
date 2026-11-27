//! Feetech STS3215 direct register access and cold servo commissioning.

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
