//! Deadline-driven torque handshakes of the bus owner.
//!
//! A torque transition (enable or disable confirmation) is a state machine holding its phase,
//! the joint mask still unconfirmed, the retry cursor and an absolute deadline. The bus owner
//! serves it at most once per call: it reads feedback (bounded by the driver's RX budget),
//! reports which joints confirmed, and gets back what to do now (retransmit to a mask, wait,
//! finish). Nothing here sleeps or reads a clock: callers pass `now`, so the same machine runs
//! under the wall clock, a virtual test clock, or an executor that serves it once per release.

use std::time::{Duration, Instant};

/// Joints a mask can hold.
pub const MAX_MASK_JOINTS: usize = 32;

/// Mask with the lowest `n` bits set.
pub fn full_mask(n: usize) -> u32 {
    debug_assert!(n <= MAX_MASK_JOINTS);
    if n >= MAX_MASK_JOINTS { u32::MAX } else { (1u32 << n) - 1 }
}

/// Joint indices (0-based) set in `mask`.
pub fn mask_joints(mask: u32) -> impl Iterator<Item = usize> {
    (0..MAX_MASK_JOINTS).filter(move |i| mask & (1 << i) != 0)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TorqueDirection { Enable, Disable }

/// Result of one service of a [`ConfirmWindow`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Service {
    /// Nothing to send; serve again no later than `wake`.
    Wait { wake: Instant },
    /// Retransmit the command to the joints in `mask` now, then serve again no later than `wake`.
    Retry { mask: u32, wake: Instant },
    /// Every joint confirmed before the deadline.
    Confirmed,
    /// Every joint answered and joint `joint` reported a fault (enable only).
    Fault { joint: usize },
}

/// One confirmation window: the command went out at `fresh_after` (feedback older than that
/// never confirms), the window runs from `started` to `deadline` (exclusive), and the command is
/// resent to unconfirmed joints at each `retry_at` offset from `started`.
#[derive(Clone, Debug)]
pub struct ConfirmWindow {
    pub direction: TorqueDirection,
    pub fresh_after: Instant,
    pub started: Instant,
    pub deadline: Instant,
    all: u32,
    confirmed: u32,
    retry_cursor: usize,
    retry_at: &'static [Duration],
}

impl ConfirmWindow {
    pub fn new(direction: TorqueDirection, joints: usize, fresh_after: Instant, started: Instant, window: Duration, retry_at: &'static [Duration]) -> Self {
        Self { direction, fresh_after, started, deadline: started + window, all: full_mask(joints), confirmed: 0, retry_cursor: 0, retry_at }
    }

    /// Joints confirmed at the last service.
    pub fn confirmed(&self) -> u32 { self.confirmed }
    /// Joints not yet confirmed.
    pub fn missing(&self) -> u32 { self.all & !self.confirmed }
    /// Retries already sent.
    pub fn retries_sent(&self) -> usize { self.retry_cursor }

    /// Absolute instant of the next retry, if one is left.
    pub fn next_retry(&self) -> Option<Instant> { self.retry_at.get(self.retry_cursor).map(|d| self.started + *d) }

    /// When the bus owner must serve the window next: the next retry or the deadline.
    pub fn wake(&self) -> Instant { self.next_retry().map_or(self.deadline, |r| r.min(self.deadline)) }

    /// The window is over: no feedback read at or after the deadline confirms anything.
    pub fn expired(&self, now: Instant) -> bool { now >= self.deadline }

    /// Expiry, checked before the feedback of this service is read: the joints that never
    /// confirmed, once the deadline passed.
    pub fn expire(&self, now: Instant) -> Option<u32> {
        self.expired(now).then(|| self.missing())
    }

    /// Serve the window at `now` (after the feedback of this service was read): `confirmed`
    /// holds the joints whose fresh feedback confirms the transition, `all_fresh` says every
    /// joint answered after `fresh_after`, `fault` is a joint reporting a fault.
    pub fn service(&mut self, now: Instant, confirmed: u32, all_fresh: bool, fault: Option<usize>) -> Service {
        self.confirmed = confirmed & self.all;
        if self.direction == TorqueDirection::Enable && all_fresh {
            if let Some(joint) = fault { return Service::Fault { joint }; }
        }
        if self.all != 0 && self.confirmed == self.all { return Service::Confirmed; }
        if self.next_retry().is_some_and(|r| now >= r) {
            self.retry_cursor += 1;
            return Service::Retry { mask: self.missing(), wake: self.wake() };
        }
        Service::Wait { wake: self.wake() }
    }
}

/// Outcome of a finished torque transition.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TorqueOutcome {
    /// Fresh feedback from every motor confirmed the new state.
    Confirmed,
    /// Disable frames went out but the window ended without confirmation from every motor
    /// (callers check the enable state themselves; never success for an enable).
    Unconfirmed,
}

/// Progress of a non-blocking torque transition (`ArmDriver::begin_torque` / `poll_torque`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TorqueProgress {
    /// Still waiting; serve again (`poll_torque`) no later than `wake`.
    Pending { wake: Instant },
    Done(TorqueOutcome),
}

/// Translate one service of a driver transition to the RT contract (`HardwarePoll::torque`):
/// `Pending` keeps its wake, a confirmed transition is `Done`, and everything else is a
/// terminal code from `rt_core::ops::TERMINAL_CODES`. An unconfirmed disable is never success:
/// `FEEDBACK_UNCONFIRMED` (the caller keeps checking the enable state).
pub fn rt_progress(enabled: bool, r: Result<TorqueProgress, super::ArmDriverError>)
    -> Result<crate::rt_core::ops::TorqueProgress, &'static str> {
    use crate::rt_core::ops::TorqueProgress as Rt;
    use super::ArmDriverError as E;
    match r {
        Ok(TorqueProgress::Pending { wake }) => Ok(Rt::Pending { wake }),
        Ok(TorqueProgress::Done(TorqueOutcome::Confirmed)) => Ok(Rt::Done),
        Ok(TorqueProgress::Done(TorqueOutcome::Unconfirmed)) => Err("FEEDBACK_UNCONFIRMED"),
        Err(e) if super::is_estop_inhibited(&e) => Err("ESTOP_ACTIVE"),
        Err(E::TorqueOffWouldDrop | E::DisableRequestExpired) => Err("TORQUE_OFF_WOULD_DROP"),
        Err(E::Transport(_)) => Err("DRIVER_WRITE_FAILED"),
        Err(_) if enabled => Err("ENABLE_FAILED"),
        Err(E::HardwareFault(_)) => Err("FEEDBACK_UNCONFIRMED"),
        Err(_) => Err("HARDWARE_FAULT"),
    }
}

/// Bound of one service of a torque transition: the driver's receive pass (Damiao: 64 frames or
/// 5 ms), then at most one frame per joint and one flush.
pub const SERVICE_TX_FRAMES_PER_JOINT: usize = 1;

/// A wait with an absolute deadline (no sleep inside): the supervisor's disable confirmation
/// and the register-reply waits use it so a caller can serve them once per release.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Deadline { pub at: Instant }

impl Deadline {
    pub fn after(now: Instant, d: Duration) -> Self { Self { at: now + d } }
    pub fn passed(&self, now: Instant) -> bool { now >= self.at }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RETRY: [Duration; 2] = [Duration::from_millis(25), Duration::from_millis(75)];
    fn ms(n: u64) -> Duration { Duration::from_millis(n) }

    #[test]
    fn retries_follow_the_cursor_and_the_deadline_is_exclusive() {
        let t0 = Instant::now();
        let mut w = ConfirmWindow::new(TorqueDirection::Enable, 3, t0, t0, ms(150), &RETRY);
        assert_eq!(w.wake(), t0 + ms(25));
        assert_eq!(w.service(t0 + ms(10), 0b001, false, None), Service::Wait { wake: t0 + ms(25) });
        assert_eq!(w.service(t0 + ms(25), 0b001, false, None), Service::Retry { mask: 0b110, wake: t0 + ms(75) });
        // a late service still sends one retry per cursor step, never two at once
        assert_eq!(w.service(t0 + ms(140), 0b011, false, None), Service::Retry { mask: 0b100, wake: t0 + ms(150) });
        assert_eq!(w.service(t0 + ms(149), 0b011, false, None), Service::Wait { wake: t0 + ms(150) });
        assert_eq!(w.expire(t0 + ms(149)), None);
        assert_eq!(w.expire(t0 + ms(150)), Some(0b100));
        assert_eq!(w.retries_sent(), 2);
    }

    #[test]
    fn fault_needs_every_joint_fresh_and_only_fails_an_enable() {
        let t0 = Instant::now();
        let mut w = ConfirmWindow::new(TorqueDirection::Enable, 2, t0, t0, ms(150), &RETRY);
        assert!(matches!(w.service(t0, 0b01, false, Some(1)), Service::Wait { .. }));
        assert_eq!(w.service(t0, 0b01, true, Some(1)), Service::Fault { joint: 1 });
        let mut d = ConfirmWindow::new(TorqueDirection::Disable, 2, t0, t0, ms(150), &RETRY);
        assert!(matches!(d.service(t0, 0b01, true, Some(1)), Service::Wait { .. }));
        assert_eq!(d.service(t0, 0b11, true, None), Service::Confirmed);
    }

    #[test]
    fn rt_progress_never_reports_an_unconfirmed_transition_as_done() {
        use crate::rt_core::ops::{TorqueProgress as Rt, TERMINAL_CODES};
        use super::super::{ArmDriverError as E, ESTOP_INHIBITED};
        use crate::transport::TransportError;
        let t = Instant::now();
        assert_eq!(rt_progress(true, Ok(TorqueProgress::Pending { wake: t })), Ok(Rt::Pending { wake: t }));
        assert_eq!(rt_progress(false, Ok(TorqueProgress::Done(TorqueOutcome::Confirmed))), Ok(Rt::Done));
        let codes = [
            rt_progress(false, Ok(TorqueProgress::Done(TorqueOutcome::Unconfirmed))),
            rt_progress(true, Err(E::HardwareFault("x".into()))),
            rt_progress(false, Err(E::HardwareFault("x".into()))),
            rt_progress(false, Err(E::Protocol(ESTOP_INHIBITED.into()))),
            rt_progress(false, Err(E::Transport(TransportError::WouldBlock))),
            rt_progress(false, Err(E::Timeout(1))),
        ];
        assert_eq!(codes.map(|c| c.unwrap_err()), ["FEEDBACK_UNCONFIRMED", "ENABLE_FAILED", "FEEDBACK_UNCONFIRMED", "ESTOP_ACTIVE", "DRIVER_WRITE_FAILED", "HARDWARE_FAULT"]);
        for c in codes { assert!(TERMINAL_CODES.contains(&c.unwrap_err()), "{c:?} not a terminal code"); }
    }

    #[test]
    fn empty_arm_never_confirms() {
        let t0 = Instant::now();
        let mut w = ConfirmWindow::new(TorqueDirection::Disable, 0, t0, t0, ms(150), &RETRY);
        assert!(!matches!(w.service(t0, 0, true, None), Service::Confirmed));
    }
}
