//! Damiao bounded TX queue, pass accounting, refresh scheduler, and TX priority types.

use std::time::{Duration, Instant};

use crate::driver::*;
use crate::driver::can_servo::*;
use crate::transport::TransportError;

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

#[derive(Clone, Copy, Debug)]
pub(in crate::driver::can_servo) struct TxProbeState {
    pub(in crate::driver::can_servo) joint: usize,
    pub(in crate::driver::can_servo) boundary_seq: u64,
    pub(in crate::driver::can_servo) sent_at: Instant,
}

#[derive(Clone, Copy, Debug)]
pub(in crate::driver::can_servo) struct TickTxPass {
    pub(in crate::driver::can_servo) deadline: Instant,
    pub(in crate::driver::can_servo) attempts: usize,
    pub(in crate::driver::can_servo) backpressured: bool,
}

/// Generation stamp captured when a motion frame is admitted into [`PendingTxQueue`].
/// Any E-stop or authorized stop (even after `requested == applied`) changes this stamp so
/// older deferred frames are never transmitted.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(in crate::driver) struct StopStamp {
    pub(in crate::driver::can_servo) estop_gen: u64,
    pub(in crate::driver::can_servo) stop_gen: u64,
}

/// Classification of every Damiao CAN transmit routed through [`DamiaoArmDriver::send`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(in crate::driver) enum TxKind {
    /// Requested torque-disable frame (`CMD_DISABLE`): exempt from the motion TX budget;
    /// unconditionally subject in every safety state to send-time K=3 rest-pose evaluation from
    /// fresh (`<= 100 ms`) feedback and the single 1 s request lifetime.
    DisableRequested { started_at: Instant },
    /// Budgeted MIT expiry recovery disable (`CMD_DISABLE`): subject to BOTH the shared
    /// per-release TX budget (`is_disable_tx_kind` returns `false`) AND Rule A′ (`started_at`).
    MitRecoveryDisable { started_at: Instant },
    /// Per-motor disable frame of a client-initiated park (`CMD_DISABLE`): exempt from the motion
    /// TX budget, subject to the exact same K=3 rest-pose gate and 1 s request lifetime as
    /// `DisableRequested`, AND stop-gated (pending Stop/E-stop, E-stop latch, and stop-generation
    /// stamp rechecked immediately before EVERY motor's `can.transmit`).
    ClientParkDisable { started_at: Instant },
    /// Protective torque-off (`CMD_DISABLE`): exempt from the motion TX budget and never
    /// refused by the stopped rest-pose rule or 1 s lifetime. Used ONLY for `Thermal` and
    /// `ProcessExit`.
    DisableProtective { reason: DisableProtectiveReason },
    /// Read-only `0x7FF 0xCC` status probe during `AwaitRestGate` while collecting K=3 rest-pose
    /// evidence: exempt from motion TX budget and stop gate.
    DisableProbe,
    /// Motion target frame (`POS_VEL`, `MIT`, or `POS_FORCE`).
    Target,
    /// Non-rollback mode-switch hold frame.
    Hold,
    /// Base-mode rollback or post-E-stop stationary hold frame for `joint`: refused while a
    /// stop/E-stop is pending; allowed while latched only when `joint` has fresh (`<= 100 ms`)
    /// measured feedback and the hold frame is rebuilt in step 4. Never used for `CTRL_MODE`
    /// register writes (those are always `ModeWrite`).
    RollbackHold(usize),
    /// Owner-captured measured-position stationary hold frame for `joint` (emitted by
    /// `write_measured_hold_masked`): obeys the same stop gate as `RollbackHold` and never resets
    /// `rest_pose_window`, while continuing to hold the owner's last captured finite position if
    /// feedback later goes stale (`JointHoldStatus::HeldStaleTarget`).
    MeasuredHold(usize),
    /// `CTRL_MODE` write or non-read-only configuration write (`RID_TIMEOUT`, `CMD_SET_ZERO`).
    ModeWrite,
    /// Read-only status poll (`0xCC` refresh) or read-only register read: refused while a stop
    /// or E-stop is pending, allowed while latched.
    Refresh,
    /// Autonomous stale-feedback recovery hold in `receive_feedback`.
    Recovery,
    /// Hardware fault-clear frame (`CMD_CLEAR_FAULTS`): refused while a stop or E-stop is
    /// pending, allowed while latched (part of operator-authorized fault clearance).
    ClearFault,
    /// Torque-enable handshake frame (`CMD_ENABLE`, enable-time restore/readback/refresh/hold).
    Handshake,
}

/// Zero-allocation refusal/failure outcome returned by [`DamiaoArmDriver::send`].
#[derive(Debug)]
pub(in crate::driver) enum TxRefusal {
    /// Step 1: caller deadline already passed (`deadline.passed(now)`).
    DeadlinePassed,
    /// Step 2: per-tick TX budget exhausted (frame count, pass duration cutoff, or backpressure).
    Deferred,
    /// Step 3: stop or E-stop gate refused this `TxKind` (pending or latched).
    StopInhibited,
    /// Step 3/5: `DisableRequested` while stopped refused by Rule A′ (not at rest / stale / no rest pose).
    TorqueOffWouldDrop,
    /// Step 3/5: `DisableRequested` while stopped is waiting ONLY for an exhausted RX drain or
    /// incomplete K=3 rest-pose evidence window inside the 1 s request lifetime (L1).
    DisableWaitEvidence,
    /// Step 3/5: `DisableRequested` while stopped exceeded its 1 s total request lifetime.
    DisableRequestExpired,
    /// Step 4 (pre-build): active MIT deadline fence tripped.
    MitFenceTripped(MitFenceTripReason),
    /// Step 4/5: active `PosForce` joint feedback aged past `POS_FORCE_FEEDBACK_MAX_AGE` (100 ms)
    /// before/at submission (`can.transmit`); distinct from FIFO `DeadlinePassed` and `BuildRefused`.
    PosForceFeedbackExpired(usize),
    /// Step 4: send-time `build` returned `None` (or latched rollback lacked fresh feedback).
    BuildRefused,
    /// Step 5: stop/E-stop generation stamp changed since admission or operation start.
    StopStampChanged,
    /// Step 3/5: motion or `CMD_ENABLE` frame's client lease generation was fenced
    /// (disconnect/release/preempt/expiry).
    LeaseFenced,
    /// Step 5: `CMD_ENABLE` frame's request/lease authority deadline expired before CAN submission
    /// (terminal; distinct from per-release `DeadlinePassed`).
    EnableAuthorityExpired,
    /// Step 6: underlying CAN transport returned an error (backpressure already marked on pass).
    Transport(TransportError),
}

/// Unencoded per-joint target parameters stored in [`PendingTxQueue`] so deferred frames are
/// always built at send time inside [`DamiaoArmDriver::send`] (never resending a cached frame).
#[derive(Clone, Copy, Debug)]
pub(in crate::driver::can_servo) enum PendingTargetCmd {
    PosVel { position: f32 },
    Mit { position: f32, velocity: Option<f32>, kp: Option<f32>, kd: Option<f32>, torque: Option<f32> },
    RollbackHold { kp: Option<f32>, kd: Option<f32>, torque: Option<f32>, slot: u8 },
    MeasuredHold { position: f32, kp: Option<f32>, kd: Option<f32>, torque: Option<f32> },
}

#[derive(Clone, Copy, Debug)]
pub(in crate::driver::can_servo) struct PendingTxFrame {
    pub(in crate::driver::can_servo) joint: usize,
    pub(in crate::driver::can_servo) frame_id: u32,
    pub(in crate::driver::can_servo) cmd: PendingTargetCmd,
    pub(in crate::driver::can_servo) queued_at: Instant,
    pub(in crate::driver::can_servo) stop_stamp: StopStamp,
    /// Client lease generation under which this motion target was produced (`0` for hold/autonomous).
    pub(in crate::driver::can_servo) lease_gen: u64,
    /// Counted once in `tx_deferred` per command instance; reset on in-place target replacement.
    pub(in crate::driver::can_servo) was_deferred: bool,
}

/// Inline fixed-size FIFO queue of unsent target frames (zero heap allocation).
pub(in crate::driver::can_servo) struct PendingTxQueue {
    pub(in crate::driver::can_servo) buf: [Option<PendingTxFrame>; DAMIAO_PENDING_TX_CAPACITY],
    pub(in crate::driver::can_servo) len: usize,
}

impl PendingTxQueue {
    pub(in crate::driver::can_servo) const fn new() -> Self {
        Self { buf: [None; DAMIAO_PENDING_TX_CAPACITY], len: 0 }
    }

    #[inline]
    pub(in crate::driver::can_servo) fn len(&self) -> usize { self.len }

    #[inline]
    pub(in crate::driver::can_servo) fn front(&self) -> Option<PendingTxFrame> {
        if self.len > 0 { self.buf[0] } else { None }
    }

    pub(in crate::driver::can_servo) fn pop_front(&mut self) -> Option<PendingTxFrame> {
        if self.len == 0 { return None; }
        let head = self.buf[0];
        for i in 1..self.len {
            self.buf[i - 1] = self.buf[i];
        }
        self.len -= 1;
        self.buf[self.len] = None;
        head
    }

    /// Peek the highest-priority stopped-hold entry ordered by
    /// `(priority_class, slot_last_tx_seq[joint * 2 + slot])` (Class 1 transition before Class 3
    /// unchanged-hold re-send; within each class, oldest waiting slot first, FIFO tie-break).
    pub(in crate::driver::can_servo) fn peek_best_hold(
        &self,
        slot_in_unchanged_hold: &[bool; MAX_MODE_JOINTS * 2],
        slot_last_tx_seq: &[u64; MAX_MODE_JOINTS * 2],
    ) -> Option<(usize, PendingTxFrame, u8, u64)> {
        let mut best: Option<(usize, PendingTxFrame, u8, u64)> = None;
        for (idx, slot_opt) in self.buf[..self.len].iter().enumerate() {
            let Some(entry) = *slot_opt else { continue };
            let sub = match entry.cmd {
                PendingTargetCmd::RollbackHold { slot, .. } => usize::from(slot).min(1),
                _ => 0,
            };
            let flat = entry.joint.saturating_mul(2).saturating_add(sub);
            let class = if flat < slot_in_unchanged_hold.len() && slot_in_unchanged_hold[flat] {
                TX_CLASS_HOLD_RESEND
            } else {
                TX_CLASS_STOP_TRANSITION
            };
            let seq = slot_last_tx_seq.get(flat).copied().unwrap_or(0);
            let better = match best {
                None => true,
                Some((_, _, best_class, best_seq)) => (class, seq) < (best_class, best_seq),
            };
            if better {
                best = Some((idx, entry, class, seq));
            }
        }
        best
    }

    pub(in crate::driver::can_servo) fn remove_at(&mut self, idx: usize) -> Option<PendingTxFrame> {
        if idx >= self.len {
            return None;
        }
        let item = self.buf[idx];
        for i in (idx + 1)..self.len {
            self.buf[i - 1] = self.buf[i];
        }
        self.len -= 1;
        self.buf[self.len] = None;
        item
    }

    pub(in crate::driver::can_servo) fn clear(&mut self) -> usize {
        let dropped = self.len;
        for slot in &mut self.buf[..self.len] {
            *slot = None;
        }
        self.len = 0;
        dropped
    }

    pub(in crate::driver::can_servo) fn discard_joint(&mut self, joint: usize) -> usize {
        let mut write = 0usize;
        let mut dropped = 0usize;
        for read in 0..self.len {
            if let Some(entry) = self.buf[read] {
                if entry.joint == joint {
                    dropped += 1;
                } else {
                    self.buf[write] = Some(entry);
                    write += 1;
                }
            }
        }
        for slot in &mut self.buf[write..self.len] {
            *slot = None;
        }
        self.len = write;
        dropped
    }

    pub(in crate::driver::can_servo) fn find_key(&self, joint: usize, frame_id: u32) -> Option<usize> {
        self.buf[..self.len].iter().position(|slot| {
            slot.is_some_and(|e| e.joint == joint && e.frame_id == frame_id)
        })
    }

    /// Drop entries with `now - queued_at >= max_age` (inclusive), compacting in place.
    /// Returns `(dropped_count, oldest_expired_age)`.
    pub(in crate::driver::can_servo) fn drop_stale(&mut self, now: Instant, max_age: Duration) -> (usize, Option<Duration>) {
        let mut write = 0usize;
        let mut dropped = 0usize;
        let mut oldest: Option<Duration> = None;
        for read in 0..self.len {
            if let Some(entry) = self.buf[read] {
                let age = now.saturating_duration_since(entry.queued_at);
                if age >= max_age {
                    dropped += 1;
                    oldest = Some(oldest.map_or(age, |o| o.max(age)));
                } else {
                    self.buf[write] = Some(entry);
                    write += 1;
                }
            }
        }
        for slot in &mut self.buf[write..self.len] {
            *slot = None;
        }
        self.len = write;
        (dropped, oldest)
    }

    /// Maximum age among currently queued entries, or `None` when empty.
    pub(in crate::driver::can_servo) fn oldest_age(&self, now: Instant) -> Option<Duration> {
        self.buf[..self.len]
            .iter()
            .flatten()
            .map(|e| now.saturating_duration_since(e.queued_at))
            .max()
    }

    /// Drop entries whose stop generation stamp no longer matches `current`, compacting in place.
    pub(in crate::driver::can_servo) fn drop_stopped(&mut self, current: StopStamp) -> usize {
        let mut write = 0usize;
        let mut dropped = 0usize;
        for read in 0..self.len {
            if let Some(entry) = self.buf[read] {
                if entry.stop_stamp != current {
                    dropped += 1;
                } else {
                    self.buf[write] = Some(entry);
                    write += 1;
                }
            }
        }
        for slot in &mut self.buf[write..self.len] {
            *slot = None;
        }
        self.len = write;
        dropped
    }

    /// True if any queued entry carries a stop generation stamp older than `current`.
    pub(in crate::driver::can_servo) fn has_stopped_entry(&self, current: StopStamp) -> bool {
        self.buf[..self.len]
            .iter()
            .any(|slot| slot.is_some_and(|e| e.stop_stamp != current))
    }

    /// Drop entries whose client lease generation has been fenced, compacting in place.
    pub(in crate::driver::can_servo) fn drop_fenced_lease(&mut self, mut is_fenced: impl FnMut(u64) -> bool) -> usize {
        let mut write = 0usize;
        let mut dropped = 0usize;
        for read in 0..self.len {
            if let Some(entry) = self.buf[read] {
                if is_fenced(entry.lease_gen) {
                    dropped += 1;
                } else {
                    self.buf[write] = Some(entry);
                    write += 1;
                }
            }
        }
        for slot in &mut self.buf[write..self.len] {
            *slot = None;
        }
        self.len = write;
        dropped
    }

    /// Remove every queued entry targeting `joint`, compacting in place (zero allocation).
    pub(in crate::driver::can_servo) fn remove_joint(&mut self, joint: usize) {
        let mut write = 0usize;
        for read in 0..self.len {
            if let Some(entry) = self.buf[read] {
                if entry.joint != joint {
                    self.buf[write] = Some(entry);
                    write += 1;
                }
            }
        }
        for slot in &mut self.buf[write..self.len] {
            *slot = None;
        }
        self.len = write;
    }

    /// Drop every queued entry targeting `joint`, compacting in place (0 heap allocs).
    pub(in crate::driver::can_servo) fn drop_joint(&mut self, joint: usize) -> usize {
        let mut write = 0usize;
        let mut dropped = 0usize;
        for read in 0..self.len {
            if let Some(entry) = self.buf[read] {
                if entry.joint == joint {
                    dropped += 1;
                } else {
                    self.buf[write] = Some(entry);
                    write += 1;
                }
            }
        }
        for slot in &mut self.buf[write..self.len] {
            *slot = None;
        }
        self.len = write;
        dropped
    }

    /// True if any queued entry targets `joint`.
    pub(in crate::driver::can_servo) fn has_joint(&self, joint: usize) -> bool {
        self.buf[..self.len]
            .iter()
            .any(|slot| slot.is_some_and(|e| e.joint == joint))
    }

    /// Drop every queued entry whose joint bit is NOT set in `hold_mask`, compacting in place.
    pub(in crate::driver::can_servo) fn drop_outside_mask(&mut self, hold_mask: u64) -> usize {
        let mut write = 0usize;
        let mut dropped = 0usize;
        for read in 0..self.len {
            if let Some(entry) = self.buf[read] {
                let keep = entry.joint < 64 && (hold_mask & (1u64 << entry.joint)) != 0;
                if !keep {
                    dropped += 1;
                } else {
                    self.buf[write] = Some(entry);
                    write += 1;
                }
            }
        }
        for slot in &mut self.buf[write..self.len] {
            *slot = None;
        }
        self.len = write;
        dropped
    }

    /// Replace an existing `(joint, frame_id)` slot in place without moving it to the back
    /// (preserving oldest-first carry-over FIFO order and its original `queued_at` under the same
    /// `stop_stamp` so carry-over frames always drain first with the latest target value and
    /// coalescing cannot renew the 45 ms expiry deadline indefinitely), or push to the back if
    /// capacity permits. If an existing slot carried a different `stop_stamp`, it is removed and
    /// re-appended at the back so entries stay monotonically ordered oldest-first by `queued_at`.
    pub(in crate::driver::can_servo) fn upsert(
        &mut self,
        joint: usize,
        frame_id: u32,
        cmd: PendingTargetCmd,
        queued_at: Instant,
        stop_stamp: StopStamp,
        lease_gen: u64,
    ) -> bool {
        if let Some(idx) = self.find_key(joint, frame_id) {
            if let Some(prev) = self
                .buf[idx]
                .filter(|prev| prev.stop_stamp == stop_stamp && prev.lease_gen == lease_gen)
            {
                self.buf[idx] = Some(PendingTxFrame {
                    joint,
                    frame_id,
                    cmd,
                    queued_at: prev.queued_at,
                    stop_stamp,
                    lease_gen,
                    was_deferred: false,
                });
                return true;
            }
            for i in (idx + 1)..self.len {
                self.buf[i - 1] = self.buf[i];
            }
            self.len -= 1;
            self.buf[self.len] = None;
        }
        if self.len >= DAMIAO_PENDING_TX_CAPACITY {
            return false;
        }
        self.buf[self.len] = Some(PendingTxFrame {
            joint,
            frame_id,
            cmd,
            queued_at,
            stop_stamp,
            lease_gen,
            was_deferred: false,
        });
        self.len += 1;
        true
    }

    /// Mark every currently queued entry not yet counted as deferred; returns how many flipped.
    pub(in crate::driver::can_servo) fn mark_newly_deferred(&mut self) -> usize {
        let mut newly = 0usize;
        for slot in &mut self.buf[..self.len] {
            if let Some(entry) = slot.as_mut() {
                if !entry.was_deferred {
                    entry.was_deferred = true;
                    newly += 1;
                }
            }
        }
        newly
    }
}

/// Unified owner-local `0xCC` read-only status refresh scheduler for [`DamiaoArmDriver`].
///
/// Zero locks, zero heap allocations. Uses a fixed `u64` bitmask (`unfinished_mask` / `swept_mask`)
/// and round-robin `cursor` to serve every joint lacking recent replies (including J7 under
/// `NoOutputUntilFresh`) across enable transitions, `FailedHold`, `Idle`-torqued, and stopped
/// states, enforcing at most ONE `0xCC` slot per tick (`slot_used_this_tick`).
#[derive(Debug, Default)]
pub(in crate::driver::can_servo) struct RefreshScheduler {
    pub(in crate::driver::can_servo) unfinished_mask: u64,
    pub(in crate::driver::can_servo) swept_mask: u64,
    pub(in crate::driver::can_servo) cursor: usize,
    pub(in crate::driver::can_servo) last_sweep_completed: Option<Instant>,
    pub(in crate::driver::can_servo) slot_used_this_tick: bool,
}

impl RefreshScheduler {
    #[inline]
    pub(in crate::driver::can_servo) fn begin_tick(&mut self) {
        self.slot_used_this_tick = false;
    }

    #[inline]
    pub(in crate::driver::can_servo) fn reset_sweep(&mut self) {
        self.unfinished_mask = 0;
        self.swept_mask = 0;
        self.cursor = 0;
    }

    #[inline]
    pub(in crate::driver::can_servo) fn clear_joint(&mut self, joint: usize) {
        if joint < 64 {
            let bit = 1u64 << joint;
            self.unfinished_mask &= !bit;
            self.swept_mask &= !bit;
        }
    }

    #[inline]
    pub(in crate::driver::can_servo) fn slot_available(&self) -> bool {
        !self.slot_used_this_tick
    }

    /// Consume this tick's single `0xCC` slot (used by causal enable `RefreshCapture` and
    /// post-freeze J7 refreshes so they share the 1-`0xCC`-per-tick cap without perturbing
    /// the round-robin sweep cursor or masks).
    #[inline]
    pub(in crate::driver::can_servo) fn mark_slot_used(&mut self) {
        self.slot_used_this_tick = true;
    }

    /// Select the next stale joint to receive this tick's single `0xCC` slot, or `None` if no
    /// slot is available, no joint is stale, or every stale joint was already polled within
    /// `cooldown`. Newly stale joints (not in `swept_mask`) are served immediately without
    /// waiting for an unrelated sweep cooldown.
    pub(in crate::driver::can_servo) fn pick_next(
        &mut self,
        stale_mask: u64,
        n: usize,
        now: Instant,
        cooldown: Duration,
    ) -> Option<usize> {
        let valid_mask = if n == 0 {
            0
        } else if n >= 64 {
            u64::MAX
        } else {
            (1u64 << n) - 1
        };
        let stale = stale_mask & valid_mask;
        // Any joint that received fresh feedback leaves both masks immediately.
        // If a previously polled joint recovered (replied), the bus is actively answering so do not
        // hold remaining stale joints behind the silent-sweep cooldown.
        if (self.swept_mask & !stale) != 0 {
            self.last_sweep_completed = None;
        }
        self.swept_mask &= stale;
        self.unfinished_mask &= stale;
        if stale == 0 || !self.slot_available() {
            return None;
        }
        let cooldown_ready = self
            .last_sweep_completed
            .is_none_or(|t| now.saturating_duration_since(t) >= cooldown);
        let candidates = if self.unfinished_mask != 0 {
            self.unfinished_mask
        } else if cooldown_ready {
            self.swept_mask = 0;
            stale
        } else {
            let new_stale = stale & !self.swept_mask;
            if new_stale != 0 {
                new_stale
            } else {
                return None;
            }
        };
        let start = self.cursor % n;
        for offset in 0..n {
            let i = (start + offset) % n;
            if (candidates & (1u64 << i)) != 0 {
                self.unfinished_mask = candidates;
                return Some(i);
            }
        }
        None
    }

    /// Record that `joint`'s `0xCC` frame transmitted and flushed in this tick.
    #[inline]
    pub(in crate::driver::can_servo) fn commit_sent(&mut self, joint: usize, n: usize, now: Instant) {
        self.slot_used_this_tick = true;
        if joint < 64 {
            let bit = 1u64 << joint;
            self.unfinished_mask &= !bit;
            self.swept_mask |= bit;
        }
        if n > 0 {
            self.cursor = (joint + 1) % n;
        }
        if self.unfinished_mask == 0 {
            self.last_sweep_completed = Some(now);
        }
    }

    /// Consume this tick's `0xCC` slot without advancing `cursor` or clearing `unfinished_mask`
    /// when TX budget/backpressure deferred transmission, so the next tick retries the same joint.
    #[inline]
    pub(in crate::driver::can_servo) fn record_deferred(&mut self) {
        self.slot_used_this_tick = true;
    }

    /// Advance past a joint that failed with a non-retryable transmit error so other stale joints
    /// are not starved on subsequent ticks.
    #[inline]
    pub(in crate::driver::can_servo) fn skip_non_retryable(&mut self, joint: usize, n: usize, now: Instant) {
        self.commit_sent(joint, n, now);
    }
}

/// Read-only `0xCC` status refresh cadence / staleness threshold for joints lacking recent replies.
pub(in crate::driver::can_servo) const DAMIAO_REFRESH_AFTER: Duration = Duration::from_millis(100);

/// Proactive watchdog hold cadence for already-enabled arm joints during multi-tick handshakes:
/// refreshes every 50 Hz tick (20 ms >= 15 ms) whenever spare TX budget remains.
pub(in crate::driver::can_servo) const WATCHDOG_HOLD_PROACTIVE_INTERVAL: Duration = Duration::from_millis(15);
/// Urgent watchdog hold threshold: strictly below one 50 Hz release (20 ms) so every release
/// during a multi-tick enable refreshes already-enabled arm joints FIRST before handshake TX,
/// guaranteeing <= 40 ms between control frames even with jitter or a 1-tick TX budget squeeze.
pub(in crate::driver::can_servo) const WATCHDOG_HOLD_URGENT_INTERVAL: Duration = Duration::from_millis(18);
/// Sub-release wake threshold for watchdog continuity: strictly above one 50 Hz release (20 ms)
/// so healthy 50 Hz releases service energized motors once per release without scheduling a
/// redundant Window 2 sleep at t + 16 ms, while still waking before 40 ms if a release misses TX
/// or runs at a slower period.
pub(in crate::driver::can_servo) const WATCHDOG_HOLD_WAKE_INTERVAL: Duration = Duration::from_millis(22);
/// Remaining release budget margin required to start optional (deferrable) CAN work such as
/// background `0xCC` status polls or autonomous recovery frames without pushing a release past
/// its deadline.
pub(crate) const OPTIONAL_WORK_RELEASE_MARGIN: Duration = Duration::from_millis(6);
/// Maximum autonomous stale-feedback recovery TX frames transmitted inside `receive_feedback`
/// during a single managed/release-bounded control release (round-robin across joints).
pub(in crate::driver::can_servo) const DAMIAO_MAX_RECOVERY_PER_RELEASE: u8 = 2;
/// Fallback watchdog hold threshold during `AwaitRestGate` / `Disable` Phase A (`repeats_sent == 0`):
/// strictly above one 50 Hz control tick (20 ms) so normal ticked control (whose Step 3 already
/// runs `service_stopped_measured_hold` every 20 ms) never emits duplicate holds, while blocking
/// `disable_torque` and headless fallback still keep enabled motors refreshed within <= 40 ms.
pub(in crate::driver::can_servo) const WATCHDOG_DISABLE_FALLBACK_INTERVAL: Duration = Duration::from_millis(30);
/// Fail-closed enable abort threshold: if a sustained multi-tick TX squeeze prevents refreshing an
/// energized arm joint for >= 60 ms during enable, abort the handshake cleanly into `FailedHold`
/// BEFORE Damiao's `TIMEOUT = 2000` (~100 ms) can expire with `0xD`.
pub(in crate::driver::can_servo) const WATCHDOG_ENABLE_ABORT_AGE: Duration = Duration::from_millis(60);
