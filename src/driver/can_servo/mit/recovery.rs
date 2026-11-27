//! Damiao MIT expiry state, deadline fence, and watchdog continuity holds.

use std::time::{Duration, Instant};

use crate::driver::*;
use crate::driver::can_servo::*;
use crate::protocol::damiao;
use crate::transport::{CanFrame, CanTransport};

impl<C: CanTransport> DamiaoArmDriver<C> {
    /// Why the MIT deadline fence tripped (None = not tripped).
    pub fn mit_fence_trip(&self) -> Option<String> { self.mit_fence_trip.as_ref().map(|(r, _)| r.to_string()) }


    /// Predictive fail-closed enable admission invariant:
    /// energized operation requires `max_frames >= configured_motors + 1`.
    #[inline]
    pub(in crate::driver::can_servo) fn check_watchdog_tx_budget_for_enable_inner(&self) -> Result<(), (usize, usize)> {
        let n = self.motor_ids.len();
        if n == 0 {
            return Ok(());
        }
        let required = n.saturating_add(1);
        if self.tx_budget.max_frames < required {
            Err((self.tx_budget.max_frames, required))
        } else {
            Ok(())
        }
    }

    /// Predictive fail-closed energized admission invariant:
    /// `max_frames >= energized_arm_axes + energized_j7 + 1` (checked at ModeSwitch & MIT recovery).
    #[inline]
    pub(in crate::driver::can_servo) fn check_watchdog_tx_budget_for_energized_inner(&self) -> Result<(), (usize, usize)> {
        let n = self.motor_ids.len().min(MAX_MODE_JOINTS);
        let pf_joint = self.active_pos_force_joint();
        let mut energized_arm = 0usize;
        let mut energized_j7 = 0usize;
        for i in 0..n {
            let energized = (i < 32 && (self.motor_energized_mask & (1u32 << i)) != 0)
                || self.motor_status.get(i).copied() == Some(1);
            if !energized {
                continue;
            }
            if pf_joint == Some(i) || self.base_mode(i) == DamiaoControlMode::PosForce {
                energized_j7 = 1;
            } else {
                energized_arm += 1;
            }
        }
        let total = energized_arm + energized_j7;
        if total == 0 {
            return Ok(());
        }
        let required = total.saturating_add(1);
        if self.tx_budget.max_frames < required {
            Err((self.tx_budget.max_frames, required))
        } else {
            Ok(())
        }
    }

    /// Single canonical due-axis eligibility predicate used identically for both TX slot
    /// reservation (`watchdog_continuity_reserved_mask`) and continuity-hold servicing.
    #[inline]
    #[allow(clippy::too_many_arguments)]
    pub(in crate::driver::can_servo) fn axis_continuity_due_age(
        &self,
        i: usize,
        now: Instant,
        min_age_override: Option<Duration>,
        confirmed_disabled_mask: u64,
        exclude_mask: u32,
        force_mask: u32,
        coalesce_release: bool,
        include_pos_force_j7: bool,
    ) -> Option<Duration> {
        if i >= self.motor_ids.len() || i >= MAX_MODE_JOINTS {
            return None;
        }
        let bit32 = 1u32 << i;
        if (exclude_mask & bit32) != 0 {
            return None;
        }
        if coalesce_release
            && ((self.release_ctrl_tx_mask & bit32) != 0
                || (self.staged_ctrl_tx_mask & bit32) != 0)
        {
            return None;
        }
        let mit_recovery = self.mit_expiry_recovery_active || self.mit_fence_trip.is_some();
        if mit_recovery {
            if i < 64 && (confirmed_disabled_mask & (1u64 << i)) != 0 {
                return None;
            }
        } else if i < 64 && (self.disable_dispatched_mask & (1u64 << i)) != 0 {
            return None;
        }
        let energized = (self.motor_energized_mask & bit32) != 0
            || self.motor_status.get(i).copied() == Some(1);
        if !energized {
            return None;
        }
        let pf_joint = self.active_pos_force_joint();
        let is_pos_force =
            pf_joint == Some(i) || self.base_mode(i) == DamiaoControlMode::PosForce;
        if is_pos_force {
            if !include_pos_force_j7 {
                return None;
            }
            let armed_mit = self.mit_fence.is_some() && self.mit_fence_trip.is_none();
            let in_mode_switch = self.mode_switch_active_mask != 0;
            if armed_mit || in_mode_switch {
                let age = self.motor_last_ctrl_tx[i]
                    .map_or(Duration::MAX, |t| now.saturating_duration_since(t));
                return Some(age);
            }
            return None;
        }
        self.verified_hold_mode(i)?;
        let armed_mit = self.mit_fence.is_some() && self.mit_fence_trip.is_none();
        let age = self.motor_last_ctrl_tx[i]
            .map_or(Duration::MAX, |t| now.saturating_duration_since(t));
        if (force_mask & bit32) != 0 {
            return Some(age);
        }
        if armed_mit && min_age_override.is_none() {
            return Some(age);
        }
        let threshold = min_age_override.unwrap_or(if mit_recovery {
            MIT_RECOVERY_CONTINUITY_HOLD_INTERVAL
        } else if self.mode_switch_active_mask != 0 {
            crate::motor_mode::DEFAULT_WATCHDOG_PROACTIVE_INTERVAL
        } else {
            WATCHDOG_DISABLE_FALLBACK_INTERVAL
        });
        if age >= threshold {
            Some(age)
        } else {
            None
        }
    }

    /// Bitmask of energized, uncovered axes whose watchdog-continuity slots are reserved FIRST
    /// inside `tx_budget.max_frames` before any other frame kind may spend capacity.
    #[inline]
    pub(in crate::driver::can_servo) fn watchdog_continuity_reserved_mask(&self) -> u32 {
        if !self.in_active_control_release() {
            return 0;
        }
        let n = self.motor_ids.len().min(MAX_MODE_JOINTS);
        let now = self.now();
        let confirmed_disabled = if self.mit_expiry_recovery_active {
            self.mit_expiry_confirmed_disabled_mask
        } else {
            0
        };
        let mut mask = 0u32;
        for i in 0..n {
            if self
                .axis_continuity_due_age(
                    i,
                    now,
                    None,
                    confirmed_disabled,
                    self.mode_write_uncertain_mask,
                    0,
                    true,
                    true,
                )
                .is_some()
            {
                mask |= 1u32 << i;
            }
        }
        mask
    }

    /// Interleave SAFE-HOLD-equivalent stationary continuity holds (`TxKind::MeasuredHold`) on
    /// every still-energized, unconfirmed, watchdog-bearing verified axis during MIT expiry
    /// recovery whenever its control-frame age reaches
    /// `MIT_RECOVERY_CONTINUITY_HOLD_INTERVAL` (<= 20 ms). Zero heap allocation.
    ///
    /// Uses the single canonical due-axis predicate (`axis_continuity_due_age`) and enforces the
    /// hard `tx_budget.max_frames` and `min(deadline, tx_release_deadline, pass.deadline)` ceilings.
    /// Never silently skips a due axis: any missing hold position, deferral, deadline miss,
    /// transport error, or flush failure returns `Err(MitRecoveryHoldErr::WatchdogMissed(joint))`.
    pub(in crate::driver::can_servo) fn service_mit_recovery_continuity_holds_inner(
        &mut self,
        n: usize,
        start_stamp: StopStamp,
        confirmed_disabled_mask: u64,
        deadline: Option<Deadline>,
        coalesce_release: bool,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> Result<bool, MitRecoveryHoldErr> {
        let now = self.now();
        let mut due = [(0usize, Duration::ZERO); MAX_MODE_JOINTS];
        let mut due_len = 0usize;
        for i in 0..n {
            if let Some(age) = self.axis_continuity_due_age(
                i,
                now,
                Some(MIT_RECOVERY_CONTINUITY_HOLD_INTERVAL),
                confirmed_disabled_mask,
                0,
                0,
                coalesce_release,
                false,
            ) {
                due[due_len] = (i, age);
                due_len += 1;
            }
        }
        if due_len == 0 {
            return Ok(false);
        }
        for idx in 1..due_len {
            let key = due[idx];
            let mut j = idx;
            while j > 0 && due[j - 1].1 < key.1 {
                due[j] = due[j - 1];
                j -= 1;
            }
            due[j] = key;
        }
        if !self.in_active_control_release() {
            self.begin_tx_tick();
        } else {
            let _ = self.ensure_tx_pass();
        }
        self.discard_staged_ctrl_tx();
        let mut sent_any = false;
        let mut missed_joint: Option<usize> = None;
        self.in_urgent_watchdog_hold = true;
        for &(i, _) in &due[..due_len] {
            if deadline.is_some_and(|d| d.passed(self.now())) {
                missed_joint.get_or_insert(i);
                break;
            }
            let Some(pos) = self.motor_keepalive_hold_pos[i].or_else(|| {
                self.cached_states
                    .get(i)
                    .map(|s| s.position)
                    .filter(|p| p.is_finite())
            }) else {
                missed_joint.get_or_insert(i);
                continue;
            };
            let target = JointTarget::position(pos);
            match self.send_with_commissioning_gate_with_bypass(
                TxKind::MeasuredHold(i),
                Some(start_stamp),
                deadline,
                ContinuityBypass::Ledger,
                pre_tx_gate,
                |d| d.build_verified_hold_frame(i, &target),
            ) {
                Ok(_) => sent_any = true,
                Err(Ok(code)) => {
                    self.in_urgent_watchdog_hold = false;
                    self.discard_staged_ctrl_tx();
                    return Err(MitRecoveryHoldErr::PreTxGate(code));
                }
                Err(Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged)) => {
                    self.in_urgent_watchdog_hold = false;
                    self.discard_staged_ctrl_tx();
                    return Err(MitRecoveryHoldErr::StopInhibited(i));
                }
                Err(Err(TxRefusal::Deferred | TxRefusal::DeadlinePassed)) => {
                    missed_joint.get_or_insert(i);
                    break;
                }
                Err(Err(TxRefusal::Transport(e))) if e.is_tx_backpressure() => {
                    missed_joint.get_or_insert(i);
                    break;
                }
                Err(Err(_)) => {
                    missed_joint.get_or_insert(i);
                }
            }
        }
        self.in_urgent_watchdog_hold = false;
        if sent_any {
            if let Err(e) = self.pending_stop_check(Some(start_stamp)) {
                self.discard_staged_ctrl_tx();
                return Err(MitRecoveryHoldErr::FlushInhibited(e));
            }
            if !self.can.flush_is_noop()
                && (self.flush_cutoff_exceeded() || deadline.is_some_and(|d| d.passed(self.now())))
            {
                self.discard_staged_ctrl_tx();
                return Err(MitRecoveryHoldErr::WatchdogMissed(due[0].0));
            }
            if let Err(e) = self.flush_can_and_commit() {
                self.discard_staged_ctrl_tx();
                let _ = e;
                return Err(MitRecoveryHoldErr::WatchdogMissed(due[0].0));
            }
        } else {
            self.discard_staged_ctrl_tx();
        }
        if let Some(j) = missed_joint {
            return Err(MitRecoveryHoldErr::WatchdogMissed(j));
        }
        Ok(sent_any)
    }

    pub(in crate::driver::can_servo) fn service_mit_recovery_continuity_holds(
        &mut self,
        n: usize,
        start_stamp: StopStamp,
        confirmed_disabled_mask: u64,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> Result<bool, String> {
        self.service_mit_recovery_continuity_holds_inner(
            n,
            start_stamp,
            confirmed_disabled_mask,
            None,
            false,
            pre_tx_gate,
        )
        .map_err(|err| match err {
            MitRecoveryHoldErr::PreTxGate(code) => format!("PRE_TX_GATE:{code}"),
            MitRecoveryHoldErr::StopInhibited(i) => format!(
                "continuity hold to J{} inhibited: {}",
                i + 1,
                ArmDriverError::EstopInhibited
            ),
            MitRecoveryHoldErr::FlushInhibited(e) => {
                format!("continuity hold flush inhibited: {e}")
            }
            MitRecoveryHoldErr::WatchdogMissed(i) => format!(
                "WATCHDOG_CONTINUITY_ABORT: failed to service continuity hold on J{}",
                i + 1
            ),
        })
    }

    /// Cap any sleep/wait during synchronous legacy MIT expiry recovery at
    /// `MIT_RECOVERY_MAX_WAIT_BETWEEN_HOLDS` (20 ms) and no longer than the remaining time until
    /// the oldest still-energized axis needs its next continuity hold.
    pub(in crate::driver::can_servo) fn mit_recovery_capped_wait(
        &self,
        n: usize,
        confirmed_disabled_mask: u64,
        requested: Duration,
    ) -> Duration {
        let mut cap = requested.min(MIT_RECOVERY_MAX_WAIT_BETWEEN_HOLDS);
        let now = self.now();
        let pf_joint = self.active_pos_force_joint();
        for i in 0..n {
            if i < 64 && (confirmed_disabled_mask & (1u64 << i)) != 0 {
                continue;
            }
            let energized =
                (i < 32 && (self.motor_energized_mask & (1u32 << i)) != 0) || self.motor_status[i] == 1;
            if !energized {
                continue;
            }
            if pf_joint == Some(i) || self.base_mode(i) == DamiaoControlMode::PosForce {
                continue;
            }
            if self.verified_hold_mode(i).is_none() {
                continue;
            }
            let age = self.motor_last_ctrl_tx[i]
                .map_or(Duration::MAX, |t| now.saturating_duration_since(t));
            let rem = MIT_RECOVERY_CONTINUITY_HOLD_INTERVAL.saturating_sub(age);
            cap = cap.min(rem);
        }
        cap.max(Duration::from_micros(500).min(requested))
            .min(MIT_RECOVERY_MAX_WAIT_BETWEEN_HOLDS)
    }

    #[inline]
    pub(in crate::driver::can_servo) fn is_mit_hold_eligible(&self, i: usize) -> bool {
        !self.mit_ownership_lost
            && self.mit_fence_trip.is_none()
            && !self.is_axis_mode_uncertain(i)
            && (self.mit_limits_verified.get(i).copied().unwrap_or(false)
                || self.control_mode == DamiaoControlMode::Mit)
    }

    pub(in crate::driver::can_servo) fn resolved_mit_params(&self, target: &JointTarget) -> (f32, f32, f32) {
        (
            target.kp.filter(|k| k.is_finite()).unwrap_or(self.default_kp),
            target.kd.filter(|k| k.is_finite()).unwrap_or(self.default_kd),
            target.torque.filter(|t| t.is_finite()).unwrap_or(0.0),
        )
    }

    pub(in crate::driver::can_servo) fn mit_frame(&self, i: usize, target: &JointTarget) -> CanFrame {
        let (kp, kd, torque) = self.resolved_mit_params(target);
        damiao::build_mit_frame(
            self.motor_ids[i],
            target.position,
            target.velocity.filter(|v| v.is_finite()).unwrap_or(0.0),
            kp,
            kd,
            torque,
            &self.limits[i],
        )
    }

    pub(in crate::driver::can_servo) fn trip_fence(&mut self, reason: MitFenceTripReason) {
        if self.mit_fence_trip.is_none() {
            self.mit_fence_trip = Some((reason, self.now()));
        }
        // nothing cached or queued may be replayed: the group's outputs are uncertain
        self.clear_cached_targets();
        self.refresh_scheduler.reset_sweep();
        self.discard_all_pending_tx();
    }

    /// Armed fence: every joint's feedback and the last complete batch younger than the deadline
    /// (zero allocation).
    pub(in crate::driver::can_servo) fn fence_check(&mut self) -> Result<(), ArmDriverError> {
        if let Some((reason, _)) = self.mit_fence_trip {
            return Err(ArmDriverError::MitFenceTripped(reason));
        }
        let Some(deadline) = self.mit_fence else { return Ok(()) };
        let now = self.now();
        // one comparison for every check: elapsed >= deadline trips
        let past = |d: Duration| d >= deadline;
        if let Some(i) = (0..self.motor_ids.len()).find(|&i| self.last_feedback_instant[i].is_none_or(|t| past(now.saturating_duration_since(t)))) {
            let age = self.last_feedback_instant[i].map(|t| now.saturating_duration_since(t));
            self.trip_fence(MitFenceTripReason::FeedbackStale { joint: (i + 1) as u8, age, deadline });
        } else if let Some(gap) = self.mit_fence_last_cmd.iter().flatten().min().map(|t| now.saturating_duration_since(*t)).filter(|g| past(*g)) {
            self.trip_fence(MitFenceTripReason::OutputGap { gap, deadline });
        }
        match self.mit_fence_trip {
            Some((reason, _)) => Err(ArmDriverError::MitFenceTripped(reason)),
            None => Ok(()),
        }
    }

    /// Zero-allocation watchdog keepalive hold pass for already-enabled non-`PosForce` arm joints.
    ///
    /// Transmits stationary `TxKind::MeasuredHold(i)` frames in each joint's verified hold mode,
    /// ordered oldest-unfed first, for every energized joint (not yet disabled) whose elapsed time
    /// since the last control frame (`POS_VEL`/`MIT`/`CMD_ENABLE`) is at least `min_age`.
    /// - Active `PosForce` J7 is excluded (`NoOutputUntilFresh`; J7 `TIMEOUT == 0` by design).
    /// - Respects `verified_hold_mode(i)`, MIT fence/ownership, stop gate, and tick TX budget.
    /// - Does NOT reset `rest_pose_window` (preserving K=3 causal rest-pose evidence).
    /// - When `abort_on_starve` is true (enable handshakes), fails closed with
    ///   [`ArmDriverError::WatchdogContinuityAbort`] if any energized arm joint still has
    ///   `age >= WATCHDOG_ENABLE_ABORT_AGE` (60 ms) after attempting holds, aborting cleanly BEFORE
    ///   Damiao's `TIMEOUT = 2000` (~100 ms) watchdog can expire with `0xD`.
    pub(in crate::driver::can_servo) fn service_urgent_enable_watchdog_holds(
        &mut self,
        exclude_mask: u32,
        reserve_slot_for_phase: bool,
    ) -> Result<(), ArmDriverError> {
        let n = self.motor_ids.len().min(MAX_MODE_JOINTS);
        // Only when a synthetic test explicitly configures `max_frames < dof` (impossible to both
        // enable and hold all arm motors in one tick) defer holds to the post-phase spare budget.
        if reserve_slot_for_phase && self.tx_budget.max_frames < n {
            return Ok(());
        }
        self.service_energized_watchdog_holds(
            WATCHDOG_HOLD_URGENT_INTERVAL,
            None,
            exclude_mask,
            true,
        )?;
        Ok(())
    }

    pub(in crate::driver::can_servo) fn service_energized_watchdog_holds(
        &mut self,
        min_age: Duration,
        deadline: Option<Deadline>,
        exclude_mask: u32,
        abort_on_starve: bool,
    ) -> Result<bool, ArmDriverError> {
        self.service_energized_watchdog_holds_with_targets(
            min_age,
            deadline,
            exclude_mask,
            None,
            abort_on_starve,
        )
    }

    /// Sends valid stationary hold frames for every armed axis in `active_mask & !covered_mask`
    /// under an armed MIT fence, bypassing per-release ledger tightening (`release_tx_limit`)
    /// and (when `complete_partial` is true) `tx_release_deadline` so watchdog continuity is
    /// never broken by ledger deferral or a mid-batch release-deadline/transient-TX interruption.
    ///
    /// Uses ONLY driver-authorized stationary hold references (never a pending motion target's
    /// position, gains, or feedforward torque) so a lease-exempt continuity hold cannot carry
    /// motion data past lease revocation.
    ///
    /// Still strictly enforces `fence_check()`, stop gates, verified hold modes, and
    /// `self.tx_budget.max_frames`.
    #[allow(clippy::too_many_arguments)]
    pub(in crate::driver::can_servo) fn send_mit_continuity_holds_for_uncovered(
        &mut self,
        active_mask: u64,
        covered_mask: u64,
        hold_positions: Option<&[Option<f32>]>,
        start_stamp: StopStamp,
        complete_partial: bool,
        started: &mut [Option<Instant>; 16],
        sent_any: &mut bool,
        sent_mask: &mut u64,
        sent_slot_mask: &mut u64,
    ) -> Result<(), (usize, TxRefusal)> {
        let hold_bypass = if complete_partial {
            ContinuityBypass::LedgerAndReleaseDeadline
        } else {
            ContinuityBypass::Ledger
        };
        if complete_partial {
            if let Some(pass) = self.tick_tx.as_mut() {
                pass.backpressured = false;
            }
        }
        let n = self.motor_ids.len().min(MAX_MODE_JOINTS);
        for i in 0..n {
            let bit_i = 1u64 << i;
            if (active_mask & bit_i) == 0 || (covered_mask & bit_i) != 0 {
                continue;
            }
            let pos_opt = hold_positions
                .and_then(|hp| hp.get(i).copied().flatten())
                .filter(|p| p.is_finite())
                .or_else(|| self.fresh_measured_position(i, STOPPED_DISABLE_FEEDBACK_MAX_AGE))
                .or(self.motor_keepalive_hold_pos[i])
                .or_else(|| {
                    self.cached_states
                        .get(i)
                        .map(|s| s.position)
                        .filter(|p| p.is_finite())
                });
            let Some(hold_pos) = pos_opt else {
                return Err((i, TxRefusal::BuildRefused));
            };
            let (kp, kd) = self
                .last_mit_params
                .get(i)
                .copied()
                .flatten()
                .map_or((None, None), |(kp, kd, _)| (Some(kp), Some(kd)));
            let hold_target = JointTarget {
                position: hold_pos,
                velocity: Some(0.0),
                kp,
                kd,
                torque: Some(0.0),
            };
            if started[i].is_none() {
                started[i] = Some(self.now());
            }
            let mut retried_transport = false;
            loop {
                match self.send_plain_with_bypass(
                    TxKind::MeasuredHold(i),
                    Some(start_stamp),
                    None,
                    true,
                    hold_bypass,
                    |d| d.build_verified_hold_frame(i, &hold_target),
                ) {
                    Ok(sent_frame) => {
                        let mit_params = self.resolved_mit_params(&hold_target);
                        self.record_sent_target(i, sent_frame, Some(mit_params));
                        *sent_any = true;
                        *sent_mask |= bit_i;
                        let bit = i.saturating_mul(2);
                        if bit < 64 {
                            *sent_slot_mask |= 1u64 << bit;
                        }
                        break;
                    }
                    Err(TxRefusal::Transport(e))
                        if complete_partial && !retried_transport && e.is_tx_backpressure() =>
                    {
                        if self.pending_stop_check(Some(start_stamp)).is_err() {
                            return Err((i, TxRefusal::StopInhibited));
                        }
                        if let Err(ArmDriverError::MitFenceTripped(reason)) = self.fence_check() {
                            return Err((i, TxRefusal::MitFenceTripped(reason)));
                        }
                        retried_transport = true;
                        if let Some(pass) = self.tick_tx.as_mut() {
                            pass.backpressured = false;
                        }
                    }
                    Err(r) => return Err((i, r)),
                }
            }
        }
        Ok(())
    }

    pub(in crate::driver::can_servo) fn service_energized_watchdog_holds_with_targets(
        &mut self,
        min_age: Duration,
        deadline: Option<Deadline>,
        exclude_mask: u32,
        hold_targets: Option<&[Option<f32>]>,
        abort_on_starve: bool,
    ) -> Result<bool, ArmDriverError> {
        if self.motor_energized_mask == 0 {
            return Ok(false);
        }
        // When an MIT fence is actively armed (untripped) outside re-arm (`!is_enabling`), never
        // silently decline watchdog continuity: emit valid stationary holds for all uncovered
        // armed axes (bypassing per-release ledger tightening) and update `mit_fence_last_cmd`.
        if self.mit_fence.is_some() && !self.is_enabling {
            if self.mit_fence_trip.is_some() {
                return Ok(false);
            }
            let n = self.motor_ids.len().min(MAX_MODE_JOINTS);
            let pf_joint = self.active_pos_force_joint();
            let now = self.now();
            let effective_exclude = exclude_mask | self.release_ctrl_tx_mask;
            let mut active_mask = 0u64;
            let mut any_due = false;
            for i in 0..n {
                if (self.motor_energized_mask & (1u32 << i)) == 0
                    || (effective_exclude & (1u32 << i)) != 0
                {
                    continue;
                }
                if i < 64 && (self.disable_dispatched_mask & (1u64 << i)) != 0 {
                    continue;
                }
                if pf_joint == Some(i) || self.base_mode(i) == DamiaoControlMode::PosForce {
                    continue;
                }
                if self.verified_hold_mode(i).is_none() {
                    continue;
                }
                active_mask |= 1u64 << i;
                let age = self.motor_last_ctrl_tx[i]
                    .map_or(Duration::MAX, |t| now.saturating_duration_since(t));
                if age >= min_age {
                    any_due = true;
                }
            }
            if active_mask == 0 || !any_due {
                return Ok(false);
            }
            if self.fence_check().is_err() || self.pending_stop_check(None).is_err() {
                return Ok(false);
            }
            let start_stamp = self.current_stop_stamp();
            let _ = self.ensure_tx_pass();
            self.discard_staged_ctrl_tx();
            let mut started = [None::<Instant>; 16];
            let mut sent_any = false;
            let mut sent_mask = 0u64;
            let mut sent_slot_mask = 0u64;
            self.in_urgent_watchdog_hold = true;
            let complete_partial = self.in_active_control_release();
            let send_res = self.send_mit_continuity_holds_for_uncovered(
                active_mask,
                0,
                hold_targets,
                start_stamp,
                complete_partial,
                &mut started,
                &mut sent_any,
                &mut sent_mask,
                &mut sent_slot_mask,
            );
            self.in_urgent_watchdog_hold = false;
            if send_res.is_err()
                || !sent_any
                || self.pending_stop_check(Some(start_stamp)).is_err()
                || self.flush_can_and_commit().is_err()
                || self.fence_check().is_err()
            {
                // On noop-flush transports (e.g. SocketCAN), any frame whose `send_plain` already
                // succeeded reached the wire: keep `mit_fence_last_cmd` and `record_flushed_joint_tx`
                // reconciled with actually transmitted frames while discarding any uncommitted staging.
                if self.can.flush_is_noop() && sent_mask != 0 {
                    for (i, t) in started.into_iter().enumerate().take(n) {
                        if t.is_some() && (sent_mask & (1u64 << i)) != 0 {
                            self.mit_fence_last_cmd[i] = t;
                            self.record_flushed_joint_tx(i, 0, false);
                        }
                    }
                }
                self.discard_staged_ctrl_tx();
                return Ok(false);
            }
            for (i, t) in started.into_iter().enumerate().take(n) {
                if t.is_some() && (sent_mask & (1u64 << i)) != 0 {
                    self.mit_fence_last_cmd[i] = t;
                    self.record_flushed_joint_tx(i, 0, false);
                }
            }
            return Ok(true);
        }
        let n = self.motor_ids.len().min(MAX_MODE_JOINTS);
        let pf_joint = self.active_pos_force_joint();
        let now = self.now();

        // Collect eligible energized joints whose control-frame age >= min_age and that have NOT
        // already received a confirmed control frame in this release, ordered oldest-unfed first
        // (largest elapsed age first). Zero heap allocation.
        let effective_exclude = exclude_mask | self.release_ctrl_tx_mask;
        let mut due = [(0usize, Duration::ZERO); MAX_MODE_JOINTS];
        let mut due_len = 0usize;
        for i in 0..n {
            if (self.motor_energized_mask & (1u32 << i)) == 0
                || (effective_exclude & (1u32 << i)) != 0
            {
                continue;
            }
            if i < 64 && (self.disable_dispatched_mask & (1u64 << i)) != 0 {
                continue;
            }
            if pf_joint == Some(i) || self.base_mode(i) == DamiaoControlMode::PosForce {
                continue;
            }
            if self.verified_hold_mode(i).is_none() {
                continue;
            }
            let age = self.motor_last_ctrl_tx[i]
                .map_or(Duration::MAX, |t| now.saturating_duration_since(t));
            if age >= min_age {
                due[due_len] = (i, age);
                due_len += 1;
            }
        }

        // Insertion sort descending by age (at most MAX_MODE_JOINTS = 16 elements).
        for idx in 1..due_len {
            let key = due[idx];
            let mut j = idx;
            while j > 0 && due[j - 1].1 < key.1 {
                due[j] = due[j - 1];
                j -= 1;
            }
            due[j] = key;
        }

        let start_stamp = self.current_stop_stamp();
        self.discard_staged_ctrl_tx();
        let mut sent_mask = 0u32;
        let mut aborted_flush = false;
        let urgent_continuity = abort_on_starve
            || (self.mit_expiry_recovery_active && due_len > 0)
            || due[..due_len]
                .iter()
                .any(|&(_, age)| age >= WATCHDOG_HOLD_URGENT_INTERVAL);
        let hold_bypass = if urgent_continuity {
            ContinuityBypass::Ledger
        } else {
            ContinuityBypass::None
        };
        self.in_urgent_watchdog_hold = true;
        for &(i, _) in &due[..due_len] {
            let Some(pos) = (match hold_targets {
                Some(targets) => targets.get(i).and_then(|&p| p).filter(|p| p.is_finite()),
                None => self.motor_keepalive_hold_pos[i].or_else(|| {
                    self.cached_states
                        .get(i)
                        .map(|s| s.position)
                        .filter(|p| p.is_finite())
                }),
            }) else {
                continue;
            };
            let target = JointTarget::position(pos);
            match self.send_plain_with_bypass(
                TxKind::MeasuredHold(i),
                Some(start_stamp),
                deadline,
                false,
                hold_bypass,
                |d| d.build_verified_hold_frame(i, &target),
            ) {
                Ok(_) => {
                    sent_mask |= 1u32 << i;
                }
                Err(TxRefusal::DeadlinePassed | TxRefusal::Deferred) => break,
                Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => break,
                Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                    aborted_flush = true;
                    break;
                }
                Err(TxRefusal::Transport(_)) => {
                    self.mark_tx_pass_backpressured();
                    aborted_flush = true;
                    break;
                }
                Err(_) => {}
            }
        }
        self.in_urgent_watchdog_hold = false;
        let mut sent_any = sent_mask != 0;
        if sent_any {
            if aborted_flush
                || self.pending_stop_check(Some(start_stamp)).is_err()
                || self.flush_cutoff_exceeded()
            {
                self.discard_staged_ctrl_tx();
                sent_any = false;
            } else if self.flush_can_and_commit().is_err() {
                sent_any = false;
            }
        } else {
            self.discard_staged_ctrl_tx();
        }

        if abort_on_starve && self.tx_budget.max_frames >= n {
            let after = self.now();
            for i in 0..n {
                if (self.motor_energized_mask & (1u32 << i)) == 0
                    || (exclude_mask & (1u32 << i)) != 0
                {
                    continue;
                }
                if i < 64 && (self.disable_dispatched_mask & (1u64 << i)) != 0 {
                    continue;
                }
                if pf_joint == Some(i) || self.base_mode(i) == DamiaoControlMode::PosForce {
                    continue;
                }
                if self.verified_hold_mode(i).is_none() {
                    continue;
                }
                if let Some(last_tx) = self.motor_last_ctrl_tx[i] {
                    let age = after.saturating_duration_since(last_tx);
                    if age >= WATCHDOG_ENABLE_ABORT_AGE {
                        let age_ms = age.as_millis().min(u128::from(u64::MAX)) as u64;
                        self.torque_hs = None;
                        self.enable_failed();
                        return Err(ArmDriverError::WatchdogContinuityAbort { joint: i, age_ms });
                    }
                }
            }
        }
        Ok(sent_any)
    }

    /// Earliest next wake instant needed to keep already-enabled arm joints refreshed within
    /// the active handshake phase's watchdog hold threshold (`WATCHDOG_DISABLE_FALLBACK_INTERVAL`
    /// during pre-dispatch disable when RX is drained, `WATCHDOG_HOLD_WAKE_INTERVAL` during enable).
    pub(in crate::driver::can_servo) fn next_watchdog_hold_wake(&self) -> Option<Instant> {
        if self.motor_energized_mask == 0 || (self.mit_fence.is_some() && !self.is_enabling) {
            return None;
        }
        let is_disable_pre_all = matches!(
            self.torque_hs,
            Some(DamiaoTorque::AwaitRestGate { .. } | DamiaoTorque::Disable { repeats_sent: 0, .. })
        );
        if is_disable_pre_all
            && (self.mit_fence_trip.is_some()
                || ((!self.last_read_drained || self.awaiting_rx_boundary)
                    && !(self.in_active_control_release() && self.driver_remaining_release_rx() == 0)))
        {
            return None;
        }
        let wake_interval = if is_disable_pre_all {
            WATCHDOG_DISABLE_FALLBACK_INTERVAL
        } else {
            WATCHDOG_HOLD_WAKE_INTERVAL
        };
        let n = self.motor_ids.len().min(MAX_MODE_JOINTS);
        let pf_joint = self.active_pos_force_joint();
        let mut earliest: Option<Instant> = None;
        for i in 0..n {
            if (self.motor_energized_mask & (1u32 << i)) == 0 {
                continue;
            }
            if i < 64 && (self.disable_dispatched_mask & (1u64 << i)) != 0 {
                continue;
            }
            if pf_joint == Some(i) || self.base_mode(i) == DamiaoControlMode::PosForce {
                continue;
            }
            if self.verified_hold_mode(i).is_none() {
                continue;
            }
            if let Some(last_tx) = self.motor_last_ctrl_tx[i] {
                let due = last_tx + wake_interval;
                earliest = Some(earliest.map_or(due, |e| e.min(due)));
            }
        }
        earliest
    }

}

/// Maximum unconsumed causal TX events (1 probe + up to 3 subsequent continuity holds) tracked
/// per axis within one MIT expiry probe window (zero alloc; overflow fails closed as ambiguous).
pub(in crate::driver::can_servo) const MIT_EXPIRY_MAX_CAUSES: usize = 4;

/// Per-axis sequential causal disable confirmation state for MIT expiry recovery (zero alloc).
///
/// SocketCAN timestamps host submission and `flush()` is a no-op, so replies cannot be correlated
/// by transport flush alone. Confirmation is strictly sequential per axis:
/// 1. `CMD_DISABLE` (`dis_tx`)
/// 2. Drain RX -> `REG_READ(RID_CTRL_MODE)` barrier (`barrier_tx` -> `barrier_ack`)
/// 3. Drain RX -> Round 1 `0xCC` status probe (`probe1_tx` -> `probe1_ack`, `state == 0`)
/// 4. Drain RX -> Round 2 `0xCC` status probe (`probe2_tx` -> `probe2_ack`, `state == 0`),
///    submitted strictly after `probe1_ack` (`p2_tx_at > p1_ack_at`).
///
/// Retries advance `min_next_fb_seq` and drain RX before submitting a fresh probe so a retry
/// never reuses a prior reply identity. Any feedback arriving strictly after `reply_window_start`
/// without a strictly earlier unconsumed probe/hold cause latches `reply_ambiguous = true` and
/// invalidates the axis (both before and after `probe2_ack` is latched).
#[derive(Debug, Clone, Copy, Default)]
pub(in crate::driver::can_servo) struct MitExpiryAxisState {
    pub(in crate::driver::can_servo) dis_tx: Option<(u64, Instant)>,
    pub(in crate::driver::can_servo) barrier_tx: Option<(u64, Instant, Option<i128>)>,
    pub(in crate::driver::can_servo) barrier_ack: Option<(u64, Instant)>,
    pub(in crate::driver::can_servo) probe1_tx: Option<(u64, Instant, Option<i128>)>,
    pub(in crate::driver::can_servo) probe1_ack: Option<(u64, Instant)>,
    pub(in crate::driver::can_servo) probe2_tx: Option<(u64, Instant, Option<i128>)>,
    pub(in crate::driver::can_servo) probe2_ack: Option<(u64, Instant)>,
    pub(in crate::driver::can_servo) min_next_fb_seq: u64,
    pub(in crate::driver::can_servo) reply_window_start: Option<Instant>,
    pub(in crate::driver::can_servo) pending_causes: [Option<Instant>; MIT_EXPIRY_MAX_CAUSES],
    pub(in crate::driver::can_servo) pending_causes_len: u8,
    pub(in crate::driver::can_servo) reply_ambiguous: bool,
}

impl MitExpiryAxisState {
    #[inline]
    pub(in crate::driver::can_servo) fn clear_reply_window(&mut self) {
        self.reply_window_start = None;
        self.pending_causes = [None; MIT_EXPIRY_MAX_CAUSES];
        self.pending_causes_len = 0;
        self.reply_ambiguous = false;
    }

    #[inline]
    pub(in crate::driver::can_servo) fn open_reply_window(&mut self, probe_tx_at: Instant) {
        self.reply_window_start = Some(probe_tx_at);
        self.pending_causes = [None; MIT_EXPIRY_MAX_CAUSES];
        self.pending_causes[0] = Some(probe_tx_at);
        self.pending_causes_len = 1;
        self.reply_ambiguous = false;
    }

    #[inline]
    pub(in crate::driver::can_servo) fn record_hold_cause(&mut self, hold_at: Instant) {
        if let Some(w) = self.reply_window_start {
            if hold_at >= w {
                let len = self.pending_causes_len as usize;
                if len < MIT_EXPIRY_MAX_CAUSES {
                    self.pending_causes[len] = Some(hold_at);
                    self.pending_causes_len = (len + 1) as u8;
                } else {
                    self.reply_ambiguous = true;
                }
            }
        }
    }

    #[inline]
    pub(in crate::driver::can_servo) fn observe_feedback(&mut self, rx_at: Instant) {
        if let Some(w) = self.reply_window_start {
            if rx_at > w {
                let len = self.pending_causes_len as usize;
                if len > 0 && self.pending_causes[0].is_some_and(|c| c < rx_at) {
                    for k in 1..len {
                        self.pending_causes[k - 1] = self.pending_causes[k];
                    }
                    self.pending_causes[len - 1] = None;
                    self.pending_causes_len = (len - 1) as u8;
                } else {
                    self.reply_ambiguous = true;
                }
            }
        }
    }

    #[inline]
    pub(in crate::driver::can_servo) fn observe_untimestamped_feedback(&mut self) {
        if self.reply_window_start.is_some() {
            self.reply_ambiguous = true;
        }
    }

    #[inline]
    pub(in crate::driver::can_servo) fn reset_probes(&mut self, seen_fb_seq: u64) {
        self.probe1_tx = None;
        self.probe1_ack = None;
        self.probe2_tx = None;
        self.probe2_ack = None;
        self.clear_reply_window();
        self.min_next_fb_seq = self.min_next_fb_seq.max(seen_fb_seq);
    }

    #[inline]
    pub(in crate::driver::can_servo) fn reset_attempt(&mut self, seen_fb_seq: u64) {
        self.dis_tx = None;
        self.barrier_tx = None;
        self.barrier_ack = None;
        self.reset_probes(seen_fb_seq);
    }
}

/// Proactive interval between continuity holds on energized axes during synchronous legacy MIT
/// expiry recovery (12 ms reserves >= 8 ms headroom for bounded RX/TX slices so per-axis
/// hold-to-hold gaps stay <= 20 ms even when RX/TX advances time, well below Damiao's 100 ms
/// hardware watchdog).
pub(in crate::driver::can_servo) const MIT_RECOVERY_CONTINUITY_HOLD_INTERVAL: Duration = Duration::from_millis(12);
/// Maximum single wait/sleep duration between continuity holds during synchronous legacy MIT
/// expiry recovery (<= 20 ms).
pub(in crate::driver::can_servo) const MIT_RECOVERY_MAX_WAIT_BETWEEN_HOLDS: Duration = Duration::from_millis(12);
