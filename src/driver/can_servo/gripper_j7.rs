//! Damiao J7 position-force gripper types, sealed `pos_force_emitter`, and `PosForceExt`.

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

impl<C: CanTransport> DamiaoArmDriver<C> {
}

/// Capability-checked transmit kind for [`DamiaoArmDriver::send`].
///
/// Plain [`TxKind`] values always return `None` from [`Self::pos_force_token`], so
/// `DamiaoArmDriver::send` rejects any `0x307` frame with `TxRefusal::BuildRefused`. Only
/// private `mod pos_force_emitter` can construct a [`PosForceSendToken`] and supply
/// `Some(&PosForceSendToken)` via `pos_force_emitter::authorize_kind`.
pub(in crate::driver::can_servo) trait DamiaoTxKind {
    fn tx_kind(&self) -> TxKind;
    fn pos_force_token(&self) -> Option<&pos_force_emitter::PosForceSendToken> {
        None
    }
}

impl DamiaoTxKind for TxKind {
    #[inline]
    fn tx_kind(&self) -> TxKind {
        *self
    }
}

/// Zero-allocation error returned by `DamiaoArmDriver::service_mit_recovery_continuity_holds_inner`.
pub(in crate::driver::can_servo) enum MitRecoveryHoldErr {
    PreTxGate(&'static str),
    StopInhibited(usize),
    FlushInhibited(ArmDriverError),
    WatchdogMissed(usize),
}

/// Explicit per-call watchdog-continuity bypass mode (NEVER ambient state on `DamiaoArmDriver`).
/// Valid ONLY for stationary Hold/Stop frames (`TxKind::Hold`, `TxKind::RollbackHold`,
/// `TxKind::MeasuredHold`); ignored (forced to `None`) for all motion (`Target`) and other kinds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(in crate::driver) enum ContinuityBypass {
    /// Normal per-release ledger and release cutoff enforcement.
    #[default]
    None,
    /// Bypass per-release ledger tightening (`release_tx_limit`), keeping `tx_budget.max_frames`
    /// and `tx_release_deadline`.
    Ledger,
    /// Bypass both per-release ledger tightening and `tx_release_deadline` (and allow bounded
    /// `watchdog_continuity_recovery_budget` slots) for non-deferrable continuity holds.
    LedgerAndReleaseDeadline,
}

impl ContinuityBypass {
    #[inline]
    pub(in crate::driver::can_servo) const fn bypasses_ledger(self) -> bool {
        !matches!(self, Self::None)
    }

    #[inline]
    pub(in crate::driver::can_servo) const fn bypasses_release_deadline(self) -> bool {
        matches!(self, Self::LedgerAndReleaseDeadline)
    }

    #[inline]
    pub(in crate::driver::can_servo) const fn for_tx_kind(self, kind: TxKind) -> Self {
        match kind {
            TxKind::Hold | TxKind::RollbackHold(_) | TxKind::MeasuredHold(_) => self,
            _ => Self::None,
        }
    }
}

/// Private single-emitter module for Damiao J7 `PosForce` (`0x300 + motor_id`) frames.
///
/// Type-system enforcement:
/// - [`PosForceSendToken`] has a private unit field `()` and is neither `Copy` nor `Clone`; it can
///   ONLY be constructed inside this private module.
/// - `damiao::build_pos_force_frame` is `pub(crate)` and requires `&PosForceSendToken`, so NO
///   module in the crate (not even `driver.rs` outside this submodule) can construct a `0x307`
///   `CanFrame` via the protocol builder.
/// - `DamiaoArmDriver::send_inner` / `send_with_retry_inner` reject any `0x307` frame with
///   `TxRefusal::BuildRefused` unless presented with `Some(&PosForceSendToken)`, and
///   `DamiaoArmDriver::send` always passes `None`.
/// - Only four narrow `pub(super)` entry points are exposed to `DamiaoArmDriver`:
///   1. [`emit_tick`]: the sole control-tick J7 emitter (invoked once per tick from the tick
///      epilogue on both Legacy and RT Executor owners, plus direct-driver fallback).
///   2. [`emit_pre_enable_hold`]: pre-enable stationary hold during torque-on handshake right
///      after `CTRL_MODE` readback (`allow_freeze = false`).
///   3. [`emit_mode_switch_hold`]: immediate hold directly following a `CTRL_MODE` register write
///      (`allow_freeze = false`).
///   4. [`emit_standalone_recovery_hold`]: standalone-driver recovery only (refuses with
///      `BuildRefused` whenever `external_pos_force_emitter == true`; `allow_freeze = false`).
pub(in crate::driver) mod pos_force_emitter {
    use super::*;

    /// Unconstructible capability token required both to build a Damiao `PosForce` (`0x307`) CAN
    /// frame via `damiao::build_pos_force_frame` and to transmit it via [`DamiaoArmDriver::send`].
    pub(crate) struct PosForceSendToken(());

    pub(in crate::driver::can_servo) struct AuthorizedTxKind<'a>(&'a PosForceSendToken, TxKind);

    impl<'a> super::DamiaoTxKind for AuthorizedTxKind<'a> {
        #[inline]
        fn tx_kind(&self) -> TxKind {
            self.1
        }
        #[inline]
        fn pos_force_token(&self) -> Option<&PosForceSendToken> {
            Some(self.0)
        }
    }

    #[inline]
    pub(in crate::driver::can_servo) fn authorize_kind<'a>(
        token: &'a PosForceSendToken,
        kind: TxKind,
    ) -> AuthorizedTxKind<'a> {
        AuthorizedTxKind(token, kind)
    }

    /// Sole production call site of `damiao::build_pos_force_frame` in `src/`.
    #[inline]
    fn build_wire_frame(
        token: &PosForceSendToken,
        motor_id: u16,
        pos: f32,
        vel_raw: u16,
        cur_raw: u16,
    ) -> CanFrame {
        damiao::build_pos_force_frame(
            token,
            damiao::POS_FORCE_BASE_ID,
            motor_id,
            pos,
            vel_raw,
            cur_raw,
        )
    }

    /// Private `0x307` (`POS_FORCE_BASE_ID + motor_id`) frame builder and transmitter.
    ///
    /// Decision table from `(intent, freshness)`:
    /// - `fresh + Target { position, velocity, current_raw }` -> capped target frame
    ///   (`vel_raw >= 1`, `i_des <= pf.current_raw`; fails closed if non-finite, `vel_raw == 0`,
    ///   or `i_des == 0`).
    /// - `fresh + Hold | Stop` -> stationary hold at current fresh measured J7 position,
    ///   `vel_raw = 0`, `i_des = pf.current_raw`.
    /// - `stale` (`allow_freeze == true`) -> FREEZE / documented boot-fallback parameters.
    /// - `stale` (`allow_freeze == false`) -> fails closed (`BuildRefused` /
    ///   `PosForceFeedbackExpired`), emitting 0 frames.
    #[allow(clippy::too_many_arguments)]
    fn send_frame<C: CanTransport>(
        driver: &mut DamiaoArmDriver<C>,
        kind: TxKind,
        expected_stamp: Option<StopStamp>,
        expected_lease_gen: u64,
        intent: PosForceIntentKind,
        allow_freeze: bool,
        retry: bool,
        fenced: bool,
        deadline: Option<Deadline>,
        continuity_bypass: ContinuityBypass,
    ) -> Result<CanFrame, TxRefusal> {
        let effective_bypass = match intent {
            PosForceIntentKind::Hold | PosForceIntentKind::Stop => {
                continuity_bypass.for_tx_kind(kind)
            }
            PosForceIntentKind::Target { .. } => ContinuityBypass::None,
        };
        let token = PosForceSendToken(());
        let build = |d: &DamiaoArmDriver<C>| -> Option<CanFrame> {
            let pf = d.pos_force.as_ref()?;
            let i = pf.joint;
            let motor_id = *d.motor_ids.get(i)?;
            let now = d.now();
            let (pos, vel_raw, cur_raw) = if d.pos_force_feedback_valid(i, now) {
                match intent {
                    PosForceIntentKind::Target { position, velocity, current_raw } => {
                        if !position.is_finite() || velocity.is_some_and(|v| !v.is_finite()) {
                            return None;
                        }
                        let measured =
                            d.fresh_measured_position_at(i, POS_FORCE_FEEDBACK_MAX_AGE, now)?;
                        let requested = velocity
                            .filter(|v| *v != 0.0)
                            .map_or(d.vlim(), f32::abs)
                            .min(d.vlim());
                        let strictly_opening =
                            (position - measured) * (pf.closed_rad - pf.open_rad) < 0.0;
                        let speed = if strictly_opening {
                            requested
                        } else {
                            requested.min(pf.close_speed_rad_s)
                        };
                        let vel_raw = damiao::pos_force_vel_raw(speed);
                        if vel_raw == 0 {
                            return None;
                        }
                        let cur = current_raw.map_or(pf.current_raw, |c| c.min(pf.current_raw));
                        if cur == 0 {
                            return None;
                        }
                        (position, vel_raw, cur)
                    }
                    PosForceIntentKind::Hold | PosForceIntentKind::Stop => {
                        let measured =
                            d.fresh_measured_position_at(i, POS_FORCE_FEEDBACK_MAX_AGE, now)?;
                        (measured, 0u16, pf.current_raw)
                    }
                }
            } else if allow_freeze {
                d.pos_force_freeze_params(i)?
            } else {
                return None;
            };
            Some(build_wire_frame(&token, motor_id, pos, vel_raw, cur_raw))
        };
        if retry {
            driver.send_with_retry_inner(kind, expected_stamp, deadline, fenced, Some(&token), build)
        } else {
            driver.send_with_lease_and_bypass(
                authorize_kind(&token, kind),
                expected_stamp,
                expected_lease_gen,
                deadline,
                fenced,
                effective_bypass,
                build,
            )
        }
    }

    /// Narrow pre-enable hold API: called ONLY from `DamiaoArmDriver::finish_pos_force_readback`
    /// while the motor is still disabled right after `CTRL_MODE` readback (`allow_freeze = false`).
    /// Emits a zero-velocity stationary `Hold` (`vel_raw = 0`, configured `current_raw`) at the
    /// fresh measured J7 position so the first `0x307` after fresh feedback on the torque-on path
    /// obeys `NoOutputUntilFresh -> Hold`.
    pub(in crate::driver::can_servo) fn emit_pre_enable_hold<C: CanTransport>(
        driver: &mut DamiaoArmDriver<C>,
        expected_stamp: Option<StopStamp>,
    ) -> Result<CanFrame, TxRefusal> {
        send_frame(
            driver,
            TxKind::Handshake,
            expected_stamp,
            0,
            PosForceIntentKind::Hold,
            false,
            false,
            false,
            None,
            ContinuityBypass::None,
        )
    }

    /// Narrow mode-switch hold API: called ONLY from `DamiaoArmDriver::write_modes_send` and
    /// `DamiaoArmDriver::write_modes_confirmed` directly after a `CTRL_MODE` register write
    /// (`allow_freeze = false`).
    pub(in crate::driver::can_servo) fn emit_mode_switch_hold<C: CanTransport>(
        driver: &mut DamiaoArmDriver<C>,
        joint: usize,
        is_rollback: bool,
        expected_stamp: Option<StopStamp>,
        intent: PosForceIntentKind,
        fenced: bool,
    ) -> Result<CanFrame, TxRefusal> {
        let kind = if is_rollback { TxKind::RollbackHold(joint) } else { TxKind::Hold };
        send_frame(
            driver,
            kind,
            expected_stamp,
            0,
            intent,
            false,
            true,
            fenced,
            None,
            ContinuityBypass::None,
        )
    }

    /// Narrow standalone-driver recovery API: called ONLY from `DamiaoArmDriver::receive_feedback`
    /// when `external_pos_force_emitter == false`. Refuses with `BuildRefused` whenever an
    /// external tick epilogue owner (`SupervisorInner`) owns J7 emission.
    pub(in crate::driver::can_servo) fn emit_standalone_recovery_hold<C: CanTransport>(
        driver: &mut DamiaoArmDriver<C>,
        expected_stamp: Option<StopStamp>,
        hold_pos: f32,
        speed: f32,
    ) -> Result<CanFrame, TxRefusal> {
        if driver.external_pos_force_emitter {
            return Err(TxRefusal::BuildRefused);
        }
        send_frame(
            driver,
            TxKind::Recovery,
            expected_stamp,
            0,
            PosForceIntentKind::Target {
                position: hold_pos,
                velocity: Some(speed),
                current_raw: None,
            },
            false,
            false,
            false,
            None,
            ContinuityBypass::None,
        )
    }

    /// Single control-tick J7 `PosForce` (`0x307`) emitter: called ONCE per control tick from the
    /// tick epilogue on both owners (Legacy and RT Executor), after whatever arm-write path ran
    /// (whether that arm path succeeded, refused, tripped the MIT fence, hit `BuildRefused`, had
    /// an invalid caller target, or returned early).
    ///
    /// Always runs unfenced (`fenced = false`) with `allow_freeze = true` so a tripped MIT fence
    /// suppresses ONLY MIT arm joints and J7 (`TIMEOUT=0`) always receives its resolved frame.
    ///
    /// Single stop-generation / E-stop epoch / lease-generation enforcement rule: any `Target`
    /// intent whose `(stop_gen, estop_epoch, lease_gen)` stamp is stale or lease-fenced is
    /// degraded to `Hold` and is NEVER re-stamped with a newer generation.
    ///
    /// `hold_bypass` is an explicit per-call parameter valid ONLY for stationary `Hold`/`Stop`
    /// frames (never applied to motion `Target` frames).
    pub(in crate::driver) fn emit_tick<C: CanTransport>(
        driver: &mut DamiaoArmDriver<C>,
        intent: PosForceIntent,
        current: PosForceIntentStamp,
        hold_bypass: ContinuityBypass,
    ) -> Result<PosForceEmitOutcome, ArmDriverError> {
        emit_tick_with_deadline(driver, intent, current, hold_bypass, None)
    }

    pub(in crate::driver) fn emit_tick_with_deadline<C: CanTransport>(
        driver: &mut DamiaoArmDriver<C>,
        intent: PosForceIntent,
        current: PosForceIntentStamp,
        hold_bypass: ContinuityBypass,
        tx_deadline: Option<Deadline>,
    ) -> Result<PosForceEmitOutcome, ArmDriverError> {
        let Some(pf_j) = driver.active_pos_force_joint() else {
            return Ok(PosForceEmitOutcome::NotApplicable);
        };
        let pf_already_disabled = pf_j < 64 && (driver.disable_dispatched_mask & (1u64 << pf_j)) != 0;
        let hs_allows_hold = matches!(
            driver.torque_hs,
            Some(
                DamiaoTorque::AwaitRestGate { .. }
                    | DamiaoTorque::Disable { repeats_sent: 0, .. }
            )
        ) && !matches!(intent.kind, PosForceIntentKind::Target { .. });
        let hs_blocks = driver.torque_hs.is_some() && !hs_allows_hold;
        if pf_already_disabled
            || hs_blocks
            || (!driver.is_enabled && matches!(intent.kind, PosForceIntentKind::Target { .. }))
        {
            return Ok(PosForceEmitOutcome::NotApplicable);
        }
        let stamp_stale = intent.stamp.is_stale_against(current)
            || driver.is_lease_fenced(intent.stamp.lease_gen);
        let effective_kind = match intent.kind {
            PosForceIntentKind::Target { .. }
                if stamp_stale || driver.pf_hold_on_first_fresh =>
            {
                PosForceIntentKind::Hold
            }
            other => other,
        };
        let mut is_target_intent = matches!(effective_kind, PosForceIntentKind::Target { .. });
        if is_target_intent {
            driver.strict_stop_check(None)?;
        } else {
            driver.pending_stop_check(None)?;
        }
        let start_stamp = driver.current_stop_stamp();
        let entry_now = driver.now();
        let _ = driver.check_pos_force_freshness_for_tx_at(entry_now);

        // Clear per-tick J7 hold hint after consulting it in the arm write pass.
        driver.pf_next_intent_is_hold = false;

        // Boot fallback (`PosForceBootFallback::NoOutputUntilFresh`): when NO provably fresh J7
        // sample has ever been decoded since this `armd` process started, send NO `0x307` frame
        // (J7 has `TIMEOUT=0`, so an existing grip stays gripped) while keeping `0xCC` refresh
        // polling active. Once the first fresh sample arrives, `pf_hold_on_first_fresh` ensures
        // the next emission is a stationary `Hold` at that measured position with `current_raw`.
        if !driver.pos_force_feedback_valid(pf_j, entry_now)
            && driver.pf_last_fresh_position.is_none()
        {
            match POS_FORCE_BOOT_FALLBACK {
                PosForceBootFallback::NoOutputUntilFresh => {
                    driver.pf_hold_on_first_fresh = true;
                    driver.pf_deferred_ticks = 0;
                    driver.pf_deferred_motion = false;
                    driver.pf_deferred_since = None;
                    driver.pf_deferred_intent = None;
                    let stop_check = |d: &DamiaoArmDriver<C>| {
                        if is_target_intent {
                            d.strict_stop_check(Some(start_stamp))
                        } else {
                            d.pending_stop_check(Some(start_stamp))
                        }
                    };
                    if driver.pending_hold_tx.len() == 0
                        && driver.pending_tx.len() == 0
                        && driver.pf_refresh_pending
                        && driver.refresh_scheduler.slot_available()
                        && driver.poll_tx_allowed()
                        && driver.can_start_tick_tx()
                        && driver.try_send_pf_refresh(start_stamp, tx_deadline)?
                    {
                        if stop_check(driver).is_err() {
                            return Err(ArmDriverError::EstopInhibited);
                        }
                        if driver.flush_cutoff_exceeded()
                            || (!driver.can.flush_is_noop()
                                && tx_deadline.is_some_and(|dl| dl.passed(driver.now())))
                        {
                            driver.mark_tx_pass_backpressured();
                            return Err(TransportError::Timeout.into());
                        }
                        if let Err(e) = driver.can.flush() {
                            driver.mark_tx_pass_backpressured();
                            if stop_check(driver).is_err() {
                                return Err(ArmDriverError::EstopInhibited);
                            }
                            return Err(e.into());
                        }
                        if stop_check(driver).is_err() {
                            return Err(ArmDriverError::EstopInhibited);
                        }
                    }
                    let pf_status = driver.take_last_pos_force_tx_status();
                    return Ok(PosForceEmitOutcome::AwaitingFirstFresh { joint: pf_j, pf_status });
                }
            }
        }

        let armed_mit = driver.mit_fence.is_some() && driver.mit_fence_trip.is_none();
        let mut active_kind = effective_kind;
        let mut retried = false;
        let mut degraded_target_to_continuity_hold = false;
        let sent_frame = loop {
            let (kind, target_lease_gen) = match active_kind {
                PosForceIntentKind::Target { .. } => (TxKind::Target, intent.stamp.lease_gen),
                PosForceIntentKind::Hold | PosForceIntentKind::Stop => (TxKind::RollbackHold(pf_j), 0),
            };
            // Only stationary Hold/Stop continuity frames receive `hold_bypass`; Target motion
            // frames ALWAYS run with `ContinuityBypass::None` and an explicit release cutoff
            // deadline checked immediately before `can.transmit`.
            let is_hold_intent = matches!(active_kind, PosForceIntentKind::Hold | PosForceIntentKind::Stop);
            let attempt_bypass = if is_hold_intent {
                hold_bypass
            } else {
                ContinuityBypass::None
            };
            let explicit_deadline = if attempt_bypass.bypasses_release_deadline() {
                tx_deadline
            } else {
                match (tx_deadline, driver.active_release_cutoff_deadline()) {
                    (Some(a), Some(b)) => Some(Deadline { at: a.at.min(b.at) }),
                    (a, b) => a.or(b),
                }
            };
            let send_res = send_frame(
                driver,
                kind,
                Some(start_stamp),
                target_lease_gen,
                active_kind,
                true,
                false,
                false,
                explicit_deadline,
                attempt_bypass,
            );
            match send_res {
                Ok(frame) => break Some(frame),
                Err(TxRefusal::LeaseFenced) => {
                    if !retried && matches!(active_kind, PosForceIntentKind::Target { .. }) {
                        retried = true;
                        active_kind = PosForceIntentKind::Hold;
                        is_target_intent = false;
                        continue;
                    }
                    break None;
                }
                Err(TxRefusal::PosForceFeedbackExpired(_)) => {
                    let _ = driver.check_pos_force_freshness_for_tx();
                    if !retried {
                        retried = true;
                        continue;
                    }
                    break None;
                }
                Err(TxRefusal::BuildRefused) => {
                    let _ = driver.check_pos_force_freshness_for_tx();
                    if !retried {
                        retried = true;
                        // An invalid/unencodable target never suppresses J7: degrade to Hold.
                        if matches!(active_kind, PosForceIntentKind::Target { .. }) {
                            active_kind = PosForceIntentKind::Hold;
                            is_target_intent = false;
                        }
                        continue;
                    }
                    break None;
                }
                Err(TxRefusal::Deferred | TxRefusal::DeadlinePassed) => {
                    if armed_mit
                        && !retried
                        && matches!(active_kind, PosForceIntentKind::Target { .. })
                    {
                        retried = true;
                        active_kind = PosForceIntentKind::Hold;
                        is_target_intent = false;
                        degraded_target_to_continuity_hold = true;
                        continue;
                    }
                    if !driver.pos_force_feedback_valid(pf_j, driver.now()) {
                        let _ = driver.check_pos_force_freshness_for_tx();
                    }
                    break None;
                }
                Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                    return Err(ArmDriverError::EstopInhibited);
                }
                Err(TxRefusal::MitFenceTripped(reason)) => {
                    return Err(ArmDriverError::MitFenceTripped(reason));
                }
                Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => {
                    let stop_res = if is_target_intent {
                        driver.strict_stop_check(Some(start_stamp))
                    } else {
                        driver.pending_stop_check(Some(start_stamp))
                    };
                    if stop_res.is_err() {
                        return Err(ArmDriverError::EstopInhibited);
                    }
                    if !driver.pos_force_feedback_valid(pf_j, driver.now()) {
                        let _ = driver.check_pos_force_freshness_for_tx();
                    }
                    break None;
                }
                Err(TxRefusal::Transport(e)) => {
                    driver.mark_tx_pass_backpressured();
                    let stop_res = if is_target_intent {
                        driver.strict_stop_check(Some(start_stamp))
                    } else {
                        driver.pending_stop_check(Some(start_stamp))
                    };
                    if stop_res.is_err() {
                        return Err(ArmDriverError::EstopInhibited);
                    }
                    return Err(e.into());
                }
                Err(r @ (TxRefusal::TorqueOffWouldDrop | TxRefusal::DisableWaitEvidence | TxRefusal::DisableRequestExpired | TxRefusal::EnableAuthorityExpired)) => {
                    return Err(DamiaoArmDriver::<C>::refusal_to_driver_err(r));
                }
            }
        };

        let Some(sent_frame) = sent_frame else {
            if driver.pf_refresh_pending
                && driver.refresh_scheduler.slot_available()
                && driver.poll_tx_allowed()
            {
                driver.pf_refresh_deferred_ticks =
                    driver.pf_refresh_deferred_ticks.saturating_add(1);
            }
            driver.pf_deferred_ticks = driver.pf_deferred_ticks.saturating_add(1);
            if is_target_intent {
                driver.pf_deferred_motion = true;
                driver.pf_deferred_since.get_or_insert(entry_now);
                driver.pf_deferred_intent = Some(intent);
            }
            let pf_status = driver.take_last_pos_force_tx_status();
            return Ok(PosForceEmitOutcome::Deferred { joint: pf_j, pf_status });
        };

        let is_freeze = driver.is_pos_force_freeze_frame(&sent_frame);
        if is_freeze && !matches!(driver.last_tx_pf_status, PosForceTxStatus::Stale { .. }) {
            let _ = driver.check_pos_force_freshness_for_tx();
        }
        driver.record_sent_target(pf_j, sent_frame, None);

        // Class 4 (`0xCC` refresh) runs strictly AFTER all Class-1, Class-2, and Class-3 frames
        // (both arm and J7 `0x307`) have drained and only if TX budget capacity still allows.
        if driver.pending_hold_tx.len() == 0
            && driver.pending_tx.len() == 0
            && driver.pf_refresh_pending
            && driver.refresh_scheduler.slot_available()
            && driver.poll_tx_allowed()
            && driver.can_start_tick_tx()
        {
            if let Err(e) = driver.try_send_pf_refresh(start_stamp, tx_deadline) {
                driver.discard_staged_ctrl_tx();
                return Err(e);
            }
        }

        let stop_check = |d: &DamiaoArmDriver<C>| {
            if is_target_intent {
                d.strict_stop_check(Some(start_stamp))
            } else {
                d.pending_stop_check(Some(start_stamp))
            }
        };
        if stop_check(driver).is_err() {
            driver.discard_staged_ctrl_tx();
            return Err(ArmDriverError::EstopInhibited);
        }
        let flush_blocked = if !is_target_intent && hold_bypass.bypasses_release_deadline() {
            !driver.can.flush_is_noop()
                && (driver.tick_tx.is_some_and(|p| driver.now() >= p.deadline)
                    || tx_deadline.is_some_and(|dl| dl.passed(driver.now())))
        } else {
            driver.flush_cutoff_exceeded()
                || (!driver.can.flush_is_noop()
                    && tx_deadline.is_some_and(|dl| dl.passed(driver.now())))
        };
        if flush_blocked {
            driver.mark_tx_pass_backpressured();
            driver.discard_staged_ctrl_tx();
            return Err(TransportError::Timeout.into());
        }
        if let Err(e) = driver.flush_can_and_commit() {
            driver.mark_tx_pass_backpressured();
            if stop_check(driver).is_err() {
                return Err(ArmDriverError::EstopInhibited);
            }
            return Err(e.into());
        }
        if stop_check(driver).is_err() {
            driver.discard_staged_ctrl_tx();
            return Err(ArmDriverError::EstopInhibited);
        }

        driver.pf_hold_on_first_fresh = false;
        let sent_vel_raw = u16::from_le_bytes([sent_frame.data[4], sent_frame.data[5]]);
        driver.record_flushed_joint_tx(pf_j, 0, sent_vel_raw > 0);
        if driver.mit_fence.is_some() && driver.mit_fence_trip.is_none() {
            driver.mit_fence_last_cmd[pf_j] = Some(entry_now);
        }
        let pf_status = driver.take_last_pos_force_tx_status();
        if degraded_target_to_continuity_hold {
            driver.pf_deferred_ticks = driver.pf_deferred_ticks.saturating_add(1);
            driver.pf_deferred_motion = true;
            driver.pf_deferred_since.get_or_insert(entry_now);
            driver.pf_deferred_intent = Some(intent);
            Ok(PosForceEmitOutcome::SentHoldInsteadOfTarget {
                joint: pf_j,
                freeze: is_freeze,
                pf_status,
            })
        } else {
            driver.pf_deferred_ticks = 0;
            driver.pf_deferred_motion = false;
            driver.pf_deferred_since = None;
            driver.pf_deferred_intent = None;
            if pf_j < 64 {
                driver.last_target_tx_sent_mask |= 1u64 << pf_j;
            }
            Ok(PosForceEmitOutcome::Sent { joint: pf_j, freeze: is_freeze, pf_status })
        }
    }
}
