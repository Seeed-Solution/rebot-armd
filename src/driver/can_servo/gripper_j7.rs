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
    pub(in crate::driver::can_servo) fn emit_pos_force_j7_with_policy(
        &mut self,
        intent: PosForceIntent,
        current: PosForceIntentStamp,
        tx_deadline: Option<Deadline>,
        hold_bypass: ContinuityBypass,
    ) -> Result<PosForceEmitOutcome, ArmDriverError> {
        if self.control_release_managed && self.control_release_id != 0 {
            if let Some(pf) = self.pos_force.as_ref() {
                if pf.joint < 32 && (self.release_ctrl_tx_mask & (1u32 << pf.joint)) != 0 {
                    return Ok(PosForceEmitOutcome::NotApplicable);
                }
            }
        }
        let res = pos_force_emitter::emit_tick_with_deadline(
            self,
            intent,
            current,
            hold_bypass,
            tx_deadline,
        );
        if let Ok(
            PosForceEmitOutcome::Sent { joint, .. }
            | PosForceEmitOutcome::SentHoldInsteadOfTarget { joint, .. },
        ) = res
        {
            if self.mit_fence.is_some() && joint < self.mit_fence_last_cmd.len() {
                self.mit_fence_last_cmd[joint] = Some(self.now());
            }
            if self.control_release_managed && self.control_release_id != 0 && joint < 32 {
                self.release_ctrl_tx_mask |= 1u32 << joint;
            }
        }
        res
    }

    /// Current driver-gate `(stop_gen, estop_epoch, lease_gen)` stamp for direct-driver callers of
    /// [`ArmDriver::emit_pos_force_j7`].
    #[inline]
    pub fn current_j7_intent_stamp(&self) -> PosForceIntentStamp {
        let s = self.current_stop_stamp();
        PosForceIntentStamp {
            stop_gen: s.stop_gen,
            estop_epoch: s.estop_gen,
            lease_gen: self.current_lease_gen(),
        }
    }

    /// Per-call continuity bypass policy for stationary J7 `Hold`/`Stop` frames.
    /// Never applied to motion `Target` intents (enforced inside `pos_force_emitter` and `send`).
    #[inline]
    pub(in crate::driver::can_servo) fn j7_hold_continuity_bypass(&self) -> ContinuityBypass {
        if self.mit_fence.is_some() && self.mit_fence_trip.is_none() {
            if self.in_active_control_release() {
                ContinuityBypass::LedgerAndReleaseDeadline
            } else {
                ContinuityBypass::Ledger
            }
        } else {
            ContinuityBypass::None
        }
    }

    /// Priority `(class, slot_last_tx_seq)` of the active `PosForce` J7 joint's `0x307` frame
    /// for the current tick epilogue, or `None` when J7 has no `0x307` frame to send.
    ///
    /// - Class 1 (`TX_CLASS_STOP_TRANSITION`): J7 is not yet in an unchanged hold/freeze
    ///   (`slot_in_unchanged_hold[pf_j * 2] == false`) and is about to emit stop / hold / freeze.
    /// - Class 2 (`TX_CLASS_MOTION_TARGET`): J7 fresh moving target.
    /// - Class 3 (`TX_CLASS_HOLD_RESEND`): J7 re-send of an unchanged hold / freeze.
    #[inline]
    pub(in crate::driver::can_servo) fn j7_tx_priority(&self, is_hold_pass: bool) -> Option<(u8, u64)> {
        if !self.is_enabled && !is_hold_pass && !self.pf_next_intent_is_hold {
            return None;
        }
        let pf_j = self.active_pos_force_joint()?;
        let now = self.now();
        let fresh = self.pos_force_feedback_valid(pf_j, now);
        if !fresh && self.pos_force_freeze_params(pf_j).is_none() {
            return None;
        }
        let j7_emitting_hold_or_freeze =
            is_hold_pass || self.pf_next_intent_is_hold || self.pf_hold_on_first_fresh || !fresh;
        let flat = pf_j.saturating_mul(2);
        let j7_in_hold = self.slot_in_unchanged_hold.get(flat).copied().unwrap_or(false);
        let class = if j7_emitting_hold_or_freeze {
            if j7_in_hold {
                TX_CLASS_HOLD_RESEND
            } else {
                TX_CLASS_STOP_TRANSITION
            }
        } else {
            TX_CLASS_MOTION_TARGET
        };
        let seq = self.slot_last_tx_seq.get(flat).copied().unwrap_or(0);
        Some((class, seq))
    }

    /// Drive `cfg.joint` in position-force mode with `cfg.current_raw` on every frame. The mode
    /// (RAM) is written at the next enable with torque off and confirmed by its register reply
    /// and a readback; until then no command frame reaches that joint.
    pub fn with_gripper_pos_force(mut self, cfg: GripperPosForce) -> Result<Self, ArmDriverError> {
        if cfg.joint >= self.motor_ids.len() {
            return Err(ArmDriverError::Protocol(format!("pos-force gripper joint {} outside {} joints", cfg.joint + 1, self.motor_ids.len())));
        }
        if cfg.current_raw == 0 || cfg.current_raw > damiao::POS_FORCE_I_DES_FULL
            || !(cfg.close_speed_rad_s.is_finite() && cfg.close_speed_rad_s > 0.0)
            || damiao::pos_force_vel_raw(cfg.close_speed_rad_s) == 0
            || !(cfg.closed_rad.is_finite() && cfg.open_rad.is_finite()) || cfg.closed_rad == cfg.open_rad {
            return Err(ArmDriverError::Protocol("pos-force gripper: current_raw 1..=10000, close_speed quantizing >= 0.01 rad/s, closed_rad != open_rad".into()));
        }
        self.base_modes[cfg.joint] = DamiaoControlMode::PosForce;
        self.pos_force = Some(cfg);
        Ok(self)
    }

    /// Apply the profile's gripper drive (`gripper_drive_from_profile`).
    pub fn with_gripper_drive(self, drive: Option<GripperDrive>) -> Result<Self, ArmDriverError> {
        match drive {
            None => Ok(self),
            Some(GripperDrive::PosVel(j)) => Ok(self.with_gripper_pos_vel(j)),
            Some(GripperDrive::PosForce(cfg)) => self.with_gripper_pos_force(cfg),
        }
    }

    /// Legacy profile: `joint` is the gripper, driven in POS_VEL; reported as "no motor current cap".
    pub fn with_gripper_pos_vel(mut self, joint: usize) -> Self {
        self.legacy_gripper = Some(joint);
        self
    }

    /// Joint index of the active, verified `PosForce` gripper (if configured and currently in `PosForce`).
    #[inline]
    pub(in crate::driver::can_servo) fn active_pos_force_joint(&self) -> Option<usize> {
        let pf = self.pos_force.as_ref()?;
        let uncertain = pf.joint < 32 && (self.mode_write_uncertain_mask & (1u32 << pf.joint)) != 0;
        (self.pos_force_verified
            && !uncertain
            && self.joint_modes.get(pf.joint).copied().flatten() == Some(DamiaoControlMode::PosForce))
            .then_some(pf.joint)
    }

    /// Single freshness authority for the `PosForce` gripper: inclusive `<= 100 ms` on BOTH
    /// `last_feedback_instant[joint]` AND `feedback_queue_bound[joint]`, plus finite position.
    #[inline]
    pub(in crate::driver::can_servo) fn pos_force_feedback_valid(&self, joint: usize, now: Instant) -> bool {
        self.fresh_measured_position_at(joint, POS_FORCE_FEEDBACK_MAX_AGE, now).is_some()
    }

    /// Diagnostic staleness age for `joint` at `now`: max of receive age and receive-queue-bound
    /// age ONLY when both timestamps exist and the cached position is finite; otherwise `None` so
    /// non-finite or unbounded samples fall back to the episode elapsed timer cleanly.
    pub(in crate::driver::can_servo) fn pos_force_stale_age(&self, joint: usize, now: Instant) -> Option<Duration> {
        if !self.cached_states.get(joint).is_some_and(|s| s.position.is_finite()) {
            return None;
        }
        let rx_age = self
            .last_feedback_instant
            .get(joint)
            .copied()
            .flatten()
            .map(|t| now.saturating_duration_since(t))?;
        let qb_age = self
            .feedback_queue_bound
            .get(joint)
            .copied()
            .flatten()
            .map(|b| now.saturating_duration_since(b))?;
        Some(rx_age.max(qb_age))
    }

    /// Returns `Some(pf.joint)` iff `frame` is a Damiao `0x300 + motor_id` PosForce command frame
    /// targeting the configured `PosForce` gripper joint.
    #[inline]
    pub(in crate::driver::can_servo) fn frame_pos_force_joint(&self, frame: &CanFrame) -> Option<usize> {
        let pf = self.pos_force.as_ref()?;
        let motor_id = *self.motor_ids.get(pf.joint)?;
        let expected_id = u32::from(damiao::POS_FORCE_BASE_ID) + u32::from(motor_id);
        (frame.id == expected_id).then_some(pf.joint)
    }

    /// Pure parameter computation `(position, vel_raw = 0, i_des)` for the Damiao `0x307` FREEZE
    /// frame of joint `i` — performs NO `CanFrame` construction.
    /// - When at least one provably fresh (`<= 100 ms` at decode time) J7 sample has been observed
    ///   since this `armd` process started (`pf_last_fresh_position == Some(pos)`):
    ///   `Some((pos, 0, pf.current_raw))`.
    /// - When NO provably fresh J7 sample has ever been observed since boot
    ///   (`pf_last_fresh_position == None`, [`PosForceBootFallback::NoOutputUntilFresh`]):
    ///   `None` (send NO `0x307` frame until the first fresh sample is decoded).
    pub(in crate::driver) fn pos_force_freeze_params(&self, i: usize) -> Option<(f32, u16, u16)> {
        let pf = self.pos_force.as_ref().filter(|pf| pf.joint == i)?;
        let _ = self.motor_ids.get(i)?;
        match self.pf_last_fresh_position.filter(|p| p.is_finite()) {
            Some(pos) => Some((pos, 0, pf.current_raw)),
            None => match POS_FORCE_BOOT_FALLBACK {
                PosForceBootFallback::NoOutputUntilFresh => None,
            },
        }
    }

    /// True iff `frame` matches bit-for-bit the expected `0x307` FREEZE (or boot-fallback) payload
    /// produced by [`Self::pos_force_freeze_params`] for the configured `PosForce` joint.
    /// Guarantees stale moving targets (`vel_raw != 0` or any new target position) can never
    /// bypass `send()`'s freshness check while freeze frames always reach the wire.
    pub(in crate::driver::can_servo) fn is_pos_force_freeze_frame(&self, frame: &CanFrame) -> bool {
        let Some(pf) = self.pos_force.as_ref() else {
            return false;
        };
        let Some(&motor_id) = self.motor_ids.get(pf.joint) else {
            return false;
        };
        let expected_id = u32::from(damiao::POS_FORCE_BASE_ID) + u32::from(motor_id);
        if frame.id != expected_id || frame.dlc != 8 {
            return false;
        }
        let Some((pos, vel_raw, cur_raw)) = self.pos_force_freeze_params(pf.joint) else {
            return false;
        };
        frame.data[0..4] == pos.to_le_bytes()
            && frame.data[4..6] == vel_raw.to_le_bytes()
            && frame.data[6..8] == cur_raw.to_le_bytes()
    }

    /// Single shared J7 position-force freshness gate used by the single J7 emitter.
    ///
    /// - When J7 feedback is provably fresh (`<= 100 ms`), records `Fresh` (unless already marked
    ///   `Stale` in this pass), clears `pf_refresh_pending` / `pf_refresh_deferred_ticks`, and
    ///   returns `None`.
    /// - When J7 feedback is stale (`> 100 ms` or missing queue bound), records
    ///   `Stale { joint, age, fresh_gen }`, schedules a `0xCC` refresh (`pf_refresh_pending = true`),
    ///   and returns `Some(pf_joint)`.
    pub(in crate::driver::can_servo) fn check_pos_force_freshness_for_tx_at(&mut self, now: Instant) -> Option<usize> {
        let pf_joint = self.active_pos_force_joint()?;
        if self.pos_force_feedback_valid(pf_joint, now) {
            if !matches!(self.last_tx_pf_status, PosForceTxStatus::Stale { .. }) {
                self.last_tx_pf_status = PosForceTxStatus::Fresh;
            }
            self.pf_refresh_pending = false;
            self.pf_refresh_deferred_ticks = 0;
            return None;
        }
        let age = self.pos_force_stale_age(pf_joint, now);
        self.last_tx_pf_status = PosForceTxStatus::Stale {
            joint: pf_joint,
            age,
            fresh_gen: self.pf_fresh_gen,
        };
        self.pf_refresh_pending = true;
        Some(pf_joint)
    }

    pub(in crate::driver::can_servo) fn check_pos_force_freshness_for_tx(&mut self) -> Option<usize> {
        let now = self.now();
        self.check_pos_force_freshness_for_tx_at(now)
    }

    /// Post-freeze `0xCC` refresh slot for a stale active `PosForce` joint, sharing the single
    /// [`RefreshScheduler`] per-tick 1-slot cap (`slot_available()`). Callers own flushing when
    /// this returns `Ok(true)`.
    pub(in crate::driver::can_servo) fn try_send_pf_refresh(
        &mut self,
        expected_stamp: StopStamp,
        explicit_deadline: Option<Deadline>,
    ) -> Result<bool, ArmDriverError> {
        let Some(pf_joint) = self.active_pos_force_joint() else {
            self.pf_refresh_pending = false;
            self.pf_refresh_deferred_ticks = 0;
            return Ok(false);
        };
        if !self.pf_refresh_pending
            || !self.refresh_scheduler.slot_available()
            || !self.poll_tx_allowed()
        {
            return Ok(false);
        }
        let effective_deadline = match (explicit_deadline, self.active_release_cutoff_deadline()) {
            (Some(a), Some(b)) => Some(Deadline { at: a.at.min(b.at) }),
            (a, b) => a.or(b),
        };
        match self.send_plain(
            TxKind::Refresh,
            Some(expected_stamp),
            effective_deadline,
            false,
            |d| Some(damiao::build_refresh_frame(d.motor_ids[pf_joint])),
        ) {
            Ok(_) => {
                self.refresh_scheduler.mark_slot_used();
                self.pf_refresh_pending = false;
                self.pf_refresh_deferred_ticks = 0;
                Ok(true)
            }
            Err(TxRefusal::Deferred | TxRefusal::DeadlinePassed) => {
                self.refresh_scheduler.record_deferred();
                self.pf_refresh_pending = true;
                self.pf_refresh_deferred_ticks = self.pf_refresh_deferred_ticks.saturating_add(1);
                Ok(false)
            }
            Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                self.discard_all_pending_tx();
                Err(ArmDriverError::EstopInhibited)
            }
            Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => {
                if self.pending_stop_check(Some(expected_stamp)).is_err() {
                    self.discard_all_pending_tx();
                    return Err(ArmDriverError::EstopInhibited);
                }
                self.refresh_scheduler.record_deferred();
                self.pf_refresh_pending = true;
                self.pf_refresh_deferred_ticks = self.pf_refresh_deferred_ticks.saturating_add(1);
                Ok(false)
            }
            Err(_) => {
                self.refresh_scheduler.record_deferred();
                self.pf_refresh_pending = true;
                self.pf_refresh_deferred_ticks = self.pf_refresh_deferred_ticks.saturating_add(1);
                Ok(false)
            }
        }
    }

    /// True when joint `i` is the configured `PosForce` joint, has provably fresh (`<= 100 ms`)
    /// measured feedback at `now`, and `target` encodes a valid finite target with `vel_raw >= 1`
    /// (NO CAN frame construction — all `0x307` frames are constructed solely in
    /// [`Self::emit_pos_force_frame`]).
    pub(in crate::driver::can_servo) fn pos_force_target_valid_at(&self, i: usize, target: &JointTarget, now: Instant) -> bool {
        let Some(pf) = self.pos_force.as_ref().filter(|pf| pf.joint == i) else {
            return false;
        };
        if !target.position.is_finite() || target.velocity.is_some_and(|v| !v.is_finite()) {
            return false;
        }
        let Some(measured) = self.fresh_measured_position_at(i, POS_FORCE_FEEDBACK_MAX_AGE, now) else {
            return false;
        };
        let requested = target.velocity.filter(|v| *v != 0.0).map_or(self.vlim(), f32::abs).min(self.vlim());
        let strictly_opening = (target.position - measured) * (pf.closed_rad - pf.open_rad) < 0.0;
        let speed = if strictly_opening { requested } else { requested.min(pf.close_speed_rad_s) };
        damiao::pos_force_vel_raw(speed) != 0
    }

    pub(in crate::driver::can_servo) fn pos_force_target_valid(&self, i: usize, target: &JointTarget) -> bool {
        self.pos_force_target_valid_at(i, target, self.now())
    }

    /// True if joint `i`'s mode hold slot `slot` must be emitted via the single `PosForce` emitter
    /// ([`Self::emit_pos_force_frame`]).
    #[inline]
    pub(in crate::driver::can_servo) fn is_pos_force_mode_hold_slot(&self, i: usize, mode: DamiaoControlMode, dual: bool, slot: usize) -> bool {
        slot == 0
            && (mode == DamiaoControlMode::PosForce
                || (dual && self.base_mode(i) == DamiaoControlMode::PosForce))
    }

    /// Validate that every position-force joint switching into `PosForce` can produce a valid
    /// capped hold frame before any `CTRL_MODE` write goes on the bus.
    pub(in crate::driver::can_servo) fn validate_pos_force_holds(&self, joints: &[usize], modes: &[DamiaoControlMode], hold: &[JointTarget], require_when_disabled: bool) -> Result<(), ArmDriverError> {
        for &i in joints {
            if modes[i] == DamiaoControlMode::PosForce
                && (require_when_disabled || self.is_enabled)
                && !self.pos_force_target_valid(i, &hold[i])
            {
                return Err(ArmDriverError::HardwareFault(format!(
                    "joint {} position-force mode switch refused: hold target invalid or feedback stale", i + 1)));
            }
        }
        Ok(())
    }

    /// Validate a `PosForce` target's static parameters (configured joint, finite position/velocity,
    /// nonzero wire speed encoding, nonzero current cap) WITHOUT checking live feedback freshness.
    pub(in crate::driver::can_servo) fn validate_pos_force_target_static(&self, i: usize, target: &JointTarget) -> Result<u32, ArmDriverError> {
        let Some(pf) = self.pos_force.as_ref().filter(|pf| pf.joint == i) else {
            return Err(ArmDriverError::PosForceTargetUnavailable { joint: i });
        };
        if !target.position.is_finite() || target.velocity.is_some_and(|v| !v.is_finite()) {
            return Err(ArmDriverError::PosForceTargetNotFinite { joint: i });
        }
        let requested = target
            .velocity
            .filter(|v| *v != 0.0)
            .map_or(self.vlim(), f32::abs)
            .min(self.vlim());
        if damiao::pos_force_vel_raw(requested) == 0
            || damiao::pos_force_vel_raw(pf.close_speed_rad_s) == 0
        {
            return Err(ArmDriverError::PosForceSpeedZero { joint: i });
        }
        Ok(u32::from(damiao::POS_FORCE_BASE_ID) + u32::from(self.motor_ids[i]))
    }

    /// Start a bounded causal `0xCC` feedback refresh for J7 while `DamiaoTorque::Enable` is in
    /// flight. The refresh deadline is fixed before any drain/TX and clamped to both
    /// `window_deadline` and `enable_tx_deadline`.
    pub(in crate::driver::can_servo) fn start_enable_pf_refresh(
        &mut self,
        joint: usize,
        window_deadline: Deadline,
        drained_now: bool,
    ) -> Result<FeedbackRefresh, ArmDriverError> {
        self.enable_estop_inhibited()?;
        self.check_enable_tx_authority()?;
        self.refresh_capture = None;
        let start = self.now();
        let mut refresh_at = (start + ENABLE_PF_REFRESH_TIMEOUT).min(window_deadline.at);
        if let Some(auth_dl) = self.enable_tx_deadline {
            refresh_at = refresh_at.min(auth_dl);
        }
        let deadline = Deadline { at: refresh_at };
        let boundary = if drained_now && !deadline.passed(self.now()) {
            match self.send_refresh_with_boundary_opts(joint, deadline, true) {
                Ok(b) => b,
                Err(e) if is_estop_inhibited(&e) => return Err(e),
                Err(e) => {
                    self.enable_failed();
                    return Err(e);
                }
            }
        } else {
            None
        };
        Ok(FeedbackRefresh { joint, boundary, deadline })
    }

    pub(in crate::driver::can_servo) fn pos_force_sample_refused(&self, joint: usize, label: &str, fresh: bool, velocity: f32, status: u8) -> Option<ArmDriverError> {
        let still = velocity.is_finite() && velocity.abs() <= 0.05;
        let off = !self.is_enabled && status == 0;
        if fresh && still && off {
            return None;
        }
        let fault = damiao_fault_name(status).map_or(String::new(), |name| format!(" ({name})"));
        Some(ArmDriverError::HardwareFault(format!(
            "joint {} {label}: needs fresh stationary feedback with torque off (fresh {fresh}, still {still}, torque off {off}, status {status}{fault}); capped operation unavailable",
            joint + 1
        )))
    }

    /// With torque off, the position-force gripper's mode is written only on fresh (100 ms),
    /// stationary (|v| <= 0.05 rad/s), state-0 feedback.
    pub(in crate::driver::can_servo) fn pos_force_write_refused(&self, stray: &[usize]) -> Option<ArmDriverError> {
        let j = self.pos_force.as_ref()?.joint;
        if !stray.contains(&j) { return None; }
        let fresh = self.fresh_measured_position(j, POS_FORCE_FEEDBACK_MAX_AGE).is_some();
        self.pos_force_sample_refused(j, "position-force mode not written", fresh, self.cached_states[j].velocity, self.motor_status[j])
    }

    /// Revalidate at `now` that a position-force gripper joint still has provably fresh (`<= 100 ms`)
    /// non-faulted feedback (both decode instant and empty-queue bound) and verified CTRL_MODE 4
    /// right before transmitting an initial, carried, or retried `CMD_ENABLE` frame.
    pub(in crate::driver::can_servo) fn pf_enable_feedback_fresh_at(&self, joint: usize, now: Instant) -> bool {
        if !self.pos_force.as_ref().is_some_and(|pf| pf.joint == joint) {
            return true;
        }
        if !self.pos_force_verified {
            return false;
        }
        let Some((rx_at, queue_bound)) = self.pf_last_fresh_feedback else {
            return false;
        };
        let status = self.motor_status.get(joint).copied().unwrap_or(0xFF);
        let status_ok = status == 0 || status == 1;
        let state_ok = self.cached_states.get(joint).is_some_and(|s| {
            s.position.is_finite()
                && if status == 0 {
                    s.velocity.is_finite() && s.velocity.abs() <= 0.05
                } else {
                    s.velocity.is_finite()
                }
        });
        status_ok
            && state_ok
            && now.saturating_duration_since(rx_at) <= POS_FORCE_FEEDBACK_MAX_AGE
            && now.saturating_duration_since(queue_bound) <= POS_FORCE_FEEDBACK_MAX_AGE
    }

    pub(in crate::driver::can_servo) fn pf_enable_feedback_fresh(&self, joint: usize) -> bool {
        self.pf_enable_feedback_fresh_at(joint, self.now())
    }

    /// Fail closed immediately if the configured `PosForce` gripper has decoded a hardware fault.
    pub(in crate::driver::can_servo) fn check_pf_enable_hardware_fault(&mut self, pf_joint: Option<usize>) -> Result<(), ArmDriverError> {
        if let Some(pf_j) = pf_joint {
            let status = self.motor_status.get(pf_j).copied().unwrap_or(0);
            if damiao_fault_name(status).is_some() {
                let motor_id = self.motor_ids[pf_j];
                self.torque_hs = None;
                self.enable_failed();
                return Err(ArmDriverError::MotorHardwareFault {
                    joint: pf_j,
                    motor_id,
                    status,
                });
            }
        }
        Ok(())
    }

    /// Single J7 enable refresh-arming helper shared across initial, carried, and retry passes:
    /// refuses immediately on a decoded J7 hardware fault, and if J7 is pending in `unsent_mask`
    /// and not fresh, reserves the bounded causal `0xCC` refresh slot (failing closed once the
    /// bounded slot budget is exhausted).
    pub(in crate::driver::can_servo) fn ensure_enable_pf_refresh(
        &mut self,
        pf_joint: Option<usize>,
        unsent_mask: u32,
        deadline: Deadline,
        pf_refresh: &mut Option<FeedbackRefresh>,
        pf_refresh_count: &mut u8,
    ) -> Result<(), ArmDriverError> {
        self.check_pf_enable_hardware_fault(pf_joint)?;
        let Some(pf_j) = pf_joint else {
            return Ok(());
        };
        let pf_bit = 1u32 << pf_j;
        if pf_refresh.is_none()
            && (unsent_mask & pf_bit) != 0
            && !self.pf_enable_feedback_fresh(pf_j)
        {
            if *pf_refresh_count >= MAX_ENABLE_PF_REFRESHES {
                self.torque_hs = None;
                self.enable_failed();
                return Err(ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint: pf_j });
            }
            *pf_refresh_count += 1;
            let drained_now = self.last_read_drained;
            *pf_refresh = Some(self.start_enable_pf_refresh(pf_j, deadline, drained_now)?);
        }
        Ok(())
    }

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

impl<C: CanTransport> PosForceExt for DamiaoArmDriver<C> {
    fn motor_current_cap(&self) -> Option<MotorCurrentCap> {
        if let Some(pf) = &self.pos_force {
            return Some(MotorCurrentCap {
                joint: pf.joint, mode: "pos_force", current_raw: Some(pf.current_raw),
                active: self.pos_force_verified && self.joint_modes.get(pf.joint).copied().flatten() == Some(DamiaoControlMode::PosForce),
                calibrated: pf.calibrated, ceiling_torque_nm: pf.ceiling_torque_nm, torque_reference: pf.torque_reference.clone(),
                close_speed_rad_s: Some(pf.close_speed_rad_s),
                note: if pf.calibrated { "motor current ceiling on every gripper frame (commissioned)" }
                    else { "motor current ceiling on every gripper frame; its torque equivalent is uncalibrated" },
            });
        }
        self.legacy_gripper.map(|joint| MotorCurrentCap {
            joint, mode: "pos_vel", current_raw: None, active: false, calibrated: false, ceiling_torque_nm: None,
            torque_reference: None, close_speed_rad_s: None, note: "no motor current cap (legacy POS_VEL gripper)",
        })
    }

    fn gripper_pos_force_info(&self) -> Option<GripperPosForceInfo> {
        let pf = self.pos_force.as_ref()?;
        if !(pf.close_speed_rad_s.is_finite() && pf.close_speed_rad_s > 0.0) {
            return None;
        }
        let active = self.pos_force_verified
            && self.joint_modes.get(pf.joint).copied().flatten() == Some(DamiaoControlMode::PosForce);
        let ceiling_torque_estimate_nm = pf.ceiling_torque_nm
            .or(Some(f32::from(pf.current_raw) * crate::model::DamiaoGripperDrive::FEEDBACK_EQUIVALENT_NM_PER_RAW))
            .filter(|v| v.is_finite() && *v > 0.0);
        Some(GripperPosForceInfo {
            joint: pf.joint,
            active,
            close_speed_rad_s: pf.close_speed_rad_s,
            ceiling_torque_estimate_nm,
        })
    }

    fn set_external_pos_force_emitter(&mut self, enabled: bool) {
        self.external_pos_force_emitter = enabled;
    }

    fn validate_pos_force_intent_target(
        &self,
        position: f32,
        velocity: Option<f32>,
        current_raw: Option<u16>,
    ) -> Result<(), ArmDriverError> {
        let Some(pf) = self.pos_force.as_ref() else {
            return Ok(());
        };
        if self.joint_modes.get(pf.joint).copied().flatten() != Some(DamiaoControlMode::PosForce) {
            return Ok(());
        }
        let target = JointTarget {
            position,
            velocity,
            ..JointTarget::position(position)
        };
        let _ = self.validate_pos_force_target_static(pf.joint, &target)?;
        if current_raw.is_some_and(|c| c == 0) {
            return Err(ArmDriverError::PosForceTargetUnavailable { joint: pf.joint });
        }
        Ok(())
    }

    fn set_pending_pos_force_hold_hint(&mut self, is_hold: bool) {
        self.pf_next_intent_is_hold = is_hold;
    }

    fn emit_pos_force_j7(
        &mut self,
        intent: PosForceIntent,
        current: PosForceIntentStamp,
    ) -> Result<PosForceEmitOutcome, ArmDriverError> {
        if self.control_release_managed && self.control_release_id != 0 {
            if let Some(pf) = self.pos_force.as_ref() {
                if pf.joint < 32 && (self.release_ctrl_tx_mask & (1u32 << pf.joint)) != 0 {
                    return Ok(PosForceEmitOutcome::NotApplicable);
                }
            }
        }
        let hold_bypass = self.j7_hold_continuity_bypass();
        let res = pos_force_emitter::emit_tick(self, intent, current, hold_bypass);
        if let Ok(
            PosForceEmitOutcome::Sent { joint, .. }
            | PosForceEmitOutcome::SentHoldInsteadOfTarget { joint, .. },
        ) = res
        {
            if self.mit_fence.is_some() && joint < self.mit_fence_last_cmd.len() {
                self.mit_fence_last_cmd[joint] = Some(self.now());
            }
            if self.control_release_managed && self.control_release_id != 0 && joint < 32 {
                self.release_ctrl_tx_mask |= 1u32 << joint;
            }
        }
        res
    }

    fn take_last_pos_force_tx_status(&mut self) -> PosForceTxStatus {
        std::mem::take(&mut self.last_tx_pf_status)
    }

    fn pos_force_feedback_fresh(&self) -> bool {
        // Resolve the configured PosForce joint independently of `pos_force_verified` or active
        // TX mode so a disable or mode invalidation can NEVER bypass the <= 100 ms clear check.
        let Some(pf) = self.pos_force.as_ref() else {
            return true;
        };
        self.pos_force_feedback_valid(pf.joint, self.now())
    }

    fn pos_force_last_fresh_position(&self) -> Option<f32> {
        let _ = self.pos_force.as_ref()?;
        self.pf_last_fresh_position
    }

    fn reset_pos_force_boot_state(&mut self) {
        if let Some(pf) = self.pos_force.as_ref() {
            let j = pf.joint;
            if j < self.last_feedback_instant.len() {
                self.last_feedback_instant[j] = None;
            }
            if j < self.feedback_queue_bound.len() {
                self.feedback_queue_bound[j] = None;
            }
            self.pf_last_fresh_feedback = None;
            self.pf_last_fresh_position = None;
            self.pf_hold_on_first_fresh = false;
            self.pf_refresh_pending = false;
            self.refresh_scheduler.clear_joint(j);
            self.pf_deferred_ticks = 0;
            self.pf_deferred_motion = false;
            self.pf_deferred_since = None;
            self.pf_deferred_intent = None;
        }
    }
}

/// Gripper joint driven in position-force mode with a motor current ceiling
/// ([`DamiaoArmDriver::with_gripper_pos_force`]).
#[derive(Debug, Clone, PartialEq)]
pub struct GripperPosForce {
    /// Joint index (0-based).
    pub joint: usize,
    /// Current ceiling of every frame: 1/10000 of the maximum phase current.
    pub current_raw: u16,
    /// Speed limit of closing frames (rad/s): min(requested, this).
    pub close_speed_rad_s: f32,
    /// Jaw positions (rad): a frame closes when it moves from the measured position toward `closed_rad`.
    pub closed_rad: f32,
    pub open_rad: f32,
    /// The ceiling's torque equivalent was measured (commissioning); false: uncalibrated.
    pub calibrated: bool,
    /// Torque equivalent of the ceiling (N·m), when commissioned.
    pub ceiling_torque_nm: Option<f32>,
    /// What the torque equivalent was measured against (`external` / `feedback_equivalent`).
    pub torque_reference: Option<String>,
}

/// The profile's choice for the Damiao gripper joint.
#[derive(Debug, Clone, PartialEq)]
pub enum GripperDrive {
    PosForce(GripperPosForce),
    /// Legacy POS_VEL on this joint (no motor current cap).
    PosVel(usize),
}

/// `ArmConfig::damiao_gripper` as a driver setting (None: no block or no gripper joint).
/// Position-force needs the gripper policy's closed / open positions (closing direction).
pub fn gripper_drive_from_profile(cfg: &crate::model::ArmConfig) -> Result<Option<GripperDrive>, String> {
    use crate::model::DamiaoGripperMode;
    let (Some(g), Some(joint)) = (cfg.damiao_gripper.as_ref(), cfg.gripper_index()) else { return Ok(None) };
    Ok(Some(match g.mode {
        DamiaoGripperMode::PosVel => GripperDrive::PosVel(joint),
        DamiaoGripperMode::PosForce => {
            let p = cfg.gripper_policy.as_ref().ok_or("damiao_gripper pos_force needs gripper_policy (closed_rad / open_rad)")?;
            let close_speed_rad_s = cfg.limits.get(joint)
                .filter(|l| l.max_velocity.is_finite() && l.max_velocity > 0.0)
                .map_or(g.close_speed_rad_s, |l| g.close_speed_rad_s.min(l.max_velocity));
            GripperDrive::PosForce(GripperPosForce {
                joint, current_raw: g.current_ceiling_raw, close_speed_rad_s,
                closed_rad: p.closed_rad, open_rad: p.open_rad, calibrated: g.calibrated,
                ceiling_torque_nm: g.ceiling_torque_nm, torque_reference: g.torque_reference.clone(),
            })
        }
    }))
}

/// Allocation-free `Copy` summary of the gripper's position-force motor current cap,
/// safe to query on the RT control thread (`armd-control`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GripperPosForceInfo {
    pub joint: usize,
    pub active: bool,
    pub close_speed_rad_s: f32,
    pub ceiling_torque_estimate_nm: Option<f32>,
}

/// Motor-level current cap of the gripper, as the driver applies it (`ArmDriver::motor_current_cap`).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct MotorCurrentCap {
    pub joint: usize,
    /// `pos_force` (capped) or `pos_vel` (legacy, no motor current cap).
    pub mode: &'static str,
    /// Configured ceiling (raw, 1/10000 of the maximum phase current); None in `pos_vel`.
    pub current_raw: Option<u16>,
    /// The motor confirmed the capped mode (register reply and readback) at the last enable.
    pub active: bool,
    pub calibrated: bool,
    pub ceiling_torque_nm: Option<f32>,
    pub torque_reference: Option<String>,
    pub close_speed_rad_s: Option<f32>,
    pub note: &'static str,
}
