//! Capability extensions of [`ArmDriver`] (S4 step 1).
//!
//! The core trait keeps what every follower implements; each cluster that only some drivers
//! implement lives in one `*Ext` trait here. A driver exposes a cluster through the accessor
//! pair on [`ArmDriver`] (`tx_ext()` / `tx_ext_mut()`, ...). Owners call the getters on
//! `dyn ArmDriver` (`drv.tx()`, `drv.tx_mut()`, ...), which fall back to [`NoExt`]: a driver
//! without the cluster behaves exactly as the default bodies below, which moved verbatim
//! from the trait. `Some(accessor)` means the driver type has the cluster, not that an
//! operation is available now (the `supports_*` predicates still decide that).
//!
//! `BoundedTxExt`, `ModeSwitchExt`, `MitExt` and `BusRecoveryExt` carry no joint-space
//! values: `ZeroOffsetDriver` passes them through without offsets. Do not add a method
//! with joint angles to them.
#![allow(clippy::too_many_arguments)]

use super::*;

/// Bounded TX / release budget accounting (Damiao). Carries no joint angles.
pub trait BoundedTxExt {
    /// Installs or clears the enable authority fence (`min(lease_expiry, request_deadline)` plus
    /// expected lease generation) checked immediately before transmitting every `CMD_ENABLE` frame
    /// so expired or fenced authority can never emit an unauthorized enable after mid-poll RX.
    fn set_enable_authority(
        &mut self,
        _deadline: Option<Instant>,
        _expected_lease_gen: Option<u64>,
    ) {}

    /// Installs or clears the disable authority fence (`request_deadline` plus optional admitted
    /// lease generation) checked before dispatching undispatched motors during multi-tick disable.
    fn set_disable_authority(
        &mut self,
        _deadline: Option<Instant>,
        _expected_lease_gen: Option<u64>,
    ) {}

    /// Open a fresh TX pass for a Window 2 sub-release wake while preserving the active release's
    /// coalescing ledger (`release_ctrl_tx_mask`, `refresh_scheduler`, `release_recovery_tx_count`),
    /// and mark that Step 5 normal output will NOT run again in this release.
    fn begin_wake_tx_pass(&mut self) {}

    /// Exit any prior managed control release and open a fresh standalone TX pass (used by
    /// direct off-release `clear_fault` calls that are not sharing a `ReleaseBudget` ledger).
    fn reset_standalone_tx_pass(&mut self) {}

    /// Configured per-release budgeted TX attempt cap, if this driver enforces one.
    fn tx_budget_max_frames(&self) -> Option<u16> { None }

    /// Verifies the enable admission invariant: `max_frames >= configured_motors + 1`.
    fn check_watchdog_tx_budget_for_enable(&self) -> Result<(), (usize, usize)> { Ok(()) }

    /// Verifies the energized admission invariant:
    /// `max_frames >= energized_arm_axes + energized_j7 + 1`.
    fn check_watchdog_tx_budget_for_energized(&self) -> Result<(), (usize, usize)> { Ok(()) }

    /// Cumulative `(rx_frames_consumed, budgeted_tx_attempts_started)` accounted by this driver
    /// during the current control release (including discarded RX frames).
    fn release_budget_consumed(&self) -> (u16, u16) { (0, 0) }

    /// Monotonically tightens this driver's remaining RX frame and budgeted TX attempt caps for
    /// the current control release from the owner's [`crate::rt_core::budget::ReleaseBudget`].
    fn sync_release_budget_limits(&mut self, _rx_remaining: u16, _tx_remaining: u16) {}

    /// Earliest next wake instant needed to keep already-enabled arm joints refreshed within the
    /// hardware watchdog window while a transition is pending (`None` if no motor is energized).
    fn next_watchdog_hold_wake(&self) -> Option<Instant> { None }

    /// Drop any queued stopped-hold frames for joints whose bit is NOT set in `hold_mask`
    /// without disturbing queued holds for other joints.
    fn drop_pending_holds_outside_mask(&mut self, _hold_mask: u64) {}

    /// True if the most recent bounded stopped-hold pass submitted at least one frame to the bus.
    fn last_hold_tx_sent_any(&self) -> bool { true }

    /// Bitmask of `(joint, slot)` hold frames (`1u64 << (joint * 2 + slot)`) that were due in the
    /// most recent bounded stopped-hold pass (`0` when nothing was due or the driver has no
    /// bounded hold queue).
    fn last_hold_tx_due_mask(&self) -> u64 { 0 }

    /// Bitmask of `(joint, slot)` hold frames (`1u64 << (joint * 2 + slot)`) that actually reached
    /// the transport (transmitted and flushed) in the most recent bounded stopped-hold pass.
    fn last_hold_tx_sent_mask(&self) -> u64 { 0 }

    /// Bitmask of 0-based joint indices (`1u64 << i`) that completed a motion target transmit
    /// (and flush) with no unsent target remaining in the carry-over queue during the most recent
    /// [`Self::write_joint_targets_bounded`] pass. Non-CAN/sim drivers default to `u64::MAX`.
    fn last_target_tx_sent_mask(&self) -> u64 { u64::MAX }

    /// Enable or disable torque / power across all joints.
    /// Calling `set_torque(false)` defaults to `DisableKind::DisableRequested` (fail-closed).
    /// Discard any carried-over pending TX frames (no-op for drivers without a TX queue).
    fn discard_pending_tx(&mut self) {}
    /// Update the bounded per-tick TX budget dynamically (used by dual-owner tests).
    #[doc(hidden)]
    fn set_tx_budget(&mut self, _budget: TxBudget) {}

    /// Configured inclusive maximum age for carried-over TX frames (default 45 ms).
    fn tx_frame_max_age(&self) -> Duration { DAMIAO_TX_FRAME_MAX_AGE }

    /// Observed age of the oldest frame dropped as expired during the most recent bounded write.
    fn last_tx_expired_age(&self) -> Option<Duration> { None }

    /// Observed age of the oldest frame currently queued in the pending TX queue.
    fn oldest_pending_tx_age(&self) -> Option<Duration> { None }

    /// Number of unsent motion target frames currently queued in the bounded TX carry-over queue.
    fn pending_motion_tx_len(&self) -> usize { 0 }

    /// Number of unsent stopped-hold frames currently queued in the bounded hold TX queue.
    fn pending_hold_tx_len(&self) -> usize { 0 }

    /// Total unsent frames across both motion and stopped-hold bounded TX carry-over queues.
    fn pending_tx_len_total(&self) -> usize {
        self.pending_motion_tx_len() + self.pending_hold_tx_len()
    }

    /// True when a bounded-CAN driver has confirmed healthy TX via a refresh/status transmit probe
    /// sent with no backpressure or error on an empty queue, followed by fresh feedback received
    /// strictly after that probe. Non-CAN/sim drivers default to `true`.
    fn tx_probe_confirmed(&self) -> bool { true }

    /// True when a TX probe was recently sent and is still awaiting post-probe fresh feedback.
    fn tx_probe_pending(&self) -> bool { false }

    /// Blocking transmit probe (Legacy `clear_fault` path): requires an empty TX queue, drains RX
    /// to establish a boundary, sends a read-only refresh/status frame without backpressure or
    /// error, flushes, and waits up to 50 ms for fresh post-probe feedback.
    /// Returns `Ok(true)` when post-probe fresh feedback arrived, `Ok(false)` when the probe
    /// transmitted cleanly but no reply arrived yet, or `Err` on backpressure/error.
    fn probe_tx_ready(&mut self) -> Result<bool, ArmDriverError> { Ok(true) }

    /// Non-blocking, zero-allocation budgeted transmit probe for the RT executor `clear_fault` path.
    fn probe_tx_ready_budgeted(&mut self, _deadline: Instant) -> Result<bool, ArmDriverError> { Ok(true) }

    /// Clear any recorded TX probe state (called on fault clear or new TX stall latch).
    fn clear_tx_probe(&mut self) {}

    /// Sets the client lease generation (`0` for hold/autonomous/lease-disabled output) stamped on
    /// motion target entries queued by the immediately following [`Self::write_joint_targets_bounded`].
    fn set_output_lease_gen(&mut self, _lease_gen: u64) {}
}

/// Non-blocking motor control-mode switching primitives (Damiao). Carries no joint angles.
pub trait ModeSwitchExt {
    // ---- PR0C: Non-blocking ModeSwitch driver primitives (0 RT heap allocs) ----

    /// True when the driver supports non-blocking tick-driven mode switching / confirmation.
    fn supports_nonblocking_mode_switch(&self) -> bool { false }

    /// Static hardware vendor binding of this driver (`None` for virtual/mock/sim drivers).
    fn driver_vendor_binding(&self) -> Option<crate::motor_mode::VendorBinding> { None }

    /// Joint count exposed to `ModeSwitch`.
    fn mode_switch_dof(&self) -> usize { 0 }

    /// True when `joint` is currently energized or has an energized/unknown torque state.
    fn axis_is_energized(&self, _joint: usize) -> bool { false }

    /// True when any joint on this driver is currently energized.
    fn any_axis_energized(&self) -> bool { false }

    /// True when `joint` carries a hardware control-frame watchdog (e.g. Damiao J1–J6; false for J7 `TIMEOUT=0`).
    fn axis_has_watchdog(&self, _joint: usize) -> bool { false }

    /// True when `joint` is configured to support `ControlMode::PositionForce` (e.g. J7 gripper).
    fn axis_supports_pos_force(&self, _joint: usize) -> bool { false }

    /// Remaining per-release RX frame allowance (charges Step 2 RX and ModeSwitch RX to one ledger).
    fn mode_switch_remaining_rx_budget(&self, default_max: u16) -> u16 { default_max }

    /// Last causally confirmed control mode for `joint` (`None` if unverified or uncertain).
    fn axis_confirmed_mode(&self, _joint: usize) -> Option<crate::motor_mode::ControlMode> { None }

    /// Sets or clears the causally confirmed control mode for `joint` (also clears/sets driver
    /// uncertainty flags and `pos_force_verified` where applicable).
    fn set_axis_confirmed_mode(&mut self, _joint: usize, _mode: Option<crate::motor_mode::ControlMode>) {}

    /// Immediately clears cached mode evidence and latches `mode_write_uncertain_mask` for `joint`
    /// so no hold, target, or recovery frame can be transmitted until causal verification passes.
    fn latch_axis_mode_uncertain(&mut self, _joint: usize) {}

    /// Bitmask of joints with an unconfirmed mode-changing write that may have reached the wire.
    fn mode_write_uncertain_mask(&self) -> u32 { 0 }

    /// Marks the active `ModeSwitch` joint mask so legacy `mode_readback_boundary` auto-updates
    /// never race with an active `ModeSwitch` transaction.
    fn set_mode_switch_active_mask(&mut self, _mask: u32) {}

    /// Confirmed delivery timestamp of the most recent control frame sent to `joint`.
    fn axis_last_ctrl_tx(&self, _joint: usize) -> Option<Instant> { None }

    /// True when `PMAX`/`VMAX`/`TMAX` wire mapping ranges have been causally verified for `joint`.
    fn axis_impedance_ranges_verified(&self, _joint: usize) -> bool { false }

    /// Configured `[p_max, v_max, t_max]` wire mapping ranges for `joint`.
    fn axis_configured_impedance_ranges(&self, _joint: usize) -> Option<[f32; 3]> { None }

    /// Updates the cached wire-range verification status for `joint`.
    fn set_axis_impedance_ranges_verified(&mut self, _joint: usize, _verified: bool) {}

    /// True iff the most recent RX drain observed an empty transport receive queue.
    fn mode_switch_rx_drained(&self) -> bool { false }

    /// Watchdog-budget-first hold pass for `ModeSwitch`: services due energized verified axes
    /// (oldest-unfed-first, plus `force_mask`) in their last confirmed mode, plus any enabled
    /// verified `PosForce` gripper via `pos_force_emitter`, within `deadline` and `max_frames`.
    fn mode_switch_service_watchdog_holds(
        &mut self,
        _min_age: Duration,
        _exclude_mask: u32,
        _force_mask: u32,
        _deadline: Instant,
        _max_frames: u8,
    ) -> Result<crate::motor_mode::WatchdogHoldResult, crate::motor_mode::ModeSwitchFailure> {
        Ok(crate::motor_mode::WatchdogHoldResult {
            frames_sent: 0,
            refreshed_mask: 0,
            all_due_serviced: true,
            max_energized_age: Duration::ZERO,
            oldest_joint: None,
        })
    }

    /// Single non-blocking `REG_READ(RID_CTRL_MODE)` TX under `deadline` with `pre_tx_gate`.
    fn mode_switch_try_send_ctrl_mode_read(
        &mut self,
        _joint: usize,
        _deadline: Instant,
        _pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::motor_mode::ModeTxResult {
        crate::motor_mode::ModeTxResult::Refused("non-blocking mode switch unsupported")
    }

    /// Single non-blocking `REG_WRITE(RID_CTRL_MODE)` RAM TX under `deadline` with `pre_tx_gate`.
    /// Must immediately clear `verified_modes[joint]` / `joint_modes[joint]` / `pos_force_verified`
    /// and set `mode_write_uncertain_mask` at the transmit boundary (including on failed flush).
    fn mode_switch_try_send_ctrl_mode_write(
        &mut self,
        _joint: usize,
        _mode: crate::motor_mode::ControlMode,
        _deadline: Instant,
        _pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::motor_mode::ModeTxResult {
        crate::motor_mode::ModeTxResult::Refused("non-blocking mode switch unsupported")
    }

    /// Single non-blocking read of `RID_PMAX` (`0`), `RID_VMAX` (`1`), or `RID_TMAX` (`2`).
    fn mode_switch_try_send_range_read(
        &mut self,
        _joint: usize,
        _rid_idx: u8,
        _deadline: Instant,
        _pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::motor_mode::ModeTxResult {
        crate::motor_mode::ModeTxResult::Refused("non-blocking mode switch unsupported")
    }

    /// Single non-blocking read-only status refresh (`0xCC`) for post-confirmation feedback.
    fn mode_switch_try_send_status_refresh(
        &mut self,
        _joint: usize,
        _deadline: Instant,
        _pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::motor_mode::ModeTxResult {
        crate::motor_mode::ModeTxResult::Refused("non-blocking mode switch unsupported")
    }

    /// Scans the preallocated register-reply buffer for a causal `RID_CTRL_MODE` reply after an
    /// observed empty RX queue (`last_read_drained`), rejecting duplicates, overflow, drift,
    /// late replies (`rx_at >= max_rx_at`), and mismatched reply CAN IDs when required.
    #[allow(clippy::too_many_arguments)]
    fn mode_switch_find_ctrl_mode_reply(
        &self,
        _joint: usize,
        _write: bool,
        _after_seq: u64,
        _min_rx_at: Instant,
        _max_rx_at: Instant,
        _tx_offset_ns: Option<i128>,
        _require_expected_id: bool,
    ) -> crate::motor_mode::ModeRegMatch {
        crate::motor_mode::ModeRegMatch::Pending
    }

    /// Scans the preallocated register-reply buffer for a causal `PMAX`/`VMAX`/`TMAX` reply.
    fn mode_switch_find_range_reply(
        &self,
        _joint: usize,
        _rid_idx: u8,
        _after_seq: u64,
        _min_rx_at: Instant,
        _max_rx_at: Instant,
        _tx_offset_ns: Option<i128>,
    ) -> crate::motor_mode::ModeRangeMatch {
        crate::motor_mode::ModeRangeMatch::Pending
    }

    /// Checks whether causal post-confirmation feedback has arrived for `joint`.
    #[allow(clippy::too_many_arguments)]
    fn mode_switch_post_confirm_feedback(
        &self,
        _joint: usize,
        _mode: crate::motor_mode::ControlMode,
        _after_seq: u64,
        _after_at: Instant,
        _max_rx_at: Instant,
        _confirm_offset_ns: Option<i128>,
        _now: Instant,
        _require_disarmed_stationary: bool,
    ) -> crate::motor_mode::ModeFeedbackMatch {
        crate::motor_mode::ModeFeedbackMatch::Pending
    }
}

/// Motor commissioning: zero, identity, command timeout, register steps.
pub trait CommissioningExt {
    /// Make the current position of joint `joint` (0-based) its zero. Never moves the arm and
    /// never enables torque. Needs torque off; a driver with enable feedback also needs fresh
    /// feedback that confirms the joint disabled. Called from the motor zero command
    /// (`/api/v1/calibration/motor_zero`), never from the control tick.
    fn set_zero_position(&mut self, _joint: usize) -> Result<(), ArmDriverError> {
        Err(ArmDriverError::Protocol("motor zero is not supported by this driver".into()))
    }

    /// Fresh register readback of the motors of `joints` (0-based): identity, firmware, mode,
    /// mapping ranges and the command timeout (MIT qualification gate, `src/mit.rs`). Read only.
    fn read_motor_identity(&mut self, _joints: &[usize]) -> Result<Vec<crate::mit::MotorIdentity>, ArmDriverError> {
        Err(ArmDriverError::Protocol("motor identity readback is not supported by this driver".into()))
    }

    /// Write the motor command timeout of `joints` to `counts` in RAM (never flash), check the
    /// acknowledgement and confirm it with a separate read. Torque off only.
    fn commission_motor_timeout(&mut self, joints: &[usize], _counts: u32) -> Result<Vec<crate::mit::TimeoutCommissioning>, crate::mit::TimeoutBatchError> {
        Err(crate::mit::TimeoutBatchError { not_attempted: joints.to_vec(),
            ..ArmDriverError::Protocol("motor timeout commissioning is not supported by this driver".into()).into() })
    }

    // ---- Non-blocking, zero-allocation RT commissioning primitives (C-F1 part 2) ----

    /// True when the driver supports non-blocking, bounded-flush RT register commissioning.
    fn supports_nonblocking_commissioning(&self) -> bool { false }

    /// True when the driver supports non-blocking RT motor-zero commissioning.
    fn supports_nonblocking_motor_zero(&self) -> bool { false }

    /// Command CAN ID of 0-based `joint` (None when unsupported or out of range).
    fn commissioning_motor_id(&self, _joint: usize) -> Option<u16> { None }

    /// Bounded non-blocking RX drain for RT commissioning (records register replies into the
    /// preallocated commissioning buffer without starting a new TX pass). Returns `(frames, drained)`.
    fn commissioning_drain_rx_step(&mut self, _max_frames: u16, _deadline: Instant) -> Result<(u16, bool), ArmDriverError> {
        Err(ArmDriverError::Protocol("non-blocking commissioning RX is not supported by this driver".into()))
    }

    /// Clears the preallocated commissioning register-reply buffer in-place (0 alloc/free).
    fn commissioning_clear_register_log(&mut self) {}

    /// Single non-blocking register read TX (`0x33`) under `deadline`, running `pre_tx_gate`
    /// immediately before `can.transmit` (Step 5 submission boundary).
    fn commissioning_try_send_register_read(
        &mut self,
        _joint: usize,
        _rid: u8,
        _deadline: Instant,
        _pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::mit::CommissioningTxStatus {
        crate::mit::CommissioningTxStatus::Refused("register read commissioning unsupported")
    }

    /// Single non-blocking read-only status refresh probe (`0xCC`) while disarmed to gather
    /// fresh causal rest-pose evidence under `deadline`, running `pre_tx_gate` immediately
    /// before `can.transmit` (Step 5 submission boundary).
    fn commissioning_try_send_rest_probe(
        &mut self,
        _joint: usize,
        _deadline: Instant,
        _pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::mit::CommissioningTxStatus {
        crate::mit::CommissioningTxStatus::Refused("commissioning rest-pose probe unsupported")
    }

    /// Single non-blocking RAM TIMEOUT write TX (`0x55`, RID 9, never flash `0xAA`) under
    /// `deadline`, running `pre_tx_gate` immediately before `can.transmit`.
    fn commissioning_try_send_timeout_write(
        &mut self,
        _joint: usize,
        _counts: u32,
        _deadline: Instant,
        _pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::mit::CommissioningTxStatus {
        crate::mit::CommissioningTxStatus::Refused("TIMEOUT write commissioning unsupported")
    }

    /// Scans the preallocated commissioning register-reply buffer for a reply causally matching
    /// `(joint, write, rid)` strictly after `(after_tx_seq, min_rx_at, tx_offset_ns)` and before
    /// `max_rx_at` (when provided).
    /// Returns [`CommissioningRegMatch::Ambiguous`] if multiple replies, buffer overflow, or any
    /// stale/drifted reply for `(joint, write, rid)` arrived after `after_tx_seq` (B1 fail-closed).
    #[allow(clippy::too_many_arguments)]
    fn commissioning_find_register_reply(
        &self,
        _joint: usize,
        _write: bool,
        _rid: u8,
        _after_tx_seq: u64,
        _min_rx_at: Instant,
        _max_rx_at: Option<Instant>,
        _tx_offset_ns: Option<i128>,
    ) -> crate::mit::CommissioningRegMatch {
        crate::mit::CommissioningRegMatch::Pending
    }

    /// One bounded non-blocking disable-all step (retaining per-motor `disable_dispatched_mask`
    /// across releases) for hardware motor zero.
    fn commissioning_disable_step(
        &mut self,
        _started_at: Instant,
        _deadline: Instant,
        _pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::mit::CommissioningDisableStep {
        crate::mit::CommissioningDisableStep::Refused("non-blocking motor zero disable unsupported")
    }

    /// Single non-blocking motor set-zero TX (`CMD_SET_ZERO`) under `deadline`, running
    /// `pre_tx_gate` immediately before `can.transmit`.
    fn commissioning_try_send_set_zero(
        &mut self,
        _joint: usize,
        _deadline: Instant,
        _pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::mit::CommissioningTxStatus {
        crate::mit::CommissioningTxStatus::Refused("non-blocking motor zero unsupported")
    }

    /// Latest per-joint feedback (raw motor position + calibrated joint position + rx_seq + rx_at).
    fn commissioning_raw_feedback(&self, _joint: usize) -> Option<crate::mit::CommissioningFeedback> {
        None
    }
}

/// Per-joint control modes (POS_VEL / MIT / POS_FORCE), hold-mode verification and readback.
pub trait JointModeExt {
    /// True when `joint` has a verified hold mode capable of receiving stationary hold frames
    /// without enable or mode-switch side effects.
    fn joint_hold_mode_verified(&self, _joint: usize) -> bool { true }

    /// Request a read-only `RID_CTRL_MODE` register readback for every joint set in
    /// `unverified_mask` whose hold mode is still unverified (L2). Bounded by the active per-tick
    /// TX budget and stop-pending gate; zero heap allocations.
    fn request_mode_readback_budgeted(&mut self, _unverified_mask: u64) -> bool {
        false
    }
    /// Calls `f` with the result of the startup check of the motors' mapping ranges against the
    /// configured ones (`None` when the driver has no such check or it has not run). A driver
    /// that stores the result passes a reference instead of a copy.
    fn with_range_check(&self, f: &mut dyn FnMut(Option<&DriverRangeCheck>)) { f(None) }
    /// Joint indices that received an autonomous driver recovery write during
    /// `read_joint_states` since the previous call, into `out` (cleared first), clearing the record.
    fn take_recovery_writes_into(&mut self, out: &mut Vec<usize>) { out.clear(); }

    /// True when single joints can be switched between the driver's position mode and
    /// MIT impedance mode while torqued (`set_joint_modes`). Used by gravity compensation.
    fn supports_mit_mode_switch(&self) -> bool { false }

    /// Switch joint `i` to `modes[i]`. Each switched joint receives `hold[i]` in its new
    /// mode immediately after the switch, so it is never left without a target. A joint
    /// whose switch cannot be confirmed is switched back; joints that still cannot be
    /// confirmed are reported by `joint_modes_uncertain` and get both a position frame
    /// and an MIT frame on every write until a later switch confirms them.
    fn set_joint_modes(&mut self, _modes: &[JointMode], _hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        Err(ArmDriverError::Protocol("joint mode switching is not supported by this driver".into()))
    }

    /// [`Self::set_joint_modes`] invoked from within an active control release: shares the
    /// release's existing per-tick TX pass instead of opening a fresh pass and bypassing any
    /// budget/cutoff already consumed in that release.
    fn set_joint_modes_in_tick(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.set_joint_modes(modes, hold)
    }

    /// At least one joint is in an unconfirmed control mode after a failed switch.
    fn joint_modes_uncertain(&self) -> bool { false }

    /// Strict control-mode handover (joint tracking enabled): a mode switch is confirmed
    /// only when its hold frames and the bus flush also succeed. Off (default) keeps the
    /// pre-tracking behaviour, where a matching mode acknowledgement confirms the switch.
    fn set_strict_mode_handover(&mut self, _strict: bool) {}

    /// MIT tracking handover (joint tracking Phase 2): `set_joint_modes` where a joint is
    /// confirmed only when its mode write, every hold frame, the flush and a matching mode
    /// acknowledgement succeed AND fresh feedback from that motor arrives after its
    /// acknowledgement (samples queued before the switch do not count). Otherwise every changed
    /// joint is switched back (dual holds) and an error is returned.
    fn set_joint_modes_confirmed(&mut self, _modes: &[JointMode], _hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        Err(ArmDriverError::Protocol("confirmed joint mode switching is not supported by this driver".into()))
    }

    /// Sets per-joint hold targets used by `mode_switch_service_watchdog_holds` while a
    /// `ModeSwitch` session is active (0 alloc). Cleared automatically when
    /// `set_mode_switch_active_mask(0)` runs or on `prepare_safe_hold`.
    fn set_mode_switch_hold_targets(&mut self, _targets: &[Option<ModeSwitchHoldTarget>]) {}
}

/// MIT deadline fence and MIT expiry recovery (Damiao). Carries no joint angles.
pub trait MitExt {
    /// Arm (`Some(deadline)`) or disarm the MIT tracking deadline fence. Armed: a joint feedback
    /// age or a gap between complete target batches reaching `deadline` trips it, checked before
    /// every transmit (retries and batch continuations included), and so does an RX backlog the
    /// bounded read cannot drain; no recovery write and no status poll is sent. Tripped: every
    /// target write and mode switch is refused and the cached frames are discarded until the
    /// next confirmed enable; no autonomous frame is sent for `holdoff`.
    fn set_mit_deadline_fence(&mut self, _deadline: Option<Duration>, _holdoff: Duration) {}

    /// Records whether supervisor MIT ownership is currently `Lost` so `verified_hold_mode`
    /// rejects MIT frames while allowing verified position-mode (`PosVel` / `PosForce`) holds
    /// to continue without masking out MIT-configured joints (R12 B2).
    fn set_mit_ownership_lost(&mut self, _lost: bool) {}

    /// Operator recovery after an MIT loss in which a motor disabled itself (its command TIMEOUT
    /// expired, error 0xD) while others stay enabled: clear-error to every motor in an error
    /// state, then disable every motor and require fresh per-motor confirmation (state 0). Sends
    /// no MIT and no POS_VEL target frame. Err names the joints that did not confirm.
    fn mit_expiry_recover(&mut self) -> Result<(), String> {
        Err("MIT expiry recovery is not supported by this driver".into())
    }

    /// Operator recovery after an MIT loss, enforcing `pre_tx_gate` immediately before EVERY
    /// `ClearFault`, `CMD_DISABLE`, barrier `REG_READ`, and `0xCC` CAN submission (after
    /// `before_final_tx_gate_hook`). Fails closed if unsupported by this driver.
    fn mit_expiry_recover_with_gate(
        &mut self,
        _pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> Result<(), String> {
        Err("MIT expiry recovery with live pre_tx_gate is unsupported by this driver".into())
    }

    /// Fresh state of every motor for the MIT expiry recovery decision: a read-only status poll
    /// to each motor after a receive boundary, and the state codes of feedback received after it.
    /// Err names the motors without fresh feedback.
    fn mit_fresh_status(&mut self) -> Result<Vec<u8>, String> {
        Err("fresh status read is not supported by this driver".into())
    }

    /// Why the MIT deadline fence tripped (None = not tripped), as bounded text: zero allocation,
    /// for RT paths (`armd-control`).
    fn mit_fence_trip_bounded(
        &self,
    ) -> Option<crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>> {
        None
    }

    /// Explicit operator clearance of a tripped MIT fence: read-only status polls may resume
    /// (after the hold-off) so fresh feedback can return; target writes stay refused until the
    /// next confirmed enable.
    fn acknowledge_mit_fence_trip(&mut self) {}

    /// True when every joint `0..dof` has received fresh feedback with a receive sequence strictly
    /// after the most recent [`Self::mit_expiry_recover`] request.
    fn mit_feedback_after_recovery(&self, _dof: usize) -> bool { true }

    /// Oldest committed control-frame TX timestamp across the energized axes in `mask`
    /// (`None` if any masked axis has no committed control TX). Zero allocation.
    fn mit_min_last_ctrl_tx(&self, _mask: u64) -> Option<Instant> { None }

    /// Zero-allocation, release-bounded MIT expiry recovery step routed through the single
    /// `TxKind::ClearFault` + `TxKind::DisableRequested { started_at }` Rule A′ choke point.
    /// `pre_tx_gate` is bound directly to the final CAN submission gate immediately before
    /// `can.transmit` (after `before_final_tx_gate_hook`) for EVERY `ClearFault` and
    /// `DisableRequested` frame; returning `Err(code)` aborts TX immediately without allocation.
    /// Returns `Ok(true)` ONLY when every motor has been cleared (if faulted), disabled, and
    /// causally confirmed in state `0` from fresh post-recovery feedback (`<= 100 ms`).
    /// Returns `Ok(false)` when partial progress requires another release within `started_at + 1 s`
    /// or when `pre_tx_gate` aborted transmission.
    /// Returns `Err(TorqueOffWouldDrop | DisableRequestExpired | ...)` on Rule A′ refusal or error.
    fn mit_expiry_recover_budgeted(
        &mut self,
        _deadline: Instant,
        _started_at: Instant,
        _pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> Result<bool, ArmDriverError> {
        Err(ArmDriverError::HardwareFault(
            "MIT expiry recovery is not supported by this driver".into(),
        ))
    }

    /// Resets any per-attempt MIT expiry recovery boundary/cleared-mask state (0 alloc).
    fn mit_expiry_recover_reset(&mut self) {}
}

/// J7 gripper POS_FORCE emission and feedback.
pub trait PosForceExt {
    /// Enable/disable external tick-epilogue ownership of the `PosForce` J7 emitter.
    ///
    /// When `true` (installed by `SupervisorInner` for both Legacy and RT Executor), arm write
    /// paths (`write_joint_targets_bounded`, `write_estop_hold_bounded`, `write_measured_hold_masked`,
    /// `receive_feedback`) NEVER write `0x307` frames; `SupervisorInner`'s tick epilogue calls
    /// [`Self::emit_pos_force_j7`] exactly once per control tick after whatever arm-write path ran.
    fn set_external_pos_force_emitter(&mut self, _enabled: bool) {}

    /// Validate a caller's `PosForce` target parameters (finite position/velocity, non-zero
    /// wire speed quantization, non-zero current cap) without checking live feedback freshness
    /// or transmitting any CAN frame.
    fn validate_pos_force_intent_target(
        &self,
        _position: f32,
        _velocity: Option<f32>,
        _current_raw: Option<u16>,
    ) -> Result<(), ArmDriverError> {
        Ok(())
    }

    /// When attached to `SupervisorInner`, informs the driver before the arm write whether the
    /// upcoming tick-epilogue J7 `PosForce` intent is a `Hold`/`Stop` (Class 1 if J7 is currently
    /// executing motion, Class 3 otherwise) vs a `Target` (Class 2), so the TX scheduler orders
    /// frames strictly by change priority across all joints (J1–J6 and J7).
    fn set_pending_pos_force_hold_hint(&mut self, _is_hold: bool) {}

    /// Single J7 `PosForce` (`0x307`) emitter called once per control tick from the tick epilogue
    /// on both owners (Legacy and RT Executor), after whatever arm-write path ran (whether that
    /// arm path succeeded, refused, tripped the MIT fence, hit `BuildRefused`, had an invalid
    /// caller target, or returned early).
    ///
    /// Always unfenced: a tripped MIT fence suppresses only MIT arm joints, never J7.
    /// Any `Target` intent whose `(stop_gen, estop_epoch)` stamp differs from `current` is
    /// degraded to `Hold` inside this single emitter (never re-stamped with a newer generation).
    fn emit_pos_force_j7(
        &mut self,
        _intent: PosForceIntent,
        _current: PosForceIntentStamp,
    ) -> Result<PosForceEmitOutcome, ArmDriverError> {
        Ok(PosForceEmitOutcome::NotApplicable)
    }

    /// Consumes once the position-force gripper freshness status observed during the most recent
    /// bounded motion or stopped-hold write pass (`Unobserved` on non-pos-force drivers).
    fn take_last_pos_force_tx_status(&mut self) -> PosForceTxStatus { PosForceTxStatus::Unobserved }

    /// Live check whether the active position-force gripper joint currently has provably fresh
    /// (`<= 100 ms` on both receive instant and receive-queue bound, finite position) feedback.
    /// Returns `true` when no active position-force gripper is configured.
    fn pos_force_feedback_fresh(&self) -> bool { true }

    /// Last provably fresh (`<= 100 ms`, bounded, finite) measured position observed for the
    /// position-force gripper joint (`None` when no pos-force gripper is configured or no fresh
    /// sample has been observed yet). Used for J7 freeze frames when J7 feedback is stale.
    fn pos_force_last_fresh_position(&self) -> Option<f32> { None }
    #[doc(hidden)]
    fn reset_pos_force_boot_state(&mut self) {}

    /// The gripper's motor-level current cap (None: the driver has no such notion).
    fn motor_current_cap(&self) -> Option<MotorCurrentCap> { None }

    /// Allocation-free `Copy` summary of an active or configured position-force gripper cap,
    /// safe to call on `armd-control`.
    fn gripper_pos_force_info(&self) -> Option<GripperPosForceInfo> {
        let cap = self.motor_current_cap()?;
        if cap.mode != "pos_force" { return None; }
        let close_speed_rad_s = cap.close_speed_rad_s.filter(|v| v.is_finite() && *v > 0.0)?;
        let ceiling_torque_estimate_nm = cap.ceiling_torque_nm
            .or_else(|| cap.current_raw.map(|raw| f32::from(raw) * crate::model::DamiaoGripperDrive::FEEDBACK_EQUIVALENT_NM_PER_RAW))
            .filter(|v| v.is_finite() && *v > 0.0);
        Some(GripperPosForceInfo {
            joint: cap.joint,
            active: cap.active,
            close_speed_rad_s,
            ceiling_torque_estimate_nm,
        })
    }
}

/// Bus-loss and RX-drain recovery steps (Damiao). Carries no joint angles.
pub trait BusRecoveryExt {
    /// Service at most one pending owner-local `0xCC` refresh slot if this tick reused a
    /// transition RX pass without running `read_joint_states` (zero locks, zero allocations).
    fn service_pending_refresh(&mut self) -> Result<(), ArmDriverError> {
        Ok(())
    }

    /// Completes remaining bounded disable repeats for an already-dispatched disable
    /// (`disable_frames_dispatched() == true`) without re-invoking `begin_disable_torque`.
    fn finish_dispatched_disable(&mut self) -> Result<(), ArmDriverError> {
        Ok(())
    }
    /// Notify the driver that `BusLoss` latched so the next enable must re-verify registers
    /// (`CTRL_MODE` and `TIMEOUT`) causally before energizing any motor.
    fn notify_bus_loss(&mut self) {}

    /// One panic-disable retry step for multi-motor drivers: transmits disable to every motor
    /// not yet confirmed disabled without aborting remaining motors on a single-motor TX error,
    /// performs one budgeted RX pass within `deadline`, and returns the 0-based indices of motors
    /// still unconfirmed. Returns `None` to fall back to the generic `begin_torque_budgeted` loop.
    fn panic_disable_step(
        &mut self,
        _first: bool,
        _deadline: Instant,
    ) -> Option<Result<Vec<usize>, ArmDriverError>> {
        None
    }

    /// Pure RX-only bounded drain with zero TX and zero heap allocations; establishes a clean
    /// empty-queue boundary before evaluating rest-pose / disarm gates for carried-over clears.
    fn drain_rx_only_bounded(&mut self, _deadline: Option<Instant>) -> Result<(), ArmDriverError> {
        Ok(())
    }

    /// Clear carried-over motor hardware errors on affected motors ONLY while disarmed, then
    /// verify on a fresh post-clear status reply that every affected motor's error bit is gone
    /// and the motor stays disabled (never sends an enable command).
    ///
    /// Runs `pre_tx_gate` immediately before every CAN frame submission (re-verifying live
    /// operator authority and pending E-stop at the hardware write boundary). When `deadline` is
    /// `Some`, operates within the caller's RT release budget with zero heap allocations.
    fn clear_carried_over_motor_errors(
        &mut self,
        _deadline: Option<Instant>,
        _pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> Result<CarryoverMotorClearOutcome, ArmDriverError> {
        Err(ArmDriverError::ProtocolStatic(
            "UNSUPPORTED: driver does not support targeted motor error clear",
        ))
    }

    /// Update pending_rearm flag to force full hardware handshake on next enable
    fn set_pending_rearm(&mut self, _pending: bool) {}
}

/// Shadow `TorqueController` observation staging.
#[cfg(not(target_arch = "wasm32"))]
pub trait ShadowExt {
    /// Vendor family reported to the shadow `TorqueController` reducer.
    fn shadow_vendor_kind(&self) -> crate::torque::VendorKind {
        crate::torque::VendorKind::Damiao
    }

    /// Enable or disable inline fixed-capacity shadow observation staging inside the driver.
    fn set_shadow_enabled(&mut self, _enabled: bool) {}

    /// Drain staged shadow observations into `out` while the caller ALREADY holds the driver lock.
    /// Returns `(copied, dropped_count)` with zero heap allocations.
    fn drain_shadow_observations(
        &mut self,
        _out: &mut [Option<crate::torque::shadow::ShadowDriverObs>],
    ) -> (usize, u64) {
        (0, 0)
    }
}

/// Joint zero offsets applied at the driver boundary (`ZeroOffsetDriver`).
pub trait JointZeroExt {
    /// Joint zero offsets (motor - joint, rad) applied at the driver boundary
    /// ([`crate::joint_zero::ZeroOffsetDriver`]); a driver without the boundary ignores them.
    fn set_joint_zero_offsets(&mut self, _offsets: &[f32]) {}

    /// The offsets in effect at the boundary; None for a driver without it.
    fn joint_zero_offsets(&self) -> Option<&[f32]> { None }
}

/// The "no extension" object: implements every `*Ext` with the default bodies only.
#[derive(Debug, Clone, Copy, Default)]
pub struct NoExt;

impl BoundedTxExt for NoExt {}
impl ModeSwitchExt for NoExt {}
impl CommissioningExt for NoExt {}
impl JointModeExt for NoExt {}
impl MitExt for NoExt {}
impl PosForceExt for NoExt {}
impl BusRecoveryExt for NoExt {}
#[cfg(not(target_arch = "wasm32"))]
impl ShadowExt for NoExt {}
impl JointZeroExt for NoExt {}

macro_rules! ext_getters {
    ($($(#[$meta:meta])* $tr:ident, $mutty:ident, $get:ident, $get_mut:ident, $raw:ident, $raw_mut:ident;)*) => {
        $(
            /// `&mut` view of one extension of a `dyn ArmDriver`: the driver's own, or [`NoExt`].
            $(#[$meta])*
            pub enum $mutty<'a> {
                Real(&'a mut dyn $tr),
                Noop(NoExt),
            }
            $(#[$meta])*
            impl<'a> std::ops::Deref for $mutty<'a> {
                type Target = dyn $tr + 'a;
                #[inline]
                fn deref(&self) -> &Self::Target {
                    match self {
                        Self::Real(e) => &**e,
                        Self::Noop(n) => n,
                    }
                }
            }
            $(#[$meta])*
            impl<'a> std::ops::DerefMut for $mutty<'a> {
                #[inline]
                fn deref_mut(&mut self) -> &mut Self::Target {
                    match self {
                        Self::Real(e) => &mut **e,
                        Self::Noop(n) => n,
                    }
                }
            }
        )*
        impl dyn ArmDriver + '_ {
            $(
                /// This driver's extension, or [`NoExt`] (the default bodies).
                $(#[$meta])*
                #[inline]
                pub fn $get(&self) -> &dyn $tr {
                    self.$raw().unwrap_or(&NoExt)
                }
                /// `&mut` form of the getter above.
                $(#[$meta])*
                #[inline]
                pub fn $get_mut(&mut self) -> $mutty<'_> {
                    match self.$raw_mut() {
                        Some(e) => $mutty::Real(e),
                        None => $mutty::Noop(NoExt),
                    }
                }
            )*
        }
    };
}

ext_getters! {
    BoundedTxExt, TxMut, tx, tx_mut, tx_ext, tx_ext_mut;
    ModeSwitchExt, ModeSwitchMut, mode_switch, mode_switch_mut, mode_switch_ext, mode_switch_ext_mut;
    CommissioningExt, CommissioningMut, commissioning, commissioning_mut, commissioning_ext, commissioning_ext_mut;
    JointModeExt, JointModeMut, joint_mode, joint_mode_mut, joint_mode_ext, joint_mode_ext_mut;
    MitExt, MitMut, mit, mit_mut, mit_ext, mit_ext_mut;
    PosForceExt, PosForceMut, pos_force, pos_force_mut, pos_force_ext, pos_force_ext_mut;
    BusRecoveryExt, BusRecoveryMut, bus_recovery, bus_recovery_mut, bus_recovery_ext, bus_recovery_ext_mut;
    #[cfg(not(target_arch = "wasm32"))]
    ShadowExt, ShadowMut, shadow, shadow_mut, shadow_ext, shadow_ext_mut;
    JointZeroExt, JointZeroMut, joint_zero, joint_zero_mut, joint_zero_ext, joint_zero_ext_mut;
}

/// Accessor pairs for the extensions a driver implements, by getter name:
/// `ext_accessors!(tx, joint_mode);` inside `impl ArmDriver for D` expands to
/// `fn tx_ext(&self) -> Option<&dyn BoundedTxExt> { Some(self) }`, its `_mut` twin, and so on.
#[macro_export]
macro_rules! ext_accessors {
    ($($get:ident),* $(,)?) => { $($crate::ext_accessors!(@one $get);)* };
    (@one tx) => {
        fn tx_ext(&self) -> Option<&dyn $crate::driver::BoundedTxExt> { Some(self) }
        fn tx_ext_mut(&mut self) -> Option<&mut dyn $crate::driver::BoundedTxExt> { Some(self) }
    };
    (@one mode_switch) => {
        fn mode_switch_ext(&self) -> Option<&dyn $crate::driver::ModeSwitchExt> { Some(self) }
        fn mode_switch_ext_mut(&mut self) -> Option<&mut dyn $crate::driver::ModeSwitchExt> { Some(self) }
    };
    (@one commissioning) => {
        fn commissioning_ext(&self) -> Option<&dyn $crate::driver::CommissioningExt> { Some(self) }
        fn commissioning_ext_mut(&mut self) -> Option<&mut dyn $crate::driver::CommissioningExt> { Some(self) }
    };
    (@one joint_mode) => {
        fn joint_mode_ext(&self) -> Option<&dyn $crate::driver::JointModeExt> { Some(self) }
        fn joint_mode_ext_mut(&mut self) -> Option<&mut dyn $crate::driver::JointModeExt> { Some(self) }
    };
    (@one mit) => {
        fn mit_ext(&self) -> Option<&dyn $crate::driver::MitExt> { Some(self) }
        fn mit_ext_mut(&mut self) -> Option<&mut dyn $crate::driver::MitExt> { Some(self) }
    };
    (@one pos_force) => {
        fn pos_force_ext(&self) -> Option<&dyn $crate::driver::PosForceExt> { Some(self) }
        fn pos_force_ext_mut(&mut self) -> Option<&mut dyn $crate::driver::PosForceExt> { Some(self) }
    };
    (@one bus_recovery) => {
        fn bus_recovery_ext(&self) -> Option<&dyn $crate::driver::BusRecoveryExt> { Some(self) }
        fn bus_recovery_ext_mut(&mut self) -> Option<&mut dyn $crate::driver::BusRecoveryExt> { Some(self) }
    };
    (@one shadow) => {
        #[cfg(not(target_arch = "wasm32"))] fn shadow_ext(&self) -> Option<&dyn $crate::driver::ShadowExt> { Some(self) }
        #[cfg(not(target_arch = "wasm32"))] fn shadow_ext_mut(&mut self) -> Option<&mut dyn $crate::driver::ShadowExt> { Some(self) }
    };
    (@one joint_zero) => {
        fn joint_zero_ext(&self) -> Option<&dyn $crate::driver::JointZeroExt> { Some(self) }
        fn joint_zero_ext_mut(&mut self) -> Option<&mut dyn $crate::driver::JointZeroExt> { Some(self) }
    };
}
