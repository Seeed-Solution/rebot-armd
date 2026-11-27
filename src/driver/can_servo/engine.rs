//! Damiao CAN actuator arm driver struct, constructors, and core accessors.

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
    pub fn new(can: C, motor_ids: Vec<u16>) -> Self {
        let dof = motor_ids.len();
        let clock = can.clock();
        Self {
            can,
            motor_ids,
            master_id: 0,
            configured_master_id: None,
            limits: vec![DamiaoLimits::default(); dof],
            control_mode: DamiaoControlMode::PosVel,
            default_vlim: 2.0,
            default_kp: 40.0,
            default_kd: 1.0,
            cached_states: vec![JointState::default(); dof],
            last_target_frames: vec![None; dof],
            last_mit_params: vec![None; dof],
            last_feedback_instant: vec![None; dof],
            pose_feedback_instant: vec![None; dof],
            feedback_seq: vec![0; dof],
            rx_empty_instant: None,
            feedback_queue_bound: vec![None; dof],
            motor_status: vec![0; dof],
            is_enabled: false,
            pending_rearm: true,
            last_query_instant: None,
            last_enable_command: None,
            last_enable_command_rx_seq: None,
            last_command_was_disable: false,
            awaiting_rx_boundary: false,
            is_enabling: false,
            last_warn_sec: AtomicU64::new(0),
            joint_modes: vec![Some(DamiaoControlMode::PosVel); dof],
            verified_modes: vec![None; dof],
            mode_readback_boundary: vec![None; dof],
            mode_write_uncertain_mask: 0,
            mode_switch_active_mask: 0,
            mode_switch_hold_targets: [None; MAX_MODE_JOINTS],
            mit_expiry_budgeted_boundary: None,
            mit_expiry_recovery_active: false,
            mit_expiry_cleared_mask: 0,
            mit_expiry_axes: [MitExpiryAxisState::default(); MAX_MODE_JOINTS],
            last_adverse_feedback_seq: [0; MAX_MODE_JOINTS],
            release_rx_frames: 0,
            release_rx_limit: crate::rt_core::ops::RX_FRAME_BUDGET,
            release_tx_limit: None,
            mit_expiry_confirmed_disabled_mask: 0,
            last_feedback_decode_offset_ns: vec![None; dof],
            uncertain_readback_probe: [None; MAX_MODE_JOINTS],
            mit_limits_verified: vec![false; dof],
            expected_timeout_counts: vec![None; dof],
            timeout_check_enabled: false,
            timeout_verified: vec![false; dof],
            startup_register_check_pending: false,
            attempt_verified_modes_mask: 0,
            staged_wire_pos: [None; MAX_MODE_JOINTS],
            staged_wire_pos_mask: 0,
            committed_wire_pos: [None; MAX_MODE_JOINTS],
            wire_pos_commit_seq: 0,
            register_replies: Vec::with_capacity(64),
            range_check: None,
            recovery_written_joints: Vec::with_capacity(dof.max(16)),
            strict_mode_handover: false,
            register_reply_log: Vec::with_capacity(64),
            comm_reg_replies: [None; 64],
            comm_reg_reply_len: 0,
            comm_reg_reply_overflow: false,
            before_final_tx_gate_hook: None,
            clock,
            rx_seq: 0,
            last_tx_mark: None,
            last_feedback_seq: vec![0; dof],
            feedback_continuity_epoch: vec![0; dof],
            rest_pose_boundary: vec![None; dof],
            control_release_id: 0,
            control_release_managed: false,
            rest_pose_window: vec![CausalWindow::default(); dof],
            motor_enable_seq: vec![0; dof],
            motor_last_enable_tx: vec![None; dof],
            motor_disarmed_after_enable: vec![false; dof],
            motor_post_enable_feedback: vec![false; dof],
            reported_torque_loss_sample_seq: 0,
            panic_disable_active: false,
            panic_disable_tx_boundary: vec![None; dof],
            panic_disable_confirmed: vec![false; dof],
            disarm_probe_active: false,
            disarm_probe_adverse_mask: 0,
            disarm_probe_adverse_state: [0; MAX_MODE_JOINTS],
            rx_empty_seq: 0,
            last_read_drained: false,
            mit_fence: None,
            mit_fence_last_cmd: vec![None; dof],
            estop_gate: None,
            output_lease_gen: 0,
            send_lease_gen: 0,
            abort_flag: None,
            stopped_disable_gate: None,
            estop_state: None,
            rt_no_sleep: false,
            mit_fence_trip: None,
            mit_fence_holdoff: Duration::ZERO,
            mit_fence_acked: false,
            mit_ownership_lost: false,
            last_mit_recovery_rx_seq: None,
            mit_tracking_used: false,
            torque_hs: None,
            disable_dispatched_mask: 0,
            disable_rest_established: false,
            disable_tx_deadline: None,
            disable_lease_gen: None,
            rx_deadline: None,
            in_poll_torque: false,
            rx_frame_limit: None,
            last_poll_rx: None,
            base_modes: vec![DamiaoControlMode::PosVel; dof],
            pos_force: None,
            pos_force_verified: false,
            legacy_gripper: None,
            refresh_capture: None,
            tx_budget: TxBudget::default(),
            pending_tx: PendingTxQueue::new(),
            pending_hold_tx: PendingTxQueue::new(),
            hold_cursor: 0,
            last_hold_tx_sent_any: false,
            last_target_tx_sent_mask: 0,
            last_expired_oldest_age: None,
            tick_tx: None,
            refresh_scheduler: RefreshScheduler::default(),
            enable_stop_stamp: None,
            pf_last_fresh_feedback: None,
            pf_last_fresh_position: None,
            tx_release_deadline: None,
            enable_tx_deadline: None,
            enable_lease_gen: None,
            enable_frames_sent_any: false,
            tx_probe: None,
            last_tx_pf_status: PosForceTxStatus::Unobserved,
            pf_fresh_gen: 0,
            pf_refresh_pending: false,
            pf_refresh_deferred_ticks: 0,
            output_ran_since_rx: false,
            last_output_was_hold: false,
            last_hold_tx_due_mask: 0,
            last_hold_tx_sent_mask: 0,
            external_pos_force_emitter: false,
            pf_deferred_ticks: 0,
            pf_deferred_motion: false,
            pf_deferred_since: None,
            pf_deferred_intent: None,
            slot_in_unchanged_hold: [false; MAX_MODE_JOINTS * 2],
            tx_progress_seq: 0,
            slot_last_tx_seq: [0; MAX_MODE_JOINTS * 2],
            pf_next_intent_is_hold: false,
            pf_hold_on_first_fresh: false,
            #[cfg(not(target_arch = "wasm32"))]
            shadow_staging: crate::torque::shadow::ShadowDriverStaging::new(false),
            #[cfg(not(target_arch = "wasm32"))]
            defer_disable_shadow: false,
            #[cfg(not(target_arch = "wasm32"))]
            shadow_disable_pass_sent_mask: 0,
            #[cfg(not(target_arch = "wasm32"))]
            pending_disable_shadow: None,
            motor_energized_mask: 0,
            motor_last_ctrl_tx: [None; MAX_MODE_JOINTS],
            motor_keepalive_hold_pos: [None; MAX_MODE_JOINTS],
            release_ctrl_tx_mask: 0,
            staged_ctrl_tx: [None; MAX_MODE_JOINTS],
            staged_ctrl_tx_mask: 0,
            output_step_pending_in_release: false,
            release_tx_pass_release_id: 0,
            release_recovery_tx_count: 0,
            recovery_cursor: 0,
            in_urgent_watchdog_hold: false,
        }
    }

    /// Active per-tick CAN transmit budget.
    pub fn tx_budget(&self) -> TxBudget { self.tx_budget }

    /// Override the per-tick CAN transmit budget (builder form).
    pub fn with_tx_budget(mut self, budget: TxBudget) -> Self {
        self.tx_budget = budget;
        self
    }

    /// Override the per-tick CAN transmit budget and open a fresh tick pass.
    pub fn set_tx_budget(&mut self, budget: TxBudget) {
        self.tx_budget = budget;
        self.tick_tx = None;
        self.tx_release_deadline = None;
        self.release_tx_limit = None;
    }

    /// Open a fresh per-tick TX budget window. Called automatically at the start of
    /// `receive_feedback()`; exposed for direct driver tests and executors that step TX
    /// independently of `read_joint_states()`.
    ///
    /// While executing inside an active managed control release that already opened its TX pass,
    /// `begin_tx_tick` preserves attempts and coalescing state already spent in this release so
    /// intra-release maintenance or recovery calls can never regain spent TX slots.
    pub fn begin_tx_tick(&mut self) {
        if self.in_active_control_release()
            && self.release_tx_pass_release_id == self.control_release_id
        {
            self.discard_staged_ctrl_tx();
            return;
        }
        self.tick_tx = None;
        self.refresh_scheduler.begin_tick();
        self.release_ctrl_tx_mask = 0;
        self.discard_staged_ctrl_tx();
        self.release_recovery_tx_count = 0;
        self.release_tx_pass_release_id = self.control_release_id;
    }

    /// True when executing inside an active owner-managed control release (both
    /// `control_release_managed` and an installed `tx_release_deadline`).
    #[inline]
    pub(in crate::driver::can_servo) fn in_active_control_release(&self) -> bool {
        self.control_release_managed && self.tx_release_deadline.is_some()
    }

    /// Record a watchdog-refreshing control TX at the confirmed delivery boundary: immediately on
    /// no-op-flush transports, or staged until `flush_can_and_commit()` succeeds on real-flush
    /// transports.
    #[inline]
    pub(in crate::driver::can_servo) fn record_ctrl_tx_delivered(&mut self, idx: usize, at: Instant) {
        if idx < MAX_MODE_JOINTS {
            if self.can.flush_is_noop() {
                self.motor_last_ctrl_tx[idx] = Some(at);
                self.release_ctrl_tx_mask |= 1u32 << idx;
                if self.mit_expiry_recovery_active {
                    self.mit_expiry_axes[idx].record_hold_cause(at);
                }
            } else {
                self.staged_ctrl_tx[idx] = Some(at);
                self.staged_ctrl_tx_mask |= 1u32 << idx;
            }
        }
    }

    /// Extract the exact wire-quantized motor-space target position (rad) from a non-skipped
    /// Damiao control command frame (PosVel `0x100 + id`, PosForce `0x300 + id`, or MIT `id`).
    #[inline]
    pub(in crate::driver::can_servo) fn extract_wire_target_pos(&self, idx: usize, frame: &CanFrame) -> Option<f32> {
        if frame.is_extended || frame.is_rtr || frame.dlc != 8 {
            return None;
        }
        let base = u32::from(self.motor_ids.get(idx).copied()?);
        if frame.id == 0x100 + base || self.frame_pos_force_joint(frame) == Some(idx) {
            let pos = f32::from_le_bytes([frame.data[0], frame.data[1], frame.data[2], frame.data[3]]);
            return pos.is_finite().then_some(pos);
        }
        if frame.id == base
            && frame.data != damiao::CMD_ENABLE
            && frame.data != damiao::CMD_DISABLE
            && frame.data != damiao::CMD_CLEAR_FAULTS
            && frame.data != damiao::CMD_SET_ZERO
        {
            let p_raw = ((frame.data[0] as u16) << 8) | (frame.data[1] as u16);
            let lim = self.limits.get(idx)?;
            let pos = damiao::uint_to_float(p_raw, lim.p_min, lim.p_max, 16);
            return pos.is_finite().then_some(pos);
        }
        None
    }

    /// Record a wire-quantized target position at the confirmed delivery boundary (staged until
    /// `flush_can_and_commit()` succeeds on real-flush transports).
    #[inline]
    pub(in crate::driver::can_servo) fn record_wire_pos_delivered(&mut self, idx: usize, tx_at: Instant, wire_q: f32) {
        if idx < MAX_MODE_JOINTS && wire_q.is_finite() {
            if self.can.flush_is_noop() {
                self.wire_pos_commit_seq = self.wire_pos_commit_seq.wrapping_add(1).max(1);
                self.committed_wire_pos[idx] = Some((self.wire_pos_commit_seq, tx_at, wire_q));
            } else {
                self.staged_wire_pos[idx] = Some((tx_at, wire_q));
                self.staged_wire_pos_mask |= 1u32 << idx;
            }
        }
    }

    /// Commit all staged real-flush control TX timestamps after a confirmed `can.flush()`.
    #[inline]
    pub(in crate::driver::can_servo) fn commit_staged_ctrl_tx(&mut self) {
        let mut mask = self.staged_ctrl_tx_mask;
        while mask != 0 {
            let idx = mask.trailing_zeros() as usize;
            mask &= mask - 1;
            if idx < MAX_MODE_JOINTS {
                if let Some(at) = self.staged_ctrl_tx[idx].take() {
                    self.motor_last_ctrl_tx[idx] = Some(at);
                    self.release_ctrl_tx_mask |= 1u32 << idx;
                    if self.mit_expiry_recovery_active {
                        self.mit_expiry_axes[idx].record_hold_cause(at);
                    }
                }
            }
        }
        self.staged_ctrl_tx_mask = 0;
        let mut wmask = self.staged_wire_pos_mask;
        while wmask != 0 {
            let idx = wmask.trailing_zeros() as usize;
            wmask &= wmask - 1;
            if idx < MAX_MODE_JOINTS {
                if let Some((tx_at, q)) = self.staged_wire_pos[idx].take() {
                    self.wire_pos_commit_seq = self.wire_pos_commit_seq.wrapping_add(1).max(1);
                    self.committed_wire_pos[idx] = Some((self.wire_pos_commit_seq, tx_at, q));
                }
            }
        }
        self.staged_wire_pos_mask = 0;
    }

    /// Discard staged control TX timestamps without crediting watchdog refreshes.
    #[inline]
    pub(in crate::driver::can_servo) fn discard_staged_ctrl_tx(&mut self) {
        let mut mask = self.staged_ctrl_tx_mask;
        while mask != 0 {
            let idx = mask.trailing_zeros() as usize;
            mask &= mask - 1;
            if idx < MAX_MODE_JOINTS {
                self.staged_ctrl_tx[idx] = None;
            }
        }
        self.staged_ctrl_tx_mask = 0;
        let mut wmask = self.staged_wire_pos_mask;
        while wmask != 0 {
            let idx = wmask.trailing_zeros() as usize;
            wmask &= wmask - 1;
            if idx < MAX_MODE_JOINTS {
                self.staged_wire_pos[idx] = None;
            }
        }
        self.staged_wire_pos_mask = 0;
    }

    /// Single confirmed-delivery flush boundary: commits staged watchdog timestamps iff `flush()`
    /// succeeds, and discards them on error.
    #[inline]
    pub(in crate::driver::can_servo) fn flush_can_and_commit(&mut self) -> Result<(), TransportError> {
        match self.can.flush() {
            Ok(()) => {
                self.commit_staged_ctrl_tx();
                Ok(())
            }
            Err(e) => {
                self.discard_staged_ctrl_tx();
                Err(e)
            }
        }
    }

    /// Open a fresh per-release TX pass only if one has not already been opened for the active
    /// managed control release (`in_active_control_release()`).
    /// Prevents multiple Step 2 / RX entry points inside Window 1 from resetting the release TX
    /// budget or refresh slot, while off-tick blocking loops (`tx_release_deadline == None`)
    /// still open a fresh TX pass on every iteration.
    #[inline]
    pub(in crate::driver::can_servo) fn ensure_release_tx_tick(&mut self) {
        if !self.in_active_control_release() || self.release_tx_pass_release_id != self.control_release_id {
            self.begin_tx_tick();
        }
    }

    /// True when enough release budget remains to start optional (deferrable) CAN work such as
    /// background `0xCC` status polls or autonomous recovery frames without risking a deadline
    /// overrun. Always returns `true` when no release deadline cap is installed.
    #[inline]
    pub(in crate::driver::can_servo) fn has_optional_release_budget(&self) -> bool {
        let Some(cap) = self.tx_release_deadline else {
            return true;
        };
        let extra_margin = OPTIONAL_WORK_RELEASE_MARGIN
            .saturating_sub(DAMIAO_TX_RELEASE_PUBLISH_MARGIN)
            .min(self.tx_budget.max_duration / 2);
        cap.saturating_duration_since(self.now()) >= extra_margin
    }

    #[inline]
    pub(in crate::driver) fn current_stop_stamp(&self) -> StopStamp {
        let (estop_gen, stop_gen) = self
            .estop_gate
            .as_ref()
            .map_or((0, 0), crate::rt_core::urgent::EstopGate::generations);
        StopStamp { estop_gen, stop_gen }
    }

    #[inline]
    pub(in crate::driver::can_servo) fn current_lease_gen(&self) -> u64 {
        self.estop_gate
            .as_ref()
            .map_or(0, crate::rt_core::urgent::EstopGate::current_lease_gen)
    }

    #[inline]
    pub(in crate::driver::can_servo) fn is_lease_fenced(&self, entry_lease_gen: u64) -> bool {
        self.estop_gate
            .as_ref()
            .is_some_and(|g| g.is_lease_fenced(entry_lease_gen))
    }

    /// Number of unsent target frames currently queued for the next tick.
    pub fn pending_tx_len(&self) -> usize { self.pending_tx.len() }

    /// Number of unsent stopped-hold frames currently queued for the next tick.
    pub fn pending_hold_tx_len(&self) -> usize { self.pending_hold_tx.len() }

    #[inline]
    pub(in crate::driver::can_servo) fn total_pending_tx_len(&self) -> u32 {
        (self.pending_tx.len().saturating_add(self.pending_hold_tx.len())) as u32
    }

    /// Explicitly drop all queued motion frames (e.g. on an ordinary stop from a bounded caller).
    /// Returns how many frames were dropped and counts them under `tx_dropped_stopped`.
    pub fn discard_pending_tx(&mut self) -> usize {
        self.tx_probe = None;
        let n = self.pending_tx.clear();
        self.pf_deferred_motion = false;
        self.pf_deferred_since = None;
        self.pf_deferred_intent = None;
        if n > 0 {
            crate::diagnostics::tick::record_tx_dropped_stopped(n as u32);
        }
        crate::diagnostics::tick::record_tx_deferred(0, self.total_pending_tx_len());
        n
    }

    /// Drop carried stopped-hold frames from an older stop generation.
    pub(in crate::driver::can_servo) fn drop_stopped_hold_tx(&mut self, stamp: StopStamp) -> usize {
        let dropped = self.pending_hold_tx.drop_stopped(stamp);
        if dropped > 0 {
            crate::diagnostics::tick::record_tx_dropped_stopped(dropped as u32);
        }
        dropped
    }

    /// Drop all queued stopped-hold frames and reset the round-robin hold cursor.
    pub(in crate::driver::can_servo) fn discard_pending_hold_tx(&mut self) {
        self.tx_probe = None;
        let n = self.pending_hold_tx.clear();
        self.hold_cursor = 0;
        if n > 0 {
            crate::diagnostics::tick::record_tx_dropped_stopped(n as u32);
        }
        crate::diagnostics::tick::record_tx_deferred(0, self.total_pending_tx_len());
    }

    /// Drop both queued motion and queued stopped-hold frames (lifecycle transitions out of hold).
    pub(in crate::driver::can_servo) fn discard_all_pending_tx(&mut self) {
        let _ = self.discard_pending_tx();
        self.discard_pending_hold_tx();
        self.pf_deferred_ticks = 0;
    }

    #[inline]
    pub(in crate::driver::can_servo) fn ensure_tx_pass(&mut self) -> TickTxPass {
        if let Some(pass) = self.tick_tx {
            pass
        } else {
            // Keep `pass.deadline` bound solely to `tx_budget.max_duration` so raising
            // `tx_release_deadline` from an expired action cutoff back to `drv_release_cap` for
            // stop/hold/freeze frames is never poisoned by the earlier action cutoff. Every TX
            // gate combines `pass.deadline` with `self.tx_release_deadline` dynamically via
            // `effective_tx_pass_deadline`.
            let pass = TickTxPass {
                deadline: self.now() + self.tx_budget.max_duration,
                attempts: 0,
                backpressured: false,
            };
            self.tick_tx = Some(pass);
            pass
        }
    }

    #[inline]
    pub(in crate::driver::can_servo) fn effective_tx_pass_deadline(&self, pass: TickTxPass) -> Instant {
        self.tx_release_deadline
            .map_or(pass.deadline, |cap| pass.deadline.min(cap))
    }

    #[inline]
    pub(in crate::driver::can_servo) fn can_start_tick_tx(&self) -> bool {
        self.can_start_tick_tx_n(1)
    }

    /// True when a transport with a real (`!flush_is_noop()`) flush cannot start `flush()`
    /// because the current tick TX pass deadline has already elapsed.
    #[inline]
    pub(in crate::driver::can_servo) fn flush_cutoff_exceeded(&self) -> bool {
        !self.can.flush_is_noop()
            && (self.tick_tx.is_some_and(|pass| self.now() >= pass.deadline)
                || self.tx_release_deadline.is_some_and(|cap| self.now() >= cap))
    }

    /// Effective budgeted TX attempt cap for the current control release (tightened monotonically
    /// by the owner's [`crate::rt_core::budget::ReleaseBudget`] only while inside an active
    /// managed control release; standalone off-tick driver operations and non-deferrable
    /// watchdog-continuity Hold/Stop frames use `self.tx_budget.max_frames`).
    #[inline]
    pub(in crate::driver::can_servo) fn effective_tx_max_frames(&self) -> usize {
        self.effective_tx_max_frames_with_bypass(ContinuityBypass::None)
    }

    #[inline]
    pub(in crate::driver::can_servo) fn effective_tx_max_frames_with_bypass(&self, bypass: ContinuityBypass) -> usize {
        if !self.in_active_control_release() || bypass.bypasses_ledger() {
            return self.tx_budget.max_frames;
        }
        match self.release_tx_limit {
            Some(lim) => self.tx_budget.max_frames.min(usize::from(lim)),
            None => self.tx_budget.max_frames,
        }
    }

    /// Explicit release cutoff deadline combining `tx_release_deadline` and the active tick pass
    /// deadline so `send()` enforces the cutoff immediately before final CAN submission.
    #[inline]
    pub(in crate::driver::can_servo) fn active_release_cutoff_deadline(&self) -> Option<Deadline> {
        match (self.tx_release_deadline, self.tick_tx.map(|p| p.deadline)) {
            (Some(cap), Some(pass_dl)) => Some(Deadline { at: cap.min(pass_dl) }),
            (Some(cap), None) => Some(Deadline { at: cap }),
            (None, Some(pass_dl)) => Some(Deadline { at: pass_dl }),
            (None, None) => None,
        }
    }

    #[inline]
    pub(in crate::driver::can_servo) fn can_start_tick_tx_n(&self, needed: usize) -> bool {
        self.can_start_tick_tx_n_with_bypass(needed, ContinuityBypass::None)
    }

    #[inline]
    pub(in crate::driver::can_servo) fn can_start_tick_tx_n_with_bypass(&self, needed: usize, bypass: ContinuityBypass) -> bool {
        let max_frames = self.effective_tx_max_frames_with_bypass(bypass);
        let release_cap = if bypass.bypasses_release_deadline() {
            None
        } else {
            self.tx_release_deadline
        };
        let Some(pass) = self.tick_tx else {
            return needed <= max_frames && release_cap.is_none_or(|cap| self.now() < cap);
        };
        let effective_deadline = release_cap.map_or(pass.deadline, |cap| pass.deadline.min(cap));
        !pass.backpressured
            && pass.attempts.saturating_add(needed) <= max_frames
            && (bypass.bypasses_release_deadline() || self.now() < effective_deadline)
    }

    #[inline]
    pub(in crate::driver::can_servo) fn can_start_tick_tx_for_kind(
        &self,
        kind: TxKind,
        _fenced: bool,
        bypass: ContinuityBypass,
        frame_axis: Option<usize>,
        is_pf_freeze: bool,
    ) -> bool {
        // Continuity bypass is valid ONLY for stationary Hold/Stop frames; never for Target motion.
        let effective_bypass = bypass.for_tx_kind(kind);
        if !self.can_start_tick_tx_n_with_bypass(1, effective_bypass) {
            return false;
        }
        if !self.in_active_control_release() {
            return true;
        }
        let reserved_mask = self.watchdog_continuity_reserved_mask();
        if reserved_mask == 0 {
            return true;
        }
        let mit_recovery = self.mit_expiry_recovery_active || self.mit_fence_trip.is_some();
        let covered_axis = match kind {
            TxKind::MeasuredHold(j) => {
                (j < 32 && (reserved_mask & (1u32 << j)) != 0).then_some(j)
            }
            TxKind::RollbackHold(j) => {
                if (j < 32 && (reserved_mask & (1u32 << j)) != 0)
                    || self.active_pos_force_joint() == Some(j)
                {
                    Some(j)
                } else {
                    None
                }
            }
            TxKind::Target if is_pf_freeze => frame_axis,
            TxKind::Target if !mit_recovery => {
                frame_axis.filter(|&j| j < 32 && (reserved_mask & (1u32 << j)) != 0)
            }
            _ => None,
        };
        // A continuity hold or healthy target covering a due reserved axis consumes one of the
        // reserved slots itself and must never be blocked by other due axes; every other frame
        // kind must leave all `reserved_mask` slots free inside `tx_budget.max_frames`.
        if covered_axis.is_some() {
            return true;
        }
        let reserved = reserved_mask.count_ones() as usize;
        let attempts = self.tick_tx.map_or(0, |p| p.attempts);
        attempts.saturating_add(1).saturating_add(reserved) <= self.tx_budget.max_frames
    }

    /// Remaining TX frame slots in the active tick pass (`0` if backpressured or past deadline).
    #[inline]
    pub(in crate::driver::can_servo) fn remaining_tick_tx_slots(&self) -> usize {
        self.remaining_tick_tx_slots_with_bypass(ContinuityBypass::None)
    }

    #[inline]
    pub(in crate::driver::can_servo) fn remaining_tick_tx_slots_with_bypass(&self, bypass: ContinuityBypass) -> usize {
        let max_frames = self.effective_tx_max_frames_with_bypass(bypass);
        let release_cap = if bypass.bypasses_release_deadline() {
            None
        } else {
            self.tx_release_deadline
        };
        let Some(pass) = self.tick_tx else {
            if release_cap.is_some_and(|cap| self.now() >= cap) {
                return 0;
            }
            return max_frames;
        };
        let effective_deadline = release_cap.map_or(pass.deadline, |cap| pass.deadline.min(cap));
        if pass.backpressured || (!bypass.bypasses_release_deadline() && self.now() >= effective_deadline) {
            return 0;
        }
        max_frames.saturating_sub(pass.attempts)
    }

    /// Record a flushed joint TX frame's execution state (`motion` vs `unchanged hold/freeze`)
    /// and advance the global per-slot progress sequence for fair round-robin ordering.
    #[inline]
    pub(in crate::driver::can_servo) fn record_flushed_joint_tx(&mut self, joint: usize, slot: usize, is_motion: bool) {
        let flat = joint.saturating_mul(2).saturating_add(slot.min(1));
        if flat < self.slot_in_unchanged_hold.len() {
            if is_motion {
                // Any motion frame invalidates both POS_VEL and MIT hold slots for this joint.
                let base = joint.saturating_mul(2);
                self.slot_in_unchanged_hold[base] = false;
                if base + 1 < self.slot_in_unchanged_hold.len() {
                    self.slot_in_unchanged_hold[base + 1] = false;
                }
            } else {
                self.slot_in_unchanged_hold[flat] = true;
            }
            self.tx_progress_seq = self.tx_progress_seq.saturating_add(1);
            self.slot_last_tx_seq[flat] = self.tx_progress_seq;
        }
    }

    /// True when any joint (J1–J6 or J7) still awaits a Class-1 motion->stop/hold/freeze frame.
    #[inline]
    pub(in crate::driver::can_servo) fn has_class1_transition_pending(&self) -> bool {
        if self.last_output_was_hold {
            let n = self.motor_ids.len().min(MAX_MODE_JOINTS);
            if (0..n).any(|i| !self.slot_in_unchanged_hold[i * 2]) {
                return true;
            }
        }
        self.j7_tx_priority(self.last_output_was_hold)
            .is_some_and(|(class, _)| class == TX_CLASS_STOP_TRANSITION)
    }

    #[inline]
    pub(in crate::driver) fn record_tx_pass_attempt(&mut self) {
        let mut pass = self.ensure_tx_pass();
        pass.attempts = pass.attempts.saturating_add(1);
        self.tick_tx = Some(pass);
    }

    #[inline]
    pub(in crate::driver::can_servo) fn mark_tx_pass_backpressured(&mut self) {
        self.tx_probe = None;
        let mut pass = self.ensure_tx_pass();
        pass.backpressured = true;
        self.tick_tx = Some(pass);
    }

    pub(in crate::driver::can_servo) fn discard_pending_tx_stopped(&mut self) {
        let _ = self.discard_pending_tx();
    }

    /// Centralized preflight rejection cleanup: invalidates any carried-over pending frames and
    /// trips an active MIT deadline fence (clearing cached frames) while preserving the caller's
    /// concrete validation error (zero allocation).
    pub(in crate::driver::can_servo) fn reject_target_tx(&mut self, reason: MitFenceTripReason) {
        self.discard_all_pending_tx();
        if self.mit_fence.is_some() || self.mit_fence_trip.is_some() {
            self.trip_fence(reason);
        }
    }

    #[inline]
    pub(in crate::driver::can_servo) fn is_stop_pending(&self) -> bool {
        self.estop_gate.as_ref().is_some_and(|g| g.pending())
    }

    #[inline]
    pub(in crate::driver::can_servo) fn is_stop_latched(&self) -> bool {
        self.estop_state.as_ref().is_some_and(|s| s.is_latched())
            || self.abort_flag.as_ref().is_some_and(|f| f.load(Ordering::SeqCst))
    }

    #[inline]
    pub(in crate::driver::can_servo) fn strict_stop_check(&self, expected_stamp: Option<StopStamp>) -> Result<(), ArmDriverError> {
        if self.is_stop_pending()
            || self.is_stop_latched()
            || expected_stamp.is_some_and(|s| self.current_stop_stamp() != s)
        {
            return Err(ArmDriverError::EstopInhibited);
        }
        Ok(())
    }

    #[inline]
    pub(in crate::driver::can_servo) fn pending_stop_check(&self, expected_stamp: Option<StopStamp>) -> Result<(), ArmDriverError> {
        if self.is_stop_pending()
            || expected_stamp.is_some_and(|s| self.current_stop_stamp() != s)
        {
            return Err(ArmDriverError::EstopInhibited);
        }
        Ok(())
    }

    /// Base mode of joint `i` (what `JointMode::Position` and an enable restore mean).
    pub(in crate::driver::can_servo) fn base_mode(&self, i: usize) -> DamiaoControlMode { self.base_modes.get(i).copied().unwrap_or(self.control_mode) }

    pub fn with_master_id(mut self, master_id: u32) -> Self {
        self.master_id = master_id;
        self.configured_master_id = Some(master_id);
        self
    }

    /// Expected CAN arbitration ID (MST_ID) for replies from joint `i`:
    /// the configured `master_id` when set, otherwise Damiao's per-motor `0x10 + motor_id`
    /// (e.g. J7 motor ID 7 -> 0x17 on B601-DM).
    pub(in crate::driver::can_servo) fn expected_reply_id(&self, i: usize) -> u32 {
        self.configured_master_id.unwrap_or(0x10 + u32::from(self.motor_ids[i]))
    }

    /// Global base mode (POS_VEL or MIT); a position-force gripper keeps its own base mode.
    pub fn with_control_mode(mut self, mode: DamiaoControlMode) -> Self {
        self.control_mode = mode;
        self.joint_modes.fill(Some(mode));
        self.verified_modes.fill(None);
        self.base_modes.fill(mode);
        if let Some(j) = self.pos_force.as_ref().map(|p| p.joint) { self.base_modes[j] = DamiaoControlMode::PosForce; }
        self
    }

    /// One MIT / feedback mapping range for every joint. It must equal the motors'
    /// PMAX / VMAX / TMAX registers; gravity compensation reads them back and refuses
    /// to start otherwise.
    pub fn with_limits(mut self, limits: DamiaoLimits) -> Self {
        self.limits.fill(limits);
        self.mit_limits_verified.fill(false);
        self.range_check = None;
        self
    }

    /// Per-joint mapping ranges, one per motor ID in order.
    pub fn with_joint_limits(mut self, limits: Vec<DamiaoLimits>) -> Result<Self, ArmDriverError> {
        if limits.len() != self.motor_ids.len() {
            return Err(ArmDriverError::JointCountMismatch { expected: self.motor_ids.len(), actual: limits.len() });
        }
        self.limits = limits;
        self.mit_limits_verified.fill(false);
        self.range_check = None;
        Ok(self)
    }

    /// Mapping range used for each joint.
    pub fn joint_limits(&self) -> &[DamiaoLimits] {
        &self.limits
    }

    pub(in crate::driver::can_servo) fn vlim(&self) -> f32 { self.default_vlim.abs().clamp(0.1, 4.0) }

    pub(in crate::driver) fn pos_vel_frame(&self, i: usize, position: f32) -> CanFrame {
        damiao::build_pos_vel_frame(0x100, self.motor_ids[i], position, self.vlim())
    }

    pub(in crate::driver::can_servo) fn record_sent_target(&mut self, i: usize, frame: CanFrame, mit_params: Option<(f32, f32, f32)>) {
        self.last_target_frames[i] = Some(frame);
        self.last_mit_params[i] = if frame.id == u32::from(self.motor_ids[i]) {
            mit_params
        } else {
            None
        };
    }

    pub(in crate::driver::can_servo) fn clear_cached_targets(&mut self) {
        self.last_target_frames.fill(None);
        self.last_mit_params.fill(None);
    }

    /// Fresh, finite measured position of joint `i` with a provable receive-age bound:
    /// both the decode instant and the prior observed-empty receive-queue boundary must
    /// exist and be within `max_age` (samples queued before the first empty queue have
    /// Measured position of joint `i` ONLY when both the receive instant AND the observed-empty
    /// receive-queue bound are within `max_age` and the cached position is finite (unbounded
    /// samples have unknown age and return `None`).
    pub(in crate::driver::can_servo) fn fresh_measured_position_at(&self, i: usize, max_age: Duration, now: Instant) -> Option<f32> {
        let fresh_rx = self.last_feedback_instant.get(i).copied().flatten()
            .is_some_and(|t| now.saturating_duration_since(t) <= max_age);
        let fresh_bound = self.feedback_queue_bound.get(i).copied().flatten()
            .is_some_and(|b| now.saturating_duration_since(b) <= max_age);
        if !(fresh_rx && fresh_bound) { return None; }
        self.cached_states.get(i).map(|s| s.position).filter(|p| p.is_finite())
    }

    pub(in crate::driver) fn fresh_measured_position(&self, i: usize, max_age: Duration) -> Option<f32> {
        self.fresh_measured_position_at(i, max_age, self.now())
    }

    pub(in crate::driver::can_servo) fn now(&self) -> Instant { self.clock.as_ref().map_or_else(Instant::now, |c| c.now()) }
    pub(in crate::driver::can_servo) fn since(&self, t: Instant) -> Duration { self.now().saturating_duration_since(t) }
    pub(in crate::driver::can_servo) fn pause(&self, d: Duration) {
        let track = crate::diagnostics::tick::hold_io_active();
        let t0 = track.then(Instant::now);
        match &self.clock { Some(c) => c.sleep(d), None => std::thread::sleep(d) }
        if let Some(t0) = t0 {
            let wall_ns = t0.elapsed().as_nanos().min(u128::from(u64::MAX)) as u64;
            let d_ns = d.as_nanos().min(u128::from(u64::MAX)) as u64;
            crate::diagnostics::tick::record_hold_sleep_ns(wall_ns.max(d_ns));
        }
    }

    pub fn with_vlim(mut self, vlim: f32) -> Self {
        self.default_vlim = vlim;
        self
    }

    pub fn with_gains(mut self, kp: f32, kd: f32) -> Self {
        self.default_kp = kp;
        self.default_kd = kd;
        self
    }

    pub fn can_mut(&mut self) -> &mut C {
        &mut self.can
    }

    #[doc(hidden)]
    pub fn with_before_final_tx_gate_hook(
        mut self,
        hook: std::sync::Arc<dyn Fn(&CanFrame) + Send + Sync>,
    ) -> Self {
        self.before_final_tx_gate_hook = Some(hook);
        self
    }

}

pub(super) mod arm_driver;
pub(super) mod estop_hold;
pub(super) mod holds;
pub(super) mod recovery;
pub(super) mod tx;
pub(super) mod tx_types;
pub(super) mod target_tx;

pub struct DamiaoArmDriver<C: CanTransport> {
    pub(in crate::driver::can_servo) can: C,
    /// Checked before each frame of a target batch (`ArmDriver::set_estop_gate`).
    pub(in crate::driver::can_servo) estop_gate: Option<crate::rt_core::urgent::EstopGate>,
    /// Client lease generation stamped onto queued motion target frames (`0` for hold/autonomous).
    pub(in crate::driver::can_servo) output_lease_gen: u64,
    /// Per-call client lease generation checked inside [`Self::send`] (set by `send_with_lease`).
    pub(in crate::driver::can_servo) send_lease_gen: u64,
    /// Atomic E-stop latch (`ArmDriver::set_abort_flag`), checked before enable transmissions.
    pub(in crate::driver::can_servo) abort_flag: Option<Arc<AtomicBool>>,
    /// Supervisor-injected stopped-disable gate enforced at send time for `DisableRequested`.
    pub(in crate::driver::can_servo) stopped_disable_gate: Option<StoppedDisableGate>,
    /// Authoritative packed E-stop state (`ArmDriver::set_estop_state`).
    pub(in crate::driver::can_servo) estop_state: Option<Arc<crate::rt_core::urgent::EstopState>>,
    /// Strict non-sleeping RT mode (`ArmDriver::set_rt_mode`), disables retry sleeps on WouldBlock.
    pub(in crate::driver::can_servo) rt_no_sleep: bool,
    pub(in crate::driver) motor_ids: Vec<u16>,
    pub(in crate::driver::can_servo) master_id: u32,
    /// Explicit master CAN ID from [`Self::with_master_id`]; `None` uses Damiao's per-motor `0x10 + motor_id`.
    pub(in crate::driver::can_servo) configured_master_id: Option<u32>,
    /// MIT / feedback mapping range per joint (must equal each motor's PMAX / VMAX / TMAX).
    pub(in crate::driver::can_servo) limits: Vec<DamiaoLimits>,
    pub(in crate::driver::can_servo) control_mode: DamiaoControlMode,
    pub(in crate::driver::can_servo) default_vlim: f32,
    pub(in crate::driver::can_servo) default_kp: f32,
    pub(in crate::driver::can_servo) default_kd: f32,
    pub(in crate::driver) cached_states: Vec<JointState>,
    pub(in crate::driver::can_servo) last_target_frames: Vec<Option<CanFrame>>,
    /// Last MIT `(kp, kd, torque)` parameters transmitted per joint (used to rebuild fresh
    /// gravity-compensation recovery frames from live measured position without replaying bytes).
    pub(in crate::driver::can_servo) last_mit_params: Vec<Option<(f32, f32, f32)>>,
    pub(in crate::driver) last_feedback_instant: Vec<Option<Instant>>,
    /// Most recent decoded feedback instant per joint, preserved across `send_enable()` (which
    /// clears `last_feedback_instant` to gate post-enable state-1 confirmation) and cleared on
    /// `awaiting_rx_boundary` reset so Rule A′ send-time pose checks during enable rollback still
    /// observe provably fresh (`<= 100 ms`) pose telemetry.
    pub(in crate::driver) pose_feedback_instant: Vec<Option<Instant>>,
    pub(in crate::driver::can_servo) feedback_seq: Vec<u64>,
    /// Instant taken before the last receive call that found the RX queue empty.
    pub(in crate::driver::can_servo) rx_empty_instant: Option<Instant>,
    /// `rx_empty_instant` when each joint's last sample was decoded.
    pub(in crate::driver) feedback_queue_bound: Vec<Option<Instant>>,
    pub(in crate::driver::can_servo) motor_status: Vec<u8>,
    pub(in crate::driver) is_enabled: bool,
    pub(in crate::driver::can_servo) pending_rearm: bool,
    pub(in crate::driver::can_servo) last_query_instant: Option<Instant>,
    pub(in crate::driver::can_servo) last_enable_command: Option<Instant>,
    /// `rx_seq` snapshot when `last_enable_command` was recorded so pre-command feedback cannot
    /// satisfy `enable_state()` even if a virtual clock does not advance between receive and send.
    pub(in crate::driver::can_servo) last_enable_command_rx_seq: Option<u64>,
    /// True when `last_enable_command` was recorded by a disable command (as opposed to enable).
    pub(in crate::driver::can_servo) last_command_was_disable: bool,
    pub(in crate::driver::can_servo) awaiting_rx_boundary: bool,
    pub(in crate::driver::can_servo) is_enabling: bool,
    pub(in crate::driver::can_servo) last_warn_sec: AtomicU64,
    /// Confirmed control mode per joint; None = unconfirmed after a failed switch.
    pub(in crate::driver) joint_modes: Vec<Option<DamiaoControlMode>>,
    /// Per-joint control mode verified from a causal `RID_CTRL_MODE` readback (or confirmed
    /// `RID_CTRL_MODE` write reply) after this daemon start (E4). Starts `None` for every joint;
    /// Damiao holds are sent ONLY in a verified mode, never assumed `PosVel`.
    pub(in crate::driver) verified_modes: Vec<Option<DamiaoControlMode>>,
    /// Open RX sequence boundary for on-tick `RID_CTRL_MODE` readback requests per joint (L2).
    /// Only causal post-boundary, post-settle `REG_READ(RID_CTRL_MODE)` replies advance
    /// `verified_modes[i]`.
    pub(in crate::driver::can_servo) mode_readback_boundary: Vec<Option<u64>>,
    /// Bitmask of joints with an unconfirmed mode-changing `REG_WRITE(RID_CTRL_MODE)` that may
    /// have reached the wire (including failed flush). Cleared only on causal confirmation.
    pub(in crate::driver::can_servo) mode_write_uncertain_mask: u32,
    /// Bitmask of joints currently owned by an active `ModeSwitch` transaction (suppresses
    /// legacy `mode_readback_boundary` auto-verification in `process_rx_frame`).
    pub(in crate::driver::can_servo) mode_switch_active_mask: u32,
    /// Per-joint hold targets used by `mode_switch_service_watchdog_holds` during an active
    /// `ModeSwitch` transaction (preserves exact MIT `kp`/`kd`/`torque` hold parameters, 0 alloc).
    pub(in crate::driver::can_servo) mode_switch_hold_targets: [Option<ModeSwitchHoldTarget>; MAX_MODE_JOINTS],
    /// RX sequence boundary captured at the start of an active budgeted MIT expiry recovery
    /// (`mit_expiry_recover_budgeted`), cleared on completion or reset.
    pub(in crate::driver::can_servo) mit_expiry_budgeted_boundary: Option<u64>,
    /// True while either legacy (`mit_expiry_recover_with_gate`) or executor
    /// (`mit_expiry_recover_budgeted`, including initial drain) MIT expiry recovery is active,
    /// keeping provisional `state == 0` feedback from clearing `motor_energized_mask` or
    /// `motor_last_ctrl_tx` before full two-round confirmation and enabling per-frame causal
    /// probe/hold reply matching on `mit_expiry_axes`.
    pub(in crate::driver::can_servo) mit_expiry_recovery_active: bool,
    /// Bitmask of joints that have had `CMD_CLEAR_FAULT` sent during budgeted MIT expiry recovery.
    pub(in crate::driver::can_servo) mit_expiry_cleared_mask: u64,
    /// Per-joint sequential causal disable confirmation state shared by both legacy and executor
    /// MIT expiry recovery: `CMD_DISABLE` -> `REG_READ(RID_CTRL_MODE)` barrier -> Round 1 `0xCC`
    /// (`state == 0`) -> Round 2 `0xCC` (`state == 0`), draining RX right before each axis's
    /// probe submission (0 alloc).
    pub(in crate::driver::can_servo) mit_expiry_axes: [MitExpiryAxisState; MAX_MODE_JOINTS],
    /// Most recent `rx_seq` at which each joint reported an adverse (`state_or_error != 0`)
    /// feedback frame (recorded before timestamp filtering so untimestamped adverse frames also
    /// veto disable confirmation).
    pub(in crate::driver::can_servo) last_adverse_feedback_seq: [u64; MAX_MODE_JOINTS],
    /// Per-release RX frame counter shared across Step 2 RX, transition polls, ModeSwitch, MIT
    /// recovery, and commissioning (one release ledger).
    pub(in crate::driver::can_servo) release_rx_frames: u16,
    /// Per-release RX frame cap tightened by the owner's [`crate::rt_core::budget::ReleaseBudget`] (default 64).
    pub(in crate::driver::can_servo) release_rx_limit: u16,
    /// Optional per-release budgeted TX cap tightened by the owner's [`crate::rt_core::budget::ReleaseBudget`].
    pub(in crate::driver::can_servo) release_tx_limit: Option<u16>,
    /// Bitmask of axes causally confirmed disabled during the active MIT expiry recovery attempt
    /// so `watchdog_continuity_reserved_mask` and `service_mit_recovery_continuity_holds_inner`
    /// share the exact same due-axis eligibility predicate.
    pub(in crate::driver::can_servo) mit_expiry_confirmed_disabled_mask: u64,
    /// Per-joint `CLOCK_REALTIME - monotonic` decode offset (ns) of the latest causal feedback.
    pub(in crate::driver::can_servo) last_feedback_decode_offset_ns: Vec<Option<i128>>,
    /// Per-joint causal `REG_READ(RID_CTRL_MODE)` probe boundary
    /// `(after_seq, tx_at, retry_at, deadline, tx_offset_ns)` used by SAFE-HOLD / stopped recovery
    /// for joints in `mode_write_uncertain_mask` (0 alloc).
    pub(in crate::driver::can_servo) uncertain_readback_probe: [Option<(u64, Instant, Instant, Instant, Option<i128>)>; MAX_MODE_JOINTS],
    /// PMAX / VMAX / TMAX read back from the motor and equal to `limits[i]`.
    pub(in crate::driver::can_servo) mit_limits_verified: Vec<bool>,
    /// Opt-in expected Damiao `RID_TIMEOUT` counts per joint (`None` = any fresh reply accepted).
    pub(in crate::driver::can_servo) expected_timeout_counts: Vec<Option<u32>>,
    /// True when pre-enable `RID_TIMEOUT` readback verification is enabled (via
    /// [`Self::with_timeout_verification`] or [`Self::with_expected_timeout_counts`]).
    pub(in crate::driver::can_servo) timeout_check_enabled: bool,
    /// Per-joint causal `RID_TIMEOUT` verification state (cleared on startup and after BusLoss).
    pub(in crate::driver::can_servo) timeout_verified: Vec<bool>,
    /// True when startup or post-BusLoss register verification is pending: next enable must run
    /// fresh causal `CTRL_MODE` and `TIMEOUT` readbacks across all motors and only clears upon
    /// confirmed enable ([`Self::finish_enable_confirmed`]).
    pub(in crate::driver::can_servo) startup_register_check_pending: bool,
    /// Bitmask of joints whose `verified_modes[j]` transitioned from unverified to verified during
    /// the current enable attempt so an aborted startup/post-BusLoss attempt discards them.
    pub(in crate::driver::can_servo) attempt_verified_modes_mask: u32,
    /// Staged quantized wire target positions `(tx_at, motor_space_rad)` awaiting `can.flush()`.
    pub(in crate::driver::can_servo) staged_wire_pos: [Option<(Instant, f32)>; MAX_MODE_JOINTS],
    pub(in crate::driver::can_servo) staged_wire_pos_mask: u32,
    /// Last wire-committed target position per joint as `(commit_seq, tx_at, motor_space_rad)`,
    /// recorded strictly at confirmed transmit/flush for non-skipped PosVel/MIT control frames.
    pub(in crate::driver::can_servo) committed_wire_pos: [Option<(u64, Instant, f32)>; MAX_MODE_JOINTS],
    pub(in crate::driver::can_servo) wire_pos_commit_seq: u64,
    /// Register replies seen by `read_joint_states` since the last clear.
    pub(in crate::driver::can_servo) register_replies: Vec<damiao::RegisterReply>,
    /// Last mapping-range check (`check_mapping_ranges`); None = not run.
    pub(in crate::driver::can_servo) range_check: Option<DriverRangeCheck>,
    /// Joints that received an autonomous recovery write during `read_joint_states`
    /// since the supervisor last queried `take_recovery_writes`.
    pub(in crate::driver::can_servo) recovery_written_joints: Vec<usize>,
    /// `ArmDriver::set_strict_mode_handover`; false keeps the e4f72b6 `write_modes`.
    pub(in crate::driver::can_servo) strict_mode_handover: bool,
    /// Register replies with the CAN ID they arrived on and their receive instant (MIT
    /// identity readback and TIMEOUT commissioning).
    pub(in crate::driver::can_servo) register_reply_log: Vec<(damiao::RegisterReply, u32, Instant, u64, u64)>,
    /// Preallocated inline register reply buffer for RT commissioning (zero heap alloc/free).
    pub(in crate::driver::can_servo) comm_reg_replies: [Option<crate::mit::CommissioningRegReply>; 64],
    pub(in crate::driver::can_servo) comm_reg_reply_len: usize,
    /// True if more than 64 register replies arrived since the last `commissioning_clear_register_log`;
    /// forces `commissioning_find_register_reply` to fail closed as `Ambiguous`.
    pub(in crate::driver::can_servo) comm_reg_reply_overflow: bool,
    /// Test-only hook invoked strictly AFTER section-6 pre-TX bookkeeping and immediately BEFORE
    /// the final authority/deadline check and `can.transmit(&frame)`.
    pub(in crate::driver::can_servo) before_final_tx_gate_hook: Option<std::sync::Arc<dyn Fn(&CanFrame) + Send + Sync>>,
    /// RX sequence number of the last observed-empty receive queue: every frame dequeued after
    /// it (sequence number larger) was enqueued after that observation.
    pub(in crate::driver::can_servo) rx_empty_seq: u64,
    /// The last `read_joint_states` observed an empty receive queue.
    pub(in crate::driver::can_servo) last_read_drained: bool,
    /// Deterministic clock of the CAN transport (tests); None = the system clock.
    pub(in crate::driver::can_servo) clock: Option<std::sync::Arc<dyn crate::transport::DriverClock>>,
    /// Sequence number of the last processed RX frame, and of each joint's last feedback.
    pub(in crate::driver::can_servo) rx_seq: u64,
    /// Exact `(rx_seq, now)` mark captured immediately before `can.transmit(&frame)` on the most
    /// recent successful `send_inner` call (with zero intervening transport callbacks or hooks).
    pub(in crate::driver::can_servo) last_tx_mark: Option<(u64, Instant)>,
    pub(in crate::driver::can_servo) last_feedback_seq: Vec<u64>,
    /// Incremented whenever joint `i`'s feedback continuity breaks (`> 100 ms` gap or non-finite).
    pub(in crate::driver::can_servo) feedback_continuity_epoch: Vec<u64>,
    /// Per-motor causal boundary captured after the last motion/enable write returns from
    /// `can.transmit()`. Used ONLY inside the rest-pose disable gate.
    pub(in crate::driver::can_servo) rest_pose_boundary: Vec<Option<RestPoseBoundary>>,
    /// Active control-release id distinguishing separate ticks for the rest-pose K=3 window.
    pub(in crate::driver::can_servo) control_release_id: u64,
    /// True once the owner has called `begin_control_release`, preventing receive passes from
    /// advancing `control_release_id` independently.
    pub(in crate::driver::can_servo) control_release_managed: bool,
    /// Per-motor K=3 causal confirmation window for rest pose (the ONLY strict evidence gate).
    pub(in crate::driver) rest_pose_window: Vec<CausalWindow>,
    /// `rx_seq` snapshot when `CMD_ENABLE` was transmitted for motor `i`.
    pub(in crate::driver::can_servo) motor_enable_seq: Vec<u64>,
    /// Per-axis `(rx_seq, tx_started)` boundary captured at the most recent `CMD_ENABLE`
    /// submission so ordinary disarm confirmation requires a fresh post-enable `state == 0` reply.
    pub(in crate::driver::can_servo) motor_last_enable_tx: Vec<Option<(u64, Instant)>>,
    /// True once motor `i` has produced a causal `state == 0` feedback frame after its latest
    /// `CMD_ENABLE` submission (cleared on any `CMD_ENABLE` or non-zero `state_or_error`).
    pub(in crate::driver::can_servo) motor_disarmed_after_enable: Vec<bool>,
    /// True once motor `i` has received feedback strictly after its latest `CMD_ENABLE`
    /// transmit (any state, including fault states, so `poll_enable` detects faults causally).
    pub(in crate::driver::can_servo) motor_post_enable_feedback: Vec<bool>,
    /// Monotonic counter bumped strictly when a post-enable motor feedback frame carries
    /// `state_or_error != 1` (R12 M2).
    pub(in crate::driver::can_servo) reported_torque_loss_sample_seq: u64,
    /// True while a `ProcessExit` panic-disable sequence is active and awaiting causal per-axis
    /// `state == 0` feedback replies after `CMD_DISABLE` transmission.
    pub(in crate::driver::can_servo) panic_disable_active: bool,
    /// Per-axis `(rx_seq, tx_started)` boundary captured when a flushed `CMD_DISABLE` was sent
    /// after an empty RX boundary during `panic_disable_step`.
    pub(in crate::driver::can_servo) panic_disable_tx_boundary: Vec<Option<(u64, Instant)>>,
    /// Per-axis causal confirmation that a fresh `state == 0` reply arrived after the axis's
    /// flushed `panic_disable_step` `CMD_DISABLE` transmission.
    pub(in crate::driver::can_servo) panic_disable_confirmed: Vec<bool>,
    /// True while `probe_hardware_disarmed_until` is actively polling read-only `0xCC` status.
    pub(in crate::driver::can_servo) disarm_probe_active: bool,
    /// Per-axis bitmask of axes that produced a non-disabled (`state != 0`) post-boundary feedback
    /// frame during an active `probe_hardware_disarmed_until` pass so a later `state == 0` frame
    /// in the same drain can never mask or cache proof for any adverse axis.
    pub(in crate::driver::can_servo) disarm_probe_adverse_mask: u32,
    /// First non-disabled (`state != 0`) state recorded per axis during an active
    /// `probe_hardware_disarmed_until` pass.
    pub(in crate::driver::can_servo) disarm_probe_adverse_state: [u8; MAX_MODE_JOINTS],
    /// MIT tracking deadline fence (`set_mit_deadline_fence`); None = off (unchanged behaviour).
    pub(in crate::driver::can_servo) mit_fence: Option<Duration>,
    /// Per joint: when its last confirmed command started out (the earliest frame of it, never
    /// its completion), while the fence is armed. The output gap is measured from the oldest.
    pub(in crate::driver::can_servo) mit_fence_last_cmd: Vec<Option<Instant>>,
    /// Trip reason and instant; refuses writes until a confirmed enable (zero allocation).
    pub(in crate::driver::can_servo) mit_fence_trip: Option<(MitFenceTripReason, Instant)>,
    /// No autonomous frame (recovery, status poll) this long after a trip.
    pub(in crate::driver::can_servo) mit_fence_holdoff: Duration,
    /// The operator cleared the trip (`acknowledge_mit_fence_trip`): polls may resume.
    pub(in crate::driver::can_servo) mit_fence_acked: bool,
    /// True while supervisor MIT ownership is `Lost`: `verified_hold_mode` rejects MIT frames
    /// so MIT-configured joints fall back to verified `PosVel` holds or stay unheld under L2.
    pub(in crate::driver::can_servo) mit_ownership_lost: bool,
    /// RX sequence boundary captured at the start of the most recent [`ArmDriver::mit_expiry_recover`]
    /// request; final MIT clear requires every joint's `last_feedback_seq` to exceed this boundary.
    pub(in crate::driver::can_servo) last_mit_recovery_rx_seq: Option<u64>,
    /// MIT tracking armed a fence since the last confirmed enable: base-mode restores use POS_VEL
    /// frames only.
    pub(in crate::driver::can_servo) mit_tracking_used: bool,
    /// Pending torque transition (`begin_torque` / `poll_torque`); None = idle.
    pub(in crate::driver::can_servo) torque_hs: Option<DamiaoTorque>,
    /// Bitmask of 0-based motor indices that have already received `CMD_DISABLE` in an active
    /// or partial disable sequence (R12 B3). Cleared on full disable completion or enable.
    pub(in crate::driver::can_servo) disable_dispatched_mask: u64,
    /// True once K=3 causal rest-pose evidence on J1–J6 has been confirmed BEFORE the first
    /// `CMD_DISABLE` frame of the active `DisableRequested` sequence.
    pub(in crate::driver::can_servo) disable_rest_established: bool,
    /// Optional absolute driver-clock authority deadline for dispatching undispatched motors
    /// during a multi-tick `DisableRequested` sequence.
    pub(in crate::driver::can_servo) disable_tx_deadline: Option<Instant>,
    /// Optional admitted client lease generation checked before dispatching undispatched motors
    /// during a multi-tick `DisableRequested` sequence.
    pub(in crate::driver::can_servo) disable_lease_gen: Option<u64>,
    /// Service deadline capping the receive pass of the current `poll_torque` (None: 5 ms only).
    pub(in crate::driver::can_servo) rx_deadline: Option<Instant>,
    /// True while inside `poll_torque_inner` so its internal `receive_feedback` calls do not
    /// reset the single TX budget pass opened at `poll_torque_inner` entry.
    pub(in crate::driver::can_servo) in_poll_torque: bool,
    /// Frame limit capping the receive pass of the current budgeted read/poll (None: 64 frames).
    pub(in crate::driver::can_servo) rx_frame_limit: Option<u16>,
    /// `(frames_processed, budget_exhausted)` of the most recent `receive_feedback` pass.
    pub(in crate::driver::can_servo) last_poll_rx: Option<(u16, bool)>,
    /// Base (position-control) mode per joint: `control_mode`, or `PosForce` for the gripper.
    pub(in crate::driver::can_servo) base_modes: Vec<DamiaoControlMode>,
    /// Gripper in position-force mode (None: every joint uses `control_mode`, legacy).
    pub(in crate::driver::can_servo) pos_force: Option<GripperPosForce>,
    /// The gripper's CTRL_MODE 4 was read back at the last full enable.
    pub(in crate::driver) pos_force_verified: bool,
    /// Legacy profile: gripper joint driven in POS_VEL (reported without a motor current cap).
    pub(in crate::driver::can_servo) legacy_gripper: Option<usize>,
    /// First timely post-boundary 0xCC refresh sample captured while a refresh is active.
    pub(in crate::driver::can_servo) refresh_capture: Option<RefreshCapture>,
    /// Per-tick bounded TX budget (attempts, start-cutoff duration, inclusive frame expiry).
    pub(in crate::driver::can_servo) tx_budget: TxBudget,
    /// Inline fixed-capacity queue of unsent motion frames carried to the next tick (zero alloc).
    pub(in crate::driver::can_servo) pending_tx: PendingTxQueue,
    /// Inline fixed-capacity queue of unsent stopped-hold frames carried across ticks (separate
    /// from `pending_tx` so `strict_stop_check` while latched in E-stop never wipes hold carryover).
    pub(in crate::driver::can_servo) pending_hold_tx: PendingTxQueue,
    /// Round-robin start joint for stopped-hold batches so slow-cadence partial passes never starve
    /// higher-index joints.
    pub(in crate::driver::can_servo) hold_cursor: usize,
    /// True if the most recent `write_estop_hold_bounded` call transmitted at least one frame.
    pub(in crate::driver::can_servo) last_hold_tx_sent_any: bool,
    /// Bitmask of 0-based joint indices whose motion target transmitted & flushed in the most
    /// recent `write_joint_targets_bounded` / `drain_pending_tx` pass with no frame still queued.
    pub(in crate::driver::can_servo) last_target_tx_sent_mask: u64,
    /// Observed age of the oldest frame expired in the most recent bounded TX pass.
    pub(in crate::driver::can_servo) last_expired_oldest_age: Option<Duration>,
    /// Shared TX pass state across `receive_feedback()`, J7 handshakes, and `write_joint_targets*()` in one tick.
    pub(in crate::driver::can_servo) tick_tx: Option<TickTxPass>,
    /// Unified owner-local `0xCC` refresh scheduler (fixed `u64` bitmask + round-robin cursor,
    /// at most one `0xCC` slot per tick across enable, FailedHold, Idle-torqued, and stopped).
    pub(in crate::driver::can_servo) refresh_scheduler: RefreshScheduler,
    /// Stop/E-stop generation stamp captured when an enable handshake starts so an applied stop
    /// (`requested == applied`) mid-handshake also inhibits subsequent enable frames.
    pub(in crate::driver::can_servo) enable_stop_stamp: Option<StopStamp>,
    /// Most recent provably fresh (`queue_bound.is_some()` + finite position) feedback
    /// `(rx_at, queue_bound)` decoded for the position-force gripper. Unlike
    /// `last_feedback_instant` (cleared by `send_enable` to enforce post-enable state-1 replies),
    /// this preserves live J7 telemetry age across carried and retried `CMD_ENABLE` frames.
    pub(in crate::driver::can_servo) pf_last_fresh_feedback: Option<(Instant, Instant)>,
    /// Last provably fresh (`<= 100 ms`, bounded, finite) measured J7 position decoded for the
    /// position-force gripper; preserved across staleness so J7 freeze frames always hold the
    /// last known fresh position with `vel_raw = 0` and the configured current cap.
    pub(in crate::driver) pf_last_fresh_position: Option<f32>,
    /// Optional absolute driver-clock ceiling on the active control release's TX start-cutoff
    /// (`release_deadline - publish_margin`), set during a release and cleared afterwards.
    pub(in crate::driver::can_servo) tx_release_deadline: Option<Instant>,
    /// Optional absolute driver-clock authority ceiling (`min(lease_expiry, request_deadline)`)
    /// checked immediately before transmitting every `CMD_ENABLE` frame.
    pub(in crate::driver::can_servo) enable_tx_deadline: Option<Instant>,
    /// Optional admitted client lease generation checked against the atomic lease fence
    /// immediately before transmitting every `CMD_ENABLE` frame.
    pub(in crate::driver::can_servo) enable_lease_gen: Option<u64>,
    /// True once at least one `CMD_ENABLE` frame has been transmitted in the current enable attempt.
    pub(in crate::driver::can_servo) enable_frames_sent_any: bool,
    /// Active/confirmed CAN transmit probe recorded for clearing `CAN_TX_STALLED` (zero alloc).
    pub(in crate::driver::can_servo) tx_probe: Option<TxProbeState>,
    /// Attempt-scoped J7 position-force freshness status observed in the most recent bounded TX pass.
    pub(in crate::driver::can_servo) last_tx_pf_status: PosForceTxStatus,
    /// Monotonic generation incremented each time a provably fresh (`<= 100 ms`, bounded, finite)
    /// J7 position-force feedback sample is decoded. Binds owner-side staleness episodes to the
    /// actual feedback sample so a fresh RX sample between two stale output passes never carries
    /// over an older stale timer.
    pub(in crate::driver::can_servo) pf_fresh_gen: u64,
    /// True when the active `PosForce` joint needs a read-only `0xCC` refresh.
    pub(in crate::driver::can_servo) pf_refresh_pending: bool,
    /// Consecutive ticks in which a needed J7 `0xCC` refresh was deferred by TX budget/backpressure;
    /// when `>= 1`, the next tick prioritizes J7's `0xCC` ahead of competing frames so refresh never starves.
    pub(in crate::driver::can_servo) pf_refresh_deferred_ticks: u8,
    /// True when an output pass ran since the previous `receive_feedback()` pass.
    pub(in crate::driver::can_servo) output_ran_since_rx: bool,
    /// True when the most recent bounded output pass was a stopped hold (`write_estop_hold_bounded`),
    /// so `receive_feedback()` yields the shared TX budget to due J1–J6 stopped holds instead of
    /// preempting them with a J7 `0xCC` refresh.
    pub(in crate::driver::can_servo) last_output_was_hold: bool,
    /// Bitmask of `(joint, slot)` stopped-hold frames (`1u64 << (joint * 2 + slot)`) that were
    /// due in the most recent `write_estop_hold_bounded` pass (`0` when nothing was due or
    /// stop-inhibited).
    pub(in crate::driver::can_servo) last_hold_tx_due_mask: u64,
    /// Bitmask of `(joint, slot)` stopped-hold frames (`1u64 << (joint * 2 + slot)`) that
    /// actually reached the transport (transmitted and flushed) in the most recent
    /// `write_estop_hold_bounded` pass.
    pub(in crate::driver::can_servo) last_hold_tx_sent_mask: u64,
    /// When `true` (installed by `SupervisorInner` on both Legacy and RT Executor), control-tick
    /// arm write paths and `receive_feedback` NEVER write `0x307` frames; `SupervisorInner`'s
    /// tick epilogue calls [`ArmDriver::emit_pos_force_j7`] once per tick.
    pub(in crate::driver::can_servo) external_pos_force_emitter: bool,
    /// Consecutive ticks in which the J7 `PosForce` emitter was deferred by TX budget or
    /// backpressure; when `>= 1`, arm writes reserve a TX budget slot for J7 so J7 never starves
    /// under tight per-tick frame budgets (e.g. 6 frames for 7 motors).
    pub(in crate::driver::can_servo) pf_deferred_ticks: u8,
    /// True when J7's motion target was deferred and awaits transmission on the next tick.
    pub(in crate::driver::can_servo) pf_deferred_motion: bool,
    /// Driver-clock timestamp when J7's current deferral streak began (used for `oldest_pending_tx_age`
    /// only while J7 feedback is fresh; stale J7 never counts as FIFO-expired).
    pub(in crate::driver::can_servo) pf_deferred_since: Option<Instant>,
    /// Last deferred J7 intent (used by direct-driver `drain_pending_tx`).
    pub(in crate::driver::can_servo) pf_deferred_intent: Option<PosForceIntent>,
    /// Per-`(joint, slot)` unchanged-hold state (`true` once a stop/hold/freeze frame has been
    /// flushed for that slot and no motion frame has superseded it; `false` after enable or any
    /// flushed motion frame). Drives TX Class 1 (motion/non-hold -> stop / hold / freeze) vs
    /// Class 3 (re-send of an unchanged hold) across ALL joints including J7.
    pub(in crate::driver::can_servo) slot_in_unchanged_hold: [bool; MAX_MODE_JOINTS * 2],
    /// Monotonic counter incremented on every flushed joint TX frame (J1–J6 slots and J7).
    pub(in crate::driver::can_servo) tx_progress_seq: u64,
    /// Per-`(joint, slot)` sequence number of the last flushed TX frame (`joint * 2 + slot`;
    /// smallest = waited longest = oldest pending first round-robin within each priority class
    /// across J1–J6 POS_VEL/MIT slots and J7).
    pub(in crate::driver::can_servo) slot_last_tx_seq: [u64; MAX_MODE_JOINTS * 2],
    /// Hint set by `SupervisorInner` before arm output indicating whether this tick's upcoming
    /// J7 intent is `Hold`/`Stop` (rather than a `Target`). Cleared after `pos_force_emitter::emit_tick`.
    pub(in crate::driver::can_servo) pf_next_intent_is_hold: bool,
    /// Latched while `pf_last_fresh_position == None` under `NoOutputUntilFresh`: forces the
    /// first `0x307` emission after the first provably fresh J7 sample arrives to be a stationary
    /// `Hold` at that measured position (`vel_raw = 0`, configured `current_raw`).
    pub(in crate::driver::can_servo) pf_hold_on_first_fresh: bool,
    /// Inline fixed-capacity shadow observation staging (zero alloc, zero lock).
    #[cfg(not(target_arch = "wasm32"))]
    pub(in crate::driver::can_servo) shadow_staging: crate::torque::shadow::ShadowDriverStaging,
    #[cfg(not(target_arch = "wasm32"))]
    pub(in crate::driver::can_servo) defer_disable_shadow: bool,
    #[cfg(not(target_arch = "wasm32"))]
    pub(in crate::driver::can_servo) shadow_disable_pass_sent_mask: u64,
    #[cfg(not(target_arch = "wasm32"))]
    pub(in crate::driver::can_servo) pending_disable_shadow: Option<PendingDisableShadowBatch>,
    /// Bitmask of joints (`1u32 << idx`) that have received a `CMD_ENABLE` frame (or reported
    /// `state == 1`) and have not subsequently received `CMD_DISABLE` or causal post-enable
    /// `state == 0` confirmation. Used to guarantee Damiao `TIMEOUT` watchdog hold continuity
    /// (`<= 40 ms` between control frames) across multi-tick handshakes and blocking loops.
    pub(in crate::driver::can_servo) motor_energized_mask: u32,
    /// Driver-clock timestamp of the most recent watchdog-refreshing control frame (`POS_VEL`,
    /// `MIT`, `POS_FORCE`) or `CMD_ENABLE` successfully transmitted to joint `idx`.
    pub(in crate::driver::can_servo) motor_last_ctrl_tx: [Option<Instant>; MAX_MODE_JOINTS],
    /// Last finite measured position observed for joint `idx` (never cleared by transient
    /// staleness) so energized non-`PosForce` arm joints always have a stationary hold target.
    pub(in crate::driver::can_servo) motor_keepalive_hold_pos: [Option<f32>; MAX_MODE_JOINTS],
    /// Bitmask of joints (`1u32 << idx`) that already received a confirmed watchdog-refreshing
    /// control frame (`POS_VEL`, `MIT`, `POS_FORCE`, or initial `CMD_ENABLE`) during the current
    /// control release (`control_release_id`). Prevents duplicate control/hold frames per release.
    pub(in crate::driver::can_servo) release_ctrl_tx_mask: u32,
    /// Staged watchdog-refreshing control TX timestamps awaiting confirmed `can.flush()` on
    /// real-flush (`!flush_is_noop()`) transports. Committed atomically on flush success and
    /// discarded on any pre-flush error, cutoff, stop, or flush failure.
    pub(in crate::driver::can_servo) staged_ctrl_tx: [Option<Instant>; MAX_MODE_JOINTS],
    /// Bitmask of joints (`1u32 << idx`) present in `staged_ctrl_tx`.
    pub(in crate::driver::can_servo) staged_ctrl_tx_mask: u32,
    /// True from `begin_control_release` through Window 1 Step 5 while a subsequent normal/hold
    /// output step in THIS release still owns watchdog servicing. Cleared on Window 2 wakes,
    /// end-of-release fallback, and release-cap teardown so off-tick and Window 2 confirmations
    /// never skip inline watchdog holds.
    pub(in crate::driver::can_servo) output_step_pending_in_release: bool,
    /// `control_release_id` for which the current per-release TX pass (`tick_tx`,
    /// `refresh_scheduler`, `release_ctrl_tx_mask`, `release_recovery_tx_count`) was opened.
    pub(in crate::driver::can_servo) release_tx_pass_release_id: u64,
    /// Autonomous stale-feedback recovery TX frames emitted during the current control release.
    pub(in crate::driver::can_servo) release_recovery_tx_count: u8,
    /// Round-robin cursor across joints for bounded per-release autonomous recovery TX.
    pub(in crate::driver::can_servo) recovery_cursor: usize,
    /// Diagnostic tag distinguishing urgent watchdog continuity holds (`TxFrameKind::UrgentHold`)
    /// from normal stopped / SAFE-HOLD / FailedHold holds (`TxFrameKind::Hold`).
    pub(in crate::driver::can_servo) in_urgent_watchdog_hold: bool,
}
