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
