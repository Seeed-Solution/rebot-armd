//! Damiao `ArmDriver` and `ShadowExt` trait implementations.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::driver::*;
use crate::driver::can_servo::*;
use crate::model::JointState;
use crate::protocol::damiao;
use crate::transport::{CanTransport, TransportError};

impl<C: CanTransport> ArmDriver for DamiaoArmDriver<C> {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            velocity_feedback: true,
            torque_feedback: true,
            temperature_feedback: true,
            torque_feedforward: self.control_mode == DamiaoControlMode::Mit,
            enable_feedback: true,
            load_feedback: false,
        }
    }

    /// Reported enable state from fresh motor feedback (telemetry / UI only; never a safety decision input).
    fn enable_state(&self) -> EnableState {
        if self.motor_ids.is_empty() || self.awaiting_rx_boundary || !self.is_feedback_valid() {
            return EnableState::Unknown;
        }
        if self.motor_status.iter().all(|&s| s == 0) {
            EnableState::Disabled
        } else if self.motor_status.iter().all(|&s| s == 1) {
            EnableState::Enabled
        } else {
            EnableState::Unknown
        }
    }

    /// Physical disarm proof for Damiao motors: requires fresh `state == 0` feedback on EVERY axis
    /// with causal per-axis evidence received AFTER the axis's latest `CMD_ENABLE` submission (and
    /// after its flushed `CMD_DISABLE` submission whenever `panic_disable_step` is active). Never
    /// trusts stale pre-enable cached `motor_status == 0`.
    fn is_hardware_disarmed_confirmed(&self) -> bool {
        !self.motor_ids.is_empty()
            && !self.is_enabled
            && !self.is_enabling
            && !self.torque_transition_pending()
            && self.disable_dispatched_mask == 0
            && !self.awaiting_rx_boundary
            && self.is_feedback_valid()
            && self.enable_state() == EnableState::Disabled
            && !self.has_any_joint_enabled()
            && !self.has_known_partial_enable()
            && (0..self.motor_ids.len()).all(|i| {
                let causal_after_enable = self
                    .motor_disarmed_after_enable
                    .get(i)
                    .copied()
                    .unwrap_or(false)
                    && match self.motor_last_enable_tx.get(i).copied().flatten() {
                        Some((en_seq, en_at)) => {
                            self.last_feedback_seq.get(i).copied().unwrap_or(0) > en_seq
                                && self
                                    .last_feedback_instant
                                    .get(i)
                                    .copied()
                                    .flatten()
                                    .is_some_and(|t| t >= en_at)
                        }
                        None => true,
                    };
                let causal_after_panic_disable = !self.panic_disable_active
                    || self
                        .panic_disable_confirmed
                        .get(i)
                        .copied()
                        .unwrap_or(false);
                causal_after_enable && causal_after_panic_disable
            })
    }

    /// Actively obtain fresh causal per-axis physical disarm proof for Damiao motors using ONLY
    /// read-only `0x7FF 0xCC` status polls (`TxKind::DisableProbe`) within a bounded window
    /// (`<= SHUTDOWN_DISARM_PROOF_WINDOW`, clamped to `wall_deadline`).
    ///
    /// - Never sends `CMD_ENABLE`, motion, or `CMD_DISABLE` frames.
    /// - Accepts fresh causal `state == 0` replies (including axes never enabled in this boot)
    ///   as disarm proof.
    /// - Fast-fails and logs the offending axis if any axis replies enabled (`state == 1`) or
    ///   faulted (`state != 0`), and logs every silent/unconfirmed axis on timeout.
    fn probe_hardware_disarmed_until(&mut self, wall_deadline: Instant) -> bool {
        self.probe_hardware_disarmed_until_impl(wall_deadline)
    }

    fn reported_torque_loss_sample_seq(&self) -> Option<u64> {
        Some(self.reported_torque_loss_sample_seq)
    }

    fn has_driver_fresh_position_api(&self) -> bool { true }

    fn fresh_measured_position_for(&self, joint: usize, max_age: Duration) -> Option<f32> {
        self.fresh_measured_position(joint, max_age)
    }

    fn fresh_measured_sample_for(&self, joint: usize, max_age: Duration) -> Option<(f32, u64)> {
        let pos = self.fresh_measured_position(joint, max_age)?;
        let epoch = self.feedback_continuity_epoch.get(joint).copied().unwrap_or(0);
        Some((pos, epoch))
    }

    fn rest_pose_gate_status(&self) -> RestPoseGateStatus {
        self.evaluate_rest_pose_gate()
    }

    fn supports_budgeted_disable_await(&self) -> bool {
        true
    }

    fn begin_control_release(&mut self, release_seq: u64) {
        if self.control_release_id != release_seq {
            self.control_release_id = release_seq;
            self.release_rx_frames = 0;
            self.release_rx_limit = crate::rt_core::ops::RX_FRAME_BUDGET;
            self.release_tx_limit = None;
            self.begin_tx_tick();
            self.output_step_pending_in_release = true;
        }
        self.control_release_managed = true;
    }

    fn service_release_watchdog_fallback(
        &mut self,
        authorized_mask: u64,
        hold_targets: Option<&[Option<f32>]>,
    ) {
        self.output_step_pending_in_release = false;
        if authorized_mask == 0
            || self.motor_energized_mask == 0
            || self.is_enabling
            || self.mit_fence_trip.is_some()
            || self.tick_tx.is_some_and(|p| p.backpressured)
        {
            return;
        }
        let exclude_mask = !(authorized_mask as u32);
        // When an MIT fence is armed (untripped), any authorized energized arm axis that missed
        // its frame in this release must receive a valid continuity hold in the same release
        // (`Duration::ZERO` since `release_ctrl_tx_mask` already excludes axes refreshed in this
        // release) so a 50 Hz release never opens a 40 ms gap that trips the fence.
        let min_age = if self.torque_hs.is_some() || self.mit_fence.is_some() {
            Duration::ZERO
        } else if self.mit_expiry_recovery_active {
            MIT_RECOVERY_CONTINUITY_HOLD_INTERVAL
        } else {
            WATCHDOG_DISABLE_FALLBACK_INTERVAL
        };
        let deadline = self.tx_release_deadline.map(|at| Deadline { at });
        if self.torque_hs.is_some() {
            if let (Some(pass), Some(cap)) = (self.tick_tx.as_mut(), self.tx_release_deadline) {
                if pass.deadline < cap {
                    pass.deadline = cap;
                }
            }
        }
        let _ = self.service_energized_watchdog_holds_with_targets(
            min_age,
            deadline,
            exclude_mask,
            hold_targets,
            false,
        );
    }

    fn has_any_joint_enabled(&self) -> bool {
        if self.awaiting_rx_boundary {
            return false;
        }
        (0..self.motor_ids.len()).any(|i| {
            self.last_feedback_instant
                .get(i)
                .copied()
                .flatten()
                .is_some_and(|t| self.since(t) <= DAMIAO_FEEDBACK_VALID_WINDOW)
                && self.motor_status.get(i).copied() == Some(1)
        })
    }

    fn has_known_partial_enable(&self) -> bool {
        if self.motor_ids.is_empty() || self.awaiting_rx_boundary || !self.is_feedback_valid() {
            return false;
        }
        let has_on = self.motor_status.contains(&1);
        let has_off = self.motor_status.contains(&0);
        has_on && has_off
    }

    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        self.last_feedback_instant.iter().map(|t| t.map(|t| self.since(t))).collect()
    }

    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
        out.clear();
        out.extend(self.last_feedback_instant.iter().map(|t| t.map(|t| self.since(t))));
    }

    fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
        self.last_feedback_instant.len() == dof
            && self
                .last_feedback_instant
                .iter()
                .all(|t| t.is_some_and(|t| self.since(t) <= max_age))
    }

    fn feedback_instants(&self) -> Vec<Option<Instant>> {
        self.last_feedback_instant.clone()
    }

    fn feedback_seqs(&self) -> Vec<u64> {
        self.feedback_seq.clone()
    }

    fn feedback_queue_bounds(&self) -> Vec<Option<Instant>> {
        self.feedback_queue_bound.clone()
    }

    fn feedback_receive_instants(&self) -> Vec<Option<Instant>> {
        self.last_feedback_instant.clone()
    }


    fn max_command_velocity(&self, joint: usize) -> Option<f32> {
        if joint >= self.motor_ids.len() || self.control_mode != DamiaoControlMode::PosVel {
            return None;
        }
        Some(self.default_vlim.abs().clamp(0.1, 4.0))
    }

    fn is_enabled(&self) -> bool {
        self.is_enabled && self.enable_state() == EnableState::Enabled
    }

    fn motor_status(&self) -> Option<Vec<u8>> {
        Some(self.motor_status.clone())
    }

    fn set_torque_flag(&mut self, enabled: bool) {
        self.is_enabled = enabled;
        if enabled {
            self.last_command_was_disable = false;
        } else {
            self.pending_rearm = true;
            self.discard_all_pending_tx();
        }
    }

    fn clock_now(&self) -> Instant { self.now() }

    fn driver_clock(&self) -> Option<std::sync::Arc<dyn crate::transport::DriverClock>> {
        self.clock.clone()
    }

    fn motor_zero_support(&self) -> MotorZeroSupport {
        MotorZeroSupport::set_current("motor_flash", false, vec![
            "Damiao set-zero frame FF FF FF FF FF FF FF FE on the motor's CAN ID, sent with every motor disabled \
             and confirmed disabled from fresh feedback, one motor at a time (reBotArm_control_py RebotArm.set_zero)".into(),
            "Not verified on hardware whether the zero survives a power cycle: the official wiki says the B601-DM is \
             calibrated once after assembly; motorbridge's Damiao notes say the zero is lost at power-off for the \
             gripper (multi-turn travel)".into(),
        ])
    }

    fn is_feedback_valid(&self) -> bool {
        if self.last_feedback_instant.is_empty() {
            return false;
        }
        let limit = DAMIAO_FEEDBACK_VALID_WINDOW;
        let mut all_valid = true;
        // Use monotonic process uptime to prevent clock-jump discontinuities on RTC synchronization
        static MONOTONIC_START: std::sync::LazyLock<Instant> = std::sync::LazyLock::new(Instant::now);
        let now_sec = MONOTONIC_START.elapsed().as_secs();
        let last_warn = self.last_warn_sec.load(Ordering::Relaxed);
        let should_warn = now_sec.saturating_sub(last_warn) >= 3;

        for (i, opt) in self.last_feedback_instant.iter().enumerate() {
            let ok = opt.is_some_and(|instant| self.since(instant) <= limit);
            if !ok {
                all_valid = false;
                if should_warn && (self.is_enabled || self.is_enabling) {
                    tracing::warn!(
                        "Damiao Joint {} (CAN ID 0x{:X}) feedback stale or missing: elapsed={:?} > {:?}",
                        i,
                        self.motor_ids.get(i).copied().unwrap_or(0),
                        opt.map(|t| self.since(t)),
                        limit
                    );
                }
            }
        }
        if !all_valid && should_warn && (self.is_enabled || self.is_enabling) {
            self.last_warn_sec.store(now_sec, Ordering::Relaxed);
        }
        all_valid
    }

    fn check_hardware_status(&self) -> Result<(), ArmDriverError> {
        for (i, &st) in self.motor_status.iter().enumerate() {
            let motor_id = self.motor_ids.get(i).copied().unwrap_or(0);
            if damiao_fault_name(st).is_some() {
                return Err(ArmDriverError::MotorHardwareFault {
                    joint: i,
                    motor_id,
                    status: st,
                });
            }
        }
        Ok(())
    }

    fn clear_fault(&mut self) -> Result<(), ArmDriverError> {
        self.discard_all_pending_tx();
        self.begin_tx_tick();
        let start_stamp = self.current_stop_stamp();
        let mut err: Option<ArmDriverError> = None;
        let mut sent_any = false;
        for idx in 0..self.motor_ids.len() {
            match self.send_plain(TxKind::ClearFault, Some(start_stamp), None, false, |d| {
                Some(damiao::build_clear_faults_frame(d.motor_ids[idx]))
            }) {
                Ok(_) => sent_any = true,
                Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                    return Err(ArmDriverError::EstopInhibited);
                }
                Err(TxRefusal::Deferred) => {
                    let pending = self.motor_ids.len().saturating_sub(idx);
                    if err.is_none() {
                        err = Some(ArmDriverError::TxDeferred { pending });
                    }
                    break;
                }
                Err(TxRefusal::Transport(e)) => {
                    err = Some(ArmDriverError::HardwareFault(e.to_string()));
                }
                Err(r) => {
                    err = Some(Self::refusal_to_driver_err(r));
                }
            }
        }
        self.pending_stop_check(Some(start_stamp))?;
        if sent_any {
            if let Err(e) = self.flush_can_and_commit() {
                if err.is_none() {
                    err = Some(ArmDriverError::HardwareFault(e.to_string()));
                }
            }
        }
        match err {
            Some(e) => Err(e),
            None => Ok(()),
        }
    }

    fn clear_fault_budgeted(&mut self, deadline: Instant) -> Result<(), ArmDriverError> {
        if !self.can.supports_nonblocking_rt() || !self.can.supports_bounded_flush() {
            return Err(ArmDriverError::Protocol(
                "UNSUPPORTED: transport does not support non-blocking RT I/O and bounded flush".into(),
            ));
        }
        self.discard_all_pending_tx();
        let dl = Deadline { at: deadline };
        let start_stamp = self.current_stop_stamp();
        let mut err: Option<ArmDriverError> = None;
        let mut sent_any = false;
        for idx in 0..self.motor_ids.len() {
            match self.send_plain(TxKind::ClearFault, Some(start_stamp), Some(dl), false, |d| {
                Some(damiao::build_clear_faults_frame(d.motor_ids[idx]))
            }) {
                Ok(_) => sent_any = true,
                Err(TxRefusal::DeadlinePassed) => {
                    return Err(ArmDriverError::Transport(TransportError::Timeout));
                }
                Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                    return Err(ArmDriverError::EstopInhibited);
                }
                Err(TxRefusal::Deferred) => {
                    let pending = self.motor_ids.len().saturating_sub(idx);
                    if err.is_none() {
                        err = Some(ArmDriverError::TxDeferred { pending });
                    }
                    break;
                }
                Err(TxRefusal::Transport(e)) => {
                    err = Some(ArmDriverError::Transport(e));
                }
                Err(r) => {
                    err = Some(Self::refusal_to_driver_err(r));
                }
            }
        }
        if dl.passed(self.now()) {
            return Err(ArmDriverError::Transport(TransportError::Timeout));
        }
        self.pending_stop_check(Some(start_stamp))?;
        if sent_any {
            if let Err(e) = self.flush_can_and_commit() {
                if err.is_none() {
                    err = Some(ArmDriverError::Transport(e));
                }
            }
        }
        if dl.passed(self.now()) {
            return Err(ArmDriverError::Transport(TransportError::Timeout));
        }
        match err {
            Some(e) => Err(e),
            None => Ok(()),
        }
    }

    /// Blocking torque transition: [`ArmDriver::begin_torque`], then served every 0.5 ms until
    /// it finishes. An unconfirmed disable returns Ok (callers check the enable state) unless the
    /// MIT fence is armed.
    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        if enabled {
            let mut progress = self.begin_torque(true)?;
            let mut first = true;
            loop {
                if let TorqueProgress::Done(_) = progress { return Ok(()); }
                if !first { self.pause(Duration::from_micros(500)); }
                first = false;
                let deadline = self.now() + DAMIAO_RX_PASS;
                progress = self.poll_torque(deadline)?;
            }
        } else {
            let started_at = self.now();
            self.disable_torque(DisableKind::DisableRequested { started_at })
        }
    }

    fn disable_torque(&mut self, kind: DisableKind) -> Result<(), ArmDriverError> {
        let prev_managed = self.control_release_managed;
        let prev_rx_frames = self.release_rx_frames;
        let prev_rx_limit = self.release_rx_limit;
        let prev_tx_limit = self.release_tx_limit;
        self.control_release_managed = true;
        self.release_rx_frames = 0;
        self.release_rx_limit = crate::rt_core::ops::RX_FRAME_BUDGET;
        self.release_tx_limit = None;
        let mut last_release_at = self.now();
        let res = (|| {
            let mut progress = self.begin_disable_torque(kind)?;
            let mut first = true;
            loop {
                if let TorqueProgress::Done(_) = progress { return Ok(()); }
                if !first { self.pause(Duration::from_micros(500)); }
                first = false;
                let now = self.now();
                if now.saturating_duration_since(last_release_at) >= EVIDENCE_PROBE_INTERVAL {
                    last_release_at = now;
                    self.control_release_id = self.control_release_id.wrapping_add(1).max(1);
                }
                self.release_rx_frames = 0;
                let deadline = self.now() + DAMIAO_RX_PASS;
                progress = self.poll_torque(deadline)?;
            }
        })();
        self.control_release_managed = prev_managed;
        self.release_rx_frames = prev_rx_frames;
        self.release_rx_limit = prev_rx_limit;
        self.release_tx_limit = prev_tx_limit;
        res
    }

    fn begin_torque(&mut self, enabled: bool) -> Result<TorqueProgress, ArmDriverError> {
        if enabled {
            self.begin_enable()
        } else {
            let started_at = self.now();
            self.begin_disable(DisableKind::DisableRequested { started_at })
        }
    }

    fn begin_disable_torque(&mut self, kind: DisableKind) -> Result<TorqueProgress, ArmDriverError> {
        self.begin_disable(kind)
    }

    fn begin_torque_budgeted(&mut self, enabled: bool, max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
        if !self.can.supports_nonblocking_rt() {
            return Err(ArmDriverError::ProtocolStatic(
                "UNSUPPORTED: CAN transport does not support non-blocking RT I/O",
            ));
        }
        if enabled {
            self.begin_enable_budgeted(Some(max_rx_frames))
        } else {
            let started_at = self.now();
            self.begin_disable_budgeted(
                DisableKind::DisableRequested { started_at },
                Some(max_rx_frames),
            )
        }
    }

    fn begin_torque_budgeted_with_range_retry(
        &mut self,
        enabled: bool,
        max_rx_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        if !self.can.supports_nonblocking_rt() {
            return Err(ArmDriverError::ProtocolStatic(
                "UNSUPPORTED: CAN transport does not support non-blocking RT I/O",
            ));
        }
        if enabled {
            self.begin_enable_budgeted_inner(Some(max_rx_frames), true)
        } else {
            let started_at = self.now();
            self.begin_disable_budgeted(
                DisableKind::DisableRequested { started_at },
                Some(max_rx_frames),
            )
        }
    }

    fn begin_disable_torque_budgeted(
        &mut self,
        kind: DisableKind,
        max_rx_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        if !self.can.supports_nonblocking_rt() {
            return Err(ArmDriverError::ProtocolStatic(
                "UNSUPPORTED: CAN transport does not support non-blocking RT I/O",
            ));
        }
        self.begin_disable_budgeted(kind, Some(max_rx_frames))
    }

    fn begin_client_park_disable_with_kind(
        &mut self,
        kind: DisableKind,
        max_rx_frames: Option<u16>,
        expected_stop_gen: Option<u64>,
    ) -> Result<TorqueProgress, ArmDriverError> {
        if max_rx_frames.is_some() && !self.can.supports_nonblocking_rt() {
            return Err(ArmDriverError::Protocol(
                "UNSUPPORTED: CAN transport does not support non-blocking RT I/O".into(),
            ));
        }
        self.begin_disable_inner(kind, max_rx_frames, true, expected_stop_gen)
    }

    fn poll_torque(&mut self, deadline: Instant) -> Result<TorqueProgress, ArmDriverError> {
        self.poll_torque_inner(deadline, 64)
    }

    fn poll_torque_budgeted(&mut self, deadline: Instant, max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
        if !self.can.supports_nonblocking_rt() {
            return Err(ArmDriverError::ProtocolStatic(
                "UNSUPPORTED: CAN transport does not support non-blocking RT I/O",
            ));
        }
        self.poll_torque_inner(deadline, max_rx_frames)
    }

    fn take_last_poll_rx(&mut self) -> Option<(u16, bool)> {
        self.last_poll_rx.take()
    }

    fn torque_transition_pending(&self) -> bool { self.torque_hs.is_some() }

    fn abort_enable(&mut self) {
        self.enable_tx_deadline = None;
        self.enable_lease_gen = None;
        if self.torque_hs.as_ref().is_some_and(|hs| {
            !matches!(hs, DamiaoTorque::Disable { .. } | DamiaoTorque::AwaitRestGate { .. })
        }) {
            if matches!(
                self.torque_hs,
                Some(DamiaoTorque::DrainBeforeEnable(_) | DamiaoTorque::RangeCheck(_))
            ) {
                self.awaiting_rx_boundary = false;
            }
            self.torque_hs = None;
            self.enable_failed();
        }
    }

    fn last_committed_wire_position(&self, joint: usize) -> Option<(u64, Instant, f32)> {
        self.committed_wire_pos.get(joint).copied().flatten()
    }

    fn prepare_safe_hold(&mut self) {
        self.abort_enable();
        // Also clear any abandoned in-flight disable handshake while preserving
        // `self.disable_dispatched_mask` so already-disabled axes remain `SkippedOff` while all
        // remaining energized axes continue receiving measured-position holds (DRV-F1 B10).
        if matches!(
            self.torque_hs,
            Some(DamiaoTorque::Disable { .. } | DamiaoTorque::AwaitRestGate { .. })
        ) {
            self.torque_hs = None;
        }
        self.disable_rest_established = false;
        self.disable_tx_deadline = None;
        self.disable_lease_gen = None;
        self.mode_switch_active_mask = 0;
        self.mode_switch_hold_targets = [None; MAX_MODE_JOINTS];
        self.mit_expiry_recover_reset();
        self.set_external_pos_force_emitter(false);
    }

    fn enable_frames_dispatched(&self) -> bool {
        self.enable_frames_sent_any
            || self.has_any_joint_enabled()
            || self.has_known_partial_enable()
    }

    fn unconfirmed_disable_joints(&self) -> Vec<usize> {
        (0..self.motor_ids.len())
            .filter(|&i| {
                let confirmed = !self.awaiting_rx_boundary
                    && self.panic_disable_confirmed.get(i).copied().unwrap_or(false)
                    && self.motor_status.get(i).copied() == Some(0)
                    && self
                        .last_feedback_instant
                        .get(i)
                        .copied()
                        .flatten()
                        .is_some_and(|t| self.since(t) <= DAMIAO_FEEDBACK_VALID_WINDOW);
                !confirmed
            })
            .collect()
    }

    fn supports_rt_executor(&self) -> Result<(), &'static str> {
        if self.can.supports_nonblocking_rt() && self.can.supports_bounded_flush() {
            Ok(())
        } else {
            Err("CAN transport does not support non-blocking RT I/O and bounded flush")
        }
    }

    fn set_rt_mode(&mut self, rt: bool) {
        self.rt_no_sleep = rt;
    }

    fn clock_pause(&self, d: Duration) { self.pause(d) }

    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        match self.write_joint_targets_bounded(targets) {
            Ok(TxBatchOutcome::Complete) => Ok(()),
            Ok(TxBatchOutcome::Deferred { pending }) => {
                // Legacy callers treat Err as output failure and stop motion without notifying
                // the driver; discard queued motion frames so none survive into a faulted state.
                self.discard_pending_tx_stopped();
                Err(ArmDriverError::TxDeferred { pending })
            }
            Ok(TxBatchOutcome::Expired { .. }) => {
                self.discard_pending_tx_stopped();
                Err(ArmDriverError::Transport(TransportError::Timeout))
            }
            Err(e) => {
                self.discard_pending_tx_stopped();
                Err(e)
            }
        }
    }

    /// Stationary restorative hold while a stop/E-stop is latched (zero allocation):
    /// discards queued motion, refuses pending stops, enforces the tick TX budget and optional
    /// MIT fence, and rebuilds every frame at send time from fresh (`<= 100 ms`) measured
    /// feedback through [`Self::send`] with `TxKind::RollbackHold`.
    fn write_estop_hold(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        match self.write_estop_hold_bounded(targets) {
            Ok(TxBatchOutcome::Complete) => Ok(()),
            Ok(TxBatchOutcome::Deferred { pending }) => {
                self.discard_pending_hold_tx();
                Err(ArmDriverError::TxDeferred { pending })
            }
            Ok(TxBatchOutcome::Expired { .. }) => {
                self.discard_pending_hold_tx();
                Err(ArmDriverError::Transport(TransportError::Timeout))
            }
            Err(e) => {
                self.discard_pending_hold_tx();
                Err(e)
            }
        }
    }

    fn disable_frames_dispatched(&self) -> bool {
        self.disable_dispatched_mask != 0
            || matches!(
                self.torque_hs,
                Some(DamiaoTorque::Disable { repeats_sent: 1.., .. })
            )
    }

    fn disable_dispatched_mask(&self) -> u64 {
        self.disable_dispatched_mask
    }

    fn abort_pending_disable(&mut self) -> bool {
        match self.torque_hs {
            None => true,
            Some(DamiaoTorque::AwaitRestGate {
                kind: DisableKind::DisableRequested { .. },
                ..
            }) if self.disable_dispatched_mask == 0 => {
                self.torque_hs = None;
                self.disable_rest_established = false;
                self.disable_tx_deadline = None;
                self.disable_lease_gen = None;
                true
            }
            Some(DamiaoTorque::Disable {
                kind: DisableKind::DisableRequested { .. },
                repeats_sent: 0,
                ..
            }) if self.disable_dispatched_mask == 0 => {
                self.torque_hs = None;
                self.disable_rest_established = false;
                self.disable_tx_deadline = None;
                self.disable_lease_gen = None;
                true
            }
            Some(DamiaoTorque::AwaitRestGate { .. } | DamiaoTorque::Disable { .. }) => false,
            Some(_) => true,
        }
    }

    fn write_estop_hold_bounded(&mut self, targets: &[JointTarget]) -> Result<TxBatchOutcome, ArmDriverError> {
        self.write_estop_hold_bounded_impl(targets)
    }

    /// Masked per-joint measured-position hold pass (§2/§3, zero heap allocation):
    /// - Validates every joint independently: one unavailable joint (e.g. a POS_FORCE gripper
    ///   with cleared mode verification, or a joint with stale feedback) NEVER vetoes other
    ///   joints' holds.
    /// - Holds EVERY non-`PosForce` joint in `hold_mask` regardless of feedback-reported enable
    ///   state (a disabled Damiao motor ignores hold frames).
    /// - NEVER constructs or transmits `0x307` (`PosForce`) frames directly: active `PosForce` J7
    ///   holds are expressed as a J7 `PosForceIntent::stop` through the single emitter
    ///   (`SupervisorInner` tick epilogue when `external_pos_force_emitter == true`, or
    ///   `self.emit_pos_force_j7` when standalone).
    /// - Uses the change-class TX scheduler (`peek_best_hold` + `j7_tx_priority(true)` yield) and
    ///   records per-motor stopped-hold progress (`last_hold_tx_due_mask` / `last_hold_tx_sent_mask`).
    fn write_measured_hold_masked(
        &mut self,
        hold_mask: u64,
        targets: &[JointTarget],
    ) -> Result<HoldPassReport, ArmDriverError> {
        self.write_measured_hold_masked_impl(hold_mask, targets)
    }

    fn defer_motion_targets_past_cutoff(&mut self, targets: &[JointTarget]) {
        let batch_lease_gen = self.output_lease_gen;
        self.output_lease_gen = 0;
        if targets.len() != self.motor_ids.len()
            || self.torque_hs.is_some()
            || self.mit_fence.is_some()
            || self.mit_fence_trip.is_some()
            || self.target_frames_refused(None, true).is_some()
            || self.strict_stop_check(None).is_err()
            || self.is_lease_fenced(batch_lease_gen)
        {
            return;
        }
        let batch_stamp = self.current_stop_stamp();
        let batch_ts = self.now();
        for (i, target) in targets.iter().enumerate() {
            if self.joint_modes.get(i).copied().flatten() == Some(DamiaoControlMode::PosForce) {
                continue;
            }
            if let Ok(cmds) = self.pending_target_cmds_checked(i, target) {
                for (frame_id, cmd) in cmds.into_iter().flatten() {
                    let _ = self
                        .pending_tx
                        .upsert(i, frame_id, cmd, batch_ts, batch_stamp, batch_lease_gen);
                }
            }
        }
    }

    fn write_joint_targets_bounded(&mut self, targets: &[JointTarget]) -> Result<TxBatchOutcome, ArmDriverError> {
        self.write_joint_targets_bounded_impl(targets)
    }

    fn set_tx_release_deadline(&mut self, deadline: Option<Instant>) {
        self.tx_release_deadline = deadline;
        if deadline.is_none() {
            self.output_step_pending_in_release = false;
        }
    }

    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        if !self.control_release_managed {
            self.control_release_id = self.control_release_id.wrapping_add(1).max(1);
        }
        self.receive_feedback()?;
        Ok(self.cached_states.clone())
    }

    fn read_joint_states_rx_only(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        if !self.control_release_managed {
            self.control_release_id = self.control_release_id.wrapping_add(1).max(1);
        }
        self.drain_rx_only_record_registers()?;
        Ok(self.cached_states.clone())
    }

    fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
        if !self.control_release_managed {
            self.control_release_id = self.control_release_id.wrapping_add(1).max(1);
        }
        self.receive_feedback()?;
        out.clear();
        out.extend_from_slice(&self.cached_states);
        Ok(())
    }

    fn read_joint_states_budgeted(
        &mut self,
        out: &mut Vec<JointState>,
        deadline: Instant,
        max_rx_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        if !self.can.supports_nonblocking_rt() {
            return Err(ArmDriverError::ProtocolStatic(
                "UNSUPPORTED: CAN transport does not support non-blocking RT I/O",
            ));
        }
        if !self.control_release_managed {
            self.control_release_id = self.control_release_id.wrapping_add(1).max(1);
        }
        self.last_poll_rx = None;
        self.rx_deadline = Some(deadline);
        let effective_rx_limit = self.clamp_release_rx_limit(max_rx_frames);
        self.rx_frame_limit = Some(effective_rx_limit);
        let r = self.receive_feedback();
        self.rx_deadline = None;
        self.rx_frame_limit = None;
        r?;
        out.clear();
        out.extend_from_slice(&self.cached_states);
        Ok(self.last_poll_rx.take().unwrap_or((0, false)))
    }

    fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
        out.clear();
        out.extend_from_slice(&self.cached_states);
        Some(())
    }

    fn feedback_joints_within(&self, joints: &[usize], max_age: Duration) -> bool {
        joints.iter().all(|&i| {
            self.last_feedback_instant
                .get(i)
                .copied()
                .flatten()
                .is_some_and(|t| self.since(t) <= max_age)
                && self
                    .feedback_queue_bound
                    .get(i)
                    .copied()
                    .flatten()
                    .is_some_and(|b| self.since(b) <= max_age)
        })
    }
    fn fresh_joint_positions_into(
        &self,
        joints: &[usize],
        max_age: Duration,
        out: &mut [f32],
    ) -> bool {
        if joints.is_empty() || out.len() < joints.len() {
            return false;
        }
        // E2: A cached resting pose never authorizes a disable while newer RX frames may still be
        // queued (`awaiting_rx_boundary` or incomplete last drain).
        if self.awaiting_rx_boundary || !self.last_read_drained {
            return false;
        }
        let caps = self.capabilities();
        for (slot, &i) in out.iter_mut().zip(joints.iter()) {
            let fresh = self
                .pose_feedback_instant
                .get(i)
                .copied()
                .flatten()
                .is_some_and(|t| self.since(t) <= max_age)
                && self
                    .feedback_queue_bound
                    .get(i)
                    .copied()
                    .flatten()
                    .is_some_and(|b| self.since(b) <= max_age);
            if !fresh {
                return false;
            }
            let Some(s) = self.cached_states.get(i) else {
                return false;
            };
            if !joint_state_valid(s, caps) {
                return false;
            }
            *slot = s.position;
        }
        true
    }
    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
        out.clear();
        out.extend(self.last_feedback_instant.iter().map(|t| t.map(|t| self.since(t).as_millis().min(u64::MAX as u128) as u64)));
    }
    fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) { out.clear(); out.extend_from_slice(&self.last_feedback_instant); }
    fn feedback_seqs_into(&self, out: &mut Vec<u64>) { out.clear(); out.extend_from_slice(&self.feedback_seq); }
    fn feedback_queue_bounds_into(&self, out: &mut Vec<Option<Instant>>) { out.clear(); out.extend_from_slice(&self.feedback_queue_bound); }
    fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) { out.clear(); out.extend_from_slice(&self.last_feedback_instant); }
    fn motor_status_into(&self, out: &mut Vec<u8>) -> bool { out.clear(); out.extend_from_slice(&self.motor_status); true }
    fn clear_fault_disables_arm_joint(&self, _arm_dof: usize) -> bool { false }
    fn has_hardware_fault(&self) -> bool {
        self.motor_status.iter().any(|&st| damiao_fault_name(st).is_some())
    }
    fn set_abort_flag(&mut self, flag: Arc<AtomicBool>) { self.abort_flag = Some(flag); }
    fn set_estop_state(&mut self, state: Arc<crate::rt_core::urgent::EstopState>) { self.estop_state = Some(state); }
    fn set_estop_gate(&mut self, gate: crate::rt_core::urgent::EstopGate) { self.estop_gate = Some(gate); }
    fn set_stopped_disable_gate(&mut self, gate: StoppedDisableGate) { self.stopped_disable_gate = Some(gate); }
    fn tx_ext(&self) -> Option<&dyn BoundedTxExt> { Some(self) }
    fn tx_ext_mut(&mut self) -> Option<&mut dyn BoundedTxExt> { Some(self) }
    fn mode_switch_ext(&self) -> Option<&dyn ModeSwitchExt> { Some(self) }
    fn mode_switch_ext_mut(&mut self) -> Option<&mut dyn ModeSwitchExt> { Some(self) }
    fn commissioning_ext(&self) -> Option<&dyn CommissioningExt> { Some(self) }
    fn commissioning_ext_mut(&mut self) -> Option<&mut dyn CommissioningExt> { Some(self) }
    fn joint_mode_ext(&self) -> Option<&dyn JointModeExt> { Some(self) }
    fn joint_mode_ext_mut(&mut self) -> Option<&mut dyn JointModeExt> { Some(self) }
    fn mit_ext(&self) -> Option<&dyn MitExt> { Some(self) }
    fn mit_ext_mut(&mut self) -> Option<&mut dyn MitExt> { Some(self) }
    fn pos_force_ext(&self) -> Option<&dyn PosForceExt> { Some(self) }
    fn pos_force_ext_mut(&mut self) -> Option<&mut dyn PosForceExt> { Some(self) }
    fn bus_recovery_ext(&self) -> Option<&dyn BusRecoveryExt> { Some(self) }
    fn bus_recovery_ext_mut(&mut self) -> Option<&mut dyn BusRecoveryExt> { Some(self) }
    #[cfg(not(target_arch = "wasm32"))]
    fn shadow_ext(&self) -> Option<&dyn ShadowExt> { Some(self) }
    #[cfg(not(target_arch = "wasm32"))]
    fn shadow_ext_mut(&mut self) -> Option<&mut dyn ShadowExt> { Some(self) }
}

#[cfg(not(target_arch = "wasm32"))]
impl<C: CanTransport> ShadowExt for DamiaoArmDriver<C> {
    fn shadow_vendor_kind(&self) -> crate::torque::VendorKind {
        crate::torque::VendorKind::Damiao
    }

    fn set_shadow_enabled(&mut self, enabled: bool) {
        self.shadow_staging.set_enabled(enabled);
        if enabled {
            let now = self.now();
            for (i, verified) in self
                .verified_modes
                .iter()
                .enumerate()
                .take(crate::torque::MAX_CHANNELS)
            {
                if let Some(m) = *verified {
                    self.shadow_staging.push(
                        crate::torque::shadow::ShadowDriverObs::ModeVerified {
                            channel: i as u8,
                            mode: damiao_mode_to_shadow_control_mode(m),
                            now,
                            provenance: crate::torque::shadow::EvidenceProvenance::HardwareCausal,
                        },
                    );
                }
            }
        }
    }

    fn drain_shadow_observations(
        &mut self,
        out: &mut [Option<crate::torque::shadow::ShadowDriverObs>],
    ) -> (usize, u64) {
        if self.pending_disable_shadow.is_some() {
            let now = self.now();
            PendingDisableShadowBatch::flush_into(
                &mut self.pending_disable_shadow,
                &mut self.shadow_staging,
                now,
            );
        }
        self.shadow_staging.drain_into(out)
    }
}
