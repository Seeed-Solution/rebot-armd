//! Damiao disable frame dispatch, rest-gate polling, and disable confirmation.

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
    /// Transmits `CMD_DISABLE` to motors through the single choke point `send`.
    /// - For `DisableRequested`: establishes K=3 causal rest-pose evidence on J1–J6 BEFORE the
    ///   first disable frame (entering `DamiaoTorque::AwaitRestGate` with 0 CAN frames if not yet
    ///   ready, or refusing with `TorqueOffWouldDrop` if outside rest tolerance). Once established,
    ///   dispatches across as many ticks as the normal per-tick TX budget allows without setting
    ///   `is_enabled = false` until every motor's disable frame has gone out.
    /// - Zero heap allocations.
    #[allow(clippy::too_many_arguments)]
    pub(in crate::driver::can_servo) fn send_disable_frames(
        &mut self,
        kind: DisableKind,
        prev_awaiting_rx_boundary: bool,
        new_awaiting_rx_boundary: bool,
        client_park: bool,
        expected_stamp: Option<StopStamp>,
        start_stop_stamp: StopStamp,
        started_in_estop: bool,
    ) -> Result<TorqueProgress, ArmDriverError> {
        let after_seq = self.rx_seq;
        let cmd_start = self.now();
        let n = self.motor_ids.len().min(64);
        let all_mask = if n >= 64 { u64::MAX } else { (1u64 << n) - 1 };
        if (self.disable_dispatched_mask & all_mask) == all_mask {
            self.disable_dispatched_mask = 0;
        }
        if client_park && self.motor_ids.is_empty() {
            self.strict_stop_check(expected_stamp)?;
            self.refresh_capture = None;
            if let Some(hs) = self.torque_hs.take() {
                if matches!(hs, DamiaoTorque::DrainBeforeEnable(_)) {
                    self.awaiting_rx_boundary = false;
                }
                if !matches!(hs, DamiaoTorque::Disable { .. } | DamiaoTorque::AwaitRestGate { .. }) {
                    self.enable_failed();
                }
            }
            self.discard_all_pending_tx();
            self.begin_tx_tick();
            self.refresh_scheduler.reset_sweep();
            self.awaiting_rx_boundary = new_awaiting_rx_boundary;
        }
        let skip_dispatched = matches!(kind, DisableKind::DisableRequested { .. });
        let expiry = match kind {
            DisableKind::DisableRequested { started_at } => {
                Some(started_at + STOPPED_DISABLE_REQUEST_LIFETIME)
            }
            DisableKind::DisableProtective { .. } => None,
        };
        if skip_dispatched {
            if let Some(exp) = expiry {
                if self.now() >= exp {
                    self.awaiting_rx_boundary = prev_awaiting_rx_boundary || !self.last_read_drained;
                    return Err(ArmDriverError::DisableRequestExpired);
                }
            }
            if self.disable_lease_gen.is_some_and(|g| self.is_lease_fenced(g)) {
                self.awaiting_rx_boundary = prev_awaiting_rx_boundary || !self.last_read_drained;
                return Err(ArmDriverError::LeaseFenced);
            }
            if self.disable_tx_deadline.is_some_and(|dl| self.now() >= dl) {
                self.awaiting_rx_boundary = prev_awaiting_rx_boundary || !self.last_read_drained;
                return Err(ArmDriverError::DisableRequestExpired);
            }
            if !self.disable_rest_established {
                match self.evaluate_rest_pose_gate() {
                    RestPoseGateStatus::Confirmed => {
                        self.disable_rest_established = true;
                    }
                    RestPoseGateStatus::RefusedNonRest => {
                        self.awaiting_rx_boundary =
                            prev_awaiting_rx_boundary || !self.last_read_drained;
                        #[cfg(not(target_arch = "wasm32"))]
                        self.stage_damiao_pretransport_refusal(
                            Self::disable_tx_kind(kind),
                            &TxRefusal::TorqueOffWouldDrop,
                            None,
                        );
                        return Err(ArmDriverError::TorqueOffWouldDrop);
                    }
                    RestPoseGateStatus::PendingDrainOrWindow | RestPoseGateStatus::StaleOrMissing => {
                        self.awaiting_rx_boundary =
                            prev_awaiting_rx_boundary || !self.last_read_drained;
                        let wake = match kind {
                            DisableKind::DisableRequested { started_at, .. } => {
                                let exp = started_at + STOPPED_DISABLE_REQUEST_LIFETIME;
                                let retry_at = if !self.last_read_drained || self.awaiting_rx_boundary
                                {
                                    cmd_start + Duration::from_millis(20)
                                } else {
                                    cmd_start + EVIDENCE_PROBE_INTERVAL
                                };
                                retry_at.min(exp)
                            }
                            DisableKind::DisableProtective { .. } => cmd_start,
                        };
                        self.torque_hs = Some(DamiaoTorque::AwaitRestGate {
                            kind,
                            next_probe_at: cmd_start,
                            client_park,
                            expected_stamp,
                            start_stop_stamp,
                            started_in_estop,
                        });
                        return Ok(TorqueProgress::Pending { wake });
                    }
                }
            }
        }
        let tx_kind = match (client_park, kind) {
            (true, DisableKind::DisableRequested { started_at }) => {
                TxKind::ClientParkDisable { started_at }
            }
            _ => Self::disable_tx_kind(kind),
        };
        let mut sent_any = false;
        let mut sent_this_pass_mask: u64 = 0;
        let mut budget_paused = false;
        let mut wait_evidence = false;
        let mut abort_reason: Option<PartialDisableReason> = None;
        let mut refused_err = None;
        let mut other_err = None;
        let step_dl = if skip_dispatched {
            self.disable_step_deadline(expiry)
        } else {
            None
        };
        #[cfg(not(target_arch = "wasm32"))]
        let mut deferred_refusal_mask = crate::torque::ChannelMask::EMPTY;
        #[cfg(not(target_arch = "wasm32"))]
        let mut deferred_refusal_outcomes =
            [crate::torque::MotorWriteOutcome::NotTargeted; crate::torque::MAX_CHANNELS];
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.defer_disable_shadow = true;
            self.shadow_disable_pass_sent_mask = if skip_dispatched {
                self.disable_dispatched_mask
            } else {
                0
            };
        }
        for idx in 0..self.motor_ids.len() {
            if skip_dispatched && idx < 64 && (self.disable_dispatched_mask & (1u64 << idx)) != 0 {
                continue;
            }
            if skip_dispatched {
                if self.disable_lease_gen.is_some_and(|g| self.is_lease_fenced(g)) {
                    abort_reason = Some(PartialDisableReason::LeaseLost);
                    break;
                }
                let now_iter = self.now();
                if expiry.is_some_and(|exp| now_iter >= exp) {
                    refused_err = Some(ArmDriverError::DisableRequestExpired);
                    break;
                }
                if self.disable_tx_deadline.is_some_and(|dl| now_iter >= dl) {
                    abort_reason = Some(PartialDisableReason::DeadlineExpired);
                    break;
                }
                if step_dl.is_some_and(|d| d.passed(now_iter)) {
                    budget_paused = true;
                    break;
                }
            }
            match self.send_plain(tx_kind, expected_stamp, step_dl, false, |d| {
                Some(damiao::build_disable_frame(d.motor_ids[idx]))
            }) {
                Ok(_) => {
                    if client_park && !sent_any {
                        self.refresh_capture = None;
                        if let Some(hs) = self.torque_hs.take() {
                            if matches!(hs, DamiaoTorque::DrainBeforeEnable(_)) {
                                self.awaiting_rx_boundary = false;
                            }
                            if !matches!(hs, DamiaoTorque::Disable { .. } | DamiaoTorque::AwaitRestGate { .. }) {
                                self.enable_failed();
                            }
                        }
                        self.discard_all_pending_tx();
                        self.begin_tx_tick();
                        self.refresh_scheduler.reset_sweep();
                        self.awaiting_rx_boundary = new_awaiting_rx_boundary;
                    }
                    sent_any = true;
                    if idx < 64 {
                        self.disable_dispatched_mask |= 1u64 << idx;
                        sent_this_pass_mask |= 1u64 << idx;
                    }
                }
                Err(r @ (TxRefusal::StopInhibited | TxRefusal::StopStampChanged)) if client_park => {
                    #[cfg(not(target_arch = "wasm32"))]
                    if idx < crate::torque::MAX_CHANNELS {
                        deferred_refusal_mask.insert(idx as u8);
                        deferred_refusal_outcomes[idx] =
                            crate::torque::MotorWriteOutcome::RejectedStale;
                    }
                    if !sent_any && self.disable_dispatched_mask == 0 {
                        #[cfg(not(target_arch = "wasm32"))]
                        {
                            self.defer_disable_shadow = false;
                            if self.shadow_staging.is_enabled() && !deferred_refusal_mask.is_empty() {
                                let wk = match tx_kind {
                                    TxKind::DisableProtective { .. } => {
                                        crate::torque::WriteKind::DisableProtective
                                    }
                                    _ => crate::torque::WriteKind::DisableRequested,
                                };
                                PendingDisableShadowBatch::merge_batch(
                                    &mut self.pending_disable_shadow,
                                    PendingDisableShadowBatch {
                                        kind: wk,
                                        target_mask: deferred_refusal_mask,
                                        outcomes: deferred_refusal_outcomes,
                                        ambiguous_gap: false,
                                        write_done: None,
                                    },
                                );
                                self.flush_pending_disable_shadow_now();
                            }
                        }
                        self.awaiting_rx_boundary = prev_awaiting_rx_boundary || !self.last_read_drained;
                        self.disable_rest_established = false;
                        return Err(Self::refusal_to_driver_err(r));
                    }
                    abort_reason = Some(if self.is_stop_pending() || (!started_in_estop && self.is_stop_latched()) {
                        PartialDisableReason::Estop
                    } else {
                        PartialDisableReason::Stopped
                    });
                    break;
                }
                Err(TxRefusal::DisableWaitEvidence) => {
                    wait_evidence = true;
                    if skip_dispatched && (sent_any || self.disable_dispatched_mask != 0) {
                        budget_paused = true;
                        break;
                    }
                }
                Err(TxRefusal::DeadlinePassed | TxRefusal::Deferred) if skip_dispatched => {
                    budget_paused = true;
                    break;
                }
                Err(TxRefusal::Transport(e))
                    if skip_dispatched
                        && e.is_tx_backpressure()
                        && (sent_any || self.disable_dispatched_mask != 0) =>
                {
                    budget_paused = true;
                    break;
                }
                Err(r @ (TxRefusal::TorqueOffWouldDrop | TxRefusal::DisableRequestExpired)) => {
                    #[cfg(not(target_arch = "wasm32"))]
                    if idx < crate::torque::MAX_CHANNELS {
                        deferred_refusal_mask.insert(idx as u8);
                        deferred_refusal_outcomes[idx] =
                            crate::torque::MotorWriteOutcome::RejectedStale;
                    }
                    if matches!(r, TxRefusal::TorqueOffWouldDrop)
                        && skip_dispatched
                        && (sent_any || self.disable_dispatched_mask != 0)
                    {
                        let moved_joint = match self.evaluate_remaining_arm_joints_rest() {
                            RemainingRestStatus::MovedBeyondRest { joint } => joint,
                            _ => idx,
                        };
                        abort_reason = Some(PartialDisableReason::MotionBeyondRest {
                            joint: moved_joint.min(u8::MAX as usize) as u8,
                        });
                        break;
                    }
                    let is_expired = matches!(r, TxRefusal::DisableRequestExpired);
                    if refused_err.is_none() {
                        refused_err = Some(Self::refusal_to_driver_err(r));
                    }
                    if is_expired {
                        break;
                    }
                }
                Err(r) => {
                    #[cfg(not(target_arch = "wasm32"))]
                    if !matches!(r, TxRefusal::Transport(_)) && idx < crate::torque::MAX_CHANNELS {
                        deferred_refusal_mask.insert(idx as u8);
                        deferred_refusal_outcomes[idx] = match r {
                            TxRefusal::Deferred => crate::torque::MotorWriteOutcome::Deferred,
                            _ => crate::torque::MotorWriteOutcome::RejectedStale,
                        };
                    }
                    other_err = Some(Self::refusal_to_driver_err(r));
                    if skip_dispatched && (sent_any || self.disable_dispatched_mask != 0) {
                        abort_reason = Some(PartialDisableReason::TransportError);
                        break;
                    }
                }
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.defer_disable_shadow = false;
            if self.shadow_staging.is_enabled() && !deferred_refusal_mask.is_empty() {
                let wk = match tx_kind {
                    TxKind::DisableProtective { .. } => {
                        crate::torque::WriteKind::DisableProtective
                    }
                    _ => crate::torque::WriteKind::DisableRequested,
                };
                PendingDisableShadowBatch::merge_batch(
                    &mut self.pending_disable_shadow,
                    PendingDisableShadowBatch {
                        kind: wk,
                        target_mask: deferred_refusal_mask,
                        outcomes: deferred_refusal_outcomes,
                        ambiguous_gap: false,
                        write_done: None,
                    },
                );
            }
        }
        let mut flush_failed = false;
        if sent_any || (!client_park && !skip_dispatched) {
            if let Err(e) = self.flush_can_and_commit() {
                flush_failed = true;
                other_err = Some(e.into());
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        self.flush_pending_disable_shadow_now();
        if let Some(reason) = abort_reason {
            let uncertain = if flush_failed && !self.can.flush_is_noop() {
                sent_this_pass_mask
            } else {
                0
            };
            if self.disable_dispatched_mask != 0 {
                return Err(self.build_partial_disable_err(reason, uncertain));
            }
            self.disable_rest_established = false;
            self.disable_tx_deadline = None;
            self.disable_lease_gen = None;
            return Err(match reason {
                PartialDisableReason::Estop | PartialDisableReason::Stopped => {
                    ArmDriverError::EstopInhibited
                }
                PartialDisableReason::LeaseLost => ArmDriverError::LeaseFenced,
                PartialDisableReason::DeadlineExpired => ArmDriverError::DisableRequestExpired,
                PartialDisableReason::MotionBeyondRest { .. } => {
                    ArmDriverError::TorqueOffWouldDrop
                }
                PartialDisableReason::TransportError => {
                    other_err.unwrap_or(ArmDriverError::HardwareFault(
                        "Torque disable transmission failed".into(),
                    ))
                }
            });
        }
        if !sent_any && self.disable_dispatched_mask == 0 && other_err.is_none() {
            if let Some(e) = refused_err {
                self.awaiting_rx_boundary = prev_awaiting_rx_boundary || !self.last_read_drained;
                self.disable_rest_established = false;
                return Err(e);
            }
            if wait_evidence || budget_paused {
                self.awaiting_rx_boundary = prev_awaiting_rx_boundary || !self.last_read_drained;
                if budget_paused && self.disable_rest_established {
                    let mit_fenced = self.mit_fence.is_some() || self.mit_fence_trip.is_some();
                    self.torque_hs = Some(DamiaoTorque::Disable {
                        kind,
                        repeats_sent: 0,
                        repeat_pass_mask: 0,
                        next_repeat_at: cmd_start,
                        next_probe_at: cmd_start,
                        mit_fenced,
                        start_stop_stamp,
                        started_in_estop,
                        client_park,
                        rest_established: true,
                    });
                    return Ok(TorqueProgress::Pending { wake: self.now() });
                }
                let wake = match kind {
                    DisableKind::DisableRequested { started_at, .. } => {
                        let exp = started_at + STOPPED_DISABLE_REQUEST_LIFETIME;
                        let retry_at = if !self.last_read_drained || self.awaiting_rx_boundary {
                            cmd_start + Duration::from_millis(20)
                        } else {
                            cmd_start + EVIDENCE_PROBE_INTERVAL
                        };
                        retry_at.min(exp)
                    }
                    DisableKind::DisableProtective { .. } => cmd_start,
                };
                self.torque_hs = Some(DamiaoTorque::AwaitRestGate {
                    kind,
                    next_probe_at: cmd_start,
                    client_park,
                    expected_stamp,
                    start_stop_stamp,
                    started_in_estop,
                });
                return Ok(TorqueProgress::Pending { wake });
            }
        }
        let all_dispatched = (self.disable_dispatched_mask & all_mask) == all_mask;
        if skip_dispatched && !all_dispatched {
            if flush_failed {
                let uncertain = if !self.can.flush_is_noop() {
                    sent_this_pass_mask
                } else {
                    0
                };
                if self.disable_dispatched_mask != 0 {
                    return Err(self.build_partial_disable_err(
                        PartialDisableReason::TransportError,
                        uncertain,
                    ));
                }
            }
            if let Some(e) = refused_err.or(other_err) {
                return Err(e);
            }
            // Multi-tick continuation: keep `self.is_enabled = true` so not-yet-disabled joints
            // continue receiving hold frames each tick until every motor's disable is dispatched.
            let mit_fenced = self.mit_fence.is_some() || self.mit_fence_trip.is_some();
            self.torque_hs = Some(DamiaoTorque::Disable {
                kind,
                repeats_sent: 0,
                repeat_pass_mask: 0,
                next_repeat_at: cmd_start,
                next_probe_at: cmd_start,
                mit_fenced,
                start_stop_stamp,
                started_in_estop,
                client_park,
                rest_established: self.disable_rest_established,
            });
            return Ok(TorqueProgress::Pending { wake: self.now() });
        }
        // All motors have received their initial CMD_DISABLE (or protective cut ran):
        // now commit driver-side disabled state (`is_enabled = false`) and enter repeats.
        let any_dispatched = sent_any || self.disable_dispatched_mask != 0;
        if all_dispatched || (!skip_dispatched && any_dispatched) {
            self.commit_all_disabled_driver_state(cmd_start, after_seq);
        } else if !client_park && (self.mit_fence.is_some() || self.mit_fence_trip.is_some()) {
            self.clear_cached_targets();
        }
        if let Some(e) = refused_err.or(other_err) {
            return Err(e);
        }
        let mit_fenced = self.mit_fence.is_some() || self.mit_fence_trip.is_some();
        if self.motor_ids.is_empty() || DAMIAO_DISABLE_REPEATS <= 1 {
            if mit_fenced {
                self.mit_fence = None;
            }
            self.disable_dispatched_mask = 0;
            self.disable_rest_established = false;
            self.disable_tx_deadline = None;
            self.disable_lease_gen = None;
            return Ok(TorqueProgress::Done(TorqueOutcome::Confirmed));
        }
        let now = self.now();
        let next_repeat_at = now + DAMIAO_DISABLE_REPEAT_INTERVAL;
        let wake = match kind {
            DisableKind::DisableRequested { started_at } => {
                next_repeat_at.min(started_at + STOPPED_DISABLE_REQUEST_LIFETIME)
            }
            DisableKind::DisableProtective { .. } => next_repeat_at,
        };
        self.torque_hs = Some(DamiaoTorque::Disable {
            kind,
            repeats_sent: 1,
            repeat_pass_mask: 0,
            next_repeat_at,
            next_probe_at: now,
            mit_fenced,
            start_stop_stamp,
            started_in_estop,
            client_park,
            rest_established: self.disable_rest_established,
        });
        Ok(TorqueProgress::Pending { wake })
    }

    /// Emits spaced read-only `0x7FF 0xCC` (`TxKind::DisableProbe`) status probes for every motor
    /// whose K=3 rest-pose window is not yet confirmed, shared by `poll_await_rest_gate` and
    /// `poll_disable` so a silent bus never starves a rest-gated initial or repeat disable pass.
    pub(in crate::driver::can_servo) fn probe_unconfirmed_rest_windows(
        &mut self,
        expiry: Option<Instant>,
        next_probe_at: &mut Instant,
    ) -> bool {
        let now = self.now();
        if !self.last_read_drained || self.awaiting_rx_boundary || now < *next_probe_at {
            return false;
        }
        let mut probe_mask = 0u32;
        let mut probe_count = 0usize;
        if let Some(ref gate) = self.stopped_disable_gate {
            let dof = self.motor_ids.len().min(handshake::MAX_MASK_JOINTS);
            for &i in gate.arm_joint_indices() {
                if i < dof
                    && !self
                        .rest_pose_window
                        .get(i)
                        .is_some_and(|w| w.is_confirmed(now, STOPPED_DISABLE_FEEDBACK_MAX_AGE))
                {
                    probe_mask |= 1u32 << i;
                    probe_count += 1;
                }
            }
        }
        if probe_count == 0 || self.remaining_rx_frame_limit() <= probe_count {
            return false;
        }
        let dl = self.disable_step_deadline(expiry);
        if dl.is_some_and(|d| d.passed(self.now())) {
            return false;
        }
        let mut probed_any = false;
        let dof = self.motor_ids.len().min(handshake::MAX_MASK_JOINTS);
        let start = if dof > 0 {
            (self.control_release_id as usize) % dof
        } else {
            0
        };
        for offset in 0..dof {
            let i = (start + offset) % dof;
            if (probe_mask & (1u32 << i)) == 0 {
                continue;
            }
            if dl.is_some_and(|d| d.passed(self.now())) {
                break;
            }
            let Some(&id) = self.motor_ids.get(i) else { continue };
            match self.send_plain(TxKind::DisableProbe, None, dl, false, |_| {
                Some(damiao::build_refresh_frame(id))
            }) {
                Ok(_) => probed_any = true,
                Err(_) => break,
            }
        }
        if probed_any {
            *next_probe_at = self.now() + EVIDENCE_PROBE_INTERVAL;
            let _ = self.flush_can_and_commit();
            let _ = self.receive_feedback();
        }
        probed_any
    }

    /// Services a requested disable waiting for an exhausted RX drain or incomplete K=3 rest-pose
    /// window to finish inside the 1 s request lifetime (L1). Transmits only read-only `0x7FF 0xCC`
    /// status probes until `evaluate_rest_pose_gate() == Confirmed`, refuses immediately on any
    /// real non-rest pose (`RefusedNonRest`) or 1 s lifetime expiry (`DisableRequestExpired`),
    /// and invokes `send_disable_frames` once confirmed. Zero heap allocations.
    pub(in crate::driver::can_servo) fn poll_await_rest_gate(
        &mut self,
        kind: DisableKind,
        mut next_probe_at: Instant,
        client_park: bool,
        expected_stamp: Option<StopStamp>,
        start_stop_stamp: StopStamp,
        started_in_estop: bool,
    ) -> Result<TorqueProgress, ArmDriverError> {
        if client_park {
            self.strict_stop_check(expected_stamp)?;
        }
        let started_at = match kind {
            DisableKind::DisableRequested { started_at } => started_at,
            DisableKind::DisableProtective { .. } => {
                let prev = self.awaiting_rx_boundary;
                return self.send_disable_frames(
                    kind,
                    prev,
                    prev,
                    false,
                    None,
                    start_stop_stamp,
                    started_in_estop,
                );
            }
        };
        if self.is_stop_pending()
            || (!started_in_estop && self.is_stop_latched())
            || self.current_stop_stamp() != start_stop_stamp
        {
            return Err(ArmDriverError::EstopInhibited);
        }
        if self.disable_lease_gen.is_some_and(|g| self.is_lease_fenced(g)) {
            return Err(ArmDriverError::LeaseFenced);
        }
        if self.now().saturating_duration_since(started_at) >= STOPPED_DISABLE_REQUEST_LIFETIME
            || self.disable_tx_deadline.is_some_and(|dl| self.now() >= dl)
        {
            return Err(ArmDriverError::DisableRequestExpired);
        }
        let _ = self.receive_feedback();
        if client_park {
            self.strict_stop_check(expected_stamp)?;
        }
        if self.is_stop_pending()
            || (!started_in_estop && self.is_stop_latched())
            || self.current_stop_stamp() != start_stop_stamp
        {
            return Err(ArmDriverError::EstopInhibited);
        }
        if self.disable_lease_gen.is_some_and(|g| self.is_lease_fenced(g)) {
            return Err(ArmDriverError::LeaseFenced);
        }
        match self.evaluate_rest_pose_gate() {
            RestPoseGateStatus::Confirmed => {
                let prev = self.awaiting_rx_boundary;
                return self.send_disable_frames(
                    kind,
                    prev,
                    prev,
                    client_park,
                    expected_stamp,
                    start_stop_stamp,
                    started_in_estop,
                );
            }
            RestPoseGateStatus::RefusedNonRest => {
                return Err(ArmDriverError::TorqueOffWouldDrop);
            }
            RestPoseGateStatus::PendingDrainOrWindow | RestPoseGateStatus::StaleOrMissing => {}
        }
        let now = self.now();
        if now.saturating_duration_since(started_at) >= STOPPED_DISABLE_REQUEST_LIFETIME {
            return Err(ArmDriverError::DisableRequestExpired);
        }
        let exp = started_at + STOPPED_DISABLE_REQUEST_LIFETIME;
        if self.probe_unconfirmed_rest_windows(Some(exp), &mut next_probe_at) {
            if client_park {
                self.strict_stop_check(expected_stamp)?;
            }
            match self.evaluate_rest_pose_gate() {
                RestPoseGateStatus::Confirmed => {
                    let prev = self.awaiting_rx_boundary;
                    return self.send_disable_frames(
                        kind,
                        prev,
                        prev,
                        client_park,
                        expected_stamp,
                        start_stop_stamp,
                        started_in_estop,
                    );
                }
                RestPoseGateStatus::RefusedNonRest => {
                    return Err(ArmDriverError::TorqueOffWouldDrop);
                }
                RestPoseGateStatus::PendingDrainOrWindow
                | RestPoseGateStatus::StaleOrMissing => {}
            }
        }
        let after = self.now();
        if after.saturating_duration_since(started_at) >= STOPPED_DISABLE_REQUEST_LIFETIME {
            return Err(ArmDriverError::DisableRequestExpired);
        }
        let wake = if !self.last_read_drained || self.awaiting_rx_boundary {
            (after + Duration::from_millis(20)).min(exp)
        } else {
            next_probe_at.min(exp)
        };
        self.torque_hs = Some(DamiaoTorque::AwaitRestGate {
            kind,
            next_probe_at,
            client_park,
            expected_stamp,
            start_stop_stamp,
            started_in_estop,
        });
        Ok(TorqueProgress::Pending { wake })
    }

    #[allow(clippy::too_many_arguments)]
    pub(in crate::driver::can_servo) fn poll_disable(
        &mut self,
        kind: DisableKind,
        mut repeats_sent: u8,
        mut repeat_pass_mask: u64,
        mut next_repeat_at: Instant,
        mut next_probe_at: Instant,
        mit_fenced: bool,
        start_stop_stamp: StopStamp,
        started_in_estop: bool,
        client_park: bool,
        rest_established: bool,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.disable_rest_established = rest_established;
        let is_requested = matches!(kind, DisableKind::DisableRequested { .. });
        let entry_now = self.now();
        let expiry = match kind {
            DisableKind::DisableRequested { started_at } => {
                let exp = started_at + STOPPED_DISABLE_REQUEST_LIFETIME;
                if entry_now >= exp {
                    if repeats_sent == 0 && self.disable_dispatched_mask != 0 {
                        return Err(self.build_partial_disable_err(
                            PartialDisableReason::DeadlineExpired,
                            0,
                        ));
                    }
                    self.disable_rest_established = false;
                    return Err(ArmDriverError::DisableRequestExpired);
                }
                Some(exp)
            }
            DisableKind::DisableProtective { .. } => None,
        };
        let _ = self.receive_feedback();
        let after_rx_now = self.now();
        if let Some(exp) = expiry {
            if after_rx_now >= exp {
                if repeats_sent == 0 && self.disable_dispatched_mask != 0 {
                    return Err(
                        self.build_partial_disable_err(PartialDisableReason::DeadlineExpired, 0)
                    );
                }
                self.disable_rest_established = false;
                return Err(ArmDriverError::DisableRequestExpired);
            }
        }
        let n = self.motor_ids.len().min(64);
        let all_mask = if n >= 64 { u64::MAX } else { (1u64 << n) - 1 };

        // Phase A: Multi-tick initial dispatch (`repeats_sent == 0`, some motors still enabled).
        if repeats_sent == 0 && is_requested {
            let cur_stamp = self.current_stop_stamp();
            let estop_now = (!started_in_estop && self.is_stop_latched())
                || cur_stamp.estop_gen != start_stop_stamp.estop_gen;
            let stop_changed = cur_stamp.stop_gen != start_stop_stamp.stop_gen;
            if estop_now || stop_changed {
                let reason = if estop_now {
                    PartialDisableReason::Estop
                } else {
                    PartialDisableReason::Stopped
                };
                if self.disable_dispatched_mask != 0 {
                    return Err(self.build_partial_disable_err(reason, 0));
                }
                self.disable_rest_established = false;
                return Err(ArmDriverError::EstopInhibited);
            }
            if self.disable_lease_gen.is_some_and(|g| self.is_lease_fenced(g)) {
                if self.disable_dispatched_mask != 0 {
                    return Err(
                        self.build_partial_disable_err(PartialDisableReason::LeaseLost, 0)
                    );
                }
                self.disable_rest_established = false;
                return Err(ArmDriverError::LeaseFenced);
            }
            if self.disable_tx_deadline.is_some_and(|dl| after_rx_now >= dl) {
                if self.disable_dispatched_mask != 0 {
                    return Err(
                        self.build_partial_disable_err(PartialDisableReason::DeadlineExpired, 0)
                    );
                }
                self.disable_rest_established = false;
                return Err(ArmDriverError::DisableRequestExpired);
            }
            match self.evaluate_remaining_arm_joints_rest() {
                RemainingRestStatus::MovedBeyondRest { joint } => {
                    if self.disable_dispatched_mask != 0 {
                        return Err(self.build_partial_disable_err(
                            PartialDisableReason::MotionBeyondRest {
                                joint: joint.min(u8::MAX as usize) as u8,
                            },
                            0,
                        ));
                    }
                    self.disable_rest_established = false;
                    return Err(ArmDriverError::TorqueOffWouldDrop);
                }
                RemainingRestStatus::NeedFreshFeedback => {
                    // Wait for fresh feedback on still-enabled arm joints (probing with 0xCC if
                    // needed) while keeping `is_enabled = true` so holds keep running.
                    if self.probe_unconfirmed_rest_windows(expiry, &mut next_probe_at) {
                        if let RemainingRestStatus::MovedBeyondRest { joint } =
                            self.evaluate_remaining_arm_joints_rest()
                        {
                            if self.disable_dispatched_mask != 0 {
                                return Err(self.build_partial_disable_err(
                                    PartialDisableReason::MotionBeyondRest {
                                        joint: joint.min(u8::MAX as usize) as u8,
                                    },
                                    0,
                                ));
                            }
                            self.disable_rest_established = false;
                            return Err(ArmDriverError::TorqueOffWouldDrop);
                        }
                    }
                    if self.evaluate_remaining_arm_joints_rest() != RemainingRestStatus::AllAtRest {
                        let after = self.now();
                        let wake = expiry.map_or(next_probe_at, |exp| next_probe_at.min(exp));
                        self.torque_hs = Some(DamiaoTorque::Disable {
                            kind,
                            repeats_sent: 0,
                            repeat_pass_mask: 0,
                            next_repeat_at,
                            next_probe_at,
                            mit_fenced,
                            start_stop_stamp,
                            started_in_estop,
                            client_park,
                            rest_established,
                        });
                        let _ = after;
                        return Ok(TorqueProgress::Pending { wake });
                    }
                }
                RemainingRestStatus::AllAtRest => {}
            }
            let after_seq = self.rx_seq;
            let pass_start = self.now();
            let tx_kind = match (client_park, kind) {
                (true, DisableKind::DisableRequested { started_at }) => {
                    TxKind::ClientParkDisable { started_at }
                }
                _ => Self::disable_tx_kind(kind),
            };
            let step_dl = self.disable_step_deadline(expiry);
            let mut sent_any = false;
            let mut sent_this_pass_mask: u64 = 0;
            let mut abort_reason: Option<PartialDisableReason> = None;
            let mut other_err = None;
            #[cfg(not(target_arch = "wasm32"))]
            let mut deferred_refusal_mask = crate::torque::ChannelMask::EMPTY;
            #[cfg(not(target_arch = "wasm32"))]
            let mut deferred_refusal_outcomes =
                [crate::torque::MotorWriteOutcome::NotTargeted; crate::torque::MAX_CHANNELS];
            #[cfg(not(target_arch = "wasm32"))]
            {
                self.defer_disable_shadow = true;
                self.shadow_disable_pass_sent_mask = self.disable_dispatched_mask;
            }
            for idx in 0..self.motor_ids.len() {
                if idx < 64 && (self.disable_dispatched_mask & (1u64 << idx)) != 0 {
                    continue;
                }
                let cur_stamp = self.current_stop_stamp();
                let estop_now = (!started_in_estop && self.is_stop_latched())
                    || cur_stamp.estop_gen != start_stop_stamp.estop_gen;
                let stop_changed = cur_stamp.stop_gen != start_stop_stamp.stop_gen;
                if estop_now || stop_changed {
                    abort_reason = Some(if estop_now {
                        PartialDisableReason::Estop
                    } else {
                        PartialDisableReason::Stopped
                    });
                    break;
                }
                if self.disable_lease_gen.is_some_and(|g| self.is_lease_fenced(g)) {
                    abort_reason = Some(PartialDisableReason::LeaseLost);
                    break;
                }
                let now_iter = self.now();
                if expiry.is_some_and(|exp| now_iter >= exp) {
                    abort_reason = Some(PartialDisableReason::DeadlineExpired);
                    break;
                }
                if self.disable_tx_deadline.is_some_and(|dl| now_iter >= dl) {
                    abort_reason = Some(PartialDisableReason::DeadlineExpired);
                    break;
                }
                if step_dl.is_some_and(|d| d.passed(now_iter)) {
                    break;
                }
                let exp_stamp = client_park.then_some(start_stop_stamp);
                match self.send_plain(tx_kind, exp_stamp, step_dl, false, |d| {
                    Some(damiao::build_disable_frame(d.motor_ids[idx]))
                }) {
                    Ok(_) => {
                        sent_any = true;
                        if idx < 64 {
                            self.disable_dispatched_mask |= 1u64 << idx;
                            sent_this_pass_mask |= 1u64 << idx;
                        }
                    }
                    Err(TxRefusal::DeadlinePassed | TxRefusal::Deferred) => break,
                    Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => {
                        break;
                    }
                    Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                        #[cfg(not(target_arch = "wasm32"))]
                        if idx < crate::torque::MAX_CHANNELS {
                            deferred_refusal_mask.insert(idx as u8);
                            deferred_refusal_outcomes[idx] =
                                crate::torque::MotorWriteOutcome::RejectedStale;
                        }
                        abort_reason = Some(
                            if self.is_stop_pending() || (!started_in_estop && self.is_stop_latched())
                            {
                                PartialDisableReason::Estop
                            } else {
                                PartialDisableReason::Stopped
                            },
                        );
                        break;
                    }
                    Err(TxRefusal::DisableRequestExpired) => {
                        #[cfg(not(target_arch = "wasm32"))]
                        if idx < crate::torque::MAX_CHANNELS {
                            deferred_refusal_mask.insert(idx as u8);
                            deferred_refusal_outcomes[idx] =
                                crate::torque::MotorWriteOutcome::RejectedStale;
                        }
                        abort_reason = Some(PartialDisableReason::DeadlineExpired);
                        break;
                    }
                    Err(TxRefusal::TorqueOffWouldDrop) => {
                        #[cfg(not(target_arch = "wasm32"))]
                        if idx < crate::torque::MAX_CHANNELS {
                            deferred_refusal_mask.insert(idx as u8);
                            deferred_refusal_outcomes[idx] =
                                crate::torque::MotorWriteOutcome::RejectedStale;
                        }
                        let moved_joint = match self.evaluate_remaining_arm_joints_rest() {
                            RemainingRestStatus::MovedBeyondRest { joint } => joint,
                            _ => idx,
                        };
                        abort_reason = Some(PartialDisableReason::MotionBeyondRest {
                            joint: moved_joint.min(u8::MAX as usize) as u8,
                        });
                        break;
                    }
                    Err(TxRefusal::DisableWaitEvidence) => break,
                    Err(r) => {
                        #[cfg(not(target_arch = "wasm32"))]
                        if !matches!(r, TxRefusal::Transport(_)) && idx < crate::torque::MAX_CHANNELS {
                            deferred_refusal_mask.insert(idx as u8);
                            deferred_refusal_outcomes[idx] = match r {
                                TxRefusal::Deferred => crate::torque::MotorWriteOutcome::Deferred,
                                _ => crate::torque::MotorWriteOutcome::RejectedStale,
                            };
                        }
                        other_err = Some(Self::refusal_to_driver_err(r));
                        abort_reason = Some(PartialDisableReason::TransportError);
                        break;
                    }
                }
            }
            #[cfg(not(target_arch = "wasm32"))]
            {
                self.defer_disable_shadow = false;
                if self.shadow_staging.is_enabled() && !deferred_refusal_mask.is_empty() {
                    let wk = match tx_kind {
                        TxKind::DisableProtective { .. } => {
                            crate::torque::WriteKind::DisableProtective
                        }
                        _ => crate::torque::WriteKind::DisableRequested,
                    };
                    PendingDisableShadowBatch::merge_batch(
                        &mut self.pending_disable_shadow,
                        PendingDisableShadowBatch {
                            kind: wk,
                            target_mask: deferred_refusal_mask,
                            outcomes: deferred_refusal_outcomes,
                            ambiguous_gap: false,
                            write_done: None,
                        },
                    );
                }
            }
            let mut flush_failed = false;
            if sent_any {
                if let Err(e) = self.flush_can_and_commit() {
                    flush_failed = true;
                    other_err = Some(e.into());
                }
            }
            #[cfg(not(target_arch = "wasm32"))]
            self.flush_pending_disable_shadow_now();
            if sent_any {
                let _ = self.receive_feedback();
            }
            if let Some(reason) = abort_reason {
                let uncertain = if flush_failed && !self.can.flush_is_noop() {
                    sent_this_pass_mask
                } else {
                    0
                };
                if self.disable_dispatched_mask != 0 {
                    return Err(self.build_partial_disable_err(reason, uncertain));
                }
                self.disable_rest_established = false;
                return Err(match reason {
                    PartialDisableReason::Estop | PartialDisableReason::Stopped => {
                        ArmDriverError::EstopInhibited
                    }
                    PartialDisableReason::LeaseLost => ArmDriverError::LeaseFenced,
                    PartialDisableReason::DeadlineExpired => ArmDriverError::DisableRequestExpired,
                    PartialDisableReason::MotionBeyondRest { .. } => {
                        ArmDriverError::TorqueOffWouldDrop
                    }
                    PartialDisableReason::TransportError => {
                        other_err.unwrap_or(ArmDriverError::HardwareFault(
                            "Torque disable transmission failed".into(),
                        ))
                    }
                });
            }
            if flush_failed {
                let uncertain = if !self.can.flush_is_noop() {
                    sent_this_pass_mask
                } else {
                    0
                };
                if self.disable_dispatched_mask != 0 {
                    return Err(self.build_partial_disable_err(
                        PartialDisableReason::TransportError,
                        uncertain,
                    ));
                }
                return Err(other_err.unwrap_or(ArmDriverError::HardwareFault(
                    "Torque disable CAN flush failed".into(),
                )));
            }
            if (self.disable_dispatched_mask & all_mask) == all_mask {
                // Every motor has now received its initial `CMD_DISABLE`: stop holds (`is_enabled = false`)
                // and transition into bounded repeats (`repeats_sent = 1`).
                self.commit_all_disabled_driver_state(pass_start, after_seq);
                if self.motor_ids.is_empty() || DAMIAO_DISABLE_REPEATS <= 1 {
                    if mit_fenced {
                        self.mit_fence = None;
                    }
                    self.disable_dispatched_mask = 0;
                    self.disable_rest_established = false;
                    self.disable_tx_deadline = None;
                    self.disable_lease_gen = None;
                    return Ok(TorqueProgress::Done(TorqueOutcome::Confirmed));
                }
                let now = self.now();
                next_repeat_at = now + DAMIAO_DISABLE_REPEAT_INTERVAL;
                let wake = expiry.map_or(next_repeat_at, |exp| next_repeat_at.min(exp));
                self.torque_hs = Some(DamiaoTorque::Disable {
                    kind,
                    repeats_sent: 1,
                    repeat_pass_mask: 0,
                    next_repeat_at,
                    next_probe_at: now,
                    mit_fenced,
                    start_stop_stamp,
                    started_in_estop,
                    client_park,
                    rest_established,
                });
                return Ok(TorqueProgress::Pending { wake });
            }
            // Still undispatched motors remain: stay Pending with `is_enabled = true` so holds keep running.
            let after = self.now();
            let wake = expiry.map_or(after, |exp| after.min(exp));
            self.torque_hs = Some(DamiaoTorque::Disable {
                kind,
                repeats_sent: 0,
                repeat_pass_mask: 0,
                next_repeat_at,
                next_probe_at,
                mit_fenced,
                start_stop_stamp,
                started_in_estop,
                client_park,
                rest_established,
            });
            return Ok(TorqueProgress::Pending { wake });
        }

        // Phase B: All repeats dispatched -> complete cleanly.
        if repeats_sent >= DAMIAO_DISABLE_REPEATS {
            if mit_fenced {
                self.mit_fence = None;
            }
            self.disable_dispatched_mask = 0;
            self.disable_rest_established = false;
            self.disable_tx_deadline = None;
            self.disable_lease_gen = None;
            // Blocker 1: Protective disables (`Thermal` / `ProcessExit`) NEVER report
            // `TorqueOutcome::Confirmed` solely from accepted TX repeats without fresh
            // post-enable causal `state == 0` confirmation on every axis. Ordinary rest-gated
            // disables retain Damiao's bounded 3-repeat protocol completion while ownership
            // release (`commit_shutdown` / `core_is_confirmed_disarmed`) independently requires
            // `is_hardware_disarmed_confirmed()`.
            let outcome = match kind {
                DisableKind::DisableProtective { .. } => {
                    if self.is_hardware_disarmed_confirmed() {
                        TorqueOutcome::Confirmed
                    } else {
                        TorqueOutcome::Unconfirmed
                    }
                }
                DisableKind::DisableRequested { .. } => {
                    TorqueOutcome::Confirmed
                }
            };
            return Ok(TorqueProgress::Done(outcome));
        }

        // Phase C: Bounded repeat passes (`1 <= repeats_sent < DAMIAO_DISABLE_REPEATS`).
        if repeat_pass_mask == 0 && entry_now < next_repeat_at {
            let wake = expiry.map_or(next_repeat_at, |exp| next_repeat_at.min(exp));
            self.torque_hs = Some(DamiaoTorque::Disable {
                kind,
                repeats_sent,
                repeat_pass_mask,
                next_repeat_at,
                next_probe_at,
                mit_fenced,
                start_stop_stamp,
                started_in_estop,
                client_park,
                rest_established,
            });
            return Ok(TorqueProgress::Pending { wake });
        }
        let tx_kind = Self::disable_tx_kind(kind);
        let step_dl = if is_requested {
            self.disable_step_deadline(expiry)
        } else {
            None
        };
        let mut sent_any = false;
        let mut budget_paused = false;
        let mut wait_evidence = false;
        let mut refused_err = None;
        let mut other_err = None;
        #[cfg(not(target_arch = "wasm32"))]
        let mut deferred_refusal_mask = crate::torque::ChannelMask::EMPTY;
        #[cfg(not(target_arch = "wasm32"))]
        let mut deferred_refusal_outcomes =
            [crate::torque::MotorWriteOutcome::NotTargeted; crate::torque::MAX_CHANNELS];
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.defer_disable_shadow = true;
            self.shadow_disable_pass_sent_mask = repeat_pass_mask;
        }
        for idx in 0..self.motor_ids.len() {
            if idx < 64 && (repeat_pass_mask & (1u64 << idx)) != 0 {
                continue;
            }
            if is_requested && step_dl.is_some_and(|d| d.passed(self.now())) {
                budget_paused = true;
                break;
            }
            match self.send_plain(tx_kind, None, step_dl, false, |d| {
                Some(damiao::build_disable_frame(d.motor_ids[idx]))
            }) {
                Ok(_) => {
                    sent_any = true;
                    if idx < 64 {
                        repeat_pass_mask |= 1u64 << idx;
                    }
                }
                Err(TxRefusal::DeadlinePassed | TxRefusal::Deferred) if is_requested => {
                    budget_paused = true;
                    break;
                }
                Err(TxRefusal::DisableWaitEvidence) => wait_evidence = true,
                Err(r @ (TxRefusal::TorqueOffWouldDrop | TxRefusal::DisableRequestExpired)) => {
                    #[cfg(not(target_arch = "wasm32"))]
                    if idx < crate::torque::MAX_CHANNELS {
                        deferred_refusal_mask.insert(idx as u8);
                        deferred_refusal_outcomes[idx] =
                            crate::torque::MotorWriteOutcome::RejectedStale;
                    }
                    if refused_err.is_none() {
                        refused_err = Some(Self::refusal_to_driver_err(r));
                    }
                }
                Err(r) => {
                    #[cfg(not(target_arch = "wasm32"))]
                    if !matches!(r, TxRefusal::Transport(_)) && idx < crate::torque::MAX_CHANNELS {
                        deferred_refusal_mask.insert(idx as u8);
                        deferred_refusal_outcomes[idx] = match r {
                            TxRefusal::Deferred => crate::torque::MotorWriteOutcome::Deferred,
                            _ => crate::torque::MotorWriteOutcome::RejectedStale,
                        };
                    }
                    tracing::warn!("Repeat disable transmit failed for motor {}: {:?}", self.motor_ids[idx], r);
                    if other_err.is_none() {
                        other_err = Some(Self::refusal_to_driver_err(r));
                    }
                }
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.defer_disable_shadow = false;
            if self.shadow_staging.is_enabled() && !deferred_refusal_mask.is_empty() {
                let wk = match tx_kind {
                    TxKind::DisableProtective { .. } => {
                        crate::torque::WriteKind::DisableProtective
                    }
                    _ => crate::torque::WriteKind::DisableRequested,
                };
                PendingDisableShadowBatch::merge_batch(
                    &mut self.pending_disable_shadow,
                    PendingDisableShadowBatch {
                        kind: wk,
                        target_mask: deferred_refusal_mask,
                        outcomes: deferred_refusal_outcomes,
                        ambiguous_gap: false,
                        write_done: None,
                    },
                );
            }
        }
        let mut flush_ok = true;
        if sent_any {
            if let Err(e) = self.flush_can_and_commit() {
                tracing::warn!("Repeat disable CAN flush failed: {:?}", e);
                flush_ok = false;
                if other_err.is_none() {
                    other_err = Some(e.into());
                }
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        self.flush_pending_disable_shadow_now();
        if sent_any {
            let _ = self.receive_feedback();
        }
        if let Some(e) = refused_err {
            self.disable_rest_established = false;
            return Err(e);
        }
        let pass_complete = (repeat_pass_mask & all_mask) == all_mask && flush_ok;
        if let Some(e) = other_err {
            let can_retry_wb = matches!(e, ArmDriverError::Transport(TransportError::WouldBlock))
                && expiry.is_some_and(|exp| self.now() < exp);
            if can_retry_wb {
                let after = self.now();
                let wake = expiry.map_or(after + Duration::from_millis(20), |exp| {
                    (after + Duration::from_millis(20)).min(exp)
                });
                self.torque_hs = Some(DamiaoTorque::Disable {
                    kind,
                    repeats_sent,
                    repeat_pass_mask,
                    next_repeat_at,
                    next_probe_at,
                    mit_fenced,
                    start_stop_stamp,
                    started_in_estop,
                    client_park,
                    rest_established,
                });
                return Ok(TorqueProgress::Pending { wake });
            }
            self.disable_rest_established = false;
            return Err(e);
        }
        if !pass_complete {
            if budget_paused {
                let after = self.now();
                let wake = expiry.map_or(after, |exp| after.min(exp));
                self.torque_hs = Some(DamiaoTorque::Disable {
                    kind,
                    repeats_sent,
                    repeat_pass_mask,
                    next_repeat_at,
                    next_probe_at,
                    mit_fenced,
                    start_stop_stamp,
                    started_in_estop,
                    client_park,
                    rest_established,
                });
                return Ok(TorqueProgress::Pending { wake });
            }
            if wait_evidence {
                if self.probe_unconfirmed_rest_windows(expiry, &mut next_probe_at)
                    && self.evaluate_rest_pose_gate() == RestPoseGateStatus::RefusedNonRest
                {
                    self.disable_rest_established = false;
                    return Err(ArmDriverError::TorqueOffWouldDrop);
                }
                let after = self.now();
                if let Some(exp) = expiry {
                    if after >= exp {
                        self.disable_rest_established = false;
                        return Err(ArmDriverError::DisableRequestExpired);
                    }
                }
                let target_wake = if !self.last_read_drained || self.awaiting_rx_boundary {
                    after + Duration::from_millis(20)
                } else if self.evaluate_rest_pose_gate() == RestPoseGateStatus::Confirmed {
                    after
                } else {
                    next_probe_at
                };
                let wake = expiry.map_or(target_wake, |exp| target_wake.min(exp));
                self.torque_hs = Some(DamiaoTorque::Disable {
                    kind,
                    repeats_sent,
                    repeat_pass_mask,
                    next_repeat_at,
                    next_probe_at,
                    mit_fenced,
                    start_stop_stamp,
                    started_in_estop,
                    client_park,
                    rest_established,
                });
                return Ok(TorqueProgress::Pending { wake });
            }
            self.disable_rest_established = false;
            return Err(ArmDriverError::HardwareFault(
                "Incomplete Damiao disable repeat pass".into(),
            ));
        }
        repeats_sent = repeats_sent.saturating_add(1);
        repeat_pass_mask = 0;
        if repeats_sent >= DAMIAO_DISABLE_REPEATS {
            if mit_fenced {
                self.mit_fence = None;
            }
            self.disable_dispatched_mask = 0;
            self.disable_rest_established = false;
            self.disable_tx_deadline = None;
            self.disable_lease_gen = None;
            // Blocker 1: Protective disables (`Thermal` / `ProcessExit`) NEVER report
            // `TorqueOutcome::Confirmed` solely from accepted TX repeats without fresh
            // post-enable causal `state == 0` confirmation on every axis. Ordinary rest-gated
            // disables retain Damiao's bounded 3-repeat protocol completion while ownership
            // release (`commit_shutdown` / `core_is_confirmed_disarmed`) independently requires
            // `is_hardware_disarmed_confirmed()`.
            let outcome = match kind {
                DisableKind::DisableProtective { .. } => {
                    if self.is_hardware_disarmed_confirmed() {
                        TorqueOutcome::Confirmed
                    } else {
                        TorqueOutcome::Unconfirmed
                    }
                }
                DisableKind::DisableRequested { .. } => {
                    TorqueOutcome::Confirmed
                }
            };
            return Ok(TorqueProgress::Done(outcome));
        }
        next_repeat_at = self.now() + DAMIAO_DISABLE_REPEAT_INTERVAL;
        let wake = expiry.map_or(next_repeat_at, |exp| next_repeat_at.min(exp));
        self.torque_hs = Some(DamiaoTorque::Disable {
            kind,
            repeats_sent,
            repeat_pass_mask,
            next_repeat_at,
            next_probe_at,
            mit_fenced,
            start_stop_stamp,
            started_in_estop,
            client_park,
            rest_established,
        });
        Ok(TorqueProgress::Pending { wake })
    }

}
