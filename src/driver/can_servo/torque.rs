//! Damiao torque state machine, rest-pose gate, disable initiation, and disarm probing.

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
    pub(in crate::driver::can_servo) fn probe_hardware_disarmed_until_impl(&mut self, wall_deadline: Instant) -> bool {
        let n = self.motor_ids.len();
        if n == 0
            || n > MAX_MODE_JOINTS
            || self.is_enabled
            || self.is_enabling
            || self.torque_transition_pending()
            || self.disable_dispatched_mask != 0
            || self.panic_disable_active
        {
            tracing::warn!(
                "Shutdown disarm proof skipped: active Damiao torque state or transition \
                 (dof={}, is_enabled={}, is_enabling={}, pending={}, disable_mask=0x{:x}, panic_active={})",
                n,
                self.is_enabled,
                self.is_enabling,
                self.torque_transition_pending(),
                self.disable_dispatched_mask,
                self.panic_disable_active,
            );
            return false;
        }
        if self.is_hardware_disarmed_confirmed() {
            return true;
        }
        if !self.poll_tx_allowed() {
            tracing::warn!(
                "Shutdown disarm proof refused: read-only 0xCC polling inhibited by active/unacknowledged MIT deadline fence"
            );
            return false;
        }
        let now_wall = Instant::now();
        let budget = wall_deadline
            .saturating_duration_since(now_wall)
            .min(SHUTDOWN_DISARM_PROOF_WINDOW);
        if budget.is_zero() {
            for i in 0..n {
                let id = self.motor_ids[i];
                let fresh = self
                    .last_feedback_instant
                    .get(i)
                    .copied()
                    .flatten()
                    .is_some_and(|t| self.since(t) <= DAMIAO_FEEDBACK_VALID_WINDOW);
                let st = self.motor_status[i];
                if !fresh || st != 0 || !self.motor_disarmed_after_enable[i] {
                    tracing::warn!(
                        "Shutdown disarm proof deadline expired before probe: Damiao joint J{} (CAN ID 0x{:X}) unconfirmed (fresh={}, state=0x{:X})",
                        i + 1,
                        id,
                        fresh,
                        st
                    );
                }
            }
            return false;
        }

        let drv_deadline = self.now() + budget;
        let hard_wall = now_wall + budget;
        const PROBE_RETRY_INTERVAL: Duration = Duration::from_millis(20);

        let log_unconfirmed_axes = |drv: &Self, boundaries: &[Option<(u64, Instant)>; MAX_MODE_JOINTS]| {
            for (i, boundary) in boundaries.iter().enumerate().take(n) {
                let fresh_reply = boundary.is_some_and(|(p_seq, p_at)| {
                    drv.last_feedback_seq[i] > p_seq
                        && drv.last_feedback_instant[i].is_some_and(|t| {
                            t >= p_at && drv.since(t) <= DAMIAO_FEEDBACK_VALID_WINDOW
                        })
                });
                let id = drv.motor_ids[i];
                if !fresh_reply {
                    tracing::warn!(
                        "Shutdown disarm proof unconfirmed: Damiao joint J{} (CAN ID 0x{:X}) did not reply to read-only 0xCC poll within {} ms",
                        i + 1,
                        id,
                        budget.as_millis()
                    );
                } else if drv.motor_status[i] == 1 {
                    tracing::warn!(
                        "Shutdown disarm proof refused: Damiao joint J{} (CAN ID 0x{:X}) replied enabled (state=1)",
                        i + 1,
                        id
                    );
                } else if drv.motor_status[i] != 0 || !drv.motor_disarmed_after_enable[i] {
                    tracing::warn!(
                        "Shutdown disarm proof unconfirmed: Damiao joint J{} (CAN ID 0x{:X}) state=0x{:X}, disarmed_after_enable={}",
                        i + 1,
                        id,
                        drv.motor_status[i],
                        drv.motor_disarmed_after_enable[i]
                    );
                }
            }
        };

        let mut probe_tx_boundary: [Option<(u64, Instant)>; MAX_MODE_JOINTS] =
            [None; MAX_MODE_JOINTS];

        // RAII guard ensures provisional probe state is ALWAYS cleaned up and fails closed on
        // EVERY exit path (adverse reply, incomplete drain, deadline expiry, or transport unwind).
        struct DisarmProbeGuard<'a, C: CanTransport> {
            drv: &'a mut DamiaoArmDriver<C>,
            committed: bool,
        }
        impl<C: CanTransport> Drop for DisarmProbeGuard<'_, C> {
            fn drop(&mut self) {
                self.drv.disarm_probe_active = false;
                self.drv.disarm_probe_adverse_mask = 0;
                self.drv.disarm_probe_adverse_state = [0; MAX_MODE_JOINTS];
                if !self.committed {
                    self.drv.motor_disarmed_after_enable.fill(false);
                    if !self.drv.last_read_drained {
                        self.drv.awaiting_rx_boundary = true;
                    }
                }
            }
        }

        // RX-only bounded drain helper (NEVER emits autonomous `0xCC` refresh TX while draining).
        let drain_rx_only = |drv: &mut Self, drv_dl: Instant, wall_dl: Instant| -> bool {
            drv.last_read_drained = false;
            loop {
                if drv.now() >= drv_dl || Instant::now() >= wall_dl {
                    return false;
                }
                let seq_before = drv.rx_seq;
                if drv.drain_rx_frames_only(Some(Deadline { at: drv_dl }), 64) {
                    return true;
                }
                // Stop looping if transport returned an error without advancing `rx_seq`.
                if drv.rx_seq == seq_before {
                    return false;
                }
            }
        };

        let mut guard = DisarmProbeGuard {
            drv: self,
            committed: false,
        };
        guard.drv.disarm_probe_adverse_mask = 0;
        guard.drv.disarm_probe_adverse_state = [0; MAX_MODE_JOINTS];
        guard.drv.disarm_probe_active = true;

        // ALWAYS fence and drain RX-only to a fresh observed-empty RX boundary IMMEDIATELY before
        // transmitting `0xCC` polls — NEVER reuse `last_read_drained` from an earlier tick and
        // NEVER emit autonomous refresh TX during the pre-poll drain.
        guard.drv.awaiting_rx_boundary = true;
        if !drain_rx_only(guard.drv, drv_deadline, hard_wall) {
            tracing::warn!(
                "Shutdown disarm proof refused: could not establish empty RX boundary before deadline"
            );
            log_unconfirmed_axes(guard.drv, &probe_tx_boundary);
            return false;
        }
        guard.drv.awaiting_rx_boundary = false;

        let mut next_tx_at = guard.drv.now();
        let mut first_pass = true;

        loop {
            let now_drv = guard.drv.now();
            let now_w = Instant::now();
            if now_drv >= drv_deadline || now_w >= hard_wall {
                break;
            }

            if first_pass || now_drv >= next_tx_at {
                first_pass = false;
                let tx_dl = Deadline { at: drv_deadline };
                let mut sent_any = false;
                let mut newly_armed_mask = 0u32;
                let clean_rx_boundary = !guard.drv.awaiting_rx_boundary;
                for i in 0..n {
                    let already_confirmed = probe_tx_boundary[i].is_some_and(|(p_seq, p_at)| {
                        guard.drv.last_feedback_seq[i] > p_seq
                            && guard.drv.last_feedback_instant[i].is_some_and(|t| {
                                t >= p_at && guard.drv.since(t) <= DAMIAO_FEEDBACK_VALID_WINDOW / 2
                            })
                            && guard.drv.motor_status[i] == 0
                            && (guard.drv.disarm_probe_adverse_mask & (1u32 << i)) == 0
                    });
                    if already_confirmed {
                        continue;
                    }
                    let id = guard.drv.motor_ids[i];
                    match guard.drv.send_plain(
                        TxKind::DisableProbe,
                        None,
                        Some(tx_dl),
                        false,
                        |_| Some(damiao::build_refresh_frame(id)),
                    ) {
                        Ok(_) => {
                            sent_any = true;
                            if clean_rx_boundary && probe_tx_boundary[i].is_none() {
                                let (seq_mark, at_mark) =
                                    guard.drv.last_tx_mark.unwrap_or((guard.drv.rx_seq, guard.drv.now()));
                                probe_tx_boundary[i] = Some((seq_mark, at_mark));
                                newly_armed_mask |= 1u32 << i;
                            }
                        }
                        Err(TxRefusal::DeadlinePassed) => break,
                        Err(r) => {
                            tracing::warn!(
                                "Shutdown disarm proof 0xCC TX failed for Damiao joint J{} (CAN ID 0x{:X}): {:?}",
                                i + 1,
                                id,
                                r
                            );
                        }
                    }
                }
                if sent_any {
                    if let Err(e) = guard.drv.can.flush() {
                        tracing::warn!("Shutdown disarm proof CAN flush failed: {e}");
                        for (i, slot) in probe_tx_boundary.iter_mut().enumerate().take(n) {
                            if (newly_armed_mask & (1u32 << i)) != 0 {
                                *slot = None;
                            }
                        }
                    }
                }
                next_tx_at = guard.drv.now() + PROBE_RETRY_INTERVAL;
            }

            // Drain RX-only through a fresh observed-empty queue boundary (bounded by
            // `drv_deadline` / `hard_wall`) so no adverse reply past frame 64 is left queued.
            let drained = drain_rx_only(guard.drv, drv_deadline, hard_wall);

            // Preserve EVERY adverse (enabled/faulted) axis observed across all drained RX batches
            // (including untimestamped/unmappable adverse frames). `DisarmProbeGuard::drop`
            // guarantees full fail-closed invalidation on return.
            if guard.drv.disarm_probe_adverse_mask != 0 {
                for i in 0..n {
                    if (guard.drv.disarm_probe_adverse_mask & (1u32 << i)) == 0 {
                        continue;
                    }
                    let bad_st = guard.drv.disarm_probe_adverse_state[i];
                    guard.drv.motor_status[i] = bad_st;
                    let id = guard.drv.motor_ids[i];
                    if bad_st == 1 {
                        tracing::warn!(
                            "Shutdown disarm proof refused: Damiao joint J{} (CAN ID 0x{:X}) replied enabled (state=1)",
                            i + 1,
                            id
                        );
                    } else {
                        tracing::warn!(
                            "Shutdown disarm proof refused: Damiao joint J{} (CAN ID 0x{:X}) replied non-disabled (state=0x{:X})",
                            i + 1,
                            id,
                            bad_st
                        );
                    }
                }
                return false;
            }

            let mut all_post_probe_disarmed = drained && guard.drv.last_read_drained;
            for (i, boundary) in probe_tx_boundary.iter().enumerate().take(n) {
                let fresh_reply = boundary.is_some_and(|(p_seq, p_at)| {
                    guard.drv.last_feedback_seq[i] > p_seq
                        && guard.drv.last_feedback_instant[i].is_some_and(|t| {
                            t >= p_at && guard.drv.since(t) <= DAMIAO_FEEDBACK_VALID_WINDOW
                        })
                });
                if !fresh_reply {
                    all_post_probe_disarmed = false;
                    continue;
                }
                let after_enable = match guard.drv.motor_last_enable_tx[i] {
                    Some((en_seq, en_at)) => {
                        guard.drv.last_feedback_seq[i] > en_seq
                            && guard.drv.last_feedback_instant[i].is_some_and(|t| t >= en_at)
                    }
                    None => true,
                };
                if !after_enable {
                    all_post_probe_disarmed = false;
                    continue;
                }
                let st = guard.drv.motor_status[i];
                if st == 1 {
                    tracing::warn!(
                        "Shutdown disarm proof refused: Damiao joint J{} (CAN ID 0x{:X}) replied enabled (state=1)",
                        i + 1,
                        guard.drv.motor_ids[i]
                    );
                    return false;
                }
                if st != 0 {
                    tracing::warn!(
                        "Shutdown disarm proof refused: Damiao joint J{} (CAN ID 0x{:X}) replied non-disabled (state=0x{:X})",
                        i + 1,
                        guard.drv.motor_ids[i],
                        st
                    );
                    return false;
                }
            }
            // Enforce caller wall and driver deadlines BEFORE committing provisional proof.
            if guard.drv.now() >= drv_deadline || Instant::now() >= hard_wall {
                tracing::warn!(
                    "Shutdown disarm proof refused: deadline ({} ms) expired before post-poll confirmation completed",
                    budget.as_millis()
                );
                break;
            }
            if all_post_probe_disarmed {
                // Commit provisional proof ONLY after every axis passed fresh causal post-poll
                // `state == 0` checks through an observed-empty RX boundary with zero adverse frames.
                for i in 0..n {
                    guard.drv.motor_disarmed_after_enable[i] = true;
                }
                if guard.drv.is_hardware_disarmed_confirmed() {
                    guard.committed = true;
                    return true;
                }
                guard.drv.motor_disarmed_after_enable.fill(false);
            }

            let rem_drv = drv_deadline.saturating_duration_since(guard.drv.now());
            let rem_wall = hard_wall.saturating_duration_since(Instant::now());
            let rem = rem_drv.min(rem_wall);
            if rem.is_zero() {
                break;
            }
            guard.drv.pause(rem.min(Duration::from_millis(2)));
        }

        log_unconfirmed_axes(guard.drv, &probe_tx_boundary);
        false
    }

    #[inline]
    pub(in crate::driver::can_servo) fn is_disable_tx_kind(kind: TxKind) -> bool {
        matches!(
            kind,
            TxKind::DisableRequested { .. }
                | TxKind::ClientParkDisable { .. }
                | TxKind::DisableProtective { .. }
                | TxKind::DisableProbe
        )
    }

    #[inline]
    pub(in crate::driver::can_servo) fn disable_tx_kind(kind: DisableKind) -> TxKind {
        match kind {
            DisableKind::DisableRequested { started_at } => {
                TxKind::DisableRequested { started_at }
            }
            DisableKind::DisableProtective { reason } => TxKind::DisableProtective { reason },
        }
    }

    #[inline]
    pub(in crate::driver::can_servo) fn is_stopped_for_disable(&self) -> bool {
        self.stopped_disable_gate
            .as_ref()
            .is_some_and(StoppedDisableGate::is_safety_stopped)
            || self.is_stop_latched()
            || self.is_stop_pending()
    }

    /// Zero-allocation evaluation of the Damiao K=3 rest-pose evidence gate (S2/L1).
    ///
    /// Covers arm joints (`gate.arm_joint_indices()`, J1–J6) ONLY, matching the supervisor rest
    /// gate and the Feetech/RobStride/FashionStar drivers: K=3 fresh (`<= 100 ms`) valid samples
    /// across 3 distinct control releases within `rest_pose_tolerance_rad` of `rest_positions[i]`,
    /// preserving causal RX provenance (`pose_feedback_instant`, `feedback_queue_bound`, and
    /// observed empty-queue boundary). J7 (gripper) never blocks a rest-pose disable.
    pub(in crate::driver::can_servo) fn evaluate_rest_pose_gate(&self) -> RestPoseGateStatus {
        let Some(ref gate) = self.stopped_disable_gate else {
            return RestPoseGateStatus::RefusedNonRest;
        };
        if !gate.has_resting_pose {
            return RestPoseGateStatus::RefusedNonRest;
        }
        let arm_joints = gate.arm_joint_indices();
        if arm_joints.is_empty() || self.motor_ids.is_empty() {
            return RestPoseGateStatus::RefusedNonRest;
        }
        let now = self.now();
        let caps = self.capabilities();
        let drain_incomplete = self.awaiting_rx_boundary || !self.last_read_drained;
        let mut missing_fresh = false;
        let mut window_incomplete = false;
        for &i in arm_joints {
            if i >= self.motor_ids.len() {
                return RestPoseGateStatus::RefusedNonRest;
            }
            let Some(s) = self.cached_states.get(i) else {
                return RestPoseGateStatus::RefusedNonRest;
            };
            let fresh_pose = self
                .pose_feedback_instant
                .get(i)
                .copied()
                .flatten()
                .is_some_and(|t| now.saturating_duration_since(t) <= STOPPED_DISABLE_FEEDBACK_MAX_AGE)
                && self
                    .feedback_queue_bound
                    .get(i)
                    .copied()
                    .flatten()
                    .is_some_and(|b| now.saturating_duration_since(b) <= STOPPED_DISABLE_FEEDBACK_MAX_AGE);
            if fresh_pose {
                if !joint_state_valid(s, caps) || !gate.joint_at_rest(i, s.position) {
                    return RestPoseGateStatus::RefusedNonRest;
                }
            } else {
                missing_fresh = true;
            }
            if !self
                .rest_pose_window
                .get(i)
                .is_some_and(|w| w.is_confirmed(now, STOPPED_DISABLE_FEEDBACK_MAX_AGE))
            {
                window_incomplete = true;
            }
        }
        if drain_incomplete {
            RestPoseGateStatus::PendingDrainOrWindow
        } else if missing_fresh {
            RestPoseGateStatus::StaleOrMissing
        } else if window_incomplete {
            RestPoseGateStatus::PendingDrainOrWindow
        } else {
            RestPoseGateStatus::Confirmed
        }
    }

    /// Zero-allocation evaluation of fresh rest-pose compliance on arm joints that have NOT YET
    /// been dispatched a `CMD_DISABLE` frame during an active multi-tick `DisableRequested`
    /// sequence whose initial K=3 rest evidence was already established before the first disable.
    pub(in crate::driver::can_servo) fn evaluate_remaining_arm_joints_rest(&self) -> RemainingRestStatus {
        let Some(ref gate) = self.stopped_disable_gate else {
            return RemainingRestStatus::MovedBeyondRest { joint: 0 };
        };
        if !gate.has_resting_pose {
            return RemainingRestStatus::MovedBeyondRest { joint: 0 };
        }
        let arm_joints = gate.arm_joint_indices();
        if arm_joints.is_empty() || self.motor_ids.is_empty() {
            return RemainingRestStatus::MovedBeyondRest { joint: 0 };
        }
        let now = self.now();
        let caps = self.capabilities();
        let drain_incomplete = self.awaiting_rx_boundary || !self.last_read_drained;
        let mut missing_fresh = false;
        for &i in arm_joints {
            if i < 64 && (self.disable_dispatched_mask & (1u64 << i)) != 0 {
                continue;
            }
            if i >= self.motor_ids.len() {
                return RemainingRestStatus::MovedBeyondRest { joint: i };
            }
            let Some(s) = self.cached_states.get(i) else {
                return RemainingRestStatus::MovedBeyondRest { joint: i };
            };
            let fresh_pose = self
                .pose_feedback_instant
                .get(i)
                .copied()
                .flatten()
                .is_some_and(|t| now.saturating_duration_since(t) <= STOPPED_DISABLE_FEEDBACK_MAX_AGE)
                && self
                    .feedback_queue_bound
                    .get(i)
                    .copied()
                    .flatten()
                    .is_some_and(|b| now.saturating_duration_since(b) <= STOPPED_DISABLE_FEEDBACK_MAX_AGE);
            if fresh_pose {
                if !joint_state_valid(s, caps) || !gate.joint_at_rest(i, s.position) {
                    return RemainingRestStatus::MovedBeyondRest { joint: i };
                }
            } else {
                missing_fresh = true;
            }
        }
        if drain_incomplete || missing_fresh {
            RemainingRestStatus::NeedFreshFeedback
        } else {
            RemainingRestStatus::AllAtRest
        }
    }

    /// True when every motor currently reports `motor_status == 0` (disabled).
    pub(in crate::driver::can_servo) fn all_motors_confirmed_disabled(&self) -> bool {
        !self.motor_ids.is_empty()
            && self.motor_status.len() >= self.motor_ids.len()
            && (0..self.motor_ids.len()).all(|i| self.motor_status[i] == 0)
    }

    /// True when every configured arm joint (or all motors when no arm subset is configured) has
    /// already confirmed `motor_status == 0` (disabled).
    pub(in crate::driver::can_servo) fn all_arm_joints_confirmed_disabled(&self) -> bool {
        if self.motor_ids.is_empty() {
            return true;
        }
        let Some(gate) = self.stopped_disable_gate.as_ref() else {
            return self.all_motors_confirmed_disabled();
        };
        if gate.arm_joints_len == 0 {
            return self.all_motors_confirmed_disabled();
        }
        for idx in 0..gate.arm_joints_len.min(gate.arm_joints.len()) {
            let j = gate.arm_joints[idx];
            if j < self.motor_ids.len() && self.motor_status.get(j).copied() != Some(0) {
                return false;
            }
        }
        true
    }

    /// Evaluates the K=3 rest-pose gate and single 1 s request lifetime at send time for EVERY
    /// `DisableRequested` in every safety state (operational and stopped alike). Once K=3 rest
    /// evidence has been established before the first disable frame (`disable_rest_established`),
    /// subsequent multi-tick passes check fresh no-motion-beyond-rest on not-yet-disabled arm
    /// joints without re-requiring a fresh K=3 window on already-disabled joints. Zero heap
    /// allocations.
    pub(in crate::driver::can_servo) fn check_stopped_disable_requested(&self, started_at: Instant) -> Result<(), TxRefusal> {
        if self.now().saturating_duration_since(started_at) >= STOPPED_DISABLE_REQUEST_LIFETIME {
            return Err(TxRefusal::DisableRequestExpired);
        }
        if self.disable_rest_established {
            let n = self.motor_ids.len().min(64);
            let all_mask = if n >= 64 { u64::MAX } else { (1u64 << n) - 1 };
            let all_dispatched = (self.disable_dispatched_mask & all_mask) == all_mask;
            if !all_dispatched {
                return match self.evaluate_remaining_arm_joints_rest() {
                    RemainingRestStatus::MovedBeyondRest { .. } => {
                        Err(TxRefusal::TorqueOffWouldDrop)
                    }
                    RemainingRestStatus::NeedFreshFeedback => Err(TxRefusal::DisableWaitEvidence),
                    RemainingRestStatus::AllAtRest => Ok(()),
                };
            }
            if self.all_arm_joints_confirmed_disabled() {
                return Ok(());
            }
        }
        match self.evaluate_rest_pose_gate() {
            RestPoseGateStatus::Confirmed => Ok(()),
            // M3: A drained bus with stale or missing evidence defers into `AwaitRestGate` and
            // runs the probe loop within the 1 s lifetime; only a real non-rest pose (or 1 s
            // expiry checked above) refuses immediately.
            RestPoseGateStatus::PendingDrainOrWindow | RestPoseGateStatus::StaleOrMissing => {
                Err(TxRefusal::DisableWaitEvidence)
            }
            RestPoseGateStatus::RefusedNonRest => Err(TxRefusal::TorqueOffWouldDrop),
        }
    }

    #[inline]
    pub(in crate::driver::can_servo) fn stop_gate_for_kind(&self, kind: TxKind) -> Result<bool, TxRefusal> {
        let pending = self.is_stop_pending();
        let latched = self.is_stop_latched();
        match kind {
            TxKind::DisableRequested { started_at }
            | TxKind::MitRecoveryDisable { started_at } => {
                self.check_stopped_disable_requested(started_at)?;
            }
            TxKind::ClientParkDisable { started_at } => {
                if pending || latched {
                    return Err(TxRefusal::StopInhibited);
                }
                self.check_stopped_disable_requested(started_at)?;
            }
            TxKind::DisableProtective { .. } | TxKind::DisableProbe => {}
            TxKind::Refresh | TxKind::RollbackHold(_) | TxKind::MeasuredHold(_) | TxKind::ClearFault => {
                if pending {
                    return Err(TxRefusal::StopInhibited);
                }
            }
            TxKind::Target
            | TxKind::Hold
            | TxKind::ModeWrite
            | TxKind::Recovery
            | TxKind::Handshake => {
                if pending || latched {
                    return Err(TxRefusal::StopInhibited);
                }
            }
        }
        Ok(latched)
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[inline]
    pub(in crate::driver::can_servo) fn flush_pending_disable_shadow_now(&mut self) {
        if self.pending_disable_shadow.is_some() {
            let now = self.now();
            PendingDisableShadowBatch::flush_into(
                &mut self.pending_disable_shadow,
                &mut self.shadow_staging,
                now,
            );
        }
    }

    /// Disable: frames to every motor at once (a pending enable is abandoned), then the
    /// confirmation window. Safety torque-off is exempt from the motion TX frame budget.
    pub(in crate::driver::can_servo) fn begin_disable(&mut self, kind: DisableKind) -> Result<TorqueProgress, ArmDriverError> {
        self.begin_disable_budgeted(kind, None)
    }

    pub(in crate::driver::can_servo) fn begin_disable_budgeted(
        &mut self,
        kind: DisableKind,
        max_rx_frames: Option<u16>,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.begin_disable_inner(kind, max_rx_frames, false, None)
    }

    pub(in crate::driver::can_servo) fn begin_disable_inner(
        &mut self,
        kind: DisableKind,
        max_rx_frames: Option<u16>,
        client_park: bool,
        expected_stop_gen: Option<u64>,
    ) -> Result<TorqueProgress, ArmDriverError> {
        let start_stop_stamp = self.current_stop_stamp();
        let started_in_estop = self.is_stop_latched();
        let expected_stamp = if client_park {
            let mut stamp = start_stop_stamp;
            if let Some(gen) = expected_stop_gen {
                stamp.stop_gen = gen;
            }
            self.strict_stop_check(Some(stamp))?;
            Some(stamp)
        } else {
            None
        };
        if !client_park {
            // Never downgrade or replace an active protective or already-dispatched hardware
            // disable with a new ordinary DisableRequested.
            if matches!(kind, DisableKind::DisableRequested { .. }) {
                let keep_existing = matches!(
                    self.torque_hs,
                    Some(
                        DamiaoTorque::Disable {
                            kind: DisableKind::DisableProtective { .. },
                            ..
                        } | DamiaoTorque::AwaitRestGate {
                            kind: DisableKind::DisableProtective { .. },
                            ..
                        }
                    )
                ) || (self.disable_dispatched_mask != 0
                    && matches!(
                        self.torque_hs,
                        Some(DamiaoTorque::Disable { .. } | DamiaoTorque::AwaitRestGate { .. })
                    ));
                if keep_existing {
                    return Ok(TorqueProgress::Pending { wake: self.now() });
                }
            }
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
        }
        if self.disable_dispatched_mask == 0 {
            self.disable_rest_established = false;
        }
        let prev_awaiting_rx_boundary = self.awaiting_rx_boundary;
        // E2/L1: Drain RX before checking Rule A′ in `send` so the send-time check sees the
        // freshest bus feedback. If `max_rx_frames == Some(0)` without a completed prior drain,
        // or if the drain exhausts without reaching an empty queue, `awaiting_rx_boundary` becomes
        // true and `send(DisableRequested | ClientParkDisable)` returns `DisableWaitEvidence`,
        // holding the request pending (`DamiaoTorque::AwaitRestGate`) to retry on subsequent
        // releases inside the 1 s lifetime (0 `CMD_DISABLE` frames). Protective disables bypass Rule A′.
        let new_awaiting_rx_boundary = match max_rx_frames {
            Some(0) if matches!(kind, DisableKind::DisableProtective { .. }) => {
                // Protective disables bypass Rule A' and must run safety TX even at 0 RX.
                self.awaiting_rx_boundary
            }
            Some(0) => self.awaiting_rx_boundary || !self.last_read_drained,
            Some(limit) => {
                self.last_poll_rx = None;
                self.rx_frame_limit = Some(self.clamp_release_rx_limit(limit));
                let drained = self.drain_rx_before_command_limit(usize::from(limit));
                self.rx_frame_limit = None;
                !drained
            }
            None => !self.drain_rx_before_command(),
        };
        if !client_park {
            self.awaiting_rx_boundary = new_awaiting_rx_boundary;
        }
        self.disable_rest_established = false;
        self.send_disable_frames(
            kind,
            prev_awaiting_rx_boundary,
            new_awaiting_rx_boundary,
            client_park,
            expected_stamp,
            start_stop_stamp,
            started_in_estop,
        )
    }

    /// Computes the effective per-tick step deadline for `DisableRequested` frames from the
    /// normal per-tick TX release cap (`tx_release_deadline`), service budget (`rx_deadline`),
    /// request lifetime (`expiry`), and disable authority deadline (`disable_tx_deadline`).
    /// Does NOT clamp to `DAMIAO_TX_MAX_DURATION` (5 ms deferral window), keeping `DisableRequested`
    /// able to beat relatching deferral loops while still respecting normal per-tick release caps.
    #[inline]
    pub(in crate::driver::can_servo) fn disable_step_deadline(&self, expiry: Option<Instant>) -> Option<Deadline> {
        let mut at: Option<Instant> = self.tx_release_deadline;
        if let Some(rx_dl) = self.rx_deadline {
            at = Some(at.map_or(rx_dl, |a| a.min(rx_dl)));
        }
        if let Some(exp) = expiry {
            at = Some(at.map_or(exp, |a| a.min(exp)));
        }
        if let Some(auth_dl) = self.disable_tx_deadline {
            at = Some(at.map_or(auth_dl, |a| a.min(auth_dl)));
        }
        at.map(|at| Deadline { at })
    }

    /// Commits driver-side fully-dispatched torque-off state once every motor has received its
    /// initial `CMD_DISABLE` frame (or on a protective cut).
    pub(in crate::driver::can_servo) fn commit_all_disabled_driver_state(&mut self, cmd_start: Instant, after_seq: u64) {
        self.is_enabled = false;
        self.motor_energized_mask = 0;
        self.motor_last_ctrl_tx = [None; MAX_MODE_JOINTS];
        self.pending_rearm = true;
        self.pos_force_verified = false;
        self.slot_in_unchanged_hold = [false; MAX_MODE_JOINTS * 2];
        self.pf_next_intent_is_hold = false;
        self.disable_tx_deadline = None;
        self.disable_lease_gen = None;
        self.clear_cached_targets();
        self.last_enable_command = Some(cmd_start);
        self.last_enable_command_rx_seq = Some(after_seq);
        self.last_command_was_disable = true;
        self.last_query_instant = Some(self.now());
    }

    pub(in crate::driver::can_servo) fn build_partial_disable_err(
        &mut self,
        reason: PartialDisableReason,
        uncertain_mask: u64,
    ) -> ArmDriverError {
        let n = self.motor_ids.len().min(64);
        let all_mask = if n >= 64 { u64::MAX } else { (1u64 << n) - 1 };
        let applied_mask = self.disable_dispatched_mask & !uncertain_mask & all_mask;
        let uncertain = uncertain_mask & all_mask;
        let remaining_mask = all_mask & !applied_mask & !uncertain;
        self.torque_hs = None;
        self.disable_rest_established = false;
        self.disable_tx_deadline = None;
        self.disable_lease_gen = None;
        ArmDriverError::PartialDisable {
            reason,
            applied_mask,
            uncertain_mask: uncertain,
            remaining_mask,
            dof: n.min(u8::MAX as usize) as u8,
        }
    }

    pub(in crate::driver::can_servo) fn poll_torque_inner(&mut self, deadline: Instant, max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
        if !self.control_release_managed {
            self.control_release_id = self.control_release_id.wrapping_add(1).max(1);
        }
        self.ensure_release_tx_tick();
        self.in_poll_torque = true;
        self.last_poll_rx = None;
        self.rx_deadline = Some(deadline);
        let effective_rx_limit = self.clamp_release_rx_limit(max_rx_frames);
        self.rx_frame_limit = Some(effective_rx_limit);
        let is_enable_hs = matches!(
            self.torque_hs,
            Some(
                DamiaoTorque::RangeCheck(_)
                    | DamiaoTorque::DrainBeforeEnable(_)
                    | DamiaoTorque::Freshen(_)
                    | DamiaoTorque::RefreshBeforeRestore { .. }
                    | DamiaoTorque::Restore { .. }
                    | DamiaoTorque::Readback(_)
                    | DamiaoTorque::PreEnableRefresh(_)
                    | DamiaoTorque::Enable { .. }
            )
        );
        let is_disable_pre_all = matches!(
            self.torque_hs,
            Some(DamiaoTorque::AwaitRestGate { .. } | DamiaoTorque::Disable { repeats_sent: 0, .. })
        );
        let mut r = match self.torque_hs.take() {
            None => Err(ArmDriverError::ProtocolStatic("no torque transition pending")),
            Some(DamiaoTorque::RangeCheck(rc)) => self.poll_range_check(rc),
            Some(DamiaoTorque::DrainBeforeEnable(d)) => self.poll_drain_before_enable(d),
            Some(DamiaoTorque::Freshen(f)) => self.poll_freshen(f),
            Some(DamiaoTorque::RefreshBeforeRestore { refresh, stray, stray_len }) => {
                self.poll_refresh_before_restore(refresh, stray, stray_len)
            }
            Some(DamiaoTorque::Restore { write, pf_refreshed }) => self.poll_restore(write, pf_refreshed),
            Some(DamiaoTorque::Readback(r)) => self.poll_readback(r),
            Some(DamiaoTorque::PreEnableRefresh(refresh)) => self.poll_pre_enable_refresh(refresh),
            Some(DamiaoTorque::Enable {
                window,
                unsent_mask,
                tx_cursor,
                j7_prioritize,
                pf_refresh,
                pf_refresh_count,
            }) => self.poll_enable(
                window,
                unsent_mask,
                tx_cursor,
                j7_prioritize,
                pf_refresh,
                pf_refresh_count,
            ),
            Some(DamiaoTorque::AwaitRestGate {
                kind,
                next_probe_at,
                client_park,
                expected_stamp,
                start_stop_stamp,
                started_in_estop,
            }) => self.poll_await_rest_gate(
                kind,
                next_probe_at,
                client_park,
                expected_stamp,
                start_stop_stamp,
                started_in_estop,
            ),
            Some(DamiaoTorque::Disable {
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
            }) => self.poll_disable(
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
            ),
        };
        // Proactive watchdog refresh AFTER phase TX in spare tick budget (15 ms cadence), plus
        // fail-closed abort BEFORE Damiao TIMEOUT (~100 ms) if an enable squeeze starves any
        // already-enabled motor >= 60 ms.
        if self.motor_energized_mask != 0 {
            if is_enable_hs {
                match r {
                    Ok(TorqueProgress::Pending { wake }) => {
                        match self.service_energized_watchdog_holds(
                            WATCHDOG_HOLD_PROACTIVE_INTERVAL,
                            None,
                            0,
                            true,
                        ) {
                            Ok(_) => {
                                // During Window 1 Step 2 (before Step 5 output), do NOT pull the
                                // optional transition wake down to pre-Step-5 `next_watchdog_hold_wake`:
                                // `ReleaseEngine::step` refreshes `urgent_wake` live AFTER Step 5.
                                let tightened = if self.output_step_pending_in_release
                                    && self.in_active_control_release()
                                {
                                    wake
                                } else {
                                    self.next_watchdog_hold_wake()
                                        .map_or(wake, |w| wake.min(w))
                                };
                                r = Ok(TorqueProgress::Pending { wake: tightened });
                            }
                            Err(e) => {
                                r = Err(e);
                            }
                        }
                    }
                    // Skip duplicate pre-Step-5 holds ONLY when a subsequent Window 1 Step 5
                    // output step in THIS release still owns watchdog servicing; Window 2
                    // confirmations and off-tick callers have no Step 5 and must service inline.
                    Ok(TorqueProgress::Done(TorqueOutcome::Confirmed))
                        if !(self.output_step_pending_in_release && self.in_active_control_release()) =>
                    {
                        let _ = self.service_energized_watchdog_holds(
                            WATCHDOG_HOLD_PROACTIVE_INTERVAL,
                            None,
                            0,
                            false,
                        );
                    }
                    _ => {}
                }
            } else if is_disable_pre_all
                && self.mit_fence_trip.is_none()
                && ((self.last_read_drained && !self.awaiting_rx_boundary)
                    || effective_rx_limit == 0)
            {
                if let Ok(TorqueProgress::Pending { wake }) = r {
                    let _ = self.service_energized_watchdog_holds(
                        WATCHDOG_DISABLE_FALLBACK_INTERVAL,
                        None,
                        0,
                        false,
                    );
                    let tightened = if self.output_step_pending_in_release
                        && self.in_active_control_release()
                    {
                        wake
                    } else {
                        self.next_watchdog_hold_wake()
                            .map_or(wake, |w| wake.min(w))
                    };
                    r = Ok(TorqueProgress::Pending { wake: tightened });
                }
            }
        }
        self.in_poll_torque = false;
        self.rx_deadline = None;
        self.rx_frame_limit = None;
        r
    }
}

pub(super) mod disable;
pub(super) mod enable;
pub(super) mod enable_freshen;

/// Phase of a pending Damiao torque transition.
#[derive(Debug)]
#[allow(clippy::large_enum_variant)] // inline ModeReadback avoids RT heap allocation
pub(in crate::driver::can_servo) enum DamiaoTorque {
    /// Budgeted legacy enable retrying mapping-range readback for joints still `Unverified` at
    /// startup before draining/restoring/enabling (zero heap allocations on the control release).
    RangeCheck(RangeCheckPhase),
    /// Budgeted enable waiting for an empty RX queue boundary before mode restore / enable TX.
    DrainBeforeEnable(Deadline),
    /// Enable, initial bounded FRESHEN phase: polls every joint with read-only `0xCC` status
    /// refreshes and waits for a fresh causal reply from every joint before CTRL_MODE restore,
    /// readback, pre-enable hold, and `CMD_ENABLE`.
    Freshen(EnableFreshen),
    /// Enable, pre-restore refresh: waiting for a fresh J7 0xCC feedback reply before writing CTRL_MODE.
    RefreshBeforeRestore {
        refresh: FeedbackRefresh,
        stray: [u8; MAX_MODE_JOINTS],
        stray_len: u8,
    },
    /// Enable, first phase: CTRL_MODE writes restoring the base mode, awaiting their replies.
    Restore { write: ModeWrite, pf_refreshed: bool },
    /// Enable, second phase: CTRL_MODE readback of the position-force gripper.
    Readback(ModeReadback),
    /// Enable, pre-enable refresh: CTRL_MODE 4 confirmed, waiting for a fresh J7 0xCC reply
    /// before transmitting the capped 0x307 pre-enable hold.
    PreEnableRefresh(FeedbackRefresh),
    /// Enable frames sent (or still queued in `unsent_mask` across TX-budgeted polls), awaiting
    /// fresh state-1 feedback from every motor. Reserves a bounded causal J7 `0xCC` refresh slot
    /// and fair J7 TX priority so a carried or retried J7 enable neither starves nor goes stale.
    Enable {
        window: ConfirmWindow,
        unsent_mask: u32,
        tx_cursor: u8,
        j7_prioritize: bool,
        pf_refresh: Option<FeedbackRefresh>,
        pf_refresh_count: u8,
    },
    /// Stopped disable waiting for an exhausted RX drain or incomplete K=3 rest-pose window
    /// to complete inside the 1 s request lifetime before transmitting any `CMD_DISABLE` (L1).
    AwaitRestGate {
        kind: DisableKind,
        next_probe_at: Instant,
        client_park: bool,
        expected_stamp: Option<StopStamp>,
        start_stop_stamp: StopStamp,
        started_in_estop: bool,
    },
    /// Accepted disable dispatching across as many ticks as the per-tick TX budget needs
    /// (`repeats_sent == 0` for the initial pass while still-enabled joints keep holding,
    /// `repeats_sent >= 1` for bounded `DAMIAO_DISABLE_REPEATS` passes and status confirmation).
    Disable {
        kind: DisableKind,
        repeats_sent: u8,
        repeat_pass_mask: u64,
        next_repeat_at: Instant,
        next_probe_at: Instant,
        mit_fenced: bool,
        start_stop_stamp: StopStamp,
        started_in_estop: bool,
        client_park: bool,
        rest_established: bool,
    },
}

/// Inline zero-allocation state for ticked mapping-range verification of joints that were
/// `RangeCheckStatus::Unverified` at daemon startup (legacy budgeted enable path).
#[derive(Debug)]
pub(in crate::driver::can_servo) struct RangeCheckPhase {
    pub(in crate::driver::can_servo) pending_mask: u32,
    pub(in crate::driver::can_servo) active_joint: Option<u8>,
    pub(in crate::driver::can_servo) boundary: Option<u64>,
    pub(in crate::driver::can_servo) sent_rids: u8,
    pub(in crate::driver::can_servo) deadline: Deadline,
}

/// Inline zero-allocation state for enable's initial bounded `0xCC` FRESHEN phase.
#[derive(Debug)]
pub(in crate::driver::can_servo) struct EnableFreshen {
    pub(in crate::driver::can_servo) target_mask: u32,
    pub(in crate::driver::can_servo) sent_mask: u32,
    pub(in crate::driver::can_servo) confirmed_mask: u32,
    pub(in crate::driver::can_servo) started: Instant,
    pub(in crate::driver::can_servo) base_rx_seq: u64,
    pub(in crate::driver::can_servo) tx_boundary_delta: [u32; MAX_MODE_JOINTS],
    pub(in crate::driver::can_servo) tx_offset_ns: [u32; MAX_MODE_JOINTS],
    pub(in crate::driver::can_servo) next_retry_at: Option<Instant>,
    pub(in crate::driver::can_servo) deadline: Deadline,
}

impl EnableFreshen {
    pub(in crate::driver::can_servo) const UNSET_NS: u32 = u32::MAX;

    pub(in crate::driver::can_servo) fn tx_boundary(&self, j: usize) -> u64 {
        self.base_rx_seq.saturating_add(u64::from(self.tx_boundary_delta[j]))
    }

    pub(in crate::driver::can_servo) fn tx_at(&self, j: usize) -> Option<Instant> {
        let ns = self.tx_offset_ns[j];
        (ns != Self::UNSET_NS).then(|| self.started + Duration::from_nanos(u64::from(ns)))
    }

    pub(in crate::driver::can_servo) fn record_tx(&mut self, j: usize, boundary: u64, tx_at: Instant) {
        self.sent_mask |= 1u32 << j;
        self.tx_boundary_delta[j] =
            u32::try_from(boundary.saturating_sub(self.base_rx_seq)).unwrap_or(u32::MAX);
        let ns = u32::try_from(tx_at.saturating_duration_since(self.started).as_nanos())
            .unwrap_or(Self::UNSET_NS - 1);
        self.tx_offset_ns[j] = ns.min(Self::UNSET_NS - 1);
    }
}

/// Time allowed for register replies during ticked range-check recovery (safe at 50 Hz ticks).
pub(in crate::driver::can_servo) const TICKED_RANGE_CHECK_REPLY_TIMEOUT: Duration = Duration::from_millis(60);

/// Bounded 0xCC feedback refresh for the position-force gripper during enable: `boundary` is the
/// transmit-time RX sequence barrier established after draining RX right before sending 0xCC
/// (None: waiting to drain RX and send 0xCC).
#[derive(Debug)]
pub(in crate::driver::can_servo) struct FeedbackRefresh { joint: usize, boundary: Option<u64>, deadline: Deadline }

#[derive(Debug, Clone, Copy)]
pub(in crate::driver::can_servo) struct RefreshSample {
    pub(in crate::driver::can_servo) position: f32,
    pub(in crate::driver::can_servo) velocity: f32,
    pub(in crate::driver::can_servo) motor_status: u8,
    pub(in crate::driver::can_servo) rx_at: Instant,
    pub(in crate::driver::can_servo) queue_bound: Option<Instant>,
    pub(in crate::driver::can_servo) seq: u64,
}

#[derive(Debug)]
pub(in crate::driver::can_servo) struct RefreshCapture {
    pub(in crate::driver::can_servo) joint: usize,
    pub(in crate::driver::can_servo) boundary: u64,
    pub(in crate::driver::can_servo) tx_at: Instant,
    pub(in crate::driver::can_servo) deadline: Deadline,
    /// When true (in-enable J7 refresh), `motor_status == 1` (already enabled on a retry after a
    /// dropped enable reply) is valid and does not trigger an immediate unsafe veto. Hardware
    /// faults and invalid states still veto immediately.
    pub(in crate::driver::can_servo) allow_enabled_state: bool,
    pub(in crate::driver::can_servo) first: Option<RefreshSample>,
    /// First post-boundary J7 reply observed during this refresh that violated stationary
    /// torque-off (`|v| > 0.05`, non-finite velocity, or `motor_status != 0`; when
    /// `allow_enabled_state` is true, `motor_status == 1` with finite velocity is permitted).
    /// Sticky across polls so a later benign reply cannot mask an intervening motion or fault event.
    pub(in crate::driver::can_servo) unsafe_sample: Option<RefreshSample>,
}

pub(in crate::driver::can_servo) enum RefreshStep {
    Ready(RefreshSample),
    Pending(FeedbackRefresh),
    TimedOut,
}
