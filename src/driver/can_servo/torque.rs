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
