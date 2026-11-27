//! Legacy RobStride CAN actuator arm driver (reBot Arm B601-RS).
//!
//! TODO: engine adoption in a later release.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::driver::*;
use crate::driver::can_servo::*;
use crate::model::JointState;
use crate::protocol::robstride::{self, RobStrideLimits};
use crate::transport::{CanFrame, CanRxTime, CanTransport, TransportError};

pub struct RobStrideArmDriver<C: CanTransport> {
    can: C,
    motor_ids: Vec<u8>,
    master_id: u8,
    limits: RobStrideLimits,
    default_kp: f32,
    default_kd: f32,
    pub(in crate::driver) cached_states: Vec<JointState>,
    pub(in crate::driver) last_feedback_instant: Vec<Option<Instant>>,
    pub(in crate::driver) pose_feedback_instant: Vec<Option<Instant>>,
    feedback_seq: Vec<u64>,
    feedback_continuity_epoch: Vec<u64>,
    /// Instant taken before the last receive call that found the RX queue empty.
    rx_empty_instant: Option<Instant>,
    /// `rx_empty_instant` when each joint's last sample was decoded.
    pub(in crate::driver) feedback_queue_bound: Vec<Option<Instant>>,
    /// Per-motor causal boundary captured after the last motion/enable write returns from
    /// `can.transmit()`. Used ONLY inside the rest-pose disable gate.
    rest_pose_boundary: Vec<Option<RestPoseBoundary>>,
    /// Active control-release id distinguishing separate ticks for the rest-pose K=3 window.
    control_release_id: u64,
    /// True once the owner has called `begin_control_release`, preventing receive passes from
    /// advancing `control_release_id` independently.
    control_release_managed: bool,
    /// Per-motor K=3 causal rest-pose evidence window.
    pub(in crate::driver) rest_pose_window: Vec<CausalWindow>,
    /// True once the most recent RX receive/drain pass reached an empty queue (`Timeout`/`WouldBlock`).
    last_read_drained: bool,
    /// True while a pre-disable RX drain did not complete (zero budget without drained state, or
    /// exhausted drain); fail-closed for the rest-pose disable gate until a drain completes.
    awaiting_rx_boundary: bool,
    fault_flags: Vec<u8>,
    clock: Option<Arc<dyn crate::transport::DriverClock>>,
    abort_flag: Option<Arc<AtomicBool>>,
    estop_gate: Option<crate::rt_core::urgent::EstopGate>,
    stopped_disable_gate: Option<StoppedDisableGate>,
    tx_release_deadline: Option<Instant>,
    disable_dispatched_mask: u64,
    #[cfg(not(target_arch = "wasm32"))]
    shadow_staging: crate::torque::shadow::ShadowDriverStaging,
}

impl<C: CanTransport> RobStrideArmDriver<C> {
    pub fn new(can: C, motor_ids: Vec<u8>) -> Self {
        let dof = motor_ids.len();
        let clock = can.clock();
        Self {
            can,
            motor_ids,
            master_id: 0xFD,
            limits: RobStrideLimits::default(),
            default_kp: 40.0,
            default_kd: 1.0,
            cached_states: vec![JointState::default(); dof],
            last_feedback_instant: vec![None; dof],
            pose_feedback_instant: vec![None; dof],
            feedback_seq: vec![0; dof],
            feedback_continuity_epoch: vec![0; dof],
            rx_empty_instant: None,
            feedback_queue_bound: vec![None; dof],
            rest_pose_boundary: vec![None; dof],
            control_release_id: 0,
            control_release_managed: false,
            rest_pose_window: vec![CausalWindow::default(); dof],
            last_read_drained: false,
            awaiting_rx_boundary: false,
            fault_flags: vec![0; dof],
            clock,
            abort_flag: None,
            estop_gate: None,
            stopped_disable_gate: None,
            tx_release_deadline: None,
            disable_dispatched_mask: 0,
            #[cfg(not(target_arch = "wasm32"))]
            shadow_staging: crate::torque::shadow::ShadowDriverStaging::new(false),
        }
    }

    #[inline]
    fn current_stop_stamp(&self) -> (u64, u64) {
        self.estop_gate
            .as_ref()
            .map_or((0, 0), crate::rt_core::urgent::EstopGate::generations)
    }

    #[inline]
    fn strict_stop_check(&self, expected_stamp: Option<(u64, u64)>) -> Result<(), ArmDriverError> {
        let latched = self.abort_flag.as_ref().is_some_and(|f| f.load(Ordering::SeqCst));
        let pending = self.estop_gate.as_ref().is_some_and(|g| g.pending());
        if latched || pending || expected_stamp.is_some_and(|s| self.current_stop_stamp() != s) {
            return Err(ArmDriverError::EstopInhibited);
        }
        if self.tx_release_deadline.is_some_and(|dl| self.now() >= dl) {
            return Err(ArmDriverError::Transport(TransportError::Timeout));
        }
        Ok(())
    }

    #[inline]
    fn pending_stop_check(&self, expected_stamp: Option<(u64, u64)>) -> Result<(), ArmDriverError> {
        if self.estop_gate.as_ref().is_some_and(|g| g.pending())
            || expected_stamp.is_some_and(|s| self.current_stop_stamp() != s)
        {
            return Err(ArmDriverError::EstopInhibited);
        }
        if self.tx_release_deadline.is_some_and(|dl| self.now() >= dl) {
            return Err(ArmDriverError::Transport(TransportError::Timeout));
        }
        Ok(())
    }

    #[inline]
    fn now(&self) -> Instant {
        self.clock.as_ref().map_or_else(Instant::now, |c| c.now())
    }

    #[inline]
    fn since(&self, earlier: Instant) -> Duration {
        self.now().saturating_duration_since(earlier)
    }

    fn evaluate_rest_pose_gate(&self) -> RestPoseGateStatus {
        let Some(ref gate) = self.stopped_disable_gate else {
            return RestPoseGateStatus::RefusedNonRest;
        };
        if !gate.has_resting_pose {
            return RestPoseGateStatus::RefusedNonRest;
        }
        let arm_joints = gate.arm_joint_indices();
        if arm_joints.is_empty() {
            return RestPoseGateStatus::RefusedNonRest;
        }
        let now = self.now();
        let caps = self.capabilities();
        let drain_incomplete = self.awaiting_rx_boundary || !self.last_read_drained;
        let mut missing_fresh = false;
        let mut window_incomplete = false;
        for &i in arm_joints {
            let Some(s) = self.cached_states.get(i) else {
                return RestPoseGateStatus::RefusedNonRest;
            };
            let fresh = self
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
            if fresh {
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

    fn check_disable_requested(&self, started_at: Instant) -> Result<(), ArmDriverError> {
        if self.since(started_at) >= STOPPED_DISABLE_REQUEST_LIFETIME {
            return Err(ArmDriverError::DisableRequestExpired);
        }
        if self.evaluate_rest_pose_gate() != RestPoseGateStatus::Confirmed {
            return Err(ArmDriverError::TorqueOffWouldDrop);
        }
        Ok(())
    }

    #[inline]
    fn capture_rx_clock_sample(&self, mono_now: Instant) -> (CanRxTime, Option<i128>) {
        if let Some(sample) = self.can.rx_clock_sample() {
            (sample.rx_now, sample.realtime_minus_monotonic_ns)
        } else {
            (CanRxTime::Monotonic(mono_now), None)
        }
    }

    /// Per-write transmit choke point for RobStride type-4 stop / fault-clear CAN frames:
    /// re-evaluates freshness, rest pose (K=3 window), and 1 s request lifetime immediately
    /// before each `can.transmit` in every safety state.
    fn send_type4_frame(&mut self, idx: usize, frame: &CanFrame, kind: DisableKind) -> Result<(), ArmDriverError> {
        if let DisableKind::DisableRequested { started_at } = kind {
            self.check_disable_requested(started_at)?;
        }
        let res = self.can.transmit(frame);
        #[cfg(not(target_arch = "wasm32"))]
        if self.shadow_staging.is_enabled() && idx < crate::torque::MAX_CHANNELS {
            let now = self.now();
            let mut outcomes = [crate::torque::MotorWriteOutcome::NotTargeted;
                crate::torque::MAX_CHANNELS];
            outcomes[idx] = if res.is_ok() {
                crate::torque::MotorWriteOutcome::Sent
            } else {
                crate::torque::MotorWriteOutcome::Failed
            };
            let wk = match kind {
                DisableKind::DisableRequested { .. } => crate::torque::WriteKind::DisableRequested,
                DisableKind::DisableProtective { .. } => {
                    crate::torque::WriteKind::DisableProtective
                }
            };
            self.shadow_staging
                .push(crate::torque::shadow::ShadowDriverObs::WriteResult {
                    device: 0,
                    kind: wk,
                    write_done: now,
                    now,
                    target_mask: crate::torque::ChannelMask::bit(idx as u8),
                    outcomes,
                    ambiguous_gap: false,
                });
        }
        res?;
        Ok(())
    }

    /// Per-write transmit choke point for RobStride stop/disable CAN frames.
    fn send_stop_frame(&mut self, idx: usize, id: u8, kind: DisableKind) -> Result<(), ArmDriverError> {
        let frame = robstride::build_stop_frame(id, self.master_id);
        self.send_type4_frame(idx, &frame, kind)
    }

    /// Shared bounded RX drain for RobStride: returns `(frames_processed, rx_drained, hit_err)`.
    /// Updates `last_read_drained` and clears `awaiting_rx_boundary` ONLY when the queue actually
    /// drains to `Timeout | WouldBlock`.
    fn drain_rx_bounded(&mut self, max_frames: usize, deadline: Option<Instant>, max_span: Duration) -> (u16, bool, bool) {
        self.last_read_drained = false;
        let release_id = self.control_release_id;
        let t0 = self.now();
        let mut frames_processed = 0usize;
        let mut rx_drained = false;
        let mut hit_err = false;
        while frames_processed < max_frames
            && deadline.is_none_or(|dl| self.now() < dl)
            && self.since(t0) < max_span
        {
            let attempt = self.now();
            match self.can.receive_with_rx_time() {
                Ok((frame, rx_time)) => {
                    frames_processed += 1;
                    if let Ok(feedback) = robstride::unpack_feedback_frame(&frame, &self.limits) {
                        if let Some(idx) = self.motor_ids.iter().position(|&id| id == feedback.motor_id) {
                            let decode_now = self.now();
                            // Host dequeue time never counts; reject missing or future-dated
                            // timestamps completely (never clamp).
                            let Some(rx_t) = rx_time else {
                                self.last_feedback_instant[idx] = None;
                                self.feedback_queue_bound[idx] = None;
                                self.rest_pose_window[idx].reset();
                                self.feedback_continuity_epoch[idx] =
                                    self.feedback_continuity_epoch[idx].wrapping_add(1);
                                #[cfg(not(target_arch = "wasm32"))]
                                if self.shadow_staging.is_enabled()
                                    && idx < crate::torque::MAX_CHANNELS
                                {
                                    let sample_id = self.shadow_staging.next_sample_id();
                                    self.shadow_staging.push(
                                        crate::torque::shadow::ShadowDriverObs::CausalSample {
                                            channel: idx as u8,
                                            sample_id,
                                            stamp: crate::torque::shadow::RxStampKind::Missing {
                                                reset_window: true,
                                            },
                                            decode_now,
                                            position_rad: feedback.position,
                                            velocity_rad_s: Some(feedback.velocity),
                                            temperature_c: Some(feedback.mos_temperature),
                                            reported_enabled: None,
                                            correlated_after_write_done: None,
                                            poll_issued: None,
                                            provenance:
                                                crate::torque::shadow::EvidenceProvenance::HardwareCausal,
                                        },
                                    );
                                }
                                continue;
                            };
                            let (rx_at, decode_offset_ns) = match can_rx_time_to_monotonic(&self.can, rx_t, decode_now) {
                                Ok(mapped) => mapped,
                                Err(reject) => {
                                    self.rest_pose_window[idx].reset();
                                    #[cfg(not(target_arch = "wasm32"))]
                                    if self.shadow_staging.is_enabled()
                                        && idx < crate::torque::MAX_CHANNELS
                                    {
                                        let sample_id = self.shadow_staging.next_sample_id();
                                        self.shadow_staging.push(
                                            crate::torque::shadow::ShadowDriverObs::CausalSample {
                                                channel: idx as u8,
                                                sample_id,
                                                stamp: can_rx_reject_to_shadow_stamp(reject),
                                                decode_now,
                                                position_rad: feedback.position,
                                                velocity_rad_s: Some(feedback.velocity),
                                                temperature_c: Some(feedback.mos_temperature),
                                                reported_enabled: None,
                                                correlated_after_write_done: None,
                                                poll_issued: None,
                                                provenance:
                                                    crate::torque::shadow::EvidenceProvenance::HardwareCausal,
                                            },
                                        );
                                    }
                                    continue;
                                }
                            };
                            let boundary_check = match self.rest_pose_boundary[idx] {
                                None => RestBoundaryCheck::Satisfied,
                                Some(boundary) => check_rest_pose_boundary(
                                    boundary,
                                    rx_t,
                                    decode_now,
                                    decode_offset_ns,
                                ),
                            };
                            if boundary_check == RestBoundaryCheck::OffsetDrifted {
                                self.rest_pose_window[idx].reset();
                                let (cur_rx, cur_off) = self.capture_rx_clock_sample(decode_now);
                                self.rest_pose_boundary[idx] = Some(RestPoseBoundary {
                                    write_done: decode_now,
                                    write_done_rx_clock: Some(cur_rx),
                                    write_done_offset_ns: cur_off,
                                });
                                #[cfg(not(target_arch = "wasm32"))]
                                if self.shadow_staging.is_enabled()
                                    && idx < crate::torque::MAX_CHANNELS
                                {
                                    let sample_id = self.shadow_staging.next_sample_id();
                                    self.shadow_staging.push(
                                        crate::torque::shadow::ShadowDriverObs::CausalSample {
                                            channel: idx as u8,
                                            sample_id,
                                            stamp:
                                                crate::torque::shadow::RxStampKind::OffsetDrifted {
                                                    rebased_settle: true,
                                                },
                                            decode_now,
                                            position_rad: feedback.position,
                                            velocity_rad_s: Some(feedback.velocity),
                                            temperature_c: Some(feedback.mos_temperature),
                                            reported_enabled: None,
                                            correlated_after_write_done: None,
                                            poll_issued: None,
                                            provenance:
                                                crate::torque::shadow::EvidenceProvenance::HardwareCausal,
                                        },
                                    );
                                }
                                continue;
                            }
                            self.cached_states[idx] = JointState {
                                position: feedback.position,
                                velocity: feedback.velocity,
                                torque: feedback.torque,
                                temperature: feedback.mos_temperature,
                            };
                            self.fault_flags[idx] = feedback.fault_flags;
                            let prev_rx = self.last_feedback_instant[idx];
                            let prev_qb = self.feedback_queue_bound[idx];
                            if prev_rx.is_none_or(|p| rx_at.saturating_duration_since(p) > STOPPED_DISABLE_FEEDBACK_MAX_AGE)
                                || prev_qb.is_none_or(|qb| rx_at.saturating_duration_since(qb) > STOPPED_DISABLE_FEEDBACK_MAX_AGE)
                                || !feedback.position.is_finite()
                            {
                                self.feedback_continuity_epoch[idx] =
                                    self.feedback_continuity_epoch[idx].wrapping_add(1);
                            }
                            self.last_feedback_instant[idx] = Some(rx_at);
                            self.feedback_seq[idx] = self.feedback_seq[idx].wrapping_add(1);
                            self.feedback_queue_bound[idx] = self.rx_empty_instant;
                            let mut window_offset_drifted = false;
                            match boundary_check {
                                RestBoundaryCheck::OffsetDrifted => unreachable!(),
                                RestBoundaryCheck::InSettle => {
                                    self.rest_pose_window[idx].reset();
                                }
                                RestBoundaryCheck::Satisfied => {
                                    self.pose_feedback_instant[idx] = Some(rx_at);
                                    if let Some(ref gate) = self.stopped_disable_gate {
                                        if gate.is_arm_joint(idx) {
                                            if !self.awaiting_rx_boundary
                                                && self.rx_empty_instant.is_some()
                                                && joint_state_valid(&self.cached_states[idx], self.capabilities())
                                                && gate.joint_at_rest(idx, feedback.position)
                                            {
                                                if self.rest_pose_window[idx].record_agree(
                                                    rx_at,
                                                    release_id,
                                                    STOPPED_DISABLE_FEEDBACK_MAX_AGE,
                                                    decode_offset_ns,
                                                ) == WindowRecord::OffsetDrifted
                                                {
                                                    window_offset_drifted = true;
                                                    let (cur_rx, cur_off) =
                                                        self.capture_rx_clock_sample(decode_now);
                                                    self.rest_pose_boundary[idx] = Some(RestPoseBoundary {
                                                        write_done: decode_now,
                                                        write_done_rx_clock: Some(cur_rx),
                                                        write_done_offset_ns: cur_off,
                                                    });
                                                }
                                            } else {
                                                self.rest_pose_window[idx].record_disagree();
                                            }
                                        }
                                    }
                                }
                            }
                            #[cfg(not(target_arch = "wasm32"))]
                            if self.shadow_staging.is_enabled() && idx < crate::torque::MAX_CHANNELS
                            {
                                let sample_id = self.shadow_staging.next_sample_id();
                                let stamp = if window_offset_drifted {
                                    crate::torque::shadow::RxStampKind::OffsetDrifted {
                                        rebased_settle: true,
                                    }
                                } else {
                                    let drift_ns = self
                                        .rest_pose_boundary[idx]
                                        .and_then(|b| b.write_done_offset_ns)
                                        .zip(decode_offset_ns)
                                        .map_or(0, |(w, d)| {
                                            (d - w).unsigned_abs().min(u128::from(u64::MAX)) as u64
                                        });
                                    crate::torque::shadow::RxStampKind::Valid {
                                        rx_at,
                                        drift_ns,
                                        live_post_settle: boundary_check
                                            == RestBoundaryCheck::Satisfied,
                                    }
                                };
                                self.shadow_staging.push(
                                    crate::torque::shadow::ShadowDriverObs::CausalSample {
                                        channel: idx as u8,
                                        sample_id,
                                        stamp,
                                        decode_now,
                                        position_rad: feedback.position,
                                        velocity_rad_s: Some(feedback.velocity),
                                        temperature_c: Some(feedback.mos_temperature),
                                        reported_enabled: None,
                                        correlated_after_write_done: None,
                                        poll_issued: None,
                                        provenance:
                                            crate::torque::shadow::EvidenceProvenance::HardwareCausal,
                                    },
                                );
                            }
                        }
                    }
                }
                Err(TransportError::Timeout) | Err(TransportError::WouldBlock) => {
                    rx_drained = true;
                    self.rx_empty_instant = Some(attempt);
                    self.last_read_drained = true;
                    if self.awaiting_rx_boundary {
                        self.awaiting_rx_boundary = false;
                        self.rest_pose_window.fill(CausalWindow::default());
                    }
                    break;
                }
                Err(_) => {
                    hit_err = true;
                    break;
                }
            }
        }
        (frames_processed as u16, rx_drained, hit_err)
    }

    fn disable_torque_with_drain(
        &mut self,
        kind: DisableKind,
        max_rx_frames: Option<u16>,
    ) -> Result<(), ArmDriverError> {
        let prev_awaiting = self.awaiting_rx_boundary;
        self.awaiting_rx_boundary = match max_rx_frames {
            Some(0) => self.awaiting_rx_boundary || !self.last_read_drained,
            Some(limit) => {
                let (_, drained, _) =
                    self.drain_rx_bounded(usize::from(limit.min(64)), None, Duration::from_millis(5));
                !drained
            }
            None => {
                let (_, drained, _) = self.drain_rx_bounded(128, None, Duration::from_millis(5));
                !drained
            }
        };
        let n = self.motor_ids.len().min(64);
        let all_mask = if n >= 64 { u64::MAX } else { (1u64 << n) - 1 };
        if (self.disable_dispatched_mask & all_mask) == all_mask {
            self.disable_dispatched_mask = 0;
        }
        let skip_dispatched = matches!(kind, DisableKind::DisableRequested { .. });
        let mut wrote_any = false;
        let mut refused_err = None;
        let mut last_err = None;
        for idx in 0..self.motor_ids.len() {
            if skip_dispatched && idx < 64 && (self.disable_dispatched_mask & (1u64 << idx)) != 0 {
                continue;
            }
            let id = self.motor_ids[idx];
            match self.send_stop_frame(idx, id, kind) {
                Ok(()) => {
                    wrote_any = true;
                    if idx < 64 {
                        self.disable_dispatched_mask |= 1u64 << idx;
                    }
                }
                Err(e @ (ArmDriverError::TorqueOffWouldDrop | ArmDriverError::DisableRequestExpired)) => {
                    refused_err = Some(e);
                    break;
                }
                Err(e) => last_err = Some(e),
            }
        }
        if !wrote_any && last_err.is_none() && self.disable_dispatched_mask == 0 {
            if let Some(e) = refused_err {
                self.awaiting_rx_boundary = prev_awaiting || !self.last_read_drained;
                return Err(e);
            }
        }
        if wrote_any {
            if let Err(e) = self.can.flush() {
                last_err = Some(e.into());
            }
        }
        if let Some(e) = refused_err.or(last_err) {
            return Err(e);
        }
        if (self.disable_dispatched_mask & all_mask) == all_mask {
            self.disable_dispatched_mask = 0;
        }
        Ok(())
    }

    pub fn with_gains(mut self, kp: f32, kd: f32) -> Self {
        self.default_kp = kp;
        self.default_kd = kd;
        self
    }

    pub fn can_mut(&mut self) -> &mut C {
        &mut self.can
    }
}

impl<C: CanTransport> ArmDriver for RobStrideArmDriver<C> {
    fn motor_zero_support(&self) -> MotorZeroSupport {
        MotorZeroSupport::unsupported("RobStride: src/protocol/robstride.rs has TYPE_SET_ZERO (communication type 6) but no \
            verified zero sequence: motorbridge, the SDK of the official reBotArm_control_py, lists RobStride zero calibration \
            as unresolved (device-side zero_sta / mechPos verification can fail), and whether type 6 persists without \
            type 22 (save parameters) is not checked")
    }

    fn rest_pose_gate_status(&self) -> RestPoseGateStatus {
        self.evaluate_rest_pose_gate()
    }

    fn begin_control_release(&mut self, release_seq: u64) {
        self.control_release_managed = true;
        self.control_release_id = release_seq;
    }

    fn capabilities(&self) -> DriverCapabilities { DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true, torque_feedforward: true, enable_feedback: false, load_feedback: false } }

    fn enable_state(&self) -> EnableState { EnableState::Unknown }

    fn has_driver_fresh_position_api(&self) -> bool { true }

    fn fresh_measured_position_for(&self, joint: usize, max_age: Duration) -> Option<f32> {
        let mut pos = [0.0f32; 1];
        self.fresh_joint_positions_into(&[joint], max_age, &mut pos)
            .then_some(pos[0])
    }

    fn fresh_measured_sample_for(&self, joint: usize, max_age: Duration) -> Option<(f32, u64)> {
        let pos = self.fresh_measured_position_for(joint, max_age)?;
        let epoch = self.feedback_continuity_epoch.get(joint).copied().unwrap_or(0);
        Some((pos, epoch))
    }

    fn set_tx_release_deadline(&mut self, deadline: Option<Instant>) {
        self.tx_release_deadline = deadline;
    }

    fn clock_now(&self) -> Instant {
        self.now()
    }

    fn driver_clock(&self) -> Option<std::sync::Arc<dyn crate::transport::DriverClock>> {
        self.clock.clone()
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

    fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        out.clear();
        out.extend_from_slice(&self.last_feedback_instant);
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

    fn is_feedback_valid(&self) -> bool {
        if self.last_feedback_instant.is_empty() {
            return false;
        }
        const FRESHNESS_LIMIT: Duration = Duration::from_millis(250);
        self.last_feedback_instant
            .iter()
            .all(|opt| opt.is_some_and(|instant| self.since(instant) <= FRESHNESS_LIMIT))
    }

    fn read_joint_states_budgeted(
        &mut self,
        out: &mut Vec<JointState>,
        deadline: Instant,
        max_rx_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        if !self.control_release_managed {
            self.control_release_id = self.control_release_id.wrapping_add(1).max(1);
        }
        let (frames_processed, rx_drained, hit_err) = self.drain_rx_bounded(
            usize::from(max_rx_frames.min(64)),
            Some(deadline),
            Duration::from_micros(2000),
        );
        out.clear();
        out.extend_from_slice(&self.cached_states);
        Ok((frames_processed, !rx_drained && !hit_err))
    }

    fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
        out.clear();
        out.extend_from_slice(&self.cached_states);
        Some(())
    }

    fn check_hardware_status(&self) -> Result<(), ArmDriverError> {
        for (i, &flags) in self.fault_flags.iter().enumerate() {
            if flags != 0 {
                let motor_id = self.motor_ids.get(i).copied().unwrap_or(0);
                return Err(ArmDriverError::HardwareFault(format!(
                    "RobStride Joint {} (CAN ID 0x{:X}) reported fault flags: 0x{:02X}",
                    i, motor_id, flags
                )));
            }
        }
        Ok(())
    }

    fn clear_fault_disables_joints(&self) -> Vec<usize> {
        if self.has_hardware_fault() {
            (0..self.motor_ids.len()).collect()
        } else {
            Vec::new()
        }
    }

    fn clear_fault_disables_arm_joint(&self, arm_dof: usize) -> bool {
        self.has_hardware_fault() && !self.motor_ids.is_empty() && arm_dof > 0
    }

    fn has_hardware_fault(&self) -> bool {
        self.fault_flags.iter().any(|&flags| flags != 0)
    }

    fn clear_fault(&mut self) -> Result<(), ArmDriverError> {
        // `build_clear_fault_frame` sends type-4 (`TYPE_STOP`) frames with `data[0] = 1`
        // (documented under the motor-stop command; hardware behaviour unverified). Fail closed
        // by draining RX and routing every write through the same per-write disable choke point
        // as `disable_torque`.
        let prev_awaiting = self.awaiting_rx_boundary;
        let (_, drained, _) = self.drain_rx_bounded(128, None, Duration::from_millis(5));
        self.awaiting_rx_boundary = !drained;
        let started_at = self.now();
        let kind = DisableKind::DisableRequested { started_at };
        let mut wrote_any = false;
        let mut refused_err = None;
        let mut last_err = None;
        for idx in 0..self.motor_ids.len() {
            let id = self.motor_ids[idx];
            let frame = robstride::build_clear_fault_frame(id, self.master_id);
            match self.send_type4_frame(idx, &frame, kind) {
                Ok(()) => wrote_any = true,
                Err(e @ (ArmDriverError::TorqueOffWouldDrop | ArmDriverError::DisableRequestExpired)) => {
                    refused_err = Some(e);
                    break;
                }
                Err(e) => last_err = Some(e),
            }
        }
        if !wrote_any && last_err.is_none() {
            if let Some(e) = refused_err {
                self.awaiting_rx_boundary = prev_awaiting || !self.last_read_drained;
                return Err(e);
            }
        }
        if wrote_any {
            if let Err(e) = self.can.flush() {
                last_err = Some(e.into());
            }
        }
        // A sent reset is not proof that the reported fault has cleared.
        // The next telemetry response updates fault_flags.
        if let Some(e) = refused_err {
            Err(e)
        } else if let Some(e) = last_err {
            Err(ArmDriverError::HardwareFault(e.to_string()))
        } else {
            Ok(())
        }
    }

    fn set_abort_flag(&mut self, flag: Arc<AtomicBool>) {
        self.abort_flag = Some(flag);
    }

    fn set_estop_gate(&mut self, gate: crate::rt_core::urgent::EstopGate) {
        self.estop_gate = Some(gate);
    }

    fn set_stopped_disable_gate(&mut self, gate: StoppedDisableGate) {
        self.stopped_disable_gate = Some(gate);
    }

    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        if !enabled {
            self.disable_torque(DisableKind::DisableRequested {
                started_at: self.now(),
            })
        } else {
            self.disable_dispatched_mask = 0;
            let start_stamp = self.current_stop_stamp();
            for idx in 0..self.motor_ids.len() {
                let id = self.motor_ids[idx];
                self.strict_stop_check(Some(start_stamp))?;
                let frame = robstride::build_enable_frame(id, self.master_id);
                self.strict_stop_check(Some(start_stamp))?;
                self.can.transmit(&frame)?;
            }
            self.strict_stop_check(Some(start_stamp))?;
            self.can.flush()?;
            #[cfg(not(target_arch = "wasm32"))]
            if self.shadow_staging.is_enabled() {
                let now = self.now();
                let mut mask = crate::torque::ChannelMask::EMPTY;
                let mut outcomes = [crate::torque::MotorWriteOutcome::NotTargeted;
                    crate::torque::MAX_CHANNELS];
                for i in 0..self.motor_ids.len().min(crate::torque::MAX_CHANNELS) {
                    mask.insert(i as u8);
                    outcomes[i] = crate::torque::MotorWriteOutcome::Sent;
                }
                if !mask.is_empty() {
                    self.shadow_staging
                        .push(crate::torque::shadow::ShadowDriverObs::WriteResult {
                            device: 0,
                            kind: crate::torque::WriteKind::Enable,
                            write_done: now,
                            now,
                            target_mask: mask,
                            outcomes,
                            ambiguous_gap: false,
                        });
                }
            }
            Ok(())
        }
    }

    fn disable_frames_dispatched(&self) -> bool {
        self.disable_dispatched_mask != 0
    }

    fn disable_dispatched_mask(&self) -> u64 {
        self.disable_dispatched_mask
    }

    fn disable_torque(&mut self, kind: DisableKind) -> Result<(), ArmDriverError> {
        self.disable_torque_with_drain(kind, None)
    }

    fn begin_torque(&mut self, enabled: bool) -> Result<TorqueProgress, ArmDriverError> {
        self.set_torque(enabled)
            .map(|()| TorqueProgress::Done(TorqueOutcome::Unconfirmed))
    }

    fn begin_disable_torque(&mut self, kind: DisableKind) -> Result<TorqueProgress, ArmDriverError> {
        self.disable_torque_with_drain(kind, None)
            .map(|()| TorqueProgress::Done(TorqueOutcome::Unconfirmed))
    }

    fn begin_disable_torque_budgeted(
        &mut self,
        kind: DisableKind,
        max_rx_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.disable_torque_with_drain(kind, Some(max_rx_frames))
            .map(|()| TorqueProgress::Done(TorqueOutcome::Unconfirmed))
    }

    fn write_estop_hold(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        let n = self.motor_ids.len().min(64);
        let mask = if n >= 64 { u64::MAX } else { (1u64 << n) - 1 };
        self.write_measured_hold_masked(mask, targets).map(|_| ())
    }

    fn write_estop_hold_bounded(&mut self, targets: &[JointTarget]) -> Result<TxBatchOutcome, ArmDriverError> {
        let n = self.motor_ids.len().min(64);
        let mask = if n >= 64 { u64::MAX } else { (1u64 << n) - 1 };
        self.write_measured_hold_masked(mask, targets).map(|r| r.outcome)
    }

    fn write_measured_hold_masked(
        &mut self,
        hold_mask: u64,
        targets: &[JointTarget],
    ) -> Result<HoldPassReport, ArmDriverError> {
        if targets.len() != self.motor_ids.len() {
            return Err(ArmDriverError::JointCountMismatch {
                expected: self.motor_ids.len(),
                actual: targets.len(),
            });
        }
        let start_stamp = self.current_stop_stamp();
        self.pending_stop_check(Some(start_stamp))?;
        let dof = self.motor_ids.len().min(crate::rt_core::mailbox::MAX_DOF);
        let mut report = HoldPassReport::empty(dof);
        let mut sent_mask = 0u64;
        let mut stale_mask = 0u64;
        for (i, target) in targets.iter().enumerate().take(dof) {
            if i < 64 && (self.disable_dispatched_mask & (1u64 << i)) != 0 {
                report.joint_status[i] = JointHoldStatus::SkippedOff;
                continue;
            }
            let fresh = self.fresh_measured_position_for(i, STOPPED_DISABLE_FEEDBACK_MAX_AGE);
            if (hold_mask & (1u64 << i)) == 0 {
                report.joint_status[i] = if fresh.is_none() {
                    JointHoldStatus::UnheldNoFreshFeedback
                } else {
                    JointHoldStatus::SkippedOff
                };
                continue;
            }
            let pos = if target.position.is_finite() {
                target.position
            } else if let Some(fresh_pos) = fresh {
                fresh_pos
            } else {
                report.joint_status[i] = JointHoldStatus::UnheldNoFreshFeedback;
                continue;
            };
            let is_stale = fresh.is_none();
            let kp = target.kp.unwrap_or(self.default_kp);
            let kd = target.kd.unwrap_or(self.default_kd);
            let torque = target.torque.unwrap_or(0.0);
            let frame = robstride::build_mit_frame(
                self.motor_ids[i],
                pos,
                0.0,
                kp,
                kd,
                torque,
                &self.limits,
            );
            self.pending_stop_check(Some(start_stamp))?;
            let t0 = Instant::now();
            let cpu0 = crate::diagnostics::tick::thread_cpu_ns();
            let res = self.can.transmit(&frame);
            let wall_ns = t0.elapsed().as_nanos().min(u128::from(u64::MAX)) as u64;
            let cpu_ns = crate::diagnostics::tick::thread_cpu_ns().saturating_sub(cpu0);
            let kind = match &res {
                Ok(()) => crate::diagnostics::tick::TxAttemptKind::Sent,
                Err(TransportError::WouldBlock) => crate::diagnostics::tick::TxAttemptKind::WouldBlock,
                Err(TransportError::NoBufferSpace) => crate::diagnostics::tick::TxAttemptKind::Enobufs,
                Err(_) => crate::diagnostics::tick::TxAttemptKind::Error,
            };
            crate::diagnostics::tick::record_tx_attempt(kind, wall_ns, cpu_ns);
            if res.is_ok() {
                crate::diagnostics::tick::record_tx_frame_kind(
                    crate::diagnostics::tick::TxFrameKind::Hold,
                );
            }
            res?;
            if is_stale {
                stale_mask |= 1u64 << i;
                report.joint_status[i] = JointHoldStatus::HeldStaleTarget;
            } else {
                report.joint_status[i] = JointHoldStatus::Queued;
            }
            sent_mask |= 1u64 << i;
            report.sent_any = true;
        }
        if report.sent_any {
            self.pending_stop_check(Some(start_stamp))?;
            self.can.flush()?;
            #[cfg(not(target_arch = "wasm32"))]
            if self.shadow_staging.is_enabled() {
                let now = self.now();
                let mut mask = crate::torque::ChannelMask::EMPTY;
                let mut outcomes = [crate::torque::MotorWriteOutcome::NotTargeted;
                    crate::torque::MAX_CHANNELS];
                for i in 0..dof.min(crate::torque::MAX_CHANNELS) {
                    if (sent_mask & (1u64 << i)) != 0 {
                        mask.insert(i as u8);
                        outcomes[i] = crate::torque::MotorWriteOutcome::Sent;
                    }
                }
                if !mask.is_empty() {
                    self.shadow_staging
                        .push(crate::torque::shadow::ShadowDriverObs::WriteResult {
                            device: 0,
                            kind: crate::torque::WriteKind::Hold,
                            write_done: now,
                            now,
                            target_mask: mask,
                            outcomes,
                            ambiguous_gap: false,
                        });
                }
            }
            for i in 0..dof {
                if (sent_mask & (1u64 << i)) != 0 {
                    report.joint_status[i] = if (stale_mask & (1u64 << i)) != 0 {
                        JointHoldStatus::HeldStaleTarget
                    } else {
                        JointHoldStatus::Held
                    };
                }
            }
        }
        Ok(report)
    }

    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        if targets.len() != self.motor_ids.len() {
            return Err(ArmDriverError::JointCountMismatch {
                expected: self.motor_ids.len(),
                actual: targets.len(),
            });
        }

        for (i, target) in targets.iter().enumerate() {
            let kp = target.kp.unwrap_or(self.default_kp);
            let kd = target.kd.unwrap_or(self.default_kd);
            let vel = target.velocity.unwrap_or(0.0);
            let torque = target.torque.unwrap_or(0.0);

            let frame = robstride::build_mit_frame(
                self.motor_ids[i],
                target.position,
                vel,
                kp,
                kd,
                torque,
                &self.limits,
            );
            self.rest_pose_window[i].reset();
            let t0 = Instant::now();
            let cpu0 = crate::diagnostics::tick::thread_cpu_ns();
            let res = self.can.transmit(&frame);
            let write_done = self.now();
            let wall_ns = t0.elapsed().as_nanos().min(u128::from(u64::MAX)) as u64;
            let cpu_ns = crate::diagnostics::tick::thread_cpu_ns().saturating_sub(cpu0);
            let kind = match &res {
                Ok(()) => crate::diagnostics::tick::TxAttemptKind::Sent,
                Err(TransportError::WouldBlock) => crate::diagnostics::tick::TxAttemptKind::WouldBlock,
                Err(TransportError::NoBufferSpace) => crate::diagnostics::tick::TxAttemptKind::Enobufs,
                Err(_) => crate::diagnostics::tick::TxAttemptKind::Error,
            };
            crate::diagnostics::tick::record_tx_attempt(kind, wall_ns, cpu_ns);
            if res.is_ok() {
                crate::diagnostics::tick::record_tx_frame_kind(
                    crate::diagnostics::tick::TxFrameKind::Normal,
                );
            }
            res?;
            let (write_done_rx_clock, write_done_offset_ns) =
                self.capture_rx_clock_sample(write_done);
            self.rest_pose_boundary[i] = Some(RestPoseBoundary {
                write_done,
                write_done_rx_clock: Some(write_done_rx_clock),
                write_done_offset_ns,
            });
            #[cfg(not(target_arch = "wasm32"))]
            if self.shadow_staging.is_enabled() && i < crate::torque::MAX_CHANNELS {
                let mut outcomes = [crate::torque::MotorWriteOutcome::NotTargeted;
                    crate::torque::MAX_CHANNELS];
                outcomes[i] = crate::torque::MotorWriteOutcome::Sent;
                self.shadow_staging
                    .push(crate::torque::shadow::ShadowDriverObs::WriteResult {
                        device: 0,
                        kind: crate::torque::WriteKind::Motion,
                        write_done,
                        now: write_done,
                        target_mask: crate::torque::ChannelMask::bit(i as u8),
                        outcomes,
                        ambiguous_gap: false,
                    });
            }
        }
        self.can.flush()?;
        Ok(())
    }

    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        if !self.control_release_managed {
            self.control_release_id = self.control_release_id.wrapping_add(1).max(1);
        }
        let _ = self.drain_rx_bounded(64, None, Duration::from_micros(2000));
        Ok(self.cached_states.clone())
    }
    #[cfg(not(target_arch = "wasm32"))]
    fn shadow_ext(&self) -> Option<&dyn ShadowExt> { Some(self) }
    #[cfg(not(target_arch = "wasm32"))]
    fn shadow_ext_mut(&mut self) -> Option<&mut dyn ShadowExt> { Some(self) }
}

#[cfg(not(target_arch = "wasm32"))]
impl<C: CanTransport> ShadowExt for RobStrideArmDriver<C> {
    fn shadow_vendor_kind(&self) -> crate::torque::VendorKind {
        crate::torque::VendorKind::RobStride
    }

    fn set_shadow_enabled(&mut self, enabled: bool) {
        self.shadow_staging.set_enabled(enabled);
    }

    fn drain_shadow_observations(
        &mut self,
        out: &mut [Option<crate::torque::shadow::ShadowDriverObs>],
    ) -> (usize, u64) {
        self.shadow_staging.drain_into(out)
    }
}
