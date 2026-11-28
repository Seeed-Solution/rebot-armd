//! Legacy FashionStar UartServo arm driver (Star Arm 102).
//!
//! TODO: engine adoption in a later release.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::driver::*;
use crate::model::JointState;
use crate::protocol::fashionstar::{self, FashionStarPacket};
use crate::transport::{Transport, TransportError};

pub struct FashionStarArmDriver<T: Transport> {
    transport: T,
    servo_ids: Vec<u8>,
    zero_offsets: Vec<u16>,
    pub(in crate::driver) cached_states: Vec<JointState>,
    pub(in crate::driver) last_feedback_instant: Vec<Option<Instant>>,
    feedback_seq: Vec<u64>,
    feedback_continuity_epoch: Vec<u64>,
    poll_cursor: usize,
    rx_buffer: Vec<u8>,
    rx_timestamps: Vec<Instant>,
    control_release_id: u64,
    control_release_managed: bool,
    pub(in crate::driver) rest_pose_window: Vec<CausalWindow>,
    /// Per-servo monotonic instant of the last motion or enable write completion (`flush()`),
    /// establishing the post-motion causality bound (`TORQUE_CAUSAL_SETTLE_WINDOW`).
    last_motion_write_at: Vec<Option<Instant>>,
    /// True after a motion/enable write until the transport input buffer has been flushed past
    /// `TORQUE_CAUSAL_SETTLE_WINDOW` so stale pre-motion serial replies cannot enter the window.
    rx_needs_post_motion_flush: bool,
    /// True when an unexpected/late/corrupt RS485 frame or query timeout was observed; cleared
    /// only after `TORQUE_CAUSAL_SETTLE_WINDOW` passes with no stray bytes.
    line_poisoned: bool,
    line_poison_quiet_until: Option<Instant>,
    clock: Option<Arc<dyn crate::transport::DriverClock>>,
    abort_flag: Option<Arc<AtomicBool>>,
    estop_gate: Option<crate::rt_core::urgent::EstopGate>,
    stopped_disable_gate: Option<StoppedDisableGate>,
    tx_release_deadline: Option<Instant>,
    disable_dispatched_mask: u64,
    #[cfg(not(target_arch = "wasm32"))]
    shadow_staging: crate::torque::shadow::ShadowDriverStaging,
}

impl<T: Transport> FashionStarArmDriver<T> {
    pub fn new(transport: T, servo_ids: Vec<u8>) -> Self {
        let dof = servo_ids.len();
        let clock = transport.clock();
        Self {
            transport,
            servo_ids,
            zero_offsets: vec![fashionstar::DEFAULT_ZERO_OFFSET; dof],
            cached_states: vec![JointState::default(); dof],
            last_feedback_instant: vec![None; dof],
            feedback_seq: vec![0; dof],
            feedback_continuity_epoch: vec![0; dof],
            poll_cursor: 0,
            rx_buffer: Vec::with_capacity(256),
            rx_timestamps: Vec::with_capacity(256),
            control_release_id: 0,
            control_release_managed: false,
            rest_pose_window: vec![CausalWindow::default(); dof],
            last_motion_write_at: vec![None; dof],
            rx_needs_post_motion_flush: false,
            line_poisoned: false,
            line_poison_quiet_until: None,
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

    fn poison_line(&mut self, now: Instant) {
        self.line_poisoned = true;
        self.line_poison_quiet_until = Some(now + TORQUE_CAUSAL_SETTLE_WINDOW);
        for w in &mut self.rest_pose_window {
            w.reset();
        }
        #[cfg(not(target_arch = "wasm32"))]
        if self.shadow_staging.is_enabled() {
            self.shadow_staging
                .push(crate::torque::shadow::ShadowDriverObs::SerialTimeout { device: 0, now });
        }
    }

    fn record_motion_write_done_all(&mut self) {
        let write_done = self.now();
        for w in &mut self.rest_pose_window {
            w.reset();
        }
        self.last_motion_write_at.fill(Some(write_done));
        self.rx_needs_post_motion_flush = true;
        self.rx_buffer.clear();
        self.rx_timestamps.clear();
        self.transport.clear_input_buffer();
    }

    fn record_motion_write_done_masked(&mut self, mask: u64) {
        let write_done = self.now();
        for i in 0..self.servo_ids.len().min(64) {
            if (mask & (1u64 << i)) != 0 {
                self.rest_pose_window[i].reset();
                self.last_motion_write_at[i] = Some(write_done);
            }
        }
        self.rx_needs_post_motion_flush = true;
        self.rx_buffer.clear();
        self.rx_timestamps.clear();
        self.transport.clear_input_buffer();
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
        let mut missing_fresh = false;
        let mut window_incomplete = false;
        for &i in arm_joints {
            let Some(s) = self.cached_states.get(i) else {
                return RestPoseGateStatus::RefusedNonRest;
            };
            let fresh = self
                .last_feedback_instant
                .get(i)
                .copied()
                .flatten()
                .is_some_and(|t| now.saturating_duration_since(t) <= STOPPED_DISABLE_FEEDBACK_MAX_AGE);
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
        if missing_fresh {
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

    /// Per-write transmit choke point for FashionStar torque-off writes: re-evaluates
    /// freshness, rest pose (K=3 window), and 1 s request lifetime immediately before each
    /// `transport.write_all` in every safety state.
    fn write_torque_off(&mut self, id: u8, kind: DisableKind) -> Result<(), ArmDriverError> {
        let pkt = fashionstar::build_set_torque(id, false);
        if let DisableKind::DisableRequested { started_at } = kind {
            self.check_disable_requested(started_at)?;
        }
        if self.tx_release_deadline.is_some_and(|dl| self.now() >= dl) {
            return Err(ArmDriverError::Transport(TransportError::WouldBlock));
        }
        let res = self.transport.write_all(&pkt);
        #[cfg(not(target_arch = "wasm32"))]
        if self.shadow_staging.is_enabled() {
            if let Some(idx) = self.servo_ids.iter().position(|&s| s == id) {
                if idx < crate::torque::MAX_CHANNELS {
                    let now = self.now();
                    let mut outcomes = [crate::torque::MotorWriteOutcome::NotTargeted;
                        crate::torque::MAX_CHANNELS];
                    outcomes[idx] = if res.is_ok() {
                        crate::torque::MotorWriteOutcome::Sent
                    } else {
                        crate::torque::MotorWriteOutcome::Failed
                    };
                    let wk = match kind {
                        DisableKind::DisableRequested { .. } => {
                            crate::torque::WriteKind::DisableRequested
                        }
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
            }
        }
        res?;
        Ok(())
    }

    pub fn with_zero_offsets(mut self, offsets: Vec<u16>) -> Self {
        self.zero_offsets = offsets;
        self
    }

    pub fn transport(&self) -> &T {
        &self.transport
    }

    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    pub fn rx_buffer(&self) -> &[u8] {
        &self.rx_buffer
    }
}

impl<T: Transport> ArmDriver for FashionStarArmDriver<T> {
    fn motor_zero_support(&self) -> MotorZeroSupport {
        MotorZeroSupport::unsupported("FashionStar: armd implements no zero command for these servos")
    }

    fn rest_pose_gate_status(&self) -> RestPoseGateStatus {
        self.evaluate_rest_pose_gate()
    }

    fn begin_control_release(&mut self, release_seq: u64) {
        self.control_release_managed = true;
        self.control_release_id = release_seq;
    }

    fn capabilities(&self) -> DriverCapabilities { DriverCapabilities::default() }

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

    fn feedback_receive_instants(&self) -> Vec<Option<Instant>> {
        self.last_feedback_instant.clone()
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
        let caps = self.capabilities();
        for (slot, &i) in out.iter_mut().zip(joints.iter()) {
            let fresh = self
                .last_feedback_instant
                .get(i)
                .copied()
                .flatten()
                .is_some_and(|t| self.since(t) <= max_age);
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
        _out: &mut Vec<JointState>,
        _deadline: Instant,
        _max_rx_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        Err(ArmDriverError::Protocol("UNSUPPORTED: serial driver does not support budgeted RX".into()))
    }

    fn begin_torque_budgeted(&mut self, _enabled: bool, _max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
        Err(ArmDriverError::Protocol("UNSUPPORTED: serial driver does not support budgeted torque transitions".into()))
    }

    fn poll_torque_budgeted(&mut self, _deadline: Instant, _max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
        Err(ArmDriverError::Protocol("UNSUPPORTED: serial driver does not support budgeted torque transitions".into()))
    }

    fn driver_clock(&self) -> Option<std::sync::Arc<dyn crate::transport::DriverClock>> {
        self.clock.clone()
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
            for (idx, &id) in self.servo_ids.iter().enumerate() {
                self.strict_stop_check(Some(start_stamp))?;
                let pkt = fashionstar::build_set_torque(id, true);
                self.strict_stop_check(Some(start_stamp))?;
                self.rest_pose_window[idx].reset();
                self.transport.write_all(&pkt)?;
            }
            self.strict_stop_check(Some(start_stamp))?;
            self.transport.flush()?;
            self.record_motion_write_done_all();
            #[cfg(not(target_arch = "wasm32"))]
            if self.shadow_staging.is_enabled() {
                let now = self.now();
                let mut mask = crate::torque::ChannelMask::EMPTY;
                let mut outcomes = [crate::torque::MotorWriteOutcome::NotTargeted;
                    crate::torque::MAX_CHANNELS];
                for i in 0..self.servo_ids.len().min(crate::torque::MAX_CHANNELS) {
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
        let n = self.servo_ids.len().min(64);
        let all_mask = if n >= 64 { u64::MAX } else { (1u64 << n) - 1 };
        if (self.disable_dispatched_mask & all_mask) == all_mask {
            self.disable_dispatched_mask = 0;
        }
        let skip_dispatched = matches!(kind, DisableKind::DisableRequested { .. });
        let mut wrote_any = false;
        let mut refused_err = None;
        let mut last_err = None;
        for idx in 0..self.servo_ids.len() {
            if skip_dispatched && idx < 64 && (self.disable_dispatched_mask & (1u64 << idx)) != 0 {
                continue;
            }
            let id = self.servo_ids[idx];
            match self.write_torque_off(id, kind) {
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
        if wrote_any {
            if let Err(e) = self.transport.flush() {
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

    fn begin_torque(&mut self, enabled: bool) -> Result<TorqueProgress, ArmDriverError> {
        self.set_torque(enabled)
            .map(|()| TorqueProgress::Done(TorqueOutcome::Unconfirmed))
    }

    fn begin_disable_torque(&mut self, kind: DisableKind) -> Result<TorqueProgress, ArmDriverError> {
        self.disable_torque(kind)
            .map(|()| TorqueProgress::Done(TorqueOutcome::Unconfirmed))
    }

    fn begin_disable_torque_budgeted(
        &mut self,
        kind: DisableKind,
        _max_rx_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.begin_disable_torque(kind)
    }

    fn begin_client_park_disable_with_kind(
        &mut self,
        kind: DisableKind,
        _max_rx_frames: Option<u16>,
        expected_stop_gen: Option<u64>,
    ) -> Result<TorqueProgress, ArmDriverError> {
        let mut stamp = self.current_stop_stamp();
        if let Some(gen) = expected_stop_gen {
            stamp.1 = gen;
        }
        self.strict_stop_check(Some(stamp))?;
        let n = self.servo_ids.len().min(64);
        let all_mask = if n >= 64 { u64::MAX } else { (1u64 << n) - 1 };
        if (self.disable_dispatched_mask & all_mask) == all_mask {
            self.disable_dispatched_mask = 0;
        }
        let skip_dispatched = matches!(kind, DisableKind::DisableRequested { .. });
        let mut wrote_any = false;
        let mut refused_err = None;
        let mut last_err = None;
        for idx in 0..self.servo_ids.len() {
            if skip_dispatched && idx < 64 && (self.disable_dispatched_mask & (1u64 << idx)) != 0 {
                continue;
            }
            if let Err(e) = self.strict_stop_check(Some(stamp)) {
                if !wrote_any && self.disable_dispatched_mask == 0 {
                    return Err(e);
                }
                last_err = Some(ArmDriverError::HardwareFault(
                    "Client park disable stopped mid-batch after partial servo disable".into(),
                ));
                break;
            }
            let id = self.servo_ids[idx];
            match self.write_torque_off(id, kind) {
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
        if wrote_any {
            if let Err(e) = self.transport.flush() {
                last_err = Some(e.into());
            }
        }
        if let Some(e) = refused_err.or(last_err) {
            return Err(e);
        }
        if (self.disable_dispatched_mask & all_mask) == all_mask {
            self.disable_dispatched_mask = 0;
        }
        Ok(TorqueProgress::Done(TorqueOutcome::Unconfirmed))
    }

    fn write_estop_hold(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        let n = self.servo_ids.len().min(64);
        let mask = if n >= 64 { u64::MAX } else { (1u64 << n) - 1 };
        self.write_measured_hold_masked(mask, targets).map(|_| ())
    }

    fn write_estop_hold_bounded(&mut self, targets: &[JointTarget]) -> Result<TxBatchOutcome, ArmDriverError> {
        let n = self.servo_ids.len().min(64);
        let mask = if n >= 64 { u64::MAX } else { (1u64 << n) - 1 };
        self.write_measured_hold_masked(mask, targets).map(|r| r.outcome)
    }

    fn write_measured_hold_masked(
        &mut self,
        hold_mask: u64,
        targets: &[JointTarget],
    ) -> Result<HoldPassReport, ArmDriverError> {
        if targets.len() != self.servo_ids.len() {
            return Err(ArmDriverError::JointCountMismatch {
                expected: self.servo_ids.len(),
                actual: targets.len(),
            });
        }
        let start_stamp = self.current_stop_stamp();
        self.pending_stop_check(Some(start_stamp))?;
        let dof = self.servo_ids.len().min(crate::rt_core::mailbox::MAX_DOF);
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
            let angle_tenth = fashionstar::rad_to_tenth_deg(pos);
            let pkt = fashionstar::build_set_angle(self.servo_ids[i], angle_tenth, 20, 1000);
            self.pending_stop_check(Some(start_stamp))?;
            self.transport.write_all(&pkt)?;
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
            self.transport.flush()?;
            self.record_motion_write_done_masked(sent_mask);
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
        if targets.len() != self.servo_ids.len() {
            return Err(ArmDriverError::JointCountMismatch {
                expected: self.servo_ids.len(),
                actual: targets.len(),
            });
        }

        for (i, target) in targets.iter().enumerate() {
            self.rest_pose_window[i].reset();
            let angle_tenth = fashionstar::rad_to_tenth_deg(target.position);
            let pkt = fashionstar::build_set_angle(self.servo_ids[i], angle_tenth, 20, 1000);
            self.transport.write_all(&pkt)?;
        }
        self.transport.flush()?;
        self.record_motion_write_done_all();
        #[cfg(not(target_arch = "wasm32"))]
        if self.shadow_staging.is_enabled() {
            let now = self.now();
            let mut mask = crate::torque::ChannelMask::EMPTY;
            let mut outcomes = [crate::torque::MotorWriteOutcome::NotTargeted;
                crate::torque::MAX_CHANNELS];
            for i in 0..self.servo_ids.len().min(crate::torque::MAX_CHANNELS) {
                mask.insert(i as u8);
                outcomes[i] = crate::torque::MotorWriteOutcome::Sent;
            }
            if !mask.is_empty() {
                self.shadow_staging
                    .push(crate::torque::shadow::ShadowDriverObs::WriteResult {
                        device: 0,
                        kind: crate::torque::WriteKind::Motion,
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

    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let dof = self.servo_ids.len();
        if dof == 0 {
            return Ok(Vec::new());
        }

        let mut temp_chunk = [0u8; 1];
        let start_time = self.now();
        if !self.control_release_managed {
            self.control_release_id = self.control_release_id.wrapping_add(1).max(1);
        }
        let release_id = self.control_release_id;
        // Post-motion flush: once past `TORQUE_CAUSAL_SETTLE_WINDOW` after the latest motion or
        // enable write, clear any buffered pre-motion bytes before issuing qualifying angle queries.
        // Backends whose transport cannot clear its input buffer stay fail-closed (`rx_needs_post_motion_flush == true`).
        if self.rx_needs_post_motion_flush {
            let latest_motion = self.last_motion_write_at.iter().flatten().copied().max();
            let past_settle = latest_motion.is_none_or(|w| {
                start_time > w && start_time.saturating_duration_since(w) > TORQUE_CAUSAL_SETTLE_WINDOW
            });
            if past_settle && self.transport.clears_input_buffer() {
                self.rx_buffer.clear();
                self.rx_timestamps.clear();
                self.transport.clear_input_buffer();
                self.rx_needs_post_motion_flush = false;
            }
        }
        if self.line_poisoned {
            let mut saw_stray = false;
            let mut drain_buf = [0u8; 32];
            while let Ok(n) = self.transport.read(&mut drain_buf) {
                if n == 0 {
                    break;
                }
                saw_stray = true;
            }
            let now = self.now();
            if saw_stray {
                self.rx_buffer.clear();
                self.rx_timestamps.clear();
                self.poison_line(now);
            } else if self.line_poison_quiet_until.is_none_or(|until| now >= until) {
                self.rx_buffer.clear();
                self.rx_timestamps.clear();
                self.line_poisoned = false;
                self.line_poison_quiet_until = None;
            }
        }
        const MAX_READ_DURATION: Duration = Duration::from_millis(3);
        const MAX_TOTAL_BYTES: usize = 256;
        let mut total_bytes_read = 0;
        let start_cursor = self.poll_cursor;

        'poll: for offset in 0..dof {
            let joint_idx = (start_cursor + offset) % dof;
            let id = self.servo_ids[joint_idx];

            if self.since(start_time) >= MAX_READ_DURATION || total_bytes_read >= MAX_TOTAL_BYTES {
                self.poll_cursor = joint_idx;
                break 'poll;
            }

            let query = fashionstar::build_query_angle(id);
            self.transport.write_all(&query)?;
            self.transport.flush()?;
            let req_written_at = self.now();

            let mut attempts = 0;
            let mut got_reply = false;
            while attempts < 10 {
                if self.since(start_time) >= MAX_READ_DURATION || total_bytes_read >= MAX_TOTAL_BYTES {
                    self.poll_cursor = (joint_idx + 1) % dof;
                    break 'poll;
                }

                match fashionstar::parse_packet(&self.rx_buffer) {
                    Ok((FashionStarPacket { header, id: pkt_id, cmd_id, content }, consumed)) => {
                        let pkt_first_rx_at = self
                            .rx_timestamps
                            .first()
                            .copied()
                            .unwrap_or(req_written_at);
                        let pkt_rx_at = self
                            .rx_timestamps
                            .get(consumed.saturating_sub(1))
                            .copied()
                            .unwrap_or(req_written_at);
                        self.rx_buffer.drain(..consumed);
                        let ts_drain = consumed.min(self.rx_timestamps.len());
                        self.rx_timestamps.drain(..ts_drain);
                        // Filter out local echo of request frame ([0x12, 0x4C]) on half-duplex RS485 bus
                        if header == fashionstar::FASHIONSTAR_REQUEST_HEADER {
                            continue;
                        }
                        if header == fashionstar::FASHIONSTAR_RESPONSE_HEADER
                            && pkt_id == id
                            && cmd_id == fashionstar::CMD_QUERY_SERVO_ANGLE
                            && content.len() >= 2
                        {
                            got_reply = true;
                            let decode_now = self.now();
                            if pkt_rx_at > decode_now {
                                self.rest_pose_window[joint_idx].reset();
                                #[cfg(not(target_arch = "wasm32"))]
                                if self.shadow_staging.is_enabled()
                                    && joint_idx < crate::torque::MAX_CHANNELS
                                {
                                    let sample_id = self.shadow_staging.next_sample_id();
                                    self.shadow_staging.push(
                                        crate::torque::shadow::ShadowDriverObs::CausalSample {
                                            channel: joint_idx as u8,
                                            sample_id,
                                            stamp: crate::torque::shadow::RxStampKind::Future,
                                            decode_now,
                                            position_rad: self.cached_states[joint_idx].position,
                                            velocity_rad_s: None,
                                            temperature_c: None,
                                            reported_enabled: None,
                                            correlated_after_write_done: self.last_motion_write_at
                                                [joint_idx],
                                            poll_issued: Some(req_written_at),
                                            provenance:
                                                crate::torque::shadow::EvidenceProvenance::HardwareCausal,
                                        },
                                    );
                                }
                                break;
                            }
                            let tenth_deg = i16::from_le_bytes([content[0], content[1]]);
                            let pos = fashionstar::tenth_deg_to_rad(tenth_deg);
                            self.cached_states[joint_idx].position = pos;
                            if self.last_feedback_instant[joint_idx].is_none_or(|prev| {
                                pkt_rx_at.saturating_duration_since(prev) > STOPPED_DISABLE_FEEDBACK_MAX_AGE
                            }) || !pos.is_finite()
                            {
                                self.feedback_continuity_epoch[joint_idx] =
                                    self.feedback_continuity_epoch[joint_idx].wrapping_add(1);
                            }
                            self.last_feedback_instant[joint_idx] = Some(pkt_rx_at);
                            self.feedback_seq[joint_idx] = self.feedback_seq[joint_idx].wrapping_add(1);
                            let correlated_after_query = pkt_first_rx_at >= req_written_at;
                            let after_motion = self.last_motion_write_at[joint_idx].is_none_or(|w| {
                                req_written_at > w
                                    && req_written_at.saturating_duration_since(w)
                                        > TORQUE_CAUSAL_SETTLE_WINDOW
                                    && pkt_first_rx_at > w
                                    && pkt_first_rx_at.saturating_duration_since(w)
                                        > TORQUE_CAUSAL_SETTLE_WINDOW
                                    && pkt_rx_at > w
                                    && pkt_rx_at.saturating_duration_since(w)
                                        > TORQUE_CAUSAL_SETTLE_WINDOW
                            });
                            if let Some(ref gate) = self.stopped_disable_gate {
                                if gate.is_arm_joint(joint_idx) {
                                    if !self.line_poisoned
                                        && !self.rx_needs_post_motion_flush
                                        && correlated_after_query
                                        && after_motion
                                        && joint_state_valid(&self.cached_states[joint_idx], self.capabilities())
                                        && gate.joint_at_rest(joint_idx, pos)
                                    {
                                        let _ = self.rest_pose_window[joint_idx].record_agree(
                                            pkt_rx_at,
                                            release_id,
                                            STOPPED_DISABLE_FEEDBACK_MAX_AGE,
                                            None,
                                        );
                                    } else {
                                        self.rest_pose_window[joint_idx].record_disagree();
                                    }
                                }
                            }
                            #[cfg(not(target_arch = "wasm32"))]
                            if self.shadow_staging.is_enabled()
                                && joint_idx < crate::torque::MAX_CHANNELS
                            {
                                let sample_id = self.shadow_staging.next_sample_id();
                                self.shadow_staging.push(
                                    crate::torque::shadow::ShadowDriverObs::CausalSample {
                                        channel: joint_idx as u8,
                                        sample_id,
                                        stamp: crate::torque::shadow::RxStampKind::Valid {
                                            rx_at: pkt_rx_at,
                                            drift_ns: 0,
                                            live_post_settle: after_motion
                                                && correlated_after_query
                                                && !self.line_poisoned
                                                && !self.rx_needs_post_motion_flush,
                                        },
                                        decode_now,
                                        position_rad: pos,
                                        velocity_rad_s: None,
                                        temperature_c: None,
                                        reported_enabled: None,
                                        correlated_after_write_done: if correlated_after_query
                                            && after_motion
                                        {
                                            self.last_motion_write_at[joint_idx]
                                        } else {
                                            None
                                        },
                                        poll_issued: Some(req_written_at),
                                        provenance:
                                            crate::torque::shadow::EvidenceProvenance::HardwareCausal,
                                    },
                                );
                            }
                            break;
                        } else if header == fashionstar::FASHIONSTAR_RESPONSE_HEADER {
                            // Unsolicited/mismatched response on half-duplex RS485 line: fail closed
                            // until the line stays quiet for `TORQUE_CAUSAL_SETTLE_WINDOW`.
                            let now = self.now();
                            self.poison_line(now);
                        }
                    }
                    Err(fashionstar::FashionStarError::IncompleteFrame { .. }) => {
                        // Incomplete frame; read more bytes from transport
                    }
                    Err(_) => {
                        // Checksum mismatch or corruption: slide window by 1 byte to recover and
                        // fail-closed rest evidence until the RS485 line settles.
                        let now = self.now();
                        self.poison_line(now);
                        if !self.rx_buffer.is_empty() {
                            self.rx_buffer.remove(0);
                            if !self.rx_timestamps.is_empty() {
                                self.rx_timestamps.remove(0);
                            }
                        }
                    }
                }

                // Bound buffer size to prevent memory leaks / lockups
                if self.rx_buffer.len() > 1024 {
                    let trim = self.rx_buffer.len() - 256;
                    self.rx_buffer.drain(..trim);
                    let ts_trim = trim.min(self.rx_timestamps.len());
                    self.rx_timestamps.drain(..ts_trim);
                }

                match self.transport.read(&mut temp_chunk) {
                    Ok(n) if n > 0 => {
                        let rx_at = self.now();
                        total_bytes_read += n;
                        self.rx_buffer.extend_from_slice(&temp_chunk[..n]);
                        self.rx_timestamps.resize(self.rx_buffer.len(), rx_at);
                    }
                    _ => {
                        attempts += 1;
                        if attempts >= 3 {
                            break;
                        }
                    }
                }
            }
            if !got_reply {
                let now = self.now();
                self.poison_line(now);
            }
        }

        Ok(self.cached_states.clone())
    }
    #[cfg(not(target_arch = "wasm32"))]
    fn shadow_ext(&self) -> Option<&dyn ShadowExt> { Some(self) }
    #[cfg(not(target_arch = "wasm32"))]
    fn shadow_ext_mut(&mut self) -> Option<&mut dyn ShadowExt> { Some(self) }
}

#[cfg(not(target_arch = "wasm32"))]
impl<T: Transport> ShadowExt for FashionStarArmDriver<T> {
    fn shadow_vendor_kind(&self) -> crate::torque::VendorKind {
        crate::torque::VendorKind::FashionStar
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
