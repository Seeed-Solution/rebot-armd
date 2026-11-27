//! Feetech STS3215 serial bus-servo driver core, state, and `ArmDriver` implementation.

#[allow(unused_imports)]
use std::sync::atomic::{AtomicBool, Ordering};
#[allow(unused_imports)]
use std::sync::Arc;
#[allow(unused_imports)]
use std::time::{Duration, Instant};

#[allow(unused_imports)]
use crate::driver::*;
#[allow(unused_imports)]
use crate::model::JointState;
#[allow(unused_imports)]
use crate::protocol::feetech;
#[allow(unused_imports)]
use crate::transport::{Transport, TransportError};

#[derive(Clone, Copy, Debug)]
pub(super) struct PendingFeetechDisable {
    pub(super) kind: DisableKind,
    pub(super) client_park: Option<(Option<u64>, Option<u64>)>,
}

pub struct FeetechArmDriver<T: Transport> {
    pub(super) transport: T,
    pub(super) clock: Option<Arc<dyn crate::transport::DriverClock>>,
    pub(super) servo_ids: Vec<u8>,
    pub(super) config: feetech::FeetechConfig,
    pub(super) cached_states: Vec<JointState>,
    pub(super) cached_loads: Vec<f32>,
    pub motor_status: Vec<u8>,
    pub(super) last_feedback_instant: Vec<Option<Instant>>,
    pub(super) feedback_seq: Vec<u64>,
    /// Incremented whenever a joint's feedback continuity breaks (`> 100 ms` gap or non-finite).
    pub(super) feedback_continuity_epoch: Vec<u64>,
    /// Reported per-servo `REG_TORQUE_ENABLE` telemetry (`Some(0 | 1)` or `None`, observation/UI only).
    pub(super) reported_enable: Vec<Option<u8>>,
    /// True after torque enable was commanded until a torque-off write is dispatched; keeps
    /// periodic causal `REG_TORQUE_ENABLE` telemetry polling active even if a servo reports 0 (R12 M2).
    pub(super) torque_enable_commanded: bool,
    /// Monotonic counter bumped strictly when a causal `REG_TORQUE_ENABLE == 0` reply is decoded (R12 M2).
    pub(super) reported_torque_loss_sample_seq: u64,
    /// True after a 1-byte register read to servo `i` timed out: a late 1-byte reply may still
    /// be in flight, so causal torque readback requires a 0-byte `PING` FIFO barrier first.
    pub(super) unmatched_1b_read: Vec<bool>,
    pub(super) tx_echo: Option<bool>,
    pub(super) tx_echo_auto: bool,
    pub(super) poll_cursor: usize,
    pub(super) rx_buffer: Vec<u8>,
    pub effective_ranges: Vec<(u16, u16)>,
    pub(super) cached_raw_steps: Vec<i32>,
    pub(super) read_poll_count: u64,
    pub(super) missed_poll_replies: u64,
    pub(super) abort_flag: Option<Arc<AtomicBool>>,
    pub(super) estop_gate: Option<crate::rt_core::urgent::EstopGate>,
    pub(super) estop_state: Option<Arc<crate::rt_core::urgent::EstopState>>,
    pub(super) stopped_disable_gate: Option<StoppedDisableGate>,
    pub(super) tx_release_deadline: Option<Instant>,
    pub last_enable_poll: Option<Instant>,
    pub(super) gripper_index: Option<usize>,
    pub hw_angle_limits: Vec<Option<(u16, u16)>>,
    pub last_enable_confirmed: Vec<Option<Instant>>,
    pub missed_enable_polls: Vec<u8>,
    pub(super) incomplete_chunk_idx: Option<usize>,
    pub(super) incomplete_frame_started_at: Option<Instant>,
    pub(super) partial_pos_rx: Vec<u8>,
    /// Per-byte host monotonic receive timestamps aligned 1:1 with `partial_pos_rx` (S2).
    pub(super) partial_pos_rx_timestamps: Vec<Instant>,
    /// Per-byte host monotonic receive timestamps aligned 1:1 with `rx_buffer` (E3).
    pub(super) rx_timestamps: Vec<Instant>,
    /// True after any Feetech request times out or observes an ambiguous duplicate reply; the next
    /// request MUST discard RX bytes until continuous line idle (20 ms) before transmitting (S2).
    pub(super) line_poisoned: bool,
    /// Host monotonic instant when continuous silence on a poisoned serial line began (S2).
    pub(super) line_poison_quiet_since: Option<Instant>,
    /// Active control-release id distinguishing separate ticks for the rest-pose K=3 window.
    pub(super) control_release_id: u64,
    /// True once the owner has called `begin_control_release`, preventing receive passes from
    /// advancing `control_release_id` independently.
    pub(super) control_release_managed: bool,
    /// Most recent `control_release_id` for which `read_joint_states_inner` completed a bus pass,
    /// avoiding redundant same-release polling inside `poll_torque` (R12 B1).
    pub(super) last_read_release_id: u64,
    /// Pending stopped disable awaiting rest-pose K=3 confirmation or poisoned-line drain within
    /// the single 1 s request lifetime (0 torque-off packets transmitted while pending).
    pub(super) pending_disable: Option<PendingFeetechDisable>,
    /// Bitmask of 0-based servo indices that have already received `TORQUE_ENABLE = 0` in an
    /// active/partial disable sequence (R12 B3). Cleared on full completion or `set_torque(true)`.
    pub(super) disable_dispatched_mask: u64,
    /// Per-servo host monotonic instant of the last motion/enable write (`write_joint_targets` or
    /// `set_torque(true)`): rest-pose samples must come from requests sent at/after this instant.
    pub(super) last_motion_write_at: Vec<Option<Instant>>,
    /// `(frames, exhausted)` summary of the most recent `poll_torque` pass.
    pub(super) last_poll_rx: Option<(u16, bool)>,
    /// Per-servo K=3 causal confirmation window for the rest-pose gate.
    pub(super) rest_pose_window: Vec<CausalWindow>,
    /// Preallocated goal-position scratch buffer for zero-alloc `FailedHold` / stopped holds (R1).
    pub(super) hold_positions_buf: Vec<(u8, u16)>,
    /// Preallocated `SYNC_WRITE` packet buffer for zero-alloc `FailedHold` / stopped holds (R1).
    pub(super) hold_sync_pkt_buf: Vec<u8>,
    pub(super) consecutive_missed_echoes: u32,
    pub(super) warned_missed_echoes: bool,
    #[cfg(not(target_arch = "wasm32"))]
    pub(super) shadow_staging: crate::torque::shadow::ShadowDriverStaging,
    #[cfg(not(target_arch = "wasm32"))]
    pub(super) last_disable_flush_ok: bool,
    #[cfg(not(target_arch = "wasm32"))]
    pub(super) in_disable_drain: bool,
    #[cfg(not(target_arch = "wasm32"))]
    pub(super) pending_disable_shadow: Option<PendingDisableShadowBatch>,
}

impl<T: Transport> FeetechArmDriver<T> {
    pub fn new(transport: T, servo_ids: Vec<u8>) -> Self {
        let clock = transport.clock();
        let dof = servo_ids.len();
        let hold_cap = dof.max(crate::rt_core::mailbox::MAX_DOF);
        let joints: Vec<feetech::FeetechJointCalib> = servo_ids
            .iter()
            .map(|&id| feetech::FeetechJointCalib {
                id,
                ..Default::default()
            })
            .collect();
        let effective_ranges = joints.iter().map(|j| j.effective_range_steps()).collect();
        Self {
            transport,
            clock,
            servo_ids,
            config: feetech::FeetechConfig {
                joints,
                ..Default::default()
            },
            cached_states: vec![JointState::default(); dof],
            cached_loads: vec![0.0; dof],
            motor_status: vec![0; dof],
            last_feedback_instant: vec![None; dof],
            feedback_seq: vec![0; dof],
            feedback_continuity_epoch: vec![0; dof],
            reported_enable: vec![None; dof],
            torque_enable_commanded: false,
            reported_torque_loss_sample_seq: 0,
            unmatched_1b_read: vec![false; dof],
            tx_echo: None,
            tx_echo_auto: false,
            poll_cursor: 0,
            rx_buffer: Vec::with_capacity(256),
            effective_ranges,
            cached_raw_steps: vec![0; dof],
            read_poll_count: 0,
            missed_poll_replies: 0,
            abort_flag: None,
            estop_gate: None,
            estop_state: None,
            stopped_disable_gate: None,
            tx_release_deadline: None,
            last_enable_poll: None,
            gripper_index: None,
            hw_angle_limits: vec![None; dof],
            last_enable_confirmed: vec![None; dof],
            missed_enable_polls: vec![0; dof],
            incomplete_chunk_idx: None,
            incomplete_frame_started_at: None,
            partial_pos_rx: Vec::with_capacity(64),
            partial_pos_rx_timestamps: Vec::with_capacity(64),
            rx_timestamps: Vec::with_capacity(256),
            line_poisoned: false,
            line_poison_quiet_since: None,
            control_release_id: 0,
            control_release_managed: false,
            last_read_release_id: 0,
            pending_disable: None,
            disable_dispatched_mask: 0,
            last_motion_write_at: vec![None; dof],
            last_poll_rx: None,
            rest_pose_window: vec![CausalWindow::default(); dof],
            hold_positions_buf: Vec::with_capacity(hold_cap),
            hold_sync_pkt_buf: Vec::with_capacity(16 + hold_cap * 3),
            consecutive_missed_echoes: 0,
            warned_missed_echoes: false,
            #[cfg(not(target_arch = "wasm32"))]
            shadow_staging: crate::torque::shadow::ShadowDriverStaging::new(false),
            #[cfg(not(target_arch = "wasm32"))]
            last_disable_flush_ok: true,
            #[cfg(not(target_arch = "wasm32"))]
            in_disable_drain: false,
            #[cfg(not(target_arch = "wasm32"))]
            pending_disable_shadow: None,
        }
    }

    pub fn with_gripper_index(mut self, gripper_index: Option<usize>) -> Self {
        self.gripper_index = gripper_index;
        self
    }

    pub fn with_zero_offsets(mut self, offsets: Vec<u16>) -> Self {
        for (i, &offset) in offsets.iter().enumerate() {
            if i < self.config.joints.len() {
                self.config.joints[i].zero_step = offset;
            }
        }
        self
    }

    pub fn with_calibrated(mut self, calibrated: bool) -> Self {
        self.config.calibrated = calibrated;
        self
    }

    pub fn missed_poll_replies(&self) -> u64 {
        self.missed_poll_replies
    }

    pub fn config(&self) -> &feetech::FeetechConfig {
        &self.config
    }

    pub fn effective_ranges(&self) -> &[(u16, u16)] {
        &self.effective_ranges
    }

    pub fn with_config(mut self, cfg: feetech::FeetechConfig) -> Self {
        if cfg.tx_echo.is_some() {
            self.tx_echo = cfg.tx_echo;
            self.tx_echo_auto = false;
        }
        let old_ids = self.servo_ids.clone();
        let old_joints = self.config.joints.clone();
        self.config = cfg;
        if self.config.joints.is_empty() {
            self.config.joints = self
                .servo_ids
                .iter()
                .map(|&id| feetech::FeetechJointCalib {
                    id,
                    ..Default::default()
                })
                .collect();
        } else {
            self.servo_ids = self.config.joints.iter().map(|j| j.id).collect();
        }

        let dof = self.servo_ids.len();
        if self.servo_ids != old_ids || self.config.joints != old_joints {
            self.cached_states = vec![JointState::default(); dof];
            self.cached_loads = vec![0.0; dof];
            self.motor_status = vec![0; dof];
            self.last_feedback_instant = vec![None; dof];
            self.feedback_seq.resize(dof, 0);
            self.feedback_continuity_epoch.resize(dof, 0);
            self.cached_raw_steps = vec![0; dof];
            self.last_enable_confirmed = vec![None; dof];
            self.missed_enable_polls = vec![0; dof];
            if self.servo_ids != old_ids {
                self.reported_enable = vec![None; dof];
                self.unmatched_1b_read = vec![false; dof];
                self.poll_cursor = 0;
                self.hw_angle_limits = vec![None; dof];
                self.invalidate_partial_pos_rx();
            } else if self.hw_angle_limits.len() != dof {
                self.hw_angle_limits.resize(dof, None);
            }
        }
        self.effective_ranges = self
            .config
            .joints
            .iter()
            .enumerate()
            .map(|(i, calib)| {
                let min_s = calib.range_min_steps
                    .or_else(|| self.hw_angle_limits.get(i).copied().flatten().map(|x| x.0))
                    .unwrap_or(0);
                let max_s = calib.range_max_steps
                    .or_else(|| self.hw_angle_limits.get(i).copied().flatten().map(|x| x.1))
                    .unwrap_or(4095);
                (min_s, max_s)
            })
            .collect();
        if self.effective_ranges.len() != dof {
            self.effective_ranges.resize(dof, (0, 4095));
        }
        let hold_cap = dof.max(crate::rt_core::mailbox::MAX_DOF);
        if self.hold_positions_buf.capacity() < hold_cap {
            self.hold_positions_buf.reserve(hold_cap - self.hold_positions_buf.len());
        }
        let pkt_cap = 16 + hold_cap * 3;
        if self.hold_sync_pkt_buf.capacity() < pkt_cap {
            self.hold_sync_pkt_buf.reserve(pkt_cap - self.hold_sync_pkt_buf.len());
        }
        self
    }

    pub fn with_tx_echo(mut self, tx_echo: Option<bool>) -> Self {
        self.tx_echo = tx_echo;
        self.tx_echo_auto = false;
        self
    }

    pub fn with_port_name(mut self, port: &str) -> Self {
        if port.starts_with("/dev/ttySC") && self.tx_echo.is_none() {
            self.tx_echo = Some(true);
            self.tx_echo_auto = true;
        }
        self
    }

    pub fn tx_echo(&self) -> Option<bool> {
        self.tx_echo
    }

    pub fn tx_echo_auto(&self) -> bool {
        self.tx_echo_auto
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

    #[doc(hidden)]
    pub fn confirmed_enable_state(&self) -> EnableState {
        self.enable_state()
    }

    #[doc(hidden)]
    pub fn set_tx_echo_state(&mut self, tx_echo: Option<bool>, auto: bool) {
        self.tx_echo = tx_echo;
        self.tx_echo_auto = auto;
    }

    #[doc(hidden)]
    pub fn read_poll_count(&self) -> u64 {
        self.read_poll_count
    }

    #[doc(hidden)]
    pub fn set_read_poll_count(&mut self, count: u64) {
        self.read_poll_count = count;
    }

    #[doc(hidden)]
    pub fn incomplete_chunk_idx(&self) -> Option<usize> {
        self.incomplete_chunk_idx
    }

    #[doc(hidden)]
    pub fn set_incomplete_chunk_idx(&mut self, idx: Option<usize>) {
        self.incomplete_chunk_idx = idx;
    }

    #[doc(hidden)]
    pub fn incomplete_frame_started_at(&self) -> Option<Instant> {
        self.incomplete_frame_started_at
    }

    #[doc(hidden)]
    pub fn set_incomplete_frame_started_at(&mut self, instant: Option<Instant>) {
        self.incomplete_frame_started_at = instant;
    }

    #[doc(hidden)]
    pub fn partial_pos_rx(&self) -> &[u8] {
        &self.partial_pos_rx
    }

    #[doc(hidden)]
    pub fn set_partial_pos_rx(&mut self, bytes: &[u8]) {
        self.partial_pos_rx = bytes.to_vec();
        let now = self.clock_now();
        self.partial_pos_rx_timestamps = vec![now; bytes.len()];
    }

    pub(crate) fn invalidate_partial_pos_rx(&mut self) {
        self.partial_pos_rx.clear();
        self.partial_pos_rx_timestamps.clear();
        self.incomplete_chunk_idx = None;
        self.incomplete_frame_started_at = None;
    }

    pub(super) fn is_aborted(&self) -> bool {
        self.estop_state.as_ref().is_some_and(|s| s.is_latched())
            || self
                .abort_flag
                .as_ref()
                .is_some_and(|f| f.load(Ordering::SeqCst))
    }

    pub fn clock_now(&self) -> Instant {
        self.clock.as_ref().map_or_else(Instant::now, |c| c.now())
    }

    pub(super) fn elapsed_since(&self, earlier: Instant) -> Duration {
        self.clock_now().saturating_duration_since(earlier)
    }

    pub(super) fn clock_sleep(&self, duration: Duration) {
        let track = crate::diagnostics::tick::hold_io_active();
        let t0 = track.then(Instant::now);
        if let Some(ref c) = self.clock {
            c.sleep(duration);
        } else {
            std::thread::sleep(duration);
        }
        if let Some(t0) = t0 {
            let wall_ns = t0.elapsed().as_nanos().min(u128::from(u64::MAX)) as u64;
            let d_ns = duration.as_nanos().min(u128::from(u64::MAX)) as u64;
            crate::diagnostics::tick::record_hold_sleep_ns(wall_ns.max(d_ns));
        }
    }

    pub(super) fn clock_yield(&self) {
        if let Some(ref c) = self.clock {
            c.yield_now();
        } else {
            std::thread::yield_now();
        }
    }

    pub(super) fn assert_clock_matches_transport(&self) {
        let captured_id = self.clock.as_ref().map(|c| c.clock_id());
        let transport_id = self.transport.clock().map(|c| c.clock_id());
        assert_eq!(
            captured_id, transport_id,
            "FeetechArmDriver transport clock domain changed after construction (cannot replace transport with a different clock)"
        );
    }

    /// Per-write Stop / E-stop / stop-generation gate for client-initiated park disable on
    /// Feetech / SO-ARM101 servos (rechecked before EVERY per-servo disable write).
    pub(super) fn client_park_stop_check(
        &self,
        initial_estop_gen: Option<u64>,
        expected_stop_gen: Option<u64>,
    ) -> Result<(), ArmDriverError> {
        if self.is_aborted() || self.estop_state.as_ref().is_some_and(|s| s.is_latched()) {
            return Err(ArmDriverError::EstopInhibited);
        }
        if let Some(ref gate) = self.estop_gate {
            if gate.pending() {
                return Err(ArmDriverError::EstopInhibited);
            }
            let (estop_gen, stop_gen) = gate.generations();
            if initial_estop_gen.is_some_and(|expected| estop_gen != expected)
                || expected_stop_gen.is_some_and(|expected| stop_gen != expected)
            {
                return Err(ArmDriverError::EstopInhibited);
            }
        }
        Ok(())
    }

    #[inline]
    pub(super) fn current_stop_stamp(&self) -> (u64, u64) {
        self.estop_gate
            .as_ref()
            .map_or((0, 0), crate::rt_core::urgent::EstopGate::generations)
    }

    #[inline]
    pub(super) fn strict_stop_check(&self, expected_stamp: Option<(u64, u64)>) -> Result<(), ArmDriverError> {
        if self.is_aborted()
            || expected_stamp.is_some_and(|s| self.current_stop_stamp() != s)
        {
            return Err(ArmDriverError::EstopInhibited);
        }
        if self.tx_release_deadline.is_some_and(|dl| self.clock_now() >= dl) {
            return Err(ArmDriverError::Transport(TransportError::Timeout));
        }
        Ok(())
    }

    #[inline]
    pub(super) fn pending_stop_check(&self, expected_stamp: Option<(u64, u64)>) -> Result<(), ArmDriverError> {
        if self.estop_gate.as_ref().is_some_and(|g| g.pending())
            || expected_stamp.is_some_and(|s| self.current_stop_stamp() != s)
        {
            return Err(ArmDriverError::EstopInhibited);
        }
        if self.tx_release_deadline.is_some_and(|dl| self.clock_now() >= dl) {
            return Err(ArmDriverError::Transport(TransportError::Timeout));
        }
        Ok(())
    }

    pub(super) fn is_benign_gripper_overload(&self, joint_idx: usize, status: u8) -> bool {
        self.gripper_index == Some(joint_idx) && status == 0x20
    }

}

impl<T: Transport> ArmDriver for FeetechArmDriver<T> {
    fn motor_zero_support(&self) -> MotorZeroSupport {
        MotorZeroSupport::unsupported("Feetech: the zero is the profile's feetech.joints calibration (zero_step, \
            homing_offset_steps), written by the SO-ARM calibration, not a motor command in armd")
    }

    fn rest_pose_gate_status(&self) -> RestPoseGateStatus {
        self.evaluate_rest_pose_gate()
    }

    fn supports_budgeted_disable_await(&self) -> bool {
        true
    }

    fn driver_clock(&self) -> Option<std::sync::Arc<dyn crate::transport::DriverClock>> {
        self.clock.clone()
    }

    fn begin_control_release(&mut self, release_seq: u64) {
        self.control_release_managed = true;
        self.control_release_id = release_seq;
    }

    fn position_limits(&self, joint: usize) -> Option<(f32, f32)> {
        self.effective_ranges.get(joint).map(|&(min_s, max_s)| {
            let calib = self.config.joint_calib(joint);
            let r0 = calib.steps_to_rad(min_s as i32);
            let r1 = calib.steps_to_rad(max_s as i32);
            (r0.min(r1), r0.max(r1))
        })
    }

    fn set_gripper_index(&mut self, index: Option<usize>) {
        self.assert_clock_matches_transport();
        self.gripper_index = index;
    }

    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            velocity_feedback: true,
            torque_feedback: false,
            temperature_feedback: true,
            torque_feedforward: false,
            enable_feedback: true,
            load_feedback: true,
        }
    }

    fn enable_state(&self) -> EnableState {
        if self.reported_enable.is_empty() {
            return EnableState::Unknown;
        }
        if self.reported_enable.iter().all(|v| *v == Some(0)) {
            EnableState::Disabled
        } else if self.reported_enable.iter().all(|v| *v == Some(1)) {
            EnableState::Enabled
        } else {
            EnableState::Unknown
        }
    }

    fn is_hardware_disarmed_confirmed(&self) -> bool {
        !self.servo_ids.is_empty()
            && !self.torque_enable_commanded
            && self.pending_disable.is_none()
            && self.disable_dispatched_mask == 0
            && self.is_feedback_valid()
            && self.reported_enable.len() == self.servo_ids.len()
            && self.last_enable_confirmed.len() == self.servo_ids.len()
            && self.reported_enable.iter().all(|v| *v == Some(0))
            && self
                .last_enable_confirmed
                .iter()
                .all(|ts| ts.is_some_and(|t| self.elapsed_since(t) <= Duration::from_millis(600)))
    }

    fn reported_torque_loss_sample_seq(&self) -> Option<u64> {
        Some(self.reported_torque_loss_sample_seq)
    }

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
        FeetechArmDriver::clock_now(self)
    }

    fn clock_pause(&self, d: Duration) {
        self.clock_sleep(d);
    }

    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        self.last_feedback_instant
            .iter()
            .map(|t| t.map(|t| self.elapsed_since(t)))
            .collect()
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
                .is_some_and(|t| self.elapsed_since(t) <= max_age);
            if !fresh {
                return false;
            }
            let Some(s) = self.cached_states.get(i) else {
                return false;
            };
            if !joint_state_valid(s, caps) {
                return false;
            }
            if caps.load_feedback && !self.cached_loads.get(i).is_some_and(|l| l.is_finite()) {
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
            .all(|opt| opt.is_some_and(|instant| self.elapsed_since(instant) <= FRESHNESS_LIMIT))
    }

    fn is_enabled(&self) -> bool {
        self.enable_state() == EnableState::Enabled && self.is_feedback_valid()
    }

    fn read_joint_states_budgeted(
        &mut self,
        _out: &mut Vec<JointState>,
        _deadline: Instant,
        _max_rx_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        Err(ArmDriverError::ProtocolStatic("UNSUPPORTED: serial driver does not support budgeted RX"))
    }

    fn read_joint_states_safe_hold(
        &mut self,
        out: &mut Vec<JointState>,
        _deadline: Instant,
        _max_rx_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        self.read_joint_states_into(out)?;
        Ok((out.len() as u16, false))
    }

    fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
        out.clear();
        out.extend_from_slice(&self.cached_states);
        Some(())
    }

    fn begin_torque_budgeted(&mut self, enabled: bool, _max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
        if !enabled {
            let started_at = self.clock_now();
            return self.begin_disable_torque_budgeted(
                DisableKind::DisableRequested { started_at },
                _max_rx_frames,
            );
        }
        Err(ArmDriverError::ProtocolStatic("UNSUPPORTED: serial driver does not support budgeted torque transitions"))
    }

    fn poll_torque(&mut self, deadline: Instant) -> Result<TorqueProgress, ArmDriverError> {
        self.poll_disable_step(deadline, u16::MAX)
    }

    fn poll_torque_budgeted(&mut self, deadline: Instant, max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
        if self.pending_disable.is_some() {
            return self.poll_disable_step(deadline, max_rx_frames);
        }
        Err(ArmDriverError::ProtocolStatic("UNSUPPORTED: serial driver does not support budgeted torque transitions"))
    }

    fn take_last_poll_rx(&mut self) -> Option<(u16, bool)> {
        self.last_poll_rx.take()
    }

    fn torque_transition_pending(&self) -> bool {
        self.pending_disable.is_some()
    }

    fn disable_frames_dispatched(&self) -> bool {
        self.disable_dispatched_mask != 0
    }

    fn disable_dispatched_mask(&self) -> u64 {
        self.disable_dispatched_mask
    }

    fn abort_pending_disable(&mut self) -> bool {
        match self.pending_disable {
            None => true,
            Some(ref p)
                if matches!(p.kind, DisableKind::DisableRequested { .. })
                    && self.disable_dispatched_mask == 0 =>
            {
                self.pending_disable = None;
                true
            }
            Some(_) => false,
        }
    }

    fn motor_status(&self) -> Option<Vec<u8>> {
        Some(self.motor_status.clone())
    }

    fn motor_status_into(&self, out: &mut Vec<u8>) -> bool {
        out.clear();
        out.extend_from_slice(&self.motor_status);
        true
    }

    fn joint_loads(&self) -> Option<Vec<f32>> {
        Some(self.cached_loads.clone())
    }

    fn joint_loads_into(&self, out: &mut Vec<f32>) -> bool {
        out.clear();
        out.extend_from_slice(&self.cached_loads);
        true
    }

    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
        let now = self.clock_now();
        out.clear();
        out.extend(
            self.last_feedback_instant
                .iter()
                .map(|opt| opt.map(|t| now.saturating_duration_since(t))),
        );
    }

    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
        let now = self.clock_now();
        out.clear();
        out.extend(self.last_feedback_instant.iter().map(|opt| {
            opt.map(|t| now.saturating_duration_since(t).as_millis().min(u64::MAX as u128) as u64)
        }));
    }

    fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        out.clear();
        out.extend_from_slice(&self.last_feedback_instant);
    }

    fn feedback_seqs_into(&self, out: &mut Vec<u64>) {
        out.clear();
        out.extend_from_slice(&self.feedback_seq);
    }

    fn feedback_queue_bounds_into(&self, out: &mut Vec<Option<Instant>>) {
        out.clear();
    }

    fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        out.clear();
        out.extend_from_slice(&self.last_feedback_instant);
    }

    fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
        let now = self.clock_now();
        self.last_feedback_instant.len() == dof
            && self
                .last_feedback_instant
                .iter()
                .all(|opt| opt.is_some_and(|t| now.saturating_duration_since(t) <= max_age))
    }

    fn decode_motor_error(&self, code: u8) -> Option<String> {
        feetech::feetech_error_name(code)
    }

    fn decode_motor_error_static(&self, _code: u8) -> Option<&'static str> {
        None
    }

    fn check_hardware_status(&self) -> Result<(), ArmDriverError> {
        for (i, &status) in self.motor_status.iter().enumerate() {
            if status != 0 {
                if self.is_benign_gripper_overload(i, status) {
                    continue;
                }
                let id = self.servo_ids.get(i).copied().unwrap_or(0);
                let desc = feetech::feetech_error_name(status)
                    .unwrap_or_else(|| format!("0x{:02X}", status));
                return Err(ArmDriverError::HardwareFault(format!(
                    "Feetech joint {} (ID {}) fault 0x{:02X}: {}",
                    i, id, status, desc
                )));
            }
        }
        Ok(())
    }

    fn clear_fault_disables_joints(&self) -> Vec<usize> {
        self.motor_status
            .iter()
            .enumerate()
            .filter(|(i, &s)| s != 0 && !self.is_benign_gripper_overload(*i, s))
            .map(|(i, _)| i)
            .collect()
    }

    fn clear_fault_disables_arm_joint(&self, arm_dof: usize) -> bool {
        self.motor_status
            .iter()
            .enumerate()
            .any(|(i, &s)| i < arm_dof && s != 0 && !self.is_benign_gripper_overload(i, s))
    }

    fn has_hardware_fault(&self) -> bool {
        self.motor_status
            .iter()
            .enumerate()
            .any(|(i, &s)| s != 0 && !self.is_benign_gripper_overload(i, s))
    }

    fn has_known_partial_enable(&self) -> bool {
        !self.reported_enable.is_empty()
            && self.reported_enable.iter().all(Option::is_some)
            && self.reported_enable.contains(&Some(1))
            && self.reported_enable.contains(&Some(0))
    }

    fn has_any_joint_enabled(&self) -> bool {
        self.reported_enable.contains(&Some(1))
    }

    fn set_abort_flag(&mut self, flag: Arc<AtomicBool>) {
        self.assert_clock_matches_transport();
        self.abort_flag = Some(flag);
    }

    fn set_estop_gate(&mut self, gate: crate::rt_core::urgent::EstopGate) {
        self.assert_clock_matches_transport();
        self.estop_gate = Some(gate);
    }

    fn begin_client_park_disable_with_kind(
        &mut self,
        kind: DisableKind,
        max_rx_frames: Option<u16>,
        expected_stop_gen: Option<u64>,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.assert_clock_matches_transport();
        let initial_estop_gen = self.estop_gate.as_ref().map(|g| g.generations().0);
        self.client_park_stop_check(initial_estop_gen, expected_stop_gen)?;
        let client_park = Some((initial_estop_gen, expected_stop_gen));
        let exp = match kind {
            DisableKind::DisableRequested { started_at } => started_at + STOPPED_DISABLE_REQUEST_LIFETIME,
            DisableKind::DisableProtective { .. } => self.clock_now(),
        };
        if matches!(kind, DisableKind::DisableRequested { .. }) && self.clock_now() >= exp {
            return Err(ArmDriverError::DisableRequestExpired);
        }
        match self.evaluate_rest_pose_gate() {
            RestPoseGateStatus::Confirmed => {}
            RestPoseGateStatus::PendingDrainOrWindow | RestPoseGateStatus::StaleOrMissing => {
                self.pending_disable = Some(PendingFeetechDisable { kind, client_park });
                self.last_poll_rx = Some((0, false));
                let wake = (self.clock_now() + EVIDENCE_PROBE_INTERVAL).min(exp);
                return Ok(TorqueProgress::Pending { wake });
            }
            RestPoseGateStatus::RefusedNonRest => {
                return Err(ArmDriverError::TorqueOffWouldDrop);
            }
        }
        let (done, rx_used, exhausted) =
            self.execute_disable_writes_budgeted(kind, client_park, None, max_rx_frames)?;
        self.last_poll_rx = Some((rx_used, exhausted));
        if done {
            Ok(TorqueProgress::Done(TorqueOutcome::Confirmed))
        } else {
            let wake = (self.clock_now() + EVIDENCE_PROBE_INTERVAL).min(exp);
            Ok(TorqueProgress::Pending { wake })
        }
    }
    fn set_estop_state(&mut self, state: Arc<crate::rt_core::urgent::EstopState>) {
        self.assert_clock_matches_transport();
        self.estop_state = Some(state);
    }

    fn set_stopped_disable_gate(&mut self, gate: StoppedDisableGate) {
        self.stopped_disable_gate = Some(gate);
    }

    fn refresh_enable_state(&mut self) -> Result<EnableState, ArmDriverError> {
        self.assert_clock_matches_transport();
        if self.servo_ids.is_empty() {
            return Ok(self.enable_state());
        }
        let ids = self.servo_ids.clone();
        let _ = self.read_torque_enable_states(&ids);
        self.last_enable_poll = Some(self.clock_now());
        Ok(self.enable_state())
    }

    fn clear_fault(&mut self) -> Result<(), ArmDriverError> {
        self.assert_clock_matches_transport();
        let faulted_ids: Vec<u8> = self
            .motor_status
            .iter()
            .enumerate()
            .filter(|(i, &s)| s != 0 && !self.is_benign_gripper_overload(*i, s))
            .filter_map(|(i, _)| self.servo_ids.get(i).copied())
            .collect();

        // Route every Feetech `TORQUE_ENABLE = 0` fault-reset write through the transmit choke
        // point (`write_torque_off`) as `DisableRequested` so Rule A′ is enforced at send time.
        let started_at = self.clock_now();
        let mut wrote_any = false;
        let mut write_err = None;
        for &id in &faulted_ids {
            match self.write_torque_off(id, DisableKind::DisableRequested { started_at }) {
                Ok(()) => wrote_any = true,
                Err(e @ (ArmDriverError::TorqueOffWouldDrop | ArmDriverError::DisableRequestExpired)) => {
                    if !wrote_any {
                        return Err(e);
                    }
                    write_err = Some(e);
                }
                Err(e) => {
                    write_err = Some(e);
                }
            }
        }

        if let Some(e) = write_err {
            return Err(e);
        }

        let ids = self.servo_ids.clone();
        let (_, _, first_error) = self.read_torque_enable_states(&ids);

        if let Some((servo_id, error_code)) = first_error {
            return Err(ArmDriverError::FeetechReadbackFailed {
                servo_id,
                error_code,
            });
        }

        Ok(())
    }

    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        self.assert_clock_matches_transport();
        self.invalidate_partial_pos_rx();
        let ids = self.servo_ids.clone();
        if enabled {
            self.pending_disable = None;
            self.disable_dispatched_mask = 0;
            self.check_hardware_status()?;
            if self.is_aborted() {
                return Err(ArmDriverError::HardwareFault("Aborted by emergency stop".into()));
            }
            if !self.config.calibrated {
                return Err(ArmDriverError::HardwareFault(
                    "Feetech arm is uncalibrated: calibrate servos and set feetech.calibrated=true".into(),
                ));
            }
            if !self.is_feedback_valid() {
                if self.last_feedback_instant.iter().any(Option::is_some) {
                    return Err(ArmDriverError::HardwareFault(
                        "Stale joint feedback (>250ms); refusing to enable torque with unverified/stale goal positions".into(),
                    ));
                }
                return Err(ArmDriverError::HardwareFault(
                    "No valid joint feedback received (<250ms); refusing to enable torque with unverified/stale goal positions".into(),
                ));
            }
            let default_calib = feetech::FeetechJointCalib::default();
            for (i, &raw) in self.cached_raw_steps.iter().enumerate() {
                let calib = self.config.joints.get(i).unwrap_or(&default_calib);
                let range = self.effective_ranges.get(i).copied().unwrap_or_else(|| calib.effective_range_steps());
                let pos_rad = calib.steps_to_rad(raw);
                calib.checked_rad_to_steps_in_range(pos_rad, range).map_err(|e| {
                    ArmDriverError::HardwareFault(format!(
                        "Joint {} (ID {}) raw position {} outside calibrated range: {}",
                        i, self.servo_ids[i], raw, e
                    ))
                })?;
            }

            // Pre-seed measured goal positions for every servo that is not currently reporting enabled.
            let mut positions_to_write = Vec::new();
            for (i, s) in self.cached_states.iter().enumerate() {
                if self.reported_enable.get(i).copied().flatten() == Some(1) {
                    continue;
                }
                let calib = self.config.joints.get(i).unwrap_or(&default_calib);
                let range = self.effective_ranges.get(i).copied().unwrap_or_else(|| calib.effective_range_steps());
                let raw_steps = calib.checked_rad_to_steps_in_range(s.position, range)
                    .map_err(|e| ArmDriverError::Protocol(e.to_string()))?;
                positions_to_write.push((self.servo_ids[i], raw_steps));
            }
            if !positions_to_write.is_empty() {
                self.discard_pending_rx_for_request()?;
                let sync_pkt = feetech::build_sync_write_positions(&positions_to_write)
                    .map_err(|e| ArmDriverError::Protocol(e.to_string()))?;
                self.transport.write_all(&sync_pkt)?;
                self.transport.flush()?;
                self.drain_tx_echo(&sync_pkt)?;
            }

            let start_stamp = self.current_stop_stamp();
            let mut write_err = None;
            for (idx, &id) in ids.iter().enumerate() {
                if let Err(e) = self.strict_stop_check(Some(start_stamp)) {
                    write_err = Some(e);
                    break;
                }
                // Invalidate this servo's reported enable telemetry and rest-pose window BEFORE
                // the write reaches the transport.
                self.reported_enable[idx] = None;
                self.last_enable_confirmed[idx] = None;
                self.rest_pose_window[idx].reset();
                if let Err(e) = self.discard_pending_rx_for_request() {
                    write_err = Some(e);
                    break;
                }
                let pkt = feetech::build_set_torque(id, true);
                if let Err(e) = self.transport.write_all(&pkt) {
                    #[cfg(not(target_arch = "wasm32"))]
                    if self.shadow_staging.is_enabled() && idx < crate::torque::MAX_CHANNELS {
                        let now = self.clock_now();
                        let mut outcomes = [crate::torque::MotorWriteOutcome::NotTargeted;
                            crate::torque::MAX_CHANNELS];
                        outcomes[idx] = if matches!(
                            e,
                            TransportError::WouldBlock | TransportError::NoBufferSpace
                        ) || matches!(
                            &e,
                            TransportError::Io(io) if io.kind() == std::io::ErrorKind::WouldBlock
                        ) {
                            crate::torque::MotorWriteOutcome::Deferred
                        } else {
                            crate::torque::MotorWriteOutcome::Failed
                        };
                        self.shadow_staging.push(
                            crate::torque::shadow::ShadowDriverObs::WriteResult {
                                device: 0,
                                kind: crate::torque::WriteKind::Enable,
                                write_done: now,
                                now,
                                target_mask: crate::torque::ChannelMask::bit(idx as u8),
                                outcomes,
                                ambiguous_gap: false,
                            },
                        );
                    }
                    write_err = Some(e.into());
                    break;
                }
                self.torque_enable_commanded = true;
                if let Err(e) = self.strict_stop_check(Some(start_stamp)) {
                    #[cfg(not(target_arch = "wasm32"))]
                    if self.shadow_staging.is_enabled() && idx < crate::torque::MAX_CHANNELS {
                        let now = self.clock_now();
                        let mut outcomes = [crate::torque::MotorWriteOutcome::NotTargeted;
                            crate::torque::MAX_CHANNELS];
                        outcomes[idx] = crate::torque::MotorWriteOutcome::Ambiguous;
                        self.shadow_staging.push(
                            crate::torque::shadow::ShadowDriverObs::WriteResult {
                                device: 0,
                                kind: crate::torque::WriteKind::Enable,
                                write_done: now,
                                now,
                                target_mask: crate::torque::ChannelMask::bit(idx as u8),
                                outcomes,
                                ambiguous_gap: true,
                            },
                        );
                    }
                    write_err = Some(e);
                    break;
                }
                if let Err(e) = self.transport.flush() {
                    #[cfg(not(target_arch = "wasm32"))]
                    if self.shadow_staging.is_enabled() && idx < crate::torque::MAX_CHANNELS {
                        let now = self.clock_now();
                        let mut outcomes = [crate::torque::MotorWriteOutcome::NotTargeted;
                            crate::torque::MAX_CHANNELS];
                        outcomes[idx] = crate::torque::MotorWriteOutcome::Ambiguous;
                        self.shadow_staging.push(
                            crate::torque::shadow::ShadowDriverObs::WriteResult {
                                device: 0,
                                kind: crate::torque::WriteKind::Enable,
                                write_done: now,
                                now,
                                target_mask: crate::torque::ChannelMask::bit(idx as u8),
                                outcomes,
                                ambiguous_gap: true,
                            },
                        );
                    }
                    write_err = Some(e.into());
                    break;
                }
                #[cfg(not(target_arch = "wasm32"))]
                if self.shadow_staging.is_enabled() && idx < crate::torque::MAX_CHANNELS {
                    let now = self.clock_now();
                    let mut outcomes = [crate::torque::MotorWriteOutcome::NotTargeted;
                        crate::torque::MAX_CHANNELS];
                    outcomes[idx] = crate::torque::MotorWriteOutcome::Sent;
                    self.shadow_staging
                        .push(crate::torque::shadow::ShadowDriverObs::WriteResult {
                            device: 0,
                            kind: crate::torque::WriteKind::Enable,
                            write_done: now,
                            now,
                            target_mask: crate::torque::ChannelMask::bit(idx as u8),
                            outcomes,
                            ambiguous_gap: false,
                        });
                }
                if let Err(e) = self.drain_tx_echo(&pkt) {
                    write_err = Some(e);
                    break;
                }
            }

            self.strict_stop_check(Some(start_stamp))?;

            let (_, one_count, _, _, first_error) =
                self.read_torque_enable_once_budgeted(&ids, Some(start_stamp), None, None)?;
            let all_enabled = write_err.is_none() && one_count == ids.len();

            if all_enabled && one_count == ids.len() {
                self.last_enable_poll = Some(self.clock_now());
                Ok(())
            } else {
                // No local rollback on Feetech enable failure: return the failure so the
                // supervisor enters FailedHold and holds every joint.
                if let Some(e) = write_err {
                    Err(ArmDriverError::HardwareFault(e.to_string()))
                } else if let Some((_, error_code)) = first_error {
                    Err(ArmDriverError::HardwareFault(error_code.to_string()))
                } else {
                    Err(ArmDriverError::HardwareFault("FEETECH_ENABLE_UNCONFIRMED".to_string()))
                }
            }
        } else {
            let started_at = self.clock_now();
            self.disable_torque(DisableKind::DisableRequested { started_at })
        }
    }

    fn disable_torque(&mut self, kind: DisableKind) -> Result<(), ArmDriverError> {
        let prev_managed = self.control_release_managed;
        self.control_release_managed = true;
        let mut last_release_at = self.clock_now();
        let res = (|| {
            let mut progress = self.begin_disable_torque(kind)?;
            let mut first = true;
            loop {
                if let TorqueProgress::Done(_) = progress {
                    return Ok(());
                }
                if !first {
                    self.clock_sleep(Duration::from_millis(2));
                }
                first = false;
                let now = self.clock_now();
                if now.saturating_duration_since(last_release_at) >= EVIDENCE_PROBE_INTERVAL {
                    last_release_at = now;
                    self.control_release_id = self.control_release_id.wrapping_add(1).max(1);
                }
                let deadline = self.clock_now() + Duration::from_millis(8);
                progress = self.poll_torque(deadline)?;
            }
        })();
        self.control_release_managed = prev_managed;
        res
    }

    fn begin_disable_torque(&mut self, kind: DisableKind) -> Result<TorqueProgress, ArmDriverError> {
        self.begin_disable_torque_budgeted(kind, u16::MAX)
    }

    fn begin_disable_torque_budgeted(
        &mut self,
        kind: DisableKind,
        max_rx_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.assert_clock_matches_transport();
        self.pending_disable = None;
        if let DisableKind::DisableRequested { started_at } = kind {
            let exp = started_at + STOPPED_DISABLE_REQUEST_LIFETIME;
            if self.clock_now() >= exp {
                return Err(ArmDriverError::DisableRequestExpired);
            }
            match self.evaluate_rest_pose_gate() {
                RestPoseGateStatus::Confirmed => {}
                // M3: Defer both incomplete drains/windows and stale/missing evidence into the
                // 1 s probe loop; refuse immediately only on a real non-rest pose or 1 s expiry.
                RestPoseGateStatus::PendingDrainOrWindow
                | RestPoseGateStatus::StaleOrMissing => {
                    self.pending_disable = Some(PendingFeetechDisable { kind, client_park: None });
                    self.last_poll_rx = Some((0, false));
                    let wake = (self.clock_now() + EVIDENCE_PROBE_INTERVAL).min(exp);
                    return Ok(TorqueProgress::Pending { wake });
                }
                RestPoseGateStatus::RefusedNonRest => {
                    return Err(ArmDriverError::TorqueOffWouldDrop);
                }
            }
        }
        let (done, rx_used, exhausted) = self.execute_disable_writes_budgeted(
            kind,
            None,
            self.tx_release_deadline,
            Some(max_rx_frames),
        )?;
        self.last_poll_rx = Some((rx_used, exhausted));
        if done {
            Ok(TorqueProgress::Done(TorqueOutcome::Confirmed))
        } else {
            let wake = match kind {
                DisableKind::DisableRequested { started_at } => {
                    let exp = started_at + STOPPED_DISABLE_REQUEST_LIFETIME;
                    (self.clock_now() + EVIDENCE_PROBE_INTERVAL).min(exp)
                }
                DisableKind::DisableProtective { .. } => self.clock_now(),
            };
            Ok(TorqueProgress::Pending { wake })
        }
    }

    fn write_estop_hold(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        let n = self.servo_ids.len().min(64);
        let hold_mask = if n >= 64 { u64::MAX } else { (1u64 << n) - 1 };
        self.write_measured_hold_masked(hold_mask, targets).map(|_| ())
    }

    fn write_estop_hold_bounded(&mut self, targets: &[JointTarget]) -> Result<TxBatchOutcome, ArmDriverError> {
        let n = self.servo_ids.len().min(64);
        let hold_mask = if n >= 64 { u64::MAX } else { (1u64 << n) - 1 };
        self.write_measured_hold_masked(hold_mask, targets).map(|r| r.outcome)
    }

    fn write_measured_hold_masked(
        &mut self,
        hold_mask: u64,
        targets: &[JointTarget],
    ) -> Result<HoldPassReport, ArmDriverError> {
        self.assert_clock_matches_transport();
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
        // R1: Reuse preallocated buffers for goal positions and SYNC_WRITE packet (0 heap allocs).
        self.hold_positions_buf.clear();
        let mut held_mask = 0u64;
        let mut stale_mask = 0u64;
        let default_calib = feetech::FeetechJointCalib::default();
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
            let calib = self.config.joints.get(i).unwrap_or(&default_calib);
            let range = self.effective_ranges.get(i).copied().unwrap_or_else(|| calib.effective_range_steps());
            match calib.checked_rad_to_steps_in_range(pos, range) {
                Ok(raw_steps) => {
                    self.hold_positions_buf.push((self.servo_ids[i], raw_steps));
                    report.joint_status[i] = if is_stale {
                        stale_mask |= 1u64 << i;
                        JointHoldStatus::HeldStaleTarget
                    } else {
                        JointHoldStatus::Queued
                    };
                    held_mask |= 1u64 << i;
                }
                Err(_) => {
                    report.joint_status[i] = JointHoldStatus::UnheldNoFreshFeedback;
                }
            }
        }
        if !self.hold_positions_buf.is_empty() {
            let mut sync_pkt = std::mem::take(&mut self.hold_sync_pkt_buf);
            let build_res = feetech::write_sync_write_positions_into(
                &self.hold_positions_buf,
                &mut sync_pkt,
            )
            .map_err(|e| ArmDriverError::Protocol(e.to_string()));
            let tx_res = build_res.and_then(|()| {
                self.pending_stop_check(Some(start_stamp))?;
                self.transport.write_all(&sync_pkt)?;
                self.pending_stop_check(Some(start_stamp))?;
                self.transport.flush()?;
                self.drain_tx_echo(&sync_pkt)?;
                Ok(())
            });
            self.hold_sync_pkt_buf = sync_pkt;
            tx_res?;
            #[cfg(not(target_arch = "wasm32"))]
            if self.shadow_staging.is_enabled() {
                let now = self.clock_now();
                let mut mask = crate::torque::ChannelMask::EMPTY;
                let mut outcomes = [crate::torque::MotorWriteOutcome::NotTargeted;
                    crate::torque::MAX_CHANNELS];
                for i in 0..dof.min(crate::torque::MAX_CHANNELS) {
                    if (held_mask & (1u64 << i)) != 0 {
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
            if self.incomplete_chunk_idx.is_some() && !self.rx_buffer.is_empty() {
                let fallback_ts = self.clock_now();
                self.rx_timestamps.resize(self.rx_buffer.len(), fallback_ts);
                self.partial_pos_rx.append(&mut self.rx_buffer);
                self.partial_pos_rx_timestamps.append(&mut self.rx_timestamps);
            }
            for i in 0..dof {
                if (held_mask & (1u64 << i)) != 0 {
                    report.joint_status[i] = if (stale_mask & (1u64 << i)) != 0 {
                        JointHoldStatus::HeldStaleTarget
                    } else {
                        JointHoldStatus::Held
                    };
                }
            }
            report.sent_any = true;
        }
        Ok(report)
    }

    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.assert_clock_matches_transport();
        if targets.len() != self.servo_ids.len() {
            return Err(ArmDriverError::JointCountMismatch {
                expected: self.servo_ids.len(),
                actual: targets.len(),
            });
        }

        let mut positions = Vec::with_capacity(targets.len());
        let default_calib = feetech::FeetechJointCalib::default();
        for (i, target) in targets.iter().enumerate() {
            let calib = self.config.joints.get(i).unwrap_or(&default_calib);
            let range = self.effective_ranges.get(i).copied().unwrap_or_else(|| calib.effective_range_steps());
            let raw_steps = calib.checked_rad_to_steps_in_range(target.position, range)
                .map_err(|e| ArmDriverError::Protocol(e.to_string()))?;
            positions.push((self.servo_ids[i], raw_steps));
        }

        let sync_pkt = feetech::build_sync_write_positions(&positions)
            .map_err(|e| ArmDriverError::Protocol(e.to_string()))?;

        for w in &mut self.rest_pose_window {
            w.reset();
        }
        self.transport.write_all(&sync_pkt)?;
        self.transport.flush()?;
        self.drain_tx_echo(&sync_pkt)?;
        let write_done = self.clock_now();
        self.last_motion_write_at.fill(Some(write_done));
        #[cfg(not(target_arch = "wasm32"))]
        if self.shadow_staging.is_enabled() {
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
                        write_done,
                        now: write_done,
                        target_mask: mask,
                        outcomes,
                        ambiguous_gap: false,
                    });
            }
        }
        if self.incomplete_chunk_idx.is_some() && !self.rx_buffer.is_empty() {
            let fallback_ts = self.clock_now();
            self.rx_timestamps.resize(self.rx_buffer.len(), fallback_ts);
            self.partial_pos_rx.append(&mut self.rx_buffer);
            self.partial_pos_rx_timestamps.append(&mut self.rx_timestamps);
        }
        Ok(())
    }

    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        self.read_joint_states_inner(true)
    }
    #[cfg(not(target_arch = "wasm32"))]
    fn shadow_ext(&self) -> Option<&dyn ShadowExt> { Some(self) }
    #[cfg(not(target_arch = "wasm32"))]
    fn shadow_ext_mut(&mut self) -> Option<&mut dyn ShadowExt> { Some(self) }
}

#[cfg(not(target_arch = "wasm32"))]
impl<T: Transport> ShadowExt for FeetechArmDriver<T> {
    fn shadow_vendor_kind(&self) -> crate::torque::VendorKind {
        crate::torque::VendorKind::Feetech
    }

    fn set_shadow_enabled(&mut self, enabled: bool) {
        self.shadow_staging.set_enabled(enabled);
    }

    fn drain_shadow_observations(
        &mut self,
        out: &mut [Option<crate::torque::shadow::ShadowDriverObs>],
    ) -> (usize, u64) {
        if self.pending_disable_shadow.is_some() {
            let now = self.clock_now();
            PendingDisableShadowBatch::flush_into(
                &mut self.pending_disable_shadow,
                &mut self.shadow_staging,
                now,
            );
        }
        self.shadow_staging.drain_into(out)
    }
}
