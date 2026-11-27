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
