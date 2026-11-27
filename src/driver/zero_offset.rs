//! [`ZeroOffsetDriver`]: the joint zero boundary (`crate::joint_zero`) as an [`ArmDriver`]
//! wrapper. Read: joint = motor - offset; command: motor = joint + offset; velocities, torques
//! and gains unchanged; offsets 0 pass every value through unchanged.
//!
//! It implements the core trait and the extensions that carry joint-space values
//! (`JointModeExt`, `PosForceExt`, `CommissioningExt`, `ShadowExt`) plus `JointZeroExt`; the
//! other four extensions are handed through by their accessors (no forwards to miss). Every
//! provided method must be implemented here: a forgotten forward would silently fall back to
//! the trait default instead of the inner driver.
#![deny(clippy::missing_trait_methods)]

use super::*;

/// The driver boundary: applies the joint zero offsets to everything that carries a position.
pub struct ZeroOffsetDriver {
    inner: Box<dyn ArmDriver>,
    offsets: Vec<f32>,
    /// Motor-side targets of the last write (reused: no allocation per write).
    motor_targets: Vec<JointTarget>,
    /// Joint-space stopped-disable gate kept so offset updates re-shift `inner`'s motor-space rest pose.
    stopped_disable_gate: Option<StoppedDisableGate>,
}

impl ZeroOffsetDriver {
    pub fn new(inner: Box<dyn ArmDriver>, offsets: &[f32]) -> Self {
        Self {
            inner,
            offsets: offsets.to_vec(),
            motor_targets: Vec::with_capacity(offsets.len().max(8)),
            stopped_disable_gate: None,
        }
    }

    fn off(&self, i: usize) -> f32 { self.offsets.get(i).copied().unwrap_or(0.0) }

    fn has_offsets(&self) -> bool { self.offsets.iter().any(|o| *o != 0.0) }

    /// Runs `f` on the inner driver with `targets` shifted to motor space (`motor = joint +
    /// offset`), using the reused `motor_targets` buffer: no allocation once it has grown to the
    /// DOF. Offsets all 0 pass `targets` through unchanged.
    fn with_motor_targets<R>(
        &mut self,
        targets: &[JointTarget],
        f: impl FnOnce(&mut dyn ArmDriver, &[JointTarget]) -> R,
    ) -> R {
        if !self.has_offsets() {
            return f(&mut *self.inner, targets);
        }
        let mut m = std::mem::take(&mut self.motor_targets);
        m.clear();
        m.extend(targets.iter().enumerate().map(|(i, t)| {
            let o = self.off(i);
            let mut t = t.clone(); // plain fields: no allocation
            if o != 0.0 { t.position += o; }
            t
        }));
        let r = f(&mut *self.inner, &m);
        self.motor_targets = m;
        r
    }
}

impl ArmDriver for ZeroOffsetDriver {
    fn capabilities(&self) -> DriverCapabilities { self.inner.capabilities() }
    fn enable_state(&self) -> EnableState { self.inner.enable_state() }
    fn is_hardware_disarmed_confirmed(&self) -> bool {
        self.inner.is_hardware_disarmed_confirmed()
    }
    fn probe_hardware_disarmed_until(&mut self, deadline: Instant) -> bool {
        self.inner.probe_hardware_disarmed_until(deadline)
    }
    fn reported_torque_loss_sample_seq(&self) -> Option<u64> {
        self.inner.reported_torque_loss_sample_seq()
    }
    fn has_driver_fresh_position_api(&self) -> bool {
        self.inner.has_driver_fresh_position_api()
    }
    fn fresh_measured_position_for(&self, joint: usize, max_age: Duration) -> Option<f32> {
        let mut pos = self.inner.fresh_measured_position_for(joint, max_age)?;
        let o = self.off(joint);
        if o != 0.0 {
            pos -= o;
        }
        pos.is_finite().then_some(pos)
    }
    fn fresh_measured_sample_for(&self, joint: usize, max_age: Duration) -> Option<(f32, u64)> {
        let (mut pos, epoch) = self.inner.fresh_measured_sample_for(joint, max_age)?;
        let o = self.off(joint);
        if o != 0.0 {
            pos -= o;
        }
        pos.is_finite().then_some((pos, epoch))
    }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { self.inner.feedback_ages() }
    fn feedback_instants(&self) -> Vec<Option<Instant>> { self.inner.feedback_instants() }
    fn feedback_seqs(&self) -> Vec<u64> { self.inner.feedback_seqs() }
    fn feedback_queue_bounds(&self) -> Vec<Option<Instant>> { self.inner.feedback_queue_bounds() }
    fn feedback_receive_instants(&self) -> Vec<Option<std::time::Instant>> { self.inner.feedback_receive_instants() }
    fn max_command_velocity(&self, joint: usize) -> Option<f32> { self.inner.max_command_velocity(joint) }
    fn position_limits(&self, joint: usize) -> Option<(f32, f32)> {
        let o = self.off(joint);
        self.inner.position_limits(joint).map(|(lo, hi)| if o == 0.0 { (lo, hi) } else { (lo - o, hi - o) })
    }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let mut states = self.inner.read_joint_states()?;
        for (i, s) in states.iter_mut().enumerate() {
            let o = self.off(i);
            if o != 0.0 { s.position -= o; }
        }
        Ok(states)
    }
    fn read_joint_states_rx_only(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let mut states = self.inner.read_joint_states_rx_only()?;
        for (i, s) in states.iter_mut().enumerate() {
            let o = self.off(i);
            if o != 0.0 { s.position -= o; }
        }
        Ok(states)
    }
    fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
        self.inner.read_joint_states_into(out)?;
        for (i, s) in out.iter_mut().enumerate() {
            let o = self.off(i);
            if o != 0.0 { s.position -= o; }
        }
        Ok(())
    }
    fn read_joint_states_budgeted(&mut self, out: &mut Vec<JointState>, deadline: Instant, max_rx_frames: u16) -> Result<(u16, bool), ArmDriverError> {
        let rx = self.inner.read_joint_states_budgeted(out, deadline, max_rx_frames)?;
        for (i, s) in out.iter_mut().enumerate() {
            let o = self.off(i);
            if o != 0.0 { s.position -= o; }
        }
        Ok(rx)
    }
    fn read_joint_states_safe_hold(&mut self, out: &mut Vec<JointState>, deadline: Instant, max_rx_frames: u16) -> Result<(u16, bool), ArmDriverError> {
        let rx = self.inner.read_joint_states_safe_hold(out, deadline, max_rx_frames)?;
        for (i, s) in out.iter_mut().enumerate() {
            let o = self.off(i);
            if o != 0.0 { s.position -= o; }
        }
        Ok(rx)
    }
    fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
        self.inner.read_cached_joint_states_into(out)?;
        for (i, s) in out.iter_mut().enumerate() {
            let o = self.off(i);
            if o != 0.0 { s.position -= o; }
        }
        Some(())
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.with_motor_targets(targets, |inner, m| inner.write_joint_targets(m))
    }
    fn write_joint_targets_bounded(&mut self, targets: &[JointTarget]) -> Result<TxBatchOutcome, ArmDriverError> {
        self.with_motor_targets(targets, |inner, m| inner.write_joint_targets_bounded(m))
    }
    fn defer_motion_targets_past_cutoff(&mut self, targets: &[JointTarget]) {
        self.with_motor_targets(targets, |inner, m| inner.defer_motion_targets_past_cutoff(m))
    }
    fn set_tx_release_deadline(&mut self, deadline: Option<Instant>) { self.inner.set_tx_release_deadline(deadline); }
    fn write_estop_hold(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.with_motor_targets(targets, |inner, m| inner.write_estop_hold(m))
    }
    fn write_estop_hold_bounded(&mut self, targets: &[JointTarget]) -> Result<TxBatchOutcome, ArmDriverError> {
        self.with_motor_targets(targets, |inner, m| inner.write_estop_hold_bounded(m))
    }
    fn write_measured_hold_masked(
        &mut self,
        hold_mask: u64,
        targets: &[JointTarget],
    ) -> Result<HoldPassReport, ArmDriverError> {
        self.with_motor_targets(targets, |inner, m| inner.write_measured_hold_masked(hold_mask, m))
    }
    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) { self.inner.feedback_ages_into(out) }
    fn rest_pose_gate_status(&self) -> RestPoseGateStatus {
        self.inner.rest_pose_gate_status()
    }
    fn supports_budgeted_disable_await(&self) -> bool {
        self.inner.supports_budgeted_disable_await()
    }
    fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool { self.inner.feedback_all_within(dof, max_age) }
    fn feedback_joints_within(&self, joints: &[usize], max_age: Duration) -> bool { self.inner.feedback_joints_within(joints, max_age) }
    fn fresh_joint_positions_into(&self, joints: &[usize], max_age: Duration, out: &mut [f32]) -> bool {
        if !self.inner.fresh_joint_positions_into(joints, max_age, out) {
            return false;
        }
        for (slot, &i) in out.iter_mut().zip(joints.iter()) {
            let o = self.off(i);
            if o != 0.0 {
                *slot -= o;
            }
            if !slot.is_finite() {
                return false;
            }
        }
        true
    }
    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) { self.inner.feedback_age_ms_into(out) }
    fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) { self.inner.feedback_instants_into(out) }
    fn feedback_seqs_into(&self, out: &mut Vec<u64>) { self.inner.feedback_seqs_into(out) }
    fn feedback_queue_bounds_into(&self, out: &mut Vec<Option<Instant>>) { self.inner.feedback_queue_bounds_into(out) }
    fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) { self.inner.feedback_receive_instants_into(out) }
    fn joint_loads_into(&self, out: &mut Vec<f32>) -> bool { self.inner.joint_loads_into(out) }
    fn motor_status_into(&self, out: &mut Vec<u8>) -> bool { self.inner.motor_status_into(out) }
    fn clear_fault_disables_arm_joint(&self, arm_dof: usize) -> bool { self.inner.clear_fault_disables_arm_joint(arm_dof) }
    fn has_hardware_fault(&self) -> bool { self.inner.has_hardware_fault() }
    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> { self.inner.set_torque(enabled) }
    fn disable_torque(&mut self, kind: DisableKind) -> Result<(), ArmDriverError> { self.inner.disable_torque(kind) }
    fn begin_torque(&mut self, enabled: bool) -> Result<TorqueProgress, ArmDriverError> { self.inner.begin_torque(enabled) }
    fn begin_disable_torque(&mut self, kind: DisableKind) -> Result<TorqueProgress, ArmDriverError> {
        self.inner.begin_disable_torque(kind)
    }
    fn begin_torque_budgeted(&mut self, enabled: bool, max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
        self.inner.begin_torque_budgeted(enabled, max_rx_frames)
    }
    fn begin_torque_budgeted_with_range_retry(&mut self, enabled: bool, max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
        self.inner.begin_torque_budgeted_with_range_retry(enabled, max_rx_frames)
    }
    fn begin_disable_torque_budgeted(&mut self, kind: DisableKind, max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
        self.inner.begin_disable_torque_budgeted(kind, max_rx_frames)
    }
    fn begin_client_park_disable(
        &mut self,
        max_rx_frames: Option<u16>,
        expected_stop_gen: Option<u64>,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.inner.begin_client_park_disable(max_rx_frames, expected_stop_gen)
    }
    fn begin_client_park_disable_with_kind(
        &mut self,
        kind: DisableKind,
        max_rx_frames: Option<u16>,
        expected_stop_gen: Option<u64>,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.inner
            .begin_client_park_disable_with_kind(kind, max_rx_frames, expected_stop_gen)
    }
    fn poll_torque(&mut self, deadline: Instant) -> Result<TorqueProgress, ArmDriverError> { self.inner.poll_torque(deadline) }
    fn poll_torque_budgeted(&mut self, deadline: Instant, max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
        self.inner.poll_torque_budgeted(deadline, max_rx_frames)
    }
    fn take_last_poll_rx(&mut self) -> Option<(u16, bool)> { self.inner.take_last_poll_rx() }
    fn torque_transition_pending(&self) -> bool { self.inner.torque_transition_pending() }
    fn disable_frames_dispatched(&self) -> bool { self.inner.disable_frames_dispatched() }
    fn disable_dispatched_mask(&self) -> u64 { self.inner.disable_dispatched_mask() }
    fn abort_enable(&mut self) { self.inner.abort_enable() }
    fn last_committed_wire_position(&self, joint: usize) -> Option<(u64, Instant, f32)> {
        let (seq, tx_at, wire_q) = self.inner.last_committed_wire_position(joint)?;
        let o = self.off(joint);
        let user_q = if o != 0.0 { wire_q - o } else { wire_q };
        user_q.is_finite().then_some((seq, tx_at, user_q))
    }
    fn prepare_safe_hold(&mut self) { self.inner.prepare_safe_hold() }
    fn enable_frames_dispatched(&self) -> bool { self.inner.enable_frames_dispatched() }
    fn abort_pending_disable(&mut self) -> bool { self.inner.abort_pending_disable() }
    fn unconfirmed_disable_joints(&self) -> Vec<usize> { self.inner.unconfirmed_disable_joints() }
    fn supports_rt_executor(&self) -> Result<(), &'static str> { self.inner.supports_rt_executor() }
    fn set_rt_mode(&mut self, rt: bool) { self.inner.set_rt_mode(rt) }
    fn clock_pause(&self, d: Duration) { self.inner.clock_pause(d) }
    fn is_feedback_valid(&self) -> bool { self.inner.is_feedback_valid() }
    fn check_hardware_status(&self) -> Result<(), ArmDriverError> { self.inner.check_hardware_status() }
    fn clear_fault(&mut self) -> Result<(), ArmDriverError> { self.inner.clear_fault() }
    fn clear_fault_budgeted(&mut self, deadline: Instant) -> Result<(), ArmDriverError> { self.inner.clear_fault_budgeted(deadline) }
    fn clear_fault_disables_joints(&self) -> Vec<usize> { self.inner.clear_fault_disables_joints() }
    fn has_known_partial_enable(&self) -> bool { self.inner.has_known_partial_enable() }
    fn has_any_joint_enabled(&self) -> bool { self.inner.has_any_joint_enabled() }
    fn refresh_enable_state(&mut self) -> Result<EnableState, ArmDriverError> { self.inner.refresh_enable_state() }
    fn set_abort_flag(&mut self, flag: std::sync::Arc<std::sync::atomic::AtomicBool>) { self.inner.set_abort_flag(flag) }
    fn set_estop_state(&mut self, state: std::sync::Arc<crate::rt_core::urgent::EstopState>) { self.inner.set_estop_state(state) }
    fn set_estop_gate(&mut self, gate: crate::rt_core::urgent::EstopGate) { self.inner.set_estop_gate(gate) }
    fn set_stopped_disable_gate(&mut self, gate: StoppedDisableGate) {
        let motor_gate = gate.with_motor_offsets(&self.offsets);
        self.stopped_disable_gate = Some(gate);
        self.inner.set_stopped_disable_gate(motor_gate);
    }
    fn set_gripper_index(&mut self, index: Option<usize>) { self.inner.set_gripper_index(index) }
    fn is_enabled(&self) -> bool { self.inner.is_enabled() }
    fn motor_status(&self) -> Option<Vec<u8>> { self.inner.motor_status() }
    fn joint_loads(&self) -> Option<Vec<f32>> { self.inner.joint_loads() }
    fn decode_motor_error(&self, code: u8) -> Option<String> { self.inner.decode_motor_error(code) }
    fn decode_motor_error_static(&self, code: u8) -> Option<&'static str> {
        self.inner.decode_motor_error_static(code)
    }
    fn set_torque_flag(&mut self, enabled: bool) { self.inner.set_torque_flag(enabled) }
    fn motor_zero_support(&self) -> MotorZeroSupport { self.inner.motor_zero_support() }
    fn clock_now(&self) -> std::time::Instant { self.inner.clock_now() }
    fn begin_control_release(&mut self, release_seq: u64) { self.inner.begin_control_release(release_seq) }
    fn service_release_watchdog_fallback(
        &mut self,
        authorized_mask: u64,
        hold_targets: Option<&[Option<f32>]>,
    ) {
        if let Some(targets) = hold_targets {
            let mut motor_targets = [None; crate::rt_core::mailbox::MAX_DOF];
            let n = targets.len().min(motor_targets.len());
            for i in 0..n {
                motor_targets[i] = targets[i].map(|p| p + self.offsets.get(i).copied().unwrap_or(0.0));
            }
            self.inner
                .service_release_watchdog_fallback(authorized_mask, Some(&motor_targets[..n]));
        } else {
            self.inner.service_release_watchdog_fallback(authorized_mask, None);
        }
    }
    fn driver_clock(&self) -> Option<std::sync::Arc<dyn crate::transport::DriverClock>> {
        self.inner.driver_clock()
    }
    // Extensions without joint-space values pass through: the inner driver's own object.
    fn tx_ext(&self) -> Option<&dyn BoundedTxExt> { self.inner.tx_ext() }
    fn tx_ext_mut(&mut self) -> Option<&mut dyn BoundedTxExt> { self.inner.tx_ext_mut() }
    fn mode_switch_ext(&self) -> Option<&dyn ModeSwitchExt> { self.inner.mode_switch_ext() }
    fn mode_switch_ext_mut(&mut self) -> Option<&mut dyn ModeSwitchExt> { self.inner.mode_switch_ext_mut() }
    fn mit_ext(&self) -> Option<&dyn MitExt> { self.inner.mit_ext() }
    fn mit_ext_mut(&mut self) -> Option<&mut dyn MitExt> { self.inner.mit_ext_mut() }
    fn bus_recovery_ext(&self) -> Option<&dyn BusRecoveryExt> { self.inner.bus_recovery_ext() }
    fn bus_recovery_ext_mut(&mut self) -> Option<&mut dyn BusRecoveryExt> { self.inner.bus_recovery_ext_mut() }
    // Extensions with joint-space values are wrapped (offsets applied) when the inner driver has them.
    fn commissioning_ext(&self) -> Option<&dyn CommissioningExt> { self.inner.commissioning_ext().map(|_| self as _) }
    fn commissioning_ext_mut(&mut self) -> Option<&mut dyn CommissioningExt> {
        if self.inner.commissioning_ext_mut().is_some() { Some(self) } else { None }
    }
    fn joint_mode_ext(&self) -> Option<&dyn JointModeExt> { self.inner.joint_mode_ext().map(|_| self as _) }
    fn joint_mode_ext_mut(&mut self) -> Option<&mut dyn JointModeExt> {
        if self.inner.joint_mode_ext_mut().is_some() { Some(self) } else { None }
    }
    fn pos_force_ext(&self) -> Option<&dyn PosForceExt> { self.inner.pos_force_ext().map(|_| self as _) }
    fn pos_force_ext_mut(&mut self) -> Option<&mut dyn PosForceExt> {
        if self.inner.pos_force_ext_mut().is_some() { Some(self) } else { None }
    }
    #[cfg(not(target_arch = "wasm32"))]
    fn shadow_ext(&self) -> Option<&dyn ShadowExt> { self.inner.shadow_ext().map(|_| self as _) }
    #[cfg(not(target_arch = "wasm32"))]
    fn shadow_ext_mut(&mut self) -> Option<&mut dyn ShadowExt> {
        if self.inner.shadow_ext_mut().is_some() { Some(self) } else { None }
    }
    // The offsets themselves: only this wrapper has them.
    fn joint_zero_ext(&self) -> Option<&dyn JointZeroExt> { Some(self) }
    fn joint_zero_ext_mut(&mut self) -> Option<&mut dyn JointZeroExt> { Some(self) }
}

impl CommissioningExt for ZeroOffsetDriver {
    fn set_zero_position(&mut self, joint: usize) -> Result<(), ArmDriverError> { self.inner.commissioning_mut().set_zero_position(joint) }
    fn read_motor_identity(&mut self, joints: &[usize]) -> Result<Vec<crate::mit::MotorIdentity>, ArmDriverError> { self.inner.commissioning_mut().read_motor_identity(joints) }
    fn commission_motor_timeout(&mut self, joints: &[usize], counts: u32) -> Result<Vec<crate::mit::TimeoutCommissioning>, crate::mit::TimeoutBatchError> {
        self.inner.commissioning_mut().commission_motor_timeout(joints, counts)
    }
    fn supports_nonblocking_commissioning(&self) -> bool {
        self.inner.commissioning().supports_nonblocking_commissioning()
    }
    fn supports_nonblocking_motor_zero(&self) -> bool {
        self.inner.commissioning().supports_nonblocking_motor_zero()
    }
    fn commissioning_motor_id(&self, joint: usize) -> Option<u16> {
        self.inner.commissioning().commissioning_motor_id(joint)
    }
    fn commissioning_drain_rx_step(&mut self, max_frames: u16, deadline: Instant) -> Result<(u16, bool), ArmDriverError> {
        self.inner.commissioning_mut().commissioning_drain_rx_step(max_frames, deadline)
    }
    fn commissioning_clear_register_log(&mut self) {
        self.inner.commissioning_mut().commissioning_clear_register_log();
    }
    fn commissioning_try_send_register_read(
        &mut self,
        joint: usize,
        rid: u8,
        deadline: Instant,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::mit::CommissioningTxStatus {
        self.inner.commissioning_mut().commissioning_try_send_register_read(joint, rid, deadline, pre_tx_gate)
    }
    fn commissioning_try_send_rest_probe(
        &mut self,
        joint: usize,
        deadline: Instant,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::mit::CommissioningTxStatus {
        self.inner.commissioning_mut().commissioning_try_send_rest_probe(joint, deadline, pre_tx_gate)
    }
    fn commissioning_try_send_timeout_write(
        &mut self,
        joint: usize,
        counts: u32,
        deadline: Instant,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::mit::CommissioningTxStatus {
        self.inner.commissioning_mut().commissioning_try_send_timeout_write(joint, counts, deadline, pre_tx_gate)
    }
    fn commissioning_find_register_reply(
        &self,
        joint: usize,
        write: bool,
        rid: u8,
        after_tx_seq: u64,
        min_rx_at: Instant,
        max_rx_at: Option<Instant>,
        tx_offset_ns: Option<i128>,
    ) -> crate::mit::CommissioningRegMatch {
        self.inner.commissioning().commissioning_find_register_reply(
            joint,
            write,
            rid,
            after_tx_seq,
            min_rx_at,
            max_rx_at,
            tx_offset_ns,
        )
    }
    fn commissioning_disable_step(
        &mut self,
        started_at: Instant,
        deadline: Instant,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::mit::CommissioningDisableStep {
        self.inner.commissioning_mut().commissioning_disable_step(started_at, deadline, pre_tx_gate)
    }
    fn commissioning_try_send_set_zero(
        &mut self,
        joint: usize,
        deadline: Instant,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::mit::CommissioningTxStatus {
        self.inner.commissioning_mut().commissioning_try_send_set_zero(joint, deadline, pre_tx_gate)
    }
    fn commissioning_raw_feedback(&self, joint: usize) -> Option<crate::mit::CommissioningFeedback> {
        let mut fb = self.inner.commissioning().commissioning_raw_feedback(joint)?;
        let o = self.off(joint);
        if o != 0.0 {
            fb.joint_position = fb.raw_position - o;
        }
        Some(fb)
    }
}

impl JointModeExt for ZeroOffsetDriver {
    fn joint_hold_mode_verified(&self, joint: usize) -> bool {
        self.inner.joint_mode().joint_hold_mode_verified(joint)
    }
    fn request_mode_readback_budgeted(&mut self, unverified_mask: u64) -> bool {
        self.inner.joint_mode_mut().request_mode_readback_budgeted(unverified_mask)
    }
    fn with_range_check(&self, f: &mut dyn FnMut(Option<&DriverRangeCheck>)) { self.inner.joint_mode().with_range_check(f) }
    fn take_recovery_writes_into(&mut self, out: &mut Vec<usize>) { self.inner.joint_mode_mut().take_recovery_writes_into(out) }
    fn supports_mit_mode_switch(&self) -> bool { self.inner.joint_mode().supports_mit_mode_switch() }
    fn set_joint_modes(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.with_motor_targets(hold, |inner, m| inner.joint_mode_mut().set_joint_modes(modes, m))
    }
    fn set_joint_modes_in_tick(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.with_motor_targets(hold, |inner, m| inner.joint_mode_mut().set_joint_modes_in_tick(modes, m))
    }
    fn joint_modes_uncertain(&self) -> bool { self.inner.joint_mode().joint_modes_uncertain() }
    fn set_strict_mode_handover(&mut self, strict: bool) { self.inner.joint_mode_mut().set_strict_mode_handover(strict) }
    fn set_joint_modes_confirmed(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.with_motor_targets(hold, |inner, m| inner.joint_mode_mut().set_joint_modes_confirmed(modes, m))
    }
    fn set_mode_switch_hold_targets(&mut self, targets: &[Option<ModeSwitchHoldTarget>]) {
        let mut shifted = [None; crate::motor_mode::MAX_MODE_SWITCH_AXES];
        let n = targets.len().min(shifted.len());
        for i in 0..n {
            if let Some(mut t) = targets[i] {
                t.position += self.off(i);
                shifted[i] = Some(t);
            }
        }
        self.inner.joint_mode_mut().set_mode_switch_hold_targets(&shifted[..n]);
    }
}

impl PosForceExt for ZeroOffsetDriver {
    fn reset_pos_force_boot_state(&mut self) { self.inner.pos_force_mut().reset_pos_force_boot_state(); }
    fn set_external_pos_force_emitter(&mut self, enabled: bool) {
        self.inner.pos_force_mut().set_external_pos_force_emitter(enabled);
    }
    fn validate_pos_force_intent_target(
        &self,
        position: f32,
        velocity: Option<f32>,
        current_raw: Option<u16>,
    ) -> Result<(), ArmDriverError> {
        let offset = self.inner.pos_force().gripper_pos_force_info().map_or(0.0, |info| self.off(info.joint));
        self.inner.pos_force().validate_pos_force_intent_target(position + offset, velocity, current_raw)
    }
    fn set_pending_pos_force_hold_hint(&mut self, is_hold: bool) {
        self.inner.pos_force_mut().set_pending_pos_force_hold_hint(is_hold);
    }
    fn emit_pos_force_j7(
        &mut self,
        intent: PosForceIntent,
        current: PosForceIntentStamp,
    ) -> Result<PosForceEmitOutcome, ArmDriverError> {
        let mapped = match intent.kind {
            PosForceIntentKind::Target { position, velocity, current_raw } => {
                let offset = self.inner.pos_force().gripper_pos_force_info().map_or(0.0, |info| self.off(info.joint));
                PosForceIntent {
                    kind: PosForceIntentKind::Target {
                        position: position + offset,
                        velocity,
                        current_raw,
                    },
                    stamp: intent.stamp,
                }
            }
            _ => intent,
        };
        self.inner.pos_force_mut().emit_pos_force_j7(mapped, current)
    }
    fn take_last_pos_force_tx_status(&mut self) -> PosForceTxStatus {
        self.inner.pos_force_mut().take_last_pos_force_tx_status()
    }
    fn pos_force_feedback_fresh(&self) -> bool { self.inner.pos_force().pos_force_feedback_fresh() }
    fn pos_force_last_fresh_position(&self) -> Option<f32> {
        let pos = self.inner.pos_force().pos_force_last_fresh_position()?;
        let offset = self.inner.pos_force().gripper_pos_force_info().map_or(0.0, |info| self.off(info.joint));
        Some(pos - offset)
    }
    fn motor_current_cap(&self) -> Option<MotorCurrentCap> { self.inner.pos_force().motor_current_cap() }
    fn gripper_pos_force_info(&self) -> Option<GripperPosForceInfo> { self.inner.pos_force().gripper_pos_force_info() }
}

#[cfg(not(target_arch = "wasm32"))]
impl ShadowExt for ZeroOffsetDriver {
    fn shadow_vendor_kind(&self) -> crate::torque::VendorKind {
        self.inner.shadow().shadow_vendor_kind()
    }
    fn set_shadow_enabled(&mut self, enabled: bool) {
        self.inner.shadow_mut().set_shadow_enabled(enabled);
    }
    fn drain_shadow_observations(
        &mut self,
        out: &mut [Option<crate::torque::shadow::ShadowDriverObs>],
    ) -> (usize, u64) {
        let (count, dropped) = self.inner.shadow_mut().drain_shadow_observations(out);
        for slot in out.iter_mut().take(count) {
            if let Some(crate::torque::shadow::ShadowDriverObs::CausalSample {
                channel,
                position_rad,
                ..
            }) = slot
            {
                let o = self.off(usize::from(*channel));
                if o != 0.0 {
                    *position_rad -= o;
                }
            }
        }
        (count, dropped)
    }
}

impl JointZeroExt for ZeroOffsetDriver {
    fn set_joint_zero_offsets(&mut self, offsets: &[f32]) {
        if self.offsets.len() == offsets.len() {
            self.offsets.copy_from_slice(offsets);
        } else {
            self.offsets = offsets.to_vec();
        }
        if let Some(ref gate) = self.stopped_disable_gate {
            self.inner.set_stopped_disable_gate(gate.with_motor_offsets(&self.offsets));
        }
    }
    fn joint_zero_offsets(&self) -> Option<&[f32]> { Some(&self.offsets) }
}
