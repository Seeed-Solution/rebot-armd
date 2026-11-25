//! Joint zero calibration: `calibration.json` `joint_zero` and the driver boundary that applies it.
//!
//! The profile's `zero_reference` is the physical pose the joint zero refers to (the folded rest
//! pose; URDF angles, gripper closed). `joint_zero` holds one offset per joint so that, in that
//! pose, the calibrated joint angles equal the reference:
//!
//! - read: joint = motor - offset (positions; velocities and torques are unchanged),
//! - command: motor = joint + offset (position targets, MIT hold targets),
//! - driver position limits: shifted into joint space (profile limits are joint space).
//!
//! [`ZeroOffsetDriver`] wraps every driver the supervisor installs and is the only place the
//! offsets are applied, so park, limits, poses, FK / IK, the cockpit and the SDK all see
//! calibrated joint angles. Offsets 0 pass every value through unchanged (no arithmetic).
//!
//! Two ways set it, both from the zero tool (WS `/api/v1/calibration/motor_zero`) with the arm
//! folded in the rest pose and torque off: the software zero (`method: "software"`, every driver,
//! no flash write: offset = motor reading - reference, averaged over [`SOFTWARE_CAPTURE`]) and the
//! motor zero (`method: "motor"`, Damiao flash: the motor reads 0 at the reference afterwards,
//! offset = -reference, 0 on the B601-DM). A software zero can be undone with WS
//! `/api/v1/calibration/joint_zero/restore {backup}`.

use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

use crate::driver::{ArmDriver, ArmDriverError, JointMode, JointTarget};
use crate::model::{DriverCapabilities, DriverRangeCheck, EnableState, JointState, MotorZeroSupport};

/// Averaging window of the software zero.
pub const SOFTWARE_CAPTURE: Duration = Duration::from_millis(500);
/// Largest |offset| accepted (rad): a joint zeroed half a turn away is not a zero calibration.
pub const MAX_OFFSET_RAD: f64 = std::f64::consts::PI;
/// A joint whose zero moves less than this (rad, 0.5 deg) keeps the dependent calibrations.
pub const STALE_THRESHOLD_RAD: f64 = 0.5 * std::f64::consts::PI / 180.0;
/// Legacy detection: |J6 - reference| within this of pi/2 (10 deg).
pub const LEGACY_J6_TOL_RAD: f64 = 10.0 * std::f64::consts::PI / 180.0;

/// `calibration.json` `joint_zero`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JointZero {
    /// One entry per joint (profile DOF).
    pub joints: Vec<JointZeroEntry>,
    /// `software` or `motor`: how the last change was made.
    pub method: String,
    /// UTC time of the last change (RFC 3339).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub set_at: Option<String>,
    /// The profile's `zero_reference.name` (`rest`).
    #[serde(default = "rest")]
    pub reference: String,
}

fn rest() -> String { "rest".into() }

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JointZeroEntry {
    /// motor angle - joint angle (rad).
    pub offset_rad: f64,
}

impl JointZero {
    pub fn new(offsets: &[f32], method: &str, reference: &str) -> Self {
        Self {
            joints: offsets.iter().map(|o| JointZeroEntry { offset_rad: f64::from(*o) }).collect(),
            method: method.into(),
            set_at: Some(chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Secs, true)),
            reference: reference.into(),
        }
    }

    pub fn offsets(&self) -> Vec<f32> { self.joints.iter().map(|j| j.offset_rad as f32).collect() }

    pub fn is_zero(&self) -> bool { self.joints.iter().all(|j| j.offset_rad == 0.0) }

    pub fn validate(&self, dof: Option<usize>) -> Result<(), String> {
        if dof.is_some_and(|n| n != self.joints.len()) {
            return Err(format!("joint_zero.joints needs {} entries", dof.unwrap_or_default()));
        }
        if !["software", "motor"].contains(&self.method.as_str()) {
            return Err("joint_zero.method must be \"software\" or \"motor\"".into());
        }
        if !self.joints.iter().all(|j| j.offset_rad.is_finite() && j.offset_rad.abs() <= MAX_OFFSET_RAD) {
            return Err(format!("joint_zero offsets must be finite and within {MAX_OFFSET_RAD:.4} rad"));
        }
        Ok(())
    }
}

/// The driver boundary: applies the joint zero offsets to everything that carries a position.
pub struct ZeroOffsetDriver {
    inner: Box<dyn ArmDriver>,
    offsets: Vec<f32>,
    /// Motor-side targets of the last write (reused: no allocation per write).
    motor_targets: Vec<JointTarget>,
}

impl ZeroOffsetDriver {
    pub fn new(inner: Box<dyn ArmDriver>, offsets: &[f32]) -> Self {
        Self { inner, offsets: offsets.to_vec(), motor_targets: Vec::with_capacity(offsets.len().max(8)) }
    }

    fn off(&self, i: usize) -> f32 { self.offsets.get(i).copied().unwrap_or(0.0) }

    fn to_motor(&self, targets: &[JointTarget]) -> Option<Vec<JointTarget>> {
        if self.offsets.iter().all(|o| *o == 0.0) { return None; }
        Some(targets.iter().enumerate().map(|(i, t)| {
            let o = self.off(i);
            let mut t = t.clone();
            if o != 0.0 { t.position += o; }
            t
        }).collect())
    }
}

impl ArmDriver for ZeroOffsetDriver {
    fn capabilities(&self) -> DriverCapabilities { self.inner.capabilities() }
    fn enable_state(&self) -> EnableState { self.inner.enable_state() }
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
    fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
        self.inner.read_cached_joint_states_into(out)?;
        for (i, s) in out.iter_mut().enumerate() {
            let o = self.off(i);
            if o != 0.0 { s.position -= o; }
        }
        Some(())
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        if self.offsets.iter().all(|o| *o == 0.0) { return self.inner.write_joint_targets(targets); }
        let mut m = std::mem::take(&mut self.motor_targets);
        m.clear();
        m.extend(targets.iter().enumerate().map(|(i, t)| {
            let o = self.off(i);
            let mut t = t.clone(); // plain fields: no allocation
            if o != 0.0 { t.position += o; }
            t
        }));
        let r = self.inner.write_joint_targets(&m);
        self.motor_targets = m;
        r
    }
    fn write_joint_targets_bounded(&mut self, targets: &[JointTarget]) -> Result<crate::driver::TxBatchOutcome, ArmDriverError> {
        if self.offsets.iter().all(|o| *o == 0.0) { return self.inner.write_joint_targets_bounded(targets); }
        let mut m = std::mem::take(&mut self.motor_targets);
        m.clear();
        m.extend(targets.iter().enumerate().map(|(i, t)| {
            let o = self.off(i);
            let mut t = t.clone(); // plain fields: no allocation
            if o != 0.0 { t.position += o; }
            t
        }));
        let r = self.inner.write_joint_targets_bounded(&m);
        self.motor_targets = m;
        r
    }
    fn discard_pending_tx(&mut self) { self.inner.discard_pending_tx(); }
    fn set_tx_release_deadline(&mut self, deadline: Option<Instant>) { self.inner.set_tx_release_deadline(deadline); }
    fn set_tx_budget(&mut self, budget: crate::driver::TxBudget) { self.inner.set_tx_budget(budget); }
    fn reset_pos_force_boot_state(&mut self) { self.inner.reset_pos_force_boot_state(); }
    fn tx_frame_max_age(&self) -> Duration { self.inner.tx_frame_max_age() }
    fn last_tx_expired_age(&self) -> Option<Duration> { self.inner.last_tx_expired_age() }
    fn oldest_pending_tx_age(&self) -> Option<Duration> { self.inner.oldest_pending_tx_age() }
    fn write_estop_hold(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        if self.offsets.iter().all(|o| *o == 0.0) { return self.inner.write_estop_hold(targets); }
        let mut m = std::mem::take(&mut self.motor_targets);
        m.clear();
        m.extend(targets.iter().enumerate().map(|(i, t)| {
            let o = self.off(i);
            let mut t = t.clone(); // plain fields: no allocation
            if o != 0.0 { t.position += o; }
            t
        }));
        let r = self.inner.write_estop_hold(&m);
        self.motor_targets = m;
        r
    }
    fn write_estop_hold_bounded(&mut self, targets: &[JointTarget]) -> Result<crate::driver::TxBatchOutcome, ArmDriverError> {
        if self.offsets.iter().all(|o| *o == 0.0) { return self.inner.write_estop_hold_bounded(targets); }
        let mut m = std::mem::take(&mut self.motor_targets);
        m.clear();
        m.extend(targets.iter().enumerate().map(|(i, t)| {
            let o = self.off(i);
            let mut t = t.clone(); // plain fields: no allocation
            if o != 0.0 { t.position += o; }
            t
        }));
        let r = self.inner.write_estop_hold_bounded(&m);
        self.motor_targets = m;
        r
    }
    fn set_external_pos_force_emitter(&mut self, enabled: bool) {
        self.inner.set_external_pos_force_emitter(enabled);
    }
    fn validate_pos_force_intent_target(
        &self,
        position: f32,
        velocity: Option<f32>,
        current_raw: Option<u16>,
    ) -> Result<(), ArmDriverError> {
        let offset = self.inner.gripper_pos_force_info().map_or(0.0, |info| self.off(info.joint));
        self.inner.validate_pos_force_intent_target(position + offset, velocity, current_raw)
    }
    fn set_pending_pos_force_hold_hint(&mut self, is_hold: bool) {
        self.inner.set_pending_pos_force_hold_hint(is_hold);
    }
    fn emit_pos_force_j7(
        &mut self,
        intent: crate::driver::PosForceIntent,
        current: crate::driver::PosForceIntentStamp,
    ) -> Result<crate::driver::PosForceEmitOutcome, ArmDriverError> {
        let mapped = match intent.kind {
            crate::driver::PosForceIntentKind::Target { position, velocity, current_raw } => {
                let offset = self.inner.gripper_pos_force_info().map_or(0.0, |info| self.off(info.joint));
                crate::driver::PosForceIntent {
                    kind: crate::driver::PosForceIntentKind::Target {
                        position: position + offset,
                        velocity,
                        current_raw,
                    },
                    stamp: intent.stamp,
                }
            }
            _ => intent,
        };
        self.inner.emit_pos_force_j7(mapped, current)
    }
    fn last_hold_tx_sent_any(&self) -> bool { self.inner.last_hold_tx_sent_any() }
    fn last_hold_tx_due_mask(&self) -> u64 { self.inner.last_hold_tx_due_mask() }
    fn last_hold_tx_sent_mask(&self) -> u64 { self.inner.last_hold_tx_sent_mask() }
    fn last_target_tx_sent_mask(&self) -> u64 { self.inner.last_target_tx_sent_mask() }
    fn take_last_pos_force_tx_status(&mut self) -> crate::driver::PosForceTxStatus {
        self.inner.take_last_pos_force_tx_status()
    }
    fn pos_force_feedback_fresh(&self) -> bool { self.inner.pos_force_feedback_fresh() }
    fn pos_force_last_fresh_position(&self) -> Option<f32> {
        let pos = self.inner.pos_force_last_fresh_position()?;
        let offset = self.inner.gripper_pos_force_info().map_or(0.0, |info| self.off(info.joint));
        Some(pos - offset)
    }
    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) { self.inner.feedback_ages_into(out) }
    fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool { self.inner.feedback_all_within(dof, max_age) }
    fn feedback_joints_within(&self, joints: &[usize], max_age: Duration) -> bool {
        self.inner.feedback_joints_within(joints, max_age)
    }
    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) { self.inner.feedback_age_ms_into(out) }
    fn with_range_check(&self, f: &mut dyn FnMut(Option<&DriverRangeCheck>)) { self.inner.with_range_check(f) }
    fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) { self.inner.feedback_instants_into(out) }
    fn feedback_seqs_into(&self, out: &mut Vec<u64>) { self.inner.feedback_seqs_into(out) }
    fn feedback_queue_bounds_into(&self, out: &mut Vec<Option<Instant>>) { self.inner.feedback_queue_bounds_into(out) }
    fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) { self.inner.feedback_receive_instants_into(out) }
    fn take_recovery_writes_into(&mut self, out: &mut Vec<usize>) { self.inner.take_recovery_writes_into(out) }
    fn joint_loads_into(&self, out: &mut Vec<f32>) -> bool { self.inner.joint_loads_into(out) }
    fn motor_status_into(&self, out: &mut Vec<u8>) -> bool { self.inner.motor_status_into(out) }
    fn clear_fault_disables_arm_joint(&self, arm_dof: usize) -> bool { self.inner.clear_fault_disables_arm_joint(arm_dof) }
    fn has_hardware_fault(&self) -> bool { self.inner.has_hardware_fault() }
    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> { self.inner.set_torque(enabled) }
    fn begin_torque(&mut self, enabled: bool) -> Result<crate::driver::TorqueProgress, ArmDriverError> { self.inner.begin_torque(enabled) }
    fn begin_torque_budgeted(&mut self, enabled: bool, max_rx_frames: u16) -> Result<crate::driver::TorqueProgress, ArmDriverError> {
        self.inner.begin_torque_budgeted(enabled, max_rx_frames)
    }
    fn poll_torque(&mut self, deadline: Instant) -> Result<crate::driver::TorqueProgress, ArmDriverError> { self.inner.poll_torque(deadline) }
    fn poll_torque_budgeted(&mut self, deadline: Instant, max_rx_frames: u16) -> Result<crate::driver::TorqueProgress, ArmDriverError> {
        self.inner.poll_torque_budgeted(deadline, max_rx_frames)
    }
    fn take_last_poll_rx(&mut self) -> Option<(u16, bool)> { self.inner.take_last_poll_rx() }
    fn torque_transition_pending(&self) -> bool { self.inner.torque_transition_pending() }
    fn abort_enable(&mut self) { self.inner.abort_enable() }
    fn panic_disable_step(&mut self, first: bool, deadline: Instant) -> Option<Result<Vec<usize>, ArmDriverError>> {
        self.inner.panic_disable_step(first, deadline)
    }
    fn unconfirmed_disable_joints(&self) -> Vec<usize> { self.inner.unconfirmed_disable_joints() }
    fn supports_rt_executor(&self) -> Result<(), &'static str> { self.inner.supports_rt_executor() }
    fn set_rt_mode(&mut self, rt: bool) { self.inner.set_rt_mode(rt) }
    fn motor_current_cap(&self) -> Option<crate::driver::MotorCurrentCap> { self.inner.motor_current_cap() }
    fn gripper_pos_force_info(&self) -> Option<crate::driver::GripperPosForceInfo> { self.inner.gripper_pos_force_info() }
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
    fn set_output_lease_gen(&mut self, lease_gen: u64) { self.inner.set_output_lease_gen(lease_gen) }
    fn set_gripper_index(&mut self, index: Option<usize>) { self.inner.set_gripper_index(index) }
    fn is_enabled(&self) -> bool { self.inner.is_enabled() }
    fn motor_status(&self) -> Option<Vec<u8>> { self.inner.motor_status() }
    fn joint_loads(&self) -> Option<Vec<f32>> { self.inner.joint_loads() }
    fn decode_motor_error(&self, code: u8) -> Option<String> { self.inner.decode_motor_error(code) }
    fn set_torque_flag(&mut self, enabled: bool) { self.inner.set_torque_flag(enabled) }
    fn set_pending_rearm(&mut self, pending: bool) { self.inner.set_pending_rearm(pending) }
    fn supports_mit_mode_switch(&self) -> bool { self.inner.supports_mit_mode_switch() }
    fn set_joint_modes(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        match self.to_motor(hold) {
            None => self.inner.set_joint_modes(modes, hold),
            Some(m) => self.inner.set_joint_modes(modes, &m),
        }
    }
    fn set_joint_modes_in_tick(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        match self.to_motor(hold) {
            None => self.inner.set_joint_modes_in_tick(modes, hold),
            Some(m) => self.inner.set_joint_modes_in_tick(modes, &m),
        }
    }
    fn pending_motion_tx_len(&self) -> usize { self.inner.pending_motion_tx_len() }
    fn pending_hold_tx_len(&self) -> usize { self.inner.pending_hold_tx_len() }
    fn tx_probe_confirmed(&self) -> bool { self.inner.tx_probe_confirmed() }
    fn tx_probe_pending(&self) -> bool { self.inner.tx_probe_pending() }
    fn probe_tx_ready(&mut self) -> Result<bool, ArmDriverError> { self.inner.probe_tx_ready() }
    fn probe_tx_ready_budgeted(&mut self, deadline: Instant) -> Result<bool, ArmDriverError> {
        self.inner.probe_tx_ready_budgeted(deadline)
    }
    fn clear_tx_probe(&mut self) { self.inner.clear_tx_probe(); }
    fn joint_modes_uncertain(&self) -> bool { self.inner.joint_modes_uncertain() }
    fn range_check(&self) -> Option<DriverRangeCheck> { self.inner.range_check() }
    fn motor_zero_support(&self) -> MotorZeroSupport { self.inner.motor_zero_support() }
    fn set_zero_position(&mut self, joint: usize) -> Result<(), ArmDriverError> { self.inner.set_zero_position(joint) }
    fn set_joint_zero_offsets(&mut self, offsets: &[f32]) { self.offsets = offsets.to_vec(); }
    fn joint_zero_offsets(&self) -> Option<Vec<f32>> { Some(self.offsets.clone()) }
    fn take_recovery_writes(&mut self) -> Vec<usize> { self.inner.take_recovery_writes() }
    fn set_strict_mode_handover(&mut self, strict: bool) { self.inner.set_strict_mode_handover(strict) }
    fn set_joint_modes_confirmed(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        match self.to_motor(hold) {
            None => self.inner.set_joint_modes_confirmed(modes, hold),
            Some(m) => self.inner.set_joint_modes_confirmed(modes, &m),
        }
    }
    fn set_mit_deadline_fence(&mut self, deadline: Option<Duration>, holdoff: Duration) { self.inner.set_mit_deadline_fence(deadline, holdoff) }
    fn mit_fence_trip(&self) -> Option<String> { self.inner.mit_fence_trip() }
    fn mit_fence_trip_bounded(
        &self,
    ) -> Option<crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>> {
        self.inner.mit_fence_trip_bounded()
    }
    fn clock_now(&self) -> std::time::Instant { self.inner.clock_now() }
    fn driver_clock(&self) -> Option<std::sync::Arc<dyn crate::transport::DriverClock>> {
        self.inner.driver_clock()
    }
    fn mit_expiry_recover(&mut self) -> Result<(), String> { self.inner.mit_expiry_recover() }
    fn mit_fresh_status(&mut self) -> Result<Vec<u8>, String> { self.inner.mit_fresh_status() }
    fn acknowledge_mit_fence_trip(&mut self) { self.inner.acknowledge_mit_fence_trip() }
    fn mit_feedback_after_recovery(&self, dof: usize) -> bool { self.inner.mit_feedback_after_recovery(dof) }
    fn read_motor_identity(&mut self, joints: &[usize]) -> Result<Vec<crate::mit::MotorIdentity>, ArmDriverError> { self.inner.read_motor_identity(joints) }
    fn commission_motor_timeout(&mut self, joints: &[usize], counts: u32) -> Result<Vec<crate::mit::TimeoutCommissioning>, crate::mit::TimeoutBatchError> {
        self.inner.commission_motor_timeout(joints, counts)
    }
}

/// Result of the zero check against the reference pose (`zero_convention`).
///
/// Stack-only (`Copy`, zero heap allocation on `armd-control`); the human-readable warning
/// `message` for `legacy_j6` is formatted off-thread on serialization or log drain.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ZeroConvention {
    /// `unchecked` (no check at the rest pose yet), `ok`, `legacy_j6`.
    pub status: &'static str,
    /// J6 reading at the check (rad).
    pub j6_rad: Option<f32>,
    /// Expected J6 rest reading (rad) for `legacy_j6` off-thread message formatting.
    expected_j6_rad: Option<f32>,
}

impl ZeroConvention {
    /// Formatted diagnostic message for `legacy_j6` (None for `unchecked` / `ok`).
    pub fn message(&self) -> Option<String> {
        if self.status != "legacy_j6" {
            return None;
        }
        let j6 = self.j6_rad?;
        let rest6 = self.expected_j6_rad.unwrap_or(0.0);
        let d6 = f64::from(j6 - rest6);
        Some(format!(
            "J6 reads {:.1} deg in the rest pose (expected {:.1} deg): this arm's J6 was zeroed with the wrist rolled 90 deg. \
             Fold the arm into the rest pose and set the zero of J6 with the zero tool (Tools > zero; software zero recommended, reversible). \
             Until then park rolls the wrist 90 deg and the 3D view / camera orientation is off by 90 deg.",
            d6.to_degrees() + f64::from(rest6).to_degrees(),
            f64::from(rest6).to_degrees(),
        ))
    }
}

impl Serialize for ZeroConvention {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        use serde::ser::SerializeStruct;
        let msg = self.message();
        let len = 1 + usize::from(msg.is_some()) + usize::from(self.j6_rad.is_some());
        let mut st = serializer.serialize_struct("ZeroConvention", len)?;
        st.serialize_field("status", self.status)?;
        if let Some(ref m) = msg {
            st.serialize_field("message", m)?;
        }
        if let Some(j6) = self.j6_rad {
            st.serialize_field("j6_rad", &j6)?;
        }
        st.end()
    }
}

impl Default for ZeroConvention {
    fn default() -> Self { Self { status: "unchecked", j6_rad: None, expected_j6_rad: None } }
}

/// Check readings `q` against the reference `rest` (joint angles) with the resting tolerances
/// `tol` (arm joints). None: the arm is not in the rest pose apart from J6 (no verdict). The
/// legacy zero: every arm joint but J6 within its tolerance and |J6 - rest| within
/// [`LEGACY_J6_TOL_RAD`] of pi/2. Allocation free.
pub fn check_convention(q: &[f32], rest: &[f32], tol: &[f32]) -> Option<ZeroConvention> {
    if q.len() < 6 || rest.len() < 6 || !q.iter().all(|v| v.is_finite()) { return None; }
    let others = (0..tol.len().min(6)).filter(|&i| i != 5).all(|i| (q[i] - rest[i]).abs() <= tol[i]);
    if !others { return None; }
    let d6 = f64::from(q[5] - rest[5]);
    if (d6.abs() - std::f64::consts::FRAC_PI_2).abs() <= LEGACY_J6_TOL_RAD {
        return Some(ZeroConvention {
            status: "legacy_j6",
            j6_rad: Some(q[5]),
            expected_j6_rad: Some(rest[5]),
        });
    }
    let tol6 = tol.get(5).copied().unwrap_or(0.08);
    (d6.abs() <= f64::from(tol6)).then_some(ZeroConvention {
        status: "ok",
        j6_rad: Some(q[5]),
        expected_j6_rad: None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn legacy_j6_detection() {
        let rest = [0.0f32; 7];
        let tol = [0.15, 0.08, 0.08, 0.08, 0.08, 0.08];
        // the office unit before its zero: J6 1.574 at rest
        let legacy = check_convention(&[0.049, 0.011, 0.027, 0.024, 0.001, 1.574, 0.052], &rest, &tol).unwrap();
        assert_eq!(legacy.status, "legacy_j6");
        assert!(check_convention(&[0.0, 0.0, 0.0, 0.0, 0.0, -1.45, 0.0], &rest, &tol).unwrap().status == "legacy_j6", "-pi/2 within 10 deg");
        assert_eq!(check_convention(&[0.01, 0.0, 0.0, 0.0, 0.0, 0.004, 0.0], &rest, &tol).unwrap().status, "ok");
        // J6 rolled 45 deg or the arm away from the rest pose: no verdict
        assert!(check_convention(&[0.0, 0.0, 0.0, 0.0, 0.0, 0.8, 0.0], &rest, &tol).is_none());
        assert!(check_convention(&[0.0, -0.5, 0.0, 0.0, 0.0, 1.57, 0.0], &rest, &tol).is_none());
        assert!(check_convention(&[0.0, 0.0, 0.0, 0.0, 0.0, 1.30, 0.0], &rest, &tol).is_none(), "75 deg is outside the 10 deg band");
    }

    #[test]
    fn joint_zero_validation() {
        let z = JointZero::new(&[0.0, 0.0, 0.0, 0.0, 0.0, 1.574, 0.0], "software", "rest");
        assert!(z.validate(Some(7)).is_ok());
        assert!(z.validate(Some(6)).is_err());
        let mut bad = z.clone();
        bad.method = "flash".into();
        assert!(bad.validate(Some(7)).is_err());
        let bad = JointZero::new(&[0.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0], "software", "rest");
        assert!(bad.validate(Some(7)).is_err());
    }
}
