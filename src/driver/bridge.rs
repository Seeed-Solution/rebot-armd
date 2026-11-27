//! TEMPORARY (S4 PR 1a, removed in PR 1d): inherent methods on `dyn ArmDriver` with the
//! names of the methods that moved to the `*Ext` traits, so owner call sites compile
//! unchanged while they are moved to the getters one extension at a time.
#![allow(clippy::too_many_arguments, dead_code)]

use super::*;

impl dyn ArmDriver + '_ {
    #[inline]
    pub fn joint_hold_mode_verified(&self, a0: usize) -> bool {
        self.joint_mode().joint_hold_mode_verified(a0)
    }
    #[inline]
    pub fn request_mode_readback_budgeted(&mut self, a0: u64) -> bool {
        self.joint_mode_mut().request_mode_readback_budgeted(a0)
    }
    #[inline]
    pub fn with_range_check(&self, a0: &mut dyn FnMut(Option<&DriverRangeCheck>))  {
        self.joint_mode().with_range_check(a0)
    }
    #[inline]
    pub fn take_recovery_writes_into(&mut self, a0: &mut Vec<usize>)  {
        self.joint_mode_mut().take_recovery_writes_into(a0)
    }
    #[inline]
    pub fn supports_mit_mode_switch(&self) -> bool {
        self.joint_mode().supports_mit_mode_switch()
    }
    #[inline]
    pub fn set_joint_modes(&mut self, a0: &[JointMode], a1: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.joint_mode_mut().set_joint_modes(a0, a1)
    }
    #[inline]
    pub fn set_joint_modes_in_tick(&mut self, a0: &[JointMode], a1: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.joint_mode_mut().set_joint_modes_in_tick(a0, a1)
    }
    #[inline]
    pub fn joint_modes_uncertain(&self) -> bool {
        self.joint_mode().joint_modes_uncertain()
    }
    #[inline]
    pub fn set_strict_mode_handover(&mut self, a0: bool)  {
        self.joint_mode_mut().set_strict_mode_handover(a0)
    }
    #[inline]
    pub fn set_joint_modes_confirmed(&mut self, a0: &[JointMode], a1: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.joint_mode_mut().set_joint_modes_confirmed(a0, a1)
    }
    #[inline]
    pub fn set_mode_switch_hold_targets(&mut self, a0: &[Option<ModeSwitchHoldTarget>])  {
        self.joint_mode_mut().set_mode_switch_hold_targets(a0)
    }
}

impl dyn ArmDriver + '_ {
    #[inline]
    pub fn set_mit_deadline_fence(&mut self, a0: Option<Duration>, a1: Duration)  {
        self.mit_mut().set_mit_deadline_fence(a0, a1)
    }
    #[inline]
    pub fn set_mit_ownership_lost(&mut self, a0: bool)  {
        self.mit_mut().set_mit_ownership_lost(a0)
    }
    #[inline]
    pub fn mit_expiry_recover(&mut self) -> Result<(), String> {
        self.mit_mut().mit_expiry_recover()
    }
    #[inline]
    pub fn mit_expiry_recover_with_gate(&mut self, a0: &mut dyn FnMut() -> Result<(), &'static str>) -> Result<(), String> {
        self.mit_mut().mit_expiry_recover_with_gate(a0)
    }
    #[inline]
    pub fn mit_fresh_status(&mut self) -> Result<Vec<u8>, String> {
        self.mit_mut().mit_fresh_status()
    }
    #[inline]
    pub fn mit_fence_trip_bounded(&self) -> Option<crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>> {
        self.mit().mit_fence_trip_bounded()
    }
    #[inline]
    pub fn acknowledge_mit_fence_trip(&mut self)  {
        self.mit_mut().acknowledge_mit_fence_trip()
    }
    #[inline]
    pub fn mit_feedback_after_recovery(&self, a0: usize) -> bool {
        self.mit().mit_feedback_after_recovery(a0)
    }
    #[inline]
    pub fn mit_min_last_ctrl_tx(&self, a0: u64) -> Option<Instant> {
        self.mit().mit_min_last_ctrl_tx(a0)
    }
    #[inline]
    pub fn mit_expiry_recover_budgeted(&mut self, a0: Instant, a1: Instant, a2: &mut dyn FnMut() -> Result<(), &'static str>) -> Result<bool, ArmDriverError> {
        self.mit_mut().mit_expiry_recover_budgeted(a0, a1, a2)
    }
    #[inline]
    pub fn mit_expiry_recover_reset(&mut self)  {
        self.mit_mut().mit_expiry_recover_reset()
    }
}

impl dyn ArmDriver + '_ {
    #[inline]
    pub fn set_external_pos_force_emitter(&mut self, a0: bool)  {
        self.pos_force_mut().set_external_pos_force_emitter(a0)
    }
    #[inline]
    pub fn validate_pos_force_intent_target(&self, a0: f32, a1: Option<f32>, a2: Option<u16>) -> Result<(), ArmDriverError> {
        self.pos_force().validate_pos_force_intent_target(a0, a1, a2)
    }
    #[inline]
    pub fn set_pending_pos_force_hold_hint(&mut self, a0: bool)  {
        self.pos_force_mut().set_pending_pos_force_hold_hint(a0)
    }
    #[inline]
    pub fn emit_pos_force_j7(&mut self, a0: PosForceIntent, a1: PosForceIntentStamp) -> Result<PosForceEmitOutcome, ArmDriverError> {
        self.pos_force_mut().emit_pos_force_j7(a0, a1)
    }
    #[inline]
    pub fn take_last_pos_force_tx_status(&mut self) -> PosForceTxStatus {
        self.pos_force_mut().take_last_pos_force_tx_status()
    }
    #[inline]
    pub fn pos_force_feedback_fresh(&self) -> bool {
        self.pos_force().pos_force_feedback_fresh()
    }
    #[inline]
    pub fn pos_force_last_fresh_position(&self) -> Option<f32> {
        self.pos_force().pos_force_last_fresh_position()
    }
    #[inline]
    pub fn reset_pos_force_boot_state(&mut self)  {
        self.pos_force_mut().reset_pos_force_boot_state()
    }
    #[inline]
    pub fn motor_current_cap(&self) -> Option<MotorCurrentCap> {
        self.pos_force().motor_current_cap()
    }
    #[inline]
    pub fn gripper_pos_force_info(&self) -> Option<GripperPosForceInfo> {
        self.pos_force().gripper_pos_force_info()
    }
}

impl dyn ArmDriver + '_ {
    #[inline]
    pub fn service_pending_refresh(&mut self) -> Result<(), ArmDriverError> {
        self.bus_recovery_mut().service_pending_refresh()
    }
    #[inline]
    pub fn finish_dispatched_disable(&mut self) -> Result<(), ArmDriverError> {
        self.bus_recovery_mut().finish_dispatched_disable()
    }
    #[inline]
    pub fn notify_bus_loss(&mut self)  {
        self.bus_recovery_mut().notify_bus_loss()
    }
    #[inline]
    pub fn panic_disable_step(&mut self, a0: bool, a1: Instant) -> Option<Result<Vec<usize>, ArmDriverError>> {
        self.bus_recovery_mut().panic_disable_step(a0, a1)
    }
    #[inline]
    pub fn drain_rx_only_bounded(&mut self, a0: Option<Instant>) -> Result<(), ArmDriverError> {
        self.bus_recovery_mut().drain_rx_only_bounded(a0)
    }
    #[inline]
    pub fn clear_carried_over_motor_errors(&mut self, a0: Option<Instant>, a1: &mut dyn FnMut() -> Result<(), &'static str>) -> Result<CarryoverMotorClearOutcome, ArmDriverError> {
        self.bus_recovery_mut().clear_carried_over_motor_errors(a0, a1)
    }
    #[inline]
    pub fn set_pending_rearm(&mut self, a0: bool)  {
        self.bus_recovery_mut().set_pending_rearm(a0)
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl dyn ArmDriver + '_ {
    #[inline]
    pub fn shadow_vendor_kind(&self) -> crate::torque::VendorKind {
        self.shadow().shadow_vendor_kind()
    }
    #[inline]
    pub fn set_shadow_enabled(&mut self, a0: bool)  {
        self.shadow_mut().set_shadow_enabled(a0)
    }
    #[inline]
    pub fn drain_shadow_observations(&mut self, a0: &mut [Option<crate::torque::shadow::ShadowDriverObs>]) -> (usize, u64) {
        self.shadow_mut().drain_shadow_observations(a0)
    }
}

impl dyn ArmDriver + '_ {
    #[inline]
    pub fn set_joint_zero_offsets(&mut self, a0: &[f32])  {
        self.joint_zero_mut().set_joint_zero_offsets(a0)
    }
    #[inline]
    pub fn joint_zero_offsets(&self) -> Option<Vec<f32>> {
        self.joint_zero().joint_zero_offsets()
    }
}
