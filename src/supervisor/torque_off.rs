//! Torque-off while stopped (E-stop or Fault) decision logic (Rule A′).
//!
//! Single authoritative check shared by both owners (legacy and RT executor) and
//! every torque-disable entry point (WS `/api/v1/torque`, `/api/v1/arm`,
//! SDK-facing commands, RT `TransitionOp::TorqueOff` / `ImmediateOp::TorqueOff`,
//! and the cockpit):
//! - Hardware (`driver.is_some()`) requires a calibrated `resting_pose` profile policy.
//! - Allowed only when arm-joint feedback (excluding the gripper) is fresh (<= 100 ms)
//!   and every arm joint is within `config.rest_pose_tolerance_rad` (default 0.05 rad)
//!   of the profile's rest pose (`config.resting_positions()`).
//! - Refused otherwise with `TORQUE_OFF_WOULD_DROP`. There is NO override.
//! - Zero heap allocations on the decision itself so the RT control thread can call
//!   it directly on both allowed and refused paths.

use std::time::Duration;

use super::SupervisorInner;
use crate::safety::SafetyError;

/// Maximum feedback age accepted for proving the arm is at the rest pose while stopped.
pub const STOPPED_TORQUE_OFF_FEEDBACK_MAX_AGE: Duration =
    crate::driver::STOPPED_DISABLE_FEEDBACK_MAX_AGE;

/// Zero-allocation refusal reason from [`SupervisorInner::may_disable_torque_while_stopped`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StoppedTorqueOffRefusal {
    /// Hardware profile has no calibrated `resting_pose` policy configured.
    NoRestingPoseConfigured,
    /// Arm-joint feedback is missing, numerically invalid, or older than 100 ms (counts as not at rest).
    StaleOrMissingFeedback,
    /// First arm joint (excluding the gripper) outside `rest_pose_tolerance_rad` of its rest pose.
    JointOutsideRest {
        joint_index: usize,
        position: f32,
        rest: f32,
        tolerance: f32,
    },
}

impl StoppedTorqueOffRefusal {
    /// Machine-readable refusal sub-reason for localized client formatting.
    pub const fn reason_code(self) -> &'static str {
        match self {
            Self::NoRestingPoseConfigured => "no_resting_pose",
            Self::StaleOrMissingFeedback => "stale_feedback",
            Self::JointOutsideRest { .. } => "outside_rest_pose",
        }
    }

    /// Zero-allocation conversion into [`crate::safety::ClearFaultError::TorqueOffWouldDrop`]
    /// for RT executor clear paths whose hardware reset transmits torque-off.
    pub const fn into_clear_fault_error(self) -> crate::safety::ClearFaultError {
        let reason = self.reason_code();
        match self {
            Self::NoRestingPoseConfigured | Self::StaleOrMissingFeedback => {
                crate::safety::ClearFaultError::TorqueOffWouldDrop {
                    joint_index: None,
                    position: None,
                    rest: None,
                    tolerance: None,
                    reason,
                }
            }
            Self::JointOutsideRest {
                joint_index,
                position,
                rest,
                tolerance,
            } => crate::safety::ClearFaultError::TorqueOffWouldDrop {
                joint_index: Some(joint_index),
                position: Some(position),
                rest: Some(rest),
                tolerance: Some(tolerance),
                reason,
            },
        }
    }

    /// Convert into the protocol-facing [`SafetyError::TorqueOffWouldDrop`] naming the first
    /// offending joint, its position, its rest value, and warning that the arm would drop.
    pub fn into_safety_error(self, joint_names: &[String]) -> SafetyError {
        let reason = Some(self.reason_code().to_string());
        match self {
            Self::NoRestingPoseConfigured => SafetyError::TorqueOffWouldDrop {
                joint: None,
                position: None,
                rest: None,
                reason,
                message: "Torque stays on: profile has no calibrated resting_pose configured; the arm would drop. Clear the fault and park, or cut power.".into(),
            },
            Self::StaleOrMissingFeedback => SafetyError::TorqueOffWouldDrop {
                joint: None,
                position: None,
                rest: None,
                reason,
                message: "Torque stays on: feedback is missing or older than 100 ms, so the rest pose cannot be verified; the arm would drop. Wait for fresh feedback, clear the fault and park, or cut power.".into(),
            },
            Self::JointOutsideRest {
                joint_index,
                position,
                rest,
                tolerance,
            } => {
                let joint = joint_names
                    .get(joint_index)
                    .cloned()
                    .unwrap_or_else(|| format!("J{}", joint_index + 1));
                let message = format!(
                    "Torque stays on: {joint} is at {position:.3} rad (rest {rest:.3} rad, tolerance {tolerance:.3} rad); the arm would drop. Clear the fault and park, or cut power."
                );
                SafetyError::TorqueOffWouldDrop {
                    joint: Some(joint),
                    position: Some(position),
                    rest: Some(rest),
                    reason,
                    message,
                }
            }
        }
    }
}

impl SupervisorInner {
    /// Last zero-allocation refusal recorded by the stopped torque-off decision (cleared on
    /// an allowed stopped disable), available for off-thread reply formatters.
    pub fn last_stopped_torque_off_refusal(&self) -> Option<StoppedTorqueOffRefusal> {
        self.last_stopped_torque_off_refusal
    }

    /// Single authoritative decision for whether torque may be disabled while in E-stop or Fault.
    ///
    /// - Hardware (`driver.is_some()`) requires a calibrated `config.resting_pose`.
    /// - On hardware, reads arm-joint positions directly from the driver's SAME fresh feedback
    ///   snapshot whose timestamps and receive-queue bounds are checked (<= 100 ms, J1..J6 only;
    ///   stale, invalid, or missing arm feedback counts as not at rest, and gripper feedback is
    ///   excluded). Driverless synthetic mode relies on `feedback_numeric_valid`, any explicit
    ///   `last_can_feedback_instant`, and `measured_positions` on J1..J6 only.
    /// - Checks every arm joint (`config.arm_joints()`, excluding the gripper) against
    ///   `config.resting_positions()` within `config.rest_pose_tolerance_rad` (default 0.05 rad).
    /// - Zero heap allocations.
    ///
    /// Rest-pose gate check against an already-locked hardware driver so callers holding the
    /// driver mutex (e.g. `mit_expiry_recovery_for_epoch`) can verify Rule A′ with zero
    /// check-to-command gap and zero lock re-entry.
    pub(crate) fn check_stopped_torque_off_with_driver(
        config: &crate::model::ArmConfig,
        drv: &dyn crate::driver::ArmDriver,
    ) -> Result<(), StoppedTorqueOffRefusal> {
        if config.resting_pose.is_none() {
            return Err(StoppedTorqueOffRefusal::NoRestingPoseConfigured);
        }
        let tol = config.rest_pose_tolerance_rad;
        if !tol.is_finite() || tol <= 0.0 {
            return Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback);
        }
        let rest = config.resting_positions();
        let arm_joints = config.arm_joints();
        let mut arm_positions = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
        if arm_joints.is_empty()
            || arm_joints.len() > arm_positions.len()
            || rest.len() < config.dof
        {
            return Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback);
        }
        let fresh = drv.fresh_joint_positions_into(
            &arm_joints,
            STOPPED_TORQUE_OFF_FEEDBACK_MAX_AGE,
            &mut arm_positions[..arm_joints.len()],
        );
        let status = drv.rest_pose_gate_status();
        if !fresh {
            return Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback);
        }
        for (&i, &pos) in arm_joints.iter().zip(arm_positions[..arm_joints.len()].iter()) {
            let rest_pos = rest[i];
            if !pos.is_finite() || !rest_pos.is_finite() {
                return Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback);
            }
            if (pos - rest_pos).abs() > tol {
                return Err(StoppedTorqueOffRefusal::JointOutsideRest {
                    joint_index: i,
                    position: pos,
                    rest: rest_pos,
                    tolerance: tol,
                });
            }
        }
        if status != crate::driver::RestPoseGateStatus::Confirmed {
            return Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback);
        }
        Ok(())
    }

    /// Non-blocking stopped torque-off evaluation for `armd-control` (M4): uses `driver.try_lock()`,
    /// returning `Err(())` on mutex contention (`TryLockError::WouldBlock`) instead of blocking.
    pub(crate) fn may_disable_torque_while_stopped_nonblocking(
        &self,
    ) -> Result<Result<(), StoppedTorqueOffRefusal>, ()> {
        if let Some(d) = self.driver.as_ref() {
            return match d.try_lock() {
                Ok(drv) => Ok(Self::check_stopped_torque_off_with_driver(&self.config, drv.as_ref())),
                Err(std::sync::TryLockError::WouldBlock) => Err(()),
                Err(std::sync::TryLockError::Poisoned(_)) => {
                    Ok(Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback))
                }
            };
        }
        Ok(self.may_disable_torque_while_stopped_driverless())
    }

    pub fn may_disable_torque_while_stopped(&self) -> Result<(), StoppedTorqueOffRefusal> {
        if let Some(d) = self.driver.as_ref() {
            if self.rt_executor_owned {
                return match d.try_lock() {
                    Ok(drv) => Self::check_stopped_torque_off_with_driver(&self.config, drv.as_ref()),
                    Err(_) => Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback),
                };
            }
            return match d.lock() {
                Ok(drv) => Self::check_stopped_torque_off_with_driver(&self.config, drv.as_ref()),
                Err(_) => Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback),
            };
        }
        self.may_disable_torque_while_stopped_driverless()
    }

    fn may_disable_torque_while_stopped_driverless(&self) -> Result<(), StoppedTorqueOffRefusal> {
        let tol = self.config.rest_pose_tolerance_rad;
        if !tol.is_finite() || tol <= 0.0 {
            return Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback);
        }
        let rest = self.config.resting_positions();
        let arm_joints = self.config.arm_joints();
        if arm_joints.is_empty()
            || arm_joints.len() > crate::rt_core::mailbox::MAX_DOF
            || rest.len() < self.config.dof
        {
            return Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback);
        }
        if !self.feedback_numeric_valid
            || self.measured_positions.len() < self.config.dof
            || !self
                .last_can_feedback_instant
                .is_none_or(|t| t.elapsed() <= STOPPED_TORQUE_OFF_FEEDBACK_MAX_AGE)
        {
            return Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback);
        }
        for &i in arm_joints.iter() {
            let pos = self.measured_positions[i];
            let rest_pos = rest[i];
            if !pos.is_finite() || !rest_pos.is_finite() {
                return Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback);
            }
            if (pos - rest_pos).abs() > tol {
                return Err(StoppedTorqueOffRefusal::JointOutsideRest {
                    joint_index: i,
                    position: pos,
                    rest: rest_pos,
                    tolerance: tol,
                });
            }
        }
        Ok(())
    }

    /// Evaluates [`Self::may_disable_torque_while_stopped`] once and records any refusal in
    /// `last_stopped_torque_off_refusal` (zero heap allocations).
    pub(crate) fn eval_stopped_torque_off(&mut self) -> Result<(), StoppedTorqueOffRefusal> {
        match self.may_disable_torque_while_stopped() {
            Ok(()) => {
                self.last_stopped_torque_off_refusal = None;
                Ok(())
            }
            Err(r) => {
                self.last_stopped_torque_off_refusal = Some(r);
                Err(r)
            }
        }
    }
}
