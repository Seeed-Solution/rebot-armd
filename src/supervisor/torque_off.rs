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
pub const STOPPED_TORQUE_OFF_FEEDBACK_MAX_AGE: Duration = Duration::from_millis(100);

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
    /// - Checks arm-joint feedback freshness (<= 100 ms, excluding the gripper; stale or missing
    ///   arm feedback counts as not at rest). Driverless synthetic mode relies on
    ///   `feedback_numeric_valid` and any explicit `last_can_feedback_instant`.
    /// - Checks every arm joint (`config.arm_joints()`, excluding the gripper) against
    ///   `config.resting_positions()` within `config.rest_pose_tolerance_rad` (default 0.05 rad).
    /// - Zero heap allocations.
    pub fn may_disable_torque_while_stopped(&self) -> Result<(), StoppedTorqueOffRefusal> {
        if self.driver.is_some() && self.config.resting_pose.is_none() {
            return Err(StoppedTorqueOffRefusal::NoRestingPoseConfigured);
        }

        let tol = self.config.rest_pose_tolerance_rad;
        if !tol.is_finite() || tol <= 0.0 {
            return Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback);
        }

        let rest = self.config.resting_positions();
        let arm_joints = self.config.arm_joints();
        if arm_joints.is_empty()
            || self.measured_positions.len() < self.config.dof
            || rest.len() < self.config.dof
        {
            return Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback);
        }

        let feedback_fresh = match self.driver.as_ref() {
            Some(d) => {
                self.feedback_numeric_valid
                    && arm_joints
                        .iter()
                        .all(|&i| self.core.feedback().instant(i).is_some())
                    && d.lock().is_ok_and(|drv| {
                        drv.feedback_joints_within(
                            &arm_joints,
                            STOPPED_TORQUE_OFF_FEEDBACK_MAX_AGE,
                        )
                    })
            }
            None => {
                self.feedback_numeric_valid
                    && self
                        .last_can_feedback_instant
                        .is_none_or(|t| t.elapsed() <= STOPPED_TORQUE_OFF_FEEDBACK_MAX_AGE)
            }
        };
        if !feedback_fresh {
            return Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback);
        }

        for i in arm_joints.iter().copied() {
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
