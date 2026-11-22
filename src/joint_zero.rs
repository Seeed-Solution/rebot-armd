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

use std::time::Duration;

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
}

impl ZeroOffsetDriver {
    pub fn new(inner: Box<dyn ArmDriver>, offsets: &[f32]) -> Self { Self { inner, offsets: offsets.to_vec() } }

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
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        match self.to_motor(targets) {
            None => self.inner.write_joint_targets(targets),
            Some(m) => self.inner.write_joint_targets(&m),
        }
    }
    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> { self.inner.set_torque(enabled) }
    fn is_feedback_valid(&self) -> bool { self.inner.is_feedback_valid() }
    fn check_hardware_status(&self) -> Result<(), ArmDriverError> { self.inner.check_hardware_status() }
    fn clear_fault(&mut self) -> Result<(), ArmDriverError> { self.inner.clear_fault() }
    fn clear_fault_disables_joints(&self) -> Vec<usize> { self.inner.clear_fault_disables_joints() }
    fn has_known_partial_enable(&self) -> bool { self.inner.has_known_partial_enable() }
    fn has_any_joint_enabled(&self) -> bool { self.inner.has_any_joint_enabled() }
    fn refresh_enable_state(&mut self) -> Result<EnableState, ArmDriverError> { self.inner.refresh_enable_state() }
    fn set_abort_flag(&mut self, flag: std::sync::Arc<std::sync::atomic::AtomicBool>) { self.inner.set_abort_flag(flag) }
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
    fn joint_modes_uncertain(&self) -> bool { self.inner.joint_modes_uncertain() }
    fn range_check(&self) -> Option<DriverRangeCheck> { self.inner.range_check() }
    fn motor_zero_support(&self) -> MotorZeroSupport { self.inner.motor_zero_support() }
    fn set_zero_position(&mut self, joint: usize) -> Result<(), ArmDriverError> { self.inner.set_zero_position(joint) }
    fn set_joint_zero_offsets(&mut self, offsets: &[f32]) { self.offsets = offsets.to_vec(); }
    fn joint_zero_offsets(&self) -> Option<Vec<f32>> { Some(self.offsets.clone()) }
    fn take_recovery_writes(&mut self) -> Vec<usize> { self.inner.take_recovery_writes() }
    fn set_strict_mode_handover(&mut self, strict: bool) { self.inner.set_strict_mode_handover(strict) }
}

/// Result of the zero check against the reference pose (`zero_convention`).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ZeroConvention {
    /// `unchecked` (no check at the rest pose yet), `ok`, `legacy_j6`.
    pub status: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub message: Option<String>,
    /// J6 reading at the check (rad).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub j6_rad: Option<f32>,
}

impl Default for ZeroConvention {
    fn default() -> Self { Self { status: "unchecked", message: None, j6_rad: None } }
}

/// Check readings `q` against the reference `rest` (joint angles) with the resting tolerances
/// `tol` (arm joints). None: the arm is not in the rest pose apart from J6 (no verdict). The
/// legacy zero: every arm joint but J6 within its tolerance and |J6 - rest| within
/// [`LEGACY_J6_TOL_RAD`] of pi/2.
pub fn check_convention(q: &[f32], rest: &[f32], tol: &[f32]) -> Option<ZeroConvention> {
    if q.len() < 6 || rest.len() < 6 || !q.iter().all(|v| v.is_finite()) { return None; }
    let others = (0..tol.len().min(6)).filter(|&i| i != 5).all(|i| (q[i] - rest[i]).abs() <= tol[i]);
    if !others { return None; }
    let d6 = f64::from(q[5] - rest[5]);
    if (d6.abs() - std::f64::consts::FRAC_PI_2).abs() <= LEGACY_J6_TOL_RAD {
        return Some(ZeroConvention { status: "legacy_j6", j6_rad: Some(q[5]), message: Some(format!(
            "J6 reads {:.1} deg in the rest pose (expected {:.1} deg): this arm's J6 was zeroed with the wrist rolled 90 deg. \
             Fold the arm into the rest pose and set the zero of J6 with the zero tool (Tools > zero; software zero recommended, reversible). \
             Until then park rolls the wrist 90 deg and the 3D view / camera orientation is off by 90 deg.",
            d6.to_degrees() + f64::from(rest[5]).to_degrees(), f64::from(rest[5]).to_degrees())) });
    }
    let tol6 = tol.get(5).copied().unwrap_or(0.08);
    (d6.abs() <= f64::from(tol6)).then_some(ZeroConvention { status: "ok", message: None, j6_rad: Some(q[5]) })
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
