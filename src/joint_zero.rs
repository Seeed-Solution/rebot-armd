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
//! [`ZeroOffsetDriver`] (`src/driver/zero_offset.rs`) wraps every driver the supervisor installs and is the only place the
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

pub use crate::driver::zero_offset::ZeroOffsetDriver;

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
