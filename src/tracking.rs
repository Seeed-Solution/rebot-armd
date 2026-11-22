//! Joint tracking compensation (Phase 1: standalone position lead, default off).
//!
//! Keeps nominal reference (`q_d, v_d, a_d`), measured state (`q_m, v_m, tau_m`),
//! and physical driver output (`q_out`) distinct. Tracking applies once at the
//! ordinary driver output boundary, before joint-zero conversion.

use serde::{Deserialize, Serialize};

/// Maximum allowed position-lead lag constant (s).
pub const MAX_TRACKING_LAG_S: f32 = 1.5;
/// Default and upper bound for per-joint position lead (rad).
pub const DEFAULT_MAX_LEAD_RAD: f32 = 0.030;
/// Maximum allowed per-joint position lead (rad).
pub const MAX_LEAD_RAD_LIMIT: f32 = 0.030;

/// Per-joint tracking control law.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TrackingStrategy {
    #[default]
    Position,
    PositionLead,
    Mit,
}

/// Per-joint tracking parameters (arm joints only, profile joint order).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct JointTrackingJoint {
    pub strategy: TrackingStrategy,
    pub lag_s: f32,
    pub max_lead_rad: f32,
    pub kp: f32,
    pub kd: f32,
    pub gravity_scale: f32,
    pub coulomb_nm: f32,
    pub viscous_nm_s_rad: f32,
    pub inertia_kg_m2: f32,
    pub effort_limit_nm: f32,
}

impl Default for JointTrackingJoint {
    fn default() -> Self {
        Self {
            strategy: TrackingStrategy::Position,
            lag_s: 0.0,
            max_lead_rad: DEFAULT_MAX_LEAD_RAD,
            kp: 0.0,
            kd: 0.0,
            gravity_scale: 0.0,
            coulomb_nm: 0.0,
            viscous_nm_s_rad: 0.0,
            inertia_kg_m2: 0.0,
            effort_limit_nm: 0.0,
        }
    }
}

/// Startup joint-tracking configuration (`ArmConfig.joint_tracking`).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct JointTrackingConfig {
    pub enabled: bool,
    pub joints: Vec<JointTrackingJoint>,
}

impl JointTrackingConfig {
    /// Disabled commissioning seed for the reBot Arm B601-DM (through-origin
    /// lag fits from `creep2.jsonl`: J1..J6 = `[0.68, 0.65, 0.71, 0.17, 0, 0]`).
    pub fn rebot_dm() -> Self {
        let seed_lags = [0.68_f32, 0.65, 0.71, 0.17, 0.0, 0.0];
        Self {
            enabled: false,
            joints: seed_lags
                .into_iter()
                .map(|lag_s| JointTrackingJoint {
                    strategy: TrackingStrategy::PositionLead,
                    lag_s,
                    ..Default::default()
                })
                .collect(),
        }
    }

    /// Validate schema invariants for Phase 1 (`position` and `position_lead` only;
    /// `mit` is rejected in Phase 1).
    pub fn validate(&self, arm_dof: usize, dof: usize, gripper_index: Option<usize>) -> Result<(), String> {
        if let Some(g) = gripper_index {
            if self.joints.len() > g {
                return Err(format!(
                    "joint_tracking.joints length {} includes gripper index {} (expected {} arm joints only)",
                    self.joints.len(),
                    g,
                    arm_dof
                ));
            }
        }
        if arm_dof == 0 || arm_dof > dof || self.joints.len() != arm_dof {
            return Err(format!(
                "joint_tracking.joints length {} must equal arm_dof {} (profile dof {})",
                self.joints.len(),
                arm_dof,
                dof
            ));
        }
        for (i, j) in self.joints.iter().enumerate() {
            let finite_nonneg = |name: &str, v: f32| -> Result<(), String> {
                if !v.is_finite() || v < 0.0 {
                    return Err(format!(
                        "joint_tracking.joints[{i}].{name} must be finite and >= 0, got {v}"
                    ));
                }
                Ok(())
            };
            finite_nonneg("lag_s", j.lag_s)?;
            if j.lag_s > MAX_TRACKING_LAG_S {
                return Err(format!(
                    "joint_tracking.joints[{i}].lag_s must be in [0, {MAX_TRACKING_LAG_S}], got {}",
                    j.lag_s
                ));
            }
            finite_nonneg("max_lead_rad", j.max_lead_rad)?;
            if j.max_lead_rad > MAX_LEAD_RAD_LIMIT {
                return Err(format!(
                    "joint_tracking.joints[{i}].max_lead_rad must be in [0, {MAX_LEAD_RAD_LIMIT}], got {}",
                    j.max_lead_rad
                ));
            }
            finite_nonneg("kp", j.kp)?;
            finite_nonneg("kd", j.kd)?;
            finite_nonneg("gravity_scale", j.gravity_scale)?;
            if j.gravity_scale > 1.0 {
                return Err(format!(
                    "joint_tracking.joints[{i}].gravity_scale must be in [0, 1], got {}",
                    j.gravity_scale
                ));
            }
            finite_nonneg("coulomb_nm", j.coulomb_nm)?;
            finite_nonneg("viscous_nm_s_rad", j.viscous_nm_s_rad)?;
            finite_nonneg("inertia_kg_m2", j.inertia_kg_m2)?;
            finite_nonneg("effort_limit_nm", j.effort_limit_nm)?;

            if matches!(j.strategy, TrackingStrategy::Mit) {
                return Err(format!(
                    "joint_tracking.joints[{i}]: strategy 'mit' is not supported in Phase 1 (only 'position' and 'position_lead' are accepted)"
                ));
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_and_snake_case_serialization() {
        let j = JointTrackingJoint::default();
        assert_eq!(j.strategy, TrackingStrategy::Position);
        assert_eq!(j.lag_s, 0.0);
        assert_eq!(j.max_lead_rad, 0.030);
        assert_eq!(j.kp, 0.0);
        assert_eq!(j.kd, 0.0);
        assert_eq!(j.gravity_scale, 0.0);
        assert_eq!(j.coulomb_nm, 0.0);
        assert_eq!(j.viscous_nm_s_rad, 0.0);
        assert_eq!(j.inertia_kg_m2, 0.0);
        assert_eq!(j.effort_limit_nm, 0.0);

        let parsed: JointTrackingJoint = serde_json::from_str(r#"{"strategy":"position_lead","lag_s":0.5}"#).unwrap();
        assert_eq!(parsed.strategy, TrackingStrategy::PositionLead);
        assert_eq!(parsed.lag_s, 0.5);
        assert_eq!(parsed.max_lead_rad, 0.030);

        let s = serde_json::to_string(&TrackingStrategy::PositionLead).unwrap();
        assert_eq!(s, "\"position_lead\"");
        assert_eq!(serde_json::to_string(&TrackingStrategy::Position).unwrap(), "\"position\"");
        assert_eq!(serde_json::to_string(&TrackingStrategy::Mit).unwrap(), "\"mit\"");
    }

    #[test]
    fn serde_rejects_unknown_fields_and_unknown_strategy() {
        assert!(serde_json::from_str::<JointTrackingConfig>(r#"{"enabled":false,"joints":[],"extra":1}"#).is_err());
        assert!(serde_json::from_str::<JointTrackingJoint>(r#"{"strategy":"position","extra":1}"#).is_err());
        assert!(serde_json::from_str::<JointTrackingJoint>(r#"{"strategy":"pd_plus"}"#).is_err());
    }

    #[test]
    fn validation_rejects_mit_wrong_length_gripper_and_bad_bounds() {
        let mut cfg = JointTrackingConfig::rebot_dm();
        assert!(cfg.validate(6, 7, Some(6)).is_ok());

        // Gripper entry rejected (7 entries when gripper is at index 6)
        cfg.joints.push(JointTrackingJoint::default());
        let err = cfg.validate(6, 7, Some(6)).unwrap_err();
        assert!(err.contains("gripper"), "{err}");

        // Short length rejected
        cfg.joints.truncate(5);
        assert!(cfg.validate(6, 7, Some(6)).is_err());

        // MIT strategy rejected in Phase 1 even with positive gains/effort and enabled=false
        let mut mit_cfg = JointTrackingConfig::rebot_dm();
        mit_cfg.joints[0].strategy = TrackingStrategy::Mit;
        mit_cfg.joints[0].kp = 10.0;
        mit_cfg.joints[0].kd = 0.5;
        mit_cfg.joints[0].effort_limit_nm = 5.0;
        assert!(mit_cfg.validate(6, 7, Some(6)).unwrap_err().contains("mit"));

        // Non-finite, negative, out-of-range
        for mutate in [
            |j: &mut JointTrackingJoint| j.lag_s = -0.01,
            |j: &mut JointTrackingJoint| j.lag_s = 1.51,
            |j: &mut JointTrackingJoint| j.lag_s = f32::NAN,
            |j: &mut JointTrackingJoint| j.lag_s = f32::INFINITY,
            |j: &mut JointTrackingJoint| j.max_lead_rad = -0.001,
            |j: &mut JointTrackingJoint| j.max_lead_rad = 0.031,
            |j: &mut JointTrackingJoint| j.kp = -1.0,
            |j: &mut JointTrackingJoint| j.kd = -0.1,
            |j: &mut JointTrackingJoint| j.gravity_scale = -0.1,
            |j: &mut JointTrackingJoint| j.gravity_scale = 1.01,
            |j: &mut JointTrackingJoint| j.coulomb_nm = -0.1,
            |j: &mut JointTrackingJoint| j.viscous_nm_s_rad = -0.1,
            |j: &mut JointTrackingJoint| j.inertia_kg_m2 = -0.1,
            |j: &mut JointTrackingJoint| j.effort_limit_nm = -0.1,
        ] {
            let mut c = JointTrackingConfig::rebot_dm();
            mutate(&mut c.joints[0]);
            assert!(c.validate(6, 7, Some(6)).is_err());
        }
    }
}
