//! Unit conversions (`rad`, `so101_norm`), normalization, and joint reordering
//! matching `rebot_companion/units.py` and `rebot_companion/policy_runner.py`.

use std::collections::HashSet;
use std::f64::consts::PI;

use crate::model::{ArmConfig, JointLimit};

use super::manifest::{StrictPolicyContract, UnitMode};
use super::PolicyError;

/// Bidirectional permutation between `ArmConfig` joint order and policy `joint_names` order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct JointPermutation {
    /// `policy_vec[p] = arm_vec[arm_to_policy[p]]`
    pub arm_to_policy: Vec<usize>,
    /// `arm_vec[a] = policy_vec[policy_to_arm[a]]`
    pub policy_to_arm: Vec<usize>,
}

impl JointPermutation {
    /// Build a permutation mapping between `policy_joint_names` and `arm_limits`.
    ///
    /// Fails if lengths differ, any name is empty or duplicated, or any policy joint name
    /// is missing from `arm_limits`.
    pub fn build(policy_joint_names: &[String], arm_limits: &[JointLimit]) -> Result<Self, PolicyError> {
        if policy_joint_names.len() != arm_limits.len() {
            return Err(PolicyError::DimensionMismatch {
                expected: arm_limits.len(),
                actual: policy_joint_names.len(),
            });
        }
        if arm_limits.is_empty() {
            return Err(PolicyError::InvalidUnits("joint list must not be empty".into()));
        }

        let mut seen_arm = HashSet::with_capacity(arm_limits.len());
        for (idx, lim) in arm_limits.iter().enumerate() {
            if lim.name.trim().is_empty() {
                return Err(PolicyError::InvalidUnits(format!(
                    "arm limit[{idx}] has an empty joint name"
                )));
            }
            if !seen_arm.insert(lim.name.as_str()) {
                return Err(PolicyError::InvalidUnits(format!(
                    "arm limits contain duplicate joint name {:?}",
                    lim.name
                )));
            }
        }

        let mut seen_policy = HashSet::with_capacity(policy_joint_names.len());
        let mut arm_to_policy = Vec::with_capacity(policy_joint_names.len());
        let mut policy_to_arm = vec![usize::MAX; arm_limits.len()];

        for (p_idx, p_name) in policy_joint_names.iter().enumerate() {
            if p_name.trim().is_empty() {
                return Err(PolicyError::InvalidUnits(format!(
                    "policy joint_names[{p_idx}] is empty"
                )));
            }
            if !seen_policy.insert(p_name.as_str()) {
                return Err(PolicyError::InvalidUnits(format!(
                    "policy joint_names contains duplicate {:?}",
                    p_name
                )));
            }
            let a_idx = arm_limits
                .iter()
                .position(|lim| lim.name == *p_name)
                .ok_or_else(|| {
                    PolicyError::InvalidUnits(format!(
                        "policy joint {p_name:?} not found in arm limits"
                    ))
                })?;
            arm_to_policy.push(a_idx);
            policy_to_arm[a_idx] = p_idx;
        }

        Ok(Self {
            arm_to_policy,
            policy_to_arm,
        })
    }

    pub fn from_arm_config(
        contract: &StrictPolicyContract,
        arm: &ArmConfig,
    ) -> Result<Self, PolicyError> {
        Self::build(&contract.joint_names, &arm.limits)
    }

    pub fn is_identity(&self) -> bool {
        self.arm_to_policy.iter().enumerate().all(|(i, &a)| i == a)
    }

    pub fn reorder_arm_to_policy(&self, arm: &[f32], policy: &mut [f32]) -> Result<(), PolicyError> {
        let n = self.arm_to_policy.len();
        if arm.len() != n {
            return Err(PolicyError::DimensionMismatch {
                expected: n,
                actual: arm.len(),
            });
        }
        if policy.len() != n {
            return Err(PolicyError::DimensionMismatch {
                expected: n,
                actual: policy.len(),
            });
        }
        for (p_idx, &a_idx) in self.arm_to_policy.iter().enumerate() {
            policy[p_idx] = arm[a_idx];
        }
        Ok(())
    }

    pub fn reorder_policy_to_arm(&self, policy: &[f32], arm: &mut [f32]) -> Result<(), PolicyError> {
        let n = self.policy_to_arm.len();
        if policy.len() != n {
            return Err(PolicyError::DimensionMismatch {
                expected: n,
                actual: policy.len(),
            });
        }
        if arm.len() != n {
            return Err(PolicyError::DimensionMismatch {
                expected: n,
                actual: arm.len(),
            });
        }
        for (a_idx, &p_idx) in self.policy_to_arm.iter().enumerate() {
            arm[a_idx] = policy[p_idx];
        }
        Ok(())
    }

    pub fn reordered_limits(&self, arm_limits: &[JointLimit]) -> Result<Vec<JointLimit>, PolicyError> {
        if arm_limits.len() != self.arm_to_policy.len() {
            return Err(PolicyError::DimensionMismatch {
                expected: self.arm_to_policy.len(),
                actual: arm_limits.len(),
            });
        }
        Ok(self
            .arm_to_policy
            .iter()
            .map(|&a_idx| arm_limits[a_idx].clone())
            .collect())
    }

    pub fn policy_index_for_arm_joint(&self, arm_idx: usize) -> Option<usize> {
        self.policy_to_arm.get(arm_idx).copied()
    }
}

fn check_all_finite(values: &[f32], label: &str) -> Result<(), PolicyError> {
    for (idx, &v) in values.iter().enumerate() {
        if !v.is_finite() {
            return Err(PolicyError::NonFinite(format!(
                "{label}[{idx}] is non-finite: {v}"
            )));
        }
    }
    Ok(())
}

fn extract_limit_f64(limit: Option<&JointLimit>) -> Result<(f64, f64), PolicyError> {
    let Some(lim) = limit else {
        return Ok((-PI, PI));
    };
    let lo = lim.min_position as f64;
    let hi = lim.max_position as f64;
    if !lo.is_finite() || !hi.is_finite() {
        return Err(PolicyError::NonFinite(format!(
            "joint '{}' has non-finite limits [{}, {}]",
            lim.name, lim.min_position, lim.max_position
        )));
    }
    Ok((lo, hi))
}

fn resolve_gripper_bounds(
    limits: &[JointLimit],
    gripper_index: Option<usize>,
    gripper_bounds: Option<(f32, f32)>,
) -> Result<(f64, f64), PolicyError> {
    if let Some((closed, open)) = gripper_bounds {
        let c = closed as f64;
        let o = open as f64;
        if !c.is_finite() || !o.is_finite() {
            return Err(PolicyError::NonFinite(
                "gripper_bounds must be finite".into(),
            ));
        }
        if (o - c).abs() <= 1e-9 {
            return Err(PolicyError::InvalidUnits(format!(
                "gripper closed_rad ({c}) and open_rad ({o}) cannot be equal"
            )));
        }
        return Ok((c, o));
    }

    let has_lim = gripper_index.is_some_and(|idx| idx < limits.len());
    let (lo, hi) = if has_lim {
        let idx = gripper_index.unwrap();
        let (lo_raw, hi_raw) = extract_limit_f64(Some(&limits[idx]))?;
        let (lo, hi) = if lo_raw <= hi_raw {
            (lo_raw, hi_raw)
        } else {
            (hi_raw, lo_raw)
        };
        if (hi - lo).abs() <= 1e-9 {
            return Err(PolicyError::InvalidUnits(format!(
                "gripper limits have zero span: [{lo}, {hi}]"
            )));
        }
        (Some(lo), Some(hi))
    } else {
        (None, None)
    };

    let o = if let (Some(lo), Some(hi)) = (lo, hi) {
        if gripper_index == Some(6) && lo <= -5.7 && -5.7 <= hi && (0.0 - (-5.7_f64)).abs() > 1e-6 {
            -5.7
        } else if (hi - 0.0).abs() >= (lo - 0.0).abs() {
            hi
        } else {
            lo
        }
    } else if gripper_index == Some(6) {
        -5.7
    } else {
        1.5
    };

    let c = if let (Some(lo), Some(hi)) = (lo, hi) {
        if lo <= 0.0 && 0.0 <= hi && (o - 0.0).abs() > 1e-6 {
            0.0
        } else if (lo - o).abs() >= (hi - o).abs() {
            lo
        } else {
            hi
        }
    } else if gripper_index == Some(6) {
        if (o - 0.0).abs() > 1e-6 {
            0.0
        } else if (o - (-5.7)).abs() > 1e-6 {
            -5.7
        } else {
            1.5
        }
    } else if (o - 0.0).abs() > 1e-6 {
        0.0
    } else {
        1.5
    };

    if !c.is_finite() || !o.is_finite() || (o - c).abs() <= 1e-9 {
        return Err(PolicyError::InvalidUnits(format!(
            "resolved invalid gripper bounds: closed_rad={c}, open_rad={o}"
        )));
    }
    Ok((c, o))
}

/// Convert armd internal radians into policy units (`rad` or `so101_norm`).
///
/// Matches `rebot_companion.units.from_armd` including reversed gripper bounds (`closed_rad > open_rad`).
pub fn rad_to_policy_units(
    values: &[f32],
    mode: UnitMode,
    limits: &[JointLimit],
    gripper_index: Option<usize>,
    gripper_bounds: Option<(f32, f32)>,
) -> Result<Vec<f32>, PolicyError> {
    check_all_finite(values, "rad_to_policy_units input")?;
    if let Some(g) = gripper_index {
        if g >= values.len() {
            return Err(PolicyError::InvalidUnits(format!(
                "gripper_index {g} out of bounds for {} values",
                values.len()
            )));
        }
    }

    match mode {
        UnitMode::Rad => Ok(values.to_vec()),
        UnitMode::So101Norm => {
            if !limits.is_empty() && limits.len() != values.len() {
                return Err(PolicyError::DimensionMismatch {
                    expected: values.len(),
                    actual: limits.len(),
                });
            }
            let mut out = Vec::with_capacity(values.len());
            for (i, &x) in values.iter().enumerate() {
                if gripper_index == Some(i) {
                    let (c, o) = resolve_gripper_bounds(limits, gripper_index, gripper_bounds)?;
                    let (lo_c, hi_c) = if c <= o { (c, o) } else { (o, c) };
                    let x_clamped = (x as f64).clamp(lo_c, hi_c);
                    let norm = if (o - c).abs() == 0.0 {
                        0.0
                    } else {
                        100.0 * (x_clamped - c) / (o - c)
                    };
                    out.push(norm.clamp(0.0, 100.0) as f32);
                } else {
                    let (lo, hi) = extract_limit_f64(limits.get(i))?;
                    if (hi - lo).abs() == 0.0 {
                        out.push(0.0);
                    } else {
                        let (lo_b, hi_b) = if lo <= hi { (lo, hi) } else { (hi, lo) };
                        let x_clamped = (x as f64).clamp(lo_b, hi_b);
                        let norm = 200.0 * (x_clamped - lo) / (hi - lo) - 100.0;
                        out.push(norm.clamp(-100.0, 100.0) as f32);
                    }
                }
            }
            Ok(out)
        }
    }
}

/// Convert policy units (`rad` or `so101_norm`) back into armd internal radians.
///
/// Matches `rebot_companion.units.to_armd` including reversed gripper bounds (`closed_rad > open_rad`).
pub fn policy_units_to_rad(
    values: &[f32],
    mode: UnitMode,
    limits: &[JointLimit],
    gripper_index: Option<usize>,
    gripper_bounds: Option<(f32, f32)>,
) -> Result<Vec<f32>, PolicyError> {
    check_all_finite(values, "policy_units_to_rad input")?;
    if let Some(g) = gripper_index {
        if g >= values.len() {
            return Err(PolicyError::InvalidUnits(format!(
                "gripper_index {g} out of bounds for {} values",
                values.len()
            )));
        }
    }

    match mode {
        UnitMode::Rad => Ok(values.to_vec()),
        UnitMode::So101Norm => {
            if !limits.is_empty() && limits.len() != values.len() {
                return Err(PolicyError::DimensionMismatch {
                    expected: values.len(),
                    actual: limits.len(),
                });
            }
            let mut out = Vec::with_capacity(values.len());
            for (i, &v) in values.iter().enumerate() {
                if gripper_index == Some(i) {
                    let (c, o) = resolve_gripper_bounds(limits, gripper_index, gripper_bounds)?;
                    let norm = (v as f64).clamp(0.0, 100.0);
                    let rad = c + (norm / 100.0) * (o - c);
                    out.push(rad as f32);
                } else {
                    let (lo, hi) = extract_limit_f64(limits.get(i))?;
                    let norm = (v as f64).clamp(-100.0, 100.0);
                    let rad = lo + (norm + 100.0) / 200.0 * (hi - lo);
                    out.push(rad as f32);
                }
            }
            Ok(out)
        }
    }
}

/// Normalize state in-place into `out`: `(state[i] - mean[i]) / std[i]`.
pub fn normalize_state_into(
    state: &[f32],
    mean: &[f32],
    std: &[f32],
    out: &mut [f32],
) -> Result<(), PolicyError> {
    if state.len() != mean.len() {
        return Err(PolicyError::DimensionMismatch {
            expected: mean.len(),
            actual: state.len(),
        });
    }
    if state.len() != std.len() {
        return Err(PolicyError::DimensionMismatch {
            expected: std.len(),
            actual: state.len(),
        });
    }
    if state.len() != out.len() {
        return Err(PolicyError::DimensionMismatch {
            expected: state.len(),
            actual: out.len(),
        });
    }
    for i in 0..state.len() {
        let x = state[i];
        let m = mean[i];
        let s = std[i];
        if !x.is_finite() || !m.is_finite() || !s.is_finite() {
            return Err(PolicyError::NonFinite(format!(
                "non-finite input to normalize_state at index {i}"
            )));
        }
        if s <= 0.0 {
            return Err(PolicyError::InvalidUnits(format!(
                "normalize_state std[{i}] must be > 0.0, got {s}"
            )));
        }
        let y = (x - m) / s;
        if !y.is_finite() {
            return Err(PolicyError::NonFinite(format!(
                "normalize_state produced non-finite output at index {i}"
            )));
        }
        out[i] = y;
    }
    Ok(())
}

/// Normalize state: `(state[i] - mean[i]) / std[i]`.
pub fn normalize_state(state: &[f32], mean: &[f32], std: &[f32]) -> Result<Vec<f32>, PolicyError> {
    let mut out = vec![0.0f32; state.len()];
    normalize_state_into(state, mean, std, &mut out)?;
    Ok(out)
}

/// Denormalize action in-place into `out`: `action[i] * std[i] + mean[i]`.
pub fn denormalize_action_into(
    action: &[f32],
    mean: &[f32],
    std: &[f32],
    out: &mut [f32],
) -> Result<(), PolicyError> {
    if action.len() != mean.len() {
        return Err(PolicyError::DimensionMismatch {
            expected: mean.len(),
            actual: action.len(),
        });
    }
    if action.len() != std.len() {
        return Err(PolicyError::DimensionMismatch {
            expected: std.len(),
            actual: action.len(),
        });
    }
    if action.len() != out.len() {
        return Err(PolicyError::DimensionMismatch {
            expected: action.len(),
            actual: out.len(),
        });
    }
    for i in 0..action.len() {
        let a = action[i];
        let m = mean[i];
        let s = std[i];
        if !a.is_finite() || !m.is_finite() || !s.is_finite() {
            return Err(PolicyError::NonFinite(format!(
                "non-finite input to denormalize_action at index {i}"
            )));
        }
        if s <= 0.0 {
            return Err(PolicyError::InvalidUnits(format!(
                "denormalize_action std[{i}] must be > 0.0, got {s}"
            )));
        }
        let y = a * s + m;
        if !y.is_finite() {
            return Err(PolicyError::NonFinite(format!(
                "denormalize_action produced non-finite output at index {i}"
            )));
        }
        out[i] = y;
    }
    Ok(())
}

/// Denormalize action: `action[i] * std[i] + mean[i]`.
pub fn denormalize_action(
    action: &[f32],
    mean: &[f32],
    std: &[f32],
) -> Result<Vec<f32>, PolicyError> {
    let mut out = vec![0.0f32; action.len()];
    denormalize_action_into(action, mean, std, &mut out)?;
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lim(name: &str, min_pos: f32, max_pos: f32) -> JointLimit {
        JointLimit::new(name, min_pos, max_pos, 1.0, 1.0)
    }

    #[test]
    fn rad_mode_is_passthrough_and_rejects_non_finite() {
        let limits = vec![
            lim("j1", -1.0, 1.0),
            lim("j2", -1.0, 1.0),
            lim("j3", -1.0, 1.0),
            lim("j4", -1.0, 1.0),
            lim("j5", -1.0, 1.0),
            lim("j6", -1.0, 1.0),
        ];
        let input = [0.1f32, -0.2, 0.3, -0.4, 0.5, -0.6];
        let out = rad_to_policy_units(&input, UnitMode::Rad, &limits, Some(5), None).unwrap();
        assert_eq!(out, input);
        let back = policy_units_to_rad(&out, UnitMode::Rad, &limits, Some(5), None).unwrap();
        assert_eq!(back, input);

        assert!(matches!(
            rad_to_policy_units(&[f32::NAN, 0.0, 0.0, 0.0, 0.0, 0.0], UnitMode::Rad, &limits, None, None),
            Err(PolicyError::NonFinite(_))
        ));
        assert!(matches!(
            policy_units_to_rad(&[f32::INFINITY, 0.0, 0.0, 0.0, 0.0, 0.0], UnitMode::Rad, &limits, None, None),
            Err(PolicyError::NonFinite(_))
        ));
    }

    #[test]
    fn so101_norm_6dof_matches_python_golden_vectors() {
        let limits = vec![
            lim("shoulder_pan", -1.9, 1.9),
            lim("shoulder_lift", -1.75, 1.75),
            lim("elbow_flex", -1.69, 1.69),
            lim("wrist_flex", -1.65, 1.65),
            lim("wrist_roll", -2.74, 2.74),
            lim("gripper", -0.17, 1.75),
        ];
        let vals = [0.0f32, 0.875, -0.845, 1.65, -2.74, 0.79];
        let expected_norm = [0.0f32, 50.0, -50.0, 100.0, -100.0, 50.0];
        let norm = rad_to_policy_units(&vals, UnitMode::So101Norm, &limits, Some(5), Some((-0.17, 1.75))).unwrap();
        for i in 0..6 {
            assert!((norm[i] - expected_norm[i]).abs() < 1e-4, "norm[{i}]={}", norm[i]);
        }
        let back = policy_units_to_rad(&norm, UnitMode::So101Norm, &limits, Some(5), Some((-0.17, 1.75))).unwrap();
        for i in 0..6 {
            assert!((back[i] - vals[i]).abs() < 1e-5, "back[{i}]={}", back[i]);
        }
    }

    #[test]
    fn so101_norm_7dof_with_reversed_gripper_matches_python_golden_vectors() {
        let limits = vec![
            lim("j1", -2.6, 2.6),
            lim("j2", -1.5, 1.5),
            lim("j3", -2.0, 2.0),
            lim("j4", -2.5, 2.5),
            lim("j5", -1.8, 1.8),
            lim("j6", -3.14, 3.14),
            lim("j7", -5.7, 0.0),
        ];
        let vals = [1.3f32, -0.75, 0.0, 2.5, -1.8, 1.57, -2.85];
        let expected_norm = [50.0f32, -50.0, 0.0, 100.0, -100.0, 50.0, 50.0];
        let norm = rad_to_policy_units(&vals, UnitMode::So101Norm, &limits, Some(6), Some((0.0, -5.7))).unwrap();
        for i in 0..7 {
            assert!((norm[i] - expected_norm[i]).abs() < 1e-4, "norm[{i}]={}", norm[i]);
        }
        // Also verify inferred gripper bounds when None matches explicit (0.0, -5.7)
        let norm_inferred = rad_to_policy_units(&vals, UnitMode::So101Norm, &limits, Some(6), None).unwrap();
        assert_eq!(norm, norm_inferred);

        // Verify endpoints: 0.0 rad -> 0.0 (closed), -5.7 rad -> 100.0 (open)
        let closed_norm = rad_to_policy_units(
            &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            UnitMode::So101Norm,
            &limits,
            Some(6),
            Some((0.0, -5.7)),
        ).unwrap();
        assert!((closed_norm[6] - 0.0).abs() < 1e-5);
        let open_norm = rad_to_policy_units(
            &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -5.7],
            UnitMode::So101Norm,
            &limits,
            Some(6),
            Some((0.0, -5.7)),
        ).unwrap();
        assert!((open_norm[6] - 100.0).abs() < 1e-5);

        let back = policy_units_to_rad(&norm, UnitMode::So101Norm, &limits, Some(6), Some((0.0, -5.7))).unwrap();
        for i in 0..7 {
            assert!((back[i] - vals[i]).abs() < 1e-5, "back[{i}]={}", back[i]);
        }
    }

    #[test]
    fn normalization_and_denormalization_round_trip_and_reject_invalid() {
        let state = [1.0f32, -2.0, 3.0];
        let mean = [0.5f32, -1.0, 1.5];
        let std = [0.5f32, 2.0, 3.0];
        let norm = normalize_state(&state, &mean, &std).unwrap();
        assert!((norm[0] - 1.0).abs() < 1e-6);
        assert!((norm[1] - (-0.5)).abs() < 1e-6);
        assert!((norm[2] - 0.5).abs() < 1e-6);

        let denorm = denormalize_action(&norm, &mean, &std).unwrap();
        for i in 0..3 {
            assert!((denorm[i] - state[i]).abs() < 1e-6);
        }

        assert!(normalize_state(&[f32::NAN, 0.0, 0.0], &mean, &std).is_err());
        assert!(normalize_state(&state, &mean, &[1.0, 0.0, 1.0]).is_err());
        assert!(denormalize_action(&[f32::INFINITY, 0.0, 0.0], &mean, &std).is_err());
    }

    #[test]
    fn joint_permutation_reorders_and_rejects_missing_or_duplicate() {
        let arm_limits = vec![
            lim("j1", -1.0, 1.0),
            lim("j2", -2.0, 2.0),
            lim("j3", -3.0, 3.0),
        ];
        let policy_names = vec!["j3".to_string(), "j1".to_string(), "j2".to_string()];
        let perm = JointPermutation::build(&policy_names, &arm_limits).unwrap();
        assert!(!perm.is_identity());
        assert_eq!(perm.arm_to_policy, vec![2, 0, 1]);
        assert_eq!(perm.policy_to_arm, vec![1, 2, 0]);

        let arm_vals = [10.0f32, 20.0, 30.0];
        let mut policy_vals = [0.0f32; 3];
        perm.reorder_arm_to_policy(&arm_vals, &mut policy_vals).unwrap();
        assert_eq!(policy_vals, [30.0, 10.0, 20.0]);

        let mut back = [0.0f32; 3];
        perm.reorder_policy_to_arm(&policy_vals, &mut back).unwrap();
        assert_eq!(back, arm_vals);

        let missing = vec!["j3".to_string(), "j1".to_string(), "unknown".to_string()];
        assert!(JointPermutation::build(&missing, &arm_limits).is_err());

        let dup = vec!["j1".to_string(), "j1".to_string(), "j2".to_string()];
        assert!(JointPermutation::build(&dup, &arm_limits).is_err());
    }
}
