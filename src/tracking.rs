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

/// Runtime output history for joint tracking compensation.
///
/// Seeded from fresh measurements on torque enable; invalidated on partial
/// writes, output suspension, safety holds/faults, or mode changes.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Default)]
pub struct TrackingRuntime {
    pub last_issued_positions: Option<Vec<f32>>,
    pub last_issued_velocities: Option<Vec<f32>>,
    /// Persistent monotonic workspace escape baseline for issued tracking outputs,
    /// tagged with `workspace_revision` and committed only after a successful write.
    pub ws_min_depths: Option<(u64, crate::kinematics::SlotBuf)>,
}

#[cfg(not(target_arch = "wasm32"))]
impl TrackingRuntime {
    /// Seed output history from fresh measurements with zero velocity.
    pub fn seed(&mut self, positions: &[f32]) {
        if positions.iter().all(|q| q.is_finite()) {
            self.last_issued_positions = Some(positions.to_vec());
            self.last_issued_velocities = Some(vec![0.0; positions.len()]);
            self.ws_min_depths = None;
        } else {
            self.invalidate();
        }
    }

    /// Record a successfully issued combined output vector.
    pub fn record_issued(&mut self, positions: Vec<f32>, velocities: Vec<f32>) {
        self.last_issued_positions = Some(positions);
        self.last_issued_velocities = Some(velocities);
    }

    /// Record a successfully issued combined output vector and commit its workspace escape depths.
    pub fn record_issued_with_ws(
        &mut self,
        positions: Vec<f32>,
        velocities: Vec<f32>,
        ws_min_depths: Option<(u64, crate::kinematics::SlotBuf)>,
    ) {
        self.last_issued_positions = Some(positions);
        self.last_issued_velocities = Some(velocities);
        self.ws_min_depths = ws_min_depths;
    }

    /// Invalidate output history (partial writes, suspension, holds/faults, mode changes).
    pub fn invalidate(&mut self) {
        self.last_issued_positions = None;
        self.last_issued_velocities = None;
        self.ws_min_depths = None;
    }
}

/// Why no position-lead candidate passed combined-vector validation.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone)]
pub enum TrackingReject {
    Workspace(crate::safety::SafetyError),
    JointLimit(String),
}

/// Validated physical output for the current control tick.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone)]
pub struct TrackingOutput {
    pub positions: Vec<f32>,
    pub velocities: Vec<f32>,
    pub multiplier: f32,
    pub sweep_evals_used: usize,
    pub ws_min_depths: Option<(u64, crate::kinematics::SlotBuf)>,
}

#[cfg(not(target_arch = "wasm32"))]
impl crate::supervisor::SupervisorInner {
    /// Applicable per-joint output limits for the active motion (planned speed cap,
    /// servo/policy speed cap, and power-governor time scaling).
    pub fn active_tracking_limits(&self, dt: f64, time_scale: f64) -> Vec<crate::model::JointLimit> {
        let gripper_idx = self.config.gripper_index();
        let mut limits = if self.servo_target.is_some() || self.servo_stream_active {
            self.servo_limits()
        } else {
            self.planning_limits()
        };
        if time_scale.is_finite() && time_scale < 1.0 {
            let scale = time_scale.clamp(0.0, 1.0) as f32;
            let dt_f32 = dt.clamp(0.001, 0.1) as f32;
            for (i, l) in limits.iter_mut().enumerate() {
                if Some(i) == gripper_idx {
                    continue;
                }
                let brake = self.commanded_velocities.get(i).map_or(0.0, |v| v.abs())
                    - 0.9 * l.max_acceleration * dt_f32;
                l.max_velocity = (l.max_velocity * scale).max(brake).max(1e-3);
            }
        }
        limits
    }

    /// Compute and validate the combined joint vector for Phase 1 position-lead tracking
    /// using the active motion's output limits.
    pub fn compute_position_lead_output(
        &self,
        dt: f64,
        jt: &JointTrackingConfig,
    ) -> Result<TrackingOutput, TrackingReject> {
        let limits = self.active_tracking_limits(dt, self.motion_time_scale());
        self.compute_position_lead_output_with_limits(dt, jt, &limits)
    }

    /// Compute and validate the combined joint vector for Phase 1 position-lead tracking
    /// against explicit `active_limits`.
    ///
    /// Pure read-only evaluation (`&self`): never mutates live escape bookkeeping
    /// (`servo_ws_min_depths` or `tracking.ws_min_depths`) or nominal state during candidate checks.
    ///
    /// Segment contracts:
    /// - Prev-issued segment (`prev_q -> q_cand`): joint position limits, active motion velocity
    ///   and acceleration limits (planned/servo/policy/power-governor caps, no multiplicative
    ///   inflation), and workspace/keep-out sweep using revision-tagged persistent escape depths.
    /// - Measured segment (`q_m -> q_cand`, when feedback is fresh): swept workspace, keep-outs,
    ///   link capsules and self-collision along the physical chord from `q_m` to `q_cand`, plus
    ///   the streaming directional `servo_max_lead_rad` allowance (all multipliers including 0).
    ///   Step velocity/acceleration limits are NOT applied to `(q_cand - q_m) / dt` because
    ///   `q_cand - q_m` includes normal steady-state plant lag.
    pub fn compute_position_lead_output_with_limits(
        &self,
        dt: f64,
        jt: &JointTrackingConfig,
        active_limits: &[crate::model::JointLimit],
    ) -> Result<TrackingOutput, TrackingReject> {
        use std::time::Duration;
        use crate::chain::WorkspaceEvaluator;
        use crate::safety::SafetyError;
        use crate::supervisor::MAX_KEEPOUT_EVALS_PER_TICK;

        const LEAD_MULTIPLIERS: &[f32] = &[1.0, 0.5, 0.25, 0.125, 0.0];
        const ZERO_ONLY: &[f32] = &[0.0];

        let dof = self.config.dof;
        if !dt.is_finite() || dt <= 0.0
            || active_limits.len() != dof
            || self.commanded_positions.len() != dof
            || self.commanded_velocities.len() != dof
            || !self.commanded_positions.iter().all(|q| q.is_finite())
        {
            return Err(TrackingReject::JointLimit("invalid nominal state, limits, or dt for tracking".into()));
        }

        let arm_indices = self.config.arm_joint_indices();
        if arm_indices.len() != jt.joints.len() {
            return Err(TrackingReject::JointLimit(format!(
                "tracking joint count {} does not match arm joint count {}",
                jt.joints.len(),
                arm_indices.len()
            )));
        }

        let fresh_meas = self.hardware_feedback_fresh(Duration::from_millis(100))
            && self.measured_positions.len() == dof
            && self.measured_positions.iter().all(|q| q.is_finite());
        let in_streaming = self.servo_target.is_some() || self.servo_stream_active;
        let in_hold = self.collision.hold.is_some() || self.servo_blocked_hold;

        // 1. Per-joint base position lead (arm joints only; gripper is strictly bypassed).
        let mut base_lead = vec![0.0_f32; dof];
        for (k, &joint_idx) in arm_indices.iter().enumerate() {
            let jc = &jt.joints[k];
            if jc.strategy != TrackingStrategy::PositionLead || in_hold || !fresh_meas {
                continue;
            }
            let vd = self.commanded_velocities[joint_idx];
            if !vd.is_finite() || vd == 0.0 {
                continue;
            }
            let lim = &self.config.limits[joint_idx];
            let qd = self.commanded_positions[joint_idx];
            let raw_lead = (jc.lag_s * vd).clamp(-jc.max_lead_rad, jc.max_lead_rad);
            let q_clamped = (qd + raw_lead).clamp(lim.min_position, lim.max_position);
            let mut lead = q_clamped - qd;

            // Streaming: trim added lead to remaining directional allowance relative to q_m.
            if in_streaming {
                let stream_lead = self.config.servo_max_lead_rad;
                let meas = self.measured_positions[joint_idx];
                if lead > 0.0 {
                    let avail = (meas + stream_lead - qd).max(0.0);
                    lead = lead.min(avail);
                } else if lead < 0.0 {
                    let avail = (qd - (meas - stream_lead)).max(0.0);
                    lead = lead.max(-avail);
                }
            }

            // Torque hold: never use compensation to advance farther into a hold.
            if let Some(h) = self.torque_guard.hold {
                if h.joint == joint_idx && lead * h.sign > 0.0 {
                    lead = 0.0;
                }
            }

            base_lead[joint_idx] = lead;
        }

        let has_desired_lead = base_lead.iter().any(|&l| l != 0.0);

        // 2. Resolve output history; if absent and no lead requested, pass nominal through.
        let seeded_q: Vec<f32>;
        let (prev_q, prev_v): (&[f32], Option<&[f32]>) = match self
            .tracking
            .last_issued_positions
            .as_ref()
            .filter(|v| v.len() == dof)
        {
            Some(pq) => (
                pq.as_slice(),
                self.tracking
                    .last_issued_velocities
                    .as_ref()
                    .filter(|v| v.len() == dof)
                    .map(Vec::as_slice),
            ),
            None if !has_desired_lead => {
                return Ok(TrackingOutput {
                    positions: self.commanded_positions.clone(),
                    velocities: vec![0.0; dof],
                    multiplier: 0.0,
                    sweep_evals_used: 0,
                    ws_min_depths: None,
                });
            }
            None => {
                // `has_desired_lead` implies `fresh_meas` is true.
                seeded_q = (0..dof)
                    .map(|i| {
                        if arm_indices.contains(&i) {
                            self.measured_positions[i]
                        } else {
                            self.commanded_positions[i]
                        }
                    })
                    .collect();
                (seeded_q.as_slice(), None)
            }
        };

        // 3. Precompute workspace start clearances without mutating live escape state.
        let ws_ctx = if let Some((ws, cam)) = self.workspace_policy() {
            let eval = WorkspaceEvaluator::new(&self.config.arm_model, ws, cam);
            let mut c_prev = eval.alloc_buf();
            eval.eval_into(prev_q, &mut c_prev, "tracking_lead")
                .map_err(|f| TrackingReject::Workspace(SafetyError::from_workspace(f)))?;
            let min_d_prev = match &self.tracking.ws_min_depths {
                Some((rev, min_d))
                    if *rev == self.workspace_revision && min_d.as_slice().len() == eval.total_slots =>
                {
                    min_d.clone()
                }
                _ => eval.init_min_depths(&c_prev),
            };
            let meas_ctx = if fresh_meas {
                let mut c_meas = eval.alloc_buf();
                eval.eval_into(&self.measured_positions, &mut c_meas, "tracking_lead")
                    .map_err(|f| TrackingReject::Workspace(SafetyError::from_workspace(f)))?;
                let min_d_meas = eval.init_min_depths(&c_meas);
                Some((c_meas, min_d_meas))
            } else {
                None
            };
            Some((eval, c_prev, min_d_prev, meas_ctx))
        } else {
            None
        };

        // 4. Try multipliers 1, 1/2, 1/4, 1/8, 0 in order on the combined joint vector,
        // sharing one per-tick workspace subdivision budget across all attempts and both segments.
        let multipliers = if has_desired_lead { LEAD_MULTIPLIERS } else { ZERO_ONLY };
        let mut last_err: Option<TrackingReject> = None;
        let mut budget = MAX_KEEPOUT_EVALS_PER_TICK;

        'cand: for &m in multipliers {
            let mut q_cand = self.commanded_positions.clone();
            if m != 0.0 {
                for &j in &arm_indices {
                    let l = base_lead[j];
                    if l != 0.0 {
                        let lim = &self.config.limits[j];
                        q_cand[j] = (q_cand[j] + m * l).clamp(lim.min_position, lim.max_position);
                    }
                }
            }

            // Per-joint position, directional allowance, velocity, and acceleration checks.
            for &j in &arm_indices {
                let lim = &active_limits[j];
                let q = q_cand[j];
                if !q.is_finite() || q < lim.min_position - 1e-5 || q > lim.max_position + 1e-5 {
                    last_err = Some(TrackingReject::JointLimit(format!(
                        "joint {} candidate {q:.4} rad outside [{:.4}, {:.4}]",
                        j + 1,
                        lim.min_position,
                        lim.max_position
                    )));
                    continue 'cand;
                }
                if in_streaming && fresh_meas {
                    let stream_lead = self.config.servo_max_lead_rad;
                    let meas = self.measured_positions[j];
                    let vd = self.commanded_velocities[j];
                    let advancing_pos = q > prev_q[j] + 1e-6 || vd > 1e-6;
                    let advancing_neg = q < prev_q[j] - 1e-6 || vd < -1e-6;
                    if (advancing_pos && q > meas + stream_lead + 1e-5)
                        || (advancing_neg && q < meas - stream_lead - 1e-5)
                    {
                        last_err = Some(TrackingReject::JointLimit(format!(
                            "joint {} candidate {q:.4} rad exceeds streaming measured-lead allowance around {meas:.4}",
                            j + 1
                        )));
                        continue 'cand;
                    }
                }
                if let Some(h) = self.torque_guard.hold {
                    if h.joint == j {
                        let qd = self.commanded_positions[j];
                        if (q - qd) * h.sign > 1e-6
                            || ((qd - h.hold_q) * h.sign <= 1e-6 && (q - h.hold_q) * h.sign > 1e-5)
                        {
                            last_err = Some(TrackingReject::JointLimit(format!(
                                "joint {} candidate {q:.4} rad advances into torque hold at {:.4}",
                                j + 1,
                                h.hold_q
                            )));
                            continue 'cand;
                        }
                    }
                }
                let v_cand = (f64::from(q) - f64::from(prev_q[j])) / dt;
                let v_prev_j = prev_v.map_or(0.0, |pv| f64::from(pv[j]));
                let a_cand = (v_cand - v_prev_j) / dt;
                let v_lim = f64::from(lim.max_velocity) + 1e-4;
                let a_lim = f64::from(lim.max_acceleration) + 1e-3;
                if !v_cand.is_finite() || v_cand.abs() > v_lim {
                    last_err = Some(TrackingReject::JointLimit(format!(
                        "joint {} output velocity {v_cand:.3} rad/s exceeds limit {:.3}",
                        j + 1,
                        lim.max_velocity
                    )));
                    continue 'cand;
                }
                if !a_cand.is_finite() || a_cand.abs() > a_lim {
                    last_err = Some(TrackingReject::JointLimit(format!(
                        "joint {} output acceleration {a_cand:.3} rad/s^2 exceeds limit {:.3}",
                        j + 1,
                        lim.max_acceleration
                    )));
                    continue 'cand;
                }
            }

            // Combined-vector workspace and swept keep-out checks on both segments.
            let mut committed_ws_min_d = None;
            if let Some((ref eval, ref c_prev, ref min_d_prev, ref meas_ctx)) = ws_ctx {
                let mut local_min_d = min_d_prev.clone();
                if let Err((_, f)) = eval.sweep_linear_step(
                    prev_q,
                    &q_cand,
                    c_prev,
                    &mut local_min_d,
                    &mut budget,
                    MAX_KEEPOUT_EVALS_PER_TICK,
                    "tracking_lead",
                    "tracking_lead",
                ) {
                    last_err = Some(TrackingReject::Workspace(SafetyError::from_workspace(f)));
                    continue 'cand;
                }
                if let Some((ref c_meas, ref min_d_meas)) = meas_ctx {
                    let mut local_meas_min_d = min_d_meas.clone();
                    if let Err((_, f)) = eval.sweep_linear_step(
                        &self.measured_positions,
                        &q_cand,
                        c_meas,
                        &mut local_meas_min_d,
                        &mut budget,
                        MAX_KEEPOUT_EVALS_PER_TICK,
                        "tracking_lead",
                        "tracking_lead",
                    ) {
                        last_err = Some(TrackingReject::Workspace(SafetyError::from_workspace(f)));
                        continue 'cand;
                    }
                }
                committed_ws_min_d = Some((self.workspace_revision, local_min_d));
            }

            let v_out: Vec<f32> = (0..dof)
                .map(|i| ((f64::from(q_cand[i]) - f64::from(prev_q[i])) / dt) as f32)
                .collect();
            return Ok(TrackingOutput {
                positions: q_cand,
                velocities: v_out,
                multiplier: m,
                sweep_evals_used: MAX_KEEPOUT_EVALS_PER_TICK - budget,
                ws_min_depths: committed_ws_min_d,
            });
        }

        Err(last_err.unwrap_or_else(|| TrackingReject::JointLimit("no valid tracking candidate".into())))
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
