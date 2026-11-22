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
/// Measured arm-joint speed at or below which velocity feedback is evidence of rest (rad/s).
pub const STATIONARY_VELOCITY_RAD_S: f32 = 0.05;
/// Largest arm-joint position change between two fresh samples at least one control tick
/// apart that still counts as evidence of rest without velocity feedback (rad;
/// 0.05 rad/s over a 20 ms tick).
pub const STATIONARY_POSITION_TOL_RAD: f32 = 1.0e-3;
/// Oldest a reference rest sample may be (its oldest joint reply, against the current
/// sample's newest reply) when it is kept and when it is used as evidence.
pub const STATIONARY_SAMPLE_MAX_AGE: std::time::Duration = std::time::Duration::from_millis(100);

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

/// `ArmConfig.joint_tracking` from a profile. Before joint tracking existed the field was
/// ignored, so only a JSON object whose `"enabled"` is the literal `true` is parsed (strictly:
/// `deny_unknown_fields`, known strategies, map form). Anything else (an object without that
/// key or with another value, an array, a scalar, numbers serde_json cannot represent) never
/// rejects the profile: a valid disabled object is kept, everything else is logged and
/// dropped (None). The value is captured unparsed (`RawValue`) so that skipping it cannot fail.
pub fn deserialize_profile_block<'de, D>(deserializer: D) -> Result<Option<JointTrackingConfig>, D::Error>
where
    D: serde::Deserializer<'de>,
{
    let Some(raw) = Option::<Box<serde_json::value::RawValue>>::deserialize(deserializer)? else {
        return Ok(None);
    };
    let text = raw.get();
    if text.trim() == "null" {
        return Ok(None);
    }
    match serde_json::from_str::<EnabledProbe>(text) {
        Ok(EnabledProbe(true)) => serde_json::from_str::<JointTrackingConfig>(text)
            .map(Some)
            .map_err(|e| serde::de::Error::custom(format!("joint_tracking: {e}"))),
        Ok(EnabledProbe(false)) => match serde_json::from_str::<JointTrackingConfig>(text) {
            // A map without a literal `"enabled": true` deserializes with enabled = false.
            Ok(cfg) if !cfg.enabled => Ok(Some(cfg)),
            Ok(_) => Ok(None),
            Err(e) => {
                tracing::warn!("joint_tracking is not enabled and does not parse (ignored): {e}");
                Ok(None)
            }
        },
        Err(e) => {
            tracing::warn!("joint_tracking is not an object with \"enabled\": true (ignored): {e}");
            Ok(None)
        }
    }
}

/// Whether a JSON value is an object holding the key `"enabled"` with the literal `true`.
/// Fails for anything that is not an object or whose `"enabled"` is not a boolean; every
/// other member is skipped without being parsed.
struct EnabledProbe(bool);

impl<'de> Deserialize<'de> for EnabledProbe {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        struct V;
        impl<'de> serde::de::Visitor<'de> for V {
            type Value = EnabledProbe;
            fn expecting(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
                f.write_str("an object")
            }
            fn visit_map<A: serde::de::MapAccess<'de>>(self, mut map: A) -> Result<EnabledProbe, A::Error> {
                let mut enabled = false;
                while let Some(key) = map.next_key::<String>()? {
                    if key == "enabled" {
                        enabled |= map.next_value::<bool>()?;
                    } else {
                        map.next_value::<serde::de::IgnoredAny>()?;
                    }
                }
                Ok(EnabledProbe(enabled))
            }
        }
        deserializer.deserialize_map(V)
    }
}

/// Runtime output history for joint tracking compensation.
///
/// Seeded from fresh measurements on torque enable; invalidated on partial
/// writes, output suspension, safety holds/faults, or mode changes.
/// Explicit state of tracking output history so unknown baselines never bypass validation.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum TrackingHistoryState {
    /// No confirmed output baseline (startup, partial write, bus suspension, recovery).
    #[default]
    Unknown,
    /// Confirmed stationary baseline seeded from fresh measurements with zero velocity.
    Stationary,
    /// Last successfully issued combined output vector and its discrete velocity.
    Issued,
}

#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Default)]
pub struct TrackingRuntime {
    pub history_state: TrackingHistoryState,
    pub last_issued_positions: Option<Vec<f32>>,
    pub last_issued_velocities: Option<Vec<f32>>,
    /// Persistent monotonic workspace escape baseline for issued tracking outputs,
    /// tagged with `workspace_revision` and committed only after a successful write.
    pub ws_min_depths: Option<(u64, crate::kinematics::SlotBuf)>,
    /// Persistent monotonic keep-out-only escape baseline used during park (`NormalUnload`)
    /// residual lead ramp-out, tagged with `workspace_revision`.
    pub ws_keepout_min_depths: Option<(u64, crate::kinematics::SlotBuf)>,
    /// True while compensated output is suspended following an autonomous driver
    /// recovery write (e.g. Damiao 45 ms POS_VEL hold) until fresh post-recovery
    /// feedback confirms every joint's baseline.
    pub recovery_suspended: bool,
    /// Per-joint instant of the latest autonomous driver recovery write awaiting
    /// fresh post-recovery feedback.
    pub recovery_joints: Vec<(usize, std::time::Instant)>,
    /// Total workspace sweep evaluations consumed on the most recent tick across
    /// primary evaluation and any rejection fallback.
    pub last_tick_sweep_evals: usize,
    /// Rejection-fallback braking ramp in progress: the issued output keeps stepping
    /// toward this hold pose within the active limits, one tick at a time, until it
    /// is at rest (also while the rejection fault is latched).
    pub fallback_ramp: Option<FallbackRamp>,
    /// Reference rest sample: a fresh measured arm position of an earlier control tick, with
    /// the receive time of its newest joint sample, kept while the arm stays within
    /// tolerance of it (`SupervisorInner::note_tracking_position_sample`): stationarity
    /// evidence for drivers without velocity feedback.
    /// Tuple: (oldest joint reply, newest joint reply, positions).
    pub position_sample: Option<(std::time::Instant, std::time::Instant, Vec<f32>)>,
    /// Motion caps (`ActiveTrajectory::motion_limits`) of a planned motion whose reference
    /// finished, kept in force until it has settled and its residual lead is ramped out.
    pub retained_motion_limits: Option<Vec<crate::model::JointLimit>>,
}

/// Rejection-fallback braking ramp state (`TrackingRuntime::fallback_ramp`).
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone)]
pub struct FallbackRamp {
    /// Hold pose the ramp brakes toward (measured pose at the rejection, jaw command kept).
    pub hold: Vec<f32>,
    /// The rejection happened during park (`NormalUnload`): validate against the park
    /// keep-out-only workspace and its escape depths.
    pub keepout_only: bool,
    /// Output limits in force when the motion was rejected (its speed_scale / servo caps);
    /// every braking tick uses them, also after the hold has cleared the motion.
    pub limits: Vec<crate::model::JointLimit>,
}

/// A fallback step evaluated before any hold or fault changes the tracking state, with the
/// state it was evaluated from.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone)]
pub struct PendingFallback {
    pub step: Result<ValidatedFallback, String>,
    pub ramp: FallbackRamp,
    pub prev_confirmed: Option<Vec<f32>>,
    /// Escape depths of the other workspace (full when `keepout_only`, park otherwise),
    /// kept unchanged when the step is recorded.
    pub other_depths: Option<(u64, crate::kinematics::SlotBuf)>,
}

#[cfg(not(target_arch = "wasm32"))]
impl TrackingRuntime {
    /// Seed output history from fresh measurements as a confirmed stationary baseline.
    pub fn seed(&mut self, positions: &[f32]) {
        if positions.iter().all(|q| q.is_finite()) {
            self.history_state = TrackingHistoryState::Stationary;
            self.last_issued_positions = Some(positions.to_vec());
            self.last_issued_velocities = Some(vec![0.0; positions.len()]);
            self.ws_min_depths = None;
            self.ws_keepout_min_depths = None;
            self.recovery_suspended = false;
            self.recovery_joints.clear();
            self.fallback_ramp = None;
        } else {
            self.invalidate();
        }
    }

    /// True when the last issued output differs from nominal `nominal` on any arm joint.
    pub fn has_residual_lead(&self, nominal: &[f32], arm_indices: &[usize]) -> bool {
        let Some(prev) = self.last_issued_positions.as_deref() else {
            return false;
        };
        arm_indices.iter().any(|&j| {
            prev.get(j)
                .zip(nominal.get(j))
                .is_some_and(|(p, q)| (p - q).abs() > 1e-5)
        })
    }

    /// Suspend compensated writes after autonomous driver recovery writes until
    /// confirmed fresh post-recovery feedback establishes a clean baseline for
    /// the recovered joint(s). Preserves unaffected joints' issued history so they
    /// never suffer an unchecked step on resume.
    pub fn suspend_for_recovery(&mut self, joints: &[usize], at: std::time::Instant) {
        self.ws_min_depths = None;
        self.ws_keepout_min_depths = None;
        self.recovery_suspended = true;
        for &j in joints {
            if let Some(entry) = self.recovery_joints.iter_mut().find(|(idx, _)| *idx == j) {
                entry.1 = at;
            } else {
                self.recovery_joints.push((j, at));
            }
        }
    }

    /// Resume after post-recovery feedback confirms the recovered joint(s).
    /// Updates issued history ONLY for `recovered_positions`, preserving unaffected
    /// joints' last issued targets (or keeping `Unknown` if no prior history existed).
    pub fn resume_from_recovery(&mut self, recovered_positions: &[(usize, f32)]) {
        self.recovery_suspended = false;
        self.recovery_joints.clear();
        self.ws_min_depths = None;
        self.ws_keepout_min_depths = None;
        if let Some(prev_q) = self.last_issued_positions.as_mut() {
            let dof = prev_q.len();
            let prev_v = self
                .last_issued_velocities
                .get_or_insert_with(|| vec![0.0; dof]);
            // Only the recovered joints were reseeded at rest by the driver's hold; every
            // other joint keeps its real issued velocity for the next acceleration check.
            for &(j, q) in recovered_positions {
                if j < dof && q.is_finite() {
                    prev_q[j] = q;
                    if let Some(v) = prev_v.get_mut(j) {
                        *v = 0.0;
                    }
                }
            }
        } else {
            self.history_state = TrackingHistoryState::Unknown;
            self.last_issued_velocities = None;
        }
    }

    /// Clear recovery suspension once a confirmed fresh baseline is established.
    pub fn clear_recovery_suspension(&mut self) {
        self.recovery_suspended = false;
        self.recovery_joints.clear();
    }

    /// Record a successfully issued combined output vector.
    pub fn record_issued(&mut self, positions: Vec<f32>, velocities: Vec<f32>) {
        self.history_state = TrackingHistoryState::Issued;
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
        self.record_issued_with_ws_depths(positions, velocities, ws_min_depths, None);
    }

    /// Record a successfully issued output vector and commit both full-workspace and
    /// park keep-out-only escape depths.
    pub fn record_issued_with_ws_depths(
        &mut self,
        positions: Vec<f32>,
        velocities: Vec<f32>,
        ws_min_depths: Option<(u64, crate::kinematics::SlotBuf)>,
        ws_keepout_min_depths: Option<(u64, crate::kinematics::SlotBuf)>,
    ) {
        self.history_state = TrackingHistoryState::Issued;
        self.last_issued_positions = Some(positions);
        self.last_issued_velocities = Some(velocities);
        self.ws_min_depths = ws_min_depths;
        self.ws_keepout_min_depths = ws_keepout_min_depths;
    }

    /// Invalidate output history (partial writes, suspension, holds/faults, mode changes).
    pub fn invalidate(&mut self) {
        self.history_state = TrackingHistoryState::Unknown;
        self.last_issued_positions = None;
        self.last_issued_velocities = None;
        self.ws_min_depths = None;
        self.ws_keepout_min_depths = None;
        self.fallback_ramp = None;
    }
}

/// Why no position-lead candidate passed combined-vector validation.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone)]
pub enum TrackingReject {
    Workspace(crate::safety::SafetyError),
    JointLimit(String),
}

/// Validated physical output for a rejection fallback hold or bounded ramp step.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone)]
pub struct ValidatedFallback {
    pub positions: Vec<f32>,
    pub velocities: Vec<f32>,
    pub ws_min_depths: Option<(u64, crate::kinematics::SlotBuf)>,
    /// True when this step leaves every arm joint at rest (the braking ramp is complete).
    pub done: bool,
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
    pub ws_keepout_min_depths: Option<(u64, crate::kinematics::SlotBuf)>,
}

#[cfg(not(target_arch = "wasm32"))]
impl crate::supervisor::SupervisorInner {
    /// Receive times of the oldest and newest joint feedback sample, when every joint has one
    /// (`ArmDriver::feedback_instants`: the driver's own receive instants, not now - age).
    fn feedback_instant_range(&self) -> Option<(std::time::Instant, std::time::Instant)> {
        let instants = self.driver.as_ref()?.lock().ok()?.feedback_instants();
        if instants.len() != self.config.dof {
            return None;
        }
        let instants: Vec<std::time::Instant> = instants.into_iter().collect::<Option<Vec<_>>>()?;
        Some((*instants.iter().min()?, *instants.iter().max()?))
    }

    /// Positive evidence that the arm joints are at rest, from fresh feedback: measured
    /// velocities within [`STATIONARY_VELOCITY_RAD_S`] when the driver reports velocity,
    /// otherwise two position samples within [`STATIONARY_POSITION_TOL_RAD`] whose feedback
    /// timestamps lie at least one control period (`1 / control_frequency_hz`) apart: the
    /// oldest joint sample of the second is that much newer than the newest of the first.
    pub fn tracking_stationary_evidence(&self) -> bool {
        let dof = self.config.dof;
        if !self.hardware_feedback_fresh(std::time::Duration::from_millis(100))
            || self.measured_positions.len() != dof
            || !self.measured_positions.iter().all(|q| q.is_finite())
        {
            return false;
        }
        let arm = self.config.arm_joint_indices();
        let velocity_feedback = self
            .driver
            .as_ref()
            .is_some_and(|d| d.lock().is_ok_and(|drv| drv.capabilities().velocity_feedback));
        if velocity_feedback {
            return arm.iter().all(|&j| {
                self.measured_velocities
                    .get(j)
                    .is_some_and(|v| v.is_finite() && v.abs() <= STATIONARY_VELOCITY_RAD_S)
            });
        }
        let (Some((sampled_oldest, sampled_newest, q_prev)), Some((oldest_now, newest_now))) =
            (self.tracking.position_sample.as_ref(), self.feedback_instant_range())
        else {
            return false;
        };
        // At least one control period between the samples by their feedback timestamps, and
        // every reply the decision uses within STATIONARY_SAMPLE_MAX_AGE: from the reference's
        // oldest reply to this sample's newest (also after a control-loop pause).
        let period = std::time::Duration::from_secs_f64(1.0 / f64::from(self.config.control_frequency_hz.max(1)));
        oldest_now.checked_duration_since(*sampled_newest).is_some_and(|gap| gap >= period)
            && newest_now.checked_duration_since(*sampled_oldest).is_some_and(|age| age <= STATIONARY_SAMPLE_MAX_AGE)
            && q_prev.len() == dof
            && arm.iter().all(|&j| (self.measured_positions[j] - q_prev[j]).abs() <= STATIONARY_POSITION_TOL_RAD)
    }

    /// Maintain the reference rest sample: keep it while it is at most
    /// [`STATIONARY_SAMPLE_MAX_AGE`] old and this tick's fresh measurement stays within
    /// [`STATIONARY_POSITION_TOL_RAD`] of it; otherwise replace it with this tick's sample
    /// (cleared when feedback is not fresh). Keeping it lets a later sample lie a full
    /// control period after it even when consecutive samples overlap (staggered replies).
    pub fn note_tracking_position_sample(&mut self) {
        let dof = self.config.dof;
        let fresh = self.hardware_feedback_fresh(std::time::Duration::from_millis(100))
            && self.measured_positions.len() == dof
            && self.measured_positions.iter().all(|q| q.is_finite());
        if !fresh {
            self.tracking.position_sample = None;
            return;
        }
        let arm = self.config.arm_joint_indices();
        let range = self.feedback_instant_range();
        let keep = self.tracking.position_sample.as_ref().is_some_and(|(oldest, newest, q_ref)| {
            // Age of the reference (its oldest reply) against this sample's newest reply
            // (driver clock); a reference newer than this sample is replaced.
            range.is_some_and(|(_, newest_now)| {
                newest_now.checked_duration_since(*newest).is_some()
                    && newest_now.checked_duration_since(*oldest).is_some_and(|age| age <= STATIONARY_SAMPLE_MAX_AGE)
            })
                && q_ref.len() == dof
                && arm.iter().all(|&j| (self.measured_positions[j] - q_ref[j]).abs() <= STATIONARY_POSITION_TOL_RAD)
        });
        if !keep {
            self.tracking.position_sample =
                range.map(|(oldest, newest)| (oldest, newest, self.measured_positions.clone()));
        }
    }

    /// Applicable per-joint output limits for the active motion (planned speed cap,
    /// servo/policy speed cap, and power-governor time scaling). A finished planned motion's
    /// caps apply until it has settled (`TrackingRuntime::retained_motion_limits`).
    pub fn active_tracking_limits(&self, dt: f64, time_scale: f64) -> Vec<crate::model::JointLimit> {
        let gripper_idx = self.config.gripper_index();
        let mut limits = if self.servo_target.is_some() || self.servo_stream_active {
            self.servo_limits()
        } else {
            let mut base = self.planning_limits();
            let caps = match self.active_trajectory.as_ref() {
                Some(a) => a.motion_limits.as_ref(),
                None => self.tracking.retained_motion_limits.as_ref(),
            };
            if let Some(mlim) = caps.filter(|v| v.len() == base.len())
            {
                for (l, m) in base.iter_mut().zip(mlim.iter()) {
                    l.max_velocity = l.max_velocity.min(m.max_velocity);
                    l.max_acceleration = l.max_acceleration.min(m.max_acceleration);
                }
            }
            base
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
        let mut budget = crate::supervisor::MAX_KEEPOUT_EVALS_PER_TICK;
        self.compute_position_lead_output_with_limits_and_budget(dt, jt, active_limits, &mut budget)
    }

    pub fn compute_position_lead_output_with_limits_and_budget(
        &self,
        dt: f64,
        jt: &JointTrackingConfig,
        active_limits: &[crate::model::JointLimit],
        budget: &mut usize,
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

        // 2. Resolve output history; unknown history NEVER bypasses validation.
        let seeded_q: Vec<f32>;
        let (prev_q, prev_v): (&[f32], Option<&[f32]>) = match self.tracking.history_state {
            TrackingHistoryState::Stationary | TrackingHistoryState::Issued => {
                let Some(pq) = self.tracking.last_issued_positions.as_ref().filter(|v| v.len() == dof) else {
                    return Err(TrackingReject::JointLimit("tracking output history missing positions".into()));
                };
                (
                    pq.as_slice(),
                    self.tracking
                        .last_issued_velocities
                        .as_ref()
                        .filter(|v| v.len() == dof)
                        .map(Vec::as_slice),
                )
            }
            TrackingHistoryState::Unknown => {
                if !fresh_meas {
                    return Err(TrackingReject::JointLimit(
                        "tracking output history is unknown without fresh feedback".into(),
                    ));
                }
                // Seeding at the measurement with zero prior velocity needs positive
                // evidence that the arm is at rest.
                if !self.tracking_stationary_evidence() {
                    return Err(TrackingReject::JointLimit(
                        "tracking output history is unknown and the arm is not confirmed at rest".into(),
                    ));
                }
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
        let budget_start = *budget;

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
                    budget,
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
                        budget,
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
                sweep_evals_used: budget_start.saturating_sub(*budget),
                ws_min_depths: committed_ws_min_d,
                ws_keepout_min_depths: None,
            });
        }

        Err(last_err.unwrap_or_else(|| TrackingReject::JointLimit("no valid tracking candidate".into())))
    }

    /// Ramp residual tracking lead smoothly from `last_issued_positions` to nominal
    /// `commanded_positions` during an ownership handover (park, torque hold, or moving
    /// lease handover) so active lead is never dropped in a single 20 ms step.
    ///
    /// When `keepout_only` is true (park / `NormalUnload`), workspace validation uses
    /// `ws.keepout_only()`, preserving park's floor/outer-box/reach exemptions while
    /// still enforcing swept keep-out boxes.
    pub fn compute_lead_ramp_out(
        &self,
        dt: f64,
        active_limits: &[crate::model::JointLimit],
        keepout_only: bool,
    ) -> Result<TrackingOutput, TrackingReject> {
        let mut budget = crate::supervisor::MAX_KEEPOUT_EVALS_PER_TICK;
        self.compute_lead_ramp_out_with_budget(dt, active_limits, keepout_only, &mut budget)
    }

    pub fn compute_lead_ramp_out_with_budget(
        &self,
        dt: f64,
        active_limits: &[crate::model::JointLimit],
        keepout_only: bool,
        budget: &mut usize,
    ) -> Result<TrackingOutput, TrackingReject> {
        use std::time::Duration;
        use crate::chain::WorkspaceEvaluator;
        use crate::safety::SafetyError;
        use crate::supervisor::MAX_KEEPOUT_EVALS_PER_TICK;

        let dof = self.config.dof;
        if !dt.is_finite()
            || dt <= 0.0
            || active_limits.len() != dof
            || self.commanded_positions.len() != dof
            || !self.commanded_positions.iter().all(|q| q.is_finite())
        {
            return Err(TrackingReject::JointLimit("invalid nominal state or dt for lead ramp-out".into()));
        }

        let arm_indices = self.config.arm_joint_indices();
        let fresh_meas = self.hardware_feedback_fresh(Duration::from_millis(100))
            && self.measured_positions.len() == dof
            && self.measured_positions.iter().all(|q| q.is_finite());

        let Some(prev_q) = self.tracking.last_issued_positions.as_deref().filter(|v| v.len() == dof) else {
            return Err(TrackingReject::JointLimit("cannot ramp lead out without prior issued output".into()));
        };
        let prev_v = self.tracking.last_issued_velocities.as_deref().filter(|v| v.len() == dof);

        let mut q_next = self.commanded_positions.clone();
        for &j in &arm_indices {
            let lim = &active_limits[j];
            let pq = prev_q[j];
            let pv = prev_v.map_or(0.0, |v| v[j]);
            let qd = self.commanded_positions[j].clamp(lim.min_position, lim.max_position);
            let stepped = if (qd - pq).abs() <= 1e-6 && pv.abs() <= lim.max_acceleration * dt as f32 {
                qd
            } else {
                let Some((q_s, _, _)) = Self::bounded_servo_step(pq, qd, pv, dt, lim) else {
                    return Err(TrackingReject::JointLimit(format!(
                        "joint {} lead ramp-out infeasible from {pq:.4} toward {qd:.4}",
                        j + 1
                    )));
                };
                q_s
            };
            let mut q_final = stepped;
            if let Some(h) = self.torque_guard.hold {
                if h.joint == j {
                    // Never advance farther into the torque-held direction while ramping out.
                    if (q_final - pq) * h.sign > 1e-6 {
                        q_final = pq;
                    }
                    if (pq - h.hold_q) * h.sign <= 1e-6 && (q_final - h.hold_q) * h.sign > 1e-5 {
                        q_final = h.hold_q;
                    }
                    // Recheck both velocity and acceleration after torque-hold clamping.
                    let v_final = (f64::from(q_final) - f64::from(pq)) / dt;
                    let v_prev_j = prev_v.map_or(0.0, |pv| f64::from(pv[j]));
                    let a_final = (v_final - v_prev_j) / dt;
                    if !v_final.is_finite()
                        || v_final.abs() > f64::from(lim.max_velocity) + 1e-4
                        || !a_final.is_finite()
                        || a_final.abs() > f64::from(lim.max_acceleration) + 1e-3
                    {
                        return Err(TrackingReject::JointLimit(format!(
                            "joint {} torque-hold ramp-out dynamics exceeded (v={v_final:.3}/{:.3}, a={a_final:.3}/{:.3})",
                            j + 1,
                            lim.max_velocity,
                            lim.max_acceleration
                        )));
                    }
                }
            }
            let in_streaming = self.servo_target.is_some() || self.servo_stream_active;
            if in_streaming && fresh_meas {
                let stream_lead = self.config.servo_max_lead_rad;
                let meas = self.measured_positions[j];
                let advancing_pos = q_final > pq + 1e-6;
                let advancing_neg = q_final < pq - 1e-6;
                if (advancing_pos && q_final > meas + stream_lead + 1e-5)
                    || (advancing_neg && q_final < meas - stream_lead - 1e-5)
                {
                    return Err(TrackingReject::JointLimit(format!(
                        "joint {} ramp-out candidate {q_final:.4} rad exceeds streaming measured-lead allowance around {meas:.4}",
                        j + 1
                    )));
                }
            }
            q_next[j] = q_final;
        }

        let budget_start = *budget;
        let mut committed_ws_min_d = None;
        let mut committed_keepout_min_d = None;
        if let Some((ws, cam)) = self.workspace_policy() {
            let keepout_ws = keepout_only.then(|| ws.keepout_only());
            let ws_ref = keepout_ws.as_ref().unwrap_or(ws);
            let skip_ws = keepout_only && ws_ref.keepout_boxes.is_empty();
            if !skip_ws {
                let eval = WorkspaceEvaluator::new(&self.config.arm_model, ws_ref, cam);
                let mut c_prev = eval.alloc_buf();
                eval.eval_into(prev_q, &mut c_prev, "tracking_ramp_out")
                    .map_err(|f| TrackingReject::Workspace(SafetyError::from_workspace(f)))?;
                let cached_depths = if keepout_only {
                    &self.tracking.ws_keepout_min_depths
                } else {
                    &self.tracking.ws_min_depths
                };
                let mut local_min_d = match cached_depths {
                    Some((rev, md))
                        if *rev == self.workspace_revision
                            && md.as_slice().len() == eval.total_slots =>
                    {
                        md.clone()
                    }
                    _ => eval.init_min_depths(&c_prev),
                };
                if let Err((_, f)) = eval.sweep_linear_step(
                    prev_q,
                    &q_next,
                    &c_prev,
                    &mut local_min_d,
                    budget,
                    MAX_KEEPOUT_EVALS_PER_TICK,
                    "tracking_ramp_out",
                    "tracking_ramp_out",
                ) {
                    return Err(TrackingReject::Workspace(SafetyError::from_workspace(f)));
                }
                if fresh_meas {
                    let mut c_meas = eval.alloc_buf();
                    eval.eval_into(&self.measured_positions, &mut c_meas, "tracking_ramp_out")
                        .map_err(|f| TrackingReject::Workspace(SafetyError::from_workspace(f)))?;
                    let mut local_meas_min_d = eval.init_min_depths(&c_meas);
                    if let Err((_, f)) = eval.sweep_linear_step(
                        &self.measured_positions,
                        &q_next,
                        &c_meas,
                        &mut local_meas_min_d,
                        budget,
                        MAX_KEEPOUT_EVALS_PER_TICK,
                        "tracking_ramp_out",
                        "tracking_ramp_out",
                    ) {
                        return Err(TrackingReject::Workspace(SafetyError::from_workspace(f)));
                    }
                }
                if keepout_only {
                    committed_keepout_min_d = Some((self.workspace_revision, local_min_d));
                } else {
                    committed_ws_min_d = Some((self.workspace_revision, local_min_d));
                }
            }
        }

        let v_out: Vec<f32> = (0..dof)
            .map(|i| ((f64::from(q_next[i]) - f64::from(prev_q[i])) / dt) as f32)
            .collect();
        Ok(TrackingOutput {
            positions: q_next,
            velocities: v_out,
            multiplier: 0.0,
            sweep_evals_used: budget_start.saturating_sub(*budget),
            ws_min_depths: committed_ws_min_d,
            ws_keepout_min_depths: committed_keepout_min_d,
        })
    }

    /// Explicitly validate a proposed fallback hold pose `q_hold` against fresh feedback,
    /// joint position limits, and swept workspace/keep-out constraints from both
    /// `prev_confirmed` and `q_m`. Returns false when feedback is stale or any sweep fails.
    /// Explicitly validate a proposed fallback hold pose `q_hold` against fresh feedback,
    /// joint position limits, dynamics (velocity & acceleration from `prev_confirmed`),
    /// and swept workspace/keep-out constraints from both `prev_confirmed` and `q_m`.
    /// Returns `Some(updated_ws_min_depths)` when valid, or `None` otherwise.
    pub fn validate_fallback_hold_with_dynamics(
        &self,
        prev_confirmed: Option<&[f32]>,
        prev_velocities: Option<&[f32]>,
        q_hold: &[f32],
        dt: f64,
        active_limits: &[crate::model::JointLimit],
        fresh_meas: bool,
        budget: &mut usize,
    ) -> Option<Option<(u64, crate::kinematics::SlotBuf)>> {
        use crate::chain::WorkspaceEvaluator;
        use crate::supervisor::MAX_KEEPOUT_EVALS_PER_TICK;

        let dof = self.config.dof;
        if !fresh_meas
            || !dt.is_finite()
            || dt <= 0.0
            || active_limits.len() != dof
            || q_hold.len() != dof
            || !q_hold.iter().all(|q| q.is_finite())
        {
            return None;
        }
        let arm_indices = self.config.arm_joint_indices();
        let prev_q = prev_confirmed.filter(|v| v.len() == dof && v.iter().all(|q| q.is_finite()));
        let prev_v = prev_velocities.filter(|v| v.len() == dof && v.iter().all(|q| q.is_finite()));
        for &j in &arm_indices {
            let lim = &active_limits[j];
            if q_hold[j] < lim.min_position - 1e-5 || q_hold[j] > lim.max_position + 1e-5 {
                return None;
            }
            if let Some(pq) = prev_q {
                let v_cand = (f64::from(q_hold[j]) - f64::from(pq[j])) / dt;
                let v_prev_j = prev_v.map_or(0.0, |pv| f64::from(pv[j]));
                let a_cand = (v_cand - v_prev_j) / dt;
                if !v_cand.is_finite()
                    || v_cand.abs() > f64::from(lim.max_velocity) + 1e-4
                    || !a_cand.is_finite()
                    || a_cand.abs() > f64::from(lim.max_acceleration) + 1e-3
                {
                    return None;
                }
            }
        }
        let mut committed_ws = None;
        if let Some((ws, cam)) = self.workspace_policy() {
            let eval = WorkspaceEvaluator::new(&self.config.arm_model, ws, cam);
            if let Some(pq) = prev_q {
                let mut c_prev = eval.alloc_buf();
                eval.eval_into(pq, &mut c_prev, "tracking_fallback").ok()?;
                let mut min_d = match &self.tracking.ws_min_depths {
                    Some((rev, md)) if *rev == self.workspace_revision && md.as_slice().len() == eval.total_slots => {
                        md.clone()
                    }
                    _ => eval.init_min_depths(&c_prev),
                };
                eval.sweep_linear_step(
                    pq,
                    q_hold,
                    &c_prev,
                    &mut min_d,
                    budget,
                    MAX_KEEPOUT_EVALS_PER_TICK,
                    "tracking_fallback",
                    "tracking_fallback",
                )
                .ok()?;
                committed_ws = Some((self.workspace_revision, min_d));
            }
            let mut c_meas = eval.alloc_buf();
            eval.eval_into(&self.measured_positions, &mut c_meas, "tracking_fallback").ok()?;
            let mut min_d_meas = eval.init_min_depths(&c_meas);
            eval.sweep_linear_step(
                &self.measured_positions,
                q_hold,
                &c_meas,
                &mut min_d_meas,
                budget,
                MAX_KEEPOUT_EVALS_PER_TICK,
                "tracking_fallback",
                "tracking_fallback",
            )
            .ok()?;
            if committed_ws.is_none() {
                committed_ws = Some((self.workspace_revision, min_d_meas));
            }
        }
        Some(committed_ws)
    }

    /// One tick of the rejection fallback: the next issued output from `prev_confirmed`
    /// (moving at `prev_velocities`) toward the hold pose `q_hold`, using the shared
    /// per-tick sweep `budget`.
    ///
    /// Candidates, in order, each checked for joint limits, velocity and acceleration
    /// from `prev_confirmed`, and swept workspace:
    /// 1. a bounded servo step toward `q_hold` (reaches it, or brakes toward it with a
    ///    feasible stopping distance; repeated over ticks it ends at rest on `q_hold`);
    /// 2. a pure braking step (velocity toward zero by at most `a_max * dt`); at rest it
    ///    repeats `prev_confirmed`.
    ///
    /// Never returns a step that changes an arm joint's issued velocity by more than
    /// `a_max * dt`. `Err` means no candidate is feasible: the caller latches a fault and
    /// sends no frame. Without `prev_confirmed`, `q_hold` itself is validated (no prior
    /// output to brake from). With stale feedback only candidate 2 is tried. With fresh
    /// feedback every moving step is also swept from the measurement; candidate 2 at rest
    /// (repeating `prev_confirmed`) is swept from `prev_confirmed` only.
    #[allow(clippy::too_many_arguments)]
    pub fn tracking_fallback_step(
        &self,
        prev_confirmed: Option<&[f32]>,
        prev_velocities: Option<&[f32]>,
        prev_ws_min_depths: Option<&(u64, crate::kinematics::SlotBuf)>,
        q_hold: &[f32],
        keepout_only: bool,
        dt: f64,
        active_limits: &[crate::model::JointLimit],
        fresh_meas: bool,
        budget: &mut usize,
    ) -> Result<ValidatedFallback, String> {
        let dof = self.config.dof;
        if !dt.is_finite() || dt <= 0.0 || active_limits.len() != dof || q_hold.len() != dof
            || !q_hold.iter().all(|q| q.is_finite())
        {
            return Err("invalid tracking fallback hold, limits or dt".into());
        }
        let arm_indices = self.config.arm_joint_indices();
        let prev_q = prev_confirmed.filter(|v| v.len() == dof && v.iter().all(|q| q.is_finite()));
        let zeros = vec![0.0_f32; dof];
        let prev_v = prev_velocities
            .filter(|v| v.len() == dof && v.iter().all(|q| q.is_finite()))
            .unwrap_or(&zeros);

        let Some(pq) = prev_q else {
            if !fresh_meas {
                return Err("tracking fallback without prior output needs fresh feedback".into());
            }
            for &j in &arm_indices {
                let lim = &active_limits[j];
                if q_hold[j] < lim.min_position - 1e-5 || q_hold[j] > lim.max_position + 1e-5 {
                    return Err(format!("joint {} fallback hold outside its limits", j + 1));
                }
            }
            let ws = self.fallback_workspace_check(None, prev_ws_min_depths, q_hold, keepout_only, true, budget)?;
            return Ok(ValidatedFallback { positions: q_hold.to_vec(), velocities: zeros, ws_min_depths: ws, done: true });
        };

        // A hold synthesized from a stale measurement is never a target: with stale
        // feedback only the pure braking step (independent of the measurement) is tried.
        let candidates: &[bool] = if fresh_meas { &[false, true] } else { &[true] };
        let mut last_err = String::from("no feasible tracking fallback step");
        for &brake_only in candidates {
            let mut q_c = q_hold.to_vec();
            let mut feasible = true;
            for &j in &arm_indices {
                let lim = &active_limits[j];
                let pv = prev_v[j];
                let q = if brake_only {
                    let dv = lim.max_acceleration * dt as f32;
                    let v_next = if pv > 0.0 { (pv - dv).max(0.0) } else { (pv + dv).min(0.0) };
                    pq[j] + v_next * dt as f32
                } else {
                    match crate::supervisor::SupervisorInner::bounded_servo_step(pq[j], q_hold[j], pv, dt, lim) {
                        Some((q_s, _, _)) => q_s,
                        None => {
                            last_err = format!("joint {} has no bounded step toward the fallback hold", j + 1);
                            feasible = false;
                            break;
                        }
                    }
                };
                q_c[j] = q;
            }
            if !feasible {
                continue;
            }
            // Torque hold: exactly the primary ramp's rule (`compute_lead_ramp_out`): never
            // advance the held joint past the last issued output or the hold position in the
            // push direction; the dynamics check below rejects a clamp beyond a_max.
            if let Some(h) = self.torque_guard.hold {
                let j = h.joint;
                if arm_indices.contains(&j) {
                    if (q_c[j] - pq[j]) * h.sign > 1e-6 {
                        q_c[j] = pq[j];
                    }
                    if (pq[j] - h.hold_q) * h.sign <= 1e-6 && (q_c[j] - h.hold_q) * h.sign > 1e-5 {
                        q_c[j] = h.hold_q;
                    }
                }
            }
            if let Err(e) = Self::fallback_dynamics_ok(&arm_indices, pq, prev_v, &q_c, dt, active_limits) {
                last_err = e;
                continue;
            }
            // Every step that moves the output (any bit of an arm target changes) is also swept
            // from the fresh measurement (a lagging arm travels from there); only a braking step
            // that repeats `prev_confirmed` bit for bit is swept from `prev_confirmed` alone.
            let moves = arm_indices.iter().any(|&j| q_c[j].to_bits() != pq[j].to_bits());
            let check_meas = fresh_meas && (moves || !brake_only);
            match self.fallback_workspace_check(Some(pq), prev_ws_min_depths, &q_c, keepout_only, check_meas, budget) {
                Ok(ws) => {
                    let velocities: Vec<f32> =
                        (0..dof).map(|i| ((f64::from(q_c[i]) - f64::from(pq[i])) / dt) as f32).collect();
                    // At rest (target bitwise unchanged) on the hold, or at rest after a pure
                    // braking step (the step toward the hold was refused): the ramp is complete.
                    let done = arm_indices.iter().all(|&j| {
                        q_c[j].to_bits() == pq[j].to_bits() && (brake_only || (q_c[j] - q_hold[j]).abs() <= 1e-5)
                    });
                    return Ok(ValidatedFallback { positions: q_c, velocities, ws_min_depths: ws, done });
                }
                Err(e) => last_err = e,
            }
        }
        Err(last_err)
    }

    /// Evaluate the rejection fallback toward `ramp.hold` from the current tracking state
    /// within `ramp.limits`, using the escape depths of the workspace `ramp.keepout_only`
    /// selects. Read-only:
    /// call it before any hold or fault invalidates the history.
    pub fn tracking_fallback_compute(&self, ramp: FallbackRamp, dt: f64, budget: &mut usize) -> PendingFallback {
        let active_limits = ramp.limits.as_slice();
        let fresh_meas = self.hardware_feedback_fresh(std::time::Duration::from_millis(100))
            && self.measured_positions.len() == self.config.dof
            && self.measured_positions.iter().all(|q| q.is_finite());
        let (depths, other_depths) = if ramp.keepout_only {
            (&self.tracking.ws_keepout_min_depths, &self.tracking.ws_min_depths)
        } else {
            (&self.tracking.ws_min_depths, &self.tracking.ws_keepout_min_depths)
        };
        let step = self.tracking_fallback_step(
            self.tracking.last_issued_positions.as_deref(),
            self.tracking.last_issued_velocities.as_deref(),
            depths.as_ref(),
            &ramp.hold,
            ramp.keepout_only,
            dt,
            active_limits,
            fresh_meas,
            budget,
        );
        PendingFallback {
            step,
            ramp,
            prev_confirmed: self.tracking.last_issued_positions.clone(),
            other_depths: other_depths.clone(),
        }
    }

    /// Joint limits, velocity and acceleration of `q` issued after `pq` at `pv`.
    fn fallback_dynamics_ok(
        arm_indices: &[usize],
        pq: &[f32],
        pv: &[f32],
        q: &[f32],
        dt: f64,
        active_limits: &[crate::model::JointLimit],
    ) -> Result<(), String> {
        for &j in arm_indices {
            let lim = &active_limits[j];
            if !q[j].is_finite() || q[j] < lim.min_position - 1e-5 || q[j] > lim.max_position + 1e-5 {
                return Err(format!("joint {} fallback step {:.4} rad outside its limits", j + 1, q[j]));
            }
            let v = (f64::from(q[j]) - f64::from(pq[j])) / dt;
            let a = (v - f64::from(pv[j])) / dt;
            if !v.is_finite() || v.abs() > f64::from(lim.max_velocity) + 1e-4
                || !a.is_finite() || a.abs() > f64::from(lim.max_acceleration) + 1e-3
            {
                return Err(format!(
                    "joint {} fallback step exceeds dynamics (v={v:.3}/{:.3}, a={a:.3}/{:.3})",
                    j + 1,
                    lim.max_velocity,
                    lim.max_acceleration
                ));
            }
        }
        Ok(())
    }

    /// Swept workspace check of a fallback step from `prev` (persistent escape depths
    /// `prev_depths`) and, when `fresh_meas`, from the measurement. With `keepout_only`
    /// (park) only the keep-out boxes and link checks apply, as in `compute_lead_ramp_out`.
    /// Returns the depths to commit.
    fn fallback_workspace_check(
        &self,
        prev: Option<&[f32]>,
        prev_depths: Option<&(u64, crate::kinematics::SlotBuf)>,
        q: &[f32],
        keepout_only: bool,
        fresh_meas: bool,
        budget: &mut usize,
    ) -> Result<Option<(u64, crate::kinematics::SlotBuf)>, String> {
        use crate::chain::WorkspaceEvaluator;
        use crate::supervisor::MAX_KEEPOUT_EVALS_PER_TICK;
        let Some((ws_full, cam)) = self.workspace_policy() else { return Ok(None) };
        let keepout_ws = keepout_only.then(|| ws_full.keepout_only());
        let ws = keepout_ws.as_ref().unwrap_or(ws_full);
        if keepout_only && ws.keepout_boxes.is_empty() {
            return Ok(prev_depths.cloned());
        }
        let eval = WorkspaceEvaluator::new(&self.config.arm_model, ws, cam);
        let mut committed = None;
        if let Some(pq) = prev {
            let mut c_prev = eval.alloc_buf();
            eval.eval_into(pq, &mut c_prev, "tracking_fallback")
                .map_err(|f| format!("fallback start violates the workspace: {}", crate::safety::SafetyError::from_workspace(f)))?;
            let mut min_d = match prev_depths {
                Some((rev, md)) if *rev == self.workspace_revision && md.as_slice().len() == eval.total_slots => md.clone(),
                _ => eval.init_min_depths(&c_prev),
            };
            eval.sweep_linear_step(pq, q, &c_prev, &mut min_d, budget, MAX_KEEPOUT_EVALS_PER_TICK,
                "tracking_fallback", "tracking_fallback")
                .map_err(|(_, f)| format!("fallback step violates the workspace: {}", crate::safety::SafetyError::from_workspace(f)))?;
            committed = Some((self.workspace_revision, min_d));
        }
        if !fresh_meas {
            return Ok(committed);
        }
        let mut c_meas = eval.alloc_buf();
        eval.eval_into(&self.measured_positions, &mut c_meas, "tracking_fallback")
            .map_err(|f| format!("measured pose violates the workspace: {}", crate::safety::SafetyError::from_workspace(f)))?;
        let mut min_d_meas = eval.init_min_depths(&c_meas);
        eval.sweep_linear_step(&self.measured_positions, q, &c_meas, &mut min_d_meas, budget,
            MAX_KEEPOUT_EVALS_PER_TICK, "tracking_fallback", "tracking_fallback")
            .map_err(|(_, f)| format!("fallback step from measured violates the workspace: {}", crate::safety::SafetyError::from_workspace(f)))?;
        Ok(committed.or(Some((self.workspace_revision, min_d_meas))))
    }

    pub fn validate_fallback_hold(
        &self,
        prev_confirmed: Option<&[f32]>,
        q_hold: &[f32],
        fresh_meas: bool,
    ) -> bool {
        let mut budget = crate::supervisor::MAX_KEEPOUT_EVALS_PER_TICK;
        self.validate_fallback_hold_with_budget(prev_confirmed, q_hold, fresh_meas, &mut budget)
    }

    pub fn validate_fallback_hold_with_budget(
        &self,
        prev_confirmed: Option<&[f32]>,
        q_hold: &[f32],
        fresh_meas: bool,
        budget: &mut usize,
    ) -> bool {
        let limits = self.active_tracking_limits(0.02, self.motion_time_scale());
        self.validate_fallback_hold_with_dynamics(
            prev_confirmed,
            self.tracking.last_issued_velocities.as_deref(),
            q_hold,
            0.02,
            &limits,
            fresh_meas,
            budget,
        )
        .is_some()
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
        // The strict parser (used for an enabled block) rejects unknown fields and strategies.
        assert!(serde_json::from_str::<JointTrackingConfig>(r#"{"enabled":false,"joints":[],"extra":1}"#).is_err());
        assert!(serde_json::from_str::<JointTrackingJoint>(r#"{"strategy":"position","extra":1}"#).is_err());
        assert!(serde_json::from_str::<JointTrackingJoint>(r#"{"strategy":"pd_plus"}"#).is_err());

        // Review 6 A: in a profile, a block that is not enabled never rejects the profile
        // (e4f72b6 ignored the field); it stays inert. An enabled block parses strictly.
        let parse = |block: &str| {
            let mut v = serde_json::to_value(crate::model::ArmConfig::rebot_dm()).unwrap();
            v["joint_tracking"] = serde_json::from_str(block).unwrap();
            serde_json::from_value::<crate::model::ArmConfig>(v)
        };
        for block in [
            r#"{"enabled":false,"joints":[],"extra":1}"#,
            r#"{"joints":[{"strategy":"pd_plus"}]}"#,
            r#"{"enabled":"yes","joints":[]}"#,
            r#"[1, 2]"#,
        ] {
            let cfg = parse(block).unwrap_or_else(|e| panic!("disabled block {block} must not reject the profile: {e}"));
            assert!(cfg.joint_tracking.is_none(), "invalid disabled block {block} stays inert");
        }
        let ok = parse(r#"{"enabled":false,"joints":[]}"#).unwrap();
        assert_eq!(ok.joint_tracking, Some(JointTrackingConfig { enabled: false, joints: vec![] }));
        assert!(parse(r#"{"enabled":true,"joints":[],"extra":1}"#).is_err(), "enabled block parses strictly");
        assert!(parse(r#"{"enabled":true,"joints":[{"strategy":"pd_plus"}]}"#).is_err());
        assert!(parse("null").unwrap().joint_tracking.is_none());
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
