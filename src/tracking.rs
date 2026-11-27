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
/// Damiao MIT frame gain ranges (`DamiaoLimits::default`: kp 0..500, kd 0..5).
pub const MIT_WIRE_KP_MAX: f32 = 500.0;
pub const MIT_WIRE_KD_MAX: f32 = 5.0;
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
    /// MIT stationary-hold stiffness (N·m/rad); only for `mit` joints, tuned separately from
    /// the tracking `kp` under the same ceilings (Phase 2).
    #[serde(default, skip_serializing_if = "Option::is_none", deserialize_with = "present_key")]
    pub hold_kp: Option<f32>,
    /// MIT stationary-hold damping (N·m·s/rad); only for `mit` joints (Phase 2).
    #[serde(default, skip_serializing_if = "Option::is_none", deserialize_with = "present_key")]
    pub hold_kd: Option<f32>,
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
            hold_kp: None,
            hold_kd: None,
        }
    }
}

/// A present key is Some, also when its value is null (as NaN, refused by validation): absent and
/// null differ like at f258cf9, where the key was unknown either way.
fn present_key<'de, D: serde::Deserializer<'de>>(d: D) -> Result<Option<f32>, D::Error> {
    Ok(Some(Option::<f32>::deserialize(d)?.unwrap_or(f32::NAN)))
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

    /// Indices into `joints` (arm order) whose strategy is `mit`.
    pub fn mit_joint_slots(&self) -> Vec<usize> {
        self.joints.iter().enumerate().filter(|(_, j)| j.strategy == TrackingStrategy::Mit).map(|(k, _)| k).collect()
    }

    /// Tracking is enabled and at least one joint selects `mit` (each such joint carries the
    /// MIT-only keys `hold_kp` / `hold_kd`, so no Phase 1 profile can request it).
    pub fn mit_requested(&self) -> bool {
        self.enabled && self.joints.iter().any(|j| j.strategy == TrackingStrategy::Mit)
    }

    /// Validate schema invariants. `position` and `position_lead` as in Phase 1. `mit` (Phase 2,
    /// disabled by default) is the explicit request: it needs the MIT-only keys `hold_kp` /
    /// `hold_kd` and positive `kp`, `kd`, `effort_limit_nm` within the wire ranges (profile
    /// ceilings and term fit: [`crate::model::ArmConfig::validate`]; activation: the
    /// qualification gate in `src/mit.rs`). Every configuration Phase 1 accepted or refused is
    /// accepted or refused unchanged: Phase 1 refused `mit` and the unknown keys `hold_kp` /
    /// `hold_kd`, and both stay refused unless they come together.
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
                if j.hold_kp.is_none() || j.hold_kd.is_none() {
                    return Err(format!(
                        "joint_tracking.joints[{i}]: strategy 'mit' needs hold_kp and hold_kd (MIT is disabled by default)"
                    ));
                }
                let positive = |name: &str, v: f32, max: f32| -> Result<(), String> {
                    if !v.is_finite() || v <= 0.0 || v > max {
                        return Err(format!("joint_tracking.joints[{i}].{name} must be in (0, {max}] for strategy 'mit', got {v}"));
                    }
                    Ok(())
                };
                positive("kp", j.kp, MIT_WIRE_KP_MAX)?;
                positive("kd", j.kd, MIT_WIRE_KD_MAX)?;
                positive("effort_limit_nm", j.effort_limit_nm, f32::MAX)?;
                positive("hold_kp", j.hold_kp.unwrap_or(f32::NAN), MIT_WIRE_KP_MAX)?;
                positive("hold_kd", j.hold_kd.unwrap_or(f32::NAN), MIT_WIRE_KD_MAX)?;
            } else if j.hold_kp.is_some() || j.hold_kd.is_some() {
                return Err(format!("joint_tracking.joints[{i}]: hold_kp / hold_kd are only for strategy 'mit'"));
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
    /// Nominal reference (`q_d`) of the tick that issued `last_issued_positions` and its
    /// discrete velocity (`(q_d - q_d_prev) / dt`). The issued lead is
    /// `last_issued_positions - last_nominal.0`. None: unknown, the issued output is taken
    /// as nominal (zero lead).
    pub last_nominal: Option<(Vec<f32>, Vec<f32>)>,
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
    /// True after a tracking rejection fallback finished braking (or stopped infeasible at the
    /// last confirmed issued pose) while the rejection fault stays latched on an `Armed` arm.
    /// Cleared on E-stop, FailedHold, clear_fault, or any new motion.
    pub holding_fallback_pose: bool,
    /// Reference rest sample: a fresh measured arm position of an earlier control tick, with
    /// the receive time of its newest joint sample, kept while the arm stays within
    /// tolerance of it (`SupervisorInner::note_tracking_position_sample`): stationarity
    /// evidence for drivers without velocity feedback.
    /// Tuple: (oldest joint reply, newest joint reply, positions).
    pub position_sample: Option<(std::time::Instant, std::time::Instant, Vec<f32>)>,
    /// Motion caps (`ActiveTrajectory::motion_limits`) of a planned motion whose reference
    /// finished, kept in force until it has settled and its residual lead is ramped out.
    pub retained_motion_limits: Option<Vec<crate::model::JointLimit>>,
    /// Zero-allocation inline velocity caps (`ActiveTrajectory::motion_vel_caps`) retained
    /// through post-quintic settling until lead is ramped out.
    pub retained_motion_vel_caps: Option<([f32; crate::model::MAX_MOTION_DOF], usize)>,
    /// Ticks whose issued output was the lead withdrawal after the lead candidate failed
    /// validation (`TrackingOutput::withdrawn`); diagnostics only.
    pub withdrawal_ticks: u64,
    /// Preallocated spare buffers returned on [`Self::invalidate`] and reused by [`Self::seed`]
    /// so PositionLead enable completion on RT wakes never allocates or frees.
    spare_issued_positions: Option<Vec<f32>>,
    spare_issued_velocities: Option<Vec<f32>>,
    spare_nominal: Option<(Vec<f32>, Vec<f32>)>,
    spare_position_sample: Option<Vec<f32>>,
    #[cfg(not(target_arch = "wasm32"))]
    spare_fallback_hold: Option<Vec<f32>>,
    #[cfg(not(target_arch = "wasm32"))]
    spare_fallback_limits: Option<Vec<crate::model::JointLimit>>,
    #[cfg(not(target_arch = "wasm32"))]
    spare_ws_min_depths: Option<Vec<f64>>,
    #[cfg(not(target_arch = "wasm32"))]
    spare_ws_keepout_min_depths: Option<Vec<f64>>,
}

/// Heap buffers displaced by [`TrackingRuntime::invalidate_retiring`] so callers on the RT
/// tick thread can retire them off-thread without freeing inside a release.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Default)]
pub(crate) struct TrackingRetiredBuffers {
    pub(crate) issued_positions: Option<Vec<f32>>,
    pub(crate) issued_velocities: Option<Vec<f32>>,
    pub(crate) nominal: Option<(Vec<f32>, Vec<f32>)>,
    pub(crate) ws_min_depths: Option<(u64, crate::kinematics::SlotBuf)>,
    pub(crate) ws_keepout_min_depths: Option<(u64, crate::kinematics::SlotBuf)>,
    pub(crate) fallback_ramp: Option<FallbackRamp>,
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
/// state it was evaluated from (previous issued pose is saved in `scratch.fallback_prev_confirmed`).
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone)]
pub struct PendingFallback {
    pub step: Result<ValidatedFallback, String>,
    pub ramp: FallbackRamp,
    /// Escape depths of the other workspace (full when `keepout_only`, park otherwise),
    /// kept unchanged when the step is recorded.
    pub other_depths: Option<(u64, crate::kinematics::SlotBuf)>,
}

/// Zero-allocation RT error for [`SupervisorInner::tracking_fallback_compute_inline`].
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FallbackStepErr {
    InvalidInputs,
    NeedsFreshFeedback,
    HoldOutsideLimits { joint: u8 },
    NoBoundedStep { joint: u8 },
    StepOutsideLimits { joint: u8, q: f32 },
    ExceedsDynamics { joint: u8, v: f32, max_v: f32, a: f32, max_a: f32 },
    WorkspaceStartViolation(crate::chain::WorkspaceFailureRt),
    WorkspaceStepViolation(crate::chain::WorkspaceFailureRt),
    WorkspaceMeasuredViolation(crate::chain::WorkspaceFailureRt),
    WorkspaceMeasuredSweepViolation(crate::chain::WorkspaceFailureRt),
    NoFeasibleStep,
}

#[cfg(not(target_arch = "wasm32"))]
fn fmt_ws_failure_rt(
    f: &mut core::fmt::Formatter<'_>,
    prefix: &str,
    err: crate::chain::WorkspaceFailureRt,
) -> core::fmt::Result {
    use crate::chain::WorkspaceFailureKindRt;
    match err.kind {
        WorkspaceFailureKindRt::Invalid(msg) => write!(f, "{prefix}: {msg}"),
        WorkspaceFailureKindRt::UnknownCapsule(ci) => {
            write!(f, "{prefix}: unknown capsule frame (index {ci})")
        }
        WorkspaceFailureKindRt::InvalidSelfPair(pi) => {
            write!(f, "{prefix}: invalid self-collision pair (index {pi})")
        }
        WorkspaceFailureKindRt::BudgetExceeded { max_budget, start_outside, .. } => {
            let path_kind = if start_outside { "return path" } else { "path" };
            write!(f, "{prefix}: {path_kind} could not be checked (needs more than {max_budget} FK evaluations)")
        }
        WorkspaceFailureKindRt::Constraint { slot, depth, min_d, .. } => {
            write!(
                f,
                "{prefix} (slot {slot}, previous depth {:.3} m, new depth {:.3} m)",
                min_d.max(0.0),
                depth,
            )
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl core::fmt::Display for FallbackStepErr {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self {
            Self::InvalidInputs => f.write_str("invalid tracking fallback hold, limits or dt"),
            Self::NeedsFreshFeedback => {
                f.write_str("tracking fallback without prior output needs fresh feedback")
            }
            Self::HoldOutsideLimits { joint } => {
                write!(f, "joint {joint} fallback hold outside its limits")
            }
            Self::NoBoundedStep { joint } => {
                write!(f, "joint {joint} has no bounded step toward the fallback hold")
            }
            Self::StepOutsideLimits { joint, q } => {
                write!(f, "joint {joint} fallback step {q:.4} rad outside its limits")
            }
            Self::ExceedsDynamics { joint, v, max_v, a, max_a } => {
                write!(
                    f,
                    "joint {joint} fallback step exceeds dynamics (v={v:.3}/{max_v:.3}, a={a:.3}/{max_a:.3})"
                )
            }
            Self::WorkspaceStartViolation(err) => {
                fmt_ws_failure_rt(f, "fallback start violates the workspace", err)
            }
            Self::WorkspaceStepViolation(err) => {
                fmt_ws_failure_rt(f, "fallback step violates the workspace", err)
            }
            Self::WorkspaceMeasuredViolation(err) => {
                fmt_ws_failure_rt(f, "measured pose violates the workspace", err)
            }
            Self::WorkspaceMeasuredSweepViolation(err) => {
                fmt_ws_failure_rt(f, "fallback step from measured violates the workspace", err)
            }
            Self::NoFeasibleStep => f.write_str("no feasible tracking fallback step"),
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl TrackingRuntime {
    /// Preallocate spare output-history vectors for `dof` joints so RT enable completion
    /// (`seed`) and `invalidate` reuse buffers with zero heap allocation or drop.
    pub fn with_capacity(dof: usize) -> Self {
        Self {
            recovery_joints: Vec::with_capacity(dof),
            spare_issued_positions: Some(Vec::with_capacity(dof)),
            spare_issued_velocities: Some(Vec::with_capacity(dof)),
            spare_nominal: Some((Vec::with_capacity(dof), Vec::with_capacity(dof))),
            spare_position_sample: Some(Vec::with_capacity(dof)),
            spare_fallback_hold: Some(Vec::with_capacity(dof)),
            spare_fallback_limits: Some(Vec::with_capacity(dof)),
            ..Self::default()
        }
    }

    fn recycle_ws_depth_slot(
        slot: &mut Option<(u64, crate::kinematics::SlotBuf)>,
        spare: &mut Option<Vec<f64>>,
    ) {
        if let Some((_, old)) = slot.take() {
            if let Some(v) = old.into_heap_vec() {
                if spare.is_none() {
                    *spare = Some(v);
                }
            }
        }
    }

    fn set_ws_depth_slot_from_slice(
        slot: &mut Option<(u64, crate::kinematics::SlotBuf)>,
        spare: &mut Option<Vec<f64>>,
        next: Option<(u64, &[f64])>,
    ) {
        let Some((rev, src)) = next else {
            Self::recycle_ws_depth_slot(slot, spare);
            return;
        };
        if let Some((cur_rev, buf)) = slot.as_mut() {
            if buf.can_fit(src.len()) {
                *cur_rev = rev;
                buf.copy_from_slice_in_place(src);
                return;
            }
        }
        if src.len() > crate::kinematics::MAX_STACK_WORKSPACE_SLOTS
            && spare.as_ref().is_some_and(|v| v.capacity() >= src.len())
        {
            let mut v = spare.take().expect("checked above");
            v.clear();
            v.extend_from_slice(src);
            if let Some((_, old)) = slot.replace((rev, crate::kinematics::SlotBuf::Heap(v))) {
                if let Some(old_v) = old.into_heap_vec() {
                    if spare.is_none() {
                        *spare = Some(old_v);
                    }
                }
            }
            return;
        }
        let mut buf = crate::kinematics::SlotBuf::zeros(src.len());
        buf.copy_from_slice_in_place(src);
        if let Some((_, old)) = slot.replace((rev, buf)) {
            if let Some(old_v) = old.into_heap_vec() {
                if spare.is_none() {
                    *spare = Some(old_v);
                }
            }
        }
    }

    /// Installs preallocated heap `Vec<f64>` spares from `bufs` (for workspaces > 256 slots)
    /// while maintaining the single-buffer spare invariant (at most one heap backing buffer across
    /// active slot + spare). When `retire_into` is `Some(old_bufs)` (RT workspace config swap),
    /// any existing cached depths and old spares are moved into `old_bufs` for off-RT drop.
    pub(crate) fn install_ws_depth_spares_from_eval_bufs(
        &mut self,
        bufs: &mut crate::chain::EvalBufs,
        retire_into: Option<&mut crate::chain::EvalBufs>,
    ) {
        let new_ws = bufs.tracking_ws_spare.take();
        let new_ko = bufs.tracking_keepout_spare.take();
        let install_one = |slot: &mut Option<(u64, crate::kinematics::SlotBuf)>,
                           spare: &mut Option<Vec<f64>>,
                           new_spare: Option<Vec<f64>>,
                           retire_slot: Option<&mut Option<Vec<f64>>>| {
            if let Some(dst) = retire_slot {
                // RT config swap: workspace revision changed, so active depths are stale.
                // Single-buffer invariant guarantees at most one of (slot, spare) is Heap.
                let old_active = slot.take().and_then(|(_, b)| b.into_heap_vec());
                let old_spare = spare.take();
                *dst = old_active.or(old_spare);
                *spare = new_spare;
            } else {
                // Off-thread prepare: preserve valid live heap depths if already sized; drop redundant spare.
                let need = new_spare.as_ref().map_or(0, Vec::len);
                if need > crate::kinematics::MAX_STACK_WORKSPACE_SLOTS
                    && slot.as_ref().is_some_and(|(_, b)| b.is_heap() && b.can_fit(need))
                {
                    *spare = None;
                } else {
                    if slot.as_ref().is_some_and(|(_, b)| b.is_heap()) {
                        *slot = None;
                    }
                    *spare = new_spare;
                }
            }
        };
        if let Some(old_bufs) = retire_into {
            install_one(
                &mut self.ws_min_depths,
                &mut self.spare_ws_min_depths,
                new_ws,
                Some(&mut old_bufs.tracking_ws_spare),
            );
            install_one(
                &mut self.ws_keepout_min_depths,
                &mut self.spare_ws_keepout_min_depths,
                new_ko,
                Some(&mut old_bufs.tracking_keepout_spare),
            );
        } else {
            install_one(&mut self.ws_min_depths, &mut self.spare_ws_min_depths, new_ws, None);
            install_one(
                &mut self.ws_keepout_min_depths,
                &mut self.spare_ws_keepout_min_depths,
                new_ko,
                None,
            );
        }
    }

    /// Preallocate `spare_fallback_hold` and `spare_fallback_limits` (including each
    /// `JointLimit::name` buffer) off-RT so the first tracking-rejection fallback on
    /// `armd-control` performs zero heap allocations, while maintaining the single-buffer
    /// spare invariant when `fallback_ramp` is already occupied so `recycle_fallback_ramp`
    /// never frees on the RT thread.
    pub(crate) fn prewarm_fallback_spares(&mut self, limits: &[crate::model::JointLimit]) {
        if self.fallback_ramp.is_some() {
            self.spare_fallback_hold = None;
            self.spare_fallback_limits = None;
            return;
        }
        let hold = self
            .spare_fallback_hold
            .get_or_insert_with(|| Vec::with_capacity(limits.len()));
        if hold.capacity() < limits.len() {
            hold.reserve(limits.len() - hold.len());
        }
        let lim = self
            .spare_fallback_limits
            .get_or_insert_with(|| Vec::with_capacity(limits.len()));
        crate::rt_core::scratch::copy_limits(lim, limits);
    }

    /// Constructs a [`FallbackRamp`] by reusing preallocated spare `hold` and `limits` buffers
    /// (zero heap allocations on RT).
    pub fn build_fallback_ramp(
        &mut self,
        hold_src: &[f32],
        keepout_only: bool,
        limits_src: &[crate::model::JointLimit],
    ) -> FallbackRamp {
        let mut hold = self
            .spare_fallback_hold
            .take()
            .unwrap_or_else(|| Vec::with_capacity(hold_src.len()));
        crate::rt_core::scratch::copy_into(&mut hold, hold_src);
        let mut limits = self
            .spare_fallback_limits
            .take()
            .unwrap_or_else(|| Vec::with_capacity(limits_src.len()));
        crate::rt_core::scratch::copy_limits(&mut limits, limits_src);
        FallbackRamp {
            hold,
            keepout_only,
            limits,
        }
    }

    /// Returns a displaced [`FallbackRamp`]'s vectors to preallocated spares so RT clear/finish
    /// paths never free heap memory (keeps `limits` entries intact so `copy_limits` reuses each
    /// `JointLimit::name` buffer without allocation on the next fallback).
    pub fn recycle_fallback_ramp(&mut self, mut ramp: FallbackRamp) {
        ramp.hold.clear();
        if self.spare_fallback_hold.is_none() {
            self.spare_fallback_hold = Some(ramp.hold);
        }
        if self.spare_fallback_limits.is_none() {
            self.spare_fallback_limits = Some(ramp.limits);
        }
    }

    /// Park any active `position_sample` buffer into `spare_position_sample` (0 dealloc on RT).
    #[inline]
    pub fn clear_position_sample(&mut self) {
        if let Some((_, _, mut buf)) = self.position_sample.take() {
            buf.clear();
            if self.spare_position_sample.is_none() {
                self.spare_position_sample = Some(buf);
            }
        }
    }

    /// Update `position_sample` reusing existing or preallocated spare storage (0 alloc on RT).
    #[inline]
    pub fn set_position_sample(
        &mut self,
        oldest: std::time::Instant,
        newest: std::time::Instant,
        positions: &[f32],
    ) {
        if let Some((o, n, q_ref)) = self.position_sample.as_mut() {
            *o = oldest;
            *n = newest;
            crate::rt_core::scratch::copy_into(q_ref, positions);
        } else {
            let mut buf = self
                .spare_position_sample
                .take()
                .unwrap_or_else(|| Vec::with_capacity(positions.len()));
            crate::rt_core::scratch::copy_into(&mut buf, positions);
            self.position_sample = Some((oldest, newest, buf));
        }
    }

    /// Seed output history from fresh measurements as a confirmed stationary baseline.
    pub fn seed(&mut self, positions: &[f32]) {
        if positions.iter().all(|q| q.is_finite()) {
            self.history_state = TrackingHistoryState::Stationary;
            let mut pos_buf = self
                .last_issued_positions
                .take()
                .or_else(|| self.spare_issued_positions.take())
                .unwrap_or_else(|| Vec::with_capacity(positions.len()));
            pos_buf.clear();
            pos_buf.extend_from_slice(positions);
            self.last_issued_positions = Some(pos_buf);

            let mut vel_buf = self
                .last_issued_velocities
                .take()
                .or_else(|| self.spare_issued_velocities.take())
                .unwrap_or_else(|| Vec::with_capacity(positions.len()));
            vel_buf.clear();
            vel_buf.resize(positions.len(), 0.0);
            self.last_issued_velocities = Some(vel_buf);

            let (mut nom_q, mut nom_v) = self
                .last_nominal
                .take()
                .or_else(|| self.spare_nominal.take())
                .unwrap_or_else(|| (Vec::with_capacity(positions.len()), Vec::with_capacity(positions.len())));
            nom_q.clear();
            nom_q.extend_from_slice(positions);
            nom_v.clear();
            nom_v.resize(positions.len(), 0.0);
            self.last_nominal = Some((nom_q, nom_v));

            Self::recycle_ws_depth_slot(&mut self.ws_min_depths, &mut self.spare_ws_min_depths);
            Self::recycle_ws_depth_slot(&mut self.ws_keepout_min_depths, &mut self.spare_ws_keepout_min_depths);
            self.recovery_suspended = false;
            self.recovery_joints.clear();
            if let Some(old) = self.fallback_ramp.take() {
                self.recycle_fallback_ramp(old);
            }
            self.holding_fallback_pose = false;
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
        Self::recycle_ws_depth_slot(&mut self.ws_min_depths, &mut self.spare_ws_min_depths);
        Self::recycle_ws_depth_slot(&mut self.ws_keepout_min_depths, &mut self.spare_ws_keepout_min_depths);
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
        Self::recycle_ws_depth_slot(&mut self.ws_min_depths, &mut self.spare_ws_min_depths);
        Self::recycle_ws_depth_slot(&mut self.ws_keepout_min_depths, &mut self.spare_ws_keepout_min_depths);
        if let Some(prev_q) = self.last_issued_positions.as_mut() {
            let dof = prev_q.len();
            if self.last_issued_velocities.is_none() {
                let mut v = self
                    .spare_issued_velocities
                    .take()
                    .unwrap_or_else(|| Vec::with_capacity(dof));
                v.clear();
                v.resize(dof, 0.0);
                self.last_issued_velocities = Some(v);
            }
            let prev_v = self.last_issued_velocities.as_mut().expect("initialized above");
            // Only the recovered joints were reseeded at rest by the driver's hold; every
            // other joint keeps its real issued velocity for the next acceleration check.
            for &(j, q) in recovered_positions {
                if j < dof && q.is_finite() {
                    prev_q[j] = q;
                    if let Some(v) = prev_v.get_mut(j) {
                        *v = 0.0;
                    }
                    // The driver's hold put the joint at rest at q with zero lead.
                    if let Some((nq, nv)) = self.last_nominal.as_mut().filter(|(nq, nv)| nq.len() == dof && nv.len() == dof) {
                        nq[j] = q;
                        nv[j] = 0.0;
                    }
                }
            }
        } else {
            self.history_state = TrackingHistoryState::Unknown;
        self.retained_motion_vel_caps = None;
            self.last_issued_velocities = None;
            self.last_nominal = None;
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
        self.last_nominal = None;
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

    /// Zero-allocation in-place commit of issued output + nominal history (reusing existing or
    /// preallocated spare buffers, 0 heap allocs and 0 heap frees on `armd-control`).
    pub(crate) fn record_issued_and_nominal_inline(
        &mut self,
        positions: &[f32],
        velocities: &[f32],
        q_d: &[f32],
        dt: f64,
        ws_min_depths: Option<(u64, &[f64])>,
        ws_keepout_min_depths: Option<(u64, &[f64])>,
    ) {
        let dof = q_d.len().min(crate::model::MAX_MOTION_DOF);
        // Compute nominal velocity from previous nominal/issued pose BEFORE overwriting.
        let mut nom_v_tmp = [0.0f32; crate::model::MAX_MOTION_DOF];
        if dt.is_finite() && dt > 0.0 {
            let prev = self
                .last_nominal
                .as_ref()
                .map(|(q, _)| q.as_slice())
                .or(self.last_issued_positions.as_deref())
                .filter(|q| q.len() == q_d.len());
            if let Some(p) = prev {
                for i in 0..dof {
                    nom_v_tmp[i] = ((f64::from(q_d[i]) - f64::from(p[i])) / dt) as f32;
                }
            }
        }
        self.history_state = TrackingHistoryState::Issued;
        if let Some(pos_buf) = self.last_issued_positions.as_mut() {
            crate::rt_core::scratch::copy_into(pos_buf, positions);
        } else {
            let mut pos_buf = self
                .spare_issued_positions
                .take()
                .unwrap_or_else(|| Vec::with_capacity(positions.len()));
            crate::rt_core::scratch::copy_into(&mut pos_buf, positions);
            self.last_issued_positions = Some(pos_buf);
        }
        if let Some(vel_buf) = self.last_issued_velocities.as_mut() {
            crate::rt_core::scratch::copy_into(vel_buf, velocities);
        } else {
            let mut vel_buf = self
                .spare_issued_velocities
                .take()
                .unwrap_or_else(|| Vec::with_capacity(velocities.len()));
            crate::rt_core::scratch::copy_into(&mut vel_buf, velocities);
            self.last_issued_velocities = Some(vel_buf);
        }
        if let Some((nq, nv)) = self.last_nominal.as_mut() {
            crate::rt_core::scratch::copy_into(nq, q_d);
            crate::rt_core::scratch::copy_into(nv, &nom_v_tmp[..dof]);
        } else {
            let (mut nq, mut nv) = self
                .spare_nominal
                .take()
                .unwrap_or_else(|| (Vec::with_capacity(q_d.len()), Vec::with_capacity(q_d.len())));
            crate::rt_core::scratch::copy_into(&mut nq, q_d);
            crate::rt_core::scratch::copy_into(&mut nv, &nom_v_tmp[..dof]);
            self.last_nominal = Some((nq, nv));
        }
        Self::set_ws_depth_slot_from_slice(&mut self.ws_min_depths, &mut self.spare_ws_min_depths, ws_min_depths);
        Self::set_ws_depth_slot_from_slice(
            &mut self.ws_keepout_min_depths,
            &mut self.spare_ws_keepout_min_depths,
            ws_keepout_min_depths,
        );
    }

    /// Zero-allocation in-place commit of issued output + both workspace depth slots (clears `last_nominal`
    /// back into `spare_nominal` so RT fallback ticks never allocate or free).
    pub(crate) fn record_issued_with_ws_depths_inline(
        &mut self,
        positions: &[f32],
        velocities: &[f32],
        ws_min_depths: Option<(u64, &[f64])>,
        ws_keepout_min_depths: Option<(u64, &[f64])>,
    ) {
        self.history_state = TrackingHistoryState::Issued;
        if let Some(p) = self.last_issued_positions.as_mut() {
            crate::rt_core::scratch::copy_into(p, positions);
        } else {
            let mut pos_buf = self
                .spare_issued_positions
                .take()
                .unwrap_or_else(|| Vec::with_capacity(positions.len()));
            crate::rt_core::scratch::copy_into(&mut pos_buf, positions);
            self.last_issued_positions = Some(pos_buf);
        }
        if let Some(v) = self.last_issued_velocities.as_mut() {
            crate::rt_core::scratch::copy_into(v, velocities);
        } else {
            let mut vel_buf = self
                .spare_issued_velocities
                .take()
                .unwrap_or_else(|| Vec::with_capacity(velocities.len()));
            crate::rt_core::scratch::copy_into(&mut vel_buf, velocities);
            self.last_issued_velocities = Some(vel_buf);
        }
        if let Some(pair) = self.last_nominal.take() {
            if self.spare_nominal.is_none() {
                self.spare_nominal = Some(pair);
            }
        }
        Self::set_ws_depth_slot_from_slice(&mut self.ws_min_depths, &mut self.spare_ws_min_depths, ws_min_depths);
        Self::set_ws_depth_slot_from_slice(
            &mut self.ws_keepout_min_depths,
            &mut self.spare_ws_keepout_min_depths,
            ws_keepout_min_depths,
        );
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
        self.last_nominal = None;
        self.ws_min_depths = ws_min_depths;
        self.ws_keepout_min_depths = ws_keepout_min_depths;
    }

    /// Record a successfully issued output vector strictly in place (without touching `last_nominal`),
    /// reusing existing/spare buffers for zero allocation and zero deallocation on `armd-control`.
    pub fn record_issued_in_place(
        &mut self,
        positions: &[f32],
        velocities: &[f32],
        ws_min_depths: Option<(u64, crate::kinematics::SlotBuf)>,
        ws_keepout_min_depths: Option<(u64, crate::kinematics::SlotBuf)>,
    ) {
        if let Some(ref mut dst) = self.last_issued_positions {
            crate::rt_core::scratch::copy_into(dst, positions);
        } else {
            let mut dst = self
                .spare_issued_positions
                .take()
                .unwrap_or_else(|| Vec::with_capacity(positions.len()));
            crate::rt_core::scratch::copy_into(&mut dst, positions);
            self.last_issued_positions = Some(dst);
        }
        if let Some(ref mut dst) = self.last_issued_velocities {
            crate::rt_core::scratch::copy_into(dst, velocities);
        } else {
            let mut dst = self
                .spare_issued_velocities
                .take()
                .unwrap_or_else(|| Vec::with_capacity(velocities.len()));
            crate::rt_core::scratch::copy_into(&mut dst, velocities);
            self.last_issued_velocities = Some(dst);
        }
        self.history_state = TrackingHistoryState::Issued;
        self.ws_min_depths = ws_min_depths;
        self.ws_keepout_min_depths = ws_keepout_min_depths;
    }

    /// Record a successfully issued output vector and update nominal history strictly in place,
    /// reusing existing/spare buffers so steady-state tracking ticks perform zero allocations
    /// and zero deallocations on `armd-control`.
    pub fn record_issued_and_nominal_in_place(
        &mut self,
        positions: &[f32],
        velocities: &[f32],
        nominal_q: &[f32],
        dt: f64,
        ws_min_depths: Option<(u64, crate::kinematics::SlotBuf)>,
        ws_keepout_min_depths: Option<(u64, crate::kinematics::SlotBuf)>,
    ) {
        let dof = nominal_q.len();
        let valid_dt = dt.is_finite() && dt > 0.0;
        if let Some((ref mut nq, ref mut nv)) = self.last_nominal {
            nv.clear();
            if nq.len() == dof && valid_dt {
                for (&q, &p0) in nominal_q.iter().zip(nq.iter()) {
                    nv.push(((f64::from(q) - f64::from(p0)) / dt) as f32);
                }
            } else if let Some(prev) = self.last_issued_positions.as_ref().filter(|q| q.len() == dof && valid_dt) {
                for (&q, &p0) in nominal_q.iter().zip(prev.iter()) {
                    nv.push(((f64::from(q) - f64::from(p0)) / dt) as f32);
                }
            } else {
                nv.resize(dof, 0.0);
            }
            crate::rt_core::scratch::copy_into(nq, nominal_q);
        } else {
            let (mut nq, mut nv) = self
                .spare_nominal
                .take()
                .unwrap_or_else(|| (Vec::with_capacity(dof), Vec::with_capacity(dof)));
            nv.clear();
            if let Some(prev) = self.last_issued_positions.as_ref().filter(|q| q.len() == dof && valid_dt) {
                for (&q, &p0) in nominal_q.iter().zip(prev.iter()) {
                    nv.push(((f64::from(q) - f64::from(p0)) / dt) as f32);
                }
            } else {
                nv.resize(dof, 0.0);
            }
            crate::rt_core::scratch::copy_into(&mut nq, nominal_q);
            self.last_nominal = Some((nq, nv));
        }

        if let Some(ref mut dst) = self.last_issued_positions {
            crate::rt_core::scratch::copy_into(dst, positions);
        } else {
            let mut dst = self
                .spare_issued_positions
                .take()
                .unwrap_or_else(|| Vec::with_capacity(positions.len()));
            crate::rt_core::scratch::copy_into(&mut dst, positions);
            self.last_issued_positions = Some(dst);
        }

        if let Some(ref mut dst) = self.last_issued_velocities {
            crate::rt_core::scratch::copy_into(dst, velocities);
        } else {
            let mut dst = self
                .spare_issued_velocities
                .take()
                .unwrap_or_else(|| Vec::with_capacity(velocities.len()));
            crate::rt_core::scratch::copy_into(&mut dst, velocities);
            self.last_issued_velocities = Some(dst);
        }

        self.history_state = TrackingHistoryState::Issued;
        self.ws_min_depths = ws_min_depths;
        self.ws_keepout_min_depths = ws_keepout_min_depths;
    }

    /// Off-thread normalization before RT executor handoff: drops only redundant spare buffers
    /// that collide with currently-occupied active history buffers while preserving live tracking
    /// history, workspace escape depths, and any active fallback braking ramp.
    pub(crate) fn normalize_spares_off_thread(&mut self) {
        if self.last_issued_positions.is_some() {
            self.spare_issued_positions = None;
        }
        if self.last_issued_velocities.is_some() {
            self.spare_issued_velocities = None;
        }
        if self.last_nominal.is_some() {
            self.spare_nominal = None;
        }
        if self.position_sample.is_some() {
            self.spare_position_sample = None;
        }
        if self.fallback_ramp.is_some() {
            self.spare_fallback_hold = None;
            self.spare_fallback_limits = None;
        }
        if self.ws_min_depths.as_ref().is_some_and(|(_, b)| b.is_heap()) {
            self.spare_ws_min_depths = None;
        }
        if self.ws_keepout_min_depths.as_ref().is_some_and(|(_, b)| b.is_heap()) {
            self.spare_ws_keepout_min_depths = None;
        }
    }

    /// Zero-free variant of [`Self::invalidate`] for RT-thread callers: moves any displaced heap
    /// buffers (including when preallocated spares, heap-backed depth slots, or `fallback_ramp`
    /// are occupied) into the returned [`TrackingRetiredBuffers`] so the caller can drop them off
    /// the RT thread.
    pub(crate) fn invalidate_retiring(&mut self) -> TrackingRetiredBuffers {
        self.history_state = TrackingHistoryState::Unknown;
        self.retained_motion_vel_caps = None;
        let mut retired = TrackingRetiredBuffers::default();
        if let Some(v) = self.last_issued_positions.take() {
            if self.spare_issued_positions.is_none() {
                self.spare_issued_positions = Some(v);
            } else {
                retired.issued_positions = Some(v);
            }
        }
        if let Some(v) = self.last_issued_velocities.take() {
            if self.spare_issued_velocities.is_none() {
                self.spare_issued_velocities = Some(v);
            } else {
                retired.issued_velocities = Some(v);
            }
        }
        if let Some(pair) = self.last_nominal.take() {
            if self.spare_nominal.is_none() {
                self.spare_nominal = Some(pair);
            } else {
                retired.nominal = Some(pair);
            }
        }
        if let Some((rev, b)) = self.ws_min_depths.take() {
            match b.into_heap_vec() {
                Some(v) if self.spare_ws_min_depths.is_none() => {
                    self.spare_ws_min_depths = Some(v);
                }
                Some(v) => {
                    retired.ws_min_depths = Some((rev, crate::kinematics::SlotBuf::Heap(v)));
                }
                None => {}
            }
        }
        if let Some((rev, b)) = self.ws_keepout_min_depths.take() {
            match b.into_heap_vec() {
                Some(v) if self.spare_ws_keepout_min_depths.is_none() => {
                    self.spare_ws_keepout_min_depths = Some(v);
                }
                Some(v) => {
                    retired.ws_keepout_min_depths = Some((rev, crate::kinematics::SlotBuf::Heap(v)));
                }
                None => {}
            }
        }
        retired.fallback_ramp = self.fallback_ramp.take();
        self.holding_fallback_pose = false;
        retired
    }

    /// Test hook exposing [`Self::invalidate_retiring`] as an opaque drop-deferred token.
    #[doc(hidden)]
    pub fn invalidate_retiring_for_test(&mut self) -> impl Sized {
        self.invalidate_retiring()
    }

    /// Invalidate output history (partial writes, suspension, holds/faults, mode changes).
    pub fn invalidate(&mut self) {
        self.history_state = TrackingHistoryState::Unknown;
        self.retained_motion_vel_caps = None;
        if let Some(v) = self.last_issued_positions.take() {
            if self.spare_issued_positions.is_none() {
                self.spare_issued_positions = Some(v);
            }
        }
        if let Some(v) = self.last_issued_velocities.take() {
            if self.spare_issued_velocities.is_none() {
                self.spare_issued_velocities = Some(v);
            }
        }
        if let Some(pair) = self.last_nominal.take() {
            if self.spare_nominal.is_none() {
                self.spare_nominal = Some(pair);
            }
        }
        Self::recycle_ws_depth_slot(&mut self.ws_min_depths, &mut self.spare_ws_min_depths);
        Self::recycle_ws_depth_slot(&mut self.ws_keepout_min_depths, &mut self.spare_ws_keepout_min_depths);
        if let Some(old) = self.fallback_ramp.take() {
            self.recycle_fallback_ramp(old);
        }
        self.holding_fallback_pose = false;
    }

    /// Nominal history entry for an output issued this tick against nominal `q_d`: `q_d`
    /// and its discrete velocity from the previous nominal (from the previous issued output
    /// when the nominal is unknown; zero without history). Call before `record_issued*`.
    pub fn nominal_entry(&self, q_d: &[f32], dt: f64) -> (Vec<f32>, Vec<f32>) {
        let dof = q_d.len();
        let prev = self
            .last_nominal
            .as_ref()
            .map(|(q, _)| q)
            .or(self.last_issued_positions.as_ref())
            .filter(|q| q.len() == dof && dt.is_finite() && dt > 0.0);
        let v = match prev {
            Some(p) => q_d.iter().zip(p).map(|(&q, &p0)| ((f64::from(q) - f64::from(p0)) / dt) as f32).collect(),
            None => vec![0.0; dof],
        };
        (q_d.to_vec(), v)
    }
}

/// Inline text capacity for zero-allocation RT tracking rejection / fallback diagnostics
/// (sized so `TrackingRejectRt` and `BoundedText<TRACKING_RT_ERR_CAP>` stay <= 128 bytes on the stack).
#[cfg(not(target_arch = "wasm32"))]
pub(crate) const TRACKING_RT_ERR_CAP: usize = 112;

/// Zero-allocation RT tracking rejection payload (`armd-control`).
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone)]
pub(crate) enum TrackingRejectRt {
    JointLimit(crate::safety::BoundedText<TRACKING_RT_ERR_CAP>),
    Workspace(crate::chain::WorkspaceFailureRt),
}

#[cfg(not(target_arch = "wasm32"))]
impl TrackingRejectRt {
    pub(crate) fn joint_limit_fmt(
        f: impl FnOnce(&mut crate::safety::BoundedText<TRACKING_RT_ERR_CAP>),
    ) -> Self {
        let mut buf = crate::safety::BoundedText::empty();
        f(&mut buf);
        Self::JointLimit(buf)
    }

    pub(crate) fn joint_limit_str(msg: &str) -> Self {
        Self::joint_limit_fmt(|b| {
            let _ = core::fmt::Write::write_str(b, msg);
        })
    }

    pub(crate) fn from_tracking_reject(err: TrackingReject) -> Self {
        match err {
            TrackingReject::JointLimit(msg) => Self::joint_limit_str(&msg),
            TrackingReject::Workspace(_) => Self::Workspace(crate::chain::WorkspaceFailureRt {
                kind: crate::chain::WorkspaceFailureKindRt::Invalid("workspace violation"),
                phase: "tracking_ramp_out",
            }),
        }
    }
}

/// Zero-allocation stack-array validated fallback step for RT ticks (`dof <= MAX_MOTION_DOF`).
/// Depth slices are staged in `ws_eval_bufs.tracking_staged_ws[..len]`.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone)]
pub(crate) struct ValidatedFallbackInline {
    pub positions: [f32; crate::model::MAX_MOTION_DOF],
    pub velocities: [f32; crate::model::MAX_MOTION_DOF],
    pub dof: usize,
    pub ws_min_depths_rev: Option<(u64, usize)>,
    pub done: bool,
}

/// Zero-allocation pending fallback step for RT ticks (`armd-control`).
/// `other_depths_rev` refers to `ws_eval_bufs.tracking_staged_other[..len]`.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone)]
pub(crate) struct PendingFallbackInline {
    pub step: Result<ValidatedFallbackInline, FallbackStepErr>,
    pub ramp: FallbackRamp,
    pub other_depths_rev: Option<(u64, usize)>,
}

/// Stack-allocated zero-allocation tracking evaluation result for RT ticks (`dof <= MAX_MOTION_DOF`).
/// Depth slices are staged in `ws_eval_bufs.tracking_staged_ws[..len]`.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone)]
pub(crate) struct TrackingOutputInline {
    pub positions: [f32; crate::model::MAX_MOTION_DOF],
    pub velocities: [f32; crate::model::MAX_MOTION_DOF],
    pub dof: usize,
    pub multiplier: f32,
    pub withdrawn: bool,
    pub sweep_evals_used: usize,
    pub ws_min_depths_rev: Option<(u64, usize)>,
    pub ws_keepout_min_depths_rev: Option<(u64, usize)>,
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
    /// 1: the shaped lead toward its target; 0: the shaped withdrawal toward zero lead.
    pub multiplier: f32,
    /// True when the lead candidate failed validation and this is its withdrawal.
    pub withdrawn: bool,
    pub sweep_evals_used: usize,
    pub ws_min_depths: Option<(u64, crate::kinematics::SlotBuf)>,
    pub ws_keepout_min_depths: Option<(u64, crate::kinematics::SlotBuf)>,
}

#[cfg(not(target_arch = "wasm32"))]
impl crate::supervisor::SupervisorInner {
    /// Receive times of the oldest and newest joint feedback sample, when every joint has one
    /// (`ArmDriver::feedback_instants`: the driver's own receive instants, not now - age).
    fn feedback_instant_range(&self) -> Option<(std::time::Instant, std::time::Instant)> {
        self.feedback_instant_range_with_driver(None)
    }

    fn feedback_instant_range_with_driver(
        &self,
        drv: Option<&dyn crate::driver::ArmDriver>,
    ) -> Option<(std::time::Instant, std::time::Instant)> {
        if drv.is_none() && self.driver.is_none() {
            return None;
        }
        // R1: Read the owner-held feedback instants populated by `poll_hardware_rx` /
        // `step_enable_validate` with zero driver lock and zero heap allocation.
        let instants = self.core.feedback().instants();
        if instants.len() != self.config.dof {
            return None;
        }
        let mut min_t: Option<std::time::Instant> = None;
        let mut max_t: Option<std::time::Instant> = None;
        for opt in instants {
            let t = (*opt)?;
            min_t = Some(min_t.map_or(t, |m| m.min(t)));
            max_t = Some(max_t.map_or(t, |m| m.max(t)));
        }
        Some((min_t?, max_t?))
    }

    /// Positive evidence that the arm joints are at rest, from fresh feedback: measured
    /// velocities within [`STATIONARY_VELOCITY_RAD_S`] when the driver reports velocity,
    /// otherwise two position samples within [`STATIONARY_POSITION_TOL_RAD`] whose feedback
    /// timestamps lie at least one control period (`1 / control_frequency_hz`) apart: the
    /// oldest joint sample of the second is that much newer than the newest of the first.
    pub fn tracking_stationary_evidence(&self) -> bool {
        self.tracking_stationary_evidence_with_driver(None)
    }

    /// Like [`Self::tracking_stationary_evidence`], accepting an externally borrowed `drv`
    /// when called from inside `with_driver_and_owner_ref` (where `self.driver` is temporarily `None`).
    pub(crate) fn tracking_stationary_evidence_with_driver(
        &self,
        drv: Option<&dyn crate::driver::ArmDriver>,
    ) -> bool {
        let dof = self.config.dof;
        let fresh = match drv {
            Some(d) => {
                self.feedback_numeric_valid
                    && d.feedback_all_within(dof, std::time::Duration::from_millis(100))
            }
            None => self.hardware_feedback_fresh(std::time::Duration::from_millis(100)),
        };
        if !fresh
            || self.measured_positions.len() != dof
            || !self.measured_positions.iter().all(|q| q.is_finite())
        {
            return false;
        }
        let arm = self.config.arm_joints();
        let velocity_feedback = match drv {
            Some(d) => d.capabilities().velocity_feedback,
            None => self
                .driver
                .as_ref()
                .is_some_and(|d| d.with_ref(|drv| drv.capabilities().velocity_feedback).unwrap_or(false)),
        };
        if velocity_feedback {
            return arm.iter().all(|&j| {
                self.measured_velocities
                    .get(j)
                    .is_some_and(|v| v.is_finite() && v.abs() <= STATIONARY_VELOCITY_RAD_S)
            });
        }
        let (Some((sampled_oldest, sampled_newest, q_prev)), Some((oldest_now, newest_now))) = (
            self.tracking.position_sample.as_ref(),
            self.feedback_instant_range_with_driver(drv),
        ) else {
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
            self.tracking.clear_position_sample();
            return;
        }
        let arm = self.config.arm_joints();
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
            match range {
                Some((oldest, newest)) => {
                    self.tracking
                        .set_position_sample(oldest, newest, &self.measured_positions);
                }
                None => {
                    self.tracking.clear_position_sample();
                }
            }
        }
    }

    /// Zero-allocation form of [`Self::active_tracking_limits`] filling preallocated `limits` in place.
    pub fn active_tracking_limits_into(
        &self,
        dt: f64,
        time_scale: f64,
        limits: &mut Vec<crate::model::JointLimit>,
    ) {
        let gripper_idx = self.config.gripper_index();
        if self.servo_target.is_some() || self.servo_stream_active {
            self.servo_limits_into(limits);
        } else {
            crate::rt_core::scratch::copy_limits(limits, &self.config.limits);
            let scale = self
                .config
                .speed_cap
                .and_then(|c| c.joint_velocity_scale)
                .unwrap_or(1.0);
            for (i, limit) in limits.iter_mut().enumerate() {
                if Some(i) != gripper_idx {
                    if let Some(cap) = self.config.planned_max_velocity {
                        limit.max_velocity = limit.max_velocity.min(cap);
                    }
                    limit.max_velocity *= scale;
                }
            }
            let caps = match self.active_trajectory.as_ref() {
                Some(a) => a.motion_limits.as_ref(),
                None => self.tracking.retained_motion_limits.as_ref(),
            };
            if let Some(mlim) = caps.filter(|v| v.len() == limits.len()) {
                for (l, m) in limits.iter_mut().zip(mlim.iter()) {
                    l.max_velocity = l.max_velocity.min(m.max_velocity);
                    l.max_acceleration = l.max_acceleration.min(m.max_acceleration);
                }
            } else if let Some((vcaps, dof)) = self
                .active_trajectory
                .as_ref()
                .and_then(|a| a.motion_vel_caps)
                .or(self.tracking.retained_motion_vel_caps)
            {
                if dof == limits.len() {
                    for (l, &v) in limits.iter_mut().zip(&vcaps[..dof]) {
                        l.max_velocity = l.max_velocity.min(v);
                    }
                }
            } else if let Some(max_v) = self
                .active_trajectory
                .as_ref()
                .and_then(|a| a.motion_max_velocity)
            {
                for (i, l) in limits.iter_mut().enumerate().take(max_v.len()) {
                    if max_v[i].is_finite() && max_v[i] > 0.0 {
                        l.max_velocity = l.max_velocity.min(max_v[i]);
                    }
                }
            }
        }
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
    }

    /// Applicable per-joint output limits for the active motion (planned speed cap,
    /// servo/policy speed cap, and power-governor time scaling). A finished planned motion's
    /// caps apply until it has settled (`TrackingRuntime::retained_motion_limits`).
    pub fn active_tracking_limits(&self, dt: f64, time_scale: f64) -> Vec<crate::model::JointLimit> {
        let mut limits = Vec::with_capacity(self.config.limits.len());
        self.active_tracking_limits_into(dt, time_scale, &mut limits);
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

    /// Compute and validate the combined joint vector for position-lead tracking against the
    /// motion caps `active_limits`.
    ///
    /// Pure read-only evaluation (`&self`): never mutates live escape bookkeeping
    /// (`servo_ws_min_depths` or `tracking.ws_min_depths`) or nominal state during candidate checks.
    ///
    /// Limits: `active_limits` (the motion's caps) bound the NOMINAL reference `q_d`; a nominal
    /// step above them is rejected. The issued output `q_d + lead` of a lead joint is bound by
    /// `|v_out| <= min(v_phys, v_cap + lag * a_cap)`, `|a_out| <= a_phys` (profile joint limits),
    /// `|lead| <= max_lead_rad` and `|lead_k - lead_{k-1}| <= lag * a_cap * dt`; joints without
    /// lead keep the motion caps. The lead is shaped (second order, braking at `a_phys / 2`)
    /// toward `lag * v_d`, so it stays continuous at motion start, stop, reversal and saturation.
    ///
    /// Candidates: the shaped lead (`multiplier` 1), then the shaped withdrawal toward zero lead
    /// under the same bounds (`multiplier` 0). A lead-only failure therefore withdraws the lead
    /// instead of failing the motion; an error is returned only for a nominal violation or when
    /// the withdrawal itself fails validation.
    ///
    /// Segment contracts:
    /// - Prev-issued segment (`prev_q -> q_cand`): joint position limits, the output velocity
    ///   and acceleration limits above, and workspace/keep-out sweep using revision-tagged
    ///   persistent escape depths.
    /// - Measured segment (`q_m -> q_cand`, when feedback is fresh): swept workspace, keep-outs,
    ///   link capsules and self-collision along the physical chord from `q_m` to `q_cand`, plus
    ///   the streaming directional `servo_max_lead_rad` allowance (every candidate).
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

    fn tracking_reject_rt_to_alloc(
        &self,
        err: TrackingRejectRt,
        bufs: &mut crate::chain::EvalBufs,
    ) -> TrackingReject {
        match err {
            TrackingRejectRt::JointLimit(msg) => TrackingReject::JointLimit(msg.as_str().to_string()),
            TrackingRejectRt::Workspace(f) => {
                let mut detail = crate::kinematics::WorkspaceViolationDetail::preallocated_rt();
                if let Some((ws, cam)) = self.workspace_policy() {
                    let eval = crate::chain::WorkspaceEvaluator::new_with_tcp_in(
                        &self.config.arm_model,
                        ws,
                        cam,
                        self.config.tcp_offset,
                        bufs,
                    );
                    eval.write_failure_detail_rt(&f, &mut detail);
                    eval.recycle(bufs);
                } else {
                    detail.constraint_id = Some(f.code().to_string());
                    detail.violation_phase = Some(f.phase.to_string());
                }
                TrackingReject::Workspace(crate::safety::SafetyError::WorkspaceViolation {
                    point: 0,
                    limit: f.code().to_string(),
                    value: 0.0,
                    bound: 0.0,
                    detail: Box::new(detail),
                })
            }
        }
    }

    pub fn compute_position_lead_output_with_limits_and_budget(
        &self,
        dt: f64,
        jt: &JointTrackingConfig,
        active_limits: &[crate::model::JointLimit],
        budget: &mut usize,
    ) -> Result<TrackingOutput, TrackingReject> {
        let mut bufs = crate::chain::EvalBufs::default();
        let out = match self.compute_position_lead_output_inline_with_bufs(
            dt,
            jt,
            false,
            active_limits,
            budget,
            &mut bufs,
        ) {
            Ok(out) => out,
            Err(err) => return Err(self.tracking_reject_rt_to_alloc(err, &mut bufs)),
        };
        let ws_min_depths = out.ws_min_depths_rev.map(|(rev, n)| {
            let mut sb = crate::kinematics::SlotBuf::zeros(n);
            sb.copy_from_slice_in_place(&bufs.tracking_staged_ws[..n]);
            (rev, sb)
        });
        let ws_keepout_min_depths = out.ws_keepout_min_depths_rev.map(|(rev, n)| {
            let mut sb = crate::kinematics::SlotBuf::zeros(n);
            sb.copy_from_slice_in_place(&bufs.tracking_staged_ws[..n]);
            (rev, sb)
        });
        Ok(TrackingOutput {
            positions: out.positions[..out.dof].to_vec(),
            velocities: out.velocities[..out.dof].to_vec(),
            multiplier: out.multiplier,
            withdrawn: out.withdrawn,
            sweep_evals_used: out.sweep_evals_used,
            ws_min_depths,
            ws_keepout_min_depths,
        })
    }

    /// Zero-allocation stack-array evaluation of position-lead tracking (`dof <= MAX_DOF`).
    /// Reuses `self.ws_eval_bufs` (`WorkspaceEvaluator::new_with_tcp_in` + `sweep_linear_step_rt`)
    /// so PositionLead / Position tracking, lead withdrawal, and rejection never allocate on `armd-control`.
    pub(crate) fn compute_position_lead_output_inline(
        &mut self,
        dt: f64,
        zero_lead: bool,
        active_limits: &[crate::model::JointLimit],
        budget: &mut usize,
    ) -> Result<TrackingOutputInline, TrackingRejectRt> {
        let Some(jt) = self.config.joint_tracking.as_ref() else {
            return Err(TrackingRejectRt::joint_limit_str("joint tracking config missing"));
        };
        let mut bufs = std::mem::take(&mut self.ws_eval_bufs);
        let res = self.compute_position_lead_output_inline_with_bufs(
            dt,
            jt,
            zero_lead,
            active_limits,
            budget,
            &mut bufs,
        );
        self.ws_eval_bufs = bufs;
        res
    }

    fn compute_position_lead_output_inline_with_bufs(
        &self,
        dt: f64,
        jt: &JointTrackingConfig,
        zero_lead: bool,
        active_limits: &[crate::model::JointLimit],
        budget: &mut usize,
        bufs: &mut crate::chain::EvalBufs,
    ) -> Result<TrackingOutputInline, TrackingRejectRt> {
        use std::time::Duration;
        use crate::chain::WorkspaceEvaluator;
        use crate::model::MAX_MOTION_DOF as MAX_DOF;
        use crate::supervisor::MAX_KEEPOUT_EVALS_PER_TICK;

        let dof = self.config.dof;
        if dof > MAX_DOF
            || !dt.is_finite()
            || dt <= 0.0
            || active_limits.len() != dof
            || self.commanded_positions.len() != dof
            || self.commanded_velocities.len() != dof
            || !self.commanded_positions.iter().all(|q| q.is_finite())
        {
            return Err(TrackingRejectRt::joint_limit_str(
                "invalid nominal state, limits, or dt for tracking",
            ));
        }

        let arm_indices = self.config.arm_joints();
        if arm_indices.len() != jt.joints.len() {
            return Err(TrackingRejectRt::joint_limit_fmt(|b| {
                let _ = core::fmt::write(
                    b,
                    format_args!(
                        "tracking joint count {} does not match arm joint count {}",
                        jt.joints.len(),
                        arm_indices.len()
                    ),
                );
            }));
        }

        let fresh_meas = self.hardware_feedback_fresh(Duration::from_millis(100))
            && self.measured_positions.len() == dof
            && self.measured_positions.iter().all(|q| q.is_finite());
        let in_streaming = self.servo_target.is_some() || self.servo_stream_active;
        let in_hold = self.collision.hold.is_some() || self.servo_blocked_hold;

        // 1. Per-joint base position lead (arm joints only; gripper is strictly bypassed).
        let mut base_lead = [0.0_f32; MAX_DOF];
        for (k, &joint_idx) in arm_indices.iter().enumerate() {
            let jc = &jt.joints[k];
            if zero_lead || jc.strategy != TrackingStrategy::PositionLead || in_hold || !fresh_meas {
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

            if let Some(h) = self.torque_guard.hold {
                if h.joint == joint_idx && lead * h.sign > 0.0 {
                    lead = 0.0;
                }
            }

            base_lead[joint_idx] = lead;
        }

        let has_desired_lead = base_lead[..dof].iter().any(|&l| l != 0.0);

        // 2. Resolve output history; unknown history NEVER bypasses validation.
        let mut seeded_q = [0.0_f32; MAX_DOF];
        let mut prev_q_arr = [0.0_f32; MAX_DOF];
        let mut prev_v_arr = [0.0_f32; MAX_DOF];
        let has_prev_v: bool;
        match self.tracking.history_state {
            TrackingHistoryState::Stationary | TrackingHistoryState::Issued => {
                let Some(pq) = self.tracking.last_issued_positions.as_ref().filter(|v| v.len() == dof) else {
                    return Err(TrackingRejectRt::joint_limit_str("tracking output history missing positions"));
                };
                prev_q_arr[..dof].copy_from_slice(pq);
                if let Some(pv) = self.tracking.last_issued_velocities.as_ref().filter(|v| v.len() == dof) {
                    prev_v_arr[..dof].copy_from_slice(pv);
                    has_prev_v = true;
                } else {
                    has_prev_v = false;
                }
            }
            TrackingHistoryState::Unknown => {
                if !fresh_meas {
                    return Err(TrackingRejectRt::joint_limit_str(
                        "tracking output history is unknown without fresh feedback",
                    ));
                }
                if !self.tracking_stationary_evidence() {
                    return Err(TrackingRejectRt::joint_limit_str(
                        "tracking output history is unknown and the arm is not confirmed at rest",
                    ));
                }
                for i in 0..dof {
                    seeded_q[i] = if arm_indices.contains(&i) {
                        self.measured_positions[i]
                    } else {
                        self.commanded_positions[i]
                    };
                }
                prev_q_arr[..dof].copy_from_slice(&seeded_q[..dof]);
                has_prev_v = false;
            }
        }
        let prev_q: &[f32] = &prev_q_arr[..dof];
        let prev_v: Option<&[f32]> = has_prev_v.then_some(&prev_v_arr[..dof]);

        // 3. Nominal reference check (stack-only arrays, 0 heap allocs).
        let dtf = dt;
        let unknown = self.tracking.history_state == TrackingHistoryState::Unknown;
        let mut nom_q_prev = [0.0_f64; MAX_DOF];
        let mut nom_v_prev = [0.0_f64; MAX_DOF];
        match (&self.tracking.last_nominal, unknown) {
            (Some((nq, nv)), false) if nq.len() == dof && nv.len() == dof => {
                for i in 0..dof {
                    nom_q_prev[i] = f64::from(nq[i]);
                    nom_v_prev[i] = f64::from(nv[i]);
                }
            }
            (_, true) => {
                for i in 0..dof {
                    nom_q_prev[i] = f64::from(prev_q[i]);
                }
                for (k, &j) in arm_indices.iter().enumerate() {
                    let jc = &jt.joints[k];
                    let ref_prev = f64::from(self.commanded_positions[j]) - f64::from(self.commanded_velocities[j]) * dtf;
                    if !zero_lead
                        && jc.strategy == TrackingStrategy::PositionLead
                        && jc.lag_s > 0.0
                        && (nom_q_prev[j] - ref_prev).abs() <= f64::from(jc.max_lead_rad)
                    {
                        nom_q_prev[j] = ref_prev;
                        nom_v_prev[j] = f64::from(self.commanded_velocities[j]);
                    }
                }
            }
            _ => {
                for i in 0..dof {
                    nom_q_prev[i] = f64::from(prev_q[i]);
                    nom_v_prev[i] = prev_v.map_or(0.0, |pv| f64::from(pv[i]));
                }
            }
        }
        let mut v_nom = [0.0_f64; MAX_DOF];
        let mut a_nom = [0.0_f64; MAX_DOF];
        for &j in arm_indices.iter() {
            let lim = &active_limits[j];
            v_nom[j] = (f64::from(self.commanded_positions[j]) - nom_q_prev[j]) / dtf;
            a_nom[j] = (v_nom[j] - nom_v_prev[j]) / dtf;
            if !v_nom[j].is_finite() || v_nom[j].abs() > f64::from(lim.max_velocity) + 1e-4 {
                return Err(TrackingRejectRt::joint_limit_fmt(|b| {
                    let _ = core::fmt::write(
                        b,
                        format_args!(
                            "joint {} nominal velocity {:.3} rad/s exceeds limit {:.3}",
                            j + 1,
                            v_nom[j],
                            lim.max_velocity
                        ),
                    );
                }));
            }
            if !a_nom[j].is_finite() || a_nom[j].abs() > f64::from(lim.max_acceleration) + 1e-3 {
                return Err(TrackingRejectRt::joint_limit_fmt(|b| {
                    let _ = core::fmt::write(
                        b,
                        format_args!(
                            "joint {} nominal acceleration {:.3} rad/s^2 exceeds limit {:.3}",
                            j + 1,
                            a_nom[j],
                            lim.max_acceleration
                        ),
                    );
                }));
            }
        }

        // 4. Output (issued) limits and lead window per arm joint.
        let mut out_lim = [(0.0_f64, 0.0_f64); MAX_DOF];
        let mut window = [(0.0_f64, 0.0_f64); MAX_DOF];
        let mut lead_prev = [0.0_f64; MAX_DOF];
        for (k, &j) in arm_indices.iter().enumerate() {
            let jc = &jt.joints[k];
            let cap = &active_limits[j];
            let phys = &self.config.limits[j];
            let lag = if !zero_lead && jc.strategy == TrackingStrategy::PositionLead { f64::from(jc.lag_s) } else { 0.0 };
            let (v_lim, a_lim) = if lag > 0.0 {
                (
                    f64::from(phys.max_velocity).min(f64::from(cap.max_velocity) + lag * f64::from(cap.max_acceleration)),
                    f64::from(phys.max_acceleration),
                )
            } else {
                (f64::from(cap.max_velocity), f64::from(cap.max_acceleration))
            };
            out_lim[j] = (v_lim, a_lim);
            let qd = f64::from(self.commanded_positions[j]);
            let qp = f64::from(prev_q[j]);
            let vp = prev_v.map_or(0.0, |pv| f64::from(pv[j]));
            lead_prev[j] = qp - nom_q_prev[j];
            let base = qp - qd;
            let mut lo = (base - v_lim * dtf).max(base + (vp - a_lim * dtf) * dtf);
            let mut hi = (base + v_lim * dtf).min(base + (vp + a_lim * dtf) * dtf);
            if lag > 0.0 {
                let step = lag * f64::from(cap.max_acceleration) * dtf;
                lo = lo.max(lead_prev[j] - step);
                hi = hi.min(lead_prev[j] + step);
            }
            window[j] = (lo, hi);
        }

        type Bound = (f64, f64);
        let mut ups_arr = [[(0.0_f64, 0.0_f64); 2]; MAX_DOF];
        let mut downs_arr = [[(0.0_f64, 0.0_f64); 2]; MAX_DOF];
        let mut bound_len = [0usize; MAX_DOF];
        for (k, &j) in arm_indices.iter().enumerate() {
            let b = f64::from(jt.joints[k].max_lead_rad).max(lead_prev[j].abs());
            ups_arr[j][0] = (b, 0.0);
            downs_arr[j][0] = (-b, 0.0);
            let mut n = 1usize;
            if in_streaming && fresh_meas {
                let sl = f64::from(self.config.servo_max_lead_rad);
                let meas = f64::from(self.measured_positions[j]);
                let qd = f64::from(self.commanded_positions[j]);
                let vm = self.measured_velocities.get(j).copied().filter(|v| v.is_finite()).map_or(0.0, f64::from);
                let rate = vm - v_nom[j];
                ups_arr[j][1] = ((meas + sl - qd).max(0.0), rate);
                downs_arr[j][1] = ((meas - sl - qd).min(0.0), rate);
                n = 2;
            }
            bound_len[j] = n;
        }

        let shaped = |target: &[f32]| -> [f32; MAX_DOF] {
            let mut q_cand = [0.0_f32; MAX_DOF];
            q_cand[..dof].copy_from_slice(&self.commanded_positions[..dof]);
            for &j in arm_indices.iter() {
                let l0 = lead_prev[j];
                let brake = 0.5 * out_lim[j].1;
                let n = bound_len[j];
                let ups: &[Bound] = &ups_arr[j][..n];
                let downs: &[Bound] = &downs_arr[j][..n];
                let b_hi = ups.iter().map(|b| b.0).fold(f64::INFINITY, f64::min);
                let b_lo = downs.iter().map(|b| b.0).fold(f64::NEG_INFINITY, f64::max);
                let t = f64::from(target[j]).clamp(b_lo.min(b_hi), b_hi);
                let e = t - l0;
                let mut r = e.signum() * (e.abs() / dtf).min((2.0 * brake * e.abs()).sqrt());
                for &(hi, rate) in ups {
                    r = r.min(rate + (2.0 * brake * (hi - l0).max(0.0)).sqrt());
                }
                for &(lo, rate) in downs {
                    r = r.max(rate - (2.0 * brake * (l0 - lo).max(0.0)).sqrt());
                }
                let (lo, hi) = window[j];
                let mut l = l0 + r * dtf;
                l = if lo <= hi { l.clamp(lo, hi) } else { 0.5 * (lo + hi) };
                l = l.min(b_hi).max(b_lo.min(b_hi));
                let lim = &self.config.limits[j];
                q_cand[j] = ((f64::from(self.commanded_positions[j]) + l) as f32).clamp(lim.min_position, lim.max_position);
            }
            q_cand
        };

        let zero_target = [0.0_f32; MAX_DOF];
        let mut candidates = [(0.0_f32, [0.0_f32; MAX_DOF]); 2];
        let mut cand_len = 0usize;
        if has_desired_lead {
            candidates[cand_len] = (1.0, shaped(&base_lead[..dof]));
            cand_len += 1;
        }
        candidates[cand_len] = (0.0, shaped(&zero_target[..dof]));
        cand_len += 1;
        let mut last_err: Option<TrackingRejectRt> = None;
        let budget_start = *budget;
        let lead_tried = cand_len > 1;

        // 5. Evaluate candidates reusing caller's `bufs` (`self.ws_eval_bufs`, 0 heap allocs on RT).
        let mut ws_eval = if let Some((ws, cam)) = self.workspace_policy() {
            let mut eval = WorkspaceEvaluator::new_with_tcp_in(
                &self.config.arm_model,
                ws,
                cam,
                self.config.tcp_offset,
                bufs,
            );
            let n = eval.total_slots;
            if bufs.scratch_c0.len() < n
                || bufs.scratch_c1.len() < n
                || bufs.scratch_cm.len() < n
                || bufs.scratch_cand_min_d.len() < n
                || bufs.tracking_staged_ws.len() < n
                || bufs.geom_bounds.len() < eval.num_geoms
                || bufs.sweep_slab.len() < 2 * n
            {
                eval.recycle(bufs);
                bufs.prepare_for(&self.config.arm_model, ws, cam, self.config.tcp_offset);
                eval = WorkspaceEvaluator::new_with_tcp_in(
                    &self.config.arm_model,
                    ws,
                    cam,
                    self.config.tcp_offset,
                    bufs,
                );
            }
            if let Err(f) = eval.eval_into_slice_rt(prev_q, &mut bufs.scratch_c0, &mut bufs.world_caps, "tracking_lead") {
                eval.recycle(bufs);
                return Err(TrackingRejectRt::Workspace(f));
            }
            Some(eval)
        } else {
            None
        };

        let eval_res = (|| {
            'cand: for (cand_idx, &(m, q_cand)) in candidates[..cand_len].iter().enumerate() {
                let is_final_cand = cand_idx + 1 == cand_len;
                for &j in arm_indices.iter() {
                    let lim = &active_limits[j];
                    let q = q_cand[j];
                    if !q.is_finite() || q < lim.min_position - 1e-5 || q > lim.max_position + 1e-5 {
                        if is_final_cand {
                            last_err = Some(TrackingRejectRt::joint_limit_fmt(|b| {
                                let _ = core::fmt::write(
                                    b,
                                    format_args!(
                                        "joint {} candidate {q:.4} rad outside [{:.4}, {:.4}]",
                                        j + 1,
                                        lim.min_position,
                                        lim.max_position
                                    ),
                                );
                            }));
                        }
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
                            if is_final_cand {
                                last_err = Some(TrackingRejectRt::joint_limit_fmt(|b| {
                                    let _ = core::fmt::write(
                                        b,
                                        format_args!(
                                            "joint {} candidate {q:.4} rad exceeds streaming measured-lead allowance around {meas:.4}",
                                            j + 1
                                        ),
                                    );
                                }));
                            }
                            continue 'cand;
                        }
                    }
                    if let Some(h) = self.torque_guard.hold {
                        if h.joint == j {
                            let qd = self.commanded_positions[j];
                            if (q - qd) * h.sign > 1e-6
                                || ((qd - h.hold_q) * h.sign <= 1e-6 && (q - h.hold_q) * h.sign > 1e-5)
                            {
                                if is_final_cand {
                                    last_err = Some(TrackingRejectRt::joint_limit_fmt(|b| {
                                        let _ = core::fmt::write(
                                            b,
                                            format_args!(
                                                "joint {} candidate {q:.4} rad advances into torque hold at {:.4}",
                                                j + 1,
                                                h.hold_q
                                            ),
                                        );
                                    }));
                                }
                                continue 'cand;
                            }
                        }
                    }
                    let (v_lim_out, a_lim_out) = out_lim[j];
                    let v_cand = (f64::from(q) - f64::from(prev_q[j])) / dt;
                    let v_prev_j = prev_v.map_or(0.0, |pv| f64::from(pv[j]));
                    let a_cand = (v_cand - v_prev_j) / dt;
                    if !v_cand.is_finite() || v_cand.abs() > v_lim_out + 1e-4 {
                        if is_final_cand {
                            last_err = Some(TrackingRejectRt::joint_limit_fmt(|b| {
                                let _ = core::fmt::write(
                                    b,
                                    format_args!(
                                        "joint {} output velocity {v_cand:.3} rad/s exceeds limit {v_lim_out:.3}",
                                        j + 1
                                    ),
                                );
                            }));
                        }
                        continue 'cand;
                    }
                    if !a_cand.is_finite() || a_cand.abs() > a_lim_out + 1e-3 {
                        if is_final_cand {
                            last_err = Some(TrackingRejectRt::joint_limit_fmt(|b| {
                                let _ = core::fmt::write(
                                    b,
                                    format_args!(
                                        "joint {} output acceleration {a_cand:.3} rad/s^2 exceeds limit {a_lim_out:.3}",
                                        j + 1
                                    ),
                                );
                            }));
                        }
                        continue 'cand;
                    }
                }

                let mut committed_ws_min_d = None;
                if let Some(eval) = ws_eval.as_ref() {
                    let n = eval.total_slots;
                    if let Err(f) = eval.eval_into_slice_rt(prev_q, &mut bufs.scratch_c0, &mut bufs.world_caps, "tracking_lead") {
                        if is_final_cand {
                            last_err = Some(TrackingRejectRt::Workspace(f));
                        }
                        continue 'cand;
                    }
                    match &self.tracking.ws_min_depths {
                        Some((rev, min_d)) if *rev == self.workspace_revision && min_d.as_slice().len() == n => {
                            bufs.scratch_cand_min_d[..n].copy_from_slice(min_d.as_slice());
                        }
                        _ => eval.init_min_depths_slice(&bufs.scratch_c0, &mut bufs.scratch_cand_min_d),
                    }
                    if let Err(f) = eval.sweep_linear_step_rt(
                        prev_q,
                        &q_cand[..dof],
                        &bufs.scratch_c0,
                        &mut bufs.scratch_cand_min_d,
                        budget,
                        MAX_KEEPOUT_EVALS_PER_TICK,
                        "tracking_lead",
                        "tracking_lead",
                        &mut bufs.scratch_c1,
                        &mut bufs.scratch_cm,
                        &mut bufs.sweep_slab,
                        &mut bufs.sweep_stack,
                        &mut bufs.sweep_free,
                        &mut bufs.geom_bounds,
                        &mut bufs.world_caps,
                    ) {
                        if is_final_cand {
                            last_err = Some(TrackingRejectRt::Workspace(f));
                        }
                        continue 'cand;
                    }
                    bufs.tracking_staged_ws[..n].copy_from_slice(&bufs.scratch_cand_min_d[..n]);
                    if fresh_meas {
                        if let Err(f) = eval.eval_into_slice_rt(&self.measured_positions, &mut bufs.scratch_c0, &mut bufs.world_caps, "tracking_lead") {
                            if is_final_cand {
                                last_err = Some(TrackingRejectRt::Workspace(f));
                            }
                            continue 'cand;
                        }
                        eval.init_min_depths_slice(&bufs.scratch_c0, &mut bufs.scratch_cand_min_d);
                        if let Err(f) = eval.sweep_linear_step_rt(
                            &self.measured_positions,
                            &q_cand[..dof],
                            &bufs.scratch_c0,
                            &mut bufs.scratch_cand_min_d,
                            budget,
                            MAX_KEEPOUT_EVALS_PER_TICK,
                            "tracking_lead",
                            "tracking_lead",
                            &mut bufs.scratch_c1,
                            &mut bufs.scratch_cm,
                            &mut bufs.sweep_slab,
                            &mut bufs.sweep_stack,
                            &mut bufs.sweep_free,
                            &mut bufs.geom_bounds,
                            &mut bufs.world_caps,
                        ) {
                            if is_final_cand {
                                last_err = Some(TrackingRejectRt::Workspace(f));
                            }
                            continue 'cand;
                        }
                    }
                    committed_ws_min_d = Some((self.workspace_revision, n));
                }

                let mut v_out = [0.0_f32; MAX_DOF];
                for i in 0..dof {
                    v_out[i] = ((f64::from(q_cand[i]) - f64::from(prev_q[i])) / dt) as f32;
                }
                return Ok(TrackingOutputInline {
                    positions: q_cand,
                    velocities: v_out,
                    dof,
                    multiplier: m,
                    withdrawn: lead_tried && m == 0.0,
                    sweep_evals_used: budget_start.saturating_sub(*budget),
                    ws_min_depths_rev: committed_ws_min_d,
                    ws_keepout_min_depths_rev: None,
                });
            }

            Err(last_err.unwrap_or_else(|| TrackingRejectRt::joint_limit_str("no valid tracking candidate")))
        })();
        if let Some(eval) = ws_eval.take() {
            eval.recycle(bufs);
        }
        eval_res
    }

    /// Zero-allocation stack variant of [`Self::compute_lead_ramp_out_with_budget`] for RT ticks
    /// (`dof <= MAX_DOF`). Reuses `self.ws_eval_bufs` so normal post-jog lead ramp-out and
    /// rejection never allocate.
    pub(crate) fn compute_lead_ramp_out_inline(
        &mut self,
        dt: f64,
        active_limits: &[crate::model::JointLimit],
        keepout_only: bool,
        budget: &mut usize,
    ) -> Result<TrackingOutputInline, TrackingRejectRt> {
        use std::time::Duration;
        use crate::chain::WorkspaceEvaluator;
        use crate::model::MAX_MOTION_DOF as MAX_DOF;
        use crate::supervisor::MAX_KEEPOUT_EVALS_PER_TICK;

        let dof = self.config.dof;
        if dof > MAX_DOF
            || !dt.is_finite()
            || dt <= 0.0
            || active_limits.len() != dof
            || self.commanded_positions.len() != dof
            || !self.commanded_positions.iter().all(|q| q.is_finite())
        {
            return Err(TrackingRejectRt::joint_limit_str(
                "invalid nominal state or dt for lead ramp-out",
            ));
        }
        let arm_indices = self.config.arm_joints();
        let fresh_meas = self.hardware_feedback_fresh(Duration::from_millis(100))
            && self.measured_positions.len() == dof
            && self.measured_positions.iter().all(|q| q.is_finite());

        let mut prev_q_arr = [0.0_f32; MAX_DOF];
        let mut prev_v_arr = [0.0_f32; MAX_DOF];
        let Some(pq_slice) = self.tracking.last_issued_positions.as_deref().filter(|v| v.len() == dof) else {
            return Err(TrackingRejectRt::joint_limit_str(
                "cannot ramp lead out without prior issued output",
            ));
        };
        prev_q_arr[..dof].copy_from_slice(pq_slice);
        let has_prev_v = if let Some(pv_slice) = self.tracking.last_issued_velocities.as_deref().filter(|v| v.len() == dof) {
            prev_v_arr[..dof].copy_from_slice(pv_slice);
            true
        } else {
            false
        };
        let prev_q: &[f32] = &prev_q_arr[..dof];
        let prev_v: Option<&[f32]> = has_prev_v.then_some(&prev_v_arr[..dof]);

        let mut q_next = [0.0_f32; MAX_DOF];
        q_next[..dof].copy_from_slice(&self.commanded_positions[..dof]);
        for &j in arm_indices.iter() {
            let lim = &active_limits[j];
            let pq = prev_q[j];
            let pv = prev_v.map_or(0.0, |v| v[j]);
            let qd = self.commanded_positions[j].clamp(lim.min_position, lim.max_position);
            let stepped = if (qd - pq).abs() <= 1e-6 && pv.abs() <= lim.max_acceleration * dt as f32 {
                qd
            } else {
                let Some((q_s, _, _)) = Self::bounded_servo_step(pq, qd, pv, dt, lim) else {
                    return Err(TrackingRejectRt::joint_limit_fmt(|b| {
                        let _ = core::fmt::write(
                            b,
                            format_args!("joint {} lead ramp-out infeasible from {pq:.4} toward {qd:.4}", j + 1),
                        );
                    }));
                };
                q_s
            };
            let mut q_final = stepped;
            if let Some(h) = self.torque_guard.hold {
                if h.joint == j {
                    if (q_final - pq) * h.sign > 1e-6 {
                        q_final = pq;
                    }
                    if (pq - h.hold_q) * h.sign <= 1e-6 && (q_final - h.hold_q) * h.sign > 1e-5 {
                        q_final = h.hold_q;
                    }
                    let v_final = (f64::from(q_final) - f64::from(pq)) / dt;
                    let v_prev_j = prev_v.map_or(0.0, |pv| f64::from(pv[j]));
                    let a_final = (v_final - v_prev_j) / dt;
                    if !v_final.is_finite()
                        || v_final.abs() > f64::from(lim.max_velocity) + 1e-4
                        || !a_final.is_finite()
                        || a_final.abs() > f64::from(lim.max_acceleration) + 1e-3
                    {
                        return Err(TrackingRejectRt::joint_limit_fmt(|b| {
                            let _ = core::fmt::write(
                                b,
                                format_args!(
                                    "joint {} torque-hold ramp-out dynamics exceeded (v={v_final:.3}/{:.3}, a={a_final:.3}/{:.3})",
                                    j + 1,
                                    lim.max_velocity,
                                    lim.max_acceleration
                                ),
                            );
                        }));
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
                    return Err(TrackingRejectRt::joint_limit_fmt(|b| {
                        let _ = core::fmt::write(
                            b,
                            format_args!(
                                "joint {} ramp-out candidate {q_final:.4} rad exceeds streaming measured-lead allowance around {meas:.4}",
                                j + 1
                            ),
                        );
                    }));
                }
            }
            q_next[j] = q_final;
        }

        let budget_start = *budget;
        let mut committed_ws_min_d = None;
        let mut committed_keepout_min_d = None;
        let mut bufs = std::mem::take(&mut self.ws_eval_bufs);
        let ws_res: Result<(), TrackingRejectRt> = (|| {
            let Some((ws, cam)) = self.workspace_policy() else {
                return Ok(());
            };
            if keepout_only && ws.keepout_boxes.is_empty() {
                return Ok(());
            }
            let mut eval = WorkspaceEvaluator::new_with_tcp_in_mode(
                &self.config.arm_model,
                ws,
                cam,
                self.config.tcp_offset,
                &mut bufs,
                keepout_only,
            );
            let n = eval.total_slots;
            if bufs.scratch_c0.len() < n
                || bufs.scratch_c1.len() < n
                || bufs.scratch_cm.len() < n
                || bufs.scratch_cand_min_d.len() < n
                || bufs.tracking_staged_ws.len() < n
                || bufs.geom_bounds.len() < eval.num_geoms
                || bufs.sweep_slab.len() < 2 * n
            {
                eval.recycle(&mut bufs);
                bufs.prepare_for(&self.config.arm_model, ws, cam, self.config.tcp_offset);
                eval = WorkspaceEvaluator::new_with_tcp_in_mode(
                    &self.config.arm_model,
                    ws,
                    cam,
                    self.config.tcp_offset,
                    &mut bufs,
                    keepout_only,
                );
            }
            let inner_res = (|| {
                eval.eval_into_slice_rt(prev_q, &mut bufs.scratch_c0, &mut bufs.world_caps, "tracking_ramp_out")
                    .map_err(TrackingRejectRt::Workspace)?;
                let cached_depths = if keepout_only {
                    &self.tracking.ws_keepout_min_depths
                } else {
                    &self.tracking.ws_min_depths
                };
                match cached_depths {
                    Some((rev, md)) if *rev == self.workspace_revision && md.as_slice().len() == n => {
                        bufs.scratch_cand_min_d[..n].copy_from_slice(md.as_slice());
                    }
                    _ => eval.init_min_depths_slice(&bufs.scratch_c0, &mut bufs.scratch_cand_min_d),
                }
                eval.sweep_linear_step_rt(
                    prev_q,
                    &q_next[..dof],
                    &bufs.scratch_c0,
                    &mut bufs.scratch_cand_min_d,
                    budget,
                    MAX_KEEPOUT_EVALS_PER_TICK,
                    "tracking_ramp_out",
                    "tracking_ramp_out",
                    &mut bufs.scratch_c1,
                    &mut bufs.scratch_cm,
                    &mut bufs.sweep_slab,
                    &mut bufs.sweep_stack,
                    &mut bufs.sweep_free,
                    &mut bufs.geom_bounds,
                    &mut bufs.world_caps,
                )
                .map_err(TrackingRejectRt::Workspace)?;
                bufs.tracking_staged_ws[..n].copy_from_slice(&bufs.scratch_cand_min_d[..n]);
                if fresh_meas {
                    eval.eval_into_slice_rt(&self.measured_positions, &mut bufs.scratch_c0, &mut bufs.world_caps, "tracking_ramp_out")
                        .map_err(TrackingRejectRt::Workspace)?;
                    eval.init_min_depths_slice(&bufs.scratch_c0, &mut bufs.scratch_cand_min_d);
                    eval.sweep_linear_step_rt(
                        &self.measured_positions,
                        &q_next[..dof],
                        &bufs.scratch_c0,
                        &mut bufs.scratch_cand_min_d,
                        budget,
                        MAX_KEEPOUT_EVALS_PER_TICK,
                        "tracking_ramp_out",
                        "tracking_ramp_out",
                        &mut bufs.scratch_c1,
                        &mut bufs.scratch_cm,
                        &mut bufs.sweep_slab,
                        &mut bufs.sweep_stack,
                        &mut bufs.sweep_free,
                        &mut bufs.geom_bounds,
                        &mut bufs.world_caps,
                    )
                    .map_err(TrackingRejectRt::Workspace)?;
                }
                if keepout_only {
                    committed_keepout_min_d = Some((self.workspace_revision, n));
                } else {
                    committed_ws_min_d = Some((self.workspace_revision, n));
                }
                Ok(())
            })();
            eval.recycle(&mut bufs);
            inner_res
        })();
        self.ws_eval_bufs = bufs;
        ws_res?;
        let mut v_out = [0.0_f32; MAX_DOF];
        for i in 0..dof {
            v_out[i] = ((f64::from(q_next[i]) - f64::from(prev_q[i])) / dt) as f32;
        }
        Ok(TrackingOutputInline {
            positions: q_next,
            velocities: v_out,
            dof,
            multiplier: 0.0,
            withdrawn: false,
            sweep_evals_used: budget_start.saturating_sub(*budget),
            ws_min_depths_rev: committed_ws_min_d,
            ws_keepout_min_depths_rev: committed_keepout_min_d,
        })
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

        let arm_indices = self.config.arm_joints();
        let fresh_meas = self.hardware_feedback_fresh(Duration::from_millis(100))
            && self.measured_positions.len() == dof
            && self.measured_positions.iter().all(|q| q.is_finite());

        let Some(prev_q) = self.tracking.last_issued_positions.as_deref().filter(|v| v.len() == dof) else {
            return Err(TrackingReject::JointLimit("cannot ramp lead out without prior issued output".into()));
        };
        let prev_v = self.tracking.last_issued_velocities.as_deref().filter(|v| v.len() == dof);

        let mut q_next = self.commanded_positions.clone();
        for &j in arm_indices.iter() {
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
                let eval = WorkspaceEvaluator::new_with_tcp(&self.config.arm_model, ws_ref, cam, self.config.tcp_offset);
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
            withdrawn: false,
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
        for &j in arm_indices.iter() {
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
            for &j in arm_indices.iter() {
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
            for &j in arm_indices.iter() {
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
    /// selects. Captures `last_issued_positions` into `scratch.fallback_prev_confirmed` (0 alloc)
    /// before any hold or fault invalidates the history.
    pub fn tracking_fallback_compute(&mut self, ramp: FallbackRamp, dt: f64, budget: &mut usize) -> PendingFallback {
        if let Some(prev) = self.tracking.last_issued_positions.as_deref() {
            crate::rt_core::scratch::copy_into(&mut self.scratch.fallback_prev_confirmed, prev);
            self.scratch.fallback_prev_confirmed_valid = true;
        } else {
            self.scratch.fallback_prev_confirmed.clear();
            self.scratch.fallback_prev_confirmed_valid = false;
        }
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
            other_depths: other_depths.clone(),
        }
    }

    /// Zero-allocation RT equivalent of [`Self::tracking_fallback_compute`]: evaluates the
    /// rejection-fallback step using stack `[f32; MAX_DOF]` arrays, [`FallbackStepErr`], and
    /// `self.ws_eval_bufs` (`WorkspaceEvaluator::new_with_tcp_in_mode` + `sweep_linear_step_rt`).
    pub(crate) fn tracking_fallback_compute_inline(
        &mut self,
        ramp: FallbackRamp,
        dt: f64,
        budget: &mut usize,
    ) -> PendingFallbackInline {
        use crate::model::MAX_MOTION_DOF as MAX_DOF;

        if let Some(prev) = self.tracking.last_issued_positions.as_deref() {
            crate::rt_core::scratch::copy_into(&mut self.scratch.fallback_prev_confirmed, prev);
            self.scratch.fallback_prev_confirmed_valid = true;
        } else {
            self.scratch.fallback_prev_confirmed.clear();
            self.scratch.fallback_prev_confirmed_valid = false;
        }
        let dof = self.config.dof;
        let fresh_meas = self.hardware_feedback_fresh(std::time::Duration::from_millis(100))
            && self.measured_positions.len() == dof
            && self.measured_positions.iter().all(|q| q.is_finite());

        let (depths, other_depths) = if ramp.keepout_only {
            (&self.tracking.ws_keepout_min_depths, &self.tracking.ws_min_depths)
        } else {
            (&self.tracking.ws_min_depths, &self.tracking.ws_keepout_min_depths)
        };
        let mut bufs = std::mem::take(&mut self.ws_eval_bufs);
        let other_depths_rev = if let Some((rev, md)) = other_depths.as_ref() {
            let src = md.as_slice();
            if bufs.tracking_staged_other.len() < src.len() {
                bufs.tracking_staged_other.resize(src.len(), 0.0);
            }
            bufs.tracking_staged_other[..src.len()].copy_from_slice(src);
            Some((*rev, src.len()))
        } else {
            None
        };
        if dof > MAX_DOF {
            self.ws_eval_bufs = bufs;
            return PendingFallbackInline {
                step: Err(FallbackStepErr::InvalidInputs),
                ramp,
                other_depths_rev,
            };
        }
        let mut prev_q_arr = [0.0_f32; MAX_DOF];
        let has_prev_q = self
            .tracking
            .last_issued_positions
            .as_deref()
            .is_some_and(|v| v.len() == dof && v.iter().all(|q| q.is_finite()));
        if has_prev_q {
            prev_q_arr[..dof].copy_from_slice(self.tracking.last_issued_positions.as_deref().unwrap());
        }
        let mut prev_v_arr = [0.0_f32; MAX_DOF];
        if let Some(v) = self
            .tracking
            .last_issued_velocities
            .as_deref()
            .filter(|v| v.len() == dof && v.iter().all(|q| q.is_finite()))
        {
            prev_v_arr[..dof].copy_from_slice(v);
        }
        let step = self.tracking_fallback_step_inline_with_bufs(
            has_prev_q.then(|| &prev_q_arr[..dof]),
            &prev_v_arr[..dof],
            depths.as_ref(),
            &ramp.hold,
            ramp.keepout_only,
            dt,
            ramp.limits.as_slice(),
            fresh_meas,
            budget,
            &mut bufs,
        );
        self.ws_eval_bufs = bufs;
        PendingFallbackInline {
            step,
            ramp,
            other_depths_rev,
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn tracking_fallback_step_inline_with_bufs(
        &self,
        prev_confirmed: Option<&[f32]>,
        prev_v: &[f32],
        prev_ws_min_depths: Option<&(u64, crate::kinematics::SlotBuf)>,
        q_hold: &[f32],
        keepout_only: bool,
        dt: f64,
        active_limits: &[crate::model::JointLimit],
        fresh_meas: bool,
        budget: &mut usize,
        bufs: &mut crate::chain::EvalBufs,
    ) -> Result<ValidatedFallbackInline, FallbackStepErr> {
        use crate::model::MAX_MOTION_DOF as MAX_DOF;

        let dof = self.config.dof;
        if dof > MAX_DOF
            || !dt.is_finite()
            || dt <= 0.0
            || active_limits.len() != dof
            || q_hold.len() != dof
            || !q_hold.iter().all(|q| q.is_finite())
        {
            return Err(FallbackStepErr::InvalidInputs);
        }
        let arm_indices = self.config.arm_joints();
        let Some(pq) = prev_confirmed else {
            if !fresh_meas {
                return Err(FallbackStepErr::NeedsFreshFeedback);
            }
            for &j in arm_indices.iter() {
                let lim = &active_limits[j];
                if q_hold[j] < lim.min_position - 1e-5 || q_hold[j] > lim.max_position + 1e-5 {
                    return Err(FallbackStepErr::HoldOutsideLimits { joint: (j + 1) as u8 });
                }
            }
            let ws = self.fallback_workspace_check_inline(
                None,
                prev_ws_min_depths,
                q_hold,
                keepout_only,
                true,
                budget,
                bufs,
            )?;
            let mut positions = [0.0_f32; MAX_DOF];
            positions[..dof].copy_from_slice(q_hold);
            return Ok(ValidatedFallbackInline {
                positions,
                velocities: [0.0_f32; MAX_DOF],
                dof,
                ws_min_depths_rev: ws,
                done: true,
            });
        };

        let candidates: &[bool] = if fresh_meas { &[false, true] } else { &[true] };
        let mut last_err: Option<FallbackStepErr> = None;
        for &brake_only in candidates {
            let mut q_c = [0.0_f32; MAX_DOF];
            q_c[..dof].copy_from_slice(q_hold);
            let mut feasible = true;
            for &j in arm_indices.iter() {
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
                            last_err = Some(FallbackStepErr::NoBoundedStep { joint: (j + 1) as u8 });
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
            if let Err(e) = Self::fallback_dynamics_ok_inline(&arm_indices, pq, prev_v, &q_c[..dof], dt, active_limits) {
                last_err = Some(e);
                continue;
            }
            let moves = arm_indices.iter().any(|&j| q_c[j].to_bits() != pq[j].to_bits());
            let check_meas = fresh_meas && (moves || !brake_only);
            match self.fallback_workspace_check_inline(
                Some(pq),
                prev_ws_min_depths,
                &q_c[..dof],
                keepout_only,
                check_meas,
                budget,
                bufs,
            ) {
                Ok(ws) => {
                    let mut velocities = [0.0_f32; MAX_DOF];
                    for i in 0..dof {
                        velocities[i] = ((f64::from(q_c[i]) - f64::from(pq[i])) / dt) as f32;
                    }
                    let done = arm_indices.iter().all(|&j| {
                        q_c[j].to_bits() == pq[j].to_bits() && (brake_only || (q_c[j] - q_hold[j]).abs() <= 1e-5)
                    });
                    return Ok(ValidatedFallbackInline {
                        positions: q_c,
                        velocities,
                        dof,
                        ws_min_depths_rev: ws,
                        done,
                    });
                }
                Err(e) => last_err = Some(e),
            }
        }
        Err(last_err.unwrap_or(FallbackStepErr::NoFeasibleStep))
    }

    fn fallback_dynamics_ok_inline(
        arm_indices: &[usize],
        pq: &[f32],
        pv: &[f32],
        q: &[f32],
        dt: f64,
        active_limits: &[crate::model::JointLimit],
    ) -> Result<(), FallbackStepErr> {
        for &j in arm_indices {
            let lim = &active_limits[j];
            if !q[j].is_finite() || q[j] < lim.min_position - 1e-5 || q[j] > lim.max_position + 1e-5 {
                return Err(FallbackStepErr::StepOutsideLimits { joint: (j + 1) as u8, q: q[j] });
            }
            let v = (f64::from(q[j]) - f64::from(pq[j])) / dt;
            let a = (v - f64::from(pv[j])) / dt;
            if !v.is_finite()
                || v.abs() > f64::from(lim.max_velocity) + 1e-4
                || !a.is_finite()
                || a.abs() > f64::from(lim.max_acceleration) + 1e-3
            {
                return Err(FallbackStepErr::ExceedsDynamics {
                    joint: (j + 1) as u8,
                    v: v as f32,
                    max_v: lim.max_velocity,
                    a: a as f32,
                    max_a: lim.max_acceleration,
                });
            }
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn fallback_workspace_check_inline(
        &self,
        prev: Option<&[f32]>,
        prev_depths: Option<&(u64, crate::kinematics::SlotBuf)>,
        q: &[f32],
        keepout_only: bool,
        fresh_meas: bool,
        budget: &mut usize,
        bufs: &mut crate::chain::EvalBufs,
    ) -> Result<Option<(u64, usize)>, FallbackStepErr> {
        use crate::chain::WorkspaceEvaluator;
        use crate::supervisor::MAX_KEEPOUT_EVALS_PER_TICK;

        let Some((ws_full, cam)) = self.workspace_policy() else {
            return Ok(None);
        };
        if keepout_only && ws_full.keepout_boxes.is_empty() {
            if let Some((rev, md)) = prev_depths {
                let src = md.as_slice();
                if bufs.tracking_staged_ws.len() < src.len() {
                    bufs.tracking_staged_ws.resize(src.len(), 0.0);
                }
                bufs.tracking_staged_ws[..src.len()].copy_from_slice(src);
                return Ok(Some((*rev, src.len())));
            }
            return Ok(None);
        }
        let mut eval = WorkspaceEvaluator::new_with_tcp_in_mode(
            &self.config.arm_model,
            ws_full,
            cam,
            self.config.tcp_offset,
            bufs,
            keepout_only,
        );
        let n = eval.total_slots;
        if bufs.scratch_c0.len() < n
            || bufs.scratch_c1.len() < n
            || bufs.scratch_cm.len() < n
            || bufs.scratch_cand_min_d.len() < n
            || bufs.tracking_staged_ws.len() < n
            || bufs.geom_bounds.len() < eval.num_geoms
            || bufs.sweep_slab.len() < 2 * n
        {
            eval.recycle(bufs);
            bufs.prepare_for(&self.config.arm_model, ws_full, cam, self.config.tcp_offset);
            eval = WorkspaceEvaluator::new_with_tcp_in_mode(
                &self.config.arm_model,
                ws_full,
                cam,
                self.config.tcp_offset,
                bufs,
                keepout_only,
            );
        }
        let res = (|| {
            let mut committed = None;
            if let Some(pq) = prev {
                eval.eval_into_slice_rt(pq, &mut bufs.scratch_c0, &mut bufs.world_caps, "tracking_fallback")
                    .map_err(FallbackStepErr::WorkspaceStartViolation)?;
                match prev_depths {
                    Some((rev, md)) if *rev == self.workspace_revision && md.as_slice().len() == n => {
                        bufs.scratch_cand_min_d[..n].copy_from_slice(md.as_slice());
                    }
                    _ => eval.init_min_depths_slice(&bufs.scratch_c0, &mut bufs.scratch_cand_min_d),
                }
                eval.sweep_linear_step_rt(
                    pq,
                    q,
                    &bufs.scratch_c0,
                    &mut bufs.scratch_cand_min_d,
                    budget,
                    MAX_KEEPOUT_EVALS_PER_TICK,
                    "tracking_fallback",
                    "tracking_fallback",
                    &mut bufs.scratch_c1,
                    &mut bufs.scratch_cm,
                    &mut bufs.sweep_slab,
                    &mut bufs.sweep_stack,
                    &mut bufs.sweep_free,
                    &mut bufs.geom_bounds,
                    &mut bufs.world_caps,
                )
                .map_err(FallbackStepErr::WorkspaceStepViolation)?;
                bufs.tracking_staged_ws[..n].copy_from_slice(&bufs.scratch_cand_min_d[..n]);
                committed = Some((self.workspace_revision, n));
            }
            if !fresh_meas {
                return Ok(committed);
            }
            eval.eval_into_slice_rt(&self.measured_positions, &mut bufs.scratch_c0, &mut bufs.world_caps, "tracking_fallback")
                .map_err(FallbackStepErr::WorkspaceMeasuredViolation)?;
            eval.init_min_depths_slice(&bufs.scratch_c0, &mut bufs.scratch_cand_min_d);
            eval.sweep_linear_step_rt(
                &self.measured_positions,
                q,
                &bufs.scratch_c0,
                &mut bufs.scratch_cand_min_d,
                budget,
                MAX_KEEPOUT_EVALS_PER_TICK,
                "tracking_fallback",
                "tracking_fallback",
                &mut bufs.scratch_c1,
                &mut bufs.scratch_cm,
                &mut bufs.sweep_slab,
                &mut bufs.sweep_stack,
                &mut bufs.sweep_free,
                &mut bufs.geom_bounds,
                &mut bufs.world_caps,
            )
            .map_err(FallbackStepErr::WorkspaceMeasuredSweepViolation)?;
            if committed.is_none() {
                bufs.tracking_staged_ws[..n].copy_from_slice(&bufs.scratch_cand_min_d[..n]);
                committed = Some((self.workspace_revision, n));
            }
            Ok(committed)
        })();
        eval.recycle(bufs);
        res
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
