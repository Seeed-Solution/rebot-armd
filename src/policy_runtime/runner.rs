//! Async policy runner, prepared policy pipeline, and supervisor `PolicyRun` state.

use std::path::Path;
use std::sync::Arc;
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::api::sandbox::validate_pack_id;
use crate::model::{ArmConfig, JointLimit, PolicyObservation};
use crate::motion::MotionError;
use crate::supervisor::{ArmError, ArmSupervisor};

use super::manifest::{PolicySnapshot, StrictPolicyContract};
use super::units::{
    denormalize_action_into, normalize_state_into, policy_units_to_rad, rad_to_policy_units,
    JointPermutation,
};
use super::{load_backend, PolicyBackend, PolicyError};

pub const DEFAULT_POLICY_FPS: f64 = 20.0;
pub const MIN_POLICY_FPS: f64 = 1.0;
pub const MIN_LIVE_POLICY_FPS: f64 = 10.0;
pub const MAX_POLICY_FPS: f64 = 50.0;
pub const MAX_POLICY_SECONDS: f64 = 86_400.0;
pub const DEFAULT_MAX_FIRST_STEP_RAD: f32 = 0.25;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PolicyMode {
    Shadow,
    Live,
}

impl PolicyMode {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Shadow => "shadow",
            Self::Live => "live",
        }
    }
}

/// Active policy run tracked by `SupervisorInner`.
#[derive(Debug, Clone)]
pub struct PolicyRun {
    pub action_id: u64,
    pub epoch_id: u64,
    pub connection_id: String,
    pub lease_generation: u64,
    pub policy_id: String,
    pub artifact_digest: String,
    pub backend: String,
    pub mode: PolicyMode,
    pub steps: u64,
    pub last_input_age_ms: Option<u64>,
    pub last_shadow_target: Option<Vec<f32>>,
    pub last_live_target: Option<Vec<f32>>,
    pub max_first_step_rad: f32,
    pub max_step_rad: Option<f32>,
    pub stop_reason: Option<String>,
}

impl PolicyRun {
    pub fn to_observation(&self, active: bool) -> PolicyObservation {
        PolicyObservation {
            active,
            action_id: self.action_id,
            policy_id: self.policy_id.clone(),
            artifact_digest: self.artifact_digest.clone(),
            backend: self.backend.clone(),
            mode: self.mode.as_str().to_string(),
            steps: self.steps,
            last_input_age_ms: self.last_input_age_ms,
            last_shadow_target: self.last_shadow_target.clone(),
            stop_reason: self.stop_reason.clone(),
        }
    }
}

/// Parameters for `SupervisorInner::begin_policy_run`.
#[derive(Debug, Clone)]
pub struct PolicyRunStart {
    pub epoch_id: u64,
    pub connection_id: String,
    pub lease_generation: u64,
    pub policy_id: String,
    pub artifact_digest: String,
    pub backend: String,
    pub mode: PolicyMode,
    pub max_first_step_rad: Option<f32>,
    pub max_step_rad: Option<f32>,
    pub tag: Option<String>,
}

/// Parsed WS `/api/v1/action/policy/start` request.
#[derive(Debug, Clone)]
pub struct PolicyStartRequest {
    pub policy_id: String,
    pub mode: PolicyMode,
    pub max_seconds: Option<f64>,
    pub fps: Option<f64>,
    pub tag: Option<String>,
}

/// Validate an effective policy rate for the requested execution mode.
pub fn validate_policy_fps(mode: PolicyMode, fps: f64) -> Result<f64, MotionError> {
    if !fps.is_finite() || fps <= 0.0 {
        return Err(MotionError::InvalidCommand(
            "fps must be a positive finite number".into(),
        ));
    }
    let min_fps = match mode {
        PolicyMode::Live => MIN_LIVE_POLICY_FPS,
        PolicyMode::Shadow => MIN_POLICY_FPS,
    };
    if fps < min_fps || fps > MAX_POLICY_FPS {
        return Err(MotionError::InvalidCommand(format!(
            "fps for {} mode must be between {min_fps} and {MAX_POLICY_FPS}, got {fps}",
            mode.as_str()
        )));
    }
    Ok(fps)
}

/// Validate `max_seconds` and convert it into a monotonic `Instant` deadline without panicking.
pub fn compute_policy_deadline(max_seconds: Option<f64>) -> Result<Option<Instant>, MotionError> {
    let Some(secs) = max_seconds else {
        return Ok(None);
    };
    if !secs.is_finite() || secs <= 0.0 || secs > MAX_POLICY_SECONDS {
        return Err(MotionError::InvalidCommand(format!(
            "max_seconds must be a positive finite number <= {MAX_POLICY_SECONDS}"
        )));
    }
    let dur = Duration::try_from_secs_f64(secs).map_err(|_| {
        MotionError::InvalidCommand("max_seconds is out of representable duration range".into())
    })?;
    let deadline = Instant::now().checked_add(dur).ok_or_else(|| {
        MotionError::InvalidCommand("max_seconds overflows system clock instant".into())
    })?;
    Ok(Some(deadline))
}

pub fn parse_policy_start_request(body: &Value) -> Result<PolicyStartRequest, MotionError> {
    let policy_id = body
        .get("policy_id")
        .and_then(Value::as_str)
        .ok_or_else(|| MotionError::InvalidCommand("policy_id must be a non-empty string".into()))?;
    if !validate_pack_id(policy_id) {
        return Err(MotionError::InvalidCommand(format!(
            "policy_id '{policy_id}' is not a valid pack id"
        )));
    }

    let mode = match body.get("mode").and_then(Value::as_str) {
        Some("shadow") => PolicyMode::Shadow,
        Some("live") | Some("live_sim") => PolicyMode::Live,
        Some(other) => {
            return Err(MotionError::InvalidCommand(format!(
                "unsupported policy mode '{other}'; expected 'shadow' or 'live'"
            )));
        }
        None => {
            return Err(MotionError::InvalidCommand(
                "mode must be 'shadow' or 'live'".into(),
            ));
        }
    };

    let max_seconds = match body.get("max_seconds") {
        None | Some(Value::Null) => None,
        Some(v) => {
            let secs = v.as_f64().ok_or_else(|| {
                MotionError::InvalidCommand("max_seconds must be a positive finite number".into())
            })?;
            if !secs.is_finite() || secs <= 0.0 || secs > MAX_POLICY_SECONDS {
                return Err(MotionError::InvalidCommand(format!(
                    "max_seconds must be a positive finite number <= {MAX_POLICY_SECONDS}"
                )));
            }
            Some(secs)
        }
    };

    let fps = match body.get("fps") {
        None | Some(Value::Null) => None,
        Some(v) => {
            let f = v.as_f64().ok_or_else(|| {
                MotionError::InvalidCommand("fps must be a positive finite number".into())
            })?;
            Some(validate_policy_fps(mode, f)?)
        }
    };

    let tag = match body.get("tag") {
        None | Some(Value::Null) => None,
        Some(v) => {
            let s = v.as_str().ok_or_else(|| {
                MotionError::InvalidCommand("tag must be a string".into())
            })?;
            Some(s.to_string())
        }
    };

    Ok(PolicyStartRequest {
        policy_id: policy_id.to_string(),
        mode,
        max_seconds,
        fps,
        tag,
    })
}

impl From<PolicyError> for ArmError {
    fn from(err: PolicyError) -> Self {
        let (status, code) = match &err {
            PolicyError::Io(_) => (404, "POLICY_IO_ERROR"),
            PolicyError::UnsupportedBackend(_) => (422, "UNSUPPORTED_BACKEND"),
            PolicyError::BackendLoad(_) | PolicyError::InferenceFailed(_) => {
                (422, "POLICY_LOAD_FAILED")
            }
            PolicyError::InvalidPath(_)
            | PolicyError::InvalidManifest(_)
            | PolicyError::ExternalOnnxData(_)
            | PolicyError::InvalidUnits(_)
            | PolicyError::DimensionMismatch { .. }
            | PolicyError::NonFinite(_)
            | PolicyError::InvalidArtifact(_) => (422, "INVALID_POLICY"),
        };
        ArmError::Rejected {
            status,
            code: code.to_string(),
            message: err.to_string(),
        }
    }
}

/// Fully validated, preallocated policy inference pipeline ready to step against an arm config.
pub struct PreparedPolicy {
    snapshot: PolicySnapshot,
    contract: StrictPolicyContract,
    backend: Box<dyn PolicyBackend>,
    perm: JointPermutation,
    reordered_limits: Vec<JointLimit>,
    policy_gripper_idx: Option<usize>,
    gripper_bounds: Option<(f32, f32)>,
    policy_state_rad: Vec<f32>,
    infer_in: Vec<f32>,
    infer_out: Vec<f32>,
    policy_act_units: Vec<f32>,
    arm_target_rad: Vec<f32>,
}

impl PreparedPolicy {
    /// Construct a validated inference pipeline from an explicit backend and arm config.
    pub fn from_backend(
        snapshot: PolicySnapshot,
        contract: StrictPolicyContract,
        backend: Box<dyn PolicyBackend>,
        config: &ArmConfig,
    ) -> Result<Self, PolicyError> {
        if config.dof != contract.action_dim {
            return Err(PolicyError::DimensionMismatch {
                expected: config.dof,
                actual: contract.action_dim,
            });
        }
        if config.limits.len() != contract.action_dim {
            return Err(PolicyError::DimensionMismatch {
                expected: contract.action_dim,
                actual: config.limits.len(),
            });
        }
        if backend.input_dim() != contract.action_dim {
            return Err(PolicyError::DimensionMismatch {
                expected: contract.action_dim,
                actual: backend.input_dim(),
            });
        }
        if backend.output_dim() != contract.action_dim {
            return Err(PolicyError::DimensionMismatch {
                expected: contract.action_dim,
                actual: backend.output_dim(),
            });
        }

        let perm = JointPermutation::from_arm_config(&contract, config)?;
        let reordered_limits = perm.reordered_limits(&config.limits)?;
        let policy_gripper_idx = config
            .gripper_index()
            .and_then(|arm_idx| perm.policy_index_for_arm_joint(arm_idx));
        let gripper_bounds = config
            .gripper_policy
            .as_ref()
            .map(|p| (p.closed_rad, p.open_rad));

        let dim = contract.action_dim;
        Ok(Self {
            snapshot,
            contract,
            backend,
            perm,
            reordered_limits,
            policy_gripper_idx,
            gripper_bounds,
            policy_state_rad: vec![0.0; dim],
            infer_in: vec![0.0; dim],
            infer_out: vec![0.0; dim],
            policy_act_units: vec![0.0; dim],
            arm_target_rad: vec![0.0; dim],
        })
    }

    pub fn artifact_digest(&self) -> &str {
        &self.snapshot.release_digest
    }

    pub fn backend_name(&self) -> &'static str {
        self.backend.backend_name()
    }

    pub fn contract(&self) -> &StrictPolicyContract {
        &self.contract
    }

    /// Run one full state -> units -> normalize -> infer -> denormalize -> radians -> reorder step.
    pub fn step_infer(&mut self, measured_arm_rad: &[f32]) -> Result<Vec<f32>, PolicyError> {
        let dim = self.contract.action_dim;
        if measured_arm_rad.len() != dim {
            return Err(PolicyError::DimensionMismatch {
                expected: dim,
                actual: measured_arm_rad.len(),
            });
        }
        for (idx, &v) in measured_arm_rad.iter().enumerate() {
            if !v.is_finite() {
                return Err(PolicyError::NonFinite(format!(
                    "measured_positions[{idx}] is non-finite: {v}"
                )));
            }
        }

        self.perm
            .reorder_arm_to_policy(measured_arm_rad, &mut self.policy_state_rad)?;

        let state_units = rad_to_policy_units(
            &self.policy_state_rad,
            self.contract.unit_mode,
            &self.reordered_limits,
            self.policy_gripper_idx,
            self.gripper_bounds,
        )?;

        if let (Some(mean), Some(std)) = (
            &self.contract.normalization.state_mean,
            &self.contract.normalization.state_std,
        ) {
            normalize_state_into(&state_units, mean, std, &mut self.infer_in)?;
        } else {
            self.infer_in.copy_from_slice(&state_units);
        }

        self.backend.infer(&self.infer_in, &mut self.infer_out)?;
        for (idx, &v) in self.infer_out.iter().enumerate() {
            if !v.is_finite() {
                return Err(PolicyError::NonFinite(format!(
                    "model output[{idx}] is non-finite: {v}"
                )));
            }
        }

        if let (Some(mean), Some(std)) = (
            &self.contract.normalization.action_mean,
            &self.contract.normalization.action_std,
        ) {
            denormalize_action_into(&self.infer_out, mean, std, &mut self.policy_act_units)?;
        } else {
            self.policy_act_units.copy_from_slice(&self.infer_out);
        }

        let act_rad = policy_units_to_rad(
            &self.policy_act_units,
            self.contract.unit_mode,
            &self.reordered_limits,
            self.policy_gripper_idx,
            self.gripper_bounds,
        )?;

        self.perm
            .reorder_policy_to_arm(&act_rad, &mut self.arm_target_rad)?;

        for (idx, &v) in self.arm_target_rad.iter().enumerate() {
            if !v.is_finite() {
                return Err(PolicyError::NonFinite(format!(
                    "arm target[{idx}] is non-finite: {v}"
                )));
            }
        }

        Ok(self.arm_target_rad.clone())
    }
}

/// Load and validate a policy pack from `dir` against `config` without holding any supervisor lock.
pub fn prepare_policy(dir: &Path, config: &ArmConfig) -> Result<PreparedPolicy, PolicyError> {
    let snapshot = PolicySnapshot::load_from_dir(dir)?;
    let contract = StrictPolicyContract::parse(&snapshot.policy_json_bytes)?;

    if config.dof != contract.action_dim {
        return Err(PolicyError::DimensionMismatch {
            expected: config.dof,
            actual: contract.action_dim,
        });
    }
    if config.limits.len() != contract.action_dim {
        return Err(PolicyError::DimensionMismatch {
            expected: contract.action_dim,
            actual: config.limits.len(),
        });
    }

    let backend = load_backend(&snapshot, &contract)?;
    PreparedPolicy::from_backend(snapshot, contract, backend, config)
}

/// Spawn the async policy runner task that executes inference outside the supervisor lock and
/// submits targets via `SupervisorInner::policy_step`.
pub fn spawn_policy_runner(
    supervisor: Arc<ArmSupervisor>,
    action_id: u64,
    mut prepared: PreparedPolicy,
    fps: f64,
    deadline: Option<Instant>,
) -> tokio::task::JoinHandle<()> {
    let clamped_fps = if fps.is_finite() && fps > 0.0 {
        fps.clamp(MIN_POLICY_FPS, MAX_POLICY_FPS)
    } else {
        DEFAULT_POLICY_FPS
    };
    let period = Duration::from_secs_f64(1.0 / clamped_fps);
    let tokio_dl = deadline.map(tokio::time::Instant::from_std);

    tokio::spawn(async move {
        let mut interval = tokio::time::interval(period);
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

        loop {
            if let Some(dl) = tokio_dl {
                tokio::select! {
                    biased;
                    _ = tokio::time::sleep_until(dl) => {
                        let mut inner = supervisor.inner().write().await;
                        inner.finish_policy_run(action_id, "MAX_SECONDS");
                        break;
                    }
                    _ = interval.tick() => {}
                }
            } else {
                interval.tick().await;
            }
            if let Some(dl) = deadline {
                if Instant::now() >= dl {
                    let mut inner = supervisor.inner().write().await;
                    inner.finish_policy_run(action_id, "MAX_SECONDS");
                    break;
                }
            }

            let (measured, base_input_age_ms, captured_at) = {
                let inner = supervisor.inner().read().await;
                if !inner.is_policy_run_active(action_id) {
                    break;
                }
                (
                    inner.measured_positions.clone(),
                    inner.policy_input_age_ms(),
                    Instant::now(),
                )
            };

            let infer_handle = tokio::task::spawn_blocking(move || {
                let res = prepared.step_infer(&measured);
                (prepared, res)
            });
            let infer_result = if let Some(dl) = tokio_dl {
                tokio::select! {
                    biased;
                    _ = tokio::time::sleep_until(dl) => {
                        let mut inner = supervisor.inner().write().await;
                        inner.finish_policy_run(action_id, "MAX_SECONDS");
                        break;
                    }
                    res = infer_handle => res,
                }
            } else {
                infer_handle.await
            };

            let target = match infer_result {
                Err(e) => {
                    tracing::warn!("Policy inference task failed for action {action_id}: {e}");
                    let mut inner = supervisor.inner().write().await;
                    inner.fail_policy_run(
                        action_id,
                        "INFERENCE_ERROR",
                        format!("inference task failed: {e}"),
                    );
                    break;
                }
                Ok((p, res)) => {
                    prepared = p;
                    match res {
                        Ok(t) => t,
                        Err(e) => {
                            tracing::warn!("Policy inference failed for action {action_id}: {e}");
                            let reason = match e {
                                PolicyError::NonFinite(_) => "NON_FINITE",
                                _ => "INFERENCE_ERROR",
                            };
                            let mut inner = supervisor.inner().write().await;
                            inner.fail_policy_run(action_id, reason, e.to_string());
                            break;
                        }
                    }
                }
            };

            let mut inner = supervisor.inner().write().await;
            if let Some(dl) = deadline {
                if Instant::now() >= dl {
                    inner.finish_policy_run(action_id, "MAX_SECONDS");
                    break;
                }
            }
            let input_age_ms = base_input_age_ms.map(|base| {
                base.saturating_add(captured_at.elapsed().as_millis().min(u64::MAX as u128) as u64)
            });
            if let Err(e) = inner.policy_step(action_id, target, input_age_ms) {
                tracing::info!("Policy runner {action_id} exited on step: {e}");
                break;
            }
        }
    })
}
