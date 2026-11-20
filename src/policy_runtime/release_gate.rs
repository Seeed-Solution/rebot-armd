//! Fail-closed `rebot-release-gate/v1` verification for real-hardware policy execution.

use std::fs::{self, File};
use std::io::Read;
use std::path::Path;

use serde_json::Value;

use crate::model::ArmConfig;

use super::manifest::{StrictPolicyContract, UnitMode, MAX_POLICY_JSON_BYTES};
use super::PolicyError;

pub const RELEASE_GATE_SCHEMA_VERSION: &str = "rebot-release-gate/v1";
pub const EMPTY_CALIBRATION_DIGEST: &str =
    "f2d07c3ba2e96404e3194aa9f22a6b546767844226c7c30ad4ad902358fdcb78";
pub const SUPERVISED_FIRST_RUN_MAX_SPEED_SCALE: f64 = 0.3;
pub const MIN_SIM_EVAL_SEEDS: u64 = 10;

/// Active deployment binding expected by a `rebot-release-gate/v1` verification.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ActiveDeploymentContext {
    pub scene_id: String,
    pub tool_id: String,
    pub calibration_digest: String,
}

/// Authorization tier declared by a `rebot-release-gate/v1` document.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuthorizationTier {
    SupervisedFirstRun,
    ProductionRelease,
}

impl AuthorizationTier {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::SupervisedFirstRun => "supervised_first_run",
            Self::ProductionRelease => "production_release",
        }
    }
}

/// Verified and digest-bound `rebot-release-gate/v1` authorization artifact.
///
/// Fields are private so callers cannot construct an unverified token or mutate
/// verified limits or digests after verification.
#[derive(Debug, Clone, PartialEq)]
pub struct VerifiedReleaseGate {
    schema_version: String,
    authorization_tier: AuthorizationTier,
    policy_sha256: String,
    scene_id: String,
    tool_id: String,
    profile_id: String,
    canonical_profile_id: String,
    calibration_digest: String,
    fps: u64,
    max_speed_scale: f64,
    unit_mode: UnitMode,
    action_space: String,
}

impl VerifiedReleaseGate {
    pub fn schema_version(&self) -> &str {
        &self.schema_version
    }

    pub fn authorization_tier(&self) -> AuthorizationTier {
        self.authorization_tier
    }

    pub fn policy_sha256(&self) -> &str {
        &self.policy_sha256
    }

    pub fn scene_id(&self) -> &str {
        &self.scene_id
    }

    pub fn tool_id(&self) -> &str {
        &self.tool_id
    }

    pub fn profile_id(&self) -> &str {
        &self.profile_id
    }

    pub fn canonical_profile_id(&self) -> &str {
        &self.canonical_profile_id
    }

    pub fn calibration_digest(&self) -> &str {
        &self.calibration_digest
    }

    pub fn fps(&self) -> u64 {
        self.fps
    }

    pub fn max_speed_scale(&self) -> f64 {
        self.max_speed_scale
    }

    pub fn unit_mode(&self) -> UnitMode {
        self.unit_mode
    }

    pub fn action_space(&self) -> &str {
        &self.action_space
    }

    /// Re-verify that this gate still matches the active scene, tool, and calibration digest.
    pub fn verify_deployment(&self, deployment: &ActiveDeploymentContext) -> Result<(), PolicyError> {
        let expected_scene = deployment.scene_id.trim();
        if expected_scene.is_empty() || self.scene_id != expected_scene {
            return Err(PolicyError::ReleaseGate(format!(
                "scene_id mismatch: gate declares {:?}, active deployment is {expected_scene:?}",
                self.scene_id
            )));
        }
        let expected_tool = deployment.tool_id.trim();
        if expected_tool.is_empty() || self.tool_id != expected_tool {
            return Err(PolicyError::ReleaseGate(format!(
                "tool_id mismatch: gate declares {:?}, active deployment is {expected_tool:?}",
                self.tool_id
            )));
        }
        let expected_calib = deployment.calibration_digest.trim();
        if expected_calib.is_empty() || !self.calibration_digest.eq_ignore_ascii_case(expected_calib) {
            return Err(PolicyError::ReleaseGate(format!(
                "calibration_digest mismatch: gate declares {}, active deployment is {expected_calib}",
                self.calibration_digest
            )));
        }
        Ok(())
    }
}

/// Parser and fail-closed verifier for `rebot-release-gate/v1` documents.
pub struct ReleaseGate;

impl ReleaseGate {
    /// Parse and strictly verify a `rebot-release-gate/v1` JSON document against the
    /// immutable policy digest, arm config, policy contract, effective runner FPS, and
    /// active deployment context (`scene_id`, `tool_id`, `calibration_digest`).
    pub fn parse_and_verify(
        raw: &Value,
        expected_policy_sha256: &str,
        config: &ArmConfig,
        contract: &StrictPolicyContract,
        effective_fps: f64,
        deployment: &ActiveDeploymentContext,
    ) -> Result<VerifiedReleaseGate, PolicyError> {
        let obj = raw
            .as_object()
            .ok_or_else(|| PolicyError::ReleaseGate("release_gate must be a JSON object".into()))?;

        let schema_version = require_non_empty_str(obj.get("schema_version"), "schema_version")?;
        if schema_version != RELEASE_GATE_SCHEMA_VERSION {
            return Err(PolicyError::ReleaseGate(format!(
                "unsupported release gate schema_version {schema_version:?}; expected {RELEASE_GATE_SCHEMA_VERSION:?}"
            )));
        }

        let tier_str = require_non_empty_str(obj.get("authorization_tier"), "authorization_tier")?;
        let authorization_tier = match tier_str.as_str() {
            "supervised_first_run" => AuthorizationTier::SupervisedFirstRun,
            "production_release" => AuthorizationTier::ProductionRelease,
            other => {
                return Err(PolicyError::ReleaseGate(format!(
                    "unsupported authorization_tier {other:?}; expected 'supervised_first_run' or 'production_release'"
                )));
            }
        };

        let expected_sha = require_sha256_hex(
            &Value::String(expected_policy_sha256.to_string()),
            "expected_policy_sha256",
        )?;
        let policy_sha256 = require_sha256_hex_opt(obj.get("policy_sha256"), "policy_sha256")?;
        if policy_sha256 != expected_sha {
            return Err(PolicyError::ReleaseGate(format!(
                "policy_sha256 mismatch: gate declares {policy_sha256}, expected {expected_sha}"
            )));
        }

        let scene_id = require_non_empty_str(obj.get("scene_id"), "scene_id")?;
        let tool_id = require_non_empty_str(obj.get("tool_id"), "tool_id")?;
        let calibration_digest =
            require_non_empty_str(obj.get("calibration_digest"), "calibration_digest")?;

        let expected_scene = deployment.scene_id.trim();
        if expected_scene.is_empty() || scene_id != expected_scene {
            return Err(PolicyError::ReleaseGate(format!(
                "scene_id mismatch: gate declares {scene_id:?}, active deployment is {expected_scene:?}"
            )));
        }
        let expected_tool = deployment.tool_id.trim();
        if expected_tool.is_empty() || tool_id != expected_tool {
            return Err(PolicyError::ReleaseGate(format!(
                "tool_id mismatch: gate declares {tool_id:?}, active deployment is {expected_tool:?}"
            )));
        }
        let expected_calib = deployment.calibration_digest.trim();
        if expected_calib.is_empty() || !calibration_digest.eq_ignore_ascii_case(expected_calib) {
            return Err(PolicyError::ReleaseGate(format!(
                "calibration_digest mismatch: gate declares {calibration_digest}, active deployment is {expected_calib}"
            )));
        }

        let profile_id = require_non_empty_str(obj.get("profile_id"), "profile_id")?;
        let canonical_profile_id = verify_profile_match(&profile_id, config, contract)?;

        // runner_config
        let rc = obj
            .get("runner_config")
            .and_then(Value::as_object)
            .ok_or_else(|| {
                PolicyError::ReleaseGate("runner_config must be a JSON object".into())
            })?;

        let fps = require_positive_int(rc.get("fps"), "runner_config.fps")?;
        if !effective_fps.is_finite() || effective_fps <= 0.0 {
            return Err(PolicyError::ReleaseGate(format!(
                "effective_fps must be positive finite, got {effective_fps}"
            )));
        }
        if ((fps as f64) - effective_fps).abs() > 1e-6 {
            return Err(PolicyError::ReleaseGate(format!(
                "runner_config.fps ({fps}) does not match effective_fps ({effective_fps})"
            )));
        }

        let max_speed_scale =
            require_finite_f64(rc.get("max_speed_scale"), "runner_config.max_speed_scale")?;
        if !(0.1..=1.0).contains(&max_speed_scale) {
            return Err(PolicyError::ReleaseGate(format!(
                "runner_config.max_speed_scale must be in [0.1, 1.0], got {max_speed_scale}"
            )));
        }
        if authorization_tier == AuthorizationTier::SupervisedFirstRun
            && max_speed_scale > SUPERVISED_FIRST_RUN_MAX_SPEED_SCALE + 1e-12
        {
            return Err(PolicyError::ReleaseGate(format!(
                "supervised_first_run runner_config.max_speed_scale ({max_speed_scale}) exceeds mandatory limit {SUPERVISED_FIRST_RUN_MAX_SPEED_SCALE}"
            )));
        }

        let unit_mode_str =
            require_non_empty_str(rc.get("unit_mode"), "runner_config.unit_mode")?;
        if unit_mode_str != contract.unit_mode.as_str() {
            return Err(PolicyError::ReleaseGate(format!(
                "runner_config.unit_mode ({unit_mode_str:?}) does not match policy unit_mode ({:?})",
                contract.unit_mode.as_str()
            )));
        }

        let action_space =
            require_non_empty_str(rc.get("action_space"), "runner_config.action_space")?;
        if action_space != "joint_positions" {
            return Err(PolicyError::ReleaseGate(format!(
                "runner_config.action_space must be 'joint_positions', got {action_space:?}"
            )));
        }
        if contract.action_space != action_space {
            return Err(PolicyError::ReleaseGate(format!(
                "runner_config.action_space ({action_space:?}) does not match policy action_space ({:?})",
                contract.action_space
            )));
        }

        // sim_eval
        let sim = obj
            .get("sim_eval")
            .and_then(Value::as_object)
            .ok_or_else(|| PolicyError::ReleaseGate("sim_eval must be a JSON object".into()))?;

        let seeds_total = require_non_neg_int(sim.get("seeds_total"), "sim_eval.seeds_total")?;
        if seeds_total < MIN_SIM_EVAL_SEEDS {
            return Err(PolicyError::ReleaseGate(format!(
                "sim_eval.seeds_total ({seeds_total}) must be >= {MIN_SIM_EVAL_SEEDS}"
            )));
        }
        let seeds_passed = require_non_neg_int(sim.get("seeds_passed"), "sim_eval.seeds_passed")?;
        if seeds_passed == 0 {
            return Err(PolicyError::ReleaseGate(
                "sim_eval.seeds_passed must be > 0".into(),
            ));
        }
        if seeds_passed > seeds_total {
            return Err(PolicyError::ReleaseGate(format!(
                "sim_eval.seeds_passed ({seeds_passed}) out of range [1, {seeds_total}]"
            )));
        }
        let min_success_rate =
            require_finite_f64(sim.get("min_success_rate"), "sim_eval.min_success_rate")?;
        if !(min_success_rate > 0.0 && min_success_rate <= 1.0) {
            return Err(PolicyError::ReleaseGate(format!(
                "sim_eval.min_success_rate must be in (0.0, 1.0], got {min_success_rate}"
            )));
        }
        let success_rate = (seeds_passed as f64) / (seeds_total as f64);
        if success_rate < min_success_rate {
            return Err(PolicyError::ReleaseGate(format!(
                "sim_eval success rate {seeds_passed}/{seeds_total} ({success_rate:.3}) is below min_success_rate ({min_success_rate:.3})"
            )));
        }
        let collisions = require_non_neg_int(sim.get("collisions"), "sim_eval.collisions")?;
        if collisions != 0 {
            return Err(PolicyError::ReleaseGate(format!(
                "sim_eval.collisions must be 0, got {collisions}"
            )));
        }
        let keepout_violations =
            require_non_neg_int(sim.get("keepout_violations"), "sim_eval.keepout_violations")?;
        if keepout_violations != 0 {
            return Err(PolicyError::ReleaseGate(format!(
                "sim_eval.keepout_violations must be 0, got {keepout_violations}"
            )));
        }

        // prerequisites
        let pre = obj
            .get("prerequisites")
            .and_then(Value::as_object)
            .ok_or_else(|| {
                PolicyError::ReleaseGate("prerequisites must be a JSON object".into())
            })?;
        if !require_bool(
            pre.get("calibration_verified"),
            "prerequisites.calibration_verified",
        )? {
            return Err(PolicyError::ReleaseGate(
                "prerequisites.calibration_verified must be true".into(),
            ));
        }
        if !require_bool(
            pre.get("power_cut_witnessed"),
            "prerequisites.power_cut_witnessed",
        )? {
            return Err(PolicyError::ReleaseGate(
                "prerequisites.power_cut_witnessed must be true".into(),
            ));
        }
        if !require_bool(
            pre.get("shadow_run_passed"),
            "prerequisites.shadow_run_passed",
        )? {
            return Err(PolicyError::ReleaseGate(
                "prerequisites.shadow_run_passed must be true".into(),
            ));
        }

        // supervised_real_run_record
        match (
            authorization_tier,
            obj.get("supervised_real_run_record").filter(|v| !v.is_null()),
        ) {
            (AuthorizationTier::ProductionRelease, None) => {
                return Err(PolicyError::ReleaseGate(
                    "authorization_tier 'production_release' requires a non-empty 'supervised_real_run_record' object".into(),
                ));
            }
            (AuthorizationTier::ProductionRelease, Some(rec)) => {
                let rec_obj = rec.as_object().ok_or_else(|| {
                    PolicyError::ReleaseGate(
                        "supervised_real_run_record must be a JSON object".into(),
                    )
                })?;
                if rec_obj.is_empty() {
                    return Err(PolicyError::ReleaseGate(
                        "supervised_real_run_record must not be empty".into(),
                    ));
                }
                if !require_bool(
                    rec_obj.get("passed"),
                    "supervised_real_run_record.passed",
                )? {
                    return Err(PolicyError::ReleaseGate(
                        "production_release supervised_real_run_record.passed must be true".into(),
                    ));
                }
                let rec_sha = require_sha256_hex_opt(
                    rec_obj.get("policy_sha256"),
                    "supervised_real_run_record.policy_sha256",
                )?;
                if rec_sha != expected_sha {
                    return Err(PolicyError::ReleaseGate(format!(
                        "supervised_real_run_record.policy_sha256 ({rec_sha}) does not match expected policy_sha256 ({expected_sha})"
                    )));
                }
            }
            (AuthorizationTier::SupervisedFirstRun, Some(rec)) => {
                let rec_obj = rec.as_object().ok_or_else(|| {
                    PolicyError::ReleaseGate(
                        "supervised_real_run_record must be a JSON object when provided".into(),
                    )
                })?;
                if rec_obj.is_empty() {
                    return Err(PolicyError::ReleaseGate(
                        "supervised_real_run_record must not be empty when provided".into(),
                    ));
                }
            }
            (AuthorizationTier::SupervisedFirstRun, None) => {}
        }

        Ok(VerifiedReleaseGate {
            schema_version,
            authorization_tier,
            policy_sha256,
            scene_id,
            tool_id,
            profile_id,
            canonical_profile_id,
            calibration_digest,
            fps,
            max_speed_scale,
            unit_mode: contract.unit_mode,
            action_space,
        })
    }
}

/// Safely load an optional `release_gate.json` inside `pack_dir` without following symlink escapes.
pub fn load_release_gate_json(pack_dir: &Path) -> Result<Option<Value>, PolicyError> {
    let dir_canon = fs::canonicalize(pack_dir).map_err(|e| {
        PolicyError::Io(format!(
            "cannot resolve policy directory '{}': {e}",
            pack_dir.display()
        ))
    })?;
    let gate_path = dir_canon.join("release_gate.json");
    let gate_canon = match fs::canonicalize(&gate_path) {
        Ok(p) => p,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => {
            return Err(PolicyError::ReleaseGate(format!(
                "cannot resolve release_gate.json '{}': {e}",
                gate_path.display()
            )));
        }
    };
    if !gate_canon.starts_with(&dir_canon) || !gate_canon.is_file() {
        return Err(PolicyError::ReleaseGate(format!(
            "release_gate.json '{}' escapes policy directory or is not a file",
            gate_canon.display()
        )));
    }
    let file = File::open(&gate_canon).map_err(|e| {
        PolicyError::ReleaseGate(format!(
            "failed to open release_gate.json '{}': {e}",
            gate_canon.display()
        ))
    })?;
    let meta = file.metadata().map_err(|e| {
        PolicyError::ReleaseGate(format!(
            "failed to stat release_gate.json '{}': {e}",
            gate_canon.display()
        ))
    })?;
    if !meta.is_file() {
        return Err(PolicyError::ReleaseGate(
            "release_gate.json is not a regular file".into(),
        ));
    }
    if meta.len() == 0 {
        return Err(PolicyError::ReleaseGate("release_gate.json is empty".into()));
    }
    if meta.len() > MAX_POLICY_JSON_BYTES as u64 {
        return Err(PolicyError::ReleaseGate(format!(
            "release_gate.json size ({} bytes) exceeds maximum ({MAX_POLICY_JSON_BYTES} bytes)",
            meta.len()
        )));
    }
    let mut buf = Vec::with_capacity((meta.len() as usize).min(MAX_POLICY_JSON_BYTES));
    let mut limited = file.take((MAX_POLICY_JSON_BYTES as u64) + 1);
    limited
        .read_to_end(&mut buf)
        .map_err(|e| PolicyError::ReleaseGate(format!("failed to read release_gate.json: {e}")))?;
    if buf.len() > MAX_POLICY_JSON_BYTES {
        return Err(PolicyError::ReleaseGate(format!(
            "release_gate.json exceeds maximum ({MAX_POLICY_JSON_BYTES} bytes)"
        )));
    }
    let parsed: Value = serde_json::from_slice(&buf)
        .map_err(|e| PolicyError::ReleaseGate(format!("invalid JSON in release_gate.json: {e}")))?;
    Ok(Some(parsed))
}

/// Normalize a known robot/profile identifier via `rebot_companion/onnx_policy.py` `ROBOT_ALIASES`.
pub fn canonical_robot_id(raw: &str) -> Option<&'static str> {
    let norm = raw.trim().to_ascii_lowercase();
    if norm.is_empty() {
        return None;
    }
    if let Some(canon) = lookup_robot_alias(&norm) {
        return Some(canon);
    }
    let collapsed = norm.replace('_', "-");
    lookup_robot_alias(&collapsed)
}

fn lookup_robot_alias(s: &str) -> Option<&'static str> {
    match s {
        "b601dm" | "b601-dm" | "b601_dm" | "rebot-dm" | "rebot_dm" | "rebot-b601dm"
        | "rebot_b601dm" | "rebot-b601-dm" | "rebot_b601_dm" => Some("b601dm"),
        "so-arm101" | "so_arm101" | "soarm101" | "so101" | "so-101" | "so_101" => {
            Some("so-arm101")
        }
        _ => None,
    }
}

fn verify_profile_match(
    gate_profile: &str,
    config: &ArmConfig,
    contract: &StrictPolicyContract,
) -> Result<String, PolicyError> {
    let cfg_profile = config
        .profile_id
        .as_deref()
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .ok_or_else(|| {
            PolicyError::ReleaseGate(
                "arm config does not declare a non-empty profile_id; failing closed".into(),
            )
        })?;

    let gate_trimmed = gate_profile.trim();
    let resolved_canon = match canonical_robot_id(cfg_profile) {
        Some(cfg_canon) => {
            let gate_canon = canonical_robot_id(gate_trimmed).ok_or_else(|| {
                PolicyError::ReleaseGate(format!(
                    "release gate profile_id {gate_trimmed:?} does not match arm canonical profile {cfg_canon:?}"
                ))
            })?;
            if gate_canon != cfg_canon {
                return Err(PolicyError::ReleaseGate(format!(
                    "release gate profile_id {gate_trimmed:?} ({gate_canon:?}) does not match arm profile_id {cfg_profile:?} ({cfg_canon:?})"
                )));
            }
            cfg_canon.to_string()
        }
        None => {
            if !gate_trimmed.eq_ignore_ascii_case(cfg_profile) {
                return Err(PolicyError::ReleaseGate(format!(
                    "release gate profile_id {gate_trimmed:?} does not match arm profile_id {cfg_profile:?}"
                )));
            }
            cfg_profile.to_ascii_lowercase()
        }
    };

    if let Some(ref manifest_robot) = contract.robot {
        let mr = manifest_robot.trim();
        let ok = match canonical_robot_id(cfg_profile) {
            Some(canon) => canonical_robot_id(mr) == Some(canon),
            None => mr.eq_ignore_ascii_case(cfg_profile),
        };
        if !ok {
            return Err(PolicyError::ReleaseGate(format!(
                "policy manifest robot {mr:?} does not match arm profile_id {cfg_profile:?}"
            )));
        }
    }

    Ok(resolved_canon)
}

fn require_non_empty_str(val: Option<&Value>, label: &str) -> Result<String, PolicyError> {
    let s = val
        .and_then(Value::as_str)
        .ok_or_else(|| PolicyError::ReleaseGate(format!("{label} must be a non-empty string")))?;
    let trimmed = s.trim();
    if trimmed.is_empty() {
        return Err(PolicyError::ReleaseGate(format!(
            "{label} must be a non-empty string"
        )));
    }
    Ok(trimmed.to_string())
}

fn require_sha256_hex_opt(val: Option<&Value>, label: &str) -> Result<String, PolicyError> {
    let v = val
        .ok_or_else(|| PolicyError::ReleaseGate(format!("{label} must be a 64-char hex SHA-256 digest")))?;
    require_sha256_hex(v, label)
}

fn require_sha256_hex(val: &Value, label: &str) -> Result<String, PolicyError> {
    let s = require_non_empty_str(Some(val), label)?;
    if s.len() != 64 || !s.bytes().all(|b| b.is_ascii_hexdigit()) {
        return Err(PolicyError::ReleaseGate(format!(
            "{label} must be a 64-character hex SHA-256 digest"
        )));
    }
    Ok(s.to_ascii_lowercase())
}

fn require_non_neg_int(val: Option<&Value>, label: &str) -> Result<u64, PolicyError> {
    let v = val.ok_or_else(|| PolicyError::ReleaseGate(format!("{label} must be an integer")))?;
    v.as_u64()
        .ok_or_else(|| PolicyError::ReleaseGate(format!("{label} must be a non-negative integer")))
}

fn require_positive_int(val: Option<&Value>, label: &str) -> Result<u64, PolicyError> {
    let n = require_non_neg_int(val, label)?;
    if n == 0 {
        return Err(PolicyError::ReleaseGate(format!("{label} must be > 0")));
    }
    Ok(n)
}

fn require_finite_f64(val: Option<&Value>, label: &str) -> Result<f64, PolicyError> {
    let v = val
        .and_then(Value::as_f64)
        .ok_or_else(|| PolicyError::ReleaseGate(format!("{label} must be a finite number")))?;
    if !v.is_finite() {
        return Err(PolicyError::ReleaseGate(format!(
            "{label} must be a finite number"
        )));
    }
    Ok(v)
}

fn require_bool(val: Option<&Value>, label: &str) -> Result<bool, PolicyError> {
    val.and_then(Value::as_bool)
        .ok_or_else(|| PolicyError::ReleaseGate(format!("{label} must be a boolean")))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::JointLimit;
    use tempfile::tempdir;

    fn test_config(profile_id: Option<&str>) -> ArmConfig {
        let mut cfg = ArmConfig::default();
        cfg.profile_id = profile_id.map(str::to_string);
        cfg.dof = 6;
        cfg.joint_names = (1..=6).map(|i| format!("j{i}")).collect();
        cfg.limits = (1..=6)
            .map(|i| JointLimit::new(&format!("j{i}"), -2.0, 2.0, 2.0, 10.0))
            .collect();
        cfg
    }

    fn test_contract() -> StrictPolicyContract {
        let pj = serde_json::json!({
            "format": "rebot-onnx-policy/v1",
            "name": "rg_unit",
            "unit_mode": "rad",
            "action_dim": 6,
            "action_space": "joint_positions",
            "joint_names": ["j1", "j2", "j3", "j4", "j5", "j6"],
            "model_file": "model.rmlp"
        });
        StrictPolicyContract::parse(&serde_json::to_vec(&pj).unwrap()).unwrap()
    }

    #[test]
    fn canonical_robot_id_matches_alias_table() {
        assert_eq!(canonical_robot_id("b601dm"), Some("b601dm"));
        assert_eq!(canonical_robot_id("rebot_dm"), Some("b601dm"));
        assert_eq!(canonical_robot_id("REBOT-B601-DM"), Some("b601dm"));
        assert_eq!(canonical_robot_id("so-arm101"), Some("so-arm101"));
        assert_eq!(canonical_robot_id("SO_101"), Some("so-arm101"));
        assert_eq!(canonical_robot_id("soarm101"), Some("so-arm101"));
        assert_eq!(canonical_robot_id("unknown-arm"), None);
        assert_eq!(canonical_robot_id(""), None);
    }

    #[test]
    fn parse_and_verify_handles_aliases_and_unrecognized_profiles() {
        let sha = "a".repeat(64);
        let contract = test_contract();
        let gate = serde_json::json!({
            "schema_version": "rebot-release-gate/v1",
            "authorization_tier": "supervised_first_run",
            "policy_sha256": sha.to_ascii_uppercase(),
            "scene_id": "pcb_suction",
            "tool_id": "suction",
            "profile_id": "so101",
            "calibration_digest": "b".repeat(64),
            "runner_config": {
                "fps": 20,
                "max_speed_scale": 0.3,
                "unit_mode": "rad",
                "action_space": "joint_positions"
            },
            "sim_eval": {
                "seeds_total": 10,
                "seeds_passed": 9,
                "min_success_rate": 0.8,
                "collisions": 0,
                "keepout_violations": 0
            },
            "prerequisites": {
                "calibration_verified": true,
                "power_cut_witnessed": true,
                "shadow_run_passed": true
            }
        });

        let deployment = ActiveDeploymentContext {
            scene_id: "pcb_suction".into(),
            tool_id: "suction".into(),
            calibration_digest: "b".repeat(64),
        };
        let ok = ReleaseGate::parse_and_verify(
            &gate,
            &sha,
            &test_config(Some("so_arm101")),
            &contract,
            20.0,
            &deployment,
        )
        .unwrap();
        assert_eq!(ok.policy_sha256(), sha);
        assert_eq!(ok.canonical_profile_id(), "so-arm101");
        assert_eq!(
            EMPTY_CALIBRATION_DIGEST,
            crate::gravity::compute_sha256(b"rebot-calibration:none/v1")
        );

        // Empty / missing config.profile_id must fail closed
        assert!(ReleaseGate::parse_and_verify(
            &gate,
            &sha,
            &test_config(None),
            &contract,
            20.0,
            &deployment,
        )
        .is_err());
        assert!(ReleaseGate::parse_and_verify(
            &gate,
            &sha,
            &test_config(Some("  ")),
            &contract,
            20.0,
            &deployment,
        )
        .is_err());

        // Unrecognized config.profile_id: exact case-insensitive match succeeds, mismatch fails
        let mut custom_gate = gate.clone();
        custom_gate["profile_id"] = serde_json::json!("Custom-Arm-V2");
        assert!(ReleaseGate::parse_and_verify(
            &custom_gate,
            &sha,
            &test_config(Some("custom-arm-v2")),
            &contract,
            20.0,
            &deployment,
        )
        .is_ok());
        assert!(ReleaseGate::parse_and_verify(
            &custom_gate,
            &sha,
            &test_config(Some("custom_arm_v2")),
            &contract,
            20.0,
            &deployment,
        )
        .is_err());
    }

    #[test]
    fn load_release_gate_json_returns_none_when_absent_and_parses_when_present() {
        let dir = tempdir().unwrap();
        assert!(load_release_gate_json(dir.path()).unwrap().is_none());
        fs::write(dir.path().join("release_gate.json"), b"{\"schema_version\":\"rebot-release-gate/v1\"}").unwrap();
        let val = load_release_gate_json(dir.path()).unwrap().unwrap();
        assert_eq!(val["schema_version"], "rebot-release-gate/v1");
    }
}
